// SPDX-License-Identifier: GPL-2.0
/*
 * VirtualBox Guest Shared Folders support: Regular file inode and file ops.
 *
 * Copyright (C) 2006-2016 Oracle Corporation
 */

#include <linux/sizes.h>
#include "vfsmod.h"

/**
 * Read from a regular file.
 * Return: The number of bytes read on success, negative errno value otherwise
 * @file	the file
 * @buf		the buffer
 * @size	length of the buffer
 * @off		offset within the file
 */
static ssize_t sf_reg_read(struct file *file, char *buf, size_t size,
			   loff_t *off)
{
	struct sf_glob_info *sf_g = GET_GLOB_INFO(file_inode(file)->i_sb);
	struct sf_reg_info *sf_r = file->private_data;
	u64 pos = *off;
	u32 nread;
	int err;

	if (!size)
		return 0;

	if (size > SHFL_MAX_RW_COUNT)
		nread = SHFL_MAX_RW_COUNT;
	else
		nread = size;

	err = vboxsf_read(sf_g->root, sf_r->handle, pos, &nread, buf, true);
	if (err)
		return err;

	*off += nread;
	return nread;
}

/**
 * Write to a regular file.
 * Return: The number of bytes written on success, negative errno val otherwise
 * @file	the file
 * @buf		the buffer
 * @size	length of the buffer
 * @off		offset within the file
 */
static ssize_t sf_reg_write(struct file *file, const char *buf, size_t size,
			    loff_t *off)
{
	struct inode *inode = file_inode(file);
	struct sf_inode_info *sf_i = GET_INODE_INFO(inode);
	struct sf_glob_info *sf_g = GET_GLOB_INFO(inode->i_sb);
	struct sf_reg_info *sf_r = file->private_data;
	u32 nwritten;
	u64 pos;
	int err;

	pos = *off;
	if (file->f_flags & O_APPEND) {
		pos = inode->i_size;
		*off = pos;
	}

	if (!size)
		return 0;

	if (size > SHFL_MAX_RW_COUNT)
		nwritten = SHFL_MAX_RW_COUNT;
	else
		nwritten = size;

	/* Make sure any pending writes done through mmap are flushed */
	err = filemap_fdatawait_range(inode->i_mapping, pos, pos + nwritten);
	if (err)
		return err;

	err = vboxsf_write(sf_g->root, sf_r->handle, pos, &nwritten, buf, true);
	if (err)
		return err;

	*off += nwritten;
	if (*off > inode->i_size)
		i_size_write(inode, *off);

	/* Invalidate page-cache so that mmap using apps see the changes too */
	invalidate_mapping_pages(inode->i_mapping, pos >> PAGE_SHIFT,
				 *off >> PAGE_SHIFT);

	/* mtime changed */
	sf_i->force_restat = 1;
	return nwritten;
}

/**
 * Open a regular file.
 * Return: 0 or negative errno value.
 * @inode	inode
 * @file	file
 */
static int sf_reg_open(struct inode *inode, struct file *file)
{
	struct sf_inode_info *sf_i = GET_INODE_INFO(inode);
	struct shfl_createparms params = {};
	struct sf_reg_info *sf_r;
	int err;

	sf_r = kmalloc(sizeof(*sf_r), GFP_KERNEL);
	if (!sf_r)
		return -ENOMEM;

	/* Already open? */
	if (sf_i->handle != SHFL_HANDLE_NIL) {
		sf_r->handle = sf_i->handle;
		sf_i->handle = SHFL_HANDLE_NIL;
		sf_i->file = file;
		file->private_data = sf_r;
		return 0;
	}

	/*
	 * We check the value of params.handle afterwards to find out if
	 * the call succeeded or failed, as the API does not seem to cleanly
	 * distinguish error and informational messages.
	 *
	 * Furthermore, we must set params.handle to SHFL_HANDLE_NIL to
	 * make the shared folders host service use our mode parameter.
	 */
	params.handle = SHFL_HANDLE_NIL;
	if (file->f_flags & O_CREAT) {
		params.create_flags |= SHFL_CF_ACT_CREATE_IF_NEW;
		/*
		 * We ignore O_EXCL, as the Linux kernel seems to call create
		 * beforehand itself, so O_EXCL should always fail.
		 */
		if (file->f_flags & O_TRUNC)
			params.create_flags |= SHFL_CF_ACT_OVERWRITE_IF_EXISTS;
		else
			params.create_flags |= SHFL_CF_ACT_OPEN_IF_EXISTS;
	} else {
		params.create_flags |= SHFL_CF_ACT_FAIL_IF_NEW;
		if (file->f_flags & O_TRUNC)
			params.create_flags |= SHFL_CF_ACT_OVERWRITE_IF_EXISTS;
	}

	switch (file->f_flags & O_ACCMODE) {
	case O_RDONLY:
		params.create_flags |= SHFL_CF_ACCESS_READ;
		break;

	case O_WRONLY:
		params.create_flags |= SHFL_CF_ACCESS_WRITE;
		break;

	case O_RDWR:
		params.create_flags |= SHFL_CF_ACCESS_READWRITE;
		break;

	default:
		WARN_ON(1);
	}

	if (file->f_flags & O_APPEND)
		params.create_flags |= SHFL_CF_ACCESS_APPEND;

	params.info.attr.mode = inode->i_mode;

	err = vboxsf_create_at_dentry(file_dentry(file), &params);
	if (err == 0 && params.handle == SHFL_HANDLE_NIL)
		err = (params.result == SHFL_FILE_EXISTS) ? -EEXIST : -ENOENT;
	if (err) {
		kfree(sf_r);
		return err;
	}

	/* the host may have given us different attr then requested */
	sf_i->force_restat = 1;
	sf_r->handle = params.handle;
	sf_i->file = file;
	file->private_data = sf_r;
	return 0;
}

/**
 * Close a regular file.
 * Return: 0 or negative errno value.
 * @inode	inode
 * @file	file
 */
static int sf_reg_release(struct inode *inode, struct file *file)
{
	struct sf_reg_info *sf_r;
	struct sf_glob_info *sf_g;
	struct sf_inode_info *sf_i = GET_INODE_INFO(inode);

	sf_g = GET_GLOB_INFO(inode->i_sb);
	sf_r = file->private_data;

	filemap_write_and_wait(inode->i_mapping);

	vboxsf_close(sf_g->root, sf_r->handle);

	kfree(sf_r);
	sf_i->file = NULL;
	sf_i->handle = SHFL_HANDLE_NIL;
	file->private_data = NULL;
	return 0;
}

const struct file_operations vboxsf_reg_fops = {
	.read = sf_reg_read,
	.open = sf_reg_open,
	.write = sf_reg_write,
	.release = sf_reg_release,
	.mmap = generic_file_mmap,
	.splice_read = generic_file_splice_read,
	.read_iter = generic_file_read_iter,
	.write_iter = generic_file_write_iter,
	.fsync = noop_fsync,
	.llseek = generic_file_llseek,
};

const struct inode_operations vboxsf_reg_iops = {
	.getattr = vboxsf_getattr,
	.setattr = vboxsf_setattr
};

static int sf_readpage(struct file *file, struct page *page)
{
	struct sf_glob_info *sf_g = GET_GLOB_INFO(file_inode(file)->i_sb);
	struct sf_reg_info *sf_r = file->private_data;
	loff_t off = page_offset(page);
	u32 nread = PAGE_SIZE;
	u8 *buf;
	int err;

	buf = kmap(page);

	err = vboxsf_read(sf_g->root, sf_r->handle, off, &nread, buf, false);
	if (err == 0) {
		memset(&buf[nread], 0, PAGE_SIZE - nread);
		flush_dcache_page(page);
		SetPageUptodate(page);
	} else {
		SetPageError(page);
	}

	kunmap(page);
	unlock_page(page);
	return err;
}

static int sf_writepage(struct page *page, struct writeback_control *wbc)
{
	struct inode *inode = page->mapping->host;
	struct sf_glob_info *sf_g = GET_GLOB_INFO(inode->i_sb);
	struct sf_inode_info *sf_i = GET_INODE_INFO(inode);
	struct sf_reg_info *sf_r = sf_i->file->private_data;
	loff_t off = page_offset(page);
	loff_t size = i_size_read(inode);
	u32 nwrite = PAGE_SIZE;
	u8 *buf;
	int err;

	if (off + PAGE_SIZE > size)
		nwrite = size & ~PAGE_MASK;

	buf = kmap(page);
	err = vboxsf_write(sf_g->root, sf_r->handle, off, &nwrite, buf, false);
	kunmap(page);

	if (err == 0) {
		ClearPageError(page);
		/* mtime changed */
		sf_i->force_restat = 1;
	} else {
		ClearPageUptodate(page);
	}

	unlock_page(page);
	return err;
}

int sf_write_end(struct file *file, struct address_space *mapping, loff_t pos,
		 unsigned int len, unsigned int copied, struct page *page,
		 void *fsdata)
{
	struct inode *inode = mapping->host;
	struct sf_glob_info *sf_g = GET_GLOB_INFO(inode->i_sb);
	struct sf_reg_info *sf_r = file->private_data;
	unsigned int from = pos & ~PAGE_MASK;
	u32 nwritten = len;
	u8 *buf;
	int err;

	buf = kmap(page);
	err = vboxsf_write(sf_g->root, sf_r->handle, pos, &nwritten,
			   buf + from, false);
	kunmap(page);

	if (err)
		goto out;

	/* mtime changed */
	GET_INODE_INFO(inode)->force_restat = 1;

	if (!PageUptodate(page) && nwritten == PAGE_SIZE)
		SetPageUptodate(page);

	pos += nwritten;
	if (pos > inode->i_size)
		i_size_write(inode, pos);

out:
	unlock_page(page);
	put_page(page);

	return err;
}

const struct address_space_operations vboxsf_reg_aops = {
	.readpage = sf_readpage,
	.writepage = sf_writepage,
	.set_page_dirty = __set_page_dirty_nobuffers,
	.write_begin = simple_write_begin,
	.write_end = sf_write_end,
};

static const char *sf_get_link(struct dentry *dentry, struct inode *inode,
			       struct delayed_call *done)
{
	struct sf_glob_info *sf_g = GET_GLOB_INFO(inode->i_sb);
	struct shfl_string *path;
	char *link;
	int err;

	if (!dentry)
		return ERR_PTR(-ECHILD);

	path = vboxsf_path_from_dentry(sf_g, dentry);
	if (IS_ERR(path))
		return (char *)path;

	link = kzalloc(PATH_MAX, GFP_KERNEL);
	if (!link) {
		__putname(path);
		return ERR_PTR(-ENOMEM);
	}

	err = vboxsf_readlink(sf_g->root, path, PATH_MAX, link);
	__putname(path);
	if (err) {
		kfree(link);
		return ERR_PTR(err);
	}

	set_delayed_call(done, kfree_link, link);
	return link;
}

const struct inode_operations vboxsf_lnk_iops = {
	.get_link = sf_get_link
};
