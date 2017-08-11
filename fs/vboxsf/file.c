// SPDX-License-Identifier: MIT
/*
 * VirtualBox Guest Shared Folders support: Regular file inode and file ops.
 *
 * Copyright (C) 2006-2018 Oracle Corporation
 */

#include <linux/mm.h>
#include <linux/page-flags.h>
#include <linux/pagemap.h>
#include <linux/highmem.h>
#include <linux/sizes.h>
#include "vfsmod.h"

struct sf_handle {
	u64 handle;
	u32 root;
	u32 access_flags;
	struct kref refcount;
	struct list_head head;
};

/**
 * Read from a regular file.
 * Return: The number of bytes read on success, negative errno value otherwise
 * @file	the file
 * @buf		the buffer
 * @size	length of the buffer
 * @off		offset within the file
 */
static ssize_t sf_reg_read(struct file *file, char __user *buf, size_t size,
			   loff_t *off)
{
	struct sf_handle *sf_handle = file->private_data;
	u64 pos = *off;
	u32 nread;
	int err;

	if (!size)
		return 0;

	if (size > SHFL_MAX_RW_COUNT)
		nread = SHFL_MAX_RW_COUNT;
	else
		nread = size;

	err = vboxsf_read(sf_handle->root, sf_handle->handle, pos, &nread,
			  (uintptr_t)buf, true);
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
static ssize_t sf_reg_write(struct file *file, const char __user *buf,
			    size_t size, loff_t *off)
{
	struct inode *inode = file_inode(file);
	struct sf_inode_info *sf_i = GET_INODE_INFO(inode);
	struct sf_handle *sf_handle = file->private_data;
	u32 nwritten;
	u64 pos;
	int err;

	if (file->f_flags & O_APPEND)
		pos = i_size_read(inode);
	else
		pos = *off;

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

	err = vboxsf_write(sf_handle->root, sf_handle->handle, pos, &nwritten,
			   (uintptr_t)buf, true);
	if (err)
		return err;

	if (pos + nwritten > i_size_read(inode))
		i_size_write(inode, pos + nwritten);

	/* Invalidate page-cache so that mmap using apps see the changes too */
	invalidate_mapping_pages(inode->i_mapping, pos >> PAGE_SHIFT,
				 (pos + nwritten) >> PAGE_SHIFT);

	/* mtime changed */
	sf_i->force_restat = 1;

	*off = pos + nwritten;
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
	struct sf_handle *sf_handle;
	u32 access_flags = 0;
	int err;

	sf_handle = kmalloc(sizeof(*sf_handle), GFP_KERNEL);
	if (!sf_handle)
		return -ENOMEM;

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
		access_flags |= SHFL_CF_ACCESS_READ;
		break;

	case O_WRONLY:
		access_flags |= SHFL_CF_ACCESS_WRITE;
		break;

	case O_RDWR:
		access_flags |= SHFL_CF_ACCESS_READWRITE;
		break;

	default:
		WARN_ON(1);
	}

	if (file->f_flags & O_APPEND)
		access_flags |= SHFL_CF_ACCESS_APPEND;

	params.create_flags |= access_flags;
	params.info.attr.mode = inode->i_mode;

	err = vboxsf_create_at_dentry(file_dentry(file), &params);
	if (err == 0 && params.handle == SHFL_HANDLE_NIL)
		err = (params.result == SHFL_FILE_EXISTS) ? -EEXIST : -ENOENT;
	if (err) {
		kfree(sf_handle);
		return err;
	}

	/* the host may have given us different attr then requested */
	sf_i->force_restat = 1;

	/* init our handle struct and add it to the inode's handles list */
	sf_handle->handle = params.handle;
	sf_handle->root = GET_GLOB_INFO(inode->i_sb)->root;
	sf_handle->access_flags = access_flags;
	kref_init(&sf_handle->refcount);

	mutex_lock(&sf_i->handle_list_mutex);
	list_add(&sf_handle->head, &sf_i->handle_list);
	mutex_unlock(&sf_i->handle_list_mutex);

	file->private_data = sf_handle;
	return 0;
}

static void sf_handle_release(struct kref *refcount)
{
	struct sf_handle *sf_handle = container_of(refcount, struct sf_handle,
						   refcount);

	vboxsf_close(sf_handle->root, sf_handle->handle);
	kfree(sf_handle);
}

/**
 * Close a regular file.
 * Return: 0 or negative errno value.
 * @inode	inode
 * @file	file
 */
static int sf_reg_release(struct inode *inode, struct file *file)
{
	struct sf_inode_info *sf_i = GET_INODE_INFO(inode);
	struct sf_handle *sf_handle = file->private_data;

	filemap_write_and_wait(inode->i_mapping);

	mutex_lock(&sf_i->handle_list_mutex);
	list_del(&sf_handle->head);
	mutex_unlock(&sf_i->handle_list_mutex);

	kref_put(&sf_handle->refcount, sf_handle_release);
	file->private_data = NULL;
	return 0;
}

/*
 * Write back dirty pages now, because there may not be any suitable
 * open files later
 */
static void sf_vma_close(struct vm_area_struct *vma)
{
	filemap_write_and_wait(vma->vm_file->f_mapping);
}

static vm_fault_t sf_page_mkwrite(struct vm_fault *vmf)
{
	struct page *page = vmf->page;
	struct inode *inode = file_inode(vmf->vma->vm_file);

	lock_page(page);
	if (page->mapping != inode->i_mapping) {
		unlock_page(page);
		return VM_FAULT_NOPAGE;
	}

	return VM_FAULT_LOCKED;
}

static const struct vm_operations_struct sf_file_vm_ops = {
	.close		= sf_vma_close,
	.fault		= filemap_fault,
	.map_pages	= filemap_map_pages,
	.page_mkwrite	= sf_page_mkwrite,
};

static int sf_reg_mmap(struct file *file, struct vm_area_struct *vma)
{
	int err;

	err = generic_file_mmap(file, vma);
	if (!err)
		vma->vm_ops = &sf_file_vm_ops;

	return err;
}

const struct file_operations vboxsf_reg_fops = {
	.read = sf_reg_read,
	.open = sf_reg_open,
	.write = sf_reg_write,
	.release = sf_reg_release,
	.mmap = sf_reg_mmap,
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
	struct sf_handle *sf_handle = file->private_data;
	loff_t off = page_offset(page);
	u32 nread = PAGE_SIZE;
	u8 *buf;
	int err;

	buf = kmap(page);

	err = vboxsf_read(sf_handle->root, sf_handle->handle, off, &nread,
			  (uintptr_t)buf, false);
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

static struct sf_handle *sf_get_writeable_handle(struct sf_inode_info *sf_i)
{
	struct sf_handle *h, *sf_handle = NULL;

	mutex_lock(&sf_i->handle_list_mutex);
	list_for_each_entry(h, &sf_i->handle_list, head) {
		if (h->access_flags == SHFL_CF_ACCESS_WRITE ||
		    h->access_flags == SHFL_CF_ACCESS_READWRITE) {
			kref_get(&h->refcount);
			sf_handle = h;
			break;
		}
	}
	mutex_unlock(&sf_i->handle_list_mutex);

	return sf_handle;
}

static int sf_writepage(struct page *page, struct writeback_control *wbc)
{
	struct inode *inode = page->mapping->host;
	struct sf_inode_info *sf_i = GET_INODE_INFO(inode);
	struct sf_handle *sf_handle;
	loff_t off = page_offset(page);
	loff_t size = i_size_read(inode);
	u32 nwrite = PAGE_SIZE;
	u8 *buf;
	int err;

	if (off + PAGE_SIZE > size)
		nwrite = size & ~PAGE_MASK;

	sf_handle = sf_get_writeable_handle(sf_i);
	if (!sf_handle)
		return -EBADF;

	buf = kmap(page);
	err = vboxsf_write(sf_handle->root, sf_handle->handle, off, &nwrite,
			   (uintptr_t)buf, false);
	kunmap(page);

	kref_put(&sf_handle->refcount, sf_handle_release);

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

static int sf_write_end(struct file *file, struct address_space *mapping,
			loff_t pos, unsigned int len, unsigned int copied,
			struct page *page, void *fsdata)
{
	struct inode *inode = mapping->host;
	struct sf_handle *sf_handle = file->private_data;
	unsigned int from = pos & ~PAGE_MASK;
	u32 nwritten = len;
	u8 *buf;
	int err;

	buf = kmap(page);
	err = vboxsf_write(sf_handle->root, sf_handle->handle, pos, &nwritten,
			   (uintptr_t)buf + from, false);
	kunmap(page);

	if (err) {
		nwritten = 0;
		goto out;
	}

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

	return nwritten;
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
