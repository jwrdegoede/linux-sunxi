// SPDX-License-Identifier: GPL-2.0
/*
 * VirtualBox Guest Shared Folders support: Utility functions.
 * Mainly conversion from/to VirtualBox/Linux data structures.
 *
 * Copyright (C) 2006-2016 Oracle Corporation
 */

#include <linux/namei.h>
#include <linux/nls.h>
#include <linux/sizes.h>
#include <linux/vfs.h>
#include "vfsmod.h"

/* set [inode] attributes based on [info], uid/gid based on [sf_g] */
void vboxsf_init_inode(struct sf_glob_info *sf_g, struct inode *inode,
		       const struct shfl_fsobjinfo *info)
{
	const struct shfl_fsobjattr *attr;
	s64 allocated;
	int mode;

	attr = &info->attr;

#define mode_set(r) ((attr->mode & (SHFL_UNIX_##r)) ? (S_##r) : 0)

	mode = mode_set(IRUSR);
	mode |= mode_set(IWUSR);
	mode |= mode_set(IXUSR);

	mode |= mode_set(IRGRP);
	mode |= mode_set(IWGRP);
	mode |= mode_set(IXGRP);

	mode |= mode_set(IROTH);
	mode |= mode_set(IWOTH);
	mode |= mode_set(IXOTH);

#undef mode_set

	/* We use the host-side values for these */
	inode->i_flags |= S_NOATIME | S_NOCMTIME;
	inode->i_mapping->a_ops = &vboxsf_reg_aops;

	if (SHFL_IS_DIRECTORY(attr->mode)) {
		inode->i_mode = sf_g->dmode != ~0 ? (sf_g->dmode & 0777) : mode;
		inode->i_mode &= ~sf_g->dmask;
		inode->i_mode |= S_IFDIR;
		inode->i_op = &vboxsf_dir_iops;
		inode->i_fop = &vboxsf_dir_fops;
		/*
		 * XXX: this probably should be set to the number of entries
		 * in the directory plus two (. ..)
		 */
		set_nlink(inode, 1);
	} else if (SHFL_IS_SYMLINK(attr->mode)) {
		inode->i_mode = sf_g->fmode != ~0 ? (sf_g->fmode & 0777) : mode;
		inode->i_mode &= ~sf_g->fmask;
		inode->i_mode |= S_IFLNK;
		inode->i_op = &vboxsf_lnk_iops;
		set_nlink(inode, 1);
	} else {
		inode->i_mode = sf_g->fmode != ~0 ? (sf_g->fmode & 0777) : mode;
		inode->i_mode &= ~sf_g->fmask;
		inode->i_mode |= S_IFREG;
		inode->i_op = &vboxsf_reg_iops;
		inode->i_fop = &vboxsf_reg_fops;
		set_nlink(inode, 1);
	}

	inode->i_uid = make_kuid(current_user_ns(), sf_g->uid);
	inode->i_gid = make_kgid(current_user_ns(), sf_g->gid);

	inode->i_size = info->size;
	inode->i_blkbits = 12;
	/* i_blocks always in units of 512 bytes! */
	allocated = info->allocated + 511;
	do_div(allocated, 512);
	inode->i_blocks = allocated;

	inode->i_atime = ns_to_timespec64(
				 info->access_time.ns_relative_to_unix_epoch);
	inode->i_ctime = ns_to_timespec64(
				 info->change_time.ns_relative_to_unix_epoch);
	inode->i_mtime = ns_to_timespec64(
			   info->modification_time.ns_relative_to_unix_epoch);
}

int vboxsf_create_at_dentry(struct dentry *dentry,
			    struct shfl_createparms *params)
{
	struct sf_glob_info *sf_g = GET_GLOB_INFO(dentry->d_sb);
	struct shfl_string *path;
	int err;

	path = vboxsf_path_from_dentry(sf_g, dentry);
	if (IS_ERR(path))
		return PTR_ERR(path);

	err = vboxsf_create(sf_g->root, path, params);
	__putname(path);

	return err;
}

int vboxsf_stat(struct sf_glob_info *sf_g, struct shfl_string *path,
		struct shfl_fsobjinfo *info)
{
	struct shfl_createparms params = {};
	int err;

	params.handle = SHFL_HANDLE_NIL;
	params.create_flags = SHFL_CF_LOOKUP | SHFL_CF_ACT_FAIL_IF_NEW;

	err = vboxsf_create(sf_g->root, path, &params);
	if (err)
		return err;

	if (params.result != SHFL_FILE_EXISTS)
		return -ENOENT;

	if (info)
		*info = params.info;

	return 0;
}

int vboxsf_stat_dentry(struct dentry *dentry, struct shfl_fsobjinfo *info)
{
	struct sf_glob_info *sf_g = GET_GLOB_INFO(dentry->d_sb);
	struct shfl_string *path;
	int err;

	path = vboxsf_path_from_dentry(sf_g, dentry);
	if (IS_ERR(path))
		return PTR_ERR(path);

	err = vboxsf_stat(sf_g, path, info);
	__putname(path);
	return err;
}

int vboxsf_inode_revalidate(struct dentry *dentry)
{
	struct sf_glob_info *sf_g = GET_GLOB_INFO(dentry->d_sb);
	struct sf_inode_info *sf_i;
	struct shfl_fsobjinfo info;
	struct timespec64 prev_mtime;
	struct inode *inode;
	int err;

	if (!dentry || !d_really_is_positive(dentry))
		return -EINVAL;

	inode = d_inode(dentry);
	prev_mtime = inode->i_mtime;
	sf_i = GET_INODE_INFO(inode);
	if (!sf_i->force_restat) {
		if (time_before(jiffies, dentry->d_time + sf_g->ttl))
			return 0;
	}

	err = vboxsf_stat_dentry(dentry, &info);
	if (err)
		return err;

	dentry->d_time = jiffies;
	sf_i->force_restat = 0;
	vboxsf_init_inode(sf_g, inode, &info);

	/*
	 * mmap()-ed files use the page-cache, if the file was changed on the
	 * host side we need to invalidate the page-cache for it.  Note this
	 * also gets triggered by our own writes, this is unavoidable.
	 */
	if (timespec64_compare(&inode->i_mtime, &prev_mtime) > 0)
		invalidate_mapping_pages(inode->i_mapping, 0, -1);

	return 0;
}

int vboxsf_getattr(const struct path *path, struct kstat *kstat,
		   u32 request_mask, unsigned int flags)
{
	int err;
	struct dentry *dentry = path->dentry;

	err = vboxsf_inode_revalidate(dentry);
	if (err)
		return err;

	generic_fillattr(d_inode(dentry), kstat);
	return 0;
}

int vboxsf_setattr(struct dentry *dentry, struct iattr *iattr)
{
	struct sf_inode_info *sf_i = GET_INODE_INFO(d_inode(dentry));
	struct sf_glob_info *sf_g = GET_GLOB_INFO(dentry->d_sb);
	struct shfl_createparms params = {};
	struct shfl_fsobjinfo info = {};
	uint32_t buf_len;
	int err;

	params.handle = SHFL_HANDLE_NIL;
	params.create_flags = SHFL_CF_ACT_OPEN_IF_EXISTS |
			      SHFL_CF_ACT_FAIL_IF_NEW |
			      SHFL_CF_ACCESS_ATTR_WRITE;

	/* this is at least required for Posix hosts */
	if (iattr->ia_valid & ATTR_SIZE)
		params.create_flags |= SHFL_CF_ACCESS_WRITE;

	err = vboxsf_create_at_dentry(dentry, &params);
	if (err || params.result != SHFL_FILE_EXISTS)
		return err ? err : -ENOENT;

#define mode_set(r) ((iattr->ia_mode & (S_##r)) ? SHFL_UNIX_##r : 0)

	/*
	 * Setting the file size and setting the other attributes has to
	 * be handled separately.
	 */
	if (iattr->ia_valid & (ATTR_MODE | ATTR_ATIME | ATTR_MTIME)) {
		if (iattr->ia_valid & ATTR_MODE) {
			info.attr.mode = mode_set(IRUSR);
			info.attr.mode |= mode_set(IWUSR);
			info.attr.mode |= mode_set(IXUSR);
			info.attr.mode |= mode_set(IRGRP);
			info.attr.mode |= mode_set(IWGRP);
			info.attr.mode |= mode_set(IXGRP);
			info.attr.mode |= mode_set(IROTH);
			info.attr.mode |= mode_set(IWOTH);
			info.attr.mode |= mode_set(IXOTH);

			if (iattr->ia_mode & S_IFDIR)
				info.attr.mode |= SHFL_TYPE_DIRECTORY;
			else
				info.attr.mode |= SHFL_TYPE_FILE;
		}

		if (iattr->ia_valid & ATTR_ATIME)
			info.access_time.ns_relative_to_unix_epoch =
					    timespec64_to_ns(&iattr->ia_atime);

		if (iattr->ia_valid & ATTR_MTIME)
			info.modification_time.ns_relative_to_unix_epoch =
					    timespec64_to_ns(&iattr->ia_mtime);

		/*
		 * Ignore ctime (inode change time) as it can't be set
		 * from userland anyway.
		 */

		buf_len = sizeof(info);
		err = vboxsf_fsinfo(sf_g->root, params.handle,
				   SHFL_INFO_SET | SHFL_INFO_FILE, &buf_len,
				   &info);
		if (err) {
			vboxsf_close(sf_g->root, params.handle);
			return err;
		}

		/* the host may have given us different attr then requested */
		sf_i->force_restat = 1;
	}

#undef mode_set

	if (iattr->ia_valid & ATTR_SIZE) {
		memset(&info, 0, sizeof(info));
		info.size = iattr->ia_size;
		buf_len = sizeof(info);
		err = vboxsf_fsinfo(sf_g->root, params.handle,
				   SHFL_INFO_SET | SHFL_INFO_SIZE, &buf_len,
				   &info);
		if (err) {
			vboxsf_close(sf_g->root, params.handle);
			return err;
		}

		/* the host may have given us different attr then requested */
		sf_i->force_restat = 1;
	}

	vboxsf_close(sf_g->root, params.handle);

	/* Update the inode with what the host has actually given us. */
	if (sf_i->force_restat)
		vboxsf_inode_revalidate(dentry);

	return 0;
}

/*
 * [dentry] contains string encoded in coding system that corresponds
 * to [sf_g]->nls, we must convert it to UTF8 here.
 * Returns a shfl_string allocated through __getname (must be freed using
 * __putname), or an ERR_PTR on error.
 */
struct shfl_string *vboxsf_path_from_dentry(struct sf_glob_info *sf_g,
					    struct dentry *dentry)
{
	struct shfl_string *shfl_path;
	int path_len, out_len, nb;
	char *buf, *path;
	wchar_t uni;
	u8 *out;

	buf = __getname();
	if (!buf)
		return ERR_PTR(-ENOMEM);

	path = dentry_path_raw(dentry, buf, PATH_MAX);
	if (IS_ERR(path)) {
		__putname(buf);
		return (struct shfl_string *)path;
	}
	path_len = strlen(path);

	if (sf_g->nls) {
		shfl_path = __getname();
		if (!shfl_path) {
			__putname(buf);
			return ERR_PTR(-ENOMEM);
		}

		out = shfl_path->string.utf8;
		out_len = PATH_MAX - SHFLSTRING_HEADER_SIZE - 1;

		while (path_len) {
			nb = sf_g->nls->char2uni(path, path_len, &uni);
			if (nb < 0) {
				__putname(shfl_path);
				__putname(buf);
				return ERR_PTR(-EINVAL);
			}
			path += nb;
			path_len -= nb;

			nb = utf32_to_utf8(uni, out, out_len);
			if (nb < 0) {
				__putname(shfl_path);
				__putname(buf);
				return ERR_PTR(-ENAMETOOLONG);
			}
			out += nb;
			out_len -= nb;
		}
		*out = 0;
		shfl_path->length = out - shfl_path->string.utf8;
		shfl_path->size = shfl_path->length + 1;
		__putname(buf);
	} else {
		if ((SHFLSTRING_HEADER_SIZE + path_len + 1) > PATH_MAX) {
			__putname(buf);
			return ERR_PTR(-ENAMETOOLONG);
		}
		/*
		 * dentry_path stores the name at the end of buf, but the
		 * shfl_string string we return must be properly aligned.
		 */
		shfl_path = (struct shfl_string *)buf;
		memmove(shfl_path->string.utf8, path, path_len);
		shfl_path->string.utf8[path_len] = 0;
		shfl_path->length = path_len;
		shfl_path->size = path_len + 1;
	}

	return shfl_path;
}

int vboxsf_nlscpy(struct sf_glob_info *sf_g, char *name, size_t name_bound_len,
		  const unsigned char *utf8_name, size_t utf8_len)
{
	if (sf_g->nls) {
		const char *in;
		char *out;
		size_t out_len;
		size_t out_bound_len;
		size_t in_bound_len;

		in = utf8_name;
		in_bound_len = utf8_len;

		out = name;
		out_len = 0;
		out_bound_len = name_bound_len;

		while (in_bound_len) {
			int nb;
			unicode_t uni;

			nb = utf8_to_utf32(in, in_bound_len, &uni);
			if (nb < 0)
				return -EINVAL;

			in += nb;
			in_bound_len -= nb;

			nb = sf_g->nls->uni2char(uni, out, out_bound_len);
			if (nb < 0)
				return nb;

			out += nb;
			out_bound_len -= nb;
			out_len += nb;
		}

		*out = 0;
	} else {
		if (utf8_len + 1 > name_bound_len)
			return -ENAMETOOLONG;

		memcpy(name, utf8_name, utf8_len + 1);
	}
	return 0;
}

static struct sf_dir_buf *sf_dir_buf_alloc(struct list_head *list)
{
	struct sf_dir_buf *b;

	b = kmalloc(sizeof(*b), GFP_KERNEL);
	if (!b)
		return NULL;

	b->buf = kmalloc(DIR_BUFFER_SIZE, GFP_KERNEL);
	if (!b->buf) {
		kfree(b);
		return NULL;
	}

	b->entries = 0;
	b->used = 0;
	b->free = DIR_BUFFER_SIZE;
	list_add(&b->head, list);

	return b;
}

static void sf_dir_buf_free(struct sf_dir_buf *b)
{
	list_del(&b->head);
	kfree(b->buf);
	kfree(b);
}

/**
 * Create a new directory buffer descriptor.
 * Return: Created sf_dir_info buffer, or NULL when malloc fails
 */
struct sf_dir_info *vboxsf_dir_info_alloc(void)
{
	struct sf_dir_info *p;

	p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;

	INIT_LIST_HEAD(&p->info_list);
	return p;
}

/**
 * Free the directory buffer.
 * @p		sf_dir_info buffer to free
 */
void vboxsf_dir_info_free(struct sf_dir_info *p)
{
	struct list_head *list, *pos, *tmp;

	list = &p->info_list;
	list_for_each_safe(pos, tmp, list) {
		struct sf_dir_buf *b;

		b = list_entry(pos, struct sf_dir_buf, head);
		sf_dir_buf_free(b);
	}
	kfree(p);
}

int vboxsf_dir_read_all(struct sf_glob_info *sf_g, struct sf_dir_info *sf_d,
			u64 handle)
{
	struct sf_dir_buf *b;
	u32 entries, size;
	int err = 0;
	void *buf;

	/* vboxsf_dirinfo returns 1 on end of dir */
	while (err == 0) {
		b = sf_dir_buf_alloc(&sf_d->info_list);
		if (!b) {
			err = -ENOMEM;
			break;
		}

		buf = b->buf;
		size = b->free;

		err = vboxsf_dirinfo(sf_g->root, handle, NULL, 0, 0,
				     &size, buf, &entries);
		if (err < 0)
			break;

		b->entries += entries;
		b->free -= size;
		b->used += size;
	}

	if (b && b->used == 0)
		sf_dir_buf_free(b);

	/* -EILSEQ means the host could not translate a filename, ignore */
	if (err > 0 || err == -EILSEQ)
		err = 0;

	return err;
}
