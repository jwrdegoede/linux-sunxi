// SPDX-License-Identifier: GPL-2.0
/*
 * VirtualBox Guest Shared Folders support: Virtual File System.
 *
 * Module initialization/finalization
 * File system registration/deregistration
 * Superblock reading
 * Few utility functions
 *
 * Copyright (C) 2006-2016 Oracle Corporation
 */

#include <linux/magic.h>
#include <linux/module.h>
#include <linux/nls.h>
#include <linux/parser.h>
#include <linux/statfs.h>
#include <linux/vbox_utils.h>
#include "vfsmod.h"

#define VBOXSF_SUPER_MAGIC 0xface

#define VBSF_MOUNT_SIGNATURE_BYTE_0 ('\000')
#define VBSF_MOUNT_SIGNATURE_BYTE_1 ('\377')
#define VBSF_MOUNT_SIGNATURE_BYTE_2 ('\376')
#define VBSF_MOUNT_SIGNATURE_BYTE_3 ('\375')

static int follow_symlinks;
module_param(follow_symlinks, int, 0444);
MODULE_PARM_DESC(follow_symlinks,
		 "Let host resolve symlinks rather than showing them");

struct fill_super_args {
	const char *dev_name;
	char *options;
};

static struct super_operations sf_super_ops; /* forward declaration */
static struct kmem_cache *sf_inode_cachep;

static char * const vboxsf_default_nls = CONFIG_NLS_DEFAULT;

enum  { opt_name, opt_nls, opt_uid, opt_gid, opt_ttl, opt_dmode, opt_fmode,
	opt_dmask, opt_fmask, opt_error };

static const match_table_t vboxsf_tokens = {
	{ opt_nls, "nls=%s" },
	{ opt_uid, "uid=%u" },
	{ opt_gid, "gid=%u" },
	{ opt_ttl, "ttl=%u" },
	{ opt_dmode, "dmode=%o" },
	{ opt_fmode, "fmode=%o" },
	{ opt_dmask, "dmask=%o" },
	{ opt_fmask, "fmask=%o" },
	{ opt_error, NULL },
};

static int vboxsf_parse_options(struct sf_glob_info *sf_g, char *options)
{
	substring_t args[MAX_OPT_ARGS];
	int value, token;
	char *p;

	if (!options)
		goto out;

	if (options[0] == VBSF_MOUNT_SIGNATURE_BYTE_0 &&
	    options[1] == VBSF_MOUNT_SIGNATURE_BYTE_1 &&
	    options[2] == VBSF_MOUNT_SIGNATURE_BYTE_2 &&
	    options[3] == VBSF_MOUNT_SIGNATURE_BYTE_3) {
		vbg_err("vboxsf: Old binary mount data not supported, remove obsolete mount.vboxsf and/or update your VBoxService.\n");
		return -EINVAL;
	}

	while ((p = strsep(&options, ",")) != NULL) {
		if (!*p)
			continue;

		token = match_token(p, vboxsf_tokens, args);
		switch (token) {
		case opt_nls:
			if (sf_g->nls_name) {
				vbg_err("vboxsf: Cannot change nls option\n");
				return -EINVAL;
			}
			sf_g->nls_name = match_strdup(&args[0]);
			if (!sf_g->nls_name)
				return -ENOMEM;
			break;
		case opt_uid:
			if (match_int(&args[0], &value))
				return -EINVAL;
			sf_g->uid = value;
			break;
		case opt_gid:
			if (match_int(&args[0], &value))
				return -EINVAL;
			sf_g->gid = value;
			break;
		case opt_ttl:
			if (match_int(&args[0], &value))
				return -EINVAL;
			sf_g->ttl = msecs_to_jiffies(value);
			break;
		case opt_dmode:
			if (match_octal(&args[0], &value))
				return -EINVAL;
			sf_g->dmode = value;
			break;
		case opt_fmode:
			if (match_octal(&args[0], &value))
				return -EINVAL;
			sf_g->fmode = value;
			break;
		case opt_dmask:
			if (match_octal(&args[0], &value))
				return -EINVAL;
			sf_g->dmask = value;
			break;
		case opt_fmask:
			if (match_octal(&args[0], &value))
				return -EINVAL;
			sf_g->fmask = value;
			break;
		default:
			vbg_err("vboxsf: Unrecognized mount option \"%s\" or missing value\n",
				p);
			return -EINVAL;
		}
	}

out:
	if (!sf_g->nls_name)
		sf_g->nls_name = vboxsf_default_nls;

	return 0;
}

/*
 * Called when vfs mounts the fs, should respect [flags],
 * initializes [sb], initializes root inode and dentry.
 */
static int sf_fill_super(struct super_block *sb, void *data, int flags)
{
	struct fill_super_args *args = data;
	struct shfl_string root_path;
	struct sf_glob_info *sf_g;
	struct dentry *droot;
	struct inode *iroot;
	size_t size;
	int err;

	if (flags & MS_REMOUNT)
		return -EINVAL;

	sf_g = kzalloc(sizeof(*sf_g), GFP_KERNEL);
	if (!sf_g)
		return -ENOMEM;

	/* Turn dev_name into a shfl_string */
	size = strlen(args->dev_name) + 1;
	sf_g->name = kmalloc(SHFLSTRING_HEADER_SIZE + size, GFP_KERNEL);
	if (!sf_g->name)
		return -ENOMEM;
	sf_g->name->size = size;
	sf_g->name->length = size - 1;
	strlcpy(sf_g->name->string.utf8, args->dev_name, size);

	/* ~0 means use whatever the host gives as mode info */
	sf_g->dmode = ~0;
	sf_g->fmode = ~0;

	err = vboxsf_parse_options(sf_g, args->options);
	if (err)
		goto fail_free;

	/* Load nls if not utf8 */
	if (strcmp(sf_g->nls_name, "utf8") != 0) {
		if (sf_g->nls_name == vboxsf_default_nls)
			sf_g->nls = load_nls_default();
		else
			sf_g->nls = load_nls(sf_g->nls_name);

		if (!sf_g->nls) {
			err = -EINVAL;
			goto fail_free;
		}
	}

	err = super_setup_bdi_name(sb, "vboxsf-%s", args->dev_name);
	if (err)
		goto fail_free;

	err = vboxsf_map_folder(sf_g->name, &sf_g->root);
	if (err)
		goto fail_free;

	root_path.length = 1;
	root_path.size = 2;
	strlcpy(root_path.string.utf8, "/",
		sizeof(root_path) - SHFLSTRING_HEADER_SIZE);
	err = vboxsf_stat(sf_g, &root_path, &sf_g->root_info);
	if (err)
		goto fail_unmap;

	sb->s_magic = VBOXSF_SUPER_MAGIC;
	sb->s_blocksize = 1024;
	sb->s_maxbytes = MAX_LFS_FILESIZE;
	sb->s_op = &sf_super_ops;

	iroot = iget_locked(sb, 0);
	if (!iroot) {
		err = -ENOMEM;
		goto fail_unmap;
	}
	vboxsf_init_inode(sf_g, iroot, &sf_g->root_info);
	unlock_new_inode(iroot);

	droot = d_make_root(iroot);
	if (!droot) {
		err = -ENOMEM;
		goto fail_unmap;
	}

	sb->s_root = droot;
	SET_GLOB_INFO(sb, sf_g);
	return 0;

fail_unmap:
	vboxsf_unmap_folder(sf_g->root);
fail_free:
	if (sf_g->nls)
		unload_nls(sf_g->nls);
	if (sf_g->nls_name != vboxsf_default_nls)
		kfree(sf_g->nls_name);
	kfree(sf_g->name);
	kfree(sf_g);
	return err;
}

static void sf_inode_init_once(void *data)
{
	struct sf_inode_info *sf_i = (struct sf_inode_info *)data;

	inode_init_once(&sf_i->vfs_inode);
}

static struct inode *sf_alloc_inode(struct super_block *sb)
{
	struct sf_inode_info *sf_i;

	sf_i = kmem_cache_alloc(sf_inode_cachep, GFP_NOFS);
	if (!sf_i)
		return NULL;

	sf_i->force_restat = 0;
	sf_i->file = NULL;
	sf_i->handle = SHFL_HANDLE_NIL;

	return &sf_i->vfs_inode;
}

static void sf_i_callback(struct rcu_head *head)
{
	struct inode *inode = container_of(head, struct inode, i_rcu);

	kmem_cache_free(sf_inode_cachep, GET_INODE_INFO(inode));
}

static void sf_destroy_inode(struct inode *inode)
{
	call_rcu(&inode->i_rcu, sf_i_callback);
}

/*
 * vfs is done with [sb] (umount called) call [sf_glob_free] to unmap
 * the folder and free [sf_g]
 */
static void sf_put_super(struct super_block *sb)
{
	struct sf_glob_info *sf_g = GET_GLOB_INFO(sb);

	generic_shutdown_super(sb);
	vboxsf_unmap_folder(sf_g->root);
	if (sf_g->nls)
		unload_nls(sf_g->nls);
	if (sf_g->nls_name != vboxsf_default_nls)
		kfree(sf_g->nls_name);
	kfree(sf_g->name);
	kfree(sf_g);
}

static int sf_statfs(struct dentry *dentry, struct kstatfs *stat)
{
	struct super_block *sb = dentry->d_sb;
	struct shfl_volinfo SHFLVolumeInfo;
	struct sf_glob_info *sf_g;
	u32 buf_len;
	int err;

	sf_g = GET_GLOB_INFO(sb);
	buf_len = sizeof(SHFLVolumeInfo);
	err = vboxsf_fsinfo(sf_g->root, 0, SHFL_INFO_GET | SHFL_INFO_VOLUME,
			    &buf_len, &SHFLVolumeInfo);
	if (err)
		return err;

	stat->f_type = VBOXSF_SUPER_MAGIC;
	stat->f_bsize = SHFLVolumeInfo.bytes_per_allocation_unit;

	do_div(SHFLVolumeInfo.total_allocation_bytes,
	       SHFLVolumeInfo.bytes_per_allocation_unit);
	stat->f_blocks = SHFLVolumeInfo.total_allocation_bytes;

	do_div(SHFLVolumeInfo.available_allocation_bytes,
	       SHFLVolumeInfo.bytes_per_allocation_unit);
	stat->f_bfree  = SHFLVolumeInfo.available_allocation_bytes;
	stat->f_bavail = SHFLVolumeInfo.available_allocation_bytes;

	stat->f_files = 1000;
	/*
	 * Don't return 0 here since the guest may then think that it is not
	 * possible to create any more files.
	 */
	stat->f_ffree = 1000;
	stat->f_fsid.val[0] = 0;
	stat->f_fsid.val[1] = 0;
	stat->f_namelen = 255;
	return 0;
}

static int sf_remount_fs(struct super_block *sb, int *flags, char *options)
{
	struct sf_glob_info *sf_g = GET_GLOB_INFO(sb);
	struct inode *iroot;
	int err;

	err = vboxsf_parse_options(sf_g, options);
	if (err)
		return err;

	iroot = ilookup(sb, 0);
	if (!iroot)
		return -ENOENT;

	/* Apply changed options to the root inode */
	vboxsf_init_inode(sf_g, iroot, &sf_g->root_info);

	return 0;
}

static struct super_operations sf_super_ops = {
	.alloc_inode	= sf_alloc_inode,
	.destroy_inode	= sf_destroy_inode,
	.put_super	= sf_put_super,
	.statfs		= sf_statfs,
	.remount_fs	= sf_remount_fs
};

static struct dentry *sf_mount(struct file_system_type *fs_type, int flags,
			       const char *dev_name, void *data)
{
	struct fill_super_args args = {
		.dev_name = dev_name,
		.options = data,
	};

	return mount_nodev(fs_type, flags, &args, sf_fill_super);
}

static struct file_system_type vboxsf_fs_type = {
	.owner = THIS_MODULE,
	.name = "vboxsf",
	.mount = sf_mount,
	.kill_sb = kill_anon_super
};

/* Module initialization/finalization handlers */
static int __init init(void)
{
	int err;

	sf_inode_cachep = kmem_cache_create("vboxsf_inode_cache",
					     sizeof(struct sf_inode_info),
					     0, (SLAB_RECLAIM_ACCOUNT|
						SLAB_MEM_SPREAD|SLAB_ACCOUNT),
					     sf_inode_init_once);
	if (sf_inode_cachep == NULL)
		return -ENOMEM;

	err = register_filesystem(&vboxsf_fs_type);
	if (err)
		return err;

	err = vboxsf_connect();
	if (err) {
		vbg_err("vboxsf: err %d connecting to guest PCI-device\n", err);
		vbg_err("vboxsf: make sure you are inside a VirtualBox VM\n");
		vbg_err("vboxsf: and check dmesg for vboxguest errors\n");
		goto fail_unregisterfs;
	}

	err = vboxsf_set_utf8();
	if (err) {
		vbg_err("vboxsf_setutf8 error %d\n", err);
		goto fail_disconnect;
	}

	if (!follow_symlinks) {
		err = vboxsf_set_symlinks();
		if (err)
			vbg_warn("vboxsf: Unable to show symlinks: %d\n", err);
	}

	return 0;

fail_disconnect:
	vboxsf_disconnect();
fail_unregisterfs:
	unregister_filesystem(&vboxsf_fs_type);
	return err;
}

static void __exit fini(void)
{
	vboxsf_disconnect();
	unregister_filesystem(&vboxsf_fs_type);
	/*
	 * Make sure all delayed rcu free inodes are flushed before we
	 * destroy cache.
	 */
	rcu_barrier();
	kmem_cache_destroy(sf_inode_cachep);
}

module_init(init);
module_exit(fini);

MODULE_DESCRIPTION("Oracle VM VirtualBox Module for Host File System Access");
MODULE_AUTHOR("Oracle Corporation");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_FS("vboxsf");
