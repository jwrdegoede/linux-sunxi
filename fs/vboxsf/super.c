// SPDX-License-Identifier: MIT
/*
 * VirtualBox Guest Shared Folders support: Virtual File System.
 *
 * Module initialization/finalization
 * File system registration/deregistration
 * Superblock reading
 * Few utility functions
 *
 * Copyright (C) 2006-2018 Oracle Corporation
 */

#include <linux/idr.h>
#include <linux/fs_parser.h>
#include <linux/magic.h>
#include <linux/module.h>
#include <linux/nls.h>
#include <linux/statfs.h>
#include <linux/vbox_utils.h>
#include "vfsmod.h"

#define VBOXSF_SUPER_MAGIC 0x786f4256 /* 'VBox' little endian */

#define VBSF_MOUNT_SIGNATURE_BYTE_0 ('\000')
#define VBSF_MOUNT_SIGNATURE_BYTE_1 ('\377')
#define VBSF_MOUNT_SIGNATURE_BYTE_2 ('\376')
#define VBSF_MOUNT_SIGNATURE_BYTE_3 ('\375')

static int follow_symlinks;
module_param(follow_symlinks, int, 0444);
MODULE_PARM_DESC(follow_symlinks,
		 "Let host resolve symlinks rather than showing them");

static DEFINE_IDA(vboxsf_bdi_ida);
static DEFINE_MUTEX(vboxsf_setup_mutex);
static bool vboxsf_setup_done;
static struct super_operations vboxsf_super_ops; /* forward declaration */
static struct kmem_cache *sf_inode_cachep;

static char * const vboxsf_default_nls = CONFIG_NLS_DEFAULT;

enum  { opt_nls, opt_uid, opt_gid, opt_ttl, opt_dmode, opt_fmode,
	opt_dmask, opt_fmask };

static const struct fs_parameter_spec vboxsf_param_specs[] = {
	fsparam_string	("nls",		opt_nls),
	fsparam_u32	("uid",		opt_uid),
	fsparam_u32	("gid",		opt_gid),
	fsparam_u32	("ttl",		opt_ttl),
	fsparam_u32oct	("dmode",	opt_dmode),
	fsparam_u32oct	("fmode",	opt_fmode),
	fsparam_u32oct	("dmask",	opt_dmask),
	fsparam_u32oct	("fmask",	opt_fmask),
	{}
};

static const struct fs_parameter_description vboxsf_fs_parameters = {
	.name  = "vboxsf",
	.specs  = vboxsf_param_specs,
};

static int vboxsf_parse_param(struct fs_context *fc, struct fs_parameter *param)
{
	struct vboxsf_fs_context *ctx = fc->fs_private;
	struct fs_parse_result result;
	kuid_t uid;
	kgid_t gid;
	int opt;

	opt = fs_parse(fc, &vboxsf_fs_parameters, param, &result);
	if (opt < 0)
		return opt;

	switch (opt) {
	case opt_nls:
		if (fc->purpose != FS_CONTEXT_FOR_MOUNT) {
			vbg_err("vboxsf: Cannot reconfigure nls option\n");
			return -EINVAL;
		}
		ctx->nls_name = param->string;
		param->string = NULL;
		break;
	case opt_uid:
		uid = make_kuid(current_user_ns(), result.uint_32);
		if (!uid_valid(uid))
			return -EINVAL;
		ctx->o.uid = uid;
		break;
	case opt_gid:
		gid = make_kgid(current_user_ns(), result.uint_32);
		if (!gid_valid(gid))
			return -EINVAL;
		ctx->o.gid = gid;
		break;
	case opt_ttl:
		ctx->o.ttl = msecs_to_jiffies(result.uint_32);
		break;
	case opt_dmode:
		if (result.uint_32 & ~0777)
			return -EINVAL;
		ctx->o.dmode = result.uint_32;
		ctx->o.dmode_set = true;
		break;
	case opt_fmode:
		if (result.uint_32 & ~0777)
			return -EINVAL;
		ctx->o.fmode = result.uint_32;
		ctx->o.fmode_set = true;
		break;
	case opt_dmask:
		if (result.uint_32 & ~07777)
			return -EINVAL;
		ctx->o.dmask = result.uint_32;
		break;
	case opt_fmask:
		if (result.uint_32 & ~07777)
			return -EINVAL;
		ctx->o.fmask = result.uint_32;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int vboxsf_fill_super(struct super_block *sb, struct fs_context *fc)
{
	struct vboxsf_fs_context *ctx = fc->fs_private;
	struct shfl_string *folder_name, root_path;
	struct sf_glob_info *sf_g;
	struct dentry *droot;
	struct inode *iroot;
	char *nls_name;
	size_t size;
	int err;

	if (!fc->source)
		return -EINVAL;

	sf_g = kzalloc(sizeof(*sf_g), GFP_KERNEL);
	if (!sf_g)
		return -ENOMEM;

	sf_g->o = ctx->o;
	idr_init(&sf_g->ino_idr);
	spin_lock_init(&sf_g->ino_idr_lock);
	sf_g->next_generation = 1;
	sf_g->bdi_id = -1;

	/* Load nls if not utf8 */
	nls_name = ctx->nls_name ? ctx->nls_name : vboxsf_default_nls;
	if (strcmp(nls_name, "utf8") != 0) {
		if (nls_name == vboxsf_default_nls)
			sf_g->nls = load_nls_default();
		else
			sf_g->nls = load_nls(nls_name);

		if (!sf_g->nls) {
			vbg_err("vboxsf: Count not load '%s' nls\n", nls_name);
			err = -EINVAL;
			goto fail_free;
		}
	}

	sf_g->bdi_id = ida_simple_get(&vboxsf_bdi_ida, 0, 0, GFP_KERNEL);
	if (sf_g->bdi_id < 0) {
		err = sf_g->bdi_id;
		goto fail_free;
	}

	err = super_setup_bdi_name(sb, "vboxsf-%s.%d", fc->source,
				   sf_g->bdi_id);
	if (err)
		goto fail_free;

	/* Turn source into a shfl_string and map the folder */
	size = strlen(fc->source) + 1;
	folder_name = kmalloc(SHFLSTRING_HEADER_SIZE + size, GFP_KERNEL);
	if (!folder_name)
		goto fail_free;
	folder_name->size = size;
	folder_name->length = size - 1;
	strlcpy(folder_name->string.utf8, fc->source, size);
	err = vboxsf_map_folder(folder_name, &sf_g->root);
	kfree(folder_name);
	if (err) {
		vbg_err("vboxsf: Host rejected mount of '%s' with error %d\n",
			fc->source, err);
		goto fail_free;
	}

	root_path.length = 1;
	root_path.size = 2;
	root_path.string.utf8[0] = '/';
	root_path.string.utf8[1] = 0;
	err = vboxsf_stat(sf_g, &root_path, &sf_g->root_info);
	if (err)
		goto fail_unmap;

	sb->s_magic = VBOXSF_SUPER_MAGIC;
	sb->s_blocksize = 1024;
	sb->s_maxbytes = MAX_LFS_FILESIZE;
	sb->s_op = &vboxsf_super_ops;
	sb->s_d_op = &vboxsf_dentry_ops;

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
	if (sf_g->bdi_id >= 0)
		ida_simple_remove(&vboxsf_bdi_ida, sf_g->bdi_id);
	if (sf_g->nls)
		unload_nls(sf_g->nls);
	idr_destroy(&sf_g->ino_idr);
	kfree(sf_g);
	return err;
}

static void sf_inode_init_once(void *data)
{
	struct sf_inode_info *sf_i = (struct sf_inode_info *)data;

	mutex_init(&sf_i->handle_list_mutex);
	inode_init_once(&sf_i->vfs_inode);
}

static struct inode *sf_alloc_inode(struct super_block *sb)
{
	struct sf_inode_info *sf_i;

	sf_i = kmem_cache_alloc(sf_inode_cachep, GFP_NOFS);
	if (!sf_i)
		return NULL;

	sf_i->force_restat = 0;
	INIT_LIST_HEAD(&sf_i->handle_list);

	return &sf_i->vfs_inode;
}

static void sf_i_callback(struct rcu_head *head)
{
	struct inode *inode = container_of(head, struct inode, i_rcu);
	struct sf_glob_info *sf_g = GET_GLOB_INFO(inode->i_sb);

	spin_lock(&sf_g->ino_idr_lock);
	idr_remove(&sf_g->ino_idr, inode->i_ino);
	spin_unlock(&sf_g->ino_idr_lock);
	kmem_cache_free(sf_inode_cachep, GET_INODE_INFO(inode));
}

static void sf_destroy_inode(struct inode *inode)
{
	call_rcu(&inode->i_rcu, sf_i_callback);
}

static void sf_put_super(struct super_block *sb)
{
	struct sf_glob_info *sf_g = GET_GLOB_INFO(sb);

	vboxsf_unmap_folder(sf_g->root);
	if (sf_g->bdi_id >= 0)
		ida_simple_remove(&vboxsf_bdi_ida, sf_g->bdi_id);
	if (sf_g->nls)
		unload_nls(sf_g->nls);
	idr_destroy(&sf_g->ino_idr);
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
	stat->f_ffree = 1000000;
	stat->f_fsid.val[0] = 0;
	stat->f_fsid.val[1] = 0;
	stat->f_namelen = 255;
	return 0;
}

static struct super_operations vboxsf_super_ops = {
	.alloc_inode	= sf_alloc_inode,
	.destroy_inode	= sf_destroy_inode,
	.put_super	= sf_put_super,
	.statfs		= sf_statfs,
};

static int vboxsf_setup(void)
{
	int err;

	mutex_lock(&vboxsf_setup_mutex);

	if (vboxsf_setup_done)
		goto success;

	sf_inode_cachep = kmem_cache_create("vboxsf_inode_cache",
					     sizeof(struct sf_inode_info),
					     0, (SLAB_RECLAIM_ACCOUNT|
						SLAB_MEM_SPREAD|SLAB_ACCOUNT),
					     sf_inode_init_once);
	if (sf_inode_cachep == NULL) {
		err = -ENOMEM;
		goto fail_nomem;
	}

	err = vboxsf_connect();
	if (err) {
		vbg_err("vboxsf: err %d connecting to guest PCI-device\n", err);
		vbg_err("vboxsf: make sure you are inside a VirtualBox VM\n");
		vbg_err("vboxsf: and check dmesg for vboxguest errors\n");
		goto fail_free_cache;
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

	vboxsf_setup_done = true;
success:
	mutex_unlock(&vboxsf_setup_mutex);
	return 0;

fail_disconnect:
	vboxsf_disconnect();
fail_free_cache:
	kmem_cache_destroy(sf_inode_cachep);
fail_nomem:
	mutex_unlock(&vboxsf_setup_mutex);
	return err;
}

int vboxsf_parse_monolithic(struct fs_context *fc, void *data)
{
	char *options = data;

	if (options && options[0] == VBSF_MOUNT_SIGNATURE_BYTE_0 &&
		       options[1] == VBSF_MOUNT_SIGNATURE_BYTE_1 &&
		       options[2] == VBSF_MOUNT_SIGNATURE_BYTE_2 &&
		       options[3] == VBSF_MOUNT_SIGNATURE_BYTE_3) {
		vbg_err("vboxsf: Old binary mount data not supported, remove obsolete mount.vboxsf and/or update your VBoxService.\n");
		return -EINVAL;
	}

	return generic_parse_monolithic(fc, data);
}

static int vboxsf_get_tree(struct fs_context *fc)
{
	int err;

	err = vboxsf_setup();
	if (err)
		return err;

	return vfs_get_super(fc, vfs_get_independent_super, vboxsf_fill_super);
}

static int vboxsf_reconfigure(struct fs_context *fc)
{
	struct sf_glob_info *sf_g = GET_GLOB_INFO(fc->root->d_sb);
	struct vboxsf_fs_context *ctx = fc->fs_private;
	struct inode *iroot;

	iroot = ilookup(fc->root->d_sb, 0);
	if (!iroot)
		return -ENOENT;

	/* Apply changed options to the root inode */
	sf_g->o = ctx->o;
	vboxsf_init_inode(sf_g, iroot, &sf_g->root_info);

	return 0;
}

static void vboxsf_free_fc(struct fs_context *fc)
{
	struct vboxsf_fs_context *ctx = fc->fs_private;

	kfree(ctx->nls_name);
	kfree(ctx);
}

static const struct fs_context_operations vboxsf_context_ops = {
	.free			= vboxsf_free_fc,
	.parse_param		= vboxsf_parse_param,
	.parse_monolithic	= vboxsf_parse_monolithic,
	.get_tree		= vboxsf_get_tree,
	.reconfigure		= vboxsf_reconfigure,
};

static int vboxsf_init_fs_context(struct fs_context *fc)
{
	struct vboxsf_fs_context *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	current_uid_gid(&ctx->o.uid, &ctx->o.gid);

	fc->fs_private = ctx;
	fc->ops = &vboxsf_context_ops;
	return 0;
}

static struct file_system_type vboxsf_fs_type = {
	.owner			= THIS_MODULE,
	.name			= "vboxsf",
	.init_fs_context	= vboxsf_init_fs_context,
	.parameters		= &vboxsf_fs_parameters,
	.kill_sb		= kill_anon_super
};

/* Module initialization/finalization handlers */
static int __init vboxsf_init(void)
{
	return register_filesystem(&vboxsf_fs_type);
}

static void __exit vboxsf_fini(void)
{
	unregister_filesystem(&vboxsf_fs_type);

	mutex_lock(&vboxsf_setup_mutex);
	if (vboxsf_setup_done) {
		vboxsf_disconnect();
		/*
		 * Make sure all delayed rcu free inodes are flushed
		 * before we destroy the cache.
		 */
		rcu_barrier();
		kmem_cache_destroy(sf_inode_cachep);
	}
	mutex_unlock(&vboxsf_setup_mutex);
}

module_init(vboxsf_init);
module_exit(vboxsf_fini);

MODULE_DESCRIPTION("Oracle VM VirtualBox Module for Host File System Access");
MODULE_AUTHOR("Oracle Corporation");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_FS("vboxsf");
