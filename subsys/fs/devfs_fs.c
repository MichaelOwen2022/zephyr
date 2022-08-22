/*
 * Copyright (c) 2022 tangchunhui@coros.com
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>

#include "devfs.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(devfs_fs);

/* Memory pool for FatFs file objects */
K_MEM_SLAB_DEFINE(devfs_filep_pool, sizeof(struct devfs_file_t), CONFIG_FS_DEVFS_NUM_FILES, 4);

/* Memory pool for FatFs directory objects */
K_MEM_SLAB_DEFINE(devfs_dirp_pool, sizeof(struct devfs_dir_t), CONFIG_FS_DEVFS_NUM_DIRS, 4);

static int devfs_fs_open(struct fs_file_t *fp, const char *path, fs_mode_t zflags)
{
	void *filep = NULL;
	int retval = 0;

	if (k_mem_slab_alloc(&devfs_filep_pool, &filep, K_NO_WAIT) == 0) {
		(void)memset(filep, 0, sizeof(struct devfs_file_t));
		fp->filep = filep;
	} else {
		return -ENOMEM;
	}

	retval = devfs_open(fp->filep, path, zflags);
	if (retval < 0) {
		k_mem_slab_free(&devfs_filep_pool, &filep);
		fp->filep = NULL;
	}

	return retval;
}

static ssize_t devfs_fs_read(struct fs_file_t *fp, void *ptr, size_t len)
{
	return devfs_read(fp->filep, ptr, len);
}

static ssize_t devfs_fs_write(struct fs_file_t *fp, const void *ptr, size_t len)
{
	return devfs_write(fp->filep, ptr, len);
}

static int devfs_fs_lseek(struct fs_file_t *fp, off_t off, int whence)
{
	return devfs_lseek(fp->filep, off, whence);
}

static int devfs_fs_ioctl(struct fs_file_t *fp, unsigned int cmd, unsigned long arg)
{
	return devfs_ioctl(fp->filep, cmd, arg);
}

static int devfs_fs_close(struct fs_file_t *fp)
{
	int retval = 0;

	retval = devfs_close(fp->filep);

	/* Free file memory */
	k_mem_slab_free(&devfs_filep_pool, &fp->filep);
	fp->filep = NULL;

	return retval;
}

static int devfs_fs_opendir(struct fs_dir_t *dp, const char *path)
{
	void *dirp = NULL;
	int retval = 0;

	if (k_mem_slab_alloc(&devfs_dirp_pool, &dirp, K_NO_WAIT) == 0) {
		(void)memset(dirp, 0, sizeof(struct devfs_dir_t));
		dp->dirp = dirp;
	} else {
		return -ENOMEM;
	}

	retval = devfs_opendir(dp->dirp, path);
	if (retval < 0) {
		k_mem_slab_free(&devfs_dirp_pool, &dirp);
		dp->dirp = NULL;
	}

	return retval;
}

static void dfs_entry_to_fs_entry(const struct devfs_dirent_t *dfs_entry, struct fs_dirent *fs_entry)
{
	if (dfs_entry->type == devfs_type_dir)
		fs_entry->type = FS_DIR_ENTRY_DIR;
	else
		fs_entry->type = FS_DIR_ENTRY_FILE;
	fs_entry->size = dfs_entry->size;
	strncpy(fs_entry->name, dfs_entry->d_name, sizeof(fs_entry->name));
	fs_entry->name[sizeof(fs_entry->name) - 1] = '\0';
}

static int devfs_fs_readdir(struct fs_dir_t *dp, struct fs_dirent *entry)
{
	struct devfs_dirent_t _entry;
	int retval = 0;

	retval = devfs_readdir(dp->dirp, &_entry);
	if (retval < 0) {
		return retval;
	}

	dfs_entry_to_fs_entry(&_entry, entry);

	return 0;
}

static int devfs_fs_closedir(struct fs_dir_t *dp)
{
	return devfs_closedir(dp->dirp);
}

static int devfs_fs_stat(struct fs_mount_t *mountp,
			 			 const char *path, struct fs_dirent *entry)
{
	struct devfs_dirent_t _entry;
	int retval = 0;

	retval = devfs_stat(path, &_entry);
	if (retval < 0) {
		return retval;
	}

	dfs_entry_to_fs_entry(&_entry, entry);

	return 0;
}

static int devfs_fs_mount(struct fs_mount_t *mountp)
{
	return devfs_mount(mountp->mnt_point);
}

static int devfs_fs_unmount(struct fs_mount_t *mountp)
{
	return devfs_umount(mountp->mnt_point);
}

/* File system interface */
static const struct fs_file_system_t devfs_fs = {
	.open = devfs_fs_open,
	.close = devfs_fs_close,
	.read = devfs_fs_read,
	.write = devfs_fs_write,
	.lseek = devfs_fs_lseek,
#if defined(CONFIG_FILE_SYSTEM_DEVFS)
	.ioctl = devfs_fs_ioctl,
#endif
	.opendir = devfs_fs_opendir,
	.readdir = devfs_fs_readdir,
	.closedir = devfs_fs_closedir,
	.mount = devfs_fs_mount,
	.unmount = devfs_fs_unmount,
	.stat = devfs_fs_stat,
};

static int devfs_fs_init(const struct device *unused)
{
	ARG_UNUSED(unused);

	static struct fs_mount_t devfs_mnt = {
		.type = FS_DEVFS,
		.mnt_point = CONFIG_DEVFS_MOUNT_POINTER,
	};

	int rc = fs_register(FS_DEVFS, &devfs_fs);

	if (rc == 0) {
		rc = fs_mount(&devfs_mnt);
		if (rc < 0) {
			LOG_ERR("Automount %s failed: %d", devfs_mnt.mnt_point, rc);

			fs_unregister(FS_DEVFS, &devfs_fs);
		} else {
			LOG_INF("Automount %s succeeded", devfs_mnt.mnt_point);
		}
	}

	return rc;
}

SYS_INIT(devfs_fs_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
