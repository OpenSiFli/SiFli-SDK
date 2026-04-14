/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2005-02-22     Bernard      The first version.
 * 2011-12-08     Bernard      Merges rename patch from iamcacy.
 * 2015-05-27     Bernard      Fix the fd clear issue.
 * 2019-01-24     Bernard      Remove file repeatedly open check.
 */

#include <dfs.h>
#include <dfs_file.h>
#include <dfs_private.h>

/**
 * @addtogroup FileApi
 */

/*@{*/

/**
 * this function will open a file which specified by path with specified flags.
 *
 * @param fd the file descriptor pointer to return the corresponding result.
 * @param path the specified file path.
 * @param flags the flags for open operator.
 *
 * @return 0 on successful, -1 on failed.
 */
int dfs_file_open(struct dfs_fd *fd, const char *path, int flags)
{
    struct dfs_filesystem *fs;
    char *fullpath;
    int result;

    /* parameter check */
    if (fd == NULL)
        return -EINVAL;

    /* make sure we have an absolute path */
    fullpath = dfs_normalize_path(NULL, path);
    if (fullpath == NULL)
    {
        return -ENOMEM;
    }

    LOG_D("open file:%s", fullpath);

    /* find filesystem */
    fs = dfs_filesystem_lookup(fullpath);
    if (fs == NULL)
    {
        rt_free(fullpath); /* release path */

        return -ENOENT;
    }

    LOG_D("open in filesystem:%s", fs->ops->name);
    fd->fs    = fs;             /* set file system */
    fd->fops  = fs->ops->fops;  /* set file ops */

    /* initialize the fd item */
    fd->type  = FT_REGULAR;
    fd->flags = flags;
    fd->size  = 0;
    fd->pos   = 0;
    fd->data  = fs;

    if (!(fs->ops->flags & DFS_FS_FLAG_FULLPATH))
    {
        if (dfs_subdir(fs->path, fullpath) == NULL)
            fd->path = rt_strdup("/");
        else
            fd->path = rt_strdup(dfs_subdir(fs->path, fullpath));
        rt_free(fullpath);
        LOG_D("Actual file path: %s", fd->path);
    }
    else
    {
        fd->path = fullpath;
    }

    /* specific file system open routine */
    if (fd->fops->open == NULL)
    {
        /* clear fd */
        rt_free(fd->path);
        fd->path = NULL;

        return -ENOSYS;
    }

    if ((result = fd->fops->open(fd)) < 0)
    {
        /* clear fd */
        rt_free(fd->path);
        fd->path = NULL;

        LOG_D("%s open failed", fullpath);

        return result;
    }

    fd->flags |= DFS_F_OPEN;
    if (flags & O_DIRECTORY)
    {
        fd->type = FT_DIRECTORY;
        fd->flags |= DFS_F_DIRECTORY;
    }

    LOG_D("open successful");
    return 0;
}

/**
 * this function will close a file descriptor.
 *
 * @param fd the file descriptor to be closed.
 *
 * @return 0 on successful, -1 on failed.
 */
int dfs_file_close(struct dfs_fd *fd)
{
    int result = 0;

    if (fd == NULL)
        return -ENXIO;

    if (fd->fops->close != NULL)
        result = fd->fops->close(fd);

    /* close fd error, return */
    if (result < 0)
        return result;

    rt_free(fd->path);
    fd->path = NULL;

    return result;
}

/**
 * this function will perform a io control on a file descriptor.
 *
 * @param fd the file descriptor.
 * @param cmd the command to send to file descriptor.
 * @param args the argument to send to file descriptor.
 *
 * @return 0 on successful, -1 on failed.
 */
int dfs_file_ioctl(struct dfs_fd *fd, int cmd, void *args)
{
    if (fd == NULL)
        return -EINVAL;

    /* regular file system fd */
    if (fd->type == FT_REGULAR)
    {
        switch (cmd)
        {
        case F_GETFL:
            return fd->flags; /* return flags */
        case F_SETFL:
        {
            int flags = (int)(rt_base_t)args;
            int mask  = O_NONBLOCK | O_APPEND;

            flags &= mask;
            fd->flags &= ~mask;
            fd->flags |= flags;
        }
        return 0;
        }
    }

    if (fd->fops->ioctl != NULL)
        return fd->fops->ioctl(fd, cmd, args);

    return -ENOSYS;
}

/**
 * this function will read specified length data from a file descriptor to a
 * buffer.
 *
 * @param fd the file descriptor.
 * @param buf the buffer to save the read data.
 * @param len the length of data buffer to be read.
 *
 * @return the actual read data bytes or 0 on end of file or failed.
 */
int dfs_file_read(struct dfs_fd *fd, void *buf, size_t len)
{
    int result = 0;

    if (fd == NULL)
        return -EINVAL;

    if (fd->fops->read == NULL)
        return -ENOSYS;

    if ((result = fd->fops->read(fd, buf, len)) < 0)
        fd->flags |= DFS_F_EOF;

    return result;
}

/**
 * this function will fetch directory entries from a directory descriptor.
 *
 * @param fd the directory descriptor.
 * @param dirp the dirent buffer to save result.
 * @param nbytes the available room in the buffer.
 *
 * @return the read dirent, others on failed.
 */
int dfs_file_getdents(struct dfs_fd *fd, struct dirent *dirp, size_t nbytes)
{
    /* parameter check */
    if (fd == NULL || fd->type != FT_DIRECTORY)
        return -EINVAL;

    if (fd->fops->getdents != NULL)
        return fd->fops->getdents(fd, dirp, nbytes);

    return -ENOSYS;
}

/**
 * this function will unlink (remove) a specified path file from file system.
 *
 * @param path the specified path file to be unlinked.
 *
 * @return 0 on successful, -1 on failed.
 */
int dfs_file_unlink(const char *path)
{
    int result;
    char *fullpath;
    struct dfs_filesystem *fs;

    /* Make sure we have an absolute path */
    fullpath = dfs_normalize_path(NULL, path);
    if (fullpath == NULL)
    {
        return -EINVAL;
    }

    /* get filesystem */
    if ((fs = dfs_filesystem_lookup(fullpath)) == NULL)
    {
        result = -ENOENT;
        goto __exit;
    }

    /* Check whether file is already open */
    if (fd_is_open(fullpath) == 0)
    {
        result = -EBUSY;
        goto __exit;
    }

    if (fs->ops->unlink != NULL)
    {
        if (!(fs->ops->flags & DFS_FS_FLAG_FULLPATH))
        {
            if (dfs_subdir(fs->path, fullpath) == NULL)
                result = fs->ops->unlink(fs, "/");
            else
                result = fs->ops->unlink(fs, dfs_subdir(fs->path, fullpath));
        }
        else
            result = fs->ops->unlink(fs, fullpath);
    }
    else result = -ENOSYS;

__exit:
    rt_free(fullpath);
    return result;
}

/**
 * this function will write some specified length data to file system.
 *
 * @param fd the file descriptor.
 * @param buf the data buffer to be written.
 * @param len the data buffer length
 *
 * @return the actual written data length.
 */
int dfs_file_write(struct dfs_fd *fd, const void *buf, size_t len)
{
    if (fd == NULL)
        return -EINVAL;

    if (fd->fops->write == NULL)
        return -ENOSYS;

    return fd->fops->write(fd, buf, len);
}

/**
 * this function will flush buffer on a file descriptor.
 *
 * @param fd the file descriptor.
 *
 * @return 0 on successful, -1 on failed.
 */
int dfs_file_flush(struct dfs_fd *fd)
{
    if (fd == NULL)
        return -EINVAL;

    if (fd->fops->flush == NULL)
        return -ENOSYS;

    return fd->fops->flush(fd);
}

/**
 * this function will seek the offset for specified file descriptor.
 *
 * @param fd the file descriptor.
 * @param offset the offset to be sought.
 *
 * @return the current position after seek.
 */
int dfs_file_lseek(struct dfs_fd *fd, off_t offset)
{
    int result;

    if (fd == NULL)
        return -EINVAL;

    if (fd->fops->lseek == NULL)
        return -ENOSYS;

    result = fd->fops->lseek(fd, offset);

    /* update current position */
    if (result >= 0)
        fd->pos = result;

    return result;
}
/**
 * this function will fast seek the offset for specified file descriptor.
 *
 * @param fd the file descriptor.
 * @param enabled function fast seek.
 *
 * @return the current position after seek.
 */
int dfs_file_enable_fast_seek(struct dfs_fd *fd, uint8_t enable)
{
    int result;

    if (fd == NULL)
        return -EINVAL;

    if (fd->fops->enable_fast_lseek == NULL)
        return -ENOSYS;

    result = fd->fops->enable_fast_lseek(fd, enable);

    /* update current position */
    if (result >= 0)
        fd->pos = result;

    return result;
}

/**
 * this function will get file information.
 *
 * @param path the file path.
 * @param buf the data buffer to save stat description.
 *
 * @return 0 on successful, -1 on failed.
 */
int dfs_file_stat(const char *path, struct stat *buf)
{
    int result;
    char *fullpath;
    struct dfs_filesystem *fs;

    fullpath = dfs_normalize_path(NULL, path);
    if (fullpath == NULL)
    {
        return -1;
    }

    if ((fs = dfs_filesystem_lookup(fullpath)) == NULL)
    {
        LOG_E(
            "can't find mounted filesystem on this path:%s", fullpath);
        rt_free(fullpath);

        return -ENOENT;
    }

    if ((fullpath[0] == '/' && fullpath[1] == '\0') ||
            (dfs_subdir(fs->path, fullpath) == NULL))
    {
        /* it's the root directory */
        buf->st_dev   = 0;

        buf->st_mode  = S_IRUSR | S_IRGRP | S_IROTH |
                        S_IWUSR | S_IWGRP | S_IWOTH;
        buf->st_mode |= S_IFDIR | S_IXUSR | S_IXGRP | S_IXOTH;

        buf->st_size    = 0;
        buf->st_mtime   = 0;

        /* release full path */
        rt_free(fullpath);

        return RT_EOK;
    }
    else
    {
        if (fs->ops->stat == NULL)
        {
            rt_free(fullpath);
            LOG_E(
                "the filesystem didn't implement this function");

            return -ENOSYS;
        }

        /* get the real file path and get file stat */
        if (fs->ops->flags & DFS_FS_FLAG_FULLPATH)
            result = fs->ops->stat(fs, fullpath, buf);
        else
            result = fs->ops->stat(fs, dfs_subdir(fs->path, fullpath), buf);
    }

    rt_free(fullpath);

    return result;
}

/**
 * this function will rename an old path name to a new path name.
 *
 * @param oldpath the old path name.
 * @param newpath the new path name.
 *
 * @return 0 on successful, -1 on failed.
 */
int dfs_file_rename(const char *oldpath, const char *newpath)
{
    int result;
    struct dfs_filesystem *oldfs, *newfs;
    char *oldfullpath, *newfullpath;

    result = RT_EOK;
    newfullpath = NULL;
    oldfullpath = NULL;

    oldfullpath = dfs_normalize_path(NULL, oldpath);
    if (oldfullpath == NULL)
    {
        result = -ENOENT;
        goto __exit;
    }

    newfullpath = dfs_normalize_path(NULL, newpath);
    if (newfullpath == NULL)
    {
        result = -ENOENT;
        goto __exit;
    }

    oldfs = dfs_filesystem_lookup(oldfullpath);
    newfs = dfs_filesystem_lookup(newfullpath);

    if (oldfs == newfs)
    {
        if (oldfs->ops->rename == NULL)
        {
            result = -ENOSYS;
        }
        else
        {
            if (oldfs->ops->flags & DFS_FS_FLAG_FULLPATH)
                result = oldfs->ops->rename(oldfs, oldfullpath, newfullpath);
            else
                /* use sub directory to rename in file system */
                result = oldfs->ops->rename(oldfs,
                                            dfs_subdir(oldfs->path, oldfullpath),
                                            dfs_subdir(newfs->path, newfullpath));
        }
    }
    else
    {
        result = -EXDEV;
    }

__exit:
    rt_free(oldfullpath);
    rt_free(newfullpath);

    /* not at same file system, return EXDEV */
    return result;
}

/**
 * this function is will cause the regular file referenced by fd
 * to be truncated to a size of precisely length bytes.
 *
 * @param fd the file descriptor.
 * @param length the length to be truncated.
 *
 * @return the status of truncated.
 */
int dfs_file_ftruncate(struct dfs_fd *fd, off_t length)
{
    int result;

    /* fd is null or not a regular file system fd, or length is invalid */
    if (fd == NULL || fd->type != FT_REGULAR || length < 0)
        return -EINVAL;

    if (fd->fops->ioctl == NULL)
        return -ENOSYS;

    result = fd->fops->ioctl(fd, RT_FIOFTRUNCATE, (void *)&length);

    /* update current size */
    if (result == 0)
        fd->size = length;

    return result;
}
const char *dfs_file_getext(const char *fn)
{
    size_t i;
    for (i = strlen(fn); i > 0; i--)
    {
        if (fn[i] == '.')
        {
            return &fn[i + 1];
        }
        else if (fn[i] == '/' || fn[i] == '\\')
        {
            return ""; /*No extension if a '\' or '/' found*/
        }
    }
    return ""; /*Empty string if no '.' in the file name.*/
}


void ls(const char *pathname)
{
    struct stat stat;
    struct dirent dirent;
    struct dfs_fd fd;
    int length;
    char *fullpath, *path;

    fullpath = NULL;
    if (pathname == NULL)
    {
#ifdef DFS_USING_WORKDIR
        /* open current working directory */
        path = rt_strdup(working_directory);
#else
        path = rt_strdup("/");
#endif
        if (path == NULL)
            return ; /* out of memory */
    }
    else
    {
        path = (char *)pathname;
    }

    /* list directory */
    if (dfs_file_open(&fd, path, O_DIRECTORY) == 0)
    {
        rt_kprintf("Directory %s:\n", path);
        do
        {
            memset(&dirent, 0, sizeof(struct dirent));
            length = dfs_file_getdents(&fd, &dirent, sizeof(struct dirent));
            if (length > 0)
            {
                memset(&stat, 0, sizeof(struct stat));

                /* build full path for each file */
                fullpath = dfs_normalize_path(path, dirent.d_name);
                if (fullpath == NULL)
                    break;

                if (dfs_file_stat(fullpath, &stat) == 0)
                {
                    rt_kprintf("%-64s", dirent.d_name);
                    if (S_ISDIR(stat.st_mode))
                    {
                        rt_kprintf("%-25s\n", "<DIR>");
                    }
                    else
                    {
                        rt_kprintf("%25lu\n", stat.st_size);
                    }
                }
                else
                    rt_kprintf("BAD file: %s\n", dirent.d_name);
                rt_free(fullpath);
            }
        }
        while (length > 0);

        dfs_file_close(&fd);
    }
    else
    {
        rt_kprintf("No such directory\n");
    }
    if (pathname == NULL)
        rt_free(path);
}

void rm(const char *filename)
{
    if (dfs_file_unlink(filename) < 0)
    {
        rt_kprintf("Delete %s failed\n", filename);
    }
}

void cat(const char *filename)
{
    uint32_t length;
    char buffer[81];
    struct dfs_fd fd;

    if (dfs_file_open(&fd, filename, O_RDONLY) < 0)
    {
        rt_kprintf("Open %s failed\n", filename);

        return;
    }

    do
    {
        memset(buffer, 0, sizeof(buffer));
        length = dfs_file_read(&fd, buffer, sizeof(buffer) - 1);
        if (length > 0)
        {
            rt_kprintf("%s", buffer);
        }
    }
    while (length > 0);

    dfs_file_close(&fd);
}

#define BUF_SZ  4096
static void copyfile(const char *src, const char *dst)
{
    struct dfs_fd src_fd;
    rt_uint8_t *block_ptr;
    rt_int32_t read_bytes;
    struct dfs_fd fd;

    block_ptr = rt_malloc(BUF_SZ);
    if (block_ptr == NULL)
    {
        rt_kprintf("out of memory\n");

        return;
    }

    if (dfs_file_open(&src_fd, src, O_RDONLY) < 0)
    {
        rt_free(block_ptr);
        rt_kprintf("Read %s failed\n", src);

        return;
    }
    if (dfs_file_open(&fd, dst, O_WRONLY | O_CREAT | O_TRUNC) < 0)
    {
        rt_free(block_ptr);
        dfs_file_close(&src_fd);

        rt_kprintf("Write %s failed\n", dst);

        return;
    }

    do
    {
        read_bytes = dfs_file_read(&src_fd, block_ptr, BUF_SZ);
        if (read_bytes > 0)
        {
            int length;

            length = dfs_file_write(&fd, block_ptr, read_bytes);
            if (length != read_bytes)
            {
                /* write failed. */
                rt_kprintf("Write file data failed, errno=%d\n", length);
                break;
            }
        }
    }
    while (read_bytes > 0);

    dfs_file_close(&src_fd);
    dfs_file_close(&fd);
    rt_free(block_ptr);
}

extern int mkdir(const char *path, mode_t mode);
static void copydir(const char *src, const char *dst)
{
    struct dirent dirent;
    struct stat stat;
    int length;
    struct dfs_fd cpfd;
    if (dfs_file_open(&cpfd, src, O_DIRECTORY) < 0)
    {
        rt_kprintf("open %s failed\n", src);
        return ;
    }

    do
    {
        memset(&dirent, 0, sizeof(struct dirent));

        length = dfs_file_getdents(&cpfd, &dirent, sizeof(struct dirent));
        if (length > 0)
        {
            char *src_entry_full = NULL;
            char *dst_entry_full = NULL;

            if (strcmp(dirent.d_name, "..") == 0 || strcmp(dirent.d_name, ".") == 0)
                continue;

            /* build full path for each file */
            if ((src_entry_full = dfs_normalize_path(src, dirent.d_name)) == NULL)
            {
                rt_kprintf("out of memory!\n");
                break;
            }
            if ((dst_entry_full = dfs_normalize_path(dst, dirent.d_name)) == NULL)
            {
                rt_kprintf("out of memory!\n");
                rt_free(src_entry_full);
                break;
            }

            memset(&stat, 0, sizeof(struct stat));
            if (dfs_file_stat(src_entry_full, &stat) != 0)
            {
                rt_kprintf("open file: %s failed\n", dirent.d_name);
                continue;
            }

            if (S_ISDIR(stat.st_mode))
            {
                mkdir(dst_entry_full, 0);
                copydir(src_entry_full, dst_entry_full);
            }
            else
            {
                copyfile(src_entry_full, dst_entry_full);
            }
            rt_free(src_entry_full);
            rt_free(dst_entry_full);
        }
    }
    while (length > 0);

    dfs_file_close(&cpfd);
}

static const char *_get_path_lastname(const char *path)
{
    char *ptr;
    if ((ptr = strrchr(path, '/')) == NULL)
        return path;

    /* skip the '/' then return */
    return ++ptr;
}
rt_err_t copy(const char *src, const char *dst)
{
#define FLAG_SRC_TYPE      0x03
#define FLAG_SRC_IS_DIR    0x01
#define FLAG_SRC_IS_FILE   0x02
#define FLAG_SRC_NON_EXSIT 0x00

#define FLAG_DST_TYPE      0x0C
#define FLAG_DST_IS_DIR    0x04
#define FLAG_DST_IS_FILE   0x08
#define FLAG_DST_NON_EXSIT 0x00

    struct stat stat;
    uint32_t flag = 0;

    /* check the staus of src and dst */
    if (dfs_file_stat(src, &stat) < 0)
    {
        rt_kprintf("copy failed, bad %s\n", src);
        return RT_ERROR;
    }
    if (S_ISDIR(stat.st_mode))
        flag |= FLAG_SRC_IS_DIR;
    else
        flag |= FLAG_SRC_IS_FILE;

    if (dfs_file_stat(dst, &stat) < 0)
    {
        flag |= FLAG_DST_NON_EXSIT;
    }
    else
    {
        if (S_ISDIR(stat.st_mode))
            flag |= FLAG_DST_IS_DIR;
        else
            flag |= FLAG_DST_IS_FILE;
    }

    //2. check status
    if ((flag & FLAG_SRC_IS_DIR) && (flag & FLAG_DST_IS_FILE))
    {
        rt_kprintf("cp faild, cp dir to file is not permitted!\n");
        return RT_ERROR;
    }

    //3. do copy
    if (flag & FLAG_SRC_IS_FILE)
    {
        if (flag & FLAG_DST_IS_DIR)
        {
            char *fdst;
            fdst = dfs_normalize_path(dst, _get_path_lastname(src));
            if (fdst == NULL)
            {
                rt_kprintf("out of memory\n");
                return RT_ERROR;
            }
            copyfile(src, fdst);
            rt_free(fdst);
        }
        else
        {
            copyfile(src, dst);
        }
    }
    else //flag & FLAG_SRC_IS_DIR
    {
        if (flag & FLAG_DST_IS_DIR)
        {
            char *fdst;
            fdst = dfs_normalize_path(dst, _get_path_lastname(src));
            if (fdst == NULL)
            {
                rt_kprintf("out of memory\n");
                return RT_ERROR;
            }
            mkdir(fdst, 0);
            copydir(src, fdst);
            rt_free(fdst);
        }
        else if ((flag & FLAG_DST_TYPE) == FLAG_DST_NON_EXSIT)
        {
            mkdir(dst, 0);
            copydir(src, dst);
        }
        else
        {
            copydir(src, dst);
        }
    }
    return RT_EOK;
}
static uint32_t dir_used_size;
static uint32_t dir_occupy_size;
static uint32_t dir_file_num;
enum
{
    DIR_DEL,
    MATCH_DEL,
    MATCH_LS,
    DIR_SIZE,
    DIR_OPEN,
    DIR_CLOSE,
};
extern int open(const char *file, int flags, ...);
void trav_file(const char *src, uint8_t mode, char *argv)
{
    struct dirent dirent;
    struct stat stat;
    int length;
    struct dfs_fd fd;
    if (dfs_file_open(&fd, src, O_DIRECTORY) >= 0)
    {
        do
        {
            memset(&dirent, 0, sizeof(struct dirent));
            length = dfs_file_getdents(&fd, &dirent, sizeof(struct dirent));
            if (length > 0)
            {
                char *src_entry_full = NULL;
                if (strcmp(dirent.d_name, "..") == 0 || strcmp(dirent.d_name, ".") == 0)
                    continue;
                if ((src_entry_full = dfs_normalize_path(src, dirent.d_name)) == NULL)
                {
                    rt_kprintf("out of memory!\n");
                    break;
                }
                memset(&stat, 0, sizeof(struct stat));
                if (dfs_file_stat(src_entry_full, &stat) != 0)
                {
                    rt_free(src_entry_full);
                    rt_kprintf("dfs_file_stat: %s failed\n", dirent.d_name);
                    continue;
                }
                if (S_ISDIR(stat.st_mode))
                {
                    trav_file(src_entry_full, mode, argv);
                }
                else
                {
                    if (DIR_DEL == mode || (MATCH_DEL == mode && strstr(dirent.d_name, argv)))
                    {
                        dir_file_num++;
                        if (dfs_file_unlink(src_entry_full) < 0)
                            rt_kprintf("Delete file %s failed!!!\n", src_entry_full);
                    }
                    else if (MATCH_LS == mode && strstr(dirent.d_name, argv))
                    {
                        dir_file_num++;
                        rt_kprintf("Match file: %-64s size %6d\n", src_entry_full, stat.st_size);
                    }
                    else if (DIR_SIZE == mode)
                    {
                        dir_file_num++;
                        dir_used_size += (stat.st_size);
                        dir_occupy_size += (stat.st_size + 2047) / 2048 * 2048;
                    }
                    else if (DIR_OPEN == mode)
                    {
                        dir_file_num++;
                        int fd = open(src_entry_full, O_RDONLY);
                        if (0 > fd)
                            rt_kprintf("open file %s failed!!!\n", src_entry_full);
                    }
                    else if (DIR_CLOSE == mode)
                    {
                        ;
                    }
                }
                rt_free(src_entry_full);
            }
        }
        while (length > 0);
        dfs_file_close(&fd);
    }
    if (DIR_DEL == mode)
    {
        if (dfs_file_unlink(src) < 0)
            rt_kprintf("Delete dir %s failed!!!\n", (src));
    }
}
void dirsize(int argc, char **argv)
{
    const char *p = "/";
    dir_file_num = 0;
    dir_used_size = 0;
    dir_occupy_size = 0;
    if (argc >= 2) p = (const char *) argv[1];
    rt_kprintf("%s: %d %s\n", __func__, argc, p);
    trav_file(p, DIR_SIZE, NULL);
    rt_kprintf("%s(%s): occupy %d used_size %d file_num\n", __func__, (p), dir_occupy_size, dir_used_size, dir_file_num);
    struct statfs buffer;
    int result = dfs_statfs(p, &buffer);
    if (result != 0)
    {
        rt_kprintf("dfs_statfs failed.\n");
        return;
    }
    uint32_t cap = ((uint32_t) buffer.f_bsize) * ((uint32_t) buffer.f_blocks);
    uint32_t free = ((uint32_t) buffer.f_bsize) * ((uint32_t) buffer.f_bfree);
    rt_kprintf("df(%s): cap %d, free %d, used %d\n", (p), cap, free, cap - free);
}
void deldir(int argc, char **argv)
{
    dir_file_num = 0;
    rt_kprintf("%s: %d\n", __func__, argc);
    if (argc < 2) return;
    trav_file(argv[1], DIR_DEL, NULL);
    rt_kprintf("%s: %s, file_num %d\n", __func__, argv[1], dir_file_num);
}
void delmatch(int argc, char **argv)
{
    dir_file_num = 0;
    rt_kprintf("%s: %d\n", __func__, argc);
    if (argc < 3) return;
    trav_file(argv[1], MATCH_DEL, argv[2]);
    rt_kprintf("%s: %s, file_num %d\n", __func__, argv[1], dir_file_num);
}
void lsmatch(int argc, char **argv)
{
    dir_file_num = 0;
    rt_kprintf("%s: %d\n", __func__, argc);
    if (argc < 3) return;
    trav_file(argv[1], MATCH_LS, argv[2]);
    rt_kprintf("%s: %s, file_num %d\n", __func__, argv[1], dir_file_num);
}
void _copydir(int argc, char **argv)
{
    dir_file_num = 0;
    rt_kprintf("%s: %d\n", __func__, argc);
    if (argc < 3) return;
    mkdir(argv[2], 0);
    copydir(argv[1], argv[2]);
    rt_kprintf("%s: %s, file_num %d\n", __func__, argv[1], dir_file_num);
}
void _opendir(int argc, char **argv)
{
    dir_file_num = 0;
    uint32_t tick = rt_tick_get_millisecond();
    rt_kprintf("%s: %d\n", __func__, argc);
    if (argc < 2) return;
    trav_file(argv[1], DIR_OPEN, NULL);
    rt_kprintf("%s: %s, %d(ms), file_num %d\n", __func__, argv[1], rt_tick_get_millisecond() - tick, dir_file_num);
}
#include "rtthread.h"
#ifdef RT_USING_FINSH
    #include <finsh.h>
    FINSH_FUNCTION_EXPORT(ls, list directory contents);
    FINSH_FUNCTION_EXPORT(rm, remove files or directories);
    FINSH_FUNCTION_EXPORT(cat, print file);
    FINSH_FUNCTION_EXPORT(copy, copy file or dir);
    MSH_CMD_EXPORT_REL(dirsize, dirsize, dirsize dir);
    MSH_CMD_EXPORT_REL(deldir, deldir, deldir file or dir);
    MSH_CMD_EXPORT_REL(delmatch, delmatch, delmatch dir match_file);
    MSH_CMD_EXPORT_REL(lsmatch, lsmatch, lsmatch dir match_file);
    MSH_CMD_EXPORT_REL(_opendir, opendir, opendir dir);
    MSH_CMD_EXPORT_REL(_copydir, cpdir, cpdir src dst);
#endif
/* @} */

