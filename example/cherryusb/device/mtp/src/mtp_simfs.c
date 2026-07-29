/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * All Intellectual Property rights in the software belongs to sakumisu.
 *
 *   Licensing information
 *   ---------------------
 *
 *   Licensor:                 sakumisu
 *   Licensed to:
 *   License software version: cherryusb mtp device v1.0
 *   Licensed platform:
 */

#include "usbd_core.h"
#include "usbd_mtp.h"

#include <dfs_posix.h>
#include <dfs_fs.h>
#include <fcntl.h>
#include <sys/stat.h>

#define MTP_SIMFS_ROOT_PATH "/"
#define MTP_SIMFS_MOUNT_POINT "/"
#define MTP_SIMFS_DEVICE_NAME "mtp_ram"
#define MTP_SIMFS_FILE_NAME "cherryusb.txt"
#define MTP_SIMFS_FILE_CAPACITY (64U * 1024U)
#define MTP_SIMFS_BLOCK_SIZE 512U
#define MTP_SIMFS_GEOMETRY_BLOCK_SIZE (MTP_SIMFS_BLOCK_SIZE)
#define MTP_SIMFS_SECTOR_COUNT (MTP_SIMFS_FILE_CAPACITY / MTP_SIMFS_BLOCK_SIZE)

struct mtp_simfs_device
{
    struct rt_device parent;
    uint8_t *storage;
};

struct mtp_simfs_dir
{
    DIR *dir;
    char path[DFS_PATH_MAX];
};

static struct mtp_simfs_device s_device;
static struct mtp_simfs_dir s_dir;
static uint8_t s_storage[MTP_SIMFS_FILE_CAPACITY];
static bool s_device_registered;
static bool s_fs_mounted;

static bool mtp_simfs_is_root_path(const char *path)
{
    const char *tail;

    if (path == NULL)
    {
        return false;
    }

    if (strcmp(path, "0:") == 0)
    {
        return true;
    }

    if (strncmp(path, MTP_SIMFS_ROOT_PATH, strlen(MTP_SIMFS_ROOT_PATH)) != 0)
    {
        return false;
    }

    tail = path + strlen(MTP_SIMFS_ROOT_PATH);
    while (*tail == '/')
    {
        tail++;
    }

    return *tail == '\0';
}

static const char *mtp_simfs_to_host_path(const char *path, char *buffer, size_t buffer_size)
{
    size_t path_len;

    if (path == NULL || buffer == NULL || buffer_size == 0U)
    {
        return NULL;
    }

    if (mtp_simfs_is_root_path(path))
    {
        if (buffer_size < sizeof(MTP_SIMFS_MOUNT_POINT))
        {
            return NULL;
        }

        strcpy(buffer, MTP_SIMFS_MOUNT_POINT);
        return buffer;
    }

    if (strncmp(path, MTP_SIMFS_ROOT_PATH, strlen(MTP_SIMFS_ROOT_PATH)) == 0)
    {
        path += strlen(MTP_SIMFS_ROOT_PATH);
        while (*path == '/')
        {
            path++;
        }
    }
    else if (path[0] == '/')
    {
        path++;
    }

    if (*path == '\0')
    {
        if (buffer_size < sizeof(MTP_SIMFS_MOUNT_POINT))
        {
            return NULL;
        }

        strcpy(buffer, MTP_SIMFS_MOUNT_POINT);
        return buffer;
    }

    path_len = strlen(path);
    if (strlen(MTP_SIMFS_MOUNT_POINT) + 1U + path_len + 1U > buffer_size)
    {
        return NULL;
    }

    strcpy(buffer, MTP_SIMFS_MOUNT_POINT);
    strcat(buffer, "/");
    strcat(buffer, path);
    return buffer;
}

static uint32_t mtp_simfs_to_mtp_mode(mode_t mode)
{
    if (S_ISDIR(mode))
    {
        return MTP_S_IFDIR | MTP_S_IRUSR | MTP_S_IRGRP | MTP_S_IROTH |
               MTP_S_IXUSR | MTP_S_IXGRP | MTP_S_IXOTH;
    }

    return MTP_S_IFREG | MTP_S_IRUSR | MTP_S_IRGRP | MTP_S_IROTH |
           MTP_S_IWUSR | MTP_S_IWGRP | MTP_S_IWOTH;
}

static uint8_t mtp_simfs_to_dtype(mode_t mode)
{
    return S_ISDIR(mode) ? 4U : 8U;
}

static rt_err_t mtp_simfs_device_init(rt_device_t dev)
{
    (void)dev;
    return RT_EOK;
}

static rt_err_t mtp_simfs_device_open(rt_device_t dev, rt_uint16_t oflag)
{
    (void)dev;
    (void)oflag;
    return RT_EOK;
}

static rt_err_t mtp_simfs_device_close(rt_device_t dev)
{
    (void)dev;
    return RT_EOK;
}

static rt_size_t mtp_simfs_device_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    struct mtp_simfs_device *device = (struct mtp_simfs_device *)dev;
    size_t offset;
    size_t bytes;

    if (device == NULL || buffer == NULL || pos < 0)
    {
        return 0;
    }

    offset = (size_t)pos * MTP_SIMFS_BLOCK_SIZE;
    bytes = size * MTP_SIMFS_BLOCK_SIZE;

    if (offset >= MTP_SIMFS_FILE_CAPACITY)
    {
        return 0;
    }

    if (offset + bytes > MTP_SIMFS_FILE_CAPACITY)
    {
        bytes = MTP_SIMFS_FILE_CAPACITY - offset;
    }

    memcpy(buffer, &device->storage[offset], bytes);
    return bytes / MTP_SIMFS_BLOCK_SIZE;
}

static rt_size_t mtp_simfs_device_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    struct mtp_simfs_device *device = (struct mtp_simfs_device *)dev;
    size_t offset;
    size_t bytes;

    if (device == NULL || buffer == NULL || pos < 0)
    {
        return 0;
    }

    offset = (size_t)pos * MTP_SIMFS_BLOCK_SIZE;
    bytes = size * MTP_SIMFS_BLOCK_SIZE;

    if (offset >= MTP_SIMFS_FILE_CAPACITY)
    {
        return 0;
    }

    if (offset + bytes > MTP_SIMFS_FILE_CAPACITY)
    {
        bytes = MTP_SIMFS_FILE_CAPACITY - offset;
    }

    memcpy(&device->storage[offset], buffer, bytes);
    return bytes / MTP_SIMFS_BLOCK_SIZE;
}

static rt_err_t mtp_simfs_device_control(rt_device_t dev, int cmd, void *args)
{
    (void)dev;

    if (cmd == RT_DEVICE_CTRL_BLK_GETGEOME)
    {
        struct rt_device_blk_geometry *geometry = (struct rt_device_blk_geometry *)args;

        if (geometry == NULL)
        {
            return -RT_ERROR;
        }

        geometry->sector_count = MTP_SIMFS_SECTOR_COUNT;
        geometry->bytes_per_sector = MTP_SIMFS_BLOCK_SIZE;
        geometry->block_size = MTP_SIMFS_GEOMETRY_BLOCK_SIZE;
        return RT_EOK;
    }

    if (cmd == RT_DEVICE_CTRL_BLK_SYNC)
    {
        return RT_EOK;
    }

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops mtp_simfs_device_ops =
{
    mtp_simfs_device_init,
    mtp_simfs_device_open,
    mtp_simfs_device_close,
    mtp_simfs_device_read,
    mtp_simfs_device_write,
    mtp_simfs_device_control,
};
#endif

static int mtp_simfs_register_device(void)
{
    rt_device_t device;

    if (s_device_registered)
    {
        return 0;
    }

    memset(&s_device, 0, sizeof(s_device));
    s_device.storage = s_storage;
    device = &s_device.parent;
    device->type = RT_Device_Class_Block;

#ifdef RT_USING_DEVICE_OPS
    device->ops = &mtp_simfs_device_ops;
#else
    device->init = mtp_simfs_device_init;
    device->open = mtp_simfs_device_open;
    device->close = mtp_simfs_device_close;
    device->read = mtp_simfs_device_read;
    device->write = mtp_simfs_device_write;
    device->control = mtp_simfs_device_control;
#endif

    if (rt_device_register(device,
                           MTP_SIMFS_DEVICE_NAME,
                           RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE) != RT_EOK)
    {
        return -1;
    }

    s_device_registered = true;
    return 0;
}

static int mtp_simfs_create_demo_file(void)
{
    static const char demo_path[] = MTP_SIMFS_MOUNT_POINT "/" MTP_SIMFS_FILE_NAME;
    static const char demo_content[] = "CherryUSB MTP test file\r\n";
    int fd;
    int result;

    fd = open(demo_path, O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd < 0)
    {
        return -1;
    }

    result = write(fd, demo_content, sizeof(demo_content) - 1U);
    if (result != (int)(sizeof(demo_content) - 1U))
    {
        close(fd);
        return -1;
    }

    return close(fd);
}

const char *usbd_mtp_fs_root_path(void)
{
    return MTP_SIMFS_ROOT_PATH;
}

const char *usbd_mtp_fs_description(void)
{
    return "CherryUSB MTP DFS RAM Disk";
}

int usbd_mtp_mkdir(const char *path)
{
    char host_path[DFS_PATH_MAX];

    path = mtp_simfs_to_host_path(path, host_path, sizeof(host_path));
    if (path == NULL || strcmp(path, MTP_SIMFS_MOUNT_POINT) == 0)
    {
        return -1;
    }

    return mkdir(path, 0);
}

int usbd_mtp_rmdir(const char *path)
{
    char host_path[DFS_PATH_MAX];

    path = mtp_simfs_to_host_path(path, host_path, sizeof(host_path));
    if (path == NULL || strcmp(path, MTP_SIMFS_MOUNT_POINT) == 0)
    {
        return -1;
    }

    return rmdir(path);
}

MTP_DIR *usbd_mtp_opendir(const char *name)
{
    char host_path[DFS_PATH_MAX];

    if (!s_fs_mounted)
    {
        return NULL;
    }

    name = mtp_simfs_to_host_path(name, host_path, sizeof(host_path));
    if (name == NULL)
    {
        return NULL;
    }

    s_dir.dir = opendir(name);
    if (s_dir.dir == NULL)
    {
        return NULL;
    }

    strncpy(s_dir.path, name, sizeof(s_dir.path) - 1U);
    s_dir.path[sizeof(s_dir.path) - 1U] = '\0';
    return (MTP_DIR *)&s_dir;
}

int usbd_mtp_closedir(MTP_DIR *dir)
{
    struct mtp_simfs_dir *simfs_dir = (struct mtp_simfs_dir *)dir;

    if (simfs_dir == NULL || simfs_dir->dir == NULL)
    {
        return -1;
    }

    simfs_dir->path[0] = '\0';
    return closedir(simfs_dir->dir);
}

struct mtp_dirent *usbd_mtp_readdir(MTP_DIR *dir)
{
    static struct mtp_dirent entry;
    char host_path[DFS_PATH_MAX];
    struct mtp_simfs_dir *simfs_dir = (struct mtp_simfs_dir *)dir;
    struct dirent *dirent;
    struct stat st;

    if (simfs_dir == NULL || simfs_dir->dir == NULL)
    {
        return NULL;
    }

    for (;;)
    {
        dirent = readdir(simfs_dir->dir);
        if (dirent == NULL)
        {
            return NULL;
        }

        if (strcmp(dirent->d_name, ".") != 0 && strcmp(dirent->d_name, "..") != 0)
        {
            break;
        }
    }

    memset(&entry, 0, sizeof(entry));
    strncpy(entry.d_name, dirent->d_name, sizeof(entry.d_name) - 1U);
    entry.d_name[sizeof(entry.d_name) - 1U] = '\0';
    entry.d_namlen = (uint8_t)strlen(entry.d_name);
    entry.d_reclen = sizeof(entry);

    if (strlen(simfs_dir->path) + 1U + strlen(dirent->d_name) + 1U <= sizeof(host_path))
    {
        strcpy(host_path, simfs_dir->path);
        if (host_path[strlen(host_path) - 1U] != '/')
        {
            strcat(host_path, "/");
        }
        strcat(host_path, dirent->d_name);

        if (stat(host_path, &st) == 0)
        {
            entry.d_type = mtp_simfs_to_dtype(st.st_mode);
        }
        else
        {
            entry.d_type = dirent->d_type;
        }
    }
    else
    {
        entry.d_type = dirent->d_type;
    }

    return &entry;
}

int usbd_mtp_stat(const char *path, struct mtp_stat *buf)
{
    char host_path[DFS_PATH_MAX];
    struct stat st;

    if (buf == NULL)
    {
        return -1;
    }

    memset(buf, 0, sizeof(*buf));

    if (mtp_simfs_is_root_path(path))
    {
        buf->st_mode = MTP_S_IFDIR | MTP_S_IRUSR | MTP_S_IRGRP | MTP_S_IROTH |
                       MTP_S_IXUSR | MTP_S_IXGRP | MTP_S_IXOTH;
        buf->st_blksize = MTP_SIMFS_BLOCK_SIZE;
        buf->st_blocks = MTP_SIMFS_SECTOR_COUNT;
        return 0;
    }

    path = mtp_simfs_to_host_path(path, host_path, sizeof(host_path));
    if (path == NULL || stat(path, &st) != 0)
    {
        return -1;
    }

    buf->st_mode = mtp_simfs_to_mtp_mode(st.st_mode);
    buf->st_size = (size_t)st.st_size;
    buf->st_blksize = st.st_blksize ? (size_t)st.st_blksize : MTP_SIMFS_BLOCK_SIZE;
    buf->st_blocks = st.st_blocks ? (size_t)st.st_blocks : (st.st_size ? (((size_t)st.st_size - 1U) / MTP_SIMFS_BLOCK_SIZE + 1U) : 0U);
    return 0;
}

int usbd_mtp_statfs(const char *path, struct mtp_statfs *buf)
{
    char host_path[DFS_PATH_MAX];
    struct statfs st;

    if (buf == NULL)
    {
        return -1;
    }

    path = mtp_simfs_to_host_path(path, host_path, sizeof(host_path));
    if (path == NULL || statfs(path, &st) != 0)
    {
        return -1;
    }

    buf->f_bsize = st.f_bsize;
    buf->f_blocks = st.f_blocks;
    buf->f_bfree = st.f_bfree;
    return 0;
}

int usbd_mtp_open(const char *path, uint8_t mode)
{
    char host_path[DFS_PATH_MAX];
    int flags;

    path = mtp_simfs_to_host_path(path, host_path, sizeof(host_path));
    if (path == NULL || strcmp(path, MTP_SIMFS_MOUNT_POINT) == 0)
    {
        return -1;
    }

    if (mode == MTP_O_RDONLY)
    {
        flags = O_RDONLY;
    }
    else if (mode == MTP_O_WRONLY)
    {
        flags = O_WRONLY | O_CREAT | O_TRUNC;
    }
    else if (mode == MTP_O_RDWR)
    {
        flags = O_RDWR | O_CREAT | O_TRUNC;
    }
    else
    {
        return -1;
    }

    return open(path, flags, 0);
}

int usbd_mtp_close(int fd)
{
    return close(fd);
}

int usbd_mtp_read(int fd, void *buf, size_t len)
{
    return read(fd, buf, len);
}

int usbd_mtp_write(int fd, const void *buf, size_t len)
{
    return write(fd, buf, len);
}

int usbd_mtp_unlink(const char *path)
{
    char host_path[DFS_PATH_MAX];

    path = mtp_simfs_to_host_path(path, host_path, sizeof(host_path));
    if (path == NULL || strcmp(path, MTP_SIMFS_MOUNT_POINT) == 0)
    {
        return -1;
    }

    return unlink(path);
}

void usbd_mtp_mount()
{
    if (mtp_simfs_register_device() != 0)
    {
        return;
    }

    memset(s_storage, 0, sizeof(s_storage));

    (void)mkdir(MTP_SIMFS_MOUNT_POINT, 0);
    (void)dfs_unmount(MTP_SIMFS_MOUNT_POINT);

    if (dfs_mkfs("elm", MTP_SIMFS_DEVICE_NAME) != 0)
    {
        return;
    }

    if (dfs_mount(MTP_SIMFS_DEVICE_NAME, MTP_SIMFS_MOUNT_POINT, "elm", 0, 0) != 0)
    {
        return;
    }

    s_fs_mounted = true;
    (void)mtp_simfs_create_demo_file();
}