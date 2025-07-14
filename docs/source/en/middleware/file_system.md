
# File System

Users can port and use a file system on RT-Thread. By default, we enable ELM-FAT and DevFs. For DevFs, it is mounted by default at /dev, and all registered devices will be visible at this path. 

ELM-FAT, as a FAT file system, supports common file operations such as open/close/read/write, mkdir, etc. To port ELM-FAT, users need to implement disk operations such as disk_read, disk_write, disk_ioctrl, and others.

ELM-FAT can use Nor-Flash, Nand-Flash, and SDCARD as memory disks. These memory devices need to meet the rt-device interface. Nand and Nor Flash are registered to the MTD devices, while SDCARD is registered to block devices. Detailed information about the file system can be found in the RT-Thread documentation.

## File System Configuration

Users can enable the file system using the `menuconfig` tool. The configuration is usually saved in a C header file. By default, configurations are saved in `rtconfig.h`.

The following example shows the flags defined in a project's header file, where the project uses the ELM-FAT file system and mounts it on NOR-FLASH1. It includes three parts: RT-Thread file system configuration:

```c
#define RT_DFS_ELM_CODE_PAGE 437
#define RT_DFS_ELM_WORD_ACCESS
#define RT_DFS_ELM_USE_LFN_3
#define RT_DFS_ELM_USE_LFN 3
#define RT_DFS_ELM_MAX_LFN 255
#define RT_DFS_ELM_DRIVES 2
#define RT_DFS_ELM_MAX_SECTOR_SIZE 4096
#define RT_DFS_ELM_REENTRANT
#define RT_USING_DFS_DEVFS
```

MTD device configuration:

```c
#define RT_USING_MTD_NOR
#define RT_USING_NOR_FS
#define RT_NOR_FS_BASE_SEC 512
```

FLASH configuration:

```c
#define BSP_USING_FLASH
#define BSP_USING_NOR_FLASH
#define BSP_ENABLE_FLASH1
#define BSP_FLASH1_USING_DMA
#define BSP_FLASH1_NOR_MODE
#define BSP_FLASH1_MTD_EN
```

After completing the configuration, users need to include the header files in all source files that need to access the file system.

## Creating and Mounting the File System

After configuration, the file system needs to be created (mkfs) and mounted.

```c
// If ELM is enabled, initialize and mount it as soon as possible
elm_init();

// Check if the file system exists
int res = dfs_mount("flash1", "/", "elm", 0, 0);
if(res != 0) // File system does not exist
{
    // Create the file system
    res = dfs_mkfs("elm", "flash1");

    // Mount the file system if creation is successful
    if(res == 0)
        dfs_mount("flash1", "/", "elm", 0, 0);
}
......
```

## File Access Functions

File open/close/read/write operations.

```c
// File open
int res = dfs_file_open(&src_fd, src, O_RDONLY);
int res2 = dfs_file_open(&fd, dst, O_WRONLY | O_CREAT);

// Read file
int read_bytes = dfs_file_read(&src_fd, block_ptr, BUF_SZ);

// Write file
int length = dfs_file_write(&fd, block_ptr, read_bytes);
......

// Close file
dfs_file_close(&src_fd);
dfs_file_close(&fd);
```

## Command Line File Operations

Some commands can be used on the command line as a normal file system: ls, copy, mkdir, cat, etc.
