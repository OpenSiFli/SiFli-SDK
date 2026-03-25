# File Systems

RT-Thread allows users to port and utilize various file systems. By default,
ELM-FAT and DevFs are enabled. DevFs is typically mounted to `/dev`, where all
registered devices are visible. As a FAT file system implementation, ELM-FAT
supports standard operations such as opening, closing, reading, writing files,
and creating directories (`mkdir`). To port ELM-FAT, users must implement disk
operations including `disk_read`, `disk_write`, and `disk_ioctrl`.

ELM-FAT can utilize Nor Flash, Nand Flash, or SD cards as storage media. These
media must interface with disk operations via the `rt-device` interface; Nand
and Nor Flash are registered as MTD devices, while SD cards are registered as
block devices. Detailed information regarding the file system can be found in
the RT-Thread documentation.


## File System Configuration

Users can enable the file system using the `menuconfig` tool. Configurations are
typically stored in a C header file, which by default is saved as `rtconfig.h`.

The following example demonstrates macro definitions in a project header file
for an ELM-FAT file system mounted on `NOR-FLASH1`. The configuration consists
of three parts. For the RT-Thread file system configuration:
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

For MTD device configuration:
```c
#define RT_USING_MTD_NOR
#define RT_USING_NOR_FS
#define RT_NOR_FS_BASE_SEC 512
```

For FLASH configuration:
```c
#define BSP_USING_FLASH
#define BSP_USING_NOR_FLASH
#define BSP_ENABLE_FLASH1
#define BSP_FLASH1_USING_DMA
#define BSP_FLASH1_NOR_MODE
#define BSP_FLASH1_MTD_EN
```

After configuration, the header file must be included in all source files that
require file system access.

## Creating and Mounting the File System

Once configured, the file system must be initialized, formatted (`mkfs`), and
mounted.
```c
// If ELM is enabled, initialize and mount it as early as possible
elm_init();

// Check if the file system was previously created
int res = dfs_mount("flash1", "/", "elm", 0, 0);
if(res != 0) // File system does not exist
{
    // Create the file system
    res = dfs_mkfs("elm","flash1");

	// Mount the file system if creation was successful
	if(res == 0)
	    dfs_mount("flash1", "/", "elm", 0, 0);
}

......
```

## File Access Functions

Standard file operations: open, close, read, and write.
```c
// Open files
int res = dfs_file_open(&amp;src_fd, src, O_RDONLY);
int res2 = dfs_file_open(&amp;fd, dst, O_WRONLY | O_CREAT);

// Read from file
int read_bytes = dfs_file_read(&amp;src_fd, block_ptr, BUF_SZ);

// Write to file 
int length = dfs_file_write(&amp;fd, block_ptr, read_bytes);

......

// Close files
dfs_file_close(&amp;src_fd);
dfs_file_close(&amp;fd);
```

## Command-Line File Operations
Standard file system commands are available via the command line (FinSH),
including `ls`, `copy`, `mkdir`, and `cat`.

