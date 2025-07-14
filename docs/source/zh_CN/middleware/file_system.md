# 文件系统

用户可以在 RT-Thread 上移植和使用文件系统，默认我们启用 ELM-FAT 和 DevFs，对于 DevFs，它默认挂载到 /dev，所有注册的设备都可以显示在此路径上。 
ELM-FAT作为一个胖文件系统，支持文件打开/关闭/读/写、mkdir等常见的文件操作。为了移植ELM-FAT，用户需要实现disk_read、disk_write、disk_ioctrl等磁盘操作 。

ELM-FAT可以使用Nor-Flash、Nand-Flash、SDCARD作为内存盘，它们对磁盘的操作要满足rt-device接口，Nand和Nor flash会注册到MTD设备，SDCARD注册到块设备。 有关文件系统的详细信息，您可以在 RT-Thread 介绍中找到 。


## 文件系统配置

用户可以使用 menuconfig 工具来启用文件系统。 配置通常保存在 C 头文件中。 默认情况下，配置保存为 rtconfig.h 。 

下面的例子显示了在一个项目头文件中定义的标志，该项目使用 ELM-FAT 文件系统并挂载在 NOR-FLASH1 上，它包括 3 部分： 对于 RT-Thread 文件系统配置 ：
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

对于 MTD 设备配置 ：
```c
#define RT_USING_MTD_NOR
#define RT_USING_NOR_FS
#define RT_NOR_FS_BASE_SEC 512
```

对于 FLASH 配置 ：
```c
#define BSP_USING_FLASH
#define BSP_USING_NOR_FLASH
#define BSP_ENABLE_FLASH1
#define BSP_FLASH1_USING_DMA
#define BSP_FLASH1_NOR_MODE
#define BSP_FLASH1_MTD_EN
```

配置完成后，用户需要在所有需要访问文件系统的源代码中包含头文件 。

## 创建文件系统并挂载

配置完成后，文件系统需要创建（mkfs）并挂载 。
```c

// if enable elm, initialize and mount it as soon as possible
elm_init();

// check if file system created before
int res = dfs_mount("flash1", "/", "elm", 0, 0);
if(res != 0) // file system not exist
{
    // create fs
    res = dfs_mkfs("elm","flash1");
	
	// mount fs if create success
	if(res == 0)
	    dfs_mount("flash1", "/", "elm", 0, 0);
}

......

```

## 文件存取功能

文件打开/关闭/读/写操作 。
```c

// file open
int res = dfs_file_open(&src_fd, src, O_RDONLY);
int res2 = dfs_file_open(&fd, dst, O_WRONLY | O_CREAT);

// read file
int read_bytes = dfs_file_read(&src_fd, block_ptr, BUF_SZ);

// write file 
int length = dfs_file_write(&fd, block_ptr, read_bytes);

......

// close file
dfs_file_close(&src_fd);
dfs_file_close(&fd);

```

## 命令行的文件操作
有一些命令可以在命令行上作为普通文件系统使用：ls、copy、mkdir、cat ...

