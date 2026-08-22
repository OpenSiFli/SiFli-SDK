# eMMC/SD卡示例
源码路径:example/rt_device/emmc
## 概述
例程展示了在eMMC或者SD卡上创建分区，在分区上挂载elm文件系统，可对文件系统进行一系列操作
## 支持平台
* sf32lb56-lcd_a128r12n1
* sf32lb56-lcd_n16r12n1
* sf32lb58-lcd_n16r64n4
* sf32lb58-lcd_a128r32n1_dsi

## 使用指南
    emmc应用会在板子的TF卡上创建一个分区，在分区中进行文件系统的挂载，文件系统采用FAT格式，在UART Console可以进行调用常用的文件命令。
```
df               - Disk free//查看文件系统磁盘空间的使用情况
mountfs          - Mount device to file system//将设备挂载到文件系统
mkfs             - Format disk with file system//用文件系统格式化磁盘
mkdir            - Create the DIRECTORY.//创建目录
pwd              - Print the name of the current working directory.//打印当前工作路径
cd               - Change the shell working directory.//切换目录
rm               - Remove(unlink) the FILE(s).//删除文件和目录
cat              - Concatenate FILE(s)//创建文件并写入内容
mv               - Rename SOURCE to DEST.//修改文件名
cp               - Copy SOURCE to DEST.//复制一个文件内容到内容到另一个文件
ls               - List information about the FILEs.//列出所有文件信息
```
## menuconfig配置
```
sdk.py menuconfig --board=56devkit_lcd(board=后面跟着的是板子名称)
```
1. 打开SDIO
![alt text](assets/sdio.png)

2. 如果需要切换到单线模式，需要开启 SDMMC1_BUS_WIDTH_1_ONLY / SDMMC2_BUS_WIDTH_1_ONLY（工程默认不开，4线模式工作）
![alt text](assets/image1.png)

3. 使能和配置sd device
![alt text](assets/sd.png)


### 编译和烧入
按照以下步骤，可以完成编译和烧录。
```
scons --board=56_devkit_lcd -j8
build_56_board_lcd_hcpu\download.bat(uart_download.bat)//可以通过jlink和串口两种烧入方式
```

## 实验结果

1、挂载成功log（如果看到输出打印log中有图片里面框中的那些，就说明挂载成功）
![alt text](assets/log2.png)

2、进行创建文件的操作，首先输入ls查看文件系统中原有的文件或目录，在使用mkdir XXX进行创建，cd到创建好的目录下，进行echo命令出创建文本，并且写入内容，在使用cat命令查看创建好的文件内容，最后使用pwd命令查看一下当前的工作路径
![alt text](assets/log1.png)

## 读写速率测试

例程提供了 4 个命令用于测试 eMMC/SD 卡的读写速率，输出单位为 Mb/s

### 基础命令（固定 512 字节块）

| 命令 | 用法示例 | 说明 |
|------|---------|------|
| `fs_write` | `fs_write /1.txt 4096` | 向文件写入 4096 个 512B 块（2MB），测文件系统写入速度 |
| `fs_read` | `fs_read /1.txt 4096` | 从文件读取 4096 个 512B 块（2MB），测文件系统读取速度 |

参数 `num` 为 512B 块的数量，总数据量 = num × 512B。例如 `fs_write /1.txt 2048` 写入 1MB，`fs_write /1.txt 4096` 写入 2MB。

### 扩展命令（推荐使用）

| 命令 | 用法示例 | 说明 |
|------|---------|------|
| `fs_write_ex` | `fs_write_ex /1.txt 2m` | 写入指定大小的数据，测文件系统写入速度 |
| `fs_read_ex` | `fs_read_ex /1.txt 2m` | 读取指定大小的数据，测文件系统读取速度 |

扩展命令内部自动换算为 512B 块数后委托基础命令执行，提供更友好的单位参数。

#### 参数说明

- `total_size`：支持 `k`/`K`（KB）和 `m`/`M`（MB）单位，如 `512k` `1m` `2m` `4m` `8m`

#### 典型测速流程

```
fs_write_ex /1.txt 2m      ← 先写入 2MB 测试数据
fs_read_ex  /1.txt 2m      ← 再读取 2MB，测读取速度

```

#### 输出示例
* 读写测速结果
```
07-10 15:26:59:328 TX:fs_write_ex /6.txt 2M 
07-10 15:27:02:279    cmd_fs_write_t path=/6.txt num=4096 blocks testtime=2718475.250000uS,speed_test=6.171554Mb/s
07-10 15:27:02:290    msh />msh />
07-10 15:28:14:246 TX:fs_read_ex /6.txt 2M 
07-10 15:28:14:748    cmd_fs_read_t  path=/6.txt num=4096 blocks testtime=496185.312500uS,speed_test=33.812401Mb/s
07-10 15:28:14:757    msh />msh />

```


## 未能按预期完成的结果（log）
![alt text](assets/log3.png)

## 失败的原因和解决方法
1、如果log中显示如下,首先，检查是否插入了TF卡，其次检查上述中的menuconfig里面的SD是否打开，可以参考上面menuconfig的操作配置一下
```
rt_mmcsd_blk_device_create find [sd0] fail !!!
```

2、如果log中显示如下，请检查TF卡是否插入
```
[E/DFS] Device (root) was not found
[E/DFS] Device (misc) was not found
```
如果未能出现预期的log，可以从以下方面进行故障排除：
* 硬件连接是否正常
* 检查USB接口是否松动
* 检查USB线是否具备数据传输功能
* TF卡是否有用
