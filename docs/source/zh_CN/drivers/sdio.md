# SDIO

SDIO驱动包括两层：硬件访问层（HAL）和RT-Thread的适配层。<br>
HAL 提供用于访问 SDIO 外设寄存器的基本 API。 有关详细信息，请参阅 SDIO HAL 的 API 文档。<br>
适配层提供 SDCARD 访问功能。 初始化后注册一个SDCARD设备，可以被文件系统访问。 

## 驱动配置

硬件驱动程序默认初始化为 SDMMC 卡的块设备。 它可以使用 menuconfig 工具为每个项目启用，通常保存在 C 头文件中。 默认情况下，配置保存为 _rtconfig.h_。

以下示例显示了在一个项目头文件中定义的标志，该项目在 BSP 中启用 SDIO。 对于 RT-Thread，它的 SDMMC 设备也需要启用。 为 BSP 选择配置的步骤：
- 在项目下的命令中输入“menuconfig”
- 选择“RTOS --->”
- 选择“On-chip Peripheral Driver--->”	
- 选择“Enable SDIO”               启用SDIO驱动，定义宏`BSP_USING_SDIO`
```c
#define BSP_USING_SDIO
```
这里显示了 RT-Thread SDIO 驱动程序配置:
- 选择“RTOS”—>
- 选择“RT-Thread Components --->”
- 选择“Device Driver--->”
- 选择“Using SD/MMC device drivers ”
- 选择“The stack size for mmcsd thread”   输入 2048, 更改宏 `RT_MMCSD_STACK_SIZE`，1024 太小。 如果不需要，其他配置不改变。

```c
#define RT_USING_SDIO
#define RT_SDIO_STACK_SIZE 512
#define RT_SDIO_THREAD_PRIORITY 15
#define RT_MMCSD_STACK_SIZE 2048
#define RT_MMCSD_THREAD_PREORITY 22
#define RT_MMCSD_MAX_PARTITION 16
```
配置完成后，用户需要在所有需要访问驱动程序的源代码中包含头文件。

## 使用 SDMMC

适配器层注册 RT-Thread 请求的硬件支持功能，并使用 HAL 实现这些功能。 对于使用 RT-Thread 的用户，可以使用以下代码作为示例：

```c
// Find and open sdcard as block device
rt_device_t dev = rt_device_find("sd0");    // get block device
rt_err_t err = rt_device_open(dev, RT_DEVICE_FLAG_RDWR);

// read sdmmc, it's a block device, read size should be block based(default as 512)
char * buf = malloc(4096);
blk = len >> 9;
int size = rt_device_read(dev, addr, (void *)buf, blk);

// Write sdmmc
size = rt_device_write(dev, addr, buf, blk);


...

// Close block device
rt_device_close(dev);

```

