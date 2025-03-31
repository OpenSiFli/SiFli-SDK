# Flash

SPI Flash驱动包括两层：硬件访问层（HAL/QSPI）和RT-Thread的适配层。<br>
HAL 提供基本的 QSPI API 来访问 QSPI 外设的寄存器。 有关详细信息，请参阅 QSPI HAL 的 API 文档。<br>
适配层提供常见的 SPI-Flash 访问功能。 用户无需操作系统即可直接使用。 它还为RT-Thread注册到MTD，可以被文件系统访问。 Flash 控制器也默认支持 FIFO DMA 模式和 QSPI 模式，可以通过配置头文件关闭它们。 主要功能包括： <br>
- 多实例支持多达 4 个
- FIFO DMA 支持读/写
- 对闪存读取的 DMA 支持
- SPI/QSPI 支持
- NOR/NAND 支持
- Sector、block32、block64、NOR的全芯片擦除和NAND的块擦除

## 驱动配置

硬件驱动程序可以使用多个实例（硬件限制最多 4 个实例）。可以使用 menuconfig 工具为每个项目选择它，并且通常保存在 C 头文件中。默认情况下，配置保存为 _rtconfig.h_。 

下面的例子显示了一个项目头文件中定义的标志，项目是启用FLASH控制器，SPI-FLASH模式启用，使用QSPI控制器1的NOR模式和QSPI22的NAND模式，以及大小为2MB的FLASH1，使用128MB的FLASH2。步骤选择配置：
- 在项目下的命令中输入“menuconfig”
- 选择“RTOS --->”
- 选择“On-chip Peripheral Driver--->”	
- 选择“Enable QSPI --->”              使用 QSPI 驱动，定义宏`BSP_USING_QSPI`
- 选择“Enable QSPI Driver”            使用 spi flash控制器，定义宏`BSP_USING_SPI_FLASH`
- 选择“QSPI Controller 1 Enable --->” 使用 QSPI1 控制器，定义宏`BSP_ENABLE_QSPI1`
- 输入“QSPI1 Mode”                    选择 QSPI1 为 Nor/Nand flash，定义宏`BSP_QSPI1_MODE`
- 输入“QSPI1 Mem Size (MB)”           以MB为单位设置flash1内存大小，定义宏`BSP_QSPI1_MEM_SIZE``
- 回到 QSPI 控制器启用：  
- 选择“QSPI Controller 2 Enable --->” 使用 QSPI2 控制器r，定义宏`BSP_ENABLE_QSPI2`
- 输入“QSPI2 Mode：”                  选择QSPI2为Nor/Nand flash，定义宏`BSP_QSPI1_MODE`
- 输入“QSPI2 Mem Size (MB)”           以MB为单位设置flash2内存大小，定义宏`BSP_QSPI2_MEM_SIZE`

```c
#define BSP_USING_QSPI
#define BSP_USING_SPI_FLASH
#define BSP_ENABLE_QSPI1
#define BSP_QSPI1_USING_DMA
#define BSP_QSPI1_MODE 0
#define BSP_QSPI1_MEM_SIZE 2
#define BSP_ENABLE_QSPI2
#define BSP_QSPI2_USING_DMA
#define BSP_QSPI2_MODE 1
#define BSP_QSPI2_MEM_SIZE 128
```

如果您想将闪存用于文件系统或将其用作 rt-device，则应启用 MTD，它还使用 menuconfig 工具并包含在头文件中。 启用 RT-DEVICE flash 界面的步骤（在 menuconfig 主菜单中）：
- 选择“RTOS”—>
- 选择“RT-Thread Components--->”
- 选择“Device Driver--->”
- 启用“Using MTD Nor Flash device drivers”    将 Nor Flash 注册到 MTD 设备，定义宏 RT_USING_MTD_NOR
- 选择“Enable Nor Flash file syste”           在 Flash 上使用文件系统，定义宏 RT_USING_NOR_FS
- 输入“Base sector for file system”           Flash 上文件系统的起始地址（按扇区）

```c
#define RT_USING_MTD_NOR
#define RT_USING_NOR_FS
#define RT_NOR_FS_BASE_SEC 4096
```
配置完成后，用户需要在所有需要访问驱动程序的源代码中包含头文件

## 内存地址和设备名称
当使用闪存作为内存时，它的基地址定义在内存映射中：
```c
#define QSPI1_MEM_BASE   (0x10000000)
#define QSPI2_MEM_BASE   (0x64000000)
#define QSPI3_MEM_BASE   (0x68000000)
#define QSPI4_MEM_BASE   (0x12000000)

#define FLASH_BASE_ADDR             (QSPI1_MEM_BASE)

//================== Flash 2 ==================
#define FLASH2_BASE_ADDR            (QSPI2_MEM_BASE)
#ifdef BSP_QSPI2_MEM_SIZE
    #define FLASH2_SIZE                 (BSP_QSPI2_MEM_SIZE*1024*1024)
#else
    #define FLASH2_SIZE                 (0)
#endif

//================== Flash 3 ==================
#define FLASH3_BASE_ADDR            (QSPI3_MEM_BASE)
#ifdef BSP_QSPI3_MEM_SIZE
    #define FLASH3_SIZE                 (BSP_QSPI3_MEM_SIZE*1024*1024)
#else
    #define FLASH3_SIZE                 (0)
#endif

//================== Flash 4 ==================
#define FLASH4_BASE_ADDR            (QSPI4_MEM_BASE)
#ifdef BSP_QSPI4_MEM_SIZE
    #define FLASH4_SIZE                 (BSP_QSPI4_MEM_SIZE*1024*1024)
#else
    #define FLASH4_SIZE                 (0)
#endif

```


注册到 RT-DEVICE 的 FLASH 设备名称是固定的。 对于 FLASH1，它的设备名称是 "flash1" 。 对于 FLASH2，它的设备名称是 "flash2" 。

## 非操作系统闪存 API
```c
/**
 * @brief Read nor-flash memory
 * @param[in] addr: start address for flash memory.
 * @param[out] buf: output data buffer, should not be null.
 * @param[in] size: read memory size, in bytes.
 * @return read size, 0 if fail.
 */
int rt_flash_read(rt_uint32_t addr, rt_uint8_t *buf, size_t size);

/**
 * @brief Write nor-flash memory
 * @param[in] addr: start address for flash memory.
 * @param[in] buf: input data buffer, should not be null.
 * @param[in] size: write memory size, in bytes.
 * @return write size, 0 if fail.
 */
int rt_flash_write(rt_uint32_t addr, const rt_uint8_t *buf, size_t size);

/**
 * @brief erase flash.
 * @param[in] addr: start address for flash memory.
 * @param[in] size: erase memory size, in bytes.
 * @return RT_EOK if success.
 */
rt_err_t rt_flash_erase(rt_uint32_t addr, size_t size);

```

## 使用 SPI 或闪存

适配器层注册 RT-Thread 请求的硬件支持功能，并使用 HAL 实现这些功能。 对于使用 RT-Thread 的用户，可以使用以下代码作为示例（对于块设备进程，它的地址和大小是基于扇区的）：

```c
// Find and open flash device
rt_device_t fdev = rt_device_find("flash1");
rt_err_t err = rt_device_open(fdev, RT_DEVICE_FLAG_RDWR);

// read flash
char * buf = malloc(4096);
int size = rt_device_read(fdev, 0, buf, 1);

// Write flash
// initial buffer, 
size = rt_device_write(fdev, 0, buf, 1);

// Erase flash
unsigned long param[2];
param[0] = 0;
param[1] = 1;
rt_device_control(fdev, RT_DEVICE_CTRL_BLK_ERASE, param);

...

// Close device, keep it open for more user.


```

用户也可以在没有系统的情况下使用带有驱动程序接口的闪存访问（地址和大小是基于字节的，地址应该是绝对地址），如下所示：

```c
// read flash
char * buf = malloc(4096);
unsigned long address = FLASH_BASE_ADDR; 
int size = rt_flash_read(address, buf, 4096);

// Write flash
// initial buffer and address, 
size = rt_flash_write(address, buf, 4096);

// Erase flash
rt_flash_erase(address, 4096);

...

```
