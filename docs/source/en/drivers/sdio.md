# SDIO

SDIO driver includes two layers: hardware access layer (HAL) and RT-Thread adaptation layer.<br>
HAL provides basic APIs for accessing SDIO peripheral registers. For details, please refer to the API documentation of SDIO HAL.<br>
The adaptation layer provides SDCARD access functionality. After initialization, it registers an SDCARD device that can be accessed by the file system.

## Driver Configuration

The hardware driver is initialized by default as a block device for SDMMC cards. It can be enabled for each project using the menuconfig tool, usually saved in C header files. By default, the configuration is saved as _rtconfig.h_.

The following example shows the flags defined in a project header file that enables SDIO in BSP. For RT-Thread, its SDMMC device also needs to be enabled. Steps to select configuration for BSP:
- Enter "menuconfig" in the command under the project
- Select "RTOS --->"
- Select "On-chip Peripheral Driver--->"	
- Select "Enable SDIO"               Enable SDIO driver, define macro `BSP_USING_SDIO`
```c
#define BSP_USING_SDIO
```
Here shows the RT-Thread SDIO driver configuration:
- Select "RTOS"—>
- Select "RT-Thread Components --->"
- Select "Device Driver--->"
- Select "Using SD/MMC device drivers"
- Select "The stack size for mmcsd thread"   Enter 2048, change macro `RT_MMCSD_STACK_SIZE`, 1024 is too small. Other configurations remain unchanged if not needed.

```c
#define RT_USING_SDIO
#define RT_SDIO_STACK_SIZE 512
#define RT_SDIO_THREAD_PRIORITY 15
#define RT_MMCSD_STACK_SIZE 2048
#define RT_MMCSD_THREAD_PREORITY 22
#define RT_MMCSD_MAX_PARTITION 16
```
After configuration, users need to include header files in all source code that needs to access the driver.

## Using SDMMC

The adapter layer registers hardware support functions requested by RT-Thread and implements these functions using HAL. For users using RT-Thread, the following code can be used as an example:

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