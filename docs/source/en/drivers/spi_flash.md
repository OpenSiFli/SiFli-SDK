# Flash

SPI Flash driver includes two layers: hardware access layer (HAL/QSPI) and RT-Thread adaptation layer.<br>
HAL provides basic QSPI APIs to access QSPI peripheral registers. For details, please refer to the API documentation of QSPI HAL.<br>
The adaptation layer provides common SPI-Flash access functionality. Users can use it directly without operating system. It also registers to MTD for RT-Thread, which can be accessed by file system. Flash controller also supports FIFO DMA mode and QSPI mode by default, which can be disabled through configuration header file. Main features include:<br>
- Multi-instance support up to 4
- FIFO DMA support for read/write
- DMA support for flash read
- SPI/QSPI support
- NOR/NAND support
- Sector, block32, block64, full chip erase for NOR and block erase for NAND

## Driver Configuration

Hardware driver can use multiple instances (hardware limited to maximum 4 instances). It can be selected for each project using menuconfig tool and usually saved in C header files. By default, configuration is saved as _rtconfig.h_.

The following example shows flags defined in a project header file where the project enables FLASH controller, SPI-FLASH mode is enabled, using QSPI controller 1 in NOR mode and QSPI2 in NAND mode, and FLASH1 with size 2MB, FLASH2 with 128MB. Steps to select configuration:
- Enter "menuconfig" in the command under project
- Select "RTOS --->"
- Select "On-chip Peripheral Driver--->"	
- Select "Enable QSPI --->"              Use QSPI driver, define macro `BSP_USING_QSPI`
- Select "Enable QSPI Driver"            Use spi flash controller, define macro `BSP_USING_SPI_FLASH`
- Select "QSPI Controller 1 Enable --->" Use QSPI1 controller, define macro `BSP_ENABLE_QSPI1`
- Enter "QSPI1 Mode"                    Select QSPI1 as Nor/Nand flash, define macro `BSP_QSPI1_MODE`
- Enter "QSPI1 Mem Size (MB)"           Set flash1 memory size in MB, define macro `BSP_QSPI1_MEM_SIZE`
- Back to QSPI controller enable:  
- Select "QSPI Controller 2 Enable --->" Use QSPI2 controller, define macro `BSP_ENABLE_QSPI2`
- Enter "QSPI2 Mode："                  Select QSPI2 as Nor/Nand flash, define macro `BSP_QSPI1_MODE`
- Enter "QSPI2 Mem Size (MB)"           Set flash2 memory size in MB, define macro `BSP_QSPI2_MEM_SIZE`

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

If you want to use flash for file system or use it as rt-device, you should enable MTD, which also uses menuconfig tool and is included in header file. Steps to enable RT-DEVICE flash interface (in menuconfig main menu):
- Select "RTOS"—>
- Select "RT-Thread Components--->"
- Select "Device Driver--->"
- Enable "Using MTD Nor Flash device drivers"    Register Nor Flash to MTD device, define macro RT_USING_MTD_NOR
- Select "Enable Nor Flash file system"           Use file system on Flash, define macro RT_USING_NOR_FS
- Enter "Base sector for file system"           Starting address for file system on Flash (by sector)

```c
#define RT_USING_MTD_NOR
#define RT_USING_NOR_FS
#define RT_NOR_FS_BASE_SEC 4096
```
After configuration, users need to include header files in all source code that needs to access the driver.

## Memory Address and Device Names
When using flash as memory, its base address is defined in memory map:
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

FLASH device names registered to RT-DEVICE are fixed. For FLASH1, its device name is "flash1". For FLASH2, its device name is "flash2".

## Non-OS Flash API
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

## Using SPI or Flash

The adapter layer registers hardware support functions requested by RT-Thread and implements these functions using HAL. For users using RT-Thread, the following code can be used as an example (for block device process, its address and size are sector-based):

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

Users can also use flash access with driver interface without system (address and size are byte-based, address should be absolute address), as shown below:

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