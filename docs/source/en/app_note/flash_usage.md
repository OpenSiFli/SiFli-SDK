# Flash Usage Guide

## 1. Introduction to FLASH
The SiFLi FLASH controller supports multiple FLASH chips simultaneously (the 55X series supports 4, the 58X series supports 5, and the 56X series supports 4). These chips can be either NOR-FLASH or NAND-FLASH.
Compared to the 55X series, the 58X and 56X series controllers have added support for 8-line and 16-line PSRAM. The controller name has been changed from QSPI to MPI, meaning it is a controller dedicated to Memory.
For the 55X series, controller one by default controls the internal NOR-FLASH, while controllers two and three are determined by the specific board solution, and controller four is connected to the NOR FLASH, which can be accessed by the LCPU (HCPU can also access).
For the 58X series, controllers one and two typically control the internal NOR-FLASH or PSRAM, depending on the chip package. Controllers three and four are determined by the specific board solution, and controller five connects to internal NOR FLASH, which can be accessed by the LCPU (HCPU can also access).
For the 56X series, controllers one and two typically control the internal NOR-FLASH or PSRAM, depending on the chip package. Controller three is determined by the specific board solution, and controller five connects to internal NOR FLASH, which can be accessed by the LCPU (HCPU can also access). (There is no controller four).
The FLASH driver is divided into the HAL and DRV layers. The HAL layer provides hardware register access and simple logical interfaces, while the DRV layer provides basic operations such as FLASH read/write/erase, initialization, control, and device and file system operations.
- The controller can provide single-wire FLASH operation and 4-wire FLASH operation.
- Internal DMA is used for acceleration.
- Supports PAGE write, supports PAGE read (NOR-FLASH can be read directly via the AHB bus in memory mode).
- Supports SECTOR/BLOCK erasure of different sizes. The basic operation unit size depends on the chip.

## 2. FLASH Configuration
The properties and usage of FLASH chips corresponding to the FLASH controller can be configured via `menuconfig` (rtconfig.h), including whether to enable it, whether to use DMA, whether to connect MTD devices, and file system information, etc.
Here is an example of some functions on the 55X EVB development board:
Enter "menuconfig" in the command line to start configuration:
- Choose "RTOS --->"
- Choose "On-chip Peripheral Driver ---> "
- Choose "Enable QSPI ---> "
- Choose "Enable QSPI Driver"
- Choose "QSPI Controller 1 Enable --->" 
- Choose "QSPI1 Mode: 0 NOR, 1 NAND, 2 PSRAM, 3 OTHERS"
- Choose "QSPI1 Use File System ---"
- Choose "QSPI1 Mem Size()MB"
- Choose "QSPI1 Manul & Device ID"

Below are examples of the configuration for each FLASH controller and their corresponding macro definitions.<br/>

### FLASH1: (FLASH1 defaults to NOR-FLASH, mode cannot be modified)
```c
#define BSP_USING_QSPI 1            /* Enable QSPI module */
#define BSP_USING_SPI_FLASH 1       /* Enable QSPI FLASH functionality (for backward compatibility) */
#define BSP_ENABLE_QSPI1 1          /* Enable QSPI controller 1 */
#define BSP_QSPI1_USING_DMA 1       /* Use FIFO DMA for QSPI1 */
#define BSP_QSPI1_MODE 0            /* QSPI1 connected to NOR FLASH */
#define BSP_QSPI1_MEM_SIZE 4        /* Configure FLASH1 total size to 4MB */
#define BSP_QSPI1_CHIP_ID 0         /* FLASH1 DEV ID automatically obtained by the system (no manual input required) */
```

### FLASH2:
```c
#define BSP_ENABLE_QSPI2 1          /* Enable QSPI2 module */
#define BSP_QSPI2_USING_DMA 1       /* Enable FIFO DMA for QSPI2 */
#define BSP_QSPI2_MODE 0            /* Set FLASH2 to NOR-FLASH */
#define BSP_QSPI2_MTD_EN 1          /* Register FLASH2 as MTD device */
#define BSP_QSPI2_FS_START 2048     /* The start address for FLASH2 MTD device is 8MB (2048*4K) */
#define BSP_QSPI2_FS_SIZE 2048      /* Length is 8MB (2048*4KB) */
#define BSP_QSPI2_MEM_SIZE 32       /* Configure FLASH2 total size to 32MB */ 
#define BSP_QSPI2_CHIP_ID 0         /* FLASH2 DEV ID automatically obtained by the system */
```

### FLASH3:
```c
#define BSP_ENABLE_QSPI3 1          /* Enable QSPI3 module */
#define BSP_QSPI3_USING_DMA 1       /* Enable FIFO DMA for QSPI3 */
#define BSP_QSPI3_MODE 1            /* Set FLASH3 to NAND-FLASH */
#define BSP_QSPI3_MEM_SIZE 128      /* Configure FLASH3 total size to 128MB */
#define BSP_QSPI3_CHIP_ID 0         /* FLASH3 DEV ID automatically obtained by the system */
```

Below is an example for some functions on the 58X EVB development board:
Enter "menuconfig" in the command line to start configuration:
- Choose "RTOS --->"
- Choose "On-chip Peripheral Driver ---> "	
- Choose "Enable MPI  --->	"
- Choose "Enable QSPI Driver "
- Choose "MPI Controller 1 Enable --->" 
- Choose "MPI Mode: 0 NOR, 1 NAND, 2 PSRAM, 3 OPSRAM, 4 HPSRAM, 5 LEGACY_PSRAM "
- Choose "MPI1 Mem Size()MB"

Below are examples of the configuration for each FLASH controller and their corresponding macro definitions.<br/>


### MPI: (MPI default settings, some configurations for backward compatibility)
```c
#define BSP_USING_MPI 1             /* Enable MPI module */
#define BSP_USING_SPI_FLASH 1       /* Enable MPI FLASH functionality (for backward compatibility) */
```

### MPI1: (FLASH1 defaults to NOR-FLASH, mode cannot be modified)
```c
#define BSP_ENABLE_MPI1 1           /* Enable MPI 1 module */
#define BSP_ENABLE_QSPI1 1          /* Enable MPI 1 (for backward compatibility) */
#define BSP_MPI1_MODE_4 1           /* Set MPI1 to function 4---16-line PSRAM */
#define BSP_QSPI1_MODE 4
#define BSP_USING_PSRAM1 1
#define BSP_QSPI1_MEM_SIZE 32       /* Configure MPI1 total size to 16MB */
```

### MPI2:
```c
#define BSP_ENABLE_MPI2 1           /* Enable MPI 2 module */
#define BSP_ENABLE_QSPI2 1          /* Enable MPI 2 (for backward compatibility) */
#define BSP_MPI2_MODE_4 1           /* Set MPI2 to function 4---16-line PSRAM */
#define BSP_QSPI2_MODE 4
#define BSP_USING_PSRAM2 1
#define BSP_QSPI2_MEM_SIZE 32       /* Configure MPI2 total size to 32MB */
```

### MPI3:
```c
#define BSP_ENABLE_MPI3 1           /* Enable MPI 3 module */
#define BSP_ENABLE_QSPI3 1          /* Enable MPI 3 (for backward compatibility) */
#define BSP_MPI3_MODE_0 1           /* Set MPI3 to function 0---NOR FLASH */
#define BSP_QSPI3_MODE 0
#define BSP_USING_NOR_FLASH3 1
#define BSP_QSPI3_USING_DMA 1       /* Use DMA for MPI3 */
#define BSP_QSPI3_MEM_SIZE 32       /* Configure MPI2 total size to 32MB */
```

### MPI4:
```c
#define BSP_ENABLE_MPI4 1           /* Enable MPI 4 module */
#define BSP_ENABLE_QSPI4 1          /* Enable MPI 4 (for backward compatibility) */
#define BSP_MPI4_MODE_1 1           /* Set MPI4 to function 1---NAND FLASH */
#define BSP_QSPI4_MODE 1
#define BSP_USING_NAND_FLASH4 1
#define BSP_QSPI4_USING_DMA 1       /* Use DMA for MPI5 */
#define BSP_QSPI4_MEM_SIZE 128       /* Configure MPI2 total size to 128MB */
```

### MPI5:
```c
#define BSP_ENABLE_MPI5 1           /* Enable MPI 5 module */
#define BSP_ENABLE_QSPI5 1          /* Enable MPI 5 (for backward compatibility) */
#define BSP_MPI5_MODE_0 1           /* Set MPI5 to function 0---NOR FLASH */
#define BSP_QSPI5_MODE 0
#define BSP_USING_NOR_FLASH5 1
#define BSP_QSPI5_USING_DMA 1       /* Use DMA for MPI5 */
#define BSP_QSPI5_MEM_SIZE 4       /* Configure MPI5 total size to 4MB */
```

The 56X configuration is mostly similar to the 58X series except for the absence of MPI4. Therefore, further examples will not be listed separately.<br/>

## 3. FLASH Address Space

### 55X Series Addresses
The default FLASH size configuration on the EVB, where the size represents the usable space, is described according to how the development board is configured:

| Name             | Start Address   | End Address     | Max Size (bytes)  | Description           |
|------------------|-----------------|-----------------|-------------------|-----------------------|
| FLASH1           | 0x10000000      | 0x11FFFFFF      | 32 * 1024 * 1024  | EVB capacity: 4MB    |
| FLASH2           | 0x64000000      | 0x67FFFFFF      | 64 * 1024 * 1024  | EVB capacity: 32MB   |
| FLASH3           | 0x68000000      | 0x6FFFFFFF      | 128 * 1024 * 1024 | EVB disabled         |
| FLASH4           | 0x12000000      | 0x13FFFFFF      | 32 * 1024 * 1024  | EVB disabled         |

The base addresses for the four FLASH controllers can be found in `_mem_map.h`. The actual size used is determined by the `menuconfig` based on the connected chips:

```c
#define QSPI1_MEM_BASE   (0x10000000)
#define QSPI2_MEM_BASE   (0x64000000)
#define QSPI3_MEM_BASE   (0x68000000)
#define QSPI4_MEM_BASE   (0x12000000)
```

### 58X Series Addresses
The default FLASH size configuration on the EVB, where the size represents the usable space, is described according to how the development board is configured:

C-BUS Addresses:
| Name             | Start Address   | End Address     | Max Size (bytes)  | Description           |
|------------------|-----------------|-----------------|-------------------|-----------------------|
| MPI1             | 0x10000000      | 0x11FFFFFF      | 32 * 1024 * 1024  | EVB capacity: 16MB   |
| MPI2             | 0x12000000      | 0x13FFFFFF      | 32 * 1024 * 1024  | EVB capacity: 16MB   |
| MPI3             | 0x14000000      | 0x17FFFFFF      | 64 * 1024 * 1024  | EVB capacity: 32MB   |
| MPI4             | 0x18000000      | 0x1BFFFFFF      | 64 * 1024 * 1024  | EVB capacity: 64MB   |
| MPI5             | 0x1C000000      | 0x1FFFFFFF      | 64 * 1024 * 1024  | EVB capacity: 4MB    |

S-BUS Addresses:
| Name             | Start Address   | End Address     | Max Size (bytes)  | Description           |
|------------------|-----------------|-----------------|-------------------|-----------------------|
| MPI1             | 0x60000000      | 0x61FFFFFF      | 32 * 1024 * 1024  | EVB capacity: 16MB   |
| MPI2             | 0x62000000      | 0x63FFFFFF      | 32 * 1024 * 1024  | EVB capacity: 16MB   |
| MPI3             | 0x64000000      | 0x67FFFFFF      | 64 * 1024 * 1024  | EVB capacity: 32MB   |
| MPI4             | 0x68000000      | 0x9FFFFFFF      | 896 * 1024 * 1024 | EVB capacity: 64MB   |

To allow NAND FLASH to use more than 64MB, it is recommended to mount it on MPI4 and start from the address 0x68000000.

The five MPI controller base addresses can be found in `_mem_map.h`. The actual size used is determined by the `menuconfig` based on the connected chips:

```c
#define QSPI1_MEM_BASE   (0x10000000)
#define QSPI2_MEM_BASE   (0x12000000)
#define QSPI3_MEM_BASE   (0x14000000)
#define QSPI4_MEM_BASE   (0x18000000)
#define QSPI5_MEM_BASE   (0x1C000000)
```

### 56X Series Addresses
The default FLASH size configuration on the EVB, where the size represents the usable space, is described according to how the development board is configured:

C-BUS Addresses:
| Name             | Start Address   | End Address     | Max Size (bytes)  | Description           |
|------------------|-----------------|-----------------|-------------------|-----------------------|
| MPI1             | 0x10000000      | 0x107FFFFF      | 8 * 1024 * 1024   | EVB capacity: 4MB    |
| MPI2             | 0x10800000      | 0x13FFFFFF      | 56 * 1024 * 1024  | EVB capacity: 16MB   |
| MPI3             | 0x14000000      | 0x17FFFFFF      | 128 * 1024 * 1024 | EVB capacity: 128MB  |
| MPI5             | 0x1C000000      | 0x1FFFFFFF      | 64 * 1024 * 1024  | EVB capacity: 1MB    |

S-BUS Addresses:
| Name             | Start Address   | End Address     | Max Size (bytes)  | Description           |
|------------------|-----------------|-----------------|-------------------|-----------------------|
| MPI1             | 0x60000000      | 0x607FFFFF      | 8 * 1024 * 1024   |                       |
| MPI2             | 0x60800000      | 0x63FFFFFF      | 56 * 1024 * 1024  |                       |
| MPI3             | 0x64000000      | 0x9FFFFFFF      | 960 * 1024 * 1024 |                       |

The four MPI controller base addresses can be found in `_mem_map.h`. The actual size used is determined by the `menuconfig` based on the connected chips:

```c
#define QSPI1_MEM_BASE   (0x10000000)
#define QSPI2_MEM_BASE   (0x10800000)
#define QSPI3_MEM_BASE   (0x14000000)
#define QSPI5_MEM_BASE   (0x1C000000)
```

## 4. FLASH RT-Thread Driver Access
Based on RT-THREAD, the FLASH provides a set of access interfaces that can be operated by address (the absolute address determines which controller to use).

```c
/** 
* @brief  Flash controller hardware initialization.
* @retval 0 if success.

int rt_hw_flash_init(void);

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

The initialization is carried out through the interface `rt_hw_flash_init`. Before this, whether each FLASH is enabled and the corresponding configuration have been determined by _rtconfig.h_, and will not be modified through input parameters here. <br/>
The data operation is performed through `rt_flash_read`. NOR-FLASH can be read directly from memory, while NAND-FLASH can only be read through this interface. The caller needs to maintain the management of the data buffer by himself. <br/>
The data write operation is performed through `rt_flash_write`. For both NAND and NOR, the write operation needs to be completed through this interface, and the caller needs to maintain the data buffer. <br/>
For NOR-FLASH, the smallest erase unit is SECTOR (usually 4KB). The address and length must be aligned to SECTOR; otherwise, incorrect erasure will occur. The caller needs to maintain the recovery of misaligned address data. <br/>
For NAND-FLASH, the minimum erase unit is BLOCK (usually 128KB), and the address and length need to be aligned according to BLOCK. The caller needs to maintain the data backup and recovery of the unaligned part by himself. <br/>

```{note}
Note that FLASH does not support writing to the same Flash when XIP is not supported, so the Flash driver related code needs to be placed in SRAM for execution. The following is a link script fragment of Keil,
The 64KB space starting at address 0x200E1000 is used to store the Flash driver code that needs to be executed in SRAM. You can modify the link script accordingly
```

```
LR_IROM1 0x10020000 0x100000  {    ; load region size_region
  ...
  ER_IROM1_EX 0x200E1000 0x10000  {  ; Flash code and RO need to put in SRAM
   drv_flash_z0.o (.text.*)
   drv_flash_z0.o (.rodata.*)
   bf0_hal_flash.o (.text.*)
   bf0_hal_flash_ext_z0.o (.text.*)
   ...
  } 
  ...
}
```

The following is an example of writing 4KB data to FLASH1 1MB address:
```c

unsigned long address = FLASH_BASE_ADDR + 0x100000; 
char * buf = NULL;
char * buf2 = NULL;

// Erase flash
int res = rt_flash_erase(address, 4096);
if(res != 0)
	goto err;


// Write flash
// malloc buf and initial data before write
buf = malloc(4096);
if(buf == NULL)
    return ERROR;
int size = rt_flash_write(address, buf, 4096);
if(size != 4096)
    goto err;

// read flash
buf2 = malloc(4096);
if(buf2 == NULL)
    goto err;
size = rt_flash_read(address, buf2, 4096);
if(size != 4096)
    goto err;
	
// check data 
for(int i=0; i<4096; i++)
    if(buf[i] != buf2[i])
	    goto err;

....
		
err:
    if(buf)
	    free(buf);
	if(buf2)
	    free(buf);
return ERROR;

...

```

## 5. FLASH RT-Thread MTD device is used
The MTD in RT-THREAD contains two device drivers, MTD-NOR and MTD-NAND. If the FLASH is correctly configured and enabled in the MTD of the configuration file, these devices will be automatically registered during system startup. Whether the registration is successful or not can be queried through the interface. <br/>
For example, the query of FLASH1 can be carried out by rt_device_find("flash1"). The device names corresponding to each controller are "flash1", "flash2", and "flash3". If the query returns non-empty, it means that the registration is successful. <br/>

At present, MTD devices have been used in file systems and USB STORAGE. In use, the device is mainly searched and then bound. The read and write of MTD has been encapsulated in mtd_nor, so users do not need to process it. <br/>
The following are the corresponding data types when registering. If the user registers himself, he needs to fill in the corresponding data and implement the operation interface. If the registration is completed, he only needs to find the device, and the subsequent operation will be completed by the system background. <br/>
MTD-NOR:
```c
struct rt_mtd_nor_device
{
	struct rt_device parent;

	rt_uint32_t block_size;			/* The Block size in the flash */
	rt_uint32_t block_start;		/* The start of available block*/
	rt_uint32_t block_end;			/* The end of available block */

	/* operations interface */
	const struct rt_mtd_nor_driver_ops* ops;
};

struct rt_mtd_nor_driver_ops
{
	rt_uint32_t (*read_id) (struct rt_mtd_nor_device* device);

	rt_size_t (*read)    (struct rt_mtd_nor_device* device, rt_off_t offset, rt_uint8_t* data, rt_uint32_t length);
	rt_size_t (*write)   (struct rt_mtd_nor_device* device, rt_off_t offset, const rt_uint8_t* data, rt_uint32_t length);

	rt_err_t (*erase_block)(struct rt_mtd_nor_device* device, rt_off_t offset, rt_uint32_t length);
};

rt_err_t rt_mtd_nor_register_device(const char* name, struct rt_mtd_nor_device* device);
```

MTD-NAND:
```c
struct rt_mtd_nand_device
{
    struct rt_device parent;

    rt_uint16_t page_size;          /* The Page size in the flash */
    rt_uint16_t oob_size;           /* Out of bank size */
    rt_uint16_t oob_free;           /* the free area in oob that flash driver not use */
    rt_uint16_t plane_num;          /* the number of plane in the NAND Flash */

    rt_uint32_t pages_per_block;    /* The number of page a block */
    rt_uint16_t block_total;

    rt_uint32_t block_start;        /* The start of available block*/
    rt_uint32_t block_end;          /* The end of available block */

    /* operations interface */
    const struct rt_mtd_nand_driver_ops* ops;
};

struct rt_mtd_nand_driver_ops
{
    rt_uint32_t (*read_id) (struct rt_mtd_nand_device* device);

    rt_err_t (*read_page)(struct rt_mtd_nand_device* device,
                          rt_off_t page,
                          rt_uint8_t* data, rt_uint32_t data_len,
                          rt_uint8_t * spare, rt_uint32_t spare_len);

    rt_err_t (*write_page)(struct rt_mtd_nand_device * device,
                           rt_off_t page,
                           const rt_uint8_t * data, rt_uint32_t data_len,
                           const rt_uint8_t * spare, rt_uint32_t spare_len);
    rt_err_t (*move_page) (struct rt_mtd_nand_device *device, rt_off_t src_page, rt_off_t dst_page);

    rt_err_t (*erase_block)(struct rt_mtd_nand_device* device, rt_uint32_t block);
    rt_err_t (*check_block)(struct rt_mtd_nand_device* device, rt_uint32_t block);
    rt_err_t (*mark_badblock)(struct rt_mtd_nand_device* device, rt_uint32_t block);
};

rt_err_t rt_mtd_nand_register_device(const char* name, struct rt_mtd_nand_device* device);
```

Here is an example of MTD-NOR in use with a file system:
```c
    // Register MTD DEVICE
    struct rt_mtd_nor_device *nod = malloc(sizeof(struct rt_mtd_nor_device));

    nod->block_start = BSP_FLASH1_FS_START;
    nod->block_size = FLASH_SECT_SIZE;
    nod->block_end = nod->block_start + BSP_FLASH1_FS_SIZE;
    nod->ops = &flash_ops;
    nod->parent.user_data = &lflash_handle[0];
    lflash_handle[0].nod = nod;

    rt_mtd_nor_register_device("flash1", (struct rt_mtd_nor_device *)(lflash_handle[0].nod));
	
	// Find MTD DEVICE:
	rt_device_t dev = rt_device_find("flash1");
    if (dev) // device find, it has beed registered to mtd
    {
        // With MTD devices, mkfs and mount directly use the device name to find the device and call the read/write interface of the device
        if (dfs_mkfs("elm", "flash1") == 0)
        {
          dfs_mount("flash1", "/", "elm", 0, 0);
		  ....
        }
    }
```

## 6. Current FLASH space allocation status

From the space listed in _mem_map.h_, the first 64KB from the start of FLASH1 (0x10000000) is used for storing the flash_table, followed by another 64KB (0x10010000) for storing the boot patch, and starting from 0x10020000 for storing bin. <br/>
FLASH2 If NOR-FLASH is the case, the watch resource file is stored from its starting position (0x64000000).
```c
#define FLASH_TABLE_SIZE            (20*1024)
#define FLASH_CAL_TABLE_SIZE        (8*1024)
#define FLASH_BOOT_PATCH_SIZE       (64*1024)

#define FLASH_BASE_ADDR             (0x10000000)
#define FLASH_TABLE_START_ADDR      (FLASH_BASE_ADDR)
#define FLASH_TABLE_END_ADDR        (END_ADDR(FLASH_TABLE_START_ADDR, FLASH_TABLE_SIZE))
#define FLASH_CAL_TABLE_START_ADDR  (FLASH_TABLE_END_ADDR+1)
#define FLASH_BOOT_PATCH_START_ADDR (0x10010000)
#define FLASH_BOOT_PATCH_END_ADDR   (END_ADDR(FLASH_BOOT_PATCH_START_ADDR, FLASH_BOOT_PATCH_SIZE)) /* 0x1001FFFF */
#define FLASH_USER_CODE_START_ADDR   (FLASH_BOOT_PATCH_END_ADDR + 1)   /* 0x10020000 */
```

FLASH is also used for storing variables and logs. Currently, FLASHDB (or EasyFlash) is used. flash1/flash2 have some space temporarily used, which needs to be replanned according to system requirements.

## 7. NOR-FLASH's Transplant

According to different development boards for different customers, different NOR-FLASH particles will be used in the design, and when using these particles, the main matching is made from hardware specifications and operation command configuration.<br/>

### Hardware specifications
- Voltage configuration: 3.3V or 1.8V
- SPI mode: whether to support double line and 4-line SPI in addition to single line
- Highest frequency: The highest frequency supported in single line/double line/4 line mode respectively
- Other things such as working temperature, writing speed, etc., which are not related to software porting, can be ignored first

### operating command 

Operation commands mainly refer to read/write/erase related granular commands, which are related to vendors and specific granules, and need to be found and compared from the specification book.<br/>
A0 has added the way to add the command table to increase the support for new FLASH particles (_flash_table.c_). The main commands involved are defined in _bf0_hal_qspi.h SPI_FLASH_CMD_E_, which includes NAND/NOR commands.<br/>

```c
/**
  * @brief  SPI_FLASH command index
  */
typedef enum
{
    SPI_FLASH_CMD_WREN = 0,  /*!<  write enable, nor+nand    */
    SPI_FLASH_CMD_WRDI,     /*!<  write disable, nor+nand    */
    SPI_FLASH_CMD_RDSR,     /*!<  read status register, nor+nand    */
    SPI_FLASH_CMD_WRSR,     /*!<  write status register, nor+nand    */
    SPI_FLASH_CMD_PREAD,    /*!<  page read, nand    */
    SPI_FLASH_CMD_READ,     /*!<  single line read, nor+nand    */
    SPI_FLASH_CMD_FREAD,    /*!<  fast read , nor + nand    */
    SPI_FLASH_CMD_DREAD,    /*!<  fast read dual output, nor+nand    */
    SPI_FLASH_CMD_QREAD,    /*!<  fast read quad output, nor+nand    */
    SPI_FLASH_CMD_2READ,    /*!<  2 line read, nor+nand    */
    SPI_FLASH_CMD_4READ,   /*!<  4 line read, nor+nand .   ==== 10    */
    SPI_FLASH_CMD_RDID,    /*!<  read id, nor+nand    */
    SPI_FLASH_CMD_PLD,     /*!<  load program data, nand    */
    SPI_FLASH_CMD_QPLD,    /*!<  qual program load, nand    */
    SPI_FLASH_CMD_PLDR,    /*!<  randome program load, nand    */
    SPI_FLASH_CMD_QPLDR,   /*!<  qual random program load, nand    */
    SPI_FLASH_CMD_PEXE,    /*!<  program execute, nand    */
    SPI_FLASH_CMD_BE,      /*!<  block erase, nand    */
    SPI_FLASH_CMD_RST,     /*!<  reset, nor+nand    */
    SPI_FLASH_CMD_RST_EN,     /*!<  reset en, nor    */
    SPI_FLASH_CMD_RDSR2,   /*!<  read status register 2, nor       ==== 20    */
    SPI_FLASH_CMD_WVSR,    /*!<  write volatile status register, nor    */
    SPI_FLASH_CMD_PP,      /*!<  PAGE PROGRAM, nor    */
    SPI_FLASH_CMD_QPP,     /*!<  QUAL PAGE PROGRAM, nor    */
    SPI_FLASH_CMD_RDEAR,     /*!<  read extended address register, nor    */
    SPI_FLASH_CMD_WREAR,     /*!<  write extended address register, nor    */
    SPI_FLASH_CMD_PE,        /*!<  page erase, nor    */
    SPI_FLASH_CMD_SE,        /*!<  SECTOR erase, nor    */
    SPI_FLASH_CMD_BE32,        /*!<  BLOCK erase 32KB, nor    */
    SPI_FLASH_CMD_BE64,        /*!<  BLOCK erase 64KB, nor    */
    SPI_FLASH_CMD_CE,        /*!<  CHIP ERASE, nor             ===== 30    */
    SPI_FLASH_CMD_RDSR3,   /*!<  read status register 3, nor    */
    SPI_FLASH_CMD_WRSR3,   /*!<  WRITE status register 3, nor    */
    SPI_FLASH_CMD_EN4BM,   /*!<  enter 4-byte address mode, nor    */
    SPI_FLASH_CMD_ET4BM,   /*!<  exit 4-byte address mode, nor    */
    SPI_FLASH_CMD_RD4BA,   /*!<  read with 4-byte address, nor    */
    SPI_FLASH_CMD_FR4BA, /*!<  fast read with 4-byte address, nor    */
    SPI_FLASH_CMD_FQR4BA, /*!<  fast read quad output with 4-byte address, nor    */
    SPI_FLASH_CMD_4RD4BA, /*!<  4 IO read with 4-byte address, nor    */
    SPI_FLASH_CMD_PP4BA, /*!<  page program with 4-byte address, nor    */
    SPI_FLASH_CMD_QPP4BA, /*!<  quad page program with 4-byte address, nor     ==== 40    */
    SPI_FLASH_CMD_SE4BA, /*!<  sector erase with 4-byte address, nor    */
    SPI_FLASH_CMD_BE4BA, /*!<  64KB block erase with 4-byte address, nor    */
    SPI_FLASH_CMD_WRSR2, /*!<  write status register command 2, nor    */
    SPI_FLASH_CMD_LEFPA, /*!< Last ECC Failue Page Address, NAND    */
    SPI_FLASH_CMD_BBM, /*!< Bad Block Management, NAND    */
    SPI_FLASH_CMD_RBLUT, /*!< Read BBM Look Up Table, NAND    */
    SPI_FLASH_CMD_COUNT /*!< current support flash command     */
} SPI_FLASH_CMD_E;

```
The system already contains the command table of some particles (flash_cmd_table_list). You can first check whether the function corresponding to the particle to be transplanted is consistent with this command code. If it is completely consistent, the particle can be directly used; otherwise, a new command table needs to be added.<br/>
Command table type definition: <br/>
```c
/**
  * @brief  SPI_FLASH manual command
  */
typedef struct
{
    uint8_t cmd;
    uint8_t func_mode;   /*!<   manual read 0 / write 1  */
    uint8_t data_mode;   /*!<   0 no data / 1 single line / 2 dual line / 3 qual line  */
    uint8_t dummy_cycle;     /*!<   dummy cycle between command and address/data  */
    uint8_t ab_size;          /*!<   alternate byte size, */
    uint8_t ab_mode;          /*!<   alternate byte mode, 0 no, 1, single line, 2, dual line, 3 fouline */
    uint8_t addr_size;   /*!<   address byte count - 1 */
    uint8_t addr_mode;   /*!<   0 no, 1 single line / 2 dual lin / 3 four line */
    uint8_t cmd_mode;    /*!<   0 no, 1 single lien / 2 dual line / 3 four line */
} FLASH_CMD_CFG_T;

```
"cmd" refers to the command ID, such as write enable command is 0x06, read status register command is 0x05, etc. <br/>
func_mode is the function type, 0 is read operation, 1 is write operation. <br/>
data_mode is the width of the data line. 0 means no data, 1 means single data operation, 2 means double data operation, and 3 means four data lines. <br/>
A dummy_cycle is the number of clock pulses that may be present between a command and data. <br/>
The ab_size and ab_mode parameters are used for setting certain commands M7~M0 after the address, such as the 4-line read command 0XEB for the PUYA. In the timing diagram, an 8-bit MBYTE setting follows ADDR (used to determine the continue mode). The table sets these parameters to 0/3, where 0 indicates a size of 1 byte (n-1), and 3 indicates a 4-line mode.
addr_size and addr_mode refer to the address parameters that the command needs to follow. addr_size is the address length (n-1 bytes), and addr_mode represents no address (0), single line (1), double line (2), and 4 lines (3). When mode = 0, there is no address requirement, and size must be 0.<br/>
cmd_mode is a flash command, and the valid command is always 1 <br/>


## 8. NAND-FLASH transplant
The transplantation of NAND particles is basically the same as that of NOR, except that the commands used will be different. This depends on the timing exported according to the specific particles.<br/>
```{note} 
The acquisition ID timing of some particles is different. If the default timing cannot obtain the correct ID, the correct command table cannot be found. In this case, either the acquisition ID timing in the code needs to be modified, or the correct ID needs to be configured in the configuration to find the newly registered command table
```

## 9.PSRAM migration
At present, PSRAM has only been ported to APM PSRAM, including 4 lines, 8 lines and 16 lines. There are models that can support sizes from 4MB to 32MB.<br/>
Among them, 55X only supports 4 line PSRAM.<br/>
Supports 58X for 4, 8, and 16 lines, <br/>
Support for 4-wire and 8-wire is supported on 56X.<br/>

