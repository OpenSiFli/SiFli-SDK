# Flash使用指南

## 1. FLASH介绍 
SiFLi FLASH 控制器支持最多同时挂载多个FLASH颗粒（55X系列支持4个， 58X系列支持5个， 56X系列支持4个），可以是NOR-FLASH, 也可以是NAND-FLASH。<br/>
相比55X， 58X和56X的控制器增加了对8线和16线PSRAM的支持，控制器名字由QSPI改为MPI，意为专门为Memory提供的控制器。<br/>
针对55X系列，一般控制器一默认控制片内NOR-FLASH , 二和三由不同方案的板子决定， 四接NOR FLASH， 可以由LCPU访问（HCPU也可以访问）。<br/>
针对58X系列，一般控制器一和二默认控制片内NOR-FLASH 或PSRAM，具体由芯片封装决定, 三和四由不同方案的板子决定， 五为片内NOR FLASH， 可以由LCPU访问（HCPU也可以访问）。<br/>
针对56X系列，一般控制器一和二默认控制片内NOR-FLASH 或PSRAM，具体由芯片封装决定, 三由不同方案的板子决定， 五为片内NOR FLASH， 可以由LCPU访问（HCPU也可以访问）（没有四）。<br/>
FLASH驱动分为HAL层和DRV层，其中HAL层提供硬件寄存器访问和简单逻辑接口，DRV层提供了FLASH读/写/擦等基本操作接口，初始化及控制接口，针对设备和文件系统操作的接口。<br/>
- 控制器可以提供单线FLASH操作及4线FLASH操作
- 内部有DMA进行加速
- 支持PAGE写，支持PAGE读（NOR-FLASH支持通过AHB总线直接以内存方式进行读）
- 支持SECTOR/BLOCK 不同大小擦除，基本操作单位大小与颗粒有关

## 2. FLASH 配置 

FLASH控制器对应的FLASH颗粒属性及使用方式可以通过menuconfig（rtconfig.h）进行配置，包括是否使能，是否使用DMA, 是否连接MTD设备及文件系统信息等。<br/>
下面是55X EVB开发板上部分功能示例：<br/>
命令行输入 "menuconfig" 开启配置：<br/>
- Choose "RTOS  --->"
- Choose "On-chip Peripheral Driver ---> "	
- Choose "Enable QSPI  --->	"
- Choose "Enable QSPI Driver "
- Choose "QSPI Controller 1 Enable --->" 
- Choose "QSPI1 Mode: 0 NOR, 1 NAND, 2 PSRAM, 3 OTHERS "
- Choose "QSPI1 Use File System ---"
- Choose "QSPI1 Mem Size()MB"
- Choose "QSPI1 Manul & Device ID"

下面是对各个FLASH控制器的配置对应定义宏的示例和意义。<br/>

### FLASH1:(FLASH1默认为NOR-FLASH, 模式不可修改)
```c
#define BSP_USING_QSPI 1            /*使能QSPI 模块*/
#define BSP_USING_SPI_FLASH 1       /*使能QSPI 控制FLASH功能（为了兼容以前版本）*/
#define BSP_ENABLE_QSPI1 1          /*使能QSPI控制器1*/
#define BSP_QSPI1_USING_DMA 1       /*QSPI1 使用FIFO DMA */
#define BSP_QSPI1_MODE 0            /*QSPI1 连接控制Nor FLASH*/
#define BSP_QSPI1_MEM_SIZE 4        /*配置FLASH1的总大小为4MB*/
#define BSP_QSPI1_CHIP_ID 0         /*FLASH1 DEV ID 由系统获取，不需要手动输入（这个配置是为了当有些FLASH获取ID的命令时序与默认设置不同时，手动添加ID以便查找正确的命令表）*/
```

### FLASH2:

```c
#define BSP_ENABLE_QSPI2 1          /*使能QSPI2 模块*/
#define BSP_QSPI2_USING_DMA 1       /*使能QSPI2 FIFO DMA <*/
#define BSP_QSPI2_MODE 0            /*设置FLASH2 为NOR-FLASH*/
#define BSP_QSPI2_MTD_EN 1          /*FLASH2 注册MTD 设备 */
#define BSP_QSPI2_FS_START 2048     /*FLASH2 MTD 设备可访问地址的起始位置为8MB(2048*4K)*/
#define BSP_QSPI2_FS_SIZE 2048      /*长度为8MB(2048*4KB)*/
#define BSP_QSPI2_MEM_SIZE 32       /*配置FLASH2的总大小为32MB*/ 
#define BSP_QSPI2_CHIP_ID 0         /*FLASH2 DEV ID 由系统获取，不需要手动输入*/
```

### FLASH3:
```c
#define BSP_ENABLE_QSPI3 1          /*使能QSPI3 模块*/
#define BSP_QSPI3_USING_DMA 1       /*使能QSPI3 FIFO DMA <*/
#define BSP_QSPI3_MODE 1            /*设置FLASH2 为NAND-FLASH*/
#define BSP_QSPI3_MEM_SIZE 128      /*配置FLASH3的总大小为128MB*/
#define BSP_QSPI3_CHIP_ID 0         /*FLASH3 DEV ID 由系统获取，不需要手动输入*/
```                              

下面是58X EVB开发板上部分功能示例：<br/>
命令行输入 "menuconfig" 开启配置：
- Choose "RTOS  --->"
- Choose "On-chip Peripheral Driver ---> "	
- Choose "Enable MPI  --->	"
- Choose "Enable QSPI Driver "
- Choose "MPI Controller 1 Enable --->" 
- Choose "MPI Mode: 0 NOR, 1 NAND, 2 PSRAM, 3 OPSRAM, 4 HPSRAM, 5 LEGACY_PSRAM "
- Choose "MPI1 Mem Size()MB"

下面是对各个FLASH控制器的配置对应定义宏的示例和意义。<br/>


### MPI:(MPI 默认设置，部分设置为了兼容之前版本)
```c
#define BSP_USING_MPI 1             /*使能MPI 模块*/
#define BSP_USING_SPI_FLASH 1       /*使能MPI 控制FLASH功能（为了兼容以前版本）*/
```

### MPI1:(FLASH1默认为NOR-FLASH, 模式不可修改)
```c
#define BSP_ENABLE_MPI1 1           /*使能MPI 1 模块*/
#define BSP_ENABLE_QSPI1 1          /*使能MPI 1（为了兼容以前版本）*/
#define BSP_MPI1_MODE_4 1           /*MPI1 设置为功能4---16线PSRAM*/
#define BSP_QSPI1_MODE 4
#define BSP_USING_PSRAM1 1
#define BSP_QSPI1_MEM_SIZE 32       /*配置MPI1的总大小为16MB*/
```

### MPI2:

```c
#define BSP_ENABLE_MPI2 1           /*使能MPI 2 模块*/
#define BSP_ENABLE_QSPI2 1          /*使能MPI 2（为了兼容以前版本）*/
#define BSP_MPI2_MODE_4 1           /*MPI2 设置为功能4---16线PSRAM*/
#define BSP_QSPI2_MODE 4
#define BSP_USING_PSRAM2 1
#define BSP_QSPI2_MEM_SIZE 32       /*配置MPI2的总大小为32MB*/
```

### MPI3:

```c
#define BSP_ENABLE_MPI3 1           /*使能MPI 3 模块*/
#define BSP_ENABLE_QSPI3 1          /*使能MPI 3（为了兼容以前版本）*/
#define BSP_MPI3_MODE_0 1           /*MPI3 设置为功能0---NOR FLASH*/
#define BSP_QSPI3_MODE 0
#define BSP_USING_NOR_FLASH3 1
#define BSP_QSPI3_USING_DMA 1       /*MPI3 使用DMA*/
#define BSP_QSPI3_MEM_SIZE 32       /*配置MPI2的总大小为32MB*/
```

### MPI4:

```c
#define BSP_ENABLE_MPI4 1           /*使能MPI 4 模块*/
#define BSP_ENABLE_QSPI4 1          /*使能MPI 4（为了兼容以前版本）*/
#define BSP_MPI4_MODE_1 1           /*MPI4 设置为功能1---NAND FLASH*/
#define BSP_QSPI4_MODE 1
#define BSP_USING_NAND_FLASH4 1
#define BSP_QSPI4_USING_DMA 1       /*MPI5 使用DMA*/
#define BSP_QSPI4_MEM_SIZE 128       /*配置MPI2的总大小为128MB*/
```

### MPI5:

```c
#define BSP_ENABLE_MPI5 1           /*使能MPI 5 模块*/
#define BSP_ENABLE_QSPI5 1          /*使能MPI 5（为了兼容以前版本）*/
#define BSP_MPI5_MODE_0 1           /*MPI5 设置为功能0---NOR FLASH*/
#define BSP_QSPI5_MODE 0
#define BSP_USING_NOR_FLASH5 1
#define BSP_QSPI5_USING_DMA 1       /*MPI5 使用DMA*/
#define BSP_QSPI5_MEM_SIZE 4       /*配置MPI5的总大小为4MB*/
```

56X的配置除了不存在MPI4,其他与58X基本一致，这里不再单独列举示例。<br/>

## 3.FLASH 地址空间

### 55X 系列地址
默认EVB上FLASH大小配置，其中大小为可使用空间，描述为实际情况下开发板进行配置： 

|名称             | 起始地址       | 结束地址       |    最大大小(字节)    | 描述
|-----------------|----------------|----------------|------------------|--------
|FLASH1           | 0x10000000     | 0x11FFFFFF     | 32 * 1024 * 1024 | EVB的容量为4MByte |
|FLASH2           | 0x64000000     | 0x67FFFFFF     | 64 * 1024 * 1024 | EVB的容量为32MByte|
|FLASH3           | 0x68000000     | 0x6FFFFFFF     | 128 * 1024 * 1024| EVB未使能         |
|FLASH4           | 0x12000000     | 0x13FFFFFF     | 32 * 1024 * 1024 | EVB未使能         |

4个FLASH控制器基地址定义可在 _mem_map.h_ 中找到， 对应的最大内存空间如表中描述，而实际使用的大小由menuconfig通过实际连接的颗粒决定：
```c
#define QSPI1_MEM_BASE   (0x10000000)
#define QSPI2_MEM_BASE   (0x64000000)
#define QSPI3_MEM_BASE   (0x68000000)
#define QSPI4_MEM_BASE   (0x12000000)

```

### 58X 系列地址
默认EVB上FLASH大小配置，其中大小为可使用空间，描述为实际情况下开发板进行配置： <br/>

C-BUS 地址：<br/>
|名称             | 起始地址       | 结束地址       |    最大大小(字节)    | 描述
|-----------------|----------------|----------------|------------------|--------
|MPI1             | 0x10000000     | 0x11FFFFFF     | 32 * 1024 * 1024 | EVB的容量为16MByte |
|MPI2             | 0x12000000     | 0x13FFFFFF     | 32 * 1024 * 1024 | EVB的容量为16MByte |
|MPI3             | 0x14000000     | 0x17FFFFFF     | 64 * 1024 * 1024 | EVB的容量为32MByte |
|MPI4             | 0x18000000     | 0x1BFFFFFF     | 64 * 1024 * 1024 | EVB的容量为64MByte |
|MPI5             | 0x1C000000     | 0x1FFFFFFF     | 64 * 1024 * 1024 | EVB的容量为4MByte  |

S-BUS 地址：<br/>
|名称             | 起始地址       | 结束地址       |    最大大小(字节)    | 描述
|-----------------|----------------|----------------|------------------|--------
|MPI1             | 0x60000000     | 0x61FFFFFF     | 32 * 1024 * 1024 | EVB的容量为16MByte |
|MPI2             | 0x62000000     | 0x63FFFFFF     | 32 * 1024 * 1024 | EVB的容量为16MByte |
|MPI3             | 0x64000000     | 0x67FFFFFF     | 64 * 1024 * 1024 | EVB的容量为32MByte |
|MPI4             | 0x68000000     | 0x9FFFFFFF     | 896 * 1024 * 1024| EVB的容量为64MByte |

为了NAND FLASH 能使用超过64MB的空间，在MPI挂载NAND的时候，建议挂载在MPI4，同时使用0x68000000开始的地址。<br/>

5个MPI控制器基地址定义可在 _mem_map.h_ 中找到， 对应的最大内存空间如表中描述，而实际使用的大小由menuconfig通过实际连接的颗粒决定：
```c
#define QSPI1_MEM_BASE   (0x10000000)
#define QSPI2_MEM_BASE   (0x12000000)
#define QSPI3_MEM_BASE   (0x14000000)
#define QSPI4_MEM_BASE   (0x18000000)
#define QSPI5_MEM_BASE   (0x1C000000)
```

### 56X 系列地址
默认EVB上FLASH大小配置，其中大小为可使用空间，描述为实际情况下开发板进行配置： <br/>

C-BUS 地址：<br/>
|名称             | 起始地址       | 结束地址       |    最大大小(字节)    | 描述
|-----------------|----------------|----------------|-------------------|--------
|MPI1             | 0x10000000     | 0x107FFFFF     |   8 * 1024 * 1024 | EVB的容量为4MByte  |
|MPI2             | 0x10800000     | 0x13FFFFFF     |  56 * 1024 * 1024 | EVB的容量为16MByte |
|MPI3             | 0x14000000     | 0x17FFFFFF     | 128 * 1024 * 1024 | EVB的容量为128MByte |
|MPI5             | 0x1C000000     | 0x1FFFFFFF     |  64 * 1024 * 1024 | EVB的容量为1MByte  |

S-BUS 地址：<br/>
|名称             | 起始地址       | 结束地址       |    最大大小(字节)    | 描述
|-----------------|----------------|----------------|-------------------|--------
|MPI1             | 0x60000000     | 0x607FFFFF     |   8 * 1024 * 1024 |   |
|MPI2             | 0x60800000     | 0x63FFFFFF     |  56 * 1024 * 1024 |   |
|MPI3             | 0x64000000     | 0x9FFFFFFF     | 960 * 1024 * 1024 |   |


4个MPI控制器基地址定义可在_mem_map.h_ 中找到， 对应的最大内存空间如表中描述，而实际使用的大小由menuconfig通过实际连接的颗粒决定：
```c
#define QSPI1_MEM_BASE   (0x10000000)
#define QSPI2_MEM_BASE   (0x10800000)
#define QSPI3_MEM_BASE   (0x14000000)
#define QSPI5_MEM_BASE   (0x1C000000)
```


## 4.FLASH RT-Thread驱动访问
基于RT-THREAD ，FLASH提供了一套访问接口，可以通过地址进行操作（绝对地址来决定使用哪一个控制器）。

```c
/**
* @brief  Flash controller hardware initial.
* @retval 0 if success.

int rt_hw_flash_init(void)；

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

通过接口`rt_hw_flash_init`进行初始化，在此之前，各个FLASH是否使能，以及对应的配置都已经由 _rtconfig.h_ 决定，这里不再通过输入参数来修改。 <br/>
通过`rt_flash_read`进行读数据操作，对NOR-FLASH可以直接通过内存方式读，对NAND-FLASH只能通过此接口，调用者需要自行维护数据缓冲区的管理。 <br/>
通过`rt_flash_write`进行数据写操作，无论NAND/NOR， 写操作都需要通过该接口来完成，同样调用者需要维护数据缓冲区。 <br/>
通过`rt_flash_erase`进行擦除，对NOR-FLASH而言，最小的擦除单位为SECTOR(一般为4KB)，需要地址和长度都按照SECTOR对齐，否则会出现误擦除的现象，调用者需要自行维护不对齐部分地址数据的恢复。 <br/>
对NAND-FLASH而言， 最小擦除单位为BLOCK(一般为128KB)，需要地址和长度都按照BLOCK对齐，调用者需要自行维护不对齐部分的数据备份和恢复。 <br/>

```{note}
需要注意，FLASH不支持XIP时擦写同一片Flash，因此需要将Flash驱动相关代码放在SRAM中执行，以下为Keil的链接脚本片段，
地址0x200E1000开始64KB空间用于存放需要在SRAM执行的Flash驱动代码，可以对照修改链接脚本
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

下面是对FLASH1 1MB地址进行4KB数据写操作的示例： 
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

## 5.FLASH RT-Thread MTD 设备使用
RT-THREAD 中的MTD包含MTD-NOR 和MTD-NAND两种设备驱动，如果在配置文件中对FLASH进行了MTD进行了正确的配置并使能，在系统启动过程中会对这些设备自动注册， 注册成功与否可以通过接口查询。 <br/>
比如FLASH1的查询，可以通过rt_device_find("flash1") 的方式进行查询，每个控制器对应的设备名分别为“flash1" , "flash2", "flash3" , 若查询返回非空，即已经注册成功。 <br/>

目前MTD设备已经用在了文件系统和USB STORAGE上，在使用上主要是查找设备，之后绑定设备即可，MTD的读写已经在mtd_nor中进行了封装，不需要用户处理。 <br/>
下面是注册时候对应的数据类型，若用户自己注册，则需要填写对应数据并实现操作接口，如果已经注册完成，则只需要查找设备，之后的操作由系统后台完成。 <br/>
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

下面是MTD-NOR 在用在文件系统时的实例:
```c
    // 注册MTD DEVICE
    struct rt_mtd_nor_device *nod = malloc(sizeof(struct rt_mtd_nor_device));

    nod->block_start = BSP_FLASH1_FS_START;
    nod->block_size = FLASH_SECT_SIZE;
    nod->block_end = nod->block_start + BSP_FLASH1_FS_SIZE;
    nod->ops = &flash_ops;
    nod->parent.user_data = &lflash_handle[0];
    lflash_handle[0].nod = nod;

    rt_mtd_nor_register_device("flash1", (struct rt_mtd_nor_device *)(lflash_handle[0].nod));
	
	// 查找MTD DEVICE:
	rt_device_t dev = rt_device_find("flash1");
    if (dev) // device find, it has beed registered to mtd
    {
        // MTD设备的使用，直接使用设备名，mkfs, mount都会去查找设备并调用设备的读写接口
        if (dfs_mkfs("elm", "flash1") == 0)
        {
          dfs_mount("flash1", "/", "elm", 0, 0);
		  ....
        }
    }
```

## 6.当前FLASH空间已分配使用情况

从 _mem_map.h_ 中已列出的空间看， 从FLASH1的起始位置（0x10000000 ）的64KB用来存放flash_table, 之后的64KB(0x10010000)用来存方boot patch, 从 0x10020000开始用来存放bin。 <br/>
FLASH2 如果为NOR-FLASH, 则watch 的资源文件从其起始位置开始存放(0x64000000).
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

FLASH 还用在了存储变量和日志上，目前使用FLASHDB(或者EasyFlash)， flash1/flash2 都有部分空间暂时用来使用, 这部分需要按系统需求重新规划。

## 7. NOR-FLASH 移植

根据不同客户的开发板，在设计时会使用不同的NOR-FLASH 颗粒，而在使用这些颗粒时，主要从硬件规格和操作命令配置上进行匹配。<br/>

### 硬件规格
  - 电压配置 ： 3.3V 或 1.8V
  - SPI 模式 ： 除单线外，是否支持双线，4线SPI.
  - 最高频率 ： 在单线/双线/4线模式下分别支持的最高频率
  - 其他如工作温度，擦写速度等，这些与软件移植无关，可以先不用考虑

### 操作命令

操作命令主要时指读/写/擦等相关的颗粒命令，这个与厂商和具体颗粒有关，需要从规格书中查找对比。<br/>
A0 上增加了添加命令表的方式，用来增加对新FLASH颗粒的支持（_flash_table.c_），主要涉及的命令都在 _bf0_hal_qspi.h中SPI_FLASH_CMD_E_ 定义，其中包含了NAND/NOR的命令。<br/>
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
系统中已经包含了部分颗粒的命令表（flash_cmd_table_list），可以先查看需要移植的颗粒对应功能与此命令码是否一致，如果完全一致，则颗粒可以直接使用，否则需要添加新的命令表。<br/>
命令表类型定义：<br/>
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
其中 "cmd" 是指命令ID,比如 write enable 命令为0x06， read status register 命令为0x05 等 <br/>
func_mode 是功能类型， 0表是读操作， 1表是写操作。 <br/>
data_mode 是数据线宽度， 0表是没有数据，1表是数据单线操作，2表示数据双线操作，3表示数据4线操作。   <br/>
dummy_cycle 是指在命令和数据之间可能存在的填充时钟数量。<br/>
ab_size 和 ab_mode 针对的是在地址之后的有些命令M7~M0的设置，比如针对PUYA 的4线读命令0XEB，在时序图中ADDR之后有8比特的MBYTE设置（这里用作了continue 模式的判断），表中设置这两个参数为0/3 ， 其中0表是size 为1字节（n-1）,3表是4线模式。<br/>
addr_size 和addr_mode 是指命令需要跟的地址参数， addr_size 为地址长度（n-1字节）， addr_mode 表是无地址（0）， 单线（1），双线（2）， 4线（3）， 当mode = 0时， 没有地址需求，size必须为0。<br/>
cmd_mode 是否flash命令，有效命令总为1 <br/>


## 8. NAND-FLASH 移植
NAND 颗粒的移植基本上和NOR 一致，只是需要用到的命令会由差别，这个要根据具体颗粒来导出时序。<br/>
```{note} 有些颗粒的获取ID的时序不同，如果用默认的时序无法获取正确的ID，则不能查找到正确的命令表，这时要么需要修改代码中的获取ID时序，要么在配置时配置上正确的ID以便查找到新注册的命令表
```

## 9.PSRAM 移植
目前PSRAM只是对APM的PSRAM进行过移植，包括4线，8线和16线。大小从4MB到32MB都有型号可以支持。<br/>
其中55X只对4线PSRAM进行了支持。<br/>
58X 对4线，8线和16线都可以支持，<br/>
56X 对4线和8线进行支持。<br/>


