
# MPI

MPI HAL provides basic APIs for accessing MPI peripheral registers, which serves as FLASH/PSRAM controller on PRO. There are 2-level HAL interfaces: bf0_hal_mpi and bf0_hal_mpi_ex.
Hal_mpi is an interface for accessing hardware registers with minimal logic. For XIP mode, this file should be placed in RAM. Hal_mpi_ex is for basic NAND/NOR/PSRAM function wrappers, including initialization/page read/page write/sector erase and other functions.

## Main Features Include:
- Supports up to 5 instances (MPI5 can be mounted to LCPU).
- Supports NAND/NOR/PSRAM.
- DMA support.
- Multi-chip support through register command tables.

## Memory Address Mapping:
 - MPI1: C-BUS from 0x10000000 to 0x11FFFFFF (total 32MB). S-BUS: from 0x60000000 to 0x61FFFFFF (total 32MB)
 - MPI2: C-BUS from 0x12000000 to 0x13FFFFFF (total 32MB). S-BUS: from 0x62000000 to 0x63FFFFFF (total 32MB).
 - MPI3: C-BUS from 0x14000000 to 0x17FFFFFF (total 64MB). S-BUS: from 0x64000000 to 0x67FFFFFF (total 64MB)
 - MPI4: C-BUS from 0x18000000 to 0x1BFFFFFF (total 64MB). S-BUS: from 0x68000000 to 0x9FFFFFFF (total 896MB)
 - MPI5: C-BUS from 0x1C000000 to 0x1FFFFFFF (total 64MB).  

## Using MPI HAL Driver
MPI can be used to control NOR-FLASH, NAND-FLASH, 8-LINE PSRAM, 16-LINE PSRAM. The following is an example for NOR-FLASH:

```c

// register command table to support more flash chip
spi_flash_register_cmd();

QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];
qspi_configure_t flash_cfg = FLASH1_CONFIG;
struct dma_config flash_dma = FLASH1_DMA_CONFIG;

flash_cfg.Instance = FLASH1;
flash_cfg.SpiMode = SPI_MODE_NOR;
flash_cfg.line = 2;
flash_cfg.base = MPI1_MEM_BASE;

flash_dma.dma_rcc = FLASH1_DMA_RCC; // 0
flash_dma.Instance = FLASH1_DMA_INSTANCE; //DMA1_Channel1
flash_dma.dma_irq = FLASH1_DMA_IRQ; //DMAC1_CH1_IRQn
flash_dma.request = FLASH1_DMA_REQUEST; //DMA_REQUEST_0

// init MPI hardware controller 
flash_cfg.SpiMode = 0; // 0 for nor and 1 for nand, 2 for qspi psram, 3 for opi psram, 4 for hpi psram
res = HAL_FLASH_Init(&(spi_flash_handle[0]), &flash_cfg, &spi_flash_dma_handle[0], &flash_dma, BSP_GetFlash1DIV());
if (res != HAL_OK)
    return error;

FLASH_HandleTypeDef hflash = &spi_flash_handle[0].handle;
// erase sector 
res = HAL_QSPIEX_SECT_ERASE(hflash, addr);
if (res < 0)
    return error;

// write a page	
res = HAL_QSPIEX_WRITE_PAGE(hflash, addr, buf, size);
if (res != size)
    return error;

// read data, it can use AHB read 
res = nor_read_rom(hflash, addr, buf, size);

...
```

The following is an example for NAND-FLASH:

```c

// register command table to support more flash chip
spi_flash_register_cmd();

QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];
qspi_configure_t flash_cfg = FLASH4_CONFIG;
struct dma_config flash_dma = FLASH4_DMA_CONFIG;

flash_cfg.Instance = FLASH4;
flash_cfg.SpiMode = SPI_MODE_NAND;
flash_cfg.line = 2;
flash_cfg.base = MPI4_MEM_BASE;

flash_dma.dma_rcc = FLASH4_DMA_RCC; // 0
flash_dma.Instance = FLASH4_DMA_INSTANCE; //DMA4_Channel1
flash_dma.dma_irq = FLASH4_DMA_IRQ; //DMAC1_CH4_IRQn
flash_dma.request = FLASH4_DMA_REQUEST; //DMA_REQUEST_3
// init QSPI hardware controller 
flash_cfg.SpiMode = 1; // 0 for nor and 1 for nand, 2 for qspi psram, 3 for opi psram, 4 for hpi psram
res = HAL_FLASH_Init(&(spi_flash_handle[3]), &flash_cfg, &spi_flash_dma_handle[1], &flash_dma, BSP_GetFlash2DIV());
if (res != HAL_OK)
    return error;

FLASH_HandleTypeDef hflash = &spi_flash_handle[3].handle;
// erase block, for nand, erase is block based 
res = HAL_NAND_ERASE_BLK(hflash, addr);
if (res < 0)
    return error;

// write a page	
res = HAL_NAND_WRITE_WITHOOB(hflash, addr, buf, size, NULL, 0);
if (res != size)
    return error;

// read data, nand can not use AHB read directly, it should use driver interface
res = HAL_NAND_READ_WITHOOB(hflash, addr, buf, size, NULL, 0);

...
```

The following is an example for OPI PSRAM:

```c

QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];

qspi_configure_t flash_cfg;

flash_cfg.Instance = FLASH2;
flash_cfg.SpiMode = SPI_MODE_OPSRAM;
flash_cfg.line = 0;
flash_cfg.base = MPI2_MEM_BASE;
flash_cfg.msize = 0x8; 

// init MPI hardware controller 
flash_cfg.SpiMode = 3; // 0 for nor and 1 for nand, 2 for qspi psram, 3 for opi psram, 4 for hpi psram
res = HAL_OPI_PSRAM_Init(handle, &qspi_cfg, 1);
HAL_MPI_MR_WRITE(handle, 8, 3);

// clk and delay based on PSRAM datasheet
sys_clk = HAL_QSPI_GET_CLK(handle);
sys_clk /= 2;
if (sys_clk <= 66 * 1000000)
	w_lat = 3;
else if (sys_clk <= 109 * 1000000)
	w_lat = 4;
else if (sys_clk <= 133 * 1000000)
	w_lat = 5;
else if (sys_clk <= 166 * 1000000)
	w_lat = 6;
else if (sys_clk <= 200 * 1000000)
	w_lat = 7;
else
	RT_ASSERT(0);

if (fix_lat)
	r_lat = w_lat * 2; //10;
else
	r_lat = w_lat; // = 6; //5;

/* configure AHB command */
HAL_FLASH_CFG_AHB_RCMD(handle, 7, r_lat - 1, 0, 0, 3, 7, 7);
HAL_FLASH_SET_AHB_RCMD(handle, OPSRAM_RD);
HAL_FLASH_CFG_AHB_WCMD(handle, 7, w_lat - 1, 0, 0, 3, 7, 7);
HAL_FLASH_SET_AHB_WCMD(handle, OPSRAM_WR);

HAL_MPI_SET_FIXLAT(handle, fix_lat, r_lat, w_lat);
//-------------------------INIT DONE ---------------------------//

int *buf = (int *)MPI2_MEM_BASE;
int i;

// Write psram memory
for(i=0; i<1000; i++)
    buf[i] = i*6543;

// Read psram
int value = *buf;

// Read and Write
int *src = (int *)MPI2_MEM_BASE;
int *dst = (int *)(MPI2_MEM_BASE + 0x100000);
memcpy(dst, src, 1000);

...
```

## API Reference
[](../api/hal/mpi.md)
