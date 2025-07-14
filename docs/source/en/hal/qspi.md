# QSPI

QSPI HAL provides basic APIs for accessing QSPI peripheral registers, used as flash controller on A0. There are 2-level HAL interfaces: bf0_hal_qspi and bf0_hal_qspi_ex.
Hal_qspi is used for accessing hardware register interfaces, containing only minimal logic. For XIP mode, this file should be placed in RAM. Hal_qspi_ex is used for basic NAND/NOR functional wrappers, including page read/page write/sector erase and other functions.

## Key Features Include:
- Supports up to 4 instances (QSPI4 can be mounted to LCPU).
- Supports NAND/NOR/PSRAM.
- DMA support.
- Multi-chip support through register command tables.

## Memory Address Mapping:
 - QSPI1: memory from 0x10000000 to 0x11FFFFFF (total 32MB). 
 - QSPI2: memory from 0x64000000 to 0x67FFFFFF (total 64MB). 
 - QSPI3: memory from 0x68000000 to 0x6FFFFFFF (total 128MB).
 - QSPI4: memory from 0x12000000 to 0x13FFFFFF (total 32MB).

## Using QSPI HAL Driver
QSPI can be used to control NOR-FLASH, NAND-FLASH, 4-LINE PSRAM. The following is an example for NOR-FLASH:

```c
// register command table to support more flash chip
spi_flash_register_cmd();

QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];
qspi_configure_t flash_cfg = FLASH1_CONFIG;
struct dma_config flash_dma = FLASH1_DMA_CONFIG;

// init QSPI hardware controller 
flash_cfg.SpiMode = 0; // 0 for nor and 1 for nand
res = HAL_FLASH_Init(&(spi_flash_handle[0]), &flash_cfg, &spi_flash_dma_handle[0], &flash_dma, BSP_GetFlash1DIV());
if (res != HAL_OK)
    return error;

// erase sector 
res = nor_sector_erase(hflash, addr);
if (res < 0)
    return error;

// write a page	
res = nor_write_page(hflash, addr, buf, size);
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
qspi_configure_t flash_cfg = FLASH1_CONFIG;
struct dma_config flash_dma = FLASH1_DMA_CONFIG;

// init QSPI hardware controller 
flash_cfg.SpiMode = 1; // 0 for nor and 1 for nand
res = HAL_FLASH_Init(&(spi_flash_handle[1]), &flash_cfg, &spi_flash_dma_handle[1], &flash_dma, BSP_GetFlash2DIV());
if (res != HAL_OK)
    return error;

// erase block, for nand, erase is block based 
res = nand_erase_block(hflash, addr);
if (res < 0)
    return error;

// write a page	
res = nand_write_page_algn(hflash, addr, buf, size);
if (res != size)
    return error;

// read data, nand cannot use AHB read directly, it should use driver interface
res = nand_read_page(hflash, addr, buf, size);

...
```

## API Reference
[bf0_hal_qspi.h](hal-qspi)