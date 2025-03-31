# QSPI

QSPI HAL 提供用于访问 qspi 外设寄存器的基本 API，它用作 A0 上的闪存(flash)控制器。 有 2 级 hal 接口，bf0_hal_qspi 和 bf0_hal_qspi_ex。\
Hal_qspi 用于访问硬件寄存器的接口，只包含很少的逻辑，对于 XIP 模式，这个文件应该放在 RAM 中。 Hal_qspi_ex 用于基本 NAND/NOR 功能包装器，包括页面读取/页面写入/扇区擦除和一些其他功能。

## 主要功能包括：
- 最多支持 4 个实例（其中QSPI4可挂载到LCPU）。
- 可支持NAND/NOR/PSRAM。
- DMA 支持。
- 通过寄存器命令表支持多芯片。

## 内存地址映射：
 - QSPI1: memory from 0x10000000 to 0x11FFFFFF(total 32MB). 
 - QSPI2: memory from 0x64000000 to 0x67FFFFFF(total 64MB). 
 - QSPI3: memory from 0x68000000 to 0x6FFFFFFF(total 128MB).
 - QSPI4: memory from 0x12000000 to 0x13FFFFFF(total 32MB).

## 使用 QSPI HAL 驱动程序
QSPI 可用于控制 NOR-FLASH、NAND-FLASH、4-LINE PSRAM。 以下是 NOR-FLASH 的示例：

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

以下是 NAND-FLASH 的示例：

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

// read data, nand can not use AHB read directly, it should use driver interface
res = nand_read_page(hflash, addr, buf, size);

...
```

## API参考
[bf0_hal_qspi.h](hal-qspi)

