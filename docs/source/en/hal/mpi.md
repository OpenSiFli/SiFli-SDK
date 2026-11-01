
# MPI

MPI HAL provides basic APIs for accessing MPI peripheral registers, which serves as FLASH/PSRAM controller on PRO. There are 2-level HAL interfaces: bf0_hal_mpi and bf0_hal_mpi_ex.
Hal_mpi is an interface for accessing hardware registers with minimal logic. For XIP mode, this file should be placed in RAM. Hal_mpi_ex is for basic NAND/NOR/PSRAM function wrappers, including initialization/page read/page write/sector erase and other functions.

## Main Features Include:
- Supports NAND/NOR/PSRAM.
- DMA support.
- Multi-chip support through register command tables.

:::{only} SF32LB52X

- Supports 2 instances: MPI1 ~ MPI2.
:::

:::{only} SF32LB56X

- Supports 4 instances: MPI1 ~ MPI3, MPI5. There is no MPI4.
- MPI5 is located in LPSYS and can be accessed by LCPU (HCPU can access it as well).
:::

:::{only} SF32LB57X

- Supports 3 instances: MPI1 ~ MPI3.
:::

:::{only} SF32LB58X

- Supports 5 instances: MPI1 ~ MPI5.
- MPI5 is located in LPSYS and can be accessed by LCPU (HCPU can access it as well).
:::

The upper limit of instances managed by the driver is defined by `FLASH_MAX_INSTANCE`, which is 5 for MPI (_bf0_hal_mpi_ex.h_). The instances actually present are the ones listed above.

## Memory Address Mapping

The memory attached to an MPI is mapped into both the C-BUS and the S-BUS address space, where S-BUS address = C-BUS address + `HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET` (0x50000000). The base addresses are defined in the _mem_map.h_ of the corresponding series.

:::{only} SF32LB52X

| Instance | C-BUS base | S-BUS base |
|----------|------------|------------|
| MPI1     | 0x10000000 | 0x60000000 |
| MPI2     | 0x12000000 | 0x62000000 |
:::

:::{only} SF32LB56X

| Instance | C-BUS base | S-BUS base |
|----------|------------|------------|
| MPI1     | 0x10000000 | 0x60000000 |
| MPI2     | 0x10800000 | 0x60800000 |
| MPI3     | 0x14000000 | 0x64000000 |
| MPI5     | 0x1C000000 | 0x6C000000 |
:::

:::{only} SF32LB57X

| Instance | C-BUS base | S-BUS base |
|----------|------------|------------|
| MPI1     | 0x10000000 | 0x60000000 |
| MPI2     | 0x12000000 | 0x62000000 |
| MPI3     | 0x14000000 | 0x64000000 |
:::

:::{only} SF32LB58X

| Instance | C-BUS base | S-BUS base |
|----------|------------|------------|
| MPI1     | 0x10000000 | 0x60000000 |
| MPI2     | 0x12000000 | 0x62000000 |
| MPI3     | 0x14000000 | 0x64000000 |
| MPI4     | 0x18000000 | 0x68000000 |
| MPI5     | 0x1C000000 | 0x6C000000 |
:::

The address window size of each instance is determined by the capacity of the memory actually attached, and is configured through `BSP_QSPIx_MEM_SIZE` (in MB) in menuconfig. The window upper bound used when resolving an address back to its controller is `QSPIx_MAX_SIZE` in _mem_map.h_. For the actual capacity on each series' EVB, refer to the [Flash Usage Guide](../app_note/flash_usage.md).

## Using MPI HAL Driver
MPI can be used to control NOR-FLASH, NAND-FLASH and PSRAM. Any instance can host them; which instance is used depends on the chip package and the board configuration. The DMA channel, interrupt number and request number used in the examples are defined by `FLASHx_DMA_INSTANCE`, `FLASHx_DMA_IRQ` and `FLASHx_DMA_REQUEST` in the board-level _dma_config.h_, and their values differ per series and per board, so just reference the macros. The following is an example for NOR-FLASH:

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

flash_dma.Instance = FLASH1_DMA_INSTANCE;
flash_dma.dma_irq = FLASH1_DMA_IRQ;
flash_dma.request = FLASH1_DMA_REQUEST;

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

The following is an example for NAND-FLASH. A NAND is normally attached to the external memory controller: MPI2 on 52X, MPI3 on 56X/57X, and MPI4 on 58X:

:::{only} SF32LB52X

```c

// register command table to support more flash chip
spi_flash_register_cmd();

QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];
qspi_configure_t flash_cfg = FLASH2_CONFIG;
struct dma_config flash_dma = FLASH2_DMA_CONFIG;

flash_cfg.Instance = FLASH2;
flash_cfg.SpiMode = SPI_MODE_NAND;
flash_cfg.line = 2;
flash_cfg.base = MPI2_MEM_BASE;

flash_dma.Instance = FLASH2_DMA_INSTANCE;
flash_dma.dma_irq = FLASH2_DMA_IRQ;
flash_dma.request = FLASH2_DMA_REQUEST;

// init MPI hardware controller 
flash_cfg.SpiMode = 1; // 0 for nor and 1 for nand, 2 for qspi psram, 3 for opi psram, 4 for hpi psram
res = HAL_FLASH_Init(&(spi_flash_handle[1]), &flash_cfg, &spi_flash_dma_handle[1], &flash_dma, BSP_GetFlash2DIV());
if (res != HAL_OK)
    return error;

FLASH_HandleTypeDef hflash = &spi_flash_handle[1].handle;
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
:::

:::{only} SF32LB56X or SF32LB57X

```c

// register command table to support more flash chip
spi_flash_register_cmd();

QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];
qspi_configure_t flash_cfg = FLASH3_CONFIG;
struct dma_config flash_dma = FLASH3_DMA_CONFIG;

flash_cfg.Instance = FLASH3;
flash_cfg.SpiMode = SPI_MODE_NAND;
flash_cfg.line = 2;
flash_cfg.base = MPI3_MEM_BASE;

flash_dma.Instance = FLASH3_DMA_INSTANCE;
flash_dma.dma_irq = FLASH3_DMA_IRQ;
flash_dma.request = FLASH3_DMA_REQUEST;

// init MPI hardware controller 
flash_cfg.SpiMode = 1; // 0 for nor and 1 for nand, 2 for qspi psram, 3 for opi psram, 4 for hpi psram
res = HAL_FLASH_Init(&(spi_flash_handle[2]), &flash_cfg, &spi_flash_dma_handle[2], &flash_dma, BSP_GetFlash3DIV());
if (res != HAL_OK)
    return error;

FLASH_HandleTypeDef hflash = &spi_flash_handle[2].handle;
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
:::

:::{only} SF32LB58X

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

flash_dma.Instance = FLASH4_DMA_INSTANCE;
flash_dma.dma_irq = FLASH4_DMA_IRQ;
flash_dma.request = FLASH4_DMA_REQUEST;

// init MPI hardware controller 
flash_cfg.SpiMode = 1; // 0 for nor and 1 for nand, 2 for qspi psram, 3 for opi psram, 4 for hpi psram
res = HAL_FLASH_Init(&(spi_flash_handle[3]), &flash_cfg, &spi_flash_dma_handle[3], &flash_dma, BSP_GetFlash4DIV());
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
:::

:::{only} SF32LB58X

Besides 8-line (OPI) PSRAM, 58X also supports 16-line (HPI) PSRAM: set `flash_cfg.SpiMode` to 4 (`SPI_MODE_HPSRAM`) and use `HAL_HYPER_PSRAM_Init()`. The rest of the flow is the same as the OPI PSRAM example below.
:::

The following is an example for OPI PSRAM:

```c

QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];
FLASH_HandleTypeDef *handle = &(spi_flash_handle[1].handle);

qspi_configure_t flash_cfg;

flash_cfg.Instance = FLASH2;
flash_cfg.SpiMode = SPI_MODE_OPSRAM;
flash_cfg.line = 0;
flash_cfg.base = MPI2_MEM_BASE;
flash_cfg.msize = 0x8; 

// init MPI hardware controller 
flash_cfg.SpiMode = 3; // 0 for nor and 1 for nand, 2 for qspi psram, 3 for opi psram, 4 for hpi psram
res = HAL_OPI_PSRAM_Init(handle, &flash_cfg, 1);
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
