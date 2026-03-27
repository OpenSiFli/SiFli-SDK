# MPI

The MPI HAL provides the primary APIs for accessing the Memory Peripheral
Interface (MPI) registers, which serve as the Flash/PSRAM controller on the PRO
core. The HAL is divided into two layers: `bf0_hal_mpi` and `bf0_hal_mpi_ex`.
`bf0_hal_mpi` provides low-level interfaces for hardware register access with
minimal logic; for XIP (Execute-In-Place) mode, this file should be placed in
RAM. `bf0_hal_mpi_ex` serves as a high-level wrapper for fundamental NAND, NOR,
and PSRAM operations, including initialization, page read/write, sector erase,
and other auxiliary functions.

## Key features include:
- Support for up to 5 instances (with MPI5 mountable to the LCPU).
- Support for NAND, NOR, and PSRAM.
- DMA support.
- Multi-chip support via a register command table.

## Memory address mapping:
 - MPI1: C-BUS from 0x10000000 to 0x11FFFFFF (32MB total). S-BUS from 0x60000000
   to 0x61FFFFFF (32MB total).
 - MPI2: C-BUS from 0x12000000 to 0x13FFFFFF (32MB total). S-BUS from 0x62000000
   to 0x63FFFFFF (32MB total).
 - MPI3: C-BUS from 0x14000000 to 0x17FFFFFF (64MB total). S-BUS from 0x64000000
   to 0x67FFFFFF (64MB total).
 - MPI4: C-BUS from 0x18000000 to 0x1BFFFFFF (64MB total). S-BUS from 0x68000000
   to 0x9FFFFFFF (896MB total).
 - MPI5: C-BUS from 0x1C000000 to 0x1FFFFFFF (64MB total).

## Using the MPI HAL Driver
MPI can be used to control NOR Flash, NAND Flash, 8-line PSRAM, and 16-line
PSRAM. Below is an example for NOR Flash:

```c
// Register command table to support additional Flash chips
spi_flash_register_cmd();

QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];
qspi_configure_t flash_cfg = FLASH1_CONFIG;
struct dma_config flash_dma = FLASH1_DMA_CONFIG;

flash_cfg.Instance = FLASH1;
flash_cfg.SpiMode = SPI_MODE_NOR;
flash_cfg.line = 2;
flash_cfg.base = MPI1_MEM_BASE;

flash_dma.dma_rcc = FLASH1_DMA_RCC; // 0
flash_dma.Instance = FLASH1_DMA_INSTANCE; // DMA1_Channel1
flash_dma.dma_irq = FLASH1_DMA_IRQ; // DMAC1_CH1_IRQn
flash_dma.request = FLASH1_DMA_REQUEST; // DMA_REQUEST_0

// Initialize MPI hardware controller 
flash_cfg.SpiMode = 0; // 0 for NOR, 1 for NAND, 2 for QSPI PSRAM, 3 for OPI PSRAM, 4 for HPI PSRAM
res = HAL_FLASH_Init(&amp;(spi_flash_handle[0]), &amp;flash_cfg, &amp;spi_flash_dma_handle[0], &amp;flash_dma, BSP_GetFlash1DIV());
if (res != HAL_OK)
    return error;

FLASH_HandleTypeDef hflash = &amp;spi_flash_handle[0].handle;
// Erase sector 
res = HAL_QSPIEX_SECT_ERASE(hflash, addr);
if (res &lt; 0)
    return error;

// Write a page	
res = HAL_QSPIEX_WRITE_PAGE(hflash, addr, buf, size);
if (res != size)
    return error;

// Read data; AHB read can be used here
res = nor_read_rom(hflash, addr, buf, size);

...
```

Below is an example for NAND Flash:

```c
// Register command table to support additional Flash chips
spi_flash_register_cmd();

QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];
qspi_configure_t flash_cfg = FLASH4_CONFIG;
struct dma_config flash_dma = FLASH4_DMA_CONFIG;

flash_cfg.Instance = FLASH4;
flash_cfg.SpiMode = SPI_MODE_NAND;
flash_cfg.line = 2;
flash_cfg.base = MPI4_MEM_BASE;

flash_dma.dma_rcc = FLASH4_DMA_RCC; // 0
flash_dma.Instance = FLASH4_DMA_INSTANCE; // DMA4_Channel1
flash_dma.dma_irq = FLASH4_DMA_IRQ; // DMAC1_CH4_IRQn
flash_dma.request = FLASH4_DMA_REQUEST; // DMA_REQUEST_3

// Initialize QSPI hardware controller 
flash_cfg.SpiMode = 1; // 0 for NOR, 1 for NAND, 2 for QSPI PSRAM, 3 for OPI PSRAM, 4 for HPI PSRAM
res = HAL_FLASH_Init(&amp;(spi_flash_handle[3]), &amp;flash_cfg, &amp;spi_flash_dma_handle[1], &amp;flash_dma, BSP_GetFlash2DIV());
if (res != HAL_OK)
    return error;

FLASH_HandleTypeDef hflash = &amp;spi_flash_handle[3].handle;
// Erase block; for NAND, erase is block-based 
res = HAL_NAND_ERASE_BLK(hflash, addr);
if (res &lt; 0)
    return error;

// Write a page	
res = HAL_NAND_WRITE_WITHOOB(hflash, addr, buf, size, NULL, 0);
if (res != size)
    return error;

// Read data; NAND cannot use AHB read directly and must use the driver interface
res = HAL_NAND_READ_WITHOOB(hflash, addr, buf, size, NULL, 0);

...
```

Below is an example for OPI PSRAM:

```c
QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];

qspi_configure_t flash_cfg;

flash_cfg.Instance = FLASH2;
flash_cfg.SpiMode = SPI_MODE_OPSRAM;
flash_cfg.line = 0;
flash_cfg.base = MPI2_MEM_BASE;
flash_cfg.msize = 0x8; 

// Initialize MPI hardware controller 
flash_cfg.SpiMode = 3; // 0 for NOR, 1 for NAND, 2 for QSPI PSRAM, 3 for OPI PSRAM, 4 for HPI PSRAM
res = HAL_OPI_PSRAM_Init(handle, &amp;qspi_cfg, 1);
HAL_MPI_MR_WRITE(handle, 8, 3);

// Calculate clock and delay based on the PSRAM datasheet
sys_clk = HAL_QSPI_GET_CLK(handle);
sys_clk /= 2;
if (sys_clk &lt;= 66 * 1000000)
	w_lat = 3;
else if (sys_clk &lt;= 109 * 1000000)
	w_lat = 4;
else if (sys_clk &lt;= 133 * 1000000)
	w_lat = 5;
else if (sys_clk &lt;= 166 * 1000000)
	w_lat = 6;
else if (sys_clk &lt;= 200 * 1000000)
	w_lat = 7;
else
	RT_ASSERT(0);

if (fix_lat)
	r_lat = w_lat * 2; // 10;
else
	r_lat = w_lat; // = 6; // 5;

/* Configure AHB command */
HAL_FLASH_CFG_AHB_RCMD(handle, 7, r_lat - 1, 0, 0, 3, 7, 7);
HAL_FLASH_SET_AHB_RCMD(handle, OPSRAM_RD);
HAL_FLASH_CFG_AHB_WCMD(handle, 7, w_lat - 1, 0, 0, 3, 7, 7);
HAL_FLASH_SET_AHB_WCMD(handle, OPSRAM_WR);

HAL_MPI_SET_FIXLAT(handle, fix_lat, r_lat, w_lat);
//------------------------- INIT DONE ---------------------------//

int *buf = (int *)MPI2_MEM_BASE;
int i;

// Write PSRAM memory
for(i=0; i&lt;1000; i++)
    buf[i] = i*6543;

// Read PSRAM
int value = *buf;

// Read and Write
int *src = (int *)MPI2_MEM_BASE;
int *dst = (int *)(MPI2_MEM_BASE + 0x100000);
memcpy(dst, src, 1000);

...
```

## API Reference
[]

