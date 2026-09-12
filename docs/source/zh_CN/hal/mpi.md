# MPI

MPI HAL 提供用于访问 MPI 外设寄存器的基本 API，它用作 PRO 上的FLASH/PSRAM控制器。 有 2 级 HAL 接口，bf0_hal_mpi 和 bf0_hal_mpi_ex。
Hal_mpi 用于访问硬件寄存器的接口，只包含很少的逻辑，对于 XIP 模式，这个文件应该放在 RAM 中。 Hal_mpi_ex 用于基本 NAND/NOR/PSRAM 功能包装器，包括初始化/页面读取/页面写入/扇区擦除和一些其他功能。

## 主要功能包括：
- 可支持NAND/NOR/PSRAM。
- DMA 支持。
- 通过寄存器命令表支持多芯片。

:::{only} SF32LB52X

- 支持 2 个实例：MPI1 ~ MPI2。
:::

:::{only} SF32LB56X

- 支持 4 个实例：MPI1 ~ MPI3、MPI5，没有 MPI4。
- MPI5 位于 LPSYS，可由 LCPU 访问（HCPU 也可以访问）。
:::

:::{only} SF32LB57X

- 支持 3 个实例：MPI1 ~ MPI3。
:::

:::{only} SF32LB58X

- 支持 5 个实例：MPI1 ~ MPI5。
- MPI5 位于 LPSYS，可由 LCPU 访问（HCPU 也可以访问）。
:::

驱动可管理的实例上限由 `FLASH_MAX_INSTANCE` 定义，MPI 为 5（_bf0_hal_mpi_ex.h_），实际存在的实例以上面列出的为准。

## 内存地址映射

MPI 挂载的存储同时映射到 C-BUS 和 S-BUS 两个地址空间，S-BUS 地址 = C-BUS 地址 + `HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET`(0x50000000)。基地址定义在对应系列的 _mem_map.h_ 中。

:::{only} SF32LB52X

| 实例   | C-BUS 基地址 | S-BUS 基地址 |
|--------|--------------|--------------|
| MPI1   | 0x10000000   | 0x60000000   |
| MPI2   | 0x12000000   | 0x62000000   |
:::

:::{only} SF32LB56X

| 实例   | C-BUS 基地址 | S-BUS 基地址 |
|--------|--------------|--------------|
| MPI1   | 0x10000000   | 0x60000000   |
| MPI2   | 0x10800000   | 0x60800000   |
| MPI3   | 0x14000000   | 0x64000000   |
| MPI5   | 0x1C000000   | 0x6C000000   |
:::

:::{only} SF32LB57X

| 实例   | C-BUS 基地址 | S-BUS 基地址 |
|--------|--------------|--------------|
| MPI1   | 0x10000000   | 0x60000000   |
| MPI2   | 0x12000000   | 0x62000000   |
| MPI3   | 0x14000000   | 0x64000000   |
:::

:::{only} SF32LB58X

| 实例   | C-BUS 基地址 | S-BUS 基地址 |
|--------|--------------|--------------|
| MPI1   | 0x10000000   | 0x60000000   |
| MPI2   | 0x12000000   | 0x62000000   |
| MPI3   | 0x14000000   | 0x64000000   |
| MPI4   | 0x18000000   | 0x68000000   |
| MPI5   | 0x1C000000   | 0x6C000000   |
:::

每个实例的地址窗口大小由实际挂载的颗粒容量决定，通过 menuconfig 的 `BSP_QSPIx_MEM_SIZE`（单位 MB）配置；由地址反查控制器时使用的窗口上限为 _mem_map.h_ 中的 `QSPIx_MAX_SIZE`。各系列 EVB 上控制器的实际容量可参考 [Flash使用指南](../app_note/flash_usage.md)。

## 使用 MPI HAL 驱动程序
MPI 可用于控制 NOR-FLASH、NAND-FLASH、PSRAM，各实例都可以挂载，选用哪个实例由芯片封装和板级配置决定。示例中 DMA 的通道、中断号和请求号由板级 _dma_config.h_ 中的 `FLASHx_DMA_INSTANCE`、`FLASHx_DMA_IRQ`、`FLASHx_DMA_REQUEST` 定义，取值随系列和板子不同，引用宏即可。下面是 NOR-FLASH 的示例：

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

以下是 NAND-FLASH 的示例。NAND 一般挂在外接存储控制器上，52X 为 MPI2、56X/57X 为 MPI3、58X 为 MPI4：

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

除 8 线(OPI)PSRAM 外，58X 还支持 16 线(HPI)PSRAM：`flash_cfg.SpiMode` 取 4（`SPI_MODE_HPSRAM`），并使用 `HAL_HYPER_PSRAM_Init()`，其余流程与下面的 OPI PSRAM 示例一致。
:::

以下是 OPI PSRAM 的示例：

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

## API参考
[](../api/hal/mpi.md)

