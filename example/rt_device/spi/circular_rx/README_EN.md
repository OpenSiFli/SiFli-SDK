# SPI Circular DMA Example
Source path: `example/rt_device/spi/circular_rx`

## Supported Boards
- `sf32lb52-lcd_n16r8`
- `sf32lb58-lcd_n16r64n4`
- `sf32lb56-lcd_a128r12n1`
## Overview
This example validates DMA circular mode in the RT-Thread SPI driver and supports three modes:
- **Master TRX mode**: full-duplex TX/RX, suitable for loopback testing
- **Slave RX mode**: receive only, requires an external Master to provide clock
- **Slave TX mode**: transmit only, requires an external Master to provide clock

## API Reference

| API | Description |
|------|------|
| `rt_device_control(..., RT_SPI_CTRL_CONFIG_DMA_CIRCULAR, ...)` | Configure circular DMA direction |
| `rt_device_set_rx_indicate()` | Register RX half-buffer / full-buffer callback |
| `rt_device_set_tx_complete()` | Register TX half-buffer / full-buffer callback |
| `rt_spi_transfer()` | Start TX+RX full-duplex circular DMA |
| `rt_device_read()` | Start RX only circular DMA |
| `rt_device_write()` | Start TX only circular DMA |
| `rt_device_control(..., RT_SPI_CTRL_STOP_DMA_CIRCULAR, ...)` | Stop circular DMA |

## Required Configuration
`project/proj.conf` should include:
```ini
CONFIG_BSP_USING_SPI1=y
CONFIG_BSP_SPI1_TX_USING_DMA=y
CONFIG_BSP_SPI1_RX_USING_DMA=y
CONFIG_BSP_USING_SPI_DMA_CIRCULAR=y

#56-LCD need to be configured to SPI2
CONFIG_BSP_USING_SPI2=y
CONFIG_BSP_SPI2_TX_USING_DMA=y
CONFIG_BSP_SPI2_RX_USING_DMA=y
CONFIG_BSP_USING_SPI_DMA_CIRCULAR=y
```

## Mode Selection
Select by the `SPI_CIRCULAR_DEMO_MODE` macro:
- `0`: Master TRX mode (default)
- `1`: Slave RX only mode
- `2`: Slave TX only mode

## Example Flow

### Master TRX Mode (Mode 0)
1. Configure SPI1 pinmux(56 configured to SPI2)
2. Attach/open the `spi_circular` device
3. Configure SPI (Master, Mode0, 8bit, 20MHz)
4. Call `rt_spi_take_bus()/rt_spi_release_bus()` to trigger DMA handle linking
5. Call `rt_device_control(..., RT_SPI_CTRL_CONFIG_DMA_CIRCULAR, ...)` with `TXRX`
6. Call `rt_spi_transfer()` to start circular DMA
7. Short MOSI and MISO for loopback verification

### Slave RX Mode (Mode 1)
1. Configure SPI1 pinmux(56 configured to SPI2)
2. Configure SPI as Slave mode
3. Call `rt_device_control(..., RT_SPI_CTRL_CONFIG_DMA_CIRCULAR, ...)` with `RX`
4. Call `rt_device_read()` to start circular receive
5. Connect an external SPI Master to provide clock

### Slave TX Mode (Mode 2)
1. Configure SPI1 pinmux(56 configured to SPI2)
2. Configure SPI as Slave mode
3. Call `rt_device_control(..., RT_SPI_CTRL_CONFIG_DMA_CIRCULAR, ...)` with `TX`
4. Call `rt_device_write()` to start circular transmit
5. Connect an external SPI Master to provide clock

## Hardware Connection

### Pin Mapping Table

| Board | Function Pin | Local Device Pin | Remote Device Pin | Physical Pin (CONN2) |
|--------|----------|--------------|--------------|-------------------|
| sf32lb52-lcd | PA_24 | dio | SPI_MOSI | 19 |
|              | PA_25 | di  | SPI_MISO | 21 |
|              | PA_28 | clk | SPI_CLK  | 23 |
|              | PA_29 | cs  | SPI_CS   | 24 |
| sf32lb58-lcd | PA_21 | dio | SPI_MOSI | 8 |
|              | PA_20 | di  | SPI_MISO | 10 |
|              | PA_28 | clk | SPI_CLK  | 5 |
|              | PA_29 | cs  | SPI_CS   | 3 |
| sf32lb56-lcd | PA_64 | do  | SPI_MOSI | 19 |
|              | PA_69 | di  | SPI_MISO | 29 |
|              | PA_73 | clk | SPI_CLK  | 23 |
|              | PA_71 | cs  | SPI_CS   | 24 |

The hardware schematic of `sf32lb52-lcd_n16r8` is shown below:

![alt text](assets/52-DevKit-lcd-V1.0.png)
### Option 1: Single-board Loopback (Master TRX Mode)
Short SPI1 MOSI and MISO:
- `sf32lb52x`: `PA24(DIO)` ↔ `PA25(DI)`
- `sf32lb58x`: `PA21(DO)` ↔ `PA20(DI)`
- `sf32lb56x`：`PA64(DO)` ↔ `PA69(DI)`

### Option 2: External SPI Slave Device (Slave Mode)
Connect with standard 4-wire SPI. An external Master must provide clock.

### sf32lb52-lcd_n16r8 Hardware Schematic

![alt text](assets/52-DevKit-lcd-V1.0.png)

## Expected Logs

### Master TRX Mode
```text
Start SPI circular DMA demo!
Mode: Master TRX (loopback)
Circular DMA started on spi1
Tip: Short MOSI(DO) to MISO(DI) for loopback verification.
RX: half=1 full=0 rx[0..7]: 00 01 02 03 04 05 06 07, rx[mid..mid+7]: 80 81 82 83 84 85 86 87, mismatch=0
```

### Slave RX Mode
```text
Start SPI circular DMA demo!
Mode: Slave RX only
Tip: Connect external SPI Master to provide clock.
Circular DMA started on spi1
RX: half=1 full=0
```

### Slave TX Mode
```text
Start SPI circular DMA demo!
Mode: Slave TX only
Tip: Connect external SPI Master to provide clock.
Circular DMA started on spi1
TX: half=1 full=0 tx[0..7]: 00 01 02 03 04 05 06 07, tx[mid..mid+7]: 80 81 82 83 84 85 86 87
```

## Callback Functions
Use standard RT device callbacks to process half-buffer / full-buffer events:

```c
rt_device_set_rx_indicate((rt_device_t)g_spi_dev, spi_dma_circular_rx_ind);
rt_device_set_tx_complete((rt_device_t)g_spi_dev, spi_dma_circular_tx_ind);

static rt_err_t spi_dma_circular_rx_ind(rt_device_t dev, rt_size_t offset)
{
    // offset == 0             -> first half is ready
    // offset == buffer_size/2 -> second half is ready
    return RT_EOK;
}

static rt_err_t spi_dma_circular_tx_ind(rt_device_t dev, void *buffer)
{
    // buffer points to the first or second half of the TX buffer
    return RT_EOK;
}
```

## FAQ

### 1) Stuck after startup
Check:
- Whether `rt_spi_take_bus()/rt_spi_release_bus()` has been executed
- Whether DMA configuration in `proj.conf` has taken effect

### 2) half/full does not increase
- Check whether SPI pinmux is overridden
- Check whether DMA IRQ is enabled

### 3) sample_mismatch remains high (Master TRX mode)
- Poor loopback wire contact
- MISO signal interference

## Related Code
- Example: `example/rt_device/spi/circular_rx/src/main.c`
- Driver: `rtos/rtthread/bsp/sifli/drivers/drv_spi.c`
- Header: `rtos/rtthread/bsp/sifli/drivers/drv_spi.h`

## Changelog
| Version | Date | Description |
|:---|:---|:---|
| 1.0.0 | 03/2026 | Initial release, supporting Master TRX / Slave RX / Slave TX modes |
| 1.0.1 | 04/2026 | Add support for sf32lb56-lcd |
build error reason on 56x is: proj.conf does not configure CONFIG_BSP_USING_SPI_DMA_CIRCULAR
and example is no support 56x,now add support for 56x,and fix the 'dma_config.h' define for 56x SPI2 DMA
