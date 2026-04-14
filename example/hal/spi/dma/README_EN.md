# SPI DMA Circular Transfer Example
Source code path: `example/hal/spi_dma`

## Supported Platforms
The example can run on the following development boards:
- sf32lb52-lcd_n16r8
- sf32lb58-lcd_n16r64n4

## Overview
This example demonstrates the continuous transmission and reception of SPI HAL in **DMA Circular mode**:
- SPI1 master mode, `2Lines`, 8bit, default `SPI MODE0`
- TX DMA + RX DMA working simultaneously, continuously triggering clock and circularly transferring data
- Statistics of operation status through DMA half-full/full callbacks
- Statistics of exceptions and printing of error codes through error callbacks

The focus of this example is to verify "whether the DMA circular link is stable", not to read NOR ID in blocking mode.

## Usage of the Example

### Compilation and Burning
#### Taking sf32lb52-lcd_n16r8 development board as an example
* Switch to the example project directory and run the scons command to compile:
```
scons --board=sf32lb52-lcd_n16r8 -j8
```
```
Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat` and select the port as prompted to download:
> build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and download, please refer to the relevant introduction in [](/quickstart/get-started.md).

Enable SPI1 DMA related switches in `project/proj.conf`:

```
CONFIG_BSP_SPI1_TX_USING_DMA=y
CONFIG_BSP_SPI1_RX_USING_DMA=y
```

### Hardware Connection

- Only short `MOSI(DIO/DO)` to `MISO(DI)`
- `CLK` and `CS` remain normal output, no need to connect to external slave devices

You can also connect to a real slave device (another board as SPI Slave), in which case connect according to the standard four-wire connection and share ground.

```{eval-rst}
+--------------+----------+---------------+---------------------+
| Board        | Function Pin | Signal       | Physical Pin (CONN2) |
+==============+==============+==============+======================+
| sf32lb52-lcd | PA_24     | SPI1_MOSI(DIO)| 19                   |
+              +--------------+--------------+----------------------+
|              | PA_25     | SPI1_MISO(DI) | 21                   |
+              +--------------+--------------+----------------------+
|              | PA_28     | SPI1_CLK      | 23                   |
+              +--------------+--------------+----------------------+
|              | PA_29     | SPI1_CS       | 24                   |
+--------------+--------------+--------------+----------------------+
| sf32lb58-lcd | PA_21     | SPI1_MOSI(DO) | 8                    |
+              +--------------+--------------+----------------------+
|              | PA_20     | SPI1_MISO(DI) | 10                   |
+              +--------------+--------------+----------------------+
|              | PA_28     | SPI1_CLK      | 5                    |
+              +--------------+--------------+----------------------+
|              | PA_29     | SPI1_CS       | 3                    |
+--------------+--------------+--------------+----------------------+
```
Refer to the following diagram for the hardware schematic of sf32lb52-lcd_n16r8:
![alt text](assets/52-DevKit-lcd-V1.0.png)
![alt text](assets/nor_flash.png)
#### Example Output Results:

### Startup Log
```text
Start spi dma circular demo!
spi dma running, tx/rx circular started.
tip: short SPI1 MOSI(DIO/DO) to MISO(DI) for loopback verification.
```

### Running Log (Example)
```text
half=1234, full=1233, err=0
rx[0..7]: 00 01 02 03 04 05 06 07, rx[mid..mid+7]: 80 81 82 83 84 85 86 87
half=2470, full=2469, err=0
rx[0..7]: 00 01 02 03 04 05 06 07, rx[mid..mid+7]: 80 81 82 83 84 85 86 87
```

Description:
- `half/full` will continuously increase, indicating that DMA is working in circular mode
- `err` should normally remain `0`
- In loopback short-circuit scenario, `rx` is commonly a cyclic sequence of 0x00~0xFF (may have phase offset)

## Functional Flow

1. Configure SPI1 pinmux and clock
2. Initialize SPI1 (Master, 2Lines, 8bit)
3. Initialize RX/TX DMA, both in `DMA_CIRCULAR` mode
4. Call `HAL_SPI_TransmitReceive_DMA()` to start continuous transmission and reception
5. Statistics status in DMA half-full/full callbacks
6. Print status and sample RX data in main loop

## Exception Diagnosis

- `half/full` not increasing for a long time
  1. Check if SPI1 DMA switch is enabled in `proj.conf`
  2. Check if DMA IRQ is correctly enabled

- `err` continuously increasing
  1. Print `spi_err` error code for location (already output in log)
  2. Check if DMA request/channel matches board-level configuration

- Data irregular in loopback scenario
  1. Check if MOSI and MISO are indeed short-circuited
  2. Check pin level and connection reliability

## Reference Documents
* EH-SF32LB52X_Pin_config_V1.3.0_20231110.xlsx
* DS0052-SF32LB52x-芯片技术规格书 V0p3.pdf
* PY25Q128HA_datasheet_V1.1.pdf

## Update Records
| Version | Date | Release Notes |
|:---|:---|:---|
| 0.1.0 | 03/2026 | Initial version |