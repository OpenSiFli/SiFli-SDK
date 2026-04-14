# SPI Polling Example
Source path: `example/hal/spi/polling`

## Supported Boards
- `sf32lb52-lcd_n16r8`
- `sf32lb58-lcd_n16r64n4`

## Overview
This example demonstrates SPI HAL in **blocking polling mode** with self loopback verification.

Core behavior:
- SPI1 master mode, 2Lines, 8-bit, default MODE0.
- Periodic `HAL_SPI_TransmitReceive()` transactions.
- Per-round 16-byte incremental TX pattern (`seed + i`) and mismatch counting.

## Build and Download
For `sf32lb52-lcd_n16r8`, run in `project` directory:
```sh
scons --board=sf32lb52-lcd_n16r8 -j8
```

For `sf32lb58-lcd_n16r64n4`:
```sh
scons --board=sf32lb58-lcd_n16r64n4 -j8
```

After build, enter the generated `build_*` directory and run the download script.

## Hardware Setup
### Single-board loopback (recommended)
Short SPI1 MOSI and MISO:
- `sf32lb52`: `PA24(DIO)` ↔ `PA25(DI)`
- `sf32lb58`: `PA21(DO)` ↔ `PA20(DI)`

Keep:
- `CLK`: `PA28`
- `CS`:  `PA29`

Notes:
- This example uses one-shot `HAL_SPI_TransmitReceive` per round; no manual `TAKE/RELEASE CS` is used.
- Without loopback wiring, `mismatch` is usually high or data appears random.

## Expected Logs
Startup logs:
```text
Start spi polling loopback demo!
tip: short SPI1 MOSI(DIO/DO) to MISO(DI) for loopback verification.
```

Runtime logs:
```text
round=0 ret=0 mismatch=0 tx: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f rx: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
round=1 ret=0 mismatch=0 tx: 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 rx: 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10
```

Field meanings:
- `ret`: HAL return code, `0` means `HAL_OK`.
- `mismatch`: number of TX/RX mismatched bytes in current 16-byte frame.
- `round`: loop counter.

## Key Configuration Flow
1. Configure SPI1 pinmux for `DIO/DO`, `DI`, `CLK`, `CS`.
2. Enable SPI1 clock: `HAL_RCC_EnableModule(RCC_MOD_SPI1)`.
3. Initialize SPI:
   - `Direction = SPI_DIRECTION_2LINES`
   - `Mode = SPI_MODE_MASTER`
   - `DataSize = SPI_DATASIZE_8BIT`
   - `CLKPhase/CLKPolarity` selected by `SPI_MODE`
4. Loop on `HAL_SPI_TransmitReceive()` and print comparison result.

## Troubleshooting
### 1) `ret != 0`
- Check SPI1 pinmux settings.
- Check SPI clock and system init status.

### 2) `mismatch` stays non-zero
- Check MOSI↔MISO jumper quality first.
- Check pin mapping and contact reliability.
- Try lower SPI clock for signal integrity check.

### 3) No periodic logs
- Confirm code reaches `spi_test()`.
- Confirm UART console output is working.

## Reference Documents
- `EH-SF32LB52X_Pin_config_V1.3.0_20231110.xlsx`
- `DS0052-SF32LB52x-芯片技术规格书 V0p3.pdf`

## Update Log
| Version | Date | Notes |
|:---|:---|:---|
| 0.1.0 | 03/2026 | Switched to polling loopback self TX/RX demo |
