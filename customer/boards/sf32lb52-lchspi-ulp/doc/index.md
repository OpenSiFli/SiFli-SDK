# SF32LB52-LCHSPI-ULP

Board `sf32lb52-lchspi-ulp` is based on the [立创·黄山派 development board (SF32LB52-黄山派)](https://wiki.sifli.com/board/sf32lb52x/SF32LB52-%E9%BB%84%E5%B1%B1%E6%B4%BE.html), which is designed as a smart watch / smart band reference design, and
has module [SF32LB52X-MOD-1-N16R8](https://wiki.sifli.com/silicon/%E6%A8%A1%E7%BB%84%E5%9E%8B%E5%8F%B7%E6%8C%87%E5%8D%97.html#sf32lb52-mod-1) on the board.
The module integrates SoC `SF32LB525UC6` with 16MB QSPI-NOR Flash and 8MB OPI-PSRAM.

Main on-board resources:

- 1.85-inch 390x450 QSPI AMOLED display (driver IC CO5300) with capacitive touch panel
- Audio: on-board MEMS microphone and Class-D audio power amplifier, with a GH-1.25mm connector for an external speaker (up to 3W/4Ω or 2W/8Ω)
- Keys: KEY1 power key (PA34, long-press to reset) and KEY2 function key (PA43)
- USB Type-C to UART for program download and debug; SPI interface TF card slot
- RGB LED, motor driver, 6-axis IMU (LSM6DS3TR-C), 3-axis magnetometer (MMC5603NJ) and ambient light sensor (LTR-303ALS-01)
- 30-pin 1.27mm expansion header exposing GPIOs, power rails and debug UART

For detailed hardware information (microphone / speaker wiring, GPIO assignment, connector pin definitions, power supply options, etc.), please refer to the [SF32LB52-黄山派 board user guide](https://wiki.sifli.com/board/sf32lb52x/SF32LB52-%E9%BB%84%E5%B1%B1%E6%B4%BE.html).
