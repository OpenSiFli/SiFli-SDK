# SF32LB57-EPD_N16R16

`sf32lb57-epd_n16r16` supports the SF32LB57-DevKit-EPD_MB_V1.0 e-paper development board. It uses an SF32LB57GYD7N6 chip with 16 MB NOR Flash and 16 MB PSRAM.

## On-board hardware

| Hardware | Interface and pins |
| --- | --- |
| E0470A03 e-paper display | 4.7 inches, 684 × 1216 in portrait orientation; LCDC1 8-bit TCON interface |
| TPS65185 display power | I2C1, address 0x68; WAKEUP PA42, PWRCOM PA43, PWRUP PA44, PGOOD PA52 |
| FT5446U touch | I2C2, address 0x38; interrupt PA3, reset PA4, power PA5 |
| AW99703 frontlight | I2C1, address 0x36; enable PA53 |
| Wi-Fi | SDMMC1: CLK PA14, CMD PA15, D0–D3 PA16/PA17/PA12/PA13; power PA8, PDN PA9 |
| TF card | SDMMC2: CLK PA47, CMD PA48, D0–D3 PA49/PA50/PA45/PA46; detection PA51 |
| NS4150B amplifier | Enable PA54 |
| Digital microphone | PDM1: CLK PA55, DATA PA56; power PA57 |
| Buttons | PA34 interrupt; ADC channel 6 distinguishes three buttons |
| Power detection | USB presence PA6, charging status PA7, battery voltage ADC channel 11 |

I2C1 uses PA20/PA21 for SCL/SDA. I2C2 uses PA1/PA2.

Display signals: SDCLK PA24, SDLE PA25, SDOE PA26, SDSTL PA27, SDAT0–SDAT7 PA28/PA29/PA30/PA31/PA32/PA33/PA37/PA38, GMODE PA39, GDCLK PA40, and GDSP PA41.

## Build

Load the build environment from the SDK root, then build Hello World:

```powershell
. ./export.ps1
cd example/get-started/hello_world/rtt/project
scons --board=sf32lb57-epd_n16r16_hcpu -j8
```

The HCPU configuration selects the SDK E0470A03 display driver and FT5446U touch driver by default. The `acpu` directory provides the board configuration for ACPU child projects in SDK multicore builds.

Projects can enable Wi-Fi and TF card support as needed. SDMMC1 uses SDIO mode; board initialization enables the PA8 supply, and the Wi-Fi driver controls PDN on PA9. SDMMC2 uses SD card mode. Call `BSP_Microphone_PowerOn()` before recording with the digital microphone and `BSP_Microphone_PowerOff()` when finished. The power-on function must run in thread context.

## Display waveform

The board uses a binary waveform table. With the default `ptab.json`, program the SDK file `customer/peripherals/display/epd_e0470a03/waveform/epd_waveform.bin` at **0x12CE8000**. The `wave_table` partition reserves 256 KiB.

When a project overrides the partition table, use the generated `CUSTOM_EPD_WAVE_TABLE_START_ADDR` as the waveform programming address.
