# LCD Display Example
Source path: example/hal/lcd

## Supported Platforms
This example can run on the following development boards.
* sf32lb58-lcd_n16r32n1_dpi

## Overview
* Demonstrates how to drive a DPI(RGB) LCD panel directly using the LCDC HAL interface
* The screen first displays a vertical color-bar test pattern, then cycles through color bars / red / green / blue / white every 2 seconds

## Usage
### Build and Flash
Switch to the example project directory and run the scons command to build:
```
scons --board=sf32lb58-lcd_n16r32n1_dpi -j8
```
Run `build_sf32lb58-lcd_n16r32n1_dpi_hcpu\uart_download.bat` and select the COM port as prompted:
```
> build_sf32lb58-lcd_n16r32n1_dpi_hcpu\uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed build and flash instructions, refer to [](/quickstart/get-started.md).

### Expected Output
* Screen display: color bars first, then red / green / blue / white cycling
* Log output:
```
I/hal.lcd main: show color bars
I/hal.lcd main: show solid color 0
I/hal.lcd main: show solid color 1
I/hal.lcd main: show solid color 2
I/hal.lcd main: show solid color 3
I/hal.lcd main: show color bars
...

```

## Troubleshooting
* No display on screen
  1. Check whether the backlight is on
  2. Check the panel reset and power supply
  3. Verify the panel model matches the development board
* Incorrect colors / position
  1. Confirm the panel matches the default HTM-H070A20 (1024x600)
  2. When using a different panel, adjust the resolution, interface timing and control pins in the example accordingly

## Revision History
| Version | Date | Release Notes |
|:---|:---|:---|
| 0.0.1 | 06/2026 | Initial version |
| | | |
