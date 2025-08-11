# Hello World Example (RT-Thread)

Source Code Path: example/get-started/hello_world/rtt

## Overview
The Hello_world application will print "hello world!" on the board's serial port.

## Usage

### Supported Development Boards
This example can run on the following development boards:
- Any board (including `pc`)

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```
scons --board=sf32lb52-lcd_n16r8 -j32
```

For simulator:
```
scons --board=pc -j32
```

## Example Output
If the example runs successfully, you will see the following output on the serial port:
```
Serial:c2,Chip:4,Package:3,Rev:3  Reason:00000000

 \ | /
- SiFli Corporation
 / | \     build on Jul 23 2025, 2.4.0 build 92567879
 2020 - 2022 Copyright by SiFli team
mount /dev sucess
[I/drv.rtc] PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
[I/drv.rtc] RTC use LXT RTC_CR=00000001

[I/drv.rtc] Init RTC, wake = 0

[I/drv.audprc] init 00 ADC_PATH_CFG0 0x606

[I/drv.audprc] HAL_AUDPRC_Init res 0

[I/drv.audcodec] HAL_AUDCODEC_Init res 0

[I/TOUCH] Regist touch screen driver, probe=1202611d 
call par CFG1(3313)

fc 9, xtal 2000, pll 2067

call par CFG1(3313)

fc 7, xtal 2000, pll 1656

Hello world!
msh />Serial:c2,Chip:4,Package:3,Rev:3  Reason:00000000
```

## Troubleshooting
No specific troubleshooting information available. For any issues, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference Documentation
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK Development Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)