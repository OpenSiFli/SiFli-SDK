# PDM Example

Source path: example/multimedia/audio/4pdm

## Supported Platforms
+ sf32lb56-lcd_a128r12n1

## Overview

This example demonstrates two or four PDM mic recording, using Ankai's algorithm by default, which includes:  
+ Recording through PDM and saving the recording as a wav file.  
+ Reading the recorded data and playing it back.

## Example Usage

### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported platforms](quick_start)).
+ PDM
+ Speaker.

### menuconfig Configuration

1. Enable PDM (two mics using PDM1 left and right, four mics using PDM1、PDM2)
2. Enable AUDIO CODEC and AUDIO PROC:
3. Enable AUDIO
4. Enable AUDIO MANAGER

```{tip}
Configurations 3 and 4 are for playing recorded files. In this example, recording playback uses Audio manager interface.
```  

### Hardware Connection\PIN CONFIG
Check which port the PDM uses. If the bsp_pinmux.c in the board is not configured, you need to configure the pin function to PDM. 
* For more detailed pin definitions, please refer to
[sf32lb52-nano](https://wiki.sifli.com/board/sf32lb52x/SF32LB52-DevKit-Nano.html)
[sf32lb52-lcd](https://wiki.sifli.com/board/sf32lb52x/SF32LB52-DevKit-LCD.html)
[sf32lb56-lcd](https://wiki.sifli.com/board/sf32lb56x/SF32LB56-DevKit-LCD.html)
[sf32lb58-lcd](https://wiki.sifli.com/board/sf32lb58x/SF32LB58-DevKit-LCD.html)

take `f32lb56-lcd_a128r12n1` as a example：
```C
    /* PIN CONFIG */
    HAL_PIN_Set(PAD_PA69, PDM1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA64, PDM1_DATA, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA73, PDM2_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA71, PDM2_DATA, PIN_PULLDOWN, 1);
````
### Compilation and Programming
Switch to the example project directory and run the scons command to execute compilation:
```c
> scons --board=f32lb56-lcd_a128r12n1 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`, select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to the relevant introduction in [Quick Start](quick_start).

## Expected Results
1. Start recording command: `pdm open [PDM interface selection] [number of channels] [algorithm selection]`  
For example:  
pdm open pdm1 1 raw  
Indicates configuring PDM1 for mono recording, generating pdm1_1.wav, without any algorithm, suitable for factory testing.  
pdm open pdm1 2 raw  
Indicates configuring PDM1 for stereo recording, generating pdm1_2.wav, without any algorithm, suitable for factory testing.  
pdm open pdm2 1 raw  
Indicates configuring PDM2 for mono recording, generating pdm2_1.wav, without any algorithm, suitable for factory testing.  
pdm open pdm2 2 raw  
Indicates configuring PDM2 for stereo recording, generating pdm2_2.wav, without any algorithm, suitable for factory testing.  
pdm open pdm12 4 raw  
Indicates configuring PDM1/PDM2 each for stereo, without any algorithm, suitable for factory testing, only outputting log.  

pdm open pdm1 1 anyka  
Indicates configuring PDM1 for mono recording, using Anyka algorithm, output as mono, generating anyka_pdm1_1.wav.  
pdm open pdm1 2 anyka  
Indicates configuring PDM1 for stereo recording, using Anyka algorithm, output as mono, generating anyka_pdm1_2.wav.  
pdm open pdm2 1 anyka  
Indicates configuring PDM2 for mono recording, using Anyka algorithm, output as mono, generating anyka_pdm2_1.wav.  
pdm open pdm2 2 anyka  
Indicates configuring PDM2 for stereo recording, using Anyka algorithm, output as mono, generating anyka_pdm2_2.wav.  
pdm open pdm12 4 anyka  
Indicates configuring PDM1/PDM2 each for stereo, using Anyka algorithm, output as mono, generating anyka_4.wav.  

2. Stop recording command: `pdm close`  
  pdm close  
Indicates stopping the recording.  

3. Playback recording command: `pdm play [filename]`  
For example:  
  pdm play anyka_pdm2_2.wav  

After recording, executing playback will allow normal recording and playback.  

## Exception Diagnosis


## Reference Documentation
<!-- For rt_device examples, the RT-Thread official website documentation provides more detailed explanations, you can add webpage links here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |12/2024 |Initial version |
| | | |
| | | |