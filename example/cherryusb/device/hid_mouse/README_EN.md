# USB Mouse Example

Source code path: example\cherryusb\device\hid_mouse

## Supported Platforms
<!-- Supported boards and chip platforms -->
+ sf32lb52-lcd_n16r8

## Overview
<!-- Example introduction -->
This example demonstrates enumerating a mouse device based on cherryusb USB-HID, including:
+ Press the KEY2 button on sf32lb52-lcd_n16r8 to sequentially trigger left button press, right button press, middle button press, wheel scroll, mouse circle, left button press and move right.

## Usage of the Example
<!-- Explain how to use the example, such as connecting hardware pins to observe waveforms, compilation and burning can reference related documents.
For rt_device examples, also list the configuration switches used in this example, such as PWM example uses PWM1, need to enable PWM1 in the onchip menu -->

### Hardware Requirements
Before running this example, prepare:
+ A development board supported by this example ([Supported Platforms](quick_start)).
+ Two USB-typec data cables with data transmission capability, one for burning and log viewing, one for connecting to the host.
+ A host device that supports mouse input.

### menuconfig Configuration

### Compilation and Burning
Switch to the example project directory, run the scons command to compile:
```c
scons --board=sf32lb52-lcd_n16r8 -j32
```
Switch to the example `project/build_xx` directory, run `uart_download.bat`, and select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:
```
For detailed steps on compilation and download, please refer to the introduction in [Quick Start](quick_start).

## Expected Results of the Example
<!-- Explain the example running results, such as which lights will light up, which logs will be printed, so that users can judge if the example is running normally, running results can be explained step by step combined with code -->
After the example starts:
The host connects to the board via data cable, and a new mouse device will appear in the Mouse and other pointing devices section of the host's device manager.
Press the KEY2 button to sequentially trigger left button press, right button press, middle button press, wheel scroll, mouse circle, left button press and move right.


## Exception Diagnosis


## Reference Documents
<!-- For rt_device examples, RT-Thread official website documents provide detailed explanations, you can add web links here, for example, refer to RT-Thread's [RTC Documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| :------ | :------ | :-------------- |
| 0.0.1   | 08/2025 | Initial version |
|         |         |                 |
|         |         |                 |
