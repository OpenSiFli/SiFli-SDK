# AES Example
Source code path: example/hal/aes
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ sf32lb52-lcd_n16r8

## Overview
<!-- Example introduction -->
This example demonstrates using AES for encryption and decryption, including:
+ AES encryption/decryption (synchronous mode)
     - AES CBC mode encryption/decryption
     - AES CTR mode encryption/decryption
     - AES ECB mode encryption/decryption
+ AES encryption (interrupt mode)
     - AES CBC mode encryption

## Example Usage
<!-- Instructions on how to use the example, such as which hardware pins to connect to observe waveforms, compilation and programming can reference related documentation.
For rt_device examples, also need to list the configuration switches used by this example, such as PWM examples using PWM1, need to enable PWM1 in the onchip menu -->

### Hardware Requirements
Before running this example, you need to prepare a development board supported by this example

### menuconfig Configuration


### Compilation and Programming
Switch to the example project directory and run the scons command to compile:
```
scons --board=sf32lb52-lcd_n16r8 -j32
```
Run `build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`, select the port as prompted to download:
```
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to the relevant introduction in [](/quickstart/get-started.md).

## Expected Results
<!-- Describe the example execution results, such as which LEDs will light up, what logs will be printed, so users can determine if the example is running normally. Results can be explained step by step combined with code -->
After the example starts, the serial port outputs the following:
1. AES CBC mode encryption and decryption:
```c
10-19 23:26:22:007    AES CBC Mode:
10-19 23:26:22:009    Raw data:
10-19 23:26:22:010     0x43, 0x4a, 0x4b, 0x2f, 0x31, 0x38, 0x32, 0x31,
10-19 23:26:22:012     0x39, 0x31, 0x31, 0x32, 0x31, 0x35, 0x36, 0x2e,
10-19 23:26:22:014    Encry. ciphertext:
10-19 23:26:22:016     0xd5, 0xb2, 0xe1, 0x75, 0x68, 0x6d, 0xb2, 0x03,
10-19 23:26:22:017     0x63, 0x96, 0xe1, 0x70, 0xc3, 0xf9, 0xf4, 0xf3,
10-19 23:26:22:019    Decry. plaintext:
10-19 23:26:22:063     0x43, 0x4a, 0x4b, 0x2f, 0x31, 0x38, 0x32, 0x31,
10-19 23:26:22:068     0x39, 0x31, 0x31, 0x32, 0x31, 0x35, 0x36, 0x2e,
```
2. AES CTR mode encryption and decryption:
```c
10-19 23:26:22:077    AES CTR Mode:
10-19 23:26:22:082    Raw data:
10-19 23:26:22:086     0x43, 0x4a, 0x4b, 0x2f, 0x31, 0x38, 0x32, 0x31,
10-19 23:26:22:091     0x39, 0x31, 0x31, 0x32, 0x31, 0x35, 0x36, 0x2e,
10-19 23:26:22:096    Encry. ciphertext:
10-19 23:26:22:100     0x86, 0xe9, 0x35, 0x58, 0xb9, 0xdd, 0x7d, 0x0b,
10-19 23:26:22:106     0x74, 0x85, 0x6c, 0x07, 0x1a, 0xe1, 0xbd, 0x9c,
10-19 23:26:22:111    Decry. plaintext:
10-19 23:26:22:114     0x43, 0x4a, 0x4b, 0x2f, 0x31, 0x38, 0x32, 0x31,
10-19 23:26:22:118     0x39, 0x31, 0x31, 0x32, 0x31, 0x35, 0x36, 0x2e,
```
3. AES ECB mode encryption and decryption:
```c
10-19 23:26:22:128    AES ECB Mode:
10-19 23:26:22:132    Raw data:
10-19 23:26:22:137     0x43, 0x4a, 0x4b, 0x2f, 0x31, 0x38, 0x32, 0x31,
10-19 23:26:22:142     0x39, 0x31, 0x31, 0x32, 0x31, 0x35, 0x36, 0x2e,
10-19 23:26:22:147    Encry. ciphertext:
10-19 23:26:22:152     0x6d, 0xf0, 0x28, 0xcd, 0x3c, 0xf5, 0x29, 0x79,
10-19 23:26:22:156     0x85, 0x06, 0xb9, 0xba, 0x35, 0x34, 0xf9, 0x46,
10-19 23:26:22:161    Decry. plaintext:
10-19 23:26:22:165     0x43, 0x4a, 0x4b, 0x2f, 0x31, 0x38, 0x32, 0x31,
10-19 23:26:22:170     0x39, 0x31, 0x31, 0x32, 0x31, 0x35, 0x36, 0x2e,
```
4. AES CBC encryption (interrupt mode):
```c
10-19 23:26:22:180    AES encryption interrupt mode:
10-19 23:26:22:184    Raw data:
10-19 23:26:22:188     0x43, 0x4a, 0x4b, 0x2f, 0x31, 0x38, 0x32, 0x31,
10-19 23:26:22:192     0x39, 0x31, 0x31, 0x32, 0x31, 0x35, 0x36, 0x2e,
10-19 23:26:22:197    Encry. ciphertext:
10-19 23:26:22:201     0xd5, 0xb2, 0xe1, 0x75, 0x68, 0x6d, 0xb2, 0x03,
10-19 23:26:22:206     0x63, 0x96, 0xe1, 0x70, 0xc3, 0xf9, 0xf4, 0xf3,
```

## Exception Diagnosis
```{warning}
AES directly reads and writes memory through hardware. If AES input and output memory uses PSRAM, attention must be paid to cache synchronization:  
1. Before executing AES run, cache synchronization is needed for input memory to ensure AES reads the required data.
2. After AES completion, cache synchronization may be needed for output memory to ensure correct data is read.
```

## Reference Documents
<!-- For rt_device examples, the RT-Thread official website documentation provides more detailed descriptions. Web page links can be added here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |10/2024 |Initial version |
| | | |
| | | |