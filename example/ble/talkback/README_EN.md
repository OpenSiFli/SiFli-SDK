# BLE Intercom Example

Source path: example/ble/talkback

## Supported Platforms
<!-- Supported boards and chip platforms -->
+ sf32lb52-lchspi-ulp
+ sf32lb52-nano series
+ sf32lb52-lcd series
+ sf32lb56-lcd series
+ sf32lb58-lcd series


## Overview
<!-- Example introduction -->
1. This example demonstrates how to implement BLE-based voice intercom functionality on this platform.
2. It leverages BLE 5.0's Periodic Advertising and Periodic Advertising Sync features to achieve low-power, low-latency wireless intercom.
3. Supports multi-device networking, with up to 3 devices simultaneously connected for intercom.
4. Uses Opus audio codec technology to ensure high-quality voice transmission.

## Using the Example
<!-- Instructions on how to use the example, such as which hardware pins to connect for waveform observation, compile and flash references.
For rt_device examples, list the configuration switches used by this example, e.g., PWM example uses PWM1 and needs to be enabled in the onchip menu -->
1. After power-on, this example automatically starts scanning to discover nearby intercom devices and establishes connections.
2. It also starts advertising so that other devices can discover this device.
3. Press the KEY1 button on the development board (corresponding to the silkscreen on the board) to start speaking; press KEY1 again to stop speaking.
4. When other devices are speaking, this device will automatically play their audio.
5. Supports system status viewing and debugging via Finsh commands.

### Hardware Requirements
Before running this example, you need:
+ A development board supported by this example
+ At least two development boards for intercom testing
+ Audio input/output devices (microphone and speaker)

### Hardware Connections
Connect the microphone and speaker according to the development board guide.
[sf32lb52-lcd_n16r8](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/supported_boards/boards/sf32lb52-lcd_n16r8/doc/index.html#)
[sf32lb52-nano_n16r16](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/supported_boards/boards/sf32lb52-nano_n16r16/doc/index.html#)
[sf32lb52-lchspi-ulp](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/supported_boards/boards/sf32lb52-lchspi-ulp/doc/index.html#)


### menuconfig Configuration
1. Enable Bluetooth
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro: `CONFIG_BLUETOOTH`
        - Function: Enables Bluetooth functionality
2. Enable audio-related features:
    - Path: Sifli middleware → Audio
    - Enable: Enable audio server
        - Macro: `CONFIG_USING_AUDIO_SERVER`
        - Function: Enables audio service
    - Enable: Enable Opus codec
        - Macro: `CONFIG_PKG_LIB_OPUS`
        - Function: Enables Opus audio codec library
3. Enable WebRTC audio processing:
    - Path: Sifli middleware → Audio → WebRTC
    - Enable: Enable WebRTC AGC
        - Macro: `CONFIG_PKG_USING_WEBRTC`, `CONFIG_WEBRTC_AGC_FIX`
        - Function: Enables automatic gain control
    - Enable: Enable WebRTC AECM
        - Macro: `CONFIG_WEBRTC_AECM`
        - Function: Enables acoustic echo cancellation
4. Enable buttons:
    - Path: Sifli middleware
    - Enable: Enable button library
        - Macro: `CONFIG_USING_BUTTON_LIB`
        - Function: Enables button library

**Note**: This system is based on BLE 5.0 Periodic Advertising and does not require traditional GATT client and connection manager functionality.

### Compiling and Flashing
Switch to the example's project directory and run the scons command to compile:
```c
> scons --board=eh-lb525 -j32
```
Switch to the example's `project/build_xx` directory and run `uart_download.bat`, then select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed compilation and download steps, please refer to the [Quick Start Guide](/quickstart/get-started.md).

## Expected Results
<!-- Describe the expected results, such as which LEDs will light up, what logs will be printed, so users can verify if the example is running correctly. Results can be explained step by step with code references -->
After the example starts:
1. The system automatically initializes the BLE and audio modules.
2. Starts scanning for nearby intercom devices and establishing connections.
3. When the KEY1 button is pressed to start speaking, other devices can hear clear audio; press KEY1 again to stop speaking.
4. When other devices are speaking, this device will automatically play the audio.
5. The serial port outputs relevant log information for debugging and system status monitoring.

## Troubleshooting
1. If other devices cannot be found, check:
   - Ensure all devices are on the same network (using the same DEFAULT_NETWORK_CODE)
   - Verify that BLE functionality is properly enabled
   - Confirm that devices are within effective range
   
2. If audio quality is poor, check:
   - Whether the microphone and speaker are correctly connected
   - Whether audio parameter configurations (sample rate, bit depth, etc.) are appropriate
   - Whether there is electromagnetic interference
   
3. If the button is unresponsive, check:
   - Whether the button pin configuration is correct
   - Whether the button driver is properly loaded
   - Whether the button hardware connection is reliable

## Reference Documentation
<!-- For rt_device examples, the RT-Thread official website provides detailed documentation. Add web links here. For example, refer to RT-Thread RTC documentation -->
- [SiFli BLE Development Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/middleware/bt_service.html)
- Opus Audio Codec Technical Documentation
- RT-Thread Audio Framework Documentation

## Revision History
|Version |Date   |Description |
|:---|:---|:---|
|0.0.1 |12/2025 |Initial version |
| | | |
| | | |
