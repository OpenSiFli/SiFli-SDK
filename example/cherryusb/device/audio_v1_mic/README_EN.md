# USB Microphone Example

Source path: example\cherryusb\device\audio_v1_mic

## Supported Platforms
+ sf32lb52-lcd_n16r8

## Overview
This example demonstrates the USB microphone recording functionality based on the USB audio class, the UAC protocol used is 1.0, including:
+ The host calls the recording function to record audio through the USB microphone.
+ PCM data is uploaded to the host via USB.

## Usage Instructions
### Hardware Requirements
Before running this example, you need to prepare:
+ A development board supported by this example ([Supported Platforms](quick_start)).
+ A USB-A to Type-C data cable with data transfer capability.
+ A host device that supports USB.

### menuconfig Configuration

1. Enable AUDIO CODEC and AUDIO PROC:
![AUDIO CODEC & PROC](./assets/mc_audcodec_audprc.png)
2. Enable AUDIO (`AUDIO`):
![AUDIO](./assets/mc_audio.png)
3. Enable AUDIO MANAGER (`AUDIO_USING_MANAGER`):
![AUDIO_USING_MANAGER](./assets/mc_audio_manager.png)
4. Modify the default AUDIO RX Path:
![AUDIO_MIC_USING_CODEC](./assets/mc_audio_audio_rx_path.png)

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:

> scons --board=sf32lb52-lcd_n16r8 -j32

Switch to the example `project/build_xx` directory and run `uart_download.bat`, then follow the prompts to select the port for downloading:

> ./uart_download.bat

> Uart Download

> please input the serial port num:5

For detailed steps on compilation and downloading, please refer to the relevant introduction in [Quick Start](quick_start).

## Expected Results
After starting the example:
The host connects to the board via the data cable, and a new microphone device (SiFli UAC DEMO) will appear in the audio input and output section of the host's device manager. When the host opens the recording device, it can select the microphone device for normal recording.

## Troubleshooting

## Reference Documents

## Change Log
| Version | Date   | Release Notes |
|:---|:---|:---|
| 0.0.1 | 08/2025 | Initial version |
| | | |
| | | |