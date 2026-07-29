# USB UVC + UAC Host Example

English | [简体中文](README.md)

Source Code Path: example/cherryusb/host/uvc_uac_host

## Overview
This example demonstrates integrated USB Host capture for UVC video + UAC audio based on CherryUSB:

- UVC video stream capture (YUYV/MJPEG)
- UAC microphone stream capture
- Runtime start/stop of video and audio streams through finsh commands
- Serial logs for frame buffer address, frame size, and video FPS

By default, startup performs USB Host initialization and enumeration only. Video and audio streams are started by commands.
This example supports both combined A/V testing on one device and separate testing with different devices.

## Hardware Requirements
- SiFli development board with USB Host support (for example, sf32lb52)
- USB data cable
- UVC camera (for video testing)
- UAC audio input device (for audio testing, can be separate from the camera)
- Serial terminal tool

## Usage

### Supported Boards
This example can run on the following board:
- sf32lb52-lcd_n16r8

### Build and Flash
Switch to the example `project` directory and build with scons:

> scons --board=sf32lb52-lcd_n16r8 -j8

Switch to `project/build_xx`, run `uart_download.bat`, and select the serial port as prompted:

> ./uart_download.bat

> Uart Download

> please input the serial port num:5

For detailed build and flashing steps, refer to [Quick Start](quick_start).

### Running Steps
1. Flash firmware to the board and open serial terminal (baud rate is usually 1000000)
2. Reset the board and confirm `cherryusb host demo!` appears
3. Connect USB device(s) to the board USB Host port and wait for enumeration:
	- Option A: UVC camera with microphone (combined test)
	- Option B: regular UVC camera + separate UAC device (separate test)

4. Start the video stream in finsh:
	- `usbh_uvc_start yuyv`: start YUYV stream (320x240)
	- `usbh_uvc_start mjpeg`: start MJPEG stream (320x240)

5. Start the audio stream:
	- `usbh_uac_start <freq>`: start mic stream with sample rate (for example `usbh_uac_start 16000`)
	- `usbh_uac_volume <volume> <is_tx>`: set volume (optional)

6. Stop streams when needed:
	- `usbh_uac_stop`: stop mic stream
	- `usbh_uvc_stop`: stop video stream

### Debug Notes
- The video thread continuously prints video frame buffer address and frame size
- The audio thread continuously prints audio frame buffer address and frame size
- The FPS thread prints current `fps` every 5 seconds

## Example Output
When running successfully, serial output is similar to the following (exact logs vary by camera):
```
cherryusb host demo!
...
msh />usbh_uvc_start mjpeg
frame buf:0x2006d5ac,frame len:6534
frame buf:0x200755ac,frame len:6408
...
fps:30
vc:300

msh />usbh_uac_start 16000
frame buf:0x2007d5ac,frame len:384
frame buf:0x2007d72c,frame len:384

msh />usbh_uac_stop
msh />usbh_uvc_stop
```

## Troubleshooting
- No output after `usbh_uvc_start`: check whether the camera is enumerated and supports UVC.
- Video frame size remains 0: try switching stream format (`yuyv`/`mjpeg`) or another camera.
- `usbh_uac_start` fails: check whether the currently connected device supports UAC audio input.
- Video works but audio does not (or vice versa): for combined test, verify the device exposes both UVC and UAC interfaces; for separate test, verify each device is enumerated successfully.

If needed, please submit an [issue on GitHub](https://github.com/OpenSiFli/SiFli-SDK/issues).

## Reference Documentation
- [SiFli-SDK Quick Start Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK Development Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)
- [CherryUSB Documentation](https://cherryusb.readthedocs.io/)