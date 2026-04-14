# Dynamic EZIP Loading Example(LVGL V8)

English | [简体中文](README.md)

Source Path: example/multimedia/lvgl/ezip

## Overview
This example demonstrates how to dynamically load ezip format image files using LVGL in RT-Thread environment. The example supports:
- Loading ezip images from Flash file system or SD card
- Choosing to load images into SRAM or PSRAM
- Displaying images via file path or preloaded descriptor
- Dynamically switching image sources through finsh command line

## Features
- **Multiple Loading Methods**: Support direct loading from file or preloading to memory
- **Flexible Memory Management**: Optional SRAM or PSRAM storage for image data
- **File System Support**: Support reading images from Flash file system and SD card
- **Command Line Control**: Dynamically switch image sources through finsh commands

## Usage

### Supported Platforms
This example can run on the following development boards:
* sf32lb52-lchspi-ulp

### Build and Flash
Navigate to the example project directory and run the scons command to build:
```
scons --board=sf32lb52-lchspi-ulp -j8
```

### Prepare Image Files

Default provided image file:
- LVGL V8: `example.ezip`

To use custom images:
- Use sdk/tools/png2ezip/ezip.exe tool to convert PNG images to ezip format
  ```
  ezip -convert xxx\yyy.png -rgb565 -binfile 2 -binext .ezip -lvgl_version 8/9
  ```
  **Command Parameter Description**:
  - `-convert`: Specify the PNG image path to convert
  - `-rgb565`: Set output color format to RGB565
  - `-binfile 2`: Set output as binary file format
  - `-binext .ezip`: Specify output file extension as .ezip, can be customized up to 20 characters
  - `-lvgl_version`: Specify target LVGL version, supports 8 or 9
  
  **Usage Examples**:
  ```
  # Convert to LVGL V8 format
  ezip -convert images\logo.png -rgb565 -binfile 2 -binext .ezip -lvgl_version 8
  
  # Convert to LVGL V9 format
  ezip -convert images\logo.png -rgb565 -binfile 2 -binext .ezip -lvgl_version 9
  ```
- Place the image file in the `disk` directory and rebuild/reflash
- Or copy the image file to the SD card root directory (SD card insertion required)

## Example Output
If the example runs successfully, you will see the following output on the serial port:
```
(...system initialization information omitted...)

Register root to mtd device with base addr 0x12820000
mount fs on flash to root success
dynamic ezip loading example.
mount fs on tf card to /sdcard success
```

The LCD screen will display the loaded ezip image, centered on the screen.

## Using finsh Commands

After the program starts, you can dynamically switch image sources through serial port commands:

### Command Format
```
set_image [path] [file/dsc] [sram/psram]
```

### Parameter Description
- `path`: Image file path
- `file/dsc`: Loading method
  - `file`: Use file path directly (no preloading to memory)
  - `dsc`: Preload to memory as descriptor
- `sram/psram`: Memory type (optional, defaults to sram)
  - `sram`: Use SRAM
  - `psram`: Use PSRAM

**Important Note**: LVGL V8 and V9 have different file path format requirements:
- **LVGL V8**: Use file name directly, e.g., `example.ezip` or `/sdcard/image.ezip`
- **LVGL V9**: Need to add drive prefix, e.g., `A:/example.ezip` or `A:/sdcard/image.ezip`

### Usage Examples
```
# LVGL V8 - Load directly from file (no extra memory usage)
set_image example.ezip file

# LVGL V8 - Preload to PSRAM (improve display performance)
set_image example.ezip dsc psram

# LVGL V9 - Load from file (note the A: prefix required)
set_image A:/example.ezip file

# Load from SD card (LVGL V8)
set_image /sdcard/image.ezip dsc sram

# Load from SD card (LVGL V9, note the A: prefix required)
set_image A:/sdcard/image.ezip dsc sram
```

## API Reference

### load_ezip
```c
int load_ezip(lv_img_dsc_t *img_dsc, const char *ezip_path, int use_psram)
```
Load ezip image file to memory.

**Parameters**:
- `img_dsc`: LVGL image descriptor pointer
- `ezip_path`: Image file path
- `use_psram`: 1 to use PSRAM, 0 to use SRAM

**Return Value**: Returns 0 on success, -1 on failure

### unload_ezip
```c
void unload_ezip(lv_img_dsc_t *img_dsc, int use_psram)
```
Unload ezip image and release memory.

**Parameters**:
- `img_dsc`: LVGL image descriptor pointer
- `use_psram`: 1 to release from PSRAM, 0 to release from SRAM

## Notes
1. Ensure the development board has sufficient SRAM or PSRAM space to store image data
2. The ezip format must match the LVGL version (V8 or V9)
3. **LVGL V8 and V9 have different file path formats**:
   - V8: Use path directly, e.g., `example_v8.ezip`
   - V9: Need drive prefix, e.g., `A:/example_v9.ezip`
4. When using SD card, ensure it is properly formatted as FAT32
5. Preload method (dsc) has better performance but uses memory, file method (file) saves memory but has lower performance

## Troubleshooting
- **Image cannot be displayed**: Check if the ezip file exists and the path is correct
- **Memory allocation failed**: Check if SRAM or PSRAM has sufficient space
- **SD card mount failed**: Ensure the SD card is formatted as FAT32

For other issues, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference Documentation
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK Development Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)
- [LVGL Documentation](https://docs.lvgl.io/)
