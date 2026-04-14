# Dynamic EZIP Loading Example(LVGL V9)

English | [简体中文](README.md)

Source code path: example/multimedia/lvgl/ezip_v9

## Overview
This example demonstrates how to dynamically load ezip format image files using LVGL in an RT-Thread environment. The example supports:
- Loading ezip images from Flash filesystem or SD card
- Selecting to load images to SRAM or PSRAM
- Displaying images via file path or preloaded descriptor
- Dynamically switching image sources through finsh command line

## Features
- **Multiple Loading Methods**: Support direct file loading or preloading to memory
- **Flexible Memory Management**: Option to store image data in SRAM or PSRAM
- **Filesystem Support**: Support reading images from Flash filesystem and SD card
- **Command Line Control**: Dynamically switch image sources through finsh commands

## Usage

### Supported Platforms
The example can run on the following development boards:
* sf32lb52-lchspi-ulp

### Compilation and Flashing
Switch to the example project directory and run scons command to compile:
```
scons --board=sf32lb52-lchspi-ulp -j8
```

### Preparing Image Files

Default provided image files:
- LVGL V9: `example.ezip`

To use custom images, you can:
- Use the sdk/tools/png2ezip/ezip.exe tool to convert PNG images to ezip format
  ```
  ezip -convert xxx\yyy.png -rgb565 -binfile 2 -binext .ezip -lvgl_version 8/9
  ```
  **Command Parameter Description**:
  - `-convert`: Specify the PNG image path to convert
  - `-rgb565`: Set output color format to RGB565
  - `-binfile 2`: Set output to binary file format
  - `-binext .ezip`: Specify output file extension as .ezip, can be customized to an extension up to 20 characters
  - `-lvgl_version`: Specify target LVGL version, supports 8 or 9
  
  **Usage Examples**:
  ```
  # Convert to LVGL V8 format
  ezip -convert images\logo.png -rgb565 -binfile 2 -binext .ezip -lvgl_version 8
  
  # Convert to LVGL V9 format
  ezip -convert images\logo.png -rgb565 -binfile 2 -binext .ezip -lvgl_version 9
  ```
- Place the image file in the `disk` directory, recompile and flash
- Or copy the image file to SD card root directory (requires SD card insertion)

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

After program startup, you can dynamically switch image sources through serial commands:

### Command Format
```
set_image [path] [file/dsc] [sram/psram]
```

### Parameter Description
- `path`: Image file path
- `file/dsc`: Loading method
  - `file`: Use file path directly (no preloading to memory)
  - `dsc`: Preload to memory as descriptor
- `sram/psram`: Memory type (optional, default sram)
  - `sram`: Use SRAM
  - `psram`: Use PSRAM

**Important Note**: LVGL V8 and V9 have different file path format requirements:
- **LVGL V8**: Use filename directly, e.g., `example.ezip` or `/sdcard/image.ezip`
- **LVGL V9**: Need to add drive letter prefix, e.g., `A:/example.ezip` or `A:/sdcard/image.ezip`

### Usage Examples
```
# LVGL V8 - Load directly from file (no extra memory usage)
set_image example.ezip file

# LVGL V8 - Preload to PSRAM (improved display performance)
set_image example.ezip dsc psram

# LVGL V9 - Load from file (note: A: prefix required)
set_image A:/example.ezip file

# Load from SD card (LVGL V8)
set_image /sdcard/image.ezip dsc sram

# Load from SD card (LVGL V9, note: A: prefix required)
set_image A:/sdcard/image.ezip dsc sram
```

## API Description

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
Unload ezip image and free memory.

**Parameters**:
- `img_dsc`: LVGL image descriptor pointer
- `use_psram`: 1 to free from PSRAM, 0 to free from SRAM

## Notes
1. Ensure the development board has sufficient SRAM or PSRAM space to store image data
2. The ezip format must match the LVGL version (V8 or V9)
3. **LVGL V8 and V9 have different file path formats**:
   - V8: Use path directly, e.g., `example.ezip`
   - V9: Need drive letter prefix, e.g., `A:/example.ezip`
4. When using SD card, ensure the card is properly formatted as FAT32
5. Preload method (dsc) has better performance but uses memory, file method (file) saves memory but has lower performance

## Troubleshooting
- **Image not displaying**: Check if ezip file exists and path is correct
- **Memory allocation failure**: Check if SRAM or PSRAM has sufficient space
- **SD card mount failure**: Ensure SD card is formatted as FAT32

For other issues, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference Documentation
- [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK Development Guide](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)
- [LVGL Documentation](https://docs.lvgl.io/)
