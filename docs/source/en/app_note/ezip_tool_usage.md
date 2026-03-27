# EZIP Image Conversion Tool User Guide

## 1. Tool Overview

Path: _$SDK_ROOT/tools/png2ezip/ezip.exe_

Purpose: Converts PNG images into EZIP or PIXEL format binary files, or
LVGL-compatible C source files. For binary files, the first 4 bytes constitute
the file header, followed by the EZIP or PIXEL data. The file header format
(little-endian) is as follows:

### File Header Format

| [31:21]      | [20:10]     | [9:5]    | [4:0]  |
| ------------ | ----------- | -------- | ------ |
| Image Height | Image Width | Reserved | Format |

### Format Value

| Format | Description                 |
| ------ | --------------------------- |
| 1      | EZIP (without Alpha)        |
| 2      | EZIP (with Alpha)           |
| 4      | Pixel formats without alpha |
| 5      | Pixel formats with alpha    |


Pixel formats without alpha include RGB565 and RGB888, while pixel formats with
alpha include ARGB1555 and ARGB8888. Detailed formats (all little-endian) are
shown below. During conversion, the tool automatically selects the appropriate
format based on whether the source PNG file contains an alpha channel; if the
source image lacks alpha, the generated format will also exclude alpha.

### RGB565
| [15:11] | [10:5] | [4:0] |
| ------- | ------ | ----- |
| Red     | Green  | Blue  |

### RGB888
| [23:16] | [15:8] | [7:0] |
| ------- | ------ | ----- |
| Red     | Green  | Blue  |

### ARGB565
| [23:16] | [15:11] | [10:5] | [4:0] |
| ------- | ------- | ------ | ----- |
| Alpha   | Red     | Green  | Blue  |


### ARGB8888
| [31:24] | [23:16] | [15:8] | [7:0] |
| ------- | ------- | ------ | ----- |
| Alpha   | Red     | Green  | Blue  |


## 2. Usage

### Generate PIXEL format binary files

- Generate RGB565 or ARGB1555
```
ezip -convert png_filename.png -rgb565 -binfile 1
```

- Generate RGB888 or ARGB888
```
ezip -convert png_filename.png -rgb888 -binfile 1
```

Upon completion, the file _png_filename.bin_ is generated in the tool directory.


### Generate EZIP format binary files

- Generate an EZIP file compressed from RGB565 or ARGB565
```
ezip -convert png_filename.png -rgb565 -binfile 2
```

- Generate an EZIP file compressed from RGB888 or ARGB888
```
ezip -convert png_filename.png -rgb888 -binfile 2
```

Upon completion, the file _png_filename.bin_ is generated in the tool directory.

### Generate LVGL C files in PIXEL format

- Generate RGB565 or ARGB565 format
```
ezip -convert png_filename.png -rgb565 -cfile 1 -section ROM3_IMG
```

- Generate RGB888 or ARGB888
```
ezip -convert png_filename.png -rgb888 -cfile 1 -section ROM3_IMG
```

Upon completion, the file _png_filename.c_ is generated in the tool directory
with the specified section name _.ROM3_IMG.png_filename_, for example:

```
#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_eZIP_RGBARGB565A
#define LV_ATTRIBUTE_IMG_eZIP_RGBARGB565A
#endif
#define LV_COLOR_DEPTH_RGB565A 3
#define LV_COLOR_16_SWAP_RGB565A 0
SECTION(".ROM3_IMG.png_filename")

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_IMG_eZIP_RGBARGB565A uint8_t png_filename_map[] = { 
...
}
```

### Generate LVGL C files in EZIP format

- Generate an EZIP format C file compressed from RGB565 or ARGB565
```
ezip -convert png_filename.png -rgb565 -cfile 2 -section ROM3_IMG
```

- Generate a C file in EZIP format by compressing RGB888 or ARGB888 data.
```
ezip -convert png_filename.png -rgb888 -cfile 2 -section ROM3_IMG
```

Upon completion, the file `png_filename.c` is generated in the tool directory,
with the specified section name `.ROM3_IMG.png_filename`. For example:

```
#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

SECTION(".ROM3_IMG.png_filename")

ALIGN(4)
const LV_ATTRIBUTE_MEM_ALIGN uint8_t png_filename_map[] = { 
...
```

### Generate a GZIP BIN file for hardware EZIP decompression.
- To compress `file.bin` in the same directory, use the following command:
```
-gzip file.bin -length -noheader
```
The command generates `file.bin.gz` in the tool directory. The first 4 bytes of
this file represent the original data length. When performing hardware EZIP
decompression (refer to `example/hal/ezip` for GZIP decompression), these 4
bytes should not be passed as input parameters; instead, use this length to
allocate the output buffer. The data following the 4-byte length header is the
GZIP-compressed payload, which serves as the input for the hardware EZIP engine.

The compressed data from a single GZIP run must be passed in its entirety to the
hardware EZIP input parameters during decompression. Consequently, when
decompressing large datasets, the required input and output buffers may exceed
available memory. It is recommended to partition the data first—for example, by
splitting the original file into 10 KB blocks—compressing each block
independently, and decompressing them sequentially to restore the data.

