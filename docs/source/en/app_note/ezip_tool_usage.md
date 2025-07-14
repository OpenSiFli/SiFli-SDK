
# EZIP Image Conversion Tool Usage

## 1. Tool Description

Path: _$SDK_ROOT/tools/png2ezip/ezip.exe_

Purpose: Converts PNG images to either EZIP format, PIXEL format binary files, or LVGL format C files. The first 4 bytes of the binary file are the file header, followed by EZIP or PIXEL formatted data.  
The header format is as follows (little-endian):

### File Header Format

| [31:21]  | [20:10]  | [9:5]      | [4:0]      |
|----------|----------|------------|------------|
| Image Height | Image Width | Reserved | Format  |

### Format Values

| Format | Meaning                           |
|--------|-----------------------------------|
| 1      | EZIP format without ALPHA         |
| 2      | EZIP format with ALPHA            |
| 4      | PIXEL format without ALPHA        |
| 5      | PIXEL format with ALPHA           |

PIXEL format without ALPHA supports RGB565 and RGB888, while PIXEL format with ALPHA supports ARGB565 and ARGB888. The detailed format is as follows (all little-endian).  
The tool automatically generates the corresponding format based on whether the original PNG file includes an alpha channel. If the source image does not contain alpha, the resulting format will not contain alpha either.

### RGB565
| [15:11] | [10:5]  | [4:0]    |
|---------|---------|----------|
| Red     | Green   | Blue     |

### RGB888
| [23:16] | [15:8]  | [7:0]    |
|---------|---------|----------|
| Red     | Green   | Blue     |

### ARGB565
| [23:16] | [15:11] | [10:5]  | [4:0]    |
|---------|---------|---------|----------|
| Alpha   | Red     | Green   | Blue     |

### ARGB888
| [31:24] | [23:16] | [15:8]  | [7:0]    |
|---------|---------|---------|----------|
| Alpha   | Red     | Green   | Blue     |

## 2. Usage Instructions

### Generate PIXEL Format Binary File

- Generate RGB565 or ARGB565:
```
ezip -convert png_filename.png -rgb565 -binfile 1
```

- Generate RGB888 or ARGB888:
```
ezip -convert png_filename.png -rgb888 -binfile 1
```

After completion, the file _png_filename.bin_ will be generated in the tool directory.

### Generate EZIP Format Binary File

- Generate an EZIP file compressed with RGB565 or ARGB565:
```
ezip -convert png_filename.png -rgb565 -binfile 2
```

- Generate an EZIP file compressed with RGB888 or ARGB888:
```
ezip -convert png_filename.png -rgb888 -binfile 2
```

After completion, the file _png_filename.bin_ will be generated in the tool directory.

### Generate PIXEL Format LVGL C File

- Generate RGB565 or ARGB565 format:
```
ezip -convert png_filename.png -rgb565 -cfile 1 -section ROM3_IMG
```

- Generate RGB888 or ARGB888 format:
```
ezip -convert png_filename.png -rgb888 -cfile 1 -section ROM3_IMG
```

After completion, the file _png_filename.c_ will be generated in the tool directory, and the section name will be _.ROM3_IMG.png_filename_, for example:

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

### Generate EZIP Format LVGL C File

- Generate an EZIP format C file compressed with RGB565 or ARGB565:
```
ezip -convert png_filename.png -rgb565 -cfile 2 -section ROM3_IMG
```

- Generate an EZIP format C file compressed with RGB888 or ARGB888:
```
ezip -convert png_filename.png -rgb888 -cfile 2 -section ROM3_IMG
```

After completion, the file _png_filename.c_ will be generated in the tool directory, and the section name will be _.ROM3_IMG.png_filename_, for example:

```
#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

SECTION(".ROM3_IMG.png_filename")

ALIGN(4)
const LV_ATTRIBUTE_MEM_ALIGN uint8_t png_filename_map[] = { 
...
}
```

### Generate GZIP BIN File for Hardware EZIP Decompression

- For example, to compress the file _file.bin_ in the same directory, use the following command:
```
-gzip file.bin -length -noheader
```

After the operation, the file _file.bin.gz_ will be generated in the tool directory.  
The first 4 bytes of this file represent the length of the original data. When performing hardware EZIP decompression (refer to example/hal/ezip for GZIP decompression), the length will be used to allocate the output buffer, and the decompressed data will be used directly as input for hardware EZIP.

After running the gzip command, the decompressed data must be completely passed as input to the hardware EZIP. Therefore, if there is a large amount of data to decompress, the input and output buffers may not be enough.  
It is recommended to first split the data into chunks, for example, by splitting the original file into 10KB parts, compressing each chunk separately. Then, during decompression, process each chunk sequentially.

