# EZIP图片转换工具使用方法

## 1. 工具说明

路径：_$SDK_ROOT/tools/png2ezip/ezip.exe_

用途：将PNG图片转为EZIP格式或者PIXEL格式的二进制文件或者LVGL格式的C文件，二进制文件的前4字节为文件头，文件头后即EZIP或者PIXEL格式的数据，
文件头格式如下（小端序）：

### 文件头格式

|[31:21]   | [20:10]   | [9:5]      | [4:0]   
|----------|-----------|------------|------------
| 图片高度 | 图片宽度  | 保留       | 格式    |

### 格式取值

|格式  | 含义                 
|------|---------------------------
| 1    | 不带ALPHA的EZIP格式  |
| 2    | 带ALPHA的EZIP格式    |
| 4    | 不带ALPHA的PIXEL格式 |
| 5    | 带ALPHA的PIXEL格式   |


不带ALPHA的PIXEL格式支持RGB565和RGB888，带ALPHA的PIXEL格式支持ARGB565和ARGB888，详细格式如下（均为小端序），
工具在转换时会根据原始PNG文件是否带alpha而自动生成相应的格式，如果原图不带alpha，则生成的格式也不带alpha。

### RGB565
|[15:11]   | [10:5]   | [4:0]    
|----------|----------|--------------
| Red      | Green    | Blue     | 

### RGB888
|[23:16]   | [15:8]   | [7:0]     
|----------|----------|--------------
| Red      | Green    | Blue     | 

### ARGB565
|[23:16]   | [15:11]  | [10:5]   | [4:0]     
|----------|----------|----------|--------------
| Alpha    | Red      | Green    | Blue     | 


### ARGB888
|[31:24]   | [23:16]  | [15:8]   | [7:0]    
|----------|----------|----------|-------------
| Alpha    | Red      | Green    | Blue     | 


## 2. 使用方法

### 生成PIXEL格式二进制文件

- 生成RGB565或者ARGB565
```
ezip -convert png_filename.png -rgb565 -binfile 1
```

- 生成RGB888或者ARGB888
```
ezip -convert png_filename.png -rgb888 -binfile 1
```

完成后在工具目录下生成文件_png_filename.bin_


### 生成EZIP格式二进制文件

- 生成由RGB565或者ARGB565压缩得到的EZIP文件
```
ezip -convert png_filename.png -rgb565 -binfile 2
```

- 生成由RGB888或者ARGB888压缩得到的EZIP文件
```
ezip -convert png_filename.png -rgb888 -binfile 2
```

完成后在工具目录下生成文件_png_filename.bin_

### 生成PIXEL格式的LVGL C文件

- 生成RGB565或者ARGB565格式
```
ezip -convert png_filename.png -rgb565 -cfile 1 -section ROM3_IMG
```

- 生成由RGB888或者ARGB888
```
ezip -convert png_filename.png -rgb888 -cfile 1 -section ROM3_IMG
```

完成后在工具目录下生成文件_png_filename.c_ ，并指定段名 _.ROM3_IMG.png_filename_ ，例如

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

### 生成EZIP格式的LVGL C文件

- 生成由RGB565或者ARGB565压缩得到的EZIP格式C文件
```
ezip -convert png_filename.png -rgb565 -cfile 2 -section ROM3_IMG
```

- 生成由RGB888或者ARGB888压缩得到的EZIP格式C文件
```
ezip -convert png_filename.png -rgb888 -cfile 2 -section ROM3_IMG
```

完成后在工具目录下生成文件 _png_filename.c_ ，并指定段名 _.ROM3_IMG.png_filename_ ，例如

```
#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

SECTION(".ROM3_IMG.png_filename")

ALIGN(4)
const LV_ATTRIBUTE_MEM_ALIGN uint8_t png_filename_map[] = { 
...
```

### 生成可供硬件EZIP解压的GZIP BIN文件
- 例如对同目录下的file.bin进行压缩，使用如下命令
```
-gzip file.bin -length -noheader
```
制作后会在工具目录下生成文件_file.bin.gz_
该文件的前4字节是原始数据的长度，当进行硬件ezip解压（详见example/hal/ezip使用gzip解压）时，不需要传入到输入参数中，解压时可以根据该长度分配输出buffer。
4字节长度后面的都是gzip压缩数据，即直接作为硬件ezip的输入部分。

运行一次gzip得到的压缩数据，解压时必须完整传入硬件ezip的输入参数。所以当有比较大的数据需要解压时，输入输出buffer申请的空间可能不够。
建议先对数据进行分块，例如按10K拆分原始文件，分块后的数据，每一块单独压缩，解压时依次解压还原数据。

