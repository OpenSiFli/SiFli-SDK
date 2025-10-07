
The HAL EPIC module provides an abstract software interface to operate the hardware EPIC module. EPIC is a 2D graphics engine with the following features:

## Main Features

- Alpha blending of two images and saving the result to an output buffer
- Rotating an image (foreground) around any point and blending the rotated image with a background image, saving the blended result to the output buffer
- Scaling (enlarge/reduce) the foreground image, blending the scaled image with the background, and saving the result to the output buffer
- Supports combined rotation and scaling in a single GPU operation without requiring an intermediate buffer
- Filling a given buffer with opaque or semi-transparent colors
- All graphics operations support both polling and interrupt modes
- Automatic color format conversion if input and output color formats differ
- The two images for blending can have different sizes and partially overlap; you can also specify a sub-region of the blended area as the output region
- The background and output images can share the same buffer, e.g., both using the frame buffer
- Input image format supports EZIP
- Supports fractional coordinate blending (not supported on 55X)


## Input/Output Limitations
| Feature      | Supported Formats                |  55X                     |  58X   |  56X   |  52X   |
|--------------|----------------------------------|--------------------------|--------|--------|--------|
| Horizontal Scaling | All color formats supported by the chip | 3.8, shrink by 8x, enlarge by 256x, precision 1/256 | 10.16, shrink by 1024x, enlarge by 65536x, precision 1/65536 | Same as 58X | Same as 58X |
| Vertical Scaling   | All color formats supported by the chip | Horizontal and vertical scaling factors must be the same | 10.16, shrink by 1024x, enlarge by 65536x, precision 1/65536, <br>horizontal and vertical scaling factors can differ | Same as 58X | Same as 58X |
| Rotation           | All except EZIP, YUV <br>A4/A8 mask cannot rotate | [0 ~ 3600], unit is 0.1 degree | Same as 55X | Same as 55X | Same as 55X |
| Horizontal Mirror  | All color formats supported by the chip | Supported | Supported | Supported | Supported |
| Vertical Mirror    | All except EZIP format | Not supported | Not supported | Not supported | Supported |


```{note}
- Rotation and scaling can be performed simultaneously, supporting the same or any anchor point.
- Mirroring supports any anchor point, but cannot be combined with rotation or scaling.
- The union of the foreground, background, and output regions must not exceed 1024*1024 pixels (the foreground refers to the image region after transformation around any anchor point, including the anchor and the original image).
> For example, after rotating and scaling the foreground image around an anchor point outside the image, the union with the background and output regions must not exceed 1024.

![EPIC Limitation Explanation](../../assets/epic_limitation.png)

```

## Supported Color Formats
| Input Color Format Supported         |  55X   |  58X   |  56X   |  52X   |
|-------------------------------------|--------|--------|--------|--------|
| RGB565/ARGB8565/RGB888/ARGB88888    |   Y    |   Y    |   Y    |   Y    |
| L8                                  |   N    |   Y    |   Y    |   Y    |
| A4/A8 (Mask, Overwrite, Fill)       |   N    |   Y    |   Y    |   Y    |
| YUV (YUYV/UYVY/iYUV)                |   N    |   N    |   Y    |   Y    |
| A2   (Fill)                         |   N    |   N    |   N    |   Y    |


| Output Color Format Supported       |  55X   |  58X   |  56X   |  52X   |
|-------------------------------------|--------|--------|--------|--------|
| RGB565/ARGB8565/RGB888/ARGB88888    |   Y    |   Y    |   Y    |   Y    |


## Image Handling Recommendations
- Add a transparent (or background color) border around images to be rotated or scaled to prevent edge clipping and aliasing
- To avoid discontinuity during scaling, the scaling factor difference for continuous scaling should be greater than 1/256 (i.e., scaling precision should not exceed 1/256)
- Although rotation and scaling can be performed simultaneously, it is recommended to perform only one transformation at a time for better output quality
- When enlarging, use the top-left corner of the image as the anchor point to prevent anchor jitter
- EZIP format images cannot be used for rotation

For detailed API documentation, refer to [EPIC](#hal-epic).


## Color Storage Format in SRAM

|        | bit31~bit24 | bit23~bit16 | bit15~bit8 | bit7~bit0 |
| ------ | ------ | ------ | ------ | ------ |
| RGB565   |    /    |    /       | R4~R0G5~G3         | G2~G0B4~B0 |
| ARGB8565 |    /    | A7 ~ A0    | R4~R0G5~G3         | G2~G0B4~B0 |
| RGB888   |    /    | R7 ~ R0    | G7 ~ G0            | B7 ~ B0 |
| ARGB8888 | A7 ~ A0 | R7 ~ R0    | G7 ~ G0            | B7 ~ B0 |
| A8       | D7 ~ D0 | C7~C0      | B7~B0              | A7~A0 |
| A4       |    /    |   /        | D3~D0C3~C0         | B3~B0A3~A0 |
| A2       |    /    |   /        | H1H0G1G0F1F0E1E0   | D1D0C1C0B1B0A1A0 |

```{note}
Color data is stored tightly. In A2/A4/A8 formats, A–H represent pixel points (displayed from left to right).
```

## Using HAL EPIC

First, call {c:func}`HAL_EPIC_Init` to initialize HAL EPIC. In the {c:type}`EPIC_HandleTypeDef` structure, specify the EPIC instance (i.e., the hardware EPIC module to use). The chip has only one EPIC instance: {c:macro}`hwp_epic`.
After initialization, you can call various graphics operation interfaces to process data.

For example,
```c
static EPIC_HandleTypeDef epic_handle;

void init_epic(void) 
{ 	// Initialize driver and enable EPIC IRQ
    HAL_NVIC_SetPriority(EPIC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EPIC_IRQn);
    
    epic_handle.Instance = hwp_epic;
    HAL_EPIC_Init(&epic_handle);
}

/* EPIC IRQ Handler */
void EPIC_IRQHandler(void)
{
    HAL_EPIC_IRQHandler(&epic_handle);
}

```

{c:func}`HAL_EPIC_BlendStartEx_IT` is used for interrupt mode fill, blend, rotate, and scale operations. You need to call {c:func}`HAL_EPIC_IRQHandler` in the interrupt service routine to handle the interrupt.


### Blending Example
As shown in Figure 1, the `blend_img` example overlays part of an image onto a background:
1. The foreground image region is (10, 20)~(59,79), the background image region is (0,0)~(99,99), and the blend region is (5,10)~(44,59). All coordinates are in the same coordinate system.
2. The foreground image is blended with the background at opacity 100.

After blending, the color values in the region (5,10)~(44,59) are sequentially written to the background buffer. The overlapping part (the crossed area, i.e., [10,20]~[44,59]) contains the blended color, while the non-overlapping part retains the original background color.

Note that all data buffers point to the top-left pixel of the corresponding region. For example, p_fg_img->data points to the color value of the foreground image at (10,20), and output_img.data points to the top-left pixel of the output region, i.e., (5,10).
output_img.width and output_img.total_width: output_img.width is the width of the output region (44-5+1=40), while output_img.total_width is the width of the output buffer (100, since the buffer is 100x100). EPIC will skip the remaining 60 pixels after writing 40 pixels per line and continue to the next line.
fg_img and bg_img width and total_width have the same meaning.


![Figure 1: Blending](../../assets/epic_blend.png)




```c
void epic_cplt_callback(EPIC_HandleTypeDef *epic)
{
    /* release the semaphore to indicate epic operation done */
    sema_release(epic_sema);
}

/* blend the foreground with background image using 100 opacity (0 is transparent, 255 is opaque)
 * output buffer is same as background image buffer, usually they're both frame buffer.
 * 
 */
void blend_img(void)
{
    EPIC_LayerConfigTypeDef layers[2];

    EPIC_LayerConfigTypeDef *p_bg_img = &layers[0];
    EPIC_LayerConfigTypeDef *p_fg_img = &layers[1];
    EPIC_LayerConfigTypeDef output_img;
    HAL_StatusTypeDef ret;         
    uint32_t buffer_start_offset;    
    
    /* foreground image, its coordinate (10,20)~(59,79), buffer size is 50*60 */
    HAL_EPIC_LayerConfigInit(p_fg_img);
    p_fg_img->data = fg_img_buf;
    p_fg_img->x_offset = 10;
    p_fg_img->y_offset = 20;
    p_fg_img->width = 50;
    p_fg_img->height = 60;
    p_fg_img->total_width = 50;
    p_fg_img->color_mode = EPIC_COLOR_RGB565;
    p_fg_img->alpha = 100;
    
    /* background image, its coordinate (0,0)~(99,99), buffer size is 100*100 */
    HAL_EPIC_LayerConfigInit(p_bg_img);
    p_bg_img->data = bg_img_buf;
    p_bg_img->x_offset = 0;
    p_bg_img->y_offset = 0;
    p_bg_img->width = 100;
    p_bg_img->height = 100;
    p_bg_img->total_width = 100;
    p_bg_img->color_mode = EPIC_COLOR_RGB565;
    
    /* output image, share the same buffer as bg_img_buf,
       output area is (5,10)~(44,59), buffer size is 100*100 */
    HAL_EPIC_LayerConfigInit(&output_img);
    /* topleft pixel is (5, 10), skip (10*100+5) pixels */
    buffer_start_offset = (10 - 0) * 100 * 2 + (5 - 0) * 2;
    output_img.data = (uint8_t *)((uint32_t)bg_img_buf + buffer_start_offset);
    /* output area topleft coordinate */
    output_img.x_offset = 5;
    output_img.y_offset = 10;
    /* output area width */
    output_img.width = 40;
    /* output area height */
    output_img.height = 50;
    /* output buffer width, it's different from output_img.width */
    output_img.total_width = 100;
    output_img.color_mode = EPIC_COLOR_RGB565;
    
    /* set complete callback, and start EPIC */
    epic_handle.XferCpltCallback = epic_cplt_callback;
    ret = HAL_EPIC_BlendStartEx_IT(&epic_handle, &layers, 2, &output_img);
    /* check ret value if any error happens */
    ...
    /* wait for completion */
    sema_take(epic_sema);
}

```


### Rotation Example

As shown in Figure 2, the `rotate_img` example rotates the foreground image at (10,20)~(59,79) clockwise by 30 degrees around its center, blends it with the background, and updates the corresponding region in the background buffer. Pixels outside the rotated region retain the background color.
Since the bounding rectangle of the rotated image expands ([x0,y0]~[x1,y1]), you can simply set the output region to the maximum; HAL will automatically calculate the bounding rectangle. If the background and output buffers are the same, only the pixels covered by the rotated region are updated.

![Figure 1: Rotation](../../assets/epic_rot.png)

```c
/* rotate the foreground image by 30 degree (clockwisely) and blend it with background using 100 opacity (0 is transparent, 255 is opaque)
 * output data is written back to background image buffer, it can also output to another buffer like blend_img_1.
 * 
 */
void rotate_img(void)
{
    EPIC_LayerConfigTypeDef layers[2];

    EPIC_LayerConfigTypeDef *p_bg_img = &layers[0];
    EPIC_LayerConfigTypeDef *p_fg_img = &layers[1];

    EPIC_LayerConfigTypeDef output_img;
    HAL_StatusTypeDef ret;
    
    /* foreground image, its coordinate (10,20)~(59,79) before rotation, buffer size is 50*60 */
    HAL_EPIC_LayerConfigInit(p_fg_img);
    p_fg_img->data = fg_img_buf;
    p_fg_img->x_offset = 10;
    p_fg_img->y_offset = 20;
    p_fg_img->width = 50;
    p_fg_img->height = 60;
    p_fg_img->total_width = 50;
    p_fg_img->color_mode = EPIC_COLOR_RGB565;
    p_fg_img->alpha = 100;
    /* foreground image is rotated by 30 degree around its center */
    p_fg_img->transform_cfg.angle = 300;
    p_fg_img->transform_cfg.pivot_x = p_fg_img->width / 2;
    p_fg_img->transform_cfg.pivot_y = p_fg_img->height / 2;
    p_fg_img->transform_cfg.scale_x = EPIC_INPUT_SCALE_NONE;
    p_fg_img->transform_cfg.scale_y = EPIC_INPUT_SCALE_NONE;    


    /* background image, its coordinate (0,0)~(99,99), buffer size is 100*100 */
    HAL_EPIC_LayerConfigInit(p_bg_img);
    p_bg_img->data = bg_img_buf;
    p_bg_img->x_offset = 0;
    p_bg_img->y_offset = 0;
    p_bg_img->width = 100;
    p_bg_img->height = 100;
    p_bg_img->total_width = 100;
    p_bg_img->color_mode = EPIC_COLOR_RGB565;
    
    /* output image, its coordinate (0,0)~(99,99), share same buffer as background image */
    HAL_EPIC_LayerConfigInit(&output_img);
    output_img.data = bg_img_buf;
    output_img.x_offset = 0;
    output_img.y_offset = 0;
    output_img.width = 100;
    output_img.height = 100;
    output_img.total_width = 100;
    output_img.color_mode = EPIC_COLOR_RGB565;
    
    
    /* set complete callback, and start EPIC */
    epic_handle.XferCpltCallback = epic_cplt_callback;
    ret = HAL_EPIC_BlendStartEx_IT(&epic_handle, &layers, 2, &output_img);
    /* check ret value if any error happens */
    ...
    /* wait for completion */
    sema_take(epic_sema);
}
```

### Scaling Example

As shown in Figure 3, the `scale_down_img` example scales the foreground image at (10,20)~(59,79) down to 71% of its original size in both horizontal and vertical directions, keeping the center point unchanged.
Similar to rotation, you can set the output region to the maximum. If the output buffer shares the background buffer, HAL will only update the pixels in the scaled region ([x0,y0]~[x1,y1]).

![Figure 1: Scaling](../../assets/epic_scaling.png)


```c

/* scale down the foreground image by 1.4 and blend it with background using 100 opacity (0 is transparent, 255 is opaque)
 * output data is written back to background image buffer, it can also output to another buffer like blend_img_1.
 * 
 */
void scale_down_img(void)
{
    EPIC_LayerConfigTypeDef layers[2];

    EPIC_LayerConfigTypeDef *p_bg_img = &layers[0];
    EPIC_LayerConfigTypeDef *p_fg_img = &layers[1];

    EPIC_LayerConfigTypeDef output_img;
    HAL_StatusTypeDef ret;
    
    /* foreground image, its coordinate (10,20)~(59,79) before scaling */
    HAL_EPIC_LayerConfigInit(p_fg_img);
    p_fg_img->data = fg_img_buf;
    p_fg_img->x_offset = 10;
    p_fg_img->y_offset = 20;
    p_fg_img->width = 50;
    p_fg_img->height = 60;
    p_fg_img->total_width = 50;
    p_fg_img->color_mode = EPIC_COLOR_RGB565;
    p_fg_img->alpha = 100;
    /* no rotation, both X and Y direction are scaled down by 1.4, 
       the image center is in the same position after scaling */
    p_fg_img->transform_cfg.pivot_x = p_fg_img->width / 2;
    p_fg_img->transform_cfg.pivot_y = p_fg_img->height / 2;
    p_fg_img->transform_cfg.scale_x = (EPIC_INPUT_SCALE_NONE*14)/10;
    p_fg_img->transform_cfg.scale_y = p_fg_img->transform_cfg.scale_x;       


    /* background image, its coordinate (0,0)~(99,99) */
    HAL_EPIC_LayerConfigInit(p_bg_img);
    p_bg_img->data = bg_img_buf;
    p_bg_img->x_offset = 0;
    p_bg_img->y_offset = 0;
    p_bg_img->width = 100;
    p_bg_img->height = 100;
    p_bg_img->total_width = 100;
    p_bg_img->color_mode = EPIC_COLOR_RGB565;
    
    /* output image, its coordinate (0,0)~(99,99), share same buffer as background image */
    HAL_EPIC_LayerConfigInit(&output_img);
    output_img.data = bg_img_buf;
    output_img.x_offset = 0;
    output_img.y_offset = 0;
    output_img.width = 100;
    output_img.height = 100;
    output_img.total_width = 100;
    output_img.color_mode = EPIC_COLOR_RGB565;

    
    /* set complete callback, and start EPIC */
    epic_handle.XferCpltCallback = epic_cplt_callback;
    ret = HAL_EPIC_BlendStartEx_IT(&epic_handle, &layers, 2, &output_img);
    /* check ret value if any error happens */
    ...
    /* wait for completion */
    sema_take(epic_sema);
}
```

### Solid Color Fill Example
For a 100x90 buffer, fill the region (20,10)~(39,49) with color RGB(99,107,123). The configured color is in RGB888 format, and the filled color format is RGB565; the hardware will convert the color format.
Opacity is 100; 255 is opaque, 0 is transparent.
Since the first pixel to fill is at (20,10), which is offset from the buffer start, the configured start address should be the offset address. total_width is the total width of the buffer (100), width is the width of the fill region (39-20+1=20).
After filling 20 pixels per line, the remaining 80 pixels are skipped, and the next line is filled, until the specified number of lines is filled.
```c
void fill_color(void)
{
    EPIC_FillingCfgTypeDef param;
    uint32_t start_offset;
    HAL_StatusTypeDef ret; 

    HAL_EPIC_FillDataInit(&param);
    /* topleft pixel offset in the output buffer */
    start_offset = 2 * (10 * 100 + 20);
    param.start = (uint8_t *)((uint32_t)output_buf + start_offset);
    /* filled color format RGB565 */
    param.color_mode = EPIC_COLOR_RGB565;
    /* filling area width */
    param.width = 20;
    /* filling area height */
    param.height = 40;
    /* filling buffer total width */
    param.total_width = 100;
    /* red part of RGB888 */
    param.color_r = 99;
    /* green part of RGB888 */
    param.color_g = 107;
    /* blue part of RGB888 */
    param.color_b = 123;
    /* opacity is 100 */
    param.alpha = 100;

    
    /* check ret if any error happens */
    /* set complete callback, and start EPIC */
    epic_handle.XferCpltCallback = epic_cplt_callback;
    ret = HAL_EPIC_FillStart_IT(&epic_handle, &param);
    /* check ret value if any error happens */
    ...
    /* wait for completion */
    sema_take(epic_sema);
}
```

### Gradient Fill
Gradient fill supports setting the colors of four corners, then interpolating the colors in between. Use the `HAL_EPIC_FillGrad_IT` interface.

### Continuous Blending
The continuous blending interface is generally used for blending small images with the same rendering properties, such as overlaying multiple characters (with the same color, format, etc., only coordinates, size, and data address change).
This set of interfaces is simple and only supports CPU polling mode.

Usage steps:
1. `HAL_EPIC_ContBlendStart`  -- Start continuous blending for the first time
2. `HAL_EPIC_ContBlendRepeat` -- For the subsequent N blends after the first
3. `HAL_EPIC_ContBlendStop`   -- Exit continuous blending mode


### Special Transformation Function
In some scenarios, you may need to change the foreground parameters after blending each small region. The `HAL_EPIC_TransStart` interface provides this capability, offering three parameters: `hor_path`, `ver_path`, and `user_data`, which control the output region stepping and allow changing the foreground image parameters.


## API Reference
[](#hal-epic)

HAL EPIC模块提供抽象的软件接口操作硬件EPIC模块，EPIC为一个2D图形引擎，它支持的特性

## 主要特性

- alpha混叠两幅图片并保存到输出buffer
- 以任意点为中心旋转图片（前景图片）, 将旋转后的图片与背景图片混叠，保存混叠后的结果到输出buffer
- 缩小/放到图片（前景图），将缩放后的图片与背景图混叠，保存混叠后的结果到输出buffer
- 一次GPU操作支持先旋转后缩放，不需要中间缓存buffer
- 在给定buffer中填充不透明或半透明色
- 所有的图形操作支持轮询和中断模式
- 输入和输出颜色格式不同即可实现颜色格式的自动转换
- 混叠的两幅图大小可以不同并部分重叠，还可以指定混叠后区域的一部分作为输出区域写入输出buffer
- 背景图和输出图可以复用同一个buffer， 比如背景图和输出图都使用frame buffer
- 输入图片格式支持EZIP
- 支持小数坐标混叠(55X 不支持)


## 输入输出限制
| 功能       |支持的格式                  |  55X                     |  58X   |  56X   |  52X   |
|----------- |---------------------------|--------------------------|--------|--------|--------|
|横向缩放    | 所有芯片能支持的颜色格式                      |3.8, 即缩小8倍，放大256倍, 精度1/256    |   10.16, 即缩小1024倍，放大65536倍, 精度1/65536     |   同58X    |   同58X    |
|纵向缩放    | 所有芯片能支持的颜色格式                      |横向纵向缩放倍数固定一样，不能分开配置  |   10.16, 即缩小1024倍，放大65536倍, 精度1/65536，<br>且可以和横向缩放系数不一样     |   同58X    |   同58X    |
|旋转       | 除EZIP、YUV格式之外 <br>另外A4/A8做mask时不能旋转 | [0 ~ 3600], 单位为0.1度    |  同55X     |   同55X      |   同55X      |
|水平镜像    | 所有芯片能支持的颜色格式                      |  支持       |   支持     |   支持      |   支持    |
|垂直镜像    | 除EZIP格式之外                               |   不支持    |   不支持    |   不支持    |   支持    |


```{note}
- 旋转和缩放支持同时进行，并且支持同一锚点, 支持任意锚点。
- 镜像支持任意锚点,且不能和旋转、缩放同时进行。
- 前景、背景、输出区域的并集不超过1024*1024像素(其中前景指绕任意锚点变形后的图像区域(包括锚点和旋转前图片))
>例如, 前景图为绕图片外锚点旋转并放大后, 与背景、输出区域的并集不超过1024

![EPIC限制说明](../../assets/epic_limitation.png)

```

## 颜色格式支持
| 输入颜色格式支持               |  55X   |  58X   |  56X   |  52X   |
|--------------------------------|--------|--------|--------|--------|
|RGB565/ARGB8565/RGB888/ARGB88888|   Y    |   Y    |   Y    |   Y    |
|L8                              |   N    |   Y    |   Y    |   Y    |
|A4/A8 (Mask,Overwrite,Fill)     |   N    |   Y    |   Y    |   Y    |
|YUV(YUYV/UYVY/iYUV)             |   N    |   N    |   Y    |   Y    |
|A2   (Fill)                     |   N    |   N    |   N    |   Y    |


| 输出颜色格式支持               |  55X   |  58X   |  56X   |  52X   |
|--------------------------------|--------|--------|--------|--------|
|RGB565/ARGB8565/RGB888/ARGB88888|   Y    |   Y    |   Y    |   Y    |


## 关图像问题的建议
- 用于旋转或缩放的图像在最外一圈加上一些透明像素(或者背景色),防止缩放时出现切边和旋转时出现锯齿
- 为防止缩放出现不连续,对于连续缩放场景缩放系数差值要大于1/256(即缩放精度不能超过1/256)
- 虽然旋转和缩放可以同时进行，但建议一次只执行一种变换以保证更好的输出图形质量
- 放大时建议使用图片左上角作为锚点，防止出现锚点抖动
- EZIP格式的图片不能用于旋转

详细的API说明请参考[EPIC](#hal-epic) 。


## 颜色在sram的存储格式

|        | bit31~bit25 | bit24~bit17 | bit16~bit8 | bit7~bit0 |
| ------ | ------ | ------ | ------ | ------ |
| RGB565   |    /    |    /       | R4~R0G5~G3         | G2~G0B4~B0 |
| ARGB8565 |    /    | A7 ~ A0    | R4~R0G5~G3         | G2~G0B4~B0 |
| RGB888   |    /    | R7 ~ R0    | G7 ~ G0            | B7 ~ B0 |
| ARGB8888 | A7 ~ A0 | R7 ~ R0    | G7 ~ G0            | B7 ~ B0 |
| A8       | D7 ~ D0 | C7~C0      | B7~B0              | A7~A0 |
| A4       |    /    |   /        | D3~D0C3~C0         | B3~B0A3~A0 |
| A2       |    /    |   /        | H1H0G1G0F1F0E1E0   | D1D0C1C0B1B0A1A0 |

```{note}
颜色数据均是紧密存放，在A2/A4/A8格式中ABCDEFGH代表像素点(从左到右显示)
```

## 使用HAL EPIC

首先调用 {c:func}`HAL_EPIC_Init` 初始化HAL EPIC. 在 {c:type}`EPIC_HandleTypeDef` 结构中需指定EPIC实例（即使用的EPIC硬件模块），芯片只有一个EPIC实例 {c:macro}`hwp_epic` 。 
初始化之后即可调用各种图形操作的接口处理数据。

例如，
```c
static EPIC_HandleTypeDef epic_handle;

void init_epic(void) 
{ 	// Initialize driver and enable EPIC IRQ
	HAL_NVIC_SetPriority(EPIC_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EPIC_IRQn);
	
	epic_handle.Instance = hwp_epic;
	HAL_EPIC_Init(&epic_handle);
}

/* EPIC IRQ Handler */
void EPIC_IRQHandler(void)
{
    HAL_EPIC_IRQHandler(&epic_handle);
}

```

{c:func}`HAL_EPIC_BlendStartEx_IT` 用于中断模式的填充、混叠、旋转和缩放操作， 需要在中断服务程序中调用 {c:func}`HAL_EPIC_IRQHandler` 处理中断。


### 混叠示例
如图1所示，示例`blend_img`中将一个图片的一部分叠到一个背景上：
1. 前景图所在区域的坐标为(10, 20)~(59,79)，背景图所在区域的坐标为(0,0)~(99,99), 混叠区域坐标为(5,10)~(44,59)，所有坐标均为一个坐标系中的数值
2. 前景图以100的透明度与背景混叠

混叠后(5,10)~(44,59)区域的颜色值被顺序写入到背景buffer，其中与前景重叠的部分（画叉的部分，即区域[10,20]~[44,59]）为混叠后的颜色，非重叠部分则是保持背景图中本来的颜色。

需要注意的是所有的数据buffer均为对应区域左上角像素的存放地址，例如p_fg_img->data指向前景图的坐标(10,20)所在像素的颜色值，outout_img.data指向输出区域的左上角像素，即(5,10)的颜色值。
output_img.width和output_img.total_width的设置，output_img.width表示输出区域的宽度，即44-5+1=40，
但output_img.total_width表示输出buffer的宽度，因为输出buffer对应的图形大小为100*100，所以output_img.total_width应设为100，
这样EPIC在写完一行40个像素的数据后，会跳过余下的60个像素，继续更新下一行的数据。
fg_img和bg_img的width和total_width也是相同的含义。


![Figure 1: Blending](../../assets/epic_blend.png)




```c
void epic_cplt_callback(EPIC_HandleTypeDef *epic)
{
    /* release the semaphore to indicate epic operation done */
    sema_release(epic_sema);
}

/* blend the foreground with background image using 100 opacity (0 is transparent, 255 is opaque)
 * output buffer is same as background image buffer, usually they're both frame buffer.
 * 
 */
void blend_img(void)
{
    EPIC_LayerConfigTypeDef layers[2];

    EPIC_LayerConfigTypeDef *p_bg_img = &layers[0];
    EPIC_LayerConfigTypeDef *p_fg_img = &layers[1];
    EPIC_LayerConfigTypeDef output_img;
    HAL_StatusTypeDef ret;         
    uint32_t buffer_start_offset;    
    
    /* foreground image, its coordinate (10,20)~(59,79), buffer size is 50*60 */
    HAL_EPIC_LayerConfigInit(p_fg_img);
    p_fg_img->data = fg_img_buf;
    p_fg_img->x_offset = 10;
    p_fg_img->y_offset = 20;
    p_fg_img->width = 50;
    p_fg_img->height = 60;
    p_fg_img->total_width = 50;
    p_fg_img->color_mode = EPIC_COLOR_RGB565;
    p_fg_img->alpha = 100;
    
    /* background image, its coordinate (0,0)~(99,99), buffer size is 100*100 */
    HAL_EPIC_LayerConfigInit(p_bg_img);
    p_bg_img->data = bg_img_buf;
    p_bg_img->x_offset = 0;
    p_bg_img->y_offset = 0;
    p_bg_img->width = 100;
    p_bg_img->height = 100;
    p_bg_img->total_width = 100;
    p_bg_img->color_mode = EPIC_COLOR_RGB565;
    
    /* output image, share the same buffer as bg_img_buf,
       output area is (5,10)~(44,59), buffer size is 100*100 */
    HAL_EPIC_LayerConfigInit(&output_img);
    /* topleft pixel is (5, 10), skip (10*100+5) pixels */
    buffer_start_offset = (10 - 0) * 100 * 2 + (5 - 0) * 2;
    output_img.data = (uint8_t *)((uint32_t)bg_img_buf + buffer_start_offset);
    /* output area topleft coordinate */
    output_img.x_offset = 5;
    output_img.y_offset = 10;
    /* output area width */
    output_img.width = 40;
    /* output area height */
    output_img.height = 50;
    /* output buffer width, it's different from output_img.width */
    output_img.total_width = 100;
    output_img.color_mode = EPIC_COLOR_RGB565;
    
    /* set complete callback, and start EPIC */
    epic_handle.XferCpltCallback = epic_cplt_callback;
    ret = HAL_EPIC_BlendStartEx_IT(&epic_handle, &layers, 2, &output_img);
    /* check ret value if any error happens */
    ...
    /* wait for completion */
    sema_take(epic_sema);
}

```


### 旋转示例

如图2所示，示例rotate_img将位于(10,20)~(59,79)的前景图以图中心为轴顺时针旋转30度，与背景图混叠后更新背景图对应位置的颜色，落在旋转后图形外的像素仍旧保持背景图的颜色。
由于旋转后图形覆盖的矩形区域会扩大(即[x0,y0]~[x1,y1])，为了保证旋转后的图形能被完整的显示出来，可以简单的将输出区域设成最大，HAL将自动计算旋转后的矩形区域，
当背景图buffer与输出buffer相同时, 只会更新输出buffer中被旋转区域覆盖的像素点的颜色。

![Figure 1: Rotation](../../assets/epic_rot.png)

```c
/* rotate the foreground image by 30 degree (clockwisely) and blend it with background using 100 opacity (0 is transparent, 255 is opaque)
 * output data is written back to background image buffer, it can also output to another buffer like blend_img_1.
 * 
 */
void rotate_img(void)
{
    EPIC_LayerConfigTypeDef layers[2];

    EPIC_LayerConfigTypeDef *p_bg_img = &layers[0];
    EPIC_LayerConfigTypeDef *p_fg_img = &layers[1];

    EPIC_LayerConfigTypeDef output_img;
    HAL_StatusTypeDef ret;
    
    /* foreground image, its coordinate (10,20)~(59,79) before rotation, buffer size is 50*60 */
    HAL_EPIC_LayerConfigInit(p_fg_img);
    p_fg_img->data = fg_img_buf;
    p_fg_img->x_offset = 10;
    p_fg_img->y_offset = 20;
    p_fg_img->width = 50;
    p_fg_img->height = 60;
    p_fg_img->total_width = 50;
    p_fg_img->color_mode = EPIC_COLOR_RGB565;
    p_fg_img->alpha = 100;
    /* foreground image is rotated by 30 degree around its center */
    p_fg_img->transform_cfg.angle = 300;
    p_fg_img->transform_cfg.pivot_x = p_fg_img->width / 2;
    p_fg_img->transform_cfg.pivot_y = p_fg_img->height / 2;
    p_fg_img->transform_cfg.scale_x = EPIC_INPUT_SCALE_NONE;
    p_fg_img->transform_cfg.scale_y = EPIC_INPUT_SCALE_NONE;    


    /* background image, its coordinate (0,0)~(99,99), buffer size is 100*100 */
    HAL_EPIC_LayerConfigInit(p_bg_img);
    p_bg_img->data = bg_img_buf;
    p_bg_img->x_offset = 0;
    p_bg_img->y_offset = 0;
    p_bg_img->width = 100;
    p_bg_img->height = 100;
    p_bg_img->total_width = 100;
    p_bg_img->color_mode = EPIC_COLOR_RGB565;
    
    /* output image, its coordinate (0,0)~(99,99), share same buffer as background image */
    HAL_EPIC_LayerConfigInit(&output_img);
    output_img.data = bg_img_buf;
    output_img.x_offset = 0;
    output_img.y_offset = 0;
    output_img.width = 100;
    output_img.height = 100;
    output_img.total_width = 100;
    output_img.color_mode = EPIC_COLOR_RGB565;
    
    
    /* set complete callback, and start EPIC */
    epic_handle.XferCpltCallback = epic_cplt_callback;
    ret = HAL_EPIC_BlendStartEx_IT(&epic_handle, &layers, 2, &output_img);
    /* check ret value if any error happens */
    ...
    /* wait for completion */
    sema_take(epic_sema);
}
```

### 缩放示例

如图3所示，示例scale_down_img将位于(10,20)~(59,79)的前景图横向与纵向都缩小到原图的71%%，同时保持图中心点位置不变。
类似旋转，也可以简单的将输出区域设成最大，如果输出buffer复用背景buffer，HAL将只更新缩小后区域（即[x0,y0]~[x1,y1]）所包含的像素的颜色值。

![Figure 1: Scaling](../../assets/epic_scaling.png)


```c

/* scale down the foreground image by 1.4 and blend it with background using 100 opacity (0 is transparent, 255 is opaque)
 * output data is written back to background image buffer, it can also output to another buffer like blend_img_1.
 * 
 */
void scale_down_img(void)
{
    EPIC_LayerConfigTypeDef layers[2];

    EPIC_LayerConfigTypeDef *p_bg_img = &layers[0];
    EPIC_LayerConfigTypeDef *p_fg_img = &layers[1];

    EPIC_LayerConfigTypeDef output_img;
    HAL_StatusTypeDef ret;
    
    /* foreground image, its coordinate (10,20)~(59,79) before scaling */
    HAL_EPIC_LayerConfigInit(p_fg_img);
    p_fg_img->data = fg_img_buf;
    p_fg_img->x_offset = 10;
    p_fg_img->y_offset = 20;
    p_fg_img->width = 50;
    p_fg_img->height = 60;
    p_fg_img->total_width = 50;
    p_fg_img->color_mode = EPIC_COLOR_RGB565;
    p_fg_img->alpha = 100;
    /* no rotation, both X and Y direction are scaled down by 1.4, 
       the image center is in the same position after scaling */
    p_fg_img->transform_cfg.pivot_x = p_fg_img->width / 2;
    p_fg_img->transform_cfg.pivot_y = p_fg_img->height / 2;
    p_fg_img->transform_cfg.scale_x = (EPIC_INPUT_SCALE_NONE*14)/10;
    p_fg_img->transform_cfg.scale_y = p_fg_img->transform_cfg.scale_x;       


    /* background image, its coordinate (0,0)~(99,99) */
    HAL_EPIC_LayerConfigInit(p_bg_img);
    p_bg_img->data = bg_img_buf;
    p_bg_img->x_offset = 0;
    p_bg_img->y_offset = 0;
    p_bg_img->width = 100;
    p_bg_img->height = 100;
    p_bg_img->total_width = 100;
    p_bg_img->color_mode = EPIC_COLOR_RGB565;
    
    /* output image, its coordinate (0,0)~(99,99), share same buffer as background image */
    HAL_EPIC_LayerConfigInit(&output_img);
    output_img.data = bg_img_buf;
    output_img.x_offset = 0;
    output_img.y_offset = 0;
    output_img.width = 100;
    output_img.height = 100;
    output_img.total_width = 100;
    output_img.color_mode = EPIC_COLOR_RGB565;

    
    /* set complete callback, and start EPIC */
    epic_handle.XferCpltCallback = epic_cplt_callback;
    ret = HAL_EPIC_BlendStartEx_IT(&epic_handle, &layers, 2, &output_img);
    /* check ret value if any error happens */
    ...
    /* wait for completion */
    sema_take(epic_sema);
}
```

### 纯色填充示例
一个大小为100*90的buffer，在其(20,10)~(39, 49)区域填充颜色RGB(99,107,123)，配置的颜色值为RGB888格式，填充后的颜色格式为RGB565，硬件会作颜色格式转换。
透明度为100，255表示不透明，0表示透明。
因为填充的第一个像素位置为(20,10)，相对buffer的首地址有偏移，配置的起始地址应为偏移后的地址，total_width为buffer的总宽度，即100，width为填充区域的宽度，即(39-20+1)=20,
填充完一行20个像素的颜色后，会跳过余下的80个颜色，转到下一行继续填充，直到填完指定行数。
```c
void fill_color(void)
{
    EPIC_FillingCfgTypeDef param;
    uint32_t start_offset;
    HAL_StatusTypeDef ret; 

    HAL_EPIC_FillDataInit(&param);
    /* topleft pixel offset in the output buffer */
    start_offset = 2 * (10 * 100 + 20);
    param.start = (uint8_t *)((uint32_t)output_buf + start_offset);
    /* filled color format RGB565 */
    param.color_mode = EPIC_COLOR_RGB565;
    /* filling area width */
    param.width = 20;
    /* filling area height */
    param.height = 40;
    /* filling buffer total width */
    param.total_width = 100;
    /* red part of RGB888 */
    param.color_r = 99;
    /* green part of RGB888 */
    param.color_g = 107;
    /* blue part of RGB888 */
    param.color_b = 123;
    /* opacity is 100 */
    param.alpha = 100;

    
    /* check ret if any error happens */
    /* set complete callback, and start EPIC */
    epic_handle.XferCpltCallback = epic_cplt_callback;
    ret = HAL_EPIC_FillStart_IT(&epic_handle, &param);
    /* check ret value if any error happens */
    ...
    /* wait for completion */
    sema_take(epic_sema);
}
```

### 渐变色填充
渐变色填充支持设置4个脚的颜色，然后中间均匀插值，使用`HAL_EPIC_FillGrad_IT`接口。

### 连续混叠
连续混叠接口一般用于相同渲染属性的小图片的混叠，比如连续叠多个字（字的颜色、格式等都相同，只是坐标、高宽、数据地址发生变化）。
这组接口的特点是功能简单、代码量小只有CPU polling模式，

使用时需要依次调用：
1. `HAL_EPIC_ContBlendStart`  -- 第一次启动连续混叠
2. `HAL_EPIC_ContBlendRepeat` -- 除第一次外，之后的N次混叠
3. `HAL_EPIC_ContBlendStop`   -- 退出连续混叠模式


### 特殊变形函数
在有些场景下需要每混叠一小块区域就改变一下前景的参数，`HAL_EPIC_TransStart`接口就提供了这样一个帮助函数, 它提供了3个参数`hor_path`,`ver_path`,`user_data`,通过这3组参数来控制输出区域的步进，同时提供了改变前景图片参数的可能。


## API参考
[](#hal-epic)