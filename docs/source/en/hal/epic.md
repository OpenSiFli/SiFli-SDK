# EPIC

HAL EPIC module provides abstract software interface to operate hardware EPIC module. EPIC is a 2D graphics engine with the following features:

## Main Features

- Alpha blend two images and save to output buffer
- Rotate image (foreground image) around any point, blend rotated image with background image, save blended result to output buffer
- Scale down/up image (foreground), blend scaled image with background image, save blended result to output buffer
- One GPU operation supports rotation followed by scaling without intermediate buffer
- Fill opaque or semi-transparent color in given buffer
- All graphics operations support polling and interrupt modes
- Automatic color format conversion when input and output color formats differ
- The two blended images can have different sizes and partial overlap, and can specify part of the blended area as output region to write to output buffer
- Background image and output image can reuse the same buffer, e.g., both background and output images use frame buffer
- Input image format supports EZIP
- Supports fractional coordinate blending (55X not supported)

## Input/Output Limitations
| Function       |Supported Formats                  |  55X                     |  58X   |  56X   |  52X   |
|----------- |---------------------------|--------------------------|--------|--------|--------|
|Horizontal scaling    | All color formats supported by all chips                      |3.8, i.e., shrink 8x, enlarge 256x, precision 1/256    |   10.16, i.e., shrink 1024x, enlarge 65536x, precision 1/65536     |   Same as 58X    |   Same as 58X    |
|Vertical scaling    | All color formats supported by all chips                      |Horizontal and vertical scaling factors fixed same, cannot be configured separately  |   10.16, i.e., shrink 1024x, enlarge 65536x, precision 1/65536,<br>and can differ from horizontal scaling factor     |   Same as 58X    |   Same as 58X    |
|Rotation       | Except EZIP, YUV formats <br>Also A4/A8 cannot rotate when used as mask | [0 ~ 3600], unit 0.1 degree    |  Same as 55X     |   Same as 55X      |   Same as 55X      |
|Horizontal mirroring    | All color formats supported by all chips                      |  Supported       |   Supported     |   Supported      |   Supported    |
|Vertical mirroring    | Except EZIP format                               |   Not supported    |   Not supported    |   Not supported    |   Supported    |

```{note}
- Rotation and scaling can be performed simultaneously with the same anchor point, supporting any anchor point.
- Mirroring supports any anchor point and cannot be performed simultaneously with rotation and scaling.
- The union of foreground, background, and output regions must not exceed 1024*1024 pixels (where foreground refers to the image region after transformation around any anchor point (including anchor point and pre-rotation image))
>For example, after foreground image is rotated around external anchor point and enlarged, the union with background and output regions must not exceed 1024
![](/docs/source/assets/epic_limitation.png) 

```

## Color Format Support
| Input Color Format Support               |  55X   |  58X   |  56X   |  52X   |
|--------------------------------|--------|--------|--------|--------|
|RGB565/ARGB8565/RGB888/ARGB88888|   Y    |   Y    |   Y    |   Y    |
|L8                              |   N    |   Y    |   Y    |   Y    |
|A4/A8 (Mask,Overwrite,Fill)     |   N    |   Y    |   Y    |   Y    |
|YUV(YUYV/UYVY/iYUV)             |   N    |   N    |   Y    |   Y    |
|A2   (Fill)                     |   N    |   N    |   N    |   Y    |

| Output Color Format Support               |  55X   |  58X   |  56X   |  52X   |
|--------------------------------|--------|--------|--------|--------|
|RGB565/ARGB8565/RGB888/ARGB88888|   Y    |   Y    |   Y    |   Y    |

## Suggestions for Graphics Issues
- Add some transparent pixels (or background color) around the outermost edge of images used for rotation or scaling to prevent edge cutting during scaling and jagged edges during rotation
- To prevent discontinuous scaling, for continuous scaling scenarios, the scaling factor difference should be greater than 1/256 (i.e., scaling precision cannot exceed 1/256)
- Although rotation and scaling can be performed simultaneously, it's recommended to perform only one transformation at a time to ensure better output graphics quality
- When enlarging, it's recommended to use the top-left corner of the image as anchor point to prevent anchor point jitter
- EZIP format images cannot be used for rotation

For detailed API documentation, please refer to [EPIC](#hal-epic).

## Color Storage Format in SRAM

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
All color data is stored compactly. In A2/A4/A8 formats, ABCDEFGH represent pixel points (displayed left to right)
```

## Using HAL EPIC

First call {c:func}`HAL_EPIC_Init` to initialize HAL EPIC. In {c:type}`EPIC_HandleTypeDef` structure, need to specify EPIC instance (i.e., the EPIC hardware module used). The chip has only one EPIC instance {c:macro}`hwp_epic`.
After initialization, various graphics operation interfaces can be called to process data.

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

{c:func}`HAL_EPIC_Rotate_IT` is used for interrupt mode blending, rotation and scaling operations. {c:func}`HAL_EPIC_BlendStart_IT` is used for interrupt mode blending operations. Need to call {c:func}`HAL_EPIC_IRQHandler` in interrupt service routine to handle interrupts.
@note {c:func}`HAL_EPIC_Rotate_IT` implements all functions of {c:func}`HAL_EPIC_BlendStart_IT`. For blending-only scenarios, it's recommended to use {c:func}`HAL_EPIC_BlendStart_IT` because it has higher blending throughput.

### Blending Example
As shown in Figure 1, example `blend_img_1` blends `blend_img_2`, where foreground image region coordinates are (10, 20)~(59,79), background image region coordinates are (0,0)~(99,99), output region coordinates are (5,10)~(44,59). All coordinates are values in the same coordinate system, reflecting the relative positional relationship of the three regions.
The foreground image is blended with the background at 39% transparency. After blending, the color values of region (5,10)~(44,59) are sequentially written to the output buffer. The overlapping part with the foreground (the cross-marked part, i.e., region [10,20]~[44,59]) contains blended colors, while non-overlapping parts contain colors from the background image.
Note that all data buffers are addresses storing pixels at the top-left corner of corresponding regions. For example, fg_img.data points to the pixel color value at foreground image coordinate (10,20), and output_img.data points to the top-left corner pixel of the output region, i.e., color value at (5,10).

![Figure 1: Blending](../../assets/epic_blend.png)

#### Background Image Buffer Different from Output Buffer

##### Using HAL_EPIC_Rotate_IT
```c
void epic_cplt_callback(EPIC_HandleTypeDef *epic)
{
    /* release the semaphore to indicate epic operation done */
    sema_release(epic_sema);
}

/* blend the foreground with background image using 100 opacity (0 is transparent, 255 is opaque)
 * output specified blended region to another buffer.
 * 
 */
void blend_img_1(void)
{
    EPIC_BlendingDataType fg_img;
    EPIC_BlendingDataType bg_img;
    EPIC_BlendingDataType output_img;
    EPIC_TransformCfgTypeDef trans_cfg;
    HAL_StatusTypeDef ret;     
    
    /* foreground image, its coordinate (10,20)~(59,79) , buffer size is 50*60 */
    HAL_EPIC_BlendDataInit(&fg_img);
    fg_img.data = fg_img_buf;
    fg_img.x_offset = 10;
    fg_img.y_offset = 20;
    /* blending area width */
    fg_img.width = 50;
    /* blending area height */
    fg_img.height = 60;
    /* image width, it can be different from fg_img.width */
    fg_img.total_width = 50;
    fg_img.color_mode = EPIC_COLOR_RGB565;
    fg_img.color_en = false;
    
    /* background image, its coordinate (0,0)~(99,99), buffer size is 100*100 */
    HAL_EPIC_BlendDataInit(&bg_img);
    bg_img.data = bg_img_buf;
    bg_img.x_offset = 0;
    bg_img.y_offset = 0;
    bg_img.width = 100;
    bg_img.height = 100;
    bg_img.total_width = 100;
    bg_img.color_mode = EPIC_COLOR_RGB565;
    bg_img.color_en = false;
    
    /* output image, its coordinate (5,10)~(44,59), buffer size is 40*50 */
    HAL_EPIC_BlendDataInit(&output_img);
    output_img.data = output_img_buf;
    output_img.x_offset = 5;
    output_img.y_offset = 10;
    output_img.width = 40;
    output_img.height = 50;
    output_img.total_width = 40;
    output_img.color_mode = EPIC_COLOR_RGB565;
    output_img.color_en = false;
    
    /* set complete callback */
    epic_handle.XferCpltCallback = epic_cplt_callback;
    
    /* no rotation and scaling, opacity 100 
     * start EPIC in interrupt mode
     */
    HAL_EPIC_RotDataInit(&trans_cfg);

    
    ret = HAL_EPIC_Rotate_IT(&epic_handle, &trans_cfg, &fg_img, &bg_img, &output_img, 100);
    /* check ret value if any error happens */
    ...
    /* wait for completion */
    sema_take(epic_sema);
}

```

##### Using HAL_EPIC_BlendStart_IT
```c
void epic_cplt_callback(EPIC_HandleTypeDef *epic)
{
    /* release the semaphore to indicate epic operation done */
    sema_release(epic_sema);
}

/* blend the foreground with background image using 100 opacity (0 is transparent, 255 is opaque)
 * output specified blended region to another buffer.
 * 
 */
void blend_img_1(void)
{
    EPIC_BlendingDataType fg_img;
    EPIC_BlendingDataType bg_img;
    EPIC_BlendingDataType output_img;
    HAL_StatusTypeDef ret;     
    
    /* foreground image, its coordinate (10,20)~(59,79) , buffer size is 50*60 */
    HAL_EPIC_BlendDataInit(&fg_img);
    fg_img.data = fg_img_buf;
    fg_img.x_offset = 10;
    fg_img.y_offset = 20;
    /* blending area width */
    fg_img.width = 50;
    /* blending area height */
    fg_img.height = 60;
    /* image width, it can be different from fg_img.width */
    fg_img.total_width = 50;
    fg_img.color_mode = EPIC_COLOR_RGB565;
    fg_img.color_en = false;
    
    /* background image, its coordinate (0,0)~(99,99), buffer size is 100*100 */
    HAL_EPIC_BlendDataInit(&bg_img);
    bg_img.data = bg_img_buf;
    bg_img.x_offset = 0;
    bg_img.y_offset = 0;
    bg_img.width = 100;
    bg_img.height = 100;
    bg_img.total_width = 100;
    bg_img.color_mode = EPIC_COLOR_RGB565;
    bg_img.color_en = false;
    
    /* output image, its coordinate (5,10)~(44,59), buffer size is 40*50 */
    HAL_EPIC_BlendDataInit(&output_img);
    output_img.data = output_img_buf;
    output_img.x_offset = 5;
    output_img.y_offset = 10;
    output_img.width = 40;
    output_img.height = 50;
    output_img.total_width = 40;
    output_img.color_mode = EPIC_COLOR_RGB565;
    output_img.color_en = false;
    
    /* set complete callback */
    epic_handle.XferCpltCallback = epic_cplt_callback;
    
    ret = HAL_EPIC_BlendStart_IT(&epic_handle, &fg_img, &bg_img, &output_img, 100);
    /* check ret value if any error happens */
    ...    
    /* wait for completion */
    sema_take(epic_sema);
}

```

#### Background Image Buffer Same as Output Buffer

`blend_img_2` demonstrates the scenario where output buffer reuses background image buffer, which is also the most commonly used case, i.e., frame buffer serves as both background image buffer and output buffer.
In this case, the color values in region (10,20)~(44,59) of the background image buffer will be modified to blended colors, while color values at other positions remain unchanged.
Note the settings of output_img.width and output_img.total_width. output_img.width represents the width of the output region, i.e., 44-5+1=40.
But output_img.total_width represents the width of the output buffer. Since the output buffer corresponds to a 100*100 image, output_img.total_width should be set to 100.
This way, after EPIC finishes writing 40 pixels in one row, it will skip the remaining 60 pixels and continue updating the next row.
The width and total_width of fg_img and bg_img have the same meaning.

##### Using HAL_EPIC_Rotate_IT

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
void blend_img_2(void)
{
    EPIC_BlendingDataType fg_img;
    EPIC_BlendingDataType bg_img;
    EPIC_BlendingDataType output_img;
    EPIC_TransformCfgTypeDef trans_cfg;
    HAL_StatusTypeDef ret;         
    uint32_t buffer_start_offset;    
    
    /* foreground image, its coordinate (10,20)~(59,79), buffer size is 50*60 */
    HAL_EPIC_BlendDataInit(&fg_img);
    fg_img.data = fg_img_buf;
    fg_img.x_offset = 10;
    fg_img.y_offset = 20;
    fg_img.width = 50;
    fg_img.height = 60;
    fg_img.total_width = 50;
    fg_img.color_mode = EPIC_COLOR_RGB565;
    fg_img.color_en = false;
    
    /* background image, its coordinate (0,0)~(99,99), buffer size is 100*100 */
    HAL_EPIC_BlendDataInit(&bg_img);
    bg_img.data = bg_img_buf;
    bg_img.x_offset = 0;
    bg_img.y_offset = 0;
    bg_img.width = 100;
    bg_img.height = 100;
    bg_img.total_width = 100;
    bg_img.color_mode = EPIC_COLOR_RGB565;
    bg_img.color_en = false;
    
    /* output image, share the same buffer as bg_img_buf,
       output area is (5,10)~(44,59), buffer size is 100*100 */
    HAL_EPIC_BlendDataInit(&output_img);
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
    output_img.color_en = false;
    
    /* set complete callback */
    epic_handle.XferCpltCallback = epic_cplt_callback;

    /* no rotation and scaling, opacity 100 
     * start EPIC in interrupt mode
     */
    HAL_EPIC_RotDataInit(&trans_cfg);

    
    ret = HAL_EPIC_Rotate_IT(&epic_handle, &trans_cfg, &fg_img, &bg_img, &output_img, 100);
    /* check ret value if any error happens */
    ...    
    /* wait for completion */
    sema_take(epic_sema);
}

```

##### Using HAL_EPIC_BlendStart_IT
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
void blend_img_2(void)
{
    EPIC_BlendingDataType fg_img;
    EPIC_BlendingDataType bg_img;
    EPIC_BlendingDataType output_img;
    HAL_StatusTypeDef ret;         
    uint32_t buffer_start_offset;    
    
    /* foreground image, its coordinate (10,20)~(59,79), buffer size is 50*60 */
    HAL_EPIC_BlendDataInit(&fg_img);
    fg_img.data = fg_img_buf;
    fg_img.x_offset = 10;
    fg_img.y_offset = 20;
    fg_img.width = 50;
    fg_img.height = 60;
    fg_img.total_width = 50;
    fg_img.color_mode = EPIC_COLOR_RGB565;
    fg_img.color_en = false;
    
    /* background image, its coordinate (0,0)~(99,99), buffer size is 100*100 */
    HAL_EPIC_BlendDataInit(&bg_img);
    bg_img.data = bg_img_buf;
    bg_img.x_offset = 0;
    bg_img.y_offset = 0;
    bg_img.width = 100;
    bg_img.height = 100;
    bg_img.total_width = 100;
    bg_img.color_mode = EPIC_COLOR_RGB565;
    bg_img.color_en = false;
    
    /* output image, share the same buffer as bg_img_buf,
       output area is (5,10)~(44,59), buffer size is 100*100 */
    HAL_EPIC_BlendDataInit(&output_img);
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
    output_img.color_en = false;
    
    /* set complete callback */
    epic_handle.XferCpltCallback = epic_cplt_callback;

    ret = HAL_EPIC_BlendStart_IT(&epic_handle, &fg_img, &bg_img, &output_img, 100);
    /* check ret value if any error happens */
    ...
    /* wait for completion */
    sema_take(epic_sema);
}

```

### Rotation Example

As shown in Figure 2, example rotate_img rotates the foreground image located at (10,20)~(59,79) clockwise by 30 degrees around the image center, blends it with the background image, and updates the corresponding colors in the background image. Pixels outside the rotated graphic still maintain the background image colors.
Since the rectangular area covered by the rotated graphic will expand (i.e., [x0,y0]~[x1,y1]), to ensure the rotated graphic can be displayed completely, you can simply set the output area to maximum. HAL will automatically calculate the rectangular area after rotation.
When background image buffer is the same as output buffer, only the pixel colors covered by the rotated area in the output buffer will be updated.

![Figure 1: Rotation](../../assets/epic_rot.png)

```c
/* rotate the foreground image by 30 degree (clockwise) and blend it with background using 100 opacity (0 is transparent, 255 is opaque)
 * output data is written back to background image buffer, it can also output to another buffer like blend_img_1.
 * 
 */
void rotate_img(void)
{
    EPIC_BlendingDataType fg_img;
    EPIC_BlendingDataType bg_img;
    EPIC_BlendingDataType output_img;
    EPIC_TransformCfgTypeDef trans_cfg;
    HAL_StatusTypeDef ret;         
    
    /* foreground image, its coordinate (10,20)~(59,79) before rotation, buffer size is 50*60 */
    HAL_EPIC_BlendDataInit(&fg_img);
    fg_img.data = fg_img_buf;
    fg_img.x_offset = 10;
    fg_img.y_offset = 20;
    fg_img.width = 50;
    fg_img.height = 60;
    fg_img.total_width = 50;
    fg_img.color_mode = EPIC_COLOR_RGB565;
    fg_img.color_en = false;
    
    /* background image, its coordinate (0,0)~(99,99), buffer size is 100*100 */
    HAL_EPIC_BlendDataInit(&bg_img);
    bg_img.data = bg_img_buf;
    bg_img.x_offset = 0;
    bg_img.y_offset = 0;
    bg_img.width = 100;
    bg_img.height = 100;
    bg_img.total_width = 100;
    bg_img.color_mode = EPIC_COLOR_RGB565;
    bg_img.color_en = false;
    
    /* output image, its coordinate (0,0)~(99,99), share same buffer as background image */
    HAL_EPIC_BlendDataInit(&output_img);
    output_img.data = bg_img_buf;
    output_img.x_offset = 0;
    output_img.y_offset = 0;
    output_img.width = 100;
    output_img.height = 100;
    output_img.total_width = 100;
    output_img.color_mode = EPIC_COLOR_RGB565;
    output_img.color_en = false;
    
    epic_handle.XferCpltCallback = epic_cplt_callback;
    
    /* foreground image is rotated by 30 degree around its center */
    HAL_EPIC_RotDataInit(&trans_cfg);
    trans_cfg.angle = 300;
    trans_cfg.pivot_x = fg_img.width / 2;
    trans_cfg.pivot_y = fg_img.height / 2;
    trans_cfg.scale_x = 1000;
    trans_cfg.scale_y = 1000;    
    
    
    /* no scaling, opacity 100 
     * start EPIC in interrupt mode
     */
    ret = HAL_EPIC_Rotate_IT(&epic_handle, &trans_cfg, &fg_img, &bg_img, &output_img, 100);
    /* check ret value if any error happens */
    ...
    /* wait for completion */
    sema_take(epic_sema);
}
```

### Scaling Example

As shown in Figure 3, example scale_down_img scales the foreground image located at (10,20)~(59,79) down to 71% of the original size both horizontally and vertically, while keeping the image center position unchanged.
Similar to rotation, you can simply set the output area to maximum. If the output buffer reuses the background buffer, HAL will only update the pixel color values contained in the scaled area (i.e., [x0,y0]~[x1,y1]).

![Figure 1: Scaling](../../assets/epic_scaling.png)

```c

/* scale down the foreground image by 1.4 and blend it with background using 100 opacity (0 is transparent, 255 is opaque)
 * output data is written back to background image buffer, it can also output to another buffer like blend_img_1.
 * 
 */
void scale_down_img(void)
{
    EPIC_BlendingDataType fg_img;
    EPIC_BlendingDataType bg_img;
    EPIC_BlendingDataType output_img;
    EPIC_TransformCfgTypeDef trans_cfg;
    HAL_StatusTypeDef ret;         
    
    /* foreground image, its coordinate (10,20)~(59,79) before scaling */
    HAL_EPIC_BlendDataInit(&fg_img);
    fg_img.data = fg_img_buf;
    fg_img.x_offset = 10;
    fg_img.y_offset = 20;
    fg_img.width = 50;
    fg_img.height = 60;
    fg_img.total_width = 50;
    fg_img.color_mode = EPIC_COLOR_RGB565;
    fg_img.color_en = false;
    
    /* background image, its coordinate (0,0)~(99,99) */
    HAL_EPIC_BlendDataInit(&bg_img);
    bg_img.data = bg_img_buf;
    bg_img.x_offset = 0;
    bg_img.y_offset = 0;
    bg_img.width = 100;
    bg_img.height = 100;
    bg_img.total_width = 100;
    bg_img.color_mode = EPIC_COLOR_RGB565;
    bg_img.color_en = false;
    
    /* output image, its coordinate (0,0)~(99,99), share same buffer as background image */
    HAL_EPIC_BlendDataInit(&output_img);
    output_img.data = bg_img_buf;
    output_img.x_offset = 0;
    output_img.y_offset = 0;
    output_img.width = 100;
    output_img.height = 100;
    output_img.total_width = 100;
    output_img.color_mode = EPIC_COLOR_RGB565;
    output_img.color_en = false;
    
    epic_handle.XferCpltCallback = epic_cplt_callback;

    /* no rotation, both X and Y direction are scaled down by 1.4, 
       the image center is in the same position after scaling */
    HAL_EPIC_RotDataInit(&trans_cfg);
    trans_cfg.pivot_x = fg_img.width / 2;
    trans_cfg.pivot_y = fg_img.height / 2;
    trans_cfg.scale_x = 1400;
    trans_cfg.scale_y = 1400;       
   
    /* opacity 100 
     * start EPIC in interrupt mode
     */
    ret = HAL_EPIC_Rotate_IT(&epic_handle, &trans_cfg, &fg_img, &bg_img, &output_img, 100);
    /* check ret value if any error happens */
    ...
    /* wait for completion */
    sema_take(epic_sema);
}
```

### Color Fill Example
A buffer of size 100*90, fill the region (20,10)~(39, 49) with color RGB(99,107,123). The configured color value is in RGB888 format, and the filled color format is RGB565. Hardware will perform color format conversion.
Transparency is 100, where 255 means opaque and 0 means transparent.
Since the first pixel position for filling is (20,10), which has an offset relative to the buffer's base address, the configured start address should be the offset address. total_width is the total width of the buffer, i.e., 100, and width is the width of the fill area, i.e., (39-20+1)=20.
After filling 20 pixels in one row, it will skip the remaining 80 colors and move to the next row to continue filling until the specified number of rows is completed.
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

    /* fill in polling mode */
    ret = HAL_EPIC_FillStart(&epic_handle, &param);
    /* check ret if any error happens */
}
```

## API Reference
[](#hal-epic)
