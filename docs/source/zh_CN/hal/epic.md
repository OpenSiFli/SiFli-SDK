# EPIC

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

{c:func}`HAL_EPIC_Rotate_IT` 用于中断模式的混叠、旋转和缩放操作， {c:func}`HAL_EPIC_BlendStart_IT` 用于中断模式的混叠操作，需要在中断服务程序中调用 {c:func}`HAL_EPIC_IRQHandler` 处理中断。
@note {c:func}`HAL_EPIC_Rotate_IT` 实现了所有 {c:func}`HAL_EPIC_BlendStart_IT` 的功能，对于只是混叠的场景，建议使用 {c:func}`HAL_EPIC_BlendStart_IT` ，因为它的叠图吞吐率更高

### 混叠示例
如图1所示，示例`blend_img_1`与`blend_img_2`中，前景图所在区域的坐标为(10, 20)~(59,79)，背景图所在区域的坐标为(0,0)~(99,99), 输出区域坐标为(5,10)~(44,59)，所有坐标均为一个坐标系中的数值，体现三个区域的相对位置关系，
前景图以39%%的透明度与背景混叠，混叠后(5,10)~(44,59)区域的颜色值被顺序写入到输出buffer，其中与前景重叠的部分（画叉的部分，即区域[10,20]~[44,59]）为混叠后的颜色，非重叠部分则是背景图中的颜色。
需要注意的是所有的数据buffer均为对应区域左上角像素的存放地址，例如fg_img.data指向前景图的坐标(10,20)所在像素的颜色值，outout_img.data指向输出区域的左上角像素，即(5,10)的颜色值。

![Figure 1: Blending](../../assets/epic_blend.png)

#### 背景图buffer与输出buffer不同 

##### 使用HAL_EPIC_Rotate_IT
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

##### 使用HAL_EPIC_BlendStart_IT
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


#### 背景图buffer与输出buffer相同

`blend_img_2`展示了输出buffer复用背景图buffer的场景，这也是最常使用的，即frame buffer作为背景图buffer和输出buffer，
这种情况下背景图buffer中的(10,20)~(44,59)区域所在的颜色值会被修改为混叠后的颜色，其他位置的颜色值保持不变，
需要注意output_img.width和output_img.total_width的设置，output_img.width表示输出区域的宽度，即44-5+1=40，
但output_img.total_width表示输出buffer的宽度，因为输出buffer对应的图形大小为100*100，所以output_img.total_width应设为100，
这样EPIC在写完一行40个像素的数据后，会跳过余下的60个像素，继续更新下一行的数据。
fg_img和bg_img的width和total_width也是相同的含义。

##### 使用HAL_EPIC_Rotate_IT

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

##### 使用HAL_EPIC_BlendStart_IT
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

### 颜色填充示例
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

    /* fill in polling mode */
    ret = HAL_EPIC_FillStart(&epic_handle, &param);
    /* check ret if any error happens */
}
```

## API参考
[](#hal-epic)