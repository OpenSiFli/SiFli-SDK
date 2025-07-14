# LCD
## 简介

LCD device是一个rt_device，内部是一个简单的LCD框架用于注册不同屏幕驱动。本章节主要介绍LCD device的使用及框架，以及如何注册一个新屏幕到该框架。

## 内部结构
LCD驱动有3层:
- rt_device_graphic层 - 对上层提供统一调用接口
    - 内部支持多个驱动查找功能，方便兼容多个屏（根据比较注册的ID和ReadID函数返回的值决定）
    - 内部包含3个framebuffer机制，让渲染和送屏可以同步进行，支持压缩。
- 具体驱动的逻辑层
    - 具体各个屏驱动的接口、频率、TE等配置，以及初始化代码、送屏命令，睡眠，开关指令
- 屏物理接口的抽象层
    - 对大部分接口提供统一的操作函数 详见[](../hal/lcdc.md)


![Figure 1: lcd device SW arch](../../assets/lcd_device_arch.png)



```{note}
SDK内实现了2个rt_device_graphic实例，rt_device name：
- lcd(drv_lcd.c)
    - 正常物理LCD屏
- ram_lcd(drv_ram_lcd.c)
    - 将输出写到SRAM（打开后将从系统堆内存分配一个LCD buffer）
    - ram_lcd的使用跟正常屏幕一样(屏幕尺寸固定）
```


## 上层使用LCD device的示例
在屏幕中央绘制一个100*100的RGB565格式的红色区域，每刷新一次红色加深一点，32次后从头循环。

```c

#define RGB565_FB_WIDTH  100
#define RGB565_FB_HEIGHT  100
static uint16_t rgb565_frambuffer[RGB565_FB_WIDTH * RGB565_FB_HEIGHT];

static struct rt_semaphore lcd_sema;

static rt_err_t lcd_flush_done(rt_device_t dev, void *buffer)
{
    rt_sem_release(&lcd_sema);
    return RT_EOK;
}

void lcd_flush_task(void *parameter)
{

    rt_err_t ret;
    uint8_t color = 0;
    struct rt_device_graphic_info info;
    uint16_t framebuffer_color_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;


    /* LCD Device Init */
    rt_device_t lcd_device = rt_device_find("lcd");
    RT_ASSERT(lcd_device != RT_NULL);
    rt_device_set_tx_complete(lcd_device, lcd_flush_done);
    if (rt_device_open(lcd_device, RT_DEVICE_OFLAG_RDWR) == RT_EOK)
    {
        if (rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_INFO, &info) == RT_EOK)
        {
            rt_kprintf("Lcd info w:%d, h%d, bits_per_pixel %d\r\n", info.width, info.height, info.bits_per_pixel);
        }
    }

    rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &framebuffer_color_format);

    rt_sem_init(&lcd_sema, "lv_lcd", 1, RT_IPC_FLAG_FIFO);


    while (1)
    {
        rt_err_t err;
        int32_t dx, dy;

        /*Draw framebuffer at center of screen*/
        dx = (info.width - RGB565_FB_WIDTH) / 2;
        dy = (info.height - RGB565_FB_HEIGHT) / 2;

         /* Fill RGB565 framebuffer with red color*/
        color = (color + 1) % 0x1F;
        for(uint32_t i = 0; i < RGB565_FB_HEIGHT; i++)
            for(uint32_t j = 0; j < RGB565_FB_WIDTH; j++)
                {
                    rgb565_frambuffer[(i * RGB565_FB_WIDTH) + j] = ((uint16_t )color) << 11;
                }

        //Wait lcd_flush_done to release lcd_sema
        err = rt_sem_take(&lcd_sema, rt_tick_from_millisecond(1000));
        RT_ASSERT(RT_EOK == err);

		/*
			设置绘图操作的有效区域，以下所有坐标的坐标系原点是屏幕左上角
		*/
        rt_graphix_ops(lcd_device)->set_window(dx, dy, dx + RGB565_FB_WIDTH - 1, dy + RGB565_FB_HEIGHT - 1);

		/*
			将一个buffer画到LCD设备上，异步函数，完成后调用lcd_flush_done
			
			关于{x0,y0,x1,y1}区域：
			   1. 该区域表示整个buffer在屏幕上占据的区域，并不是用于剪切该buffer的
			   2. 如果只更新buffer中的某一部分，需配合set_window函数使用
        */
        rt_graphix_ops(lcd_device)->draw_rect_async((const char *)&rgb565_frambuffer, dx, dy, dx + RGB565_FB_WIDTH - 1, dy + RGB565_FB_HEIGHT - 1);

    }
}
```



## 增加一个新屏幕的流程
## 1. 选择example\\rt_driver下对应板子的工程
- 这个工程里面有一个简单的绘制矩形区域的示例（见前面 “_rt_device_graphic_层接口的使用示例”）
- 如果选择用_watch_demo_工程，需要 ***把tp线程关闭再调屏幕*** ，防止TP不通阻塞UI送屏线程（关闭办法：`drv_touch.c` `touch_open`函数，去掉_rt_thread_startup(touch_thread);_ ）

## 2. 将新驱动添加到编译工程里面
- 添加新屏幕代码到目录_customer\\peripherals_内
    - 可以从其他已有的驱动复制一份代码，然后将名字、ID、对应的命令（绝大部分都一样不需要改）改成自己的
    - 注意修改内部的Kconfig文件的depend宏
    - 屏驱注册的宏说明：
    ```c
    LCD_DRIVER_EXPORT(
        rm69090,   //屏驱的名称，字符串
        RM69090_ID,  //屏驱的ID，用于和RM69090_ReadID的返回值做比较
        &lcdc_int_cfg, //屏驱的初始化配置（包括接口、频率、输出颜色格式等）
        &RM69090_drv,  //屏驱的对外API接口实现
        RM69090_LCD_PIXEL_WIDTH,  // IC的横向最大分辨率（注意不是玻璃的分辨率）
        RM69090_LCD_PIXEL_HEIGHT, // IC的垂直最大分辨率（注意不是玻璃的分辨率）
        2                //刷新时的像素对齐（不需要则写1）
        );
    ```
    
```{note} 
玻璃的分辨率固定在宏`LCD_DRIVER_EXPORT`的定义内,见宏`LCD_HOR_RES_MAX`和`LCD_VER_RES_MAX`
```
- 在_customer\\peripherals\\Kconfig_内，为新加的驱动添加一个隐藏开选项，比如：
    ```c
    config LCD_USING_RM69090
        bool
        default n
    ```
- 在板级的配置中添加屏幕模组的开关，模组开关内选上前面添加的隐藏开关以及使用的接口开关（接口开关用于修改pinmux），比如_customer\\boards\\ec-lb551XXX\\Kconfig_内，添加模组开关：
    ```c
    config LCD_USING_ED_LB55SPI17801_QADSPI_LB551
        bool "1.78 rect QAD-SPI LCD(ED-LB55SPI17801)"
        select TSC_USING_FT3168 if BSP_USING_TOUCHD
        select LCD_USING_RM69090
        select BSP_LCDC_USING_QADSPI
        if LCD_USING_ED_LB55SPI17801_QADSPI_LB551
            config LCD_RM69090_VSYNC_ENABLE
                bool "Enable LCD VSYNC (TE signal)"
                def_bool n
        endif
    ```
```{note}
    触控的添加流程类似屏，并在此处模组开关内一并选择
```
- 还是在上面文件中，为新添加的屏幕定义分辨率、DPI值（***注意此处是模组玻璃的分辨率，不是屏幕IC能支持的最大分辨率***）
    ```c
    config LCD_HOR_RES_MAX
        int
        default 454 if LCD_USING_ED_LB55SPI17201_QADSPI_LB551 
        default 400 if LCD_USING_ED_LB55_77903_QADSPI_LB551 
        default 240 if LCD_USING_ED_LB55_387A_JDI_LB551 
        default 368 if LCD_USING_ED_LB55SPI17801_QADSPI_LB551    <-------- 新添加的行

    config LCD_VER_RES_MAX
        int
        default 454 if LCD_USING_ED_LB55SPI17201_QADSPI_LB551 
        default 400 if LCD_USING_ED_LB55_77903_QADSPI_LB551
        default 240 if LCD_USING_ED_LB55_387A_JDI_LB551
        default 448 if LCD_USING_ED_LB55SPI17801_QADSPI_LB551    <-------- 新添加的行

    config LCD_DPI
        int
        default 315 if LCD_USING_ED_LB55SPI17201_QADSPI_LB551 
        default 315 if LCD_USING_ED_LB55_77903_QADSPI_LB551
        default 315 if LCD_USING_ED_LB55_387A_JDI_LB551
        default 315 if LCD_USING_ED_LB55SPI17801_QADSPI_LB551    <-------- 新添加的行
    ```
- 若用scons 编译，则需要进入工程的menuconfig选择菜单，然后选择前面新增的屏幕模组，最终生成_.config_和_rtconfig.h_
- 若用Keil编译，也可以直接添加源代码进入（但还是建议和scons编译添加方法一样，这样下次重新生成Keil工程后会自动加入）

## 3. 检查新增LCD用到的pin，以及reset pin 的pinmux是否正确
- 如前所述，在模组的配置时会选择一个接口的宏，SDK内部有对不同的LCDC接口宏做pin mux处理
- reset pin由于是独立的GPIO控制，需要确认`BPS_LCD_Reset`函数控制的pin是否正确

## 4. 修改新屏幕的接口、频率、输出颜色格式
- 见下面示例，修改`LCD_DRIVER_EXPORT`宏内`init_cfg`结构体（输出频率跟配置的可能会有偏差，请以实际输出的为准，因为HAL层实现输出频率=HCLK/divider， HCLK在console输入"sysinfo"可以查看，divider为2~255的整数）
- TE在调试初期建议关闭，防止LCDCD因为等不到TE信号而不送数（我们的TE信号是LCDC自动处理，不需要软件参与）
```{note} 
我们的TE信号是LCDC自动处理，不需要软件参与，所以没有TE软件中断上来。只要TE pin mux和极性正确配置，启动LCDC送数后（_HAL_LCDC_SendLayerData2Reg_IT_），收到了TE脉冲LCDC就会启动送数
```

## 5 使用任意GPIO作为TE信号(可选)
大部分情况下，只要定义相应的管脚为TE功能，LCDC就可以自动处理TE信号，但在某些特殊情况下，TE信号无法从期望的通路来时，需要改成普通GPIO来实现，此时可以通过软件GPIO中断办法实现:
- 按照正常的配置将TE打开、设置好TE的延迟
- 定义宏“`LCD_USE_GPIO_TE`”为一个普通GPIO, 它将会在中断上升沿主动去人为制造一个TE信号（翻转TE极性），从而触发LCDC送数据
```{note} 
因为正常TE通路不正常，所以第一步的设置无法工作，只能靠第二步的人为制造信号去触发送数，从而实现正常TE处理
```

## 6. 修改新屏幕驱动的初始化代码
- 一般是先初始化LCDC,配置接口、频率等.  调用API -  HAL_LCDC_Init.
- 然后是reset LCD , 通过drv_io.c内实现的`BPS_LCD_Reset`函数去控制GPIO 复位屏幕。
- 然后就是屏厂给的初始化代码

## 7. 修改read id函数
- `drv_lcd.c`` 将利用该函数返回值和`LCD_DRIVER_EXPORT`注册的ID比较， ***相同则认为该驱动可用，才会去调用。***

## 8. QAD-SPI LCD扩展命令修改
- QAD-SPI LCD 一般会把标准8bit命令扩展成32bit，需要修改扩展方法。 可以参考rm69330.c， 一般有这几个函数需要修改： _RM69330_WriteMultiplePixels_， _RM69330_WriteReg_，_RM69330_ReadData_。

<br>
<br>
<br>

## 客户新增屏幕驱动代码示例（部分）
以下示例代码展示了RM69330如何注册到`drv_lcd.c`(rt_device_graphic层)以及接口配置、函数回调等. 具体每个函数的实现请参考SDK代码，此处不赘述。


```c
//RM69330 chip IDs
#define RM69330_ID                  0x8000


//RM69330 resulution
#define  RM69330_LCD_PIXEL_WIDTH    (454)
#define  RM69330_LCD_PIXEL_HEIGHT   (454)

//Interface, Frequency, Output color format, TE
static LCDC_InitTypeDef lcdc_int_cfg_qspi =
{
    .lcd_itf = LCDC_INTF_SPI_DCX_4DATA, //QAD-SPI mode
    .freq = 48000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 0, //0: QAD-SPI/SPI3   1:SPI4
            .syn_mode = HAL_LCDC_SYNC_VER,
            .vsyn_polarity = 0,
            .vsyn_delay_us = 1000,
            .hsyn_num = 0,
        },
    },

};

//Function callbacks
static const LCD_DrvOpsDef RM69330_drv =
{
    RM69330_Init,
    RM69330_ReadID,
    RM69330_DisplayOn,
    RM69330_DisplayOff,

    RM69330_SetRegion,
    RM69330_WritePixel,
    RM69330_WriteMultiplePixels,

    RM69330_ReadPixel,

    RM69330_SetColorMode,
    RM69330_SetBrightness
};


//Regist to drv_lcd.c
LCD_DRIVER_EXPORT(rm69330, RM69330_ID, &lcdc_int_cfg_qspi,
    &RM69330_drv,
    RM69330_LCD_PIXEL_WIDTH,
    RM69330_LCD_PIXEL_HEIGHT,2);



```
```{note} 
如前面所述，`drv_lcd.c``初始化时将比较 ***LCD_DRIVER_EXPORT注册的RM69330_ID*** 和 ***RM69330_ReadID返回的ID***,如果相同才会调用。
```

## 同时兼容多个屏幕模组

假设要兼容2各模组：
- 模组一为LB55SPI17801(屏幕IC是RM69090，触控IC是FT3168), 
- 模组二为LB55BILI8688E(屏幕IC是ILI8688E，触控IC是CST918)，
   
跟前面添加屏幕一样，往工程对应的Kconfig文件添加一项同时选择2款模组的屏驱和触控驱动即可（注意：屏驱的ReadID函数要能分别2款IC，同理触控的probe函数也要能区分不同的IC)。

Kconfig文件示例如下：
```c
config LCD_USING_ED_LB55SPI17801_LB55BILI8688E_QADSPI_LB551
    bool "1.78 rect QAD-SPI LCD(ED-LB55SPI17801 and ED-LB55BILI8688E)"
    select TSC_USING_FT3168 if BSP_USING_TOUCHD
    select TSC_USING_CST918 if BSP_USING_TOUCHD
    select LCD_USING_RM69090
    select LCD_USING_ILI8688E
    select BSP_LCDC_USING_QADSPI
    if LCD_USING_ED_LB55SPI17801_LB55BILI8688E_QADSPI_LB551
        config LCD_RM69090_VSYNC_ENABLE
            bool
            def_bool n
        config LCD_ILI8688E_VSYNC_ENABLE
            bool
            def_bool y
    endif
```
```{note} 
其他的屏幕分辨率配置，DPI配置就共用`LCD_USING_ED_LB55SPI17801_LB55BILI8688E_QADSPI_LB551`宏去设置即可，既然兼容这些参数应该都是一样的
```

<br>
<br>
<br>

## DSI屏幕调试

建议先调低速模式 因为低速可以绕过因为硬件导致的问题，并且好用示波器分析。    低速模式调通后读ID,读ID 能检查屏幕上电是否正常。
低速模式读ID，刷屏都正常后，再改用高速模式刷屏


## DSI低速模式操作流程
- 降低LCDC送数的速度
    - 需要将系统时钟降到48M，在`drv_io.c`内，将_HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, XXX)_; XXX就是系统时钟频率，改成_RCC_SYSCLK_HXT48_(晶体时钟48MHz)
    - 将所有命令都改成LP模式(低速模式)发送,并且打开LCD acknowledge，方便用逻分仪或示波器抓取波形分析(在前面的`LCDC_InitTypeDef`那个结构体内配置，如下：)
    ```c
        .LPCmd = {
        .LPGenShortWriteNoP    = DSI_LP_GSW0P_ENABLE,
        .LPGenShortWriteOneP   = DSI_LP_GSW1P_ENABLE,
        .LPGenShortWriteTwoP   = DSI_LP_GSW2P_ENABLE,
        .LPGenShortReadNoP     = DSI_LP_GSR0P_ENABLE,
        .LPGenShortReadOneP    = DSI_LP_GSR1P_ENABLE,
        .LPGenShortReadTwoP    = DSI_LP_GSR2P_ENABLE,
        .LPGenLongWrite        = DSI_LP_GLW_ENABLE,
        .LPDcsShortWriteNoP    = DSI_LP_DSW0P_ENABLE,
        .LPDcsShortWriteOneP   = DSI_LP_DSW1P_ENABLE,
        .LPDcsShortReadNoP     = DSI_LP_DSR0P_ENABLE,
        .LPDcsLongWrite        = DSI_LP_DLW_ENABLE,      //DSI_LP_DLW_DISABLE,      ENABLE - LP mode， DISABLE - high speed mode
        .LPMaxReadPacket       = DSI_LP_MRDP_ENABLE,
        .AcknowledgeRequest    = DSI_ACKNOWLEDGE_ENABLE, //Enable LCD error reports
    },
    ```
    ```{note}
        _init->cfg.dsi.LPCmd.LPDcsLongWrite_调成`DSI_LP_DLW_ENABLE`后, _HAL_LCDC_Init_内部会主动将DBI的送数频率降到最低(48 * 16 / 126 = 6Mbps 左右，48为系统时钟，16为DBI带宽，126为最大分频系数)
    ```
        

- 调整DSI LP 模式的频率到屏幕能支持的范围(一般在6~20Mbps), 如下配置时LP频率 = 480 / 8 / 4 = 15Mbps(其中480为freq, 8 为固定值， 4为TXEscapeCkdiv)
    ```c
    static LCDC_InitTypeDef lcdc_int_cfg_dsi =
    {
        .lcd_itf = LCDC_INTF_DSI,
        .freq = DSI_FREQ_480MHZ,
        .color_mode = LCDC_PIXEL_FORMAT_RGB888,

        .cfg = {

            .dsi = {

                .Init = {
                    .AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_ENABLE,
                    .NumberOfLanes = RM69090_DSI_DATALANES,
                    .TXEscapeCkdiv = 0x4,
                },

                ...
                }
                ...
            }
            ...
    }
    ```

- 将drv_lcd 层的刷屏超时时间加长，否则默认的500ms在低速下容易超时，调整宏`MAX_LCD_DRAW_TIME`的值即可

- 使能`bf0_hal_dsi.c`里面的log(需要覆盖_HAL_DBG_printf_), 可以通过log检查通信过程发生的错误
    ```c
    #define DSI_LOG_D(...)   HAL_DBG_printf(__VA_ARGS__)
    #define DSI_LOG_E(...)   HAL_DBG_printf(__VA_ARGS__)
    ```

- 如果有条件可以用逻分仪或示波器抓取DATALANE0上P,N两个引脚的波形，解析各个命令、颜色格式等是否是期望的数据
```{note} 
读ID时，屏幕在bus turnaround之后有没有ack，若没有可能上电或者reset异常
```

## DSI 高速模式配置
- 低速模式可以读ID和刷期望的颜色后，可以调成高速模式，一般来说就通了。
    - 高速模式要把AcknowledgeRequest改成disable，否则容易引起发数fifo溢出，
    - 另外注意有的屏幕不同的颜色格式对应不一样的最高频率。

        ```
        在前面的LCDC_InitTypeDef那个结构体内配置，如下：
            .LPCmd = {
            ...
            .LPDcsLongWrite        = DSI_LP_DLW_DISABLE, //高速模式主要就是这里把长包改成高速模式
            ...
            .AcknowledgeRequest    = DSI_ACKNOWLEDGE_DISABLE, //高速模式下需要把Ack包关掉
        }
        ```    



## DSI 颜色，TE功能配置
- DSI颜色格式 参考[](../hal/dsi.md)
- DSI TE功能
    DSI TE有2条通路可选：通过DSI链路或者通过LCDC_TE的功能管脚
    如果走LCDC_TE管脚，需要同时指定一个物理管脚的pinmux 为 LCDC_TE功能(可以是LCDCx_SPI_TE/LCDCx_8080_TE)
    ```
        static LCDC_InitTypeDef lcdc_int_cfg_dsi =
        {
            .lcd_itf = LCDC_INTF_DSI,
            .freq = DSI_FREQ_480MHZ,
            .color_mode = LCDC_PIXEL_FORMAT_RGB888,

            .cfg = {
                .dsi = {
                        ...
                    .CmdCfg = {
                        ...
                        .TearingEffectSource   = DSI_TE_EXTERNAL, <<<----  DSI_TE_EXTERNAL 代表走LCDC_TE管脚， DSI_TE_DSILINK代表走DSI链路

    ```
    ```{note} 
    在55x系列芯片上，不支持DSI使用LCDC_TE的功能管脚，请参考上面“使用任意GPIO作为TE信号”章节的办法。
    ```

