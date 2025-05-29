# GPIO 示例
源码路径：example/hal/gpio

## 支持的平台
例程可以运行在以下开发板.
* sf32lb52-lcd_n16r8
* sf32lb58-lcd_n16r64n4

## 示例概述
* 配置GPIO输出，输入中断操作，进行GPIO HAL函数演示
* 每一秒翻转GPIO_out电平值，输入GPIO在上升沿和下降沿触发中断，串口打印中断信息
## GPIO概述
HAL GPIO 模块提供抽象的软件接口操作硬件GPIO模块. 
HPSYS和LPSYS各有一个GPIO模块，支持的特性有:
- 输出模式
- 输入模式， 可检测输入电平触发中断，支持高电平、低电平、上升沿、下降沿和双沿检测

HPSYS的硬件GPIO模块为 `hwp_gpio1` (或称为GPIO_A), LPSYS的硬件GPIO模块为 `hwp_gpio2` (或称为GPIO_B). 

```{note}
如果需要设置GPIO管脚为其他功能，或者更改上下拉驱动能力，请参考pinmux的设置[PINMUX](#hal-pinmux)
```

详细的API说明参考[GPIO](#hal-gpio) 


## 例程的使用

### 硬件连接
* 注:将输入GPIO和输出GPIO通过跳线相连，这样就能把GPIO_out电平跳变赋值给GPIO_in以达到进入中断打印信息的效果

|开发板    |OUT管脚 |OUT管脚名称|IN管脚 |IN管脚名称 |
|:---     |:---    |:---      |:---   |:---      |
|sf32lb52-lcd_n16r8 |5       |PA41      |3      |PA42      |
|sf32lb58-lcd_n16r64n4 |5       |PB28      |3      |PB29      |

* 更详细的引脚定义请参考\
`[sf32lb52-lcd_n16r8]()`\
`[sf32lb58-lcd_n16r64n4]()`

### 编译和烧录
#### SF525工程代码编译
切换到例程project目录，运行scons命令执行编译：

```
scons --board=sf32lb52-lcd_n16r8 -j8
```

运行`build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`，按提示选择端口即可进行下载：

```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```

#### SF587工程代码编译
切换到例程project目录，运行scons命令执行编译：

```
scons --board=sf32lb58-lcd_n16r64n4 -j8
```

运行`build_sf32lb58-lcd_n16r64n4_hcpu\download.bat`，程序通过JLink自动下载：

```
build_sf32lb58-lcd_n16r64n4_hcpu\download.bat
```


### 例程输出结果展示:
* log输出:
```
   Serial:c2,Chip:2,Package:0,Rev:1  Reason:00000000
   \ | /
  - SiFli Corporation
   / | \     build on Dec 18 2024, 2.1.7 build bb6471d6
   2020 - 2022 Copyright by SiFli team
   mount /dev sucess
   [D/USBD] No class register on usb device
   [I/drv.rtc] PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
   [I/drv.rtc] RTC use LXT RTC_CR=00000000
   [I/drv.rtc] Init RTC, wake = 0
   [I/drv.sdhci] rt_hw_sdmmc_init 2 begin
   [I/drv.sdhci] host version = 2
   [I/drv.sdhci] SDHCI clock 288000000
   [I/drv.sdhci] Maximum Clock Supported by HOST : 288 MHz 
   [I/drv.sdhci] host minclock 400000  host maxclock 288000000  
   [I/drv.sdhci] SDHCI controller on sdmmc2 using DMA
   [I/drv.sdhci] Add host success
   [I/drv.sdhci] rt_hw_sdmmc_init 2 done
   [I/drv.audprc] init 00 ADC_PATH_CFG0 0x924
   [I/drv.audprc] HAL_AUDPRC_Init res 0
   [I/drv] HAL_AUDCODEC_Init res 0
   call par CFG1(35bb)
   fc 9, xtal 2000, pll 2054
   call par CFG1(35bb)
   fc 9, xtal 2000, pll 2054
   Start gpio demo!
```
每秒GPIO_out电平反转时:
```
Interrupt occurred!
Pin_Out 41 has been toggle, value = 1
Pin_In 42, value = 1
```
1秒后翻转:
```
Interrupt occurred!
Pin_Out 41 has been toggle, value = 0
Pin_In 42, value = 0
```
## 例程说明

```{note}
例程说明以sf32lb52-lcd_n16r8为例
```

### 宏定义
适配不同开发板引脚不同

```C
#ifdef SF32LB52X
    #define Pin_Out 41
    #define Pin_In 42
    #define GPIO_IRQn GPIO1_IRQn
    #define hwp_gpio hwp_gpio1
    #define RCC_MOD_GPIO RCC_MOD_GPIO1
#elif defined(SF32LB58X)
    #define Pin_Out 28
    #define Pin_In 29
    #define GPIO_IRQn GPIO2_IRQn
    #define hwp_gpio hwp_gpio2
    #define RCC_MOD_GPIO RCC_MOD_GPIO2
#endif
```

### 输出模式
配置`GPIO1 pin41`（即GPIO_A41）为输出模式

```C
GPIO_InitStruct.Pin = Pin_Out;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(hwp_gpio, &GPIO_InitStruct);
```

### 输入模式（有中断）

#### GPIO初始化
配置`GPIO1 pin42`（即GPIO_A42）为输入模式上升沿和下降沿检测

```C
PIO_InitStruct.Pin = Pin_In;
GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; //Set interrupt to trigger on raising and falling edge
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(hwp_gpio, &GPIO_InitStruct);
HAL_NVIC_SetPriority(GPIO_IRQn, 2, 1); // Configure NVIC priority
```
#### 中断服务函数
根据启动文件里中断向量表，找出中断服务函数
```C
void GPIO1_IRQHandler(void) // Define the interrupt siervice routine (ISR) according to the interrupt vector table
{
    HAL_GPIO_IRQHandler(hwp_gpio);  
}
```

#### 中断回掉函数
```C
// override the weak Callback to add user defined action, it's called by HAL_GPIO_EXTI_IRQHandler 
void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Pin_In)
    {
        rt_kprintf("Interrupt occurred!\n");
        rt_kprintf("Pin_Out %d has been toggle, value = %d\n", Pin_Out, HAL_GPIO_ReadPin(hwp_gpio, Pin_Out));
        rt_kprintf("Pin_In %d, value = %d\n", Pin_In, HAL_GPIO_ReadPin(hwp_gpio, Pin_In));
        rt_kprintf(" \n");
    }
}
```
## API参考
[](#hal-gpio)

## 更新记录
|版本  |日期    |发布说明 |
|:---  |:---    |:---    |
|0.0.1 |12/2024 |初始版本 |
|      |        |        |
