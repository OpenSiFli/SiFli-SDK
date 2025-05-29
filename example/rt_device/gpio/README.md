# GPIO 示例
源码路径：example\rt_device\gpio

## 支持的平台
例程可以运行在以下开发板.
* sf32lb52-lcd_n16r8
* sf32lb58-lcd_n16r64n4

## 示例概述
* 配置GPIO输出，输入中断操作，进行GPIO HAL函数演示
* 每一秒翻转GPIO_out电平值，输入GPIO在上升沿和下降沿触发中断，串口打印中断信息


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

> scons --board=sf32lb52-lcd_n16r8 -j8

切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：

> build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

> Uart Download

> please input the serial port num:5

#### SF587工程代码编译
切换到例程project目录，运行scons命令执行编译：

> scons --board=sf32lb58-lcd_n16r64n4 -j8

切换到例程`project/build_xx`目录，运行`download.bat`，程序通过JLink自动下载：

> build_sf32lb58-lcd_n16r64n4_hcpu\download.bat


### 例程输出结果展示:
* log输出:
```
   SFBL
   Serial:c2,Chip:4,Package:4,Rev:7  Reason:00000000
   [I/drv.adc] Get ADC configure fail
   \ | /
  - SiFli Corporation
   / | \     build on Jan 23 2025, 2.1.7 build bef6b3d8
   2020 - 2022 Copyright by SiFli team
   mount /dev sucess
   [D/USBD] No class register on usb device
   [I/drv.rtc] PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
   [I/drv.rtc] RTC use LXT RTC_CR=00000001
   [I/drv.rtc] Init RTC, wake = 0
   [I/drv.audprc] init 00 ADC_PATH_CFG0 0x606
   [I/drv.audprc] HAL_AUDPRC_Init res 0
   [I/drv.audcodec] HAL_AUDCODEC_Init res 0
   call par CFG1(3313)
   fc 9, xtal 2000, pll 2075
   call par CFG1(3313)
   fc 7, xtal 2000, pll 1657
   Start gpio rtt demo!
   msh />
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
    #define hwp_gpio hwp_gpio1
#elif defined(SF32LB58X)
    #define Pin_Out 124
    #define Pin_In 125
    #define hwp_gpio hwp_gpio2
#endif
```

```{note}
使用PB引脚时，引脚编号需要加96，详细说明请参考[](/drivers/gpio.md)
```

### 输出模式
配置`GPIO1 pin41`（即GPIO_A41）为输出模式

```C
struct rt_device_pin_mode m;
m.pin = Pin_Out;
m.mode = PIN_MODE_OUTPUT;
rt_device_control(device, 0, &m);
```

### 输入模式（有中断）

#### GPIO初始化
配置`GPIO1 pin42`（即GPIO_A42）为输入模式上升沿和下降沿检测

```C
//set pin to PIN_MODE_INPUT
m.pin = Pin_In;
m.mode = PIN_MODE_INPUT;
rt_device_control(device, 0, &m);

//set irq mode
rt_pin_attach_irq(m.pin, PIN_IRQ_MODE_RISING_FALLING, irq_handler, (void *)(rt_uint32_t)m.pin);
rt_pin_irq_enable(m.pin, 1);
```

#### 中断回掉函数
```C
void irq_handler(void *args)
{
    //set your own irq handle
    rt_kprintf("Interrupt occurred!\n");
    
    //read Pin_Out
    struct rt_device_pin_status st;
    st.pin = Pin_Out;
    rt_device_read(device, 0, &st, sizeof(struct rt_device_pin_status));
    rt_kprintf("Pin_Out %d has been toggle, value = %d\n", Pin_Out, st.status);

    //read Pin_In
    st.pin = Pin_In;
    rt_device_read(device, 0, &st, sizeof(struct rt_device_pin_status));
    rt_kprintf("Pin_In %d, value = %d\n", Pin_In, st.status);
    rt_kprintf(" \n");
}
```
## API参考
[](#hal-gpio)

## 更新记录
|版本  |日期    |发布说明 |
|:---  |:---    |:---    |
|0.0.1 |12/2024 |初始版本 |
|      |        |        |
