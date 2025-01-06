# PWM示例

源码路径：example\rt_device\pwm
## 支持的平台
例程可以运行在以下开发板.
* em-lb525
* em-lb587
## 概述
* 包含了GPtimer通过IO口输出PWM的示例



## 例程的使用
### 编译和烧录
切换到例程project目录，运行scons命令执行编译：

> scons --board=em-lb525 -j8

切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：


> build_em-lb525_hcpu\uart_download.bat

>Uart Download

>please input the serial port num:5

关于编译、下载的详细步骤，请参考[快速上手](quick_start)的相关介绍。
### GPtimer输出PWM
#### 例程输出结果展示:
* log输出:
```
Start gtimer pwm demo!
[32m][2821] I/pwm: pwm_set:percentage:20,period:1000000,freq:1000hz
[0m][32m][2848] I/drv.pwm: psc 2, Period 60000,
[0m][32m][2866] I/drv.pwm: Pulse 12000
[0m]gtimer pwm demo end!
msh />
```
* PA09输出PWM波形(默认200Hz,20%占空比)

![alt text](assets/gptimer_pwm.jpg)

#### PWM参数修改
* IO输出修改

物理位置指管脚对应在板子上的引脚排针位置

|版型名称  | PWM      | CHX     | 引脚(物理位置)            |    
|--------|------------|---------------|-------------------|
|525    | GPTIM1     | CH1    | PA20 （10）                  |   
|587  | GPTIM1    | CH2  |PA51 （CONN2 28）                  |



```c

    #if defined(BSP_USING_BOARD_EM_LB525XXX)/* 52系列默认PA20(物理位置10)输出 */
    HAL_PIN_Set(PAD_PA20, GPTIM1_CH2, PIN_NOPULL, 1);/*GPTIM1_CH1功能*/
    #elif defined (BSP_USING_BOARD_EM_LB587XXX)/* 587系列默认PA51输出 */
    HAL_PIN_Set(PAD_PA51, GPTIM1_CH2, PIN_NOPULL, 1);/*GPTIM1_CH2功能*/
    #endif


```
**注意**: 
1. 除55x芯片外,可以配置到任意带有PA_TIM功能的IO输出PWM波形
2.  HAL_PIN_Set 最后一个参数为hcpu/lcpu选择, 1:选择hcpu,0:选择lcpu 
* PWM周期period,脉宽pulse修改





## 异常诊断
如果未能出现预期的log和PWM波形输出，可以从以下方面进行故障排除：
* 硬件连接是否正常
* 管脚配置是否正确 


## 参考文档
- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc)

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |10/2024 |初始版本 |
|0.0.2 | 12/2024| 2.0|
| | | |
```