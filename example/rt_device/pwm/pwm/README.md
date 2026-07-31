# PWM示例

源码路径：example/rt_device/pwm/pwm
## 支持的平台
例程可以运行在以下开发板.
* sf32lb52-nano系列
* sf32lb52-lcd系列
* sf32lb56-lcd系列
* sf32lb58-lcd系列
* dpi/spi-hdk_lb57x系列

## 概述
* 包含了GPtimer通过IO口输出PWM的示例
* 包含了GPtimer输入捕获（Input Capture）的示例，通过GPTIM2_CH1（58x 为 GPTIM3_CH2/PB01）采集GPTIM1_CH2输出的PWM信号，验证周期和占空比

## 例程的使用
### 编译和烧录
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

关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。
### GPtimer输出PWM
#### 例程输出结果展示:
* log输出:
```
07-31 14:21:42:818    Start gtimer pwm + input capture demo!
07-31 14:21:42:821    pwm_set:percentage:20,period:1000000,freq:1000hz
07-31 14:21:42:822    gtimer pwm + input capture demo started!
07-31 14:21:42:823    connect PWM output pin to INCAP input pin with jumper wire
07-31 14:21:42:828    msh />

```
输入捕获需通过MSH命令手动启动。启动后例程每秒自动打印一次最新捕获的高低电平成对数据（每对两行，一对为完整周期，含占空比）：
```
07-31 14:21:46:087 TX:incap_start //启动输入捕获
07-31 14:21:46:096    input capture started on incap2c1
07-31 14:21:47:104    captured: pulse=200 us, HIGH
07-31 14:21:47:104    captured: pulse=800 us, LOW,  duty=20%
07-31 14:21:48:106    captured: pulse=200 us, HIGH
07-31 14:21:48:106    captured: pulse=800 us, LOW,  duty=20%
...
07-31 14:21:51:441 TX:incap_stop //停止输入捕获
07-31 14:21:51:453    input capture stopped

07-31 14:22:00:145 TX:pwm_set 42 1000 //设置PWM占空比为42%，周期1000us
07-31 14:22:00:159    pwm_set:percentage:42,period:1000000,freq:1000hz
07-31 14:22:21:616 TX:incap_start //启动输入捕获
07-31 14:22:21:629    input capture started on incap2c1
07-31 14:22:22:637    captured: pulse=420 us, HIGH
07-31 14:22:22:637    captured: pulse=580 us, LOW,  duty=42%
...
```
* 输出PWM波形(默认1000Hz,20%占空比)

![alt text](assets/gptimer_pwm.jpg)

#### PWM参数修改
* IO输出修改

物理位置指管脚对应在板子上的引脚排针位置

|版型名称  | PWM      | CHX     | 引脚(物理位置)            |    
|--------|------------|---------------|-------------------|
|sf32lb52-nano  | GPTIM1     | CH2    | PA20 (物理引脚在板子背面需要自行飞线引出)    | 
|sf32lb52-lcd    | GPTIM1     | CH2    | PA20 （10）                  |   
|sf32lb58-lcd | GPTIM1    | CH2  |PA51 （CONN2 28）                  |
|sf32lb56-lcd | GPTIM1    | CH2  |PA36  (40)                 |
|dpi/spi-hdk_lb57x | GPTIM1    | CH2  |PA51                  |

```c

    #if defined(SF32LB52X)/* 52系列默认PA20(物理位置10)输出 */
    HAL_PIN_Set(PAD_PA20, GPTIM1_CH2, PIN_NOPULL, 1);
    #elif defined (SF32LB58X)/* 58系列默认PA51输出 */
    HAL_PIN_Set(PAD_PA51, GPTIM1_CH2, PIN_NOPULL, 1);
    #elif defined (SF32LB56X)/* 56系列默认PA36输出 */
    HAL_PIN_Set(PAD_PA36, GPTIM1_CH2, PIN_NOPULL, 1);
    #elif defined (SF32LB57X)/* 57系列默认PA51输出 */
    HAL_PIN_Set(PAD_PA51, GPTIM1_CH2, PIN_NOPULL, 1);
    #endif


```
**注意**: 
1. 除55x芯片外,可以配置到任意带有PA_TIM功能的IO输出PWM波形
2.  HAL_PIN_Set 最后一个参数为hcpu/lcpu选择, 1:选择hcpu,0:选择lcpu 
* PWM周期period,脉宽pulse修改


### GPtimer输入捕获
输入捕获使用GPTIM2_CH1（58x 因 GPTIM2_CH1 引脚未引出，改用 GPTIM3_CH2/PB01）采集GPTIM1_CH2输出的PWM信号，需用杜邦线将两个引脚短接。

#### 硬件连接

|版型名称  | PWM输出 (GPTIM1_CH2) | 输入捕获 (GPTIM2_CH1, 58x为GPTIM3_CH2) |
|--------|---------------------|----------------------|
|sf32lb52-nano  | PA20 (引脚在背面需飞线)    | PA37 |
|sf32lb52-lcd   | PA20 (10)                | PA37 |
|sf32lb58-lcd   | PA51 (CONN2 28)          | PB01 |
|sf32lb56-lcd   | PA36 (40)                | PA39 |
|dpi/spi-hdk_lb57x | PA51                  | PA21 |

**用杜邦线将PWM输出引脚连接到输入捕获引脚。**

**设备名：52x/56x/57x 为 `incap2c1`，58x 为 `incap3c2`。**

#### MSH命令

例程启动后自动开启PWM输出，输入捕获需通过MSH命令手动控制：

```
pwm_set <percentage> <period_us>   # 设置PWM占空比和周期(us)，如: pwm_set 50 1000 (50%占空比, 1KHz)
incap_start                        # 启动输入捕获
incap_stop                         # 停止输入捕获
```

> `period_us` 有效范围为 1~65000：输入捕获驱动以 1MHz 时钟、16 位计数器测量脉宽，仅支持小于 65535us 的脉宽，超出后无法正确测量。

#### 输入捕获输出说明

启动输入捕获后，例程每秒打印一次最新捕获到的成对数据（每对两行：HIGH 一行、LOW/占空比一行，一对为完整周期）：

- `pulse=200 us, HIGH` 表示高电平持续200微秒
- `pulse=800 us, LOW, duty=20%` 表示低电平持续800微秒，占空比为 200/(200+800)=20%

捕获回调（`rx_indicate`）运行在 GPTIM 中断上下文，仅释放信号量通知主线程；缓冲区排水与打印均在主线程中完成，避免在中断中打印导致捕获边沿丢失、测量结果错乱。

若启动捕获后未连接杜邦线（或 PWM 占空比为 0%/100% 无边沿），每秒打印一次：

```
no capture event, check the jumper wire!
```


## 异常诊断
如果未能出现预期的log和PWM波形输出，可以从以下方面进行故障排除：
* 硬件连接是否正常
* 管脚配置是否正确
* 输入捕获无数据：检查杜邦线是否正确连接PWM输出引脚到输入捕获引脚，确认已执行 `incap_start`
* 输入捕获数据显示不全：确认对应芯片的输入捕获配置已使能（52x/56x/57x 为 `CONFIG_BSP_USING_INPUT_CAPTURE_GPTIM2=y`，58x 为 `CONFIG_BSP_USING_INPUT_CAPTURE_GPTIM3=y`，配置在各芯片的 `project/sf32lb5xx/proj.conf` 中）
* 持续提示 `no capture event, check the jumper wire!`：确认已执行 `incap_start`，PWM 信号正常（占空比为 0%/100% 时无捕获边沿）


## 参考文档
- 对于rt_device的示例，rt-thread官网文档提供了较详细说明，例如，可参考RT-Thread的[PWM设备文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/pwm/pwm)

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |10/2024 |初始版本 |
|0.0.2 | 12/2024| 2.0|
|0.0.3 |9/2026 |新增输入捕获功能（GPTIM2_CH1，58x 为 GPTIM3_CH2）与 57x 板卡支持；捕获打印移至线程上下文，每秒打印最新一对数据 |
