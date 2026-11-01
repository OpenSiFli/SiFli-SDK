# 输入捕获示例

源码路径：example/hal/pulse_in
## 支持的平台
例程可以运行在以下开发板上：
* sf32lb52-lcd_n16r8
* sf32lb52-lcd_a128r16
* sf32lb52-nano_a128r16
* sf32lb52-nano_n16r16
* sf32lb56-lcd_a128r12n1
* sf32lb56-lcd_n16r12n1
* sf32lb58-lcd_a128r32n1_qspi
* sf32lb58-lcd_n16r32n1_qspi
## 概述
* 包含了GPT PWM输入捕获（Input Capture）的示例
* 例程先通过GPT输出PWM波形作为信号源，再用杜邦线将PWM信号引入捕获定时器，测量信号的周期、频率、脉宽和占空比
* PWM信号源：52x/56x使用GPTIM2_CH1，58x使用GPTIM1_CH2
* 捕获定时器：52x/56x使用GPTIM1_CH1，58x使用GPTIM3_CH2/PB01（GPTIM2的通道引脚在58x开发板上未引出）
* 支持中断采集（INPUT_CAPTURE_USE_IT=1，默认）和轮询采集（INPUT_CAPTURE_USE_IT=0）两种方式

## 例程的使用
### 编译和烧录
切换到例程project目录，运行scons命令执行编译(board=版型)：
```
scons --board=sf32lb52-lcd_n16r8 -j8
```
`build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`，按提示选择端口即可进行下载：

```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```

关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。
### 配置工程
本例程无需额外配置，使用默认配置即可编译运行。如需查看工程配置项，可执行：
```
sdk.py menuconfig --board=sf32lb52-lcd_n16r8
```
### 硬件连接
例程运行后会先输出一路PWM波形作为信号源（默认200Hz，20%占空比）并保持运行，用杜邦线将PWM输出引脚与输入捕获引脚短接，即可开始测量。

PWM输出与捕获引脚如下表：

|版型名称  | PWM输出 | PWM输出引脚 | 输入捕获定时器 | 输入捕获引脚 |
|--------|--------|---------|------------|---------|
|sf32lb52-nano    | GPTIM2_CH1 |    PA09 | GPTIM1_CH1 |    PA27 |
|sf32lb52-lcd     | GPTIM2_CH1 |    PA09 | GPTIM1_CH1 |    PA27 |
|sf32lb56-lcd     | GPTIM2_CH1 |    PA36 | GPTIM1_CH1 |    PA27 |
|sf32lb58-lcd     | GPTIM1_CH2 |    PA51 | GPTIM3_CH2 |    PB01 |

**注意**:
1. 58x使用LPSYS域的GPTIM3_CH2（PB01）
2. `HAL_PIN_Set(pad, func, flags, hcpu)`的第4个参数虽然注释为hcpu/lcpu选择，但实际由`pad`决定：HPSYS引脚（PA，`pad < PIN_PAD_MAX_H`）强制为hcpu，LPSYS引脚（PB，`pad >= PIN_PAD_MAX_H`）强制为lcpu，传入的0/1会被忽略
3. 中断入口重名的处理：例程通过HAL直接驱动GPT、不使用hwtimer设备，因此根目录`proj.conf`中关闭了hwtimer驱动（`# CONFIG_BSP_USING_TIM is not set`）。该驱动关闭后`drv_pwm_tim.c`会接管`GPTIMx_IRQHandler`（这些中断入口由`!BSP_USING_TIM`条件保护），所以与捕获定时器对应的PWM实例也要一并关闭：`PWMT1`对应GPTIM1、`PWMT3`对应GPTIM3，而例程在52x/56x用GPTIM1捕获、58x用GPTIM3捕获。因此根目录`proj.conf`中关闭了`PWMT1`（52x板默认开启该实例，其余芯片为no-op），`PWMT3`只对58x有意义、放在`sf32lb58x/proj.conf`中关闭（56x板虽默认开启`PWMT3`，但其捕获用GPTIM1，不受影响）。如需在本例程中同时使用hwtimer设备，请改用其它定时器做输入捕获
### 例程输出结果展示
* log输出（中断模式，`INPUT_CAPTURE_USE_IT=1`，默认）:
```
SFBL
Start PWM signal source!
GPT_clock 24000000,psc 2, Period 60000,Pulse 12000
PWM signal source setup done!
Start input capture demo (interrupt mode)!
connect PWM output pin to capture input pin with a jumper wire
captured(IT): period=5000 us, freq=200 Hz, pulse=1000 us, duty=20%
captured(IT): period=5000 us, freq=200 Hz, pulse=1000 us, duty=20%
```
* log输出（轮询模式，`INPUT_CAPTURE_USE_IT=0`）:
```
SFBL
Start PWM signal source!
GPT_clock 24000000,psc 2, Period 60000,Pulse 12000
PWM signal source setup done!
Start input capture demo!
connect PWM output pin to capture input pin with a jumper wire
captured: period=5000 us, freq=200 Hz, pulse=1000 us, duty=20%
captured: period=5000 us, freq=200 Hz, pulse=1000 us, duty=20%
```
* 每秒打印一次捕获结果：信号源为200Hz/20%占空比的PWM，对应周期5000us、脉宽1000us
* 若未接好杜邦线，会每秒打印一次：
```
no capture event, check the jumper wire!
```
### 捕获原理说明
* PWM输入捕获模式：周期通道（上升沿、直接TIx捕获）保存周期值，脉冲通道（下降沿、间接TIx捕获，与周期通道同引脚）保存脉宽值
* 从机复位模式：在周期边沿清零计数器，使CCR直接保存当前周期的period/pulse值，无需再计算相邻两次捕获的差值
* 52x/56x使用TI1FP1触发从机复位（GPTIM1_CH1），58x使用TI2FP2触发从机复位（GPTIM3_CH2）
* 捕获计数时钟为`PCLK1/(CAPTURE_PRESCALER+1)`（当前`CAPTURE_PRESCALER=9`，即分频10）。CCR为16位，故可测最大周期约为`65535/(PCLK1/(CAPTURE_PRESCALER+1))`；若目标芯片`PCLK1`更高导致长周期溢出，需调大该预分频。
* 捕获定时器选择（main.c宏定义）:
```c
#if defined(SF32LB52X) || defined(SF32LB56X)

#define CAPTURE_INSTANCE     hwp_gptim1
#define CAPTURE_CORE         CORE_ID_HCPU
#define CAPTURE_TRIGGER      GPT_TS_TI1FP1
#define CAPTURE_PERIOD_CH    GPT_CHANNEL_1
#define CAPTURE_PULSE_CH     GPT_CHANNEL_2
#define CAPTURE_PERIOD_FLAG  GPT_FLAG_CC1
#define CAPTURE_PULSE_FLAG   GPT_FLAG_CC2
#define CAPTURE_PERIOD_ACT   HAL_GPT_ACTIVE_CHANNEL_1
#define CAPTURE_PULSE_ACT    HAL_GPT_ACTIVE_CHANNEL_2
#define CAPTURE_IRQn         GPTIM1_IRQn
#define CAPTURE_IRQ_HANDLER  GPTIM1_IRQHandler

#elif defined(SF32LB58X)

#define CAPTURE_INSTANCE     hwp_gptim3
#define CAPTURE_CORE         CORE_ID_LCPU
#define CAPTURE_TRIGGER      GPT_TS_TI2FP2
#define CAPTURE_PERIOD_CH    GPT_CHANNEL_2
#define CAPTURE_PULSE_CH     GPT_CHANNEL_1
#define CAPTURE_PERIOD_FLAG  GPT_FLAG_CC2
#define CAPTURE_PULSE_FLAG   GPT_FLAG_CC1
#define CAPTURE_PERIOD_ACT   HAL_GPT_ACTIVE_CHANNEL_2
#define CAPTURE_PULSE_ACT    HAL_GPT_ACTIVE_CHANNEL_1
#define CAPTURE_IRQn         GPTIM3_IRQn
#define CAPTURE_IRQ_HANDLER  GPTIM3_IRQHandler

#endif
```
* 捕获引脚配置（main.c capture_pin_set）:
```c
#if defined(SF32LB52X) || defined(SF32LB56X)
    HAL_PIN_Set(PAD_PA27, GPTIM1_CH1, PIN_NOPULL, 1);
#elif defined(SF32LB58X)
    HAL_PIN_Set(PAD_PB01, GPTIM3_CH2, PIN_NOPULL, 0);
#endif
```
### 中断/轮询模式切换
通过`INPUT_CAPTURE_USE_IT`宏（main.c顶部）切换采集方式：
* `1`（默认）：中断方式，由`HAL_GPT_IC_CaptureCallback`回调在中断中采集数据，主循环等待信号量，每秒打印一次最新结果，log输出格式为`captured(IT): ...`
* `0`：轮询方式，主循环查询捕获标志寄存器，每秒打印一次结果

注意：中断方式需要例程自己定义捕获定时器的中断入口（`CAPTURE_IRQ_HANDLER`），如果工程中同时打开了占用同一定时器的其它驱动（如hwtimer实例），会出现中断入口重名，详见[支持的平台](#支持的平台)下的注意事项。

## 异常诊断
如果未能出现预期的log和波形输出，可以从以下方面进行故障排除：
* 硬件连接是否正常（杜邦线是否将PWM输出引脚与捕获引脚正确短接）
* 管脚配置是否正确
* 管脚对应的通道是否匹配
* 捕获无数据：确认信号源PWM正常输出，且杜邦线连接牢固

如有任何技术疑问，请在GitHub上提出[issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档
* [SiFli-SDK 快速入门](/quickstart/get-started.md)
* [GPT 外设驱动说明](https://docs.sifli.com/projects/sdk/latest/zh_CN/hal/gpt.html)

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |9/2026 |初始版本（由example/hal/pwm改造为输入捕获示例） |
