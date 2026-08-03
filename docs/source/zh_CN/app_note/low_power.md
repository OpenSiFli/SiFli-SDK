# 低功耗开发指南

## 1 简介
```{only} SF32LB52X or SF32LB56X or SF32LB55X
SiFli MCU 芯片为双核 Cortex‑M33 STAR SoC 芯片。大核 HCPU 工作频率为 0~240MHz，属于 HPSYS 子系统，适用于进行图形、音频、神经网络等高性能运算；小核 LCPU 工作频率为 0~48MHz，属于 LPSYS 子系统，适用于运行蓝牙、传感器收集与运算等任务。
```
```{only} SF32LB58X
SF32LB58X为三核芯片(双大核性能处理器 + 小核低功耗处理器)，芯片中的双大核性能处理器最高工作频率 240MHz，单核 CoreMark 跑分高达 984，功耗效率8.29uA/CoreMark，用于提供丰富应用和流畅人机交
互所需的图形和音频算力。小核低功耗处理器最高工作频率96MHz，CoreMark 跑分达到 394，功耗效率3.88uA/CoreMark，在作为低功耗传感器控制中心（Sensor Hub）的同时兼顾运行蓝牙协议栈。
```
低功耗开发例程参考 `example\pm\classical`。

:::{only} SF32LB56X
功耗测试报告参考：[功耗测试报告](https://docs.sifli.com/projects/rpt5602_sf32lb56x-low-power-measurement-report/latest/zh_CN/index.html)
:::

:::{only} SF32LB52X
功耗测试报告参考：[功耗测试报告](https://docs.sifli.com/projects/rpt5202_sf32lb52x-low-power-measurement-report/latest/zh_CN/index.html)
:::

## 2 配置低功耗模式

### 2.1 打开低功耗模式
```{only} SF32LB52X
在 工程目录下运行 `sdk.py menuconfig` 打开软件配置菜单：

1. 使能低功耗功能(`Enable Low power support`)：
    - 路径：Sifli middleware → Enable Low power support
    - 开启：Enable Low power support
        - 宏开关：`BSP_USING_PM`
        - 作用：开启低功耗功能

```{figure} ../../assets/enable_pm.png
:align: center
图  enable Low Power 配置菜单
```
```{only} SF32LB52X
2. 选择低功耗模式(`Enable Deep Mode`)：
    - 路径：RTOS → RT-Thread Components → Device Drivers → Using Power Management device drivers → Select PM Mode
    - 选择：Enable Deep Mode
        - 宏开关：`PM_DEEP_ENABLE`
        - 作用：选择低功耗模式为Deep Sleep 模式

```{figure} ../../assets/deep.png
:align: center
图  deep Sleep 配置菜单 
```
```{only} SF32LB52X
3. 开启低功耗调试开关，打开后会输出低功耗相关日志 (非必选，开启后会有低功耗日志打印，会占用时间，对功耗有影响) (`Enable PM Debug`)
    - 路径：Sifli middleware → Enable Low power support -> Enable PM Debug
        - 选择：Enable PM Debug
            - 宏开关：`BSP_PM_DEBUG`
            - 作用：开启低功耗日志调试开关

```{figure} ../../assets/low_power11.png
:align: center
图  debug 配置菜单  
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X or SF32LB57X
在 工程目录下运行 `sdk.py menuconfig` 打开软件配置菜单：

1. 使能低功耗功能(`Enable Low power support`)：
    - 路径：Sifli middleware → Enable Low power support
    - 开启：Enable Low power support
        - 宏开关：`BSP_USING_PM`
        - 作用：开启低功耗功能
        
```{figure} ../../assets/enable_pm.png
:align: center
图 enable Low Power 配置菜单
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X or SF32LB57X
2. 选择低功耗模式(`Enable Standby Mode`)：
    - 路径：RTOS → RT-Thread Components → Device Drivers → Using Power Management device drivers → Select PM Mode
    - 选择：Enable Standby Mode
        - 宏开关：`PM_Standby_ENABLE`
        - 作用：选择低功耗模式为Standby Sleep 模式

```{figure} ../../assets/stabdby.png
:align: center
图  Standby Sleep 配置菜单   
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X or SF32LB57X
3. 开启低功耗调试开关，打开后会输出低功耗相关日志 (非必选，开启后会有低功耗日志打印，会占用时间，对功耗有影响) (`Enable PM Debug`)
    - 路径：Sifli middleware → Enable Low power support -> Enable PM Debug
        - 选择：Enable PM Debug
            - 宏开关：`BSP_PM_DEBUG`
            - 作用：开启低功耗日志调试开关

```{figure} ../../assets/low_power11.png
:align: center
图  debug 配置菜单  
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X or SF32LB57X
- 注意：如果 LCPU 打开了 STANDBY 模式，那么 HCPU 也必须打开 STANDBY 模式。
```
4) 配置好后，确认工程配置文件 `rtconfig.h` 内已包含下面的定义：
```{only} SF32LB52X
```c
#define RT_USING_PM 1           // 启动 PM 模块
#define PM_DEEP_ENABLE 1        // DEEP 休眠模式
#define BSP_USING_PM 1          // 启动 PM 模块
#define BSP_PM_DEBUG 1          // 打印 PM[S], PM[W] 的日志(非必选)
```
```{only} SF32LB58X or SF32LB56X or SF32LB55X or SF32LB57X
```c
#define RT_USING_PM 1           // 启动 PM 模块
#define PM_STANDBY_ENABLE 1     // STANDBY 休眠模式
#define BSP_USING_PM 1          // 启动 PM 模块
#define BSP_PM_DEBUG 1          // 打印 PM[S], PM[W] 的日志(非必选)
```

### 2.2 关闭低功耗模式

在 工程目录下运行 `sdk.py menuconfig`，按照 开启低功耗 的路径反向取消对应配置即可。

## 3 按键唤醒配置

配置方法参照 `SDK\example\pm\classical` 例程。

**待机唤醒配置（适用于 standby/deep 唤醒）**

如下为 HAL 层待机唤醒 API 示例,可以使用 `HAL_LPAON_QueryWakeupPin() `与 `HAL_HPAON_QueryWakeupPin()` 来获取唤醒引脚对应的PIN序号;若 IO 唤醒需要处理事件，还需配置 GPIO 中断：

```{only} SF32LB55X
```c
HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN3, AON_PIN_MODE_LOW);         // 55x PA80 #WKUP_A3
HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_PIN5, AON_PIN_MODE_NEG_EDGE);    // 55x PB48 #WKUP_PIN5
// 配置是否生效, 可对照芯片手册查看对应寄存器:
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_hpsys_aon->WSR, hwp_hpsys_aon->WER); // hcpu
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_lpsys_aon->WSR, hwp_lpsys_aon->WER); // lcpu
```

```{only} SF32LB52X
52系列deepsleep模式下的休眠不需要额外配置唤醒pin，所有pin都可以唤醒，唤醒源走WSR_GPIO1
```

```{only} SF32LB56X
```c
HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN3, AON_PIN_MODE_LOW);         // 56x PB35 #WKUP_PIN3
HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_PIN5, AON_PIN_MODE_NEG_EDGE);    // 56x PA50 #WKUP_PIN5
// 配置是否生效, 可对照芯片手册查看对应寄存器:
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_hpsys_aon->WSR, hwp_hpsys_aon->WER); // hcpu
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_lpsys_aon->WSR, hwp_lpsys_aon->WER); // lcpu
```
```{only} SF32LB58X
```c
HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN3, AON_PIN_MODE_LOW);         // 58x PB57 #WKUP_PIN3
HAL_LPAON_EnableWakeupSrc(LPAON_WAKEUP_SRC_PIN5, AON_PIN_MODE_NEG_EDGE);    // 58x PB59 #WKUP_PIN5
// 配置是否生效, 可对照芯片手册查看对应寄存器:
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_hpsys_aon->WSR, hwp_hpsys_aon->WER); // hcpu
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_lpsys_aon->WSR, hwp_lpsys_aon->WER); // lcpu
```

**关机唤醒配置（适用于 hibernate 唤醒）**
```{only} SF32LB55X
55 系列 MCU：Hibernate 下只有 LCPU 的唤醒 PIN0‑5 具有唤醒功能；具体参见 55 系列用户手册 PMUC WER 寄存器配置。
```c
// 55x 配置方法:
HAL_PMU_EnablePinWakeup(5, AON_PIN_MODE_NEG_EDGE); // 55x PB48 #WKUP_PIN5
rt_kprintf("CR:0x%x,WER:0x%x\n", hwp_pmuc->CR, hwp_pmuc->WER);
```

```{only} SF32LB52X or SF32LB56X or SF32LB57X or SF32LB58X
55 系列之后 MCU：允许同时存在两个唤醒源 PIN0 和 PIN1，每个唤醒源可指定到任意 HCPU/LCPU 唤醒 PIN；具体参见用户手册 PMUC CR 寄存器配置。
```c
// 58x/56x/52x 配置方法:
HAL_PMU_SelectWakeupPin(0, HAL_HPAON_QueryWakeupPin(hwp_gpio1, BSP_KEY1_PIN)); // select PA34 to wake_pin0
HAL_PMU_EnablePinWakeup(0, AON_PIN_MODE_HIGH);                                  // enable wake_pin0 
rt_kprintf("CR:0x%x,WER:0x%x\n", hwp_pmuc->CR, hwp_pmuc->WER);
```



Hibernate 关机后，唤醒后等同于冷启动（但有 `PM_HIBERNATE_BOOT` 标志位），不同于 Standby 唤醒能恢复到原有程序继续运行。唤醒 PIN 和电平模式由 PMU 寄存器控制，可打印 WER/CR 寄存器核对；若同一 IO 同时用于待机与关机唤醒，两者配置都需要生效。

## 4 低功耗模式调试方法

### 4.1 低功耗模式

- `PM_SLEEP_MODE_IDLE`:
CPU 进 休闲(idle)模式，CPU停在WFI 或 WFE，系统中有高速时钟(HRC/HXT/DBLR/DLL)， 所有外设均可使能并产生中断。
```{only} SF32LB52X 
- `PM_SLEEP_MODE_LIGHT`:
CPU进入浅睡(light)，CPU 停在 WFI 指令,系统中高速时钟关闭，CPU相关的外围设备停止工作,但是不掉电，系统切换到32K时钟。
可以由低功耗定时器(LPTIM)，RTC，BLE MAC(LCPU Only)，Mailbox(其他CPU)，或者pin唤醒。唤醒时间30us-100us。
唤醒之后继续在WFI之后的指令运行。
```
```{only} SF32LB56X or SF32LB55X or SF32LB58X or SF32LB57X 
- `PM_SLEEP_MODE_LIGHT`:
CPU进入浅睡(light)，CPU 停在 WFI 指令,系统中高速时钟关闭，CPU相关的外围设备停止工作,但是不掉电，系统切换到32K时钟。
可以由低功耗定时器(LPTIM)，RTC，BLE MAC(LCPU Only)，Mailbox(其他CPU)，或者特定的唤醒pin来唤醒。唤醒时间30us-100us。
唤醒之后继续在WFI之后的指令运行。
```
- `PM_SLEEP_MODE_DEEP`:
和PM_SLEEP_MODE_LIGHT一样，但是系统的供电切换到RET_LDO，唤醒时间增加到100us-1ms。
唤醒之后继续在WFI之后的指令运行。

- `PM_SLEEP_MODE_STANDBY`:
CPU进入待机(standby)模式，系统中高速时钟关闭，CPU相关的外围设备掉电，RAM除了系统配置的部分，都停止供电，PIN的状态保持，系统的供电切换到RET_LDO。
可以由低功耗定时器，RTC，BLE MAC(LCPU Only)，Mailbox(其他CPU)，或者特定的唤醒pin来唤醒。唤醒时间1ms-2ms。
唤醒之后，系统会重新启动，根据AON寄存器中的低功耗模式指示，软件判断系统从standby模式启动，以便与冷启动区别。

各种低功耗模式的电流见表4‑1。若存在 PSRAM，HCPU 会将掉电 RAM 需保留的数据备份到 PSRAM，唤醒后再恢复；无 PSRAM 则备份到 64KB Retention RAM。下文如无特别说明，“进入睡眠”指进入 IDLE 之外的低功耗模式，“唤醒”指退出 IDLE 之外的低功耗模式。

表4‑1：低功耗模式
```{only} SF32LB52X 
| 低功耗模式           | CPU 状态 | 外设状态 | SRAM                                                     | 唤醒源   | 唤醒时间     | 
|----------------------|----------|----------|----------------------------------------------------------|----------|---------------------------------------------------------|
| PM_SLEEP_MODE_IDLE   | stop     | run      | 可访问                                                   | 任意中断 | <1µs       |
| PM_SLEEP_MODE_DEEP   | stop     | stop     | LPSYS：不可访问，全保留<br>HPSYS：不可访问，全保留 | RTC, 唤醒PIN, IO(PA),<br>LPTIM1,蓝牙 | ~ 250us    |
| PM_SLEEP_MODE_STANDBY| reset    | reset    | LPSYS：不可访问，全保留<br>HPSYS：不可访问，只保留 384KB | RTC, 唤醒PIN, <br>LPTIM1,蓝牙 | ~ 1ms    | 
```
```{only} SF32LB55X 
| 低功耗模式           | CPU 状态 | 外设状态 | SRAM                                                     | 唤醒源   | 唤醒时间     | 
|----------------------|----------|----------|----------------------------------------------------------|----------|---------------------------------------------------------|
| PM_SLEEP_MODE_IDLE   | stop     | run      | 可访问                                                   | 任意中断 | <1µs       |
| PM_SLEEP_MODE_DEEP   | stop     | stop     | LPSYS：不可访问，全保留<br>HPSYS：不可访问，全保留 |RTC, 唤醒PIN, IO(PA),<br>LPTIM1, LPSYS, MAILBOX2| ~ 250us   |
| PM_SLEEP_MODE_STANDBY| reset    | reset    | LPSYS：不可访问，全保留<br>HPSYS：不可访问，只保留 64KB | RTC, 唤醒PIN, <br>LPTIM1, LPSYS, MAILBOX2 | ~ 1.5ms      | 
```
```{only} SF32LB56X 
| 低功耗模式           | CPU 状态 | 外设状态 | SRAM                                                     | 唤醒源   | 唤醒时间     | 
|----------------------|----------|----------|----------------------------------------------------------|----------|---------------------------------------------------------|
| PM_SLEEP_MODE_IDLE   | stop     | run      | 可访问                                                   | 任意中断 | <1µs       |
| PM_SLEEP_MODE_DEEP   | stop     | stop     | LPSYS：不可访问，全保留<br>HPSYS：不可访问，全保留 |RTC, 唤醒PIN, IO(PA),<br>LPTIM1, LPSYS, MAILBOX2| ~ 250us   |
| PM_SLEEP_MODE_STANDBY| reset    | reset    | LPSYS：不可访问，全保留<br>HPSYS：不可访问，只保留 64KB | RTC, 唤醒PIN, <br>LPTIM1, LPSYS, MAILBOX2 | ~ 1.5ms      | 
```
```{only} SF32LB58X 
| 低功耗模式           | CPU 状态 | 外设状态 | SRAM                                                     | 唤醒源   | 唤醒时间     | 
|----------------------|----------|----------|----------------------------------------------------------|----------|---------------------------------------------------------|
| PM_SLEEP_MODE_IDLE   | stop     | run      | 可访问                                                   | 任意中断 | <1µs       |
| PM_SLEEP_MODE_DEEP   | stop     | stop     | LPSYS：不可访问，全保留<br>HPSYS：不可访问，全保留 | RTC, 唤醒 PIN, IO(PA),LPTIM1,<br> LPSYS,
 MAILBOX2 | ~ 250us   |
| PM_SLEEP_MODE_STANDBY| reset    | reset    | LPSYS：不可访问，全保留<br>HPSYS：不可访问，只保留 64KB | RTC, 唤醒 PIN,  LPTIM1,<br>,LPSYS,
 MAILBOX2| ~ 1.5ms       | 
```
### 4.2 关机模式

除了每个子系统提供上述四种低功耗模式，芯片还提供两个系统级关机模式
- Hibernate：所有子系统掉电，系统切到 32K 晶体，可被 PIN 和 RTC 唤醒（RTC 唤醒时间准确）。接口：`HAL_PMU_EnterHibernate`。
- Shutdown：所有子系统掉电，系统切到 RC10K，可被 PIN 和 RTC 唤醒（RTC 唤醒时间不准确）。接口：`HAL_PMU_EnterShutdown`。

表4‑2：关机模式

| 低功耗模式 | CPU 状态 | 外设状态 | SRAM        | IO  | 唤醒源    | 唤醒时间 |
|------------|----------|----------|-------------|-----|-----------|----------|
| Hibernate  | reset    | reset    | 数据不保留  | 高阻| RTC 和 PIN| >2ms     | 
| Shutdown   | reset    | reset    | 数据不保留  | 高阻| RTC 和 PIN| >2ms     | 

注：电流数据仅供参考，实际数值会因外设使能和 IO 设置不同而变化。stop 表示停止工作，退出低功耗后无需重新配置即可继续；reset 表示退出后已被复位（CPU 从 ROM 开始，外设需重新配置）。

不满足睡眠但仍需降低功耗，可参考 WFI 自动降频与 场景化动态调频。
### 4.3 WFI 自动降频
当进入 `IDLE` 线程但不满足睡眠条件时，可通过降低 HCPU 频率来降低 `WFI` 期间电流。允许降频的前提：所有高速外设不在工作。高速外设包括：
- EPIC
- EZIP
- LCDC
- USB
- SD

说明：EPIC/EZIP 的在忙判定集成在 SDK 自带的 LVGL 实现中；如未使用该实现，需在外设开始/结束工作时分别调用 `rt_pm_hw_device_start`/`rt_pm_hw_device_stop` 指示忙闲，避免在忙时降频执行 WFI。LCDC/USB/SD 的忙闲判定集成在 RT-Thread 的 LCD Device 驱动中。
```{only} SF32LB52X 
降频后的 WFI 频率由 `HAL_RCC_HCPU_SetDeepWFIDiv` 配置。当有音频外设工作时，仅可降至 48MHz；其它情况下可降至 4MHz。同时需要将 `hwp_hpsys_rcc->DBGR` 的 `HPSYS_RCC_DBGR_FORCE_HP` 位置 1。
```
```{only} SF32LB56X or SF32LB58X  
降频后的 WFI 频率由 `HAL_RCC_HCPU_SetDeepWFIDiv` 配置。当有音频外设工作时，仅可降至 48MHz；其它情况下可降至 1MHz。
```
### 4.4 场景化动态调频
对于无需高性能的场景，大核可降频降压以降低工作功耗（例如灭屏后仅运行抬腕算法可降至 48MHz）。虽然频率降低使执行时间变长，但总能耗（电流 × 时间）可能更低。可在不同场景下实测选择最优模式。

使用 `rt_pm_run_enter` 配置当前运行模式，HCPU 支持以下四种模式（应用启动默认 `PM_RUN_MODE_HIGH_SPEED`）；向高速模式切换立即生效，向低速模式切换会延迟到 IDLE 线程执行时完成：

| 模式                      | 系统时钟（MHz） |
|---------------------------|-----------------|
| PM_RUN_MODE_HIGH_SPEED    | 240             |
| PM_RUN_MODE_NORMAL_SPEED  | 144             |
| PM_RUN_MODE_MEDIUM_SPEED  | 48              |
| PM_RUN_MODE_LOW_SPEED     | 24              |

SDK 还提供 `pm_scenario_start`/`pm_scenario_stop` 便于按场景切换。目前支持 `UI` 与 `Audio`：
- 其中任一开启 → 使用 HIGH_SPEED；
- `UI` 与 `Audio` 均未开启 → 使用 MEDIUM_SPEED。
### 4.5 低功耗流程

方案中，只有在熄屏之后，HPSYS 才能进入睡眠模式；亮屏状态下，当 HCPU 不工作时，HPSYS 只能进入 IDLE 模式。只有当 HPSYS 进入睡眠后，LPSYS 才能进入睡眠；若 HPSYS 未睡，即使 LCPU 空闲，LPSYS 也只能进入 IDLE（52 系列例外，HCPU 与 LCPU 可独立休眠）。在 HPSYS 已睡眠的情况下，LPSYS 可自由进出睡眠，不必唤醒 HPSYS。

#### 4.5.1 熄屏

锁屏时间可在设置界面选择。当屏幕无操作超过锁屏时间后屏幕熄灭，在 IDLE 线程中检查睡眠条件，若满足，则 HPSYS 进入睡眠，LPSYS 随之也可进入睡眠。

```{figure} ../../assets/low_power13.png
:align: center
图 4.1 熄屏流程
```

#### 4.5.2 HPSYS 唤醒
```{only} SF32LB52X 
HPSYS 可以由低功耗定时器(LPTIM)，RTC，BLE MAC(LCPU Only)，Mailbox(其他CPU)，deeepsleep模式下任意pin唤醒。比如说将当按键按下，HPSYS则会被唤醒。
```
```{only} SF32LB55X or SF32LB58X or SF32LB56X
HPSYS 可以由以下几种方式唤醒：低功耗定时器(LPTIM)、RTC、BLE MAC(仅LCPU)、Mailbox(来自其他CPU)或者特定的唤醒引脚。例如，当我们使能按键引脚的唤醒功能后，按下按键时HPSYS就会被唤醒。
```
以按键唤醒亮屏为例，流程如图 4.2；亮屏后进入新一轮熄屏判断流程。来自手机 APP 的 setting 事件触发的唤醒流程如图 4.3，处理完 Setting 请求回到 IDLE 线程可立即进入睡眠。

```{figure} ../../assets/low_power14.png
:align: center
图 4.2 按键亮屏熄屏流程
```

```{figure} ../../assets/low_power25.png
:align: center
图 4.3 收到手机 Setting 事件的唤醒流程
```
```{only} SF32LB55X or SF32LB58X or SF32LB56X
#### 4.5.3 LPSYS 唤醒

LPSYS 可被以下事件唤醒：
- 按键
- HPSYS 醒来
- Sensor 数据采集定时器超时
- BLE 周期性定时器超时


#### 4.5.3 跨电源域外设共享与睡眠约束

SiFli 的 HCPU 与 LCPU 分属不同电源域，分别控制各自电源域内的外设。为支持资源共享：
- 当 LCPU 处于 IDLE/ACTIVE（未进入睡眠）时，HCPU 可以使用 LCPU 电源域内的外设；但必须避免两核同时使用同一外设，否则在并行运行时会发生硬件资源冲突。
- 常见场景是 HCPU 使用 LCPU 外设。此时只要 HCPU 需要该外设，LCPU 就必须保持唤醒（不可进入睡眠），否则访问将失败。
- 若 LCPU 进入待机模式，其相关外设会掉电，进而约束 HCPU 的最低功耗模式：HCPU 也需进入待机路径。唤醒顺序上，需要先唤醒 LCPU 并重新初始化相关外设（包括 LCPU 外设），随后 HCPU 才能正常使用这些资源。
```
```{only} SF32LB55X or SF32LB58X or SF32LB56X
关于待机下的现场保留：
- HCPU：在待机前会将需保留的数据/上下文备份至 PSRAM；唤醒后从 PSRAM 恢复运行现场（无 PSRAM 时仅依赖 HPSYS 的 Retention 区域）。
- LCPU：保持所有 RAM 供电，将 CPU 寄存器上下文保存在 RAM 中；唤醒后直接从 RAM 恢复现场。
```
```{only} SF32LB52X
关于待机下的现场保留：
- HCPU：DEEPSLEEP 模式下,RAM全保留,唤醒后直接从 RAM 恢复现场。
- LCPU：保持所有 RAM 供电，将 CPU 寄存器上下文保存在 RAM 中；唤醒后直接从 RAM 恢复现场。
```
### 4.6 日志解读

HCPU 和 LCPU 会通过 console 输出日志。按 2.1 节的方法打开低功耗调试开关后，可以在日志中搜索下列关键字分析流程。

表4‑3：日志关键字解析

| 日志          | 含义                                         |
|---------------|----------------------------------------------|
| gui_suspend   | 熄屏                                         |
| gui_resume    | 亮屏                                         |
| [pm]S: mode,gtime | 进入睡眠；mode=2 表示 LIGHT，4 表示 STANDBY；gtime 单位 32768Hz |
| [pm]W: gtime  | 退出睡眠；gtime 单位 32768Hz                 |
| [pm]WSR:0xXXX | 唤醒原因（按寄存器位含义解析）               |

`gtime` 在 HCPU 与 LCPU 侧同步，例如：在 2136602 时刻进入睡眠、2142330 时刻唤醒，则睡眠时长为 `sleep_time=(2142330-2136602)/32768=175ms`；若 `WSR=0x200`，表示由 LPSYS 触发的 mailbox 中断唤醒。WSR 每个比特含义参见芯片手册。

```{figure} ../../assets/low_power15.png
:align: center
图 4.4 低功耗日志示例
```
```{only} SF32LB55X
* HPSYS 的 WSR 含义

| 比特域 | 含义                    |
|--------|-------------------------|
| [0]    | RTC 唤醒                |
| [1]    | LPTIM1 唤醒            |
| [2]    | PIN0 唤醒              |
| [3]    | PIN1 唤醒              |
| [4]    | PIN2 唤醒              |
| [5]    | PIN3 唤醒              |
| [8]    | LPSYS 手动唤醒 HPSYS   |
| [9]    | LPSYS 使用 Mailbox 唤醒 HPSYS |

* LPSYS 的 WSR 含义

| 比特域 | 含义                    |
|--------|-------------------------|
| [0]    | RTC 唤醒                |
| [1]    | LPTIM2 唤醒            |
| [2]    | LPCOMP1 唤醒           |
| [3]    | LPCOMP2 唤醒           |
| [4]    | BLE 唤醒               |
| [5]    | PIN0 唤醒              |
| [6]    | PIN1 唤醒              |
| [7]    | PIN2 唤醒              |
| [8]    | PIN3 唤醒              |
| [9]    | PIN4 唤醒              |
| [10]   | PIN5 唤醒              |
| [11]   | HPSYS 手动唤醒 LPSYS   |
| [12]   | HPSYS 使用 Mailbox 唤醒 LPSYS |
```
```{only} SF32LB56X
* HPSYS 的 WSR 含义

| 比特域 | 含义                    |
|--------|-------------------------|
| [0]    | RTC 唤醒                |
| [1]    | GPIO1 唤醒            |
| [2]    | LPTIM1 唤醒              |
| [6]    | LPSYS 手动唤醒 HPSYS           |
| [7]    | LPSYS 使用 Mailbox 唤醒 HPSYS  |
| [8]    | PIN0 唤醒   |
| [9]    | PIN1 唤醒   |
| [10]    | PIN2 唤醒   |
| [11]    | PIN3 唤醒   |
| [12]    | PIN4 唤醒   |
| [13]    | PIN5 唤醒   |
| [14]    | PIN6 唤醒   |
| [15]    | PIN7 唤醒   |
| [16]    | PIN8 唤醒   |
| [17]    | PIN9 唤醒   |
| [18]    | PIN10 唤醒   |
| [19]    | PIN11 唤醒   |
| [20]    | PIN12 唤醒   |
| [21]    | PIN13 唤醒   |


* LPSYS 的 WSR 含义

| 比特域 | 含义                    |
|--------|-------------------------|
| [0]    | RTC 唤醒                |
| [1]    | GPIO2 唤醒            |
| [2]    | LPTIM2 唤醒              |
| [3]    | LPCOMP1 唤醒              |
| [4]    | LPCOMP2 唤醒            |
| [5]    | BT 唤醒  |
| [6]    | HPSYS 手动唤醒 LPSYS   |
| [7]    | HPSYS 使用 Mailbox 唤醒 LPSYS |
| [8]    | PIN0 唤醒   |
| [9]    | PIN1 唤醒   |
| [10]    | PIN2 唤醒   |
| [11]    | PIN3 唤醒   |
| [12]    | PIN4 唤醒   |
| [13]    | PIN5 唤醒   |
| [14]    | PIN6 唤醒   |
| [15]    | PIN7 唤醒   |
| [16]    | PIN8 唤醒   |
| [17]    | PIN9 唤醒   |
| [18]    | PIN10 唤醒   |
| [19]    | PIN11 唤醒   |
| [20]    | PIN12 唤醒   |
| [21]    | PIN13 唤醒   |
```
```{only} SF32LB58X
* HPSYS 的 WSR 含义

| 比特域 | 含义                    |
|--------|-------------------------|
| [0]    | RTC 唤醒                |
| [1]    | GPIO1 唤醒            |
| [2]    | LPTIM1 唤醒              |
| [3]    | LPCOMP 唤醒              |
| [6]    | LPSYS 手动唤醒 HPSYS           |
| [7]    | LPSYS 使用 Mailbox 唤醒 HPSYS  |
| [8]    | PIN0 唤醒   |
| [9]    | PIN1 唤醒   |
| [10]    | PIN2 唤醒   |
| [11]    | PIN3 唤醒   |
| [12]    | PIN4 唤醒   |
| [13]    | PIN5 唤醒   |
| [14]    | PIN6 唤醒   |
| [15]    | PIN7 唤醒   |
| [16]    | PIN8 唤醒   |
| [17]    | PIN9 唤醒   |
| [18]    | PIN10 唤醒   |
| [19]    | PIN11 唤醒   |
| [20]    | PIN12 唤醒   |
| [21]    | PIN13 唤醒   |
| [22]    | PIN14 唤醒   |
| [23]    | PIN15 唤醒   |
| [24]    | PIN16 唤醒   |
| [25]    | PIN17 唤醒   |


* LPSYS 的 WSR 含义

| 比特域 | 含义                    |
|--------|-------------------------|
| [0]    | RTC 唤醒                |
| [1]    | GPIO2 唤醒            |
| [2]    | LPTIM2 唤醒              |
| [3]    | LPCOMP1 唤醒              |
| [4]    | LPCOMP2 唤醒            |
| [5]    | BT 唤醒  |
| [6]    | HPSYS 手动唤醒 LPSYS   |
| [7]    | HPSYS 使用 Mailbox 唤醒 LPSYS |
| [8]    | PIN0 唤醒   |
| [9]    | PIN1 唤醒   |
| [10]    | PIN2 唤醒   |
| [11]    | PIN3 唤醒   |
| [12]    | PIN4 唤醒   |
| [13]    | PIN5 唤醒   |
| [14]    | PIN6 唤醒   |
| [15]    | PIN7 唤醒   |
| [16]    | PIN8 唤醒   |
| [17]    | PIN9 唤醒   |
| [18]    | PIN10 唤醒   |
| [19]    | PIN11 唤醒   |
| [20]    | PIN12 唤醒   |
| [21]    | PIN13 唤醒   |
| [22]    | PIN15 唤醒   |
| [23]    | PIN15 唤醒   |
| [24]    | PIN16 唤醒   |
| [25]    | PIN17 唤醒   |
```
```{only} SF32LB52X
* HPSYS 的 WSR 含义

| 比特域 | 含义                    |
|--------|-------------------------|
| [0]    | RTC 唤醒                |
| [1]    | GPIO1 唤醒            |
| [2]    | LPTIM1 唤醒              |
| [3]    | PMUC 唤醒              |
| [6]    | LPSYS 手动唤醒 HPSYS           |
| [7]    | LPSYS 使用 Mailbox 唤醒 HPSYS  |
| [8]    | PIN0 唤醒   |
| [9]    | PIN1 唤醒   |
| [10]    | PIN2 唤醒   |
| [11]    | PIN3 唤醒   |
| [18]    | PIN10 唤醒   |
| [19]    | PIN11 唤醒   |
| [20]    | PIN12 唤醒   |
| [21]    | PIN13 唤醒   |
| [22]    | PIN14 唤醒   |
| [23]    | PIN15 唤醒   |
| [24]    | PIN16 唤醒   |
| [25]    | PIN17 唤醒   |
| [26]    | PIN18 唤醒   |
| [27]    | PIN19 唤醒   |
| [28]    | PIN20 唤醒   |
```

```{only} SF32LB58X
表4‑4：55 系列 HPSYS 唤醒 PIN 映射表

| 唤醒 | PIN 含义 |
|------|----------|
| PIN0 | PA77     |
| PIN1 | PA78     |
| PIN2 | PA79     |
| PIN3 | PA80     |

表4‑5：55 系列 LPSYS 唤醒 PIN 映射表

| 唤醒 | PIN 含义 |
|------|----------|
| PIN0 | PB43     |
| PIN1 | PB44     |
| PIN2 | PB45     |
| PIN3 | PB46     |
| PIN4 | PB47     |
| PIN5 | PB48     |
```
```{only} SF32LB56X
表4‑4：56 系列 HPSYS 唤醒 PIN 映射表（部分）

| 唤醒 | PIN 含义 |
|------|----------|
| PIN0 | PB32     |
| PIN1 | PB33     |
| PIN2 | PB34     |
| PIN3 | PB35     |

表4‑5：56 系列 LPSYS 唤醒 PIN 映射表（部分）

| 唤醒 | PIN 含义 |
|------|----------|
| PIN0 | PB54     |
| PIN1 | PB55     |
| PIN2 | PB56     |
| PIN3 | PB57     |
| PIN4 | PB58     |
| PIN5 | PB59     |
```
```{only} SF32LB58X
表4‑4：58 系列 HPSYS 唤醒 PIN 映射表（部分）

| 唤醒 | PIN 含义 |
|------|----------|
| PIN0 | PB54     |
| PIN1 | PB55     |
| PIN2 | PB56     |
| PIN3 | PB57     |

表4‑5：58 系列 LPSYS 唤醒 PIN 映射表 （部分）

| 唤醒 | PIN 含义 |
|------|----------|
| PIN0 | PB32     |
| PIN1 | PB33     |
| PIN2 | PB34     |
| PIN3 | PB35     |
| PIN4 | PB36     |
| PIN5 | PA50     |
```

```{only} SF32LB52X
表4‑4：52 系列 HPSYS 唤醒 PIN 映射表（部分）

| 唤醒 | PIN 含义 |
|------|----------|
| PIN0 | PA24     |
| PIN1 | PA25     |
| PIN10| PA34     |
| PIN11| PA35     |
| PIN19| PA43     |
```

### 4.7 常见问题分析

由于进入睡眠模式后 SWD 无法连接，必须使用 UART 作为 console 端口抓取日志分析问题。

#### 4.7.1 是否进入睡眠模式

若满足以下任一条件，很可能 HPSYS 已进入睡眠：
- SWD 无法连接
- HCPU 的 console 无应答
- HCPU 日志出现 “S: mode, gtime”

若满足以下任一条件，很可能 LPSYS 已进入睡眠：
- LCPU 的 console 无应答
- LCPU 日志出现 “S: mode, gtime”

需确认 LCPU 已打开 Command shell 中的 finsh shell 选项。

也可通过测量芯片电源管脚电压判断当前低功耗模式：
- HPSYS 处于 active/sleep/deepsleep：`LDO1_VOUT`≈1.1V；HPSYS 处于 standby：`LDO1_VOUT` 逐渐下降至 0V。
- LPSYS 处于 active/sleep/deepsleep：`LDO2_VOUT` 或 `BUCK2_VOUT`≈0.9V；LPSYS 处于 standby：相应电压逐渐下降至 0V。
- 进入 hibernate：`LDO1_VOUT/LDO2_VOUT/BUCK2_VOUT/VDD_RET` 均下降至 0V。

```{only} SF32LB55X
55 系列低功耗模式下的电源管脚电压示意见图 4.5（来源：SF32LB55x 用户手册 4.2.9）。

```{figure} ../../assets/low_power16.png
:align: center
图 4.5 低功耗模式下的电源管脚电压
```

#### 4.7.2 为什么没有进入睡眠模式

**进入睡眠判定条件**

对应用程序而言，睡眠/工作模式的切换由最低优先级的 `IDLE` 线程透明控制：当所有高优先级线程无任务时，`IDLE` 得到调度并检查是否满足以下全部条件，方可进入睡眠：

- 没有禁止睡眠模式（不存在未释放的 `rt_pm_request(PM_SLEEP_MODE_IDLE)`）
- 最近一个将要超时的操作系统定时器时间 > 门限（默认 100ms）
- 不满足唤醒条件（例如已使能的唤醒源当前未被激活）
- 发送给小核的数据已被读走（IPC 队列无未消费数据）

**睡眠前的定时唤醒配置**

进入睡眠前，会依据“最近一个定时器”的超时值配置 `LPTIM`，使其在该时刻产生中断唤醒大核。例如：最近一个定时器 200ms 后超时，则将 `LPTIM` 配置为 200ms 后中断，从而保证睡眠期间仍能准时触发 OS 定时器回调。

**禁止/释放睡眠接口**

应用可在关键区内显式抑制睡眠，驱动框架在外设工作期间也会自动抑制，以避免中断模式下误睡：

```c
// 禁止进入睡眠，直到 release
rt_pm_request(PM_SLEEP_MODE_IDLE);
// ... 关键区/外设操作 ...
rt_pm_release(PM_SLEEP_MODE_IDLE);
```

**定时门限与策略**

用户可以参考 电源管理 中的  configuration`, 对工程打开低功耗的支持，使用不同的低功耗策略，可以进入不同的低功耗模式。系统目前默认的策咯是：

默认策略示例：

```c
static const pm_policy_t default_pm_policy[] =
{
    {15, PM_SLEEP_MODE_LIGHT},                  // 空闲超过 15ms，进入浅睡
#ifdef PM_STANDBY_ENABLE
    {10000, PM_SLEEP_MODE_STANDBY},             // 空闲超过 10s，进入待机
#endif /* PM_STANDBY_ENABLE */
};
```

常见原因检查（HCPU/LCPU 类似）：

1) 打开 PM 模块并确认宏定义：

```{only} SF32LB52X
```c
#define RT_USING_PM 1
#define BSP_USING_PM 1          // 开启低功耗模式
#define PM_DEEP_ENABLE 1       // 进入 Deep 低功耗
#define BSP_PM_DEBUG 1          // 打开低功耗调试日志
```
```{only} SF32LB56X or SF32LB58X or SF32LB55X
```c
#define RT_USING_PM 1
#define BSP_USING_PM 1          // 开启低功耗模式
#define PM_STANDBY_ENABLE 1     // 进入 standby 低功耗
#define BSP_PM_DEBUG 1          // 打开低功耗调试日志
```
2) 确认 CPU 已空闲并进入 idle 线程：
- 通过 finsh 命令 `list_thread` 查看线程状态；除 `tshell`、`tidle` 为 ready 外，其他应为 suspend，否则 IDLE 线程无法运行。

```{figure} ../../assets/low_power17.png
:align: center
图 4.6 list_thread 命令返回的信息
```

3) 确认未禁止进入睡眠：
- 在 console 执行 `pm_dump`；若 “Idle Mode 的 Counter”>0，说明有模块 `rt_pm_request(PM_SLEEP_MODE_IDLE)` 禁止了睡眠，需对应 `rt_pm_release(PM_SLEEP_MODE_IDLE)` 解除。

```{figure} ../../assets/low_power18.png
:align: center
图 4.7 pm_dump 命令返回的信息
```

4) 确认 OS 定时器超时时间大于睡眠门限：
在 console 中发送命令list_timer，显示操作系统的所有已创建的定时器，将 flag 为 activated 定时器的 timeout 值与睡眠门限作比较，若小于睡眠门限，则表示因为该定时器导致无法进入睡眠。操作系统定时器timeout 的单位为 ms。
```{figure} ../../assets/low_power19.png
:align: center
图 4.8 pm_dump 命令返回的信息
```
见如下配置,HPSYS 的睡眠门限默认为100ms，LPSYS 的睡眠门限为 10ms
```c
RT_WEAK const pm_policy_t pm_policy[] =
{
#ifdef PM_STANDBY_ENABLE
#ifdef SOC_BF0_HCPU
    {100, PM_SLEEP_MODE_STANDBY}, //Hcpu 当100ms之内没有定时器唤醒,就进入standby休眠
#else
    {10, PM_SLEEP_MODE_STANDBY}, //Lcpu 当10ms之内没有定时器唤醒,就进入standby休眠
#endif /* SOC_BF0_HCPU */
#elif defined(PM_DEEP_ENABLE)
#ifdef SOC_BF0_HCPU
    {100, PM_SLEEP_MODE_DEEP}, //Hcpu 当100ms之内没有定时器唤醒,就进入Deep休眠
#else
    {10, PM_SLEEP_MODE_DEEP}, //Lcpu 当10ms之内没有定时器唤醒,就进入Deep休眠
#endif /* SOC_BF0_HCPU */
#else
#ifdef SOC_BF0_HCPU
    {100, PM_SLEEP_MODE_LIGHT},
#else
```
若 HCPU 代码中存在 90ms 周期延时，将导致永不睡眠：

```c
while (1)
{
    rt_thread_delay(90); // 90ms delay
}
```

注意延时函数区别：
- HAL 层（延时期间不会切换线程）：

```c
HAL_Delay(10);     // 10ms
HAL_Delay_us(10);  // 10us
```

- RTT 接口（会切到其他线程，可能触发 idle→休眠）：

```c
rt_thread_delay(100); // 100ms
```

5) 确认不存在未处理的唤醒源：
- 通过命令读取寄存器核对 `WER/WSR`：
```{only} SF32LB55X
55 系列WSR地址：
```text
regop unlock 0000
regop read 4007001c 1   # LPSYS WSR
regop read 4003001c 1   # HPSYS WSR
```
```{only} SF32LB52X
52 系列WSR地址：
```text
regop unlock 0000
regop read 40040024 1   # LPSYS WSR
regop read 500c0024 1   # HPSYS WSR
```
```{only} SF32LB56X
56 系列WSR地址：
```text
regop unlock 0000
regop read 50040020 1   # LPSYS WSR
regop read 40040020 1   # HPSYS WSR
```
```{only} SF32LB58X
58 系列WSR地址：
```text
regop unlock 0000
regop read 50040020 1   # LPSYS WSR
regop read 40040020 1   # HPSYS WSR
```

- 也可用 Jlink/SifliUsartServer 读取寄存器或通过日志打印：

```c
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_hpsys_aon->WSR, hwp_hpsys_aon->WER); // hcpu
rt_kprintf("wsr:0x%x,wer:0x%x,\n", hwp_lpsys_aon->WSR, hwp_lpsys_aon->WER); // lcpu
```

常见问题：唤醒 pin 电平配置不当（例如低电平唤醒但电平一直为低）。

6) 确认发给另一核的数据已被读走：
- 可通过 Ozone 连接、dump 内存用 trace32 查看，或日志打印 `ipc_ctx` 队列的 `read_idx_mirror/write_idx_mirror`；不相等表示有未取走数据，会阻断睡眠。

```{figure} ../../assets/low_power20.png
:align: center
图 4.9 非空环形缓冲的情形
```

```{figure} ../../assets/low_power21.png
:align: center
图 4.10 空环形缓冲的情形
```

示例打印激活队列索引：

```c
for (i = 0; i < IPC_LOGICAL_QUEUE_NUM; i++)
{
    if (ipc_ctx.queues[i].active == true)
    {
        if (ipc_ctx.queues[i].rx_ring_buffer && ipc_ctx.queues[i].tx_ring_buffer)
        {
            LOG_I("ipc_ctx.queues[%d].tx read_idx_mirror=0x%x, write_idx_mirror=0x%x\n",
                  i,
                  ipc_ctx.queues[i].tx_ring_buffer->read_idx_mirror,
                  ipc_ctx.queues[i].tx_ring_buffer->write_idx_mirror);
        }
    }
}
```

```{figure} ../../assets/low_power22.png
:align: center
图 4.11 LCPU 未开启 data service 导致通道缺失示例
```


## 5 功耗优化方法

### 5.1 待机漏电分析

当 HPSYS 与 LPSYS 均已进入睡眠，整机功耗优化重点：
- 屏、传感器、充电 IC 等可拆器件先拆，测最小系统电流；
- 软件 IO 电平配置不当导致电压差/浮空漏电；
- 芯片内部 PSRAM/Flash 与外部 NAND/Flash/eMMC 未进入休眠。

若硬件可分路测电流，可定位是 `VSYS/ VLDO2/ VLDO3/ VDD_SIP/ VDDIOA` 哪路漏电以收敛范围。

#### 5.1.1 外设漏电

常见原因：
1) 板级器件未断电；
2) 板级器件已掉电，但芯片管脚设置不当导致由芯片管脚向板级器件倒灌。

针对 2)，应避免：连接掉电器件的芯片管脚输出高或使能上拉。常用外设在工作/睡眠下的推荐配置如下（外部电路不掉电时无需改变管脚，外部电路掉电时需切换为低功耗配置）。

表 5‑1：管脚推荐设置

| 外设      | 管脚           | 方向 | 工作状态   | 睡眠（外部电路不掉电） | 睡眠（外部电路掉电）      |
|-----------|----------------|------|------------|-------------------------|----------------------------|
| PSRAM     | PSRAM_CLK      | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| PSRAM     | PSRAM_CLKB     | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| PSRAM     | PSRAM_CS       | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| PSRAM     | PSRAM_DM0      | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| PSRAM     | PSRAM_DM1      | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| PSRAM     | PSRAM_DQS0     | I/O  | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| PSRAM     | PSRAM_DQS1     | I/O  | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| PSRAM     | PSRAM_DQx      | I/O  | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| QSPI      | QSPIx_CLK      | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| QSPI      | QSPIx_CS       | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| QSPI      | QSPIx_DIO0     | I/O  | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| QSPI      | QSPIx_DIO1     | I/O  | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| QSPI      | QSPIx_DIO2     | I/O  | 数字输入上拉 | 数字输入上拉          | 数字输入下拉               |
| QSPI      | QSPIx_DIO3     | I/O  | 数字输入上拉 | 数字输入上拉          | 数字输入下拉               |
| QSPI      | QSPIx_DIO4     | I/O  | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| QSPI      | QSPIx_DIO5     | I/O  | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| QSPI      | QSPIx_DIO6     | I/O  | 数字输入上拉 | 数字输入上拉          | 数字输入下拉               |
| QSPI      | QSPIx_DIO7     | I/O  | 数字输入上拉 | 数字输入上拉          | 数字输入下拉               |
| USART     | USARTx_RXD     | I    | 数字输入上拉 | 数字输入上拉          | 数字输入下拉               |
| USART     | USARTx_TXD     | O    | 数字输出   | 数字输出                | 数字输出                   |
| USART     | USARTx_CTS     | I    | 数字输入上拉 | 数字输入上拉          | 数字输入下拉               |
| USART     | USARTx_RTS     | O    | 数字输出   | 数字输出                | 数字输出                   |
| I2C       | I2Cx_SCL       | I/O  | 数字输入   | 数字输入                | 数字输入下拉               |
| I2C       | I2Cx_SDA       | I/O  | 数字输入   | 数字输入                | 数字输入下拉               |
| SPI M     | SPIx_CLK       | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| SPI M     | SPIx_CS        | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| SPI M     | SPIx_DI        | I    | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| SPI M     | SPIx_DO        | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| SPI M     | SPIx_DIO       | I/O  | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| LCDC SPI  | LCDCx_SPI_CS   | O    | 数字输出   | 数字输出                | GPIO 模式输入下拉          |
| LCDC SPI  | LCDCx_SPI_CLK  | O    | 数字输出   | 数字输出                | GPIO 模式输入下拉          |
| LCDC SPI  | LCDCx_SPI_DIO0 | I/O  | 数字输入下拉 | 数字输入下拉          | GPIO 模式输入下拉          |
| LCDC SPI  | LCDCx_SPI_DIO1 | O    | 数字输出   | 数字输出                | GPIO 模式输入下拉          |
| LCDC SPI  | LCDCx_SPI_DIO2 | O    | 数字输出   | 数字输出                | GPIO 模式输入下拉          |
| LCDC SPI  | LCDCx_SPI_DIO3 | O    | 数字输出   | 数字输出                | GPIO 模式输入下拉          |
| LCDC SPI  | LCDCx_SPI_RSTB | O    | 数字输出   | 数字输出                | GPIO 输出低                |
| LCDC SPI  | LCDCx_SPI_TE   | I    | 数字输入   | 数字输入                | GPIO 模式输入下拉          |
| SDIO      | SD_CLK         | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| SDIO      | SD_CMD         | I/O  | 数字输入上拉 | 数字输入上拉          | 数字输入下拉               |
| SDIO      | SD_DIOx        | I/O  | 数字输入上拉 | 数字输入上拉          | 数字输入下拉               |
| I2S       | I2S1_BCK       | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| I2S       | I2S1_LRCK      | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| I2S       | I2S1_SDI       | I    | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| I2S       | I2S2_BCK       | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| I2S       | I2S2_LRCK      | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| I2S       | I2S2_SDI       | I    | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| I2S       | I2S2_SDO       | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| PDM       | PDM_CLK        | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| PDM       | PDM_DATA       | I    | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| GPTIM 出  | GPTIMx_CHx     | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |
| GPTIM 入  | GPTIMx_CHx     | I    | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| GPTIM     | GPTIMx_ETR     | I    | 数字输入下拉 | 数字输入下拉          | 数字输入下拉               |
| GPIO In   | GPIO           | I    | 数字输入   | 数字输入                | GPIO 输出低 或 数字输入下拉 |
| GPIO Out  | GPIO           | O    | 数字输出   | 数字输出                | GPIO 模式输出低            |

#### 5.1.2 芯片 IO 内部漏电

常见模型（详见 FAQ 8.7/8.8）：
1) 输入管脚悬空（对端设备断电等同悬空）导致不确定电平；
2) IO 输出电平与内/外部上下拉不匹配。

下图为管脚内部结构（示意功能：DS/OE/O/IE/PE/PS 等）。

```{figure} ../../assets/low_power23.png
:align: center
图 5.1 管脚内部结构图
```
```{only} SF32LB55X
注意：55 系列 USB 的 `PA01` 口内部默认 18K 下拉，输出高或外接高电平时会产生漏电；处理参考 FAQ “55 系列 MCU 复用 USB 的 PA01/PA03 漏电风险”。
```
#### 5.1.3 芯片内外部存储芯片漏电

PSRAM 进出 Half_sleep 示例：

```c
void BSP_Power_Up(bool is_deep_sleep)
{
#ifdef SOC_BF0_HCPU
    if (!is_deep_sleep)
    {
#if defined(BSP_USING_PSRAM1)
        rt_psram_exit_low_power("psram1"); // 退出 half_sleep
#endif
    }
    // ...
}

void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)
{
#ifdef SOC_BF0_HCPU
    if (coreid == CORE_ID_HCPU)
    {
#if defined(BSP_USING_PSRAM1)
        rt_psram_enter_low_power("psram1");  // 进入 half_sleep
#endif
    }
#else
    // ...
#endif
}
```

Flash 掉电与 Deep Sleep 示例：

```c
HAL_RAM_RET_CODE_SECT(BSP_PowerDownCustom, void BSP_PowerDownCustom(int coreid, bool is_deep_sleep))
{
#ifdef SOC_BF0_HCPU
#ifdef BSP_USING_NOR_FLASH2
    HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, false, true); // 关闭 nor flash 供电

    HAL_PIN_Set(PAD_PA16, GPIO_A16, PIN_PULLDOWN, 1); // 断电后将 IO 改为下拉
    HAL_PIN_Set(PAD_PA12, GPIO_A12, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA15, GPIO_A15, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA13, GPIO_A13, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA14, GPIO_A14, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA17, GPIO_A17, PIN_PULLDOWN, 1);

    HAL_PIN_Set(PAD_PA35, GPIO_A35, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA36, GPIO_A36, PIN_PULLDOWN, 1);
#elif defined(BSP_USING_NOR_FLASH1)
    FLASH_HandleTypeDef *flash_handle;
    flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(MPI1_MEM_BASE);
    HAL_FLASH_DEEP_PWRDOWN(flash_handle); // nor flash 进入 deep sleep；IO 状态无需修改
    HAL_Delay_us(3);
#endif /* BSP_USING_NOR_FLASH2 */
#else
    { ; }
#endif
}

HAL_RAM_RET_CODE_SECT(BSP_PowerUpCustom, void BSP_PowerUpCustom(bool is_deep_sleep))
{
#ifdef SOC_BF0_HCPU
    if (!is_deep_sleep)
    {
#ifdef BSP_USING_NOR_FLASH2
        HAL_PIN_Set(PAD_PA16, MPI2_CLK,  PIN_NOPULL,   1); // 先恢复 IO 到工作态再上电
        HAL_PIN_Set(PAD_PA12, MPI2_CS,   PIN_NOPULL,   1);
        HAL_PIN_Set(PAD_PA15, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA13, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA14, MPI2_DIO2, PIN_PULLUP,   1);
        HAL_PIN_Set(PAD_PA17, MPI2_DIO3, PIN_PULLUP,   1);

        HAL_PIN_Set(PAD_PA35, GPIO_A35, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA36, GPIO_A36, PIN_PULLUP, 1);

        HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, true, true); // 打开 nor flash 供电
        BSP_Flash_hw2_init(); // 断电后需重新初始化 nor flash
#elif defined(BSP_USING_NOR_FLASH1)
        FLASH_HandleTypeDef *flash_handle;
        flash_handle = (FLASH_HandleTypeDef *)rt_flash_get_handle_by_addr(MPI1_MEM_BASE);
        HAL_FLASH_RELEASE_DPD(flash_handle); // 退出 deep sleep
        HAL_Delay_us(20); // 具体延时参见芯片手册 tRES1
#endif
    }
    else if (PM_STANDBY_BOOT == SystemPowerOnModeGet())
    {
    }
#elif defined(SOC_BF0_LCPU)
    { ; }
#endif
}
```

注意：XIP 运行在 nor flash 的情况下，操作 nor flash 休眠/唤醒的代码需放 RAM（`HAL_RAM_RET_CODE_SECT`）。

### 5.2 代码实现
```{only} SF32LB55X
管脚配置代码位于开发板的 `pinmux.c` 与 `drv_io.c`。需依据 IO 定义与硬件实现 `BSP_PIN_Init`、`BSP_Power_Up`、`BSP_IO_Power_Down` 等接口。
```
```{only} SF32LB58X or SF32LB56X or SF32LB57X or SF32LB52X
管脚配置代码位于开发板的 `pinmux.c` 与 `bsp_power.c`。需依据 IO 定义与硬件实现 `BSP_PIN_Init`、`BSP_Power_Up`、`BSP_IO_Power_Down` 等接口。
```
#### 5.2.1 工作状态管脚设置

`BSP_PIN_Init` 在冷启动与 STANDBY 唤醒后都会执行一次，可在其中设定工作态的功能/输入输出模式。例如将 `PB46` 配为 `USART3_RX`，数字输入上拉：

```c
HAL_PIN_Set(PAD_PB46, USART3_RXD, PIN_PULLUP, 0);
```

对于输出 IO，若仅配置为 `PIN_NOPULL` 且未设置 GPIO 输出，将导致“默认输入态”而悬空漏电。需设置输出电平，例如：

```c
HAL_PIN_Set(PAD_PA35, GPIO_A35, PIN_NOPULL, 1);
// 随后请配置为明确高/低电平输出，或在需要时 HAL_GPIO_DeInit 恢复输入态
```

#### 5.2.2 睡眠状态管脚设置
```{only} SF32LB55X
在 `drv_io.c`中可实现以下虚函数，用于进出睡眠时动态切换管脚设置：
```
```{only} SF32LB58X or SF32LB56X or SF32LB57X or SF32LB52X
在 `bsp_power.c`中可实现以下虚函数，用于进出睡眠时动态切换管脚设置：
```
表 5‑2：睡眠状态的管脚设置 API

| 函数名             | 说明                                       |
|--------------------|--------------------------------------------|
| BSP_IO_Power_Down  | 进入睡眠前执行                             |
| BSP_Power_Up       | 唤醒后执行（STANDBY 唤醒在 `BSP_PIN_Init` 后） |
| BSP_TP_PowerDown   | 熄屏后执行                                 |
| BSP_TP_PowerUp     | 亮屏前执行                                 |
| BSP_LCD_PowerDown  | 熄屏后执行                                 |
| BSP_LCD_PowerUp    | 亮屏前执行                                 |

如果板级器件的掉电和上电控制都伴随睡眠进行，可以在 BSP_IO_Power_Down 中将板级器件断电并修改相应的 管脚设置，反向操作则在 BSP_Power_Up 中完成。但这种方法的缺点是控制不够精细，比如熄屏后，可能过一段 时间 HPSYS 才会进入睡眠，在此之前如果仍旧给 LCD 供电的话就会增加耗电，或者当 HPSYS 被唤醒执行一段 时间任务但又不需要亮屏时，如果在 BSP_Power_Up 中就打开屏幕的供电，也会增加耗电。 为此，用户可以在 BSP_IO_Power_Down 和 BSP_Power_Up 中实现更复杂的控制逻辑。以屏幕与触控为例，将屏幕 与触控的掉电处理放在 BSP_TP_PowerDown 和 BSP_LCD_PowerDown 中，这样一旦熄屏后屏幕与触控芯片即可 马上断电，而在 BSP_Power_Up 里需要再次调用 BSP_TP_PowerDown 和 BSP_LCD_PowerDown，这样即使 HPSYS 被唤醒，也可以将管脚的设置恢复到掉电状态，如果满足了亮屏条件，系统会在亮屏前调用 BSP_TP_PowerUp 和 BSP_LCD_PowerUp 恢复屏幕与触控的供电与工作状态下的管脚设置。

调用顺序与 coreid 说明：
- `void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)` 会在 HCPU 进入睡眠前被调用两次：
    - 第一次 `coreid=CORE_ID_LCPU`：在撤销 LCPU 唤醒请求之前执行，用于关闭“由 HCPU 使用的 LCPU 外设相关管脚”。撤销后 LCPU 可能进入低功耗，HCPU 将无法再访问 LCPU 电源域寄存器。
    - 第二次 `coreid=CORE_ID_HCPU`：在 HCPU 真正入睡前执行，用于关闭 HCPU 自身使用的其他管脚。
- 在 LCPU 工程中，该函数在 LCPU 入睡前调用一次，用于关闭 LCPU 使用的管脚。
- 不同外设的低功耗管脚配置不同。一般应关闭上/下拉以避免回路漏电；管脚输出高低取决于板级设计（是否与外设上电状态匹配）。

### 5.3 休眠流程

```{only} SF32LB56X

• HCPU 休眠唤醒（简化流程）：
`rt_thread_idle_entry → rt_system_power_manager → _pm_enter_sleep → pm->ops->sleep(pm, mode) → sifli_sleep →` 日志 `[pm]S:4,11620140` → 设备 `RT_DEVICE_CTRL_SUSPEND` → `sifli_standby_handler → BSP_IO_Power_Down → WFI` 进入 standby → 定时器/IO 唤醒 → `SystemInitFromStandby → HAL_Init → BSP_IO_Init → restore_context`（PC 回到 WFI 后）→ `BSP_Power_Up →` 设备 `RT_DEVICE_CTRL_RESUME` → 日志 `[pm]W:11620520`、`[pm]WSR:0x80`。

• LCPU 休眠唤醒与 HCPU 类似，差异点：`sifli_standby_handler → sifli_standby_handler_core → BSP_IO_Power_Down → soc_power_down → WFI → SystemPowerOnModeInit → SystemPowerOnInitLCPU → HAL_Init → BSP_IO_Init → restore_context → soc_power_up → BSP_Power_Up → RT_DEVICE_CTRL_RESUME →` 日志同上。
```
```{only} SF32LB55X
• HCPU 休眠唤醒（简化流程）：
`rt_thread_idle_entry → rt_system_power_manager → _pm_enter_sleep → pm->ops->sleep(pm, mode) → sifli_sleep →` 日志 `[pm]S:4,11620140` → 设备 `RT_DEVICE_CTRL_SUSPEND` → `sifli_standby_handler → BSP_IO_Power_Down → WFI` 进入 standby → 定时器/IO 唤醒 → `SystemPowerOnModeInit → HAL_Init → BSP_IO_Init → restore_context`（PC 回到 `sifli_standby_handler` 之后）→ `BSP_Power_Up → RT_DEVICE_CTRL_RESUME →` 日志 `[pm]W:11620520`、`[pm]WSR:0x80`。

• LCPU 休眠唤醒与 HCPU 类似，差异点：`sifli_standby_handler → sifli_standby_handler_core → BSP_IO_Power_Down → soc_power_down → WFI → SystemPowerOnModeInit → SystemPowerOnInitLCPU → HAL_Init → BSP_IO_Init → restore_context → soc_power_up → BSP_Power_Up → RT_DEVICE_CTRL_RESUME →` 日志同上。
```
```{only} SF32LB52X
建议使用deepsleep低功耗模式（睡眠模式），该模式下所有RAM数据和硬件配置都能保持，从睡眠模式回到工作状态所需的恢复时间也较短，睡眠期间IO电平可以保持在工作时的状态，但睡眠模式下外设停止工作，CPU只能被有限几个唤醒源唤醒，包括GPIO中断、RTC中断、LPTIM中断以及核间通信中断。

• HCPU 休眠唤醒（简化流程）：
进入 `sifli_deep_handler()`，且无外设 SUSPEND/RESUME 与现场恢复，唤醒更快：
`sifli_sleep →` 日志 `[pm]S:3,11620140` → `sifli_deep_handler → BSP_IO_Power_Down → WFI` 进入 deep → 定时器/IO 唤醒 → 回到 WFI 后继续 → `BSP_Power_Up →` 日志 `[pm]W:11620520`、`[pm]WSR:0x80`。

注：52 系列的 LCPU 不开放代码修改。
```
### 5.4 Hibernate 关机漏电分析

#### 5.4.1 Hibernate 关机流程
```{only} SF32LB55X
进入 Hibernate：调用 `HAL_PMU_EnterHibernate()`。休眠前需配置好 Hibernate 下 PMU 的唤醒 PIN 与电平。
```
```{only} SF32LB52X
进入 Hibernate：调用 `HAL_PMU_EnterHibernate()`。休眠前需配置好 Hibernate 下 PMU 的唤醒 PIN 与电平。52 系列因内置 3 个 LDO，需结合硬件通过 `HAL_PMU_ConfigPeriLdo` 关闭无用 LDO。
```
```{only} SF32LB56X or SF32LB58X
进入 Hibernate：调用 `HAL_PMU_EnterHibernate()`。休眠前需配置好 Hibernate 下 PMU 的唤醒 PIN 与电平。56/58 系列因 Hibernate 下新增 PMU 上下拉系统，建议用 `HAL_PIN_Set` 配置唤醒 PIN 上下拉。
```
Hibernate 唤醒：按下唤醒 PIN 后唤醒。可用 `PM_HIBERNATE_BOOT == SystemPowerOnModeGet()` 判断是否为 hibernate boot，并结合按键时间判断是否开机。

#### 5.4.2 Hibernate 关机配置

进入 Hibernate 前：
- 调用 `HAL_PMU_EnterHibernate()`；
- 配置 PMU 唤醒 PIN 与电平，确保可被唤醒；
```{only} SF32LB55X
- 55 系列：Hibernate 下唤醒 PIN 均为浮空输入，为防止浮空漏电需外部上下拉；
```
```{only} SF32LB52X or SF32LB56X or SF32LB57X or SF32LB58X
- 58/56/52 系列：Hibernate 下 PMU 侧提供上下拉（`hwp_rtc->PAWK1R/PAWK2R`），建议 `HAL_PIN_Set` 配置；
- `hwp_pmuc->WKUP_CNT` 可配置外部信号持续时间门限（限 58/56/52 系列）。
```
```{only} SF32LB52X
- 52 系列：内置 3 个 LDO（`PMU_PERI_LDO_1V8/PMU_PERI_LDO2_3V3/PMU_PERI_LDO3_3V3`），视硬件考虑是否关闭；

示例：

```c
rt_kprintf("SF32LB52X entry_hibernate\n");
HAL_PMU_SelectWakeupPin(0, HAL_HPAON_QueryWakeupPin(hwp_gpio1, BSP_KEY1_PIN)); // select PA34 → wake_pin0
HAL_PMU_EnablePinWakeup(0, AON_PIN_MODE_HIGH);                                 // enable wake_pin0 
hwp_pmuc->WKUP_CNT = 0x50005; // 31-16bit: PIN1 wake CNT, 15-0bit: PIN0 wake CNT
rt_kprintf("SF32LB52X CR:0x%x,WER:0x%x\n", hwp_pmuc->CR, hwp_pmuc->WER);

HAL_PIN_Set(PAD_PA24, GPIO_A24, PIN_PULLDOWN, 1); // #WKUP_PIN0
HAL_PIN_Set(PAD_PA25, GPIO_A25, PIN_PULLDOWN, 1); // #WKUP_PIN1
HAL_PIN_Set(PAD_PA26, GPIO_A26, PIN_PULLDOWN, 1); // #WKUP_PIN2
HAL_PIN_Set(PAD_PA27, GPIO_A27, PIN_PULLDOWN, 1); // #WKUP_PIN3

HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_PULLDOWN, 1); // #WKUP_PIN10
HAL_PIN_Set(PAD_PA35, GPIO_A35, PIN_PULLDOWN, 1); // #WKUP_PIN11
HAL_PIN_Set(PAD_PA36, GPIO_A36, PIN_PULLDOWN, 1); // #WKUP_PIN12
HAL_PIN_Set(PAD_PA37, GPIO_A37, PIN_PULLDOWN, 1); // #WKUP_PIN13
HAL_PIN_Set(PAD_PA38, GPIO_A38, PIN_PULLDOWN, 1); // #WKUP_PIN14
HAL_PIN_Set(PAD_PA39, GPIO_A39, PIN_PULLDOWN, 1); // #WKUP_PIN15
HAL_PIN_Set(PAD_PA40, GPIO_A40, PIN_PULLDOWN, 1); // #WKUP_PIN16
HAL_PIN_Set(PAD_PA41, GPIO_A41, PIN_PULLDOWN, 1); // #WKUP_PIN17
HAL_PIN_Set(PAD_PA42, GPIO_A42, PIN_PULLDOWN, 1); // #WKUP_PIN18
HAL_PIN_Set(PAD_PA43, GPIO_A43, PIN_PULLDOWN, 1); // #WKUP_PIN19
HAL_PIN_Set(PAD_PA44, GPIO_A44, PIN_PULLDOWN, 1); // #WKUP_PIN20

rt_hw_interrupt_disable();
HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO2_3V3, false, false);
HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO3_3V3, false, false);
HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO_1V8,  false, false);
HAL_PMU_EnterHibernate();
```

注意：
```{only} SF32LB55X
- 55 系列 MCU：每个唤醒 pin 可单独使能，仅需 `HAL_PMU_EnablePinWakeup` 即可；
```
```{only} SF32LB52X or SF32LB56X or SF32LB57X or SF32LB58X
- 58/56/52 系列：同时仅允许 2 个唤醒源 `pin0/pin1`，需用 `HAL_PMU_SelectWakeupPin` 指定映射；
```
```{only} SF32LB52X
- 52 系列的 `#WKUP_PIN4-9 (PA28-PA33)` 与 ADC 复用，已取消唤醒口功能；处理为断开外部 IO 并内部下拉（Hibernate 下无需处理亦不会漏电），勿直接操作 `hwp_rtc->PAWK1R/PAWK2R` 上拉以免漏电。

```{figure} ../../assets/low_power24.png
:align: center
图 5.2 52 系列 Hibernate 下 #WKUP_PIN4-9 处理示意
```

Hibernate 唤醒判断：

```c
if (PM_HIBERNATE_BOOT == SystemPowerOnModeGet())
{
    // 根据按键持续时间等决定是否开机
}
```






