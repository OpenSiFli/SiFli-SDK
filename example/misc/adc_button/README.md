# GPADC 按键

源码路径：example/misc/adc_button

## 支持的平台

* sf32lb57-spi-hdk_n16r4

## 概述

GPADC 按键通过电阻分压网络，把多个按键的状态转换成不同的电压值，再用一路 ADC 采样识别具体是哪个按键被按下。相比一键一个 GPIO 的传统做法，它能显著节省引脚：1 路 ADC 通道 + 1 个用于唤醒的 GPIO 中断就能支持多个按键，适合引脚资源紧张的场景。

本例程演示如何使用 SDK 的 button 库驱动 GPADC 按键，按下按键后在串口打印对应的按键编号和动作类型。

## 原理说明

### 硬件原理

多个按键各自串联一个不同阻值的电阻后并接到同一路 ADC 引脚。按下不同按键时，分压比不同，ADC 引脚上会得到唯一且可区分的电压：

```
按键 1 按下 → ADC 电压 = VCC × R1 / (R_pullup + R1)
按键 2 按下 → ADC 电压 = VCC × R2 / (R_pullup + R2)
...
```

只要电阻值设计得当，让各按键对应的电压区间互不重叠，就能通过 ADC 采样值反查按键编号。

![硬件原理图](assets/adc.png)

### 软件识别流程

1. 任意按键按下 → 共用的 GPIO 产生边沿中断
2. button 库在软定时器（`timer` 线程）中做消抖
3. 消抖通过后，把该引脚临时切换为模拟功能，读一次 ADC
4. 用采样电压逐个比对各按键的配置区间，命中即得到按键索引
5. 读完把引脚切回 GPIO，重新使能中断，然后调用应用注册的回调

电压比对的代码逻辑（`middleware/button/button.c`）：

```c
/* ADC 读回来的原始值单位是 0.1mV，先转成 mV */
read_arg.value /= 10;
for (i = 0; i < adc_btn_group_cfg->num; i++)
{
    if ((read_arg.value >= (adc_btn_cfg[i].voltage - adc_btn_cfg[i].volt_range))
            && (read_arg.value <= (adc_btn_cfg[i].voltage + adc_btn_cfg[i].volt_range)))
    {
        break;      /* 命中，i 即按键索引 */
    }
}
```

所以配置项里的 `VOLT` 和 `RANGE` 单位都是 **mV**，判定条件是 `VOLT - RANGE ≤ 实测电压 ≤ VOLT + RANGE`。

## 例程的使用

### menuconfig 配置

```
sdk.py menuconfig --board=sf32lb57-spi-hdk_n16r4_hcpu
```

长按判定时长由 `BUTTON_ADV_ACTION_CHECK_DELAY` 控制（单位 ms），在 `SiFli Middleware -> Enable button library` 下配置。本例程设为 1000，即按住超过 1 秒判定为长按。

![menuconfig 配置](assets/menuconfig_action.png)


### 编译和烧录

切换到例程 project 目录，执行编译：

```
scons --board=sf32lb57-spi-hdk_n16r4_hcpu -j8
```

运行 `build_sf32lb57-spi-hdk_n16r4_hcpu\uart_download.bat`，按提示选择端口下载：

```
build_sf32lb57-spi-hdk_n16r4_hcpu\uart_download.bat
Uart Download
please input the serial port num: 5
```

详细说明见[编译烧录文档](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/build.html)。


### 适配新硬件

换板子时按以下步骤标定电压：

1. 从原理图算出或用万用表实测每个按键按下时 ADC 引脚的电压
2. 取区间中间值作为 `BUTTONx_VOLT`
3. 取区间半宽作为 `BUTTONx_RANGE`，确保相邻按键的 `[VOLT-RANGE, VOLT+RANGE]` 不重叠
4. 按实际数量设置 `GROUP1_MAX_NUM`，按实际接线设置 `GROUP1_ADC_DEV_CHANNEL` 和 `BSP_KEY1_PIN`

如果不确定实测值，可以先烧一版进去，日志里 `adc control origin data ... Voltage ...` 会打印每次采样的原始值（单位 0.1mV，除以 10 得 mV），据此反推配置。

## 例程输出结果展示

启动后依次短按按键 1、2、3，再长按按键 2：

```
ADC button ready, 3 keys. Press a key to see the log.
msh />
adc control origin data 3712, Voltage 30865
key 1 pressed
key 1 clicked
key 1 released
adc control origin data 3311, Voltage 26582
key 2 pressed
key 2 clicked
key 2 released
adc control origin data 2914, Voltage 22342
key 3 pressed
key 3 clicked
key 3 released
adc control origin data 3309, Voltage 26561
key 2 pressed
key 2 long pressed
key 2 released
```

日志解读：

* `adc control origin data` 是 ADC 原始码值，`Voltage` 单位是 0.1mV
* 按键 1：30865 × 0.1mV = 3086mV，落在 2973 ± 150 = [2823, 3123] 内 → 索引 0，打印 `key 1`
* 按键 2：26582 × 0.1mV = 2658mV，落在 2578 ± 150 = [2428, 2728] 内 → 索引 1，打印 `key 2`
* 按键 3：22342 × 0.1mV = 2234mV，落在 2185 ± 150 = [2035, 2335] 内 → 索引 2，打印 `key 3`
* 每次按下只在 `BUTTON_PRESSED` 前采样一次 ADC，后续的 clicked / long pressed / released 复用同一个按键索引，所以不会重复打印电压
* 短按序列是 `pressed → clicked → released`；长按序列是 `pressed → long pressed → released`

## 故障排除

| 问题 | 可能原因 | 解决方法 |
|----|----|----|
| 启动即断言 `button_bind_adc_button` 处 `s_adc_dev` 为空 | `BSP_USING_ADC1` 未使能，`bat1` 设备未注册 | 在 `proj.conf` 中加 `CONFIG_BSP_USING_ADC1=y` |
| `button_init failed` | `BSP_KEY1_PIN` 引脚号非法，或该引脚已被其它 button 占用 | 检查板级 `CONFIG_BSP_KEY1_PIN` 配置 |
| 按键无响应 | 中断 GPIO 配错，或 `USING_ADC_BUTTON` 未使能 | 核对 `BSP_KEY1_PIN` 与实际接线，确认 `CONFIG_USING_ADC_BUTTON=y` |
| 打印 `Unknown pin:...` | 实测电压不在任何按键的配置区间内 | 按日志里的 `Voltage` 值重新标定 `BUTTONx_VOLT` / `RANGE` |
| 按键识别成相邻按键 | 各按键电压区间重叠，或 `RANGE` 设得过大 | 减小 `RANGE`，必要时调整硬件分压电阻拉开差异 |
| 长按不触发 | 长按阈值过大 | 减小 `BUTTON_ADV_ACTION_CHECK_DELAY` |
| ADC 通道读数异常 | `GROUP1_ADC_DEV_CHANNEL` 与实际接线不符 | 核对原理图上按键接入的 ADC 通道 |

## 更新记录

| 版本 | 日期 | 发布说明 |
|:---|:---|:---|
| 0.0.2 | 07/2026 | 适配 sf32lb57-spi-hdk_n16r4，重写例程与文档 |
| 0.0.1 | 08/2025 | 初始版本 |
