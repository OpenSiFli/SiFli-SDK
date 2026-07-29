# SDADC_HAL示例
源码路径：example/hal/adc/sdadc

## 概述
* 使用 HAL API 直接操作 SDADC，实现单通道电压采集。

## 支持的平台
例程可以运行在以下开发板。
+ sf32lb58 HDK

> **注意**：SDADC IP 仅在 SF32LB55X 与 SF32LB58X 芯片上可用，该例程基于58调试。

## 硬件连接
SDADC 通道与引脚的对应关系：

| 芯片 | CH1 | CH2 | CH3 | CH4 |
|------|-----|-----|-----|-----|
| SF32LB55X | PB_23 | PB_24 | PB_25 | PB_26 |
| SF32LB58X | PB_40 | PB_41 | PB_42 | PB_43 |

> CH0 仅用于 BGA 封装，普通 GPIO 不可用。

将目标 SDADC 引脚连接到外部电压源（0 ~ 3.3V），**GND 必须与板子共地**。

## 例程的使用
### 编译和烧录
* 此例程使用 HAL 层 API，需要确保 `rtconfig.h` 中启用了 SDADC 模块：

```c
#define BSP_USING_SDADC 1
```

* 切换到例程 project 目录，运行 scons 编译（以 ec-lb587 为例）：

```
scons --board=ec-lb587_hcpu -j8
```

* 运行 `build_xxx_hcpu\uart_download.bat`，按提示选择端口下载。

#### 例程输出结果展示
以 CH3（58x: PB_42 ）为例，每秒循环打印寄存器值和换算电压：

```
======== SDADC HAL Example ========
Chip: SF32LB58X
Test channel: 3 (CH3)
Calibrate: sdadc_cal <reg1> <mV1> <reg2> <mV2>
SDADC factory calib not found, using defaults
  offset=961912, ratio=7068 (×1000000)
SDADC channel 3 pinmux configured (gain=1/4)
SDADC Init done
Waiting 2s for VREF to stabilize...
msh >
SDADC channel 3 configured, ready
Reading SDADC every 1 second...
SDADC ch3: reg=1055680 (min=1041502 max=1062947), voltage=662 mV
```

* 浮空状态下约 600~700 mV（SDADC 高阻抗输入，浮空值受环境影响，属正常现象）
* 接入已知电压源后读数应接近实际电压
* 寄存器值 `reg` 为 24-bit 原始输出，`voltage` 为换算后的 mV 值

### 校准
如果芯片没做sdadc校准，可以发送以下命令执行两点校准：

```sh
sdadc_cal <reg1> <mV1> <reg2> <mV2>
```

例如，接入 1.2V 和 2.6V 电压源，记下两次的 reg 值，串口发送：

```sh
sdadc_cal 1083587 1200 1268771 2600
```

校准值重启后失效。


## 异常诊断
* log 显示 `Invalid SDADC channel`
  * `SDADC_TEST_CHANNEL` 选择了无效通道
* log 显示 `SDADC poll timeout!`
  * SDADC 转换未完成，检查引脚配置与参考电压是否正常
* 读数异常或与实际电压偏差较大
  * 无校准值导致数据异常，执行 `sdadc_cal` 手动校准

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |07/2026 |初始版本 |
