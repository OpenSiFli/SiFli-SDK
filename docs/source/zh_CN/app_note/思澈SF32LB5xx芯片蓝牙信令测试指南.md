# 思澈 SF32LB5xx 芯片蓝牙信令测试指南

## 1. 基本介绍

在信令测试模式下，被测样机（DUT）和综测仪建立连接后可以通过软件工具控制综测仪完成自动化测试（如使用 R&S®CMWrun 控制 CMW 综测仪），也可以通过手动控制综测仪测试 DUT 指标参数。本文重点描述被测样机（DUT）如何与综测仪建立信令连接及 TX 测试和 RX 测试，用于协助硬件工程师完成 SF32LB55x/58x/56x/52x 系列芯片的蓝牙信令测试，以综测仪 CMW500 为例。

测试需准备如下资源：

1) 硬件
   - 综测仪
   - 被测样机（DUT，在一些文档中表述为 EUT）
   - Windows PC
   - USB 转串口线
   - 射频同轴线

2) 软件
   - 串口工具（如 SiFli_Trace 工具）

BLE 没有官方定义的测试模式，所以需要串口和综测仪连接，一般 BLE 信令测试的拓扑如图 1-1。

```{figure} assets/figure_1-1.jpg
:align: center

图 1-1
```

BT 信令测试和 BLE 信令测试区别在于，BT 信令测试可以不需要串口连接综测仪，一般 BT 信令测试的拓扑如图 1-2。

```{figure} assets/figure_1-2.jpg
:align: center

图 1-2
```

---

## 2. 测试准备

### 2.1. 硬件接线

**1. DUT 测试点引线**

需引出以下测试点，用于供电与串口通信：**VBAT、GND、UART1 TX/RX**。

**2. 芯片串口引脚对应关系**

不同型号的 UART1 引脚存在差异，具体对应关系如表 1：

**表 1**

| 引脚名称 | SF32LB551 | SF32LB555&557 | SF32LB56xU | SF32LB56xV | SF32LB58x | SF32LB52x |
|---------|-----------|---------------|------------|------------|-----------|-----------|
| UART1 TX | PA17 | PA34 | PA17 | PA34 | PA32 | PA19 |
| UART1 RX | PA18 | PA30 | PA18 | PA30 | PA31 | PA18 |

> **注：** SF32LB52x 为所有 52 系列，SF32LB58x 表示所有 58 系列，SF32LB56xU 为 QFN 封装（如 SF32LB563），SF32LB56xV 为 BGA 封装（如 SF32LB567）。上述引脚为默认引脚配置，使用时应当以项目配置的具体引脚为准。

**环境搭建**：DUT 的 UART1 连接到电脑上，用于发送指令进入测试模式；RF 通过同轴线连接到综测仪上，测试 RF 性能（BLE 信令测试时，需在进入测试模式后把串口连接到综测仪上）。

### 2.2. DUT 进入 BLE 测试模式

1) 给 DUT 上电，确保 DUT 可正常开机；
2) 确保 DFU 不在休眠状态，用 USB 线连接 DUT 的 UART1 到电脑上；
3) 用串口工具（如 SiFli_Trace 工具）给 DUT 发送 Finsh 指令 `bt_cm uart_dut`；
4) 串口工具需改为"HEX"显示，则收到 `04 0E 04 XX 03 0C 00` 的回复表示 DUT 成功进入测试模式如图 2-1；
5) 在串口工具上，断开串口连接；
6) 把连接到电脑上的 USB 线切换到综测仪上（即 DUT 的 UART1 通过 USB 线连接到综测仪）。

```{figure} assets/figure_2-1.png
:align: center

图 2-1
```

### 2.3. DUT 进入 BT 测试模式

1. 给 DUT 上电，确保 DUT 可正常开机，如果 DUT 已经进入了 BLE 测试模式需重新给 DUT 下上电开机。
2. 在亮屏界面（不让 DUT 进入休眠），用 USB 线连接 DUT 的 UART1 到电脑上；
3. 用串口工具（如 SiFli_Trace 工具）给 DUT 发送 Finsh 指令 `bt_cm dut`；
4. 串口工具"字符"显示收到 `Write scan enable success` 的回复表示 DUT 成功进入测试模式，由于 User_bin 会有其他 log，具体回复如图 2-2；

```{figure} assets/figure_2-2.png
:align: center

图 2-2
```

---

## 3. 建立信令连接

### 3.1. 仪器基本设置

- 在综测仪按键区左上方找到【SIGNAL GEN】并按下此按钮，如图 3-1，可以进入到仪器的选择界面；
- 此时选择 Bluetooth signaling 如图 3-2，进入到 Bluetooth 信令设置界面；
- 进入信令设置界面可以看到 Bluetooth signaling 默认为 OFF 的状态如图 3-3；
- 此时点击【Bluetooth signaling】再点击按键区的【ON/OFF】打开测试，如图 3-4，仪器基本设置完成。

```{figure} assets/figure_3-1.jpg
:align: center

图 3-1
```

```{figure} assets/figure_3-2.jpg
:align: center

图 3-2
```

```{figure} assets/figure_3-3.jpg
:align: center

图 3-3
```

```{figure} assets/figure_3-4.jpg
:align: center

图 3-4
```

### 3.2. 建立 BLE 信令连接

BLE 信令测试的拓扑如图 1-1，和综测仪需要通过串口交互。

1) 在 Bluetooth signaling 界面点击【EUT Control】，出现对应界面后根据图 3-5 进行配置；选择 HW Interface 为 `USB to RS232 adapter`，选择 Baud Rate 为 `1000000`，Burst Type 选择 `Low Energy`，此时串口没有插入 Virtual COM port 为灰色；

```{figure} assets/figure_3-5.jpg
:align: center

图 3-5
```

2) 把 DUT 的 UART1（不同芯片型号的 UART1 参考表 1）连接到综测仪上（DUT 已进入测试模式，User_bin 进入方式参考第二章节），DUT 的射频同轴线连接到综测仪的 RF 输入口，此时重新开关 Bluetooth signaling 仪器会自动检测到串口，检测到串口后如图 3-6；

3) 仪器识别到串口后，点击左下方的【connection check】，3S 左右会有弹窗弹出，显示 `LE comm test passed` 点击 OK，则可进行自动或手动测试，若仪器连接了自动化软件则可以跑自动化测试。

```{figure} assets/figure_3-6.jpg
:align: center

图 3-6
```

### 3.3. 建立 BT 信令连接

BT 信令测试的拓扑如图 1-2，BT 信令连接综测仪不需要使用串口。

1) Bluetooth signaling 界面设置和 BLE 连接有所区别，如图 3-7，主要是把 HW Interface 选项修改为 `None (EUT Control off)`；

2) 确认 DUT 已进入信令测试模式（User_bin 进入方式参考第二章节）且射频同轴线已经连接到综测仪上，点击左下方的 Inquire 搜索蓝牙，当搜索到 DUT 的蓝牙地址时即可停止搜索点击【Connect Test Mode】连接 DUT 设备，连接成功如图 3-8。

3) 连接成功后可以进行自动化或手动测试，若仪器连接了自动化测试软件可以开始跑自动化。

```{figure} assets/figure_3-7.jpg
:align: center

图 3-7
```

```{figure} assets/figure_3-8.jpg
:align: center

图 3-8
```

---

## 4. TX 测试（BLE/BT 通用）

BLE 和 BT 建立信令连接后，手动测试 TX 操作方式一致。

1) DUT 和综测仪完成连接后，在 Bluetooth signaling 界面点击【Bluetooth 1 Multi Eval.】如图 4-1；

```{figure} assets/figure_4-1.jpg
:align: center

图 4-1
```

2) 跳转到的 TX 界面默认关闭如图 4-2；

3) 按【ON/OFF】将测试界面打开，打开 Multi Evaluation 测试后对应界面会显示 DUT 的相关测试情况如图 4-3。

4) 如果需要测试其他参数，如 BLE 测试需修改为 BLE 2M 或 BT 测试修改成 EDR 在屏幕右侧选择【Input Signal】，然后在仪器屏幕底部【Burst Type】选择对应选项即可，若需要修改包类包长等可以在 Bluetooth signaling 界面设置或选择【Input Signal】按钮后在屏幕下方选择对应选项，如图 4-4。

```{figure} assets/figure_4-2.jpg
:align: center

图 4-2
```

```{figure} assets/figure_4-3.jpg
:align: center

图 4-3
```

```{figure} assets/figure_4-4.jpg
:align: center

图 4-4
```

---

## 5. RX 测试（BLE/BT 通用）

BLE 和 BT 建立信令连接后，手动测试 TX 操作方式一致。如果是 TX 测试完成后再进行 RX 测试，先选择【Bluetooth 1 Multi Eval.】，按【ON/OFF】将 TX 测试关闭，在屏幕右侧点击【Bluetooth signaling】退回到连接界面。

1) 在 Bluetooth signaling 界面点击【Bluetooth 1 RX Meas.】如图 5-1；

2) 跳转到的 RX 界面默认关闭，如图 5-2；

3) 选择左上方的【BER】/【PER】（BT 测试对应 BER，BLE 测试对应 PER）并设置好所需要测试的通道和包类包长等，点击【ON/OFF】打开 Rx Quality，BT RX 测试配置界面如图 5-3，BLE RX 测试配置界面如图 5-4；

4) 选中界面中的【Tx Level (CMW)】缓慢减小 Tx Level 的值（可以以 0.01 或 0.1 的步进减小 Tx Level 值），当屏幕左上角 PER[%] 或 BER[%] 中显示值变红，上一个值即目前所测信道的接收灵敏度，如图 5-5；

如需测试其他频点或其他类型的接收灵敏度，在对应项中修改即可。

```{figure} assets/figure_5-1.jpg
:align: center

图 5-1
```

```{figure} assets/figure_5-2.jpg
:align: center

图 5-2
```

```{figure} assets/figure_5-3.jpg
:align: center

图 5-3
```

```{figure} assets/figure_5-4.jpg
:align: center

图 5-4
```

```{figure} assets/figure_5-5.jpg
:align: center

图 5-5
```
