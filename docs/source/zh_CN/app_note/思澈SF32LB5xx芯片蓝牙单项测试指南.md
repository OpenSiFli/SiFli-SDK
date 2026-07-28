# 思澈 SF32LB5xx 芯片蓝牙单项测试指南

## 1. 基本介绍

本文主要目的是协助工程师完成 SF32LB5xx 芯片蓝牙 RF 单项测试。测试需准备如下的资源：

1) 硬件
   - 综测仪/频谱仪
   - 被测样机（DUT，在一些文档中表述为 EUT）
   - Windows PC（电脑）
   - USB Type-C 数据线（**注意**：不能是只有充电功能的 Type-C 线）
   - 射频同轴线

2) 软件
   - [SiFli_RfTool](../tools/SiFli_RfTool/SiFli_RfTool_UM.md) 工具

测试拓扑如图 1-1 所示，DUT 和电脑通过串口连接，RF 通过同轴线和综测仪相连。

```{figure} assert/figure_11-1.png
:align: center

图 1-1
```

---

````{only} SF32LB55X

## 2. 测试准备

### 2.1. 硬件接线

**1. DUT 测试点引线**

需引出以下测试点，用于供电与串口通信：**VBAT、GND、UART1 TX/RX、UART3 TX/RX**。

**2. 芯片串口引脚对应关系**

不同型号芯片的 UART1 引脚存在差异，UART3 引脚配置一致，具体对应关系如下：

| 引脚名称 | SF32LB551 | SF32LB555 & SF32LB557 |
|---------|-----------|------------------------|
| UART1 TX | PA49 | PA17 |
| UART1 RX | PA51 | PA19 |
| UART3 TX | PB45 | PB45 |
| UART3 RX | PB46 | PB46 |

**环境搭建**：DUT 的 UART1 连接到电脑上，RF 通过同轴线连接到综测仪上，可参考图 1-1。上述引脚为默认引脚配置，使用时应当以项目配置的具体引脚为准。

### 2.2. DUT 进入测试模式

1. 给 DUT 上电，确保 DUT 可正常开机；
2. 确保 DFU 不在休眠状态，用 USB 线连接 DUT 的 UART1 到电脑上；
3. 用串口工具（如 SiFli_Trace 工具）给 DUT 发送 Finsh 指令 `bt_cm uart_dut`；
4. 串口工具收到 `04 0E 04 XX 03 0C 00` 的回复表示 DUT 成功进入测试模式；
5. 在串口工具上，断开串口连接；
6. PC 通过 [SiFli_RfTool](../tools/SiFli_RfTool/SiFli_RfTool_UM.md) 工具和 DUT 交互（HCI 命令）进行测试。

## 3. 调制信号测试

### 3.1. TX 测试

在 [SiFli_RfTool](../tools/SiFli_RfTool/SiFli_RfTool_UM.md) 工具上，如图 3-1：

- 选择对应的芯片类型（LB55X）
- 选择与 UART1 口对应的 COM 口，波特率设成 1000000
- 点击连接。连接成功后，状态指示灯会变为绿色

```{figure} assert/figure_12-1.png
:align: center

图 3-1
```

- 在 Frequency、Data Length、payload、PHY 中设置测试所需的信道、PHY 类型等，如图 3-2 红色框所示：

```{figure} assert/figure_12-2.png
:align: center

图 3-2
```

- 点击 Start TX，Start TX 变成 Stop TX，且右边的指示灯变成绿色，则表示发射成功。如图 3-3 所示，发送成功后可在综测仪上看到对应参数。

```{figure} assert/figure_12-3.png
:align: center

图 3-3
```

- 如果要测试其他信道、PHY 类型等，须先点击 Stop TX，指示灯变成灰色后表示断开，如图 3-4：

```{figure} assert/figure_12-4.png
:align: center

图 3-4
```

### 3.2. RX 测试

> **注意**：测试完 TX 后，一定要先点击 Stop TX，才能进行测试 RX。

- 选择 RX，设置 Frequency，PHY 类型
- 点击 Start RX，Start RX 变成 Stop RX，且指示灯变成绿色，表示进入对应 RX 状态，如图 3-5。

```{figure} assert/figure_12-5.png
:align: center

图 3-5
```

- 如果测试其他信道，需先点击 Stop RX，右侧指示灯变成灰色，停止当前 RX 测试状态，再去设置，如图 3-6。

```{figure} assert/figure_12-6.png
:align: center

图 3-6
```

## 4. 单载波信号测试

> **注意**：单载波（CW）测试时需要更换串口，DUT 和电脑之间的连接串口需要从 UART1 改为 UART3（PB45，PB46）。

- 在 [SiFli_RfTool](../tools/SiFli_RfTool/SiFli_RfTool_UM.md) 工具上，选择 UART3 对应的 COM 端口，波特率设成 1000000
- 点击连接，指示灯变成绿色，如图 4-1。

```{figure} assert/figure_12-7.png
:align: center

图 4-1
```

- 设置对应频率和功率等级
- 点击 Start TX，右侧指示灯变成绿色，如图 4-2。4s 左右后，可在综测仪上看对应测试值。（切换其他频率测试时，先点击 Stop TX，再修改频率和功率等级，修改完频率和功率后，再点击 Start TX 即可。）

```{figure} assert/figure_12-8.png
:align: center

图 4-2
```

````

````{only} SF32LB52X or SF32LB56X or SF32LB57X or SF32LB58X

## 2. 测试准备

### 2.1. 硬件接线

**1. DUT 测试点引线**

引出以下测试点，用于供电与串口通信：**VBAT、GND、UART1 TX/RX**。

**2. 芯片串口对应关系**

不同芯片型号的管脚有所差异，具体对应关系如下：

| 引脚名称 | SF32LB56xU | SF32LB56xV | SF32LB58x | SF32LB52x |
|---------|------------|------------|-----------|-----------|
| UART1 TX | PA17 | PA34 | PA32 | PA19 |
| UART1 RX | PA18 | PA30 | PA31 | PA18 |

> **注**：SF32LB52x 为所有 52 系列，SF32LB58x 表示所有 58 系列，SF32LB56xU 为 QFN 封装（如 SF32LB563），SF32LB56xV 为 BGA 封装（如 SF32LB567）。

**环境搭建**：DUT 的 UART1 连接到电脑上，RF 通过同轴线连接到综测仪上，可参考图 1-1。

### 2.2. DUT 进入测试模式

1. 给 DUT 上电，确保 DUT 可正常开机；
2. 在亮屏界面（不让 DUT 进入休眠），用 USB 线连接 DUT 的 UART1 到电脑上；
3. 用串口工具（如 SiFli_Trace 工具）给 DUT 发送 Finsh 指令 `bt_cm uart_dut`；
4. 串口工具收到 `04 0E 04 XX 03 0C 00` 的回复表示 DUT 成功进入测试模式；
5. 在串口工具上，断开串口连接；
6. PC 通过 [SiFli_RfTool](../tools/SiFli_RfTool/SiFli_RfTool_UM.md) 工具和 DUT 交互（HCI 命令）进行测试。

## 3. 调制信号测试

### 3.1. TX 测试

#### 3.1.1. BLE 测试方法

在 [SiFli_RfTool](../tools/SiFli_RfTool/SiFli_RfTool_UM.md) 工具上，如图 3-1：

- 选择对应的芯片型号（图中选择的是 52x 系列）
- 选择 BLE 非信令以及 UART1 对应的 COM 端口，波特率设成 1000000
- 点击连接。连接成功后，状态指示灯会变为绿色

```{figure} assert/figure_13-1.jpg
:align: center

图 3-1
```

- 在 Frequency、Data Length、payload、PHY 中设置测试所需的信道、PHY 类型等
- 点击 Start TX，按钮右侧灰色点变成绿色点，则表示发射成功，可在综测仪上看到对应参数，如图 3-2
- 如果要测试其他信道、PHY 类型等，须先点击 Stop TX，更改设置后再点击 Start TX。

```{figure} assert/figure_13-2.png
:align: center

图 3-2
```

#### 3.1.2. BT 测试方法

在 [SiFli_RfTool](../tools/SiFli_RfTool/SiFli_RfTool_UM.md) 工具上，如图 3-3：

- 选择对应的芯片型号（图中选择的是 52x 系列）
- 选择 BT 非信令，以及 UART1 对应的 COM 端口，波特率设成 1000000
- 点击连接。连接成功后，状态指示灯会变为绿色，如图 3-3

> **注**：如果直接从 BLE 非信令测试转到 BT 非信令测试，只需要先点击"Stop TX"，再选择切换测试模式为 BT 非信令即可，串口连接无需再次连接。

```{figure} assert/figure_13-3.png
:align: center

图 3-3
```

- 选择 Frequency，设置 Data Length 以及 Packet Type
- 点击 Start TX，如按钮右侧灰色点变成绿色点，则表示发射成功，此时可在综测仪上看到对应指标参数，如图 3-4
- 如果要测试其他信道、Packet 类型等，须先点击 Stop TX，更改设置后再点击 Start TX。

```{figure} assert/figure_13-4.png
:align: center

图 3-4
```

### 3.2. RX 测试

- 测试完 TX 后，一定要先点击 Stop 停止 TX 测试，才能测试 RX。
- 设置 Frequency 类型等
- 点击 Start RX，如按钮右侧灰色点变成绿色点，则表示进入对应 RX 状态，如图 3-5。

```{figure} assert/figure_13-5.png
:align: center

图 3-5
```

- 测试其他信道需要先点击 Stop RX，停止当前 RX 测试状态，再修改设置。

### 3.3. 实测参考数据（LB52x HDK）

下面是思澈 LB52x HDK 上 SiFli_RfTool 设置不同 Power level 时的实测发射功率，供调试参考。实际项目因板级匹配、PA 工艺差异会有偏差，请以自家板子为准。

测试条件：Cable loss = 0.3 dB

#### 3.3.1. BLE TX power（Frequency = 2440 MHz）

| Level setting | BLE 1M (dBm) | BLE 2M (dBm) |
|:---:|:---:|:---:|
| 13 | 13.46 | 13.44 |
| 10 | 9.76 | 9.75 |
| 8 | 7.66 | 7.66 |
| 6 | 6.57 | 6.58 |
| 3 | 2.79 | 2.80 |
| 0 | 0.26 | 0.27 |

```{figure} assert/figure_23-2.png
:align: center

图 3-6 BLE TX power 实测曲线
```

#### 3.3.2. Classic BT TX power（Frequency = 2441 MHz）

| Level setting | BR DH1 (dBm) | EDR 3DH1 (dBm) |
|:---:|:---:|:---:|
| 13 | 12.52 | 12.47 |
| 10 | 9.56 | 9.67 |
| 8 | 7.65 | — |
| 6 | 6.64 | 6.46 |
| 3 | 3.10 | 3.74 |
| 0 | 0.96 | 0.93 |

> EDR 不支持 8 dBm 档位。

```{figure} assert/figure_24-2.png
:align: center

图 3-7 Classic BT TX power 实测曲线
```

## 4. 单载波信号测试

- 切换测试项的时候要先点击 Stop TX，再选择测试项测试
- 设置对应频率和功率等级
- 点击 Start TX，按钮右侧灰色点变成绿色点，表示发送成功，可在综测仪上看对应指标参数，如图 4-1。

```{figure} assert/figure_13-6.png
:align: center

图 4-1
```

- 切换其他信道测试时，先点击 Stop TX，修改设置后再点击 Start TX 即可。

````

---

## 5. 综测仪配置

本章是"工具 + 综测仪"配对使用：DUT 通过 [SiFli_RfTool](../tools/SiFli_RfTool/SiFli_RfTool_UM.md) 发射或接收 RF 信号，综测仪侧根据测试方向（DUT 发 / DUT 收）配置对应模式。

DUT 可通过同轴线测试传导或者空口测试耦合数据，耦合数据一般会比传导低，需做补偿。

### 5.1. TX 测试综测仪配置

完成 §3-§4 的 SiFli_RfTool 端 TX 配置后，DUT 已按设定参数发射 RF 信号（以 LB56X / BT 非信令 / ch0 / DH1 / PRBS9 / 0 dBm 为例，如图 5-1）。

```{figure} assert/figure_14-2.png
:align: center

图 5-1 SiFli_RfTool 端 TX 配置示例
```

综测仪侧据此配置非信令接收：在 Multi Evaluation Configuration 界面把 "Scenario" 选项设为 "StandAlone (Non Signaling)"，并把信道、Packet type、Payload 等接收参数与 SiFli_RfTool 端保持一致，如图 5-2。

```{figure} assert/figure_14-1.jpg
:align: center

图 5-2 综测仪 Non-Signaling 配置界面
```

### 5.2. RX 测试综测仪配置

DUT 接收能力测试方向相反——综测仪发射已知信号，DUT 接收并统计 PER/BER。BLE 与 Classic BT 的仪表配置思路不同，分别说明。

#### 5.2.1. BLE Rx

CMW500 切到 RX Measurement 界面，参数如图 5-3：

| 参数 | 设定值 |
|---|---|
| Standard | LE |
| Operating Mode | Direct Test Mode |
| PHY | 1 Mbps |
| Channel | 19 |
| Frequency | 2440.0 MHz |
| Tx Level (CMW) | -67.00 dBm |
| Packet Type | RF PHY TestRef |
| Payload Length | 37 byte(s) |
| Pattern Type | PRBS9 |
| **Repeat Select** | **Single shot**（关键：必须设此选项才能正确统计） |

```{figure} assert/figure_26-1.png
:align: center

图 5-3 CMW500 BLE Rx 仪表配置
```

操作步骤：

1. 在 SiFli_RfTool 上点击 **Start RX**，DUT 进入 ch19 BLE 接收
2. 在 CMW500 上把 PER 按钮置 ON，仪表开始发射 1500 个标定包
3. 等待约 5 秒，期间 SiFli_RfTool 工具显示已收到的包数和 RSSI
4. 在 SiFli_RfTool 上点击 **Stop RX**，工具显示 Total rx n packets
5. 计算 PER：**PER = (n − 1500) / 1500 × 100%**（n 为 DUT 实际收到的包数）
6. 同时查看工具上的 RSSI 值（理论值约 -67 dBm，对应 CMW Tx Level）

```{figure} assert/figure_26-3.png
:align: center

图 5-4 BLE Rx 测试结果显示（RSSI、收包数）
```

#### 5.2.2. Classic BT Rx

Classic BT 没有 SIG DTM 标准模式，CMW500 通过 GPRF Generator 加载预制波形文件发射。波形文件如 `DH1_UAP00.wv` 等，仪表配置如图 5-5。

```{figure} assert/figure_27-1.png
:align: center

图 5-5 GPRF Generator 仪表配置（Classic BT 波形发射）
```

操作步骤：

1. 在 CMW500 上选择 Classic BT 波形文件（DH1 / 2-DH1 / 3-DH1 等）
2. 在 SiFli_RfTool 上点击 **Start RX**，DUT 进入对应 BT 接收
3. 在 CMW500 上把 GPRF Generator 切 OFF → ON 启动发射，等待约 5 秒
4. 在 SiFli_RfTool 上点击 **Stop RX**，工具显示 RSSI、收到包数、误包数、误包率、收到 bit 数、误 bit 数、误比特率（参考图 5-6）

```{figure} assert/figure_27-2.png
:align: center

图 5-6 Classic BT Rx 测试结果
```

---

## 6. 频谱仪配置

若没有综测仪也可以使用频谱仪观测 DUT 的部分指标参数，频谱仪频率需根据 [SiFli_RfTool](../tools/SiFli_RfTool/SiFli_RfTool_UM.md) 发送的频率进行配置中心频率，DUT 通过同轴线连接到频谱仪上就可以看到 DUT 的输出波形，如图 6-1。

```{figure} assert/figure_15-1.png
:align: center

图 6-1
```

---

## 7. 异常处理

实际操作中如出现工具响应异常、数据明显错误、连接断开等情况，按以下步骤复位：

1. 关闭 SiFli_RfTool 工具
2. DUT 断电
3. 重新给 DUT 上电，等待开机完成
4. 重新打开 SiFli_RfTool 工具
5. 按本指南章节顺序重新操作
