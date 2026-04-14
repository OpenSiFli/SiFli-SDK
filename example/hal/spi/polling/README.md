# SPI Polling 示例
源码路径：`example/hal/spi/polling`

## 支持平台
- `sf32lb52-lcd_n16r8`
- `sf32lb58-lcd_n16r64n4`

## 概述
该例程演示 SPI HAL 在**阻塞轮询模式**下的自发自收（loopback）例程

核心行为：
- SPI1 主机模式、2Lines、8bit、默认 MODE0。
- 周期调用 `HAL_SPI_TransmitReceive()`。
- 每轮发送 16 字节递增 pattern（`seed + i`），并统计收发不一致个数 `mismatch`。

## 编译和下载
以 `sf32lb52-lcd_n16r8` 为例，在 `project` 目录执行：
```sh
scons --board=sf32lb52-lcd_n16r8 -j8
```

或 `sf32lb58-lcd_n16r64n4`：
```sh
scons --board=sf32lb58-lcd_n16r64n4 -j8
```

编译后进入对应 `build_*` 目录运行下载脚本。

## 硬件连接
### 单板回环（推荐）
将 SPI1 MOSI 与 MISO 短接：
- `sf32lb52`：`PA24(DIO)` ↔ `PA25(DI)`
- `sf32lb58`：`PA21(DO)` ↔ `PA20(DI)`

并保持：
- `CLK`: `PA28`
- `CS`:  `PA29`

说明：
- 此例程用 `HAL_SPI_TransmitReceive` 一次完成收发，不再手动 `TAKE/RELEASE CS`。
- 没有回环短接时，`mismatch` 往往较大或数据无规律。

## 预期日志
启动日志示例：
```text
Start spi polling loopback demo!
tip: short SPI1 MOSI(DIO/DO) to MISO(DI) for loopback verification.
```

运行日志示例：
```text
round=0 ret=0 mismatch=0 tx: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f rx: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
round=1 ret=0 mismatch=0 tx: 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 rx: 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10
```

字段说明：
- `ret`：HAL 返回值，`0` 表示 `HAL_OK`。
- `mismatch`：本轮 16 字节中 TX 与 RX 不一致个数。
- `round`：轮询计数。

## 关键配置流程
1. 配置 SPI1 引脚复用：`DIO/DO`、`DI`、`CLK`、`CS`。
2. 使能 SPI1 时钟：`HAL_RCC_EnableModule(RCC_MOD_SPI1)`。
3. 初始化 SPI 参数：
   - `Direction = SPI_DIRECTION_2LINES`
   - `Mode = SPI_MODE_MASTER`
   - `DataSize = SPI_DATASIZE_8BIT`
   - `CLKPhase/CLKPolarity` 由 `SPI_MODE` 选择
4. 进入循环执行 `HAL_SPI_TransmitReceive()` 并打印比较结果。

## 异常诊断
### 1) `ret != 0`
- 检查 SPI1 引脚复用是否正确。
- 检查 SPI 时钟与电源初始化是否完成。

### 2) `mismatch` 长期不为 0
- 优先检查 MOSI↔MISO 短接是否可靠。
- 检查线序是否接反、接触不良。
- 适当降低 SPI 时钟再观察（便于排除信号质量问题）。

### 3) 无日志刷新
- 确认程序确实运行到 `spi_test()`。
- 确认串口打印配置正常。

## 参考文档
- `EH-SF32LB52X_Pin_config_V1.3.0_20231110.xlsx`
- `DS0052-SF32LB52x-芯片技术规格书 V0p3.pdf`

## 更新记录
| 版本 | 日期 | 说明 |
|:---|:---|:---|
| 0.1.0 | 03/2026 | 改为 polling loopback 自发自收示例 |
