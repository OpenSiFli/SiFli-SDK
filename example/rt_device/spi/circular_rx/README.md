# SPI Circular DMA 示例
源码路径：`example/rt_device/spi/circular_rx`

## 支持平台
- `sf32lb52-lcd_n16r8`
- `sf32lb58-lcd_n16r64n4`
- `sf32lb56-lcd_a128r12n1`
## 概述
该例程用于验证 RT-Thread SPI 驱动中的 DMA 循环模式，支持三种模式：
- **Master TRX 模式**：全双工收发，适用于 loopback 测试
- **Slave RX 模式**：只接收，需外部 Master 提供时钟
- **Slave TX 模式**：只发送，需外部 Master 提供时钟

## API 接口

| 接口 | 说明 |
|------|------|
| `rt_device_control(..., RT_SPI_CTRL_CONFIG_DMA_CIRCULAR, ...)` | 配置循环 DMA 方向 |
| `rt_device_set_rx_indicate()` | 注册 RX 半缓冲 / 满缓冲回调 |
| `rt_device_set_tx_complete()` | 注册 TX 半缓冲 / 满缓冲回调 |
| `rt_spi_transfer()` | 启动 TX+RX 全双工循环 DMA |
| `rt_device_read()` | 启动 RX only 循环 DMA |
| `rt_device_write()` | 启动 TX only 循环 DMA |
| `rt_device_control(..., RT_SPI_CTRL_STOP_DMA_CIRCULAR, ...)` | 停止循环 DMA |

## 关键配置
`project/proj.conf` 需要包含：
```ini
CONFIG_BSP_USING_SPI1=y
CONFIG_BSP_SPI1_TX_USING_DMA=y
CONFIG_BSP_SPI1_RX_USING_DMA=y
CONFIG_BSP_USING_SPI_DMA_CIRCULAR=y

#56-LCD需要配置为SPI2
CONFIG_BSP_USING_SPI2=y
CONFIG_BSP_SPI2_TX_USING_DMA=y
CONFIG_BSP_SPI2_RX_USING_DMA=y
CONFIG_BSP_USING_SPI_DMA_CIRCULAR=y
```


## 模式选择
通过 `SPI_CIRCULAR_DEMO_MODE` 宏选择：
- `0`：Master TRX 模式（默认）
- `1`：Slave RX only 模式
- `2`：Slave TX only 模式

## 例程流程

### Master TRX 模式 (模式 0)
1. 配置 SPI1 pinmux(56为SPI2)
2. attach/open `spi_circular` 设备
3. 配置 SPI（Master、Mode0、8bit、20MHz）
4. 调用 `rt_spi_take_bus()/rt_spi_release_bus()` 触发 DMA handle 链接
5. 调用 `rt_device_control(..., RT_SPI_CTRL_CONFIG_DMA_CIRCULAR, ...)` 配置 `TXRX`
6. 调用 `rt_spi_transfer()` 启动循环 DMA
7. 短接 MOSI-MISO 进行 loopback 验证

### Slave RX 模式 (模式 1)
1. 配置 SPI1 pinmux(56为SPI2)
2. 配置 SPI 为 Slave 模式
3. 调用 `rt_device_control(..., RT_SPI_CTRL_CONFIG_DMA_CIRCULAR, ...)` 配置 `RX`
4. 调用 `rt_device_read()` 启动循环接收
5. 连接外部 SPI Master 提供时钟

### Slave TX 模式 (模式 2)
1. 配置 SPI1 pinmux(56为SPI2)
2. 配置 SPI 为 Slave 模式
3. 调用 `rt_device_control(..., RT_SPI_CTRL_CONFIG_DMA_CIRCULAR, ...)` 配置 `TX`
4. 调用 `rt_device_write()` 启动循环发送
5. 连接外部 SPI Master 提供时钟

## 硬件连接

### 引脚映射表

| 开发板 | 功能引脚 | 本端设备引脚 | 对端设备引脚 | 物理引脚（CONN2） |
|--------|----------|--------------|--------------|-------------------|
| sf32lb52-lcd | PA_24 | dio | SPI_MOSI | 19 |
|              | PA_25 | di  | SPI_MISO | 21 |
|              | PA_28 | clk | SPI_CLK  | 23 |
|              | PA_29 | cs  | SPI_CS   | 24 |
| sf32lb58-lcd | PA_21 | dio | SPI_MOSI | 8 |
|              | PA_20 | di  | SPI_MISO | 10 |
|              | PA_28 | clk | SPI_CLK  | 5 |
|              | PA_29 | cs  | SPI_CS   | 3 |
| sf32lb56-lcd | PA_64 | do  | SPI_MOSI | 19 |
|              | PA_69 | di  | SPI_MISO | 29 |
|              | PA_73 | clk | SPI_CLK  | 23 |
|              | PA_71 | cs  | SPI_CS   | 24 |

sf32lb52-lcd_n16r8硬件原理图参考如下图：

![alt text](assets/52-DevKit-lcd-V1.0.png)
### 方式一：单板回环（Master TRX 模式）
短接 SPI1 MOSI 与 MISO：
- `sf32lb52x`：`PA24(DIO)` ↔ `PA25(DI)`
- `sf32lb58x`：`PA21(DO)` ↔ `PA20(DI)`
- `sf32lb56x`：`PA64(DO)` ↔ `PA69(DI)`

### 方式二：外接 SPI 从设备（Slave 模式）
按标准 4 线 SPI 连接，需外部 Master 提供时钟。

### sf32lb52-lcd_n16r8 硬件原理图

![alt text](assets/52-DevKit-lcd-V1.0.png)

## 预期日志

### Master TRX 模式
```text
Start SPI circular DMA demo!
Mode: Master TRX (loopback)
Circular DMA started on spi1
Tip: Short MOSI(DO) to MISO(DI) for loopback verification.
RX: half=1 full=0 rx[0..7]: 00 01 02 03 04 05 06 07, rx[mid..mid+7]: 80 81 82 83 84 85 86 87, mismatch=0
```

### Slave RX 模式
```text
Start SPI circular DMA demo!
Mode: Slave RX only
Tip: Connect external SPI Master to provide clock.
Circular DMA started on spi1
RX: half=1 full=0
```

### Slave TX 模式
```text
Start SPI circular DMA demo!
Mode: Slave TX only
Tip: Connect external SPI Master to provide clock.
Circular DMA started on spi1
TX: half=1 full=0 tx[0..7]: 00 01 02 03 04 05 06 07, tx[mid..mid+7]: 80 81 82 83 84 85 86 87
```

## 回调函数
用户通过标准 RT 设备回调处理半缓冲/满缓冲事件：

```c
rt_device_set_rx_indicate((rt_device_t)g_spi_dev, spi_dma_circular_rx_ind);
rt_device_set_tx_complete((rt_device_t)g_spi_dev, spi_dma_circular_tx_ind);

static rt_err_t spi_dma_circular_rx_ind(rt_device_t dev, rt_size_t offset)
{
    // offset == 0             -> buffer 前半就绪
    // offset == buffer_size/2 -> buffer 后半就绪
    return RT_EOK;
}

static rt_err_t spi_dma_circular_tx_ind(rt_device_t dev, void *buffer)
{
    // buffer 指向 TX buffer 前半或后半
    return RT_EOK;
}
```

## 常见问题

### 1) 启动后卡住
检查：
- `rt_spi_take_bus()/rt_spi_release_bus()` 是否执行
- `proj.conf` 中 DMA 配置是否生效

### 2) half/full 不增长
- 检查 SPI pinmux 是否被覆盖
- 检查 DMA IRQ 是否使能

### 3) sample_mismatch 持续较高（Master TRX 模式）
- loopback 线接触不良
- MISO 信号干扰

## 相关代码
- 示例：`example/rt_device/spi/circular_rx/src/main.c`
- 驱动：`rtos/rtthread/bsp/sifli/drivers/drv_spi.c`
- 头文件：`rtos/rtthread/bsp/sifli/drivers/drv_spi.h`

## 更新记录
| 版本 | 日期 | 说明 |
|:---|:---|:---|
| 1.0.0 | 03/2026 | 初始版本，支持 Master TRX / Slave RX / Slave TX 三种模式 |
| 1.0.1 | 04/2026 | 添加 sf32lb56-lcd 支持 |