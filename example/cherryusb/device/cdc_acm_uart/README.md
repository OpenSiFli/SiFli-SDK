# cdc_acm_uart

一个基于CherryUSB的USB CDC ACM到UART桥接驱动程序，可以将USB虚拟串口数据透明地转发到物理UART设备。

## 功能特性

- USB CDC ACM设备: 实现标准的USB通信设备类，在PC端显示为虚拟串口
- 双向数据桥接: USB ↔ UART数据透明传输
- 动态参数配置: 支持运行时修改UART参数（波特率、数据位、停止位、奇偶校验）
- 高性能传输: 使用DMA/中断接收

## 硬件连接
### 支持的开发板
- SF32LB52_LCD_N16R8： UART2 使用 PA03(RX), PA04(TX)
- SF32LB58_LCD_N16R64N4: UART2 使用 PA29(RX), PA28(TX)

## 编译和使用

### 1  编译

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

### 2 使用方法

1. 连接硬件: 将配置的UART2引脚RX、TX与USB转串口模块的TX、RX正确连接，将模块连接到电脑的USB接口上；将开发板上标有USB和UART的Type-C接口分别连接到电脑的USB接口上，UART接口用于下载和调试，而USB则用于后续的例程运行。
2. 烧录程序: 将编译好的固件烧录到开发板，烧录步骤见“编译和使用”。
3. 打开串口: 为测试UART2串口到USB虚拟串口的透明收发，主机需要运行两个串口调试助手，各打开“USB串行设备”和USB转串口模块所在COM口，配置好相同的波特率后，就可以互相收发消息了。此时开发板相当于一个USB转串口模块。

## 故障排除

### 常见问题

1. 设备无法识别
   - 检查USB连接
   - 确认VID/PID配置
   - 检查USB驱动程序

2. 数据传输异常
   - 检查UART引脚配置
   - 确认波特率设置
   - 检查硬件连接

## 参考文档
* CDC ACM设备类: https://cherryusb.readthedocs.io/zh_CN/latest/quick_start/cdc_acm.html
* CherryUSB文档: https://cherryusb.readthedocs.io/
* CherryUSB官方仓库: https://github.com/cherry-embedded/CherryUSB
* RT-Thread官网 https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/README

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |9/2025 |初始版本 |