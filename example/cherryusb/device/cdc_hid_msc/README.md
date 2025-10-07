# USB复合设备示例项目
源码路径：example\cherryusb\device\cdc_hid_msc
## 支持的开发板
- SF32LB52_LCD_N16R8

## 示例概述
本项目演示了如何在单一USB连接中同时实现三种不同的USB设备功能：
- MSC (大容量存储类): 基于RAM的虚拟U盘
- CDC_ACM (通信设备类): 提供虚拟串口通信功能
- HID (人机接口设备): 模拟USB键盘输入功能

## 功能特性

### USB复合设备功能
- 虚拟磁盘: 访问设备内部Flash存储，支持文件读写操作
- 虚拟串口: 数据回显功能，可用于调试和通信
- 虚拟键盘: 通过按键触发，向主机发送文本输入

### 按键交互功能
- KEY1: 发送 "SiFli" 字符串
- KEY2: 发送 "CherryUSB" 字符串
- 支持按键事件检测和处理



## 编译和使用

### 1  编译

* 切换到例程project目录，运行scons命令执行编译：
```
scons --board=sf32lb52-lcd_n16r8 -j8
```
* 运行`build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat`，按提示选择端口即可进行下载：

```
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

Uart Download

please input the serial port num:5
```

### 2 使用方法
1. 连接硬件: 将主机与开发板的UART口，USB口通过USB数据线进行连接
2. 烧录程序: 将编译好的固件烧录到开发板
3. 打开串口: 用串口实用工具打开USB虚拟串口以及板载物理串口。

### 3 现象
1、按下按键，串口输出如下，对应文本框输入CherryUSB和SiFli，模拟USB键盘输入功能正常。
```
button:11,0
button:11,3
KEY2 clicked, sending 'CherryUSB'
HID: Sending string: CherryUSB
button:11,1
button:34,0
button:34,3
KEY1 clicked, sending 'SiFli'
HID: Sending string: SiFli
button:34,1
```
2、打开文件管理器，发现名为SiFli MSC的磁盘，内含README.TXT。打开该文件，该文件有以下内容，基于RAM的虚拟U盘功能正常。
```
SiFli USB Composite Device Demo!
Features:
- MSC: Virtual Disk (RAM-based)
- CDC ACM: Virtual COM Port
- HID: Virtual Keyboard
```
3、从USB虚拟出来的串口发送消息会进行回显，同时物理UART将打印消息长度。
例：从USB虚拟串口发送“SiFli”，此时会在打开了物理UART所在COM口的串口助手中看到
```
CDC actual out len:5

CDC actual in len:5
```
同时，在打开了USB虚拟串口的串口助手中看到相同的文本被发送回来
```
SiFli
```
则认为CDC_ACM正常。
### 常见问题

1. 设备无法识别
   - 检查USB连接
   - 确认VID/PID配置
   - 检查USB驱动程序

2. 数据传输异常
   - 检查UART引脚配置
   - 确认波特率设置
   - 检查硬件连接

3. 编译错误
   - 检查RT-Thread配置
   - 确认CherryUSB组件已启用
   - 检查头文件路径

## 参考文档
* CDC ACM设备类: https://cherryusb.readthedocs.io/zh_CN/latest/quick_start/cdc_acm.html
* CherryUSB文档: https://cherryusb.readthedocs.io/
* CherryUSB官方仓库: https://github.com/cherry-embedded/CherryUSB
* RT-Thread官网 https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/README

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |9/2025 |初始版本 |