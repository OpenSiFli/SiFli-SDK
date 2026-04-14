# USB 通信设备示例（CDC ACM）
源码路径：example/cherryusb/device/cdc_acm

本示例演示了如何使用CherryUSB栈实现一个CDC ACM（Communication Device Class - Abstract Control Model）设备，将USB接口模拟为串口设备。设备可以被主机识别为虚拟串口（COM口），支持数据的双向传输。

## 功能特性

- **CDC ACM设备类实现**：符合USB CDC ACM标准，支持虚拟串口通信
- **数据传输功能**：支持USB到UART的数据双向传输
- **DTR（Data Terminal Ready）控制**：支持DTR信号检测和控制
- **高速传输**：支持高速USB传输，最大包长度可达512字节（高速模式）
- **自动数据发送**：周期性发送测试数据，演示数据传输功能
- **实时操作系统集成**：基于RT-Thread RTOS实现
- **CherryUSB栈**：使用轻量级、高性能的CherryUSB协议栈



## 硬件连接

### 支持的开发板
- **SF32LB52_LCD_N16R8**: 基于SF32LB52芯片的LCD开发板

### USB连接
- 使用USB线连接开发板的USB接口到PC
- 确保USB接口供电正常
- 支持USB 2.0全速模式和高速模式（根据配置）

### 引脚配置
本示例使用内部USB控制器，主要涉及以下配置：
- **USB VID**: 0x38f4（SiFli Technologies）
- **USB PID**: 0xFFFF
- **端点配置**:
  - CDC数据输入端点: 0x85
  - CDC数据输出端点: 0x02  
  - CDC中断端点: 0x86

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

#### 2.1 运行程序
1. 将编译生成的固件下载到开发板
2. 复位或重启开发板
3. 程序启动后会打印："cherryusb device cdc acm demo!"

#### 2.2 设备识别
1. 用USB线连接开发板到PC
2. PC会识别出一个新的串口设备（COM口）
3. 设备信息：
   - 制造商：CherryUSB
   - 产品名称：CherryUSB CDC DEMO
   - 序列号：2022123456

#### 2.3 数据传输测试
1. **自动数据发送**：
   - 程序每秒自动发送2048字节测试数据（当DTR使能时）
   - 数据内容：前10字节为"1234567890"，后续为字母'a'填充

2. **串口工具连接**：
   - 使用串口调试工具连接虚拟COM口
   - 任意波特率（CDC ACM不依赖波特率设置）
   - 8位数据位，1位停止位，无校验

3. **数据接收**：
   - 可以接收从PC串口工具发送的数据
   - 接收到的数据会通过USB LOG输出实际长度

#### 2.4 DTR控制
- 设备支持DTR（Data Terminal Ready）信号控制
- 只有当DTR信号使能时，设备才会自动发送数据
- 大多数串口工具连接时会自动使能DTR



## 技术细节

### 代码结构
- **main.c**: 主程序入口，初始化CDC ACM设备并启动数据发送循环
- **cdc_acm_template.c**: CDC ACM设备实现，包含USB描述符、端点配置和数据处理
- **usb_config.h**: CherryUSB配置文件，定义各种USB参数和选项

### 配置参数
- **缓冲区大小**: 2048字节（读写缓冲区）
- **最大包长度**: 64字节（全速）/ 512字节（高速）
- **USB配置**: 总线供电，最大功耗100mA
- **端点类型**: 批量传输端点 + 中断端点

### CherryUSB特性
- 支持高级描述符注册API
- 支持USB 2.0设备质量描述符
- 支持零长度包（ZLP）处理
- 支持端点状态管理和流控

## 故障排除

### 常见问题

1. **设备无法识别**
   - 检查USB线缆和连接状态
   - 确认VID/PID配置（0x38f4/0xFFFF）
   - 检查Windows设备管理器是否有未知设备
   - 尝试重新插拔USB连接

2. **虚拟串口无法使用**
   - 确认设备已正确识别为CDC设备
   - 检查是否安装了正确的CDC ACM驱动
   - Windows 10/11通常自动安装驱动

3. **数据传输异常**
   - 检查DTR信号状态（使用串口工具查看）
   - 确认串口工具正确连接和配置
   - 检查USB日志输出以确认数据收发

4. **编译或下载失败**
   - 确认SiFli SDK环境配置正确
   - 检查工具链路径设置
   - 验证目标板型选择是否正确

## 参考文档
* CDC ACM设备类: https://cherryusb.readthedocs.io/zh_CN/latest/quick_start/cdc_acm.html
* CherryUSB文档: https://cherryusb.readthedocs.io/
* CherryUSB官方仓库: https://github.com/cherry-embedded/CherryUSB
* RT-Thread官网 https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/README

## 开发指南

### 自定义开发
如需基于此示例进行自定义开发，可以修改以下内容：

1. **USB描述符自定义**：
   - 修改VID/PID（需要合法的USB ID）
   - 更改设备字符串描述符
   - 调整端点配置

2. **数据处理逻辑**：
   - 修改`usbd_cdc_acm_bulk_out()`函数处理接收数据
   - 修改`cdc_acm_data_send_with_dtr_test()`函数自定义发送数据
   - 添加数据格式转换或协议处理

3. **配置参数调整**：
   - 修改缓冲区大小适应应用需求
   - 调整发送频率和数据量
   - 配置USB传输模式（全速/高速）

### 相关API说明
- `cdc_acm_init()`: 初始化CDC ACM设备
- `usbd_ep_start_read()`: 启动端点数据读取
- `usbd_ep_start_write()`: 启动端点数据写入
- `usbd_cdc_acm_set_dtr()`: DTR状态回调函数

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|1.0.0 |11/2025 |完善功能描述和使用说明，添加技术细节 |
|0.0.1 |9/2025 |初始版本 |
