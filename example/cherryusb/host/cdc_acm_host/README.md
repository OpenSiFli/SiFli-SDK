# USB CDC ACM 主机示例
源码路径：example/cherryusb/host/cdc_acm_host

本示例演示了如何使用CherryUSB栈实现一个CDC ACM（Communication Device Class - Abstract Control Model）主机，用于识别和连接USB CDC ACM设备（如虚拟串口设备）。主机可以枚举CDC ACM设备，接收设备发送的数据，并支持数据的双向传输。

## 功能特性

- **CDC ACM主机类实现**：符合USB CDC ACM标准，支持作为主机连接CDC设备
- **设备自动识别**：自动枚举和识别连接的CDC ACM设备
- **数据接收功能**：支持从USB CDC设备接收数据并实时打印
- **DTR/RTS控制**：支持设置CDC设备的DTR和RTS控制信号
- **中断接收模式**：使用中断方式接收数据，提高响应速度
- **实时操作系统集成**：基于RT-Thread RTOS实现
- **CherryUSB栈**：使用轻量级、高性能的CherryUSB主机协议栈



## 硬件连接

### 支持的开发板
- **SF32LB52_LCD_N16R8**: 基于SF32LB52芯片的LCD开发板

### USB连接
- 将USB CDC ACM设备（如虚拟串口设备、Arduino等）连接到开发板的USB Host接口
- 确保开发板USB Host接口供电正常，能够为外部设备供电
- 支持USB 2.0全速模式（根据配置）

### 连接拓扑
```
[PC] <--USB--> [开发板(USB Host)] <--USB--> [CDC ACM设备]
      调试串口                              虚拟串口设备
```

### USB Host配置
本示例使用内部USB控制器作为主机模式，主要配置：
- **USB Base地址**: USBC_BASE
- **CDC设备名称**: ttyACM0
- **接收缓冲区**: 4096字节
- **DTR控制**: 使能（1）
- **RTS控制**: 禁用（0）

## 编译和使用

### 1  编译

切换到例程project目录，运行scons命令执行编译：

**编译LCD开发板版本：**
```bash
scons --board=sf32lb52-lcd_n16r8 -j8
```

### 2 下载

运行对应开发板的下载脚本，按提示选择端口即可进行下载：

**LCD开发板下载：**
```bash
build_sf32lb52-lcd_n16r8_hcpu\uart_download.bat

Uart Download
please input the serial port num:5
```

### 3 使用方法

#### 3.1 运行程序
1. 将编译生成的固件下载到开发板
2. 复位或重启开发板
3. 程序启动后会打印："cherryusb host demo!"
4. 系统初始化USB主机控制器

#### 3.2 连接CDC设备
1. 将CDC ACM设备通过USB线连接到开发板的USB Host接口
2. 等待约2秒，系统会自动枚举设备
3. 如果设备识别成功，会打印："ttyACM0 found!"
4. 如果设备未找到，会打印："ttyACM0 not found!"

#### 3.3 数据接收测试
1. **自动接收数据**：
   - 当CDC设备发送数据时，主机会自动触发接收回调
   - 接收到的数据会立即打印到调试串口
   - 每次接收会显示数据长度和内容

2. **典型输出示例**：
   ```
   cherryusb host demo!
   ttyACM0 found!
   rx callback, size:2048
   receive data: 1234567890aaaaaa...
   rx callback, size:2048
   receive data: 1234567890aaaaaa...
   ```

3. **配合设备端测试**：
   - 可以使用CDC ACM Device例程作为设备端
   - 设备端会周期性发送2048字节数据
   - 主机端会接收并打印这些数据

#### 3.4 DTR/RTS控制
- 主机会在设备打开后自动设置DTR信号为1
- RTS信号设置为0
- 这些信号会传递给CDC设备，可用于控制设备行为
- 例如CDC Device例程只有在DTR使能时才发送数据



## 技术细节

### 代码结构
- **main.c**: 主程序入口，初始化USB主机控制器并管理CDC ACM设备连接
- **usb_config.h**: CherryUSB主机配置文件，定义各种USB参数和选项

### 配置参数
- **接收缓冲区大小**: 4096字节
- **设备枚举延时**: 2000毫秒（等待设备枚举完成）
- **设备名称**: ttyACM0
- **打开标志**: RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX
- **DTR状态**: 使能（1）
- **RTS状态**: 禁用（0）

### CherryUSB主机特性
- 支持USB主机控制器初始化
- 支持CDC ACM设备自动识别和枚举
- 支持中断接收模式，数据到达时自动触发回调
- 支持DTR/RTS控制信号设置
- 集成RT-Thread设备驱动框架

### 工作流程
1. **初始化阶段**：
   - 调用`usbh_initialize()`初始化USB主机控制器
   - 配置USB基地址和控制器参数

2. **设备枚举阶段**：
   - 延时2秒等待设备插入和枚举
   - USB主机栈自动识别CDC ACM设备
   - 创建ttyACM0设备节点

3. **设备打开阶段**：
   - 使用`rt_device_find()`查找设备
   - 使用`rt_device_open()`打开设备
   - 注册接收回调函数`rx_cb()`
   - 设置DTR/RTS控制信号

4. **数据接收阶段**：
   - 当CDC设备发送数据时触发中断
   - 调用`rx_cb()`回调函数
   - 读取数据到接收缓冲区并打印

## 故障排除

### 常见问题

1. **设备无法识别（打印"ttyACM0 not found!"）**
   - 检查CDC设备是否正确连接到开发板USB Host接口
   - 确认CDC设备供电正常（部分设备需要外部供电）
   - 检查USB线缆质量和数据线完整性
   - 增加枚举延时时间（当前为2秒）
   - 确认CDC设备本身工作正常（可先连接PC测试）
   - 查看调试日志确认USB枚举过程

2. **设备识别但无数据接收**
   - 确认CDC设备是否有数据发送
   - 检查设备端DTR信号要求（部分设备需要DTR使能才发送数据）
   - 验证接收回调函数是否正确注册
   - 检查USB主机控制器初始化是否成功
   - 查看设备打开标志是否包含INT_RX

3. **数据接收不完整或乱码**
   - 检查接收缓冲区大小（当前4096字节）
   - 确认数据传输速率是否过快
   - 验证USB总线质量和信号完整性
   - 检查是否有数据溢出或丢失

4. **系统运行不稳定**
   - 检查USB供电能力是否足够
   - 确认外接设备功耗不超过USB Host供电能力
   - 验证内存使用情况，避免内存不足
   - 检查是否有栈溢出问题

5. **编译或下载失败**
   - 确认SiFli SDK环境配置正确
   - 检查工具链路径设置
   - 验证目标板型选择是否正确
   - 确认project目录结构完整

## 测试场景

### 配合CDC ACM Device例程测试
1. 准备两块开发板：一块运行CDC ACM Device例程，一块运行本Host例程
2. 将Device端通过USB连接到Host端的USB Host接口
3. Host端会自动识别Device并开始接收数据
4. 可以看到Host端持续打印Device发送的测试数据

## 参考文档
* CDC ACM主机类: https://cherryusb.readthedocs.io/zh_CN/latest/quick_start/cdc_acm.html
* CherryUSB文档: https://cherryusb.readthedocs.io/
* CherryUSB官方仓库: https://github.com/cherry-embedded/CherryUSB
* RT-Thread官网: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/README
* USB CDC规范: https://www.usb.org/document-library/class-definitions-communication-devices-12

## 开发指南

### 自定义开发
如需基于此示例进行自定义开发，可以修改以下内容：

1. **数据接收处理**：
   - 修改`rx_cb()`函数实现自定义数据处理逻辑
   - 添加数据解析、协议处理或存储功能
   - 可以将数据转发到其他模块或外设

2. **数据发送功能**：
   - 添加`rt_device_write()`调用实现数据发送
   - 可以实现双向通信功能
   - 支持命令下发和响应接收

3. **多设备支持**：
   - 查找ttyACM1、ttyACM2等设备节点
   - 支持同时连接多个CDC设备
   - 为每个设备注册独立的回调函数

4. **配置参数调整**：
   - 修改接收缓冲区大小（recv_buffer）
   - 调整枚举延时时间适应不同设备
   - 配置DTR/RTS控制信号

5. **错误处理增强**：
   - 添加设备断开检测和重连机制
   - 实现数据传输超时处理
   - 增加异常情况的日志记录

### 相关API说明

#### USB主机初始化
- `usbh_initialize(busid, reg_base)`: 初始化USB主机控制器
  - busid: 总线ID（通常为0）
  - reg_base: USB控制器基地址

#### RT-Thread设备操作
- `rt_device_find(name)`: 查找设备
  - 返回设备句柄，失败返回RT_NULL
  
- `rt_device_open(dev, flags)`: 打开设备
  - flags: RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX
  
- `rt_device_read(dev, pos, buffer, size)`: 读取设备数据
  - pos: 偏移位置（CDC设备通常为0）
  - 返回实际读取字节数
  
- `rt_device_write(dev, pos, buffer, size)`: 写入设备数据
  - 用于向CDC设备发送数据
  
- `rt_device_set_rx_indicate(dev, rx_ind)`: 设置接收指示回调
  - rx_ind: 接收回调函数指针

#### CDC ACM特定API
- `usbh_cdc_acm_set_line_state(cdc_acm, dtr, rts)`: 设置线路状态
  - dtr: DTR信号（0或1）
  - rts: RTS信号（0或1）

### 数据发送示例代码
```c
// 向CDC设备发送数据
const char *send_data = "Hello CDC Device\n";
rt_size_t len = rt_device_write(ttyACM0, 0, send_data, strlen(send_data));
if (len > 0) {
    rt_kprintf("Sent %d bytes\n", len);
}
```

## 相关例程

- **CDC ACM Device例程**: 配合使用的CDC ACM设备端例程，可用于测试主机功能

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|1.0.0 |11/2025 |完善功能描述和使用说明，添加技术细节和开发指南 |
|0.0.1 |11/2025 |初始版本，实现基本CDC ACM主机功能 |
