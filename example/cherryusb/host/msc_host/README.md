# USB MSC Host 示例

[English](README_EN.md) | 简体中文

源码路径：example/cherryusb/host/msc_host

## 概述
本示例演示如何使用CherryUSB协议栈实现USB Host大容量存储设备（MSC）功能。该例程可以识别U盘等USB存储设备，并进行文件读写操作。

主要功能：
- 初始化USB Host控制器
- 检测并挂载USB存储设备
- 读取USB设备上的README.TXT文件并打印内容
- 在USB设备上创建并写入WRITE.TXT文件

## 硬件要求
- 支持USB Host功能的SiFli开发板（如sf32lb52）
- USB线缆
- U盘或其他USB大容量存储设备

**注意**：建议配合`example/cherryusb/device/msc/ram_disk`例程一起使用。该例程可以模拟一个USB存储设备，并在其中预置README.TXT文件，方便测试USB Host的文件读取功能。

## 用法

### 支持的开发板
此示例可在以下开发板上运行：
- sf32lb52-lcd_n16r8

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：

> scons --board=sf32lb52-lcd_n16r8 -j8

切换到例程`project/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：

 >./uart_download.bat

>Uart Download

>please input the serial port num:5

关于编译、下载的详细步骤，请参考[快速上手](quick_start)的相关介绍。

### 运行步骤

**方式一：使用真实U盘**
1. 在U盘根目录下创建一个名为`README.TXT`的文本文件（可选）
2. 将编译好的固件烧录到开发板
3. 连接串口终端（波特率通常为1000000）
4. 复位开发板
5. 程序启动后会等待USB存储设备的插入
6. 将U盘插入开发板的USB Host接口
7. 设备插入后，程序将自动继续运行并执行文件读写操作
8. 观察串口输出

**方式二：配合MSC Device例程测试（推荐）**
1. 准备两块开发板
2. 一块板子烧录本例程（MSC Host）
3. 另一块板子烧录`example/cherryusb/device/msc/ram_disk`例程（MSC Device）
4. 先启动Host板，程序将等待USB存储设备插入
5. 使用USB线将两块板子连接（Device板的USB Device口连接到Host板的USB Host口）
6. 然后启动Device板
7. Host板会自动识别Device板模拟的存储设备，并继续执行读取其中的README.TXT文件等操作

### 调试功能
除了自动运行的示例代码外，您还可以通过finsh控制台手动操作USB存储设备上的文件系统：

- `ls` - 列出当前目录下的文件和文件夹
- `cat <文件名>` - 显示文件内容
- `echo <内容> > <文件名>` - 将内容写入文件
- `mkdir <目录名>` - 创建新目录
- `cd <目录名>` - 切换目录
- `rm <文件名>` - 删除文件

示例操作：
```bash
msh />ls                          # 查看根目录
msh />cd /                        # 进入根目录
msh />cat README.TXT              # 查看文件内容
msh />echo Hello USB > test.txt  # 创建并写入文件
msh />mkdir newfolder             # 创建新文件夹
```

## 示例输出
如果示例运行成功，您将在串口看到以下输出：
```
cherryusb host demo!
Waiting for USB mass storage device connection...
msh />udisk: /dev/sda mount successfully
File content:
cherryusb device msc_ram demo!
```

## 异常诊断
暂无特定异常诊断信息。如有问题，请在GitHub上提出[issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK 开发指南](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)
