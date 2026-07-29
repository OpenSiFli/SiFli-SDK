# USB MTP 设备示例（DFS + RAM DISK）

源码路径：example/cherryusb/device/mtp

## 支持的平台
+ sf32lb52-lcd_n16r8_hcpu
+ sf32lb58-lcd_n16r32n1_dpi_hcpu

## 概述
本例程演示基于 CherryUSB 的 MTP 设备功能，文件系统侧使用 RT-Thread DFS 与 POSIX API，
在内存中创建 64 KiB 的 RAM Block Device（`mtp_ram`），格式化为 `elm` 文件系统后提供给 MTP 访问。

连接到 PC 后，设备会以 `CherryUSB MTP DEMO` 的形式出现，可通过文件管理器进行浏览、拷贝、删除等操作。
默认会在根目录创建示例文件 `cherryusb.txt`。

## 例程的使用

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板。
+ 带数据传输功能的 USB 线（建议使用可稳定传输数据的 Type-C 线）。
+ 一台支持 MTP 的主机（Windows/Linux/macOS）。

### menuconfig配置
本例程依赖以下关键配置：
+ `CONFIG_PKG_CHERRYUSB_DEVICE=y`
+ `CONFIG_PKG_CHERRYUSB_DEVICE_MTP=y`
+ `CONFIG_RT_USING_DFS_ELMFAT=y`

如需调整对象数量、路径长度、线程栈等参数，可参考：
+ `src/usb_config.h`

### 编译和烧录
切换到例程 `project` 目录，运行 scons 命令执行编译：

> scons --board=sf32lb58-lcd_n16r32n1_dpi_hcpu -j8

切换到例程 `project/build_xx` 目录，运行 `uart_download.bat`，按提示选择串口进行下载：

> .\uart_download.bat

关于编译、下载的详细步骤，请参考[快速上手](quick_start)的相关介绍。

## 例程的预期结果
例程启动并连接主机后，预期现象如下：
+ 主机识别到一个 MTP 设备（产品名为 `CherryUSB MTP DEMO`）。
+ 打开设备后可看到示例文件 `cherryusb.txt`。
+ 可对文件执行读写、删除、重命名等基础操作。

## 异常诊断
若主机侧无法正常枚举或文件操作失败，可优先检查：
+ 是否启用了 `CONFIG_RT_USING_DFS_ELMFAT`，未启用时通常会出现 `File system (elm) was not found`。
+ USB 数据线是否支持数据传输（仅充电线会导致设备不可见）。
+ 板端串口日志中是否出现 MTP/DFS 相关报错。

如有问题，请在 GitHub 上提交 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档
+ [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
+ [SiFli-SDK 开发指南](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |06/2026 |初始版本，补充 MTP(DFS+RAM Disk) 例程说明 |
