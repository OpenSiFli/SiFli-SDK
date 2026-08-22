# USB UVC + UAC Host 示例

[English](README_EN.md) | 简体中文

源码路径：example/cherryusb/host/uvc_uac_host

## 概述
本示例演示如何基于 CherryUSB 实现 USB Host 端 UVC 视频 + UAC 音频的一体化采集能力：

- UVC 视频流采集（YUYV/MJPEG）
- UAC 麦克风音频流采集
- 通过 finsh 命令动态启动/停止视频与音频流
- 串口打印帧地址、帧长度以及视频帧率信息

默认启动后仅完成 USB Host 初始化与设备枚举，视频流与音频流通过命令触发。
本例程既支持同一设备的音视频联合测试，也支持视频与音频分开测试。

## 硬件要求
- 支持 USB Host 的 SiFli 开发板（例如 sf32lb52）
- 支持数据传输的 USB 线缆
- UVC 摄像头（用于视频测试）
- UAC 音频输入设备（用于音频测试，可与摄像头分开）
- 串口调试工具

## 用法

### 支持的开发板
此示例可在以下开发板上运行：
- sf32lb52-lcd_n16r8

### 编译和烧录
切换到例程 `project` 目录，运行 scons 命令执行编译：

> scons --board=sf32lb52-lcd_n16r8 -j8

切换到例程 `project/build_xx` 目录，运行 `uart_download.bat`，按提示选择端口进行下载：

> ./uart_download.bat

> Uart Download

> please input the serial port num:5

关于编译、下载的详细步骤，请参考[快速上手](quick_start)的相关介绍。

### 运行步骤
1. 将固件烧录到开发板并打开串口终端（波特率通常为 1000000）
2. 复位开发板，确认看到 `cherryusb host demo!`
3. 将 USB 设备连接到开发板 USB Host 接口并等待枚举完成：
	- 可选 A：带麦克风的 UVC 摄像头（联合测试）
	- 可选 B：普通 UVC 摄像头 + 独立 UAC 设备（分开测试）

4. 启动视频流：
	- `usbh_uvc_start yuyv`：启动 YUYV 视频流（320x240）
	- `usbh_uvc_start mjpeg`：启动 MJPEG 视频流（320x240）
	
5. 启动音频流：
	- `usbh_uac_start <freq>`：按采样率启动麦克风流（如 `usbh_uac_start 16000`）
	- `usbh_uac_volume <volume> <is_tx>`：设置音量（可选）

6. 停止时可分别执行：
	- `usbh_uac_stop`：停止麦克风流
	- `usbh_uvc_stop`：停止视频流

### 调试说明
- 视频线程会持续打印视频帧缓冲区地址与帧长度
- 音频线程会持续打印音频帧缓冲区地址与帧长度
- 帧率统计线程每 5 秒打印一次当前 `fps`

## 示例输出
示例运行成功时，串口可见如下日志（不同摄像头日志会有差异）：
```
cherryusb host demo!
...
msh />usbh_uvc_start mjpeg
frame buf:0x2006d5ac,frame len:6534
frame buf:0x200755ac,frame len:6408
...
fps:30
vc:300

msh />usbh_uac_start 16000
frame buf:0x2007d5ac,frame len:384
frame buf:0x2007d72c,frame len:384

msh />usbh_uac_stop
msh />usbh_uvc_stop
```

## 异常诊断
- 执行 `usbh_uvc_start` 无输出：请先确认摄像头已成功枚举并支持 UVC。
- 视频帧长度长期为 0：请尝试切换流格式（`yuyv`/`mjpeg`）或更换摄像头。
- 执行 `usbh_uac_start` 失败：请确认当前连接的是支持 UAC 输入的音频设备。
- 仅有视频无音频（或反之）：联合测试时请检查设备是否同时暴露 UVC 和 UAC 接口；分开测试时请分别确认对应设备已枚举成功。

如有问题，请在 GitHub 提交 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [SiFli-SDK 开发指南](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/development/index.html)
- [CherryUSB 官方文档](https://cherryusb.readthedocs.io/)
