# DFU V2 CDC Loader 示例

源码路径：example/dfu_v2/cdc/loader

(Platform_dfu_v2_cdc_loader)=
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
- `sf32lb52-lcd_n16r8_hcpu` — SF32LB52X，16MB NOR Flash（`n16r8`）。

## 概述
<!-- 例程简介 -->
本例程是 DFU V2 在 USB-CDC 通道上的 DFU loader 子程序。它独立烧写在 `DFU_V2_LOADER` 分区，开机由 bootloader 在检测到升级标志后跳入，负责通过 USB-CDC 接收新固件并直写目标分区，可选地在屏幕上显示升级进度。

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
本 loader 通常无需单独构建：CDC 用户应用工程的 `SConstruct` 通过 `AddDFU_CDC(SIFLI_SDK)`（见 `tools/build/building.py`）把它作为子工程自动加入，构建用户应用时会一并生成 loader 固件。也可以单独构建本 loader。

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](#Platform_dfu_v2_cdc_loader)）。

### menuconfig配置
本例程的关键配置（已在 proj.conf 中默认开启）：
- `CONFIG_USING_DFU_V2=y` — 启用 DFU V2 中间件。
- `CONFIG_DFU_V2_USE_CDC_TRANSPORT=y` — 选择 USB-CDC 作为 DFU 传输通道。
- `CONFIG_PKG_USING_CHERRYUSB=y` / `CONFIG_PKG_CHERRYUSB_DEVICE=y` — 启用 CherryUSB 设备栈。
- `CONFIG_PKG_CHERRYUSB_DEVICE_MUSB_SIFLI=y` — SiFli MUSB USB 设备控制器驱动。
- `CONFIG_PKG_CHERRYUSB_DEVICE_CDC_ACM=y` — CDC ACM 设备类。
- `CONFIG_PKG_USING_LITTLEVGL2RTT=y` / `CONFIG_LVGL_V9=y` — 启用 LVGL（v9）升级界面；关闭后 loader 仍可完成升级，仅不显示 UI。
- `CONFIG_PKG_SIFLI_MBEDTLS_BOOT=y` — 引导阶段固件校验所需的 mbedTLS 支持。

### 编译和烧录
直接构建 CDC 用户应用（会自动带上本 loader），切换到用户应用 project 目录，运行 scons 命令执行编译：
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
如需单独构建本 loader，切换到本例程 project 目录，运行 scons 命令执行编译：
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
产物在对应的 `build_<board>` 目录下。关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
1. 烧入工程后，板子运行的是用户应用（屏幕显示版本号 `Ver: 1.0.9`）。
2. PC 端通过 USB-CDC 向用户应用发送 4 字节触发魔数 `0xAA 0x55 0xDF 0x00`；应用回 ACK `0xAA 0x55 0xDF 0x01`，随后调用 `dfu_enter_dfu_mode()` 写下升级标志并重启。
3. bootloader 重启后检测到升级标志，跳入本 loader（`DFU_V2_LOADER` 分区）。loader 在 `main()` 中先 `dfu_init()` 初始化中间件，再 `dfu_check_install()` 处理可能的待装任务，然后等待 USB-CDC 连接。
4. PC 工具连上后把新固件分块发给 loader。loader 通过 `dfu_process()` 接收并直写目标分区，期间 `on_dfu_event()` 回调把进度推给 LVGL 界面，屏幕显示百分比；全部完成后弹出成功提示。
5. loader 完成安装并重启，bootloader 跳回新的用户应用。屏幕上的版本号变化即可确认升级成功。

调试时可在串口控制台用 `cdc_dfu_info` 命令查看 USB 就绪状态与当前传输进度。

## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |06/2026 |初始版本 |
| | | |
| | | |
