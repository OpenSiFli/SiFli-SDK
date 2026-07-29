# DFU V2 PAN Loader 示例

源码路径：example/dfu_v2/bt_pan/loader

(Platform_dfu_v2_bt_pan_loader)=
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
本 loader 自身不单独维护分区表，跟随同通道的 PAN 用户应用板级工程一起编译。当前 PAN 用户应用工程覆盖以下 SF32LB52X（NOR flash）板子：

- `sf32lb52-lcd_n16r8_hcpu`
- `sf32lb52-lchspi-ulp_hcpu`
- `sf32lb52-nano_a128r16_hcpu`

## 概述
<!-- 例程简介 -->
本例程是 DFU V2 在 Bluetooth PAN 通道下的 DFU loader 子程序。它通过手机共享的 PAN 网络拉取新固件，并直接写入目标分区，完成后重启回新的用户应用。

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
本 loader 通常不需要单独编译。同通道的 PAN 用户应用工程在 `SConstruct` 中通过 `AddDFU_PAN_V2(SIFLI_SDK)` 把本 loader 作为子工程自动拉入，编译应用时会一并生成 loader 固件。常规做法是进入 PAN 用户应用工程目录构建，也可以进入本 loader 工程目录单独构建。

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](#Platform_dfu_v2_bt_pan_loader)）。
+ 一部提供 PAN 网络的手机。

### menuconfig配置
本例程的关键配置（已在 proj.conf 中默认开启）：

- 宏开关：`CONFIG_USING_DFU_V2`
    - 作用：启用 DFU V2 中间件。
- 宏开关：`CONFIG_DFU_V2_USE_DEVICE_MODE`
    - 作用：以设备（被升级端）模式运行。
- 宏开关：`CONFIG_DFU_V2_USE_PAN_TRANSPORT`
    - 作用：选用 PAN 作为固件传输通道。
- 宏开关：`CONFIG_BLUETOOTH`
    - 作用：使能蓝牙功能。
- 宏开关：`CONFIG_CFG_PAN`
    - 作用：使能 PAN profile。
- 宏开关：`CONFIG_CFG_HID`
    - 作用：使能 HID profile。
- 宏开关：`CONFIG_BT_AUTO_CONNECT_LAST_DEVICE`
    - 作用：自动回连上次连接的设备。
- 宏开关：`CONFIG_RT_USING_LWIP`
    - 作用：使能 LWIP 网络协议栈，经 PAN 网络拉取固件。
- 宏开关：`CONFIG_LWIP_ALTCP_TLS`
    - 作用：使能 LWIP ALTCP TLS 支持。
- 宏开关：`CONFIG_PKG_USING_MBEDTLS`
    - 作用：使能 mbedTLS 加密库。
- 宏开关：`CONFIG_PKG_USING_WEBCLIENT`
    - 作用：使能 webclient，经 PAN 网络下载固件。
- 宏开关：`CONFIG_PKG_USING_LITTLEVGL2RTT`
    - 作用：使能 LVGL 升级界面。
- 宏开关：`CONFIG_LVGL_V9`
    - 作用：使用 LVGL V9 绘制升级界面。

### 编译和烧录
切换到 PAN 用户应用工程目录，运行 scons 命令执行编译，编译应用会自动包含本 loader：
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
也可以切换到本 loader 工程目录单独构建：
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后：
1. 设置本机名 `sifli-pan`，等待手机连接。
2. 手机完成连接并连上 PAN 网络后，自动触发升级流程 `execute_ota_update()`：先对 `ota.sifli.com` 做 DNS 校验 → 从 flash 的 `DFU_FWINFO_BASE_ADDR` 读取由用户应用预先写入的固件信息 → 调用 `dfu_download()` 阻塞下载。
3. 下载成功后清除 flash 中的固件信息，界面显示进度，再重启回新的用户应用。

调试可用 finsh 命令 `ota_cmd`，子命令包括：`del_bond`、`conn_pan`、`download`、`print`、`clear`、`test_flags`、`test_dns`。

## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |06/2026 |初始版本 |
| | | |
| | | |
