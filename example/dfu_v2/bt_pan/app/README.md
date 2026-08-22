# DFU V2 PAN 用户应用示例

源码路径：example/dfu_v2/bt_pan/app

(Platform_dfu_v2_bt_pan_app)=
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
- sf32lb52-lcd_n16r8_hcpu
- sf32lb52-lchspi-ulp_hcpu
- sf32lb52-nano_a128r16_hcpu

## 概述
<!-- 例程简介 -->
本例程演示 DFU V2 的 PAN（蓝牙 PAN 网络 / HTTP）升级通道的用户应用一侧：设备通过蓝牙 PAN 接入网络后，主动向 OTA 服务器查询版本，并把待升级固件信息写入 flash，随后重启交给 loader 子程序完成实际下载与安装。

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. 烧录运行后，应用会初始化蓝牙协议栈，把本地名设为 `sifli_pan`（或 `BT_DEVICE_NAME`），并开启 HID/PAN 自动重连。请先用手机与设备配对，让设备通过手机的 PAN 网络共享上网；HID 连上后应用会自动发起 PAN 连接，串口日志出现 `PAN connected` 即表示网络通道就绪。
2. 通过串口 finsh 控制台执行升级流程：
    1) `ota_version` — 查看当前固件版本。
    2) `ota_check` — 向 OTA 服务器（`https://ota.sifli.com`）注册设备并查询最新版本。若服务器返回的版本号比当前新，固件清单会被逐项解析并写入 flash。
    3) `ota_print` — 可选，打印已写入 flash 的固件信息（名称、URL、地址、大小、CRC32 等）。
    4) `ota_go` — 校验 flash 中确有待升级条目后写下升级标志，延时 2 秒后重启。
3. 辅助命令：`ota_clear` 清除 flash 中的固件信息；`pan_cmd` 提供 `del_bond`、`conn_pan`、`autoconnect` 等 PAN 调试子命令。

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](#Platform_dfu_v2_bt_pan_app)）。
+ 一部可提供 PAN 网络共享的手机。

### menuconfig配置
本例程的关键配置（已在 proj.conf 中默认开启）：
- `CONFIG_USING_DFU_V2=y` — 启用 DFU V2 中间件，提供 `dfu_fwinfo_*` 等固件信息 API。
- `CONFIG_RT_USING_BLUETOOTH=y`、`CONFIG_BLUETOOTH=y` — 启用蓝牙协议栈。
- `CONFIG_CFG_PAN=y`、`CONFIG_CFG_HID=y` — 启用 PAN 与 HID 配置，PAN 提供网络通道，HID 用于触发 PAN 连接。
- `CONFIG_BT_PROFILE_CUSTOMIZE=y`、`CONFIG_BT_AUTO_CONNECT_LAST_DEVICE=y` — 自定义 profile 组合并自动回连上次设备。
- `CONFIG_RT_USING_LWIP=y`、`CONFIG_RT_USING_LWIP212=y`、`CONFIG_LWIP_ALTCP=y`、`CONFIG_LWIP_ALTCP_TLS=y` — 启用 lwIP 协议栈及 TLS 支持，承载 HTTPS 下载。
- `CONFIG_PKG_USING_WEBCLIENT=y`、`CONFIG_PKG_USING_CJSON=y`、`CONFIG_PKG_USING_MBEDTLS=y` — HTTP 客户端、JSON 解析、TLS 库，用于版本查询与固件信息解析。
- `CONFIG_BT_FINSH=y` — 启用 finsh 控制台，便于通过命令驱动升级。

### 编译和烧录
切换到例程 project 目录，运行 scons 命令执行编译（`<board>` 为上述三块板之一）：
```c
> scons --board=<board> -j8
```
例如：
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
编译用户应用时会通过 `AddDFU_PAN_V2` 自动把同级 loader 子程序作为子工程一起编进来。
关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后：
1. 应用把本地名设为 `sifli_pan`，与手机配对后自动发起 PAN 连接，串口出现 `PAN connected`，网络通道就绪。
2. 执行 `ota_check`，若服务器有更新版本，固件信息被写入 flash。
3. 执行 `ota_go` 后设备重启，交给 loader 通过 PAN 网络下载固件、直写目标分区，完成后重启回新的用户应用。
4. 再次执行 `ota_version` 看到版本号变化，即表示升级成功。

## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |06/2026 |初始版本 |
