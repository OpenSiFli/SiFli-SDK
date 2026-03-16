# DFU V2 BLE 用户应用示例

源码路径：example/dfu_v2/ble/app

(Platform_dfu_v2_ble_app)=
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
- `sf32lb52-lcd_n16r8_hcpu` —— SF32LB52X，16MB NOR Flash + 8MB PSRAM。

## 概述
<!-- 例程简介 -->
本例程是一个标准的 BLE 外设应用，集成了 DFU V2 中间件。它既能在运行过程中实时更新非自身的镜像（资源、字体、loader 等），也能在需要更新 HCPU 自身应用时切换进独立的 DFU loader 子程序完成升级。

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
1. 设备上电后启动 BLE 广播，手机 / 上位机通过 BLE 连接后即可推送固件。推送资源、字体、loader 等非 HCPU 镜像时，设备在运行中实时写入对应分区。
2. 更新 HCPU 自身应用时，运行中的应用不能擦写自己，需要在串口里执行 `cmd_diss ota_reboot` 切换进 loader 完成升级。
3. 手机 APP 与旧版 BLE DFU 保持一致，无需改动。

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](#Platform_dfu_v2_ble_app)）。
+ 一台手机或上位机（作为 BLE 主机推送固件）。

### menuconfig配置
本例程的关键配置（已在 proj.conf 中默认开启）：
- `CONFIG_USING_DFU_V2=y` —— 启用 DFU V2 中间件。
- `CONFIG_DFU_V2_USE_BLE_TRANSPORT=y` —— 选择 BLE 作为 DFU 传输通道。
- `CONFIG_BLUETOOTH=y` —— 启用蓝牙协议栈（BLE 外设所需）。
- `CONFIG_BSP_BLE_SERIAL_TRANSMISSION=y` —— 启用 BLE 串行透传服务，作为 DFU 数据通道。
- `CONFIG_BT_CON_NUM_CUSTOMIZE=y` / `CONFIG_CFG_MAX_BT_ACL_NUM=2` —— 自定义并限制蓝牙连接数。
- `CONFIG_PKG_SIFLI_MBEDTLS_BOOT=y` —— 引入 mbedTLS，用于固件校验。

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
编译用户应用时，配套的同通道 loader 子程序会通过 `AddDFU_BLE()` 作为子工程自动一并编译。

关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
例程启动后：
1. 串口打印启动横幅 `=== peripheral_with_ota V2 Start ===`，随后初始化 DFU V2 中间件并启动 BLE 广播，广播名形如 `SIFLI_APP-xx-...`。
2. 初始化阶段调用 `dfu_mode_host_set_self_img_id(0)`，告知 Host 模式跳过 HCPU 镜像（`img_id=0`），因为运行中的应用无法覆盖自己。
3. 手机 / 上位机通过 BLE 推送资源、字体、loader 等非 HCPU 镜像时，串口打印 OTA 进度日志（`OTA progress`、`OTA image N complete`、`OTA all complete`）。
4. 更新 HCPU 应用时，执行 `cmd_diss ota_reboot` 后设备重启并跳进 loader，由它接收并直写新的 HCPU 固件，完成后重启回新版本应用。
5. 随时可用 `cmd_diss ota_info` 查询当前 OTA 进度，空闲时打印 `OTA: idle`。

## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |06/2026 |初始版本 |
| | | |
| | | |
