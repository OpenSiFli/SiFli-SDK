# DFU V2 BLE Loader 示例

源码路径：example/dfu_v2/ble/loader

(Platform_dfu_v2_ble_loader)=
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
loader 工程不固化板子，目标板由构建时的 `--board` 参数决定，与配套 BLE 用户应用保持一致（已验证 `sf32lb52-lcd_n16r8_hcpu`）。其他带有对应分区表的 SF32LB52X 板子同样可用。

## 概述
<!-- 例程简介 -->
本例程是 DFU V2 通过 BLE 通道进行固件升级时使用的 DFU loader 子程序。它独立烧在 `DFU_V2_LOADER` 分区并独立运行，通过 BLE 接收手机端推送的新固件并直写目标分区，是真正完成擦写动作的一段程序。

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
通常不需要单独构建本 loader。它通过 `building.py` 中的 `AddDFU_BLE()`，由 BLE 用户应用的 `SConstruct` 以 `AddChildProj` 的形式自动作为子工程拉入。也就是说，只要在应用目录里正常构建，loader 会被一并编译。如需单独调试，也可以进入本工程目录单独构建。

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](#Platform_dfu_v2_ble_loader)）。
+ 一台带 BLE 的手机及配套升级 APP。

### menuconfig配置
本例程的关键配置（已在 proj.conf 中默认开启）：
- 宏开关：`CONFIG_USING_DFU_V2`
    - 作用：启用 DFU V2 中间件。
- 宏开关：`CONFIG_DFU_V2_USE_HOST_MODE`
    - 作用：启用 Host 模式，由本端主导接收并写入固件。
- 宏开关：`CONFIG_DFU_V2_USE_BLE_TRANSPORT`
    - 作用：选择 BLE 作为传输通道。
- 宏开关：`CONFIG_BLUETOOTH`
    - 作用：启用蓝牙协议栈。
- 宏开关：`CONFIG_BSP_BLE_SERIAL_TRANSMISSION`
    - 作用：启用 BLE 串行透传服务，作为 DFU 数据通道。
- 宏开关：`CONFIG_OTA_55X`
    - 作用：启用 OTA 镜像布局支持。
- 宏开关：`CONFIG_PKG_SIFLI_MBEDTLS_BOOT`、`CONFIG_PKG_USING_FLASHDB`
    - 作用：DFU 校验与存储所需依赖组件。

### 编译和烧录
切换到 BLE 用户应用的 project 目录，运行 scons 命令执行编译，构建应用时会自动包含本 loader：
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
如需单独构建该 loader（例如单独调试），进入本工程目录执行：
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
产物生成在 `build_<board_name>` 目录下。关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
1. 设备进入 BLE 升级模式后开启广播，设备名形如 `SIFLI_DFU-xx-xx-xx-xx-xx-xx`，其中 xx 代表本设备的蓝牙地址，等待手机连接。
2. 手机端 APP 连接后通过 BLE 串行透传服务推送固件，loader 接收并直写目标分区；升级过程中上报进度百分比，开启 UI 时会显示进度条。
3. 为避免卡死在升级模式，loader 内置两道超时保护：全局超时 5 分钟（`DFU_MODE_TIMEOUT_MS`，整个升级模式的总时限）、连接后无数据 60 秒的不活跃超时（`DFU_INACTIVITY_TIMEOUT_MS`，BLE 连接后每收到一笔进度便重置）。任一超时触发且升级未完成时，自动重启回到用户应用。
4. 固件全部写完后，loader 延时 3 秒（`REBOOT_DELAY_MS`，留出时间显示成功提示）后重启，回到新的用户应用。
5. 调试时可在 loader 控制台输入 `dfu_ble_info` 查看 BLE 上电状态、MTU 与当前进度。

## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |06/2026 |初始版本 |
| | | |
| | | |
