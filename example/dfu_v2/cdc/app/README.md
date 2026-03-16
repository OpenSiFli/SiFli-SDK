# DFU V2 CDC 用户应用示例

源码路径：example/dfu_v2/cdc/app

(Platform_dfu_v2_cdc_app)=
## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
- `sf32lb52-lcd_n16r8_hcpu` — SF32LB52X 芯片，NOR Flash（16Mb NOR + 8MB PSRAM）。

## 概述
<!-- 例程简介 -->
本例程演示通过 USB-CDC（虚拟串口）通道进行 DFU V2 固件升级的用户应用侧。设备枚举为一个 CDC ACM 串口，正常运行时在屏幕上显示版本号；当 PC 工具通过该串口发来升级触发指令后，设备应答并重启进入 DFU loader 完成刷写。
PC 端配套工具为 `dfu_v2_cdc_tool.py`，负责先发送触发指令让设备进入 DFU 模式，再通过 USB-CDC 上传固件。

## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
把编译产物烧录到设备后，设备正常运行并枚举为 USB-CDC 串口，屏幕显示版本号。准备一份新版本固件，在 PC 上运行 `dfu_v2_cdc_tool.py` 并加 `--trigger` 触发升级，设备重启进入 loader 完成刷写后再回到新的用户应用。

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](#Platform_dfu_v2_cdc_app)）。
+ 一根 USB 数据线，用于连接开发板与 PC。

### menuconfig配置
本例程的关键配置（已在 proj.conf 中默认开启）：
- `CONFIG_USING_DFU_V2=y` — 启用 DFU V2 中间件。
- `CONFIG_DFU_V2_USE_CDC_TRANSPORT=y` — 选择 USB-CDC 作为 DFU 传输通道。
- `CONFIG_PKG_USING_CHERRYUSB=y` / `CONFIG_PKG_CHERRYUSB_DEVICE=y` — 启用 CherryUSB 设备栈。
- `CONFIG_PKG_CHERRYUSB_DEVICE_MUSB_SIFLI=y` — 选用 SiFli MUSB 设备控制器。
- `CONFIG_PKG_CHERRYUSB_DEVICE_CDC_ACM=y` — 启用 CDC ACM 类（虚拟串口）。
- `CONFIG_PKG_USING_LITTLEVGL2RTT=y` / `CONFIG_LVGL_V9=y` — 启用 LVGL（v9）用于显示版本号界面。
- `CONFIG_RT_USING_MEMHEAP=y` — LVGL 内存适配所需的 RT-Thread memheap。

### 编译和烧录
切换到例程project目录，运行scons命令执行编译：
```c
> scons --board=sf32lb52-lcd_n16r8_hcpu -j8
```
编译应用工程时会通过 building.py 中的 `AddDFU_CDC` 自动把同级 loader 作为子工程一并构建，无需单独编译 loader。
关于编译、下载的详细步骤，请参考[快速入门](/quickstart/get-started.md)的相关介绍。

## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
1. 将编译产物烧录到设备并复位。设备运行后枚举为一个 USB-CDC 串口，屏幕显示标题 `CDC DFU EXAMPLE`、品牌 `SIFLI` 以及版本号 `Ver: 1.0.9`（版本号来自 `main.c` 中的 `APP_VERSION`）。
2. 准备一份“新版本”固件用于验证升级：把 `src/main.c` 里的 `APP_VERSION` 改一个新值（如 `1.1.0`）后重新编译，得到 `project/build_sf32lb52-lcd_n16r8_hcpu/main.bin`。然后在 PC 上运行升级工具，指定串口、这份新固件和目标分区地址，并加上 `--trigger` 让工具先发送触发指令：

   ```
   python dfu_v2_cdc_tool.py --port COM3 --firmware project/build_sf32lb52-lcd_n16r8_hcpu/main.bin --addr 0x12218000 --trigger
   ```

   其中 `0x12218000` 是 `HCPU_FLASH_CODE` 分区地址（分区表基址 `0x12000000` 加偏移 `0x00218000`）。
3. 触发流程：工具向设备发送 4 字节魔数 `AA 55 DF 00`；`main.c` 的 `usbd_cdc_acm_bulk_out` 回调识别该魔数后，回送应答 `AA 55 DF 01`，并置位 `g_dfu_trigger_requested`；主循环检测到该标志后调用 `dfu_enter_dfu_mode()`，写下升级标志并重启。
4. 重启后 bootloader 检测到升级标志，跳入 DFU loader。工具自动重连，按 DFU V2 协议把固件上传并直写到目标分区，完成后命令设备复位。
5. 设备重启回新的用户应用。屏幕上的 `Ver:` 版本号变为新固件中的值，即表示升级成功。

## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |06/2026 |初始版本 |
| | | |
| | | |
