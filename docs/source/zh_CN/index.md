# SiFli-SDK编程指南
SiFli-SDK是基于RT-Thread定制开发的一套软件开发框架，使用它可以快速开发运行于思澈科技芯片平台的应用程序。

```{only} SF32LB55X
本文档说明如何在SF32LB55x芯片上使用SDK。
```

```{only} SF32LB56X
本文档说明如何在SF32LB56x芯片上使用SDK。
```
```{only} SF32LB58X
本文档说明如何在SF32LB58x芯片上使用SDK。
```

```{only} SF32LB52X
本文档说明如何在SF32LB52x芯片上使用SDK。
```

SDK使用Kconfig作为软件配置工具，使用SCons进行命令行编译，也可以生成Keil、Eclipse、GNU Makefile的项目用于编译调试，同时也支持生成MS Visual Studio项目，用于PC模拟运行。 

SDK具有以下特点：
- 硬件抽象层 (HAL) 功能用于访问芯片组硬件\
    这部分是独立于实时操作系统（RTOS），可以与不同的 RTOS 集成，也可以在没有 RTOS 的情况下使用。 
- RT-thread设备驱动
- LVGL GUI 库\
    对LVGL进行深度优化，使用芯片自带的GPU加速图形渲染，实现流畅的图形界面效果
- BT/BLE协议栈以及其他中间件, 包括:
    - HFP/A2DP host协议栈以及应用接口 
    - PAN协议栈以及相应的TCP/IP协议栈集成 
    - 快速生成的BLE 服务，可加快 BLE GATT 服务器的开发。
    - 具有硬件加速功能的 CMSIS DSP 和 CMSIS NN 
    - 使用 Flash XIP 硬件加速进行加密/解密。
    - NOR 和 NAND 闪存上的文件系统支持。
    - 具有全面 USB 类支持的 USB 设备堆栈 
- 安全启动\
    它通过安全传输和加密固件更新提供固件更新。SDK带有脚本，负责固件的加密，签名以及传送。


```{toctree}
:hidden:

quickstart/index
app_development/index
hal/index
drivers/index
app_note/index
middleware/index
supported_boards/index
bootloader
example/index
api/index
contribute/index
```


