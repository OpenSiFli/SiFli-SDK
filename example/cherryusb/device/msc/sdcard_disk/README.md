# USB 大容量存储设备示例（SD/eMMC）

源码路径：example/cherryusb/device/msc/sdcard_disk

## 支持的平台

+ sf32lb52-lcd_n16r8（SPI SD/TF 卡）
+ sf32lb52-lcd_a128r16（SPI SD/TF 卡）
+ sf32lb52-core_e8r16（SDIO/eMMC）
+ sf32lb56-lcd_a128r12n1（SDIO/eMMC）
+ sf32lb56-lcd_n16r12n1（SDIO/eMMC）
+ sf32lb58-lcd_n16r32n1_dsi（SDIO/eMMC）
+ sf32lb58-lcd_a128r32n1_qspi（SDIO/eMMC）
+ sf32lb58-lcd_n16r32n1_qspi（SDIO/eMMC）
+ spi-hdk_lb573ub7n6（SDIO/eMMC单线）

## 概述

本例程演示基于 CherryUSB MSC 将本地块设备导出为 USB 大容量存储设备。后端可以是：

+ SPI SD/TF 卡：通过 `RT_USING_SPI_MSD` 注册 `sd0`
+ SDIO/eMMC：通过 RT-Thread SDIO/MMC 驱动注册 `sd0` 或 `sd1`

PC 通过文件管理器可以看到一个名为 `SiFli MSC DEMO` 的 U 盘。

注意：块设备导出给 USB 主机期间，不要在板端同时 mount 或访问同一个块设备，否则可能造成文件系统损坏。

## 例程的使用

### 硬件需求

+ 一块本例程支持的开发板
+ 带数据传输功能的 USB-A 转 Type-C 数据线
+ 支持 USB 的主机设备
+ 对应板载或外接的 SD/eMMC 存储介质

### menuconfig 配置

本例程通过工程配置选择后端：

+ `CHERRYUSB_MSC_BACKEND_SPI_MSD`：SPI SD/TF 卡后端
+ `CHERRYUSB_MSC_BACKEND_SDIO`：SDIO/eMMC 后端
+ `CHERRYUSB_DEVICE_MSC_DEVNAME`：导出的块设备名，例如 `sd0` 或 `sd1`

SPI 后端需要启用：

+ `BSP_USING_SPI`
+ `BSP_USING_SPI1`
+ `RT_USING_SPI_MSD`

SDIO/eMMC 后端需要启用：

+ `BSP_USING_SDIO`
+ 对应的 `BSP_USING_SDMMC1` 或 `BSP_USING_SDMMC2`
+ 按板级连接配置 `CHERRYUSB_DEVICE_MSC_DEVNAME`

### 编译和烧录

切换到例程 `project` 目录，运行：

```sh
scons --board=sf32lb56-lcd_a128r12n1 -j10
```

其它支持板子替换 `--board` 即可。烧录方式请参考快速上手文档。

## 例程的预期结果

例程启动后会等待配置的块设备 ready。块设备 ready 后才注册 USB MSC，避免 CherryUSB 在设备几何信息未就绪时获取到 `block_size=0`。

主机连接后，PC 文件管理器中出现 `SiFli MSC DEMO` U 盘，设备管理器中出现 USB 大容量存储设备。

## 更新记录

|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |09/2025 |初始版本 |
|0.0.2 |05/2026 |合并 SPI SD、SDIO SD 和 eMMC 后端 |
