# USB Mass Storage Device Example (SD/eMMC)

Source path: example/cherryusb/device/msc/sdcard_disk

## Supported Platforms

+ sf32lb52-lcd_n16r8 (SPI SD/TF card)
+ sf32lb52-lcd_a128r16 (SPI SD/TF card)
+ sf32lb52-core_e8r16 (SDIO/eMMC)
+ sf32lb56-lcd_a128r12n1 (SDIO/eMMC)
+ sf32lb56-lcd_n16r12n1 (SDIO/eMMC)
+ sf32lb58-lcd_n16r32n1_dsi (SDIO/eMMC)
+ sf32lb58-lcd_a128r32n1_qspi (SDIO/eMMC)
+ sf32lb58-lcd_n16r32n1_qspi (SDIO/eMMC)
+ spi-hdk_lb573ub7n6（SDIO/eMMC 1-line）

## Overview

This example exports a local block device as a USB mass storage device through CherryUSB MSC. The backend can be:

+ SPI SD/TF card: registered as `sd0` by `RT_USING_SPI_MSD`
+ SDIO/eMMC: registered as `sd0` or `sd1` by the RT-Thread SDIO/MMC driver

The host PC will see a removable disk named `SiFli MSC DEMO`.

Do not mount or access the same block device locally while it is exported over USB. Doing so can corrupt the filesystem.

## Usage

### Hardware Requirements

+ A supported board
+ A USB-A to Type-C cable with data lines
+ A USB host device
+ The corresponding onboard or external SD/eMMC storage medium

### menuconfig

This example selects the storage backend through project configuration:

+ `CHERRYUSB_MSC_BACKEND_SPI_MSD`: SPI SD/TF backend
+ `CHERRYUSB_MSC_BACKEND_SDIO`: SDIO/eMMC backend
+ `CHERRYUSB_DEVICE_MSC_DEVNAME`: exported block device name, for example `sd0` or `sd1`

The SPI backend requires:

+ `BSP_USING_SPI`
+ `BSP_USING_SPI1`
+ `RT_USING_SPI_MSD`

The SDIO/eMMC backend requires:

+ `BSP_USING_SDIO`
+ The matching `BSP_USING_SDMMC1` or `BSP_USING_SDMMC2`
+ `CHERRYUSB_DEVICE_MSC_DEVNAME` matching the board wiring

### Build and Flash

Switch to the example `project` directory and run:

```sh
scons --board=sf32lb56-lcd_a128r12n1 -j10
```

Replace `--board` with another supported board as needed. Refer to the quick start documentation for flashing steps.

## Expected Result

After startup, the example waits until the configured block device is ready. USB MSC is registered only after valid geometry is available, so CherryUSB will not see `block_size=0` during initialization.

After connecting to the host, the PC file manager shows a `SiFli MSC DEMO` removable disk, and the device manager shows a USB mass storage device.

## Update History

| Version | Date | Description |
|:---|:---|:---|
| 0.0.1 | 09/2025 | Initial version |
| 0.0.2 | 05/2026 | Unified SPI SD, SDIO SD, and eMMC backends |
