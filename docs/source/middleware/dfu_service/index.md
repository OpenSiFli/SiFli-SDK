# 固件升级服务

固件升级 (DFU,Device Firmware Update)是将目标固件下载到设备并替换掉当前运行固件，从而完成设备的固件版本升级。通常设备可以通过串口或者空中(OTA，Over-the-Air)方式进行升级固件的下载。SiFli 固件升级服务支持通过BLE的方式下载升级固件。

SIFLI固件升级服务默认支持固件、资源和字体三种内容的下载，其中固件先下载压缩的升级包到预留区域，下载完成后再安装覆盖；资源和字体下载后直接覆盖。

固件升级服务分为2个部分：
```{toctree}
:titlesonly:

firmware_flow.md
firmware_generation.md

```