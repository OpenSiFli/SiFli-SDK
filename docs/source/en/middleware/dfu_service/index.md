# Firmware Upgrade Service

Device Firmware Update (DFU) is the process of downloading target firmware to a device and replacing the currently running firmware, thereby completing the device's firmware version upgrade. Typically, devices can download upgrade firmware via serial port or Over-the-Air (OTA) methods. SiFli firmware upgrade service supports downloading upgrade firmware via BLE.

SIFLI firmware upgrade service supports downloading three types of content by default: firmware, resources, and fonts. Among these, firmware is first downloaded as a compressed upgrade package to a reserved area, then installed and overwritten after download completion; resources and fonts are directly overwritten after download.

The firmware upgrade service is divided into 2 parts:
```{toctree}
:titlesonly:

firmware_flow.md
firmware_generation.md

```