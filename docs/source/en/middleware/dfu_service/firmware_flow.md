# Process Introduction

## Flash Layout

As shown in the figure below, by default, Bootrom will jump to 0x10020000 according to the flash config table (for details, refer to [](/bootloader.md)). The firmware upgrade service requires creating a separate OTA manager project, and the user project (User Code) needs to be placed after the OTA manager.

After Bootrom jumps to OTA manager, it will act according to the current state: if no upgrade is needed, it jumps to the user project; if an upgrade is needed, it waits for the upgrade.

The starting address of the user project and the Upgrade bin address where firmware upgrade packages are stored can be configured through the project's _memory_map.h_ and _custom_memory_map.h_. It is recommended to place the Upgrade bin at the end of Flash.

 ![](../../../assets/ota_flash_layout.png)


## Process Overview

The diagram below illustrates the basic OTA process. A complete OTA upgrade file contains _ctrl_packet.bin_ with verification information, followed by firmware packages, resource packages, and font packages.

![](../../../assets/ota_flow.png)


- Step 1. The remote device sends _ctrl_packet.bin_ to user bin to confirm whether OTA can proceed. If yes, continue to step 2
- Step 2. User bin needs to restart and enter ota manager bin, then re-establish Bluetooth connection with the remote device. If no connection is made within a certain time, it will return to user bin.
- Step 3. After establishing Bluetooth connection, the remote device will continuously transmit upgrade packages until all upgrade packages are transmitted.
    - If firmware upgrade package download fails, it will return to user bin.
    - If resource and font downloads fail, it will forcibly remain in ota manager bin until the download succeeds.
    - After download failure, OTA supports both resume and forced restart from beginning methods, determined by the remote device.
- Step 4. Decompress the firmware upgrade package and complete installation.
- Step 5. After installation is complete, jump to user bin to complete OTA.


## Important Notes

- OTA upgrade packages use encryption. Ensure using a flash config table with encryption information; if decryption exists in efuse, ensure efuse has correctly programmed relevant content. If verification fails, the remote device will receive error code 09 (DFU_ERR_CONTROL_PACKET_INVALID).
- If the OTA process fails due to BLE disconnection or other reasons, the remote device will receive error code 10 (DFU_ERR_OTA_ONGOING). The remote device can decide whether to resume or restart the upgrade from the beginning.


## Project Configuration

User project configuration:
- Both cores need configuration:
    ![](../../../assets/ota_hcpu_config1.png)
    ![](../../../assets/ota_hcpu_config2.png)
- LCPU configuration:
    If BLE service is on LCPU, LCPU port service needs to be configured so that OTA manager can obtain BLE data
    ![](../../../assets/ota_lcpu_config1.png)

OTA Manager project configuration:
- In addition to the content that both cores need to configure in the user project, zlib also needs to be added
    ![](../../../assets/ota_lcpu_config2.png)