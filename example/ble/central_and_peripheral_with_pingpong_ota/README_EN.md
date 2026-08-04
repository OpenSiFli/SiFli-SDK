# BLE Central and Peripheral with Pingpong OTA Example
Source Code Path: example/ble/central_and_peripheral_with_pingpong_ota
(Platform_cen_peri)=
## Supported Platforms
<!-- Which boards and chip platforms are supported -->
Projects with flash table programmed in mpi5
## Overview
<!-- Brief introduction of the example -->
This example demonstrates how the platform can function as both GAP central and peripheral, as well as GATT client and server simultaneously.
It also shows how to upgrade the project using the pingpong method
## Example Usage
<!-- Instructions on how to use the example, such as connecting hardware pins to observe waveforms. Compilation and flashing can reference related documents.
For rt_device examples, also list the configuration switches used in this example, such as PWM example using PWM1 which needs to be enabled in the onchip menu -->
1. The Finsh commands for this example can be printed by entering 'diss help' to show command usage.
2. When acting as a peripheral device, it will start advertising on boot with a name formatted as SIFLI_APP-xx-xx-xx-xx-xx-xx, where xx represents the device's Bluetooth address. You can connect using a mobile BLE APP.
3. When acting as a central device, you can search for other peripheral devices through finsh commands and initiate connections.
4. When acting as a GATT server, you can perform write and read operations on the mobile terminal, or enable CCCD, and the device will update the characteristic value every second.
5. When acting as a GATT client, you can search and display the server's database through finsh commands, and perform read or write operations on characteristic values.
6. Connect the board through the SIFLE BLE APP, select a file, and perform the upgrade
### Hardware Requirements
Before running this example, you need to prepare:
+ A development board supported by this example ([Supported Platforms](#Platform_cen_peri)).
+ A mobile device.
### menuconfig Configuration
1. Enable Bluetooth(`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
         - Macro switch: `CONFIG_BLUETOOTH`
         - Description: Enable Bluetooth functionality
2. Enable GAP, GATT Client, BLE connection manager:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → BLE service
    - Enable: Enable BLE GAP central role
         - Macro switch: `CONFIG_BLE_GAP_CENTRAL`
         - Description: Switch for BLE CENTRAL (central device). When enabled, provides scanning and active connection initiation with peripherals.
    - Enable: Enable BLE GATT client
         - Macro switch: `CONFIG_BLE_GATT_CLIENT`
         - Description: Switch for GATT CLIENT. When enabled, allows active service discovery, data reading/writing, and notification reception.
    - Enable: Enable BLE connection manager
         - Macro switch: `CONFIG_BSP_BLE_CONNECTION_MANAGER`
         - Description: Provides BLE connection control management, including multi-connection management, BLE pairing, and link connection parameter updates.
3. Enable NVDS:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Common service
    - Enable: Enable NVDS synchronous
        - Macro switch: `CONFIG_BSP_BLE_NVDS_SYNC`
        - Description: Bluetooth NVDS synchronization. When Bluetooth is configured to HCPU, BLE NVDS can be accessed synchronously, so enable this option; when Bluetooth is configured to LCPU, this option needs to be disabled.
4. Enable pingpong OTA related features:
    - Path: Sifli middleware → Device firmware update support functions
    - Enable: Device firmware using compress
        - Macro switch: `CONFIG_BSP_USING_DFU_COMPRESS`
        - Description: Enable DFU compression functionality
    - Enable: NAND flash OTA
        - Macro switch: `CONFIG_OTA_56X_NAND`
        - Description: Enable pingpong OTA architecture. Although the macro name includes 56X, it is not limited to specific chipset types.
5. Enable solution OTA macro:
    - Path: Home page
    - Enable: using solution dfu protocol
        - Macro switch: `CONFIG_SOL_DFU`
        - Description: Enable solution dfu protocol.

### Other Access Related Notes
1. SConstruct
FTAB needs to be added via addchildproj:
![Sconstruct](./assets/SConstruct.png)
2. ptab.json
Need to allocate a main and base2 region with the same size as HCPU_FLASH_CODE_LOAD_REGION as the pong region:
![PTAB](./assets/ptab.png)
3. Boot loader
Need to add jump logic, please refer to the implementation of this example for details:
![BOOT](./assets/bootloader.png)
4. Generation Command
```shell
.\imgtoolv37.exe gen_dfu --img_para main 0 0 --key=s01 --sigkey=sig --dfu_id=1 --hw_ver=51 --sdk_ver=7001 --fw_ver=1001001 --com_type=0 --bksize=2048 --align=0
```
main is the name of the bin to be upgraded. If it's hcpu.bin, just fill in hcpu.
The first parameter after the bin name is for compression, 16 means using compression, 0 means no compression. Only no compression can be selected.
The second parameter after the bin name represents the image id, hcpu is 0, and multiple bins can be generated simultaneously.
Then you need to implement whether the subsequent bins are stored in the backup area and installed finally, or directly overwrite the target address.
5. Mobile Operation
The operation is shown in the following figure: search for the board's BLE broadcast, click on the corresponding device, then select nand dfu, first select the ctrl file, then select hcpu as the outmain.bin generated刚才制作的outmain.bin, and finally click the start button.
![app1](./assets/app.jpg)![app2](./assets/app2.jpg)
![app3](./assets/app3.jpg)![app2](./assets/app4.jpg)
### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
> scons --board=sf32lb58-lcd_n16r64n4 -j8
```
Switch to the example `project/build_xx` directory, run `uart_download.bat`, and select the port as prompted to download:
```c
$ ./uart_download.bat
     Uart Download
please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to the relevant introduction in [Quick Start](/quickstart/get-started.md).
## Expected Results
<!-- Describe the expected behavior after running the example, such as which LEDs light up or which logs are printed, so that users can determine whether the example is running properly. The result can be explained step by step together with the code. -->
After the example starts up:
1. The board can be discovered and connected by a mobile BLE APP, which can perform GATT characteristic read/write operations.
2. The board can scan for other BLE devices, connect to them, discover their GATT database, and perform GATT read/write operations.
3. The upgrade can be performed via the APP. You can confirm whether the upgrade succeeded through the following steps:

    a. First compile and flash a firmware build to the board as the original firmware before the upgrade.

    b. Open `src/main.c`, add the following line at the beginning of the `main` function, then save and recompile (recompiling is enough — do not flash it to the board again, otherwise the log would be printed before the upgrade):

    ```c
    rt_kprintf("Upgrade succeeded\n");
    ```
    ![text](./assets/upyes.png)

    c. Use the firmware recompiled in step b to build the HCPU upgrade package, then perform the upgrade via the APP. If the upgraded firmware prints "Upgrade succeeded" over the serial port at runtime, the upgrade succeeded.
   
## Troubleshooting
## Reference Documentation
<!-- For rt_device examples, RT-Thread official documentation provides detailed explanations. You can add web links here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->
## Update History
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |07/2025 |Initial version |
| | | |
| | | |
No newline at end of right file.