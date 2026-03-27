# BT SPP Profile Example

Source code path: example/bt/spp

{#Platform_spp}
## Supported Platforms
<!-- 支持哪些板子和芯片平台 -->
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x
+ sf32lb52-lcd series
+ sf32lb56-lcd series
+ sf32lb58-lcd series

## Overview
<!-- 例程简介 -->
This example demonstrates SPP connection and disconnection, data transmission,
file transfer, and speed testing, including:
+ SPP server
+ SPP client
+ SPP server and SPP client

### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported
  Platforms](#Platform_spp)).
+ An Android phone (SPP only supports Android phones, iOS phones cannot use
  SPP), and you need to download an SPP Bluetooth serial port tool APP (e.g.,
  SPP Bluetooth Serial Port, e-Debug, etc.).

## Example Usage
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->
The example will enable Bluetooth by default at startup, and the phone can enter
the SPP Bluetooth serial port tool APP to search and connect (using e-Debug app
as an example):
1. Phone app connects to development board SPP: Enter the classic Bluetooth
   device search interface on the phone to search for the development board:
   ![e-Debug Search Device](./assets/1.png) After finding the specified device,
   you can long-press the device to modify the UUID of the SPP to connect
   (default is 0x1101 UUID). <font color=red>This example supports custom SPP
   UUIDs to increase the number of SPP connections, and supports up to 7 SPP
   connections with the same phone. Users can add custom UUIDs by implementing
   the bt_spp_srv_add_uuid_list interface. Also, if the phone wants to connect
   multiple SPP connections, it may need to establish one SPP connection in each
   app separately.</font> ![e-Debug Modify Device UUID to
   Connect](./assets/2.png) ![e-Debug Connect Device](./assets/3.png) You can
   see the successful connection log in the serial port output: ![e-Debug
   Connection Success](./assets/4.png)

2. Phone sends data to development board You can send data from the phone app to
   the development board, as shown below: ![e-Debug Send Data](./assets/7.png)
   The development board's serial port log will print the received data size and
   speed: ![e-Debug Send Data](./assets/11.png)

3. Development board sends data to phone Use the command `spp send_data +
   address + service channel` to send test data to the phone, where the service
   channel can be obtained from the serial port output: ![Get Connected Service
   Channel](./assets/5.png) The phone app will display the received data:
   ![Phone Receives Data from Development Board](./assets/6.png)

4. Phone sends file to development board In the phone app, you can select to
   send a file to the development board: ![Phone Sends File to Development
   Board](./assets/8.png) ![Set Send Interval and Data Size per
   Packet](./assets/9.png) The development board's serial port log will print
   the received data size and speed, and the phone will also display the sending
   progress: ![e-Debug Send Data](./assets/11.png) ![e-Debug Send
   Data](./assets/10.png)

5. Development board sends file to phone If there are files on the development
   board, you can use the command `spp send_file + address + service channel +
   filename` to send the specified file to the phone, where the service channel
   can be obtained from the serial port output: ![Get Connected Service
   Channel](./assets/5.png) ![View Files on Development Board](./assets/12.png)
   ![Send File to Phone](./assets/13.png) The phone will display the received
   content and data size: ![Send File to Phone](./assets/14.png)

6. Development board initiates SPP connection The development board can also
   initiate SPP connections, but since many phones only support 0x1101 SPP, and
   the development board will be quickly disconnected by the phone after
   actively connecting, you can choose to connect development board to
   development board. Since this example supports custom SPP UUIDs to increase
   the number of SPP connections, you can flash this example to two development
   boards separately, allowing multiple SPP connections between development
   boards, with one acting as the phone role. For example, if development board
   A needs to connect to development board B's 0x3001 SPP channel, you can use
   the command `spp search + address + uuid length + uuid` to first query
   whether the peer device supports 0x3001 SPP: ![Search Peer
   SPP](./assets/spp_search.png) Development board A can use the command `spp
   connect + address + uuid length + uuid` to connect to the peer device's
   0x3001 SPP: ![Connect to Peer SPP](./assets/spp_connect.png)

7. Throughput test between development boards The functions mentioned above also
   exist in connections between development boards. Here we mainly explain the
   throughput test between development boards. Development board A can use the
   command `spp through_put + address + service channel + transmitted data size`
   to transmit random data of specified size to development board B. Development
   board B will calculate the rate after receiving all data: The service channel
   can be obtained from the serial port output: ![Get Connected Service
   Channel](./assets/5.png) ![SPP Throughput](./assets/spp_through_put.png)

8. Disconnect specified SPP connection The development board can use the command
   `spp disc + address + service channel` to disconnect the specified SPP
   connection: The service channel can be obtained from the serial port output:
   ![Get Connected Service Channel](./assets/5.png) ![Disconnect SPP Service
   Channel](./assets/spp_disconnect.png)

9. Disconnect all connected SPP The development board can use the command `spp
   disc_all` to disconnect all SPP connections.

### menuconfig Configuration
1. This example needs to read and write files, so it requires a file system.
   Configure the `FAT` file system:
    - Path: RTOS → RT-Thread Components → Device virtual file system
    - Enable: Enable elm-chan fatfs
        - Macro switch: `CONFIG_RT_USING_DFS_ELMFAT`
        - Description: Enable fatfs file system
     ```{tip}
     mnt_init mounts the root partition.
     ```
2. Enable Bluetooth (`BLUETOOTH`):
    - Path: Sifli middleware → Bluetooth
    - Enable: Enable bluetooth
        - Macro switch: `CONFIG_BLUETOOTH`
        - Description: Enable bluetooth function
3. Enable SPP server and SPP client:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT
      service
    - Enable: Enable BT finsh (optional)
        - Macro switch: `CONFIG_BT_FINSH`
        - Description: Enable finsh command line for Bluetooth control
    - Enable: Manually select profiles
        - Macro switch: `CONFIG_BT_PROFILE_CUSTOMIZE`
        - Description: Manually select profiles to enable
    - Enable: Enable SPP client
        - Macro switch: `CONFIG_CFG_SPP_CLT`
        - Description: Enable SPP client function for actively initiating SPP
          connections, etc.
    - Enable: Enable SPP server
        - Macro switch: `CONFIG_CFG_SPP_SRV`
        - Description: Enable SPP server function so that peer devices can
          search and connect to SPP service
4. Enable BT connection manager:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Classic BT
      service
    - Enable: Enable BT connection manager
        - Macro switch: `CONFIG_BSP_BT_CONNECTION_MANAGER`
        - Description: Use connection manager module to manage BT connections
5. Enable NVDS:
    - Path: Sifli middleware → Bluetooth → Bluetooth service → Common service
    - Enable: Enable NVDS synchronous
        - Macro switch: `CONFIG_BSP_BLE_NVDS_SYNC`
        - Description: Bluetooth NVDS synchronization. When Bluetooth is
          configured to HCPU, BLE NVDS can be accessed synchronously, enable
          this option; when Bluetooth is configured to LCPU, this option needs
          to be disabled

### Compilation and Flashing
Switch to the example project directory and run the scons command to compile:
```c
&gt; scons --board=eh-lb525 -j32
```
Switch to the example `project/build_xx` directory and run `uart_download.bat`,
then select the port as prompted to download:
```c
$ ./uart_download.bat

     UART Download

Please input the serial port number: 5
```
For detailed steps on compilation and downloading, please refer to the related
introduction in [Quick Start](/quickstart/get-started.md).

## Expected Results
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
After the example starts:
1. SPP functionality works normally

## Troubleshooting


## Reference Documentation
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

## Update History
| Version | Date    | Release Notes   |
| ------- | ------- | --------------- |
| 0.0.1   | 03/2025 | Initial version |
|         |         |                 |
|         |         |                 |
