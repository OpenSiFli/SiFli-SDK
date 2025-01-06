@使用指南

    @介绍：ams工程是基于RTthreadOS，使用Sifli SDK AMS service的示例。
                - 包含了BLE的广播、连接，以及对ANCS数据的基本处理。
                - BLE的service和application均运行在HCPU(High performance CPU)
    

    @工程编译及下载：Common工程可以通过指定board来编译适应相对board的工程
                - 比如想编译可以在SF32LB555上运行的工程，执行scons --board=eh-lb555即可生成工程
                - 下载可以通过build目录下的download.bat进行，比如同样想烧录上一步生成的555工程，可以执行.\build_eh-lb555\download.bat来通过jlink下载
                - 特别说明下，对于SF32LB52x系列会生成额外的uart_download.bat。可以执行该脚本并输入下载UART的端口号执行下载
    @函数入口：
        1. main(): 系统开始调度后会被call到，该函数会enable BLE service进而打开BLE，初始化OS的mailbox，并进入while loop。在收到蓝牙power on的通知后打开广播。
        2. ble_app_event_handler(): 该函数通过BLE_EVENT_REGISTER注册到BLE service中，处理GAP/GATT/Common等BLE相关的事件。
                - 由于IOS要求配对以后才能访问ANCS，所以该handler里在连接后会主动发起配对。
        3. app_ancs_init(): 该函数会订阅Data service的ANCS服务，并注册callback获取订阅数据。
		4. 该Example没有本地定义的GATT service。

    @相关Shell命令：
        1. 出厂化BLE相关Flash数据：nvds reset_all 1
                - 为避免Flash冲突，第一次使用最好先下该命令。
        2. 设置蓝牙MAC地址：nvds update addr 6 [addr]. Example: nvds update addr 6 2345670123C3

    @手机端建议：
        1. iPhone手机推荐用第三方软件LightBlue，Android端用nRF Connect进行BLE测试。