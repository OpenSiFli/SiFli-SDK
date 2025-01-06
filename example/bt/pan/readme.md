@使用指南

    @介绍：pan工程是基于RTthreadOS，使用Sifli SDK的pan的示例。
                - 包含了pan的连接，以及基本使用方法。
    
    @工程编译及下载：Common工程可以通过指定board来编译适应相对board的工程
                - 比如想编译可以在SF32LB555上运行的工程，执行scons --board=eh-lb555即可生成工程
                - 下载可以通过build目录下的download.bat进行，比如同样想烧录上一步生成的555工程，可以执行.\build_eh-lb555\download.bat来通过jlink下载
                - 特别说明下，对于SF32LB52x系列会生成额外的uart_download.bat。可以执行该脚本并输入下载UART的端口号执行下载
    @函数入口：
        1. main(): 系统开始调度后会被call到，该函数会enable BT，初始化协议栈，然后等待手机来连接bt之后会主动发起pan的连接。
        2. bt_app_event_hdl(): 该函数通过BT_EVENT_REGISTER注册，用于处理BT连接，pan相关的事件。
		3. 该Example也提供主动连接前一个设备的pan的命令。

    @相关Shell命令：
        4. 设置蓝牙MAC地址：nvds update addr 6 [addr]. Example: nvds update addr 6 2345670123C3
        5. connect_last，主动连接上一个配对设备的pan

    @手机端建议：
        6. 手机一定要保证蓝牙网络共享是开着的，具体的Android和ios系统蓝牙共享网络的开启方法可在网上查找。