@使用指南

    @介绍：music工程是基于RTthreadOS，使用Sifli SDK的a2dp的示例。
                - 包含了a2dp的连接，以及基本使用方法。
    
    @工程编译及下载：Common工程可以通过指定board来编译适应相对board的工程
                - 比如想编译可以在SF32LB555上运行的工程，执行scons --board=eh-lb555即可生成工程
                - 下载可以通过build目录下的download.bat进行，比如同样想烧录上一步生成的555工程，可以执行.\build_eh-lb555\download.bat来通过jlink下载
                - 特别说明下，对于SF32LB52x系列会生成额外的uart_download.bat。可以执行该脚本并输入下载UART的端口号执行下载
    @函数入口：
        1. main(): 系统开始调度后会被call到，该函数会enable BT，初始化协议栈和BT name，并进入while loop。然后等待手机连接。
        2. bt_app_event_hdl(): 该函数通过BT_EVENT_REGISTER注册，用于处理BT连接，a2dp相关的事件。
		3. 该Example没有任何主动发起连接的策略，只能手机来连接。

    @相关Shell命令：
        4. 设置蓝牙MAC地址：nvds update addr 6 [addr]. Example: nvds update addr 6 2345670123C3

    @手机端建议：
        5. 手表必须保证打开媒体音频。