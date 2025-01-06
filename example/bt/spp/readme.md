@使用指南

    @介绍：spp工程是基于RTthreadOS，使用Sifli SDK的spp的示例。
                - 包含了spp的连接，以及基本使用方法。
    
    @工程编译及下载：Common工程可以通过指定board来编译适应相对board的工程
                - 比如想编译可以在SF32LB555上运行的工程，执行scons --board=eh-lb555即可生成工程
                - 下载可以通过build目录下的download.bat进行，比如同样想烧录上一步生成的555工程，可以执行.\build_eh-lb555\download.bat来通过jlink下载
                - 特别说明下，对于SF32LB52x系列会生成额外的uart_download.bat。可以执行该脚本并输入下载UART的端口号执行下载
    @函数入口：
        1. main(): 系统开始调度后会被call到，该函数会enable BT，初始化协议栈，然后等待手机来连接spp。
        2. bt_app_event_hdl(): 该函数通过BT_EVENT_REGISTER注册，用于处理BT连接相关的事件。
		3. 该Example也提供主动连接前一个设备的pan的命令。

    @相关Shell命令：
        4. 设置蓝牙MAC地址：nvds update addr 6 [addr]. Example: nvds update addr 6 2345670123C3
        5. 通过在串口输入btskey r->btskey 3即可进入spp的finsh菜单
            printf("######################################################\n");
            printf("##                                                  ##\n");
            printf("##           SPP Server Menu                        ##\n");
            printf("##   0. Set SPP instance index                      ##\n");
            printf("##   1. Accept the remote side conn request         ##\n");
            printf("##   2. Reject the remote side conn request         ##\n");
            printf("##   3. Input send data                             ##\n");
            printf("##   4. Transfer a file                             ##\n");
            printf("##   5. Mode change                                 ##\n");
            printf("##   6. Rand send                                   ##\n");
            printf("##   7. spp disconnect                              ##\n");
            printf("##          Please specify device_id and srv_chl:   ##\n");
            printf("##          can get connect information by btskey d ##\n");
            printf("##          exp:btskey 7 0 2                        ##\n");
            printf("##              disconnect device 0 with channel 2  ##\n");
            printf("##   8. disconnect all spp connect                  ##\n");
            printf("##   9. received data is written into a file        ##\n");
            printf("##   x. received data is not written into a file    ##\n");
            printf("##   d. dump connection information                 ##\n");
            printf("##   s. Show Menu                                   ##\n");
            printf("##   r. Return to last menu                         ##\n");
            printf("##                                                  ##\n");
            printf("######################################################\n");

    @手机端建议：
        6. 手机需要下载一个串口app(例如：e调试，spp蓝牙串口)，然后在app里搜索bt设备并连接。