@使用指南
    @介绍：
        BLE/BT功耗测试例程（LCPU主频为24MHz）。BLE可测试广播与连接模式的功耗，BT可测试Scan和Sniff模式的功耗。
        唤醒PIN在不同开发板上会有所不同，对应关系如下
            - EC-LB58X： 使用PA64
            - EH-LB561/EH-LB563： 使用PB34，对应HDK上的HR_INT
            - EH-ss6500/EH-LB523/EH-LB520： 使用PA24，对应HDK上的GPS_PEN

        当唤醒PIN为低电平时HCPU和LCPU均无法进入低功耗模式，
        此时可以通过HCPU的Console发送命令修改参数，
        当唤醒PIN接高电平时，HCPU可以进入低功耗模式，LCPU则周期性的进出低功耗模式，此时Console无法使用。
        HCPU使用UART1作为Console端口，LCPU使用UART4作为Console端口。

    @相关Console命令
        - HCPU
            1. 出厂化BLE相关Flash数据：nvds reset_all 1
                    - 为避免Flash冲突，第一次使用最好先下该命令。
            2. 设置蓝牙MAC地址：nvds update addr 6 [addr]. Example: nvds update addr 6 2345670123C3
            3. 'ble_config adv interval_in_ms': 修改广播周期，其中'interval_in_ms'是毫秒为单位的广播间隔
            4. 'ble_config conn interval_in_ms': 修改连接周期，其中'interval_in_ms'是毫秒为单位的连接间隔，
              需要在与手机连接后发送命令
            5. 'btskey': BTS菜单控制命令，可修改BT参数。开机后默认处于BTS主菜单，BTS菜单为多级菜单，可发送'btskey s'命令显示当前的菜单内容，
               再根据菜单提示发送命令进入下一级菜单或者执行某个菜单的功能。比如，在主菜单下，依次发送以下三个命令可以打开Page Scan并关闭Inquiry Scan
               1) btskey 1
               2) btskey 7
               3) btskey 2

              主菜单如下图
'''               
                    ######################################################
                    ##                                                  ##
                    ##           BTS2 Demo Main Menu                    ##
                    ##   1. Generic Command                             ##
                    ##   2. SPP Client                                  ##
                    ##   3. SPP Server                                  ##
                    ##   4. HFP HF                                      ##
                    ##   6. A2DP Sink                                   ##
                    ##   8. L2CAP bqb test                              ##
                    ##   p. AVRCP                                       ##
                    ##   s. Show Menu                                   ##
                    ##   q. Exit                                        ##
                    ##                                                  ##
                    ######################################################
'''
    
              发送了'btskey 1'之后显示子菜单Generic Command Menu  
                    ######################################################
                    ##                                                  ##
                    ##           Generic Command Menu                   ##
                    ##   1. Inquiry start                               ##
                    ##   2. Inquiry cancel                              ##
                    ##   3. Select device from Inquiry list             ##
                    ##   4. Sc menu                                     ##
                    ##   5. Local device info                           ##
                    ##   6. Get remote device info                      ##
                    ##   7. Scan mode                                   ##
                    ##   8. Link menu                                   ##
                    ##   9. Service Browse                              ##
                    ##   a. Set IO Settings                             ##
                    ##   s. Show Menu                                   ##
                    ##   r. Return to last menu                         ##
                    ##                                                  ##
                    ######################################################
    
	@手机端建议：
		1. iPhone手机推荐用第三方软件LightBlue，Android端用nRF Connect进行BLE测试。
    @工程说明
        - common工程支持的开发板有
            - ec-lb583
            - ec-lb587
            - eh-lb561
            - eh-lb563
            - eh-lb523
        - 编译方法: 进入hcpu目录执行命令`scons --board=<board_name> -j8`， 其中board_name为板子名称，例如编译eh-lb561板子，完整命令为`scons --board=eh-lb561 -j8`
          编译生成的image文件存放在HCPU的build_<board_name>目录下，common工程的用法参考通<<用工程构建方法>>
        对于使用NAND的开发板，如ec-lb583、ec-lb587和eh-lb563，第一次使用开发板还需执行HCPU工程开发板目录下的download_fs.bat烧写文件系统镜像到Flash中，否则启动后会提示文件系统mount失败，
        文件系统镜像只需要烧写一次，若重烧了文件系统，蓝牙地址也会丢失，需要重新配置。
           
   