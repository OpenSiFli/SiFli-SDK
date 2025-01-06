@使用指南
    @介绍：
        BLE广播与连接场景功耗测试例程（LCPU主频为24MHz）。
        唤醒PIN在不同开发板上会有不同，对应关系如下
            - EH-LB551：使用PA80，对应HDK板上外置蓝牙或GPS接口上的INT管脚
            - EH-LB555：使用PA79，对应底HDK板上的TP_INT
        当唤醒PIN为低电平时HCPU和LCPU均无法进入低功耗模式，
        此时可以通过Console给LCPU发送命令修改参数，当唤醒PIN接高电平（即1.8V电压）时，
        HCPU进入低功耗模式，LCPU则周期性的进出低功耗模式，此时Console无法使用。
        PC与HDk使用USB Type-C线连接后会枚举出两个串口，HCPU使用第二个串口作为Console端口，LCPU使用第一个串口作为Console端口。

    @相关Console命令
        - HCPU
            1. 出厂化BLE相关Flash数据：nvds reset_all 1
                    - 为避免Flash冲突，第一次使用最好先下该命令。
            2. 设置蓝牙MAC地址：nvds update addr 6 [addr]. Example: nvds update addr 6 2345670123C3
        - LCPU
            - 'ble_config adv interval_in_ms': 修改广播周期，其中'interval_in_ms'是毫秒为单位的广播间隔
            - 'ble_config conn interval_in_ms': 修改连接周期，其中'interval_in_ms'是毫秒为单位的连接间隔，
              需要在与手机连接后发送命令
	@手机端建议：
		1. iPhone手机推荐用第三方软件LightBlue，Android端用nRF Connect进行BLE测试。
    @工程说明
        - common工程支持的开发板有
            - eh-lb551
            - eh-lb555
        - 编译方法: 进入hcpu目录执行命令`scons --board=<board_name> -j8`， 其中board_name为板子名称，例如编译eh-lb551板子，完整命令为`scons --board=eh-lb551 -j8`
          编译生成的image文件存放在HCPU的build_<board_name>目录下，common工程的用法参考通<<用工程构建方法>>

        - LCPU的主频为24MHz，可以修改LCPU工程目录下的'board/bf0_ap_hal_msp.c'文件，将函数HAL_MspInit中的语句'HAL_RCC_LCPU_SetDiv(2, 0, 3)'去掉，
          这样就会保持HAL_PreInit设置的48MHz频率。
        - 32K晶振默认配置: EH-LB551不使用，EH-LB555使用. 相关的配置修改方法: menuconfig -->select board peripherals -->Low power crystal disabled
