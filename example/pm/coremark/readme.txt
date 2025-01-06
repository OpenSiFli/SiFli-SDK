@使用指南
    @介绍：
        处理器功耗测试例程，包括以下场景：
            - 一个核执行CoreMark基准测试程序
            - 一个核执行一段时间的while循环，循环中执行nop指令
            - 系统关机，可由RTC定时唤醒
            - 系统关机，由按键唤醒
        唤醒PIN在不同开发板上会有不同，对应关系如下
            - EH-LB551： 使用PA80，对应HDK板上外置蓝牙或GPS接口上的INT管脚
            - EH-LB555：使用PA79，对应底HDK板上的TP_INT
            - EC-LB58X：使用PA64
            - EH-LB561/EH-LB563：使用PB34，对应HDK板上的HR_INT
            - EH-ss6500/EH-LB52x： 使用PA24，对应HDK上的GPS_PEN
        当唤醒PIN为低电平时HCPU无法进入低功耗模式，
        此时可以通过console给HCPU发送命令执行指定任务，当唤醒PIN接高电平（即1.8V电压）时，HCPU进入低功耗模式，此时HCPU无法响应console命令。
        LCPU始终不进入低功耗模式，如果启动了LCPU，当不执行任务时LCPU处于WFI状态，可以响应来自console的命令，
        未启动LCPU时，则认为LCPU处于halt状态，无法处理console命令。            
            
        PC与底板使用USB Type-C线连接后会枚举出两个串口。HCPU使用UART1作为Console端口，LCPU使用UART4作为Console端口。

    @相关Console命令
        - HCPU和LCPU均支持
            - 'run_coremark freq_in_mhz':  修改主频并执行CoreMark，'freq_in_mhz'是以MHz为单位的频率，例如'run_coremark 48'，以48MHz主频执行CoreMark
            - 'run_while_loop freq_in_mhz':  修改主频并执行一段时间while loop，'freq_in_mhz'是以MHz为单位的频率，`run_while_loop 48`，以48MHz主频执行while loop
            - HCPU支持的主频为: 240MHZ/192MHz/144MHz/96MHz
            - LCPU支持的主频为: 48MHz/24MHz
        - HCPU支持
            - 'lcpu on': 启动LCPU，启动成功后LCPU可以接收console命令执行指定任务
            - 'shutdown [wakeup_time_in_sec]': 关机，'wakeup_time_in_sec'为可选参数，单位为秒，表示关机后多久自动开机，如果不带参数，则关机后只能被PIN唤醒，
                                               在551和555平台上，PIN为KEY1按键，按下即开机， 
                                               在557平台上，PIN对应J0804旁的SPI3_INT管脚，关机前该管脚需接高电平，关机后若接低电平则触发开机，
                                               在58X平台上，PIN对应KEY_HOME_RST按键，按下即开机，

    @工程说明
        - common工程支持的开发板有
            - eh-lb551
            - eh-lb555
            - ec-lb583
            - ec-lb587
            - eh-lb561
            - eh-lb563
            - eh-lb523
        - 编译方法: 进入hcpu目录执行命令`scons --board=<board_name> -j8`， 其中board_name为板子名称，例如编译eh-lb561板子，完整命令为`scons --board=eh-lb561 -j8`
          编译生成的image文件存放在HCPU的build_<board_name>目录下，common工程的用法参考通<<用工程构建方法>>