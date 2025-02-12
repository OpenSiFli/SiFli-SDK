# IPC Queue示例
- 55x平台，HCPU使用UART1(枚举出的第二个串口)作为console，LCPU使用UART3作为console(枚举出的第一个串口)，
  58x平台，HCPU使用UART1(枚举出的第一个串口)作为console，LCPU使用UART4作为console(枚举出的第三个串口)，
- 在HCPU的console里发送命令`lcpu on`启动LCPU，启动成功后可以在LCPU的console上看到启动log
- HCPU和LCPU的console均可以使用命令`send message`发送字符串给另外一个核，
  `message`是需要发送的内容，如果字符串包含空格，需要使用双引号将字符串包起来，
  另外一个核则打印接收到的字符串。
  例如在HCPU的console里发送`send "Hello LCPU, this is HCPU"`，在LCPU的console出现打印`rx: Hello LCPU, this is HCPU`

## 工程说明
- common工程支持的开发板有
    - eh-lb551
    - eh-lb555
    - ec-lb583
    - ec-lb587
    - eh-lb561
    - eh-lb563
- 编译方法: 进入hcpu目录执行命令`scons --board=<board_name> -j8`， 其中board_name为板子名称，例如编译eh-lb561板子，完整命令为`scons --board=eh-lb561 -j8`
  编译生成的image文件存放在HCPU的build_<board_name>目录下，common工程的用法参考通<<用工程构建方法>>          
- HCPU和LCPU使用queue 2作为通信通道，HCPU给LCPU的发送buffer大小为256字节，LCPU给HCPU的发送buffer也为256字节，
  相应的宏定义在`/src/common/ipc_config.h`和各自工程的`linker_scripts/custom_mem_map.h`中，HCPU的主函数在`src/hcpu/main.c`， LCPU的主函数在`src/lcpu/main.c`
