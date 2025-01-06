# RPMsg-Lite示例
## 概述
本例程展示了RPMsg-Lite组件的基本用法


## 如何使用例程
- RPMsg-Lite使用queue4和queue5作为双向通信通道，HCPU作为master，LCPU作为remote，master的endpoint为30，remote的endpoint为40，
  共享buffer必须分配在LCPU的RAM中，地址由宏`RPMSG_BUF_ADDR_MASTER`指定，buffer大小由宏`RPMSG_BUF_SIZE`指定，
  这些宏定义在头文件`src\common\ipc_config.h`、`project\hcpu\custom_mem_map.h`和`project\lcpu\custom_mem_map.h`中
- 建议在`INIT_APP_EXPORT`阶段初始化RPMsg-Lite模块，避免过早打开mailbox中断对data_service模块产生影响
- HCPU的主函数在`src/hcpu/main.c`， LCPU的主函数在`src/lcpu/main.c`
- 编译方法参考通用工程编译，例如在`project\hcpu`目录下执行`scons --board=eh-lb551 -j8`编译运行在eh-lb551板子中的程序，其中--board后跟板子的名称，
  如果要编译555hdk的程序，则执行命令`scons --board=eh-lb555 -j8`，编译完成后使用`build_eh-lb551/download.bat`命令下载bin文件到板子中


## 例程的输出
- 根据开发板的文档选择HCPU和LCPU的log所对应的串口，有的开发板HCPU和LCPU各自使用一个串口输出log，有的开发板则复用同一个串口输出log
- 上电后HCPU自动调用lcpu_power_on启动LCPU，启动成功后可以在LCPU的console上看到开机log
- HCPU和LCPU自动定时发送消息给对方，LCPU收到消息后会打印`rx: hello_from_hcpu`，HCPU收到消息后会打印`rx: hello_from_lcpu`，
- 大核可以通过console命令`send message`发送字符串给另外一个核，`message`是需要发送的内容，如果字符串包含空格，需要使用双引号将字符串包起来，
  另外一个核则打印接收到的字符串。
  例如在HCPU的console里发送`send "Hello LCPU, this is HCPU"`，在LCPU的console出现打印`rx: Hello LCPU, this is HCPU`，
  为了演示睡眠功能，HCPU在执行了send命令后就会进入睡眠状态，LCPU也会随之睡眠下去，各自被定时器或者消息唤醒

