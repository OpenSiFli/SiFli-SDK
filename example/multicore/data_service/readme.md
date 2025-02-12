# Data Service示例
## 介绍：
   
- 55x平台，HCPU使用UART1(枚举出的第二个串口)作为console，LCPU使用UART3作为console(枚举出的第一个串口)，
58x平台，HCPU使用UART1(枚举出的第一个串口)作为console，LCPU使用UART4作为console(枚举出的第三个串口)，
- LCPU注册了名为"test"的service，HCPU订阅该service。
- 在HCPU的console中发送`request`命令发送消息MSG_SERVICE_TEST_DATA_REQ给test service，
  test service收到请求后回复MSG_SERVICE_TEST_DATA_RSP消息，消息中携带一个计数值，每次累加
- 在LCPU的console中发送`trigger`命令触发test service发送消息MSG_SERVICE_DATA_NTF_IND给client，
  消息内容携带一个计数值，每次累加
- 在LCPU的console中发送`trigger2`命令触发test service发送消息MSG_SERVICE_TEST_DATA2_IND给client，
  消息内容携带一个计数值，每次累加

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
- test service自定义的消息ID和结构定义在`src/common/test_service.h`，
  `test_service_data_rsp_t`为`MSG_SERVICE_TEST_DATA_RSP`消息体的结构，
  `test_service_data_ntf_ind_t`为`MSG_SERVICE_DATA_NTF_IND`消息体的结构，
  `test_service_data2_ind_t`为`MSG_SERVICE_TEST_DATA2_IND`消息体的结构，
- HCPU的代码实现在`src/hcpu/main.c`， LCPU的代码实现在`src/lcpu/main.c`
