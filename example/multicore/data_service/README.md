# Data Service Example

（有关示例及其用法的总体概述，请参阅上级 “examples” 目录中的 `README.md` 文件。）

源码路径: example/multicore/data_service

这是一个演示多核系统中HCPU与LCPU之间数据服务通信的示例。其核心功能包括：
- 平台UART配置：55x平台HCPU使用UART1(第二个串口)、LCPU使用UART3(第一个串口)；58x平台HCPU使用UART1(第一个串口)、LCPU使用UART4(第三个串口)
- 服务通信机制：LCPU注册"test"服务，HCPU订阅该服务实现双向通信
- 命令交互流程：HCPU发送`request`命令获取带计数值的响应，LCPU发送`trigger`/`trigger2`命令触发带计数值的通知消息

该示例使用了SiFli-SDK的多核通信框架和服务注册订阅机制，可作为开发多核异构系统中进程间通信的基础参考。
## 用法

### 支持的开发板
此示例可在以下开发板上运行：
- eh-lb551
- eh-lb555
- ec-lb583
- ec-lb587
- eh-lb561
- eh-lb563

### 硬件需求
需要上述支持的开发板，无需额外硬件。

### 编译方法
进入hcpu目录执行以下命令编译：
```bash
scons --board=<board_name> -j8
```
编译生成的image文件存放在HCPU的build_<board_name>目录下。

## 工程实现细节
- 工程支持的开发板有
    - eh-lb551
    - eh-lb555
    - ec-lb583
    - ec-lb587
    - eh-lb561
    - eh-lb563
- 编译方法: 进入hcpu目录执行命令`scons --board=<board_name> -j8`， 其中board_name为板子名称，例如编译eh-lb561板子，完整命令为`scons --board=eh-lb561 -j8`
  编译生成的image文件存放在HCPU的build_<board_name>目录下，工程的用法参考通<<用工程构建方法>>          
- test service自定义的消息ID和结构定义在`src/common/test_service.h`
  - `test_service_data_rsp_t`: `MSG_SERVICE_TEST_DATA_RSP`消息体结构
  - `test_service_data_ntf_ind_t`: `MSG_SERVICE_DATA_NTF_IND`消息体结构
  - `test_service_data2_ind_t`: `MSG_SERVICE_TEST_DATA2_IND`消息体结构
- 代码实现位置
  - HCPU: `src/hcpu/main.c`
  - LCPU: `src/lcpu/main.c`

## 示例输出
通过控制台命令与服务交互：
- 在HCPU控制台发送`request`命令发送请求消息
- 在LCPU控制台发送`trigger`或`trigger2`命令触发通知消息

## 异常诊断
暂无特定异常诊断信息。如遇问题，请参考SiFli-SDK官方文档或提交issue。

## 参考文档
- [SiFli-SDK 官方文档](https://docs.sifli.com)
