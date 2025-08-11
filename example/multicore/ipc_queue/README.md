# IPC Queue示例
源码路径: example/multicore/ipc_queue
IPC Queue示例展示了如何在SiFli-SDK中使用跨核队列（IPC Queue）实现HCPU与LCPU之间的双向通信。该示例使用了SDK的IPC队列管理API，支持核间消息的可靠传递，适用于需要高效核间通信的多核心应用场景。

## 用法
下面的小节仅提供绝对必要的信息。有关配置SiFli-SDK及使用其构建和运行项目的完整步骤，请参阅[SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)。

### 支持的开发板
此示例可在以下开发板上运行：
- eh-lb551
- eh-lb555
- ec-lb583
- ec-lb587
- eh-lb561
- eh-lb563

### 硬件需求
本示例无需特殊硬件，可直接在支持的开发板上运行。

### 编译和烧录
进入hcpu目录执行编译命令：
```bash
scons --board=<board_name> -j8
```
其中board_name为板子名称，例如编译eh-lb561板子，完整命令为：
```bash
scons --board=eh-lb561 -j8
```
编译生成的image文件存放在HCPU的build_<board_name>目录下，使用以下命令烧录：
```bash
build_<board_name>/download.bat
```

### 控制台配置
- 55x平台，HCPU使用UART1(枚举出的第二个串口)作为console，LCPU使用UART3作为console(枚举出的第一个串口)，
  58x平台，HCPU使用UART1(枚举出的第一个串口)作为console，LCPU使用UART4作为console(枚举出的第三个串口)，
- 在HCPU的console里发送命令`lcpu on`启动LCPU，启动成功后可以在LCPU的console上看到启动log
- HCPU和LCPU的console均可以使用命令`send message`发送字符串给另外一个核，
  `message`是需要发送的内容，如果字符串包含空格，需要使用双引号将字符串包起来，
  另外一个核则打印接收到的字符串。
  例如在HCPU的console里发送`send "Hello LCPU, this is HCPU"`，在LCPU的console出现打印`rx: Hello LCPU, this is HCPU`

## 异常诊断
- **编译错误**：确保已正确配置SiFli-SDK开发环境，检查板型名称是否正确。
- **烧录失败**：确认开发板已正确连接，尝试重新插拔USB线缆。
- **通信异常**：检查IPC队列配置是否正确，确认宏定义在`src/common/ipc_config.h`和`linker_scripts/custom_mem_map.h`中的设置。

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [多核通信开发指南](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/multicore/index.html)
- [IPC队列API文档](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/api/ipc_queue.html)
