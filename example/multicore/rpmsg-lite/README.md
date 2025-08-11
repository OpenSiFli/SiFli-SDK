# RPMsg-Lite Example

(有关示例及其用法的总体概述，请参阅上级 "examples" 目录中的 `README.md` 文件。)

源码路径: example/multicore/rpmsg-lite

RPMsg-Lite示例展示了如何在SiFli-SDK中使用RPMsg-Lite组件实现HCPU与LCPU之间的双向通信。该示例使用SDK的RPMsg-Lite API，通过共享内存和mailbox中断机制实现跨核通信，适用于需要高效核间数据传输的多核心应用场景。

RPMsg-Lite是一种轻量级远程处理器消息传递协议，支持跨核间的异步通信，使用端点(endpoint)机制实现多通道通信隔离。
## 用法

### 支持的开发板
此示例可在以下开发板上运行：
- eh-lb551
- eh-lb555

下面的小节仅提供绝对必要的信息。有关配置SiFli-SDK及使用其构建和运行项目的完整步骤，请参阅 [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)。

### 硬件需求
本示例无需特殊硬件，可直接在支持的开发板上运行。

### 配置项目
- RPMsg-Lite使用queue4和queue5作为双向通信通道，HCPU作为master，LCPU作为remote，master的endpoint为30，remote的endpoint为40，
  共享buffer必须分配在LCPU的RAM中，地址由宏`RPMSG_BUF_ADDR_MASTER`指定，buffer大小由宏`RPMSG_BUF_SIZE`指定，
  这些宏定义在头文件`src\common\ipc_config.h`、`project\hcpu\custom_mem_map.h`和`project\lcpu\custom_mem_map.h`中
- 建议在`INIT_APP_EXPORT`阶段初始化RPMsg-Lite模块，避免过早打开mailbox中断对data_service模块产生影响
- HCPU的主函数在`src/hcpu/main.c`， LCPU的主函数在`src/lcpu/main.c`

### 编译和烧录
编译方法参考通用工程编译流程，在`project/hcpu`目录下执行：
```bash
scons --board=<board_name> -j8
```
其中board_name为板子名称，例如编译eh-lb551板子的程序：
```bash
scons --board=eh-lb551 -j8
```
编译完成后使用以下命令下载bin文件到板子中：
```bash
build_<board_name>/download.bat
```

### 控制台配置
- 根据开发板的文档选择HCPU和LCPU的log所对应的串口，有的开发板HCPU和LCPU各自使用一个串口输出log，有的开发板则复用同一个串口输出log
- 上电后HCPU自动调用lcpu_power_on启动LCPU，启动成功后可以在LCPU的console上看到开机log

## 示例输出
HCPU和LCPU自动定时发送消息给对方：
- LCPU收到消息后打印：`rx: hello_from_hcpu`
- HCPU收到消息后打印：`rx: hello_from_lcpu`
- 大核可以通过console命令`send message`发送字符串给另外一个核，`message`是需要发送的内容，如果字符串包含空格，需要使用双引号将字符串包起来，另外一个核则打印接收到的字符串。
```bash
# 在HCPU控制台发送消息
send "Hello LCPU, this is HCPU"
```
LCPU控制台将显示：
```
rx: Hello LCPU, this is HCPU
```

> 为演示睡眠功能，HCPU执行send命令后会进入睡眠状态，LCPU也会随之睡眠，两者均会被定时器或消息唤醒。

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- [多核通信开发指南](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/multicore/index.html)

