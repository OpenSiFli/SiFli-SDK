# 日志

SIFLI SDK使用RT-Thread的ulog作为日志(log)输出机制。作为简洁易用的日志输出组件，ulog支持
	- 不同级别的输出日志级别
	- 按模块进行输出
	- 线程安全并可以选择同步或异步的日志输出方式。
	- 配置输出格式
	- 配置不同的输出后端，可以是UART或者flash。

具体的设计架构及实现方式，可以参考<a href="https://www.rt-thread.org/document/site/programming-manual/ulog/ulog">ulog日志</a> 

## ulog配置

ulog可以在menuconfig里面进行众多选项的配置，除了日志输出级别，同步异步等方式外，还可以自定义输出格式。

![](../../assets/logger_config.png)

## ulog使用示例

```c

// Define module name for output level and format.
#define LOG_TAG  "app"
// Define the module minimum output level as info
#define LOG_LVL  LOG_LVL_INFO

void app_log_demo(void)
{
	uing8_t hex_data = {0x01, 0x02, 0x03, 0x04, 0x05, ... ,0xFF};
	
	// All logs are belongs to module 'app'
	LOG_D("Debug log"); // Output debug level log, but it could not output due to module 'app' set the log level as info.
	LOG_I("Info log");  // Output info level log
	LOG_W("Warning log"); // Output warning level log
	LOG_E("Error log"); // Output error level log
	LOG_HEX("hex_data", 16, hex_data, sizeof(hex_data)); // Output hex data
}

```

