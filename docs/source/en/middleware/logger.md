
# Logger

SIFLI SDK uses RT-Thread's `ulog` as the logging (log) output mechanism. As a concise and easy-to-use logging output component, `ulog` supports:
- Logs with different output levels.
- Output by module.
- Thread safety with options for synchronous or asynchronous log output.
- Configurable output format.
- Configurable output backends, which can be UART or flash.

For specific design architecture and implementation methods, you can refer to the [ulog documentation](https://www.rt-thread.org/document/site/programming-manual/ulog/ulog).

## ulog Configuration

`ulog` can be configured through many options in `menuconfig`, including log output levels, synchronous or asynchronous output, and custom output formats.

![Logger Configuration]( ../../assets/logger_config.png)

## ulog Usage Example

```c
// Define module name for output level and format.
#define LOG_TAG  "app"
// Define the module's minimum output level as info
#define LOG_LVL  LOG_LVL_INFO

void app_log_demo(void)
{
    uint8_t hex_data[] = {0x01, 0x02, 0x03, 0x04, 0x05, ... , 0xFF};

    // All logs belong to the 'app' module
    LOG_D("Debug log"); // Output debug level log, but it won't output because the 'app' module is set to 'info' level.
    LOG_I("Info log");  // Output info level log
    LOG_W("Warning log"); // Output warning level log
    LOG_E("Error log"); // Output error level log
    LOG_HEX("hex_data", 16, hex_data, sizeof(hex_data)); // Output hex data
}
```

