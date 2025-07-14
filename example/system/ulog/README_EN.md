# Log Usage Example

## Supported Platforms
The example can run on the following development boards.
- sf32lb52-lcd_n16r8
- sf32lb58-lcd_n16r64n4

## Overview
- Logging is the output of software runtime status, process, and other information to different media (such as: files, console, display, etc.) for display and storage. It provides reference basis for problem tracing, performance analysis, system monitoring, fault warning, and other functions during software debugging and maintenance.

## ulog Function Configuration
- ulog configuration options are described below. You can configure according to actual functional requirements; generally, the default configuration can be used. Enter `menuconfig` in the compilation interface and configure under `RTOS→ RT-Thread Components→ Utilities`.
```
[*] Enable ulog                   /* Enable ulog */
      The static output log level./* Select static log output level. After selection, logs lower than the set level (specifically referring to logs using LOG_X API) will not be compiled into ROM */
[ ]   Enable ISR log.             /* Enable interrupt ISR log, i.e., log output API can also be used in ISR */
[*]   Enable assert check.        /* Enable assert check. When disabled, assert logs will not be compiled into ROM */
(128) The log's max width.        /* Maximum log length. Since ulog's log API operates on a line basis, this length also represents the maximum length of one log line */
[ ]   Enable async output mode.   /* Enable asynchronous log output mode. After enabling this mode, logs will not be output to the backend immediately, but will be cached first, then handed over to the log output thread (e.g., idle thread) for output */
      log format  --->            /* Configure log format, such as: time information, color information, thread information, floating point support, etc. */
[*]   Enable console backend.     /* Enable console as backend. After enabling, logs can be output to console serial port. It is recommended to keep enabled. */
[ ]   Enable file backend.        /* ulog file backend */
[ ]   Enable runtime log filter.  /* Enable runtime log filter, i.e., dynamic filtering. After enabling, logs will support dynamic filtering by tags, keywords, etc. during system runtime. */
[*]   Enable syslog format log and API.  /* Enable syslog format log and API */
```
* Enter `menuconfig` in the compilation interface and configure log format options under `TOS → RT-Thread Components → Utilities → Enable ulog → log format` as described below:
```
[ ] Enable float number support. It will using more thread stack.   /* Floating point number support (traditional rtdbg/rt_kprintf do not support floating point logs) */
[*] Enable color log.                   /* Colored logs */
[*] Enable time information.            /* Time information */
[ ]   Enable timestamp format for time. /* Include timestamp */
[*] Enable level information.           /* Level information */
[*] Enable tag information.             /* Tag information */
[ ] Enable thread information.          /* Thread information */

```


## Usage Example
- The following introduces using the ulog example, defining the `LOG_TAG` macro.
```c
#define LOG_TAG     "example"     // The tag corresponding to this module. If not defined, default: NO_TAG
#define LOG_LVL     LOG_LVL_DBG   // The log output level corresponding to this module. If not defined, default: debug level
#include <ulog.h>                 // Must be below LOG_TAG and LOG_LVL
```
The `int ulog_test()` function uses `LOG_X API`, roughly as follows:
```c
/* output different level log by LOG_X API */
LOG_D("LOG_D(%d): RT-Thread is an open source IoT operating system from China.", count);
LOG_I("LOG_I(%d): RT-Thread is an open source IoT operating system from China.", count);
LOG_W("LOG_W(%d): RT-Thread is an open source IoT operating system from China.", count);
LOG_E("LOG_E(%d): RT-Thread is an open source IoT operating system from China.", count);
```
Enter `ulog_test` and the serial port output result is shown below:  
![alt text](assets/ulog1.png)       

## Usage in Interrupt ISR
- Often logs need to be output in interrupt ISR, but interrupt ISR may interrupt threads that are outputting logs. To ensure interrupt logs and thread logs do not interfere with each other, special handling for interrupt situations is needed.
ulog has integrated interrupt log functionality, but it is not enabled by default. When using, simply enable the `Enable ISR log` option, and the log API usage is consistent with thread usage.

## Setting Log Format
- ulog supported log formats can be configured in `menuconfig`, located at `RTOS → RT-Thread Components → Utilities → Enable ulog → log format`, specific configuration as follows 
![alt text](assets/menuconfig.png)        

- You can configure: floating point number support, colored logs, time information (including timestamp), level information, tag information, thread information. Let's select all these options, save, recompile, and run the ulog example again to see the actual effect:  
![alt text](assets/ulog2.png)       

## hexdump Output Usage
- `hexdump` is also a commonly used function for log output. Through `hexdump`, you can output a section of data in hex format. See the specific usage and runtime effect below:
```c
/* Define an array of 128 bytes length */
uint8_t i, buf[128];
/* Fill the array with numbers */
for (i = 0; i < sizeof(buf); i++)
{
    buf[i] = i;
}
/* Dump the data in the array in hex format, width 16 */
ulog_hexdump("buf_dump_test", 16, buf, sizeof(buf));
```
- Run the ulog example again to see the actual effect:    
![alt text](assets/ulog3.png)       

## Dynamic Log Filter
- Static filtering has advantages such as resource saving, but often users need to dynamically adjust log filtering methods during software runtime. This is where ulog's dynamic filter functionality can be used. Using dynamic filter functionality requires enabling the `Enable runtime log filter` option in `menuconfig`. This option is disabled by default.
ulog supports the following four dynamic filtering methods.
- Filter by module level  

|Parameter |Description    |   
|:---|:---| 
|tag |Log tag  |   
| level|Set log level  |  
|Return |--   |   
|>=0| Success |    
|-5  |Failure, insufficient memory |    
  -  Command format: `ulog_tag_lvl <tag><level>`.     
The module here refers to a class of log code with the same tag attributes. Sometimes it's necessary to dynamically modify a module's log output level during runtime.    
Parameter level log levels can take the following values: 

      |Level |Name |Value |
      |:---|:---|:---|
      |LOG-LVL_ASSERT |Assert |0 |
      |LOG_LVL_ERROR |Error |3 |
      |LOG_LVL_WARNING |Warning   |4 |
      |LOG_LVL_INFO|Info|6|
      |LOG_LVL_DBG |Debug |7|
      |LOG_FILTER_LVL_SILENT |Silent |0|
      |LOG_FILTER_LVL_ALL |All | 7|

- Global filter by tag  
  - Command format: `ulog_tag [tag]`, when tag is empty, tag filtering is cancelled.      
This filtering method can perform tag filtering on all logs, only logs containing tag information are allowed to output.    
For example: with logs of tags `wifi.driveer`, `wifi.mgnt`, `aydio.driver`, when filter tag is set to `wifi`, only logs with tags `wifi.driver` and `wifi.mgnt` will output.
- Global filter by level  
  - Command format: `ulog_lvl <level>`, level values refer to above.      
After setting the global filter level through function or command, logs below the set level will stop outputting.
- Global filter by keyword
  - Command format: `ulog_kw [keyword]`, when keyword is empty, keyword filtering is cancelled.   
This filtering method can perform keyword filtering on all logs, only logs containing keyword information are allowed to output.    
- View filter information  
After setting filter parameters, if you want to view current filter information, you can enter the `ulog_filter` command, roughly as shown below:   
![alt text](assets/ulog_filter.png)    



## Reference Documentation
* 
## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |1/2025 |Initial version |
| | | |