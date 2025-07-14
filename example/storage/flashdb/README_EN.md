# FlashDB Example

Source path: example/storage/flashdb

## Supported Platforms
<!-- Which boards and chip platforms are supported -->
+ sf32lb52-lcd_n16r8
+ eh-lb523

## Overview
<!-- Example introduction -->
This example demonstrates the configuration and usage of FlashDB, including:
+ project/nand: KVDB/TSDB configuration and usage on Nand flash (`FDB_USING_FILE_MODE`).
+ project/nor: KVDB/TSDB configuration and usage on Nor flash (`FDB_USING_FAL_MODE`).


## Example Usage
<!-- Explain how to use the example, such as which hardware pins to connect to observe waveforms, compilation and flashing can reference related documentation.
For rt_device examples, you also need to list the configuration switches used by this example, such as PWM example using PWM1, which needs to be enabled in the onchip menu -->

### Hardware Requirements
Before running this example, you need to prepare:
+ One development board supported by this example ([Supported platforms](quick_start)).

### menuconfig Configuration

1. Enable FlashDB:
![FLASHDB](./assets/mc_flashdb.png)  
     ```{tip}
     + Nand uses `FILE MODE`, operates through file system, `FDB Mode` configured as `PKG_FDB_USING_FILE_POSIX_MODE`.
     + Nor uses `FAL MODE`, operates flash directly, `FDB Mode` configured as `PKG_FDB_USING_FAL_MODE`.
     ```
2. Configure `FAT` file system (when using `FILE MODE`, `FAL MODE` does not require file system configuration)   
![RT_USING_DFS_ELMFAT](./assets/mc_fat.png)

     ```{tip}
     Mount root partition in mnt_init.
     ```
3. FAL partition configuration (when using `FAL MODE`, `FILE MODE` does not require independent partition configuration)   
+ `project/nor/ptab.json`:
     ```c
            {
                "offset": "0x00620000", 
                "max_size": "0x00004000", 
                "tags": [
                    "KVDB_TST_REGION"
                ]
            }, 
            {
                "offset": "0x00624000", 
                "max_size": "0x00004000", 
                "tags": [
                    "TSDB_TST_REGION"
                ]
            }, 
     ```  
+ `project/nor/custom_mem_map.h`
     ```c
     #define FAL_PART_TABLE \
     { \
          {FAL_PART_MAGIC_WORD,       "kvdb_tst",      NOR_FLASH2_DEV_NAME,    KVDB_TST_REGION_OFFSET,   KVDB_TST_REGION_SIZE, 0}, \
          {FAL_PART_MAGIC_WORD,       "tsdb_tst",      NOR_FLASH2_DEV_NAME,    TSDB_TST_REGION_OFFSET,   TSDB_TST_REGION_SIZE, 0}, \
          ... ...
     }
     ``` 

### Compilation and Programming
Switch to the example project/nand directory and run the scons command to execute compilation:
```c
> scons --board=eh-lb525 -j32
```
Switch to the example `project/nand/build_xx` directory and run `uart_download.bat`, select the port as prompted to download:
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
For detailed steps on compilation and downloading, please refer to the relevant introduction in [Quick Start](quick_start).

```{tip}
project/nor is the corresponding nor solution, compilation and download methods are the same, the difference is the corresponding board.
```
## Expected Results
<!-- Explain the example running results, such as which LEDs will light up, what logs will be printed, so that users can judge whether the example is running normally. The running results can be explained step by step combined with the code -->
This example operates FlashDB through FINSH commands:  
KVDB:
Purpose | Command | Example
|---|--|--|
Set kvdb|kvdb set [key] [data type:int\|str] [value]|`kvdb set "kv1" int 100` 
Read kvdb|kvdb get [key] [data type:int\|str]|`kvdb get "kv1" int`
Delete kvdb|kvdb del [key]|`kvdb del "kv1"`

Serial port output as follows:  

```c
// Set, read integer data
12-23 00:51:23:316 TX:kvdb set "key1" int 100
12-23 00:51:23:353    kvdb set "key1" int 100
12-23 00:51:23:465    set the key1 value to 100
12-23 00:51:23:579    msh />
12-23 00:51:30:771 TX:kvdb get "key1" int
12-23 00:51:30:827    kvdb get "key1" int
12-23 00:51:30:831    [key1] int
12-23 00:51:30:836    get the key1 value is 100 
// Set, read string
12-23 00:52:21:753 TX:kvdb set "key2" str "hello"
12-23 00:52:21:810    kvdb set "key2" str "hello"
12-23 00:52:22:003    set key2 value to hello
12-23 00:52:22:115    msh />
12-23 00:52:29:612 TX:kvdb get "key2" str
12-23 00:52:29:667    kvdb get "key2" str
12-23 00:52:29:672    [key2] str
12-23 00:52:29:677    get the key2 value is hello 
// Delete kvdb
12-23 00:53:16:528 TX:kvdb del "key1"
12-23 00:53:16:585    kvdb del "key1"
12-23 00:53:16:675    delete the key1 finish
12-23 00:53:16:788    msh />
12-23 00:53:20:062 TX:kvdb get "key1" int
12-23 00:53:20:116    kvdb get "key1" int
12-23 00:53:20:120    [key1] int
12-23 00:53:20:147    get the key1 failed
```  
TSDB:
Purpose | Command | Example
|---|--|--|
Add tsdb|tsdb append [value]|`tsdb append 1` 
Query all tsdb|tsdb query_all|`tsdb query_all`
Query tsdb by time|tsdb query_by_time [from timestamp] [to timestamp]|`tsdb query_by_time 0 946686530`
Clear tsdb|tsdb clear|`tsdb clear`

Serial port output as follows:  
```c
// clear tsdb
12-23 00:55:21:376 TX:tsdb clear
12-23 00:55:21:430    tsdb clear
12-23 00:55:23:455    clear tsdb.
// Add tsdb entries
12-23 00:55:56:845 TX:tsdb append 1
12-23 00:55:56:902    tsdb append 1
12-23 00:55:57:198    append tsdb item : value = 1
12-23 00:55:57:244    tsdb count is: 1
12-23 00:55:57:361    msh />
12-23 00:55:59:988 TX:tsdb append 2
12-23 00:56:00:045    tsdb append 2
12-23 00:56:00:134    append tsdb item : value = 2
12-23 00:56:00:162    tsdb count is: 2
12-23 00:56:00:278    msh />
12-23 00:56:01:521 TX:tsdb append 3
12-23 00:56:01:577    tsdb append 3
12-23 00:56:01:666    append tsdb item : value = 3
12-23 00:56:01:693    tsdb count is: 3
// Query all
12-23 00:56:39:698 TX:tsdb query_all
12-23 00:56:39:753    tsdb query_all
12-23 00:56:39:757    query all:
12-23 00:56:39:783    [query_cb] queried a TSL: value: 1 time: 946689062 Sat Jan  1 01:11:02 2000
12-23 00:56:39:788    [query_cb] queried a TSL: value: 2 time: 946689065 Sat Jan  1 01:11:05 2000
12-23 00:56:39:793    [query_cb] queried a TSL: value: 3 time: 946689067 Sat Jan  1 01:11:07 2000
// Query by time
12-23 00:57:04:317 TX:tsdb query_by_time 0 946689065
12-23 00:57:04:371    tsdb query_by_time 0 946689065
12-23 00:57:04:375    query by time:
12-23 00:57:04:380    from time:0 Thu Jan  1 00:00:00 1970
12-23 00:57:04:385    to time:946689065 Sat Jan  1 01:11:05 2000
12-23 00:57:04:391    [query_by_time_cb] queried a TSL: value: 1 time: 946689062 Sat Jan  1 01:11:02 2000
12-23 00:57:04:396    [query_by_time_cb] queried a TSL: value: 2 time: 946689065 Sat Jan  1 01:11:05 2000
12-23 00:57:04:402    query count is: 2
```
## Exception Diagnosis


## Reference Documentation
<!-- For rt_device examples, the RT-Thread official website documentation provides more detailed explanations, you can add webpage links here, for example, refer to RT-Thread's [RTC documentation](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

+ siflisdk\external\FlashDB\README.md
+ siflisdk\external\FlashDB\README_zh.md

## Update Log
|Version |Date   |Release Notes |
|:---|:---|:---|
|0.0.1 |10/2024 |Initial version |
| | | |
| | | |