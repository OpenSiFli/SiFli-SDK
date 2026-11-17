# FlashDB示例

源码路径：example/storage/flashdb

## 支持的平台
<!-- 支持哪些板子和芯片平台 -->
+ sf32lb52-nano系列
+ sf32lb52-lcd系列
+ sf32lb56-lcd系列
+ sf32lb58-lcd系列
+ sf32lb57-lcd系列

## 概述
<!-- 例程简介 -->
`FlashDB` 是一款超轻量级的嵌入式数据库，专注于提供嵌入式产品的数据存储方案。`FlashDB` 提供两种数据库模式：
+ 键值数据库 (`KVDB`)：是一种非关系数据库，它将数据存储为键值（Key-Value）对集合，其中键作为唯一标识符；
+ 时序数据库 (`TSDB`)：时间序列数据库。  

本例程演示FlashDB的配置使用，包含：
+ project/nand：KVDB/TSDB在Nand flash上配置使用。
+ project/nor：KVDB/TSDB在Nor flash上配置使用。


## 例程的使用
<!-- 说明如何使用例程，比如连接哪些硬件管脚观察波形，编译和烧写可以引用相关文档。
对于rt_device的例程，还需要把本例程用到的配置开关列出来，比如PWM例程用到了PWM1，需要在onchip菜单里使能PWM1 -->

### 硬件需求
运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](quick_start)）。

### menuconfig配置

1. 使能FlashDB：  
![FLASHDB](./assets/mc_flashdb.png)  
     ```{tip}
     FDB Mode:
     + `Use FAL Mode` : Using FAL storage mode
     + `Use File LIBC Mode` : Using file storage mode by LIBC file API, like fopen/fread/fwrte/fclose
     + `Use File POSIX Mode` : Using file storage mode by POSIX file API, like open/read/write/close  

     本例程中:
     + Nand 使用`FILE MODE`(`FDB Mode` 配置为`Use File POSIX Mode`), 通过文件系统存储。
     + Nor 使用`FAL MODE`(`FDB Mode` 配置为`Use FAL Mode`), 直接操作Flash。
     ```
2. 配置`FAT`文件系统（当`FDB Mode`配置为`Use File LIBC Mode`或`Use File POSIX Mode`时需要）   
![RT_USING_DFS_ELMFAT](./assets/mc_fat.png)

     ```{tip}
     `mnt_init` 中mount 文件系统分区，FDB初始化时需指定存储路径（在文件系统中的目录）。
     ```
3. FAL 分区配置（当`FDB Mode`配置为`Use FAL Mode`时需要）   
+ 典型工程可直接提供完整 `ptab.yaml`，也可以只提供 `ptab.overlay.yaml` 覆盖板级分区。  
+ 本例中以下 board variant 已改为工程级 overlay：
  - `project/nor/sf32lb56-lcd_n16r12n1_hcpu/ptab.overlay.yaml`
  - `project/nor/sf32lb58-lcd_n16r64n4_hcpu/ptab.overlay.yaml`

  ```yaml
  partitions:
    - op: add
      name: kvdb_tst
      type: data
      subtype: flashdb_kv
      region: mpi3
      offset: 0x00608000
      size: 0x00004000
      aliases:
        - KVDB_TST_REGION
    - op: add
      name: tsdb_tst
      type: data
      subtype: flashdb_kv
      region: mpi3
      offset: 0x0060C000
      size: 0x00004000
      aliases:
        - TSDB_TST_REGION
  ```

+ `sf32lb58-lcd_n16r64n4_hcpu` 的 overlay 还会额外调整 `hcpu_flash_code/fs_region/fs_ex_region/acpu`
  的 offset 或 size，以便插入 `kvdb_tst/tsdb_tst` 分区。

+ 对尚未迁到 v3 的工程，仍可沿用 board variant 目录下的 `ptab.json` / `custom_mem_map.h`。
+ 例如：
  - `project/nor/sf32lb52-lcd_n16r8_hcpu/ptab.json`
  - `project/nor/sf32lb58-lcd_n16r32n1_dsi_hcpu/custom_mem_map.h`
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
     }
     ```  

     ```{tip}
     FDB初始化时需指定Flash分区名（比如本例程中是"kvdb_tst"/"tsdb_tst"）。
     对于 v3 工程，`ptab.h` 会自动生成 `KVDB_TST_REGION_*` / `TSDB_TST_REGION_*` 兼容宏，以及对应的 `FAL_PART_TABLE` 条目，不再需要 `custom_mem_map.h`。
     ```

     ```{tip}
     如果使用 overlay，可以在工程目录执行
     `sdk.py ptab-export --board=sf32lb56-lcd_n16r12n1_hcpu`
     检查最终生效的 `ptab.effective.yaml`。
     ```

     ```{tip}
     `sf32lb52-lcd_n16r8_hcpu` 和 `sf32lb52-nano_n16r16_hcpu` 目前仍保留 `ptab.json`（v2），
     因为它们不仅修改分区，还修改了板级 `memory` 拓扑；这超出了当前分区级 overlay 的范围。
     ```

### 编译和烧录
切换到例程project/nand目录，运行scons命令执行编译：
```c
> scons --board=eh-lb525 -j32
```
切换到例程`project/nand/build_xx`目录，运行`uart_download.bat`，按提示选择端口即可进行下载：
```c
$ ./uart_download.bat

     Uart Download

please input the serial port num:5
```
关于编译、下载的详细步骤，请参考[快速上手](quick_start)的相关介绍。

```{tip}
project/nor 是对应nor方案，编译下载方式相同，区别是对应的board不同。
```
## 例程的预期结果
<!-- 说明例程运行结果，比如哪几个灯会亮，会打印哪些log，以便用户判断例程是否正常运行，运行结果可以结合代码分步骤说明 -->
本例程中通过FINSH命令来操作FlashDB：  
KVDB:
用途 | 命令 | 示例
|---|--|--|
设置kvdb|kvdb set [key] [数据类型:int\|str] [value]|`kvdb set "kv1" int 100` 
读取kvdb|kvdb get [key] [数据类型:int\|str]|`kvdb get "kv1" int`
删除kvdb|kvdb del [key]|`kvdb del "kv1"`

串口打印如下：  

```c
// 设置、读取整形数据
12-23 00:51:23:316 TX:kvdb set "key1" int 100
12-23 00:51:23:465    set the key1 value to 100
12-23 00:51:23:579    msh />
12-23 00:51:30:771 TX:kvdb get "key1" int
12-23 00:51:30:831    [key1] int
12-23 00:51:30:836    get the key1 value is 100 
// 设置、读取string
12-23 00:52:21:753 TX:kvdb set "key2" str "hello"
12-23 00:52:22:003    set key2 value to hello
12-23 00:52:22:115    msh />
12-23 00:52:29:612 TX:kvdb get "key2" str
12-23 00:52:29:672    [key2] str
12-23 00:52:29:677    get the key2 value is hello 
// 删除kvdb
12-23 00:53:16:528 TX:kvdb del "key1"
12-23 00:53:16:675    delete the key1 finish
12-23 00:53:16:788    msh />
12-23 00:53:20:062 TX:kvdb get "key1" int
12-23 00:53:20:120    [key1] int
12-23 00:53:20:147    get the key1 failed
```  
TSDB:
用途 | 命令 | 示例
|---|--|--|
增加tsdb|tsdb append [value]|`tsdb append 1` 
全部查询tsdb|tsdb query_all|`tsdb query_all`
按时间查询tsdb|tsdb query_by_time [from timestamp] [to timestamp]|`tsdb query_by_time 0 946686530`
清除tsdb|tsdb clear|`tsdb clear`

串口打印如下：  
```c
// clear tsdb
12-23 00:55:21:376 TX:tsdb clear
12-23 00:55:23:455    clear tsdb.
// 新增tsdb条目
12-23 00:55:56:845 TX:tsdb append 1
12-23 00:55:57:198    append tsdb item : value = 1
12-23 00:55:57:244    tsdb count is: 1
12-23 00:55:57:361    msh />
12-23 00:55:59:988 TX:tsdb append 2
12-23 00:56:00:134    append tsdb item : value = 2
12-23 00:56:00:162    tsdb count is: 2
12-23 00:56:00:278    msh />
12-23 00:56:01:521 TX:tsdb append 3
12-23 00:56:01:666    append tsdb item : value = 3
12-23 00:56:01:693    tsdb count is: 3
// 全部查询
12-23 00:56:39:698 TX:tsdb query_all
12-23 00:56:39:757    query all:
12-23 00:56:39:783    [query_cb] queried a TSL: value: 1 time: 946689062 Sat Jan  1 01:11:02 2000
12-23 00:56:39:788    [query_cb] queried a TSL: value: 2 time: 946689065 Sat Jan  1 01:11:05 2000
12-23 00:56:39:793    [query_cb] queried a TSL: value: 3 time: 946689067 Sat Jan  1 01:11:07 2000
// 按时间查询
12-23 00:57:04:317 TX:tsdb query_by_time 0 946689065
12-23 00:57:04:375    query by time:
12-23 00:57:04:380    from time:0 Thu Jan  1 00:00:00 1970
12-23 00:57:04:385    to time:946689065 Sat Jan  1 01:11:05 2000
12-23 00:57:04:391    [query_by_time_cb] queried a TSL: value: 1 time: 946689062 Sat Jan  1 01:11:02 2000
12-23 00:57:04:396    [query_by_time_cb] queried a TSL: value: 2 time: 946689065 Sat Jan  1 01:11:05 2000
12-23 00:57:04:402    query count is: 2
```
## 异常诊断


## 参考文档
<!-- 对于rt_device的示例，rt-thread官网文档提供的较详细说明，可以在这里添加网页链接，例如，参考RT-Thread的[RTC文档](https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/rtc/rtc) -->

+ siflisdk\external\FlashDB\README.md
+ siflisdk\external\FlashDB\README_zh.md

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |10/2024 |初始版本 |
| | | |
| | | |
