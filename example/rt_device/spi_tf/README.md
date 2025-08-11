# SPI TF卡文件系统示例
本示例展示了如何在 SF32LB52x 平台上通过 SPI 接口对 TF 卡进行文件系统性能测试。该示例实现了实时速度监控、缓冲区优化和详细的性能分析功能。

本示例使用了 SiFli-SDK 的以下功能：
- **RT-Thread 文件系统**：使用 DFS（设备文件系统）框架进行文件操作 - [API参考](https://www.rt-thread.org/document/site/programming-manual/filesystem/filesystem/)
- **SPI 驱动**：通过 SPI 总线与 SD 卡通信 - [API参考](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/drivers/spi.html)
- **FAT 文件系统**：使用 ELM FAT 文件系统进行数据存储
- **HAL 层接口**：使用硬件抽象层进行底层硬件控制

基于此示例，可以创建以下应用：
- 数据记录器应用（如传感器数据存储）
- 多媒体文件存储系统
- 嵌入式数据库应用
- 固件更新存储方案
- 大容量数据缓存系统

## 用法

下面的小节仅提供绝对必要的信息。有关配置 SiFli-SDK 及使用其构建和运行项目的完整步骤，请参阅 [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)。

## 支持的开发板

此示例已验证可在以下开发板上运行：
- sf32lb52-lcd_n16r8
- sf32lb52-lcd_52d

### 硬件需求

1. **开发板**：任意支持的 SF32LB52x 开发板
2. **SD 卡**：
   - Micro SD 卡（建议容量 2GB 以上）
   - 速度等级：Class 10 或 UHS-I
   - 文件系统格式：FAT32
3. **连接方式**：
   - SD 卡通过板载 Micro SD 卡槽连接
   - 使用 SPI1 总线（已在板上连接）

**硬件连接说明**：
| SD卡引脚 | 功能 | 开发板引脚 |
|---------|------|-----------|
| CS      | 片选 | 自动配置  |
| MOSI    | 数据输入 | SPI1_MOSI |
| MISO    | 数据输出 | SPI1_MISO |
| CLK     | 时钟 | SPI1_CLK  |
| VDD     | 电源 | 3.3V     |
| GND     | 地   | GND      |

### 软件需求

- RT-Thread 操作系统（已集成在 SDK 中）
- 串口终端软件（如 PuTTY、SecureCRT 等，用于查看输出和输入命令）


## 示例输出

如果您看到以下控制台输出，则示例应已正确运行：

```
SFBL
Serial:c2,Chip:4,Package:4,Rev:7  Reason:00000000

[I/drv.adc] Get ADC configure fail


 \ | /
- SiFli Corporation
 / | \     build on Jul 18 2025, 2.4.0 build 00000000
 2020 - 2022 Copyright by SiFli team
mount /dev sucess
[BUS]spi1 probe sdcard...
[MSD] 1006 [err] wait ready timeout!

[MSD] 1006 [info] SD card goto IDLE mode OK!

[MSD] 1007 [info] CMD8 response : 0x01 0xF0 0x00 0x01 0xAA

[MSD] 1008 [info] Ver2.00 or later or SDHC or SDXC memory card!

[MSD] 1009 SD_V2: READ_OCR
[MSD] 1009 response:1,0,ff,80
[MSD] 1010 [info] OCR is 0x00FF8000

[MSD] 1041 SD_V2 again: READ_OCR
[MSD] 1041 [info] OCR 2nd read is 0xC0FF8000

[MSD] 1042 [info] It is SD2.0 SDHC Card!!!

[MSD] 1044 [info] CSD Version 2.0

[MSD] 1044 [info] TRAN_SPEED: 0x32, 10Mbit/s.

[MSD] 1045 [info] CSD : C_SIZE : 60719

[MSD] 1045 [info] card capacity : 29.64 Gbyte

[MSD] 1046 [info] sector_count : 62177280

[SD]msd init ok
find sd0 ok ! 2000d1ac
[I/drv.rtc] PSCLR=0x80000100 DivAI=128 DivAF=0 B=256
[I/drv.rtc] RTC use LXT RTC_CR=00000001

[I/drv.rtc] Init RTC, wake = 0

[I/drv.audprc] init 00 ADC_PATH_CFG0 0x606

[I/drv.audprc] HAL_AUDPRC_Init res 0

[I/drv.audcodec] HAL_AUDCODEC_Init res 0

[I/TOUCH] Regist touch screen driver, probe=120260f5 
call par CFG1(3313)

fc 9, xtal 2000, pll 2095

call par CFG1(3313)

fc 7, xtal 2000, pll 1676

fal_mtd_msd_device_create dev:sd0 part:root offset:0x0, size:0xfa000
fal_mtd_msd_device_create dev:sd0 part:misc offset:0xfa000, size:0xfa000
mount fs on flash to root success
mount fs on flash to FS_MSIC success

========== SD Card File System Performance Test ==========
SF32LB52x SD Card Test Program
Tick Per Second: 1000
Optimal Buffer Size: 64 KB
SD Operation Interval: 5 ms

Use 'help' to see available commands
Quick commands:
  fs_write /test.dat 16         - Test write speed (16MB)
  fs_read /test.dat 16          - Test read speed (16MB)
  sd_optimize                   - Check SD configuration
  buffer_optimize               - Test different buffer sizes
  fs_speed_test 32              - Complete speed test (32MB)
=========================================================

msh />
```

此时用户需要通过串口终端输入命令与示例交互。例如，输入 `fs_write /test.dat 16` 测试写入 16MB 文件的速度。
### Quick commands中命令的使用方法
#### fs_write /test.dat <...>
在<...>位置输入一个数字，测试写入该数字（以MB为单位）大小的文件的速度，例如
`fs_write /test.dat 32`
```
========== Write Speed Test ==========
File: /test.dat
Size: 32 MB
Buffer: 64 KB
=====================================

[00:01] Write: 1.15 MB/s (Avg: 1.15 MB/s) - 1.2/32.0 MB (3.7%)
[00:02] Write: 1.14 MB/s (Avg: 1.14 MB/s) - 2.4/32.0 MB (7.4%)
[00:03] Write: 1.13 MB/s (Avg: 1.14 MB/s) - 3.6/32.0 MB (11.1%)
[00:04] Write: 1.13 MB/s (Avg: 1.14 MB/s) - 4.8/32.0 MB (14.8%)
<--  省略过程  -->
[00:26] Write: 1.14 MB/s (Avg: 1.14 MB/s) - 29.6/32.0 MB (92.4%)
[00:27] Write: 1.13 MB/s (Avg: 1.14 MB/s) - 30.8/32.0 MB (96.1%)
[00:28] Write: 1.15 MB/s (Avg: 1.14 MB/s) - 31.9/32.0 MB (99.8%)

----- Write Test Results -----
Total bytes: 33554432 (32 MB)
Total time: 28.16 seconds
Average speed: 1.14 MB/s
------------------------------

```
#### fs_read /test.dat <...>
在<...>位置输入一个数字，测试读取该数字（以MB为单位）大小的文件的速度。例如：
`fs_read /test.dat 8`
```
========== Read Speed Test ==========
File: /test.dat
Size: 8 MB
Buffer: 64 KB
====================================

[00:01] Read: 1.20 MB/s (Avg: 1.20 MB/s) - 1.3/8.0 MB (15.6%)
[00:02] Read: 1.20 MB/s (Avg: 1.20 MB/s) - 2.5/8.0 MB (31.3%)
[00:03] Read: 1.20 MB/s (Avg: 1.20 MB/s) - 3.8/8.0 MB (46.9%)
[00:04] Read: 1.20 MB/s (Avg: 1.20 MB/s) - 5.0/8.0 MB (62.5%)
[00:05] Read: 1.20 MB/s (Avg: 1.20 MB/s) - 6.3/8.0 MB (78.1%)
[00:06] Read: 1.20 MB/s (Avg: 1.20 MB/s) - 7.5/8.0 MB (93.8%)

----- Read Test Results -----
Total bytes: 8388608 (8 MB)
Total time: 6.67 seconds
Average speed: 1.20 MB/s
-----------------------------
```
#### sd_optimize
此为检查SD配置项，该项会列出本示例所运行的环境的主要参数，效果如下：
```
========== SD Card Configuration ==========
SD Card Type: SDHC
SD Card Max Clock: 10 MHz (from CSD register)
Block Size: 512 bytes
Sector Count: 62177280
Capacity: 1688 MB

--- SPI Status ---
SPI Device Flags: 0x0F13
  DMA RX: ENABLED
  DMA TX: ENABLED

SPI Configuration:
  Data Width: 8 bits
  Current SPI Hz: 12 MHz
  Mode: 0x07
  Effective Freq: 10 MHz (limited by SD card)   //该项意味着，如果SD卡频率高于SPI频率，则有效频率为SPI频率，如果低于，则标明速度(limited by SD card)受限于SD卡。

Buffer Address: 0x60000000
Buffer Size: 64 KB
===========================================

```
#### buffer_optimize
此为测试不同的缓冲区大小的选项，可以分别测试4KB-512KB时的读写速度，并显示出最优结果，给出参数修改建议，效果如下：
```
========== Buffer Size Optimization Test ==========
Testing with proper SD card rest intervals...
Each test uses isolated files and adequate rest time.
====================================================
[1/8] Testing 4 KB buffer:
  SD card resting (3 seconds)...
  Write test (/buftest_4k_0.dat)...
    Write: 0.92 MB/s [NEW BEST]
  Read test...
    Read: 1.13 MB/s [NEW BEST]
  Summary: Write 0.92 MB/s, Read 1.13 MB/s

[2/8] Testing 8 KB buffer:
  SD card resting (3 seconds)...
  Write test (/buftest_8k_1.dat)...
    Write: 1.04 MB/s [NEW BEST]
  Read test...
    Read: 1.16 MB/s [NEW BEST]
  Summary: Write 1.04 MB/s, Read 1.16 MB/s

[3/8] Testing 16 KB buffer:
  SD card resting (3 seconds)...
  Write test (/buftest_16k_2.dat)...
    Write: 1.10 MB/s [NEW BEST]
  Read test...
    Read: 1.19 MB/s [NEW BEST]
  Summary: Write 1.10 MB/s, Read 1.19 MB/s

[4/8] Testing 32 KB buffer:
  SD card resting (3 seconds)...
  Write test (/buftest_32k_3.dat)...
    Write: 1.14 MB/s [NEW BEST]
  Read test...
    Read: 1.20 MB/s [NEW BEST]
  Summary: Write 1.14 MB/s, Read 1.20 MB/s

[5/8] Testing 64 KB buffer:
  SD card resting (3 seconds)...
  Write test (/buftest_64k_4.dat)...
    Write: 1.14 MB/s
  Read test...
    Read: 1.20 MB/s
  Summary: Write 1.14 MB/s, Read 1.20 MB/s

[6/8] Testing 128 KB buffer:
  SD card resting (3 seconds)...
  Write test (/buftest_128k_5.dat)...
    Write: 1.14 MB/s [NEW BEST]
  Read test...
    Read: 1.20 MB/s
  Summary: Write 1.14 MB/s, Read 1.20 MB/s

[7/8] Testing 256 KB buffer:
  SD card resting (3 seconds)...
  Write test (/buftest_256k_6.dat)...
    Write: 1.14 MB/s
  Read test...
    Read: 1.20 MB/s
  Summary: Write 1.14 MB/s, Read 1.20 MB/s

[8/8] Testing 512 KB buffer:
  SD card resting (3 seconds)...
  Write test (/buftest_512k_7.dat)...
    Write: 1.15 MB/s [NEW BEST]
  Read test...
    Read: 1.21 MB/s [NEW BEST]
  Summary: Write 1.15 MB/s, Read 1.21 MB/s


========== Optimization Results ==========
Best Write: 512 KB buffer -> 1.15 MB/s
Best Read:  512 KB buffer -> 1.21 MB/s
Current buffer: 64 KB

Recommendations:
Recommended buffer size: 512 KB
Consider updating OPTIMAL_BUFFER_SIZE to 512 KB
==========================================
```
#### fs_speed_test <...>
综合测试命令。在<...>位置输入一个数字，确定待测文件大小(MB)，进行完整的速度测试。例如：
```
========== Enhanced Speed Test ==========
File: /speed_test.dat, Size: 8 MB
Buffer: 64 KB @ 0x60000000
========================================

--- Write Test ---

========== Write Speed Test ==========
File: /speed_test.dat
Size: 8 MB
Buffer: 64 KB
=====================================

[00:01] Write: 1.15 MB/s (Avg: 1.15 MB/s) - 1.2/8.0 MB (14.8%)
[00:02] Write: 1.13 MB/s (Avg: 1.14 MB/s) - 2.4/8.0 MB (29.7%)
[00:03] Write: 1.14 MB/s (Avg: 1.14 MB/s) - 3.6/8.0 MB (44.5%)
[00:04] Write: 1.13 MB/s (Avg: 1.14 MB/s) - 4.8/8.0 MB (59.4%)
[00:05] Write: 1.14 MB/s (Avg: 1.14 MB/s) - 5.9/8.0 MB (74.2%)
[00:06] Write: 1.13 MB/s (Avg: 1.14 MB/s) - 7.1/8.0 MB (89.1%)

----- Write Test Results -----
Total bytes: 8388608 (8 MB)
Total time: 7.03 seconds
Average speed: 1.14 MB/s
------------------------------

--- Read Test ---

========== Read Speed Test ==========
File: /speed_test.dat
Size: 8 MB
Buffer: 64 KB
====================================

[00:01] Read: 1.20 MB/s (Avg: 1.20 MB/s) - 1.3/8.0 MB (15.6%)
[00:02] Read: 1.20 MB/s (Avg: 1.20 MB/s) - 2.5/8.0 MB (31.3%)
[00:03] Read: 1.20 MB/s (Avg: 1.20 MB/s) - 3.8/8.0 MB (46.9%)
[00:04] Read: 1.20 MB/s (Avg: 1.20 MB/s) - 5.0/8.0 MB (62.5%)
[00:05] Read: 1.20 MB/s (Avg: 1.20 MB/s) - 6.3/8.0 MB (78.1%)
[00:06] Read: 1.20 MB/s (Avg: 1.20 MB/s) - 7.5/8.0 MB (93.8%)

----- Read Test Results -----
Total bytes: 8388608 (8 MB)
Total time: 6.67 seconds
Average speed: 1.20 MB/s
-----------------------------

========== Test Complete ==========
```


## 示例解析

### 代码结构
```
main.c
├── 文件系统初始化
│   └── mnt_init() - 创建分区并挂载文件系统
├── 速度测试功能
│   ├── cmd_fs_write_t_with_buffer() - 写入速度测试
│   └── cmd_fs_read_t_with_buffer() - 读取速度测试
├── 优化功能
│   ├── cmd_buffer_optimize() - 缓冲区大小优化
│   └── cmd_sd_optimize() - SD卡配置检查
└── 命令行接口
    └── FINSH命令导出
```

### 关键实现细节

1. **时间片速度监控**
   ```c
   if (slice_elapsed >= RT_TICK_PER_SECOND)  /* 每1秒更新一次 */
   {
       stats.instant_speed = (float)stats.slice_bytes / (1024.0f * 1024.0f) / slice_time_s;
       stats.average_speed = (float)stats.total_bytes / (1024.0f * 1024.0f) / total_time_s;
   }
   ```

2. **SD卡操作优化**
   ```c
   /* 添加操作间隔 - 关键优化 */
   if ((written % SD_BLOCK_REST_INTERVAL) == 0) {
       rt_thread_mdelay(SD_OPERATION_INTERVAL_MS);
   }
   ```

3. **缓冲区对齐**
   ```c
   /* 确保缓冲区按512字节对齐，提高SD卡访问效率 */
   rt_uint8_t *test_buffer = (rt_uint8_t*)((((uint32_t)buff_test) + 511) & ~511);
   ```

### 性能参数说明

- **SPI时钟频率**：最高12MHz（受SD卡和硬件限制）
- **缓冲区大小**：默认64KB（可通过buffer_optimize命令优化）
- **操作间隔**：每256KB数据传输后延迟5ms
- **典型性能**：
  - 写入速度：2.0-2.5 MB/s
  - 读取速度：3.0-4.0 MB/s

## 异常诊断

### 常见问题及解决方案

#### 1. SD卡未检测到
**错误信息**：
```
Error: the flash device name (sd0) is not found.
```
**解决方法**：
- 检查SD卡是否正确插入
- 确认SD卡槽连接良好
- 尝试更换SD卡

#### 2. 文件系统挂载失败
**错误信息**：
```
mount fs on flash to root fail
```
**解决方法**：
- 检查SD卡格式是否为FAT32
- 使用Windows/Linux格式化SD卡为FAT32
- 或在命令行执行：`dfs_mkfs elm root`

#### 3. 读写速度异常低
**症状**：
- 写入速度 < 1MB/s
- 读取速度 < 2MB/s

**解决方法**：
1. 运行缓冲区优化：
   ```
   msh />buffer_optimize
   ```
2. 检查SD卡配置：
   ```
   msh />sd_optimize
   ```
3. 更换更高速度等级的SD卡（建议Class 10或UHS-I）

#### 4. 测试过程中无响应
**解决方法**：
1. 等待30秒，SD卡可能在处理内部操作
2. 按复位键重启系统
3. 减小测试文件大小（如从32MB改为4MB）
4. 检查SD卡是否有物理损坏

### 调试技巧

1. **启用详细日志**
   在 `spi_msd.c` 中定义：
   ```c
   #define MSD_TRACE
   ```

2. **监控内存使用**
   ```
   msh />free
   ```

3. **查看线程状态**
   ```
   msh />list_thread
   ```

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档

- [RT-Thread 文件系统文档](https://www.rt-thread.org/document/site/programming-manual/filesystem/filesystem/)
- [SiFli SDK SPI 驱动文档](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/drivers/spi.html)
- [FAT 文件系统规范](http://elm-chan.org/fsw/ff/00index_e.html)
- [SD 卡 SPI 模式协议规范](https://www.sdcard.org/downloads/pls/)


## 【附录】SD卡性能测试-快速指令参考

### 基本测试命令 | Basic Test Commands

#### 写入速度测试 | Write Speed Test
```bash
fs_write <filename> <size_mb>
# 示例 | Example:
fs_write /test.dat 16    # 测试写入16MB | Test writing 16MB
```

#### 读取速度测试 | Read Speed Test
```bash
fs_read <filename> <size_mb>
# 示例 | Example:
fs_read /test.dat 16     # 测试读取16MB | Test reading 16MB
```

#### 综合速度测试 | Complete Speed Test
```bash
fs_speed_test [size_mb]
# 示例 | Example:
fs_speed_test 32         # 32MB读写测试 | 32MB read/write test
fs_speed_test            # 默认32MB | Default 32MB
```

### 优化命令 | Optimization Commands

#### 缓冲区优化 | Buffer Optimization
```bash
buffer_optimize
# 自动测试4KB到512KB的缓冲区大小
# Automatically tests buffer sizes from 4KB to 512KB
```

#### SD卡配置检查 | SD Card Configuration Check
```bash
sd_optimize
# 显示SD卡类型、容量、速度等信息
# Shows SD card type, capacity, speed, etc.
```

### 文件系统命令 | File System Commands

#### 列出文件 | List Files
```bash
ls [path]
# 示例 | Example:
ls /              # 列出根目录 | List root directory
ls /misc          # 列出misc目录 | List misc directory
```

#### 删除文件 | Delete File
```bash
rm <filename>
# 示例 | Example:
rm /test.dat      # 删除测试文件 | Delete test file
```

#### 查看文件内容 | View File Content
```bash
cat <filename>
# 示例 | Example:
cat /readme.txt
```

#### 复制文件 | Copy File
```bash
copy <source> <destination>
# 示例 | Example:
copy /test.dat /backup.dat
```

### 典型测试流程 | Typical Test Flow

```bash
# 1. 检查SD卡状态 | Check SD card status
sd_optimize

# 2. 快速测试 | Quick test
fs_speed_test 4

# 3. 标准测试 | Standard test
fs_speed_test 16

# 4. 优化缓冲区 | Optimize buffer
buffer_optimize

# 5. 清理文件 | Clean up files
rm /speed_test.dat
ls /
```


### 快捷键提示 | Tips

- 使用 `help` 查看所有命令 | Use `help` to see all commands
- 使用 Tab 键自动补全命令 | Use Tab for command completion
- 使用上下箭头键查看命令历史 | Use arrow keys for command history

### 故障快速排查 | Quick Troubleshooting

| 问题 Issue | 解决方案 Solution |
|-----------|------------------|
| SD卡未找到 | 重新插入SD卡 |
| SD not found | Re-insert SD card |
| 挂载失败 | 执行 `dfs_mkfs elm root` |
| Mount failed | Execute `dfs_mkfs elm root` |
| 速度过低 | 运行 `buffer_optimize` |
| Low speed | Run `buffer_optimize` |
| 测试卡死 | 重启系统 |
| Test hangs | Restart system |
