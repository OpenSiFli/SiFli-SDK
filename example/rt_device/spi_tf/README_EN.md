# SD Card File System Performance Test Example

(For an overall overview of the examples and their usage, please refer to the `README.md` file in the parent "examples" directory.)

This example demonstrates how to perform file system performance testing on SD cards via SPI interface on the SF32LB52x platform. The example implements real-time speed monitoring, buffer optimization, and detailed performance analysis functionality.

This example utilizes the following features of SiFli-SDK:
- **RT-Thread File System**: Using DFS (Device File System) framework for file operations - [API Reference](https://www.rt-thread.org/document/site/programming-manual/filesystem/filesystem/)
- **SPI Driver**: Communication with SD card through SPI bus - [API Reference](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/drivers/spi.html)
- **FAT File System**: Using ELM FAT file system for data storage
- **HAL Layer Interface**: Using Hardware Abstraction Layer for low-level hardware control

Based on this example, the following applications can be created:
- Data logger applications (such as sensor data storage)
- Multimedia file storage systems
- Embedded database applications
- Firmware update storage solutions
- Large-capacity data caching systems

## Usage

The following subsections provide only absolutely necessary information. For complete steps on configuring SiFli-SDK and using it to build and run projects, please refer to [SiFli-SDK Quick Start](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html).

## Supported Development Boards

This example has been verified to run on the following development boards:
- sf32lb52-lcd_n16r8
- sf32lb52-lcd_52d

### Hardware Requirements

1. **Development Board**: Any supported SF32LB52x development board
2. **SD Card**:
   - Micro SD card (recommended capacity 2GB or above)
   - Speed class: Class 10 or UHS-I
   - File system format: FAT32
3. **Connection Method**:
   - SD card connected via onboard Micro SD card slot
   - Uses SPI1 bus (already connected on board)

**Hardware Connection Description**:
| SD Card Pin | Function | Dev Board Pin |
|-------------|----------|---------------|
| CS          | Chip Select | Auto-configured |
| MOSI        | Data Input | SPI1_MOSI |
| MISO        | Data Output | SPI1_MISO |
| CLK         | Clock | SPI1_CLK |
| VDD         | Power | 3.3V |
| GND         | Ground | GND |

### Software Requirements

- RT-Thread operating system (integrated in SDK)
- Serial terminal software (such as PuTTY, SecureCRT, etc., for viewing output and inputting commands)

## Example Output

If you see the following console output, the example should be running correctly:

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

At this point, users need to input commands through the serial terminal to interact with the example. For example, input `fs_write /test.dat 16` to test the speed of writing a 16MB file.

### Usage of Commands in Quick Commands

#### fs_write /test.dat <...>
Enter a number in the <...> position to test the speed of writing a file of that size (in MB). For example:
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
<--  Process omitted  -->
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
Enter a number in the <...> position to test the speed of reading a file of that size (in MB). For example:
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
This is a command to check SD configuration. It will list the main parameters of the environment where this example is running, with the following effect:
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
  Effective Freq: 10 MHz (limited by SD card)   // This indicates that if the SD card frequency is higher than the SPI frequency, the effective frequency is the SPI frequency; if lower, it shows the speed is limited by the SD card.

Buffer Address: 0x60000000
Buffer Size: 64 KB
===========================================

```

#### buffer_optimize
This is an option to test different buffer sizes. It can test read and write speeds at 4KB-512KB respectively, display the optimal results, and provide parameter modification suggestions, with the following effect:
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
Comprehensive test command. Enter a number in the <...> position to determine the test file size (MB) and perform a complete speed test. For example:
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

## Example Analysis

### Code Structure
```
main.c
├── File System Initialization
│   └── mnt_init() - Create partitions and mount file system
├── Speed Test Functions
│   ├── cmd_fs_write_t_with_buffer() - Write speed test
│   └── cmd_fs_read_t_with_buffer() - Read speed test
├── Optimization Functions
│   ├── cmd_buffer_optimize() - Buffer size optimization
│   └── cmd_sd_optimize() - SD card configuration check
└── Command Line Interface
    └── FINSH command export
```

### Key Implementation Details

1. **Time Slice Speed Monitoring**
   ```c
   if (slice_elapsed >= RT_TICK_PER_SECOND)  /* Update every 1 second */
   {
       stats.instant_speed = (float)stats.slice_bytes / (1024.0f * 1024.0f) / slice_time_s;
       stats.average_speed = (float)stats.total_bytes / (1024.0f * 1024.0f) / total_time_s;
   }
   ```

2. **SD Card Operation Optimization**
   ```c
   /* Add operation interval - key optimization */
   if ((written % SD_BLOCK_REST_INTERVAL) == 0) {
       rt_thread_mdelay(SD_OPERATION_INTERVAL_MS);
   }
   ```

3. **Buffer Alignment**
   ```c
   /* Ensure buffer is aligned to 512 bytes for improved SD card access efficiency */
   rt_uint8_t *test_buffer = (rt_uint8_t*)((((uint32_t)buff_test) + 511) & ~511);
   ```

### Performance Parameter Description

- **SPI Clock Frequency**: Maximum 12MHz (limited by SD card and hardware)
- **Buffer Size**: Default 64KB (can be optimized using buffer_optimize command)
- **Operation Interval**: 5ms delay after every 256KB data transfer
- **Typical Performance**:
  - Write Speed: 2.0-2.5 MB/s
  - Read Speed: 3.0-4.0 MB/s

## Exception Diagnosis

### Common Issues and Solutions

#### 1. SD Card Not Detected
**Error Message**:
```
Error: the flash device name (sd0) is not found.
```
**Solution**:
- Check if SD card is properly inserted
- Confirm SD card slot connection is good
- Try replacing the SD card

#### 2. File System Mount Failure
**Error Message**:
```
mount fs on flash to root fail
```
**Solution**:
- Check if SD card format is FAT32
- Format SD card to FAT32 using Windows/Linux
- Or execute in command line: `dfs_mkfs elm root`

#### 3. Abnormally Low Read/Write Speed
**Symptoms**:
- Write speed < 1MB/s
- Read speed < 2MB/s

**Solution**:
1. Run buffer optimization:
   ```
   msh />buffer_optimize
   ```
2. Check SD card configuration:
   ```
   msh />sd_optimize
   ```
3. Replace with higher speed class SD card (recommend Class 10 or UHS-I)

#### 4. No Response During Test
**Solution**:
1. Wait 30 seconds, SD card may be processing internal operations
2. Press reset button to restart system
3. Reduce test file size (e.g., from 32MB to 4MB)
4. Check if SD card has physical damage

### Debugging Tips

1. **Enable Verbose Logging**
   Define in `spi_msd.c`:
   ```c
   #define MSD_TRACE
   ```

2. **Monitor Memory Usage**
   ```
   msh />free
   ```

3. **View Thread Status**
   ```
   msh />list_thread
   ```

If you have any technical questions, please submit an [issue](https://github.com/OpenSiFli/SiFli-SDK/issues) on GitHub.

## Reference Documentation

- [RT-Thread File System Documentation](https://www.rt-thread.org/document/site/programming-manual/filesystem/filesystem/)
- [SiFli SDK SPI Driver Documentation](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/drivers/spi.html)
- [FAT File System Specification](http://elm-chan.org/fsw/ff/00index_e.html)
- [SD Card SPI Mode Protocol Specification](https://www.sdcard.org/downloads/pls/)

## SD Card Performance Test - Quick Command Reference

### Basic Test Commands

#### Write Speed Test
```bash
fs_write <filename> <size_mb>
# Example:
fs_write /test.dat 16    # Test writing 16MB
```

#### Read Speed Test
```bash
fs_read <filename> <size_mb>
# Example:
fs_read /test.dat 16     # Test reading 16MB
```

#### Complete Speed Test
```bash
fs_speed_test [size_mb]
# Example:
fs_speed_test 32         # 32MB read/write test
fs_speed_test            # Default 32MB
```

### Optimization Commands

#### Buffer Optimization
```bash
buffer_optimize
# Automatically tests buffer sizes from 4KB to 512KB
```

#### SD Card Configuration Check
```bash
sd_optimize
# Shows SD card type, capacity, speed, etc.
```

### File System Commands

#### List Files
```bash
ls [path]
# Example:
ls /              # List root directory
ls /misc          # List misc directory
```

#### Delete File
```bash
rm <filename>
# Example:
rm /test.dat      # Delete test file
```

#### View File Content
```bash
cat <filename>
# Example:
cat /readme.txt
```

#### Copy File
```bash
copy <source> <destination>
# Example:
copy /test.dat /backup.dat
```

### Typical Test Flow

```bash
# 1. Check SD card status
sd_optimize

# 2. Quick test
fs_speed_test 4

# 3. Standard test
fs_speed_test 16

# 4. Optimize buffer
buffer_optimize

# 5. Clean up files
rm /speed_test.dat
ls /
```

### Tips

- Use `help` to see all commands
- Use Tab for command completion
- Use arrow keys for command history

### Quick Troubleshooting

| Issue | Solution |
|-------|----------|
| SD not found | Re-insert SD card |
| Mount failed | Execute `dfs_mkfs elm root` |
| Low speed | Run `buffer_optimize` |
| Test hangs | Restart system |

