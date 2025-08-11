#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "dfs_file.h"
#include "spi_msd.h"

#define FS_ROOT "root"
#define FS_ROOT_PATH "/"
#define FS_ROOT_OFFSET  0X00000000
#define FS_ROOT_LEN     500*1024*1024  //500M

#define FS_MSIC "misc"
#define FS_MSIC_PATH "/misc"
#define FS_MSIC_OFFSET  (FS_ROOT_OFFSET + FS_ROOT_LEN)
#define FS_MSIC_LEN     500*1024*1024 //500MB

#define FS_BLOCK_SIZE 0x200

/* Optimized buffer configuration - based on actual test results */
#define BUFFER_SIZE_4KB     (4 * 1024)      // 4KB
#define BUFFER_SIZE_8KB     (8 * 1024)      // 8KB  
#define BUFFER_SIZE_16KB    (16 * 1024)     // 16KB
#define BUFFER_SIZE_32KB    (32 * 1024)     // 32KB
#define BUFFER_SIZE_64KB    (64 * 1024)     // 64KB
#define BUFFER_SIZE_128KB   (128 * 1024)    // 128KB
#define BUFFER_SIZE_256KB   (256 * 1024)    // 256KB
#define BUFFER_SIZE_512KB   (512 * 1024)    // 512KB

/* Based on actual test results, 64KB performs best in standalone tests */
#define OPTIMAL_BUFFER_SIZE BUFFER_SIZE_64KB
#define TEST_FILE_SIZE      (32 * 1024 * 1024)  // 32MB test file

/* Time slice related definitions */
#define TIME_SLICE_MS       1000         // 1 second time slice
#define TICKS_PER_SECOND    RT_TICK_PER_SECOND

/* SD card operation interval control - key optimization */
#define SD_OPERATION_INTERVAL_MS    5    // 大块操作间隔
#define SD_TEST_REST_MS            3000  // 测试间休息时间
#define SD_BLOCK_REST_INTERVAL     (256*1024)  // 每256KB休息一次

/* Test status structure */
typedef struct {
    rt_uint32_t total_bytes;      // Total transferred bytes
    rt_uint32_t slice_bytes;      // Current time slice transferred bytes
    rt_tick_t   start_tick;       // Start time
    rt_tick_t   slice_start_tick; // Time slice start time
    rt_uint32_t slice_count;      // Time slice count
    float       instant_speed;    // Instantaneous speed MB/s
    float       average_speed;    // Average speed MB/s
} speed_test_t;

/* Global buffer pointer - use static allocation to avoid dynamic memory allocation overhead */
rt_uint32_t *buff_test = (rt_uint32_t *)(0x60000000);  // Use original code memory address

/* FAL MTD device creation function */
struct rt_device *fal_mtd_msd_device_create(char *name, long offset, long len)
{
    rt_device_t msd = rt_device_find("sd0");
    if (msd == NULL)
    {
        rt_kprintf("Error: the flash device name (sd0) is not found.\n");
        return NULL;
    }
    struct msd_device *msd_dev = (struct msd_device *)msd->user_data;

    struct msd_device *msd_file_dev = (struct msd_device *)rt_malloc(sizeof(struct msd_device));
    if (msd_file_dev)
    {
        msd_file_dev->parent.type        = RT_Device_Class_MTD;
#ifdef RT_USING_DEVICE_OPS
        msd_file_dev->parent.ops        = msd_dev->parent.ops;
#else
        msd_file_dev->parent.init       = msd_dev->parent.init;
        msd_file_dev->parent.open       = msd_dev->parent.open;
        msd_file_dev->parent.close      = msd_dev->parent.close;
        msd_file_dev->parent.read       = msd_dev->parent.read;
        msd_file_dev->parent.write      = msd_dev->parent.write;
        msd_file_dev->parent.control    = msd_dev->parent.control;
#endif
        msd_file_dev->offset            = offset;
        msd_file_dev->spi_device        = msd_dev->spi_device;
        msd_file_dev->geometry.bytes_per_sector = FS_BLOCK_SIZE;
        msd_file_dev->geometry.block_size = FS_BLOCK_SIZE;
        msd_file_dev->geometry.sector_count = len;

        rt_device_register(&msd_file_dev->parent, name,
                           RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE);
        rt_kprintf("fal_mtd_msd_device_create dev:sd0 part:%s offset:0x%x, size:0x%x\n", 
                   name, msd_file_dev->offset, msd_file_dev->geometry.sector_count);
        return RT_DEVICE(&msd_file_dev->parent);
    }
    return NULL;
}

/* File system mount initialization */
int mnt_init(void)
{
    uint16_t time_out = 100;
    while (time_out --)
    {
        rt_thread_mdelay(30);
        if (rt_device_find("sd0"))
            break;
    }
    
    fal_mtd_msd_device_create(FS_ROOT, FS_ROOT_OFFSET >> 9, FS_ROOT_LEN >> 9);
    fal_mtd_msd_device_create(FS_MSIC, FS_MSIC_OFFSET >> 9, FS_MSIC_LEN >> 9);
    
    if (dfs_mount(FS_ROOT, FS_ROOT_PATH, "elm", 0, 0) == 0)
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", FS_ROOT) == 0)
        {
            rt_kprintf("make elm fs on flash success, mount again\n");
            if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
            {
                rt_kprintf("mount to fs on flash fail\n");
                return RT_ERROR;
            }
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    
    extern int mkdir(const char *path, mode_t mode);
    mkdir(FS_MSIC_PATH, 0);
    
    if (dfs_mount(FS_MSIC, FS_MSIC_PATH, "elm", 0, 0) == 0)
    {
        rt_kprintf("mount fs on flash to FS_MSIC success\n");
    }
    else
    {
        rt_kprintf("mount fs on flash to FS_MISC fail\n");
        if (dfs_mkfs("elm", FS_MSIC) == 0)
        {
            rt_kprintf("make elm fs on flash success, mount again\n");
            if (dfs_mount(FS_MSIC, "/misc", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail err=%d\n", rt_get_errno());
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);

/* Optimized write test function - add operation interval control */
void cmd_fs_write_t_with_buffer(const char *path, int num, uint32_t buffer_size)
{
    struct dfs_fd fd_test_sd;
    speed_test_t stats = {0};
    uint32_t total_size = num * 1024 * 1024;  // num is MB count
    uint32_t written = 0;
    int ret;
    
    /* Ensure buffer is aligned to 512 bytes for improved SD card access efficiency */
    rt_uint8_t *test_buffer = (rt_uint8_t*)((((uint32_t)buff_test) + 511) & ~511);
    memset(test_buffer, 0x55, buffer_size);
    
    rt_kprintf("\n========== Write Speed Test ==========\n");
    rt_kprintf("File: %s\n", path);
    rt_kprintf("Size: %d MB\n", num);
    rt_kprintf("Buffer: %d KB\n", buffer_size / 1024);
    rt_kprintf("=====================================\n");
    
    if (dfs_file_open(&fd_test_sd, path, O_RDWR | O_CREAT | O_TRUNC) == 0)
    {
        stats.start_tick = rt_tick_get();
        stats.slice_start_tick = stats.start_tick;
        
        while (written < total_size)
        {
            uint32_t write_size = (total_size - written) > buffer_size ? 
                                  buffer_size : (total_size - written);
            
            ret = dfs_file_write(&fd_test_sd, test_buffer, write_size);
            if (ret != write_size)
            {
                rt_kprintf("\nWrite error at %.1f MB, expected: %d, actual: %d\n", 
                          (float)written / (1024.0f * 1024.0f), write_size, ret);
                break;
            }
            
            written += ret;
            stats.total_bytes += ret;
            stats.slice_bytes += ret;
            
            /* Add operation interval - key optimization */
            if ((written % SD_BLOCK_REST_INTERVAL) == 0) {
                rt_thread_mdelay(SD_OPERATION_INTERVAL_MS);
            }
            
            /* Check if reached time slice (1 second) */
            rt_tick_t current_tick = rt_tick_get();
            rt_tick_t slice_elapsed = current_tick - stats.slice_start_tick;
            
            if (slice_elapsed >= RT_TICK_PER_SECOND)  /* Update every 1 second */
            {
                /* Calculate speed */
                float slice_time_s = (float)slice_elapsed / RT_TICK_PER_SECOND;
                float total_time_s = (float)(current_tick - stats.start_tick) / RT_TICK_PER_SECOND;
                
                stats.instant_speed = (float)stats.slice_bytes / (1024.0f * 1024.0f) / slice_time_s;
                stats.average_speed = (float)stats.total_bytes / (1024.0f * 1024.0f) / total_time_s;
                
                /* Display progress - update every 1 second */
                rt_kprintf("\r[%02d:%02d] Write: %.2f MB/s (Avg: %.2f MB/s) - %.1f/%.1f MB (%.1f%%)",
                           (int)(total_time_s / 60), (int)((int)total_time_s % 60),
                           stats.instant_speed, stats.average_speed,
                           (float)written / (1024.0f * 1024.0f), (float)total_size / (1024.0f * 1024.0f),
                           ((float)written * 100.0f) / total_size);
                
                /* Reset time slice statistics  */
                stats.slice_bytes = 0;
                stats.slice_start_tick = current_tick;
            }
        }
        
        dfs_file_close(&fd_test_sd);
        
        /* Display final results */
        rt_tick_t total_ticks = rt_tick_get() - stats.start_tick;
        float total_time_s = (float)total_ticks / RT_TICK_PER_SECOND;
        float final_speed = (float)stats.total_bytes / (1024.0f * 1024.0f) / total_time_s;
        
        rt_kprintf("\n\n----- Write Test Results -----\n");
        rt_kprintf("Total bytes: %d (%d MB)\n", stats.total_bytes, stats.total_bytes/(1024*1024));
        rt_kprintf("Total time: %.2f seconds\n", total_time_s);
        rt_kprintf("Average speed: %.2f MB/s\n", final_speed);
        rt_kprintf("------------------------------\n");
    }
    else
    {
        rt_kprintf("Failed to open file: %s\n", path);
    }
}

/* Optimized read test function - add operation interval control */
void cmd_fs_read_t_with_buffer(const char *path, int num, uint32_t buffer_size)
{
    struct dfs_fd fd_read;
    speed_test_t stats = {0};
    uint32_t total_size = num * 1024 * 1024;  //num is MB count
    uint32_t read_bytes = 0;
    int ret;
    
    rt_uint8_t *test_buffer = (rt_uint8_t*)((((uint32_t)buff_test) + 511) & ~511);
    rt_memset(test_buffer, 0, buffer_size);
    
    rt_kprintf("\n========== Read Speed Test ==========\n");
    rt_kprintf("File: %s\n", path);
    rt_kprintf("Size: %d MB\n", num);
    rt_kprintf("Buffer: %d KB\n", buffer_size / 1024);
    rt_kprintf("====================================\n");
    
    if (dfs_file_open(&fd_read, path, O_RDONLY) == 0)
    {
        stats.start_tick = rt_tick_get();
        stats.slice_start_tick = stats.start_tick;
        
        while (read_bytes < total_size)
        {
            uint32_t read_size = (total_size - read_bytes) > buffer_size ? 
                                 buffer_size : (total_size - read_bytes);
            
            ret = dfs_file_read(&fd_read, test_buffer, read_size);
            if (ret <= 0)
            {
                if (ret == 0) 
                {
                    rt_kprintf("\nReached end of file at %.1f MB\n", 
                              (float)read_bytes / (1024.0f * 1024.0f));
                }
                else
                {
                    rt_kprintf("\nRead error at %.1f MB\n", 
                              (float)read_bytes / (1024.0f * 1024.0f));
                }
                break;
            }
            
            read_bytes += ret;
            stats.total_bytes += ret;
            stats.slice_bytes += ret;
            
            /* Add operation interval - key optimization */
            if ((read_bytes % SD_BLOCK_REST_INTERVAL) == 0) {
                rt_thread_mdelay(SD_OPERATION_INTERVAL_MS);
            }
            
            /* Check if reached time slice (1 second) */
            rt_tick_t current_tick = rt_tick_get();
            rt_tick_t slice_elapsed = current_tick - stats.slice_start_tick;
            
            if (slice_elapsed >= RT_TICK_PER_SECOND)  /* Update every 1 second */
            {
                /* Calculate speed */
                float slice_time_s = (float)slice_elapsed / RT_TICK_PER_SECOND;
                float total_time_s = (float)(current_tick - stats.start_tick) / RT_TICK_PER_SECOND;
                
                stats.instant_speed = (float)stats.slice_bytes / (1024.0f * 1024.0f) / slice_time_s;
                stats.average_speed = (float)stats.total_bytes / (1024.0f * 1024.0f) / total_time_s;
                
                /* Display progress - update every 1 second */
                rt_kprintf("\r[%02d:%02d] Read: %.2f MB/s (Avg: %.2f MB/s) - %.1f/%.1f MB (%.1f%%)",
                           (int)(total_time_s / 60), (int)((int)total_time_s % 60),
                           stats.instant_speed, stats.average_speed,
                           (float)read_bytes / (1024.0f * 1024.0f), (float)total_size / (1024.0f * 1024.0f),
                           ((float)read_bytes * 100.0f) / total_size);
                
                /* Reset time slice statistics */
                stats.slice_bytes = 0;
                stats.slice_start_tick = current_tick;
            }
        }
        
        dfs_file_close(&fd_read);
        
        /* Display final results */
        rt_tick_t total_ticks = rt_tick_get() - stats.start_tick;
        float total_time_s = (float)total_ticks / RT_TICK_PER_SECOND;
        float final_speed = (float)stats.total_bytes / (1024.0f * 1024.0f) / total_time_s;
        
        rt_kprintf("\n\n----- Read Test Results -----\n");
        rt_kprintf("Total bytes: %d (%d MB)\n", stats.total_bytes, stats.total_bytes/(1024*1024));
        rt_kprintf("Total time: %.2f seconds\n", total_time_s);
        rt_kprintf("Average speed: %.2f MB/s\n", final_speed);
        rt_kprintf("-----------------------------\n");
    }
    else
    {
        rt_kprintf("Failed to open file: %s\n", path);
    }
}

/* Compatibility functions - use default buffer size */
void cmd_fs_write_t(const char *path, int num)
{
    cmd_fs_write_t_with_buffer(path, num, OPTIMAL_BUFFER_SIZE);
}

void cmd_fs_read_t(const char *path, int num)
{
    cmd_fs_read_t_with_buffer(path, num, OPTIMAL_BUFFER_SIZE);
}

/* Corrected buffer optimization test function */
void cmd_buffer_optimize(int argc, char **argv)
{
    uint32_t buffer_sizes[] = {
        BUFFER_SIZE_4KB,
        BUFFER_SIZE_8KB,
        BUFFER_SIZE_16KB,
        BUFFER_SIZE_32KB,
        BUFFER_SIZE_64KB,
        BUFFER_SIZE_128KB,
        BUFFER_SIZE_256KB,
        BUFFER_SIZE_512KB
    };
    
    int i;
    float best_write_speed = 0;
    float best_read_speed = 0;
    uint32_t best_write_size = 0;
    uint32_t best_read_size = 0;
    
    rt_kprintf("\n========== Buffer Size Optimization Test ==========\n");
    rt_kprintf("Testing with proper SD card rest intervals...\n");
    rt_kprintf("Each test uses isolated files and adequate rest time.\n");
    rt_kprintf("====================================================\n");
    
    for (i = 0; i < sizeof(buffer_sizes)/sizeof(buffer_sizes[0]); i++)
    {
        uint32_t buf_size = buffer_sizes[i];
        
        rt_kprintf("[%d/%d] Testing %d KB buffer:\n", 
                   i+1, sizeof(buffer_sizes)/sizeof(buffer_sizes[0]), 
                   buf_size / 1024);
        
        /* Let SD card rest adequately before each test */
        rt_kprintf("  SD card resting (%d seconds)...\n", SD_TEST_REST_MS/1000);
        rt_thread_mdelay(SD_TEST_REST_MS);
        
        /* Use unique file names to avoid conflicts */
        char test_filename[64];
        rt_sprintf(test_filename, "/buftest_%dk_%d.dat", buf_size/1024, i);
        
        /* Ensure file doesn't exist */
        dfs_file_unlink(test_filename);
        rt_thread_mdelay(200);
        
        rt_kprintf("  Write test (%s)...\n", test_filename);
        
        /* Perform write test */
        struct dfs_fd fd;
        rt_uint8_t *test_buffer = (rt_uint8_t*)((((uint32_t)buff_test) + 511) & ~511);
        memset(test_buffer, 0x55, buf_size);
        
        uint32_t test_size = 4 * 1024 * 1024; // 4MB test
        uint32_t start_tick, end_tick;
        float write_speed = 0, read_speed = 0;
        rt_bool_t write_success = RT_FALSE, read_success = RT_FALSE;
        
        /* Write test */
        if (dfs_file_open(&fd, test_filename, O_RDWR | O_CREAT | O_TRUNC) == 0) {
            uint32_t written = 0;
            start_tick = rt_tick_get();
            
            while (written < test_size) {
                uint32_t chunk = (test_size - written) > buf_size ? buf_size : (test_size - written);
                int ret = dfs_file_write(&fd, test_buffer, chunk);
                
                if (ret != chunk) {
                    rt_kprintf("    Write error at offset %d: expected %d, got %d\n", 
                              written, chunk, ret);
                    break;
                }
                
                written += ret;
                
                /* Add operation interval */
                if ((written % SD_BLOCK_REST_INTERVAL) == 0) {
                    rt_thread_mdelay(SD_OPERATION_INTERVAL_MS);
                }
            }
            
            end_tick = rt_tick_get();
            dfs_file_close(&fd);
            
            if (written == test_size) {
                float time_s = (float)(end_tick - start_tick) / RT_TICK_PER_SECOND;
                write_speed = (float)(written / 1024.0f / 1024.0f) / time_s;
                write_success = RT_TRUE;
                
                rt_kprintf("    Write: %.2f MB/s", write_speed);
                if (write_speed > best_write_speed) {
                    best_write_speed = write_speed;
                    best_write_size = buf_size;
                    rt_kprintf(" [NEW BEST]");
                }
                rt_kprintf("\n");
            } else {
                rt_kprintf("    Write incomplete: %d/%d bytes\n", written, test_size);
            }
        } else {
            rt_kprintf("    Cannot create file %s\n", test_filename);
        }
        
        /* Rest between write and read tests */
        if (write_success) {
            rt_thread_mdelay(1000);
            rt_kprintf("  Read test...\n");
            
            /* Read test */
            memset(test_buffer, 0, buf_size);
            if (dfs_file_open(&fd, test_filename, O_RDONLY) == 0) {
                uint32_t read_data = 0;
                start_tick = rt_tick_get();
                
                while (read_data < test_size) {
                    uint32_t chunk = (test_size - read_data) > buf_size ? buf_size : (test_size - read_data);
                    int ret = dfs_file_read(&fd, test_buffer, chunk);
                    
                    if (ret <= 0) {
                        rt_kprintf("    Read error at offset %d: got %d\n", read_data, ret);
                        break;
                    }
                    
                    read_data += ret;
                    
                    /* Add operation interval */
                    if ((read_data % SD_BLOCK_REST_INTERVAL) == 0) {
                        rt_thread_mdelay(SD_OPERATION_INTERVAL_MS);
                    }
                }
                
                end_tick = rt_tick_get();
                dfs_file_close(&fd);
                
                if (read_data == test_size) {
                    float time_s = (float)(end_tick - start_tick) / RT_TICK_PER_SECOND;
                    read_speed = (float)(read_data / 1024.0f / 1024.0f) / time_s;
                    read_success = RT_TRUE;
                    
                    rt_kprintf("    Read: %.2f MB/s", read_speed);
                    if (read_speed > best_read_speed) {
                        best_read_speed = read_speed;
                        best_read_size = buf_size;
                        rt_kprintf(" [NEW BEST]");
                    }
                    rt_kprintf("\n");
                } else {
                    rt_kprintf("    Read incomplete: %d/%d bytes\n", read_data, test_size);
                }
            } else {
                rt_kprintf("    Cannot open file %s for reading\n", test_filename);
            }
        }
        
        /* Clean up test files */
        dfs_file_unlink(test_filename);
        
        /* Display test summary */
        if (write_success && read_success) {
            rt_kprintf("  Summary: Write %.2f MB/s, Read %.2f MB/s\n", write_speed, read_speed);
        } else {
            rt_kprintf("  Test failed - some operations incomplete\n");
        }
        
        rt_kprintf("\n");
    }
    
    rt_kprintf("\n========== Optimization Results ==========\n");
    rt_kprintf("Best Write: %d KB buffer -> %.2f MB/s\n", 
               best_write_size / 1024, best_write_speed);
    rt_kprintf("Best Read:  %d KB buffer -> %.2f MB/s\n", 
               best_read_size / 1024, best_read_speed);
    rt_kprintf("Current buffer: %d KB\n", OPTIMAL_BUFFER_SIZE / 1024);
    
    rt_kprintf("\nRecommendations:\n");
    if (best_write_size > 0 && best_read_size > 0) {
        uint32_t recommended_size = (best_write_size + best_read_size) / 2;
        rt_kprintf("Recommended buffer size: %d KB\n", recommended_size / 1024);
        
        if (recommended_size == OPTIMAL_BUFFER_SIZE) {
            rt_kprintf("Current configuration is already optimal!\n");
        } else {
            rt_kprintf("Consider updating OPTIMAL_BUFFER_SIZE to %d KB\n", recommended_size / 1024);
        }
    }
    
    rt_kprintf("==========================================\n");
}

/* Command line interface - write test */
void cmd_fs_write(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("Usage: cmd_fs_write <filename> <num_mb>\n");
        return;
    }
    
    cmd_fs_write_t(argv[1], atoi(argv[2]));
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_fs_write, __cmd_fs_write, test write speed);

/* Command line interface - read test */
void cmd_fs_read(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("Usage: cmd_fs_read <filename> <num_mb>\n");
        return;
    }
    
    cmd_fs_read_t(argv[1], atoi(argv[2]));
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_fs_read, __cmd_fs_read, test read speed);

/*  Set buffer address */
int cmd_emmc_test_buff(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: cmd_emmc_test_buff <512|1024>\n");
        return -1;
    }
    
    if (atoi(argv[1]) == 512)
        buff_test = (rt_uint32_t *)(0x60000000);
    else if (atoi(argv[1]) == 1024)
        buff_test = (rt_uint32_t *)(0x60100000);  // 1M offset
    else 
        buff_test = (rt_uint32_t *)(0x60000003);
        
    rt_kprintf("%s buff_test=%p\n", __func__, buff_test);
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_emmc_test_buff, __cmd_emmc_test_buff, cmd emmc test buff);

/* Enhanced speed test function - with time slice display */
void cmd_fs_speed_test(int argc, char **argv)
{
    const char *test_file = "/speed_test.dat";
    uint32_t test_size_mb = 32;
    
    if (argc > 1)
        test_size_mb = atoi(argv[1]);
    
    rt_kprintf("\n========== Enhanced Speed Test ==========\n");
    rt_kprintf("File: %s, Size: %d MB\n", test_file, test_size_mb);
    rt_kprintf("Buffer: %d KB @ 0x%08X\n", OPTIMAL_BUFFER_SIZE/1024, (uint32_t)buff_test);
    rt_kprintf("========================================\n");
    
    /* Write test */
    rt_kprintf("\n--- Write Test ---\n");
    cmd_fs_write_t(test_file, test_size_mb);
    
    /* Delay 1 second */
    rt_thread_mdelay(1000);
    
    /* Read test */
    rt_kprintf("\n--- Read Test ---\n");
    cmd_fs_read_t(test_file, test_size_mb);
    
    rt_kprintf("\n========== Test Complete ==========\n");
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_fs_speed_test, __cmd_fs_speed_test, enhanced speed test);

/* Check and optimize SD card configuration */
void cmd_sd_optimize(int argc, char **argv)
{
    rt_device_t sd_dev;
    rt_device_t spi_dev;
    struct msd_device *msd;
    
    rt_kprintf("\n========== SD Card Configuration ==========\n");
    
    sd_dev = rt_device_find("sd0");
    if (sd_dev == NULL)
    {
        rt_kprintf("SD card device not found!\n");
        return;
    }
    
    msd = (struct msd_device *)sd_dev->user_data;
    if (msd != NULL)
    {
        rt_kprintf("SD Card Type: ");
        switch(msd->card_type)
        {
            case MSD_CARD_TYPE_MMC:
                rt_kprintf("MMC\n");
                break;
            case MSD_CARD_TYPE_SD_V1_X:
                rt_kprintf("SD V1.x\n");
                break;
            case MSD_CARD_TYPE_SD_V2_X:
                rt_kprintf("SD V2.0\n");
                break;
            case MSD_CARD_TYPE_SD_SDHC:
                rt_kprintf("SDHC\n");
                break;
            default:
                rt_kprintf("Unknown\n");
        }
        
        rt_kprintf("SD Card Max Clock: %d MHz (from CSD register)\n", msd->max_clock / 1000000);
        rt_kprintf("Block Size: %d bytes\n", msd->geometry.block_size);
        rt_kprintf("Sector Count: %d\n", msd->geometry.sector_count);
        rt_kprintf("Capacity: %d MB\n", 
                   (msd->geometry.sector_count * msd->geometry.bytes_per_sector) / (1024 * 1024));
        
        /* Check SPI device status */
        rt_kprintf("\n--- SPI Status ---\n");
        spi_dev = rt_device_find("sdcard");
        if (spi_dev != NULL)
        {
            rt_kprintf("SPI Device Flags: 0x%04X\n", spi_dev->flag);
            if (spi_dev->flag & RT_DEVICE_FLAG_DMA_RX)
                rt_kprintf("  DMA RX: ENABLED\n");
            else
                rt_kprintf("  DMA RX: DISABLED\n");
                
            if (spi_dev->flag & RT_DEVICE_FLAG_DMA_TX)
                rt_kprintf("  DMA TX: ENABLED\n");
            else
                rt_kprintf("  DMA TX: DISABLED\n");
                
            /* Check SPI configuration */
            if (msd->spi_device != NULL)
            {
                rt_kprintf("\nSPI Configuration:\n");
                rt_kprintf("  Data Width: %d bits\n", msd->spi_device->config.data_width);
                rt_kprintf("  Current SPI Hz: %d MHz\n", msd->spi_device->config.max_hz / 1000000);
                rt_kprintf("  Mode: 0x%02X\n", msd->spi_device->config.mode);
                
                /* Calculate actual effective frequency */
                uint32_t effective_freq = msd->max_clock < msd->spi_device->config.max_hz ? 
                                         msd->max_clock : msd->spi_device->config.max_hz;
                rt_kprintf("  Effective Freq: %d MHz (limited by SD card)\n", effective_freq / 1000000);
            }
        }
        else
        {
            rt_kprintf("SPI device 'sdcard' not found!\n");
        }
    }
    
    rt_kprintf("\nBuffer Address: 0x%08X\n", (uint32_t)buff_test);
    rt_kprintf("Buffer Size: %d KB\n", OPTIMAL_BUFFER_SIZE / 1024);
    rt_kprintf("===========================================\n");
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_sd_optimize, __cmd_sd_optimize, check SD card configuration);

FINSH_FUNCTION_EXPORT_ALIAS(cmd_buffer_optimize, __cmd_buffer_optimize, optimize buffer size);

/* Automatic benchmark test */
void auto_benchmark(void)
{
    rt_kprintf("\n========== Auto Benchmark Test ==========\n");
    rt_kprintf("Testing SD card performance...\n");
    
    /* 4MB test */
    rt_kprintf("\n--- 4MB File Test ---\n");
    cmd_fs_write_t("/bench_4mb.dat", 4);
    rt_thread_mdelay(500);
    cmd_fs_read_t("/bench_4mb.dat", 4);
    rt_thread_mdelay(500);
    
    /* 16MB test */
    rt_kprintf("\n--- 16MB File Test ---\n");
    cmd_fs_write_t("/bench_16mb.dat", 16);
    rt_thread_mdelay(500);
    cmd_fs_read_t("/bench_16mb.dat", 16);
    rt_thread_mdelay(500);
    
    /* 32MB test */
    rt_kprintf("\n--- 32MB File Test ---\n");
    cmd_fs_write_t("/bench_32mb.dat", 32);
    rt_thread_mdelay(500);
    cmd_fs_read_t("/bench_32mb.dat", 32);
    
    rt_kprintf("\n========================================\n");
}

int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("\n========== SD Card File System Performance Test ==========\n");
    rt_kprintf("SF32LB52x SD Card Test Program\n");
    rt_kprintf("Tick Per Second: %d\n", RT_TICK_PER_SECOND);
    rt_kprintf("Optimal Buffer Size: %d KB\n", OPTIMAL_BUFFER_SIZE / 1024);
    rt_kprintf("SD Operation Interval: %d ms\n", SD_OPERATION_INTERVAL_MS);
    rt_kprintf("\nUse 'help' to see available commands\n");
    rt_kprintf("Quick commands:\n");
    rt_kprintf("  fs_write /test.dat 16         - Test write speed (16MB)\n");
    rt_kprintf("  fs_read /test.dat 16          - Test read speed (16MB)\n");
    rt_kprintf("  sd_optimize                   - Check SD configuration\n");
    rt_kprintf("  buffer_optimize               - Test different buffer sizes\n");
    rt_kprintf("  fs_speed_test 32              - Complete speed test (32MB)\n");
    rt_kprintf("=========================================================\n\n");
    
    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);    // Let system breath.
    }
    return 0;
}