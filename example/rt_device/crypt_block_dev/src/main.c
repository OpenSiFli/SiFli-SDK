/*
 * SPDX-FileCopyrightText: 2024 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file main.c
 * @brief Crypt block device example — transparent AES encryption on SD/eMMC
 *
 * This example demonstrates:
 *   1. Waiting for the SD/eMMC block device to become ready
 *   2. Creating a transparent AES-encrypted wrapper (crypt_block_dev) over it
 *   3. Partitioning the encrypted device and mounting FAT filesystems
 *   4. Speed test commands for encrypted read/write
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "dfs_file.h"
#include "drivers/mmcsd_core.h"
#include "dfs_posix.h"
#include "crypt_block_dev.h"

#if defined(BSP_USING_SDMMC1)
    #define APP_SD_DEV_NAME  "sd0"
#elif defined(BSP_USING_SDMMC2)
    #define APP_SD_DEV_NAME  "sd1"
#endif /* BSP_USING_SDMMC1 */

/* Crypt device names */
#define CRYPT_DEV_NAME      "crypt_sd0"
#define RAW_PART_ROOT       "root"
#define RAW_PART_MISC       "misc"
#define CRYPT_PART_ROOT     "crypt_root"
#define CRYPT_PART_MISC     "crypt_misc"

/* AES key & IV — user should replace these with secure keys */
static const uint8_t aes_key[16] =
{
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
};
static const uint8_t aes_iv[16]  = { 0 };

/* Partition layout on the crypt device (sector offsets & sector counts) */
// #ifndef FS_ROOT_OFFSET
//     #define FS_ROOT_OFFSET  0x00001000
//     #define FS_ROOT_LEN     (48 * 1024 * 1024)      /* 48 MB */
//     #define FS_MISC_OFFSET  0x00f81000
//     #define FS_MISC_LEN     (500 * 1024 * 1024)     /* 500 MB */
// #endif


#define FS_ROOT "root"
#define FS_ROOT_OFFSET  0X00081000
#define FS_ROOT_LEN     48*1024*1024  //48M

#define FS_MISC "misc"
#define FS_MISC_OFFSET  0X00f81000
#define FS_MISC_LEN     500*1024*1024 //500MB



/* ---- Filesystem mount ---------------------------------------------------- */
int mnt_init(void)
{
    uint16_t wait_ticks = 400; /* 8s: 400 * 20ms */
    rt_device_t sd_dev = RT_NULL;
    rt_err_t ret;

    /* 1. Wait for underlying SD/eMMC device */
    rt_kprintf("wait %s device ready...\n", APP_SD_DEV_NAME);
    while (wait_ticks--)
    {
        rt_thread_mdelay(20);
        sd_dev = rt_device_find(APP_SD_DEV_NAME);
        if (sd_dev)
        {
            rt_kprintf("%s device ready\n", APP_SD_DEV_NAME);
            break;
        }
    }
    if (sd_dev == RT_NULL)
    {
        rt_kprintf("ERROR: %s device not found!\n", APP_SD_DEV_NAME);
        return -RT_ERROR;
    }

    /* 2. Create partitions on the raw SD device */
    rt_mmcsd_blk_device_create(APP_SD_DEV_NAME, RAW_PART_ROOT,
                               FS_ROOT_OFFSET >> 9, FS_ROOT_LEN >> 9);
    rt_mmcsd_blk_device_create(APP_SD_DEV_NAME, RAW_PART_MISC,
                               FS_MISC_OFFSET >> 9, FS_MISC_LEN >> 9);

    /* 3. Wrap each partition with AES-encrypted block device (CTR-128) */
    ret = rt_crypt_blk_device_create(CRYPT_PART_ROOT, RAW_PART_ROOT,
                                     aes_key, 16, aes_iv);
    if (ret != RT_EOK)
    {
        rt_kprintf("ERROR: create '%s' failed (err=%d)\n", CRYPT_PART_ROOT, ret);
        return ret;
    }
    rt_kprintf("crypt device '%s' created (AES-CTR-128)\n", CRYPT_PART_ROOT);

    ret = rt_crypt_blk_device_create(CRYPT_PART_MISC, RAW_PART_MISC,
                                     aes_key, 16, aes_iv);
    if (ret != RT_EOK)
    {
        rt_kprintf("ERROR: create '%s' failed (err=%d)\n", CRYPT_PART_MISC, ret);
        return ret;
    }
    rt_kprintf("crypt device '%s' created (AES-CTR-128)\n", CRYPT_PART_MISC);

    /* 4. Mount FAT filesystems on the encrypted partition devices */
    if (dfs_mount(CRYPT_PART_ROOT, "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("mount encrypted fs on root success\n");
    }
    else
    {
        rt_kprintf("mount encrypted fs on root fail, try mkfs...\n");
        if (dfs_mkfs("elm", CRYPT_PART_ROOT) == 0)
        {
            rt_kprintf("mkfs success, mount again\n");
            if (dfs_mount(CRYPT_PART_ROOT, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount encrypted fs on root success\n");
            else
                rt_kprintf("mount encrypted fs fail\n");
        }
        else
            rt_kprintf("mkfs fail\n");
    }

    mkdir("/misc", 0);
    if (dfs_mount(CRYPT_PART_MISC, "/misc", "elm", 0, 0) == 0)
    {
        rt_kprintf("mount encrypted fs on /misc success\n");
    }
    else
    {
        rt_kprintf("mount encrypted fs on /misc fail, try mkfs...\n");
        if (dfs_mkfs("elm", CRYPT_PART_MISC) == 0)
        {
            rt_kprintf("mkfs success, mount again\n");
            if (dfs_mount(CRYPT_PART_MISC, "/misc", "elm", 0, 0) == 0)
                rt_kprintf("mount encrypted fs on /misc success\n");
            else
                rt_kprintf("mount encrypted fs on /misc fail (err=%d)\n",
                           rt_get_errno());
        }
        else
            rt_kprintf("mkfs fail\n");
    }

    rt_kprintf("crypt block device example init done.\n");
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);

/**
 * @brief Application main entry.
 *
 * Called by RT-Thread's main thread after all INIT_* components are initialized.
 * The crypt device and filesystem are already set up by mnt_init via INIT_ENV_EXPORT.
 */
int main(void)
{
    rt_kprintf("\n");
    rt_kprintf("========================================\n");
    rt_kprintf("  Crypt Block Device Example\n");
    rt_kprintf("  AES-CTR-128 encrypted SD/eMMC\n");
    rt_kprintf("========================================\n");
    rt_kprintf("\n");
    rt_kprintf("Finsh commands:\n");
    rt_kprintf("  crypt_write    <path> <MB> [blk/call]        - encrypted file write speed test\n");
    rt_kprintf("  crypt_read     <path> <MB> [blk/call]        - encrypted file read speed test\n");
    rt_kprintf("  crypt_raw_read <sectors> [blk_per_call]      - raw block read test\n");
    rt_kprintf("  crypt_raw_write <sectors> [blk_per_call]     - raw block write test\n");
    rt_kprintf("  crypt_verify <path>                          - write 1MB, verify 2nd 512KB\n");
    rt_kprintf("\n");

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);    // Let system breath.
    }

    return 0;
}

/* ---- Speed test commands ------------------------------------------------ */
#define TEST_BUF_SIZE   512

/**
 * @brief Write speed test: write N blocks to a file on the encrypted filesystem.
 * Usage: crypt_write <path> <num_MB> [blocks_per_call]
 */
void cmd_crypt_write_t(const char *path, int num_mb, int blocks_per_call)
{
    struct dfs_fd fd;
    uint32_t t_start, t_end;
    float test_time, speed;
    uint64_t total_bits;
    int remain, buf_size, chunk;

    if (blocks_per_call < 1) blocks_per_call = 1;
    buf_size = blocks_per_call * TEST_BUF_SIZE;

    char *buf = rt_malloc(buf_size);
    if (buf == RT_NULL)
    {
        rt_kprintf("malloc failed (%d bytes)\n", buf_size);
        return;
    }
    rt_memset(buf, 0x55, buf_size);

    remain = (num_mb * 1024 * 1024) / TEST_BUF_SIZE;
    total_bits = (uint64_t)num_mb * 1024 * 1024 * 8;

    if (dfs_file_open(&fd, path, O_RDWR | O_CREAT | O_TRUNC) != 0)
    {
        rt_kprintf("open %s failed\n", path);
        rt_free(buf);
        return;
    }

    t_start = HAL_GTIMER_READ();
    while (remain > 0)
    {
        chunk = (remain > blocks_per_call) ? blocks_per_call : remain;
        if (dfs_file_write(&fd, buf, chunk * TEST_BUF_SIZE) != chunk * TEST_BUF_SIZE)
        {
            rt_kprintf("write error at remain=%d\n", remain);
            break;
        }
        remain -= chunk;
    }
    t_end = HAL_GTIMER_READ();

    dfs_file_close(&fd);

    test_time = (float)(t_end - t_start) / HAL_LPTIM_GetFreq();
    speed = (float)total_bits / test_time / 1024 / 1024;
    rt_kprintf("crypt_write: path=%s, %dMB, blocks_per_call=%d, "
               "time=%.3f s, speed=%.2f MBps\n",
               path, num_mb, blocks_per_call, (double)test_time, (double)speed);
    rt_free(buf);
}
void cmd_crypt_write(int argc, char **argv)
{
    int blocks_per_call = 1;
    if (argc < 3)
    {
        rt_kprintf("Usage: crypt_write <path> <num_MB> [blocks_per_call]\n");
        return;
    }
    if (argc >= 4)
        blocks_per_call = atoi(argv[3]);
    cmd_crypt_write_t(argv[1], atoi(argv[2]), blocks_per_call);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_crypt_write, __cmd_crypt_write,
                            test encrypted write speed);

/**
 * @brief Read speed test: read N blocks from a file on the encrypted filesystem.
 * Usage: crypt_read <path> <num_MB> [blocks_per_call]
 */
void cmd_crypt_read_t(const char *path, int num_mb, int blocks_per_call)
{
    struct dfs_fd fd;
    uint32_t t_start, t_end;
    float test_time, speed;
    uint64_t total_bits;
    int remain, buf_size, chunk;

    if (blocks_per_call < 1) blocks_per_call = 1;
    buf_size = blocks_per_call * TEST_BUF_SIZE;

    char *buf = rt_malloc(buf_size);
    if (buf == RT_NULL)
    {
        rt_kprintf("malloc failed (%d bytes)\n", buf_size);
        return;
    }

    remain = (num_mb * 1024 * 1024) / TEST_BUF_SIZE;
    total_bits = (uint64_t)num_mb * 1024 * 1024 * 8;

    if (dfs_file_open(&fd, path, O_RDONLY) != 0)
    {
        rt_kprintf("open %s failed\n", path);
        rt_free(buf);
        return;
    }

    t_start = HAL_GTIMER_READ();
    while (remain > 0)
    {
        chunk = (remain > blocks_per_call) ? blocks_per_call : remain;
        if (dfs_file_read(&fd, buf, chunk * TEST_BUF_SIZE) != chunk * TEST_BUF_SIZE)
        {
            rt_kprintf("read error at remain=%d\n", remain);
            break;
        }
        remain -= chunk;
    }
    t_end = HAL_GTIMER_READ();

    dfs_file_close(&fd);

    test_time = (float)(t_end - t_start) / HAL_LPTIM_GetFreq();
    speed = (float)total_bits / test_time / 1024 / 1024;
    rt_kprintf("crypt_read: path=%s, %dMB, blocks_per_call=%d, "
               "time=%.3f s, speed=%.2f MBps\n",
               path, num_mb, blocks_per_call, (double)test_time, (double)speed);
    rt_free(buf);
}
void cmd_crypt_read(int argc, char **argv)
{
    int blocks_per_call = 1;
    if (argc < 3)
    {
        rt_kprintf("Usage: crypt_read <path> <num_MB> [blocks_per_call]\n");
        return;
    }
    if (argc >= 4)
        blocks_per_call = atoi(argv[3]);
    cmd_crypt_read_t(argv[1], atoi(argv[2]), blocks_per_call);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_crypt_read, __cmd_crypt_read,
                            test encrypted read speed);

/**
 * @brief Raw block device read through the crypt layer (no filesystem).
 * Usage: crypt_raw_read <num_sectors> [blocks_per_call]
 */
void cmd_crypt_raw_read_t(int num_sectors, int blocks_per_call)
{
    rt_device_t dev;
    uint32_t t_start, t_end;
    float test_time, speed;
    uint64_t total_bits;
    int remain, buf_size, chunk;

    if (blocks_per_call < 1) blocks_per_call = 1;
    buf_size = blocks_per_call * 512;

    char *buf = rt_malloc(buf_size);
    if (buf == RT_NULL)
    {
        rt_kprintf("malloc failed (%d bytes)\n", buf_size);
        return;
    }

    dev = rt_device_find(CRYPT_PART_ROOT);
    if (dev == RT_NULL)
    {
        rt_kprintf("device '%s' not found\n", CRYPT_PART_ROOT);
        rt_free(buf);
        return;
    }
    rt_device_open(dev, RT_DEVICE_FLAG_RDWR);

    total_bits = (uint64_t)num_sectors * 512 * 8;
    remain = num_sectors;

    t_start = HAL_GTIMER_READ();
    while (remain > 0)
    {
        chunk = (remain > blocks_per_call) ? blocks_per_call : remain;
        if (rt_device_read(dev, 0, buf, chunk) != chunk)
        {
            rt_kprintf("read error at remain=%d\n", remain);
            break;
        }
        remain -= chunk;
    }
    t_end = HAL_GTIMER_READ();

    rt_device_close(dev);

    test_time = (float)(t_end - t_start) / HAL_LPTIM_GetFreq();
    speed = (float)total_bits / test_time / 1024 / 1024;
    rt_kprintf("crypt_raw_read: sectors=%d, blocks_per_call=%d, "
               "time=%.3f s, speed=%.2f MBps\n",
               num_sectors, blocks_per_call, (double)test_time, (double)speed);
    rt_free(buf);
}
void cmd_crypt_raw_read(int argc, char **argv)
{
    int blocks_per_call = 1;
    if (argc < 2)
    {
        rt_kprintf("Usage: crypt_raw_read <num_sectors> [blocks_per_call]\n");
        return;
    }
    if (argc >= 3)
        blocks_per_call = atoi(argv[2]);
    cmd_crypt_raw_read_t(atoi(argv[1]), blocks_per_call);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_crypt_raw_read, __cmd_crypt_raw_read,
                            test encrypted raw block read);

/**
 * @brief Raw block device write through the crypt layer (no filesystem).
 * Usage: crypt_raw_write <num_sectors> [blocks_per_call]
 */
void cmd_crypt_raw_write_t(int num_sectors, int blocks_per_call)
{
    rt_device_t dev;
    uint32_t t_start, t_end;
    float test_time, speed;
    uint64_t total_bits;
    int remain, buf_size, chunk;

    if (blocks_per_call < 1) blocks_per_call = 1;
    buf_size = blocks_per_call * 512;

    char *buf = rt_malloc(buf_size);
    if (buf == RT_NULL)
    {
        rt_kprintf("malloc failed (%d bytes)\n", buf_size);
        return;
    }
    rt_memset(buf, 0x55, buf_size);

    dev = rt_device_find(CRYPT_PART_ROOT);
    if (dev == RT_NULL)
    {
        rt_kprintf("device '%s' not found\n", CRYPT_PART_ROOT);
        rt_free(buf);
        return;
    }
    rt_device_open(dev, RT_DEVICE_FLAG_RDWR);

    total_bits = (uint64_t)num_sectors * 512 * 8;
    remain = num_sectors;

    t_start = HAL_GTIMER_READ();
    while (remain > 0)
    {
        chunk = (remain > blocks_per_call) ? blocks_per_call : remain;
        if (rt_device_write(dev, 0, buf, chunk) != chunk)
        {
            rt_kprintf("write error at remain=%d\n", remain);
            break;
        }
        remain -= chunk;
    }
    t_end = HAL_GTIMER_READ();

    rt_device_close(dev);

    test_time = (float)(t_end - t_start) / HAL_LPTIM_GetFreq();
    speed = (float)total_bits / test_time / 1024 / 1024;
    rt_kprintf("crypt_raw_write: sectors=%d, blocks_per_call=%d, "
               "time=%.3f s, speed=%.2f MBps\n",
               num_sectors, blocks_per_call, (double)test_time, (double)speed);
    rt_free(buf);
}
void cmd_crypt_raw_write(int argc, char **argv)
{
    int blocks_per_call = 1;
    if (argc < 2)
    {
        rt_kprintf("Usage: crypt_raw_write <num_sectors> [blocks_per_call]\n");
        return;
    }
    if (argc >= 3)
        blocks_per_call = atoi(argv[2]);
    cmd_crypt_raw_write_t(atoi(argv[1]), blocks_per_call);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_crypt_raw_write, __cmd_crypt_raw_write,
                            test encrypted raw block write);

/**
 * @brief Verify data correctness with random data: write 1MB, then read back
 *        the second 512KB and compare using a deterministic PRNG.
 * Usage: crypt_verify <path>
 */
void cmd_crypt_verify(const char *path)
{
#define CVERIFY_TOTAL   (1 * 1024 * 1024)   /* 1 MB */
#define CVERIFY_HALF    (512 * 1024)         /* 512 KB */
    struct dfs_fd fd;
    uint8_t *buf = RT_NULL;
    int i, sec, errors = 0;
    int total_secs, half_secs;

    buf = rt_malloc(TEST_BUF_SIZE);
    if (!buf)
    {
        rt_kprintf("malloc failed\n");
        return;
    }

    total_secs = CVERIFY_TOTAL / TEST_BUF_SIZE;   /* 2048 sectors */
    half_secs  = CVERIFY_HALF / TEST_BUF_SIZE;    /* 1024 sectors */

    /* Write 1 MB sector by sector with PRNG data */
    if (dfs_file_open(&fd, path, O_RDWR | O_CREAT | O_TRUNC) != 0)
    {
        rt_kprintf("open '%s' for write failed\n", path);
        goto out;
    }
    for (sec = 0; sec < total_secs; sec++)
    {
        srand((unsigned int)sec);
        for (i = 0; i < TEST_BUF_SIZE; i++)
            buf[i] = (uint8_t)rand();
        if (dfs_file_write(&fd, buf, TEST_BUF_SIZE) != TEST_BUF_SIZE)
        {
            rt_kprintf("write failed at sector %d\n", sec);
            dfs_file_close(&fd);
            goto out;
        }
    }
    dfs_file_close(&fd);
    rt_kprintf("Wrote 1MB random data to '%s'\n", path);

    /* Read second half (offset = 512 KB) and verify */
    if (dfs_file_open(&fd, path, O_RDONLY) != 0)
    {
        rt_kprintf("open '%s' for read failed\n", path);
        goto out;
    }
    if (dfs_file_lseek(&fd, CVERIFY_HALF) != CVERIFY_HALF)
    {
        rt_kprintf("lseek to %d failed\n", CVERIFY_HALF);
        dfs_file_close(&fd);
        goto out;
    }
    for (sec = 0; sec < half_secs; sec++)
    {
        if (dfs_file_read(&fd, buf, TEST_BUF_SIZE) != TEST_BUF_SIZE)
        {
            rt_kprintf("read failed at sector %d\n", half_secs + sec);
            dfs_file_close(&fd);
            goto out;
        }
        /* Regenerate expected random data for this sector */
        srand((unsigned int)(half_secs + sec));
        for (i = 0; i < TEST_BUF_SIZE; i++)
        {
            uint8_t expected = (uint8_t)rand();
            if (buf[i] != expected)
            {
                if (errors < 10)
                    rt_kprintf("  MISMATCH [%d]: expected 0x%02x, got 0x%02x\n",
                               (half_secs + sec) * TEST_BUF_SIZE + i,
                               expected, buf[i]);
                errors++;
            }
        }
    }
    dfs_file_close(&fd);

    if (errors == 0)
        rt_kprintf("\nVerify PASS: all 512KB at offset 512KB match\n");
    else
        rt_kprintf("\nVerify FAIL: %d mismatches out of %d bytes\n",
                   errors, CVERIFY_HALF);

out:
    rt_free(buf);
#undef CVERIFY_TOTAL
#undef CVERIFY_HALF
}
void cmd_crypt_verify_entry(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: crypt_verify <path>\n");
        return;
    }
    cmd_crypt_verify(argv[1]);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_crypt_verify_entry, __cmd_crypt_verify,
                            write 1MB and verify 512KB at offset 512KB);
