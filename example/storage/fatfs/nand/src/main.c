/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "dfs_file.h"
#include "drv_flash.h"
#include "finsh.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <stdlib.h>

/* User code start from here --------------------------------------------------------*/
#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#else
    #define FS_ROOT "root"
#endif

#define FS_ROOT_NAND "root_n"

#define KBYTE(n) ((n) * 1024)

#define BUFFER_SIZE_MAX  KBYTE(32)

/* Use only one largest buffer */
static char buffer[BUFFER_SIZE_MAX] ALIGN(4);


/**
 * @brief The command for creating a test file
 */
int fs_test_cmd(int argc, char **argv)
{
    int fd;
    int i, j, size = 1024;
    rt_tick_t start_time, end_time;
    rt_uint32_t elapsed_time_ms;
    rt_uint32_t write_bytes = 0;
    rt_uint32_t read_bytes = 0;
    int buffer_size_kb = 1; // default uses 1K buffer

    if (argc >= 2)
    {
        size = atoi(argv[1]);
        if (size <= 0) size = 1024;
    }

    // The third parameter specifies the buffer size in KB.
    if (argc >= 3)
    {
        buffer_size_kb = atoi(argv[2]);
        // Make sure buffer size is valid and not exceed max size
        if (buffer_size_kb <= 0 || buffer_size_kb > 32)
            buffer_size_kb = 1; // fallback to 1K if invalid
    }

    rt_kprintf("Creating test file with %d KB data using %dK buffer...\n",
               size, buffer_size_kb);

    int buffer_size = buffer_size_kb * 1024;

    /* Write tests*/
    fd = open("/test_file.txt", O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd >= 0)
    {
        rt_memset(buffer, 'A', buffer_size);

        start_time = rt_tick_get();

        for (i = 0; i < size;)
        {
            int kb_to_write = (size - i) < buffer_size_kb ? (size - i) : buffer_size_kb;
            int bytes_to_write = kb_to_write * 1024;

            int bytes_written = write(fd, buffer, bytes_to_write);
            write_bytes += bytes_written;

            i += kb_to_write;

            if (i % 100 == 0 || i >= size)
            {
                rt_kprintf("Written %d KB...\n", i);
            }
        }
        end_time = rt_tick_get();

        close(fd);


        elapsed_time_ms = (end_time - start_time) * 1000 / RT_TICK_PER_SECOND;

        rt_kprintf("Write test completed\n");
        if (elapsed_time_ms > 0)
        {
            rt_kprintf("Write Speed: %d bytes/sec (%.2f KB/s)\n",
                       (write_bytes * 1000) / elapsed_time_ms,
                       (float)(write_bytes * 1000) / elapsed_time_ms / 1024.0f);
        }
    }
    else
    {
        rt_kprintf("Failed to create test file\n");
        return -1;
    }

    /* Read the test. */
    start_time = rt_tick_get();
    fd = open("/test_file.txt", O_RDONLY, 0);
    if (fd >= 0)
    {
        rt_kprintf("Reading test file...\n");

        for (i = 0; i < size;)
        {
            int kb_to_read = (size - i) < buffer_size_kb ? (size - i) : buffer_size_kb;
            int bytes_to_read = kb_to_read * 1024;

            int bytes_read = read(fd, buffer, bytes_to_read);
            read_bytes += bytes_read;

            i += kb_to_read;

            if (i % 100 == 0 || i >= size)
            {
                rt_kprintf("Read %d KB...\n", i);
            }
        }
        close(fd);

        end_time = rt_tick_get();
        elapsed_time_ms = (end_time - start_time) * 1000 / RT_TICK_PER_SECOND;

        rt_kprintf("Read test completed\n");
        if (elapsed_time_ms > 0)
        {
            rt_kprintf("Read Speed: %d bytes/sec (%.2f KB/s)\n",
                       (read_bytes * 1000) / elapsed_time_ms,
                       (float)(read_bytes * 1000) / elapsed_time_ms / 1024.0f);
        }
    }
    else
    {
        rt_kprintf("Failed to open test file for reading\n");
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT_ALIAS(fs_test_cmd, fs_test, Test file system performance);



int mnt_init(void)
{
    register_mtd_dhara_device(FS_REGION_START_ADDR - FS_REGION_OFFSET,
                              FS_REGION_OFFSET, FS_REGION_SIZE,
                              FS_ROOT, FS_ROOT_NAND);
    if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", FS_ROOT) == 0)
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(FS_ROOT, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail\n");
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("Use help to check file system command!\n");
    rt_kprintf("Use 'fs_test to perform file system performance test!\n");

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);    // Let system breath.
    }
    return 0;
}