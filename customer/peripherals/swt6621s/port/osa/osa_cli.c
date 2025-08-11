/** @file osa_cli.c
 *
 *  @brief OSA CLI Interface
 *
 *  Copyright 2023 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <inttypes.h>
#include <osa.h>

#if CONFIG_MEM_POOLS
    #include <mem_pool_config.h>
#endif

/* Freertos does no size check on this buffer and hence kept higher than
 * minimal size that would be required */
#define MAX_TASK_INFO_BUF 1024

void OSA_DumpThreadInfo(char *name)
{
    char *task_info_buf = NULL;
#if !CONFIG_MEM_POOLS
    task_info_buf = (char *)OSA_MemoryAllocate(MAX_TASK_INFO_BUF);
#else
    task_info_buf = (char *)OSA_MemoryPoolAllocate(buf_1024_MemoryPool);
#endif

    if (task_info_buf == NULL)
        return;

#ifndef __ZEPHYR__
#if !CONFIG_MEM_POOLS
    //vTaskList(task_info_buf);
#endif
#endif

    PRINTF("Name                  State   Priority Stack  Num\r\n");
    PRINTF("==================================================\r\n");
    PRINTF("%s\r\n", task_info_buf);

#if !CONFIG_MEM_POOLS
    OSA_MemoryFree(task_info_buf);
#else
    OSA_MemoryPoolFree(buf_1024_MemoryPool, task_info_buf);
    ;
#endif
}
