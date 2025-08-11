/** @file os_cli.c
 *
 *  @brief OS CLI Interface
 *
 *  Copyright 2008-2023 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <inttypes.h>
#include <stdio.h>
#include <wm_os.h>

/* Freertos does no size check on this buffer and hence kept higher than
 * minimal size that would be required */
#define MAX_TASK_INFO_BUF 1024

void os_dump_threadinfo(char *name)
{
    char *task_info_buf = (char *)os_mem_alloc(MAX_TASK_INFO_BUF);

    if (task_info_buf == NULL)
        return;

    vTaskList(task_info_buf);

    PRINTF("Name                  State   Priority Stack  Num\r\n");
    PRINTF("==================================================\r\n");
    PRINTF("%s\r\n", task_info_buf);

    os_mem_free(task_info_buf);
}
