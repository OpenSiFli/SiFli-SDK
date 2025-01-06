/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-21     zylx         first version
 */

#include "board.h"
#include "dfs_file.h"
#include "dfs_posix.h"
#include "drv_flash.h"
#include "lvgl.h"
#include "app_mem.h"

#if 0
    uint8_t ALIGN(4) mpi_test_data[SPI_NAND_PAGE_SIZE];
    uint8_t ALIGN(4) mpi_test_data2[SPI_NAND_PAGE_SIZE];
#endif

void SystemClock_Config(void)
{

}

#ifdef RT_USING_DFS
    extern int auto_mnt_init(void);
    INIT_ENV_EXPORT(auto_mnt_init);
#endif /* RT_USING_DFS */

