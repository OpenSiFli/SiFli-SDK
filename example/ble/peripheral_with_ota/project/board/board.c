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

#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS)

#include "dfs_file.h"
#include "dfs_posix.h"
#include "drv_flash.h"

#define NAND_MTD_NAME    "root"
int mnt_init(void)
{
    //TODO: how to get base address
    register_mtd_device(FS_REGION_START_ADDR, FS_REGION_SIZE, NAND_MTD_NAME);
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0) // fs exist
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else
    {
        // auto mkfs, remove it if you want to mkfs manual
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0)
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
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

#elif defined(BSP_USING_SDIO) || defined(BSP_USING_SDMMC1) || defined(BSP_USING_SDMMC2)

#include "fal.h"
#include "drivers/mmcsd_core.h"
#include <dfs_fs.h>

static int app_emmc_partition_init(void)
{
    size_t len;
    uint32_t i;
    const struct fal_partition *partition;
    const struct fal_flash_dev *fal_flash = NULL;
    const char *sd_dev_name = NULL;
    rt_device_t sd_dev = RT_NULL;

    partition = fal_get_partition_table(&len);

    /* 查找是否有 eMMC 分区 sd0*/
    for (i = 0; i < len; i++)
    {
        if ((fal_flash = fal_flash_device_find(partition[i].flash_name)) == NULL)
            continue;
        if (fal_flash->nand_flag == 2)
        {
            sd_dev_name = partition[i].flash_name;
            break;
        }
    }

    if (sd_dev_name == NULL)
        return 0; /* 没有 eMMC 分区 */

    /* 等待 sd 设备就绪，最多等 8 秒 */
    uint16_t wait_ticks = 400;
    rt_kprintf("app: wait %s device ready...\n", sd_dev_name);
    while (wait_ticks--)
    {
        rt_thread_mdelay(20);
        sd_dev = rt_device_find(sd_dev_name);
        if (sd_dev)
        {
            rt_kprintf("app: %s device ready\n", sd_dev_name);
            break;
        }
    }

    if (!sd_dev)
    {
        rt_kprintf("app: %s not found, emmc partition init failed!\n", sd_dev_name);
        return -1;
    }

    // /* 创建 eMMC 分区设备 */
    // for (i = 0; i < len; i++)
    // {
    //     if ((fal_flash = fal_flash_device_find(partition[i].flash_name)) == NULL)
    //         continue;
    //     if (fal_flash->nand_flag == 2)
    //     {
    //         rt_mmcsd_blk_device_create(partition[i].flash_name, partition[i].name,
    //                                    partition[i].offset >> 9, partition[i].len >> 9);
    //     }
    // }

#ifdef PKG_FDB_USING_FILE_POSIX_MODE
    {
        const char *part_name = "fs_root";
        const char *mount_path = "/";

        if (rt_device_find(part_name) != RT_NULL)
        {
            /* 尝试挂载到根目录 */
            if (dfs_mount(part_name, mount_path, "elm", 0, 0) != 0)
            {
                /* 挂载失败，先格式化再挂载 */
                rt_kprintf("app: format %s and mount to %s\n", part_name, mount_path);
                dfs_mkfs("elm", part_name);
                if (dfs_mount(part_name, mount_path, "elm", 0, 0) != 0)
                {
                    rt_kprintf("app: mount %s to %s failed!\n", part_name, mount_path);
                }
                else
                {
                    rt_kprintf("app: mount %s to %s OK\n", part_name, mount_path);
                }
            }
            else
            {
                rt_kprintf("app: mount %s to %s OK\n", part_name, mount_path);
            }
        }
        else
        {

        }
    }
#endif

    return 0;
}
INIT_ENV_EXPORT(app_emmc_partition_init);
#endif

void SystemClock_Config(void)
{
#if 0
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /**Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);
    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI14
                                       | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI14CalibrationValue = 16;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2
                                         | RCC_PERIPHCLK_RTC;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
#endif
}
