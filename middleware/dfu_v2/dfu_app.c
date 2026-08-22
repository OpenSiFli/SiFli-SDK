/**
 * @file dfu_app.c
 * @brief DFU V2 — User Application API Implementation
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "board.h"          /* drv_reboot() */
#include "dfu_app.h"
#include "dfu_fwinfo.h"     /* dfu_fwinfo_set_update_flags() */

#define LOG_TAG "dfu.app"
#include <log.h>

void dfu_enter_dfu_mode(void)
{
    LOG_I("Setting DFU V2 mode flag and rebooting...");

    /* Mark pending images (or create a trigger entry) through the
     * NAND/NOR-aware firmware-info layer. dfu_fwinfo_set_update_flags()
     * performs the erase, write and NAND page-cache flush internally, so this
     * path works on both NOR and NAND flash. */
    if (dfu_fwinfo_set_update_flags() != 0)
        LOG_E("failed to set DFU update flags");

    rt_thread_mdelay(100);
    drv_reboot();
    /* Does not return */
}