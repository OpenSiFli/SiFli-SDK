/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>
#include "utest.h"
#include "bf0_hal.h"

/*
    This example demo:
        1. Read system boot mode
*/

static uint32_t g_id;
static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static void testcase(int argc, char **argv)
{
    LOG_I("Boot mode is in %s mode\n", __HAL_SYSCFG_GET_BOOT_MODE() ? "bootrom" : "normal");
}

UTEST_TC_EXPORT(testcase, "example_syscfg", utest_tc_init, utest_tc_cleanup, 10);

