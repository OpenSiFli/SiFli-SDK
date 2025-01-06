/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     zylx         first version
 */
#include <stdbool.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "register.h"

#if 0
void HAL_DBG_printf(const char *fmt, ...)
{
    va_list args;
    static char rt_log_buf[128];
    extern void rt_kputs(const char *str);

    va_start(args, fmt);
    rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    rt_kputs(rt_log_buf);
    rt_kputs("\r\n");
    va_end(args);
}
#endif

#ifndef SF32LB55X
//#if 1
static uint32_t g_pre_cnt;

#define CAL_LP_CYCLE 20

void rc10k_cal_hook_func(void)
{
    uint32_t curr_cnt = hwp_lpsys_aon->GTIMR;
    if ((curr_cnt - g_pre_cnt) > 3000)
    {
        uint32_t count;
        int res = HAL_RC_CALget_curr_cycle_on_48M(HAL_RC_CAL_GetLPCycle_ex(), &count);
        if (res == 0)
        {
            g_pre_cnt = curr_cnt;
#ifndef SF32LB52X
            HAL_Set_backup(RTC_BACKUP_BT_LPCYCLE, count);
#endif
        }
    }
}


int rc10k_cal_init(void)
{
#ifdef SF32LB52X
    // HCPU already init LCPU config
    uint8_t *is_lcpu_cal = (uint8_t *)(0x2040FDDB);
    if (*is_lcpu_cal == 0)
        return 0;
#endif
    g_pre_cnt = hwp_lpsys_aon->GTIMR;
    rt_thread_idle_sethook(rc10k_cal_hook_func);
    uint32_t count;
    HAL_RC_CAL_SetLPCycle_ex(CAL_LP_CYCLE);

    int res = HAL_RC_CALget_curr_cycle_on_48M(HAL_RC_CAL_GetLPCycle_ex(), &count);
#ifndef SF32LB52X
    if (res == 0)
    {
        HAL_Set_backup(RTC_BACKUP_BT_LPCYCLE, count);
    }
#else
    (void)res;
#endif
    return 0;
}


INIT_PRE_APP_EXPORT(rc10k_cal_init);
#endif


int main(void)
{

    while (1)
    {
#ifndef SF32LB52X
        if (HAL_PMU_LXT_DISABLED())
        {
            uint8_t delay_time = 15;
            int cal_lvl = HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
            rt_thread_mdelay(delay_time * 1000);
        }
        else
        {
            //rt_thread_mdelay(3600000);
            break;
        }
        //rt_thread_mdelay(5000);
        //rt_kprintf("lcpu test \n");
#else
        break;
#endif
    }
}
