/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"


extern void mtp_init(uint8_t busid, uint32_t reg_base);

/**
 * @brief  Main program
 * @param  None
 * @retval 0 if success, otherwise failure number
 */
int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("cherry USB mtp device demo!\n");
    mtp_init(0, (uint32_t)USBC_BASE);

    /* Infinite loop */
    while (1)
    {
        // Delay for 1000 ms to yield CPU time to other threads
        rt_thread_mdelay(1000);
    }
    return 0;
}

