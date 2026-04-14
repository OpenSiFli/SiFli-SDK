/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "usbh_core.h"

extern void usbh_class_test(void);

/**
 * @brief  Main program
 * @param  None
 * @retval 0 if success, otherwise failure number
 */
int main(void)
{
    rt_kprintf("cherryusb host demo!\n");
    usbh_initialize(0, (uintptr_t)USBC_BASE, RT_NULL);
    usbh_class_test();

    while (1)
    {
        rt_thread_mdelay(1000);
    }
    return 0;
}
