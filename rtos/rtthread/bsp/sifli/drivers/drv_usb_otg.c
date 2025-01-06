/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2017-10-30     ZYH            the first version
 * 2019-12-19     tyustli           port to stm32 series
 */
#include "rtthread.h"
#include "bf0_hal.h"
#include "board.h"

extern void OTG_FS_IRQHandler(void);
extern void USBD_IRQHandler(void);

#ifndef BSP_USING_USBD
    #define USBD_IRQHandler() OTG_FS_IRQHandler()
#endif


#ifndef BSP_USING_USBH
#define OTG_FS_IRQHandler() \
{ \
    uint8_t temp= hwp_usbc->intrusb; \
    hwp_usbc->intrusb=temp;\
}
#endif

void USBC_IRQHandler(void)
{
    rt_interrupt_enter();

    if (hwp_usbc->devctl & USB_DEVCTL_HM)
    {
        OTG_FS_IRQHandler();
    }
    else
    {
        USBD_IRQHandler();
    }
    /* leave interrupt */
    rt_interrupt_leave();

}



