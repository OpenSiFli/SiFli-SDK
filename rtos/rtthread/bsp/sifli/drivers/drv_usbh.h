/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2017-12-12     ZYH               the first version
 * 2019-12-19     tyustli           port to stm32 series
 */
#ifndef __DRV_USBH_H__
#define __DRV_USBH_H__

#define OTG_FS_PORT 1

/** @defgroup USB_LL Device Speed
  * @{
  */
#define USBD_HS_SPEED                          0U
#define USBD_HSINFS_SPEED                      1U
#define USBH_HS_SPEED                          0U
#define USBD_FS_SPEED                          2U
#define USBH_FSLS_SPEED                        1U
/**
  * @}
  */

/** @defgroup USB_LL_Core_Speed USB Low Layer Core Speed
  * @{
  */
#define USB_OTG_SPEED_HIGH                     0U
#define USB_OTG_SPEED_HIGH_IN_FULL             1U
#define USB_OTG_SPEED_FULL                     3U
/**
  * @}
  */


int sifli_usbh_register(void);



#endif

/************* end of file ************/
