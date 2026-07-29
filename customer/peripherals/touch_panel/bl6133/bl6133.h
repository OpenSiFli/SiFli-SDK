/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BL6133_H
#define __BL6133_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "board.h"
#include "bl6133.h"
#include "drv_touch.h"
#include "fw_update.h"

#define  DBG_LEVEL            DBG_INFO  // DBG_LOG //
#define LOG_TAG              "drv.bl6133"
#include <drv_log.h>

/* type ------------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
extern int CTP_FLASH_I2C_WRITE(unsigned char i2c_addr, unsigned char *buf, int len);
extern int CTP_FLASH_I2C_READ(unsigned char i2c_addr, unsigned char *buf, int len);

extern void bl_ts_reset_wakeup(void);
extern void bl_ts_set_intup(char level);
extern void bl_ts_set_intmode(char mode);

#ifdef __cplusplus
}
#endif

#endif /* __BL6133_H */

