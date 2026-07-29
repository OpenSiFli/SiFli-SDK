/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __EPD_TPS_H__
#define __EPD_TPS_H__
#include <rtthread.h>
#ifdef __cplusplus
extern "C" {
#endif
// Module VCOM voltage (2100 means -2.10V)
void tps_init(uint16_t vcom_voltage);
rt_err_t tps_enter_sleep(void);
rt_err_t tps_exit_sleep(void);
#ifdef __cplusplus
}
#endif
#endif /*__EPD_TPS_H__*/