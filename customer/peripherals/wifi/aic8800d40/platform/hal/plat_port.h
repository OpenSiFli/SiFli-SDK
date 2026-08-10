/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _PLAT_PORT_H_
#define _PLAT_PORT_H_

#include "rtconfig.h"

#define PLATFORM_CACHE_LINE_SIZE        (32)

void platform_pwr_wifi_pin_init(void);
void platform_pwr_wifi_pin_enable(void);
void platform_pwr_wifi_pin_disable(void);
void platform_rst_wifi_pin_init(void);
void platform_rst_wifi_pin_enable(void);
void platform_rst_wifi_pin_disable(void);
void platform_rst_bt_pin_init(void);
void platform_rst_bt_pin_enable(void);
void platform_rst_bt_pin_disable(void);

#endif /* _PLAT_PORT_H_ */