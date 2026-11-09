/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LV_PORT_H
#define LV_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Bind the "lcd" device to an LVGL display. */
void lv_port_disp_init(void);

/* Bind the "touch" device to an LVGL pointer input device. */
void lv_port_indev_init(void);

#ifdef __cplusplus
}
#endif

#endif /* LV_PORT_H */
