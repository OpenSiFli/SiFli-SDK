/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_COMM_H_
#define _LVSF_COMM_H_
#include "lvgl.h"
#include <stdlib.h>


uint16_t lvsf_get_img_zoom(lv_coord_t *img_width, lv_coord_t *img_height,
                           lv_coord_t parent_width, lv_coord_t parent_height);

#endif
