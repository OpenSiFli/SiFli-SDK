/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lvgl.h"
#include "lvsf_conf_internal.h"
#include "lvsf_widget_src.h"

/* All scrollbar variants share the precompiled imgprog dependency. */
#if LVGL_VERSION_MAJOR == 8 && (LVSF_USE_IMGPROG != 0 || LVSF_USE_SCROLLBAR != 0)

LV_IMG_DECLARE(lvsf_scrollbar_bg);
LV_IMG_DECLARE(lvsf_scrollbar_ind);
LV_IMG_DECLARE(lvsf_scrollbar_mask);

const void *widget_get_img_prog_src(img_prog_src_t index)
{
    switch (index)
    {
    case IMG_PROG_BG_SRC:
        return &lvsf_scrollbar_bg;
    case IMG_PROG_IND_SRC:
        return &lvsf_scrollbar_ind;
    case IMG_PROG_MASK_SRC:
        return &lvsf_scrollbar_mask;
    default:
        return NULL;
    }
}

#endif
