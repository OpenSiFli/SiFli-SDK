/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LVSF_WIDGET_SRC_H_
#define _LVSF_WIDGET_SRC_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    IMG_PROG_BG_SRC = 0,
    IMG_PROG_IND_SRC,
    IMG_PROG_MASK_SRC,
};
typedef uint16_t img_prog_src_t;

/**
 * @brief Get a default image progress resource supplied by the SDK or application.
 * @param index Resource identifier, see IMG_PROG_*_SRC.
 * @return Image source, or NULL when the resource is unavailable.
 */
const void *widget_get_img_prog_src(img_prog_src_t index);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _LVSF_WIDGET_SRC_H_ */
