/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file lvsf_sjpg.h
 * SJPG decoder framework header
 */

#ifndef LVSF_SJPEG_H
#define LVSF_SJPEG_H

#include "lvgl.h"
#include "lvsf_conf_internal.h"

#if LVSF_USING_SJPG

    /*********************
    *      DEFINES
    *********************/

    /**********************
    *      TYPEDEFS
    **********************/

    /**********************
    * GLOBAL PROTOTYPES
    **********************/

    int lvsf_split_jpeg_init(void);
    bool lvsf_sjpg_is_jpg(const void *src, lv_img_header_t *header);
    lv_res_t lvsf_sjpg_decoder_info(const void *src, lv_img_header_t *header);
    lv_res_t lvsf_sjpg_decoder_open(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc);
    void lvsf_sjpg_decoder_close(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc);


    /**********************
    *      MACROS
    **********************/

#endif /*LVSF_USING_SJPG*/

#endif /* LVSF_SJPEG_H */