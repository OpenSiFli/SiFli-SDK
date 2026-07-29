/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef LVSF_SJPG_ALGO_H
#define LVSF_SJPG_ALGO_H

#include <stdint.h>

#ifndef JD_ALIGN
#define JD_ALIGN(size, align)   (((size) + (align) - 1) & ~((align) - 1))
#endif

typedef struct {
    uint8_t *img_cache_buff;
    int img_cache_x_res;
    int img_cache_y_res;
    int img_src_x_res;
    int img_src_y_res;
    uint16_t *y_row_map;
    uint16_t *y_col_map;
    uint16_t *uv_row_map;
    uint16_t *uv_col_map;
} lvsf_sjpg_io_source_t;

typedef struct {
    uint16_t left;
    uint16_t right;
    uint16_t top;
    uint16_t bottom;
} lvsf_sjpg_rect_t;

typedef struct {
    int (*calc_decode_size_yuv)(int src_w, int src_h, int *dst_w, int *dst_h);
    int (*calc_decode_size_rgb)(int src_w, int src_h, int *dst_w, int *dst_h);
    int (*calc_cache_size_yuv)(int src_w, int src_h, uint32_t *cache_size);
    int (*calc_cache_size_rgb)(int src_w, int src_h, uint32_t *cache_size);
    int (*is_near_half_scale_yuv)(int src_w, int src_h, int dst_w, int dst_h);
    int (*is_near_half_scale_rgb)(int src_w, int src_h, int dst_w, int dst_h);
    int (*prepare_scale_maps)(lvsf_sjpg_io_source_t *io);
    int (*fast_copy_420_no_scale)(const lvsf_sjpg_io_source_t *io, const uint8_t *src_data,
                                   const lvsf_sjpg_rect_t *rect, int valid_w, int valid_h);
    int (*fast_copy_420_half_scale)(const lvsf_sjpg_io_source_t *io, const uint8_t *src_data,
                                     const lvsf_sjpg_rect_t *rect, int valid_w, int valid_h);
    int (*fast_copy_420_mapped)(const lvsf_sjpg_io_source_t *io, const uint8_t *src_data,
                                 const lvsf_sjpg_rect_t *rect, int valid_w, int valid_h);
    int (*process_rgb)(lvsf_sjpg_io_source_t *io, const uint8_t *data,
                       const lvsf_sjpg_rect_t *rect, int valid_w, int valid_h);
} lvsf_sjpg_algo_ops_t;

const lvsf_sjpg_algo_ops_t *lvsf_sjpg_algo_get_ops(void);

#endif
