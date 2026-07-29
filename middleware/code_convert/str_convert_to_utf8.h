/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __STR_CONVERT_TO_UTF8_H__
#define __STR_CONVERT_TO_UTF8_H__

#include "str_encoding_detector.h"
#include <stdint.h>

#define STR_GBK_CONVERT 1


#ifdef STR_GBK_CONVERT
    uint32_t gbk_unicode(uint8_t *dst, uint32_t max_dst_size, uint8_t *src, uint32_t src_size);
#endif
uint32_t str_convert_to_utf8(str_encode_t src_encode, uint8_t *dst, uint32_t max_size,
                             uint8_t *src, uint32_t src_size);
#endif


