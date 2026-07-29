/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __UTF8_CONVERT_TO_UNICODE_H__
#define __UTF8_CONVERT_TO_UNICODE_H__

#include <stdint.h>

uint32_t utf8decode2unicode(uint8_t *p_dst, uint32_t max_size, uint8_t *p_src, uint32_t length);
uint32_t unicodedecode2utf8(uint8_t *p_dst, uint32_t max_size, uint8_t *p_src, uint32_t length);


#endif