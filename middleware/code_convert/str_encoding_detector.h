/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __STR_ENCODING_DETECTOR_H__
#define __STR_ENCODING_DETECTOR_H__

typedef enum
{
    STR_ENCODE_UNKNOWN,
    STR_ENCODE_GBK,
    STR_ENCODE_UTF16_LE,
    STR_ENCODE_UTF16_BE,
    STR_ENCODE_UTF8,
    STR_ENCODE_ASCII
} str_encode_t;
str_encode_t str_detect_encoding(const unsigned char *data, size_t size);
#endif