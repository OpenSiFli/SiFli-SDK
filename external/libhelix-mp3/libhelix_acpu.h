/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LIBHELIX_ACPU_H__
#define __LIBHELIX_ACPU_H__

typedef struct
{
    HMP3Decoder hMP3Decoder;
    unsigned char **inbuf;
    int *bytesLeft;
    short *outbuf;
    int useSize;
    int is_seeking;
} libhelix_decode_parameter_t;

#endif
