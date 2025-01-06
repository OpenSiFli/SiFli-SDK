/*
* Copyright 2019, Synopsys, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-3-Clause license found in
* the LICENSE file in the root directory of this source tree.
*
*/

#ifndef __CVSD_H__
#define __CVSD_H__


#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t bswap32(uint32_t input_byte);

typedef struct cvsd_s
{
    int accumulator;
    int step_size;
    unsigned int output_byte;
} cvsd_t;

short cvsdInit(cvsd_t *cvsd);
void cvsdEncode(cvsd_t *cvsd,  const short *in, unsigned int input_len,  unsigned int *out);
void cvsdDecode(cvsd_t *cvsd,  const unsigned char *in, unsigned int input_len,  short *out);

#ifndef min  //mod by prife
#define min(x,y) (x<y?x:y)
#endif
#ifndef max
#define max(x,y) (x<y?y:x)
#endif

#ifdef __cplusplus
}
#endif

#endif // __CVSD_H__
