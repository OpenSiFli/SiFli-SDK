/*
* Copyright 2019, Synopsys, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-3-Clause license found in
* the LICENSE file in the root directory of this source tree.
*
*/

#ifdef __FXAPI__
    #include "fxarc.h"
#else
    #include "stdint.h"
#endif

#define FIR_FILTER_LENGTH 64

int interpolation_x8(int16_t *inp_buf, int inp_len, int16_t *out_buf, int out_len);
int decimation_x8(int16_t *inp_buf, int inp_len, int16_t *out_buf, int out_len);