/******************************************************************************
 *
 *  Copyright 2022 Google LLC
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

    .syntax unified
    .thumb


/**
 * Synthesize samples of a 4 subbands block
 * state         <r0>    Previous transformed samples of the channel
 * in            <r1>    Sub-band input samples
 * scale         <r2>    Scale factor of samples
 * out           <r3>    Output adress of PCM samples
 * pitch         <sp+4>  Number of PCM samples between two consecutive
 */

    .text

    .align  4

cos_table_4:
    .short  5793,  5793, -8192, -8192,  3135, -7568

win_table_4_0:
    .short    0, -126,  -358, -848, -4443, -9644, 4443,  -848,  358, -126
    .short    0, -126,  -358, -848, -4443, -9644, 4443,  -848
    .short
    .short  -18, -128,  -670, -201, -6389, -9235, 2544, -1055,  100,  -90
    .short  -18, -128,  -670, -201, -6389, -9235, 2544, -1055
    .short
    .short  -49,  -61,  -946,  944, -8082, -8082,  944,  -946,  -61,  -49
    .short  -49,  -61,  -946,  944, -8082, -8082,  944,  -946
    .short
    .short  -90,  100, -1055, 2544, -9235, -6389, -201,  -670, -128,  -18
    .short  -90,  100, -1055, 2544, -9235, -6389, -201,  -670

win_table_4_1:
    .short       -126,  -358, -848, -4443, -9644, 4443,  -848,  358, -126
    .short    0, -126,  -358, -848, -4443, -9644, 4443,  -848,  358
    .short
    .short       -128,  -670, -201, -6389, -9235, 2544, -1055,  100,  -90
    .short  -18, -128,  -670, -201, -6389, -9235, 2544, -1055,  100
    .short
    .short        -61,  -946,  944, -8082, -8082,  944,  -946,  -61,  -49
    .short  -49,  -61,  -946,  944, -8082, -8082,  944,  -946,  -61
    .short
    .short        100, -1055, 2544, -9235, -6389, -201,  -670, -128,  -18
    .short  -90,  100, -1055, 2544, -9235, -6389, -201,  -670, -128

    .thumb_func
    .global sbc_synthesize_4

sbc_synthesize_4:

    push   { r3-r11, lr }              /* Push registers                      */

    /* DCT of subbands samples .............................................. */

    ldm    r1  , { r6-r7 }             /* r6-7 = s[0-3]                       */

    shasx  r5  , r6  , r7              /* r5  = s12 : d03                     */
    shsax  r6  , r6  , r7              /* r6  = d12 : s03                     */

    pkhtb  r4  , r5  , r6              /* r4  = s12 : s03                     */
    pkhtb  r5  , r6  , r5              /* r5  = d12 : d03                     */

    ldr    r1  , [r0], #4              /* r1  = state->idx                    */
    mov    r11 , r0                    /* r11 = &state->v[0]                  */
    mov    r12 , r0                    /* r12 = &state->v[0]                  */
    ands   r10 , r1  , #1              /* r10 = state->idx & 1, Z = r10 EQ 0  */
    ite    ne                          /* If-Then-Else ...                    */
    addne  r11 , r11 , #160            /* ... r11 = &state->v[1]              */
    addeq  r12 , r12 , #160            /* ... r12 = &state->v[1]              */

    cmp    r1  , #0                    /* Compare state->idx                  */
    ite    ne                          /* If-Then-Else ...                    */
    rsbne  r1  , r1  , #10             /* ... r1 = state->idx ?               */
    moveq  r1  , #0                    /* ...      10 - state->idx : 0        */
    add    r11 , r11 , r1  , LSL #1    /* r11 = &state->v[ odd][0][idx]       */
    add    r12 , r12 , r1  , LSL #1    /* r12 = &state->v[!odd][0][idx]       */

    adr    r14 , cos_table_4           /* r14 = &cos_table                    */

    add    r2  , r2 , #12              /* r2  = scale + 12 = shr              */
    mov    r1  , #1                    /* r1 = 1                              */
    sub    r8  , r2  , #1              /* r8 = shr - 1                        */
    lsl    r1  , r1  , r8              /* r1 = 1 << (shr - 1)                 */

    ldm    r14!, { r7-r9 }             /* r7-r10 = cos( 2,2, -0,-0, 3,-1 )    */

    smlsd  r6  , r4  , r7  , r1        /* r6 +=  s03 cos(2) -  s12 cos(2)     */
    asr    r6  , r2                    /* r6  = a0                            */
    rsb    r3  , r6  , #0              /* r3  = -a0                           */

    ssat   r6  , #16 , r6              /* r6  = sat( a0)                      */
    strh   r6  , [r11, #0]             /* Store v0[0]                         */

    ssat   r6  , #16 , r3              /* r6  = sat(-a0)                      */
    strh   r6  , [r12, #0]             /* Store v1[0]                         */

    smlad  r6  , r4  , r8  , r1        /* r6 += -s03 cos(0) + -s12 cos(0)     */
    asr    r6  , r2                    /* r6  = b1                            */
    ssat   r6  , #16 , r6              /* r6  = sat(b1)                       */
    strh   r6  , [r12, #40]            /* Store v1[2]                         */

    smlad  r6  , r5  , r9  , r1        /* r6 +=  d03 cos(3) + -d12 cos(1)     */
    asr    r6  , r2                    /* r6  = a1                            */
    rsb    r3  , r6  , #0              /* r6  = -a1                           */

    ssat   r6  , #16 , r6              /* r6  = sat(a1)                       */
    strh   r6  , [r11, #20]            /* Store v0[1]                         */

    ssat   r6  , #16 , r3              /* r6  = sat(-a1)                      */
    strh   r6  , [r11, #60]            /* Store v0[3]                         */

    smlsdx r6  , r5  , r9  , r1        /* r6 += -d03 cos(3) - -d12 cos(3)     */
    asr    r6  , r2                    /* r6  = b0                            */
    ssat   r6  , #16 , r6              /* r6  = sat(b0)                       */
    strh   r6  , [r12, #20]            /* Store v1[1]                         */
    strh   r6  , [r12, #60]            /* Store v1[3]                         */

    /* Update state index ................................................... */

    ldr    r1  , [r0, #-4]             /* r1 = state->idx                     */
    cmp    r1  , #9                    /* Compare state->idx                  */
    ite    lt                          /* If-Then-Else ...                    */
    addlt  r2  , r1  , #1              /* ... r2 = state->idx < 9 ?           */
    movge  r2  , #0                    /* ...      state->idx + 1 : 0         */
    str    r2  , [r0, #-4]             /* state->idx = r2                     */

    /* Window and output samples ............................................ */

    mov    r12 , r0                    /* r12 = &state->v[0]                  */
    ands   r11 , r1  , #1              /* r11 = state->idx & 1, Z = r11 EQ 0  */
    itte   ne                          /* If-Then-Then-Else ...               */
    addne  r12 , r12 , #160            /* ... r12 = &state->v[1]              */
    adrne  r14 , win_table_4_1         /* ... r14 = &win_table_0              */
    adreq  r14 , win_table_4_0         /* ... r14 = &win_table_1              */

    sub    r1  , r1  , r11             /* r1  = state->idx &~ 1               */
    add    r14 , r14 , r1  , LSL 1     /* r14 = &window + state->idx          */

    pop    { r1 }                      /* r1  = out_ptr                       */
    ldr    r2  , [sp, #36]             /* r2  = pitch                         */

    mov    r0  , #0x1000               /* r0  = 1 << 12                       */

    ldm    r12!, { r4-r7  }            /* r4-r7  = u[0][0-7]                  */
    ldm    r14!, { r8-r11 }            /* r8-r11 = w[0][0-7]                  */

    smlad  r3  , r4  , r8   , r0       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r5  , r9   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r6  , r10  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r7  , r11  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ldm    r12!, { r4-r7 }             /* r4-r7  = u[0][8-9], u[1][0-5]       */
    ldr    r8  , [r14], #20            /* r8     = w[0][8-9], r14 = &w[1]     */
    ldm    r14!, { r9-r11 }            /* r9-r11 = w[1][0-5]                  */

    smlad  r3  , r4  , r8  , r3        /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ssat   r3  , #16 , r3  , ASR #13   /* r3    = s[0]                        */
    strh   r3  , [r1]                  /* *out  = s[0]                        */
    add    r1  , r1  , r2  , LSL #1    /* out  += pitch                       */

    smlad  r3  , r5  , r9  , r0        /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r6  , r10 , r3        /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r7  , r11 , r3        /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ldm    r12!, { r4-r7 }             /* r4-r7   = u[1][6-9], u[2][0-3]      */
    ldrd   r8  , r9  , [r14], #24      /* r10-r11 = w[1][6-9], r14 = &w[2]    */
    ldm    r14!, { r10-r11 }           /* r9-r11  = w[2][0-3]                 */

    smlad  r3  , r4  , r8   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r5  , r9   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ssat   r3  , #16 , r3  , ASR #13   /* r3    = s[1]                        */
    strh   r3  , [r1]                  /* *out  = s[1]                        */
    add    r1  , r1  , r2  , LSL #1    /* out  += pitch                       */

    smlad  r3  , r6  , r10 , r0        /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r7  , r11 , r3        /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ldm    r12!, { r4-r7 }             /* r4-r7   = u[2][4-9], u[3][0-1]      */
    ldm    r14!, { r8-r9 }             /* r8-r9   = w[2][4-7]                 */
    ldr    r10 , [r14], #20            /* r10     = w[2][8-9], r14 = &w[3]    */
    ldr    r11 , [r14], #4             /* r11     = w[3][0-1]                 */

    smlad  r3  , r4  , r8   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r5  , r9   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r6  , r10  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ssat   r3  , #16 , r3  , ASR #13   /* r3    = s[2]                        */
    strh   r3  , [r1]                  /* *out  = s[2]                        */
    add    r1  , r1  , r2  , LSL #1    /* out  += pitch                       */

    smlad  r3  , r7  , r11  , r0       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ldm    r12!, { r4-r7  }            /* r4-r7   = u[3][2-9]                 */
    ldm    r14!, { r8-r10 }            /* r9-r11  = w[3][2-7]                 */
    ldr    r11 , [r14]                 /* r10     = w[3][8-9], r14 = &w[4]    */

    smlad  r3  , r4  , r8   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r5  , r9   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r6  , r10  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r7  , r11  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ssat   r3  , #16 , r3  , ASR #13   /* r3    = s[3]                        */
    strh   r3  , [r1]                  /* *out  = s[3]                        */

    pop    { r4-r11, pc }              /* Pop registers and return            */


/**
 * Synthesize samples of a 8 subbands block
 * state         <r0>    Previous transformed samples of the channel
 * in            <r1>    Sub-band input samples
 * scale         <r2>    Scale factor of samples
 * out           <r3>    Output adress of PCM samples
 * pitch         <sp+4>  Number of PCM samples between two consecutive
 */

    .text

    .align  4

cos_table_8:
    .short  5793, -5793, -8192, -8192
    .short  7568, -7568,  3135, -3135
    .short  4551, -6811,  1598, -8035

win_table_8_0:
    .short     0, -132,  -371, -848, -4456, -9631, 4456,  -848,  371, -132
    .short     0, -132,  -371, -848, -4456, -9631, 4456,  -848

    .short   -10, -138,  -526, -580, -5438, -9528, 3486, -1004,  229, -117
    .short   -10, -138,  -526, -580, -5438, -9528, 3486, -1004

    .short   -22, -131,  -685, -192, -6395, -9224, 2561, -1063,  108,  -97
    .short   -22, -131,  -685, -192, -6395, -9224, 2561, -1063

    .short   -36, -106,  -835,  322, -7287, -8734, 1711, -1042,   12,  -75
    .short   -36, -106,  -835,  322, -7287, -8734, 1711, -1042

    .short   -54,  -59,  -960,  959, -8078, -8078,  959,  -960,  -59,  -54
    .short   -54,  -59,  -960,  959, -8078, -8078,  959,  -960

    .short   -75,   12, -1042, 1711, -8734, -7287,  322,  -835, -106,  -36
    .short   -75,   12, -1042, 1711, -8734, -7287,  322,  -835

    .short   -97,  108, -1063, 2561, -9224, -6395, -192,  -685, -131,  -22
    .short   -97,  108, -1063, 2561, -9224, -6395, -192,  -685

    .short  -117,  229, -1004, 3486, -9528, -5438, -580,  -526, -138,  -10
    .short  -117,  229, -1004, 3486, -9528, -5438, -580,  -526

win_table_8_1:
    .short        -132,  -371, -848, -4456, -9631, 4456,  -848,  371, -132
    .short     0, -132,  -371, -848, -4456, -9631, 4456,  -848,  371

    .short        -138,  -526, -580, -5438, -9528, 3486, -1004,  229, -117
    .short   -10, -138,  -526, -580, -5438, -9528, 3486, -1004,  229

    .short        -131,  -685, -192, -6395, -9224, 2561, -1063,  108,  -97
    .short   -22, -131,  -685, -192, -6395, -9224, 2561, -1063,  108

    .short        -106,  -835,  322, -7287, -8734, 1711, -1042,   12,  -75
    .short   -36, -106,  -835,  322, -7287, -8734, 1711, -1042,   12

    .short         -59,  -960,  959, -8078, -8078,  959,  -960,  -59,  -54
    .short   -54,  -59,  -960,  959, -8078, -8078,  959,  -960,  -59

    .short          12, -1042, 1711, -8734, -7287,  322,  -835, -106,  -36
    .short   -75,   12, -1042, 1711, -8734, -7287,  322,  -835, -106

    .short         108, -1063, 2561, -9224, -6395, -192,  -685, -131,  -22
    .short   -97,  108, -1063, 2561, -9224, -6395, -192,  -685, -131

    .short         229, -1004, 3486, -9528, -5438, -580,  -526, -138,  -10
    .short  -117,  229, -1004, 3486, -9528, -5438, -580,  -526, -138

    .thumb_func
    .global sbc_synthesize_8

sbc_synthesize_8:

    push   { r3-r11, lr }              /* Push registers                      */

    /* DCT of subbands samples .............................................. */

    ldm    r1  , { r8-r11 }            /* r8-11 = s[0-7]                      */

    shasx  r6  , r8  , r11             /* r6  = s16 : d07                     */
    shasx  r7  , r9  , r10             /* r7  = s34 : d25                     */
    shsax  r8  , r8  , r11             /* r8  = d16 : s07                     */
    shsax  r9  , r9  , r10             /* r9  = d34 : s25                     */

    pkhtb  r4  , r7  , r8              /* r4  = s34 : s07                     */
    pkhtb  r5  , r6  , r9              /* r5  = s16 : s25                     */
    pkhtb  r6  , r9  , r6              /* r6  = d34 : d07                     */
    pkhtb  r7  , r8  , r7              /* r7  = d16 : d25                     */

    ldr    r1  , [r0], #4              /* r1  = state->idx                    */
    mov    r11 , r0                    /* r11 = &state->v[0]                  */
    mov    r12 , r0                    /* r12 = &state->v[0]                  */
    ands   r10 , r1  , #1              /* r10 = state->idx & 1, Z = r10 EQ 0  */
    ite    ne                          /* If-Then-Else ...                    */
    addne  r11 , r11 , #160            /* ... r11 = &state->v[1]              */
    addeq  r12 , r12 , #160            /* ... r12 = &state->v[1]              */

    cmp    r1  , #0                    /* Compare state->idx                  */
    ite    ne                          /* If-Then-Else ...                    */
    rsbne  r1  , r1  , #10             /* ... r1 = state->idx ?               */
    moveq  r1  , #0                    /* ...      10 - state->idx : 0        */
    add    r11 , r11 , r1  , LSL #1    /* r11 = &state->v[ odd][0][idx]       */
    add    r12 , r12 , r1  , LSL #1    /* r12 = &state->v[!odd][0][idx]       */

    adr    r14 , cos_table_8           /* r14 = &cos_table                    */

    add    r2  , r2 , #12              /* r2  = scale + 12 = shr              */
    mov    r1  , #1                    /* r1 = 1                              */
    sub    r8  , r2  , #1              /* r8 = shr - 1                        */
    lsl    r1  , r1  , r8              /* r1 = 1 << (shr - 1)                 */

    ldm    r14!, { r9-r10 }            /* r9-r10 = cos( 4, -4, -0, -0 )       */

    smlsd  r8  , r4  , r9  , r1        /* r8 +=  s07 cos(4) - -s34 cos(4)     */
    smlsdx r8  , r5  , r9  , r8        /* r8 += -s25 cos(4) -  s16 cos(4)     */
    asr    r8  , r2                    /* r8  = a0                            */
    rsb    r3  , r8  , #0              /* r3  = -a0                           */

    ssat   r8  , #16 , r8              /* r8  = sat( a0)                      */
    strh   r8  , [r11, #0]             /* Store v0[0]                         */

    ssat   r8  , #16 , r3              /* r8  = sat(-a0)                      */
    strh   r8  , [r12, #0]             /* Store v1[0]                         */

    smlad  r8  , r4  , r10 , r1        /* r8 += -s07 cos(0) + -s16 cos(0)     */
    smlad  r8  , r5  , r10 , r8        /* r8 += -s25 cos(0) + -s34 cos(0)     */
    asr    r8  , r2                    /* r8  = b3                            */
    ssat   r8  , #16 , r8              /* r8  = sat(b3)                       */
    strh   r8  , [r12, #80]            /* Store v1[4]                         */

    ldm    r14!, { r9-r10 }            /* r9-r10 = cos16( 2, -2, 6, -6 )      */

    smlad  r8  , r4  , r10 , r1        /* r8 +=  s07 cos(6) + -s34 cos(6)     */
    smlad  r8  , r5  , r9  , r8        /* r8 +=  s25 cos(2) + -s16 cos(2)     */
    asr    r8  , r2                    /* r8  = a2                            */
    rsb    r3  , r8  , #0              /* r3  = -a2                           */

    ssat   r8  , #16 , r8              /* r8  = sat(a2)                       */
    strh   r8  , [r11, #40]            /* Store v0[2]                         */

    ssat   r8  , #16 , r3              /* r8  = sat(-a2)                      */
    strh   r8  , [r11, #120]           /* Store v0[6]                         */

    smladx r8  , r4  , r9  , r1        /* r8 += -s07 cos(2) +  s34 cos(2)     */
    smlad  r8  , r5  , r10 , r8        /* r8 +=  s25 cos(6) -  s16 cos(6)     */
    asr    r8  , r2                    /* r8  = b1                            */
    ssat   r8  , #16 , r8              /* r8  = sat(b1)                       */
    strh   r8  , [r12,  #40]           /* Store v1[2]                         */
    strh   r8  , [r12, #120]           /* Store v1[6]                         */

    ldm    r14!, { r9-r10 }            /* r9-r10 = cos( 5, -3, 7, -1 )        */

    smlsd  r8  , r6  , r9  , r1        /* r8 +=  d07 cos(5) - -d34 cos(3)     */
    smlad  r8  , r7  , r10 , r8        /* r8 +=  d25 cos(7) + -d16 cos(1)     */
    asr    r8  , r2                    /* r8  = a1                            */
    rsb    r3  , r8  , #0              /* r3  = -a1                           */

    ssat   r8  , #16 , r8              /* r8  = sat(a1)                       */
    strh   r8  , [r11, #20]            /* Store v0[1]                         */

    ssat   r8  , #16 , r3              /* r8  = sat(-a1)                      */
    strh   r8  , [r11, #140]           /* Store v0[7]                         */

    smladx r8  , r6  , r9  , r1        /* r8 += -d07 cos(3) +  d34 cos(5)     */
    smlsdx r8  , r10 , r7  , r8        /* r8 +=  d16 cos(7) - -d25 cos(1)     */
    asr    r8  , r2                    /* r8  = b0                            */
    ssat   r8  , #16 , r8              /* r8  = sat(b0)                       */
    strh   r8  , [r12,  #20]           /* Store v1[1]                         */
    strh   r8  , [r12, #140]           /* Store v1[7]                         */

    mov    r8  , #0                    /* r14 = 0                             */
    ssub16 r9  , r8  , r9              /* r9  = cos( -5, 3 )                  */

    smlad  r8  , r6  , r10 , r1        /* r8 +=  d07 cos(7) + -d34 cos(1)     */
    smladx r8  , r7  , r9  , r8        /* r8 +=  d25 cos(3) + -d16 cos(5)     */
    asr    r8  , r2                    /* r8  = a3                            */
    rsb    r3  , r8  , #0              /* r3  = -a3                           */

    ssat   r8  , #16 , r8              /* r8  = sat(a3)                       */
    strh   r8  , [r11, #60]            /* Store v0[3]                         */

    ssat   r8  , #16 , r3              /* r8  = sat(-a3)                      */
    strh   r8  , [r11, #100]           /* Store v0[5]                         */

    smlsdx r8  , r6  , r10 , r1        /* r8 += -d07 cos(1) -  d34 cos(7)     */
    smlsd  r8  , r7  , r9  , r8        /* r8 += -d25 cos(5) -  d16 cos(3)     */
    asr    r8  , r2                    /* r8  = b2                            */
    ssat   r8  , #16 , r8              /* r8  = sat(b2)                       */
    strh   r8  , [r12,  #60]           /* Store v1[3]                         */
    strh   r8  , [r12, #100]           /* Store v1[5]                         */

    /* Update state index ................................................... */

    ldr    r1  , [r0, #-4]             /* r1 = state->idx                     */
    cmp    r1  , #9                    /* Compare state->idx                  */
    ite    lt                          /* If-Then-Else ...                    */
    addlt  r2  , r1  , #1              /* ... r2 = state->idx < 9 ?           */
    movge  r2  , #0                    /* ...      state->idx + 1 : 0         */
    str    r2  , [r0, #-4]             /* state->idx = r2                     */

    /* Window and output samples ............................................ */

    mov    r12 , r0                    /* r12 = &state->v[0]                  */
    ands   r11 , r1  , #1              /* r11 = state->idx & 1, Z = r11 EQ 0  */
    itte   ne                          /* If-Then-Then-Else ...               */
    addne  r12 , r12 , #160            /* ... r12 = &state->v[1]              */
    adrne  r14 , win_table_8_1         /* ... r14 = &win_table_0              */
    adreq  r14 , win_table_8_0         /* ... r14 = &win_table_1              */

    sub    r1  , r1  , r11             /* r1  = state->idx &~ 1               */
    add    r14 , r14 , r1  , LSL 1     /* r14 = &window + state->idx          */

    pop    { r1 }                      /* r1  = out_ptr                       */
    ldr    r2  , [sp, #36]             /* r2  = pitch                         */

    mov    r0  , #0x1000               /* r0  = 1 << 12                       */

    ldm    r12!, { r4-r7  }            /* r4-r7  = u[0][0-7]                  */
    ldm    r14!, { r8-r11 }            /* r8-r11 = w[0][0-7]                  */

    smlad  r3  , r4  , r8   , r0       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r5  , r9   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r6  , r10  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r7  , r11  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ldm    r12!, { r4-r7 }             /* r4-r7  = u[0][8-9], u[1][0-5]       */
    ldr    r8  , [r14], #20            /* r8     = w[0][8-9], r14 = &w[1]     */
    ldm    r14!, { r9-r11 }            /* r9-r11 = w[1][0-5]                  */

    smlad  r3  , r4  , r8  , r3        /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ssat   r3  , #16 , r3  , ASR #13   /* r3    = s[0]                        */
    strh   r3  , [r1]                  /* *out  = s[0]                        */
    add    r1  , r1  , r2  , LSL #1    /* out  += pitch                       */

    smlad  r3  , r5  , r9  , r0        /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r6  , r10 , r3        /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r7  , r11 , r3        /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ldm    r12!, { r4-r7 }             /* r4-r7   = u[1][6-9], u[2][0-3]      */
    ldrd   r8  , r9  , [r14], #24      /* r10-r11 = w[1][6-9], r14 = &w[2]    */
    ldm    r14!, { r10-r11 }           /* r9-r11  = w[2][0-3]                 */

    smlad  r3  , r4  , r8   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r5  , r9   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ssat   r3  , #16 , r3  , ASR #13   /* r3    = s[1]                        */
    strh   r3  , [r1]                  /* *out  = s[1]                        */
    add    r1  , r1  , r2  , LSL #1    /* out  += pitch                       */

    smlad  r3  , r6  , r10 , r0        /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r7  , r11 , r3        /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ldm    r12!, { r4-r7 }             /* r4-r7   = u[2][4-9], u[3][0-1]      */
    ldm    r14!, { r8-r9 }             /* r8-r9   = w[2][4-7]                 */
    ldr    r10 , [r14], #20            /* r10     = w[2][8-9], r14 = &w[3]    */
    ldr    r11 , [r14], #4             /* r11     = w[3][0-1]                 */

    smlad  r3  , r4  , r8   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r5  , r9   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r6  , r10  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ssat   r3  , #16 , r3  , ASR #13   /* r3    = s[2]                        */
    strh   r3  , [r1]                  /* *out  = s[2]                        */
    add    r1  , r1  , r2  , LSL #1    /* out  += pitch                       */

    smlad  r3  , r7  , r11  , r0       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ldm    r12!, { r4-r7  }            /* r4-r7   = u[3][2-9]                 */
    ldm    r14!, { r8-r10 }            /* r9-r11  = w[3][2-7]                 */
    ldr    r11 , [r14], #20            /* r10     = w[3][8-9], r14 = &w[4]    */

    smlad  r3  , r4  , r8   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r5  , r9   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r6  , r10  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r7  , r11  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ssat   r3  , #16 , r3  , ASR #13   /* r3    = s[3]                        */
    strh   r3  , [r1]                  /* *out  = s[3]                        */
    add    r1  , r1  , r2  , LSL #1    /* out  += pitch                       */

    ldm    r12!, { r4-r7  }            /* r4-r7  = u[4][0-7]                  */
    ldm    r14!, { r8-r11 }            /* r8-r11 = w[4][0-7]                  */

    smlad  r3  , r4  , r8   , r0       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r5  , r9   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r6  , r10  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r7  , r11  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ldm    r12!, { r4-r7 }             /* r4-r7  = u[4][8-9], u[5][0-5]       */
    ldr    r8  , [r14], #20            /* r8     = w[4][8-9], r14 = &w[5]     */
    ldm    r14!, { r9-r11 }            /* r9-r11 = w[5][0-5]                  */

    smlad  r3  , r4  , r8  , r3        /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ssat   r3  , #16 , r3  , ASR #13   /* r3    = s[4]                        */
    strh   r3  , [r1]                  /* *out  = s[4]                        */
    add    r1  , r1  , r2  , LSL #1    /* out  += pitch                       */

    smlad  r3  , r5  , r9  , r0        /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r6  , r10 , r3        /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r7  , r11 , r3        /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ldm    r12!, { r4-r7  }            /* r4-r7   = u[5][6-9], u[6][0-3]      */
    ldrd   r8  , r9  , [r14], #24      /* r10-r11 = w[5][6-9], r14 = &w[6]    */
    ldm    r14!, { r10-r11 }           /* r9-r11  = w[6][0-3]                 */

    smlad  r3  , r4  , r8   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r5  , r9   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ssat   r3  , #16 , r3  , ASR #13   /* r3    = s[5]                        */
    strh   r3  , [r1]                  /* *out  = s[5]                        */
    add    r1  , r1  , r2  , LSL #1    /* out  += pitch                       */

    smlad  r3  , r6  , r10 , r0        /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r7  , r11 , r3        /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ldm    r12!, { r4-r7 }             /* r4-r7   = u[6][4-9], u[7][0-1]      */
    ldm    r14!, { r8-r9 }             /* r8-r9   = w[6][4-7]                 */
    ldr    r10 , [r14], #20            /* r10     = w[6][8-9], r14 = &w[7]    */
    ldr    r11 , [r14], #4             /* r11     = w[7][0-1]                 */

    smlad  r3  , r4  , r8   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r5  , r9   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r6  , r10  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ssat   r3  , #16 , r3  , ASR #13   /* r3    = s[6]                        */
    strh   r3  , [r1]                  /* *out  = s[6]                        */
    add    r1  , r1  , r2  , LSL #1    /* out  += pitch                       */

    smlad  r3  , r7  , r11  , r0       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ldm    r12!, { r4-r7  }            /* r4-r7   = u[7][2-9]                 */
    ldm    r14!, { r8-r11 }            /* r9-r11  = w[7][2-9]                 */

    smlad  r3  , r4  , r8   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r5  , r9   , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r6  , r10  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */
    smlad  r3  , r7  , r11  , r3       /* r3 += *(u++) *(w++) + *(u++) *(w++) */

    ssat   r3  , #16 , r3  , ASR #13   /* r3    = s[7]                        */
    strh   r3  , [r1]                  /* *out  = s[7]                        */

    pop    { r4-r11, pc }              /* Pop registers and return            */
