/**
  ******************************************************************************
  * @file   av_sbc_api.h
  * @author Sifli software development team
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _BTS2_SBC_API_H_
#define _BTS2_SBC_API_H_

#include "bts2_type.h"

#ifdef __cplusplus
extern "C" {
#endif


#define SBC_MAX_BLOCKS            16
#define SBC_MAX_CHNLS             2
#define SBC_MAX_SUBBANDS          8

typedef enum
{
    SBC_MONO = 0,
    SBC_DUAL,
    SBC_STEREO,
    SBC_JOINT_STEREO
} BTS2E_SBC_CHNL_MODE;

typedef enum
{
    SBC_METHOD_LOUDNESS = 0,
    SBC_METHOD_SNR
} BTS2E_SBC_ALLOC_METHOD;

typedef struct
{
    U8  *psrc;
    U16 src_len;
    U16 src_len_used;
    U8  *pdst;
    U16 dst_len;
    U16 dst_len_used;
} BTS2S_SBC_STREAM;

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Calculate the bit pool value before encoding or decoding.
 *
 *
 * INPUT:
 *      U8 *bit_pool_alt
 *      U8 *toggle_period
 *      BTS2E_SBC_CHNL_MODE chnl_mode
 *      U16 sample_freq
 *      U8 nrof_blocks
 *      U8 nrof_subbands
 *      U16 bitrate
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 bts2_sbc_calc_bit_pool(U8 *bit_pool_alt,
                          U8 *toggle_period,
                          BTS2E_SBC_CHNL_MODE chnl_mode,
                          U16 sample_freq,
                          U8 nrof_blocks,
                          U8 nrof_subbands,
                          U16 bitrate);

U16 bts2_sbc_calculate_framelen(BTS2E_SBC_CHNL_MODE chnl_mode, U8 nrof_blocks, U8 nrof_subbands, U8 bitpool);
U16 bts2_sbc_calculate_pcm_samples_per_sbc_frame(U8 nrof_blocks, U8 nrof_subbands);

/*----------------------------------------------------------------------------*
*
* DESCRIPTION:
*       Configure the settings required before encoding.
*
* INPUT:
*      BTS2E_SBC_CHNL_MODE chnl_mode
*      BTS2E_SBC_ALLOC_METHOD alloc_method
*      U16 sample_freq
*      U8 nrof_blocks
*      U8 nrof_subbands
*      U8 bitpool
*
* OUTPUT:
*      Frame size in bytes if inputs are valid else returns zero.
*
* NOTE:
*      none.
*
*----------------------------------------------------------------------------*/
U16 bts2_sbc_encode_cfg(BTS2E_SBC_CHNL_MODE chnl_mode,
                        BTS2E_SBC_ALLOC_METHOD alloc_method,
                        U16 sample_freq,
                        U8 nrof_blocks,
                        U8 nrof_subbands,
                        U8 bitpool);

/*----------------------------------------------------------------------------*
*
* DESCRIPTION:
*       Configure the settings required before decoding.
*
* INPUT:
*      BTS2E_SBC_CHNL_MODE chnl_mode
*      BTS2E_SBC_ALLOC_METHOD alloc_method
*      U16 sample_freq
*      U8 nrof_blocks
*      U8 nrof_subbands
*      U8 bitpool
*
* OUTPUT:
*      If inputs are valid else returns FALSE.
*
* NOTE:
*      none.
*
*----------------------------------------------------------------------------*/
U8 bts2_sbc_decode_cfg(BTS2E_SBC_CHNL_MODE chnl_mode,
                       BTS2E_SBC_ALLOC_METHOD alloc_method,
                       U16 sample_freq,
                       U8 nrof_blocks,
                       U8 nrof_subbands,
                       U8 bitpool);

/*----------------------------------------------------------------------------*
*
* DESCRIPTION:
*      SBC encode.
*
*
*
* INPUT:
*      BTS2S_SBC_STREAM *pbss
*
*
* OUTPUT:
*      void.
*
* NOTE:
*      none.
*
*----------------------------------------------------------------------------*/
void bts2_sbc_encode(BTS2S_SBC_STREAM *pbss);

/*----------------------------------------------------------------------------*
*
* DESCRIPTION:
*      SBC decode.
*
*
*
* INPUT:
*      BTS2S_SBC_STREAM *pbss
*
*
* OUTPUT:
*      void.
*
* NOTE:
*      none.
*
*----------------------------------------------------------------------------*/
void bts2_sbc_decode(BTS2S_SBC_STREAM *pbss);

void bts2_sbc_encode_completed(void);


void bts2_sbc_decode_completed(void);


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------------*
*
* DESCRIPTION:
*       Configure the settings required before encoding.
*
* INPUT:
*
* OUTPUT:
*      Frame size in bytes if inputs are valid else returns zero.
*
* NOTE:
*      none.
*
*----------------------------------------------------------------------------*/
U16 bts2_msbc_encode_cfg(void);

/*----------------------------------------------------------------------------*
*
* DESCRIPTION:
*       Configure the settings required before decoding.
*
* INPUT:
*
* OUTPUT:
*      If inputs are valid else returns FALSE.
*
* NOTE:
*      none.
*
*----------------------------------------------------------------------------*/
U8 bts2_msbc_decode_cfg(void);

/*----------------------------------------------------------------------------*
*
* DESCRIPTION:
*      mSBC encode.
*
*
*
* INPUT:
*      BS_SBC_STREAM *pbss
*
*
* OUTPUT:
*      void.
*
* NOTE:
*      none.
*
*----------------------------------------------------------------------------*/
void bts2_msbc_encode(BTS2S_SBC_STREAM *pbss);

/*----------------------------------------------------------------------------*
*
* DESCRIPTION:
*      mSBC decode.
*
*
*
* INPUT:
*      BS_SBC_STREAM *pbss
*
*
* OUTPUT:
*      void.
*
* NOTE:
*      none.
*
*----------------------------------------------------------------------------*/
void bts2_msbc_decode(BTS2S_SBC_STREAM *pbss);


void bts2_msbc_encode_completed(void);


void bts2_msbc_decode_completed(void);


#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
