/**
  ******************************************************************************
  * @file   bts2_buff.h
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

#ifndef _BTS2_BUFF_H_
#define _BTS2_BUFF_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct BTS2S_BUFF_POOL_TAG
{
    U16 wr_hdl;
    U16 rd_hdl;
    U32 size;
    U8  *mem;
} BTS2S_BUFF_POOL;

typedef struct BTS2S_BUFF_PATCH_TAG
{
    BTS2S_BUFF_POOL *pool;
    U32            start;
    U32            len;
    BOOL           bfirst;
} BTS2S_BUFF_PATCH;

typedef struct BTS2S_BUFF_TAG
{
    U32         offset;
    U32         len;
    U8          bnep_network;
    U8          from_bnep_clt;
    U8          bnep_network_special; /* if it is bnep_network,does first word only contain an octet? */
    U8          *data;
    void        *rel;
    U8          *data_orig;
    BOOL        use_offset;
    U16         len_orig;

    BTS2S_BUFF_PATCH *patch;
    struct BTS2S_BUFF_TAG *prev;
    struct BTS2S_BUFF_TAG *next;
    U8          flag;
} BTS2S_BUFF;

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Initialize the BUFF module.
 *
 * INPUT:
 *      void
 *
 * OUTPUT:
 *      void
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void buff_init(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Create a new BUFF with the given size.
 *
 * INPUT:
 *      U32 size.
 * OUTPUT:
 *      Pointer of the new BUFF or NULL if failed.
 *
 * NOTE:
 *      none.
 *
*----------------------------------------------------------------------------*/
BTS2S_BUFF *buff_malloc(U32 size);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Free the input BUFF list.
 *
 * INPUT:
 *      BTS2S_BUFF *buff:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void buff_free(BTS2S_BUFF *buff);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Read the specified length of data from the LIST to BUFF.
 *
 * INPUT:
 *      BTS2S_BUFF **list
 *      U8        *buff
 *      U32       size
 *
 * OUTPUT:
 *      the length of red data.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U32 buff_read(BTS2S_BUFF **list,
              U8 *buff,
              U32 size);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Read the specified length of data from LIST to BUFF with starting
 *      from position OFFSET bytes from the start of list.
 *
 * INPUT:
 *      const BTS2S_BUFF *list,
 *      U32             offset
 *      U8              *buff
 *      U32             size
 * OUTPUT:
 *      the length of red data.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U32 buff_get(const BTS2S_BUFF *list,
             U32 offset,
             U8 *buff,
             U32 size);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      ADD the new BUFF to the end of LIST.
 *
 * INPUT:
 *      BTS2S_BUFF **list
 *      BTS2S_BUFF *buff
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
*
*----------------------------------------------------------------------------*/
void buff_add(BTS2S_BUFF **list, BTS2S_BUFF *buff);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Get the pointer of BUFF data with OFFSET.
 *
 * INPUT:
 *      const BTS2S_BUFF *buff
 *
 * OUTPUT:
 *      U8 *: pointer of data with offset.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 *buff_map(const BTS2S_BUFF *buff);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Copy one BUFF to another new allocated one.
 *
 * INPUT:
 *      const BTS2S_BUFF *buff:
 *
 * OUTPUT:
 *      Pointer of the new BUFF.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
BTS2S_BUFF *buff_copy(const BTS2S_BUFF *buff);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Find the related BUFF handle with the specified connection handle.
 *
 * INPUT:
 *      U16 conn_hdl
 *
 * OUTPUT:
 *      BUFF handle if found, otherwise NULL.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
BTS2S_BUFF_POOL *buff_get_hdl(U16 conn_hdl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Get the total length of a BUFF list.
 *
 * INPUT:
 *      BTS2S_BUFF *list
 *
 * OUTPUT:
 *      the length of the BUFF list.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U32 buff_get_len(BTS2S_BUFF *list);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Set the read position of LIST.
 *
 * INPUT:
 *      BTS2S_BUFF **list
 *      U16 pos
 *
 * OUTPUT:
 *      byte number
 *
 * NOTE:
 *      none.
 *
*----------------------------------------------------------------------------*/
U32 buff_set_rd_pos(BTS2S_BUFF **list, U32 pos);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      BUFF patch to data stream.
 *
 * INPUT:
 *      BTS2S_BUFF *buff
 *      BOOL flag
 *
 * OUTPUT:
 *      data pointer.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 *buff_patch_2_stream(BTS2S_BUFF *buff, BOOL flag);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      BUFF to data stream.
 *
 * INPUT:
 *      BTS2S_BUFF *list
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void buff_convert_patch(BTS2S_BUFF *list);

#ifdef CFG_PAN
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
BTS2S_BUFF *buff_new_in(U16 data_len, BOOL special);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
BTS2S_BUFF *buff_insert(BTS2S_BUFF *p_first, BTS2S_BUFF *p_rest);

#endif

#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
