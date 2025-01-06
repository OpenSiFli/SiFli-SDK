/**
  ******************************************************************************
  * @file   hci_drv.h
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

#ifndef _HCI_DRV_H_
#define _HCI_DRV_H_

#include "bts2_type.h"
/* this struct be defined in driver */
#ifndef BTS2S_RECV_MSG_DEF
#define BTS2S_RECV_MSG_DEF
typedef struct
{
    char *buf;
    unsigned buflen;
    unsigned dex;
} BTS2S_RECV_DATA;
#endif

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *      void
 *
 * OUTPUT:
 *      void
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/

void hcid_reset(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      received a HCI cmd and send it.
 *
 * INPUT:
 *      U8 *msg
 *      U16 len
 *
 * OUTPUT:
 *      void
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/

BOOL hcid_send_hci_cmd(U8 *cmd, U16 len);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      received an acl data pkt and send it.
 *
 * INPUT:
 *      U8 *msg
 *      U16 len
 *
 * OUTPUT:
 *      void
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/

BOOL hcid_send_acl_data(U8 *data, U16 len);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      received a sco data pkt and send it.
 *
 * INPUT:
 *      U8 *msg
 *      U16 len
 *
 * OUTPUT:
 *      void
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/

BOOL hcid_send_sco_data(U8 *data, U16 len);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function used to send HCI vendor command out.
 *
 * INPUT:
 *      U8 *cmd
 *      U16 len
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void hcid_send_vendor_cmd(U8 *cmd, U16 len);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *    U8 ch
 *    U8 type
 *    U8 *msg
 *    U16 len
 *
 * OUTPUT:
 *    void.
 *
 * NOTE:
 *    hcid_bcsp_send(BCSP_CHNL_HQ, BCSP_RELIABLE_CHNL, msg, len) can be used
 *    to send HQ command.
 *
 *----------------------------------------------------------------------------*/
void hcid_bcsp_send(U8 ch, U8 type, U8 *data, U16 len);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The function used to handle all type of data from controller.
 *
 * INPUT:
 *      void *msg
 *      U16 chnl
 *      U16 rel
 * OUTPUT:
 *      void
 *
 * NOTE:
 *      none.
 *----------------------------------------------------------------------------*/

void hcid_recv_data(void *msg, U8 chnl, U16 rel);
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
