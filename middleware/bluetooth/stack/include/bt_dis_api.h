/**
  ******************************************************************************
  * @file   bt_dis_api.h
  * @author Sifli software development team
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2024 - 2024,  Sifli Technology
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

#ifndef _BT_DIS_API_H_
#define _BT_DIS_API_H_

#ifdef __cplusplus
extern "C" {
#endif

// customize
#if !defined(DIS_VENDOR_ID)
#define DIS_VENDOR_ID  0x0AC4   //SiFli Technologies (shanghai) Inc
#endif
#if !defined(DIS_PRODUCT_ID)
#define DIS_PRODUCT_ID 0x0000
#endif
#if !defined(DIS_PRODUCT_VERSION)
#define DIS_PRODUCT_VERSION 0x001F  //0.1.F
#endif

// values should keep untouched
#define DIS_VENDID_SOURCE_BTSIG        0x0001
#define DIS_VENDID_SOURCE_USBIF        0x0002


typedef enum
{
    BTS2MD_BT_DIS_SDP_REG_REQ = BTS2MD_START,//0x00
    BTS2MD_BT_DIS_SDP_UNREG_REQ,
} bt_dis_event_t;

typedef enum
{
    BTS2MU_BT_DIS_SDP_RES = BTS2MU_START,
    BTS2MU_BT_DIS_SDP_UNRES,
} bt_dis_response_t;

typedef struct
{
    uint8_t  prim_rec;
    uint16_t spec_id;
    uint16_t prod_id;
    uint16_t prod_ver;
    uint16_t vend_id;
    uint16_t vend_id_source;
} bt_dis_device_info_t;

typedef struct
{
    U16 type;
    bt_dis_device_info_t dis_info;
} bt_dis_sdp_reg_req_t;

typedef struct
{
    U16 type;
    uint32_t sdp_sec_hdl;
} bt_dis_sdp_unreg_req_t;

typedef struct
{
    U16 type;
    uint8_t res;
    uint32_t sdp_sec_hdl;
} bt_dis_sdp_reg_cfm_t;

void bt_dis_sdp_req_api(bt_dis_device_info_t *dis_info);
void bt_dis_sdp_unreq_api(U32 sdp_rec_hdl);
void bt_dis_sdp_status_cfm(U16 type, U32 sdp_sec_hdl, U8 res);

#ifdef __cplusplus
}
#endif
#endif

