/**
  ******************************************************************************
  * @file   config.h
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




/*Host lib supported profiles*/
/*CFG_NO_L2C mean profile service can't work without gap_l2c*/
#ifndef CFG_NO_L2C
    #define CFG_AVRCP_LIB
    #define CFG_AV_LIB
    #define CFG_AV_SRC_LIB
    #define CFG_AV_SNK_LIB
    #define CFG_HID_LIB
    #define CFG_BR_GATT_SRV_LIB
#endif

#define CFG_SPP_CLT_LIB
#define CFG_SPP_SRV_LIB
#define CFG_HFP_HF_LIB
#define CFG_HFP_AG_LIB
#define CFG_PAN_LIB
#define CFG_PBAP_CLT_LIB
#define CFG_BT_DIS_LIB
#define CFG_BT_L2CAP_PROFILE_LIB

/*If not customized, use all profiles*/
#ifndef BT_PROFILE_CUSTOMIZE
    #ifndef CFG_NO_L2C
        #ifdef CFG_AVRCP_LIB
            #define CFG_AVRCP
        #endif
        #ifdef CFG_AV_LIB
            #define CFG_AV
        #endif
        #ifdef CFG_AV_SRC_LIB
            #define CFG_AV_SRC
        #endif
        #ifdef CFG_AV_SNK_LIB
            #define CFG_AV_SNK
        #endif
        #ifdef CFG_BR_GATT_SRV_LIB
            #define CFG_BR_GATT_SRV
        #endif
        #ifdef CFG_HID_LIB
            #define CFG_HID
        #endif
        #ifdef CFG_BT_L2CAP_PROFILE_LIB
            #define CFG_BT_L2CAP_PROFILE
        #endif
    #endif
    #ifdef CFG_SPP_CLT_LIB
        #define CFG_SPP_CLT
    #endif
    #ifdef CFG_SPP_SRV_LIB
        #define CFG_SPP_SRV
    #endif
    #ifdef CFG_HFP_HF_LIB
        #define CFG_HFP_HF
    #endif
    #ifdef CFG_HFP_AG_LIB
        #define CFG_HFP_AG
    #endif
    #ifdef CFG_PAN_LIB
        #define CFG_PAN
    #endif
    #ifdef CFG_PBAP_CLT_LIB
        #define CFG_PBAP_CLT
    #endif
    #ifdef CFG_BT_DIS_LIB
        #define CFG_BT_DIS
    #endif
#endif




/*
 * HID
 */


#define CFG_OPEN_SCAN
#ifdef CFG_AV_SNK
    #define CFG_OPEN_AVSNK
#endif
//#define CFG_OPEN_SDKRECONN
#ifdef CFG_AVRCP
    #define CFG_OPEN_AVRCP
#endif

#ifdef CFG_HID
    #define CFG_OPEN_HID
#endif
//#define CFG_OPNE_3WAY


#define CFG_MAX_ACL_CONN_NUM (7)
#define SPP_CLT_MAX_CONN_NUM 1
#define SPP_SRV_MAX_CONN_NUM 7



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
