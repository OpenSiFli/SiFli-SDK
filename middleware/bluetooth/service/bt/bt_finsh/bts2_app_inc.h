/**
  ******************************************************************************
  * @file   bts2_app_inc.h
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

#ifndef _BTS2_APP_INC_H_
#define _BTS2_APP_INC_H_


#ifdef __cplusplus
extern "C" {
#endif
#include "rtconfig.h"
#if defined (BSP_USING_PC_SIMULATOR)
#include <sys\timeb.h>
#include <windows.h>
#include <process.h>
#include <errno.h>
#include <winnls.h>
#include <conio.h>
#else
#include "bf0_hal.h"
#endif

#include "bts2_global.h"
#include "bf0_ble_common.h"

/* standard library */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>
#include <ctype.h>
#include <time.h>
#include <sys/stat.h>

#if 0
#define CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#endif
#ifndef BSP_USING_PC_SIMULATOR
#include "bts2_file.h"
#endif

#include <errno.h>

/* BTS2 include header files */
#include "gap_api.h"
#include "sc_api.h"
#include "sdp_api.h"
#include "hci_api.h"

#ifdef CFG_TL_USB
#include "bts2_usb_sys.h"
#endif

#ifdef CFG_SPP_CLT
#include "spp_clt_api.h"
#endif

#ifdef CFG_SPP_SRV
#include "spp_srv_api.h"
#endif

#ifdef CFG_HFP_HF
#include "hfp_hf_api.h"
#include "hfp_type_api.h"
#endif

#ifdef CFG_AV
#include "av_api.h"
#include "av_sbc_api.h"
#endif

#ifdef CFG_AVRCP
#include "avrcp_api.h"
#endif

#ifdef CFG_HID
#include "hid_api.h"
#endif

#ifdef CFG_PAN
//#include "bnep.h"
#include "pan_api.h"
//#include "pan_inter_ui.h"
#endif
#ifdef CFG_PBAP_CLT
#include "pbap_public.h"
#include "pbap_clt_api.h"
#endif

#ifdef CFG_BR_GATT_SRV
#include "bt_gatt_api.h"
#include "bts2_app_bt_gatt.h"
#endif

#ifdef CFG_BT_DIS
#include "bts2_app_bt_dis.h"
#endif

#ifdef CFG_BT_L2CAP_PROFILE
#include "bt_l2cap_profile_api.h"
#include "bts2_app_bt_l2cap_profile.h"
#endif

/* app header files */
#include "bts2_app_demo.h"
#include "bts2_app_spp_c.h"
#include "bts2_app_spp_s.h"
#include "bts2_app_generic.h"
#include "bts2_app_menu.h"
#include "bts2_app_hfp_hf.h"
#include "bts2_app_av.h"
#include "bts2_app_av_snk.h"
#include "bts2_app_av_src.h"
#include "bts2_app_avrcp.h"
#include "bts2_app_hid.h"
#include "bts2_app_sys.h"
#include "bts2_app_pan.h"
#include "bts2_app_pbap_c.h"
#include "bts2_app_interface.h"
#include "bts2_app_hfp_ag.h"

#define SMS
#ifdef __cplusplus
}
#endif
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
