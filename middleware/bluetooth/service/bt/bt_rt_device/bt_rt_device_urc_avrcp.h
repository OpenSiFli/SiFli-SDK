/**
  ******************************************************************************
  * @file   bt_rt_device_urc_avrcp.h
  * @author Sifli software development team
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2022 - 2022,  Sifli Technology
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
#ifndef _BT_RT_DEVICE_URC_AVRCP_H
#define _BT_RT_DEVICE_URC_AVRCP_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bts2_bt.h"


extern    void urc_func_bt_avrcp_open_complete_sifli(void);
extern    void urc_func_bt_avrcp_close_complete_sifli(void);
extern    void urc_func_bt_avrcp_playback_status_sifli(uint8_t play_status);
extern    void urc_func_bt_avrcp_volume_change_rigister_sifli(void);
extern    void urc_func_bt_avrcp_track_change_sifli(uint8_t track_change);
extern    void urc_func_bt_avrcp_mp3_detail_sifli(bt_mp3_detail_info_t *detail);
extern    void urc_func_bt_avrcp_song_progress_sifli(uint32_t progress);
extern    void urc_func_bt_avrcp_absolute_volume_sifli(uint8_t volume);
extern    int bt_sifli_notify_avrcp_event_hdl(uint16_t event_id, uint8_t *data, uint16_t data_len);

#endif /* _BT_RT_DEVICE_URC_AVRCP_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

