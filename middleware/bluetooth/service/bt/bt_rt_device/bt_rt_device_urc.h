/**
  ******************************************************************************
  * @file   bt_rt_device_urc.h
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
#ifndef _BT_RT_DEVICE_URC_H
#define _BT_RT_DEVICE_URC_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bts2_bt.h"


extern    void urc_func_inq_sifli(uint8_t *adrr, uint32_t nameSize, char *name, uint32_t dev_cls);
extern    void urc_func_inq_finished_sifli(void);
void urc_func_profile_conn_sifli(uint8_t *addr, uint8_t profile);
extern    void urc_func_profile_disc_sifli(uint8_t *addr, bt_profile_t profile, uint8_t reason);
extern    void urc_func_disc_sifli(uint8_t *addr, uint8_t reason);
void urc_func_key_missing_sifli(uint8_t *addr);
void urc_func_encryption_sifli(uint8_t *addr);
extern    void urc_func_call_link_ested_sifli(uint8_t res);
extern    void urc_func_link_down_sifli(void);
extern    void urc_func_bt_stack_ready_sifli(void);
extern    void urc_func_bt_addr_sifli(uint8_t *addr);
extern    uint8_t urc_func_bt_close_complete_sifli(void);
extern    void urc_func_rd_local_name_sifli(BTS2S_DEV_NAME local_name);
extern    void urc_func_rd_local_rssi_sifli(S8 rssi);
extern    void urc_func_acl_opened_ind_sifli(uint8_t *addr, uint8_t res, uint8_t incoming, uint32_t dev_cls);
void urc_func_pair_ind_sifli(uint8_t *addr, uint8_t result);
extern    void urc_func_change_bd_addr_sifli(uint32_t state);
extern    void urc_func_bt_pairing_state(uint8_t state);
extern    void urc_func_rmt_version_inq(bt_rmt_version_t *version);

#ifdef BT_USING_PAIRING_CONFIRMATION
    extern    void urc_func_io_capability_sifli(bt_mac_t *mac);
    extern    void urc_func_confirm_sifli(bt_pair_confirm_t *msg);
#endif

extern    void urc_func_key_overlaid_sifli(uint8_t *addr);
extern    void urc_func_bt_remote_name_sifli(uint8_t *addr, uint8_t res, BTS2S_DEV_NAME *name);

extern __ROM_USED void rt_bt_event_notify(bt_notify_t *param);


#endif /* _BT_RT_DEVICE_URC_H */

/************************ (C); COPYRIGHT Sifli Technology *******END OF FILE****/

