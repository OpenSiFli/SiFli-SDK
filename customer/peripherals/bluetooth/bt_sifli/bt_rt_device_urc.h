/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BT_RT_DEVICE_URC_H
#define _BT_RT_DEVICE_URC_H

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bts2_bt.h"

extern    rt_bt_device_t *urc_func_bt_get_device(void);
extern    uint8_t urc_func_bt_profile_channel_by_conn_idx(uint8_t profile, uint8_t profile_channel);
extern    uint8_t urc_func_bt_get_conn_idx(bt_mac_t *mac);
extern    void urc_func_inq_sifli(uint8_t *adrr, uint32_t nameSize, char *name, uint32_t dev_cls, int rssi);
extern    void urc_func_inq_finished_sifli(void);
void      urc_func_profile_conn_sifli(bt_notify_profile_state_info_t *profile_info, uint8_t profile);
extern    void urc_func_profile_disc_sifli(uint8_t *addr, bt_profile_t profile, uint8_t reason);
extern    void urc_func_disc_sifli(uint8_t *addr, uint8_t reason);
void urc_func_key_missing_sifli(uint8_t *addr);
void urc_func_encryption_sifli(uint8_t *addr);
extern    void urc_func_call_link_ested_sifli(bt_notify_device_sco_info_t *sco_info);
extern    void urc_func_link_down_sifli(bt_notify_device_sco_info_t *sco_info);
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

