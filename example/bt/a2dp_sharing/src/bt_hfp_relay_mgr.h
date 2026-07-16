/*
 * SPDX-FileCopyrightText: 2026-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __BT_HFP_RELAY_MGR_H__
#define __BT_HFP_RELAY_MGR_H__

#include <rtthread.h>
#include "bf0_sibles.h"
#include "bts2_app_inc.h"
#include "bt_connection_manager.h"
#include "ulog.h"

#if defined(AUDIO_USING_MANAGER)
    #include "audio_server.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define BT_HFP_RELAY_INVALID_CHANNEL 0xffff

typedef struct
{
    uint16_t            type;
    uint16_t            event_id;
    uint16_t            data_len;
    uint8_t             *data;
} bt_hfp_relay_notify_data_t;

typedef enum
{
    BT_HFP_RELAY_CLCC_STATUS_START = 0x00,
    BT_HFP_RELAY_CLCC_STATUS_IN_PROGRESS,
    BT_HFP_RELAY_CLCC_STATUS_COMPLETE,
} bt_hfp_relay_clcc_status_t;

typedef struct
{
    uint16_t hf_channel;
    uint16_t ag_channel;
    hfp_cind_status_t cind_status;
    hfp_phone_number_t local_phone_num;
    hfp_phone_call_info_t remote_call;
    hfp_remote_calls_info_t remote_calls;
    uint8_t has_local_phone_num;
    uint8_t has_remote_call;
    uint8_t pending_ag_make_call;
    uint8_t phone_vgs;
    uint8_t phone_vgm;
    uint8_t phone_bsir;
    uint8_t phone_bvra;
    uint8_t hfp_ag_sco_state;
    bt_notify_device_mac_t hf_mac;
    bt_notify_device_mac_t ag_mac;
} bt_hfp_relay_context_t;

void bt_hfp_relay_mgr_init(void);

int bt_hfp_relay_hf_event_handle(bt_hfp_relay_notify_data_t *msg);
int bt_hfp_relay_ag_event_handle(bt_hfp_relay_notify_data_t *msg);

bt_hfp_relay_context_t *bt_hfp_relay_get_context(void);
uint16_t bt_hfp_relay_get_hf_channel(void);
uint16_t bt_hfp_relay_get_ag_channel(void);

void bt_hfp_relay_set_hf_channel(uint16_t channel, bt_notify_device_mac_t *mac);
void bt_hfp_relay_set_ag_channel(uint16_t channel, bt_notify_device_mac_t *mac);
void bt_hfp_relay_clear_hf_channel(uint16_t channel);
void bt_hfp_relay_clear_ag_channel(uint16_t channel);

void bt_hfp_relay_reset_cached_call(void);
void bt_hfp_relay_cache_local_phone_num(uint8_t *number, uint16_t number_len);
void bt_hfp_relay_update_cind_from_all_status(bt_notify_all_call_status *call_status);
void bt_hfp_relay_update_cind_by_indicator(uint8_t type, uint8_t val);
void bt_hfp_relay_notify_ag_call_state(void);
void bt_hfp_relay_cache_remote_call(bt_notify_clcc_ind_t *clcc_info);
void bt_hfp_relay_start_get_clcc(void);
void bt_hfp_relay_handle_at_cmd_cfm(uint8_t cmd_id, uint8_t res);
void bt_hfp_relay_handle_sco_event(uint16_t event_id, bt_notify_device_sco_info_t *sco_info);
#ifdef __cplusplus
}
#endif

#endif /* __BT_HFP_RELAY_MGR_H__ */