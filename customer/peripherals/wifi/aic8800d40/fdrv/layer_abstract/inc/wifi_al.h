/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _WIFI_AL_H_
#define _WIFI_AL_H_

#include "co_list.h"
#include "userconfig.h"
#include "wifi_def.h"
#include "wifi_driver_event.h"

#define MACADDR_LEN 6
struct macaddr
{
    u16_l array[MACADDR_LEN/2];
};

struct aic_sta_info
{
    struct co_list_hdr list_hdr;
    uint8_t inst_nbr;
    uint8_t staid;
    bool valid;
    struct macaddr mac_addr;
    uint32_t last_active_time_us;
};

#define MAX_CHANNELS    14 // 2.4G
typedef struct _aic_config_scan_chan
{
    uint16_t action;
    uint16_t band;
    uint16_t total_chan;
    uint8_t  chan_num[MAX_CHANNELS];
} aic_config_scan_chan;

extern wifi_drv_event_cbk sp_aic_wifi_event_cb;

void aic_wifi_sw_version(char *ver);
AIC_WIFI_MODE aic_wifi_get_mode(void);
void aic_wifi_set_mode(AIC_WIFI_MODE mode);
void aic_set_adap_test(int value);
int aic_get_adap_test(void);
uint32_t aic_wifi_fmac_remote_sta_max_get(void);

int aic_wifi_start_ap(struct aic_ap_cfg *cfg);
int aic_wifi_stop_ap(void);

int aic_wifi_set_ps_lvl(int lvl);
int aic_wifi_get_temp(int32_t *temp);
int aic_wifi_init(int mode, int chip_id, void *param);
void aic_wifi_deinit(int mode);

int aic_send_txpwr_lvl_req(aic_txpwr_lvl_conf_v2_t *txpwrlvl);

int aic_ap_clear_blacklist(int fhost_vif_idx);
int aic_ap_clear_whitelist(int fhost_vif_idx);
int aic_ap_add_blacklist(int fhost_vif_idx, struct macaddr *macaddr);
int aic_ap_add_whitelist(int fhost_vif_idx, struct macaddr *macaddr);
int aic_ap_macaddr_acl(int fhost_vif_idx, uint8_t acl);
uint8_t aic_ap_get_associated_sta_cnt(void);
void *aic_ap_get_associated_sta_list(void);
int8_t aic_ap_get_associated_sta_rssi(uint8_t *addr);
int aic_channel_set(aic_config_scan_chan *config_scan_chan);
int aic_ap_wps_pbc(int fhost_vif_idx);
int aic_ap_wps_pin(int fhost_vif_idx,char *pin);
int aic_ap_wps_disable(int fhost_vif_idx);
int aic_ap_wps_cacel(int fhost_vif_idx);

int aic_wifi_fvif_mac_address_get(int fhost_vif_idx, uint8_t *mac_addr);
int aic_wifi_fvif_active_get(int fhost_vif_idx);
int aic_wifi_fvif_mode_get(int fhost_vif_idx);

#endif /* _WIFI_AL_H_ */