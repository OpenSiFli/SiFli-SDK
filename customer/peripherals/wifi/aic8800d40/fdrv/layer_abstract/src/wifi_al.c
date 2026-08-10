/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "lmac_msg.h"
#include "rwnx_msg_tx.h"
#include "rwnx_main.h"
#include "fhost_wpa.h"
#include "fhost_config.h"
#include "wlan_if.h"
#include "wifi_if.h"
#include "hal_co.h"
#include "sys_al.h"
#include "wifi_al.h"
#include "rtos_port.h"
#include "aic_plat_log.h"
#include "fhost_tx.h"

#ifdef CONFIG_SDIO_ADMA
#define AICWF_RELEASE_VERSION       "V1.3.7-0605"
#else
#define AICWF_RELEASE_VERSION       "NV1.3.1-0506"
#endif

static AIC_WIFI_MODE dev_mode = WIFI_MODE_UNKNOWN;

void aic_wifi_sw_version(char *ver)
{
	memcpy(ver,AICWF_RELEASE_VERSION,sizeof(AICWF_RELEASE_VERSION));
}

AIC_WIFI_MODE aic_wifi_get_mode(void)
{
    return dev_mode;
}

void aic_wifi_set_mode(AIC_WIFI_MODE mode)
{
    dev_mode = mode;
}

void aic_set_adap_test(int value)
{
    adap_test = value;
}

int aic_get_adap_test(void)
{
    return adap_test;
}

uint32_t aic_wifi_fmac_remote_sta_max_get(void)
{
    return rwnx_fmac_remote_sta_max_get(g_rwnx_hw);
}

int aic_wifi_start_ap(struct aic_ap_cfg *cfg)
{
	//if(g_rwnx_hw->mode != WIFI_MODE_AP)
	//	return -1;

    #ifdef CONFIG_SDIO_BUS_PWRCTRL
    aicwf_hal_bus_pwr_stctl(HOST_BUS_ACTIVE);
    #endif /* CONFIG_SDIO_BUS_PWRCTRL */

	g_rwnx_hw->net_id = wlan_start_ap(cfg);

	return g_rwnx_hw->net_id;
}

int aic_wifi_stop_ap(void)
{
	int ret = 0;

	//if(g_rwnx_hw->mode != WIFI_MODE_AP)
	//	return -1;

	ret = wlan_stop_ap();

    #ifdef CONFIG_SDIO_BUS_PWRCTRL
    aicwf_hal_bus_pwr_stctl(HOST_BUS_SLEEP);
    #endif /* CONFIG_SDIO_BUS_PWRCTRL */

	if (ret)
	{
		AIC_LOG_PRINTF("wlan_stop_ap failed have stoped! %d\n", ret);
		return -1;
	}
	else
	{
		AIC_LOG_PRINTF("wlan_stop_ap success: %d\n", ret);
	}

	rtos_res_list_info_show();

	g_rwnx_hw->net_id = 0;

	return 0;
}

int aic_wifi_set_ps_lvl(int lvl)
{
	struct mm_set_ap_ps_level_req req;
	int ret = -1;

	req.ap_ps_level = lvl;

	if (req.ap_ps_level == AP_PS_CLK_1 || req.ap_ps_level == AP_PS_CLK_2 ||
		req.ap_ps_level == AP_PS_CLK_3 || req.ap_ps_level == AP_PS_CLK_4 ||
		req.ap_ps_level == AP_PS_CLK_5 )
	{
		req.hwconfig_id = AP_PS_LEVEL_SET_REQ;
		ret = rwnx_send_set_vendor_hwconfig_req(g_rwnx_hw, AP_PS_LEVEL_SET_REQ, (int32_t *)&req, NULL);
	}

	return ret;
}

int aic_wifi_get_temp(int32_t *temp)
{
    return rwnx_send_set_vendor_hwconfig_req(g_rwnx_hw, CHIP_TEMP_GET_REQ, NULL, temp);
}

static void aic_wifi_event_register(aic_wifi_event_cb cb)
{
    g_aic_wifi_event_cb = (aic_wifi_event_cb)cb;
}

extern void aic_wifi_event_callback(AIC_WIFI_EVENT enEvent, aic_wifi_event_data *enData);
int aic_wifi_init(int mode, int chip_id, void *param)
{
    int ret = 0;
    unsigned char mac_addr[6] = {0x88, 0x00, 0x33, 0x77, 0x69, 0x22};
    unsigned int mac_local[6] = {0};

    AIC_LOG_PRINTF("aic_wifi_init, mode=%d\r\n", mode);
    AIC_LOG_PRINTF("release version:%s\r\n", AICWF_RELEASE_VERSION);

    //set_mac_address(mac_addr);
    ret = aic_wifi_open(mode, param, chip_id);
    if (ret)
	{
        AIC_LOG_PRINTF("wifi_open fail, ret=%d\n", ret);
        return -1;
    }

    aic_wifi_event_register(aic_wifi_event_callback);

    if(sp_aic_wifi_event_cb)
	{
        wifi_drv_event drv_event;
        struct wifi_dev_event dev_event;
        dev_event.drv_status = WIFI_DEVICE_DRIVER_LOADED;
        dev_event.local_mac_addr[0] = mac_addr[0];
        dev_event.local_mac_addr[1] = mac_addr[1];
        dev_event.local_mac_addr[2] = mac_addr[2];
        dev_event.local_mac_addr[3] = mac_addr[3];
        dev_event.local_mac_addr[4] = mac_addr[4];
        dev_event.local_mac_addr[5] = mac_addr[5];
        drv_event.type = WIFI_DRV_EVENT_NET_DEVICE;
        drv_event.node.dev_event = dev_event;
        sp_aic_wifi_event_cb(&drv_event);
    }

    AIC_LOG_PRINTF("aic_wifi_init ok\r\n");
    return g_rwnx_hw->net_id;
}

void aic_wifi_deinit(int mode)
{
    AIC_LOG_PRINTF("aic_wifi_deinit, mode=%d\r\n", mode);

	aic_wifi_close(mode);

	aic_wifi_event_register(NULL);

    if(sp_aic_wifi_event_cb)
	{
        wifi_drv_event drv_event;
        struct wifi_dev_event dev_event;
        dev_event.drv_status = WIFI_DEVICE_DRIVER_UNLOAD;
        drv_event.type = WIFI_DRV_EVENT_NET_DEVICE;
        drv_event.node.dev_event = dev_event;
        sp_aic_wifi_event_cb(&drv_event);
    }

    AIC_LOG_PRINTF("aic_wifi_deinit ok\r\n");
}

int aic_send_txpwr_lvl_req(aic_txpwr_lvl_conf_v2_t *txpwrlvl)
{
    return aicwf_send_txpwr_lvl_req((txpwr_lvl_conf_v2_t *)txpwrlvl);
}

int aic_ap_clear_blacklist(int fhost_vif_idx)
{
    return fhost_ap_clear_blacklist(fhost_vif_idx);
}

int aic_ap_clear_whitelist(int fhost_vif_idx)
{
    return fhost_ap_clear_whitelist(fhost_vif_idx);
}

int aic_ap_add_blacklist(int fhost_vif_idx, struct macaddr *macaddr)
{
    return fhost_ap_add_blacklist(fhost_vif_idx, (struct mac_addr *)macaddr);
}

int aic_ap_add_whitelist(int fhost_vif_idx, struct macaddr *macaddr)
{
    return fhost_ap_add_whitelist(fhost_vif_idx, (struct mac_addr *)macaddr);
}

int aic_ap_macaddr_acl(int fhost_vif_idx, uint8_t acl)
{
    return fhost_ap_macaddr_acl(fhost_vif_idx, acl);
}

uint8_t aic_ap_get_associated_sta_cnt(void)
{
    return wlan_ap_get_associated_sta_cnt();
}

void *aic_ap_get_associated_sta_list(void)
{
    return wlan_ap_get_associated_sta_list();
}

int8_t aic_ap_get_associated_sta_rssi(uint8_t *addr)
{
    return wlan_ap_get_associated_sta_rssi(addr);
}

int aic_channel_set(aic_config_scan_chan *config_scan_chan)
{
    return aicwf_channel_set((aicwf_config_scan_chan *)config_scan_chan);
}

int aic_ap_wps_pbc(int fhost_vif_idx)
{
    return fhost_ap_wps_pbc(fhost_vif_idx);
}

int aic_ap_wps_pin(int fhost_vif_idx,char *pin)
{
    return fhost_ap_wps_pin(fhost_vif_idx, pin);
}

int aic_ap_wps_disable(int fhost_vif_idx)
{
    return fhost_ap_wps_disable(fhost_vif_idx);
}

int aic_ap_wps_cacel(int fhost_vif_idx)
{
    return fhost_ap_wps_cacel(fhost_vif_idx);
}

int aic_wifi_fvif_mac_address_get(int fhost_vif_idx, uint8_t *mac_addr)
{
    int ret = 0;
    do {
        struct fhost_vif_tag *fhost_vif;
        if (fhost_vif_idx >= NX_VIRT_DEV_MAX) {
            ret = -1;
            break;
        }
        if (mac_addr == NULL) {
            ret = -2;
            break;
        }
        fhost_vif = &fhost_env.vif[fhost_vif_idx];
        memcpy(mac_addr, &fhost_vif->mac_addr, 6);
    } while (0);
    return ret;
}

int aic_wifi_fvif_active_get(int fhost_vif_idx)
{
    int ret = 0;
    struct fhost_vif_tag *fhost_vif = &fhost_env.vif[fhost_vif_idx];
    if (fhost_vif->mac_vif) {
        if (fhost_vif->mac_vif->type == VIF_STA) {
            if (fhost_vif->ap_id != INVALID_STA_IDX) {
                ret = 1;
            }
        } else if (fhost_vif->mac_vif->type == VIF_AP) {
            if (fhost_vif->conn_sock != -1) {
                ret = 1;
            }
        }
    }
    return ret;
}

int aic_wifi_fvif_mode_get(int fhost_vif_idx)
{
    uint8_t fvif_type = WIFI_MODE_UNKNOWN;
    if (fhost_vif_idx < NX_VIRT_DEV_MAX) {
        struct fhost_vif_tag *fhost_vif = &fhost_env.vif[fhost_vif_idx];
        if (fhost_vif->mac_vif) {
            if (fhost_vif->mac_vif->type == VIF_STA) {
                fvif_type = WIFI_MODE_STA;
            } else if (fhost_vif->mac_vif->type == VIF_AP) {
                fvif_type = WIFI_MODE_AP;
            }
        }
    }
    return fvif_type;
}