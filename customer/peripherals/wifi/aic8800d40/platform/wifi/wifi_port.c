/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "cli_al.h"
#include "sys_al.h"
#include "wifi_al.h"
#include "netif_al.h"
#include "wifi_def.h"
#include "sdio_port.h"
#include "wifi_port.h"
#include "dbg_assert.h"
#include "aic_plat_log.h"

#include "wlan_dev.h"
#if 0
#include "MRD.h"
#include "uapapi.h"
#include "duster.h"
#include "teldef.h"
#include "telutl.h"
#include "platform.h"
#include "wifi_api.h"
#include "rdiskfsys.h"
#include "ReliableData.h"
#include "FlashPartition.h"
#endif

#define ASR_BASE            4 // 4->V64  3->V62

#define AP_SSID_STRING_DFT  "AIC-AP-BTD"
#define AP_PASS_STRING_DFT  "11223344"

int wifi_ap_isolate = 0;
unsigned char wifi_st = 0x0;
//static uap_config_param wifi_uap_cfg;
static unsigned int  g_athandle;

//0x01: UI_Refresh_BSS_Status support
unsigned int aic_feature_config = 0;

struct aic_wifi_dev wifi_dev = {NULL,};

struct aic_ap_cfg wifi_ap_cfg =
{
	.aic_ap_ssid =
	{
		sizeof(AP_SSID_STRING_DFT)-1,
		AP_SSID_STRING_DFT
	},
	.aic_ap_passwd =
	{
		sizeof(AP_PASS_STRING_DFT)-1,
		AP_PASS_STRING_DFT
	},
	.band = 0,
	.type = 0,
	.channel = 6,
	.hidden_ssid = 0,
	.max_inactivity = 60,
	.enable_he = 1,
	.enable_acs = 0,
	.bcn_interval = 100,
	.sta_num = 10,
};

#if 0
unsigned char  UAP_STATUS(void)
{
	unsigned char ret = 0xff;
	//AIC_LOG_PRINTF("%s->%d", __func__,wifi_st);
	switch(wifi_st)
	{
		case 0:
			ret = UAP_BSS_STOP;
		break;
		case 1:
			ret = UAP_BSS_START;
		break;
		case 0xff:
			ret = UAP_BSS_RESET;
		break;
	}

    return ret;
}

extern char *IMSI_Match_CountryCode(void);
extern void UI_Refresh_BSS_Status(char start_stop);
extern unsigned char  UAP_STATUS(void);
extern uint8_t InProduction_Mode(void);
extern unsigned char RDWIFI_Fw_type(void);
extern heronWifiAtCmdSend wifi_send_at_cmd;

extern int sdio_reset(void);
extern void sio_pdn_wifi(void);
extern void sdio_reset_wifi(void);

//-------------------------------------------------------------------
// ASR Platform UAP interface
//-------------------------------------------------------------------
static INT32 UAP_Params_Check_Sanity(uap_config_param *uap_param)
{
    AIC_LOG_PRINTF("ssid_len=%d,%s\r\n", uap_param->ssid_len,uap_param->ssid);
    AIC_LOG_PRINTF("bcast_ssid_ctrl=%d\r\n", uap_param->bcast_ssid_ctrl);
    AIC_LOG_PRINTF("mode=%d\r\n", uap_param->mode);
    AIC_LOG_PRINTF("channel=%d\r\n", uap_param->channel);
    AIC_LOG_PRINTF("acs_mode=%d\r\n", uap_param->acs_mode);
    AIC_LOG_PRINTF("rf_band=%d\r\n", uap_param->rf_band);
    AIC_LOG_PRINTF("channel_bandwidth=%d\r\n", uap_param->channel_bandwidth);
    AIC_LOG_PRINTF("sec_chan=%d\r\n", uap_param->sec_chan);
    AIC_LOG_PRINTF("max_sta_count=%d\r\n", uap_param->max_sta_count);
    AIC_LOG_PRINTF("sta_ageout_timer=%ld\r\n", uap_param->sta_ageout_timer);
    AIC_LOG_PRINTF("ps_sta_ageout_timer=%ld\r\n", uap_param->ps_sta_ageout_timer);
    AIC_LOG_PRINTF("auth_mode=%d\r\n", uap_param->auth_mode);
    AIC_LOG_PRINTF("protocol=%d\r\n", uap_param->protocol);
    AIC_LOG_PRINTF("ht_cap_info=%d\r\n", uap_param->ht_cap_info);
    AIC_LOG_PRINTF("ampdu_param=%d\r\n", uap_param->ampdu_param);
    AIC_LOG_PRINTF("tx_power=%d\r\n", uap_param->tx_power);
    AIC_LOG_PRINTF("state_802_11d=%d\r\n", uap_param->state_802_11d);
    AIC_LOG_PRINTF("country_code=%s\r\n", uap_param->country_code);
    AIC_LOG_PRINTF("filter_mode=0x%x\r\n",uap_param->filter.filter_mode);

    if (uap_param->ssid_len > UAP_MAX_SSID_LENGTH) {
        AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
        uap_param->ssid_len = UAP_MAX_SSID_LENGTH;
        return UAP_SUCCESS;
    }

    if (uap_param->mode != UAP_INVALID_MODE) {//mode = UAP_INVALID_MODE is ignored
        //don't support 802.11a/802.11an/802.11ac.
        if (uap_param->mode & (UAP_BAND_A | UAP_BAND_AN | UAP_BAND_AC)) {
            AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
            uap_param->mode &= ~(UAP_BAND_A | UAP_BAND_AN | UAP_BAND_AC);
            return UAP_SUCCESS;
        }
        //802.11gn is MUST
        if ((uap_param->mode & UAP_BAND_GN) == 0) {
            AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
            uap_param->mode |= UAP_BAND_GN;
            return UAP_SUCCESS;
        }

        //no 802.11gn, no HT40
        if ((uap_param->mode & UAP_BAND_GN) == 0 && uap_param->channel_bandwidth == UAP_CHAN_WIDTH_40MHZ) {
            AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
            uap_param->channel_bandwidth = UAP_CHAN_WIDTH_20MHZ;
            return UAP_SUCCESS;
        }
    }

    if (!(uap_param->channel == 0 && uap_param->acs_mode == 0)) {//if acs_mode is ignored, rf_band is ignored
        if (uap_param->rf_band == UAP_RF_24_G || uap_param->rf_band == UAP_RF_50_G) {//any value other than UAP_RF_24_G or UAP_RF_50_G is ignored
            //Only support band 2.4GHz
            if (uap_param->rf_band != UAP_RF_24_G) {
                AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
                uap_param->rf_band = UAP_RF_24_G;
                return UAP_SUCCESS;
            }
        }
    }

    if (uap_param->protocol != 0) {//protocol = 0 is ignored
        if ((uap_param->protocol & UAP_PROTOCOL_WPA) && (uap_param->protocol & UAP_PROTOCOL_WPA3)) {
            AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
            return UAP_FAILURE_INVALID_PROTO;
        }
    }

    if (!(uap_param->channel == 0 && uap_param->acs_mode == 0)) {//if acs_mode is ignored, channel_bandwidth is ignored
        if (uap_param->channel_bandwidth == UAP_CHAN_WIDTH_20MHZ || uap_param->channel_bandwidth == UAP_CHAN_WIDTH_40MHZ) {//any value other than UAP_CHAN_WIDTH_20MHZ or UAP_CHAN_WIDTH_40MHZ is ignored
            if ((uap_param->channel_bandwidth == UAP_CHAN_WIDTH_20MHZ) && ((uap_param->sec_chan == UAP_SEC_CHAN_ABOVE) || (uap_param->sec_chan == UAP_SEC_CHAN_BELOW))) {
                AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
                uap_param->sec_chan = UAP_NO_SEC_CHAN;
                return UAP_SUCCESS;
            }

            if ((uap_param->channel_bandwidth == UAP_CHAN_WIDTH_40MHZ) && (uap_param->sec_chan != UAP_SEC_CHAN_ABOVE && uap_param->sec_chan != UAP_SEC_CHAN_BELOW)) {
                AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
                return UAP_FAILURE_INVALID_BW;
            }
        }
    }

    if (uap_param->wep_cfg.key0.length > 16) {
        AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
        uap_param->wep_cfg.key0.length = 16;
        return UAP_SUCCESS;
    }

    if (uap_param->wep_cfg.key1.length > 16) {
        AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
        uap_param->wep_cfg.key1.length = 16;
        return UAP_SUCCESS;
    }

    if (uap_param->wep_cfg.key2.length > 16) {
        AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
        uap_param->wep_cfg.key2.length = 16;
        return UAP_SUCCESS;
    }

    if (uap_param->wep_cfg.key3.length > 16) {
        AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
        uap_param->wep_cfg.key3.length = 16;
        return UAP_SUCCESS;
    }

    return UAP_SUCCESS;
}

static INT32 UAP_Params_Freq2chan(int32_t freq, int8_t *chan)
{
    //XXX
    if (freq >= 2412 && freq <= 2472) {
        if ((freq - 2407) % 5) {
            AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
            return UAP_FAILURE_INVALID_FREQ;
        }

        *chan = (freq - 2407) / 5;

        return UAP_SUCCESS;
    }

    if (freq == 2484) {
        *chan = 14;

        return UAP_SUCCESS;
    }

    AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
    return UAP_FAILURE_INVALID_FREQ;
}

static INT32 UAP_Params_Chan2freq(uint8_t chan, uint32_t *freq) {
    //XXX
    if (chan < 1 || chan > 14) {
        AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
        return UAP_FAILURE_INVALID_CHAN;
    }

    if (chan == 14) {
        *freq = 2484;
    } else {
        *freq = (2407 + chan*5);
    }
    return UAP_SUCCESS;
}

static void UAP_Params_init(uap_config_param *uap_param)
{
    uap_param->ssid_len = 0;
    uap_param->ssid[0] = '\0';
    uap_param->bcast_ssid_ctrl = UAP_INVALID_BYTE;
    uap_param->mode = UAP_INVALID_MODE;
    uap_param->channel = 0;
    uap_param->acs_mode = 0;
    uap_param->rf_band = UAP_INVALID_BYTE;
    uap_param->channel_bandwidth = UAP_INVALID_BYTE;
    uap_param->sec_chan = UAP_INVALID_BYTE;
    uap_param->max_sta_count = UAP_INVALID_BYTE;/* Follow Wi-Fi Draft-v1 0 */
    uap_param->sta_ageout_timer = 1;
    uap_param->ps_sta_ageout_timer = 1;
    uap_param->auth_mode = UAP_INVALID_HWORD;
    uap_param->protocol = 0;
    uap_param->ht_cap_info = 0;
    uap_param->ampdu_param = UAP_INVALID_BYTE;
    uap_param->tx_power = UAP_INVALID_BYTE;
    uap_param->enable_2040coex = UAP_INVALID_BYTE;
    uap_param->filter.filter_mode = UAP_INVALID_HWORD;
    uap_param->wpa_cfg.length = 0;
    uap_param->wpa_cfg.passphrase[0] = '\0';
    uap_param->wpa_cfg.gk_rekey_time = UAP_INVALID_WORD;
    uap_param->wpa_cfg.pairwise_cipher_wpa = 0;
    uap_param->wpa_cfg.pairwise_cipher_wpa2 = 0;
    uap_param->wpa_cfg.group_cipher = 0;
    uap_param->wep_cfg.key0.key_index = 0;
    uap_param->wep_cfg.key0.length = 0;
    uap_param->wep_cfg.key0.key[0] = '\0';
    uap_param->wep_cfg.key0.is_default = 0;
    uap_param->wep_cfg.key1.key_index = 1;
    uap_param->wep_cfg.key1.length = 0;
    uap_param->wep_cfg.key1.key[0] = '\0';
    uap_param->wep_cfg.key1.is_default = 0;
    uap_param->wep_cfg.key2.key_index = 2;
    uap_param->wep_cfg.key2.length = 0;
    uap_param->wep_cfg.key2.key[0] = '\0';
    uap_param->wep_cfg.key2.is_default = 0;
    uap_param->wep_cfg.key3.key_index = 3;
    uap_param->wep_cfg.key3.length = 0;
    uap_param->wep_cfg.key3.key[0] = '\0';
    uap_param->wep_cfg.key3.is_default = 0;
    uap_param->state_802_11d = UAP_INVALID_BYTE;
    uap_param->country_code[0] = '\0';
    uap_param->power_mode.flags = UAP_INVALID_HWORD;
}

static void UAP_Params_clear_wep_keys(void)
{

}

static void UAP_Params_clear_wpa_key(void)
{

}

static INT32 UAP_Params_Set_Ssid(uap_config_param *uap_param)
{
	//ssid_len = 0 is ignored
    if (uap_param->ssid_len != 0) {
		wifi_ap_cfg.aic_ap_ssid.length = uap_param->ssid_len;
		memcpy(wifi_ap_cfg.aic_ap_ssid.array,uap_param->ssid,uap_param->ssid_len);
	}

    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Bcast_Ssid_Ctrl(uap_config_param *uap_param)
{
    if (uap_param->bcast_ssid_ctrl == 0 || uap_param->bcast_ssid_ctrl == 1)
	{//any value other than 0 or 1 is ignored
		wifi_ap_cfg.hidden_ssid = !uap_param->bcast_ssid_ctrl;
    }

	AIC_LOG_PRINTF("%s: %d: %s(): %d\r\n", __FILE__, __LINE__, __func__,wifi_ap_cfg.hidden_ssid);
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Mode(uap_config_param *uap_param) {
	wifi_ap_cfg.enable_he = 0;

	if (uap_param->mode != UAP_INVALID_MODE) {//mode = UAP_INVALID_MODE is ignored
        if (uap_param->mode & UAP_BAND_GN) {

        }

        if (uap_param->mode & UAP_BAND_AX) {
			wifi_ap_cfg.enable_he = 1;
		}
		//he default
		//wifi_ap_cfg.enable_he = 1;
    }
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Channel(uap_config_param *uap_param) {
    if (uap_param->channel != 0)
	{//channel = 0 is ignored
		wifi_ap_cfg.channel = uap_param->channel;
    }
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Acs_Mode(uap_config_param *uap_param)
{
    if (uap_param->acs_mode == 1)
	{//automatic selection
		wifi_ap_cfg.enable_acs = 1;
    }
	else if (uap_param->acs_mode == 0 && uap_param->channel != 0) //manual selection
	{
		wifi_ap_cfg.enable_acs = 0;
    }

    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Rf_Band(uap_config_param *uap_param)
{
    if (!(uap_param->channel == 0 && uap_param->acs_mode == 0)) {//if acs_mode is ignored, rf_band is ignored
        //do nothing
    }
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Channel_Bandwidth(uap_config_param *uap_param)
{
    if (!(uap_param->channel == 0 && uap_param->acs_mode == 0)) {//if acs_mode is ignored, channel_bandwidth is ignored
        if (uap_param->channel_bandwidth == UAP_CHAN_WIDTH_20MHZ || uap_param->channel_bandwidth == UAP_CHAN_WIDTH_40MHZ) {//any value other than UAP_CHAN_WIDTH_20MHZ or UAP_CHAN_WIDTH_40MHZ is ignored
            switch (uap_param->channel_bandwidth) {
                case UAP_CHAN_WIDTH_20MHZ:
                {
					wifi_ap_cfg.type = 0;
                    break;
                }
                case UAP_CHAN_WIDTH_40MHZ:
                {
                    if (uap_param->sec_chan == UAP_SEC_CHAN_ABOVE)
					{

					}
					else if (uap_param->sec_chan == UAP_SEC_CHAN_BELOW)
					{

                    }
					else
					{
                        AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
                        return UAP_FAILURE_INVALID_BW;
                    }

					wifi_ap_cfg.type = 1;
                    break;
                }
            }
        }
    }
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Max_Sta_Count(uap_config_param *uap_param)
{
    if (uap_param->max_sta_count != UAP_INVALID_BYTE) {//max_sta_count = 0xFF is ignored
		wifi_ap_cfg.sta_num = uap_param->max_sta_count;
    }
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Sta_Ageout_Timer(uap_config_param *uap_param)
{
    if (uap_param->sta_ageout_timer != 1) {//sta_ageout_timer = 1 is ignored

    }
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Ps_Sta_Ageout_Timer(uap_config_param *uap_param)
{
    //XXX: not support
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Auth_Mode(uap_config_param *uap_param)
{
    if (uap_param->protocol == UAP_PROTOCOL_STATIC_WEP)
	{//protocol != UAP_PROTOCOL_STATIC_WEP is ignored
        if (uap_param->auth_mode != UAP_INVALID_HWORD) {//0xFFFF is ignored
            switch (uap_param->auth_mode) {
                case UAP_AUTH_MODE_OPEN:
                {

                    break;
                }
                case UAP_AUTH_MODE_SHARED:
                {

                    break;
                }
                default:
                {
                    AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
                    return UAP_FAILURE_INVALID_AUTH_MODE;
                }
            }
        }
    }
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Protocol(uap_config_param *uap_param)
{
    if (uap_param->protocol != 0)
	{//protocol = 0 is ignored

        switch (uap_param->protocol)
		{
            case UAP_PROTOCOL_NO_SECURITY:
            {
				wifi_ap_cfg.sercurity_type = KEY_NONE;
                break;
            }
            case UAP_PROTOCOL_STATIC_WEP:
            {
				wifi_ap_cfg.sercurity_type = KEY_WEP;

                break;
            }
            case UAP_PROTOCOL_WPA:
            {
            	//may hv some bug instead WPA2
				wifi_ap_cfg.sercurity_type = KEY_WPA;
                break;
            }
            case UAP_PROTOCOL_WPA2:
            {
				wifi_ap_cfg.sercurity_type = KEY_WPA2;
                break;
            }
            case UAP_PROTOCOL_WPA2_WPA_MIXED:
            {
				wifi_ap_cfg.sercurity_type = KEY_WPA_WPA2;
                break;
            }
            case UAP_PROTOCOL_WPA3:
            {
            	if(uap_param->mode & UAP_BAND_AX)
            		wifi_ap_cfg.sercurity_type = KEY_WPA3;
				else
					wifi_ap_cfg.sercurity_type = KEY_WPA2;
                break;
            }
            case UAP_PROTOCOL_WPA3_WPA2_MIXED:
            {
				wifi_ap_cfg.sercurity_type = KEY_WPA2_WPA3;
                break;
            }
            default:
            {
                AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
                return UAP_FAILURE_INVALID_PROTO;
            }
        }

        if (uap_param->protocol == UAP_PROTOCOL_NO_SECURITY)
		{
            UAP_Params_clear_wep_keys();
            UAP_Params_clear_wpa_key();
        }
		else if (uap_param->protocol == UAP_PROTOCOL_STATIC_WEP)
		{
            UAP_Params_clear_wpa_key();
        }
		else
		{
            UAP_Params_clear_wep_keys();
            if (uap_param->wpa_cfg.length != 0)
			{//This is ignored, if protocol = 0

			}
            if (uap_param->protocol & UAP_PROTOCOL_WPA) {

                if (uap_param->wpa_cfg.pairwise_cipher_wpa & UAP_CIPHER_TKIP)
				{

                }
                if (uap_param->wpa_cfg.pairwise_cipher_wpa & UAP_CIPHER_AES_CCMP)
				{

                }
            }
            if ((uap_param->protocol & UAP_PROTOCOL_WPA2) || (uap_param->protocol & UAP_PROTOCOL_WPA3)) {

                if (uap_param->wpa_cfg.pairwise_cipher_wpa2 & UAP_CIPHER_TKIP)
				{

                }
                if (uap_param->wpa_cfg.pairwise_cipher_wpa2 & UAP_CIPHER_AES_CCMP)
				{

                }
            }
            if (uap_param->wpa_cfg.group_cipher != 0) {//0 is ignored

                if (uap_param->wpa_cfg.group_cipher & UAP_CIPHER_TKIP)
				{

                }
                if (uap_param->wpa_cfg.group_cipher & UAP_CIPHER_AES_CCMP)
				{

                }

				wifi_ap_cfg.aic_ap_passwd.length = uap_param->wpa_cfg.length;
				memcpy(wifi_ap_cfg.aic_ap_passwd.array,uap_param->wpa_cfg.passphrase,wifi_ap_cfg.aic_ap_passwd.length);
            }


        }

        if (uap_param->wpa_cfg.gk_rekey_time < 86400)
		{//gk_rekey_time >= 86400 is ignored

        }
    }
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Ht_Cap_Info(uap_config_param *uap_param) {
    //XXX: not support
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Ampdu_Param(uap_config_param *uap_param) {
    //XXX: not support
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Tx_Power(uap_config_param *uap_param) {
    //XXX: not support
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Enable_2040coex(uap_config_param *uap_param)
{
    if (uap_param->enable_2040coex == 0 || uap_param->enable_2040coex == 1)
	{//any value other than 0 or 1 is ignored

	}
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Filter(uap_config_param *uap_param)
{
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Wep_Cfg(uap_config_param *uap_param)
{
    if (uap_param->protocol == UAP_PROTOCOL_STATIC_WEP)
	{//protocol != UAP_PROTOCOL_STATIC_WEP is ignored
        if (uap_param->wep_cfg.key0.length != 0)
        {

            if (uap_param->wep_cfg.key0.is_default != 0)
			{
				if((uap_param->wep_cfg.key0.length != 5)||(uap_param->wep_cfg.key0.length != 13))
				{
					wifi_ap_cfg.aic_ap_passwd.length = uap_param->wep_cfg.key0.length;
					memcpy(wifi_ap_cfg.aic_ap_passwd.array,uap_param->wep_cfg.key0.key,wifi_ap_cfg.aic_ap_passwd.length);
				}
            }
        }
        if (uap_param->wep_cfg.key1.length != 0)
		{//wep key length = 0 is ignored
            if (uap_param->wep_cfg.key1.is_default != 0)
			{
				if((uap_param->wep_cfg.key1.length != 5)||(uap_param->wep_cfg.key1.length != 13))
				{
					wifi_ap_cfg.aic_ap_passwd.length = uap_param->wep_cfg.key1.length;
					memcpy(wifi_ap_cfg.aic_ap_passwd.array,uap_param->wep_cfg.key1.key,wifi_ap_cfg.aic_ap_passwd.length);
				}
			}
        }
        if (uap_param->wep_cfg.key2.length != 0)
		{//wep key length = 0 is ignored

            if (uap_param->wep_cfg.key2.is_default != 0)
			{
				if((uap_param->wep_cfg.key2.length != 5)||(uap_param->wep_cfg.key2.length != 13))
				{
					wifi_ap_cfg.aic_ap_passwd.length = uap_param->wep_cfg.key2.length;
					memcpy(wifi_ap_cfg.aic_ap_passwd.array,uap_param->wep_cfg.key2.key,wifi_ap_cfg.aic_ap_passwd.length);
				}
            }
        }
        if (uap_param->wep_cfg.key3.length != 0)
		{//wep key length = 0 is ignored
            if (uap_param->wep_cfg.key3.is_default != 0)
			{
				if((uap_param->wep_cfg.key3.length != 5)||(uap_param->wep_cfg.key3.length != 13))
				{
					wifi_ap_cfg.aic_ap_passwd.length = uap_param->wep_cfg.key3.length;
					memcpy(wifi_ap_cfg.aic_ap_passwd.array,uap_param->wep_cfg.key3.key,wifi_ap_cfg.aic_ap_passwd.length);
				}
            }
        }
    }
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_State_802_11d(uap_config_param *uap_param)
{
    if (uap_param->state_802_11d == 0 || uap_param->state_802_11d == 1)
	{//any value other than 0 or 1 is ignored

    }
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Country_Code(uap_config_param *uap_param)
{
    if (uap_param->country_code[0] != '\0')
	{//country_code with first NULL character is ignored

    }
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Set_Power_Mode(uap_config_param *uap_param)
{
    //XXX: not support
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Ssid(uap_config_param *uap_param)
{
    uap_param->ssid_len = wifi_ap_cfg.aic_ap_ssid.length;
	memcpy(uap_param->ssid,wifi_ap_cfg.aic_ap_ssid.array,uap_param->ssid_len);

    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Bcast_Ssid_Ctrl(uap_config_param *uap_param)
{
    uap_param->bcast_ssid_ctrl = wifi_ap_cfg.hidden_ssid;

    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Mode(uap_config_param *uap_param)
{
    //802.11b and 802.11g are mandotary.
    uap_param->mode = (UAP_BAND_B | UAP_BAND_G);

	uap_param->mode |= wifi_uap_cfg.mode;

    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Channel(uap_config_param *uap_param)
{
    uint8_t chan;

	uap_param->channel = wifi_uap_cfg.channel;

    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Acs_Mode(uap_config_param *uap_param)
{
    uap_param->acs_mode = wifi_ap_cfg.enable_acs;

    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Rf_Band(uap_config_param *uap_param)
{
    uap_param->rf_band = UAP_RF_24_G;

	return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Channel_Bandwidth(uap_config_param *uap_param)
{
	uap_param->channel_bandwidth = wifi_uap_cfg.channel_bandwidth;
	uap_param->sec_chan = wifi_uap_cfg.sec_chan;
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Max_Sta_Count(uap_config_param *uap_param)
{
    uap_param->max_sta_count = wifi_ap_cfg.sta_num;

    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Sta_Ageout_Timer(uap_config_param *uap_param)
{
    uap_param->sta_ageout_timer = wifi_ap_cfg.max_inactivity;

    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Ps_Sta_Ageout_Timer(uap_config_param *uap_param)
{
    //XXX: not support
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Auth_Mode(uap_config_param *uap_param)
{
	uap_param->auth_mode = wifi_uap_cfg.auth_mode;
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Protocol(uap_config_param *uap_param)
{
    uap_param->protocol = wifi_uap_cfg.protocol;

    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Ht_Cap_Info(uap_config_param *uap_param)
{
    //XXX: not support
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Ampdu_Param(uap_config_param *uap_param)
{
    //XXX: not support
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Tx_Power(uap_config_param *uap_param)
{
    //XXX: not support
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Enable_2040coex(uap_config_param *uap_param)
{
    uap_param->enable_2040coex = wifi_uap_cfg.enable_2040coex;
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Wpa_Cfg(uap_config_param *uap_param)
{
	memcpy(&uap_param->wpa_cfg,&wifi_uap_cfg.wpa_cfg,sizeof(wifi_uap_cfg.wpa_cfg));
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Wep_Cfg(uap_config_param *uap_param)
{
	memcpy(&uap_param->wep_cfg,&wifi_uap_cfg.wep_cfg,sizeof(wifi_uap_cfg.wep_cfg));
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_State_802_11d(uap_config_param *uap_param)
{
	uap_param->state_802_11d = wifi_uap_cfg.state_802_11d;
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Country_Code(uap_config_param *uap_param)
{
    memcpy(&uap_param->country_code,&wifi_uap_cfg.country_code,2);
    uap_param->country_code[2] = '\0';
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Power_Mode(uap_config_param *uap_paramy)
{
    //XXX: not support
    return UAP_SUCCESS;
}

static INT32 UAP_Params_Get_Filter(uap_mac_filter *uap_filter)
{
    uint8_t i = 0;

    uap_filter->filter_mode = wifi_uap_cfg.filter.filter_mode;
    uap_filter->mac_count = wifi_uap_cfg.filter.mac_count;

	for (i = 0; i < uap_filter->mac_count && uap_filter->mac_list[i]; i++)
	{
        memcpy(&uap_filter->mac_list[i], wifi_uap_cfg.filter.mac_list[i], 6);
    }

    return UAP_SUCCESS;
}

static void _uap_mac_filter(void)
{
	unsigned char i;

	//if(wifi_uap_cfg.filter.filter_mode != UAP_MAC_FILTER_MODE_DISABLED)
	{
        aic_ap_clear_blacklist(0);
		aic_ap_clear_whitelist(0);
	}
	//black list
	if(wifi_uap_cfg.filter.filter_mode == UAP_MAC_FILTER_MODE_BLACK_LIST)
	{
		for(i=0;i<wifi_uap_cfg.filter.mac_count;i++)
		{
			aic_ap_add_blacklist(0,(struct macaddr *)wifi_uap_cfg.filter.mac_list[i]);
		}
	}
	//white list
	else if(wifi_uap_cfg.filter.filter_mode == UAP_MAC_FILTER_MODE_WHITE_LIST)
	{
		for(i=0;i<wifi_uap_cfg.filter.mac_count;i++)
		{
			aic_ap_add_whitelist(0,(struct macaddr *)wifi_uap_cfg.filter.mac_list[i]);
		}
		aic_ap_macaddr_acl(0, 1);
	}
	else
	{
		aic_ap_macaddr_acl(0, 0);
	}
}

INT32 UAP_Params_Config(uap_config_param *uap_param)
{
    INT32 ret;


    if (uap_param->action == UAP_ACTION_GET)
	{

        UAP_Params_init(uap_param);

        if ((ret = UAP_Params_Get_Ssid(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Bcast_Ssid_Ctrl(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Mode(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Channel(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Acs_Mode(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Rf_Band(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Channel_Bandwidth(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Max_Sta_Count(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Sta_Ageout_Timer(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Auth_Mode(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Protocol(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Ht_Cap_Info(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Ampdu_Param(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Tx_Power(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Enable_2040coex(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Filter(&uap_param->filter)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Wpa_Cfg(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Wep_Cfg(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_State_802_11d(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Country_Code(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Get_Power_Mode(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

    }
	else if (uap_param->action == UAP_ACTION_SET)
	{
		memcpy(&wifi_uap_cfg,uap_param,sizeof(uap_config_param));

        if ((ret = UAP_Params_Check_Sanity(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Ssid(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Bcast_Ssid_Ctrl(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Mode(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Channel(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Acs_Mode(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Rf_Band(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Channel_Bandwidth(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Max_Sta_Count(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Sta_Ageout_Timer(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Ps_Sta_Ageout_Timer(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Auth_Mode(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Protocol(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Ht_Cap_Info(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Ampdu_Param(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Tx_Power(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Enable_2040coex(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Filter(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Wep_Cfg(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_State_802_11d(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Country_Code(uap_param)) != UAP_SUCCESS) {
            return ret;
        }

        if ((ret = UAP_Params_Set_Power_Mode(uap_param)) != UAP_SUCCESS) {
            return ret;
        }


    }

    return UAP_SUCCESS;
}

INT32 UAP_BSS_Config(UINT32 start_stop)
{

	int ret;

	if(wifi_st == 0xff)
		return -1;

	if(wifi_ap_cfg.sercurity_type == KEY_NONE)
	{
		wifi_ap_cfg.aic_ap_passwd.length = 0;
	}
	else if(wifi_ap_cfg.sercurity_type == KEY_WEP)
	{
		UAP_Params_Set_Wep_Cfg(&wifi_uap_cfg);
	}
	else
	{
		wifi_ap_cfg.aic_ap_passwd.length = wifi_uap_cfg.wpa_cfg.length;
		memcpy(wifi_ap_cfg.aic_ap_passwd.array,wifi_uap_cfg.wpa_cfg.passphrase,wifi_ap_cfg.aic_ap_passwd.length);
	}

	//config by web ui
    //wifi_ap_cfg.type = PHY_CHNL_BW_40;

	wifi_st = 0xff;

	if(start_stop)
	{
		ret = aic_wifi_start_ap(&wifi_ap_cfg);
		if(ret >= 0)
		{
			_uap_mac_filter();
			wifi_st = 1;
		}
		else
		{
			wifi_st = 0;
		}
	}
	else
	{
		ret = aic_wifi_stop_ap();
		wifi_st = 0;
	}

	if(aic_feature_config&0x01)
		UI_Refresh_BSS_Status(wifi_st);
	AIC_LOG_PRINTF("%s:%d", __func__,ret);
    return ret;
}

INT32 UAP_STA_List(sta_list *list)
{
	AIC_LOG_PRINTF("%s st %d ", __func__,wifi_st);

	if(wifi_st != 1)
		return -1;

	uint8_t index, cnt = aic_ap_get_associated_sta_cnt();
	void *sta_list = aic_ap_get_associated_sta_list();
	struct co_list_hdr *list_hdr = co_list_pick(sta_list);
	index = 0;

	list->sta_count = cnt;

	while (list_hdr != NULL)
	{
		struct aic_sta_info *sta = (struct aic_sta_info *)list_hdr;
		list_hdr = co_list_next(list_hdr);
		list->info[index].rssi = aic_ap_get_associated_sta_rssi((uint8_t*)(sta->mac_addr.array));
		list->info[index].power_mfg_status = 0;
		memcpy(list->info[index].mac_address,sta->mac_addr.array,6);
		AIC_LOG_PRINTF("STA[%d] = %x:%x:%x rssi:%d\n", index, sta->mac_addr.array[0], sta->mac_addr.array[1], sta->mac_addr.array[2],list->info[index].rssi);

		index ++;
	}

    return UAP_SUCCESS;
}


INT32 UAP_Sta_Deauth(uap_802_11_mac_addr *mac)
{

    if (!mac)
	{
        AIC_LOG_PRINTF("%s: %d: %s():\r\n", __FILE__, __LINE__, __func__);
        return UAP_FAILURE_INVALID_PARAM;
    }

    return UAP_SUCCESS;
}

INT32 UAP_PktFwd_Ctl(uap_pkt_fwd_ctl *param)
{

    if (param->action == UAP_ACTION_GET)
	{

        param->pkt_fwd_ctl = 0;
        if (wifi_ap_isolate & UAP_PKT_FWD_INTRA_BSS_UCAST) {
            param->pkt_fwd_ctl |= UAP_PKT_FWD_INTRA_BSS_UCAST;
        }
        if (wifi_ap_isolate & UAP_PKT_FWD_INTRA_BSS_BCAST) {
            param->pkt_fwd_ctl |= UAP_PKT_FWD_INTRA_BSS_BCAST;
        }
    }
	else if (param->action == UAP_ACTION_SET)
	{
		wifi_ap_isolate = 0;

        if (param->pkt_fwd_ctl & UAP_PKT_FWD_INTRA_BSS_UCAST)
		{
            wifi_ap_isolate |= UAP_PKT_FWD_INTRA_BSS_UCAST;
        }
        if (param->pkt_fwd_ctl & UAP_PKT_FWD_INTRA_BSS_BCAST)
		{
            wifi_ap_isolate |= UAP_PKT_FWD_INTRA_BSS_BCAST;
        }

    }
	AIC_LOG_PRINTF("%s:0x%x\r\n",__func__,wifi_ap_isolate);
    return UAP_SUCCESS;
}

/*
 * int aicwf_channel_set(aicwf_config_scan_chan *config_scan_chan);
 * @param brief:
 *   config_scan_chan->band: 0 -> 2.4G
 *                           1 -> 5G
 */
INT32 UAP_Scan_Channels_Config(uap_config_scan_chan *param)
{
    AIC_LOG_PRINTF("%s\n", __func__);

	//UAP_RF_24_G  asr define 1  aic define 0
	if(param->band)
		param->band -= 1;

    aic_channel_set((aic_config_scan_chan *)param);
    return UAP_SUCCESS;
}

INT32 UAP_Sta_Conn_Notify_Reg(void (*notify_cb)(uint8_t, uint8_t*))
{

    return UAP_SUCCESS;
}

INT32 UAP_GetCurrentChannel(void)
{
    uint8_t channel = 0;

    return (INT32)channel;
}

void UAP_GetChannelByCountry(int *first, int *last)
{
	char *ccode;
	int found = 1;

	*first = 1;

	ccode = IMSI_Match_CountryCode();
	if(ccode == NULL)
	{
		found = 0;
		goto cfound;
	}

	AIC_LOG_PRINTF("UAP_GetChannelByCountry: country code is %s", ccode);

	/* Search specific domain name */
	if ((ccode[0] == 'J') && (ccode[1] == 'P'))
	{
		/* japan support 1 ~ 14 channel */
		*last  = 14;
	}
	else if (((ccode[0] == 'U') && (ccode[1] == 'S')) ||	 /* United States */
			 ((ccode[0] == 'A') && (ccode[1] == 'S')) ||	 /* American Samoa */
			 ((ccode[0] == 'C') && (ccode[1] == 'A')) ||	 /* Canada */
			 ((ccode[0] == 'F') && (ccode[1] == 'M')) ||	 /* Federated States of Micronesia */
			 ((ccode[0] == 'G') && (ccode[1] == 'U')) ||	 /* Guam */
			 ((ccode[0] == 'K') && (ccode[1] == 'Y')) ||	 /* Cayman Islands */
			 ((ccode[0] == 'P') && (ccode[1] == 'R')) ||	 /* Puerto Rico */
			 ((ccode[0] == 'T') && (ccode[1] == 'W')) ||	 /* Taiwan */
			 ((ccode[0] == 'U') && (ccode[1] == 'M')) ||	 /* United States(Minor Outlying Islands) */
			 ((ccode[0] == 'V') && (ccode[1] == 'I')))		 /* Vingin Islands(United States) */
	{
		/* support 1 ~ 11 channel */
		*last  = 11;
	}
	else if (((ccode[0] == 'I') && (ccode[1] == 'R')) ||	 /* Iran */
			 ((ccode[0] == 'S') && (ccode[1] == 'Y')))		 /* Syria */
	{
		found = 0;
	}
	else
	{
		/* support 1 ~ 13 channel */
		*last  = 13;
	}

cfound:
	if(found == 0)
	{
		*last = 11;
	}

	AIC_LOG_PRINTF("UAP_GetChannelByCountry: first channel is %d, last channel is %d", *first, *last);

	return;
}

void UAP_IntraBSSUnicastDeny(int enable)
{
    uap_pkt_fwd_ctl fwctl;
    fwctl.action = 0;
    UAP_PktFwd_Ctl(&fwctl);
    fwctl.action = 1;

	fwctl.pkt_fwd_ctl = 1;

    if(enable)
    {
        fwctl.pkt_fwd_ctl |= 0x4;
    }
    else
    {
        fwctl.pkt_fwd_ctl &= (~0x4);
    }

    UAP_PktFwd_Ctl(&fwctl);
}

int UAP_Params_Set_Filter_Only(void *filter)
{
	unsigned char i;

	uap_mac_filter *uap_filter = (uap_mac_filter *)filter;


	if(wifi_st != 1)
	{
		AIC_LOG_PRINTF("UAP_Params_Set_Filter_Only:can'sset[st:%d]", wifi_st);
		return -1;
	}

	AIC_LOG_PRINTF("UAP_Params_Set_Filter_Only:set:%d\n",uap_filter->filter_mode);

	memcpy(&(wifi_uap_cfg.filter),uap_filter,sizeof(uap_mac_filter));

	//if(wifi_uap_cfg.filter.filter_mode != UAP_MAC_FILTER_MODE_DISABLED)
	{
		aic_ap_clear_blacklist(0);
		aic_ap_clear_whitelist(0);
	}
	//black list
	if(wifi_uap_cfg.filter.filter_mode == UAP_MAC_FILTER_MODE_BLACK_LIST)
	{
		for(i=0;i<wifi_uap_cfg.filter.mac_count;i++)
		{
			aic_ap_add_blacklist(0,(struct macaddr *)wifi_uap_cfg.filter.mac_list[i]);
		}
	}
	//white list
	else if(wifi_uap_cfg.filter.filter_mode == UAP_MAC_FILTER_MODE_WHITE_LIST)
	{
		for(i=0;i<wifi_uap_cfg.filter.mac_count;i++)
		{
			aic_ap_add_whitelist(0,(struct macaddr *)wifi_uap_cfg.filter.mac_list[i]);
		}
		aic_ap_macaddr_acl(0, 1);
	}
	else
	{
		aic_ap_macaddr_acl(0, 0);
	}

	return 0;
}

void UAP_configure_wifimac(uint8_t *mac)
{
	AIC_LOG_PRINTF("%s  %x:%x:%x:%x:%x:%x",__func__,mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

	if (!(mac[0] == 0 && mac[1] == 0 && mac[2] == 0 &&
        mac[3] == 0 && mac[4] == 0 && mac[5] == 0))
		aic_set_mac_address(mac);
}

uint8_t GetWifiStatus(void)
{
	if(wifi_dev.wifi_config_st == 1)
	{
		if(UAP_STATUS() == UAP_BSS_START)
			return 1;
		else
			return 0;
	}
	else
	{
		return 0xff;
	}
}

int UAP_GetStaNum(void)
{
	if(GetWifiStatus() != 1)
		return -1;

	uint8_t num = aic_ap_get_associated_sta_cnt();

	AIC_LOG_PRINTF("%s %d", __func__,num);

	return num;
}

void KickClients(char *MAC_addr)
{
	AIC_LOG_PRINTF("%s", __func__);

    return;
}
int UAP_Notify(unsigned char state)
{
	AIC_LOG_PRINTF("%s %d", __func__,state);

    return 0;
}
void Wlan_Status_Check(int status)
{
	AIC_LOG_PRINTF("%s", __func__);

    return;
}

int uap_packet_free(char *buf)
{
	AIC_LOG_PRINTF("%s", __func__);

	return 0;
}

// kick sta?
INT32 UAP_Sta_Block_Unblock(void *mac, UINT32 isBlock)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 1;
}

//wifi sleep timer set
uint8_t UAP_PowerSaveMode(uint8_t on, UINT32 min_sleep, UINT32 max_sleep, UINT32 inactivity_time, UINT32 min_awake, UINT32 max_awake)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

void get_powersave(UINT16* ps_mode, UINT32* min_sleep, UINT32* max_sleep,
                    UINT32* inactivity_time, UINT32* min_awake, UINT32* max_awake)
{
	AIC_LOG_PRINTF("%s", __func__);

    return;
}

void wifi_txtrigger_firmwaredump(void)
{
	AIC_LOG_PRINTF("%s", __func__);

	return;
}

//usb mode set cal value
void WIFI_SetCal(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return;
}

//gen random mac addr
void rand_gen(char *MAC)
{
    return;
}

BOOL IsMlanTask(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int UAP_ConfigBand(int argc, uint8_t *band)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}
/* AT+WIFIBANDWIDTH= */
int UAP_ConfigBandWidth(int argc, int *bandwidth)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

/*AT+WIFISTA?*/
int UAP_GetStaInfo(char *str)
{
	if(UAP_STATUS() == 0)
		return -1;

	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

//AT+WIFIVERSION=
int UAP_VER(char *ver)
{
	AIC_LOG_PRINTF("%s", __func__);

	aic_wifi_sw_version(ver);

    return 0;
}

int Uap_Get_Beacon_Period(void)
{
	AIC_LOG_PRINTF("%s %d", __func__,wifi_ap_cfg.bcn_interval);

    return wifi_ap_cfg.bcn_interval;
}

int Uap_Set_Dtim_period(int period)
{
	AIC_LOG_PRINTF("%s %d", __func__,period);

	wifi_ap_cfg.dtim = period;

    return 0;
}

/* AT+WIFIMFGTEST= */
int Uap_Set_GI(uint8_t enable)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}
/* AT+WIFIMFGTEST= */
int Uap_Get_Preamble(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}
/* AT+WIFIMFGTEST= 23*/
void Uap_set_ed_mac(int enable, int offset)
{
	AIC_LOG_PRINTF("%s", __func__);

    return;
}

/* AT+WIFIMFGTEST= 18*/
int Uap_Get_Dtim_period(void)
{
	AIC_LOG_PRINTF("%s,%d", __func__,wifi_ap_cfg.dtim);

	return wifi_ap_cfg.dtim;
}

int Uap_Get_GI(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int Uap_Set_Beacon_Period(int period)
{
	AIC_LOG_PRINTF("%s %d", __func__,period);

	wifi_ap_cfg.bcn_interval = period;

    return 0;
}

int Uap_Set_Preamble(int type)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int apcmd_bss_stop(void)
{
	AIC_LOG_PRINTF("%s", __func__);

	UAP_BSS_Config(0);

	return 0;
}

/* AT+WIFIDBG= */
int apcmd_regrdwr_process(int reg, char *offset, char *strvalue, unsigned int *retvalue)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

void wifi_trigger_recv(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return;
}

void sys_cfg_tx_power(UINT16 action, uint8_t *txpower)
{
	AIC_LOG_PRINTF("%s", __func__);

    return;
}

/*
AT+WIFICHANNEL=
*/
int apcmd_sys_cfg_channel(uint8_t *chl,int argc,...)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

/* AT+WIFITIMCFG */
INT32 apcmd_sys_cfg_sticky_tim_config(int argc, char *argv[])
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

//config wifi mac AT+RDMIFIMAC
int apcmd_sys_cfg_ap_mac_address(uint8_t *mac, UINT16 action)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

/* AT+WIFITIMMAC= */
INT32 apcmd_sys_cfg_sticky_tim_sta_mac_addr(int argc, char *argv[])
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}
/* AT+WIFITXDATARATE= */
int apcmd_sys_cfg_tx_data_rate(int argc, int *rate)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

/************************************************************
wps feature  for hal/wps
*************************************************************/

int woal_uap_do_ioctl(UINT32 cmd, uint8_t *cmd_buf)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

// wifi device not surpport wap? depend on wifi type?
int WIFI_Sendto_Wapi(uint8_t *data, UINT32 len)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}
// wifi device not surpport wap? depend on wifi type?
signed long WIFI_Sendto_Wps(uint8_t bss_type, uint8_t *data, UINT32 len)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

// wifi device not surpport wap? depend on wifi type?
signed long WIFI_Recvfrom_Wps(uint8_t bss_type, uint8_t *data, UINT32 len)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int GetWifiRecoveryFlag(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}
signed long wapi_send_to_wifi(uint8_t *data, UINT32 len)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

/************************************************
wifi test mode  command can use aic's
**************************************************/
//AT+LOG wifi en
void WiFiDrvDbg(UINT32 value)
{
	AIC_LOG_PRINTF("%s %d", __func__,value);

    return;
}

//AT+WIFIMFGTEST=3,0  get current channel band
int Cf_GetRfChannel(int *channel, double *pFreqInMHz, DWORD DeviceId)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

//AT+WIFIMFGTEST=2,2 get current band
int Dev_GetBandAG(int *band, DWORD DeviceId)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

//AT+WIFIMFGTEST=1,6 get current bw
int Dev_GetChannelBw(int *ChannelBw, DWORD DeviceId)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

void MRVL_DUT_Menu(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return;
}

//AT+WIFIMFGTEST=4  dut mod
int MRVL_RFT_OpenDUT(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

/* AT+WIFITEST= 0,1,0,0*/
int MRVL_RFT_CloseDUT(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

/* AT+WIFITEST= 0,0,1*/
int MRVL_RFT_TxDataRate(DWORD TxDataRate)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}
/* AT+WIFIMFGTEST= */
int MRVL_RFT_SetPreamble(int PreambleType)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_Channel(int ChannelNo)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_TxBurstInterval(DWORD BurstSifsInUs)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_TxPayloadLength(DWORD TxPayLength)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

// tx Packet mode
int MRVL_RFT_SetTxInitMode(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

// Continuous Tx with Modulation
int MRVL_RFT_SetTxContMode(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

// Continuous Tx without Modulation
int MRVL_RFT_SetTxContMode_CW_Mode(int cw_enable)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

// Carrier Suppression Tx Mode
int MRVL_RFT_etTxCarrierSuppression(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_TxStart(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_TxStop(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_RxStart(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_RxStop(DWORD DeviceId )
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_FRError(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_FRGood(void)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_TxGain(int Pwr4Pa)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_SetShortGI(int mGuardInterval)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_SetBandAG(int mBand)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_SetChannelBw(int mBandWidth)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}

int MRVL_RFT_SetSSID(char *m_ssid)
{
	AIC_LOG_PRINTF("%s", __func__);

    return 0;
}



//-------------------------------------------------------------------
// ASR Platform WPS interface
//-------------------------------------------------------------------
typedef enum
{
	WPS_REG_INVALIDE = 0x0,
	WPS_REG_WAIT_FOR_NEW_SESSION,
	WPS_REG_ONGOING,
	WPS_REG_SUCC,
	WPS_REG_FAIL
} WPS_RET_STATE;

enum wps_event {
	WPS_EV_M2D,
	WPS_EV_FAIL,
	WPS_EV_SUCCESS,
	WPS_EV_PWD_AUTH_FAIL,
	WPS_EV_PBC_OVERLAP,
	WPS_EV_PBC_TIMEOUT,
	WPS_EV_PBC_ACTIVE,
	WPS_EV_PBC_DISABLE,
	WPS_EV_ER_AP_ADD,
	WPS_EV_ER_AP_REMOVE,
	WPS_EV_ER_ENROLLEE_ADD,
	WPS_EV_ER_ENROLLEE_REMOVE,
	WPS_EV_ER_AP_SETTINGS,
	WPS_EV_ER_SET_SELECTED_REGISTRAR,
	WPS_EV_AP_PIN_SUCCESS,
	WPS_EV_AP_PIN_TIMEOUT,
	WPS_EV_SELECTED_TIMEOUT
};

typedef void (*aic_wps_status_notify)(int event);
extern void aic_set_wps_status_cb(aic_wps_status_notify cb);

struct plat_wps_dev
{
    int           wpsmethod;  // 0 is pin, 1 is pbc
    WPS_RET_STATE wps_ret;    // WPS_REG_INVALIDE
};

struct plat_wps_dev wps_dev = {NULL,};

void SetWPSMethod(int method)
{
	wps_dev.wpsmethod = method;
}

int GetWPSMethod()
{
	return wps_dev.wpsmethod;
}

/*
*@brief:WSPNOTIFYFUNC is a callback function.
*@param:state indicate the state of wps session.
  state = 0: wps session is inactive
  state = 1: wps session is ongoing
  state = 2: wps session is success
  state = 3: wps session is fail
  state = 4: wps session is cancelled
*@note:Don't process too much in this function,don't suspend in this function
*/
typedef void (*WSPNOTIFYFUNC)(unsigned char state);

#define     WPSNOTIFYLISTNUM    5
static WSPNOTIFYFUNC   WpsNotifyList[5];
static int WpsNotifyCnt = 0;

/*
*@brief:Add callback functions to wps application.
*@param:NotifyFunc is the callback function.
*@return:-1 indicate add callback function error;
         0 indicate add callback function sucess.
*/
int WPSApp_NotifyReg(WSPNOTIFYFUNC NotifyFunc)
{
    if(WpsNotifyCnt < WPSNOTIFYLISTNUM)
    {
        WpsNotifyList[WpsNotifyCnt++] = NotifyFunc;
        return 0;
    }

    AIC_LOG_PRINTF("Wps notify list is full");

	return -1;
}

int WPSApp_Notify(unsigned char state)
{
    int i = 0;
    AIC_LOG_PRINTF("Wps notify %d", state);

	for(i=0; i<WpsNotifyCnt; i++)
    {
        (WpsNotifyList[i])(state);
    }

	return 0;
}

int WpsApp_GetRegStatus(void)
{
	return wps_dev.wps_ret;
}

/*
state = 0: wps session is inactive
state = 1: wps session is ongoing
state = 2: wps session is success
state = 3: wps session is fail
state = 4: wps session is cancelled
*/
static void wps_status_callback(int event)
{
    struct plat_wps_dev *wpsdev = &wps_dev;
    switch(event)
    {
        //PBC
        case WPS_EV_PBC_TIMEOUT:
            wpsdev->wps_ret = WPS_REG_FAIL;
            WPSApp_Notify(3);
            break;
        case WPS_EV_PBC_ACTIVE:
            wpsdev->wps_ret = WPS_REG_ONGOING;
            WPSApp_Notify(1);
            break;
        case WPS_EV_PBC_DISABLE:
            /*
             * WPS_EV_SUCCESS event will be triggered once pbc is
             * successfully matched, so no need to notify APP that
             * match is success here.
             */
            //wps_ret = WPS_REG_SUCC;
            //WPSApp_Notify(2);
            break;
		//PIN
        case WPS_EV_PWD_AUTH_FAIL:
        case WPS_EV_AP_PIN_TIMEOUT:
            wpsdev->wps_ret = WPS_REG_FAIL;
            WPSApp_Notify(3);
            break;
        case WPS_EV_AP_PIN_SUCCESS:
            wpsdev->wps_ret = WPS_REG_SUCC;
            WPSApp_Notify(2);
            break;
        case WPS_EV_SUCCESS:
            wpsdev->wps_ret = WPS_REG_SUCC;
            WPSApp_Notify(2);
            break;
        case WPS_EV_SELECTED_TIMEOUT:
            wpsdev->wps_ret = WPS_REG_FAIL;
            WPSApp_Notify(3);
            break;
        default:
            break;
    }
}
void WpsMsg_UsePBC(void)
{
	int ret = -1;
    struct plat_wps_dev *wpsdev = &wps_dev;

	wpsdev->wps_ret = WPS_REG_INVALIDE;

	if(UAP_STATUS() != 0)
		return;

	wpsdev->wps_ret = WPS_REG_ONGOING;
	AIC_LOG_PRINTF("[AIC] WPS Use PBC...\r\n");
	SetWPSMethod(1);

	aic_set_wps_status_cb(wps_status_callback);
	//wpa_cli wps_pbc
	aic_ap_wps_pbc(0);
	return;
}

void WPSAPP_Launch(void)
{
    struct plat_wps_dev *wpsdev = &wps_dev;

	wpsdev->wps_ret = WPS_REG_INVALIDE;

	if(UAP_STATUS() != 0)
		return;

	AIC_LOG_PRINTF("[AIC] WPS Launch...\r\n");
	wpsdev->wps_ret = WPS_REG_SUCC;
	return;
}

void WPSApp_Disable(void)
{
	int ret = -1;
    struct plat_wps_dev *wpsdev = &wps_dev;

	wpsdev->wps_ret = WPS_REG_INVALIDE;

	if(UAP_STATUS() != 0)
		return;
	AIC_LOG_PRINTF("[AIC] WPS Disable...\r\n");

	//wpa_cli wps_ap_pin disable
	ret = aic_ap_wps_disable(0);;

	if(ret == 0)
		wpsdev->wps_ret = WPS_REG_SUCC;
	else
		wpsdev->wps_ret = WPS_REG_FAIL;

	ret = aic_ap_wps_cacel(0);;
	if(ret == 0)
		wpsdev->wps_ret = WPS_REG_SUCC;
	else
	{
		wpsdev->wps_ret = WPS_REG_FAIL;
	}

	return;
}

void WPSApp_Enable(void)
{
    struct plat_wps_dev *wpsdev = &wps_dev;

	AIC_LOG_PRINTF("[AIC] WPS Enable...\r\n");
	wpsdev->wps_ret = WPS_REG_INVALIDE;

	if(UAP_STATUS() != 0)
		return;

	wpsdev->wps_ret = WPS_REG_SUCC;
	return;
}

void WpsMsg_UseCancel(void)
{
	int ret = -1;
    struct plat_wps_dev *wpsdev = &wps_dev;

	wpsdev->wps_ret = WPS_REG_ONGOING;


	AIC_LOG_PRINTF("[AIC] WPS Cancel...\r\n");

	wpsdev->wps_ret = WPS_REG_INVALIDE;

	if(UAP_STATUS() != 0)
		return;
	//wpa_cli wps_cancel
	ret = aic_ap_wps_cacel(0);;

	if(ret == 0)
		wpsdev->wps_ret = WPS_REG_SUCC;
	else
		wpsdev->wps_ret = WPS_REG_FAIL;
	return;
}

void WpsMsg_UsePIN(char *pin)
{
	int ret = -1;
    struct plat_wps_dev *wpsdev = &wps_dev;

	AIC_LOG_PRINTF("[AIC] WPS Use PIN %s ...\r\n", pin);
	wpsdev->wps_ret = WPS_REG_INVALIDE;

	if(UAP_STATUS() != 0)
		return;

	if(pin == NULL)
		return;

	SetWPSMethod(0);
	aic_set_wps_status_cb(wps_status_callback);
	//wpa_cli wps_pin any xxxxxxxx
	ret = aic_ap_wps_pin(0,pin);

	if(ret != 0)
	{
		wpsdev->wps_ret = WPS_REG_FAIL;
		return;
	}
	return;
}

int ValidateChecksum(unsigned long int PIN)
{
    unsigned int accum = 0;


    accum += 3 * ((PIN / 10000000) % 10);
    accum += 1 * ((PIN / 1000000) % 10);
    accum += 3 * ((PIN / 100000) % 10);
    accum += 1 * ((PIN / 10000) % 10);
    accum += 3 * ((PIN / 1000) % 10);
    accum += 1 * ((PIN / 100) % 10);
    accum += 3 * ((PIN / 10) % 10);
    accum += 1 * ((PIN / 1) % 10);

    return (0 == (accum % 10));
}
#endif

#if 0
int strncasecmp(const char *s1, const char *s2, size_t n)
{
	int c1, c2;
	while (n > 0) {

		/* Use "unsigned char" to make the implementation 8-bit clean */
		c1 = *((unsigned char *)(s1++));
		if (c1 >= 'A' && c1 <= 'Z')
			c1 = c1 + ('a' - 'A');

		c2 = *((unsigned char *)(s2++));
		if (c2 >= 'A' && c2 <= 'Z')
			c2 = c2 + ('a' - 'A');

		if (c1 != c2) {
			return (c1 - c2);
		}

		if (c1 == '\0') {
			return 0;
		}

		--n;
	}
	return 0;
}
#endif


//-------------------------------------------------------------------
// wifi_init_task
//-------------------------------------------------------------------
static void CliCmdCb(char *rsp)
{
	//ATRESP( g_athandle,1,0,rsp);
}

#if 0
int GetWiFiType(void)
{
	return ASR_BASE;
}

INT32 WifiSwitch(uint8_t value)
{
	AIC_LOG_PRINTF("%s %d\n", __func__,value);

	return UAP_BSS_Config(value);
}

int uap_readmrd_mac(void)
{
#define MAC_FILE_FULL_NAME "www\\temp\\mac"
#define MAC_FILE_NAME "MAC.bin"
	MRDErrorCodes	ret_val = MRD_NO_ERROR;
	UINT32	version = 0;
	FILE_ID MacFile = NULL;
	UINT32	date = 0;
	char mac[6];

	ret_val = MRDInitNucleus();
	AIC_LOG_PRINTF("%s:%d", __func__,ret_val);

	if(ret_val == MRD_NO_ERROR)
	{
		if (MRDFileRead(MAC_FILE_NAME, MRD_MAC_TYPE, 0, &version, &date, MAC_FILE_FULL_NAME) == MRD_NO_ERROR)
		{
			MacFile = Rdisk_fopen(MAC_FILE_FULL_NAME);
			AIC_LOG_PRINTF("%s:%p", __func__,MacFile);

			if (MacFile!=NULL)
			{
				Rdisk_fread (mac, 6, 1,MacFile);
				Rdisk_fclose (MacFile);
				aic_set_mac_address((uint8_t*)mac);
				return 0;
			}
		}
	}

	return -1;
}
#endif

int get_wifi_mac(uint8_t *wifi_mac)
{
	uint8_t * mac;

	mac = aic_get_mac_address();

	memcpy(wifi_mac,mac,6);

	AIC_LOG_PRINTF("%s :%x %x %x\n", __func__,wifi_mac[3],wifi_mac[4],wifi_mac[5]);

    return 0;
}

int GetMACAddr(uint8_t *macbuf)
{
	AIC_LOG_PRINTF("%s", __func__);

	get_wifi_mac(macbuf);

    return 0;
}

uint8_t GetWiFiInitStat(void)
{
	AIC_LOG_PRINTF("%s->>%d", __func__, wifi_dev.wifi_init_status);

    return wifi_dev.wifi_init_status;
}

void SetWifiConfigFlag(uint8_t value)
{
	AIC_LOG_PRINTF("%s %d", __func__,value);

	wifi_dev.wifi_config_st = value;

    return;
}

unsigned char GetWifiConfigFlag(void)
{
    return wifi_dev.wifi_config_st;
}

static void wifi_set_mode(AIC_WIFI_MODE mode)
{
	unsigned char fwtype = RDWIFI_Fw_type();
	unsigned char need_startap = 0;
    struct aic_wifi_dev *wifidev = &wifi_dev;

	if(fwtype&0x02)
		wifidev->wifi_mode = WIFI_MODE_RFTEST;
	else if(InProduction_Mode()&&(fwtype&0x04))
		wifidev->wifi_mode = WIFI_MODE_RFTEST;
	else if((mode == WIFI_MODE_RFTEST)&&(fwtype&0x04))
		wifidev->wifi_mode = WIFI_MODE_RFTEST;
	else
		wifidev->wifi_mode = WIFI_MODE_AP;
}

void WiFiClientConnect(unsigned char *MAC_addr, unsigned char mode)
{
    #if 0
    OS_STATUS  		os_status;
    WlanClientStatusMessage  status_msg;
    char *MAC_buf = NULL;

    MAC_buf = malloc(6); //this will be free in duster
    if(MAC_buf == NULL)
    	return;

    memcpy(MAC_buf, MAC_addr, 6);

    if(mode == 1)
        status_msg.MsgId = client_connected;
    else
        status_msg.MsgId = client_disconnected;

    status_msg.MsgData = MAC_buf;

    if(mode == 1){
        AIC_LOG_PRINTF("add client: %02X:%02X:%02X:%02X:%02X:%02X",
                        status_msg.MsgData[0],status_msg.MsgData[1],
                        status_msg.MsgData[2],status_msg.MsgData[3],
                        status_msg.MsgData[4],status_msg.MsgData[5]);
        AIC_LOG_PRINTF("Moal Add gWlanIndMSGQ=%x",gWlanIndMSGQ);
    }
    else
    {
        AIC_LOG_PRINTF("delete client: %02X:%02X:%02X:%02X:%02X:%02X",
                        status_msg.MsgData[0],status_msg.MsgData[1],
                        status_msg.MsgData[2],status_msg.MsgData[3],
                        status_msg.MsgData[4],status_msg.MsgData[5]);
        AIC_LOG_PRINTF("Moal Dis gWlanIndMSGQ=%x",gWlanIndMSGQ);
    }
    os_status = OSAMsgQSend(gWlanIndMSGQ, sizeof(status_msg), (unsigned char *)&status_msg, OSA_NO_SUSPEND);
	AIC_LOG_PRINTF("os_status=%d",os_status);
    #endif
}

wifi_priv_event_cb_asr asr_aic_wifi_event_cb = NULL;
void aic_wifi_asr_event_register(wifi_priv_event_cb_asr cb)
{
	asr_aic_wifi_event_cb = cb;
};

int g_vendor_specific_apple = 0;
wifi_drv_event_cbk sp_aic_wifi_event_cb = NULL;
int wifi_drv_event_set_cbk(wifi_drv_event_cbk cbk)
{
    sp_aic_wifi_event_cb = cbk;
    AIC_LOG_PRINTF("%s is called, sp_aic_wifi_event_cb: %p\r\n", __func__, sp_aic_wifi_event_cb);

    return 1;
}

void wifi_drv_event_reset_cbk(void) {
    sp_aic_wifi_event_cb = NULL;

    aic_wifi_deinit(WIFI_MODE_AP);
}

static wifi_event_handle m_scan_result_handle = NULL;
static wifi_event_handle m_scan_done_handle = NULL;
static wifi_event_handle m_join_success_handle = NULL;
static wifi_event_handle m_join_fail_handle = NULL;
static wifi_event_handle m_leave_handle = NULL;
static unsigned int aic_p2p_dev_port = 7236; //default port number

extern wifi_priv_event_cb_asr asr_aic_wifi_event_cb;


static void aic_wifi_ap_associated(char* mac)
{
    struct rt_wlan_buff buff;
    struct rt_wlan_info sta_info;

    memset(&sta_info, 0, sizeof(sta_info));
    memcpy(sta_info.bssid, mac, 6);

    buff.len = sizeof(sta_info);
    buff.data = (void *)&sta_info;

    aic_dbg("%s: %02X:%02X:%02X:%02X:%02X:%02X\n",
        __func__,  sta_info.bssid[0], sta_info.bssid[1], sta_info.bssid[2], sta_info.bssid[3], sta_info.bssid[4], sta_info.bssid[5]);

    rt_wlan_dev_indicate_event_handle(s_ap_dev, RT_WLAN_DEV_EVT_AP_ASSOCIATED, &buff);
}

static void aic_wifi_ap_disassociated(char* mac)
{
    struct rt_wlan_buff buff;
    struct rt_wlan_info sta_info;

    memset(&sta_info, 0, sizeof(sta_info));
    memcpy(sta_info.bssid, mac, 6);

    buff.len = sizeof(sta_info);
    buff.data = (void *)&sta_info;

    aic_dbg("%s: %02X:%02X:%02X:%02X:%02X:%02X\n",
        __func__,  sta_info.bssid[0], sta_info.bssid[1], sta_info.bssid[2], sta_info.bssid[3], sta_info.bssid[4], sta_info.bssid[5]);

    rt_wlan_dev_indicate_event_handle(s_ap_dev, RT_WLAN_DEV_EVT_AP_DISASSOCIATED, &buff);
}

void aic_wifi_event_callback(AIC_WIFI_EVENT enEvent, aic_wifi_event_data *enData)
{
    switch(enEvent)
    {
        case SCAN_RESULT_EVENT:
            {
                if (m_scan_result_handle)
                {
                    m_scan_result_handle(enData);
                }
                break;
            }
        case SCAN_DONE_EVENT:
            {
                if (m_scan_done_handle)
                {
                    m_scan_done_handle(enData);
                }
                break;
            }
        case JOIN_SUCCESS_EVENT:
            {
                if (m_join_success_handle)
                {
                    m_join_success_handle(enData);
                }
                break;
            }
        case JOIN_FAIL_EVENT:
            {
                if (m_join_fail_handle)
                {
                    m_join_fail_handle(enData);
                }
                break;
            }
        case LEAVE_RESULT_EVENT:
            {
                if (m_leave_handle)
                {
                    m_leave_handle(enData);
                }
                break;
            }
        case PRO_DISC_REQ_EVENT:
            {
                uint32_t *mac_addr = enData->data.auth_deauth_data.reserved;
                aic_dbg("PRO_DISC_REQ_EVENT mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n"
                       , mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
                if (sp_aic_wifi_event_cb) {
                    wifi_drv_event drv_event;
                    struct wifi_p2p_event p2p_event;
                    p2p_event.event_type = WIFI_P2P_EVENT_GOT_PRO_DISC_REQ_AFTER_GONEGO_OK;
                    p2p_event.peer_dev_mac_addr[0] = (unsigned char)enData->data.auth_deauth_data.reserved[0];
                    p2p_event.peer_dev_mac_addr[1] = (unsigned char)enData->data.auth_deauth_data.reserved[1];
                    p2p_event.peer_dev_mac_addr[2] = (unsigned char)enData->data.auth_deauth_data.reserved[2];
                    p2p_event.peer_dev_mac_addr[3] = (unsigned char)enData->data.auth_deauth_data.reserved[3];
                    p2p_event.peer_dev_mac_addr[4] = (unsigned char)enData->data.auth_deauth_data.reserved[4];
                    p2p_event.peer_dev_mac_addr[5] = (unsigned char)enData->data.auth_deauth_data.reserved[5];
                    drv_event.type = WIFI_DRV_EVENT_P2P;
                    drv_event.node.p2p_event = p2p_event;

					#ifdef CONFIG_P2P
                    aic_p2p_dev_port = enData->p2p_dev_port_num;
					#endif
                    sp_aic_wifi_event_cb(&drv_event);
                }
                break;
            }
        case EAPOL_STA_FIN_EVENT:
            {
                uint32_t *mac_addr = enData->data.auth_deauth_data.reserved;
                aic_dbg("EAPOL_STA_FIN_EVENT mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n"
                       , mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
                if (sp_aic_wifi_event_cb) {
                    wifi_drv_event drv_event;
                    struct wifi_ap_event ap_event;
                    ap_event.event_type = WIFI_AP_EVENT_ON_ASSOC;
                    drv_event.type = WIFI_DRV_EVENT_AP;
                    ap_event.peer_dev_mac_addr[0] = (unsigned char)enData->data.auth_deauth_data.reserved[0];
                    ap_event.peer_dev_mac_addr[1] = (unsigned char)enData->data.auth_deauth_data.reserved[1];
                    ap_event.peer_dev_mac_addr[2] = (unsigned char)enData->data.auth_deauth_data.reserved[2];
                    ap_event.peer_dev_mac_addr[3] = (unsigned char)enData->data.auth_deauth_data.reserved[3];
                    ap_event.peer_dev_mac_addr[4] = (unsigned char)enData->data.auth_deauth_data.reserved[4];
                    ap_event.peer_dev_mac_addr[5] = (unsigned char)enData->data.auth_deauth_data.reserved[5];
                    drv_event.node.ap_event = ap_event;

                    //diag_dump_buf(ap_event.peer_dev_mac_addr, 6);
                    sp_aic_wifi_event_cb(&drv_event);
                }
                break;
            }
        case EAPOL_P2P_FIN_EVENT:
            {
                uint32_t *mac_addr = enData->data.auth_deauth_data.reserved;
                aic_dbg("EAPOL_P2P_FIN_EVENT mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n"
                       , mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
                if (sp_aic_wifi_event_cb) {
                    wifi_drv_event drv_event;
                    struct wifi_p2p_event p2p_event;
                    p2p_event.event_type = WIFI_P2P_EVENT_ON_ASSOC_REQ;
                    p2p_event.peer_dev_port = aic_p2p_dev_port;
                    p2p_event.peer_dev_mac_addr[0] = (unsigned char)enData->data.auth_deauth_data.reserved[0];
                    p2p_event.peer_dev_mac_addr[1] = (unsigned char)enData->data.auth_deauth_data.reserved[1];
                    p2p_event.peer_dev_mac_addr[2] = (unsigned char)enData->data.auth_deauth_data.reserved[2];
                    p2p_event.peer_dev_mac_addr[3] = (unsigned char)enData->data.auth_deauth_data.reserved[3];
                    p2p_event.peer_dev_mac_addr[4] = (unsigned char)enData->data.auth_deauth_data.reserved[4];
                    p2p_event.peer_dev_mac_addr[5] = (unsigned char)enData->data.auth_deauth_data.reserved[5];
                    drv_event.type = WIFI_DRV_EVENT_P2P;
                    drv_event.node.p2p_event = p2p_event;

                    sp_aic_wifi_event_cb(&drv_event);
                }
                break;
            }
        case ASSOC_IND_EVENT:
            {
                uint32_t *mac_addr = enData->data.auth_deauth_data.reserved;
                aic_dbg("ASSOC_IND_EVENT mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n"
                       , mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
				{
					uint8_t mac[6];
                    mac[0] = enData->data.auth_deauth_data.reserved[0];
                    mac[1] = enData->data.auth_deauth_data.reserved[1];
                    mac[2] = enData->data.auth_deauth_data.reserved[2];
                    mac[3] = enData->data.auth_deauth_data.reserved[3];
                    mac[4] = enData->data.auth_deauth_data.reserved[4];
                    mac[5] = enData->data.auth_deauth_data.reserved[5];
                    aic_wifi_ap_associated(mac);
				}
                break;
            }
        case STA_DISCONNECT_EVENT:
            {
                AIC_WIFI_MODE mode = aic_wifi_get_mode();
                aic_dbg("STA_DISCONNECT_EVENT, current mode:%d\r\n", mode);
                if (sp_aic_wifi_event_cb &&  mode == WIFI_MODE_STA) {
                    wifi_drv_event drv_event;
                    struct wifi_sta_event dev_event;
                    dev_event.event_type = WIFI_STA_EVENT_ON_DISASSOC;
                    drv_event.type = WIFI_DRV_EVENT_STA;
                    drv_event.node.sta_event = dev_event;
                    sp_aic_wifi_event_cb(&drv_event);
                }
                break;
            }
        case DISASSOC_STA_IND_EVENT:
            {
                uint32_t *mac_addr = enData->data.auth_deauth_data.reserved;
                aic_dbg("DISASSOC_STA_IND_EVENT mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n"
                       , mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
                if(sp_aic_wifi_event_cb) {
                    wifi_drv_event drv_event;
                    struct wifi_ap_event ap_event;
                    ap_event.event_type = WIFI_AP_EVENT_ON_DISASSOC;
                    ap_event.peer_dev_mac_addr[0] = (unsigned char)enData->data.auth_deauth_data.reserved[0];
                    ap_event.peer_dev_mac_addr[1] = (unsigned char)enData->data.auth_deauth_data.reserved[1];
                    ap_event.peer_dev_mac_addr[2] = (unsigned char)enData->data.auth_deauth_data.reserved[2];
                    ap_event.peer_dev_mac_addr[3] = (unsigned char)enData->data.auth_deauth_data.reserved[3];
                    ap_event.peer_dev_mac_addr[4] = (unsigned char)enData->data.auth_deauth_data.reserved[4];
                    ap_event.peer_dev_mac_addr[5] = (unsigned char)enData->data.auth_deauth_data.reserved[5];
                    drv_event.type = WIFI_DRV_EVENT_AP;
                    drv_event.node.ap_event = ap_event;

                    sp_aic_wifi_event_cb(&drv_event);
                }

				{
					uint8_t mac[6];

                    mac[0] = enData->data.auth_deauth_data.reserved[0];
                    mac[1] = enData->data.auth_deauth_data.reserved[1];
                    mac[2] = enData->data.auth_deauth_data.reserved[2];
                    mac[3] = enData->data.auth_deauth_data.reserved[3];
                    mac[4] = enData->data.auth_deauth_data.reserved[4];
                    mac[5] = enData->data.auth_deauth_data.reserved[5];
                    aic_wifi_ap_disassociated(mac);
				}
                break;
            }
        case DISASSOC_P2P_IND_EVENT:
            {
                uint32_t *mac_addr = enData->data.auth_deauth_data.reserved;
                aic_dbg("DISASSOC_P2P_IND_EVENT mac_addr = %02x:%02x:%02x:%02x:%02x:%02x\r\n"
                       , mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
                if(sp_aic_wifi_event_cb) {
                    wifi_drv_event drv_event;
                    struct wifi_p2p_event p2p_event;
                    p2p_event.event_type = WIFI_P2P_EVENT_ON_DISASSOC;
                    p2p_event.peer_dev_mac_addr[0] = (unsigned char)enData->data.auth_deauth_data.reserved[0];
                    p2p_event.peer_dev_mac_addr[1] = (unsigned char)enData->data.auth_deauth_data.reserved[1];
                    p2p_event.peer_dev_mac_addr[2] = (unsigned char)enData->data.auth_deauth_data.reserved[2];
                    p2p_event.peer_dev_mac_addr[3] = (unsigned char)enData->data.auth_deauth_data.reserved[3];
                    p2p_event.peer_dev_mac_addr[4] = (unsigned char)enData->data.auth_deauth_data.reserved[4];
                    p2p_event.peer_dev_mac_addr[5] = (unsigned char)enData->data.auth_deauth_data.reserved[5];
                    drv_event.type = WIFI_DRV_EVENT_P2P;
                    drv_event.node.p2p_event = p2p_event;

                    sp_aic_wifi_event_cb(&drv_event);
                }
                break;
            }
        default:
            {
                break;
            }
    }
}

void aic_wifi_cmd(char *atcmd, unsigned int athandle, unsigned char chan_num);
static void wifi_init_task(void *argv)
{
	char mac[6];
	unsigned char fwtype = 0;//RDWIFI_Fw_type();
	unsigned char need_startap = 0;
    struct aic_wifi_dev *wifidev = &wifi_dev;

	if(fwtype&0x02)
		wifidev->wifi_mode = WIFI_MODE_RFTEST;
	//else if(InProduction_Mode()&&(fwtype&0x04))
	//	wifidev->wifi_mode = WIFI_MODE_RFTEST;
	else
		wifidev->wifi_mode = WIFI_MODE_AP;

	//register at cmd cb
	//wifi_send_at_cmd = aic_wifi_cmd;
    aic_cli_set_cb(CliCmdCb);
	aic_wifi_asr_event_register(WiFiClientConnect);

	while(1)
	{
		rtos_semaphore_wait(wifidev->wifi_task_sem, AIC_RTOS_WAIT_FOREVEVR);
		AIC_LOG_PRINTF("op:%d\n", wifidev->wifi_drive_op);

		switch(wifidev->wifi_drive_op)
		{
			// open driver,reset if needed
			case 0x06:
			{
				//sdio_reset_wifi();
				//if(sdio_reset())
				//	break;
				need_startap = 1;
			}
			case 0x01:
				aic_wifi_init(wifidev->wifi_mode, 0, NULL);
				if((need_startap)&&(wifidev->g_wifi_init))
				{
					wifidev->wifi_init_status = wifidev->g_wifi_init;
					wifi_st = 0;
					//if(wifidev->wifi_mode == WIFI_MODE_AP)
					//	UAP_BSS_Config(1);
					need_startap = 0;
				}
				else if(!wifidev->g_wifi_init)
				{
					aic_driver_unexcepted(0x08);
				}
			break;

			// close driver and  pdn
			case 0x05:// ldo dis
				//sio_pdn_wifi();
			case 0x02:
				aic_wifi_deinit(wifidev->wifi_mode);
				need_startap = 0;
			break;

			//reset
			//adap_test
			case 0x04:
                aic_set_adap_test(1);
			case 0x03:
				//ldo dn
				//sio_pdn_wifi();
				//deubut
				aic_wifi_deinit(wifidev->wifi_mode);
				//reset wifi
				//sdio_reset_wifi();
				//reset sdio
				//if(!sdio_reset())
				{
					//ok
					if(aic_wifi_init(wifidev->wifi_mode, 0, NULL) >= 0)
					{
						wifidev->wifi_init_status = wifidev->g_wifi_init;
						wifi_st = 0;
						//if(wifidev->wifi_mode == WIFI_MODE_AP)
						//	UAP_BSS_Config(1);
					}
					else if(!wifidev->g_wifi_init)
					{
						aic_driver_unexcepted(0x08);
					}
				}
			break;

			default:break;
		}

		wifidev->wifi_drive_op = 0;
        aic_set_adap_test(0);
		wifidev->wifi_init_status = wifidev->g_wifi_init;

		if(!wifidev->g_wifi_init)
			wifi_st = 0xff;
	}

}

void wifi_driver_action(unsigned int  act)
{
    struct aic_wifi_dev *wifidev = &wifi_dev;
	if(rtos_semaphore_get_count(wifidev->wifi_task_sem) != 0)
	{
		AIC_LOG_PRINTF("r u creazy?");
		return;
	}

	wifidev->wifi_drive_op = act;

	rtos_semaphore_signal(wifidev->wifi_task_sem, 0);
}

void wifi_driver_init(void)
{
	int ret = 0;
    struct aic_wifi_dev *wifidev = &wifi_dev;

	AIC_LOG_PRINTF("%s\n", __func__);
	//first read mac from mrd file,if efuse exist mac will coverit

	ret = rtos_semaphore_create_only(&wifidev->wifi_task_sem, "wifi_init_sem", 1, 0);

	if(ret)
	{
		AIC_LOG_PRINTF("wifi task sem create fail\n");
	}

    ret = rtos_task_create_only(wifi_init_task, "wifi_init_task", 0,
                           0x2000, NULL, 40,
                           &wifidev->wifi_task_init_hdl);

    if (ret || (wifidev->wifi_task_init_hdl == NULL))
	{
        AIC_LOG_PRINTF("wifi task init create fail\n");
    }

	wifi_driver_action(0x01);

    return;
}

void aic_driver_reboot(unsigned int mode)
{
	wifi_driver_action(mode);
}

//driver restart interface
//0x03  close and open
//0x04: adap_test mode.
//0x05: power off and close
//0x06: power on and open

//unexcepted action interfae
void aic_driver_unexcepted(unsigned int evt)
{
	switch(evt)
	{
		//sdio rx buf alloc fail keep 1.5s
		case 0x01:
			//AIC_ASSERT_ERR(0);
			aic_driver_reboot(0x03);
		break;
		//tx buf flow ctrl keep 2.5s
		case 0x02:
			//AIC_ASSERT_ERR(0);
			aic_driver_reboot(0x03);
		break;
		//sdio err may reset system
		case 0x04:
			AIC_ASSERT_ERR(0);
		break;
		//driver init fail
		case 0x08:
			AIC_ASSERT_ERR(0);
		break;
		//start ap fail
		case 0x10:
			//ASSERT(0);
		break;

		default:
		break;
	}
}

//-------------------------------------------------------------------
// aic wifi privite cmd  AT+wifi=....
//-------------------------------------------------------------------
static int cmd_parse_argv_x(char *cmd, char *argv[])
{
	int last, argc;
	char *start, *end;

	argc = 0;
	start = cmd;

	while (start != '\0') {
		while (*start == ' ' || *start == '\t')
			start++;
		if (*start == '\0')
			break;
		end = start;
		while (*end != ' ' && *end != '\t' && *end != '\0')
			end++;
		last = *end == '\0';
		*end = '\0';
		argv[argc++] = start;
		if (last)
			break;
		start = end + 1;
	}

	return argc;
}

#if 0
static void aic_wifi_cmd(char *atcmd, unsigned int athandle, unsigned char chan_num)
{
	int argc, handle, ret=0,i;
	char *argv[20];
	char rev_c[64];
	char rsp[128]= {0};
    struct aic_wifi_dev *wifidev = &wifi_dev;

	AIC_LOG_PRINTF("%s %s\n", __func__,atcmd);
	g_athandle = athandle;
	if(os_strlen(atcmd) > 64)
	{
		AIC_LOG_PRINTF("%s too long\n", __func__);
		goto end;
	}

	memset(rev_c,0,64);
	memcpy(rev_c,atcmd,os_strlen(atcmd));

	argc = cmd_parse_argv_x(atcmd, &argv[0]);

	//iperf test
	if(os_strcmp(argv[0],"iperf") == 0)
	{
		handle = aic_iperf_parse_argv(argc-1, &argv[1]);

		if (handle == -1)
		{
			goto end;
		}

		if (handle >= 0 && handle < aic_get_iperf_handle_max())
		{
			ret = aic_iperf_handle_start(/*g_wlan_netif*/0, handle);
			if (ret == -1)
				aic_iperf_handle_free(handle);
		}
	}
	//rf test
	else if(os_strcmp(argv[0],"aicrftest") == 0)
	{
		if(wifidev->wifi_mode == WIFI_MODE_RFTEST)
		{
			if(argc>1)
			{
				ret = aic_cli_run_cmd(rev_c+os_strlen("aicrftest")+1);
			}
		}
		else
		{
			ATRESP( athandle,1,0,"rf bin is not exist or not in productmode!!!");
		}
	}
	//rf test mode
	else if(os_strcmp(argv[0],"rfmode") == 0)
	{
		wifi_set_mode(WIFI_MODE_RFTEST);

		if(wifidev->wifi_mode != WIFI_MODE_RFTEST)
		{
			sprintf(rsp,"set fail mode:%d", wifidev->wifi_mode);
		}
		else
		{
			wifi_driver_action(0x03);
			sprintf(rsp,"set OK wait 2-3 s then test");
		}

		ATRESP( athandle,1,0,rsp);
	}
	else if(os_strcmp(argv[0],"stainfo") == 0)
	{
		sta_list stal;
		uint8_t n;

		int ret = UAP_STA_List(&stal);

		if(ret == 0)
		{
			for(n=0;n<stal.sta_count;n++)
			{
				sprintf(rsp,"addr:%x:%x:%x",stal.info[n].mac_address[0],stal.info[n].mac_address[1],stal.info[n].mac_address[2]);
				ATRESP( athandle,1,0,rsp);
			}

			sprintf(rsp,"sta:n:%d",stal.sta_count);
			ATRESP( athandle,1,0,rsp);
		}
		else
		{
			ATRESP( athandle,1,0,"read list fail");
		}
	}
	else if(os_strcmp(argv[0],"uapst") == 0)
	{
		uint8_t st = GetWifiStatus();
		uint8_t ist = GetWiFiInitStat();

		sprintf(rsp,"wifi st:%d init:%d\n",st,ist);

		ATRESP( athandle,1,0,rsp);

		return;
	}
	else if(os_strcmp(argv[0],"mac") == 0)
	{
		if(GetWiFiInitStat())
		{
			char mac[6];
			char rsp[32]= {0};

			GetMACAddr((uint8_t*)mac);
			if(argc > 1)
			{
				mac[5] = (char)strtoul(argv[1],NULL,16);
				mac[4] = (char)strtoul(argv[2],NULL,16);
				mac[3] = (char)strtoul(argv[3],NULL,16);
				aic_set_mac_address((uint8_t*)mac);
			}
			sprintf(rsp,"f:%x:%x:%x:%x:%x:%x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
			ATRESP( athandle,1,0,rsp);
		}
		else
			ATRESP( athandle,1,0,"wifi is not init");
	}
	else if(os_strcmp(argv[0],"stanum") == 0)
	{
		int n;
		n = UAP_GetStaNum();
		sprintf(rsp,"n:%d\n",n);
		ATRESP( athandle,1,0,rsp);
	}
	else if(os_strcmp(argv[0],"wpa") == 0)
	{
		wifi_ap_cfg.sercurity_type = (AIC_WIFI_SECURITY_TYPE)strtoul(argv[1],NULL,10);
		if(argc > 2)
			memcpy(wifi_ap_cfg.aic_ap_passwd.array,argv[2],strlen(argv[2])+1);
		UAP_BSS_Config(0);
		rtos_msleep(10);
		UAP_BSS_Config(1);
		sprintf(rsp,"resetart ap t:%d\n",wifi_ap_cfg.sercurity_type);
		ATRESP( athandle,1,0,rsp);
	}
	else if(os_strcmp(argv[0],"mem") == 0)
	{
		uint32_t mcnt,fcnt = 0,rcnt;
		static void *test_p = NULL;
		mcnt = strtoul(argv[1],NULL,10);
		if(mcnt == 0)
		{
			rtos_mem_cur_cnt(&mcnt,&fcnt,&rcnt);
			sprintf(rsp,"mem u:%d  m:%d f:%d\n",mcnt-fcnt,mcnt,fcnt);
			if(test_p)
				rtos_free(test_p);
			ATRESP( athandle,1,0,rsp);
		}
		else if(mcnt == 1)
		{
			rtos_res_list_info_show();
			ATRESP( athandle,1,0,"ok see log");
			return;
		}
		else if(mcnt == 2)
		{
			rtos_remove_all();
			ATRESP( athandle,1,0,"ok see log");
			return;
		}
		else
		{
			test_p = rtos_malloc(mcnt);
		}

        if (mcnt > fcnt)
			sprintf(rsp,"mem u:%d  m:%d f:%d %p\n",mcnt-fcnt,mcnt,fcnt,test_p);
        else
            sprintf(rsp,"mem u:%d  m:%d f:%d %p\n",mcnt+(0xFFFFFFFF-fcnt),mcnt,fcnt,test_p);

		ATRESP( athandle,1,0,rsp);

	}
	else if(os_strcmp(argv[0],"info") == 0)
	{
		uint8_t info[3] = {0};
		char ver[32] = {0};
		aic_wifi_sw_version(ver);
		aic_get_chip_info(info);
		sprintf(rsp,"ver:%s cid:0x%x(%s) sid:%d fmat:%d mode:%d\n",ver,info[0],(info[0]&0xc0)?"H":"T",info[1],info[2],wifidev->wifi_mode);
		ATRESP( athandle,1,0,rsp);
	}
	else if(os_strcmp(argv[0],"wps") == 0)
	{
		extern void WpsMsg_UsePIN(char *pin);
		extern void WpsMsg_UsePBC(void);
		extern void WpsMsg_UseCancel(void);
		extern void WPSApp_Disable(void);
		extern void WPSApp_Enable(void);

		if(os_strcmp(argv[1],"pbc") == 0)
		{
			WpsMsg_UsePBC();
		}
		else if(os_strcmp(argv[1],"pin") == 0)
		{
			WpsMsg_UsePIN(argv[2]);
		}
		else if(os_strcmp(argv[1],"cancel") == 0)
		{
			WpsMsg_UseCancel();
		}
		else if(os_strcmp(argv[1],"disable") == 0)
		{
			WPSApp_Disable();
		}
	}
    else if(os_strcmp(argv[0],"ps") == 0)
    {
        int ret = 0, lvl = 0;

        lvl = strtoul(argv[1],NULL,10);
        ret = aic_wifi_set_ps_lvl(lvl);
        if (!ret) {
            sprintf(rsp, "ap_ps ret: %d\n", ret);
            ATRESP( athandle,1,0,rsp);
        } else {
            ATRESP( athandle,1,0,"+ERR\n");
        }
    }
	else if(os_strcmp(argv[0],"pp") == 0)
	{
		extern void dbg_tune_pp(int para1);
		int lv = strtoul(argv[1],NULL,10);
		dbg_tune_pp(lv);
	}
	else if(os_strcmp(argv[0],"temp") == 0)
    {
        int32_t temp = 0;
        ret = aic_wifi_get_temp(&temp);
        if (!ret)
            sprintf(rsp, "get chip temp: %d\n", temp);
        else
            sprintf(rsp, "get chip temp err: %d\n", ret);
        ATRESP( athandle,1,0,rsp);
    }
    else if(os_strcmp(argv[0],"dpd") == 0)
    {
        uint32_t result = aic_get_dpd_result();
        #ifdef CONFIG_DPD
        sprintf(rsp, "dpd result: %x\n", result);
        ATRESP( athandle,1,0,rsp);
        #else
        ATRESP( athandle,1,0,"+ERR");
        #endif /* CONFIG_DPD */
    }
	else if(os_strcmp(argv[0],"shut") == 0)
	{
		uint32_t op;
		op = (AIC_WIFI_SECURITY_TYPE)strtoul(argv[1],NULL,10);
		if(op == 222)
		{
			sdio_unexcept_test = 1;
		}
		else
		{
			wifi_driver_action(op);
		}
	}
    else if(os_strcmp(argv[0],"fw") == 0)
    {
        sprintf(rsp, "fw ver: %s\n", aic_get_fw_ver());
        ATRESP( athandle,1,0,rsp);
    }
	else if(os_strcmp(argv[0],"filter") == 0)
	{
		extern int UAP_Params_Set_Filter_Only(void *filter);
		uap_mac_filter filter;

		int fmode = strtoul(argv[1],NULL,10);

		filter.filter_mode = fmode;
		filter.mac_count = 1;
		filter.mac_list[0][0] = strtoul(argv[2],NULL,16);
		filter.mac_list[0][1]= strtoul(argv[3],NULL,16);
		filter.mac_list[0][2] = strtoul(argv[4],NULL,16);
		filter.mac_list[0][3] = strtoul(argv[5],NULL,16);
		filter.mac_list[0][4] = strtoul(argv[6],NULL,16);
		filter.mac_list[0][5]= strtoul(argv[7],NULL,16);

		UAP_Params_Set_Filter_Only(&filter);
	}
end:
	ATRESP( athandle,1,ret,NULL);
}
#endif
