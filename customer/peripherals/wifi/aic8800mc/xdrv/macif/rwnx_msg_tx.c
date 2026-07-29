/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rwnx_defs.h"
//#include "rwnx_utils.h"
#include "lmac_msg.h"
#include "aic_rwnx_main.h"
#include "fhostif_cmd.h"
#include "osal_debug.h"
#include "co_utils.h"
//#include "fhost.h"
#include <string.h>
//#include "rf_config.h"
#include "rwnx_platform.h"
//#include "rwnx_main.h"

#define RESIZE4(n) ((n+3)&~3)

extern u8 chip_id;
extern u8 chip_sub_id;
extern u8 chip_mcu_id;

u32 wifi_txgain_table_24g_8800dcdw[32] =
{
    0xA4B22189, //index 0
    0x00007825,
    0xA4B2214B, //index 1
    0x00007825,
    0xA4B2214F, //index 2
    0x00007825,
    0xA4B221D5, //index 3
    0x00007825,
    0xA4B221DC, //index 4
    0x00007825,
    0xA4B221E5, //index 5
    0x00007825,
    0xAC9221E5, //index 6
    0x00006825,
    0xAC9221EF, //index 7
    0x00006825,
    0xBC9221EE, //index 8
    0x00006825,
    0xBC9221FF, //index 9
    0x00006825,
    0xBC9221FF, //index 10
    0x00004025,
    0xB792203F, //index 11
    0x00004026,
    0xDC92203F, //index 12
    0x00004025,
    0xE692203F, //index 13
    0x00004025,
    0xFF92203F, //index 14
    0x00004035,
    0xFFFE203F, //index 15
    0x00004832
};

u32 wifi_txgain_table_24g_1_8800dcdw[32] =
{
    0x096E2011, //index 0
    0x00004001,
    0x096E2015, //index 1
    0x00004001,
    0x096E201B, //index 2
    0x00004001,
    0x116E2018, //index 3
    0x00004001,
    0x116E201E, //index 4
    0x00004001,
    0x116E2023, //index 5
    0x00004001,
    0x196E2021, //index 6
    0x00004001,
    0x196E202B, //index 7
    0x00004001,
    0x216E202B, //index 8
    0x00004001,
    0x236E2027, //index 9
    0x00004001,
    0x236E2031, //index 10
    0x00004001,
    0x246E2039, //index 11
    0x00004001,
    0x26922039, //index 12
    0x00004001,
    0x2E92203F, //index 13
    0x00004001,
    0x3692203F, //index 14
    0x00004001,
    0x3FF2203F, //index 15
    0x00004001,
};

u32 wifi_txgain_table_24g_8800dcdw_h[32] =
{
    0xA55629C9, //index 0
    0x00005825,
    0xAE5629C9, //index 1
    0x00005825,
    0xAD5629CD, //index 2
    0x00005825,
    0xAD5629D1, //index 3
    0x00005825,
    0xAD5629D7, //index 4
    0x00005825,
    0xAD5629DE, //index 5
    0x00005825,
    0xAD5629E6, //index 6
    0x00005825,
    0xBD5629E6, //index 7
    0x00005825,
    0xBD5629F0, //index 8
    0x00005825,
    0xCD5629F0, //index 9
    0x00005825,
    0xE55629F0, //index 10
    0x00005825,
    0xE55629FF, //index 11
    0x00005825,
    0xE55629FF, //index 12
    0x00002825,
    0xE75629FF, //index 13
    0x00002825,
    0xFF5629FF, //index 14
    0x00001825,
    0xFF5628FF, //index 15
    0x00001025,
};

u32 wifi_txgain_table_24g_1_8800dcdw_h[32] =
{
    0x941A2048, //index 0
    0x00001825,
    0x961A2048, //index 1
    0x00001825,
    0x9D1A2048, //index 2
    0x00001825,
    0x9A1A204F, //index 3
    0x00001825,
    0x961A204F, //index 4
    0x00001825,
    0x9A1A2057, //index 5
    0x00001825,
    0x9C1A2057, //index 6
    0x00001825,
    0xA31A205B, //index 7
    0x00001825,
    0xAB1A205B, //index 8
    0x00001825,
    0xAD1A205B, //index 9
    0x00001825,
    0xA71A2064, //index 10
    0x00001825,
    0xAD1A2070, //index 11
    0x00001825,
    0xAD72207F, //index 12
    0x00001825,
    0xBCAE207F, //index 13
    0x00001825,
    0xBFB2207F, //index 14
    0x00001825,
    0xD73A207F, //index 15
    0x00001825,
};

u32 wifi_rxgain_table_24g_20m_8800dcdw[64] =
{
    0x82f282d1,//index 0
    0x9591a324,
    0x80808419,
    0x000000f0,
    0x42f282d1,//index 1
    0x95923524,
    0x80808419,
    0x000000f0,
    0x22f282d1,//index 2
    0x9592c724,
    0x80808419,
    0x000000f0,
    0x02f282d1,//index 3
    0x9591a324,
    0x80808419,
    0x000000f0,
    0x06f282d1,//index 4
    0x9591a324,
    0x80808419,
    0x000000f0,
    0x0ef29ad1,//index 5
    0x9591a324,
    0x80808419,
    0x000000f0,
    0x0ef29ad3,//index 6
    0x95923524,
    0x80808419,
    0x000000f0,
    0x0ef29ad7,//index 7
    0x9595a324,
    0x80808419,
    0x000000f0,
    0x02f282d2,//index 8
    0x95951124,
    0x80808419,
    0x000000f0,
    0x02f282f4,//index 9
    0x95951124,
    0x80808419,
    0x000000f0,
    0x02f282e6,//index 10
    0x9595a324,
    0x80808419,
    0x000000f0,
    0x02f282e6,//index 11
    0x9599a324,
    0x80808419,
    0x000000f0,
    0x02f282e6,//index 12
    0x959da324,
    0x80808419,
    0x000000f0,
    0x02f282e6,//index 13
    0x959f5924,
    0x80808419,
    0x000000f0,
    0x06f282e6,//index 14
    0x959f5924,
    0x80808419,
    0x000000f0,
    0x0ef29ae6,//index 15
    0x959f5924,           //loft [35:34]=3
    0x80808419,
    0x000000f0
};

u32 wifi_rxgain_table_24g_40m_8800dcdw[64] =
{
    0x83428151,//index 0
    0x9631a328,
    0x80808419,
    0x000000f0,
    0x43428151,//index 1
    0x96323528,
    0x80808419,
    0x000000f0,
    0x23428151,//index 2
    0x9632c728,
    0x80808419,
    0x000000f0,
    0x03428151,//index 3
    0x9631a328,
    0x80808419,
    0x000000f0,
    0x07429951,//index 4
    0x9631a328,
    0x80808419,
    0x000000f0,
    0x0f42d151,//index 5
    0x9631a328,
    0x80808419,
    0x000000f0,
    0x0f42d153,//index 6
    0x96323528,
    0x80808419,
    0x000000f0,
    0x0f42d157,//index 7
    0x9635a328,
    0x80808419,
    0x000000f0,
    0x03428152,//index 8
    0x96351128,
    0x80808419,
    0x000000f0,
    0x03428174,//index 9
    0x96351128,
    0x80808419,
    0x000000f0,
    0x03428166,//index 10
    0x9635a328,
    0x80808419,
    0x000000f0,
    0x03428166,//index 11
    0x9639a328,
    0x80808419,
    0x000000f0,
    0x03428166,//index 12
    0x963da328,
    0x80808419,
    0x000000f0,
    0x03428166,//index 13
    0x963f5928,
    0x80808419,
    0x000000f0,
    0x07429966,//index 14
    0x963f5928,
    0x80808419,
    0x000000f0,
    0x0f42d166,//index 15
    0x963f5928,
    0x80808419,
    0x000000f0
};

static inline void *rwnx_msg_zalloc(lmac_msg_id_t const id,
                                    lmac_task_id_t const dest_id,
                                    lmac_task_id_t const src_id,
                                    uint16_t const param_len)
{
    struct lmac_msg *msg;
    msg = (struct lmac_msg *)rtos_malloc(sizeof(struct lmac_msg) + param_len);
    if (msg == NULL)
    {
        DBG_MACIF_INF("%s: msg allocation failed\n", __func__);
        return NULL;
    }
    memset(msg, 0, sizeof(struct lmac_msg));

    msg->id = id;
    msg->dest_id = dest_id;
    msg->src_id = src_id;
    msg->param_len = param_len;

    return msg->param;
}

static void rwnx_msg_free(struct rwnx_hw *rwnx_hw, const void *msg_params)
{
    struct lmac_msg *msg = container_of((void *)msg_params, struct lmac_msg, param);
    //RWNX_DBG(RWNX_FN_ENTRY_STR);
    /* Free the message */
    rtos_free(msg);
}

/******************************************************************************
 *    Control messages handling functions (SOFTMAC and  FULLMAC)
 *****************************************************************************/
#if CONFIG_NOT_DEFINED
int rwnx_send_reset(struct rwnx_hw *rwnx_hw)
{
    void *void_param;

    /* RESET REQ has no parameter */
    void_param = rwnx_msg_zalloc(MM_RESET_REQ, TASK_MM, DRV_TASK_ID, 0);
    if (!void_param)
        return -1;

    return rwnx_host_send_msg(rwnx_hw, void_param, 1, MM_RESET_CFM, NULL);
}

int rwnx_send_start(struct rwnx_hw *rwnx_hw, struct mm_start_req *start)
{
    struct mm_start_req *start_req_param;

    /* Build the START REQ message */
    start_req_param = rwnx_msg_zalloc(MM_START_REQ, TASK_MM, DRV_TASK_ID,
                                      sizeof(struct mm_start_req));
    if (!start_req_param)
        return -1;

    /* Set parameters for the START message */
    memcpy(&start_req_param->phy_cfg,  &start->phy_cfg, sizeof(start->phy_cfg));
    //start_req_param->uapsd_timeout = (u32_l)rwnx_hw->mod_params->uapsd_timeout;
    //start_req_param->lp_clk_accuracy = (u16_l)rwnx_hw->mod_params->lp_clk_ppm;

    /* Send the START REQ message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, start_req_param, 1, MM_START_CFM, NULL);
}

int rwnx_send_version_req(struct rwnx_hw *rwnx_hw, struct mm_version_cfm *cfm)
{
    void *void_param;

    /* VERSION REQ has no parameter */
    void_param = rwnx_msg_zalloc(MM_VERSION_REQ, TASK_MM, DRV_TASK_ID, 0);
    if (!void_param)
        return -1;

    return rwnx_host_send_msg(rwnx_hw, void_param, 1, MM_VERSION_CFM, cfm);

}

int rwnx_send_me_config_req(struct rwnx_hw *rwnx_hw, struct me_config_req *cfg_req)
{
    struct me_config_req *req;
    int ret;

    /* Build the ME_CONFIG_REQ message */
    req = rwnx_msg_zalloc(ME_CONFIG_REQ, TASK_ME, DRV_TASK_ID,
                          sizeof(struct me_config_req));
    if (!req)
        return -1;

    memcpy(req, cfg_req, sizeof_b(struct me_config_req));

    /* Send the ME_CONFIG_REQ message to LMAC FW */
    ret = rwnx_host_send_msg(rwnx_hw, req, 1, ME_CONFIG_CFM, NULL);
    if (ret < 0)
    {
        DBG_MACIF_INF("send msg(ME_CONFIG_REQ) err: %d\n", ret);
    }
    return ret;
}

int rwnx_send_me_chan_config_req(struct rwnx_hw *rwnx_hw, struct me_chan_config_req *chan_cfg)
{
    struct me_chan_config_req *req;
    int ret;

    /* Build the ME_CHAN_CONFIG_REQ message */
    req = rwnx_msg_zalloc(ME_CHAN_CONFIG_REQ, TASK_ME, DRV_TASK_ID,
                          sizeof(struct me_chan_config_req));
    if (!req)
        return -1;

    memcpy(req, chan_cfg, sizeof_b(struct me_chan_config_req));

    ret = rwnx_host_send_msg(rwnx_hw, req, 1, ME_CHAN_CONFIG_CFM, NULL);
    if (ret < 0)
    {
        DBG_MACIF_INF("send msg(ME_CHAN_CONFIG_REQ) err: %d\n", ret);
    }

    return ret;
}

int rwnx_send_me_sta_del(struct rwnx_hw *rwnx_hw, u8 sta_idx, bool tdls_sta)
{
    struct me_sta_del_req *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the MM_STA_DEL_REQ message */
    req = rwnx_msg_zalloc(ME_STA_DEL_REQ, TASK_ME, DRV_TASK_ID,
                          sizeof(struct me_sta_del_req));
    if (!req)
        return -12;//ENOMEM;

    /* Set parameters for the MM_STA_DEL_REQ message */
    req->sta_idx = sta_idx;
    req->tdls_sta = tdls_sta;

    /* Send the ME_STA_DEL_REQ message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, ME_STA_DEL_CFM, NULL);
}

int rwnx_set_hw_idle_req(struct rwnx_hw *rwnx_hw, uint8_t idle)
{
    struct mm_set_idle_req *req;
    int ret;

    /* Build the ADD_IF_REQ message */
    req = rwnx_msg_zalloc(MM_SET_IDLE_REQ, TASK_MM, DRV_TASK_ID,
                          sizeof(struct mm_set_idle_req));
    if (!req)
        return -1;

    ret = rwnx_host_send_msg(rwnx_hw, req, 1, MM_SET_IDLE_CFM, NULL);
    if (ret < 0)
    {
        DBG_MACIF_INF("send msg(MM_SET_IDLE_REQ) err: %d\n", ret);
    }

    return ret;
}

int rwnx_send_add_if(struct rwnx_hw *rwnx_hw, struct mm_add_if_req *req, struct mm_add_if_cfm *cfm)
{
    struct mm_add_if_req *add_if_req_param;
    int ret;

    /* Build the ADD_IF_REQ message */
    add_if_req_param = rwnx_msg_zalloc(MM_ADD_IF_REQ, TASK_MM, DRV_TASK_ID,
                                       sizeof(struct mm_add_if_req));
    if (!add_if_req_param)
        return -1;

    memcpy(add_if_req_param, req, sizeof_b(struct mm_add_if_req));

    ret = rwnx_host_send_msg(rwnx_hw, add_if_req_param, 1, MM_ADD_IF_CFM, cfm);
    if (ret < 0)
    {
        DBG_MACIF_INF("send msg(MM_ADD_IF_REQ) err: %d\n", ret);
    }

    return ret;
}

int rwnx_send_remove_if(struct rwnx_hw *rwnx_hw, struct mm_remove_if_req *rem_req)
{
    struct mm_remove_if_req *remove_if_req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the MM_REMOVE_IF_REQ message */
    remove_if_req = rwnx_msg_zalloc(MM_REMOVE_IF_REQ, TASK_MM, DRV_TASK_ID,
                                    sizeof(struct mm_remove_if_req));
    if (!remove_if_req)
        return -12;//ENOMEM;

    memcpy(remove_if_req, rem_req, sizeof_b(struct mm_remove_if_req));

    /* Send the MM_REMOVE_IF_REQ message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, remove_if_req, 1, MM_REMOVE_IF_CFM, NULL);
}

int rwnx_set_disable_agg_req(struct rwnx_hw *rwnx_hw, uint8_t agg_disable, uint8_t sta_idx)
{
    struct mm_set_agg_disable_req *req;
    int ret;

    /* Build the ADD_IF_REQ message */
    req = rwnx_msg_zalloc(MM_SET_AGG_DISABLE_REQ, TASK_MM, DRV_TASK_ID,
                          sizeof(struct mm_set_agg_disable_req));
    if (!req)
        return -1;

    req->disable = agg_disable;
    req->sta_idx = sta_idx;

    ret = rwnx_host_send_msg(rwnx_hw, req, 1, MM_SET_AGG_DISABLE_CFM, NULL);

    if (ret < 0)
    {
        DBG_MACIF_INF("send msg(MM_SET_AGG_DISABLE_REQ) err: %d\n", ret);
    }

    return ret;
}

int rwnx_set_coex_config_req(struct rwnx_hw *rwnx_hw, uint8_t disable_coexnull, uint8_t enable_periodic_timer, uint8_t enable_nullcts,
                             uint8_t coex_timeslot_set, uint32_t param1, uint32_t param2)
{
    struct mm_set_coex_req *req;
    int ret;

    /* Build the ADD_IF_REQ message */
    req = rwnx_msg_zalloc(MM_SET_COEX_REQ, TASK_MM, DRV_TASK_ID,
                          sizeof(struct mm_set_coex_req));
    if (!req)
        return -1;

    req->bt_on = 1;
    req->disable_coexnull = disable_coexnull;
    req->enable_periodic_timer = enable_periodic_timer;
    req->enable_nullcts = enable_nullcts;
    req->coex_timeslot_set = coex_timeslot_set;
    req->coex_timeslot[0] = param1;
    req->coex_timeslot[1] = param2;

    ret = rwnx_host_send_msg(rwnx_hw, req, 1, MM_SET_COEX_CFM, NULL);

    if (ret < 0)
    {
        DBG_MACIF_INF("send msg(MM_SET_COEX_REQ) err: %d\n", ret);
    }

    return ret;
}

int rwnx_send_rf_config_req(struct rwnx_hw *rwnx_hw, u8_l ofst, u8_l sel, u8_l *tbl, u16_l len)
{
    struct mm_set_rf_config_req *rf_config_req;
    int error;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the MM_SET_RF_CONFIG_REQ message */
    rf_config_req = rwnx_msg_zalloc(MM_SET_RF_CONFIG_REQ, TASK_MM, DRV_TASK_ID,
                                    sizeof(struct mm_set_rf_config_req));

    if (!rf_config_req)
    {
        return -12;//ENOMEM;
    }

    rf_config_req->table_sel = sel;
    rf_config_req->table_ofst = ofst;
    rf_config_req->table_num = 16;
    rf_config_req->deft_page = 0;

    memcpy(rf_config_req->data, tbl, len);

    /* Send the MM_SET_RF_CONFIG_REQ message to UMAC FW */
    error = rwnx_host_send_msg(rwnx_hw, rf_config_req, 1, MM_SET_RF_CONFIG_CFM, NULL);

    return (error);
}

int rwnx_send_rf_calib_req(struct rwnx_hw *rwnx_hw, struct mm_set_rf_calib_cfm *cfm)
{
    struct mm_set_rf_calib_req *req;
    xtal_cap_conf_t xtal_cap = {0,};
    int ret;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the ADD_IF_REQ message */
    req = rwnx_msg_zalloc(MM_SET_RF_CALIB_REQ, TASK_MM, DRV_TASK_ID,
                          sizeof(struct mm_set_rf_calib_req));
    if (!req)
        return -12;//ENOMEM;

    if (rwnx_hw->chipid == PRODUCT_ID_AIC8801)
    {
        req->cal_cfg_24g = 0xbf;
        req->cal_cfg_5g = 0x3f;
    }
    else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)
    {
        req->cal_cfg_24g = 0x0f8f;
        req->cal_cfg_5g = 0;
    }
    else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)
    {
        req->cal_cfg_24g = 0x0f8f;
        req->cal_cfg_5g = 0x0f0f;
    }
    req->param_alpha = 0x0c34c008;
    req->bt_calib_en = 0;
    req->bt_calib_param = 0x264203;

    if (rwnx_hw->chipid == PRODUCT_ID_AIC8801)
        get_nvram_xtal_cap(&xtal_cap);
    else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
             rwnx_hw->chipid == PRODUCT_ID_AIC8800DW ||
             rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)
        get_userconfig_xtal_cap(&xtal_cap);

    if (xtal_cap.enable)
    {
        DBG_MACIF_INF("user ctal cap: %d, cap_fine: %d\n", xtal_cap.xtal_cap, xtal_cap.xtal_cap_fine);
        req->xtal_cap = xtal_cap.xtal_cap;
        req->xtal_cap_fine = xtal_cap.xtal_cap_fine;
    }
    else
    {
        req->xtal_cap = 0;
        req->xtal_cap_fine = 0;
    }

    ret = rwnx_host_send_msg(rwnx_hw, req, 1, MM_SET_RF_CALIB_CFM, cfm);

    if (ret < 0)
    {
        DBG_MACIF_INF("send msg(MM_SET_RF_CALIB_REQ) err: %d\n", ret);
    }

    return ret;
}

int aicwf_set_rf_config_8800dc(struct rwnx_hw *rwnx_hw, struct mm_set_rf_calib_cfm *cfm)
{
    int ret = 0;

    if ((ret = rwnx_send_txpwr_lvl_req(rwnx_hw)))
    {
        return -1;
    }

    if ((ret = rwnx_send_txpwr_ofst_req(rwnx_hw)))
    {
        return -1;
    }

    if (rwnx_hw->mode != WIFI_MODE_RFTEST)
    {
        if (IS_CHIP_ID_H())
        {
            if ((ret = rwnx_send_rf_config_req(rwnx_hw, 0,    1, (u8_l *)wifi_txgain_table_24g_8800dcdw_h, 128)))
                return -1;

            if ((ret = rwnx_send_rf_config_req(rwnx_hw, 16,    1, (u8_l *)wifi_txgain_table_24g_1_8800dcdw_h, 128)))
                return -1;
        }
        else
        {
            if ((ret = rwnx_send_rf_config_req(rwnx_hw, 0,    1, (u8_l *)wifi_txgain_table_24g_8800dcdw, 128)))
                return -1;

            if ((ret = rwnx_send_rf_config_req(rwnx_hw, 16,    1, (u8_l *)wifi_txgain_table_24g_1_8800dcdw, 128)))
                return -1;
        }

        if ((ret = rwnx_send_rf_config_req(rwnx_hw, 0,    0, (u8_l *)wifi_rxgain_table_24g_20m_8800dcdw, 256)))
            return -1;

        if ((ret = rwnx_send_rf_config_req(rwnx_hw, 32,  0, (u8_l *)wifi_rxgain_table_24g_40m_8800dcdw, 256)))
            return -1;

        if ((ret = rwnx_send_rf_calib_req(rwnx_hw, cfm)))
        {
            return -1;
        }
    }
    else if (rwnx_hw->mode == WIFI_MODE_RFTEST)
    {
        if (chip_sub_id >= 1)
        {
#ifdef CONFIG_DPD
#if 0 //ifndef CONFIG_FORCE_DPD_CALIB
            if (is_file_exist(FW_DPDRESULT_NAME_8800DC) == 1)
            {
                DBG_MACIF_INF("%s load dpd bin\n", __func__);
                ret = aicwf_dpd_result_load_8800dc(rwnx_hw, &dpd_res);
                if (ret)
                {
                    DBG_MACIF_INF("load dpd bin fail: %d\n", ret);
                    return ret;
                }
            }
#endif
            if (dpd_res.bit_mask[1])
            {
                ret = aicwf_dpd_result_apply_8800dc(rwnx_hw, &dpd_res);
                if (ret)
                {
                    DBG_MACIF_INF("apply dpd bin fail: %d\n", ret);
                    return ret;
                }
            }
#else
            {
                ret = aicwf_misc_ram_init_8800dc(rwnx_hw);
                if (ret)
                {
                    DBG_MACIF_INF("misc ram init fail: %d\n", ret);
                    return ret;
                }
            }
#endif
            ret = rwnx_send_rf_calib_req(rwnx_hw, &cfm);
            if (ret)
            {
                DBG_MACIF_INF("rf calib req fail: %d\n", ret);
                return -1;
            }
        }
    }

    return 0 ;
}

int aicwf_set_rf_config_8800d80(struct rwnx_hw *rwnx_hw, struct mm_set_rf_calib_cfm *cfm)
{
    int ret = 0;

    if ((ret = rwnx_send_txpwr_lvl_v3_req(rwnx_hw)))
    {
        return -1;
    }

    if ((ret = rwnx_send_txpwr_ofst2x_req(rwnx_hw)))
    {
        return -1;
    }

    if (rwnx_hw->mode != WIFI_MODE_RFTEST)
    {
        if ((ret = rwnx_send_rf_calib_req(rwnx_hw, cfm)))
        {
            return -1;
        }
    }

    return 0 ;
}

int rwnx_send_set_stack_start_req(struct rwnx_hw *rwnx_hw, u8_l on, u8_l efuse_valid, u8_l set_vendor_info,
                                  u8_l fwtrace_redir_en, struct mm_set_stack_start_cfm *cfm)
{
    struct mm_set_stack_start_req *req;
    int error;

    /* Build the MM_SET_STACK_START_REQ message */
    req = rwnx_msg_zalloc(MM_SET_STACK_START_REQ, TASK_MM, DRV_TASK_ID, sizeof(struct mm_set_stack_start_req));

    if (!req)
    {
        return -12;//ENOMEM;
    }

    req->is_stack_start = on;
    req->efuse_valid = efuse_valid;
    req->set_vendor_info = set_vendor_info;
    req->fwtrace_redir = fwtrace_redir_en;
    /* Send the MM_SET_STACK_START_REQ  message to UMAC FW */
    error = rwnx_host_send_msg(rwnx_hw, req, 1, MM_SET_STACK_START_CFM, cfm);

    return error;
}

int rwnx_send_txpwr_idx_req(struct rwnx_hw *rwnx_hw)
{
    struct mm_set_txpwr_idx_req *txpwr_idx_req;
    txpwr_idx_conf_t *txpwr_idx;
    int error;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the MM_SET_TXPWR_IDX_REQ message */
    txpwr_idx_req = rwnx_msg_zalloc(MM_SET_TXPWR_IDX_LVL_REQ, TASK_MM, DRV_TASK_ID,
                                    sizeof(struct mm_set_txpwr_idx_req));

    if (!txpwr_idx_req)
    {
        return -12;//ENOMEM;
    }

    txpwr_idx = &txpwr_idx_req->txpwr_idx;
    txpwr_idx->enable = 1;
    txpwr_idx->dsss = 9;
    txpwr_idx->ofdmlowrate_2g4 = 8;
    txpwr_idx->ofdm64qam_2g4 = 8;
    txpwr_idx->ofdm256qam_2g4 = 8;
    txpwr_idx->ofdm1024qam_2g4 = 8;
    txpwr_idx->ofdmlowrate_5g = 10;
    txpwr_idx->ofdm64qam_5g = 9;
    txpwr_idx->ofdm256qam_5g = 9;
    txpwr_idx->ofdm1024qam_5g = 8;

#ifdef CONFIG_LOAD_USERCONFIG
    get_nvram_txpwr_idx(txpwr_idx);
#endif

    if (txpwr_idx->enable == 0)
    {
        rwnx_msg_free(rwnx_hw, txpwr_idx_req);
        return 0;
    }
    else
    {
        DBG_MACIF_INF("%s:enable:%d\r\n", __func__, txpwr_idx->enable);
        DBG_MACIF_INF("%s:dsss:%d\r\n", __func__, txpwr_idx->dsss);
        DBG_MACIF_INF("%s:ofdmlowrate_2g4:%d\r\n", __func__, txpwr_idx->ofdmlowrate_2g4);
        DBG_MACIF_INF("%s:ofdm64qam_2g4:%d\r\n", __func__, txpwr_idx->ofdm64qam_2g4);
        DBG_MACIF_INF("%s:ofdm256qam_2g4:%d\r\n", __func__, txpwr_idx->ofdm256qam_2g4);
        DBG_MACIF_INF("%s:ofdm1024qam_2g4:%d\r\n", __func__, txpwr_idx->ofdm1024qam_2g4);
        DBG_MACIF_INF("%s:ofdmlowrate_5g:%d\r\n", __func__, txpwr_idx->ofdmlowrate_5g);
        DBG_MACIF_INF("%s:ofdm64qam_5g:%d\r\n", __func__, txpwr_idx->ofdm64qam_5g);
        DBG_MACIF_INF("%s:ofdm256qam_5g:%d\r\n", __func__, txpwr_idx->ofdm256qam_5g);
        DBG_MACIF_INF("%s:ofdm1024qam_5g:%d\r\n", __func__, txpwr_idx->ofdm1024qam_5g);

        /* Send the MM_SET_TXPWR_IDX_REQ message to UMAC FW */
        error = rwnx_host_send_msg(rwnx_hw, txpwr_idx_req, 1, MM_SET_TXPWR_IDX_LVL_CFM, NULL);

        return (error);
    }
}

int rwnx_send_txpwr_lvl_req(struct rwnx_hw *rwnx_hw)
{
    struct mm_set_txpwr_lvl_req *txpwr_lvl_req;
    txpwr_lvl_conf_v2_t txpwr_lvl_v2_tmp;
    txpwr_lvl_conf_v2_t *txpwr_lvl_v2;
    int error;
    int i;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the MM_SET_TXPWR_LVL_REQ message */
    txpwr_lvl_req = rwnx_msg_zalloc(MM_SET_TXPWR_IDX_LVL_REQ, TASK_MM, DRV_TASK_ID,
                                    sizeof(struct mm_set_txpwr_lvl_req));

    if (!txpwr_lvl_req)
    {
        return -12;//ENOMEM;
    }

    txpwr_lvl_v2 = &txpwr_lvl_v2_tmp;

#ifdef CONFIG_LOAD_USERCONFIG
    get_userconfig_txpwr_lvl_v2(txpwr_lvl_v2);
#endif

    if (txpwr_lvl_v2->enable == 0)
    {
        rwnx_msg_free(rwnx_hw, txpwr_lvl_req);
        return 0;
    }
    else
    {
        DBG_MACIF_INF("%s:enable:%d\r\n",               __func__, txpwr_lvl_v2->enable);
        DBG_MACIF_INF("%s:lvl_11b_11ag_1m_2g4:%d\r\n",  __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[0]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_2m_2g4:%d\r\n",  __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[1]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_5m5_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[2]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_11m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[3]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_6m_2g4:%d\r\n",  __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[4]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_9m_2g4:%d\r\n",  __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[5]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_12m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[6]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_18m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[7]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_24m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[8]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_36m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[9]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_48m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[10]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_54m_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[11]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs0_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[0]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs1_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[1]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs2_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[2]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs3_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[3]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs4_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[4]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs5_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[5]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs6_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[6]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs7_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[7]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs8_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[8]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs9_2g4:%d\r\n", __func__, txpwr_lvl_v2->pwrlvl_11n_11ac_2g4[9]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs0_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[0]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs1_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[1]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs2_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[2]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs3_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[3]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs4_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[4]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs5_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[5]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs6_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[6]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs7_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[7]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs8_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[8]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs9_2g4:%d\r\n",    __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[9]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs10_2g4:%d\r\n",   __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[10]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs11_2g4:%d\r\n",   __func__, txpwr_lvl_v2->pwrlvl_11ax_2g4[11]);

        if ((rwnx_hw->mode != WIFI_MODE_RFTEST) && (chip_sub_id == 0))
        {
            txpwr_lvl_req->txpwr_lvl.enable         = txpwr_lvl_v2->enable;
            txpwr_lvl_req->txpwr_lvl.dsss           = txpwr_lvl_v2->pwrlvl_11b_11ag_2g4[3]; // 11M
            txpwr_lvl_req->txpwr_lvl.ofdmlowrate_2g4 = txpwr_lvl_v2->pwrlvl_11ax_2g4[4]; // MCS4
            txpwr_lvl_req->txpwr_lvl.ofdm64qam_2g4  = txpwr_lvl_v2->pwrlvl_11ax_2g4[7]; // MCS7
            txpwr_lvl_req->txpwr_lvl.ofdm256qam_2g4 = txpwr_lvl_v2->pwrlvl_11ax_2g4[9]; // MCS9
            txpwr_lvl_req->txpwr_lvl.ofdm1024qam_2g4 = txpwr_lvl_v2->pwrlvl_11ax_2g4[11]; // MCS11
            txpwr_lvl_req->txpwr_lvl.ofdmlowrate_5g = 13; // unused
            txpwr_lvl_req->txpwr_lvl.ofdm64qam_5g   = 13; // unused
            txpwr_lvl_req->txpwr_lvl.ofdm256qam_5g  = 13; // unused
            txpwr_lvl_req->txpwr_lvl.ofdm1024qam_5g = 13; // unused
        }
        else
        {
            txpwr_lvl_req->txpwr_lvl_v2  = *txpwr_lvl_v2;
        }

        /* Send the MM_SET_TXPWR_LVL_REQ message to UMAC FW */
        error = rwnx_host_send_msg(rwnx_hw, txpwr_lvl_req, 1, MM_SET_TXPWR_IDX_LVL_CFM, NULL);

        return (error);
    }
}

int rwnx_send_txpwr_lvl_v3_req(struct rwnx_hw *rwnx_hw)
{
    struct mm_set_txpwr_lvl_req *txpwr_lvl_req;
    txpwr_lvl_conf_v3_t txpwr_lvl_v3_tmp;
    txpwr_lvl_conf_v3_t *txpwr_lvl_v3;
    int error;
    int i;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the MM_SET_TXPWR_LVL_REQ message */
    txpwr_lvl_req = rwnx_msg_zalloc(MM_SET_TXPWR_IDX_LVL_REQ, TASK_MM, DRV_TASK_ID,
                                    sizeof(struct mm_set_txpwr_lvl_req));

    if (!txpwr_lvl_req)
    {
        return -12;//ENOMEM;
    }

    txpwr_lvl_v3 = &txpwr_lvl_v3_tmp;

#ifdef CONFIG_LOAD_USERCONFIG
    get_userconfig_txpwr_lvl_v3(txpwr_lvl_v3);
#endif

    if (txpwr_lvl_v3->enable == 0)
    {
        rwnx_msg_free(rwnx_hw, txpwr_lvl_req);
        return 0;
    }
    else
    {
        DBG_MACIF_INF("%s:enable:%d\r\n",               __func__, txpwr_lvl_v3->enable);
        DBG_MACIF_INF("%s:lvl_11b_11ag_1m_2g4:%d\r\n",  __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[0]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_2m_2g4:%d\r\n",  __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[1]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_5m5_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[2]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_11m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[3]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_6m_2g4:%d\r\n",  __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[4]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_9m_2g4:%d\r\n",  __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[5]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_12m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[6]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_18m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[7]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_24m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[8]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_36m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[9]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_48m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[10]);
        DBG_MACIF_INF("%s:lvl_11b_11ag_54m_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11b_11ag_2g4[11]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs0_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[0]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs1_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[1]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs2_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[2]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs3_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[3]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs4_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[4]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs5_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[5]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs6_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[6]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs7_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[7]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs8_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[8]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs9_2g4:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_2g4[9]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs0_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[0]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs1_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[1]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs2_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[2]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs3_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[3]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs4_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[4]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs5_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[5]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs6_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[6]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs7_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[7]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs8_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[8]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs9_2g4:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[9]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs10_2g4:%d\r\n",   __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[10]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs11_2g4:%d\r\n",   __func__, txpwr_lvl_v3->pwrlvl_11ax_2g4[11]);

        DBG_MACIF_INF("%s:lvl_11a_1m_5g:%d\r\n",        __func__, txpwr_lvl_v3->pwrlvl_11a_5g[0]);
        DBG_MACIF_INF("%s:lvl_11a_2m_5g:%d\r\n",        __func__, txpwr_lvl_v3->pwrlvl_11a_5g[1]);
        DBG_MACIF_INF("%s:lvl_11a_5m5_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[2]);
        DBG_MACIF_INF("%s:lvl_11a_11m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[3]);
        DBG_MACIF_INF("%s:lvl_11a_6m_5g:%d\r\n",        __func__, txpwr_lvl_v3->pwrlvl_11a_5g[4]);
        DBG_MACIF_INF("%s:lvl_11a_9m_5g:%d\r\n",        __func__, txpwr_lvl_v3->pwrlvl_11a_5g[5]);
        DBG_MACIF_INF("%s:lvl_11a_12m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[6]);
        DBG_MACIF_INF("%s:lvl_11a_18m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[7]);
        DBG_MACIF_INF("%s:lvl_11a_24m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[8]);
        DBG_MACIF_INF("%s:lvl_11a_36m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[9]);
        DBG_MACIF_INF("%s:lvl_11a_48m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[10]);
        DBG_MACIF_INF("%s:lvl_11a_54m_5g:%d\r\n",       __func__, txpwr_lvl_v3->pwrlvl_11a_5g[11]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs0_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[0]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs1_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[1]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs2_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[2]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs3_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[3]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs4_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[4]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs5_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[5]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs6_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[6]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs7_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[7]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs8_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[8]);
        DBG_MACIF_INF("%s:lvl_11n_11ac_mcs9_5g:%d\r\n", __func__, txpwr_lvl_v3->pwrlvl_11n_11ac_5g[9]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs0_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[0]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs1_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[1]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs2_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[2]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs3_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[3]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs4_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[4]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs5_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[5]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs6_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[6]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs7_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[7]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs8_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[8]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs9_5g:%d\r\n",     __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[9]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs10_5g:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[10]);
        DBG_MACIF_INF("%s:lvl_11ax_mcs11_5g:%d\r\n",    __func__, txpwr_lvl_v3->pwrlvl_11ax_5g[11]);

        txpwr_lvl_req->txpwr_lvl_v3  = *txpwr_lvl_v3;

        /* Send the MM_SET_TXPWR_LVL_REQ message to UMAC FW */
        error = rwnx_host_send_msg(rwnx_hw, txpwr_lvl_req, 1, MM_SET_TXPWR_IDX_LVL_CFM, NULL);

        return (error);
    }
}

int rwnx_send_txpwr_ofst_req(struct rwnx_hw *rwnx_hw)
{
    struct mm_set_txpwr_ofst_req *txpwr_ofst_req;
    txpwr_ofst_conf_t *txpwr_ofst;
    int error;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the MM_SET_TXPWR_OFST_REQ message */
    txpwr_ofst_req = rwnx_msg_zalloc(MM_SET_TXPWR_OFST_REQ, TASK_MM, DRV_TASK_ID,
                                     sizeof(struct mm_set_txpwr_ofst_req));

    if (!txpwr_ofst_req)
    {
        return -12;//ENOMEM;
    }

    txpwr_ofst = &txpwr_ofst_req->txpwr_ofst;
    txpwr_ofst->enable       = 1;
    txpwr_ofst->chan_1_4     = 0;
    txpwr_ofst->chan_5_9     = 0;
    txpwr_ofst->chan_10_13   = 0;
    txpwr_ofst->chan_36_64   = 0;
    txpwr_ofst->chan_100_120 = 0;
    txpwr_ofst->chan_122_140 = 0;
    txpwr_ofst->chan_142_165 = 0;

#ifdef CONFIG_LOAD_USERCONFIG
    if (rwnx_hw->chipid == PRODUCT_ID_AIC8801)
    {
        get_nvram_txpwr_ofst(txpwr_ofst);
    }
    else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC ||
             rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)
    {
        get_userconfig_txpwr_ofst(txpwr_ofst);
    }
#endif

    if (txpwr_ofst->enable == 0)
    {
        rwnx_msg_free(rwnx_hw, txpwr_ofst_req);
        return 0;
    }
    else
    {
        DBG_MACIF_INF("%s:enable      :%d\r\n", __func__, txpwr_ofst->enable);
        DBG_MACIF_INF("%s:chan_1_4    :%d\r\n", __func__, txpwr_ofst->chan_1_4);
        DBG_MACIF_INF("%s:chan_5_9    :%d\r\n", __func__, txpwr_ofst->chan_5_9);
        DBG_MACIF_INF("%s:chan_10_13  :%d\r\n", __func__, txpwr_ofst->chan_10_13);
        DBG_MACIF_INF("%s:chan_36_64  :%d\r\n", __func__, txpwr_ofst->chan_36_64);
        DBG_MACIF_INF("%s:chan_100_120:%d\r\n", __func__, txpwr_ofst->chan_100_120);
        DBG_MACIF_INF("%s:chan_122_140:%d\r\n", __func__, txpwr_ofst->chan_122_140);
        DBG_MACIF_INF("%s:chan_142_165:%d\r\n", __func__, txpwr_ofst->chan_142_165);

        /* Send the MM_SET_TXPWR_OFST_REQ message to UMAC FW */
        error = rwnx_host_send_msg(rwnx_hw, txpwr_ofst_req, 1, MM_SET_TXPWR_OFST_CFM, NULL);

        return (error);
    }
}

int rwnx_send_txpwr_ofst2x_req(struct rwnx_hw *rwnx_hw)
{
    struct mm_set_txpwr_ofst_req *txpwr_ofst_req;
    txpwr_ofst2x_conf_t *txpwr_ofst2x;
    int error = 0;
    int type, ch_grp;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the MM_SET_TXPWR_OFST_REQ message */
    txpwr_ofst_req = rwnx_msg_zalloc(MM_SET_TXPWR_OFST_REQ, TASK_MM, DRV_TASK_ID,
                                     sizeof(struct mm_set_txpwr_ofst_req));

    if (!txpwr_ofst_req)
    {
        return -12; //ENOMEM;
    }

    txpwr_ofst2x = &txpwr_ofst_req->txpwr_ofst2x;
    txpwr_ofst2x->enable = 0;
    for (type = 0; type < 3; type++)
    {
        for (ch_grp = 0; ch_grp < 6; ch_grp++)
        {
            if (ch_grp < 3)
            {
                txpwr_ofst2x->pwrofst2x_tbl_2g4[type][ch_grp] = 0;
            }
            txpwr_ofst2x->pwrofst2x_tbl_5g[type][ch_grp] = 0;
        }
    }
    if (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)
    {
        get_userconfig_txpwr_ofst2x(txpwr_ofst2x);
    }
    if (txpwr_ofst2x->enable)
    {
        DBG_MACIF_INF("%s:enable:%d\r\n", __func__, txpwr_ofst2x->enable);
        DBG_MACIF_INF("pwrofst2x 2.4g: [0]:11b, [1]:ofdm_highrate, [2]:ofdm_lowrate\n"
                      "  chan=" "\t1-4" "\t5-9" "\t10-13");
        for (type = 0; type < 3; type++)
        {
            DBG_MACIF_INF("\n  [%d] =", type);
            for (ch_grp = 0; ch_grp < 3; ch_grp++)
            {
                DBG_MACIF_INF("\t%d", txpwr_ofst2x->pwrofst2x_tbl_2g4[type][ch_grp]);
            }
        }
        DBG_MACIF_INF("\npwrofst2x 5g: [0]:ofdm_lowrate, [1]:ofdm_highrate, [2]:ofdm_midrate\n"
                      "  chan=" "\t36-50" "\t51-64" "\t98-114" "\t115-130" "\t131-146" "\t147-166");
        for (type = 0; type < 3; type++)
        {
            DBG_MACIF_INF("\n  [%d] =", type);
            for (ch_grp = 0; ch_grp < 6; ch_grp++)
            {
                DBG_MACIF_INF("\t%d", txpwr_ofst2x->pwrofst2x_tbl_5g[type][ch_grp]);
            }
        }
        DBG_MACIF_INF("\n");

        /* Send the MM_SET_TXPWR_OFST_REQ message to UMAC FW */
        error = rwnx_host_send_msg(rwnx_hw, txpwr_ofst_req, 1, MM_SET_TXPWR_OFST_CFM, NULL);
    }
    else
    {
        DBG_MACIF_INF("%s:Do not use txpwr_ofst2x\r\n", __func__);
        rwnx_msg_free(rwnx_hw, txpwr_ofst_req);
    }

    return (error);
}


int rwnx_send_sm_connect_req(struct rwnx_hw *rwnx_hw,
                             struct sm_connect_req *conn_req,
                             struct sm_connect_cfm *cfm)
{
    struct sm_connect_req *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the SM_CONNECT_REQ message */
    req = rwnx_msg_zalloc(SM_CONNECT_REQ, TASK_SM, DRV_TASK_ID,
                          sizeof(struct sm_connect_req));
    if (!req)
        return -12;//ENOMEM;

    //memcpy(req, conn_req, sizeof_b(struct sm_connect_req));
    *req = *conn_req;

    /* Send the SM_CONNECT_REQ message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, SM_CONNECT_CFM, cfm);
}

int rwnx_send_sm_disconnect_req(struct rwnx_hw *rwnx_hw,
                                struct sm_disconnect_req *disconn_req)
{
    struct sm_disconnect_req *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the SM_DISCONNECT_REQ message */
    req = rwnx_msg_zalloc(SM_DISCONNECT_REQ, TASK_SM, DRV_TASK_ID,
                          sizeof(struct sm_disconnect_req));
    if (!req)
        return -12;//ENOMEM;

    *req = *disconn_req;

    /* Send the SM_DISCONNECT_REQ message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, SM_DISCONNECT_CFM, NULL);
}

int rwnx_send_scanu_req(struct rwnx_hw *rwnx_hw, struct scanu_start_req *start_req)
{
    struct scanu_start_req *req = NULL;
    int err;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the SCANU_START_REQ message */
    req = rwnx_msg_zalloc(SCANU_START_REQ, TASK_SCANU, DRV_TASK_ID,
                          sizeof(struct scanu_start_req));
    if (!req)
        return -12;//ENOMEM;

    memcpy(req, start_req, sizeof_b(struct scanu_start_req));

    /* Send the SCANU_START_REQ message to LMAC FW */
    err = rwnx_host_send_msg(rwnx_hw, req, 1,  SCANU_START_CFM_ADDTIONAL, NULL);
    if (err)
    {
        DBG_MACIF_INF("send msg(SCANU_START_REQ) err: %d\n", err);
    }
    return err;
}
#endif

/**********************************************************************
 *    Debug Messages
 *********************************************************************/
int rwnx_send_dbg_trigger_req(struct rwnx_hw *rwnx_hw, char *msg)
{
    struct mm_dbg_trigger_req *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the MM_DBG_TRIGGER_REQ message */
    req = rwnx_msg_zalloc(MM_DBG_TRIGGER_REQ, TASK_MM, DRV_TASK_ID,
                          sizeof(struct mm_dbg_trigger_req));
    if (!req)
        return -12;//ENOMEM;

    /* Set parameters for the MM_DBG_TRIGGER_REQ message */
    strncpy(req->error, msg, sizeof(req->error));

    /* Send the MM_DBG_TRIGGER_REQ message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 0, -1, NULL);
}

int rwnx_send_dbg_mem_read_req(struct rwnx_hw *rwnx_hw, u32 mem_addr,
                               struct dbg_mem_read_cfm *cfm)
{
    struct dbg_mem_read_req *mem_read_req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the DBG_MEM_READ_REQ message */
    mem_read_req = rwnx_msg_zalloc(DBG_MEM_READ_REQ, TASK_DBG, DRV_TASK_ID,
                                   sizeof(struct dbg_mem_read_req));
    if (!mem_read_req)
        return -12;//ENOMEM;

    /* Set parameters for the DBG_MEM_READ_REQ message */
    mem_read_req->memaddr = mem_addr;

    /* Send the DBG_MEM_READ_REQ message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, mem_read_req, 1, DBG_MEM_READ_CFM, cfm);
}

int rwnx_send_dbg_mem_write_req(struct rwnx_hw *rwnx_hw, u32 mem_addr,
                                u32 mem_data)
{
    struct dbg_mem_write_req *mem_write_req;

    //RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the DBG_MEM_WRITE_REQ message */
    mem_write_req = rwnx_msg_zalloc(DBG_MEM_WRITE_REQ, TASK_DBG, DRV_TASK_ID,
                                    sizeof(struct dbg_mem_write_req));
    if (!mem_write_req)
        return -12;//ENOMEM;

    /* Set parameters for the DBG_MEM_WRITE_REQ message */
    mem_write_req->memaddr = mem_addr;
    mem_write_req->memdata = mem_data;

    /* Send the DBG_MEM_WRITE_REQ message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, mem_write_req, 1, DBG_MEM_WRITE_CFM, NULL);
}

int rwnx_send_dbg_mem_mask_write_req(struct rwnx_hw *rwnx_hw, u32 mem_addr,
                                     u32 mem_mask, u32 mem_data)
{
    struct dbg_mem_mask_write_req *mem_mask_write_req;

    //RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the DBG_MEM_MASK_WRITE_REQ message */
    mem_mask_write_req = rwnx_msg_zalloc(DBG_MEM_MASK_WRITE_REQ, TASK_DBG, DRV_TASK_ID,
                                         sizeof(struct dbg_mem_mask_write_req));
    if (!mem_mask_write_req)
        return -12;//ENOMEM;

    /* Set parameters for the DBG_MEM_MASK_WRITE_REQ message */
    mem_mask_write_req->memaddr = mem_addr;
    mem_mask_write_req->memmask = mem_mask;
    mem_mask_write_req->memdata = mem_data;

    /* Send the DBG_MEM_MASK_WRITE_REQ message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, mem_mask_write_req, 1, DBG_MEM_MASK_WRITE_CFM, NULL);
}

int rwnx_send_dbg_mem_block_write_req(struct rwnx_hw *rwnx_hw, u32 mem_addr,
                                      u32 mem_size, u32 *mem_data)
{
    struct dbg_mem_block_write_req *mem_blk_write_req;

    //RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the DBG_MEM_BLOCK_WRITE_REQ message */
    mem_blk_write_req = rwnx_msg_zalloc(DBG_MEM_BLOCK_WRITE_REQ, TASK_DBG, DRV_TASK_ID,
                                        sizeof(struct dbg_mem_block_write_req) - 1024 + mem_size);
    if (!mem_blk_write_req)
        return -12;//ENOMEM;

    /* Set parameters for the DBG_MEM_BLOCK_WRITE_REQ message */
    mem_blk_write_req->memaddr = mem_addr;
    mem_blk_write_req->memsize = mem_size;
    memcpy(mem_blk_write_req->memdata, mem_data, mem_size);

    /* Send the DBG_MEM_BLOCK_WRITE_REQ message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, mem_blk_write_req, 1, DBG_MEM_BLOCK_WRITE_CFM, NULL);
}

int rwnx_send_dbg_start_app_req(struct rwnx_hw *rwnx_hw, u32 boot_addr,
                                u32 boot_type)
{
    struct dbg_start_app_req *start_app_req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the DBG_START_APP_REQ message */
    start_app_req = rwnx_msg_zalloc(DBG_START_APP_REQ, TASK_DBG, DRV_TASK_ID,
                                    sizeof(struct dbg_start_app_req));
    if (!start_app_req)
        return -12;//ENOMEM;

    /* Set parameters for the DBG_START_APP_REQ message */
    start_app_req->bootaddr = boot_addr;
    start_app_req->boottype = boot_type;

    /* Send the DBG_START_APP_REQ message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, start_app_req, 1, DBG_START_APP_CFM, NULL);
}

int rwnx_send_fhcustmsg_connect_req(struct rwnx_hw *rwnx_hw, char *ssid, char *passwd)
{
    struct fhcustmsg_connect_req *req;
    int ssid_len, pw_len;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    if (ssid == NULL)
    {
        return -1;
    }
    ssid_len = strlen(ssid);
    if (ssid_len >= AP_SSID_BUF_MAX)
        return -2;
    if (passwd == NULL)
    {
        pw_len = 0;
    }
    else
    {
        pw_len = strlen(passwd);
        if (pw_len >= AP_PSWD_BUF_MAX)
            return -3;
    }

    /* Build the message */
    req = rwnx_msg_zalloc(CUSTOM_MSG_CONNECT_REQ, TASK_DBG, DRV_TASK_ID,
                          sizeof(struct fhcustmsg_connect_req));
    if (!req)
        return -12;//ENOMEM;

    /* Set parameters for the message */
    strncpy((char *)req->ssid, ssid, ssid_len);
    if (pw_len)
    {
        strncpy((char *)req->pw, passwd, pw_len);
    }
    req->ssid[ssid_len] = '\0';
    req->pw[pw_len] = '\0';

    /* Send the message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, CUSTOM_MSG_CONNECT_CFM, NULL);
}

int rwnx_send_fhcustmsg_disconnect_req(struct rwnx_hw *rwnx_hw)
{
    void *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the message */
    req = rwnx_msg_zalloc(CUSTOM_MSG_DISCONNECT_REQ, TASK_DBG, DRV_TASK_ID,
                          1);
    if (!req)
        return -12;//ENOMEM;

    /* Send the message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, CUSTOM_MSG_DISCONNECT_CFM, NULL);
}

int rwnx_send_fhcustmsg_enter_sleep_req(struct rwnx_hw *rwnx_hw, uint8_t lvl, struct fhcustmsg_sleep_cfm *cfm)
{
    struct fhcustmsg_sleep_req *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    if (lvl != PM_LEVEL_LIGHT_SLEEP && lvl != PM_LEVEL_DEEP_SLEEP)
        return -1;

    /* Build the message */
    req = rwnx_msg_zalloc(CUSTOM_MSG_ENTER_SLEEP_REQ, TASK_DBG, DRV_TASK_ID,
                          sizeof(struct fhcustmsg_sleep_req));
    if (!req)
        return -12;//ENOMEM;

    req->sleep_lvl = lvl;
    /* Send the message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, CUSTOM_MSG_ENTER_SLEEP_CFM, cfm);
}

int rwnx_send_fhcustmsg_exit_sleep_req(struct rwnx_hw *rwnx_hw)
{
    void *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the message */
    req = rwnx_msg_zalloc(CUSTOM_MSG_EXIT_SLEEP_REQ, TASK_DBG, DRV_TASK_ID,
                          1);
    if (!req)
        return -12;//ENOMEM;

    /* Send the message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, CUSTOM_MSG_EXIT_SLEEP_CFM, NULL);
}

int rwnx_send_fhcustmsg_set_mac_addr_req(struct rwnx_hw *rwnx_hw, uint8_t *mac_addr)
{
    struct fhcustmsg_set_mac_req *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    if (mac_addr == NULL)
        return -1;

    /* Build the message */
    req = rwnx_msg_zalloc(CUSTOM_MSG_SET_MAC_ADDR_REQ, TASK_DBG, DRV_TASK_ID,
                          sizeof(struct fhcustmsg_set_mac_req));
    if (!req)
        return -12;//ENOMEM;

    rtos_memcpy(req->mac_addr, mac_addr, 6);

    /* Send the message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, CUSTOM_MSG_GET_MAC_ADDR_CFM, NULL);
}

int rwnx_send_fhcustmsg_get_mac_addr_req(struct rwnx_hw *rwnx_hw, struct fhcustmsg_mac_addr_cfm *cfm)
{
    void *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the message */
    req = rwnx_msg_zalloc(CUSTOM_MSG_GET_MAC_ADDR_REQ, TASK_DBG, DRV_TASK_ID,
                          1);
    if (!req)
        return -12;//ENOMEM;

    /* Send the message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, CUSTOM_MSG_GET_MAC_ADDR_CFM, cfm);
}

int rwnx_send_fhcustmsg_wlan_status_req(struct rwnx_hw *rwnx_hw, struct fhcustmsg_wlan_status_cfm *cfm)
{
    void *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the message */
    req = rwnx_msg_zalloc(CUSTOM_MSG_GET_WLAN_STATUS_REQ, TASK_DBG, DRV_TASK_ID,
                          1);
    if (!req)
        return -12;//ENOMEM;

    /* Send the message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, CUSTOM_MSG_GET_WLAN_STATUS_CFM, cfm);
}

int rwnx_send_fhcustmsg_scan_req(struct rwnx_hw *rwnx_hw)
{
    void *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the message */
    req = rwnx_msg_zalloc(CUSTOM_MSG_SCAN_WIFI_REQ, TASK_DBG, DRV_TASK_ID,
                          1);
    if (!req)
        return -12;//ENOMEM;

    /* Send the message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, CUSTOM_MSG_SCAN_WIFI_CFM, NULL);
}

int rwnx_send_fhcustmsg_host_ota_req(struct rwnx_hw *rwnx_hw, uint32_t step, void *param, struct fhcustmsg_host_ota_cfm *cfm)
{
    struct fhcustmsg_host_ota_req *req;

    //RWNX_DBG(RWNX_FN_ENTRY_STR);

    ASSERT_ERR(step <= OTA_STEP_LZMA_CHECK);

    /* Build the message */
    req = rwnx_msg_zalloc(CUSTOM_MSG_HOST_OTA_REQ, TASK_DBG, DRV_TASK_ID,
                          sizeof(struct fhcustmsg_host_ota_req));
    if (!req)
        return -12;//ENOMEM;

    req->ota_step = step;
    if (OTA_STEP_FLASH_ERASE == step || OTA_STEP_HEADER_WRITE == step || OTA_STEP_LZMA_CHECK == step)
    {
        struct ota_file_info *info = (struct ota_file_info *) param;
        //req->ota_msg.ota_file_info.file_size = info->file_size;
        //req->ota_msg.ota_file_info.file_crc = info->file_crc;
        memcpy(&req->ota_msg.ota_file_info, info, sizeof(struct ota_file_info));
    }
    else
    {
        struct ota_pkg_info *pkg = (struct ota_pkg_info *) param;
        ASSERT_ERR(pkg->flash_size <= sizeof(req->ota_msg.ota_pkg_info.pkg_buf));
        req->ota_msg.ota_pkg_info.flash_addr = pkg->flash_addr;
        req->ota_msg.ota_pkg_info.flash_size = pkg->flash_size;
        rtos_memcpy(&req->ota_msg.ota_pkg_info.pkg_buf, pkg->pkg_buf, pkg->flash_size);
    }

    /* Send the message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, CUSTOM_MSG_HOST_OTA_CFM, cfm);
}

int rwnx_send_fhcustmsg_get_fw_version_req(struct rwnx_hw *rwnx_hw, struct fhcustmsg_get_fw_version_cfm *cfm)
{
    void *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the message */
    req = rwnx_msg_zalloc(CUSTOM_MSG_GET_FW_VERSION_REQ, TASK_DBG, DRV_TASK_ID,
                          1);
    if (!req)
        return -12;//ENOMEM;

    /* Send the message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, CUSTOM_MSG_GET_FW_VERSION_CFM, cfm);
}

int rwnx_send_fhcustmsg_reset_req(struct rwnx_hw *rwnx_hw)
{
    void *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the message */
    req = rwnx_msg_zalloc(CUSTOM_MSG_RESET_REQ, TASK_DBG, DRV_TASK_ID,
                          1);
    if (!req)
        return -12;//ENOMEM;

    /* Send the message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, CUSTOM_MSG_RESET_CFM, NULL);
}

int rwnx_send_fhcustmsg_sta_cfg_req(struct rwnx_hw *rwnx_hw, int op, char *ssid, int enable, struct fhcustmsg_sta_cfg_cfm *cfm)
{
    struct fhcustmsg_sta_cfg_req *req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the message */
    req = rwnx_msg_zalloc(CUSTOM_MSG_STA_CFG_REQ, TASK_DBG, DRV_TASK_ID,
                          sizeof(struct fhcustmsg_sta_cfg_req));
    if (!req)
        return -12;//ENOMEM;

    req->op = op;
    req->enable = enable;
    if (op == STA_CFG_REQ_OP_SET || op == STA_CFG_REQ_OP_DELETE)
    {
        if (!ssid || strlen(ssid) == 0 || strlen(ssid) >= AP_SSID_BUF_MAX)
        {
            rwnx_msg_free(rwnx_hw, req);
            return -1;
        }

        strcpy((char *)req->ssid, ssid);
    }

    /* Send the message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, CUSTOM_MSG_STA_CFG_CFM, cfm);
}

#if 0
int rwnx_send_fhcustmsg_http_req(struct rwnx_hw *rwnx_hw, char *uri)
{
    struct fhcustmsg_http_req *req;
    int uri_len;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    if (uri == NULL)
    {
        return -1;
    }

    uri_len = strlen(uri);
    if (uri_len >= HTTP_URI_BUF_MAX)
    {
        return -2;
    }

    /* Build the message */
    req = rwnx_msg_zalloc(CUSTOM_MSG_HTTP_REQ, TASK_DBG, DRV_TASK_ID,
                          sizeof(struct fhcustmsg_http_req));
    if (!req)
        return -12;//ENOMEM;

    /* Set parameters for the message */
    strncpy((char *)req->uri, uri, uri_len);
    req->uri[uri_len] = '\0';

    /* Send the message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, req, 1, CUSTOM_MSG_HTTP_CFM, NULL);
}
#endif

int rwnx_send_msg_tx(struct rwnx_hw *rwnx_hw, lmac_task_id_t dst_id, lmac_msg_id_t msg_id, uint16_t msg_len, void *msg, int reqcfm, lmac_msg_id_t reqid, void *cfm)
{
    int ret = 0;
    void *req;
    /* Build the ME_CHAN_CONFIG_REQ message */
    req = rwnx_msg_zalloc(msg_id, dst_id, DRV_TASK_ID, msg_len);
    if (!req)
        return -1;

    if (msg_len)
        memcpy(req, msg, msg_len);

    //AIC_LOG_PRINTF("%s msg_id %x\n", __FUNCTION__, msg_id);

    ret = rwnx_host_send_msg(rwnx_hw, req, reqcfm, reqid, cfm);
    if (ret < 0)
    {
        DBG_MACIF_INF("send msgID 0x%x err %d\n", msg_id, ret);
    }

    return ret;
}

int rwnx_send_rftest_req(struct rwnx_hw *rwnx_hw, u32_l cmd, u32_l argc, u8_l *argv, struct dbg_rftest_cmd_cfm *cfm)
{
    struct dbg_rftest_cmd_req *mem_rftest_cmd_req;

    RWNX_DBG(RWNX_FN_ENTRY_STR);

    /* Build the DBG_RFTEST_CMD_REQ message */
    mem_rftest_cmd_req = rwnx_msg_zalloc(DBG_RFTEST_CMD_REQ, TASK_DBG, DRV_TASK_ID,
                                         sizeof(struct dbg_rftest_cmd_req));
    if (!mem_rftest_cmd_req)
        return -12;//-ENOMEM;

    if (argc > 30)
        return -12;//-ENOMEM;

    /* Set parameters for the DBG_MEM_MASK_WRITE_REQ message */
    mem_rftest_cmd_req->cmd = cmd;
    mem_rftest_cmd_req->argc = argc;
    if (argc != 0)
        memcpy(mem_rftest_cmd_req->argv, argv, argc);

    /* Send the DBG_RFTEST_CMD_REQ message to LMAC FW */
    return rwnx_host_send_msg(rwnx_hw, mem_rftest_cmd_req, 1, DBG_RFTEST_CMD_CFM, cfm);
}

