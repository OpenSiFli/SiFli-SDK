/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file rwnx_msg_tx.h
 *
 * @brief TX function declarations
 *
 * Copyright (C) RivieraWaves 2012-2019
 *
 ****************************************************************************************
 */

#ifndef _RWNX_MSG_TX_H_
#define _RWNX_MSG_TX_H_

#include "rwnx_defs.h"
#include "lmac_msg.h"

/*
 * c.f LMAC/src/co/mac/mac_frame.h
 */

int rwnx_send_reset(struct rwnx_hw *rwnx_hw);
int rwnx_send_start(struct rwnx_hw *rwnx_hw, struct mm_start_req *start);
int rwnx_send_version_req(struct rwnx_hw *rwnx_hw, struct mm_version_cfm *cfm);
int rwnx_send_add_if(struct rwnx_hw *rwnx_hw, struct mm_add_if_req *req, struct mm_add_if_cfm *cfm);
int rwnx_send_remove_if(struct rwnx_hw *rwnx_hw, struct mm_remove_if_req *rem_req);
int rwnx_send_me_chan_config_req(struct rwnx_hw *rwnx_hw, struct me_chan_config_req *chan_cfg);
int rwnx_send_me_config_req(struct rwnx_hw *rwnx_hw, struct me_config_req *cfg_req);
int rwnx_send_me_sta_del(struct rwnx_hw *rwnx_hw, u8 sta_idx, bool tdls_sta);
int rwnx_set_hw_idle_req(struct rwnx_hw *rwnx_hw, uint8_t idle);
int rwnx_set_disable_agg_req(struct rwnx_hw *rwnx_hw, uint8_t agg_disable, uint8_t sta_idx);
int rwnx_set_coex_config_req(struct rwnx_hw *rwnx_hw, uint8_t disable_coexnull, uint8_t enable_periodic_timer, uint8_t enable_nullcts,
                                    uint8_t coex_timeslot_set, uint32_t param1, uint32_t param2);
int rwnx_send_rf_config_req(struct rwnx_hw *rwnx_hw, u8_l ofst, u8_l sel, u8_l *tbl, u16_l len);
int rwnx_send_rf_calib_req(struct rwnx_hw *rwnx_hw, struct mm_set_rf_calib_cfm *cfm);
int aicwf_set_rf_config_8800dc(struct rwnx_hw *rwnx_hw, struct mm_set_rf_calib_cfm *cfm);
int aicwf_set_rf_config_8800d80(struct rwnx_hw *rwnx_hw, struct mm_set_rf_calib_cfm *cfm);
int rwnx_send_get_macaddr_req(struct rwnx_hw *rwnx_hw, struct mm_get_mac_addr_cfm *cfm);
int rwnx_send_set_vendor_hwconfig_req(struct rwnx_hw *rwnx_hw, uint32_t hwconfig_id, int32_t *param, int32_t *param_out);
int rwnx_send_set_stack_start_req(struct rwnx_hw *rwnx_hw, u8_l on, u8_l efuse_valid, u8_l set_vendor_info,
                    u8_l fwtrace_redir_en, struct mm_set_stack_start_cfm *cfm);
int rwnx_send_txpwr_idx_req(struct rwnx_hw *rwnx_hw);
int rwnx_send_txpwr_ofst_req(struct rwnx_hw *rwnx_hw);
int rwnx_send_txpwr_ofst2x_req(struct rwnx_hw *rwnx_hw);
int rwnx_send_txpwr_lvl_req(struct rwnx_hw *rwnx_hw);
int aicwf_send_txpwr_lvl_req(txpwr_lvl_conf_v2_t *txpwrlvl);
int rwnx_send_txpwr_lvl_v3_req(struct rwnx_hw *rwnx_hw);
int rwnx_send_sm_connect_req(struct rwnx_hw *rwnx_hw,
                             struct sm_connect_req *conn_req,
                             struct sm_connect_cfm *cfm);
int rwnx_send_sm_disconnect_req(struct rwnx_hw *rwnx_hw,
                                struct sm_disconnect_req *disconn_req);
int rwnx_send_scanu_req(struct rwnx_hw *rwnx_hw, struct scanu_start_req *start_req);

/* Debug messages */
int rwnx_send_dbg_trigger_req(struct rwnx_hw *rwnx_hw, char *msg);
int rwnx_send_dbg_mem_read_req(struct rwnx_hw *rwnx_hw, u32 mem_addr,
                               struct dbg_mem_read_cfm *cfm);
int rwnx_send_dbg_mem_write_req(struct rwnx_hw *rwnx_hw, u32 mem_addr,
                                u32 mem_data);
int rwnx_send_dbg_mem_mask_write_req(struct rwnx_hw *rwnx_hw, u32 mem_addr,
                                     u32 mem_mask, u32 mem_data);
int rwnx_send_dbg_set_mod_filter_req(struct rwnx_hw *rwnx_hw, u32 filter);
int rwnx_send_rftest_req(struct rwnx_hw *rwnx_hw, u32_l cmd, u32_l argc, u8_l *argv, struct dbg_rftest_cmd_cfm *cfm);
#ifdef CONFIG_MCU_MESSAGE
int rwnx_send_dbg_custom_msg_req(struct rwnx_hw *rwnx_hw,
                                 u32 cmd, void *buf, u32 len, u32 action,
                                 struct dbg_custom_msg_cfm *cfm);
#endif
int rwnx_send_dbg_set_sev_filter_req(struct rwnx_hw *rwnx_hw, u32 filter);
int rwnx_send_dbg_get_sys_stat_req(struct rwnx_hw *rwnx_hw,
                                   struct dbg_get_sys_stat_cfm *cfm);
int rwnx_send_dbg_mem_block_write_req(struct rwnx_hw *rwnx_hw, u32 mem_addr,
                                      u32 mem_size, u32 *mem_data);
int rwnx_send_dbg_start_app_req(struct rwnx_hw *rwnx_hw, u32 boot_addr,
                                u32 boot_type);

int rwnx_send_msg_tx(struct rwnx_hw *rwnx_hw, lmac_task_id_t dst_id, lmac_msg_id_t msg_id, uint16_t msg_len, void *msg, int reqcfm, lmac_msg_id_t reqid, void *cfm);

#endif /* _RWNX_MSG_TX_H_ */