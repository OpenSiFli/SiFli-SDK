/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rwnx_defs.h"
#include "rwnx_cmds.h"
#include "lmac_msg.h"
#include "aic_plat_log.h"
#include <string.h>

static inline bool is_non_blocking_msg(int id)
{
    return ((id == MM_TIM_UPDATE_REQ) || (id == ME_RC_SET_RATE_REQ) ||
            (id == MM_BFMER_ENABLE_REQ) || (id == ME_TRAFFIC_IND_REQ) ||
            (id == TDLS_PEER_TRAFFIC_IND_REQ) ||
            (id == SM_EXTERNAL_AUTH_REQUIRED_RSP));
}


int rwnx_host_send_msg(struct rwnx_hw *rwnx_hw, const void *msg_params,
                         int reqcfm, lmac_msg_id_t reqid, void *cfm)
{
    struct lmac_msg *msg;
    struct rwnx_cmd *cmd;
    bool nonblock;
    int ret;

    msg = container_of((void *)msg_params, struct lmac_msg, param);

#if 0
    if (reqid != MM_RESET_CFM && reqid != MM_VERSION_CFM &&
        reqid != MM_START_CFM && reqid != MM_SET_IDLE_CFM &&
        reqid != ME_CONFIG_CFM && reqid != MM_SET_PS_MODE_CFM &&
        reqid != ME_CHAN_CONFIG_CFM) {
        aic_dbg("%s: bypassing (RWNX_DEV_RESTARTING set) 0x%02x\n",
               __func__, reqid);
        return -1;
    }
#endif

    nonblock = is_non_blocking_msg(msg->id);
    cmd = rtos_malloc(sizeof(struct rwnx_cmd));
    if (!cmd)
        return -1;
    memset(cmd, 0, sizeof(struct rwnx_cmd));
    cmd->result  = -1;
    cmd->id      = msg->id;
    cmd->reqid   = reqid;
    cmd->a2e_msg = msg;
    cmd->e2a_msg = cfm;
    if (nonblock)
        cmd->flags = RWNX_CMD_FLAG_NONBLOCK;
    if (reqcfm)
        cmd->flags |= RWNX_CMD_FLAG_REQ_CFM;

    if (!nonblock) {
        if (rtos_semaphore_create(&cmd->sema, "cmd->sema", 1, 0)){
			rtos_free(cmd);
            return -1;
        }
    }
    ret = rwnx_hw->cmd_mgr.queue(&rwnx_hw->cmd_mgr, cmd);


    if (!ret) {
        if (cmd->result) {
            aic_dbg("cmd res:%d\n",cmd->result);
        }
        ret = cmd->result;
    }

    if (!nonblock)
        rtos_free(cmd);
    return ret;
}

