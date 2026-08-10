/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ******************************************************************************
 *
 * rwnx_cmds.c
 *
 * Handles queueing (push to IPC, ack/cfm from IPC) of commands issued to
 * LMAC FW
 *
 * Copyright (C) RivieraWaves 2014-2018
 *
 ******************************************************************************
 */
#include "rwnx_cmds.h"
#include "rwnx_defs.h"
#include "lmac_msg.h"
#include "hal_co.h"
#include "rtos_port.h"
#include "aic_plat_log.h"
#include <string.h>

static void cmd_complete(struct rwnx_cmd_mgr *cmd_mgr, struct rwnx_cmd *cmd)
{
    list_del(&cmd->list);
    cmd_mgr->queue_sz--;

    cmd->flags |= RWNX_CMD_FLAG_DONE;
    if (cmd->flags & RWNX_CMD_FLAG_NONBLOCK) {
        rtos_free(cmd);
    } else {
        if (RWNX_CMD_WAIT_COMPLETE(cmd->flags)) {
            cmd->result = 0;
            rtos_semaphore_signal(cmd->sema, 0);
        }
    }
}

static int cmd_mgr_queue(struct rwnx_cmd_mgr *cmd_mgr, struct rwnx_cmd *cmd)
{
    bool defer_push = false;

    rtos_mutex_lock(cmd_mgr->mutex, -1);
    if (!list_empty(&cmd_mgr->cmds)) {
        struct rwnx_cmd *last;
        if (cmd_mgr->queue_sz == cmd_mgr->max_queue_sz) {
            aic_dbg("Too many cmds (%d) already queued\n", cmd_mgr->max_queue_sz);
            cmd->result = -2;
            rtos_mutex_unlock(cmd_mgr->mutex);
            return -2;
        }
        last = list_entry(cmd_mgr->cmds.prev, struct rwnx_cmd, list);
        if (last->flags & (RWNX_CMD_FLAG_WAIT_ACK | RWNX_CMD_FLAG_WAIT_PUSH)) {
            cmd->flags |= RWNX_CMD_FLAG_WAIT_PUSH;
            defer_push = true;
        }
    }

    if (cmd->flags & RWNX_CMD_FLAG_REQ_CFM)
        cmd->flags |= RWNX_CMD_FLAG_WAIT_CFM;

    cmd->tkn    = cmd_mgr->next_tkn++;
    cmd->result = -3;

    list_add_tail(&cmd->list, &cmd_mgr->cmds);
    cmd_mgr->queue_sz++;

    //aic_dbg("%s %d defer_push %d\r\n", __func__, __LINE__, defer_push);

    if (!defer_push) {
        rwnx_msg_push(cmd->a2e_msg, sizeof(struct lmac_msg) + cmd->a2e_msg->param_len);
        rtos_free(cmd->a2e_msg);
        cmd->a2e_msg = NULL;
    }

    if (!(cmd->flags & RWNX_CMD_FLAG_NONBLOCK)) {
        int ret;
        rtos_mutex_unlock(cmd_mgr->mutex);
        ret = rtos_semaphore_wait(cmd->sema, 5000);
        if (ret < 0) {
            aic_dbg("cmd_mgr sema wait err\n");
            list_del(&cmd->list);
            cmd_mgr->queue_sz--;
        } else if (ret == 1) {
            aic_dbg("cmd_mgr sema wait timeout\n");
            list_del(&cmd->list);
            cmd_mgr->queue_sz--;
        }
        rtos_semaphore_delete(cmd->sema);
    } else {
        cmd->result = 0;
        rtos_mutex_unlock(cmd_mgr->mutex);
    }

    return 0;
}

static int cmd_mgr_llind(struct rwnx_cmd_mgr *cmd_mgr, struct rwnx_cmd *cmd)
{
    struct rwnx_cmd *cur, *acked = NULL, *next = NULL;

    rtos_mutex_lock(cmd_mgr->mutex, -1);
    list_for_each_entry(cur, &cmd_mgr->cmds, list) {
        if (!acked) {
            if (cur->tkn == cmd->tkn) {
                //if (WARN_ON_ONCE(cur != cmd)) {
                //if (cur != cmd) {
                //   cmd_dump(cmd);
                //}
                acked = cur;
                continue;
            }
        }
        if (cur->flags & RWNX_CMD_FLAG_WAIT_PUSH) {
                next = cur;
                break;
        }
    }
    if (!acked) {
        aic_dbg("Error: acked cmd not found\n");
    } else {
        cmd->flags &= ~RWNX_CMD_FLAG_WAIT_ACK;
        if (RWNX_CMD_WAIT_COMPLETE(cmd->flags))
            cmd_complete(cmd_mgr, cmd);
    }
    if (next) {
        //struct rwnx_hw *rwnx_hw = container_of(cmd_mgr, struct rwnx_hw, cmd_mgr);
        next->flags &= ~RWNX_CMD_FLAG_WAIT_PUSH;
        rwnx_msg_push(cmd->a2e_msg, sizeof(struct lmac_msg) + cmd->a2e_msg->param_len);
        rtos_free(next->a2e_msg);
    }
    rtos_mutex_unlock(cmd_mgr->mutex);
    return 0;
}

static int cmd_mgr_run_callback(struct rwnx_hw *rwnx_hw, struct rwnx_cmd *cmd,
                                struct rwnx_cmd_e2amsg *msg, msg_cb_fct cb)
{
    int res;

    if (! cb)
        return 0;
    res = cb(rwnx_hw, cmd, msg);

    return res;
}

static int cmd_mgr_msgind(struct rwnx_cmd_mgr *cmd_mgr, struct rwnx_cmd_e2amsg *msg,
                          msg_cb_fct cb)
{
    struct rwnx_hw *rwnx_hw = container_of(cmd_mgr, struct rwnx_hw, cmd_mgr);
    struct rwnx_cmd *cmd;
    bool found = false;

    //aic_dbg("msg->id=%x\n",msg->id);
    //aic_dbg("is_empty:%d, queue_sz=%d\n", list_empty(&cmd_mgr->cmds), cmd_mgr->queue_sz);
    rtos_mutex_lock(cmd_mgr->mutex, -1);
    list_for_each_entry(cmd, &cmd_mgr->cmds, list) {
        //aic_dbg("cmd->reqid=%x\n",cmd->reqid);
        if (cmd->reqid == msg->id &&
            (cmd->flags & RWNX_CMD_FLAG_WAIT_CFM)) {

            if (!cmd_mgr_run_callback(rwnx_hw, cmd, msg, cb)) {
                found = true;
                cmd->flags &= ~RWNX_CMD_FLAG_WAIT_CFM;

                if (msg->param_len > RWNX_CMD_E2AMSG_LEN_MAX) {
                    aic_dbg("Unexpect E2A msg len %d > %d\n", msg->param_len, RWNX_CMD_E2AMSG_LEN_MAX);
                    msg->param_len = RWNX_CMD_E2AMSG_LEN_MAX;
                }

                if (cmd->e2a_msg && msg->param_len)
                    memcpy(cmd->e2a_msg, &msg->param, msg->param_len);

                if (RWNX_CMD_WAIT_COMPLETE(cmd->flags)) {
                    cmd_complete(cmd_mgr, cmd);
                }

                break;
            }
        }
    }
    rtos_mutex_unlock(cmd_mgr->mutex);

    if (!found)
        cmd_mgr_run_callback(rwnx_hw, NULL, msg, cb);

    return 0;
}

void rwnx_cmd_mgr_init(struct rwnx_cmd_mgr *cmd_mgr)
{
    int ret;
    INIT_LIST_HEAD(&cmd_mgr->cmds);
    ret = rtos_mutex_create(&cmd_mgr->mutex, "cmd_mgr->mutex");
    if (ret) {
        aic_dbg("cmd mgr mutex create fail, ret=%d\n", ret);
        return;
    }
    cmd_mgr->max_queue_sz = RWNX_CMD_MAX_QUEUED;
    cmd_mgr->queue_sz     = 0;
    cmd_mgr->queue  = &cmd_mgr_queue;
    //cmd_mgr->print  = &cmd_mgr_print;
    //cmd_mgr->drain  = &cmd_mgr_drain;
    cmd_mgr->llind  = &cmd_mgr_llind;
    cmd_mgr->msgind = &cmd_mgr_msgind;
}

void rwnx_cmd_mgr_deinit(struct rwnx_cmd_mgr *cmd_mgr)
{
// if config onekey remove,ignore it
#ifndef	CONFIG_DRIVER_ORM
    //cmd_mgr->print(cmd_mgr);
    //cmd_mgr->drain(cmd_mgr);
    //cmd_mgr->print(cmd_mgr);
    rtos_mutex_delete(cmd_mgr->mutex);
#endif
    memset(cmd_mgr, 0, sizeof(*cmd_mgr));
}

int rwnx_msg_push(struct lmac_msg *msg, uint16_t len)
{
    uint8_t ret;
    uint16_t index = 0;
    uint8_t *buffer_unaligned = rtos_malloc(CMD_BUF_MAX + CMD_BUF_ALIGN_SIZE);
    uint8_t *buffer;

    //AIC_LOG_PRINTF("%s, enter\n", __func__);
    DBG_MACIF_VRB("TxM,%x\n", msg->id);

    if (buffer_unaligned == NULL) {
        aic_dbg("Error: cmd buf malloc fail\n");
        return -1;
    }

    if (CMD_BUF_ALIGN_SIZE > 0) {
        if ((size_t)buffer_unaligned & (CMD_BUF_ALIGN_SIZE - 1)) {
            buffer = buffer_unaligned + CMD_BUF_ALIGN_SIZE - ((size_t)buffer_unaligned & (CMD_BUF_ALIGN_SIZE - 1));
        } else {
            buffer = buffer_unaligned;
        }
    } else {
        buffer = buffer_unaligned;
    }

    memset(buffer, 0, CMD_BUF_MAX);

    ret = aicwf_hal_msg_push(buffer, msg, len);
    //AIC_LOG_PRINTF("%s, ret %d\n", __func__, ret);

    rtos_free(buffer_unaligned);

    return ret;
}

