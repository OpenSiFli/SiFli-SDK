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

#include "rwnx_cmds.h"
#include "rwnx_defs.h"
#include "lmac_msg.h"
#include "aic_rwnx_main.h"
#include "aicwf_txrxif.h"
#include "osal_service.h"
#include "osal_debug.h"
//#define CREATE_TRACE_POINTS
#include <string.h>

#define KERN_CRIT   "[KERN]"

#define lockdep_assert_held(...)
#define spin_lock_bh(...)
#define spin_unlock_bh(...)
#define spin_lock_init(...)
#define spin_lock(...)
#define spin_unlock(...)
#define trace_msg_recv(...)
#define trace_msg_send(...)


#define WARN(con, ...)  (con)

#define RWNX_ID2STR(tag) "Cmd"

extern struct rwnx_hw *g_rwnx_hw;

int aicwf_sdio_fault_handler(struct aic_sdio_dev *sdiodev);

static void cmd_complete(struct rwnx_cmd_mgr *cmd_mgr, struct rwnx_cmd *cmd)
{
    lockdep_assert_held(&cmd_mgr->lock);

//    list_del(&cmd->list);
//    cmd_mgr->queue_sz--;

#if 0
    DBG_MACIF_ERR("cmd_complete, %d\r\n", cmd_mgr->queue_sz);
    struct rwnx_cmd *cur;

    for (cur = list_entry((cmd_mgr->cmds.next), rwnx_cmd, list);
            (&cur->list != &cmd_mgr->cmds);
            cur = list_entry(cur->list.next, rwnx_cmd, list))
    {
        //cnt++;
        //if(cnt > cmd_mgr->queue_sz)
        //    return;
        //DBG_MACIF_ERR("cmd_complete List head %08x, next %08x\r\n", &cmd_mgr->cmds, cmd_mgr->cmds.next);

        DBG_MACIF_ERR("cmd_complete cur %08x, next %08x\n", cur, list_entry(cur->list.next, rwnx_cmd, list));

        //DBG_MACIF_ERR("cmd_complete &cmd_mgr->cmds %08x, &cur->list %08x\r\n", &cmd_mgr->cmds, &cur->list);

    }
#endif
    cmd->flags |= RWNX_CMD_FLAG_DONE;
    if (cmd->flags & RWNX_CMD_FLAG_NONBLOCK)
    {
        rtos_free(cmd);
    }
    else
    {
        if (RWNX_CMD_WAIT_COMPLETE(cmd->flags))
        {
            cmd->result = 0;
            rtos_semaphore_signal(cmd->sema, 0);
        }
    }
}

/**
 *
 */
static int cmd_mgr_queue(struct rwnx_cmd_mgr *cmd_mgr, struct rwnx_cmd *cmd)
{
    bool defer_push = false;
    struct rwnx_hw *p_rwnx_hw = container_of(cmd_mgr, struct rwnx_hw, cmd_mgr);

    rtos_mutex_lock(cmd_mgr->mutex, -1);
#if 0
    if (cmd_mgr->state == RWNX_CMD_MGR_STATE_CRASHED)
    {
        DBG_MACIF_ERR("cmd queue crashed\n");
        cmd->result = -1;
        rtos_mutex_unlock(cmd_mgr->mutex);
        return -1;
    }
#endif
    if (!list_empty(&cmd_mgr->cmds))
    {
        struct rwnx_cmd *last;
        if (cmd_mgr->queue_sz == cmd_mgr->max_queue_sz)
        {
            DBG_MACIF_ERR("Too many cmds (%d) already queued\n", cmd_mgr->max_queue_sz);
            cmd->result = -2;
            rtos_mutex_unlock(cmd_mgr->mutex);
            return -2;
        }
        last = list_entry(cmd_mgr->cmds.prev, struct rwnx_cmd, list);
        if (last->flags & (RWNX_CMD_FLAG_WAIT_ACK | RWNX_CMD_FLAG_WAIT_PUSH))
        {
#if 0 // queue even NONBLOCK command.
            if (cmd->flags & RWNX_CMD_FLAG_NONBLOCK)
            {
                printk(KERN_CRIT"cmd queue busy\n");
                cmd->result = -EBUSY;
                spin_unlock_bh(&cmd_mgr->lock);
                return -EBUSY;
            }
#endif
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

    //DBG_MACIF_ERR("%s %d defer_push %d\r\n", __func__, __LINE__, defer_push);

    if (!defer_push)
    {
        //rwnx_ipc_msg_push(rwnx_hw, cmd, sizeof(struct lmac_msg) + cmd->a2e_msg->param_len);
        aicwf_bus_msg_tx(p_rwnx_hw->bus_if, cmd->a2e_msg, sizeof(struct lmac_msg) + cmd->a2e_msg->param_len);
        rtos_free(cmd->a2e_msg);
        cmd->a2e_msg = NULL;
    }

    if (!(cmd->flags & RWNX_CMD_FLAG_NONBLOCK))
    {
        int ret;
        rtos_mutex_unlock(cmd_mgr->mutex);
        ret = rtos_semaphore_wait(cmd->sema, CMD_TX_TIMEOUT);
        if (ret < 0)
        {
            DBG_MACIF_ERR("cmd_mgr sema wait err, ret=%d\n", ret);
            if (cmd->reqid != DBG_MEM_READ_CFM)
                aicwf_sdio_fault_handler(p_rwnx_hw->sdiodev);
//            list_del(&cmd->list);
//            cmd_mgr->queue_sz--;
        }
        else if (ret == 1)
        {
            DBG_MACIF_ERR("cmd_mgr sema wait timeout\n");
//            list_del(&cmd->list);
//            cmd_mgr->queue_sz--;
        }
        rtos_mutex_lock(cmd_mgr->mutex, -1);
        list_del(&cmd->list);
        cmd_mgr->queue_sz--;
        rtos_mutex_unlock(cmd_mgr->mutex);
        rtos_semaphore_delete(cmd->sema);
    }
    else
    {
        cmd->result = 0;
        rtos_mutex_unlock(cmd_mgr->mutex);
    }

    return 0;
}

#if 0
/**
 *
 */
static int cmd_mgr_llind(struct rwnx_cmd_mgr *cmd_mgr, struct rwnx_cmd *cmd)
{
    struct rwnx_cmd *cur, *acked = NULL, *next = NULL;

    rtos_mutex_lock(cmd_mgr->mutex, -1);
    list_for_each_entry(cur, &cmd_mgr->cmds, list)
    {
        if (!acked)
        {
            if (cur->tkn == cmd->tkn)
            {
                //if (WARN_ON_ONCE(cur != cmd)) {
                //if (cur != cmd) {
                //   cmd_dump(cmd);
                //}
                acked = cur;
                continue;
            }
        }
        if (cur->flags & RWNX_CMD_FLAG_WAIT_PUSH)
        {
            next = cur;
            break;
        }
    }
    if (!acked)
    {
        DBG_MACIF_ERR("Error: acked cmd not found\n");
    }
    else
    {
        cmd->flags &= ~RWNX_CMD_FLAG_WAIT_ACK;
        if (RWNX_CMD_WAIT_COMPLETE(cmd->flags))
            cmd_complete(cmd_mgr, cmd);
    }
    if (next)
    {
        //struct rwnx_hw *rwnx_hw = container_of(cmd_mgr, struct rwnx_hw, cmd_mgr);
        next->flags &= ~RWNX_CMD_FLAG_WAIT_PUSH;
        //rwnx_ipc_msg_push(rwnx_hw, next, RWNX_CMD_A2EMSG_LEN(next->a2e_msg));
        aicwf_bus_msg_tx(cmd->a2e_msg, sizeof(struct lmac_msg) + cmd->a2e_msg->param_len);
        rtos_free(next->a2e_msg);
    }
    rtos_mutex_unlock(cmd_mgr->mutex);
    return 0;
}
#endif

static int cmd_mgr_run_callback(struct rwnx_hw *rwnx_hw, struct rwnx_cmd *cmd,
                                struct rwnx_cmd_e2amsg *msg, msg_cb_fct cb)
{
    int res;

    if (! cb)
        return 0;
    spin_lock(&rwnx_hw->cb_lock);
    res = cb(rwnx_hw, cmd, msg);
    spin_unlock(&rwnx_hw->cb_lock);

    return res;
}

static int cmd_mgr_msgind(struct rwnx_cmd_mgr *cmd_mgr, struct rwnx_cmd_e2amsg *msg,
                          msg_cb_fct cb)
{
    struct rwnx_hw *rwnx_hw = container_of(cmd_mgr, struct rwnx_hw, cmd_mgr);
    struct rwnx_cmd *cmd;
    bool found = false;

#if 0
    for (cur = list_entry((cmd_mgr->cmds.next), rwnx_cmd, list);
            (&cur->list != &cmd_mgr->cmds);
            cur = list_entry(cur->list.next, rwnx_cmd, list))
    {
        //cnt++;
        //if(cnt > cmd_mgr->queue_sz)
        //    return;
        //DBG_MACIF_ERR("List head %08x, next %08x\r\n", &cmd_mgr->cmds, cmd_mgr->cmds.next);

        DBG_MACIF_ERR(" cur %08x, next %08x\n", cur, list_entry(cur->list.next, rwnx_cmd, list));

        //DBG_MACIF_ERR(" &cmd_mgr->cmds %08x, &cur->list %08x\r\n", &cmd_mgr->cmds, &cur->list);

    }
#endif
    DBG_MACIF_VRB("msg->id=%x\n", msg->id);
    //DBG_MACIF_ERR("is_empty:%d, queue_sz=%d\n", list_empty(&cmd_mgr->cmds), cmd_mgr->queue_sz);
    rtos_mutex_lock(cmd_mgr->mutex, -1);
    list_for_each_entry(cmd, &cmd_mgr->cmds, list)
    {
        //DBG_MACIF_ERR("cmd->reqid=%x\n",cmd->reqid);
        if (cmd->reqid == msg->id &&
                (cmd->flags & RWNX_CMD_FLAG_WAIT_CFM))
        {

            if (!cmd_mgr_run_callback(rwnx_hw, cmd, msg, cb))
            {
                found = true;
                cmd->flags &= ~RWNX_CMD_FLAG_WAIT_CFM;

                if (msg->param_len > RWNX_CMD_E2AMSG_LEN_MAX)
                {
                    DBG_MACIF_ERR("Unexpect E2A msg len %d > %d\n", msg->param_len, RWNX_CMD_E2AMSG_LEN_MAX);
                    msg->param_len = RWNX_CMD_E2AMSG_LEN_MAX;
                }

                if (cmd->e2a_msg && msg->param_len)
                    memcpy(cmd->e2a_msg, &msg->param, msg->param_len);

                if (RWNX_CMD_WAIT_COMPLETE(cmd->flags))
                {
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

#if 0
static void cmd_dump(const struct rwnx_cmd *cmd)
{
    DBG_MACIF_ERR(KERN_CRIT "tkn[%d]  flags:%04x  result:%3d  cmd:%4d-%-24s - reqcfm(%4d-%-s)\n",
                  cmd->tkn, cmd->flags, cmd->result, cmd->id, RWNX_ID2STR(cmd->id),
                  cmd->reqid, cmd->reqid != (lmac_msg_id_t) -1 ? RWNX_ID2STR(cmd->reqid) : "none");
}

/**
 *
 */
static void cmd_mgr_print(struct rwnx_cmd_mgr *cmd_mgr)
{
    struct rwnx_cmd *cur;

    spin_lock_bh(&cmd_mgr->lock);
    list_for_each_entry(cur, &cmd_mgr->cmds, list)
    {
        cmd_dump(cur);
    }
    spin_unlock_bh(&cmd_mgr->lock);
}

/**
 *
 */
static void cmd_mgr_drain(struct rwnx_cmd_mgr *cmd_mgr)
{
#if 0
    struct rwnx_cmd *cur, *nxt;
    spin_lock_bh(&cmd_mgr->lock);
    list_for_each_entry_safe(cur, nxt, &cmd_mgr->cmds, list)
    {
        list_del(&cur->list);
        cmd_mgr->queue_sz--;
        //if (!(cur->flags & RWNX_CMD_FLAG_NONBLOCK))
        //    complete(&cur->complete);
    }
    spin_unlock_bh(&cmd_mgr->lock);
#endif
}
#endif

/**
 *
 */
void rwnx_cmd_mgr_init(struct rwnx_cmd_mgr *cmd_mgr)
{
    int ret;
    INIT_LIST_HEAD(&cmd_mgr->cmds);
    ret = rtos_mutex_create(&cmd_mgr->mutex, "cmd_mgr->mutex");
    if (ret)
    {
        DBG_MACIF_ERR("cmd mgr mutex create fail, ret=%d\n", ret);
        return;
    }
    cmd_mgr->max_queue_sz = RWNX_CMD_MAX_QUEUED;
    cmd_mgr->queue_sz     = 0;
    cmd_mgr->queue  = &cmd_mgr_queue;
    //cmd_mgr->print  = &cmd_mgr_print;
    //cmd_mgr->drain  = &cmd_mgr_drain;
    //cmd_mgr->llind  = &cmd_mgr_llind;
    cmd_mgr->msgind = &cmd_mgr_msgind;
}

/**
 *
 */
void rwnx_cmd_mgr_deinit(struct rwnx_cmd_mgr *cmd_mgr)
{
    //cmd_mgr->print(cmd_mgr);
    //cmd_mgr->drain(cmd_mgr);
    //cmd_mgr->print(cmd_mgr);
    rtos_mutex_delete(cmd_mgr->mutex);
    memset(cmd_mgr, 0, sizeof(*cmd_mgr));
}
