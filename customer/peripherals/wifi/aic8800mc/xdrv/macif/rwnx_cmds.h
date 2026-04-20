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

#ifndef _RWNX_CMDS_H_
#define _RWNX_CMDS_H_

#include <stddef.h>

#include "lmac_msg.h"
#include "aic_rwnx_main.h"
#include "aic_list.h"
#include "co_int.h"
#include "co_bool.h"
#include "compiler.h"
#include "co_list.h"
#include "osal_service.h"
#include "osal_debug.h"


#define RWNX_CMD_FLAG_NONBLOCK      CO_BIT(0)
#define RWNX_CMD_FLAG_REQ_CFM       CO_BIT(1)
#define RWNX_CMD_FLAG_WAIT_PUSH     CO_BIT(2)
#define RWNX_CMD_FLAG_WAIT_ACK      CO_BIT(3)
#define RWNX_CMD_FLAG_WAIT_CFM      CO_BIT(4)
#define RWNX_CMD_FLAG_DONE          CO_BIT(5)
/* ATM IPC design makes it possible to get the CFM before the ACK,
 * otherwise this could have simply been a state enum */
#define RWNX_CMD_WAIT_COMPLETE(flags) \
    (!(flags & (RWNX_CMD_FLAG_WAIT_ACK | RWNX_CMD_FLAG_WAIT_CFM)))

#define RWNX_CMD_MAX_QUEUED         8

/// Message structure for MSGs from Emb to App
struct e2a_msg
{
    uint16_t id;                ///< Message id.
    uint16_t dummy_dest_id;     ///< Not used
    uint16_t dummy_src_id;      ///< Not used
    uint16_t param_len;         ///< Parameter embedded struct length.

    uint32_t pattern;           ///< Used to stamp a valid MSG buffer
    uint32_t param[256];  ///< Parameter embedded struct. Must be word-aligned.
};


#define rwnx_cmd_e2amsg e2a_msg
#define rwnx_cmd_a2emsg lmac_msg
#define RWNX_CMD_A2EMSG_LEN(m) (sizeof(struct lmac_msg) + m->param_len)
#define RWNX_CMD_E2AMSG_LEN_MAX  (256 * 4)


struct rwnx_hw;
struct rwnx_cmd;
typedef int (*msg_cb_fct)(struct rwnx_hw *rwnx_hw, struct rwnx_cmd *cmd,
                          struct rwnx_cmd_e2amsg *msg);

static inline void put_u16(u8 *buf, u16 data)
{
    buf[0] = (u8)(data & 0x00ff);
    buf[1] = (u8)((data >> 8) & 0x00ff);
}

#ifdef PLATFORM_UNISOC_THREADX
    #define CMD_BUF_ALIGN_SIZE 64
#else
    #define CMD_BUF_ALIGN_SIZE 0
#endif

enum rwnx_cmd_mgr_state
{
    RWNX_CMD_MGR_STATE_DEINIT,
    RWNX_CMD_MGR_STATE_INITED,
    RWNX_CMD_MGR_STATE_CRASHED,
};

typedef struct rwnx_cmd
{
    struct list_head list;
    lmac_msg_id_t id;
    lmac_msg_id_t reqid;
    struct rwnx_cmd_a2emsg *a2e_msg;
    char *e2a_msg;
    uint32_t tkn;
    uint16_t flags;
    rtos_semaphore sema;

    //struct completion complete;
    uint32_t result;
#ifdef CONFIG_RWNX_FHOST
    struct rwnx_term_stream *stream;
#endif
} rwnx_cmd;

struct rwnx_cmd_mgr
{
    enum rwnx_cmd_mgr_state state;
    //spinlock_t lock;
    rtos_mutex mutex;
    uint32_t next_tkn;
    uint32_t queue_sz;
    uint32_t max_queue_sz;

    struct list_head cmds;

    int (*queue)(struct rwnx_cmd_mgr *, struct rwnx_cmd *);
    int (*llind)(struct rwnx_cmd_mgr *, struct rwnx_cmd *);
    int (*msgind)(struct rwnx_cmd_mgr *, struct rwnx_cmd_e2amsg *, msg_cb_fct);
    void (*print)(struct rwnx_cmd_mgr *);
    void (*drain)(struct rwnx_cmd_mgr *);
};

void rwnx_cmd_mgr_init(struct rwnx_cmd_mgr *cmd_mgr);
void rwnx_cmd_mgr_deinit(struct rwnx_cmd_mgr *cmd_mgr);

#endif /* _RWNX_CMDS_H_ */
