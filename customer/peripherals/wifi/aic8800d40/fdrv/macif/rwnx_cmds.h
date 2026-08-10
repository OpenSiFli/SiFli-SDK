/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ******************************************************************************
 *
 * rwnx_cmds.h
 *
 * Copyright (C) RivieraWaves 2014-2018
 *
 ******************************************************************************
 */

#ifndef _RWNX_CMDS_H_
#define _RWNX_CMDS_H_

#include "lmac_msg.h"
#include "aic8800_list.h"
#include "dbg_assert.h"
#include "compiler.h"
#include "co_list.h"
#include "rtos_port.h"
#include "plat_port.h"
#include "aic_plat_types.h"
#include <stddef.h>

#define RWNX_CMD_FLAG_NONBLOCK      BIT(0)
#define RWNX_CMD_FLAG_REQ_CFM       BIT(1)
#define RWNX_CMD_FLAG_WAIT_PUSH     BIT(2)
#define RWNX_CMD_FLAG_WAIT_ACK      BIT(3)
#define RWNX_CMD_FLAG_WAIT_CFM      BIT(4)
#define RWNX_CMD_FLAG_DONE          BIT(5)
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
	buf[0] = (u8)(data&0x00ff);
	buf[1] = (u8)((data >> 8)&0x00ff);
}

#define CMD_BUF_MAX 1536
#define CMD_BUF_ALIGN_SIZE PLATFORM_CACHE_LINE_SIZE

enum rwnx_cmd_mgr_state {
    RWNX_CMD_MGR_STATE_DEINIT,
    RWNX_CMD_MGR_STATE_INITED,
    RWNX_CMD_MGR_STATE_CRASHED,
};

typedef struct rwnx_cmd {
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

struct rwnx_cmd_mgr {
    enum rwnx_cmd_mgr_state state;
    //spinlock_t lock;
    rtos_mutex mutex;
    uint32_t next_tkn;
    uint32_t queue_sz;
    uint32_t max_queue_sz;

    struct list_head cmds;

    int  (*queue)(struct rwnx_cmd_mgr *, struct rwnx_cmd *);
    int  (*llind)(struct rwnx_cmd_mgr *, struct rwnx_cmd *);
    int  (*msgind)(struct rwnx_cmd_mgr *, struct rwnx_cmd_e2amsg *, msg_cb_fct);
    void (*print)(struct rwnx_cmd_mgr *);
    void (*drain)(struct rwnx_cmd_mgr *);
};

int rwnx_msg_push(struct lmac_msg *msg, uint16_t len);
void rwnx_cmd_mgr_init(struct rwnx_cmd_mgr *cmd_mgr);
void rwnx_cmd_mgr_deinit(struct rwnx_cmd_mgr *cmd_mgr);

#endif /* _RWNX_CMDS_H_ */
