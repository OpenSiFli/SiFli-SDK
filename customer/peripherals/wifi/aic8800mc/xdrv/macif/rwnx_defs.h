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

#ifndef _RWNX_DEFS_H_
#define _RWNX_DEFS_H_

#include "rwnx_cmds.h"
#include "lmac_mac.h"
#include "aic_rwnx_main.h"

#include "aicwf_sdio.h"
#include "osal_service.h"
#include "osal_debug.h"

#if 1//def CONFIG_RWNX_DBG
    #define RWNX_DBG(fmt, ...) DBG_MACIF_INF(fmt, ## __VA_ARGS__)
#else
    #define RWNX_DBG(a...) do {} while (0)
#endif

#define RWNX_FN_ENTRY_STR ">>> %s()\n", __func__

/**
 * rwnx_container_of - return the member address of ptr, if the type of ptr is the
 * struct type.
 */
#define rwnx_container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - (unsigned long)(&((type *)0)->member)))

enum survey_info_flags
{
    SURVEY_INFO_NOISE_DBM       = CO_BIT(0),
    SURVEY_INFO_IN_USE          = CO_BIT(1),
    SURVEY_INFO_TIME            = CO_BIT(2),
    SURVEY_INFO_TIME_BUSY       = CO_BIT(3),
    SURVEY_INFO_TIME_EXT_BUSY   = CO_BIT(4),
    SURVEY_INFO_TIME_RX         = CO_BIT(5),
    SURVEY_INFO_TIME_TX         = CO_BIT(6),
    SURVEY_INFO_TIME_SCAN       = CO_BIT(7),
};


/* Structure containing channel survey information received from MAC */
struct rwnx_survey_info
{
    // Filled
    u32 filled;
    // Amount of time in ms the radio spent on the channel
    u32 chan_time_ms;
    // Amount of time the primary channel was sensed busy
    u32 chan_time_busy_ms;
    // Noise in dbm
    s8 noise_dbm;
};

//#define ipc_host_env_tag ipc_fhost_host_env_tag;
/**
 * struct rwnx_hw - Main driver data
 *
 * @dev: device structure
 * @plat: Platform info
 * @task: Tasklet used to IRQ bottom half
 * @class: Class used to create device
 * @major: Major number for the driver
 * @term: Term device structure
 * @trace: Trace device structure
 * @cb_lock: Lock used to run message callback
 * @ipc_env: Pointer to IPC shared struture
 * @e2amsgs_pool: Pool of DMA buffer for messages push by FW
 * @dbgmsgs_pool: Pool of DMA buffer for dbg messages push by FW
 * @dbgdump_elem: DMA buffer for FW to upload debug dump (TODO)
 * @cmd_mgr: Command (aka fw message) structure
 * @mod_params: Module parameters used to initialize configuration buffer
 * @config: Configuration buffer passed to FW before start
 * @tv: Time at which we started the firmware
 * @debugfs: debugfs entries configuration
 */

struct rwnx_hw
{
    struct rwnx_plat *plat;
    struct rwnx_cmd_mgr cmd_mgr;
    struct aicwf_bus *bus_if;
    struct aic_sdio_dev *sdiodev;
    int mode;
    int net_id;
    //struct rwnx_mod_params *mod_params;
    //uint32_t config[IPC_CFG_SIZE];
    //struct mm_version_cfm version_cfm;          /* Lower layers versions - obtained via MM_VERSION_REQ */
    struct rwnx_survey_info survey[SCAN_CHANNEL_MAX];

    //rtos_semaphore cli_cmd_sema;
    //rtos_task_handle cli_cmd_task_hdl;
    u16 chipid;
    u8 fw_patch;    // fw: 0    patch: 1
};

struct rwnx_term_stream
{
    //struct list_head list;
    //struct rwnx_term *term;
    //char buf[2 * TERM_BUFFER_SIZE];
    char *read, *write;
    int avail, last_idx;
};

int rwnx_ipc_init(struct rwnx_hw *rwnx_hw, void *shared_ram);
void rwnx_ipc_start(struct rwnx_hw *rwnx_hw);
void rwnx_ipc_msg_push(struct rwnx_hw *rwnx_hw, void *msg_buf, uint16_t len);
int rwnx_host_send_msg(struct rwnx_hw *rwnx_hw, const void *msg_params,
                       int reqcfm, lmac_msg_id_t reqid, void *cfm);
int rwnx_ipc_txdesc_push(struct rwnx_hw *rwnx_hw, void *tx_desc,
                         void *hostid, int hw_queue);
int rwnx_ipc_rxbuf_elem_alloc(struct rwnx_hw *rwnx_hw);
void wifi_patch_prepare(void);

#endif
