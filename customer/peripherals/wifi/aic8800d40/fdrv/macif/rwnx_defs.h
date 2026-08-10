/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _RWNX_DEFS_H_
#define _RWNX_DEFS_H_

#include "rwnx_cmds.h"
#include "rtos_port.h"
#include "lmac_mac.h"
#include "rwnx_config.h"

enum survey_info_flags {
    SURVEY_INFO_NOISE_DBM       = BIT(0),
    SURVEY_INFO_IN_USE          = BIT(1),
    SURVEY_INFO_TIME            = BIT(2),
    SURVEY_INFO_TIME_BUSY       = BIT(3),
    SURVEY_INFO_TIME_EXT_BUSY   = BIT(4),
    SURVEY_INFO_TIME_RX         = BIT(5),
    SURVEY_INFO_TIME_TX         = BIT(6),
    SURVEY_INFO_TIME_SCAN       = BIT(7),
};


/* Structure containing channel survey information received from MAC */
struct rwnx_survey_info {
    // Filled
    u32 filled;
    // Amount of time in ms the radio spent on the channel
    u32 chan_time_ms;
    // Amount of time the primary channel was sensed busy
    u32 chan_time_busy_ms;
    // Noise in dbm
    s8 noise_dbm;
};

/* Structure that will contains all RoC information received from cfg80211 */
struct rwnx_roc_elem {
    struct mac_chan_def chan;
    unsigned int duration;
    /* Used to avoid call of CFG80211 callback upon expiration of RoC */
    bool mgmt_roc;
    /* Indicate if we have switch on the RoC channel */
    bool on_chan;
};

enum nl80211_band {
    NL80211_BAND_2GHZ,   /* 2.4 GHz ISM band */
    NL80211_BAND_5GHZ,   /* around 5 GHz band (4.9 - 5.7 GHz) */
    NL80211_BAND_60GHZ,  /* around 60 GHz band (58.32 - 64.80 GHz) */
};

enum nl80211_chan_width {
    NL80211_CHAN_WIDTH_20_NOHT,
    NL80211_CHAN_WIDTH_20,
    NL80211_CHAN_WIDTH_40,
    NL80211_CHAN_WIDTH_80,
    NL80211_CHAN_WIDTH_80P80,
    NL80211_CHAN_WIDTH_160,
};

struct cfg80211_chan_def {
    struct ieee80211_channel *chan;
    enum nl80211_chan_width width;
    u32 center_freq1;
    u32 center_freq2;
};

/* Structure containing channel context information */
struct rwnx_chanctx {
    struct cfg80211_chan_def chan_def; /* channel description */
    u8 count;                          /* number of vif using this ctxt */
};

struct rwnx_sec_phy_chan {
    u16 prim20_freq;
    u16 center_freq1;
    u16 center_freq2;
    enum nl80211_band band;
    u8 type;
};

/**
 * rwnx_phy_info - Phy information
 *
 * @phy_cnt: Number of phy interface
 * @cfg: Configuration send to firmware
 * @sec_chan: Channel configuration of the second phy interface (if phy_cnt > 1)
 * @limit_bw: Set to true to limit BW on requested channel. Only set to use
 * VHT with old radio that don't support 80MHz (deprecated)
 */
struct rwnx_phy_info {
    u8 cnt;
    struct phy_cfg_tag cfg;
    struct rwnx_sec_phy_chan sec_chan;
    bool limit_bw;
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

struct rwnx_hw {
    int mode;
    int net_id;
    #if NX_P2P
    /* RoC Management */
    struct rwnx_roc_elem *roc_elem;             /* Information provided by cfg80211 in its remain on channel request */
    uint32_t roc_cookie_cnt;                         /* Counter used to identify RoC request sent by cfg80211 */
    #endif
    struct rwnx_cmd_mgr cmd_mgr;
    struct mm_version_cfm version_cfm;          /* Lower layers versions - obtained via MM_VERSION_REQ */
    struct rwnx_survey_info survey[SCAN_CHANNEL_MAX];

    rtos_semaphore cli_cmd_sema;
    rtos_task_handle cli_cmd_task_hdl;
    u16 chipid;
    u8 fw_patch;    // fw: 0    patch: 1
};

struct rwnx_term_stream
{
    char *read, *write;
    int avail, last_idx;
};

int rwnx_host_send_msg(struct rwnx_hw *rwnx_hw, const void *msg_params,
                         int reqcfm, lmac_msg_id_t reqid, void *cfm);

#endif