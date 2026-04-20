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
#ifndef _RWNX_RX_H_
#define _RWNX_RX_H_

#include "aicwf_txrxif.h"
#include "aicwf_config.h"

#ifndef __packed
    #define __packed __PACKED
#endif /* __packed */

#ifdef CONFIG_RX_REORDER
    #define AICWF_RX_REORDER
#endif
#define AICWF_RWNX_TIMER_EN 1

/*
 * Decryption status subfields.
 */
#define RWNX_RX_HD_DECR_UNENC           0 // ENCRYPTION TYPE NONE
#define RWNX_RX_HD_DECR_WEP             1 // ENCRYPTION TYPE WEP
#define RWNX_RX_HD_DECR_TKIP            2 // ENCRYPTION TYPE TKIP
#define RWNX_RX_HD_DECR_CCMP128         3 // ENCRYPTION TYPE CCMP128
#define RWNX_RX_HD_DECR_CCMP256         4 // ENCRYPTION TYPE CCMP256
#define RWNX_RX_HD_DECR_GCMP128         5 // ENCRYPTION TYPE GCMP128
#define RWNX_RX_HD_DECR_GCMP256         6 // ENCRYPTION TYPE GCMP256
#define RWNX_RX_HD_DECR_WAPI            7 // ENCRYPTION TYPE WAPI

struct rx_vector_1_old
{
    /** Receive Vector 1a */
    u32    leg_length         : 12;
    u32    leg_rate           : 4;
    u32    ht_length          : 16;

    /** Receive Vector 1b */
    u32    _ht_length         : 4; // FIXME
    u32    short_gi           : 1;
    u32    stbc               : 2;
    u32    smoothing          : 1;
    u32    mcs                : 7;
    u32    pre_type           : 1;
    u32    format_mod         : 3;
    u32    ch_bw              : 2;
    u32    n_sts              : 3;
    u32    lsig_valid         : 1;
    u32    sounding           : 1;
    u32    num_extn_ss        : 2;
    u32    aggregation        : 1;
    u32    fec_coding         : 1;
    u32    dyn_bw             : 1;
    u32    doze_not_allowed   : 1;

    /** Receive Vector 1c */
    u32    antenna_set        : 8;
    u32    partial_aid        : 9;
    u32    group_id           : 6;
    u32    first_user         : 1;
    s32    rssi1              : 8;

    /** Receive Vector 1d */
    s32    rssi2              : 8;
    s32    rssi3              : 8;
    s32    rssi4              : 8;
    u32    reserved_1d        : 8;
};

struct rx_leg_vect
{
    u8    dyn_bw_in_non_ht     : 1;
    u8    chn_bw_in_non_ht     : 2;
    u8    rsvd_nht             : 4;
    u8    lsig_valid           : 1;
} __packed;

struct rx_ht_vect
{
    u16   sounding             : 1;
    u16   smoothing            : 1;
    u16   short_gi             : 1;
    u16   aggregation          : 1;
    u16   stbc                 : 1;
    u16   num_extn_ss          : 2;
    u16   lsig_valid           : 1;
    u16   mcs                  : 7;
    u16   fec                  : 1;
    u16   length               : 16;
} __packed;

struct rx_vht_vect
{
    u8   sounding              : 1;
    u8   beamformed            : 1;
    u8   short_gi              : 1;
    u8   rsvd_vht1             : 1;
    u8   stbc                  : 1;
    u8   doze_not_allowed      : 1;
    u8   first_user            : 1;
    u8   rsvd_vht2             : 1;
    u16  partial_aid           : 9;
    u16  group_id              : 6;
    u16  rsvd_vht3             : 1;
    u32  mcs                   : 4;
    u32  nss                   : 3;
    u32  fec                   : 1;
    u32  length                : 20;
    u32  rsvd_vht4             : 4;
} __packed;

struct rx_he_vect
{
    u8   sounding              : 1;
    u8   beamformed            : 1;
    u8   gi_type               : 2;
    u8   stbc                  : 1;
    u8   rsvd_he1              : 3;

    u8   uplink_flag           : 1;
    u8   beam_change           : 1;
    u8   dcm                   : 1;
    u8   he_ltf_type           : 2;
    u8   doppler               : 1;
    u8   rsvd_he2              : 2;

    u8   bss_color             : 6;
    u8   rsvd_he3              : 2;

    u8   txop_duration         : 7;
    u8   rsvd_he4              : 1;

    u8   pe_duration           : 4;
    u8   spatial_reuse         : 4;

    u8   sig_b_comp_mode       : 1;
    u8   dcm_sig_b             : 1;
    u8   mcs_sig_b             : 3;
    u8   ru_size               : 3;

    u32  mcs                   : 4;
    u32  nss                   : 3;
    u32  fec                   : 1;
    u32  length                : 20;
    u32  rsvd_he6              : 4;
} __packed;

struct rx_vector_1
{
    u8     format_mod         : 4;
    u8     ch_bw              : 3;
    u8     pre_type           : 1;
    u8     antenna_set        : 8;
    s32    rssi_leg           : 8;
    u32    leg_length         : 12;
    u32    leg_rate           : 4;
    s32    rssi1              : 8;

    union
    {
        struct rx_leg_vect leg;
        struct rx_ht_vect ht;
        struct rx_vht_vect vht;
        struct rx_he_vect he;
    };
} __packed;

struct rx_vector_2_old
{
    /** Receive Vector 2a */
    u32    rcpi               : 8;
    u32    evm1               : 8;
    u32    evm2               : 8;
    u32    evm3               : 8;

    /** Receive Vector 2b */
    u32    evm4               : 8;
    u32    reserved2b_1       : 8;
    u32    reserved2b_2       : 8;
    u32    reserved2b_3       : 8;

};

struct rx_vector_2
{
    /** Receive Vector 2a */
    u32    rcpi1              : 8;
    u32    rcpi2              : 8;
    u32    rcpi3              : 8;
    u32    rcpi4              : 8;

    /** Receive Vector 2b */
    u32    evm1               : 8;
    u32    evm2               : 8;
    u32    evm3               : 8;
    u32    evm4               : 8;
};

struct phy_channel_info_desc
{
    /** PHY channel information 1 */
    u32    phy_band           : 8;
    u32    phy_channel_type   : 8;
    u32    phy_prim20_freq    : 16;
    /** PHY channel information 2 */
    u32    phy_center1_freq   : 16;
    u32    phy_center2_freq   : 16;
};

struct hw_vect
{
    /** Total length for the MPDU transfer */
    u32 len                   : 16;

    u32 reserved              : 8;//data type is included
    /** AMPDU Status Information */
    u32 mpdu_cnt              : 6;
    u32 ampdu_cnt             : 2;

    /** TSF Low */
    u32 tsf_lo;
    /** TSF High */
    u32 tsf_hi;

    /** Receive Vector 1 */
    struct rx_vector_1 rx_vect1;
    /** Receive Vector 2 */
    struct rx_vector_2 rx_vect2;

    /** Status **/
    u32    rx_vect2_valid     : 1;
    u32    resp_frame         : 1;
    /** Decryption Status */
    u32    decr_status        : 3;
    u32    rx_fifo_oflow      : 1;

    /** Frame Unsuccessful */
    u32    undef_err          : 1;
    u32    phy_err            : 1;
    u32    fcs_err            : 1;
    u32    addr_mismatch      : 1;
    u32    ga_frame           : 1;
    u32    current_ac         : 2;

    u32    frm_successful_rx  : 1;
    /** Descriptor Done  */
    u32    desc_done_rx       : 1;
    /** Key Storage RAM Index */
    u32    key_sram_index     : 10;
    /** Key Storage RAM Index Valid */
    u32    key_sram_v         : 1;
    u32    type               : 2;
    u32    subtype            : 4;
};

struct hw_rxhdr
{
    /** RX vector */
    struct hw_vect hwvect;

    /** PHY channel information */
    struct phy_channel_info_desc phy_info;

    /** RX flags */
    u32    flags_is_amsdu     : 1;
    u32    flags_is_80211_mpdu: 1;
    u32    flags_is_4addr     : 1;
    u32    flags_new_peer     : 1;
#if 1//defined(AICWF_SDIO_SUPPORT) || defined(AICWF_USB_SUPPORT)
    u32    flags_user_prio    : 1; // aic: fw not fill any more
    u32    flags_need_reord   : 1;
    u32    flags_upload       : 1;
#else
    u32    flags_user_prio    : 3;
#endif
#ifndef AICWF_RX_REORDER
    u32    flags_rsvd0        : 1;
#else
    u32    is_monitor_vif     : 1;
#endif
    u32    flags_vif_idx      : 8;    // 0xFF if invalid VIF index
    u32    flags_sta_idx      : 8;    // 0xFF if invalid STA index
    u32    flags_dst_idx      : 8;    // 0xFF if unknown destination STA
    /** Pattern indicating if the buffer is available for the driver */
    u32    pattern;
};

struct rx_buf_tag
{
    /// Structure containing the information about the received payload
    struct hw_rxhdr hwrxhdr;
    /// Reserved field
    u16 reserved[2];
    /// Payload buffer space
    u32 payload[];
};

#ifdef AICWF_RX_REORDER
    struct recv_msdu *reord_rxframe_alloc(spinlock_t *lock, struct list_head *q);
    void reord_rxframe_free(spinlock_t *lock, struct list_head *q, struct list_head *list);
    struct reord_ctrl_info *reord_init_sta(struct aicwf_rx_priv *rx_priv, const u8 *mac_addr);
    void reord_deinit_sta(struct aicwf_rx_priv *rx_priv, struct reord_ctrl_info *reord_info);
    int reord_need_check(struct reord_ctrl *preorder_ctrl, u16 seq_num);
    int reord_rxframe_enqueue(struct reord_ctrl *preorder_ctrl, struct recv_msdu *prframe);
    void reord_timeout_worker(struct work_struct *work);
    int reord_single_frame_ind(struct aicwf_rx_priv *rx_priv, struct recv_msdu *prframe);
    #if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
        void reord_timeout_handler(ulong data);
    #else
        void reord_timeout_handler(struct timer_list *t);
    #endif

#endif

int aicwf_process_rxframes(struct aicwf_rx_priv *rx_priv);
#ifdef AICWF_RX_REORDER
    int rwnx_rxdataind_aicwf(struct rwnx_hw *rwnx_hw, void *hostid, void *rx_priv);
#else
    int rwnx_rxdata_forward(struct rwnx_hw *rwnx_hw, void *hostid, void *rx_priv);
#endif

#endif /* _RWNX_RX_H_ */
