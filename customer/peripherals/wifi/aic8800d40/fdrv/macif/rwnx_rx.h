/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ******************************************************************************
 *
 * @file rwnx_rx.h
 *
 * Copyright (C) RivieraWaves 2012-2019
 *
 ******************************************************************************
 */
#ifndef _RWNX_RX_H_
#define _RWNX_RX_H_

#include "fhost_api.h"

#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
    #ifndef __packed
        #define __packed __attribute__((__packed__))
    #endif /* __packed */
#endif /* __GNUC__ */

enum rx_status_bits
{
    /// The buffer can be forwarded to the networking stack
    RX_STAT_FORWARD = 1 << 0,
    /// A new buffer has to be allocated
    RX_STAT_ALLOC = 1 << 1,
    /// The buffer has to be deleted
    RX_STAT_DELETE = 1 << 2,
    /// The length of the buffer has to be updated
    RX_STAT_LEN_UPDATE = 1 << 3,
    /// The length in the Ethernet header has to be updated
    RX_STAT_ETH_LEN_UPDATE = 1 << 4,
    /// Simple copy
    RX_STAT_COPY = 1 << 5,
    /// Spurious frame (inform upper layer and discard)
    RX_STAT_SPURIOUS = 1 << 6,
    /// packet for monitor interface
    RX_STAT_MONITOR = 1 << 7,
};

/* MACIF RX Filter */
#define enDuplicateDetection     (1 << 31)
#define      acceptUnknown       (1 << 30)
#define acceptOtherDataFrames    (1 << 29)
#define      acceptQoSNull       (1 << 28)
#define    acceptQCFWOData       (1 << 27)
#define        acceptQData       (1 << 26)
#define     acceptCFWOData       (1 << 25)
#define         acceptData       (1 << 24)
#define acceptOtherCntrlFrames   (1 << 23)
#define        acceptCFEnd       (1 << 22)
#define          acceptACK       (1 << 21)
#define          acceptCTS       (1 << 20)
#define          acceptRTS       (1 << 19)
#define       acceptPSPoll       (1 << 18)
#define           acceptBA       (1 << 17)
#define          acceptBAR       (1 << 16)
#define acceptOtherMgmtFrames    (1 << 15)
#define  acceptBfmeeFrames       (1 << 14)
#define    acceptAllBeacon       (1 << 13)
#define acceptNotExpectedBA      (1 << 12)
#define acceptDecryptErrorFrames (1 << 11)
#define       acceptBeacon       (1 << 10)
#define    acceptProbeResp       (1 <<  9)
#define     acceptProbeReq       (1 <<  8)
#define    acceptMyUnicast       (1 <<  7)
#define      acceptUnicast       (1 <<  6)
#define  acceptErrorFrames       (1 <<  5)
#define   acceptOtherBSSID       (1 <<  4)
#define    acceptBroadcast       (1 <<  3)
#define    acceptMulticast       (1 <<  2)
#define        dontDecrypt       (1 <<  1)
#define     excUnencrypted       (1 <<  0)


#define IEEE80211_RADIOTAP_VERSION 0


#define IEEE80211_RADIOTAP_TSFT            0
#define IEEE80211_RADIOTAP_FLAGS           1
#define IEEE80211_RADIOTAP_RATE            2
#define IEEE80211_RADIOTAP_CHANNEL         3
#define IEEE80211_RADIOTAP_FHSS            4
#define IEEE80211_RADIOTAP_DBM_ANTSIGNAL   5
#define IEEE80211_RADIOTAP_DBM_ANTNOISE    6
#define IEEE80211_RADIOTAP_LOCK_QUALITY    7
#define IEEE80211_RADIOTAP_TX_ATTENUATION  8
#define IEEE80211_RADIOTAP_DB_TX_ATTENUATION 9
#define IEEE80211_RADIOTAP_DBM_TX_POWER    10
#define IEEE80211_RADIOTAP_ANTENNA         11
#define IEEE80211_RADIOTAP_DB_ANTSIGNAL    12
#define IEEE80211_RADIOTAP_DB_ANTNOISE     13
#define IEEE80211_RADIOTAP_RX_FLAGS        14
#define IEEE80211_RADIOTAP_TX_FLAGS        15
#define IEEE80211_RADIOTAP_RTS_RETRIES     16
#define IEEE80211_RADIOTAP_DATA_RETRIES    17
#define IEEE80211_RADIOTAP_MCS             18
#define IEEE80211_RADIOTAP_AMPDU_STATUS    19
#define IEEE80211_RADIOTAP_VHT             20
#define IEEE80211_RADIOTAP_TIMESTAMP       21
#define IEEE80211_RADIOTAP_HE              22
#define IEEE80211_RADIOTAP_HE_MU           23
#define IEEE80211_RADIOTAP_ZERO_LEN_PSDU   24
#define IEEE80211_RADIOTAP_LSIG            25

#define IEEE80211_RADIOTAP_RADIOTAP_NAMESPACE 29


#define IEEE80211_RADIOTAP_EXT             31


#define IEEE80211_RADIOTAP_F_CFP           0x01
#define IEEE80211_RADIOTAP_F_SHORTPRE      0x02
#define IEEE80211_RADIOTAP_F_WEP           0x04
#define IEEE80211_RADIOTAP_F_FRAG          0x08
#define IEEE80211_RADIOTAP_F_FCS           0x10
#define IEEE80211_RADIOTAP_F_DATAPAD       0x20
#define IEEE80211_RADIOTAP_F_BADFCS        0x40


#define IEEE80211_RADIOTAP_VENDOR_NAMESPACE 0x03


#define IEEE80211_RADIOTAP_ALIGN(len) (((len) + 3) & ~3)

/*
 * Decryption status subfields.
 * {
 */
#define RWNX_RX_HD_DECR_UNENC           0 // Frame unencrypted
#define RWNX_RX_HD_DECR_ICVFAIL         1 // WEP/TKIP ICV failure
#define RWNX_RX_HD_DECR_CCMPFAIL        2 // CCMP failure
#define RWNX_RX_HD_DECR_AMSDUDISCARD    3 // A-MSDU discarded at HW
#define RWNX_RX_HD_DECR_NULLKEY         4 // NULL key found
#define RWNX_RX_HD_DECR_WEPSUCCESS      5 // Security type WEP
#define RWNX_RX_HD_DECR_TKIPSUCCESS     6 // Security type TKIP
#define RWNX_RX_HD_DECR_CCMPSUCCESS     7 // Security type CCMP (or WPI)
// @}

/* Values for formatModTx */
#define FORMATMOD_NON_HT          0
#define FORMATMOD_NON_HT_DUP_OFDM 1
#define FORMATMOD_HT_MF           2
#define FORMATMOD_HT_GF           3
#define FORMATMOD_VHT             4
#define FORMATMOD_HE_SU           5
#define FORMATMOD_HE_MU           6
#define FORMATMOD_HE_ER           7


/// Structure containing information about the received frame (length, timestamp, rate, etc.)
struct rx_vector
{
    /// Total length of the received MPDU
    uint16_t            frmlen;
    /// AMPDU status information
    uint16_t            ampdu_stat_info;
    /// TSF Low
    uint32_t            tsflo;
    /// TSF High
    uint32_t            tsfhi;
    /// Contains the bytes 4 - 1 of Receive Vector 1
    uint32_t            recvec1a;
    /// Contains the bytes 8 - 5 of Receive Vector 1
    uint32_t            recvec1b;
    /// Contains the bytes 12 - 9 of Receive Vector 1
    uint32_t            recvec1c;
    /// Contains the bytes 16 - 13 of Receive Vector 1
    uint32_t            recvec1d;
    /// Contains the bytes 4 - 1 of Receive Vector 2
    uint32_t            recvec2a;
    ///  Contains the bytes 8 - 5 of Receive Vector 2
    uint32_t            recvec2b;
    /// MPDU status information
    uint32_t            statinfo;
};

/// Structure containing the information about the PHY channel that was used for this RX
struct phy_channel_info
{
    /// PHY channel information 1
    uint32_t info1;
    /// PHY channel information 2
    uint32_t info2;
};

/// Structure containing the information about the received payload
struct rx_info
{
    /// Rx header descriptor (this element MUST be the first of the structure)
    struct rx_vector vect;
    /// Structure containing the information about the PHY channel that was used for this RX
    struct phy_channel_info phy_info;
    /// Word containing some SW flags about the RX packet
    uint32_t flags;
#if NX_AMSDU_DEAGG
    /// Array of host buffer identifiers for the other A-MSDU subframes
    uint32_t amsdu_hostids[NX_MAX_MSDU_PER_RX_AMSDU - 1];
#endif
    /// Spare room for LMAC FW to write a pattern when last DMA is sent
    uint32_t pattern;

    uint16_t reserved[2];
};

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

/**
 * struct rwnx_rx_rate_stats - Store statistics for RX rates
 *
 * @table: Table indicating how many frame has been receive which each
 * rate index. Rate index is the same as the one used by RC algo for TX
 * @size: Size of the table array
 * @cpt: number of frames received
 * @rate_cnt: number of rate for which at least one frame has been received
 */
struct rwnx_rx_rate_stats
{
    int *table;
    int size;
    int cpt;
    int rate_cnt;
};

struct rwnx_legrate
{
    int idx;
    int rate;
};

struct ieee80211_radiotap_header
{
    u8 it_version;
    u8 it_pad;
    u16 it_len;
    u32 it_present;
};

struct ieee80211_radiotap_he
{
    __le16 data1, data2, data3, data4, data5, data6;
};

enum ieee80211_radiotap_he_bits
{
    IEEE80211_RADIOTAP_HE_DATA1_FORMAT_MASK      = 3,
    IEEE80211_RADIOTAP_HE_DATA1_FORMAT_SU        = 0,
    IEEE80211_RADIOTAP_HE_DATA1_FORMAT_EXT_SU    = 1,
    IEEE80211_RADIOTAP_HE_DATA1_FORMAT_MU        = 2,
    IEEE80211_RADIOTAP_HE_DATA1_FORMAT_TRIG      = 3,

    IEEE80211_RADIOTAP_HE_DATA1_BSS_COLOR_KNOWN     = 0x0004,
    IEEE80211_RADIOTAP_HE_DATA1_BEAM_CHANGE_KNOWN   = 0x0008,
    IEEE80211_RADIOTAP_HE_DATA1_UL_DL_KNOWN         = 0x0010,
    IEEE80211_RADIOTAP_HE_DATA1_DATA_MCS_KNOWN      = 0x0020,
    IEEE80211_RADIOTAP_HE_DATA1_DATA_DCM_KNOWN      = 0x0040,
    IEEE80211_RADIOTAP_HE_DATA1_CODING_KNOWN        = 0x0080,
    IEEE80211_RADIOTAP_HE_DATA1_LDPC_XSYMSEG_KNOWN  = 0x0100,
    IEEE80211_RADIOTAP_HE_DATA1_STBC_KNOWN          = 0x0200,
    IEEE80211_RADIOTAP_HE_DATA1_SPTL_REUSE_KNOWN    = 0x0400,
    IEEE80211_RADIOTAP_HE_DATA1_SPTL_REUSE2_KNOWN   = 0x0800,
    IEEE80211_RADIOTAP_HE_DATA1_SPTL_REUSE3_KNOWN   = 0x1000,
    IEEE80211_RADIOTAP_HE_DATA1_SPTL_REUSE4_KNOWN   = 0x2000,
    IEEE80211_RADIOTAP_HE_DATA1_BW_RU_ALLOC_KNOWN   = 0x4000,
    IEEE80211_RADIOTAP_HE_DATA1_DOPPLER_KNOWN       = 0x8000,

    IEEE80211_RADIOTAP_HE_DATA2_PRISEC_80_KNOWN     = 0x0001,
    IEEE80211_RADIOTAP_HE_DATA2_GI_KNOWN            = 0x0002,
    IEEE80211_RADIOTAP_HE_DATA2_NUM_LTF_SYMS_KNOWN  = 0x0004,
    IEEE80211_RADIOTAP_HE_DATA2_PRE_FEC_PAD_KNOWN   = 0x0008,
    IEEE80211_RADIOTAP_HE_DATA2_TXBF_KNOWN          = 0x0010,
    IEEE80211_RADIOTAP_HE_DATA2_PE_DISAMBIG_KNOWN   = 0x0020,
    IEEE80211_RADIOTAP_HE_DATA2_TXOP_KNOWN          = 0x0040,
    IEEE80211_RADIOTAP_HE_DATA2_MIDAMBLE_KNOWN      = 0x0080,
    IEEE80211_RADIOTAP_HE_DATA2_RU_OFFSET           = 0x3f00,
    IEEE80211_RADIOTAP_HE_DATA2_RU_OFFSET_KNOWN     = 0x4000,
    IEEE80211_RADIOTAP_HE_DATA2_PRISEC_80_SEC       = 0x8000,

    IEEE80211_RADIOTAP_HE_DATA3_BSS_COLOR           = 0x003f,
    IEEE80211_RADIOTAP_HE_DATA3_BEAM_CHANGE         = 0x0040,
    IEEE80211_RADIOTAP_HE_DATA3_UL_DL               = 0x0080,
    IEEE80211_RADIOTAP_HE_DATA3_DATA_MCS            = 0x0f00,
    IEEE80211_RADIOTAP_HE_DATA3_DATA_DCM            = 0x1000,
    IEEE80211_RADIOTAP_HE_DATA3_CODING              = 0x2000,
    IEEE80211_RADIOTAP_HE_DATA3_LDPC_XSYMSEG        = 0x4000,
    IEEE80211_RADIOTAP_HE_DATA3_STBC                = 0x8000,

    IEEE80211_RADIOTAP_HE_DATA4_SU_MU_SPTL_REUSE    = 0x000f,
    IEEE80211_RADIOTAP_HE_DATA4_MU_STA_ID           = 0x7ff0,
    IEEE80211_RADIOTAP_HE_DATA4_TB_SPTL_REUSE1      = 0x000f,
    IEEE80211_RADIOTAP_HE_DATA4_TB_SPTL_REUSE2      = 0x00f0,
    IEEE80211_RADIOTAP_HE_DATA4_TB_SPTL_REUSE3      = 0x0f00,
    IEEE80211_RADIOTAP_HE_DATA4_TB_SPTL_REUSE4      = 0xf000,

    IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC    = 0x000f,
    IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_20MHZ  = 0,
    IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_40MHZ  = 1,
    IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_80MHZ  = 2,
    IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_160MHZ = 3,
    IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_26T    = 4,
    IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_52T    = 5,
    IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_106T   = 6,
    IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_242T   = 7,
    IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_484T   = 8,
    IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_996T   = 9,
    IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_2x996T = 10,

    IEEE80211_RADIOTAP_HE_DATA5_GI                  = 0x0030,
    IEEE80211_RADIOTAP_HE_DATA5_GI_0_8      = 0,
    IEEE80211_RADIOTAP_HE_DATA5_GI_1_6      = 1,
    IEEE80211_RADIOTAP_HE_DATA5_GI_3_2      = 2,

    IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE            = 0x00c0,
    IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE_UNKNOWN    = 0,
    IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE_1X         = 1,
    IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE_2X         = 2,
    IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE_4X         = 3,
    IEEE80211_RADIOTAP_HE_DATA5_NUM_LTF_SYMS        = 0x0700,
    IEEE80211_RADIOTAP_HE_DATA5_PRE_FEC_PAD         = 0x3000,
    IEEE80211_RADIOTAP_HE_DATA5_TXBF                = 0x4000,
    IEEE80211_RADIOTAP_HE_DATA5_PE_DISAMBIG         = 0x8000,

    IEEE80211_RADIOTAP_HE_DATA6_NSTS                = 0x000f,
    IEEE80211_RADIOTAP_HE_DATA6_DOPPLER             = 0x0010,
    IEEE80211_RADIOTAP_HE_DATA6_TXOP                = 0x7f00,
    IEEE80211_RADIOTAP_HE_DATA6_MIDAMBLE_PDCTY      = 0x8000,
};

struct ieee80211_radiotap_he_mu
{
    __le16 flags1, flags2;
    u8 ru_ch1[4];
    u8 ru_ch2[4];
};

enum ieee80211_radiotap_he_mu_bits
{
    IEEE80211_RADIOTAP_HE_MU_FLAGS1_SIG_B_MCS               = 0x000f,
    IEEE80211_RADIOTAP_HE_MU_FLAGS1_SIG_B_MCS_KNOWN         = 0x0010,
    IEEE80211_RADIOTAP_HE_MU_FLAGS1_SIG_B_DCM               = 0x0020,
    IEEE80211_RADIOTAP_HE_MU_FLAGS1_SIG_B_DCM_KNOWN         = 0x0040,
    IEEE80211_RADIOTAP_HE_MU_FLAGS1_CH2_CTR_26T_RU_KNOWN    = 0x0080,
    IEEE80211_RADIOTAP_HE_MU_FLAGS1_CH1_RU_KNOWN            = 0x0100,
    IEEE80211_RADIOTAP_HE_MU_FLAGS1_CH2_RU_KNOWN            = 0x0200,
    IEEE80211_RADIOTAP_HE_MU_FLAGS1_CH1_CTR_26T_RU_KNOWN    = 0x1000,
    IEEE80211_RADIOTAP_HE_MU_FLAGS1_CH1_CTR_26T_RU          = 0x2000,
    IEEE80211_RADIOTAP_HE_MU_FLAGS1_SIG_B_COMP_KNOWN        = 0x4000,
    IEEE80211_RADIOTAP_HE_MU_FLAGS1_SIG_B_SYMS_USERS_KNOWN  = 0x8000,

    IEEE80211_RADIOTAP_HE_MU_FLAGS2_BW_FROM_SIG_A_BW        = 0x0003,
    IEEE80211_RADIOTAP_HE_MU_FLAGS2_BW_FROM_SIG_A_BW_20MHZ  = 0x0000,
    IEEE80211_RADIOTAP_HE_MU_FLAGS2_BW_FROM_SIG_A_BW_40MHZ  = 0x0001,
    IEEE80211_RADIOTAP_HE_MU_FLAGS2_BW_FROM_SIG_A_BW_80MHZ  = 0x0002,
    IEEE80211_RADIOTAP_HE_MU_FLAGS2_BW_FROM_SIG_A_BW_160MHZ = 0x0003,
    IEEE80211_RADIOTAP_HE_MU_FLAGS2_BW_FROM_SIG_A_BW_KNOWN  = 0x0004,
    IEEE80211_RADIOTAP_HE_MU_FLAGS2_SIG_B_COMP              = 0x0008,
    IEEE80211_RADIOTAP_HE_MU_FLAGS2_SIG_B_SYMS_USERS        = 0x00f0,
    IEEE80211_RADIOTAP_HE_MU_FLAGS2_PUNC_FROM_SIG_A_BW      = 0x0300,
    IEEE80211_RADIOTAP_HE_MU_FLAGS2_PUNC_FROM_SIG_A_BW_KNOWN = 0x0400,
    IEEE80211_RADIOTAP_HE_MU_FLAGS2_CH2_CTR_26T_RU          = 0x0800,
};

void rwnx_rx_monitor_cb(struct fhost_frame_info *info, void *arg);

#endif /* _RWNX_RX_H_ */