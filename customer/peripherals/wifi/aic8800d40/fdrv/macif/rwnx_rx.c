/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ******************************************************************************
 *
 * @file rwnx_rx.c
 *
 * Copyright (C) RivieraWaves 2012-2019
 *
 ******************************************************************************
 */

#include "rwnx_rx.h"
#include "mac_frame.h"
#include "rtos_port.h"
#include "aic_plat_log.h"
#include "build_config.h"

#ifdef CONFIG_MONITOR_RADIO_HEADER
/**
 * rwnx_rx_vector_convert - Convert a legacy RX vector into a new RX vector format
 *
 * @rwnx_hw: main driver data.
 * @rx_vect1: Rx vector 1 descriptor of the received frame.
 * @rx_vect2: Rx vector 2 descriptor of the received frame.
 */
static void rwnx_rx_vector_convert(struct rwnx_hw *rwnx_hw,
                                   struct rx_vector_1 *rx_vect1,
                                   struct rx_vector_2 *rx_vect2)
{
    struct rx_vector_1_old rx_vect1_leg;
    struct rx_vector_2_old rx_vect2_leg;
    // u32_l phy_vers = rwnx_hw->version_cfm.version_phy_2;

    // Check if we need to do the conversion. Only if old modem is used
    // if (__MDM_MAJOR_VERSION(phy_vers) > 0) {
    //     rx_vect1->rssi1 = rx_vect1->rssi_leg;
    //     return;
    // }

    // Copy the received vector locally
    memcpy(&rx_vect1_leg, rx_vect1, sizeof(struct rx_vector_1_old));

    // Reset it
    memset(rx_vect1, 0, sizeof(struct rx_vector_1));

    // Perform the conversion
    rx_vect1->format_mod = rx_vect1_leg.format_mod;
    rx_vect1->ch_bw = rx_vect1_leg.ch_bw;
    rx_vect1->antenna_set = rx_vect1_leg.antenna_set;
    rx_vect1->leg_length = rx_vect1_leg.leg_length;
    rx_vect1->leg_rate = rx_vect1_leg.leg_rate;
    rx_vect1->rssi1 = rx_vect1_leg.rssi1;

    switch (rx_vect1->format_mod) {
    case FORMATMOD_NON_HT:
    case FORMATMOD_NON_HT_DUP_OFDM:
        rx_vect1->leg.lsig_valid = rx_vect1_leg.lsig_valid;
        rx_vect1->leg.chn_bw_in_non_ht = rx_vect1_leg.num_extn_ss;
        rx_vect1->leg.dyn_bw_in_non_ht = rx_vect1_leg.dyn_bw;
        break;
    case FORMATMOD_HT_MF:
    case FORMATMOD_HT_GF:
        rx_vect1->ht.aggregation = rx_vect1_leg.aggregation;
        rx_vect1->ht.fec = rx_vect1_leg.fec_coding;
        rx_vect1->ht.lsig_valid = rx_vect1_leg.lsig_valid;
        rx_vect1->ht.length = rx_vect1_leg.ht_length;
        rx_vect1->ht.mcs = rx_vect1_leg.mcs;
        rx_vect1->ht.num_extn_ss = rx_vect1_leg.num_extn_ss;
        rx_vect1->ht.short_gi = rx_vect1_leg.short_gi;
        rx_vect1->ht.smoothing = rx_vect1_leg.smoothing;
        rx_vect1->ht.sounding = rx_vect1_leg.sounding;
        rx_vect1->ht.stbc = rx_vect1_leg.stbc;
        break;
    case FORMATMOD_VHT:
        rx_vect1->vht.beamformed = !rx_vect1_leg.smoothing;
        rx_vect1->vht.fec = rx_vect1_leg.fec_coding;
        rx_vect1->vht.length = rx_vect1_leg.ht_length | rx_vect1_leg._ht_length << 8;
        rx_vect1->vht.mcs = rx_vect1_leg.mcs & 0x0F;
        rx_vect1->vht.nss = rx_vect1_leg.stbc ? rx_vect1_leg.n_sts/2 : rx_vect1_leg.n_sts;
        rx_vect1->vht.doze_not_allowed = rx_vect1_leg.doze_not_allowed;
        rx_vect1->vht.short_gi = rx_vect1_leg.short_gi;
        rx_vect1->vht.sounding = rx_vect1_leg.sounding;
        rx_vect1->vht.stbc = rx_vect1_leg.stbc;
        rx_vect1->vht.group_id = rx_vect1_leg.group_id;
        rx_vect1->vht.partial_aid = rx_vect1_leg.partial_aid;
        rx_vect1->vht.first_user = rx_vect1_leg.first_user;
        break;
    }

    if (!rx_vect2)
        return;

    // Copy the received vector 2 locally
    memcpy(&rx_vect2_leg, rx_vect2, sizeof(struct rx_vector_2_old));

    // Reset it
    memset(rx_vect2, 0, sizeof(struct rx_vector_2));

    rx_vect2->rcpi1 = rx_vect2_leg.rcpi;
    rx_vect2->rcpi2 = rx_vect2_leg.rcpi;
    rx_vect2->rcpi3 = rx_vect2_leg.rcpi;
    rx_vect2->rcpi4 = rx_vect2_leg.rcpi;

    rx_vect2->evm1 = rx_vect2_leg.evm1;
    rx_vect2->evm2 = rx_vect2_leg.evm2;
    rx_vect2->evm3 = rx_vect2_leg.evm3;
    rx_vect2->evm4 = rx_vect2_leg.evm4;
}

static inline unsigned int generic_hweight32(unsigned int w) {
    unsigned int res = (w & 0x55555555) + ((w >> 1) & 0x55555555);
    res = (res & 0x33333333) + ((res >> 2) & 0x33333333);
    res = (res & 0x0F0F0F0F) + ((res >> 4) & 0x0F0F0F0F);
    res = (res & 0x00FF00FF) + ((res >> 8) & 0x00FF00FF);
    return (res & 0x0000FFFF) + ((res >> 16) & 0x0000FFFF);
}

unsigned char hweight8(unsigned char n) {
    n = (n & 0x55) + ((n >> 1) & 0x55);
    n = (n & 0x33) + ((n >> 2) & 0x33);
    n = (n & 0x0F) + (n >> 4);
    return n;
}
#define ALIGN(x, align) (((x) + (align) - 1) & ~((align) - 1))

const u16 tx_legrates_lut_rate[] = {
    10,
    20,
    55,
    110,
    60,
    90,
    120,
    180,
    240,
    360,
    480,
    540
};

const struct rwnx_legrate legrates_lut[] = {
    [0] = { .idx = 0, .rate = 10},
    [1] = { .idx = 1, .rate = 20},
    [2] = { .idx = 2, .rate = 55},
    [3] = { .idx = 3, .rate = 110},
    [4] = { .idx = -1, .rate = 0},
    [5] = { .idx = -1, .rate = 0},
    [6] = { .idx = -1, .rate = 0},
    [7] = { .idx = -1, .rate = 0},
    [8] = { .idx = 10, .rate = 480},
    [9] = { .idx = 8, .rate = 240},
    [10] = { .idx = 6, .rate = 120},
    [11] = { .idx = 4, .rate = 60},
    [12] = { .idx = 11, .rate = 540},
    [13] = { .idx = 9, .rate = 360},
    [14] = { .idx = 7, .rate = 180},
    [15] = { .idx = 5, .rate = 90},
};

/**
 * rwnx_rx_rtap_hdrlen - Return radiotap header length
 *
 * @rxvect: Rx vector used to fill the radiotap header
 * @has_vend_rtap: boolean indicating if vendor specific data is present
 *
 * Compute the length of the radiotap header based on @rxvect and vendor
 * specific data (if any).
 */
static u8 rwnx_rx_rtap_hdrlen(struct rx_vector_1 *rxvect,
                              bool has_vend_rtap)
{
    u8 rtap_len;

    /* Compute radiotap header length */
    rtap_len = sizeof(struct ieee80211_radiotap_header) + 8;

    DBG_MACIF_VRB("fmt=%x,bw=%x,pre=%x,ant=%x,r=%d,r1=%d\n",
        rxvect->format_mod, rxvect->ch_bw, rxvect->pre_type, rxvect->antenna_set,rxvect->rssi_leg,rxvect->rssi1);
    // Check for multiple antennas
    if (generic_hweight32(rxvect->antenna_set) > 1)
        // antenna and antenna signal fields
        rtap_len += 4 * hweight8(rxvect->antenna_set);

    // TSFT
    if (!has_vend_rtap) {
        rtap_len = ALIGN(rtap_len, 8);
        rtap_len += 8;
    }

    // IEEE80211_HW_SIGNAL_DBM
    rtap_len++;

    // Check if single antenna
    if (generic_hweight32(rxvect->antenna_set) == 1)
        rtap_len++; //Single antenna

    // padding for RX FLAGS
    rtap_len = ALIGN(rtap_len, 2);

    // aic_dbg("rtap_len %x\r\n", rtap_len);
    // aic_dbg("format %d\r\n", rxvect->format_mod);
    // Check for HT frames
    if ((rxvect->format_mod == FORMATMOD_HT_MF) ||
        (rxvect->format_mod == FORMATMOD_HT_GF))
        rtap_len += 3;

    // Check for AMPDU
    if (!(has_vend_rtap) && ((rxvect->format_mod >= FORMATMOD_VHT) ||
                             ((rxvect->format_mod > FORMATMOD_NON_HT_DUP_OFDM) &&
                                                     (rxvect->ht.aggregation)))) {
        rtap_len = ALIGN(rtap_len, 4);
        rtap_len += 8;
    }

    // Check for VHT frames
    if (rxvect->format_mod == FORMATMOD_VHT) {
        rtap_len = ALIGN(rtap_len, 2);
        rtap_len += 12;
    }

    // Check for HE frames
    if (rxvect->format_mod == FORMATMOD_HE_SU) {
        rtap_len = ALIGN(rtap_len, 2);
        rtap_len += sizeof(struct ieee80211_radiotap_he);
    }

    // Check for multiple antennas
    if (generic_hweight32(rxvect->antenna_set) > 1) {
        // antenna and antenna signal fields
        rtap_len += 2 * hweight8(rxvect->antenna_set);
    }

    // Check for vendor specific data
    if (has_vend_rtap) {
        /* vendor presence bitmap */
        rtap_len += 4;
        /* alignment for fixed 6-byte vendor data header */
        rtap_len = ALIGN(rtap_len, 2);
    }

    return rtap_len;
}

static void put_unaligned_le16(unsigned short val, unsigned char *p)
{
    *p++ = val;
    *p++ = val >> 8;
}

static void put_unaligned_le32(unsigned int val, void *p)
{
    unsigned char *ptr = (unsigned char *)p;

    ptr[0] = (val & 0xFF);
    ptr[1] = ((val >> 8) & 0xFF);
    ptr[2] = ((val >> 16) & 0xFF);
    ptr[3] = ((val >> 24) & 0xFF);
}

static void put_unaligned_le64(uint64_t value, void *dest) {
    uint8_t *ptr = (uint8_t*)dest;
    ptr[0] = (uint8_t)(value >> 0);
    ptr[1] = (uint8_t)(value >> 8);
    ptr[2] = (uint8_t)(value >> 16);
    ptr[3] = (uint8_t)(value >> 24);
    ptr[4] = (uint8_t)(value >> 32);
    ptr[5] = (uint8_t)(value >> 40);
    ptr[6] = (uint8_t)(value >> 48);
    ptr[7] = (uint8_t)(value >> 56);
}

#ifndef IEEE80211_MAX_CHAINS
#define IEEE80211_MAX_CHAINS 4
#endif

#define IEEE80211_CHAN_2GHZ       0x0080
#define IEEE80211_CHAN_5GHZ       0x0100
#define IEEE80211_CHAN_6GHZ       0x0200
#define IEEE80211_CHAN_OFDM       0x0040
#define IEEE80211_CHAN_CCK        0x0020
#define IEEE80211_CHAN_HT40PLUS   0x0800
#define IEEE80211_CHAN_HT40MINUS  0x0400
#define IEEE80211_CHAN_RADAR      0x0010
#define IEEE80211_CHAN_DYN        0x0020
#define IEEE80211_CHAN_NO_IR      0x0002
#define IEEE80211_CHAN_DISABLED   0x0001
#define IEEE80211_CHAN_NO_HT40    0x0040
#define IEEE80211_CHAN_NO_80MHZ   0x1000
#define IEEE80211_CHAN_NO_160MHZ  0x2000
#define IEEE80211_CHAN_NO_20MHZ   0x8000
#define IEEE80211_CHAN_PSD        0x4000


#define IEEE80211_RADIOTAP_MCS_HAVE_MCS   0x01
#define IEEE80211_RADIOTAP_MCS_HAVE_BW    0x02
#define IEEE80211_RADIOTAP_MCS_HAVE_GI    0x04
#define IEEE80211_RADIOTAP_MCS_HAVE_FMT   0x08
#define IEEE80211_RADIOTAP_MCS_HAVE_FEC   0x10
#define IEEE80211_RADIOTAP_MCS_HAVE_STBC  0x20
#define IEEE80211_RADIOTAP_MCS_HAVE_NESS  0x40


#define IEEE80211_RADIOTAP_MCS_BW_MASK    0x03
#define IEEE80211_RADIOTAP_MCS_BW_20      0
#define IEEE80211_RADIOTAP_MCS_BW_40      1
#define IEEE80211_RADIOTAP_MCS_BW_20L     2
#define IEEE80211_RADIOTAP_MCS_BW_20U     3
#define IEEE80211_RADIOTAP_MCS_BW_80      4
#define IEEE80211_RADIOTAP_MCS_BW_160     5

#define IEEE80211_RADIOTAP_MCS_SGI        0x04
#define IEEE80211_RADIOTAP_MCS_LDPC       0x08
#define IEEE80211_RADIOTAP_MCS_STBC_MASK  0x30
#define IEEE80211_RADIOTAP_MCS_STBC_SHIFT 4

#define IEEE80211_RADIOTAP_MCS_FMT_GF     0x40
#define IEEE80211_RADIOTAP_MCS_FMT_LGI    0x80

/* IEEE80211 Radiotap VHT known flags */
#define IEEE80211_RADIOTAP_VHT_KNOWN_STBC            0x0001
#define IEEE80211_RADIOTAP_VHT_KNOWN_TXOP_PS_NA      0x0002
#define IEEE80211_RADIOTAP_VHT_KNOWN_GI              0x0004
#define IEEE80211_RADIOTAP_VHT_KNOWN_SGI_NSYM_MCS    0x0008
#define IEEE80211_RADIOTAP_VHT_KNOWN_LDPC_EXTRA_OFDM 0x0010
#define IEEE80211_RADIOTAP_VHT_KNOWN_BEAMFORMED      0x0020
#define IEEE80211_RADIOTAP_VHT_KNOWN_BANDWIDTH       0x0040
#define IEEE80211_RADIOTAP_VHT_KNOWN_GROUP_ID        0x0080
#define IEEE80211_RADIOTAP_VHT_KNOWN_PARTIAL_AID     0x0100

#define IEEE80211_RADIOTAP_VHT_FLAG_SGI              0x01
#define IEEE80211_RADIOTAP_VHT_FLAG_STBC             0x02
#define IEEE80211_RADIOTAP_VHT_FLAG_TXOP_PS_NA       0x04
#define IEEE80211_RADIOTAP_VHT_FLAG_SGI_NSYM_MCS     0x08
#define IEEE80211_RADIOTAP_VHT_FLAG_LDPC_EXTRA_OFDM  0x10
#define IEEE80211_RADIOTAP_VHT_FLAG_BEAMFORMED       0x20

#define IEEE80211_RADIOTAP_CODING_LDPC_USER0  0x01


static inline uint8_t find_first_set_ll(uint64_t x) {
    if (x == 0) return 0;
    uint8_t pos = 1;
    while ((x & 1) == 0) {
        x >>= 1;
        pos++;
    }
    return pos;
}

#define __bf_shf(x) (find_first_set_ll(x) - 1)
#define FIELD_PREP(_mask, _val) \
    (((uint32_t)(_val) << __bf_shf(_mask)) & (_mask))

/**
 * rwnx_rx_add_rtap_hdr - Add radiotap header to sk_buff
 *
 * @skb: skb received (will include the radiotap header)
 * @rxvect: Rx vector
 * @phy_info: Information regarding the phy
 * @hwvect: HW Info (NULL if vendor specific data is available)
 * @rtap_len: Length of the radiotap header
 * @vend_rtap_len: radiotap vendor length (0 if not present)
 * @vend_it_present: radiotap vendor present
 *
 * Builds a radiotap header and add it to @skb.
 */
static void rwnx_rx_add_rtap_hdr(void *skb,
                                 struct rx_vector_1 *rxvect,
                                 struct phy_channel_info_desc *phy_info,
                                 struct hw_vect *hwvect,
                                 int rtap_len,
                                 u8 vend_rtap_len,
                                 u32 vend_it_present)
{
    struct ieee80211_radiotap_header *rtap;
    u8 *pos, rate_idx;
    __le32 *it_present;
    u32 it_present_val = 0;
    bool fec_coding = false;
    bool short_gi = false;
    bool stbc = false;
    bool aggregation = false;

    rtap = (struct ieee80211_radiotap_header *)skb;
    memset((u8 *) rtap, 0, rtap_len);

    rtap->it_version = 0;
    rtap->it_pad = 0;
    rtap->it_len = cpu_to_le16(rtap_len + vend_rtap_len);

    it_present = &rtap->it_present;

    // Check for multiple antennas
    if (generic_hweight32(rxvect->antenna_set) > 1) {
        // int chain;
        unsigned long chains = rxvect->antenna_set;

        //for_each_set_bit(chain, &chains, IEEE80211_MAX_CHAINS) {
        for (int e = 0; e < IEEE80211_MAX_CHAINS; e++) {
            if (chains & BIT(e)) {
                it_present_val |=
                    BIT(IEEE80211_RADIOTAP_EXT) |
                    BIT(IEEE80211_RADIOTAP_RADIOTAP_NAMESPACE);
                put_unaligned_le32(it_present_val, it_present);
                it_present++;
                it_present_val = BIT(IEEE80211_RADIOTAP_ANTENNA) |
                                BIT(IEEE80211_RADIOTAP_DBM_ANTSIGNAL);
            }
        }
    }

    // Check if vendor specific data is present
    if (vend_rtap_len) {
        it_present_val |= BIT(IEEE80211_RADIOTAP_VENDOR_NAMESPACE) |
                          BIT(IEEE80211_RADIOTAP_EXT);
        put_unaligned_le32(it_present_val, it_present);
        it_present++;
        it_present_val = vend_it_present;
    }

    put_unaligned_le32(it_present_val, it_present);
    pos = (void *)(it_present + 1);

    // IEEE80211_RADIOTAP_TSFT
    if (hwvect) {
        rtap->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_TSFT);
        // padding
        while ((pos - (u8 *)rtap) & 7)
            *pos++ = 0;
        put_unaligned_le64((((u64)le32_to_cpu(hwvect->tsf_hi) << 32) +
                            (u64)le32_to_cpu(hwvect->tsf_lo)), pos);
        // aic_dbg("TS %lx\r\n", (((u64)le32_to_cpu(hwvect->tsf_hi) << 32) +
        //                     (u64)le32_to_cpu(hwvect->tsf_lo)));
        // aic_dbg("tsf_hi %x, tsf_lo %x\r\n", hwvect->tsf_hi, hwvect->tsf_lo);
        pos += 8;
    }

    // IEEE80211_RADIOTAP_FLAGS
    rtap->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_FLAGS);
    if (hwvect && (!hwvect->frm_successful_rx))
        *pos |= IEEE80211_RADIOTAP_F_BADFCS;
    if (!rxvect->pre_type
            && (rxvect->format_mod <= FORMATMOD_NON_HT_DUP_OFDM))
        *pos |= IEEE80211_RADIOTAP_F_SHORTPRE;
    pos++;

    // IEEE80211_RADIOTAP_RATE
    // check for HT, VHT or HE frames
    if (rxvect->format_mod >= FORMATMOD_HE_SU) {
        rate_idx = rxvect->he.mcs;
        fec_coding = rxvect->he.fec;
        stbc = rxvect->he.stbc;
        aggregation = true;
        *pos = 0;
    } else if (rxvect->format_mod == FORMATMOD_VHT) {
        rate_idx = rxvect->vht.mcs;
        fec_coding = rxvect->vht.fec;
        short_gi = rxvect->vht.short_gi;
        stbc = rxvect->vht.stbc;
        aggregation = true;
        *pos = 0;
    } else if (rxvect->format_mod > FORMATMOD_NON_HT_DUP_OFDM) {
        rate_idx = rxvect->ht.mcs;
        fec_coding = rxvect->ht.fec;
        short_gi = rxvect->ht.short_gi;
        stbc = rxvect->ht.stbc;
        aggregation = rxvect->ht.aggregation;
        *pos = 0;
    } else {
        #define DIV_ROUND_UP(a, b) (((a) + (b) - 1) / (b))
        // struct ieee80211_supported_band *band =
        //         rwnx_hw->wiphy->bands[phy_info->phy_band];
        rtap->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_RATE);
        rate_idx = legrates_lut[rxvect->leg_rate].idx;
        // if (phy_info->phy_band == NL80211_BAND_5GHZ)
        //     rate_idx -= 4;  /* rwnx_ratetable_5ghz[0].hw_value == 4 */
        *pos = DIV_ROUND_UP(tx_legrates_lut_rate[rate_idx], 5);
        // aic_dbg("leg_rate %d %x\r\n", rxvect->leg_rate, *pos);
    }
    pos++;

    // IEEE80211_RADIOTAP_CHANNEL
    rtap->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_CHANNEL);
    put_unaligned_le16(phy_info->phy_prim20_freq, pos);
    pos += 2;

    if (phy_info->phy_band == NL80211_BAND_5GHZ)
        put_unaligned_le16(IEEE80211_CHAN_OFDM | IEEE80211_CHAN_5GHZ, pos);
    else if (rxvect->format_mod > FORMATMOD_NON_HT_DUP_OFDM)
        put_unaligned_le16(IEEE80211_CHAN_DYN | IEEE80211_CHAN_2GHZ, pos);
    else
        put_unaligned_le16(IEEE80211_CHAN_CCK | IEEE80211_CHAN_2GHZ, pos);
    pos += 2;

    if (generic_hweight32(rxvect->antenna_set) == 1) {
        // IEEE80211_RADIOTAP_DBM_ANTSIGNAL
        rtap->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_DBM_ANTSIGNAL);
        *pos++ = rxvect->rssi1;

        // IEEE80211_RADIOTAP_ANTENNA
        rtap->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_ANTENNA);
        *pos++ = rxvect->antenna_set;
    }

    // IEEE80211_RADIOTAP_LOCK_QUALITY is missing
    // IEEE80211_RADIOTAP_DB_ANTNOISE is missing

    // IEEE80211_RADIOTAP_RX_FLAGS
    rtap->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_RX_FLAGS);
    // 2 byte alignment
    if ((pos - (u8 *)rtap) & 1)
        *pos++ = 0;
    put_unaligned_le16(0, pos);
    //Right now, we only support fcs error (no RX_FLAG_FAILED_PLCP_CRC)
    pos += 2;

    // aic_dbg("rtap->it_present %08x\r\n", rtap->it_present);
    // Check if HT
    if ((rxvect->format_mod == FORMATMOD_HT_MF)
            || (rxvect->format_mod == FORMATMOD_HT_GF)) {
        rtap->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_MCS);
        *pos++ = IEEE80211_RADIOTAP_MCS_HAVE_MCS |
                 IEEE80211_RADIOTAP_MCS_HAVE_GI |
                 IEEE80211_RADIOTAP_MCS_HAVE_BW;
        *pos = 0;
        if (short_gi)
            *pos |= IEEE80211_RADIOTAP_MCS_SGI;
        if (rxvect->ch_bw  == PHY_CHNL_BW_40)
            *pos |= IEEE80211_RADIOTAP_MCS_BW_40;
        if (rxvect->format_mod == FORMATMOD_HT_GF)
            *pos |= IEEE80211_RADIOTAP_MCS_FMT_GF;
        if (fec_coding)
            *pos |= IEEE80211_RADIOTAP_MCS_HAVE_FEC;//IEEE80211_RADIOTAP_MCS_FEC_LDPC;
        // #if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
        // *pos++ |= stbc << 5;
        // #else
        *pos++ |= stbc << IEEE80211_RADIOTAP_MCS_STBC_SHIFT;
        // #endif
        *pos++ = rate_idx;
    }

    // check for HT or VHT frames
    if (aggregation && hwvect) {
        // 4 byte alignment
        while ((pos - (u8 *)rtap) & 3)
            pos++;
        rtap->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_AMPDU_STATUS);
        put_unaligned_le32(hwvect->ampdu_cnt, pos);
        pos += 4;
        put_unaligned_le32(0, pos);
        pos += 4;
    }

    // Check for VHT frames
    if (rxvect->format_mod == FORMATMOD_VHT) {
        u16 vht_details = IEEE80211_RADIOTAP_VHT_KNOWN_GI |
                          IEEE80211_RADIOTAP_VHT_KNOWN_BANDWIDTH;
        u8 vht_nss = rxvect->vht.nss + 1;

        rtap->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_VHT);

        if ((rxvect->ch_bw == PHY_CHNL_BW_160)
                && phy_info->phy_center2_freq)
            vht_details &= ~IEEE80211_RADIOTAP_VHT_KNOWN_BANDWIDTH;
        put_unaligned_le16(vht_details, pos);
        pos += 2;

        // flags
        if (short_gi)
            *pos |= IEEE80211_RADIOTAP_VHT_FLAG_SGI;
        if (stbc)
            *pos |= IEEE80211_RADIOTAP_VHT_FLAG_STBC;
        pos++;

        // bandwidth
        if (rxvect->ch_bw == PHY_CHNL_BW_40)
            *pos++ = 1;
        if (rxvect->ch_bw == PHY_CHNL_BW_80)
            *pos++ = 4;
        else if ((rxvect->ch_bw == PHY_CHNL_BW_160)
                && phy_info->phy_center2_freq)
            *pos++ = 0; //80P80
        else if  (rxvect->ch_bw == PHY_CHNL_BW_160)
            *pos++ = 11;
        else // 20 MHz
            *pos++ = 0;

        // MCS/NSS
        *pos = (rate_idx << 4) | vht_nss;
        pos += 4;
        if (fec_coding)
            // #if LINUX_VERSION_CODE < KERNEL_VERSION(3, 15, 0)
            // *pos |= 0x01;
            // #else
            *pos |= IEEE80211_RADIOTAP_CODING_LDPC_USER0;
            // #endif
        pos++;
        // group ID
        pos++;
        // partial_aid
        pos += 2;
    }

    // Check for HE frames
    if (rxvect->format_mod == FORMATMOD_HE_SU) {
        struct ieee80211_radiotap_he he;
        #define HE_PREP(f, val) cpu_to_le16(FIELD_PREP(IEEE80211_RADIOTAP_HE_##f, val))
        #define D1_KNOWN(f) cpu_to_le16(IEEE80211_RADIOTAP_HE_DATA1_##f##_KNOWN)
        #define D2_KNOWN(f) cpu_to_le16(IEEE80211_RADIOTAP_HE_DATA2_##f##_KNOWN)

        he.data1 = D1_KNOWN(DATA_MCS) | D1_KNOWN(BSS_COLOR) | D1_KNOWN(BEAM_CHANGE) |
                   D1_KNOWN(UL_DL) | D1_KNOWN(CODING) |  D1_KNOWN(STBC) |
                   D1_KNOWN(BW_RU_ALLOC) | D1_KNOWN(DOPPLER) | D1_KNOWN(DATA_DCM);
        he.data2 = D2_KNOWN(GI) | D2_KNOWN(TXBF);

        if (stbc) {
            he.data6 |= HE_PREP(DATA6_NSTS, 2);
            he.data3 |= HE_PREP(DATA3_STBC, 1);
        } else {
            he.data6 |= HE_PREP(DATA6_NSTS, rxvect->he.nss);
        }

        he.data3 |= HE_PREP(DATA3_BSS_COLOR, rxvect->he.bss_color);
        he.data3 |= HE_PREP(DATA3_BEAM_CHANGE, rxvect->he.beam_change);
        he.data3 |= HE_PREP(DATA3_UL_DL, rxvect->he.uplink_flag);
        he.data3 |= HE_PREP(DATA3_BSS_COLOR, rxvect->he.bss_color);
        he.data3 |= HE_PREP(DATA3_DATA_MCS, rxvect->he.mcs);
        he.data3 |= HE_PREP(DATA3_DATA_DCM, rxvect->he.dcm);
        he.data3 |= HE_PREP(DATA3_CODING, rxvect->he.fec);

        he.data5 |= HE_PREP(DATA5_GI, rxvect->he.gi_type);
        he.data5 |= HE_PREP(DATA5_TXBF, rxvect->he.beamformed);
        he.data5 |= HE_PREP(DATA5_LTF_SIZE, rxvect->he.he_ltf_type + 1);

        switch (rxvect->ch_bw) {
        case PHY_CHNL_BW_20:
            he.data5 |= HE_PREP(DATA5_DATA_BW_RU_ALLOC,
                        IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_20MHZ);
            break;
        case PHY_CHNL_BW_40:
            he.data5 |= HE_PREP(DATA5_DATA_BW_RU_ALLOC,
                        IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_40MHZ);
            break;
        case PHY_CHNL_BW_80:
            he.data5 |= HE_PREP(DATA5_DATA_BW_RU_ALLOC,
                        IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_80MHZ);
            break;
        case PHY_CHNL_BW_160:
            he.data5 |= HE_PREP(DATA5_DATA_BW_RU_ALLOC,
                        IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_160MHZ);
            break;
        default:
            DBG_MACIF_ERR("Invalid SU BW %d\n", rxvect->ch_bw);
        }

        he.data6 |= HE_PREP(DATA6_DOPPLER, rxvect->he.doppler);

        /* ensure 2 byte alignment */
        while ((pos - (u8 *)rtap) & 1)
            pos++;
        rtap->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_HE);
        memcpy(pos, &he, sizeof(he));
        pos += sizeof(he);
    }

    // Rx Chains
    if (generic_hweight32(rxvect->antenna_set) > 1) {
        // int chain;
        unsigned long chains = rxvect->antenna_set;
        u8 rssis[4] = {rxvect->rssi1, rxvect->rssi1, rxvect->rssi1, rxvect->rssi1};

        //for_each_set_bit(chain, &chains, IEEE80211_MAX_CHAINS) {
        for (int e = 0; e < IEEE80211_MAX_CHAINS; e++) {
            if (chains & BIT(e)) {
                *pos++ = rssis[e];
                *pos++ = e;
            }
        }
    }
}
#endif /* CONFIG_MONITOR_RADIO_HEADER */

/*
void dump_hex32(uint32_t *data, int cnt)
{
    unsigned long i;
    i = 0;
    OSAL_DBG_PRINTF("%p: ", &data[i]);
    for (i = 0; i < cnt; i++) {
        if (i && (i % 16 == 0))
            OSAL_DBG_PRINTF("\n%p: ", &data[i]);
        OSAL_DBG_PRINTF("%08x ", data[i]);
    }
    OSAL_DBG_PRINTF("\n");
}
*/

/**
 ****************************************************************************************
 * @brief callback function
 *
 * Extract received packet informations (frame length, type, mac addr ...) in monitor mode
 *
 * @param[in] info  RX Frame information.
 * @param[in] arg   Not used
 ****************************************************************************************
 */
void rwnx_rx_monitor_cb(struct fhost_frame_info *info, void *arg)
{
    int info_size = sizeof(struct rx_info);
    //dump_hex32((uint32_t *)info->prinfo, (info_size + 3) / sizeof(uint32_t));
    if (info->payload == NULL) {
        DBG_MACIF_ERR("Unsupported frame: length = %d\r\n", info->length);
    } else {
        struct mac_hdr *hdr = (struct mac_hdr *)info->payload;
        uint8_t *adr1 = ((uint8_t *)(hdr->addr1.array));
        uint8_t *adr2 = ((uint8_t *)(hdr->addr2.array));
        uint8_t *adr3 = ((uint8_t *)(hdr->addr3.array));
        if ((hdr->fctl & MAC_FCTRL_TYPESUBTYPE_MASK) == MAC_FCTRL_BEACON) {
            return;
        }
        DBG_MACIF_INF("fc=%04X SN=%d len=%d rssi=%d\n",
                 hdr->fctl, hdr->seq >> 4, info->length, info->rssi);
        // if ( ((hdr->fctl & MAC_FCTRL_TYPESUBTYPE_MASK) == MAC_FCTRL_ACTION) ||
        //      ((hdr->fctl & MAC_FCTRL_TYPE_MASK) == MAC_FCTRL_DATA_T)) {
        //     printf("a1=%02x:%02x:%02x:%02x:%02x:%02x a2=%02x:%02x:%02x:%02x:%02x:%02x "
        //         "a3=%02x:%02x:%02x:%02x:%02x:%02x fc=%04X SN=%d len=%d\n",
        //     adr1[0], adr1[1], adr1[2], adr1[3], adr1[4], adr1[5],
        //         adr2[0], adr2[1], adr2[2], adr2[3], adr2[4], adr2[5],
        //         adr3[0], adr3[1], adr3[2], adr3[3], adr3[4], adr3[5],
        //         hdr->fctl, hdr->seq >> 4, info->length);
        //     rwnx_data_dump("Frame", info->payload, 32);
        // }
        #ifdef CONFIG_MONITOR_RADIO_HEADER
        //dump_hex32((uint32_t *)(&info->prinfo->vect.recvec1a), 4);
        uint16_t radio_hdr_len = rwnx_rx_rtap_hdrlen((struct rx_vector_1 *)(&info->prinfo->vect.recvec1a), false);
        uint8_t *radiotap_hdr = (uint8_t *)rtos_malloc(radio_hdr_len);
        if (radiotap_hdr) {
            rwnx_rx_add_rtap_hdr(radiotap_hdr, &info->prinfo->vect.recvec1a,
            &info->prinfo->phy_info, &info->prinfo->vect,
            radio_hdr_len, 0, 0);
            DBG_MACIF_VRB("radio_hdr_len=%d\n", radio_hdr_len);
            rtos_free(radiotap_hdr);
        }
        #endif /* CONFIG_MONITOR_RADIO_HEADER */
    }
}