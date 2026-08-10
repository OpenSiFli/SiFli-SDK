/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _RWNX_CONFIG_H_
#define _RWNX_CONFIG_H_

#define CONFIG_SLEEP_LEVEL 0

#define NX_DEBUG  0

#define CFG_SOFTAP

#define NX_CRYPTOLIB 1


#ifdef CFG_SOFTAP
#define NX_BEACONING 1
#else  /* CFG_SOFTAP */
#define NX_BEACONING 0
#endif /* CFG_SOFTAP */

//#ifdef CFG_WPS
#if 1
#define NX_WPS 1
#else  /* CFG_WPS */
#define NX_WPS 0
#endif /* CFG_WPS */

/// Maximum size of a beacon frame
#define NX_BCNFRAME_LEN 512
/** @} Beacon */

#define NX_TXQ_CNT 5

#define NX_REORD 5
#define NX_AMPDU_TX 1

#define RX_MAX_AMSDU_SUBFRAME_LEN 1600

#ifdef CFG_HS_IPERF
#define NX_REORD_BUF_SIZE 32
#define NX_TXDESC_CNT0 8
#define NX_TXDESC_CNT1 32
#define NX_TXDESC_CNT2 32
#define NX_TXDESC_CNT3 16
#define NX_TXDESC_CNT4 0
#else
#define NX_REORD_BUF_SIZE 8
#define NX_TXDESC_CNT0 8
#define NX_TXDESC_CNT1 16
#define NX_TXDESC_CNT2 16
#define NX_TXDESC_CNT3 16
#define NX_TXDESC_CNT4 0
#endif

#define RW_USER_MAX 1

#define NX_VIRT_DEV_MAX 4

#define NX_REMOTE_STA_MAX 32
#define NX_REMOTE_STA_MAX_WD4M 8

/// uPattern for TX header descriptor.
#define TX_HEADER_DESC_PATTERN 0xCAFEBABE
/// uPattern for TX buffer descriptor
#define TX_PAYLOAD_DESC_PATTERN 0xCAFEFADE
/// uPattern for RX header descriptor.
#define RX_HEADER_DESC_PATTERN 0xBAADF00D
/// uPattern for RX payload descriptor.
#define RX_PAYLOAD_DESC_PATTERN 0xC0DEDBAD



#define NX_AMSDU_DEAGG 0

#define NX_VHT 1

#define NX_SMARTCONFIG 0
#define NX_FHOST_MONITOR 1

#define NX_UF_EN 0

////////////////////////////NO warning /////////////////////////////
#define NX_SYS_STAT 0

#define NX_BCN_AUTONOMOUS_TX 0
#define NX_TG 0
#define NX_FULLY_HOSTED 0
#define NX_HE 1

#define NX_MULTI_ROLE 0

#if 1//defined(CFG_P2P)
#define NX_P2P 1
#define NX_P2P_VIF_MAX     2
#define NX_P2P_GO 1
#else
#define NX_P2P 0
#define NX_P2P_VIF_MAX     2
#define NX_P2P_GO 0
#endif
#define NX_CHNL_CTXT 0
#define NX_TDLS 0
#define NX_POWERSAVE 0
#define NX_UAPSD 0
#define NX_CONNECTION_MONITOR 0
#define NX_UMAC_PRESENT 0
#define NX_TD_STA 0
#define NX_MAC_HE 0
#define RW_UMESH_EN 0

#define NX_TX_FRAME 0
#define RW_MESH_EN 0

#define NX_AMSDU_TX 0
// Force MFP support if HE is enabled, as it is mandatory for WiFi certification
#if NX_HE
#define NX_MFP 1
#else
#define NX_MFP 0
#endif
#define RW_MUMIMO_TX_EN 0
#define NX_BW_LEN_ADAPT 0
#define RW_MESH_EN 0
#define RW_BFMER_EN 0
#define NX_11AX_DRAFT_2_0 0

/**
 *******************************************************************************
 * @name Radar Configuration
 *******************************************************************************
 */
#define NX_RADAR_DETECT 0
#define RW_RADAR_EN     0
/** @} Radar */

/**
 *******************************************************************************
 * @name Misc
 *******************************************************************************
 */
/// Implement IEEE802.11ax-D2.0 (instead of D3.2)
//#define NX_11AX_DRAFT_2_0               0

/// This macro appears in some generated header files, define it to avoid warning
//#define RW_NX_LDPC_DEC                  1

/// This macro appears in some generated header files, define it to avoid warning
#define RW_NX_AGC_SNR_EN                1

/// This macro appears in some generated header files, define it to avoid warning
#define RW_NX_IQ_COMP_EN                1

/// This macro appears in some generated header files, define it to avoid warning
#define RW_NX_FIQ_COMP_EN               1

/// This macro appears in some generated header files, define it to avoid warning
#define RW_NX_DERIV_80211B              1

/// This macro appears in some generated header files, define it to avoid warning
//#define RW_KEY_EXTENDED                 1
/// @} misc

#endif //_RWNX_CONFIG_H_
