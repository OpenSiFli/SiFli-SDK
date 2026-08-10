/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file fhost_rx.h
 *
 * @brief Definitions of the fully hosted RX task.
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

#ifndef _FHOST_RX_H_
#define _FHOST_RX_H_

/**
 ****************************************************************************************
 * @defgroup FHOST_RX FHOST_RX
 * @ingroup FHOST
 * @brief Fully Hosted RX task implementation.
 * This module creates a task that will be used to handle the RX descriptors passed by
 * the WiFi task.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "cfgrwnx.h"
#include "co_math.h"
#include "co_list.h"
#include "rwnx_rx.h"
#include "hal_co.h"
#include "netif_def.h"
#include "rtos_port.h"


/// Maximum number MSDUs supported in one received A-MSDU
#define NX_MAX_MSDU_PER_RX_AMSDU 8

/// Decryption status mask.
#define RX_HD_DECRSTATUS 0x0000007C

/// Decryption type offset
#define RX_HD_DECRTYPE_OFT 2
/// Frame decrypted using WEP.
#define RX_HD_DECR_WEP (0x01 << RX_HD_DECRTYPE_OFT)
/// Frame decrypted using TKIP.
#define RX_HD_DECR_TKIP (0x02 << RX_HD_DECRTYPE_OFT)
/// Frame decrypted using CCMP 128bits.
#define RX_HD_DECR_CCMP128 (0x03 << RX_HD_DECRTYPE_OFT)

/// Packet contains an A-MSDU
#define RX_FLAGS_IS_AMSDU_BIT         CO_BIT(0)
/// Packet contains a 802.11 MPDU
#define RX_FLAGS_IS_MPDU_BIT          CO_BIT(1)
/// Packet contains 4 addresses
#define RX_FLAGS_4_ADDR_BIT           CO_BIT(2)
/// Packet is a Mesh Beacon received from an unknown Mesh STA
#define RX_FLAGS_NEW_MESH_PEER_BIT    CO_BIT(3)
#define RX_FLAGS_NEED_TO_REORD_BIT    CO_BIT(5)
#define RX_FLAGS_UPLOAD_BIT           CO_BIT(6)
#define RX_FLAGS_MONITOR_BIT          CO_BIT(7)

/// Offset of the VIF index field
#define RX_FLAGS_VIF_INDEX_OFT  8
/// Mask of the VIF index field
#define RX_FLAGS_VIF_INDEX_MSK  (0xFF << RX_FLAGS_VIF_INDEX_OFT)
/// Offset of the STA index field
#define RX_FLAGS_STA_INDEX_OFT  16
/// Mask of the STA index field
#define RX_FLAGS_STA_INDEX_MSK  (0xFF << RX_FLAGS_STA_INDEX_OFT)
/// Offset of the destination STA index field
#define RX_FLAGS_DST_INDEX_OFT  24
/// Mask of the destination STA index field
#define RX_FLAGS_DST_INDEX_MSK  (0xFF << RX_FLAGS_DST_INDEX_OFT)

/// Bitmask indicating that a received packet is not a MSDU
#define RX_FLAGS_NON_MSDU_MSK        (RX_FLAGS_IS_AMSDU_BIT | RX_FLAGS_IS_MPDU_BIT |     \
                                      RX_FLAGS_4_ADDR_BIT | RX_FLAGS_NEW_MESH_PEER_BIT)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
#define FHOST_RX_BUF_SIZE           (RX_MAX_AMSDU_SUBFRAME_LEN + 1)
#define FHOST_RX_BUF_COUNT          (32)
#define FHOST_RX_BUF_ALIGN_NUM      (64)

/// FHOST RX environment structure
struct fhost_rx_buf_tag
{
    /// Structure containing the information about the received payload
    struct rx_info info;
    /// Payload buffer space
    uint32_t payload[CO_ALIGN4_HI(FHOST_RX_BUF_SIZE)/sizeof(uint32_t)];
};

#if NX_UF_EN
/// Structure for receive Vector 1
struct rx_vector_1
{
    /// Contains the bytes 4 - 1 of Receive Vector 1
    uint32_t            recvec1a;
    /// Contains the bytes 8 - 5 of Receive Vector 1
    uint32_t            recvec1b;
    /// Contains the bytes 12 - 9 of Receive Vector 1
    uint32_t            recvec1c;
    /// Contains the bytes 16 - 13 of Receive Vector 1
    uint32_t            recvec1d;
};

struct rx_vector_desc
{
    /// Structure containing the information about the PHY channel that was used for this RX
    struct phy_channel_info phy_info;

    /// Structure containing the rx vector 1
    struct rx_vector_1 rx_vec_1;

    /// Used to mark a valid rx vector
    uint32_t pattern;
};
/// FHOST UF environment structure
struct fhost_rx_uf_buf_tag
{
    struct co_list_hdr hdr;
    struct rx_vector_desc rx_vector;

    /// Payload buffer space (empty in case of UF buffer)
    uint32_t payload[];
};

/// Structure used, when receiving UF, for the buffer elements exchanged between the FHOST RX and the MAC thread
struct fhost_rx_uf_buf_desc_tag
{
    /// Id of the RX buffer
    uint32_t host_id;
};
#endif // NX_UF_EN


#define IS_QOS_DATA(frame_cntrl) ((frame_cntrl & MAC_FCTRL_TYPESUBTYPE_MASK) == MAC_FCTRL_QOS_DATA)


typedef struct {
    struct co_list list;
    rtos_mutex mutex;
} fhost_rxq_t;

/// FHOST RX environment structure
struct fhost_rx_env_tag
{
    uint32_t  flags;
    uint16_t  next_rx_seq_no;
    /// Management frame Callback function pointer
    cb_fhost_rx mgmt_cb;
    /// Management Callback parameter
    void *mgmt_cb_arg;
    /// Monitor Callback function pointer
    cb_fhost_rx monitor_cb;
    /// Monitor Callback parameter
    void *monitor_cb_arg;
    hal_buf_list_s rxq;
    rtos_semaphore rxq_trigg;
};

/// Structure used for the buffer elements exchanged between the FHOST RX and the MAC thread
struct fhost_rx_buf_desc_tag
{
    /// Id of the RX buffer
    uint32_t host_id;
    /// Address of the payload inside the buffer
    uint32_t addr;
};

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */
/// FHOST RX environment
extern struct fhost_rx_env_tag fhost_rx_env;

/*
 * FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Set the callback to call when receiving management frames (i.e. they have
 * not been processed by the wifi task).
 *
 * @attention The callback is called with a @ref fhost_frame_info parameter that is only
 * valid during the callback. If needed the callback is responsible to save the frame for
 * futher processing.
 *
 * @param[in] cb   Callback function pointer
 * @param[in] arg  Callback parameter (NULL if not needed)
 ****************************************************************************************
 */
void fhost_rx_set_mgmt_cb(cb_fhost_rx cb, void *arg);

/**
 ****************************************************************************************
 * @brief Set the callback to call when receiving packets in monitor mode.
 *
 * @attention The callback is called with a @ref fhost_frame_info parameter that is only
 * valid during the callback. If needed the callback is responsible to save the frame for
 * futher processing.
 *
 * @param[in] cb   Callback function pointer
 * @param[in] arg  Callback parameter (NULL if not needed)
 ****************************************************************************************
 */
void fhost_rx_set_monitor_cb(cb_fhost_rx cb, void *arg);
uint8_t machdr_len_get(uint16_t frame_cntl);
void fhost_rx_buf_push(void *net_buf);
#ifdef CONFIG_RX_NOCOPY
void fhost_rx_buf_forward(struct fhost_rx_buf_tag *buf, void *lwip_msg, uint8_t is_reorder);
#else
void fhost_rx_buf_forward(struct fhost_rx_buf_tag *buf);
#endif /* CONFIG_RX_NOCOPY */
void fhost_rx_monitor_cb(void *buf, bool uf);

#if NX_UF_EN
void fhost_rx_uf_buf_push(struct fhost_rx_uf_buf_tag *);
#endif /* NX_UF_EN */

int fhost_rx_data_resend(net_if_t *net_if, struct fhost_rx_buf_tag *buf,
                         struct mac_addr *da, struct mac_addr *sa, uint8_t machdr_len);

void fhost_rx_init(struct rwnx_hw *rwnx_hw);
void fhost_rx_deinit(void);
struct sdio_buf_node_s * fhost_rxframe_dequeue(void);
void fhost_rxframe_enqueue(struct sdio_buf_node_s *node);
void fhost_rx_taskmem_free(void);

#ifdef CONFIG_RX_NOCOPY
uint8_t fhost_mac2ethernet_offset(void *buf);
#endif
/// @}

#endif // _FHOST_RX_H_