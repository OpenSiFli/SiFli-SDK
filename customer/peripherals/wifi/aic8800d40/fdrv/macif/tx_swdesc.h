/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file tx_swdesc.h
 *
 * @brief Definition of Tx SW descriptors
 *
 * Copyright (C) RivieraWaves 2011-2019
 *
 ****************************************************************************************
 */

#ifndef _TX_SWDESC_H_
#define _TX_SWDESC_H_

/**
 ****************************************************************************************
 * @defgroup TX TX
 * @ingroup LMAC
 * @brief Definition of TX Path functionality
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @defgroup TX_SWDESC TX_SWDESC
 * @ingroup TX
 * @brief Definition and management of Tx SW descriptors
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "co_list.h"
#include "lmac_mac.h"
#include "mac_frame.h"
#include "rwnx_config.h"
#include "aic_plat_types.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#if NX_AMPDU_TX
///AMDPU delimiter length in bytes
#define DELIMITER_LEN             4
#endif //(NX_AMPDU_TX)

#define RATE_CONTROL_STEPS 4

/// Singleton frame can be transmitted using beamforming
#define TX_SWDESC_UMAC_BEAMFORM_BIT     (CO_BIT(0))
/// Singleton trial frame can be transmitted using beamforming
#define TX_SWDESC_UMAC_TRIAL_BEAMFORM_BIT   (CO_BIT(1))
/// Trial frame can be transmitted using STBC
#define TX_SWDESC_UMAC_TRIAL_STBC_BIT       (CO_BIT(2))

/*
 * TYPE and STRUCT DEFINITIONS
 ****************************************************************************************
 */

/// Structure indicating the status and other information about the transmission
struct tx_cfm_tag
{
/*
    /// PN that was used for the transmission
    uint16_t pn[4];
    /// Sequence number of the packet
    uint16_t sn;
    /// Timestamp of first transmission of this MPDU
    uint16_t timestamp;
*/
    /// Number of flow control credits allocated
    int8_t credits;
    /// Size (in mdpu) of the A-MPDU in which this frame as been transmitted.
    /// Valid only for singleton (=1) or last frame of the A-MPDU
    uint8_t ampdu_size;
    #if NX_AMSDU_TX
    /// Size allowed for AMSDU
    uint16_t amsdu_size;
    #endif

    /// TX status
    uint32_t status;
    u32_l hostid;
};


/// Descriptor filled by the Host
struct hostdesc
{
    /// Size of the payload
    u16_l packet_len;

    u16_l flags_ext;

    u32_l hostid;
    /// Address of the status descriptor in host memory (used for confirmation upload)
//    u32_l status_desc_addr;
    /// Destination Address
    struct mac_addr eth_dest_addr;
    /// Source Address
    struct mac_addr eth_src_addr;

    /// Ethernet Type
    u16_l ethertype;

    u8_l ac;
    /// Packet TID (0xFF if not a QoS frame)
    u8_l tid;
    /// Interface Id
    u8_l vif_idx;
    /// Station Id (0xFF if station is unknown)
    u8_l staid;
#if 0//def CONFIG_RWNX_MUMIMO_TX
    /// MU-MIMO information (GroupId and User Position in the group) - The GroupId
    /// is located on bits 0-5 and the User Position on bits 6-7. The GroupId value is set
    /// to 63 if MU-MIMO shall not be used
    u8_l mumimo_info;
#endif /* CONFIG_RWNX_MUMIMO_TX */
    /// TX flags
    u16_l flags;

    /// TX confirmation callback (Only used for mgmt frame)
    void *cfm_cb;
    /// TX confirmation callback argument
    void *cfm_cb_arg;
    /// TX confirmation info
    struct tx_cfm_tag cfm;

    /// Pointer to packet payload
    uint32_t packet_addr;
};
/// Policy Table Structure used to store Policy Table Information used by MAC HW to
/// prepare transmit vector to used by PHY.
struct tx_policy_tbl
{
    /// Unique Pattern at the start of Policy Table
    uint32_t upatterntx;
    /// PHY Control Information 1 used by MAC HW
    uint32_t phycntrlinfo1;
    /// PHY Control Information 2 used by MAC HW
    uint32_t phycntrlinfo2;
    /// MAC Control Information 1 used by MAC HW
    uint32_t maccntrlinfo1;
    /// MAC Control Information 2 used by MAC HW
    uint32_t maccntrlinfo2;
    /// Rate Control Information used by MAC HW
    uint32_t ratecntrlinfo[RATE_CONTROL_STEPS];
    /// Power Control Information used by MAC HW
    uint32_t powercntrlinfo[RATE_CONTROL_STEPS];
};
/// Compressed Policy Table Structure used to store Policy Table Information used by MAC HW
/// to get TX information for secondary users in case of MU-MIMO PPDU transmission.
struct tx_compressed_policy_tbl
{
    /// Unique Pattern at the start of Policy Table
    uint32_t upatterntx;
    /// Secondary User Control Information
    uint32_t sec_user_control;
};

/// Packet descriptor
struct txl_buffer_control
{
    union
    {
        /// Policy table (filled by the upper MAC)
        struct tx_policy_tbl policy_tbl;
        /// Compressed policy table (used for MU-MIMO if enabled)
        struct tx_compressed_policy_tbl comp_pol_tbl;
    };
    /// MAC Control Information field (filled by the upper MAC)
    uint32_t mac_control_info;
    /// PHY Control Information field (filled by the upper MAC)
    uint32_t phy_control_info;
    /**
     * Flags from UMAC which do not fit with tx_hd.macctrlinfo2 format
     *      Bit 0 - Indicate if frame can be beamformed when sent as a singleton
     *      Bit 1 - Indicate if trial frame can be beamformed when sent as a singleton
     *      Bit 2 - Indicate if trial frame can be sent using STBC
     */
    uint32_t tx_flags;
};

/// Descriptor filled by the UMAC
struct umacdesc
{
    #if NX_AMPDU_TX
    ///First Sequence Number of the BlockAck window
    uint16_t sn_win;
    /// Flags from UMAC (match tx_hd.macctrlinfo2 format)
    uint32_t flags;
    /// PHY related flags field - nss, rate, GI type, BW type - filled by driver
    uint32_t phy_flags;
    #endif //(NX_AMPDU_TX)
    /// Pointer to the buffer control to use
    struct txl_buffer_control *buf_control;
    /// Length of the payload
    uint16_t payl_len;
    /// Header length (MAC + IV + EIV + 802.2 Header)
    uint8_t head_len;
    /// 802.2 header length
    uint8_t hdr_len_802_2;
    /// Tail length (MIC + ICV)
    uint8_t tail_len;
    #if (RW_MESH_EN)
    /// Indicate if Mesh Control field is present in the sent frame
    bool has_mesh_ctrl;
    /// Number of External Addresses to be inserted in the Mesh Control field
    uint8_t nb_ext_addr;
    /// Index of the path to be used
    uint8_t path_idx;
    /// Index of the proxy information to be used
    uint8_t proxy_idx;
    #endif //(RW_MESH_EN)
};

/// Definition of a TX header descriptor.
struct tx_hd
{
    /// Unique pattern for transmit DMA.
    uint32_t             upatterntx;
    /// Starting descriptor of the next atomic frame exchange sequence.
    uint32_t             nextfrmexseq_ptr;
    /// Next MPDU in the frame exchange.
    uint32_t             nextmpdudesc_ptr;
    /// First payload buffer descriptor/Secondary user 1 THD address
    union {
        /// First payload buffer descriptor
        uint32_t             first_pbd_ptr;
        /// Secondary user 1 THD address
        uint32_t             sec_user1_ptr;
    };
    /// Data buffer start/Secondary user 2 THD address.
    union {
        /// Data buffer start
        uint32_t             datastartptr;
        /// Secondary user 2 THD address
        uint32_t             sec_user2_ptr;
    };
    /// Data buffer end /Secondary user 3 THD address.
    union {
        /// Data buffer end
        uint32_t             dataendptr;
        /// Secondary user 3 THD address
        uint32_t             sec_user3_ptr;
    };
    /// Total length of the frame on air.
    uint32_t             frmlen;
    /// MSDU lifetime parameter (for EDCA frame).
    uint32_t             frmlifetime;
    /// Valid only for A-MPDU header descriptor and for singleton MPDUs.
    uint32_t             phyctrlinfo;
    /// Valid only for A-MPDU header descriptor and for singleton MPDUs.
    uint32_t             policyentryaddr;
    /// Optional length fields in case of BW drop
    uint32_t             optlen[3];
    /// Valid only for A-MPDU header descriptor and for singleton MPDUs.
    uint32_t             macctrlinfo1;
    /// Valid only for MPDU header descriptor and for singleton MPDUs.
    uint32_t             macctrlinfo2;
    /// Valid only for A-MPDU header descriptor and for singleton MPDUs.
    uint32_t             statinfo;
    /// Medium time used.
    uint32_t             mediumtimeused;
};

/// Definition of a TX payload buffer descriptor.
struct tx_pbd
{
    /// Unique pattern for transmit buffer descriptor.
    uint32_t             upatterntx;
    /// Next buffer descriptor of the MPDU when the MPDU is split over several buffers.
    uint32_t             next;
    /// Data start in the buffer associated with this buffer descriptor.
    uint32_t             datastartptr;
    /// Data end in the buffer associated with this buffer descriptor (inclusive).
    uint32_t             dataendptr;
    /// Buffer control for transmit DMA.
    uint32_t             bufctrlinfo;
};


/// Aggregation descriptor, containing AMPDU THD, BAR descriptor, BAR payload and Policy Table
struct tx_agg_desc
{
    /// Pointer to the next element in the queue
    struct co_list_hdr list_hdr;
    /// Current status of the aggregation process
    uint16_t status;
    /// Amount of data currently downloaded for this A-MPDU
    uint16_t available_len;
    /// Number of MPDUs downloaded for this A-MPDU
    uint8_t available_cnt;
    /// STA IDX for which AMPDU is being built
    uint8_t sta_idx;
    /// TID for which AMPDU is being built
    uint8_t tid;
    /// Number of "users" currently sharing this descriptor
    /// The descriptor will be freed only when no more SW modules are using it
    uint8_t user_cnt;
    /// AMPDU THD (no payload)
    struct tx_hd a_thd;
    /// BAR THD
    struct tx_hd bar_thd;
    /// BAR payload space (no FCS)
    struct bar_frame bar_payl;
    /// Policy table for explicit BAR frame
    struct tx_policy_tbl bar_pol_tbl;
    /// Pointer to the BA SSC + bitmap
    struct ba_ssc_bitmap *ssc_bitmap;
    /// Pointer to the BA SSC + bitmap
    struct ba_ssc_bitmap_256 ssc_bitmap_tab;
    #if NX_MAC_HE
    /// Policy table for A-MPDU
    struct tx_policy_tbl pol_tbl;
    /// Pointer to last TX descriptor of the A-MPDU
    struct txdesc *txdesc_last;
    #endif
    #if NX_BW_LEN_ADAPT
    /// Pointer to TX descriptors finishing a BW step
    struct txdesc *txdesc[NX_BW_LEN_STEPS - 1];
    #endif
    #if RW_MUMIMO_TX_EN
    /// List containing the descriptors to be confirmed once BA is received for this
    /// secondary user
    struct co_list cfm;
    /// Pointer to the primary TX descriptor
    struct tx_agg_desc *prim_agg_desc;
    /// Pointer to the last BAR THD of the MU-MIMO frame exchange
    struct tx_hd *last_bar_thd;
    /// Bitfield indicating which user indexes have still data to download before being
    /// able to be chained to HW
    uint8_t download;
    #endif
    /// List in which the A-MPDU descriptor has to be pushed back after transmission
    struct co_list *free_list;
};

/// Definition of a TX confirmation descriptor
struct tx_hw_desc
{
    /// TX header descriptor attached to the MPDU
    struct tx_hd    thd;
};

/// LMAC Tx Descriptor
struct txdesc
{
    /// Pointer to the next element in the queue
    struct co_list_hdr list_hdr;
    /// Information provided by Host
    struct hostdesc host;
};

/// Description of the TX API
struct txdesc_api
{
    /// Information provided by Host
    struct hostdesc host;
};
/// @}
/// @}

#endif // _CO_SWDESC_H_