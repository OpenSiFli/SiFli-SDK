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

#ifndef _MAC_H_
#define _MAC_H_

/**
 ****************************************************************************************
 * @defgroup MAC MAC
 * @ingroup COMMON
 * @brief  Common defines,structures
 *
 * This module contains defines commonaly used for MAC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "co_int.h"
#include "co_math.h"
#include "co_utils.h"
#include "lmac_mac.h"

/* Sync with mac.h */

/*
 * DEFINES
 ****************************************************************************************
 */
/// duration of a Time Unit in microseconds
#define TU_DURATION                     1024

/// Mask to test if it's a basic rate - BIT(7)
#define MAC_BASIC_RATE                  0x80

/// Mask for extracting/checking word alignment
#define WORD_ALIGN                      3

/**
 ****************************************************************************************
 * Compare two MAC addresses.
 * The MAC addresses MUST be 16 bit aligned.
 * @param[in] addr1_ptr Pointer to the first MAC address.
 * @param[in] addr2_ptr Pointer to the second MAC address.
 * @return True if equal, false if not.
 ****************************************************************************************
 */
#define MAC_ADDR_CMP(addr1_ptr, addr2_ptr)                                              \
    ((*(((uint16_t*)(addr1_ptr)) + 0) == *(((uint16_t*)(addr2_ptr)) + 0)) &&            \
     (*(((uint16_t*)(addr1_ptr)) + 1) == *(((uint16_t*)(addr2_ptr)) + 1)) &&            \
     (*(((uint16_t*)(addr1_ptr)) + 2) == *(((uint16_t*)(addr2_ptr)) + 2)))

/**
 ****************************************************************************************
 * Compare two MAC addresses whose alignment is not known.
 * @param[in] __a1 Pointer to the first MAC address.
 * @param[in] __a2 Pointer to the second MAC address.
 * @return True if equal, false if not.
 ****************************************************************************************
 */
#define MAC_ADDR_CMP_PACKED(__a1, __a2)                                                 \
    (memcmp(__a1, __a2, MAC_ADDR_LEN) == 0)

/**
 ****************************************************************************************
 * Copy a MAC address.
 * The MAC addresses MUST be 16 bit aligned.
 * @param[in] addr1_ptr Pointer to the destination MAC address.
 * @param[in] addr2_ptr Pointer to the source MAC address.
 ****************************************************************************************
 */
#define MAC_ADDR_CPY(addr1_ptr, addr2_ptr)                                              \
    *(((uint16_t*)(addr1_ptr)) + 0) = *(((uint16_t*)(addr2_ptr)) + 0);                  \
    *(((uint16_t*)(addr1_ptr)) + 1) = *(((uint16_t*)(addr2_ptr)) + 1);                  \
    *(((uint16_t*)(addr1_ptr)) + 2) = *(((uint16_t*)(addr2_ptr)) + 2)

/**
 ****************************************************************************************
 * Compare two SSID.
 * @param[in] ssid1_ptr Pointer to the first SSID structure.
 * @param[in] ssid2_ptr Pointer to the second SSID structure.
 * @return True if equal, false if not.
 ****************************************************************************************
 */
#define MAC_SSID_CMP(ssid1_ptr,ssid2_ptr)                                               \
    (((ssid1_ptr)->length == (ssid2_ptr)->length) &&                                    \
     (memcmp((&(ssid1_ptr)->array[0]), (&(ssid2_ptr)->array[0]), (ssid1_ptr)->length) == 0))

/**
 ****************************************************************************************
 * Check if MAC address is a group address: test the multicast bit.
 * @param[in] mac_addr_ptr Pointer to the MAC address to be checked.
 * @return 0 if unicast address, 1 otherwise
 ****************************************************************************************
 */
#define MAC_ADDR_GROUP(mac_addr_ptr) ((*((uint8_t *)(mac_addr_ptr))) & 1)

#define MAC_ADDR_VALID(mac_addr_ptr) (mac_addr_ptr[0] | mac_addr_ptr[1] | mac_addr_ptr[2] | mac_addr_ptr[3] | mac_addr_ptr[4] | mac_addr_ptr[5])
/// MAC HT operation element
struct mac_htoprnelmt
{
    /// Primary channel information
    uint8_t     prim_channel;
    /// HT operation information 1
    uint8_t     ht_oper_1;
    /// HT operation information 2
    uint16_t    ht_oper_2;
    /// HT operation information 3
    uint16_t    ht_oper_3;
    /// Basic MCS set
    uint8_t     mcs_rate[MAX_MCS_LEN];

};

/// MU EDCA Parameter Set Element
struct mac_mu_edca_param_set
{
    /// Per-AC MU EDCA parameters
    uint32_t        ac_param[AC_MAX];
    /// QoS information
    uint8_t         qos_info;
    /// Parameter set counter
    uint8_t         param_set_cnt;
};

/// EDCA Parameter Set Element
struct mac_edca_param_set
{
    /// Per-AC EDCA parameters
    uint32_t        ac_param[AC_MAX];
    /// QoS information
    uint8_t         qos_info;
    /// Admission Control Mandatory bitfield
    uint8_t         acm;
    /// Parameter set counter
    uint8_t         param_set_cnt;
};

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */
/// Array converting a TID to its associated AC
extern const uint8_t mac_tid2ac[];

/// Array converting an AC to its corresponding bit in the QoS Information field
extern const uint8_t mac_ac2uapsd[];

/// Array converting an ACI to its corresponding AC. See Table 9-136 in 802.11-2016
extern const uint8_t mac_aci2ac[];

/// Array converting an AC to its corresponding ACI. See Table 9-136 in 802.11-2016
extern const uint8_t mac_ac2aci[];

/// Array converting a MAC HW rate id into its corresponding standard rate value
extern const uint8_t mac_id2rate[];

/// Constant value corresponding to the Broadcast MAC address
extern const struct mac_addr mac_addr_bcst;


/**
 ****************************************************************************************
 * Get the value of a bit field in the HE MAC capability element.
 * @param[in] he_cap Pointer to the HE capability structure
 * @param[in] field  Bit field to be read
 * @return The value of the field
 ****************************************************************************************
 */
#define HE_MAC_CAPA_VAL_GET(he_cap, field)    co_val_get((he_cap)->mac_cap_info,         \
                                                         HE_MAC_CAPA_##field##_OFT,      \
                                                         HE_MAC_CAPA_##field##_WIDTH)

/**
 ****************************************************************************************
 * Set the value of a bit field in the HE MAC capability element.
 * @param[in] he_cap Pointer to the HE capability structure
 * @param[in] field Bit field to be written
 * @param[in] val The value of the field
 ****************************************************************************************
 */
#define HE_MAC_CAPA_VAL_SET(he_cap, field, val) co_val_set((he_cap)->mac_cap_info,       \
                                                           HE_MAC_CAPA_##field##_OFT,    \
                                                           HE_MAC_CAPA_##field##_WIDTH,  \
                                                           HE_MAC_CAPA_##field##_##val)

/**
 ****************************************************************************************
 * Test if a bit in the HE MAC capability element is set.
 * @param[in] he_cap Pointer to the HE capability structure
 * @param[in] bit    Bit to be set
 * @return true if set, false otherwise
 ****************************************************************************************
 */
#define HE_MAC_CAPA_BIT_IS_SET(he_cap, bit)   co_bit_is_set((he_cap)->mac_cap_info,      \
                                                            HE_MAC_CAPA_##bit##_POS)

/**
 ****************************************************************************************
 * Set a bit in the HE MAC capability element.
 * @param[in] he_cap Pointer to the HE capability structure
 * @param[in] bit    Bit to be set
 ****************************************************************************************
 */
#define HE_MAC_CAPA_BIT_SET(he_cap, bit)      co_bit_set((he_cap)->mac_cap_info,         \
                                                          HE_MAC_CAPA_##bit##_POS)

/**
 ****************************************************************************************
 * Clear a bit in the HE MAC capability element.
 * @param[in] he_cap Pointer to the HE capability structure
 * @param[in] bit    Bit to be cleared
 ****************************************************************************************
 */
#define HE_MAC_CAPA_BIT_CLR(he_cap, bit)      co_bit_clr((he_cap)->mac_cap_info,         \
                                                         HE_MAC_CAPA_##bit##_POS)

/**
 ****************************************************************************************
 * Get the value of a bit field in the HE PHY capability element.
 * @param[in] he_cap Pointer to the HE capability structure
 * @param[in] field  Bit field to be read
 * @return The value of the field
 ****************************************************************************************
 */
#define HE_PHY_CAPA_VAL_GET(he_cap, field)    co_val_get((he_cap)->phy_cap_info,         \
                                                         HE_PHY_CAPA_##field##_OFT,      \
                                                         HE_PHY_CAPA_##field##_WIDTH)

/**
 ****************************************************************************************
 * Set the value of a bit field in the HE PHY capability element.
 * @param[in] he_cap Pointer to the HE capability structure
 * @param[in] field Bit field to be written
 * @param[in] val The value of the field
 ****************************************************************************************
 */
#define HE_PHY_CAPA_VAL_SET(he_cap, field, val) co_val_set((he_cap)->phy_cap_info,       \
                                                           HE_PHY_CAPA_##field##_OFT,    \
                                                           HE_PHY_CAPA_##field##_WIDTH,  \
                                                           HE_PHY_CAPA_##field##_##val)

/**
 ****************************************************************************************
 * Test if a bit in the HE PHY capability element is set.
 * @param[in] he_cap Pointer to the HE capability structure
 * @param[in] bit    Bit to be set
 * @return true if set, false otherwise
 ****************************************************************************************
 */
#define HE_PHY_CAPA_BIT_IS_SET(he_cap, bit)   co_bit_is_set((he_cap)->phy_cap_info,      \
                                                            HE_PHY_CAPA_##bit##_POS)

/**
 ****************************************************************************************
 * Set a bit in the HE PHY capability element.
 * @param[in] he_cap Pointer to the HE capability structure
 * @param[in] bit    Bit to be set
 ****************************************************************************************
 */
#define HE_PHY_CAPA_BIT_SET(he_cap, bit)      co_bit_set((he_cap)->phy_cap_info,         \
                                                         HE_PHY_CAPA_##bit##_POS)

/**
 ****************************************************************************************
 * Clear a bit in the HE PHY capability element.
 * @param[in] he_cap Pointer to the HE capability structure
 * @param[in] bit    Bit to be cleared
 ****************************************************************************************
 */
#define HE_PHY_CAPA_BIT_CLR(he_cap, bit)      co_bit_clr((he_cap)->phy_cap_info,         \
                                                         HE_PHY_CAPA_##bit##_POS)


/* Sync with llc.h */

/** @name LLC field definitions
 * LLC = DSAP + SSAP + CTRL
 * @note: The 0xAA values indicate the presence of the SNAP
 */
#define LLC_LLC_LEN                  3
#define LLC_DSAP                     0xAA
#define LLC_SSAP                     0xAA
#define LLC_CTRL                     0x03

/** @name SNAP field definitions
 * SNAP = OUI + ETHER_TYPE
 */
#define LLC_SNAP_LEN                 5
#define LLC_OUI_LEN                  3

/** @name 802.2 Header definitions
 * The 802.2 Header is the LLC + the SNAP
 */
#define LLC_802_2_HDR_LEN            (LLC_LLC_LEN + LLC_SNAP_LEN)


/** @name 802.2 Ethernet definitions */
/// Ethernet MTU
#define LLC_ETHER_MTU                1500
/// Size of the Ethernet header
#define LLC_ETHER_HDR_LEN            14
/// Size of the Ethernet type
#define LLC_ETHERTYPE_LEN            2
/// Offset of the Ethernet type within the header
#define LLC_ETHERTYPE_LEN_OFT        12
/// Offset of the EtherType in the 802.2 header
#define LLC_802_2_ETHERTYPE_OFT      6
/// Maximum value of the frame length
#define LLC_FRAMELENGTH_MAXVALUE     0x600

/** @name Useful EtherType values */
#define LLC_ETHERTYPE_APPLETALK      0x809B
#define LLC_ETHERTYPE_IPX            0x8137
#define LLC_ETHERTYPE_AARP           0x80F3
#define LLC_ETHERTYPE_EAP_T          0x888E
#define LLC_ETHERTYPE_802_1Q         0x8100
#define LLC_ETHERTYPE_IP             0x0800
#define LLC_ETHERTYPE_IPV6           0x86DD
#define LLC_ETHERTYPE_ARP            0x0806

/** @name 802_1Q VLAN header definitions */
#define LLC_802_1Q_TCI_VID_MASK       0x0FFF
#define LLC_802_1Q_TCI_PRI_MASK       0xE000
#define LLC_802_1Q_TCI_CFI_MASK       0x1000
#define LLC_802_1Q_HDR_LEN            4
#define LLC_802_1Q_VID_NULL           0
#define LLC_OFFSET_ETHERTYPE          12
#define LLC_OFFSET_IPV4_PRIO          15
#define LLC_OFFSET_802_1Q_TCI         14
/// @}


/*
 * TYPES DEFINITION
 ****************************************************************************************
 */

/// LLC/SNAP structure
struct llc_snap_short
{
    /// DSAP + SSAP fieldsr
    uint16_t dsap_ssap;
    /// LLC control + OUI byte 0
    uint16_t control_oui0;
    /// OUI bytes 1 and 2
    uint16_t oui1_2;
};

/// LLC/SNAP structure
struct llc_snap
{
    /// DSAP + SSAP fieldsr
    uint16_t dsap_ssap;
    /// LLC control + OUI byte 0
    uint16_t control_oui0;
    /// OUI bytes 1 and 2
    uint16_t oui1_2;
    /// Protocol
    uint16_t proto_id;
};


/* Sync with phy.h */

/// 5G lower bound freq
#define PHY_FREQ_5G 5000

/// Macro retrieving the channel of the phy channel info
/// @param[in] __x phy channel info 1 value.
#define PHY_INFO_CHAN(__x) (((__x.info1) & 0xFFFF0000) >> 16)
/// Macro retrieving the HT length of the RX vector
/// @param[in] __x RX bytes 12 - 9 of Receive Vector 1.
#define RXVEC_HTLENGTH_0(__x)    (((__x) & 0x0000FFFF))
/// Macro retrieving the modulation format of the RX vector
/// @param[in] __x bytes 4 - 1 of Receive Vector 1.
#define RXVEC_FORMATMOD(__x)   (((__x) & 0x0000000F))

/**
 ****************************************************************************************
 * @brief Compute the channel number from its center frequency and its band
 * @param[in] band  RF band (must be @ref PHY_BAND_2G4 or @ref PHY_BAND_5G)
 * @param[in] freq  Center frequency of the channel
 *
 * @return The channel number
 ****************************************************************************************
 */
int phy_freq_to_channel(uint8_t band, uint16_t freq);


uint16_t phy_channel_to_freq(uint8_t band, int channel);




#endif // _MAC_H_
