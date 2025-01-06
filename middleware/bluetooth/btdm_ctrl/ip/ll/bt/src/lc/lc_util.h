/**
****************************************************************************************
*
* @file lc_util.h
*
* @brief LC Util API
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

#ifndef LC_UTIL_H_
#define LC_UTIL_H_

/**
 ****************************************************************************************
 * @addtogroup LCUTIL Link Controller
 * @ingroup CONTROLLER
 * @brief Link Layer Controller utility
 *
 * Declaration of the API functions to be called by other layers
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>         // integer
#include <stdbool.h>        // bool
#include "co_bt.h"

#include "ke_msg.h"            // kernel timer definition
/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * CONSTANT DEFINITIONS
 ****************************************************************************************
 */

#define QOS_HL_REQ                  0
#define QOS_PEER_REQ                1
#define QOS_PEER_FORCE              2
#define QOS_PEER_REJECT             3

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function is used to compute the resulting ACL packet types.
 *
 * @param[in] PreferredPacketType      Preferred packet type
 * @param[in] CurrentPacketType        Current packet type in use
 * @param[in] KeepCurrent              Boolean to keep the current packet type
 *
 * @return PacketType
 ****************************************************************************************
 */
uint16_t LM_ComputePacketType(uint16_t PreferredPacketType, uint16_t CurrentPacketType, bool KeepCurrent);

/**
 ****************************************************************************************
 * @brief This function is used to check which ACL packet types are allowed.
 *
 * @param[in] PacketType                ACL packet type
 * @param[in] RemoteCapability          Remote device capability
 *
 * @return PacketType
 ****************************************************************************************
 */
uint16_t LM_UpdateAclPacketType(uint16_t PacketType, uint8_t RemoteCapability);

/**
 ****************************************************************************************
 * @brief This function is used to check which EDR ACL packet types are allowed.
 *
 * @param[in] PacketType                  Requested Packet type
 * @param[in] RemoteCapabilityByte3       Capability of Remote device
 * @param[in] RemoteCapabilityByte4       Capability of Remote device
 * @param[in] RemoteCapabilityByte5       Capability of Remote device
 *
 * @return PossiblePacketType
 ****************************************************************************************
 */
uint16_t LM_UpdateAclEdrPacketType(uint16_t PacketType,
                                   uint8_t RemoteCapabilityByte3,
                                   uint8_t RemoteCapabilityByte4,
                                   uint8_t RemoteCapabilityByte5);

/**
 ****************************************************************************************
 * @brief This function is used to removed ACL packet type.
 *
 * @param[out] ReqPacketType    Possible packet type
 * @param[in]  Slot             Packet slot (1, 3, 5)
 ****************************************************************************************
 */
void LM_SuppressAclPacket(uint16_t *ReqPacketType, uint8_t Slot);

/**
 ****************************************************************************************
 * @brief This function is used to retrieve the Quality of Service information
 *        linked with the connection handle.
 *
 * @param[in]  Role               Device role connection
 * @param[out] ServiceType        Type of service
 * @param[out] TokenRate          Token rate
 * @param[out] PeakBandwidth      Bandwidth peak
 * @param[out] Latency            Latency
 * @param[out] DelayVariation     Variation
 * @param[out] PollInterval       Polling interval. Should be even value.
 * @param[in]  PacketType         Type of packet
 * @param[in]  Request            Request type (HL, PEER, FORCE, REJECT)
 *
 * @return Status
 ****************************************************************************************
 */
uint8_t LM_GetQoSParam(uint8_t Role, uint8_t *ServiceType,
                       uint32_t *TokenRate, uint32_t *PeakBandwidth, uint32_t *Latency,
                       uint32_t *DelayVariation, uint16_t *PollInterval,
                       uint16_t PacketType, uint8_t Request);

/**
 ****************************************************************************************
 * @brief This function make a COF from a BD_ADDR.
 *
 * @param[in]  BdAddr        BD Address
 * @param[out] Aco           ACO value
 ****************************************************************************************
 */
void LM_MakeCof(struct bd_addr BdAddr, struct aco *Aco);

/**
 ****************************************************************************************
 * @brief This function is used to check all the EDR features available.
 *
 * @param[in] RequestFeatures      Feature table
 *
 * @return True if found, False otherwise.
 ****************************************************************************************
 */
bool LM_CheckEdrFeatureRequest(uint16_t RequestFeatures);

/**
 ****************************************************************************************
 * @brief This function is used to get a feature.
 *
 * @param[in] FeaturesPtr          Feature table
 * @param[in] Feature              Feature to extract
 *
 * @return 0 (not found) or 1 (found)
 ****************************************************************************************
 */
uint16_t LM_GetFeature(struct features *FeaturesPtr, uint8_t Feature);

/**
 ****************************************************************************************
 * @brief Converts the Preferred Rate into TX packet type for
 *        the corresponding link ID.
 *
 * @param[in] PreferredRate     Preferred Rate for the link
 *
 * @return Preferred Packet types
 ****************************************************************************************
 */
uint16_t lc_util_convert_pref_rate_to_packet_type(uint8_t PreferredRate);

/**
 ****************************************************************************************
 * @brief Converts the maxslot into TX packet type
 *
 * @param[in] maxslot     Maximum number of slots
 *
 * @return Possible Packet types
 ****************************************************************************************
 */
uint16_t lc_util_convert_maxslot_to_packet_type(uint8_t maxslot);

#if MAX_NB_SYNC
    /**
    ****************************************************************************************
    * @brief This function is used to get the common packet types
    *
    * @param[in] SyncPacketType           Original packets type (SCO and/or eSCO)
    * @param[in] *RemoteFeaturesPtr       Pointer on Remote features
    *
    * @return Common packet types (bit mask)
    ****************************************************************************************
    */
    uint16_t lm_get_common_pkt_types(uint16_t SyncPacketType, struct features *RemoteFeaturesPtr);
#endif // MAX_NB_SYNC

/// @} LC
#endif // LC_UTIL_H_
