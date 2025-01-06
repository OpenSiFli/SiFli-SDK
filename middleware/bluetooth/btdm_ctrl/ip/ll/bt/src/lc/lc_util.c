/**
****************************************************************************************
*
* @file lc_util.c
*
* @brief LC utilities source code
*
* Copyright (C) RivieraWaves 2009-2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LCUTIL
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <string.h>              // for mem* functions
#include "co_endian.h"           // stack endian definition
#include "co_math.h"
#include "co_lmp.h"
#include "ke_timer.h"            // kernel timer definition
#include "ke_mem.h"              // kernel memory definition
#include "co_utils.h"

#include "lc.h"                 // link controller
#include "lc_int.h"             // link controller internal
#include "lc_util.h"            // link controller utilities

#include "ld.h"                 // link driver
#if (EAVESDROPPING_SUPPORT)
    #include "lm.h"                 // link manager
#endif // EAVESDROPPING_SUPPORT

/*
 * DEFINES
 *****************************************************************************************
 */

#define QOS_NO_TRAFFIC_POLL     200


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// Packet size table. Zeros are added for reserved packet and are needed to get the packet size from the packet type flags
__STATIC const uint16_t br_packet_size[16] =
{
    0,
    0,
    0,
    DM1_PACKET_SIZE,
    DH1_PACKET_SIZE,
    0,
    0,
    0,
    DV_ACL_PACKET_SIZE,
    AUX1_PACKET_SIZE,
    DM3_PACKET_SIZE,
    DH3_PACKET_SIZE,
    0,
    0,
    DM5_PACKET_SIZE,
    DH5_PACKET_SIZE
};

///  EDR Packet size table. Zeros are added for reserved packet and are needed to get the packet size from the packet type flags
__STATIC const uint16_t edr_packet_size[16] =
{
    DM1_PACKET_SIZE,
    DH1_2_PACKET_SIZE,
    DH1_3_PACKET_SIZE,
    0,
    0,
    0,
    0,
    0,
    DH3_2_PACKET_SIZE,
    DH3_3_PACKET_SIZE,
    0,
    0,
    DH5_2_PACKET_SIZE,
    DH5_3_PACKET_SIZE,
    0,
    0
};

/*
 *
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */


/*
 * MODULE FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

uint16_t lc_util_get_max_packet_size(uint16_t PacketType)
{
    int index = 0;
    uint16_t NonEdrPacketType = 0, EdrPacketType = 0;
    uint16_t EDR_max_size = 0;
    uint16_t BR_max_size = 0;


    /* For non EDR packet types, remove EDR flags                                   */
    NonEdrPacketType = PacketType & ~(PACKET_TYPE_NO_2_DH1_FLAG |
                                      PACKET_TYPE_NO_3_DH1_FLAG |
                                      PACKET_TYPE_NO_2_DH3_FLAG |
                                      PACKET_TYPE_NO_3_DH3_FLAG |
                                      PACKET_TYPE_NO_2_DH5_FLAG |
                                      PACKET_TYPE_NO_3_DH5_FLAG);
    /* Get only the EDR packet allowed                                              */

    PacketType = PacketType ^ (PACKET_TYPE_NO_2_DH1_FLAG |
                               PACKET_TYPE_NO_3_DH1_FLAG |
                               PACKET_TYPE_NO_2_DH3_FLAG |
                               PACKET_TYPE_NO_3_DH3_FLAG |
                               PACKET_TYPE_NO_2_DH5_FLAG |
                               PACKET_TYPE_NO_3_DH5_FLAG);

    /* For EDR packet types, remove non EDR flags                                   */
    EdrPacketType = PacketType & ~(PACKET_TYPE_DM1_FLAG |
                                   PACKET_TYPE_DH1_FLAG |
                                   PACKET_TYPE_DM3_FLAG |
                                   PACKET_TYPE_DH3_FLAG |
                                   PACKET_TYPE_DM5_FLAG |
                                   PACKET_TYPE_DH5_FLAG);

    if (EdrPacketType)
    {
        for (index = 15 ; index >= 0 ; index --)
        {
            if ((EdrPacketType & (1 << index)) != 0)
            {
                /* If the bit in PacketType is set, set the buffer size             */
                /* corresponding to this type and exit since we need the maximum    */
                /* allowed size                                                     */
                break;
            }
        }
        if (index >= 0)
        {
            EDR_max_size = edr_packet_size[index];
        }
        else
        {
            ASSERT_ERR_FORCE(0);
        }
    }


    if (NonEdrPacketType)
    {
        for (index = 15 ; index >= 0 ; index --)
        {
            if ((NonEdrPacketType & (1 << index)) != 0)
            {
                /* If the bit in PacketType is set, set the buffer size             */
                /* corresponding to this type and exit since we need the maximum    */
                /* allowed size                                                     */
                break;
            }
        }
        if (index >= 0)
        {
            BR_max_size = br_packet_size[index];
        }
        else
        {
            ASSERT_ERR_FORCE(0);
        }
    }
    return (co_max(EDR_max_size, BR_max_size));
}

uint16_t LM_ComputePacketType(uint16_t PreferredPacketType, uint16_t CurrentPacketType, bool KeepCurrent)
{
    uint16_t PacketType;

    /* If KeepCurrent is true and the PreferredPacketType is 0
       then return current packet type
    */

    if ((KeepCurrent) &&
            (PreferredPacketType == 0))
    {
        PacketType = CurrentPacketType;
        return (PacketType);
    }

    /* Compute allowed packet types for non-EDR packets (bit reset=> packet not allowed)*/
    PacketType = (PreferredPacketType & PACKET_TYPE_GFSK_MSK) &
                 (CurrentPacketType & PACKET_TYPE_GFSK_MSK);

    /* Compute allowed packet types for EDR packets (bit set => packet not allowed)     */
    PacketType |= (PreferredPacketType & PACKET_TYPE_EDR_MSK) |
                  (CurrentPacketType & PACKET_TYPE_EDR_MSK);

    return (PacketType);
}

uint16_t LM_UpdateAclPacketType(uint16_t PacketType, uint8_t  RemoteCapability)
{
    uint16_t PossiblePacketType;
    uint8_t  Capability;

    Capability = RemoteCapability;

    /* If Higher Layer request DM1 and/or DH1, allow these packet types                 */
    PossiblePacketType = PacketType & (PACKET_TYPE_DM1_FLAG | PACKET_TYPE_DH1_FLAG);

    /* If Higher Layer request DM3 and both device can manage DM3 => allow DM3          */
    if ((PacketType & PACKET_TYPE_DM3_FLAG) &&
            (Capability & B0_3_SLOT_MSK))
    {
        PossiblePacketType |= PACKET_TYPE_DM3_FLAG;
    }

    /* If Higher Layer request DH3 and both device can manage DH3 => allow DH3          */
    if ((PacketType & PACKET_TYPE_DH3_FLAG) &&
            (Capability & B0_3_SLOT_MSK))
    {
        PossiblePacketType |= PACKET_TYPE_DH3_FLAG;
    }

    /* If Higher Layer request DM5 and both device can manage DM5 => allow DM5          */
    if ((PacketType & PACKET_TYPE_DM5_FLAG) &&
            (Capability & B0_5_SLOT_MSK))
    {
        PossiblePacketType |= PACKET_TYPE_DM5_FLAG;
    }

    /* If Higher Layer request DH5 and both device can manage DH5 => allow DH5          */
    if ((PacketType & PACKET_TYPE_DH5_FLAG) &&
            (Capability & B0_5_SLOT_MSK))
    {
        PossiblePacketType |= PACKET_TYPE_DH5_FLAG;
    }

    return (PossiblePacketType);
}

uint16_t LM_UpdateAclEdrPacketType(uint16_t PacketType,
                                   uint8_t RemoteCapabilityByte3,
                                   uint8_t RemoteCapabilityByte4,
                                   uint8_t RemoteCapabilityByte5)
{
    uint16_t PossiblePacketType;
    uint8_t  CapabilityByte3;
    uint8_t  CapabilityByte4;
    uint8_t  CapabilityByte5;

    CapabilityByte3 = RemoteCapabilityByte3;
    CapabilityByte4 = RemoteCapabilityByte4;
    CapabilityByte5 = RemoteCapabilityByte5;

    PossiblePacketType = 0;

    /* If Higher Layer request 2-DH1 and both device can manage 3-DH3 => allow 3-DH3    */
    if (!(((PacketType & PACKET_TYPE_NO_2_DH1_FLAG) == 0) && (CapabilityByte3 & B3_EDR_2MBPS_ACL_MSK)))
    {
        PossiblePacketType |= PACKET_TYPE_NO_2_DH1_FLAG;
    }
    /* If Higher Layer request 3-DH5 and both device can manage 3-DH3 => allow 3-DH3    */
    if (!(((PacketType & PACKET_TYPE_NO_3_DH1_FLAG) == 0) && (CapabilityByte3 & B3_EDR_3MBPS_ACL_MSK)))
    {
        PossiblePacketType |= PACKET_TYPE_NO_3_DH1_FLAG;
    }
    /* If Higher Layer request 2-DH5 and both device can manage 2-DH3 => allow 3-DH3    */
    if (!(((PacketType & PACKET_TYPE_NO_2_DH3_FLAG) == 0) && (CapabilityByte3 & B3_EDR_2MBPS_ACL_MSK) && (CapabilityByte4 & B4_3_SLOT_EDR_ACL_MSK)))
    {
        PossiblePacketType |= PACKET_TYPE_NO_2_DH3_FLAG;
    }

    /* If Higher Layer request 3-DH5 and both device can manage 3-DH3 => allow 3-DH3    */
    if (!(((PacketType & PACKET_TYPE_NO_3_DH3_FLAG) == 0) && (CapabilityByte3 & B3_EDR_3MBPS_ACL_MSK) && (CapabilityByte4 & B4_3_SLOT_EDR_ACL_MSK)))
    {
        PossiblePacketType |= PACKET_TYPE_NO_3_DH3_FLAG;
    }

    /* If Higher Layer request 2-DH5 and both device can manage 2-DH5 => allow 3-DH5    */
    if (!(((PacketType & PACKET_TYPE_NO_2_DH5_FLAG) == 0) && (CapabilityByte3 & B3_EDR_2MBPS_ACL_MSK) && (CapabilityByte5 & B5_5_SLOT_EDR_ACL_MSK)))
    {
        PossiblePacketType |= PACKET_TYPE_NO_2_DH5_FLAG;
    }
    /* If Higher Layer request 3-DH5 and both device can manage 3-DH5 => allow 3-DH5    */
    if (!(((PacketType & PACKET_TYPE_NO_3_DH5_FLAG) == 0) && (CapabilityByte3 & B3_EDR_3MBPS_ACL_MSK) && (CapabilityByte5 & B5_5_SLOT_EDR_ACL_MSK)))
    {
        PossiblePacketType |= PACKET_TYPE_NO_3_DH5_FLAG;
    }

    return (PossiblePacketType);
}

void LM_SuppressAclPacket(uint16_t *ReqPacketType, uint8_t Slot)
{
    /* First keep only correct ACL packet type                                          */
    *ReqPacketType &= PACKET_TYPE_DM1_FLAG | PACKET_TYPE_DH1_FLAG |
                      PACKET_TYPE_DM3_FLAG | PACKET_TYPE_DH3_FLAG |
                      PACKET_TYPE_DM5_FLAG | PACKET_TYPE_DH5_FLAG |
                      PACKET_TYPE_NO_2_DH5_FLAG | PACKET_TYPE_NO_3_DH5_FLAG |
                      PACKET_TYPE_NO_2_DH3_FLAG | PACKET_TYPE_NO_3_DH3_FLAG |
                      PACKET_TYPE_NO_2_DH1_FLAG | PACKET_TYPE_NO_3_DH1_FLAG;

    /* Check if remove 5 slots                                                          */
    if (Slot == 5)
    {
        /* Remove 5 slots packets                                                       */
        *ReqPacketType &= ~(PACKET_TYPE_DM5_FLAG | PACKET_TYPE_DH5_FLAG);

        *ReqPacketType |= (PACKET_TYPE_NO_2_DH5_FLAG | PACKET_TYPE_NO_3_DH5_FLAG);
    }
    /* Check if remove 3 slots                                                          */
    else if (Slot == 3)
    {
        /* Remove 3 slots packets                                                       */
        *ReqPacketType &= ~(PACKET_TYPE_DM3_FLAG | PACKET_TYPE_DH3_FLAG);

        *ReqPacketType |= (PACKET_TYPE_NO_2_DH3_FLAG | PACKET_TYPE_NO_3_DH3_FLAG);
    }
}

uint8_t LM_MaxSlot(uint16_t PacketType)
{
    uint8_t MaxSlot = 0;

    /* DM1 and DH1 use only 1 slot packet                                               */
    if (((PacketType & (PACKET_TYPE_DM1_FLAG | PACKET_TYPE_DH1_FLAG)) != 0)
            || ((PacketType & PACKET_TYPE_NO_2_DH1_FLAG) == 0)
            || ((PacketType & PACKET_TYPE_NO_3_DH1_FLAG) == 0))
    {
        MaxSlot = 1;
    }

    /* DM3 and DH3 use 3 slot packet                                                    */
    if (((PacketType & (PACKET_TYPE_DM3_FLAG | PACKET_TYPE_DH3_FLAG)) != 0)
            || ((PacketType & PACKET_TYPE_NO_2_DH3_FLAG) == 0)
            || ((PacketType & PACKET_TYPE_NO_3_DH3_FLAG) == 0))
    {
        MaxSlot = 3;
    }

    /* DM5 and DH5 use 5 slot packet                                                    */
    if (((PacketType & (PACKET_TYPE_DM5_FLAG | PACKET_TYPE_DH5_FLAG)) != 0)
            || ((PacketType & PACKET_TYPE_NO_2_DH5_FLAG) == 0)
            || ((PacketType & PACKET_TYPE_NO_3_DH5_FLAG) == 0))
    {
        MaxSlot = 5;
    }

#if (EAVESDROPPING_SUPPORT)
    MaxSlot = co_min(MaxSlot, lm_get_host_max_slot());
#endif // EAVESDROPPING_SUPPORT

    return (MaxSlot);
}

uint8_t LM_GetQoSParam(uint8_t Role,            uint8_t *ServiceType,
                       uint32_t *TokenRate,      uint32_t *PeakBandwidth, uint32_t *Latency,
                       uint32_t *DelayVariation, uint16_t *PollInterval,  uint16_t PacketType, uint8_t Request)
{
    uint32_t Tmpuint32_t = 0;
    uint8_t  RetCode = CO_ERROR_NO_ERROR;

    if (Request == QOS_HL_REQ)
    {
        if (*ServiceType == QOS_NO_TRAFFIC)
        {
            /* If no traffic requested, use a fixed large poll interval                */
            *PollInterval = QOS_NO_TRAFFIC_POLL;
        }
        else if (*ServiceType == QOS_BEST_EFFORT)
        {
            *PollInterval = POLL_INTERVAL_DFT;
        }
        else if (*ServiceType == QOS_GUARANTEED)
        {
            Tmpuint32_t = (uint32_t) lc_util_get_max_packet_size(PacketType);

            if (*TokenRate != 0)
            {
                if (*TokenRate != QOS_WILD_CARD)
                {
                    /* Get the poll interval from packet size and token rate*/
                    Tmpuint32_t = Tmpuint32_t * 1000000 / (*TokenRate * 625);
                    *PollInterval = (uint16_t)Tmpuint32_t;
                    *PollInterval &= 0xFFFE;
                }
                else
                {
                    /* HL request to go as fast than possible                           */
                    *PollInterval = POLL_INTERVAL_DFT;
                }
            }
            else
            {
                /* No TokenRate requested, try to go as fast than possible              */
                *PollInterval = POLL_INTERVAL_DFT;
            }

            /* Check if the poll interval is possible                                   */
            if (*PollInterval < POLL_INTERVAL_MIN)
            {
                *PollInterval = POLL_INTERVAL_MIN;
            }
        }
        else
        {
            RetCode = CO_ERROR_INVALID_HCI_PARAM;
        }

        /* If Latency requested, check if the computed PollInterval match it            */
        if (*Latency != QOS_WILD_CARD)
        {
            /* Convert the delay (microsec) to an even number of slots                  */
            Tmpuint32_t = (uint32_t)(*Latency / 625);
            Tmpuint32_t &= 0xFFFFFFFE;

            /* If to reach the Latency requested, the poll interval need to be decreased*/
            if (Tmpuint32_t < *PollInterval)
            {
                /* Check if it is possible to reach this poll interval                  */
                if (Tmpuint32_t < POLL_INTERVAL_MIN)
                {
                    *PollInterval = POLL_INTERVAL_MIN;
                }
                else
                {
                    *PollInterval = Tmpuint32_t;
                }
            }
        }
    }
    else if (Request == QOS_PEER_REQ)
    {
        /* If remote slave request a poll interval, check if it is allowed             */
        if (Role == MASTER_ROLE)
        {
            if (*PollInterval < POLL_INTERVAL_MIN)
            {
                RetCode = CO_ERROR_INVALID_LMP_PARAM;
                /* Return directly no other processing is needed */
                return (RetCode);
            }
        }

        *PollInterval &= 0xFFFE;

        /* Get the maximum packet size                                                  */
        Tmpuint32_t = (uint32_t) lc_util_get_max_packet_size(PacketType);

        /* Get the token rate from packet size and poll interval*/
        Tmpuint32_t = Tmpuint32_t * 1000000 / (*PollInterval * 625);

        *TokenRate = Tmpuint32_t;

        *PeakBandwidth = 0;                     /* maximum bandwidth is unknown         */
        *Latency = *PollInterval * 625;         /* update latency                         */
        *DelayVariation = QOS_WILD_CARD;        /* do not care                          */
    }
    else if (Request == QOS_PEER_FORCE)
    {
        *PollInterval &= 0xFFFE;

        /* Get the maximum packet size                                                  */
        Tmpuint32_t = (uint32_t) lc_util_get_max_packet_size(PacketType);

        /* Get the token rate from packet size and poll interval*/
        Tmpuint32_t = Tmpuint32_t * 1000000 / (*PollInterval * 625);

        *TokenRate = Tmpuint32_t;

        *PeakBandwidth = 0;                 /* maximum bandwidth is unknown             */
        *Latency = *PollInterval * 625;     /* update latency                              */
        *DelayVariation = QOS_WILD_CARD;    /* do not care                              */
    }
    else if (Request == QOS_PEER_REJECT)
    {
        if (*ServiceType == QOS_BEST_EFFORT)
        {
            *PollInterval = *PollInterval + 2;  /* Try with a bigger poll-Interval      */
            if (*PollInterval > QOS_NO_TRAFFIC_POLL)
            {
                RetCode = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
            }
        }
        else
        {
            RetCode = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
        }
    }
    return (RetCode);
}

void LM_MakeCof(struct bd_addr BdAddr, struct aco *Aco)
{
    uint8_t Index;

    /* COF = BD_ADDR U BD_ADDR                                                          */
    for (Index = 0 ; Index < 6 ; Index++)
    {
        Aco->a[Index] = BdAddr.addr[Index];
        Aco->a[Index + 6] = BdAddr.addr[Index];
    }
}

bool LM_CheckEdrFeatureRequest(uint16_t RequestFeatures)
{
    uint16_t MaskNotEDRFeatures = PACKET_TYPE_NO_2_DH1_FLAG + PACKET_TYPE_NO_3_DH1_FLAG\
                                  + PACKET_TYPE_NO_2_DH3_FLAG + PACKET_TYPE_NO_3_DH3_FLAG\
                                  + PACKET_TYPE_NO_2_DH5_FLAG + PACKET_TYPE_NO_3_DH5_FLAG;
    if ((MaskNotEDRFeatures & RequestFeatures) == MaskNotEDRFeatures)
    {
        return (false);
    }

    return (true);
}

uint16_t LM_GetFeature(struct features *FeaturesPtr, uint8_t Feature)
{
    uint8_t Offset;
    uint8_t Mask;
    uint16_t Result = 0;

    while (Feature >= MAX_FEAT_BITS_PER_PAGE)
    {
        Feature -= MAX_FEAT_BITS_PER_PAGE;
    }
    Offset = Feature >> 3;              /* Compute features byte offset                 */
    Mask = 1 << (Feature & 0x07);       /* Compute features bit mask                    */

    if ((FeaturesPtr->feats[Offset] & Mask) != 0)
    {
        Result = 1;
    }
    else
    {
        Result = 0;
    }
    return Result;
}

uint16_t lc_util_convert_pref_rate_to_packet_type(uint8_t PreferredRate)
{
    /* Always allow DM1 packets                                                         */
    uint16_t PreferredPacketTypes = PACKET_TYPE_DM1_FLAG;

    /* Check for FEC or non FEC packets                                             */
    if ((PreferredRate & FEC_RATE_MSK) == USE_FEC_RATE)
    {
        PreferredPacketTypes |= (PACKET_TYPE_DM3_FLAG |
                                 PACKET_TYPE_DM5_FLAG);
    }
    else
    {
        PreferredPacketTypes |= (PACKET_TYPE_DH1_FLAG |
                                 PACKET_TYPE_DM3_FLAG |
                                 PACKET_TYPE_DH3_FLAG |
                                 PACKET_TYPE_DM5_FLAG |
                                 PACKET_TYPE_DH5_FLAG);
    }
    /* Check the preferred packet size if any                                       */
    if ((PreferredRate & PREF_PACK_MSK) == USE_3_SLOT_PACKET)
    {
        /* Then remove 5 slots packets                                              */
        PreferredPacketTypes &= ~(PACKET_TYPE_DM5_FLAG |
                                  PACKET_TYPE_DH5_FLAG);
    }
    else
    {
        if ((PreferredRate & PREF_PACK_MSK) == USE_1_SLOT_PACKET)
        {
            /* Then remove 5 and 3 slots packets                                    */
            PreferredPacketTypes &= ~(PACKET_TYPE_DM3_FLAG |
                                      PACKET_TYPE_DH3_FLAG |
                                      PACKET_TYPE_DM5_FLAG |
                                      PACKET_TYPE_DH5_FLAG);
        }
    }

    if ((PreferredRate & PREF_EDR_MSK) == USE_DM1_ONLY)
    {
        /* Don't allow EDR packets, i.e. set the NO_FLAG                            */
        PreferredPacketTypes |= (PACKET_TYPE_NO_2_DH1_FLAG |
                                 PACKET_TYPE_NO_3_DH1_FLAG |
                                 PACKET_TYPE_NO_2_DH3_FLAG |
                                 PACKET_TYPE_NO_3_DH3_FLAG |
                                 PACKET_TYPE_NO_2_DH5_FLAG |
                                 PACKET_TYPE_NO_3_DH5_FLAG);
    }
    else
    {
        if ((PreferredRate & PREF_EDR_MSK) == USE_2_MBPS_RATE)
        {
            /* Remove 3 MBps packets (For EDR packets, set the NO_FLAG)             */
            PreferredPacketTypes |= (PACKET_TYPE_NO_3_DH1_FLAG |
                                     PACKET_TYPE_NO_3_DH3_FLAG |
                                     PACKET_TYPE_NO_3_DH5_FLAG);
            /* Check the preferred EDR packet size if any                           */
            if ((PreferredRate & PREF_PACK_EDR_MSK) == USE_3_SLOT_EDR_PKT)
            {
                /* Then remove 5 slots packets (i.e. set the "NO" bit)              */
                PreferredPacketTypes |= PACKET_TYPE_NO_2_DH5_FLAG;
            }
            else
            {
                if ((PreferredRate & PREF_PACK_EDR_MSK) == USE_1_SLOT_EDR_PKT)
                {
                    /* Then remove 5 and 3 slots packets (i.e. set the "NO" bit)    */
                    PreferredPacketTypes |= (PACKET_TYPE_NO_2_DH3_FLAG |
                                             PACKET_TYPE_NO_2_DH5_FLAG);
                }
            }
        }
        else
        {
            /* Remove 2 MBps packets (For EDR packets, set the NO_FLAG)             */
            /* Check the preferred EDR packet size if any                           */
            if ((PreferredRate & PREF_PACK_EDR_MSK) == USE_3_SLOT_EDR_PKT)
            {
                /* Then remove 5 slots packets (i.e. set the "NO" bit)              */
                PreferredPacketTypes |= (PACKET_TYPE_NO_3_DH5_FLAG |
                                         PACKET_TYPE_NO_2_DH5_FLAG);
            }
            else
            {
                if ((PreferredRate & PREF_PACK_EDR_MSK) == USE_1_SLOT_EDR_PKT)
                {
                    /* Then remove 5 and 3 slots packets (i.e. set the "NO" bit)    */
                    PreferredPacketTypes |= (PACKET_TYPE_NO_3_DH3_FLAG |
                                             PACKET_TYPE_NO_3_DH5_FLAG |
                                             PACKET_TYPE_NO_2_DH3_FLAG |
                                             PACKET_TYPE_NO_2_DH5_FLAG);
                }
            }
        }
    }

    return (PreferredPacketTypes);
}

uint16_t lc_util_convert_maxslot_to_packet_type(uint8_t maxslot)
{
    // Initialize with 1-slot packet types
    uint16_t pkt_type  = PACKET_TYPE_DM1_FLAG + PACKET_TYPE_DH1_FLAG
                         + PACKET_TYPE_NO_2_DH3_FLAG + PACKET_TYPE_NO_3_DH3_FLAG
                         + PACKET_TYPE_NO_2_DH5_FLAG + PACKET_TYPE_NO_3_DH5_FLAG;

    if (maxslot == 3)
    {
        // 1/3 slots packet types
        pkt_type = PACKET_TYPE_DM1_FLAG + PACKET_TYPE_DH1_FLAG
                   + PACKET_TYPE_DM3_FLAG + PACKET_TYPE_DH3_FLAG
                   + PACKET_TYPE_NO_2_DH5_FLAG + PACKET_TYPE_NO_3_DH5_FLAG;
    }
    else if (maxslot == 5)
    {
        // 1/3/5 slots packet types
        pkt_type = PACKET_TYPE_DM1_FLAG + PACKET_TYPE_DH1_FLAG
                   + PACKET_TYPE_DM3_FLAG + PACKET_TYPE_DH3_FLAG
                   + PACKET_TYPE_DM5_FLAG + PACKET_TYPE_DH5_FLAG;
    }

    return pkt_type;
}

#if MAX_NB_SYNC
uint16_t lm_get_common_pkt_types(uint16_t SyncPacketType, struct features *RemoteFeaturesPtr)
{
    /* If local or remote device does not support SCO, remove SCO packets types bits    */
    if ((RemoteFeaturesPtr->feats[1] & B1_SCO_MSK) == 0)
    {
        SyncPacketType &= ~(SYNC_PACKET_TYPE_HV1_FLAG |
                            SYNC_PACKET_TYPE_HV2_FLAG |
                            SYNC_PACKET_TYPE_HV3_FLAG);
    }

    // No support of HV2
    SyncPacketType &= ~SYNC_PACKET_TYPE_HV2_FLAG;

    /* If local or remote device does not support HV3, remove HV3 packet type bit */
    if ((RemoteFeaturesPtr->feats[1] & B1_HV3_MSK) == 0)
    {
        SyncPacketType &= ~SYNC_PACKET_TYPE_HV3_FLAG;
    }
    /* If local or remote device does not support eSCO, remove eSCO packets type bits */
    if ((RemoteFeaturesPtr->feats[3] & B3_ESCO_EV3_MSK) == 0)
    {
        SyncPacketType &= ~(SYNC_PACKET_TYPE_EV3_FLAG |
                            SYNC_PACKET_TYPE_EV4_FLAG |
                            SYNC_PACKET_TYPE_EV5_FLAG);
    }
    /* If local or remote device does not support EV4, remove EV4 packet type bit   */
    if ((RemoteFeaturesPtr->feats[4] & B4_EV4_PKT_MSK) == 0)
    {
        SyncPacketType &= ~SYNC_PACKET_TYPE_EV4_FLAG;
    }
    /* If local or remote device does not support EV5, remove EV5 packet type bit */
    if ((RemoteFeaturesPtr->feats[4] & B4_EV5_PKT_MSK) == 0)
    {
        SyncPacketType &= ~SYNC_PACKET_TYPE_EV5_FLAG;
    }
    /* If local or remote device does not support EDR 2Mbps eSCO, remove EV3_2 EV5_2 bits */
    if ((RemoteFeaturesPtr->feats[5] & B5_EDR_ESCO_2MBPS_MSK) == 0)
    {
        SyncPacketType &= ~(SYNC_PACKET_TYPE_EV3_2_FLAG |
                            SYNC_PACKET_TYPE_EV5_2_FLAG);
    }
    /* If local or remote device does not support EDR 3Mbps eSCO, remove EV3_3 EV5_3 bits */
    if ((RemoteFeaturesPtr->feats[5] & B5_EDR_ESCO_3MBPS_MSK) == 0)
    {
        SyncPacketType &= ~(SYNC_PACKET_TYPE_EV3_3_FLAG |
                            SYNC_PACKET_TYPE_EV5_3_FLAG);
    }
    /* If local or remote device does not support EDR 3slots eSCO, remove EV5_3 bits    */
    if ((RemoteFeaturesPtr->feats[5] & B5_3_SLOT_EDR_ESCO_MSK) == 0)
    {
        SyncPacketType &= ~ SYNC_PACKET_TYPE_EV5_3_FLAG;
    }

    return (SyncPacketType);
}
#endif // MAX_NB_SYNC

///@} LC
