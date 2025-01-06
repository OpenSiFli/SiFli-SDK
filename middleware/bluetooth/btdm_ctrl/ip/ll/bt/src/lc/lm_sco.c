/**
****************************************************************************************
*
* @file lm_sco.c
*
* @brief Link Manager synchronous link functions
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup LMSCO
* @{
****************************************************************************************
*/

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"      // stack configuration

#if (MAX_NB_SYNC > 0)

#include <string.h>             // for mem* functions
#include "co_math.h"
#include "co_bt.h"              // stack common definition
#include "co_error.h"           // common error definition
#include "co_utils.h"           // utility definitions
#include "ke_mem.h"

#include "lm_sco.h"             // lm sco definitions

#include "lc.h"
#include "lc_int.h"
#include "lc_util.h"
#include "ld.h"                 // ld definitions
#include "lm.h"             // lm utility definitions
#include "sch_plan.h"           // Scheduling Planner definitions


/*
 * DEFINITIONS
 ****************************************************************************************
 */
/// Number of eSCO packet types
#define ESCO_LINKTYPE_NUM       7
/// Number of eSCO interval (supported)
#define ESCO_INTERVAL_NUM       17

/// eSCO packet indexes
#define EV3_PACKET_INDEX        0
#define EV4_PACKET_INDEX        1
#define EV5_PACKET_INDEX        2
#define EV3_2_PACKET_INDEX      3
#define EV3_3_PACKET_INDEX      4
#define EV5_2_PACKET_INDEX      5
#define EV5_3_PACKET_INDEX      6

/// eSCO maximum interval
#define ESCO_MAX_INTERVAL           36
#define ESCO_WESCO_MAX_1SLOTPKT     4
#define ESCO_WESCO_MIN_1SLOTPKT     2
#define ESCO_WESCO_3SLOTPKT         6
#define ESCO_WESCO_NO_REPETITION    0

/// Max number of negotiation before failure
#define ESCO_NEGOCNT_MAX            10
/*
 * NEW TYPES
 ****************************************************************************************
 */
/// lm synchronous parameters
typedef struct
{
    /// SCO/eSCO offset (in slots)
    uint8_t Offset;
    /// SCO/eSCO Interval (in slots)
    uint8_t Interval;
    /// eSCO retransmission window (in slots)
    uint8_t Window;
    /// SCO/eSCO Master To Slave Packet
    uint8_t PacketM2S;
    /// SCO/eSCO Slave To Master Packet
    uint8_t PacketS2M;
    /// eSCO Master To Slave Length
    uint16_t LenM2S;
    /// eSCO Slave To Master Length
    uint16_t LenS2M;
    /// SCO/eSCO timing Flag
    uint8_t TimingFlag;
    /// Air Mode
    uint8_t AirMode;
    /// Size of a eSCO frame (2, 4 or 6) (in slots)
    uint8_t FrameSize;
} lm_sync_param;

/// lm synchronous configuration
typedef struct
{
    /// SCO_TYPE or ESCO_TYPE
    uint8_t Type;
    /// ACL Link Id for the Link
    uint8_t LinkId;
    /// ScoHandle or eScoHandle
    uint8_t SyncHdl;
    /// Logical Transport address (eSCO only)
    uint8_t LtAddr;
    /// Local and Remote common supported packet type
    uint16_t CommonPacketType;
    /// HL or Peer requested packet type
    uint16_t ReqPacketType;
    /// Master/Slave role
    uint8_t Role;
    /// Sync Voice (content_format)
    uint16_t SyncVoice;
    /// Sync Tx Bandwidth
    uint32_t SyncTxBw;
    /// Sync Rx Bandwidth
    uint32_t SyncRxBw;
    /// eSCO Negotiation State
    uint8_t NegoState;
    /// Sync Latency
    uint16_t Latency;
    /// Sync ReTransmition Effort
    uint8_t ReTx;
    /// current sync parameter settings
    lm_sync_param CurParam;
    /// new sync parameter settings
    lm_sync_param NewParam;
} lm_sync_config;

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

// Sync Configuration
/// sync link configuration
__STATIC lm_sync_config *lm_sync_conf[MAX_NB_SYNC];
/// Current number of active SCO links
__STATIC uint8_t lm_nb_sync_active;
/// boolean sync negotiation
__STATIC bool lm_sync_nego;
/// nb of negotiation
__STATIC uint8_t lm_nego_cnt;
/// maximum negotiation count
__STATIC uint8_t lm_nego_max_cnt;
/// negotiation cntl
__STATIC uint8_t lm_nego_cntl;
/// negotiation for packet used
__STATIC uint16_t lm_nego_pkt_used;

/*
 * INTERNAL FUNCTIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief This function converts a packet type flag (HCI parameter) into a packet
 *        type (LM parameter)
 *
 * @param[in] PacketFlag          Packet type flag from HCI
 *
 * @return Packet type for LM
 *
 ****************************************************************************************
 */
__STATIC uint8_t lm_convert_flag_to_type(uint16_t PacketFlag)
{
    uint8_t PacketType = 0;

    /* Convert packet Type Mask to Packet Type (needed for LMP) */
    switch (PacketFlag)
    {
    case SYNC_PACKET_TYPE_EV3_FLAG:
        PacketType = ESCO_PACKET_EV3;
        break;
    case SYNC_PACKET_TYPE_EV4_FLAG:
        PacketType = ESCO_PACKET_EV4;
        break;
    case SYNC_PACKET_TYPE_EV5_FLAG:
        PacketType = ESCO_PACKET_EV5;
        break;
    case SYNC_PACKET_TYPE_EV3_2_FLAG:
        PacketType = ESCO_PACKET_EV3_2;
        break;
    case SYNC_PACKET_TYPE_EV3_3_FLAG:
        PacketType = ESCO_PACKET_EV3_3;
        break;
    case SYNC_PACKET_TYPE_EV5_2_FLAG:
        PacketType = ESCO_PACKET_EV5_2;
        break;
    case SYNC_PACKET_TYPE_EV5_3_FLAG:
        PacketType = ESCO_PACKET_EV5_3;
        break;
    default:
        break;
    }
    return (PacketType);
}

/*
 ****************************************************************************************
 * @brief This function converts a packet type flag (HCI parameter) into a packet
 *        type (LM parameter)
 *
 * @param[in] PacketType,          Packet type from the LM
 *
 * @returns Packet type flag for the HCI
 *
 ****************************************************************************************
 */
__STATIC uint16_t lm_convert_type_to_flag(uint8_t PacketType)
{
    uint16_t PacketFlag = 0;

    /*
     * Convert packet Type to Packet Type Mask (needed for HCI)
     * NB : If the packet type is set to NULL, the returned flag is EV3 since:
     *          - there is no flag corresponding to NULL packets
     *          - the size of a NULL packet is 1 slot (as the EV3 packet)
     */

    switch (PacketType)
    {
    case ESCO_PACKET_EV3:
        PacketFlag = SYNC_PACKET_TYPE_EV3_FLAG;
        break;
    case ESCO_PACKET_EV4:
        PacketFlag = SYNC_PACKET_TYPE_EV4_FLAG;
        break;
    case ESCO_PACKET_EV5:
        PacketFlag = SYNC_PACKET_TYPE_EV5_FLAG;
        break;
    case ESCO_PACKET_EV3_2:
        PacketFlag = SYNC_PACKET_TYPE_EV3_2_FLAG;
        break;
    case ESCO_PACKET_EV3_3:
        PacketFlag = SYNC_PACKET_TYPE_EV3_3_FLAG;
        break;
    case ESCO_PACKET_EV5_2:
        PacketFlag = SYNC_PACKET_TYPE_EV5_2_FLAG;
        break;
    case ESCO_PACKET_EV5_3:
        PacketFlag = SYNC_PACKET_TYPE_EV5_3_FLAG;
        break;
    case ESCO_PACKET_NULL:
        PacketFlag = SYNC_PACKET_TYPE_EV3_FLAG;
        break;
    default:
        break;
    }

    return (PacketFlag);
}

/*
 ****************************************************************************************
 * @brief This function is used to check that the found frame size and window
 *        retransmission size match the max latency requirement, depending on the
 *        retransmission effort
 *
 * @param[in]  FrameSize                eSCO Frame Size
 * @param[in]  Retransmission_Effort    Retransmission effort
 * @param[in]  Tesco                    eSCO interval
 * @param[out] *Wesco                   Wesco to be updated
 *
 * @return Status
 *
 ****************************************************************************************
 */
__STATIC uint8_t lm_sync_get_best_win_cont(uint8_t FrameSize, uint8_t Retransmission_Effort,
        uint8_t Tesco,     signed int *Wesco)
{
    uint8_t RetCode = CO_ERROR_NO_ERROR;

    /* If no retransmission requested => Window size = 0 */
    if (Retransmission_Effort == SYNC_NO_RE_TX)
    {
        *Wesco = 0;
    }
    else
    {
        if (FrameSize == 2 && Tesco >= 6)
        {
            /*
             * If Tx/Rx packet are both 1 slot (EV3 or NULL)
             * If Quality requested, use up to 2 retransmissions
             */
            if ((*Wesco >= 4) && (Retransmission_Effort == SYNC_RE_TX_QUALITY))
            {
                *Wesco = 4;
            }
            /* Else use up to 1 retransmission  */
            else if (*Wesco >= 2)
            {
                *Wesco = 2;
            }
            else
            {
                if (Retransmission_Effort == SYNC_RE_TX_DONT_CARE)
                {
                    *Wesco = 0;
                }
                else
                {
                    RetCode = CO_ERROR_INVALID_HCI_PARAM;
                }
            }
        }
        else
        {
            /*
             * If the link contains at least one EV4 or EV5 packet
             * Only one repetition supported with multi-slots
             */
            if (*Wesco >= FrameSize)
            {
                *Wesco = FrameSize;
            }
            else
            {
                if (Retransmission_Effort == SYNC_RE_TX_DONT_CARE)
                {
                    *Wesco = 0;
                }
                else
                {
                    RetCode = CO_ERROR_INVALID_HCI_PARAM;
                }
            }
        }
    }

    if (RetCode == CO_ERROR_NO_ERROR)
    {
        /*
         * Check if framesize + Wesco > Tesco (if at least one reTX)
         * Check if framesize + 2 > Tesco (2 slots for ACL traffic if no reTX)
         */
        if ((*Wesco) == 0)
        {
            if ((FrameSize + 2) > Tesco)
            {
                RetCode = CO_ERROR_INVALID_HCI_PARAM;
            }
        }
        else
        {
            if ((FrameSize + (*Wesco)) > Tesco)
            {
                RetCode = CO_ERROR_INVALID_HCI_PARAM;
            }
        }
    }
    return (RetCode);
}

/*
 ****************************************************************************************
 * @brief This function is used to check that the found frame size and window
 *        retransmission size match the max latency requirement, depending on the
 *        retransmission effort
 *
 * @param[in]  FrameSize                 eSCO Frame Size
 * @param[in]  Retransmission_Effort     Retransmission effort
 * @param[in]  Tesco                     eSCO interval
 * @param[out] *Wesco                    eSCO window
 *
 * @return Status
 *
 ****************************************************************************************
 */
__STATIC uint8_t lm_sync_get_best_win(uint8_t FrameSize, uint8_t Retransmission_Effort,
                                      uint8_t Tesco,     signed int *Wesco)
{
    uint8_t RetCode = CO_ERROR_NO_ERROR;

    /*
     * If Tesco > max latency: look for smaller Tesco, this interval cannot be supported
     * so return false status
     */
    if (*Wesco < 0)
    {
        RetCode = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        /* Get the best window size  */
        RetCode = lm_sync_get_best_win_cont(FrameSize,
                                            Retransmission_Effort,
                                            Tesco,
                                            Wesco);

        /* If wrong window size, and if ReTx effort is don't care  */
        if ((RetCode               != CO_ERROR_NO_ERROR) &&
                (Retransmission_Effort == SYNC_RE_TX_DONT_CARE))
        {
            *Wesco = 0; /* Try with Windows = 0   */

            RetCode = lm_sync_get_best_win_cont(FrameSize,
                                                Retransmission_Effort,
                                                Tesco,
                                                Wesco);
        }
    }
    return RetCode;
}

/*
 ****************************************************************************************
 * @brief This function is used to get the size of a sync frame (in slots).
 *
 * @param[in] M2SPacketType         Packets type (SCO or eSCO) M->S
 * @param[in] S2MPacketType         Packets type (SCO or eSCO) S->M
 *
 * @return Synchronous Frame size in slot
 *
 ****************************************************************************************
 */
__STATIC uint8_t lm_get_sync_frame_size(uint8_t M2SPacketType, uint8_t S2MPacketType)
{
    uint8_t FrameSize = 2;

    switch (M2SPacketType)
    {
    case ESCO_PACKET_EV4:
    case ESCO_PACKET_EV5:
    case ESCO_PACKET_EV5_2:
    case ESCO_PACKET_EV5_3:
        FrameSize += 2;
        break;
    default:
        break;
    }
    switch (S2MPacketType)
    {
    case ESCO_PACKET_EV4:
    case ESCO_PACKET_EV5:
    case ESCO_PACKET_EV5_2:
    case ESCO_PACKET_EV5_3:
        FrameSize += 2;
        break;
    default:
        break;
    }
    return FrameSize;
}

/*
 ****************************************************************************************
 * @brief This function computes the number of bytes per frame, depending on the
 *        Tesco and the bandwidth
 *
 * @param[in] Tesco,               eSCO interval
 * @param[in] Bandwidth            Bandwidth
 *
 * @return Number of bytes per frame, or 0 if this number is not an integer
 *
 ****************************************************************************************
 */
__STATIC uint16_t lm_compute_nb_bytes(uint32_t Tesco, uint32_t Bandwidth)
{
    uint32_t NBytesPerSlot, NBytesPerSec;

    NBytesPerSec = Bandwidth * Tesco;
    /*
     * The number of byte per slot is the number of byte per seconds divided by 1600
     * (there are 1600 BT slots in one second)
     */
    NBytesPerSlot = (NBytesPerSec >> 4) / 100;

    /*
     * Check if the found number is an integer, i.e. the found nb * 1600 gives back the
     * correct number of bytes per second
     */
    if ((NBytesPerSlot * 1600) == NBytesPerSec)
    {
        return (NBytesPerSlot);
    }

    return (0);
}

/**
 ****************************************************************************************
 * @brief This function is used to get an offset for the eSCO connection
 *
 * @param[in]  sco_link_id      synchronous connection link identifier
 * @param[in]  SyncInterval     synchronous interval (in slots)
 * @param[out] *SyncOffset   eSCO offset (in slots)
 * @param[in]  FrameSize        frame size for eSCO link (in slots)
 * @param[in]  SyncWindow       Retransmission window size (in slots)
 * @param[in]  timing_flag      Timing initialization flag
 ****************************************************************************************
 */
__STATIC void lm_get_sync_offset(uint8_t sco_link_id, uint8_t SyncInterval, uint8_t *SyncOffset, uint8_t FrameSize, uint8_t SyncWindow, uint8_t timing_flag)
{
    struct sch_plan_req_param req_param;
    uint16_t Size;
    uint16_t offset = 0;
    uint8_t LinkId = lm_sync_conf[sco_link_id]->LinkId;
    uint32_t clk_off = ld_acl_clock_offset_get(LinkId).hs;
    uint32_t clk = ld_read_clock();

    Size = FrameSize + SyncWindow;

    // Request parameters to the planner
    req_param.interval_min = SyncInterval * 2;
    req_param.interval_max = SyncInterval * 2;
    req_param.duration_min = FrameSize * 2;
    req_param.duration_max = Size * 2;
    req_param.offset_min   = 0;
    req_param.offset_max   = req_param.interval_max - 1;
    req_param.conhdl       = BT_SYNC_CONHDL(LinkId, sco_link_id);
    req_param.conhdl_ref   = BT_ACL_CONHDL_MIN + LinkId;
    req_param.pref_period  = 0;
    req_param.margin       = 1 * (lm_sync_conf[sco_link_id]->Role == SLAVE_ROLE);

    if (sch_plan_req(&req_param) == SCH_PLAN_ERROR_OK)
    {
        offset = req_param.offset_min;
    }

    // Convert local offset to d_esco
    {
        uint32_t master_clock = CLK_ADD_2(clk, clk_off);
        bool init = master_clock & (1 << 27);
        uint8_t sync_offset = lc_offset_local_to_lmp(offset, req_param.interval, clk, clk_off, init);

        // Convert to even master slot
        sync_offset = CO_ALIGN4_HI(sync_offset) / 2;
        *SyncOffset = sync_offset;
    }
}

/*
 ****************************************************************************************
 * @brief This function is used to get the possible Tesco depending on the given
 *        bandwidth, retransmission effort and allowed packet types
 *
 * @param[in]  Bandwidth                Required bandwidth
 * @param[in]  Retransmission_Effort    Retransmission effort
 * @param[out] *PktType_Mask            Allowed packet types
 * @param[out] *TescoIndex_BitMask      Tesco bitmask to be updated
 *
 * @return Status
 *
 ****************************************************************************************
 */
__STATIC uint8_t lm_extract_tesco_bitmask(uint32_t Bandwidth,     uint32_t Retransmission_Effort,
        uint16_t *PktType_Mask, uint32_t *TescoIndex_BitMask)
{
    uint32_t Tesco;
    int i;
    uint16_t NBytes = 0;

    uint8_t RetCode = CO_ERROR_INVALID_HCI_PARAM;

    /* Loop on Tesco index (i=0 <=> Tesco=36 ... i=16 <=> Tesco=4 )  */
    for (i = 0; i < ESCO_INTERVAL_NUM ; i++)
    {
        /*
         * Calculate Tesco from the current index
         *      Tesco = 36 - (index*2)
         */
        Tesco = ESCO_MAX_INTERVAL - (i << 1);

        /*
         * Compute the number of bytes per frame according to the
         * calculated Tesco and the given bandwidth
         */
        NBytes = lm_compute_nb_bytes(Tesco, Bandwidth);

        /* If an integer number of bytes has been calculated   */
        if (NBytes != 0)
        {
            if (NBytes > EV5_3_PACKET_SIZE)
            {
                /*
                 * If the given size is bigger than the maximum size allowed (3-EV5 size),
                 * do not set any bit in the bitmask
                 */
            }
            else if (NBytes > EV3_3_PACKET_SIZE)
            {
                /*
                 * If the given size is bigger than the 3-EV3 size, set a bit in :
                 *     - 3-EV5 (if 3-EV5 allowed) or
                 *     - 2-EV5 (if 2-EV5 allowed and length < 360) or
                 *     - EV5 (if EV5 allowed and (120<length<180 or retx=power) or
                 *     - EV4 (if EV4 allowed and length < 120)
                 */
                if ((*PktType_Mask & SYNC_PACKET_TYPE_EV5_3_FLAG) != 0)
                {
                    TescoIndex_BitMask[EV5_3_PACKET_INDEX] |= (1 << i);
                    RetCode = CO_ERROR_NO_ERROR;
                }
                else if ((NBytes <= EV5_2_PACKET_SIZE) &&
                         ((*PktType_Mask & SYNC_PACKET_TYPE_EV5_2_FLAG) != 0))
                {
                    TescoIndex_BitMask[EV5_2_PACKET_INDEX] |= (1 << i);
                    RetCode = CO_ERROR_NO_ERROR;
                }
                else if ((NBytes <= EV5_PACKET_SIZE) &&
                         ((*PktType_Mask & SYNC_PACKET_TYPE_EV5_FLAG) != 0))
                {
                    TescoIndex_BitMask[EV5_PACKET_INDEX] |= (1 << i);
                    RetCode = CO_ERROR_NO_ERROR;
                }
                else if (NBytes <= EV4_PACKET_SIZE)
                {
                    if (Retransmission_Effort == SYNC_RE_TX_POWER)
                    {
                        /*
                         * If the retransmission effort is "optimize for power consumption",
                         * we first try to set the bit in the EV5 bitmask (EV5 are shorter
                         * than EV4 packets since there is no FEC)
                         * If EV5 are not allowed, we set the bit in the EV4 bitmask
                         */
                        if ((*PktType_Mask & SYNC_PACKET_TYPE_EV5_FLAG) != 0)
                        {
                            TescoIndex_BitMask[EV5_PACKET_INDEX] |= (1 << i);
                            RetCode = CO_ERROR_NO_ERROR;
                        }
                        else if ((*PktType_Mask & SYNC_PACKET_TYPE_EV4_FLAG) != 0)
                        {
                            TescoIndex_BitMask[EV4_PACKET_INDEX] |= (1 << i);
                            RetCode = CO_ERROR_NO_ERROR;
                        }
                    }
                    else
                    {
                        /*
                         * If the retransmission effort is NOT "optimize for power
                         * consumption", we first try to set the bit in the EV4 bitmask (EV4
                         * have a better quality than EV5 packets since there is FEC)
                         * If EV4 are not allowed, we set the bit in the EV5 bitmask
                         */
                        if ((*PktType_Mask & SYNC_PACKET_TYPE_EV4_FLAG) != 0)
                        {
                            TescoIndex_BitMask[EV4_PACKET_INDEX] |= (1 << i);
                            RetCode = CO_ERROR_NO_ERROR;
                        }
                        else if ((*PktType_Mask & SYNC_PACKET_TYPE_EV5_FLAG) != 0)
                        {
                            TescoIndex_BitMask[EV5_PACKET_INDEX] |= (1 << i);
                            RetCode = CO_ERROR_NO_ERROR;
                        }
                    }
                }
            }
            else /* if(NBytes < EV3_3_PACKET_SIZE) */
            {
                /*
                 * If the given size is lower than the 3-EV3 size, set a bit in :
                 *     - 3-EV3 (if 3-EV3 allowed) or
                 *     - 2-EV3 (if 2-EV3 allowed and length < 60) or
                 *     - EV3 (if EV3 allowed and length<30)
                 *     - 3-EV5 (if 3-EV5 allowed) or
                 *     - 2-EV5 (if 2-EV5 allowed) or
                 *     - EV5 (if EV5 allowed and retx=power) or
                 *     - EV4 (if EV4 allowed and retx=quality)
                 */
                if ((*PktType_Mask & SYNC_PACKET_TYPE_EV3_3_FLAG) != 0)
                {
                    TescoIndex_BitMask[EV3_3_PACKET_INDEX] |= (1 << i);
                    RetCode = CO_ERROR_NO_ERROR;
                }
                else if ((NBytes <= EV3_2_PACKET_SIZE) &&
                         ((*PktType_Mask & SYNC_PACKET_TYPE_EV3_2_FLAG) != 0))
                {
                    TescoIndex_BitMask[EV3_2_PACKET_INDEX] |= (1 << i);
                    RetCode = CO_ERROR_NO_ERROR;
                }
                else if ((NBytes <= EV3_PACKET_SIZE) &&
                         ((*PktType_Mask & SYNC_PACKET_TYPE_EV3_FLAG) != 0))
                {
                    TescoIndex_BitMask[EV3_PACKET_INDEX] |= (1 << i);
                    RetCode = CO_ERROR_NO_ERROR;
                }
                else if ((*PktType_Mask & SYNC_PACKET_TYPE_EV5_3_FLAG) != 0)
                {
                    TescoIndex_BitMask[EV5_3_PACKET_INDEX] |= (1 << i);
                    RetCode = CO_ERROR_NO_ERROR;
                }
                else if ((*PktType_Mask & SYNC_PACKET_TYPE_EV5_2_FLAG) != 0)
                {
                    TescoIndex_BitMask[EV5_2_PACKET_INDEX] |= (1 << i);
                    RetCode = CO_ERROR_NO_ERROR;
                }
                else
                {
                    /*
                     * If the given size is between an the EV3 and an EV4 size, the set bit
                     * depends on the retransmission effort
                     */
                    if (Retransmission_Effort == SYNC_RE_TX_POWER)
                    {
                        /*
                         * If the retransmission effort is "optimize for power consumption",
                         * we first try to set the bit in the EV5 bitmask (EV5 are shorter
                         * than EV4 packets since there is no FEC)
                         * If EV5 are not allowed, we set the bit in the EV4 bitmask
                         */
                        if ((*PktType_Mask & SYNC_PACKET_TYPE_EV5_FLAG) != 0)
                        {
                            TescoIndex_BitMask[EV5_PACKET_INDEX] |= (1 << i);
                            RetCode = CO_ERROR_NO_ERROR;
                        }
                        else if ((*PktType_Mask & SYNC_PACKET_TYPE_EV4_FLAG) != 0)
                        {
                            TescoIndex_BitMask[EV4_PACKET_INDEX] |= (1 << i);
                            RetCode = CO_ERROR_NO_ERROR;
                        }
                    }
                    else
                    {
                        /*
                         * If the retransmission effort is NOT "optimize for power
                         * consumption", we first try to set the bit in the EV4 bitmask (EV4
                         * have a better quality than EV5 packets since there is FEC)
                         * If EV4 are not allowed, we set the bit in the EV5 bitmask
                         */
                        if ((*PktType_Mask & SYNC_PACKET_TYPE_EV4_FLAG) != 0)
                        {
                            TescoIndex_BitMask[EV4_PACKET_INDEX] |= (1 << i);
                            RetCode = CO_ERROR_NO_ERROR;
                        }
                        else if ((*PktType_Mask & SYNC_PACKET_TYPE_EV5_FLAG) != 0)
                        {
                            TescoIndex_BitMask[EV5_PACKET_INDEX] |= (1 << i);
                            RetCode = CO_ERROR_NO_ERROR;
                        }
                    }
                }
            }
        }
    }
    return RetCode;
}

/*
 ****************************************************************************************
 * @brief This function check if the length is compatible with the packet type.
 *
 * @param[in] Length               Packet Length
 * @param[in] PacketMask           Packet type flag from HCI
 *
 * @return Packet type for LM
 *****************************************************************************************
 */
__STATIC uint8_t lm_check_packet_len(uint16_t Length, uint16_t PacketMask)
{
    uint8_t RetCode = CO_ERROR_INVALID_LMP_PARAM;

    if (PacketMask & SYNC_PACKET_TYPE_EV3_FLAG)
    {
        if (Length <= EV3_PACKET_SIZE)
        {
            RetCode = CO_ERROR_NO_ERROR;
        }
    }
    if (PacketMask & SYNC_PACKET_TYPE_EV4_FLAG)
    {
        if (Length <= EV4_PACKET_SIZE)
        {
            RetCode = CO_ERROR_NO_ERROR;
        }
    }
    if (PacketMask & SYNC_PACKET_TYPE_EV5_FLAG)
    {
        if (Length <= EV5_PACKET_SIZE)
        {
            RetCode = CO_ERROR_NO_ERROR;
        }
    }
    if (PacketMask & SYNC_PACKET_TYPE_EV3_2_FLAG)
    {
        if (Length <= EV3_2_PACKET_SIZE)
        {
            RetCode = CO_ERROR_NO_ERROR;
        }
    }
    if (PacketMask & SYNC_PACKET_TYPE_EV3_3_FLAG)
    {
        if (Length <= EV3_3_PACKET_SIZE)
        {
            RetCode = CO_ERROR_NO_ERROR;
        }
    }
    if (PacketMask & SYNC_PACKET_TYPE_EV5_2_FLAG)
    {
        if (Length <= EV5_2_PACKET_SIZE)
        {
            RetCode = CO_ERROR_NO_ERROR;
        }
    }
    if (PacketMask & SYNC_PACKET_TYPE_EV5_3_FLAG)
    {
        if (Length <= EV5_3_PACKET_SIZE)
        {
            RetCode = CO_ERROR_NO_ERROR;
        }
    }
    return RetCode;
}

/*
 ****************************************************************************************
 * @brief This Function Checks all the current voice links, and tells if BandWidth
 *        is sufficient to create a new link.
 *
 * @param[in]  Index               synchronous index
 * @param[in]  Tesco               synchronous interval
 * @param[in]  FrameSize           length of the frame
 * @param[out] *Weso               synchronous window
 *
 * @return Status
 *
 ****************************************************************************************
 */
__STATIC uint8_t lm_check_esco_bandwidth(uint8_t Index, uint8_t Tesco, uint8_t Framesize, int *Wesco)
{
    uint8_t Status = CO_ERROR_NO_ERROR;

    uint8_t MinInterval, MaxInterval, ReservedSlot;
    uint8_t ReTxSlot, NumMasterLink, NumSlaveLink;

    /* Loop on Sync links to get the Minimum Interval, the Maximum Interval, the number */
    /* of Reserved Slot and the number of ReTransmission slots                           */
    MinInterval = 0xFF;
    MaxInterval = 0x00;
    ReservedSlot = 0;
    ReTxSlot = 0;
    NumMasterLink = lm_get_nb_acl(MASTER_FLAG);
    NumSlaveLink = lm_get_nb_acl(SLAVE_FLAG);

    for (int i = 0 ; i < MAX_NB_SYNC ; i++)
    {
        if ((lm_sync_conf[i] != NULL) && (i != Index))
        {
            MinInterval = co_min(MinInterval, lm_sync_conf[i]->CurParam.Interval);
            MaxInterval = co_max(MaxInterval, lm_sync_conf[i]->CurParam.Interval);

            ReservedSlot += lm_sync_conf[i]->CurParam.FrameSize;

            ReTxSlot += lm_sync_conf[i]->CurParam.Window;
        }
    }
    if (MinInterval == 0xFF)
    {
        MinInterval = Tesco;
        MaxInterval = Tesco;
    }

    do
    {
        if (NumMasterLink && NumSlaveLink)
        {
            /* Means we are in  a scatternet  */
            if (MinInterval >= (ReservedSlot + ReTxSlot + Framesize + *Wesco))
            {
                if ((MinInterval - (ReservedSlot + ReTxSlot + Framesize + *Wesco)) == 0)
                {
                    Status = CO_ERROR_INVALID_HCI_PARAM;
                }
            }
            else
            {
                Status = CO_ERROR_INVALID_HCI_PARAM;
            }
        }
        else
        {
            if (MinInterval < (ReservedSlot + ReTxSlot + Framesize + *Wesco))
            {
                Status = CO_ERROR_INVALID_HCI_PARAM;
            }
        }
        if (Status == CO_ERROR_NO_ERROR)
        {
            break;
        }
        /* Try with Lower Windows only if its Dont care or No Retx   */
        else if ((Status == CO_ERROR_INVALID_HCI_PARAM) &&
                 ((*Wesco) && ((lm_sync_conf[Index]->ReTx == SYNC_NO_RE_TX) ||
                               (lm_sync_conf[Index]->ReTx == SYNC_RE_TX_DONT_CARE)
                              )
                 )
                )
        {
            Status = CO_ERROR_NO_ERROR;
            *Wesco = 0;
        }
    }
    while (Status == CO_ERROR_NO_ERROR);

    return (Status);
}

/*
 ****************************************************************************************
 * @brief This function is used to get the eSCO parameters depending on the given
 *        bandwidths and previously computed Tesco bitmasks
 *
 * @param[in]  SyncConHdl          Synchronous link connection handle
 * @param[out] *SyncInterval    Returned Tesco value
 * @param[out] *SyncPacketM2S   Returned M->S packet type
 * @param[out] *SyncPacketS2M   Returned M->S packet type
 * @param[out] *SyncLenM2S      Returned M->S packet length
 * @param[out] *SyncLenS2M      Returned M->S packet length
 * @param[out] *SyncWindow      Returned Wesco value
 * @param[out] *TxTescoBitMask     Tesco bitmask for TX side
 * @param[out] *RxTescoBitMask     Tesco bitmask for RX side
 *
 * @return Status
 ****************************************************************************************
 */
__STATIC uint8_t lm_extract_esco_params(uint8_t sco_link_id,       uint8_t *SyncInterval,
                                        uint8_t *SyncPacketM2S, uint8_t *SyncPacketS2M,
                                        uint16_t *SyncLenM2S,   uint16_t *SyncLenS2M,
                                        uint8_t *SyncWindow,    uint32_t *TxTescoBitMask,
                                        uint32_t *RxTescoBitMask)
{
    uint32_t Max_Latency = 0;
    int FrameSize = 0;
    int ShiftIndex = 0;
    int TypeIndex = 0;
    int TypeIndexTx = 0;
    int TypeIndexRx = 0;
    int Wesco = 0;
    uint16_t Tx_NBytes = 0;
    uint16_t Rx_NBytes = 0;
    uint16_t TxPacketTypeMsk = 0;
    uint16_t RxPacketTypeMsk = 0;
    uint16_t Tesco = 0;
    uint8_t  TxPacketType = 0;
    uint8_t  RxPacketType = 0;
    uint8_t  RetCode = CO_ERROR_INVALID_HCI_PARAM;

    /*
     * Compute Max Latency in number of Slot.
     * The higher layers give the latency in ms and one BT slot is 625 us
     */
    Max_Latency = ((uint32_t)(lm_sync_conf[sco_link_id]->Latency) * 1000) / 625;

    /* Start a loop on the packet types, starting with the 3-EV5 packet type down to EV3 */
    for (TypeIndex = (ESCO_LINKTYPE_NUM - 1) ;
            (TypeIndex >= 0) && (RetCode != CO_ERROR_NO_ERROR) ;
            TypeIndex--)
    {
        /*
         * Test the bits of the possible Tesco bitmask for the given link type.
         * We start with the biggest Tesco.
         */
        for (ShiftIndex = 0 ;
                (ShiftIndex < ESCO_INTERVAL_NUM) && (RetCode != CO_ERROR_NO_ERROR) ;
                ShiftIndex++)
        {
            /* Compute Tesco from the shift index   */
            Tesco = ESCO_MAX_INTERVAL - (ShiftIndex << 1);

            /* Check if the interval is supported for TX direction */
            if ((TxTescoBitMask[TypeIndex] & (1 << ShiftIndex)) != 0)
            {
                if (lm_sync_conf[sco_link_id]->SyncRxBw == 0)
                {
                    /*
                     * If the interval is supported for TX direction AND the RX
                     * bandwidth is null, then check directly this found Tesco
                     */
                    /* Compute the number of bytes per frame in Tx   */
                    Tx_NBytes = lm_compute_nb_bytes(Tesco, lm_sync_conf[sco_link_id]->SyncTxBw);
                    TxPacketTypeMsk = 1 << (TypeIndex + 3);
                    Rx_NBytes = 0;
                    RxPacketTypeMsk = 0;

                    RetCode = lm_check_packet_len(Tx_NBytes, TxPacketTypeMsk);
                    if (RetCode == CO_ERROR_NO_ERROR)
                    {
                        /*
                         * Compute the frame size of this eSCO with RX type set NULL
                         * packets (1 slot)
                         */
                        FrameSize = lm_get_sync_frame_size
                                    (lm_convert_flag_to_type(TxPacketTypeMsk),
                                     ESCO_PACKET_NULL);
                        /*
                         * Compute the maximum retransmission window length in number of
                         * slots. The formula is : Latency = Tesco + FrameSize + Wesco
                         */
                        Wesco = Max_Latency - Tesco - FrameSize;
                        /* Compute the best retransmission window  */
                        RetCode = lm_sync_get_best_win(FrameSize,
                                                       lm_sync_conf[sco_link_id]->ReTx,
                                                       Tesco,
                                                       &Wesco);
                    }
                }
                else
                {
                    /* For each RX packet type    */
                    for (TypeIndexRx = (ESCO_LINKTYPE_NUM - 1) ;
                            (TypeIndexRx >= 0) && (RetCode != CO_ERROR_NO_ERROR) ;
                            TypeIndexRx--)
                    {
                        /* Test if interval is supported in Rx direction */
                        if ((RxTescoBitMask[TypeIndexRx] & (1 << ShiftIndex)) != 0)
                        {
                            /*
                             * If interval is supported by both directions, calculate the
                             * link parameters
                             */

                            /* Compute the number of bytes per frame in Tx              */
                            Tx_NBytes = lm_compute_nb_bytes(Tesco, lm_sync_conf[sco_link_id]->SyncTxBw);
                            TxPacketTypeMsk = 1 << (TypeIndex + 3);

                            /* Compute the number of bytes per frame in Rx              */
                            Rx_NBytes = lm_compute_nb_bytes(Tesco, lm_sync_conf[sco_link_id]->SyncRxBw);
                            RxPacketTypeMsk = 1 << (TypeIndexRx + 3);

                            RetCode = lm_check_packet_len(Tx_NBytes, TxPacketTypeMsk);
                            if (RetCode == CO_ERROR_NO_ERROR)
                            {
                                RetCode = lm_check_packet_len(Rx_NBytes, RxPacketTypeMsk);
                            }

                            if (RetCode == CO_ERROR_NO_ERROR)
                            {
                                /* Compute the frame size of this eSCO                  */
                                FrameSize = lm_get_sync_frame_size
                                            (lm_convert_flag_to_type(TxPacketTypeMsk),
                                             lm_convert_flag_to_type(RxPacketTypeMsk));
                                /*
                                 * Compute the maximum retransmission window length in nb
                                 * of slots.
                                 * The formula is : Latency = Tesco + FrameSize + Wesco
                                 */
                                Wesco = Max_Latency - Tesco - FrameSize;
                                /* Compute the best retransmission window               */
                                RetCode = lm_sync_get_best_win(FrameSize,
                                                               lm_sync_conf[sco_link_id]->ReTx,
                                                               Tesco,
                                                               &Wesco);
                            }
                        }
                    }
                }
            }
            else
            {
                /*
                 * If the interval is not supported in TX for this packet type, check
                 * if the interval is supported in RX direction for this packet type
                 */
                if ((RxTescoBitMask[TypeIndex] & (1 << ShiftIndex)) != 0)
                {
                    if (lm_sync_conf[sco_link_id]->SyncTxBw == 0)
                    {
                        /*
                         * If the interval is supported for RX direction AND the TX
                         * bandwidth is null, then check directly this found Tesco
                         */
                        Tx_NBytes = 0;
                        TxPacketTypeMsk = 0;
                        /* Compute the number of bytes per frame in RX    */
                        Rx_NBytes = lm_compute_nb_bytes(Tesco, lm_sync_conf[sco_link_id]->SyncRxBw);
                        RxPacketTypeMsk = 1 << (TypeIndex + 3);

                        RetCode = lm_check_packet_len(Rx_NBytes, RxPacketTypeMsk);
                        if (RetCode == CO_ERROR_NO_ERROR)
                        {
                            /*
                             * Compute the frame size of this eSCO with TX type set NULL
                             * packets (1 slot)
                             */
                            FrameSize = lm_get_sync_frame_size
                                        (ESCO_PACKET_NULL,
                                         lm_convert_flag_to_type(RxPacketTypeMsk));
                            /*
                             * Compute the maximum retransmission window length in nb of
                             * slots. The formula is : Latency = Tesco + FrameSize + Wesco
                             */
                            Wesco = Max_Latency - Tesco - FrameSize;
                            /* Compute the best retransmission window   */
                            RetCode = lm_sync_get_best_win(FrameSize,
                                                           lm_sync_conf[sco_link_id]->ReTx,
                                                           Tesco,
                                                           &Wesco);
                        }
                    }
                    else
                    {
                        for (TypeIndexTx = (ESCO_LINKTYPE_NUM - 1) ;
                                (TypeIndexTx >= 0) && (RetCode != CO_ERROR_NO_ERROR) ;
                                TypeIndexTx--)
                        {
                            /* Test if interval is supported in TX direction            */
                            if ((TxTescoBitMask[TypeIndexTx] & (1 << ShiftIndex)) != 0)
                            {
                                /*
                                 * If interval is supported by both directions, calculate
                                 * the link parameters
                                 */

                                /* Compute the number of bytes per frame in Tx          */
                                Tx_NBytes = lm_compute_nb_bytes(Tesco, lm_sync_conf[sco_link_id]->SyncTxBw);
                                TxPacketTypeMsk = 1 << (TypeIndexTx + 3);

                                /* Compute the number of bytes per frame in Rx          */
                                Rx_NBytes = lm_compute_nb_bytes(Tesco, lm_sync_conf[sco_link_id]->SyncRxBw);
                                RxPacketTypeMsk = 1 << (TypeIndex + 3);

                                RetCode = lm_check_packet_len(Tx_NBytes, TxPacketTypeMsk);
                                if (RetCode == CO_ERROR_NO_ERROR)
                                {
                                    RetCode = lm_check_packet_len(Rx_NBytes, RxPacketTypeMsk);
                                }

                                if (RetCode == CO_ERROR_NO_ERROR)
                                {
                                    /* Compute the frame size of this eSCO              */
                                    FrameSize = lm_get_sync_frame_size
                                                (lm_convert_flag_to_type(TxPacketTypeMsk),
                                                 lm_convert_flag_to_type(RxPacketTypeMsk));
                                    /*
                                     * Compute the maximum retransmission window length in
                                     * nb of slots.
                                     * The formula is: Latency = Tesco + FrameSize + Wesco
                                     */
                                    Wesco = Max_Latency - Tesco - FrameSize;
                                    /* Compute the best retransmission window           */
                                    RetCode = lm_sync_get_best_win(FrameSize,
                                                                   lm_sync_conf[sco_link_id]->ReTx,
                                                                   Tesco,
                                                                   &Wesco);
                                }
                            }
                        }
                    }
                }
            }
        } /* for ShiftIndex */
    } /* for TypeIndex */

    if (RetCode == CO_ERROR_NO_ERROR)
    {
        *SyncInterval = Tesco;
        *SyncWindow = (uint8_t)Wesco;
        /* Convert Tx packet Type Mask to Packet Type (needed for LMP)  */
        TxPacketType = lm_convert_flag_to_type(TxPacketTypeMsk);

        /* Convert Rx packet Type Mask to Packet Type (needed for LMP) */
        RxPacketType = lm_convert_flag_to_type(RxPacketTypeMsk);

        if (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE)
        {
            *SyncPacketM2S = TxPacketType;
            *SyncPacketS2M = RxPacketType;
            *SyncLenM2S = Tx_NBytes;
            *SyncLenS2M = Rx_NBytes;
        }
        else
        {
            *SyncPacketM2S = RxPacketType;
            *SyncPacketS2M = TxPacketType;
            *SyncLenM2S = Rx_NBytes;
            *SyncLenS2M = Tx_NBytes;
        }
    }

    return (RetCode);
}
/*
 ****************************************************************************************
 * @brief This function is used to check packet parameters
 *
 * @param[in] SyncType           Type of the SyncLink
 * @param[in] SyncPacket         PacketType to check
 * @param[in] SyncPacketLen      PacketLenght to check (ESCO_TYPE only)
 * @param[in] CommonPacketType   Common packets types
 *
 * @return Status
 ****************************************************************************************
 */
__STATIC uint8_t lm_sync_check_pkt(uint8_t SyncType,       uint8_t  SyncPacket,
                                   uint16_t SyncPacketLen, uint16_t CommonPacketType)
{
    /* If this is a SCO link    */
    if (SyncType == SCO_TYPE)
    {
        /* If HV3 requested, Check if HV3 allowed  */
        if ((SyncPacket == SCO_PACKET_HV3) &&
                ((CommonPacketType & SYNC_PACKET_TYPE_HV3_FLAG) == 0))
        {
            return CO_ERROR_SCO_INTERVAL_REJECTED;
        }
        // HV2 not supported
        else if (SyncPacket == SCO_PACKET_HV2)
        {
            return CO_ERROR_SCO_INTERVAL_REJECTED;
        }
        /* If HV1 requested, Check if HV1 allowed     */
        else if (SyncPacket == SCO_PACKET_HV1)
        {
            if ((CommonPacketType & SYNC_PACKET_TYPE_HV1_FLAG) == 0)
            {
                return CO_ERROR_SCO_INTERVAL_REJECTED;
            }
            /* A HV1 can be added only if there is one ACL link    */
            if (lm_get_nb_acl(MASTER_FLAG | SLAVE_FLAG) > 1)
            {
                return CO_ERROR_SCO_INTERVAL_REJECTED;
            }
        }
    }
    /* Else, it is a eSCO link     */
    else
    {
        /* If EV3 requested     */
        if (SyncPacket == ESCO_PACKET_EV3)
        {
            /* Check if EV3 allowed and is length fit in EV3   */
            if (((CommonPacketType & SYNC_PACKET_TYPE_EV3_FLAG) == 0) ||
                    (SyncPacketLen > EV3_PACKET_SIZE))
            {
                return CO_ERROR_INVALID_LMP_PARAM;
            }
        }
        /* If EV4 requested   */
        else if (SyncPacket == ESCO_PACKET_EV4)
        {
            /* Check if EV4 allowed and is length fit in EV4  */
            if (((CommonPacketType & SYNC_PACKET_TYPE_EV4_FLAG) == 0) ||
                    (SyncPacketLen > EV4_PACKET_SIZE))
            {
                return CO_ERROR_INVALID_LMP_PARAM;
            }
        }
        /* If EV5 requested     */
        else if (SyncPacket == ESCO_PACKET_EV5)
        {
            /* Check if EV5 allowed and is length fit in EV5   */
            if (((CommonPacketType & SYNC_PACKET_TYPE_EV5_FLAG) == 0) ||
                    (SyncPacketLen > EV5_PACKET_SIZE))
            {
                return CO_ERROR_INVALID_LMP_PARAM;
            }
        }
        /* If EV3_2 requested     */
        else if (SyncPacket == ESCO_PACKET_EV3_2)
        {
            /* Check if EV3_2 allowed and is length fit in EV3_2    */
            if (((CommonPacketType & SYNC_PACKET_TYPE_EV3_2_FLAG) == 0) ||
                    (SyncPacketLen > EV3_2_PACKET_SIZE))
            {
                return CO_ERROR_INVALID_LMP_PARAM;
            }
        }
        /* If EV3_3 requested  */
        else if (SyncPacket == ESCO_PACKET_EV3_3)
        {
            /* Check if EV3_3 allowed and is length fit in EV3_3 */
            if (((CommonPacketType & SYNC_PACKET_TYPE_EV3_3_FLAG) == 0) ||
                    (SyncPacketLen > EV3_3_PACKET_SIZE))
            {
                return CO_ERROR_INVALID_LMP_PARAM;
            }
        }
        /* If EV5_2 requested   */
        else if (SyncPacket == ESCO_PACKET_EV5_2)
        {
            /* Check if EV5_2 allowed and is length fit in EV5_2    */
            if (((CommonPacketType & SYNC_PACKET_TYPE_EV5_2_FLAG) == 0) ||
                    (SyncPacketLen > EV5_2_PACKET_SIZE))
            {
                return CO_ERROR_INVALID_LMP_PARAM;
            }
        }
        /* If EV5_3 requested   */
        else if (SyncPacket == ESCO_PACKET_EV5_3)
        {
            /* Check if EV5_3 allowed and is length fit in EV5_3 */
            if (((CommonPacketType & SYNC_PACKET_TYPE_EV5_3_FLAG) == 0) ||
                    (SyncPacketLen > EV5_3_PACKET_SIZE))
            {
                return CO_ERROR_INVALID_LMP_PARAM;
            }
        }
        /* If NULL/POLL requested   */
        else if (SyncPacket == ESCO_PACKET_NULL)
        {
            /* Check if length is null  */
            if (SyncPacketLen != 0)
            {
                return CO_ERROR_INVALID_LMP_PARAM;
            }
        }
    }
    return CO_ERROR_NO_ERROR;
}

/*
 ****************************************************************************************
 * @brief This function is used to check Sync parameters
 *
 * @param[in] SyncType               Type of the SyncLink
 * @param[in] SyncPacketM2S          Master to Slave Packet
 * @param[in] SyncPacketS2M          Slave to Master Packet
 * @param[in] SyncWindow             Retransmission windows
 * @param[in] SyncInterval           Interval
 * @param[in] SyncOffset             Offset
 *
 * @returns Status
 ****************************************************************************************
 */
__STATIC uint8_t lm_sync_check_win(uint8_t SyncType,   uint8_t SyncPacketM2S, uint8_t SyncPacketS2M,
                                   uint8_t SyncWindow, uint8_t SyncInterval,  uint8_t SyncOffset)
{
    /* The Sync Offset must be even     */
    if ((SyncOffset & 0x01) != 0)
    {
        return CO_ERROR_INVALID_LMP_PARAM;
    }

    /* The Sync Interval must be even         */
    if ((SyncInterval & 0x01) != 0)
    {
        return CO_ERROR_INVALID_LMP_PARAM;
    }

    /* If This is a SCO Link              */
    if (SyncType == SCO_TYPE)
    {
        /* If this is a HV1 link      */
        if (SyncPacketM2S == SCO_PACKET_HV1)
        {
            /* Interval must be 2 and offset must be 0    */
            if ((SyncInterval != TSCO_HV1) ||
                    (SyncOffset   != 0))
            {
                return CO_ERROR_INVALID_LMP_PARAM;
            }
        }
        /* If this is a HV3 link    */
        else if (SyncPacketM2S == SCO_PACKET_HV3)
        {
            /* Interval must be 6 and offset must be less or equal to 4   */
            if ((SyncInterval != TSCO_HV3) ||
                    (SyncOffset   >  4))
            {
                return CO_ERROR_INVALID_LMP_PARAM;
            }
        }
    }
    /* If This is a eSCO Link */
    else
    {
        /* The Retransmission Windows must be even  */
        if ((SyncWindow & 0x01) != 0)
        {
            return CO_ERROR_INVALID_LMP_PARAM;
        }
    }
    return CO_ERROR_NO_ERROR;
}
/*
 ****************************************************************************************
 * @brief This function is used to check Sync parameters.
 *
 * @param[in] SyncType            Sync type
 * @param[in] SyncInterval        Tsco or Tesco
 * @param[in] SyncOffset,         Dsco or Desco
 * @param[in] SyncPacketM2S,      Packet M->S type
 * @param[in] SyncPacketS2M,      Packet S->M type
 * @param[in] SyncLenM2S,         Packet M->S length for eSCO link
 * @param[in] SyncLenS2M,         Packet S->M length for eSCO link
 * @param[in] SyncWindow,         Retransmission window size
 * @param[in] CommonPacketType    Common packets types
 *
 * @returns Status
 *
 ****************************************************************************************
 */
__STATIC uint8_t lm_sync_check_params(uint8_t Index,         uint8_t SyncType,
                                      uint8_t SyncPacketM2S, uint8_t SyncPacketS2M,
                                      uint16_t SyncLenM2S,   uint16_t SyncLenS2M,
                                      uint8_t SyncInterval,  uint8_t SyncOffset,
                                      uint8_t SyncWindow,    uint16_t CommonPacketType)
{
    uint8_t RetCode = CO_ERROR_NO_ERROR;

    /* If current Sync link is SCO */
    if (SyncType == SCO_TYPE)
    {
        if (SyncPacketM2S != SyncPacketS2M)
        {
            return CO_ERROR_UNSPECIFIED_ERROR;
        }
        /* Check if requested packet is allowed */
        RetCode = lm_sync_check_pkt(SCO_TYPE,
                                    SyncPacketM2S,
                                    0,
                                    CommonPacketType);

        if (RetCode != CO_ERROR_NO_ERROR)
        {
            return RetCode;
        }

        /* Check Windows, Interval, and offset parameters */
        RetCode = lm_sync_check_win(SCO_TYPE,
                                    SyncPacketM2S,
                                    SyncPacketS2M,
                                    0,
                                    SyncInterval,
                                    SyncOffset);
        if (RetCode != CO_ERROR_NO_ERROR)
        {
            return RetCode;
        }
    }
    /* If remote device request eSCO */
    else
    {
        /* Check if Requested Interval is even */
        if ((SyncInterval & 0x01) != 0)
        {
            return CO_ERROR_INVALID_LMP_PARAM;
        }

        /* Check Master to Slave packet type and Length  */
        RetCode = lm_sync_check_pkt(ESCO_TYPE,
                                    SyncPacketM2S,
                                    SyncLenM2S,
                                    CommonPacketType);

        if (RetCode == CO_ERROR_NO_ERROR)
        {
            /* Check Slave to Master packet type and Length  */
            RetCode = lm_sync_check_pkt(ESCO_TYPE,
                                        SyncPacketS2M,
                                        SyncLenS2M,
                                        CommonPacketType);
            if (RetCode != CO_ERROR_NO_ERROR)
            {
                return RetCode;
            }
        }
        else
        {
            return RetCode;
        }

        /* Check Windows, Interval, and offset parameters */
        RetCode = lm_sync_check_win(ESCO_TYPE,
                                    SyncPacketM2S,
                                    SyncPacketS2M,
                                    SyncWindow,
                                    SyncInterval,
                                    SyncOffset);

        if (RetCode != CO_ERROR_NO_ERROR)
        {
            return RetCode;
        }
    } /* eSCO */

    return RetCode;
}

/*
 ****************************************************************************************
 * @brief This function is used to check if requested packet are allowed by host.
 *
 * @param[in] Index              Sync Index
 *
 * @returns Status
 ****************************************************************************************
 */
__STATIC uint8_t lm_sync_check_pktAllowed(uint8_t SyncType,      uint8_t SyncPacketM2S,
        uint8_t SyncPacketS2M, uint16_t SyncReqPacketType)
{
    uint8_t RetCode = CO_ERROR_NO_ERROR;

    /* If remote device request an SCO link */
    if (SyncType == SCO_TYPE)
    {
        /* If remote device request HV1  */
        if (SyncPacketM2S == SCO_PACKET_HV1)
        {
            /* But is Host does not allow this packet type */
            if ((SyncReqPacketType & SYNC_PACKET_TYPE_HV1_FLAG) == 0)
            {
                RetCode = CO_ERROR_SCO_INTERVAL_REJECTED;
            }
        }

        /* If remote device request HV2  */
        if (SyncPacketM2S == SCO_PACKET_HV2)
        {
            /* Controller does not support this packet type  */
            RetCode = CO_ERROR_SCO_INTERVAL_REJECTED;
        }

        /* If remote device request HV3  */
        if (SyncPacketM2S == SCO_PACKET_HV3)
        {
            /* But is Host does not allow this packet type  */
            if ((SyncReqPacketType & SYNC_PACKET_TYPE_HV3_FLAG) == 0)
            {
                RetCode = CO_ERROR_SCO_INTERVAL_REJECTED;
            }
        }
    }
    /* If remote device request an eSCO link  */
    else
    {
        /* If remote device request EV3 in any direction  */
        if ((SyncPacketM2S == ESCO_PACKET_EV3) ||
                (SyncPacketS2M == ESCO_PACKET_EV3))
        {
            /* But is Host does not allow this packet type  */
            if ((SyncReqPacketType & SYNC_PACKET_TYPE_EV3_FLAG) == 0)
            {
                /* Packet must be re-negotiated */
                RetCode = CO_ERROR_INVALID_LMP_PARAM;
            }
        }

        /* If remote device request EV4 in any direction */
        if ((SyncPacketM2S == ESCO_PACKET_EV4) ||
                (SyncPacketS2M == ESCO_PACKET_EV4))
        {
            /* But if Host does not allow this packet type  */
            if ((SyncReqPacketType & SYNC_PACKET_TYPE_EV4_FLAG) == 0)
            {
                /* Packet must be re-negotiated  */
                RetCode = CO_ERROR_INVALID_LMP_PARAM;
            }
        }

        /* If remote device request EV5 in any direction  */
        if ((SyncPacketM2S == ESCO_PACKET_EV5) ||
                (SyncPacketS2M == ESCO_PACKET_EV5))
        {
            /* But if Host does not allow this packet type */
            if ((SyncReqPacketType & SYNC_PACKET_TYPE_EV5_FLAG) == 0)
            {
                /* Packet must be re-negotiated */
                RetCode = CO_ERROR_INVALID_LMP_PARAM;
            }
        }

        /* If remote device request EV3_2 in any direction */
        if ((SyncPacketM2S == ESCO_PACKET_EV3_2) ||
                (SyncPacketS2M == ESCO_PACKET_EV3_2))
        {
            /* But if Host does not allow this packet type */
            if ((SyncReqPacketType & SYNC_PACKET_TYPE_EV3_2_FLAG) == 0)
            {
                /* Packet must be re-negotiated */
                RetCode = CO_ERROR_INVALID_LMP_PARAM;
            }
        }

        /* If remote device request EV3_3 in any direction */
        if ((SyncPacketM2S == ESCO_PACKET_EV3_3) ||
                (SyncPacketS2M == ESCO_PACKET_EV3_3))
        {
            /* But if Host does not allow this packet type */
            if ((SyncReqPacketType & SYNC_PACKET_TYPE_EV3_3_FLAG) == 0)
            {
                /* Packet must be re-negotiated */
                RetCode = CO_ERROR_INVALID_LMP_PARAM;
            }
        }

        /* If remote device request EV5_2 in any direction */
        if ((SyncPacketM2S == ESCO_PACKET_EV5_2) ||
                (SyncPacketS2M == ESCO_PACKET_EV5_2))
        {
            /* But if Host does not allow this packet type */
            if ((SyncReqPacketType & SYNC_PACKET_TYPE_EV5_2_FLAG) == 0)
            {
                /* Packet must be re-negotiated */
                RetCode = CO_ERROR_INVALID_LMP_PARAM;
            }
        }

        /* If remote device request EV5_3 in any direction */
        if ((SyncPacketM2S == ESCO_PACKET_EV5_3) ||
                (SyncPacketS2M == ESCO_PACKET_EV5_3))
        {
            /* But if Host does not allow this packet type */
            if ((SyncReqPacketType & SYNC_PACKET_TYPE_EV5_3_FLAG) == 0)
            {
                /* Packet must be re-negotiated */
                RetCode = CO_ERROR_INVALID_LMP_PARAM;
            }
        }

    }
    return RetCode;
}


/*
 ****************************************************************************************
 * @brief This function is used to get the total number of packet types
 *        supported
 *
 * @param[in] SyncPacket        synchronous packet type
 *
 * @returns Total bits set
 ****************************************************************************************
 */
__STATIC uint8_t lm_get_total_pkt_type(uint16_t SyncPacket)
{
    SyncPacket = (0x5555UL & SyncPacket) + (0x5555UL & (SyncPacket >> 1));
    SyncPacket = (0x3333UL & SyncPacket) + (0x3333UL & (SyncPacket >> 2));
    SyncPacket = (0x0f0fUL & SyncPacket) + (0x0f0fUL & (SyncPacket >> 4));
    SyncPacket = (0x00ffUL & SyncPacket) + (0x00ffUL & (SyncPacket >> 8));
    return ((uint8_t)SyncPacket);
}


/**
 ****************************************************************************************
 * @brief This function is used to get the eSCO parameters when the request or
 *        the accept comes from the higher layers
 *
 * @param[in]  SyncConHdl          synchronous connection handle
 * @param[out] *SyncInterval    Interval (in slots)
 * @param[out] *SyncOffset      eSCO offset (in slots)
 * @param[out] *SyncPacketM2S   Packet M->S type for eSCO link,
 * @param[out] *SyncPacketS2M   Packet S->M type for eSCO link,
 * @param[out] *SyncLenM2S      Packet M->S length for eSCO link,
 * @param[out] *SyncLenS2M      Packet S->M length for eSCO link,
 * @param[out] *SyncWindow      Retransmission window size (in slots)
 * @param[out] *TimingFlag      Timing flags
 *
 * @brief Status
 ****************************************************************************************
 */
__STATIC uint8_t lm_get_esco_params(uint8_t sco_link_id,       uint8_t *SyncInterval,
                                    uint8_t *SyncOffset,    uint8_t *SyncPacketM2S,
                                    uint8_t *SyncPacketS2M, uint16_t *SyncLenM2S,
                                    uint16_t *SyncLenS2M,   uint8_t *SyncWindow, uint8_t *TimingFlag)
{
    uint8_t FrameSize;
    uint8_t RetCode = CO_ERROR_NO_ERROR;
    /*
     * TescoIndex_BitMask = Bit mask that flags the possible Tesco for each packet type
     * Available Tesco are: 36, 34, 32, ..., 16, 14, 12, 10, 8 and 6.
     *              Bit 0 <=> Tesco = 36 .... Bit 16 <=> Tesco = 4
     * When the bit is set in the bitmask, this means the Tesco gives an integer number of
     * bytes per slot for a given bandwidth
     * The table index meaning depends on the packet table (1 Mbps or 2/3 Mbps)
     * The table indexes are as follow:
     *          - index 0 : Bitmask for matching Tesco for EV3 packet type
     *          - index 1 : Bitmask for matching Tesco for EV4 packet type
     *          - index 2 : Bitmask for matching Tesco for EV5 packet type
     *          - index 3 : Bitmask for matching Tesco for 2-EV3 packet type
     *          - index 4 : Bitmask for matching Tesco for 3-EV3 packet type
     *          - index 5 : Bitmask for matching Tesco for 2-EV5 packet type
     *          - index 6:  Bitmask for matching Tesco for 3-EV5 packet type
     */
    uint32_t     Tx_TescoBitMask[ESCO_LINKTYPE_NUM] = {0, 0, 0, 0, 0, 0, 0};
    uint32_t     Rx_TescoBitMask[ESCO_LINKTYPE_NUM] = {0, 0, 0, 0, 0, 0, 0};
    uint16_t     Packet_Type;

    /* Copy Allowed packet type in a local variable                                     */
    Packet_Type = lm_sync_conf[sco_link_id]->ReqPacketType;

    /* First extract the Tesco Bitmask for TX side parameters                           */
    if (lm_sync_conf[sco_link_id]->SyncTxBw != 0)
    {
        /*
         * If the TX bandwidth is not null, compute the matching Tesco bitmask
         * If the TX bandwidth is null, let the TX Tesco bitmask to all zeroes
         */
        RetCode = lm_extract_tesco_bitmask(lm_sync_conf[sco_link_id]->SyncTxBw,
                                           lm_sync_conf[sco_link_id]->ReTx,
                                           &Packet_Type,
                                           Tx_TescoBitMask);
    }
    if (RetCode == CO_ERROR_NO_ERROR)
    {
        /* Then extract the Tesco Bitmask for RX side parameters                        */
        if (lm_sync_conf[sco_link_id]->SyncRxBw != 0)
        {
            /*
             * If the RX bandwidth is not null, compute the matching Tesco bitmask
             * If the RX bandwidth is null, let the RX Tesco bitmask to all zeroes
             */
            RetCode = lm_extract_tesco_bitmask(lm_sync_conf[sco_link_id]->SyncRxBw,
                                               lm_sync_conf[sco_link_id]->ReTx,
                                               &Packet_Type,
                                               Rx_TescoBitMask);
        }
    }

    if (RetCode == CO_ERROR_NO_ERROR)
    {
        /*
         * From the resulting Tesco bitmasks, extract ESCO parameters and check if they
         * are compliant with Max latency
         */
        RetCode = lm_extract_esco_params(sco_link_id,
                                         SyncInterval,
                                         SyncPacketM2S,
                                         SyncPacketS2M,
                                         SyncLenM2S,
                                         SyncLenS2M,
                                         SyncWindow,
                                         Tx_TescoBitMask,
                                         Rx_TescoBitMask);
    }

    /* Check if these is not bandwidth conflict between several links               */
    if (RetCode == CO_ERROR_NO_ERROR)
    {
        FrameSize = lm_get_sync_frame_size(*SyncPacketM2S, *SyncPacketS2M);
        // Get an offset for the synchronous link
        lm_get_sync_offset(sco_link_id, *SyncInterval, SyncOffset, FrameSize, *SyncWindow, *TimingFlag);
    }
    return (RetCode);
}

/**
 ****************************************************************************************
 * @brief This function is used to get parameters for a SCO link
 *
 * @param[in]  ConHdl             Connection handle of the SCO link
 * @param[out] *ScoInterval    SCO interval (in slots)
 * @param[out] *ScoOffset      SCO offset (in slots)
 * @param[out] *ScoPacket      SCO packet
 * @param[out] *TimingFlag     Timing flags
 *
 * @returns Status
 ****************************************************************************************
 */
__STATIC uint8_t lm_get_sco_param_hl_req(uint8_t sco_link_id,    uint8_t *SyncInterval,
        uint8_t *SyncOffset, uint8_t *SyncPacket, uint8_t *TimingFlag)
{
    uint8_t RetCode = CO_ERROR_SYNC_CON_LIMIT_DEV_EXCEED;

    if ((lm_sync_conf[sco_link_id]->SyncRxBw != SCO_BANDWIDTH) &&
            (lm_sync_conf[sco_link_id]->SyncTxBw != SCO_BANDWIDTH))
    {
        return CO_ERROR_UNSUPPORTED;
    }

    /* If HL enable HV3 and if Interval 6 is allowed                                    */
    if ((lm_sync_conf[sco_link_id]->ReqPacketType & SYNC_PACKET_TYPE_HV3_FLAG) != 0)
    {
        *SyncPacket = SCO_PACKET_HV3;        /* Set SCO Packet                   */
        *SyncInterval = TSCO_HV3;                /* Set TSCO                     */
        RetCode = CO_ERROR_NO_ERROR;
    }
    /* Else If HL enable HV1 and if Interval 2 is allowed                               */
    else if ((lm_sync_conf[sco_link_id]->ReqPacketType & SYNC_PACKET_TYPE_HV1_FLAG) != 0)
    {
        /* A HV1 can be added only if there is one ACL link                             */
        if (lm_get_nb_acl(MASTER_FLAG | SLAVE_FLAG) == 1)
        {
            *SyncPacket = SCO_PACKET_HV1;    /* Set SCO Packet                       */
            *SyncInterval = TSCO_HV1;                /* Set TSCO                     */
            RetCode = CO_ERROR_NO_ERROR;
        }
        else
        {
            RetCode = CO_ERROR_SCO_INTERVAL_REJECTED;
        }
    }
    else
    {
        RetCode = CO_ERROR_UNSUPPORTED;
    }

    /* If the SCO can be added, setup parameters (interval and delay)                   */
    if (RetCode == CO_ERROR_NO_ERROR)
    {
        // Get an offset for the synchronous link
        lm_get_sync_offset(sco_link_id, *SyncInterval, SyncOffset, 2, 0, *TimingFlag);
    }
    return (RetCode);
}

/**
 ****************************************************************************************
 * @brief This function is used to update parameters for a synchronous link.
 *
 * @param[in]  SyncConHdl         Connection handle of the SCO link
 * @param[out] *SyncInterval   Tsco or Tesco
 * @param[out] *SyncOffset     eSCO offset (in slots)
 * @param[out] *SyncAirMode    Air mode
 * @param[out] *SyncLenM2S     Packet M->S length for eSCO link
 * @param[out] *SyncLenS2M     Packet S->M length for eSCO link
 * @param[out] *SyncWindow     Retransmission window size
 * @param[out] *TimingFlag     Timing flags
 ****************************************************************************************
 */
__STATIC void lm_get_sync_param_update(uint8_t sco_link_id,
                                       uint8_t *SyncInterval,  uint8_t *SyncOffset,
                                       uint8_t *SyncAirMode,
                                       uint16_t *SyncLenM2S, uint16_t *SyncLenS2M,   uint8_t *SyncWindow,
                                       uint8_t *TimingFlag)
{
    /* If This is a SCO link  */
    if (lm_sync_conf[sco_link_id]->Type == SCO_TYPE)
    {
        *SyncWindow = 0;
        *SyncLenM2S = 0;
        *SyncLenS2M = 0;
    }

    /* Copy New Parameters in current parameters */
    lm_sync_conf[sco_link_id]->CurParam.Offset     = lm_sync_conf[sco_link_id]->NewParam.Offset;
    lm_sync_conf[sco_link_id]->CurParam.Interval   = lm_sync_conf[sco_link_id]->NewParam.Interval;
    lm_sync_conf[sco_link_id]->CurParam.AirMode    = lm_sync_conf[sco_link_id]->NewParam.AirMode;
    lm_sync_conf[sco_link_id]->CurParam.TimingFlag = lm_sync_conf[sco_link_id]->NewParam.TimingFlag;

    lm_sync_conf[sco_link_id]->CurParam.PacketM2S  = lm_sync_conf[sco_link_id]->NewParam.PacketM2S;
    lm_sync_conf[sco_link_id]->CurParam.PacketS2M  = lm_sync_conf[sco_link_id]->NewParam.PacketS2M;
    lm_sync_conf[sco_link_id]->CurParam.LenM2S     = lm_sync_conf[sco_link_id]->NewParam.LenM2S;
    lm_sync_conf[sco_link_id]->CurParam.LenS2M     = lm_sync_conf[sco_link_id]->NewParam.LenS2M;

    lm_sync_conf[sco_link_id]->CurParam.FrameSize  = lm_sync_conf[sco_link_id]->NewParam.FrameSize;
    lm_sync_conf[sco_link_id]->CurParam.Window     = lm_sync_conf[sco_link_id]->NewParam.Window;

    /* Return current parameters  */
    *SyncOffset   = lm_sync_conf[sco_link_id]->CurParam.Offset;
    *SyncInterval = lm_sync_conf[sco_link_id]->CurParam.Interval;
    *SyncAirMode  = lm_sync_conf[sco_link_id]->CurParam.AirMode;
    *TimingFlag   = lm_sync_conf[sco_link_id]->CurParam.TimingFlag;

    lm_sync_nego = false;  // End of SCO negotiation
}

/**
 ****************************************************************************************
 * @brief This function is used to get parameters for a Sync link if the request
 *        comes from the Higher layers
 *
 * @param[in]  sco_link_id     Synchronous link ID
 * @param[out] *SyncType       SCO or eSCO
 * @param[out] sco_params      SCO parameters
 *
 * @returns Status
 ****************************************************************************************
 */
__STATIC uint8_t lm_get_sync_param_hl_req(uint8_t sco_link_id, uint8_t *SyncType, struct lc_sco_air_params_tag *sco_params)
{
    uint8_t RetCode = CO_ERROR_NO_ERROR;
    uint16_t VoiceSetting;

    /* If EVx packet not allowed => only SCO allowed                                    */
    if ((lm_sync_conf[sco_link_id]->ReqPacketType & (SYNC_PACKET_TYPE_EV3_FLAG |
            SYNC_PACKET_TYPE_EV4_FLAG |
            SYNC_PACKET_TYPE_EV5_FLAG |
            SYNC_PACKET_TYPE_EV3_2_FLAG |
            SYNC_PACKET_TYPE_EV3_3_FLAG |
            SYNC_PACKET_TYPE_EV5_2_FLAG |
            SYNC_PACKET_TYPE_EV5_3_FLAG)) == 0)
    {
        RetCode = lm_get_sco_param_hl_req(sco_link_id,
                                          &sco_params->t_esco,
                                          &sco_params->d_esco,
                                          &sco_params->m2s_pkt_type,
                                          &sco_params->flags);
        if (RetCode == CO_ERROR_NO_ERROR)
        {
            /* This is a SCO type                                                       */
            sco_params->s2m_pkt_type = sco_params->m2s_pkt_type;
            *SyncType = SCO_TYPE;
            sco_params->w_esco = 0;
            lm_sync_conf[sco_link_id]->Type = SCO_TYPE;
            lm_sync_conf[sco_link_id]->NewParam.FrameSize = 2;
        }
        else
        {
            return RetCode;
        }
    }
    /* If at least a EVx allowed                                                        */
    else
    {
        /* Try first with eSCO                                                          */
        RetCode = lm_get_esco_params(sco_link_id,
                                     &sco_params->t_esco,
                                     &sco_params->d_esco,
                                     &sco_params->m2s_pkt_type,
                                     &sco_params->s2m_pkt_type,
                                     &sco_params->m2s_pkt_len,
                                     &sco_params->s2m_pkt_len,
                                     &sco_params->w_esco,
                                     &sco_params->flags);

        if (RetCode == CO_ERROR_NO_ERROR)
        {
            /* This is a eSCO type                                                      */
            *SyncType = ESCO_TYPE;
            lm_sync_conf[sco_link_id]->Type = ESCO_TYPE;
            lm_sync_conf[sco_link_id]->NewParam.FrameSize = lm_get_sync_frame_size(sco_params->m2s_pkt_type,
                    sco_params->s2m_pkt_type);
            /* Store the Negotiation Count from the Total Packet types that are
               supported*/
            lm_nego_max_cnt = lm_get_total_pkt_type(lm_sync_conf[sco_link_id]->ReqPacketType);
        }
        else
        {
            /* If SCO allowed                                                           */
            if ((lm_sync_conf[sco_link_id]->ReqPacketType & (SYNC_PACKET_TYPE_HV1_FLAG |
                    SYNC_PACKET_TYPE_HV2_FLAG |
                    SYNC_PACKET_TYPE_HV3_FLAG)) != 0)
            {
                RetCode = lm_get_sco_param_hl_req(sco_link_id,
                                                  &sco_params->t_esco,
                                                  &sco_params->d_esco,
                                                  &sco_params->m2s_pkt_type,
                                                  &sco_params->flags);
                if (RetCode == CO_ERROR_NO_ERROR)
                {
                    /* This is a SCO type                                               */
                    sco_params->s2m_pkt_type = sco_params->m2s_pkt_type;
                    *SyncType = SCO_TYPE;
                    lm_sync_conf[sco_link_id]->Type = SCO_TYPE;
                    lm_sync_conf[sco_link_id]->NewParam.FrameSize = 2;
                }
                else
                {
                    return RetCode;
                }
            }
        }
    }

    /* Update AirMode CVSD, ALaw, uLaw, Transparent */
    VoiceSetting = lm_sync_conf[sco_link_id]->SyncVoice & AIR_COD_MSK;
    if (VoiceSetting == AIR_COD_MULAW)
    {
        sco_params->air_mode = MU_LAW_MODE;
    }
    else if (VoiceSetting == AIR_COD_ALAW)
    {
        sco_params->air_mode = A_LAW_MODE;
    }
    else if (VoiceSetting == AIR_COD_CVSD)
    {
        sco_params->air_mode = CVSD_MODE;
    }
    else
    {
        sco_params->air_mode = TRANS_MODE;
    }

    // Choose Timing Flag, if master or eSCO (for slave/SCO, the timing control flag shall be chosen by the master)
    sco_params->flags = 0;
    if ((*SyncType == ESCO_TYPE) || (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE))
    {
        // Check MSB of clock (bit 27 of the BT clock)
        if ((CLK_ADD_2(ld_read_clock(), ld_acl_clock_offset_get(lm_sync_conf[sco_link_id]->LinkId).hs) & BT_CLOCK_MSB) != 0)
        {
            sco_params->flags = INIT2_FLAG;
        }
    }

    /* Store Packet types                                                               */
    lm_sync_conf[sco_link_id]->NewParam.PacketM2S = sco_params->m2s_pkt_type;
    lm_sync_conf[sco_link_id]->NewParam.PacketS2M = sco_params->s2m_pkt_type;

    if (*SyncType == ESCO_TYPE)
    {
        lm_sync_conf[sco_link_id]->NewParam.LenM2S = sco_params->m2s_pkt_len;
        lm_sync_conf[sco_link_id]->NewParam.LenS2M = sco_params->s2m_pkt_len;
        lm_sync_conf[sco_link_id]->NewParam.Window = sco_params->w_esco;

        /* If master, try to allocate a LtAddr                                          */
        if (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE)
        {
            lm_sync_conf[sco_link_id]->LtAddr = lm_lt_addr_alloc();
            if (lm_sync_conf[sco_link_id]->LtAddr == 0x00)
            {
                return CO_ERROR_CON_LIMIT_EXCEED;
            }
        }
        /* If slave, use LtAddr 0 (master will send back the real one)                  */
        else
        {
            lm_sync_conf[sco_link_id]->LtAddr = 0x00;
        }
        sco_params->esco_lt_addr = lm_sync_conf[sco_link_id]->LtAddr;
        sco_params->nego_state = ESCO_NEGO_INIT;
    } /* eSCO */
    else
    {
        lm_sync_conf[sco_link_id]->NewParam.LenM2S = 0;
        lm_sync_conf[sco_link_id]->NewParam.LenS2M = 0;
        lm_sync_conf[sco_link_id]->NewParam.Window = 0;
    }

    /* Store Proposed &sco_params-> parameters                                                   */
    lm_sync_conf[sco_link_id]->NewParam.Offset = sco_params->d_esco;
    lm_sync_conf[sco_link_id]->NewParam.Interval = sco_params->t_esco;
    lm_sync_conf[sco_link_id]->NewParam.AirMode = sco_params->air_mode;
    lm_sync_conf[sco_link_id]->NewParam.TimingFlag = sco_params->flags;

    return (RetCode);
}

/**
 ****************************************************************************************
 * @brief This function is used to get parameters for a Sync link if the request
 *        comes from the peer device
 *
 * @param[in]  sco_link_id     Synchronous link ID
 * @param[in]  SyncType           SCO or eSCO
 * @param[out] sco_params      SCO parameters
 *
 * @returns Status
 ****************************************************************************************
 */
__STATIC uint8_t lm_get_sync_param_peer_req(uint8_t sco_link_id, uint8_t SyncType, struct lc_sco_air_params_tag *sco_params)
{
    uint32_t BandWidth;
    uint8_t  RetCode = CO_ERROR_NO_ERROR, NewWindow = sco_params->w_esco;
    uint16_t TxLen, RxLen;

    /* Check if remote proposed parameters are correct/supported                        */
    RetCode = lm_sync_check_params(sco_link_id,
                                   lm_sync_conf[sco_link_id]->Type,
                                   sco_params->m2s_pkt_type,
                                   sco_params->s2m_pkt_type,
                                   sco_params->m2s_pkt_len,
                                   sco_params->s2m_pkt_len,
                                   sco_params->t_esco,
                                   sco_params->d_esco,
                                   sco_params->w_esco,
                                   lm_sync_conf[sco_link_id]->CommonPacketType);

    if (RetCode != CO_ERROR_NO_ERROR)
    {
        return RetCode;
    }

    /* If Remote device request SCO link                                                */
    if (SyncType == SCO_TYPE)
    {
        // Only 1 x HV3/HV1 supported
        if ((sco_params->m2s_pkt_type == SCO_PACKET_HV3) || (sco_params->m2s_pkt_type == SCO_PACKET_HV1))
        {
            if (lm_nb_sync_active > 1)
            {
                return CO_ERROR_SYNC_CON_LIMIT_DEV_EXCEED;
            }
        }
        else
        {
            return CO_ERROR_SCO_INTERVAL_REJECTED;
        }
        lm_sync_conf[sco_link_id]->NewParam.FrameSize = 2;
        sco_params->w_esco = NewWindow = 0;
    }
    /* else if remote device request an eSCO link                                       */
    else if (SyncType == ESCO_TYPE)
    {
        if (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE)
        {
            /* Extract Tx and Rx Length                                                 */
            TxLen = sco_params->m2s_pkt_len;
            RxLen = sco_params->s2m_pkt_len;

            /* If master, try to allocate a LtAddr                                      */
            lm_sync_conf[sco_link_id]->LtAddr = lm_lt_addr_alloc();
            if (lm_sync_conf[sco_link_id]->LtAddr == 0x00)
            {
                return CO_ERROR_CON_LIMIT_EXCEED;
            }
            sco_params->esco_lt_addr = lm_sync_conf[sco_link_id]->LtAddr;
        }
        else
        {
            /* Extract Tx and Rx Length                                                 */
            TxLen = sco_params->s2m_pkt_len;
            RxLen = sco_params->m2s_pkt_len;

            /* If slave, Store the LtAddr sent by the master                            */
            lm_sync_conf[sco_link_id]->LtAddr = sco_params->esco_lt_addr;
        }

        /* Compute Tx bandwidth                                                         */
        BandWidth = (1600 * TxLen) / sco_params->t_esco;
        lm_sync_conf[sco_link_id]->SyncTxBw = (uint16_t)BandWidth;

        /* Compute Rx bandwidth                                                         */
        BandWidth = (1600 * RxLen) / sco_params->t_esco;
        lm_sync_conf[sco_link_id]->SyncRxBw = (uint16_t)BandWidth;

        /* Store the requested frame size                                               */
        lm_sync_conf[sco_link_id]->NewParam.FrameSize = lm_get_sync_frame_size(sco_params->m2s_pkt_type,
                sco_params->s2m_pkt_type);
    }

    /* Store requested SCO/eSCO parameters                                              */
    lm_sync_conf[sco_link_id]->NewParam.PacketM2S = sco_params->m2s_pkt_type;
    lm_sync_conf[sco_link_id]->NewParam.LenM2S = sco_params->m2s_pkt_len;
    lm_sync_conf[sco_link_id]->NewParam.PacketS2M = sco_params->s2m_pkt_type;
    lm_sync_conf[sco_link_id]->NewParam.LenS2M = sco_params->s2m_pkt_len;

    /* If the Requested window size is not supported, propose a new one                 */
    if (NewWindow != sco_params->w_esco)
    {
        sco_params->nego_state = ESCO_NEGO_UNSUPPORTED;
        lm_sync_conf[sco_link_id]->NewParam.Window = NewWindow;
    }
    else
    {
        sco_params->nego_state = ESCO_NEGO_INIT;
        lm_sync_conf[sco_link_id]->NewParam.Window = sco_params->w_esco;
    }

    if (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE)
    {
        // Get an offset for the synchronous link
        lm_get_sync_offset(sco_link_id, sco_params->t_esco, &sco_params->d_esco, lm_sync_conf[sco_link_id]->NewParam.FrameSize, 0, sco_params->flags);
    }

    lm_sync_conf[sco_link_id]->NewParam.TimingFlag = sco_params->flags;
    lm_sync_conf[sco_link_id]->NewParam.Offset = sco_params->d_esco;
    lm_sync_conf[sco_link_id]->NewParam.Interval = sco_params->t_esco;
    lm_sync_conf[sco_link_id]->NewParam.AirMode = sco_params->air_mode;
    return (RetCode);
}

/**
 ****************************************************************************************
 * @brief This function is used to get parameters for a Sync link if this is a response from the higher layers
 *
 * @param[in]  sco_link_id     Synchronous link ID
 * @param[out] sco_params      SCO parameters
 *
 * @returns Status
 ****************************************************************************************
 */
__STATIC uint8_t lm_get_sync_param_hl_rsp(uint8_t sco_link_id, struct lc_sco_air_params_tag *sco_params)
{
    uint32_t   Max_Latency = 0;
    signed int NewWesco = sco_params->w_esco;
    uint8_t NewFrameSize = 0;
    uint8_t RetCode = CO_ERROR_NO_ERROR, NextNegoState = ESCO_NEGO_INIT;

    /* Check If requested packet type are allowed by host                               */
    RetCode = lm_sync_check_pktAllowed(lm_sync_conf[sco_link_id]->Type,
                                       lm_sync_conf[sco_link_id]->NewParam.PacketM2S,
                                       lm_sync_conf[sco_link_id]->NewParam.PacketS2M,
                                       lm_sync_conf[sco_link_id]->ReqPacketType);

    /* If remote device request an SCO link                                             */
    if (lm_sync_conf[sco_link_id]->Type == SCO_TYPE)
    {
        if (RetCode != CO_ERROR_NO_ERROR)
        {
            return RetCode;
        }

        /* Check that the retransmission window size is 0                               */
        if (NewWesco != 0)
        {
            return (CO_ERROR_INVALID_LMP_PARAM);
        }

        if (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE)
        {
            // Get an offset for the synchronous link
            lm_get_sync_offset(sco_link_id, lm_sync_conf[sco_link_id]->NewParam.Interval, &lm_sync_conf[sco_link_id]->NewParam.Offset, 2, 0, sco_params->flags);
        }

        /* For a SCO link, return the peer parameters                                   */
        sco_params->m2s_pkt_type = lm_sync_conf[sco_link_id]->NewParam.PacketM2S;
        sco_params->s2m_pkt_type = lm_sync_conf[sco_link_id]->NewParam.PacketS2M;
        sco_params->t_esco = lm_sync_conf[sco_link_id]->NewParam.Interval;
    }
    /* If remote device request an eSCO link                                            */
    else
    {

        do
        {
            /* Packet type are not allowed by host, but it should be possible to change
               them
             */
            if (RetCode != CO_ERROR_NO_ERROR)
            {
                /* Packet must be renegotiated                                          */
                NextNegoState = ESCO_NEGO_UNSUPPORTED;
                break;
            }

            /* Compute Max Latency in number of Slot.                                   */
            Max_Latency = (lm_sync_conf[sco_link_id]->Latency * 1000) / 625;

            /* Check if it is compatible with the interval given by the peer            */
            if ((uint32_t)(lm_sync_conf[sco_link_id]->NewParam.Interval +
                           lm_sync_conf[sco_link_id]->NewParam.FrameSize +
                           lm_sync_conf[sco_link_id]->NewParam.Window) > Max_Latency)
            {
                /* Parameters must be renegotiated                                      */
                NextNegoState = ESCO_NEGO_LAT_VIOLATION;
                break;
            }

            /*
             * Check the retransmission window size and the reTX effort:
             *  If the ReTX window is not null but the host asked for No RETX, we only
             * need to recompute the window size using lm_sync_get_best_win since we
             * already check latency and interval
             */
            if ((lm_sync_conf[sco_link_id]->ReTx != SYNC_NO_RE_TX) &&
                    (lm_sync_conf[sco_link_id]->ReTx != SYNC_RE_TX_DONT_CARE) &&
                    (NewWesco == 0))
            {
                /* Retransmission window must be renegotiated                           */
                NextNegoState = ESCO_NEGO_UNSUPPORTED;
                break;
            }

            /* If Packets are allowed and if latency is correct, check window size      */
            NewFrameSize = lm_get_sync_frame_size(sco_params->m2s_pkt_type, sco_params->s2m_pkt_type);
            /* Compute best windows size                                                */
            RetCode = lm_sync_get_best_win(NewFrameSize,
                                           lm_sync_conf[sco_link_id]->ReTx,
                                           sco_params->t_esco,
                                           &NewWesco);
            if (RetCode != CO_ERROR_NO_ERROR)
            {
                return CO_ERROR_INVALID_LMP_PARAM;
            }
            /* Check if the Bandwidth is OK                                             */
            RetCode = lm_check_esco_bandwidth(sco_link_id,
                                              sco_params->t_esco,
                                              NewFrameSize,
                                              &NewWesco);
            if (RetCode != CO_ERROR_NO_ERROR)
            {
                NextNegoState = ESCO_NEGO_UNSUPPORTED;
                break;
            }
        }
        while (0);
        /* If parameters (packet type or latency) must be renegotiated                  */
        if (NextNegoState == ESCO_NEGO_UNSUPPORTED ||
                NextNegoState == ESCO_NEGO_LAT_VIOLATION)
        {
            sco_params->w_esco = NewWesco;
            /* Find the appropriate parameters                                          */
            RetCode = lm_get_esco_params(sco_link_id,
                                         &sco_params->t_esco,
                                         &lm_sync_conf[sco_link_id]->NewParam.Offset,
                                         &sco_params->m2s_pkt_type,
                                         &sco_params->s2m_pkt_type,
                                         &sco_params->m2s_pkt_len,
                                         &sco_params->s2m_pkt_len,
                                         &sco_params->w_esco,
                                         &sco_params->flags);
            if (RetCode != CO_ERROR_NO_ERROR)
            {
                return RetCode;
            }
        }
        else if (NextNegoState == ESCO_NEGO_SLOT_VIOLATION)
        {
            sco_params->w_esco = NewWesco;

            // Get an offset for the synchronous link
            lm_get_sync_offset(sco_link_id, sco_params->t_esco, &lm_sync_conf[sco_link_id]->NewParam.Offset, NewFrameSize, sco_params->w_esco, sco_params->flags);
        }
        else
        {
            /* If the Windows is the same                                               */
            if (sco_params->w_esco == NewWesco)
            {
                if (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE)
                {
                    NextNegoState = ESCO_NEGO_LATEST_POSSIBLE;
                }
                else
                {
                    /* On the slave side, request SDL so send Accepted msg              */
                    NextNegoState = ESCO_NEGO_INIT;
                }
            }
            else
            {
                /*
                 * If the sent computed Wesco is different from the sent one, try to
                 * negotiate it with the peer device
                 */
                sco_params->w_esco = NewWesco;
                NextNegoState = ESCO_NEGO_UNSUPPORTED;
                lm_sync_conf[sco_link_id]->NewParam.Window = NewWesco;
            }
        }
        /* Store New parameters proposed                                            */
        lm_sync_conf[sco_link_id]->NewParam.LenM2S = sco_params->m2s_pkt_len;
        lm_sync_conf[sco_link_id]->NewParam.LenS2M = sco_params->s2m_pkt_len;
        lm_sync_conf[sco_link_id]->NewParam.PacketM2S = sco_params->m2s_pkt_type;
        lm_sync_conf[sco_link_id]->NewParam.PacketS2M = sco_params->s2m_pkt_type;
        lm_sync_conf[sco_link_id]->NewParam.Interval = sco_params->t_esco;
        sco_params->d_esco = lm_sync_conf[sco_link_id]->NewParam.Offset;
        lm_sync_conf[sco_link_id]->NewParam.FrameSize = lm_get_sync_frame_size(sco_params->m2s_pkt_type,
                sco_params->s2m_pkt_type);
        lm_sync_conf[sco_link_id]->NewParam.Window = sco_params->w_esco;
        sco_params->nego_state = NextNegoState;
        sco_params->esco_lt_addr = lm_sync_conf[sco_link_id]->LtAddr;
    }/* eSCO */

    /* Get Sync parameters                                                              */
    sco_params->esco_hdl = lm_sync_conf[sco_link_id]->SyncHdl;
    sco_params->air_mode = lm_sync_conf[sco_link_id]->NewParam.AirMode;

    // Choose Timing Flag, if master/eSCO (for slave or eSCO, the timing control flag has been chosen by the peer)
    if ((lm_sync_conf[sco_link_id]->Type == SCO_TYPE) && (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE))
    {
        // Check MSB of clock (bit 27 of the BT clock)
        if ((ld_read_clock() & BT_CLOCK_MSB) != 0)
        {
            sco_params->flags = INIT2_FLAG;
        }
        lm_sync_conf[sco_link_id]->NewParam.TimingFlag = sco_params->flags;
    }

    if (RetCode == CO_ERROR_NO_ERROR)
    {
        lm_get_sync_param_update(sco_link_id,
                                 &sco_params->t_esco, &sco_params->d_esco,
                                 &sco_params->air_mode,
                                 &sco_params->m2s_pkt_len, &sco_params->s2m_pkt_len, &sco_params->w_esco,
                                 &sco_params->flags);
    }

    return (RetCode);

}

/**
 ****************************************************************************************
 * @brief This function is used to get parameters for a Sync link if this is a response from the peer device.
 *
 * @param[in]  sco_link_id     Synchronous link ID
 * @param[out] sco_params      SCO parameters
 *
 * @returns Status
 ****************************************************************************************
 */
__STATIC uint8_t lm_get_sync_param_peer_rsp(uint8_t sco_link_id, struct lc_sco_air_params_tag *sco_params)
{
    uint32_t Max_Latency = 0;
    signed int NewWesco = sco_params->w_esco;
    uint8_t NextNegoState = ESCO_NEGO_INIT;
    uint8_t FrameSize = 0, RetCode = CO_ERROR_NO_ERROR;

    /* If this is a SCO link                                                            */
    if (lm_sync_conf[sco_link_id]->Type == SCO_TYPE)
    {
        /* Check that the packet proposed is the same than the original one             */
        if (sco_params->m2s_pkt_type != lm_sync_conf[sco_link_id]->NewParam.PacketM2S)
        {
            return CO_ERROR_INVALID_LMP_PARAM;
        }

        /* Check if remote proposed parameters are correct/supported                    */
        RetCode = lm_sync_check_params(sco_link_id,
                                       lm_sync_conf[sco_link_id]->Type,
                                       sco_params->m2s_pkt_type,
                                       sco_params->s2m_pkt_type,
                                       sco_params->m2s_pkt_len,
                                       sco_params->s2m_pkt_len,
                                       sco_params->t_esco,
                                       sco_params->d_esco,
                                       sco_params->w_esco,
                                       lm_sync_conf[sco_link_id]->CommonPacketType);

        if (RetCode != CO_ERROR_NO_ERROR)
        {
            return RetCode;
        }

        // Only 1 x HV3/HV1 supported
        if ((sco_params->m2s_pkt_type != SCO_PACKET_HV3) && (sco_params->m2s_pkt_type != SCO_PACKET_HV1))
        {
            return CO_ERROR_SCO_INTERVAL_REJECTED;
        }
    }
    /* If this is an eSCO link                                                          */
    else
    {
        if (++lm_nego_cnt > lm_nego_max_cnt)
        {
            return CO_ERROR_UNSPECIFIED_ERROR;
        }
        if (lm_nego_cntl > 2)
        {
            return CO_ERROR_UNSPECIFIED_ERROR;
        }

        do
        {
            /* Check if remote proposed parameters are correct/supported                */
            RetCode = lm_sync_check_params(sco_link_id,
                                           lm_sync_conf[sco_link_id]->Type,
                                           sco_params->m2s_pkt_type,
                                           sco_params->s2m_pkt_type,
                                           sco_params->m2s_pkt_len,
                                           sco_params->s2m_pkt_len,
                                           sco_params->t_esco,
                                           sco_params->d_esco,
                                           sco_params->w_esco,
                                           lm_sync_conf[sco_link_id]->CommonPacketType);

            if (RetCode != CO_ERROR_NO_ERROR)
            {
                return RetCode;
            }

            /*  First Check if Packet Type is allowed by host                           */
            RetCode = lm_sync_check_pktAllowed(lm_sync_conf[sco_link_id]->Type,
                                               sco_params->m2s_pkt_type,
                                               sco_params->s2m_pkt_type,
                                               lm_sync_conf[sco_link_id]->ReqPacketType);

            if (RetCode != CO_ERROR_NO_ERROR)
            {
                lm_nego_cntl = 0;
                NextNegoState = ESCO_NEGO_UNSUPPORTED;
                break;
            }

            /* Check the latency                                                        */

            /* Compute Max Latency in number of Slot                                    */
            Max_Latency = (lm_sync_conf[sco_link_id]->Latency * 1000) / 625;
            /* Compute synchronous frame size in slots                                  */
            FrameSize = lm_get_sync_frame_size(sco_params->m2s_pkt_type, sco_params->s2m_pkt_type);

            /* Check if it is compatible with the interval given by the peer            */
            if ((uint32_t)(sco_params->t_esco + sco_params->w_esco + FrameSize) > Max_Latency)
            {
                lm_nego_cnt--;
                lm_nego_cntl++;
                NextNegoState = ESCO_NEGO_LAT_VIOLATION;
                break;
            }

            /* Finally check the retransmission window                                  */
            /* Store the new frame size                                                 */
            lm_sync_conf[sco_link_id]->NewParam.FrameSize = FrameSize;

            /* Compute best windows size                                                */
            RetCode = lm_sync_get_best_win(FrameSize,
                                           lm_sync_conf[sco_link_id]->ReTx,
                                           sco_params->t_esco,
                                           &NewWesco);
            if (RetCode != CO_ERROR_NO_ERROR)
            {
                return CO_ERROR_INVALID_LMP_PARAM;
            }

            /* Check if the Bandwidth is OK                                             */
            RetCode = lm_check_esco_bandwidth(sco_link_id,
                                              sco_params->t_esco,
                                              FrameSize,
                                              &NewWesco);
            if (RetCode != CO_ERROR_NO_ERROR)
            {
                lm_nego_cnt--;
                lm_nego_cntl++;
                NextNegoState = ESCO_NEGO_UNSUPPORTED;
            }
        }
        while (0);

        lm_sync_conf[sco_link_id]->NewParam.Offset = sco_params->d_esco;

        if (NextNegoState == ESCO_NEGO_UNSUPPORTED ||
                NextNegoState == ESCO_NEGO_LAT_VIOLATION)
        {
            uint16_t TempPacketType;
            /* Mask the Last send packet type and so that function returns a new
               packet type
            */
            /* Mask all the packet types till the last send one                     */
            lm_nego_pkt_used |= (lm_convert_type_to_flag(lm_sync_conf[sco_link_id]->NewParam.PacketM2S));
            TempPacketType = lm_sync_conf[sco_link_id]->ReqPacketType;
            lm_sync_conf[sco_link_id]->ReqPacketType &= ~lm_nego_pkt_used;

            /* Find the appropriate parameters                                      */
            RetCode = lm_get_esco_params(sco_link_id,
                                         &sco_params->t_esco,
                                         &sco_params->d_esco,
                                         &sco_params->m2s_pkt_type,
                                         &sco_params->s2m_pkt_type,
                                         &sco_params->m2s_pkt_len,
                                         &sco_params->s2m_pkt_len,
                                         &sco_params->w_esco,
                                         &sco_params->flags);
            lm_sync_conf[sco_link_id]->ReqPacketType = TempPacketType;
            if (RetCode != CO_ERROR_NO_ERROR)
            {
                return RetCode;
            }
        }
        else if (NextNegoState != ESCO_NEGO_SLOT_VIOLATION)
        {
            if (sco_params->w_esco == NewWesco)
            {
                /* Request SDL to send Accepted msg                                 */
                NextNegoState = ESCO_NEGO_INIT;
            }
            else
            {
                if ((sco_params->w_esco > 6) ||
                        ((lm_sync_conf[sco_link_id]->ReTx == SYNC_NO_RE_TX) &&
                         ((sco_params->w_esco) != 0)))
                {
                    /*
                     * If the sent computed Wesco is > 6 or not 0 with a reTX
                     * effort requested as no reTX, negotiate it with the peer
                     * device
                     */
                    sco_params->w_esco = NewWesco;
                    NextNegoState = ESCO_NEGO_UNSUPPORTED;
                    lm_nego_cnt--;
                    lm_nego_cntl++;
                }
                else
                {
                    /*
                     * Try to negotiate a better reTX window, if this has not be
                     * done yet
                     */
                    sco_params->w_esco = NewWesco;
                    NextNegoState = ESCO_NEGO_LATEST_POSSIBLE;
                }
            } /* latency */
        } /* packet allowed by host */
        if (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE)
        {
            /* If master, send the previously allocated LtAddr                          */
            sco_params->esco_lt_addr = lm_sync_conf[sco_link_id]->LtAddr;
        }
        else
        {
            /* If slave, take the LtAddr sent by the master                             */
            lm_sync_conf[sco_link_id]->LtAddr = sco_params->esco_lt_addr;
        }

    } /* eSCO */

    /* Get Sync parameters                                                              */
    if (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE)
    {
        /* On the Master side, get the previous allocated sync handle                   */
        sco_params->esco_hdl = lm_sync_conf[sco_link_id]->SyncHdl;
    }
    else
    {
        /* On the slave side, no handle was allocated. Keep the one sent by the master  */
        lm_sync_conf[sco_link_id]->SyncHdl = sco_params->esco_hdl;
    }

    lm_sync_conf[sco_link_id]->NewParam.PacketM2S  = sco_params->m2s_pkt_type;
    lm_sync_conf[sco_link_id]->NewParam.LenM2S     = sco_params->m2s_pkt_len;
    lm_sync_conf[sco_link_id]->NewParam.PacketS2M  = sco_params->s2m_pkt_type;
    lm_sync_conf[sco_link_id]->NewParam.LenS2M     = sco_params->s2m_pkt_len;
    lm_sync_conf[sco_link_id]->NewParam.Interval   = sco_params->t_esco;
    lm_sync_conf[sco_link_id]->NewParam.AirMode    = sco_params->air_mode;
    lm_sync_conf[sco_link_id]->NewParam.Window     = sco_params->w_esco;
    lm_sync_conf[sco_link_id]->NewParam.Offset     = sco_params->d_esco;
    lm_sync_conf[sco_link_id]->NewParam.TimingFlag = sco_params->flags;
    sco_params->nego_state = NextNegoState;

    if ((RetCode == CO_ERROR_NO_ERROR) && (NextNegoState == ESCO_NEGO_INIT))
    {
        lm_get_sync_param_update(sco_link_id,
                                 &sco_params->t_esco, &sco_params->d_esco,
                                 &sco_params->air_mode,
                                 &sco_params->m2s_pkt_len, &sco_params->s2m_pkt_len, &sco_params->w_esco,
                                 &sco_params->flags);
    }

    return (RetCode);
}

/**
 ****************************************************************************************
 * @brief This function is used to get parameters for a Sync link if this is a
 *        request from Higher Layer to modify the Sync link parameters
 *
 * @param[in]  sco_link_id     Synchronous link ID
 * @param[out] sco_params      SCO parameters
 *
 * @returns Status
 ****************************************************************************************
 */
__STATIC uint8_t lm_get_sync_param_hl_modif(uint8_t sco_link_id, struct lc_sco_air_params_tag *sco_params)
{
    uint8_t RetCode = CO_ERROR_NO_ERROR;

    /* If current link is a SCO */
    if (lm_sync_conf[sco_link_id]->Type == SCO_TYPE)
    {
        RetCode = lm_get_sco_param_hl_req(sco_link_id,
                                          &sco_params->t_esco,
                                          &sco_params->d_esco,
                                          &sco_params->m2s_pkt_type,
                                          &sco_params->flags);
        if (RetCode == CO_ERROR_NO_ERROR)
        {
            sco_params->s2m_pkt_type = sco_params->m2s_pkt_type;
        }
        else
        {
            return RetCode;
        }
    }
    /* If current link is an eSCO */
    else
    {
        RetCode = lm_get_esco_params(sco_link_id,
                                     &sco_params->t_esco,
                                     &sco_params->d_esco,
                                     &sco_params->m2s_pkt_type,
                                     &sco_params->s2m_pkt_type,
                                     &sco_params->m2s_pkt_len,
                                     &sco_params->s2m_pkt_len,
                                     &sco_params->w_esco,
                                     &sco_params->flags);

        if (RetCode != CO_ERROR_NO_ERROR)
        {
            return RetCode;
        }
    }

    // Choose Timing Flag, if master or eSCO (for slave/SCO, the timing control flag shall be chosen by the master)
    sco_params->flags = 0;
    if ((lm_sync_conf[sco_link_id]->Type == ESCO_TYPE) || (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE))
    {
        // Check MSB of clock (bit 27 of the BT clock)
        if ((CLK_ADD_2(ld_read_clock(), ld_acl_clock_offset_get(lm_sync_conf[sco_link_id]->LinkId).hs) & BT_CLOCK_MSB) != 0)
        {
            sco_params->flags = INIT2_FLAG;
        }
    }

    sco_params->air_mode = lm_sync_conf[sco_link_id]->CurParam.AirMode;
    sco_params->esco_hdl = lm_sync_conf[sco_link_id]->SyncHdl;

    if (lm_sync_conf[sco_link_id]->Type == ESCO_TYPE)
    {
        lm_sync_conf[sco_link_id]->NewParam.LenM2S = sco_params->m2s_pkt_len;
        lm_sync_conf[sco_link_id]->NewParam.LenS2M = sco_params->s2m_pkt_len;
        lm_sync_conf[sco_link_id]->NewParam.Window = sco_params->w_esco;
        sco_params->esco_lt_addr = lm_sync_conf[sco_link_id]->LtAddr;
        sco_params->nego_state = ESCO_NEGO_INIT;
    }

    /* Store Proposed SCO parameters */
    lm_sync_conf[sco_link_id]->NewParam.PacketM2S = sco_params->m2s_pkt_type;
    lm_sync_conf[sco_link_id]->NewParam.PacketS2M = sco_params->s2m_pkt_type;
    lm_sync_conf[sco_link_id]->NewParam.Offset = sco_params->d_esco;
    lm_sync_conf[sco_link_id]->NewParam.Interval = sco_params->t_esco;
    lm_sync_conf[sco_link_id]->NewParam.AirMode = sco_params->air_mode;
    lm_sync_conf[sco_link_id]->NewParam.TimingFlag = sco_params->flags;
    lm_sync_conf[sco_link_id]->NewParam.FrameSize = lm_get_sync_frame_size(sco_params->m2s_pkt_type,
            sco_params->s2m_pkt_type);
    return (RetCode);
}

/**
 ****************************************************************************************
 * @brief This function is used to get parameters for a Sync link if this is a
 *        request from Peer device to modify the Sync link parameters
 *
 * @param[in]  sco_link_id     Synchronous link ID
 * @param[out] sco_params      SCO parameters
 *
 * @returns Status
 ****************************************************************************************
 */
__STATIC uint8_t lm_get_sync_param_peer_modif(uint8_t sco_link_id, struct lc_sco_air_params_tag *sco_params)
{
    uint32_t   BandWidth = 0;
    uint32_t   Max_Latency = 0;
    signed int NewWesco = sco_params->w_esco;
    uint16_t   TxLen = 0;
    uint16_t   RxLen = 0;
    uint8_t    FrameSize = 0;
    uint8_t    RetCode = CO_ERROR_NO_ERROR;
    uint8_t    Check_Failed = 0;

    /* If this is a SCO link                                                            */
    if (lm_sync_conf[sco_link_id]->Type == SCO_TYPE)
    {

        /* Check if remote proposed parameters are correct/supported                    */
        RetCode = lm_sync_check_params(sco_link_id,
                                       lm_sync_conf[sco_link_id]->Type,
                                       sco_params->m2s_pkt_type,
                                       sco_params->s2m_pkt_type,
                                       sco_params->m2s_pkt_len,
                                       sco_params->s2m_pkt_len,
                                       sco_params->t_esco,
                                       sco_params->d_esco,
                                       sco_params->w_esco,
                                       lm_sync_conf[sco_link_id]->CommonPacketType);

        if (RetCode != CO_ERROR_NO_ERROR)
        {
            return RetCode;
        }

        // Only 1 x HV3/HV1 supported
        if ((sco_params->m2s_pkt_type != SCO_PACKET_HV3) && (sco_params->m2s_pkt_type != SCO_PACKET_HV1))
        {
            return CO_ERROR_SCO_INTERVAL_REJECTED;
        }

        FrameSize = 2;
    }
    /* else if current Sync link is ESCO                                                */
    else
    {
        /* Check for all the error conditions first                                     */
        if (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE)
        {
            TxLen = sco_params->m2s_pkt_len;
            RxLen = sco_params->s2m_pkt_len;
        }
        else
        {
            TxLen = sco_params->s2m_pkt_len;
            RxLen = sco_params->m2s_pkt_len;
        }

        /* Compute new Tx bandwidth (use a uint32_t tmpVar to avoid overflow)                */
        BandWidth = (1600 * TxLen) / sco_params->t_esco;

        /* Check that the BandWidth is the same                                         */
        if (lm_sync_conf[sco_link_id]->SyncTxBw != (uint16_t)BandWidth)
        {
            return CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
        }

        /* Compute new Rx bandwidth (use a uint32_t tmpVar to avoid overflow)                */
        BandWidth = (1600 * RxLen) / sco_params->t_esco;

        /* Check that the BandWidth is the same                                         */
        if (lm_sync_conf[sco_link_id]->SyncRxBw != (uint16_t)BandWidth)
        {
            return CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
        }

        /* Check if the LtAddr is the same                                              */
        if (lm_sync_conf[sco_link_id]->LtAddr != sco_params->esco_lt_addr)
        {
            return CO_ERROR_INVALID_LMP_PARAM;
        }

        if (++lm_nego_cnt > lm_nego_max_cnt)
        {
            return CO_ERROR_UNSPECIFIED_ERROR;
        }
        else if (lm_nego_cntl > 2)
        {
            return CO_ERROR_UNSPECIFIED_ERROR;
        }

        do
        {
            /* Check if remote proposed parameters are correct/supported                */
            RetCode = lm_sync_check_params(sco_link_id,
                                           lm_sync_conf[sco_link_id]->Type,
                                           sco_params->m2s_pkt_type,
                                           sco_params->s2m_pkt_type,
                                           sco_params->m2s_pkt_len,
                                           sco_params->s2m_pkt_len,
                                           sco_params->t_esco,
                                           sco_params->d_esco,
                                           sco_params->w_esco,
                                           lm_sync_conf[sco_link_id]->CommonPacketType);

            if (RetCode != CO_ERROR_NO_ERROR)
            {
                return RetCode;
            }

            /* Check if proposed packet type are allowed by host                        */
            RetCode = lm_sync_check_pktAllowed(lm_sync_conf[sco_link_id]->Type,
                                               sco_params->m2s_pkt_type,
                                               sco_params->s2m_pkt_type,
                                               lm_sync_conf[sco_link_id]->ReqPacketType);
            if (RetCode != CO_ERROR_NO_ERROR)
            {
                Check_Failed = 1;
                lm_nego_cntl = 0;
                sco_params->nego_state = ESCO_NEGO_UNSUPPORTED;
                break;
            }

            /* Compute Max Latency in number of Slot                                    */
            Max_Latency = (lm_sync_conf[sco_link_id]->Latency * 1000) / 625;
            /* Compute synchronous frame size in slots                                  */
            FrameSize = lm_get_sync_frame_size(sco_params->m2s_pkt_type, sco_params->s2m_pkt_type);

            /* Check if it is compatible with the interval given by the peer            */
            if ((uint32_t)(sco_params->t_esco + sco_params->w_esco + FrameSize) > Max_Latency)
            {
                lm_nego_cnt --;
                lm_nego_cntl++;
                Check_Failed = 1;
                sco_params->nego_state = ESCO_NEGO_LAT_VIOLATION;
                break;
            }

            /* Store the new frame size                                             */
            lm_sync_conf[sco_link_id]->NewParam.FrameSize = FrameSize;

            /* Compute best windows size                                            */
            RetCode = lm_sync_get_best_win(FrameSize,
                                           lm_sync_conf[sco_link_id]->ReTx,
                                           sco_params->t_esco,
                                           &NewWesco);
            if (RetCode != CO_ERROR_NO_ERROR)
            {
                return CO_ERROR_INVALID_LMP_PARAM;
            }

            /* Check if the Bandwidth is OK                                             */
            RetCode = lm_check_esco_bandwidth(sco_link_id,
                                              sco_params->t_esco,
                                              FrameSize,
                                              &NewWesco);
            if (RetCode != CO_ERROR_NO_ERROR)
            {
                lm_nego_cnt--;
                lm_nego_cntl++;
                Check_Failed = 1;
                break;
            }

            /* If a new Offset must be proposed                                             */
            if (lm_sync_conf[sco_link_id]->NewParam.Offset != sco_params->d_esco)
            {
                /* Ask LMP_eSCO transmission                                            */
                sco_params->nego_state  = ESCO_NEGO_SLOT_VIOLATION;
                lm_nego_cnt--;
                lm_nego_cntl++;
            }
        }
        while (0);

        if (Check_Failed == 1)
        {
            uint16_t TempPacketType;
            /* Mask the Last send packet type and so that function returns a new
               packet type
             */
            /* Mask all the packet types till the last send one                         */
            lm_nego_pkt_used |= (lm_convert_type_to_flag(
                                     lm_sync_conf[sco_link_id]->NewParam.PacketM2S));
            TempPacketType = lm_sync_conf[sco_link_id]->ReqPacketType;
            lm_sync_conf[sco_link_id]->ReqPacketType &= ~lm_nego_pkt_used;
            sco_params->nego_state = ESCO_NEGO_UNSUPPORTED;
            /* Find the appropriate parameters                                          */
            RetCode = lm_get_esco_params(sco_link_id,
                                         &sco_params->t_esco,
                                         &sco_params->d_esco,
                                         &sco_params->m2s_pkt_type,
                                         &sco_params->s2m_pkt_type,
                                         &sco_params->m2s_pkt_len,
                                         &sco_params->s2m_pkt_len,
                                         &sco_params->w_esco,
                                         &sco_params->flags);
            lm_sync_conf[sco_link_id]->ReqPacketType = TempPacketType;
            if (RetCode != CO_ERROR_NO_ERROR)
            {
                return RetCode;
            }
            sco_params->m2s_pkt_type = lm_sync_conf[sco_link_id]->NewParam.PacketM2S;
            sco_params->s2m_pkt_type = lm_sync_conf[sco_link_id]->NewParam.PacketM2S;
            sco_params->m2s_pkt_len = lm_sync_conf[sco_link_id]->NewParam.LenM2S;
            sco_params->s2m_pkt_len = lm_sync_conf[sco_link_id]->NewParam.LenS2M;
            sco_params->t_esco = lm_sync_conf[sco_link_id]->NewParam.Interval;
            sco_params->w_esco = lm_sync_conf[sco_link_id]->NewParam.Window;
        }
        else
        {
            if (sco_params->w_esco == NewWesco)
            {
                /* Request SDL to send Accepted msg                                 */
                sco_params->nego_state = ESCO_NEGO_INIT;
            }
            else
            {
                if ((sco_params->w_esco > 6) ||
                        ((lm_sync_conf[sco_link_id]->ReTx == SYNC_NO_RE_TX) &&
                         ((sco_params->w_esco) != 0)))
                {
                    /*
                     * If the sent computed Wesco is > 6 or not 0 with a reTX
                     * effort requested as no reTX, negotiate it with the peer
                     * device
                     */
                    sco_params->w_esco = NewWesco;
                    sco_params->nego_state = ESCO_NEGO_UNSUPPORTED;
                }
                else
                {
                    sco_params->w_esco = NewWesco;
                    sco_params->nego_state = ESCO_NEGO_LATEST_POSSIBLE;
                }
            }
        }
        if (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE)
        {
            /* If master, send the previously allocated LtAddr                          */
            sco_params->esco_lt_addr = lm_sync_conf[sco_link_id]->LtAddr;
        }
        else
        {
            /* If slave, take the LtAddr sent by the master                             */
            lm_sync_conf[sco_link_id]->LtAddr = sco_params->esco_lt_addr;
        }
    } /* eSCO */

    // Choose Timing Flag, if master/eSCO (for slave or eSCO, the timing control flag has been chosen by the peer)
    if ((lm_sync_conf[sco_link_id]->Type == SCO_TYPE) && (lm_sync_conf[sco_link_id]->Role == MASTER_ROLE))
    {
        // Check MSB of clock (bit 27 of the BT clock)
        if ((ld_read_clock() & BT_CLOCK_MSB) != 0)
        {
            sco_params->flags = INIT2_FLAG;
        }
    }

    /* Store requested SCO/eSCO parameters                                              */
    lm_sync_conf[sco_link_id]->NewParam.PacketM2S = sco_params->m2s_pkt_type;
    lm_sync_conf[sco_link_id]->NewParam.LenM2S = sco_params->m2s_pkt_len;
    lm_sync_conf[sco_link_id]->NewParam.PacketS2M = sco_params->s2m_pkt_type;
    lm_sync_conf[sco_link_id]->NewParam.LenS2M = sco_params->s2m_pkt_len;
    lm_sync_conf[sco_link_id]->NewParam.Window = sco_params->w_esco;
    lm_sync_conf[sco_link_id]->NewParam.FrameSize = FrameSize;
    lm_sync_conf[sco_link_id]->NewParam.Offset = sco_params->d_esco;
    lm_sync_conf[sco_link_id]->NewParam.Interval = sco_params->t_esco;
    lm_sync_conf[sco_link_id]->NewParam.AirMode = sco_params->air_mode;
    lm_sync_conf[sco_link_id]->NewParam.TimingFlag = sco_params->flags;

    return (RetCode);
}

uint8_t lm_get_sync_param(uint8_t sco_link_id, struct lc_sco_air_params_tag *sco_params, uint8_t Request)
{
    uint8_t RetCode = CO_ERROR_UNSPECIFIED_ERROR;

    switch (Request)
    {
    /* If this is a Peer Response                                                   */
    case SYNC_PEER_RSP:
        RetCode = lm_get_sync_param_peer_rsp(sco_link_id, sco_params);
        if (RetCode == CO_ERROR_INVALID_HCI_PARAM)
        {
            RetCode = CO_ERROR_INVALID_LMP_PARAM;
        }
        break;

    /* If this Request is requested by peer device                                  */
    case SYNC_PEER_MODIF:
        RetCode = lm_get_sync_param_peer_modif(sco_link_id, sco_params);
        if (RetCode == CO_ERROR_INVALID_HCI_PARAM)
        {
            RetCode = CO_ERROR_INVALID_LMP_PARAM;
        }
        break;

    /* If this is a update current parameters with new parameters                   */
    case SYNC_UPDATE:
        lm_get_sync_param_update(sco_link_id,
                                 &sco_params->t_esco, &sco_params->d_esco,
                                 &sco_params->air_mode,
                                 &sco_params->m2s_pkt_len, &sco_params->s2m_pkt_len, &sco_params->w_esco,
                                 &sco_params->flags);
        RetCode = CO_ERROR_NO_ERROR;
        break;

    default:
        break;
    }

    if (RetCode != CO_ERROR_NO_ERROR)
    {
        lm_sync_nego = false;
    }

    return (RetCode);
}


/*
 * FUNCTION DEFINITION
 ****************************************************************************************
 */

void lm_init_sync(void)
{
    memset(&lm_sync_conf, 0x00, sizeof(lm_sync_conf));
    lm_nb_sync_active = 0;
    lm_nego_cnt = 0;
    lm_nego_cntl = 0;
    lm_sync_nego = false;
}

void lm_reset_sync(void)
{
    // Free allocated memory
    for (int i = 0 ; i < MAX_NB_SYNC ; i++)
    {
        if (lm_sync_conf[i] != NULL)
        {
            // Free SCO parameters memory
            ke_free(lm_sync_conf[i]);
        }
    }

    lm_init_sync();
}

uint8_t lm_add_sync(uint8_t sco_link_id, struct features *RemoteFeatures,
                    uint8_t LinkId,      uint8_t Role,           uint8_t *SyncType,
                    uint32_t SyncTxBw,   uint32_t SyncRxBw,       uint16_t SyncLat,        uint16_t VoiceSetting,
                    uint8_t SyncReTx,        uint16_t SyncPacketType,
                    struct lc_sco_air_params_tag *sco_params, uint8_t Request)
{
    uint16_t CommonPacketType;
    uint8_t status = CO_ERROR_NO_ERROR;

    /* Check if max number of Sync already reached                                      */
    if (lm_nb_sync_active == MAX_NB_SYNC)
    {
        return CO_ERROR_SYNC_CON_LIMIT_DEV_EXCEED;
    }

    /* Check if another Sync is already in negotiation                                  */
    if (lm_sync_nego == true)
    {
        if (Request == SYNC_HL_REQ)
        {
            return CO_ERROR_COMMAND_DISALLOWED;
        }
        else
        {
            return CO_ERROR_LMP_PDU_NOT_ALLOWED;
        }
    }

    /*
     * Check Parameters
     */
    if (Request == SYNC_HL_REQ)
    {
        /* Get Common packet type (SCO & eSCO)                                          */
        CommonPacketType = lm_get_common_pkt_types(SYNC_PACKET_TYPE_HV1_FLAG |
                           SYNC_PACKET_TYPE_HV2_FLAG |
                           SYNC_PACKET_TYPE_HV3_FLAG |
                           SYNC_PACKET_TYPE_EV3_FLAG |
                           SYNC_PACKET_TYPE_EV4_FLAG |
                           SYNC_PACKET_TYPE_EV5_FLAG |
                           SYNC_PACKET_TYPE_EV3_2_FLAG |
                           SYNC_PACKET_TYPE_EV3_3_FLAG |
                           SYNC_PACKET_TYPE_EV5_2_FLAG |
                           SYNC_PACKET_TYPE_EV5_3_FLAG,
                           RemoteFeatures);

        sco_params->esco_lt_addr = 0;
        *SyncType = 0;
    }
    /* If the request comes from peer device                                           */
    else if (Request == SYNC_PEER_REQ)
    {
        /* SCO link type                                                                */
        if (*SyncType == SCO_TYPE)
        {
            /* Check only HVx packet types                                              */
            SyncPacketType = SYNC_PACKET_TYPE_HV1_FLAG |
                             SYNC_PACKET_TYPE_HV2_FLAG |
                             SYNC_PACKET_TYPE_HV3_FLAG;
            sco_params->esco_lt_addr = 0;
            SyncTxBw = SCO_BANDWIDTH;
            SyncRxBw = SCO_BANDWIDTH;
        }
        /* SCO link type                                                                */
        else if (*SyncType == ESCO_TYPE)
        {
            /* Check only EVx packet types                                              */
            SyncPacketType = SYNC_PACKET_TYPE_EV3_FLAG |
                             SYNC_PACKET_TYPE_EV4_FLAG |
                             SYNC_PACKET_TYPE_EV5_FLAG |
                             SYNC_PACKET_TYPE_EV3_2_FLAG |
                             SYNC_PACKET_TYPE_EV3_3_FLAG |
                             SYNC_PACKET_TYPE_EV5_2_FLAG |
                             SYNC_PACKET_TYPE_EV5_3_FLAG ;

            if (Role == MASTER_ROLE)
            {
                sco_params->esco_lt_addr = 0;
            }
        }
        else
        {
            return CO_ERROR_UNSPECIFIED_ERROR;
        }

        /* Get Common packet type                                                       */
        CommonPacketType = lm_get_common_pkt_types(SyncPacketType, RemoteFeatures);
        /* Check if at least one common packet type exit                            */
        if (CommonPacketType == 0x0000)
        {
            return CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
        }
        /* Keep only compatible packet types                                           */
        SyncPacketType &= CommonPacketType;

        /* Check Air mode requested                                                     */
        if (sco_params->air_mode == CVSD_MODE)
        {
            if ((RemoteFeatures->feats[2] & B2_CVSD_MSK) == 0)
            {
                return CO_ERROR_SCO_AIR_MODE_REJECTED;
            }
        }
        else if (sco_params->air_mode == MU_LAW_MODE)
        {
            if ((RemoteFeatures->feats[1] & B1_MULAW_MSK) == 0)
            {
                return CO_ERROR_SCO_AIR_MODE_REJECTED;
            }
        }
        else if (sco_params->air_mode == A_LAW_MODE)
        {
            if ((RemoteFeatures->feats[1] & B1_ALAW_MSK) == 0)
            {
                return CO_ERROR_SCO_AIR_MODE_REJECTED;
            }
        }
        else if (sco_params->air_mode == TRANS_MODE)
        {
            if ((RemoteFeatures->feats[2] & B2_TRANSPARENT_SCO_MSK) == 0)
            {
                return CO_ERROR_SCO_AIR_MODE_REJECTED;
            }
        }
        else
        {
            return CO_ERROR_SCO_AIR_MODE_REJECTED;
        }
    }
    else
    {
        return CO_ERROR_UNSPECIFIED_ERROR;
    }

    // Allocate a data structure for SCO parameters
    lm_sync_conf[sco_link_id] = (lm_sync_config *) ke_malloc_system(sizeof(lm_sync_config), KE_MEM_ENV);

    if (lm_sync_conf[sco_link_id] != NULL)
    {
        lm_sync_config *conf = lm_sync_conf[sco_link_id];

        conf->Role = Role;
        conf->LinkId = LinkId;
        conf->ReqPacketType = SyncPacketType;
        conf->CommonPacketType = CommonPacketType;
        conf->SyncVoice = VoiceSetting;
        conf->Type = *SyncType;
        conf->NegoState = ESCO_NEGO_INIT;
        conf->LtAddr = sco_params->esco_lt_addr;
        conf->SyncTxBw = SyncTxBw;
        conf->SyncRxBw = SyncRxBw;

        conf->Latency = SyncLat;
        conf->ReTx = SyncReTx;

        /* If Master, Initialize Sco Handle and Timing Flag                 */
        if (Role == MASTER_ROLE)
        {
            sco_params->esco_hdl = sco_link_id + 1; /* Sync Handle goes from 1 to 0xFF     */
        }
        /* If Slave                                                         */
        else
        {
            /* If Request from HL, set Sync Handle to 0                     */
            if (Request == SYNC_HL_REQ)
            {
                sco_params->esco_hdl = 0x00;     /* Slave send Sync Handle 0         */
            }
        }

        conf->SyncHdl = sco_params->esco_hdl;/* Store Sync Handle       */

        conf->NewParam.AirMode = sco_params->air_mode;

        lm_nb_sync_active++;              /* One Sync more                    */

        lm_sync_nego = true;   /* A Sync is in negotiation        */
        lm_nego_cnt = 0;
        lm_nego_cntl = 0;
        lm_nego_max_cnt = ESCO_NEGOCNT_MAX;
        lm_nego_pkt_used = 0;
    }
    else
    {
        ASSERT_ERR(0);
        return CO_ERROR_UNSPECIFIED_ERROR;
    }

    // Determine the initial parameters
    if (Request == SYNC_HL_REQ)
    {
        status = lm_get_sync_param_hl_req(sco_link_id, SyncType, sco_params);
    }
    else
    {
        status = lm_get_sync_param_peer_req(sco_link_id, *SyncType, sco_params);
    }

    return status;
}

uint8_t lm_modif_sync(uint8_t sco_link_id, uint8_t SyncType,
                      uint32_t SyncTxBw, uint32_t SyncRxBw, uint16_t SyncLat,
                      uint16_t SyncVoice, uint8_t SyncReTx, uint16_t SyncPacketType,
                      struct lc_sco_air_params_tag *sco_params, uint8_t Request)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    /* Check if another Sync is already in negotiation      2                            */
    if (lm_sync_nego == true)
    {
        if (Request == SYNC_HL_MODIF)
        {
            return CO_ERROR_COMMAND_DISALLOWED;
        }
        else
        {
            return CO_ERROR_LMP_PDU_NOT_ALLOWED;
        }
    }

    /*
     * Check Parameters
     */
    if (Request == SYNC_HL_MODIF)
    {
        if (SyncType == ESCO_TYPE)
        {
            /* The current sync link is eSCO => keep only eSCO packet type              */
            SyncPacketType &= (SYNC_PACKET_TYPE_EV3_FLAG |
                               SYNC_PACKET_TYPE_EV4_FLAG |
                               SYNC_PACKET_TYPE_EV5_FLAG |
                               SYNC_PACKET_TYPE_EV3_2_FLAG |
                               SYNC_PACKET_TYPE_EV3_3_FLAG |
                               SYNC_PACKET_TYPE_EV5_2_FLAG |
                               SYNC_PACKET_TYPE_EV5_3_FLAG) ;

            /* The Tx and Rx Bandwidth cannot be renegotiated                           */
            if ((SyncTxBw != lm_sync_conf[sco_link_id]->SyncTxBw)  ||
                    (SyncRxBw != lm_sync_conf[sco_link_id]->SyncRxBw))
            {
                return CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
            }

            /* Check Air mode requested                                                     */
            if ((SyncVoice & AIR_COD_MSK) == AIR_COD_CVSD)
            {
                if ((lm_sync_conf[sco_link_id]->CurParam.AirMode) != CVSD_MODE)
                {
                    return CO_ERROR_SCO_AIR_MODE_REJECTED;
                }
            }
            else if ((SyncVoice & AIR_COD_MSK) == AIR_COD_MULAW)
            {
                if ((lm_sync_conf[sco_link_id]->CurParam.AirMode) != MU_LAW_MODE)
                {
                    return CO_ERROR_SCO_AIR_MODE_REJECTED;
                }
            }
            else if ((SyncVoice & AIR_COD_MSK) == AIR_COD_ALAW)
            {
                if ((lm_sync_conf[sco_link_id]->CurParam.AirMode) != A_LAW_MODE)
                {
                    return CO_ERROR_SCO_AIR_MODE_REJECTED;
                }
            }
            else if ((SyncVoice & AIR_COD_MSK) == AIR_COD_TRANS)
            {
                if ((lm_sync_conf[sco_link_id]->CurParam.AirMode) != TRANS_MODE)
                {
                    return CO_ERROR_SCO_AIR_MODE_REJECTED;
                }
            }
        } /* if eSCO */

        /* Check if requested packet type is compatible                                 */
        if ((SyncPacketType & lm_sync_conf[sco_link_id]->CommonPacketType) == 0x0000)
        {
            /* If not, this means the remote device does not support what the higher
             * layers asked for.
             */
            return CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
        }


        lm_sync_conf[sco_link_id]->SyncVoice = SyncVoice;
        lm_sync_conf[sco_link_id]->Latency = SyncLat;
        lm_sync_conf[sco_link_id]->ReTx = SyncReTx;
        lm_sync_conf[sco_link_id]->ReqPacketType = SyncPacketType;



    } /* if SYNC_HL_MODIF */
    else if (Request == SYNC_PEER_MODIF)
    {
        /* Check if remote device use LM_SCOLinkreq to change a SCO link and            */
        /* LM_SCOLinkreq to change an eSCO link                                         */
        if (SyncType != lm_sync_conf[sco_link_id]->Type)
        {
            return CO_ERROR_LMP_PDU_NOT_ALLOWED;
        }
    }
    else
    {
        return CO_ERROR_UNSPECIFIED_ERROR;
    }

    lm_sync_conf[sco_link_id]->NegoState = ESCO_NEGO_INIT;

    lm_sync_nego = true;   /* A Sync is in negotiation                        */
    lm_nego_cnt = 0;
    lm_nego_cntl = 0;
    lm_nego_pkt_used = 0;
    lm_nego_max_cnt = lm_get_total_pkt_type(lm_sync_conf[sco_link_id]->ReqPacketType);

    if (Request == SYNC_HL_MODIF)
    {
        status = lm_get_sync_param_hl_modif(sco_link_id, sco_params);
    }
    else if (Request == SYNC_PEER_MODIF)
    {
        status = lm_get_sync_param_peer_modif(sco_link_id, sco_params);


        if ((status == CO_ERROR_NO_ERROR) && (sco_params->nego_state == ESCO_NEGO_INIT))
        {
            lm_get_sync_param_update(sco_link_id,
                                     &sco_params->t_esco, &sco_params->d_esco,
                                     &sco_params->air_mode,
                                     &sco_params->m2s_pkt_len, &sco_params->s2m_pkt_len, &sco_params->w_esco,
                                     &sco_params->flags);
        }
    }

    return status;
}

uint8_t lm_check_sync_hl_rsp(uint8_t sco_link_id, uint32_t SyncTxBw,  uint32_t SyncRxBw,
                             uint16_t SyncLat,    uint16_t SyncVoice, uint8_t SyncReTx, uint16_t SyncPacketType,
                             struct lc_sco_air_params_tag *sco_params)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    /* If HL does not reply with a don't care tx bandwidth */
    if (SyncTxBw != SYNC_BANDWIDTH_DONT_CARE)
    {
        /* If This is a SCO link */
        if (lm_sync_conf[sco_link_id]->Type == SCO_TYPE)
        {
            /* Tx bandwidth must be 64000 bps */
            if (SyncTxBw != SCO_BANDWIDTH)
            {
                return CO_ERROR_UNSUPPORTED;
            }
        }
        else if (lm_sync_conf[sco_link_id]->Type == ESCO_TYPE)
        {
            /* Tx bandwidth must be same that requested by remote */
            if (lm_sync_conf[sco_link_id]->SyncTxBw != SyncTxBw)
            {
                return (CO_ERROR_INVALID_HCI_PARAM);
            }
        }
    }

    /* If HL does not reply with a don't care rx bandwidth */
    if (SyncRxBw != SYNC_BANDWIDTH_DONT_CARE)
    {
        /* If This is a SCO link */
        if (lm_sync_conf[sco_link_id]->Type == SCO_TYPE)
        {
            /* Tx bandwidth must be 64000 bps */
            if (SyncRxBw != SCO_BANDWIDTH)
            {
                return CO_ERROR_UNSUPPORTED;
            }
        }
        else if (lm_sync_conf[sco_link_id]->Type == ESCO_TYPE)
        {
            /* Tx bandwidth must be same that requested by remote */
            if (lm_sync_conf[sco_link_id]->SyncRxBw != SyncRxBw)
            {
                return (CO_ERROR_INVALID_HCI_PARAM);
            }
        }
    }

    /*
     * Just store the latency value. The peer interval/reTX will be checked with this
     * latency
     */

    lm_sync_conf[sco_link_id]->Latency = SyncLat;
    /* Check if HL reply with the same air mode */
    if ((SyncVoice & AIR_COD_MSK) == AIR_COD_CVSD)
    {
        if (lm_sync_conf[sco_link_id]->NewParam.AirMode != CVSD_MODE)
        {
            return CO_ERROR_SCO_AIR_MODE_REJECTED;
        }
    }
    else if ((SyncVoice & AIR_COD_MSK) == AIR_COD_MULAW)
    {
        if (lm_sync_conf[sco_link_id]->NewParam.AirMode != MU_LAW_MODE)
        {
            return CO_ERROR_SCO_AIR_MODE_REJECTED;
        }
    }
    else if ((SyncVoice & AIR_COD_MSK) == AIR_COD_ALAW)
    {
        if (lm_sync_conf[sco_link_id]->NewParam.AirMode != A_LAW_MODE)
        {
            return CO_ERROR_SCO_AIR_MODE_REJECTED;
        }
    }
    else if ((SyncVoice & AIR_COD_MSK) == AIR_COD_TRANS)
    {
        if (lm_sync_conf[sco_link_id]->NewParam.AirMode != TRANS_MODE)
        {
            return CO_ERROR_SCO_AIR_MODE_REJECTED;
        }
    }

    /* Check Retransmission effort param */
    if (lm_sync_conf[sco_link_id]->Type == SCO_TYPE)
    {
        if ((SyncReTx == SYNC_RE_TX_POWER) ||
                (SyncReTx == SYNC_RE_TX_QUALITY))
        {
            return (CO_ERROR_INVALID_HCI_PARAM);
        }
    }
    else if (lm_sync_conf[sco_link_id]->Type == ESCO_TYPE)
    {
        /* Store the new retransmission effort (used in GetSyncParamHLRsp function) */
        lm_sync_conf[sco_link_id]->ReTx = SyncReTx;
    }

    /* Check SyncPacketType allowed */
    lm_sync_conf[sco_link_id]->ReqPacketType = SyncPacketType;
    /* If no common allowed packet type */
    if (lm_sync_conf[sco_link_id]->ReqPacketType == 0x0000)
    {
        return (CO_ERROR_INVALID_HCI_PARAM);
    }
    /* Store the Negotiation Count from the Total Packet types that are
       supported*/
    lm_nego_max_cnt = lm_get_total_pkt_type(lm_sync_conf[sco_link_id]->ReqPacketType);

    status = lm_get_sync_param_hl_rsp(sco_link_id, sco_params);

    return (status);
}

uint8_t lm_get_synchdl(uint8_t sco_link_id)
{
    uint8_t sco_hdl = 0;

    if (sco_link_id < MAX_NB_SYNC)
    {
        sco_hdl = lm_sync_conf[sco_link_id]->SyncHdl;
    }

    return sco_hdl;
}

bool lm_look_for_sync(uint8_t link_id, uint8_t type)
{
    /* Look for the structure which store informations for this Connection Handle       */
    for (int i = 0 ; i < MAX_NB_SYNC ; i++)
    {
        if (lm_sync_conf[i] != NULL)
        {
            if (lm_sync_conf[i]->LinkId == link_id)
            {
                if ((type != ACL_TYPE) && (lm_sync_conf[i]->Type != type))
                    continue;

                return (true);
            }
        }
    }
    return (false);
}

uint8_t lm_get_nb_sync_link(void)
{
    return lm_nb_sync_active;
}

uint8_t lm_get_min_sync_intv(void)
{
    uint8_t min_sync_intv = 0xFF;

    // Loop on Sync links to get the Minimum Interval
    for (int i = 0 ; i < MAX_NB_SYNC ; i++)
    {
        if (lm_sync_conf[i] != NULL)
        {
            min_sync_intv = co_min(min_sync_intv, lm_sync_conf[i]->CurParam.Interval);
        }
    }

    return min_sync_intv;
}

void lm_remove_sync(uint8_t sco_link_id)
{
    if (lm_sync_conf[sco_link_id] != NULL)
    {
        /* Free LtAddr (allocated only if master and eSCO used) */
        if ((lm_sync_conf[sco_link_id]->Role == MASTER_ROLE) && (lm_sync_conf[sco_link_id]->Type == ESCO_TYPE) && (lm_sync_conf[sco_link_id]->LtAddr != 0))
        {
            lm_lt_addr_free(lm_sync_conf[sco_link_id]->LtAddr);
        }

        // Free SCO parameters memory
        ke_free(lm_sync_conf[sco_link_id]);
        lm_sync_conf[sco_link_id] = NULL;

        lm_nb_sync_active--; // One SCO less
        lm_sync_nego = false;  // End of SCO negotiation
    }
}

void lm_sco_nego_end(void)
{
    lm_sync_nego = false;  // End of SCO negotiation
}

#endif //(MAX_NB_SYNC > 0)

///@} LMSCO
