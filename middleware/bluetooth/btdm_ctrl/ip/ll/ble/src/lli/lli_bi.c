

/**
 ****************************************************************************************
 *
 * @file lli_bi.c
 *
 * @brief Definition of the functions used by the link layer Broadcast Isochronous module
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLI
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"              // Stack configuration
#if (BLE_BIS)

#include <string.h>
#include "rwip.h"                     // IP definitions
#include "lli_int.h"                  // LLI internal definitions
#include "lli.h"                      // LLI definitions
#include "llm.h"                      // LLM definitions
#include "lld.h"                      // LLD definitions
#include "hci.h"                      // For HCI definitions
#include "ke_task.h"                  // For task message returns
#include "ke_mem.h"                   // For memory allocation
#include "co_math.h"                  // For mathematics stuff
#include "co_endian.h"                // For endianess
#include "co_utils.h"                 // For GETF macro
#include "ble_util.h"                 // BLE utility functions
#include "data_path.h"                // Isochronous channel Data path
#include "sch_plan.h"                 // Bandwidth allocation
#include "aes.h"                      // For encryption key generation

#include "reg_em_ble_tx_iso_buf.h"    // For BIS PDU transmission

#if (BLE_ISOGEN)
    #include "isogen.h"         // ISOGEN definitions
#endif //(BLE_ISOGEN)
#include "btdm_patch.h"


/*
 * DEFINES
 ****************************************************************************************
 */

/// Number of packet transmission attemtps
#define LLI_BIS_NB_TX_ATTEMPT   6

/// Retrieve BIG environment
#define LLI_BIG_ENV_GET(grp_hdl, env_ptr)\
    lli_group_env_get(grp_hdl, LLI_ISO_GROUP_BIG, (struct lli_group_env**) (env_ptr))

/// Handler definition of LLCP packet
#if (BLE_OBSERVER)
#define HANDLER(name, hdl, pack) \
    [name##_OPCODE]   = { (lli_big_pdu_handler_func_t) hdl##_handler, pack, name##_LEN }
#else // (BLE_OBSERVER)
#define HANDLER(name, hdl, pack) \
    [name##_OPCODE]   = { NULL, pack, name##_LEN }
#endif // (BLE_OBSERVER)


/*
 * ENUMRATIONS DEFINTIONS
 ****************************************************************************************
 */

/// State of the BIG
enum lli_big_state
{
    /// Stream is Disabled
    LLI_BIG_DISABLED,
    /// Stream is starting
    LLI_BIG_STARTING,
    /// Stream is Enabled
    LLI_BIG_ENABLED,
    /// Stream is Stopping
    LLI_BIG_STOPPING,
};

/// Information about the BIG
enum lli_big_info
{
    /// State of the BIG (@see enum lli_big_state)
    LLI_BIG_STATE_LSB       = 0,
    LLI_BIG_STATE_MASK      = 0x03,
    /// Used to know if BIG is encrypted or not
    LLI_BIG_ENCRYPTED_POS   = 2,
    LLI_BIG_ENCRYPTED_BIT   = 0x04,
    /// True if BIG Broadcaster, else receiver
    LLI_BIG_BROADCASTER_POS = 3,
    LLI_BIG_BROADCASTER_BIT = 0x08,
    /// True if BIG termination requested
    LLI_BIG_TERM_REQ_POS    = 4,
    LLI_BIG_TERM_REQ_BIT    = 0x10,
    /// True if BIG channel map update requested
    LLI_BIG_CH_MAP_UP_POS   = 5,
    LLI_BIG_CH_MAP_UP_BIT   = 0x20,
};



/// BIG Encrypt Info
enum lli_bi_encrypt_info
{
    /// Generate Intermediate Group LTK (IGLTK)
    LLI_BI_ENC_STATE_GEN_IGLTK  = 0,
    /// Generate Group LTK (GLTK)
    LLI_BI_ENC_STATE_GEN_GLTK   = 1,
    /// Generate Group Initialization Vector (GIV)
    LLI_BI_ENC_STATE_GEN_GIV    = 2,
    /// Generate Group Session Key Diversifier (GSKD)
    LLI_BI_ENC_STATE_GEN_GSKD   = 3,
    /// Generate Group Session Key (GSK)
    LLI_BI_ENC_STATE_GEN_GSK    = 4,

    // AES Encryption source info field
    /// Position of BIG handle in AES encryption source info
    LLI_BI_ENC_GRP_HDL_LSB      = 0,
    LLI_BI_ENC_GRP_HDL_MASK     = 0x00FF,
    /// Position of Encryption state in the AES encryption source info
    LLI_BI_ENC_STATE_LSB        = 8,
    LLI_BI_ENC_STATE_MASK       = 0xFF00,
};

/*
 * TYPE DEFINITION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback used to handle BIG PDU message
 *
 * @param[in] grp_hdl        BIG Handle
 * @param[in] pdu            BIS PDU information received
 * @param[in] event_cnt      Event counter value when PDU has been received
 *
 ****************************************************************************************
 */
typedef void (*lli_big_pdu_handler_func_t)(uint8_t grp_hdl, union big_pdu *pdu, uint16_t event_cnt);

/// Information required to start the BIS activity
struct lli_bi_start_params
{
    /// Information about the stream
    struct big_info        info;
    /// Driver parameters
    struct lld_big_params  drv_params;
    /// Big info data
    uint8_t                big_info_data[__ARRAY_EMPTY];
};

/// BIG environment structure
struct lli_big_env
{
    /// ISO Group information
    struct lli_group_env         group;
    /// Information about the stream
    struct lli_bi_start_params  *p_start;
    /// SDU interval in microseconds
    uint32_t                     sdu_interval;
    /// Stream Interval in multiple of 1.25 ms (Range 0x0004-0xC80)
    uint16_t                     iso_interval;
    /// Max SDU Size (in bytes)
    uint16_t                     max_sdu_size;
#if (BLE_OBSERVER)
    /// Synchronization timeout of BIS (10ms step)
    uint16_t                     sync_timeout;
    /// Maximum number of sub-event supported by receiver
    uint8_t                      max_se;
#endif // (BLE_OBSERVER)
    /// Max PDU size (in bytes)
    uint8_t                      max_pdu;
    /// Burst number: Number of new packet per channel interval
    uint8_t                      bn;
    /// Framing mode (@see enum iso_frame)
    uint8_t                      framing;
    /// BIG handle identifier
    uint8_t                      big_hdl;
    /// Activity identifier
    uint8_t                      act_id;
    /// Periodic sync or periodic advertising activity identifier
    uint8_t                      per_act_id;
    /// Extended advertising activity identifier (for transmitter only)
    uint8_t                      ext_act_id;
    /// State, @see enum lli_big_info
    uint8_t                      info;
    /// Encryption Code (used to compute GLTK)
    uint8_t                      broadcast_code[KEY_LEN];
#if (BLE_BROADCASTER)
    ///Channel map
    struct le_ch_map             ch_map;
    /// Host termination reason
    uint8_t                      host_reason;
#endif // (BLE_BROADCASTER)
    /// Number of BIS streams
    uint8_t                      nb_bis;
    /// List of BIS activity IDs
    uint8_t                      bis_act_id[__ARRAY_EMPTY];
};

/// BIS Channel environment structure
struct lli_bis_env
{
    /// TX or RX Isochronous Data path
    const struct data_path_itf *p_dp;
    /// Group handle
    uint8_t                     grp_hdl;
    /// BIS identifier (receiver only)
    uint8_t                     bis_id;
};

/// BIS PDU handler information for packing/unpacking and function handler
struct lli_big_pdu_handler_info
{
    /// Message handler
    lli_big_pdu_handler_func_t  handler;
    /// Pack/Unpack format string
    const char                 *pack_format;
    /// Length of LLCP PDU
    uint16_t                    length;
};

/// BI Environment structure to manage the BIS configuration
struct lli_bi_env_tag
{
    /// Callback to execute when a BIS is started
    lli_bis_start_evt_cb                cb_start_evt;
};

#if (BLE_BIS_IN_ROM)
/*
 * BIS HANDLER FUNCTION DECLARATION
 ****************************************************************************************
 */

#if (BLE_OBSERVER)
    __STATIC void big_channel_map_ind_handler(uint8_t grp_hdl, struct big_channel_map_ind *pdu, uint16_t event_cnt);
    __STATIC void big_terminate_ind_handler(uint8_t grp_hdl, struct big_terminate_ind *pdu, uint16_t event_cnt);
#endif // (BLE_OBSERVER)
__STATIC void lli_bis_aes_result_cb(uint8_t status, const uint8_t *p_res, uint32_t metainfo);
__STATIC void lli_big_cleanup(uint8_t grp_hdl);
#if (BLE_BROADCASTER)
    __STATIC void lli_big_saa_gen(uint32_t activity_id, uint32_t *p_seed_acc_addr);
    __STATIC void lli_big_info_pack(const struct big_info *p_big_info, uint8_t *p_data);
#endif // (BLE_BROADCASTER)

/*
 * LOCAL VARIABLE DEFINITION
 ****************************************************************************************
 */

///Table for normal BIS PDUs handler utilities
__STATIC const struct lli_big_pdu_handler_info lli_big_pdu_handler[BIG_OPCODE_MAX_OPCODE] =
{
    HANDLER(BIG_CHANNEL_MAP_IND,       big_channel_map_ind,       "B5BH"),
    HANDLER(BIG_TERMINATE_IND,         big_terminate_ind,         "BBH"),
};

/// KEY identifier for IGLTK Generation using h7() algorithm : "BIG1"
__STATIC const uint8_t lli_bis_h7_salt[]   = {'1', 'G', 'I', 'B', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00',};

/// KEY identifier for GLTK Generation using h6() algorithm : "BIG2"
__STATIC const uint8_t lli_bis_h6_key_id[] = {'2', 'G', 'I', 'B'};

/// KEY identifier for GSK Generation using h8() algorithm : "BIG3"
__STATIC const uint8_t lli_bis_h8_key_id[] = {'3', 'G', 'I', 'B'};

/// BIS environment structure
struct lli_bis_env *lli_bis_env[BLE_ACTIVITY_MAX];

/// BI Environment variable
struct lli_bi_env_tag lli_bi_env;

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Cleanup a BIS environment
 *
 * @param[in] act_id Activity identifier of the BIS
 ****************************************************************************************
 */
__STATIC void lli_bis_cleanup(uint8_t act_id)
{
    struct lli_bis_env *p_bis_env = lli_bis_env[act_id];

    // Check if BIS exists
    if (p_bis_env != NULL)
    {
        // Remove data path
        p_bis_env->p_dp = data_path_itf_get(ISO_DP_DISABLE, ISO_SEL_RX);

        // Clean-up BIS
        llm_activity_free_set(act_id);

        // Free environment
        ke_free(lli_bis_env[act_id]);
        lli_bis_env[act_id] = NULL;
    }
}

/**
 ****************************************************************************************
 * @brief Send that BIG is created or if an error occurs during creation
 *
 * @param[in] p_big_env       Pointer to the BIG to remove
 * @param[in] big_sync_delay  Duration of the BIG synchronization delay
 * @param[in] status          Status code of creation
 * @param[in] rate            BIG transmission rate
 * @param[in] trans_latency   BIG transport latency in microseconds
 ****************************************************************************************
 */
__STATIC void lli_big_create_evt_send(struct lli_big_env *p_big_env, uint32_t big_sync_delay, uint8_t status,
                                      uint8_t rate, uint32_t trans_latency)
{
    uint8_t cursor;

    // inform that the stream is disabled
    struct hci_le_create_big_cmp_evt *p_evt;
    // Allocate message to return
    p_evt                 = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_create_big_cmp_evt);
    p_evt->subcode        = HCI_LE_CREATE_BIG_CMP_EVT_SUBCODE;
    p_evt->status         = status;
    p_evt->big_hdl        = p_big_env->big_hdl;

    if (status == CO_ERROR_NO_ERROR)
    {
        struct big_info *p_big_info = &(p_big_env->p_start->info);

        p_evt->num_bis           = p_big_env->nb_bis;
        p_evt->phy               = co_phy_mask_to_value[co_rate_to_phy_mask[rate]];
        p_evt->nse               = p_big_info->nse;
        p_evt->bn                = p_big_info->bn;
        p_evt->pto               = p_big_info->pto;
        p_evt->irc               = p_big_info->irc;
        p_evt->max_pdu           = p_big_info->max_pdu;
        p_evt->iso_interval      = p_big_info->iso_interval;
        p_evt->big_sync_delay    = big_sync_delay;
        p_evt->big_trans_latency = trans_latency;

        for (cursor = 0 ; cursor < p_big_env->nb_bis ; cursor++)
        {
            uint8_t stream_hdl = p_big_env->bis_act_id[cursor];

            // fill BIS connection handle
            p_evt->conhdl[cursor] =  BLE_ACTID_TO_BISHDL(stream_hdl);
        }
    }
    else
    {
        p_evt->num_bis = 0;
    }

    // send command complete event.
    hci_send_2_host(p_evt);
}

/**
 ****************************************************************************************
 * @brief Send that BIG Sync is created or if an error occurs during creation
 *
 * @param[in] p_big_env       Pointer to the BIG to remove
 * @param[in] status          Status code of creation
 * @param[in] trans_latency   BIG transport latency in microseconds
 ****************************************************************************************
 */
__STATIC void lli_big_create_sync_evt_send(struct lli_big_env *p_big_env, uint8_t status, uint32_t trans_latency)
{
    uint8_t cursor;

    // inform that the stream is disabled
    struct hci_le_big_sync_est_evt *p_evt;
    // Allocate message to return
    p_evt                 = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_big_sync_est_evt);
    p_evt->subcode        = HCI_LE_BIG_SYNC_ESTABLISHED_EVT_SUBCODE;
    p_evt->status         = status;
    p_evt->big_hdl        = p_big_env->big_hdl;

    if (status == CO_ERROR_NO_ERROR)
    {
        struct big_info *p_big_info = &(p_big_env->p_start->info);

        p_evt->num_bis           = p_big_env->nb_bis;
        p_evt->big_trans_latency = trans_latency;
        p_evt->nse               = p_big_info->nse;
        p_evt->bn                = p_big_info->bn;
        p_evt->pto               = p_big_info->pto;
        p_evt->irc               = p_big_info->irc;
        p_evt->max_pdu           = p_big_info->max_pdu;
        p_evt->iso_interval      = p_big_info->iso_interval;

        for (cursor = 0 ; cursor < p_big_env->nb_bis ; cursor++)
        {
            uint8_t stream_hdl = p_big_env->bis_act_id[cursor];

            // fill BIS connection handle
            p_evt->conhdl[cursor] =  BLE_ACTID_TO_BISHDL(stream_hdl);
        }
    }
    else
    {
        p_evt->num_bis           = 0;
        p_evt->big_trans_latency = 0;
        p_evt->nse               = 0;
        p_evt->bn                = 0;
        p_evt->pto               = 0;
        p_evt->irc               = 0;
        p_evt->max_pdu           = 0;
        p_evt->iso_interval      = 0;
    }

    // send command complete event.
    hci_send_2_host(p_evt);
}

#if (BLE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Send that BIG is terminated to host
 *
 * @param[in] big_hdl   BIG handle
 * @param[in] status    Status of termination execution
 ****************************************************************************************
 */
__STATIC void lli_big_terminate_evt_send(uint8_t big_hdl, uint8_t reason)
{
    // inform that the stream is disabled
    struct hci_le_terminate_big_cmp_evt *p_evt;
    // Allocate message to return
    p_evt                 = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_terminate_big_cmp_evt);
    p_evt->subcode        = HCI_LE_TERMINATE_BIG_CMP_EVT_SUBCODE;
    p_evt->big_hdl        = big_hdl;
    p_evt->reason         = reason;

    // send command complete event.
    hci_send_2_host(p_evt);
}
#endif // (BLE_BROADCASTER)

#if (BLE_OBSERVER)
/**
 ****************************************************************************************
 * @brief Send that BIG Sync is terminated to host
 *
 * @param[in] big_hdl   BIG handle
 * @param[in] reason    Sync Termination reason
 ****************************************************************************************
 */
__STATIC void lli_big_sync_lost_evt_send(uint8_t big_hdl, uint8_t reason)
{
    // inform that the stream is disabled
    struct hci_le_big_sync_lost_evt *p_evt;
    // Allocate message to return
    p_evt                 = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_big_sync_lost_evt);
    p_evt->subcode        = HCI_LE_BIG_SYNC_LOST_EVT_SUBCODE;
    p_evt->big_hdl        = big_hdl;
    p_evt->reason         = reason;

    // send command complete event.
    hci_send_2_host(p_evt);
}

/**
 ****************************************************************************************
 * @brief Handle reception of a channel map update
 *
 * @param grp_hdl    BIG Handler
 * @param pdu        BIS Channel Map indication PDU
 * @param event_cnt  Event Counter when PDU is received
 ****************************************************************************************
 */
__STATIC void big_channel_map_ind_handler(uint8_t grp_hdl, struct big_channel_map_ind *pdu, uint16_t event_cnt)
{
    struct lli_big_env *p_big_env;

    do
    {
        // load stream environment
        if (LLI_BIG_ENV_GET(grp_hdl, &p_big_env))
        {
            break;
        }

        // check that stream is Enable
        if (GETF(p_big_env->info, LLI_BIG_STATE) != LLI_BIG_ENABLED)
        {
            break;
        }

        // check that instant is not passed
        if (BLE_UTIL_INSTANT_PASSED(pdu->evt_cnt, event_cnt))
        {
            lld_big_stop(p_big_env->group.hdl, CO_ERROR_INSTANT_PASSED);
            break;
        }

        // inform that channel map must be updated
        if (lld_big_ch_map_update(grp_hdl, &pdu->ch_map, pdu->evt_cnt) != CO_ERROR_NO_ERROR)
        {
            lld_big_stop(p_big_env->group.hdl, CO_ERROR_LMP_PDU_NOT_ALLOWED);
        }
    }
    while (0);
}

/**
 ****************************************************************************************
 * @brief Handle reception of a terminate indication
 *
 * @param grp_hdl    BIG Handler
 * @param pdu        BIS Channel Map indication PDU
 * @param event_cnt  Event Counter when PDU is received
 ****************************************************************************************
 */
__STATIC void big_terminate_ind_handler(uint8_t grp_hdl, struct big_terminate_ind *pdu, uint16_t event_cnt)
{
    struct lli_big_env *p_big_env;

    do
    {
        // load stream environment
        if (LLI_BIG_ENV_GET(grp_hdl, &p_big_env))
        {
            break;
        }

        // check that stream is Enable
        if (GETF(p_big_env->info, LLI_BIG_STATE) == LLI_BIG_ENABLED)
        {
            uint8_t reason = pdu->err_code;

            // check that instant is not passed
            if (BLE_UTIL_INSTANT_PASSED(pdu->evt_cnt, event_cnt))
            {
                reason = CO_ERROR_INSTANT_PASSED;
            }

            SETF(p_big_env->info, LLI_BIG_STATE, LLI_BIG_STOPPING);
            lld_big_stop(p_big_env->group.hdl, reason);
        }
    }
    while (0);
}
#endif // (BLE_OBSERVER)

#if (BLE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Pack and send pdu within BIG
 *
 * @param[in] grp_hdl    BIG handle
 * @param[in] pdu        PDU buffer to transmit
 * @param[in] nb_tx      Number of Packet transmission
 *
 * @return CO_ERROR_NO_ERROR if driver sends the buffer.
 ****************************************************************************************
 */
__STATIC uint8_t lli_big_pdu_send(uint8_t grp_hdl, union big_pdu *pdu, uint8_t nb_tx)
{
    uint8_t  status = CO_ERROR_COMMAND_DISALLOWED;
    if (pdu && (pdu->op_code < BIG_OPCODE_MAX_OPCODE))
    {
        uint16_t tx_len = lli_big_pdu_handler[pdu->op_code].length;
        uint8_t  tx_pdu[BIG_PDU_LENGTH_MAX];

        // Perform PDU packing
        status = co_util_pack(tx_pdu, (uint8_t *) pdu, &tx_len, sizeof(union llcp_pdu),
                              lli_big_pdu_handler[pdu->op_code].pack_format);

        ASSERT_INFO(status == CO_UTIL_PACK_OK, status, pdu->op_code);
        if (status == CO_UTIL_PACK_OK)
        {
            ;
            // Reuse a LLCP Buffer
            struct ble_em_llcp_buf_elt *tx_elt = ble_util_buf_llcp_tx_alloc();

            em_wr(tx_pdu, tx_elt->buf_ptr + EM_BLE_TXISODATABUF_INDEX * 2, tx_len);
            tx_elt->length = tx_len;

            // Send the PDU immediately
            status = lld_big_cntl_tx(grp_hdl, tx_elt->buf_ptr, tx_len, nb_tx);

            // check if the driver accept transmission
            if (status != CO_ERROR_NO_ERROR)
            {
                // remove the buffer
                ble_util_buf_llcp_tx_free(tx_elt->buf_ptr);
            }
        }
        else
        {
            status = CO_ERROR_INVALID_LMP_PARAM;
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, grp_hdl, pdu ? pdu->op_code : 0xFFFF);
        status = CO_ERROR_UNSPECIFIED_ERROR;
    }

    return status;
}

/**
 ****************************************************************************************
 * @brief Send BIG_TERMINATE_IND message over BIG
 *
 * @param[in] grp_hdl    BIG handle
 * @param[in] reason     Reason of BIG termination
 * @param[in] evt_cnt    Event Counter when BIG is terminated
 *
 * @return CO_ERROR_NO_ERROR if driver sends the buffer.
 ****************************************************************************************
 */
__STATIC uint8_t lli_big_terminate_ind_send(uint8_t grp_hdl, uint8_t reason, uint16_t evt_cnt, uint8_t nb_tx)
{
    struct big_terminate_ind pdu;

    pdu.op_code  = BIG_TERMINATE_IND_OPCODE;
    pdu.err_code = reason;
    pdu.evt_cnt  = evt_cnt;

    return lli_big_pdu_send(grp_hdl, (union big_pdu *) &pdu, nb_tx);
}

/**
 ****************************************************************************************
 * @brief Send BIG_CHMAP_UPDATE_IND message over BIG
 *
 * @param[in] grp_hdl    BIG handle
 * @param[in] chmap      New channel map
 * @param[in] evt_cnt    Event Counter when channel map must be applied
 * @param[in] nb_tx      Number of transmission of the PDU
 *
 * @return CO_ERROR_NO_ERROR if driver sends the buffer.
 ****************************************************************************************
 */
__STATIC uint8_t lli_big_ch_map_update_ind_send(uint8_t grp_hdl, uint8_t *p_ch_map, uint16_t evt_cnt, uint8_t nb_tx)
{
    struct big_channel_map_ind pdu;

    pdu.op_code  = BIG_CHANNEL_MAP_IND_OPCODE;
    memcpy(&(pdu.ch_map), p_ch_map, LE_CH_MAP_LEN);
    pdu.evt_cnt  = evt_cnt;

    return lli_big_pdu_send(grp_hdl, (union big_pdu *) &pdu, nb_tx);
}
#endif // (BLE_BROADCASTER)

/**
 * Get BIG parameter information
 *
 * @param[in] big_hdl  BIG handle identifier.
 *
 * @return Group parameters information
 */
__STATIC struct lli_big_env *lli_big_get(uint16_t big_hdl)
{
    // Counter
    uint8_t cnt;
    struct lli_big_env *p_env = NULL;

    for (cnt = 0; cnt < BLE_ISO_GROUP_MAX; cnt++)
    {
        // check if group exist
        if ((LLI_BIG_ENV_GET(cnt, &p_env) == CO_ERROR_NO_ERROR) && (p_env->big_hdl == big_hdl))
        {
            break;
        }
    }

    if (cnt == BLE_ISO_GROUP_MAX)
    {
        p_env = NULL;
    }

    // Return the stream parameters
    return (p_env);
}

/**
 ****************************************************************************************
 * @brief Allocate environment structure for a new BIG.
 *
 * @param[out] pp_big_env   Address of pointer containing address of allocated environment
 * @param[in]  nb_bis       Number of BIS present in the BIG
 *
 * @return Status of the allocation (@see enum co_error)
 ****************************************************************************************
 */
__STATIC uint8_t lli_big_create(struct lli_big_env **pp_big_env, uint8_t nb_bis)
{
    struct lli_group_env *p_big_env;
    uint8_t  act_id = BLE_ACTIVITY_MAX;
    // get available activity
    uint8_t status;

    do
    {
        status = llm_activity_free_get(&act_id);

        // Verify that an activity identifier and channel handle are available
        if (status != CO_ERROR_NO_ERROR)
            break;

        // Allocate stream environment
        status = lli_group_create(LLI_ISO_GROUP_BIG, sizeof(struct lli_big_env) + nb_bis, &(p_big_env));

        // Check that stream environment properly loaded, else exist
        if (status != CO_ERROR_NO_ERROR)
            break;

        *pp_big_env = (struct lli_big_env *) p_big_env;

        (*pp_big_env)->act_id     = act_id;

        // Reserve an activity for the BIG
        llm_activity_bis_reserve(act_id);

    }
    while (0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Perform clean-up of BIG
 *
 * @param[in] stream_hdl   Handle of the stream
 ****************************************************************************************
 */
__STATIC void lli_big_cleanup(uint8_t grp_hdl)
{
    struct lli_big_env *p_big_env = NULL;

    // load stream environment
    if (LLI_BIG_ENV_GET(grp_hdl, &p_big_env) == CO_ERROR_NO_ERROR)
    {
        // Cleanup all BISes
        for (uint8_t cursor = 0 ; cursor < p_big_env->nb_bis ; cursor++)
        {
            // Cleanup BIS
            lli_bis_cleanup(p_big_env->bis_act_id[cursor]);
        }

        // Set activity in reserved state and clean-up Group activity
        llm_activity_free_set(p_big_env->act_id);

        // check that start parameters must be clean-up
        if (p_big_env->p_start != NULL)
        {
            ke_free(p_big_env->p_start);
        }

        // perform a clean-up of the stream
        lli_group_cleanup(grp_hdl);
    }
}

/**
 ****************************************************************************************
 * @brief Allocate environment structure for a new BIS channel.
 *
 * @param[out] pp_bis_env   Address of pointer containing address of allocated environment
 * @param[in]  role         Role of the channel (Broadcaster or broadcast receiver)
 * @param[in]  grp_hdl      BIG Handle of stream that manage channel
 * @param[in]  bis_id       BIS Identifier  (range: [1:15])
 *
 * @return Status of the allocation (@see enum co_error)
 ****************************************************************************************
 */
__STATIC uint8_t lli_bis_create(struct lli_bis_env **pp_bis_env, uint8_t role, uint8_t grp_hdl, uint8_t bis_id, uint8_t *p_act_id)
{
    uint8_t status;

    do
    {
        uint8_t  act_id = BLE_ACTIVITY_MAX;

        // Get an activity identifier
        status = llm_activity_free_get(&act_id);
        if (status != CO_ERROR_NO_ERROR)
        {
            status = CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
            break;
        }

        // Check which channel and activity identifier are available.
        lli_bis_env[act_id] = (struct lli_bis_env *) ke_malloc_system(sizeof(struct lli_bis_env), KE_MEM_ENV);

        // Verify that an activity identifier and channel handle are available
        if (lli_bis_env[act_id] == NULL)
        {
            status = CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
            break;
        }

        // Initialize memory
        *pp_bis_env = lli_bis_env[act_id];
        *p_act_id = act_id;
        memset(*pp_bis_env, 0, sizeof(struct lli_bis_env));

        // Reserve an activity for the BIS
        llm_activity_bis_reserve(act_id);

        // Initialize default channel parameters
        (*pp_bis_env)->grp_hdl = grp_hdl;
        (*pp_bis_env)->bis_id  = bis_id;

        (*pp_bis_env)->p_dp = data_path_itf_get(ISO_DP_DISABLE, (role == ROLE_BROADCASTER) ? ISO_SEL_TX : ISO_SEL_RX);
    }
    while (0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Start the BIS driver
 *
 * @param[in] p_big_env Stream parameter
 ****************************************************************************************
 */
__STATIC uint8_t lli_big_start(struct lli_big_env *p_big_env)
{
    uint8_t status;

    struct lld_big_params *p_big_params    = &(p_big_env->p_start->drv_params);
    struct big_info        *p_big_info      = &(p_big_env->p_start->info);
    uint8_t                *p_big_info_data = &(p_big_env->p_start->big_info_data[0]);
    uint32_t                trans_latency;
    struct lld_big_start_time  big_start_time;

    // Number of pre-transmitted events
    p_big_params->nb_pt_evt     = (p_big_info->nse - (p_big_info->bn * p_big_info->irc)) / p_big_info->bn;
    // compute pre-transmission duration
    p_big_params->pretrans_dur  = p_big_params->nb_pt_evt * p_big_params->iso_interval_us * p_big_info->pto;

    // Unframed mode - compute transport latency
    if (p_big_info->framing == ISO_UNFRAMED_MODE)
    {
        // Transport_Latency = BIG_Sync_Delay + (PTO x (NSE/BN - IRC) + 1) * ISO_Interval - SDU_Interval
        p_big_params->trans_latency = (p_big_params->ctrl_sub_evt_offset - BLE_MSS_DUR) + p_big_params->pretrans_dur
                                      + p_big_params->iso_interval_us - p_big_info->sdu_interval;
    }
    // Framed mode
    else
    {
        // Transport_Latency = BIG_Sync_Delay + (PTO x (NSE/BN - IRC) + 1) * ISO_Interval + SDU_Interval
        p_big_params->trans_latency = (p_big_params->ctrl_sub_evt_offset - BLE_MSS_DUR) + p_big_params->pretrans_dur
                                      + p_big_params->iso_interval_us + p_big_info->sdu_interval;
    }

    trans_latency = p_big_params->trans_latency;

#if (BLE_BROADCASTER)
    if (p_big_params->role == ROLE_BROADCASTER)
    {
        // Random number for Base CRC init generation
        uint32_t random_nb = 0xFFFFFF & co_rand_word();
        p_big_info->base_crc_init = co_htobs(0x0000FFFF & random_nb);

        // Generate Base Access Address - take activity id of first channel
        lli_big_saa_gen(p_big_params->bis_params[0].act_id, &p_big_info->seed_access_addr);

        // Initialize Payload counter
        memset(p_big_info->bis_pkt_cnt, 0, BLE_PLD_CNT_SIZE);

        // Pack the big info into an array of data
        lli_big_info_pack(p_big_info, p_big_env->p_start->big_info_data);

        // Compute channel map
        llm_ch_map_compute(&(p_big_env->ch_map));

        memcpy(&(p_big_info->ch_map), &(p_big_env->ch_map), LE_CH_MAP_LEN);
    }
#endif // (BLE_BROADCASTER)
#if (BLE_OBSERVER)
    if ((p_big_params->role == ROLE_BROADCAST_RECEIVER) && (p_big_env->max_se != 0))
    {
        // update BIG parameters according to number of sub-events
        p_big_info->nse  = co_min(p_big_env->max_se, p_big_info->nse);
        p_big_info->nse  = p_big_info->bn * (p_big_info->nse / p_big_info->bn);

        if ((p_big_info->bn * p_big_info->irc) > p_big_info->nse)
        {
            p_big_info->irc = p_big_info->nse / p_big_info->bn;
        }

        // update timing information for the driver
        p_big_params->trans_latency -= p_big_params->pretrans_dur;
        p_big_params->nb_pt_evt      = (p_big_info->nse - (p_big_info->bn * p_big_info->irc)) / p_big_info->bn;
        p_big_params->pretrans_dur   = p_big_params->nb_pt_evt * p_big_params->iso_interval_us * p_big_info->pto;
        p_big_params->trans_latency += p_big_params->pretrans_dur;
    }
#endif // (BLE_OBSERVER)

    // Start the channel
    status = lld_big_start(p_big_params, p_big_info, p_big_info_data, &big_start_time);

    // Update stream state
    SETF(p_big_env->info, LLI_BIG_STATE, ((status == CO_ERROR_NO_ERROR) ? LLI_BIG_ENABLED : LLI_BIG_DISABLED));

    // *************************************************************************************************************
    // Send the Creation complete event
    // *************************************************************************************************************
    if (GETB(p_big_env->info, LLI_BIG_BROADCASTER))
    {
        uint16_t big_sync_delay;
        // compute BIG delay
        big_sync_delay = p_big_params->ctrl_sub_evt_offset - BLE_MSS_DUR;
        lli_big_create_evt_send(p_big_env, big_sync_delay, status, p_big_params->rate, trans_latency);
    }
    else if (status != CO_ERROR_NO_ERROR)
    {
        lli_big_create_sync_evt_send(p_big_env, status, trans_latency);
    }

    // inform that new BIS are established.
    // It must be sent after the sending of Creation complete event to let the HOST computes stream_lid and group_lid.
    if (lli_bi_env.cb_start_evt != NULL)
    {
        lli_bi_env.cb_start_evt(p_big_params, p_big_info, &big_start_time);
    }

    if ((GETB(p_big_env->info, LLI_BIG_BROADCASTER)) || (status != CO_ERROR_NO_ERROR))
    {
        // Clean-up the environment structures
        ke_free(p_big_env->p_start);
        p_big_env->p_start = NULL;
    }

    if (status != CO_ERROR_NO_ERROR)
    {
        // Cleanup BIG
        lli_big_cleanup(p_big_env->group.hdl);
    }

    return status;
}

/**
 ****************************************************************************************
 * @brief Call back definition of the function that can handle result of an AES based algorithm
 *
 * @param[in] status       Execution status
 * @param[in] aes_res      16 bytes block result
 * @param[in] metainfo     Metadata information provided by API user (see \glos{METAINFO})
 ****************************************************************************************
 */
__STATIC void lli_bis_aes_result_cb(uint8_t status, const uint8_t *p_res, uint32_t metainfo)
{
    uint8_t stream_hdl = (uint8_t) GETF(metainfo, LLI_BI_ENC_GRP_HDL);
    uint8_t enc_state  = (uint8_t) GETF(metainfo, LLI_BI_ENC_STATE);
    struct lli_big_env *p_big_env;

    do
    {
        ASSERT_INFO(status == CO_ERROR_NO_ERROR, metainfo, status);

        // load stream environment
        if (LLI_BIG_ENV_GET(stream_hdl, &p_big_env))
        {
            break;
        }

        // check that stream is stopping
        if (GETF(p_big_env->info, LLI_BIG_STATE) == LLI_BIG_STARTING)
        {
            // retrieve stream info parameter
            struct big_info *p_big_info = &(p_big_env->p_start->info);

            switch (enc_state)
            {
            // Generate IGLTK from Encryption Code
            case LLI_BI_ENC_STATE_GEN_IGLTK:
            {
                // mark that GLTK must be generated
                SETF(metainfo, LLI_BI_ENC_STATE, LLI_BI_ENC_STATE_GEN_GLTK);
                aes_h6(p_res, lli_bis_h6_key_id, lli_bis_aes_result_cb, metainfo);
            }
            break;

            // Generate GLTK from Encryption Code
            case LLI_BI_ENC_STATE_GEN_GLTK:
            {
                // Replace the encryption code by GLTK computed
                memcpy(p_big_env->broadcast_code, p_res, KEY_LEN);

                if (GETB(p_big_env->info, LLI_BIG_BROADCASTER))
                {
                    SETF(metainfo, LLI_BI_ENC_STATE,   LLI_BI_ENC_STATE_GEN_GIV);
                    // request generation of Group Initialization vector (GIV)
                    aes_rand(lli_bis_aes_result_cb, metainfo);
                }
                else
                {
                    // mark that GSK must be generated
                    SETF(metainfo, LLI_BI_ENC_STATE, LLI_BI_ENC_STATE_GEN_GSK);
                    // request generation of GSKD
                    // GSK = h8 (GLTK, GSKD, "BIS0").
                    aes_h8(p_big_env->broadcast_code, p_big_info->gskd, lli_bis_h8_key_id, lli_bis_aes_result_cb, metainfo);
                }
            }
            break;

            // Group Initialization Vector
            case LLI_BI_ENC_STATE_GEN_GIV:
            {
                // store GIV in stream info
                memcpy(p_big_info->giv, p_res, IV_LEN);
                // mark that GSKD must be generated
                SETF(metainfo, LLI_BI_ENC_STATE, LLI_BI_ENC_STATE_GEN_GSKD);
                // request generation of GSKD
                aes_rand(lli_bis_aes_result_cb, metainfo);
            }
            break;
            // Group Session Key Diversifier
            case LLI_BI_ENC_STATE_GEN_GSKD:
            {
                // Store computed GSDK
                memcpy(p_big_info->gskd, p_res, KEY_LEN);

                // mark that GSK must be generated
                SETF(metainfo, LLI_BI_ENC_STATE, LLI_BI_ENC_STATE_GEN_GSK);
                // request generation of GSKD
                // GSK = h8 (GLTK, GSKD, "BIS0").
                aes_h8(p_big_env->broadcast_code, p_big_info->gskd, lli_bis_h8_key_id, lli_bis_aes_result_cb, metainfo);
            }
            break;
            // Group Session Key
            case LLI_BI_ENC_STATE_GEN_GSK:
            {
                struct lld_big_params *p_big_params = &(p_big_env->p_start->drv_params);

                // store the generated group session key
                // Store computed GSDK
                memcpy((uint8_t *) & (p_big_params->gsk), p_res, KEY_LEN);

                // start the BIG activity
                lli_big_start(p_big_env);
            }
            break;

            default:
            {
                ASSERT_INFO_FORCE(0, stream_hdl, enc_state);
            }
            break;
            }
        }

    }
    while (0);
}

#if (BLE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Generate Seed access address for an BIG
 * A Connectionless Broadcaster shall generate a unique Seed Access Address
 *
 * SAA15-0 shall differ in at least two bits.
 * SAA 25 shall be set to 0, SAA 23 shall be set to 1
 * SAA 19 == SAA 15
 * SAA 22 == SAA 16 != SAA 15
 *
 * @param[in] activity_id       Activity ID for first channel of the BIG
 * @param[in] p_acc_addr        Pointer to the buffer that will contain the generated access address
 ****************************************************************************************
 */
__STATIC void lli_big_saa_gen(uint32_t activity_id, uint32_t *p_seed_acc_addr)
{
    *p_seed_acc_addr = co_rand_word();

    // SAA15-0 shall differ in at least two bits.
    *p_seed_acc_addr = (*p_seed_acc_addr & 0xFFFFF0F0) | ((~activity_id & 0xF) << 0) | ((activity_id & 0xF) << 8);

    // DW does not apply to lower 16 bits. Ensure derived AA shall have no more than six consecutive zeros or ones.
    if ((*p_seed_acc_addr & CO_BIT(5)) == 0)
    {
        *p_seed_acc_addr |= CO_BIT(6);
        *p_seed_acc_addr &= ~CO_BIT(12);
    }
    else
    {
        *p_seed_acc_addr &= ~CO_BIT(6);
        *p_seed_acc_addr |= CO_BIT(12);
    }

    // SAA 25 shall be set to 0, SAA 23 shall be set to 1
    *p_seed_acc_addr &= ~CO_BIT(25);
    *p_seed_acc_addr |=  CO_BIT(23);

    // SAA 19 == SAA 15
    // SAA 22 == SAA 16 != SAA 15
    if ((*p_seed_acc_addr & CO_BIT(15)) == 0)
    {
        *p_seed_acc_addr &= ~CO_BIT(19);
        *p_seed_acc_addr |=  CO_BIT(22);
        *p_seed_acc_addr |=  CO_BIT(16);
    }
    else
    {
        *p_seed_acc_addr |=  CO_BIT(19);
        *p_seed_acc_addr &= ~CO_BIT(22);
        *p_seed_acc_addr &= ~CO_BIT(16);
    }

    // DW does not change number of transitions in most significant six bits. Ensure derived AA shall have a minimum of two transitions here.
    if ((*p_seed_acc_addr & CO_BIT(27)) == 0)
    {
        *p_seed_acc_addr |= CO_BIT(26);
        *p_seed_acc_addr |= CO_BIT(28);
    }
    else
    {
        *p_seed_acc_addr &= ~CO_BIT(26);
        *p_seed_acc_addr &= ~CO_BIT(28);
    }
}

/**
 * Pack the BIGInfo ACAD data fields into an array of memory.
 *
 * @param[in]  p_big_info BIGInfo fields unpacked structure
 * @param[out] p_data     Data array packed
 */
__STATIC void lli_big_info_pack(const struct big_info *p_big_info, uint8_t *p_data)
{
    uint32_t temp;
    DBG_MEM_INIT(p_data, BLE_EXT_ACAD_BIG_INFO_ENC_LEN);

    // Bytes 0-3
    // offset information dynamically filled in ACAD data
    temp = 0;
    SETF(temp, BIG_ISO_INTERVAL, p_big_info->iso_interval);
    SETF(temp, BIG_NUM_BIS,      p_big_info->num_bis);
    co_write32p(&(p_data[BIG_OFFSET_POS]), temp);

    // Byte 4
    SETF(p_data[BIG_NSE_POS], BIG_NSE, p_big_info->nse);
    SETF(p_data[BIG_BN_POS],  BIG_BN,  p_big_info->bn);

    // Bytes 5-7
    temp = 0;
    SETF(temp, BIG_SUB_INTERVAL, p_big_info->sub_interval);
    SETF(temp, BIG_PTO,          p_big_info->pto);
    co_write24p(&(p_data[BIG_SUB_INTERVAL_POS]), temp);

    // Bytes 8-10
    temp = 0;
    SETF(temp, BIG_BIS_SPACING,  p_big_info->bis_spacing);
    SETF(temp, BIG_IRC,          p_big_info->irc);
    co_write24p(&(p_data[BIG_BIS_SPACING_POS]), temp);

    // Byte 11
    p_data[BIG_MAX_PDU_POS] = p_big_info->max_pdu;

    // Byte 12
    p_data[BIG_RFU_POS] = 0;

    // Bytes 13-16
    co_write32p(&(p_data[BIG_SEED_ACCESS_ADDRESS_POS]), p_big_info->seed_access_addr);

    // Bytes 17-20
    temp = 0;
    SETF(temp, BIG_SDU_INTERVAL, p_big_info->sdu_interval);
    SETF(temp, BIG_MAX_SDU,      p_big_info->max_sdu);
    co_write32p(&(p_data[BIG_SDU_INTERVAL_POS]), temp);

    // Bytes 21-25
    co_write16p(&(p_data[BIG_BASE_CRC_INIT_POS]), p_big_info->base_crc_init);
    // Chmap Dynamically filled

    // PHY field of byte 27
    SETF(p_data[BIG_PHY_POS],  BIG_PHY,  co_phy_mask_to_rate[p_big_info->phy]);

    // Bytes 28-32
    // BIS_PLD_COUNT Dynamically filled
    // Framing field of byte 32
    SETF(p_data[BIG_FRAMING_POS], BIG_FRAMING, p_big_info->framing);

    if (p_big_info->encrypted)
    {
        // Bytes 33-40
        memcpy(&(p_data[BIG_GIV_POS]),  p_big_info->giv,  IV_LEN);
        // Bytes 41-56
        memcpy(&(p_data[BIG_GSKD_POS]), p_big_info->gskd, KEY_LEN);
    }
}

/**
 ****************************************************************************************
 * @brief Compute parameters for the LE Create BIG Test command
 *
 * @param[in]  p_param Create BIG parameters
 * @param[out] p_comp  Create BIG Test parameters
 ****************************************************************************************
 */
__STATIC uint8_t lli_big_compute_params(const struct hci_le_create_big_cmd *p_param, struct hci_le_create_big_test_cmd *p_comp)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t framing = p_param->framing;
    uint8_t bn, nse, max_pdu;
    uint16_t iso_intv = p_param->sdu_interval / (SLOT_SIZE * 2);

    // These parameters do not change
    p_comp->big_hdl               = p_param->big_hdl;
    p_comp->adv_hdl               = p_param->adv_hdl;
    p_comp->num_bis               = p_param->num_bis;
    p_comp->sdu_interval          = p_param->sdu_interval;
    p_comp->max_sdu               = p_param->max_sdu;
    p_comp->packing               = p_param->packing;
    p_comp->pto                   = 1;
    p_comp->encryption            = p_param->encryption;
    memcpy(p_comp->broadcast_code,  p_param->broadcast_code, KEY_LEN);
    p_comp->phy = 0;
#if (BLE_PHY_CODED_SUPPORT)
    if (p_param->phy & PHY_CODED_BIT)
    {
        p_comp->phy = PHY_CODED_BIT;
    }
#endif // (BLE_PHY_CODED_SUPPORT)
    if (p_param->phy & PHY_1MBPS_BIT)
    {
        p_comp->phy = PHY_1MBPS_BIT;
    }
#if (BLE_PHY_2MBPS_SUPPORT)
    if (p_param->phy & PHY_2MBPS_BIT)
    {
        p_comp->phy = PHY_2MBPS_BIT;
    }
#endif // (BLE_PHY_2MBPS_SUPPORT)

    // If the SDU interval is not a multiple of 1.25 ms
    if (CO_MOD(p_param->sdu_interval, (SLOT_SIZE * 2)))
    {
        framing = ISO_FRAMED_MODE;
    }

    if (framing == ISO_UNFRAMED_MODE)
    {
        // iso_intv is either sdu_interval or 2*sdu_interval_
        uint8_t factor = ((p_param->trans_latency * 1000) >= 2 * iso_intv * (SLOT_SIZE * 2)) ? 2 : 1;
        uint8_t pdu_nb_per_sdu;

        if (p_param->max_sdu <= BLE_ISO_MAX_PAYLOAD_SIZE)
        {
            pdu_nb_per_sdu = 1;
            max_pdu = p_param->max_sdu;
        }
        else
        {
            pdu_nb_per_sdu = CO_DIVIDE_CEIL(p_param->max_sdu, BLE_ISO_MAX_PAYLOAD_SIZE);
            max_pdu = CO_DIVIDE_CEIL(p_param->max_sdu, pdu_nb_per_sdu);
        }

        iso_intv = iso_intv * factor;
        bn = factor * pdu_nb_per_sdu;
    }
    else // (framing == ISO_FRAMED_MODE)
    {
        uint8_t overhead = BLE_ISOAL_SEG_HEADER_SIZE + BLE_ISOAL_TIME_OFFSET_SIZE;
        uint8_t pdu_nb_per_sdu;

        if (p_param->max_sdu <= BLE_ISO_MAX_PAYLOAD_SIZE - overhead)
        {
            pdu_nb_per_sdu = 1;
            max_pdu = p_param->max_sdu + overhead;
        }
        else
        {
            pdu_nb_per_sdu = CO_DIVIDE_CEIL(p_param->max_sdu, (BLE_ISO_MAX_PAYLOAD_SIZE - overhead));
            max_pdu = CO_DIVIDE_CEIL(p_param->max_sdu, pdu_nb_per_sdu) + overhead;
        }

        bn = pdu_nb_per_sdu;
    }

    // Save the number of subevents
    {
        // Check if parameters are valid according to PHY, etc..
        uint8_t tx_rate = co_phy_to_rate[co_phy_mask_to_value[p_comp->phy]];

        // Compute duration of a sub event plus minimum sub-event space
        uint32_t sub_interval = ble_util_pkt_dur_in_us(max_pdu + (p_param->encryption ? MIC_LEN : 0), tx_rate) + BLE_MSS_DUR + lli_env.sub_evt_margin;

        // Consider a bandwidth use of (6/10)
        uint8_t max_nse = (iso_intv * SLOT_SIZE * 2 * 6) / (p_param->num_bis * sub_interval * 10);
        // Make sure that max_nse is a multiple of bn
        max_nse = CO_DIVIDE_CEIL(max_nse, bn) * bn;

        nse = (p_param->rtn + 1) * bn;
        nse = co_max(co_min(nse, max_nse), bn);
    }

    if (status == CO_ERROR_NO_ERROR)
    {
        // Save computed parameters
        p_comp->framing = framing;
        p_comp->iso_interval = iso_intv;
        p_comp->max_pdu = max_pdu;
        p_comp->bn = bn;
        p_comp->nse = nse;
        p_comp->irc = nse / bn;
    }

    return (status);
}
#endif //(BLE_BROADCASTER)

void lli_bis_start_evt_set(lli_bis_start_evt_cb evt_cb)
{
    lli_bi_env.cb_start_evt = evt_cb;
}

/*
 * GLOBAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */



#endif // (BLE_BIS_IN_ROM)
void lli_bi_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLI_BI_PATCH_TYPE, LLI_BI_INIT_FUN_BIT, is_reset);
#if (BLE_BIS_IN_ROM)
    if (is_reset)
    {
        uint8_t hdl;

        // Clean all allocated BIG environments
        for (hdl = 0; hdl < BLE_ISO_GROUP_MAX ; hdl++)
        {
            lli_big_cleanup(hdl);
        }
    }
    // No break

    // Initialize environment
    memset(&lli_bis_env, 0, sizeof(lli_bis_env));
#endif
}



//lli_bis_data_path_set_handler _lli_bis_data_path_set_handler = _lli_bis_data_path_set;

uint8_t lli_bis_data_path_set(uint8_t act_id, uint8_t direction, uint8_t data_path_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, LLI_BIS_DATA_PATH_SET_FUN_BIT, uint8_t, act_id, direction, data_path_type);

#if (BLE_BIS_IN_ROM)

    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    do
    {
        uint8_t exp_direction;
        struct lli_bis_env *p_bis_env = lli_bis_env[act_id];
        struct lli_big_env *p_big_env = NULL;

        if (p_bis_env == NULL)
            break;

        // Load group environment
        status = LLI_BIG_ENV_GET(p_bis_env->grp_hdl, &p_big_env);
        if (status != CO_ERROR_NO_ERROR)
            break;

        // Check direction
        exp_direction = GETB(p_big_env->info, LLI_BIG_BROADCASTER) ? ISO_SEL_TX : ISO_SEL_RX;
        if (exp_direction != direction)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // check BN
        if ((direction == ISO_SEL_TX) && (p_big_env->bn == 0))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Check the SDU size in ISO test mode
#if (BLE_ISOGEN)
        if ((data_path_type == ISO_DP_ISOGEN) && (p_big_env->max_sdu_size < ISO_TEST_PKT_CNT_SIZE))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }
#endif //(BLE_ISOGEN)

        // Check the SDU size in ISOOHCI
#if (BLE_ISOOHCI)
        if ((data_path_type == ISO_DP_ISOOHCI) && (direction == ISO_SEL_TX) && (p_big_env->max_sdu_size > BLE_HCI_ISO_IN_SDU_BUF_SIZE))
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }
#endif //(BLE_ISOOHCI)

        // Check if the data path has already been set up
        if (!data_path_is_disabled(p_bis_env->p_dp))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Load RX data path
        p_bis_env->p_dp = data_path_itf_get(data_path_type, direction);

        // Check if the data path has already been set up
        if (data_path_is_disabled(p_bis_env->p_dp))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Check that stream is enabled
        if (GETF(p_big_env->info, LLI_BIG_STATE) == LLI_BIG_ENABLED)
        {
            // Update the data path to use
            status = lld_isoal_datapath_set(act_id, direction, p_bis_env->p_dp);

            if (p_bis_env->p_dp->cb_local_sync != NULL)
            {
                // Enable the local synchronization on this BIS
                lld_bis_local_sync_en(act_id, p_bis_env->p_dp->cb_local_sync);
            }

#if (BLE_OBSERVER)
            if (p_bis_env->p_dp->cb_peer_sync != NULL)
            {
                // Enable the peer synchronization on this BIS
                lld_bis_peer_sync_en(act_id, p_bis_env->p_dp->cb_peer_sync);
            }
#endif // (BLE_OBSERVER)

#if (BLE_ISOGEN)
            // Special packet counter initialization for ISO transmit test mode / Unframed
            if ((data_path_type == ISO_DP_ISOGEN) && (p_big_env->framing == ISO_UNFRAMED_MODE))
            {
                GLOBAL_INT_DISABLE();

                uint32_t pld_cnt;

                // Get current payload counter
                lld_bis_pld_cnt_get(act_id, &pld_cnt);

                // Initialize the SDU counter
                isogen_sdu_cnt_set(act_id, direction, pld_cnt / (p_big_env->bn * (p_big_env->sdu_interval / (p_big_env->iso_interval * 2 * SLOT_SIZE))));

                GLOBAL_INT_RESTORE();
            }
#endif //(BLE_ISOGEN)
        }
    }
    while (0);
#else // BLE_BIS_IN_ROM
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

#endif //BLE_BIS_IN_ROM
    return (status);
}

uint8_t lli_bis_data_path_remove(uint8_t act_id, uint8_t direction)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, LLI_BIS_DATA_PATH_REMOVE_FUN_BIT, uint8_t, act_id, direction);
#if (BLE_BIS_IN_ROM)
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    struct lli_bis_env *p_bis_env = lli_bis_env[act_id];

    do
    {
        uint8_t exp_direction;
        struct lli_big_env *p_big_env = NULL;

        if (p_bis_env == NULL)
            break;

        // Load group environment
        status = LLI_BIG_ENV_GET(p_bis_env->grp_hdl, &p_big_env);
        if (status != CO_ERROR_NO_ERROR)
            break;

        // Check direction
        exp_direction = GETB(p_big_env->info, LLI_BIG_BROADCASTER) ? ISO_SEL_TX : ISO_SEL_RX;
        if (exp_direction != direction)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Check if data path isn't disabled
        if (data_path_is_disabled(p_bis_env->p_dp))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // if stream is disabled
        if (GETF(p_big_env->info, LLI_BIG_STATE) != LLI_BIG_ENABLED)
            break;

        if (p_bis_env->p_dp->cb_local_sync != NULL)
        {
            // Disable the local synchronization on this BIS
            lld_bis_local_sync_dis(act_id, p_bis_env->p_dp->cb_local_sync);
        }

#if (BLE_OBSERVER)
        if (p_bis_env->p_dp->cb_peer_sync != NULL)
        {
            // Disable the peer synchronization on this BIS
            lld_bis_peer_sync_dis(act_id, p_bis_env->p_dp->cb_peer_sync);
        }
#endif // (BLE_OBSERVER)

        // Remove data path
        lld_isoal_datapath_remove(act_id, direction);

    }
    while (0);

    if (status == CO_ERROR_NO_ERROR)
    {
        // Unregister the data path
        p_bis_env->p_dp = data_path_itf_get(ISO_DP_DISABLE, direction);
    }
#else // BLE_BIS_IN_ROM
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

#endif // BLE_BIS_IN_ROM
    return (status);
}




#if (BLE_BROADCASTER)
/**
 ****************************************************************************************
 * Create new BIG using Test parameters
 ****************************************************************************************
 */
int hci_le_create_big_test_cmd_handler(struct hci_le_create_big_test_cmd *p_param, uint16_t opcode)
{
    // Initialize returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, HCI_LE_CREATE_BIG_TEST_CMD_HANDLER_FUN_BIT, int, p_param, opcode);
#if (BLE_BIS_IN_ROM)
    uint8_t i;

    struct lli_big_env *p_big_env;
    uint8_t  ext_act_id, per_act_id;
    bool     immediate_stream_start = false;

    do
    {
        // parameters needed to check stream validity
        uint32_t total_evt_dur = 0;
        uint32_t iso_interval_us = p_param->iso_interval * (SLOT_SIZE * 2);

        // Check if feature is locally supported
        uint8_t supp_phy_loc_msk = PHY_1MBPS_BIT;
        SETB(supp_phy_loc_msk, PHY_2MBPS, BLE_PHY_2MBPS_SUPPORT);
        SETB(supp_phy_loc_msk, PHY_CODED, BLE_PHY_CODED_SUPPORT);

        // *************************************************************************************************************
        // 0. Check that periodic advertiser exists / Inform Periodic ADV Driver that it is linked to a BIS Link (through LLM)
        // *************************************************************************************************************
        status = llm_adv_per_id_get(p_param->adv_hdl, &ext_act_id, &per_act_id);

        if (status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        // Parse all BIGs
        {
            uint8_t cnt;
            struct lli_big_env *p_env = NULL;

            for (cnt = 0; cnt < BLE_ISO_GROUP_MAX; cnt++)
            {
                if (LLI_BIG_ENV_GET(cnt, &p_env) == CO_ERROR_NO_ERROR)
                {
                    // Check if the periodic advertising train is associated with another BIG
                    if ((p_param->big_hdl != p_env->big_hdl) && (per_act_id == p_env->per_act_id))
                    {
                        status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
                        break;
                    }

                    // Check if the BIG_Handle identifies a BIG that is already created
                    if ((p_param->big_hdl == p_env->big_hdl) && (GETF(p_env->info, LLI_BIG_STATE) == LLI_BIG_ENABLED))
                    {
                        status = CO_ERROR_COMMAND_DISALLOWED;
                        break;
                    }
                }
            }

            if (cnt < BLE_ISO_GROUP_MAX)
            {
                break;
            }
        }

        // Check provided parameters
        if ((p_param->big_hdl > BLE_BIG_MAX_HANDLE)
                || (p_param->num_bis < BLE_BIS_MIN_NB)                || (p_param->num_bis > BLE_BIS_MAX_NB)
                || (p_param->iso_interval < BLE_ISO_MIN_INTERVAL)     || (p_param->iso_interval > BLE_ISO_MAX_INTERVAL)
                || (p_param->sdu_interval < BLE_ISO_MIN_SDU_INTERVAL) || (p_param->sdu_interval > BLE_ISO_MAX_SDU_INTERVAL)
                || (p_param->nse < BLE_ISO_MIN_NSE)                   || (p_param->nse > BLE_ISO_MAX_NSE)
                || (p_param->max_pdu > BLE_ISO_MAX_PAYLOAD_SIZE)      || (p_param->max_pdu == 0)
                || (p_param->max_sdu > BLE_ISO_MAX_SDU_SIZE)
                || (p_param->packing >= ISO_PACKING_MAX)               || (p_param->framing >= ISO_FRAME_MODE_MAX)
                || (p_param->bn < BLE_BIS_MIN_BN)                     || (p_param->bn > BLE_BIS_MAX_BN)
                || (p_param->irc < BLE_BIS_MIN_IRC)                   || (p_param->irc > BLE_BIS_MAX_IRC)
                || (p_param->pto > BLE_BIS_MAX_PTO))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // If the value of the NSE parameter is not an integer multiple of BN or NSE is less than (IRC * BN)
        if ((p_param->nse % p_param->bn) || (p_param->nse < p_param->irc * p_param->bn))
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }

        // If pretransmissions are required
        if (p_param->nse > p_param->irc * p_param->bn)
        {
            // Reject if PTO is 0
            if (p_param->pto == 0)
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }
            else // Check if the number of buffers is sufficient for the pretransmissions
            {
                uint8_t gc = p_param->nse / p_param->bn;
                uint16_t buf_nb = p_param->pto * (gc - p_param->irc) * p_param->bn + p_param->bn;

                if (buf_nb > EM_BLE_ISO_BUF_NB)
                {
                    status = CO_ERROR_UNSUPPORTED;
                    break;
                }
            }
        }

        // ISO interval must be a multiple of SDU interval in Unframed Mode
        if (p_param->framing == ISO_UNFRAMED_MODE)
        {
            if (CO_MOD(iso_interval_us, p_param->sdu_interval) != 0)
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }
            else
            {
                uint16_t max_sdu = p_param->max_pdu * ((p_param->bn * p_param->sdu_interval) / iso_interval_us);

                // check that maximum SDU size requested can be transmitted
                if (p_param->max_sdu > max_sdu)
                {
                    status = CO_ERROR_INVALID_HCI_PARAM;
                    break;
                }
            }
        }
        else
        {
            uint16_t max_sdu = (p_param->max_pdu - BLE_ISOAL_SEG_HEADER_SIZE) * p_param->bn
                               * CO_DIVIDE_CEIL(p_param->sdu_interval, iso_interval_us)
                               - BLE_ISOAL_TIME_OFFSET_SIZE;

            // check that maximum SDU size requested can be transmitted
            if (p_param->max_sdu > max_sdu)
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }
        }

        // Check if PHY proposed are supported
        if (((p_param->phy & supp_phy_loc_msk) == 0)  || (NB_ONE_BITS(p_param->phy) != 1))
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }
        else
        {
            uint32_t ctrl_sub_evt_offset  = 0;
            uint32_t bis_spacing;
            uint32_t sub_interval;
            uint16_t sub_evt_dur;
            uint16_t ctrl_sub_evt_dur;
            uint8_t  tx_rate;
            uint8_t  packing;

            // Check if parameters are valid according to PHY, etc..
            tx_rate = co_phy_to_rate[co_phy_mask_to_value[p_param->phy]];

            // Compute duration of a sub event
            sub_evt_dur      = ble_util_pkt_dur_in_us(p_param->max_pdu + (p_param->encryption ? MIC_LEN : 0), tx_rate);
            ctrl_sub_evt_dur = ble_util_pkt_dur_in_us(BLE_BIG_MAX_CTRL_PDU_LEN + (p_param->encryption ? MIC_LEN : 0), tx_rate) + BLE_MSS_DUR;

            // retrieve scheduling method
            packing = p_param->packing;

            // add the margin
            sub_interval = sub_evt_dur + BLE_MSS_DUR + lli_env.sub_evt_margin;

            // compute cis timing parameters for interleave scheduling
            if (packing == ISO_PACKING_INTERLEAVED)
            {
                bis_spacing         = sub_interval;
                sub_interval        = sub_interval * p_param->num_bis;
                ctrl_sub_evt_offset = (sub_interval * p_param->nse);
            }
            // compute cis timing parameters for sequential scheduling
            else // Sequential
            {
                bis_spacing         = sub_interval * p_param->nse;
                ctrl_sub_evt_offset = (bis_spacing * p_param->num_bis);
            }

            // Compute total duration of event
            total_evt_dur = ctrl_sub_evt_offset + ctrl_sub_evt_dur;

            // check if all sub event can enter into the channel interval
            if (total_evt_dur > ((uint32_t)p_param->iso_interval * SLOT_SIZE * 2))
            {
                status = CO_ERROR_UNACCEPTABLE_CONN_PARAM;
                break;
            }

            // Check if Group already exist, if yes reject
            p_big_env = lli_big_get(p_param->big_hdl);

            if (p_big_env == NULL)
            {
                struct lli_bis_env *p_bis_env;
                uint16_t act_offset;

                // Allocate a new BIG environment
                status = lli_big_create(&(p_big_env), p_param->num_bis);

                // An error occurs
                if (status != CO_ERROR_NO_ERROR)
                {
                    if (status == CO_ERROR_MEMORY_CAPA_EXCEED)
                    {
                        status = CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
                    }
                    break;
                }

                // Store framing mode
                p_big_env->framing = p_param->framing;

                // allocate a stream environment for all streams of the BIG
                for (i = 0 ; i < p_param->num_bis ; i++)
                {
                    uint8_t act_id;
                    status = lli_bis_create(&p_bis_env, ROLE_BROADCASTER, p_big_env->group.hdl, i + 1, &act_id);

                    // check if activity allocation fails
                    if (status != CO_ERROR_NO_ERROR)
                    {
                        break;
                    }

                    // keep channel handle
                    p_big_env->bis_act_id[i] = act_id;
                    p_big_env->nb_bis++;
                }

                // Try to attach Periodic advertising
                if (status == CO_ERROR_NO_ERROR)
                {
                    status = llm_adv_big_attach(ext_act_id, per_act_id, p_big_env->act_id);
                }

                // An error occurs
                if (status != CO_ERROR_NO_ERROR)
                {
                    // Detach periodic advertising
                    llm_adv_big_detach(ext_act_id, per_act_id, p_big_env->group.hdl);

                    // Cleanup BIG
                    lli_big_cleanup(p_big_env->group.hdl);
                }
                else
                {
                    // BIG Driver parameters
                    struct lld_big_params  *p_big_params;
                    struct big_info        *p_big_info;

                    struct sch_plan_req_param req_param;
                    struct sch_plan_elt_tag *plan_elt = llm_plan_elt_get(p_big_env->act_id);

                    // Request an offset range to the planner
                    req_param.interval_min    = p_param->iso_interval << 2;
                    req_param.interval_max    = req_param.interval_min;
                    req_param.duration_min    = (2 * total_evt_dur + (HALF_SLOT_SIZE - 1)) / HALF_SLOT_SIZE;
                    req_param.duration_max    = req_param.duration_min;
                    req_param.offset_min      = 0;
                    req_param.offset_max      = req_param.interval_max - 1;
                    req_param.margin          = 1;
                    req_param.pref_period     = 0;
                    req_param.conhdl          = BLE_ACTID_TO_BISHDL(p_big_env->act_id);
                    req_param.conhdl_ref      = req_param.conhdl;

                    if (sch_plan_req(&req_param) != SCH_PLAN_ERROR_OK)
                    {
                        status = CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
                        break;
                    }

                    // As BIG timings are deterministic (its event duration is fixed and accurate), BIG is placed right before another concurrent activity
                    act_offset = req_param.offset_max;

                    // Register BIS in the planner
                    plan_elt->offset       = act_offset;
                    plan_elt->interval     = req_param.interval_max;
                    plan_elt->duration_min = req_param.duration_min;
                    plan_elt->duration_max = req_param.duration_max;
                    plan_elt->margin       = 1;
                    plan_elt->conhdl       = req_param.conhdl;
                    plan_elt->conhdl_ref   = req_param.conhdl;
                    plan_elt->cb_move      = NULL;
                    plan_elt->mobility     = SCH_PLAN_MB_LVL_0;
                    sch_plan_set(plan_elt);

                    // Save BIG parameters
                    p_big_env->big_hdl         = p_param->big_hdl;
                    p_big_env->ext_act_id      = ext_act_id;
                    p_big_env->per_act_id      = per_act_id;
                    p_big_env->info            = 0;
                    SETF(p_big_env->info, LLI_BIG_STATE,       LLI_BIG_DISABLED);
                    SETB(p_big_env->info, LLI_BIG_BROADCASTER, true);
                    SETB(p_big_env->info, LLI_BIG_ENCRYPTED,   p_param->encryption);

                    p_big_env->nb_bis          = p_param->num_bis;

                    p_big_env->max_sdu_size    = p_param->max_sdu;
                    p_big_env->max_pdu         = p_param->max_pdu;
                    p_big_env->bn              = p_param->bn;
                    p_big_env->iso_interval    = p_param->iso_interval;
                    p_big_env->sdu_interval    = p_param->sdu_interval;

                    // Allocate start info structure from non-retention memory - will be free before having reason to go in deep sleep
                    p_big_env->p_start = (struct lli_bi_start_params *) ke_malloc_system(sizeof(struct lli_bi_start_params) + BLE_EXT_ACAD_BIG_INFO_ENC_LEN, KE_MEM_NON_RETENTION);

                    // *************************************************************************************************************
                    // Create stream info structure needed to start BIS driver
                    // *************************************************************************************************************
                    // Fill Stream information
                    p_big_info = &(p_big_env->p_start->info);

                    p_big_info->num_bis                = p_big_env->nb_bis;
                    p_big_info->iso_interval           = p_param->iso_interval;
                    p_big_info->sub_interval           = sub_interval;
                    p_big_info->bis_spacing            = bis_spacing;
                    p_big_info->phy                    = co_rate_to_phy_mask[tx_rate];
                    p_big_info->max_pdu                = p_param->max_pdu;
                    p_big_info->max_sdu                = p_param->max_sdu;
                    p_big_info->nse                    = p_param->nse;
                    p_big_info->bn                     = p_param->bn;
                    p_big_info->irc                    = p_param->irc;
                    p_big_info->pto                    = p_param->pto;
                    p_big_info->encrypted              = p_param->encryption;
                    p_big_info->framing                = p_param->framing;
                    p_big_info->sdu_interval           = p_param->sdu_interval;

                    // Fill driver parameters
                    p_big_params = &(p_big_env->p_start->drv_params);

                    p_big_params->iso_interval_us      = ((uint32_t)p_big_info->iso_interval * SLOT_SIZE * 2);
                    p_big_params->ctrl_sub_evt_offset  = ctrl_sub_evt_offset;
                    p_big_params->sub_evt_dur          = sub_evt_dur;
                    p_big_params->ctrl_sub_evt_dur     = ctrl_sub_evt_dur;
                    p_big_params->act_offset           = act_offset;
                    p_big_params->grp_hdl              = p_big_env->group.hdl;
                    p_big_params->role                 = ROLE_BROADCASTER;
                    p_big_params->max_pdu              = p_param->max_pdu;
                    p_big_params->rate                 = tx_rate;
                    p_big_params->packing              = packing;

                    // one more channel required for Update sub-event
                    p_big_params->bis_params[0].act_id = p_big_env->act_id;
                    p_big_params->bis_params[0].idx    = 0;

                    // Configure all other streams
                    for (i = 0 ; i < p_big_env->nb_bis ; i++)
                    {
                        p_big_params->bis_params[i + 1].act_id = p_big_env->bis_act_id[i];
                        p_big_params->bis_params[i + 1].idx    = i + 1;
                    }

                    // *************************************************************************************************************
                    // If AccessCode present, compute GLTK and GSK (group session key)
                    // *************************************************************************************************************
                    if (p_param->encryption)
                    {
                        uint32_t metainfo = 0;

                        // Copy Access code in environment
                        memcpy(p_big_env->broadcast_code,  p_param->broadcast_code, KEY_LEN);
                        memset(p_big_info->bis_pkt_cnt, 0,                            BLE_PLD_CNT_SIZE);

                        // Mark stream starting
                        SETF(p_big_env->info, LLI_BIG_STATE, LLI_BIG_STARTING);
                        SETF(metainfo, LLI_BI_ENC_GRP_HDL, p_big_env->group.hdl);

                        // Request generation of IGLTK Using Encryption code
                        SETF(metainfo, LLI_BI_ENC_STATE,   LLI_BI_ENC_STATE_GEN_IGLTK);
                        aes_h7(lli_bis_h7_salt, p_big_env->broadcast_code, lli_bis_aes_result_cb, metainfo);
                    }
                    // *************************************************************************************************************
                    // Else start the BIS driver with computed parameters.
                    // *************************************************************************************************************
                    else
                    {
                        // mark that stream can be immediately started
                        immediate_stream_start = true;
                    }

                    // Request operation is finished and no error has occurred
                    status = CO_ERROR_NO_ERROR;
                }
            }
        }
    }
    while (0);

    // Allocate the status event to be sent to the host
    struct hci_cmd_stat_event *p_evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
    // Send the command complete event
    p_evt->status   = status;
    hci_send_2_host(p_evt);

    // check if driver can be started
    if (immediate_stream_start)
    {
        lli_big_start(p_big_env);
    }
#else // (BLE_BIS_IN_ROM)
    // Allocate the status event to be sent to the host
    struct hci_cmd_stat_event *p_evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
    // Send the command complete event
    p_evt->status   = CO_ERROR_COMMAND_DISALLOWED;
    hci_send_2_host(p_evt);
#endif // (BLE_BIS_IN_ROM)
    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * Create new BIG
 ****************************************************************************************
 */
int hci_le_create_big_cmd_handler(struct hci_le_create_big_cmd *p_param, uint16_t opcode)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, HCI_LE_CREATE_BIG_CMD_HANDLER_FUN_BIT, int, p_param, opcode);
#if (BLE_BIS_IN_ROM)

    struct hci_le_create_big_test_cmd test_param;

    // Check provided parameters
    if ((p_param->big_hdl > BLE_BIG_MAX_HANDLE)
            || (p_param->adv_hdl > ADV_HDL_MAX)
            || (p_param->num_bis < BLE_BIS_MIN_NB)                  || (p_param->num_bis > BLE_BIS_MAX_NB)
            || (p_param->sdu_interval < BLE_ISO_MIN_SDU_INTERVAL)   || (p_param->sdu_interval > BLE_ISO_MAX_SDU_INTERVAL)
            || (p_param->max_sdu < BLE_ISO_MIN_SDU_SIZE)            || (p_param->max_sdu > BLE_ISO_MAX_SDU_SIZE)
            || (p_param->trans_latency < BLE_ISO_MIN_TRANS_LATENCY) || (p_param->trans_latency > BLE_ISO_MAX_TRANS_LATENCY)
            || (p_param->rtn > BLE_BIS_MAX_RTN)
            || (p_param->phy > PHY_ALL)
            || (p_param->packing >= ISO_PACKING_MAX)
            || (p_param->framing >= ISO_FRAME_MODE_MAX)
            || (p_param->encryption > ENCRYPTION_ON))
    {
        struct hci_cmd_stat_event *p_evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);

        // Send the command complete event - RFU settings as unsupported parameter values
        p_evt->status   = CO_ERROR_UNSUPPORTED;
        hci_send_2_host(p_evt);
    }
    else
    {
        // compute parameters into test parameter format
        uint8_t status = lli_big_compute_params(p_param, &test_param);

        if (status == CO_ERROR_NO_ERROR)
        {
            // use test command
            hci_le_create_big_test_cmd_handler(&test_param, opcode);
        }
        else
        {
            // Allocate the status event to be sent to the host
            struct hci_cmd_stat_event *p_evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
            // Send the command complete event
            p_evt->status   = status;
            hci_send_2_host(p_evt);
        }
    }

#else // (BLE_BIS_IN_ROM)
    // Allocate the status event to be sent to the host
    struct hci_cmd_stat_event *p_evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
    // Send the command complete event
    p_evt->status   = CO_ERROR_COMMAND_DISALLOWED;
    hci_send_2_host(p_evt);

#endif //(BLE_BIS_IN_ROM)
    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

#endif //(BLE_BROADCASTER)

#if (BLE_OBSERVER)
/**
 ****************************************************************************************
 * Acquire a BIG (Broadcast receiver side)
 ****************************************************************************************
 */
int hci_le_big_create_sync_cmd_handler(struct hci_le_big_create_sync_cmd const *p_param, uint16_t opcode)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, HCI_LE_BIG_CREATE_SYNC_CMD_HANDLER_FUN_BIT, int, p_param, opcode);
    // Allocate the status event to be sent to the host
    struct hci_cmd_stat_event *p_evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
#if    (BLE_BIS_IN_ROM)
    // Command status
    uint8_t status = CO_ERROR_NO_ERROR;
    struct lli_big_env *p_big_env = NULL;
    struct lli_bis_env *p_bis_env;
    uint8_t cursor = 0;

    do
    {
        // *************************************************************************************************************
        // Check input parameters
        // *************************************************************************************************************

        // Check if stream already exist, if yes reject
        p_big_env = lli_big_get(p_param->big_hdl);

        // if stream already exists, reject request
        if (p_big_env != NULL)
        {
            p_big_env = NULL; // Do not perform cleanup
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Check if the sync handle is already in use
        {
            struct lli_big_env *p_env = NULL;

            for (uint8_t cnt = 0; cnt < BLE_ISO_GROUP_MAX; cnt++)
            {
                if ((LLI_BIG_ENV_GET(cnt, &p_env) == CO_ERROR_NO_ERROR) && (p_env->per_act_id == BLE_SYNCHDL_TO_ACTID(p_param->sync_hdl)))
                {
                    status = CO_ERROR_COMMAND_DISALLOWED;
                    break;
                }
            }

            if (status != CO_ERROR_NO_ERROR)
            {
                break;
            }
        }

        // check that at least one channel is expected
        if (p_param->num_bis == 0)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // perform a sanity check on HCI parameters
        for (cursor = 0 ; cursor < p_param->num_bis ; cursor++)
        {
            if ((p_param->bis_id[cursor] < BLE_BIS_MIN_NB) || (p_param->bis_id[cursor] > BLE_BIS_MAX_NB))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }
        }

        // clear cursor because needed for channel/stream clean-up in case of error
        cursor = 0;

        // check if an error occurs
        if (status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        // *************************************************************************************************************
        // Check that there is enough activities for the BIS (reserve them to be ready as soon as possible)
        // *************************************************************************************************************

        if (p_big_env == NULL)
        {
            // Allocate a new stream environment
            status = lli_big_create(&(p_big_env), p_param->num_bis);

            // An error occurs
            if (status != CO_ERROR_NO_ERROR)
            {
                break;
            }

            // Allocate a stream environment for all streams of the BIG
            for (cursor = 0 ; cursor < p_param->num_bis ; cursor++)
            {
                uint8_t act_id;
                status = lli_bis_create(&p_bis_env, ROLE_BROADCAST_RECEIVER, p_big_env->group.hdl, p_param->bis_id[cursor], &act_id);

                // check if activity allocation fails
                if (status != CO_ERROR_NO_ERROR)
                {
                    if (status == CO_ERROR_MEMORY_CAPA_EXCEED)
                    {
                        status = CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
                    }
                    break;
                }

                // keep channel handle
                p_big_env->bis_act_id[cursor] = act_id;
                p_big_env->nb_bis++;
            }

            // An error occurs
            if (status != CO_ERROR_NO_ERROR)
            {
                break;
            }

            // Save BIS parameters
            p_big_env->big_hdl      = p_param->big_hdl;
            p_big_env->per_act_id   = BLE_SYNCHDL_TO_ACTID(p_param->sync_hdl);

            p_big_env->info         = 0;
            SETF(p_big_env->info, LLI_BIG_STATE,       LLI_BIG_STARTING);
            SETB(p_big_env->info, LLI_BIG_BROADCASTER, false);
            SETB(p_big_env->info, LLI_BIG_ENCRYPTED,   p_param->encryption);

            p_big_env->max_se       = p_param->mse;
            p_big_env->sync_timeout = p_param->big_sync_timeout;

            p_big_env->nb_bis       = p_param->num_bis;
            p_big_env->p_start      = NULL;

            if (p_param->encryption)
            {
                memcpy(p_big_env->broadcast_code, p_param->broadcast_code, KEY_LEN);
            }
        }

        // *************************************************************************************************************
        // Mark that BIS procedure is waiting for Periodic ADV data packet with ACAD information
        // *************************************************************************************************************
        status = llm_scan_sync_acad_attach(p_big_env->per_act_id, BLE_EXT_ACAD_BIG_INFO_AD_TYPE, TASK_LLI);

        if (status != CO_ERROR_NO_ERROR)
        {
            break;
        }

    }
    while (0);

    // perform clean-up in case of error
    if ((status != CO_ERROR_NO_ERROR) && (p_big_env != NULL))
    {
        // Cleanup BIG
        lli_big_cleanup(p_big_env->group.hdl);
    }
#else // (BLE_BIS_IN_ROM)

    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

#endif // (BLE_BIS_IN_ROM)
    // Set status and send the complete event
    p_evt->status = status;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}
#endif //(BLE_OBSERVER)

#if (BLE_BROADCASTER)
/**
 ****************************************************************************************
 * Terminate BIG
 ****************************************************************************************
 */
int hci_le_terminate_big_cmd_handler(struct hci_le_terminate_big_cmd const *p_param, uint16_t opcode)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, HCI_LE_TERMINATE_BIG_CMD_HANDLER_FUN_BIT, int, p_param, opcode);
    // Allocate the status event to be sent to the host
    struct hci_cmd_stat_event *p_evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
#if (BLE_BIS_IN_ROM)
    // Command status
    uint8_t status = CO_ERROR_NO_ERROR;
    struct lli_big_env *p_big_env;
    bool canceled = false;

    do
    {
        // Check if stream already exist, if not reject
        p_big_env = lli_big_get(p_param->big_hdl);
        if (p_big_env == NULL)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        if (!GETB(p_big_env->info, LLI_BIG_BROADCASTER))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // check that stream is enabled
        if ((GETF(p_big_env->info, LLI_BIG_STATE) != LLI_BIG_ENABLED)
                // or in starting state
                && (GETF(p_big_env->info, LLI_BIG_STATE) != LLI_BIG_STARTING))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Inform Periodic ADV Driver that it must stop information transmission about BIS Link (through LLM)
        llm_adv_big_detach(p_big_env->ext_act_id, p_big_env->per_act_id, p_big_env->act_id);

        p_big_env->host_reason = p_param->reason;

        if (GETF(p_big_env->info, LLI_BIG_STATE) == LLI_BIG_ENABLED)
        {
            // No Channel Map update on-going
            if (!GETB(p_big_env->info, LLI_BIG_CH_MAP_UP))
            {
                // compute BIG termination instant
                uint16_t instant = lld_big_event_counter_get(p_big_env->group.hdl) + LLI_BIS_NB_TX_ATTEMPT;

                // Send BIS TERMINATE_IND (6 times)
                status = lli_big_terminate_ind_send(p_big_env->group.hdl, p_param->reason, instant, LLI_BIS_NB_TX_ATTEMPT);
            }
            else
            {
                // Mark that BIG termination packet shall be sent
                SETB(p_big_env->info, LLI_BIG_TERM_REQ, true);
            }

            // mark that stream can be stopped
            SETF(p_big_env->info, LLI_BIG_STATE, (status == CO_ERROR_NO_ERROR) ? LLI_BIG_STOPPING : LLI_BIG_DISABLED);
        }
        // BIG can be immediately removed
        else
        {
            canceled = true;
        }
    }
    while (0);

    // Set status and send the complete event
    p_evt->status = status;
    hci_send_2_host(p_evt);

    if (canceled)
    {
        lli_big_create_evt_send(p_big_env, 0, CO_ERROR_OPERATION_CANCELED_BY_HOST, 0, 0);

        // Cleanup BIG
        lli_big_cleanup(p_big_env->group.hdl);
    }
#else // (BLE_BIS_IN_ROM)
    p_evt->status = CO_ERROR_COMMAND_DISALLOWED;
    hci_send_2_host(p_evt);
#endif // (BLE_BIS_IN_ROM)
    // Message can be consumed
    return (KE_MSG_CONSUMED);
}
#endif // (BLE_BROADCASTER)

#if (BLE_OBSERVER)
/**
 ****************************************************************************************
 * Terminate BIG
 ****************************************************************************************
 */
int hci_le_big_terminate_sync_cmd_handler(struct hci_le_big_terminate_sync_cmd const *p_param, uint16_t opcode)
{
    FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, HCI_LE_BIG_TERMINATE_SYNC_CMD_HANDLER_FUN_BIT, int, p_param, opcode);
#if    (BLE_BIS_IN_ROM)

    // Command status
    uint8_t status = CO_ERROR_NO_ERROR;
    struct lli_big_env *p_big_env;
    bool canceled = false;

    do
    {
        // Check if stream already exist, if not reject
        p_big_env = lli_big_get(p_param->big_hdl);
        if (p_big_env == NULL)
        {
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // Check that the device is not broadcaster
        if (GETB(p_big_env->info, LLI_BIG_BROADCASTER))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // check that stream is enabled
        if ((GETF(p_big_env->info, LLI_BIG_STATE) != LLI_BIG_ENABLED)
                // or in starting state
                && (GETF(p_big_env->info, LLI_BIG_STATE) != LLI_BIG_STARTING))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }
        if (GETF(p_big_env->info, LLI_BIG_STATE) == LLI_BIG_ENABLED)
        {
            // stop the driver
            status = lld_big_stop(p_big_env->group.hdl, CO_ERROR_CON_TERM_BY_LOCAL_HOST);

            if (status == CO_ERROR_NO_ERROR)
            {
                // mark that stream can be stopped
                SETF(p_big_env->info, LLI_BIG_STATE, LLI_BIG_STOPPING);
            }
            else
            {
                // Cleanup BIG
                lli_big_cleanup(p_big_env->group.hdl);
            }
        }
        else
        {
            // detach activity to not receive anymore ACAD Data reports
            llm_scan_sync_acad_detach(p_big_env->per_act_id);
            canceled = true;
        }

    }
    while (0);

    if (canceled)
    {
        // inform that Sync has been canceled
        lli_big_create_sync_evt_send(p_big_env, CO_ERROR_OPERATION_CANCELED_BY_HOST, 0);

        // Cleanup BIG
        lli_big_cleanup(p_big_env->group.hdl);
    }

    // Only send the command complete event here if the driver is not stopping or sync establishment canceled
    if (canceled || (status != CO_ERROR_NO_ERROR))
    {
        // Send terminate sync complete event
        struct hci_le_big_terminate_sync_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_BIG_TERMINATE_SYNC_CMD_OPCODE, hci_le_big_terminate_sync_cmd_cmp_evt);
        p_evt->status = status;
        p_evt->big_hdl = p_param->big_hdl;
        hci_send_2_host(p_evt);
    }
#else // (BLE_BIS_IN_ROM)
    // Send terminate sync complete event
    struct hci_le_big_terminate_sync_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_BIG_TERMINATE_SYNC_CMD_OPCODE, hci_le_big_terminate_sync_cmd_cmp_evt);
    p_evt->status = CO_ERROR_COMMAND_DISALLOWED;
    p_evt->big_hdl = p_param->big_hdl;
    hci_send_2_host(p_evt);

#endif // (BLE_BIS_IN_ROM)
    // Message can be consumed
    return (KE_MSG_CONSUMED);
}
#endif // (BLE_OBSERVER)

/**
 ****************************************************************************************
 * Message used to inform about BIS driver termination.
 ****************************************************************************************
 */
int lld_big_stop_ind_handler(ke_msg_id_t const msgid, struct lld_big_stop_ind const *param,
                             ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, LLD_BIG_STOP_IND_HANDLER_FUN_BIT, int, msgid, param, dest_id, src_id);
#if (BLE_BIS_IN_ROM)
    // Pointer to the BIG information structure
    struct lli_big_env *p_big_env;

    if (LLI_BIG_ENV_GET(param->grp_hdl, &p_big_env) == CO_ERROR_NO_ERROR)
    {
#if (BLE_BROADCASTER)
        if (GETB(p_big_env->info, LLI_BIG_BROADCASTER))
        {
            // Send event to host
            lli_big_terminate_evt_send(p_big_env->big_hdl, p_big_env->host_reason);
        }
        else
#endif // (BLE_BROADCASTER)
        {
#if (BLE_OBSERVER)
            // if driver terminated by host
            if (param->reason == CO_ERROR_CON_TERM_BY_LOCAL_HOST)
            {
                // Send terminate sync complete event
                struct hci_le_big_terminate_sync_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_BIG_TERMINATE_SYNC_CMD_OPCODE, hci_le_big_terminate_sync_cmd_cmp_evt);
                p_evt->status = CO_ERROR_NO_ERROR;
                p_evt->big_hdl = p_big_env->big_hdl;
                hci_send_2_host(p_evt);
            }
            // unexpected termination
            else
            {
                // inform that we get sync lost
                lli_big_sync_lost_evt_send(p_big_env->big_hdl, param->reason);
            }
#endif // (BLE_OBSERVER)

        }

        // Cleanup BIG
        lli_big_cleanup(p_big_env->group.hdl);
    }
#endif //(BLE_BIS_IN_ROM)

    return (KE_MSG_CONSUMED);
}

#if (BLE_BROADCASTER)
/**
 ****************************************************************************************
 * Message used to confirm that LL control packet is transmitted.
 ****************************************************************************************
 */
int lld_big_tx_ind_handler(ke_msg_id_t const msgid, struct lld_big_tx_ind const *param,
                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, LLD_BIG_TX_IND_HANDLER_FUN_BIT, int, msgid, param, dest_id, src_id);
#if (BLE_BIS_IN_ROM)
    struct lli_big_env *p_big_env;

    do
    {
        // load goup environment
        if (LLI_BIG_ENV_GET(param->grp_hdl, &p_big_env) != CO_ERROR_NO_ERROR)
        {
            break;
        }

        if (GETB(p_big_env->info, LLI_BIG_CH_MAP_UP))
        {
            SETB(p_big_env->info, LLI_BIG_CH_MAP_UP, false);

            if (GETB(p_big_env->info, LLI_BIG_TERM_REQ))
            {
                // compute BIG termination instant
                uint16_t instant = lld_big_event_counter_get(p_big_env->group.hdl) + LLI_BIS_NB_TX_ATTEMPT;

                SETB(p_big_env->info, LLI_BIG_TERM_REQ, false);

                if (lli_big_terminate_ind_send(p_big_env->group.hdl, p_big_env->host_reason, instant, LLI_BIS_NB_TX_ATTEMPT) != CO_ERROR_NO_ERROR)
                {
                    ASSERT_ERR_FORCE(0);
                }
            }

            break;
        }

        // check that group is stopping
        if (GETF(p_big_env->info, LLI_BIG_STATE) == LLI_BIG_STOPPING)
        {
            lld_big_stop(p_big_env->group.hdl, (p_big_env->host_reason == CO_ERROR_REMOTE_USER_TERM_CON)
                         ? CO_ERROR_CON_TERM_BY_LOCAL_HOST : p_big_env->host_reason);
        }
    }
    while (0);
#endif // (BLE_BIS_IN_ROM)

    return (KE_MSG_CONSUMED);
}

void lli_bi_ch_map_update(struct le_ch_map *p_llm_ch_map)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LLI_BI_PATCH_TYPE, LLI_BI_CH_MAP_UPDATE_FUN_BIT, p_llm_ch_map);
#if (BLE_BIS_IN_ROM)
    uint8_t nbchgood = 0;

    // Copy LLM channel map to local channel map
    nbchgood = ble_util_nb_good_channels(p_llm_ch_map);

    // Check if the map has a sufficient number of used channels
    if (nbchgood >= DATA_CHANNEL_USED_NB_MIN)
    {
        uint8_t grp_hdl;
        for (grp_hdl = 0 ; grp_hdl < BLE_ISO_GROUP_MAX ; grp_hdl++)
        {
            struct lli_big_env *p_big_env;
            uint16_t instant;

            // load group environment
            if ((LLI_BIG_ENV_GET(grp_hdl, &p_big_env) != CO_ERROR_NO_ERROR)
                    // check that group is enabled
                    || (GETF(p_big_env->info, LLI_BIG_STATE) != LLI_BIG_ENABLED)
                    // check that the role is broadcaster
                    || (!GETB(p_big_env->info, LLI_BIG_BROADCASTER))
                    // channel map update procedure on-going
                    || GETB(p_big_env->info, LLI_BIG_CH_MAP_UP)
                    // no update of the channel map
                    || (memcmp(p_big_env->ch_map.map, p_llm_ch_map->map, LE_CH_MAP_LEN) == 0))
            {
                continue;
            }

            // compute channel map update instant
            instant = lld_big_event_counter_get(grp_hdl) + LLI_BIS_NB_TX_ATTEMPT;

            // prepare channel map update at instant
            if (lld_big_ch_map_update(grp_hdl, p_llm_ch_map, instant) != CO_ERROR_NO_ERROR)
            {
                continue;
            }

            // Send the channel map control message
            if (lli_big_ch_map_update_ind_send(grp_hdl, p_llm_ch_map->map, instant, LLI_BIS_NB_TX_ATTEMPT) != CO_ERROR_NO_ERROR)
            {
                ASSERT_INFO_FORCE(0, grp_hdl, instant);
            }

            // mark that channel map update procedure on-going
            SETB(p_big_env->info, LLI_BIG_CH_MAP_UP, true);

            // update channel map of BIG
            memcpy(p_big_env->ch_map.map, p_llm_ch_map->map, LE_CH_MAP_LEN);
        }
    }
#endif //(BLE_BIS_IN_ROM)

}
#endif // (BLE_BROADCASTER)

#if (BLE_OBSERVER)
/**
 ****************************************************************************************
 * Inform that LL packet has been received
 ****************************************************************************************
 */
int lld_big_rx_ind_handler(ke_msg_id_t const msgid, struct lld_big_rx_ind const *param,
                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, LLD_BIG_RX_IND_HANDLER_FUN_BIT, int, msgid, param, dest_id, src_id);
#if (BLE_BIS_IN_ROM)
    // Pointer to the BIG information structure
    struct lli_big_env *p_big_env;

    if (LLI_BIG_ENV_GET(param->grp_hdl, &p_big_env) == CO_ERROR_NO_ERROR)
    {
        uint8_t status;
        uint8_t rx_data[BIG_PDU_LENGTH_MAX];
        union big_pdu pdu;

        // load pdu data
        em_rd(rx_data, param->em_buf, BIG_PDU_LENGTH_MAX);
        pdu.op_code = rx_data[0];

        // Check op_code
        if (pdu.op_code >= BIG_OPCODE_MAX_OPCODE)
        {
            status = CO_ERROR_UNKNOWN_LMP_PDU;
        }
        // check message length
        else if (param->length != lli_big_pdu_handler[pdu.op_code].length)
        {
            status = CO_ERROR_INVALID_LMP_PARAM; // incorrect length
        }
        else
        {
            uint16_t pdu_len = sizeof(union llcp_pdu);

            // Perform PDU unpacking
            status = co_util_unpack((uint8_t *) & (pdu), rx_data, &pdu_len, param->length,
                                    lli_big_pdu_handler[pdu.op_code].pack_format);

            // check if unpack succeed or not
            if (status == CO_UTIL_PACK_OK)
            {
                status = CO_ERROR_NO_ERROR;
            }
            else
            {
                status = CO_ERROR_INVALID_LMP_PARAM;
            }
        }

        // valid BIS PDU
        if (status == CO_ERROR_NO_ERROR)
        {
            ASSERT_INFO(pdu.op_code < BIG_OPCODE_MAX_OPCODE, dest_id, pdu.op_code);

            // execute message handler
            lli_big_pdu_handler[pdu.op_code].handler(param->grp_hdl, &pdu, param->evt_cnt);
        }
        // unknown PDU
        else
        {
            // Ignore it for the moment
        }
    }

    // Free RX buffer
    ble_util_buf_rx_free(param->em_buf);
#endif //(BLE_BIS_IN_ROM)
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * Inform that BIG sync has been established
 ****************************************************************************************
 */
int lld_big_sync_estab_ind_handler(ke_msg_id_t const msgid, struct lld_big_sync_estab_ind const *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, LLD_BIG_SYNC_ESTAB_IND_HANDLER_FUN_BIT, int, msgid, param, dest_id, src_id);
#if (BLE_BIS_IN_ROM)
    // Pointer to the BIG information structure
    struct lli_big_env *p_big_env;

    if (LLI_BIG_ENV_GET(param->grp_hdl, &p_big_env) == CO_ERROR_NO_ERROR)
    {
        ASSERT_ERR(!GETB(p_big_env->info, LLI_BIG_BROADCASTER));

        lli_big_create_sync_evt_send(p_big_env, param->status, p_big_env->p_start->drv_params.trans_latency);

        // perform clean-up in case of error
        if (param->status != CO_ERROR_NO_ERROR)
        {
            // Cleanup BIG
            lli_big_cleanup(p_big_env->group.hdl);
        }
        else
        {
            // Register BIS in the planner
            struct sch_plan_elt_tag *plan_elt = llm_plan_elt_get(p_big_env->act_id);
            plan_elt->offset       = param->act_offset;
            plan_elt->interval     = 4 * p_big_env->iso_interval;
            plan_elt->duration_min = CO_DIVIDE_ROUND((2 * p_big_env->p_start->drv_params.ctrl_sub_evt_offset), HALF_SLOT_SIZE);
            plan_elt->duration_max = plan_elt->duration_min;
            plan_elt->margin       = 1;
            plan_elt->conhdl       = p_big_env->act_id;
            plan_elt->conhdl_ref   = p_big_env->act_id;
            plan_elt->cb_move      = NULL;
            plan_elt->mobility     = SCH_PLAN_MB_LVL_0;
            sch_plan_set(plan_elt);

            // Clean-up the environment structures
            ke_free(p_big_env->p_start);
            p_big_env->p_start = NULL;
        }
    }
#endif // (BLE_BIS_IN_ROM)
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * Inform that BIG sync has been established
 ****************************************************************************************
 */
int lld_big_sync_offset_upd_ind_handler(ke_msg_id_t const msgid, struct lld_big_sync_offset_upd_ind const *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, LLD_BIG_SYNC_OFFSET_UPD_IND_HANDLER_FUN_BIT, int, msgid, param, dest_id, src_id);
#if (BLE_BIS_IN_ROM)
    // Pointer to the BIG information structure
    struct lli_big_env *p_big_env;

    if (LLI_BIG_ENV_GET(param->grp_hdl, &p_big_env) == CO_ERROR_NO_ERROR)
    {
        // Register BIS in the planner
        struct sch_plan_elt_tag *plan_elt = llm_plan_elt_get(p_big_env->act_id);

        ASSERT_ERR(!GETB(p_big_env->info, LLI_BIG_BROADCASTER));

        if (plan_elt->offset != param->act_offset)
        {
            // Update planner element
            sch_plan_shift(p_big_env->act_id, param->act_offset - plan_elt->offset);
        }
    }
#endif // (BLE_BIS_IN_ROM)
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * ACAD data received by periodic sync activity
 ****************************************************************************************
 */
int llm_acad_data_ind_handler(ke_msg_id_t const msgid, struct llm_acad_data_ind const *param,
                              ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, LLM_ACAD_DATA_IND_HANDLER_FUN_BIT, int, msgid, param, dest_id, src_id);
#if (BLE_BIS_IN_ROM)
    uint8_t status = CO_ERROR_NO_ERROR;
    struct lli_big_env *p_big_env = NULL;
    struct lli_bis_env   *p_bis_env   = NULL;

    do
    {
        // BIG parameters
        struct lld_big_params  *p_big_params;
        struct big_info        *p_big_info;

        // parameters needed to check stream validity
        uint16_t sub_evt_dur, ctrl_sub_evt_dur;
        uint32_t total_evt_dur = 0;
        uint32_t ctrl_sub_evt_offset;
        uint8_t  rx_rate;
        uint8_t  packing = ISO_PACKING_SEQUENTIAL;

        uint8_t i;

        uint8_t supp_phy_loc_msk = PHY_1MBPS_BIT;
        SETB(supp_phy_loc_msk, PHY_2MBPS, BLE_PHY_2MBPS_SUPPORT);
        SETB(supp_phy_loc_msk, PHY_CODED, BLE_PHY_CODED_SUPPORT);

        // *************************************************************************************************************
        // Load the stream targeted
        // *************************************************************************************************************

        // Search the receiver BIS linked with sync handle
        for (i = 0; i < BLE_ISO_GROUP_MAX; i++)
        {
            // check if stream exist
            if ((LLI_BIG_ENV_GET(i, &p_big_env) == CO_ERROR_NO_ERROR)
                    && (!GETB(p_big_env->info, LLI_BIG_BROADCASTER))
                    && (p_big_env->per_act_id == BLE_SYNCHDL_TO_ACTID(param->sync_handle)))
            {
                break;
            }
        }

        // BIG not found
        if (i == BLE_ISO_GROUP_MAX)
        {
            p_big_env = NULL;
            status = CO_ERROR_UNKNOWN_ADVERTISING_ID;
            break;
        }

        // detach activity to not receive anymore ACAD Data reports
        llm_scan_sync_acad_detach(p_big_env->per_act_id);

        // check if operation succeed or periodic advertising is over
        if (param->status != CO_ERROR_NO_ERROR)
        {
            status  = param->status;
            break;
        }

        // *************************************************************************************************************
        // Unpack stream information
        // *************************************************************************************************************

        // allocate start info structure from non-retention memory - will be free before having reason to go in deep sleep
        p_big_env->p_start = (struct lli_bi_start_params *) ke_malloc_system(sizeof(struct lli_bi_start_params), KE_MEM_NON_RETENTION);

        p_big_info    = &(p_big_env->p_start->info);
        p_big_params  = &(p_big_env->p_start->drv_params);

        // Save BIG info
        memcpy(p_big_info, &param->data, sizeof(struct big_info));

        // Parameter sanity check
        if ((p_big_info->bn == 0) || (CO_MOD(p_big_info->nse, p_big_info->bn) != 0))
        {
            status = CO_ERROR_INVALID_LMP_PARAM;
            break;
        }

        // Check also if device has enough number of sub-events to receive at least all Burst once
        if ((p_big_env->max_se != 0) && ((co_min(p_big_env->max_se, p_big_info->nse) / p_big_info->bn) == 0))
        {
            status = CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
            break;
        }

        // *************************************************************************************************************
        //  Check that there is enough channel in broadcaster BIG / can reach BIS PHY ...
        // *************************************************************************************************************
        for (i = 0 ; i < p_big_env->nb_bis ; i++)
        {
            p_bis_env = lli_bis_env[p_big_env->bis_act_id[i]];
            ASSERT_INFO(p_bis_env != NULL, p_big_env->group.hdl, p_big_env->bis_act_id[i]);

            if (p_bis_env->bis_id > p_big_info->num_bis)
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }
        }

        // check that there is no error
        if (status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        // *************************************************************************************************************
        // Check that stream parameters are valid / compute the sequential or interleave scheduling
        // *************************************************************************************************************

        // ensure that encryption state is conform to expectation
        if (GETB(p_big_env->info, LLI_BIG_ENCRYPTED) != p_big_info->encrypted)
        {
            status = CO_ERROR_ENC_MODE_NOT_ACCEPT;
            break;
        }

        // Check if rate proposed are supported
        if ((p_big_info->phy & supp_phy_loc_msk) == 0)
        {
            status = CO_ERROR_UNSUPPORTED;
            break;
        }

        // Interpret the packing method used by the transmitter
        if ((p_big_info->bis_spacing > p_big_info->sub_interval) || (p_big_info->nse == 1))
        {
            packing = ISO_PACKING_SEQUENTIAL;
        }
        else
        {
            packing = ISO_PACKING_INTERLEAVED;
        }

        // Check if parameters are valid according to PHY, etc..
        rx_rate = co_phy_to_rate[co_phy_mask_to_value[p_big_info->phy]];

        sub_evt_dur      = ble_util_pkt_dur_in_us(p_big_info->max_pdu + (p_big_info->encrypted ? MIC_LEN : 0), rx_rate);
        ctrl_sub_evt_dur = ble_util_pkt_dur_in_us(BLE_BIG_MAX_CTRL_PDU_LEN + (p_big_info->encrypted ? MIC_LEN : 0), rx_rate);
        ctrl_sub_evt_offset = p_big_info->bis_spacing * p_big_info->num_bis;

        // Check parameters if sequential stream is used
        if (packing == ISO_PACKING_SEQUENTIAL)
        {
            // ensure that the sub event interval and BIS spacing are valid
            if (((uint32_t)(sub_evt_dur + BLE_MSS_DUR) > p_big_info->sub_interval)
                    || ((p_big_info->sub_interval * (p_big_info->nse - 1) + sub_evt_dur + BLE_MSS_DUR) >  p_big_info->bis_spacing))
            {
                status = CO_ERROR_INVALID_LMP_PARAM;
                break;
            }
        }
        else
        {
            // ensure that the sub event interval and BIS spacing are valid
            if (((uint32_t)(sub_evt_dur + BLE_MSS_DUR) > p_big_info->bis_spacing)
                    || ((p_big_info->nse > 1) && (p_big_info->bis_spacing * (p_big_info->num_bis - 1) + sub_evt_dur + BLE_MSS_DUR) >  p_big_info->sub_interval))
            {
                status = CO_ERROR_INVALID_LMP_PARAM;
                break;
            }

            if (p_big_info->nse > 1)
            {
                ctrl_sub_evt_offset += p_big_info->sub_interval * (p_big_info->nse - 1);
            }
        }

        total_evt_dur = ctrl_sub_evt_offset + ctrl_sub_evt_dur;
        p_big_params->iso_interval_us = ((uint32_t)p_big_info->iso_interval * SLOT_SIZE * 2);

        // ensure that the stream interval is valid
        if ((total_evt_dur + BLE_MSS_DUR) > p_big_params->iso_interval_us)
        {
            status = CO_ERROR_INVALID_LMP_PARAM;
            break;
        }

        // *************************************************************************************************************
        // Store the BIS parameters
        // *************************************************************************************************************
        p_big_env->max_sdu_size                  = p_big_info->max_sdu;
        p_big_env->max_pdu                       = p_big_info->max_pdu;
        p_big_env->bn                            = p_big_info->bn;
        p_big_env->iso_interval                  = p_big_info->iso_interval;
        p_big_env->sdu_interval                  = p_big_info->sdu_interval;

        // fill driver parameters
        p_big_params->sub_evt_dur                = sub_evt_dur;
        p_big_params->ctrl_sub_evt_offset        = ctrl_sub_evt_offset;
        p_big_params->ctrl_sub_evt_dur           = ctrl_sub_evt_dur;

        p_big_params->grp_hdl                    = p_big_env->group.hdl;
        p_big_params->role                       = ROLE_BROADCAST_RECEIVER;
        p_big_params->max_pdu                    = p_big_info->max_pdu;
        p_big_params->rate                       = rx_rate;
        p_big_params->packing                    = packing;

        p_big_params->sync_timeout               = p_big_env->sync_timeout;
        p_big_params->ref_event_cnt              = param->ref_evt_cnt;
        p_big_params->per_sync_id                = BLE_SYNCHDL_TO_ACTID(param->sync_handle);

        // one more channel required for Update sub-event
        p_big_params->bis_params[0].act_id = p_big_env->act_id;
        p_big_params->bis_params[0].idx    = 0;

        // Configure all other streams
        for (i = 0 ; i < p_big_env->nb_bis ; i++)
        {
            p_bis_env = lli_bis_env[p_big_env->bis_act_id[i]];
            ASSERT_INFO(p_bis_env != NULL, p_big_env->group.hdl, p_big_env->bis_act_id[i]);

            p_big_params->bis_params[i + 1].act_id = p_big_env->bis_act_id[i];
            p_big_params->bis_params[i + 1].idx    = p_bis_env->bis_id;
            p_big_env->framing = p_big_info->framing;

            // Store framing
            lli_env.framing[i] = p_big_info->framing;
        }

        p_big_info->num_bis = p_big_env->nb_bis;

        // *************************************************************************************************************
        // BIG is encrypted, compute the GSK
        // *************************************************************************************************************
        if (p_big_info->encrypted)
        {
            uint32_t metainfo = 0;

            SETF(metainfo, LLI_BI_ENC_GRP_HDL, p_big_env->group.hdl);

            // Request generation of IGLTK Using Encryption code
            SETF(metainfo, LLI_BI_ENC_STATE,   LLI_BI_ENC_STATE_GEN_IGLTK);
            aes_h7(lli_bis_h7_salt, p_big_env->broadcast_code, lli_bis_aes_result_cb, metainfo);
        }
        // *************************************************************************************************************
        // BIG not encrypted, BIG receiver can be immediately started
        // *************************************************************************************************************
        else
        {
            // start the procedure
            status = lli_big_start(p_big_env);
        }
    }
    while (0);

    if ((p_big_env != NULL) && (status != CO_ERROR_NO_ERROR))
    {
        // *************************************************************************************************************
        // Send the creation complete event with error
        // *************************************************************************************************************
        lli_big_create_sync_evt_send(p_big_env, status, 0);

        // Cleanup BIG
        lli_big_cleanup(p_big_env->group.hdl);
    }
#endif //(BLE_BIS_IN_ROM)

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_OBSERVER)



uint8_t lli_bis_stats_get(uint8_t act_id, uint32_t *crc_error_packets, uint32_t *rx_unreceived_packets)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLI_BI_PATCH_TYPE, LLI_BIS_STATS_GET_FUN_BIT, uint8_t, act_id, crc_error_packets, rx_unreceived_packets);
#if (BLE_BIS_IN_ROM)
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    do
    {
        struct lli_bis_env *p_bis_env = lli_bis_env[act_id];
        struct lli_big_env *p_big_env = NULL;

        // load group environment
        status = LLI_BIG_ENV_GET(p_bis_env->grp_hdl, &p_big_env);

        if (status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        // check direction of data-path loaded
        if GETB(p_big_env->info, LLI_BIG_BROADCASTER)
        {
            break;
        }

        status = lld_bis_stats_get(act_id, crc_error_packets, rx_unreceived_packets);
    }
    while (0);
#else // (BLE_BIS_IN_ROM)
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

#endif // (BLE_BIS_IN_ROM)
    return (status);
}
#endif // (BLE_BIS)
/// @} LLI

