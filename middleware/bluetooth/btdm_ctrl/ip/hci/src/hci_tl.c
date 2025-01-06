/**
 ****************************************************************************************
 *
 * @file hci_tl.c
 *
 * @brief HCI module source file.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup HCI
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"       // SW configuration

#if (HCI_PRESENT)

#if (HCI_TL_SUPPORT)

#include <string.h>          // string manipulation
#include "co_error.h"        // error definition
#include "co_utils.h"        // common utility definition
#include "co_endian.h"       // common endianess definition
#include "co_list.h"         // list definition

#include "hci.h"             // hci definition
#include "hci_int.h"         // hci internal definition

#include "ke_msg.h"          // kernel message declaration
#include "ke_task.h"         // kernel task definition
#include "ke_mem.h"          // kernel memory definition
#include "ke_timer.h"        // kernel timer definition
#include "rwip.h"            // rw bt core interrupt
#include "dbg.h"             // debug
#include "rom_config.h"
#if (H4TL_SUPPORT)
    #include "h4tl.h"            // H4TL definitions
#endif // H4TL_SUPPORT

#if BLE_EMB_PRESENT
    #include "ble_util_buf.h"
    #include "em_map.h"
    #include "reg_access.h"
    #if (BLE_ISOOHCI)
        #include "isoohci.h"
    #endif //(BLE_ISOOHCI)
#endif //BLE_EMB_PRESENT

#if BT_EMB_PRESENT
    #include "bt_util_buf.h"
    #include "em_map.h"
    #include "reg_access.h"
#endif //BT_EMB_PRESENT

#if (HOST_PRESENT)
    #if (!EMB_PRESENT)
        #include "gap.h"
    #endif // (!EMB_PRESENT)
    #include "hl_hci.h"
#endif // (HOST_PRESENT)

#include "ke_lp.h"
/*
 * DEFINES
 ****************************************************************************************
 */

/// Offset of the Kernel message parameters compared to the event packet position
#define HCI_CMD_PARAM_OFFSET               3
#define HCI_EVT_CC_PARAM_OFFSET            5
#define HCI_EVT_CS_PARAM_OFFSET            5
#define HCI_EVT_PARAM_OFFSET               2
#define HCI_EVT_LE_PARAM_OFFSET            2
#define HCI_EVT_DBG_PARAM_OFFSET           2

#if (HOST_PRESENT && !EMB_PRESENT)
    /// Buffer size for HOST unpacking
    #define HCI_TL_BUFFER_SIZE                 256
#endif // (HOST_PRESENT && !EMB_PRESENT)

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_EMB_PRESENT)
#if (BLE_OBSERVER)
/// Status of an advertising report chain
enum adv_rep_chain_stat
{
    /// No chain
    HCI_ADV_REP_CHAIN_NO         = 0,
    /// A valid advertising report chain is ongoing
    HCI_ADV_REP_CHAIN_VALID      = 1,
    /// A filtered advertising report chain is ongoing
    HCI_ADV_REP_CHAIN_FILTERED   = 2,
};
#endif // (BLE_OBSERVER)
#endif // (BLE_EMB_PRESENT)

///HCI TX states
enum HCI_TX_STATE
{
    ///HCI TX Start State - when packet is ready to be sent
    HCI_STATE_TX_ONGOING,
    ///HCI TX Done State - TX ended with no error
    HCI_STATE_TX_IDLE
};


/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */

#if (BT_HOST_PRESENT && !EMB_PRESENT)
/// Structure used to keep information about BT Host HCI packet to controller
typedef struct hci_tl_bt_tx_info
{
    /// To put information in transmission queue
    co_list_hdr_t hdr;
    /// Function to call when HCI message is transmitted to controller
    hci_done_cb   cb_done;
    /// Pointer to HCI message buffer pointer
    uint8_t      *p_buf;
    /// Size of HCI message to transmit
    uint16_t      length;
    /// HCI Message Type (@see enum enum hci_msg_type)
    uint8_t       type;
} hci_tl_bt_tx_info_t;
#endif // (BT_HOST_PRESENT && !EMB_PRESENT)

#if (BLE_HOST_PRESENT && !EMB_PRESENT)
typedef struct
{
    co_list_hdr_t hdr;
    uint16_t opcode;
    uint8_t conn_idx;
} hci_tl_ble_cmd_st_info_t;
#endif

///HCI Environment context structure
struct hci_tl_env_tag
{
    /// Queue of kernel messages corresponding to HCI TX packets
    co_list_t tx_queue;

#if (BT_EMB_PRESENT || (HCI_BLE_CON_SUPPORT && BLE_EMB_PRESENT))
    /// acl data messages
    co_list_t acl_queue;
#endif // (BT_EMB_PRESENT || (HCI_BLE_CON_SUPPORT && BLE_EMB_PRESENT))

#if (!EMB_PRESENT && BT_HOST_PRESENT)
    /// Initialize the BT reception queue
    co_list_t bt_rx_queue;
#endif // (!EMB_PRESENT && BT_HOST_PRESENT)

#if (BLE_EMB_PRESENT && BLE_ISO_PRESENT)
    /// iso data messages
    co_list_t iso_queue;
#endif // (BLE_EMB_PRESENT && BLE_ISO_PRESENT)

#if (BT_EMB_PRESENT && VOICE_OVER_HCI)
    /// sco data messages
    co_list_t sync_queue;
#endif // (BT_EMB_PRESENT && VOICE_OVER_HCI)

#if (BLE_HOST_PRESENT && !EMB_PRESENT)
    /// command list for command status handle
    co_list_t cmd_st_queue;
#endif


    /// keep the link to message in transmission to a Host
    struct ke_msg *curr_tx_msg;

#if (BT_HOST_PRESENT && !EMB_PRESENT)
    /// Keep information about BT Host HCI packet to controller
    hci_tl_bt_tx_info_t bt_tx_info;
#endif // (BT_HOST_PRESENT && !EMB_PRESENT)

    ///Tx state - either transmitting, done or error.
    uint8_t  tx_state;

    /// Allowed number of commands from Host to controller (sent from controller to host)
    int8_t  nb_h2c_cmd_pkts;

    /// This flag indicates that the current ACL payload is received in a trash buffer
    bool acl_trash_payload;

#if (BLE_EMB_PRESENT && BLE_OBSERVER)
    /// Counter of IQ report
    uint8_t iq_rep_cnt;

    /// Counter of advertising report fragments
    uint8_t adv_rep_frag_cnt;

    /// Status of an advertising report chain from 1M scan
    uint8_t adv_rep_chain_stat_1M;

    /// Status of an advertising report chain from coded scan
    uint8_t adv_rep_chain_stat_coded;

    /// Status of an advertising report chain from periodic scans
    uint8_t per_adv_rep_chain_stat[BLE_ACTIVITY_MAX];
#endif // (BLE_EMB_PRESENT && BLE_OBSERVER)
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///HCI environment structure external global variable declaration
__STATIC struct hci_tl_env_tag hci_tl_env;

#if (HOST_PRESENT && !EMB_PRESENT)
    // Buffer used for message exchange with host
    ALIGN(4)
    __STATIC uint8_t hci_tl_buffer[HCI_TL_BUFFER_SIZE];
#endif // (HOST_PRESENT && !EMB_PRESENT)
/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

__STATIC void hci_tl_tx_done(void);

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_EMB_PRESENT)
#if (BLE_OBSERVER)
/**
 ****************************************************************************************
 * @brief Filter advertising reports if exceeding the maximum capacity
 *
 * This function controls the flow of HCI advertising reports to the Host. It monitors
 * the number of accumulated reports. If the maximum number of reports has been exceeded,
 * new reports are trashed.
 *
 * @param[in]   msg    Kernel message containing the event
 * @return      True: message is filtered | False: message can be transmitted
 *****************************************************************************************
 */
__STATIC bool hci_tl_ble_adv_report_filter_check(struct ke_msg *msg)
{
    bool filtered = false;

    do
    {
        uint8_t chain_stat = HCI_ADV_REP_CHAIN_NO;
        uint8_t subcode = 0xFF;
        bool adv_rep = false;

        // Check if the message is an LE event
        if (msg->id != HCI_LE_EVENT)
            break;

        // Retrieve event subcode
        subcode = *((uint8_t *) msg->param);

        // Check event subcode HCI_LE_ADV_Report or HCI_LE_Ext_ADV_Report
        switch (subcode)
        {
        case HCI_LE_EXT_ADV_REPORT_EVT_SUBCODE:
        {
            // Point to event parameters
            struct hci_le_ext_adv_report_evt *evt = (struct hci_le_ext_adv_report_evt *) &msg->param;
            bool more_data = (GETF(evt->adv_rep[0].evt_type, ADV_EVT_DATA_STATUS) == ADV_EVT_DATA_STATUS_INCOMPLETE);

            chain_stat = (evt->adv_rep[0].phy == PHY_1MBPS_VALUE) ? hci_tl_env.adv_rep_chain_stat_1M : hci_tl_env.adv_rep_chain_stat_coded;

            // Check if report can be queued for transmission, or must be flushed
            if ((chain_stat == HCI_ADV_REP_CHAIN_FILTERED)
                    || ((chain_stat == HCI_ADV_REP_CHAIN_NO) && (hci_tl_env.adv_rep_frag_cnt >= BLE_MAX_NB_ADV_REP_FRAG)))
            {
                filtered = true;
            }

            // Evaluate new chain status
            if (!more_data)
            {
                chain_stat = HCI_ADV_REP_CHAIN_NO;
            }
            else
            {
                chain_stat = filtered ? HCI_ADV_REP_CHAIN_FILTERED : HCI_ADV_REP_CHAIN_VALID;
            }

            // Update chain info for the scan PHY
            if (evt->adv_rep[0].phy == PHY_1MBPS_VALUE)
            {
                hci_tl_env.adv_rep_chain_stat_1M = chain_stat;
                DBG_SWDIAG(LEADVFILT, CHAIN_STAT_1M, chain_stat);
            }
            else
            {
                hci_tl_env.adv_rep_chain_stat_coded = chain_stat;
            }

            adv_rep = true;
        }
        break;

        case HCI_LE_ADV_REPORT_EVT_SUBCODE:
        {
            // Filter legacy report if the max number is reached
            filtered = (hci_tl_env.adv_rep_frag_cnt >= BLE_MAX_NB_ADV_REP_FRAG);

            adv_rep = true;
        }
        break;

        case HCI_LE_PER_ADV_REPORT_EVT_SUBCODE:
        {
            // Point to event parameters
            struct hci_le_per_adv_report_evt *evt = (struct hci_le_per_adv_report_evt *) &msg->param;
            bool more_data = (evt->status == PER_ADV_EVT_DATA_STATUS_INCOMPLETE);

            chain_stat = hci_tl_env.per_adv_rep_chain_stat[BLE_SYNCHDL_TO_ACTID(evt->sync_handle)];

            // Check if report can be queued for transmission, or must be flushed
            if ((chain_stat == HCI_ADV_REP_CHAIN_FILTERED)
                    || ((chain_stat == HCI_ADV_REP_CHAIN_NO) && (hci_tl_env.adv_rep_frag_cnt >= BLE_MAX_NB_ADV_REP_FRAG)))
            {
                filtered = true;
            }

            // Evaluate new chain status
            if (!more_data)
            {
                chain_stat = HCI_ADV_REP_CHAIN_NO;
            }
            else
            {
                chain_stat = filtered ? HCI_ADV_REP_CHAIN_FILTERED : HCI_ADV_REP_CHAIN_VALID;
            }

            // Update chain info for the sync handle
            hci_tl_env.per_adv_rep_chain_stat[BLE_SYNCHDL_TO_ACTID(evt->sync_handle)] = chain_stat;

            adv_rep = true;
        }
        break;

        case HCI_LE_CONLESS_IQ_REPORT_EVT_SUBCODE:
        {
            // Check if report can be queued for transmission, or must be flushed
            filtered = (hci_tl_env.iq_rep_cnt >= BLE_MAX_NB_IQ_REP);

            // If transmitted, increment report counter
            if (!filtered)
            {
                DBG_SWDIAG(LEADVFILT, OK, 1);

                // Increment report counter
                hci_tl_env.iq_rep_cnt++;

                DBG_SWDIAG(LEADVFILT, OK, 0);
            }
        }
        break;

        default:
        {
        }
        break;
        }

        if (filtered)
        {
            DBG_SWDIAG(LEADVFILT, KO, 1);

            DBG_SWDIAG(LEADVFILT, KO, 0);
        }
        else if (adv_rep)
        {
            DBG_SWDIAG(LEADVFILT, OK, 1);

            // Add a critical section as function can be called under interrupt context
            GLOBAL_INT_DISABLE();
            // Increment report fragment counter
            hci_tl_env.adv_rep_frag_cnt++;
            GLOBAL_INT_RESTORE();

            DBG_SWDIAG(LEADVFILT, CNT, hci_tl_env.adv_rep_frag_cnt);

            DBG_SWDIAG(LEADVFILT, OK, 0);
        }

    }
    while (0);

    return filtered;
}

/**
 ****************************************************************************************
 * @brief Update advertising reports flow control when message is transmitted
 *
 * @param[in]   msg    Kernel message containing the event
 *****************************************************************************************
 */
__STATIC void hci_tl_ble_adv_report_tx_check(struct ke_msg *msg)
{
    do
    {
        uint8_t subcode;

        // Check if the message is an LE event
        if (msg->id != HCI_LE_EVENT)
            break;

        // Retrieve event subcode
        subcode = *((uint8_t *) msg->param);

        // Check if the event is HCI_LE_ADV_Report or HCI_LE_Ext_ADV_Report or HCI_LE_Per_ADV_Report
        if ((subcode == HCI_LE_ADV_REPORT_EVT_SUBCODE) || (subcode == HCI_LE_EXT_ADV_REPORT_EVT_SUBCODE)
                || (subcode == HCI_LE_PER_ADV_REPORT_EVT_SUBCODE))
        {
            // Check that at least one advertising report is queued for transmission
            if (hci_tl_env.adv_rep_frag_cnt == 0)
            {
                ASSERT_ERR_FORCE(0);
                break;
            }

            // Add a critical section as function can be called under interrupt context
            GLOBAL_INT_DISABLE();
            // Decrement report fragment counter
            hci_tl_env.adv_rep_frag_cnt--;
            GLOBAL_INT_RESTORE();

            DBG_SWDIAG(LEADVFILT, CNT, hci_tl_env.adv_rep_frag_cnt);

            break;
        }

        // Check if the event is HCI_LE_Conless_IQ_Report
        if (subcode == HCI_LE_CONLESS_IQ_REPORT_EVT_SUBCODE)
        {
            // Check that at least one IQ report is queued for transmission
            if (hci_tl_env.iq_rep_cnt == 0)
            {
                ASSERT_ERR_FORCE(0);
                break;
            }

            // Decrement IQ report counter
            hci_tl_env.iq_rep_cnt--;

            break;
        }

    }
    while (0);
}
#endif // (BLE_OBSERVER)
#endif // (BLE_EMB_PRESENT)

#if (EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Build a HCI Command Status event
 *
 * This function build a HCI CS event from the CS event Kernel message.
 *
 * @param[in]   msg    Kernel message containing the CS event
 * @return      Pointer to the beginning of the event
 *****************************************************************************************
 */
__STATIC uint8_t *hci_tl_build_cs_evt(struct ke_msg *msg)
{
    uint8_t *param = ke_msg2param(msg);
    uint8_t *buf = param - HCI_EVT_CS_PARAM_OFFSET;
    uint8_t *pk = buf;
    uint16_t opcode = msg->src_id;

    //pack event code
    *pk++ = HCI_CMD_STATUS_EVT_CODE;

    //pack event parameter length
    *pk++ = HCI_CSEVT_PARLEN;

    //pack the status
    *pk++ = *param;

    //pack the number of h2c packets
    *pk++ = (hci_tl_env.nb_h2c_cmd_pkts > 0) ? (uint8_t) hci_tl_env.nb_h2c_cmd_pkts : 0;

    //pack opcode
    co_write16p(pk, co_htobs(opcode));

    return buf;
}

/**
 ****************************************************************************************
 * @brief Build a HCI Command Complete event
 *
 * This function build a HCI CC event from the CC event Kernel message.
 *
 * @param[in]   msg    Kernel message containing the CC event
 * @return      Pointer to the beginning of the event
 *****************************************************************************************
 */
__STATIC uint8_t *hci_tl_build_cc_evt(struct ke_msg *msg)
{
    uint8_t  status = CO_UTIL_PACK_OK;
    uint8_t *param = ke_msg2param(msg);
    uint8_t *buf = param - HCI_EVT_CC_PARAM_OFFSET;
    uint8_t *pk = buf;
    uint16_t opcode = msg->src_id;
    uint16_t ret_par_len = msg->param_len;

    // Look for the command descriptor
    const hci_cmd_desc_t *p_cmd_desc = hci_env.p_cmd_evt_desc;

    if ((p_cmd_desc != NULL) && (ret_par_len > 0))
    {
        status = hci_msg_cmd_cmp_pkupk(p_cmd_desc, param, param, &ret_par_len, ret_par_len);
        ASSERT_INFO((status == CO_UTIL_PACK_OK), status, opcode);
    }
    else if (opcode != HCI_NO_OPERATION_CMD_OPCODE)
    {
        // Set the status "Unknown HCI command" as unique returned parameter
        *param = CO_ERROR_UNKNOWN_HCI_COMMAND;
    }
    else
    {
        ASSERT_INFO_FORCE(0, opcode, ret_par_len);
    }

    if (status == CO_UTIL_PACK_OK)
    {
        //pack event code
        *pk++ = HCI_CMD_CMP_EVT_CODE;

        //pack event parameter length
        *pk++ = HCI_CCEVT_HDR_PARLEN + ret_par_len;

        //pack the number of h2c packets
        *pk++ = (hci_tl_env.nb_h2c_cmd_pkts > 0) ? (uint8_t) hci_tl_env.nb_h2c_cmd_pkts : 0;

        //pack opcode
        co_write16p(pk, co_htobs(opcode));
    }

    return buf;
}

/**
 ****************************************************************************************
 * @brief Build a HCI event
 *
 * This function build a HCI event from the event Kernel message.
 *
 * @param[in]   msg    Kernel message containing the event
 * @return      Pointer to the beginning of the event
 *****************************************************************************************
 */
__STATIC uint8_t *hci_tl_build_evt(struct ke_msg *msg)
{
    uint8_t *param = ke_msg2param(msg);
    uint8_t *buf = param - HCI_EVT_PARAM_OFFSET;

    // Look for the event descriptor
    const hci_evt_desc_t *p_evt_desc = hci_env.p_evt_desc;
    ASSERT_ERR(p_evt_desc != NULL);

    if (p_evt_desc != NULL)
    {
        uint8_t *pk = buf;
        uint8_t evt_code = (uint8_t) msg->src_id;
        uint16_t par_len = msg->param_len;
        uint8_t status = hci_msg_evt_pkupk(p_evt_desc, param, param, &par_len, par_len);

        ASSERT_INFO((status == CO_UTIL_PACK_OK), status, evt_code);
        ASSERT_INFO(par_len <= msg->param_len, status, evt_code);

        if (status == CO_UTIL_PACK_OK)
        {
            //pack event code
            *pk++ = evt_code;

            //pack event parameter length
            *pk++ = par_len;
        }
    }

    return buf;
}

#if (RW_DEBUG || AUDIO_SYNC_SUPPORT || EAVESDROPPING_SUPPORT)
/**
 ****************************************************************************************
 * @brief Build a HCI DBG event
 *
 * This function build a HCI LE event from the DBG event Kernel message.
 *
 * @param[in]   msg    Kernel message containing the DBG event
 * @return      Pointer to the beginning of the event
 *****************************************************************************************
 */
__STATIC uint8_t *hci_tl_build_dbg_evt(struct ke_msg *msg)
{
    uint8_t *param = ke_msg2param(msg);
    uint8_t *buf = param - HCI_EVT_DBG_PARAM_OFFSET;

    // Retrieve event descriptor
    const hci_evt_desc_t *p_evt_desc = hci_env.p_evt_desc;
    ASSERT_ERR(p_evt_desc != NULL);

    if (p_evt_desc != NULL)
    {
        uint8_t *pk = buf;
        uint16_t par_len = msg->param_len;
        uint8_t status = hci_msg_evt_pkupk(p_evt_desc, param, param, &par_len, par_len);

        ASSERT_INFO((status == CO_UTIL_PACK_OK), status, *param); // *param is subcode
        ASSERT_INFO(par_len <= msg->param_len, par_len, *param);  // *param is subcode
        if (status == CO_UTIL_PACK_OK)
        {
            //pack event code
            *pk++ = HCI_DBG_META_EVT_CODE;

            //pack event parameter length
            *pk++ = par_len;
        }
    }

    return buf;
}
#endif // (RW_DEBUG || AUDIO_SYNC_SUPPORT || 0)


#if BLE_EMB_PRESENT
/**
 ****************************************************************************************
 * @brief Build a HCI LE event
 *
 * This function build a HCI LE event from the LE event Kernel message.
 *
 * @param[in]   msg    Kernel message containing the LE event
 * @return      Pointer to the beginning of the event
 *****************************************************************************************
 */
__STATIC uint8_t *hci_tl_build_le_evt(struct ke_msg *msg)
{
    uint8_t *param = ke_msg2param(msg);
    uint8_t *buf = param - HCI_EVT_LE_PARAM_OFFSET;

    // Retrieve event descriptor
    const hci_evt_desc_t *p_evt_desc = hci_env.p_evt_desc;
    ASSERT_ERR(p_evt_desc != NULL);

    if (p_evt_desc != NULL)
    {
        uint8_t *pk = buf;
        uint16_t par_len = msg->param_len;

        // Pack
        uint8_t status = hci_msg_evt_pkupk(p_evt_desc, param, param, &par_len, par_len);

        ASSERT_INFO((status == CO_UTIL_PACK_OK), status, *param); // *param = subcode
        ASSERT_INFO(par_len <= msg->param_len, par_len, *param);  // *param = subcode
        if (status == CO_UTIL_PACK_OK)
        {
            //pack event code
            *pk++ = HCI_LE_META_EVT_CODE;

            //pack event parameter length
            *pk++ = par_len;
        }
    }

    return buf;
}

#if BLE_ISO_PRESENT
/**
 ****************************************************************************************
 * @brief Build a HCI ISO data packet
 *
 * This function builds a HCI ISO data packet from the Kernel message.
 *
 * @param[in]   msg    Kernel message associated to the HCI ISO data
 *****************************************************************************************
 */
__STATIC uint8_t *hci_tl_build_iso_data(struct ke_msg *msg)
{
    // Point to message parameters structure
    struct hci_iso_data *param   = (struct hci_iso_data *) ke_msg2param(msg);
    uint16_t handle_flags        = param->conhdl_pbf_tsf;
    uint16_t iso_data_load_len   = param->iso_data_load_len & 0x3FFF;
    uint8_t *buf = ((uint8_t *) param->iso_data_load.iso_sdu_ptr) - (HCI_ISO_DATA_LOAD_HDR_LEN_MAX + HCI_ISO_HDR_LEN);
    uint8_t *pk;

    pk = buf;

    // Pack connection handle + data flags
    co_write16p(pk, co_htobs(handle_flags));
    pk += 2;

    // Pack the ISO_Data_Load_Length
    co_write16p(pk, co_htobs(iso_data_load_len));
    pk += 2;

    // Pack the ISO_Data_Load Time_Stamp
    co_write32p(pk, co_htobl(param->iso_data_load.time_stamp));
    pk += 4;

    // Pack the ISO_Data_Load Packet_Sequence_Number
    co_write16p(pk, co_htobs(param->iso_data_load.pkt_seq_nb));
    pk += 2;

    // Pack the ISO_Data_Load ISO_SDU_Length + RFU + Packet_Status_Flag
    co_write16p(pk, co_htobs(param->iso_data_load.iso_sdu_length_psf));
    pk += 2;

    return (buf);
}
#endif //BLE_ISO_PRESENT
#endif //BLE_EMB_PRESENT
#endif //(EMB_PRESENT)

#if BT_EMB_PRESENT
#if VOICE_OVER_HCI
/**
 ****************************************************************************************
 * @brief Build a HCI SYNC data packet
 *
 * This function build a HCI SYNC data packet from the Kernel message.
 *
 * @param[in]   msg    Kernel message associated to the HCI SYNC data
 *****************************************************************************************
 */
__STATIC uint8_t *hci_tl_build_sync_data(struct ke_msg *msg)
{
    struct hci_sync_data *param = (struct hci_sync_data *) ke_msg2param(msg);
    uint16_t pk_start_addr = param->buf_ptr - HCI_SYNC_HDR_LEN;

    // Pack connection handle and packet status flags
    em_wr16p(pk_start_addr + HCI_SYNC_HDR_HDL_FLAGS_POS, param->conhdl_psf);

    // Pack the data length
    em_wr8p(pk_start_addr + HCI_SYNC_HDR_DATA_LEN_POS, param->length);

    return (uint8_t *)(EM_BASE_ADDR + pk_start_addr);
}
#endif //VOICE_OVER_HCI
#endif //BT_EMB_PRESENT

#if (BT_EMB_PRESENT || HCI_BLE_CON_SUPPORT)
/**
 ****************************************************************************************
 * @brief Build a HCI ACL data packet
 *
 * This function build a HCI ACL data packet from the Kernel message.
 *
 * @param[in]   msg    Kernel message associated to the HCI ACL data
 *****************************************************************************************
 */
__STATIC uint8_t *hci_tl_build_acl_data(struct ke_msg *msg)
{
    // Point to message parameters structure
    struct hci_acl_data *param = (struct hci_acl_data *) ke_msg2param(msg);
    uint16_t handle_flags      = param->conhdl_pb_bc_flag;
    uint8_t *buf = ((uint8_t *)param->buf_ptr) - HCI_ACL_HDR_LEN;
    uint8_t *pk;

#if (!BLE_HOST_PRESENT || BLE_EMB_PRESENT)
    buf += EM_BASE_ADDR;
#endif // (!BLE_HOST_PRESENT || BLE_EMB_PRESENT)

    pk = buf;

    // Pack connection handle and data flags
    co_write16p(pk, co_htobs(handle_flags));
    pk += 2;

    // Pack the data length
    co_write16p(pk, co_htobs(param->length));
    pk += 2;

    return (buf);
}
#endif // (BT_EMB_PRESENT || HCI_BLE_CON_SUPPORT)


#if (HOST_PRESENT && !EMB_PRESENT)
#if (BT_HOST_PRESENT)

/**
 ****************************************************************************************
 * @brief Function called when BT host has handled received packet.
 ****************************************************************************************
 */
__STATIC void hci_tl_bt_rx_handled(void)
{
    // clean-up first element of the queue
    co_list_hdr_t *p_data_rx = co_list_pop_front(&(hci_tl_env.bt_rx_queue));
    ke_free(p_data_rx);
}

#endif // (BT_HOST_PRESENT)

/**
 ****************************************************************************************
 * @brief Build a HCI command
 *
 * This function builds a HCI command from the Kernel message.
 *
 * @param[in]   msg    Kernel message containing the HCI command
 * @return      Pointer to the beginning of the event
 *****************************************************************************************
 */
__STATIC uint8_t *hci_tl_build_cmd(struct ke_msg *p_msg)
{
    uint8_t *p_param = ke_msg2param(p_msg);
    uint8_t *p_buf = p_param - HCI_CMD_PARAM_OFFSET;
    uint8_t *p_pk = p_buf;
    uint16_t opcode = p_msg->src_id;
    uint16_t par_len = p_msg->param_len;
    uint8_t status = CO_UTIL_PACK_ERROR;

    // Look for the command descriptor
    const hci_cmd_desc_t *p_cmd_desc = hci_msg_cmd_desc_get(opcode);
    // Command not found
    ASSERT_INFO((p_cmd_desc != NULL), par_len, opcode);

    if (p_cmd_desc != NULL)
    {
        status = CO_UTIL_PACK_OK;

        if (par_len > 0)
        {
            // Pack command
            if (hci_msg_cmd_ll_dest_get(p_cmd_desc) == BLE_CTRL && hci_msg_cmd_ret_para_get(p_cmd_desc) == NULL)
            {
                hci_tl_ble_cmd_st_info_t *cmd_st = ke_malloc(sizeof(hci_tl_ble_cmd_st_info_t), KE_MEM_KE_MSG);
                ASSERT_ERR(cmd_st != NULL);
                cmd_st->opcode = opcode;
                cmd_st->conn_idx = p_msg->dest_id;
                co_list_push_back(&hci_tl_env.cmd_st_queue, &cmd_st->hdr);
            }
            status = hci_msg_cmd_pkupk(p_cmd_desc, p_param, p_param, &par_len, par_len);
            ASSERT_INFO((status == CO_UTIL_PACK_OK), status, opcode);
        }
    }

    //pack command opcode
    co_write16p(p_pk, co_htobs(opcode));
    p_pk += 2;

    //pack command parameter length
    *p_pk++ = par_len;

    return (status == CO_UTIL_PACK_OK) ? p_buf : NULL;
}
#endif // (HOST_PRESENT && !EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Check if a transmission has to be done and start it.
 *****************************************************************************************
 */
__STATIC void hci_tl_tx_start(void)
{
    struct ke_msg *msg;

    do
    {
        // Check default TX queue
        msg = (struct ke_msg *)co_list_pop_front(&hci_tl_env.tx_queue);
        if (msg != NULL)
            break;

#if (BLE_EMB_PRESENT && BLE_ISO_PRESENT)
        // Check ISO data queue
        msg = (struct ke_msg *) co_list_pop_front(&hci_tl_env.iso_queue);
        if (msg != NULL)
            break;
#endif // (BLE_EMB_PRESENT && BLE_ISO_PRESENT)

#if (BT_EMB_PRESENT && VOICE_OVER_HCI)
        // Check SYNC data flow control
        if (hci_fc_check_host_available_nb_sync_packets())
        {
            msg = (struct ke_msg *)co_list_pop_front(&hci_tl_env.sync_queue);
            if (msg != NULL)
                break;
        }
#endif // (BT_EMB_PRESENT && VOICE_OVER_HCI)

#if (BT_EMB_PRESENT || (HCI_BLE_CON_SUPPORT && BLE_EMB_PRESENT))
        // Check ACL data flow control
        if (hci_fc_check_host_available_nb_acl_packets())
        {
            // Check ACL data queue
            msg = (struct ke_msg *) co_list_pop_front(&hci_tl_env.acl_queue);
            if (msg != NULL)
                break;
        }
#endif //(BT_EMB_PRESENT ||  (HCI_BLE_CON_SUPPORT && BLE_EMB_PRESENT))

    }
    while (0);


    if (msg != NULL)
    {
        uint8_t tl_type = HCI_TL_NONE;
        uint8_t *p_buf = NULL;
        uint16_t len = 0;
        uint8_t type = 0;

        // Store message of the packet under transmission
        hci_tl_env.curr_tx_msg = msg;

#if(BT_HOST_PRESENT && !EMB_PRESENT)
        if ((hci_tl_bt_tx_info_t *) msg == &(hci_tl_env.bt_tx_info))
        {
            p_buf = hci_tl_env.bt_tx_info.p_buf;
            len   = hci_tl_env.bt_tx_info.length;
            type  = hci_tl_env.bt_tx_info.type;
        }
        else
#endif // (BT_HOST_PRESENT && !EMB_PRESENT)
        {

            uint8_t *param = ke_msg2param(msg);

            // Check what kind of TX packet
            switch (msg->id)
            {
#if (EMB_PRESENT)
            case HCI_CMD_CMP_EVENT:
            {
                // Extract information (buffer, length, type)
                p_buf = param - HCI_EVT_CC_PARAM_OFFSET;
                len = HCI_EVT_HDR_LEN + *(p_buf + HCI_EVT_CODE_LEN);
                type = HCI_EVT_MSG_TYPE;
            }
            break;
            case HCI_CMD_STAT_EVENT:
            {
                // Extract information (buffer, length, type)
                p_buf = param - HCI_EVT_CS_PARAM_OFFSET;
                len = HCI_EVT_HDR_LEN + *(p_buf + HCI_EVT_CODE_LEN);
                type = HCI_EVT_MSG_TYPE;
            }
            break;
            case HCI_EVENT:
            {
                // Extract information (buffer, length, type)
                p_buf = param - HCI_EVT_PARAM_OFFSET;
                len = HCI_EVT_HDR_LEN + *(p_buf + HCI_EVT_CODE_LEN);
                type = HCI_EVT_MSG_TYPE;
            }
            break;
#if BLE_EMB_PRESENT
            case HCI_LE_EVENT:
            {
                // Extract information (buffer, length, type)
                p_buf = param - HCI_EVT_LE_PARAM_OFFSET;
                len = HCI_EVT_HDR_LEN + *(p_buf + HCI_EVT_CODE_LEN);
                type = HCI_EVT_MSG_TYPE;
            }
            break;
#if BLE_ISO_PRESENT
            case HCI_ISO_DATA:
            {
                struct hci_iso_data *iso_param = (struct hci_iso_data *) param;

                // Extract information (buffer, length, type)
                p_buf = ((uint8_t *) iso_param->iso_data_load.iso_sdu_ptr) - (HCI_ISO_DATA_LOAD_HDR_LEN_MAX + HCI_ISO_HDR_LEN);
                len = HCI_ISO_HDR_LEN + co_read16p(p_buf + HCI_ISO_HDR_HDL_FLAGS_LEN);
                type = HCI_ISO_MSG_TYPE;
            }
            break;
#endif //BLE_ISO_PRESENT
#endif //BLE_EMB_PRESENT
#if (BT_EMB_PRESENT || HCI_BLE_CON_SUPPORT)
            case HCI_ACL_DATA:
            {
                struct hci_acl_data *acl_param = (struct hci_acl_data *) param;

                // Extract information (buffer, length, type)
                p_buf  = ((uint8_t *)acl_param->buf_ptr) - HCI_ACL_HDR_LEN;
                p_buf += EM_BASE_ADDR;
                len  = HCI_ACL_HDR_LEN + co_read16p(p_buf + HCI_ACL_HDR_HDL_FLAGS_LEN);
                type = HCI_ACL_MSG_TYPE;
            }
            break;
#endif // (BT_EMB_PRESENT || HCI_BLE_CON_SUPPORT)
#if BT_EMB_PRESENT
#if VOICE_OVER_HCI
            case HCI_SYNC_DATA:
            {
                struct hci_sync_data *sync_param = (struct hci_sync_data *) param;

                // Extract information (buffer, length, type)
                p_buf  = ((uint8_t *)((uint32_t)sync_param->buf_ptr)) - HCI_SYNC_HDR_LEN;
                p_buf += EM_BASE_ADDR;
                len  = HCI_SYNC_HDR_LEN + *(p_buf + HCI_SYNC_HDR_DATA_LEN_POS);
                type = HCI_SYNC_MSG_TYPE;
            }
            break;
#endif //VOICE_OVER_HCI
#endif //BT_EMB_PRESENT
#endif //(EMB_PRESENT)

#if (HOST_PRESENT && !EMB_PRESENT)
            case HCI_COMMAND:
            {
                // Extract information (buffer, length, type)
                p_buf = param - HCI_CMD_PARAM_OFFSET;
                len = HCI_CMD_HDR_LEN + *(p_buf + HCI_CMD_OPCODE_LEN);
                type = HCI_CMD_MSG_TYPE;
            }
            break;

            case HCI_ACL_DATA:
            {
                struct hci_acl_data *acl_param = (struct hci_acl_data *) param;
                // Extract information (buffer, length, type)
                p_buf = ((uint8_t *)acl_param->buf_ptr) - HCI_ACL_HDR_LEN;
                len = HCI_ACL_HDR_LEN + co_read16p(p_buf + HCI_ACL_HDR_HDL_FLAGS_LEN);
                type = HCI_ACL_MSG_TYPE;
            }
            break;
            case HCI_PACKED_DATA:
            {
                struct hci_packed_data *pkt = (struct hci_packed_data *) param;
                p_buf = param + sizeof(struct hci_packed_data);
                type = pkt->type;
                len = pkt->length;
                break;
            }

#endif // (HOST_PRESENT && !EMB_PRESENT)

#if (EMB_PRESENT)
                // only one debug event supported for the moment, so flag to be
                // removed as soon as more debug event are supported
#if (RW_DEBUG ||  AUDIO_SYNC_SUPPORT || EAVESDROPPING_SUPPORT)
            case HCI_DBG_EVT:
            {
                // Extract information (buffer, length, type)
                p_buf = param - HCI_EVT_DBG_PARAM_OFFSET;
                len = HCI_EVT_HDR_LEN + *(p_buf + HCI_EVT_CODE_LEN);
                type = HCI_EVT_MSG_TYPE;
            }
            break;
#endif // (RW_DEBUG || AUDIO_SYNC_SUPPORT || 0)
#endif // (EMB_PRESENT)

            default:
            {
                ASSERT_INFO_FORCE(0, msg->id, 0);
            }
            break;
            }
        }

        // Set TX state
        hci_tl_env.tx_state = HCI_STATE_TX_ONGOING;

        // Select the transport type
#if (EMB_PRESENT && BT_HOST_PRESENT)
        tl_type = (hci_ext_host) ? HCI_TL_H4 : HCI_TL_VIRTUAL;
#elif (H4TL_SUPPORT)
        tl_type = HCI_TL_H4;
#endif// (EMB_PRESENT && BT_HOST_PRESENT)

        // Forward the message to the Transport Layer for immediate transmission
        switch (tl_type)
        {
#if (H4TL_SUPPORT)
        case HCI_TL_H4:
        {
            h4tl_write(type, p_buf, len, &hci_tl_tx_done);
        }
        break;
#endif //(H4TL_SUPPORT)
#if (BT_HOST_PRESENT && EMB_PRESENT)
        case HCI_TL_VIRTUAL:
        {
            hl_hci_pk_send_to_host(type, p_buf, len, &hci_tl_tx_done);
        }
        break;
#endif //(BT_HOST_PRESENT)
        default:
        {
            ASSERT_ERR_FORCE(0);
        }
        break;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Function called after sending message through UART, to free ke_msg space and
 * push the next message for transmission if any.
 *
 * The message is popped from the tx queue kept in hci_tl_env and freed using ke_msg_free.
 *****************************************************************************************
 */
__STATIC void hci_tl_tx_done(void)
{
    struct ke_msg *msg = hci_tl_env.curr_tx_msg;
    hci_tl_env.curr_tx_msg = NULL;

#if (BLE_EMB_PRESENT && BLE_OBSERVER)
    // Check advertising report flow control mechanism
    hci_tl_ble_adv_report_tx_check(msg);
#endif // (BLE_EMB_PRESENT && BLE_OBSERVER)

    // Go back to IDLE state
    hci_tl_env.tx_state = HCI_STATE_TX_IDLE;


#if(BT_HOST_PRESENT && !EMB_PRESENT)
    // Check if message is from BT Host
    if ((hci_tl_bt_tx_info_t *) msg == &(hci_tl_env.bt_tx_info))
    {
        // force next pointer to NULL for sanity tests
        hci_tl_env.bt_tx_info.hdr.next = NULL;
        // Calle the TX Done callback of BT Host
        hci_tl_env.bt_tx_info.cb_done();
    }
    else
#endif // (BT_HOST_PRESENT && !EMB_PRESENT)
    {
        // Free the message resources
        switch (msg->id)
        {
        case HCI_EVENT:
#if (BLE_EMB_PRESENT)
        case HCI_LE_EVENT:
#endif // (BLE_EMB_PRESENT)
#if (RW_DEBUG || AUDIO_SYNC_SUPPORT || EAVESDROPPING_SUPPORT)
        case HCI_DBG_EVT:
#endif // (RW_DEBUG || AUDIO_SYNC_SUPPORT || 0)
        case HCI_CMD_CMP_EVENT:
        case HCI_CMD_STAT_EVENT:
        case HCI_COMMAND:
        {
            // Nothing to do
        }
        break;
        case HCI_ACL_DATA:
        {
            // Pop message from HCI parameters
            struct hci_acl_data *data_msg = (struct hci_acl_data *) ke_msg2param(msg);
#if (EMB_PRESENT)
            // Get connection handle
            uint16_t conhdl = GETF(data_msg->conhdl_pb_bc_flag, HCI_ACL_HDR_HDL);

#if (BLE_EMB_PRESENT)
            // If connection is BLE
            if (conhdl <= BLE_CONHDL_MAX)
            {
                // Free the RX buffer associated with the message
                ble_util_buf_rx_free((uint16_t) data_msg->buf_ptr);
            }
#endif // (BLE_EMB_PRESENT)

#if (BT_EMB_PRESENT)
            // If connection is BT
#if EAVESDROPPING_SUPPORT
            if (((conhdl >= BT_ACL_CONHDL_MIN) && (conhdl <= BT_ACL_CONHDL_MAX))
                    || ((conhdl >= BT_ACL_CORR_CONHDL_MIN) && (conhdl <= BT_ACL_CORR_CONHDL_MAX)))
#else // EAVESDROPPING_SUPPORT
            if ((conhdl >= BT_ACL_CONHDL_MIN) && (conhdl <= BT_ACL_CONHDL_MAX))
#endif // EAVESDROPPING_SUPPORT
            {
                // Free the RX buffer associated with the message
                bt_util_buf_acl_rx_free((uint16_t) data_msg->buf_ptr);
            }
#endif //(BT_EMB_PRESENT)

            //count packets for flow control
            hci_fc_acl_packet_sent();

#else // !(EMB_PRESENT)
            uint8_t *p_data = (uint8_t *)data_msg->buf_ptr;
            p_data         -= (HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN);
            // free ACL TX buffer
            ke_free(p_data);
#endif // (EMB_PRESENT)
        }
        break;
#if (BT_EMB_PRESENT && VOICE_OVER_HCI)
        case HCI_SYNC_DATA:
        {
            struct hci_sync_data *data_rx;
            data_rx = (struct hci_sync_data *)(msg->param);
            // Free the RX buffer associated with the message
            bt_util_buf_sync_rx_free(BT_SYNC_CONHDL_LID(GETF(data_rx->conhdl_psf, HCI_SYNC_HDR_HDL)), data_rx->buf_ptr);
            //count packets for flow control
            hci_fc_sync_packet_sent();
        }
        break;
#endif // (BT_EMB_PRESENT && VOICE_OVER_HCI)
#if (BLE_EMB_PRESENT && BLE_ISO_PRESENT && BLE_ISOOHCI)
        case HCI_ISO_DATA:
        {
            // Pop message from HCI parameters
            struct hci_iso_data *data_msg = (struct hci_iso_data *) ke_msg2param(msg);

            // Make sure that the connection handle is at least BLE_CISHDL_MIN
            ASSERT_ERR(GETF(data_msg->conhdl_pbf_tsf, HCI_ISO_HDR_HDL) >= BLE_CISHDL_MIN);

            // Free the RX buffer associated with the message
            isoohci_out_buf_release(data_msg->iso_data_load.iso_sdu_ptr);
        }
        break;
#endif //(BLE_EMB_PRESENT && BLE_ISO_PRESENT && BLE_ISOOHCI)
        case HCI_PACKED_DATA:
            break;
        default:
        {
            ASSERT_INFO_FORCE(0, msg->id, 0);
        }
        break;
        }

        // Free the kernel message space
        ke_msg_free(msg);
    }

    // Check if there is a new message pending for transmission
    hci_tl_tx_start();
}

/**
 ****************************************************************************************
 * @brief Trigger the transmission over HCI TL if possible
 *****************************************************************************************
 */
__STATIC void hci_tl_tx_trigger(void)
{
    // Check if there is no transmission ongoing
    if (hci_tl_env.tx_state == HCI_STATE_TX_IDLE)
    {
        // Start the transmission
        hci_tl_tx_start();
    }
}


/*
 * MODULES INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void hci_tl_packed_data(struct ke_msg *msg, uint8_t log_on)
{
    uint16_t opcode = msg->src_id;
    switch (msg->id)
    {
#if (BT_EMB_PRESENT || HCI_BLE_CON_SUPPORT)
    case HCI_ACL_DATA:
    {
        // Build the HCI data packet
        uint8_t *dbg_data = hci_tl_build_acl_data(msg);
        struct hci_acl_data *param1 = (struct hci_acl_data *) ke_msg2param(msg);
#if BT_EMB_PRESENT
        if (log_on)
            TRC_REQ_HCI_ACL_DATA(0, param1->conhdl_pb_bc_flag, param1->length, (uint32_t *)(dbg_data + HCI_ACL_HDR_LEN));
#endif
    }
    break;
#endif //(BT_EMB_PRESENT || HCI_BLE_CON_SUPPORT)

#if BLE_EMB_PRESENT
#if BLE_ISO_PRESENT
    case HCI_ISO_DATA:
    {
        // Build the HCI ISO data packet
        hci_tl_build_iso_data(msg);
    }
    break;
#endif //BLE_ISO_PRESENT
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
#if VOICE_OVER_HCI
    case HCI_SYNC_DATA:
    {
        // Build the HCI data packet
        hci_tl_build_sync_data(msg);
    }
    break;
#endif //VOICE_OVER_HCI
#endif //(BT_EMB_PRESENT)
#if (EMB_PRESENT)
    case HCI_CMD_CMP_EVENT:
    {


        // Increase the number of commands available for reception (except for Reset and Host Number Of Completed Packets commands)
        if ((opcode != HCI_DBG_PLF_RESET_CMD_OPCODE) && (opcode != HCI_RESET_CMD_OPCODE) && (opcode != HCI_HOST_NB_CMP_PKTS_CMD_OPCODE))
        {
            hci_tl_env.nb_h2c_cmd_pkts++;
            ASSERT_ERR(hci_tl_env.nb_h2c_cmd_pkts <= ble_max_nb_of_cp_get());
        }

        // Build the HCI event
        hci_tl_build_cc_evt(msg);
        if (log_on)
        {
            uint8_t *para = (uint8_t *)msg->param - HCI_EVT_CC_PARAM_OFFSET;
            TRC_REQ_HCI_CMD_CMP_EVT(*para, *(para + 1), (para + 2));
        }
    }
    break;
    case HCI_CMD_STAT_EVENT:
    {
        // Increase the number of commands available for reception
        hci_tl_env.nb_h2c_cmd_pkts++;
        ASSERT_ERR(hci_tl_env.nb_h2c_cmd_pkts <= ble_max_nb_of_cp_get());

        // Build the HCI event
        hci_tl_build_cs_evt(msg);
        if (log_on)
        {
            uint8_t *para = (uint8_t *)msg->param - HCI_EVT_CS_PARAM_OFFSET;
            TRC_REQ_HCI_CMD_STAT_EVT(*para, *(para + 1), (para + 2));
        }
    }
    break;
    case HCI_EVENT:
    {
        // Build the HCI event
        hci_tl_build_evt(msg);
        if (log_on)
        {
            uint8_t *para = (uint8_t *)msg->param - HCI_EVT_PARAM_OFFSET;
            TRC_REQ_HCI_EVT(*para, *(para + 1), (para + 2));
        }
    }
    break;
#if (BLE_EMB_PRESENT)
    case HCI_LE_EVENT:
    {
        // Build the HCI event
        hci_tl_build_le_evt(msg);
        if (log_on)
        {
            uint8_t *para = (uint8_t *)msg->param - HCI_EVT_LE_PARAM_OFFSET;
            TRC_REQ_HCI_LE_EVT(*para, *(para + 1), (para + 2));
        }
    }
    break;
#endif // (BLE_EMB_PRESENT)
#if (RW_DEBUG || AUDIO_SYNC_SUPPORT || 0)
    case HCI_DBG_EVT:
    {
        // Build the DBG event
        hci_tl_build_dbg_evt(msg);
    }
    break;
#endif // (RW_DEBUG || AUDIO_SYNC_SUPPORT || 0)
#endif // (EMB_PRESENT)
#if (HOST_PRESENT && !EMB_PRESENT)
    case HCI_COMMAND:
    {
        // Build the HCI command
        hci_tl_build_cmd(msg);
        TRC_REQ_HCI_CMD(*((uint16_t *)((uint8_t *)msg->param - 3)), *((uint8_t *)msg->param - 1), msg->param);
    }
    break;
    case HCI_PACKED_DATA:
        break;
#endif // (HOST_PRESENT && !EMB_PRESENT)
    default:
    {
        ASSERT_ERR_FORCE(0);
    }
    break;
    }

}

void hci_tl_send(struct ke_msg *msg)
{

    uint16_t opcode = msg->src_id;
#if (BLE_EMB_PRESENT && BLE_OBSERVER)
    // Flow control ADV reports
    if ((msg->id == HCI_LE_EVENT) && hci_tl_ble_adv_report_filter_check(msg))
    {
        // Trash the report
        ke_msg_free(msg);
    }
    else
#endif // (BLE_EMB_PRESENT && BLE_OBSERVER)
    {
        /// Pack message
        hci_tl_packed_data(msg, 1);

        /// put the message into corresponding queue
        switch (msg->id)
        {
#if (BT_EMB_PRESENT || (HCI_BLE_CON_SUPPORT && BLE_EMB_PRESENT))
        case HCI_ACL_DATA:
        {
            co_list_push_back(&hci_tl_env.acl_queue, &msg->hdr);
        }
        break;
#endif // (BT_EMB_PRESENT || (HCI_BLE_CON_SUPPORT && BLE_EMB_PRESENT))

#if (BLE_EMB_PRESENT && BLE_ISO_PRESENT)
        case HCI_ISO_DATA:
        {
            co_list_push_back(&hci_tl_env.iso_queue, &msg->hdr);
        }
        break;
#endif // (BLE_EMB_PRESENT && BLE_ISO_PRESENT)

#if (BT_EMB_PRESENT && VOICE_OVER_HCI)
        case HCI_SYNC_DATA:
        {
            co_list_push_back(&hci_tl_env.sync_queue, &msg->hdr);
        }
        break;
#endif // (BT_EMB_PRESENT && VOICE_OVER_HCI)
        default:
        {
            // Push the message into the HCI queue
            co_list_push_back(&hci_tl_env.tx_queue, &msg->hdr);
        }
        break;
        }

        // Trigger the HCI transmission
        hci_tl_tx_trigger();
    }
}


#if (BT_HOST_PRESENT && !EMB_PRESENT)
void hci_tl_pk_send(uint8_t type, uint16_t length, uint8_t *p_buf, hci_done_cb cb_done)
{
    // ensure that no TX for BT host on-going
    ASSERT_ERR(hci_tl_env.bt_tx_info.hdr.next == NULL);

    hci_tl_env.bt_tx_info.cb_done = cb_done;
    hci_tl_env.bt_tx_info.p_buf   = p_buf;
    hci_tl_env.bt_tx_info.length  = length;
    hci_tl_env.bt_tx_info.type    = type;


    // Push the message into the HCI queue
    co_list_push_back(&hci_tl_env.tx_queue, &(hci_tl_env.bt_tx_info.hdr));
    // Trigger the HCI transmission
    hci_tl_tx_trigger();
}
#endif // (BT_HOST_PRESENT && !EMB_PRESENT)


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void hci_tl_init(bool is_reset)
{
    if (!is_reset)
    {
        // Reset the HCI environment
        memset(&hci_tl_env, 0, sizeof(hci_tl_env));

        // Initialize the HCI event transmit queues
        co_list_init(&hci_tl_env.tx_queue);

#if (BT_EMB_PRESENT || (HCI_BLE_CON_SUPPORT && BLE_EMB_PRESENT))
        // Initialize the BT ACL transmit queues
        co_list_init(&hci_tl_env.acl_queue);
#endif // (BT_EMB_PRESENT || (HCI_BLE_CON_SUPPORT && BLE_EMB_PRESENT))

#if (!EMB_PRESENT && BT_HOST_PRESENT)
        // Initialize the BT reception queue
        co_list_init(&hci_tl_env.bt_rx_queue);
#endif // (!EMB_PRESENT && BT_HOST_PRESENT)

#if (BLE_EMB_PRESENT && BLE_ISO_PRESENT)
        // Initialize the ISO transmit queues
        co_list_init(&hci_tl_env.iso_queue);
#endif //(BLE_EMB_PRESENT && BLE_ISO_PRESENT)

#if (BT_EMB_PRESENT && VOICE_OVER_HCI)
        // Initialize the BT SYNC transmit queues
        co_list_init(&hci_tl_env.sync_queue);
#endif // (BT_EMB_PRESENT && VOICE_OVER_HCI)

#if (!EMB_PRESENT && BLE_HOST_PRESENT)
        // Initialize the BT reception queue
        co_list_init(&hci_tl_env.cmd_st_queue);
#endif // (!EMB_PRESENT && BT_HOST_PRESENT)


        // Initialize TX state machine
        hci_tl_env.tx_state = HCI_STATE_TX_IDLE;
    }
    // Initialize the number of HCI commands the stack can handle simultaneously
    hci_tl_env.nb_h2c_cmd_pkts = ble_max_nb_of_cp_get();
}

#if (EMB_PRESENT)
uint8_t hci_tl_cmd_get_max_param_size(uint16_t opcode)
{
    uint8_t max_param_size = HCI_MAX_CMD_PARAM_SIZE;

    // Find a command descriptor associated to the command opcode
    hci_env.p_cmd_desc = hci_msg_cmd_desc_get(opcode);

    // Check if the command is supported
    if (hci_env.p_cmd_desc != NULL)
    {
        max_param_size = hci_msg_cmd_get_max_param_size(hci_env.p_cmd_desc);
    }

    return (max_param_size);
}


static uint8_t hci_tl_cmd_dynamic_check(const hci_cmd_desc_t *p_cmd_desc)
{
    uint8_t ret = 1;
    if (p_cmd_desc)
    {
        uint8_t ll_dest = hci_msg_cmd_ll_dest_get(p_cmd_desc);
        switch (ll_dest)
        {
        case BT_MNG:
        case BT_CTRL_CONHDL:
        case BT_CTRL_BD_ADDR:
        case BT_BCST:
        {
            if (rom_config_get_bt_controller_enabled() == 0)
                ret = 0;
            break;
        }
        case BLE_MNG:
        case BLE_CTRL:
        case BLE_ISO:
        {
            if (rom_config_get_ble_controller_enabled() == 0)
                ret = 0;
            break;
        }
        }
    }
    return ret;
}

void hci_tl_cmd_received(uint8_t tl_type, uint16_t opcode, uint8_t length, uint8_t *payload)
{
    // Find a command descriptor associated to the command opcode
    const hci_cmd_desc_t *p_cmd_desc = hci_env.p_cmd_desc;
    if (ke_MAC_sleep_check() == true)
    {
        // Wait MAC wakeup.
        // TODO added a timer counter.
        ke_MAC_sleep_wakeup(1);
        while (ke_MAC_sleep_check() == true);
    }
    TRC_REQ_HCI_CMD(opcode, length, payload);
#if (EMB_PRESENT && HOST_PRESENT && HCI_TL_SUPPORT)
#if (H4TL_SUPPORT)
    if (tl_type == HCI_TL_H4)
    {
        // HCI used by external Host
        hci_ext_host = true;
    }
#endif // (H4TL_SUPPORT)
#endif // (EMB_PRESENT && HOST_PRESENT && HCI_TL_SUPPORT)

    // Decrease the number of commands available for reception (except for HCI Host Number Of Completed Packets command)
    ASSERT_ERR(hci_tl_env.nb_h2c_cmd_pkts >= 0);
    if (opcode != HCI_HOST_NB_CMP_PKTS_CMD_OPCODE)
    {
        hci_tl_env.nb_h2c_cmd_pkts--;
    }


    // Check if the command is supported
    if (p_cmd_desc != NULL)
    {
        if (hci_tl_cmd_dynamic_check(p_cmd_desc) != 0)
        {
            // Check if a new command can be received
            if (hci_tl_env.nb_h2c_cmd_pkts >= 0)
            {
                uint16_t dest = TASK_NONE;
                uint8_t ll_dest = hci_msg_cmd_ll_dest_get(p_cmd_desc);

                // Find the lower layers destination task
                dest = hci_msg_task_dest_compute(ll_dest, length, payload);

                // Check if the command can be handled by the controller
                if (dest != TASK_NONE)
                {
                    uint16_t unpk_length = 0;
                    uint8_t status = CO_UTIL_PACK_OK;

                    // Check if there are parameters (to compute the unpacked parameters size for Kernel message allocation)
                    if (length > 0)
                    {
                        // Unpack
                        status = hci_msg_cmd_pkupk(p_cmd_desc, NULL, payload, &unpk_length, length);
                    }

                    // Check if there is input buffer overflow (received parameter size is less than expected for this command)
                    if (status == CO_UTIL_PACK_IN_BUF_OVFLW)
                    {
                        // Reject the command with error code "Invalid HCI parameters"
                        hci_msg_cmd_reject_send(p_cmd_desc, opcode, CO_ERROR_INVALID_HCI_PARAM, NULL);
                    }
                    else
                    {
                        // Allocate a Kernel message (with space for unpacked parameters)
                        void *cmd = ke_msg_alloc(HCI_COMMAND, dest, opcode, unpk_length);

                        ASSERT_INFO(status == CO_UTIL_PACK_OK, status, opcode);

                        if (cmd == NULL)
                        {
                            // Reject the command with error code "Memory Capacity Exceeded"
                            hci_msg_cmd_reject_send(p_cmd_desc, opcode, CO_ERROR_MEMORY_CAPA_EXCEED, NULL);
                        }
                        else
                        {
                            if (unpk_length > 0)
                            {
                                // Unpack
                                status = hci_msg_cmd_pkupk(p_cmd_desc, (uint8_t *) cmd, payload, &unpk_length, length);
                            }

                            ASSERT_INFO(status == CO_UTIL_PACK_OK, status, opcode);

                            TRC_REQ_HCI_CMD(opcode, unpk_length, cmd);

                            // Send the command to the internal destination task associated to this command
                            ke_msg_send(cmd);
                        }
                    }
                }
                else
                {
                    // Reject the command with error code "Unknown connection identifier"
                    hci_msg_cmd_reject_send(p_cmd_desc, opcode, CO_ERROR_UNKNOWN_CONNECTION_ID, payload);
                }
            }
            else
            {
                // Reject the command with error code "Memory Capacity Exceeded"
                hci_msg_cmd_reject_send(p_cmd_desc, opcode, CO_ERROR_MEMORY_CAPA_EXCEED, NULL);
            }
        }
        else
        {
            // Reject due to state now allowed
            hci_msg_cmd_reject_send(p_cmd_desc, opcode, CO_ERROR_COMMAND_DISALLOWED, NULL);
        }
    }
    else
    {
        // Reject the command with error code "Unknown HCI Command"
        hci_msg_cmd_reject_send(NULL, opcode, CO_ERROR_UNKNOWN_HCI_COMMAND, NULL);
    }
}

uint8_t *hci_tl_acl_tx_data_alloc(uint8_t tl_type, uint16_t hdl_flags, uint16_t len)
{
    uint8_t *buf = NULL;
    uint16_t dest = TASK_NONE;
    uint16_t max_len = 0;

#if ((BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT)) || BT_EMB_PRESENT)
    // Retrieve connection handle from command parameters (expecting at payload 1st 2 bytes)
    uint16_t conhdl  = GETF(hdl_flags, HCI_ACL_HDR_HDL);
    uint8_t  bc_flag = GETF(hdl_flags, HCI_ACL_HDR_BC_FLAG);
#endif // ((BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT)) || BT_EMB_PRESENT)

#if (BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))
    // Check if the connection handle corresponds to an active BLE link
    if ((conhdl <= BLE_CONHDL_MAX) && (bc_flag == BCF_P2P))
    {
        // Build the destination task ID
        dest = KE_BUILD_ID(TASK_LLC, conhdl);
        max_len = BLE_MAX_OCTETS;
    }
#endif //(BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))

#if BT_EMB_PRESENT
    // Check if broadcast flags are set
    if (bc_flag == BCF_ACTIVE_SLV_BCST)
    {
        dest = TASK_LB;
        max_len = ACL_DATA_BUF_SIZE;
    }
    // Check if the connection handle corresponds to an active BT link
    else if (((bc_flag & BCF_MASK) == BCF_P2P) && (conhdl >= BT_ACL_CONHDL_MIN) && (conhdl <= BT_ACL_CONHDL_MAX))
    {
        if (hci_env.bt_acl_con_tab[(conhdl - BT_ACL_CONHDL_MIN)].state == HCI_BT_ACL_STATUS_BD_ADDR_CONHDL)
        {
            // Build the destination task ID
            dest = KE_BUILD_ID(TASK_LC, conhdl - BT_ACL_CONHDL_MIN);
            max_len = ACL_DATA_BUF_SIZE;
        }
    }
#endif //BT_EMB_PRESENT

    // Check if the requested size fits within BT/BLE data max length
    if ((dest != TASK_NONE) && (len <= max_len))
    {
        switch (KE_TYPE_GET(dest))
        {
#if (BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))
        case TASK_LLC:
        {
            // Try allocating a buffer from BLE pool
            uint16_t buf_ptr = ble_util_buf_acl_tx_alloc();

            if (buf_ptr != 0)
            {
                // Give the pointer to the data space
                buf = (uint8_t *)(EM_BASE_ADDR + buf_ptr);
            }
        }
        break;
#endif //(BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT))

#if BT_EMB_PRESENT
        case TASK_LC:
        case TASK_LB:
        {
            // Try allocating a buffer from BT pool
            uint16_t buf_ptr = bt_util_buf_acl_tx_alloc();

            if (buf_ptr != 0)
            {
                // Give the pointer to the data space
                buf = (uint8_t *)(EM_BASE_ADDR + buf_ptr);
            }
        }
        break;
#endif //BT_EMB_PRESENT

        default:
        {
            ASSERT_ERR_FORCE(0);
        }
        break;
        }

        if (buf == NULL)
        {
            // Report buffer oveflow
            struct hci_data_buf_ovflw_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_DATA_BUF_OVFLW_EVT_CODE, hci_data_buf_ovflw_evt);
            evt->link_type = ACL_TYPE;
            hci_send_2_host(evt);

            // Allocate a temporary buffer from heap
            buf = ke_malloc_system(len, KE_MEM_KE_MSG);

            // Indicate the reception of ACL payload in a trash buffer
            hci_tl_env.acl_trash_payload = true;
        }
    }

    return buf;
}

void hci_tl_acl_tx_data_received(uint8_t tl_type, uint16_t hdl_flags, uint16_t datalen, uint8_t *payload)
{
    do
    {
#if ((BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT)) || BT_EMB_PRESENT)
        uint16_t conhdl = GETF(hdl_flags, HCI_ACL_HDR_HDL);
#if BT_EMB_PRESENT
        uint8_t  bc_flag = GETF(hdl_flags, HCI_ACL_HDR_BC_FLAG);
#endif //BT_EMB_PRESENT
#endif // ((BLE_EMB_PRESENT && (HCI_BLE_CON_SUPPORT)) || BT_EMB_PRESENT)

        TRC_REQ_HCI_ACL_DATA(1, hdl_flags, datalen, payload);
        // If payload is received in trash buffer
        if (hci_tl_env.acl_trash_payload)
        {
            // Free the temporary buffer
            ke_free(payload);

            // Clear the indication of ACL payload reception in a trash buffer
            hci_tl_env.acl_trash_payload = false;

            break;
        }


#if defined(CFG_EMB)
        if (ke_MAC_sleep_check() == true)
        {
            // Wait MAC wakeup.
            // TODO added a timer counter.
            ke_MAC_sleep_wakeup(1);
            while (ke_MAC_sleep_check() == true);
        }
#endif // RT_USING_PM

#if (BLE_EMB_PRESENT && HCI_BLE_CON_SUPPORT)
        // Check if the received packet was considered BLE one at header reception
        if (conhdl <= BLE_CONHDL_MAX)
        {
            // Allocate a Kernel message
            struct hci_acl_data *data_tx = KE_MSG_ALLOC(HCI_ACL_DATA, KE_BUILD_ID(TASK_LLC, conhdl), 0, hci_acl_data);
            data_tx->conhdl_pb_bc_flag   = hdl_flags;
            data_tx->length              = datalen;
            data_tx->buf_ptr             = (datalen > 0) ? (((uint32_t) payload) - EM_BASE_ADDR) : 0;
            ke_msg_send(data_tx);

            break;
        }
#endif //(BLE_EMB_PRESENT && HCI_BLE_CON_SUPPORT)

#if BT_EMB_PRESENT
        // Check if the received packet was considered BT one at header reception
        if (((bc_flag == BCF_ACTIVE_SLV_BCST) || ((conhdl >= BT_ACL_CONHDL_MIN) && (conhdl <= BT_ACL_CONHDL_MAX))))
        {
            // Select the destination task, according to the broadcast flag
            uint16_t dest_id = (bc_flag == BCF_ACTIVE_SLV_BCST) ? TASK_LB : KE_BUILD_ID(TASK_LC, conhdl - BT_ACL_CONHDL_MIN);

            // Allocate a Kernel message
            struct hci_acl_data *data_tx = KE_MSG_ALLOC(HCI_ACL_DATA, dest_id, 0, hci_acl_data);
            data_tx->conhdl_pb_bc_flag   = hdl_flags;
            data_tx->length              = datalen;
            data_tx->buf_ptr             = (datalen > 0) ? (((uint32_t) payload) - EM_BASE_ADDR) : 0;
            ke_msg_send(data_tx);

            break;
        }
#endif //BT_EMB_PRESENT

        ASSERT_INFO_FORCE(0, datalen, hdl_flags);
    }
    while (0);
}
#endif //(EMB_PRESENT)

#if ((BT_EMB_PRESENT || BT_HOST_PRESENT) && VOICE_OVER_HCI)
uint8_t *hci_tl_sync_tx_data_alloc(uint8_t tl_type, uint16_t conhdl_flags, uint8_t len)
{
    uint8_t *p_buf = NULL;
#if (EMB_PRESENT)
    // Retrieve connection handle from command parameters (expecting at payload 1st 2 bytes)
    uint16_t conhdl = GETF(conhdl_flags, HCI_SYNC_HDR_HDL);
    uint8_t sync_link_id = BT_SYNC_CONHDL_LID(conhdl);

    // There are many conditions where the packet is not valid (invalid connection handle, no SCO link, not in Voice over HCI)
    do
    {
        // Check if the connection handle corresponds to a possible synchronous link
        if (sync_link_id >= MAX_NB_SYNC)
            break;

        // Check if the connection handle corresponds to a possible an BT link
        conhdl &= ~(BT_SYNC_CONHDL_MSK);
        if ((conhdl < BT_ACL_CONHDL_MIN) || (conhdl > BT_ACL_CONHDL_MAX))
            break;

        // Check if the ACL link is active
        if (hci_env.bt_acl_con_tab[(conhdl - BT_ACL_CONHDL_MIN)].state != HCI_BT_ACL_STATUS_BD_ADDR_CONHDL)
            break;

        // Try allocating a buffer from BT pool
        p_buf = (uint8_t *)(uint32_t) bt_util_buf_sync_tx_alloc(sync_link_id, len);

    }
    while (0);

    // If a buffer element has been allocated
    if (p_buf != 0)
    {
        // Give the pointer to the data space
        p_buf += EM_BASE_ADDR;
    }
    else
    {
        // Allocate a temporary buffer from heap
        p_buf = ke_malloc_system(len, KE_MEM_KE_MSG);

        // Indicate the reception of ACL payload in a trash buffer
        hci_tl_env.acl_trash_payload = true;
    }
#else // !(EMB_PRESENT)
    p_buf  = (uint8_t *) ke_malloc_system(len + HCI_HOST_BUF_HEAD_LEN, KE_MEM_NON_RETENTION);
    p_buf += HCI_HOST_BUF_HEAD_LEN;
#endif // (EMB_PRESENT)

    return p_buf;
}

void hci_tl_sync_tx_data_received(uint8_t tl_type, uint16_t conhdl_flags, uint8_t len, uint8_t *p_payload)
{
#if (EMB_PRESENT)
    uint16_t conhdl = GETF(conhdl_flags, HCI_SYNC_HDR_HDL);
    conhdl &= ~(BT_SYNC_CONHDL_MSK);

    // Check if the received packet is in a valid buffer
    if (!hci_tl_env.acl_trash_payload)
    {
        // Sends the Kernel message (with space for unpacked parameters)
        struct hci_sync_data *data_tx = KE_MSG_ALLOC(HCI_SYNC_DATA, KE_BUILD_ID(TASK_LC, conhdl - BT_ACL_CONHDL_MIN), 0, hci_sync_data);
        data_tx->buf_ptr = (uint16_t)((uint32_t)p_payload - EM_BASE_ADDR);
        data_tx->conhdl_psf = conhdl_flags;
        data_tx->length = len;
        ke_msg_send(data_tx);
    }
    else
    {
        // Free the temporary buffer
        ke_free(p_payload);

        // Clear the indication of ACL payload reception in a trash buffer
        hci_tl_env.acl_trash_payload = false;
    }
#else // !(EMB_PRESENT)
    struct hci_acl_data *p_data_rx = NULL;
    ASSERT_ERR(p_payload != NULL);
    p_data_rx = (struct hci_acl_data *)(p_payload -  HCI_HOST_BUF_HEAD_LEN);
    // Use allocated structure has a list element -- other header data not used
    // list used only for kepping pointer to data to free later.
    co_list_push_back(&(hci_tl_env.bt_rx_queue), (co_list_hdr_t *) p_data_rx);

    // Append ACL header in payload
    p_payload -= HCI_SYNC_HDR_DATA_LEN_LEN;
    co_write16p(p_payload, co_htobs(len));
    p_payload -= HCI_SYNC_HDR_HDL_FLAGS_LEN;
    co_write16p(p_payload, co_htobs(conhdl_flags));
    len   += HCI_SYNC_HDR_LEN;
    hl_hci_pk_send_to_host(HCI_SYNC_MSG_TYPE, p_payload, len, hci_tl_bt_rx_handled);
    ke_free(p_data_rx);
#endif // (EMB_PRESENT)

}
#endif // ((BT_EMB_PRESENT || BT_HOST_PRESENT) && VOICE_OVER_HCI)

#if (HOST_PRESENT && !EMB_PRESENT)
uint8_t *hci_tl_acl_rx_data_alloc(uint8_t tl_type, uint16_t hdl_flags, uint16_t len)
{
    uint8_t *p_buf = (uint8_t *) ke_malloc_system(len + HCI_HOST_BUF_HEAD_LEN, KE_MEM_NON_RETENTION);

    return (p_buf + HCI_HOST_BUF_HEAD_LEN);
}

void hci_tl_acl_rx_data_received(uint8_t tl_type, uint16_t hdl_flags, uint16_t datalen, uint8_t *p_payload)
{
    uint8_t hl_tl_dest = HOST_NONE;
    struct hci_acl_data *p_data_rx = NULL;
    bool release = false;

    TRC_REQ_HCI_ACL_DATA(0, hdl_flags, datalen, p_payload);

    if (p_payload != NULL)
    {
        p_data_rx = (struct hci_acl_data *)(p_payload -  HCI_HOST_BUF_HEAD_LEN);
    }
    else
    {
        p_data_rx = (struct hci_acl_data *) ke_malloc_system(HCI_HOST_BUF_HEAD_LEN, KE_MEM_NON_RETENTION);
        p_payload = ((uint8_t *) p_data_rx) + HCI_HOST_BUF_HEAD_LEN;
    }

    hl_tl_dest = hl_hci_get_host_tl_dest_from_conhdl(GETF(hdl_flags, HCI_ACL_HDR_HDL));

#if (BT_HOST_PRESENT)
    if (hl_tl_dest == HOST_TL_PK)
    {
        // Append ACL header in payload
        p_payload -= HCI_ACL_HDR_DATA_LEN_LEN;
        co_write16p(p_payload, co_htobs(datalen));
        p_payload -= HCI_ACL_HDR_HDL_FLAGS_LEN;
        co_write16p(p_payload, co_htobs(hdl_flags));
        datalen   += HCI_ACL_HDR_LEN;

        hl_hci_pk_send_to_host(HCI_ACL_MSG_TYPE, p_payload, datalen, hci_tl_bt_rx_handled);
        release = true;
    }
    else
#endif // (BT_HOST_PRESENT)
#if (BLE_HOST_PRESENT && HCI_BLE_CON_SUPPORT)
        if (hl_tl_dest == HOST_TL_UPK)
        {
            p_data_rx->conhdl_pb_bc_flag = hdl_flags;
            p_data_rx->length = datalen;
            p_data_rx->buf_ptr = (uint32_t) p_payload;

            hl_hci_send_data_to_host(HL_HCI_ACL_DATA, p_data_rx, 1);
        }
        else
#endif // (BLE_HOST_PRESENT && HCI_BLE_CON_SUPPORT)
            if (hl_tl_dest == HOST_NONE)
            {
                ASSERT_ERR_FORCE(0);
                ke_free(p_data_rx);
            }

    if (release)
        ke_free(p_data_rx);
}

uint8_t *hci_tl_evt_data_alloc(uint8_t tl_type, uint8_t code, uint8_t length)
{
    uint8_t *p_buf = (uint8_t *) ke_malloc_system(length + HCI_HOST_BUF_HEAD_LEN, KE_MEM_NON_RETENTION);
    return (p_buf + HCI_HOST_BUF_HEAD_LEN);
}

uint8_t hci_tl_evt_received(uint8_t tl_type, uint8_t code, uint8_t length, uint8_t *p_payload)
{
    uint8_t status = CO_UTIL_PACK_OK;
    uint16_t opcode = 0;
    uint8_t evt_type = HL_HCI_UNDEF;
    const hci_cmd_desc_t *p_cmd_desc = NULL;
    const hci_evt_desc_t *p_evt_desc = NULL;
    uint8_t hl_tl_dest = HOST_NONE;
    bool release = true;
    // retrieve pointer to allocated buffer
    uint8_t *p_buf;

    ASSERT_ERR(p_payload != NULL);

    p_buf = (p_payload - HCI_HOST_BUF_HEAD_LEN);

    switch (code)
    {
    case HCI_CMD_CMP_EVT_CODE:
    {
        ASSERT_WARN((length >= HCI_CCEVT_HDR_PARLEN), length, 0);
        if (length < HCI_CCEVT_HDR_PARLEN) break;

        // Retrieve opcode from parameters, expected at position 1 in payload (after nb cmp pkts)
        opcode = co_btohs(co_read16p(p_payload + 1));
        // Look for the command descriptor
        p_cmd_desc = hci_msg_cmd_desc_get(opcode);
        evt_type = HL_HCI_CMD_CMP_EVT;
        ASSERT_WARN((p_cmd_desc != NULL), opcode, 0);
        TRC_REQ_HCI_CMD_CMP_EVT(code, length, p_payload);
    }
    break;
    case HCI_CMD_STATUS_EVT_CODE:
    {
        ASSERT_WARN((length == HCI_CSEVT_PARLEN), length, 0);
        if (length != HCI_CSEVT_PARLEN) break;
        // Retrieve opcode from parameters, expected at position 2 in payload (after status and nb cmp pkts)
        opcode = co_btohs(co_read16p(p_payload + 2));
        // Look for the command descriptor
        p_cmd_desc = hci_msg_cmd_desc_get(opcode);
        evt_type = HL_HCI_CMD_STAT_EVT;
        ASSERT_WARN((p_cmd_desc != NULL), opcode, 0);
        TRC_REQ_HCI_CMD_STAT_EVT(code, length, p_payload);
    }
    break;
    default:
    {
        opcode = code;
        // Find an event descriptor associated to the event code
        if (opcode == HCI_LE_META_EVT_CODE)
        {
            opcode = *p_payload;
            p_evt_desc = hci_msg_le_evt_desc_get(opcode);
            evt_type = HL_HCI_LE_EVT;
            TRC_REQ_HCI_LE_EVT(code, length, p_payload);
        }
        else if (opcode == HCI_DBG_META_EVT_CODE
#if (RW_DEBUG)
                 || opcode == HCI_DBG_ASSERT_EVT_SUBCODE
#endif // RW_DEBUG
                )
        {
            opcode = *p_payload;
#if (RW_DEBUG || AUDIO_SYNC_SUPPORT || EAVESDROPPING_SUPPORT)
            p_evt_desc = hci_msg_dbg_evt_desc_get(opcode);
#endif // (RW_DEBUG || AUDIO_SYNC_SUPPORT || 0)
            evt_type = HL_HCI_DBG_EVT;
        }
        else
        {
            p_evt_desc = hci_msg_evt_desc_get(opcode);
            evt_type = HL_HCI_EVT;
            TRC_REQ_HCI_EVT(code, length, p_payload);
        }

        ASSERT_WARN((p_evt_desc != NULL), opcode, evt_type);
    }
    break;
    }

    if (p_cmd_desc != NULL)
    {
        hl_tl_dest = hl_hci_cmd_evt_get_host_tl_dest(opcode);
        uint8_t *p_evt = hci_tl_buffer;

        if (hl_tl_dest == HOST_TL_UPK)
        {
#if (BT_HOST_OTHER_STACK)
            // speical handle for BT addr since BT/BLE stack use different MAC format
            if (evt_type == HL_HCI_CMD_CMP_EVT && opcode == HCI_RD_BD_ADDR_CMD_OPCODE)
                p_cmd_desc = hci_msg_cmd_desc_get_for_dual_stack(HCI_RD_BD_ADDR_CMD_OPCODE);
#endif // BT_HOST_OTHER_STACK

            // At least, should have status code
            uint16_t evt_len = 1;
            uint8_t dest = hci_msg_cmd_ll_dest_get(p_cmd_desc);
            if (evt_type == HL_HCI_CMD_CMP_EVT)
            {
                length    -= HCI_CCEVT_HDR_PARLEN;
                p_payload += HCI_CCEVT_HDR_PARLEN;
                // Check if there are parameters to unpack
                if (length > 0)
                {
                    uint16_t unpk_length = HCI_TL_BUFFER_SIZE;
                    // Unpack
                    status = hci_msg_cmd_cmp_pkupk(p_cmd_desc, p_evt, p_payload, &unpk_length, length);
                    evt_len = unpk_length;
                }
            }
            else // HL_HCI_CMD_STAT_EVT
            {
                p_evt[0] = *p_payload; // status code only
                if (dest == BLE_CTRL && hci_msg_cmd_ret_para_get(p_cmd_desc) == NULL)
                {
                    if (!co_list_is_empty(&hci_tl_env.cmd_st_queue))
                    {
                        hci_tl_ble_cmd_st_info_t *next_st = (hci_tl_ble_cmd_st_info_t *)co_list_pick(&hci_tl_env.cmd_st_queue);
                        do
                        {
                            if (next_st->opcode == opcode)
                            {
                                p_evt[1] = next_st->conn_idx;
                                evt_len++;
                                co_list_extract(&hci_tl_env.cmd_st_queue, (co_list_hdr_t *)next_st);
                                ke_free(next_st);
                                break;
                            }
                            next_st = (hci_tl_ble_cmd_st_info_t *)co_list_next((co_list_hdr_t *)next_st);
                        }
                        while (next_st != NULL);

                    }
                }
            }
            hl_hci_send_cmd_evt_to_host(dest, evt_type, opcode, evt_len, p_evt);
        }
    }
    else if (p_evt_desc != NULL)
    {
        hl_tl_dest = hci_msg_evt_get_hl_tl_dest(p_evt_desc);
        uint16_t conhdl = GAP_INVALID_CONHDL;
        if (hl_tl_dest == HOST_UNDEF)
        {
#if (BLE_CIS) && (BLE_BIS)
            if ((evt_type == HL_HCI_LE_EVT) && (opcode >= HCI_LE_CIS_ESTABLISHED_EVT_SUBCODE) && (opcode <= HCI_LE_BIG_SYNC_LOST_EVT_SUBCODE))
            {
                hl_tl_dest = HOST_NONE;//TODO ISO   conhdl is different
            }
            else
#endif
            {
                // Retrieve host destination according to connection handle present in event
                conhdl = hci_msg_evt_look_for_conhdl(p_evt_desc, p_payload);
#if (BT_HOST_PRESENT)
                hl_tl_dest = hl_hci_get_host_tl_dest_from_conhdl(conhdl);
#else // (BLE_HOST_PRESENT)
                hl_tl_dest = HOST_TL_UPK;
#endif // (BT_HOST_PRESENT)
            }
        }

        if (hl_tl_dest == HOST_TL_UPK)
        {
            uint8_t *p_evt = hci_tl_buffer;
            uint8_t host_lid = hci_msg_evt_host_lid_get(p_evt_desc);
            uint16_t evt_len = 0;
            // Check if there are parameters to unpack
            if (length > 0)
            {
                uint16_t unpk_length = HCI_TL_BUFFER_SIZE;
                status =  hci_msg_evt_pkupk(p_evt_desc, p_evt, p_payload, &unpk_length, length);
                ASSERT_INFO((status == CO_UTIL_PACK_OK), status, code);
                evt_len = unpk_length;
            }

            hl_hci_send_evt_to_host(evt_type, (uint8_t)opcode, conhdl, evt_len, p_evt);
        }
    }

#if (BT_HOST_PRESENT)
    if (hl_tl_dest == HOST_TL_PK)
    {
#if 0
        release = false;

        // Use allocated structure has a list element -- other header data not used
        // list used only for kepping pointer to data to free later.
        co_list_push_back(&(hci_tl_env.bt_rx_queue), (co_list_hdr_t *) p_buf);
#else
        release = true;
#endif
        // Append ACL header in payload
        p_payload  -= HCI_CMDEVT_PARLEN_LEN;
        *p_payload  = length;
        p_payload  -= HCI_EVT_CODE_LEN;
        *p_payload  = code;
        {
            uint16_t len = length;
            len += HCI_EVT_CODE_LEN + HCI_CMDEVT_PARLEN_LEN;
            hl_hci_pk_send_to_host(HCI_EVT_MSG_TYPE, p_payload, len, NULL);
        }
    }
#endif // (BT_HOST_PRESENT)

    if (release)
    {
        // release buffer allocated
        ke_free(p_buf);
    }

    return (status);
}
#endif //(HOST_PRESENT && !EMB_PRESENT)


#endif // (HCI_TL_SUPPORT)
#endif //(HCI_PRESENT)

/// @} HCI
