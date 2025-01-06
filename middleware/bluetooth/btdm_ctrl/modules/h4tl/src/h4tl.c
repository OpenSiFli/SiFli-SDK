/**
 ****************************************************************************************
 *
 * @file h4tl.c
 *
 * @brief H4 UART Transport Layer source code.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup H4TL
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"      // stack configuration

#if (H4TL_SUPPORT)

#include <string.h>           // in order to use memset
#include "co_endian.h"        // endian-ness definition
#include "co_utils.h"         // stack common utility definitions
#include "co_error.h"         // error definition
#include "co_hci.h"           // HCI definitions
#include "co_djob.h"          // Defer function execution
#include "h4tl.h"             // hci External Interface definition
#include "ke_mem.h"           // kernel memory
#include "ke_msg.h"           // kernel event
#include "ke_event.h"         // kernel event definition
#include "rwip.h"             // rw bt core interrupt

#if (HCI_TL_SUPPORT)
    #include "hci.h"              // hci definition
#endif //(HCI_TL_SUPPORT))

#if (AHI_TL_SUPPORT)
    #include "ahi.h"              // Application Host Interface definition
    #include "gapm_task.h"
    #include "gapm.h"             // GAP Manager Native API
#endif // (AHI_TL_SUPPORT)

#if (BLE_ISOOHCI)
    #include "isoohci.h"
#endif //(BLE_ISOOHCI)

#include "dbg.h"
/*
 * DEFINES
 ****************************************************************************************
 */

/// Size of the RX Buffer that can both receive the header and valid packet to exit from out of sync mode
#if (AHI_TL_SUPPORT)
    #define RX_TMP_BUFF_SIZE     AHI_RESET_MSG_LEN
#elif (HCI_TL_SUPPORT)
    #define RX_TMP_BUFF_SIZE     HCI_RESET_MSG_LEN
#endif // (AHI_TL_SUPPORT) or (HCI_TL_SUPPORT)


/*
 * ENUMERATIONS DEFINTION
 ****************************************************************************************
 */

///H4TL RX states
enum H4TL_STATE_RX
{
    ///H4TL RX Start State - receive message type
    H4TL_STATE_RX_START,
    ///H4TL RX Header State - receive message header
    H4TL_STATE_RX_HDR,
    ///H4TL RX Header State - receive (rest of) message payload
    H4TL_STATE_RX_PAYL,
    ///H4TL RX Out Of Sync state - receive message type
    H4TL_STATE_RX_OUT_OF_SYNC
};


///H4TL queue type
enum H4TL_QUEUE
{
    /// HCI/AHI Tx Queue
    H4TL_TX_QUEUE_MSG,

#if (BLE_ISOOHCI)
    /// ISO Tx Queue
    H4TL_TX_QUEUE_ISO,
#endif // (BLE_ISOOHCI)

#if (TRACER_PRESENT)
    /// Tracer Message Queue
    H4TL_TX_QUEUE_TRACER,
#endif // (TRACER_PRESENT)

    /// Maximum number of TX queue
    H4TL_TX_QUEUE_MAX,

    /// Queue in Idle mode
    H4TL_TX_QUEUE_IDLE  = 0xFF,
};



/*
 * STRUCTURES DEFINITION
 ****************************************************************************************
 */
/// Information about message under transmission
typedef struct h4tl_tx_info_t
{
    /// message buffer
    uint8_t *buf;
    /// Tx callback
    void (*callback)(void);
    /// buffer length
    uint16_t buf_len;
    /// Priority
    uint8_t prio;
} h4tl_tx_info_t;

/// H4TL Environment context structure

typedef struct h4tl_env_
{
    /// Defer end of data reception
    co_djob_t rx_djob;
    /// Defer end of data transmission
    co_djob_t tx_djob;
    /// pointer to External interface api
    const struct bt_eif_api *ext_if;
    ///Pointer to space reserved for received payload.
    uint8_t *curr_payl_buff;
    /// Maximum Tx queue
    h4tl_tx_info_t tx_queue[H4TL_TX_QUEUE_MAX];
    /// Ensure that array is 32bits aligned
    /// Latest received message header, or used to receive a message allowing to exit from out of sync
    uint8_t rx_buf[RX_TMP_BUFF_SIZE];
    ///Rx state - can be receiving message type, header, payload or error
    uint8_t rx_state;
    ///Latest received message type: CMD/EVT/ACL.
    uint8_t rx_type;
    /// Reception status
    uint8_t rx_status;
    /// Transport layer interface
    uint8_t tl_itf;
    /// Information about on-going transmission
    uint8_t tx_active_queue;

#if (RW_DEBUG)
    /// Data length expected to be received
    uint16_t rx_len;
#endif // (RW_DEBUG)
} h4tl_env_t;

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///HCI table for correspondence between External Interface message type and header length.
__STATIC const uint8_t h4tl_msgtype2hdrlen[] =
{
#if (HCI_TL_SUPPORT)
    [HCI_CMD_MSG_TYPE]     = HCI_CMD_HDR_LEN,
    [HCI_ACL_MSG_TYPE]     = HCI_ACL_HDR_LEN,
#if (BT_EMB_PRESENT || BT_HOST_PRESENT)
#if (VOICE_OVER_HCI)
    [HCI_SYNC_MSG_TYPE]    = HCI_SYNC_HDR_LEN,
#endif // (VOICE_OVER_HCI)
#endif // (BT_EMB_PRESENT || BT_HOST_PRESENT)
    [HCI_EVT_MSG_TYPE]     = HCI_EVT_HDR_LEN,
#endif // (HCI_TL_SUPPORT)
#if (AHI_TL_SUPPORT)
    [AHI_KE_MSG_TYPE]      = AHI_KE_MSG_HDR_LEN,
#endif // (AHI_TL_SUPPORT)
#if (BLE_ISOOHCI)
    [HCI_ISO_MSG_TYPE]     = HCI_ISO_HDR_LEN,
#endif //(BLE_ISOOHCI)
#if (TRACER_PRESENT)
    [TRC_MSG_TYPE]         = TRC_MSG_HDR_LEN,
#endif // (TRACER_PRESENT)
};

/// H4TL environment context
__STATIC h4tl_env_t h4tl_env[H4TL_NB_CHANNEL_MAX];

/*
 * LOCAL FUNCTION DECLARATION
 ****************************************************************************************
 */

#if (HCI_TL_SUPPORT)
    #if (EMB_PRESENT)
        __STATIC void h4tl_rx_cmd_hdr_extract(h4tl_env_t *env, struct hci_cmd_hdr *p_hdr);
    #endif //(EMB_PRESENT)
    #if (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
        __STATIC void h4tl_rx_acl_hdr_extract(h4tl_env_t *env, struct hci_acl_hdr *p_hdr);
    #endif // (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
    #if (HOST_PRESENT && !EMB_PRESENT)
        __STATIC void h4tl_rx_evt_hdr_extract(h4tl_env_t *env, struct hci_evt_hdr *p_hdr);
    #endif // (HOST_PRESENT && !EMB_PRESENT)
    #if (EMB_PRESENT)
        __STATIC void h4tl_cmd_hdr_rx_evt_handler(h4tl_env_t *p_env);
        __STATIC void h4tl_cmd_pld_rx_evt_handler(h4tl_env_t *p_env);
    #endif //(EMB_PRESENT)
    #if (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
        __STATIC void h4tl_acl_hdr_rx_evt_handler(h4tl_env_t *p_env);
    #endif // (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)

#endif // (HCI_TL_SUPPORT)

__STATIC void h4tl_read_start(h4tl_env_t *p_env);
__STATIC void h4tl_read_hdr(h4tl_env_t *p_env, uint8_t len);
__STATIC void h4tl_read_payl(h4tl_env_t *p_env, uint16_t len);
__STATIC void h4tl_read_next_out_of_sync(h4tl_env_t *p_env);
__STATIC void h4tl_out_of_sync(h4tl_env_t *p_env);
__STATIC bool h4tl_out_of_sync_check(h4tl_env_t *p_env);
__STATIC void h4tl_tx_done_isr_handler(h4tl_env_t *p_env, uint8_t status);
__STATIC void h4tl_rx_done_isr_handler(h4tl_env_t *p_env, uint8_t status);
__STATIC void h4tl_tx_done_handler(co_djob_t *p_djob);
__STATIC void h4tl_rx_done_handler(co_djob_t *p_djob);


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (HCI_TL_SUPPORT)
#if (EMB_PRESENT)
/**
****************************************************************************************
* @brief Local function : extracts command header components
*
* @param[in] env Environment of transport layer
* @param[out]   p_hdr   Pointer to command header structure
*****************************************************************************************
*/
__STATIC void h4tl_rx_cmd_hdr_extract(h4tl_env_t *env, struct hci_cmd_hdr *p_hdr)
{
    //extract command header:opcode, parameter length
    p_hdr->opcode = co_btohs(co_read16p(&(env->rx_buf[0])));
    p_hdr->parlen = env->rx_buf[HCI_CMD_OPCODE_LEN];
}
#endif //(EMB_PRESENT)

#if (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
/**
****************************************************************************************
* @brief Local function : extracts ACL header components
*
* @param[in] env Environment of transport layer
* @param[out]   p_hdr   Pointer to ACL header structure
*****************************************************************************************
*/
__STATIC void h4tl_rx_acl_hdr_extract(h4tl_env_t *env, struct hci_acl_hdr *p_hdr)
{
    // Extract ACL header: data length, connection handle and flags
    p_hdr->datalen = co_btohs(co_read16p(env->rx_buf + HCI_ACL_HDR_HDL_FLAGS_LEN));
    p_hdr->hdl_flags = co_btohs(co_read16p(env->rx_buf));
}
#endif //(HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)

#if ((BT_EMB_PRESENT || BT_HOST_PRESENT) && VOICE_OVER_HCI)
/**
****************************************************************************************
* @brief Local function : extracts synchronous header components
*
* @param[in] env Environment of transport layer
* @param[out]   p_hdr   Pointer to synchronous header structure
*****************************************************************************************
*/
__STATIC void h4tl_rx_sync_hdr_extract(h4tl_env_t *env, struct hci_sync_hdr *p_hdr)
{
    // Extract ACL header: data length, connection handle and flags
    p_hdr->data_total_len = *(env->rx_buf + HCI_SYNC_HDR_HDL_FLAGS_LEN);
    p_hdr->conhdl_flags = co_btohs(co_read16p(env->rx_buf));
}
#endif // ((BT_EMB_PRESENT || BT_HOST_PRESENT) && VOICE_OVER_HCI)

#if (HOST_PRESENT && !EMB_PRESENT)
/**
****************************************************************************************
* @brief Local function : extracts event header components
*
* @param[in] env Environment of transport layer
* @param[out]   p_hdr   Pointer to event header structure
*****************************************************************************************
*/
__STATIC void h4tl_rx_evt_hdr_extract(h4tl_env_t *env, struct hci_evt_hdr *p_hdr)
{
    //extract event header:code, parameter length
    p_hdr->code   = env->rx_buf[0];
    p_hdr->parlen = env->rx_buf[HCI_EVT_CODE_LEN];
}
#endif // (HOST_PRESENT && !EMB_PRESENT)

#endif // (HCI_TL_SUPPORT)

/**
******************************************************************************************
* @brief Local function : places H4TL in RX_START state and sets the External Interface environment.
*
* @param[in] p_env Environment of transport layer
******************************************************************************************
*/
__STATIC void h4tl_read_start(h4tl_env_t *p_env)
{
    //Initialize External Interface in reception mode state
    p_env->rx_state = H4TL_STATE_RX_START;

    //Set the External Interface environment to message type 1 byte reception
    p_env->ext_if->read(&(p_env->rx_buf[RX_TMP_BUFF_SIZE - 1]), H4TL_LOGICAL_CHANNEL_LEN, (rwip_eif_callback) &h4tl_rx_done_isr_handler, p_env);

#if (EMB_PRESENT)
    // No HCI reception is ongoing, so allow going to sleep
    rwip_prevent_sleep_clear(RW_TL_1_RX_ONGOING << p_env->tl_itf);
#endif // (EMB_PRESENT)
}


/**
****************************************************************************************
* @brief Local function : places H4TL in RX header state and sets the External Interface env.
*
* @param[in] p_env Environment of transport layer
* @param[in] len Length of header to be received in the currently set buffer.
*****************************************************************************************
*/
__STATIC void h4tl_read_hdr(h4tl_env_t *p_env, uint8_t len)
{
    //change Rx state - wait for header next
    p_env->rx_state = H4TL_STATE_RX_HDR;

    //set External Interface environment to header reception of len bytes
    p_env->ext_if->read(&p_env->rx_buf[0], len, (rwip_eif_callback) &h4tl_rx_done_isr_handler, p_env);

    // An HCI reception is ongoing
    rwip_prevent_sleep_set(RW_TL_1_RX_ONGOING << p_env->tl_itf);
}


/**
******************************************************************************************
* @brief Local function : places H4TL in RX payload state and request the External IF
*
* @param[in] p_env Environment of transport layer
* @param[in] buf Buffer for payload reception
* @param[in] len Length of payload to be received in the currently set buffer.
******************************************************************************************
*/
__STATIC void h4tl_read_payl(h4tl_env_t *p_env, uint16_t len)
{
    //change rx state to payload reception
    p_env->rx_state = H4TL_STATE_RX_PAYL;

#if (RW_DEBUG)
    p_env->rx_len = len;
#endif // (RW_DEBUG)

    //set External Interface environment to payload reception of len bytes
    p_env->ext_if->read(p_env->curr_payl_buff, len, (rwip_eif_callback) &h4tl_rx_done_isr_handler, p_env);
}

/**
******************************************************************************************
* @brief Local function : places H4TL in RX_START_OUT_OF_SYNC state.
*
* @param[in] p_env Environment of transport layer
******************************************************************************************
*/
__STATIC void h4tl_read_next_out_of_sync(h4tl_env_t *p_env)
{
    //Set External Interface reception state to H4TL_STATE_RX_START_OUT_OF_SYNC
    p_env->rx_state = H4TL_STATE_RX_OUT_OF_SYNC;

    //Set the External Interface environment to 1 byte reception (at end of rx buffer)
    p_env->ext_if->read(&(p_env->rx_buf[RX_TMP_BUFF_SIZE - 1]), H4TL_LOGICAL_CHANNEL_LEN, (rwip_eif_callback) &h4tl_rx_done_isr_handler, p_env);
}

/**
 *****************************************************************************************
 *@brief Static function handling External Interface out of synchronization detection.
 *
 * At External Interface reception, when packet indicator opcode of a command is not
 * recognized.
 *
* @param[in] env Environment of transport layer
 *****************************************************************************************
 */
__STATIC void h4tl_out_of_sync(h4tl_env_t *p_env)
{
#if (HCI_TL_SUPPORT && (EMB_PRESENT))
    // Send HCI hardware error event
    struct hci_hw_err_evt *evt = KE_MSG_ALLOC(HCI_EVENT, 0, HCI_HW_ERR_EVT_CODE, hci_hw_err_evt);
    evt->hw_code = CO_ERROR_HW_UART_OUT_OF_SYNC;
    hci_send_2_host(evt);
#endif // (HCI_TL_SUPPORT && (EMB_PRESENT))

    // Initialize receive buffer
    memset(&(p_env->rx_buf[0]), 0, RX_TMP_BUFF_SIZE - 2);
    // keep last octet received
    p_env->rx_buf[RX_TMP_BUFF_SIZE - 2] = p_env->rx_buf[RX_TMP_BUFF_SIZE - 1];
    // Start reception of new packet ID
    h4tl_read_next_out_of_sync(p_env);

#if (EMB_PRESENT)
    // No HCI reception is ongoing, so allow going to sleep
    rwip_prevent_sleep_clear(RW_TL_1_RX_ONGOING << p_env->tl_itf);
#endif // (EMB_PRESENT)
}


/**
 ****************************************************************************************
 * @brief Check received byte in out of sync state
 *
 * This function is the algorithm to check that received byte stream in out of sync state
 * corresponds to HCI_reset command.
 *
 * Level of reception is incremented when bytes of HCI_reset_cmd are detected.
 *
 * @param[in] p_env Environment of transport layer
 *****************************************************************************************
 */
__STATIC bool h4tl_out_of_sync_check(h4tl_env_t *p_env)
{
    bool sync_ok = false;

#if (HCI_TL_SUPPORT)
#if (EMB_PRESENT)
    const uint8_t hci_reset_msg[HCI_RESET_MSG_LEN] = HCI_RESET_MSG_BUF;

    // check if valid HCI reset has been received
    if (memcmp(&(hci_reset_msg[0]), &(p_env->rx_buf[RX_TMP_BUFF_SIZE - HCI_RESET_MSG_LEN]), HCI_RESET_MSG_LEN) == 0)
    {
        // HCI processes the command
        hci_tl_cmd_get_max_param_size(HCI_RESET_CMD_OPCODE); // force command descriptor search
        hci_tl_cmd_received(HCI_TL_H4, HCI_RESET_CMD_OPCODE, 0, NULL);
        sync_ok = true;
    }
#endif //(EMB_PRESENT)
#endif // (HCI_TL_SUPPORT)

#if (AHI_TL_SUPPORT)
    // Check reset message
    if (!sync_ok)
    {
        const uint8_t ahi_reset_msg[AHI_RESET_MSG_LEN] = AHI_RESET_MSG_BUF;
        // check if valid HCI reset has been received
        if (memcmp(&(ahi_reset_msg[0]), &(p_env->rx_buf[RX_TMP_BUFF_SIZE - AHI_RESET_MSG_LEN]), AHI_RESET_MSG_LEN) == 0)
        {
            //TODO: check why use ahi_reset.
            ahi_reset();
            sync_ok = true;
        }
    }
#endif // (AHI_TL_SUPPORT)

    // sync not found, ensure that a packet will be received
    if (!sync_ok)
    {
        uint8_t i;

        // An HCI reception is ongoing
        rwip_prevent_sleep_set(RW_TL_1_RX_ONGOING << p_env->tl_itf);

        // shift received bytes left into rx buffer
        for (i = 0 ; i < (RX_TMP_BUFF_SIZE - 1); i++)
        {
            p_env->rx_buf[i] = p_env->rx_buf[i + 1];
        }
    }

    return sync_ok;
}

/**
 ****************************************************************************************
 * @brief Callback for TL to indicate the end of TX
 *
 * @param[in] p_env  Pointer to environment of transport layer
 * @param[in] status External Interface Tx status: ok or error.
 *****************************************************************************************
 */
__STATIC void h4tl_tx_done_isr_handler(h4tl_env_t *p_env, uint8_t status)
{
    // Sanity check: Transmission should always work
    ASSERT_ERR(status == RWIP_EIF_STATUS_OK);

    // defer execution
    if (H4TL_CHANNEL_DATA == p_env->tl_itf)
    {
        co_djob_isr_reg(&(p_env->tx_djob));
    }
    else
    {
        co_djob_reg(CO_DJOB_LOW, &(p_env->tx_djob));
    }
}


/**
 ****************************************************************************************
 * @brief Function called at each RX interrupt.
 *
 * @param[in] env    Pointer to environment of transport layer
 * @param[in] status External Interface RX status: ok or error
 *****************************************************************************************
 */
__STATIC void h4tl_rx_done_isr_handler(h4tl_env_t *p_env, uint8_t status)
{
    p_env->rx_status = status;
    if (H4TL_CHANNEL_DATA == p_env->tl_itf)
    {
        // Yier: 1800 is a test value to avoid memory is not enough
        if (ke_is_lower_than_target_all(2000))
            co_djob_reg(CO_DJOB_HIGH, &(p_env->rx_djob));
        else
            co_djob_isr_reg(&(p_env->rx_djob));
    }
    else
    {
        co_djob_reg(CO_DJOB_LOW, &(p_env->rx_djob));
    }
}

/**
 ****************************************************************************************
 * @brief Handle TX ISR in a deferred function
 *
 * @param[in] p_djob  Pointer to the delay job environment
 *****************************************************************************************
 */
__STATIC void h4tl_tx_done_handler(co_djob_t *p_djob)
{
    h4tl_env_t *p_env = CONTAINER_OF(p_djob,  h4tl_env_t, tx_djob);
    uint8_t queue = p_env->tx_active_queue;
    h4tl_tx_info_t *tx_info = &(p_env->tx_queue[queue]);
    uint8_t cursor;
    uint8_t prio = 0;
    bool on_going_tx = false;

    ASSERT_INFO((queue != H4TL_TX_QUEUE_IDLE), p_djob, p_env);

    // mark transmission done
    tx_info->buf = NULL;
    // execute TX done callback
    if (tx_info->callback != NULL)
    {
        tx_info->callback();
    }

    p_env->tx_active_queue = H4TL_TX_QUEUE_IDLE;

    // by default nothing to do
    queue = H4TL_TX_QUEUE_IDLE;

    // search new transmission to perform
    tx_info = NULL;
    for (cursor = 0 ; cursor < H4TL_TX_QUEUE_MAX; cursor++)
    {
        tx_info = &(p_env->tx_queue[cursor]);
        if ((tx_info->buf != NULL) && (tx_info->prio > prio))
        {
            queue = cursor;
            prio = tx_info->prio;
        }
    }

    p_env->tx_active_queue = queue;
    if (queue != H4TL_TX_QUEUE_IDLE)
    {
        tx_info = &(p_env->tx_queue[queue]);
        p_env->ext_if->write(tx_info->buf, tx_info->buf_len + HCI_TRANSPORT_HDR_LEN,
                             (rwip_eif_callback)&h4tl_tx_done_isr_handler, p_env);
        on_going_tx = true;
    }

    // Deep Sleep management
    if (!on_going_tx)
    {
        // No HCI reception is ongoing, so allow going to sleep
        rwip_prevent_sleep_clear(RW_TL_1_TX_ONGOING << p_env->tl_itf);
    }
}

/**
 ****************************************************************************************
 * @brief Function called at each RX interrupt deferred.
 *
 * @param[in] p_djob  Pointer to the delay job environment
 *****************************************************************************************
 */
__STATIC void h4tl_rx_done_handler(co_djob_t *p_djob)
{
    h4tl_env_t *p_env = CONTAINER_OF(p_djob,  h4tl_env_t, rx_djob);

    // detect that an event occurs on interface
    if (p_env->rx_status != RWIP_EIF_STATUS_OK)
    {
        //detect External Interface RX error and handle accordingly
        if ((p_env->rx_status == RWIP_EIF_STATUS_ERROR) || (p_env->rx_state != H4TL_STATE_RX_START))
        {
            // External Interface RX error -> enter in out of sync
            h4tl_out_of_sync(p_env);
        }
        else
        {
            // restart logical channel reception
            h4tl_read_start(p_env);
        }
    }
    else
    {
        //check HCI state to see what was received
        switch (p_env->rx_state)
        {
        /* RECEIVE MESSAGE TYPE STATE*/
        case H4TL_STATE_RX_START:
        {
            // extract RX type
            p_env->rx_type = p_env->rx_buf[RX_TMP_BUFF_SIZE - 1];

            // Check received packet indicator
            switch (p_env->rx_type)
            {
#if (HCI_TL_SUPPORT)
#if (EMB_PRESENT)
            case HCI_CMD_MSG_TYPE:
#endif //(EMB_PRESENT)
            case HCI_ACL_MSG_TYPE:
#if ((BT_EMB_PRESENT || BT_HOST_PRESENT) && VOICE_OVER_HCI)
            case HCI_SYNC_MSG_TYPE:
#endif // ((BT_EMB_PRESENT || BT_HOST_PRESENT) && VOICE_OVER_HCI)
#if (!EMB_PRESENT && HOST_PRESENT)
            case HCI_EVT_MSG_TYPE:
#endif // (!EMB_PRESENT && HOST_PRESENT)
#if (BLE_ISOOHCI)
            case HCI_ISO_MSG_TYPE:
#endif //(BLE_ISOOHCI)
#endif // (HCI_TL_SUPPORT)
#if (AHI_TL_SUPPORT)
            case AHI_KE_MSG_TYPE:
#endif // (AHI_TL_SUPPORT)
#if (TRACER_PRESENT)
            case TRC_MSG_TYPE:
#endif /*(TRACER_PRESENT)*/
            {
                //change state to header reception
                h4tl_read_hdr(p_env, h4tl_msgtype2hdrlen[p_env->rx_type]);
            }
            break;

            default:
            {
                // Incorrect packet indicator -> enter in out of sync
                h4tl_out_of_sync(p_env);
            }
            break;
            }
        }
        break;
        /* RECEIVE MESSAGE TYPE STATE END*/

        /* RECEIVE HEADER STATE*/
        case H4TL_STATE_RX_HDR:
        {
            switch (p_env->rx_type)
            {
#if (HCI_TL_SUPPORT)
#if (EMB_PRESENT)
            //Command Header reception
            case HCI_CMD_MSG_TYPE:
            {
                h4tl_cmd_hdr_rx_evt_handler(p_env);
            }
            break;
#endif //(EMB_PRESENT)

#if (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
            case HCI_ACL_MSG_TYPE:
            {
                h4tl_acl_hdr_rx_evt_handler(p_env);
            }
            break;
#endif //(HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)

#if ((BT_EMB_PRESENT || BT_HOST_PRESENT) && VOICE_OVER_HCI)
            case HCI_SYNC_MSG_TYPE:
            {
                struct hci_sync_hdr *p_hdr = (struct hci_sync_hdr *) p_env->rx_buf;

                // Extract the Synchronous header components
                h4tl_rx_sync_hdr_extract(p_env, p_hdr);

                // HCI allocate a buffer for data reception
                p_env->curr_payl_buff = hci_tl_sync_tx_data_alloc(HCI_TL_H4, p_hdr->conhdl_flags,
                                        p_hdr->data_total_len);

                // Check data length
                if (p_env->curr_payl_buff == NULL)
                {
                    // Incorrect payload size -> enter in out of sync
                    h4tl_out_of_sync(p_env);
                }
                else
                {
                    //change HCI rx state to payload reception
                    h4tl_read_payl(p_env, p_hdr->data_total_len);
                }
            }
            break;
#endif // ((BT_EMB_PRESENT || BT_HOST_PRESENT) && VOICE_OVER_HCI)

#if (HOST_PRESENT && !EMB_PRESENT)
            //Event Header reception
            case HCI_EVT_MSG_TYPE:
            {
                struct hci_evt_hdr *p_hdr = (struct hci_evt_hdr *) p_env->rx_buf;

                // Extract the event header components
                h4tl_rx_evt_hdr_extract(p_env, p_hdr);

                // HCI allocate a buffer for data reception
                p_env->curr_payl_buff = hci_tl_evt_data_alloc(HCI_TL_H4, p_hdr->code, p_hdr->parlen);

                // Problem in the RX buffer allocation -> enter in out of sync
                if (p_env->curr_payl_buff == NULL)
                {
                    h4tl_out_of_sync(p_env);
                }
                // Check if the event has parameters
                else if (p_hdr->parlen == 0)
                {
                    // HCI processes the event
                    hci_tl_evt_received(HCI_TL_H4, p_hdr->code, p_hdr->parlen, p_env->curr_payl_buff);

                    //change hci rx state to message type reception
                    h4tl_read_start(p_env);
                }
                else
                {
                    //change HCI rx state to payload reception
                    h4tl_read_payl(p_env, p_hdr->parlen);
                }
            }
            break;
#endif // (HOST_PRESENT && !EMB_PRESENT)
#endif // (HCI_TL_SUPPORT)

#if (AHI_TL_SUPPORT)
            case AHI_KE_MSG_TYPE:
            {
                uint16_t rem_length;
                uint8_t *p_rx_data = ahi_rx_hdr_handle(&p_env->rx_buf[0], &rem_length);

                //no params
                if (p_rx_data == NULL)
                {
                    // No buffer can handle received data
                    h4tl_out_of_sync(p_env);
                }
                else if (rem_length == 0)
                {
                    // inform that reception is over
                    ahi_rx_done(p_rx_data);
                    // Restart a new packet reception
                    h4tl_read_start(p_env);
                }
                else
                {
                    p_env->curr_payl_buff = p_rx_data;
                    // Start payload reception
                    h4tl_read_payl(p_env, rem_length);
                }
            }
            break;
#endif // (AHI_TL_SUPPORT)

#if (BLE_ISOOHCI)
            case HCI_ISO_MSG_TYPE:
            {
                struct hci_iso_hdr *p_hdr = (struct hci_iso_hdr *) &p_env->rx_buf[0];

                // Allocate a buffer for data reception
                p_env->curr_payl_buff = isoohci_in_buf_alloc(p_hdr);

                // Check data length
                if (p_env->curr_payl_buff == NULL)
                {
                    // Incorrect payload size -> enter in out of sync
                    h4tl_out_of_sync(p_env);
                }
                else
                {
                    //change HCI rx state to payload reception
                    h4tl_read_payl(p_env, GETF(p_hdr->iso_data_load_len, HCI_ISO_HDR_ISO_DATA_LOAD_LEN));
                }
            }
            break;
#endif //(BLE_ISOOHCI)

#if (TRACER_PRESENT)
            case TRC_MSG_TYPE:
            {
                //change rx state to payload reception
                p_env->curr_payl_buff = dbg_trc_pay_buff_get();
                h4tl_read_payl(p_env, co_read16p(p_env->rx_buf + 1));
            }
            break;
#endif /*(TRACER_PRESENT)*/

            default:
            {
                ASSERT_INFO_FORCE(0, p_env->rx_type, p_env->tl_itf);
            }
            break;
            }//end switch

#if(RW_DEBUG)
            if (p_env->rx_state == H4TL_STATE_RX_START)
            {
                DUMP_HCI(p_env->rx_type, 1, p_env->rx_buf, h4tl_msgtype2hdrlen[p_env->rx_type]);
            }
#endif // (RW_DEBUG)
        }
        break;
        /* RECEIVE HEADER STATE END*/

        /* RECEIVE PAYLOAD STATE */
        case H4TL_STATE_RX_PAYL:
        {
            DUMP_HCI_2(p_env->rx_type, 1, p_env->rx_buf, h4tl_msgtype2hdrlen[p_env->rx_type],
                       p_env->curr_payl_buff, p_env->rx_len);

            switch (p_env->rx_type)
            {
#if (HCI_TL_SUPPORT)
#if (EMB_PRESENT)
            case HCI_CMD_MSG_TYPE:
            {
                h4tl_cmd_pld_rx_evt_handler(p_env);
            }
            break;
#endif //(EMB_PRESENT)

            case HCI_ACL_MSG_TYPE:
            {
#if (BT_EMB_PRESENT || (BLE_EMB_PRESENT && HCI_BLE_CON_SUPPORT))
                struct hci_acl_hdr *p_hdr = (struct hci_acl_hdr *) p_env->rx_buf;
                // HCI processes the data
                hci_tl_acl_tx_data_received(HCI_TL_H4, p_hdr->hdl_flags, p_hdr->datalen, p_env->curr_payl_buff);
                //change hci rx state to message type reception - common to all types
                h4tl_read_start(p_env);
#elif (HOST_PRESENT && !EMB_PRESENT)
                struct hci_acl_hdr *p_hdr = (struct hci_acl_hdr *) p_env->rx_buf;
                // HCI processes the data
                hci_tl_acl_rx_data_received(HCI_TL_H4, p_hdr->hdl_flags, p_hdr->datalen, p_env->curr_payl_buff);
                //change hci rx state to message type reception - common to all types
                h4tl_read_start(p_env);
#endif //BLE_EMB_PRESENT/HOST_PRESENT/BT_EMB_PRESENT
            }
            break;

#if ((BT_EMB_PRESENT || BT_HOST_PRESENT) && VOICE_OVER_HCI)
            case HCI_SYNC_MSG_TYPE:
            {
                struct hci_sync_hdr *p_hdr = (struct hci_sync_hdr *) p_env->rx_buf;
                // HCI processes the data
                hci_tl_sync_tx_data_received(HCI_TL_H4, p_hdr->conhdl_flags, p_hdr->data_total_len, p_env->curr_payl_buff);

                //change hci rx state to message type reception - common to all types
                h4tl_read_start(p_env);
            }
            break;
#endif // ((BT_EMB_PRESENT || BT_HOST_PRESENT) && VOICE_OVER_HCI)

#if (HOST_PRESENT && !EMB_PRESENT)
            case HCI_EVT_MSG_TYPE:
            {
                struct hci_evt_hdr *p_hdr = (struct hci_evt_hdr *) p_env->rx_buf;

                // HCI processes the event
                hci_tl_evt_received(HCI_TL_H4, p_hdr->code, p_hdr->parlen, p_env->curr_payl_buff);

                // Clear current payload buffer pointer
                p_env->curr_payl_buff = NULL;

                //change hci rx state to message type reception - common to all types
                h4tl_read_start(p_env);
            }
            break;
#endif //((HOST_PRESENT) && (!BLE_EMB_PRESENT))
#endif // (HCI_TL_SUPPORT)
#if (AHI_TL_SUPPORT)
            case AHI_KE_MSG_TYPE:
            {
                // Handle received data
                ahi_rx_done(p_env->curr_payl_buff);
                p_env->curr_payl_buff = NULL;

                // Restart a new packet reception
                h4tl_read_start(p_env);
            }
            break;
#endif // (AHI_TL_SUPPORT)
#if (BLE_ISOOHCI)
            case HCI_ISO_MSG_TYPE:
            {
                // Handle reception of the packet
                isoohci_in_buf_received((struct hci_iso_hdr *) &p_env->rx_buf, p_env->curr_payl_buff);

                // Restart a new packet reception
                h4tl_read_start(p_env);
            }
            break;
#endif //(BLE_ISOOHCI)
#if (TRACER_PRESENT)
            case TRC_MSG_TYPE:
            {
                // Alert the tracer that the configuration packet has been fully received
                dbg_trc_cfg_received();

                // Restart a new packet reception
                h4tl_read_start(p_env);
            }
            break;
#endif /*(TRACER_PRESENT)*/
            default:
            {
                ASSERT_INFO_FORCE(0, p_env->rx_type, p_env->tl_itf);
            }
            break;
            }
        }
        break;
        /* RECEIVE PAYLOAD STATE END*/

        /* RX OUT OF SYNC STATE */
        case H4TL_STATE_RX_OUT_OF_SYNC:
        {
            // Check received byte
            if (h4tl_out_of_sync_check(p_env))
            {
                // sync found, return to normal mode
                h4tl_read_start(p_env);
            }
            else
            {
                // Start a new byte reception
                h4tl_read_next_out_of_sync(p_env);
            }
        }
        break;
        /* RX OUT OF SYNC STATE END*/

        /* DEFAULT STATE */
        default:
        {
            ASSERT_ERR_FORCE(0);
        }
        break;
            /* DEFAULT END*/

        }
        /* STATE SWITCH END */
    }
}

#if (HCI_TL_SUPPORT)
#if (EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Actions after reception of HCI command header
 *
 * @param[in] p_env Environment of transport layer
 *****************************************************************************************
 */
__STATIC void h4tl_cmd_hdr_rx_evt_handler(h4tl_env_t *p_env)
{
    struct hci_cmd_hdr *p_hdr = (struct hci_cmd_hdr *) p_env->rx_buf;

    // Extract the command header components
    h4tl_rx_cmd_hdr_extract(p_env, p_hdr);

    // Check received parameter size
    if (p_hdr->parlen > hci_tl_cmd_get_max_param_size(p_hdr->opcode))
    {
        // Incorrect header -> enter in out of sync
        h4tl_out_of_sync(p_env);
    }
    // Check if the command has parameters
    else if (p_hdr->parlen == 0)
    {
        // HCI processes the command
        hci_tl_cmd_received(HCI_TL_H4, p_hdr->opcode, p_hdr->parlen, NULL);

        //change hci rx state to message type reception
        h4tl_read_start(p_env);
    }
    else
    {
        // Allocate memory buffer for receiving the payload
        p_env->curr_payl_buff = (uint8_t *) ke_malloc_system(p_hdr->parlen, KE_MEM_KE_MSG);

        if (p_env->curr_payl_buff != NULL)
        {
            //change HCI rx state to payload reception
            h4tl_read_payl(p_env, p_hdr->parlen);
        }
        else
        {
            // Problem in the RX buffer allocation -> enter in out of sync
            h4tl_out_of_sync(p_env);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Actions after reception of HCI command payload
 *
 * @param[in] p_env Environment of transport layer
 *****************************************************************************************
 */
__STATIC void h4tl_cmd_pld_rx_evt_handler(h4tl_env_t *p_env)
{
    struct hci_cmd_hdr *p_hdr = (struct hci_cmd_hdr *) p_env->rx_buf;

    // HCI processes the command
    hci_tl_cmd_received(HCI_TL_H4, p_hdr->opcode, p_hdr->parlen, p_env->curr_payl_buff);

    if (p_env->curr_payl_buff != NULL)
    {
        // Free payload buffer
        ke_free(p_env->curr_payl_buff); // TODO if resource is not an allocated memory block, an error can occurs

        // Clear current payload buffer pointer
        p_env->curr_payl_buff = NULL;
    }

    //change hci rx state to message type reception - common to all types
    h4tl_read_start(p_env);
}
#endif //(EMB_PRESENT)

#if (HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
/**
 ****************************************************************************************
 * @brief Actions after reception of HCI ACL header
 *
 * @param[in] p_env Environment of transport layer
 *****************************************************************************************
 */
__STATIC void h4tl_acl_hdr_rx_evt_handler(h4tl_env_t *p_env)
{
    struct hci_acl_hdr *p_hdr = (struct hci_acl_hdr *) p_env->rx_buf;

    // Extract the ACL header components
    h4tl_rx_acl_hdr_extract(p_env, p_hdr);

    if (p_hdr->datalen > 0)
    {
#if (EMB_PRESENT)
        // HCI allocate a buffer for data reception (HL rx or LL tx)
        p_env->curr_payl_buff = hci_tl_acl_tx_data_alloc(HCI_TL_H4, p_hdr->hdl_flags, p_hdr->datalen);
#elif HOST_PRESENT
        // HCI allocate a buffer for data reception (HL rx or LL tx)
        p_env->curr_payl_buff = hci_tl_acl_rx_data_alloc(HCI_TL_H4, p_hdr->hdl_flags, p_hdr->datalen);
#else
        ASSERT_ERR_FORCE(0);
#endif //BLE_EMB_PRESENT/HOST_PRESENT

        // Check data length
        if (p_env->curr_payl_buff == NULL)
        {
            // Incorrect payload size -> enter in out of sync
            h4tl_out_of_sync(p_env);
        }
        else
        {
            //change HCI rx state to payload reception
            h4tl_read_payl(p_env, p_hdr->datalen);
        }
    }
    else
    {
#if (EMB_PRESENT)
        // HCI processes the data
        hci_tl_acl_tx_data_received(HCI_TL_H4, p_hdr->hdl_flags, 0, NULL);
#elif HOST_PRESENT
        // HCI processes the data
        hci_tl_acl_rx_data_received(HCI_TL_H4, p_hdr->hdl_flags, 0, NULL);
#else
        ASSERT_ERR_FORCE(0);
#endif //BLE_EMB_PRESENT/HOST_PRESENT

        // Restart reception of new packet
        h4tl_read_start(p_env);
    }
}
#endif //(HCI_BLE_CON_SUPPORT || BT_EMB_PRESENT)
#endif //(HCI_TL_SUPPORT)


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */



void h4tl_init(uint8_t tl_itf, const struct bt_eif_api *eif)
{
    uint8_t cursor;

    h4tl_env_t *p_env = &(h4tl_env[tl_itf]);
    // Store external interface API
    p_env->ext_if              = eif;
    //initialize tx state
    p_env->tx_active_queue     = H4TL_TX_QUEUE_IDLE;
    p_env->tl_itf              = tl_itf;
    // Initialize buffers
    for (cursor = 0 ; cursor < H4TL_TX_QUEUE_MAX; cursor ++)
    {
        p_env->tx_queue[cursor].buf  = NULL;

        // TODO priority mechanism to be changed
        p_env->tx_queue[cursor].prio = 0xFF - cursor;
    }

    // Enable External Interface
    p_env->ext_if->flow_on();

    // setup defer handlers
    co_djob_init(&(p_env->tx_djob), h4tl_tx_done_handler);
    co_djob_init(&(p_env->rx_djob), h4tl_rx_done_handler);

    //start External Interface reception
    h4tl_read_start(p_env);
}


void h4tl_write(uint8_t type, uint8_t *buf, uint16_t len, void (*tx_callback)(void))
{
    uint8_t tl_itf   = H4TL_NB_CHANNEL;
    uint8_t tx_queue = H4TL_TX_QUEUE_IDLE;
    DUMP_HCI(type, 0, buf, len);


    //pack event type message (External Interface header)
    buf -= HCI_TRANSPORT_HDR_LEN;
    *buf = type;

    // retrieve Transport Layer Type
    switch (type)
    {
#if (AHI_TL_SUPPORT)
    case AHI_KE_MSG_TYPE:
    {
        tl_itf = H4TL_CHANNEL_DATA;
        tx_queue = H4TL_TX_QUEUE_MSG;
    }
    break;
#endif // (AHI_TL_SUPPORT)

#if (HCI_TL_SUPPORT)
#if (BLE_ISOOHCI)
    case HCI_ISO_MSG_TYPE:
    {
        tl_itf = H4TL_CHANNEL_DATA;
        tx_queue = H4TL_TX_QUEUE_ISO;
    }
    break;
#endif //(BLE_ISOOHCI)
    case HCI_CMD_MSG_TYPE:
    case HCI_ACL_MSG_TYPE:
    case HCI_SYNC_MSG_TYPE:
    case HCI_EVT_MSG_TYPE:
    {
        tl_itf = H4TL_CHANNEL_DATA;
        tx_queue = H4TL_TX_QUEUE_MSG;
    }
    break;
#endif // (HCI_TL_SUPPORT)

#if (TRACER_PRESENT)
    case TRC_MSG_TYPE:
    {
        tl_itf = H4TL_CHANNEL_TRC;
        tx_queue = H4TL_TX_QUEUE_TRACER;
    }
    break;
#endif /*(TRACER_PRESENT)*/

    default:
    {
        ASSERT_INFO_FORCE(0, type, tl_itf);
    }
    break;
    }

    // Sanity check
    if ((tx_queue != H4TL_TX_QUEUE_IDLE) && (tl_itf < H4TL_NB_CHANNEL))
    {
        // fill queued message info
        h4tl_env_t *p_env = &(h4tl_env[tl_itf]);
        h4tl_tx_info_t *tx_info = &(p_env->tx_queue[tx_queue]);
        tx_info->buf      = buf;
        tx_info->buf_len  = len;
        tx_info->callback = tx_callback;

        //send only if there is no pending messages
        if (p_env->tx_active_queue == H4TL_TX_QUEUE_IDLE)
        {
            p_env->tx_active_queue = tx_queue;
            p_env->ext_if->write(buf, len + HCI_TRANSPORT_HDR_LEN, (rwip_eif_callback)&h4tl_tx_done_isr_handler, p_env);

            // An HCI transmission is ongoing - The bit has to be set prior to call to write
            // as this function may call h4tl_tx_done immediately
            rwip_prevent_sleep_set(RW_TL_1_TX_ONGOING << p_env->tl_itf);
        }
    }
    else
    {
        ASSERT_INFO_FORCE(0, type, tl_itf);
    }
}

void h4tl_start(void)
{
    uint8_t i;

    for (i = 0 ; i < H4TL_NB_CHANNEL ; i++)
    {
        // Enable External Interface flow
        h4tl_env[i].ext_if->flow_on();
    }
}

bool h4tl_stop(void)
{
    bool res = true;
    uint8_t i;
    for (i = 0 ; (i < H4TL_NB_CHANNEL) && (res); i++)
    {
        // Disable External Interface flow
        res = h4tl_env[i].ext_if->flow_off();
    }

    // If Flow cannot be stopped on all interfaces, restart flow on interfaces stopped
    if (!res)
    {
        uint8_t j;
        for (j = 0 ; (j < i) ; j++)
        {
            // Enable External Interface flow
            h4tl_env[j].ext_if->flow_on();
        }
    }

    return (res);
}

#endif //(H4TL_SUPPORT)

/// @} H4TL
