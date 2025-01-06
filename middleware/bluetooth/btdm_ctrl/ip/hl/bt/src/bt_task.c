/**
 ****************************************************************************************
 *
 * @file ahi.c
 *
 * @brief This file contains definitions related to the Application Host Interface
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup AHI
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (CFG_BT_HOST)

#include "ke_event.h"
#include "ke_task.h"
#include "ke_mem.h"
#include "co_bt.h"           // BT standard definitions
#include "co_list.h"
#include "co_utils.h"
#include "h4tl.h"            // H4 Transport Layer
#include "hci.h"

#include "bts2_global_int.h"
#include "hci_drv.h"
#include "hci_spec.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * STATES
 ****************************************************************************************
 */
/// Possible states of the BT task
enum BT_STATE
{
    /// TX IDLE state
    BT_TX_IDLE,
    /// TX ONGOING state
    BT_TX_ONGOING,
    /// Number of states.
    BT_TASK_STATE_MAX
};


/// Data structure that describe packet to send
struct bt_tx_data
{
    /// list element
    struct co_list_hdr hdr;
    /// Message data pointer to send
    uint8_t           *data;
    /// TX callback to be call when TX is over
    void (*tx_callback)(uint8_t *);
    /// Data length to send.
    uint16_t           len;
    /// Message type
    uint8_t            msg_type;
};


///BT Environment context structure
struct bt_env_tag
{
    /// list of TX buffer in pending queue
    struct co_list      tx_queue;

    /// message wich is currently TX.
    struct bt_tx_data *tx_ptr;
};

/// Maximum number of instances of the BT task
#define BT_IDX_MAX  1

/// Defines the placeholder for the states of all the task instances.
ke_state_t bt_state[BT_IDX_MAX];

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/// Specifies the message handlers that are common to all states.
__STATIC int bt_msg_handler(ke_msg_id_t const msgid,    void *param,     ke_task_id_t const dest_id, ke_task_id_t const src_id);

KE_MSG_HANDLER_TAB(bt)
{
    { HCI_CMD_CMP_EVENT, (ke_msg_func_t) bt_msg_handler },
    { HCI_CMD_STAT_EVENT, (ke_msg_func_t) bt_msg_handler },
    { HCI_EVENT, (ke_msg_func_t) bt_msg_handler },
    { HCI_COMMAND, (ke_msg_func_t) bt_msg_handler },
    { HCI_ACL_DATA, (ke_msg_func_t) bt_msg_handler },
    { HCI_SYNC_DATA, (ke_msg_func_t) bt_msg_handler },
    { HCI_DBG_EVT, (ke_msg_func_t) bt_msg_handler },
};

const struct ke_task_desc TASK_DESC_BT = {bt_msg_handler_tab, bt_state, BT_IDX_MAX,  ARRAY_LEN(bt_msg_handler_tab)};


/// BT environment context
struct bt_env_tag bt_env;


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void bt_init(void)
{
    // Start host stack
    bts2_init(NULL);

    // Create BT Task
    ke_task_create(TASK_BT, &TASK_DESC_BT);

    // Initialize BT task to idle state
    ke_state_set(TASK_BT, BT_TX_IDLE);

    // Initialize TX queue
    co_list_init(&bt_env.tx_queue);
}

#if 0
void bt_send_msg(uint8_t msg_type, uint16_t len, uint8_t *data, void (*tx_callback)(uint8_t *))
{
    switch (msg_type)
    {
    case AHI_KE_MSG_TYPE:
    {
        struct bt_tx_data *tx_data = (struct bt_tx_data *) ke_malloc(sizeof(struct bt_tx_data), KE_MEM_KE_MSG);
        tx_data->len         = len;
        tx_data->data        = data;
        tx_data->msg_type    = msg_type;
        tx_data->tx_callback = tx_callback;

        // Check if there is no transmission ongoing
        if (ke_state_get(TASK_AHI) == AHI_TX_IDLE)
        {
            // request to send the message to H4 TL.
            ahi_h4tl_send(tx_data);
        }
        else
        {
            // put message at end of queue
            co_list_push_back(&bt_env.tx_queue, &(tx_data->hdr));
        }
    }
    break;
    // ensure that only supported messages are sent over AHI interface
    default:
    {
        ASSERT_INFO(0, msg_type, len);
    }
    break;
    }
}
#endif

/**
 ****************************************************************************************
 * @brief Function called to to handle message for BT host stack.
 *
 * @param[in]  msgid   U16 message id from ke_msg.
 * @param[in] *param   Pointer to parameters of the message in ke_msg.
 * @param[in]  dest_id Destination task id.
 * @param[in]  src_id  Source task ID.
 *
 * @return             Kernel message state, must be KE_MSG_NO_FREE.
 *****************************************************************************************
 */
__STATIC int bt_msg_handler(ke_msg_id_t const msgid,
                            void *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    //extract the ke_msg pointer from the param passed and push it in AHI queue
    struct ke_msg *msg = ke_param2msg(param);
    int r = KE_MSG_NO_FREE;

    switch (msg->id)
    {
    case HCI_EVENT:
    case HCI_CMD_CMP_EVENT:
    case HCI_CMD_STAT_EVENT:
    {
        BTS2S_RECV_DATA data;
        BTS2S_HCI_EV_COMMON *ev;
        extern void hcit_hdl_ev(BTS2S_RECV_DATA * src);

        data.buflen = msg->param_len + sizeof(BTS2S_HCI_EV_COMMON);
        data.buf = bmalloc(data.buflen);
        ev = (BTS2S_HCI_EV_COMMON *)data.buf;
        ev->ev_code = msg->src_id;
        ev->len = msg->param_len;
        memcpy((void *)(data.buf + sizeof(BTS2S_HCI_EV_COMMON)), msg->param, msg->param_len);
        hcit_hdl_ev(&data);
        r = KE_MSG_CONSUMED;
    }
    case HCI_ACL_DATA:
    {
        BTS2S_RECV_DATA data;
        extern void hcit_hdl_acl_data(BTS2S_RECV_DATA * hci_rx_msg);

        struct hci_acl_data *p_acl_data = (struct hci_acl_data *) param;
        data.buflen = p_acl_data->length + sizeof(BTS2S_HCI_ACL_DATA);
        data.buf = (char *)p_acl_data;
        hcit_hdl_acl_data(&data);
        r = KE_MSG_CONSUMED;
    }
    default:
        break;
    }
    return r;
}


uint8_t *bt_rx_hdr_handle(uint8_t *buffer, uint16_t *length)
{
    return NULL;
}

void bt_rx_done(uint8_t *buffer)
{

}

void bt_reset(void)
{

}


#endif //AHI_TL_SUPPORT

/// @} AHI
