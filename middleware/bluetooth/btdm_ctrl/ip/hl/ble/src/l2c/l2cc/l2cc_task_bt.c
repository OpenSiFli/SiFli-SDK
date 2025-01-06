/**
 ****************************************************************************************
 *
 * @file l2cc_task_bt.c
 *
 * @brief L2CAP Controller Task implementation for gatt over bredr.
 *
 * Copyright (C) RivieraWaves 2009-2023
 *
 *
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "bts2_lib_config.h"

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_L2CC)


#if BLE_EMB_PRESENT
    #include "ble_util_buf.h"     // stack buffering
    #include "em_map.h"
#endif //#if BLE_EMB_PRESENT
#include "string.h"
#include "l2cc_task.h"
#include "l2cc_sig.h"
#include "l2cc_lecb.h"
#include "l2cc_int.h"
#include "l2cc_pdu_int.h"
#include "l2cc_task_bt.h"

#include "../l2cm/l2cm_int.h" // Internal API required

#include "gap.h"
#include "gapm.h"
#include "gapc.h"
#include "gattc.h"
#include "gattm.h"

#include "hci.h"

#include "co_math.h"
#include "co_utils.h"

#include "ke_mem.h"
#include "ke_event.h"

#include "dbg.h"

l2cc_task_bt_env_t g_gatt_over_bredr_env;

static l2cc_task_bt_env_t *l2cc_task_bt_get_env(void)
{
    return &g_gatt_over_bredr_env;
}

static const char *const l2cc_signaling_pkt_format[L2C_CODE_SIGNALING_MAX] =
{
    // Reserved code
    [L2C_CODE_RESERVED]                = NULL,
    // Reject request
    // [ Pkt_id | Len | reason | opt1 | opt2 ]
    [L2C_CODE_REJECT]                  = "BWWlW",
    // Connection request
    [L2C_CODE_CONNECTION_REQ]          = NULL,
    // Connection response
    [L2C_CODE_CONNECTION_RESP]         = NULL,
    // Configuration request
    [L2C_CODE_CONFIGURATION_REQ]       = NULL,
    // Configuration response
    [L2C_CODE_CONFIGURATION_RESP]      = NULL,
    // Disconnection request
    // [ Pkt_id | Len | Destination CID | Source CID ]
    [L2C_CODE_DISCONNECTION_REQ]       = "BLWW",
    // Disconnection response
    // [ Pkt_id | Len | Destination CID | Source CID ]
    [L2C_CODE_DISCONNECTION_RESP]      = "BLWW",
    // Echo request
    [L2C_CODE_ECHO_REQ]                = NULL,
    // Echo response
    [L2C_CODE_ECHO_RESP]               = NULL,
    // Information request
    [L2C_CODE_INFORMATION_REQ]         = NULL,
    // Information response
    [L2C_CODE_INFORMATION_RESP]        = NULL,
    // Create channel request
    [L2C_CODE_CREATE_CHANNEL_REQ]      = NULL,
    // Create channel response
    [L2C_CODE_CREATE_CHANNEL_RESP]     = NULL,
    // Move channel request
    [L2C_CODE_MOVE_CHANNEL_REQ]        = NULL,
    // Move channel response
    [L2C_CODE_MOVE_CHANNEL_RESP]       = NULL,
    // Move channel confirmation
    [L2C_CODE_MOVE_CHANNEL_CFM]        = NULL,
    // Move channel confirmation response
    [L2C_CODE_MOVE_CHANNEL_CFM_RESP]   = NULL,
    // Connection Parameter Update Request
    // [ Pkt_id | Len | ITV Min | ITV Max | Latency | Timeout ]
    [L2C_CODE_CONN_PARAM_UPD_REQ]      = "BLWWWW",
    // Connection Parameter Update Request
    // [ Pkt_id | Len | reason ]
    [L2C_CODE_CONN_PARAM_UPD_RESP]     = "BLW",
    // LE Credit Based Connection request
    // [ Pkt_id | Len | LE_PSM | Source CID | MTU | MPS | Initial Credits ]
    [L2C_CODE_LE_CB_CONN_REQ]          = "BLWWWWW",
    // LE Credit Based Connection response
    // [ Pkt_id | Len | Destination CID | MTU | MPS | Initial Credits | Result ]
    [L2C_CODE_LE_CB_CONN_RESP]         = "BLWWWWW",
    // LE Flow Control Credit
    // [ Pkt_id | Len | Destination CID | Credits ]
    [L2C_CODE_LE_FLOW_CONTROL_CREDIT]  = "BLWW",
};

/// Security packet format
static const char *const l2cc_security_pkt_format[L2C_CODE_SECURITY_MAX] =
{
    // Reserved code
    [L2C_CODE_RESERVED]                     = NULL,
    // Pairing Request
    // [iocap | oob | auth | max Key | IKeyX | RKeyX]
    [L2C_CODE_PAIRING_REQ]                  = "BBBBBB",
    // Pairing Response
    // [iocap | oob | auth | max Key | IKeyX | RKeyX]
    [L2C_CODE_PAIRING_RESP]                 = "BBBBBB",
    // Pairing Confirm
    // [confirm]
    [L2C_CODE_PAIRING_CFM]                  = "K",
    // Pairing Random
    // [random]
    [L2C_CODE_PAIRING_RANDOM]               = "K",
    // Pairing Failed
    // [reason]
    [L2C_CODE_PAIRING_FAILED]               = "B",
    // Encryption Information
    // [LTK]
    [L2C_CODE_ENCRYPTION_INF]               = "K",
    // Master Identification
    // [ediv | rand]
    [L2C_CODE_MASTER_ID]                    = "WR",
    // Identity Information
    // [IRK]
    [L2C_CODE_IDENTITY_INF]                 = "K",
    // Identity Address Information
    // [Addr_type | Addr]
    [L2C_CODE_ID_ADDR_INF]                  = "BA",
    // Signing Information
    // [CSRK]
    [L2C_CODE_SIGNING_INF]                  = "K",
    // Security Request
    // [auth]
    [L2C_CODE_SECURITY_REQ]                 = "B",

#if (SECURE_CONNECTIONS)
    // Pairing Public Key
    // [public key X | public key Y]
    [L2C_CODE_PUBLIC_KEY]                   = "PP",

    // DHkey Check
    // [DHkey]
    [L2C_CODE_DHKEY_CHECK]                  = "K",

    // Key Press Notification
    // [Notification Type]
    [L2C_CODE_KEYPRESS_NOTIFICATION]        = "B"
#endif // (SECURE_CONNECTIONS)
};



/// attribute protocol PDU packets format
static const char *const l2cc_attribute_pkt_format[L2C_CODE_ATT_MAX] =
{
    // Reserved code
    [L2C_CODE_RESERVED]                     = NULL,
    // Error response
    // [op_code | Handle | Err_Code]
    [L2C_CODE_ATT_ERR_RSP]                  = "BWB",
    // Exchange MTU Request
    // [MTU]
    [L2C_CODE_ATT_MTU_REQ]                  = "W",
    // Exchange MTU Response
    // [MTU]
    [L2C_CODE_ATT_MTU_RSP]                  = "W",
    // Find Information Request
    // [sHdl | eHdl]
    [L2C_CODE_ATT_FIND_INFO_REQ]            = "WW",
    // Find Information Response
    // [Format | Uuid]
    [L2C_CODE_ATT_FIND_INFO_RSP]            = "BlB",
    // Find By Type Value Request
    // [sHdl | eHdl | AttType | AttVal]
    [L2C_CODE_ATT_FIND_BY_TYPE_REQ]         = "WWWlB",
    // Find By Type Value Response
    // [InfoList]
    [L2C_CODE_ATT_FIND_BY_TYPE_RSP]         = "lB",
    // Read By Type Request
    // [sHdl | eHdl | UUID]
    [L2C_CODE_ATT_RD_BY_TYPE_REQ]           = "WWlB",
    // Read By Type Response
    // [number | data]
    [L2C_CODE_ATT_RD_BY_TYPE_RSP]           = "BlB",
    // Read Request
    // [Handle]
    [L2C_CODE_ATT_RD_REQ]                   = "W",
    // Read Response
    // [value]
    [L2C_CODE_ATT_RD_RSP]                   = "lB",
    // Read Blob Request
    // [Handle | Offset]
    [L2C_CODE_ATT_RD_BLOB_REQ]              = "WW",
    // Read Blob Response
    // [value]
    [L2C_CODE_ATT_RD_BLOB_RSP]              = "lB",
    // Read Multiple Request
    // [Handles]
    [L2C_CODE_ATT_RD_MULT_REQ]              = "lW",
    // Read Multiple Response
    // [value]
    [L2C_CODE_ATT_RD_MULT_RSP]              = "lB",
    // Read by Group Type Request
    // [sHdl | eHdl | UUID]
    [L2C_CODE_ATT_RD_BY_GRP_TYPE_REQ]       = "WWlB",
    // Read By Group Type Response
    // [number | data]
    [L2C_CODE_ATT_RD_BY_GRP_TYPE_RSP]       = "BlB",
    // Write Request
    // [Handle | Value]
    [L2C_CODE_ATT_WR_REQ]                   = "WlB",
    // Write Response
    // []
    [L2C_CODE_ATT_WR_RSP]                   = "",
    // Write Command
    // [Handle | Value]
    [L2C_CODE_ATT_WR_CMD_INFO]              = "WlB",
    // Signed Write Command
    // [Handle | Value]
    [L2C_CODE_ATT_SIGN_WR_CMD_INFO]         = "WlB",
    // Prepare Write Request
    // [Handle | Offset | Value]
    [L2C_CODE_ATT_PREP_WR_REQ]              = "WWlB",
    // Prepare Write Response
    // [Handle | Offset | Value]
    [L2C_CODE_ATT_PREP_WR_RSP]              = "WWlB",
    // Execute Write Request
    // [Flags]
    [L2C_CODE_ATT_EXE_WR_REQ]               = "B",
    // Execute Write Response
    // []
    [L2C_CODE_ATT_EXE_WR_RSP]               = "",
    // Handle Value Notification
    // [Handle | Value]
    [L2C_CODE_ATT_HDL_VAL_NTF]              = "WlB",
    // Handle Value Indication
    // [Handle | Value]
    [L2C_CODE_ATT_HDL_VAL_IND]              = "WlB",
    // Handle Value Confirmation
    // []
    [L2C_CODE_ATT_HDL_VAL_CFM]              = "",
};

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

uint8_t l2cc_pdu_unpack_bt(uint8_t conidx, struct l2cc_pdu *p_pdu, uint16_t *p_offset, uint16_t *p_rem_len,
                           uint8_t *p_buffer, uint16_t pkt_length, uint8_t pb_flag)
{
    // Status of unpacking
    uint8_t status = GAP_ERR_NO_ERROR;

    uint8_t rx_exp_len = L2C_HEADER_LEN;
    rx_exp_len += co_min(*p_rem_len,  L2C_MIN_LE_MTUSIG);

    if (pb_flag == PBF_1ST_HL_FRAG)
    {
        uint8_t code = p_pdu->data.code;
        uint8_t total_length = 0;

        // Packet descriptor information
        uint8_t **pkt_desc_array = NULL;
        uint8_t pkt_desc_array_size = L2C_CODE_RESERVED;
        uint8_t *pkt_desc = NULL;

        uint8_t *input = p_buffer + L2C_CODE_LEN;

        // Pointer to the data to pack
        uint8_t *p_data_pack;
        // packet can be divided in multiple fragment
        bool mult_fragment = false;

        do
        {
            // Extract channel identifier
            switch (p_pdu->chan_id)
            {
            // signaling packet
            case (L2C_CID_LE_SIGNALING):
            {
                // identifier 0 is invalid
                if (*(input) != 0)
                {
                    pkt_desc_array = (uint8_t **)l2cc_signaling_pkt_format;
                    pkt_desc_array_size = L2C_CODE_SIGNALING_MAX;
                }
            }
            break;

            case (L2C_CID_SECURITY):
            {
                pkt_desc_array = (uint8_t **)l2cc_security_pkt_format;
                pkt_desc_array_size = L2C_CODE_SECURITY_MAX;
            }
            break;

            case (L2C_CID_ATTRIBUTE):
            {
                // Update code to an existing index in packet definition array
                if (code == L2C_CODE_ATT_WR_CMD)
                {
                    code = L2C_CODE_ATT_WR_CMD_INFO;
                }
                else if (code == L2C_CODE_ATT_SIGN_WR_CMD)
                {
                    code = L2C_CODE_ATT_SIGN_WR_CMD_INFO;
                }

                pkt_desc_array = (uint8_t **)l2cc_attribute_pkt_format;
                pkt_desc_array_size = L2C_CODE_ATT_MAX;
            }
            break;

#if (BLE_ISO_MODE_0_PROTOCOL)
            // Audio Mode 0 L2CAP Protocol
            case (AM0_L2C_CID_AUDIO_MODE_0):
            {
                if (gapm_is_audio_am0_sup())
                {
                    pkt_desc_array = am0_pdu_pkt_format_get(&pkt_desc_array_size);

                    break;
                }
            } // no break
#endif // (BLE_ISO_MODE_0_PROTOCOL)

            default:
            {
                // Shall not happen
                ASSERT_INFO(0, p_pdu->chan_id, p_pdu->payld_len);

                // Error code, Invalid CID.
                status = L2C_ERR_INVALID_CID;
            }
            break;
            }

            // Prepare destination pointer
            p_data_pack   = &(p_pdu->data.code) + L2C_CODE_LEN;
            total_length += L2C_CODE_LEN;

            // Error occurs stop parsing
            if (status != GAP_ERR_NO_ERROR)
            {
                break;
            }

            // check if code is valid or not
            if ((pkt_desc_array == NULL) || (code >= pkt_desc_array_size))
            {
                // error packet code invalid
                status = L2C_ERR_UNKNOWN_PDU;
                break;
            }

            pkt_desc = pkt_desc_array[code];

            // check if it's a known packet type
            if (pkt_desc == NULL)
            {
                // error packet cannot be extracted, not supported
                status = L2C_ERR_UNKNOWN_PDU;
                break;
            }

            // parse packet description
            while ((*pkt_desc != '\0') && (status == GAP_ERR_NO_ERROR) && (total_length <= *p_rem_len))
            {
                // check character by character
                switch (*pkt_desc)
                {
                case ('B'): // Byte
                {
                    *p_data_pack = *input;

                    input += 1;
                    p_data_pack  += 1;
                    total_length += 1;
                }
                break;

                case ('L'): // Word Length
                case ('W'): // Word
                {
                    // Align data buffer to a 16 bit
                    uint16_t *word = ((uint16_t *)((((uint32_t)p_data_pack) + 1) & ~1));

                    p_data_pack = (uint8_t *)(word + 1);

                    *word = co_read16p(input);
                    input += 2;
                    total_length += 2;
                }
                break;

                case ('K'):
                {
                    // Unpack the key value
                    memcpy(p_data_pack, input, GAP_KEY_LEN);

                    p_data_pack += GAP_KEY_LEN;
                    input += GAP_KEY_LEN;
                    total_length += GAP_KEY_LEN;
                }
                break;
#if (SECURE_CONNECTIONS)
                case ('P'):
                {
                    // Remaining data length in the received packet
                    uint8_t len = co_min(((rx_exp_len - L2C_HEADER_LEN) - total_length), GAP_P256_KEY_LEN);

                    // Unpack the key value
                    memcpy(p_data_pack, input, len);

                    if (len != GAP_P256_KEY_LEN)
                    {
                        mult_fragment = true;
                    }

                    p_data_pack += len;
                    input += len;
                    total_length += len;

                }
                break;
#endif //  (SECURE_CONNECTIONS)

                case ('R'):
                {
                    // Unpack the rand value
                    memcpy(p_data_pack, input, GAP_RAND_NB_LEN);

                    p_data_pack += GAP_RAND_NB_LEN;
                    input += GAP_RAND_NB_LEN;
                    total_length += GAP_RAND_NB_LEN;
                }
                break;

                case ('A'):
                {
                    // Unpack the bd address value
                    memcpy(p_data_pack, input, BD_ADDR_LEN);

                    p_data_pack += BD_ADDR_LEN;
                    input += BD_ADDR_LEN;
                    total_length += BD_ADDR_LEN;
                }
                break;

                // Array of byte or word to unpack
                case ('l'):
                {
                    // Remaining data length in the received packet
                    uint8_t len = (rx_exp_len - L2C_HEADER_LEN) - total_length;
                    uint16_t length = p_pdu->payld_len - total_length;
                    mult_fragment = true;

                    pkt_desc++;
                    // Align data pointer to a 16 bits
                    p_data_pack = (uint8_t *)((((uint32_t)p_data_pack) + 1) & ~1);

                    // Check array type
                    if (*pkt_desc == 'W')
                    {
                        // Divide number of data in the array by 2
                        length /= 2;
                    }
                    else if (*pkt_desc != 'B')
                    {
                        // Error, invalid PDU
                        status = L2C_ERR_INVALID_PDU;
                        break;
                    }

                    // Number of data in the array = Total remaining payload size
                    co_write16p(p_data_pack, length);

                    // Update pointer of data value - uint16_t
                    p_data_pack += 2;

                    // Retrieve data
                    memcpy(p_data_pack, input, len);

                    total_length += len;
                    p_pdu->payld_len += 2;
                    p_data_pack  += len;
                }
                break;

                default:
                {
                    // Packet description error
                    status = L2C_ERR_INVALID_PDU;
                }
                break;
                }

                pkt_desc++;
            }

            // check that received PDU length is not less than expected PDU
            if (((uint16_t)total_length > *p_rem_len)
                    // Packet not fully extracted
                    || (*pkt_desc != '\0')
                    // Or received data is bigger than expected (does not contains an array of byte)
                    || (!mult_fragment && ((uint16_t)total_length != *p_rem_len)))
            {
                // check if SIG MTU exceed
                if ((p_pdu->chan_id == L2C_CID_LE_SIGNALING) && (p_pdu->payld_len > L2C_MIN_LE_MTUSIG))
                {
                    status = L2C_ERR_INVALID_MTU_EXCEED;
                }
                else
                {
                    status = L2C_ERR_INVALID_PDU;
                }
            }
            else
            {
                // Update the remaining length
                *p_rem_len -= (uint16_t)total_length;
                // Update the offset value
                *p_offset += (uint16_t)(p_data_pack - & (p_pdu->data.code));
            }
        }
        while (0);

        if (status != GAP_ERR_NO_ERROR)
        {
            switch (p_pdu->chan_id)
            {
            case (L2C_CID_ATTRIBUTE):
            case (L2C_CID_SECURITY):
            {
                // Nothing to do
            } break;

            // Signaling packet
            default:
            {
                // Retrieve packet id
                p_pdu->data.reject.pkt_id = p_buffer[L2C_HEADER_LEN + L2C_CODE_LEN];
            }
            break;
            }
        }
    }

    // We have valid data in received buffer (and not temporary buffer)
    if ((pkt_length > 0) && (status == GAP_ERR_NO_ERROR))
    {
        if (pkt_length <= *p_rem_len)
        {
            // Retrieve data
            memcpy(&p_pdu->data.code + *p_offset, p_buffer, pkt_length);

            *p_offset += pkt_length;
            // Update the remaining length
            *p_rem_len -= pkt_length;
        }
        else
        {
            // Length of the PDU is not valid
            status = L2C_ERR_INVALID_PDU;
        }
    }

    return (status);
}

#ifdef CFG_BR_GATT_SRV
void bt_gatt_over_bredr_init()
{
    l2cc_task_bt_env_t *env = l2cc_task_bt_get_env();
    memset(env, 0, sizeof(l2cc_task_bt_env_t));

    env->is_init = 1;
    env->conidx = GATT_OVER_BREDR_INDEX;

    // Inform L2CAP about new connection
    l2cm_create(env->conidx);
    // Inform GATT about new connection
    gattm_create(env->conidx);
    gattc_con_enable(env->conidx);

    extern uint8_t gapc_con_create_bt(struct hci_le_enh_con_cmp_evt const * con_params,
                                      ke_task_id_t requester, bd_addr_t *laddr, uint8_t laddr_type);

    struct hci_le_enh_con_cmp_evt const con_params;
    bd_addr_t laddr;
    gapc_con_create_bt(&con_params, 0, &laddr, 0);
}


void bt_gatt_over_bredr_release()
{
    l2cc_task_bt_env_t *env = l2cc_task_bt_get_env();
    env->is_init = 0;
    l2cm_cleanup(env->conidx);
    gattm_cleanup(env->conidx);

    extern uint8_t gapc_con_cleanup(uint8_t conidx);
    gapc_con_cleanup(env->conidx);
}

uint8_t is_gatt_over_bredr_conidx(uint8_t conidx)
{
    l2cc_task_bt_env_t *env = l2cc_task_bt_get_env();
    if (!env->is_init)
    {
        return 0;
    }
    if (env->conidx == conidx)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void bt_att_data_send(void const *param_ptr)
{
    l2cc_task_bt_env_t *env = l2cc_task_bt_get_env();
    // origin process l2cc_data_send
    uint8_t *buffer;
    uint8_t  status = GAP_ERR_NO_ERROR;
    buffer = malloc(l2cm_get_buffer_size() + HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN);
    // move pointer to payload beginning
    buffer += HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN;

    struct ke_msg *msg = ke_param2msg(param_ptr);

    //rt_kprintf("bt_att_data_send id %d\n", msg->id);
    struct l2cc_pdu_send_cmd *pdu_cmd = (struct l2cc_pdu_send_cmd *) ke_msg2param(msg);
    struct l2cc_pdu *pdu = (struct l2cc_pdu *) & (pdu_cmd->pdu);
    uint8_t  pb_flag = PBF_1ST_NF_HL_FRAG;

    uint16_t tx_length = 0;

    status = l2cc_pdu_pack(pdu, &(pdu_cmd->offset), &tx_length, buffer, &pb_flag);

    if ((status != GAP_ERR_NO_ERROR) || (pdu->payld_len == 0))
    {
        // send command completed event
        rt_kprintf("l2cc_send_cmp_evt\n");
        l2cc_send_cmp_evt(env->conidx, L2CC_PDU_SEND, msg->src_id, status, pdu->chan_id, 0);
        ke_msg_free(msg);
    }
    else
    {
        ke_msg_free(msg);
    }

    uint16_t data_len = tx_length - 4;
    uint8_t *data = buffer + 4;

    //rt_kprintf("data %d, len %d\n", *data, data_len);

    extern void bt_gatt_send_data_req(char *payload, U16 payload_len);
    bt_gatt_send_data_req((char *)data, data_len);

    free(buffer - (HCI_ACL_HDR_LEN + HCI_TRANSPORT_HDR_LEN));
}

int bt_att_data_handler(uint16_t len, uint8_t *data, uint16_t cid)
{
    l2cc_task_bt_env_t *env = l2cc_task_bt_get_env();

    if (!env->is_init)
    {
        return (KE_MSG_CONSUMED);
    }

    uint8_t conidx = env->conidx;
    env->cid = cid;

    uint16_t remain_len = len;
    uint8_t pb_flag = 2;
    uint8_t code = data[0];
    uint8_t status = 0;

    uint8_t *buffer = NULL;
    buffer = data;
    ASSERT_ERR(buffer != NULL);
    struct ke_msg *l2cc_msg;
    cid = 0x04;


    ke_task_id_t l2cc_dest_id;

    l2cc_dest_id = KE_BUILD_ID(TASK_GAPC, conidx);
    switch (cid)
    {
    case (L2C_CID_LE_SIGNALING):
    {
        l2cc_dest_id  = KE_BUILD_ID(TASK_L2CC, conidx);
    }
    break;

    case (L2C_CID_SECURITY):
    {
        l2cc_dest_id = KE_BUILD_ID(TASK_GAPC, conidx);
    }
    break;

    case (L2C_CID_ATTRIBUTE):
    {
        l2cc_dest_id = KE_BUILD_ID(TASK_GATTC, conidx);
    }
    break;
    }

    struct l2cc_pdu_recv_ind *pdu;
    pdu = KE_MSG_ALLOC_DYN(L2CC_PDU_RECV_IND, l2cc_dest_id, KE_BUILD_ID(TASK_L2CC, conidx),
                           l2cc_pdu_recv_ind, len);

    pdu->offset         = 0;
    pdu->status         = status;
    pdu->pdu.chan_id    = cid;
    pdu->pdu.payld_len  = len;
    pdu->pdu.data.code  = code;

    l2cc_msg      = ke_param2msg(pdu);

    struct l2cc_pdu_recv_ind *pdu_ind =
        (struct l2cc_pdu_recv_ind *) ke_msg2param(l2cc_msg);

    // process reception of message
    status = l2cc_pdu_unpack_bt(conidx, &(pdu_ind->pdu), &(pdu_ind->offset), &remain_len,
                                buffer, 0, pb_flag);

    l2cc_msg = NULL;
    pdu_ind->status = status;
    ke_msg_send(pdu_ind);

    return (KE_MSG_CONSUMED);
}
#endif

#endif //(BLE_L2CC)

/// @} L2CCTASKBT
