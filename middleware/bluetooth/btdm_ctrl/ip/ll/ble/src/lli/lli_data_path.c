/**
 ****************************************************************************************
 *
 * @file lli_data_path.c
 *
 * @brief Definition of the functions used by the link layer manager
 *
 * Copyright (C) RivieraWaves 2009-2016
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

#include "rwip_config.h"    // Stack configuration
#include "em_map_ble.h"
#if (BLE_ISO_PRESENT)
#include "ke_task.h"        // For KE_MSG_HANDLER_NO_STATIC macro definition
#include "co_bt.h"          // HCI Definitions
#include "hci.h"            // HCI Interface
#include "lli_int.h"        // Isochronous internals
#include "data_path.h"      // Data path Manager
#include "co_math.h"        // for CO_BIT macro usage

#if BLE_ISOOHCI
    #include "isoohci.h"        // ISOOHCI Definitions
#endif // BLE_ISOOHCI


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */


/*
 * HCI COMMAND HANDLERS
 ****************************************************************************************
 */

#if (HCI_TL_SUPPORT && (BLE_CIS || (BLE_BIS && BLE_BROADCASTER)))
int hci_le_rd_iso_tx_sync_cmd_handler(struct hci_basic_conhdl_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    // Send the command complete event
    struct hci_le_rd_iso_tx_sync_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_iso_tx_sync_cmd_cmp_evt);
    p_evt->conhdl = param->conhdl;
    p_evt->pkt_seq_nb   = 0;
    p_evt->time_stamp   = 0;
    p_evt->time_offset  = 0;

#if BLE_ISOOHCI
    do
    {
        // Channel handle
        uint8_t act_id = BLE_ISOHDL_TO_ACTID(param->conhdl);

        // Check if ISO channel exists
        if (act_id >= BLE_ACTIVITY_MAX)
            break;

        // Get timing information from ISOoHCI Input data path
        status = isoohci_in_tx_sync_get(act_id, &p_evt->pkt_seq_nb, &p_evt->time_stamp, &p_evt->time_offset);

        // If the Host issues this command on an existing connection handle for a CIS or BIS that is not configured for transmitting SDUs
        if (status != CO_ERROR_NO_ERROR)
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // If the Host issues this command before an SDU has been transmitted by the Controller
        if ((p_evt->pkt_seq_nb == 0) && (p_evt->time_stamp == 0))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // When the Connection_Handle identifies a CIS or BIS that is transmitting unframed PDUs, the value of Time_Offset returned shall be zero
        if (lli_env.framing[act_id] == ISO_UNFRAMED_MODE)
        {
            p_evt->time_offset = 0;
        }

    }
    while (0);
#endif // BLE_ISOOHCI

    p_evt->status = status;
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}
#endif // (HCI_TL_SUPPORT && (BLE_CIS || (BLE_BIS && BLE_BROADCASTER)))

#if (BLE_CIS || BLE_BIS)
#if (HCI_TL_SUPPORT)
int hci_le_rd_buf_size_v2_cmd_handler(void const *param, uint16_t opcode)
{
    // Send the command complete event
    struct hci_le_rd_buf_size_v2_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_buf_size_v2_cmd_cmp_evt);
    p_evt->status                 = CO_ERROR_NO_ERROR;
    p_evt->le_acl_data_packet_length     = BLE_MAX_OCTETS;
    p_evt->total_num_le_acl_data_packets = EM_NUM_BLE_ACLTXBUF;//BLE_ACL_BUF_NB_TX;
#if BLE_ISOOHCI
    p_evt->iso_data_packet_length        = BLE_HCI_ISO_IN_SDU_BUF_SIZE;
    p_evt->total_num_iso_data_packets    = BLE_HCI_ISO_IN_SDU_BUF_NB;
#else // BLE_ISOOHCI
    p_evt->iso_data_packet_length        = 0;
    p_evt->total_num_iso_data_packets    = 0;
#endif // BLE_ISOOHCI
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}
#endif // (HCI_TL_SUPPORT)

int hci_le_setup_iso_data_path_cmd_handler(struct hci_le_setup_iso_data_path_cmd const *param, uint16_t opcode)
{
    // Allocate the complete event to be sent to the host
    struct hci_le_setup_iso_data_path_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_setup_iso_data_path_cmd_cmp_evt);
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    do
    {
        uint8_t direction;

        // Check and convert data path direction
        if (param->data_path_direction == ISO_DP_INPUT)
        {
            direction = ISO_SEL_TX;
        }
        else if (param->data_path_direction == ISO_DP_OUTPUT)
        {
            direction = ISO_SEL_RX;
        }
        else
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

#if (BLE_BIS)
        // If stream is a BIS
        if (param->conhdl >= BLE_BISHDL_MIN)
        {
            uint8_t act_id = BLE_BISHDL_TO_ACTID(param->conhdl);

            if (act_id < BLE_ACTIVITY_MAX)
            {
                status = lli_bis_data_path_set(act_id, direction, param->data_path_id);
            }
            break;
        }
#endif // (BLE_BIS)

#if (BLE_CIS)
        // If stream is a CIS
        if (param->conhdl >= BLE_CISHDL_MIN)
        {
            uint8_t act_id = BLE_CISHDL_TO_ACTID(param->conhdl);

            if (act_id < BLE_ACTIVITY_MAX)
            {
                status = lli_cis_data_path_set(act_id, direction, param->data_path_id);
            }
            break;
        }
#endif // (BLE_CIS)

    }
    while (0);

    // Set status and send the complete event
    p_evt->status = status;
    p_evt->conhdl = param->conhdl;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

int hci_le_remove_iso_data_path_cmd_handler(struct hci_le_remove_iso_data_path_cmd const *param, uint16_t opcode)
{
    // Allocate the status event to be sent to the host
    struct hci_le_remove_iso_data_path_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_remove_iso_data_path_cmd_cmp_evt);
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    do
    {
#if (BLE_BIS)
        // If stream is a BIS
        if (param->conhdl >= BLE_BISHDL_MIN)
        {
            uint8_t act_id = BLE_BISHDL_TO_ACTID(param->conhdl);

            if (act_id < BLE_ACTIVITY_MAX)
            {
                status = CO_ERROR_NO_ERROR;

                if (param->data_path_direction & CO_BIT(ISO_SEL_RX))
                {
                    // Remove RX data path on BIS channel
                    status = lli_bis_data_path_remove(act_id, ISO_SEL_RX);
                }

                if ((status == CO_ERROR_NO_ERROR) && (param->data_path_direction & CO_BIT(ISO_SEL_TX)))
                {
                    // Remove TX data path on BIS channel
                    status = lli_bis_data_path_remove(act_id, ISO_SEL_TX);
                }
            }
            break;
        }
#endif // (BLE_BIS)

#if (BLE_CIS)
        // If stream is a CIS
        if (param->conhdl >= BLE_CISHDL_MIN)
        {
            uint8_t act_id = BLE_CISHDL_TO_ACTID(param->conhdl);

            if (act_id < BLE_ACTIVITY_MAX)
            {
                status = CO_ERROR_NO_ERROR;

                if (param->data_path_direction & CO_BIT(ISO_SEL_RX))
                {
                    // Remove RX data path on CIS
                    status = lli_cis_data_path_remove(act_id, ISO_SEL_RX);
                }

                if ((status == CO_ERROR_NO_ERROR) && (param->data_path_direction & CO_BIT(ISO_SEL_TX)))
                {
                    // Remove TX data path on CIS
                    status = lli_cis_data_path_remove(act_id, ISO_SEL_TX);
                }
            }
            break;
        }
#endif // (BLE_CIS)

    }
    while (0);

    // Set status and send the complete event
    p_evt->status = status;
    p_evt->conhdl = param->conhdl;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}
#endif // (BLE_CIS || BLE_BIS)

#endif // (BLE_ISO_PRESENT)
/// @} LLM
