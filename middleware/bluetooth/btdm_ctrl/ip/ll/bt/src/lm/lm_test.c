/**
 ****************************************************************************************
 *
 * @file lm_test.c
 *
 * @brief LM test mode source file
 *
 * Copyright (C) RivieraWaves 2009-2021
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LMTEST
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration
#if (BT_HCI_TEST_MODE)
#include <string.h>

#include "ke_task.h"        // kernel task definitions

#include "ld.h"             // link layer driver
#include "lm.h"
#include "lm_int.h"         // link layer manager internal definitions

#if HCI_PRESENT
    #include "hci.h"            // host controller interface
#endif //HCI_PRESENT


/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/// Check that there is no ongoing activity (scanning, advertising, initiating, or connection)
__STATIC bool lm_no_activity(void)
{
    uint8_t act_id;

    // Check all activities
    for (act_id = 0; act_id < MAX_NB_ACTIVE_ACL; act_id++)
    {
        if (lm_env.con_info[act_id].state != LM_FREE)
            break;
    }

    return (act_id >= MAX_NB_ACTIVE_ACL);
}

/// Send HCI CC event returning a status only
__STATIC void cmd_cmp_send(uint16_t opcode, uint8_t status)
{
    // allocate the complete event message
    struct hci_basic_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);
    // update the status
    evt->status = status;
    // send the message
    hci_send_2_host(evt);
}


/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

KE_MSG_HANDLER_NO_STATIC(lm_test_end_ind, struct lm_test_end_ind)
{
    // Send the complete event
    struct hci_vs_test_end_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_VS_TEST_END_CMD_OPCODE, hci_vs_test_end_cmd_cmp_evt);
    event->status = param->status;
    event->nb_packets = param->nb_pkt;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}


/*
 * HCI COMMAND HANDLERS
 ****************************************************************************************
 */

int hci_vs_rx_test_cmd_handler(struct hci_vs_rx_test_cmd const *param, uint16_t opcode)
{
    // Status returned in the command complete event
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (lm_no_activity())
    {
        if ((param->rx_channel > HOP_NB_CHANNEL)
                || (param->pkt_type > 21))

        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        else
        {
            struct ld_test_params test_params;
            test_params.type = 0;
            test_params.channel = param->rx_channel;
            test_params.pkt_type = param->pkt_type;
            status = ld_test_start(&test_params);
        }
    }

    // Send the command complete event
    cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_vs_tx_test_cmd_handler(struct hci_vs_tx_test_cmd const *param, uint16_t opcode)
{
    // Status returned in the command complete event
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (lm_no_activity())
    {
        if ((param->tx_channel > HOP_NB_CHANNEL)
                || (param->test_data_len > DH5_3_PACKET_SIZE)
                || (param->pkt_payl > PAYL_01010101)
                || (param->pkt_type > 21)
                || (((param->tx_pwr_lvl < LOW_TX_PWR_LVL) || (param->tx_pwr_lvl > HIGH_TX_PWR_LVL)) && (param->tx_pwr_lvl != MIN_TX_PWR_LVL) && (param->tx_pwr_lvl != MAX_TX_PWR_LVL)))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        else
        {
            struct ld_test_params test_params;
            test_params.type = 1;
            test_params.channel = param->tx_channel;
            test_params.data_len = param->test_data_len;
            test_params.payload = param->pkt_payl;
            test_params.pkt_type = param->pkt_type;
            test_params.tx_pwr_lvl = param->tx_pwr_lvl;
            status = ld_test_start(&test_params);
        }
    }

    // Send the command complete event
    cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_vs_test_end_cmd_handler(void const *param, uint16_t opcode)
{
    uint8_t status = ld_test_stop();

    if (status != CO_ERROR_NO_ERROR)
    {
        // Send the complete event
        struct hci_vs_test_end_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_VS_TEST_END_CMD_OPCODE, hci_vs_test_end_cmd_cmp_evt);
        event->status = status;
        event->nb_packets = 0;
        hci_send_2_host(event);
    }

    return (KE_MSG_CONSUMED);
}


#endif //(BT_HCI_TEST_MODE)
/// @} LMTEST
