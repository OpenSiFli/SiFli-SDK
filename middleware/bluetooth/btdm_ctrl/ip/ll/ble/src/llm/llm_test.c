/**
 ****************************************************************************************
 *
 * @file llm_test.c
 *
 * @brief LLM test mode source file
 *
 * Copyright (C) RivieraWaves 2009-2018
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLMTEST
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration
#if (BLE_TEST_MODE_SUPPORT)
#include <string.h>

#include "ke_task.h"        // kernel task definitions

#include "lld.h"            // link layer driver
#include "llm_int.h"        // link layer manager internal definitions

#if HCI_PRESENT
    #include "hci.h"            // host controller interface
#endif //HCI_PRESENT


/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/// Check that there is no ongoing activity (scanning, advertising, initiating, or connection)
__STATIC bool llm_no_activity(void)
{
    uint8_t act_id;

    // Check all activities
    for (act_id = 0; act_id < BLE_ACTIVITY_MAX_CFG; act_id++)
    {
        if (llm_env.act_info[act_id].state != LLM_FREE)
            break;
    }

    return (act_id >= BLE_ACTIVITY_MAX_CFG);
}


/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

KE_MSG_HANDLER_NO_STATIC(lld_test_end_ind, struct lld_test_end_ind)
{
#if (EAVESDROPPING_SUPPORT)
    if ((llm_env.cust_tx_test_params.enable != 0) && (llm_env.cust_tx_test_params.tx_packets != 0))
    {
        // allocate the status event message
        struct hci_vs_cust_tx_test_end_evt *event = KE_MSG_ALLOC(HCI_DBG_EVT, 0, 0, hci_vs_cust_tx_test_end_evt);
        event->subcode = HCI_VS_CUST_TX_TEST_END_EVT_SUBCODE;
        hci_send_2_host(event);
    }
    else
#endif // EAVESDROPPING_SUPPORT
    {
        // Send the complete event
        struct hci_test_end_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_TEST_END_CMD_OPCODE, hci_test_end_cmd_cmp_evt);
        event->status = param->status;
        event->nb_packet_received = param->nb_pkt_recv;
        hci_send_2_host(event);
    }

    return (KE_MSG_CONSUMED);
}


/*
 * HCI COMMAND HANDLERS
 ****************************************************************************************
 */

int hci_le_rx_test_v1_cmd_handler(struct hci_le_rx_test_v1_cmd const *param, uint16_t opcode)
{
    // Status returned in the command complete event
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (llm_no_activity())
    {
        if (param->rx_channel > TEST_FREQ_MAX)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        else
        {
            struct lld_test_params test_params;
            test_params.type = 0;
            test_params.channel = param->rx_channel;
            test_params.phy = PHY_1MBPS_BIT;
            test_params.cte_len = NO_CTE;
            status = lld_test_start(&test_params);
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_tx_test_v1_cmd_handler(struct hci_le_tx_test_v1_cmd const *param, uint16_t opcode)
{
    // Status returned in the command complete event
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (llm_no_activity())
    {
        if ((param->tx_channel > TEST_FREQ_MAX)
                || (param->pkt_payl > PAYL_01010101))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        else
        {
            struct lld_test_params test_params;
            test_params.type = 1;
            test_params.channel = param->tx_channel;
            test_params.data_len = param->test_data_len;
            test_params.payload = param->pkt_payl;
            test_params.phy = PHY_1MBPS_BIT;
            test_params.cte_len = NO_CTE;
            test_params.tx_pwr_lvl = MAX_TX_PWR_LVL;

#if (EAVESDROPPING_SUPPORT)
            test_params.cust_tx_test_params.enable = llm_env.cust_tx_test_params.enable;
            test_params.cust_tx_test_params.tx_packets = llm_env.cust_tx_test_params.tx_packets;
            test_params.cust_tx_test_params.tx_power = llm_env.cust_tx_test_params.tx_power;
#endif // EAVESDROPPING_SUPPORT

            status = lld_test_start(&test_params);
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_rx_test_v2_cmd_handler(struct hci_le_rx_test_v2_cmd const *param, uint16_t opcode)
{
    // Status returned in the command complete event
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (llm_no_activity())
    {
        if ((param->rx_channel > TEST_FREQ_MAX)
                || (param->phy < TEST_PHY_MIN)
                || (param->phy > RX_TEST_PHY_MAX)
                || (param->mod_idx > 0x01))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        else
        {
            struct lld_test_params test_params;
            test_params.type = 0;
            test_params.channel = param->rx_channel;
            test_params.phy = param->phy;
            test_params.cte_len = NO_CTE;
            status = lld_test_start(&test_params);
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_tx_test_v2_cmd_handler(struct hci_le_tx_test_v2_cmd const *param, uint16_t opcode)
{
    // Status returned in the command complete event
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (llm_no_activity())
    {
        if ((param->tx_channel > TEST_FREQ_MAX)
                || (param->pkt_payl > PAYL_01010101)
                || (param->phy < TEST_PHY_MIN)
                || (param->phy > TX_TEST_PHY_MAX))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        else
        {
            struct lld_test_params test_params;
            test_params.type = 1;
            test_params.channel = param->tx_channel;
            test_params.data_len = param->test_data_len;
            test_params.payload = param->pkt_payl;
            test_params.phy = param->phy;
            test_params.cte_len = NO_CTE;
            test_params.tx_pwr_lvl = MAX_TX_PWR_LVL;

#if (EAVESDROPPING_SUPPORT)
            test_params.cust_tx_test_params.enable = llm_env.cust_tx_test_params.enable;
            test_params.cust_tx_test_params.tx_packets = llm_env.cust_tx_test_params.tx_packets;
            test_params.cust_tx_test_params.tx_power = llm_env.cust_tx_test_params.tx_power;
#endif // EAVESDROPPING_SUPPORT

            status = lld_test_start(&test_params);
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_rx_test_v3_cmd_handler(struct hci_le_rx_test_v3_cmd const *param, uint16_t opcode)
{
    // Status returned in the command complete event
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (llm_no_activity())
    {
        if ((param->rx_channel > TEST_FREQ_MAX)
                || (param->phy < TEST_PHY_MIN)
                || (param->phy > RX_TEST_PHY_MAX)
                || (param->mod_idx > STABLE_MOD_IDX)
                || ((param->exp_cte_len != NO_CTE) && (param->exp_cte_len < CTE_LEN_MIN))
                || (param->exp_cte_len > CTE_LEN_MAX)
                || (param->exp_cte_type > CTE_TYPE_AOD_2US)
                || (param->slot_dur < SLOT_DUR_1US)
                || (param->slot_dur > SLOT_DUR_2US)
                || (param->switching_pattern_len < MIN_SWITCHING_PATTERN_LEN)
                || (param->switching_pattern_len > MAX_SWITCHING_PATTERN_LEN))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        // If Expected_CTE_Length is not zero and PHY specifies a PHY that does not allow Constant Tone Extensions
        else if ((param->exp_cte_len != NO_CTE) && (param->phy == RX_TEST_PHY_MAX))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
        else if ((param->switching_pattern_len > BLE_MAX_SW_PAT_LEN))
        {
            status = CO_ERROR_UNSUPPORTED;
        }
        else
        {
            struct lld_test_params test_params;
            test_params.type = 0;
            test_params.channel = param->rx_channel;
            test_params.phy = param->phy;
            test_params.cte_len = param->exp_cte_len;
            test_params.cte_type = param->exp_cte_type;
            test_params.slot_dur = param->slot_dur;
            test_params.switching_pattern_len = param->switching_pattern_len;
            memcpy(&test_params.antenna_id[0], &param->antenna_id[0], param->switching_pattern_len);
            status = lld_test_start(&test_params);
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_tx_test_v3_cmd_handler(struct hci_le_tx_test_v3_cmd const *param, uint16_t opcode)
{
    // Status returned in the command complete event
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (llm_no_activity())
    {
        if ((param->tx_channel > TEST_FREQ_MAX)
                || (param->pkt_payl > PAYL_01010101)
                || (param->phy < TEST_PHY_MIN)
                || (param->phy > TX_TEST_PHY_MAX)
                || ((param->cte_len != NO_CTE) && (param->cte_len < CTE_LEN_MIN))
                || (param->cte_len > CTE_LEN_MAX)
                || (param->cte_type > CTE_TYPE_AOD_2US)
                || (param->switching_pattern_len < MIN_SWITCHING_PATTERN_LEN)
                || (param->switching_pattern_len > MAX_SWITCHING_PATTERN_LEN))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        // If CTE_Length is not zero and PHY specifies a PHY that does not allow Constant Tone Extensions
        else if ((param->cte_len != NO_CTE) && (param->phy >= RX_TEST_PHY_MAX))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
        else if ((param->switching_pattern_len > BLE_MAX_SW_PAT_LEN))
        {
            status = CO_ERROR_UNSUPPORTED;
        }
        else
        {
            struct lld_test_params test_params;
            test_params.type = 1;
            test_params.channel = param->tx_channel;
            test_params.data_len = param->test_data_len;
            test_params.payload = param->pkt_payl;
            test_params.phy = param->phy;
            test_params.cte_len = param->cte_len;
            test_params.cte_type = param->cte_type;
            test_params.switching_pattern_len = param->switching_pattern_len;
            memcpy(&test_params.antenna_id[0], &param->antenna_id[0], param->switching_pattern_len);
            test_params.tx_pwr_lvl = MAX_TX_PWR_LVL;

#if (EAVESDROPPING_SUPPORT)
            test_params.cust_tx_test_params.enable = llm_env.cust_tx_test_params.enable;
            test_params.cust_tx_test_params.tx_packets = llm_env.cust_tx_test_params.tx_packets;
            test_params.cust_tx_test_params.tx_power = llm_env.cust_tx_test_params.tx_power;
#endif // EAVESDROPPING_SUPPORT

            status = lld_test_start(&test_params);
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_tx_test_v4_cmd_handler(struct hci_le_tx_test_v4_cmd const *param, uint16_t opcode)
{
    // Status returned in the command complete event
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (llm_no_activity())
    {
        if ((param->tx_channel > TEST_FREQ_MAX)
                || (param->pkt_payl > PAYL_01010101)
                || (param->phy < TEST_PHY_MIN)
                || (param->phy > TX_TEST_PHY_MAX)
                || ((param->cte_len != NO_CTE) && (param->cte_len < CTE_LEN_MIN))
                || (param->cte_len > CTE_LEN_MAX)
                || (param->cte_type > CTE_TYPE_AOD_2US)
                || (param->switching_pattern_len < MIN_SWITCHING_PATTERN_LEN)
                || (param->switching_pattern_len > MAX_SWITCHING_PATTERN_LEN)
                || (((param->tx_pwr_lvl < LOW_TX_PWR_LVL) || (param->tx_pwr_lvl > HIGH_TX_PWR_LVL)) && (param->tx_pwr_lvl != MIN_TX_PWR_LVL) && (param->tx_pwr_lvl != MAX_TX_PWR_LVL)))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
        }
        // If CTE_Length is not zero and PHY specifies a PHY that does not allow Constant Tone Extensions
        else if ((param->cte_len != NO_CTE) && (param->phy >= RX_TEST_PHY_MAX))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
        else if ((param->switching_pattern_len > BLE_MAX_SW_PAT_LEN))
        {
            status = CO_ERROR_UNSUPPORTED;
        }
        else
        {
            struct lld_test_params test_params;
            test_params.type = 1;
            test_params.channel = param->tx_channel;
            test_params.data_len = param->test_data_len;
            test_params.payload = param->pkt_payl;
            test_params.phy = param->phy;
            test_params.cte_len = param->cte_len;
            test_params.cte_type = param->cte_type;
            test_params.switching_pattern_len = param->switching_pattern_len;
            memcpy(&test_params.antenna_id[0], &param->antenna_id[0], param->switching_pattern_len);
            test_params.tx_pwr_lvl = param->tx_pwr_lvl;

#if (EAVESDROPPING_SUPPORT)
            test_params.cust_tx_test_params.enable = llm_env.cust_tx_test_params.enable;
            test_params.cust_tx_test_params.tx_packets = llm_env.cust_tx_test_params.tx_packets;
            test_params.cust_tx_test_params.tx_power = llm_env.cust_tx_test_params.tx_power;
#endif // EAVESDROPPING_SUPPORT

            status = lld_test_start(&test_params);
        }
    }

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}

int hci_le_test_end_cmd_handler(void const *param, uint16_t opcode)
{
    uint8_t status = lld_test_stop();

    if (status != CO_ERROR_NO_ERROR)
    {
        // Send the complete event
        struct hci_test_end_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_TEST_END_CMD_OPCODE, hci_test_end_cmd_cmp_evt);
        event->status = status;
        event->nb_packet_received = 0;
        hci_send_2_host(event);
    }

    return (KE_MSG_CONSUMED);
}

#if (EAVESDROPPING_SUPPORT)
int hci_vs_le_set_cust_transmitter_test_cmd_handler(struct hci_vs_le_set_cust_transmitter_test_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    llm_env.cust_tx_test_params.enable = param->enable;
    llm_env.cust_tx_test_params.tx_packets = param->tx_packets;
    llm_env.cust_tx_test_params.tx_power = param->tx_power;

    status = CO_ERROR_NO_ERROR;

    // Send the command complete event
    llm_cmd_cmp_send(opcode, status);

    return (KE_MSG_CONSUMED);
}
#endif // EAVESDROPPING_SUPPORT

#endif //(BLE_TEST_MODE_SUPPORT)
/// @} LLMTEST
