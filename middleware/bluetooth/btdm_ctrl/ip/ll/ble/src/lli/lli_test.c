/**
 ****************************************************************************************
 *
 * @file lli_test.c
 *
 * @brief Definition of the functions used by ISO test commands
 *
 * Copyright (C) RivieraWaves 2009-2019
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
#if (BLE_ISO_PRESENT)
#include "ke_task.h"        // For KE_MSG_HANDLER_NO_STATIC macro definition
#include "co_bt.h"          // HCI Definitions
#include "hci.h"            // HCI Interface
#include "lli_int.h"        // Isochronous internals
#include "co_math.h"        // for CO_BIT macro usage

#if (BLE_ISOGEN)
    #include "isogen.h"         // Isochronous Payload generator
#endif //(BLE_ISOGEN)

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

#if (BLE_CIS || BLE_BIS)
int hci_le_iso_tx_test_cmd_handler(struct hci_le_iso_tx_test_cmd const *param, uint16_t opcode)
{
    // Allocate HCI message
    struct hci_basic_conhdl_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_conhdl_cmd_cmp_evt);
    // Command status
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        if (param->payload_type > ISO_TEST_MAX_LEN)
            break;

        status = CO_ERROR_UNKNOWN_CONNECTION_ID;

#if (BLE_BIS)
        // If stream is a BIS
        if (param->conhdl >= BLE_BISHDL_MIN)
        {
            uint8_t act_id = BLE_BISHDL_TO_ACTID(param->conhdl);

            if (act_id < BLE_ACTIVITY_MAX)
            {
                // Load TX Data Path
                status = lli_bis_data_path_set(act_id, ISO_SEL_TX, ISO_DP_ISOGEN);

#if (BLE_ISOGEN)
                // Set payload type
                isogen_payload_type_set(act_id, ISO_SEL_TX, param->payload_type);
#endif // (BLE_ISOGEN)
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
                // Load TX Data Path
                status = lli_cis_data_path_set(act_id, ISO_SEL_TX, ISO_DP_ISOGEN);

#if (BLE_ISOGEN)
                // Set payload type
                isogen_payload_type_set(act_id, ISO_SEL_TX, param->payload_type);
#endif // (BLE_ISOGEN)
            }
            break;
        }
#endif // (BLE_CIS)

    }
    while (0);

    // Fill the message
    p_evt->status                 = status;
    p_evt->conhdl                 = param->conhdl;

    // Send the message
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}

int hci_le_iso_rx_test_cmd_handler(struct hci_le_iso_rx_test_cmd const *param, uint16_t opcode)
{
    // Allocate HCI message
    struct hci_basic_conhdl_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_conhdl_cmd_cmp_evt);
    // Command status
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    do
    {
        if (param->payload_type > ISO_TEST_MAX_LEN)
            break;

        status = CO_ERROR_UNKNOWN_CONNECTION_ID;

#if (BLE_BIS)
        // If stream is a BIS
        if (param->conhdl >= BLE_BISHDL_MIN)
        {
            uint8_t act_id = BLE_BISHDL_TO_ACTID(param->conhdl);

            if (act_id < BLE_ACTIVITY_MAX)
            {
                // Load TX Data Path
                status = lli_bis_data_path_set(act_id, ISO_SEL_RX, ISO_DP_ISOGEN);

#if (BLE_ISOGEN)
                // Set payload type
                isogen_payload_type_set(act_id, ISO_SEL_RX, param->payload_type);
#endif // (BLE_ISOGEN)
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
                // Load TX Data Path
                status = lli_cis_data_path_set(act_id, ISO_SEL_RX, ISO_DP_ISOGEN);

#if (BLE_ISOGEN)
                // Set payload type
                isogen_payload_type_set(act_id, ISO_SEL_RX, param->payload_type);
#endif // (BLE_ISOGEN)
            }
            break;
        }
#endif // (BLE_CIS)

    }
    while (0);

    // Fill the message
    p_evt->status                 = status;
    p_evt->conhdl                 = param->conhdl;

    // Send the message
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}

int hci_le_iso_read_test_counters_cmd_handler(struct hci_le_iso_read_test_counters_cmd const *param, uint16_t opcode)
{
    // Allocate HCI message
    struct hci_le_iso_read_test_counters_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_iso_read_test_counters_cmd_cmp_evt);

    // Fill the message
    p_evt->conhdl = param->conhdl;

#if (BLE_ISOGEN)
    p_evt->status = isogen_get_rx_stat(BLE_ISOHDL_TO_ACTID(param->conhdl), &p_evt->received_packet_count, &p_evt->missed_packet_count, &p_evt->failed_packet_count);
#endif // (BLE_ISOGEN)

    // Send the message
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}

int hci_le_iso_test_end_cmd_handler(struct hci_le_iso_test_end_cmd const *param, uint16_t opcode)
{
    // Allocate HCI message
    struct hci_le_iso_test_end_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_iso_test_end_cmd_cmp_evt);
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

#if (BLE_ISOGEN)
                status = isogen_get_rx_stat(act_id, &p_evt->received_packet_count, &p_evt->missed_packet_count, &p_evt->failed_packet_count);
#endif // (BLE_ISOGEN)

                // CO_ERROR_UNSUPPORTED means that the RX data path has not been set, which is allowed
                // The counters are set to 0 in this case
                if ((status != CO_ERROR_NO_ERROR) && (status != CO_ERROR_UNSUPPORTED))
                    break;

                // Remove RX data path on BIS channel
                status = lli_bis_data_path_remove(act_id, ISO_SEL_RX);

                if (status != CO_ERROR_NO_ERROR)
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

#if (BLE_ISOGEN)
                status = isogen_get_rx_stat(act_id, &p_evt->received_packet_count, &p_evt->missed_packet_count, &p_evt->failed_packet_count);
#endif // (BLE_ISOGEN)

                // CO_ERROR_UNSUPPORTED means that the RX data path has not been set, which is allowed
                // The counters are set to 0 in this case
                if ((status != CO_ERROR_NO_ERROR) && (status != CO_ERROR_UNSUPPORTED))
                    break;

                // Remove RX data path on CIS
                status = lli_cis_data_path_remove(act_id, ISO_SEL_RX);

                if (status != CO_ERROR_NO_ERROR)
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

    // Fill the message
    p_evt->status                 = status;
    p_evt->conhdl                 = param->conhdl;

    // Send the message
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}

#endif // (BLE_CIS || BLE_BIS)
#endif // (BLE_ISO_PRESENT)
/// @} LLI
