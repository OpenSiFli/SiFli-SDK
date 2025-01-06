/**
  ******************************************************************************
  * @file   common_task.c
  * @author Sifli software development team
  * @brief Source file - Handle common command.
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2022 - 2023,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/// Maximum number of instances of the common task

#include "board.h"
#include "rwip_config.h"     // RW SW configuration

#include <string.h>          // for mem* functions
#include <stdio.h>
#include "compiler.h"
#include "co_utils.h"

#include "ke_msg.h"
#include "ke_task.h"

#if (NVDS_SUPPORT)
    #include "rom_config.h"
    #include "sifli_nvds.h"         // NVDS definitions
#endif // NVDS_SUPPORT

#include "rwip.h"
#include "hci.h"

#if (BLE_HOST_PRESENT)
    #include "gapm.h"
    #include "gapm_int.h"
    #include "prf.h"
#endif

#include "common_task.h"

#include "co_hci.h"


#define COMMON_IDX_MAX 1

/// Test mode packet types
enum BT_TEST_PKT_TYPE
{
    DTM_DM1,
    DTM_DH1,
    DTM_DM3,
    DTM_DH3,
    DTM_DM5,
    DTM_DH5,
    DTM_2DH1,
    DTM_3DH1,
    DTM_2DH3,
    DTM_3DH3,
    DTM_2DH5,
    DTM_3DH5,
    DTM_HV1,
    DTM_HV2,
    DTM_HV3,
    DTM_EV3,
    DTM_EV4,
    DTM_EV5,
    DTM_2EV3,
    DTM_3EV3,
    DTM_2EV5,
    DTM_3EV5,
};

typedef struct
{
    ke_task_id_t dest_id;
    uint8_t operation;
} bt_test_cmd_env_t;

extern const struct ke_task_desc TASK_DESC_COMMON;


#if ((BLE_HOST_PRESENT) || (BT_HOST_PRESENT))
    static bt_test_cmd_env_t g_bt_test_cmd_env;
#endif

#if ((BLE_EMB_PRESENT) || (BT_EMB_PRESENT))

#if (NVDS_SUPPORT)
__STATIC int hci_nvds_data_update_cmd_handler(ke_msg_id_t const msgid, struct hci_nvds_data_update_cmd_t const *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct hci_basic_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, src_id, hci_basic_cmd_cmp_evt);
    int32_t ret = sifli_nvds_update((uint8_t *)param->data, param->length);

    event->status = ret == NVDS_OK ? CO_ERROR_NO_ERROR : CO_ERROR_UNSPECIFIED_ERROR;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}
#endif // (NVDS_SUPPORT)



__STATIC int
hci_wlan_coex_enable_cmd_handler(ke_msg_id_t const msgid, struct hci_wlan_coex_enable_cmd_t const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct hci_basic_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, src_id, hci_basic_cmd_cmp_evt);
    event->status = CO_ERROR_UNSUPPORTED;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}


__STATIC int
hci_wlan_coex_set_priority_cmd_handler(ke_msg_id_t const msgid, struct hci_wlan_coex_set_priority_cmd_t const *param,
                                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct hci_basic_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, src_id, hci_basic_cmd_cmp_evt);
    event->status = CO_ERROR_UNSUPPORTED;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}


__STATIC int
hci_wlan_coex_set_fiter_channel_cmd_handler(ke_msg_id_t const msgid, struct hci_wlan_coex_set_filter_channel_cmd_t const *param,
        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    struct hci_basic_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, src_id, hci_basic_cmd_cmp_evt);
    event->status = CO_ERROR_UNSUPPORTED;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

__STATIC int
hci_dbg_reserved_32_cmd_handler(ke_msg_id_t const msgid,
                                struct hci_dbg_reserved_32_cmd_t const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    struct hci_basic_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, msgid, hci_basic_cmd_cmp_evt);

    event->status = CO_ERROR_NO_ERROR;

    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

__STATIC int
hci_dbg_reserved_64_cmd_handler(ke_msg_id_t const msgid,
                                struct hci_dbg_reserved_64_cmd_t const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    struct hci_basic_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, msgid, hci_basic_cmd_cmp_evt);

    event->status = CO_ERROR_NO_ERROR;

    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

__STATIC int
hci_dbg_reserved_128_cmd_handler(ke_msg_id_t const msgid,
                                 struct hci_dbg_reserved_128_cmd_t const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    struct hci_basic_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, msgid, hci_basic_cmd_cmp_evt);

    event->status = CO_ERROR_NO_ERROR;

    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}




/// The message handlers for HCI command complete events
__STATIC const struct ke_msg_handler common_hci_command_handler_tab[] =
{
    // nvds
#if (NVDS_SUPPORT)
    {HCI_NVDS_DATA_UPDATE_CMD_OPCODE, (ke_msg_func_t)hci_nvds_data_update_cmd_handler},
#endif


    // Wlan coex
    {HCI_WLAN_COEX_ENABLE_CMD_OPCODE, (ke_msg_func_t)hci_wlan_coex_enable_cmd_handler},
    {HCI_WLAN_COEX_SET_PRIORITY_CMD_OPCODE, (ke_msg_func_t)hci_wlan_coex_set_priority_cmd_handler},
    {HCI_WLAN_COEX_SET_FILTER_CHANNEL_CMD_OPCODE, (ke_msg_func_t)hci_wlan_coex_set_fiter_channel_cmd_handler},

    {HCI_DBG_RESERVED1_CMD_OPCODE, (ke_msg_func_t)hci_dbg_reserved_32_cmd_handler},
    {HCI_DBG_RESERVED2_CMD_OPCODE, (ke_msg_func_t)hci_dbg_reserved_32_cmd_handler},
    {HCI_DBG_RESERVED3_CMD_OPCODE, (ke_msg_func_t)hci_dbg_reserved_32_cmd_handler},
    {HCI_DBG_RESERVED4_CMD_OPCODE, (ke_msg_func_t)hci_dbg_reserved_32_cmd_handler},
    {HCI_DBG_RESERVED5_CMD_OPCODE, (ke_msg_func_t)hci_dbg_reserved_32_cmd_handler},
    {HCI_DBG_RESERVED6_CMD_OPCODE, (ke_msg_func_t)hci_dbg_reserved_64_cmd_handler},
    {HCI_DBG_RESERVED7_CMD_OPCODE, (ke_msg_func_t)hci_dbg_reserved_64_cmd_handler},
    {HCI_DBG_RESERVED8_CMD_OPCODE, (ke_msg_func_t)hci_dbg_reserved_64_cmd_handler},
    {HCI_DBG_RESERVED9_CMD_OPCODE, (ke_msg_func_t)hci_dbg_reserved_64_cmd_handler},
    {HCI_DBG_RESERVED10_CMD_OPCODE, (ke_msg_func_t)hci_dbg_reserved_64_cmd_handler},
    {HCI_DBG_RESERVED11_CMD_OPCODE, (ke_msg_func_t)hci_dbg_reserved_128_cmd_handler},
    {HCI_DBG_RESERVED12_CMD_OPCODE, (ke_msg_func_t)hci_dbg_reserved_128_cmd_handler},
};


/**
****************************************************************************************
    * @brief Handles any HCI command
    *
    * @param[in] msgid Id of the message received (probably unused).
    * @param[in] param Pointer to the parameters of the message.
    * @param[in] dest_id ID of the receiving task instance (probably unused).
    * @param[in] src_id ID of the sending task instance.
    * @return If the message was consumed or not.
****************************************************************************************
*/
__STATIC int
hci_command_handler(ke_msg_id_t const msgid,
                    void const *param,
                    ke_task_id_t const dest_id,
                    ke_task_id_t const src_id)
{
    int return_status = KE_MSG_CONSUMED;

    // Check if there is a handler corresponding to the original command opcode
    for (uint16_t i = 0; i < (sizeof(common_hci_command_handler_tab) / sizeof(common_hci_command_handler_tab[0])); i++)
    {
        // Check if opcode matches
        if (common_hci_command_handler_tab[i].id == src_id)
        {
            // Check if there is a handler function
            if (common_hci_command_handler_tab[i].func != NULL)
            {
                // Call handler
                return_status = common_hci_command_handler_tab[i].func(src_id, param, dest_id, src_id);
            }
            break;
        }
    }

    return return_status;
}
#endif // #if ((BLE_EMB_PRESENT) || (BT_EMB_PRESENT))

#if ((BLE_HOST_PRESENT) || (BT_HOST_PRESENT))
__STATIC int
hci_event_handler(ke_msg_id_t const msgid,
                  void const *param,
                  ke_task_id_t const dest_id,
                  ke_task_id_t const src_id)
{
    int return_status = KE_MSG_CONSUMED;
    return return_status;
}

__STATIC int
hci_cmd_comp_event_handler(ke_msg_id_t const msgid,
                           void const *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id)
{
    int return_status = KE_MSG_CONSUMED;
    rt_kprintf("id %x, det %d\r\n", src_id, g_bt_test_cmd_env.dest_id);
    switch (src_id)
    {
    case HCI_DBG_RESERVED6_EXT_CMD_OPCODE:
    case HCI_DBG_RESERVED6_CMD_OPCODE:
    {
        ke_msg_forward(param, prf_get_task_from_id(TASK_ID_SIBLES), src_id);
        return_status = KE_MSG_NO_FREE;
    }
    break;
    case HCI_VS_TX_TEST_CMD_OPCODE:
    {

        struct hci_basic_cmd_cmp_evt const *evt = (struct hci_basic_cmd_cmp_evt const *)param;
        struct bt_test_mode_ctrl_rsp *rsp = KE_MSG_ALLOC(COMM_BT_TEST_MODE_CTRL_RSP, g_bt_test_cmd_env.dest_id,
                                            KE_BUILD_ID(TASK_COMMON, 0), bt_test_mode_ctrl_rsp);
        rsp->op = BT_TEST_OP_TX_TEST;
        rsp->status = evt->status;

        if (g_bt_test_cmd_env.operation != BT_TEST_OP_TX_TEST)
            rt_kprintf("TX test op is wrongly %d\r\n", g_bt_test_cmd_env.operation);

        g_bt_test_cmd_env.operation = 0;
        ke_msg_send(rsp);
    }
    break;
    case HCI_VS_RX_TEST_CMD_OPCODE:
    {
        struct hci_basic_cmd_cmp_evt const *evt = (struct hci_basic_cmd_cmp_evt const *)param;
        struct bt_test_mode_ctrl_rsp *rsp = KE_MSG_ALLOC(COMM_BT_TEST_MODE_CTRL_RSP, g_bt_test_cmd_env.dest_id,
                                            KE_BUILD_ID(TASK_COMMON, 0), bt_test_mode_ctrl_rsp);
        rsp->op = BT_TEST_OP_RX_TEST;
        rsp->status = evt->status;

        if (g_bt_test_cmd_env.operation != BT_TEST_OP_RX_TEST)
            rt_kprintf("TX test op is wrongly %d\r\n", g_bt_test_cmd_env.operation);

        g_bt_test_cmd_env.operation = 0;
        ke_msg_send(rsp);
    }
    break;
    case HCI_VS_TEST_END_CMD_OPCODE:
    {
        struct hci_vs_test_end_cmd_cmp_evt *evt = (struct hci_vs_test_end_cmd_cmp_evt *)param;
        struct bt_test_mode_ctrl_rsp *rsp = KE_MSG_ALLOC(COMM_BT_TEST_MODE_CTRL_RSP, g_bt_test_cmd_env.dest_id,
                                            KE_BUILD_ID(TASK_COMMON, 0), bt_test_mode_ctrl_rsp);
        rsp->op = BT_TEST_OP_STOP_TEST;
        rsp->status = evt->status;
        rsp->para.stop_para.cnt = evt->nb_packets;

        if (g_bt_test_cmd_env.operation != BT_TEST_OP_STOP_TEST)
            rt_kprintf("TX test op is wrongly %d\r\n", g_bt_test_cmd_env.operation);

        g_bt_test_cmd_env.operation = 0;
        ke_msg_send(rsp);
    }
    }

    return return_status;
}

__STATIC int
hci_packed_data_handler(ke_msg_id_t const msgid,
                        struct hci_packed_data *param,
                        ke_task_id_t const dest_id,
                        ke_task_id_t const src_id)
{
    hci_send_2_controller(param);
    return KE_MSG_NO_FREE;
}

static void bt_test_config_pwr(uint8_t pkt_type, int tx_pwr)
{
    uint8_t is_edr;

    switch (pkt_type)
    {
    case DTM_2DH1:
    case DTM_3DH1:
    case DTM_2DH3:
    case DTM_3DH3:
    case DTM_2DH5:
    case DTM_3DH5:
    case DTM_2EV3:
    case DTM_3EV3:
    case DTM_2EV5:
    case DTM_3EV5:
        is_edr = 1;
        break;
    default:
        is_edr = 0;
    }
    {
        extern void blebredr_rf_power_set(uint8_t type, int8_t txpwr);
        blebredr_rf_power_set(is_edr, tx_pwr);
    }
}

__STATIC int
bt_test_mode_ctrl_handler(ke_msg_id_t const msgid,
                          struct bt_test_mode_ctrl_cmd *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    uint8_t is_rsp = 0;
    uint8_t status = 0;
    uint8_t operation = 0;

    rt_kprintf("test handler\r\n");
    do
    {
        if (g_bt_test_cmd_env.operation != 0)
        {
            is_rsp = 1;
            // busying
            status = 2;
            break;
        }

        switch (param->op)
        {
        case BT_TEST_OP_ENTER_TEST:
        {
            // Stop activity in Host and reset controller.
            // Reset the stack
            struct gapm_reset_cmd *p_cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,
                                           TASK_GAPM, TASK_COMMON,
                                           gapm_reset_cmd);

            p_cmd->operation = GAPM_RESET;
            operation = BT_TEST_OP_ENTER_TEST;
            ke_msg_send(p_cmd);
        }
        break;
        case BT_TEST_OP_EXIT_TEST:
        {
            // Reset to app
            struct gapm_reset_cmd *p_cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,
                                           TASK_GAPM, TASK_APP,
                                           gapm_reset_cmd);

            p_cmd->operation = GAPM_RESET;
            //operation = BT_TEST_OP_EXIT_TEST;
            is_rsp = 1;
            status = 0;
            ke_msg_send(p_cmd);
        }
        break;
        case BT_TEST_OP_TX_TEST:
        {

            struct hci_vs_tx_test_cmd *tx_test
                = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_VS_TX_TEST_CMD_OPCODE, hci_vs_tx_test_cmd);

            tx_test->tx_channel = param->para.tx_para.channel;
            tx_test->test_data_len = param->para.tx_para.pkt_len;
            tx_test->pkt_type = param->para.tx_para.pkt_type;
            tx_test->pkt_payl = param->para.tx_para.pkt_payload;
            tx_test->tx_pwr_lvl = param->para.tx_para.pwr_lvl;

            operation = BT_TEST_OP_TX_TEST;

            bt_test_config_pwr(tx_test->pkt_type, tx_test->tx_pwr_lvl);

            // message send */
            hci_send_2_controller(tx_test);
        }
        break;
        case BT_TEST_OP_RX_TEST:
        {
            struct hci_vs_rx_test_cmd *rx_test
                = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_VS_RX_TEST_CMD_OPCODE, hci_vs_rx_test_cmd);

            rx_test->rx_channel = param->para.rx_para.channel;
            rx_test->pkt_type = param->para.rx_para.pkt_type;


            operation = BT_TEST_OP_RX_TEST;

            // message send */
            hci_send_2_controller(rx_test);
        }
        break;
        case BT_TEST_OP_STOP_TEST:
        {

            operation = BT_TEST_OP_STOP_TEST;
            hci_basic_cmd_send_2_controller(HCI_VS_TEST_END_CMD_OPCODE);
        }
        break;
        default:
        {
            is_rsp = 1;
            status = 1;
        }
        break;
        }
    }
    while (0);

    if (operation != 0)
    {
        g_bt_test_cmd_env.operation = operation;
        g_bt_test_cmd_env.dest_id = src_id;
    }

    if (is_rsp)
    {
        struct bt_test_mode_ctrl_rsp *rsp = KE_MSG_ALLOC(COMM_BT_TEST_MODE_CTRL_RSP, src_id, KE_BUILD_ID(TASK_COMMON, 0), bt_test_mode_ctrl_rsp);
        rsp->op = param->op;
        rsp->status = status;
        if (param->op == BT_TEST_OP_STOP_TEST)
            rsp->para.stop_para.cnt = 0;

        ke_msg_send(rsp);
    }

    return KE_MSG_CONSUMED;
}


static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    switch (param->operation)
    {
    case GAPM_RESET:
    {
        struct bt_test_mode_ctrl_rsp *rsp = KE_MSG_ALLOC(COMM_BT_TEST_MODE_CTRL_RSP, g_bt_test_cmd_env.dest_id, KE_BUILD_ID(TASK_COMMON, 0), bt_test_mode_ctrl_rsp);
        rsp->op = BT_TEST_OP_ENTER_TEST;
        rsp->status = param->status;
        if (g_bt_test_cmd_env.operation != BT_TEST_OP_ENTER_TEST)
            rt_kprintf("Enter test op is wrongly %d\r\n", g_bt_test_cmd_env.operation);
        g_bt_test_cmd_env.operation = 0;
        ke_msg_send(rsp);
        break;
    }
    default:
        break;
    }

    return KE_MSG_CONSUMED;
}


#endif // #if ((BLE_HOST_PRESENT) || (BT_HOST_PRESENT))

void common_task_init(bool is_reset)
{
    if (!is_reset)
        ke_task_create(TASK_COMMON, &TASK_DESC_COMMON);
}



/// Message handlers table
KE_MSG_HANDLER_TAB(common)
{
    // Note: all messages must be sorted in ID ascending order
#if ((BLE_EMB_PRESENT) || (BT_EMB_PRESENT))
    {HCI_COMMAND, (ke_msg_func_t)hci_command_handler},
#endif
#if ((BLE_HOST_PRESENT) || (BT_HOST_PRESENT))
    {HCI_EVENT, (ke_msg_func_t)hci_event_handler},
    {HCI_CMD_CMP_EVENT, (ke_msg_func_t)hci_cmd_comp_event_handler},
    {HCI_PACKED_DATA, (ke_msg_func_t)hci_packed_data_handler},
    {COMM_BT_TEST_MODE_CTRL_CMD, (ke_msg_func_t)bt_test_mode_ctrl_handler},
    {GAPM_CMP_EVT, (ke_msg_func_t)gapm_cmp_evt_handler},
#endif
};

/// Defines the placeholder for the states of all the task instances.
ke_state_t common_state[COMMON_IDX_MAX];

/// DEBUG task descriptor
const struct ke_task_desc TASK_DESC_COMMON = {common_msg_handler_tab, common_state, COMMON_IDX_MAX, ARRAY_LEN(common_msg_handler_tab)};

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
