/**
 ****************************************************************************************
 *
 * @file lli_task.c
 *
 * @brief LLI task source file
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLITASK
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
#include "co_utils.h"       // For KE_MSG_HANDLER_TAB macro definition
#include "lli_int.h"        // Internal LLI Task definitions
#include "hci.h"            // HCI Definitions
#include "llc.h"            // Definition of LLC messages
#include "lld.h"            // Definition of LLD messages
#include "llm.h"            // Definition of LLM messages
#include "aes.h"            // AES function result handling

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPES DEFINITION
 ****************************************************************************************
 */

/// Format of a HCI command handler function
typedef int (*lli_hci_cmd_hdl_func_t)(void const *param, uint16_t opcode);

/*
 * STRUCT DEFINITION
 ****************************************************************************************
 */

/// Element of a HCI command handler table.
struct lli_hci_cmd_handler
{
    /// Command opcode
    uint16_t opcode;
    /// Pointer to the handler function for HCI command.
    lli_hci_cmd_hdl_func_t func;
};

/*
 * CONSTANT DEFINITION
 ****************************************************************************************
 */

/*
 * ENUMERATIONS DEFINITION
 ****************************************************************************************
 */

/*
 * MESSAGE HANDLERS DECLARATION
 ****************************************************************************************
 */

/// CIS Specific commands (in lli_cis.c)
#if (BLE_CIS)
    #if (BLE_CENTRAL)
        extern int hci_le_set_cig_params_cmd_handler(struct hci_le_set_cig_params_cmd const *param, uint16_t opcode);
        extern int hci_le_set_cig_params_test_cmd_handler(struct hci_le_set_cig_params_test_cmd *param, uint16_t opcode);
        extern int hci_le_create_cis_cmd_handler(struct hci_le_create_cis_cmd const *param, uint16_t opcode);
        extern int hci_le_remove_cig_cmd_handler(struct hci_le_remove_cig_cmd const *param, uint16_t opcode);
    #endif //(BLE_CENTRAL)
    #if (BLE_PERIPHERAL)
        extern int hci_le_accept_cis_req_cmd_handler(struct hci_le_accept_cis_req_cmd const *param, uint16_t opcode);
        extern int hci_le_reject_cis_req_cmd_handler(struct hci_le_reject_cis_req_cmd const *param, uint16_t opcode);
    #endif //(BLE_PERIPHERAL)
    extern int hci_disconnect_cis_cmd_handler(struct hci_disconnect_cmd const *param, uint16_t opcode);
    // Handlers in lli_cis.c
    extern KE_MSG_HANDLER_NO_STATIC(lld_cis_stop_ind, struct lld_cis_stop_ind);
    extern KE_MSG_HANDLER_NO_STATIC(lld_cis_estab_ind, struct lld_cis_estab_ind);
#endif // (BLE_CIS)

/// BIS Specific commands (in lli_bis.c)
#if (BLE_BIS)
    #if (BLE_BROADCASTER)
        extern int hci_le_create_big_cmd_handler(struct hci_le_create_big_cmd const *param, uint16_t opcode);
        extern int hci_le_create_big_test_cmd_handler(struct hci_le_create_big_test_cmd const *param, uint16_t opcode);
        extern int lld_big_tx_ind_handler(ke_msg_id_t const msgid, struct lld_big_tx_ind const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
        extern int hci_le_terminate_big_cmd_handler(struct hci_le_terminate_big_cmd const *param, uint16_t opcode);
    #endif // (BLE_BROADCASTER)
    #if (BLE_OBSERVER)
        extern int hci_le_big_create_sync_cmd_handler(struct hci_le_big_create_sync_cmd const *param, uint16_t opcode);
        extern int hci_le_big_terminate_sync_cmd_handler(struct hci_le_big_terminate_sync_cmd const *param, uint16_t opcode);
        extern int llm_acad_data_ind_handler(ke_msg_id_t const msgid, struct llm_acad_data_ind const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
        extern int lld_big_rx_ind_handler(ke_msg_id_t const msgid, struct lld_big_rx_ind const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
        extern KE_MSG_HANDLER_NO_STATIC(lld_big_sync_estab_ind, struct lld_big_sync_estab_ind);
        extern KE_MSG_HANDLER_NO_STATIC(lld_big_sync_offset_upd_ind, struct lld_big_sync_offset_upd_ind);
    #endif // (BLE_OBSERVER)
    extern int lld_big_stop_ind_handler(ke_msg_id_t const msgid, struct lld_big_stop_ind const *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);
#endif // (BLE_BIS)

/// ISO Data Path configuration command (in lli_data_path.c)
#if (HCI_TL_SUPPORT && (BLE_CIS || (BLE_BIS && BLE_BROADCASTER)))
    int hci_le_rd_iso_tx_sync_cmd_handler(struct hci_basic_conhdl_cmd const *param, uint16_t opcode);
#endif // (HCI_TL_SUPPORT && (BLE_CIS || (BLE_BIS && BLE_BROADCASTER)))
#if (BLE_BIS || BLE_CIS)
    #if (HCI_TL_SUPPORT)
        extern int hci_le_rd_buf_size_v2_cmd_handler(void const *param, uint16_t opcode);
    #endif // (HCI_TL_SUPPORT)
    extern int hci_le_setup_iso_data_path_cmd_handler(struct hci_le_setup_iso_data_path_cmd const *param, uint16_t opcode);
    extern int hci_le_remove_iso_data_path_cmd_handler(struct hci_le_remove_iso_data_path_cmd const *param, uint16_t opcode);
    /// ISO test commands (in lli_test.c)
    extern int hci_le_iso_tx_test_cmd_handler(struct hci_le_iso_tx_test_cmd const *param, uint16_t opcode);
    extern int hci_le_iso_rx_test_cmd_handler(struct hci_le_iso_rx_test_cmd const *param, uint16_t opcode);
    extern int hci_le_iso_read_test_counters_cmd_handler(struct hci_le_iso_read_test_counters_cmd const *param, uint16_t opcode);
    extern int hci_le_iso_test_end_cmd_handler(struct hci_le_iso_test_end_cmd const *param, uint16_t opcode);
    extern int hci_le_rd_iso_link_quality_cmd_handler(struct hci_le_rd_iso_link_quality_cmd const *param, uint16_t opcode);
#endif // (BLE_BIS || BLE_CIS)

#if (BLE_ISO_MODE_0)
    /// ISO Mode 0 Specific commands (in lli_am0.c)
    extern int hci_vs_mic_less_set_cmd_handler(struct hci_vs_mic_less_set_cmd const *param, uint16_t opcode);
    extern int hci_vs_setup_am0_data_path_cmd_handler(struct hci_vs_setup_am0_data_path_cmd const *param, uint16_t opcode);
    extern int hci_vs_remove_am0_data_path_cmd_handler(struct hci_vs_remove_am0_data_path_cmd const *param, uint16_t opcode);
    extern int hci_vs_setup_am0_stream_cmd_handler(struct hci_vs_setup_am0_stream_cmd const *param, uint16_t opcode);
    extern int hci_vs_remove_am0_stream_cmd_handler(struct hci_vs_remove_am0_stream_cmd const *param, uint16_t opcode);
#endif // (BLE_ISO_MODE_0)

#if (RW_DEBUG)
    /// ISO Debug Command
    #if (BLE_BIS || BLE_CIS)
        extern int hci_dbg_iso_set_param_cmd_handler(struct hci_dbg_iso_set_param_cmd const *param, uint16_t opcode);
    #endif // (BLE_BIS || BLE_CIS)
#endif //(RW_DEBUG)

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */


/*
 * HCI COMMAND HANDLING
 ****************************************************************************************
 */

/// The message handlers for HCI commands
HCI_CMD_HANDLER_TAB(lli)
{
#if (HCI_TL_SUPPORT && (BLE_CIS || (BLE_BIS && BLE_BROADCASTER)))
    {HCI_LE_RD_ISO_TX_SYNC_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_rd_iso_tx_sync_cmd_handler        },
#endif // (HCI_TL_SUPPORT && (BLE_CIS || (BLE_BIS && BLE_BROADCASTER)))

#if (BLE_CIS)
    /// CIS Specific commands
#if (BLE_CENTRAL)
    {HCI_LE_SET_CIG_PARAMS_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_set_cig_params_cmd_handler        },
    {HCI_LE_SET_CIG_PARAMS_TEST_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_set_cig_params_test_cmd_handler   },
    {HCI_LE_CREATE_CIS_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_create_cis_cmd_handler            },
    {HCI_LE_REMOVE_CIG_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_remove_cig_cmd_handler            },
#endif //(BLE_CENTRAL)
#if (BLE_PERIPHERAL)
    {HCI_LE_ACCEPT_CIS_REQ_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_accept_cis_req_cmd_handler        },
    {HCI_LE_REJECT_CIS_REQ_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_reject_cis_req_cmd_handler        },
#endif //(BLE_PERIPHERAL)
    {HCI_DISCONNECT_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_disconnect_cis_cmd_handler           },
#endif // (BLE_CIS)

    /// BIS Specific commands
#if (BLE_BIS)
#if (BLE_BROADCASTER)
    {HCI_LE_CREATE_BIG_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_create_big_cmd_handler            },
    {HCI_LE_CREATE_BIG_TEST_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_create_big_test_cmd_handler       },
#endif // (BLE_BROADCASTER)
#if (BLE_OBSERVER)
    {HCI_LE_BIG_CREATE_SYNC_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_big_create_sync_cmd_handler       },
    {HCI_LE_BIG_TERMINATE_SYNC_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_big_terminate_sync_cmd_handler    },
#endif // (BLE_OBSERVER)
#if (BLE_BROADCASTER)
    {HCI_LE_TERMINATE_BIG_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_terminate_big_cmd_handler         },
#endif // (BLE_BROADCASTER)
#endif // (BLE_BIS)

#if (BLE_BIS || BLE_CIS)
    /// ISO Data Path configuration commands
#if (HCI_TL_SUPPORT)
    {HCI_LE_RD_BUF_SIZE_V2_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_rd_buf_size_v2_cmd_handler        },
#endif // (HCI_TL_SUPPORT)
    {HCI_LE_SETUP_ISO_DATA_PATH_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_setup_iso_data_path_cmd_handler   },
    {HCI_LE_REMOVE_ISO_DATA_PATH_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_remove_iso_data_path_cmd_handler  },
    /// Test commands
    {HCI_LE_ISO_TX_TEST_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_iso_tx_test_cmd_handler     },
    {HCI_LE_ISO_RX_TEST_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_iso_rx_test_cmd_handler      },
    {HCI_LE_ISO_READ_TEST_COUNTERS_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_iso_read_test_counters_cmd_handler},
    {HCI_LE_ISO_TEST_END_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_iso_test_end_cmd_handler          },
    {HCI_LE_RD_ISO_LINK_QUALITY_CMD_OPCODE, (lli_hci_cmd_hdl_func_t) hci_le_rd_iso_link_quality_cmd_handler   },
#endif // (BLE_BIS || BLE_CIS)

#if (BLE_ISO_MODE_0)
    {HCI_VS_MIC_LESS_SET_CMD_OPCODE, (lli_hci_cmd_hdl_func_t)  hci_vs_mic_less_set_cmd_handler         },
    {HCI_VS_SETUP_AM0_DATA_PATH_CMD_OPCODE, (lli_hci_cmd_hdl_func_t)  hci_vs_setup_am0_data_path_cmd_handler  },
    {HCI_VS_REMOVE_AM0_DATA_PATH_CMD_OPCODE, (lli_hci_cmd_hdl_func_t)  hci_vs_remove_am0_data_path_cmd_handler },
    {HCI_VS_SETUP_AM0_STREAM_CMD_OPCODE, (lli_hci_cmd_hdl_func_t)  hci_vs_setup_am0_stream_cmd_handler     },
    {HCI_VS_REMOVE_AM0_STREAM_CMD_OPCODE, (lli_hci_cmd_hdl_func_t)  hci_vs_remove_am0_stream_cmd_handler    },
#endif // (BLE_ISO_MODE_0)

#if (RW_DEBUG)
#if (BLE_BIS || BLE_CIS)
    {HCI_DBG_ISO_SET_PARAM_CMD_OPCODE, (lli_hci_cmd_hdl_func_t)  hci_dbg_iso_set_param_cmd_handler       },
#endif // (BLE_BIS || BLE_CIS)
#endif //(RW_DEBUG)
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
KE_MSG_HANDLER(hci_command_lli, void)
{
    int return_status = KE_MSG_CONSUMED;

    // Check if there is a handler corresponding to the original command opcode
    for (uint16_t i = 0; i < ARRAY_LEN(lli_hci_command_handler_tab); i++)
    {
        // Check if opcode matches
        if (lli_hci_command_handler_tab[i].opcode == src_id)
        {
            // Check if there is a handler function
            if (lli_hci_command_handler_tab[i].func != NULL)
            {
                // Call handler
                return_status = lli_hci_command_handler_tab[i].func(param, src_id);
            }

            break;
        }
    }

    return return_status;
}

/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Message handlers table
KE_MSG_HANDLER_TAB(lli)
{
    // Note: all messages must be sorted in ID ascending order
    /*
     * ************** Msg LLM->LLI ****************
     */
#if (BLE_BIS && BLE_OBSERVER)
    { LLM_ACAD_DATA_IND, (ke_msg_func_t) llm_acad_data_ind_handler       },
#endif // (BLE_BIS && BLE_OBSERVER)

    /*
     * ************** Msg LLD->LLI ****************
     */
#if (BLE_CIS)
    { LLD_CIS_STOP_IND, (ke_msg_func_t) lld_cis_stop_ind_handler        },
    { LLD_CIS_ESTAB_IND, (ke_msg_func_t) lld_cis_estab_ind_handler       },
#endif // (BLE_CIS)

#if (BLE_BIS)
    { LLD_BIG_STOP_IND, (ke_msg_func_t) lld_big_stop_ind_handler        },
#if (BLE_OBSERVER)
    { LLD_BIG_RX_IND, (ke_msg_func_t) lld_big_rx_ind_handler          },
#endif // (BLE_OBSERVER)
#if (BLE_BROADCASTER)
    { LLD_BIG_TX_IND, (ke_msg_func_t) lld_big_tx_ind_handler          },
#endif // (BLE_BROADCASTER)
#if (BLE_OBSERVER)
    { LLD_BIG_SYNC_ESTAB_IND, (ke_msg_func_t) lld_big_sync_estab_ind_handler  },
    { LLD_BIG_SYNC_OFFSET_UPD_IND, (ke_msg_func_t) lld_big_sync_offset_upd_ind_handler},
#endif // (BLE_OBSERVER)
#endif // (BLE_BIS)

    /*
     * ************** Msg HCI->LLI ****************
     */
    { HCI_COMMAND, (ke_msg_func_t) hci_command_lli_handler         },
};

/// Defines the place holder for the states of all the task instances.
__STATIC ke_state_t lli_state[LLI_IDX_MAX];

/// LLI task descriptor
const struct ke_task_desc TASK_DESC_LLI = {lli_msg_handler_tab, lli_state, LLI_IDX_MAX, ARRAY_LEN(lli_msg_handler_tab)};

#endif // (BLE_ISO_PRESENT)
/// @} LLITASK
