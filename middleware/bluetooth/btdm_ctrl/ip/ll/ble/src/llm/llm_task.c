/**
 ****************************************************************************************
 *
 * @file llm_task.c
 *
 * @brief LLM task source file
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LMTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <string.h>
#include "co_utils.h"
#include "co_bt.h"          // BLE standard definitions
#include "co_version.h"

#include "ke_timer.h"       // kernel timer definitions
#include "ke_msg.h"         // kernel message definitions
#include "ke_task.h"        // kernel task definitions

#include "llm.h"            // link layer manager definitions
#include "lld.h"            // link layer driver
#include "llm_int.h"        // link layer manager internal definitions

#include "rwip.h"           // stack main module

#include "sch_plan.h"       // Scheduling Planner

#if HCI_PRESENT
    #include "hci.h"            // host controller interface
#endif //HCI_PRESENT

#include "ecc_p256.h"

#include "aes.h"            // For encryption request

#include "dbg.h"


/*
 * DEFINES
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

extern KE_MSG_HANDLER_NO_STATIC(hci_command_llm, void);

#if (BLE_BROADCASTER)
    extern KE_MSG_HANDLER_NO_STATIC(lld_scan_req_ind, struct lld_scan_req_ind);
    extern KE_MSG_HANDLER_NO_STATIC(lld_adv_end_ind, struct lld_adv_end_ind);
    extern KE_MSG_HANDLER_NO_STATIC(lld_per_adv_end_ind, struct lld_per_adv_end_ind);
#endif // (BLE_BROADCASTER)

#if (BLE_OBSERVER)
    extern KE_MSG_HANDLER_NO_STATIC(llm_scan_period_to, void);
    extern KE_MSG_HANDLER_NO_STATIC(lld_adv_rep_ind, struct lld_adv_rep_ind);
    extern KE_MSG_HANDLER_NO_STATIC(lld_scan_end_ind, struct lld_scan_end_ind);
    extern KE_MSG_HANDLER_NO_STATIC(lld_sync_start_req, struct lld_sync_start_req);
    extern KE_MSG_HANDLER_NO_STATIC(lld_per_adv_rep_ind, struct lld_per_adv_rep_ind);
    extern KE_MSG_HANDLER_NO_STATIC(lld_per_adv_rx_end_ind, struct lld_per_adv_rx_end_ind);
    #if BLE_CONLESS_CTE_RX
        extern KE_MSG_HANDLER_NO_STATIC(lld_conless_cte_rx_ind, struct lld_conless_cte_rx_ind);
    #endif // BLE_CONLESS_CTE_RX
#endif //(BLE_OBSERVER)

#if (BLE_CENTRAL)
    extern KE_MSG_HANDLER_NO_STATIC(lld_init_end_ind, struct lld_init_end_ind);
#endif //(BLE_CENTRAL)

#if (BLE_TEST_MODE_SUPPORT)
    extern KE_MSG_HANDLER_NO_STATIC(lld_test_end_ind, struct lld_test_end_ind);
#endif // (BLE_TEST_MODE_SUPPORT)

#if (BLE_OBSERVER || BLE_BROADCASTER)
    extern KE_MSG_HANDLER_NO_STATIC(lld_ch_scan_end_ind, struct lld_ch_scan_end_ind);
#endif // (BLE_OBSERVER || BLE_BROADCASTER)

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */



/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles the channel map time out message.
 ****************************************************************************************
 */
#if (BLE_CENTRAL)
KE_MSG_HANDLER(llm_ch_map_to, void)
{
    llm_ch_map_update();

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the new Host classification time out message.
 ****************************************************************************************
 */
KE_MSG_HANDLER(llm_new_host_class_to, void)
{
    // Clear the flag indicating new Host classification
    llm_env.ch_map_info.new_host_class = false;

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_CENTRAL)

/**
 ****************************************************************************************
 * @brief Handles an encrypt confirmation
 ****************************************************************************************
 */
KE_MSG_HANDLER(llm_encrypt_ind, struct llm_encrypt_ind)
{
    // Report the encrypted data to Host via HCI
    struct hci_le_enc_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_LE_ENC_CMD_OPCODE, hci_le_enc_cmd_cmp_evt);
    memcpy(&event->encrypted_data[0], &param->result[0], ENC_DATA_LEN);
    event->status = CO_ERROR_NO_ERROR;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}


/*
 * TASK DESCRIPTOR DEFINITIONS
 ****************************************************************************************
 */

/// Message handlers table
KE_MSG_HANDLER_TAB(llm)
{
    // Note: all messages must be sorted in ID ascending order

#if (BLE_OBSERVER)
    {LLM_SCAN_PERIOD_TO, (ke_msg_func_t) llm_scan_period_to_handler     },
#endif //(BLE_OBSERVER)

    /*
     * ************** Msg LM->LM****************
     */
#if (BLE_CENTRAL)
    {LLM_CH_MAP_TO, (ke_msg_func_t) llm_ch_map_to_handler           },
    {LLM_NEW_HOST_CLASS_TO, (ke_msg_func_t) llm_new_host_class_to_handler   },
#endif //(BLE_CENTRAL)
    {LLM_ENCRYPT_IND, (ke_msg_func_t) llm_encrypt_ind_handler         },

    /*
     * ************** Msg LD->LM****************
     */
#if (BLE_OBSERVER)
    {LLD_ADV_REP_IND, (ke_msg_func_t) lld_adv_rep_ind_handler        },
#endif //(BLE_OBSERVER)

#if (BLE_BROADCASTER)
    {LLD_SCAN_REQ_IND, (ke_msg_func_t) lld_scan_req_ind_handler       },
#endif // (BLE_BROADCASTER)

#if (BLE_OBSERVER)
    {LLD_SYNC_START_REQ, (ke_msg_func_t) lld_sync_start_req_handler     },
    {LLD_PER_ADV_REP_IND, (ke_msg_func_t) lld_per_adv_rep_ind_handler    },
    {LLD_PER_ADV_RX_END_IND, (ke_msg_func_t) lld_per_adv_rx_end_ind_handler },
    {LLD_SCAN_END_IND, (ke_msg_func_t) lld_scan_end_ind_handler       },
#endif //(BLE_OBSERVER)

#if (BLE_BROADCASTER)
    {LLD_ADV_END_IND, (ke_msg_func_t) lld_adv_end_ind_handler        },
    {LLD_PER_ADV_END_IND, (ke_msg_func_t) lld_per_adv_end_ind_handler    },
#endif // (BLE_BROADCASTER)

#if (BLE_CENTRAL)
    {LLD_INIT_END_IND, (ke_msg_func_t) lld_init_end_ind_handler},
#endif //(BLE_CENTRAL)

#if (BLE_TEST_MODE_SUPPORT)
    {LLD_TEST_END_IND, (ke_msg_func_t) lld_test_end_ind_handler       },
#endif // (BLE_TEST_MODE_SUPPORT)

#if (BLE_OBSERVER)
#if BLE_CONLESS_CTE_RX
    {LLD_CONLESS_CTE_RX_IND, (ke_msg_func_t) lld_conless_cte_rx_ind_handler },
#endif // BLE_CONLESS_CTE_RX
#endif //(BLE_OBSERVER)

#if (BLE_CH_SCAN_SUPPORT)
    {LLD_CH_SCAN_END_IND, (ke_msg_func_t) lld_ch_scan_end_ind_handler    },
#endif //(BLE_CH_SCAN_SUPPORT)

    /*
     * ************** Msg HCI->LM****************
     */
    {HCI_COMMAND, (ke_msg_func_t) hci_command_llm_handler         },
};

/// Defines the place holder for the states of all the task instances.
__STATIC ke_state_t llm_state[LLM_IDX_MAX];

/// LLM task descriptor
const struct ke_task_desc TASK_DESC_LLM = {llm_msg_handler_tab, llm_state, LLM_IDX_MAX, ARRAY_LEN(llm_msg_handler_tab)};

/// @} LMTASK
