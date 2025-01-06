/**
 ****************************************************************************************
 *
 * @file gapm_cfg.c
 *
 * @brief Generic Access Profile Manager Device configuration management_module
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM_CFG Generic Access Profile Manager Device configuration management
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "rwble_hl.h"
#include "rwip.h"            // Common API to retrieve device parameters

#include <string.h>

#include "co_error.h"
#include "co_bt.h"
#include "co_math.h"
#include "co_version.h"
#include "co_utils.h"        // core utility functions

#include "gap.h"
#include "gapm_task.h"
#include "gapm_int.h"

#include "l2cm.h"
#include "gattm.h"
#include "attm.h"

#if (BLE_PROFILES)
    #include "prf.h"
#endif // (BLE_PROFILES)

#include "ke_mem.h"
#include "hci.h"
#include "ke_timer.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



/*
 * DEFINES
 ****************************************************************************************
 */


// GAP database default features
#define GAP_DB_DEFAULT_FEAT         0x001F
// GAP database features in peripheral role
#define GAP_DB_PERIPH_FEAT          0x0060
// GAP database features in central role
#define GAP_DB_CENTRAL_FEAT         0x0180
// GAP database features when privacy enabled
#define GAP_DB_PRIV_FEAT            0x0600

/// Low energy mask
#define GAP_EVT_MASK                {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9F, 0x00, 0x20}
#define GAP_LE_EVT_MASK             {0xFD, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00}
/// Mask to be used for debug if device is configured as a 4.2 device (command not available)
#define GAP_LE_EVT_4_2_MASK         {0xFC, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
/// Mask to be used for debug if device is configured as a 4.0 device
#define GAP_LE_EVT_4_0_MASK         {0x1C, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

/// state machine of reset operation
enum gapm_op_reset_state
{
    /// initialization of reset operation state machine
    GAPM_OP_RESET_INIT,
    /// HCI_RESET_CMD command execution completed
    GAPM_OP_RESET_HCI,
    /// HCI_SET_EVT_MASK_CMD command execution completed
    GAPM_OP_RESET_SET_EVT_MASK,
    /// HCI_LE_SET_EVT_MASK_CMD command execution completed
    GAPM_OP_RESET_LE_SET_EVT_MASK,
    /// HCI_RD_BD_ADDR_CMD command execution completed
    GAPM_OP_RESET_RD_BD_ADDR,
#if (BLE_CENTRAL || BLE_PERIPHERAL)
    /// HCI_LE_RD_BUFF_SIZE_CMD command execution completed
    GAPM_OP_RESET_LE_RD_BUFF_SIZE,
    /// HCI_RD_BUFF_SIZE_CMD command execution completed
    GAPM_OP_RESET_RD_BUFF_SIZE,
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)
#if (BLE_BROADCASTER)
    /// HCI_RD_MAX_ADV_DATA_LEN command execution completed
    GAPM_OP_RESET_RD_MAX_ADV_DATA_LEN,
    /// HCI_RD_NB_ADV_SETS command execution completed
    GAPM_OP_RESET_RD_NB_ADV_SETS,
#endif //(BLE_BROADCASTER)
};

/// state machine of setup operation
enum gapm_op_setup_state
{
    /// initialization of device setup operation state machine
    GAPM_OP_SETUP_INIT,
    /// HCI_LE_WR_SUGGTED_DFT_DATA_LEN_CMD command execution completed
    GAPM_OP_SETUP_WR_LE_DFT_DATA_LEN_CMD,
    /// HCI_LE_SET_DFT_PHY_CMD command execution completed
    GAPM_OP_SETUP_SET_LE_DFT_PHY_CMD,
    /// HCI_LE_WR_RF_PATH_COMP_CMD command execution completed
    GAPM_OP_SETUP_WR_RF_PATH_COMP_CMD,
    /// HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD command execution completed
    GAPM_OP_SETUP_SET_RENEW_TO,
    /// HCI_LE_SET_ADDR_RESOL_EN_CMD command execution completed
    GAPM_OP_SETUP_EN_CTRL_PRIV,
    /// HCI_RD_BD_ADDR_CMD command execution completed
    GAPM_OP_SETUP_RD_PUBLIC_ADDR_MGT,
#if (SECURE_CONNECTIONS)
    /// HCI_LE_RD_LOC_P256_PUB_KEY_CMD command execution completed
    GAPM_OP_SETUP_RD_PRIV_KEY,
#endif // (SECURE_CONNECTIONS)
};

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * Handles the Reset state machine
 *
 * @param[in] current_state  Current operation state (@see enum gapm_reset_state)
 * @param[in] status         Current operation status
 ****************************************************************************************
 */
__STATIC void gapm_op_reset_continue(uint8_t current_state, uint8_t status)
{
    bool finished = false;

    if (status != GAP_ERR_NO_ERROR)
    {
        // Abort of reset procedure
        finished = true;
    }
    else
    {
        switch (current_state)
        {
        case GAPM_OP_RESET_INIT:
        {
#if (SECURE_CONNECTIONS)
            // Try to load p256 Public Key
            uint8_t p256_key[PARAM_LEN_PUBLIC_KEY_P256];
            uint8_t p256_key_len = PARAM_LEN_PUBLIC_KEY_P256;

            if (rwip_param.get(PARAM_ID_LE_PUBLIC_KEY_P256, &p256_key_len, (uint8_t *)p256_key) == PARAM_OK)
            {
                // Copy the Public Key read from NVRAM to GAP.
                memcpy(&gapm_env.public_key.x[0], &p256_key[0], 32);
                memcpy(&gapm_env.public_key.y[0], &p256_key[32], 32);
            }
#endif // (SECURE_CONNECTIONS)

            // send a reset message to lower layers
            hci_basic_cmd_send_2_controller(HCI_RESET_CMD_OPCODE);
        }
        break;

        case GAPM_OP_RESET_HCI:
        {
            /* default device mask */
            uint8_t default_dev_mask[EVT_MASK_LEN] = GAP_EVT_MASK;

            /* query set event mask */
            struct hci_set_evt_mask_cmd *event_mask = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_SET_EVT_MASK_CMD_OPCODE, hci_set_evt_mask_cmd);

            /* fill up the event masking */
            memcpy(&event_mask->event_mask.mask[0], default_dev_mask, EVT_MASK_LEN);

            /* send the event mask */
            hci_send_2_controller(event_mask);

        }
        break;
        case GAPM_OP_RESET_SET_EVT_MASK:
        {
            /* default device mask */
            uint8_t default_dev_mask[EVT_MASK_LEN] = GAP_LE_EVT_MASK;

            /* send query LE event mask */
            struct hci_le_set_evt_mask_cmd *event_mask = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_EVT_MASK_CMD_OPCODE, hci_le_set_evt_mask_cmd);

            /* fill up the event masking */
            memcpy(&event_mask->le_mask.mask[0], default_dev_mask, EVT_MASK_LEN);

            /* send the event mask */
            hci_send_2_controller(event_mask);
        }
        break;

        case GAPM_OP_RESET_LE_SET_EVT_MASK:
        {
            /* Get local device address. */
            hci_basic_cmd_send_2_controller(HCI_RD_BD_ADDR_CMD_OPCODE);
        }
        break;

        case GAPM_OP_RESET_RD_BD_ADDR:
        {
#if (BLE_CENTRAL || BLE_PERIPHERAL)
            /* send read buffer size */
            hci_basic_cmd_send_2_controller(HCI_LE_RD_BUF_SIZE_CMD_OPCODE);
#elif (BLE_BROADCASTER)
            // Send read maximum advertising data length command
            hci_basic_cmd_send_2_controller(HCI_LE_RD_MAX_ADV_DATA_LEN_CMD_OPCODE);
#else
            // End of reset procedure
            finished = true;
#endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)
        }
        break;

#if (BLE_CENTRAL || BLE_PERIPHERAL)
        case GAPM_OP_RESET_LE_RD_BUFF_SIZE:
        {
            // check if number of ble packet is available
            if (l2cm_get_nb_buffer_available() == 0)
            {
                // number of buffer are shared with BT ACL packets

                // Send read buffer size legacy command to link layer
                hci_basic_cmd_send_2_controller(HCI_RD_BUF_SIZE_CMD_OPCODE);
            }
            else
            {
#if (BLE_BROADCASTER)
#ifdef RWIP_BLE_4
                //TODO:
                /* begin: temp */
                finished = true;

                gapm_env.max_adv_data_len = 0x672;
                gapm_env.max_adv_set = 1;
#else

                // Send read maximum advertising data length command
                hci_basic_cmd_send_2_controller(HCI_LE_RD_MAX_ADV_DATA_LEN_CMD_OPCODE);

#endif

                /*end*/
#else
                // End of reset procedure
                finished = true;
#endif //(BLE_BROADCASTER)
            }
        }
        break;
        case GAPM_OP_RESET_RD_BUFF_SIZE:
        {
#if (BLE_BROADCASTER)
            // Send read maximum advertising data length command
            hci_basic_cmd_send_2_controller(HCI_LE_RD_MAX_ADV_DATA_LEN_CMD_OPCODE);
#else
            // End of reset procedure
            finished = true;
#endif //(BLE_BROADCASTER)
        }
        break;
#endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)
#if (BLE_BROADCASTER)
        case GAPM_OP_RESET_RD_MAX_ADV_DATA_LEN:
        {
            // Send read number of supported advertising sets command to the controller
            hci_basic_cmd_send_2_controller(HCI_LE_RD_NB_SUPP_ADV_SETS_CMD_OPCODE);
        }
        break;
        case GAPM_OP_RESET_RD_NB_ADV_SETS:
        {
            // End of reset procedure
            finished = true;
        }
        break;
#endif //(BLE_BROADCASTER)
        default:
        {
            ASSERT_INFO(0, current_state, status);
            // Abort of reset procedure
            status = GAP_ERR_PROTOCOL_PROBLEM;
            finished = true;
        }
        break;
        }
    }

    if (finished)
    {
        // reset operation is finished, inform upper layers.
        gapm_send_complete_evt(GAPM_OP_CFG, status);

        // reset correctly finished.
        if (status == CO_ERROR_NO_ERROR)
        {
            // set state to idle
            ke_state_set(TASK_GAPM, GAPM_IDLE);
        }
    }

}

/**
 ****************************************************************************************
 * Handles the Device Setup state machine
 *
 * @param[in] current_state  Current operation state (@see enum gapm_reset_state)
 * @param[in] status         Current operation status
 ****************************************************************************************
 */
__STATIC void gapm_op_setup_continue(uint8_t current_state, uint8_t status)
{
    bool finished = false;

    // retrieve set config operation
    struct gapm_set_dev_config_cmd *set_cfg = (struct gapm_set_dev_config_cmd *) gapm_get_operation_ptr(GAPM_OP_CFG);

    if ((set_cfg == NULL) || (set_cfg->operation != GAPM_SET_DEV_CONFIG))
    {
        status = GAP_ERR_PROTOCOL_PROBLEM;
    }

    if (status != GAP_ERR_NO_ERROR)
    {
        // Abort of reset procedure
        finished = true;
    }
    else
    {
        switch (current_state)
        {
        case GAPM_OP_SETUP_INIT:
        {
            ke_timer_clear(GAPM_ADDR_RENEW_TO_IND, TASK_GAPM);
#if (BLE_CENTRAL || BLE_PERIPHERAL)
            // Write Default LE PHY
            struct hci_le_set_dft_phy_cmd *dft_phy = KE_MSG_ALLOC(HCI_COMMAND, 0,
                    HCI_LE_SET_DFT_PHY_CMD_OPCODE, hci_le_set_dft_phy_cmd);

            // Fill data
            dft_phy->all_phys  = (set_cfg->tx_pref_phy == GAP_PHY_ANY) ? ALL_PHYS_TX_NO_PREF : 0;
            dft_phy->all_phys |= (set_cfg->rx_pref_phy == GAP_PHY_ANY) ? ALL_PHYS_RX_NO_PREF : 0;

            dft_phy->rx_phys  = set_cfg->rx_pref_phy;
            dft_phy->tx_phys  = set_cfg->tx_pref_phy;

            /* send the message */
            hci_send_2_controller(dft_phy);

        }
        break;

        case GAPM_OP_SETUP_SET_LE_DFT_PHY_CMD:
        {
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)
            // Write RF Path Compensation values
            struct hci_le_wr_rf_path_comp_cmd *p_cmd
                = KE_MSG_ALLOC(HCI_COMMAND,
                               0, HCI_LE_WR_RF_PATH_COMP_CMD_OPCODE,
                               hci_le_wr_rf_path_comp_cmd);

            // Fill data
            p_cmd->tx_path_comp = set_cfg->tx_path_comp;
            p_cmd->rx_path_comp = set_cfg->rx_path_comp;

            /* send the message */
            hci_send_2_controller(p_cmd);
        }
        break;

        case GAPM_OP_SETUP_WR_RF_PATH_COMP_CMD:
        {
#if (BLE_CENTRAL || BLE_PERIPHERAL)
            // Write Suggested Default LE Data Length
            struct hci_le_wr_suggted_dft_data_len_cmd *set_sugg_data = KE_MSG_ALLOC(HCI_COMMAND, 0,
                    HCI_LE_WR_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, hci_le_wr_suggted_dft_data_len_cmd);

            // Fill data
            set_sugg_data->suggted_max_tx_octets = set_cfg->sugg_max_tx_octets;
            set_sugg_data->suggted_max_tx_time = set_cfg->sugg_max_tx_time;

            /* send the message */
            hci_send_2_controller(set_sugg_data);
        }
        break;


        case GAPM_OP_SETUP_WR_LE_DFT_DATA_LEN_CMD:
        {
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)
            // Set Address configuration.
            GAPM_F_SET(gapm_env.cfg_flags, ADDR_TYPE, set_cfg->privacy_cfg);
            gapm_env.renew_dur = co_max(set_cfg->renew_dur, GAP_TMR_PRIV_ADDR_MIN);
            gapm_env.renew_dur = co_min(gapm_env.renew_dur, GAP_TMR_PRIV_ADDR_MAX);

            // Check if controller privacy is enabled
            if (set_cfg->privacy_cfg & GAPM_PRIV_CFG_PRIV_EN_BIT)
            {
                // Set timeout for address generation
                struct hci_le_set_rslv_priv_addr_to_cmd *set_to = KE_MSG_ALLOC(HCI_COMMAND,
                        0, HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD_OPCODE,
                        hci_le_set_rslv_priv_addr_to_cmd);
                // Fill data
                set_to->rpa_timeout = set_cfg->renew_dur;

                /* send the message */
                hci_send_2_controller(set_to);
            }
            else
            {
                // If public address is used, it must be retrieved from the controller
                if (!(set_cfg->privacy_cfg & GAPM_PRIV_CFG_PRIV_ADDR_BIT))
                {
                    // Read public address in controller
                    hci_basic_cmd_send_2_controller(HCI_RD_BD_ADDR_CMD_OPCODE);
                }
                else
                {
                    finished = true;
                }
            }
        }
        break;

        case GAPM_OP_SETUP_SET_RENEW_TO:
        {
            // Enable Address Resolution in controller
            struct hci_le_set_addr_resol_en_cmd *en_addr_resol = KE_MSG_ALLOC(HCI_COMMAND,
                    0, HCI_LE_SET_ADDR_RESOL_EN_CMD_OPCODE,
                    hci_le_set_addr_resol_en_cmd);

            // Fill data
            en_addr_resol->enable = 1;

            /* send the message */
            hci_send_2_controller(en_addr_resol);
        }
        break;

        case GAPM_OP_SETUP_EN_CTRL_PRIV:
        {
            // If public address is used, it must be retrieved from the controller
            if (!(set_cfg->privacy_cfg & GAPM_PRIV_CFG_PRIV_ADDR_BIT))
            {
                // Read public address in controller
                hci_basic_cmd_send_2_controller(HCI_RD_BD_ADDR_CMD_OPCODE);
            }
            else
            {
                finished = true;
            }
        }
        break;

        case GAPM_OP_SETUP_RD_PUBLIC_ADDR_MGT:
        {
#if (SECURE_CONNECTIONS)
            // check if p256 public key has to be generated
            if ((gapm_env.pairing_mode & GAPM_PAIRING_FORCE_P256_KEY_GEN) != 0)
            {
                // clear generation status bit
                gapm_env.pairing_mode &= ~GAPM_PAIRING_FORCE_P256_KEY_GEN;
                // If Public Key not in NVRAM - then generate a new Public Key and store in NVRAM.
                hci_basic_cmd_send_2_controller(HCI_LE_RD_LOC_P256_PUB_KEY_CMD_OPCODE);
            }
            else
#endif // (SECURE_CONNECTIONS)
            {
                finished = true;
            }
        }
        break;

#if (SECURE_CONNECTIONS)
        case GAPM_OP_SETUP_RD_PRIV_KEY:
        {
            finished = true;
        }
        break;
#endif // (SECURE_CONNECTIONS)

        default:
        {
            ASSERT_INFO(0, current_state, status);
            // Abort of reset procedure
            status = GAP_ERR_PROTOCOL_PROBLEM;
            finished = true;
        }
        break;
        }
    }

    if (finished)
    {
        // Setup operation is finished, inform upper layers.
        gapm_send_complete_evt(GAPM_OP_CFG, status);
    }
}

#if (BLE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Allocate, fill and send a GAPM_MAX_ADV_DATA_LEN_IND message to the application.
 ****************************************************************************************
 */
__STATIC void gapm_cfg_send_max_adv_data_len_ind(void)
{
    // Allocate a GAPM_MAX_ADV_DATA_LEN_IND message
    struct gapm_max_adv_data_len_ind *p_ind = KE_MSG_ALLOC(GAPM_MAX_ADV_DATA_LEN_IND,
            gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
            gapm_max_adv_data_len_ind);

    // Fill the message
    p_ind->length = gapm_env.max_adv_data_len;

    // Send the message
    ke_msg_send(p_ind);
}

/**
 ****************************************************************************************
 * @brief Allocate, fill and send a GAPM_NB_ADV_SETS_IND message to the application.
 *
 * @param[in] nb_sets   Number of advertising sets currently supported by controller
 ****************************************************************************************
 */
__STATIC void gapm_cfg_send_nb_adv_sets_ind(uint8_t nb_sets)
{
    // Allocate a GAPM_NB_ADV_SETS_IND message
    struct gapm_nb_adv_sets_ind *p_ind = KE_MSG_ALLOC(GAPM_NB_ADV_SETS_IND,
                                         gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
                                         gapm_nb_adv_sets_ind);

    // Fill the message
    p_ind->nb_adv_sets = nb_sets;

    // Send the message
    ke_msg_send(p_ind);
}
#endif //(BLE_BROADCASTER)

/**
 ****************************************************************************************
 * @brief Allocate, fill and send a GAPM_DEV_TX_PWR_IND message to the application.
 *
 * @param[in] nb_sets   Number of advertising sets currently supported by controller
 ****************************************************************************************
 */
__STATIC void gapm_cfg_send_tx_pwr_ind(int8_t min_tx_pwr, int8_t max_tx_pwr)
{
    // Allocate a GAPM_DEV_TX_PWR_IND message
    struct gapm_dev_tx_pwr_ind *p_ind = KE_MSG_ALLOC(GAPM_DEV_TX_PWR_IND,
                                        gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
                                        gapm_dev_tx_pwr_ind);

    // Fill the message
    p_ind->min_tx_pwr = min_tx_pwr;
    p_ind->max_tx_pwr = max_tx_pwr;

    // Send the message
    ke_msg_send(p_ind);
}

/**
 ****************************************************************************************
 * @brief Allocate, fill and send a GAPM_DEV_RF_PATH_COMP_IND message to the application.
 *
 * @param[in] nb_sets   Number of advertising sets currently supported by controller
 ****************************************************************************************
 */
__STATIC void gapm_cfg_send_rf_path_comp_ind(uint16_t tx_path_comp, uint16_t rx_path_comp)
{
    // Allocate a GAPM_DEV_RF_PATH_COMP_IND message
    struct gapm_dev_rf_path_comp_ind *p_ind = KE_MSG_ALLOC(GAPM_DEV_RF_PATH_COMP_IND,
            gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
            gapm_dev_rf_path_comp_ind);

    // Fill the message
    p_ind->tx_path_comp = tx_path_comp;
    p_ind->rx_path_comp = rx_path_comp;

    // Send the message
    ke_msg_send(p_ind);
}

/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */



/*
 * MESSAGES HANDLERS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles request of host reset + Initialization of lower layers.
 *  - GAPM_RESET: software reset operation.
 *  - GAPM_PLF_RESET: Platform reset
 *
 *  Procedure:
 *   1. HCI_RESET_CMD
 *   2. HCI_SET_EVT_MASK_CMD
 *   3. HCI_LE_SET_EVT_MASK_CMD
 *   4. HCI_RD_BD_ADDR_CMD
 *   5. HCI_LE_RD_BUFF_SIZE_CMD
 *   6. HCI_RD_BUFF_SIZE_CMD
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
int gapm_reset_cmd_handler(ke_msg_id_t const msgid, struct gapm_reset_cmd *param,
                           ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Returned message status
    int msg_status = KE_MSG_CONSUMED; // Reset State

    // check operation is a reset request
    if (param->operation == GAPM_RESET)
    {
        // perform a reset of higher layers.
        rwble_hl_reset();

        // Initialize operation execution.
        gapm_set_operation_ptr(GAPM_OP_CFG, param);

        msg_status = KE_MSG_NO_FREE;

        // initialize reset operation
        gapm_op_reset_continue(GAPM_OP_RESET_INIT, GAP_ERR_NO_ERROR);
    }
    else if (param->operation == GAPM_PLF_RESET)
    {
        // Reset the platform
        platform_reset(RESET_AND_LOAD_FW);
    }
    else
    {
        // Invalid parameters set.
        gapm_send_error_evt(param->operation, src_id, GAP_ERR_INVALID_PARAM);
    }

    return msg_status;
}



/**
 ****************************************************************************************
 * @brief Handles modification of device GAP configuration such as role, security
 * parameters, etc:
 *  - GAPM_SET_DEV_CONFIG: Set device configuration
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
int gapm_set_dev_config_cmd_handler(ke_msg_id_t const msgid, struct gapm_set_dev_config_cmd *param,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_SET_DEV_CONFIG, GAPM_NO_OP};
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // no need to check GAPM_SET_DEV_CONFIG operation, this has been done by gapm_process_op
        uint8_t status = GAP_ERR_NO_ERROR;
        uint8_t authorized_role = GAP_ROLE_NONE;

        do
        {
            uint8_t role = param->role & GAP_ROLE_ALL;

            /* Configuration can be modified only if no connection exists and no air
             * operation on-going.
             */
            if ((gapm_env.connections != 0)
                    || (gapm_get_operation_ptr(GAPM_OP_AIR) != NULL) // No air operation
                    || gapm_env.started_actvs
               )
            {
                /* send command complete event with error */
                status = GAP_ERR_COMMAND_DISALLOWED;
                break;
            }

            // create a variable used to check if role is supported or not
#if (BLE_BROADCASTER)
            authorized_role |= GAP_ROLE_BROADCASTER;
#endif // (BLE_BROADCASTER)

#if (BLE_OBSERVER)
            authorized_role |= GAP_ROLE_OBSERVER;
#endif // (BLE_OBSERVER)

#if (BLE_CENTRAL)
            authorized_role |= GAP_ROLE_CENTRAL;
#endif // (BLE_CENTRAL)

#if (BLE_PERIPHERAL)
            authorized_role |= GAP_ROLE_PERIPHERAL;
#endif // (BLE_PERIPHERAL)

            // Check that configured role is in authorized role
            if ((role | authorized_role) != authorized_role)
            {
                /* Role which is configured is not supported by the Host stack */
                status = GAP_ERR_NOT_SUPPORTED;
                break;
            }

            // Data Length sanity check
            if (gapm_dle_val_check(param->sugg_max_tx_octets, param->sugg_max_tx_time))
            {
                status = GAP_ERR_INVALID_PARAM;
                break;
            }

            // clear role and config
            gapm_env.cfg_flags = 0;
            gapm_env.role = GAP_ROLE_NONE;

#if (BLE_ATTS)
            {
                /* First Clear all the database */
                attmdb_destroy();

                // database available only for central and peripheral roles.
                if (((role & GAP_ROLE_PERIPHERAL) == GAP_ROLE_PERIPHERAL)
                        || ((role & GAP_ROLE_CENTRAL) == GAP_ROLE_CENTRAL))
                {
                    // define gap database features
                    uint32_t db_cfg = GAP_DB_DEFAULT_FEAT;

                    if (GAPM_F_GET(param->att_cfg, ATT_SLV_PREF_CON_PAR_EN))
                    {
                        db_cfg |= GAP_DB_PERIPH_FEAT;
                        GAPM_F_SET(gapm_env.cfg_flags, PREF_CON_PAR_PRES, 1);
                    }

                    if (role & GAP_ROLE_CENTRAL)
                    {
                        if (param->privacy_cfg & GAPM_PRIV_CFG_PRIV_EN_BIT)
                        {
                            db_cfg |= GAP_DB_CENTRAL_FEAT;
                        }
                    }

                    db_cfg |= GAP_DB_PRIV_FEAT;

#if (RW_DEBUG)
                    gapm_env.dbg_cfg = 0;
                    if (GAPM_F_GET(param->att_cfg, ATT_DBG_MODE_EN))
                    {
                        GAPM_F_SET(gapm_env.dbg_cfg, DBG_MODE_EN, 1);
                    }

                    if (GAPM_F_GET(param->att_cfg, ATT_DBG_L2CAP_TRAFFIC_EN))
                    {
                        GAPM_F_SET(gapm_env.dbg_cfg, DBG_L2CAP_TRAFFIC_FWD_EN, 1);
                    }
#endif // (RW_DEBUG)

                    /* Create GAP Attribute Database */
                    status = gapm_init_attr(param->gap_start_hdl, db_cfg);
                    if (status != GAP_ERR_NO_ERROR)
                    {
                        break;
                    }

                    // define gatt database features
                    if (GAPM_F_GET(param->att_cfg, ATT_SVC_CHG_EN))
                    {
                        GAPM_F_SET(gapm_env.cfg_flags, SVC_CHG_EN, 1);
                    }

                    /* Create GATT Attribute Database */
                    status = gattm_init_attr(param->gatt_start_hdl, GAPM_F_GET(param->att_cfg, ATT_SVC_CHG_EN));
                    if (status != GAP_ERR_NO_ERROR)
                    {
                        break;
                    }
                }

                // Set appearance characteristic permissions
                if (GAPM_F_GET(param->att_cfg, ATT_APPEARENCE_PERM) != GAPM_WRITE_DISABLE)
                {
                    // Set appearance write permission
                    ATTMDB_UPDATE_PERM_VAL(gapm_get_att_handle(GAP_IDX_ICON), WP, (GAPM_F_GET(param->att_cfg, ATT_APPEARENCE_PERM) - 1));
                    ATTMDB_UPDATE_PERM(gapm_get_att_handle(GAP_IDX_ICON), WRITE_REQ, ENABLE);
                }

                // Set device name characteristic permissions
                if (GAPM_F_GET(param->att_cfg, ATT_NAME_PERM) != GAPM_WRITE_DISABLE)
                {
                    // Set device name write permission
                    ATTMDB_UPDATE_PERM_VAL(gapm_get_att_handle(GAP_IDX_DEVNAME), WP, (GAPM_F_GET(param->att_cfg, ATT_NAME_PERM) - 1));
                    ATTMDB_UPDATE_PERM(gapm_get_att_handle(GAP_IDX_DEVNAME), WRITE_REQ, ENABLE);
                }
            }
#endif //(BLE_ATTS)

#if (BLE_PROFILES)
            /* Then remove all profile tasks */
            prf_init(true);
#endif // (BLE_PROFILES)

            // set role
            gapm_env.role = role;

#if (BLE_CENTRAL || BLE_PERIPHERAL)
            // Set maximal MTU
            gapm_set_max_mtu(param->max_mtu);
            // Set maximal MPS
            gapm_set_max_mps(param->max_mps);

#if (BLE_LECB)
            // remove all registered LE_PSM
            gapm_le_psm_cleanup();
            gapm_env.nb_lecb     = 0;
            gapm_env.max_nb_lecb = param->max_nb_lecb;
#endif // (BLE_LECB)

            // security configuration only needed if device is peripheral or central
            if ((gapm_env.role & ~(GAP_ROLE_OBSERVER | GAP_ROLE_BROADCASTER)) != 0)
            {
                // Store pairing mode
                gapm_env.pairing_mode = param->pairing_mode;

#if (SECURE_CONNECTIONS)
                // check if secure connection is allowed or not
                if ((gapm_env.pairing_mode & GAPM_PAIRING_SEC_CON) == 0)
#endif // (SECURE_CONNECTIONS)
                {
                    // ensure that p256 public key will not be generated
                    gapm_env.pairing_mode &= ~GAPM_PAIRING_FORCE_P256_KEY_GEN;
                    gapm_env.pairing_mode &= ~GAPM_PAIRING_SEC_CON;
                }
            }
            else
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)
            {
                gapm_env.pairing_mode = 0;
            }

            // Set device Identity key (IRK)
            memcpy(&(gapm_env.irk), &(param->irk), sizeof(struct gap_sec_key));

            if (param->privacy_cfg & GAPM_PRIV_CFG_PRIV_ADDR_BIT)
            {
                // Copy static address received from application in local memory
                memcpy(&(gapm_env.addr), &(param->addr), GAP_BD_ADDR_LEN);
            }

#if(BLE_ISO_MODE_0_PROTOCOL)
            // copy audio configuration
            gapm_env.audio_cfg = param->audio_cfg;
#endif // (BLE_ISO_MODE_0_PROTOCOL)
        }
        while (0);

        if (status == GAP_ERR_NO_ERROR)
        {
            gapm_op_setup_continue(GAPM_OP_SETUP_INIT, GAP_ERR_NO_ERROR);
        }
        else
        {
            gapm_send_complete_evt(GAPM_OP_CFG, status);
        }
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles request of modifying local channel map:
 *  - GAPM_SET_CHANNEL_MAP:  Set device channel map
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
int gapm_set_channel_map_cmd_handler(ke_msg_id_t const msgid, struct gapm_set_channel_map_cmd *param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_SET_CHANNEL_MAP, GAPM_NO_OP};
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // no need to check GAPM_SET_CHANNEL_MAP operation, this has been done by gapm_process_op

        // this is only for central role
        if (GAPM_IS_ROLE_SUPPORTED(GAP_ROLE_CENTRAL))
        {
            // allocate set host channel classification message
            struct hci_le_set_host_ch_class_cmd *ch_class = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_SET_HOST_CH_CLASS_CMD_OPCODE, hci_le_set_host_ch_class_cmd);

            // update channel map value
            memcpy(&ch_class->ch_map.map[0], &param->chmap.map[0], LE_CH_MAP_LEN);

            // message send */
            hci_send_2_controller(ch_class);
        }
        else
        {
            // send command complete event with error
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_COMMAND_DISALLOWED);
        }
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles request of getting information about local device such as:
 * - GAPM_GET_DEV_NAME: Get Local device name indication event
 * - GAPM_GET_DEV_VERSION: Get Local device version indication event
 * - GAPM_GET_DEV_BDADDR: Get Local device BD Address indication event
 * - GAPM_GET_DEV_ADV_TX_POWER: Get device advertising power level
 * - GAPM_DBG_GET_MEM_INFO: Get memory usage (debug only)
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
int gapm_get_dev_info_cmd_handler(ke_msg_id_t const msgid, struct gapm_get_dev_info_cmd *param,
                                  ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = { GAPM_GET_DEV_VERSION,
                                       GAPM_GET_DEV_BDADDR, GAPM_GET_DEV_ADV_TX_POWER,
                                       GAPM_DBG_GET_MEM_INFO,
                                       GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN,
                                       GAPM_GET_MAX_LE_DATA_LEN,
                                       GAPM_GET_WLIST_SIZE,
                                       GAPM_GET_PAL_SIZE,
                                       GAPM_GET_RAL_SIZE,
                                       GAPM_GET_NB_ADV_SETS,
                                       GAPM_GET_MAX_LE_ADV_DATA_LEN,
                                       GAPM_NO_OP
                                     };
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {

        // check operation
        switch (param->operation)
        {
        // Get Local device version
        case GAPM_GET_DEV_VERSION:
        {
            /* Get the device local version */
            hci_basic_cmd_send_2_controller(HCI_RD_LOCAL_VER_INFO_CMD_OPCODE);
        }
        break;
        // Get Local device BD Address
        case GAPM_GET_DEV_BDADDR:
        {
            struct gapm_dev_bdaddr_ind *bdaddr_ind =
                KE_MSG_ALLOC(GAPM_DEV_BDADDR_IND, src_id, dest_id, gapm_dev_bdaddr_ind);
            /* fill up the parameters */
            memcpy(&(bdaddr_ind->addr.addr), &(gapm_env.addr), BD_ADDR_LEN);
            bdaddr_ind->addr.addr_type
                = (gapm_get_address_type() & GAPM_PRIV_CFG_PRIV_ADDR_BIT) ? ADDR_RAND : ADDR_PUBLIC;
            bdaddr_ind->actv_idx = GAPM_ACTV_INVALID_IDX;

            /* send the message indication */
            ke_msg_send(bdaddr_ind);

            /* send command complete event with error */
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_NO_ERROR);
        }
        break;
        // Get device advertising Tx Power level
        case GAPM_GET_DEV_ADV_TX_POWER:
        {
#if ((BLE_OBSERVER) || (BLE_PERIPHERAL))
            /* send read adv tx power level */
            hci_basic_cmd_send_2_controller(HCI_LE_RD_ADV_CHNL_TX_PW_CMD_OPCODE);
#else // ((BLE_OBSERVER) || (BLE_PERIPHERAL))
            /* send command complete event with error */
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_NOT_SUPPORTED);
#endif // ((BLE_OBSERVER) || (BLE_PERIPHERAL))
        }
        break;
        // Get device memory usage
        case GAPM_DBG_GET_MEM_INFO:
        {
#if (KE_PROFILING)
            uint8_t cursor;
            struct gapm_dbg_mem_info_ind meminfo;
            struct gapm_dbg_mem_info_ind *meminfo_msg;

            // First remove command message in order to be sure it's not taken in account.
            ke_msg_free(ke_param2msg(param));
            gapm_set_operation_ptr(GAPM_OP_CFG, NULL);

            // Then retrieve memory information from kernel
            meminfo.max_mem_used = ke_get_max_mem_usage();
            for (cursor = 0; cursor < KE_MEM_BLOCK_MAX ; cursor++)
            {
                meminfo.mem_used[cursor] = ke_get_mem_usage(cursor);
            }

            // Finally send indication to application that request memory information
            meminfo_msg = KE_MSG_ALLOC(GAPM_DBG_MEM_INFO_IND, src_id, dest_id, gapm_dbg_mem_info_ind);
            memcpy(meminfo_msg, &meminfo, sizeof(struct gapm_dbg_mem_info_ind));
            ke_msg_send(meminfo_msg);

            /* send command complete event with no error */
            gapm_send_error_evt(GAPM_DBG_GET_MEM_INFO, src_id, GAP_ERR_NO_ERROR);
            // restore GAPM state to idle
            gapm_update_state(GAPM_OP_CFG, false);

#else
            /* send command complete event with error */
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_NOT_SUPPORTED);
#endif /* (KE_PROFILING) */
        }
        break;
        // Get Suggested Default Data Length
        case GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN:
        {
            // Allocate message
            struct gapm_operation_cmd *get_sugg_data = KE_MSG_ALLOC(HCI_COMMAND, 0,
                    HCI_LE_RD_SUGGTED_DFT_DATA_LEN_CMD_OPCODE, gapm_operation_cmd);

            get_sugg_data->operation = GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN;
            /* send the message */
            hci_send_2_controller(get_sugg_data);
        }
        break;
        // Get Maximum LE Data Length
        case GAPM_GET_MAX_LE_DATA_LEN:
        {
            /* send the message */
            hci_basic_cmd_send_2_controller(HCI_LE_RD_MAX_DATA_LEN_CMD_OPCODE);
        }
        break;
        // Get white list size
        case GAPM_GET_WLIST_SIZE:
        {
            // Send the message
            hci_basic_cmd_send_2_controller(HCI_LE_RD_WLST_SIZE_CMD_OPCODE);
        }
        break;
        // Get periodic advertiser list size
        case GAPM_GET_PAL_SIZE:
        {
            // Send the message
            hci_basic_cmd_send_2_controller(HCI_LE_RD_PER_ADV_LIST_SIZE_CMD_OPCODE);
        }
        break;
        // Get resolving list size
        case GAPM_GET_RAL_SIZE:
        {
            // Send the message
            hci_basic_cmd_send_2_controller(HCI_LE_RD_RSLV_LIST_SIZE_CMD_OPCODE);
        }
        break;
        // Get available number of advertising sets
        case GAPM_GET_NB_ADV_SETS:
        {
            // Send the message
            hci_basic_cmd_send_2_controller(HCI_LE_RD_NB_SUPP_ADV_SETS_CMD_OPCODE);
        }
        break;
        // Get maximum advertising data length supported by controller
        case GAPM_GET_MAX_LE_ADV_DATA_LEN:
        {
            // Send the message
            hci_basic_cmd_send_2_controller(HCI_LE_RD_MAX_ADV_DATA_LEN_CMD_OPCODE);
        }
        break;
        // Get minimum and maximum transmit power level supported by the controller
        case GAPM_GET_DEV_TX_PWR:
        {
            // Send the message
            hci_basic_cmd_send_2_controller(HCI_LE_RD_TX_PWR_CMD_OPCODE);
        }
        break;
        // Get RF path compensation values
        case GAPM_GET_DEV_RF_PATH_COMP:
        {
            // Send the message
            hci_basic_cmd_send_2_controller(HCI_LE_RD_RF_PATH_COMP_CMD_OPCODE);
        }
        break;
        default:
        {
            // no need to check operation, this has been done by gapm_process_op
            /* nothing to do */
        }
        break;
        };
    }

    return msg_status;
}

#if (BLE_PROFILES)
/**
 ****************************************************************************************
 * @brief Handles request to add a new profile task.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
int gapm_profile_task_add_cmd_handler(ke_msg_id_t const msgid, struct gapm_profile_task_add_cmd *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_PROFILE_TASK_ADD, GAPM_NO_OP};
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        ke_task_id_t prf_task;
        uint8_t status;
#if(AHI_TL_SUPPORT)
        // for external application which AHI, update task Number from task ID
        if (KE_TYPE_GET(param->app_task) == TASK_ID_AHI)
        {
            param->app_task = gapm_get_task_from_id(param->app_task);
        }
#endif // (AHI_TL_SUPPORT)

        // request to add the profile
        status = prf_add_profile(param, &prf_task);

        if (status == GAP_ERR_NO_ERROR)
        {
            struct gapm_profile_added_ind *ind;
#if(AHI_TL_SUPPORT)
            // for external application which AHI, update task Number from task ID
            if (KE_TYPE_GET(param->app_task) == TASK_AHI)
            {
                prf_task = gapm_get_id_from_task(prf_task);
            }
#endif // (AHI_TL_SUPPORT)

            // send an indication to inform that profile has been added
            ind = KE_MSG_ALLOC(GAPM_PROFILE_ADDED_IND, src_id, dest_id, gapm_profile_added_ind);
            ind->prf_task_id = param->prf_task_id;
            ind->prf_task_nb = prf_task;
            ind->start_hdl   = param->start_hdl;
            ke_msg_send(ind);
        }

        gapm_send_complete_evt(GAPM_OP_CFG, status);
    }

    return msg_status;
}
#endif // (BLE_PROFILES)


/**
 ****************************************************************************************
 * @brief Handles request of changing the IRK
 * - GAPM_SET_IRK:  Set IRK
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
int gapm_set_irk_cmd_handler(ke_msg_id_t const msgid, struct gapm_set_irk_cmd *param,
                             ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_SET_IRK, GAPM_NO_OP};

    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_AIR, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // Set device Identity key (IRK)
        memcpy(&(gapm_env.irk), &(param->irk), sizeof(struct gap_sec_key));

        // Send complete event
        gapm_send_complete_evt(GAPM_OP_AIR, GAP_ERR_NO_ERROR);
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles request of registering LE Protocol multiplexer
 * - GAPM_LEPSM_REG: Register a LE Protocol/Service Multiplexer
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
int gapm_lepsm_register_cmd_handler(ke_msg_id_t const msgid, struct gapm_lepsm_register_cmd *param,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

#if (BLE_LECB)

    if (0 == gapm_env.max_nb_lecb)
    {
        gapm_env.max_nb_lecb = 1;
    }
#endif // (BLE_LECB)
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_LEPSM_REG, GAPM_NO_OP};

    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        uint8_t status = GAP_ERR_INVALID_PARAM;
#if (BLE_LECB)

        // 1. search if LE_PSM is present.
        struct gapm_le_psm_info *info = gapm_le_psm_find(param->le_psm);

        // check that LE_PSM found
        if ((info == NULL) && (KE_TYPE_GET(param->app_task) != TASK_NONE))
        {
#if(AHI_TL_SUPPORT)
            // for external application which AHI, update task Number from task ID
            if (KE_TYPE_GET(param->app_task) == TASK_ID_AHI)
            {
                param->app_task = gapm_get_task_from_id(param->app_task);
            }
#endif // (AHI_TL_SUPPORT)

            // 2. allocate a data structure for new LE_PSM registered.
            info = (struct gapm_le_psm_info *)ke_malloc(sizeof(struct gapm_le_psm_info), KE_MEM_ATT_DB);

            // 3. fill structure and put it in LE_PSM registered list.
            info->le_psm    = param->le_psm;
            info->sec_lvl   = param->sec_lvl;
            info->task_id   = KE_TYPE_GET(param->app_task);
            info->nb_est_lk = 0;

            // 4. put LE_PSM info in registered list
            co_list_push_back(&(gapm_env.reg_le_psm), &(info->hdr));

            status = GAP_ERR_NO_ERROR;
        }

#else // !(BLE_LECB)
        status = GAP_ERR_COMMAND_DISALLOWED;
#endif // (BLE_LECB)
        // Send complete event
        gapm_send_complete_evt(GAPM_OP_CFG, status);
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles request of unregistering LE Protocol multiplexer
 * - GAPM_LEPSM_UNREG: Unregister a LE Protocol/Service Multiplexer
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
int gapm_lepsm_unregister_cmd_handler(ke_msg_id_t const msgid, struct gapm_lepsm_unregister_cmd *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_LEPSM_UNREG, GAPM_NO_OP};

    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        uint8_t status = GAP_ERR_NO_ERROR;
#if (BLE_LECB)

        // 1. search if LE_PSM is present.
        struct gapm_le_psm_info *info = gapm_le_psm_find(param->le_psm);

        if (info == NULL)
        {
            status = L2C_ERR_LEPSM_NOT_SUPP;
        }
        // 2. If present remove it from list
        else if (info->nb_est_lk == 0)
        {
            // remove registered LE_PSM from list
            co_list_extract(&(gapm_env.reg_le_psm), &(info->hdr));
            ke_free(info);
        }
        else
        {
            status = GAP_ERR_COMMAND_DISALLOWED;
        }

#else // !(BLE_LECB)
        status = GAP_ERR_NOT_SUPPORTED;
#endif // (BLE_LECB)
        // Send complete event
        gapm_send_complete_evt(GAPM_OP_CFG, status);
    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles request of controlling test mode
 *  - GAPM_LE_TEST_STOP: Unregister a LE Protocol/Service Multiplexer
 *  - GAPM_LE_TEST_RX_START: Start RX Test Mode
 *  - GAPM_LE_TEST_TX_START: Start TX Test Mode
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
int gapm_le_test_mode_ctrl_cmd_handler(ke_msg_id_t const msgid, struct gapm_le_test_mode_ctrl_cmd *param,
                                       ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_LE_TEST_STOP, GAPM_LE_TEST_RX_START, GAPM_LE_TEST_TX_START, GAPM_NO_OP};

    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        switch (param->operation)
        {
        case GAPM_LE_TEST_STOP:
        {
            /* Request end of test mode */
            hci_basic_cmd_send_2_controller(HCI_LE_TEST_END_CMD_OPCODE);
        }
        break;

        case GAPM_LE_TEST_RX_START:
        {
            // allocate Start RX test mode command
            struct hci_le_rx_test_v2_cmd *rx_test
                = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_RX_TEST_V2_CMD_OPCODE, hci_le_rx_test_v2_cmd);

            rx_test->rx_channel    = param->channel;
            rx_test->phy           = param->phy;
            rx_test->mod_idx       = param->modulation_idx;

            // message send */
            hci_send_2_controller(rx_test);
        }
        break;
        case GAPM_LE_TEST_TX_START:
        {
            // allocate Start TX test mode command
            struct hci_le_tx_test_v2_cmd *tx_test
                = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_TX_TEST_V2_CMD_OPCODE, hci_le_tx_test_v2_cmd);

            tx_test->tx_channel     = param->channel;
            tx_test->test_data_len  = param->tx_data_length;
            tx_test->pkt_payl       = param->tx_pkt_payload;
            tx_test->phy            = param->phy;

            // message send */
            hci_send_2_controller(tx_test);
        }
        break;
        default:
        {
            ASSERT_INFO(0, param->operation, 0);
        }
        break;
        }
    }

    return msg_status;
}


/*
 * HCI EVENT HANDLERS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles common complete event for configuration and reset purpose.
 * Send the complete event.
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_basic_cmd_cmp_evt_cfg_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *event)
{
    int ret = KE_MSG_CONSUMED;
    switch (opcode)
    {
    // LL reset completed
    case HCI_RESET_CMD_OPCODE:
    {
        extern void hci_tl_packed_data(struct ke_msg * msg, uint8_t log_on);

        gapm_op_reset_continue(GAPM_OP_RESET_HCI, RW_ERR_HCI_TO_HL(event->status));
#if 0
        hci_tl_packed_data(ke_param2msg((void *)event), 0);
        ke_msg_forward(event, TASK_BT, opcode);
        ret = KE_MSG_SAVED;
#endif
    }
    break;

    case HCI_SET_EVT_MASK_CMD_OPCODE:
    {
        gapm_op_reset_continue(GAPM_OP_RESET_SET_EVT_MASK, RW_ERR_HCI_TO_HL(event->status));
    }
    break;

    case HCI_LE_SET_EVT_MASK_CMD_OPCODE:
    {
        gapm_op_reset_continue(GAPM_OP_RESET_LE_SET_EVT_MASK, RW_ERR_HCI_TO_HL(event->status));
    }
    break;

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    // Set Host channel map completed
    case HCI_LE_SET_HOST_CH_CLASS_CMD_OPCODE:
    {
        // finish operation execution with given status code
        gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(event->status));
    }
    break;
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

    // Set suggested default data length completed
    case HCI_LE_WR_SUGGTED_DFT_DATA_LEN_CMD_OPCODE:
    {
        gapm_op_setup_continue(GAPM_OP_SETUP_WR_LE_DFT_DATA_LEN_CMD, RW_ERR_HCI_TO_HL(event->status));
    }
    break;

    // Set default PHY  command completed
    case HCI_LE_SET_DFT_PHY_CMD_OPCODE:
    {
        gapm_op_setup_continue(GAPM_OP_SETUP_SET_LE_DFT_PHY_CMD, RW_ERR_HCI_TO_HL(event->status));
    }
    break;

    // Set RF path compensation values command completed
    case HCI_LE_WR_RF_PATH_COMP_CMD_OPCODE:
    {
        gapm_op_setup_continue(GAPM_OP_SETUP_WR_RF_PATH_COMP_CMD, RW_ERR_HCI_TO_HL(event->status));
    }
    break;
    // Test mode control
    case HCI_LE_RX_TEST_V2_CMD_OPCODE:
    case HCI_LE_TX_TEST_V2_CMD_OPCODE:
    {
        // finish operation execution with given status code
        gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(event->status));
    }
    break;

    default: /* Nothing to do */
        break;
    } /* end of switch */

    /* message is consumed */
    return (ret);
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Handles LE read buffer size command complete event.
 * Used to read the maximum size of the data portion of HCI LE ACL Data Packets sent
 * from the Host to the Controller.
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_rd_buff_size_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_buf_size_cmd_cmp_evt const *event)
{
    //ASSERT_ERR(event->status == CO_ERROR_NO_ERROR);

    if (event->status == CO_ERROR_NO_ERROR)
    {
        /* update the buffer size */
        l2cm_set_link_layer_buff_size(event->hc_data_pk_len, event->hc_tot_nb_data_pkts);
    }

    gapm_op_reset_continue(GAPM_OP_RESET_LE_RD_BUFF_SIZE, RW_ERR_HCI_TO_HL(event->status));


    // message is consumed
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles read buffer size command complete event.
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_rd_buff_size_cmd_cmp_evt_handler(uint16_t opcode, struct hci_rd_buf_size_cmd_cmp_evt const *event)
{
    ASSERT_ERR(event->status == CO_ERROR_NO_ERROR);

    // sanity check
    ASSERT_ERR(event->hc_tot_nb_data_pkts != 0);

    if (event->status == CO_ERROR_NO_ERROR)
    {
        /* update the buffer size */
        l2cm_set_link_layer_buff_size(event->hc_data_pk_len, event->hc_tot_nb_data_pkts);
    }

    gapm_op_reset_continue(GAPM_OP_RESET_RD_BUFF_SIZE, RW_ERR_HCI_TO_HL(event->status));

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}
#endif // #if (BLE_CENTRAL || BLE_PERIPHERAL)





/**
 ****************************************************************************************
 * @brief Handles the read Bluetooth device version complete event.
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_rd_local_ver_info_cmd_cmp_evt_handler(uint16_t opcode, struct hci_rd_local_ver_info_cmd_cmp_evt const *event)
{
    // Current process state
    uint8_t state = ke_state_get(TASK_GAPM);
    if (state != GAPM_DEVICE_SETUP)
    {
        // check if there is no protocol issues.
        if (gapm_get_operation(GAPM_OP_CFG) != GAPM_GET_DEV_VERSION)
        {
            // finish operation execution with an error: unexpected response received
            gapm_send_complete_evt(GAPM_OP_CFG, GAP_ERR_PROTOCOL_PROBLEM);
        }
        else
        {
            /* status should be OK */
            ASSERT_ERR(event->status == CO_ERROR_NO_ERROR);

            // send device version value
            if (event->status == CO_ERROR_NO_ERROR)
            {
                struct gapm_dev_version_ind *version_ind =
                    KE_MSG_ALLOC(GAPM_DEV_VERSION_IND, gapm_get_requester(GAPM_OP_CFG),
                                 TASK_GAPM, gapm_dev_version_ind);
                /* fill up the parameters */
                /* HCI version */
                version_ind->hci_ver = event->hci_ver;
                /* HCI sub version */
                version_ind->hci_subver = event->hci_rev;
                /* LMP sub version */
                version_ind->lmp_subver = event->lmp_subver;
                /* LMP version */
                version_ind->lmp_ver = event->lmp_ver;
                /* Manufacturing name */
                version_ind->manuf_name = event->manuf_name;
                /* Host version */
                version_ind->host_ver = RWBT_SW_VERSION_MAJOR;
                /* Host sub version */
                version_ind->host_subver = CO_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR,
                                           RWBT_SW_VERSION_BUILD);

                /* send the message indication */
                ke_msg_send(version_ind);
            }

            // finish operation execution
            gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(event->status));
        }
    }
    /* message is consumed */
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles the read Bluetooth device address complete event.
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_rd_bd_addr_cmd_cmp_evt_handler(uint16_t opcode, struct hci_rd_bd_addr_cmd_cmp_evt const *event)
{
    /* status should be OK */
    ASSERT_ERR(event->status == CO_ERROR_NO_ERROR);

    /* Store Local BD address */
    memcpy(&(gapm_env.addr), &(event->local_addr), BD_ADDR_LEN);

    if (gapm_get_operation(GAPM_OP_CFG) == GAPM_SET_DEV_CONFIG)
    {
        gapm_op_setup_continue(GAPM_OP_SETUP_RD_PUBLIC_ADDR_MGT, RW_ERR_HCI_TO_HL(event->status));
    }
    else
    {
        gapm_op_reset_continue(GAPM_OP_RESET_RD_BD_ADDR, RW_ERR_HCI_TO_HL(event->status));
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

#if ((BLE_OBSERVER) || (BLE_PERIPHERAL))

/**
 ****************************************************************************************
 * @brief Handles LE read adv tx power level value.
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_rd_adv_chnl_tx_pw_cmd_cmp_evt_handler(uint16_t opcode, struct hci_rd_adv_chnl_tx_pw_cmd_cmp_evt const *event)
{
    uint8_t status = GAP_ERR_PROTOCOL_PROBLEM;

    // sanity check
    if (gapm_get_operation(GAPM_OP_CFG) == GAPM_GET_DEV_ADV_TX_POWER)
    {
        if ((event->status == CO_ERROR_NO_ERROR))
        {
            // send event to upper layers.
            struct gapm_dev_adv_tx_power_ind *ind = KE_MSG_ALLOC(GAPM_DEV_ADV_TX_POWER_IND,
                                                    gapm_get_requester(GAPM_OP_CFG),  TASK_GAPM, gapm_dev_adv_tx_power_ind);

            ind->power_lvl = event->adv_tx_pw_lvl;

            /* send the indication */
            ke_msg_send(ind);
            status = GAP_ERR_NO_ERROR;
        }
        else
        {
            status = RW_ERR_HCI_TO_HL(event->status);
        }
    }

    gapm_send_complete_evt(GAPM_OP_CFG, status);

    return (KE_MSG_CONSUMED);
}
#endif // ((BLE_OBSERVER) || (BLE_PERIPHERAL))



/**
 ****************************************************************************************
 * @brief LE Read Suggested Default LE Data Length complete event handler.
  *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_rd_suggted_dft_data_len_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_suggted_dft_data_len_cmd_cmp_evt const *event)
{
    // Check if a command is currently handled, drop the message if not the case.
    if (ke_state_get(TASK_GAPM) != GAPM_DEVICE_SETUP)
    {
        // Send indication to the APP
        struct gapm_sugg_dflt_data_len_ind *suggted_data = KE_MSG_ALLOC(GAPM_SUGG_DFLT_DATA_LEN_IND,
                gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
                gapm_sugg_dflt_data_len_ind);

        // Fill parameters
        suggted_data->suggted_max_tx_octets = event->suggted_max_tx_octets;
        suggted_data->suggted_max_tx_time = event->suggted_max_tx_time;

        // Send the message
        ke_msg_send(suggted_data);

        // Send complete event
        gapm_send_complete_evt(GAPM_OP_CFG, event->status);
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief LE Read Maximum LE Data Lenght complete event handler.
  *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_rd_max_data_len_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_rd_max_data_len_cmd_cmp_evt const *event)
{
    // Check if a command is currently handled, drop the message if not the case.
    if (ke_state_get(TASK_GAPM) != GAPM_DEVICE_SETUP)
    {
        // Send indication to the APP
        struct gapm_max_data_len_ind *max_data = KE_MSG_ALLOC(GAPM_MAX_DATA_LEN_IND,
                gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
                gapm_max_data_len_ind);

        // Fill parameters
        max_data->suppted_max_rx_octets = event->suppted_max_rx_octets;
        max_data->suppted_max_rx_time = event->suppted_max_rx_time;
        max_data->suppted_max_tx_octets = event->suppted_max_tx_octets;
        max_data->suppted_max_tx_time = event->suppted_max_tx_time;

        // Send the message
        ke_msg_send(max_data);

        // Send complete event
        gapm_send_complete_evt(GAPM_OP_CFG, event->status);
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles common complete event for resolving list management purpose.
 * Send the complete event.
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_basic_cmd_cmp_evt_rl_cfg_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *event)
{
    // Current process state
    uint8_t state = ke_state_get(TASK_GAPM);
    uint8_t status = GAP_ERR_PROTOCOL_PROBLEM;
    bool op_finished = true;

    if (state != GAPM_DEVICE_SETUP)
    {
        switch (opcode)
        {
        case HCI_LE_SET_ADDR_RESOL_EN_CMD_OPCODE:
        {
            gapm_op_setup_continue(GAPM_OP_SETUP_EN_CTRL_PRIV, RW_ERR_HCI_TO_HL(event->status));

            // Continue setting up device
            op_finished = false;
        }
        break;

        case HCI_LE_SET_RSLV_PRIV_ADDR_TO_CMD_OPCODE:
        {
            gapm_op_setup_continue(GAPM_OP_SETUP_SET_RENEW_TO, RW_ERR_HCI_TO_HL(event->status));

            // Continue setting up device
            op_finished = false;
        }
        break;

        default:
        {
            /* Nothing to do */
        }
        break;
        } /* end of switch */
    }

    // check that operation not rescheduled
    if (op_finished)
    {
        // send command complete
        gapm_send_complete_evt(GAPM_OP_CFG, status);
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

#if (BLE_MESH || BLE_CENTRAL || BLE_PERIPHERAL)
#if (SECURE_CONNECTIONS)
/**
 ****************************************************************************************
 * @brief Handles the LE read local public key complete event.
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] cmp_evt   Pointer to the parameters of the event.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int hci_le_rd_local_p256_public_key_cmp_evt_handler(uint16_t opcode, struct hci_rd_local_p256_public_key_cmp_evt const *cmp_evt)
{
    if (cmp_evt->status == CO_ERROR_NO_ERROR)
    {
        // Current process state
        memcpy(&gapm_env.public_key.x[0], &cmp_evt->public_key[0], 32);
        memcpy(&gapm_env.public_key.y[0], &cmp_evt->public_key[32], 32);

        if (rwip_param.set(PARAM_ID_LE_PUBLIC_KEY_P256, PARAM_LEN_PUBLIC_KEY_P256, (uint8_t *)&cmp_evt->public_key[0])  == PARAM_OK)
        {
            // Write successful, notify upper layer
            struct gapm_pub_key_ind *ind = KE_MSG_ALLOC(GAPM_PUB_KEY_IND, gapm_get_AHI_task_id(), TASK_GAPM,
                                           gapm_pub_key_ind);

            memcpy(&ind->pub_key_x, &cmp_evt->public_key[0],  32);
            memcpy(&ind->pub_key_y, &cmp_evt->public_key[32], 32);
            ke_msg_send(ind);
        }
        else
        {
            // Could not write new secret key to NVDS
            // Public key value sent to the requester
        }
    }

    // Check if public key requested by an application
    if (gapm_get_operation(GAPM_OP_DHKEY) == GAPM_GET_PUB_KEY)
    {
        if (cmp_evt->status == CO_ERROR_NO_ERROR)
        {
            // Public key value sent to the requester
            struct gapm_pub_key_ind *ind = KE_MSG_ALLOC(GAPM_PUB_KEY_IND, gapm_get_requester(GAPM_OP_DHKEY), TASK_GAPM,
                                           gapm_pub_key_ind);

            memcpy(&ind->pub_key_x, &cmp_evt->public_key[0],  32);
            memcpy(&ind->pub_key_y, &cmp_evt->public_key[32], 32);
            ke_msg_send(ind);
        }

        gapm_send_complete_evt(GAPM_OP_DHKEY, cmp_evt->status);
    }
    // Or part of reset process
    else
    {
        gapm_op_setup_continue(GAPM_OP_SETUP_RD_PRIV_KEY, RW_ERR_HCI_TO_HL(cmp_evt->status));
    }

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}
#endif // (SECURE_CONNECTIONS)
#endif // (BLE_MESH || BLE_CENTRAL || BLE_PERIPHERAL)

#if (BLE_BROADCASTER)
/**
 ****************************************************************************************
 * @brief Handle reception of HCI Read Maximum Advertising Data Length command complete
 * event.
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_read_max_adv_data_len_cmd_cmp_evt_handler(uint16_t opcode,
        struct hci_le_rd_max_adv_data_len_cmd_cmp_evt const *p_event)
{
    // Retrieve GAPM task current state
    ke_state_t state = ke_state_get(TASK_GAPM);

    if (state == GAPM_DEVICE_SETUP)
    {
        if (p_event->status == CO_ERROR_NO_ERROR)
        {
            // Keep the read length in mind, should not changed
            gapm_env.max_adv_data_len = p_event->max_adv_data_len;
        }

        // Continue reset procedure
        gapm_op_reset_continue(GAPM_OP_RESET_RD_MAX_ADV_DATA_LEN, RW_ERR_HCI_TO_HL(p_event->status));
    }
    else
    {
        // Check if read command has been triggered by application
        if (gapm_get_operation(GAPM_OP_CFG) == GAPM_GET_MAX_LE_ADV_DATA_LEN)
        {
            if (p_event->status == CO_ERROR_NO_ERROR)
            {
                // Send a GAPM_MAX_ADV_LEN_IND to the application
                gapm_cfg_send_max_adv_data_len_ind();
            }

            // Send complete event
            gapm_send_complete_evt(GAPM_OP_CFG, p_event->status);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI Read Number of Supported Advertising Sets command complete
 * event. The read value may change over time.
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt_handler(uint16_t opcode,
        struct hci_le_rd_nb_supp_adv_sets_cmd_cmp_evt const *p_event)
{
    // Retrieve GAPM task current state
    ke_state_t state = ke_state_get(TASK_GAPM);

    if (state == GAPM_DEVICE_SETUP)
    {
        if (p_event->status == CO_ERROR_NO_ERROR)
        {
            // Keep the value in mind
            gapm_env.max_adv_set = p_event->nb_supp_adv_sets;
        }

        // Continue reset procedure
        gapm_op_reset_continue(GAPM_OP_RESET_RD_NB_ADV_SETS, RW_ERR_HCI_TO_HL(p_event->status));
    }
    else
    {
        // Check if read command has been triggered by application
        if (gapm_get_operation(GAPM_OP_CFG) == GAPM_GET_NB_ADV_SETS)
        {
            if (p_event->status == CO_ERROR_NO_ERROR)
            {
                gapm_cfg_send_nb_adv_sets_ind(p_event->nb_supp_adv_sets);
            }

            // Send complete event
            gapm_send_complete_evt(GAPM_OP_CFG, p_event->status);
        }
    }

    return (KE_MSG_CONSUMED);
}
#endif //(BLE_BROADCASTER)

/**
 ****************************************************************************************
 * @brief Handle reception of HCI Read Transmit Power command complete event.
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_rd_tx_pwr_cmd_cmp_evt_handler(uint16_t opcode,
        struct hci_le_rd_tx_pwr_cmd_cmp_evt const *p_event)
{
    // Check GAPM task current state
    if (ke_state_get(TASK_GAPM) != GAPM_DEVICE_SETUP)
    {
        // Check if read command has been triggered by application
        if (gapm_get_operation(GAPM_OP_CFG) == GAPM_GET_DEV_TX_PWR)
        {
            if (p_event->status == CO_ERROR_NO_ERROR)
            {
                // Send GAPM_DEV_TX_PWR_IND message to the application
                gapm_cfg_send_tx_pwr_ind(p_event->min_tx_pwr, p_event->max_tx_pwr);
            }

            // Send complete event
            gapm_send_complete_evt(GAPM_OP_CFG, p_event->status);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handle reception of HCI Read RF Path Compensation command complete event.
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_rd_rf_path_comp_cmd_cmp_evt_handler(uint16_t opcode,
        struct hci_le_rd_rf_path_comp_cmd_cmp_evt const *p_event)
{
    // Check GAPM task current state
    if (ke_state_get(TASK_GAPM) != GAPM_DEVICE_SETUP)
    {
        // Check if read command has been triggered by application
        if (gapm_get_operation(GAPM_OP_CFG) == GAPM_GET_DEV_RF_PATH_COMP)
        {
            if (p_event->status == CO_ERROR_NO_ERROR)
            {
                // Send GAPM_DEV_RF_PATH_COMP_IND message to the application
                gapm_cfg_send_rf_path_comp_ind(p_event->tx_path_comp, p_event->rx_path_comp);
            }

            // Send complete event
            gapm_send_complete_evt(GAPM_OP_CFG, p_event->status);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles Test mode end command complete event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_test_end_cmd_cmp_evt_handler(uint16_t opcode, struct hci_test_end_cmd_cmp_evt const *p_event)
{
    // send number of packet received only if no error and number of packet received > 0
    if ((p_event->status == CO_ERROR_NO_ERROR) && (p_event->nb_packet_received > 0))
    {
        struct gapm_le_test_end_ind *ind = KE_MSG_ALLOC(GAPM_LE_TEST_END_IND,
                                           gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
                                           gapm_le_test_end_ind);
        ind->nb_packet_received = p_event->nb_packet_received;
        ke_msg_send(ind);
    }

    // finish operation execution with given status code
    gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(p_event->status));

    /* message is consumed */
    return (KE_MSG_CONSUMED);
}

/// @} GAPM_CFG
