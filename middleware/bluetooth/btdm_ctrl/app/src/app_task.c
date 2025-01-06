/**
 ****************************************************************************************
 *
 * @file appm_task.c
 *
 * @brief RW APP Task implementation
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APPTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"          // SW configuration

#if (BLE_APP_PRESENT)

#include "app_task.h"             // Application Manager Task API
#include "app.h"                  // Application Manager Definition
#include "gapc_task.h"            // GAP Controller Task API
#include "gapm_task.h"            // GAP Manager Task API
#include "arch.h"                 // Platform Definitions
#include "gattc_task.h"
#include <string.h>
#include "co_utils.h"
#include "ke_timer.h"             // Kernel timer

#if (BLE_APP_SEC)
    #include "app_sec.h"              // Security Module Definition
#endif //(BLE_APP_SEC)

#if (BLE_APP_HT)
    #include "app_ht.h"               // Health Thermometer Module Definition
    #include "htpt_task.h"
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
    #include "app_dis.h"              // Device Information Module Definition
    #include "diss_task.h"
#endif //(BLE_APP_DIS)


#if (BLE_APP_SIBLE)
    #include "app_sible.h"              // SiFli BLE Module Definition
    #include "sibles_task.h"
#endif //(BLE_APP_SIBLE)


#if (BLE_APP_BATT)
    #include "app_batt.h"             // Battery Module Definition
    #include "bass_task.h"
#endif //(BLE_APP_BATT)

#if (BLE_APP_HID)
    #include "app_hid.h"              // HID Module Definition
    #include "hogpd_task.h"
#endif //(BLE_APP_HID)

#if (BLE_APP_AM0)
    #include "app_am0.h"             // Audio Mode 0 Application
#endif //(BLE_APP_AM0)

#if (DISPLAY_SUPPORT)
    #include "app_display.h"          // Application Display Definition
#endif //(DISPLAY_SUPPORT)

#if (NVDS_SUPPORT)
    #define APP_NVDS_SUPPORT  1
#else
    #define APP_NVDS_SUPPORT  0
#endif

#undef NVDS_SUPPORT
/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static uint8_t app_get_handler(const struct app_subtask_handlers *handler_list_desc,
                               ke_msg_id_t msgid,
                               void *param,
                               ke_task_id_t src_id)
{
    // Counter
    uint8_t counter;

    // Get the message handler function by parsing the message table
    for (counter = handler_list_desc->msg_cnt; 0 < counter; counter--)
    {
        struct ke_msg_handler handler
            = (*(handler_list_desc->p_msg_handler_tab + counter - 1));

        if ((handler.id == msgid) ||
                (handler.id == KE_MSG_DEFAULT_HANDLER))
        {
            // If handler is NULL, message should not have been received in this state
            ASSERT_ERR(handler.func);

            return (uint8_t)(handler.func(msgid, param, TASK_APP, src_id));
        }
    }

    // If we are here no handler has been found, drop the message
    return (KE_MSG_CONSUMED);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles GAPM_ACTIVITY_CREATED_IND event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_activity_created_ind_handler(ke_msg_id_t const msgid,
        struct gapm_activity_created_ind const *p_param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    if (app_env.adv_state == APP_ADV_STATE_CREATING)
    {
        // Store the advertising activity index
        app_env.adv_actv_idx = p_param->actv_idx;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAPM_ACTIVITY_STOPPED_IND event.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_activity_stopped_ind_handler(ke_msg_id_t const msgid,
        struct gapm_activity_stopped_ind const *p_param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    if (app_env.adv_state == APP_ADV_STATE_STARTED)
    {
        // Act as if activity had been stopped by the application
        app_env.adv_state = APP_ADV_STATE_STOPPING;

        // Perform next operation
        appm_adv_fsm_next();
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAPM_PROFILE_ADDED_IND event
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_profile_added_ind_handler(ke_msg_id_t const msgid,
        struct gapm_profile_added_ind *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Current State
    ke_state_t state = ke_state_get(dest_id);

    if (state == APPM_CREATE_DB)
    {
        switch (param->prf_task_id)
        {
#if (BLE_APP_AM0)
        case (TASK_ID_AM0_HAS):
        {
            app_am0_set_prf_task(param->prf_task_nb);
        }
        break;
#endif //(BLE_APP_AM0)

        default: /* Nothing to do */
            break;
        }
    }
    else
    {
        ASSERT_INFO(0, state, src_id);
    }

    return KE_MSG_CONSUMED;
}

#ifndef BSP_USING_PC_SIMULATOR
    __WEAK
#endif
uint16_t ble_max_mtu_get()
{
    uint16_t mtu = 1024;
    return mtu;
}

/**
 ****************************************************************************************
 * @brief Handles GAP manager command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
#if (NVDS_SUPPORT)
    uint8_t key_len = KEY_LEN;
#endif //(NVDS_SUPPORT)

    switch (param->operation)
    {
    // Reset completed
    case (GAPM_RESET):
    {

        if (param->status == GAP_ERR_NO_ERROR)
        {
#if (APP_NVDS_SUPPORT)
            nvds_tag_len_t len;
#endif //(NVDS_SUPPORT)
#if (BLE_APP_HID)
            app_hid_start_mouse();
#endif //(BLE_APP_HID)

            // Set Device configuration
            struct gapm_set_dev_config_cmd *cmd = KE_MSG_ALLOC(GAPM_SET_DEV_CONFIG_CMD,
                                                  TASK_GAPM, TASK_APP,
                                                  gapm_set_dev_config_cmd);
            // Set the operation
            cmd->operation = GAPM_SET_DEV_CONFIG;
            // Set the device role - Peripheral

            cmd->role      = GAP_ROLE_NONE;
#if (BLE_BROADCASTER)
            cmd->role |= GAP_ROLE_BROADCASTER;
#endif // (BLE_BROADCASTER)

#if (BLE_OBSERVER)
            cmd->role |= GAP_ROLE_OBSERVER;
#endif // (BLE_OBSERVER)

#if (BLE_CENTRAL)
            cmd->role |= GAP_ROLE_CENTRAL;
#endif // (BLE_CENTRAL)

#if (BLE_PERIPHERAL)
            cmd->role |= GAP_ROLE_PERIPHERAL;
#endif // (BLE_PERIPHERAL)



#if (BLE_APP_SEC_CON)
            // The Max MTU is increased to support the Public Key exchange
            // HOWEVER, with secure connections enabled you cannot sniff the
            // LEAP and LEAS protocols
            cmd->max_mtu = ble_max_mtu_get();
            cmd->pairing_mode = GAPM_PAIRING_SEC_CON | GAPM_PAIRING_LEGACY;
            uint8_t p256_key[PARAM_LEN_PUBLIC_KEY_P256];
            uint8_t p256_key_len = PARAM_LEN_PUBLIC_KEY_P256;
#if (APP_NVDS_SUPPORT)
            if (sifli_nvds_get(PARAM_ID_LE_PUBLIC_KEY_P256, &p256_key_len, (uint8_t *)p256_key) != NVDS_OK)
#endif
                cmd->pairing_mode |= GAPM_PAIRING_FORCE_P256_KEY_GEN;

#else // !(BLE_APP_SEC_CON)
            // Do not support secure connections
            cmd->pairing_mode = GAPM_PAIRING_LEGACY;
#endif //(BLE_APP_SEC_CON)

            // Set Data length parameters
            cmd->sugg_max_tx_octets = 251;//BLE_MIN_OCTETS; //TODO
            cmd->sugg_max_tx_time   = 2120;//BLE_MIN_TIME; //TODO

#if (BLE_APP_HID)
            // Enable Slave Preferred Connection Parameters present
            cmd->att_cfg = GAPM_MASK_ATT_SLV_PREF_CON_PAR_EN;
#endif //(BLE_APP_HID)

            // Host privacy enabled by default
            cmd->privacy_cfg = GAPM_PRIV_CFG_PRIV_EN_BIT;
            cmd->renew_dur = 300;


#if (APP_NVDS_SUPPORT)
            if (sifli_nvds_get(NVDS_TAG_BD_ADDRESS, &len, &cmd->addr.addr[0]) == NVDS_OK)
            {
                // Check if address is a static random address
                if (cmd->addr.addr[5] & 0xC0)
                {
                    // Host privacy enabled by default
                    cmd->privacy_cfg |= GAPM_PRIV_CFG_PRIV_ADDR_BIT;
                }
            }
#endif //(NVDS_SUPPORT)

#if (BLE_APP_AM0)
            cmd->audio_cfg   = GAPM_MASK_AUDIO_AM0_SUP;
            cmd->att_cfg    |= GAPM_MASK_ATT_SVC_CHG_EN;
#endif //(BLE_APP_AM0)
            cmd->att_cfg    |= GAPM_MASK_ATT_SVC_CHG_EN;


#if (NVDS_SUPPORT)
            if ((sifli_nvds_get(NVDS_TAG_LOC_IRK, &key_len, app_env.loc_irk) == NVDS_OK))
            {
                memcpy(cmd->irk.key, app_env.loc_irk, 16);
            }
            else
#endif //(NVDS_SUPPORT)
            {
                memset((void *)&cmd->irk.key[0], 0x00, KEY_LEN);
            }
            // Send message
            ke_msg_send(cmd);
        }
        else
        {
            // No need assert here.
            //ASSERT_ERR(0);
        }
    }
    break;

#if (BLE_APP_SEC)
    case (GAPM_PROFILE_TASK_ADD):
    {
#if (NVDS_SUPPORT)
        // If Bonded retrieve the local IRK from NVDS
        if (sifli_nvds_get(NVDS_TAG_LOC_IRK, &key_len, app_env.loc_irk) == NVDS_OK)
        {
            // Set the IRK in the GAP
            struct gapm_set_irk_cmd *cmd = KE_MSG_ALLOC(GAPM_SET_IRK_CMD,
                                           TASK_GAPM, TASK_APP,
                                           gapm_set_irk_cmd);
            ///  - GAPM_SET_IRK:
            cmd->operation = GAPM_SET_IRK;
            memcpy(&cmd->irk.key[0], &app_env.loc_irk[0], KEY_LEN);
            ke_msg_send(cmd);
        }
        else
#endif //(NVDS_SUPPORT)
        {
            struct gapm_set_irk_cmd *cmd = KE_MSG_ALLOC(GAPM_SET_IRK_CMD,
                                           TASK_GAPM, TASK_APP,
                                           gapm_set_irk_cmd);

            memset((void *)&cmd->irk.key[0], 0x00, KEY_LEN);
            cmd->operation = GAPM_SET_IRK;
            ke_msg_send(cmd);

            // If cannot read IRK from NVDS ASSERT
            //ASSERT_ERR(0);
        }
    }
    break;
#endif

    case (GAPM_GEN_RAND_NB) :
    {
        if (app_env.rand_cnt == 1)
        {
            // Generate a second random number
            app_env.rand_cnt++;
            struct gapm_gen_rand_nb_cmd *cmd = KE_MSG_ALLOC(GAPM_GEN_RAND_NB_CMD,
                                               TASK_GAPM, TASK_APP,
                                               gapm_gen_rand_nb_cmd);
            cmd->operation = GAPM_GEN_RAND_NB;
            ke_msg_send(cmd);
        }
        else
        {
            struct gapm_set_irk_cmd *cmd = KE_MSG_ALLOC(GAPM_SET_IRK_CMD,
                                           TASK_GAPM, TASK_APP,
                                           gapm_set_irk_cmd);
            app_env.rand_cnt = 0;
            ///  - GAPM_SET_IRK
            cmd->operation = GAPM_SET_IRK;
            memcpy(&cmd->irk.key[0], &app_env.loc_irk[0], KEY_LEN);
            ke_msg_send(cmd);
        }
    }
    break;

#if (BLE_APP_SEC)
    case (GAPM_SET_IRK):
    {
        // ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->operation, param->status);

        // If not Bonded already store the generated value in NVDS
#if (NVDS_SUPPORT)
        if (sifli_nvds_put(NVDS_TAG_LOC_IRK, KEY_LEN, (uint8_t *)&app_env.loc_irk) != NVDS_OK)
#endif //(NVDS_SUPPORT)
        {
            //ASSERT_INFO(0, 0, 0);
        }

        app_env.rand_cnt = 0;
        // Add the next requested service
        if (!appm_add_svc())
        {
            // Go to the ready state
            ke_state_set(TASK_APP, APPM_READY);
#if 0
            // No more service to add, start advertising
            if (app_env.start_adv)
                appm_update_adv_state(true);
#endif
        }
    }
    break;

#endif /* BLE_APP_SEC  */
    // Device Configuration updated
    case (GAPM_SET_DEV_CONFIG):
    {
//#ifndef RWIP_BLE_4
//        ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->operation, param->status);
//#endif

        // Go to the create db state
        ke_state_set(TASK_APP, APPM_CREATE_DB);

        // Add the first required service in the database
        // and wait for the PROFILE_ADDED_IND
        appm_add_svc();
    }
    break;
#if 0
    case (GAPM_CREATE_ADV_ACTIVITY):
    case (GAPM_STOP_ACTIVITY):
    case (GAPM_START_ACTIVITY):
    case (GAPM_DELETE_ACTIVITY):
    case (GAPM_SET_ADV_DATA):
    case (GAPM_SET_SCAN_RSP_DATA):
    {
        // Sanity checks
        ASSERT_INFO(app_env.adv_op == param->operation, app_env.adv_op, param->operation);
        //ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->status, app_env.adv_op);

        // Perform next operation
        appm_adv_fsm_next();
    }
    break;

    case (GAPM_DELETE_ALL_ACTIVITIES) :
    {
        // Re-Invoke Advertising
        app_env.adv_state = APP_ADV_STATE_IDLE;
        appm_adv_fsm_next();
    }
    break;
#endif

    default:
    {
        // Drop the message
    }
    break;
    }

    return (KE_MSG_CONSUMED);

}

static int gapc_get_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_get_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    switch (param->req)
    {
    case GAPC_DEV_NAME:
    {
        struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC_DYN(GAPC_GET_DEV_INFO_CFM,
                                            src_id, dest_id,
                                            gapc_get_dev_info_cfm, APP_DEVICE_NAME_MAX_LEN);
        cfm->req = param->req;
        cfm->info.name.length = appm_get_dev_name(cfm->info.name.value);

        // Send message
        ke_msg_send(cfm);
    }
    break;

    case GAPC_DEV_APPEARANCE:
    {
        // Allocate message
        struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                            src_id, dest_id,
                                            gapc_get_dev_info_cfm);

        cfm->req = param->req;
        appm_get_appearance(&cfm->info.appearance);

        // Send message
        ke_msg_send(cfm);
    }
    break;

    case GAPC_DEV_SLV_PREF_PARAMS:
    {
        // Allocate message
        struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                            src_id, dest_id,
                                            gapc_get_dev_info_cfm);
        cfm->req = param->req;
        appm_get_slv_pref_params(&cfm->info.slv_pref_params);

        // Send message
        ke_msg_send(cfm);
    }
    break;

    default: /* Do Nothing */
        break;
    }


    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles GAPC_SET_DEV_INFO_REQ_IND message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_set_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_set_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    // Set Device configuration
    struct gapc_set_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_CFM, src_id, dest_id,
                                        gapc_set_dev_info_cfm);
    // Reject to change parameters
    switch (param->req)
    {
    case GAPC_DEV_NAME:
    {
        appm_set_dev_name((uint8_t *)param->info.name.value, param->info.name.length);
        break;
    }
    case GAPC_DEV_APPEARANCE:
    {
        appm_set_appearance(param->info.appearance);
        break;
    }
    case GAPC_DEV_SLV_PREF_PARAMS:
    {
        appm_set_slv_pref_params((struct gap_slv_pref *)&param->info.slv_pref_params);
        break;
    }

    }
    cfm->status = GAP_ERR_NO_ERROR;
    cfm->req = param->req;
    // Send message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_connection_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_connection_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
#if 0
    app_env.conidx = KE_IDX_GET(src_id);

    // Check if the received Connection Handle was valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
        // Retrieve the connection info from the parameters
        app_env.conhdl = param->conhdl;

        // Send connection confirmation
        struct gapc_connection_cfm *cfm = KE_MSG_ALLOC(GAPC_CONNECTION_CFM,
                                          KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                          gapc_connection_cfm);

#if(BLE_APP_SEC)
        cfm->auth      = app_sec_get_bond_status() ? GAP_AUTH_REQ_NO_MITM_BOND : GAP_AUTH_REQ_NO_MITM_NO_BOND; // TODO [FBE] restore valid data
#else // !(BLE_APP_SEC)
        cfm->auth      = GAP_AUTH_REQ_NO_MITM_NO_BOND;
#endif // (BLE_APP_SEC)
        // Send the message
        ke_msg_send(cfm);

#if DISPLAY_SUPPORT
        // Update displayed information
        app_display_set_adv(false);
        app_display_set_con(true);
#endif //(DISPLAY_SUPPORT)

        /*--------------------------------------------------------------
         * ENABLE REQUIRED PROFILES
         *--------------------------------------------------------------*/

#if (BLE_APP_BATT)
        // Enable Battery Service
        app_batt_enable_prf(app_env.conhdl);
#endif //(BLE_APP_BATT)

#if (BLE_APP_HID)
        // Enable HID Service
        app_hid_enable_prf(app_env.conhdl);
#endif //(BLE_APP_HID)

        // We are now in connected State
        ke_state_set(dest_id, APPM_CONNECTED);

#if (BLE_APP_SEC && !defined(BLE_APP_AM0))
        if (app_sec_get_bond_status())
        {
            // Ask for the peer device to either start encryption
            app_sec_send_security_req(app_env.conidx);
        }
#endif // (BLE_APP_SEC && !defined(BLE_APP_AM0))
    }
    else
    {
        // No connection has been established, restart advertising
        appm_update_adv_state(true);
    }
    ke_msg_forward(param, gapm_get_AHI_task_id(), src_id);

    return (KE_MSG_NO_FREE);
#endif
    app_env.conidx = KE_IDX_GET(src_id);

    //rt_kprintf("gapc_connection_req_ind_handler: %d, \n", app_env.conidx);
    // Check if the received Connection Handle was valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
        //g_conn_idx = app_env.conidx;

        // Retrieve the connection info from the parameters
        app_env.conhdl = param->conhdl;


        // should use gapm_get_AHI_task_id rather than TASK_AHI in dual-core
        // ke_msg_forward(param, gapm_get_AHI_task_id(), dest_id);

        //ke_msg_forward(param, TASK_AHI, dest_id);
        ke_state_set(dest_id, APPM_CONNECTED);
        // return (KE_MSG_NO_FREE);

    }
    ke_msg_forward(param, gapm_get_AHI_task_id(), src_id);
    return (KE_MSG_NO_FREE);

}

/**
 ****************************************************************************************
 * @brief Handles GAP controller command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapc_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    switch (param->operation)
    {
    case (GAPC_UPDATE_PARAMS):
    {
        if (param->status != GAP_ERR_NO_ERROR)
        {
//                appm_disconnect();
        }
    } break;

    default:
    {
    } break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid,
                                       struct gapc_disconnect_ind const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    // Go to the ready state
    ke_state_set(TASK_APP, APPM_READY);

#if (BLE_APP_HT)
    // Stop interval timer
    app_stop_timer();
#endif //(BLE_APP_HT)

#if (DISPLAY_SUPPORT)
    // Update Connection State screen
    app_display_set_con(false);
#endif //(DISPLAY_SUPPORT)

#if (BLE_ISO_MODE_0_PROTOCOL)
    app_env.adv_state = APP_ADV_STATE_CREATING;
#endif //(BLE_ISO_MODE_0_PROTOCOL)

#if (!BLE_APP_HID)
    // Restart Advertising
    //appm_update_adv_state(true);
#endif //(!BLE_APP_HID)

    ke_msg_forward(param, gapm_get_AHI_task_id(), src_id);
    return (KE_MSG_NO_FREE);
    //return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of all messages sent from the lower layers to the application
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int appm_msg_handler(ke_msg_id_t const msgid,
                            void *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    // Retrieve identifier of the task from received message
    ke_task_id_t src_task_id = MSG_T(msgid);
    // Message policy
    uint8_t msg_pol = 0xFF;

    switch (src_task_id)
    {
    case (TASK_ID_GAPC):
    {
#if (BLE_APP_SEC)
        if ((msgid >= GAPC_BOND_CMD) &&
                (msgid <= GAPC_SECURITY_IND))
        {
            // Call the Security Module
            msg_pol = app_get_handler(&app_sec_handlers, msgid, param, src_id);
        }
#endif //(BLE_APP_SEC)
        // else drop the message
    }
    break;

    case (TASK_ID_GATTC):
    {
        // Service Changed - Drop
    } break;

#if (BLE_APP_HT)
    case (TASK_ID_HTPT):
    {
        // Call the Health Thermometer Module
        msg_pol = app_get_handler(&app_ht_handlers, msgid, param, src_id);
    }
    break;
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
    case (TASK_ID_DISS):
    {
        // Call the Device Information Module
        msg_pol = app_get_handler(&app_dis_handlers, msgid, param, src_id);
    }
    break;
#endif //(BLE_APP_DIS)

#if (0)
    case (TASK_ID_SIBLES):
    {
        // Call the SiFli BLE Module
        msg_pol = app_get_handler(&app_sible_handlers, msgid, param, src_id);
    }
    break;
#endif //(BLE_APP_SIBLE)

#if (BLE_APP_HID)
    case (TASK_ID_HOGPD):
    {
        // Call the HID Module
        msg_pol = app_get_handler(&app_hid_handlers, msgid, param, src_id);
    }
    break;
#endif //(BLE_APP_HID)

#if (BLE_APP_BATT)
    case (TASK_ID_BASS):
    {
        // Call the Battery Module
        msg_pol = app_get_handler(&app_batt_handlers, msgid, param, src_id);
    }
    break;
#endif //(BLE_APP_BATT)

#if (BLE_APP_AM0)
    case (TASK_ID_AM0):
    {
        // Call the Audio Mode 0 Module
        msg_pol = app_get_handler(&app_am0_handlers, msgid, param, src_id);
    }
    break;

    case (TASK_ID_AM0_HAS):
    {
        // Call the Audio Mode 0 Module
        msg_pol = app_get_handler(&app_am0_has_handlers, msgid, param, src_id);
    }
    break;
#endif //(BLE_APP_AM0)

    default:
    {
#if (BLE_APP_HT)
        if (msgid == APP_HT_MEAS_INTV_TIMER)
        {
            msg_pol = app_get_handler(&app_ht_handlers, msgid, param, src_id);
        }
#endif //(BLE_APP_HT)

#if (BLE_APP_HID)
        if (msgid == APP_HID_MOUSE_TIMEOUT_TIMER)
        {
            msg_pol = app_get_handler(&app_hid_handlers, msgid, param, src_id);
        }
#endif //(BLE_APP_HID)

#if (NVDS_SUPPORT)
        if (msgid >= APP_SIFLI_NVDS_GET_REQ &&
                msgid <= APP_SIFLI_NVDS_SET_CNF)
        {
            msg_pol = app_get_handler(&sifli_nvds_msg_handler, msgid, param, src_id);
        }
#endif // (NVDS_SUPPORT)
    }
    break;
    }

    // No handler handle it
    if (msg_pol == 0xFF)
    {
        ke_msg_forward(param, gapm_get_AHI_task_id(), src_id);
        msg_pol =  KE_MSG_NO_FREE;
    }

    return (msg_pol);
}

/**
 ****************************************************************************************
 * @brief Handles reception of random number generated message
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_gen_rand_nb_ind_handler(ke_msg_id_t const msgid, struct gapm_gen_rand_nb_ind *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if (app_env.rand_cnt == 1)    // First part of IRK
    {
        memcpy(&app_env.loc_irk[0], &param->randnb.nb[0], 8);
    }
    else if (app_env.rand_cnt == 2) // Second part of IRK
    {
        memcpy(&app_env.loc_irk[8], &param->randnb.nb[0], 8);
    }

    return KE_MSG_CONSUMED;
}

static int gapc_param_update_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_param_update_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
#if 0
    struct gapc_param_update_cfm *cfm = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM, src_id, dest_id, gapc_param_update_cfm);
    cfm->accept = 1;
    cfm->ce_len_min = param->intv_min;
    cfm->ce_len_max = param->intv_max;
    ke_msg_send(cfm);
#endif
    ke_msg_forward(param, gapm_get_AHI_task_id(), src_id);
    return (KE_MSG_NO_FREE);
}

static int gapc_param_updated_ind_handler(ke_msg_id_t const msgid,
        struct gapc_param_updated_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    ke_msg_forward(param, gapm_get_AHI_task_id(), src_id);
    return (KE_MSG_NO_FREE);
}

static int gattc_mtu_update_ind_handler(ke_msg_id_t const msgid,
                                        struct gattc_mtu_changed_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    ke_msg_forward(param, gapm_get_AHI_task_id(), src_id);
    return (KE_MSG_NO_FREE);
}

/*
 * GLOBAL VARIABLES DEFINITION
 ****************************************************************************************
 */

/* Default State handlers definition. */
KE_MSG_HANDLER_TAB(appm)
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER, (ke_msg_func_t)appm_msg_handler},

    // GAPM messages
    {GAPM_PROFILE_ADDED_IND, (ke_msg_func_t)gapm_profile_added_ind_handler},
#if 0
    {GAPM_ACTIVITY_CREATED_IND, (ke_msg_func_t)gapm_activity_created_ind_handler},
    {GAPM_ACTIVITY_STOPPED_IND, (ke_msg_func_t)gapm_activity_stopped_ind_handler},
#endif
    {GAPM_CMP_EVT, (ke_msg_func_t)gapm_cmp_evt_handler},
    {GAPM_GEN_RAND_NB_IND, (ke_msg_func_t)gapm_gen_rand_nb_ind_handler},

    // GAPC messages
    {GAPC_GET_DEV_INFO_REQ_IND, (ke_msg_func_t)gapc_get_dev_info_req_ind_handler},
    {GAPC_SET_DEV_INFO_REQ_IND, (ke_msg_func_t)gapc_set_dev_info_req_ind_handler},
    {GAPC_CONNECTION_REQ_IND, (ke_msg_func_t)gapc_connection_req_ind_handler},
    {GAPC_CMP_EVT, (ke_msg_func_t)gapc_cmp_evt_handler},
    {GAPC_DISCONNECT_IND, (ke_msg_func_t)gapc_disconnect_ind_handler},
    {GAPC_PARAM_UPDATE_REQ_IND, (ke_msg_func_t)gapc_param_update_req_ind_handler},
    {GAPC_PARAM_UPDATED_IND, (ke_msg_func_t)gapc_param_updated_ind_handler},

    // GATTC messages
    {GATTC_MTU_CHANGED_IND, (ke_msg_func_t)gattc_mtu_update_ind_handler},
};

/* Defines the place holder for the states of all the task instances. */
ke_state_t appm_state[APP_IDX_MAX];

// Application task descriptor
const struct ke_task_desc TASK_DESC_APP = {appm_msg_handler_tab, appm_state, APP_IDX_MAX, ARRAY_LEN(appm_msg_handler_tab)};

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
