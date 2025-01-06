/**
 ****************************************************************************************
 *
 * @file app_sec.c
 *
 * @brief Application Security Entry Point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_APP_SEC)

#include <string.h>
#include "co_utils.h"
#include "co_math.h"
#include "gapc_task.h"      // GAP Controller Task API Definition
#include "gap.h"            // GAP Definition
#include "gapc.h"           // GAPC Definition
#include "prf_types.h"

#include "app.h"            // Application API Definition
#include "app_sec.h"        // Application Security API Definition
#include "app_task.h"       // Application Manager API Definition

#if (DISPLAY_SUPPORT)
    #include "app_display.h"    // Display Application Definitions
#endif //(DISPLAY_SUPPORT)

#if (NVDS_SUPPORT)
    #include "sifli_nvds.h"           // NVDS API Definitions
#endif //(NVDS_SUPPORT)

#if (BLE_APP_AM0)
    #include "app_am0.h"
#endif //(BLE_APP_AM0)

#undef NVDS_SUPPORT
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Application Security Environment Structure
struct app_sec_env_tag app_sec_env;

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_sec_init()
{
    /*------------------------------------------------------
     * RETRIEVE BOND STATUS
     *------------------------------------------------------*/
#if (NVDS_SUPPORT)
    uint8_t length = NVDS_LEN_PERIPH_BONDED;

    // Get bond status from NVDS
    if (sifli_nvds_get(NVDS_TAG_PERIPH_BONDED, &length, (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
    {
        // If read value is invalid, set status to not bonded
        app_sec_env.bonded = false;
    }

    if ((app_sec_env.bonded != true) && (app_sec_env.bonded != false))
    {
        app_sec_env.bonded = false;
    }

#if (DISPLAY_SUPPORT)
    // Update the bond status screen value
    app_display_set_bond(app_sec_env.bonded);
#endif //(DISPLAY_SUPPORT)
#endif //(NVDS_SUPPORT)
}

bool app_sec_get_bond_status(void)
{
    return app_sec_env.bonded;
}

#if (NVDS_SUPPORT)
void app_sec_remove_bond(void)
{
#if (BLE_APP_HID)
    uint16_t ntf_cfg = PRF_CLI_STOP_NTFIND;
#endif //(BLE_APP_HID)

    // Check if we are well bonded
    if (app_sec_env.bonded == true)
    {
        // Update the environment variable
        app_sec_env.bonded = false;

        if (sifli_nvds_put(NVDS_TAG_PERIPH_BONDED, NVDS_LEN_PERIPH_BONDED,
                           (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
        {
            ASSERT_ERR(0);
        }

#if (BLE_APP_HT)
        if (sifli_nvds_del(NVDS_TAG_LTK) != NVDS_OK)
        {
            ASSERT_ERR(0);
        }

        if (sifli_nvds_del(NVDS_TAG_PEER_BD_ADDRESS) != NVDS_OK)
        {
            ASSERT_ERR(0);
        }
#endif //(BLE_APP_HT)

#if (BLE_APP_HID)
        if (sifli_nvds_put(NVDS_TAG_MOUSE_NTF_CFG, NVDS_LEN_MOUSE_NTF_CFG,
                           (uint8_t *)&ntf_cfg) != NVDS_OK)
        {
            ASSERT_ERR(0);
        }
#endif //(BLE_APP_HID)
    }
}
#endif //(NVDS_SUPPORT)

void app_sec_send_security_req(uint8_t conidx)
{
    // Send security request
    struct gapc_security_cmd *cmd = KE_MSG_ALLOC(GAPC_SECURITY_CMD,
                                    KE_BUILD_ID(TASK_GAPC, conidx), TASK_APP,
                                    gapc_security_cmd);

    cmd->operation = GAPC_SECURITY_REQ;

#if (BLE_APP_HID || BLE_APP_HT)
    cmd->auth      = GAP_AUTH_REQ_MITM_BOND;
#elif defined(BLE_APP_AM0)
    cmd->auth      = GAP_AUTH_REQ_NO_MITM_BOND;
#else
    cmd->auth      = GAP_AUTH_REQ_NO_MITM_NO_BOND;
#endif //(BLE_APP_HID || BLE_APP_HT)

    // Send the message
    ke_msg_send(cmd);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

static int gapc_bond_req_ind_handler(ke_msg_id_t const msgid,
                                     struct gapc_bond_req_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{

#if 0
    // Prepare the GAPC_BOND_CFM message
    struct gapc_bond_cfm *cfm = KE_MSG_ALLOC(GAPC_BOND_CFM,
                                src_id, TASK_APP,
                                gapc_bond_cfm);

    switch (param->request)
    {
    case (GAPC_PAIRING_REQ):
    {
        cfm->request = GAPC_PAIRING_RSP;

#ifndef BLE_APP_AM0
        cfm->accept  = false;

        // Check if we are already bonded (Only one bonded connection is supported)
        if (!app_sec_env.bonded)
#endif // !BLE_APP_AM0
        {
            cfm->accept  = true;

#if (BLE_APP_HID || BLE_APP_HT)
            // Pairing Features
            cfm->data.pairing_feat.auth      = GAP_AUTH_REQ_MITM_BOND;
#elif defined(BLE_APP_AM0)
#if (BLE_APP_SEC_CON == 1)
            if (param->data.auth_req & GAP_AUTH_SEC_CON)
            {
                cfm->data.pairing_feat.auth      = GAP_AUTH_REQ_SEC_CON_BOND;
                app_env.sec_con_enabled = true;
            }
            else
            {
                cfm->data.pairing_feat.auth      = GAP_AUTH_REQ_NO_MITM_BOND;
                app_env.sec_con_enabled = false;
            }
#else  // !(BLE_APP_SEC_CON)
            cfm->data.pairing_feat.auth      = GAP_AUTH_REQ_NO_MITM_BOND;
            app_env.sec_con_enabled = false;
#endif // (BLE_APP_SEC_CON)
#else
            cfm->data.pairing_feat.auth      = GAP_AUTH_REQ_NO_MITM_NO_BOND;
#endif //(BLE_APP_HID || BLE_APP_HT)

#if (BLE_APP_HT)
            cfm->data.pairing_feat.iocap     = GAP_IO_CAP_DISPLAY_ONLY;
#else
            cfm->data.pairing_feat.iocap     = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
#endif //(BLE_APP_HT)

            cfm->data.pairing_feat.key_size  = 16;
            cfm->data.pairing_feat.oob       = GAP_OOB_AUTH_DATA_NOT_PRESENT;
            cfm->data.pairing_feat.sec_req   = GAP_NO_SEC;
#if (defined(BLE_APP_AM0))
            cfm->data.pairing_feat.rkey_dist = GAP_KDIST_ENCKEY | GAP_KDIST_IDKEY;
#if BLE_APP_AM0
            // No Keys required from the Initiator
            cfm->data.pairing_feat.ikey_dist = 0;
#else
            cfm->data.pairing_feat.ikey_dist = GAP_KDIST_ENCKEY | GAP_KDIST_IDKEY;
#endif
#else
            cfm->data.pairing_feat.ikey_dist = GAP_KDIST_NONE;
            cfm->data.pairing_feat.rkey_dist = GAP_KDIST_ENCKEY;
#endif // (defined(BLE_APP_AM0))
        }
    }
    break;

    case (GAPC_LTK_EXCH):
    {
        // Counter
        uint8_t counter;

        cfm->accept  = true;
        cfm->request = GAPC_LTK_EXCH;

        // Generate all the values
        cfm->data.ltk.ediv = (uint16_t)co_rand_word();

        for (counter = 0; counter < RAND_NB_LEN; counter++)
        {
            cfm->data.ltk.ltk.key[counter]    = (uint8_t)co_rand_word();
            cfm->data.ltk.randnb.nb[counter] = (uint8_t)co_rand_word();
        }

        for (counter = RAND_NB_LEN; counter < KEY_LEN; counter++)
        {
            cfm->data.ltk.ltk.key[counter]    = (uint8_t)co_rand_word();
        }

#if (NVDS_SUPPORT)
        // Store the generated value in NVDS
        if (sifli_nvds_put(NVDS_TAG_LTK, NVDS_LEN_LTK,
                           (uint8_t *)&cfm->data.ltk) != NVDS_OK)
        {
            ASSERT_ERR(0);
        }
#endif // #if (NVDS_SUPPORT)
    }
    break;


    case (GAPC_IRK_EXCH):
    {
#if (NVDS_SUPPORT)
        uint8_t addr_len = BD_ADDR_LEN;
#endif //(NVDS_SUPPORT)

        cfm->accept  = true;
        cfm->request = GAPC_IRK_EXCH;

        // Load IRK
        memcpy(cfm->data.irk.irk.key, app_env.loc_irk, KEY_LEN);

#if (NVDS_SUPPORT)
        if (sifli_nvds_get(NVDS_TAG_BD_ADDRESS, &addr_len, cfm->data.irk.addr.addr.addr) != NVDS_OK)
#endif //(NVDS_SUPPORT)
        {
            //ASSERT_ERR(0);
        }
        // load device address
        cfm->data.irk.addr.addr_type = (cfm->data.irk.addr.addr.addr[5] & 0xC0) ? ADDR_RAND : ADDR_PUBLIC;
    }
    break;


#if (BLE_APP_HT)
    case (GAPC_TK_EXCH):
    {
        // Generate a PIN Code- (Between 100000 and 999999)
        uint32_t pin_code = (100000 + (co_rand_word() % 900000));

#if DISPLAY_SUPPORT
        // Display the PIN Code
        app_display_pin_code(pin_code);
#endif //DISPLAY_SUPPORT

        cfm->accept  = true;
        cfm->request = GAPC_TK_EXCH;

        // Set the TK value
        memset(cfm->data.tk.key, 0, KEY_LEN);

        cfm->data.tk.key[0] = (uint8_t)((pin_code & 0x000000FF) >>  0);
        cfm->data.tk.key[1] = (uint8_t)((pin_code & 0x0000FF00) >>  8);
        cfm->data.tk.key[2] = (uint8_t)((pin_code & 0x00FF0000) >> 16);
        cfm->data.tk.key[3] = (uint8_t)((pin_code & 0xFF000000) >> 24);
    }
    break;
#endif //(BLE_APP_HT)

    default:
    {
        ASSERT_ERR(0);
    }
    break;
    }

    // Send the message
    ke_msg_send(cfm);
    return (KE_MSG_CONSUMED);
#endif
    ke_msg_forward(param, gapm_get_AHI_task_id(), src_id);
    return (KE_MSG_NO_FREE);

}

static int gapc_bond_ind_handler(ke_msg_id_t const msgid,
                                 struct gapc_bond_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
#if 0
    switch (param->info)
    {
    case (GAPC_PAIRING_SUCCEED):
    {
        // Update the bonding status in the environment
        app_sec_env.bonded = true;

        // Update the bonding status in the environment
#if (NVDS_SUPPORT)
        if (sifli_nvds_put(NVDS_TAG_PERIPH_BONDED, NVDS_LEN_PERIPH_BONDED,
                           (uint8_t *)&app_sec_env.bonded) != NVDS_OK)
        {
            // An error has occurred during access to the NVDS
            ASSERT_ERR(0);
        }

        // Set the BD Address of the peer device in NVDS
        if (sifli_nvds_put(NVDS_TAG_PEER_BD_ADDRESS, NVDS_LEN_PEER_BD_ADDRESS,
                           (uint8_t *)gapc_get_bdaddr(0, SMPC_INFO_PEER)) != NVDS_OK)
        {
            // An error has occurred during access to the NVDS
            ASSERT_ERR(0);
        }

        // Update nvds while bond success
        sifli_nvds_update();

#if (DISPLAY_SUPPORT)
        // Update the bond status screen value
        app_display_set_bond(app_sec_env.bonded);
#endif //(DISPLAY_SUPPORT)
#endif //(NVDS_SUPPORT)

#if (BLE_APP_AM0)
        app_am0_send_audio_init(KE_IDX_GET(src_id));
#endif //(BLE_APP_AM0)
        ke_msg_forward(param, gapm_get_AHI_task_id(), dest_id);
        return (KE_MSG_NO_FREE);

    }
    break;

    case (GAPC_REPEATED_ATTEMPT):
    {
        appm_disconnect();
    }
    break;

    case (GAPC_IRK_EXCH):
    {
#if (NVDS_SUPPORT)
        // Store peer identity in NVDS
        if (sifli_nvds_put(NVDS_TAG_PEER_IRK, NVDS_LEN_PEER_IRK, (uint8_t *)&param->data.irk) != NVDS_OK)
        {
            ASSERT_ERR(0);
        }
#endif // (NVDS_SUPPORT)
    }
    break;

    case (GAPC_PAIRING_FAILED):
    {
        app_sec_send_security_req(0);
    }
    break;

    // In Secure Connections we get BOND_IND with SMPC calculated LTK
    case (GAPC_LTK_EXCH) :
    {
#if (BLE_APP_SEC_CON)
        if (app_env.sec_con_enabled == true)
        {
#if (NVDS_SUPPORT)
            // Store LTK in NVDS
            if (sifli_nvds_put(NVDS_TAG_LTK, NVDS_LEN_LTK, (uint8_t *)&param->data.ltk.ltk.key[0]) != NVDS_OK)
            {
                ASSERT_ERR(0);
            }
#endif // (NVDS_SUPPORT)
        }
#endif // (BLE_APP_SEC_CON)
    }
    break;

    default:
    {
        ASSERT_ERR(0);
    }
    break;
    }

    return (KE_MSG_CONSUMED);
#endif
    //uint8_t conn_ind = KE_IDX_GET(src_id);
    //if (conn_ind != g_conn_idx)
    //{
    //  return (KE_MSG_CONSUMED);
    //}

    ke_msg_forward(param, gapm_get_AHI_task_id(), src_id);
    return (KE_MSG_NO_FREE);

}

static int gapc_encrypt_req_ind_handler(ke_msg_id_t const msgid,
                                        struct gapc_encrypt_req_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
#if 0
#if (NVDS_SUPPORT)
    // LTK value
    struct gapc_ltk ltk;
    // Length
    uint8_t length = NVDS_LEN_LTK;
#endif // #if (NVDS_SUPPORT)

    // Prepare the GAPC_ENCRYPT_CFM message
    struct gapc_encrypt_cfm *cfm = KE_MSG_ALLOC(GAPC_ENCRYPT_CFM,
                                   src_id, TASK_APP,
                                   gapc_encrypt_cfm);

    cfm->found    = false;

    if (app_sec_env.bonded)
    {
#if (NVDS_SUPPORT)
        // Retrieve the required informations from NVDS
        if (sifli_nvds_get(NVDS_TAG_LTK, &length, (uint8_t *)&ltk) == NVDS_OK)
        {
            // Check if the provided EDIV and Rand Nb values match with the stored values
            if ((param->ediv == ltk.ediv) &&
                    !memcmp(&param->rand_nb.nb[0], &ltk.randnb.nb[0], sizeof(struct rand_nb)))
            {
                cfm->found    = true;
                cfm->key_size = 16;
                memcpy(&cfm->ltk, &ltk.ltk, sizeof(struct gap_sec_key));
            }
            /*
             * else we are bonded with another device, disconnect the link
             */
        }
        else
        {
            ASSERT_ERR(0);
        }
#endif // #if (NVDS_SUPPORT)
    }
    /*
     * else the peer device is not known, an error should trigger a new pairing procedure.
     */

    // Send the message
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
#endif
    ke_msg_forward(param, gapm_get_AHI_task_id(), src_id);
    return (KE_MSG_NO_FREE);

}


static int gapc_encrypt_ind_handler(ke_msg_id_t const msgid,
                                    struct gapc_encrypt_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // encryption/ re-encryption succeeded
#if 0
#if (BLE_APP_AM0)
    // Need to Setup Authenicated Payload TO for the connection.
    struct gapc_set_le_ping_to_cmd *cmd = KE_MSG_ALLOC(GAPC_SET_LE_PING_TO_CMD,
                                          KE_BUILD_ID(TASK_GAPC, KE_IDX_GET(src_id)), TASK_APP,
                                          gapc_set_le_ping_to_cmd);

    // encryption/ re-encryption succeeded

    app_am0_send_audio_init(KE_IDX_GET(src_id));

    cmd->operation = GAPC_SET_LE_PING_TO;
    cmd->timeout = 1000; // 10 Sec

    // Send the message
    ke_msg_send(cmd);
#endif //(BLE_APP_AM0)
    ke_msg_forward(param, gapm_get_AHI_task_id(), dest_id);
    return (KE_MSG_NO_FREE);
    //return (KE_MSG_CONSUMED);
#endif
    ke_msg_forward(param, gapm_get_AHI_task_id(), src_id);
    return (KE_MSG_NO_FREE);

}

static int app_sec_msg_dflt_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    // Drop the message
    ke_msg_forward(param, gapm_get_AHI_task_id(), src_id);
    return (KE_MSG_NO_FREE);

}

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
const struct ke_msg_handler app_sec_msg_handler_list[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER, (ke_msg_func_t)app_sec_msg_dflt_handler},

    {GAPC_BOND_REQ_IND, (ke_msg_func_t)gapc_bond_req_ind_handler},
    {GAPC_BOND_IND, (ke_msg_func_t)gapc_bond_ind_handler},

    {GAPC_ENCRYPT_REQ_IND, (ke_msg_func_t)gapc_encrypt_req_ind_handler},
    {GAPC_ENCRYPT_IND, (ke_msg_func_t)gapc_encrypt_ind_handler},
};

const struct app_subtask_handlers app_sec_handlers = {&app_sec_msg_handler_list[0], ARRAY_LEN(app_sec_msg_handler_list)};

#endif //(BLE_APP_SEC)

/// @} APP
