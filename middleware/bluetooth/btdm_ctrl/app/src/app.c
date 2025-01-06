/**
 ****************************************************************************************
 *
 * @file app.c
 *
 * @brief Application entry point
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

#include "rwip_config.h"             // SW configuration

#if (BLE_APP_PRESENT)

#include <string.h>

#include "app_task.h"                // Application task Definition
#include "app.h"                     // Application Definition
#include "gap.h"                     // GAP Definition
#include "gapm_task.h"               // GAP Manager Task API
#include "gapc_task.h"               // GAP Controller Task API

#include "co_bt.h"                   // Common BT Definition
#include "co_math.h"                 // Common Maths Definition
#include "arch.h"

#if (BLE_APP_SEC)
    #include "app_sec.h"                 // Application security Definition
#endif // (BLE_APP_SEC)

#if (BLE_APP_HT)
    #include "app_ht.h"                  // Health Thermometer Application Definitions
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
    #include "app_dis.h"                 // Device Information Service Application Definitions
#endif //(BLE_APP_DIS)

#if (BLE_APP_BATT)
    #include "app_batt.h"                // Battery Application Definitions
#endif //(BLE_APP_DIS)

#if (BLE_APP_HID)
    #include "app_hid.h"                 // HID Application Definitions
#endif //(BLE_APP_HID)

#if (BLE_APP_SIBLE)
    #include "app_sible.h"                 // Sifli BLE server Application
#endif //(BLE_APP_SIBLES)

#if (DISPLAY_SUPPORT)
    #include "app_display.h"             // Application Display Definition
#endif //(DISPLAY_SUPPORT)

#if (BLE_APP_AM0)
    #include "app_am0.h"                 // Audio Mode 0 Application
#endif //(BLE_APP_AM0)

#if (NVDS_SUPPORT)
    #include "sifli_nvds.h"                    // NVDS Definitions
#endif //(NVDS_SUPPORT)

#include "dbg.h"

#undef NVDS_SUPPORT
/*
 * DEFINES
 ****************************************************************************************
 */

/// Default Device Name
#define APP_DFLT_DEVICE_NAME            ("SIFLI-BLE-DEV")
#define APP_DFLT_DEVICE_NAME_LEN        (sizeof(APP_DFLT_DEVICE_NAME))


#if (BLE_APP_HID)
    // HID Mouse
    #define DEVICE_NAME        "Hid Mouse"
#else
    #define DEVICE_NAME        "RW DEVICE"
#endif

#define DEVICE_NAME_SIZE    sizeof(DEVICE_NAME)

/**
 * UUID List part of ADV Data
 * --------------------------------------------------------------------------------------
 * x03 - Length
 * x03 - Complete list of 16-bit UUIDs available
 * x09\x18 - Health Thermometer Service UUID
 *   or
 * x12\x18 - HID Service UUID
 * --------------------------------------------------------------------------------------
 */

#if (BLE_APP_HT)
    #define APP_HT_ADV_DATA_UUID        "\x03\x03\x09\x18"
    #define APP_HT_ADV_DATA_UUID_LEN    (4)
#endif //(BLE_APP_HT)

#if (BLE_APP_HID)
    #define APP_HID_ADV_DATA_UUID       "\x03\x03\x12\x18"
    #define APP_HID_ADV_DATA_UUID_LEN   (4)
#endif //(BLE_APP_HID)

/**
 * Appearance part of ADV Data
 * --------------------------------------------------------------------------------------
 * x03 - Length
 * x19 - Appearance
 * x03\x00 - Generic Thermometer
 *   or
 * xC2\x04 - HID Mouse
 * --------------------------------------------------------------------------------------
 */

#if (BLE_APP_HT)
    #define APP_HT_ADV_DATA_APPEARANCE    "\x03\x19\x00\x03"
#endif //(BLE_APP_HT)

#if (BLE_APP_HID)
    #define APP_HID_ADV_DATA_APPEARANCE   "\x03\x19\xC2\x03"
#endif //(BLE_APP_HID)

#define APP_ADV_DATA_APPEARANCE_LEN  (4)

/**
 * Default Scan response data
 * --------------------------------------------------------------------------------------
 * x09                             - Length
 * xFF                             - Vendor specific advertising type
 * x00\x60\x52\x57\x2D\x42\x4C\x45 - "RW-BLE"
 * --------------------------------------------------------------------------------------
 */
#define APP_SCNRSP_DATA         "\x08\xFF\x00\x60\x53\x49\x42\x4C\x45"
#define APP_SCNRSP_DATA_LEN     (9)

/**
 * Advertising Parameters
 */
#if (BLE_APP_HID)
    /// Default Advertising duration - 30s (in multiple of 10ms)
    #define APP_DFLT_ADV_DURATION   (3000)
#endif //(BLE_APP_HID)
/// Advertising channel map - 37, 38, 39
#define APP_ADV_CHMAP           (0x07)
/// Advertising minimum interval - 40ms (64*0.625ms)
#define APP_ADV_INT_MIN         (64)
/// Advertising maximum interval - 40ms (64*0.625ms)
#define APP_ADV_INT_MAX         (64)
/// Fast advertising interval
#define APP_ADV_FAST_INT        (32)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

typedef void (*appm_add_svc_func_t)(void);

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// List of service to add in the database
enum appm_svc_list
{
#if (BLE_APP_HT)
    APPM_SVC_HTS,
#endif //(BLE_APP_HT)
#if (BLE_APP_DIS)
    APPM_SVC_DIS,
#endif //(BLE_APP_DIS)
#if (BLE_APP_BATT)
    APPM_SVC_BATT,
#endif //(BLE_APP_BATT)
#if (BLE_APP_HID)
    APPM_SVC_HIDS,
#endif //(BLE_APP_HID)
#if (BLE_APP_SIBLE)
    APPM_SVC_SIBLES,
#endif //(BLE_APP_HID)
#if (BLE_APP_AM0)
    APPM_SVC_AM0_HAS,
#endif //(BLE_APP_AM0)

    APPM_SVC_LIST_STOP,
};

/*
 * LOCAL VARIABLES DEFINITIONS
 ****************************************************************************************
 */

/// Application Task Descriptor
extern const struct ke_task_desc TASK_DESC_APP;

/// List of functions used to create the database
static const appm_add_svc_func_t appm_add_svc_func_list[APPM_SVC_LIST_STOP] =
{
#if (BLE_APP_HT)
    (appm_add_svc_func_t)app_ht_add_hts,
#endif //(BLE_APP_HT)
#if (BLE_APP_DIS)
    (appm_add_svc_func_t)app_dis_add_dis,
#endif //(BLE_APP_DIS)
#if (BLE_APP_BATT)
    (appm_add_svc_func_t)app_batt_add_bas,
#endif //(BLE_APP_BATT)
#if (BLE_APP_HID)
    (appm_add_svc_func_t)app_hid_add_hids,
#endif //(BLE_APP_HID)
#if (BLE_APP_SIBLE)
    (appm_add_svc_func_t)app_sible_add_sibles,
#endif //(BLE_APP_SIBLES)
#if (BLE_APP_AM0)
    (appm_add_svc_func_t)app_am0_add_has,
#endif //(BLE_APP_AM0)
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Application Environment Structure
struct app_env_tag app_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (!BLE_APP_AM0)
static void appm_build_adv_data(uint16_t max_length, uint16_t *p_length, uint8_t *p_buf)
{
    // Remaining Length
    uint8_t rem_len = max_length;

#if (BLE_APP_HT)
    // Set list of UUIDs
    memcpy(p_buf, APP_HT_ADV_DATA_UUID, APP_HT_ADV_DATA_UUID_LEN);
    *p_length += APP_HT_ADV_DATA_UUID_LEN;
    p_buf += APP_HT_ADV_DATA_UUID_LEN;

    // Set appearance
    memcpy(p_buf, APP_HT_ADV_DATA_APPEARANCE, APP_ADV_DATA_APPEARANCE_LEN);
    *p_length += APP_ADV_DATA_APPEARANCE_LEN;
    p_buf += APP_ADV_DATA_APPEARANCE_LEN;
#endif //(BLE_APP_HT)

#if (BLE_APP_HID)
    // Set list of UUIDs
    memcpy(p_buf, APP_HID_ADV_DATA_UUID, APP_HID_ADV_DATA_UUID_LEN);
    *p_length += APP_HID_ADV_DATA_UUID_LEN;
    p_buf += APP_HID_ADV_DATA_UUID_LEN;

    // Set appearance
    memcpy(p_buf, APP_HID_ADV_DATA_APPEARANCE, APP_ADV_DATA_APPEARANCE_LEN);
    *p_length += APP_ADV_DATA_APPEARANCE_LEN;
    p_buf += APP_ADV_DATA_APPEARANCE_LEN;
#endif //(BLE_APP_HID)

#if (BLE_APP_SIBLE)
    {
        int len;
        len = sibles_get_adv_data(p_buf);
        p_buf += len;
        *p_length += len;
    }
#endif

    // Sanity check
    ASSERT_ERR(rem_len >= max_length);

    // Get remaining space in the Advertising Data - 2 bytes are used for name length/flag
    rem_len -= *p_length;

    // Check if additional data can be added to the Advertising data - 2 bytes needed for type and length
    if (rem_len > 2)
    {
        uint8_t dev_name_length = co_min(app_env.dev_name_len, (rem_len - 2));

        // Device name length
        *p_buf = dev_name_length + 1;
        // Device name flag (check if device name is complete or not)
        *(p_buf + 1) = (dev_name_length == app_env.dev_name_len) ? '\x08' : '\x09';
        // Copy device name
        memcpy(p_buf + 2, app_env.dev_name, dev_name_length);

        // Update advertising data length
        *p_length += (dev_name_length + 2);
    }
}
#endif //(!BLE_APP_AM0)

static void appm_start_advertising(void)
{
    // Prepare the GAPM_ACTIVITY_START_CMD message
    struct gapm_activity_start_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_START_CMD,
                                            TASK_GAPM, TASK_APP,
                                            gapm_activity_start_cmd);

    p_cmd->operation = GAPM_START_ACTIVITY;
    p_cmd->actv_idx = app_env.adv_actv_idx;
#if (BLE_APP_HID)
    p_cmd->u_param.adv_add_param.duration = APP_DFLT_ADV_DURATION;
#else //(BLE_APP_HID)
    p_cmd->u_param.adv_add_param.duration = 0;
#endif //(BLE_APP_HID)
    p_cmd->u_param.adv_add_param.max_adv_evt = 0;

    // Send the message
    ke_msg_send(p_cmd);

    // Keep the current operation
    app_env.adv_state = APP_ADV_STATE_STARTING;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_START_ACTIVITY;
}


static void appm_stop_advertising(void)
{
    // Prepare the GAPM_ACTIVITY_STOP_CMD message
    struct gapm_activity_stop_cmd *cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_STOP_CMD,
                                         TASK_GAPM, TASK_APP,
                                         gapm_activity_stop_cmd);

    // Fill the allocated kernel message
    cmd->operation = GAPM_STOP_ACTIVITY;
    cmd->actv_idx = app_env.adv_actv_idx;

    // Send the message
    ke_msg_send(cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_STOPPING;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_STOP_ACTIVITY;
}


static void appm_set_adv_data(void)
{
    // Prepare the GAPM_SET_ADV_DATA_CMD message
    struct gapm_set_adv_data_cmd *p_cmd = KE_MSG_ALLOC_DYN(GAPM_SET_ADV_DATA_CMD,
                                          TASK_GAPM, TASK_APP,
                                          gapm_set_adv_data_cmd,
                                          ADV_DATA_LEN);

    // Fill the allocated kernel message
    p_cmd->operation = GAPM_SET_ADV_DATA;
    p_cmd->actv_idx = app_env.adv_actv_idx;

#if (BLE_APP_AM0)
    app_am0_fill_adv_data(p_cmd);
#else //(BLE_APP_AM0)
    p_cmd->length = 0;
    // GAP will use 3 bytes for the AD Type
    appm_build_adv_data(ADV_DATA_LEN - 3, &p_cmd->length, &p_cmd->data[0]);
#endif //(BLE_APP_AM0)

    // Send the message
    ke_msg_send(p_cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_SETTING_ADV_DATA;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_SET_ADV_DATA;
}

static void appm_set_scan_rsp_data(void)
{
    // Prepare the GAPM_SET_ADV_DATA_CMD message
    struct gapm_set_adv_data_cmd *p_cmd = KE_MSG_ALLOC_DYN(GAPM_SET_ADV_DATA_CMD,
                                          TASK_GAPM, TASK_APP,
                                          gapm_set_adv_data_cmd,
                                          ADV_DATA_LEN);

    // Fill the allocated kernel message
    p_cmd->operation = GAPM_SET_SCAN_RSP_DATA;
    p_cmd->actv_idx = app_env.adv_actv_idx;

#if (BLE_APP_AM0)
    app_am0_fill_scan_rsp_data(p_cmd);
#elif (BLE_APP_SIBLE)
    p_cmd->length = sibles_get_scan_rsp(&p_cmd->data[0]);
#else
    p_cmd->length = APP_SCNRSP_DATA_LEN;
    memcpy(&p_cmd->data[0], APP_SCNRSP_DATA, APP_SCNRSP_DATA_LEN);
#endif //(BLE_APP_AM0)

    // Send the message
    ke_msg_send(p_cmd);

    // Update advertising state
    app_env.adv_state = APP_ADV_STATE_SETTING_SCAN_RSP_DATA;
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_SET_SCAN_RSP_DATA;
}


static void appm_send_gapm_reset_cmd(void)
{
    // Reset the stack
    struct gapm_reset_cmd *p_cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,
                                   TASK_GAPM, TASK_APP,
                                   gapm_reset_cmd);

    p_cmd->operation = GAPM_RESET;

    ke_msg_send(p_cmd);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


void appm_task_init(void)
{
    TRC_PRINTF("appm_init");
    // Reset the application manager environment
    memset(&app_env, 0, sizeof(app_env));

    // Create APP task
    ke_task_create(TASK_APP, &TASK_DESC_APP);

    // Initialize Task state
    ke_state_set(TASK_APP, APPM_INIT);

}


void appm_init()
{

    appm_task_init();
    // Get the Device Name to add in the Advertising Data (Default one or NVDS one)
#if (NVDS_SUPPORT)
    app_env.dev_name_len = APP_DEVICE_NAME_MAX_LEN;
    if (sifli_nvds_get(NVDS_TAG_DEVICE_NAME, &(app_env.dev_name_len), app_env.dev_name) != NVDS_OK)
#endif //(NVDS_SUPPORT)
    {
        // Get default Device Name (No name if not enough space)
        memcpy(app_env.dev_name, APP_DFLT_DEVICE_NAME, APP_DFLT_DEVICE_NAME_LEN);
        app_env.dev_name_len = APP_DFLT_DEVICE_NAME_LEN;

        // TODO update this value per profiles
    }

    // Initial slave pref params.
    app_env.pref.conn_timeout = 200;
    app_env.pref.con_intv_max = 10;
    app_env.pref.con_intv_min = 8;
    app_env.pref.slave_latency = 0;
    //app_env.start_adv = 1;

    /*------------------------------------------------------
     * INITIALIZE ALL MODULES
     *------------------------------------------------------*/

    // load device information:

#if (DISPLAY_SUPPORT)
    app_display_init();
#endif //(DISPLAY_SUPPORT)

#if (BLE_APP_SEC)
    // Security Module
    app_sec_init();
#endif // (BLE_APP_SEC)

#if (BLE_APP_HT)
    // Health Thermometer Module
    app_ht_init();
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
    // Device Information Module
    app_dis_init();
#endif //(BLE_APP_DIS)

#if (BLE_APP_HID)
    // HID Module
    app_hid_init();
#endif //(BLE_APP_HID)

#if (BLE_APP_BATT)
    // Battery Module
    app_batt_init();
#endif //(BLE_APP_BATT)

#if (BLE_APP_AM0)
    // Audio Mode 0 Module
    app_am0_init();
#endif //(BLE_APP_AM0)

    // Reset the stack
    appm_send_gapm_reset_cmd();
}


void appm_reset(void)
{
    app_env.next_svc = 0;
}



bool appm_add_svc(void)
{
    // Indicate if more services need to be added in the database
    bool more_svc = false;

    // Check if another should be added in the database
    if (app_env.next_svc != APPM_SVC_LIST_STOP)
    {
        ASSERT_INFO(appm_add_svc_func_list[app_env.next_svc] != NULL, app_env.next_svc, 1);

        // Call the function used to add the required service
        appm_add_svc_func_list[app_env.next_svc]();

        // Select following service to add
        app_env.next_svc++;
        more_svc = true;
    }

    return more_svc;
}

void appm_disconnect(void)
{
    struct gapc_disconnect_cmd *cmd = KE_MSG_ALLOC(GAPC_DISCONNECT_CMD,
                                      KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                      gapc_disconnect_cmd);

    cmd->operation = GAPC_DISCONNECT;
    cmd->reason    = CO_ERROR_REMOTE_USER_TERM_CON;

    // Send the message
    ke_msg_send(cmd);
}

static void appm_create_advertising(void)
{
    if (app_env.adv_state == APP_ADV_STATE_IDLE)
    {
        // Prepare the GAPM_ACTIVITY_CREATE_CMD message
        struct gapm_activity_create_adv_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_CREATE_CMD,
                TASK_GAPM, TASK_APP,
                gapm_activity_create_adv_cmd);

        memset(p_cmd, 0, sizeof(struct gapm_activity_create_adv_cmd));
        // Set operation code
        p_cmd->operation = GAPM_CREATE_ADV_ACTIVITY;

#if (BLE_APP_AM0)
        // Let AM0 application module set advertising parameters
        app_am0_fill_create_adv_actv(p_cmd);
#else //(BLE_APP_AM0)
        // Fill the allocated kernel message
        p_cmd->own_addr_type = GAPM_STATIC_ADDR;
        p_cmd->adv_param.type = GAPM_ADV_TYPE_LEGACY;
        p_cmd->adv_param.prop = GAPM_ADV_PROP_UNDIR_CONN_MASK;
        p_cmd->adv_param.filter_pol = ADV_ALLOW_SCAN_ANY_CON_ANY;
        p_cmd->adv_param.prim_cfg.chnl_map = APP_ADV_CHMAP;
        p_cmd->adv_param.prim_cfg.phy = GAP_PHY_LE_1MBPS;

#if (BLE_APP_HID)
        p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_LIM_DISC;

        /*
         * If the peripheral is already bonded with a central device, use the direct advertising
         * procedure (BD Address of the peer device is stored in NVDS.
         */
        if (app_sec_get_bond_status())
        {
//            // BD Address of the peer device
//            struct gap_bdaddr peer_bd_addr;
//            // Length
//            uint8_t length = NVDS_LEN_PEER_BD_ADDRESS;
//
//            // Get the BD Address of the peer device in NVDS
//            if (nvds_get(NVDS_TAG_PEER_BD_ADDRESS, &length, (uint8_t *)&peer_bd_addr) != NVDS_OK)
//            {
//                // The address of the bonded peer device should be present in the database
//                ASSERT_ERR(0);
//            }
//
//            // Set the DIRECT ADVERTISING mode
//            cmd->op.code = GAPM_ADV_DIRECT;
//            // Copy the BD address of the peer device and the type of address
//            memcpy(&cmd->info.direct, &peer_bd_addr, NVDS_LEN_PEER_BD_ADDRESS);

            p_cmd->adv_param.prim_cfg.adv_intv_min = APP_ADV_FAST_INT;
            p_cmd->adv_param.prim_cfg.adv_intv_max = APP_ADV_FAST_INT;
        }
        else
        {
            p_cmd->adv_param.prim_cfg.adv_intv_min = APP_ADV_INT_MIN;
            p_cmd->adv_param.prim_cfg.adv_intv_max = APP_ADV_INT_MAX;
        }
#else //(BLE_APP_HID)
        p_cmd->adv_param.disc_mode = GAPM_ADV_MODE_GEN_DISC;
        p_cmd->adv_param.prim_cfg.adv_intv_min = APP_ADV_INT_MIN;
        p_cmd->adv_param.prim_cfg.adv_intv_max = APP_ADV_INT_MAX;
#endif //(BLE_APP_HID)
#endif //(BLE_APP_AM0)

        // Send the message
        ke_msg_send(p_cmd);

        // Keep the current operation
        app_env.adv_state = APP_ADV_STATE_CREATING;
        // And the next expected operation code for the command completed event
        app_env.adv_op = GAPM_CREATE_ADV_ACTIVITY;
    }
}


void appm_delete_advertising(void)
{
    // Prepare the GAPM_ACTIVITY_CREATE_CMD message
    struct gapm_activity_delete_cmd *p_cmd = KE_MSG_ALLOC(GAPM_ACTIVITY_DELETE_CMD,
            TASK_GAPM, TASK_APP,
            gapm_activity_delete_cmd);

    // Set operation code
    p_cmd->operation = GAPM_DELETE_ALL_ACTIVITIES;

    // Send the message
    ke_msg_send(p_cmd);

    // Keep the current operation
    // And the next expected operation code for the command completed event
    app_env.adv_op = GAPM_DELETE_ALL_ACTIVITIES;
}

void appm_adv_fsm_next(void)
{
    switch (app_env.adv_state)
    {
    case (APP_ADV_STATE_IDLE):
    {
        // Create advertising
        appm_create_advertising();
    }
    break;

    case (APP_ADV_STATE_CREATING):
    {
        // Set advertising data
        appm_set_adv_data();
    }
    break;

    case (APP_ADV_STATE_SETTING_ADV_DATA):
    {
        // Set scan response data
        appm_set_scan_rsp_data();
    }
    break;

    case (APP_ADV_STATE_CREATED):
    case (APP_ADV_STATE_SETTING_SCAN_RSP_DATA):
    {
        // Start advertising activity
        appm_start_advertising();
    }
    break;

    case (APP_ADV_STATE_STARTING):
    {
#if (DISPLAY_SUPPORT)
        // Update advertising state screen
        app_display_set_adv(true);
#endif //(DISPLAY_SUPPORT)

        // Go to started state
        app_env.adv_state = APP_ADV_STATE_STARTED;
    }
    break;

    case (APP_ADV_STATE_STARTED):
    {
        // Stop advertising activity
        appm_stop_advertising();
    }
    break;

    case (APP_ADV_STATE_STOPPING):
    {
#if (DISPLAY_SUPPORT)
        // Update advertising state screen
        app_display_set_adv(false);
#endif //(DISPLAY_SUPPORT)

        // Go created state
        app_env.adv_state = APP_ADV_STATE_CREATING;
    }
    break;



    default:
    {
        ASSERT_ERR(0);
    }
    break;
    }
}

void appm_update_param(struct gapc_conn_param *conn_param)
{
    // Prepare the GAPC_PARAM_UPDATE_CMD message
    struct gapc_param_update_cmd *cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
                                        KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                                        gapc_param_update_cmd);

    cmd->operation  = GAPC_UPDATE_PARAMS;
    cmd->intv_min   = conn_param->intv_min;
    cmd->intv_max   = conn_param->intv_max;
    cmd->latency    = conn_param->latency;
    cmd->time_out   = conn_param->time_out;

    // not used by a slave device
    cmd->ce_len_min = 0xFFFF;
    cmd->ce_len_max = 0xFFFF;

    // Send the message
    ke_msg_send(cmd);
}

void appm_set_dev_name(uint8_t *name, uint8_t name_len)
{
    ASSERT_INFO(name, NULL, NULL);
    app_env.dev_name_len = name_len < NVDS_LEN_DEVICE_NAME ? name_len : NVDS_LEN_DEVICE_NAME;
    memcpy(app_env.dev_name, name, app_env.dev_name_len);
#if (NVDS_SUPPORT)
    sifli_nvds_put(NVDS_TAG_DEVICE_NAME, app_env.dev_name_len, app_env.dev_name);
#endif
}

uint8_t appm_get_dev_name(uint8_t *name)
{
    ASSERT_INFO(name, NULL, NULL);
    // copy name to provided pointer
    memcpy(name, app_env.dev_name, app_env.dev_name_len);
    // return name length
    return app_env.dev_name_len;
}

void appm_set_appearance(uint16_t appearance)
{
    app_env.appearance = appearance;
}

void appm_get_appearance(uint16_t *appearance)
{
    ASSERT_INFO(appearance, NULL, NULL);
#if (BLE_APP_HT)
    // Generic Thermometer - TODO: Use a flag
    *appearance = 728;
#elif (BLE_APP_HID)
    // HID Mouse
    *appearance = 962;
#else
    // No appearance
    *appearance = app_env.appearance;
#endif

}

void appm_set_slv_pref_params(struct gap_slv_pref *pref)
{
    ASSERT_INFO(pref, NULL, NULL);
    memcpy(&app_env.pref, pref, sizeof(struct gap_slv_pref));
}

void appm_get_slv_pref_params(struct gap_slv_pref *pref)
{
    ASSERT_INFO(pref, NULL, NULL);
    memcpy(pref, &app_env.pref, sizeof(struct gap_slv_pref));
}

void appm_update_adv_state(bool start)
{
    // TODO [LT] - Check current advertising state

    // Start or stop advertising
    appm_adv_fsm_next();
}


#endif //(BLE_APP_PRESENT)

/// @} APP
