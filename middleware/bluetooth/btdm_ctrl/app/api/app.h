/**
 ****************************************************************************************
 *
 * @file app.h
 *
 * @brief Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef APP_H_
#define APP_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief Application entry point.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_PRESENT)

#include <stdint.h>          // Standard Integer Definition
#include <co_bt.h>           // Common BT Definitions
#include "arch.h"            // Platform Definitions
#include "gapc.h"            // GAPC Definitions

#if (NVDS_SUPPORT)
    #include "sifli_nvds.h"
#endif // (NVDS_SUPPORT)

#include "gapm.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximal length of the Device Name value
#define APP_DEVICE_NAME_MAX_LEN      (NVDS_LEN_DEVICE_NAME)

/*
 * MACROS
 ****************************************************************************************
 */

#define APP_HANDLERS(subtask)    {&subtask##_msg_handler_list[0], ARRAY_LEN(subtask##_msg_handler_list)}

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

#if (NVDS_SUPPORT)
/// List of Application NVDS TAG identifiers
enum app_nvds_tag
{
    /// BD Address
    NVDS_TAG_BD_ADDRESS                 = PARAM_ID_APP_SPECIFIC_FIRST + 0x01,
    NVDS_LEN_BD_ADDRESS                 = 6,

    /// Device Name
    NVDS_TAG_DEVICE_NAME                = PARAM_ID_APP_SPECIFIC_FIRST + 0x02,
    NVDS_LEN_DEVICE_NAME                = 62,

    /// BLE Application Advertising data
    NVDS_TAG_APP_BLE_ADV_DATA           = PARAM_ID_APP_SPECIFIC_FIRST + 0x03,
    NVDS_LEN_APP_BLE_ADV_DATA           = 32,

    /// BLE Application Scan response data
    NVDS_TAG_APP_BLE_SCAN_RESP_DATA     = PARAM_ID_APP_SPECIFIC_FIRST + 0x04,
    NVDS_LEN_APP_BLE_SCAN_RESP_DATA     = 32,

    /// Mouse Sample Rate
    NVDS_TAG_MOUSE_SAMPLE_RATE          = PARAM_ID_APP_SPECIFIC_FIRST + 0x05,
    NVDS_LEN_MOUSE_SAMPLE_RATE          = 1,

    /// Peripheral Bonded
    NVDS_TAG_PERIPH_BONDED              = PARAM_ID_APP_SPECIFIC_FIRST + 0x06,
    NVDS_LEN_PERIPH_BONDED              = 1,

    /// Mouse NTF Cfg
    NVDS_TAG_MOUSE_NTF_CFG              = PARAM_ID_APP_SPECIFIC_FIRST + 0x07,
    NVDS_LEN_MOUSE_NTF_CFG              = 2,

    /// Mouse Timeout value
    NVDS_TAG_MOUSE_TIMEOUT              = PARAM_ID_APP_SPECIFIC_FIRST + 0x08,
    NVDS_LEN_MOUSE_TIMEOUT              = 2,

    /// Peer Device BD Address
    NVDS_TAG_PEER_BD_ADDRESS            = PARAM_ID_APP_SPECIFIC_FIRST + 0x09,
    NVDS_LEN_PEER_BD_ADDRESS            = 7,

    /// Mouse Energy Safe
    NVDS_TAG_MOUSE_ENERGY_SAFE          = PARAM_ID_APP_SPECIFIC_FIRST + 0x0A,
    NVDS_LEN_MOUSE_SAFE_ENERGY          = 2,

    /// EDIV (2bytes), RAND NB (8bytes),  LTK (16 bytes), Key Size (1 byte)
    NVDS_TAG_LTK                        = PARAM_ID_APP_SPECIFIC_FIRST + 0x0B,
    NVDS_LEN_LTK                        = 28,

    /// PAIRING
    NVDS_TAG_PAIRING                    = PARAM_ID_APP_SPECIFIC_FIRST + 0x0C,
    NVDS_LEN_PAIRING                    = 54,

    /// Audio mode 0 task
    NVDS_TAG_AM0_FIRST                  = PARAM_ID_APP_SPECIFIC_FIRST + 0x0D,
    NVDS_TAG_AM0_LAST                   = PARAM_ID_APP_SPECIFIC_FIRST + 0x1D,

    /// Local device Identity resolving key
    NVDS_TAG_LOC_IRK                    = PARAM_ID_APP_SPECIFIC_FIRST + 0x1E,
    NVDS_LEN_LOC_IRK                    = KEY_LEN,

    /// Peer device Resolving identity key (+identity address)
    NVDS_TAG_PEER_IRK                   = PARAM_ID_APP_SPECIFIC_FIRST + 0x1F,
    NVDS_LEN_PEER_IRK                   = sizeof(struct gapc_irk),

    NVDS_TAG_APP_EDN                    = PARAM_ID_APP_SPECIFIC_LAST,
};
#endif // (NVDS_SUPPORT)

/// Advertising state machine
enum app_adv_state
{
    /// Advertising activity does not exists
    APP_ADV_STATE_IDLE = 0,
    /// Creating advertising activity
    APP_ADV_STATE_CREATING,
    /// Setting advertising data
    APP_ADV_STATE_SETTING_ADV_DATA,
    /// Setting scan response data
    APP_ADV_STATE_SETTING_SCAN_RSP_DATA,

    /// Advertising activity created
    APP_ADV_STATE_CREATED,
    /// Starting advertising activity
    APP_ADV_STATE_STARTING,
    /// Advertising activity started
    APP_ADV_STATE_STARTED,
    /// Stopping advertising activity
    APP_ADV_STATE_STOPPING,

};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Structure containing information about the handlers for an application subtask
struct app_subtask_handlers
{
    /// Pointer to the message handler table
    const struct ke_msg_handler *p_msg_handler_tab;
    /// Number of messages handled
    uint16_t msg_cnt;
};

/// Application environment structure
struct app_env_tag
{
    /// Connection handle
    uint16_t conhdl;
    /// Connection Index
    uint8_t  conidx;

    /// Advertising activity index
    uint8_t adv_actv_idx;
    /// Current advertising state (@see enum app_adv_state)
    uint8_t adv_state;
    /// Next expected operation completed event
    uint8_t adv_op;

    /// Last initialized profile
    uint8_t next_svc;

    /// Bonding status
    bool bonded;

    /// Device Name length
    uint8_t dev_name_len;
    /// Device Name
    uint8_t dev_name[APP_DEVICE_NAME_MAX_LEN];

    uint16_t appearance;

    struct gap_slv_pref pref;

    /// Local device IRK
    uint8_t loc_irk[KEY_LEN];

    /// Secure Connections on current link
    bool sec_con_enabled;

    /// Start advertisement after reset
    bool start_adv;

    /// Counter used to generate IRK
    uint8_t rand_cnt;
};

/*
 * GLOBAL VARIABLE DECLARATION
 ****************************************************************************************
 */

/// Application environment
extern struct app_env_tag app_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the BLE demo application.
 ****************************************************************************************
 */
void appm_init(void);
void appm_reset(void);

/**
 ****************************************************************************************
 * @brief Add a required service in the database
 ****************************************************************************************
 */
bool appm_add_svc(void);


/**
 ****************************************************************************************
 * @brief
 ****************************************************************************************
 */
void appm_adv_fsm_next(void);

/**
 ****************************************************************************************
 * @brief Send to request to update the connection parameters
 ****************************************************************************************
 */
void appm_update_param(struct gapc_conn_param *conn_param);

/**
 ****************************************************************************************
 * @brief Send a disconnection request
 ****************************************************************************************
 */
void appm_disconnect(void);

/**
 ****************************************************************************************
 * @brief Retrieve device name
 *
 * @param[out] device name
 *
 * @return name length
 ****************************************************************************************
 */
uint8_t appm_get_dev_name(uint8_t *name);

/**
 ****************************************************************************************
 * @brief Start/stop advertising
 *
 * @param[in] start     True if advertising has to be started, else false
 ****************************************************************************************
 */
void appm_update_adv_state(bool start);

/**
 ****************************************************************************************
 * @brief delete advertising
 *
 * @param[in] none
 ****************************************************************************************
 */

void appm_delete_advertising(void);
/**
 ****************************************************************************************
 * @brief Return if the device is currently bonded
 ****************************************************************************************
 */
bool app_sec_get_bond_status(void);

void appm_task_init(void);
void appm_set_dev_name(uint8_t *name, uint8_t name_len);
void appm_set_appearance(uint16_t appearance);
void appm_get_appearance(uint16_t *appearance);
void appm_set_slv_pref_params(struct gap_slv_pref *pref);
void appm_get_slv_pref_params(struct gap_slv_pref *pref);
void appm_update_adv_state(bool start);



/// @} APP

#endif //(BLE_APP_PRESENT)

#endif // APP_H_
