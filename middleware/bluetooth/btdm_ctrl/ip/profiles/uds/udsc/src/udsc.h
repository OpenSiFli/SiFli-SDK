/**
 ****************************************************************************************
 *
 * @file udsc.h
 *
 * @brief Header file - User Data Service - Collector Role.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

#ifndef _UDSC_H_
#define _UDSC_H_

/**
 ****************************************************************************************
 * @addtogroup UDSC User Data Service  Profile Collector
 * @ingroup UDSP
 * @brief User Data Service Collector
 * @{
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of User Data Service Task instances
#define UDSC_IDX_MAX        (BLE_CONNECTION_MAX)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "uds_common.h"

#if (BLE_UDS_CLIENT)

#include "ke_task.h"
#include "prf_types.h"
#include "prf_utils.h"

//#define CCC_READY

/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/// Possible states of the UDSC task
enum udsc_states
{
    /// Idle state
    UDSC_FREE,
    /// Connected state
    UDSC_IDLE,
    /// Discovering Services
    UDSC_DISCOVERING_SVC,
    /// Busy state
    UDSC_BUSY,
    /// Number of defined states.
    UDSC_STATE_MAX
};

/// Operation Codes for pending operations in the client.
enum udsc_op_codes
{
    /// Reserved operation code
    UDSC_RESERVED_OP_CODE = 0x00,

    /// Read op-codes
    UDSC_READ_CHAR_OP_CODE = 1,

    /// Read op-codes
    UDSC_READ_CCC_OP_CODE = 2,

    /// Write op-codes.
    UDSC_WRITE_CHAR_OP_CODE = 3,
    UDSC_WRITE_CCC_OP_CODE = 4,

    UDSC_ENABLE_CMD_OP_CODE = 5

};

/*
 * STRUCTURES
 ****************************************************************************************
 */

/**
 *  Stored Environment content/context information for a connection
 */
struct udsc_cnx_env
{
    /// Current Operation
    void *operation;

    uint8_t op_pending;
    ///Last requested UUID(to keep track of the two services and char)
    uint16_t last_uuid_req;
    ///Last service for which something was discovered
    uint16_t last_svc_req;
    /// Last characteristic code used in a read or a write request
    uint16_t last_char_code;
    /// Counter used to check service uniqueness
    uint8_t nb_svc;

    /// User Data Service Service Characteristics
    struct udsc_uds_content uds;
};


/// Enviromental sensing environment variable
struct udsc_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// Environment variable pointer for each connections
    struct udsc_cnx_env *env[UDSC_IDX_MAX];
    /// State of different task instances
    ke_state_t state[UDSC_IDX_MAX];
};



/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Pool of UDS Profile Collector task environments.
extern struct udsc_env_tag **udsc_envs;

/*
 * GLOBAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve UDS client profile interface
 *
 * @return UDS client profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *udsc_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Send User Data Service ATT DB discovery results to UDSC host.
 ****************************************************************************************
 */
void udsc_enable_rsp_send(struct udsc_env_tag *udsc_env, uint8_t conidx, uint8_t status);

/**
 ****************************************************************************************
 * @brief Send a UDSC_CMP_EVT message to the task which enabled the profile
 ****************************************************************************************
 */
void udsc_send_cmp_evt(struct udsc_env_tag *udsc_env, uint8_t conidx, uint8_t operation, uint8_t status);



/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * Initialize task handler
 *
 * @param task_desc Task descriptor to fill
 ****************************************************************************************
 */
void udsc_task_init(struct ke_task_desc *task_desc);

#endif //(BLE_UDS_CLIENT)

/// @} UDSC

#endif //(_UDSC_H_)
