/**
 ****************************************************************************************
 *
 * @file scppc.h
 *
 * @brief Header file - Scan Parameters Profile -Client Role.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */


#ifndef _SCPPC_H_
#define _SCPPC_H_


/**
 ****************************************************************************************
 * @addtogroup SCPPC Scan Parameters Profile Client
 * @ingroup SCPP
 * @brief Scan Parameters Profile Client
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#if (BLE_SP_CLIENT)

#include "scpp_common.h"
#include "scppc_task.h"
#include "ke_task.h"
#include "prf_types.h"
#include "prf_utils.h"
#include "prf.h"


/*
 * Defines
 ****************************************************************************************
 */

///Maximum number of Scan Parameters Client task instances
#define SCPPC_IDX_MAX    (BLE_CONNECTION_MAX)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Possible states of the SCPPC task
enum scppc_state
{
    /// Disconnected state
    SCPPC_FREE,
    /// IDLE state
    SCPPC_IDLE,
    /// busy state
    SCPPC_BUSY,
    /// Number of defined states.
    SCPPC_STATE_MAX
};

enum scppc_operation
{
    /// No operation
    SCPPC_OP_NONE = 0,
    /// Discover peer service
    SCPPC_OP_DISCOVER,
    /// Write Notification configuration
    SCPPC_OP_WR_NTF_CFG,
    /// Read Notification configuration
    SCPPC_OP_RD_NTF_CFG,
    /// Write Scan intervals
    SCPPC_OP_WR_SCAN_INTV,
};



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/// Environment variable for each Connections
struct scppc_cnx_env
{
    ///SCPS characteristics
    struct scps_content scps;
    /// on-going operation
    uint8_t operation;
    /// counter used to check service uniqueness
    uint8_t nb_svc;
};

/// Scan Parameters Profile Client environment variable
struct scppc_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// Environment variable pointer for each connections
    struct scppc_cnx_env *env[SCPPC_IDX_MAX];
    /// State of different task instances
    ke_state_t state[SCPPC_IDX_MAX];
};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve SCPP client profile interface
 *
 * @return SCPP client profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *scppc_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Send SCP ATT DB discovery results to SCPPC host.
 ****************************************************************************************
 */
void scppc_enable_rsp_send(struct scppc_env_tag *scppc_env, uint8_t conidx, uint8_t status);

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
void scppc_task_init(struct ke_task_desc *task_desc);

#endif /* (BLE_SP_CLIENT) */

/// @} SCPPC

#endif /* _SCPPC_H_ */
