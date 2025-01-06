/**
 ****************************************************************************************
 *
 * @file scpps.h
 *
 * @brief Header file - Scan Parameters Profile Server.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

#ifndef _SCPPS_H_
#define _SCPPS_H_

/**
 ****************************************************************************************
 * @addtogroup SCPPS Scan Parameters Profile Server
 * @ingroup SCPP
 * @brief Scan Parameters Profile Server
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if (BLE_SP_SERVER)

#include "scpp_common.h"
#include "scpps_task.h"
#include "prf_types.h"
#include "prf.h"

/*
 * DEFINES
 ****************************************************************************************
 */

///Maximum number of Scan Parameters Device task instances
#define SCPPS_IDX_MAX     0x01

#define SCPPS_CFG_FLAG_MANDATORY_MASK       (0x07)
#define SCPPS_CFG_FLAG_SCAN_REFRESH_MASK    (0x38)

/// Scanning set parameters range min
#define SCPPS_SCAN_INTERVAL_MIN                    4//(0x4)
/// Scanning set parameters range max
#define SCPPS_SCAN_INTERVAL_MAX                    16384//(0x4000)

/// Scanning set parameters range min
#define SCPPS_SCAN_WINDOW_MIN                    4//(0x4)
/// Scanning set parameters range max
#define SCPPS_SCAN_WINDOW_MAX                    16384//(0x4000)


/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Possible states of the SCPPS task
enum scpps_state
{
    /// Idle state
    SCPPS_IDLE,
    /// Busy state
    SCPPS_BUSY,

    /// Number of defined states.
    SCPPS_STATE_MAX
};

/// Scan Parameters Service Attributes Indexes
enum
{
    SCPS_IDX_SVC,

    SCPS_IDX_SCAN_INTV_WD_CHAR,
    SCPS_IDX_SCAN_INTV_WD_VAL,

    SCPS_IDX_SCAN_REFRESH_CHAR,
    SCPS_IDX_SCAN_REFRESH_VAL,
    SCPS_IDX_SCAN_REFRESH_NTF_CFG,

    SCPS_IDX_NB,
};


enum
{
    SCPP_SERVER_REQUIRES_REFRESH    = 0x00,
};



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Scan Parameters Profile Server environment variable
struct scpps_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// SCPS Start Handle
    uint16_t start_hdl;
    /// Database configuration
    uint8_t features;
    /// Notification configuration of peer devices.
    uint8_t ntf_cfg[BLE_CONNECTION_MAX];
    /// BASS task state
    ke_state_t state[SCPPS_IDX_MAX];

};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve SCPPS service profile interface
 *
 * @return SCPPS service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *scpps_prf_itf_get(void);


/**
 ****************************************************************************************
 * @brief Send a SCPPS_SCAN_REFRESH_SEND_RSP message to the application
 ****************************************************************************************
 */
void scpps_scan_refresh_rsp_send(struct scpps_env_tag *scpps_env, uint8_t conidx, uint8_t status);

/**
 ****************************************************************************************
 * @brief Disable actions grouped in getting back to IDLE and sending configuration to requester task
 ****************************************************************************************
 */
void scpps_disable(uint16_t conhdl);

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
void scpps_task_init(struct ke_task_desc *task_desc);


#endif /* #if (BLE_SP_SERVER) */

/// @} SCPPS

#endif /* _SCPPS_H_ */
