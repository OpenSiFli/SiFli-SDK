/**
 ****************************************************************************************
 *
 * @file wscs.h
 *
 * @brief Header file - Weight SCale Profile Sensor.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

#ifndef _WSCS_H_
#define _WSCS_H_

/**
 ****************************************************************************************
 * @addtogroup WSCS Weight SCale Profile Sensor
 * @ingroup CSCP
 * @brief Weight SCale Profile Sensor
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_WSC_SERVER)
#include "wsc_common.h"
#include "wscs_task.h"

#include "prf_types.h"
#include "prf.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of Weight SCale Profile Sensor role task instances
#define WSCS_IDX_MAX        (BLE_CONNECTION_MAX)



/*
 * ENUMERATIONS
 ****************************************************************************************
 */
/// Possible states of the TIPS task
enum
{
    /// not connected state
    WSCS_FREE,
    /// idle state
    WSCS_IDLE,

    /// indicate
    WSCS_OP_INDICATE,

    /// Number of defined states.
    ENVS_STATE_MAX
};



/// Weight SCale Service - Attribute List
enum wscs_att_list
{
    /// Weight SCale Service
    WSCS_IDX_SVC,
    /// Included Body Composition Service
    WSCS_IDX_INC_SVC,
    /// Weight SCale Feature Characteristic
    WSCS_IDX_FEAT_CHAR,
    WSCS_IDX_FEAT_VAL,
    /// Weight SCale Measurement Characteristic
    WSCS_IDX_MEAS_CHAR,
    WSCS_IDX_MEAS_IND,
    /// CCC Descriptor
    WSCS_IDX_MEAS_CCC,

    /// Number of attributes
    WSCS_IDX_NB,
};

//32bit flag
#define WSCS_FEAT_VAL_SIZE   (4)
#define WSCS_MEAS_IND_SIZE sizeof(struct wscs_meas_ind)

/*
 * STRUCTURES
 ****************************************************************************************
 */


/// Weight Measurement
struct wscs_meas_ind
{
    uint8_t flags;
    uint16_t weight;
    struct prf_date_time time_stamp;
    uint8_t user_id;
    uint16_t bmi;
    uint16_t height;
};


/// Weight SCale Profile Sensor environment variable
struct wscs_env_tag
{
    /// profile environment
    prf_env_t prf_env;

    /// Weight SCale Service Start Handle
    uint16_t shdl;
    /// Feature configuration
    uint32_t feature;
    /// BCS Service Included
    uint8_t bcs_included;
    /// CCC for each connections
    uint8_t prfl_ind_cfg[BLE_CONNECTION_MAX];

    /// State of different task instances
    ke_state_t state[WSCS_IDX_MAX];

};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

void wscs_send_cmp_evt(struct wscs_env_tag *wscs_env, uint8_t conidx, uint8_t operation, uint8_t status);

/**
 ****************************************************************************************
 * @brief Retrieve WSCP service profile interface
 *
 * @return WSCP service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *wscs_prf_itf_get(void);


/**
 ****************************************************************************************
 * @brief  This function sends control point indication error
 ****************************************************************************************
 */
void wscs_send_rsp_ind(uint8_t conidx, uint8_t req_op_code, uint8_t status);


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
void wscs_task_init(struct ke_task_desc *task_desc);

#endif //(BLE_WSC_SERVER)

/// @} WSCS

#endif //(_WSCS_H_)
