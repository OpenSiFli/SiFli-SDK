/**
 ****************************************************************************************
 *
 * @file wscs.h
 *
 * @brief Header file - Body Composition Service Sensor.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

#ifndef _BCSS_H_
#define _BCSS_H_

/**
 ****************************************************************************************
 * @addtogroup BCSS Body Composition Service Sensor
 * @ingroup CSCP
 * @brief Body Composition Service Sensor
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if 1//(BLE_BCS_SERVER)
#include "bcs_common.h"
#include "bcss_task.h"

#include "prf_types.h"
#include "prf.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of Body Composition  Sensor role task instances
#define BCSS_IDX_MAX        (BLE_CONNECTION_MAX)



/*
 * ENUMERATIONS
 ****************************************************************************************
 */
/// Possible states of the Body Composition Service task
enum
{
    /// not connected state
    BCSS_FREE,
    /// idle state
    BCSS_IDLE,

    /// indicate
    BCSS_OP_INDICATE,

    /// Number of defined states.
    BCSS_STATE_MAX
};



/// Body Composition Service - Attribute List
enum bcss_bcs_att_list
{
    /// Body Composition Service
    BCSS_IDX_SVC,
    /// Body Composition Feature Characteristic
    BCSS_IDX_FEAT_CHAR,
    BCSS_IDX_FEAT_VAL,
    /// Body Composition Measurement Characteristic
    BCSS_IDX_MEAS_CHAR,
    BCSS_IDX_MEAS_IND,
    /// CCC Descriptor
    BCSS_IDX_MEAS_CCC,

    /// Number of attributes
    BCSS_IDX_NB,
};

//32bit flag
#define BCSS_FEAT_VAL_SIZE   (4)
#define BCSS_MEAS_IND_SIZE sizeof(struct bcss_meas_ind)

/*
 * STRUCTURES
 ****************************************************************************************
 */


/// Body Composition Measurement
struct bcss_meas_ind
{
    uint8_t flags;
    struct prf_date_time time_stamp;
    uint8_t user_id;
    uint16_t basal_metab;
    uint16_t muscle_percent;
    uint16_t muscle_mass;
    uint16_t fat_free_mass;
    uint16_t soft_lean_mass;
    uint16_t body_water_mass;
    uint16_t impedance;
    uint16_t weight;
    uint16_t height;
};


/// Body Composition Service Sensor environment variable
struct bcss_env_tag
{
    /// profile environment
    prf_env_t prf_env;

    /// Body Composition Service Start Handle
    uint16_t shdl;
    /// Feature configuration
    uint32_t feature;

    /// CCC for each connections
    uint8_t prfl_ind_cfg[BLE_CONNECTION_MAX];

    /// State of different task instances
    ke_state_t state[BCSS_IDX_MAX];

    struct ke_msg *meas_cmd_msg[BLE_CONNECTION_MAX];

};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

void bcss_send_cmp_evt(struct bcss_env_tag *bcss_env, uint8_t conidx, uint8_t operation, uint8_t status);

/**
 ****************************************************************************************
 * @brief Retrieve BSCS service profile interface
 *
 * @return WSCP service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *bcss_prf_itf_get(void);


/**
 ****************************************************************************************
 * @brief
 ****************************************************************************************
 */
void bcss_send_rsp_ind(uint8_t conidx, uint8_t req_op_code, uint8_t status);


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
void bcss_task_init(struct ke_task_desc *task_desc);

#endif //(BLE_BSC_SERVER)

/// @} BSCS

#endif //(_BSCS_H_)
