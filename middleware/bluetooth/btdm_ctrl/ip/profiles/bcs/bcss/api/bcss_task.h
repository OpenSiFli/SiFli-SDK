/**
 ****************************************************************************************
 *
 * @file bcss_task.h
 *
 * @brief Header file - Body Composition Service Server Task.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

#ifndef _BCSS_TASK_H_
#define _BCSS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup BCSSTASK Task
 * @ingroup BCSS
 * @brief Body Composition Service Task.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

//#include "rwip_config.h"

//#if (BLE_BCS_SERVER)
#include "bcs_common.h"

//#include <stdint.h>
#include "rwip_task.h" // Task definitions

/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/// Messages for Weight SCale Profile Sensor
/*@TRACE*/
enum bcss_msg_id
{
    /// Enable the WSCP Sensor task for a connection
    BCSS_ENABLE_REQ = TASK_FIRST_MSG(TASK_ID_BCSS),
    /// Enable the WSCP Sensor task for a connection
    BCSS_ENABLE_RSP,
    /// Indicate a new IND configuration to the application
    /// Send CCC descriptor write update to the APP
    BCSS_WR_CCC_IND,


    /// Send a WSC Measurement to the peer device (Indication)
    BCSS_MEAS_INDICATE_CMD,

    /// Send a complete event status to the application
    BCSS_CMP_EVT,
};


enum bcss_op_codes
{
    BCSS_MEAS_INDICATE_CMD_OP_CODE = 1,
};
/*
 * STRUCTURES
 ****************************************************************************************
 */

/// Parameters of the initialization function
struct bcss_db_cfg
{

    /// Body Composition service Feature Value -
    uint32_t feature;

    uint8_t secondary_service;
};

/// Parameters of the @ref BCSS_ENABLE_REQ message
struct bcss_enable_req
{
    /// CCC for the current connection
    uint16_t ind_cfg;
};

/// Parameters of the @ref WSCS_ENABLE_RSP message
struct bcss_enable_rsp
{
    /// Status
    uint8_t status;
};

/// Parameters of the @ref WSCS_MEAS_INDICATE_CMD message
struct bcss_meas_indicate_cmd
{
    uint16_t flags;
    uint16_t body_fat_percent;
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

/// Parameters of the @ref WSCS_WR_CCC_IND message
struct bcss_wr_ccc_ind
{
    /// Char. Client Characteristic Configuration
    uint16_t ind_cfg;
};

/// Parameters of the @ref WSCS_CMP_EVT message
struct bcss_cmp_evt
{
    /// operation
    uint8_t operation;
    /// Operation Status
    uint8_t  status;
};

//#endif //(BLE_BCS_SERVER)

/// @} BCSSTASK

#endif //(_BCSS_TASK_H_)
