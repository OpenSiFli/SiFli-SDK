/**
 ****************************************************************************************
 *
 * @file wscs_task.h
 *
 * @brief Header file - Weight SCale Profile Sensor Task.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

#ifndef _WSCS_TASK_H_
#define _WSCS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup WSCSTASK Task
 * @ingroup WSCS
 * @brief Weight SCale Profile Task.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

//#include "rwip_config.h"

//#if (BLE_WSC_SERVER)
#include "wsc_common.h"

#include <stdint.h>
#include "rwip_task.h" // Task definitions

/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/// Messages for Weight SCale Profile Sensor
/*@TRACE*/
enum wscs_msg_id
{
    /// Enable the WSCP Sensor task for a connection
    WSCS_ENABLE_REQ = TASK_FIRST_MSG(TASK_ID_WSCS),
    /// Enable the WSCP Sensor task for a connection
    WSCS_ENABLE_RSP,
    /// Indicate a new IND configuration to the application
    /// Send CCC descriptor write update to the APP
    WSCS_WR_CCC_IND,


    /// Send a WSC Measurement to the peer device (Indication)
    WSCS_MEAS_INDICATE_CMD,

    /// Send a complete event status to the application
    WSCS_CMP_EVT,
};


enum wscs_op_codes
{
    WSCS_MEAS_INDICATE_CMD_OP_CODE = 1,
};
/*
 * STRUCTURES
 ****************************************************************************************
 */

/// Parameters of the initialization function
struct wscs_db_cfg
{

    /// Weight SCale service Feature Value -
    uint32_t feature;

    uint16_t bcs_start_hdl;

};

/// Parameters of the @ref WSCS_ENABLE_REQ message
struct wscs_enable_req
{
    /// CCC for the current connection
    uint16_t ind_cfg;
};

/// Parameters of the @ref WSCS_ENABLE_RSP message
struct wscs_enable_rsp
{
    /// Status
    uint8_t status;
};

/// Parameters of the @ref WSCS_MEAS_INDICATE_CMD message
struct wscs_meas_indicate_cmd
{
    uint8_t flags;
    uint16_t weight;
    struct prf_date_time time_stamp;
    uint8_t user_id;
    uint16_t bmi;
    uint16_t height;
};

/// Parameters of the @ref WSCS_WR_CCC_IND message
struct wscs_wr_ccc_ind
{
    /// Char. Client Characteristic Configuration
    uint16_t ind_cfg;
};

/// Parameters of the @ref WSCS_CMP_EVT message
struct wscs_cmp_evt
{
    /// operation
    uint8_t operation;
    /// Operation Status
    uint8_t  status;
};

//#endif //(BLE_WSC_SERVER)

/// @} WSCSTASK

#endif //(_WSCS_TASK_H_)
