/**
 ****************************************************************************************
 *
 * @file bcsc_task.h
 *
 * @brief Header file - Body Composition Collector/Client Role Task.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

#ifndef _BCSC_TASK_H_
#define _BCSC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup BCSC  Body Composition Service Collector Task
 * @ingroup BCSC
 * @brief  Body Composition Service Collector
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

//#include "rwip_config.h"

//#if (BLE_BCS_CLIENT)

#include "rwip_task.h" // Task definitions
#include "prf_types.h"
#include "bcs_common.h"

/*
 * ENUMERATIONS
 ****************************************************************************************
 */


/// Message IDs
/*@TRACE*/
enum bcsc_msg_ids
{
    /// Enable the Profile Collector task - at connection
    BCSC_ENABLE_REQ = TASK_FIRST_MSG(TASK_ID_BCSC),
    /// Response to Enable the Profile Collector task - at connection
    BCSC_ENABLE_RSP,
    ///*** BCS CHARACTERISTIC/DESCRIPTOR READ REQUESTS
    /// Read the Body Compositio Feature
    BCSC_RD_FEATURE_REQ,
    /// Read the CCC of a measurement sensor characteristic
    BCSC_RD_MEAS_CCC_REQ,

    ///***  CHARACTERISTIC/DESCRIPTOR WRITE REQUESTS
    /// Write the CCC of a measurement sensor characteristic
    BCSC_WR_MEAS_CCC_CMD,

    ///*** WSC CHARACTERISTIC/DESCRIPTOR READ RESPONSE
    /// Read the Body Composition Feature
    BCSC_RD_FEATURE_RSP,
    /// Read the CCC of a measurement sensor characteristic
    BCSC_RD_MEAS_CCC_RSP,

    /// Characteristic Measurement Indication from peer
    BCSC_MEAS_IND,
    /// Complete Event Information
    BCSC_CMP_EVT,
};


/// Body Composition Service Characteristics
enum
{
    /// Features
    BCSC_CHAR_BCS_FEATURE,
    /// Measurement
    BCSC_CHAR_BCS_MEAS,

    BCSC_CHAR_BCS_MAX,
};

/// Body Composition Service Characteristic Descriptors
enum
{
    /// Client config
    BCSC_DESC_BCS_MEAS_CCC,

    BCSC_DESC_BCS_MAX,
};

/**
 * Structure containing the characteristics handles, value handles and descriptors for
 * the Body Composition Service
 */
struct bcsc_bcs_content
{
    /// service info
    struct prf_svc svc;

    /// Characteristic info:
    ///  - Feature
    ///  - Measurement
    struct prf_char_inf chars[BCSC_CHAR_BCS_MAX];

    /// Descriptor handles:
    ///  - Client cfg
    struct prf_char_desc_inf descs[BCSC_DESC_BCS_MAX];
};




/*
 * API MESSAGE STRUCTURES
 ****************************************************************************************
 */

/// Parameters of the @ref BCSC_ENABLE_REQ message
struct bcsc_enable_req
{
    /// Connection type
    uint8_t con_type;

    /// Primary or Secondary Service
    uint8_t svc_type;

    /// If Secondary Service - the Start and End Handle for the service are contained
    /// in the bcs.

    /// Existing handle values of BCS
    struct bcsc_bcs_content bcs;

};

/// Parameters of the @ref BCSC_ENABLE_RSP message
struct bcsc_enable_rsp
{
    /// status
    uint8_t status;
    ///  handle values of BCS
    struct bcsc_bcs_content bcs;
};



///*** BCS CHARACTERISTIC/DESCRIPTOR READ REQUESTS
/// Parameters of the @ref BCSC_RD_FEATURE_REQ message
/// Read the Body Composition Feature
struct bcsc_rd_feature_req
{
    /// dummy
    uint8_t dummy;
};

/// Parameters of the @ref BCSC_RD_MEAS_CCC_REQ message
/// Read the CCC of a measurement sensor characteristic
struct bcsc_rd_meas_ccc_req
{
    /// dummy
    uint8_t dummy;
};


///***  CHARACTERISTIC/DESCRIPTOR WRITE REQUESTS
/// Parameters of the @ref BCSC_WR_MEAS_CCC_CMD message
/// Write the CCC of a measurement sensor characteristic
struct bcsc_wr_meas_ccc_cmd
{
    uint16_t ccc;
};

///*** BCS CHARACTERISTIC/DESCRIPTOR READ RESPONSE
/// Parameters of the @ref BCSC_RD_FEATURE_RSP message
/// Read the Body Composition Feature
struct bcsc_rd_feature_rsp
{
    uint32_t feature;
    uint8_t  status;
};

/// Parameters of the @ref BCSC_RD_MEAS_CCC_RSP message
/// Read the CCC of a measurement sensor characteristic
struct bcsc_rd_meas_ccc_rsp
{
    uint8_t  status;
    uint16_t ccc;
};


/// Parameters of the @ref BCSC_MEAS_IND message
/// Characteristic Measurement Indication from peer
struct bcsc_meas_ind
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


/// Parameters of the @ref BCSC_CMP_EVT message
struct bcsc_cmp_evt
{
    /// Operation code
    uint8_t operation;
    /// Status
    uint8_t status;
};

//#endif //(BLE_BCS_CLIENT)
/// @} BCSC TASK

#endif //(_BCSC_TASK_H_)
