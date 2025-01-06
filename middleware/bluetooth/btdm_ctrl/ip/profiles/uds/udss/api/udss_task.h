/**
 ****************************************************************************************
 *
 * @file udss_task.h
 *
 * @brief Header file - User Data Service Profile Task.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 * $ Rev $
 *
 ****************************************************************************************
 */

#ifndef _UDSS_TASK_H_
#define _UDSS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup ENVSTASK Task
 * @ingroup ENVS
 * @brief User Data Service Profile Task.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_UDS_SERVER)
#include "uds_common.h"

#include <stdint.h>
#include "ke_task.h"

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Used in Complete event to indicate the operation
enum udss_op_codes
{
    UDSS_RESERVED_OP_CODE = 0,
    UDSS_READ_OP_CODE     = 1,
    UDSS_WRITE_OP_CODE    = 2,
    UDSS_INDICATE_OP_CODE = 3,
    UDSS_NOTIFY_OP_CODE   = 4
};

/// Messages for User Data Service Profile Sensor
/*@TRACE*/
enum udss_msg_id
{
    /// Enable the Service task - at connection
    UDSS_ENABLE_REQ = TASK_FIRST_MSG(TASK_ID_UDSS),
    /// Response to Enable the Service task - at connection
    UDSS_ENABLE_RSP,

    /// Send String Characteristic write update to the APP
    UDSS_WR_CHAR_UTF8_REQ_IND,
    /// Send Rest Characteristic write update to the APP
    UDSS_WR_CHAR_REQ_IND,

    /// write confirmation Characteristic from the APP
    UDSS_WR_CHAR_CFM,

    /// Send Rest Characteristic Read Request to the APP
    UDSS_RD_CHAR_REQ_IND,

    /// Read confirmation String Characteristic from the APP
    UDSS_RD_CHAR_UTF8_CFM,
    /// Read confirmation Characteristic from the APP
    UDSS_RD_CHAR_CFM,

    /// Database Change Increment
    /// Write DB_Change Inc Characteristic CCC update to the APP
    UDSS_WR_CHAR_CCC_IND,

    /// Indication DB_Change Inc Characteristic from the APP
    UDSS_DB_CHG_INC_NOTIFY_CMD,

    /// User Control Point
    /// Send User Control_Point Characteristic write update to the APP
    UDSS_WR_USER_CTRL_PT_REQ_IND,
    /// Indication User Control_Point Characteristic from the APP
    UDSS_USER_CTRL_PT_INDICATE_CMD,

    /// Send a complete event status to the application
    UDSS_CMP_EVT,
};


/*
 * STRUCTURES
 ****************************************************************************************
 */

/// Parameters of the initialization function
/// bitmask of enabled Characteristics ref@ uds_char_id
struct udss_db_cfg
{
    /// Database configuration
    uint32_t char_mask;
#ifdef USE_EXTRA_CHARS
    // extra UUID to support
    uint16_t UUID1;
    uint16_t UUID2;
#endif //def USE_EXTRA_CHARS
};



union char_val
{
    struct prf_date date;
    uint32_t db_chg_inc; //for db_chg_inc

    uint8_t user_idx;
    uint8_t age;
    uint8_t gender;
    uint8_t vo2_max;
    uint8_t max_heart_rate;
    uint8_t resting_heart_rate;
    uint8_t maximum_recommended_heart_rate;
    uint8_t aerobic_threshold;
    uint8_t anaerobic_threshold;
    uint8_t sport_type_for_aerobic_and_anaerobic_thresholds;
    uint8_t fat_burn_heart_rate_lower_limit;
    uint8_t fat_burn_heart_rate_upper_limit;
    uint8_t aerobic_heart_rate_lower_limit;
    uint8_t aerobic_heart_rate_upper_limit;
    uint8_t anaerobic_heart_rate_lower_limit;
    uint8_t anaerobic_heart_rate_upper_limit;
    uint8_t two_zone_heart_rate_limit;

    uint16_t uint16;
    uint16_t weight;
    uint16_t height;
    uint16_t waist_circumference;
    uint16_t hip_circumference;

    uint8_t  set[4];
};

/*
 * Messages Exchange Parameters
 ****************************************************************************************
 */

/// Parameters of the @ref UDSS_ENABLE_REQ message
struct udss_enable_req
{
    uint16_t db_chg_inc_ccc;
    uint16_t user_ctrl_pt_ccc;
};

/// Parameters of the @ref UDSS_ENABLE_RSP message
struct udss_enable_rsp
{
    /// Operation Status
    uint8_t status;
};

/// Parameters of the @ref UDSS_WR_CHAR_UTF8_REQ_IND message
struct udss_wr_char_utf8_req_ind
{
    // characteristic id @ref uds_char_id
    uint8_t char_idx;
    // offset??? do we need this for long messages
    uint8_t offset;
    /// Actual data for the string utf8 characteristic
    struct utf_8_name utf_name;
};


/// Parameters of the @ref UDSS_WR_CHAR_REQ_IND message
struct udss_wr_char_req_ind
{
    // characteristic id @ref uds_char_id
    uint8_t char_idx;
    // data for the characteristic
    union char_val value;
};

/// Parameters of the @ref UDSS_WR_CHAR_CFM message
struct udss_wr_char_cfm
{
    // characteristic id @ref uds_char_id
    uint8_t char_idx;
    /// Operation Status
    uint8_t status;
};

///// Parameters of the @ref UDSS_RD_CHAR_UTF8_REQ_IND message
//struct udss_rd_char_utf8_req_ind
//{
//    // characteristic id @ref uds_char_id
//    uint8_t char_idx;
//};

/// Parameters of the @ref UDSS_RD_CHAR_REQ_IND message
struct udss_rd_char_req_ind
{
    // characteristic id @ref uds_char_id
    uint8_t char_idx;
};

/// Parameters of the @ref UDSS_RD_CHAR_UTF8_CFM message
struct udss_rd_char_utf8_cfm
{
    // characteristic id @ref uds_char_id
    uint8_t char_idx;
    /// Operation Status
    uint8_t status;
    /// Actual data for the string utf8 characteristic
    struct utf_8_name utf_name;
};

/// Parameters of the @ref UDSS_RD_CHAR_CFM message
struct udss_rd_char_cfm
{
    // characteristic id @ref uds_char_id
    uint8_t char_idx;
    /// Operation Status
    uint8_t status;
    // data for the characteristic
    union char_val value;
};

///// Parameters of the @ref UDSS_WR_DB_CHG_INC_REQ_IND message
//struct udss_wr_db_chg_inc_req_ind
//{
//    // characteristic value
//    uint32_t value;
//};

///// Parameters of the @ref UDSS_RD_DB_CHG_INC_CFM message
//struct udss_rd_db_chg_inc_cfm
//{
//    // characteristic id @ref uds_char_id
//    uint8_t char_idx;
//    /// Operation Status
//    uint8_t status;
//    // characteristic value
//    uint32_t value;
//};

/// Parameters of the @ref UDSS_WR_CHAR_CCC_IND message
struct udss_wr_char_ccc_ind
{
    // characteristic id @ref uds_char_id
    uint8_t char_idx;
    // Characteristic CCC
    uint16_t ind_cfg;
};

/// Parameters of the @ref UDSS_DB_CHG_INC_NOTIFY_CMD message
struct udss_db_chg_inc_notify_cmd
{
    // characteristic value
    uint32_t value;
};

/// User Index
/// Parameters of the @ref UDSS_WR_USER_CTRL_PT_REQ_IND message
struct udss_wr_user_ctrl_pt_req_ind
{
    uint8_t op_code;
    // used for sending consent command
    uint8_t user_id;
    // for registering new user and selecting existing
    uint16_t consent;
    /// for the future extension  to send additional commands
    uint8_t length;
    uint8_t parameter[__ARRAY_EMPTY];
};

/// Parameters of the @ref UDSS_WR_USER_CTRL_PT_CFM message
struct udss_wr_user_ctrl_pt_cfm
{
    /// Operation Status
    uint8_t status;
};

/// Parameters of the @ref UDSS_USER_CTRL_PT_INDICATE_CMD message
struct udss_user_ctrl_pt_indicate_cmd
{
    uint8_t resp_code; // response op-code =0x20
    uint8_t req_op_code; // requested op-code
    uint8_t resp_value;
    uint8_t length;
    uint8_t parameter[__ARRAY_EMPTY];
};


/*
 * Confirmation to the APP after Update, Indicate, Notify messages
 ****************************************************************************************
 */
/// Parameters of the @ref UDSS_CMP_EVT message
struct udss_cmp_evt
{
    /// Operation Code
    uint8_t operation;
    /// Operation Status
    uint8_t status;
};


#endif //(BLE_UDS_SERVER)

/// @} ENVSTASK

#endif //(_UDSS_TASK_H_)

