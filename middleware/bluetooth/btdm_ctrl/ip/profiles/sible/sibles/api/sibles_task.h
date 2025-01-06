/**
 ****************************************************************************************
 *
 * @file sibles_task.h
 *
 * @brief Header file - SIBLESTASK.
 *
 * Copyright (C) Sifli 2019-2022
 *
 *
 ****************************************************************************************
 */

#ifndef SIBLES_TASK_H_
#define SIBLES_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup SIBLESTASK Task
 * @ingroup SIBLES
 * @brief Sifli BLE Server Task
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include "rwip_task.h" // Task definitions
#include "prf_types.h"
#include "co_bt_defines.h"
/*
 * DEFINES
 ****************************************************************************************
 */
extern void sifli_mbox_bcpu2hcpu(uint8_t *param);

/// Messages for Device Information Service Server
/*@TRACE*/
enum sibles_msg_id
{
    /// APP Write value to Sibles
    ///Set the value of an attribute - Request
    SIBLES_SET_VALUE_REQ = TASK_FIRST_MSG(TASK_ID_SIBLES), //!< SIBLES_SET_VALUE_REQ
    ///Set the value of an attribute - Response
    SIBLES_SET_VALUE_RSP,                               //!< SIBLES_SET_VALUE_RSP

    /// Sibles ask APP for data if not initialized, when peer device try to read.
    /// Peer device request to get profile attribute value
    SIBLES_VALUE_REQ_IND,                               //!< SIBLE_VALUE_REQ_IND
    /// Peer device confirm value of requested attribute
    SIBLES_VALUE_REQ_CFM,                                   //!< SIBLES_VALUE_REQ_CFM

    /// Sibles report to APP that a value has changed, when peer device wrote a attribute.
    /// Indicate a value changes, the change is NOT saved in BLE core. Use SIBLES_VALUE_REQ_IND
    /// to update the firmware cache.
    SIBLES_VALUE_WRITE_IND,                              //!< SIBLES_VALUE_WRITE_IND
    /// APP confirm got the changed value
    SIBLES_VALUE_WRITE_CFM,                               //!< SIBLES_VALUE_WRITE_CFM

    /// Application report to Peer that a value has changed
    SIBLES_VALUE_IND_REQ,                              //!< SIBLES_VALUE_IND_REQ
    /// GATTC Inform application when sending of Service Changed indications has been enabled or disabled
    SIBLES_VALUE_IND_RSP,                               //!< SIBLES_VALUE_IND_RSP

    /// Messages for init and de-init
    /// Register a service
    SIBLES_SVC_REG_REQ,                                //!< SIBLES_SVC_REG_REQ
    SIBLES_SVC_REG128_REQ,                            //!< SIBLES_SVC_REG128_REQ
    /// Register confirmation.
    SIBLES_SVC_RSP,                                     //!< SIBLES_SVC_CFM

    /// GAP commands
    /// Set Adv data
    SIBLES_ADV_DATA_REQ,                                 // SIBLES_ADV_DATA_REQ
    /// Set Scan response data
    SIBLES_SCAN_RSP_REQ,                                    // !<SIBLES_SCAN_RSP_REQ
    /// Toggle ADV state
    SIBLES_ADV_CMD_REQ,
    /// Update device name
    SIBLES_NAME_REQ,
    /// Response to GAP commands
    SIBLES_CMD_RSP,

    SIBLES_SVC_SEARCH_REQ,
    SIBLES_DISC_SVC_IND,
    SIBLES_DISC_CHAR_IND,
    SIBLES_DISC_CHAR_DESC_IND,
    SIBLES_SVC_SEARCH_RSP,

    SIBLES_REGISTER_NOTIFY_REQ,
    SIBLES_REGISTER_NOTIFY_RSP,
    SIBLES_VALUE_WRITE_REQ,
    SIBLES_EVENT_IND,
    SIBLES_VALUE_READ_REQ,
    SIBLES_VALUE_READ_RSP,

    SIBLES_CONNECTED_IND,

    SIBLES_VALUE_NTF_IND,
    SIBLES_SET_STATIC_RANDOM_ADDR_REQ,
    SIBLES_WLAN_COEX_ENABLE_REQ,
    SIBLES_TRC_CFG_REQ,

    SIBLES_WRTIE_VALUE_RSP,

    SIBLES_ENABLE_DBG,

    SIBLES_CH_BD_ADDR,

    SIBLES_CH_BD_ADDR_RSP,

    SIBLES_UNREGISTER_NOTIFY_REQ,

    SIBLES_UPDATE_ATT_PERM_REQ,
    SIBLES_UPDATE_ATT_PERM_RSP,

    //  Sibles ready
    SIBLES_SVC_READY_IND = 0xFF,                        //!< SIBLES_SVC_READY_IND
};

/*
 * API MESSAGES STRUCTURES
 ****************************************************************************************
 * For value name, if not set, will use lower 16bits of attribute UUID as name.
 * Please set other name bytes to 0.
 *
 */


///SIBLES_SET_VALUE_REQ: Set the value of an attribute - Request
///SIBLES_VALUE_REQ_CFM: Peer device  value of requested attribute
///SIBLES_ADV_DATA_REQ:  Set adv data. hdl is not used.
struct sibles_value
{
    /// Value to Set
    uint8_t hdl;
    /// Value length
    uint16_t length;
    /// Value data
    uint8_t data[__ARRAY_EMPTY];
};


///SIBLES_SET_VALUE_RSP:Set the value of an attribute - Response
///SIBLES_VALUE_WRITE_CFM: APP confirm the value changes
struct sibles_value_ack
{
    /// Value Set
    uint8_t hdl;
    /// status of the request
    uint8_t status;
};


/// SIBLES_VALUE_REQ_IND: Peer device request to get profile attribute value
struct sibles_value_req_ind
{
    /// Requested value
    uint8_t hdl;
};

///SIBLES_VALUE_WRITE_IND:Peer set a value change to APP
struct sibles_value_write_ind
{
    /// Value Set
    uint16_t hdl;
    /// Value offset
    uint16_t offset;
    /// Value length
    uint16_t length;
    /// Value data
    uint8_t is_cmd;
    uint8_t data[__ARRAY_EMPTY];
};

/// SIBLES_SVC_REG_REQ: Register one BLE service.
struct sibles_svc_reg_req
{
    uint16_t svc_uuid;
    uint8_t sec_lvl;
    uint8_t attm_entries;
    struct attm_desc *att_db;
};

/// SIBLES_SVC_REG128_REQ: Register one BLE service with 128 bit UUID.
struct sibles_svc_reg128_req
{
    uint8_t svc_uuid[ATT_UUID_128_LEN];
    uint8_t sec_lvl;
    uint8_t attm_entries;
    uint16_t reserved;
    struct attm_desc_128 *att_db;
};


/// SIBLES_SVC_RSP: Confirm on SVC register.
struct sibles_svc_rsp
{
    uint8_t status;
    uint8_t start_hdl;
};

///
struct sibles_svc_search_req
{
    /// Search all if uuid is zero.
    uint8_t conn_idx;
    uint8_t len;
    uint8_t svc_uuid[ATT_UUID_128_LEN];
};

struct sibles_svc_search_rsp
{
    uint8_t status;
    uint8_t len;
    uint8_t svc_uuid[ATT_UUID_128_LEN];
};


/// Discover Service indication Structure
/*@TRACE*/
struct sibles_disc_svc_ind
{
    /// start handle
    uint16_t start_hdl;
    /// end handle
    uint16_t end_hdl;
    /// UUID length
    uint8_t  uuid_len;
    /// service UUID
    uint8_t  uuid[ATT_UUID_128_LEN];
};

/// Discovery All Characteristic indication Structure
/*@TRACE*/
struct sibles_disc_char_ind
{
    /// database element handle
    uint16_t attr_hdl;
    /// pointer attribute handle to UUID
    uint16_t pointer_hdl;
    /// properties
    uint8_t prop;
    /// UUID length
    uint8_t uuid_len;
    /// characteristic UUID
    uint8_t uuid[ATT_UUID_128_LEN];

};


/// Discovery Characteristic Descriptor indication Structure
/*@TRACE*/
struct sibles_disc_char_desc_ind
{
    /// database element handle
    uint16_t attr_hdl;
    /// UUID length
    uint8_t uuid_len;
    /// Descriptor UUID
    uint8_t uuid[ATT_UUID_128_LEN];
};

struct sibles_register_notify_req_t
{
    uint16_t hdl_start;
    uint16_t hdl_end;
    uint16_t seq_num;
};

struct sibles_register_notify_rsp_t
{
    uint8_t status;
    uint16_t seq_num;
};

enum sibles_write_type_t
{
    SIBLES_WRITE,
    SIBLES_WRITE_WITHOUT_RSP
};

typedef enum
{
    SIBLES_READ
} sibles_read_type_t;


struct sibles_value_write_req_t
{
    uint8_t write_type;
    uint16_t seq_num;
    /// Attribute handle
    uint16_t handle;
    /// Write length
    uint16_t length;
    /// Value to write
    uint8_t value[__ARRAY_EMPTY];
};

struct sibles_value_read_req_t
{
    uint8_t read_type;
    uint16_t seq_num;
    uint16_t handle;
    uint16_t offset;
    uint16_t length;
};

struct sibles_wlan_coex_enable_req_t
{
    uint8_t enable;
};

struct sibles_trc_cfg_req_t
{
    uint32_t mask;
};

struct sibles_random_addr
{
    uint8_t addr_type;
    uint8_t addr[BD_ADDR_LEN];
};

struct sibles_ch_bd_addr_t
{
    uint8_t addr_type;
    uint8_t addr_method;
    uint8_t addr[BD_ADDR_LEN];
};

struct sibles_ch_bd_addr_rsp_t
{
    uint8_t status;
};


uint8_t sibles_get_adv_data(uint8_t *data);

struct sibles_update_att_perm_req
{
    uint16_t handle;
    uint16_t access_mask;
    uint16_t perm;
};

struct sibles_update_att_perm_rsp
{
    uint16_t handle;
    uint8_t status;
};

/// @} SIBLESTASK
#endif // SIBLES_TASK_H_
