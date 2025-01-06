/**
 ****************************************************************************************
 *
 * @file gapm.h
 *
 * @brief Generic Access Profile Manager Header.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


#ifndef _GAPM_H_
#define _GAPM_H_

/**
 ****************************************************************************************
 * @addtogroup GAPM Generic Access Profile Manager
 * @ingroup GAP
 * @brief Generic Access Profile Manager.
 *
 * The GAP Manager module is responsible for providing an API to the application in order
 * to manage all non connected stuff such as configuring device to go in desired mode
 * (discoverable, connectable, etc.) and perform required actions (scanning, connection,
 * etc.). GAP Manager is also responsible of managing GAP Controller state according to
 * corresponding BLE connection states.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "ke_task.h"
#include "gap.h"
#include "gapm_task.h"

/*
 * DEFINES
 ****************************************************************************************
 */
/// Bit checking
#define GAPM_ISBITSET(flag, mask)     (((flag)&(mask)) == mask)
/// Maximum number of advertising reports from different advertisers that can be reassembled in parallel
#define GAPM_REPORT_NB_MAX            (5)

/*
 * STRUCTURE DEFINITION
 ****************************************************************************************
 */

/// Contain a received fragment of advertising or scan response report
struct gapm_report_elem
{
    /// List Header
    struct co_list_hdr list_hdr;
    /// Data length
    uint8_t data_len;
    /// Data
    uint8_t data[];
};

/// Contain a list of fragments received for an advertising or scan response report sent
/// by a given advertiser
struct gapm_report_list
{
    // List of received reports (@see struct gapm_report_elem)
    struct co_list report_list;
    // Advertiser address
    struct gap_bdaddr adv_addr;
    // Received length
    uint16_t length;
};

/// GAP Manager activity structure (common part for advertising, scanning,
/// initiating and periodic synchronization activities)
struct gapm_actv_tag
{
    /// Function to be called for activity start
    uint8_t (*cb_actv_start)(struct gapm_actv_tag *, struct gapm_activity_start_cmd *);
    /// Function to be called for activity stop
    void (*cb_actv_stop)(struct gapm_actv_tag *);
    /// Function to be called for activity delete
    void (*cb_actv_delete)(struct gapm_actv_tag *);

    /// Identifier
    uint8_t idx;
    /// Type (@see enum gapm_actv_type)
    uint8_t type;
    /// Subtype - meaning depends on activity type
    ///  - Advertising activity: @see enum gap_adv_subtype
    ///  - Scanning activity: @see enum gap_scan_subtype
    ///  - Initiating activity: @see enum gap_init_subtype
    ///  - Periodic Synchronization activity: @see enum gap_period_sync_subtype
    uint8_t subtype;
    /// State (@see enum gapm_actv_state)
    uint8_t state;
    /// Information bit field, meaning depends on activity type
    uint8_t info;
    /// Own address type
    uint8_t own_addr_type;
    /// Next expected HCI event opcode
    uint16_t next_exp_opcode;
    /// Task ID of task that has requested creation of the activity
    ke_task_id_t requester;
    /// BD Address used by the activity (can be different if controller privacy is used and
    /// application chose to use a resolvable private address)
    bd_addr_t addr;
};

/// GAP Manager activity structure for advertising activity
struct gapm_actv_adv_tag
{
    /// Common activity parameters
    struct gapm_actv_tag common;
    /// Data offset for the set advertising data procedure
    uint16_t data_offset;
    /// Advertising mode (@see enum gapm_adv_disc_mode)
    uint8_t mode;
    /// Stored status
    uint8_t kept_status;
    /// Selected TX power
    uint8_t tx_pwr;
};

/// GAP Manager activity structure for scanning activity
struct gapm_actv_scan_tag
{
    /// Common activity parameters
    struct gapm_actv_tag common;
    /// Lists containing fragments for GAPM_REPORT_NB_MAX reports that can be received in parallel
    struct gapm_report_list report_lists[GAPM_REPORT_NB_MAX];
    /// Scan filtering Array
    struct gap_bdaddr *p_scan_filter;
};

/// GAP Manager activity structure for initiating activity
struct gapm_actv_init_tag
{
    /// Common activity parameters
    struct gapm_actv_tag common;
    /// Initiating parameters
    struct gapm_init_param init_param;
    /// Number of connection to be established for automatic connection
    ///    -> Number of devices in the white list when GAPM_ACTIVITY_START_CMD is received
    uint8_t nb_auto_conn;
    /// Stored status
    uint8_t kept_status;
};

/// GAP Manager activity structure for periodic synchronization activity
struct gapm_actv_per_sync_tag
{
    /// Common activity parameters
    struct gapm_actv_tag common;
    // List of received reports fragment (@see struct gapm_report_elem)
    struct co_list report_list;
    // Received length
    uint16_t length;
    /// Synchronization Handle
    uint16_t sync_hdl;
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize Generic Access Profile Manager Module.
 *
 * @param[in] reset  true if it's requested by a reset; false if it's boot initialization
 *
 ****************************************************************************************
 */
void gapm_init(bool reset);


/**
 ****************************************************************************************
 * @brief Retrieve Task Identifier from Task number
 * (automatically update index of task in returned task id)
 *
 * @param task Task number
 * @return Task Identifier
 ****************************************************************************************
 */
ke_task_id_t gapm_get_id_from_task(ke_msg_id_t task);

/**
 ****************************************************************************************
 * @brief Retrieve Task Number from Task Identifier
 * (automatically update index of task in returned task id)
 *
 * @param id Task Identifier
 * @return Task Number
 ****************************************************************************************
 */
ke_task_id_t gapm_get_task_from_id(ke_msg_id_t id);

/**
 ****************************************************************************************
 * @brief Acquire AHI task ID for in-core and out-core scenaraio.
 *
 * @return AHI Task ID
 ****************************************************************************************
 */
ke_task_id_t gapm_get_AHI_task_id(void);


#if (BLE_GAPC)
    /**
    ****************************************************************************************
    * @brief Created link connection parameters (from bond data) has been set, connection
    * ready to be used.
    *
    * @param[in] conidx     Connection Index
    *
    ****************************************************************************************
    */
    void gapm_con_enable(uint8_t conidx);


    /**
    ****************************************************************************************
    * @brief A link has been disconnected, clean-up host stack for this connection.
    *
    * @param[in] conidx     Connection Index
    * @param[in] conhdl     Connection Handle
    * @param[in] reason     Reason of the disconnection
    *
    ****************************************************************************************
    */
    void gapm_con_cleanup(uint8_t conidx, uint16_t conhdl, uint8_t reason);

#endif // (BLE_GAPC)


/**
 ****************************************************************************************
 * @brief Retrieve device identity key.
 *
 * @return Device Identity Key
 ****************************************************************************************
 */
struct gap_sec_key *gapm_get_irk(void);


/**
 ****************************************************************************************
 * @brief Retrieve local public address.
 *
 * @return Return local public address
 ****************************************************************************************
 */
bd_addr_t *gapm_get_bdaddr(void);



#if (BLE_ISO_MODE_0_PROTOCOL)
    /**
    ****************************************************************************************
    * @brief Return if LE Audio Mode 0 is supported or not
    *
    * @return True if supported, False else
    ****************************************************************************************
    */
    bool gapm_is_audio_am0_sup(void);
#endif // (BLE_ISO_MODE_0_PROTOCOL)

/**
 ****************************************************************************************
 * @brief Retrieve the device address type (@see enum gapm_addr_type)
 *
 * @return address type
 ****************************************************************************************
 */
uint8_t gapm_get_address_type(void);


#if (BLE_ATTS)

    /**
    ****************************************************************************************
    * @brief Get if preferred connection parameters present in GAP ATT database
    *
    * @return True if referred connection parameters present in GAP ATT database, False else
    ****************************************************************************************
    */
    bool gapm_is_pref_con_param_pres(void);

    /**
    ****************************************************************************************
    * @brief retrieve gap attribute handle from attribute index.
    *
    * @param[in] att_idx Attribute index
    *
    * @return Attribute handle
    ****************************************************************************************
    */
    uint16_t gapm_get_att_handle(uint8_t att_idx);

#endif // (BLE_ATTS)

#if (SECURE_CONNECTIONS)
    /**
    ****************************************************************************************
    * @brief Returns the local Public Key
    *
    * @return pointer to the local Public Key
    ****************************************************************************************
    */
    public_key_t *gapm_get_local_public_key(void);
#endif // (SECURE_CONNECTIONS)

/**
 ****************************************************************************************
 * @brief Retrieve if Legacy pairing is supported on local device
 *
 * @return True if legacy pairing is supported
 ****************************************************************************************
 */
bool gapm_is_legacy_pairing_supp(void);


/**
 ****************************************************************************************
  * @brief Retrieve if Secure Connection pairing is supported on local device
 *
 * @return True if Secure Connection pairing is supported
 ****************************************************************************************
 */
bool gapm_is_sec_con_pairing_supp(void);


#if (BLE_LECB)
    /**
    ****************************************************************************************
    * @brief Check if LECB connection can be registered (established)
    *
    * @param[in]  le_psm        LE Protocol/Service Multiplexer
    * @param[in]  conidx        Connection index for app_task computation
    * @param[out] app_task      Destination application/profile task
    * @param[out] sec_lvl       Security level requirements
    *
    *
    * @return GAP_ERR_NOT_FOUND if LE_PSM not register, GAP_ERR_NO_ERROR else
    ****************************************************************************************
    */
    uint8_t gapm_le_psm_get_info(uint16_t le_psm, uint8_t conidx, ke_task_id_t *app_task, uint8_t *sec_lvl);


    /**
    ****************************************************************************************
    * @brief Check if LECB connection can be registered (established)
    *
    * @param[in] le_psm        LE Protocol/Service Multiplexer
    * @param[in] peer_con_init Info to know if connection is initiated by peer device
    *
    * @return L2C_ERR_NO_RES_AVAIL if all LECB link are established, GAP_ERR_NO_ERROR else
    ****************************************************************************************
    */
    uint8_t gapm_lecb_register(uint16_t le_psm, bool peer_con_init);


    /**
    ****************************************************************************************
    * @brief Unregister an existing LECB connection
    *
    * @param[in] le_psm        LE Protocol/Service Multiplexer
    * @param[in] peer_con_init Info to know if connection is initiated by peer device
    *
    * @return GAP_ERR_NO_ERROR
    ****************************************************************************************
    */
    uint8_t gapm_lecb_unregister(uint16_t le_psm, bool peer_con_init);

#endif // (BLE_LECB)

/**
 ****************************************************************************************
 * @brief Return the maximal MTU value
 *
 * @param[out] Maximal MTU value
 ****************************************************************************************
 */
uint16_t gapm_get_max_mtu(void);

/**
 ****************************************************************************************
 * @brief Return the maximal MPS value
 *
 * @param[out] Maximal MPS value
 ****************************************************************************************
 */
uint16_t gapm_get_max_mps(void);



/**
 ****************************************************************************************
 * @brief Check If Service changed feature is enabled or not
 *
 * @return true if enabled, false else.
 *
 ****************************************************************************************
 */
bool gapm_svc_chg_en(void);

#if (RW_DEBUG)
    /**
    ****************************************************************************************
    * @brief Check If Debug mode feature is enabled or not
    *
    * @return true if enabled, false else.
    *
    ****************************************************************************************
    */
    bool gapm_dbg_mode_en(void);

    /**
    ****************************************************************************************
    * @brief Check If received L2CAP packet must be forwarded to the application
    *
    * @return true if enabled, false else.
    *
    ****************************************************************************************
    */
    bool gapm_dbg_fwd_traffic(void);

    /**
    ****************************************************************************************
    * @brief Force the GAP service start handle
    ****************************************************************************************
    */
    void gapm_set_svc_start_hdl(uint16_t start_hdl);
#endif // (RW_DEBUG)

/**
 ****************************************************************************************
 * Retrieve if current connection index is used for a discovery purpose such as
 * Name discovery
 *
 * @param conidx Index of the specific connection
 *
 * @return true if connection has a discovery purpose, False else
 ****************************************************************************************
 */
bool gapm_is_disc_connection(uint8_t conidx);

/// @} GAPM

#endif /* _GAPM_H_ */
