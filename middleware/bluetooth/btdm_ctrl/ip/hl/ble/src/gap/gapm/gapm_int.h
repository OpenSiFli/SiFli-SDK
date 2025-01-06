/**
 ****************************************************************************************
 *
 * @file gapm_int.h
 *
 * @brief Generic Access Profile Manager Internal Header.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


#ifndef _GAPM_INT_H_
#define _GAPM_INT_H_

/**
 ****************************************************************************************
 * @addtogroup GAPM_INT Generic Access Profile Manager Internal
 * @ingroup GAPM
 * @brief defines for internal GAPM usage
 *
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gapm.h"
#include "gapm_task.h"
#include "co_bt.h"


/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of GAP Manager process
#define GAPM_IDX_MAX                                 0x01

/// Scan filter size
#define GAPM_SCAN_FILTER_SIZE   10

/// check if current role is supported by configuration
#define GAPM_IS_ROLE_SUPPORTED(role_type)\
    ((gapm_env.role & (role_type)) == (role_type))

/// Number of supported activities in host
#define GAPM_ACTV_NB            (BLE_ACTIVITY_MAX)
/// Invalid activity identifier
#define GAPM_ACTV_INVALID_IDX   (0xFF)

/*
 * INTERNAL API TYPES
 ****************************************************************************************
 */


/// Retrieve information about memory usage
struct gapm_dbg_get_mem_info_cmd
{
    /// GAPM requested operation:
    ///  - GAPM_DBG_GET_MEM_INFO: Get memory usage
    uint8_t operation;
};

/// Indication containing information about memory usage.
/*@TRACE*/
struct gapm_dbg_mem_info_ind
{
    /// peak of memory usage measured
    uint32_t max_mem_used;
    /// memory size currently used into each heaps.
    uint16_t mem_used[KE_MEM_BLOCK_MAX];
};


/// Operation type
enum gapm_op_type
{
    /// Configuration operation
    GAPM_OP_CFG         = 0x00,

    /// Air mode operation (scanning, advertising, connection establishment)
    /// Note: Restriction, only one air operation supported.
    GAPM_OP_AIR         = 0x01,
#if (SECURE_CONNECTIONS)
    /// GAP State for DH Key Generation
    GAPM_OP_DHKEY       = 0x02,
#endif // (SECURE_CONNECTIONS)
    /// Max number of operations
    GAPM_OP_MAX
};

/// GAPM states
enum gapm_state_id
{
    /// Idle state - no on going operation
    GAPM_IDLE,

    /// Busy state - Configuration operation on going
    GAPM_CFG_BUSY       = 1 << GAPM_OP_CFG,
    /// Busy state - Air operation on going
    GAPM_AIR_BUSY       = 1 << GAPM_OP_AIR,
#if (SECURE_CONNECTIONS)
    /// Busy state - DH Key Calculation operation on going
    GAPM_DHKEY_BUSY     = 1 << GAPM_OP_DHKEY,
#endif //  (SECURE_CONNECTIONS)

    /// Reset state - Reset operation on going
    GAPM_DEVICE_SETUP   = 1 << GAPM_OP_MAX,

    GAPM_STATE_MAX
};

/// Device configuration flags
///    7    6    5    4    3    2    1    0
/// +-----+----+----+----+----+----+----+----+
/// | RFU | SC | CP | GA | RE |CTLP|HSTP|ADDR|
/// +-----+----+----+----+----+----+----+----+
/// - Bit [0-2]: Address Type @see enum gapm_cfg_flag
///             Bit 0: 0 = Public address, 1 = Private address
///             Bit 2: 0 = Controller-based Privacy disabled, 1 = enabled
///
/// - Bit [3]  : Address to renew (only if privacy is enabled, 1 address to renew else 0)
/// - Bit [4]  : Generated Address type (1 = Resolvable, 0 = Non Resolvable)
/// - Bit [5]  : Preferred Connection parameters present in GAP DB
/// - Bit [6]  : Service Change feature present
/// Configuration flag bit description
enum gapm_cfg_flag_def
{
    /// Address Type
    GAPM_MASK_ADDR_TYPE           = 0x07,
    GAPM_POS_ADDR_TYPE            = 0x00,
    /// Address to renew
    GAPM_MASK_ADDR_RENEW          = 0x08,
    GAPM_POS_ADDR_RENEW           = 0x03,
    /// Generated Address type
    GAPM_MASK_RESOLV_ADDR         = 0x10,
    GAPM_POS_RESOLV_ADDR          = 0x04,
    /// Preferred Connection parameters present in GAP DB
    GAPM_MASK_PREF_CON_PAR_PRES   = 0x20,
    GAPM_POS_PREF_CON_PAR_PRES    = 0x05,
    /// Service Change feature present
    GAPM_MASK_SVC_CHG_EN          = 0x40,
    GAPM_POS_SVC_CHG_EN           = 0x06,
};

#if (RW_DEBUG)
/// Device configuration flags
///    7    6    5    4    3    2    1    0
/// +-----+----+----+----+----+----+----+----+
/// |              RFU             |DBGT| DBG|
/// +-----+----+----+----+----+----+----+----+
/// - Bit [0]: Enable Debug mode
/// - Bit [1]: Forward all L2cap traffic
enum gapm_dbg_cfg
{
    ///  Enable Debug mode
    GAPM_MASK_DBG_MODE_EN              = 0x01,
    GAPM_POS_DBG_MODE_EN               = 0x00,

    ///  Enable Debug mode
    GAPM_MASK_DBG_L2CAP_TRAFFIC_FWD_EN = 0x02,
    GAPM_POS_DBG_L2CAP_TRAFFIC_FWD_EN  = 0x01,
};
#endif // (RW_DEBUG)

/// Activity states
enum gapm_actv_state
{
    /// Activity is being created - next state is CREATED
    GAPM_ACTV_CREATING = 0,
    /// Activity has been successfully created
    GAPM_ACTV_CREATED,
    /// Activity is being started - next state is STARTED
    GAPM_ACTV_STARTING,
    /// Activity has been successfully started
    GAPM_ACTV_STARTED,
    /// Activity is being stopped - next state is CREATED
    GAPM_ACTV_STOPPING,
    /// Activity is being deleted - no next state
    GAPM_ACTV_DELETING,
};

/// Connection role
enum gapm_conn_role
{
    /// Master
    GAPM_ROLE_MASTER = 0,
    /// Slave
    GAPM_ROLE_SLAVE,
};

/*
 * MACROS
 ****************************************************************************************
 */
/// Macro used to retrieve field
#define GAPM_F_GET(data, field)\
        (((data) & (GAPM_MASK_ ## field)) >> (GAPM_POS_ ## field))

/// Macro used to set field
#define GAPM_F_SET(data, field, val)\
    (data) = (((data) & ~(GAPM_MASK_ ## field)) \
           | ((val << (GAPM_POS_ ## field)) & (GAPM_MASK_ ## field)))

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Information about registered LE_PSM
struct gapm_le_psm_info
{
    /// List header structure to put information within a list
    struct co_list_hdr hdr;

    /// LE Protocol/Service Multiplexer
    uint16_t le_psm;

    /// Registered task identifier
    ke_task_id_t task_id;

    /// Security level
    ///   7   6   5   4   3   2   1   0
    /// +---+---+---+---+---+---+---+---+
    /// |MI |      RFU      |EKS|SEC_LVL|
    /// +---+---+---+---+---+---+---+---+
    /// bit[0-1]: Security level requirement (0=NO_AUTH, 1=UNAUTH, 2=AUTH, 3=SEC_CON)
    /// bit[2]  : Encryption Key Size length must have 16 bytes
    /// bit[7]  : Multi-instantiated task
    uint8_t sec_lvl;

    /// Number of established link
    uint8_t nb_est_lk;
};

/// GAP Manager environment structure
struct gapm_env_tag
{
    /// Request operation Kernel message
    void *operation[GAPM_OP_MAX];

    /// Array of pointers to the allocated activities
    struct gapm_actv_tag *actvs[GAPM_ACTV_NB];

#if (BLE_LECB)
    /// Registered list of LE Protocol/Service Multiplexer for LE Credit Based Connection
    struct co_list reg_le_psm;
#endif // (BLE_LECB)

#if (BLE_ATTS)
    /// GAP service start handle
    uint16_t svc_start_hdl;
#endif // (BLE_ATTS)

#if (BLE_BROADCASTER)
    /// Maximum length of advertising data that can be pushed to the controller
    uint16_t max_adv_data_len;
#endif //(BLE_BROADCASTER)

    /// Duration before regenerate device address when privacy is enabled. - in seconds
    uint16_t renew_dur;
    /// Device IRK used for resolvable random BD address generation (MSB -> LSB)
    struct gap_sec_key irk;

    /// Current device Address
    bd_addr_t addr;
    /// Device Role
    uint8_t role;
    /// Number of BLE connection
    uint8_t connections;
    /// Device configuration flags - (@see enum gapm_cfg_flag_def)
    uint8_t cfg_flags;
    /// Pairing mode authorized (see enum gapm_pairing_mode)
    uint8_t pairing_mode;
    /// Maximum device MTU size
    uint16_t max_mtu;
    /// Maximum device MPS size
    uint16_t max_mps;
#if (SECURE_CONNECTIONS)
    /// Local device Public Key
    public_key_t public_key;
#endif // (SECURE_CONNECTIONS)

#if(BLE_ISO_MODE_0_PROTOCOL)
    /// Audio configuration flag (see gapm_audio_cfg_flag)
    uint16_t audio_cfg;
#endif // (BLE_ISO_MODE_0_PROTOCOL)

#if (BLE_LECB)
    /// Maximum number of allowed LE Credit Based channels
    uint8_t max_nb_lecb;
    /// Current number of LE Credit Based channel connections established
    uint8_t nb_lecb;
#endif // (BLE_LECB)

#if (BLE_BROADCASTER)
    /// Number of advertising sets supported by controller
    uint8_t max_adv_set;
    /// Number of created advertising activities
    uint8_t nb_adv_actv;
#endif //(BLE_BROADCASTER)
#if (BLE_OBSERVER)
    /// Activity identifier for currently started scanning activity
    uint8_t scan_actv_idx;
#endif //(BLE_OBSERVER)
#if (BLE_CENTRAL)
    /// Activity identifier for currently started initiating activity
    uint8_t init_actv_idx;
#endif //(BLE_CENTRAL)
    /// Activity identifier currently used for air operation
    uint8_t actv_idx;
    /// Number of created activities
    uint8_t created_actvs;
    /// Number of started activities
    uint8_t started_actvs;
    /// Number of devices in the white list
    uint8_t nb_dev_wl;
    /// State of GAPM ADDR module
    uint8_t gapm_addr_state;
    // arry of paused activities
    uint8_t pause_act_idx[GAPM_ACTV_NB];
    // number of pause activities
    uint8_t nb_pause_act;
    // pause activities flag
    uint8_t pause_act_flg;

#if (RW_DEBUG)
    /// Debug configuration @see gapm_dbg_cfg
    uint8_t dbg_cfg;
#endif // (RW_DEBUG)
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */
/// GAP Manager environment variable.
extern struct gapm_env_tag gapm_env;


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize GAP attribute database
 *
 * @param[in] start_hdl Service Start Handle
 * @param[in] feat      Attribute database features
 *
 * @return status code of attribute database initialization
 *  - @ref ATT_ERR_NO_ERROR: If database creation succeeds.
 *  - @ref ATT_ERR_INVALID_HANDLE: If start_hdl given in parameter + nb of attribute override
 *                            some existing services handles.
 *  - @ref ATT_ERR_INSUFF_RESOURCE: There is not enough memory to allocate service buffer.
 *                           or of new attribute cannot be added because all expected
 *                           attributes already add
 ****************************************************************************************
 */
uint8_t gapm_init_attr(uint16_t start_hdl, uint32_t feat);

/**
 ****************************************************************************************
 * @brief Send operation completed message according to operation type.
 * Perform also am operation clean-up
 *
 * @param[in] op_type Operation type
 * @param[in] status  Command status
 *****************************************************************************************
 */
void gapm_send_complete_evt(uint8_t op_type, uint8_t status);

/**
 ****************************************************************************************
 * @brief Send operation completed message with status error code not related to a
 * running operation.
 *
 * @param[in] operation Operation code
 * @param[in] requester requester of operation
 * @param[in] status    Error status code
 ****************************************************************************************
 */
void gapm_send_error_evt(uint8_t operation, const ke_task_id_t requester, uint8_t status);


/**
 ****************************************************************************************
 * @brief Get operation pointer
 *
 * @param[in] op_type       Operation type.
 *
 * @return operation pointer on going
 ****************************************************************************************
 */
__INLINE void *gapm_get_operation_ptr(uint8_t op_type)
{
    ASSERT_ERR(op_type < GAPM_OP_MAX);
    // return operation pointer
    return gapm_env.operation[op_type];
}


/**
 ****************************************************************************************
 * @brief Set operation pointer
 *
 * @param[in] op_type       Operation type.
 * @param[in] op            Operation pointer.
 *
 ****************************************************************************************
 */
__INLINE void gapm_set_operation_ptr(uint8_t op_type, void *op)
{
    ASSERT_ERR(op_type < GAPM_OP_MAX);
    // update operation pointer
    gapm_env.operation[op_type] = op;
}


/**
 ****************************************************************************************
 * @brief Check if current operation can be processed or not.
 * if it can be proceed, initialize an operation request.
 * If a command complete event with error code can be triggered.
 *
 * Function returns how the message should be handled by message handler.
 *
 * @param[in] op_type       Operation type.
 * @param[in] op_msg        Requested operation message (note op_msg cannot be null)
 * @param[in] supp_ops      Supported operations array.
 *                          Latest array value shall be GAPM_NO_OP.
 *
 * @return operation can be executed if message status equals KE_MSG_NO_FREE,
 * else nothing to do, just exit from the handler.
 ****************************************************************************************
 */
int gapm_process_op(uint8_t op_type, void *op_msg, enum gapm_operation *supp_ops);

/**
 ****************************************************************************************
 * @brief Get operation on going
 *
 * @param[in] op_type       Operation type.
 *
 * @return operation code on going
 ****************************************************************************************
 */
uint8_t gapm_get_operation(uint8_t op_type);

/**
 ****************************************************************************************
 * @brief Get requester of on going operation
 *
 * @param[in] op_type       Operation type.
 *
 * @return task that requests to execute the operation
 ****************************************************************************************
 */
ke_task_id_t gapm_get_requester(uint8_t op_type);




#if (BLE_GAPC)
    /**
    ****************************************************************************************
    * @brief A connection has been created, initialize host stack to be ready for connection.
    *
    * @param[in] msgid      Message id
    * @param[in] con_params Connection parameters from lower layers
    *
    * @return Connection index allocated to the new connection.
    ****************************************************************************************
    */
    uint8_t gapm_con_create(ke_msg_id_t const msgid, struct hci_le_enh_con_cmp_evt const *con_params);
#endif // (BLE_GAPC)


/**
 ****************************************************************************************
 * @brief Set the maximal MTU value
 *
 * @param[in] mtu   Max MTU value (Minimum is 23)
 ****************************************************************************************
 */
void gapm_set_max_mtu(uint16_t mtu);

/**
 ****************************************************************************************
 * @brief Set the maximal MPS value
 *
 * @param[in] mps   Max MPS value (Minimum is 23)
 ****************************************************************************************
 */
void gapm_set_max_mps(uint16_t mps);

/**
 ****************************************************************************************
 * @brief Checks validity of the address type
 *
 * @param[in] addr_type   Address type
 ****************************************************************************************
 */
uint8_t gapm_addr_check(uint8_t addr_type);

/**
 ****************************************************************************************
 * @brief Checks validity of the Data Length Suggested values
 *
 * @param[in] sugg_oct   Suggested octets
 * @param[in] sugg_time  Suggested time
 ****************************************************************************************
 */
uint8_t gapm_dle_val_check(uint16_t sugg_oct, uint16_t sugg_time);



/**
 ****************************************************************************************
 * @brief Update task state
 *
 * @param[in] operation that modify the state
 * @param[in] set state to busy (true) or idle (false)
 *
 ****************************************************************************************
 */
void gapm_update_state(uint8_t operation, bool busy);



#if (BLE_LECB)


    /**
    ****************************************************************************************
    * @brief Find Information about LE_PSM registered in GAPM
    *
    * @param[in] le_psm    LE Protocol/Service Multiplexer
    *
    * @return Null if not found or LE_PSM info structure
    ****************************************************************************************
    */
    struct gapm_le_psm_info *gapm_le_psm_find(uint16_t le_psm);


    /**
    ****************************************************************************************
    * @brief Remove all registered LE_PSM
    ****************************************************************************************
    */
    void gapm_le_psm_cleanup(void);
#endif // (BLE_LECB)

/**
 ****************************************************************************************
 * @brief Initialize GAPM environment for support of activities or reset the environment
 * by cleaning all existing activities
 *
 * @param[in] reset
 ****************************************************************************************
 */
void gapm_actv_init(bool reset);

/**
 ****************************************************************************************
 * @brief Allocate an activity structure for a given identifier. Provided identifer is
 * then no more marked as available.
 * It is considered here that provided activity identifier is an available one.
 *
 * @param[in] actv_idx      Activity index
 * @param[in] size          Size of the activity structure to be allocated
 *
 * @return Pointer to the allocated activity structure
 ****************************************************************************************
 */
struct gapm_actv_tag *gapm_actv_alloc(uint8_t actv_idx, uint8_t size);

/**
 ****************************************************************************************
 * @brief Retrieve activity for which an HCI command complete event has been received.
 * Note that at any time only one HCI command complete event can be expected by GAPM.
 * Hence only one activity can be found.
 * This function also check that the operation code of the complete command well
 * match with the command that has been sent for the foun activity.
 *
 * @param[in|out] pp_actv    Pointer to the variable that will contain address of
 *                           activity structure for which command complete event
 *                           has been received.
 * @param[in] opcode         Command operation code for which a command complete event has
 *                           been received
 *
 * @return true if activity has been retrieved, else false
 ****************************************************************************************
 */
bool gapm_actv_retrieve_cmd_cmp_evt(struct gapm_actv_tag **pp_actv, uint16_t opcode);

/**
 ****************************************************************************************
 * @brief Get own address type to be sent in HCI commands.
 *
 * @param[in] app_addr_type     Address type provided by application using GAPM_SET_DEV_CONFIG_CMD.
 *
 * @return Own address type value
 *      - 0: Public Device Address
 *      - 1: Random Device Address
 *      - 2: Controller generates the RPA based on the local IRK from the resolving list.
 * If the resolving list contains no matching entry, use the public address
 *      - 3: Controller generates the RPA based on the local IRK from the resolving list.
 * If the resolving list contains no matching entry, use the random address that has
 * been set using HCI LE Set Advertising Set Random Address.
 ****************************************************************************************
 */
uint8_t gapm_actv_get_hci_own_addr_type(uint8_t app_addr_type);

/**
 ****************************************************************************************
 * @brief Notify the activity module that activity creation procedure is over.
 *
 * @param[in] p_actv    Pointer to the structure describing the created activity
 * @param[in] error     Indicate if an error has been raised during activity creation
 ****************************************************************************************
 */
void gapm_actv_created(struct gapm_actv_tag *p_actv, uint8_t error);

/**
 ****************************************************************************************
 * @brief Notify the activity module that a start activity procedure is over
 *
 * @param[in] p_actv    Pointer to the structure describing the started activity
 * @param[in] error     Indicate if an error has been raised during activity start procedure
 ****************************************************************************************
 */
void gapm_actv_started(struct gapm_actv_tag *p_actv, uint8_t error);

/**
 ****************************************************************************************
 * @brief Notify the activity module that an activity has well been stopped.
 *
 * @param[in] p_actv    Pointer to the structure describing the stopped activity
 * @param[in] status    Activity stop operation status
 ****************************************************************************************
 */
void gapm_actv_stopped(struct gapm_actv_tag *p_actv, uint8_t status);

/**
 ****************************************************************************************
 * @brief Function used once an activity has been cleaned and can be deleted
 *
 * @param[in] p_actv    Pointer to the structure describing the stopped activity
 * @param[in] error     Indicate if activity has well been stopped
 ****************************************************************************************
 */
void gapm_actv_deleted(struct gapm_actv_tag *p_actv);

/**
 ****************************************************************************************
 * @brief Callback function used by GAPM ADDR module in order to inform the GAPM ACTV
 * module that a random address (generated or not) has been set in the controller
 * upon its request.
 *
 * @param[in] p_actv    Pointer to the activity structure for which random address has
 *                      been set.
 ****************************************************************************************
 */
void gapm_actv_rand_addr_set_ind(struct gapm_actv_tag *p_actv);

#if (BLE_BROADCASTER)
    /**
    ****************************************************************************************
    * @brief Allocate an activity structure to be used for an advertising activity.
    *
    * @param[in] actv_idx      Allocated activity identifier
    * @param[in] p_param       GAPM_ACTIVITY_CREATE_CMD message parameters
    ****************************************************************************************
    */
    uint8_t gapm_adv_create(uint8_t actv_idx, struct gapm_activity_create_adv_cmd *p_param);
#endif //(BLE_BROADCASTER)

#if (BLE_OBSERVER)
    /**
    ****************************************************************************************
    * @brief Clean content of a scan activity, more specifically the stored fragments
    * of advertising report and the filtering list.
    *
    * @param[in] p_actv    Pointer to the scanning activity structure.
    ****************************************************************************************
    */
    void gapm_scan_actv_clean(struct gapm_actv_tag *p_actv);

    /**
    ****************************************************************************************
    * @brief Allocate an activity structure to be used for a scanning activity.
    *
    * @param[in] actv_idx      Allocated activity identifier
    * @param[in] p_param       GAPM_ACTIVITY_CREATE_CMD message parameters
    *
    * @return GAP_ERR_NO_ERROR if creation of activity can be started,
    *         GAP_ERR_COMMAND_DISALLOWED if activity cannot be started
    *         GAP_ERR_INSUFF_RESOURCES if not enough memory is found for activity structure
    *         allocation.
    ****************************************************************************************
    */
    uint8_t gapm_scan_create(uint8_t actv_idx, struct gapm_activity_create_cmd *p_param);
#endif //(BLE_OBSERVER)

#if (BLE_CENTRAL)
    /**
    ****************************************************************************************
    * @brief Allocate an activity structure to be used for an initiating activity.
    *
    * @param[in] actv_idx      Allocated activity identifier
    * @param[in] p_param       GAPM_ACTIVITY_CREATE_CMD message parameters
    ****************************************************************************************
    */
    uint8_t gapm_init_create(uint8_t actv_idx, struct gapm_activity_create_cmd *p_param);

    /**
    ****************************************************************************************
    * @brief Function used by GAPM_ACTV module to inform GAPM_INIT that a new connection
    * has been established.
    * If current initiating type is a name discovery, it triggers sending of a GAPC_GET_INFO_CMD
    * in order to retrieve the peer device name.
    * If current initiating type is an automatic connection establishment, a new HCI LE
    * Extended Create Connection command will be sent to the controller if not all device in
    * the white list have been connected.
    * If no more connection is expected, the activity is stopped.
    *
    * @param[in] conidx    Connection index allocated for the newly established connection
    *
    * @return True if the activity has to be stopped, else false.
    ****************************************************************************************
    */
    bool gapm_init_connection_ind(uint8_t conidx);
#endif //(BLE_CENTRAL)

#if (BLE_OBSERVER)
    /**
    ****************************************************************************************
    * @brief Allocate an activity structure to be used for a periodic synchronization activity.
    *
    * @param[in] actv_idx      Allocated activity identifier
    * @param[in] p_param       GAPM_ACTIVITY_CREATE_CMD message parameters
    *
    * @return GAP_ERR_NO_ERROR if activity creation has ended properly, GAP_ERR_INSUFF_RESOURCES
    * if no available memory has been found for activity allocation.
    ****************************************************************************************
    */
    uint8_t gapm_per_sync_create(uint8_t actv_idx, struct gapm_activity_create_cmd *p_param);

    /**
    ****************************************************************************************
    * @brief Request to clear all report fragments stored in the periodic synchronization
    * activity
    *
    * @param[in] p_actv_per_sync      Pointer to the activity structure
    ****************************************************************************************
    */
    void gapm_per_sync_clear_fragments(struct gapm_actv_per_sync_tag *p_actv_per_sync);
#endif //(BLE_OBSERVER)

/**
 ****************************************************************************************
 * @brief Initialize GAPM address manager module
 ****************************************************************************************
 */
void gapm_addr_init(void);

/**
 ****************************************************************************************
 * @brief Function to be used to check if address type provided for a scanning or an
 * initiating activity is valid or not.
 * As a same address shall be shared between all these activities, all created scanning and
 * initiating activities shall have the same address type.
 *
 * @param[in] addr_type     Address type to be checked
 *
 * @return true if provided address type is valid, else false
 ****************************************************************************************
 */
bool gapm_is_addr_type_valid(uint8_t addr_type);

/**
 ****************************************************************************************
 * @brief Request to clear all report fragments stored in the periodic synchronization
 * activity
 *
 * @param[in] p_actv      Pointer to the activity structure for which a random has to be
 *                        generated and/or set in the controller.
 ****************************************************************************************
 */
void gapm_addr_set_rand_addr(struct gapm_actv_tag *p_actv);

/**
 ****************************************************************************************
 * @brief Sends a basic HCI command (with no parameter)
 *
 * @param[in] opcode       Command opcode
 ****************************************************************************************
 */
void hci_basic_cmd_send_2_controller(uint16_t opcode);


/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

/// @} GAPM_INT

#endif /* _GAPM_INT_H_ */
