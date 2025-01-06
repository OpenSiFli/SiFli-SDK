/**
 ****************************************************************************************
 *
 * @file lc.h
 *
 * @brief Main API file for the Link Controller
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

#ifndef LC_H_
#define LC_H_

/**
 ****************************************************************************************
 * @defgroup LC Link Controller
 * @ingroup ROOT
 * @brief BT Lower Layers
 *
 * The CONTROLLER contains the modules allowing the physical link establishment,
 * maintenance and management.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_task.h"    // Task definitions
#include "ke_task.h"      // kernel task definitions
#include "co_bt.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// No connection handle
#define NO_CON_HDL                     65535
/// No TID
#define NO_TID_FREE                    NO_CON_HDL

/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */

/// Message API of the LC task
enum LC_CON_REASON
{
    LC_MASTER_CON,
    LC_SLAVE_CON,
    LC_REM_NAME,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
///Key status
enum
{
    KEY_ABSENT = 0,
    KEY_PRESENT,
    KEY_PENDING,
};

///Pin Status
enum
{
    PIN_ABSENT = 0,
    PIN_PRESENT,
};

/*
 * MESSAGES
 ****************************************************************************************
 */

/// Message API of the LC task
/*@TRACE*/
enum lc_msg_id
{
    LC_MSG_ID_MSG_FIRST = TASK_FIRST_MSG(TASK_ID_LC),

    /*
     * ************** Msg LC->LC****************
     */

    LC_LMP_RX,
    LC_SYNC_IND,

    LC_LMP_RSP_TO,
    LC_CON_ACCEPT_TO,
    LC_AFH_REPORT_TO,
    LC_AUTH_PAYL_NEARLY_TO,
    LC_AUTH_PAYL_REAL_TO,

    LC_OP_LOC_SWITCH_REQ,
    LC_OP_LOC_SNIFF_REQ,
    LC_OP_LOC_UNSNIFF_REQ,
#if MAX_NB_SYNC
    LC_OP_LOC_SYNC_CON_REQ,
#endif // MAX_NB_SYNC

    LC_SAM_SUBMAP_UPDATE_IND,
    LC_SAM_MAP_UPDATE_IND,

    /*
     * ************** Msg LB->LC****************
     */
    LC_MST_KEY_REQ,
#if PCA_SUPPORT
    LC_PCA_SSCAN_CLK_IND,
#endif // PCA_SUPPORT

    /*
     * ************** Msg LMP****************
     */
    LC_OP_FEAT_IND,
    LC_OP_VERS_IND,
    LC_OP_CLKOFF_IND,
    LC_OP_TIMACC_IND,
    LC_OP_PT_IND,
#if RW_BT_MWS_COEX
    LC_OP_SAM_IND,
    LC_OP_SAM_CFM,
#endif //RW_BT_MWS_COEX
    LC_OP_HL_CNX_IND,
    LC_OP_HL_CNX_CONT_IND,
    LC_OP_AUTH_IND,
    LC_OP_ENC_IND,
    LC_OP_EPR_IND,
    LC_OP_SET_CMP_IND,
    LC_OP_NAME_REQ_IND,
    LC_OP_SSRNEGO_IND,
    LC_OP_QOS_IND,
    LC_OP_DETACH_IND,

    LC_AUTH_STG1_RANDN,
    LC_QUERY_HOST_EXP_IND,

    /*
     * ************** Msg LD->LC****************
     */
    LC_LMP_RX_IND,
    LC_LMP_TX_CFM,
    LC_ACL_RX_IND,
    LC_ACL_RX_MIC_ERR_IND,
    LC_ACL_TX_CFM,

    LC_ACL_DISC_IND,

    LC_RSW_END_IND,
    LC_RSW_SLOT_OFFSET_IND,
    LC_PWR_INCR_IND,
    LC_PWR_DECR_IND,
    LC_PWR_MAX_IND,
    LC_SSR_INST_IND,

    LC_SNIFF_OFFSET_UPD_IND,
    LC_SCO_OFFSET_UPD_IND,

#if VOICE_OVER_HCI
    LC_SYNC_RX_IND,
    LC_SYNC_TX_CFM,
#endif //VOICE_OVER_HCI

#if PCA_SUPPORT
#if (RW_BT_MWS_COEX)
    LC_LOCAL_PCA_REQ,
#endif // (RW_BT_MWS_COEX)
    LC_PCA_SSCAN_START_REQ,
    LC_PCA_SSCAN_STOP_REQ,
#endif // PCA_SUPPORT

#if (RW_BT_MWS_COEX)
    LC_MWS_PATTERN_IND,
#endif // (RW_BT_MWS_COEX)
    LC_SAM_CHANGE_IND,

#if (EAVESDROPPING_SUPPORT)
    LC_SCO_DELAY_HOST_REQ,
    LC_MIC_ENC_KEY_REFRESH,
    LC_MIC_DISC,
#endif // EAVESDROPPING_SUPPORT
};

/*
 * ************** API LM->LC ****************
 */

/// API structure to be set by LLM and used by the LLC for each new link
struct lc_init_parameters
{
    /// Purpose of the connection (0: Master | 1: Slave | 2: Rem Name)
    uint8_t reason;
    /// Peer device bd_addr
    struct bd_addr bd_addr;
    /// ACL packet types (master links only)
    uint16_t acl_pkt_type;
    /// Indicates whether role switch is enabled or not (master links only)
    uint8_t role_switch_en;
    /// Peer device Class of Device (slave links only)
    struct devclass class_of_dev;
    /// Default link policy settings
    uint8_t link_pol_stg;
    /// Slave timing information pointer (slave links only)
    void *slave_timing_info_ptr;
#if (BT_53)
    /// Minimum encryption key size (in bytes)
    uint8_t min_enc_key_size;
#if (RW_DEBUG)
    /// Maximum encryption key size (in bytes)
    uint8_t max_enc_key_size;
#endif  // (RW_DEBUG)
#endif // (BT_53)
};

/*
 * ************** API LD->LC ****************
 */

/// LC LMP reception indication structure
/*@TRACE*/
struct lc_lmp_rx_ind
{
    /// PDU length + Flags (BF - Broadcast flag)
    uint8_t lmp_len_flags;
    /// PDU (packed)
    uint8_t pdu[__ARRAY_EMPTY];
};

/// LC LMP reception structure
/*@TRACE*/
struct lc_lmp_rx
{
    /// PDU (unpacked)
    uint8_t pdu[1];
};

/// LC LMP transmission confirmation structure
/*@TRACE*/
struct lc_lmp_tx_cfm
{
    /// EM buffer containing PDU
    uint16_t em_buf;
};

/// LC ACL reception indication structure
/*@TRACE*/
struct lc_acl_rx_ind
{
    /// EM buffer containing PDU
    uint16_t em_buf;
    /// Data length + Data Flags (PBF + BF)
    uint16_t data_len_flags;
};

/// LC ACL transmission confirmation structure
/*@TRACE*/
struct lc_acl_tx_cfm
{
    /// Indicated whether buffer has been successfully transmitted (False) or flushed (True)
    bool flushed;
};

/// LC Role Switch end indication structure
/*@TRACE*/
struct lc_rsw_end_ind
{
    /// Status
    uint8_t status;
};

/// LC sniff offset update indication structure
/*@TRACE*/
struct lc_sniff_offset_upd_ind
{
    /// Sniff offset in half-slots
    uint16_t sniff_offset;
};

/// LC SCO offset update indication structure
/*@TRACE*/
struct lc_sco_offset_upd_ind
{
    /// SCO link id
    uint8_t sco_link_id;
    /// SCO offset in slots
    uint16_t sco_offset;
};

#if VOICE_OVER_HCI
/// LC SYNC reception indication structure
/*@TRACE*/
struct lc_sync_rx_ind
{
    /// EM buffer containing PDU
    uint16_t em_buf;
    /// Data length
    uint8_t data_len;
    /// Packet status flag
    uint8_t packet_status_flag;
    /// Synchronous link identifier
    uint8_t sync_link_id;
};

/// LC SYNC transmission confirmation structure
/*@TRACE*/
struct lc_sync_tx_cfm
{
    /// Synchronous link identifier
    uint8_t sync_link_id;
};
#endif //VOICE_OVER_HCI

#if PCA_SUPPORT
/// LC PCA local adjustment request structure
/*@TRACE*/
struct lc_local_pca_req
{
    /// Clock adjusted us
    int16_t clk_adj_us;
    /// Clock adjusted slots
    uint8_t clk_adj_slots;
    /// Clock adjusted period (slots)
    uint8_t clk_adj_period;
};
#endif // PCA_SUPPORT

/*
 * ************** API LB->LC ****************
 */
///LC master key request message structure
/*@TRACE*/
struct lc_mst_key_req
{
    ///New key flag
    uint8_t new_key_flag;
};

#if PCA_SUPPORT
///LB indicates clock recovery data to LC
/*@TRACE*/
struct lc_pca_sscan_clk_ind
{
    /// Clock_Offset (28 bits) - (CLKNslave - CLK) modulo 2^28
    uint32_t clock_offset;
    /// Bit offset (CLKNslave - CLK) modulo 2^28
    int16_t bit_offset;
    /// BD_ADDR
    struct bd_addr bd_addr;
};
#endif // PCA_SUPPORT

/*
 * ************** API LC->LC ****************
 */

/// Setup synchronous connection message structure
/*@TRACE*/
struct lc_op_loc_sniff_req
{
    /// Maximum interval (in slots)
    uint16_t    max_int;
    /// Minimum interval (in slots)
    uint16_t    min_int;
    /// Attempts (number of receive slots) (in slots)
    uint16_t    attempt;
    /// Timeout (number of receive slots) (in slots)
    uint16_t    timeout;
};

#if MAX_NB_SYNC
/// Setup synchronous connection message structure
/*@TRACE*/
struct lc_op_loc_sync_con_req
{
    /// Connection handle
    uint16_t conhdl;
    /// Indicate if the incoming HCI command is HCI_Setup_Sync_Con or HCI_Enhanced_Setup_Sync_Con
    bool old_style;
    /// Indicate if the procedure is initiated by the host (true) or not (false)
    bool host_initiated;
};
#endif // MAX_NB_SYNC

/// LC SAM Submap update indication structure
struct lc_sam_submap_update_ind
{
    /// Update mode
    uint8_t update_mode;
    /// Submap
    uint8_t submap[SAM_TYPE0_SUBMAP_LEN];
    /// num Tx slots
    uint16_t n_tx_slots;
    /// num Rx slots
    uint16_t n_rx_slots;
};

/// LC SAM Map update indication structure
struct lc_sam_map_update_ind
{
    /// SAM Pattern Index
    uint8_t pattern_index;
    /// Tsam-sm (slots)
    uint8_t t_sam_sm;
    /// Nsam-sm (slots)
    uint8_t n_sam_sm;
    /// SAM Submaps
    struct sam_submaps submaps;
};

/// LC MWS Pattern index update indication strucutre
struct lc_mws_pattern_ind
{
    /// Active pattern index
    uint8_t pattern_index;
    /// Interval
    //uint16_t interval;
    /// Time when pattern will be activated
    rwip_time_t time;
    /// Tsam - length of SAM pattern (slots)
    uint16_t t_sam;
    /// Tsam_sm = length of SAM submaps (slots)
    uint8_t t_sam_sm;
    /// Number of TX slots in this pattern
    uint16_t n_tx_slots;
    /// Number of RX slots in this pattern
    uint16_t n_rx_slots;
    /// Number of external submaps in this pattern
    uint8_t n_ex_sm;
};


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the BT LC task
 *
 * This function initializes the LC task, as well as the environment of the LC
 ****************************************************************************************
 */
void lc_init(void);

/**
 ****************************************************************************************
 * @brief Reset the BT LC task
 ****************************************************************************************
 */
void lc_reset(void);

/**
 ****************************************************************************************
 * @brief Start an LC task instance
 *
 * This function starts an LC task instance for a connection establishment or name
 * request.
 *
 * @param[in]   link_id       Link identifier
 * @param[in]   reason        Purpose of the connection (0: Master | 1: Slave | 2: Rem Name)
 * @param[in]   bd_addr       Peer device bd_addr
 * @param[in]   pkt_type      ACL packet types (master links only)
 * @param[in]   switch_en     Indicates whether role switch is enabled or not (master links only)
 * @param[in]   class_of_dev  Peer device Class of Device (slave links only)
 * @param[in]   link_pol_stg  Default link policy settings
 * @param[in]   slave_timing_info_ptr  Slave timing information pointer (slave links only)
 ****************************************************************************************
 */
void lc_start(uint8_t link_id, struct lc_init_parameters *con_params);

/**
 ****************************************************************************************
 * @brief Stop an LC task instance
 *
 * param[in]    pid      Process identifier (task / instance)
 * param[in]    reason   Detach reason (error code)
 ****************************************************************************************
 */
void lc_stop(ke_task_id_t pid, uint8_t reason);

/**
 ****************************************************************************************
 * @brief LC util to get max packet size - used external to LC
 *
 * @param[in]   pkt_type      ACL packet types
 *
 * @return max packet size
 ****************************************************************************************
 */
uint16_t lc_util_get_max_packet_size(uint16_t PacketType);

/**
 ****************************************************************************************
 * @brief This function is used to compute the number of slot required.
 *
 * @param[in] PacketType      Packet type
 *
 * @return Nb of computed slot of the packet type.
 ****************************************************************************************
 */
uint8_t LM_MaxSlot(uint16_t PacketType);

#if PCA_SUPPORT
    /**
    ****************************************************************************************
    * @brief This function prepares all slaves in advance of an LMP_clk_adj.
    *
    * @return status as a result of processing the request.
    ****************************************************************************************
    */
    uint8_t lc_prepare_all_slaves_for_clk_adj(void);

    /**
    ****************************************************************************************
    * @brief This function notifies all slaves of  clk_adj procedure completion.
    *
    ****************************************************************************************
    */
    void lc_notify_all_slaves_clk_adj_complete(void);
#endif // PCA_SUPPORT

/**
 ****************************************************************************************
 * @brief This function checks if all connected support secure connections.
 *
 * @return True: secure connection supported by all slaves / False otherwise
 ****************************************************************************************
 */
bool lc_check_all_slaves_support_sec_con(void);

#if (MAX_NB_SYNC > 0)
    /**
    ****************************************************************************************
    * @brief This function returns the number of synchronous links.
    *
    * @return number of synchronous links present.
    *
    ****************************************************************************************
    */
    uint8_t lm_get_nb_sync_link(void);

    /**
    ****************************************************************************************
    * @brief This function returns the minimum synchronous link interval.
    *
    * @return minimum synchronous link interval, else 0xFF if no synchronous links are present.
    *
    ****************************************************************************************
    */
    uint8_t lm_get_min_sync_intv(void);
#endif //(MAX_NB_SYNC > 0)

#if (EAVESDROPPING_SUPPORT)
/**
 ****************************************************************************************
 * @brief This function returns pointer to AFH part of LC environment.
 *
 * @return Pointer to AFH settings
 *
 ****************************************************************************************
 */
struct lc_afh_tag *lc_get_afh_settings(uint8_t link_id);

/**
 ****************************************************************************************
 * @brief This function returns pointer to LC environment.
 *
 * @return Pointer to LC environment
 *
 ****************************************************************************************
 */
struct lc_env_tag *lc_get_env(uint8_t link_id);

void lc_get_enc_params(uint8_t link_id, uint8_t *key, uint8_t *iv);

void lc_max_slot_update(void);

void lc_send_sniff_request_evt(struct lc_env_tag *lc_env_ptr,
                               uint16_t sniff_interval, uint16_t sniff_delay,
                               uint16_t sniff_attempt, uint16_t sniff_timeout);
#endif // EAVESDROPPING_SUPPORT

/// @} LC

#endif // LC_H_
