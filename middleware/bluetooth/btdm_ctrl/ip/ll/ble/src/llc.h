/**
 ****************************************************************************************
 *
 * @file llc.h
 *
 * @brief Main API file for the Link Layer Controller
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

#ifndef LLC_H_
#define LLC_H_

/**
 ****************************************************************************************
 * @defgroup LLC Link Layer Controller
 * @ingroup ROOT
 * @brief BLE Lower Layers
 *
 * The CONTROLLER contains the modules allowing the link establishment, maintenance and management.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#if(BLE_CENTRAL || BLE_PERIPHERAL)
#include "ke_task.h"      // kernel task definitions
#include "co_bt.h"
#include "co_llcp.h"


/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// API structure to be set by LLM and used by the LLC for each new link
struct llc_init_parameters
{
    /// Interval (in units of 1,25 ms, i.e. 2 slots)
    uint16_t            interval;

    /// Latency
    uint16_t            latency;

    /// Timeout (in units of 10 ms, i.e. 16 slots)
    uint16_t            timeout;

#if (BT_53) && (BLE_CENTRAL)
    /// Subrate min
    uint16_t subrate_min;
    /// Subrate max
    uint16_t subrate_max;
    /// Max latency (N x con intv)
    uint16_t subrate_max_latency;
    /// Max Continuation number (N x con evts)
    uint16_t subrate_cont_num;
    /// Max Supervision timeout (N x 10ms)
    uint16_t subrate_timeout;
#endif // (BT_53) && (BLE_CENTRAL)

    /// Channel mapping
    struct le_ch_map  ch_map;

    /**
     * Master Sleep clock accuracy (only for slave)
     * 0 251 ppm to 500 ppm
     * 1 151 ppm to 250 ppm
     * 2 101 ppm to 150 ppm
     * 3 76 ppm to 100 ppm
     * 4 51 ppm to 75 ppm
     * 5 31 ppm to 50 ppm
     * 6 21 ppm to 30 ppm
     * 7 0 ppm to 20 ppm
     */
    uint8_t master_sca;

    /// Connection transmit/receive rate (@see enum lld_rate)
    uint8_t rate;

    /// Role (0: Master | 1: Slave)
    uint8_t role;

    // Default Data Length parameters
    /// Suggested value for the Controller's maximum transmitted number of payload octets
    uint16_t   suggested_max_tx_octets;
    /// Suggested value for the Controller's maximum packet transmission time (in us)
    uint16_t   suggested_max_tx_time;

#if (EAVESDROPPING_SUPPORT)
    /// Suggested value for the Controller's maximum received number of payload octets
    uint16_t   suggested_max_rx_octets;
    /// Suggested value for the Controller's maximum packet reception time (in us)
    uint16_t   suggested_max_rx_time;
    /// Enhanced set data length enabled
    bool       ehn_set_data_length_enabled;
#endif //(EAVESDROPPING_SUPPORT)

#if (BT_53 && BLE_PERIPHERAL)
    /// Custom channel classification reporting interval (in ms) (0 if not used)
    uint16_t   ch_class_rep_intv;
#endif // (BT_53 && BLE_PERIPHERAL)

    // Default PHY parameters
    /// Phy options indicated by Host (@see enum le_phy_opt) (by default 0 if never set by Host)
    uint16_t   phy_opt;
    /// Default TX preferred PHY to use (@see enum le_phy_mask)
    uint8_t    tx_phys;
    /// Default RX preferred PHY to use (@see enum le_phy_mask)
    uint8_t    rx_phys;

#if BLE_PAST
    // Default PAST parameters
    /// Mode (@see enum per_adv_sync_info_rec_mode)
    uint8_t   past_mode;
    /// The number of periodic advertising packets that can be skipped after a successful receive
    uint16_t  past_skip;
    /// Sync timeout (Time=N*10ms)
    uint16_t  past_sync_to;
    /// CTE type (@see enum sync_cte_type)
    uint8_t   past_cte_type;
#endif // BLE_PAST
};


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Initialization/Reset of the BLE LLC task
 *
 * This function initializes the LLC task, as well as the environment of the LLC
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void llc_init(bool is_reset);


/**
 ****************************************************************************************
 * @brief Start an initialize a new LLC environment
 *
 * @param[in] link_id             link identifier.
 * @param[in] con_params          All parameters needed to start the LLC
 * @param[in] pref                Link preferences (set by host)
 *
 * @return If the environment allocation and initialization has been done or not.
 ****************************************************************************************
 */
uint8_t llc_start(uint8_t link_id, struct llc_init_parameters *con_params);

/**
 ****************************************************************************************
 * @brief Immediate Disconnection
 *
 * @param[in] link_id   Link Identifier
 * @param[in] reason    Reason of disconnection
 * @param[in] immediate True:  Try to stop event as soon as possible
 *                      False: Wait for new connection event to be programmed
 ****************************************************************************************
 */
void llc_disconnect(uint8_t link_id, uint8_t reason, bool immediate);

/**
 ****************************************************************************************
 * @brief Handles a planner request to move the activity
 *
 * @param[in] id             sch_plan identifier.
 ****************************************************************************************
 */
void llc_con_move_cbk(uint16_t id);

/**
 ****************************************************************************************
 *  @brief Modify sleep clock accuracy
 *
 * @param[in] link_id        Link identifier
 * @param[in] action         Switch to more or less accurate clock (@see enum clk_acc_action)
 ****************************************************************************************
 */
void llc_clk_acc_modify(uint8_t link_id, uint8_t action);

/**
 ****************************************************************************************
 * @brief Retrieve rolee for a specific link identifier
 *
 * @param[in]  link_id             link identifier.
 * @param[out] role                Master or slave role
 ****************************************************************************************
 */
uint8_t llc_role_get(uint8_t link_id, uint8_t *role);

#if (BLE_CENTRAL)
    /**
    ****************************************************************************************
    * @brief Create a new channel map
    *
    * @param[in] link_id        Link identifier
    * @param[in] llm_ch_map     Pointer to LLM channel map
    ****************************************************************************************
    */
    void llc_ch_map_create(uint8_t link_id, struct le_ch_map *ch_map);

    /**
    ****************************************************************************************
    *  @brief Update LLC channel map (if needed)
    *
    * @param[in] link_id        Link identifier
    * @param[in] llm_ch_map     Pointer to LLM channel map
    ****************************************************************************************
    */
    void llc_ch_map_update(uint8_t link_id, struct le_ch_map *llm_ch_map);
#endif //(BLE_CENTRAL)

#if (BLE_CIS)
    #if (BLE_CENTRAL)
        /**
        ****************************************************************************************
        * @brief Start Creation of the CIS channel from master side.
        *
        * @param[in]  link_id             link identifier.
        * @param[in]  act_id              Allocated activity ID
        * @param[in]  act_offset          Reserved planning activity offset for the stream (in half-slots)
        * @param[in]  act_margin          Reserved activity margin to use during negotiation (in half-slots)
        ****************************************************************************************
        */
        uint8_t llc_cis_create_req(uint8_t link_id, uint8_t act_id, uint16_t act_offset, uint16_t act_margin);
    #endif //(BLE_CENTRAL)

    #if (BLE_PERIPHERAL)
        /**
        ****************************************************************************************
        * @brief Inform CIS creation negotiation of slave accept / reject new CIS channel
        *
        * @param[in]  link_id   Link identifier.
        * @param[in]  status    Status of CIS Creation request (CO_ERROR_NO_ERROR = accept, else reject)
        ****************************************************************************************
        */
        void llc_cis_create_rsp(uint8_t link_id, uint8_t status);
    #endif //(BLE_PERIPHERAL)

    /**
    ****************************************************************************************
    * @brief Initiate stopping an CIS channel negotiation initiated by host.
    *
    * @param[in]  link_id   Link identifier.
    * @param[in]  act_id    Activity ID
    * @param[in]  reason    Termination reason
    ****************************************************************************************
    */
    uint8_t llc_cis_stop_req(uint8_t link_id, uint8_t act_id, uint8_t reason);

    #if (BLE_CENTRAL)
        /**
        ****************************************************************************************
        * @brief Cancels the CIS negotiation procedure when the CIS is not yet established.
        *
        * @param[in]  link_id   Link identifier.
        * @param[in]  reason    CIS cancel status code
        *
        * @return status        0: success | 1-255: error
        ****************************************************************************************
        */
        uint8_t llc_cis_cancel(uint8_t link_id, uint8_t reason);
    #endif //(BLE_CENTRAL)
#endif // (BLE_CIS)

#endif// (BLE_CENTRAL || BLE_PERIPHERAL)

/// @} LLC

#endif // LLC_H_
