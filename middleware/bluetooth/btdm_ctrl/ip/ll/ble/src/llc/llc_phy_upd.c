/**
 ****************************************************************************************
 *
 * @file llc_phy_upd.c
 *
 * @brief PHY Update Procedure
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup LLC_PHY_UPD PHY Update Procedure
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_CENTRAL || BLE_PERIPHERAL)
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "co_utils.h"    // For bit field manipulation
#include "co_math.h"     // For co_max

#include "ble_util.h"    // BLE utility functions

#include "ke_msg.h"      // Kernel message

#include "llc_int.h"    // Internal LLC API
#include "llc_llcp.h"   // Internal LLCP API

#include "lld.h"        // To update the channel map
#include "llm.h"        // To retrieve channel map to apply

#include "hci.h"        // For HCI handler
#include "btdm_patch.h"
/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/// Phys update operation indication structure definition
/*@TRACE*/
struct llc_op_phy_upd_ind
{
    /// Procedure information
    llc_procedure_t                 proc;
    ///Instant
    uint16_t    instant;
    /// Tx PHYs bit mask (@see enum le_phy_mask), resulting from peer and local preferences
    uint8_t     tx_phys;
    /// Rx PHYs bit mask (@see enum le_phy_mask), resulting from peer and local preferences
    uint8_t     rx_phys;
    /// Master to slave PHY (@see enum le_phy_mask), max 1 bit should be set
    uint8_t     m_to_s_phy;
    /// Slave to master PHY (@see enum le_phy_mask), max 1 bit should be set
    uint8_t     s_to_m_phy;
    ///Flag to indicate if requested by host
    bool        host_req;
    ///Flag to indicate if symmetric rate should be used
    bool        sym_rate;
    /// Phy options indicated by Host (@see enum le_phy_opt) (by default 0 if never set by Host)
    uint16_t    phy_opt;
};



/*
 * DEFINES
 ****************************************************************************************
 */

/*@TRACE*/
enum llc_phy_update_state
{
    // state machine of the local initiated PHY update

    /// Start PHYs Update procedure
    LLC_LOC_PHY_UPD_START,
    /// Force sending the LLCP_PHY_UPDATE_IND (Master only)
    /// because procedure initiated by slave but instant must be set by Master
    LLC_LOC_PHY_UPD_FORCE,
    /// Wait for LLCP_PHY_RSP (Master only)
    LLC_LOC_WAIT_PHY_RSP,
    /// Wait for LLCP_PHY_UPDATE_IND (Slave only)
    LLC_LOC_WAIT_PHY_UPD_IND,
    /// Wait for instant
    LLC_LOC_WAIT_PHY_UPD_INSTANT,

    // state machine of the peer initiated PHY update

    /// Start PHYs Update procedure
    LLC_REM_PHY_UPD_START,
    /// Wait for LLCP_PHY_UPDATE_IND (Slave only)
    LLC_REM_WAIT_PHY_UPD_IND,
    /// Wait for instant
    LLC_REM_WAIT_PHY_UPD_INSTANT,
};

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DECLARATIONS
 ****************************************************************************************
 */
__STATIC void llc_rem_phy_upd_proc_continue(uint8_t link_id, uint8_t state, uint8_t status);
__STATIC void llc_loc_phy_upd_proc_continue(uint8_t link_id, uint8_t state, uint8_t status);

__STATIC void llc_ll_phy_req_pdu_send(uint8_t link_id, uint8_t tx_phys, uint8_t rx_phys);
__STATIC void llc_ll_phy_rsp_pdu_send(uint8_t link_id, uint8_t tx_phys, uint8_t rx_phys);
__STATIC void llc_llcp_phy_upd_ind_pdu_send(uint8_t link_id, uint8_t m_to_s_phy, uint8_t s_to_m_phy, uint16_t instant);

__STATIC void llc_hci_le_phy_upd_cmp_evt_send(uint8_t link_id, uint8_t status, uint8_t tx_phy, uint8_t rx_phy);

__STATIC void llc_dl_chg_check(uint8_t link_id, uint8_t old_tx_phy, uint8_t old_rx_phy);
__STATIC void llc_cte_chg_check(uint8_t link_id, uint8_t tx_phy, uint8_t rx_phy);

__STATIC void llc_loc_phy_upd_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param);
__STATIC void llc_rem_phy_upd_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param);
/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * Continue execution of local procedure
 *
 *
 * @startuml llc_loc_phy_upd_proc_continue_peer_init.png
 * participant HCI
 * participant LLC
 * participant LLD
 * == Peer initiated PHY Negotiation (Master Role) ==
 * LLC --> LLC : LLC_OP_PHY_UPD_IND
 * LLC -> LLC: llc_op_phy_upd_ind_handler()
 * activate LLC
 *     hnote over LLC : LOC_PROC = Busy
 *     LLC -> LLC: llc_loc_phy_upd_proc_continue(PHY_UPD_FORCE)
 *     activate LLC
 *         alt  TX or RX or Both PHY Changes
 *             note over LLC #aqua: Compute instant
 *             note over LLC: state = WAIT_PHY_UPD_INSTANT
 *             LLC -> LLD: lld_con_phys_update()
 *             LLC-->LLD: LLCP_PHY_UPD_IND
 *         else No PHY Change
 *             note over LLC #aqua: Nothing to do, finish negotiation
 *             hnote over LLC : LOC_PROC = Idle
 *             LLC-->LLD: LLCP_PHY_UPD_IND
 *         end
 *     deactivate LLC
 * deactivate LLC
 * == Wait For PHY Update Instant ==
 * alt PHY Updated
 *     LLD --> LLC : LLD_PHY_UPD_CFM
 *     LLC -> LLC: lld_phy_upd_cfm_handler()
 *     activate LLC
 *         LLC -> LLC: llc_loc_phy_upd_proc_continue(PHY_UPD_INSTANT)
 *         activate LLC
 *             note over LLC #aqua: Store new connection parameters
 *             hnote over LLC: LOC_PROC=Idle
 *             LLC-->HCI: HCI_LE_PHY_UPD_CMP_EVT
 *         deactivate LLC
 *     deactivate LLC
 * end
 * @enduml
 *
 * @startuml llc_loc_phy_upd_proc_continue_local_init.png
 * participant HCI
 * participant LLC
 * participant LLD
 * == Local initiated PHY Negotiation ==
 * LLC --> LLC : LLC_OP_PHY_UPD_IND
 * LLC -> LLC: llc_op_phy_upd_ind_handler()
 * activate LLC
 *     hnote over LLC : LOC_PROC = Busy
 *     LLC -> LLC: llc_loc_phy_upd_proc_continue(START)
 *     activate LLC
 *         note over LLC #aqua: store preferred host parameters
 *         alt Slave Role
 *             note over LLC: state = WAIT_PHY_UPD_IND
 *             LLC-->LLD: LLCP_PHY_REQ
 *         else Master Role
 *             note over LLC: state = WAIT_PHY_RSP
 *             LLC-->LLD: LLCP_PHY_REQ
 *         end
 *     deactivate LLC
 * deactivate LLC
 * == Peer does not support feature ==
 *     LLD --> LLC : LLCP_UNKNOWN_RSP
 *     LLC -> LLC: llc_loc_phy_upd_proc_err_cb(LLCP_UNKNOWN_RSP)
 *     activate LLC
 *         note over LLC: Mark feature not supported by peer device.
 *         LLC -> LLC: llc_loc_phy_upd_proc_continue(PHY_RSP, UNSUPPORTED)
 *         activate LLC
 *             note over LLC #aqua: Stop procedure
 *             hnote over LLC: LOC_PROC=Idle
 *             alt Host initiated feature
 *                 LLC-->HCI: HCI_LE_PHY_UPD_CMP_EVT
 *             end
 *         deactivate LLC
 *     deactivate LLC
 * == Wait For PHY Response (Master Only) ==
 *     LLD --> LLC : LLCP_PHY_RSP
 *     LLC -> LLC: ll_phy_rsp_handler()
 *     activate LLC
 *         note over LLC: Store peer PHYs
 *         LLC -> LLC: llc_loc_phy_upd_proc_continue(PHY_RSP)
 *         activate LLC
 *             note over LLC: Compute PHYs
 *             alt  TX or RX or Both PHY Changes
 *                 note over LLC #aqua: Compute instant
 *                 note over LLC: state = WAIT_PHY_UPD_INSTANT
 *                 LLC -> LLD: lld_con_phys_update()
 *                 LLC-->LLD: LLCP_PHY_UPD_IND
 *             else No PHY Change
 *                 note over LLC #aqua: Nothing to do, finish negotiation
 *                 hnote over LLC : LOC_PROC = Idle
 *                 LLC-->LLD: LLCP_PHY_UPD_IND
 *                 alt Host initiated feature
 *                     LLC-->HCI: HCI_LE_PHY_UPD_CMP_EVT
 *                 end
 *             end
 *         deactivate LLC
 *     deactivate LLC
 * == Wait For PHY Update Indication (SlaveOnly) ==
 *     LLD --> LLC : LLCP_PHY_UPD_IND
 *     LLC -> LLC: ll_phy_update_ind_handler()
 *     activate LLC
 *         note over LLC: Store peer configured PHYs
 *         LLC -> LLC: llc_loc_phy_upd_proc_continue(PHY_RSP)
 *         activate LLC
 *             alt  TX or RX or Both PHY Changes
 *                 note over LLC: state = WAIT_PHY_UPD_INSTANT
 *                 LLC -> LLD: lld_con_phys_update()
 *             else No PHY Change
 *                 note over LLC #aqua: Nothing to do, finish negotiation
 *                 hnote over LLC : LOC_PROC = Idle
 *                 alt Host initiated feature
 *                     LLC-->HCI: HCI_LE_PHY_UPD_CMP_EVT
 *                 end
 *             end
 *         deactivate LLC
 *     deactivate LLC
 * == Wait For PHY Update Instant ==
 * alt PHY Updated
 *     LLD --> LLC : LLD_PHY_UPD_CFM
 *     LLC -> LLC: lld_phy_upd_cfm_handler()
 *     activate LLC
 *         LLC -> LLC: llc_loc_phy_upd_proc_continue(PHY_UPD_INSTANT)
 *         activate LLC
 *             note over LLC #aqua: Store new connection parameters
 *             hnote over LLC: LOC_PROC=Idle
 *             LLC-->HCI: HCI_LE_PHY_UPD_CMP_EVT
 *         deactivate LLC
 *     deactivate LLC
 * end
 * @enduml
 *
 *
 * @param[in] link_id Link identifier
 * @param[in] state   Expected state of the procedure
 * @param[in] status  Status of the operation
 ****************************************************************************************
 */
__STATIC void llc_loc_phy_upd_proc_continue(uint8_t link_id, uint8_t state, uint8_t status)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_LOC_PHY_UPD_PROC_CONTINUE_C_FUN_BIT, link_id, state, status);
    /// Gets the LLC environment dedicated to this link
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    /// retrieve procedure parameters
    struct llc_op_phy_upd_ind *param = (struct llc_op_phy_upd_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);
    bool finished = false;
    uint8_t old_tx_phy = llc_env_ptr->con_params.tx_phy;
    uint8_t old_rx_phy = llc_env_ptr->con_params.rx_phy;

    // an unexpected error occurs, close the procedure
    if (status != CO_ERROR_NO_ERROR)
    {
        // Stop LLCP exchange timer
        llc_proc_timer_set(link_id, LLC_PROC_LOCAL, false);

        // Restore the TX length as per current TX rate
        lld_con_tx_len_update_for_rate(link_id, CO_RATE_UNDEF);

        finished = true;
    }
    else
    {
        // check that current procedure state equals to expected state given in parameter
        if (llc_proc_state_get(&param->proc) != state)
        {
            ASSERT_WARN(0, llc_proc_state_get(&param->proc), state);
            // unexpected state, ignore it
        }
        else
        {
            switch (llc_proc_state_get(&param->proc))
            {
            // Procedure started
            case LLC_LOC_PHY_UPD_START:
            {
                uint16_t current_phy_opt = llc_env_ptr->phy_opt;

                // procedure with instant started
                SETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC, true);

                // store new preferred values for PHY selection
                llc_env_ptr->rx_phys = param->rx_phys;
                llc_env_ptr->tx_phys = param->tx_phys;
                llc_env_ptr->phy_opt = param->phy_opt;

                // Check if the PHY needs to be changed
                if ((co_phy_value_to_mask[llc_env_ptr->con_params.rx_phy] & llc_env_ptr->rx_phys) && (co_phy_value_to_mask[llc_env_ptr->con_params.tx_phy] & llc_env_ptr->tx_phys))
                {
                    // If the Tx PHY is coded and the host prefers a different rate (125 or 500 Kbps)
                    if ((llc_env_ptr->con_params.tx_phy == PHY_CODED_VALUE) && (param->phy_opt != PHY_OPT_NO_LE_CODED_TX_PREF) && (param->phy_opt != current_phy_opt))
                    {
                        uint8_t rx_rate = co_phy_to_rate[llc_env_ptr->con_params.rx_phy];
                        uint8_t tx_rate = (param->phy_opt == PHY_OPT_S8_LE_CODED_TX_PREF) ? CO_RATE_125KBPS : CO_RATE_500KBPS;

                        // There is no negotiation needed, the local driver can apply the change as soon as possible
                        param->instant = lld_con_event_counter_get(link_id);

                        // request driver to update the PHY
                        status = lld_con_phys_update(link_id, tx_rate, rx_rate, param->instant);

                        if (status == CO_ERROR_NO_ERROR)
                        {
                            llc_proc_state_set(&param->proc, link_id, LLC_LOC_WAIT_PHY_UPD_INSTANT);
                        }
                        else // update rejected.
                        {
                            // finish procedure without changing the rate
                            finished = true;
                            param->instant = 0;
                            param->m_to_s_phy = 0;
                            param->s_to_m_phy = 0;
                        }
                    }
                    else
                    {
                        finished = true;
                    }
                }
                else
                {
                    // First send the LLCP_PHY_REQ
                    llc_ll_phy_req_pdu_send(link_id, param->tx_phys, param->rx_phys);

                    // Start the LLCP Response TO
                    llc_proc_timer_set(link_id, LLC_PROC_LOCAL, true);

                    // if master wait for LLCP_PHY_RSP
                    if (GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
                    {
                        llc_proc_state_set(&param->proc, link_id, LLC_LOC_WAIT_PHY_RSP);
                    }
                    // if slave wait for LLCP_PHY_UPDATE_IND
                    else
                    {
                        // Find the slowest rate
                        uint8_t slowest_tx_rate = CO_RATE_2MBPS;

                        if (param->tx_phys & PHY_CODED_BIT)
                        {
                            slowest_tx_rate = (param->phy_opt == PHY_OPT_S8_LE_CODED_TX_PREF) ? CO_RATE_125KBPS : CO_RATE_500KBPS;
                        }
                        else if (param->tx_phys & PHY_1MBPS_BIT)
                        {
                            slowest_tx_rate = CO_RATE_1MBPS;
                        }

                        // Update the TX length as per the slowest TX rate
                        lld_con_tx_len_update_for_rate(link_id, slowest_tx_rate);

                        llc_proc_state_set(&param->proc, link_id, LLC_LOC_WAIT_PHY_UPD_IND);
                    }
                }
            }
            break;
            case LLC_LOC_WAIT_PHY_RSP:
            {
                // Stop LLCP exchange timer
                llc_proc_timer_set(link_id, LLC_PROC_LOCAL, false);
            }
            // no break
            case LLC_LOC_PHY_UPD_FORCE:
            {
                // Select TX rate to apply
                if (param->tx_phys != 0)
                {
                    // Check if the current rate is among the possible rates
                    if (param->tx_phys & co_phy_value_to_mask[llc_env_ptr->con_params.tx_phy])
                    {
                        // The rate does not need to be changed
                        param->m_to_s_phy = 0;
                    }
                    else
                    {
                        // Select one single PHY among the possible ones. Priority 2M -> 1M -> Uncoded
                        if (GETB(param->tx_phys, PHY_2MBPS))
                        {
                            param->m_to_s_phy = PHY_2MBPS_BIT;
                        }
                        else if (GETB(param->tx_phys, PHY_1MBPS))
                        {
                            param->m_to_s_phy = PHY_1MBPS_BIT;
                        }
                        else
                        {
                            param->m_to_s_phy = PHY_CODED_BIT;
                        }
                    }
                }

                // Select RX rate to apply
                if (param->rx_phys != 0)
                {
                    // Check if the current rate is among the possible rates
                    if (param->rx_phys & co_phy_value_to_mask[llc_env_ptr->con_params.rx_phy])
                    {
                        // The rate does not need to be changed
                        param->s_to_m_phy = 0;
                    }
                    else
                    {
                        // Select one single PHY among the possible ones. Priority 2M -> 1M -> Uncoded
                        if (GETB(param->rx_phys, PHY_2MBPS))
                        {
                            param->s_to_m_phy = PHY_2MBPS_BIT;
                        }
                        else if (GETB(param->rx_phys, PHY_1MBPS))
                        {
                            param->s_to_m_phy = PHY_1MBPS_BIT;
                        }
                        else
                        {
                            param->s_to_m_phy = PHY_CODED_BIT;
                        }
                    }
                }

                /* If the slave specified a single PHY in both the TX_PHYS and RX_PHYS fields and both fields are
                 * the same, the master shall either select the PHY specified by the slave for both directions or
                 * shall leave both directions unchanged */
                if (param->sym_rate)
                {
                    // Compute the resulting PHY after update
                    uint8_t select_tx_phy = (param->m_to_s_phy == 0) ? llc_env_ptr->con_params.tx_phy : co_phy_mask_to_value[param->m_to_s_phy];
                    uint8_t select_rx_phy = (param->s_to_m_phy == 0) ? llc_env_ptr->con_params.rx_phy : co_phy_mask_to_value[param->s_to_m_phy];

                    // If selected PHYs are different, no PHY change
                    if (select_tx_phy != select_rx_phy)
                    {
                        param->s_to_m_phy = 0;
                        param->m_to_s_phy = 0;
                    }
                }

                // nothing has changed, just inform peer device that procedure is finished
                if ((param->m_to_s_phy == 0) && (param->s_to_m_phy == 0))
                {
                    // The Instant field is RFU in this case
                    param->instant = 0;
                    finished = true;
                }
                else
                {
                    uint8_t tx_phy, rx_phy;
                    uint8_t tx_rate, rx_rate;

                    // Get the TX PHY to apply to the driver
                    tx_phy = (param->m_to_s_phy == 0) ? llc_env_ptr->con_params.tx_phy : co_phy_mask_to_value[param->m_to_s_phy];
                    tx_rate = co_phy_to_rate[tx_phy];

                    // For coded PHY, check if S8 is preferred by the Host
                    if ((tx_phy == PHY_CODED_VALUE) && (param->phy_opt == PHY_OPT_S8_LE_CODED_TX_PREF))
                    {
                        tx_rate = CO_RATE_125KBPS;
                    }

                    // Get the RX PHY to apply to the driver
                    rx_phy = (param->s_to_m_phy == 0) ? llc_env_ptr->con_params.rx_phy : co_phy_mask_to_value[param->s_to_m_phy];
                    rx_rate = co_phy_to_rate[rx_phy];

                    // Compute procedure instant
                    param->instant = llc_inst_select(link_id);

                    // request driver to update the rate
                    status = lld_con_phys_update(link_id, tx_rate, rx_rate, param->instant);

                    // request driver to perform channel map update
                    if (status == CO_ERROR_NO_ERROR)
                    {
                        llc_proc_state_set(&param->proc, link_id, LLC_LOC_WAIT_PHY_UPD_INSTANT);

                        llc_cte_chg_check(link_id, tx_phy, rx_phy);
                    }
                    else // update rejected.
                    {
                        // finish procedure without changing the rate
                        finished = true;
                        param->instant = 0;
                        param->m_to_s_phy = 0;
                        param->s_to_m_phy = 0;
                    }
                }

                // send the LLCP_PHY_UPDATE_IND
                llc_llcp_phy_upd_ind_pdu_send(link_id, param->m_to_s_phy, param->s_to_m_phy, param->instant);
            }
            break;
            case LLC_LOC_WAIT_PHY_UPD_IND:
            {
                // Stop LLCP exchange timer
                llc_proc_timer_set(link_id, LLC_PROC_LOCAL, false);

                // nothing has changed, just inform peer device that procedure is finished
                if ((param->m_to_s_phy == 0) && (param->s_to_m_phy == 0))
                {
                    // Restore the TX length as per the current TX rate
                    lld_con_tx_len_update_for_rate(link_id, CO_RATE_UNDEF);

                    finished = true;
                }
                else
                {
                    uint8_t tx_phy, rx_phy;
                    uint8_t tx_rate, rx_rate;

                    // Get the TX PHY to apply to the driver
                    tx_phy = (param->s_to_m_phy == 0) ? llc_env_ptr->con_params.tx_phy : co_phy_mask_to_value[param->s_to_m_phy];
                    tx_rate = co_phy_to_rate[tx_phy];

                    // For coded PHY, check if S8 is preferred by the Host
                    if ((tx_phy == PHY_CODED_VALUE) && (param->phy_opt == PHY_OPT_S8_LE_CODED_TX_PREF))
                    {
                        tx_rate = CO_RATE_125KBPS;
                    }

                    // Get the RX PHY to apply to the driver
                    rx_phy = (param->m_to_s_phy == 0) ? llc_env_ptr->con_params.rx_phy : co_phy_mask_to_value[param->m_to_s_phy];
                    rx_rate = co_phy_to_rate[rx_phy];

                    // request driver to update the rate
                    status = lld_con_phys_update(link_id, tx_rate, rx_rate, param->instant);

                    // request driver to perform channel map update
                    if (status == CO_ERROR_NO_ERROR)
                    {
                        llc_proc_state_set(&param->proc, link_id, LLC_LOC_WAIT_PHY_UPD_INSTANT);

                        llc_cte_chg_check(link_id, tx_phy, rx_phy);
                    }
                    else // update rejected.
                    {
                        // finish procedure without changing the rate
                        finished = true;
                        ASSERT_INFO_FORCE(0, param->s_to_m_phy, param->m_to_s_phy);
                    }
                }
            }
            break;

            case LLC_LOC_WAIT_PHY_UPD_INSTANT:
            {
                // nothing more to do.
                finished = true;

                // save new PHY parameters
                if (param->m_to_s_phy != 0)
                {
                    if (GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
                    {
                        llc_env_ptr->con_params.tx_phy = co_phy_mask_to_value[param->m_to_s_phy];
                    }
                    else
                    {
                        llc_env_ptr->con_params.rx_phy = co_phy_mask_to_value[param->m_to_s_phy];
                    }
                }
                if (param->s_to_m_phy != 0)
                {
                    if (GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
                    {
                        llc_env_ptr->con_params.rx_phy = co_phy_mask_to_value[param->s_to_m_phy];
                    }
                    else
                    {
                        llc_env_ptr->con_params.tx_phy = co_phy_mask_to_value[param->s_to_m_phy];
                    }
                }
            }
            break;

            default:
            {
                status = CO_ERROR_UNSPECIFIED_ERROR;
                ASSERT_INFO_FORCE(0, link_id, llc_proc_state_get(&param->proc));
            }
            break;
            }
        }
    }

    if (finished)
    {
        // procedure with instant finished
        SETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC, false);

        // check if HCI event should be sent or not.
        if ((param->host_req) || (param->m_to_s_phy != 0) || (param->s_to_m_phy != 0))
        {
            llc_hci_le_phy_upd_cmp_evt_send(link_id, status, llc_env_ptr->con_params.tx_phy, llc_env_ptr->con_params.rx_phy);

            // Check if the data length has changed and send an event to the host if needed
            if (status == CO_ERROR_NO_ERROR)
            {
                llc_dl_chg_check(link_id, old_tx_phy, old_rx_phy);
            }

            // Clear the Host flow control flag
            if (param->host_req)
            {
                SETB(llc_env_ptr->flow_ctrl, LLC_HCI_SET_PHY_REQ, false);
            }
        }

        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_LOCAL);

#if (BT_53) && (BLE_CENTRAL)
        // start procedure to resume subrating if needed
        llc_subrate_restore(link_id, LLC_SUBRATE_FULL_RESTORE);
#endif // (BT_53) && (BLE_CENTRAL)
    }
}


/**
 ****************************************************************************************
 * Continue execution of remote procedure
 *
 * @startuml llc_rem_phy_upd_proc_continue.png
 * participant HCI
 * participant LLC
 * participant LLD
 * == Reception of PHY Request from peer device ==
 * LLD --> LLC : LLCP_PHY_REQ
 * LLC -> LLC: ll_phy_req_handler()
 * activate LLC
 *     note over LLC #aqua: Allocate Remote procedure structure\nstore peer parameter\nregister procedure
 *     hnote over LLC: REM_PROC=busy
 *     LLC -> LLC: llc_rem_phy_upd_proc_continue(START)
 *     activate LLC
 *         alt Master Role
 *             note over LLC #lightgreen: Create a new Local procedure to manage instant\n see __llc_loc_phy_upd_proc_continue__
 *             LLC --> LLC: LLC_OP_PHY_UPD_IND
 *             hnote over LLC: REM_PROC=Idle
 *         else Slave Role
 *             note over LLC: state = WAIT_PHY_UPD_IND
 *             LLC --> LLD: LLCP_PHY_RSP
 *         end
 *     deactivate LLC
 * deactivate LLC
 * == Wait For PHY Update Indication (Slave Only) ==
 * alt Slave Role
 *     LLD --> LLC : LLCP_PHY_UPD_IND
 *     LLC -> LLC: ll_phy_update_ind_handler()
 *     activate LLC
 *         note over LLC #aqua: Store PHY programmed by peer
 *         LLC -> LLC: llc_rem_phy_upd_proc_continue(PHY_UPD_IND)
 *         activate LLC
 *             alt No change in PHY parameters
 *                 note over LLC #aqua: Nothing to do
 *                 hnote over LLC: REM_PROC=Idle
 *             else PHY Updated (TX or RX or both)
 *                 note over LLC: state = WAIT_PHY_UPD_INSTANT
 *                 note over LLC #aqua: Configure new PHY at expected instant
 *                 LLC -> LLD: lld_con_phys_update()
 *             end
 *         deactivate LLC
 *     deactivate LLC
 * end
 * == Wait For PHY Update Instant (Slave Only) ==
 * alt Slave Role, PHY Updated
 *     LLD --> LLC : LLD_PHY_UPD_CFM
 *     LLC -> LLC: lld_phy_upd_cfm_handler()
 *     activate LLC
 *         LLC -> LLC: llc_rem_phy_upd_proc_continue(PHY_UPD_INSTANT)
 *         activate LLC
 *             note over LLC #aqua: Store new connection parameters
 *             hnote over LLC: REM_PROC=Idle
 *             LLC-->HCI: HCI_LE_PHY_UPD_CMP_EVT
 *         deactivate LLC
 *     deactivate LLC
 * end
 * @enduml
 *
 * @param[in] link_id Link identifier
 * @param[in] state   Expected state of the procedure
 * @param[in] status  Status of the operation
 ****************************************************************************************
 */
__STATIC void llc_rem_phy_upd_proc_continue(uint8_t link_id, uint8_t state, uint8_t status)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_REM_PHY_UPD_PROC_CONTINUE_C_FUN_BIT, link_id, state, status);
    /// Gets the LLC environment dedicated to this link
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    /// retrieve procedure parameters
    struct llc_op_phy_upd_ind *param = (struct llc_op_phy_upd_ind *) llc_proc_get(link_id, LLC_PROC_REMOTE);
    bool finished = false;
    uint8_t old_tx_phy = llc_env_ptr->con_params.tx_phy;
    uint8_t old_rx_phy = llc_env_ptr->con_params.rx_phy;

    // an unexpected error occurs, close the procedure
    if (status != CO_ERROR_NO_ERROR)
    {
        // Restore the TX length as per the current TX rate
        lld_con_tx_len_update_for_rate(link_id, CO_RATE_UNDEF);

        finished = true;
    }
    else
    {
        // check that current procedure state equals to expected state given in parameter
        if ((llc_proc_state_get(&param->proc) != state))
        {
            ASSERT_WARN(0, llc_proc_state_get(&param->proc), state);
            // unexpected state, ignore it
        }
        else
        {
            switch (llc_proc_state_get(&param->proc))
            {
            // Procedure started
            case LLC_REM_PHY_UPD_START:
            {
                // procedure with instant started
                SETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC, true);

                // if master, check if PHY has to be renew.
                if (GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
                {
                    ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
                    // Transform remote procedure to a local procedure to manage instant correctly
                    struct llc_op_phy_upd_ind *phy_update = KE_MSG_ALLOC(LLC_OP_PHY_UPD_IND, llc_id, llc_id, llc_op_phy_upd_ind);

                    llc_proc_init(&phy_update->proc, LLC_PROC_PHY_UPDATE, llc_loc_phy_upd_proc_err_cb);
                    llc_proc_state_set(&phy_update->proc, link_id, LLC_LOC_PHY_UPD_FORCE);
                    // store only meaning full information
                    phy_update->tx_phys        = llc_env_ptr->tx_phys & param->tx_phys;
                    phy_update->rx_phys        = llc_env_ptr->rx_phys & param->rx_phys;
                    phy_update->phy_opt        = llc_env_ptr->phy_opt;
                    phy_update->sym_rate       = param->sym_rate;
                    phy_update->host_req       = false;

                    phy_update->m_to_s_phy = 0;
                    phy_update->s_to_m_phy = 0;

                    ke_msg_send(phy_update);

                    // consider remote procedure finished
                    finished = true;
                }
                // if slave, send PHY Response and wait for PHY update indication
                else
                {
                    // Find the slowest rate
                    uint8_t slowest_tx_rate = CO_RATE_2MBPS;

                    if (llc_env_ptr->tx_phys & param->tx_phys & PHY_CODED_BIT)
                    {
                        slowest_tx_rate = (llc_env_ptr->phy_opt == PHY_OPT_S8_LE_CODED_TX_PREF) ? CO_RATE_125KBPS : CO_RATE_500KBPS;
                    }
                    else if (llc_env_ptr->tx_phys & param->tx_phys & PHY_1MBPS_BIT)
                    {
                        slowest_tx_rate = CO_RATE_1MBPS;
                    }

                    // Update the TX length as per the slowest TX rate
                    lld_con_tx_len_update_for_rate(link_id, slowest_tx_rate);

                    llc_proc_state_set(&param->proc, link_id, LLC_REM_WAIT_PHY_UPD_IND);

                    // send preferred PHYs
                    llc_ll_phy_rsp_pdu_send(link_id, llc_env_ptr->tx_phys, llc_env_ptr->rx_phys);

                    // Start the LLCP Response TO
                    llc_proc_timer_set(link_id, LLC_PROC_REMOTE, true);
                }
            }
            break;

            // PHY information received from peer device
            case LLC_REM_WAIT_PHY_UPD_IND:
            {
                // Stop LLCP exchange timer
                llc_proc_timer_set(link_id, LLC_PROC_REMOTE, false);

                // nothing has changed, just inform peer device that procedure is finished
                if ((param->m_to_s_phy == 0) && (param->s_to_m_phy == 0))
                {
                    // Restore the TX length as per the current TX rate
                    lld_con_tx_len_update_for_rate(link_id, CO_RATE_UNDEF);

                    finished = true;
                }
                else
                {
                    uint8_t tx_phy, rx_phy;
                    uint8_t tx_rate, rx_rate;

                    // Get the TX PHY to apply to the driver
                    tx_phy = (param->s_to_m_phy == 0) ? llc_env_ptr->con_params.tx_phy : co_phy_mask_to_value[param->s_to_m_phy];
                    tx_rate = co_phy_to_rate[tx_phy];

                    // For coded PHY, check if S8 is preferred by the Host
                    if ((tx_phy == PHY_CODED_VALUE) && (param->phy_opt == PHY_OPT_S8_LE_CODED_TX_PREF))
                    {
                        tx_rate = CO_RATE_125KBPS;
                    }

                    // Get the RX PHY to apply to the driver
                    rx_phy = (param->m_to_s_phy == 0) ? llc_env_ptr->con_params.rx_phy : co_phy_mask_to_value[param->m_to_s_phy];
                    rx_rate = co_phy_to_rate[rx_phy];

                    // request driver to update the rate
                    status = lld_con_phys_update(link_id, tx_rate, rx_rate, param->instant);

                    if (status == CO_ERROR_NO_ERROR)
                    {
                        llc_proc_state_set(&param->proc, link_id, LLC_REM_WAIT_PHY_UPD_INSTANT);

                        llc_cte_chg_check(link_id, tx_phy, rx_phy);
                    }
                    else // update rejected.
                    {
                        // finish procedure without changing the rate
                        finished = true;
                        ASSERT_INFO_FORCE(0, param->s_to_m_phy, param->m_to_s_phy);
                    }
                }
            }
            break;

            // PHY instant occurs, update information
            case LLC_REM_WAIT_PHY_UPD_INSTANT:
            {
                finished = true;

                // save new PHY parameters
                if (param->m_to_s_phy != 0)
                {
                    llc_env_ptr->con_params.rx_phy = co_phy_mask_to_value[param->m_to_s_phy];
                }
                if (param->s_to_m_phy != 0)
                {
                    llc_env_ptr->con_params.tx_phy = co_phy_mask_to_value[param->s_to_m_phy];
                }
            }
            break;
            default:
            {
                status = CO_ERROR_UNSPECIFIED_ERROR;
                ASSERT_INFO_FORCE(0, link_id, llc_proc_state_get(&param->proc));
            }
            break;
            }
        }
    }

    if (finished)
    {
        // Slave role only
        if (!GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
        {
            // procedure with instant started
            SETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC, false);

            // check if HCI event should be sent or not.
            if (((param->m_to_s_phy != 0) || (param->s_to_m_phy != 0))
                    && (status != CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE)
                    && (llc_env_ptr->con_params.rx_phy != PHY_UNDEF_VALUE)
                    && (llc_env_ptr->con_params.tx_phy != PHY_UNDEF_VALUE))
            {
                llc_hci_le_phy_upd_cmp_evt_send(link_id, status, llc_env_ptr->con_params.tx_phy, llc_env_ptr->con_params.rx_phy);

                // Check if the data length has changed and send an event to the host if needed
                if (status == CO_ERROR_NO_ERROR)
                {
                    llc_dl_chg_check(link_id, old_tx_phy, old_rx_phy);
                }
            }
        }

        // unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_REMOTE);

#if (BT_53) && (BLE_CENTRAL)
        // start procedure to resume subrating if needed
        llc_subrate_restore(link_id, LLC_SUBRATE_FULL_RESTORE);
#endif // (BT_53) && (BLE_CENTRAL)
    }
}

/**
 ****************************************************************************************
 * @brief Sends the phy request pdu.
 *
 * @param[in] link_id    Link identifier
 * @param[in] tx_phys    TX PHY selection
 * @param[in] rx_phys    RX PHY selection
 ****************************************************************************************
 */
__STATIC void llc_ll_phy_req_pdu_send(uint8_t link_id, uint8_t tx_phys, uint8_t rx_phys)
{
    struct ll_phy_req pdu;

    pdu.op_code = LL_PHY_REQ_OPCODE;
    pdu.rx_phys = rx_phys;
    pdu.tx_phys = tx_phys;

    llc_llcp_send(link_id, (union llcp_pdu *)&pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Sends the phy response pdu.
 *
 * @param[in] link_id    Link identifier
 * @param[in] tx_phys    TX PHY selection
 * @param[in] rx_phys    RX PHY selection
 ****************************************************************************************
 */
__STATIC void llc_ll_phy_rsp_pdu_send(uint8_t link_id, uint8_t tx_phys, uint8_t rx_phys)
{
    struct ll_phy_rsp pdu;

    pdu.op_code  = LL_PHY_RSP_OPCODE;
    pdu.rx_phys  = rx_phys;
    pdu.tx_phys  = tx_phys;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Sends the PHY update indication PDU.
 *
 * @param[in] link_id    Link identifier
 * @param[in] m_to_s_phy Master to slave PHY selected
 * @param[in] s_to_m_phy Slave to master PHY selected
 * @param[in] instant    Event time when new values has to be taken into account
 ****************************************************************************************
 */
__STATIC void llc_llcp_phy_upd_ind_pdu_send(uint8_t link_id, uint8_t m_to_s_phy, uint8_t s_to_m_phy, uint16_t instant)
{
    struct ll_phy_update_ind pdu;

    pdu.op_code     = LL_PHY_UPDATE_IND_OPCODE;
    pdu.m_to_s_phy  = m_to_s_phy;
    pdu.s_to_m_phy  = s_to_m_phy;
    pdu.instant     = instant;

    llc_llcp_send(link_id, (union llcp_pdu *) &pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Send HCI_LE_PHY_UPD_CMP_EVT to host
 *
 * @param[in] link_id        Link identifier
 * @param[in] status         Status of Encryption procedure
 * @param[in] tx_phy         Negotiated TX PHY Rate (mask, with only 1 bit set)
 * @param[in] rx_phy         Negotiated RX PHY Rate (mask, with only 1 bit set)
 ****************************************************************************************
 */
__STATIC void llc_hci_le_phy_upd_cmp_evt_send(uint8_t link_id, uint8_t status, uint8_t tx_phy, uint8_t rx_phy)
{
    ASSERT_INFO((tx_phy == PHY_1MBPS_VALUE) || (tx_phy == PHY_2MBPS_VALUE) || (tx_phy == PHY_CODED_VALUE), tx_phy, 0);
    ASSERT_INFO((rx_phy == PHY_1MBPS_VALUE) || (rx_phy == PHY_2MBPS_VALUE) || (rx_phy == PHY_CODED_VALUE), rx_phy, 0);

    uint16_t conhdl = BLE_LINKID_TO_CONHDL(link_id);

    // allocates the message to send
    struct hci_le_phy_upd_cmp_evt *phy_chg = KE_MSG_ALLOC(HCI_LE_EVENT, conhdl, HCI_LE_META_EVT_CODE, hci_le_phy_upd_cmp_evt);
    phy_chg->conhdl  = conhdl;
    phy_chg->subcode = HCI_LE_PHY_UPD_CMP_EVT_SUBCODE;
    phy_chg->tx_phy  = tx_phy;
    phy_chg->rx_phy  = rx_phy;
    phy_chg->status  = status;
    hci_send_2_host(phy_chg);
}

/**
 * Local procedure callback used to inform if an unexpected error is raised during procedure execution
 *
 * @param[in] link_id     Link Identifier
 * @param[in] error_type  Error type (@see enum llc_error_type)
 * @param[in] param       Parameter according to error type:
 *   - LLC_ERR_DISCONNECT:          reason
 *   - LLC_ERR_LLCP_UNKNOWN_RSP:    struct ll_unknown_rsp*
 *   - LLC_ERR_LLCP_REJECT_IND:     struct llcp_reject_ind*
 *   - LLC_ERR_LLCP_REJECT_IND_EXT: struct ll_reject_ext_ind*
 */
__STATIC void llc_loc_phy_upd_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    switch (error_type)
    {
    case LLC_ERR_DISCONNECT:
    {
        status = *((uint8_t *) param);
    }
    break;
    case LLC_ERR_LLCP_UNKNOWN_RSP:
    {
        struct ll_unknown_rsp *rsp = (struct ll_unknown_rsp *) param;
        if (rsp->unk_type == LL_PHY_REQ_OPCODE)
        {
            status = CO_ERROR_UNKNOWN_LMP_PDU;
        }
    }
    break;
    case LLC_ERR_LLCP_REJECT_IND:
    {
        struct ll_reject_ind *reject = (struct ll_reject_ind *) param;
        status =  reject->err_code;
    }
    break;
    case LLC_ERR_LLCP_REJECT_EXT_IND:
    {
        struct ll_reject_ext_ind *reject_ext = (struct ll_reject_ext_ind *) param;
        if (reject_ext->rej_op_code == LL_PHY_REQ_OPCODE)
        {
            status =  reject_ext->err_code;
        }
    }
    break;
    default: /* Nothing to do, ignore */
        break;
    }

    if (status != CO_ERROR_NO_ERROR)
    {
        if ((status == CO_ERROR_UNKNOWN_LMP_PDU) || (status == CO_ERROR_UNSUPPORTED_REMOTE_FEATURE))
        {
            llc_le_feature_set(link_id, BLE_FEAT_2M_PHY, false);
        }

        llc_loc_phy_upd_proc_continue(link_id, 0, status);
    }
}

/**
 * Remote procedure callback used to inform if an unexpected error is raised during procedure execution
 *
 * @param[in] link_id     Link Identifier
 * @param[in] error_type  Error type (@see enum llc_error_type)
 * @param[in] param       Parameter according to error type:
 *   - LLC_ERR_DISCONNECT:          reason
 *   - LLC_ERR_LLCP_UNKNOWN_RSP:    struct ll_unknown_rsp*
 *   - LLC_ERR_LLCP_REJECT_IND:     struct llcp_reject_ind*
 *   - LLC_ERR_LLCP_REJECT_IND_EXT: struct ll_reject_ext_ind*
 */
__STATIC void llc_rem_phy_upd_proc_err_cb(uint8_t link_id, uint8_t error_type, void *param)
{
    switch (error_type)
    {
    // link disconnection occurs
    case LLC_ERR_DISCONNECT:
    {
        uint8_t reason = *((uint8_t *) param);
        llc_rem_phy_upd_proc_continue(link_id, 0, reason);
    }
    break;
    case LLC_ERR_LLCP_UNKNOWN_RSP:
    case LLC_ERR_LLCP_REJECT_EXT_IND:
    case LLC_ERR_LLCP_REJECT_IND:
    {
        // nothing to do, ignore.
    } break;
    default:
    {
        // not expected at all
        ASSERT_INFO_FORCE(0, link_id, error_type);
    }
    break;
    }
}

/**
 ****************************************************************************************
 * @brief Checks if the data length has changed and sends the data length effective values to the host if needed.
 *
 * @param[in] link_id        Link identifier
 * @param[in] old_tx_phy     The Tx PHY before the update (@see enum le_phy_value)
 * @param[in] old_rx_phy     The Rx PHY before the update (@see enum le_phy_value)
 ****************************************************************************************
 */
__STATIC void llc_dl_chg_check(uint8_t link_id, uint8_t old_tx_phy, uint8_t old_rx_phy)
{
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint8_t new_tx_phy = llc_env_ptr->con_params.tx_phy;
    uint8_t new_rx_phy = llc_env_ptr->con_params.rx_phy;
    bool new_tx_phy_coded = (new_tx_phy == PHY_CODED_VALUE);
    bool new_rx_phy_coded = (new_rx_phy == PHY_CODED_VALUE);
    bool old_tx_phy_coded = (old_tx_phy == PHY_CODED_VALUE);
    bool old_rx_phy_coded = (old_rx_phy == PHY_CODED_VALUE);

    if ((new_tx_phy_coded != old_tx_phy_coded) || (new_rx_phy_coded != old_rx_phy_coded))
    {
        // Save the current effective values
        uint16_t old_eff_max_tx_octets = llc_env_ptr->con_params.eff_max_tx_octets;
        uint16_t old_eff_max_rx_octets = llc_env_ptr->con_params.eff_max_rx_octets;
        uint16_t old_eff_max_tx_time   = llc_env_ptr->con_params.eff_max_tx_time;
        uint16_t old_eff_max_rx_time   = llc_env_ptr->con_params.eff_max_rx_time;

        // Compute the new effective values
        llc_dl_compute_eff(link_id);

        // If the effective values have changed
        if ((llc_env_ptr->con_params.eff_max_rx_octets != old_eff_max_rx_octets)
                || (llc_env_ptr->con_params.eff_max_tx_octets != old_eff_max_tx_octets)
                || (llc_env_ptr->con_params.eff_max_rx_time   != old_eff_max_rx_time)
                || (llc_env_ptr->con_params.eff_max_tx_time   != old_eff_max_tx_time))
        {
            // Update the driver
            lld_con_data_len_update(link_id,
                                    llc_env_ptr->con_params.eff_max_tx_time,
                                    llc_env_ptr->con_params.eff_max_tx_octets,
                                    llc_env_ptr->con_params.eff_max_rx_time,
                                    llc_env_ptr->con_params.eff_max_rx_octets);

            // Send the data length change event
            llc_hci_dl_upd_info_send(link_id, CO_ERROR_NO_ERROR);
        }
    }
}

/**
 ****************************************************************************************
 * @brief Disables CTE requests/responses if needed.
 *
 * @param[in] link_id        Link identifier
 * @param[in] tx_phy         The Tx PHY after the update (@see enum le_phy_value)
 * @param[in] rx_phy         The Rx PHY after the update (@see enum le_phy_value)
 ****************************************************************************************
 */
__STATIC void llc_cte_chg_check(uint8_t link_id, uint8_t tx_phy, uint8_t rx_phy)
{
    // Check new PHY compatibility with CTE
#if BLE_CON_CTE_REQ
    if (rx_phy == PHY_CODED_VALUE)
    {
        llc_cte_req_dis(link_id);
    }
#endif // BLE_CON_CTE_REQ
#if BLE_CON_CTE_RSP
    if (tx_phy == PHY_CODED_VALUE)
    {
        llc_cte_rsp_dis(link_id);
    }
#endif // BLE_CON_CTE_RSP
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * LLCP Handlers
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles the reception of a PHY request
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent.
 * @param[in] pdu            LLCP PDU information received
 * @param[in] event_cnt      Event counter value when PDU has been received
 *
 * @return status code of handler:
 *    - CO_ERROR_NO_ERROR:               Nothing more to do
 *    - CO_ERROR_TERMINATED_MIC_FAILURE: Immediately disconnect the link
 *    - others:                          Send an LLCP_REJECT_IND or LLCP_REJECT_IND_EXT
 ****************************************************************************************
 */
uint8_t ll_phy_req_handler(uint8_t link_id, struct ll_phy_req *pdu, uint16_t event_cnt)
{
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint8_t status = CO_ERROR_NO_ERROR;

    // check that no remote procedure is on-going
    if (llc_proc_id_get(link_id, LLC_PROC_REMOTE) != LLC_PROC_NONE)
    {
        // unexpected message, reject it.
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }
    // If values are valid or not
    else if ((pdu->rx_phys == 0) || (pdu->tx_phys == 0))
    {
        status = CO_ERROR_INVALID_LMP_PARAM;
    }
    else
    {
        // Check if there is a possible procedure collision
        if (GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE))
        {
            status = llc_proc_collision_check(link_id, LLC_PROC_PHY_UPDATE);
        }

        // remote procedure can be started
        if (status == CO_ERROR_NO_ERROR)
        {
            ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
            // prepare local channel map update procedure
            struct llc_op_phy_upd_ind *phys_upd = KE_MSG_ALLOC(LLC_OP_PHY_UPD_IND, llc_id, llc_id, llc_op_phy_upd_ind);
            llc_proc_init(&phys_upd->proc, LLC_PROC_PHY_UPDATE, llc_rem_phy_upd_proc_err_cb);
            llc_proc_state_set(&phys_upd->proc, link_id, LLC_REM_PHY_UPD_START);

            // Swap peers TX/RX parameters, align to local point of view
            phys_upd->rx_phys       = pdu->tx_phys;
            phys_upd->tx_phys       = pdu->rx_phys;

            // Check if peer requires symmetric rates
            phys_upd->sym_rate = (pdu->rx_phys == pdu->tx_phys) && (one_bits[pdu->rx_phys] == 1) && (one_bits[pdu->tx_phys] == 1);

            phys_upd->m_to_s_phy = 0;
            phys_upd->s_to_m_phy = 0;
            phys_upd->phy_opt = 0;

            // store local procedure
            llc_proc_reg(link_id, LLC_PROC_REMOTE, &(phys_upd->proc));

            // start procedure execution
            llc_rem_phy_upd_proc_continue(link_id, LLC_REM_PHY_UPD_START, CO_ERROR_NO_ERROR);
        }
    }

    return (status);
}


/**
 ****************************************************************************************
 * @brief Handles the reception of a PHY response
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent.
 * @param[in] pdu            LLCP PDU information received
 * @param[in] event_cnt      Event counter value when PDU has been received
 *
 * @return status code of handler:
 *    - CO_ERROR_NO_ERROR:               Nothing more to do
 *    - CO_ERROR_TERMINATED_MIC_FAILURE: Immediately disconnect the link
 *    - others:                          Send an LLCP_REJECT_IND or LLCP_REJECT_IND_EXT
 ****************************************************************************************
 */
uint8_t ll_phy_rsp_handler(uint8_t link_id, struct ll_phy_rsp *pdu, uint16_t event_cnt)
{
    /// Gets the LLC environment dedicated to this link
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint8_t status = CO_ERROR_NO_ERROR;

    // If values are valid or not
    if ((pdu->rx_phys == 0) || (pdu->tx_phys == 0))
    {
        status = CO_ERROR_INVALID_LMP_PARAM;
    }

    // Only Master can receive this message
    if (!GETB(llc_env_ptr->link_info, LLC_INFO_MASTER_ROLE)
            // check that local procedure is the correct one
            || (llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_PHY_UPDATE))
    {
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }
    else
    {
        struct llc_op_phy_upd_ind *param = (struct llc_op_phy_upd_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);

        // Swap peers TX/RX parameters, align to local point of view
        param->tx_phys = param->tx_phys & pdu->rx_phys;
        param->rx_phys = param->rx_phys & pdu->tx_phys;

        // Check if peer requires symmetric rates
        param->sym_rate = (pdu->rx_phys == pdu->tx_phys) && (one_bits[pdu->rx_phys] == 1) && (one_bits[pdu->tx_phys] == 1);

        // continue feature execution
        llc_loc_phy_upd_proc_continue(link_id, LLC_LOC_WAIT_PHY_RSP, status);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of a PHY update indication
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent.
 * @param[in] pdu            LLCP PDU information received
 * @param[in] event_cnt      Event counter value when PDU has been received
 *
 * @return status code of handler:
 *    - CO_ERROR_NO_ERROR:               Nothing more to do
 *    - CO_ERROR_TERMINATED_MIC_FAILURE: Immediately disconnect the link
 *    - others:                          Send an LLCP_REJECT_IND or LLCP_REJECT_IND_EXT
 ****************************************************************************************
 */
uint8_t ll_phy_update_ind_handler(uint8_t link_id, struct ll_phy_update_ind *pdu, uint16_t event_cnt)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t supp_phy_msk = PHY_1MBPS_BIT;
    SETB(supp_phy_msk, PHY_2MBPS, BLE_PHY_2MBPS_SUPPORT);
    SETB(supp_phy_msk, PHY_CODED, BLE_PHY_CODED_SUPPORT);

    // This LLCP shall be issued by the master only
    if (GETB(llc_env[link_id]->link_info, LLC_INFO_MASTER_ROLE))
    {
        // unexpected message, reject it
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }
    else // The local device is peripheral
    {
        /*
        If the Peripheral receives an LL_PHY_UPDATE_IND where either PHY field
        specifies a PHY that the Peripheral does not support, has a bit set that is
        reserved for future use, or has more than one bit set, the Peripheral shall not
        change the PHY in that direction.
        */
        if ((pdu->m_to_s_phy & ~(supp_phy_msk)) || (NB_ONE_BITS(pdu->m_to_s_phy) > 1))
        {
            pdu->m_to_s_phy = 0;
        }

        if ((pdu->s_to_m_phy & ~(supp_phy_msk)) || (NB_ONE_BITS(pdu->s_to_m_phy) > 1))
        {
            pdu->s_to_m_phy = 0;
        }

        // If the new PHY is the same as the current one
        if ((pdu->m_to_s_phy != 0) && (pdu->m_to_s_phy == llc_env[link_id]->con_params.rx_phy))
        {
            pdu->m_to_s_phy = 0;
        }

        if ((pdu->s_to_m_phy != 0) && (pdu->s_to_m_phy == llc_env[link_id]->con_params.tx_phy))
        {
            pdu->s_to_m_phy = 0;
        }
    }

    // check that local procedure is the correct one
    if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_PHY_UPDATE)
    {
        struct llc_op_phy_upd_ind *param = (struct llc_op_phy_upd_ind *) llc_proc_get(link_id, LLC_PROC_LOCAL);

        param->m_to_s_phy = pdu->m_to_s_phy;
        param->s_to_m_phy = pdu->s_to_m_phy;
        param->instant    = pdu->instant;

        // continue feature execution
        llc_loc_phy_upd_proc_continue(link_id, LLC_LOC_WAIT_PHY_UPD_IND, status);
    }
    // or if slave check that remote procedure is the correct one
    else if (llc_proc_id_get(link_id, LLC_PROC_REMOTE) == LLC_PROC_PHY_UPDATE)
    {
        struct llc_op_phy_upd_ind *param = (struct llc_op_phy_upd_ind *) llc_proc_get(link_id, LLC_PROC_REMOTE);

        param->m_to_s_phy = pdu->m_to_s_phy;
        param->s_to_m_phy = pdu->s_to_m_phy;
        param->instant    = pdu->instant;

        // continue feature execution
        llc_rem_phy_upd_proc_continue(link_id, LLC_REM_WAIT_PHY_UPD_IND, status);
    }
    else
    {
        // unexpected message, reject it.
        status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
    }

    return (status);
}

/*
 ****************************************************************************************
 * HCI Handlers
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Handles the command HCI read PHY command.
 *
 * @param[in] link_id    Link Identifier
 * @param[in] param      Pointer to the parameters of the message.
 * @param[in] opcode     HCI message operation code.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_rd_phy_cmd_handler(uint8_t link_id, struct hci_le_rd_phy_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // allocate the status event message
    struct hci_le_rd_phy_cmd_cmp_evt *event ;

    // allocate the Command Complete event message
    event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_le_rd_phy_cmd_cmp_evt);

    // check if state is Free or in disconnected state
    if (!llc_is_disconnecting(link_id))
    {
        // Fill the PHY information
        struct llc_env_tag *llc_env_ptr = llc_env[link_id];
        event->rx_phy = llc_env_ptr->con_params.rx_phy;
        event->tx_phy = llc_env_ptr->con_params.tx_phy;
        status = CO_ERROR_NO_ERROR;
    } // else nothing to do

    event->status = status;
    event->conhdl = param->conhdl;

    // sends the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the command HCI set default PHY command.
 * The handler processes the command to do an update of the PHY for a dedicated link
 *
 * @startuml llc_hci_le_set_phy_cmd.png
 * participant HCI
 * participant LLC
 * HCI --> LLC : HCI_LE_SET_PHY_CMD
 * LLC -> LLC: hci_le_set_phy_cmd_handler()
 * activate LLC
 * alt link disconnected \nor slave \nor peer not support encryption
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(DISALLOWED)
 * else  Peer does not support feature
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(UNSUPPORTED_REMOTE_FEATURE)
 * else  Invalid parameters
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(INVALID_HCI_PARAM)
 * else  Local device does not support configured parameters
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(UNSUPPORTED)
 * else  Procedure already started
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(BUSY)
 * else  Procedure can be started
 *     note over LLC: Mark procedure started
 *     LLC --> LLC : LLC_OP_PHY_UPD_IND
 *     note right LLC #lightgreen: See __llc_op_phy_upd_ind_handler()__
 *     LLC --> HCI : HCI_CMD_STAT_EVENT(OK)
 * end
 * deactivate LLC
 * @enduml
 *
 * @param[in] link_id    Link Identifier
 * @param[in] param      Pointer to the parameters of the message.
 * @param[in] opcode     HCI message operation code.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_set_phy_cmd_handler(uint8_t link_id, struct hci_le_set_phy_cmd const *param, uint16_t opcode)
{
    // Command status
    uint8_t status = CO_ERROR_NO_ERROR;
    //Get environment pointer
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];
    uint8_t supp_phy_loc_msk = PHY_1MBPS_BIT;
    uint8_t supp_phy_rem_msk = PHY_1MBPS_BIT;
    SETB(supp_phy_loc_msk, PHY_2MBPS, BLE_PHY_2MBPS_SUPPORT);
    SETB(supp_phy_loc_msk, PHY_CODED, BLE_PHY_CODED_SUPPORT);
    SETB(supp_phy_rem_msk, PHY_2MBPS, llc_le_feature_check(link_id, BLE_FEAT_2M_PHY));
    SETB(supp_phy_rem_msk, PHY_CODED, llc_le_feature_check(link_id, BLE_FEAT_CODED_PHY));

    // check if state is Free or in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        status = CO_ERROR_COMMAND_DISALLOWED;
    }
    // ensure that host not currently processing request
    else if (GETB(llc_env_ptr->flow_ctrl, LLC_HCI_SET_PHY_REQ))
    {
        status = CO_ERROR_CONTROLLER_BUSY;
    }
    // Check PHY preference and value in the range.
    else if ((!(param->all_phys & ALL_PHYS_RX_NO_PREF) && (param->rx_phys == 0))
             || (!(param->all_phys & ALL_PHYS_TX_NO_PREF) && (param->tx_phys == 0)))
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    // Phy preference and value supported
    else if ((!(param->all_phys & ALL_PHYS_RX_NO_PREF) && ((param->rx_phys & supp_phy_loc_msk) != param->rx_phys))
             || (!(param->all_phys & ALL_PHYS_TX_NO_PREF) && ((param->tx_phys & supp_phy_loc_msk) != param->tx_phys))
             || (param->phy_opt > PHY_OPT_S8_LE_CODED_TX_PREF)
             || (param->all_phys > (ALL_PHYS_TX_NO_PREF | ALL_PHYS_RX_NO_PREF)))
    {
        status = CO_ERROR_UNSUPPORTED;
    }
    // Ensure that at least one preferred PHY is supported by the peer
    else if ((!(param->all_phys & ALL_PHYS_RX_NO_PREF) && ((param->rx_phys & supp_phy_rem_msk) == 0))
             || (!(param->all_phys & ALL_PHYS_TX_NO_PREF) && ((param->tx_phys & supp_phy_rem_msk) == 0)))
    {
        status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
    }
    else
    {
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
        // start PHY negotiation
        struct llc_op_phy_upd_ind *phy_update = KE_MSG_ALLOC(LLC_OP_PHY_UPD_IND, llc_id, llc_id, llc_op_phy_upd_ind);

        llc_proc_init(&phy_update->proc, LLC_PROC_PHY_UPDATE, llc_loc_phy_upd_proc_err_cb);
        llc_proc_state_set(&phy_update->proc, link_id, LLC_LOC_PHY_UPD_START);
        phy_update->rx_phys        = (param->all_phys & ALL_PHYS_RX_NO_PREF) ? (supp_phy_loc_msk & supp_phy_rem_msk) : (param->rx_phys & supp_phy_rem_msk);
        phy_update->tx_phys        = (param->all_phys & ALL_PHYS_TX_NO_PREF) ? (supp_phy_loc_msk & supp_phy_rem_msk) : (param->tx_phys & supp_phy_rem_msk);
        phy_update->phy_opt        = param->phy_opt;
        phy_update->host_req       = true;

        phy_update->m_to_s_phy = 0;
        phy_update->s_to_m_phy = 0;

        ke_msg_send(phy_update);

        SETB(llc_env_ptr->flow_ctrl, LLC_HCI_SET_PHY_REQ, true);
    }

    // Send the command status event
    llc_cmd_stat_send(link_id, opcode, status);

    return (KE_MSG_CONSUMED);
}


/*
 ****************************************************************************************
 * Local Messages Handlers
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles the Phys update procedure indication message.
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles the Channel Map Update Local procedure indication message.
 *
 * @startuml llc_op_phy_upd_ind_handler.png
 * participant LLC
 *  --> LLC : LLC_OP_PHY_UPD_IND
 * LLC -> LLC: llc_op_phy_upd_ind_handler()
 * activate LLC
 * hnote over LLC : LOC_PROC = Busy
 * LLC -> LLC: llc_loc_phy_upd_proc_continue(START)
 * activate LLC
 * note right LLC #lightgreen: See __llc_loc_phy_upd_proc_continue()__
 * deactivate LLC
 * deactivate LLC
 * @enduml
 *
 ****************************************************************************************
 */
int llc_op_phy_upd_ind_handler(ke_msg_id_t const msgid, struct llc_op_phy_upd_ind *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Current message status
    int msg_status = KE_MSG_CONSUMED;
    uint8_t link_id = KE_IDX_GET(dest_id);
    struct llc_env_tag *llc_env_ptr = llc_env[link_id];

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // Abort request immediately with disconnection reason
        if (param->host_req)
        {
            llc_hci_le_phy_upd_cmp_evt_send(link_id, llc_env_ptr->disc_reason, llc_env_ptr->con_params.tx_phy, llc_env_ptr->con_params.rx_phy);
        }
    }
    // check if another local procedure is on-going
    else if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_NONE)
    {
        // process this message later
        msg_status = KE_MSG_SAVED;
    }
    else
    {
        // check if procedure with instant on-going
        if (GETB(llc_env_ptr->link_info, LLC_INFO_INSTANT_PROC) && (llc_proc_state_get(&param->proc) == LLC_LOC_PHY_UPD_START))
        {
            // process this message later
            msg_status = KE_MSG_SAVED;
        }
#if (BT_53) && (BLE_CENTRAL)
        // check to suspend subrating for this procedure if needed
        else if (llc_subrate_suspend(link_id))
        {
            // process this message later
            msg_status = KE_MSG_SAVED;
        }
#endif // (BT_53) && (BLE_CENTRAL)
        else
        {
            msg_status = KE_MSG_NO_FREE;

            ASSERT_INFO((llc_proc_state_get(&param->proc) == LLC_LOC_PHY_UPD_START) || (llc_proc_state_get(&param->proc) == LLC_LOC_PHY_UPD_FORCE),
                        link_id, llc_proc_state_get(&param->proc));

            // store local procedure
            llc_proc_reg(link_id, LLC_PROC_LOCAL, &(param->proc));
            // execute local procedure
            llc_loc_phy_upd_proc_continue(link_id, llc_proc_state_get(&param->proc), CO_ERROR_NO_ERROR);
        }
    }
    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles the Channel Map Update Local procedure indication message.
 ****************************************************************************************
 */
int lld_phy_upd_cfm_handler(ke_msg_id_t const msgid, void *param,
                            ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t link_id = KE_IDX_GET(dest_id);

    // PHY has been correctly updated
    if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_PHY_UPDATE)
    {
        llc_loc_phy_upd_proc_continue(link_id, LLC_LOC_WAIT_PHY_UPD_INSTANT, CO_ERROR_NO_ERROR);
    }

    else if (llc_proc_id_get(link_id, LLC_PROC_REMOTE) == LLC_PROC_PHY_UPDATE)
    {
        llc_rem_phy_upd_proc_continue(link_id, LLC_REM_WAIT_PHY_UPD_INSTANT, CO_ERROR_NO_ERROR);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * Start PHY update procedure
 *
 * @param[in] link_id Link identifier
 ****************************************************************************************
 */
void llc_phy_upd_proc_start(uint8_t link_id)
{
    ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);

    // start PHY negotiation
    struct llc_op_phy_upd_ind *phy_update = KE_MSG_ALLOC(LLC_OP_PHY_UPD_IND, llc_id, llc_id, llc_op_phy_upd_ind);

    llc_proc_init(&phy_update->proc, LLC_PROC_PHY_UPDATE, llc_loc_phy_upd_proc_err_cb);
    llc_proc_state_set(&phy_update->proc, link_id, LLC_LOC_PHY_UPD_START);
    phy_update->rx_phys        = llc_env[link_id]->rx_phys;
    phy_update->tx_phys        = llc_env[link_id]->tx_phys;
    phy_update->phy_opt        = 0;
    phy_update->host_req       = false;

    phy_update->m_to_s_phy = 0;
    phy_update->s_to_m_phy = 0;

    ke_msg_send(phy_update);
}

#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/// @} LLC_PHY_UPD
