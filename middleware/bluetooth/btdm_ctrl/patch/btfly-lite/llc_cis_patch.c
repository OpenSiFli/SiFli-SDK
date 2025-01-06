/**
 ****************************************************************************************
 *
 * @file llc_cis.c
 *
 * @brief Connected Isochronous Stream negotiation
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup LLC_CIS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_CIS)

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "compiler.h"    // __ARRAY_EMPTY and __STATIC define
#include "co_bt.h"       // BT Standard defines (HCI, LLCP, Error codes)
#include "co_math.h"     // For co_min
#include "co_utils.h"    // For device pdu preparation
#include "ble_util.h"    // BLE utility functions
#include "ke_msg.h"      // Kernel message
#include "ke_timer.h"    // Kernel timers
#include "sch_arb.h"     // Scheduling Arbiter
#include "sch_prog.h"    // Scheduling Programmer
#include "sch_slice.h"   // Scheduling Slicer
#include "llc_int.h"     // Internal LLC API
#include "llc_llcp.h"    // Internal LLCP API
#include "llm.h"         // LLM API
#include "lld.h"         // LLD API
#include "lli.h"         // LLI API
#include "llc.h"         // LLC API
#include "sch_plan.h"    // Scheduling planner
#include "hci.h"         // For HCI handler
#include "btdm_patch.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// CIS Creation structure definition
/*@TRACE*/
struct llc_op_cis_create_ind
{
    /// Procedure information
    llc_procedure_t proc;

    /// Activity ID
    uint8_t         act_id;
    /// Local or remote procedure
    bool            local;
    /// Role
    uint8_t         role;

    /// Generated access address
    uint32_t        access_addr;

    /// Time between the start of an ACL Anchor Point associated with the Instant and the first CIS Anchor Point.
    /// In microseconds
    uint32_t        cis_offset;

    /// Minimum time between the start of an ACL Anchor Point associated with Event_Count_Ref and the start of
    /// the CIS Channel Anchor Point the first CIS Channel Anchor Point. (Local value in us)
    uint32_t        cis_offset_min;
    /// Maximum time between the start of an ACL Anchor Point associated with Event_Count_Ref and the start of
    /// the CIS Channel Anchor Point (Local value in us)
    uint32_t        cis_offset_max;
    /// Air Time required by the negotiation (Stream for slave, Group for master) (in us)
    uint32_t        air_time;

    /// Event counter of the ACL event from which the Offset_Min and the Offset_Max are referenced
    /// or when the CIS connection will be created
    uint16_t        conn_event_cnt;
    /// Activity offset (in half-slots)
    uint16_t        act_offset;
    /// Activity offset, fractional part (in half-us)
    uint16_t        act_offset_hus;
    /// Activity margin that can be used during negotiation (in half-us)
    uint16_t        act_margin_hus;
};

/// CIS Stop structure definition
/*@TRACE*/
struct llc_op_cis_stop_ind
{
    /// Procedure information
    llc_procedure_t proc;

    /// Activity ID
    uint8_t         act_id;
    /// Stop reason
    uint8_t         reason;
};

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// States for local and remote CIS create procedures
/*@TRACE*/
enum llc_cis_create_state
{
    // Local states --------------------------------------------------

    /// Local start CIS creation procedure
    LLC_LOC_CIS_CREATE_START,
    /// Wait for reception of LL_CIS_RSP LLCP
    LLC_LOC_CIS_WAIT_CREATE_RSP,
    /// Error occurs during CIS creation update
    LLC_LOC_CIS_CREATE_ERROR,
    /// Cancel local CIS procedure
    LLC_LOC_CIS_CANCEL,

    // Remote states -------------------------------------------------

    /// Remote start CIS creation procedure
    LLC_REM_CIS_CREATE_START,
    /// Wait for host status
    LLC_REM_CIS_WAIT_HOST_STATUS,
    /// Wait for reception of LL_CIS_UPDATE_IND LLCP
    LLC_REM_CIS_WAIT_UPDATE_IND,
    /// Host reject the peers request
    LLC_REM_CIS_HOST_REJECT,
};

/// States for local and remote CIS stop procedures
/*@TRACE*/
enum llc_cis_stop_state
{
    // Local states --------------------------------------------------

    /// Local start CIS stop procedure
    LLC_LOC_CIS_STOP_START,
    /// Wait for acknowledgment of LL_CIS_TERMINATE_IND LLCP
    LLC_LOC_CIS_WAIT_TERMINATE_ACK,
};


#if !(BLE_CIS_IN_ROM)
/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if (BLE_CENTRAL)
    __STATIC void llc_cis_loc_create_proc_continue(uint8_t link_id, uint8_t state, uint8_t status);
#endif // (BLE_CENTRAL)
__STATIC void llc_cis_loc_stop_proc_continue(uint8_t link_id, uint8_t state, uint8_t status);

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * Translate a CIS offset range from one connection event to another one
 *
 * @param[in] ref_con_evt_cnt     Reference connection event count
 * @param[in] new_con_evt_cnt     New connection event count
 * @param[in] con_intv            Connection interval (in 1.25ms)
 * @param[in] iso_intv            Isochronous interval (in 1.25ms)
 * @param[in|out] cis_offset_min  CIS offset min (in us)
 * @param[in|out] cis_offset_max  CIS offset max, NULL if not used (in us)
 ****************************************************************************************
 */
__STATIC void llc_cis_offset_translate(uint16_t ref_con_evt_cnt, uint16_t new_con_evt_cnt, uint16_t con_intv, uint16_t iso_intv, uint32_t *cis_offset_min, uint32_t *cis_offset_max)
{
    // ISO interval in us
    uint32_t iso_interval_us = iso_intv * 2 * SLOT_SIZE;
    uint32_t distance_us;

    // Direction of the translation
    if (ref_con_evt_cnt >= new_con_evt_cnt)
    {
        distance_us = CO_MOD(((ref_con_evt_cnt - new_con_evt_cnt) * (con_intv << 1)), (iso_intv << 1)) * SLOT_SIZE;
    }
    else
    {
        distance_us = iso_interval_us - CO_MOD(((new_con_evt_cnt - ref_con_evt_cnt) * (con_intv << 1)), (iso_intv << 1)) * SLOT_SIZE;
    }

    // Translate and rebase
    *cis_offset_min = CO_MOD(*cis_offset_min + distance_us, iso_interval_us);

    if (cis_offset_max != NULL)
    {
        // Translate and rebase
        *cis_offset_max = CO_MOD(*cis_offset_max + distance_us, iso_interval_us);

        // If CIS offset max is rebased below CIS offset min, move to next ISO interval
        if (*cis_offset_max < *cis_offset_min)
        {
            *cis_offset_max += iso_interval_us;
        }
    }
}

/**
 ****************************************************************************************
 * Handles reception of LL_CIS_TERMINATE baseband acknowledgment
 *
 * @param[in] link_id Link identifier
 * @param[in] op_code Operation code acknowledged
 ****************************************************************************************
 */
__STATIC void llc_cis_terminate_ind_ack(uint8_t link_id, uint8_t op_code)
{
    if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) == LLC_PROC_CIS_STOP)
    {
        // Continue the stop procedure
        llc_cis_loc_stop_proc_continue(link_id, LLC_LOC_CIS_WAIT_TERMINATE_ACK, CO_ERROR_NO_ERROR);
    }
}

#if (BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief Sends the LL_CIS_REQ PDU
 *
 * @param[in] link_id       Identifier of link on which PDU has to be sent
 * @param[in] p_create_ind  Negotiated parameters
 * @param[in] p_cig_param   Pointer to the structure containing the parameters for the CIG.
 * @param[in] p_cis_param   Pointer to the structure containing the parameters for the CIS
 *                          to be created.
 ****************************************************************************************
 */
__STATIC void llc_cis_req_pdu_send(uint8_t link_id, struct llc_op_cis_create_ind *p_create_ind,
                                   struct lli_cig_param *p_cig_param, struct lli_cis_param *p_cis_param)
{
    // Get PDU structure
    struct ll_cis_req pdu;

    // Fill the PDU
    pdu.op_code           = LL_CIS_REQ_OPCODE;
    pdu.cig_id            = p_cis_param->cig_id;
    pdu.cis_id            = p_cis_param->cis_id;
    pdu.phy_m2s           = co_rate_to_phy_mask[p_cis_param->tx_rate];
    pdu.phy_s2m           = co_rate_to_phy_mask[p_cis_param->rx_rate];
    pdu.max_pdu_m2s       = p_cis_param->tx_max_pdu;
    pdu.max_pdu_s2m       = p_cis_param->rx_max_pdu;
    pdu.nse               = p_cis_param->nse;
    pdu.sub_interval      = (pdu.nse != 1) ? p_cis_param->sub_interval : 0;
    pdu.ft_m2s            = p_cig_param->tx_ft;
    pdu.ft_s2m            = p_cig_param->rx_ft;
    pdu.iso_interval      = p_cig_param->iso_interval;
    pdu.cis_offset_min    = p_create_ind->cis_offset_min;
    pdu.cis_offset_max    = p_create_ind->cis_offset_max;
    pdu.conn_event_cnt    = p_create_ind->conn_event_cnt;

    // Pack SDU parameters
    pdu.max_sdu_m2s       = p_cis_param->tx_max_sdu;
    pdu.max_sdu_s2m       = p_cis_param->rx_max_sdu;

    SETB(pdu.max_sdu_m2s, BLE_CIS_FRAME_MODE, p_cig_param->framing);

    pdu.sdu_interval_m2s  = p_cig_param->tx_sdu_interval;
    pdu.sdu_interval_s2m  = p_cig_param->rx_sdu_interval;

    // Set burst number
    pdu.bn                = 0;
    SETF(pdu.bn, LLCP_CIS_BN_M2S, p_cis_param->tx_bn);
    SETF(pdu.bn, LLCP_CIS_BN_S2M, p_cis_param->rx_bn);

    // Send the PDU
    llc_llcp_send(link_id, (union llcp_pdu *)&pdu, NULL);
}

/**
 ****************************************************************************************
 * @brief Sends the LL_CIS_UPDATE_IND PDU
 *
 * @param[in] link_id              Identifier of link on which PDU has to be sent
 * @param[in] p_create_ind         Pointer to the procedure structure containing the negotiated
 *                                 parameters
 * @param[in] cis_offset_in_cig    CIS offset in the CIG (in us)
 ****************************************************************************************
 */
__STATIC void llc_cis_ind_pdu_send(uint8_t link_id, struct llc_op_cis_create_ind *p_create_ind, uint32_t cis_offset_in_cig)
{
    // Get PDU structure
    struct ll_cis_ind pdu;

    // Fill the PDU
    pdu.op_code             = LL_CIS_IND_OPCODE;
    pdu.aa                  = p_create_ind->access_addr;
    pdu.conn_event_cnt      = p_create_ind->conn_event_cnt;
    pdu.cis_sync_delay      = p_create_ind->air_time - cis_offset_in_cig;
    pdu.cis_offset          = p_create_ind->cis_offset;
    pdu.cig_sync_delay      = p_create_ind->air_time;

    // Send the PDU
    llc_llcp_send(link_id, (union llcp_pdu *)&pdu, NULL);
}
#endif //(BLE_CENTRAL)

#if (BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Sends the LL_CIS_RSP PDU
 *
 * @param[in] link_id       Identifier of link on which PDU has to be sent
 * @param[in] p_create_ind  Pointer to the procedure structure containing the negotiated
 *                          parameters
 ****************************************************************************************
 */
__STATIC void llc_cis_rsp_pdu_send(uint8_t link_id, struct llc_op_cis_create_ind *p_create_ind)
{
    // Get PDU structure
    struct ll_cis_rsp pdu;

    // Fill the PDU
    pdu.op_code         = LL_CIS_RSP_OPCODE;
    pdu.cis_offset_min  = p_create_ind->cis_offset_min;
    pdu.cis_offset_max  = p_create_ind->cis_offset_max;
    pdu.conn_event_cnt  = p_create_ind->conn_event_cnt;

    // Send the PDU
    llc_llcp_send(link_id, (union llcp_pdu *)&pdu, NULL);
}
#endif //(BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Sends the LL_CIS_TERMINATE_IND PDU
 *
 * @param[in] link_id       Identifier of link on which PDU has to be sent
 * @param[in] p_cis_param   Pointer to the structure containing the parameters for the CIS
 *                          Channel.
 * @param[in] reason        Terminate reason
 ****************************************************************************************
 */
__STATIC void llc_cis_terminate_ind_pdu_send(uint8_t link_id, struct lli_cis_param *p_cis_param, uint8_t reason)
{
    // Get PDU structure
    struct ll_cis_terminate_ind pdu;

    // Fill the PDU
    pdu.op_code       = LL_CIS_TERMINATE_IND_OPCODE;
    pdu.cig_id        = p_cis_param->cig_id;
    pdu.cis_id        = p_cis_param->cis_id;
    pdu.err_code      = reason;

    // Send the PDU
    llc_llcp_send(link_id, (union llcp_pdu *)&pdu, &llc_cis_terminate_ind_ack);
}

/**
 ****************************************************************************************
 * @brief Compute distance from connection event to ISO event
 *
 * @param[in]     link_id             Link identifier
 * @param[in]     con_evt_time_hs     Time of the reference connection event (in half-slot)
 * @param[in|out] con_evt_cnt         Input: Reference connection event counter, Output: new reference connection event counter
 * @param[in]     iso_intv            ISO interval (in multiple of 1.25ms)
 * @param[in]     act_offset          Activity offset (in half-slot)
 *
 * @return Distance from connection event to ISO event (in half-us)
 ****************************************************************************************
 */
__STATIC uint32_t llc_cis_con_evt_dist_compute(uint8_t link_id, uint32_t con_evt_time_hs, uint16_t *con_evt_cnt, uint16_t iso_intv, uint16_t act_offset, bool inst_sel)
{
    struct llc_env_tag *p_llc_env = llc_env[link_id];
    uint32_t dist_con_evt_to_iso;
    uint16_t init_con_evt_cnt = *con_evt_cnt;

    if (inst_sel)
    {
        // Rules for Procedures with Instants shall apply to the connEventCount field
        *con_evt_cnt = llc_inst_select(link_id);
    }
    else
    {
        // Select a reference connection event in the future
        *con_evt_cnt = lld_con_event_counter_get(link_id) + LLC_PROC_CIS_NEGO_INSTANT_DELAY;
    }

    // Compute connection event time
    con_evt_time_hs = CLK_ADD_2(con_evt_time_hs, BLE_UTIL_EVT_CNT_DIFF(*con_evt_cnt, init_con_evt_cnt) * (p_llc_env->con_params.interval << 2));

    // Compute distance from connection event to ISO event (in half-slot)
    dist_con_evt_to_iso = CO_MOD(CLK_SUB(con_evt_time_hs, act_offset), (iso_intv << 2));
    if (dist_con_evt_to_iso > 0)
    {
        dist_con_evt_to_iso = (iso_intv << 2) - dist_con_evt_to_iso;
    }

    // If the distance is higher than connection interval
    if (dist_con_evt_to_iso >= (uint32_t)(p_llc_env->con_params.interval << 2))
    {
        // Select another reference connection event, the closest before the ISO event
        uint16_t diff_con_cnt = dist_con_evt_to_iso / ((p_llc_env->con_params.interval << 2));
        *con_evt_cnt += diff_con_cnt;

        // Update the distance
        dist_con_evt_to_iso -= diff_con_cnt * (p_llc_env->con_params.interval << 2);
    }

    // Convert distance to half-us
    dist_con_evt_to_iso = dist_con_evt_to_iso * HALF_SLOT_SIZE;

    return dist_con_evt_to_iso;
}


#if (BLE_CENTRAL)

/**
 ****************************************************************************************
 * @brief Continue execution of local create procedure
 *
 * @param[in] link_id   Link identifier
 * @param[in] state     Expected state of the procedure
 * @param[in] status    Status of the operation
 ****************************************************************************************
 */
__STATIC void llc_cis_loc_create_proc_continue(uint8_t link_id, uint8_t state, uint8_t status)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_CIS_LOC_CREATE_PROC_CONTINUE_C_FUN_BIT, link_id, state, status);
    struct llc_env_tag *p_llc_env = llc_env[link_id];
    // Indicate if procedure if finished
    bool finished = false;
    // Get the procedure environment
    struct llc_op_cis_create_ind *p_create_ind = (struct llc_op_cis_create_ind *)llc_proc_get(link_id, LLC_PROC_LOCAL);

    // Check that current procedure state equals to expected state given in parameter
    if (llc_proc_state_get(&p_create_ind->proc) != state)
    {
        ASSERT_WARN(0, llc_proc_state_get(&p_create_ind->proc), state);

        // Stop the procedure
        finished = true;
        // Status
        status = CO_ERROR_UNDEFINED;
    }
    else
    {
        // CIS channel parameters
        struct lli_cig_param *p_cig_param;
        struct lli_cis_param *p_cis_param;

        if (status != CO_ERROR_NO_ERROR)
        {
            // Set error state
            llc_proc_state_set(&p_create_ind->proc, link_id, LLC_LOC_CIS_CREATE_ERROR);
        }

        // load CIS parameters
        lli_ci_param_get(p_create_ind->act_id, &p_cig_param, &p_cis_param);

        // Perform next operation based on current procedure state
        switch (llc_proc_state_get(&p_create_ind->proc))
        {
        case LLC_LOC_CIS_CREATE_START:
        {
            uint32_t con_evt_time_hs;
            uint16_t con_evt_cnt;
            uint32_t dist_con_evt_to_iso;
            uint32_t cis_offset_max = (p_llc_env->con_params.interval * 2 * SLOT_SIZE - p_create_ind->air_time) - 1;

            // Get current connection event time
            lld_con_time_get(link_id, &con_evt_cnt, &con_evt_time_hs, NULL);

            // Compute distance from the connection event to ISO event
            dist_con_evt_to_iso = llc_cis_con_evt_dist_compute(link_id, con_evt_time_hs, &con_evt_cnt, p_cig_param->iso_interval, p_create_ind->act_offset, false);

            // Save the new reference connection event counter
            p_create_ind->conn_event_cnt = con_evt_cnt;

            // Compute distance from connection event to ISO event (in us)
            p_create_ind->cis_offset_min = (uint32_t)((dist_con_evt_to_iso + p_create_ind->act_offset_hus) >> 1) + p_cis_param->cis_offset_in_cig;
            p_create_ind->cis_offset_max = p_create_ind->cis_offset_min + (p_create_ind->act_margin_hus >> 1);

            // Ensure CIS offset is greater than minimum value
            if (p_create_ind->cis_offset_min < BLE_CIS_MIN_OFFSET)
            {
                // Move to the next ISO interval
                p_create_ind->cis_offset_min += p_cig_param->iso_interval * 2 * SLOT_SIZE;
                p_create_ind->cis_offset_max += p_cig_param->iso_interval * 2 * SLOT_SIZE;
            }

            // Ensure CIS offset is lower than maximum value
            p_create_ind->cis_offset_max = co_min(p_create_ind->cis_offset_max, cis_offset_max);
            p_create_ind->cis_offset_min = co_min(p_create_ind->cis_offset_min, cis_offset_max);

            // Send LL_CIS_REQ PDU
            llc_cis_req_pdu_send(link_id, p_create_ind, p_cig_param, p_cis_param);

            // Start the LLCP Response TO for reception of LL_CIS_RSP
            llc_proc_timer_set(link_id, LLC_PROC_LOCAL, true);

            // Wait for LL_CIS_RSP
            llc_proc_state_set(&p_create_ind->proc, link_id, LLC_LOC_CIS_WAIT_CREATE_RSP);
        }
        break;

        case LLC_LOC_CIS_WAIT_CREATE_RSP:
        {
            uint32_t con_evt_time_hs, iso_evt_time_hs;
            uint32_t iso_evt_time_hus;
            uint16_t con_evt_time_hus;
            uint16_t con_evt_cnt;
            uint16_t act_offset;
            uint16_t act_offset_hus;
            uint16_t act_margin_hus;
            uint32_t dist_con_evt_to_iso;

            // Stop the LLCP Response TO
            llc_proc_timer_set(link_id, LLC_PROC_LOCAL, false);

            // Get current connection event time
            lld_con_time_get(link_id, &con_evt_cnt, &con_evt_time_hs, &con_evt_time_hus);

            // Compute connection event time
            con_evt_time_hs = CLK_ADD_2(con_evt_time_hs, BLE_UTIL_EVT_CNT_DIFF(p_create_ind->conn_event_cnt, con_evt_cnt) * (p_llc_env->con_params.interval << 2));

            // Compute first ISO event time
            iso_evt_time_hus = con_evt_time_hus + 2 * p_create_ind->cis_offset_min - 2 * p_cis_param->cis_offset_in_cig;
            iso_evt_time_hs = con_evt_time_hs + iso_evt_time_hus / HALF_SLOT_SIZE;
            iso_evt_time_hus = CO_MOD(iso_evt_time_hus, HALF_SLOT_SIZE);

            // Compute the offset, based on ISO interval and local clock
            act_offset = CO_MOD(iso_evt_time_hs, p_cig_param->iso_interval << 2);
            act_offset_hus = iso_evt_time_hus;
            act_margin_hus = 2 * (p_create_ind->cis_offset_max - p_create_ind->cis_offset_min);

            // Select a CIS anchor point, the most far possible from the connection event, i.e. the upper limit of the negotiated window
            p_create_ind->act_offset     = act_offset + ((act_offset_hus + act_margin_hus) / HALF_SLOT_SIZE);
            p_create_ind->act_offset_hus = CO_MOD(act_offset_hus + act_margin_hus, HALF_SLOT_SIZE);
            p_create_ind->act_margin_hus = 0;

            // If possible, select a half-slot boundary
            if ((act_offset_hus == 0) || ((act_offset_hus + act_margin_hus) > HALF_SLOT_SIZE))
            {
                p_create_ind->act_offset_hus = 0;
            }

            // Compute distance from the connection event to ISO event
            con_evt_cnt = p_create_ind->conn_event_cnt;
            dist_con_evt_to_iso = llc_cis_con_evt_dist_compute(link_id, con_evt_time_hs, &con_evt_cnt, p_cig_param->iso_interval, p_create_ind->act_offset, true);

            // Save the new reference connection event counter
            p_create_ind->conn_event_cnt = con_evt_cnt;

            // add the internal CIS offset
            p_create_ind->cis_offset = ((dist_con_evt_to_iso + p_create_ind->act_offset_hus) >> 1) + p_cis_param->cis_offset_in_cig;

            // Generate access address of CIS channel to create
            lld_cis_aa_gen(link_id, &p_create_ind->access_addr);

            // Send LL_CIS_UPDATE_IND PDU
            llc_cis_ind_pdu_send(link_id, p_create_ind, p_cis_param->cis_offset_in_cig);

            finished = true;
        }
        break;

        case LLC_LOC_CIS_CREATE_ERROR:
        case LLC_LOC_CIS_CANCEL:
        {
            finished = true;
        }
        break;

        default:
        {
            ASSERT_INFO(0, link_id, llc_proc_state_get(&p_create_ind->proc));
        }
        break;
        }
    }

    if (finished)
    {
        // Stop the LLCP Response TO
        llc_proc_timer_set(link_id, LLC_PROC_LOCAL, false);

        // If not a cancellation
        if (state != LLC_LOC_CIS_CANCEL)
        {
            // If CIS creation succeeds
            if (status == CO_ERROR_NO_ERROR)
            {
                // Inform LLI CIS module that creation procedure is finished
                lli_cis_create_end(p_create_ind->act_id, status, p_create_ind->access_addr,
                                   p_create_ind->conn_event_cnt, p_create_ind->cis_offset,
                                   p_create_ind->act_offset, p_create_ind->act_offset_hus,
                                   p_create_ind->air_time, (GETB(llc_env[link_id]->link_info, LLC_INFO_LINK_ENCRYPTED)));
            }
            else
            {
                // Inform LLI CIS module that creation procedure is finished
                lli_cis_create_end(p_create_ind->act_id, status, 0, 0, 0, 0, 0, 0, 0);
            }
        }

        // Unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_LOCAL);
    }
}

/**
 ****************************************************************************************
 * @brief Handles the CIS creation error (UNKNOWN/REJECT).
 *
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void llc_cis_loc_create_err_cb(uint8_t link_id, uint8_t error_type, void *p_param)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    switch (error_type)
    {
    case LLC_ERR_DISCONNECT:
    {
        status = *((uint8_t *)p_param);
    }
    break;

    case LLC_ERR_LLCP_UNKNOWN_RSP:
    {
        status = CO_ERROR_UNKNOWN_LMP_PDU;
    }
    break;

    case LLC_ERR_LLCP_REJECT_IND:
    {
        struct ll_reject_ind *reject = (struct ll_reject_ind *) p_param;
        status =  reject->err_code;
    }
    break;
    case LLC_ERR_LLCP_REJECT_EXT_IND:
    {
        struct ll_reject_ext_ind *reject_ext = (struct ll_reject_ext_ind *) p_param;
        if (reject_ext->rej_op_code == LL_CIS_REQ_OPCODE)
        {
            status =  reject_ext->err_code;
        }
    }
    break;

    default:
    {
        // Nothing to do, ignore
    } break;
    }

    if (status != CO_ERROR_NO_ERROR)
    {
        // Get the procedure environment
        struct llc_op_cis_create_ind *p_create_ind
            = (struct llc_op_cis_create_ind *)llc_proc_get(link_id, LLC_PROC_LOCAL);

        if ((status == CO_ERROR_UNKNOWN_LMP_PDU) || (status == CO_ERROR_UNSUPPORTED_REMOTE_FEATURE))
        {
            // Keep in mind that feature is not supported by peer device
            llc_le_feature_set(link_id, BLE_FEAT_CIS_SLAVE, false);
            llc_le_feature_set(link_id, BLE_FEAT_CIS_HOST_SUPPORT, false);
        }

        // Update state
        llc_proc_state_set(&p_create_ind->proc, link_id, LLC_LOC_CIS_CREATE_ERROR);

        // Finish the procedure
        llc_cis_loc_create_proc_continue(link_id, LLC_LOC_CIS_CREATE_ERROR, status);
    }
}
#endif //(BLE_CENTRAL)

#if (BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Continue execution of remote create procedure
 *
 * @param[in] link_id Link identifier
 * @param[in] state   Expected state of the procedure
 * @param[in] status  Status of the operation
 ****************************************************************************************
 */
__STATIC void llc_cis_rem_create_proc_continue(uint8_t link_id, uint8_t state, uint8_t status)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_CIS_REM_CREATE_PROC_CONTINUE_C_FUN_BIT, link_id, state, status);
    // Indicate if procedure if finished
    bool finished = false;
    // Gets the procedure environment
    struct llc_op_cis_create_ind *p_create_ind
        = (struct llc_op_cis_create_ind *)llc_proc_get(link_id, LLC_PROC_REMOTE);

    // Check that current procedure state equals to expected state given in parameter
    if (llc_proc_state_get(&p_create_ind->proc) != state)
    {
        ASSERT_WARN(0, llc_proc_state_get(&p_create_ind->proc), state);

        // Status
        status = CO_ERROR_UNDEFINED;
    }

    // check if an error occurs
    if (status != CO_ERROR_NO_ERROR)
    {
        finished = true;
    }
    else
    {
        // Perform next operation based on current procedure state
        switch (llc_proc_state_get(&p_create_ind->proc))
        {
        case LLC_REM_CIS_CREATE_START:
        {
            // Wait for host accept or reject status
            llc_proc_state_set(&p_create_ind->proc, link_id, LLC_REM_CIS_WAIT_HOST_STATUS);

            // Start the CIS connection accept timeout timer
            ke_timer_set(LLC_CIS_ACCEPT_TO, KE_BUILD_ID(TASK_LLC, link_id), 10 * co_slot_to_duration(hci_con_accept_to_get()));
        }
        break;

        case LLC_REM_CIS_WAIT_HOST_STATUS:
        {
            struct llc_env_tag *p_llc_env = llc_env[link_id];
            struct lli_cig_param *p_cig_param;
            uint32_t con_evt_time_hs, iso_evt_time_hs;
            uint32_t iso_evt_time_hus;
            uint16_t con_evt_time_hus;
            uint16_t con_evt_cnt;
            uint32_t dist_con_evt_to_iso;
            uint16_t act_offset;
            uint32_t act_offset_hus;
            uint32_t act_margin_hus;

            // load CIS parameters
            lli_ci_param_get(p_create_ind->act_id, &p_cig_param, NULL);

            // Get current connection event time
            lld_con_time_get(link_id, &con_evt_cnt, &con_evt_time_hs, &con_evt_time_hus);

            // Compute connection event time
            con_evt_time_hs = CLK_ADD_2(con_evt_time_hs, BLE_UTIL_EVT_CNT_DIFF(p_create_ind->conn_event_cnt, con_evt_cnt) * (p_llc_env->con_params.interval << 2));

            // Compute ISO event time at the lower limit
            iso_evt_time_hus = con_evt_time_hus + 2 * p_create_ind->cis_offset_min;
            iso_evt_time_hs = con_evt_time_hs + iso_evt_time_hus / HALF_SLOT_SIZE;
            iso_evt_time_hus = CO_MOD(iso_evt_time_hus, HALF_SLOT_SIZE);

            // Compute the offset, based on ISO interval and local clock
            act_offset = CO_MOD(iso_evt_time_hs, p_cig_param->iso_interval << 2);
            act_offset_hus = iso_evt_time_hus;
            act_margin_hus = 2 * (p_create_ind->cis_offset_max - p_create_ind->cis_offset_min);

            // Check if at least 2 half-slot boundaries are within the window proposed by master
            if (((act_offset_hus == 0) + ((act_offset_hus + act_margin_hus) / HALF_SLOT_SIZE)) > 1)
            {
                // Request an offset range to the planner (within the master proposed window)
                struct sch_plan_req_param req_param;
                req_param.interval_min    = p_cig_param->iso_interval << 2;
                req_param.interval_max    = req_param.interval_min;
                req_param.duration_min    = (2 * p_create_ind->air_time + (HALF_SLOT_SIZE - 1)) / HALF_SLOT_SIZE;
                req_param.duration_max    = req_param.duration_min;
                req_param.offset_min      = act_offset;
                req_param.offset_max      = act_offset + (act_margin_hus + 1) / HALF_SLOT_SIZE;
                req_param.margin          = 1;
                req_param.pref_period     = 0;
                req_param.conhdl          = BLE_ACTID_TO_CISHDL(p_create_ind->act_id);
                req_param.conhdl_ref      = BLE_LINKID_TO_CONHDL(link_id);

                if (sch_plan_req(&req_param) == SCH_PLAN_ERROR_OK)
                {
                    // The window preferred by the planner will be proposed to master
                    act_offset = req_param.offset_min;
                    act_margin_hus = (req_param.offset_max - req_param.offset_min) * HALF_SLOT_SIZE;
                }
            }

            // Offset of the ISO connection, based on ISO interval
            p_create_ind->act_offset = act_offset;
            p_create_ind->act_offset_hus = act_offset_hus;
            p_create_ind->act_margin_hus = act_margin_hus;

            // Select a reference connection event in the future
            con_evt_cnt = p_create_ind->conn_event_cnt;

            // Compute distance from the connection event to ISO event
            dist_con_evt_to_iso = llc_cis_con_evt_dist_compute(link_id, con_evt_time_hs, &con_evt_cnt, p_cig_param->iso_interval, p_create_ind->act_offset, false);

            // Save the new reference connection event counter
            p_create_ind->conn_event_cnt = con_evt_cnt;

            // Compute distance from connection event to ISO event (in us)
            p_create_ind->cis_offset_min = (dist_con_evt_to_iso - con_evt_time_hus + p_create_ind->act_offset_hus) >> 1;
            p_create_ind->cis_offset_max = p_create_ind->cis_offset_min + (p_create_ind->act_margin_hus >> 1);

            // Ensure CIS offset is greater than minimum value
            if (p_create_ind->cis_offset_min < BLE_CIS_MIN_OFFSET)
            {
                // Move to the next ISO interval
                p_create_ind->cis_offset_min += p_cig_param->iso_interval * 2 * SLOT_SIZE;
                p_create_ind->cis_offset_max += p_cig_param->iso_interval * 2 * SLOT_SIZE;
            }

            // Send LL_CIS_RSP PDU
            llc_cis_rsp_pdu_send(link_id, p_create_ind);

            // Start the LLCP Response TO
            llc_proc_timer_set(link_id, LLC_PROC_REMOTE, true);

            // Wait for reception of LL_CIS_UPDATE_IND from master
            llc_proc_state_set(&p_create_ind->proc, link_id, LLC_REM_CIS_WAIT_UPDATE_IND);
        }
        break;

        case LLC_REM_CIS_WAIT_UPDATE_IND:
        {
            finished = true;
        }
        break;

        default:
        {
            ASSERT_INFO(0, link_id, llc_proc_state_get(&p_create_ind->proc));
        }
        break;
        }
    }


    if (finished)
    {
        // Stop the LLCP Response TO
        llc_proc_timer_set(link_id, LLC_PROC_REMOTE, false);

        if (state != LLC_REM_CIS_HOST_REJECT)
        {
            if (status == CO_ERROR_NO_ERROR)
            {
                // Inform LLI CIS module that CIS channel creation procedure is finished
                lli_cis_create_end(p_create_ind->act_id, status, p_create_ind->access_addr,
                                   p_create_ind->conn_event_cnt, p_create_ind->cis_offset,
                                   p_create_ind->act_offset, p_create_ind->act_offset_hus,
                                   p_create_ind->air_time, (GETB(llc_env[link_id]->link_info, LLC_INFO_LINK_ENCRYPTED)));
            }
            else
            {
                // Inform LLI CIS module that CIS channel creation procedure is finished
                lli_cis_create_end(p_create_ind->act_id, status, 0, 0, 0, 0, 0, 0, 0);
            }
        }

        // Unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_REMOTE);
    }
}

/**
 ****************************************************************************************
 * @brief Handles the CIS creation error (UNKNOWN/REJECT).
 *
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void llc_cis_rem_create_err_cb(uint8_t link_id, uint8_t error_type, void *p_param)
{
    uint8_t status  = CO_ERROR_NO_ERROR;

    switch (error_type)
    {
    case LLC_ERR_DISCONNECT:
    {
        status = *((uint8_t *)p_param);
    }
    break;

    case LLC_ERR_LLCP_UNKNOWN_RSP:
    {
        struct ll_unknown_rsp *p_rsp = (struct ll_unknown_rsp *) p_param;
        if (p_rsp->unk_type == LL_CONNECTION_PARAM_REQ_OPCODE)
        {
            status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
        }
    }
    break;

    case LLC_ERR_LLCP_REJECT_EXT_IND:
    {
        struct ll_reject_ext_ind *p_ind = (struct ll_reject_ext_ind *)p_param;
        if (p_ind->rej_op_code == LL_CIS_REQ_OPCODE)
        {
            status = p_ind->err_code;
        }
    }
    break;
    case LLC_ERR_LLCP_REJECT_IND:
    {
        struct ll_reject_ind *p_reject = (struct ll_reject_ind *) p_param;
        status =  p_reject->err_code;
    }
    break;
    default:
    {
        // Nothing to do, ignore
    } break;
    }

    if (status != CO_ERROR_NO_ERROR)
    {
        // Get the procedure environment
        struct llc_op_cis_create_ind *p_create_ind
            = (struct llc_op_cis_create_ind *)llc_proc_get(link_id, LLC_PROC_REMOTE);

        // Finish the procedure
        llc_cis_rem_create_proc_continue(link_id, llc_proc_state_get(&p_create_ind->proc), status);
    }
}
#endif //(BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Continue execution of local stop procedure
 *
 * @param[in] link_id   Link identifier
 * @param[in] state     Expected state of the procedure
 * @param[in] status    Status of the operation
 ****************************************************************************************
 */
__STATIC void llc_cis_loc_stop_proc_continue(uint8_t link_id, uint8_t state, uint8_t status)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(PROC_CONTINUE_PATCH_TYPE, LLC_CIS_LOC_STOP_PROC_CONTINUE_C_FUN_BIT, link_id, state, status);
    // Indicate if procedure if finished
    bool finished = false;
    // Get the procedure environment
    struct llc_op_cis_stop_ind *p_stop_ind = (struct llc_op_cis_stop_ind *)llc_proc_get(link_id, LLC_PROC_LOCAL);

    // Check that current procedure state equals to expected state given in parameter
    if (llc_proc_state_get(&p_stop_ind->proc) != state)
    {
        ASSERT_WARN(0, llc_proc_state_get(&p_stop_ind->proc), state);

        // Stop the procedure
        finished = true;
    }
    else
    {
        // Perform next operation based on current procedure state
        switch (llc_proc_state_get(&p_stop_ind->proc))
        {
        case LLC_LOC_CIS_STOP_START:
        {
            struct lli_cis_param *p_cis_param;

            // CIS channel parameters
            lli_ci_param_get(p_stop_ind->act_id, NULL, &p_cis_param);

            // Send LL_CIS_TERMINATE_IND PDU
            llc_cis_terminate_ind_pdu_send(link_id, p_cis_param, p_stop_ind->reason);

            // Wait for acknowledgment
            llc_proc_state_set(&p_stop_ind->proc, link_id, LLC_LOC_CIS_WAIT_TERMINATE_ACK);
        }
        break;

        case LLC_LOC_CIS_WAIT_TERMINATE_ACK:
        {
            finished = true;
        }
        break;

        default:
        {
            ASSERT_INFO(0, link_id, llc_proc_state_get(&p_stop_ind->proc));
        }
        break;
        }
    }

    if (finished)
    {
        // Unregister procedure
        llc_proc_unreg(link_id, LLC_PROC_LOCAL);
    }
}

/**
 ****************************************************************************************
 * @brief Handles the CIS stop error (UNKNOWN/REJECT).
 *
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
__STATIC void llc_cis_loc_stop_err_cb(uint8_t link_id, uint8_t error_type, void *p_param)
{
    uint8_t status  = CO_ERROR_NO_ERROR;

    switch (error_type)
    {
    case LLC_ERR_DISCONNECT:
    {
        status = *((uint8_t *)p_param);
    }
    break;

    case LLC_ERR_LLCP_UNKNOWN_RSP:
    {
        struct ll_unknown_rsp *p_rsp = (struct ll_unknown_rsp *) p_param;
        if (p_rsp->unk_type == LL_CONNECTION_PARAM_REQ_OPCODE)
        {
            status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
        }
    }
    break;

    case LLC_ERR_LLCP_REJECT_EXT_IND:
    {
        struct ll_reject_ext_ind *p_ind = (struct ll_reject_ext_ind *)p_param;
        if (p_ind->rej_op_code == LL_CIS_TERMINATE_IND_OPCODE)
        {
            status = p_ind->err_code;
        }
    }
    break;
    case LLC_ERR_LLCP_REJECT_IND:
    {
        struct ll_reject_ind *p_reject = (struct ll_reject_ind *) p_param;
        status =  p_reject->err_code;
    }
    break;
    default:
    {
        // Nothing to do, ignore
    } break;
    }

    if (status != CO_ERROR_NO_ERROR)
    {
        // Get the procedure environment
        struct llc_op_cis_stop_ind *p_stop_ind
            = (struct llc_op_cis_stop_ind *)llc_proc_get(link_id, LLC_PROC_LOCAL);

        // Finish the procedure
        llc_cis_loc_stop_proc_continue(link_id, llc_proc_state_get(&p_stop_ind->proc), status);
    }
}


#if (BLE_CENTRAL)
uint8_t llc_cis_create_req(uint8_t link_id, uint8_t act_id, uint16_t act_offset, uint16_t act_margin)
{
    // Command status
    uint8_t status = CO_ERROR_NO_ERROR;

    do
    {
        struct lli_cig_param *p_cig_param;
        struct lli_cis_param *p_cis_param;
        // Compute LLC task index for provided connection
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
        // Allocated procedure structure for CIS creation
        struct llc_op_cis_create_ind *p_create_ind;

        uint8_t supp_phy_rem_msk = PHY_1MBPS_BIT;
        SETB(supp_phy_rem_msk, PHY_2MBPS, llc_le_feature_check(link_id, BLE_FEAT_2M_PHY));
        SETB(supp_phy_rem_msk, PHY_CODED, llc_le_feature_check(link_id, BLE_FEAT_CODED_PHY));

        // Check if link exists or is being disconnected and if we are well master
        if (llc_is_disconnecting(link_id) || !GETB(llc_env[link_id]->link_info, LLC_INFO_MASTER_ROLE))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Check if peer support this feature
        if (!llc_le_feature_check(link_id, BLE_FEAT_CIS_SLAVE))
        {
            status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
            break;
        }

        lli_ci_param_get(act_id, &p_cig_param, &p_cis_param);

        // check that at least one PHY could be supported
        if (((co_rate_to_phy_mask[p_cis_param->tx_rate] &supp_phy_rem_msk) == 0)
                || ((co_rate_to_phy_mask[p_cis_param->rx_rate] &supp_phy_rem_msk) == 0))
        {
            status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
            break;
        }

        // Allocate procedure structure for CIS creation
        p_create_ind = KE_MSG_ALLOC(LLC_OP_CIS_CREATE_IND, llc_id, llc_id, llc_op_cis_create_ind);

        llc_proc_init(&p_create_ind->proc, LLC_PROC_CIS_CREATE, llc_cis_loc_create_err_cb);
        p_create_ind->act_id          = act_id;
        p_create_ind->local           = true;
        p_create_ind->role            = ROLE_MASTER;
        p_create_ind->act_offset      = act_offset;
        p_create_ind->act_margin_hus  = act_margin * HALF_SLOT_SIZE;
        p_create_ind->act_offset_hus  = 0;
        p_create_ind->air_time        = p_cig_param->cig_sync_delay;
        // Send the kernel message
        ke_msg_send(p_create_ind);
    }
    while (0);

    // Return status
    return (status);
}
#endif //(BLE_CENTRAL)

#if (BLE_PERIPHERAL)
void llc_cis_create_rsp(uint8_t link_id, uint8_t status)
{
    // Get the procedure environment
    struct llc_op_cis_create_ind *p_create_ind = (struct llc_op_cis_create_ind *)llc_proc_get(link_id, LLC_PROC_REMOTE);

    // Sanity check
    ASSERT_ERR(llc_proc_id_get(link_id, LLC_PROC_REMOTE) == LLC_PROC_CIS_CREATE);
    ASSERT_ERR(llc_proc_state_get(&p_create_ind->proc) == LLC_REM_CIS_WAIT_HOST_STATUS);

    // Stop the CIS connection accept timeout timer
    ke_timer_clear(LLC_CIS_ACCEPT_TO, KE_BUILD_ID(TASK_LLC, link_id));

    if (status != CO_ERROR_NO_ERROR)
    {
        // Send LL_REJECT_EXT_IND
        llc_ll_reject_ind_pdu_send(link_id, LL_CIS_REQ_OPCODE, status, NULL);
        // Clear the state machine
        llc_proc_state_set(&p_create_ind->proc, link_id, LLC_REM_CIS_HOST_REJECT);

        // Continue remote procedure execution
        llc_cis_rem_create_proc_continue(link_id, LLC_REM_CIS_HOST_REJECT, status);
    }
    else
    {
        // Continue remote procedure execution
        llc_cis_rem_create_proc_continue(link_id, LLC_REM_CIS_WAIT_HOST_STATUS, status);
    }
}
#endif //(BLE_PERIPHERAL)

uint8_t llc_cis_stop_req(uint8_t link_id, uint8_t act_id, uint8_t reason)
{
    // Status
    uint8_t status = CO_ERROR_NO_ERROR;

    do
    {
        // Compute LLC task index for provided connection
        ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
        // Allocated procedure structure for CIS stop
        struct llc_op_cis_stop_ind *p_stop_ind;

        // Check if link exists or is being disconnected
        if (llc_is_disconnecting(link_id))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Allocate procedure structure for CIS stop
        p_stop_ind = KE_MSG_ALLOC(LLC_OP_CIS_STOP_IND, llc_id, llc_id, llc_op_cis_stop_ind);

        llc_proc_init(&p_stop_ind->proc, LLC_PROC_CIS_STOP, llc_cis_loc_stop_err_cb);
        p_stop_ind->act_id         = act_id;
        p_stop_ind->reason         = reason;

        // Send the kernel message
        ke_msg_send(p_stop_ind);
    }
    while (0);

    // Return status
    return (status);
}

#if (BLE_CENTRAL)
uint8_t llc_cis_cancel(uint8_t link_id, uint8_t reason)
{
    // Status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    // Get the procedure environment
    struct llc_op_cis_create_ind *p_create_ind = (struct llc_op_cis_create_ind *)llc_proc_get(link_id, LLC_PROC_LOCAL);
    uint8_t state = llc_proc_state_get(&p_create_ind->proc);

    // Cancel the procedure if not yet received the LL CIS RSP
    if ((state == LLC_LOC_CIS_CREATE_START) || (state == LLC_LOC_CIS_WAIT_CREATE_RSP))
    {
        llc_proc_state_set(&p_create_ind->proc, link_id, LLC_LOC_CIS_CANCEL);

        llc_cis_loc_create_proc_continue(link_id, LLC_LOC_CIS_CANCEL, reason);

        status = CO_ERROR_NO_ERROR;
    }

    return status;
}
#endif //(BLE_CENTRAL)


#endif // BLE_CIS_IN_ROM

/*
 * LLCP HANDLERS
 ****************************************************************************************
 */

#if (BLE_PERIPHERAL)
/**
 ****************************************************************************************
 *  @brief Handles the reception of a LLCP CIS Creation request. Can be received either as
 *  slave or master. Initiate creation of a remote procedure.
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent.
 * @param[in] p_pdu          LLCP PDU information received
 * @param[in] event_cnt      Event counter value when PDU has been received
 *
 * @return status code of handler:
 *    - CO_ERROR_NO_ERROR:               Nothing more to do
 *    - others:                          Send an LLCP_REJECT_IND or LLCP_REJECT_IND_EXT
 ****************************************************************************************
 */
uint8_t ll_cis_req_handler_patch(uint8_t link_id, struct ll_cis_req *p_pdu, uint16_t event_cnt)
{
    //FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLC_CIS_PATCH_TYPE, LL_CIS_REQ_HANDLER_FUN_BIT, uint8_t, link_id, p_pdu, event_cnt);
#if !(BLE_CIS_IN_ROM)
    // Returned status
    uint8_t status;

    do
    {
        // Allocated activity ID
        uint8_t  act_id;
        // Connection and channel interval in microseconds
        uint32_t conn_intv_us, iso_intv_us;
        uint16_t air_exch_dur;
        uint32_t cis_air_time;
        uint8_t  framing;
        uint8_t  mic_len;

        uint8_t supp_phy_msk = PHY_1MBPS_BIT;
        SETB(supp_phy_msk, PHY_2MBPS, BLE_PHY_2MBPS_SUPPORT);
        SETB(supp_phy_msk, PHY_CODED, BLE_PHY_CODED_SUPPORT);

        // Check that no remote procedure is on-going and that we are slave
        if ((llc_proc_id_get(link_id, LLC_PROC_REMOTE) != LLC_PROC_NONE)
                || GETB(llc_env[link_id]->link_info, LLC_INFO_MASTER_ROLE))
        {
            // Already handling a remote procedure
            status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
            break;
        }

        // Check if the Isochronous Channels (Host Support) feature bit is set
        if (!llm_le_feature_check(BLE_FEAT_CIS_HOST_SUPPORT))
        {
            status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
            break;
        }

        // Check if feature is locally supported
        if (!llm_le_evt_mask_check(LE_EVT_MASK_CIS_REQ_EVT_BIT))
        {
            // Reject to inform that remote feature is not supported
            status = CO_ERROR_UNSUPPORTED_REMOTE_FEATURE;
            break;
        }

        // Get connection interval in microseconds - Given in multiple of 1,25ms
        conn_intv_us = llc_env[link_id]->con_params.interval * SLOT_SIZE * 2;
        iso_intv_us = p_pdu->iso_interval * SLOT_SIZE * 2;


        // Check received CIS offset and subevent interval values
        if ((p_pdu->cis_offset_min < BLE_CIS_MIN_OFFSET)
                || (p_pdu->cis_offset_min > p_pdu->cis_offset_max)
                || ((p_pdu->nse > 1) && ((p_pdu->sub_interval < BLE_CIS_MIN_SUBEVENT_INTV)
                                         || (p_pdu->sub_interval > iso_intv_us))))
        {
            status = CO_ERROR_INVALID_LMP_PARAM;
            break;
        }

        // Unpack SDU parameters
        framing                 = GETB(p_pdu->max_sdu_m2s,      BLE_CIS_FRAME_MODE);
        p_pdu->max_sdu_m2s      = GETF(p_pdu->max_sdu_m2s,      BLE_CIS_MAX_SDU);
        p_pdu->max_sdu_s2m      = GETF(p_pdu->max_sdu_s2m,      BLE_CIS_MAX_SDU);
        p_pdu->sdu_interval_m2s = GETF(p_pdu->sdu_interval_m2s, BLE_CIS_SDU_INTERVAL);
        p_pdu->sdu_interval_s2m = GETF(p_pdu->sdu_interval_s2m, BLE_CIS_SDU_INTERVAL);

        // Check provided parameters
        if ((p_pdu->cig_id > BLE_CIG_MAX_ID)                     || (p_pdu->cis_id > BLE_CIS_MAX_ID)
                || (p_pdu->iso_interval < BLE_ISO_MIN_INTERVAL)         || (p_pdu->iso_interval > BLE_ISO_MAX_INTERVAL)
                || (p_pdu->max_pdu_m2s > BLE_ISO_MAX_PAYLOAD_SIZE)      || (p_pdu->max_pdu_s2m > BLE_ISO_MAX_PAYLOAD_SIZE)
                || (p_pdu->nse < BLE_ISO_MIN_NSE)                       || (p_pdu->nse > BLE_ISO_MAX_NSE)
                || (p_pdu->ft_m2s < BLE_CIS_MIN_FT)                     || (p_pdu->ft_m2s > BLE_CIS_MAX_FT)
                || (p_pdu->ft_s2m < BLE_CIS_MIN_FT)                     || (p_pdu->ft_s2m > BLE_CIS_MAX_FT)
                || (GETF(p_pdu->bn, LLCP_CIS_BN_M2S) > BLE_CIS_MAX_BN)  || (GETF(p_pdu->bn, LLCP_CIS_BN_S2M) > BLE_CIS_MAX_BN)
                || (NB_ONE_BITS(p_pdu->phy_m2s) != 1)                   || (NB_ONE_BITS(p_pdu->phy_s2m) != 1)
                || (p_pdu->max_sdu_m2s > BLE_ISO_MAX_SDU_SIZE)          || (p_pdu->max_sdu_s2m > BLE_ISO_MAX_SDU_SIZE)
                || (p_pdu->max_sdu_m2s > (p_pdu->max_pdu_m2s * GETF(p_pdu->bn, LLCP_CIS_BN_M2S) * p_pdu->sdu_interval_m2s / iso_intv_us))
                || (p_pdu->max_sdu_s2m > (p_pdu->max_pdu_s2m * GETF(p_pdu->bn, LLCP_CIS_BN_S2M) * p_pdu->sdu_interval_s2m / iso_intv_us)))
        {
            status = CO_ERROR_INVALID_LMP_PARAM;
            break;
        }

        // The SDU interval can be ignored for a unidirectional CIS
        if ((p_pdu->max_sdu_m2s > 0)
                && ((p_pdu->sdu_interval_m2s < BLE_ISO_MIN_SDU_INTERVAL) || (p_pdu->sdu_interval_m2s > BLE_ISO_MAX_SDU_INTERVAL)))
        {
            status = CO_ERROR_INVALID_LMP_PARAM;
            break;
        }

        // The SDU interval can be ignored for a unidirectional CIS
        if ((p_pdu->max_sdu_s2m > 0)
                && ((p_pdu->sdu_interval_s2m < BLE_ISO_MIN_SDU_INTERVAL) || (p_pdu->sdu_interval_s2m > BLE_ISO_MAX_SDU_INTERVAL)))
        {
            status = CO_ERROR_INVALID_LMP_PARAM;
            break;
        }

        if (framing == ISO_UNFRAMED_MODE)
        {
            // ISO interval and SDU interval must be multiple
            if (((p_pdu->max_sdu_m2s > 0) && (CO_MOD(iso_intv_us, p_pdu->sdu_interval_m2s) != 0))
                    || ((p_pdu->max_sdu_s2m > 0) && (CO_MOD(iso_intv_us, p_pdu->sdu_interval_s2m) != 0)))
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }

            // a round number of bursts is required per SDU interval
            if (((p_pdu->max_sdu_m2s > 0) && (CO_MOD(GETF(p_pdu->bn, LLCP_CIS_BN_M2S), iso_intv_us / p_pdu->sdu_interval_m2s) != 0))
                    || ((p_pdu->max_sdu_s2m > 0) && (CO_MOD(GETF(p_pdu->bn, LLCP_CIS_BN_S2M), iso_intv_us / p_pdu->sdu_interval_s2m) != 0)))
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }
        }

        // Check if proposed rate is supported
        if (((p_pdu->phy_m2s & ~(supp_phy_msk)) != 0) || ((p_pdu->phy_s2m & ~(supp_phy_msk)) != 0))
        {
            status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
            break;
        }

        // extract the rate and copy it in PDU
        p_pdu->phy_s2m = co_phy_to_rate[co_phy_mask_to_value[p_pdu->phy_s2m]];
        p_pdu->phy_m2s = co_phy_to_rate[co_phy_mask_to_value[p_pdu->phy_m2s]];

        // Check if the link is encrypted to determine the length of the MIC
        mic_len = (GETB(llc_env[link_id]->link_info, LLC_INFO_LINK_ENCRYPTED)) ? MIC_LEN : 0;

        //compute air exchange duration and check that it fits into sub_event interval
        air_exch_dur  = (p_pdu->max_pdu_m2s > 0) // check if mic len must be added if slave 2 master packet len != 0
                        ? ble_util_pkt_dur_in_us(p_pdu->max_pdu_m2s + mic_len, p_pdu->phy_m2s)
                        : ble_util_pkt_dur_in_us(0, p_pdu->phy_m2s);
        air_exch_dur += (p_pdu->max_pdu_s2m > 0) // check if mic len must be added if slave 2 master packet len != 0
                        ? ble_util_pkt_dur_in_us(p_pdu->max_pdu_s2m + mic_len, p_pdu->phy_s2m)
                        : ble_util_pkt_dur_in_us(0, p_pdu->phy_s2m);
        air_exch_dur += 2 * BLE_IFS_DUR;


        // Compute the required CIS Air Time
        cis_air_time       = (p_pdu->sub_interval * (p_pdu->nse - 1)) + air_exch_dur;

        // check that packet duration is not greater that sub event interval
        if (((p_pdu->nse > 1) && (air_exch_dur > p_pdu->sub_interval))
                // check that event duration is not greater than channel interval
                || (cis_air_time > iso_intv_us)
                // check that cis_offset_max is less than (connInterval - cis_air_time)
                || (p_pdu->cis_offset_max >= (conn_intv_us - cis_air_time)))
        {
            status = CO_ERROR_INVALID_LMP_PARAM;
            break;
        }

        // Inform LLI that peer has requested creation of a new CIS channel
        status = lli_cis_create_req(link_id, p_pdu, framing, &act_id);

        if (status == CO_ERROR_NO_ERROR)
        {
            // Compute LLC task index for provided connection
            ke_task_id_t llc_id = KE_BUILD_ID(TASK_LLC, link_id);
            // Allocate procedure structure for CIS creation
            struct llc_op_cis_create_ind *p_create_ind = KE_MSG_ALLOC(LLC_OP_CIS_CREATE_IND, llc_id, llc_id, llc_op_cis_create_ind);

            llc_proc_init(&p_create_ind->proc, LLC_PROC_CIS_CREATE, llc_cis_rem_create_err_cb);
            p_create_ind->act_id           = act_id;
            p_create_ind->local            = false;
            p_create_ind->role             = ROLE_SLAVE;
            p_create_ind->air_time         = cis_air_time;
            p_create_ind->cis_offset_min   = p_pdu->cis_offset_min;
            p_create_ind->cis_offset_max   = p_pdu->cis_offset_max;
            p_create_ind->conn_event_cnt   = p_pdu->conn_event_cnt;

            // Initialize procedure state
            llc_proc_state_set(&p_create_ind->proc, link_id, LLC_REM_CIS_CREATE_START);

            // Store new procedure as remote procedure
            llc_proc_reg(link_id, LLC_PROC_REMOTE, &p_create_ind->proc);

            // Continue remote procedure execution
            llc_cis_rem_create_proc_continue(link_id, LLC_REM_CIS_CREATE_START, CO_ERROR_NO_ERROR);
        }
    }
    while (0);

    return (status);
#else // BLE_CIS_IN_ROM
    return CO_ERROR_LMP_PDU_NOT_ALLOWED;

#endif // BLE_CIS_IN_ROM
}

/**
 ****************************************************************************************
 *  @brief Handles the reception of a LLCP CIS Creation/Update indication. Can be only
 *  received when slave as part of a local or a remote procedure.
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent.
 * @param[in] p_pdu          LLCP PDU information received
 * @param[in] event_cnt      Event counter value when PDU has been received
 *
 * @return status code of handler:
 *    - CO_ERROR_NO_ERROR:               Nothing more to do
 *    - others:                          Send an LLCP_REJECT_IND or LLCP_REJECT_IND_EXT
 ****************************************************************************************
 */
uint8_t ll_cis_ind_handler_patch(uint8_t link_id, struct ll_cis_ind *p_pdu, uint16_t event_cnt)
{
    //FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLC_CIS_PATCH_TYPE, LL_CIS_IND_HANDLER_FUN_BIT, uint8_t, link_id, p_pdu, event_cnt);
    // Handling status
    uint8_t status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
#if !(BLE_CIS_IN_ROM)
    do
    {
        // Procedure environment
        struct llc_op_cis_create_ind *p_create_ind;
        struct lli_cig_param *p_cig_param;
        struct lli_cis_param *p_cis_param;
        struct llc_env_tag *p_llc_env = llc_env[link_id];
        uint32_t iso_interval_us;
        uint32_t cis_offset;

        // Check that reception of this PDU is expected
        if (llc_proc_id_get(link_id, LLC_PROC_REMOTE) != LLC_PROC_CIS_CREATE)
        {
            break;
        }

        // Get the procedure environment
        p_create_ind = (struct llc_op_cis_create_ind *)llc_proc_get(link_id, LLC_PROC_REMOTE);

        // load CIS parameters
        lli_ci_param_get(p_create_ind->act_id, &p_cig_param, &p_cis_param);

        // Translate range from LL_CIS_RSP to the reference event of LL_CIS_IND
        llc_cis_offset_translate(p_create_ind->conn_event_cnt,
                                 p_pdu->conn_event_cnt,
                                 p_llc_env->con_params.interval,
                                 p_cig_param->iso_interval,
                                 &p_create_ind->cis_offset_min,
                                 &p_create_ind->cis_offset_max);

        // ISO interval in us
        iso_interval_us = p_cig_param->iso_interval * 2 * SLOT_SIZE;

        // Rebase CIS offset received in LL_CIS_IND
        cis_offset = CO_MOD(p_pdu->cis_offset, iso_interval_us);
        if (p_create_ind->cis_offset_max >= iso_interval_us)
        {
            cis_offset += iso_interval_us;
        }

        // The first CIS anchor point shall lie within a window equivalent to that specified by the LL_CIS_RSP PDU
        if ((cis_offset < p_create_ind->cis_offset_min) || (cis_offset > p_create_ind->cis_offset_max))
        {
            status = CO_ERROR_UNACCEPTABLE_CONN_PARAM;
        }
        else
        {
            uint32_t con_evt_time_hs, iso_evt_time_hs;
            uint32_t iso_evt_time_hus;
            uint16_t con_evt_time_hus;
            uint16_t con_evt_cnt;
            uint16_t act_offset;
            uint16_t act_offset_hus;

            // Store received parameters
            p_create_ind->access_addr      = p_pdu->aa;
            p_create_ind->cis_offset       = p_pdu->cis_offset;
            p_create_ind->conn_event_cnt   = p_pdu->conn_event_cnt;
            p_cig_param->cig_sync_delay    = p_pdu->cig_sync_delay;
            p_cis_param->cis_offset_in_cig = p_pdu->cig_sync_delay - p_pdu->cis_sync_delay;

            // Get current connection event time
            lld_con_time_get(link_id, &con_evt_cnt, &con_evt_time_hs, &con_evt_time_hus);

            // Compute connection event time
            con_evt_time_hs = CLK_ADD_2(con_evt_time_hs, BLE_UTIL_EVT_CNT_DIFF(p_create_ind->conn_event_cnt, con_evt_cnt) * (p_llc_env->con_params.interval << 2));

            // Compute first ISO event time
            iso_evt_time_hus = con_evt_time_hus + 2 * p_create_ind->cis_offset;
            iso_evt_time_hs = con_evt_time_hs + iso_evt_time_hus / HALF_SLOT_SIZE;
            iso_evt_time_hus = CO_MOD(iso_evt_time_hus, HALF_SLOT_SIZE);

            // Compute the offset, based on ISO interval and local clock
            act_offset = CO_MOD(iso_evt_time_hs, p_cig_param->iso_interval << 2);
            act_offset_hus = iso_evt_time_hus;

            p_create_ind->act_offset = act_offset;
            p_create_ind->act_offset_hus = act_offset_hus;
            status = CO_ERROR_NO_ERROR;
        }

        // Continue remote procedure execution
        llc_cis_rem_create_proc_continue(link_id, LLC_REM_CIS_WAIT_UPDATE_IND, status);
    }
    while (0);
#endif // BLE_CIS_IN_ROM
    return (status);
}
#endif //(BLE_PERIPHERAL)

#if (BLE_CENTRAL)
/**
 ****************************************************************************************
 *  @brief Handles the reception of a LLCP CIS Creation response. Can be received only
 *  when master of link, part of local procedure.
 *
 * @param[in] link_id        Link identifier on which the pdu will be sent.
 * @param[in] p_pdu          LLCP PDU information received
 * @param[in] event_cnt      Event counter value when PDU has been received
 *
 * @return status code of handler:
 *    - CO_ERROR_NO_ERROR:               Nothing more to do
 *    - others:                          Send an LLCP_REJECT_IND or LLCP_REJECT_IND_EXT
 ****************************************************************************************
 */
uint8_t ll_cis_rsp_handler_patch(uint8_t link_id, struct ll_cis_rsp *p_pdu, uint16_t event_cnt)
{
    //FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLC_CIS_PATCH_TYPE, LL_CIS_RSP_HANDLER_FUN_BIT, uint8_t, link_id, p_pdu, event_cnt);
#if !(BLE_CIS_IN_ROM)
    // Handling status
    uint8_t status = CO_ERROR_NO_ERROR;
    // Procedure environment
    struct llc_op_cis_create_ind *p_create_ind = NULL;

    do
    {
        // CIS channel parameters
        struct lli_cig_param *p_cig_param;
        struct lli_cis_param *p_cis_param;
        struct llc_env_tag *p_llc_env = llc_env[link_id];
        uint32_t iso_interval_us;

        // Check that local procedure is in progress for CIS channel creation else reject
        if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_CIS_CREATE)
        {
            status = CO_ERROR_LMP_PDU_NOT_ALLOWED;
            break;
        }

        // Get environment for local procedure
        p_create_ind = (struct llc_op_cis_create_ind *)llc_proc_get(link_id, LLC_PROC_LOCAL);

        // Load CIS parameters
        lli_ci_param_get(p_create_ind->act_id, &p_cig_param, &p_cis_param);

        // Check received CIS offset values
        if ((p_pdu->cis_offset_min < BLE_CIS_MIN_OFFSET) || (p_pdu->cis_offset_min > p_pdu->cis_offset_max)
                || (p_pdu->cis_offset_max > (uint32_t)((p_llc_env->con_params.interval * 2 * SLOT_SIZE) - p_cis_param->sub_evt_dur)))
        {
            status = CO_ERROR_INVALID_LMP_PARAM;
            break;
        }

        // Translate received CIS offset range to the reference event of LL_CIS_REQ
        llc_cis_offset_translate(p_pdu->conn_event_cnt,
                                 p_create_ind->conn_event_cnt,
                                 p_llc_env->con_params.interval,
                                 p_cig_param->iso_interval,
                                 &p_pdu->cis_offset_min,
                                 &p_pdu->cis_offset_max);

        // ISO interval in us
        iso_interval_us = p_cig_param->iso_interval * 2 * SLOT_SIZE;

        // Rebase CIS offsets min/max from LL_CIS_REQ
        p_create_ind->cis_offset_min = CO_MOD(p_create_ind->cis_offset_min, iso_interval_us);
        p_create_ind->cis_offset_max = CO_MOD(p_create_ind->cis_offset_max, iso_interval_us);
        if (p_create_ind->cis_offset_max < p_create_ind->cis_offset_min)
        {
            p_create_ind->cis_offset_max += iso_interval_us;
        }

        // The window specified by the LL_CIS_RSP PDU shall lie entirely within a window equivalent to that specified by the LL_CIS_REQ PDU
        if ((p_pdu->cis_offset_min < p_create_ind->cis_offset_min) || (p_pdu->cis_offset_max > p_create_ind->cis_offset_max))
        {
            status = CO_ERROR_UNACCEPTABLE_CONN_PARAM;
            break;
        }

        // Update initial range by the received range
        p_create_ind->cis_offset_min = p_pdu->cis_offset_min;
        p_create_ind->cis_offset_max = p_pdu->cis_offset_max;

    }
    while (0);

    if (p_create_ind)
    {
        // Continue local procedure execution
        llc_cis_loc_create_proc_continue(link_id, LLC_LOC_CIS_WAIT_CREATE_RSP, status);
    }

    return (status);
#else // BLE_CIS_IN_ROM
    return CO_ERROR_LMP_PDU_NOT_ALLOWED;
#endif // BLE_CIS_IN_ROM
}
#endif //(BLE_CENTRAL)

/**
 ****************************************************************************************
 *  @brief Handles the reception of a LLCP CIS termination indication.
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
uint8_t ll_cis_terminate_ind_handler_patch(uint8_t link_id, struct ll_cis_terminate_ind *p_pdu, uint16_t event_cnt)
{
    //FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LLC_CIS_PATCH_TYPE, LL_CIS_TERMINATE_IND_HANDLER_FUN_BIT, uint8_t, link_id, p_pdu, event_cnt);

#if !(BLE_CIS_IN_ROM)
    // Inform the LLI CIS module than LL_CIS_TERM_IND has been received
    lli_cis_stop_req(link_id, p_pdu);

#endif // BLE_CIS_IN_ROM
    return (CO_ERROR_NO_ERROR);
}

/*
 * KERNEL MESSAGE HANDLERS
 ****************************************************************************************
 */

#if (BLE_CENTRAL)
/**
 ****************************************************************************************
 * @brief handles message that convey master creation of an CIS channel as a local procedure
 ****************************************************************************************
 */
int llc_op_cis_create_ind_handler_patch(ke_msg_id_t const msgid, struct llc_op_cis_create_ind *p_param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    //FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLC_CIS_PATCH_TYPE, LLC_OP_CIS_CREATE_IND_HANDLER_FUN_BIT, int, msgid, p_param, dest_id, src_id);
#if !(BLE_CIS_IN_ROM)
    // Message status
    int msg_status = KE_MSG_NO_FREE;

    do
    {
        // Get link ID
        uint8_t link_id = KE_IDX_GET(dest_id);

        // Check if state in disconnected state
        if (llc_is_disconnecting(link_id))
        {
            // Discard the message
            msg_status = KE_MSG_CONSUMED;
            break;
        }

        // Sanity check
        ASSERT_ERR(p_param->local);

        // Check if another procedure is in progress
        if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_NONE)
        {
            // Process this message later
            msg_status = KE_MSG_SAVED;
            break;
        }

        // Initialize procedure state
        llc_proc_state_set(&p_param->proc, link_id, LLC_LOC_CIS_CREATE_START);

        // Store new procedure as local procedure
        llc_proc_reg(link_id, LLC_PROC_LOCAL, &p_param->proc);

        // Continue local procedure execution
        llc_cis_loc_create_proc_continue(link_id, LLC_LOC_CIS_CREATE_START, CO_ERROR_NO_ERROR);
    }
    while (0);

    return (msg_status);
#else // !(BLE_CIS_IN_ROM)
    return KE_MSG_CONSUMED;
#endif // !(BLE_CIS_IN_ROM)
}
#endif // (BLE_CENTRAL)

/**
 ****************************************************************************************
 * @brief handles request to terminate an CIS as a local procedure
 ****************************************************************************************
 */
int llc_op_cis_stop_ind_handler_patch(ke_msg_id_t const msgid, struct llc_op_cis_stop_ind *p_param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    //FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLC_CIS_PATCH_TYPE, LLC_OP_CIS_STOP_IND_HANDLER_FUN_BIT, int, msgid, p_param, dest_id, src_id);

#if !(BLE_CIS_IN_ROM)
    // Message status
    int msg_status = KE_MSG_NO_FREE;

    do
    {
        // Get link ID
        uint8_t link_id = KE_IDX_GET(dest_id);

        // Check if state in disconnected state
        if (llc_is_disconnecting(link_id))
        {
            // Discard the message
            msg_status = KE_MSG_CONSUMED;
            break;
        }

        // Check if another procedure is in progress
        if (llc_proc_id_get(link_id, LLC_PROC_LOCAL) != LLC_PROC_NONE)
        {
            // Process this message later
            msg_status = KE_MSG_SAVED;
            break;
        }

        // Initialize procedure state
        llc_proc_state_set(&p_param->proc, link_id, LLC_LOC_CIS_STOP_START);

        // Store new procedure as local procedure
        llc_proc_reg(link_id, LLC_PROC_LOCAL, &p_param->proc);

        // Continue local procedure execution
        llc_cis_loc_stop_proc_continue(link_id, LLC_LOC_CIS_STOP_START, CO_ERROR_NO_ERROR);
    }
    while (0);

    return (msg_status);
#else // !(BLE_CIS_IN_ROM)
    return KE_MSG_CONSUMED;
#endif // !(BLE_CIS_IN_ROM)
}

#if (BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Handles the CIS connection accept timeout
 ****************************************************************************************
 */
int llc_cis_accept_to_handler_patch(ke_msg_id_t const msgid, void *param,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    //FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LLC_CIS_PATCH_TYPE, LLC_CIS_ACCEPT_TO_HANDLER_FUN_BIT, int, msgid, param, dest_id, src_id);

#if    !(BLE_CIS_IN_ROM)

    uint8_t link_id = KE_IDX_GET(dest_id);

    // Check if state in disconnected state
    if (llc_is_disconnecting(link_id))
    {
        // LLC is IDLE, discard the message
    }
    else
    {
        // Sanity check
        ASSERT_ERR(llc_proc_id_get(link_id, LLC_PROC_REMOTE) == LLC_PROC_CIS_CREATE);

        // Send LL_REJECT_EXT_IND
        llc_ll_reject_ind_pdu_send(link_id, LL_CIS_REQ_OPCODE, CO_ERROR_CONN_ACCEPT_TIMEOUT_EXCEED, NULL);

        // Continue remote procedure execution
        llc_cis_rem_create_proc_continue(link_id, LLC_REM_CIS_WAIT_HOST_STATUS, CO_ERROR_CONN_ACCEPT_TIMEOUT_EXCEED);
    }
#endif //!(BLE_CIS_IN_ROM)
    return (KE_MSG_CONSUMED);
}
#endif //(BLE_PERIPHERAL)



/*
 * GLOBAL FUNCTIONS
 ****************************************************************************************
 */


#endif // (BLE_CIS)

/// @} LLC_CIS
