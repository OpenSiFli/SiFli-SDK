/**
****************************************************************************************
*
* @file lc_sniff.c
*
* @brief Link Controller Sniff functions
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup LCSNIFF
* @{
****************************************************************************************
*/

#include "rwip_config.h"

#include "ke_msg.h"
#include "ke_timer.h"
#include "co_utils.h"
#include "co_math.h"

#include "lc.h"
#include "lc_int.h"
#include "lc_sniff.h"            // lm low power definitions
#include "lc_lmppdu.h"
#include "lc_util.h"             // lc utilities definitions
#include "sch_plan.h"            // Scheduling Planner definitions
#include "ld.h"                  // link driver definitions
#include "hci.h"

#include "dbg.h"                 // debug definitions
#include "string.h"              // for memset usage

#if (EAVESDROPPING_SUPPORT)
    #include "../lm/lm_int.h"
#endif // EAVESDROPPING_SUPPORT

/*
 * DEFINITIONS
 ****************************************************************************************
 */
#define SNIFF_SUBRATE_SAFETY_MARGIN 0x03

/// Minimum SSR BB exchange
#define SSR_MIN_BB_EXCHG             10
/// Minimum SSR offset
#define SSR_MIN_OFFSET               96


/*
 * ENUMERATIONS DEFINITION
 ****************************************************************************************
 */

/// Sniff Negotiation Action
enum LC_SNIFF_ACTION
{
    LC_SNIFF_ACCEPT,
    LC_SNIFF_REJECT,
    LC_SNIFF_REQUEST,
};

#if (EAVESDROPPING_SUPPORT)
/// Sniff mode parameter in VS sniff mode change event
enum VS_SNIFF_MODE
{
    ACTIVE,
    SNIFF_PENDING,
    SNIFF,
    UNSNIFF_PENDING,
};
#endif // EAVESDROPPING_SUPPORT


/*
 * STRUCTURES DEFINITION
 ****************************************************************************************
 */

/// Sniff parameters
struct lc_sniff_params
{
    /// Sniff interval (in slots)
    uint16_t intv;
    /// Sniff offset (in slots)
    uint16_t offset;
    /// Number of sniff attempts
    uint16_t attempt;
    /// Sniff timeout (number of sniff attempts to reach timeout)
    uint16_t to;
    /// Timing initialization flag
    uint8_t  timing_flag;
    /// Initiator of the sniff procedure
    bool     initiator;
};

/// Sniff Subrating parameters
struct lc_sniff_ssr_params
{
    uint32_t ssr_instant;
    uint16_t max_lat;
    uint16_t min_rem_to;
    uint16_t min_loc_to_hci;
    uint16_t min_loc_to_peer;
    uint16_t min_loc_to;
    uint8_t  LocalSniffSubRate;
    uint8_t  RemoteSniffSubRate;
};

/// LC Sniff structure
struct lc_sniff_env_tag
{
    struct lc_sniff_params sniff;
    struct lc_sniff_ssr_params ssr;
    struct sch_plan_elt_tag plan_elt;
};


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// Sniff Configuration
struct lc_sniff_env_tag  lc_sniff_env[MAX_NB_ACTIVE_ACL];


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function is used to compute the remote sniff sub rate
 *        to send a LMP_SniffSubrating_req to the peer device once the local
 *        device is in sniff mode
 *
 * @param[in]  LinkSupTO                link supervision time out
 * @param[out] RemoteSniffSubRate       remote sniff sub rate value
 * @param[in]  max_lat               maximum delay
 * @param[in]  SniffInterval            sniff interval
 * @param[in]  SniffAttempts            sniff attempts
 *
 ****************************************************************************************
 */
__STATIC void lc_sniff_subrate_rem_compute(uint16_t LinkSupTO, uint8_t *RemoteSniffSubRate,
        uint16_t max_lat, uint16_t SniffInterval, uint16_t SniffAttempts)
{
    uint16_t UsedLatency;
    /*
     * Get the shortest value between the maximum allowed latency and the link
     * supervision TO, except if the link timeout is infinite
     */
    if (LinkSupTO == 0)
    {
        UsedLatency = max_lat;
    }
    else if (LinkSupTO <= SniffAttempts)
    {
        /* If link Timeout is short, set the latency to 0 */
        UsedLatency = 0;
    }
    else
    {
        uint32_t CalculatedLantency = (uint32_t)max_lat;
        CalculatedLantency *= SNIFF_SUBRATE_SAFETY_MARGIN;
        if ((uint32_t)LinkSupTO > (CalculatedLantency + SniffAttempts))
        {
            UsedLatency = max_lat;
        }
        else
        {
            UsedLatency = LinkSupTO / SNIFF_SUBRATE_SAFETY_MARGIN;
        }
    }
    /* Get Remote Sniff SubRating configuration */
    if ((UsedLatency / SniffInterval) > 0xFF)
    {
        *RemoteSniffSubRate = 0xFF;
    }
    else
    {
        *RemoteSniffSubRate = (uint8_t)(UsedLatency / SniffInterval);
    }
    if (*RemoteSniffSubRate == 0)
    {
        *RemoteSniffSubRate = 1;
    }
}

/**
 ****************************************************************************************
 * @brief Compute the sub rate from the local and remote sub rates.
 *
 * @param[in] LocalSniffSubRate        local sniff sub rate value
 * @param[in] RemoteSniffSubRate       remote sniff sub rate value
 *
 * @return sniff sub rate value
 ****************************************************************************************
 */
__STATIC uint8_t lc_sniff_subrate_compute(uint8_t LocalSniffSubRate, uint8_t RemoteSniffSubRate)
{
    uint8_t SniffSubRate;

    if (LocalSniffSubRate > RemoteSniffSubRate)
    {
        SniffSubRate = LocalSniffSubRate / RemoteSniffSubRate;
        SniffSubRate = SniffSubRate * RemoteSniffSubRate;
    }
    else
    {
        /* If the local sub-rate is the smallest or if both are equal, use the local one*/
        SniffSubRate = LocalSniffSubRate;
    }

    return (SniffSubRate);
}

__STATIC void lc_send_pdu_unsniff_req(uint8_t link_id, uint8_t role)
{
    struct lmp_unsniff_req pdu;

    pdu.opcode = LMP_OPCODE(LMP_UNSNIFF_REQ_OPCODE, role);

    lc_send_lmp(link_id, &pdu);
}

__STATIC void lc_send_pdu_sniff_req(uint8_t link_id)
{
    struct lmp_sniff_req pdu;
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    pdu.d_sniff       = lc_sniff_env[link_id].sniff.offset;
    pdu.flags         = lc_sniff_env[link_id].sniff.timing_flag;
    pdu.sniff_attempt = lc_sniff_env[link_id].sniff.attempt;
    pdu.sniff_to      = lc_sniff_env[link_id].sniff.to;
    pdu.t_sniff       = lc_sniff_env[link_id].sniff.intv;
    pdu.opcode        = LMP_OPCODE(LMP_SNIFF_REQ_OPCODE, lc_env_ptr->link.Role ^ !lc_sniff_env[link_id].sniff.initiator);
    lc_send_lmp(link_id, &pdu);
}

#if (EAVESDROPPING_SUPPORT)
__STATIC void lc_send_sniff_mode_change_evt(uint8_t link_id, enum VS_SNIFF_MODE sniff_mode)
{
    if (lm_env.hci.notify_sniff_events)
    {
        struct lc_env_tag *lc_env_ptr = lc_env[link_id];

        // Sniff mode has changed -> send the custom HCI-VS Sniff mode change event
        struct hci_vs_sniff_mode_change_evt *event = KE_MSG_ALLOC(HCI_DBG_EVT, 0, 0, hci_vs_sniff_mode_change_evt);
        event->subcode = HCI_VS_SNIFF_MODE_CHANGE_EVT_SUBCODE;
        memcpy(&event->bd_addr.addr[0], &lc_env_ptr->info.BdAddr.addr[0], BD_ADDR_LEN);
        // Sniff mode change data
        event->role = lc_env_ptr->link.Role;
        event->sniff_mode = sniff_mode;
        if (sniff_mode != ACTIVE)
        {
            event->sniff_interval = lc_sniff_env[link_id].sniff.intv;
            event->sniff_delay = lc_sniff_env[link_id].sniff.offset;
            event->sniff_attempt = lc_sniff_env[link_id].sniff.attempt;
            event->sniff_timeout = lc_sniff_env[link_id].sniff.to;
        }
        else
        {
            event->sniff_interval = 0;
            event->sniff_delay = 0;
            event->sniff_attempt = 0;
            event->sniff_timeout = 0;
        }
        // Send event
        hci_send_2_host(event);
    }
}
#endif // EAVESDROPPING_SUPPORT

/**
 ****************************************************************************************
 * @brief Enter sniff mode
 * @param[in] pid        process identifier.
 ****************************************************************************************
 */
__STATIC void lc_sniff_enter(ke_task_id_t pid)
{
    int link_id = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    uint16_t interval = lc_sniff_env[link_id].sniff.intv;
    uint16_t offset = lc_sniff_env[link_id].sniff.offset;

    ld_acl_sniff(link_id, offset, interval,
                 lc_sniff_env[link_id].sniff.attempt, lc_sniff_env[link_id].sniff.to,
                 ((lc_sniff_env[link_id].sniff.timing_flag & INIT2_FLAG) != 0));

    if ((lc_env_ptr->link.LinkTimeout != 0)
            && (lc_env_ptr->link.LinkTimeout < interval))
    {
        ld_acl_lsto_set(link_id, interval);

        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            //  send LMP_SupervisionTimeout
            lc_send_pdu_lsto(link_id, interval, lc_env_ptr->link.Role);
        }
    }

    lc_env_ptr->link.CurrentMode = LM_SNIFF_MODE;

#if (EAVESDROPPING_SUPPORT)
    // Sniff mode change
    lc_send_sniff_mode_change_evt(link_id, SNIFF);
#endif // EAVESDROPPING_SUPPORT
    // Register sniff link in the planner
    lc_sniff_env[link_id].plan_elt.offset       = lc_offset_lmp_to_local(2 * offset, 2 * interval, ld_read_clock(), ld_acl_clock_offset_get(link_id).hs, ((lc_sniff_env[link_id].sniff.timing_flag & INIT2_FLAG) != 0));
    lc_sniff_env[link_id].plan_elt.interval     = 2 * interval;
    lc_sniff_env[link_id].plan_elt.duration_min = co_min((4 * lc_sniff_env[link_id].sniff.attempt) / 2, 4);
    lc_sniff_env[link_id].plan_elt.duration_max = 4 * lc_sniff_env[link_id].sniff.attempt;
    lc_sniff_env[link_id].plan_elt.margin       = 1 * (lc_env_ptr->link.Role == SLAVE_ROLE);
    lc_sniff_env[link_id].plan_elt.conhdl       = BT_ACL_CONHDL_MIN + link_id;
    lc_sniff_env[link_id].plan_elt.conhdl_ref   = lc_sniff_env[link_id].plan_elt.conhdl;
    lc_sniff_env[link_id].plan_elt.cb_move      = NULL;
    lc_sniff_env[link_id].plan_elt.mobility     = SCH_PLAN_MB_LVL_0;
    sch_plan_set(&lc_sniff_env[link_id].plan_elt);

    if (lc_sniff_env[link_id].ssr.max_lat)
    {
        ke_msg_send_basic(LC_OP_SSRNEGO_IND, pid, pid);
    }

    {
        // send mode change event
        uint16_t conhdl = BT_ACL_CONHDL_MIN + link_id;
        struct hci_mode_chg_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_MODE_CHG_EVT_CODE, hci_mode_chg_evt);
        evt->status = CO_ERROR_NO_ERROR;
        evt->conhdl = conhdl;
        evt->interv = interval;
        evt->cur_mode = lc_env_ptr->link.CurrentMode;
        hci_send_2_host(evt);
    }

    ke_state_set(pid, LC_CONNECTED);
}

/**
 ****************************************************************************************
 * @brief Exit sniff mode
 * @param[in] pid        process identifier.
 ****************************************************************************************
 */
__STATIC void lc_sniff_exit(ke_task_id_t pid)
{
    int link_id = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    ld_acl_unsniff(link_id);

    lc_env_ptr->link.CurrentMode = LM_ACTIVE_MODE;

    // Unregister sniff link from the planner
    sch_plan_rem(&lc_sniff_env[link_id].plan_elt);

    lc_restore_to(pid);
    ke_state_set(pid, LC_CONNECTED);

    memset(&lc_sniff_env[link_id].sniff, 0, sizeof(lc_sniff_env[link_id].sniff));

#if (EAVESDROPPING_SUPPORT)
    // Sniff mode change
    lc_send_sniff_mode_change_evt(link_id, ACTIVE);
#endif // EAVESDROPPING_SUPPORT

    {
        uint16_t conhdl = BT_ACL_CONHDL_MIN + link_id;
        struct hci_mode_chg_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_MODE_CHG_EVT_CODE, hci_mode_chg_evt);
        evt->status   = CO_ERROR_NO_ERROR;
        evt->conhdl   = conhdl;
        evt->interv   = 0;
        evt->cur_mode = lc_env_ptr->link.CurrentMode;
        // send the mode change event
        hci_send_2_host(evt);
    }
}

/**
 ****************************************************************************************
 * @brief Sniff negotiation fail
 * @param[in] pid        process identifier.
 ****************************************************************************************
 */
__STATIC void lc_sniff_nego_fail(ke_task_id_t pid, uint8_t status)
{
    int link_id = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    if (lc_env_ptr->link.Role == MASTER_ROLE)
    {
        ld_acl_unsniff(link_id);
    }

    if (lc_sniff_env[link_id].sniff.initiator)
    {
        uint16_t conhdl = BT_ACL_CONHDL_MIN + link_id;
        struct hci_mode_chg_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_MODE_CHG_EVT_CODE, hci_mode_chg_evt);
        evt->status   = status;
        evt->conhdl   = conhdl;
        evt->interv   = 0;
        evt->cur_mode = lc_env_ptr->link.CurrentMode;
        hci_send_2_host(evt);

        lc_sniff_env[link_id].sniff.initiator = false;
    }

    ke_state_set(pid, LC_CONNECTED);
}

/**
 ****************************************************************************************
 * @brief Enter sniff sub rating mode
 * @param[in] link_id        Link identifier.
 ****************************************************************************************
 */
__STATIC void lc_sniff_ssr_enter(ke_task_id_t pid)
{
    int link_id = KE_IDX_GET(pid);
    uint8_t sub_rate;
    uint16_t min_loc_to;

    sub_rate = lc_sniff_subrate_compute(lc_sniff_env[link_id].ssr.LocalSniffSubRate, lc_sniff_env[link_id].ssr.RemoteSniffSubRate);
    min_loc_to = co_max(lc_sniff_env[link_id].ssr.min_loc_to_hci, lc_sniff_env[link_id].ssr.min_loc_to_peer);

    ld_acl_ssr_set(link_id, sub_rate, min_loc_to, lc_sniff_env[link_id].ssr.ssr_instant);

    lc_sniff_env[link_id].ssr.min_loc_to = min_loc_to;

    // Wait for SSR instant
    ke_state_set(pid, LC_WAIT_SSR_INSTANT);
}


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void lc_sniff_init(bool reset)
{
    memset(&lc_sniff_env, 0, sizeof(lc_sniff_env));
}

void lc_sniff_clear(uint8_t link_id)
{
    sch_plan_rem(&lc_sniff_env[link_id].plan_elt);

    memset(&lc_sniff_env[link_id], 0, sizeof(lc_sniff_env[link_id]));
}

void lc_sniff_interval_get(uint8_t link_id, uint16_t *interval, uint8_t *max_subrate)
{
    *interval = lc_sniff_env[link_id].sniff.intv;

    if (max_subrate != NULL)
    {
        *max_subrate = co_max(lc_sniff_env[link_id].ssr.LocalSniffSubRate, lc_sniff_env[link_id].ssr.RemoteSniffSubRate);
    }
}

void lc_sniff_offset_update(uint8_t link_id, uint16_t sniff_offset)
{
    ASSERT_ERR(sniff_offset < lc_sniff_env[link_id].plan_elt.interval);

    // Check if sniff offset has changed
    if (sniff_offset != lc_sniff_env[link_id].plan_elt.offset)
    {
        // Update planner element
        sch_plan_shift(BT_ACL_CONHDL_MIN + link_id, sniff_offset - lc_sniff_env[link_id].plan_elt.offset);
    }
}

void lc_sniff_host_request(ke_task_id_t pid, uint16_t max_int, uint16_t min_int, uint16_t attempt, uint16_t timeout)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    int link_id = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    // Input parameters
    struct sch_plan_req_param req_param;
    uint32_t clk_off = ld_acl_clock_offset_get(link_id).hs;
    uint32_t clk = ld_read_clock();

    lc_sniff_env[link_id].sniff.attempt = attempt;
    lc_sniff_env[link_id].sniff.to = timeout;

    // Choose Timing Flag, according to master clock MSB (bit 27 of the BT clock)
    lc_sniff_env[link_id].sniff.timing_flag = 0;
    if ((CLK_ADD_2(ld_read_clock(), clk_off) & BT_CLOCK_MSB) != 0)
    {
        lc_sniff_env[link_id].sniff.timing_flag = INIT2_FLAG;
    }

    // Compute the scheduling parameters
    req_param.interval_min = min_int * 2;
    req_param.interval_max = max_int * 2;
    req_param.duration_min = co_min((4 * attempt) / 2, 4); // Half sniff attempts
    req_param.duration_max = 4 * attempt;
    req_param.offset_min   = 0;
    req_param.offset_max   = req_param.interval_max - 1;
    req_param.pref_period  = 0;
    req_param.conhdl       = BT_ACL_CONHDL_MIN + link_id;
    req_param.conhdl_ref   = req_param.conhdl;
    req_param.margin       = 1 * (lc_env_ptr->link.Role == SLAVE_ROLE);

    // SAM Support
    // The device should align sniff with SAM if possible by choosing sniff period to be an integer multiple of TSAM
    if ((lc_env_ptr->sam_info.loc_idx != SAM_DISABLED) || (lc_env_ptr->sam_info.rem_idx != SAM_DISABLED))
    {
        uint16_t intv = 2 * lc_sam_intv_get(link_id);

        if (intv < req_param.interval_min)
        {
            intv = ((req_param.interval_min + intv - 1) / intv) * intv;
        }

        if (intv <= req_param.interval_max)
        {
            req_param.interval_min = intv;
            req_param.interval_max = intv;
        }
    }

    if (sch_plan_req(&req_param) != SCH_PLAN_ERROR_OK)
    {
        status = CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE;
    }
    else
    {
        // Select an even master slot from the minimum offset
        uint16_t offset = CO_ALIGN4_LO(req_param.offset_min) + ((4 - (clk_off & 0x03)) & 0x03);
        lc_sniff_env[link_id].sniff.offset = lc_offset_local_to_lmp(offset, req_param.interval, clk, clk_off, ((lc_sniff_env[link_id].sniff.timing_flag & INIT2_FLAG) != 0)) / 2;
        lc_sniff_env[link_id].sniff.intv = req_param.interval / 2;

        // SAM Support
        // The device should align sniff with SAM if possible by choosing sniff offset to be an available master-to-slave slot
        if ((lc_env_ptr->sam_info.loc_idx != SAM_DISABLED) || (lc_env_ptr->sam_info.rem_idx != SAM_DISABLED))
        {
            uint16_t intv = 2 * lc_sam_intv_get(link_id);

            // Determine suitable offset if sniff period is an integer multiple of TSAM
            if (CO_MOD(req_param.interval, intv) == 0)
            {
                // check for an available slot in a contiguous range from offset_min circular to an offset_max.
                uint16_t offset_min = lc_sniff_env[link_id].sniff.offset;
                uint16_t offset_max = offset_min + CO_MOD(req_param.interval + req_param.offset_max - req_param.offset_min, req_param.interval) / 2;
                lc_sniff_env[link_id].sniff.offset = CO_MOD(lc_sam_slot_av_get(link_id, offset_min, offset_max), req_param.interval / 2);
            }
        }
    }

    if (status == CO_ERROR_NO_ERROR)
    {
        lc_sniff_env[link_id].sniff.initiator = true;

        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            lc_util_set_loc_trans_coll(link_id, LMP_SNIFF_REQ_OPCODE, LC_UTIL_NOT_USED, LC_UTIL_INUSED);

            // Enter sniff transition mode
            ld_acl_sniff_trans(link_id, lc_sniff_env[link_id].sniff.offset, lc_sniff_env[link_id].sniff.intv,
                               lc_sniff_env[link_id].sniff.attempt, lc_sniff_env[link_id].sniff.to,
                               ((lc_sniff_env[link_id].sniff.timing_flag & INIT2_FLAG) != 0));

        }

        // send LMP_sniffreq
        lc_send_pdu_sniff_req(link_id);

        lc_start_lmp_to(pid);
        ke_state_set(pid, LC_WAIT_SNIFF_REQ);

#if (EAVESDROPPING_SUPPORT)
        // Sniff mode change
        lc_send_sniff_mode_change_evt(link_id, SNIFF_PENDING);
#endif // EAVESDROPPING_SUPPORT
    }
    else
    {
        // Report the procedure completion
        uint16_t conhdl = BT_ACL_CONHDL_MIN + link_id;
        struct hci_mode_chg_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_MODE_CHG_EVT_CODE, hci_mode_chg_evt);
        evt->status = status;
        evt->conhdl = conhdl;
        evt->interv = 0;
        evt->cur_mode = lc_env_ptr->link.CurrentMode;
        hci_send_2_host(evt);
    }
}

void lc_sniff_peer_request(ke_task_id_t pid, uint8_t flags, uint16_t d_sniff, uint16_t t_sniff, uint16_t sniff_attempt, uint16_t sniff_to)
{
    uint8_t action = LC_SNIFF_ACCEPT;
    int link_id = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    ke_timer_clear(LC_LMP_RSP_TO, pid);

    do
    {
        // Input parameters
        struct sch_plan_chk_param chk_param;
        uint16_t offset;
        uint32_t clk_off = ld_acl_clock_offset_get(link_id).hs;
        uint32_t clk = ld_read_clock();

        // Check parameters validity
        if ((t_sniff < SNIFF_INTERVAL_MIN)
                || (t_sniff > SNIFF_INTERVAL_MAX)
                || ((t_sniff & 0x01) == 0x01)
                || ((d_sniff & 0x01) == 0x01)
                || (d_sniff >= t_sniff)
                || (sniff_attempt == 0)
                || (sniff_attempt > (t_sniff / 2))
                || (sniff_to > SNIFF_TIMEOUT_MAX)
                || ((uint32_t)(2 * (sniff_to + sniff_attempt)) >= t_sniff))
        {
            // send LMP_NotAccepted(LMP_SNIFF_REQ_OPCODE)
            lc_send_pdu_not_acc(link_id, LMP_SNIFF_REQ_OPCODE, CO_ERROR_INVALID_HCI_PARAM, lc_env_ptr->link.RxTrIdServer);

            break;
        }


        if (ke_state_get(pid) == LC_CONNECTED)
        {
            lc_sniff_env[link_id].sniff.initiator = false;
        }

        // Compute the scheduling parameters
        chk_param.interval     = t_sniff * 2;
        chk_param.duration_min = co_min((4 * sniff_attempt) / 2, 4);
        chk_param.conhdl       = BT_ACL_CONHDL_MIN + link_id;
        chk_param.conhdl_ref   = chk_param.conhdl;
        chk_param.offset       = lc_offset_lmp_to_local(2 * d_sniff, 2 * t_sniff, clk, clk_off, ((flags & INIT2_FLAG) != 0));
        chk_param.margin       = 1 * (lc_env_ptr->link.Role == SLAVE_ROLE);

        offset = chk_param.offset;

        // Check offset proposed by peer
        if (sch_plan_chk(&chk_param) != SCH_PLAN_ERROR_OK)
        {
            action = LC_SNIFF_REJECT;

            // If local device did not send any request yet
            if (ke_state_get(pid) == LC_CONNECTED)
            {
                struct sch_plan_req_param req_param;
                req_param.interval_min = t_sniff * 2;
                req_param.interval_max = t_sniff * 2;
                req_param.duration_min = co_min((4 * sniff_attempt) / 2, 4); // Half sniff attempts
                req_param.duration_max = 4 * sniff_attempt;
                req_param.offset_min   = 0;
                req_param.offset_max   = req_param.interval_max - 1;
                req_param.pref_period  = 0;
                req_param.conhdl       = BT_ACL_CONHDL_MIN + link_id;
                req_param.conhdl_ref   = req_param.conhdl;
                req_param.margin       = 1 * (lc_env_ptr->link.Role == SLAVE_ROLE);

                // Try to find and propose an alternative offset
                if (sch_plan_req(&req_param) == SCH_PLAN_ERROR_OK)
                {
                    offset = CO_ALIGN4_LO(req_param.offset_min) + ((4 - (clk_off & 0x03)) & 0x03);
                    action = LC_SNIFF_REQUEST;

                    // SAM Support
                    // The device should align sniff with SAM if possible by choosing sniff offset to be an available master-to-slave slot
                    if ((lc_env_ptr->sam_info.loc_idx != SAM_DISABLED) || (lc_env_ptr->sam_info.rem_idx != SAM_DISABLED))
                    {
                        uint16_t intv = 2 * lc_sam_intv_get(link_id);

                        // Determine suitable offset if sniff period is an integer multiple of TSAM
                        if (CO_MOD(t_sniff * 2, intv) == 0)
                        {
                            // check for an available slot in a contiguous range from offset_min circular to an offset_max.
                            uint16_t d_sniff_min = (lc_env_ptr->link.Role == MASTER_ROLE) ? offset / 2 : CO_MOD(clk_off + offset, t_sniff * 2) / 2;
                            uint16_t d_sniff_max = d_sniff_min + CO_MOD(t_sniff * 2 + req_param.offset_max - req_param.offset_min, req_param.interval) / 2;
                            d_sniff = CO_MOD(lc_sam_slot_av_get(link_id, d_sniff_min, d_sniff_max), t_sniff);
                            offset = (lc_env_ptr->link.Role == MASTER_ROLE) ? 2 * offset : 2 * CO_MOD(offset + t_sniff - CO_MOD(clk_off / 2, t_sniff), t_sniff);
                        }
                    }
                }
            }
        }

        // Store the parameters
        lc_sniff_env[link_id].sniff.offset      = lc_offset_local_to_lmp(offset, 2 * t_sniff, clk, clk_off, ((flags & INIT2_FLAG) != 0)) / 2;
        lc_sniff_env[link_id].sniff.attempt     = sniff_attempt;
        lc_sniff_env[link_id].sniff.intv        = t_sniff;
        lc_sniff_env[link_id].sniff.to          = sniff_to;
        lc_sniff_env[link_id].sniff.timing_flag = flags;

#if (EAVESDROPPING_SUPPORT)
        if (action != LC_SNIFF_REJECT)
        {
            // Sniff mode change
            lc_send_sniff_mode_change_evt(link_id, SNIFF_PENDING);
        }
#endif // EAVESDROPPING_SUPPORT

        // Decide response to the peer
        switch (action)
        {
        case LC_SNIFF_ACCEPT:
        {
            // Accept sniff request
            if (lc_env_ptr->link.Role == MASTER_ROLE)
            {
                // Enter sniff transition mode
                ld_acl_sniff_trans(link_id, d_sniff, t_sniff, sniff_attempt, sniff_to, ((flags & INIT2_FLAG) != 0));
            }
            // LMP_Accepted (LMP_SNIFF_REQ_OPCODE)
            lc_send_pdu_acc(link_id, LMP_SNIFF_REQ_OPCODE, lc_env_ptr->link.RxTrIdServer);

            ke_state_set(pid, LC_WAIT_SNIFF_ACC_TX_CFM);
        }
        break;
        case LC_SNIFF_REQUEST:
        {
            if (lc_env_ptr->link.Role == MASTER_ROLE)
            {
                // Enter sniff transition mode
                ld_acl_sniff_trans(link_id, d_sniff, t_sniff, sniff_attempt, sniff_to, ((flags & INIT2_FLAG) != 0));
            }

            // send LMP_sniffreq
            lc_send_pdu_sniff_req(link_id);

            lc_start_lmp_to(pid);

            ke_state_set(pid, LC_WAIT_SNIFF_REQ);
        }
        break;
        case LC_SNIFF_REJECT:
        {
            // send LMP_NotAccepted(LMP_SNIFF_REQ_OPCODE)
            lc_send_pdu_not_acc(link_id, LMP_SNIFF_REQ_OPCODE, CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE, lc_env_ptr->link.RxTrIdServer);

            lc_sniff_nego_fail(pid, CO_ERROR_UNSUPPORTED_LMP_PARAM_VALUE);
        }
        break;
        default:
            ASSERT_ERR_FORCE(0);
            break;
        }

    }
    while (0);

}

void lc_sniff_peer_accept(ke_task_id_t pid)
{
    ke_timer_clear(LC_LMP_RSP_TO, pid);
    lc_sniff_enter(pid);
}

void lc_sniff_peer_reject(ke_task_id_t pid, uint8_t reason)
{
#if (EAVESDROPPING_SUPPORT)
    int link_id = KE_IDX_GET(pid);
    // Sniff mode change
    lc_send_sniff_mode_change_evt(link_id, ACTIVE);
#endif // EAVESDROPPING_SUPPORT

    ke_timer_clear(LC_LMP_RSP_TO, pid);

    lc_sniff_nego_fail(pid, reason);
}

void lc_sniff_baseband_ack(ke_task_id_t pid, uint8_t lmp_opcode)
{
    ke_timer_clear(LC_LMP_RSP_TO, pid);

    lc_sniff_enter(pid);
}

void lc_sniff_timeout(ke_task_id_t pid)
{
    lc_sniff_nego_fail(pid, CO_ERROR_LMP_RSP_TIMEOUT);
}

void lc_sniff_unsniff(ke_task_id_t pid)
{
    int link_id = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

#if (EAVESDROPPING_SUPPORT)
    // Sniff mode change
    lc_send_sniff_mode_change_evt(link_id, UNSNIFF_PENDING);
#endif // EAVESDROPPING_SUPPORT

    // send LMP_UnsniffReq(link_id,role)
    lc_send_pdu_unsniff_req(link_id, lc_env_ptr->link.Role);

    if (lc_env_ptr->link.Role == MASTER_ROLE)
    {
        // Enter sniff transition mode (from sniff mode)
        ld_acl_sniff_trans(link_id, 0, 0, 0, 0, 0);
    }

    lc_start_lmp_to(pid);

    ke_state_set(pid, LC_WAIT_UNSNIFF_ACC);
}

void lc_sniff_unsniff_peer_request(ke_task_id_t pid)
{
    int link_id = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

#if (EAVESDROPPING_SUPPORT)
    // Sniff mode change
    lc_send_sniff_mode_change_evt(link_id, UNSNIFF_PENDING);
#endif // EAVESDROPPING_SUPPORT

    if ((lc_env_ptr->link.Role == MASTER_ROLE) && (ke_state_get(pid) == LC_WAIT_UNSNIFF_ACC))
    {
        // send LMP_NotAccepted(LMP_UNSNIFF_REQ_OPCODE)
        lc_send_pdu_not_acc(link_id, LMP_UNSNIFF_REQ_OPCODE, CO_ERROR_LMP_COLLISION, lc_env_ptr->link.RxTrIdServer);
    }
    else
    {
        // Accept the unsniff request
        lc_send_pdu_acc(link_id, LMP_UNSNIFF_REQ_OPCODE, !lc_env_ptr->link.Role);
    }

    switch (ke_state_get(pid))
    {
    case LC_WAIT_UNSNIFF_ACC:
    {
        /*
         * Collision with local unsniff procedure:
         *    - If slave, exit sniff and closes the procedure
         *    - If master, stay in sniff transition mode, waits for LMP accepted to complete the procedure
         */
        if (lc_env_ptr->link.Role == SLAVE_ROLE)
        {
            lc_sniff_exit(pid);
        }
    }
    break;

    case LC_CONNECTED:
    case LC_WAIT_SSR_INSTANT:
    {
        if (lc_env_ptr->link.CurrentMode == LM_SNIFF_MODE)
        {
            if (lc_env_ptr->link.Role == SLAVE_ROLE)
            {
                lc_sniff_exit(pid);
            }
            else
            {
                // Enter sniff transition mode (from sniff mode)
                ld_acl_sniff_trans(link_id, 0, 0, 0, 0, 0);

                lc_start_lmp_to(pid);
                ke_state_set(pid, LC_WAIT_UNSNIFF_ACC_TX_CFM);
            }
        }
    }
    break;

    default:
        ASSERT_ERR_FORCE(0);
        break;
    }
}

void lc_sniff_unsniff_peer_accept(ke_task_id_t pid)
{
    ke_timer_clear(LC_LMP_RSP_TO, pid);

    lc_sniff_exit(pid);
}

void lc_sniff_unsniff_peer_reject(ke_task_id_t pid, uint8_t reason)
{
    int link_id = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

#if (EAVESDROPPING_SUPPORT)
    // Sniff mode change
    lc_send_sniff_mode_change_evt(link_id, SNIFF);
#endif // EAVESDROPPING_SUPPORT

    ke_timer_clear(LC_LMP_RSP_TO, pid);
    ke_state_set(pid, LC_CONNECTED);

    {
        uint16_t conhdl = BT_ACL_CONHDL_MIN + link_id;
        struct hci_mode_chg_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_MODE_CHG_EVT_CODE, hci_mode_chg_evt);
        evt->status   = reason;
        evt->conhdl   = conhdl;
        evt->interv   = 0;
        evt->cur_mode = lc_env_ptr->link.CurrentMode;
        hci_send_2_host(evt);
    }
}

void lc_sniff_ssr_nego(ke_task_id_t pid)
{
    int link_id = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    /*
     * Get the shortest value between the maximum allowed latency and the link
     * supervision TO, except if the link timeout is infinite
     */
    lc_sniff_subrate_rem_compute(lc_env_ptr->link.LinkTimeout,
                                 &lc_sniff_env[link_id].ssr.RemoteSniffSubRate,
                                 lc_sniff_env[link_id].ssr.max_lat,
                                 lc_sniff_env[link_id].sniff.intv,
                                 lc_sniff_env[link_id].sniff.attempt);

    // Master chooses the SSR instant
    if (lc_env_ptr->link.Role == MASTER_ROLE)
    {
        // Compute Sniff sub rating instant
        uint32_t master_clock = ld_read_clock() >> 1;
        // TODO: use planner offset
        int32_t diff = lc_sniff_env[link_id].sniff.offset - CO_MOD((master_clock & (~(1 << 26))), lc_sniff_env[link_id].sniff.intv);
        uint32_t Temp = 0, Temp1 = 0;

        if (diff < 0)
        {
            diff += lc_sniff_env[link_id].sniff.intv;
        }

        Temp = SSR_MIN_BB_EXCHG / lc_sniff_env[link_id].sniff.attempt + 1;
        Temp1 = SSR_MIN_OFFSET / lc_sniff_env[link_id].sniff.intv;
        Temp = (Temp >= Temp1) ? Temp : (Temp1 + 1);

        lc_sniff_env[link_id].ssr.ssr_instant = CLK_ADD_3(2 * master_clock, 2 * diff, 2 * Temp * lc_sniff_env[link_id].sniff.intv) >> 1;
    }
    else
    {
        lc_sniff_env[link_id].ssr.ssr_instant = 0;
    }

    {
        struct lmp_ssr_req pdu;

        pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->link.Role);
        pdu.ext_opcode = LMP_SSR_REQ_EXTOPCODE;
        pdu.max_subrate = lc_sniff_env[link_id].ssr.RemoteSniffSubRate;
        pdu.instant = lc_sniff_env[link_id].ssr.ssr_instant;
        pdu.min_to = lc_sniff_env[link_id].ssr.min_rem_to;

        lc_send_lmp(link_id, &pdu);
        // start the LMP timer
        lc_start_lmp_to(pid);

        ke_state_set(pid, LC_WAIT_SNIFF_SUB_RSP);

        if (lc_env_ptr->link.Role == MASTER_ROLE)
        {
            lc_util_set_loc_trans_coll(link_id, LMP_ESC4_OPCODE, LMP_SSR_REQ_EXTOPCODE, LC_UTIL_INUSED);
        }
    }
}

void lc_sniff_ssr_host_request(ke_task_id_t pid, uint16_t max_lat, uint16_t min_rem_to, uint16_t min_loc_to)
{
    int link_id = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    lc_sniff_env[link_id].ssr.min_loc_to_hci = min_loc_to;

    if (lc_env_ptr->link.CurrentMode == LM_SNIFF_MODE)
    {
        // Check if remote parameters have changed
        if (lc_sniff_env[link_id].ssr.max_lat != max_lat || lc_sniff_env[link_id].ssr.min_rem_to != min_rem_to)
        {
            lc_sniff_env[link_id].ssr.max_lat = max_lat;
            lc_sniff_env[link_id].ssr.min_rem_to = min_rem_to;

            ke_msg_send_basic(LC_OP_SSRNEGO_IND, pid, pid);
        }
    }
    else
    {
        // Store Host parameters without starting negotiation
        lc_sniff_env[link_id].ssr.max_lat = max_lat;
        lc_sniff_env[link_id].ssr.min_rem_to = min_rem_to;
    }
}

void lc_sniff_ssr_peer_request(ke_task_id_t pid, uint8_t max_subrate, uint16_t min_to, uint32_t instant)
{
    int link_id = KE_IDX_GET(pid);
    struct lc_env_tag *lc_env_ptr = lc_env[link_id];

    lc_sniff_env[link_id].ssr.LocalSniffSubRate = max_subrate;
    lc_sniff_env[link_id].ssr.min_loc_to_peer = min_to;

    /*
     * Get the shortest value between the maximum allowed latency and the link
     * supervision TO, except if the link timeout is infinite
     */
    lc_sniff_subrate_rem_compute(lc_env_ptr->link.LinkTimeout,
                                 &lc_sniff_env[link_id].ssr.RemoteSniffSubRate,
                                 lc_sniff_env[link_id].ssr.max_lat,
                                 lc_sniff_env[link_id].sniff.intv,
                                 lc_sniff_env[link_id].sniff.attempt);

    if (lc_env_ptr->link.Role == MASTER_ROLE)
    {
        // Compute Sniff sub rating instant
        uint32_t master_clock = ld_read_clock() >> 1;
        // TODO: use planner offset
        int32_t diff = lc_sniff_env[link_id].sniff.offset - CO_MOD((master_clock & (~(1 << 26))), lc_sniff_env[link_id].sniff.intv);
        uint32_t Temp = 0, Temp1 = 0;

        if (diff < 0)
        {
            diff += lc_sniff_env[link_id].sniff.intv;
        }

        Temp = SSR_MIN_BB_EXCHG / lc_sniff_env[link_id].sniff.attempt + 1;
        Temp1 = SSR_MIN_OFFSET / lc_sniff_env[link_id].sniff.intv;
        Temp = (Temp >= Temp1) ? Temp : (Temp1 + 1);

        lc_sniff_env[link_id].ssr.ssr_instant = CLK_ADD_3(2 * master_clock, 2 * diff, 2 * Temp * lc_sniff_env[link_id].sniff.intv) >> 1;
    }
    /* Slave sends Sniff SubRate Instant from master or 0 if not yet received   */
    else
    {
        lc_sniff_env[link_id].ssr.ssr_instant = instant;
    }

    {
        struct lmp_ssr_res pdu;
        pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->link.RxTrIdServer);
        pdu.ext_opcode = LMP_SSR_RES_EXTOPCODE;
        pdu.max_subrate = lc_sniff_env[link_id].ssr.RemoteSniffSubRate;
        pdu.instant = lc_sniff_env[link_id].ssr.ssr_instant;
        pdu.min_to = lc_sniff_env[link_id].ssr.min_rem_to;
        lc_send_lmp(link_id, &pdu);

        lc_start_lmp_to(pid);
        ke_state_set(pid, LC_WAIT_SNIFF_SUB_RSP_TX_CFM);
    }
}

void lc_sniff_ssr_peer_response(ke_task_id_t pid, uint8_t max_subrate, uint16_t min_to, uint32_t instant)
{
    int link_id = KE_IDX_GET(pid);

    lc_sniff_env[link_id].ssr.LocalSniffSubRate = max_subrate;
    lc_sniff_env[link_id].ssr.min_loc_to_peer = min_to;
    lc_sniff_env[link_id].ssr.ssr_instant = instant;

    // Enter sniff subrating mode
    lc_sniff_ssr_enter(pid);

    // Wait for SSR instant
    ke_state_set(pid, LC_WAIT_SSR_INSTANT);
}

void lc_sniff_ssr_peer_reject(ke_task_id_t pid, uint8_t reason)
{
    int link_id = KE_IDX_GET(pid);

    if (reason != CO_ERROR_LMP_COLLISION)
    {
        // Send sniff-subrating event
        uint16_t conhdl = BT_ACL_CONHDL_MIN + link_id;
        struct hci_sniff_sub_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_SNIFF_SUB_EVT_CODE, hci_sniff_sub_evt);
        evt->status     = reason;
        evt->conhdl     = conhdl;
        evt->max_lat_rx = 0;
        evt->max_lat_tx = 0;
        evt->min_loc_to = 0;
        evt->min_rem_to = 0;
        hci_send_2_host(evt);
    }
    ke_state_set(pid, LC_CONNECTED);
}

void lc_sniff_ssr_baseband_ack(ke_task_id_t pid)
{
    // Enter sniff subrating mode
    lc_sniff_ssr_enter(pid);

    // Wait for SSR instant
    ke_state_set(pid, LC_WAIT_SSR_INSTANT);
}

void lc_sniff_ssr_instant(ke_task_id_t pid)
{
    int link_id = KE_IDX_GET(pid);
    uint8_t sub_rate;
    uint16_t rx_lat;
    uint16_t tx_lat;

    sub_rate = lc_sniff_subrate_compute(lc_sniff_env[link_id].ssr.RemoteSniffSubRate, lc_sniff_env[link_id].ssr.LocalSniffSubRate);
    tx_lat = sub_rate * lc_sniff_env[link_id].sniff.intv;

    sub_rate = lc_sniff_subrate_compute(lc_sniff_env[link_id].ssr.LocalSniffSubRate, lc_sniff_env[link_id].ssr.RemoteSniffSubRate);
    rx_lat = sub_rate * lc_sniff_env[link_id].sniff.intv;

    // Report the sniff sub-rating to the Host
    uint16_t conhdl = BT_ACL_CONHDL_MIN + link_id;
    struct hci_sniff_sub_evt *evt = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_SNIFF_SUB_EVT_CODE, hci_sniff_sub_evt);
    evt->status = CO_ERROR_NO_ERROR;
    evt->conhdl = conhdl;
    evt->max_lat_rx = rx_lat;
    evt->max_lat_tx = tx_lat;
    evt->min_rem_to = lc_sniff_env[link_id].ssr.min_rem_to;
    evt->min_loc_to = lc_sniff_env[link_id].ssr.min_loc_to;
    hci_send_2_host(evt);

    ke_state_set(pid, LC_CONNECTED);
}

///@} LCSNIFF
