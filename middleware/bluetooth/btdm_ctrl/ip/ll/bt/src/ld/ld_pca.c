/**
****************************************************************************************
*
* @file ld_pca.c
*
* @brief LD PCA source code
*
* Copyright (C) RivieraWaves 2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LDPCA
 * @ingroup LD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"    // stack configuration

#if PCA_SUPPORT

#include <string.h>
#include <stdbool.h>
#include <stddef.h>

#include "co_math.h"
#include "co_utils.h"
#include "co_bt.h"

#include "arch.h"
#include "rwip.h"
#include "ke_mem.h"
#include "ke_timer.h"
#include "ke_event.h"

#include "ld.h"             // link driver API
#include "ld_util.h"        // link driver utilities
#include "ld_int.h"         // link driver internal
#include "ld_util.h"        // link driver utilities

#include "lm.h"
#include "lb.h"


#include "sch_arb.h"            // Scheduling Arbiter
#include "sch_alarm.h"          // Scheduling Alarm
#include "sch_prog.h"           // Scheduling Programmer

#include "reg_ipcore.h"         // IP core registers
#include "reg_btcore.h"         // BT core registers

#include "dbg.h"
#include "dbg_mwsgen.h"
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// PCA event duration minimum in slots
#define LD_PCA_EVT_PRE_ADJUST_DURATION_MIN   10
#define LD_PCA_EVT_PST_ADJUST_DURATION_MIN   4
#define LD_PCA_EVT_DURATION_MIN    (LD_PCA_EVT_PRE_ADJUST_DURATION_MIN + LD_PCA_EVT_PST_ADJUST_DURATION_MIN)

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LD piconet clock adjust environment structure
struct ld_pca_config_tag
{
    /// saved intcntl0 mask at start of procedure
    uint32_t intcntl0;
    /// saved intcntl1 mask at start of procedure
    uint32_t intcntl1;

#if RW_BT_MWS_COEX
    /// Timestamp of latest frame sync
    rwip_time_t frame_sync_ts;

    /// downlink intraslot Offset (-624us..+624us)
    int16_t downlink_us_offset;
    /// downlink Slot Offset (0..255 slots)
    uint8_t downlink_slot_offset;

    /// uplink intraslot Offset (-624us..+624us)
    int16_t uplink_us_offset;
    /// uplink Slot Offset (0..255 slots)
    uint8_t uplink_slot_offset;

    /// Target slot period (0..255 slots)
    uint8_t slot_period;

    /// MWS_Frame_Sync_Assert_Jitter (usecs)
    uint16_t jitter;

    /// Frame sync configured
    bool frame_sync_configured;
    /// Frame sync detected
    bool frame_sync_det;

    /// Target slot offset
    uint8_t target_slot_offset;
    /// Target us offset
    int16_t target_us_offset;
    /// Moment us offset
    int16_t moment_us_offset;

    /// PCA alignment requests enable
    bool pca_req_en;
#endif // RW_BT_MWS_COEX

    /// Target link_id
    uint8_t link_id;

#if RW_DEBUG
    /// Moment offset interrupt count
    uint16_t mtoffint_cnt;
#endif //RW_DEBUG

    /// PCA clock dragging active
    bool clk_drag_active;
};

struct ld_pca_env_tag
{
    /// Piconet clock adjust event
    struct sch_arb_elt_tag evt;
    /// Alarm for clk_adj_instant
    struct sch_alarm_tag pca_alarm;
    /// coarse clock adjust usecs
    int16_t clk_adj_us;
    /// coarse clock adjust slots
    uint8_t clk_adj_slots;
    /// clock adjust instant (in 625us BT slots)
    uint32_t clk_adj_instant;
};



/*
 * VARIABLES DEFINITIONS
 *****************************************************************************************
 */
/// PCA environment variable
__STATIC struct ld_pca_env_tag *ld_pca_env;
struct ld_pca_config_tag ld_pca_config;

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */
__STATIC void ld_pca_end_ind(uint8_t status);
__STATIC void ld_pca_evt_start_cbk(struct sch_arb_elt_tag *evt);
__STATIC void ld_pca_evt_canceled_cbk(struct sch_arb_elt_tag *evt);
__STATIC void ld_pca_clk_recover_isr(uint32_t clock, uint8_t id);

/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handle PCA clk_adj_instant alarm
 ****************************************************************************************
 */
__STATIC void ld_pca_instant_cbk(struct sch_alarm_tag *elt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_INSTANT_CBK_BIT, elt);

    DBG_SWDIAG(PCA, ALARM_INSTANT, 1);

    if (elt != NULL)
    {
        struct ld_pca_env_tag *pca_par = CONTAINER_OF(elt, struct ld_pca_env_tag, pca_alarm);

        int16_t clk_adj_us = pca_par->clk_adj_us;

        /* A positive clock_shift signifies the clock will be advanced forward relative to frame_sync,
         * while a negative clock_shift signifies the clock domain will be pulled back. This is equivalent
         * to the LMP clk_adj_us parameter, where a positive value indicates the clock advances. */
        if (clk_adj_us)
        {
            rwip_time_adj(clk_adj_us);
        }

        // Enable programming
        sch_prog_enable(&ld_pca_clk_recover_isr, ld_pca_config.link_id, 0, 0);

        // Save intcntl
        ld_pca_config.intcntl0 = bt_intcntl0_get();
        ld_pca_config.intcntl1 = ip_intcntl1_get();

        // Disable all interrupts except the IP_CLKINT interrupt and Error status
        bt_intcntl0_set(BT_ERRORINTMSK_BIT);
        ip_intcntl1_set(IP_CLKNINTMSK_BIT);
        ip_intack1_clear(IP_CLKNINTACK_BIT);
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(PCA, ALARM_INSTANT, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_pca_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_EVT_START_CBK_BIT, evt);

    DBG_SWDIAG(PCA, EVT_START, 1);

    if (evt != NULL)
    {
        // Here the reservation of slots precedes the clk_adj_instant - now set an alarm for the clk_adj_instant.
        // Point to parameters
        struct ld_pca_env_tag *pca_par = ld_pca_env;

        // Program clk_adj_instant via an alarm
        pca_par->pca_alarm.time.hs = CLK_SUB(2 * pca_par->clk_adj_instant, rwip_prog_delay);
        pca_par->pca_alarm.time.hus = 0;

        pca_par->pca_alarm.cb_alarm = &ld_pca_instant_cbk;

        sch_alarm_set(&pca_par->pca_alarm);
    }

    DBG_SWDIAG(PCA, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_pca_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_EVT_CANCELED_CBK_BIT, evt);

    DBG_SWDIAG(PCA, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        // Notify of error
        ld_pca_end_ind(CO_ERROR_UNSPECIFIED_ERROR);

        // Delete event
        ke_free(ld_pca_env);
        ld_pca_env = NULL;
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(PCA, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Indicates end of PCA event
 ****************************************************************************************
 */
__STATIC void ld_pca_end_ind(uint8_t status)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_END_IND_BIT, status);

    struct lb_pca_end_ind *msg = KE_MSG_ALLOC(LB_PCA_END_IND, TASK_LB, TASK_NONE, lb_pca_end_ind);
    msg->status = status;
    ke_msg_send(msg);
}

/**
 ****************************************************************************************
 * @brief Stop clock dragging
 ****************************************************************************************
 */
__STATIC void ld_pca_clk_drag_stop(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_CLK_DRAG_STOP_BIT);

    // disable incremental phase shift, as is now aligned with target offset
    bt_pcacntl0_phase_shift_en_setf(0);
    // disable blindcorr
    bt_pcacntl0_blindcorr_en_setf(0);

    // Disable MTOFFINT0/MTOFFINT1 interrupts
    bt_intcntl0_set(bt_intcntl0_get() & ~(BT_MTOFFINT0MSK_BIT | BT_MTOFFINT1MSK_BIT));
    bt_intack0_clear(BT_MTOFFINT0ACK_BIT | BT_MTOFFINT1ACK_BIT);

#if RW_BT_MWS_COEX
    if (ld_pca_config.frame_sync_configured)
    {
        // restore target offset of frame_sync
        bt_pcacntl0_target_offset_setf((ld_pca_config.target_us_offset) & 0x7FF);
    }
#endif // RW_BT_MWS_COEX

    // Indicate clock dragging is inactive
    ld_pca_config.clk_drag_active = false;
}

/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void ld_pca_init(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_INIT_BIT);

    memset(&ld_pca_config, 0, sizeof(ld_pca_config));
    ld_pca_env = NULL;

    bt_coexifcntl0_mwscoex_en_setf(0);
    bt_intcntl0_set(bt_intcntl0_get() & ~(BT_MTOFFINT0MSK_BIT | BT_MTOFFINT1MSK_BIT));
    bt_intack0_clear(BT_FRSYNCINTACK_BIT | BT_MTOFFINT0ACK_BIT | BT_MTOFFINT1ACK_BIT);

    bt_pcacntl0_pack(BT_TARGET_OFFSET_RST, BT_SLVLBL_RST, BT_CORR_STEP_RST, BT_BLINDCORR_EN_RST, BT_FRSYNC_POL_RST, BT_SYNC_SOURCE_RST, BT_PHASE_SHIFT_EN_RST);
    bt_pcacntl1_pack(BT_CORR_INTERVAL_RST, BT_CLOCK_SHIFT_EN_RST, BT_CLOCK_SHIFT_RST);
}

void ld_pca_reset(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_RESET_BIT);

    memset(&ld_pca_config, 0, sizeof(ld_pca_config));

    if (ld_pca_env != NULL)
    {
        // Delete event
        ke_free(ld_pca_env);
        ld_pca_env = NULL;
    }

    bt_coexifcntl0_mwscoex_en_setf(0);
    bt_intcntl0_set(bt_intcntl0_get() & ~(BT_MTOFFINT0MSK_BIT | BT_MTOFFINT1MSK_BIT));
    bt_intack0_clear(BT_FRSYNCINTACK_BIT | BT_MTOFFINT0ACK_BIT | BT_MTOFFINT1ACK_BIT);

    bt_pcacntl0_pack(BT_TARGET_OFFSET_RST, BT_SLVLBL_RST, BT_CORR_STEP_RST, BT_BLINDCORR_EN_RST, BT_FRSYNC_POL_RST, BT_SYNC_SOURCE_RST, BT_PHASE_SHIFT_EN_RST);
    bt_pcacntl1_pack(BT_CORR_INTERVAL_RST, BT_CLOCK_SHIFT_EN_RST, BT_CLOCK_SHIFT_RST);
}

#if RW_BT_MWS_COEX
/**
 ****************************************************************************************
 * @brief Configure offsets relative to frame_sync, and external frame duration.
 ****************************************************************************************
 */
void ld_pca_local_config(uint16_t downlink_us_offset, uint16_t uplink_us_offset, uint16_t period_duration, uint16_t jitter, uint8_t nb_acl)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_LOCAL_CONFIG_BIT, 5, downlink_us_offset, uplink_us_offset, period_duration, jitter, nb_acl);

    /* MWS downlink usec offset relative to alignment instant, Master to Slave slot transition within a [-625:+625]us range,
       and associated slot offset, calculated in number of complete frames. */
    ld_pca_config.downlink_us_offset  = (int16_t)CO_MOD(downlink_us_offset, (2 * SLOT_SIZE));
    if (ld_pca_config.downlink_us_offset > SLOT_SIZE)
        ld_pca_config.downlink_us_offset -= (2 * SLOT_SIZE);
    ld_pca_config.downlink_slot_offset = (2 * ((downlink_us_offset - ld_pca_config.downlink_us_offset) / (2 * SLOT_SIZE)));

    /* MWS uplink usec offset relative to alignment instant, Master to Slave slot transition within a [-625:+625]us range,
       and associated slot offset, calculated in number of complete frames. */
    ld_pca_config.uplink_us_offset  = (int16_t)CO_MOD(uplink_us_offset, (2 * SLOT_SIZE));
    if (ld_pca_config.uplink_us_offset > SLOT_SIZE)
        ld_pca_config.uplink_us_offset -= (2 * SLOT_SIZE);
    ld_pca_config.uplink_slot_offset = (2 * ((uplink_us_offset - ld_pca_config.uplink_us_offset) / (2 * SLOT_SIZE)));


    ld_pca_config.slot_period = period_duration / SLOT_SIZE;
    ld_pca_config.jitter = jitter;

    ld_pca_config.frame_sync_configured = true;

    if (nb_acl) // Use latest configured link_id
    {
        ld_pca_update_target_offset(ld_pca_config.link_id);
    }
    else // Use local/master alignment
    {
        ld_pca_config.target_slot_offset = ld_pca_config.uplink_slot_offset;
        ld_pca_config.target_us_offset = ld_pca_config.uplink_us_offset;

        bt_pcacntl0_target_offset_setf((ld_pca_config.target_us_offset) & 0x7FF);
    }

    // Enable COEX to drive PCA alignment
    bt_coexifcntl0_mwscoex_en_setf(1);
}

/**
 ****************************************************************************************
 * @brief re-enabe/disable clock adjust reporting/requests
 ****************************************************************************************
 */
void ld_pca_reporting_enable(bool enable)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_REPORTING_ENABLE_BIT, enable);

    ld_pca_config.pca_req_en = enable;
}

/**
 ****************************************************************************************
 * @brief Reconfigure target offset for MWS uplink/downlink based on new role
  ****************************************************************************************
 */
void ld_pca_update_target_offset(uint8_t link_id)
{
    uint8_t target_slot_offset = 0;
    int16_t target_us_offset = 0;

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_UPDATE_TARGET_OFFSET_BIT, link_id);

    if (ld_pca_config.frame_sync_configured)
    {
        if (SLAVE_ROLE == ld_acl_role_get(link_id))
        {
            // align clock on new slave role, regardless of existing master/slave roles
            rwip_time_t clk_offset = ld_acl_clock_offset_get(link_id);
            target_slot_offset = ld_pca_config.downlink_slot_offset;
            target_us_offset = SLOT_SIZE - (((clk_offset.hs & 3) ^ 1) * HALF_SLOT_SIZE + (clk_offset.hus)) / 2;
        }
        else //MASTER_ROLE
        {
            // align clock on new master role. clock dragging employed if existing slave roles
            target_slot_offset = ld_pca_config.uplink_slot_offset;
            target_us_offset = ld_pca_config.uplink_us_offset;
        }

        ld_pca_config.link_id = link_id;
        ld_pca_config.target_us_offset = target_us_offset;
        ld_pca_config.target_slot_offset = target_slot_offset;

        // write the target offset, cut sign extension to 11 bit register size
        bt_pcacntl0_target_offset_setf(target_us_offset & 0x7FF);
    }
    else
    {
        // The target offset will be configured for this link when frame_sync configured
        ld_pca_config.link_id = link_id;
    }
}

/**
 ****************************************************************************************
 * @brief Handle PCA MWS frame sync interrupt
 ****************************************************************************************
 */
void ld_pca_mws_frame_sync(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_MWS_FRAME_SYNC_BIT);

    DBG_SWDIAG(PCA, FRAME_SYNC_INT, 1);

    rwip_time_t frame_sync_ts = rwip_time_get();

    if (ld_pca_config.pca_req_en && !ld_pca_config.clk_drag_active)
    {
        int16_t moment_us_offset = (int16_t)bt_pcastat_moment_offset_getf();

#if BT_PCA_CLK_ADJ_SLOTS
        /* Nominate a moment_slot_offset based on local clock */
        uint8_t moment_slot_offset = (uint8_t)CO_MOD(((frame_sync_ts.hs >> 1) & 0xFFFFFFFE), ld_pca_config.slot_period);
#endif //BT_PCA_CLK_ADJ_SLOTS

        /* Extend sign on Moment_offset[10:0] */
        if (moment_us_offset & 0x0400)
            moment_us_offset |= 0xF800;

        ld_pca_config.frame_sync_det = true;

        // Determine if clock is aligned, within expected jitter
        bool clk_aligned = ((co_abs(moment_us_offset) <= ld_pca_config.jitter)
#if BT_PCA_CLK_ADJ_SLOTS
                            && (moment_slot_offset == ld_pca_config.target_slot_offset)
#endif //BT_PCA_CLK_ADJ_SLOTS
                           );

        if (!clk_aligned)
        {
            struct lb_local_pca_req *msg = KE_MSG_ALLOC(LB_LOCAL_PCA_REQ, TASK_LB, TASK_NONE, lb_local_pca_req);

#if BT_PCA_CLK_ADJ_SLOTS
            int16_t clk_adj_slots = ld_pca_config.target_slot_offset - moment_slot_offset;
#else //!BT_PCA_CLK_ADJ_SLOTS
            int16_t clk_adj_slots = 0;
#endif //BT_PCA_CLK_ADJ_SLOTS

            // Special case handling: 625us momentary offset reported from HW
            ASSERT_ERR(co_abs(moment_us_offset) <= SLOT_SIZE);
            if (co_abs(moment_us_offset) == SLOT_SIZE)
            {
                clk_adj_slots = (moment_us_offset == SLOT_SIZE) ? (clk_adj_slots + 1) : (clk_adj_slots - 1);
                moment_us_offset = 0;
            }

            // In BT, clk_adj_slots must be a positive value, so round up by slot_period if calculated as negative
            if (clk_adj_slots < 0)
            {
                clk_adj_slots += ld_pca_config.slot_period;
            }

            /*
             * A positive moment_us_offset value signifies the clock shall advance forward relative to frame_sync,
             * while a negative moment_us_value signifies the clock domain shall be pulled back. This has equivalence
             * with the LMP clk_adj_us parameter, where a positive value indicates the clock advances.
             */
            msg->clk_adj_us = moment_us_offset;
            msg->clk_adj_slots = (uint8_t)clk_adj_slots;
            msg->clk_adj_period = ld_pca_config.slot_period;

            ke_msg_send(msg);

            // Processing - so disable further PCA reporting until completed
            ld_pca_config.pca_req_en = false;
        }
    }

    // SAM Support - Notify of MWS pattern index changes
    // Wait for PCA alignment if Master
    if (ld_pca_config.pca_req_en)
    {
        ld_mwsifstat_t mwsifstat = ld_mws_pattern_index_get();

        if (mwsifstat.pattern_index != SAM_INDEX_CONTINUE)
        {
            ASSERT_ERR((mwsifstat.pattern_index < SAM_INDEX_MAX) || (mwsifstat.pattern_index == SAM_DISABLED));

            // Enable local SAM at top level if not already enabled
            bt_lsamcntl0_local_sam_en_setf(1);

            // Always inform LM: LM determines if pattern index update is applicable, as it manages the active pattern set.
            if ((mwsifstat.pattern_index < SAM_INDEX_MAX) || (mwsifstat.pattern_index == SAM_DISABLED))
            {
                struct lm_mws_pattern_ind *msg = KE_MSG_ALLOC(LM_MWS_PATTERN_IND, TASK_LM, TASK_NONE, lm_mws_pattern_ind);
                uint32_t bitoff_hus = ((ld_env.mws_ext_fr_duration - ld_env.mws_ext_fr_offset) << 1) + frame_sync_ts.hus;

                // activation time is at slot offset, aligned to mws frame.
                memset(msg, 0, sizeof(struct lm_mws_pattern_ind));
                msg->time.hs = (frame_sync_ts.hs + (bitoff_hus / HALF_SLOT_SIZE)) + (mwsifstat.slot_offset << 1);
                msg->time.hus = CO_MOD(bitoff_hus, HALF_SLOT_SIZE);
                msg->pattern_index = mwsifstat.pattern_index;

                ke_msg_send(msg);
            }
        }
    }

    ld_pca_config.frame_sync_ts =  frame_sync_ts;

#if (RW_MWS_COEX_TEST)
    dbg_mwscoex_frame_sync_handler();
#endif //(RW_MWS_COEX_TEST)

    DBG_SWDIAG(PCA, FRAME_SYNC_INT, 0);
}

/**
 ****************************************************************************************
 * @brief Returns timestamp of the start of the next expected  MWS frame
 ****************************************************************************************
 */
rwip_time_t ld_pca_ext_frame_ts_get(void)
{
    rwip_time_t time;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_EXT_FRAME_TS_GET_BIT);

    uint32_t bitoff_hus = ((ld_env.mws_ext_fr_duration - ld_env.mws_ext_fr_offset) << 1) + ld_pca_config.frame_sync_ts.hus;
    time.hs = (ld_pca_config.frame_sync_ts.hs + (bitoff_hus / HALF_SLOT_SIZE));
    time.hus = CO_MOD(bitoff_hus, HALF_SLOT_SIZE);

    return time;
}

#endif // RW_BT_MWS_COEX

/**
 ****************************************************************************************
 * @brief Handle PCA MWS momentary offset 'greater than correction step' interrupt
 ****************************************************************************************
 */
void ld_pca_mws_moment_offset_gt(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_MWS_MOMENT_OFFSET_GT_BIT);

    DBG_SWDIAG(PCA, CLK_DRAG, 1);

#if RW_BT_MWS_COEX
    if (ld_pca_config.frame_sync_det)
    {
        int16_t moment_us_offset = (int16_t)bt_pcastat_moment_offset_getf();

        /* Extend sign on Moment_offset[10:0] */
        if (moment_us_offset & 0x0400)
            moment_us_offset |= 0xF800;

        if (((co_abs(moment_us_offset) + ld_pca_config.jitter) > co_abs(ld_pca_config.moment_us_offset))
                && (co_abs(moment_us_offset) > BT_PCA_MIN_CC_ADJ_US))
        {
            /* The frame sync reference has moved significantly while dragging - cancel dragging, and permit coarse clock adjustment */
            ld_pca_clk_drag_stop();
            ld_pca_config.pca_req_en = true;
        }
    }
#endif // RW_BT_MWS_COEX

#if RW_DEBUG
    ld_pca_config.mtoffint_cnt++;
#endif //RW_DEBUG

    DBG_SWDIAG(PCA, CLK_DRAG, 0);
}

/**
 ****************************************************************************************
 * @brief Handle PCA MWS momentary offset 'less than correction step' interrupt
 ****************************************************************************************
 */
void ld_pca_mws_moment_offset_lt(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_MWS_MOMENT_OFFSET_LT_BIT);

    DBG_SWDIAG(PCA, MTOFF_LT_INT, 1);

    // Stop clock dragging
    ld_pca_clk_drag_stop();

#if RW_BT_MWS_COEX
    ld_pca_config.pca_req_en = true;
#endif // RW_BT_MWS_COEX

    DBG_SWDIAG(PCA, MTOFF_LT_INT, 0);
}

/**
 ****************************************************************************************
 * @brief Handle clock recovery after a PCA adjustment
 ****************************************************************************************
 */
__STATIC void ld_pca_clk_recover_isr(uint32_t clock, uint8_t id)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_CLK_RECOVER_ISR_BIT, clock, id);

    if (ld_pca_env)
    {
        struct sch_arb_elt_tag *evt = &(ld_pca_env->evt);
        struct ld_pca_env_tag *pca_par = ld_pca_env;

        DBG_SWDIAG(PCA, SLOT_INT, 1);

        uint32_t clk_adj_slots = pca_par->clk_adj_slots;

        // Wait for a slot IRQ before requesting the SCLK update in order to minimize potential issues
        // Write down the clock slot adjustment

        if (clk_adj_slots)
        {
            // Adjust local clock
            rwip_time_set(rwip_time_get().hs + (clk_adj_slots << 1));
        }

        // Re-enable the default RWBT interrupts
        bt_intcntl0_set(ld_pca_config.intcntl0);
        ip_intcntl1_set(ld_pca_config.intcntl1);

        // Clear interrupts, Verify no scenario where we need to clear the FIFO interrupt
        ASSERT_ERR(0 == ip_intstat1_fifointstat_getf());
        bt_intack0_clear(BT_ERRORINTACK_BIT | BT_FRSYNCINTACK_BIT);

        // Call Scheduling Arbiter
        sch_arb_event_start_isr();

        // Indicate end of PCA procedure
        ld_pca_end_ind(CO_ERROR_NO_ERROR);

        sch_prog_disable(ld_pca_config.link_id);

        // Remove event
        sch_arb_remove(evt, true);

        // Delete event
        ke_free(ld_pca_env);
        ld_pca_env = NULL;

        DBG_SWDIAG(PCA, SLOT_INT, 0);
    }
}

/**
 ****************************************************************************************
 * @brief Handle PCA corase clock adjustment directive for MWS frame synchronization
 *
 * @param[in]  clk_adj_us         clock adjust in us
 * @param[in]  clk_adj_slots      clock adjust slots
 * @param[in]  clk_adj_instant    clock adjust instant (in 625us BT slots)
 *
 * @return status            0: success | 1-255: error
 ****************************************************************************************
 */
uint8_t ld_pca_coarse_clock_adjust(int16_t clk_adj_us, uint8_t clk_adj_slots, uint32_t clk_adj_instant)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    struct sch_arb_elt_tag *evt;
    struct ld_pca_env_tag *pca_par;

    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_COARSE_CLOCK_ADJUST_BIT, uint8_t, clk_adj_us, clk_adj_slots, clk_adj_instant);

    DBG_SWDIAG(PCA, COARSE_CLK_ADJ, 1);

    GLOBAL_INT_DISABLE();

    if (ld_pca_env == NULL)
    {
        // Allocate PCA event
        ld_pca_env = LD_ALLOC_EVT(ld_pca_env_tag);

        if (ld_pca_env != NULL)
        {
            evt = &(ld_pca_env->evt);
            pca_par = ld_pca_env;

            LD_INIT_EVT(evt, ld_pca_env_tag);

            // Initialize event parameters (common part)
            evt->cb_cancel       = &ld_pca_evt_canceled_cbk;
            evt->cb_start        = &ld_pca_evt_start_cbk;
            evt->cb_stop         = NULL; // no need stop cbk as activity cannot be stopped by HW
            evt->current_prio    = rwip_priority[RWIP_PRIO_PCA_DFT_IDX].value;

            // Add a virtual clk_adj_slots to LD_PCA_EVT_DURATION to accommodate slots lost in forward clock adjustment.
            evt->duration_min    = 2 * (LD_PCA_EVT_DURATION_MIN + clk_adj_slots) * SLOT_SIZE;

            // The PCA adjust instant is configured via an alarm in the event callback to occur at clk_adj_instant.
            // Here timestamp the event such that reserved time is scheduled before (& after) the PCA adjust instant.

            // Program clk_adj_instant via an alarm
            evt->time.hs = CLK_SUB(2 * clk_adj_instant, 2 * LD_PCA_EVT_PRE_ADJUST_DURATION_MIN);

            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_NO_LIMIT, SCH_ARB_PHASE_0, 0, 0);

            // Set event parameters
            pca_par->clk_adj_us = clk_adj_us;
            pca_par->clk_adj_slots = clk_adj_slots;
            pca_par->clk_adj_instant = clk_adj_instant;

            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
#if RW_BT_MWS_COEX
                ld_pca_config.pca_req_en = false;
#endif // RW_BT_MWS_COEX

                if (ld_pca_config.clk_drag_active)
                {
                    ld_pca_clk_drag_stop();
                }

                status = CO_ERROR_NO_ERROR;
            }
            else
            {
                ASSERT_ERR_FORCE(0);
            }
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR(0);
        }
    }

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(PCA, COARSE_CLK_ADJ, 0);

    return (status);
}

/**
 ****************************************************************************************
 * @brief Initiates a PCA clock dragging adjustment for MWS frame synchronization
 ****************************************************************************************
 */
uint8_t ld_pca_initiate_clock_dragging(int16_t clk_adj_us)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(LD_PCA_PATCH_TYPE, LD_PCA_INITIATE_CLOCK_DRAGGING_BIT, uint8_t, clk_adj_us);

    DBG_SWDIAG(PCA, CLK_DRAG, 1);

    GLOBAL_INT_DISABLE();

    // do not initiate clock draging if ongoing PCA event
    if (ld_pca_env == NULL)
    {
#if RW_BT_MWS_COEX
        if (ld_pca_config.frame_sync_det)
        {
            int16_t moment_us_offset = (int16_t)bt_pcastat_moment_offset_getf();
            int16_t target_us_offset = (int16_t)bt_pcacntl0_target_offset_getf();

            /* Extend sign on Moment_offset[10:0] */
            if (moment_us_offset & 0x0400)
                moment_us_offset |= 0xF800;

            /* Extend sign on Target_offset[10:0] */
            if (target_us_offset & 0x0400)
                target_us_offset |= 0xF800;

            // Record current momentary offset
            ld_pca_config.moment_us_offset = moment_us_offset;

            /* Compensate requested adjustment if existing delta between target and moment offset */
            clk_adj_us += (target_us_offset - moment_us_offset);

            bt_pcacntl0_blindcorr_en_setf(0);
        }
        else
#endif // RW_BT_MWS_COEX
        {
            bt_pcacntl0_blindcorr_en_setf(1);
        }

        // write the target offset, cut sign extension to 11 bit register size
        bt_pcacntl0_target_offset_setf(clk_adj_us & 0x7FF);

        // enable incremental phase shift
        bt_pcacntl0_phase_shift_en_setf(1);

        // Enable MTOFFINT0/MTOFFINT1 interrupts
        bt_intcntl0_set(bt_intcntl0_get() | (BT_MTOFFINT0MSK_BIT | BT_MTOFFINT1MSK_BIT));

        // Indicate clock dragging is active
        ld_pca_config.clk_drag_active = true;

#if RW_DEBUG
        // Moment offset interrupt count reset
        ld_pca_config.mtoffint_cnt = 0;
#endif //RW_DEBUG

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    DBG_SWDIAG(PCA, CLK_DRAG, 0);

    return (status);
}

#endif // PCA_SUPPORT

///@} LDPCA
