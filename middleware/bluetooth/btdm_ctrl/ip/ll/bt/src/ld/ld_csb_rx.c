/**
****************************************************************************************
*
* @file ld_csb_rx.c
*
* @brief LD Connectionless Slave Broadcast RX source code
*
* Copyright (C) RivieraWaves 2009-2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LDCSB_RX
 * @ingroup LD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#if CSB_SUPPORT

#include <string.h>

#include "co_utils.h"
#include "co_math.h"

#include "arch.h"
#include "rwip.h"
#include "ke_mem.h"

#include "ld.h"             // link driver API
#include "ld_util.h"        // link driver utilities
#include "ld_int.h"         // link driver internal

#include "lb.h"             // link broadcast API

#include "dbg.h"

#include "sch_arb.h"            // Scheduling Arbiter
#include "sch_prog.h"           // Scheduling Programmer

#include "reg_btcore.h"         // BT core registers
#include "reg_em_bt_cs.h"       // BT EM Control Structure
#include "reg_em_bt_rxdesc.h"   // BT EM RX descriptors


/*
 * DEFINES
 *****************************************************************************************
 */

/// Initial RX window size (in us)
#define LD_CSB_RX_WIN_SIZE_INIT       (2*SLOT_SIZE)

/// CSB RX event duration min (in us)
#define LD_CSB_RX_EVT_DUR_MIN         (SLOT_SIZE)

/// CSB RX event states
enum CSB_RX_EVT_STATE
{
    CSB_RX_EVT_WAIT,
    CSB_RX_EVT_ACTIVE,
    CSB_RX_EVT_END,
};

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LD CSB_RX environment structure
struct ld_csb_rx_env_tag
{
    /// CSB RX event
    struct sch_arb_elt_tag evt;

    /// Value of CLKN when the last sync was detected
    uint32_t last_sync_ts;

    /// Current clock offset (in BT half-slots)
    uint32_t clk_off;

    /// Current bit offset (in half-us)
    int16_t bit_off;

    /// Interval (in slots)
    uint16_t interval;

    /// Offset of CSB instant (D = I % T) (in slots)
    uint16_t offset;

    /// CSB_supervisionTO (in slots)
    uint16_t csb_supv_to;

    /// Additional drift to consider on given time reference, for the 1st reception attempt (in half-us)
    uint16_t add_drift;

    /// Remote_Timing_Accuracy (in ppm)
    uint8_t remote_timing_accuracy;

    /// Skip (number of consecutive CSB instants that can be skipped after successfully receiving a CSB packet)
    uint8_t skip;

    /// State
    uint8_t state;

    /// LT address
    uint8_t lt_addr;

    /// Target clock adjust for window size (in half-slots)
    uint8_t win_adjust;

    /// Remote BD_ADDR
    struct bd_addr bd_addr;
};


/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LD CSB_RX environment variable
__STATIC struct ld_csb_rx_env_tag *ld_csb_rx_env;


/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void ld_csb_rx_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Synchronize slave after correct reception
 *
 * Note: this algorithm supports large sync windows but not multi-attempts
 ****************************************************************************************
 */
__STATIC void ld_csb_rx_sync(uint32_t timestamp)
{
    DBG_SWDIAG(CSB_RX, SYNC, 1);

    uint8_t cs_idx = EM_BT_CS_CSB_RX_INDEX;

    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_csb_rx_env->evt);
    struct ld_csb_rx_env_tag *csb_rx_par = ld_csb_rx_env;

    // Read clock value where sync has been found (in half-slots)
    uint32_t rxclkn = (em_bt_rxclkn1_getf(cs_idx) << 16) | em_bt_rxclkn0_getf(cs_idx);
    // Read bit position where sync has been found (in half-us)
    uint16_t rxbit = HALF_SLOT_TIME_MAX - em_bt_rxbit_rxbit_getf(cs_idx);
    // Get estimated clock offset (in half-slots)
    uint32_t est_clk_off = (em_bt_clkoff1_getf(cs_idx) << 16) + em_bt_clkoff0_getf(cs_idx);

    // Compute new clock offset
    uint32_t new_clk_off = CLK_SUB(CLK_ADD_2(est_clk_off, timestamp), rxclkn);

    // Compute new bit offset
    int16_t new_bit_off = (rxbit - 2 * ld_env.exp_sync_pos);

    // Adjust bit offset and clock offset if needed
    if (new_bit_off < 0)
    {
        new_bit_off += HALF_SLOT_SIZE;
        new_clk_off = CLK_ADD_2(new_clk_off, 1);
        rxclkn = CLK_SUB(rxclkn, 1);
    }

    // Update event delay used for scheduling
    evt->time.hus = new_bit_off;

    // Save clock offset
    csb_rx_par->clk_off = new_clk_off;

    // Save bit offset
    csb_rx_par->bit_off = new_bit_off;

    // Update last sync timestamp
    csb_rx_par->last_sync_ts = rxclkn;

    // Restore the default synchronization window
    em_bt_wincntl_pack(cs_idx, 0, NORMAL_WIN_SIZE);

    DBG_SWDIAG(CSB_RX, SYNC, 0);
}

/**
 ****************************************************************************************
 * @brief Check the reception during activity
 ****************************************************************************************
 */
__STATIC bool ld_csb_rx(uint16_t *buf_ptr, uint16_t *length)
{
    bool csb_received = false;

    // Check if CSB has been received
    if (ld_rxdesc_check(EM_BT_CS_CSB_RX_INDEX))
    {
        do
        {
            // Get current RX descriptor index
            uint8_t index = ld_env.curr_rxdesc_index;
            uint16_t rxpheader;
            uint16_t rxstat = em_bt_rxstat_get(index);

            // Check if CSB reception is correct
            if (rxstat & (EM_BT_RXSYNCERR_BIT | EM_BT_RXHECERR_BIT | EM_BT_RXCRCERR_BIT | EM_BT_RXBADLT_BIT))
                break;

            DBG_SWDIAG(CSB_RX, PKT_RX, 1);

            // Retrieve CSB RX payload length
            rxpheader = em_bt_rxpheader_get(index);

            // IUT shall ignore any data with LLID not equal to LLID_START
            if (LLID_START == ((rxpheader & EM_BT_RXLLID_MASK) >> EM_BT_RXLLID_LSB))
            {
                *length = (rxpheader & EM_BT_RXLENGTH_MASK) >> EM_BT_RXLENGTH_LSB;

                // Retrieve CSB RX payload buffer
                *buf_ptr = em_bt_rxaclbufptr_getf(index);

                // Indicate successful reception
                csb_received = true;
            }

            DBG_SWDIAG(CSB_RX, PKT_RX, 0);

        }
        while (0);

        // Free RX descriptor
        ld_rxdesc_free();
    }

    return (csb_received);
}

/**
 ****************************************************************************************
 * @brief Report the end of the activity
 ****************************************************************************************
 */
__STATIC void ld_csb_rx_end(uint8_t reason)
{
    DBG_SWDIAG(CSB_RX, END, 1);

    if (CO_ERROR_CON_TIMEOUT == reason)
    {
        // Point to parameters
        struct ld_csb_rx_env_tag *csb_rx_par = ld_csb_rx_env;

        // Report CSB supervisionTO
        struct lb_csb_rx_end_ind *msg = KE_MSG_ALLOC(LB_CSB_RX_END_IND, TASK_LB, TASK_NONE, lb_csb_rx_end_ind);
        memcpy(&msg->bd_addr.addr[0], &csb_rx_par->bd_addr.addr[0], BD_ADDR_LEN);
        msg->lt_addr = csb_rx_par->lt_addr;
        ke_msg_send(msg);
    }

    // Remove permission/status of CS as now unused
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(EM_BT_CS_CSB_RX_INDEX)), REG_EM_BT_CS_SIZE, false, false, false);

    // Free event memory
    ke_free(ld_csb_rx_env);
    ld_csb_rx_env = NULL;

    DBG_SWDIAG(CSB_RX, END, 0);
}

/**
 ****************************************************************************************
 * @brief Schedule CSB RX
 ****************************************************************************************
 */
__STATIC void ld_csb_rx_sched(uint8_t skip)
{
    DBG_SWDIAG(CSB_RX, SCHED, 1);

    // Point to parameters
    struct sch_arb_elt_tag *evt = &(ld_csb_rx_env->evt);
    struct ld_csb_rx_env_tag *csb_rx_par = ld_csb_rx_env;
    uint32_t target_clock;
    bool found = false;
    int16_t new_bit_off = 0;
    uint32_t new_clk_off = 0;
    uint32_t rx_win_size = 0;

    // Check if sync in last receive window
    if (CLK_DIFF(evt->time.hs, csb_rx_par->last_sync_ts) >= 0)
    {
        // If sync, update target clock relative to this
        target_clock = CLK_ADD_2(csb_rx_par->last_sync_ts, 2 * csb_rx_par->interval);
    }
    else
    {
        // If no sync, update target clock based on unadjusted timings
        target_clock = CLK_ADD_3(evt->time.hs, csb_rx_par->win_adjust, 2 * csb_rx_par->interval);
    }

    // Check if some instants can be skipped
    if (skip)
    {
        target_clock = CLK_ADD_2(target_clock, 2 * skip * csb_rx_par->interval);
    }

    // Try to schedule and adjust the Rx window size accordingly if needed
    while (CLK_SUB(target_clock, csb_rx_par->last_sync_ts) < (uint32_t)(2 * csb_rx_par->csb_supv_to - rwip_prog_delay))
    {
        // The scheduling timestamp is initialized to the targeted clock value of the sync
        uint32_t sched_ts = target_clock;

        // Compute maximum drift
        uint32_t clock_diff = CLK_SUB(sched_ts, csb_rx_par->last_sync_ts);

        uint8_t local_drift = rwip_current_drift_get();
        uint8_t peer_drift = csb_rx_par->remote_timing_accuracy;

        uint32_t max_drift = clock_diff * (local_drift + peer_drift) / 1600; // half_slots * ppm * 625 half_us / 1000000;

        // Calculate the new Rx window size
        rx_win_size = 2 * max_drift + 2 * NORMAL_WIN_SIZE + csb_rx_par->add_drift;

        // Load bit offset / clock offset of the last synced packet
        new_bit_off = csb_rx_par->bit_off;
        new_clk_off = csb_rx_par->clk_off;

        // Compute the new bit offset / clock offset
        new_bit_off -= (rx_win_size - 2 * NORMAL_WIN_SIZE) / 2;
        if (new_bit_off < 0)
        {
            uint8_t num_hs = CO_DIVIDE_CEIL((-new_bit_off), HALF_SLOT_SIZE);
            sched_ts = CLK_SUB(sched_ts, num_hs);
            new_bit_off += num_hs * HALF_SLOT_SIZE;
            new_clk_off = CLK_ADD_2(new_clk_off, num_hs);
        }

        // Save adjustment to scheduling for window
        csb_rx_par->win_adjust = (uint8_t)CLK_SUB(target_clock, sched_ts);

        // Set new bit offset as event delay for scheduling
        evt->time.hus = new_bit_off;

        // Load minimum duration including sync window
        evt->duration_min = 2 * LD_CSB_RX_EVT_DUR_MIN + rx_win_size;

        // Try to schedule
        evt->time.hs = sched_ts;
        if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
        {
            found = true;
            break;
        }
        else
        {
            // Increment priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_CSB_RX_DFT_IDX));

            // Move timestamp to the next anchor point
            target_clock = CLK_ADD_2(target_clock, 2 * csb_rx_par->interval);
        }
    }

    if (found)
    {
        uint32_t rx_win_size_us = (rx_win_size + 1) >> 1;
        uint8_t cs_idx = EM_BT_CS_CSB_RX_INDEX;

        // Apply the values calculated for the clock offset and the bit offset
        em_bt_clkoff0_setf(cs_idx, new_clk_off & EM_BT_CLKOFF0_MASK);
        em_bt_clkoff1_setf(cs_idx, (new_clk_off >> 16));
        // Check if the wide-open mode should be used
        if (rx_win_size_us > EM_BT_CS_RXWINSZ_MAX)
        {
            if (CO_MOD(rx_win_size_us, SLOT_SIZE))
            {
                em_bt_wincntl_pack(cs_idx, 1, rx_win_size_us / SLOT_SIZE + 1);
            }
            else
            {
                em_bt_wincntl_pack(cs_idx, 1, rx_win_size_us / SLOT_SIZE);
            }
        }
        else
        {
            em_bt_wincntl_pack(cs_idx, 0, (rx_win_size_us + 1) >> 1);
        }

        csb_rx_par->state = CSB_RX_EVT_WAIT;
    }
    else
    {
        // Report CSB end
        ld_csb_rx_end(CO_ERROR_CON_TIMEOUT);
    }

    DBG_SWDIAG(CSB_RX, SCHED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_csb_rx_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    DBG_SWDIAG(CSB_RX, EVT_START, 1);

    ASSERT_ERR((&(ld_csb_rx_env->evt)) == evt);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_csb_rx_env_tag *csb_rx_par = ld_csb_rx_env;
        uint8_t cs_idx = EM_BT_CS_CSB_RX_INDEX;

        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk         = &ld_csb_rx_frm_cbk;
            prog_par.time.hs         = evt->time.hs;
            prog_par.time.hus        = evt->time.hus;
            prog_par.cs_idx          = cs_idx;
            prog_par.dummy           = cs_idx;
            prog_par.bandwidth       = evt->duration_min;
            prog_par.prio_1          = evt->current_prio;
            prog_par.prio_2          = 0;
            prog_par.prio_3          = 0;
            prog_par.pti_prio        = BT_PTI_BCAST_IDX;
            prog_par.add.bt.frm_type = SCH_BT_FRAME_TYPE_CSB;
            prog_par.mode            = SCH_PROG_BT;
            sch_prog_push(&prog_par);
        }

        // Move state
        csb_rx_par->state = CSB_RX_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(CSB_RX, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_csb_rx_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    DBG_SWDIAG(CSB_RX, EVT_CANCELED, 1);

    ASSERT_ERR((&(ld_csb_rx_env->evt)) == evt);

    if (evt != NULL)
    {
        // Check if CSB RX is requested to stop
        if (ld_csb_rx_env->state == CSB_RX_EVT_END)
        {
            // Report CSB RX end
            ld_csb_rx_end(CO_ERROR_CON_TERM_BY_LOCAL_HOST);
        }
        else
        {
            // Increment priority
            evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_CSB_RX_DFT_IDX));

            // Reschedule
            ld_csb_rx_sched(0);
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(CSB_RX, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_csb_rx_frm_isr(uint32_t timestamp)
{
    DBG_SWDIAG(CSB_RX, FRM_ISR, 1);

    if (ld_csb_rx_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_csb_rx_env->evt);
        struct ld_csb_rx_env_tag *csb_rx_par = ld_csb_rx_env;

        // Remove event
        sch_arb_remove(evt, true);

        // Check if CSB RX is requested to stop
        if (csb_rx_par->state == CSB_RX_EVT_END)
        {
            // Report CSB RX end
            ld_csb_rx_end(CO_ERROR_CON_TERM_BY_LOCAL_HOST);
        }
        else
        {
            uint8_t cs_idx = EM_BT_CS_CSB_RX_INDEX;
            uint8_t skip = 0;
            uint16_t buf_ptr;
            uint16_t length;

            // Check if a sync has been found in the frame
            if (em_bt_rxbit_frrxok_getf(cs_idx))
            {
                // Refresh slave synchronization as per new reception
                ld_csb_rx_sync(timestamp);

                csb_rx_par->add_drift = 0;
            }

            // Check if CSB packet has been received
            if (ld_csb_rx(&buf_ptr, &length))
            {
                // Report CSB reception
                struct lb_csb_rx_ind *ind = KE_MSG_ALLOC_DYN(LB_CSB_RX_IND, TASK_LB, TASK_NONE, lb_csb_rx_ind, length);
                memcpy(&ind->bd_addr.addr[0], &csb_rx_par->bd_addr.addr[0], BD_ADDR_LEN);
                ind->lt_addr = csb_rx_par->lt_addr;
                ind->offset = (csb_rx_par->clk_off << 1) + (csb_rx_par->bit_off > (SLOT_SIZE / 2));
                ind->clk = timestamp;
                ind->rx_status = CSB_RX_OK;
                ind->length = length;
                em_rd(ind->pdu, buf_ptr, length);
                ke_msg_send(ind);

                // Load the skip parameter
                skip = csb_rx_par->skip;
            }
            else
            {
                // Report CSB instant (missed reception)
                struct lb_csb_rx_ind *ind = KE_MSG_ALLOC(LB_CSB_RX_IND, TASK_LB, TASK_NONE, lb_csb_rx_ind);
                memcpy(&ind->bd_addr.addr[0], &csb_rx_par->bd_addr.addr[0], BD_ADDR_LEN);
                ind->lt_addr = csb_rx_par->lt_addr;
                ind->offset = (csb_rx_par->clk_off << 1) + (csb_rx_par->bit_off > (SLOT_SIZE / 2));
                ind->clk = timestamp;
                ind->rx_status = CSB_RX_KO;
                ke_msg_send(ind);
            }

            // Reschedule
            ld_csb_rx_sched(skip);
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

    DBG_SWDIAG(CSB_RX, FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_csb_rx_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_csb_rx_frm_isr(timestamp);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_csb_rx_env->evt);

        // Remove event
        sch_arb_remove(evt, true);

        // Invoke event cancel callback
        ld_csb_rx_evt_canceled_cbk(evt);
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, dummy, irq_type);
    }
    break;
    }
}


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void ld_csb_rx_init(void)
{
    ld_csb_rx_env = NULL;
}

void ld_csb_rx_reset(void)
{
    // Check if CSB RX is active
    if (ld_csb_rx_env != NULL)
    {
        // Free event memory
        ke_free(ld_csb_rx_env);
        ld_csb_rx_env = NULL;
    }
}

uint8_t ld_csb_rx_start(struct ld_csb_rx_params *params)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    DBG_SWDIAG(CSB_RX, START, 1);

    // Check if CSB RX is inactive
    if (ld_csb_rx_env == NULL)
    {
        // Allocate event
        ld_csb_rx_env = LD_ALLOC_EVT(ld_csb_rx_env_tag);

        if (ld_csb_rx_env != NULL)
        {
            uint8_t bch[LD_BCH_SIZE];
            uint8_t cs_idx = EM_BT_CS_CSB_RX_INDEX;

            // Point to parameters
            struct sch_arb_elt_tag *evt = &(ld_csb_rx_env->evt);
            struct ld_csb_rx_env_tag *csb_rx_par = ld_csb_rx_env;

            LD_INIT_EVT(evt, ld_csb_rx_env_tag);

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, true);

            // Initialize event parameters (common part)
            evt->cb_cancel        = &ld_csb_rx_evt_canceled_cbk;
            evt->cb_start         = &ld_csb_rx_evt_start_cbk;
            evt->cb_stop          = NULL;
            evt->current_prio        = rwip_priority[RWIP_PRIO_CSB_RX_DFT_IDX].value;
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, 0, 0, 0);

            // Initialize event parameters (CSB RX part)
            csb_rx_par->clk_off                = params->clock_offset;
            csb_rx_par->bit_off                = 0;
            csb_rx_par->interval               = params->interval;
            csb_rx_par->offset                 = params->next_csb_clock - ((params->next_csb_clock / params->interval) * params->interval);
            csb_rx_par->csb_supv_to            = params->csb_supv_to;
            csb_rx_par->remote_timing_accuracy = params->remote_timing_accuracy;
            csb_rx_par->skip                   = params->skip;
            csb_rx_par->lt_addr                = params->lt_addr;
            csb_rx_par->add_drift              = 4 * SLOT_SIZE;
            csb_rx_par->win_adjust             = 0;
            memcpy(&csb_rx_par->bd_addr.addr[0], &params->bd_addr.addr[0], BD_ADDR_LEN);

            // Compute the BCH
            ld_util_bch_create(&params->bd_addr.addr[0], &bch[0]);

            // Set control structure fields
            em_bt_frcntl_pack(cs_idx, 1 /*fpoll*/, 0 /*fmwspatt*/, RWIP_COEX_GET(BCAST, TXBSY), RWIP_COEX_GET(BCAST, RXBSY), RWIP_COEX_GET(BCAST, DNABORT), 0 /*samen*/, 1 /*lsam_dsb*/, EM_BT_CS_FMT_BCST_SCAN);
            em_bt_bdaddr_setf(cs_idx, 0, (params->bd_addr.addr[1] << 8) | params->bd_addr.addr[0]);
            em_bt_bdaddr_setf(cs_idx, 1, (params->bd_addr.addr[3] << 8) | params->bd_addr.addr[2]);
            em_bt_bdaddr_setf(cs_idx, 2, (params->bd_addr.addr[5] << 8) | params->bd_addr.addr[4]);
            em_bt_bch0_setf(cs_idx, (bch[1] << 8) | bch[0]);
            em_bt_bch1_setf(cs_idx, (bch[3] << 8) | bch[2]);
            em_bt_rxmaxbuf_bch2_pack(cs_idx, ACL_DATA_BUF_SIZE, 0, bch[4] & EM_BT_BCH2_MASK);
            em_bt_txdescptr_setf(cs_idx, 0);
            em_bt_linkcntl_pack(cs_idx,
                                /*aknena*/ 0,
                                /*afhena*/ 1,
                                /*laap*/ 0,
                                /*whdsb*/ 0,
                                /*acledr*/ params->edr,
                                /*aclltaddr*/ params->lt_addr,
#if (EAVESDROPPING_SUPPORT)
                                /*aclbinackena*/ 0, /*escobinackena*/ 0, /*duplfltena*/ 0,
#endif // EAVESDROPPING_SUPPORT
                                /*linklbl*/ cs_idx);
            em_bt_pwrcntl_pack(cs_idx, /*fhen*/ 1, /*freq*/ 0, /*txpwr*/ 0);
            em_bt_clkoff0_setf(cs_idx, csb_rx_par->clk_off & EM_BT_CLKOFF0_MASK);
            em_bt_clkoff1_setf(cs_idx, (csb_rx_par->clk_off >> 16));
            em_bt_txrxcntl_pack(cs_idx, 0,
#if (EAVESDROPPING_SUPPORT)
                                0,
#endif // EAVESDROPPING_SUPPORT
                                0, 0, 0, 0, 0, 0, 0, 0, 0);
            em_bt_loopcntl_pack(cs_idx, /*tonb*/ 0, /*attnb*/ 1);

            // Write channel map to CS
            em_bt_chmap0_setf(cs_idx, co_read16p(&params->afh_ch_map.map[0]));
            em_bt_chmap1_setf(cs_idx, co_read16p(&params->afh_ch_map.map[2]));
            em_bt_chmap2_setf(cs_idx, co_read16p(&params->afh_ch_map.map[4]));
            em_bt_chmap3_setf(cs_idx, co_read16p(&params->afh_ch_map.map[6]));
            em_bt_chmap4_setf(cs_idx, co_read16p(&params->afh_ch_map.map[8]) & EM_BT_CHMAP4_MASK);

            // Default clear other control
            em_bt_maxfrmtime_set(cs_idx, 0);
#if (EAVESDROPPING_SUPPORT)
            em_bt_cidcntl_set(cs_idx, 0);
            em_bt_rxcrc_set(cs_idx, 0);
#endif // EAVESDROPPING_SUPPORT

            // Schedule for the first time
            {
                uint32_t master_clock;
                int32_t diff;

                uint32_t clock = ld_read_clock();

                master_clock = CLK_ADD_2(clock, csb_rx_par->clk_off);

                // Make sure that the master clock considered is even (corresponds to a BT slot)
                if (master_clock & 0x1)
                {
                    master_clock &= ~0x1;
                    clock = CLK_SUB(clock, 1);
                }

                // Find the next CSB anchor point
                diff = csb_rx_par->offset - CO_MOD((master_clock >> 1), csb_rx_par->interval);
                if (diff <= 0)
                {
                    diff += csb_rx_par->interval;
                }
                evt->time.hs = CLK_ADD_2(clock, 2 * diff);

                // Subtract the interval because it will be added by the ld_csb_rx_sched function
                evt->time.hs = CLK_SUB(evt->time.hs, 2 * csb_rx_par->interval);
                // Initialize last sync timestamp
                csb_rx_par->last_sync_ts = evt->time.hs;

                GLOBAL_INT_DISABLE();

                ld_csb_rx_sched(0);

                GLOBAL_INT_RESTORE();
            }

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR(0);
        }
    }

    DBG_SWDIAG(CSB_RX, START, 0);

    return (status);
}

uint8_t ld_csb_rx_afh_update(struct bt_ch_map *afh_ch_map)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    if (ld_csb_rx_env != NULL)
    {
        uint8_t cs_idx = EM_BT_CS_CSB_RX_INDEX;

        // Write channel map to CS
        em_bt_chmap0_setf(cs_idx, co_read16p(&afh_ch_map->map[0]));
        em_bt_chmap1_setf(cs_idx, co_read16p(&afh_ch_map->map[2]));
        em_bt_chmap2_setf(cs_idx, co_read16p(&afh_ch_map->map[4]));
        em_bt_chmap3_setf(cs_idx, co_read16p(&afh_ch_map->map[6]));
        em_bt_chmap4_setf(cs_idx, co_read16p(&afh_ch_map->map[8]) & EM_BT_CHMAP4_MASK);

        status = CO_ERROR_NO_ERROR;
    }
    else
    {
        ASSERT_ERR(0);
    }

    return (status);
}

uint8_t ld_csb_rx_stop(void)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (ld_csb_rx_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_csb_rx_env->evt);
        struct ld_csb_rx_env_tag *csb_rx_par = ld_csb_rx_env;

        switch (csb_rx_par->state)
        {
        case CSB_RX_EVT_WAIT:
        {
            // Remove event
            sch_arb_remove(evt, false);

            // Report synchronization train end
            ld_csb_rx_end(CO_ERROR_CON_TERM_BY_LOCAL_HOST);
        }
        break;

        case CSB_RX_EVT_ACTIVE:
        {
            // Move state
            csb_rx_par->state = CSB_RX_EVT_END;
        }
        break;

        default:
        {
            // Nothing to do
            ASSERT_INFO(0, csb_rx_par->state, 0);
        }
        break;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}
#endif // CSB_SUPPORT

///@} LDCSB_RX
