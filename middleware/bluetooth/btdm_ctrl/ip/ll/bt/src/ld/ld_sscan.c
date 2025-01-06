/**
****************************************************************************************
*
* @file ld_sscan.c
*
* @brief LD Synchronization Scan source code
*
* Copyright (C) RivieraWaves 2009-2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LDSSCAN
 * @ingroup LD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#if CSB_SUPPORT || PCA_SUPPORT

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
#include "sch_slice.h"          // Scheduling Slicer

#include "reg_ipcore.h"         // IP core registers
#include "reg_btcore.h"         // BT core registers
#include "reg_em_bt_cs.h"       // BT EM Control Structure
#include "reg_em_bt_rxdesc.h"   // BT EM RX descriptors
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/// Synchronization Scan event states
enum SSCAN_EVT_STATE
{
    SSCAN_EVT_WAIT,
    SSCAN_EVT_ACTIVE,
    SSCAN_EVT_END,
};

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// LD SSCAN environment structure
struct ld_sscan_env_tag
{
    /// Synchronization scan event
    struct sch_arb_elt_tag evt;

    /// BD Address of the paged device
    struct bd_addr bd_addr;
    /// Synchronization Scan Timeout (in slots)
    uint32_t sscan_to;
    /// Amount of time between consecutive synchronization scans in slots (625 us)
    uint16_t sscan_intv;
    /// Amount of time for the duration of the synchronization scan in slots (625 us)
    uint16_t sscan_win;
    /// Remaining slots in current scan window (in 625us BT slots)
    uint32_t win_rem_slots;
    /// State
    uint8_t state;
    /// Channel index
    uint8_t ch_ind;
};

/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// LD SSCAN environment variable
__STATIC struct ld_sscan_env_tag *ld_sscan_env;


/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

__STATIC void ld_sscan_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Cleanup sscan environment
 ****************************************************************************************
 */
__STATIC void ld_sscan_cleanup(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_SSCAN_PATCH_TYPE, LD_SSCAN_CLEANUP_BIT);

    // Remove permission/status of CS as now unused
    DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(EM_BT_CS_SSCAN_INDEX)), REG_EM_BT_CS_SIZE, false, false, false);

    // Free event memory
    ke_free(ld_sscan_env);
    ld_sscan_env = NULL;
}

/**
 ****************************************************************************************
 * @brief Check the reception during activity
 ****************************************************************************************
 */
__STATIC bool ld_sscan_stp_rx(uint32_t *clk_off, int16_t *bit_off, uint32_t *future_csb_inst, struct bt_ch_map *afh_ch_map, struct bd_addr *mst_bd_addr, uint16_t *csb_intv, uint8_t *csb_lt_addr,  uint8_t *svc_data)
{
    bool stp_received = false;

    FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(LD_SSCAN_PATCH_TYPE, LD_SSCAN_STP_RX_BIT, bool, 8, clk_off, bit_off, future_csb_inst, afh_ch_map, mst_bd_addr, csb_intv, csb_lt_addr,  svc_data);

    // Check if STP has been received
    if (ld_rxdesc_check(EM_BT_CS_SSCAN_INDEX))
    {
        DBG_SWDIAG(SSCAN, STP_RX, 1);

        do
        {
            uint8_t cs_idx = EM_BT_CS_SSCAN_INDEX;
            // Get current RX descriptor index
            uint8_t index = ld_env.curr_rxdesc_index;

            uint32_t clk_hs;
            uint32_t rx_clk;
            uint16_t rxheader;
            uint16_t rxpheader;
            uint8_t rxtype;
            uint8_t rxltaddr;
            uint16_t rxlength;
            uint16_t buf_ptr;
            uint16_t rxstat = em_bt_rxstat_get(index);

            // Check if STP reception is correct
            if (rxstat & (EM_BT_RXSYNCERR_BIT | EM_BT_RXHECERR_BIT | EM_BT_RXCRCERR_BIT | EM_BT_RXBADLT_BIT))
                break;

            rxheader = em_bt_rxheader_get(index);
            rxtype = (rxheader & EM_BT_RXTYPE_MASK) >> EM_BT_RXTYPE_LSB;

            ASSERT_WARN(em_bt_rxheader_rxtype_getf(index) == DM3_TYPE, em_bt_rxheader_rxtype_getf(index), 0);

            if (rxtype != DM3_TYPE)
                break;

            rxltaddr = (rxheader & EM_BT_RXLTADDR_MASK) >> EM_BT_RXLTADDR_LSB;

            if (rxltaddr != LT_ADDR_BCST)
                break;

            rxpheader = em_bt_rxpheader_get(index);
            rxlength = (rxpheader & EM_BT_RXLENGTH_MASK) >> EM_BT_RXLENGTH_LSB;

            if (rxlength < STP_PACKET_SIZE)
                break;

            // Retrieve STP payload buffer
            buf_ptr = em_bt_rxaclbufptr_getf(index);

            // Extract fields from packet
            ld_util_stp_unpk(buf_ptr, &clk_hs, future_csb_inst, afh_ch_map, mst_bd_addr, csb_intv, csb_lt_addr, svc_data);

            // Read RX clock (in half-slots)
            rx_clk = (em_bt_rxclkn1_getf(cs_idx) << 16) | em_bt_rxclkn0_getf(cs_idx);

            // Compute clock offset
            *clk_off = CLK_SUB(clk_hs, rx_clk);

#if PCA_SUPPORT
            // Calculate the bit offset and adjust the clock offset accordingly
            *bit_off = (HALF_SLOT_TIME_MAX - em_bt_rxbit_rxbit_getf(cs_idx)) - 2 * ld_env.exp_sync_pos;
            if (*bit_off < 0)
            {
                *bit_off += HALF_SLOT_SIZE;
                *clk_off = CLK_ADD_2(*clk_off, 1);
            }
#endif // PCA_SUPPORT

            // Indicate successful reception
            stp_received = true;

        }
        while (0);

        // Free RX descriptor
        ld_rxdesc_free();

        DBG_SWDIAG(SSCAN, STP_RX, 0);
    }

    return (stp_received);
}

/**
 ****************************************************************************************
 * @brief Handle event start notification
 ****************************************************************************************
 */
__STATIC void ld_sscan_evt_start_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_SSCAN_PATCH_TYPE, LD_SSCAN_EVT_START_CBK_BIT, evt);

    ASSERT_ERR((&(ld_sscan_env->evt)) == evt);

    DBG_SWDIAG(SSCAN, EVT_START, 1);

    if (evt != NULL)
    {
        // Point to parameters
        struct ld_sscan_env_tag *sscan_par = ld_sscan_env;
        uint8_t cs_idx = EM_BT_CS_SSCAN_INDEX;

        // Prepare CS
        em_bt_wincntl_pack(EM_BT_CS_SSCAN_INDEX, 1, sscan_par->win_rem_slots);

        // Set the current frequency
        em_bt_pwrcntl_freq_setf(cs_idx, ld_sync_train_channels[sscan_par->ch_ind] / 2);

        {
            // Push the programming to SCH PROG
            struct sch_prog_params prog_par;
            prog_par.frm_cbk         = &ld_sscan_frm_cbk;
            prog_par.time.hs         = evt->time.hs;
            prog_par.time.hus        = 0;
            prog_par.cs_idx          = cs_idx;
            prog_par.dummy           = cs_idx;
            prog_par.bandwidth       = evt->duration_min;
            prog_par.prio_1          = evt->current_prio;
            prog_par.prio_2          = 0;
            prog_par.prio_3          = 0;
            prog_par.pti_prio        = BT_PTI_BCAST_IDX;
            prog_par.add.bt.frm_type = SCH_BT_FRAME_TYPE_NORMAL;
            prog_par.mode            = SCH_PROG_BT;
            sch_prog_push(&prog_par);
        }

        // Register the scan window as active to scheduling parameters
        sch_slice_bg_add(BT_SSCAN);

        // Move state
        sscan_par->state = SSCAN_EVT_ACTIVE;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(SSCAN, EVT_START, 0);
}

/**
 ****************************************************************************************
 * @brief Handle event canceled notification
 ****************************************************************************************
 */
__STATIC void ld_sscan_evt_canceled_cbk(struct sch_arb_elt_tag *evt)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_SSCAN_PATCH_TYPE, LD_SSCAN_EVT_CANCELED_CBK_BIT, evt);

    ASSERT_ERR((&(ld_sscan_env->evt)) == evt);

    DBG_SWDIAG(SSCAN, EVT_CANCELED, 1);

    if (evt != NULL)
    {
        ASSERT_ERR(ld_sscan_env->state == SSCAN_EVT_WAIT);

        // Increment priority
        evt->current_prio = RWIP_PRIO_ADD_2(evt->current_prio, RWIP_PRIO_INC(RWIP_PRIO_SSCAN_DFT_IDX));

        // Reschedule ASAP
        if (sch_arb_insert(evt) != SCH_ARB_ERROR_OK)
        {
            DBG_SWDIAG(SSCAN, TIMEOUT, 1);
            // Sync scan timeout
            struct lb_sscan_end_ind *ind = KE_MSG_ALLOC(LB_SSCAN_END_IND, TASK_LB, TASK_NONE, lb_sscan_end_ind);
            ind->status = CO_ERROR_CON_TIMEOUT;
            ke_msg_send(ind);

            // Free event memory
            ld_sscan_cleanup();

            DBG_SWDIAG(SSCAN, TIMEOUT, 0);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(SSCAN, EVT_CANCELED, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt
 ****************************************************************************************
 */
__STATIC void ld_sscan_frm_isr(uint32_t timestamp)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_SSCAN_PATCH_TYPE, LD_SSCAN_FRM_ISR_BIT, timestamp);

    DBG_SWDIAG(SSCAN, FRM_ISR, 1);

    if (ld_sscan_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_sscan_env->evt);
        struct ld_sscan_env_tag *sscan_par = ld_sscan_env;

        uint32_t clk_off;
        int16_t bit_off;
        uint32_t future_csb_inst;
        struct bt_ch_map afh_ch_map;
        struct bd_addr mst_bd_addr;
        uint16_t csb_intv;
        uint8_t csb_lt_addr;
        uint8_t svc_data;

        uint32_t clock = ld_read_clock();

        // Remove event
        sch_arb_remove(evt, true);

        // Unregister the scan window from scheduling parameters
        sch_slice_bg_remove(BT_SSCAN);

        if (SSCAN_EVT_END == sscan_par->state)
        {
            // Free event memory
            ld_sscan_cleanup();
        }
        // Check if STP has been received
        else if (ld_sscan_stp_rx(&clk_off, &bit_off, &future_csb_inst, &afh_ch_map, &mst_bd_addr, &csb_intv, &csb_lt_addr, &svc_data))
        {
            DBG_SWDIAG(SSCAN, STP_RX_OK, 1);

            // Report sync train packet reception
            struct lb_sscan_end_ind *ind = KE_MSG_ALLOC(LB_SSCAN_END_IND, TASK_LB, TASK_NONE, lb_sscan_end_ind);
            memcpy(&ind->afh_ch_map.map[0], &afh_ch_map.map[0], BT_CH_MAP_LEN);
            memcpy(&ind->bd_addr.addr[0], &mst_bd_addr.addr[0], BD_ADDR_LEN);
            ind->clock_offset = clk_off;
#if PCA_SUPPORT
            ind->bit_offset = bit_off;
#endif // PCA_SUPPORT
            ind->next_bcst_instant = future_csb_inst;
            ind->lt_addr = csb_lt_addr;
            ind->csb_int = csb_intv;
            ind->service_data = svc_data;
            ind->status = CO_ERROR_NO_ERROR;
            ke_msg_send(ind);

            // Free event memory
            ld_sscan_cleanup();

            DBG_SWDIAG(SSCAN, STP_RX_OK, 0);
        }
        else
        {
            uint32_t effective_slots = CLK_SUB(clock, timestamp) >> 1;

            if (effective_slots <= sscan_par->win_rem_slots)
            {
                // Subtract the number of effective scanning slots
                sscan_par->win_rem_slots -= effective_slots;
            }
            else
            {
                sscan_par->win_rem_slots = 0;
            }

            // Check if current synchronization scan window is finished
            if (sscan_par->win_rem_slots == 0)
            {
                // Add scan window period with a little random back off period
                int32_t delay = sscan_par->sscan_intv - sscan_par->sscan_win;
                int32_t shift = ((co_rand_byte() + ip_finetimtgt_get() + (clock >> 1)) & 0x7F) - 0x40; // shift within [-64:+63]
                delay += shift;
                evt->time.hs = CLK_ADD_2(clock, 2 * delay);

                // Reload window length
                sscan_par->win_rem_slots = sscan_par->sscan_win;

                // Change frequency
                sscan_par->ch_ind++;
                if (sscan_par->ch_ind >= SYNC_TRAIN_CHANNEL_NB)
                    sscan_par->ch_ind = 0;
            }

            // Restore original priority
            evt->current_prio = rwip_priority[RWIP_PRIO_SSCAN_DFT_IDX].value;

            // Set duration min
            evt->duration_min = co_min((2 * sscan_par->win_rem_slots * SLOT_SIZE), sch_slice_params.scan_evt_dur);

            // Try reschedule
            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                sscan_par->state = SSCAN_EVT_WAIT;
            }
            else
            {
                DBG_SWDIAG(SSCAN, TIMEOUT, 1);

                // Sync scan timeout
                struct lb_sscan_end_ind *ind = KE_MSG_ALLOC(LB_SSCAN_END_IND, TASK_LB, TASK_NONE, lb_sscan_end_ind);
                ind->status = CO_ERROR_CON_TIMEOUT;
                ke_msg_send(ind);

                // Free event memory
                ld_sscan_cleanup();

                DBG_SWDIAG(SSCAN, TIMEOUT, 0);
            }
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(SSCAN, FRM_ISR, 0);
}

/**
 ****************************************************************************************
 * @brief Handle frame interrupt notification
 ****************************************************************************************
 */
__STATIC void ld_sscan_frm_cbk(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_SSCAN_PATCH_TYPE, LD_SSCAN_FRM_CBK_BIT, timestamp, dummy, irq_type);

    DBG_SWDIAG(SSCAN, FRM_CBK, 1);

    switch (irq_type)
    {
    case SCH_FRAME_IRQ_EOF:
    case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
    case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
    {
        ld_sscan_frm_isr(timestamp);
    }
    break;
    case SCH_FRAME_IRQ_SKIP:
    {
        if (ld_sscan_env != NULL)
        {
            // Point to parameters
            struct sch_arb_elt_tag *evt = &(ld_sscan_env->evt);
            struct ld_sscan_env_tag *sscan_par = ld_sscan_env;

            // Remove event
            sch_arb_remove(evt, true);

            if (SSCAN_EVT_END == sscan_par->state)
            {
                // Free event memory
                ld_sscan_cleanup();
            }
            else
            {
                sscan_par->state = SSCAN_EVT_WAIT;

                // Invoke event cancel callback
                ld_sscan_evt_canceled_cbk(evt);
            }
        }
    }
    break;
    default:
    {
        ASSERT_INFO_FORCE(0, dummy, irq_type);
    }
    break;
    }

    DBG_SWDIAG(SSCAN, FRM_CBK, 0);
}


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void ld_sscan_init(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_SSCAN_PATCH_TYPE, LD_SSCAN_INIT_BIT);

    ld_sscan_env = NULL;
}

void ld_sscan_reset(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(LD_SSCAN_PATCH_TYPE, LD_SSCAN_RESET_BIT);

    // Check if synchronization scan is active
    if (ld_sscan_env != NULL)
    {
        // Free event memory
        ke_free(ld_sscan_env);
        ld_sscan_env = NULL;
    }
}

uint8_t ld_sscan_start(struct bd_addr *bd_addr, uint32_t sscan_to, uint16_t sscan_intv, uint16_t sscan_win)
{
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(LD_SSCAN_PATCH_TYPE, LD_SSCAN_START_BIT, uint8_t, bd_addr, sscan_to, sscan_intv, sscan_win);

    DBG_SWDIAG(SSCAN, START, 1);

    // Check if synchronization scan is inactive
    if (ld_sscan_env == NULL)
    {
        // Allocate event
        ld_sscan_env = LD_ALLOC_EVT(ld_sscan_env_tag);

        if (ld_sscan_env != NULL)
        {
            uint32_t clock = ld_read_clock();
            uint8_t bch[LD_BCH_SIZE];
            uint8_t cs_idx = EM_BT_CS_SSCAN_INDEX;

            // Point to parameters
            struct sch_arb_elt_tag *evt = &(ld_sscan_env->evt);
            struct ld_sscan_env_tag *sscan_par = ld_sscan_env;

            LD_INIT_EVT(evt, ld_sscan_env_tag);

            // Set permission/status of CS as R/W but uninitialized
            DBG_MEM_PERM_SET((const void *)(REG_EM_BT_CS_BASE_ADDR + REG_EM_BT_CS_ADDR_GET(cs_idx)), REG_EM_BT_CS_SIZE, true, true, true);

            // Initialize event parameters (common part)
            evt->cb_cancel        = &ld_sscan_evt_canceled_cbk;
            evt->cb_start         = &ld_sscan_evt_start_cbk;
            evt->cb_stop          = NULL;
            evt->current_prio        = rwip_priority[RWIP_PRIO_SSCAN_DFT_IDX].value;

            evt->duration_min = sch_slice_params.scan_evt_dur;

            evt->asap_limit          = CLK_ADD_2(clock, 2 * sscan_to);
            SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_ASAP_LIMIT, SCH_ARB_PHASE_0, 0, 0);

            // Initialize event parameters (synchronization scan part)
            memcpy(&sscan_par->bd_addr.addr[0], &bd_addr->addr[0], BD_ADDR_LEN);
            sscan_par->sscan_intv        = sscan_intv;
            sscan_par->sscan_win         = sscan_win;
            sscan_par->sscan_to          = sscan_to;
            sscan_par->win_rem_slots     = sscan_par->sscan_win;

            // Compute the BCH
            ld_util_bch_create(&bd_addr->addr[0], &bch[0]);

            // Set control structure fields
            em_bt_frcntl_pack(cs_idx, 1 /*fpoll*/, 0 /*fmwspatt*/, RWIP_COEX_GET(BCAST, TXBSY), RWIP_COEX_GET(BCAST, RXBSY), RWIP_COEX_GET(BCAST, DNABORT), 0 /*samen*/, 1 /*lsam_dsb*/, EM_BT_CS_FMT_BCST_SCAN);
            em_bt_bdaddr_setf(cs_idx, 0, (bd_addr->addr[1] << 8) | bd_addr->addr[0]);
            em_bt_bdaddr_setf(cs_idx, 1, (bd_addr->addr[3] << 8) | bd_addr->addr[2]);
            em_bt_bdaddr_setf(cs_idx, 2, (bd_addr->addr[5] << 8) | bd_addr->addr[4]);
            em_bt_bch0_setf(cs_idx, (bch[1] << 8) | bch[0]);
            em_bt_bch1_setf(cs_idx, (bch[3] << 8) | bch[2]);
            em_bt_rxmaxbuf_bch2_pack(cs_idx, DM3_PACKET_SIZE, 0, bch[4] & EM_BT_BCH2_MASK);
            em_bt_txdescptr_setf(cs_idx, 0);
            em_bt_linkcntl_pack(cs_idx,
                                /*aknena */ 0,
                                /*afhena*/ 0,
                                /*laap*/ 0,
                                /*whdsb*/ 1,
                                /*acledr*/ 0,
                                /*aclltaddr*/ LT_ADDR_BCST,
#if (EAVESDROPPING_SUPPORT)
                                /*aclbinackena*/ 0, /*escobinackena*/ 0, /*duplfltena*/ 0,
#endif // EAVESDROPPING_SUPPORT
                                /*linklbl*/ cs_idx);
            em_bt_pwrcntl_pack(cs_idx, /*fhen*/ 0, /*freq*/ 0, /*txpwr*/ 0);
            em_bt_clkoff0_setf(cs_idx, 0);
            em_bt_clkoff1_setf(cs_idx, 0);
            em_bt_txrxcntl_pack(cs_idx, /*rxthr*/ 0,
#if (EAVESDROPPING_SUPPORT)
                                /*stopmod*/ 0,
#endif // EAVESDROPPING_SUPPORT
                                /*rxeir*/ 0, /*rxbcry*/ 0, /*rxcrypt*/ 0, /*stopmod/escofnak*/ 0, /*nullflt*/ 0, /*extpaen*/ 0, /*txeir*/ 0, /*txbcry*/ 0, /*txcrypt*/ 0);
            em_bt_loopcntl_pack(cs_idx, /*tonb*/ 0, /*attnb*/ 1);

            // Default clear other control
            em_bt_chmap0_set(cs_idx, 0);
            em_bt_chmap1_set(cs_idx, 0);
            em_bt_chmap2_set(cs_idx, 0);
            em_bt_chmap3_set(cs_idx, 0);
            em_bt_chmap4_set(cs_idx, 0);
            em_bt_maxfrmtime_set(cs_idx, 0);
#if (EAVESDROPPING_SUPPORT)
            em_bt_cidcntl_set(cs_idx, 0);
            em_bt_rxcrc_set(cs_idx, 0);
#endif // EAVESDROPPING_SUPPORT

            // Schedule event ASAP
            evt->time.hs = clock;

            GLOBAL_INT_DISABLE();

            if (sch_arb_insert(evt) == SCH_ARB_ERROR_OK)
            {
                sscan_par->state = SSCAN_EVT_WAIT;
            }
            else
            {
                ASSERT_ERR_FORCE(0);
            }

            GLOBAL_INT_RESTORE();

            status = CO_ERROR_NO_ERROR;
        }
        else
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            ASSERT_ERR_FORCE(0);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    DBG_SWDIAG(SSCAN, START, 0);

    return (status);
}

uint8_t ld_sscan_stop(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(LD_SSCAN_PATCH_TYPE, LD_SSCAN_STOP_BIT, uint8_t);
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;

    GLOBAL_INT_DISABLE();

    if (ld_sscan_env != NULL)
    {
        // Point to parameters
        struct sch_arb_elt_tag *evt = &(ld_sscan_env->evt);
        struct ld_sscan_env_tag *sscan_par = ld_sscan_env;

        if (SSCAN_EVT_WAIT == sscan_par->state)
        {
            // Remove event
            sch_arb_remove(evt, false);
            // Free event memory
            ld_sscan_cleanup();
        }
        else
        {
            // Move state
            sscan_par->state = SSCAN_EVT_END;
        }

        status = CO_ERROR_NO_ERROR;
    }

    GLOBAL_INT_RESTORE();

    return (status);
}

#endif // CSB_SUPPORT || PCA_SUPPORT

///@} LDSSCAN
