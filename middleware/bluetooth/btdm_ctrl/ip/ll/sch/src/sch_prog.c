/**
****************************************************************************************
*
* @file sch_prog.c
*
* @brief Scheduling Programmer source code
*
* Copyright (C) RivieraWaves 2009-2017
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup SCH_PROG
 * @ingroup SCH
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <string.h>

#include "arch.h"
#include "co_list.h"
#include "co_math.h"

#include "rwip.h"
#include "dbg.h"

#include "reg_em_et.h"       // EM Exchange Table

#include "reg_ipcore.h"      // DM core registers
#if BLE_EMB_PRESENT
    #include "reg_em_ble_cs.h"   // EM Control Structure
    #include "reg_blecore.h"     // for ble_leschcntl_pack
#endif // BLE_EMB_PRESENT

#if BT_EMB_PRESENT
    #include "reg_em_bt_cs.h"   // EM Control Structure
    #include "reg_btcore.h"     // for bt_leschcntl_pack
#endif // BT_EMB_PRESENT

#include "sch_prog.h"        // link layer driver Scheduling Programmer
#include "btdm_patch.h"


/*
 * DEFINES
 *****************************************************************************************
 */

/// Mask used for wrapping ET indexes
#define ET_IDX_MASK              (REG_EM_ET_SIZE - 1)

/// Maximum number of programmed entries before forcing an abort of the ongoing frame
#define SCH_PROG_ABORT_THR       (REG_EM_ET_SIZE - 2)
/// Maximum priority level
#define SCH_PROG_PRIO_MAX        (EM_SCH_PRIO1_MASK >> EM_SCH_PRIO1_LSB)

/*
 * MACROS
 *****************************************************************************************
 */

// align name of HW/SW interface macros
#if BT_EMB_PRESENT
    #define em_extab_status_getf em_bt_extab_status_getf
    #define em_extab_status_setf em_bt_extab_status_setf
#elif BLE_EMB_PRESENT
    #define em_extab_status_getf em_le_extab_status_getf
    #define em_extab_status_setf em_le_extab_status_setf
#endif // BLE_EMB_PRESENT

/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

#if BT_EMB_PRESENT
/// Structure of a clock element
struct sch_prog_fm_clk_elt
{
    /// Callback for handling interrupts related to the clock
    clk_cbk_t clk_cbk;
    /// Identification value
    uint8_t id;
};
#endif // BT_EMB_PRESENT

/// Structure of a frame element
struct sch_prog_frm_elt
{
    /// Timestamp of the programmed frame (in BLE half slots, based on local clock)
    uint32_t timestamp;
    /// Callback for handling interrupts related to the frame
    frm_cbk_t frm_cbk;
    /// Dummy value (to be reported to the driver)
    uint32_t dummy;
    /// Indicate if the frame is valid (programmed and not skipped or finished)
    bool valid;
};

/// SCH_PROG environment structure
struct sch_prog_env_tag
{
    /// Frame elements pool
    struct sch_prog_frm_elt tab[REG_EM_ET_SIZE];

    /// Exchange table index of the oldest entry currently used by the HW
    uint8_t et_idx_current;

    /// Next exchange table index to program
    uint8_t et_idx_next_prog;

    /// Number of programmed frames
    uint8_t nb_prog;

#if BT_EMB_PRESENT
    /// Pointer to the clock element
    struct sch_prog_fm_clk_elt clk_elt;
#endif // BT_EMB_PRESENT
};


/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */

/// SCH_PROG environment variable
__STATIC struct sch_prog_env_tag sch_prog_env;

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handle "RX" interrupt
 *
 * @param[in] et_idx Entry index that trigger the ISR (@note Valid only with 5.0 + ISO HW)
 ****************************************************************************************
 */
void sch_prog_rx_isr(uint8_t et_idx);

/**
 ****************************************************************************************
 * @brief Handle "TX" interrupt
 *
 * @param[in] et_idx Entry index that trigger the ISR (@note Valid only with 5.0 + ISO HW)
 ****************************************************************************************
 */
void sch_prog_tx_isr(uint8_t et_idx);


#if (BLE_ISO_PRESENT)
    /**
    ****************************************************************************************
    * @brief Handle "RX ISO" interrupt
    *
    * @param[in] et_idx Entry index that trigger the ISR (@note Valid only with 5.0 + ISO HW)
    ****************************************************************************************
    */
    void sch_prog_rx_iso_isr(uint8_t et_idx);

    /**
    ****************************************************************************************
    * @brief Handle "TX ISO" interrupt
    *
    * @param[in] et_idx Entry index that trigger the ISR (@note Valid only with 5.0 + ISO HW)
    ****************************************************************************************
    */
    void sch_prog_tx_iso_isr(uint8_t et_idx);
#endif // (BLE_ISO_PRESENT)


/**
 ****************************************************************************************
 * @brief Handle "Skip" interrupt
 *
 * @param[in] et_idx Entry index that trigger the ISR (@note Valid only with 5.0 + ISO HW)
 ****************************************************************************************
 */
void sch_prog_skip_isr(uint8_t et_idx);

/**
 ****************************************************************************************
 * @brief Handle "End of Event" interrupt
 *
 * @param[in] et_idx Entry index that trigger the ISR (@note Valid only with 5.0 + ISO HW)
 ****************************************************************************************
 */
void sch_prog_end_isr(uint8_t et_idx);

/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void sch_prog_rx_isr(uint8_t et_idx)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_PROG_PATCH_TYPE, SCH_PROG_RX_ISR_FUNC_BIT, et_idx);

    DBG_SWDIAG(SCH_PROG, RXTX_ISR, 1);

    ASSERT_INFO(et_idx == sch_prog_env.et_idx_current, et_idx, sch_prog_env.et_idx_current);

    // Check if there is at least one entry programmed
    ASSERT_ERR(sch_prog_env.nb_prog > 0);

    // Check that exchange table entry is valid
    ASSERT_ERR(sch_prog_env.tab[et_idx].valid);

    if (sch_prog_env.tab[et_idx].frm_cbk != NULL)
    {
        // Invoke frame callback
        sch_prog_env.tab[et_idx].frm_cbk(sch_prog_env.tab[et_idx].timestamp,
                                         sch_prog_env.tab[et_idx].dummy,
                                         SCH_FRAME_IRQ_RX);
    }

    DBG_SWDIAG(SCH_PROG, RXTX_ISR, 0);
}

void sch_prog_tx_isr(uint8_t et_idx)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_PROG_PATCH_TYPE, SCH_PROG_TX_ISR_FUNC_BIT, et_idx);

    DBG_SWDIAG(SCH_PROG, RXTX_ISR, 1);

    ASSERT_INFO(et_idx == sch_prog_env.et_idx_current, et_idx, sch_prog_env.et_idx_current);

    // Check if there is at least one entry programmed
    ASSERT_ERR(sch_prog_env.nb_prog > 0);

    // Check that exchange table entry is valid
    ASSERT_ERR(sch_prog_env.tab[et_idx].valid);

    if (sch_prog_env.tab[et_idx].frm_cbk != NULL)
    {
        // Invoke frame callback
        sch_prog_env.tab[et_idx].frm_cbk(sch_prog_env.tab[et_idx].timestamp,
                                         sch_prog_env.tab[et_idx].dummy,
                                         SCH_FRAME_IRQ_TX);
    }

    DBG_SWDIAG(SCH_PROG, RXTX_ISR, 0);
}

#if (BLE_ISO_PRESENT)
void sch_prog_rx_iso_isr(uint8_t et_idx)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_PROG_PATCH_TYPE, SCH_PROG_RX_ISO_ISR_FUNC_BIT, et_idx);

    ASSERT_INFO(et_idx == sch_prog_env.et_idx_current, et_idx, sch_prog_env.et_idx_current);

    // Check that exchange table entry is valid
    ASSERT_ERR(sch_prog_env.tab[et_idx].valid);

    if (sch_prog_env.tab[et_idx].frm_cbk != NULL)
    {
        // Invoke frame callback
        sch_prog_env.tab[et_idx].frm_cbk(sch_prog_env.tab[et_idx].timestamp,
                                         sch_prog_env.tab[et_idx].dummy, SCH_FRAME_IRQ_RX_ISO);
    }
}

void sch_prog_tx_iso_isr(uint8_t et_idx)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_PROG_PATCH_TYPE, SCH_PROG_TX_ISO_ISR_FUNC_BIT, et_idx);

    ASSERT_INFO(et_idx == sch_prog_env.et_idx_current, et_idx, sch_prog_env.et_idx_current);

    // Check that exchange table entry is valid
    ASSERT_ERR(sch_prog_env.tab[et_idx].valid);

    if (sch_prog_env.tab[et_idx].frm_cbk != NULL)
    {
        // Invoke frame callback
        sch_prog_env.tab[et_idx].frm_cbk(sch_prog_env.tab[et_idx].timestamp,
                                         sch_prog_env.tab[et_idx].dummy, SCH_FRAME_IRQ_TX_ISO);
    }
}
#endif // (BLE_ISO_PRESENT)

void sch_prog_skip_isr(uint8_t et_idx)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_PROG_PATCH_TYPE, SCH_PROG_SKIP_ISR_FUNC_BIT, et_idx);

    DBG_SWDIAG(SCH_PROG, SKIP, 1);

    // Check if there is at least this one entry programmed
    ASSERT_ERR(sch_prog_env.nb_prog > 0);

    // Retrieve status
    uint8_t et_status = em_extab_status_getf(et_idx);

    // Check that exchange table entry is valid, skipped
    if ((sch_prog_env.tab[et_idx].valid) && ((EM_ET_STATUS_SKIPPED == et_status) || (EM_ET_STATUS_SKIPPED_WAITING_TIME == et_status)))
    {
        if (sch_prog_env.tab[et_idx].frm_cbk != NULL)
        {
            // Invoke frame callback
            sch_prog_env.tab[et_idx].frm_cbk(sch_prog_env.tab[et_idx].timestamp,
                                             sch_prog_env.tab[et_idx].dummy,
                                             SCH_FRAME_IRQ_SKIP);
        }
        else
        {
            ASSERT_ERR_FORCE(0);
        }

        // If current index is skipped, the current index is moved to the next entry
        if (et_idx == sch_prog_env.et_idx_current)
        {
            sch_prog_env.et_idx_current = (sch_prog_env.et_idx_current + 1) & ET_IDX_MASK;
        }

        // Invalidate entry element
        sch_prog_env.tab[et_idx].valid = false;

        sch_prog_env.nb_prog--;
        DBG_SWDIAG(SCH_PROG, NB_PROG, sch_prog_env.nb_prog);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }

    // If one entry remaining
    if (sch_prog_env.nb_prog == 1)
    {
        // Reuse the skipped entry for next programming
        sch_prog_env.et_idx_next_prog = (sch_prog_env.et_idx_current + 1) & ET_IDX_MASK;
    }
    // If no more entry
    else if (sch_prog_env.nb_prog == 0)
    {
        // Re-allow deep sleep
        rwip_prevent_sleep_clear(RW_BB_FRAME_ONGOING);
    }

    DBG_SWDIAG(SCH_PROG, SKIP, 0);
}

void sch_prog_end_isr(uint8_t et_idx)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_PROG_PATCH_TYPE, SCH_PROG_END_ISR_FUNC_BIT, et_idx);

    DBG_SWDIAG(SCH_PROG, END_ISR, 1);

    ASSERT_INFO(et_idx == sch_prog_env.et_idx_current, et_idx, sch_prog_env.et_idx_current);

    {
        // Retrieve status
        uint8_t et_status = em_extab_status_getf(et_idx);
        uint8_t irq_type;

        // Check if there is at least one entry programmed
        ASSERT_ERR(sch_prog_env.nb_prog > 0);

        // Check that exchange table entry is valid
        ASSERT_INFO(sch_prog_env.tab[et_idx].valid, et_status, et_idx);

        // Check that entry if ended
        ASSERT_ERR((et_status == EM_ET_STATUS_TERM_NORMAL) || (et_status == EM_ET_STATUS_TERM_ABORT_IN_PRIO_BW) || (et_status == EM_ET_STATUS_TERM_ABORT));

        //Trace the completed frame
        TRC_REQ_FRM_CMP(sch_prog_env.tab[et_idx].timestamp, sch_prog_env.tab[et_idx].dummy, (uint32_t) sch_prog_env.tab[et_idx].frm_cbk);

        if (et_status == EM_ET_STATUS_TERM_NORMAL)
        {
            irq_type = SCH_FRAME_IRQ_EOF;
        }
        else if (et_status == EM_ET_STATUS_TERM_ABORT_IN_PRIO_BW)
        {
            irq_type = SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO;
        }
        else if (et_status == EM_ET_STATUS_TERM_ABORT)
        {
            irq_type = SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO;
        }
        else
        {
            irq_type = SCH_FRAME_IRQ_ERROR;
        }

        if (sch_prog_env.tab[et_idx].frm_cbk != NULL)
        {
            // Invoke frame callback
            sch_prog_env.tab[et_idx].frm_cbk(sch_prog_env.tab[et_idx].timestamp,
                                             sch_prog_env.tab[et_idx].dummy,
                                             irq_type);
        }
        else
        {
            ASSERT_ERR_FORCE(0);
        }

        // Invalidate entry element
        sch_prog_env.tab[et_idx].valid = false;

        // Subtract the number of completed frames
        sch_prog_env.nb_prog--;
        DBG_SWDIAG(SCH_PROG, NB_PROG, sch_prog_env.nb_prog);

        // Search from the next entry that was not skipped
        while ((!sch_prog_env.tab[et_idx].valid) && (et_idx != sch_prog_env.et_idx_next_prog))
        {
            et_idx = (et_idx + 1) & ET_IDX_MASK;
        }

        // Place current index
        sch_prog_env.et_idx_current = et_idx;
    }

    // Check if a frame is still pending
    if (sch_prog_env.nb_prog == 0)
    {
        // Re-allow deep sleep
        rwip_prevent_sleep_clear(RW_BB_FRAME_ONGOING);
    }

    DBG_SWDIAG(SCH_PROG, END_ISR, 0);
}


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void sch_prog_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_PROG_PATCH_TYPE, SCH_PROG_INIT_FUNC_BIT, is_reset);
    uint8_t et_idx;
    memset(&sch_prog_env, 0, sizeof(sch_prog_env));

    DBG_SWDIAG(SCH_PROG, NB_PROG, sch_prog_env.nb_prog);

    // Initialize the exchange table
    for (et_idx = 0 ; et_idx < REG_EM_ET_SIZE ; et_idx++)
    {
        em_extab_status_setf(et_idx, EM_ET_STATUS_TERM_NORMAL);
    }
}

void sch_prog_fifo_isr(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(SCH_PROG_PATCH_TYPE, SCH_PROG_FIFO_ISR_FUNC_BIT);

    // Fetch interrupt FIFO
    uint32_t actfifostat = ip_actfifostat_get();
    uint8_t curr_et_idx = GETF(actfifostat, IP_CURRENT_ET_IDX);
    uint8_t skip_et_idx = GETF(actfifostat, IP_SKIP_ET_IDX);

    // TX
    if (actfifostat & IP_TXINTSTAT_BIT)
    {
        DBG_SWDIAG(BLE_ISR, TXINT, 1);

        // Call handler
        sch_prog_tx_isr(curr_et_idx);

        DBG_SWDIAG(BLE_ISR, TXINT, 0);
    }
    // RX
    if (actfifostat & IP_RXINTSTAT_BIT)
    {
        DBG_SWDIAG(BLE_ISR, RXINT, 1);

        // Call handler
        sch_prog_rx_isr(curr_et_idx);

        DBG_SWDIAG(BLE_ISR, RXINT, 0);
    }

#if (BLE_ISO_PRESENT)
    // RX ISO interrupt
    if (actfifostat & IP_ISORXINTSTAT_BIT)
    {
        DBG_SWDIAG(BLE_ISR, RXISOINT, 1);

        // Call handler
        sch_prog_rx_iso_isr(curr_et_idx);

        DBG_SWDIAG(BLE_ISR, RXISOINT, 0);
    }
    // TX ISO interrupt
    if (actfifostat & IP_ISOTXINTSTAT_BIT)
    {
        DBG_SWDIAG(BLE_ISR, TXISOINT, 1);

        // Call handler
        sch_prog_tx_iso_isr(curr_et_idx);

        DBG_SWDIAG(BLE_ISR, TXISOINT, 0);
    }
#endif // (BLE_ISO_PRESENT)

    // Skip event
    if (actfifostat & IP_SKIPACTINTSTAT_BIT)
    {
        DBG_SWDIAG(BLE_ISR, SKIPINT, 1);

        // Handle skip interrupt
        sch_prog_skip_isr(skip_et_idx);

        DBG_SWDIAG(BLE_ISR, SKIPINT, 0);
    }

    // End of event
    if (actfifostat & IP_ENDACTINTSTAT_BIT)
    {
        DBG_SWDIAG(BLE_ISR, EVENTINT, 1);

        // Handle end of frame
        sch_prog_end_isr(curr_et_idx);

        DBG_SWDIAG(BLE_ISR, EVENTINT, 0);
    }
}

void sch_prog_push(struct sch_prog_params *params)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_PROG_PATCH_TYPE, SCH_PROG_PUSH_FUNC_BIT, params);

    DBG_SWDIAG(SCH_PROG, PROG_PUSH, 1);

    ASSERT_WARN(((params->prio_1 != RWIP_PRIO_MAX) && (params->prio_2 != RWIP_PRIO_MAX) && (params->prio_3 != RWIP_PRIO_MAX)), params->cs_idx, params->dummy);

    // Keep the 5 most significant bits of prio_1/prio_2/prio_3 (reduce the range from [0:255] to [0:31]
    params->prio_1 = params->prio_1 >> 3;
    params->prio_2 = params->prio_2 >> 3;
    params->prio_3 = params->prio_3 >> 3;

    // Get next table entry
    uint8_t et_idx = sch_prog_env.et_idx_next_prog;

    // Check if previous activity must be stopped
    bool stop_prev_act = (((sch_prog_env.et_idx_next_prog - sch_prog_env.et_idx_current) & ET_IDX_MASK) >= SCH_PROG_ABORT_THR);

    // Check if there is a free entry to program the new frame
    ASSERT_INFO(!(sch_prog_env.nb_prog && (sch_prog_env.et_idx_next_prog == sch_prog_env.et_idx_current)), sch_prog_env.et_idx_current, sch_prog_env.et_idx_next_prog);

    // Check that Exchange Table entry is not under processing by HW
    ASSERT_INFO((em_extab_status_getf(et_idx) != EM_ET_STATUS_UNDER_PROCESS)
                && (em_extab_status_getf(et_idx) != EM_ET_STATUS_STARTED),
                sch_prog_env.et_idx_current, et_idx);

    // Warning if the exchange table is close to overflow
    ASSERT_WARN(!stop_prev_act, em_extab_status_getf((et_idx - 2) & ET_IDX_MASK), em_extab_status_getf((et_idx - 1) & ET_IDX_MASK));

    // Store information about the new frame
    sch_prog_env.tab[et_idx].timestamp = params->time.hs;
    sch_prog_env.tab[et_idx].dummy = params->dummy;
    sch_prog_env.tab[et_idx].frm_cbk = params->frm_cbk;
    sch_prog_env.tab[et_idx].valid = true;

    // Program ET
    em_rawstp0_setf(et_idx, params->time.hs & EM_RAWSTP0_MASK);
    em_rawstp1_setf(et_idx, (params->time.hs >> 16) & EM_RAWSTP1_MASK);
    ASSERT_ERR(params->time.hus < HALF_SLOT_SIZE);
    em_finestp_setf(et_idx, HALF_SLOT_TIME_MAX - params->time.hus);

    if (params->bandwidth > (EM_PRIO1D_MASK >> EM_PRIO1D_LSB))
    {
        em_priobw_pack(et_idx, /*prio1dunit*/ 1, /*prio1d*/ ((params->bandwidth + HALF_SLOT_SIZE) / HALF_SLOT_SIZE)); // converted to half-slots
    }
    else
    {
        em_priobw_pack(et_idx, /*prio1dunit*/ 0, /*prio1d*/ ((params->bandwidth + 1) >> 1)); // converted to us
    }

    em_priolvl_pack(et_idx, /*sch_prio3*/ co_min(params->prio_3, SCH_PROG_PRIO_MAX), /*sch_prio2*/ co_min(params->prio_2, SCH_PROG_PRIO_MAX));

    em_pti_vxchan_pti_prio_setf(et_idx, params->pti_prio);

#if BLE_EMB_PRESENT
    if (params->mode == SCH_PROG_BLE)
    {
        em_csptr_setf(et_idx, REG_EM_BLE_CS_ADDR_GET(params->cs_idx) >> 2);

        if (params->add.ble.iso)
        {
            em_le_extab_pack(et_idx, /*schprio1*/ co_min(params->prio_1, SCH_PROG_PRIO_MAX), /*spa*/ stop_prev_act,
                             /* sic */ params->add.ble.sic,
                             /*aenps*/ params->add.ble.ae_nps,
                             /*rsvd*/ params->add.ble.rsvd, /*iso*/ 1,
                             /*status*/ EM_ET_STATUS_READY, /*mode*/ EM_ET_MODE_BLE);
        }
        else
        {
            em_le_extab_pack(et_idx, /*schprio1*/ co_min(params->prio_1, SCH_PROG_PRIO_MAX), /*spa*/ stop_prev_act,
                             /* sic */ 0,
                             /*aenps*/ params->add.ble.ae_nps,
                             /*rsvd*/ 0, /*iso*/ 0,
                             /*status*/ EM_ET_STATUS_READY, /*mode*/ EM_ET_MODE_BLE);
        }

        // Trace the control structure
        TRC_REQ_CS_BLE(REG_EM_BLE_CS_ADDR_GET(params->cs_idx));
    }
#endif //BLE_EMB_PRESENT

#if BT_EMB_PRESENT
    if (params->mode == SCH_PROG_BT)
    {
        uint8_t frm_type = params->add.bt.frm_type;
        uint8_t vxchan = ((frm_type == SCH_BT_FRAME_TYPE_ESCO) || (frm_type == SCH_BT_FRAME_TYPE_ESCO_RETX)) ? params->add.bt.vxchan : 0;

        em_csptr_setf(et_idx, REG_EM_BT_CS_ADDR_GET(params->cs_idx) >> 2);

        em_pti_vxchan_vxchan_setf(et_idx, vxchan);

        em_bt_extab_pack(et_idx, /*schprio1*/ co_min(params->prio_1, SCH_PROG_PRIO_MAX), /*spa*/ stop_prev_act,
                         /*csb*/ (frm_type == SCH_BT_FRAME_TYPE_CSB), /*sniff*/ (frm_type == SCH_BT_FRAME_TYPE_SNIFF),
                         /*rsvd*/ (frm_type == SCH_BT_FRAME_TYPE_ESCO), /*esco*/ ((frm_type == SCH_BT_FRAME_TYPE_ESCO) || (frm_type == SCH_BT_FRAME_TYPE_ESCO_RETX)),
                         /*status*/ EM_ET_STATUS_READY, /*mode*/ EM_ET_MODE_BREDR);

        // Trace the control structure
        TRC_REQ_CS_BT(REG_EM_BT_CS_ADDR_GET(params->cs_idx));
    }
#endif //BT_EMB_PRESENT
    rwip_rf.txpwr_cs_update(params->mode, params->cs_idx, et_idx);

    // Inform that new entry is available
    ip_actschcntl_pack(true, et_idx);

    //Trace the programming of the ET
    TRC_REQ_ET_PROG(et_idx);

    // Increment SW index
    sch_prog_env.et_idx_next_prog = (sch_prog_env.et_idx_next_prog + 1) & ET_IDX_MASK;
    sch_prog_env.nb_prog++;
    DBG_SWDIAG(SCH_PROG, NB_PROG, sch_prog_env.nb_prog);

    // Disallow deep sleep
    rwip_prevent_sleep_set(RW_BB_FRAME_ONGOING);

    DBG_SWDIAG(SCH_PROG, PROG_PUSH, 0);
}

#if BT_EMB_PRESENT
void sch_prog_enable(clk_cbk_t clk_cbk, uint8_t id, uint8_t sr_size, uint8_t sr_val)
{
    FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(SCH_PROG_PATCH_TYPE, SCH_PROG_ENABLE_FUNC_BIT, clk_cbk, id, sr_size, sr_val);

    if (sch_prog_env.clk_elt.clk_cbk == NULL)
    {
        // Store programming element
        sch_prog_env.clk_elt.clk_cbk = clk_cbk;
        sch_prog_env.clk_elt.id = id;
        ip_intcntl1_clknintsrmsk_setf(sr_size);
        ip_intcntl1_clknintsrval_setf(sr_val);
        ip_intack1_clknintack_clearf(1);
        ip_intcntl1_clknintmsk_setf(1);
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

void sch_prog_disable(uint8_t id)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_PROG_PATCH_TYPE, SCH_PROG_DISABLE_FUNC_BIT, id);

    if (sch_prog_env.clk_elt.id == id)
    {
        // Clear programming element
        sch_prog_env.clk_elt.clk_cbk = NULL;
        ip_intcntl1_clknintmsk_setf(0);
    }
    else
    {
        ASSERT_INFO_FORCE(0, id, sch_prog_env.clk_elt.id);
    }
}

void sch_prog_clk_isr(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(SCH_PROG_PATCH_TYPE, SCH_PROG_CLK_ISR_FUNC_BIT);

    // Check the current programming element
    if (sch_prog_env.clk_elt.clk_cbk != NULL)
    {
        // Read clock (not rounded)
        uint32_t clock = rwip_time_get().hs;

        // Invoke programming element clock callback
        sch_prog_env.clk_elt.clk_cbk(clock, sch_prog_env.clk_elt.id);
    }
}
#endif //BT_EMB_PRESENT


///@} SCH_PROG
