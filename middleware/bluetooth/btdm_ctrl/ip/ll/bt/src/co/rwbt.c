/**
****************************************************************************************
*
* @file rwbt.c
*
* @brief RWBT core interrupt handler
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup RWBTINT Interrupt Handler
 * @ingroup ROOT
 * @brief The RWBT Interrupt controller.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"          // stack configuration

#include <string.h>          // for mem* functions
#include "co_version.h"
#include "co_math.h"
#include "rwbt.h"            // rwbt interrupt definitions
#include "rwip.h"           // stack main module

#include "bt_util_buf.h"       // BT EM buffer management
#include "ld.h"              // link driver definition

#include "lb.h"              // link broadcast definition
#include "lc.h"              // link controller definition
#include "lm.h"              // link manager definition

#include "ke_event.h"        // kernel event definition

#include "sch_arb.h"         // Scheduling Arbiter
#include "sch_prog.h"        // Scheduling Programmer

#include "dbg.h"             // debug definitions

#include "reg_ipcore.h"      // IP Core registers
#include "reg_btcore.h"      // BT Core registers

#if (EAVESDROPPING_SUPPORT)
    #include "ed.h"
#endif // EAVESDROPPING_SUPPORT
#include "rom_config.h"
#include "btdm_patch.h"

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void rwbt_init(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(RWBT_PATCH_TYPE, RWBT_INIT_FUN_BIT);

    if (rom_config_get_bt_controller_enabled() == 0)
        return;

    // Initialize buffer management system
    bt_util_buf_init(false);

    // Initialize the Link Layer Driver
    ld_init();

    // Initialize the Link Layer Broadcaster
    lb_init();

    // Initialize the Link Layer Controller
    lc_init();

    // Initialize the Link Layer Manager
    lm_init(false);

#if (EAVESDROPPING_SUPPORT)
    // Initialize Eavesdropping
    ed_init();
#endif // EAVESDROPPING_SUPPORT
}

void rwbt_reset(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(RWBT_PATCH_TYPE, RWBT_RESET_FUN_BIT);

    if (rom_config_get_bt_controller_enabled() == 0)
        return;

    // Initialize buffer management system
    bt_util_buf_init(true);

    // Reset the BT core
    ld_reset();

    // Initialize the Link Layer Broadcaster
    lb_reset();

    // Reset the Link Layer Controller
    lc_reset();

    // Reset the Link Layer Manager
    lm_init(true);

    // Turn on BT Core
    bt_rwbtcntl_rwbten_setf(1);

#if (EAVESDROPPING_SUPPORT)
    // Reset Eavesdropping
    ed_reset();
#endif // EAVESDROPPING_SUPPORT
}

__BTIRQ void bt_isr(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(RWBT_PATCH_TYPE, BT_ISR_FUN_BIT);
    DBG_SWDIAG(ISR, BT, 1);

    // Check BT interrupt status and call the appropriate handlers
    uint32_t irq_stat = bt_intstat0_get();

    // Error interrupt should be checked first
    if (irq_stat & BT_ERRORINTSTAT_BIT)
    {
        // Clear the interrupt
        bt_intack0_errorintack_clearf(1);

#if !RW_DEBUG
        // Report the HW error
        rwip_hw_error_report(CO_ERROR_HW_BT_ERRORINTSTAT, bt_errortypestat_get());
#endif // !RW_DEBUG

        ASSERT_INFO_FORCE(0, bt_errortypestat_get(), (bt_errortypestat_get() >> 16));
    }

#if VOICE_OVER_HCI
    // Audio interrupt 0
    if (irq_stat & BT_AUDIOINT0STAT_BIT)
    {
        // Clear the interrupt
        bt_intack0_audioint0ack_clearf(1);

#if (EAVESDROPPING_SUPPORT)
        // Try to handle eavesdropping audio ISR first
        if (!ld_ed_sco_audio_isr())
#endif // EAVESDROPPING_SUPPORT
        {
            // Handle audio IRQ
            ld_sco_audio_isr(0);
        }
    }

    // Audio interrupt 1
    if (irq_stat & BT_AUDIOINT1STAT_BIT)
    {
        // Clear the interrupt
        bt_intack0_audioint1ack_clearf(1);

        // Handle audio IRQ
        ld_sco_audio_isr(1);
    }

    // Audio interrupt 2
    if (irq_stat & BT_AUDIOINT2STAT_BIT)
    {
        // Clear the interrupt
        bt_intack0_audioint2ack_clearf(1);

        // Handle audio IRQ
        ld_sco_audio_isr(2);
    }
#endif //VOICE_OVER_HCI

#if RW_BT_MWS_COEX
    // MWS FRAME_SYNC interrupt pending.
    if (irq_stat & BT_FRSYNCINTSTAT_BIT)
    {
        bt_intack0_frsyncintack_clearf(1);
        // Handle the IRQ
        ld_pca_mws_frame_sync();
    }
#endif // RW_BT_MWS_COEX
#if PCA_SUPPORT
    // Momentary offset interrupt pending. The newly calculated momentary offset is greater than the correction step.
    if (irq_stat & BT_MTOFFINT0STAT_BIT)
    {
        // Clear the interrupt
        bt_intack0_mtoffint0ack_clearf(1);
        // Handle the IRQ
        ld_pca_mws_moment_offset_gt();
    }
    // Momentary offset interrupt pending. The newly calculated momentary offset is lower than the correction step.
    if (irq_stat & BT_MTOFFINT1STAT_BIT)
    {
        // Clear the interrupt
        bt_intack0_mtoffint1ack_clearf(1);
        // Handle the IRQ
        ld_pca_mws_moment_offset_lt();
    }
#endif // PCA_SUPPORT

#if (EAVESDROPPING_SUPPORT)
    // ED MIC error interrupt
    if (irq_stat & BT_EDMICERRINTSTAT_BIT)
    {
        DBG_SWDIAG(BT_ISR_ED, BINACK, 1);
        // Clear the interrupt
        bt_intack0_edmicerrintack_clearf(1);

        // EDMICERR interrupt
        ld_acl_micerr_isr(); // EDE
        ld_ed_micerr_isr(); // ED

        DBG_SWDIAG(BT_ISR_ED, BINACK, 0);
    }

    // Clock capture interrupt
    if (irq_stat & BT_CLKCAPINTSTAT_BIT)
    {
        DBG_SWDIAG(BT_ISR_ED, CLKCAP, 1);
        // Clear the interrupt
        bt_intack0_clkcapintack_clearf(1);

        // Call capture interrupt
        ld_clock_captured_isr();

        DBG_SWDIAG(BT_ISR_ED, CLKCAP, 0);
    }

    // Clock set interrupt
    if (irq_stat & BT_CLKSETINTSTAT_BIT)
    {
        DBG_SWDIAG(BT_ISR_ED, CLKSET, 1);
        // Clear the interrupt
        bt_intack0_clksetintack_clearf(1);
        // Call set interrupt
        ld_clock_adjusted_isr();

        DBG_SWDIAG(BT_ISR_ED, CLKSET, 0);
    }
#endif // EAVESDROPPING_SUPPORT

    DBG_SWDIAG(ISR, BT, 0);
}

///@} RWBTINT
