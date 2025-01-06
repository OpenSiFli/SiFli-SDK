/**
****************************************************************************************
*
* @file rwble.c
*
* @brief RWBLE core interrupt handler
*
* Copyright (C) RivieraWaves 2009-2016
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"      // stack configuration

#include <string.h>           // for mem* functions
#include "co_version.h"
#include "co_math.h"
#include "rwble.h"            // BLE API definition
#include "rwip.h"             // stack main module

#include "ble_util_buf.h"     // BLE EM buffer management
#include "lld.h"              // link layer driver definition
#include "llc.h"              // link layer controller definition
#include "llm.h"              // link layer manager definition

#if (BLE_ISO_PRESENT)
    #include "lli.h"              // Link Layer ISO definition
#endif // (BLE_ISO_PRESENT)

#include "ke_event.h"         // kernel event definition

#include "sch_arb.h"          // Scheduling Arbiter
#include "sch_prog.h"         // Scheduling Programmer

#include "dbg.h"              // debug definitions

#include "reg_blecore.h"      // BLE Core registers
#include "btdm_patch.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLES
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

void rwble_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(RWBLE_PATCH_TYPE, RWBLE_INIT_FUN_BIT, is_reset);

    if (rom_config_get_ble_controller_enabled() == 0)
    {
        // Enable the BLE core    used for timer
        ble_rwblecntl_rwble_en_setf(1);
        return;
    }
    // Initialize buffer management system
    ble_util_buf_init(is_reset);

    // Initialize the Link Layer Driver
    lld_init(is_reset);

#if(BLE_CENTRAL || BLE_PERIPHERAL)
    // Initialize the Link Layer Controller
    llc_init(is_reset);
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

    // Initialize the Link Layer Manager
    llm_init(is_reset);

#if (BLE_ISO_PRESENT)
    // Initialize the Link Layer ISO
    lli_init(is_reset);
#endif // (BLE_ISO_PRESENT)
}

#if BT_DUAL_MODE
bool rwble_activity_ongoing_check(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(RWBLE_PATCH_TYPE, RWBLE_ACTIVITY_ONGOING_CHECK_FUN_BIT, bool);
    // check that a BLE activity is ongoing (advertising, scan, initiating, connection)
    return llm_activity_ongoing_check();
}
#endif //BT_DUAL_MODE

__BLEIRQ void ble_isr(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(RWBLE_PATCH_TYPE, BLE_ISR_FUN_BIT);
    DBG_SWDIAG(ISR, BLE, 1);

    // Check BLE interrupt status and call the appropriate handlers
    uint32_t irq_stat      = ble_intstat0_get();

    // Error interrupt should be checked first
    if (irq_stat & BLE_ERRORINTSTAT_BIT)
    {
        // Clear the interrupt
        ble_intack0_errorintack_clearf(1);

#if !RW_DEBUG
        // Report the HW error
        rwip_hw_error_report(CO_ERROR_HW_BLE_ERRORINTSTAT, ble_errortypestat_get());
#endif // !RW_DEBUG

        ASSERT_INFO_FORCE(0, ble_errortypestat_get(), (ble_errortypestat_get() >> 16));
    }

#if (BLE_BIS | BLE_CIS)
    // Hopping scheme computation
    if (irq_stat & BLE_HOPINTACK_BIT)
    {
        // Clear the interrupt
        ble_intack0_hopintack_clearf(1);

        // Handle end of hopping HW accelerator
        lld_iso_hop_isr();
    }
#endif // (BLE_BIS | BLE_CIS)

#if (EAVESDROPPING_SUPPORT)
    // Advertising trigger interrupt
    if (irq_stat & BLE_ADVTRIGINTSTAT_BIT)
    {
        // Clear the interrupt
        ble_intack0_advtrigintack_clearf(1);

        // Handle advertising trigger
        lld_adv_trig_isr();
    }
#endif // EAVESDROPPING_SUPPORT

    DBG_SWDIAG(ISR, BLE, 0);
}

///@} RWBTINT
