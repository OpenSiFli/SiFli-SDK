/**
****************************************************************************************
*
* @file dbg_swdiag.c
*
* @brief SW profiling module
*
* Copyright (C) RivieraWaves 2009-2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup DBGSWDIAG
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>                  // standard integer definitions
#include "co_utils.h"                // common utility declaration
#include "dbg_swdiag.h"              // SW profiling definition
#include "rwip.h"

#if (RW_SWDIAG)

#if (BT_EMB_PRESENT)
    #include "reg_btcore.h"          // bt core registers
#elif (BLE_EMB_PRESENT)
    #include "reg_blecore.h"         // ble core registers
#endif //BT_EMB_PRESENT / BLE_EMB_PRESENT

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Allocate SW profiles to HW banks
__STATIC uint8_t hw_to_sw[DBG_SWDIAG_NB_HW_BANKS];

/// Allocate HW banks to SW profiles
uint8_t sw_to_hw[DBG_SWDIAG_NB_PROFILES];


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ******************************************************************************************
 * @brief Update SW profile.
 *
 * This function reserves the allocation of SW profiles to HW bank. It allocates to each
 * SW profile a HW bank, depending on the HwToSW allocation table.
 ******************************************************************************************
 */
__STATIC void dbg_swdiag_update(void)
{
    uint8_t i;

    GLOBAL_INT_DISABLE();

    // Clear sw_to_hw table
    for (i = 0 ; i < DBG_SWDIAG_NB_PROFILES ; i++)
    {
        sw_to_hw[i] = DBG_SWDIAG_UNDEFINED;
    }

    // Parse all HW banks
    for (i = 0 ; i < DBG_SWDIAG_NB_HW_BANKS ; i++)
    {
        // Check if a valid profile is allocated to this HW bank
        if (hw_to_sw[i] != DBG_SWDIAG_UNDEFINED)
        {
            ASSERT_INFO(hw_to_sw[i] < DBG_SWDIAG_NB_PROFILES, hw_to_sw[i], DBG_SWDIAG_NB_PROFILES);
            // Allocate the SW profile
            sw_to_hw[hw_to_sw[i]] = i;
        }
    }

    // Clear signals
    SW_DIAG_SET(0);

    GLOBAL_INT_RESTORE();
}

static void dbg_swdiag_ids_init(void)
{
    hw_to_sw[0] = DBG_SWDIAG_IP_ISR;
    hw_to_sw[1] = DBG_SWDIAG_BLE_ISR;
    hw_to_sw[2] = DBG_SWDIAG_SCH_PROG;
    hw_to_sw[3] = DBG_SWDIAG_SLEEP;
}



/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void dbg_swdiag_init(void)
{
    uint8_t length = PARAM_LEN_DIAG_SW;

    // Check SW diags configuration in NVDS
    if (rwip_param.get(PARAM_ID_DIAG_SW, &length, hw_to_sw) != PARAM_OK)
    {
        // Allocation is NULL
        hw_to_sw[0] = DBG_SWDIAG_UNDEFINED;
        hw_to_sw[1] = DBG_SWDIAG_UNDEFINED;
        hw_to_sw[2] = DBG_SWDIAG_UNDEFINED;
        hw_to_sw[3] = DBG_SWDIAG_UNDEFINED;
        dbg_swdiag_ids_init();
    }


    // Update profiles allocation
    dbg_swdiag_update();
}

uint32_t dbg_swdiag_read(void)
{
    return co_read32p(hw_to_sw); //*((uint32_t*) &hw_to_sw[0]);
}

void dbg_swdiag_write(uint32_t profile)
{
    // Write new configuration into global variable
    co_write32p(hw_to_sw, profile);
//    *((uint32_t*) &hw_to_sw[0]) = profile;

    // Update profiles allocation
    dbg_swdiag_update();
}

void btdm_swprofiling_set(uint32_t value)
{
    uint8_t swprofiling_config;
    swprofiling_config = rom_config_get_default_swprofiling();

    if (0 == swprofiling_config)
    {
        ip_swprofiling_set(value);
    }
    else if (1 == swprofiling_config)
    {
        ble_swprofiling_set(value);
    }
    else
    {
        bt_swprofiling_set(value);
    }
}
uint32_t btdm_swprofiling_get(void)
{
    uint32_t value;
    uint8_t swprofiling_config;
    swprofiling_config = rom_config_get_default_swprofiling();

    if (0 == swprofiling_config)
    {
        value = ip_swprofiling_get();
    }
    else if (1 == swprofiling_config)
    {
        value = ble_swprofiling_get();
    }
    else
    {
        value = bt_swprofiling_get();
    }
    return value;
}
#endif //RW_SWDIAG

/// @} DBGSWDIAG
