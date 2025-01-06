/**
****************************************************************************************
*
* @file sch_slice.c
*
* @brief LD scheduling parameters source code
*
* Copyright (C) RivieraWaves 2009-2017
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup SCHSLICE
 * @ingroup LD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration
#include <string.h>
#include <stdbool.h>
#include "co_math.h"
#include "co_utils.h"
#include "arch.h"
#include "rwip.h"

#include "sch_slice.h"       // Scheduling Slicer


#include "btdm_patch.h"


/*
 * DEFINES
 *****************************************************************************************
 */

/// Default scan event duration (in half-us) - also applicable for inquiry, page, HDC adv
#define SCH_SCAN_EVT_DUR_DFT     (18*2*HALF_SLOT_SIZE)

#if BT_EMB_PRESENT
    /// Default ACL event duration (in half-us)
    #define SCH_ACL_EVT_DUR_DFT      (15*HALF_SLOT_SIZE)
#endif //BT_EMB_PRESENT


/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */


/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

/// SCH SLICE periodic activity parameters structure
struct sch_slice_per_tag
{
    /// Activity interval (in half-slots), 0 if not used
    uint32_t intv;

    /// Activity duration (in half-us)
    uint16_t duration;

    /// Potential retransmissions (applicable to SCO only)
    bool retx;
};

/// SCH SLICE environment structure
struct sch_slice_env_tag
{
    /// Bit field for access modes (@see enum activity_type)
    uint8_t access_modes;

#if BLE_EMB_PRESENT
    /// End of ongoing HDC adv activity(in BT half-slot clock, SCH_CLOCK_UNDEF if inactive)
    uint32_t adv_end_ts;

    /// BLE connections or periodic advertising (bit field indicating presence of the activity)
    uint32_t ble_bf;

    /// BLE connections or periodic advertising parameters
    struct sch_slice_per_tag ble[BLE_ACTIVITY_MAX];
#endif //BLE_EMB_PRESENT

#if BT_EMB_PRESENT
    /// End of ongoing page activity (in BT half-slot clock, SCH_CLOCK_UNDEF if inactive)
    uint32_t page_end_ts;

    /// End of ongoing inquiry activity (in BT half-slot clock, SCH_CLOCK_UNDEF if inactive)
    uint32_t inq_end_ts;

    /// Sniff links (bit field indicating presence of the activity)
    uint32_t sniff_bf;

    /// Sniff links parameters
    struct sch_slice_per_tag sniff[MAX_NB_ACTIVE_ACL];

#if (MAX_NB_SYNC > 0)
    /// SCO links (bit field indicating presence of the activity)
    uint32_t sco_bf;

    /// SCO links
    struct sch_slice_per_tag sco[MAX_NB_SYNC];
#endif // (MAX_NB_SYNC > 0)
#endif //BT_EMB_PRESENT

    /// Default Scan / Inq / Page event duration (in half-us)
    uint16_t scan_evt_dur_dft;
};


/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// SCH SLICE environment variable
__STATIC struct sch_slice_env_tag sch_slice_env;

/// SCH SLICE parameters variable (accessible from drivers)
struct sch_slice_params_tag sch_slice_params;


/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */



/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Compute the scheduling parameters based on the current context
 ****************************************************************************************
 */
__STATIC void sch_slice_compute(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(SCH_SLICE_PATCH_TYPE, SCH_SLICE_COMPUTE_FUNC_BIT);

    uint32_t field;

#if BT_EMB_PRESENT
#if (MAX_NB_SYNC > 0)
    uint8_t t_esco_min = 0xFF;
    bool retx = false;
#endif // (MAX_NB_SYNC > 0)
#endif //BT_EMB_PRESENT

    uint16_t free_space_min = 0xFFFF;

    // Set default output
    sch_slice_params.scan_evt_dur = sch_slice_env.scan_evt_dur_dft;

#if BLE_EMB_PRESENT
    /************************************************************************
     * Implement the constraints of BLE over scan
     ************************************************************************/

    // Compute the minimum free space in between BLE events
    field = sch_slice_env.ble_bf;
    while (field)
    {
        int i =  32 - co_clz(field) - 1;

        ASSERT_ERR(i < BLE_ACTIVITY_MAX);

        if (sch_slice_env.ble[i].intv > 0)
        {
            // Compute the size in between events
            uint16_t free_space = sch_slice_env.ble[i].intv - (sch_slice_env.ble[i].duration / HALF_SLOT_SIZE);
            if (free_space > 2)
                free_space -= 2;
            free_space_min = co_min(free_space_min, free_space);
        }

        field &= ~(1 << i);
    }

    // Check if the scan window has to be limited by BLE connection
    sch_slice_params.scan_evt_dur = co_min(sch_slice_params.scan_evt_dur, free_space_min * HALF_SLOT_SIZE);
#endif //BLE_EMB_PRESENT

#if BT_EMB_PRESENT
    sch_slice_params.acl_evt_dur = 0xFFFF;

#if (MAX_NB_SYNC > 0)
    /************************************************************************
     * Implement the constraints of SCO over ACL/scan
     ************************************************************************/

    // Compute the minimum SCO interval
    field = sch_slice_env.sco_bf;
    while (field)
    {
        int i =  32 - co_clz(field) - 1;

        ASSERT_ERR(i < MAX_NB_SYNC);

        if (sch_slice_env.sco[i].intv > 0)
        {
            t_esco_min = co_min(t_esco_min, sch_slice_env.sco[i].intv);
            retx = sch_slice_env.sco[i].retx;
        }

        field &= ~(1 << i);
    }

    if (t_esco_min == 12) // Tesco = 6 slots (EV3/HV3)
    {
        sch_slice_params.scan_evt_dur = (6 * HALF_SLOT_SIZE);

        if (retx)
        {
            sch_slice_params.acl_evt_dur = (3 * HALF_SLOT_SIZE);
        }
        else
        {
            sch_slice_params.acl_evt_dur = (7 * HALF_SLOT_SIZE);
        }
    }
    else if (t_esco_min == 24) // Tesco = 12 slots (2-EV3)
    {
        sch_slice_params.scan_evt_dur = (16 * HALF_SLOT_SIZE);
    }
#endif // (MAX_NB_SYNC > 0)


    /************************************************************************
     * Implement the constraints of sniff over scan
     ************************************************************************/

    // Compute the minimum sniff interframe space
    field = sch_slice_env.sniff_bf;
    while (field)
    {
        int i =  32 - co_clz(field) - 1;

        ASSERT_ERR(i < MAX_NB_ACTIVE_ACL);

        if (sch_slice_env.sniff[i].intv > 0)
        {
            // Compute the size in between events
            uint16_t free_space = sch_slice_env.sniff[i].intv - (sch_slice_env.sniff[i].duration / HALF_SLOT_SIZE);
            if (free_space > 2)
                free_space -= 2;
            free_space_min = co_min(free_space_min, free_space);
        }

        field &= ~(1 << i);
    }

    // Check if the scan window has to be limited by sniff
    sch_slice_params.scan_evt_dur = co_min(sch_slice_params.scan_evt_dur, free_space_min * HALF_SLOT_SIZE);


#if (MAX_NB_SYNC > 0)
    /************************************************************************
     * Implement the constraints of scan over SCO
     ************************************************************************/

    if (sch_slice_env.access_modes || !retx)
    {
        sch_slice_params.sco_retx_allowed = false;
    }
    else
    {
        sch_slice_params.sco_retx_allowed = true;
    }
#endif // (MAX_NB_SYNC > 0)

    /************************************************************************
     * Implement the constraints of periodic activities over ACL
     ************************************************************************/
    if (sch_slice_params.acl_evt_dur == 0xFFFF)
    {
        sch_slice_params.acl_evt_dur = co_min(sch_slice_params.scan_evt_dur, SCH_ACL_EVT_DUR_DFT);
    }
#endif //BT_EMB_PRESENT
}


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void sch_slice_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_SLICE_PATCH_TYPE, SCH_SLICE_INIT_FUNC_BIT, is_reset);

    uint8_t length;
    memset(&sch_slice_env, 0, sizeof(sch_slice_env));

    // Set default values

#if BT_EMB_PRESENT
    sch_slice_params.acl_evt_dur = SCH_ACL_EVT_DUR_DFT;
#endif //BT_EMB_PRESENT

    // Retrieve default scan duration from parameters
    length = PARAM_LEN_SCHED_SCAN_DUR;
    if (rwip_param.get(PARAM_ID_SCHED_SCAN_DUR, &length, (uint8_t *) &sch_slice_env.scan_evt_dur_dft) != PARAM_OK)
    {
        sch_slice_env.scan_evt_dur_dft = SCH_SCAN_EVT_DUR_DFT;
    }

    sch_slice_params.scan_evt_dur = sch_slice_env.scan_evt_dur_dft;
}

void sch_slice_bg_add(uint8_t type)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_SLICE_PATCH_TYPE, SCH_SLICE_BG_ADD_FUNC_BIT, type);

    sch_slice_env.access_modes |= (1 << type);

    // Compute new parameters
    sch_slice_compute();
}

void sch_slice_bg_remove(uint8_t type)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(SCH_SLICE_PATCH_TYPE, SCH_SLICE_BG_REMOVE_FUNC_BIT, type);

    sch_slice_env.access_modes &= ~(1 << type);

    // Compute new parameters
    sch_slice_compute();
}

void sch_slice_per_add(uint8_t type, uint8_t id, uint32_t intv, uint16_t duration, bool retx)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(SCH_SLICE_PATCH_TYPE, SCH_SLICE_PER_ADD_FUNC_BIT, 5, type, id, intv, duration, retx);

    switch (type)
    {
#if BLE_EMB_PRESENT
    case BLE_CON:
    case BLE_PER_ADV:
    {
        sch_slice_env.ble[id].intv = intv;
        sch_slice_env.ble[id].duration = duration;
        sch_slice_env.ble_bf |= (1 << id);
    }
    break;
#endif //BLE_EMB_PRESENT

#if BT_EMB_PRESENT
    case BT_SNIFF:
    {
        sch_slice_env.sniff[id].intv = intv;
        sch_slice_env.sniff[id].duration = duration;
        sch_slice_env.sniff_bf |= (1 << id);
    }
    break;

#if (MAX_NB_SYNC > 0)
    case BT_SCO:
    {
        sch_slice_env.sco[id].intv = intv;
        sch_slice_env.sco[id].duration = duration;
        sch_slice_env.sco[id].retx = retx;
        sch_slice_env.sco_bf |= (1 << id);
    }
    break;
#endif // (MAX_NB_SYNC > 0)
#endif //BT_EMB_PRESENT

    default:
    {
        ASSERT_ERR_FORCE(0);
    }
    break;
    }

    // Compute new parameters
    sch_slice_compute();
}

void sch_slice_per_remove(uint8_t type, uint8_t id)
{
    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(SCH_SLICE_PATCH_TYPE, SCH_SLICE_PER_REMOVE_FUNC_BIT, type, id);

    switch (type)
    {
#if BLE_EMB_PRESENT
    case BLE_CON:
    case BLE_PER_ADV:
    {
        sch_slice_env.ble[id].intv = 0;
        sch_slice_env.ble_bf &= ~(1 << id);
    }
    break;
#endif //BLE_EMB_PRESENT

#if BT_EMB_PRESENT
    case BT_SNIFF:
    {
        sch_slice_env.sniff[id].intv = 0;
        sch_slice_env.sniff_bf &= ~(1 << id);
    }
    break;

#if (MAX_NB_SYNC > 0)
    case BT_SCO:
    {
        sch_slice_env.sco[id].intv = 0;
        sch_slice_env.sco_bf &= ~(1 << id);
    }
    break;
#endif // (MAX_NB_SYNC > 0)
#endif //BT_EMB_PRESENT

    default:
    {
        ASSERT_ERR_FORCE(0);
    }
    break;
    }

    // Compute new parameters
    sch_slice_compute();
}

///@} SCHSLICE
