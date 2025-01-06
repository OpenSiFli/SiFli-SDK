/**
 ****************************************************************************************
 *
 * @file data_path.c
 *
 * @brief Definition of the functions used by the Link Layer Data Path ISO manager
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup DATA_PATH
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"    // IP configuration
#if (BLE_ISO_PRESENT)

#include "data_path.h"      // Data Path API
#include "plf_data_path.h"  // Data Path API - platform specific

#if (BLE_ISO_PCM)
    #include "isopcm.h"         // Isochronous data over PCM driver
#endif //(BLE_ISO_PCM)

#include <stddef.h>
#include "co_bt.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * CONSTANTS DEFINITION
 *****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */

/*
 * MODULE INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void plf_data_path_init(uint8_t init_type)
{
    // Add platform specific data-path initialization here


#if (BLE_ISO_PCM)
    isopcm_init(init_type);
#endif //(BLE_ISO_PCM)
}


const struct data_path_itf *plf_data_path_itf_get(uint8_t type, uint8_t direction)
{
    const struct data_path_itf *res = NULL;

    switch (type)
    {
        // Add platform specific data-path here

#if (BLE_ISO_PCM)
    case ISO_DP_PCM:
    {
        res = isopcm_itf_get(direction);
    }
    break;
#endif // BLE_ISO_PCM

    default:
    {
        /* Nothing to do */
    } break;
    }

    return res;
}

#endif // (BLE_ISO_PRESENT)

/// @} DATA_PATH
