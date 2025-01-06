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

#include "co_bt.h"          // Specification defines

#if (BLE_ISOOHCI)
    #include "isoohci.h"        // Isochronous data over HCI data path driver
#endif //(BLE_ISOOHCI)

#if (BLE_ISOGEN)
    #include "isogen.h"         // Isochronous Payload generator
#endif //(BLE_ISOGEN)

#if (BLE_BLUEBUD_TWS)
    #include "bluebud_tws.h"    // Bluebud TWS data path
#endif //(BLE_BLUEBUD_TWS)

#include <stddef.h>
#include "btdm_patch.h"

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
__STATIC const struct data_path_itf data_path_itf_default = {NULL, NULL, NULL, NULL, NULL, NULL};

/*
 * MODULE INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void data_path_init(bool is_reset)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(DATA_PATH_PATCH_TYPE, DATA_PATH_INIT_FUN_BIT, is_reset);
#if (BLE_ISOOHCI)
    isoohci_init(is_reset);
#endif //(BLE_ISOOHCI)

#if (BLE_ISOGEN)
    isogen_init(is_reset);
#endif //(BLE_ISOGEN)

#if (BLE_BLUEBUD_TWS)
    bluebud_tws_init(is_reset);
#endif //(BLE_BLUEBUD_TWS)

    // platform data path initialization
    plf_data_path_init(is_reset);
}

uint8_t data_path_config(uint8_t type, uint8_t len, const uint8_t *cfg)
{
    FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(DATA_PATH_PATCH_TYPE, DATA_PATH_CONFIG_FUN_BIT, uint8_t, type, len, cfg);
    uint8_t status = CO_ERROR_NO_ERROR;

    switch (type)
    {
#if (BLE_ISOGEN)
    case ISO_DP_ISOGEN:
    {
        // TODO - Configure ISOGEN specifics
    } break;
#endif //(BLE_ISOGEN)

#if (BLE_BLUEBUD_TWS)
    case ISO_DP_BLUEBUD_TWS:
    {
        // TODO - Configure BlueBud TWS specifics
    } break;
#endif //(BLE_BLUEBUD_TWS)

    default:
    {
        // Unrecognized data path, or ID reserved for future use
        status = CO_ERROR_INVALID_HCI_PARAM;
    }
    break;
    }

    return status;
}

const struct data_path_itf *_data_path_itf_get(uint8_t type, uint8_t direction)
{
    //FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(DATA_PATH_PATCH_TYPE, DATA_PATH_ITF_GET_FUN_BIT, uint32_t##" *", type, direction);
    const struct data_path_itf *res = NULL;

    switch (type)
    {
    case ISO_DP_DISABLE:
        break;

#if (BLE_ISOOHCI)
    case ISO_DP_ISOOHCI:
    {
        res = isoohci_itf_get(direction);
    }
    break;
#endif //(BLE_ISOOHCI)

#if (BLE_ISOGEN)
    case ISO_DP_ISOGEN:
    {
        res = isogen_itf_get(direction);
    }
    break;
#endif //(BLE_ISOGEN)

#if (BLE_BLUEBUD_TWS)
    case ISO_DP_BLUEBUD_TWS:
    {
        res = bluebud_tws_itf_get(direction);
    }
    break;
#endif //(BLE_BLUEBUD_TWS)

    default:
    {
        /* Check if there is a platform data_path that can be loaded */
        res = plf_data_path_itf_get(type, direction);
    }
    break;
    }

    // If no data path, assign the default one
    if (res == NULL)
    {
        res = &data_path_itf_default;
    }

    return res;
}


data_path_itf_get_handler _data_path_itf_get_handler = _data_path_itf_get;

bool data_path_is_disabled(const struct data_path_itf *dp)
{
    FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(DATA_PATH_PATCH_TYPE, DATA_PATH_IS_DISABLED_FUN_BIT, bool, dp);
    return (dp == &data_path_itf_default);
}

#endif // (BLE_ISO_PRESENT)

/// @} DATA_PATH
