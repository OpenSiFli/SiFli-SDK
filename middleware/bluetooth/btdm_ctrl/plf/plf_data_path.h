/**
 ****************************************************************************************
 *
 * @file dplf_ata_path.h
 *
 * @brief Main API file for the Link Layer platform specific Data path manager
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */

#ifndef PLF_DATA_PATH_H_
#define PLF_DATA_PATH_H_

/**
 ****************************************************************************************
 * @defgroup PLF_DATA_PATH Link Layer platform specific ISO data path
 * @ingroup ROOT
 * @brief Link Layer platform specific ISO data path
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#if (BLE_ISO_PRESENT)

#include <stdbool.h>        // boolean definition
#include <stdint.h>         // integer definition


/*
 * DEFINES
 ****************************************************************************************
 */

/// Isochronous Channel data path selection
enum plf_dp_type
{
    // -------- VENDOR SPECIFIC --------- //
    // Add vendor specific data-path number here

    /// PCM Data path
    ISO_DP_PCM                      = 0xF1,
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */



/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the BLE Data Path driver
 *
 * @param[in] init_type  Type of initialization (@see enum rwip_init_type)
 ****************************************************************************************
 */
void plf_data_path_init(uint8_t init_type);

/**
 ****************************************************************************************
 * @brief Retrieve the data path interface according to the direction
 *
 * @param[in]  type      Type of data path interface (@see enum iso_dp_type)
 * @param[in]  direction Data Path direction (@see enum iso_rx_tx_select)
 *
 * @return Pointer to the interface of the data path driver, NULL if no driver found
 ****************************************************************************************
 */
const struct data_path_itf *plf_data_path_itf_get(uint8_t type, uint8_t direction);

#endif // (BLE_ISO_PRESENT)
/// @} PLF_DATA_PATH

#endif // PLF_DATA_PATH_H_
