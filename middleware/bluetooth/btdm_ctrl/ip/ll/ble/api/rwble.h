/**
 ****************************************************************************************
 *
 * @file rwble.h
 *
 * @brief Entry points of the BLE software
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

#ifndef RWBLE_H_
#define RWBLE_H_

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @brief Entry points of the BLE stack
 *
 * This module contains the primitives that allow an application accessing and running the
 * BLE protocol stack
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>                 // standard integer definitions
#include <stdbool.h>                // standard boolean definitions
#include "rwip_config.h"            // stack configuration
#include "compiler.h"

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the BLE stack.
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void rwble_init(bool is_reset);


/**
 ****************************************************************************************
 * @brief Return true if no BLE activity on going
 *
 ****************************************************************************************
 */
bool rwble_activity_ongoing_check(void);

/**
 ****************************************************************************************
 * @brief RWBLE interrupt service routine
 *
 * This function is the interrupt service handler of RWBLE.
 *
 ****************************************************************************************
 */
__BLEIRQ void ble_isr(void);

/// @} RWBLE

#endif // RWBLE_H_
