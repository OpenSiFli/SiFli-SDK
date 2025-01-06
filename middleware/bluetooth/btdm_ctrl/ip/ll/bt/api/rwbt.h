/**
****************************************************************************************
*
* @file rwbt.h
*
* @brief RWBT interrupt definitions
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/
#ifndef _RW_BT_H_
#define _RW_BT_H_

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @brief Entry points of the BT stack
 *
 * This module contains the primitives that allow an application accessing and running the
 * BT protocol stack.
 *
 *
 * @{
 ****************************************************************************************
 */

#include <stdint.h>                 // standard integer definitions
#include <stdbool.h>                // standard boolean definitions
#include "rwip_config.h"            // stack configuration
#include "arch.h"


/*
 * FUNCTION DECLARATION
*****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes the RW BT SW.
 *
 ****************************************************************************************
 */
void rwbt_init(void);

/**
 ****************************************************************************************
 * @brief Reset the RW BT SW.
 *
 ****************************************************************************************
 */
void rwbt_reset(void);

/**
 ****************************************************************************************
 * @brief The RWBT core interrupt handler.
 *
 * Handles both the clock interrupt and the Rx Packet interrupt (when activated)
 * In the case of a clock interrupt, check if there are active links and which one
 * has the highest priority.
 *
 ****************************************************************************************
 */
__BTIRQ void bt_isr(void);

///@} ROOT

#endif // _RW_BT_H_
