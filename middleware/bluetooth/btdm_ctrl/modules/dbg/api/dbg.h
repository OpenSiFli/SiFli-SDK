/**
****************************************************************************************
*
* @file dbg.h
*
* @brief Debug function
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/
#ifndef DBG_H_
#define DBG_H_

/**
****************************************************************************************
* @addtogroup DBG
* @ingroup CONTROLLER
* @brief Debug
*
* @{
****************************************************************************************
*/


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // stack configuration

#ifdef EMB_PRESENT
    #include "dbg_swdiag.h"        // sw profiling definitions
#endif

#include "dbg_trc.h"           // debug tracer definition

/*
 * FUNCTION DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the BT Debug task
 *
 * This function initializes the the DBG task
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void dbg_init(bool is_reset);

/**
 ****************************************************************************************
 * @brief Send back to host status of platform reset request.
 *
 * @param status Reset error code
 *
 ****************************************************************************************
 */
void dbg_platform_reset_complete(uint32_t error);

///@} DBG

#endif // DBG_H_
