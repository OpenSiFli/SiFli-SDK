/**
 ****************************************************************************************
 *
 * @file co_djob.h
 *
 * @brief Common delayed job definitions
 *
 * This module is used to defer function execution in background.
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */
#ifndef _CO_DJOB_H_
#define _CO_DJOB_H_

/**
 ****************************************************************************************
 * @defgroup CO_DJOB_API Delayed Job
 * @ingroup COMMON_API
 * @brief  Delayed job utilities
 *
 * This module contains the delayed job utilities functions and macros.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>       // standard definitions
#include <stddef.h>       // standard definitions
#include "co_list.h"      // common bt definitions


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
typedef struct co_djob co_djob_t;

/**
 ****************************************************************************************
 * @brief Job function to called into a background context
 *
 * @param[in] p_djob  Pointer to Delayed job structure.
 ****************************************************************************************
 */
typedef void (*co_djob_cb)(co_djob_t *p_djob);

/// Job element structure
struct co_djob
{
    /// List element header
    co_list_hdr_t hdr;
    /// Callback to execute in background context
    co_djob_cb    cb;
};


/*
 * CONSTANT DECLARATIONS
 ****************************************************************************************
 */
/// Supported deferred job execution priority level
///
/// All function with High priority execution will be executed before lower layer priorities.
enum co_djob_prio_level
{
    /// Low priority defer job execution
    CO_DJOB_LOW,
    /// High priority defer job execution
    CO_DJOB_HIGH,
    // Number of priority level
    CO_DJOB_PRIO_NB,
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * Delayed Job functions
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Init Delayed job structure
 *
 * @param[in] p_djob   Pointer to the delayed job structure
 * @param[in] cb       Callback to execute in background context
 ****************************************************************************************
 */
void co_djob_init(co_djob_t *p_djob, co_djob_cb cb);

/**
 ****************************************************************************************
 * @brief Defer execution of an interrupt. Execution of the defer function cannot be canceled
 *
 * @param[in] p_djob   Pointer to the delayed job structure
 ****************************************************************************************
 */
void co_djob_isr_reg(co_djob_t *p_djob);

/**
 ****************************************************************************************
 * @brief Register to execute a job delayed in background - Shall not be called in an interrupt context
 *
 * @param[in] prio_lvl Priority level (@see enum co_djob_prio_level)
 * @param[in] p_djob   Pointer to the delayed job structure
 ****************************************************************************************
 */
void co_djob_reg(uint8_t prio_lvl, co_djob_t *p_djob);

/**
 ****************************************************************************************
 * @brief Un-register a job that waits to be executed- Shall not be called in an interrupt context
 *
 * Priority level must be equals to the priority level provided in @see co_djob_reg.
 *
 * @param[in] prio_lvl Priority level (@see enum co_djob_prio_level)
 * @param[in] p_djob   Pointer to the delayed job structure
 ****************************************************************************************
 */
void co_djob_unreg(uint8_t prio_lvl, co_djob_t *p_djob);

/// @}

/**
 ****************************************************************************************
 * @brief Initialize Common delayed job module.
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void co_djob_initialize(bool is_reset);

#endif // _CO_DJOB_H_
