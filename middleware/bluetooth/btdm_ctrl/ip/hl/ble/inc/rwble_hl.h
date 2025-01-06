/**
 ****************************************************************************************
 *
 * @file rwble_hl.h
 *
 * @brief Entry points of the BLE HL software
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

#ifndef RWBLE_HL_H_
#define RWBLE_HL_H_

#include <stdint.h>

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @brief Entry points of the BLE Host stack
 *
 * This module contains the primitives that allow an application accessing and running the
 * BLE protocol stack
 *
 * @{
 ****************************************************************************************
 */


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

typedef void (*rwble_hl_init_handler)(void);
typedef void (*rwble_hl_reset_handler)(void);

extern rwble_hl_init_handler _rwble_hl_init_handler;
extern rwble_hl_reset_handler _rwble_hl_reset_handler;

#define rwble_hl_init() _rwble_hl_init_handler()
#define rwble_hl_reset() _rwble_hl_reset_handler()






/**
 ****************************************************************************************
 * @brief Initialize the BLE Host stack.
 ****************************************************************************************
 */
void _rwble_hl_init(void);

/**
 ****************************************************************************************
 * @brief Initialize the host (reset requested)
 *
 ****************************************************************************************
 */
void _rwble_hl_reset(void);

/// @} RWBLE_HL

#endif // RWBLE_HL_H_
