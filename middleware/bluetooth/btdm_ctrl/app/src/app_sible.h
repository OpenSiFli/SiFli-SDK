/**
 ****************************************************************************************
 *
 * @file app_sible.h
 *
 * @brief Sifli BLE Application Module entry point
 *
 * Copyright (C) Sifli 2019-2022
 *
 *
 ****************************************************************************************
 */

#ifndef APP_SIBLE_H_
#define APP_SIBLE_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup SIFLI
 *
 * @brief SiFli BLE Application Module entry point
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_SIBLE)

#include <stdint.h>          // Standard Integer Definition
#include "ke_task.h"         // Kernel Task Definition

/*
 * STRUCTURES DEFINITION
 ****************************************************************************************
 */

/// Sifil BLE Application Module Environment Structure
struct app_sible_env_tag
{
    /// Connection handle
    uint8_t conidx;
};

/*
 * GLOBAL VARIABLES DECLARATIONS
 ****************************************************************************************
 */

/// SiFli BLE Application environment
extern struct app_sible_env_tag app_sible_env;

/// Table of message handlers
extern const struct app_subtask_handlers app_sible_handlers;

/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 *
 * Health Thermometer Application Functions
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize SiFli BLE Application Module
 ****************************************************************************************
 */
void app_sible_init(void);

/**
 ****************************************************************************************
 * @brief Add a SiFli BLE Service instance in the DB
 ****************************************************************************************
 */
void app_sible_add_sibles(void);

/**
 ****************************************************************************************
 * @brief Enable the SiFli BLE Service
 ****************************************************************************************
 */
void app_sible_enable_prf(uint8_t conidx);

uint8_t sibles_get_adv_data(uint8_t *data);
uint8_t sibles_get_scan_rsp(uint8_t *data);


#endif //(BLE_APP_SIBLE)

/// @} APP

#endif // APP_SIBLE_H_
