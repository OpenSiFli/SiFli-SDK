/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LCPU_CONFIG_TYPE_H
#define __LCPU_CONFIG_TYPE_H

#include "lcpu_config_type_int.h"

/**
 * @brief  ENUM definition of configration type.
 */


typedef enum
{
    HAL_LCPU_CONFIG_XTAL_ENABLED = 0,      /*!< ADC calibration value. */
    HAL_LCPU_CONFIG_LPCYCLE_CURR = 1,             /*!< Chip revision. */
    HAL_LCPU_CONFIG_LPCYCLE_AVE = 2,  /*!< Battery calibration value. */
    HAL_LCPU_CONFIG_WDT_TIME = 3,          /*!< SDADC calibration value. */
    HAL_LCPU_CONFIG_WDT_STATUS = 4,  /*!< Battery calibration value. */
    HAL_LCPU_CONFIG_WDT_CLK_FEQ = 5,
    HAL_LCPU_CONFIG_BT_TX_PWR = 6,  /*!< Battery calibration value. */
    HAL_LCPU_CONFIG_BT_RC_CAL_IN_L = 7,   /*!< Enable RCCal for BT in LCPU. */
    HAL_LCPU_CONFIG_SOFT_CVSD = 8,        /*!< Enable soft cvsd encode decode for BT in LCPU. */
    // Following type only support after revID >= 4
    HAL_LCPU_CONFIG_BT_EM_BUF = 9,  /*!< BT custormized EM buffer. */
    HAL_LCPU_CONFIG_BT_ACT_CFG = 10,  /*!< BT custormized link related config. */
    HAL_LCPU_CONFIG_BT_CONFIG = 11,  /*!< BT custormized other config. */
    HAL_LCPU_CONFIG_BT_KE_BUF = 12,  /*!< BT custormized KE buffer. */
    HAL_LCPU_CONFIG_SEC_ADDR = 13, /*!< Security protection address. */
    HAL_LCPU_CONFIG_HCPU_TX_QUEUE = 14, /*!< TX buffer of HCPU IPC queue. */
    HAL_LCPU_CONFIG_ADC_CALIBRATION = 15,      /*!< ADC calibration value. */
    HAL_LCPU_CONFIG_SDADC_CALIBRATION = 16,    /*!< SDADC calibration value. */
    HAL_LCPU_CONFIG_SN = 17,                   /*!< mcu serial number. */
    HAL_LCPU_CONFIG_CHIP_REV = 18,             /*!< Chip revision. */
    HAL_LCPU_CONFIG_BATTERY_CALIBRATION = 19,  /*!< Battery calibration value. */
    HAL_LCPU_CONFIG_BT_ACTMOVE_CONFIG = 20,  /*!< BT custormized activity move config. */
    HAL_LCPU_CONFIG_BLE_FLUSH_CONFIG = 21, /*!< BLE.data flushed mode config */
    HAL_LCPU_CONFIG_AFH_CONFIG = 22,/*!< about freq hopping algo config */
    HAL_LCPU_CONFIG_MAX = 0xFE,
} HAL_LCPU_CONFIG_TYPE_T;


uint16_t LCPU_CONFIG_get_total_size(void);
uint8_t LCPU_CONFIG_set(uint8_t *base_addr, uint8_t config_type, uint8_t *value, uint16_t length);
uint8_t LCPU_CONFIG_get(uint8_t *base_addr, uint8_t config_type, uint8_t *value, uint16_t *length);


#endif // __LCPU_CONFIG_TYPE_H
