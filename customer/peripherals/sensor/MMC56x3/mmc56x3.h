#pragma once

#include <stdint.h>
#include "rtthread.h"
#include "sensor.h"
#define MMC56X3_DEFAULT_ADDRESS 0x30 //!< Default address
#define MMC56X3_CHIP_ID 0x10         //!< Chip ID from WHO_AM_I register

/*=========================================================================*/

/*!
 * @brief MMC56X3 I2C register address bits
 */
typedef enum {
  MMC56X3_PRODUCT_ID = 0x39,
  MMC56X3_CTRL0_REG = 0x1B,
  MMC56X3_CTRL1_REG = 0x1C,
  MMC56X3_CTRL2_REG = 0x1D,
  MMC56X3_STATUS_REG = 0x18,
  MMC56X3_OUT_TEMP = 0x09,
  MMC56X3_OUT_X_L = 0x00,
  MMC5603_ODR_REG = 0x1A,

} mmc56x3_register_t;

typedef struct {
  float x;
  float y;
  float z;
} mmc56x3_data_t;

rt_err_t MMC56x3_Init(struct rt_sensor_config *cfg);
mmc56x3_data_t MMC56x3_ReadData(void);