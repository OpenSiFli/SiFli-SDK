#ifndef _LTR303_H_
#define _LTR303_H_

#include <stdint.h>
#include "rtthread.h"
#include "sensor.h"

#define LTR303_ADDRESS 0x29 // default address

// LTR303 register addresses
#define LTR303_I2CADDR_DEFAULT 0x29 ///< I2C address
#define LTR303_PART_ID 0x86         ///< Part id/revision register
#define LTR303_MANU_ID 0x87         ///< Manufacturer ID register
#define LTR303_ALS_CTRL 0x80        ///< ALS control register
#define LTR303_STATUS 0x8C          ///< Status register
#define LTR303_CH1DATA 0x88         ///< Data for channel 1 (read all 4 bytes!)
#define LTR303_MEAS_RATE 0x85       ///< Integration time and data rate

// These registers on LTR-303 only!
#define LTR303_REG_INTERRUPT 0x8F ///< Register to enable/configure int output
#define LTR303_REG_THRESHHIGH_LSB 0x97 ///< ALS 'high' threshold limit
#define LTR303_REG_THRESHLOW_LSB 0x99  ///< ALS 'low' threshold limit
#define LTR303_REG_INTPERSIST 0x9E ///< Register for setting the IRQ persistance

/*!    @brief  Sensor gain for ALS  */
typedef enum {
  LTR3XX_GAIN_1 = 0,
  LTR3XX_GAIN_2 = 1,
  LTR3XX_GAIN_4 = 2,
  LTR3XX_GAIN_8 = 3,
  // 4 & 5 unused!
  LTR3XX_GAIN_48 = 6,
  LTR3XX_GAIN_96 = 7,
} ltr303_gain_t;

/*!    @brief Integration times, in milliseconds */
typedef enum {
  LTR3XX_INTEGTIME_100,
  LTR3XX_INTEGTIME_50,
  LTR3XX_INTEGTIME_200,
  LTR3XX_INTEGTIME_400,
  LTR3XX_INTEGTIME_150,
  LTR3XX_INTEGTIME_250,
  LTR3XX_INTEGTIME_300,
  LTR3XX_INTEGTIME_350,
} ltr303_integrationtime_t;

/*!    @brief Measurement rates, in milliseconds */
typedef enum {
  LTR3XX_MEASRATE_50,
  LTR3XX_MEASRATE_100,
  LTR3XX_MEASRATE_200,
  LTR3XX_MEASRATE_500,
  LTR3XX_MEASRATE_1000,
  LTR3XX_MEASRATE_2000,
} ltr303_measurerate_t;

/***********************************************************************************/
rt_err_t LTR303_Init(struct rt_sensor_config *cfg);
uint16_t LTR303_ReadVisible(void);
void LTR303_PowerOn(void);
void LTR303_PowerOff(void);

#endif // _LTR303_H_