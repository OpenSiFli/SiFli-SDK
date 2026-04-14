/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_HAL_LCPU_CONFIG_H
#define __BF0_HAL_LCPU_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"
#include "lcpu_config_type.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */


/** @defgroup LPCONFIG LCPU configure
  * @brief LCOU configure to share parameter with HCPU
  * @{
  */


/**
  * @addtogroup  LPCONFIG_exported_constants
  * @{
*/

#define HAL_LCPU_CONFIG_SN_MAX_NUM 128


/**
  * @brief  Structure definition of @ref HAL_LCPU_CONFIG_ADC_CALIBRATION for LCPU_CONFIG_V1.
*/
typedef struct
{
    uint16_t vol10;     /*!< Reg value for low voltage. */
    uint16_t vol25;      /*!< Reg value for high voltage. */
#ifndef SF32LB55X
    uint16_t low_mv;     /*!< voltage for low with mv . */
    uint16_t high_mv;    /*!< voltage for high with mv. */
#endif
} HAL_LCPU_CONFIG_ADC_T;


/**
  * @brief  Structure definition of @ref HAL_LCPU_CONFIG_ADC_CALIBRATION for LCPU_CONFIG_V2.
*/
typedef struct
{
    uint16_t vol10;     /*!< Reg value for low voltage. */
    uint16_t vol25;      /*!< Reg value for high voltage. */
    uint16_t low_mv;     /*!< voltage for low with mv . */
    uint16_t high_mv;    /*!< voltage for high with mv. */
} HAL_LCPU_CONFIG_ADC_V2_T;


/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_SDADC_CALIBRATION for LCPU_CONFIG_V1.
*/
typedef struct
{
    uint32_t vol_mv;
    uint32_t value;
} HAL_LCPU_CONFIG_SDMADC_T;

/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_SDADC_CALIBRATION for LCPU_CONFIG_V2.
*/
typedef struct
{
    uint32_t vol_mv;
    uint32_t value;
} HAL_LCPU_CONFIG_SDMADC_V2_T;


/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_SN for LCPU_CONFIG_V1.
*/
typedef struct
{
    uint8_t sn[HAL_LCPU_CONFIG_SN_MAX_NUM];
} HAL_LCPU_CONFIG_SN_T;

/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_SN for LCPU_CONFIG_V2.
*/
typedef struct
{
    uint16_t sn_len;
    uint8_t sn[HAL_LCPU_CONFIG_SN_MAX_NUM];
} HAL_LCPU_CONFIG_SN_V2_T;



/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_BATTERY_CALIBRATION for LCPU_CONFIG_V1.  ax+b   a*10000 for integer
*/
typedef struct
{
    int magic;
    uint32_t a;
    int32_t b;
} HAL_LCPU_CONFIG_BATTERY_T;

/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_BATTERY_CALIBRATION for LCPU_CONFIG_V2.  ax+b   a*10000 for integer
*/
typedef struct
{
    int magic;
    uint32_t a;
    int32_t b;
} HAL_LCPU_CONFIG_BATTERY_V2_T;



#define HAL_LCPU_CONFIG_EM_BUF_MAX_NUM 40

/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_BT_EM_BUF.
*/
typedef struct
{
    uint8_t is_valid;
    uint16_t em_buf[HAL_LCPU_CONFIG_EM_BUF_MAX_NUM];
} hal_lcpu_bluetooth_em_config_t;


/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_BT_ACT_CFG.
*/
typedef struct
{
    // max valid bit is 7
    uint32_t bit_valid;
    uint8_t bt_max_acl;
    uint8_t bt_max_sco;
    uint8_t ble_max_act;
    uint8_t ble_max_ral;
    uint8_t ble_max_iso;
    uint8_t ble_rx_desc;
    uint8_t bt_rx_desc;
    uint8_t bt_name_len;
} hal_lcpu_bluetooth_act_configt_t;

/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_BT_CONFIG.
*/
typedef struct
{
    // max valid bit is 11
    uint32_t bit_valid;
    uint32_t max_sleep_time;
    // bit 0: BLE, 1: BT
    uint8_t controller_enable_bit;
    uint8_t lld_prog_delay;
    uint8_t lld_prog_delay_min;
    uint8_t default_sleep_mode;
    uint8_t default_sleep_enabled;
    uint8_t default_xtal_enabled;
    uint8_t default_rc_cycle;
    uint8_t default_swprofiling_cfg;
    // 0: boot to rom; 1: boot to ram; 2. debug boot
    uint8_t boot_mode;
    // 1: fpga, others asic
    uint8_t is_fpga;
    uint8_t en_inq_filter;
    uint8_t support_3m;
    uint8_t sco_cfg;
} hal_lcpu_bluetooth_rom_config_t;

/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_BT_KE_BUF.
*/
typedef struct
{
    //  max valid bit is 5, buf and size should be aligned.
    uint32_t bit_valid;
    uint8_t *env_buf;
    uint8_t *msg_buf;
    uint8_t *nt_buf;
    uint8_t *log_buf;
    uint8_t *nvds_buf;
    uint8_t *aud_buf;
    uint16_t env_buf_size;
    uint16_t msg_buf_size;
    uint16_t nt_buf_size;
    uint16_t log_buf_size;
    uint16_t nvds_buf_size;
    uint16_t aud_buf_size;
    int8_t max_nb_of_hci_completed;
} hal_lcpu_ble_mem_config_t;

/**
  * @brief  Structure definition of #HAL_LCPU_CONFIG_BT_ACTMOVE_CONFIG.
*/
typedef struct
{
    //  max valid bit is 5, buf and size should be aligned.
    uint8_t bit_valid;
    uint8_t act_mov;
    uint8_t sco_pingpong;

} hal_lcpu_bluetooth_actmove_config_t;

/**
  * @} LPCONFIG_exported_constants
*/


/**
  * @defgroup LPCONFIG_exported_functions LCPU configure Exported functions
  * @ingroup LPCONFIG
  * @{
  *
 */



/**
* @brief  Set configure parameter, only support in HCPU
* @param type Parameter type
* @param value Parameter values
* @param length Parameter length
* @retval HAL_OK if successful, otherwise error
*/
HAL_StatusTypeDef HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_TYPE_T type, void *value, uint16_t length);

/**
* @brief  Get configure parameter
* @param type Parameter type
* @param value Parameter values
* @param length Parameter length
* @retval HAL_OK if successful, otherwise error
*/
HAL_StatusTypeDef HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_TYPE_T type, uint8_t *value, uint16_t *length);

/**
* @brief Force init context without check
* @return void
*/
void HAL_LCPU_CONFIG_InitContext(void);

/**
 * @brief Query whether LCPU crashes
 * @return HAL_OK if LCPU crashes, HAL_BUSY: no crash happens
 */
HAL_StatusTypeDef HAL_LCPU_ASSERT_INFO_get(void);

/**
 * @brief LCPU update flag to indicate crash happens, HCPU can query this flag by HAL_LCPU_ASSERT_INFO_get
 */
void HAL_LCPU_ASSERT_INFO_set(void);

/**
 * @brief clear LCPU crash flag
 */
void HAL_LCPU_ASSERT_INFO_clear(void);


/**
  *@} LPCONFIG_exported_functions
*/


/**
  *@} LPCONFIG
  */


/**
  *@} BF0_HAL_Driver
  */



#ifdef __cplusplus
}
#endif


#endif // __BF0_HAL_LCPU_CONFIG_H

/**
  *@}
  */