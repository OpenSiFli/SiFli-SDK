/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_RF_CW_H
#define BT_RF_CW_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  RF set power interface,just used for RF test
 * @param[in]  type, 0:ble or br, 1:edr. only used for 52x
 * @param[in]  tx power,dbm unit
 * @param[out]     return param,0:sucess, other:error
 */
extern uint8_t btdm_rf_power_set(uint8_t type, int8_t txpwr);

/**
 * @brief  bt rf single carrier test function.
 * @param[in]  is_start: start test or stop test. 1:start test;0:stop test
 * @param[in]  pa is transmit power. dbm unit
 * @param[in]  set channel N, means 2402+ N MHz.
 */
extern void cw_config_bt(uint8_t is_start, uint8_t pa, uint8_t channel);



#ifdef __cplusplus
}
#endif

#endif /* BT_RF_CW_H */