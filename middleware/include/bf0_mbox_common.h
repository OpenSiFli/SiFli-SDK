/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_MBOX_COMMON_H__
#define __BF0_MBOX_COMMON_H__


#include <rtconfig.h>


#ifdef __cplusplus
extern "C" {
#endif

#if defined(USING_IPC_QUEUE) || defined(_SIFLI_DOXYGEN_)
#include "ipc_queue.h"
#include "bf0_hal_rcc.h"

/**
****************************************************************************************
* @addtogroup mbox_common Mailbox interface
* @ingroup middleware
* @brief Sifli Mailbox interface among cores.
* @{
****************************************************************************************
*/

// Define other cores
#ifdef SOC_BF0_HCPU
#define PEER_CORE_1     CORE_ID_LCPU
#endif

#ifdef SOC_BF0_LCPU
#define PEER_CORE_1     CORE_ID_HCPU
#endif

#if defined(SOC_BF0_HCPU)
/** @brief power on LCPU,
*
*
* @return 0: OK, >0: ERROR
*/
uint8_t lcpu_power_on(void);
/** @brief power off LCPU,
*
*
* @return 0: OK, >0: ERROR
*/
uint8_t lcpu_power_off(void);

/** @brief Disable/Enable RF Calibration when power up LCPU
*
* @psram is_disable  1: disable RF calibration, 0: enable RF Calibration
*
* @return void
*/
void lcpu_disable_rf_cal(uint8_t is_disable);
#endif /* SOC_BF0_HCPU */

#ifdef SOC_BF0_HCPU
ipc_queue_handle_t sys_init_hl_ipc_queue(ipc_queue_rx_ind_t rx_ind);
ipc_queue_handle_t sys_get_hl_ipc_queue(void);

ipc_queue_handle_t sys_init_ha_ipc_queue(ipc_queue_rx_ind_t rx_ind);
ipc_queue_handle_t sys_get_ha_ipc_queue(void);
#endif /* SOC_BF0_HCPU */

#ifdef SOC_BF0_ACPU
ipc_queue_handle_t sys_init_ah_ipc_queue(ipc_queue_rx_ind_t rx_ind);
ipc_queue_handle_t sys_get_ah_ipc_queue(void);
#endif  // SOC_BF0_ACPU

#ifdef SOC_BF0_LCPU
ipc_queue_handle_t sys_init_lh_ipc_queue(ipc_queue_rx_ind_t rx_ind);
ipc_queue_handle_t sys_get_lh_ipc_queue(void);
#endif /* SOC_BF0_LCPU */

#else

#endif  // USING_IPC_QUEUE

void print_sysinfo(char *buf, uint32_t buf_len);

/**
 * @brief Calculate the compensated rt_tick
 *
 *  @param[in] curr_tick current rt_tick
 *  @param[in] curr_time current GTIME
 *  @param[in] gtime_freq GTIMER frequency
 *  @param[in] user_data user data
 *
 * @retval compensated rt_tick
 */
rt_tick_t sf_latch_tick(rt_tick_t curr_tick, uint32_t curr_time, float gtime_freq, void *user_data);

/**
 * @brief Update last latch tick if rt_tick cannot be updated as expected
 *
 *  @param[in] last_latch_tick last_latch_tick
 *
 * @return void
 */
void sf_set_last_latch_tick(rt_tick_t last_latch_tick);

/// @}  mbox_common
/// @}  file

#ifdef __cplusplus
}
#endif
#endif

