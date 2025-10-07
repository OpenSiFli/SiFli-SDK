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
#endif /* SOC_BF0_HCPU */

#ifdef SOC_BF0_LCPU
ipc_queue_handle_t sys_init_lh_ipc_queue(ipc_queue_rx_ind_t rx_ind);
ipc_queue_handle_t sys_get_lh_ipc_queue(void);
#endif /* SOC_BF0_LCPU */

#else

#endif  // USING_IPC_QUEUE

void print_sysinfo(char *buf, uint32_t buf_len);

/// @}  mbox_common
/// @}  file

#ifdef __cplusplus
}
#endif
#endif

