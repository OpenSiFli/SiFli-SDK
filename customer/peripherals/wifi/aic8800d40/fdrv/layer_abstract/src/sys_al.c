/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "rwnx_platform.h"
#include "sys_al.h"
#include "aic_plat_types.h"

/* Task Priority */
uint32_t sdio_datrx_priority       = SDIO_DATRX_PRIORITY;
uint32_t sdio_oobirq_priority      = SDIO_OOBIRQ_PRIORITY;
uint32_t sdio_pwrclt_priority      = SDIO_PWRCLT_PRIORITY;
uint32_t fhost_cntrl_priority      = FHOST_CNTRL_PRIORITY;
uint32_t fhost_wpa_priority        = FHOST_WPA_PRIORITY;
uint32_t fhost_tx_priority         = FHOST_TX_PRIORITY;
uint32_t fhost_rx_priority         = FHOST_RX_PRIORITY;
uint32_t cli_cmd_priority          = CLI_CMD_PRIORITY;
uint32_t rwnx_timer_priority       = RWNX_TIMER_PRIORITY;
uint32_t rwnx_apm_staloss_priority = RWNX_APM_STALOSS_PRIORITY;
uint32_t tcpip_priority            = TCPIP_PRIORITY;
uint32_t wifi_api_priority         = WIFI_API_PRIORITY;
uint32_t task_end_prio             = TASK_END_PRIO;

/* Task Stack Size */
uint32_t sdio_datrx_stack_size       = SDIO_DATRX_STACK_SIZE;
uint32_t sdio_oobirq_stack_size      = SDIO_OOOBIRQ_STACK_SIZE;
uint32_t sdio_pwrctl_stack_size      = SDIO_PWRCTL_STACK_SIZE;
uint32_t fhost_cntrl_stack_size      = FHOST_CNTRL_STACK_SIZE;
uint32_t fhost_wpa_stack_size        = FHOST_WPA_STACK_SIZE;
uint32_t fhost_tx_stack_size         = FHOST_TX_STACK_SIZE;
uint32_t fhost_rx_stack_size         = FHOST_RX_STACK_SIZE;
uint32_t cli_cmd_stack_size          = CLI_CMD_STACK_SIZE*2;
uint32_t rwnx_timer_stack_size       = RWNX_TIMER_STACK_SIZE;
uint32_t rwnx_apm_staloss_stack_size = RWNX_APM_STALOSS_STACK_SIZE;
uint32_t wifi_api_stack_size         = WIFI_API_STACK_SIZE;

static uint8_t chip_online = 0;

//__INLINE
void wifi_chip_status_set(void)
{
    chip_online = 1;
}

//__INLINE
void wifi_chip_status_clr(void)
{
    chip_online = 0;
}

//__INLINE
uint8_t wifi_chip_status_get(void)
{
    return chip_online;
}

void aic_get_chip_info(uint8* info)
{
    rwnx_get_chip_info(info);
}

uint8_t *aic_get_fw_ver(void)
{
    return rwnx_get_fw_ver();
}

uint32_t aic_get_dpd_result(void)
{
    return aicwf_dpd_get_result();
}