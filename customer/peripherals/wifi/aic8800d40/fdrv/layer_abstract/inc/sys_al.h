/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _SYS_DESIRE_H_
#define _SYS_DESIRE_H_

#include "compiler.h"
#include "aic_plat_types.h"

#define TCPIP_PRIORITY              (16)
#define TASK_END_PRIO               (20)
#define DYNAMIC_PRIORITY            (0)

/* Task Priority */
#define SDIO_DATRX_PRIORITY         (TCPIP_PRIORITY-1)
#define SDIO_OOBIRQ_PRIORITY        (5)
#define SDIO_PWRCLT_PRIORITY        (TCPIP_PRIORITY+3)
#define FHOST_CNTRL_PRIORITY        (TCPIP_PRIORITY+1)
#define FHOST_WPA_PRIORITY          (TCPIP_PRIORITY+2)
#ifndef PROTECT_TX_ADMA
    #define FHOST_TX_PRIORITY           (TCPIP_PRIORITY-0)
#else
    #define FHOST_TX_PRIORITY           (TCPIP_PRIORITY-2)
#endif /* CONFIG_ADMA_PROTECT */
#define FHOST_RX_PRIORITY           (TCPIP_PRIORITY+1)
#define CLI_CMD_PRIORITY            (TCPIP_PRIORITY+3)
#define RWNX_TIMER_PRIORITY         (TCPIP_PRIORITY+1)
#define RWNX_APM_STALOSS_PRIORITY   (TCPIP_PRIORITY-1)
#define WIFI_API_PRIORITY           (TCPIP_PRIORITY+3)

extern uint32_t sdio_datrx_priority;
extern uint32_t sdio_oobirq_priority;
extern uint32_t sdio_pwrclt_priority;
extern uint32_t fhost_cntrl_priority;
extern uint32_t fhost_wpa_priority;
extern uint32_t fhost_tx_priority;
extern uint32_t fhost_rx_priority;
extern uint32_t cli_cmd_priority;
extern uint32_t rwnx_timer_priority;
extern uint32_t rwnx_apm_staloss_priority;
extern uint32_t tcpip_priority;
extern uint32_t wifi_api_priority;
extern uint32_t task_end_prio;

/* Task Stack Size */
#define SDIO_DATRX_STACK_SIZE       (2048)
#define SDIO_OOOBIRQ_STACK_SIZE     (4096)
#define SDIO_PWRCTL_STACK_SIZE      (2048)
#define FHOST_CNTRL_STACK_SIZE      (4096)
#define FHOST_WPA_STACK_SIZE        (20480)
#define FHOST_TX_STACK_SIZE         (8096)
#define FHOST_RX_STACK_SIZE         (4096)
#define CLI_CMD_STACK_SIZE          (3072)
#define RWNX_TIMER_STACK_SIZE       (2048)
#define RWNX_APM_STALOSS_STACK_SIZE (8096)
#define WIFI_API_STACK_SIZE         (8096)

extern uint32_t sdio_datrx_stack_size;
extern uint32_t sdio_oobirq_stack_size;
extern uint32_t sdio_pwrctl_stack_size;
extern uint32_t fhost_cntrl_stack_size;
extern uint32_t fhost_wpa_stack_size;
extern uint32_t fhost_tx_stack_size;
extern uint32_t fhost_rx_stack_size;
extern uint32_t cli_cmd_stack_size;
extern uint32_t rwnx_timer_stack_size;
extern uint32_t rwnx_apm_staloss_stack_size;
extern uint32_t wifi_api_stack_size;

#define CHIP_REV_U01                0x1
#define CHIP_REV_U02                0x3
#define CHIP_REV_U03                0x7
#define CHIP_SUB_REV_U04            0x20

typedef enum
{
    PRODUCT_ID_AIC8801 = 0,
    PRODUCT_ID_AIC8800DC,
    PRODUCT_ID_AIC8800DW,
    PRODUCT_ID_AIC8800D80,
    PRODUCT_ID_AIC8800D81,
} AICWF_IC;

#define HOST_BUS_SLEEP              0
#define HOST_BUS_ACTIVE             1

typedef enum
{
    HOST_TYPE_DATA_FW2SOC  = 0X00,
    HOST_TYPE_DATA_SOC2FW  = 0X01,
    HOST_TYPE_CFG          = 0X10,
    HOST_TYPE_CFG_CMD_RSP  = 0X11,
    HOST_TYPE_CFG_DATA_CFM = 0X12
} host_type;

//__INLINE
void wifi_chip_status_set(void);
//__INLINE
void wifi_chip_status_clr(void);
//__INLINE
uint8_t wifi_chip_status_get(void);
void aic_get_chip_info(uint8 *info);
uint8_t *aic_get_fw_ver(void);
uint32_t aic_get_dpd_result(void);


#endif /* _SYS_DESIRE_H_ */