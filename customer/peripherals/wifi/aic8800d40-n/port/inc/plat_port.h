/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform port header: ops-dispatch macros.
 */

#ifndef _PLAT_PORT_H_
#define _PLAT_PORT_H_

#include "aic_porting_types.h"

/* Common macros */
#define DRV_TRG_SDIO                64
#define DRV_TRG_SDIO_RX             65
#define DRV_TRG_FHOST_RX            66
#define DRV_TRG_FHOST_TX            67
#define DRV_TRG_NET_TX              68
#define DRV_TRG_POWCTR              69
#define DRV_TRG_USB                 70

/* Export variables (provided by platform or stubs) */
extern int aicwifi_vrf_mode;
extern int aicwifi_wpa_log_level;

/* ============================================================
 * Task Priority Configuration
 *
 * Priority direction configuration:
 *   CONFIG_PRIO_HIGH_IS_LARGE = 1: larger value means higher priority (default, e.g. FreeRTOS)
 *   CONFIG_PRIO_HIGH_IS_LARGE = 0: smaller value means higher priority (e.g. RT-Thread, NuttX)
 *
 * TCPIP_PRIORITY / SDIO_TASK_PRI are base values and should be set by the
 * target platform according to the actual priority range of its RTOS. Other
 * priorities are offset from the base via the PRIO_HIGHER / PRIO_LOWER macros,
 * with the direction adapted automatically by CONFIG_PRIO_HIGH_IS_LARGE.
 * When porting to a new RTOS/SDK, only the base values and the direction switch
 * need changing; the host sources stay untouched.
 * ============================================================ */
#ifndef CONFIG_PRIO_HIGH_IS_LARGE
    #define CONFIG_PRIO_HIGH_IS_LARGE  1
#endif

#if CONFIG_PRIO_HIGH_IS_LARGE
    /* larger value means higher priority */
    #define PRIO_HIGHER(base, offset)   ((base) + (offset))
    #define PRIO_LOWER(base, offset)    ((base) - (offset))
#else
    /* smaller value means higher priority -- offset direction is inverted */
    #define PRIO_HIGHER(base, offset)   ((base) - (offset))
    #define PRIO_LOWER(base, offset)    ((base) + (offset))
#endif

#define TCPIP_PRIORITY              (29)
#define TASK_END_PRIO               (32)
#define DYNAMIC_PRIORITY            (0)

#define SDIO_TASK_PRI               PRIO_HIGHER(TCPIP_PRIORITY, 2)
#define SDIO_DATRX_PRIORITY         PRIO_LOWER(SDIO_TASK_PRI, 1)
#define SDIO_OOBIRQ_PRIORITY        (SDIO_TASK_PRI)
#define SDIO_PWRCLT_PRIORITY        PRIO_LOWER(TCPIP_PRIORITY, 3)
#define FHOST_CNTRL_PRIORITY        PRIO_HIGHER(TCPIP_PRIORITY, 1)
#define FHOST_WPA_PRIORITY          (TCPIP_PRIORITY)
#ifndef PROTECT_TX_ADMA
    #define FHOST_TX_PRIORITY           PRIO_LOWER(TCPIP_PRIORITY, 1)
#else
    #define FHOST_TX_PRIORITY           PRIO_LOWER(TCPIP_PRIORITY, 2)
#endif
#define FHOST_RX_PRIORITY           PRIO_HIGHER(TCPIP_PRIORITY, 1)
#define FHOST_TCPACK_PRIORITY       (TCPIP_PRIORITY)
#define CLI_CMD_PRIORITY            PRIO_LOWER(TCPIP_PRIORITY, 10)
#define RWNX_TIMER_PRIORITY         PRIO_LOWER(SDIO_TASK_PRI, 1)
#define RWNX_WIFI_PRIORITY          (SDIO_TASK_PRI)
#define RWNX_APM_STALOSS_PRIORITY   PRIO_LOWER(TCPIP_PRIORITY, 1)

/* Exported priority variables — defined in plat_port.c */
extern uint32_t sdio_datrx_priority;
extern uint32_t sdio_oobirq_priority;
extern uint32_t sdio_pwrclt_priority;
extern uint32_t fhost_cntrl_priority;
extern uint32_t fhost_wpa_priority;
extern uint32_t fhost_tx_priority;
extern uint32_t fhost_rx_priority;
extern uint32_t fhost_tcpack_priority;
extern uint32_t cli_cmd_priority;
extern uint32_t rwnx_timer_priority;
extern uint32_t rwnx_wifi_priority;
extern uint32_t rwnx_apm_priority;
extern uint32_t tcpip_priority;
extern uint32_t task_end_prio;

/* ============================================================
 * Task Stack Size Configuration
 * ============================================================ */
#define SDIO_DATRX_STACK_SIZE       (2048)
#define SDIO_OOOBIRQ_STACK_SIZE     (2048)
#define SDIO_PWRCTL_STACK_SIZE      (1024)
#define FHOST_CNTRL_STACK_SIZE      (2048)
#define FHOST_WPA_STACK_SIZE        (3072)
#define FHOST_TX_STACK_SIZE         (2048)
#define FHOST_RX_STACK_SIZE         (2048)
#define FHOST_TCPACK_STACK_SIZE     (2048)
#define CLI_CMD_STACK_SIZE          (1024)
#define RWNX_TIMER_STACK_SIZE       (1024)
#define RWNX_WIFI_STACK_SIZE        (4096)
#define RWNX_APM_STACK_SIZE         (1024)

/* Exported stack size variables — defined in plat_port.c */
extern uint32_t sdio_datrx_stack_size;
extern uint32_t sdio_oobirq_stack_size;
extern uint32_t sdio_pwrctl_stack_size;
extern uint32_t fhost_cntrl_stack_size;
extern uint32_t fhost_wpa_stack_size;
extern uint32_t fhost_tx_stack_size;
extern uint32_t fhost_rx_stack_size;
extern uint32_t fhost_tcpack_stack_size;
extern uint32_t cli_cmd_stack_size;
extern uint32_t rwnx_timer_stack_size;
extern uint32_t rwnx_wifi_stack_size;
extern uint32_t rwnx_apm_stack_size;

#define RWNX_APM_QUEUE_SIZE         (32)

/* ---- HAL/Platform abstraction — platform must implement ALL functions ---- */

/* Power control */
void aicwf_platform_pwr_wifi_pin_init(void);
void aicwf_platform_pwr_wifi_pin_enable(void);
void aicwf_platform_pwr_wifi_pin_disable(void);

/* SDIO reset */
int  aicwf_sdio_reset(void);
void aicwf_sdio_pdn_wifi(void);
void aicwf_sdio_reset_wifi(void);

/* GPIO */
void aicwf_plat_gpio_trigger(uint8_t id, uint8_t value);

/* Debug/format */
uint32_t aicwf_dbg_snprintf(char *buffer, uint32_t size, const char *fmt, ...);
uint32_t aicwf_dbg_sprintf(char *buffer, const char *fmt, ...);
void aicwf_dump_buffer(char *tag, uint8_t *data, uint32_t len);

/* Misc */
uint32_t aicwf_hal_IsInProduction_Mode(void);
void aicwf_hal_switchPP(uint8_t lvl);
void aicwf_driver_unexcepted(unsigned int evt);
void aicwf_ATRESP(int handle, int type, int level, char *buf);

#endif /* _PLAT_PORT_H_ */
