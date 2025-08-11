/** @file sdio.h
 *
 *  @brief SDIO Generic API related header file
 *
 *  Copyright 2021-2022 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef _SDIO_H_
#define _SDIO_H_

#include <wmerrno.h>
#include "type_decls.h"
#include <osa.h>

#ifndef __ZEPHYR__
    #include "fsl_sdmmc_common.h"
    #include "fsl_sdmmc_host.h"
    //#include "fsl_common.h"
    #include "sdmmc_config.h"
#else
    #include <wifi.h>
    #include <zephyr/sd/sdio.h>
#endif

/*! @brief Data block count accessed in card */
#define DATA_BLOCK_COUNT (4U)
/*! @brief Data buffer size. */
#ifndef __ZEPHYR__
    #define DATA_BUFFER_SIZE (FSL_SDMMC_DEFAULT_BLOCK_SIZE * DATA_BLOCK_COUNT)
#else
    #define DATA_BUFFER_SIZE (SDMMC_DEFAULT_BLOCK_SIZE * DATA_BLOCK_COUNT)
#endif

/** The number of times to try when polling for status bits */
#define MAX_POLL_TRIES 100U

#define sdio_io_e(...) wmlog_e("wifi_io", ##__VA_ARGS__)
#define sdio_io_w(...) wmlog_w("wifi_io", ##__VA_ARGS__)

#if CONFIG_SDIO_IO_DEBUG
    #define sdio_io_d(...) wmlog("wifi_io", ##__VA_ARGS__)
#else
    #define sdio_io_d(...)
#endif /* ! CONFIG_SDIO_IO_DEBUG */

#if CONFIG_SDIO_MULTI_PORT_RX_AGGR
    #define INBUF_SIZE (SDIO_MP_AGGR_DEF_PKT_LIMIT * 2 * DATA_BUFFER_SIZE)
#else
    #define INBUF_SIZE (2 * DATA_BUFFER_SIZE)
#endif /*CONFIG_SDIO_MULTI_PORT_RX_AGGR*/

extern uint8_t outbuf[];

extern uint8_t inbuf[];

/** Card Control Registers : Function 1 Block size 0 */
#define FN1_BLOCK_SIZE_0 0x110
/** Card Control Registers : Function 1 Block size 1 */
#define FN1_BLOCK_SIZE_1 0x111

int sdio_init(void);
int sdio_ioport_init(void);
void calculate_sdio_write_params(t_u32 txlen, t_u32 *tx_blocks, t_u32 *buflen);
bool wlan_card_status(t_u8 bits);
t_u16 wlan_card_read_f1_base_regs(void);
uint8_t *wifi_get_sdio_outbuf(uint32_t *outbuf_len);
unsigned int wifi_get_sdio_ioport();

#endif //_SDIO_H_
