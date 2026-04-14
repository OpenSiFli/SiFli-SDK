/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DRV_CONFIG_H__
#define __DRV_CONFIG_H__

#include <board.h>
#include <rtconfig.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "dma_config.h"
#include "uart_config.h"
#include "spi_config.h"
#include "i2c_config.h"
#include "tim_config.h"
#include "pwm_config.h"
#include "adc_config.h"
#include "sdio_config.h"
#include "flash_config.h"
#include "audio_config.h"

enum PLL_SET_GRADE
{
    PLL_ADD_ONE_HUND_PPM,
    PLL_ADD_TWO_HUND_PPM,
    PLL_SUB_ONE_HUND_PPM,
    PLL_SUB_TWO_HUND_PPM,
    PLL_ADD_TEN_PPM,
    PLL_SUB_TEN_PPM,
};

#ifdef __cplusplus
}
#endif

#endif
