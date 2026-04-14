/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"
#include "bf0_hal.h"
#include "rtthread.h"
#include <stdint.h>

#include "ll_hpsys_cfg.h"
#include "ll_pinmux.h"
#include "ll_usart.h"

#ifndef SF32LB52X
    #error "This example currently supports SF32LB52x only."
#endif

#define LL_USART_PORT hwp_usart2
#define LL_USART_BAUDRATE 115200U

/**
 * @brief Configure UART2 pins to USART2 function.
 */
static void ll_usart2_pinmux_init(void)
{
    ll_cfg_usart_pinr_config_t pinr_cfg;
    ll_pinmux_pad_config_t tx_pad_cfg = {
        .fsel = 4U,
        .pull = LL_PINMUX_PULL_UP,
        .input_en = LL_PINMUX_INPUT_DISABLE,
        .input_type = LL_PINMUX_INPUT_SCHMITT,
        .slew = LL_PINMUX_SLEW_FAST,
        .drive = LL_PINMUX_DRIVE_1,
    };
    ll_pinmux_pad_config_t rx_pad_cfg = tx_pad_cfg;

    rx_pad_cfg.input_en = LL_PINMUX_INPUT_ENABLE;

#if defined(BSP_USING_BOARD_SF32LB52_LCD_N16R8)
    pinr_cfg.txd_pa = 27U;
    pinr_cfg.rxd_pa = 20U;
#elif defined(BSP_USING_BOARD_SF32LB52_NANO_A128R16) ||                        \
    defined(BSP_USING_BOARD_SF32LB52_NANO_N16R16)
    pinr_cfg.txd_pa = 28U;
    pinr_cfg.rxd_pa = 25U;
#else
    #error "Please add UART2 pinmux mapping for this board."
#endif

    pinr_cfg.rts_pa = LL_CFG_PINR_FLOAT;
    pinr_cfg.cts_pa = LL_CFG_PINR_FLOAT;

    ll_cfg_config_usart2_pinr(hwp_hpsys_cfg, &pinr_cfg);
    ll_pinmux_config_pad(hwp_pinmux1, 13U + pinr_cfg.txd_pa, &tx_pad_cfg);
    ll_pinmux_config_pad(hwp_pinmux1, 13U + pinr_cfg.rxd_pa, &rx_pad_cfg);
}

/**
 * @brief Initialize USART2 with 115200-8N1.
 */
static void ll_usart2_init(void)
{
    ll_usart_frame_config_t frame = {
        .data_width = LL_USART_DATAWIDTH_8B,
        .parity = LL_USART_PARITY_NONE,
        .stop_bits = LL_USART_STOPBITS_1,
    };

    ll_usart_disable(LL_USART_PORT);
    ll_usart_config_frame(LL_USART_PORT, &frame);
    ll_usart_config_baudrate(LL_USART_PORT, LL_USART_BAUDRATE);
    ll_usart_config_hwflow(LL_USART_PORT, LL_USART_HWCONTROL_NONE);
    ll_usart_enable_tx(LL_USART_PORT);
    ll_usart_enable_rx(LL_USART_PORT);
    ll_usart_enable(LL_USART_PORT);

    while (ll_usart_is_active_flag_txe(LL_USART_PORT) == 0U)
    {
    }
}

/**
 * @brief Send one byte with TXE polling.
 * @param[in] data Byte to transmit.
 */
static void ll_usart2_send_byte(uint8_t data)
{
    while (ll_usart_is_active_flag_txe(LL_USART_PORT) == 0U)
    {
    }
    ll_usart_transmit_data8(LL_USART_PORT, data);
}

/**
 * @brief Send a C string by polling TXE/TC.
 * @param[in] str Null-terminated string.
 */
static void ll_usart2_send_string(const char *str)
{
    while (*str != '\0')
    {
        ll_usart2_send_byte((uint8_t)*str);
        str++;
    }

    while (ll_usart_is_active_flag_tc(LL_USART_PORT) == 0U)
    {
    }
    ll_usart_clear_flag_tc(LL_USART_PORT);
}

/**
 * @brief Receive one byte with RXNE polling.
 * @return Received byte.
 */
static uint8_t ll_usart2_receive_byte(void)
{
    while (ll_usart_is_active_flag_rxne(LL_USART_PORT) == 0U)
    {
    }

    return ll_usart_receive_data8(LL_USART_PORT);
}

/**
 * @brief Application entry for LL USART polling example.
 * @return This function does not return.
 */
int main(void)
{
    uint8_t ch;

    ll_usart2_pinmux_init();
    HAL_RCC_EnableModule(RCC_MOD_USART2);
    ll_usart2_init();

    ll_usart2_send_string("LL USART polling example started.\r\n");
    ll_usart2_send_string("Type any character, it will be echoed.\r\n");

    while (1)
    {
        ch = ll_usart2_receive_byte();
        ll_usart2_send_byte(ch);
    }
}
