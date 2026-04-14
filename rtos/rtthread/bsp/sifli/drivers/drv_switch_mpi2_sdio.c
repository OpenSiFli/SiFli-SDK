/*
 * SPDX-FileCopyrightText: 2025-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtthread.h"
#include "board.h"
#include "bf0_hal_mpi.h"
#include "drv_flash.h"
#include "drv_sdio.h"

/**
 * @brief  Handle SDIO timeout event.
 *
 * This function is implemented elsewhere and is called after
 * switching to SDIO mode to clear or handle pending timeout states.
 */
void rt_hw_sdio_timeout_handle(void);

typedef enum
{
    SWITCH_MPI2_SDIO_STATE_UNKNOWN = 0,
    SWITCH_MPI2_SDIO_STATE_SDIO1 = 1,
    SWITCH_MPI2_SDIO_STATE_MPI2 = 2,
} sdio1_mpi2_switch_state_t;


static sdio1_mpi2_switch_state_t pinmux_state = SWITCH_MPI2_SDIO_STATE_UNKNOWN;

static struct rt_mutex  sdio_sw_mutex;

/**
 * @brief  Initialize SDIO/MPI2 switch related resources.
 *
 * This function initializes the mutex used to protect the
 * SDIO/MPI2 pinmux switching.
 *
 * @return Always returns 0 on success.
 */
static int rt_switch_init(void)
{
    rt_err_t result;
    result = rt_mutex_init(&sdio_sw_mutex, "sdio_sw_mutex", RT_IPC_FLAG_FIFO);
    RT_ASSERT(result == RT_EOK);
    return 0;
}
INIT_PREV_EXPORT(rt_switch_init);

void rt_switch_sdio_lock(void)
{
    rt_err_t result;

    result = rt_mutex_take(&sdio_sw_mutex, RT_WAITING_FOREVER);
    RT_ASSERT(result == RT_EOK);
    if (pinmux_state != SWITCH_MPI2_SDIO_STATE_SDIO1)
    {
        HAL_NVIC_EnableIRQ(SDIO_IRQn);
        rt_hw_sdio_timeout_handle();
        BSP_GPIO_Set(BSP_SWITCH_MPI2_SDIO_PIN, 0, 1);
        HAL_Delay_us(5);
        HAL_PIN_Set(PAD_PA14, SD1_CLK,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA15, SD1_CMD, PIN_PULLUP, 1);
        HAL_Delay_us(20);
        HAL_PIN_Set(PAD_PA12, SD1_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA13, SD1_DIO3, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA16, SD1_DIO0, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA17, SD1_DIO1, PIN_PULLUP, 1);
        pinmux_state = SWITCH_MPI2_SDIO_STATE_SDIO1;
    }
}

void rt_switch_sdio_unlock(void)
{
    rt_err_t result;

    result = rt_mutex_release(&sdio_sw_mutex);
    RT_ASSERT(result == RT_EOK);
}

void rt_switch_mpi2_lock(void)
{
    rt_err_t result;

    result = rt_mutex_take(&sdio_sw_mutex, RT_WAITING_FOREVER);
    RT_ASSERT(result == RT_EOK);
    if (pinmux_state != SWITCH_MPI2_SDIO_STATE_MPI2)
    {
        HAL_NVIC_DisableIRQ(SDIO_IRQn);

        BSP_GPIO_Set(BSP_SWITCH_MPI2_SDIO_PIN, 1, 1);
        HAL_Delay_us(5);
        HAL_PIN_Set(PAD_PA16, MPI2_CLK,  PIN_NOPULL,   1);
        HAL_PIN_Set(PAD_PA12, MPI2_CS,   PIN_NOPULL,   1);
        HAL_PIN_Set(PAD_PA15, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA13, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA14, MPI2_DIO2, PIN_PULLUP,   1);
        HAL_PIN_Set(PAD_PA17, MPI2_DIO3, PIN_PULLUP,   1);
        pinmux_state = SWITCH_MPI2_SDIO_STATE_MPI2;
    }

}

void rt_switch_mpi2_unlock(void)
{
    rt_err_t result;

    result = rt_mutex_release(&sdio_sw_mutex);
    RT_ASSERT(result == RT_EOK);
}