/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_board.h"

#ifdef AUDIO
#include <rtthread.h>

void audio_hardware_pa_init(void)
{
    BSP_GPIO_Set(54, 0, 1);
}

void audio_hardware_pa_start(uint32_t samplerate, uint32_t reserved)
{
    (void)samplerate;
    (void)reserved;
    /* NS4150B CTRL: high enables the amplifier; typical startup is 120 ms. */
    BSP_GPIO_Set(54, 1, 1);
    rt_thread_mdelay(120);
}

void audio_hardware_pa_stop(void)
{
    /* Keep CTRL low for the NS4150B's specified minimum shutdown time. */
    BSP_GPIO_Set(54, 0, 1);
    rt_thread_mdelay(80);
}
#endif

#if defined(SOC_BF0_HCPU) && defined(BSP_USING_PDM1) && !defined(CFG_BOOTLOADER)
#include <rtthread.h>

void BSP_Microphone_PowerOn(void)
{
    BSP_GPIO_Set(57, 1, 1);
    rt_thread_mdelay(10);
}

void BSP_Microphone_PowerOff(void)
{
    BSP_GPIO_Set(57, 0, 1);
}
#endif
