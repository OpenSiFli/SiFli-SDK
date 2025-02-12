/**
  ******************************************************************************
  * @file   bluetooth_misc.c
  * @author Sifli software development team
  * @brief SIFLI Bluetooth stack misc c file.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2022 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include <stdbool.h>
#include <stdint.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#ifndef BSP_USING_PC_SIMULATOR
    #include "register.h"
#endif
#include "bluetooth_int.h"


#if (defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X)) && defined(BF0_LCPU)

#define BLUETOOTH_PTC_CONFIG

#if defined(BLUETOOTH_PTC_CONFIG)

#define RF_FORCE 0

static PTC_HandleTypeDef    PtcHandle[6];
const static uint8_t g_ptc_task[6] = {1, 2, 4, 5, 6, 7};

#ifdef PTC_DYN_CTRL
    #ifdef SOC_SF32LB58X
        #define PTC_CONFIG_ADDRESS 0x204e0000
    #else
        #error "Need reconfig address"
    #endif
#else // PTC_DYN_CTRL
#endif
#define PTC_CONFIG_NUMBER 5



uint32_t *g_ptc_config;


static void agc_config(uint8_t is_force)
{
    uint32_t agc_stat = hwp_bt_phy->AGC_STATUS;


    // LNA GAIN
    do
    {
        uint8_t lna_gain = (agc_stat & BT_PHY_AGC_STATUS_LNA_MIXER_GAIN_INDEX_Msk) >> BT_PHY_AGC_STATUS_LNA_MIXER_GAIN_INDEX_Pos;
        if ((lna_gain > 0) || is_force)
        {
#ifdef PTC_DYN_CTRL
            lna_gain = is_force ? g_ptc_config[0] : (lna_gain + 1);
#else
            lna_gain = (lna_gain + 1);
#endif
            hwp_bt_phy->AGC_CFG4 &= ~BT_PHY_AGC_CFG4_LNA_MIXER_GAIN_INDEX_INIT;
            hwp_bt_phy->AGC_CFG4 |= (lna_gain) << BT_PHY_AGC_CFG4_LNA_MIXER_GAIN_INDEX_INIT_Pos;
            //hwp_bt_phy->AGC_CFG4 |= (lna_gain + 1) << BT_PHY_AGC_CFG4_LNA_MIXER_GAIN_INDEX_INIT_Pos;
        }
        else
            break;

        // CBPF GAIN
        uint8_t cbpf_gain = (agc_stat & BT_PHY_AGC_STATUS_CBPF_GAIN_INDEX_Msk) >> BT_PHY_AGC_STATUS_CBPF_GAIN_INDEX_Pos;
#ifdef PTC_DYN_CTRL
        cbpf_gain = is_force ? g_ptc_config[1] : (cbpf_gain);
#endif
        //if (cbpf_gain > 0 || is_force)
        {
            hwp_bt_phy->AGC_CFG4 &= ~BT_PHY_AGC_CFG4_CBPF_GAIN_INDEX_INIT;
            hwp_bt_phy->AGC_CFG4 |= (cbpf_gain) << BT_PHY_AGC_CFG4_CBPF_GAIN_INDEX_INIT_Pos;
            //hwp_bt_phy->AGC_CFG4 |= (cbpf_gain) << BT_PHY_AGC_CFG4_CBPF_GAIN_INDEX_INIT_Pos;
        }

        // VGA GAIN
        int8_t vga_gain = (agc_stat & BT_PHY_AGC_STATUS_VGA_GAIN_INDEX_Msk) >> BT_PHY_AGC_STATUS_VGA_GAIN_INDEX_Pos;
#ifdef PTC_DYN_CTRL
        vga_gain = is_force ? g_ptc_config[2] : (vga_gain);
#endif
        //vga_gain = vga_gain > 6 ? (vga_gain) : 0;
        {
            hwp_bt_phy->AGC_CFG4 &= ~BT_PHY_AGC_CFG4_VGA_GAIN_INDEX_INIT;
            hwp_bt_phy->AGC_CFG4 |= vga_gain << BT_PHY_AGC_CFG4_VGA_GAIN_INDEX_INIT_Pos;
        }


        // DIG gain
        //uint8_t dig_gain = (agc_stat & BT_PHY_AGC_STATUS_ADC_DIG_GAIN_Msk) >> BT_PHY_AGC_STATUS_ADC_DIG_GAIN_Pos;
        hwp_bt_phy->AGC_CFG5 &= ~BT_PHY_AGC_CFG5_DIG_GAIN_LOW;
        hwp_bt_phy->AGC_CFG5 |= 0x10 << BT_PHY_AGC_CFG5_DIG_GAIN_LOW_Pos;
#ifdef PTC_DYN_CTRL
        HAL_Delay_us(g_ptc_config[3]);
#endif
        //HAL_GPIO_TogglePin(hwp_gpio2, (GPIO_B19 - GPIO_B0));
        // Disable AGC
        hwp_bt_phy->AGC_CTRL &= ~BT_PHY_AGC_CTRL_AGC_ENABLE_Msk;
    }
    while (0);

}

static void agc_recovery(void)
{
    hwp_bt_phy->AGC_CTRL |=  1 << BT_PHY_AGC_CTRL_AGC_ENABLE_Pos;

    hwp_bt_phy->AGC_CFG4 &= ~(BT_PHY_AGC_CFG4_LNA_MIXER_GAIN_INDEX_INIT | BT_PHY_AGC_CFG4_CBPF_GAIN_INDEX_INIT
                              | BT_PHY_AGC_CFG4_VGA_GAIN_INDEX_INIT);
    hwp_bt_phy->AGC_CFG5 &= ~BT_PHY_AGC_CFG5_DIG_GAIN_LOW;
    hwp_bt_phy->AGC_CFG5 |= (0x10) << BT_PHY_AGC_CFG5_DIG_GAIN_LOW_Pos;
}

void PTC2_IRQHandler(void)
{
    uint8_t isr_flag = hwp_ptc2->ISR & 0xFF;

    rt_interrupt_enter();
    if ((isr_flag & (1 << g_ptc_task[0])) != 0)
    {
        HAL_PTC_IRQHandler(&PtcHandle[0]);
        agc_config(RF_FORCE);
    }

    if ((isr_flag & (1 << g_ptc_task[1])) != 0)
    {
        HAL_PTC_IRQHandler(&PtcHandle[1]);
        agc_config(RF_FORCE);
    }

    if ((isr_flag & (1 << g_ptc_task[2])) != 0)
    {
        HAL_PTC_IRQHandler(&PtcHandle[2]);
        agc_recovery();
    }

    if ((isr_flag & (1 << g_ptc_task[3])) != 0)
    {
        HAL_PTC_IRQHandler(&PtcHandle[3]);
        agc_recovery();
    }
#if SF_WLAN_COEX
    if ((isr_flag & (1 << g_ptc_task[4])) != 0)
    {
        HAL_PTC_IRQHandler(&PtcHandle[4]);
    }
    if ((isr_flag & (1 << g_ptc_task[5])) != 0)
    {
        HAL_PTC_IRQHandler(&PtcHandle[5]);
    }
#endif

    rt_interrupt_leave();


}


void ptc_config(uint8_t index, uint8_t sel_idx, uint8_t tripol, uint16_t delay)
{
    PtcHandle[index].Instance = hwp_ptc2;
    PtcHandle[index].Init.Channel = g_ptc_task[index];                                 // Use PTC Channel 0
    PtcHandle[index].Init.Address = (uint32_t) & (hwp_ptc2->RSVD2[g_ptc_task[index]]);   // Bus monitor clear register
    PtcHandle[index].Init.data = 0;     // data to handle with value in Address.
    PtcHandle[index].Init.Operation = PTC_OP_WRITE;                       // Or and write back
    PtcHandle[index].Init.Sel = sel_idx;
    PtcHandle[index].Init.Tripol = tripol;
    PtcHandle[index].Init.Delay = delay;
    NVIC_EnableIRQ(PTC2_IRQn);

    if (HAL_PTC_Init(& PtcHandle[index]) != HAL_OK)                     // Initialize PTC
    {
        /* Initialization Error */
        RT_ASSERT(RT_FALSE);
    }
    HAL_PTC_Enable(&PtcHandle[index], 1);                              // Enable PTC

}

static uint16_t  ptc_delay_cal(uint16_t ns)
{
    static uint32_t sysclk_m;
    if (ns == 0 || sysclk_m == 0)
    {
        sysclk_m = HAL_RCC_GetHCLKFreq(CORE_ID_DEFAULT) / 1000000;
        if (ns == 0)
            return 0;
    }

    return (sysclk_m * ns / 1000);

}

#if SF_WLAN_COEX

#ifdef SOC_SF32LB58X
//PB10
uint32_t g_pta_address[2] = {0x50080008, 0x5008000C};
uint32_t g_pta_data[2] = {0x400, 0x400};
#endif
void pta_ptc_config(uint8_t index, uint8_t sel_idx, uint8_t tripol, uint16_t delay)
{
    PtcHandle[index].Instance = hwp_ptc2;
    PtcHandle[index].Init.Channel = g_ptc_task[index];
    PtcHandle[index].Init.Address = g_pta_address[index - 4];
    PtcHandle[index].Init.data = g_pta_data[index - 4];   // data to handle with value in Address.
    PtcHandle[index].Init.Operation = PTC_OP_OR;                       // Or and write back
    PtcHandle[index].Init.Sel = sel_idx;
    PtcHandle[index].Init.Tripol = tripol;
    PtcHandle[index].Init.Delay = delay;
    NVIC_EnableIRQ(PTC2_IRQn);

    if (HAL_PTC_Init(& PtcHandle[index]) != HAL_OK)                     // Initialize PTC
    {
        /* Initialization Error */
        RT_ASSERT(RT_FALSE);
    }
    HAL_PTC_Enable(&PtcHandle[index], 1);                              // Enable PTC

    PtcHandle[index].Instance->IER &= (~(1UL << PtcHandle[index].Init.Channel));//disable isr

}
#endif

#endif // BLUETOOTH_PTC_CONFIG

void rf_ptc_config(uint8_t is_reset)
{
#ifdef BLUETOOTH_PTC_CONFIG
    HAL_RCC_EnableModule(RCC_MOD_PTC2);
    if (is_reset)
    {
#ifdef PTC_DYN_CTRL
        memset((void *)PTC_CONFIG_ADDRESS, 0, PTC_CONFIG_NUMBER * 4);
        g_ptc_config = (uint32_t *)PTC_CONFIG_ADDRESS;
#endif
    }

#if 1
    ptc_config(0, PTC_LCPU_BT_EDR2, 0, ptc_delay_cal(10667));
    ptc_config(1, PTC_LCPU_BT_EDR3, 0, ptc_delay_cal(10667));
#else
    ptc_config(0, PTC_LCPU_RF_PKTDET);
#endif
    ptc_config(2, PTC_LCPU_BT_EDR2, 1, 0); //PTC_LCPU_BT_RXDONE
    ptc_config(3, PTC_LCPU_BT_EDR3, 1, 0);

#if SF_WLAN_COEX
    pta_ptc_config(4, PTC_LCPU_BT_PRIORITY, 0, 0);
    pta_ptc_config(5, PTC_LCPU_BT_PRIORITY, 1, 0);
#endif

#endif // BLUETOOTH_PTC_CONFIG
}

#endif // defined(SOC_SF32LB58X) && defined(BF0_LCPU)

#if (defined(SOC_SF32LB52X)) && defined(BF0_LCPU)

static PTC_HandleTypeDef    PtcHandle[1];
const static uint8_t g_ptc_task[1] = {1};//{1, 2, 4, 5};

typedef struct
{
    uint32_t cfo_phase[2];
    uint32_t  cnt;
} cfo_phase_t;

cfo_phase_t *pt_cfo = (cfo_phase_t *)0x40082790;

void ptc_save_phase(void)
{
    uint16_t phase_tmp = hwp_bt_phy->RX_STATUS1 & BT_PHY_RX_STATUS1_CFO_PHASE;


    if (phase_tmp >= 0x800)
    {
        phase_tmp = (~phase_tmp) & BT_PHY_RX_STATUS1_CFO_PHASE;
    }

    if (phase_tmp == 0)
    {
        return;
    }

#if 0
    uint32_t  pos, old_phase;
    pos = (pt_cfo->cnt >> 1);
    old_phase = pt_cfo->cfo_phase[pos];
    if (pt_cfo->cnt & 1)
    {
        pt_cfo->cfo_phase[pos] = (((uint32_t)phase_tmp << 16) | (old_phase & 0xFFFF));
    }
    else
    {
        pt_cfo->cfo_phase[pos] = (((uint32_t)phase_tmp) | (old_phase & 0xFFFF0000));
    }

    pt_cfo->cnt = (pt_cfo->cnt + 1) & 0x3;
#else
    pt_cfo->cfo_phase[0] = phase_tmp;
#endif
}

void PTC2_IRQHandler(void)
{
    uint8_t isr_flag = hwp_ptc2->ISR & 0xFF;

    rt_interrupt_enter();
    if ((isr_flag & (1 << g_ptc_task[0])) != 0)
    {
        HAL_PTC_IRQHandler(&PtcHandle[0]);

        ptc_save_phase();
    }

    rt_interrupt_leave();


}

void ptc_config(uint8_t index, uint8_t sel_idx, uint8_t tripol, uint16_t delay)
{
    PtcHandle[index].Instance = hwp_ptc2;
    PtcHandle[index].Init.Channel = g_ptc_task[index];                                 // Use PTC Channel 0
    PtcHandle[index].Init.Address = (uint32_t) & (pt_cfo->cfo_phase[0]);
    PtcHandle[index].Init.data = 0;     // data to handle with value in Address.
    PtcHandle[index].Init.Operation = PTC_OP_OR;                       // Or and write back
    PtcHandle[index].Init.Sel = sel_idx;
    PtcHandle[index].Init.Tripol = tripol;
    PtcHandle[index].Init.Delay = delay;
    HAL_NVIC_SetPriority(PTC2_IRQn, 0, 0);
    //NVIC_EnableIRQ(PTC2_IRQn);

    if (HAL_PTC_Init(& PtcHandle[index]) != HAL_OK)                     // Initialize PTC
    {
        /* Initialization Error */
        RT_ASSERT(RT_FALSE);
    }
    HAL_PTC_Enable(&PtcHandle[index], 1);                              // Enable PTC

}
void rf_ptc_config(uint8_t is_reset)
{
#if (!defined(USING_SEC_ENV)) && (!defined(FPGA))
    if (is_reset)
    {
        memset((void *)pt_cfo, 0, 10);
    }
    HAL_RCC_EnableModule(RCC_MOD_PTC2);

    ptc_config(0, PTC_LCPU_BT_PKTDET, 0, 0);
#endif
}

#endif

#if defined(SF_WLAN_COEX) && defined(BF0_LCPU)
void pta_io_config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_PIN_Set(PAD_PB08, BT_ACTIVE,   PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB09, WLAN_ACTIVE, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB10, GPIO_B10,  PIN_PULLDOWN, 0);

    GPIO_InitStruct.Pin = 8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init((GPIO_TypeDef *)hwp_gpio2, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = 10;
    HAL_GPIO_Init((GPIO_TypeDef *)hwp_gpio2, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = 9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init((GPIO_TypeDef *)hwp_gpio2, &GPIO_InitStruct);

}

void wlan_coex_config(void)
{
    hwp_bt_mac->BTCOEXIFCNTL1 = (4 << BT_MAC_BTCOEXIFCNTL1_WLCPRXTHR_Pos);
    hwp_bt_mac->BTCOEXIFCNTL2 |= BT_MAC_BTCOEXIFCNTL2_PTA_MASKTX |
                                 BT_MAC_BTCOEXIFCNTL2_PTA_MASKRX  |
                                 (3 << BT_MAC_BTCOEXIFCNTL2_PTA_ACTSEL_Pos);

    hwp_bt_mac->BTCOEXIFCNTL0 |= BT_MAC_BTCOEXIFCNTL0_WLANCOEX_EN;
    hwp_bt_mac->BLECOEXIFCNTL0 |= BT_MAC_BLECOEXIFCNTL0_WLANCOEX_EN;
}

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
