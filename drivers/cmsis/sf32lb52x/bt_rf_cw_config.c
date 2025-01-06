/**
  ******************************************************************************
  * @file   cw_config.c
  * @author Sifli software development team
  * @brief Header file - cw config.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2023 - 2023,  Sifli Technology
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


//#include <rtthread.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <board.h>


extern void ble_rf_power_set(int8_t txpwr);


static void cw_rf_power_set(uint8_t pa)
{
    int8_t pwr = 0;
    switch (pa)
    {
    case 0:
        pwr = 0;
        break;
    case 1:
        pwr = 4;
        break;
    case 2:
        pwr = 10;
        break;
    default:
        break;
    }
    ble_rf_power_set(pwr);
}

static void cw_rf_channel_set(uint8_t channel)
{
    //RADIOCNTL1    FORCE_TX_VAL    0x4009_0074[21] w   0x0 force tx off
    //hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL;
    HAL_Delay_us(20);

    //RADIOCNTL1    FORCE_CHANNEL   0x4009_0074[30] w   0x1 使能channel number的force
    //RADIOCNTL1    CHANNEL 0x4009_0074[29:23]  w   0xN 设置为chnnelN，即2402+2*N MHz
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_CHANNEL;
    hwp_bt_mac->DMRADIOCNTL1 |= (2 * channel) << BT_MAC_DMRADIOCNTL1_CHANNEL_Pos;
    HAL_Delay_us(20);
    hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_CHANNEL;

    hwp_bt_mac->DMRADIOCNTL1 |= 1 << BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL_Pos;
    HAL_Delay_us(20);
}

static void cw_config_start(uint8_t pa, uint8_t channel)
{

    //LB52X_MAC_RESET   LB55X_MAC_RESET 0x4000_0000[20] w   0x1 reset MAC
    hwp_lpsys_rcc->RSTR1 |= LPSYS_RCC_RSTR1_MAC;
    HAL_Delay_us(20);
    hwp_lpsys_rcc->RSTR1 &= ~LPSYS_RCC_RSTR1_MAC;

    //RADIOCNTL1    FORCE_SYNCWORD  0x4009_0074[31] w   0x1 使能syncword的force,避免受干扰误同步
    //RADIOCNTL1    FORCE_NBT_BLE   0x4009_0074[15] w   0x1 使能bt/ble mode选择的force
    //RADIOCNTL1    FORCE_NBT_BLE_VAL   0x4009_0074[14] w   0x1 设置为ble模式
    //RADIOCNTL1    FORCE_CHANNEL   0x4009_0074[30] w   0x1 使能channel number的force
    //RADIOCNTL1    CHANNEL 0x4009_0074[29:23]  w   0xN 设置为chnnelN，即2402+2*N MHz
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_SYNCWORD;
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_NBT_BLE;
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_NBT_BLE_VAL;
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_CHANNEL;
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_CHANNEL;

    hwp_bt_mac->DMRADIOCNTL1 |= 0x01 << BT_MAC_DMRADIOCNTL1_FORCE_SYNCWORD_Pos;
    hwp_bt_mac->DMRADIOCNTL1 |= 0x01 << BT_MAC_DMRADIOCNTL1_FORCE_NBT_BLE_Pos;
    hwp_bt_mac->DMRADIOCNTL1 |= 0x01 << BT_MAC_DMRADIOCNTL1_FORCE_NBT_BLE_VAL_Pos;
    //hwp_bt_mac->DMRADIOCNTL1 |= 0x01 << BT_MAC_DMRADIOCNTL1_FORCE_CHANNEL_Pos;

    //AESCNTL   FORCE_POLAR_PWR       0x4009_00B0[8]    w  0x1      使能polar power target的force
    //AESCNTL   FORCE_POLAR_PWR_VAL   0x4009_00B0[7:2]  w  0x3D     设置4dBm对应的polar power value
    hwp_bt_mac->AESCNTL &= ~BT_MAC_AESCNTL_FORCE_POLAR_PWR_VAL;
    hwp_bt_mac->AESCNTL &= ~BT_MAC_AESCNTL_FORCE_POLAR_PWR;

    hwp_bt_mac->AESCNTL |= BT_MAC_AESCNTL_FORCE_POLAR_PWR_Pos;
    hwp_bt_mac->AESCNTL |= 0x3D << BT_MAC_AESCNTL_FORCE_POLAR_PWR_VAL_Pos;

    //TRF_REG1  BRF_PA_PM_LV    0x4008_2834[2:1]    w   0x0
    //TRF_REG1  BRF_PA_CAS_BP_LV    0x4008_2834[0]  w   0x1
    //TRF_REG1  BRF_TRF_LDO_VREF_SEL_LV 0x4008_28534[20:17] w   0xD
    hwp_bt_rfc->TRF_REG1 &= ~BT_RFC_TRF_REG1_BRF_PA_PM_LV;
    hwp_bt_rfc->TRF_REG1 &= ~BT_RFC_TRF_REG1_BRF_PA_CAS_BP_LV;
    hwp_bt_rfc->TRF_REG1 &= ~BT_RFC_TRF_REG1_BRF_TRF_LDO_VREF_SEL_LV;

    hwp_bt_rfc->TRF_REG1 |= 0x1 << BT_RFC_TRF_REG1_BRF_PA_CAS_BP_LV_Pos;
    hwp_bt_rfc->TRF_REG1 |= 0xD << BT_RFC_TRF_REG1_BRF_TRF_LDO_VREF_SEL_LV_Pos;

    //TRF_REG2  BRF_PA_UNIT_SEL_LV  0x4008_2838[15:11]  w   0x1F
    //TRF_REG2  BRF_PA_MCAP_LV  0x4008_2838[10] w   0x1
    hwp_bt_rfc->TRF_REG2 &= ~BT_RFC_TRF_REG2_BRF_PA_MCAP_LV;
    hwp_bt_rfc->TRF_REG2 &= ~BT_RFC_TRF_REG2_BRF_PA_UNIT_SEL_LV;

    hwp_bt_rfc->TRF_REG2 |= 0x1 << BT_RFC_TRF_REG2_BRF_PA_MCAP_LV_Pos;
    hwp_bt_rfc->TRF_REG2 |= 0x1F << BT_RFC_TRF_REG2_BRF_PA_UNIT_SEL_LV_Pos;

    //-- TX_CTRL    MAC_PWR_CTRL_EN 0x4008_40EC[2]  w   0x0 使能pwr target的寄存器控制
    //TX_CTRL   MOD_METHOD_BR,MOD_METHOD_BLE    0x4008_40EC[3:2]    w   0x0 设置为polar 发射模式
    hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MAC_MOD_CTRL_EN;
    hwp_bt_phy->TX_CTRL &= ~(BT_PHY_TX_CTRL_MOD_METHOD_BLE | BT_PHY_TX_CTRL_MOD_METHOD_BR);

    //-- TX_PA_CFG  PA_CTRL_TARGET  0x5008_4114[7:2]    w   0x3D
    //TX_PA_CFG PA_RAMP_FORCE   0x4008_4120[1:0]    w   0x1 关闭ramp
    hwp_bt_phy->TX_PA_CFG &= ~BT_PHY_TX_PA_CFG_PA_RAMP_FORCE;
    hwp_bt_phy->TX_PA_CFG |= 0x1 << BT_PHY_TX_PA_CFG_PA_RAMP_FORCE_Pos;



    //RADIOCNTL1    FORCE_TX    0x5009_0074[22] w   0x1 使能tx force
    //RADIOCNTL1    FORCE_TX_VAL    0x5009_0074[21] w   0x1 force tx on
    hwp_bt_mac->DMRADIOCNTL1 |= 0x1 << BT_MAC_DMRADIOCNTL1_FORCE_TX_Pos;
    //hwp_bt_mac->DMRADIOCNTL1 |= 0x1 << BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL_Pos;
    HAL_Delay_us(20);

    cw_rf_power_set(pa);
    cw_rf_channel_set(channel);
}

static void cw_config_stop(void)
{
    hwp_lpsys_rcc->RSTR1 |= LPSYS_RCC_RSTR1_MAC;
    HAL_Delay_us(20);
    hwp_lpsys_rcc->RSTR1 &= ~LPSYS_RCC_RSTR1_MAC;

    //hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL;
}

/**
  * @brief  single-carrier test
  * @param  start or stop test
  * @param  power value, 0:0dbm 1:4dbm  2:10dbm
  * @param  set channel N, means 2402+2*N MHz
  */

void cw_config(uint8_t is_start, uint8_t pa, uint8_t channel)
{
    //RT_ASSERT((pa < 3) && (channel < 40));
    if (is_start)
        cw_config_start(pa, channel);
    else
        cw_config_stop();
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
