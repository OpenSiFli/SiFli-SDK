/**
  ******************************************************************************
  * @file   bt_power_config.c
  * @author Sifli software development team
  * @brief Source file - BT hop test.
 *
  ******************************************************************************
*/
/*
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




static const int8_t rf_blebr_db[] = {0, 4, 10, 13, 16, 19};
static const int8_t rf_edr_db[] = {0, 3, 6, 10, 13};

static void rf_tx_power_basic_config(int8_t basic_dbm)
{
    hwp_bt_rfc->TRF_REG1 &= ~BT_RFC_TRF_REG1_BRF_PA_PM_LV_Msk;
    hwp_bt_rfc->TRF_REG1 &= ~BT_RFC_TRF_REG1_BRF_PA_CAS_BP_LV_Msk;

    hwp_bt_rfc->TRF_REG2 &= ~BT_RFC_TRF_REG2_BRF_PA_UNIT_SEL_LV_Msk;
    hwp_bt_rfc->TRF_REG2 &= ~BT_RFC_TRF_REG2_BRF_PA_MCAP_LV_Msk;

    switch (basic_dbm)
    {
    case 0:
    {
        hwp_bt_rfc->TRF_REG1 |= 0x01 << BT_RFC_TRF_REG1_BRF_PA_PM_LV_Pos;
        hwp_bt_rfc->TRF_REG1 |= 0x01 << BT_RFC_TRF_REG1_BRF_PA_CAS_BP_LV_Pos;

        hwp_bt_rfc->TRF_REG2 |= 0x01 << BT_RFC_TRF_REG2_BRF_PA_UNIT_SEL_LV_Pos;
        hwp_bt_rfc->TRF_REG2 |= 0x0 << BT_RFC_TRF_REG2_BRF_PA_MCAP_LV_Pos;
    }
    break;
    case 4:
    {
        hwp_bt_rfc->TRF_REG1 |= 0x00 << BT_RFC_TRF_REG1_BRF_PA_PM_LV_Pos;
        hwp_bt_rfc->TRF_REG1 |= 0x01 << BT_RFC_TRF_REG1_BRF_PA_CAS_BP_LV_Pos;

        hwp_bt_rfc->TRF_REG2 |= 0x1F << BT_RFC_TRF_REG2_BRF_PA_UNIT_SEL_LV_Pos;
        hwp_bt_rfc->TRF_REG2 |= 0x01 << BT_RFC_TRF_REG2_BRF_PA_MCAP_LV_Pos;
    }
    break;
    case 10:
    {
        hwp_bt_rfc->TRF_REG1 |= 0x02 << BT_RFC_TRF_REG1_BRF_PA_PM_LV_Pos;
        hwp_bt_rfc->TRF_REG1 |= 0x00 << BT_RFC_TRF_REG1_BRF_PA_CAS_BP_LV_Pos;

        hwp_bt_rfc->TRF_REG2 |= 0x1F << BT_RFC_TRF_REG2_BRF_PA_UNIT_SEL_LV_Pos;
        hwp_bt_rfc->TRF_REG2 |= 0x01 << BT_RFC_TRF_REG2_BRF_PA_MCAP_LV_Pos;
    }
    break;
        //default:
        //RT_ASSERT(0);

    }

}

static uint8_t rf_tx_power_cs_get(int8_t pwr)
{
    // Just handle 0,3,6,10
    uint8_t cs_val = 0x3E;
    if (pwr == 3)
        cs_val = 0x34;
    else if (pwr == 6)
        cs_val = 0x24;

    return cs_val;
}

static int8_t rf_iq_tx_ctrl_force_set(uint8_t is_edr, int8_t pwr)
{
    int8_t ret = 0;
    uint8_t power_level = 0;
    if (is_edr)
    {

        switch (pwr)
        {
        case 0:
        {
            power_level = 0;
            break;
        }
        case 3:
        {
            power_level = 1;
            break;
        }
        case 6:
        {
            power_level = 2;
            break;
        }
        case 10:
        {
            power_level = 3;
            break;
        }
        case 13:
        {
            power_level = 4;
            break;
        }
        default:
            ret = -1;
            break;
        }
    }
    else if (pwr >= 13)
    {
        switch (pwr)
        {
        case 13:
        {
            power_level = 5;
            break;
        }
        case 16:
        {
            power_level = 6;
            break;
        }
        case 19:
        {
            power_level = 7;
            break;
        }
        default:
            ret = -1;
            break;

        }
    }
    else
        ret = -1;

    if (ret == 0)
    {
        hwp_bt_mac->AESCNTL &= ~BT_MAC_AESCNTL_FORCE_IQ_PWR_VAL;
        hwp_bt_mac->AESCNTL |= power_level << BT_MAC_AESCNTL_FORCE_IQ_PWR_VAL_Pos;
    }
    else
    {
        //rt_kprintf("set power error!");
    }
    return ret;
}

void ble_rf_power_set(int8_t txpwr)
{
    hwp_bt_mac->AESCNTL |= BT_MAC_AESCNTL_FORCE_POLAR_PWR; // Force dedicated value

    uint32_t i, max = sizeof(rf_blebr_db) / sizeof(rf_blebr_db[0]);

    for (i = 0; i < max - 1; i++)
    {
        if (rf_blebr_db[i] >= txpwr)
            break;
    }
    //rt_kprintf("br/ble set txpwr %d, actully pwr %d\r\n", txpwr, rf_blebr_db[i]);

    if (rf_blebr_db[i] <= 10)
    {
        hwp_bt_phy->TX_CTRL &= ~(BT_PHY_TX_CTRL_MOD_METHOD_BLE | BT_PHY_TX_CTRL_MOD_METHOD_BR);
        rf_tx_power_basic_config(rf_blebr_db[i]);
        uint8_t pwr_lvl = rf_tx_power_cs_get(txpwr);
        hwp_bt_mac->AESCNTL &= ~BT_MAC_AESCNTL_FORCE_POLAR_PWR_VAL;
        hwp_bt_mac->AESCNTL |= pwr_lvl << BT_MAC_AESCNTL_FORCE_POLAR_PWR_VAL_Pos;
    }
    else
    {
        hwp_bt_phy->TX_CTRL |= (BT_PHY_TX_CTRL_MOD_METHOD_BLE | BT_PHY_TX_CTRL_MOD_METHOD_BR);

        rf_iq_tx_ctrl_force_set(0, rf_blebr_db[i]);
    }
}

static void bt_rf_power_set(int8_t txpwr)
{
    hwp_bt_mac->AESCNTL |= BT_MAC_AESCNTL_FORCE_IQ_PWR; // Force dedicated value

    uint32_t i, max = sizeof(rf_edr_db) / sizeof(rf_edr_db[0]);

    for (i = 0; i < max - 1; i++)
    {
        if (rf_edr_db[i] >= txpwr)
            break;
    }

    //rt_kprintf("edr set txpwr %d, actully pwr %d\r\n", txpwr, rf_edr_db[i]);

    rf_iq_tx_ctrl_force_set(1, rf_edr_db[i]);
}

void rf_power_set(int8_t tx_pwr, uint8_t is_edr)
{
    hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MAC_MOD_CTRL_EN;

    //ble_rf_power_set(tx_pwr, bt_mode);
    //bt_rf_power_set(tx_pwr, bt_mode);

    if (!is_edr)
    {
        ble_rf_power_set(tx_pwr);
    }
    else
    {
        bt_rf_power_set(tx_pwr);
    }
    //hwp_bt_phy->TX_CTRL |= BT_PHY_TX_CTRL_MAC_MOD_CTRL_EN;
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
