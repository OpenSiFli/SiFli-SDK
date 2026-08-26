/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#include <rtthread.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <board.h>




#define  USEED_NEW_PWR_CONFIG   1

static const int8_t rf_blebr_db[] = {0, 4, 10, 13, 16, 19};
static const int8_t rf_edr_db[] = {0, 3, 6, 10, 13};


static uint8_t rf_iq_tx_ctrl_force_set(uint8_t is_edr, int8_t pwr)
{
    uint8_t ret = 0;
    uint8_t power_level;

    if ((pwr >= -3) && (pwr < 13))
    {
        power_level = (uint8_t)(pwr + 3);
    }
    else if (pwr < -3)
    {
        power_level = 0; //min power level
    }
    else
    {
        power_level = 15; //max power level
    }


    if (ret == 0)
    {
        hwp_bt_mac->AESCNTL &= ~BT_MAC_AESCNTL_FORCE_IQ_PWR_VAL;
        hwp_bt_mac->AESCNTL |= power_level << BT_MAC_AESCNTL_FORCE_IQ_PWR_VAL_Pos;
        hwp_bt_mac->AESCNTL &= ~BT_MAC_AESCNTL_FORCE_IQ_PWR;
        hwp_bt_mac->AESCNTL |= 1 << BT_MAC_AESCNTL_FORCE_IQ_PWR_Pos;
    }
    else
    {
        //rt_kprintf("set power error!");
    }
    return ret;

}

extern uint32_t bt_rf_polar_get_pwr_level_para(int8_t pwr);
static uint8_t blebr_rf_power_set(int8_t txpwr)
{
    uint8_t ret = 0;
    hwp_bt_mac->AESCNTL |= BT_MAC_AESCNTL_FORCE_POLAR_PWR; // Force dedicated value

    uint32_t lvl_para;
    uint8_t  level_val, pwr_val;

    //rt_kprintf("set txpwr %d, actully pwr %d\r\n", txpwr, rf_blebr_db[i]);
#if 1
    if (txpwr <= 10)
    {
        lvl_para = bt_rf_polar_get_pwr_level_para(txpwr);
        level_val = (uint8_t)(lvl_para & 0xE) >> 1;
        pwr_val = (uint8_t)((lvl_para >> 4) & 0xFF);
        hwp_bt_mac->AESCNTL &= ~BT_MAC_AESCNTL_FORCE_POLAR_PWR_VAL;
        hwp_bt_mac->AESCNTL |= pwr_val << BT_MAC_AESCNTL_FORCE_POLAR_PWR_VAL_Pos;
        //hwp_bt_mac->AESCNTL &= ~BT_MAC_AESCNTL_FORCE_POLAR_PWR;
        //hwp_bt_mac->AESCNTL |= 1 << BT_MAC_AESCNTL_FORCE_POLAR_PWR_Pos;
        hwp_bt_mac->AESCNTL &= ~BT_MAC_AESCNTL_FORCE_POLAR_LEVEL_VAL;
        hwp_bt_mac->AESCNTL |= level_val << BT_MAC_AESCNTL_FORCE_POLAR_LEVEL_VAL_Pos;
        hwp_bt_mac->AESCNTL |= BT_MAC_AESCNTL_FORCE_POLAR_LEVEL;
    }
    else
#endif
    {
        hwp_bt_mac->AESCNTL |= BT_MAC_AESCNTL_FORCE_IQ_PWR;
        hwp_bt_phy->TX_CTRL |= (BT_PHY_TX_CTRL_MOD_METHOD_BLE | BT_PHY_TX_CTRL_MOD_METHOD_BR);
        ret = rf_iq_tx_ctrl_force_set(0, txpwr);
    }
    return ret;
}

static uint8_t edr_rf_power_set(int8_t txpwr)
{
    uint8_t ret = 0;
    hwp_bt_mac->AESCNTL |= BT_MAC_AESCNTL_FORCE_IQ_PWR; // Force dedicated value
#if 0
    uint32_t i, max = sizeof(rf_edr_db) / sizeof(rf_edr_db[0]);

    for (i = 0; i < max - 1; i++)
    {
        if (rf_edr_db[i] >= txpwr)
            break;
    }

    //rt_kprintf("set txpwr %d, actully pwr %d\r\n", txpwr, rf_edr_db[i]);

    ret = rf_iq_tx_ctrl_force_set(1, rf_edr_db[i]);
#else
    ret = rf_iq_tx_ctrl_force_set(1, txpwr);
#endif
    return ret;
}

/**
  * @brief  RF set power interface,just used for RF test
  * @param[in]  type, 0:ble or br, 1:edr
  * @param[in]  tx power,dbm unit
  * @param[out]     return param,0:sucess, other:error
  */
uint8_t btdm_rf_power_set(uint8_t type, int8_t txpwr)
{
    uint8_t ret = 0;
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MAC_MOD_CTRL_EN;
    if (type == 0)
    {
        ret = blebr_rf_power_set(txpwr);
    }
    else if (type == 1)
    {
        ret = edr_rf_power_set(txpwr);
    }
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();

    return ret;
}

//this interface should deprecated, new interface btdm_rf_power_set add return paramters
void blebredr_rf_power_set(uint8_t type, int8_t txpwr)
{
    btdm_rf_power_set(type, txpwr);
}
