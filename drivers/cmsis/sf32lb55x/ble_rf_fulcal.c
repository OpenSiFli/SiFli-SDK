/**
  ******************************************************************************
  * @file   bt_rf_fulcal.c
  * @author Sifli software development team
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


#include "bf0_hal_rcc.h"
#include "register.h"
#include "ble_rf_cal.h"

#if !defined(CFG_FACTORY_DEBUG)


#define _HAL_Delay_us HAL_Delay_us
//#define RF_PRINTF rt_kprintf
#define RF_PRINTF(...)

// 1 (sltu) + 1 (addiu) + 3 (btnez) = 5 cycles
#define WAIT_US_LOOP_CYCLE 5

const static uint32_t ref_residual_cnt_tbl_tx[40] = {30545,
                                                     30625,
                                                     30705,
                                                     30785,
                                                     30865,
                                                     30945,
                                                     31025,
                                                     31105,
                                                     31185,
                                                     31265,
                                                     31345,
                                                     31425,
                                                     31505,
                                                     31585,
                                                     31665,
                                                     31745,
                                                     31825,
                                                     31905,
                                                     31985,
                                                     32065,
                                                     32145,
                                                     32225,
                                                     32305,
                                                     32385,
                                                     32465,
                                                     32545,
                                                     32625,
                                                     32705,
                                                     32785,
                                                     32865,
                                                     32945,
                                                     33025,
                                                     33105,
                                                     33185,
                                                     33265,
                                                     33345,
                                                     33425,
                                                     33505,
                                                     33585,
                                                     33665
                                                    } ;
const static uint32_t ref_residual_cnt_tbl_rx_1m[40] =
{
    30485,
    30565,
    30645,
    30725,
    30805,
    30885,
    30965,
    31045,
    31125,
    31205,
    31285,
    31365,
    31445,
    31525,
    31605,
    31685,
    31765,
    31845,
    31925,
    32005,
    32085,
    32165,
    32245,
    32325,
    32405,
    32485,
    32565,
    32645,
    32725,
    32805,
    32885,
    32965,
    33045,
    33125,
    33205,
    33285,
    33365,
    33445,
    33525,
    33605
};

const static  uint32_t ref_residual_cnt_tbl_rx_2m[40] =
{
    30425,
    30505,
    30585,
    30665,
    30745,
    30825,
    30905,
    30985,
    31065,
    31145,
    31225,
    31305,
    31385,
    31465,
    31545,
    31625,
    31705,
    31785,
    31865,
    31945,
    32025,
    32105,
    32185,
    32265,
    32345,
    32425,
    32505,
    32585,
    32665,
    32745,
    32825,
    32905,
    32985,
    33065,
    33145,
    33225,
    33305,
    33385,
    33465,
    33545
};


typedef struct
{
    int8_t base_dbm;
    int8_t tx_pwr;
    uint8_t tx_lvl;
} ble_rf_tx_pwr_lvl_tb_t;

#define BLE_RF_MAX_STEP_COUNT 21

const static ble_rf_tx_pwr_lvl_tb_t tx_pwr_lvl_tb[BLE_RF_MAX_STEP_COUNT + 1] =
{
    {0, -10, 27},
    {0, -9,  29},
    {0, -8,  30},
    {0, -7,  32},
    {0, -6,  34},
    {0, -5,  36},
    {0, -4,  39},
    {0, -3,  43},
    {0, -2,  47},
    {0, -1,  52},
    {0,  0,  62},
    {4,  1,  43},
    {4,  2,  47},
    {4,  3,  52},
    {4,  4,  61},
    {10, 5,  32},
    {10, 6,  36},
    {10, 7,  40},
    {10, 8,  46},
    {10, 9,  52},
    {10, 10, 62},
    {0,  0,  62}, // Default value.
};


__WEAK int8_t bt_rf_get_init_tx_pwr(void)
{
    return 0;
}


static void ble_rf_tx_power_basic_config(int8_t basic_dbm)
{
    hwp_ble_rfc->TRF_REG1 &= ~BLE_RF_DIG_TRF_REG1_BRF_PA_PM_LV_Msk;
    hwp_ble_rfc->TRF_REG1 &= ~BLE_RF_DIG_TRF_REG1_BRF_PA_CAS_BP_LV_Msk;

    hwp_ble_rfc->TRF_REG2 &= ~BLE_RF_DIG_TRF_REG2_BRF_PA_UNIT_SEL_LV_Msk;
    hwp_ble_rfc->TRF_REG2 &= ~BLE_RF_DIG_TRF_REG2_BRF_PA_MCAP_LV_Msk;

    switch (basic_dbm)
    {
    case 0:
    {
        hwp_ble_rfc->TRF_REG1 |= 0x01 << BLE_RF_DIG_TRF_REG1_BRF_PA_PM_LV_Pos;
        hwp_ble_rfc->TRF_REG1 |= 0x01 << BLE_RF_DIG_TRF_REG1_BRF_PA_CAS_BP_LV_Pos;

        hwp_ble_rfc->TRF_REG2 |= 0x01 << BLE_RF_DIG_TRF_REG2_BRF_PA_UNIT_SEL_LV_Pos;
        hwp_ble_rfc->TRF_REG2 |= 0x0 << BLE_RF_DIG_TRF_REG2_BRF_PA_MCAP_LV_Pos;
    }
    break;
    case 4:
    {
        hwp_ble_rfc->TRF_REG1 |= 0x00 << BLE_RF_DIG_TRF_REG1_BRF_PA_PM_LV_Pos;
        hwp_ble_rfc->TRF_REG1 |= 0x01 << BLE_RF_DIG_TRF_REG1_BRF_PA_CAS_BP_LV_Pos;

        hwp_ble_rfc->TRF_REG2 |= 0x1F << BLE_RF_DIG_TRF_REG2_BRF_PA_UNIT_SEL_LV_Pos;
        hwp_ble_rfc->TRF_REG2 |= 0x01 << BLE_RF_DIG_TRF_REG2_BRF_PA_MCAP_LV_Pos;
    }
    break;
    case 10:
    {
        hwp_ble_rfc->TRF_REG1 |= 0x02 << BLE_RF_DIG_TRF_REG1_BRF_PA_PM_LV_Pos;
        hwp_ble_rfc->TRF_REG1 |= 0x00 << BLE_RF_DIG_TRF_REG1_BRF_PA_CAS_BP_LV_Pos;

        hwp_ble_rfc->TRF_REG2 |= 0x1F << BLE_RF_DIG_TRF_REG2_BRF_PA_UNIT_SEL_LV_Pos;
        hwp_ble_rfc->TRF_REG2 |= 0x01 << BLE_RF_DIG_TRF_REG2_BRF_PA_MCAP_LV_Pos;
    }
    break;
    default:
        HAL_ASSERT(0);

    }

}

void ble_rf_tx_power_set(int8_t txpwr)
{
    uint32_t i;
    for (i = 0; i < BLE_RF_MAX_STEP_COUNT; i++)
        if (tx_pwr_lvl_tb[i].tx_pwr == txpwr)
            break;

    // Config RF
    ble_rf_tx_power_basic_config(tx_pwr_lvl_tb[i].base_dbm);

    // Config PA
    hwp_ble_phy->TX_PA_CFG &= ~BLE_PHY_TX_PA_CFG_PA_CTRL_TARGET_Msk;
    hwp_ble_phy->TX_PA_CFG |= tx_pwr_lvl_tb[i].tx_lvl << BLE_PHY_TX_PA_CFG_PA_CTRL_TARGET_Pos;
}

void ble_rf_fulcal(void)
{
    // To fix the problem that reinit PA cause PA check stucked.

    HAL_RCC_ResetBluetoothRF();
    uint32_t ConstA = 216 * (2048)   ;
    float    ConstB = 1.0 / 96800 ;

    uint8_t i, j, sweep_num ;

    uint8_t acal_cnt;
    uint8_t acal_cnt_fs;
    uint8_t fcal_cnt;
    uint8_t fcal_cnt_fs;

    uint8_t idac0;
    uint8_t idac1;
    uint8_t capcode0;
    uint8_t capcode1;
    uint32_t     error0    = 0xffffffff;
    uint32_t     error1    = 0xffffffff;
    uint32_t     err_tx    = 0xffffffff;
    uint32_t     err_rx_1m = 0xffffffff;
    uint32_t     err_rx_2m = 0xffffffff;

    uint8_t idac_tbl[128];
    uint8_t capcode_tbl[128];
    uint32_t residual_cnt_tbl[128];



    uint32_t idac_tbl_tx[40];
    uint32_t idac_tbl_rx_2m[40];
    uint32_t idac_tbl_rx_1m[40];

    uint32_t capcode_tbl_tx[40];
    uint32_t capcode_tbl_rx_2m[40];
    uint32_t capcode_tbl_rx_1m[40];

    uint32_t residual_cnt;
    const uint32_t residual_cnt_vth = 33864;
    const uint32_t residual_cnt_vtl = 30224;
    uint32_t p0;
    uint32_t p1;


    uint32_t pre_acal_up;
    uint32_t curr_acal_up;
    uint8_t  pre_acal_up_vld;
    uint8_t  seq_acal_jump_cnt ; //cnt for consecutive jump
    uint8_t  seq_acal_ful_cnt ;  //cnt for consecutive all0 all1

    uint32_t pmin, pmax;
    int       p_delta;
    uint32_t  kcal_tbl[40];
    int      kcal_norm;
    uint32_t reg_data;

    //printf("begin fulcal\n");


    //printf("begin LO cal\n");

    //hwp_pmuc->HXT_CR1 &= ~PMUC_HXT_CR1_CBANK_SEL_Msk;
    //hwp_pmuc->HXT_CR1 |= 0x1EA << PMUC_HXT_CR1_CBANK_SEL_Pos;
    //hwp_pmuc->HXT_CR1 |= 0xF << PMUC_HXT_CR1_LDO_VREF_Pos;

    hwp_ble_rfc->INCCAL_REG1 &= ~BLE_RF_DIG_INCCAL_REG1_INCACAL_EN ;
    hwp_ble_rfc->INCCAL_REG1 &= ~BLE_RF_DIG_INCCAL_REG1_INCFCAL_EN ;
    hwp_ble_rfc->MISC_CTRL_REG |= BLE_RF_DIG_MISC_CTRL_REG_IDAC_FORCE_EN  | BLE_RF_DIG_MISC_CTRL_REG_PDX_FORCE_EN;
    hwp_ble_rfc->MISC_CTRL_REG |= BLE_RF_DIG_MISC_CTRL_REG_BRF_EN_RFBG_LV | BLE_RF_DIG_MISC_CTRL_REG_BRF_EN_VDDSW_LV ;

    hwp_ble_rfc->DCO_REG2 &= ~BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_VL_SEL_LV_Msk;
    hwp_ble_rfc->DCO_REG2 |= 0x7 << BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_VL_SEL_LV_Pos;
    //LO full ACAL
    //printf("begin LO acal\n");
    hwp_ble_rfc->DCO_REG1 |= BLE_RF_DIG_DCO_REG1_BRF_VCO_EN_LV ;
    hwp_ble_rfc->DCO_REG2 |= BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_EN_LV;
    hwp_ble_rfc->DCO_REG2 |= BLE_RF_DIG_DCO_REG2_BRF_VCO_FKCAL_EN_LV;
    //VCO_ACAL_SEL ??
    hwp_ble_rfc->LPF_REG  |= BLE_RF_DIG_LPF_REG_BRF_LO_OPEN_LV;
    hwp_ble_phy->TX_HFP_CFG &= ~(BLE_PHY_TX_HFP_CFG_HFP_FCW_Msk);
    hwp_ble_phy->TX_HFP_CFG |= (0x07 << BLE_PHY_TX_HFP_CFG_HFP_FCW_Pos);

    hwp_ble_rfc->DCO_REG1 |= BLE_RF_DIG_DCO_REG1_BRF_EN_2M_MOD_LV;

    hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Msk;
    hwp_ble_rfc->DCO_REG1 |= (0x40) << BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Pos;

    acal_cnt    = 0x40;
    acal_cnt_fs = 0x40;

    //wait 4us
    _HAL_Delay_us(4);
    //acal binary search
    for (i = 1; i < 7; i++)
    {
        //printf("begin LO acal binary search\n");
        //
        //printf("pre acal_cnt = %d\n", acal_cnt);
        //printf("step = %d\n", acal_cnt_fs>>i);
        if (!(hwp_ble_rfc->DCO_REG2 & BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_INCAL_LV_Msk))
            break;
        else if (!(hwp_ble_rfc->DCO_REG2 & BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_UP_LV_Msk))
            acal_cnt = acal_cnt - (acal_cnt_fs >> i) ;
        else  if (hwp_ble_rfc->DCO_REG2 & BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_UP_LV_Msk)
            acal_cnt = acal_cnt + (acal_cnt_fs >> i)  ;
        hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Msk;
        hwp_ble_rfc->DCO_REG1 |= (acal_cnt) << BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Pos;
        //wait 1us
        //printf("acal_cnt = %d\n", acal_cnt);

    }
    hwp_ble_rfc->DCO_REG2 &= ~BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_EN_LV;

    //LO full fcal
    //printf("begin LO fcal\n");
    fcal_cnt    = 0x80;
    fcal_cnt_fs = 0x80;
    hwp_ble_rfc->FBDV_REG1 |= BLE_RF_DIG_FBDV_REG1_BRF_FBDV_EN_LV ;
    hwp_ble_rfc->DCO_REG2 |= BLE_RF_DIG_DCO_REG2_BRF_VCO_FKCAL_EN_LV;
    hwp_ble_rfc->FBDV_REG2 &= ~BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_DIVN_LV ;
    hwp_ble_rfc->FBDV_REG2 |= 7680 << BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_DIVN_LV_Pos;
    //set lfp_fcw
    hwp_ble_phy->TX_LFP_CFG &= ~(BLE_PHY_TX_LFP_CFG_LFP_FCW_Msk);
    hwp_ble_phy->TX_LFP_CFG |= (0x08 << BLE_PHY_TX_LFP_CFG_LFP_FCW_Pos);
    hwp_ble_phy->TX_LFP_CFG   &= (~BLE_PHY_TX_LFP_CFG_LFP_FCW_SEL);

    hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_PDX_LV_Msk;
    hwp_ble_rfc->DCO_REG1 |= (0x80) << BLE_RF_DIG_DCO_REG1_BRF_VCO_PDX_LV_Pos;
    hwp_ble_phy->TX_HFP_CFG &= ~(BLE_PHY_TX_HFP_CFG_HFP_FCW_Msk);
    hwp_ble_phy->TX_HFP_CFG |= (0x07 << BLE_PHY_TX_HFP_CFG_HFP_FCW_Pos);
    hwp_ble_phy->TX_HFP_CFG &=  ~BLE_PHY_TX_HFP_CFG_HFP_FCW_SEL;
    hwp_ble_rfc->DCO_REG1 |= BLE_RF_DIG_DCO_REG1_BRF_EN_2M_MOD_LV;

    hwp_ble_rfc->FBDV_REG1 |= BLE_RF_DIG_FBDV_REG1_BRF_FBDV_RSTB_LV ;
    hwp_ble_rfc->FBDV_REG1 &= ~BLE_RF_DIG_FBDV_REG1_BRF_FBDV_RSTB_LV ;

    hwp_ble_rfc->FBDV_REG1 &=  ~BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV_Msk;
    hwp_ble_rfc->FBDV_REG1 |=  BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV;

    hwp_ble_rfc->MISC_CTRL_REG |= BLE_RF_DIG_MISC_CTRL_REG_XTAL_REF_EN | BLE_RF_DIG_MISC_CTRL_REG_XTAL_REF_EN_FRC_EN ;
    hwp_ble_rfc->PFDCP_REG |= BLE_RF_DIG_PFDCP_REG_BRF_PFDCP_EN_LV ;


    hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Msk;
    hwp_ble_rfc->DCO_REG1 |= (0x40) << BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Pos;

    //pdx binary search
    //should store the cnt value of last pdx, so loop 8 times
    for (i = 1; i < 9; i++)
    {
        //printf("begin LO fcal binary search\n");
        //--------full acal in full fcal --------
        //{{{
        acal_cnt    = 0x40;
        acal_cnt_fs = 0x40;

        hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Msk;
        hwp_ble_rfc->DCO_REG1 |= (acal_cnt) << BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Pos;
        hwp_ble_rfc->DCO_REG2 |= BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_EN_LV;
        //wait 4us

        //acal binary search
        for (j = 1; j < 7; j++)
        {
            //
            if (!(hwp_ble_rfc->DCO_REG2 & BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_INCAL_LV_Msk))
                break;
            else if (!(hwp_ble_rfc->DCO_REG2 & BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_UP_LV_Msk))
                acal_cnt = acal_cnt - (acal_cnt_fs >> j) ;
            else  if (hwp_ble_rfc->DCO_REG2 & BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_UP_LV_Msk)
                acal_cnt = acal_cnt + (acal_cnt_fs >> j) ;
            hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Msk;
            hwp_ble_rfc->DCO_REG1 |= (acal_cnt) << BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Pos;
            //wait 1us

        }
        hwp_ble_rfc->DCO_REG2 &= ~BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_EN_LV;
        //}}}

        hwp_ble_rfc->FBDV_REG1 |= 1 << BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_EN_LV_Pos;



        while (!(hwp_ble_rfc->FBDV_REG1 &  BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_RDY_LV));
        residual_cnt  =  hwp_ble_rfc->FBDV_REG2 & BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Msk ;
        residual_cnt  = residual_cnt >> BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;

        //printf( "residual_cnt = %d,cnt_vth = %d\n",residual_cnt,residual_cnt_vth );
        if (residual_cnt > residual_cnt_vth)
        {
            idac1    = acal_cnt;
            p1       = residual_cnt ;
            error1   = residual_cnt - residual_cnt_vth ;
            capcode1 = fcal_cnt;
            fcal_cnt = fcal_cnt + (fcal_cnt_fs >> i) ;
        }
        else if (residual_cnt <= residual_cnt_vth)
        {
            idac0    = acal_cnt;
            p0       = residual_cnt ;
            error0   = residual_cnt_vth - residual_cnt ;
            capcode0 = fcal_cnt;
            fcal_cnt = fcal_cnt - (fcal_cnt_fs >> i) ;
        }
        //printf( "fcal bin fcal_cnt = %x,acal_cnt = %x\n",fcal_cnt,acal_cnt );
        hwp_ble_rfc->FBDV_REG1 &=  ~BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;
        hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_PDX_LV_Msk;
        hwp_ble_rfc->DCO_REG1 |= (fcal_cnt) << BLE_RF_DIG_DCO_REG1_BRF_VCO_PDX_LV_Pos;
    }

    //printf( "sweep start idac0 = %x,capcode0 = %x\n",idac0,capcode0 );
    //printf( "sweep start idac1 = %x,capcode1 = %x\n",idac1,capcode1 );
    //printf( "sweep start error0 = %x,error1 = %x\n",error0,error1 );
    if (error0 < error1)
    {
        idac_tbl[0]         = idac0 ;
        capcode_tbl[0]      = capcode0;
        residual_cnt_tbl[0] = p0;
    }
    else
    {
        idac_tbl[0]         = idac1 ;
        capcode_tbl[0]      = capcode1;
        residual_cnt_tbl[0] = p1;
    }

    hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_PDX_LV_Msk;
    hwp_ble_rfc->DCO_REG1 |= (fcal_cnt) << BLE_RF_DIG_DCO_REG1_BRF_VCO_PDX_LV_Pos;
    hwp_ble_rfc->FBDV_REG1 &= ~BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;

    fcal_cnt = capcode_tbl[0] ;
    acal_cnt = idac_tbl[0] ;
    //printf( "sweep start fcal_cnt = %x,acal_cnt = %x\n",fcal_cnt,acal_cnt );
    //sweep pdx until 4.8G
    i = 0;
    hwp_ble_rfc->RSVD_REG2 = 0 ;

    do
    {
        hwp_ble_rfc->RSVD_REG2 +=  1 ;//DEBUG
        hwp_ble_rfc->RSVD_REG1  =  0 ;//DEBUG
        i                     +=  1 ;
        fcal_cnt              +=  1 ;
        seq_acal_jump_cnt      =  0 ;
        seq_acal_ful_cnt       =  0 ;
        pre_acal_up_vld        =  0 ;
        hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_PDX_LV_Msk;
        hwp_ble_rfc->DCO_REG1 |= (fcal_cnt) << BLE_RF_DIG_DCO_REG1_BRF_VCO_PDX_LV_Pos;
        //seq acal {{{
        hwp_ble_rfc->DCO_REG2 |= BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_EN_LV;
        //VCO_ACAL_SEL ??
        hwp_ble_rfc->LPF_REG  |= BLE_RF_DIG_LPF_REG_BRF_LO_OPEN_LV;
        while ((seq_acal_jump_cnt < 4) & (seq_acal_ful_cnt < 2))
        {
            hwp_ble_rfc->RSVD_REG1 +=  1 ;//DEBUG
            hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Msk;
            hwp_ble_rfc->DCO_REG1 |= (acal_cnt) << BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Pos;
            //wait for 4us
            _HAL_Delay_us(4);
            //for(int j=0;j<100;j++)
            //printf( "wait idac settling\n" );
            //printf( "wait idac settling\n" );
            //printf( "wait idac settling\n" );
            //printf( "wait idac settling\n" );
            //printf( "wait idac settling\n" );
            //printf( "wait idac settling\n" );
            if (!(hwp_ble_rfc->DCO_REG2 & BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_INCAL_LV_Msk))
                break;
            curr_acal_up = hwp_ble_rfc->DCO_REG2 & BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_UP_LV_Msk;
            if (!(curr_acal_up))
            {
                if (acal_cnt > 0)
                {
                    acal_cnt = acal_cnt - 1 ;
                    seq_acal_ful_cnt = 0;
                }
                else
                {
                    seq_acal_ful_cnt += 1;
                    acal_cnt = 0 ;
                }
            }
            else if (curr_acal_up)
            {
                if (acal_cnt < 0x3f)
                {
                    acal_cnt = acal_cnt + 1 ;
                    seq_acal_ful_cnt = 0;
                }
                else
                {
                    seq_acal_ful_cnt += 1;
                    acal_cnt = 0x3f ;
                }
            }

            if (pre_acal_up_vld)
            {
                if (pre_acal_up == curr_acal_up)
                    seq_acal_jump_cnt = 0;
                else if (pre_acal_up != curr_acal_up)
                    seq_acal_jump_cnt += 1;
            }
            pre_acal_up     = curr_acal_up ;
            pre_acal_up_vld = 1;
        }
        hwp_ble_rfc->DCO_REG2 &= ~BLE_RF_DIG_DCO_REG2_BRF_VCO_ACAL_EN_LV;
        ///}}}

        hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Msk;
        hwp_ble_rfc->DCO_REG1 |= (acal_cnt) << BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Pos;
        //wait for 4us
        _HAL_Delay_us(4);
        //for(int j=0;j<100;j++)
        //printf( "wait idac settling\n" );
        //printf( "wait idac settling\n" );
        //printf( "wait idac settling\n" );
        //printf( "wait idac settling\n" );
        //printf( "wait idac settling\n" );
        //printf( "wait idac settling\n" );
        hwp_ble_rfc->FBDV_REG1 |= 1 << BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_EN_LV_Pos;

        hwp_ble_rfc->FBDV_REG1 &=  ~BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV_Msk;
        hwp_ble_rfc->FBDV_REG1 |=  BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV;

        _HAL_Delay_us(200);
        //for( j=0;j<100;j++)
        //printf( "wait idac settling\n" );

        while (!(hwp_ble_rfc->FBDV_REG1 & BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_RDY_LV));

        residual_cnt  = hwp_ble_rfc->FBDV_REG2 & BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Msk ;
        //residual_cnt  = residual_cnt >> BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;
        //printf( "residual_cnt = %d, residual_cnt_vtl=%d\n", residual_cnt, residual_cnt_vtl );

        if (residual_cnt <= residual_cnt_vtl)
            break;

        idac_tbl[i]         = acal_cnt ;
        capcode_tbl[i]      = fcal_cnt ;
        residual_cnt_tbl[i] = residual_cnt ;
    }
    while (residual_cnt > residual_cnt_vtl) ;

    hwp_ble_rfc->DCO_REG2 &= ~BLE_RF_DIG_DCO_REG2_BRF_VCO_FKCAL_EN_LV;

    //search result for each channel
    sweep_num   = i ;
    hwp_ble_rfc->RSVD_REG2 =  10 ;//DEBUG
    for (j = 0; j < 40; j++)
    {
        hwp_ble_rfc->RSVD_REG2 +=  1 ;//DEBUG
        hwp_ble_rfc->RSVD_REG1 =  100 ;//DEBUG
        err_tx    = 0 ;
        err_rx_1m = 0 ;
        err_rx_2m = 0 ;
        for (i = 0; i < sweep_num; i++)
        {
            hwp_ble_rfc->RSVD_REG1 +=  1 ;//DEBUG
            if (i == 0)
            {
                if (ref_residual_cnt_tbl_tx[j] > residual_cnt_tbl[i])
                {
                    err_tx     =  ref_residual_cnt_tbl_tx[j]    - residual_cnt_tbl[i] ;
                    err_rx_1m  =  ref_residual_cnt_tbl_rx_1m[j] - residual_cnt_tbl[i] ;
                    err_rx_2m  =  ref_residual_cnt_tbl_rx_2m[j] - residual_cnt_tbl[i] ;
                }
                else
                {
                    err_tx     =  residual_cnt_tbl[i] - ref_residual_cnt_tbl_tx[j]    ;
                    err_rx_1m  =  residual_cnt_tbl[i] - ref_residual_cnt_tbl_rx_1m[j] ;
                    err_rx_2m  =  residual_cnt_tbl[i] - ref_residual_cnt_tbl_rx_2m[j] ;
                }
                idac_tbl_tx[j]       =  idac_tbl[i];
                idac_tbl_rx_1m[j]    =  idac_tbl[i];
                idac_tbl_rx_2m[j]    =  idac_tbl[i];
                capcode_tbl_tx[j]    =  capcode_tbl[i];
                capcode_tbl_rx_1m[j] =  capcode_tbl[i];
                capcode_tbl_rx_2m[j] =  capcode_tbl[i];
            }
            else
            {
                if (ref_residual_cnt_tbl_tx[j] > residual_cnt_tbl[i])
                {
                    error0     =  ref_residual_cnt_tbl_tx[j]    - residual_cnt_tbl[i] ;
                    if (error0 < err_tx)
                    {
                        err_tx = error0;
                        idac_tbl_tx[j]       =  idac_tbl[i];
                        capcode_tbl_tx[j]    =  capcode_tbl[i];
                    }
                    error0     =  ref_residual_cnt_tbl_rx_1m[j] - residual_cnt_tbl[i] ;
                    if (error0 < err_rx_1m)
                    {
                        err_rx_1m            = error0;
                        idac_tbl_rx_1m[j]    =  idac_tbl[i];
                        capcode_tbl_rx_1m[j] =  capcode_tbl[i];
                    }
                    error0     =  ref_residual_cnt_tbl_rx_2m[j] - residual_cnt_tbl[i] ;
                    if (error0 < err_rx_2m)
                    {
                        err_rx_2m            = error0;
                        idac_tbl_rx_2m[j]    =  idac_tbl[i];
                        capcode_tbl_rx_2m[j] =  capcode_tbl[i];
                    }
                }
                else
                {
                    error0     = residual_cnt_tbl[i] - ref_residual_cnt_tbl_tx[j] ;
                    if (error0 < err_tx)
                    {
                        err_tx = error0;
                        idac_tbl_tx[j]       =  idac_tbl[i];
                        capcode_tbl_tx[j]    =  capcode_tbl[i];
                    }
                    error0     =  ref_residual_cnt_tbl_rx_1m[j] - residual_cnt_tbl[i] ;
                    if (error0 < err_rx_1m)
                    {
                        err_rx_1m            = error0;
                        idac_tbl_rx_1m[j]    =  idac_tbl[i];
                        capcode_tbl_rx_1m[j] =  capcode_tbl[i];
                    }
                    error0     =  ref_residual_cnt_tbl_rx_2m[j] - residual_cnt_tbl[i] ;
                    if (error0 < err_rx_2m)
                    {
                        err_rx_2m            = error0;
                        idac_tbl_rx_2m[j]    =  idac_tbl[i];
                        capcode_tbl_rx_2m[j] =  capcode_tbl[i];
                    }
                }

            }
        }
    }

    //------------kcal -----------
    //{{{
    hwp_ble_rfc->RSVD_REG2 =  100 ;//DEBUG
    //printf("begin LO 0-19th ch kcal\n");
    hwp_ble_rfc->DCO_REG2    |= BLE_RF_DIG_DCO_REG2_BRF_VCO_FKCAL_EN_LV;
    hwp_ble_rfc->FBDV_REG2   &= ~BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_DIVN_LV ;
    hwp_ble_rfc->FBDV_REG2   |= 17280 << BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_DIVN_LV_Pos;
    hwp_ble_phy->TX_LFP_CFG &= ~(BLE_PHY_TX_LFP_CFG_LFP_FCW_Msk);
    hwp_ble_phy->TX_LFP_CFG |= (0x08 << BLE_PHY_TX_LFP_CFG_LFP_FCW_Pos);
    hwp_ble_phy->TX_LFP_CFG &=  ~BLE_PHY_TX_LFP_CFG_LFP_FCW_SEL;
    //TODO replace with 10th chnl pdx
    hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_PDX_LV_Msk;
    hwp_ble_rfc->DCO_REG1 |= (capcode_tbl_tx[9]) << BLE_RF_DIG_DCO_REG1_BRF_VCO_PDX_LV_Pos;
    hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Msk;
    hwp_ble_rfc->DCO_REG1 |= (idac_tbl_tx[9]) << BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Pos;
    hwp_ble_phy->TX_HFP_CFG &= ~(BLE_PHY_TX_HFP_CFG_HFP_FCW_Msk);
    hwp_ble_phy->TX_HFP_CFG |= (0x00 << BLE_PHY_TX_HFP_CFG_HFP_FCW_Pos);
    hwp_ble_phy->TX_HFP_CFG &=  ~BLE_PHY_TX_HFP_CFG_HFP_FCW_SEL;

    //wait 20us
    _HAL_Delay_us(20);
    hwp_ble_rfc->FBDV_REG1 &= ~BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;
    hwp_ble_rfc->FBDV_REG1 |= 1 << BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_EN_LV_Pos;

    while (!(hwp_ble_rfc->FBDV_REG1 &  BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_RDY_LV));
    pmin = hwp_ble_rfc->FBDV_REG2 & BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Msk ;
    //pmin = pmin >> BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;
    hwp_ble_rfc->FBDV_REG1 &= ~BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;


    hwp_ble_phy->TX_HFP_CFG &= ~(BLE_PHY_TX_HFP_CFG_HFP_FCW_Msk);
    hwp_ble_phy->TX_HFP_CFG |= (0x3f << BLE_PHY_TX_HFP_CFG_HFP_FCW_Pos);
    //wait 20us
    _HAL_Delay_us(20);
    hwp_ble_rfc->FBDV_REG1 |= 1 << BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_EN_LV_Pos;

    while (!(hwp_ble_rfc->FBDV_REG1 &  BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_RDY_LV));
    pmax = hwp_ble_rfc->FBDV_REG2 & BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Msk ;
    pmax = pmax >> BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;

    hwp_ble_rfc->FBDV_REG1 &= ~BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;
    hwp_ble_rfc->DCO_REG2  &= ~BLE_RF_DIG_DCO_REG2_BRF_VCO_FKCAL_EN_LV;
    //}}}

    kcal_norm = ConstA / (pmax - pmin) ;
    //printf("kcal_norm = %d,pmin = %d, pmax=%d\n", kcal_norm,pmin,pmax  );

    for (i = 0; i < 20; i++)
    {
        hwp_ble_rfc->RSVD_REG2 +=  1 ;//DEBUG
        p_delta = (ref_residual_cnt_tbl_tx[i] - ref_residual_cnt_tbl_tx[9]) ;
        kcal_tbl[i] = kcal_norm * (1.0 - 3 * p_delta * ConstB);
        //printf( "kcal[%d] = %x\n",i,kcal_tbl[i] );//DEBUG
    }

    ConstA = 216 * (2048) ;
    ConstB = 1.0 / 98400;
    //printf("begin LO 20-39th ch kcal\n");
    hwp_ble_rfc->RSVD_REG2 =  1000 ;//DEBUG
    hwp_ble_rfc->DCO_REG2    |= BLE_RF_DIG_DCO_REG2_BRF_VCO_FKCAL_EN_LV;
    hwp_ble_rfc->FBDV_REG2   &= ~BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_DIVN_LV ;
    hwp_ble_rfc->FBDV_REG2   |= 17280 << BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_DIVN_LV_Pos;
    hwp_ble_phy->TX_LFP_CFG &= ~(BLE_PHY_TX_LFP_CFG_LFP_FCW_Msk);
    hwp_ble_phy->TX_LFP_CFG |= (0x08 << BLE_PHY_TX_LFP_CFG_LFP_FCW_Pos);
    hwp_ble_phy->TX_LFP_CFG &=  ~BLE_PHY_TX_LFP_CFG_LFP_FCW_SEL;
    //TODO replace with 10th chnl pdx
    hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_PDX_LV_Msk;
    hwp_ble_rfc->DCO_REG1 |= (capcode_tbl_tx[29]) << BLE_RF_DIG_DCO_REG1_BRF_VCO_PDX_LV_Pos;
    hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Msk;
    hwp_ble_rfc->DCO_REG1 |= (idac_tbl_tx[29]) << BLE_RF_DIG_DCO_REG1_BRF_VCO_IDAC_LV_Pos;

    hwp_ble_phy->TX_HFP_CFG &= ~(BLE_PHY_TX_HFP_CFG_HFP_FCW_Msk);
    hwp_ble_phy->TX_HFP_CFG |= (0x00 << BLE_PHY_TX_HFP_CFG_HFP_FCW_Pos);
    hwp_ble_phy->TX_HFP_CFG &=  ~BLE_PHY_TX_HFP_CFG_HFP_FCW_SEL;
    //wait 20us
    _HAL_Delay_us(20);
    hwp_ble_rfc->FBDV_REG1 |= 1 << BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_EN_LV_Pos;

    while (!(hwp_ble_rfc->FBDV_REG1 &  BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_RDY_LV));
    pmin = hwp_ble_rfc->FBDV_REG2 & BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Msk ;
    pmin = pmin >> BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;
    hwp_ble_rfc->FBDV_REG1 &= ~BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;


    hwp_ble_phy->TX_HFP_CFG &= ~(BLE_PHY_TX_HFP_CFG_HFP_FCW_Msk);
    hwp_ble_phy->TX_HFP_CFG |= (0x3F << BLE_PHY_TX_HFP_CFG_HFP_FCW_Pos);
    //wait 20us
    _HAL_Delay_us(20);
    hwp_ble_rfc->FBDV_REG1 |= 1 << BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_EN_LV_Pos;

    while (!(hwp_ble_rfc->FBDV_REG1 &  BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_RDY_LV));
    pmax = hwp_ble_rfc->FBDV_REG2 & BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Msk ;
    pmax = pmax >> BLE_RF_DIG_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;

    hwp_ble_rfc->FBDV_REG1 &= ~BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;
    //}}}

    kcal_norm = ConstA / (pmax - pmin) ;
    //printf("kcal_norm = %d,pmin = %d, pmax=%d\n", kcal_norm,pmin,pmax  );

    for (i = 20; i < 40; i++)
    {
        hwp_ble_rfc->RSVD_REG2 +=  1 ;//DEBUG
        p_delta = (ref_residual_cnt_tbl_tx[i] - ref_residual_cnt_tbl_tx[29]) ;
        kcal_tbl[i] = kcal_norm * (1 - 3 * p_delta * ConstB);
        //printf( "kcal[%d] = %x\n",i,kcal_tbl[i] );//DEBUG
    }


    hwp_ble_rfc->PFDCP_REG &= ~BLE_RF_DIG_PFDCP_REG_BRF_PFDCP_EN_LV ;


    //write to register
    uint32_t reg_addr = BLE_RFC_BASE ;
    reg_data = 0;
    for (i = 0; i < 40; i++)
    {
        //store tx cal result
        reg_data = (kcal_tbl[i] << BLE_RF_DIG_FULCAL_REG0_KCAL_CHNL0_Pos) + (idac_tbl_tx[i] << BLE_RF_DIG_FULCAL_REG0_ACAL_CHNL0_Pos) + (capcode_tbl_tx[i] << BLE_RF_DIG_FULCAL_REG0_FCAL_CHNL0_Pos) ;
        write_memory(reg_addr + 0x54 + i * 4, reg_data);
        //store rx cal result
        reg_data  = (capcode_tbl_rx_2m[i] << BLE_RF_DIG_FULCAL_REG_RX0_RX2M_FCAL_CHNL0_Pos) + (idac_tbl_rx_2m[i] << BLE_RF_DIG_FULCAL_REG_RX0_RX2M_ACAL_CHNL0_Pos) ;
        reg_data += (capcode_tbl_rx_1m[i] << BLE_RF_DIG_FULCAL_REG_RX0_RX1M_FCAL_CHNL0_Pos) + (idac_tbl_rx_1m[i] << BLE_RF_DIG_FULCAL_REG_RX0_RX1M_ACAL_CHNL0_Pos);
        write_memory(reg_addr + 0x2A0 + i * 4, reg_data);
    }



#if 1
    //PACAL {{{
    uint8_t setbc_rslt;
    uint8_t setsgn_rslt;
    //LO lock setting

    hwp_ble_rfc->TRF_REG1 &= ~BLE_RF_DIG_TRF_REG1_BRF_TRF_LDO_VREF_SEL_LV_Msk;
    hwp_ble_rfc->TRF_REG1 |= 0x0E << BLE_RF_DIG_TRF_REG1_BRF_TRF_LDO_VREF_SEL_LV_Pos;

    hwp_ble_rfc->MISC_CTRL_REG |= BLE_RF_DIG_MISC_CTRL_REG_BRF_LODIST_TX_EN_LV;
    hwp_ble_rfc->TRF_REG2 |= BLE_RF_DIG_TRF_REG2_BRF_PA_UNIT_SEL_LV ; //for 4dbm power mode
    hwp_ble_rfc->TRF_REG1 |= BLE_RF_DIG_TRF_REG1_BRF_PA_BUF_PU_LV | BLE_RF_DIG_TRF_REG1_BRF_TRF_SIG_EN_LV;
    hwp_ble_rfc->TRF_REG1 |= BLE_RF_DIG_TRF_REG1_BRF_PA_BCSEL_LV;
    hwp_ble_rfc->TRF_REG2 |= BLE_RF_DIG_TRF_REG2_BRF_PA_BUFLOAD_SEL_LV;
    hwp_ble_rfc->PACAL_REG |= BLE_RF_DIG_PACAL_REG_PA_RSTB_FRC_EN | BLE_RF_DIG_PACAL_REG_PACAL_CLK_EN;

    _HAL_Delay_us(20);
    hwp_ble_rfc->TRF_REG1  &= ~BLE_RF_DIG_TRF_REG1_BRF_PA_RSTN_LV ;
    _HAL_Delay_us(2);
    hwp_ble_rfc->TRF_REG1  |= BLE_RF_DIG_TRF_REG1_BRF_PA_RSTN_LV ;
    //hwp_ble_rfc->PACAL_REG &= ~BLE_RF_DIG_PACAL_REG_PA_RSTB_FRC_EN;
    _HAL_Delay_us(20);

    hwp_ble_rfc->PACAL_REG &= (~BLE_RF_DIG_PACAL_REG_PACAL_START);
    hwp_ble_rfc->PACAL_REG |= (BLE_RF_DIG_PACAL_REG_PACAL_START);

    while (!(hwp_ble_rfc->PACAL_REG & BLE_RF_DIG_PACAL_REG_PACAL_DONE));
    hwp_ble_rfc->TRF_REG1 &= ~BLE_RF_DIG_TRF_REG1_BRF_PA_SETBC_LV;
    hwp_ble_rfc->TRF_REG1 &= ~BLE_RF_DIG_TRF_REG1_BRF_PA_SETSGN_LV;
    setbc_rslt  = (hwp_ble_rfc->PACAL_REG & BLE_RF_DIG_PACAL_REG_BC_CAL_RSLT_Msk) >> BLE_RF_DIG_PACAL_REG_BC_CAL_RSLT_Pos;
    setsgn_rslt = (hwp_ble_rfc->PACAL_REG & BLE_RF_DIG_PACAL_REG_SGN_CAL_RSLT_Msk) >> BLE_RF_DIG_PACAL_REG_SGN_CAL_RSLT_Pos;
    hwp_ble_rfc->TRF_REG1 |= setbc_rslt << BLE_RF_DIG_TRF_REG1_BRF_PA_SETBC_LV_Pos;
    hwp_ble_rfc->TRF_REG1 |= setsgn_rslt << BLE_RF_DIG_TRF_REG1_BRF_PA_SETSGN_LV_Pos;

    //hwp_ble_rfc->TRF_REG2 &= ~BLE_RF_DIG_TRF_REG2_BRF_PA_BUFLOAD_SEL_LV_Msk;
    hwp_ble_rfc->PACAL_REG &= (~BLE_RF_DIG_PACAL_REG_PACAL_START);
    hwp_ble_rfc->TRF_REG1 &= ~BLE_RF_DIG_TRF_REG1_BRF_PA_BCSEL_LV ;
    hwp_ble_rfc->TRF_REG1 &= ~BLE_RF_DIG_TRF_REG1_BRF_PA_BUF_PU_LV;
    hwp_ble_rfc->PACAL_REG  &= ~BLE_RF_DIG_PACAL_REG_PACAL_CLK_EN;
    hwp_ble_rfc->MISC_CTRL_REG &= ~BLE_RF_DIG_MISC_CTRL_REG_BRF_LODIST_TX_EN_LV;
    //}}}
#endif

    //after RF calibration
    hwp_ble_rfc->MISC_CTRL_REG &= ~BLE_RF_DIG_MISC_CTRL_REG_XTAL_REF_EN_FRC_EN ;
    hwp_ble_rfc->INCCAL_REG1 |= BLE_RF_DIG_INCCAL_REG1_INCACAL_EN ;
    hwp_ble_rfc->INCCAL_REG1 |= BLE_RF_DIG_INCCAL_REG1_INCFCAL_EN ;
    hwp_ble_rfc->LPF_REG     &= ~BLE_RF_DIG_LPF_REG_BRF_LO_OPEN_LV_Msk;
    hwp_ble_rfc->DCO_REG2    &= ~BLE_RF_DIG_DCO_REG2_BRF_VCO_FKCAL_EN_LV;
    hwp_ble_phy->TX_LFP_CFG |=  BLE_PHY_TX_LFP_CFG_LFP_FCW_SEL;
    hwp_ble_phy->TX_HFP_CFG |=  BLE_PHY_TX_HFP_CFG_HFP_FCW_SEL;
    hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_EN_2M_MOD_LV;
    hwp_ble_rfc->MISC_CTRL_REG &= ~BLE_RF_DIG_MISC_CTRL_REG_IDAC_FORCE_EN ;
    hwp_ble_rfc->MISC_CTRL_REG &= ~BLE_RF_DIG_MISC_CTRL_REG_PDX_FORCE_EN;
    hwp_ble_rfc->TRF_REG1 |= BLE_RF_DIG_TRF_REG1_BRF_PA_CAS_BP_LV;
    hwp_ble_rfc->TRF_REG2 |= BLE_RF_DIG_TRF_REG2_BRF_PA_MCAP_LV;
    hwp_ble_rfc->FBDV_REG1 |=  BLE_RF_DIG_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV_Msk;
    hwp_ble_rfc->ADC_REG |= BLE_RF_DIG_ADC_REG_BRF_RSTB_ADC_LV ;
    hwp_ble_rfc->DCO_REG1 &= ~BLE_RF_DIG_DCO_REG1_BRF_VCO_EN_LV ;
    hwp_ble_rfc->FBDV_REG1 &= ~BLE_RF_DIG_FBDV_REG1_BRF_FBDV_EN_LV ;

    uint32_t rc_capcode;
    hwp_ble_rfc->RBB_REG1 |= BLE_RF_DIG_RBB_REG1_BRF_EN_LDO_RBB_LV;
    hwp_ble_rfc->RBB_REG5 |= BLE_RF_DIG_RBB_REG5_BRF_EN_IARRAY_LV;
    hwp_ble_rfc->RBB_REG5 |= BLE_RF_DIG_RBB_REG5_BRF_RCCAL_SELXO_LV;
    hwp_ble_rfc->RBB_REG5 &= ~BLE_RF_DIG_RBB_REG5_BRF_RCCAL_MANCAP_LV;
    hwp_ble_rfc->RCROSCAL_REG &= ~BLE_RF_DIG_RCROSCAL_REG_RC_CAPCODE_OFFSET_Msk;
    hwp_ble_rfc->RBB_REG5 |= (BLE_RF_DIG_RBB_REG5_BRF_EN_RCCAL_LV);
    hwp_ble_rfc->RBB_REG5 &= ~(BLE_RF_DIG_RBB_REG5_BRF_RSTB_RCCAL_LV);
    hwp_ble_rfc->RBB_REG5 |= (BLE_RF_DIG_RBB_REG5_BRF_RSTB_RCCAL_LV);
    hwp_ble_rfc->RCROSCAL_REG |= BLE_RF_DIG_RCROSCAL_REG_RCCAL_START;
    while (!(hwp_ble_rfc->RCROSCAL_REG & BLE_RF_DIG_RCROSCAL_REG_RCCAL_DONE));

    rc_capcode = hwp_ble_rfc->RCROSCAL_REG & BLE_RF_DIG_RCROSCAL_REG_RC_CAPCODE_Msk;
    rc_capcode >>= BLE_RF_DIG_RCROSCAL_REG_RC_CAPCODE_Pos;
    hwp_ble_rfc->RBB_REG5 &= ~BLE_RF_DIG_RBB_REG5_BRF_CBPF_CAPMAN_LV_Msk;
    hwp_ble_rfc->RBB_REG5 |= (rc_capcode <<    BLE_RF_DIG_RBB_REG5_BRF_CBPF_CAPMAN_LV_Pos);
    hwp_ble_rfc->RBB_REG5 |= BLE_RF_DIG_RBB_REG5_BRF_RCCAL_MANCAP_LV;
    hwp_ble_rfc->RCROSCAL_REG &= ~BLE_RF_DIG_RCROSCAL_REG_RCCAL_START;
    hwp_ble_rfc->RBB_REG5 &= ~(BLE_RF_DIG_RBB_REG5_BRF_EN_IARRAY_LV);
    hwp_ble_rfc->RBB_REG1 &= ~BLE_RF_DIG_RBB_REG1_BRF_EN_LDO_RBB_LV;

    // IF = 1.995M
    hwp_ble_rfc->RBB_REG2 &= ~BLE_RF_DIG_RBB_REG2_BRF_CBPF_FC_LV;
    hwp_ble_rfc->RBB_REG2 |= BLE_RF_DIG_RBB_REG2_BRF_CBPF_FC_LV;

    //after calibration
    hwp_ble_rfc->MISC_CTRL_REG &= ~BLE_RF_DIG_MISC_CTRL_REG_BRF_EN_RFBG_LV ;
    hwp_ble_rfc->MISC_CTRL_REG &= ~BLE_RF_DIG_MISC_CTRL_REG_BRF_EN_VDDSW_LV ;

    hwp_ble_phy->TX_HFP_CFG &= ~(BLE_PHY_TX_HFP_CFG_HFP_DELAY_SEL_Msk);
    hwp_ble_phy->TX_HFP_CFG |= (0x01 << BLE_PHY_TX_HFP_CFG_HFP_DELAY_SEL_Pos);

    // Set TX power
    ble_rf_tx_power_set(bt_rf_get_init_tx_pwr());

    // Handle off to phy to control power
    //hwp_ble_phy->TX_CTRL &= ~BLE_PHY_TX_CTRL_MAC_PWR_CTRL_EN;

    // temp try, adjust TX GAUSS gain, default value is 0xFF
    hwp_ble_phy->TX_GAUSSFLT_CFG &= ~BLE_PHY_TX_GAUSSFLT_CFG_GAUSS_GAIN_1;
    hwp_ble_phy->TX_GAUSSFLT_CFG |= 0x0FF << BLE_PHY_TX_GAUSSFLT_CFG_GAUSS_GAIN_1_Pos;


    // Configuration for RX
    hwp_ble_rfc->ADC_REG |= BLE_RF_DIG_ADC_REG_BRF_RSTB_ADC_LV;
    //hwp_ble_rfc->ADC_REG |= BLE_RF_DIG_ADC_REG_BRF_EN_ADC_Q_LV; // For IF = 1.995M
    //hwp_ble_rfc->ADC_REG |= BLE_RF_DIG_ADC_REG_BRF_EN_ADC_I_LV;

    //hwp_ble_phy->AGC_CTRL &= ~BLE_PHY_AGC_CTRL_AGC_ENABLE;
    //hwp_ble_phy->RX_CTRL1 |= BLE_PHY_RX_CTRL1_ADC_Q_EN;  // IF = 1.995M

    // LDO, improve RX sensitive
    hwp_ble_rfc->RRF_REG &= ~BLE_RF_DIG_RRF_REG_BRF_RRF_LDO_VREF_SEL_LV;
    hwp_ble_rfc->RRF_REG |= 0x0D << BLE_RF_DIG_RRF_REG_BRF_RRF_LDO_VREF_SEL_LV_Pos;

    // For low temperature stable
    hwp_ble_rfc->MISC_CTRL_REG &= ~BLE_RF_DIG_MISC_CTRL_REG_RVGA_WX2_STG1_FRC_EN;
    hwp_ble_rfc->MISC_CTRL_REG |= 0x01 << BLE_RF_DIG_MISC_CTRL_REG_RVGA_WX2_STG1_FRC_EN_Pos;

    hwp_ble_rfc->RBB_REG3 &= ~BLE_RF_DIG_RBB_REG3_BRF_RVGA_W2X_STG1_LV;
    hwp_ble_rfc->RBB_REG3 |= 0x0 << BLE_RF_DIG_RBB_REG3_BRF_RVGA_W2X_STG1_LV_Pos;

    // Fow 2M mode problem
    hwp_ble_rfc->MISC_CTRL_REG &= ~BLE_RF_DIG_MISC_CTRL_REG_PKDET_EN_EARLY_OFF_EN;


    //phy init, placed here for temp
    hwp_ble_phy->DEMOD_CFG1 &= ~BLE_PHY_DEMOD_CFG1_MU_ERR_Msk;
    hwp_ble_phy->DEMOD_CFG1 &= ~BLE_PHY_DEMOD_CFG1_MU_DC_Msk;

    hwp_ble_phy->DEMOD_CFG1 &= ~BLE_PHY_DEMOD_CFG1_DEMOD_G_Msk;// temp


    //hwp_ble_phy->DEMOD_CFG1 |= 0xB8 << BLE_PHY_DEMOD_CFG1_MU_ERR_Pos;
    //hwp_ble_phy->DEMOD_CFG1 |= 0x13 << BLE_PHY_DEMOD_CFG1_MU_DC_Pos;
    hwp_ble_phy->DEMOD_CFG1 |= 0x168 << BLE_PHY_DEMOD_CFG1_MU_ERR_Pos;
    hwp_ble_phy->DEMOD_CFG1 |= 0x22 << BLE_PHY_DEMOD_CFG1_MU_DC_Pos;
    hwp_ble_phy->DEMOD_CFG1 |= 0xB0 << BLE_PHY_DEMOD_CFG1_DEMOD_G_Pos;

    //hwp_ble_phy->NOTCH_CFG3 = 0x00800800 ;
    //hwp_ble_phy->NOTCH_CFG4 = 0x8 ;

    hwp_ble_phy->RSSI_CFG1 &= ~BLE_PHY_RSSI_CFG1_RSSI_OFFSET;
    hwp_ble_phy->RSSI_CFG1 |= 0x0F << BLE_PHY_RSSI_CFG1_RSSI_OFFSET_Pos;

    // IF = 1.995M
    hwp_ble_phy->MIXER_CFG1 &= ~BLE_PHY_MIXER_CFG1_RX_MIXER_PHASE_1;
    hwp_ble_phy->MIXER_CFG1 |= 0xA6 << BLE_PHY_MIXER_CFG1_RX_MIXER_PHASE_1_Pos;
    hwp_ble_phy->LFP_MMDIV_CFG0 &= ~BLE_PHY_LFP_MMDIV_CFG0_RX_MMDIV_OFFSET_1M;
    hwp_ble_phy->LFP_MMDIV_CFG1 &= ~BLE_PHY_LFP_MMDIV_CFG1_RX_MMDIV_OFFSET_2M;
    hwp_ble_phy->LFP_MMDIV_CFG0 |= 0x1AAE1 << BLE_PHY_LFP_MMDIV_CFG0_RX_MMDIV_OFFSET_1M_Pos;
    hwp_ble_phy->LFP_MMDIV_CFG1 |= 0x155C3 << BLE_PHY_LFP_MMDIV_CFG1_RX_MMDIV_OFFSET_2M_Pos;

    // For coded
    hwp_ble_phy->INTERP_CFG1 |= 0x02 << BLE_PHY_INTERP_CFG1_TED_MU_F_C_Pos;
    hwp_ble_phy->INTERP_CFG1 |= 0x04 << BLE_PHY_INTERP_CFG1_TED_MU_P_C_Pos;
    hwp_ble_phy->INTERP_CFG1 |= 0x01 << BLE_PHY_INTERP_CFG1_INTERP_METHOD_U_Pos;
    hwp_ble_phy->CODED_CFG3  |= 0x80 << BLE_PHY_CODED_CFG3_CI_MU_ERR_Pos;
    hwp_ble_phy->CODED_CFG3  |= 0x20 << BLE_PHY_CODED_CFG3_CI_MU_DC_Pos;
    hwp_ble_phy->CODED_CFG3  |= 0x30 << BLE_PHY_CODED_CFG3_CI_G_Pos;
    hwp_ble_phy->CODED_CFG4  |= 0x90 << BLE_PHY_CODED_CFG4_DEC_MU_ERR_Pos;
    hwp_ble_phy->CODED_CFG4  |= 0x20 << BLE_PHY_CODED_CFG4_DEC_MU_DC_Pos;
    hwp_ble_phy->CODED_CFG4  |= 0x00 << BLE_PHY_CODED_CFG4_DEC_G_Pos;

    // Sensetive improve
    hwp_ble_phy->NOTCH_CFG3 = 0x00800800;
    hwp_ble_phy->NOTCH_CFG4 |= 0xAA << BLE_PHY_NOTCH_CFG4_NOTCH_RSSI_THD_Pos;
    // Improve 2448Mhz against 48M hormony interference
    hwp_ble_phy->NOTCH_CFG1 &= ~BLE_PHY_NOTCH_CFG1_NOTCH_B1;
    hwp_ble_phy->NOTCH_CFG1 |= 0x3000 << BLE_PHY_NOTCH_CFG1_NOTCH_B1_Pos;

    // Improve TX freq offset to meet the requirement of stable modulation index.
    hwp_ble_phy->TX_GAUSSFLT_CFG &= ~BLE_PHY_TX_GAUSSFLT_CFG_GAUSS_GAIN_2;
    hwp_ble_phy->TX_GAUSSFLT_CFG |= 0x0FF << BLE_PHY_TX_GAUSSFLT_CFG_GAUSS_GAIN_2_Pos;


    hwp_ble_phy->TX_PA_CFG &= ~BLE_PHY_TX_PA_CFG_PA_RAMP_FACTOR_IDX_Msk;
    hwp_ble_phy->TX_PA_CFG |= 0x06 << BLE_PHY_TX_PA_CFG_PA_RAMP_FACTOR_IDX_Pos;

    // debug
    //hwp_ble_phy->PKTDET_CFG1 |= 0x01 << BLE_PHY_PKTDET_CFG1_HARD_CORR_THD_Pos;
    //hwp_ble_phy->PKTDET_CFG1 &= ~BLE_PHY_PKTDET_CFG1_PKTDET_THD;
    //hwp_ble_phy->PKTDET_CFG1 |= 0x1800 << BLE_PHY_PKTDET_CFG1_PKTDET_THD_Pos;

    // Test code, modify threshold
#if 0
    hwp_ble_phy->PKTDET_CFG1 &= ~BLE_PHY_PKTDET_CFG1_HARD_CORR_THD;
    hwp_ble_phy->PKTDET_CFG1 |= 0x0 << BLE_PHY_PKTDET_CFG1_HARD_CORR_THD_Pos;
    hwp_ble_phy->CODED_CFG2 &=  ~BLE_PHY_CODED_CFG2_HARD_CORR_THD_CODED;
    hwp_ble_phy->CODED_CFG2 |= 0x14f << BLE_PHY_CODED_CFG2_HARD_CORR_THD_CODED_Pos;
#endif

    // Optimize for 2M, 2448/2424/2472
    hwp_ble_phy->MIXER_CFG1 &= ~BLE_PHY_MIXER_CFG1_RX_MIXER_PHASE_2;
    hwp_ble_phy->MIXER_CFG1 |= 0xAA << BLE_PHY_MIXER_CFG1_RX_MIXER_PHASE_2_Pos;
    hwp_ble_phy->NOTCH_CFG4 &= ~BLE_PHY_NOTCH_CFG4_CHNL_NOTCH_EN1_Msk;
    hwp_ble_phy->NOTCH_CFG4 |= 0x08 << BLE_PHY_NOTCH_CFG4_CHNL_NOTCH_EN1_Pos;
    hwp_ble_rfc->RBB_REG1 &= ~BLE_RF_DIG_RBB_REG1_BRF_CBPF_FC_LV_2M_Msk;
    hwp_ble_rfc->RBB_REG1 |= 0x3 << BLE_RF_DIG_RBB_REG1_BRF_CBPF_FC_LV_2M_Pos;

    return;
    //while( 1 );
}

void ble_rf_rfc_init()
{
    uint8_t i;

    uint32_t rxon_addr;
    uint32_t rxoff_addr;
    uint32_t txon_addr;
    uint32_t txoff_addr;

    uint32_t rxon_cmd[70];
    uint32_t rxoff_cmd[70];
    uint32_t txon_cmd[70];
    uint32_t txoff_cmd[70];

    uint32_t cmd;
    uint32_t reg_data;

    //reset rccal
    hwp_ble_rfc->RBB_REG5 &= ~BLE_RF_DIG_RBB_REG5_BRF_RSTB_RCCAL_LV ;
    //release adc reset
    hwp_ble_rfc->ADC_REG  |= BLE_RF_DIG_ADC_REG_BRF_RSTB_ADC_LV ;
    //enable rf ref clk and adc clk
    //hwp_ble_rfc->MISC_CTRL_REG |= BLE_RF_DIG_MISC_CTRL_REG_PKDET_EN_EARLY_OFF_EN;
    hwp_ble_rfc->MISC_CTRL_REG |= BLE_RF_DIG_MISC_CTRL_REG_XTAL_REF_EN;
    hwp_ble_rfc->MISC_CTRL_REG |= BLE_RF_DIG_MISC_CTRL_REG_ADC_CLK_EN;
    //change adc fifo wr clk phase
    //hwp_ble_rfc->MISC_CTRL_REG |= BLE_RF_DIG_MISC_CTRL_REG_ADC_FIFO_CLK_PHASE_SEL;
    //fulcal data for debug
    uint32_t reg_addr = BLE_RFC_BASE ;
    reg_data = 0;

    //inccal time setting
    hwp_ble_rfc->INCCAL_REG1 |= (0x3f << BLE_RF_DIG_INCCAL_REG1_INCFCAL_WAIT_TIME_Pos) | \
                                (0x3f << BLE_RF_DIG_INCCAL_REG1_INCACAL_WAIT_TIME_Pos) ;
    //printf("BLE rf inccal init start\n");
    //------------RXON CMD----------------{{{
    //VDDPSW RFBG_EN
    rxon_cmd[0] = WR(0x4C) ;
    rxon_cmd[1] = RD(0x8) ;
    rxon_cmd[2] = OR(8) ;
    rxon_cmd[3] = OR(9) ;
    rxon_cmd[4] = WR(0x8) ;

    //wait 1us
    rxon_cmd[5] = WAIT(2) ;

    //VCO_EN
    rxon_cmd[6] = RD(0x0) ;
    rxon_cmd[7] = OR(27) ;
    rxon_cmd[8] = WR(0x0) ;

    //FBDV_EN
    rxon_cmd[9] = RD(0xC) ;
    rxon_cmd[10] = OR(11) ;
    rxon_cmd[11] = WR(0xC) ;

    //PFDCP_EN
    rxon_cmd[12] = RD(0x14) ;
    rxon_cmd[13] = OR(19) ;
    rxon_cmd[14] = WR(0x14) ;

    //LDO11_EN & LNA_SHUNTSW
    rxon_cmd[15] = RD(0x2C) ;
    rxon_cmd[16] = OR(22) ;   //not default value
    rxon_cmd[17] = AND(6) ;   //not default value
    rxon_cmd[18] = WR(0x2C) ;

    //ADC & LDO_ADC & LDO_ADCREF
    rxon_cmd[19] = RD(0x44) ;
    rxon_cmd[20] = OR(4) ;
    rxon_cmd[21] = OR(9) ;
    rxon_cmd[22] = OR(21) ;
    rxon_cmd[23] = OR(20) ;    //not default value
    rxon_cmd[24] = WR(0x44) ;

    //LDO_RBB
    rxon_cmd[25] = RD(0x30) ;
    rxon_cmd[26] = OR(13) ;   //not default value
    rxon_cmd[27] = WR(0x30) ;

    //PA_TX_RX
    rxon_cmd[28] = RD(0x28) ;
    rxon_cmd[29] = AND(9) ;  //not default value
    rxon_cmd[30] = WR(0x28) ;

    //EN_IARRAY & EN_OSDAC
    rxon_cmd[31] = RD(0x40) ;
    rxon_cmd[32] = OR(3) ;
    rxon_cmd[33] = OR(4) ;
    rxon_cmd[34] = OR(5) ;
    rxon_cmd[35] = WR(0x40) ;

    //EN_CBPF & EN_RVGA
    rxon_cmd[36] = RD(0x34) ;
    rxon_cmd[37] = OR(27) ;
    rxon_cmd[38] = OR(6) ;
    rxon_cmd[39] = OR(7) ;
    rxon_cmd[40] = WR(0x34) ;

    //EN_PKDET
    rxon_cmd[41] = RD(0x38) ;
    rxon_cmd[42] = OR(0) ;
    rxon_cmd[43] = OR(1) ;
    rxon_cmd[44] = OR(2) ;
    rxon_cmd[45] = OR(3) ;
    rxon_cmd[46] = WR(0x38) ;

    //wait 4us
    rxon_cmd[47] = WAIT(5) ;

    //LODIST_RX_EN
    rxon_cmd[48] = RD(0x8) ;
    rxon_cmd[49] = OR(7) ;   //not default value
    rxon_cmd[50] = WR(0x8) ;
    //LNA_Pu & MX_PU
    rxon_cmd[51] = RD(0x2c) ;
    rxon_cmd[52] = OR(3) ;
    rxon_cmd[53] = OR(17) ;  //not default value
    rxon_cmd[54] = WR(0x2c) ;

    //wait 6us
    rxon_cmd[55] = WAIT(7) ;

    //VCO_FLR_EN
    rxon_cmd[56] = RD(0x0) ;
    rxon_cmd[57] = OR(22) ;
    rxon_cmd[58] = WR(0x0) ;

    //FBDV_RSTB
    rxon_cmd[59] = RD(0xC) ;
    rxon_cmd[60] = AND(0x6) ;
    rxon_cmd[61] = WR(0xC) ;

    //wait 30us for lo lock
    rxon_cmd[62] = WAIT(45) ;

    //START INCCAL
    rxon_cmd[63] = RD(0xF4) ;   //inccal start
    rxon_cmd[64] = OR(29) ;
    rxon_cmd[65] = WR(0xF4) ;

    rxon_cmd[66] = WAIT(30) ;
    //END
    rxon_cmd[67] = END ;

    //}}}

    //----------------RXOFF CMD----------------------{{{
    //VDDPSW/RFBG/LODIST_RX_EN
    rxoff_cmd[0] = WR(0x4C) ;   //to avoid rx on/off collision in normal rx
    //rxoff_cmd[0] = END ; //to retain rx state when rx off in dc_est mode
    rxoff_cmd[1] = RD(0x8) ;
    rxoff_cmd[2] = AND(8) ;
    rxoff_cmd[3] = AND(9) ;
    rxoff_cmd[4] = AND(7) ;   //not default value
    rxoff_cmd[5] = WR(0x8) ;
    //VCO_EN & VCO_FLR_EN
    rxoff_cmd[6] = RD(0x0) ;
    rxoff_cmd[7] = AND(27) ;
    rxoff_cmd[8] = AND(22) ;
    rxoff_cmd[9] = WR(0x0) ;
    //FBDV_EN
    rxoff_cmd[10] = RD(0xC) ;
    rxoff_cmd[11] = AND(11) ;
    rxoff_cmd[12] = WR(0xC) ;
    //FBDV RSTB
    rxoff_cmd[13] = RD(0xC) ;
    rxoff_cmd[14] = OR(0x6) ;
    rxoff_cmd[15] = WR(0xC) ;

    //PFDCP_EN
    rxoff_cmd[16] = RD(0x14) ;
    rxoff_cmd[17] = AND(19) ;
    rxoff_cmd[18] = WR(0x14) ;

    //LNA_PU & MX_PU & LDO11_EN & LNA_SHUNTSW
    rxoff_cmd[19] = RD(0x2C) ;
    rxoff_cmd[20] = AND(3) ;
    rxoff_cmd[21] = AND(17) ;   //not default value
    rxoff_cmd[22] = AND(22) ;   //not default value
    rxoff_cmd[23] = OR(6) ;   //not default value
    rxoff_cmd[24] = WR(0x2C) ;

    //ADC & LDO_ADC & LDO_ADCREF
    rxoff_cmd[25] = RD(0x44) ;
    rxoff_cmd[26] = AND(4) ;
    rxoff_cmd[27] = AND(9) ;
    rxoff_cmd[28] = AND(21) ;
    rxoff_cmd[29] = AND(20) ;   //not default value
    rxoff_cmd[30] = WR(0x44) ;

    //LDO_RBB
    rxoff_cmd[31] = RD(0x30) ;
    rxoff_cmd[32] = AND(13)  ;  //not default value
    rxoff_cmd[33] = WR(0x30) ;

    //PA_TX_RX
    rxoff_cmd[34] = RD(0x28) ;
    rxoff_cmd[35] = OR(9) ;   //not default value
    rxoff_cmd[36] = WR(0x28) ;

    //EN_IARRAY & EN_OSDAC
    rxoff_cmd[37] = RD(0x40) ;
    rxoff_cmd[38] = AND(3) ;
    rxoff_cmd[39] = AND(4) ;
    rxoff_cmd[40] = AND(5) ;
    rxoff_cmd[41] = WR(0x40) ;

    //EN_CBPF & EN_RVGA
    rxoff_cmd[42] = RD(0x34) ;
    rxoff_cmd[43] = AND(27) ;
    rxoff_cmd[44] = AND(6) ;
    rxoff_cmd[45] = AND(7) ;
    rxoff_cmd[46] = WR(0x34) ;

    //EN_PKDET
    rxoff_cmd[47] = RD(0x38) ;
    rxoff_cmd[48] = AND(0) ;
    rxoff_cmd[49] = AND(1) ;
    rxoff_cmd[50] = AND(2) ;
    rxoff_cmd[51] = AND(3) ;
    rxoff_cmd[52] = WR(0x38) ;
    rxoff_cmd[53] = END ;

    //}}}

    //----------------TXON CMD----------------------{{{
    //VDDPSW RFBG_EN
    txon_cmd[0] = WR(0x4C) ;
    txon_cmd[1] = RD(0x8) ;
    txon_cmd[2] = OR(8) ;
    txon_cmd[3] = OR(9) ;
    txon_cmd[4] = WR(0x8) ;

    //wait 1us
    txon_cmd[5] = WAIT(2) ;

    //VCO_EN
    txon_cmd[6] = RD(0x0) ;
    txon_cmd[7] = OR(27) ;
    txon_cmd[8] = WR(0x0) ;

    //FBDV_EN
    txon_cmd[9] = RD(0xC) ;
    txon_cmd[10] = OR(11) ;
    txon_cmd[11] = WR(0xC) ;

    //PFDCP_EN&PFDCP_CSD_EN
    txon_cmd[12] = RD(0x14) ;
    txon_cmd[13] = OR(19) ;
    txon_cmd[14] = OR(4) ;
    txon_cmd[15] = WR(0x14) ;

    txon_cmd[16] = WAIT(20) ;
    //FBDV_RSTB
    txon_cmd[17] = RD(0xC) ;
    txon_cmd[18] = AND(0x6) ;
    txon_cmd[19] = WR(0xC) ;

    //wait 30us for lo lock
    txon_cmd[20] = WAIT(45) ;

    //LODIST_TX_EN
    txon_cmd[21] = RD(0x8) ;
    txon_cmd[22] = OR(4) ;   //not default value
    txon_cmd[23] = WR(0x8) ;

    //PA_BUF_PU for normal tx
    txon_cmd[24] = RD(0x24) ;
    txon_cmd[25] = OR(22) ;
    txon_cmd[26] = WR(0x24) ;


    ////ATTEN EN for dc est
    //txon_cmd[15] = RD( 0x28 ) ;
    //txon_cmd[16] = OR( 4 ) ;
    //txon_cmd[17] = WR( 0x28 ) ;


    //txon_cmd[20] = RD(0x24) ;
    //txon_cmd[21] = OR(22) ;
    //txon_cmd[22] = WR(0x24) ;
    //wait 4us
    txon_cmd[27] = WAIT(5) ;
    //PA_OUT_PU & TRF_SIG_EN
    txon_cmd[28] = RD(0x24) ;
    txon_cmd[29] = OR(16) ;
    txon_cmd[30] = OR(21) ;   //pa_out_pu for normal tx
    txon_cmd[31] = WR(0x24) ;

    //wait 6us
    txon_cmd[32] = WAIT(7) ;

    //txon_cmd[23] = WAIT(5) ;
    //PA_OUT_PU & TRF_SIG_EN
    //txon_cmd[24] = RD(0x24) ;
    //txon_cmd[25] = OR(16) ;
    //txon_cmd[26] = OR(21) ;   //pa_out_pu for normal tx


    //txon_cmd[27] = WR(0x24) ;
    //wait 6us
    //txon_cmd[28] = WAIT(7) ;

    //LODIST_TX_EN
    // txon_cmd[29] = RD(0x8) ;
    // txon_cmd[30] = OR(4) ;   //not default value
    // txon_cmd[31] = WR(0x8) ;

    //VCO_FLR_EN
    // txon_cmd[32] = RD(0x0) ;
    // txon_cmd[33] = OR(22) ;
    // txon_cmd[34] = WR(0x0) ;



    //START INCCAL
    txon_cmd[33] = RD(0xF4) ;   //inccal start
    txon_cmd[34] = OR(29) ;
    txon_cmd[35] = WR(0xF4) ;
    txon_cmd[36] = WAIT(30) ;

    //txon_cmd[38] = END ;
    txon_cmd[37] = END ;
    txon_cmd[38] = END ;
    txon_cmd[39] = END ;

    //}}}

    //----------------TXOFF CMD----------------------{{{
    //VDDPSW RFBG_EN LODIST_TX/RX_EN
    txoff_cmd[0] = RD(0x4C) ;
    txoff_cmd[1] = RD(0x8) ;
    //txoff_cmd[0] = END ;
    txoff_cmd[2] = AND(4) ;
    txoff_cmd[3] = AND(8) ;
    txoff_cmd[4] = AND(9) ;
    txoff_cmd[5] = WR(0x8) ;
    //VCO_EN & VCO_FLR_EN
    txoff_cmd[6] = RD(0x0) ;
    txoff_cmd[7] = AND(27) ;
    txoff_cmd[8] = AND(22) ;
    txoff_cmd[9] = WR(0x0) ;
    //FBDV_EN
    txoff_cmd[10] = RD(0xC) ;
    txoff_cmd[11] = AND(11) ;
    txoff_cmd[12] = WR(0xC) ;
    //FBDV_RSTB
    txoff_cmd[13] = RD(0xC) ;
    txoff_cmd[14] = OR(0x6) ;
    txoff_cmd[15] = WR(0xC) ;
    //PFDCP_EN
    txoff_cmd[16] = RD(0x14) ;
    txoff_cmd[17] = AND(19) ;
    txoff_cmd[18] = WR(0x14) ;

    //PA_BUF_PU & PA_OUT_PU & TRF_SIG_EN
    txoff_cmd[19] = RD(0x24) ;
    txoff_cmd[20] = AND(22) ;
    txoff_cmd[21] = AND(16) ;
    txoff_cmd[22] = AND(21) ;
    txoff_cmd[23] = WR(0x24) ;
    txoff_cmd[24] = END ;
    txoff_cmd[25] = END ;

    //}}}

    //printf("BLE rf rxon inccal init start\n");
    rxon_addr = BLE_RFC_BASE + 0x110; //0x41040110;
    hwp_ble_rfc->CU_ADDR_REG0 = 0;
    hwp_ble_rfc->CU_ADDR_REG0 = 0x110;
    for (i = 0; i < 34; i = i + 1)
    {
        //printf("cmd_addr = %x\n",rxon_addr );
        //printf("rxon_cmd[%d] = %x\n",i*2+1,rxon_cmd[i*2+1]);
        //printf("rxon_cmd[%d] = %x\n",i*2,rxon_cmd[i*2]);
        cmd = rxon_cmd[i * 2] + (rxon_cmd[i * 2 + 1] << 16) ;
        write_memory(rxon_addr, cmd);
        rxon_addr += 4;
    }
    //printf("BLE rf rxoff inccal init start\n");
    rxoff_addr = BLE_RFC_BASE + 0x198;//0x41040198;
    hwp_ble_rfc->CU_ADDR_REG0 += (0x198 << 10);
    for (i = 0; i < 27; i = i + 1)
    {
        //printf("cmd_addr = %x\n",rxoff_addr );
        //printf("rxoff_cmd[%d] = %x\n",i*2+1,rxoff_cmd[i*2+1]);
        //printf("rxoff_cmd[%d] = %x\n",i*2,rxoff_cmd[i*2]);
        cmd = rxoff_cmd[i * 2] + (rxoff_cmd[i * 2 + 1] << 16) ;
        write_memory(rxoff_addr, cmd);
        rxoff_addr += 4 ;
    }

    txon_addr = BLE_RFC_BASE + 0x204;//0x41040204;
    hwp_ble_rfc->CU_ADDR_REG1 = 0;
    hwp_ble_rfc->CU_ADDR_REG1 = 0x204;
    for (i = 0; i < 20; i = i + 1)
    {
        //printf("cmd_addr = %x\n",txon_addr );
        //printf("txon_cmd[%d] = %x\n",i*2+1,txon_cmd[i*2+1]);
        //printf("txon_cmd[%d] = %x\n",i*2,txon_cmd[i*2]);
        cmd = txon_cmd[i * 2] + (txon_cmd[i * 2 + 1] << 16) ;
        write_memory(txon_addr, cmd);
        txon_addr += 4;
    }
    //printf("BLE rf rxoff inccal init start\n");
    txoff_addr = BLE_RFC_BASE + 0x25C; //0x4104025C;
    hwp_ble_rfc->CU_ADDR_REG1 += (0x25C << 10);
    for (i = 0; i < 14; i = i + 1)
    {
        //printf("cmd_addr = %x\n",txoff_addr );
        //printf("txoff_cmd[%d] = %x\n",i*2+1,txoff_cmd[i*2+1]);
        //printf("txoff_cmd[%d] = %x\n",i*2,txoff_cmd[i*2]);
        cmd = txoff_cmd[i * 2] + (txoff_cmd[i * 2 + 1] << 16) ;
        write_memory(txoff_addr, cmd);
        txoff_addr += 4 ;
    }

}


void bt_rf_cal(void)
{
    ble_rf_fulcal();
    ble_rf_rfc_init();
}

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
