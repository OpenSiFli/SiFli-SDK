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


#include <stdlib.h>

#include "bf0_hal_rcc.h"
#include "register.h"
#include "ble_rf_cal.h"
//#include "rtthread.h"
//#include "rtdef.h"
//#include "rthw.h"


#define _HAL_Delay_us(x) HAL_Delay_us(x)
//#define RF_PRINTF rt_kprintf
#define RF_PRINTF(...)

#define RF_PWR_PARA(max, min, init, is_bqb) (uint32_t)((is_bqb << 24) | (init << 16) | (min << 8) | (int8_t)(max))

//#define ENABLE_EDR_5G
#define ENABLE_EDR_3G
//#define ENABLE_EDR_2G
//#define ENABLE_RF_ATE
//#define ENABLE_IQ_MODULE
uint8_t bt_is_in_BQB_mode(void);
int8_t bt_rf_get_max_tx_pwr(void);
int8_t bt_rf_get_init_tx_pwr(void);
int8_t bt_rf_get_min_tx_pwr(void);



static const uint32_t ref_residual_cnt_tbl_rx_1m[40] =
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

static const uint32_t ref_residual_cnt_tbl_rx_2m[40] =
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
#if defined(ENABLE_EDR_5G)
static const uint32_t residual_cnt_vth_5g = 39870; //TODO
static const uint32_t residual_cnt_vtl_5g = 36540; //TODO

#elif defined(ENABLE_EDR_3G)
static const uint32_t residual_cnt_vth_3g = 33670;//TODO
static const uint32_t residual_cnt_vtl_3g = 30530;//TODO

static const uint32_t ref_residual_cnt_tbl_tx_3g[79] =
{
    30544,
    30584,
    30624,
    30664,
    30704,
    30744,
    30784,
    30824,
    30864,
    30904,
    30944,
    30984,
    31024,
    31064,
    31104,
    31144,
    31184,
    31224,
    31264,
    31304,
    31344,
    31384,
    31424,
    31464,
    31504,
    31544,
    31584,
    31624,
    31664,
    31704,
    31744,
    31784,
    31824,
    31864,
    31904,
    31944,
    31984,
    32024,
    32064,
    32104,
    32144,
    32184,
    32224,
    32264,
    32304,
    32344,
    32384,
    32424,
    32464,
    32504,
    32544,
    32584,
    32624,
    32664,
    32704,
    32744,
    32784,
    32824,
    32864,
    32904,
    32944,
    32984,
    33024,
    33064,
    33104,
    33144,
    33184,
    33224,
    33264,
    33304,
    33344,
    33384,
    33424,
    33464,
    33504,
    33544,
    33584,
    33624,
    33664
};
#elif defined(ENABLE_EDR_2G)


const uint32_t residual_cnt_vth_2g = 33864; //TODO
const uint32_t residual_cnt_vtl_2g = 30224; //TODO

uint32_t ref_residual_cnt_tbl_tx_2g[79] =
{
    51043,
    51064,
    51085,
    51106,
    51128,
    51149,
    51170,
    51191,
    51213,
    51234,
    51255,
    51276,
    51298,
    51319,
    51340,
    51361,
    51383,
    51404,
    51425,
    51446,
    51468,
    51489,
    51510,
    51531,
    51553,
    51574,
    51595,
    51616,
    51638,
    51659,
    51680,
    51701,
    51723,
    51744,
    51765,
    51786,
    51808,
    51829,
    51850,
    51871,
    51893,
    51914,
    51935,
    51956,
    51978,
    51999,
    52020,
    52041,
    52063,
    52084,
    52105,
    52126,
    52148,
    52169,
    52190,
    52211,
    52233,
    52254,
    52275,
    52296,
    52318,
    52339,
    52360,
    52381,
    52403,
    52424,
    52445,
    52466,
    52488,
    52509,
    52530,
    52551,
    52573,
    52594,
    52615,
    52636,
    52658,
    52679,
    52700
};

#endif

static const uint32_t ref_residual_cnt_tbl_rx_bt[79] =
{
    30445,
    30485,
    30525,
    30565,
    30605,
    30645,
    30685,
    30725,
    30765,
    30805,
    30845,
    30885,
    30925,
    30965,
    31005,
    31045,
    31085,
    31125,
    31165,
    31205,
    31245,
    31285,
    31325,
    31365,
    31405,
    31445,
    31485,
    31525,
    31565,
    31605,
    31645,
    31685,
    31725,
    31765,
    31805,
    31845,
    31885,
    31925,
    31965,
    32005,
    32045,
    32085,
    32125,
    32165,
    32205,
    32245,
    32285,
    32325,
    32365,
    32405,
    32445,
    32485,
    32525,
    32565,
    32605,
    32645,
    32685,
    32725,
    32765,
    32805,
    32845,
    32885,
    32925,
    32965,
    33005,
    33045,
    33085,
    33125,
    33165,
    33205,
    33245,
    33285,
    33325,
    33365,
    33405,
    33445,
    33485,
    33525,
    33565
};

static const uint32_t ref_residual_cnt_tbl_tx[79] =
{
    30545,
    30585,
    30625,
    30665,
    30705,
    30745,
    30785,
    30825,
    30865,
    30905,
    30945,
    30985,
    31025,
    31065,
    31105,
    31145,
    31185,
    31225,
    31265,
    31305,
    31345,
    31385,
    31425,
    31465,
    31505,
    31545,
    31585,
    31625,
    31665,
    31705,
    31745,
    31785,
    31825,
    31865,
    31905,
    31945,
    31985,
    32025,
    32065,
    32105,
    32145,
    32185,
    32225,
    32265,
    32305,
    32345,
    32385,
    32425,
    32465,
    32505,
    32545,
    32585,
    32625,
    32665,
    32705,
    32745,
    32785,
    32825,
    32865,
    32905,
    32945,
    32985,
    33025,
    33065,
    33105,
    33145,
    33185,
    33225,
    33265,
    33305,
    33345,
    33385,
    33425,
    33465,
    33505,
    33545,
    33585,
    33625,
    33665
};


static const uint32_t ref_residual_cnt_tbl_tx_5g[79] =
{
    36549,
    36592,
    36634,
    36677,
    36719,
    36762,
    36804,
    36847,
    36889,
    36932,
    36974,
    37017,
    37059,
    37102,
    37144,
    37187,
    37229,
    37272,
    37314,
    37357,
    37399,
    37442,
    37484,
    37527,
    37569,
    37612,
    37654,
    37697,
    37739,
    37782,
    37824,
    37867,
    37909,
    37952,
    37994,
    38037,
    38079,
    38122,
    38164,
    38207,
    38249,
    38292,
    38334,
    38377,
    38419,
    38462,
    38504,
    38547,
    38589,
    38632,
    38674,
    38717,
    38759,
    38802,
    38844,
    38887,
    38929,
    38972,
    39014,
    39057,
    39099,
    39142,
    39184,
    39227,
    39269,
    39312,
    39354,
    39397,
    39439,
    39482,
    39524,
    39567,
    39609,
    39652,
    39694,
    39737,
    39779,
    39822,
    39864
};

uint8_t dpsk_gain[79] =
{
    0x3E,
    0x3E,
    0x3E,
    0x3E,
    0x3F,
    0x3E,
    0x3E,
    0x3E,
    0x3E,
    0x3E,
    0x3D,
    0x3D,
    0x3E,
    0x3D,
    0x3D,
    0x3D,
    0x3D,
    0x3C,
    0x3C,
    0x3C,
    0x3C,
    0x3C,
    0x3B,
    0x3B,
    0x3A,
    0x3A,
    0x3A,
    0x39,
    0x39,
    0x39,
    0x38,
    0x38,
    0x38,
    0x37,
    0x37,
    0x36,
    0x36,
    0x35,
    0x35,
    0x35,
    0x34,
    0x34,
    0x34,
    0x34,
    0x33,
    0x33,
    0x33,
    0x33,
    0x32,
    0x32,
    0x32,
    0x32,
    0x32,
    0x31,
    0x31,
    0x31,
    0x31,
    0x31,
    0x31,
    0x31,
    0x32,
    0x32,
    0x32,
    0x32,
    0x33,
    0x33,
    0x33,
    0x34,
    0x34,
    0x34,
    0x35,
    0x35,
    0x35,
    0x36,
    0x36,
    0x36,
    0x37,
    0x37,
    0x37
};

#define BT_CHANNEL_NUM 110
uint32_t bt_rfc_init()
{
    uint32_t i;

    uint32_t rxon_addr;
    uint32_t rxoff_addr;
    uint32_t txon_addr;
    uint32_t txoff_addr;
    uint32_t bt_txon_addr;
    uint32_t bt_txoff_addr;

    uint32_t rxon_cmd_num;
    uint32_t rxoff_cmd_num;
    uint32_t txon_cmd_num;
    uint32_t txoff_cmd_num;
    uint32_t bt_txon_cmd_num;
    uint32_t bt_txoff_cmd_num;

    uint32_t *rxon_cmd = malloc(BT_CHANNEL_NUM * 4);
    uint32_t *rxoff_cmd = malloc(BT_CHANNEL_NUM * 4);
    uint32_t *txon_cmd = malloc(BT_CHANNEL_NUM * 4);
    uint32_t *txoff_cmd = malloc(BT_CHANNEL_NUM * 4);
    uint32_t *bt_txon_cmd = malloc(BT_CHANNEL_NUM * 4);
    uint32_t *bt_txoff_cmd = malloc(BT_CHANNEL_NUM * 4);

    uint32_t cmd;
    //uint32_t reg_data;
    uint32_t reg_addr = 0x0 ;
    //enable adc q for all phy
    hwp_bt_phy->RX_CTRL1 |= BT_PHY_RX_CTRL1_ADC_Q_EN_1;
    hwp_bt_phy->RX_CTRL2 |= BT_PHY_RX_CTRL2_ADC_Q_EN_FRC_EN;
    //hwp_bt_phy->RX_CTRL2 |= BT_PHY_RX_CTRL2_ADC_Q_EN_C;
    //hwp_bt_phy->RX_CTRL2 |= BT_PHY_RX_CTRL2_ADC_Q_EN_BR;

    //zero if
    hwp_bt_phy->TX_IF_MOD_CFG  &= ~BT_PHY_TX_IF_MOD_CFG_TX_IF_PHASE_BLE_Msk ;
    //hwp_bt_phy->MIXER_CFG1 = 0;

    //reset rccal
    hwp_bt_rfc->RBB_REG5 &= ~BT_RFC_RBB_REG5_BRF_RSTB_RCCAL_LV ;
    //release adc reset
    hwp_bt_rfc->ADC_REG  |= BT_RFC_ADC_REG_BRF_RSTB_ADC_LV ;
    //disable pkdet det early off
    hwp_bt_rfc->MISC_CTRL_REG &= ~BT_RFC_MISC_CTRL_REG_PKDET_EN_EARLY_OFF_EN;


    //to select 5G VCO for iq tx
#if defined(ENABLE_EDR_5G)
    hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MMDIV_SEL;
    hwp_bt_rfc->EDR_PLL_REG4 &= ~BT_RFC_EDR_PLL_REG4_BRF_EDR_SEL_VC_PATH_LV;
    hwp_bt_rfc->EDR_PLL_REG4 |= 1 << BT_RFC_EDR_PLL_REG4_BRF_EDR_SEL_VC_PATH_LV_Pos;
    hwp_bt_rfc->EDR_OSLO_REG &= ~BT_RFC_EDR_OSLO_REG_BRF_EDR_SEL_LODIST_TX_LV;
    hwp_bt_rfc->EDR_OSLO_REG |= 1 << BT_RFC_EDR_OSLO_REG_BRF_EDR_SEL_LODIST_TX_LV_Pos;
    //select 3G VCO for iq tx
#elif defined(ENABLE_EDR_3G)
    //select 3G VCO for iq tx
    hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MMDIV_SEL;
    hwp_bt_phy->TX_CTRL |= 0x1 << BT_PHY_TX_CTRL_MMDIV_SEL_Pos;
#elif defined(ENABLE_EDR_2G)
    hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MMDIV_SEL;
    hwp_bt_phy->TX_CTRL |= 0x2 << BT_PHY_TX_CTRL_MMDIV_SEL_Pos;
#else
#error "Must defined frequency"
#endif


    //to select 2G VCO for iq tx
    //hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MMDIV_SEL;
    //hwp_bt_phy->TX_CTRL |= 0x2 << BT_PHY_TX_CTRL_MMDIV_SEL_Pos;

    //change adc fifo wr clk phase
    //hwp_bt_rfc->MISC_CTRL_REG |= BT_RFC_MISC_CTRL_REG_ADC_FIFO_CLK_PHASE_SEL;
    //{{{---------fulcal and dccal data for debug----------------------------
    /*
    uint32_t reg_data;
    reg_data = 0;
    //store ble rx cal result
    hwp_bt_rfc->CAL_ADDR_REG1 = 0;
    hwp_bt_rfc->CAL_ADDR_REG1 = reg_addr;
    for( i=0;i<40;i++ ){
       reg_data = ((((79-i)<<BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos) + ((79-i) << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos))<<16) + \
                  (i<<BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos) + (i << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos);
       //RF_PRINTF("reg_data is %x i=%d\n",reg_data, i);
       write_memory( BT_RFC_MEM_BASE+reg_addr,reg_data  );
       reg_addr +=4;
    }
    //store bt rx cal result
    hwp_bt_rfc->CAL_ADDR_REG1 += reg_addr<<16;
    for( i=0;i<40;i++ ){
       reg_data = ((((i*2)<<BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos)   + ((i*2) << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos))<<16) + \
                   ((i*2+1)<<BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos)  + ((i*2+1) << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos);
       //RF_PRINTF("reg_data is %x i=%d\n",reg_data, i);
       write_memory( BT_RFC_MEM_BASE+reg_addr,reg_data  );
       reg_addr +=4;
    }
    //store ble tx cal result
    hwp_bt_rfc->CAL_ADDR_REG2 = reg_addr;
    for( i=0;i<79;i++ ){
       reg_data = ((79-i)<< BT_RFC_DCO_REG3_TX_KCAL_Pos) + ((79-i)<<BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos) + ((79-i) << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos);
       write_memory( BT_RFC_MEM_BASE+reg_addr,reg_data  );
       reg_addr +=4;
    }
    //store bt tx cal result
    hwp_bt_rfc->CAL_ADDR_REG2 += reg_addr<<16;
    for( i=0;i<79;i++ ){
       reg_data = ( ((i) << BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_FC_LV_Pos       ) & BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_FC_LV_Msk        )+ \
                  ( ((i) << BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_BM_LV_Pos       ) & BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_BM_LV_Msk        )+ \
                  ( ((i) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos      ) & BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk       )+ \
                  ( ((i) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos       ) & BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk        )+ \
                  ( ((i) << BT_RFC_EDR_CAL_REG1_BRF_TRF_EDR_TMXCAP_SEL_LV_Pos) & BT_RFC_EDR_CAL_REG1_BRF_TRF_EDR_TMXCAP_SEL_LV_Msk );

       //RF_PRINTF("reg_data is %x i=%d\n",reg_data, i);
       write_memory( BT_RFC_MEM_BASE+reg_addr,reg_data  );
       reg_addr +=4;
    }
    //store txdc cal result
    hwp_bt_rfc->CAL_ADDR_REG3 = reg_addr;
    for( i=0;i<8;i++ ){
       reg_data = (i+1) + ((0x1000+i+1)<<BT_RFC_TXDC_CAL_REG1_TX_DC_CAL_COEF1_Pos);
       write_memory( BT_RFC_MEM_BASE+reg_addr,reg_data  );
       reg_addr +=4;
       reg_data = (8-i) + ((8-i)<<BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_I_Pos);
       write_memory( BT_RFC_MEM_BASE+reg_addr,reg_data  );
       reg_addr +=4;
    }
    //}}}
    */
    //temp bp dccal_coef
    //hwp_bt_phy->TX_DC_CAL_CFG3 |=BT_PHY_TX_DC_CAL_CFG3_TX_DC_CAL_COEF_FRC_EN ;
    //hwp_bt_phy->TX_DC_CAL_CFG4 |=BT_PHY_TX_DC_CAL_CFG4_TX_DC_CAL_OFFSET_FRC_EN ;

    //inccal time setting
    hwp_bt_rfc->INCCAL_REG1 |= (0x3f << BT_RFC_INCCAL_REG1_INCFCAL_WAIT_TIME_Pos) | \
                               (0x3f << BT_RFC_INCCAL_REG1_INCACAL_WAIT_TIME_Pos) ;
    //RF_PRINTF("BLE rf inccal init start\n");
    //----------------RXON CMD----------------{{{
    i = 0;
    //VDDPSW RFBG_EN
    rxon_cmd[i++] = RD(0x10) ;
    rxon_cmd[i++] = RD(0x10) ;
    rxon_cmd[i++] = OR(8) ;
    rxon_cmd[i++] = OR(9) ;
    rxon_cmd[i++] = WR(0x10) ;

    //wait 1us
    rxon_cmd[i++] = WAIT(2) ;

    //VCO_EN
    rxon_cmd[i++] = RD(0x0) ;
    rxon_cmd[i++] = OR(12) ;
    rxon_cmd[i++] = WR(0x0) ;

    //FBDV_EN
    rxon_cmd[i++] = RD(0x14) ;
    rxon_cmd[i++] = OR(11) ;
    rxon_cmd[i++] = WR(0x14) ;

    //PFDCP_EN
    rxon_cmd[i++] = RD(0x1C) ;
    rxon_cmd[i++] = OR(19) ;
    rxon_cmd[i++] = WR(0x1C) ;

    //LDO11_EN & LNA_SHUNTSW
    rxon_cmd[i++] = RD(0x60) ;
    rxon_cmd[i++] = OR(22) ;
    rxon_cmd[i++] = AND(6) ;
    rxon_cmd[i++] = WR(0x60) ;

    //ADC & LDO_ADC & LDO_ADCREF
    rxon_cmd[i++] = RD(0x7C) ;
    rxon_cmd[i++] = OR(4) ;
    rxon_cmd[i++] = OR(9) ;
    rxon_cmd[i++] = OR(21) ;
    rxon_cmd[i++] = OR(20) ;   //if disable adc-1, change to 22
    rxon_cmd[i++] = WR(0x7C) ;

    //LDO_RBB
    rxon_cmd[i++] = RD(0x64) ;
    rxon_cmd[i++] = OR(13) ;
    rxon_cmd[i++] = WR(0x64) ;

    //PA_TX_RX
    rxon_cmd[i++] = RD(0x54) ;
    rxon_cmd[i++] = AND(9) ;
    rxon_cmd[i++] = WR(0x54) ;

    //EN_IARRAY & EN_OSDAC
    rxon_cmd[i++] = RD(0x74) ;
    rxon_cmd[i++] = OR(5) ;
    rxon_cmd[i++] = OR(6) ;
    rxon_cmd[i++] = OR(7) ;
    rxon_cmd[i++] = WR(0x74) ;

    //EN_CBPF & EN_RVGA
    rxon_cmd[i++] = RD(0x68) ;
    rxon_cmd[i++] = OR(27) ;
    rxon_cmd[i++] = OR(6) ;
    rxon_cmd[i++] = OR(7) ;
    rxon_cmd[i++] = WR(0x68) ;

    //EN_PKDET
    rxon_cmd[i++] = RD(0x6c) ;
    rxon_cmd[i++] = OR(0) ; //temp comment out for work around
    rxon_cmd[i++] = OR(1) ;
    rxon_cmd[i++] = OR(2) ;
    rxon_cmd[i++] = OR(3) ;
    rxon_cmd[i++] = WR(0x6c) ;

    //wait 4us
    rxon_cmd[i++] = WAIT(5) ;

    //LODIST_RX_EN
    rxon_cmd[i++] = RD(0x10) ;
    rxon_cmd[i++] = OR(7) ;
    rxon_cmd[i++] = WR(0x10) ;
    //LNA_PU & MX_PU
    rxon_cmd[i++] = RD(0x60) ;
    rxon_cmd[i++] = OR(3) ;
    rxon_cmd[i++] = OR(17) ;
    rxon_cmd[i++] = WR(0x60) ;

    //FULCAL RSLT
    rxon_cmd[i++] = RD_FULCAL;
    rxon_cmd[i++] = WR(0x8) ;

    //wait 6us
    rxon_cmd[i++] = WAIT(7) ;

    //VCO_FLR_EN
    rxon_cmd[i++] = RD(0x0) ;
    rxon_cmd[i++] = OR(7) ;
    rxon_cmd[i++] = WR(0x0) ;

    //FBDV_RSTB
    rxon_cmd[i++] = RD(0x14) ;
    rxon_cmd[i++] = AND(0x6) ;
    rxon_cmd[i++] = WR(0x14) ;

    //wait 30us for lo lock
    rxon_cmd[i++] = WAIT(45) ;

    //START INCCAL
    rxon_cmd[i++] = RD(0x90) ;   //inccal start
    rxon_cmd[i++] = OR(29) ;
    rxon_cmd[i++] = WR(0x90) ;

    rxon_cmd[i++] = WAIT(30) ;
    //END
    rxon_cmd[i++] = END ;
    if (i % 2)
    {
        rxon_cmd[i++] = END ;
    }
    HAL_ASSERT(i <= BT_CHANNEL_NUM);
    rxon_cmd_num = i ;
    //}}}

    //----------------RXOFF CMD----------------------{{{
    i = 0;
    //VDDPSW/RFBG/LODIST_RX_EN
    rxoff_cmd[i++] = RD(0x10) ;   //to avoid rx on/off collision in normal rx
    //rxoff_cmd[i++] = END ; //to retain rx state when rx off in dc_est mode
    rxoff_cmd[i++] = RD(0x10) ;
    rxoff_cmd[i++] = AND(8) ;
    rxoff_cmd[i++] = AND(9) ;
    rxoff_cmd[i++] = AND(7) ;
    rxoff_cmd[i++] = WR(0x10) ;
    //VCO_EN & VCO_FLR_EN
    rxoff_cmd[i++] = RD(0x0) ;
    rxoff_cmd[i++] = AND(12) ;
    rxoff_cmd[i++] = AND(7) ;
    rxoff_cmd[i++] = WR(0x0) ;
    //FBDV_EN
    //FBDV RSTB
    rxoff_cmd[i++] = RD(0x14) ;
    rxoff_cmd[i++] = AND(11) ;
    rxoff_cmd[i++] = OR(0x6) ;
    rxoff_cmd[i++] = WR(0x14) ;

    //PFDCP_EN
    rxoff_cmd[i++] = RD(0x1C) ;
    rxoff_cmd[i++] = AND(19) ;
    rxoff_cmd[i++] = WR(0x1C) ;

    //LNA_PU & MX_PU & LDO11_EN & LNA_SHUNTSW
    rxoff_cmd[i++] = RD(0x60) ;
    rxoff_cmd[i++] = AND(3) ;
    rxoff_cmd[i++] = OR(6) ;
    rxoff_cmd[i++] = AND(17) ;
    rxoff_cmd[i++] = AND(22) ;
    rxoff_cmd[i++] = WR(0x60) ;

    //ADC & LDO_ADC & LDO_ADCREF
    rxoff_cmd[i++] = RD(0x7C) ;
    rxoff_cmd[i++] = AND(4) ;
    rxoff_cmd[i++] = AND(9) ;
    rxoff_cmd[i++] = AND(21) ;
    rxoff_cmd[i++] = AND(20) ;
    rxoff_cmd[i++] = WR(0x7C) ;

    //LDO_RBB
    rxoff_cmd[i++] = RD(0x64) ;
    rxoff_cmd[i++] = AND(13) ;
    rxoff_cmd[i++] = WR(0x64) ;

    //PA_TX_RX
    rxoff_cmd[i++] = RD(0x54) ;
    rxoff_cmd[i++] = OR(9) ;
    rxoff_cmd[i++] = WR(0x54) ;

    //EN_IARRAY & EN_OSDAC
    rxoff_cmd[i++] = RD(0x74) ;
    rxoff_cmd[i++] = AND(5) ;
    rxoff_cmd[i++] = AND(6) ;
    rxoff_cmd[i++] = AND(7) ;
    rxoff_cmd[i++] = WR(0x74) ;

    //EN_CBPF & EN_RVGA
    rxoff_cmd[i++] = RD(0x68) ;
    rxoff_cmd[i++] = AND(27) ;
    rxoff_cmd[i++] = AND(6) ;
    rxoff_cmd[i++] = AND(7) ;
    rxoff_cmd[i++] = WR(0x68) ;

    //EN_PKDET
    rxoff_cmd[i++] = RD(0x6C) ;
    rxoff_cmd[i++] = AND(0) ;
    rxoff_cmd[i++] = AND(1) ;
    rxoff_cmd[i++] = AND(2) ;
    rxoff_cmd[i++] = AND(3) ;
    rxoff_cmd[i++] = WR(0x6C) ;
    //END
    rxoff_cmd[i++] = END ;
    if (i % 2)
    {
        rxoff_cmd[i++] = END ;
    }
    HAL_ASSERT(i <= BT_CHANNEL_NUM);
    rxoff_cmd_num = i;

    //}}}

    //----------------TXON CMD----------------------{{{
    i = 0;

    //RD FULCAL
    txon_cmd[i++] = RD_FULCAL ;
    txon_cmd[i++] = RD_FULCAL ;
    txon_cmd[i++] = WR(0x8) ;

    //VDDPSW RFBG_EN
    //txon_cmd[i++] = RD( 0x10 ) ;
    txon_cmd[i++] = RD(0x10) ;
    txon_cmd[i++] = OR(8) ;
    txon_cmd[i++] = OR(9) ;
    txon_cmd[i++] = WR(0x10) ;

    //wait 1us
    txon_cmd[i++] = WAIT(2) ;

    //VCO_EN
    txon_cmd[i++] = RD(0x0) ;
    txon_cmd[i++] = OR(12) ;
    txon_cmd[i++] = WR(0x0) ;

    //FBDV_EN
    txon_cmd[i++] = RD(0x14) ;
    txon_cmd[i++] = OR(11) ;
    txon_cmd[i++] = WR(0x14) ;

    //PFDCP_EN&PFDCP_CSD_EN
    txon_cmd[i++] = RD(0x1C) ;
    txon_cmd[i++] = OR(19) ;
    txon_cmd[i++] = OR(4) ;
    txon_cmd[i++] = WR(0x1C) ;

    //wait 6us
    txon_cmd[i++] = WAIT(7) ;

    //VCO_FLR_EN
    txon_cmd[i++] = RD(0x0) ;
    txon_cmd[i++] = OR(7) ;
    txon_cmd[i++] = WR(0x0) ;

    //FBDV_RSTB
    txon_cmd[i++] = RD(0x14) ;
    txon_cmd[i++] = AND(0x6) ;
    txon_cmd[i++] = WR(0x14) ;

    //wait 30us for lo lock
    txon_cmd[i++] = WAIT(45) ;

    //LODIST_TX_EN
    txon_cmd[i++] = RD(0x10) ;
    txon_cmd[i++] = OR(4) ;
    txon_cmd[i++] = WR(0x10) ;

    //PA_BUF_PU for normal tx
    txon_cmd[i++] = RD(0x50) ;
    txon_cmd[i++] = OR(22) ;
    txon_cmd[i++] = WR(0x50) ;

    ////ATTEN EN for dc est
    //txon_cmd[i++] = RD( 0x28 ) ;
    //txon_cmd[i++] = OR( 4 ) ;
    //txon_cmd[i++] = WR( 0x28 ) ;

    //EDR_IARRAY_EN
    txon_cmd[i++] = RD(0x58) ;
    txon_cmd[i++] = OR(29) ;
    txon_cmd[i++] = WR(0x58) ;

    //EDR_XFMR_SG
    txon_cmd[i++] = RD(0x5C) ;
    txon_cmd[i++] = AND(11) ;
    txon_cmd[i++] = WR(0x5C) ;

    //wait 4us
    txon_cmd[i++] = WAIT(5) ;


    //PA_OUT_PU & TRF_SIG_EN
    txon_cmd[i++] = RD(0x50) ;
    txon_cmd[i++] = OR(16) ;
    txon_cmd[i++] = OR(21) ;   //pa_out_pu for normal tx
    //txon_cmd[i++] = OR( 16 ) ; //no pa_out_pu for dc est tx
    txon_cmd[i++] = WR(0x50) ;


    //START INCCAL
    txon_cmd[i++] = RD(0x90) ;   //inccal start
    txon_cmd[i++] = OR(29) ;
    txon_cmd[i++] = WR(0x90) ;
    txon_cmd[i++] = WAIT(30) ;

    //END
    txon_cmd[i++] = END ;
    if (i % 2)
    {
        txon_cmd[i++] = END ;
    }
    HAL_ASSERT(i <= BT_CHANNEL_NUM);
    txon_cmd_num = i ;
    //}}}

    //----------------TXOFF CMD----------------------{{{
    i = 0;
    //VDDPSW RFBG_EN LODIST_TX/RX_EN
    txoff_cmd[i++] = RD(0x10) ;
    txoff_cmd[i++] = RD(0x10) ;
    //txoff_cmd[i++] = END ;
    txoff_cmd[i++] = AND(4) ;
    txoff_cmd[i++] = AND(8) ;
    txoff_cmd[i++] = AND(9) ;
    txoff_cmd[i++] = WR(0x10) ;
    //VCO_EN & VCO_FLR_EN
    txoff_cmd[i++] = RD(0x0) ;
    txoff_cmd[i++] = AND(12) ;
    txoff_cmd[i++] = AND(7) ;
    txoff_cmd[i++] = WR(0x0) ;
    //FBDV_EN
    txoff_cmd[i++] = RD(0x14) ;
    txoff_cmd[i++] = AND(11) ;
    //FBDV_RSTB
    txoff_cmd[i++] = OR(0x6) ;
    txoff_cmd[i++] = WR(0x14) ;
    //PFDCP_EN
    txoff_cmd[i++] = RD(0x1C) ;
    txoff_cmd[i++] = AND(19) ;
    txoff_cmd[i++] = WR(0x1C) ;

    //PA_BUF_PU & PA_OUT_PU & TRF_SIG_EN
    txoff_cmd[i++] = RD(0x50) ;
    txoff_cmd[i++] = AND(22) ;
    txoff_cmd[i++] = AND(16) ;
    txoff_cmd[i++] = AND(21) ;
    txoff_cmd[i++] = WR(0x50) ;

    //TRF_EDR_IARRAY_EN
    txoff_cmd[i++] = RD(0x58) ;
    txoff_cmd[i++] = AND(29) ;
    txoff_cmd[i++] = WR(0x58) ;

    //EDR_XFMR_SG
    txoff_cmd[i++] = RD(0x5C) ;
    txoff_cmd[i++] = OR(11) ;
    txoff_cmd[i++] = WR(0x5C) ;

    //END
    txoff_cmd[i++] = END ;
    if (i % 2)
    {
        txoff_cmd[i++] = END ;
    }
    HAL_ASSERT(i <= BT_CHANNEL_NUM);
    txoff_cmd_num = i ;

    //}}}

    //----------------BT_TXON CMD----------------------{{{
    i = 0;
    //VDDPSW RFBG_EN
    bt_txon_cmd[i++] = RD(0x10) ;
    bt_txon_cmd[i++] = RD(0x10) ;
    bt_txon_cmd[i++] = OR(8) ;
    bt_txon_cmd[i++] = OR(9) ;
    bt_txon_cmd[i++] = WR(0x10) ;

    //EDR_LO_EN_IARY
    bt_txon_cmd[i++] = RD(0x24) ;
    bt_txon_cmd[i++] = OR(12) ;
    bt_txon_cmd[i++] = WR(0x24) ;

    //wait 2us
    bt_txon_cmd[i++] = WAIT(2) ;

    //LDO_RBB
    bt_txon_cmd[i++] = RD(0x64) ;
    bt_txon_cmd[i++] = OR(13) ;
    bt_txon_cmd[i++] = WR(0x64) ;

    //RD FULCAL
    bt_txon_cmd[i++] = RD_FULCAL ;
    bt_txon_cmd[i++] = WR(0x40) ;
    bt_txon_cmd[i++] = WR(0x88) ;

    //EDR VCO3G_EN
    bt_txon_cmd[i++] = RD(0x24) ;
#if defined(ENABLE_EDR_5G)
    bt_txon_cmd[i++] = OR(11) ;
#elif defined(ENABLE_EDR_3G)
    bt_txon_cmd[i++] = OR(11) ;
#endif
    bt_txon_cmd[i++] = WR(0x24) ;

    //EDR PFDCP_EN
    bt_txon_cmd[i++] = RD(0x2C) ;
    bt_txon_cmd[i++] = OR(21) ;
#if defined(ENABLE_EDR_2G)
    bt_txon_cmd[i++] = OR(29) ;
#endif
    bt_txon_cmd[i++] = WR(0x2C) ;

    //EDR EN_LF
    bt_txon_cmd[i++] = RD(0x30) ;
    bt_txon_cmd[i++] = OR(26) ;
    bt_txon_cmd[i++] = WR(0x30) ;

    //EDR FBDV_EN
    bt_txon_cmd[i++] = RD(0x34) ;
    bt_txon_cmd[i++] = OR(28) ;
    bt_txon_cmd[i++] = WR(0x34) ;

    //EDR_EN_OSLO
    bt_txon_cmd[i++] = RD(0x44) ;
#ifdef ENABLE_EDR_3G
    bt_txon_cmd[i++] = OR(18) ;
#endif
    //EDR_EN_LODIST
    bt_txon_cmd[i++] = OR(6) ;
    bt_txon_cmd[i++] = WR(0x44) ;

    //EN_TBB_IARRY & EN_LDO_DAC_AVDD & EN_LDO_DAC_DVDD & EN_DAC
    bt_txon_cmd[i++] = RD(0x80) ;
    bt_txon_cmd[i++] = OR(8) ;
    bt_txon_cmd[i++] = OR(9) ;
    bt_txon_cmd[i++] = OR(10) ;
    bt_txon_cmd[i++] = OR(11) ;
    bt_txon_cmd[i++] = WR(0x80) ;

    //TRF_EDR_IARRAY_EN
    bt_txon_cmd[i++] = RD(0x58) ;
    bt_txon_cmd[i++] = OR(29) ;
    bt_txon_cmd[i++] = WR(0x58) ;

    //EDR_PACAP_EN & EDR_PA_XFMR_SG
    bt_txon_cmd[i++] = RD(0x5C) ;
    bt_txon_cmd[i++] = OR(11) ;
    bt_txon_cmd[i++] = OR(17) ;
    bt_txon_cmd[i++] = WR(0x5C) ;


    //RD DCCAL
    bt_txon_cmd[i++] = RD_DCCAL1 ;
    bt_txon_cmd[i++] = WR(0xC4) ;

    bt_txon_cmd[i++] = RD_DCCAL2 ;
    bt_txon_cmd[i++] = WR(0xC8) ;

    //wait 1us
    bt_txon_cmd[i++] = WAIT(1) ;

    //EDR_TMXBUF_PU EDR_TMX_PU
    //bt_txon_cmd[i++] = RD(0x58) ;
    //bt_txon_cmd[i++] = OR(17) ;
    //bt_txon_cmd[i++] = OR(28) ;
    //bt_txon_cmd[i++] = WR(0x58) ;

    //EDR_VCO_FLR_EN
    bt_txon_cmd[i++] = RD(0x24) ;
    bt_txon_cmd[i++] = OR(6) ;
    bt_txon_cmd[i++] = WR(0x24) ;

    //EDR_FBDV_RSTB
    bt_txon_cmd[i++] = RD(0x34) ;
    bt_txon_cmd[i++] = AND(11) ;
    bt_txon_cmd[i++] = WR(0x34) ;

    //EDR_TMXBUF_PU EDR_TMX_PU
    bt_txon_cmd[i++] = RD(0x58) ;
    bt_txon_cmd[i++] = OR(17) ;
    bt_txon_cmd[i++] = OR(28) ;
    bt_txon_cmd[i++] = WR(0x58) ;

    //wait 1us
    bt_txon_cmd[i++] = WAIT(1) ;
    /////////////////////////////////
    //        cmd for cal
    //RBB_REG5: EN_IARRAY
    bt_txon_cmd[i++] = RD(0x74) ;
    bt_txon_cmd[i++] = OR(5) ;
    bt_txon_cmd[i++] = WR(0x74) ;
    //en rvga_i EN_RVGA_I
    bt_txon_cmd[i++] = RD(0x68) ;
    bt_txon_cmd[i++] = OR(7) ;
    bt_txon_cmd[i++] = WR(0x68) ;
    //adc* ADC & LDO_ADC & LDO_ADCREF
    bt_txon_cmd[i++] = RD(0x7c) ;
    bt_txon_cmd[i++] = OR(4) ;
    bt_txon_cmd[i++] = OR(9) ;
    bt_txon_cmd[i++] = OR(21) ;
    bt_txon_cmd[i++] = WR(0x7c) ;
    //wait 5us
    bt_txon_cmd[i++] = WAIT(8) ;
    //pwrmtr_en
    bt_txon_cmd[i++] = RD(0x5C) ;
    bt_txon_cmd[i++] = OR(10) ;
    bt_txon_cmd[i++] = WR(0x5C) ;
    //wait 3us
    bt_txon_cmd[i++] = WAIT(5) ;
    //lpbk en
    bt_txon_cmd[i++] = RD(0x74) ;
    bt_txon_cmd[i++] = OR(0) ;
    bt_txon_cmd[i++] = WR(0x74) ;
    /////////////////////////////////


    //wait 30us for lo lock
    bt_txon_cmd[i++] = WAIT(45) ;
    //START INCCAL
    bt_txon_cmd[i++] = RD(0x90) ;   //inccal start
    bt_txon_cmd[i++] = OR(29) ;
    bt_txon_cmd[i++] = WR(0x90) ;
    bt_txon_cmd[i++] = WAIT(20) ;

    //DAC_START
    bt_txon_cmd[i++] = RD(0x80) ;
    bt_txon_cmd[i++] = OR(12) ;
    bt_txon_cmd[i++] = WR(0x80) ;

    //EDR_PA_PU
    bt_txon_cmd[i++] = RD(0x58) ;
    bt_txon_cmd[i++] = OR(7) ;
    bt_txon_cmd[i++] = WR(0x58) ;



    //END
    bt_txon_cmd[i++] = END ;
    if (i % 2)
    {
        bt_txon_cmd[i++] = END ;
    }
    HAL_ASSERT(i <= BT_CHANNEL_NUM);
    bt_txon_cmd_num = i ;
    //}}}

    //----------------BT_TXOFF CMD----------------------{{{
    i = 0;

    //EDR_PA_PD
    bt_txoff_cmd[i++] = RD(0x58) ;
    bt_txoff_cmd[i++] = RD(0x58) ;
    bt_txoff_cmd[i++] = AND(7) ;
    bt_txoff_cmd[i++] = WR(0x58) ;

    //DAC_STOP
    //EN_TBB_IARRY & EN_LDO_DAC_AVDD & EN_LDO_DAC_DVDD & EN_DAC
    bt_txoff_cmd[i++] = RD(0x80) ;
    bt_txoff_cmd[i++] = AND(8) ;
    bt_txoff_cmd[i++] = AND(9) ;
    bt_txoff_cmd[i++] = AND(10) ;
    bt_txoff_cmd[i++] = AND(11) ;
    bt_txoff_cmd[i++] = AND(12) ;
    bt_txoff_cmd[i++] = WR(0x80) ;

    //EDR_PA_PU
    //EDR_TMXBUF_PU EDR_TMX_PU
    bt_txoff_cmd[i++] = RD(0x58) ;
    bt_txoff_cmd[i++] = AND(7) ;
    bt_txoff_cmd[i++] = AND(17) ;
    bt_txoff_cmd[i++] = AND(28) ;
    bt_txoff_cmd[i++] = WR(0x58) ;

    //EDR_PACAP_EN & EDR_PA_XFMR_SG
    bt_txoff_cmd[i++] = RD(0x5c) ;
    bt_txoff_cmd[i++] = AND(11) ;
    bt_txoff_cmd[i++] = AND(17) ;
    bt_txoff_cmd[i++] = WR(0x5c) ;

    /////////////////////////////////
    //        cmd for cal
    //lpbk en
    bt_txoff_cmd[i++] = RD(0x74) ;
    bt_txoff_cmd[i++] = AND(0) ;
    bt_txoff_cmd[i++] = WR(0x74) ;
    //wait 1us
    bt_txoff_cmd[i++] = WAIT(2) ;
    //pwrmtr_en
    bt_txoff_cmd[i++] = RD(0x5C) ;
    bt_txoff_cmd[i++] = AND(10) ;
    bt_txoff_cmd[i++] = WR(0x5C) ;
    //wait 1us
    bt_txoff_cmd[i++] = WAIT(2) ;
    //en iarray EN_IARRAY
    bt_txoff_cmd[i++] = RD(0x74) ;
    bt_txoff_cmd[i++] = AND(5) ;
    bt_txoff_cmd[i++] = WR(0x74) ;
    //en rvga_i EN_RVGA_I
    bt_txoff_cmd[i++] = RD(0x68) ;
    bt_txoff_cmd[i++] = AND(7) ;
    bt_txoff_cmd[i++] = WR(0x68) ;
    //adc* ADC & LDO_ADC & LDO_ADCREF
    bt_txoff_cmd[i++] = RD(0x7c) ;
    bt_txoff_cmd[i++] = AND(4) ;
    bt_txoff_cmd[i++] = AND(9) ;
    bt_txoff_cmd[i++] = AND(21) ;
    bt_txoff_cmd[i++] = WR(0x7c) ;
    /////////////////////////////////

    //TRF_EDR_IARRAY_EN
    bt_txoff_cmd[i++] = RD(0x58) ;
    bt_txoff_cmd[i++] = AND(29) ;
    bt_txoff_cmd[i++] = WR(0x58) ;

    //EDR_EN_OSLO
    bt_txoff_cmd[i++] = RD(0x44) ;
    bt_txoff_cmd[i++] = AND(18) ;
    //EDR_EN_LODIST
    bt_txoff_cmd[i++] = AND(6) ;
    bt_txoff_cmd[i++] = WR(0x44) ;

    //EDR_VCO_FLR_EN
    bt_txoff_cmd[i++] = RD(0x24) ;
    bt_txoff_cmd[i++] = AND(6) ;
    bt_txoff_cmd[i++] = WR(0x24) ;

    //EDR_FBDV_RSTB
    bt_txoff_cmd[i++] = RD(0x34) ;
    bt_txoff_cmd[i++] = OR(11) ;
    bt_txoff_cmd[i++] = WR(0x34) ;


    //EDR PFDCP_EN
    bt_txoff_cmd[i++] = RD(0x2C) ;
    bt_txoff_cmd[i++] = AND(21) ;
    bt_txoff_cmd[i++] = AND(29) ;
    bt_txoff_cmd[i++] = WR(0x2C) ;

    //EDR EN_LF
    bt_txoff_cmd[i++] = RD(0x30) ;
    bt_txoff_cmd[i++] = AND(26) ;
    bt_txoff_cmd[i++] = WR(0x30) ;

    //EDR FBDV_EN
    bt_txoff_cmd[i++] = RD(0x34) ;
    bt_txoff_cmd[i++] = AND(28) ;
    bt_txoff_cmd[i++] = WR(0x34) ;
    bt_txoff_cmd[i++] = WAIT(10) ;
    //LDO_RBB
    bt_txoff_cmd[i++] = RD(0x64) ;
    bt_txoff_cmd[i++] = AND(13) ;
    bt_txoff_cmd[i++] = WR(0x64) ;

    //EDR_EN_IARY
    bt_txoff_cmd[i++] = RD(0x24) ;
    bt_txoff_cmd[i++] = AND(12) ;
    //bt_txoff_cmd[i++] = WR( 0x24 ) ;

    //EDR VCO3G_EN/EDR_VCO5G_EN
    //bt_txoff_cmd[i++] = RD( 0x24 ) ;
    //bt_txoff_cmd[i++] = AND( 12 ) ;
    bt_txoff_cmd[i++] = AND(11) ;
    bt_txoff_cmd[i++] = WR(0x24) ;
    //VDDPSW RFBG_EN
    //bt_txoff_cmd[i++] = RD( 0x10 ) ;
    bt_txoff_cmd[i++] = RD(0x10) ;
    bt_txoff_cmd[i++] = AND(8) ;
    bt_txoff_cmd[i++] = AND(9) ;
    bt_txoff_cmd[i++] = WR(0x10) ;

    //END
    bt_txoff_cmd[i++] = END ;
    if (i % 2)
    {
        bt_txoff_cmd[i++] = END ;
    }
    bt_txoff_cmd_num = i ;

    //}}}

    //RF_PRINTF("BLE rf rxon inccal init start\n");
    rxon_addr = reg_addr;
    hwp_bt_rfc->CU_ADDR_REG1 = 0;
    hwp_bt_rfc->CU_ADDR_REG1 = rxon_addr;
    for (i = 0; i < rxon_cmd_num / 2; i = i + 1)
    {
        //RF_PRINTF("cmd_addr = %x\n",rxon_addr );
        //RF_PRINTF("rxon_cmd[%d] = %x\n",i*2+1,rxon_cmd[i*2+1]);
        //RF_PRINTF("rxon_cmd[%d] = %x\n",i*2,rxon_cmd[i*2]);
        cmd = rxon_cmd[i * 2] + (rxon_cmd[i * 2 + 1] << 16) ;
        write_memory(BT_RFC_MEM_BASE + rxon_addr, cmd);
        rxon_addr += 4;
    }
    //RF_PRINTF("BLE rf rxoff inccal init start\n");
    //rxoff_addr = BT_RFC_BASE + 0x298;//0x41040198;
    rxoff_addr = rxon_addr + 4 ;
    hwp_bt_rfc->CU_ADDR_REG1 += (rxoff_addr << 16);
    for (i = 0; i < rxoff_cmd_num / 2; i = i + 1)
    {
        //RF_PRINTF("cmd_addr = %x\n",rxoff_addr );
        //RF_PRINTF("rxoff_cmd[%d] = %x\n",i*2+1,rxoff_cmd[i*2+1]);
        //RF_PRINTF("rxoff_cmd[%d] = %x\n",i*2,rxoff_cmd[i*2]);
        cmd = rxoff_cmd[i * 2] + (rxoff_cmd[i * 2 + 1] << 16) ;
        write_memory(BT_RFC_MEM_BASE + rxoff_addr, cmd);
        rxoff_addr += 4 ;
    }

    //txon_addr = BT_RFC_BASE + 0x304;//0x41040204;
    txon_addr = rxoff_addr + 0x4;
    hwp_bt_rfc->CU_ADDR_REG2 = 0;
    hwp_bt_rfc->CU_ADDR_REG2 = txon_addr ;
    for (i = 0; i < txon_cmd_num / 2; i = i + 1)
    {
        //RF_PRINTF("cmd_addr = %x\n",txon_addr );
        //RF_PRINTF("txon_cmd[%d] = %x\n",i*2+1,txon_cmd[i*2+1]);
        //RF_PRINTF("txon_cmd[%d] = %x\n",i*2,txon_cmd[i*2]);
        cmd = txon_cmd[i * 2] + (txon_cmd[i * 2 + 1] << 16) ;
        write_memory(BT_RFC_MEM_BASE + txon_addr, cmd);
        txon_addr += 4;
    }
    //RF_PRINTF("BLE rf rxoff inccal init start\n");
    txoff_addr = txon_addr + 0x4;//0x4104025C;
    hwp_bt_rfc->CU_ADDR_REG2 += (txoff_addr << 16);
    for (i = 0; i < txoff_cmd_num / 2; i = i + 1)
    {
        //RF_PRINTF("cmd_addr = %x\n",txoff_addr );
        //RF_PRINTF("txoff_cmd[%d] = %x\n",i*2+1,txoff_cmd[i*2+1]);
        //RF_PRINTF("txoff_cmd[%d] = %x\n",i*2,txoff_cmd[i*2]);
        cmd = txoff_cmd[i * 2] + (txoff_cmd[i * 2 + 1] << 16) ;
        write_memory(BT_RFC_MEM_BASE + txoff_addr, cmd);
        txoff_addr += 4 ;
    }

    bt_txon_addr = txoff_addr + 0x4;
    hwp_bt_rfc->CU_ADDR_REG3 = 0;
    hwp_bt_rfc->CU_ADDR_REG3 = bt_txon_addr ;
    for (i = 0; i < bt_txon_cmd_num / 2; i = i + 1)
    {
        //RF_PRINTF("cmd_addr = %x\n",txon_addr );
        //RF_PRINTF("txon_cmd[%d] = %x\n",i*2+1,txon_cmd[i*2+1]);
        //RF_PRINTF("txon_cmd[%d] = %x\n",i*2,txon_cmd[i*2]);
        cmd = bt_txon_cmd[i * 2] + (bt_txon_cmd[i * 2 + 1] << 16) ;
        write_memory(BT_RFC_MEM_BASE + bt_txon_addr, cmd);
        bt_txon_addr += 4;
    }
    //RF_PRINTF("BLE rf rxoff inccal init start\n");
    bt_txoff_addr = bt_txon_addr + 0x4;//0x4104025C;
    hwp_bt_rfc->CU_ADDR_REG3 += (bt_txoff_addr << 16);
    for (i = 0; i < bt_txoff_cmd_num / 2; i = i + 1)
    {
        //RF_PRINTF("cmd_addr = %x\n",txoff_addr );
        //RF_PRINTF("txoff_cmd[%d] = %x\n",i*2+1,txoff_cmd[i*2+1]);
        //RF_PRINTF("txoff_cmd[%d] = %x\n",i*2,txoff_cmd[i*2]);
        cmd = bt_txoff_cmd[i * 2] + (bt_txoff_cmd[i * 2 + 1] << 16) ;
        write_memory(BT_RFC_MEM_BASE + bt_txoff_addr, cmd);
        bt_txoff_addr += 4 ;
    }
    RF_PRINTF("cmd_addr = %x\n", bt_txoff_addr);


    free(rxon_cmd);
    free(rxoff_cmd);
    free(txoff_cmd);
    free(txon_cmd);
    free(bt_txon_cmd);
    free(bt_txoff_cmd);

    return bt_txoff_addr ;
}

#define MAX_LO_CAL_STEP 120
uint32_t bt_rfc_lo_cal(uint32_t rslt_start_addr)
{

    uint8_t  *idac_tbl = malloc(MAX_LO_CAL_STEP);
    uint8_t  *capcode_tbl = malloc(MAX_LO_CAL_STEP);
    uint32_t *residual_cnt_tbl = malloc(MAX_LO_CAL_STEP * 4);



    uint32_t *idac_tbl_tx = malloc(79 * 4);
    uint32_t *idac_tbl_rx_2m = malloc(40 * 4);
    uint32_t *idac_tbl_rx_1m = malloc(40 * 4);
    uint32_t *idac_tbl_rx_bt = malloc(79 * 4);

    uint32_t *capcode_tbl_tx = malloc(79 * 4);
    uint32_t *capcode_tbl_rx_2m = malloc(40 * 4);
    uint32_t *capcode_tbl_rx_1m = malloc(40 * 4);
    uint32_t *capcode_tbl_rx_bt = malloc(79 * 4);

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
    int      p_delta;
    uint32_t kcal_tbl[79];
    int      kcal_norm;

    uint32_t ConstA = 216 * (2048)   ;
    float    ConstB = 1.0 / 96800 ;

    uint32_t reg_data;
    uint32_t reg_addr =  rslt_start_addr;

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
    uint32_t     err_rx_bt = 0xffffffff;

    //RF_PRINTF("begin fulcal\n");


    //RF_PRINTF("begin LO cal\n");

    //hwp_pmuc->HXT_CR1 &= ~PMUC_HXT_CR1_CBANK_SEL_Msk;
    //hwp_pmuc->HXT_CR1 |= 0x1EA << PMUC_HXT_CR1_CBANK_SEL_Pos;
    //hwp_pmuc->HXT_CR1 |= 0xF << PMUC_HXT_CR1_LDO_VREF_Pos;

    hwp_bt_rfc->INCCAL_REG1 &= ~BT_RFC_INCCAL_REG1_INCACAL_EN ;
    hwp_bt_rfc->INCCAL_REG1 &= ~BT_RFC_INCCAL_REG1_INCFCAL_EN ;
    hwp_bt_rfc->MISC_CTRL_REG |= BT_RFC_MISC_CTRL_REG_IDAC_FORCE_EN  | BT_RFC_MISC_CTRL_REG_PDX_FORCE_EN | BT_RFC_MISC_CTRL_REG_EN_2M_MOD_FRC_EN;
    hwp_bt_rfc->RF_LODIST_REG |= BT_RFC_RF_LODIST_REG_BRF_EN_RFBG_LV | BT_RFC_RF_LODIST_REG_BRF_EN_VDDPSW_LV ;

    hwp_bt_rfc->DCO_REG2 &= ~BT_RFC_DCO_REG2_BRF_VCO_ACAL_VL_SEL_LV_Msk;
    hwp_bt_rfc->DCO_REG2 |= 0x7 << BT_RFC_DCO_REG2_BRF_VCO_ACAL_VL_SEL_LV_Pos;
    //LO full ACAL
    //RF_PRINTF("begin LO acal\n");
    hwp_bt_rfc->DCO_REG1 |= BT_RFC_DCO_REG1_BRF_VCO_EN_LV ;
    hwp_bt_rfc->DCO_REG2 |= BT_RFC_DCO_REG2_BRF_VCO_ACAL_EN_LV;
    hwp_bt_rfc->DCO_REG2 |= BT_RFC_DCO_REG2_BRF_VCO_FKCAL_EN_LV;
    //VCO_ACAL_SEL ??
    hwp_bt_rfc->LPF_REG  |= BT_RFC_LPF_REG_BRF_LO_OPEN_LV;
    hwp_bt_phy->TX_HFP_CFG &= ~(BT_PHY_TX_HFP_CFG_HFP_FCW_Msk);
    hwp_bt_phy->TX_HFP_CFG |= (0x07 << BT_PHY_TX_HFP_CFG_HFP_FCW_Pos);

    hwp_bt_rfc->DCO_REG1 |= BT_RFC_DCO_REG1_BRF_EN_2M_MOD_LV;

    hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Msk;
    hwp_bt_rfc->DCO_REG3 |= (0x40) << BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos;


    //LO full fcal
    //RF_PRINTF("begin LO fcal\n");
    fcal_cnt    = 0x80;
    fcal_cnt_fs = 0x80;
    hwp_bt_rfc->FBDV_REG1 |= BT_RFC_FBDV_REG1_BRF_FBDV_EN_LV ;
    hwp_bt_rfc->DCO_REG2 |= BT_RFC_DCO_REG2_BRF_VCO_FKCAL_EN_LV;
    hwp_bt_rfc->FBDV_REG2 &= ~BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_DIVN_LV ;
    hwp_bt_rfc->FBDV_REG2 |= 7680 << BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_DIVN_LV_Pos;
    //set lfp_fcw
    hwp_bt_phy->TX_LFP_CFG &= ~(BT_PHY_TX_LFP_CFG_LFP_FCW_Msk);
    hwp_bt_phy->TX_LFP_CFG |= (0x08 << BT_PHY_TX_LFP_CFG_LFP_FCW_Pos);
    hwp_bt_phy->TX_LFP_CFG    &= (~BT_PHY_TX_LFP_CFG_LFP_FCW_SEL);

    hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Msk;
    hwp_bt_rfc->DCO_REG3 |= (0x80) << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos;
    hwp_bt_phy->TX_HFP_CFG &= ~(BT_PHY_TX_HFP_CFG_HFP_FCW_Msk);
    hwp_bt_phy->TX_HFP_CFG |= (0x07 << BT_PHY_TX_HFP_CFG_HFP_FCW_Pos);
    hwp_bt_phy->TX_HFP_CFG &=  ~BT_PHY_TX_HFP_CFG_HFP_FCW_SEL;
    hwp_bt_rfc->DCO_REG1 |= BT_RFC_DCO_REG1_BRF_EN_2M_MOD_LV;

    hwp_bt_rfc->FBDV_REG1 |= BT_RFC_FBDV_REG1_BRF_FBDV_RSTB_LV ;
    hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FBDV_RSTB_LV ;

    hwp_bt_rfc->FBDV_REG1 &=  ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV_Msk;
    hwp_bt_rfc->FBDV_REG1 |=  BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV;

    hwp_bt_rfc->MISC_CTRL_REG |= BT_RFC_MISC_CTRL_REG_XTAL_REF_EN | BT_RFC_MISC_CTRL_REG_XTAL_REF_EN_FRC_EN ;
    hwp_bt_rfc->PFDCP_REG |= BT_RFC_PFDCP_REG_BRF_PFDCP_EN_LV ;


    hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Msk;
    hwp_bt_rfc->DCO_REG3 |= (0x40) << BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos;

    //pdx binary search
    //should store the cnt value of last pdx, so loop 8 times
    for (i = 1; i < 9; i++)
    {
        //RF_PRINTF("begin LO fcal binary search\n");
        //--------full acal in full fcal --------
        //{{{
        acal_cnt    = 0x40;
        acal_cnt_fs = 0x40;

        hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Msk;
        hwp_bt_rfc->DCO_REG3 |= (acal_cnt) << BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos;
        //wait 4us
        _HAL_Delay_us(4);
        hwp_bt_rfc->DCO_REG2 |= BT_RFC_DCO_REG2_BRF_VCO_ACAL_EN_LV;

        //acal binary search
        for (j = 1; j < 7; j++)
        {
            //
            if (!(hwp_bt_rfc->DCO_REG2 & BT_RFC_DCO_REG2_BRF_VCO_ACAL_INCAL_LV_Msk))
                break;
            else if (!(hwp_bt_rfc->DCO_REG2 & BT_RFC_DCO_REG2_BRF_VCO_ACAL_UP_LV_Msk))
                acal_cnt = acal_cnt - (acal_cnt_fs >> j) ;
            else  if (hwp_bt_rfc->DCO_REG2 & BT_RFC_DCO_REG2_BRF_VCO_ACAL_UP_LV_Msk)
                acal_cnt = acal_cnt + (acal_cnt_fs >> j) ;
            hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Msk;
            hwp_bt_rfc->DCO_REG3 |= (acal_cnt) << BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos;
            //wait 1us
            _HAL_Delay_us(1);

        }
        hwp_bt_rfc->DCO_REG2 &= ~BT_RFC_DCO_REG2_BRF_VCO_ACAL_EN_LV;
        //}}}

        _HAL_Delay_us(4);
        hwp_bt_rfc->FBDV_REG1 &=  ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;
        hwp_bt_rfc->FBDV_REG1 &=  ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV_Msk;
        hwp_bt_rfc->FBDV_REG1 |=  BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV;
        hwp_bt_rfc->FBDV_REG1 |= 1 << BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV_Pos;

        _HAL_Delay_us(10);

        while (!(hwp_bt_rfc->FBDV_REG1 &  BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RDY_LV));   //TODO: add timeout
        residual_cnt  =  hwp_bt_rfc->FBDV_REG2 & BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Msk ;
        residual_cnt  = residual_cnt >> BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;

        //RF_PRINTF( "residual_cnt = %d,cnt_vth = %d\n",residual_cnt,residual_cnt_vth );
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
        //RF_PRINTF( "fcal bin fcal_cnt = %x,acal_cnt = %x\n",fcal_cnt,acal_cnt );
        hwp_bt_rfc->FBDV_REG1 &=  ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;
        hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Msk;
        hwp_bt_rfc->DCO_REG3 |= (fcal_cnt) << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos;
    }

    //RF_PRINTF( "sweep start idac0 = %x,capcode0 = %x\n",idac0,capcode0 );
    //RF_PRINTF( "sweep start idac1 = %x,capcode1 = %x\n",idac1,capcode1 );
    //RF_PRINTF( "sweep start error0 = %x,error1 = %x\n",error0,error1 );
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

    hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Msk;
    hwp_bt_rfc->DCO_REG3 |= (fcal_cnt) << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos;
    hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;

    fcal_cnt = capcode_tbl[0] ;
    acal_cnt = idac_tbl[0] ;
    //RF_PRINTF( "sweep start fcal_cnt = %x,acal_cnt = %x\n",fcal_cnt,acal_cnt );
    //sweep pdx until 4.8G
    i = 0;
    //hwp_bt_rfc->RSVD_REG2 = 0 ;

    do
    {
        //hwp_bt_rfc->RSVD_REG2 +=  1 ;//DEBUG
        //hwp_bt_rfc->RSVD_REG1  =  0 ;//DEBUG
        i                     +=  1 ;
        fcal_cnt              +=  1 ;
        seq_acal_jump_cnt      =  0 ;
        seq_acal_ful_cnt       =  0 ;
        pre_acal_up_vld        =  0 ;
        hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Msk;
        hwp_bt_rfc->DCO_REG3 |= (fcal_cnt) << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos;
        //seq acal {{{
        hwp_bt_rfc->DCO_REG2 |= BT_RFC_DCO_REG2_BRF_VCO_ACAL_EN_LV;
        //VCO_ACAL_SEL ??
        hwp_bt_rfc->LPF_REG  |= BT_RFC_LPF_REG_BRF_LO_OPEN_LV;
        while ((seq_acal_jump_cnt < 4) & (seq_acal_ful_cnt < 2))
        {
            //hwp_bt_rfc->RSVD_REG1 +=  1 ;//DEBUG
            hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Msk;
            hwp_bt_rfc->DCO_REG3 |= (acal_cnt) << BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos;
            //wait for 4us
            _HAL_Delay_us(4);
            //for(int j=0;j<100;j++)
            //RF_PRINTF( "wait idac settling\n" );

            if (!(hwp_bt_rfc->DCO_REG2 & BT_RFC_DCO_REG2_BRF_VCO_ACAL_INCAL_LV_Msk))
                break;
            curr_acal_up = hwp_bt_rfc->DCO_REG2 & BT_RFC_DCO_REG2_BRF_VCO_ACAL_UP_LV_Msk;
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
        hwp_bt_rfc->DCO_REG2 &= ~BT_RFC_DCO_REG2_BRF_VCO_ACAL_EN_LV;
        ///}}}

        hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Msk;
        hwp_bt_rfc->DCO_REG3 |= (acal_cnt) << BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos;
        //wait for 4us
        _HAL_Delay_us(4);
        //for(int j=0;j<100;j++)
        //RF_PRINTF( "wait idac settling\n" );
        hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;

        hwp_bt_rfc->FBDV_REG1 &=  ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV_Msk;
        hwp_bt_rfc->FBDV_REG1 |=  BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV;
        hwp_bt_rfc->FBDV_REG1 |= 1 << BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV_Pos;
        _HAL_Delay_us(10);
        //for( j=0;j<100;j++)
        //RF_PRINTF( "wait idac settling\n" );

        while (!(hwp_bt_rfc->FBDV_REG1 & BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RDY_LV));

        residual_cnt  = hwp_bt_rfc->FBDV_REG2 & BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Msk ;
        //residual_cnt  = residual_cnt >> BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;
        //RF_PRINTF( "residual_cnt = %d, residual_cnt_vtl=%d\n", residual_cnt, residual_cnt_vtl );

        if (residual_cnt <= residual_cnt_vtl)
            break;

        HAL_ASSERT(i < MAX_LO_CAL_STEP);
        idac_tbl[i]         = acal_cnt ;
        capcode_tbl[i]      = fcal_cnt ;
        residual_cnt_tbl[i] = residual_cnt ;
    }
    while (residual_cnt > residual_cnt_vtl) ;

    hwp_bt_rfc->DCO_REG2 &= ~BT_RFC_DCO_REG2_BRF_VCO_FKCAL_EN_LV;

    //search result for each channel
    sweep_num   = i ;
    //hwp_bt_rfc->RSVD_REG2 =  10 ;//DEBUG
    //for bt 40 chnl
    for (j = 0; j < 40; j++)
    {
        //hwp_bt_rfc->RSVD_REG2 +=  1 ;//DEBUG
        //hwp_bt_rfc->RSVD_REG1 =  100 ;//DEBUG
        err_tx    = 0 ;
        err_rx_1m = 0 ;
        err_rx_2m = 0 ;
        for (i = 0; i < sweep_num; i++)
        {
            //hwp_bt_rfc->RSVD_REG1 +=  1 ;//DEBUG
            if (i == 0)
            {
                if (ref_residual_cnt_tbl_rx_1m[j] > residual_cnt_tbl[i])
                {
                    err_rx_1m  =  ref_residual_cnt_tbl_rx_1m[j] - residual_cnt_tbl[i] ;
                }
                else
                {
                    err_rx_1m  =  residual_cnt_tbl[i] - ref_residual_cnt_tbl_rx_1m[j] ;
                }

                if (ref_residual_cnt_tbl_rx_2m[j] > residual_cnt_tbl[i])
                {
                    err_rx_2m  =  ref_residual_cnt_tbl_rx_2m[j] - residual_cnt_tbl[i] ;
                }
                else
                {
                    err_rx_2m  =  residual_cnt_tbl[i] - ref_residual_cnt_tbl_rx_2m[j] ;
                }

                idac_tbl_rx_1m[j]    =  idac_tbl[i];
                idac_tbl_rx_2m[j]    =  idac_tbl[i];
                capcode_tbl_rx_1m[j] =  capcode_tbl[i];
                capcode_tbl_rx_2m[j] =  capcode_tbl[i];

            }
            else
            {
                if (ref_residual_cnt_tbl_rx_1m[j] > residual_cnt_tbl[i])
                {
                    error0     =  ref_residual_cnt_tbl_rx_1m[j] - residual_cnt_tbl[i] ;
                    if (error0 < err_rx_1m)
                    {
                        err_rx_1m            = error0;
                        idac_tbl_rx_1m[j]    =  idac_tbl[i];
                        capcode_tbl_rx_1m[j] =  capcode_tbl[i];
                    }
                }
                else
                {
                    error0     =  residual_cnt_tbl[i] - ref_residual_cnt_tbl_rx_1m[j] ;
                    if (error0 < err_rx_1m)
                    {
                        err_rx_1m            = error0;
                        idac_tbl_rx_1m[j]    =  idac_tbl[i];
                        capcode_tbl_rx_1m[j] =  capcode_tbl[i];
                    }
                }

                if (ref_residual_cnt_tbl_rx_2m[j] > residual_cnt_tbl[i])
                {
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
                    error0     = residual_cnt_tbl[i]  - ref_residual_cnt_tbl_rx_2m[j] ;
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

    //for bt 79 chnl
    for (j = 0; j < 79; j++)
    {
        err_tx    = 0 ;
        err_rx_bt = 0 ;
        for (i = 0; i < sweep_num; i++)
        {
            //hwp_bt_rfc->RSVD_REG1 +=  1 ;//DEBUG
            if (i == 0)
            {
                if (ref_residual_cnt_tbl_tx[j] > residual_cnt_tbl[i])
                {
                    err_tx     =  ref_residual_cnt_tbl_tx[j]    - residual_cnt_tbl[i] ;
                }
                else
                {
                    err_tx     =  residual_cnt_tbl[i] - ref_residual_cnt_tbl_tx[j]    ;
                }
                if (ref_residual_cnt_tbl_rx_bt[j] > residual_cnt_tbl[i])
                {
                    err_rx_bt  =  ref_residual_cnt_tbl_rx_bt[j] - residual_cnt_tbl[i] ;
                }
                else
                {
                    err_rx_bt  =  residual_cnt_tbl[i] - ref_residual_cnt_tbl_rx_bt[j] ;
                }
                idac_tbl_tx[j]       =  idac_tbl[i];
                idac_tbl_rx_bt[j]    =  idac_tbl[i];
                capcode_tbl_tx[j]    =  capcode_tbl[i];
                capcode_tbl_rx_bt[j] =  capcode_tbl[i];
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
                }

                if (ref_residual_cnt_tbl_rx_bt[j] > residual_cnt_tbl[i])
                {
                    error0     =  ref_residual_cnt_tbl_rx_bt[j]    - residual_cnt_tbl[i] ;
                    if (error0 < err_rx_bt)
                    {
                        err_rx_bt = error0;
                        idac_tbl_rx_bt[j]       =  idac_tbl[i];
                        capcode_tbl_rx_bt[j]    =  capcode_tbl[i];
                    }
                }
                else
                {
                    error0     = residual_cnt_tbl[i] - ref_residual_cnt_tbl_rx_bt[j] ;
                    if (error0 < err_rx_bt)
                    {
                        err_rx_bt = error0;
                        idac_tbl_rx_bt[j]       =  idac_tbl[i];
                        capcode_tbl_rx_bt[j]    =  capcode_tbl[i];
                    }
                }

            }
        }
    }

    //------------kcal -----------
    //{{{
    //hwp_bt_rfc->RSVD_REG2 =  100 ;//DEBUG
    //RF_PRINTF("begin LO 0-39th ch kcal\n");
    hwp_bt_rfc->DCO_REG2    |= BT_RFC_DCO_REG2_BRF_VCO_FKCAL_EN_LV;
    hwp_bt_rfc->FBDV_REG2   &= ~BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_DIVN_LV ;
    hwp_bt_rfc->FBDV_REG2   |= 17280 << BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_DIVN_LV_Pos;
    hwp_bt_phy->TX_LFP_CFG &= ~(BT_PHY_TX_LFP_CFG_LFP_FCW_Msk);
    hwp_bt_phy->TX_LFP_CFG |= (0x08 << BT_PHY_TX_LFP_CFG_LFP_FCW_Pos);
    hwp_bt_phy->TX_LFP_CFG &=  ~BT_PHY_TX_LFP_CFG_LFP_FCW_SEL;
    //replace with 20th chnl pdx
    hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Msk;
    hwp_bt_rfc->DCO_REG3 |= (capcode_tbl_tx[19]) << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos;
    hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Msk;
    hwp_bt_rfc->DCO_REG3 |= (idac_tbl_tx[19]) << BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos;
    hwp_bt_phy->TX_HFP_CFG &= ~(BT_PHY_TX_HFP_CFG_HFP_FCW_Msk);
    hwp_bt_phy->TX_HFP_CFG |= (0x00 << BT_PHY_TX_HFP_CFG_HFP_FCW_Pos);
    hwp_bt_phy->TX_HFP_CFG &=  ~BT_PHY_TX_HFP_CFG_HFP_FCW_SEL;

    //wait 4us
    _HAL_Delay_us(4);
    hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;
    hwp_bt_rfc->FBDV_REG1 &=  ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV_Msk;
    hwp_bt_rfc->FBDV_REG1 |=  BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV;
    hwp_bt_rfc->FBDV_REG1 |= 1 << BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV_Pos;
    _HAL_Delay_us(10);
    while (!(hwp_bt_rfc->FBDV_REG1 &  BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RDY_LV));
    pmin = hwp_bt_rfc->FBDV_REG2 & BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Msk ;
    //pmin = pmin >> BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;
    hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;


    hwp_bt_phy->TX_HFP_CFG &= ~(BT_PHY_TX_HFP_CFG_HFP_FCW_Msk);
    hwp_bt_phy->TX_HFP_CFG |= (0x3f << BT_PHY_TX_HFP_CFG_HFP_FCW_Pos);
    //wait 4us
    _HAL_Delay_us(4);
    hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;
    hwp_bt_rfc->FBDV_REG1 &=  ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV_Msk;
    hwp_bt_rfc->FBDV_REG1 |=  BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV;
    hwp_bt_rfc->FBDV_REG1 |= 1 << BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV_Pos;
    _HAL_Delay_us(10);
    while (!(hwp_bt_rfc->FBDV_REG1 &  BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RDY_LV));
    pmax = hwp_bt_rfc->FBDV_REG2 & BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Msk ;
    pmax = pmax >> BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;

    hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;
    hwp_bt_rfc->DCO_REG2  &= ~BT_RFC_DCO_REG2_BRF_VCO_FKCAL_EN_LV;
    //}}}

    kcal_norm = ConstA / (pmax - pmin) ;
    RF_PRINTF("kcal_norm = %d,pmin = %d, pmax=%d\n", kcal_norm, pmin, pmax);

    for (i = 0; i < 40; i++)
    {
        //hwp_bt_rfc->RSVD_REG2 +=  1 ;//DEBUG
        p_delta = (ref_residual_cnt_tbl_tx[i] - ref_residual_cnt_tbl_tx[19]) ;
        kcal_tbl[i] = kcal_norm * (1.0 - 3 * p_delta * ConstB);
        //RF_PRINTF( "kcal[%d] = %x\n",i,kcal_tbl[i] );//DEBUG
    }

    ConstA = 216 * (2048) ;
    ConstB = 1.0 / 98400;
    RF_PRINTF("begin LO 40-79th ch kcal\n");
    //hwp_bt_rfc->RSVD_REG2 =  1000 ;//DEBUG
    hwp_bt_rfc->DCO_REG2    |= BT_RFC_DCO_REG2_BRF_VCO_FKCAL_EN_LV;
    hwp_bt_rfc->FBDV_REG2   &= ~BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_DIVN_LV ;
    hwp_bt_rfc->FBDV_REG2   |= 17280 << BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_DIVN_LV_Pos;
    hwp_bt_phy->TX_LFP_CFG &= ~(BT_PHY_TX_LFP_CFG_LFP_FCW_Msk);
    hwp_bt_phy->TX_LFP_CFG |= (0x08 << BT_PHY_TX_LFP_CFG_LFP_FCW_Pos);
    hwp_bt_phy->TX_LFP_CFG &=  ~BT_PHY_TX_LFP_CFG_LFP_FCW_SEL;
    // replace with 60th chnl pdx
    hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Msk;
    hwp_bt_rfc->DCO_REG3 |= (capcode_tbl_tx[59]) << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos;
    hwp_bt_rfc->DCO_REG3 &= ~BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Msk;
    hwp_bt_rfc->DCO_REG3 |= (idac_tbl_tx[59]) << BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos;
    hwp_bt_phy->TX_HFP_CFG &= ~(BT_PHY_TX_HFP_CFG_HFP_FCW_Msk);
    hwp_bt_phy->TX_HFP_CFG |= (0x00 << BT_PHY_TX_HFP_CFG_HFP_FCW_Pos);
    hwp_bt_phy->TX_HFP_CFG &=  ~BT_PHY_TX_HFP_CFG_HFP_FCW_SEL;
    //wait 4us
    _HAL_Delay_us(4);
    hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;
    hwp_bt_rfc->FBDV_REG1 &=  ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV_Msk;
    hwp_bt_rfc->FBDV_REG1 |=  BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV;
    hwp_bt_rfc->FBDV_REG1 |= 1 << BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV_Pos;
    _HAL_Delay_us(10);

    while (!(hwp_bt_rfc->FBDV_REG1 &  BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RDY_LV));
    pmin = hwp_bt_rfc->FBDV_REG2 & BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Msk ;
    pmin = pmin >> BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;
    hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;


    hwp_bt_phy->TX_HFP_CFG &= ~(BT_PHY_TX_HFP_CFG_HFP_FCW_Msk);
    hwp_bt_phy->TX_HFP_CFG |= (0x3F << BT_PHY_TX_HFP_CFG_HFP_FCW_Pos);
    //wait 4us
    _HAL_Delay_us(4);
    hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;
    hwp_bt_rfc->FBDV_REG1 &=  ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV_Msk;
    hwp_bt_rfc->FBDV_REG1 |=  BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV;
    hwp_bt_rfc->FBDV_REG1 |= 1 << BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV_Pos;
    _HAL_Delay_us(10);

    while (!(hwp_bt_rfc->FBDV_REG1 &  BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RDY_LV));  //timeout needed
    pmax = hwp_bt_rfc->FBDV_REG2 & BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Msk ;
    pmax = pmax >> BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;

    hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;
    //}}}

    kcal_norm = ConstA / (pmax - pmin) ;
    //RF_PRINTF("kcal_norm = %d,pmin = %d, pmax=%d\n", kcal_norm,pmin,pmax  );

    for (i = 40; i < 79; i++)
    {
        //hwp_bt_rfc->RSVD_REG2 +=  1 ;//DEBUG
        p_delta = (ref_residual_cnt_tbl_tx[i] - ref_residual_cnt_tbl_tx[59]) ;
        kcal_tbl[i] = kcal_norm * (1 - 3 * p_delta * ConstB);
        //RF_PRINTF( "kcal[%d] = %x\n",i,kcal_tbl[i] );//DEBUG
    }

    hwp_bt_rfc->PFDCP_REG &= ~BT_RFC_PFDCP_REG_BRF_PFDCP_EN_LV_Msk ;

    //write to rf_mem

    //store ble rx cal result
    reg_addr = rslt_start_addr;
    reg_data = 0;
    hwp_bt_rfc->CAL_ADDR_REG1 = 0;
    hwp_bt_rfc->CAL_ADDR_REG1 = reg_addr;
    for (i = 0; i < 40; i++)
    {
        //store rx fulcal result
        reg_data  = (((capcode_tbl_rx_2m[i] << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos) + (idac_tbl_rx_2m[i] << BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos)) << 16) + \
                    (capcode_tbl_rx_1m[i] << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos) + (idac_tbl_rx_1m[i] << BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos);
        write_memory(reg_addr + BT_RFC_MEM_BASE, reg_data);
        reg_addr += 4;
    }

    //store bt rx cal result
    hwp_bt_rfc->CAL_ADDR_REG1 += reg_addr << 16;
    for (i = 0; i < 40; i++)
    {
        //store btrx fulcal result
        reg_data  = (((capcode_tbl_rx_bt[2 * i + 1] << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos) + (idac_tbl_rx_bt[2 * i + 1] << BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos)) << 16) + \
                    (capcode_tbl_rx_bt[2 * i]   << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos) + (idac_tbl_rx_bt[2 * i]  << BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos);
        write_memory(reg_addr + BT_RFC_MEM_BASE, reg_data);
        reg_addr += 4;
    }

    //store ble tx cal result
    hwp_bt_rfc->CAL_ADDR_REG2 = 0;
    hwp_bt_rfc->CAL_ADDR_REG2 = reg_addr;
    for (i = 0; i < 79; i++)
    {
        //store tx cal result
        reg_data = (kcal_tbl[i] << BT_RFC_DCO_REG3_TX_KCAL_Pos) + (idac_tbl_tx[i] << BT_RFC_DCO_REG3_BRF_VCO_IDAC_LV_Pos) + (capcode_tbl_tx[i] << BT_RFC_DCO_REG3_BRF_VCO_PDX_LV_Pos) ;
        write_memory(reg_addr + BT_RFC_MEM_BASE, reg_data);
        reg_addr += 4;
    }


    RF_PRINTF("begin pacal\n");
    //PACAL {{{
    uint8_t setbc_rslt;
    uint8_t setsgn_rslt;
    //LO lock setting
    //debug pacal
    //hwp_tsen->ANAU_ANA_TP &= ~TSEN_ANAU_ANA_TP_CHIP_DC_TE_Msk;
    //hwp_tsen->ANAU_ANA_TP &= ~TSEN_ANAU_ANA_TP_CHIP_DC_UR_Msk;
    //hwp_tsen->ANAU_ANA_TP |= 1<<TSEN_ANAU_ANA_TP_CHIP_DC_TE_Pos;
    //hwp_tsen->ANAU_ANA_TP |= 5<<TSEN_ANAU_ANA_TP_CHIP_DC_UR_Pos;
    //hwp_bt_rfc->ATEST_REG = 0xB2;
    //hwp_pinmux2->PAD_PB05 = 0xA;
    //debug pacal

    hwp_bt_rfc->TRF_REG1 &= ~BT_RFC_TRF_REG1_BRF_TRF_LDO_VREF_SEL_LV_Msk;
    hwp_bt_rfc->TRF_REG1 |= 0x0E << BT_RFC_TRF_REG1_BRF_TRF_LDO_VREF_SEL_LV_Pos;

    hwp_bt_rfc->RF_LODIST_REG |= BT_RFC_RF_LODIST_REG_BRF_LODIST_TX_EN_LV;
    hwp_bt_rfc->TRF_REG2 |= BT_RFC_TRF_REG2_BRF_PA_UNIT_SEL_LV ; //for 4dbm power mode
    hwp_bt_rfc->TRF_REG1 |= BT_RFC_TRF_REG1_BRF_PA_BUF_PU_LV | BT_RFC_TRF_REG1_BRF_TRF_SIG_EN_LV;
    hwp_bt_rfc->TRF_REG1 |= BT_RFC_TRF_REG1_BRF_PA_BCSEL_LV;
    hwp_bt_rfc->TRF_REG2 |= BT_RFC_TRF_REG2_BRF_PA_BUFLOAD_SEL_LV;
    hwp_bt_rfc->PACAL_REG |= BT_RFC_PACAL_REG_PA_RSTB_FRC_EN | BT_RFC_PACAL_REG_PACAL_CLK_EN;

    //delay_20_us
    _HAL_Delay_us(20);
    hwp_bt_rfc->TRF_REG1  &= ~BT_RFC_TRF_REG1_BRF_PA_RSTN_LV ;
    //delay_2_us
    _HAL_Delay_us(2);
    hwp_bt_rfc->TRF_REG1  |= BT_RFC_TRF_REG1_BRF_PA_RSTN_LV ;
    //delay_20_us
    _HAL_Delay_us(2);
    //hwp_bt_rfc->PACAL_REG &= ~BT_RFC_PACAL_REG_PA_RSTB_FRC_EN;


    hwp_bt_rfc->PACAL_REG &= (~BT_RFC_PACAL_REG_PACAL_START);
    hwp_bt_rfc->PACAL_REG |= (BT_RFC_PACAL_REG_PACAL_START);

    while (!(hwp_bt_rfc->PACAL_REG & BT_RFC_PACAL_REG_PACAL_DONE));  //TODO:timeout needed
    hwp_bt_rfc->TRF_REG1 &= ~BT_RFC_TRF_REG1_BRF_PA_SETBC_LV;
    hwp_bt_rfc->TRF_REG1 &= ~BT_RFC_TRF_REG1_BRF_PA_SETSGN_LV;
    setbc_rslt  = (hwp_bt_rfc->PACAL_REG & BT_RFC_PACAL_REG_BC_CAL_RSLT_Msk) >> BT_RFC_PACAL_REG_BC_CAL_RSLT_Pos;
    setsgn_rslt = (hwp_bt_rfc->PACAL_REG & BT_RFC_PACAL_REG_SGN_CAL_RSLT_Msk) >> BT_RFC_PACAL_REG_SGN_CAL_RSLT_Pos;
    hwp_bt_rfc->TRF_REG1 |= setbc_rslt << BT_RFC_TRF_REG1_BRF_PA_SETBC_LV_Pos;
    hwp_bt_rfc->TRF_REG1 |= setsgn_rslt << BT_RFC_TRF_REG1_BRF_PA_SETSGN_LV_Pos;

    //hwp_bt_rfc->TRF_REG2 &= ~BT_RFC_TRF_REG2_BRF_PA_BUFLOAD_SEL_LV_Msk;
    hwp_bt_rfc->PACAL_REG &= (~BT_RFC_PACAL_REG_PACAL_START);
    hwp_bt_rfc->TRF_REG1 &= ~BT_RFC_TRF_REG1_BRF_PA_BCSEL_LV ;
    hwp_bt_rfc->TRF_REG1 &= ~BT_RFC_TRF_REG1_BRF_PA_BUF_PU_LV;
    hwp_bt_rfc->PACAL_REG  &= ~BT_RFC_PACAL_REG_PACAL_CLK_EN;
    hwp_bt_rfc->RF_LODIST_REG &= ~BT_RFC_RF_LODIST_REG_BRF_LODIST_TX_EN_LV;
    //}}}

    //after RF calibration
    hwp_bt_rfc->MISC_CTRL_REG &= ~BT_RFC_MISC_CTRL_REG_XTAL_REF_EN_FRC_EN ;
    hwp_bt_rfc->INCCAL_REG1   |= BT_RFC_INCCAL_REG1_INCACAL_EN ;
    hwp_bt_rfc->INCCAL_REG1   |= BT_RFC_INCCAL_REG1_INCFCAL_EN ;
    hwp_bt_rfc->LPF_REG       &= ~BT_RFC_LPF_REG_BRF_LO_OPEN_LV_Msk;
    hwp_bt_rfc->DCO_REG2      &= ~BT_RFC_DCO_REG2_BRF_VCO_FKCAL_EN_LV;
    hwp_bt_phy->TX_LFP_CFG   |=  BT_PHY_TX_LFP_CFG_LFP_FCW_SEL;
    hwp_bt_phy->TX_HFP_CFG   |=  BT_PHY_TX_HFP_CFG_HFP_FCW_SEL;
    hwp_bt_rfc->DCO_REG1      &= ~BT_RFC_DCO_REG1_BRF_EN_2M_MOD_LV;
    hwp_bt_rfc->MISC_CTRL_REG &= ~BT_RFC_MISC_CTRL_REG_IDAC_FORCE_EN ;
    hwp_bt_rfc->MISC_CTRL_REG &= ~BT_RFC_MISC_CTRL_REG_PDX_FORCE_EN;
    hwp_bt_rfc->MISC_CTRL_REG &= ~BT_RFC_MISC_CTRL_REG_EN_2M_MOD_FRC_EN;
    hwp_bt_rfc->TRF_REG1      |= BT_RFC_TRF_REG1_BRF_PA_CAS_BP_LV;
    hwp_bt_rfc->TRF_REG2      |= BT_RFC_TRF_REG2_BRF_PA_MCAP_LV;
    hwp_bt_rfc->FBDV_REG1     |=  BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_RSTB_LV_Msk;
    hwp_bt_rfc->ADC_REG       |= BT_RFC_ADC_REG_BRF_RSTB_ADC_LV ;
    hwp_bt_rfc->DCO_REG1      &= ~BT_RFC_DCO_REG1_BRF_VCO_EN_LV ;
    hwp_bt_rfc->FBDV_REG1     &= ~BT_RFC_FBDV_REG1_BRF_FBDV_EN_LV ;


    //RF_PRINTF("begin roscal\n");
    //ROSCAL{{{
    hwp_bt_rfc->RBB_REG1 |= BT_RFC_RBB_REG1_BRF_EN_LDO_RBB_LV;
    hwp_bt_rfc->RBB_REG5 |= BT_RFC_RBB_REG5_BRF_EN_IARRAY_LV;

    //enabt adc
    hwp_bt_rfc->MISC_CTRL_REG |= BT_RFC_MISC_CTRL_REG_ADC_CLK_EN_FRC_EN;
    hwp_bt_rfc->MISC_CTRL_REG |= BT_RFC_MISC_CTRL_REG_ADC_CLK_EN;
    hwp_bt_rfc->ADC_REG |= BT_RFC_ADC_REG_BRF_EN_LDO_ADC_LV ;
    hwp_bt_rfc->ADC_REG |= BT_RFC_ADC_REG_BRF_EN_LDO_ADCREF_LV ;
    hwp_bt_rfc->ADC_REG |= (BT_RFC_ADC_REG_BRF_EN_ADC_I_LV | BT_RFC_ADC_REG_BRF_EN_ADC_Q_LV);
    //power down lna, power up mxgm mixer rvga adc
    hwp_bt_rfc->RBB_REG2 |= (BT_RFC_RBB_REG2_BRF_EN_RVGA_I_LV);
    hwp_bt_rfc->RBB_REG2 |= (BT_RFC_RBB_REG2_BRF_EN_RVGA_Q_LV);
    hwp_bt_rfc->RBB_REG2 |= BT_RFC_RBB_REG2_BRF_EN_CBPF_LV;
    hwp_bt_rfc->RBB_REG2 &= ~BT_RFC_RBB_REG2_BRF_CBPF_EN_RC;

    hwp_bt_rfc->RBB_REG5 |= BT_RFC_RBB_REG5_BRF_EN_OSDACI_LV;
    hwp_bt_rfc->RBB_REG5 |= BT_RFC_RBB_REG5_BRF_EN_OSDACQ_LV;

    //start ROSCAL

    hwp_bt_rfc->ROSCAL_REG1 |= BT_RFC_ROSCAL_REG1_EN_ROSDAC_I;
    hwp_bt_rfc->ROSCAL_REG1 |= BT_RFC_ROSCAL_REG1_EN_ROSDAC_Q;
    hwp_bt_rfc->ROSCAL_REG1 &= ~(BT_RFC_ROSCAL_REG1_ROSCAL_BYPASS);
    hwp_bt_rfc->ROSCAL_REG1 |= (BT_RFC_ROSCAL_REG1_ROSCAL_START);
    while (!(hwp_bt_rfc->ROSCAL_REG2 & BT_RFC_ROSCAL_REG2_ROSCAL_DONE));

    reg_data = hwp_bt_rfc->RBB_REG4 & BT_RFC_RBB_REG4_BRF_DOS_Q_LV;
    reg_data >>= BT_RFC_RBB_REG4_BRF_DOS_Q_LV_Pos;
    hwp_bt_rfc->ROSCAL_REG2 &= ~BT_RFC_ROSCAL_REG2_DOS_Q_SW;
    hwp_bt_rfc->ROSCAL_REG2 |= reg_data << BT_RFC_ROSCAL_REG2_DOS_Q_SW_Pos;

    reg_data = hwp_bt_rfc->RBB_REG4 & BT_RFC_RBB_REG4_BRF_DOS_I_LV;
    reg_data >>= BT_RFC_RBB_REG4_BRF_DOS_I_LV_Pos;
    hwp_bt_rfc->ROSCAL_REG2 &= ~BT_RFC_ROSCAL_REG2_DOS_I_SW;
    hwp_bt_rfc->ROSCAL_REG2 |= reg_data << BT_RFC_ROSCAL_REG2_DOS_I_SW_Pos;


    hwp_bt_rfc->RBB_REG2 &= ~BT_RFC_RBB_REG2_BRF_EN_CBPF_LV;
    hwp_bt_rfc->RBB_REG2 |= BT_RFC_RBB_REG2_BRF_CBPF_EN_RC;
    hwp_bt_rfc->RBB_REG2 &= (~BT_RFC_RBB_REG2_BRF_EN_RVGA_I_LV);
    hwp_bt_rfc->RBB_REG2 &= (~BT_RFC_RBB_REG2_BRF_EN_RVGA_Q_LV);
    hwp_bt_rfc->RBB_REG5 &= ~BT_RFC_RBB_REG5_BRF_EN_OSDACI_LV;
    hwp_bt_rfc->RBB_REG5 &= ~BT_RFC_RBB_REG5_BRF_EN_OSDACQ_LV;

    hwp_bt_rfc->ADC_REG &= ~BT_RFC_ADC_REG_BRF_EN_ADC_I_LV &
                           ~BT_RFC_ADC_REG_BRF_EN_ADC_Q_LV &
                           ~BT_RFC_ADC_REG_BRF_EN_LDO_ADC_LV &
                           ~BT_RFC_ADC_REG_BRF_EN_LDO_ADCREF_LV;
    hwp_bt_rfc->ROSCAL_REG1   |=  BT_RFC_ROSCAL_REG1_ROSCAL_BYPASS;
    hwp_bt_rfc->ROSCAL_REG1 &= ~BT_RFC_ROSCAL_REG1_ROSCAL_START;
    hwp_bt_rfc->MISC_CTRL_REG &= ~BT_RFC_MISC_CTRL_REG_ADC_CLK_EN_FRC_EN;
    //}}}

    //RF_PRINTF("begin rccal\n");
    //RCCAL {{{
    uint32_t rc_capcode;

    hwp_bt_rfc->RBB_REG1 |= BT_RFC_RBB_REG1_BRF_EN_LDO_RBB_LV;
    hwp_bt_rfc->RBB_REG5 |= BT_RFC_RBB_REG5_BRF_EN_IARRAY_LV;
    hwp_bt_rfc->RBB_REG5 |= BT_RFC_RBB_REG5_BRF_RCCAL_SELXO_LV;
    hwp_bt_rfc->RBB_REG5 &= ~BT_RFC_RBB_REG5_BRF_RCCAL_MANCAP_LV;
    hwp_bt_rfc->RCROSCAL_REG &= ~BT_RFC_RCROSCAL_REG_RC_CAPCODE_OFFSET_Msk;
    hwp_bt_rfc->RBB_REG5 |= (BT_RFC_RBB_REG5_BRF_EN_RCCAL_LV);
    hwp_bt_rfc->RBB_REG5 &= ~(BT_RFC_RBB_REG5_BRF_RSTB_RCCAL_LV);
    hwp_bt_rfc->RBB_REG5 |= (BT_RFC_RBB_REG5_BRF_RSTB_RCCAL_LV);
    hwp_bt_rfc->RCROSCAL_REG |= BT_RFC_RCROSCAL_REG_RCCAL_START;
    i = 20;
    while ((i > 0) & (!(hwp_bt_rfc->RCROSCAL_REG & BT_RFC_RCROSCAL_REG_RCCAL_DONE)))
    {
        _HAL_Delay_us(1);//delay 1us
        i--;
    };


    rc_capcode = hwp_bt_rfc->RCROSCAL_REG & BT_RFC_RCROSCAL_REG_RC_CAPCODE_Msk;
    rc_capcode >>= BT_RFC_RCROSCAL_REG_RC_CAPCODE_Pos;
    hwp_bt_rfc->RBB_REG5 &= ~BT_RFC_RBB_REG5_BRF_CBPF_CAPMAN_LV_Msk;
    hwp_bt_rfc->RBB_REG5 |= (rc_capcode <<    BT_RFC_RBB_REG5_BRF_CBPF_CAPMAN_LV_Pos);
    hwp_bt_rfc->RBB_REG5 |= BT_RFC_RBB_REG5_BRF_RCCAL_MANCAP_LV;
    hwp_bt_rfc->RCROSCAL_REG &= ~BT_RFC_RCROSCAL_REG_RCCAL_START;
    hwp_bt_rfc->RBB_REG5 &= ~(BT_RFC_RBB_REG5_BRF_EN_IARRAY_LV | BT_RFC_RBB_REG5_BRF_EN_RCCAL_LV);
    hwp_bt_rfc->RBB_REG1 &= ~BT_RFC_RBB_REG1_BRF_EN_LDO_RBB_LV;
    //}}}

    //after calibration
    hwp_bt_rfc->RF_LODIST_REG &= ~BT_RFC_RF_LODIST_REG_BRF_EN_RFBG_LV ;
    hwp_bt_rfc->RF_LODIST_REG &= ~BT_RFC_RF_LODIST_REG_BRF_EN_VDDPSW_LV ;
    hwp_bt_rfc->FBDV_REG1 |= BT_RFC_FBDV_REG1_BRF_FBDV_RSTB_LV ;


    //phy init, placed here for temp
    //hwp_bt_phy->DEMOD_CFG1 &= ~BT_PHY_DEMOD_CFG1_MU_ERR_Msk;
    //hwp_bt_phy->DEMOD_CFG1 &= ~BT_PHY_DEMOD_CFG1_MU_DC_Msk;
    //hwp_bt_phy->DEMOD_CFG1 |= 0xB8 << BT_PHY_DEMOD_CFG1_MU_ERR_Pos;
    //hwp_bt_phy->DEMOD_CFG1 |= 0x13 << BT_PHY_DEMOD_CFG1_MU_DC_Pos;
    //hwp_bt_phy->TX_GAUSSFLT_CFG &= ~BT_PHY_TX_GAUSSFLT_CFG_GAUSS_GAIN_2_Msk;
    //hwp_bt_phy->TX_GAUSSFLT_CFG |= 0xff << BT_PHY_TX_GAUSSFLT_CFG_GAUSS_GAIN_2_Pos;
    //
    //hwp_bt_phy->NOTCH_CFG3 = 0x00800000 ;
    //hwp_bt_phy->NOTCH_CFG1 &= ~BT_PHY_NOTCH_CFG1_NOTCH_B1_Msk;
    //hwp_bt_phy->NOTCH_CFG1 |= 0x3000 << BT_PHY_NOTCH_CFG1_NOTCH_B1_Pos;


    free(idac_tbl);
    free(capcode_tbl);
    free(residual_cnt_tbl);
    free(idac_tbl_tx);
    free(idac_tbl_rx_2m);
    free(idac_tbl_rx_1m);
    free(idac_tbl_rx_bt);
    free(capcode_tbl_tx);
    free(capcode_tbl_rx_2m);
    free(capcode_tbl_rx_1m);
    free(capcode_tbl_rx_bt);


    return reg_addr;
    //while( 1 );

}

#if defined(ENABLE_EDR_3G)

#define MAX_CAL_STEP 150
uint32_t bt_rfc_edrlo_3g_cal(uint32_t rslt_start_addr)
{

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
    uint32_t     err_tx_3g = 0xffffffff;

    uint8_t  *idac_tbl = malloc(MAX_CAL_STEP);
    uint8_t  *capcode_tbl = malloc(MAX_CAL_STEP);
    uint32_t *residual_cnt_tbl = malloc(MAX_CAL_STEP * 4);




    uint32_t *idac_tbl_tx_3g = malloc(79 * 4);

    uint32_t *capcode_tbl_tx_3g = malloc(79 * 4);

    uint32_t residual_cnt;

    uint32_t p0;
    uint32_t p1;


    uint32_t pre_acal_up;
    uint32_t curr_acal_up;
    uint8_t  pre_acal_up_vld;
    uint8_t  seq_acal_jump_cnt ; //cnt for consecutive jump
    uint8_t  seq_acal_ful_cnt ;  //cnt for consecutive all0 all1

    uint32_t reg_data;

    RF_PRINTF("begin edr 3G LO fulcal\n");

    RF_PRINTF("begin EDR 3G LO cal\n");

    //hwp_pmuc->HXT_CR1 &= ~PMUC_HXT_CR1_CBANK_SEL_Msk;
    //hwp_pmuc->HXT_CR1 |= 0x1EA << PMUC_HXT_CR1_CBANK_SEL_Pos;
    //hwp_pmuc->HXT_CR1 |= 0xF << PMUC_HXT_CR1_LDO_VREF_Pos;

    //enable iq mod tx for iq tx calibration
    hwp_bt_phy->TX_CTRL |= BT_PHY_TX_CTRL_MOD_METHOD_BLE | BT_PHY_TX_CTRL_MOD_METHOD_BR ;
    hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_TX | BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL;
    _HAL_Delay_us(150);

    hwp_bt_rfc->INCCAL_REG2 &= ~BT_RFC_INCCAL_REG2_EDR_INCACAL_EN ;
    hwp_bt_rfc->INCCAL_REG2 &= ~BT_RFC_INCCAL_REG2_EDR_INCFCAL_EN ;
    hwp_bt_rfc->EDR_PLL_REG7 |= BT_RFC_EDR_PLL_REG7_BT_IDAC_FORCE_EN  | BT_RFC_EDR_PLL_REG7_BT_PDX_FORCE_EN;
    hwp_bt_rfc->RF_LODIST_REG |= BT_RFC_RF_LODIST_REG_BRF_EN_RFBG_LV | BT_RFC_RF_LODIST_REG_BRF_EN_VDDPSW_LV ;

    //normal value is A(5,7) F(2,5),(0,2) is for zero crc error test
    hwp_bt_rfc->EDR_PLL_REG2 &= ~(BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VL_SEL_LV_Msk | BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VH_SEL_LV_Msk | \
                                  BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_INCFCAL_VH_SEL_LV_Msk | BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_INCFCAL_VL_SEL_LV_Msk);
    hwp_bt_rfc->EDR_PLL_REG2 |= 0x5 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VL_SEL_LV_Pos | \
                                0x7 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VH_SEL_LV_Pos | \
                                0x2 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_INCFCAL_VL_SEL_LV_Pos | \
                                0x5 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_INCFCAL_VH_SEL_LV_Pos;


    //set edr_vco_ldo_vref for zero crc error test
    hwp_bt_rfc->EDR_PLL_REG1 &= ~BT_RFC_EDR_PLL_REG1_BRF_EDR_VCO_LDO_VREF_LV;
    hwp_bt_rfc->EDR_PLL_REG1 |= 0xA << BT_RFC_EDR_PLL_REG1_BRF_EDR_VCO_LDO_VREF_LV_Pos;


    hwp_bt_rfc->EDR_PLL_REG5 &= ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_SEL_CKIN_LV ;
    //LO full ACAL
    RF_PRINTF("begin LO acal\n");
    hwp_bt_rfc->EDR_PLL_REG1 |= BT_RFC_EDR_PLL_REG1_BRF_EDR_EN_VCO3G_LV ;
    //hwp_bt_rfc->EDR_PLL_REG1 |= BT_RFC_EDR_PLL_REG1_BRF_EDR_EN_VCO5G_LV ;
    hwp_bt_rfc->EDR_PLL_REG1 |= BT_RFC_EDR_PLL_REG1_BRF_EDR_LO_EN_IARY_LV ;
    hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_FKCAL_EN_LV;
    //VCO_ACAL_SEL ??
    hwp_bt_rfc->EDR_PLL_REG4 |= BT_RFC_EDR_PLL_REG4_BRF_EDR_LO_OPEN_LV;
    //hwp_bt_phy->TX_HFP_CFG &= ~( BT_PHY_TX_HFP_CFG_HFP_FCW_Msk );
    //hwp_bt_phy->TX_HFP_CFG |= ( 0x07<< BT_PHY_TX_HFP_CFG_HFP_FCW_Pos );


    //LO full fcal
    RF_PRINTF("begin EDR LO fcal\n");
    fcal_cnt    = 0x80;
    fcal_cnt_fs = 0x80;
    hwp_bt_rfc->EDR_PLL_REG5 |= BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_EN_LV ;
    hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_FKCAL_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG6 &= ~BT_RFC_EDR_PLL_REG6_BRF_EDR_FKCAL_CNT_DIVN_LV ;
    hwp_bt_rfc->EDR_PLL_REG6 |= 11520 << BT_RFC_EDR_PLL_REG6_BRF_EDR_FKCAL_CNT_DIVN_LV_Pos;
    //set lfp_fcw
    hwp_bt_phy->TX_LFP_CFG &= ~(BT_PHY_TX_LFP_CFG_BT_LFP_FCW_Msk);
    hwp_bt_phy->TX_LFP_CFG |= (0x08 << BT_PHY_TX_LFP_CFG_BT_LFP_FCW_Pos);
    hwp_bt_phy->TX_LFP_CFG    &= (~BT_PHY_TX_LFP_CFG_BT_LFP_FCW_SEL);

    hwp_bt_rfc->EDR_CAL_REG1 &= ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk;
    hwp_bt_rfc->EDR_CAL_REG1 |= (0x80) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos;
    //hwp_bt_phy->TX_HFP_CFG &= ~( BT_PHY_TX_HFP_CFG_HFP_FCW_Msk );
    //hwp_bt_phy->TX_HFP_CFG |= ( 0x07<< BT_PHY_TX_HFP_CFG_HFP_FCW_Pos );
    //hwp_bt_phy->TX_HFP_CFG &=  ~BT_PHY_TX_HFP_CFG_HFP_FCW_SEL;
    //hwp_bt_rfc->DCO_REG1 |= BT_RFC_DCO_REG1_BRF_EN_2M_MOD_LV;

    hwp_bt_rfc->EDR_PLL_REG5 |=  BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;
    hwp_bt_rfc->EDR_PLL_REG5 &= ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;

    hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV_Msk;
    hwp_bt_rfc->EDR_PLL_REG5 |=   BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV;

    hwp_bt_rfc->MISC_CTRL_REG |= BT_RFC_MISC_CTRL_REG_EDR_XTAL_REF_EN | BT_RFC_MISC_CTRL_REG_EDR_XTAL_REF_EN_FRC_EN ;
    hwp_bt_rfc->EDR_PLL_REG3  |= BT_RFC_EDR_PLL_REG3_BRF_EDR_PFDCP_EN_LV ;


    hwp_bt_rfc->EDR_CAL_REG1 &=            ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
    hwp_bt_rfc->EDR_CAL_REG1 |= (0x40) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;

    //pdx binary search for 3307MHz
    //should store the cnt value of last pdx, so loop 8 times
    for (i = 1; i < 9; i++)
    {
        RF_PRINTF("begin EDR LO fcal binary search\n");
        //--------full acal in full fcal --------
        //{{{
        acal_cnt    = 0x40;
        acal_cnt_fs = 0x40;

        hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
        hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
        //wait 4us

        //acal binary search
        for (j = 1; j < 7; j++)
        {
            //
            if (!(hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO3G_ACAL_INCAL_LV_Msk))
                break;
            else if (!(hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO3G_ACAL_UP_LV_Msk))
                acal_cnt = acal_cnt - (acal_cnt_fs >> j) ;
            else  if (hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO3G_ACAL_UP_LV_Msk)
                acal_cnt = acal_cnt + (acal_cnt_fs >> j) ;
            hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
            hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
            //wait 1us

        }
        hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
        //}}}
        _HAL_Delay_us(4);
        hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV;
        hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV_Msk;
        hwp_bt_rfc->EDR_PLL_REG5 |=   BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV;
        hwp_bt_rfc->EDR_PLL_REG5 |= 1 << BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV_Pos;
        _HAL_Delay_us(10);



        while (!(hwp_bt_rfc->EDR_PLL_REG5 &  BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RDY_LV));  //TODO timeout needed
        residual_cnt  =  hwp_bt_rfc->EDR_PLL_REG6 & BT_RFC_EDR_PLL_REG6_BRF_EDR_FKCAL_CNT_OP_LV_Msk ;
        //residual_cnt  = residual_cnt >> BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;

        //RF_PRINTF( "residual_cnt = %d,cnt_vth = %d\n",residual_cnt,residual_cnt_vth );
        if (residual_cnt > residual_cnt_vth_3g)
        {
            idac1    = acal_cnt;
            p1       = residual_cnt ;
            error1   = residual_cnt - residual_cnt_vth_3g ;
            capcode1 = fcal_cnt;
            fcal_cnt = fcal_cnt + (fcal_cnt_fs >> i) ;
        }
        else if (residual_cnt <= residual_cnt_vth_3g)
        {
            idac0    = acal_cnt;
            p0       = residual_cnt ;
            error0   = residual_cnt_vth_3g - residual_cnt ;
            capcode0 = fcal_cnt;
            fcal_cnt = fcal_cnt - (fcal_cnt_fs >> i) ;
        }
        //RF_PRINTF( "fcal bin fcal_cnt = %x,acal_cnt = %x\n",fcal_cnt,acal_cnt );
        hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV;
        hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (fcal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos;
    }

    //RF_PRINTF( "sweep start idac0 = %x,capcode0 = %x\n",idac0,capcode0 );
    //RF_PRINTF( "sweep start idac1 = %x,capcode1 = %x\n",idac1,capcode1 );
    //RF_PRINTF( "sweep start error0 = %x,error1 = %x\n",error0,error1 );
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

    hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk;
    hwp_bt_rfc->EDR_CAL_REG1 |= (fcal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos;
    hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV;
    //hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;

    fcal_cnt = capcode_tbl[0] ;
    acal_cnt = idac_tbl[0] ;
    //RF_PRINTF( "sweep start fcal_cnt = %x,acal_cnt = %x\n",fcal_cnt,acal_cnt );
    //sweep pdx until 3203M
    i = 0;

    do
    {
        i                     +=  1 ;
        fcal_cnt              +=  1 ;
        seq_acal_jump_cnt      =  0 ;
        seq_acal_ful_cnt       =  0 ;
        pre_acal_up_vld        =  0 ;
        hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (fcal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos;
        //seq acal {{{
        hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
        //VCO_ACAL_SEL ??
        hwp_bt_rfc->EDR_PLL_REG4 |= BT_RFC_EDR_PLL_REG4_BRF_EDR_LO_OPEN_LV;
        while ((seq_acal_jump_cnt < 4) & (seq_acal_ful_cnt < 2))
        {
            hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
            hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
            //wait for 4us
            //for(int j=0;j<100;j++)
            //RF_PRINTF( "wait idac settling\n" );
            if (!(hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO3G_ACAL_INCAL_LV_Msk))
                break;
            curr_acal_up = hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO3G_ACAL_UP_LV_Msk;
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
        hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
        ///}}}

        hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
        //wait for 4us
        //for(int j=0;j<100;j++)
        //RF_PRINTF( "wait idac settling\n" );
        _HAL_Delay_us(4);
        hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV;

        hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV_Msk;
        hwp_bt_rfc->EDR_PLL_REG5 |=   BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV;
        hwp_bt_rfc->EDR_PLL_REG5 |= 1 << BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV_Pos;

        //for( j=0;j<100;j++)
        //RF_PRINTF( "wait idac settling\n" );
        _HAL_Delay_us(10);
        while (!(hwp_bt_rfc->EDR_PLL_REG5 &  BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RDY_LV));
        residual_cnt  =  hwp_bt_rfc->EDR_PLL_REG6 & BT_RFC_EDR_PLL_REG6_BRF_EDR_FKCAL_CNT_OP_LV_Msk ;
        //residual_cnt  = residual_cnt >> BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;
        //RF_PRINTF( "residual_cnt = %d, residual_cnt_vtl=%d\n", residual_cnt, residual_cnt_vtl );

        if (residual_cnt <= residual_cnt_vtl_3g)
            break;

        HAL_ASSERT(i < MAX_CAL_STEP);
        idac_tbl[i]         = acal_cnt ;
        capcode_tbl[i]      = fcal_cnt ;
        residual_cnt_tbl[i] = residual_cnt ;
    }
    while (residual_cnt > residual_cnt_vtl_3g) ;

    hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_FKCAL_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV;

    //search result for each channel
    sweep_num   = i ;
    //for bt 79 chnl
    for (j = 0; j < 79; j++)
    {
        err_tx_3g    = 0 ;
        for (i = 0; i < sweep_num; i++)
        {
            if (i == 0)
            {
                if (ref_residual_cnt_tbl_tx_3g[j] > residual_cnt_tbl[i])
                {
                    err_tx_3g     =  ref_residual_cnt_tbl_tx_3g[j]    - residual_cnt_tbl[i] ;
                }
                else
                {
                    err_tx_3g     =  residual_cnt_tbl[i] - ref_residual_cnt_tbl_tx_3g[j]    ;
                }
                idac_tbl_tx_3g[j]       =  idac_tbl[i];
                capcode_tbl_tx_3g[j]    =  capcode_tbl[i];
            }
            else
            {
                if (ref_residual_cnt_tbl_tx_3g[j] > residual_cnt_tbl[i])
                {
                    error0     =  ref_residual_cnt_tbl_tx_3g[j]    - residual_cnt_tbl[i] ;
                    if (error0 < err_tx_3g)
                    {
                        err_tx_3g = error0;
                        idac_tbl_tx_3g[j]       =  idac_tbl[i];
                        capcode_tbl_tx_3g[j]    =  capcode_tbl[i];
                    }
                }
                else
                {
                    error0     = residual_cnt_tbl[i] - ref_residual_cnt_tbl_tx_3g[j] ;
                    if (error0 < err_tx_3g)
                    {
                        err_tx_3g = error0;
                        idac_tbl_tx_3g[j]       =  idac_tbl[i];
                        capcode_tbl_tx_3g[j]    =  capcode_tbl[i];
                    }
                }
            }
        }
    }
    //after edrlo calibration
    hwp_bt_rfc->EDR_PLL_REG1  &= ~BT_RFC_EDR_PLL_REG1_BRF_EDR_EN_VCO3G_LV;
    //hwp_bt_rfc->EDR_PLL_REG1  &= ~BT_RFC_EDR_PLL_REG1_BRF_EDR_EN_VCO5G_LV;
    hwp_bt_rfc->EDR_PLL_REG1  &= ~BT_RFC_EDR_PLL_REG1_BRF_EDR_LO_EN_IARY_LV;
    hwp_bt_rfc->RF_LODIST_REG &= ~BT_RFC_RF_LODIST_REG_BRF_EN_RFBG_LV ;
    hwp_bt_rfc->RF_LODIST_REG &= ~BT_RFC_RF_LODIST_REG_BRF_EN_VDDPSW_LV ;

    //write to rf_mem
    uint32_t reg_addr = rslt_start_addr  ;
    reg_data = 0;

    hwp_bt_rfc->CAL_ADDR_REG2 &= 0xffff;
    hwp_bt_rfc->CAL_ADDR_REG2 += reg_addr << 16;
    for (i = 0; i < 79; i++)
    {
        //store bttx cal result
        reg_data = ((idac_tbl_tx_3g[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos) + (capcode_tbl_tx_3g[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos) + \
                    (3 << BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_FC_LV_Pos)               + (0x10 << BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_BM_LV_Pos)) ;
        reg_data |= 0x6 << BT_RFC_EDR_CAL_REG1_BRF_TRF_EDR_TMXCAP_SEL_LV_Pos ;
        write_memory(reg_addr + BT_RFC_MEM_BASE, reg_data);
        reg_addr += 4;
    }

    //oslo calibration
    uint16_t adc_value;
    uint16_t max_adc_value = 0;
    uint8_t  fc[79];
    uint8_t  bm[79];

    uint8_t bm_step;
    uint8_t acal_cmp;

    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL;
    _HAL_Delay_us(10 * 5); //wait 10us
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_TX ;
    hwp_bt_rfc->EDR_PLL_REG4  &= ~BT_RFC_EDR_PLL_REG4_BRF_EDR_LO_OPEN_LV;
    hwp_bt_phy->TX_LFP_CFG   |= BT_PHY_TX_LFP_CFG_BT_LFP_FCW_SEL;


    //Enable power switch and bandgap
    hwp_bt_rfc->RF_LODIST_REG |= BT_RFC_RF_LODIST_REG_BRF_EN_RFBG_LV | BT_RFC_RF_LODIST_REG_BRF_EN_VDDPSW_LV ;
    _HAL_Delay_us(2);
    //Enable EDR LO
    hwp_bt_rfc->EDR_PLL_REG1 |= BT_RFC_EDR_PLL_REG1_BRF_EDR_EN_VCO3G_LV |  BT_RFC_EDR_PLL_REG1_BRF_EDR_LO_EN_IARY_LV;
    hwp_bt_rfc->EDR_PLL_REG3 |= BT_RFC_EDR_PLL_REG3_BRF_EDR_PFDCP_EN_LV ;
    hwp_bt_rfc->EDR_PLL_REG4 |= BT_RFC_EDR_PLL_REG4_BRF_EDR_EN_LF_LV ;
    hwp_bt_rfc->EDR_PLL_REG5 |= BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_EN_LV ;
    hwp_bt_rfc->EDR_OSLO_REG |= BT_RFC_EDR_OSLO_REG_BRF_EDR_EN_OSLO_LV ;
    hwp_bt_rfc->EDR_OSLO_REG |= BT_RFC_EDR_OSLO_REG_BRF_EDR_EN_LODIST_LV ;

    //Enable edr idac/pdx force
    hwp_bt_rfc->EDR_PLL_REG7 |= BT_RFC_EDR_PLL_REG7_BT_IDAC_FORCE_EN  | BT_RFC_EDR_PLL_REG7_BT_PDX_FORCE_EN;

    //OSLO cal setting
    hwp_bt_rfc->EDR_OSLO_REG |= BT_RFC_EDR_OSLO_REG_BRF_EDR_EN_OSLO_PKDET_LV ;
    hwp_bt_rfc->EDR_OSLO_REG |= BT_RFC_EDR_OSLO_REG_BRF_EDR_EN_OSLO_FCAL_LV ;
    hwp_bt_rfc->EDR_CAL_REG1 &= ~BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_BM_LV ;
    hwp_bt_rfc->EDR_CAL_REG1 |= 0x10 << BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_BM_LV_Pos ;

    hwp_gpadc->ADC_CFG_REG1 |= GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_BG      | \
                               GPADC_ADC_CFG_REG1_ANAU_GPADC_P_INT_EN   | \
                               GPADC_ADC_CFG_REG1_ANAU_GPADC_SE         | \
                               GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN ;

    hwp_gpadc->ADC_CTRL_REG2 &= ~GPADC_ADC_CTRL_REG2_CONV_WIDTH;
    hwp_gpadc->ADC_CTRL_REG2 |= 10 << GPADC_ADC_CTRL_REG2_CONV_WIDTH_Pos;
    hwp_gpadc->ADC_CTRL_REG2 &= ~GPADC_ADC_CTRL_REG2_SAMP_WIDTH;
    hwp_gpadc->ADC_CTRL_REG2 |= 10 << GPADC_ADC_CTRL_REG2_SAMP_WIDTH_Pos;
    hwp_gpadc->ADC_SLOT0_REG |= GPADC_ADC_SLOT0_REG_SLOT_EN;

    //NVIC_EnableIRQ(GPADC_IRQn);

    //sweep each channel for oslo fcal
    for (i = 0; i < 79; i++)
    {
        //apply calibration result
        //fulcal_rslt = read_memory( fulcal_addr+i*4 );
//        hwp_bt_rfc->EDR_CAL_REG1 &= ~(BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV | BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV);
//        hwp_bt_rfc->EDR_CAL_REG1 |= (idac_tbl_tx_3g[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos) + (capcode_tbl_tx_3g[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos) ;
//        //lock LO
//        hwp_bt_rfc->EDR_PLL_REG5 |=  BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;
//        hwp_bt_rfc->EDR_PLL_REG5 &= ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;
        hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_CHANNEL;
        hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_CHANNEL;
        hwp_bt_mac->DMRADIOCNTL1 |= i << BT_MAC_DMRADIOCNTL1_CHANNEL_Pos;
        hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_TX | BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL;


        _HAL_Delay_us(40);  //40us

        //sweep fc
        fc[i] = 0;
        max_adc_value = 0;
        for (j = 0; j < 8; j++)
        {
            hwp_bt_rfc->EDR_CAL_REG1  &= ~BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_FC_LV;
            hwp_bt_rfc->EDR_CAL_REG1  |= j << BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_FC_LV_Pos;
            _HAL_Delay_us(40);
            //adc1_sw_start_gpadc(  );
            hwp_gpadc->ADC_CTRL_REG |= GPADC_ADC_CTRL_REG_ADC_START;
            //__WFI(  );
            while (!(hwp_gpadc->GPADC_IRQ & GPADC_GPADC_IRQ_GPADC_IRSR));

            adc_value =  hwp_gpadc->ADC_RDATA0 & 0xFFF;
            hwp_gpadc->GPADC_IRQ |= GPADC_GPADC_IRQ_GPADC_ICR ;
            if (max_adc_value < adc_value)
            {
                max_adc_value = adc_value ;
                fc[i] = j;
            }
        }
        //RF_PRINTF("fc[%d] = %d\n",i,fc[i]  );

        //OSLO ACAL
        bm_step  = 0x8;
        bm[i] = 0x10;
        hwp_bt_rfc->EDR_CAL_REG1 &= ~BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_FC_LV ;
        hwp_bt_rfc->EDR_CAL_REG1 |= fc[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_FC_LV_Pos ;

        hwp_bt_rfc->EDR_CAL_REG1 &= ~BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_BM_LV ;
        hwp_bt_rfc->EDR_CAL_REG1 |= bm[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_BM_LV_Pos ;

        //lock LO
        hwp_bt_rfc->EDR_PLL_REG5 |=  BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;
        hwp_bt_rfc->EDR_PLL_REG5 &= ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;
        _HAL_Delay_us(40);  //40us

        for (j = 0; j < 5; j++)
        {
            acal_cmp = (hwp_bt_rfc->EDR_OSLO_REG & BT_RFC_EDR_OSLO_REG_BRF_EDR_OSLO_ACAL_CMP_LV_Msk) >> BT_RFC_EDR_OSLO_REG_BRF_EDR_OSLO_ACAL_CMP_LV_Pos ;
            if (acal_cmp)
            {
                bm[i] -= bm_step;
            }
            else
            {
                bm[i] += bm_step;
            }
            bm_step >>= 1;
            hwp_bt_rfc->EDR_CAL_REG1 &= ~BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_BM_LV ;
            hwp_bt_rfc->EDR_CAL_REG1 |= bm[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_BM_LV_Pos ;
            _HAL_Delay_us(4);

            //RF_PRINTF( "bm_value@%d = %d\n",i,bm_value );
        }
        hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL;
        _HAL_Delay_us(10 * 5); //wait 10us
    }


    //after calibration
    hwp_bt_rfc->MISC_CTRL_REG &= ~BT_RFC_MISC_CTRL_REG_EDR_XTAL_REF_EN_FRC_EN  ;
    hwp_bt_rfc->EDR_PLL_REG7  &= ~BT_RFC_EDR_PLL_REG7_BT_IDAC_FORCE_EN & ~BT_RFC_EDR_PLL_REG7_BT_PDX_FORCE_EN;
    hwp_bt_rfc->INCCAL_REG2   |= BT_RFC_INCCAL_REG2_EDR_INCACAL_EN ;
    hwp_bt_rfc->INCCAL_REG2   |= BT_RFC_INCCAL_REG2_EDR_INCFCAL_EN ;
    hwp_bt_rfc->RF_LODIST_REG &= ~BT_RFC_RF_LODIST_REG_BRF_EN_RFBG_LV & ~BT_RFC_RF_LODIST_REG_BRF_EN_VDDPSW_LV ;
    hwp_bt_rfc->EDR_PLL_REG1 &= ~BT_RFC_EDR_PLL_REG1_BRF_EDR_EN_VCO3G_LV ;
    hwp_bt_rfc->EDR_PLL_REG1 &= ~BT_RFC_EDR_PLL_REG1_BRF_EDR_LO_EN_IARY_LV;
    hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_FKCAL_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG4  &= ~BT_RFC_EDR_PLL_REG4_BRF_EDR_LO_OPEN_LV;
    hwp_bt_rfc->EDR_PLL_REG4  &= ~BT_RFC_EDR_PLL_REG4_BRF_EDR_EN_LF_LV;
    //hwp_bt_rfc->EDR_PLL_REG2  |= BT_RFC_EDR_PLL_REG2_BRF_EDR_EN_VCO5G_TX_LV ;
    hwp_bt_rfc->EDR_OSLO_REG &= ~(BT_RFC_EDR_OSLO_REG_BRF_EDR_EN_OSLO_LV | BT_RFC_EDR_OSLO_REG_BRF_EDR_EN_LODIST_LV);
    hwp_bt_phy->TX_LFP_CFG    |= BT_PHY_TX_LFP_CFG_BT_LFP_FCW_SEL;
    hwp_bt_rfc->EDR_PLL_REG5 |=  BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;
    hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG3 &= ~BT_RFC_EDR_PLL_REG3_BRF_EDR_PFDCP_EN_LV ;

    //release bt mac force
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL;
    _HAL_Delay_us(20);
    hwp_bt_mac->DMRADIOCNTL1 &= ~(BT_MAC_DMRADIOCNTL1_FORCE_TX | BT_MAC_DMRADIOCNTL1_FORCE_CHANNEL) ;


    //disable iq mod tx after iq tx calibration
    hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MOD_METHOD_BLE & ~BT_PHY_TX_CTRL_MOD_METHOD_BR ;

    hwp_bt_rfc->EDR_OSLO_REG &= ~BT_RFC_EDR_OSLO_REG_BRF_EDR_EN_OSLO_FCAL_LV ;
    hwp_gpadc->ADC_CFG_REG1 &= ~GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_BG      & \
                               ~GPADC_ADC_CFG_REG1_ANAU_GPADC_P_INT_EN   & \
                               ~GPADC_ADC_CFG_REG1_ANAU_GPADC_SE         & \
                               ~GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOREF_EN ;
    //write to rf_mem
    reg_addr = rslt_start_addr  ;
    reg_data = 0;

    //hwp_bt_rfc->CAL_ADDR_REG2 &= 0xffff;
    //hwp_bt_rfc->CAL_ADDR_REG2 += reg_addr << 16;
    for (i = 0; i < 79; i++)
    {
        //store bttx cal result
        reg_data = ((idac_tbl_tx_3g[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos) + (capcode_tbl_tx_3g[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos) + \
                    (fc[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_FC_LV_Pos)               + (bm[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_OSLO_BM_LV_Pos)) ;
        reg_data |= 0x6 << BT_RFC_EDR_CAL_REG1_BRF_TRF_EDR_TMXCAP_SEL_LV_Pos ;

        write_memory(reg_addr + BT_RFC_MEM_BASE, reg_data);
        reg_addr += 4;
    }


    free(idac_tbl);
    free(capcode_tbl);
    free(residual_cnt_tbl);
    free(idac_tbl_tx_3g);
    free(capcode_tbl_tx_3g);

    return reg_addr;
}

#elif defined(ENABLE_EDR_5G)
uint32_t bt_rfc_edrlo_5g_cal(uint32_t rslt_start_addr)
{

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
    uint32_t     err_tx_5g = 0xffffffff;

    uint8_t  idac_tbl[150];
    uint8_t  capcode_tbl[150];
    uint32_t residual_cnt_tbl[150];



    uint32_t idac_tbl_tx_5g[79];

    uint32_t capcode_tbl_tx_5g[79];

    uint32_t residual_cnt;

    uint32_t p0;
    uint32_t p1;


    uint32_t pre_acal_up;
    uint32_t curr_acal_up;
    uint8_t  pre_acal_up_vld;
    uint8_t  seq_acal_jump_cnt ; //cnt for consecutive jump
    uint8_t  seq_acal_ful_cnt ;  //cnt for consecutive all0 all1

    uint32_t reg_data;

    //RF_PRINTF("begin edr fulcal\n");

    //RF_PRINTF("begin EDR LO cal\n");

    //hwp_pmuc->HXT_CR1 &= ~PMUC_HXT_CR1_CBANK_SEL_Msk;
    //hwp_pmuc->HXT_CR1 |= 0x1EA << PMUC_HXT_CR1_CBANK_SEL_Pos;
    //hwp_pmuc->HXT_CR1 |= 0xF << PMUC_HXT_CR1_LDO_VREF_Pos;

    //enable iq mod tx for iq tx calibration
    hwp_bt_phy->TX_CTRL |= BT_PHY_TX_CTRL_MOD_METHOD_BLE | BT_PHY_TX_CTRL_MOD_METHOD_BR ;
    hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_TX | BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL;
    _HAL_Delay_us(20);

    hwp_bt_rfc->INCCAL_REG2 &= ~BT_RFC_INCCAL_REG2_EDR_INCACAL_EN ;
    hwp_bt_rfc->INCCAL_REG2 &= ~BT_RFC_INCCAL_REG2_EDR_INCFCAL_EN ;
    hwp_bt_rfc->EDR_PLL_REG7 |= BT_RFC_EDR_PLL_REG7_BT_IDAC_FORCE_EN  | BT_RFC_EDR_PLL_REG7_BT_PDX_FORCE_EN;
    hwp_bt_rfc->RF_LODIST_REG |= BT_RFC_RF_LODIST_REG_BRF_EN_RFBG_LV | BT_RFC_RF_LODIST_REG_BRF_EN_VDDPSW_LV ;

    hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VL_SEL_LV_Msk;
    hwp_bt_rfc->EDR_PLL_REG2 |= 0x7 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VL_SEL_LV_Pos;
    _HAL_Delay_us(2);
    hwp_bt_rfc->EDR_PLL_REG5 &= ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_SEL_CKIN_LV ;
    _HAL_Delay_us(2);
    hwp_bt_rfc->EDR_PLL_REG5 |= 1 << BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_SEL_CKIN_LV_Pos ;
    _HAL_Delay_us(2);
    hwp_bt_rfc->EDR_PLL_REG5 &= ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_MOD_STG_LV ;
    hwp_bt_rfc->EDR_PLL_REG5 |= 2 << BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_MOD_STG_LV_Pos ;
    hwp_bt_rfc->EDR_PLL_REG7 |= BT_RFC_EDR_PLL_REG7_BRF_EDR_SEL_SDM_CLK_LV;
    //LO full ACAL
    RF_PRINTF("begin LO acal\n");
    //hwp_bt_rfc->EDR_PLL_REG1 |= BT_RFC_EDR_PLL_REG1_BRF_EDR_EN_VCO3G_LV ;
    hwp_bt_rfc->EDR_PLL_REG1 |= BT_RFC_EDR_PLL_REG1_BRF_EDR_EN_VCO5G_LV ;
    hwp_bt_rfc->EDR_PLL_REG1 |= BT_RFC_EDR_PLL_REG1_BRF_EDR_LO_EN_IARY_LV;
    hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_FKCAL_EN_LV;

    //VCO_ACAL_SEL ??
    hwp_bt_rfc->EDR_PLL_REG4 |= BT_RFC_EDR_PLL_REG4_BRF_EDR_EN_LF_LV;
    hwp_bt_rfc->EDR_PLL_REG4 |= BT_RFC_EDR_PLL_REG4_BRF_EDR_LO_OPEN_LV;
    //hwp_bt_phy->TX_HFP_CFG &= ~( BT_PHY_TX_HFP_CFG_HFP_FCW_Msk );
    //hwp_bt_phy->TX_HFP_CFG |= ( 0x07<< BT_PHY_TX_HFP_CFG_HFP_FCW_Pos );

    /*
        hwp_bt_rfc->EDR_CAL_REG1 &= ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (0x40) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;

        acal_cnt    = 0x40;
        acal_cnt_fs = 0x40;

        //wait 4us

        //acal binary search
        for (i = 1; i < 7; i++)
        {
            RF_PRINTF("begin EDR LO acal binary search\n");
            //
            RF_PRINTF("pre acal_cnt = %d\n", acal_cnt);
            RF_PRINTF("step = %d\n", acal_cnt_fs >> i);
            if (!(hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO5G_ACAL_INCAL_LV_Msk))
                break;
            else if (!(hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO5G_ACAL_UP_LV_Msk))
                acal_cnt = acal_cnt - (acal_cnt_fs >> i) ;
            else  if (hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO5G_ACAL_UP_LV_Msk)
                acal_cnt = acal_cnt + (acal_cnt_fs >> i)  ;
            hwp_bt_rfc->EDR_CAL_REG1 &= ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
            hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
            //wait 1us
            RF_PRINTF("acal_cnt = %d\n", acal_cnt);

        }
        hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
    */

    //LO full fcal
    //RF_PRINTF("begin EDR LO fcal\n");
    fcal_cnt    = 0x80;
    fcal_cnt_fs = 0x80;
    hwp_bt_rfc->EDR_PLL_REG5 |= BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_EN_LV ;
    hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_FKCAL_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG6 &= ~BT_RFC_EDR_PLL_REG6_BRF_EDR_FKCAL_CNT_DIVN_LV ;
    hwp_bt_rfc->EDR_PLL_REG6 |= 8160 << BT_RFC_EDR_PLL_REG6_BRF_EDR_FKCAL_CNT_DIVN_LV_Pos;
    //set lfp_fcw
    hwp_bt_phy->TX_LFP_CFG &= ~(BT_PHY_TX_LFP_CFG_BT_LFP_FCW_Msk);
    hwp_bt_phy->TX_LFP_CFG |= (0x08 << BT_PHY_TX_LFP_CFG_BT_LFP_FCW_Pos);
    hwp_bt_phy->TX_LFP_CFG    &= (~BT_PHY_TX_LFP_CFG_BT_LFP_FCW_SEL);

    hwp_bt_rfc->EDR_CAL_REG1 &= ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk;
    hwp_bt_rfc->EDR_CAL_REG1 |= (0x80) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos;
    //hwp_bt_phy->TX_HFP_CFG &= ~( BT_PHY_TX_HFP_CFG_HFP_FCW_Msk );
    //hwp_bt_phy->TX_HFP_CFG |= ( 0x07<< BT_PHY_TX_HFP_CFG_HFP_FCW_Pos );
    //hwp_bt_phy->TX_HFP_CFG &=  ~BT_PHY_TX_HFP_CFG_HFP_FCW_SEL;
    //hwp_bt_rfc->DCO_REG1 |= BT_RFC_DCO_REG1_BRF_EN_2M_MOD_LV;

    hwp_bt_rfc->EDR_PLL_REG5 |=  BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;
    hwp_bt_rfc->EDR_PLL_REG5 &= ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;

    hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV_Msk;
    hwp_bt_rfc->EDR_PLL_REG5 |=   BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV;

    hwp_bt_rfc->MISC_CTRL_REG |= BT_RFC_MISC_CTRL_REG_EDR_XTAL_REF_EN | BT_RFC_MISC_CTRL_REG_EDR_XTAL_REF_EN_FRC_EN;
    hwp_bt_rfc->EDR_PLL_REG3  |= BT_RFC_EDR_PLL_REG3_BRF_EDR_PFDCP_EN_LV ;


    hwp_bt_rfc->EDR_CAL_REG1 &=            ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
    hwp_bt_rfc->EDR_CAL_REG1 |= (0x40) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;

    //pdx binary search
    //should store the cnt value of last pdx, so loop 8 times
    for (i = 1; i < 9; i++)
    {
        RF_PRINTF("begin EDR LO fcal binary search\n");
        //--------full acal in full fcal --------
        //{{{
        acal_cnt    = 0x40;
        acal_cnt_fs = 0x40;

        hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
        hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
        //wait 4us
        _HAL_Delay_us(5);
        //acal binary search
        for (j = 1; j < 7; j++)
        {
            //
            if (!(hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO5G_ACAL_INCAL_LV_Msk))
                break;
            else if (!(hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO5G_ACAL_UP_LV_Msk))
                acal_cnt = acal_cnt - (acal_cnt_fs >> j) ;
            else  if (hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO5G_ACAL_UP_LV_Msk)
                acal_cnt = acal_cnt + (acal_cnt_fs >> j) ;
            hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
            hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
            //wait 1us
            _HAL_Delay_us(2);
        }
        hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
        //}}}

        hwp_bt_rfc->EDR_PLL_REG5 |= 1 << BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV_Pos;

        _HAL_Delay_us(400);

        while (!(hwp_bt_rfc->EDR_PLL_REG5 &  BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RDY_LV));
        residual_cnt  =  hwp_bt_rfc->EDR_PLL_REG6 & BT_RFC_EDR_PLL_REG6_BRF_EDR_FKCAL_CNT_OP_LV_Msk ;
        //residual_cnt  = residual_cnt >> BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;

        //RF_PRINTF( "residual_cnt = %d,cnt_vth = %d\n",residual_cnt,residual_cnt_vth );
        if (residual_cnt > residual_cnt_vth_5g)
        {
            idac1    = acal_cnt;
            p1       = residual_cnt ;
            error1   = residual_cnt - residual_cnt_vth_5g ;
            capcode1 = fcal_cnt;
            fcal_cnt = fcal_cnt + (fcal_cnt_fs >> i) ;
        }
        else if (residual_cnt <= residual_cnt_vth_5g)
        {
            idac0    = acal_cnt;
            p0       = residual_cnt ;
            error0   = residual_cnt_vth_5g - residual_cnt ;
            capcode0 = fcal_cnt;
            fcal_cnt = fcal_cnt - (fcal_cnt_fs >> i) ;
        }
        //RF_PRINTF( "fcal bin fcal_cnt = %x,acal_cnt = %x\n",fcal_cnt,acal_cnt );
        hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV;
        hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (fcal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos;
    }

    //RF_PRINTF( "sweep start idac0 = %x,capcode0 = %x\n",idac0,capcode0 );
    //RF_PRINTF( "sweep start idac1 = %x,capcode1 = %x\n",idac1,capcode1 );
    //RF_PRINTF( "sweep start error0 = %x,error1 = %x\n",error0,error1 );
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

    hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk;
    hwp_bt_rfc->EDR_CAL_REG1 |= (fcal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos;
    hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV;
    //hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;

    fcal_cnt = capcode_tbl[0] ;
    acal_cnt = idac_tbl[0] ;
    //RF_PRINTF( "sweep start fcal_cnt = %x,acal_cnt = %x\n",fcal_cnt,acal_cnt );
    //sweep pdx until 3203M
    i = 0;

    do
    {
        i                     +=  1 ;
        fcal_cnt              +=  1 ;
        seq_acal_jump_cnt      =  0 ;
        seq_acal_ful_cnt       =  0 ;
        pre_acal_up_vld        =  0 ;
        hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (fcal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos;
        //seq acal {{{
        hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
        //VCO_ACAL_SEL ??
        hwp_bt_rfc->EDR_PLL_REG4 |= BT_RFC_EDR_PLL_REG4_BRF_EDR_LO_OPEN_LV;
        while ((seq_acal_jump_cnt < 4) & (seq_acal_ful_cnt < 2))
        {
            hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
            hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
            //wait for 4us
            //for(int j=0;j<100;j++)
            //RF_PRINTF( "wait idac settling\n" );
            _HAL_Delay_us(2);
            if (!(hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO5G_ACAL_INCAL_LV_Msk))
                break;
            curr_acal_up = hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO5G_ACAL_UP_LV_Msk;
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
        hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
        ///}}}

        hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
        //wait for 4us
        //for(int j=0;j<100;j++)
        //RF_PRINTF( "wait idac settling\n" );
        _HAL_Delay_us(4);
        hwp_bt_rfc->EDR_PLL_REG5 &= ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV_Msk;

        hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV_Msk;
        hwp_bt_rfc->EDR_PLL_REG5 |=   BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV;
        hwp_bt_rfc->EDR_PLL_REG5 |= 1 << BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV_Pos;
        //for( j=0;j<100;j++)
        //RF_PRINTF( "wait idac settling\n" );
        _HAL_Delay_us(10);
        while (!(hwp_bt_rfc->EDR_PLL_REG5 &  BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RDY_LV));
        residual_cnt  =  hwp_bt_rfc->EDR_PLL_REG6 & BT_RFC_EDR_PLL_REG6_BRF_EDR_FKCAL_CNT_OP_LV_Msk ;
        //residual_cnt  = residual_cnt >> BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;
        //RF_PRINTF( "residual_cnt = %d, residual_cnt_vtl=%d\n", residual_cnt, residual_cnt_vtl );

        if (residual_cnt <= residual_cnt_vtl_5g)
            break;

        idac_tbl[i]         = acal_cnt ;
        capcode_tbl[i]      = fcal_cnt ;
        residual_cnt_tbl[i] = residual_cnt ;
    }
    while (residual_cnt > residual_cnt_vtl_5g) ;

    hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_FKCAL_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG5 &= ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV;

    //search result for each channel
    sweep_num   = i ;
    //for bt 79 chnl
    for (j = 0; j < 79; j++)
    {
        err_tx_5g    = 0 ;
        for (i = 0; i < sweep_num; i++)
        {
            if (i == 0)
            {
                if (ref_residual_cnt_tbl_tx_5g[j] > residual_cnt_tbl[i])
                {
                    err_tx_5g     =  ref_residual_cnt_tbl_tx_5g[j]    - residual_cnt_tbl[i] ;
                }
                else
                {
                    err_tx_5g     =  residual_cnt_tbl[i] - ref_residual_cnt_tbl_tx_5g[j]    ;
                }
                idac_tbl_tx_5g[j]       =  idac_tbl[i];
                capcode_tbl_tx_5g[j]    =  capcode_tbl[i];
            }
            else
            {
                if (ref_residual_cnt_tbl_tx_5g[j] > residual_cnt_tbl[i])
                {
                    error0     =  ref_residual_cnt_tbl_tx_5g[j]    - residual_cnt_tbl[i] ;
                    if (error0 < err_tx_5g)
                    {
                        err_tx_5g = error0;
                        idac_tbl_tx_5g[j]       =  idac_tbl[i];
                        capcode_tbl_tx_5g[j]    =  capcode_tbl[i];
                    }
                }
                else
                {
                    error0     = residual_cnt_tbl[i] - ref_residual_cnt_tbl_tx_5g[j] ;
                    if (error0 < err_tx_5g)
                    {
                        err_tx_5g = error0;
                        idac_tbl_tx_5g[j]       =  idac_tbl[i];
                        capcode_tbl_tx_5g[j]    =  capcode_tbl[i];
                    }
                }
            }
        }
    }
    //write to rf_mem
    uint32_t reg_addr = rslt_start_addr ;
    reg_data = 0;
    hwp_bt_rfc->CAL_ADDR_REG2 &= 0xffff;
    hwp_bt_rfc->CAL_ADDR_REG2 += reg_addr << 16;
    for (i = 0; i < 79; i++)
    {
        //store bttx cal result
        reg_data = ((idac_tbl_tx_5g[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos) + (capcode_tbl_tx_5g[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos)) ;
        write_memory(reg_addr + BT_RFC_MEM_BASE, reg_data);
        reg_addr += 4;
    }


    //after calibration
    hwp_bt_rfc->MISC_CTRL_REG &= ~BT_RFC_MISC_CTRL_REG_EDR_XTAL_REF_EN_FRC_EN;
    hwp_bt_rfc->EDR_PLL_REG7  &= ~(BT_RFC_EDR_PLL_REG7_BT_IDAC_FORCE_EN | BT_RFC_EDR_PLL_REG7_BT_PDX_FORCE_EN);
    hwp_bt_rfc->INCCAL_REG2   |= BT_RFC_INCCAL_REG2_EDR_INCACAL_EN ;
    hwp_bt_rfc->INCCAL_REG2   |= BT_RFC_INCCAL_REG2_EDR_INCFCAL_EN ;
    hwp_bt_rfc->RF_LODIST_REG &= ~BT_RFC_RF_LODIST_REG_BRF_EN_RFBG_LV & ~BT_RFC_RF_LODIST_REG_BRF_EN_VDDPSW_LV ;
    hwp_bt_rfc->EDR_PLL_REG1 &= ~BT_RFC_EDR_PLL_REG1_BRF_EDR_EN_VCO5G_LV ;
    hwp_bt_rfc->EDR_PLL_REG1 &= ~BT_RFC_EDR_PLL_REG1_BRF_EDR_LO_EN_IARY_LV;
    hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_FKCAL_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG4  &= ~BT_RFC_EDR_PLL_REG4_BRF_EDR_LO_OPEN_LV;
    hwp_bt_rfc->EDR_PLL_REG4  &= ~BT_RFC_EDR_PLL_REG4_BRF_EDR_EN_LF_LV;
    hwp_bt_rfc->EDR_PLL_REG2  |= BT_RFC_EDR_PLL_REG2_BRF_EDR_EN_VCO5G_TX_LV ;
    hwp_bt_phy->TX_LFP_CFG    |= BT_PHY_TX_LFP_CFG_BT_LFP_FCW_SEL;
    hwp_bt_rfc->EDR_PLL_REG5 |=  BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;
    hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG3 &= ~BT_RFC_EDR_PLL_REG3_BRF_EDR_PFDCP_EN_LV ;

    //release bt mac force
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL;
    _HAL_Delay_us(20);
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_TX
                                //disable iq mod tx after iq tx calibration
                                hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MOD_METHOD_BLE & ~BT_PHY_TX_CTRL_MOD_METHOD_BR ;

    return reg_addr;

}
#elif defined(ENABLE_EDR_2G)
uint32_t bt_rfc_edrlo_2g_cal(uint32_t rslt_start_addr)
{


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
    uint32_t     err_tx_2g = 0xffffffff;

    uint8_t  idac_tbl[150];
    uint8_t  capcode_tbl[150];
    uint32_t residual_cnt_tbl[150];



    uint32_t idac_tbl_tx_2g[79];

    uint32_t capcode_tbl_tx_2g[79];

    uint32_t residual_cnt;

    uint32_t p0;
    uint32_t p1;


    uint32_t pre_acal_up;
    uint32_t curr_acal_up;
    uint8_t  pre_acal_up_vld;
    uint8_t  seq_acal_jump_cnt ; //cnt for consecutive jump
    uint8_t  seq_acal_ful_cnt ;  //cnt for consecutive all0 all1

    uint32_t reg_data;

    RF_PRINTF("begin edr fulcal\n");

    RF_PRINTF("begin EDR LO cal\n");

    //hwp_pmuc->HXT_CR1 &= ~PMUC_HXT_CR1_CBANK_SEL_Msk;
    //hwp_pmuc->HXT_CR1 |= 0x1EA << PMUC_HXT_CR1_CBANK_SEL_Pos;
    //hwp_pmuc->HXT_CR1 |= 0xF << PMUC_HXT_CR1_LDO_VREF_Pos;

    hwp_bt_rfc->INCCAL_REG2 &= ~BT_RFC_INCCAL_REG2_EDR_INCACAL_EN ;
    hwp_bt_rfc->INCCAL_REG2 &= ~BT_RFC_INCCAL_REG2_EDR_INCFCAL_EN ;
    hwp_bt_rfc->EDR_PLL_REG7 |= BT_RFC_EDR_PLL_REG7_BT_IDAC_FORCE_EN  | BT_RFC_EDR_PLL_REG7_BT_PDX_FORCE_EN;
    hwp_bt_rfc->RF_LODIST_REG |= BT_RFC_RF_LODIST_REG_BRF_EN_RFBG_LV | BT_RFC_RF_LODIST_REG_BRF_EN_VDDPSW_LV ;

    hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VL_SEL_LV_Msk;
    hwp_bt_rfc->EDR_PLL_REG2 |= 0x7 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VL_SEL_LV_Pos;
    //LO full ACAL
    RF_PRINTF("begin LO acal\n");
    hwp_bt_rfc->EDR_PLL_REG1 |= BT_RFC_EDR_PLL_REG1_BRF_EDR_EN_VCO3G_LV ;
    hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_FKCAL_EN_LV;
    //VCO_ACAL_SEL ??
    hwp_bt_rfc->EDR_PLL_REG4 |= BT_RFC_EDR_PLL_REG4_BRF_EDR_LO_OPEN_LV;
    //hwp_bt_phy->TX_HFP_CFG &= ~( BT_PHY_TX_HFP_CFG_HFP_FCW_Msk );
    //hwp_bt_phy->TX_HFP_CFG |= ( 0x07<< BT_PHY_TX_HFP_CFG_HFP_FCW_Pos );

    hwp_bt_rfc->EDR_CAL_REG1 &= ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
    hwp_bt_rfc->EDR_CAL_REG1 |= (0x40) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;

    acal_cnt    = 0x40;
    acal_cnt_fs = 0x40;

    //wait 4us

    //acal binary search
    for (i = 1; i < 7; i++)
    {
        RF_PRINTF("begin EDR LO acal binary search\n");
        //
        RF_PRINTF("pre acal_cnt = %d\n", acal_cnt);
        RF_PRINTF("step = %d\n", acal_cnt_fs >> i);
        if (!(hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO3G_ACAL_INCAL_LV_Msk))
            break;
        else if (!(hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO3G_ACAL_UP_LV_Msk))
            acal_cnt = acal_cnt - (acal_cnt_fs >> i) ;
        else  if (hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO3G_ACAL_UP_LV_Msk)
            acal_cnt = acal_cnt + (acal_cnt_fs >> i)  ;
        hwp_bt_rfc->EDR_CAL_REG1 &= ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
        //wait 1us
        RF_PRINTF("acal_cnt = %d\n", acal_cnt);

    }
    hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;

    //LO full fcal
    RF_PRINTF("begin EDR LO fcal\n");
    fcal_cnt    = 0x80;
    fcal_cnt_fs = 0x80;
    hwp_bt_rfc->EDR_PLL_REG5 |= BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_EN_LV ;
    hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_FKCAL_EN_LV;
    hwp_bt_rfc->EDR_PLL_REG6 &= ~BT_RFC_EDR_PLL_REG6_BRF_EDR_FKCAL_CNT_DIVN_LV ;
    hwp_bt_rfc->EDR_PLL_REG6 |= 8160 << BT_RFC_EDR_PLL_REG6_BRF_EDR_FKCAL_CNT_DIVN_LV_Pos;
    //set lfp_fcw
    hwp_bt_phy->TX_LFP_CFG &= ~(BT_PHY_TX_LFP_CFG_BT_LFP_FCW_Msk);
    hwp_bt_phy->TX_LFP_CFG |= (0x08 << BT_PHY_TX_LFP_CFG_BT_LFP_FCW_Pos);
    hwp_bt_phy->TX_LFP_CFG    &= (~BT_PHY_TX_LFP_CFG_BT_LFP_FCW_SEL);

    hwp_bt_rfc->EDR_CAL_REG1 &= ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk;
    hwp_bt_rfc->EDR_CAL_REG1 |= (0x80) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos;
    //hwp_bt_phy->TX_HFP_CFG &= ~( BT_PHY_TX_HFP_CFG_HFP_FCW_Msk );
    //hwp_bt_phy->TX_HFP_CFG |= ( 0x07<< BT_PHY_TX_HFP_CFG_HFP_FCW_Pos );
    //hwp_bt_phy->TX_HFP_CFG &=  ~BT_PHY_TX_HFP_CFG_HFP_FCW_SEL;
    //hwp_bt_rfc->DCO_REG1 |= BT_RFC_DCO_REG1_BRF_EN_2M_MOD_LV;

    hwp_bt_rfc->EDR_PLL_REG5 |=  BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;
    hwp_bt_rfc->EDR_PLL_REG5 &= ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;

    hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV_Msk;
    hwp_bt_rfc->EDR_PLL_REG5 |=   BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV;

    hwp_bt_rfc->MISC_CTRL_REG |= BT_RFC_MISC_CTRL_REG_EDR_XTAL_REF_EN | BT_RFC_MISC_CTRL_REG_EDR_XTAL_REF_EN_FRC_EN ;
    hwp_bt_rfc->EDR_PLL_REG3  |= BT_RFC_EDR_PLL_REG3_BRF_EDR_PFDCP_EN_LV ;


    hwp_bt_rfc->EDR_CAL_REG1 &=            ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
    hwp_bt_rfc->EDR_CAL_REG1 |= (0x40) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;

    //pdx binary search
    //should store the cnt value of last pdx, so loop 8 times
    for (i = 1; i < 9; i++)
    {
        RF_PRINTF("begin EDR LO fcal binary search\n");
        //--------full acal in full fcal --------
        //{{{
        acal_cnt    = 0x40;
        acal_cnt_fs = 0x40;

        hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
        hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
        //wait 4us

        //acal binary search
        for (j = 1; j < 7; j++)
        {
            //
            if (!(hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO3G_ACAL_INCAL_LV_Msk))
                break;
            else if (!(hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO3G_ACAL_UP_LV_Msk))
                acal_cnt = acal_cnt - (acal_cnt_fs >> j) ;
            else  if (hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO3G_ACAL_UP_LV_Msk)
                acal_cnt = acal_cnt + (acal_cnt_fs >> j) ;
            hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
            hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
            //wait 1us

        }
        hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
        //}}}

        hwp_bt_rfc->EDR_PLL_REG5 |= 1 << BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV_Pos;



        while (!(hwp_bt_rfc->EDR_PLL_REG5 &  BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RDY_LV));
        residual_cnt  =  hwp_bt_rfc->EDR_PLL_REG6 & BT_RFC_EDR_PLL_REG6_BRF_EDR_FKCAL_CNT_OP_LV_Msk ;
        //residual_cnt  = residual_cnt >> BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;

        //RF_PRINTF( "residual_cnt = %d,cnt_vth = %d\n",residual_cnt,residual_cnt_vth );
        if (residual_cnt > residual_cnt_vth_2g)
        {
            idac1    = acal_cnt;
            p1       = residual_cnt ;
            error1   = residual_cnt - residual_cnt_vth_2g ;
            capcode1 = fcal_cnt;
            fcal_cnt = fcal_cnt + (fcal_cnt_fs >> i) ;
        }
        else if (residual_cnt <= residual_cnt_vth_2g)
        {
            idac0    = acal_cnt;
            p0       = residual_cnt ;
            error0   = residual_cnt_vth_2g - residual_cnt ;
            capcode0 = fcal_cnt;
            fcal_cnt = fcal_cnt - (fcal_cnt_fs >> i) ;
        }
        //RF_PRINTF( "fcal bin fcal_cnt = %x,acal_cnt = %x\n",fcal_cnt,acal_cnt );
        hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV;
        hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (fcal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos;
    }

    //RF_PRINTF( "sweep start idac0 = %x,capcode0 = %x\n",idac0,capcode0 );
    //RF_PRINTF( "sweep start idac1 = %x,capcode1 = %x\n",idac1,capcode1 );
    //RF_PRINTF( "sweep start error0 = %x,error1 = %x\n",error0,error1 );
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

    hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk;
    hwp_bt_rfc->EDR_CAL_REG1 |= (fcal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos;
    hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV;
    //hwp_bt_rfc->FBDV_REG1 &= ~BT_RFC_FBDV_REG1_BRF_FKCAL_CNT_EN_LV;

    fcal_cnt = capcode_tbl[0] ;
    acal_cnt = idac_tbl[0] ;
    //RF_PRINTF( "sweep start fcal_cnt = %x,acal_cnt = %x\n",fcal_cnt,acal_cnt );
    //sweep pdx until 3203M
    i = 0;

    do
    {
        i                     +=  1 ;
        fcal_cnt              +=  1 ;
        seq_acal_jump_cnt      =  0 ;
        seq_acal_ful_cnt       =  0 ;
        pre_acal_up_vld        =  0 ;
        hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (fcal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos;
        //seq acal {{{
        hwp_bt_rfc->EDR_PLL_REG2 |= BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
        //VCO_ACAL_SEL ??
        hwp_bt_rfc->EDR_PLL_REG4 |= BT_RFC_EDR_PLL_REG4_BRF_EDR_LO_OPEN_LV;
        while ((seq_acal_jump_cnt < 4) & (seq_acal_ful_cnt < 2))
        {
            hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
            hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
            //wait for 4us
            //for(int j=0;j<100;j++)
            //RF_PRINTF( "wait idac settling\n" );
            if (!(hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO3G_ACAL_INCAL_LV_Msk))
                break;
            curr_acal_up = hwp_bt_rfc->EDR_PLL_REG2 & BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO3G_ACAL_UP_LV_Msk;
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
        hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_EN_LV;
        ///}}}

        hwp_bt_rfc->EDR_CAL_REG1 &=                ~BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk;
        hwp_bt_rfc->EDR_CAL_REG1 |= (acal_cnt) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos;
        //wait for 4us
        //for(int j=0;j<100;j++)
        //RF_PRINTF( "wait idac settling\n" );
        _HAL_Delay_us(4);
        hwp_bt_rfc->EDR_PLL_REG5 &= ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV_Msk;

        hwp_bt_rfc->EDR_PLL_REG5 &=  ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV_Msk;
        hwp_bt_rfc->EDR_PLL_REG5 |=   BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RSTB_LV;
        hwp_bt_rfc->EDR_PLL_REG5 |= 1 << BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_EN_LV_Pos;
        //for( j=0;j<100;j++)
        //RF_PRINTF( "wait idac settling\n" );
        _HAL_Delay_us(10);
        while (!(hwp_bt_rfc->EDR_PLL_REG5 &  BT_RFC_EDR_PLL_REG5_BRF_EDR_FKCAL_CNT_RDY_LV));
        residual_cnt  =  hwp_bt_rfc->EDR_PLL_REG6 & BT_RFC_EDR_PLL_REG6_BRF_EDR_FKCAL_CNT_OP_LV_Msk ;
        //residual_cnt  = residual_cnt >> BT_RFC_FBDV_REG2_BRF_FKCAL_CNT_OP_LV_Pos ;
        //RF_PRINTF( "residual_cnt = %d, residual_cnt_vtl=%d\n", residual_cnt, residual_cnt_vtl );

        if (residual_cnt <= residual_cnt_vtl_2g)
            break;

        idac_tbl[i]         = acal_cnt ;
        capcode_tbl[i]      = fcal_cnt ;
        residual_cnt_tbl[i] = residual_cnt ;
    }
    while (residual_cnt > residual_cnt_vtl_2g) ;

    hwp_bt_rfc->EDR_PLL_REG2 &= ~BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_FKCAL_EN_LV;

    //search result for each channel
    sweep_num   = i ;
    //for bt 79 chnl
    for (j = 0; j < 79; j++)
    {
        err_tx_2g    = 0 ;
        for (i = 0; i < sweep_num; i++)
        {
            if (i == 0)
            {
                if (ref_residual_cnt_tbl_tx_2g[j] > residual_cnt_tbl[i])
                {
                    err_tx_2g     =  ref_residual_cnt_tbl_tx_2g[j]    - residual_cnt_tbl[i] ;
                }
                else
                {
                    err_tx_2g     =  residual_cnt_tbl[i] - ref_residual_cnt_tbl_tx_2g[j]    ;
                }
                idac_tbl_tx_2g[j]       =  idac_tbl[i];
                capcode_tbl_tx_2g[j]    =  capcode_tbl[i];
            }
            else
            {
                if (ref_residual_cnt_tbl_tx_2g[j] > residual_cnt_tbl[i])
                {
                    error0     =  ref_residual_cnt_tbl_tx_2g[j]    - residual_cnt_tbl[i] ;
                    if (error0 < err_tx_2g)
                    {
                        err_tx_2g = error0;
                        idac_tbl_tx_2g[j]       =  idac_tbl[i];
                        capcode_tbl_tx_2g[j]    =  capcode_tbl[i];
                    }
                }
                else
                {
                    error0     = residual_cnt_tbl[i] - ref_residual_cnt_tbl_tx_2g[j] ;
                    if (error0 < err_tx_2g)
                    {
                        err_tx_2g = error0;
                        idac_tbl_tx_2g[j]       =  idac_tbl[i];
                        capcode_tbl_tx_2g[j]    =  capcode_tbl[i];
                    }
                }
            }
        }
    }

    //write to rf_mem
    uint32_t reg_addr = rslt_start_addr ;
    reg_data = 0;
    hwp_bt_rfc->CAL_ADDR_REG2 &= 0xffff;
    hwp_bt_rfc->CAL_ADDR_REG2 += reg_addr << 16;
    for (i = 0; i < 79; i++)
    {
        //store bttx cal result
        reg_data = ((idac_tbl_tx_2g[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos) + (capcode_tbl_tx_2g[i] << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos)) ;
        write_memory(reg_addr + BT_RFC_MEM_BASE, reg_data);
        reg_addr += 4;
    }


    //after calibration
    hwp_bt_rfc->RF_LODIST_REG &= ~BT_RFC_RF_LODIST_REG_BRF_EN_RFBG_LV ;
    hwp_bt_rfc->RF_LODIST_REG &= ~BT_RFC_RF_LODIST_REG_BRF_EN_VDDPSW_LV ;


    //phy init, placed here for temp
    //hwp_bt_phy->DEMOD_CFG1 &= ~BT_PHY_DEMOD_CFG1_MU_ERR_Msk;
    //hwp_bt_phy->DEMOD_CFG1 &= ~BT_PHY_DEMOD_CFG1_MU_DC_Msk;
    //hwp_bt_phy->DEMOD_CFG1 |= 0xB8 << BT_PHY_DEMOD_CFG1_MU_ERR_Pos;
    //hwp_bt_phy->DEMOD_CFG1 |= 0x13 << BT_PHY_DEMOD_CFG1_MU_DC_Pos;
    //hwp_bt_phy->TX_GAUSSFLT_CFG &= ~BT_PHY_TX_GAUSSFLT_CFG_GAUSS_GAIN_2_Msk;
    //hwp_bt_phy->TX_GAUSSFLT_CFG |= 0xff << BT_PHY_TX_GAUSSFLT_CFG_GAUSS_GAIN_2_Pos;
    //
    //hwp_bt_phy->NOTCH_CFG3 = 0x00800000 ;
    //hwp_bt_phy->NOTCH_CFG1 &= ~BT_PHY_NOTCH_CFG1_NOTCH_B1_Msk;
    //hwp_bt_phy->NOTCH_CFG1 |= 0x3000 << BT_PHY_NOTCH_CFG1_NOTCH_B1_Pos;

    return reg_addr;
}

#endif


#define BT_RFC_TXDC_DMA_ADDR 0x20418000
uint32_t bt_rfc_txdc_cal(uint32_t rslt_start_addr)
{

    uint16_t  i, j;
    uint32_t data;
    uint16_t coef0[9];
    uint16_t coef1[9];
    uint16_t offset_i[9];
    uint16_t offset_q[9];
    uint32_t rcc_reg, div;
    uint32_t tmxcap_sel[79];


    // Change xtal to 24M
    rcc_reg = READ_REG(hwp_lpsys_rcc->CFGR);
    div = (rcc_reg & LPSYS_RCC_CFGR_HDIV1_Msk) >> LPSYS_RCC_CFGR_HDIV1_Pos;

    HAL_RCC_LCPU_SetDiv(2, -1, -1);

#if 0

    //////////////////////////////////////////////////////////////////////
    //settings for zero crc error @13dBm
    //////////////////////////////////////////////////////////////////////
    hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV;
    hwp_bt_rfc->TRF_EDR_REG1 |= 0x1f << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos;
    hwp_bt_phy->TX_IF_MOD_CFG2 &=  ~BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_BLE_FRC;
    hwp_bt_phy->TX_IF_MOD_CFG2 |= 0x80 << BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_BLE_FRC_Pos;
    hwp_bt_phy->TX_IF_MOD_CFG2 &=  ~BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_BR_FRC;
    hwp_bt_phy->TX_IF_MOD_CFG2 |= 0x80 << BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_BR_FRC_Pos;
    hwp_bt_phy->TX_IF_MOD_CFG2 &=  ~BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_EDR_FRC;
    hwp_bt_phy->TX_IF_MOD_CFG2 |= 0x50 << BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_EDR_FRC_Pos;
    hwp_bt_phy->TX_DPSK_CFG1 &= ~BT_PHY_TX_DPSK_CFG1_TX_DPSK_GAIN_FRC;
    hwp_bt_phy->TX_DPSK_CFG1 |= 0x50 << BT_PHY_TX_DPSK_CFG1_TX_DPSK_GAIN_FRC_Pos;
    hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0x66666666;
    hwp_bt_phy->TX_EDR_LPF_CFG &= ~BT_PHY_TX_EDR_LPF_CFG_TX_EDR_LPF_BYPASS;


    //set rcos bw to 0.7M
    write_memory(0x50084190, 0x00001000);
    write_memory(0x50084194, 0x00002001);
    write_memory(0x50084198, 0x00FFD000);
    write_memory(0x5008419C, 0x00FF1FF7);
    write_memory(0x500841A0, 0x00FECFED);
    write_memory(0x500841A4, 0x00FF8FEF);
    write_memory(0x500841A8, 0x00011004);
    write_memory(0x500841AC, 0x0001F01B);
    write_memory(0x500841B0, 0x0001101D);
    write_memory(0x500841B4, 0x00FDFFFC);
    write_memory(0x500841B8, 0x00FA0FBF);
    write_memory(0x500841BC, 0x00F7FF88);
    write_memory(0x500841C0, 0x00FB2F8B);
    write_memory(0x500841C4, 0x0005BFF7);
    write_memory(0x500841C8, 0x001740DC);
    write_memory(0x500841CC, 0x002C621B);
    write_memory(0x500841D0, 0x003F7368);
    write_memory(0x500841D4, 0x004AC465);
    write_memory(0x500841D8, 0x000004C4);
#endif


    //////////////////////////////////////////////////////////////////////
    //lock LO @2440MHz
    //////////////////////////////////////////////////////////////////////

    //set BR tx to IQ modulation
    hwp_bt_phy->TX_CTRL |= BT_PHY_TX_CTRL_MOD_METHOD_BR ;
    hwp_bt_phy->TX_CTRL |= BT_PHY_TX_CTRL_MOD_METHOD_BLE | BT_PHY_TX_CTRL_MOD_METHOD_BR | BT_PHY_TX_CTRL_MOD_METHOD_EDR ;

    //force to BR mode
    hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_NBT_BLE;
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_NBT_BLE_VAL;

    //force channel to 2440MHz
    hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_CHANNEL | BT_MAC_DMRADIOCNTL1_FORCE_SYNCWORD;
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_CHANNEL;
    hwp_bt_mac->DMRADIOCNTL1 |= 38 << BT_MAC_DMRADIOCNTL1_CHANNEL_Pos;

    //enable sine wave tx and dc cal module
    hwp_bt_phy->TX_DC_CAL_CFG0 |= BT_PHY_TX_DC_CAL_CFG0_TX_DC_CAL_EN ;

    //disable adc_q
    hwp_bt_phy->RX_CTRL1 &= ~BT_PHY_RX_CTRL1_ADC_Q_EN_1;

    //force rx on
    //hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_RX;
    //hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_RX_VAL;

    //temp setting for 12dBm tx
    //write_memory(0x50082858, 0x30F35DCA);   //EDR PA
    //write_memory(0x50082858, 0x30F35DFE);   //EDR PA,for 13dBm
    //write_memory(0x5008285c, 0x0002FC80);   //EDR PA

    //write_memory(0x50084130, 0x00000142);   //FORCE DPSK_GAIN
    //write_memory(0x50084100, 0x06424280);   //FORCE MOD_GAIN_EDR
    //write_memory(0x5008412C, 0x66666666);   //FORCE EDR_TMX_BUF_GC_CFG2

    hwp_bt_rfc->EDR_PLL_REG3 &= ~BT_RFC_EDR_PLL_REG3_BRF_EDR_PFDCP_ICP_SET_LV;
    hwp_bt_rfc->EDR_PLL_REG3 |= 2 << BT_RFC_EDR_PLL_REG3_BRF_EDR_PFDCP_ICP_SET_LV_Pos;
    hwp_bt_rfc->EDR_PLL_REG4 &= ~(BT_RFC_EDR_PLL_REG4_BRF_EDR_LPF_RZ_SET_LV | \
                                  BT_RFC_EDR_PLL_REG4_BRF_EDR_LPF_RP4_SET_LV | \
                                  BT_RFC_EDR_PLL_REG4_BRF_EDR_LPF_CZ_SET_LV | \
                                  BT_RFC_EDR_PLL_REG4_BRF_EDR_LPF_CP3_SET_LV
                                 );
    hwp_bt_rfc->EDR_PLL_REG4 |= 4 << BT_RFC_EDR_PLL_REG4_BRF_EDR_LPF_RZ_SET_LV_Pos | \
                                5 << BT_RFC_EDR_PLL_REG4_BRF_EDR_LPF_RP4_SET_LV_Pos | \
                                2 << BT_RFC_EDR_PLL_REG4_BRF_EDR_LPF_CZ_SET_LV_Pos | \
                                2 << BT_RFC_EDR_PLL_REG4_BRF_EDR_LPF_CP3_SET_LV_Pos;


    //enable rx path
    //hwp_bt_rfc->TRF_EDR_REG2 &= ~BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_EN_LV ; // //relocated
    hwp_bt_rfc->TRF_EDR_REG2 &= ~(BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_BM_LV | BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_GC_LV);
    hwp_bt_rfc->TRF_EDR_REG2 |= 0x0 << BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_GC_LV_Pos;
    hwp_bt_rfc->TRF_EDR_REG2 |= 0x3 << BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_BM_LV_Pos ;
    hwp_bt_rfc->TRF_EDR_REG2 |= BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_OS_LV | BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_OS_PN_LV;

    hwp_bt_rfc->RBB_REG2 &= ~BT_RFC_RBB_REG2_BRF_RVGA_GC_LV;
    hwp_bt_rfc->RBB_REG2 |= 0xC << BT_RFC_RBB_REG2_BRF_RVGA_GC_LV_Pos;
    hwp_bt_rfc->AGC_REG |= BT_RFC_AGC_REG_VGA_GAIN_FRC_EN;

    //setting to fix pwrmtr
    hwp_bt_rfc->RBB_REG3 &= ~BT_RFC_RBB_REG3_BRF_RVGA_VCMREF_LV & ~BT_RFC_RBB_REG3_BRF_RVGA_VSTART_LV;


    //hwp_bt_rfc->RBB_REG1 |= BT_RFC_RBB_REG1_BRF_EN_LDO_RBB_LV;

#if 0
    hwp_bt_rfc->RBB_REG2 |= BT_RFC_RBB_REG2_BRF_EN_RVGA_I_LV ;
    hwp_bt_rfc->RBB_REG5 |= BT_RFC_RBB_REG5_BRF_EN_IARRAY_LV;
    hwp_bt_rfc->ADC_REG  |= BT_RFC_ADC_REG_BRF_EN_ADC_I_LV | BT_RFC_ADC_REG_BRF_EN_LDO_ADCREF_LV | BT_RFC_ADC_REG_BRF_EN_LDO_ADC_LV;

    _HAL_Delay_us(5);
    hwp_bt_rfc->RBB_REG5 |= BT_RFC_RBB_REG5_BRF_RVGA_TX_LPBK_EN_LV;

    //move here to avoid conflict with RVGA_EN
    hwp_bt_rfc->TRF_EDR_REG2 |= BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_EN_LV ;
#endif

    hwp_bt_rfc->MISC_CTRL_REG |= BT_RFC_MISC_CTRL_REG_ADC_CLK_EN_FRC_EN | BT_RFC_MISC_CTRL_REG_ADC_CLK_EN;

    hwp_bt_phy->DCCAL_MPT_CFG |= BT_PHY_DCCAL_MPT_CFG_TX_DC_CAL | BT_PHY_DCCAL_MPT_CFG_DC_EST_EN;
    //hwp_bt_phy->DCCAL_MPT_CFG &= ~BT_PHY_DCCAL_MPT_CFG_DC_EST_MU;
    hwp_bt_phy->RX_CTRL1 |= BT_PHY_RX_CTRL1_FORCE_RX_ON;

    //PHY DUMP and DMAC setting
    hwp_bt_phy->RX_CTRL1 |= (1 << BT_PHY_RX_CTRL1_PHY_RX_DUMP_EN_Pos) | (0x0 << BT_PHY_RX_CTRL1_RX_DUMP_DATA_SEL_Pos) ;
    hwp_dmac2->CPAR8  = 0x500c0000;
    hwp_dmac2->CM0AR8 = BT_RFC_TXDC_DMA_ADDR;

    hwp_dmac2->CCR8 = DMAC_CCR8_MINC | (0x2 << DMAC_CCR8_MSIZE_Pos) | (0x2 << DMAC_CCR8_PSIZE_Pos);
    hwp_dmac2->CCR8 |= DMAC_CCR8_MEM2MEM | DMAC_CCR8_EN | DMAC_CCR8_TCIE;

    //froce tx on, lock LO
    hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_TX ;
    hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL;
    _HAL_Delay_us(40);

    //set clk, dmac2 hclk set to 24M to dump adc data
    hwp_lpsys_rcc->CSR &= ~LPSYS_RCC_CSR_SEL_SYS;
    hwp_lpsys_rcc->CSR |= 1 << LPSYS_RCC_CSR_SEL_SYS_Pos;
    hwp_lpsys_rcc->CFGR &= ~(LPSYS_RCC_CFGR_HDIV1 | LPSYS_RCC_CFGR_HDIV2);
    hwp_lpsys_rcc->CFGR |= 2 << LPSYS_RCC_CFGR_HDIV1_Pos;
    //force mod gain & dpsk gain

    hwp_bt_phy->TX_IF_MOD_CFG2 |= BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_BR_FRC_EN;
    hwp_bt_phy->TX_DPSK_CFG1   |= BT_PHY_TX_DPSK_CFG1_TX_DPSK_GAIN_FRC_EN;


    int64_t mixer_pwr;
    int64_t mixer_pwr_i;
    int64_t mixer_pwr_q;
    int64_t mixer_i_sum = 0;
    int64_t mixer_q_sum = 0;
    int64_t dc_out_min = 0x7fffffffffffffff;
    //uint32_t dc_out;
    int mixer_i;
    int mixer_q;
    int mixer_cos;
    int mixer_sin;
    uint16_t adc_data;
    int cos_table[16] = {1024, 946, 724, 392, 0, -392, -724, -946, -1024, -946, -724, -392, 0, 392, 724, 946};
    int sin_table[16] = {0, -392, -724, -946, -1024, -946, -724, -392, 0, 392, 724, 946, 1024, 946, 724, 392};
    uint32_t mem_data;
    int k, m;

    //int64_t dc_out[512];
#if defined(ENABLE_RF_ATE)
    for (i = 6; i < 7; i++)
#else
    for (i = 0; i < 7; i++)//cal edr 0/3/6/9/13/16/19 dBm level
#endif
    {
        //set power level
        if (i == 0)
        {
            //level 0 : edr tx 0dBm
            hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_TMXBUF_IBLD_LV;
            hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV;
            hwp_bt_rfc->TRF_EDR_REG1 |= 0x0 << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos;
            hwp_bt_phy->TX_DC_CAL_CFG2 = 0x70;

            hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0x33333333;

            hwp_bt_rfc->RBB_REG2 &= ~BT_RFC_RBB_REG2_BRF_RVGA_GC_LV;
            hwp_bt_rfc->RBB_REG2 |= 0x10 << BT_RFC_RBB_REG2_BRF_RVGA_GC_LV_Pos;
            hwp_bt_rfc->TRF_EDR_REG2 &= ~BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_GC_LV;
        }
        else if (i == 1)
        {

            //level 1 : edr tx 3dBm
            hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV;
            hwp_bt_rfc->TRF_EDR_REG1 |= 0x5 << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos;
            hwp_bt_phy->TX_DC_CAL_CFG2 = 0x60;
            hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0x33333333;

        }
        else if (i == 2)
        {

            //level 2 : edr tx 6dBm
            hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV;
            hwp_bt_rfc->TRF_EDR_REG1 |= 0xC << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos;
            hwp_bt_phy->TX_DC_CAL_CFG2 = 0x60;

            hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0x44444444;
            hwp_bt_rfc->RBB_REG2 &= ~BT_RFC_RBB_REG2_BRF_RVGA_GC_LV;
            hwp_bt_rfc->RBB_REG2 |= 0x4 << BT_RFC_RBB_REG2_BRF_RVGA_GC_LV_Pos;


        }
        else if (i == 3)
        {

            //level 3 : edr tx 9dBm
            hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV;
            hwp_bt_rfc->TRF_EDR_REG1 |= 0x14 << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos;
            hwp_bt_phy->TX_DC_CAL_CFG2 = 0x70;

            hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0x55555555;
            hwp_bt_rfc->RBB_REG2 &= ~BT_RFC_RBB_REG2_BRF_RVGA_GC_LV;
            hwp_bt_rfc->RBB_REG2 |= 0x10 << BT_RFC_RBB_REG2_BRF_RVGA_GC_LV_Pos;
            hwp_bt_rfc->TRF_EDR_REG2 &= ~BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_GC_LV;
            //hwp_bt_rfc->TRF_EDR_REG2 |= 1 << BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_GC_LV_Pos;


        }
        else if (i == 4)
        {

            //level 4 : edr tx 13dBm
            hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV;
            hwp_bt_rfc->TRF_EDR_REG1 |= 0x1B << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos; //set PA_BM to 0 to minimize current
            hwp_bt_phy->TX_DC_CAL_CFG2 = 0x50;

            hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0x77777777;

            hwp_bt_rfc->TRF_EDR_REG2 &= ~BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_GC_LV;
            //BRF_RVGA_GC_LV=0x10
            hwp_bt_rfc->RBB_REG2 &= ~BT_RFC_RBB_REG2_BRF_RVGA_GC_LV;
            hwp_bt_rfc->RBB_REG2 |= 0x10 << BT_RFC_RBB_REG2_BRF_RVGA_GC_LV_Pos;

        }
        /*13 dBm use the same setting
        else if (i == 5)
        {


        //level 5 : gfsk tx 13dBm
        hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV;
        hwp_bt_rfc->TRF_EDR_REG1 |= 0x1B << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos;

        hwp_bt_phy->TX_DC_CAL_CFG2 = 0x30;

        hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0x55555555;


        }
        */
        else if (i == 5)
        {

            //level 6 : gfsk tx 16dBm
            hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV;
            hwp_bt_rfc->TRF_EDR_REG1 |= 0x1f << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos;

            hwp_bt_phy->TX_DC_CAL_CFG2 = 0x30;


            hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0x88888888;
            hwp_bt_rfc->RBB_REG2 &= ~BT_RFC_RBB_REG2_BRF_RVGA_GC_LV;

        }
        else if (i == 6)
        {


            //level 7 : gfsk tx 19dBm
            hwp_bt_rfc->TRF_EDR_REG1 &= ~(BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV |  BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_TMXBUF_IBLD_LV);
            hwp_bt_rfc->TRF_EDR_REG1 |= 0x1F << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos;

            hwp_bt_phy->TX_DC_CAL_CFG2 = 0x18;
            hwp_bt_phy->EDR_TMXBUF_GC_CFG2 &= ~BT_PHY_EDR_TMXBUF_GC_CFG2_EDR_TMXBUF_GC_0 ;

            hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0xFFFFFFFF;

            hwp_bt_rfc->TRF_EDR_REG2 &= ~BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_GC_LV & \
                                        ~BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_BM_LV;
            hwp_bt_rfc->TRF_EDR_REG2 |= 0 << BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_GC_LV_Pos |
                                        1 << BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_BM_LV_Pos;
            //BRF_RVGA_GC_LV=0
            hwp_bt_rfc->RBB_REG2 &= ~BT_RFC_RBB_REG2_BRF_RVGA_GC_LV;
            hwp_bt_rfc->RBB_REG2 |= 6 << BT_RFC_RBB_REG2_BRF_RVGA_GC_LV_Pos;

        }
        else if (i == 7)
        {
            //level 8 : edr tx 13dBm for test
            //settings for test mode
            hwp_bt_rfc->EDR_PLL_REG1 &= ~BT_RFC_EDR_PLL_REG1_BRF_EDR_VCO_LDO_VREF_LV;
            hwp_bt_rfc->EDR_PLL_REG1 |= 0x8 << BT_RFC_EDR_PLL_REG1_BRF_EDR_VCO_LDO_VREF_LV_Pos;

            hwp_bt_rfc->EDR_PLL_REG2 &= ~(BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VL_SEL_LV_Msk | BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VH_SEL_LV_Msk | \
                                          BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_INCFCAL_VH_SEL_LV_Msk | BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_INCFCAL_VL_SEL_LV_Msk);
            hwp_bt_rfc->EDR_PLL_REG2 |= 0x0 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VL_SEL_LV_Pos | \
                                        0x2 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VH_SEL_LV_Pos | \
                                        0x0 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_INCFCAL_VL_SEL_LV_Pos | \
                                        0x2 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_INCFCAL_VH_SEL_LV_Pos;

            hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV & ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_TMXBUF_IBLD_LV & \
                                        ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_LOBIAS_BM_LV & ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PACAS_BM_LV;
            hwp_bt_rfc->TRF_EDR_REG1 |= 0x1F << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos | \
                                        0x2  << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PACAS_BM_LV_Pos;

            hwp_bt_phy->TX_IF_MOD_CFG2 &=  ~BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_BR_FRC;
            hwp_bt_phy->TX_IF_MOD_CFG2 |= 0x50 << BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_BR_FRC_Pos;
            hwp_bt_phy->TX_DPSK_CFG1 &= ~BT_PHY_TX_DPSK_CFG1_TX_DPSK_GAIN_FRC;
            hwp_bt_phy->TX_DPSK_CFG1 |= 0x80 << BT_PHY_TX_DPSK_CFG1_TX_DPSK_GAIN_FRC_Pos;
            hwp_bt_phy->TX_DC_CAL_CFG2 = 0x50;
            hwp_bt_phy->EDR_TMXBUF_GC_CFG2 &= ~BT_PHY_EDR_TMXBUF_GC_CFG2_EDR_TMXBUF_GC_0 ;

            hwp_bt_phy->TX_EDR_LPF_CFG &= ~BT_PHY_TX_EDR_LPF_CFG_TX_EDR_LPF_BYPASS;

            hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0x66666666;

            //set rcos bw to 0.4M
            //write_memory(0x50084180, 0x00FFF000);
            //write_memory(0x50084184, 0x00FFCFFD);
            //write_memory(0x50084188, 0x00FFEFFC);
            //write_memory(0x5008418C, 0x00010005);
            //write_memory(0x50084190, 0x0002B01D);
            //write_memory(0x50084194, 0x0003C037);
            //write_memory(0x50084198, 0x0002B038);
            //write_memory(0x5008419C, 0x00FF8015);
            //write_memory(0x500841A0, 0x00FAFFD5);
            //write_memory(0x500841A4, 0x00F69F8A);
            //write_memory(0x500841A8, 0x00F48F51);
            //write_memory(0x500841AC, 0x00F6DF50);
            //write_memory(0x500841B0, 0x00FECFA1);
            //write_memory(0x500841B4, 0x000C304E);
            //write_memory(0x500841B8, 0x001D5148);
            //write_memory(0x500841BC, 0x002F1265);
            //write_memory(0x500841C0, 0x003DB36F);
            //write_memory(0x500841C4, 0x0045F42C);
            //write_memory(0x500841C8, 0x00000470);
            //set rcos bw to 0.7M
            write_memory(0x50084180, 0x00001000);
            write_memory(0x50084184, 0x00002001);
            write_memory(0x50084188, 0x00FFD000);
            write_memory(0x5008418C, 0x00FF1FF7);
            write_memory(0x50084190, 0x00FECFED);
            write_memory(0x50084194, 0x00FF8FEF);
            write_memory(0x50084198, 0x00011004);
            write_memory(0x5008419C, 0x0001F01B);
            write_memory(0x500841a0, 0x0001101D);
            write_memory(0x500841a4, 0x00FDFFFC);
            write_memory(0x500841a8, 0x00FA0FBF);
            write_memory(0x500841aC, 0x00F7FF88);
            write_memory(0x500841b0, 0x00FB2F8B);
            write_memory(0x500841b4, 0x0005BFF7);
            write_memory(0x500841b8, 0x001740DC);
            write_memory(0x500841bC, 0x002C621B);
            write_memory(0x500841c0, 0x003F7368);
            write_memory(0x500841c4, 0x004AC465);
            write_memory(0x500841c8, 0x000004C4);
        }

        //hwp_bt_phy->EDR_TMXBUF_GC_CFG2 &= ~BT_PHY_EDR_TMXBUF_GC_CFG2_EDR_TMXBUF_GC_0 ;
        //hwp_bt_phy->EDR_TMXBUF_GC_CFG2 |= 7;


        //set rx mixer phase to 750K for dc offset cal
        hwp_bt_phy->MIXER_CFG1 &= ~BT_PHY_MIXER_CFG1_RX_MIXER_PHASE_1;
        hwp_bt_phy->MIXER_CFG1 |= 0x40 << BT_PHY_MIXER_CFG1_RX_MIXER_PHASE_1_Pos;//750KHz



        //fix coef
        hwp_bt_rfc->TXDC_CAL_REG1  = 0x10000000;
        hwp_bt_rfc->TXDC_CAL_REG2  = 0 ;

        dc_out_min = 0x7fffffffffffffff;
        //coarse search for offset i
        for (j = 0; j < 64; j++)
        {
            //set cal_offset_i;
            data = 0;
            data = j ;
            //data <<= 5;
            data += 0x7E0;
            data &= 0x7ff;
            hwp_bt_rfc->TXDC_CAL_REG2  &= ~BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_I;
            hwp_bt_rfc->TXDC_CAL_REG2  |= data << BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_I_Pos;

            hwp_dmac2->CNDTR8 = 512;
            while (!(hwp_dmac2->ISR & DMAC_ISR_TCIF8)) {};
            hwp_dmac2->IFCR |= DMAC_IFCR_CTCIF8;
            mixer_i_sum = 0;
            mixer_q_sum = 0;
            int phase_index = 0;
            for (k = 0; k < 512; k++)
            {

                mem_data = read_memory(BT_RFC_TXDC_DMA_ADDR + k * 4);
                //adc_data = (int)((mem_data&0x3ff)<<22);
                //adc_data = adc_data >> 22;
                adc_data = (mem_data & 0x3ff);

                mixer_cos = cos_table[phase_index];
                mixer_sin = sin_table[phase_index];
                phase_index = phase_index + 1;
                if (phase_index == 16)
                {
                    phase_index = 0;
                }

                mixer_i = adc_data * mixer_cos; //s20.19
                mixer_q = adc_data * mixer_sin; //s20.19
                mixer_i_sum += mixer_i;
                mixer_q_sum += mixer_q;
            }

            mixer_i_sum = mixer_i_sum / 512; //s20.19
            mixer_q_sum = mixer_q_sum / 512; //s20.19
            mixer_pwr = mixer_i_sum * mixer_i_sum + mixer_q_sum * mixer_q_sum; //u32.38
            //dc_out[j] = mixer_pwr  ;

            if (dc_out_min > mixer_pwr)
            {
                dc_out_min = mixer_pwr;
                offset_i[i] = data;
            }
        }

        //fix cal offset i
        hwp_bt_rfc->TXDC_CAL_REG2  &= ~BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_I;
        hwp_bt_rfc->TXDC_CAL_REG2  |= offset_i[i] << BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_I_Pos;
        dc_out_min = 0x7fffffffffffffff;
        for (j = 0; j < 64; j++)
        {
            //set cal_offset_q;
            data = 0;
            data += j ;
            //data <<= 5;
            data += 0x7e0;
            data &= 0x7ff;
            hwp_bt_rfc->TXDC_CAL_REG2  &= ~BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_Q;
            hwp_bt_rfc->TXDC_CAL_REG2  |= data << BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_Q_Pos;

            hwp_dmac2->CNDTR8 = 512;
            while (!(hwp_dmac2->ISR & DMAC_ISR_TCIF8)) {};
            hwp_dmac2->IFCR |= DMAC_IFCR_CTCIF8;
            mixer_i_sum = 0;
            mixer_q_sum = 0;
            int phase_index = 0;
            for (k = 0; k < 512; k++)
            {
                mem_data = read_memory(BT_RFC_TXDC_DMA_ADDR + k * 4);
                //adc_data = (int)((mem_data&0x3ff)<<22);
                //adc_data = adc_data >> 22;
                adc_data = (mem_data & 0x3ff);

                mixer_cos = cos_table[phase_index];
                mixer_sin = sin_table[phase_index];
                phase_index = phase_index + 1;
                if (phase_index == 16)
                {
                    phase_index = 0;
                }
                mixer_i = adc_data * mixer_cos; //s20.19
                mixer_q = adc_data * mixer_sin; //s20.19
                mixer_i_sum += mixer_i;
                mixer_q_sum += mixer_q;
            }

            mixer_i_sum = mixer_i_sum / 512; //s20.19
            mixer_q_sum = mixer_q_sum / 512; //s20.19
            mixer_pwr = mixer_i_sum * mixer_i_sum + mixer_q_sum * mixer_q_sum; //u32.38
            //dc_out[j] = mixer_pwr;
            if (dc_out_min > mixer_pwr)
            {
                dc_out_min = mixer_pwr;
                offset_q[i] = data;
            }
        }

        //fix offset i and q
        //hwp_bt_rfc->TXDC_CAL_REG2  &= ~BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_I;
        //hwp_bt_rfc->TXDC_CAL_REG2  &= ~BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_Q;
        hwp_bt_rfc->TXDC_CAL_REG2  &= ~BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_Q;
        hwp_bt_rfc->TXDC_CAL_REG2  |= offset_q[i] << BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_Q_Pos;

        //offset cal, 2nd

        dc_out_min = 0x7fffffffffffffff;
        //coarse search for offset i
        for (j = 0; j < 64; j++)
        {
            //set cal_offset_i;
            data = 0;
            data = j ;
            //data <<= 5;
            data += 0x7E0;
            data &= 0x7ff;
            hwp_bt_rfc->TXDC_CAL_REG2  &= ~BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_I;
            hwp_bt_rfc->TXDC_CAL_REG2  |= data << BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_I_Pos;

            hwp_dmac2->CNDTR8 = 512;
            while (!(hwp_dmac2->ISR & DMAC_ISR_TCIF8)) {};
            hwp_dmac2->IFCR |= DMAC_IFCR_CTCIF8;
            mixer_i_sum = 0;
            mixer_q_sum = 0;
            int phase_index = 0;
            for (k = 0; k < 512; k++)
            {

                mem_data = read_memory(BT_RFC_TXDC_DMA_ADDR + k * 4);
                //adc_data = (int)((mem_data&0x3ff)<<22);
                //adc_data = adc_data >> 22;
                adc_data = (mem_data & 0x3ff);

                mixer_cos = cos_table[phase_index];
                mixer_sin = sin_table[phase_index];
                phase_index = phase_index + 1;
                if (phase_index == 16)
                {
                    phase_index = 0;
                }

                mixer_i = adc_data * mixer_cos; //s20.19
                mixer_q = adc_data * mixer_sin; //s20.19
                mixer_i_sum += mixer_i;
                mixer_q_sum += mixer_q;
            }

            mixer_i_sum = mixer_i_sum / 512; //s20.19
            mixer_q_sum = mixer_q_sum / 512; //s20.19
            mixer_pwr = mixer_i_sum * mixer_i_sum + mixer_q_sum * mixer_q_sum; //u32.38
            //dc_out[j] = mixer_pwr  ;

            if (dc_out_min > mixer_pwr)
            {
                dc_out_min = mixer_pwr;
                offset_i[i] = data;
            }
        }

        //fix cal offset i
        hwp_bt_rfc->TXDC_CAL_REG2  &= ~BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_I;
        hwp_bt_rfc->TXDC_CAL_REG2  |= offset_i[i] << BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_I_Pos;
        dc_out_min = 0x7fffffffffffffff;
        for (j = 0; j < 64; j++)
        {
            //set cal_offset_q;
            data = 0;
            data += j ;
            //data <<= 5;
            data += 0x7e0;
            data &= 0x7ff;
            hwp_bt_rfc->TXDC_CAL_REG2  &= ~BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_Q;
            hwp_bt_rfc->TXDC_CAL_REG2  |= data << BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_Q_Pos;

            hwp_dmac2->CNDTR8 = 512;
            while (!(hwp_dmac2->ISR & DMAC_ISR_TCIF8)) {};
            hwp_dmac2->IFCR |= DMAC_IFCR_CTCIF8;
            mixer_i_sum = 0;
            mixer_q_sum = 0;
            int phase_index = 0;
            for (k = 0; k < 512; k++)
            {
                mem_data = read_memory(BT_RFC_TXDC_DMA_ADDR + k * 4);
                //adc_data = (int)((mem_data&0x3ff)<<22);
                //adc_data = adc_data >> 22;
                adc_data = (mem_data & 0x3ff);

                mixer_cos = cos_table[phase_index];
                mixer_sin = sin_table[phase_index];
                phase_index = phase_index + 1;
                if (phase_index == 16)
                {
                    phase_index = 0;
                }
                mixer_i = adc_data * mixer_cos; //s20.19
                mixer_q = adc_data * mixer_sin; //s20.19
                mixer_i_sum += mixer_i;
                mixer_q_sum += mixer_q;
            }

            mixer_i_sum = mixer_i_sum / 512; //s20.19
            mixer_q_sum = mixer_q_sum / 512; //s20.19
            mixer_pwr = mixer_i_sum * mixer_i_sum + mixer_q_sum * mixer_q_sum; //u32.38
            //dc_out[j] = mixer_pwr;
            if (dc_out_min > mixer_pwr)
            {
                dc_out_min = mixer_pwr;
                offset_q[i] = data;
            }
        }

        //fix offset i and q
        hwp_bt_rfc->TXDC_CAL_REG2  &= ~BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_I;
        hwp_bt_rfc->TXDC_CAL_REG2  &= ~BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_Q;
        hwp_bt_rfc->TXDC_CAL_REG2  |= offset_i[i] << BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_I_Pos;
        hwp_bt_rfc->TXDC_CAL_REG2  |= offset_q[i] << BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_Q_Pos;
        //set rx mixer phase for coef calibration
        hwp_bt_phy->MIXER_CFG1 &= ~BT_PHY_MIXER_CFG1_RX_MIXER_PHASE_1;
        hwp_bt_phy->MIXER_CFG1 |= 0x80 << BT_PHY_MIXER_CFG1_RX_MIXER_PHASE_1_Pos;//1.5MHz

        //fix coef1
        hwp_bt_rfc->TXDC_CAL_REG1  &= ~BT_RFC_TXDC_CAL_REG1_TX_DC_CAL_COEF1;
        hwp_bt_rfc->TXDC_CAL_REG1  |= 0x1000 << BT_RFC_TXDC_CAL_REG1_TX_DC_CAL_COEF1_Pos;
        dc_out_min    = 0x7fffffffffffffff;
        //coarse search for coef0
        for (j = 0; j < 512; j++)
        {
            //set coef_0;
            data =  0x3000 + (j << 4);
            data &= 0x3FFF;
            hwp_bt_rfc->TXDC_CAL_REG1  &= ~BT_RFC_TXDC_CAL_REG1_TX_DC_CAL_COEF0;
            hwp_bt_rfc->TXDC_CAL_REG1  |= data << BT_RFC_TXDC_CAL_REG1_TX_DC_CAL_COEF0_Pos;

            hwp_dmac2->CNDTR8 = 512;
            while (!(hwp_dmac2->ISR & DMAC_ISR_TCIF8)) {};
            hwp_dmac2->IFCR |= DMAC_IFCR_CTCIF8;
            mixer_i_sum = 0;
            mixer_q_sum = 0;
            int phase_index = 0;
            for (k = 0; k < 512; k++)
            {

                mem_data = read_memory(BT_RFC_TXDC_DMA_ADDR + k * 4);
                //adc_data = (int)((mem_data & 0x3ff) << 22);
                //adc_data = adc_data >> 22;
                adc_data = (mem_data & 0x3ff);
                mixer_cos = cos_table[phase_index];
                mixer_sin = sin_table[phase_index];
                phase_index = phase_index + 2;
                if (phase_index == 16)
                {
                    phase_index = 0;
                }
                mixer_i = adc_data * mixer_cos; //s20.19
                mixer_q = adc_data * mixer_sin; //s20.19
                mixer_i_sum += mixer_i;
                mixer_q_sum += mixer_q;
            }
            mixer_i_sum = mixer_i_sum / 512; //s20.19
            mixer_q_sum = mixer_q_sum / 512; //s20.19
            //mixer_pwr_i = mixer_i_sum*mixer_i_sum;
            //mixer_pwr_q = mixer_q_sum*mixer_q_sum;
            mixer_pwr = mixer_i_sum * mixer_i_sum + mixer_q_sum * mixer_q_sum; //u32.38
            //mixer_pwr = mixer_pwr_i + mixer_pwr_q;
            //if(mixer_pwr > 0x7fffff00000000)
            //  printf("overflow");
            //dc_out[j]=mixer_pwr;

            if (dc_out_min > mixer_pwr)
            {
                dc_out_min = mixer_pwr;
                coef0[i] = data ;
            }
        }

        //fix coef0
        hwp_bt_rfc->TXDC_CAL_REG1  &= ~BT_RFC_TXDC_CAL_REG1_TX_DC_CAL_COEF0;
        hwp_bt_rfc->TXDC_CAL_REG1  |= coef0[i] << BT_RFC_TXDC_CAL_REG1_TX_DC_CAL_COEF0_Pos;
        dc_out_min    = 0x7fffffffffffffff;
        //coarse search for coef1
        for (j = 0; j < 128; j++)
        {
            //set coef_1;
            data =  0xfc0 + (j);
            data &= 0x3FFF;
            hwp_bt_rfc->TXDC_CAL_REG1  &= ~BT_RFC_TXDC_CAL_REG1_TX_DC_CAL_COEF1;
            hwp_bt_rfc->TXDC_CAL_REG1  |= data << BT_RFC_TXDC_CAL_REG1_TX_DC_CAL_COEF1_Pos;
            hwp_dmac2->CNDTR8 = 512;
            while (!(hwp_dmac2->ISR & DMAC_ISR_TCIF8));
            hwp_dmac2->IFCR |= DMAC_IFCR_CTCIF8;
            mixer_i_sum = 0;
            mixer_q_sum = 0;
            int phase_index = 0;
            for (k = 0; k < 512; k++)
            {
                mem_data = read_memory(BT_RFC_TXDC_DMA_ADDR + k * 4);
                //adc_data = (int)((mem_data & 0x3ff) << 22);
                //adc_data = adc_data >> 22;
                adc_data = (mem_data & 0x3ff);
                mixer_cos = cos_table[phase_index];
                mixer_sin = sin_table[phase_index];
                phase_index = phase_index + 2;
                if (phase_index == 16)
                {
                    phase_index = 0;
                }

                mixer_i = adc_data * mixer_cos; //s20.19
                mixer_q = adc_data * mixer_sin; //s20.19
                mixer_i_sum += mixer_i;
                mixer_q_sum += mixer_q;

            }
            mixer_i_sum = mixer_i_sum / 512; //s20.19
            mixer_q_sum = mixer_q_sum / 512; //s20.19
            mixer_pwr = mixer_i_sum * mixer_i_sum + mixer_q_sum * mixer_q_sum; //u32.38
            //dc_out[j]=mixer_pwr;

            if (dc_out_min > mixer_pwr)
            {
                dc_out_min = mixer_pwr;
                coef1[i] = data ;
            }
        }
        //rt_kprintf("rf_cal offset_i:0x%x, offset_q:0x%x, coef0:0x%x, coef1:0x%x\n", offset_i[i], offset_q[i], coef0[i], coef1[i]);
        hwp_bt_rfc->TXDC_CAL_REG1  &= ~BT_RFC_TXDC_CAL_REG1_TX_DC_CAL_COEF1;
        hwp_bt_rfc->TXDC_CAL_REG1  |= coef1[i] << BT_RFC_TXDC_CAL_REG1_TX_DC_CAL_COEF1_Pos;
    } //for i

    ////////////////////////////////////////////////////////////////////
    //                calibration of tmxcap_sel
    ////////////////////////////////////////////////////////////////////

    //resume to normal mode
    hwp_bt_rfc->EDR_PLL_REG2 &= ~(BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VL_SEL_LV_Msk | BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VH_SEL_LV_Msk | \
                                  BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_INCFCAL_VH_SEL_LV_Msk | BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_INCFCAL_VL_SEL_LV_Msk);
    hwp_bt_rfc->EDR_PLL_REG2 |= 0x5 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VL_SEL_LV_Pos | \
                                0x7 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_ACAL_VH_SEL_LV_Pos | \
                                0x2 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_INCFCAL_VL_SEL_LV_Pos | \
                                0x5 << BT_RFC_EDR_PLL_REG2_BRF_EDR_VCO_INCFCAL_VH_SEL_LV_Pos;

    hwp_bt_rfc->EDR_PLL_REG1 &= ~BT_RFC_EDR_PLL_REG1_BRF_EDR_VCO_LDO_VREF_LV;
    hwp_bt_rfc->EDR_PLL_REG1 |= 0xA << BT_RFC_EDR_PLL_REG1_BRF_EDR_VCO_LDO_VREF_LV_Pos;

    //level 4 : edr tx 13dBm
    //hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_TMXBUF_IBLD_LV & ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV &\
    //                            ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PACAS_BM_LV    & ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_LOBIAS_BM_LV;

    //hwp_bt_rfc->TRF_EDR_REG1 |= 0x12 << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos | 0x1 << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PACAS_BM_LV_Pos | \
    //                            0xA << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_TMXBUF_IBLD_LV_Pos | 0x2 << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_LOBIAS_BM_LV_Pos;
    hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_TMXBUF_IBLD_LV & ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV ;

    //hwp_bt_rfc->TRF_EDR_REG1 |= 0x1B << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos ;
    hwp_bt_rfc->TRF_EDR_REG1 |= 0xE << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos ;
    hwp_bt_phy->TX_DC_CAL_CFG2 = 0x30;

    hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0x77777777;

    hwp_bt_rfc->RBB_REG2 &= ~BT_RFC_RBB_REG2_BRF_RVGA_GC_LV;
//    hwp_bt_rfc->RBB_REG2 |= 0x4 << BT_RFC_RBB_REG2_BRF_RVGA_GC_LV_Pos;
    hwp_bt_rfc->TRF_EDR_REG2 &= ~BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_GC_LV;
//    hwp_bt_rfc->TRF_EDR_REG2 |= 1 << BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_GC_LV_Pos;

    //set rcos bw to 0.4M
    write_memory(0x50084180, 0x00FFF000);
    write_memory(0x50084184, 0x00FFCFFD);
    write_memory(0x50084188, 0x00FFEFFC);
    write_memory(0x5008418C, 0x00010005);
    write_memory(0x50084190, 0x0002B01D);
    write_memory(0x50084194, 0x0003C037);
    write_memory(0x50084198, 0x0002B038);
    write_memory(0x5008419C, 0x00FF8015);
    write_memory(0x500841A0, 0x00FAFFD5);
    write_memory(0x500841A4, 0x00F69F8A);
    write_memory(0x500841A8, 0x00F48F51);
    write_memory(0x500841AC, 0x00F6DF50);
    write_memory(0x500841B0, 0x00FECFA1);
    write_memory(0x500841B4, 0x000C304E);
    write_memory(0x500841B8, 0x001D5148);
    write_memory(0x500841BC, 0x002F1265);
    write_memory(0x500841C0, 0x003DB36F);
    write_memory(0x500841C4, 0x0045F42C);
    write_memory(0x500841C8, 0x00000470);

    uint32_t pwr_ref;
    uint32_t reg_addr = rslt_start_addr;
    for (i = 0; i < 3; i++)
    {


        hwp_bt_phy->TX_DC_CAL_CFG2 = 0x30;
        //set rf channel
        hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_CHANNEL;
        if (i == 0)
            hwp_bt_mac->DMRADIOCNTL1 |= 0   << BT_MAC_DMRADIOCNTL1_CHANNEL_Pos;
        else if (i == 1)
            hwp_bt_mac->DMRADIOCNTL1 |= 39   << BT_MAC_DMRADIOCNTL1_CHANNEL_Pos;
        else if (i == 2)
            hwp_bt_mac->DMRADIOCNTL1 |= 78   << BT_MAC_DMRADIOCNTL1_CHANNEL_Pos;
        //hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL;
        //_HAL_Delay_us(10);
        //hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL;
        mem_data = hwp_bt_rfc->CAL_ADDR_REG2;//addr to be determined
        mem_data >>= 16;
        mem_data += BT_RFC_MEM_BASE;
        if (i == 0) //0 channel
            mem_data = read_memory(mem_data);
        else if (i == 1) //39 channel
            mem_data = read_memory(mem_data + 39 * 4);
        else if (i == 2) //78 channel
            mem_data = read_memory(mem_data + 78 * 4);
        hwp_bt_rfc->EDR_CAL_REG1 = mem_data;
        hwp_bt_rfc->EDR_PLL_REG5 |=  BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;
        hwp_bt_rfc->EDR_PLL_REG5 &= ~BT_RFC_EDR_PLL_REG5_BRF_EDR_FBDV_RSTB_LV ;

        _HAL_Delay_us(40);//40us
        hwp_bt_rfc->TXDC_CAL_REG1  = 0x10000000;
        hwp_bt_rfc->TXDC_CAL_REG2  = 0 ;
        uint8_t tmxcap_sel_cnt[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        //go through tmxcap_sel,and store dc_out
#ifdef ENABLE_RF_ATE
        for (int l = 0; l < 1; l++)
        {
#else
        for (int l = 0; l < 30; l++)
        {
#endif
            int64_t dc_out_max = 0;
            for (j = 15; j > 2; j--)
            {
                //set coef_0;
                data =  j;

                hwp_bt_rfc->EDR_CAL_REG1  &= ~BT_RFC_EDR_CAL_REG1_BRF_TRF_EDR_TMXCAP_SEL_LV;
                hwp_bt_rfc->EDR_CAL_REG1  |= data << BT_RFC_EDR_CAL_REG1_BRF_TRF_EDR_TMXCAP_SEL_LV_Pos;

                _HAL_Delay_us(2);//40us
                //uint32_t mixer_pwr;
                hwp_dmac2->CNDTR8 = 512;
                while (!(hwp_dmac2->ISR & DMAC_ISR_TCIF8));
                hwp_dmac2->IFCR |= DMAC_IFCR_CTCIF8;
                mixer_i_sum = 0;
                mixer_q_sum = 0;
                int phase_index = 0;
                for (k = 0; k < 512; k++)
                {

                    //int phase_index = 0;
                    mem_data = read_memory(BT_RFC_TXDC_DMA_ADDR + k * 4);
                    //adc_data = (int)((mem_data&0x3ff)<<22);
                    //adc_data = adc_data >> 22;
                    adc_data = (mem_data & 0x3ff);
                    /*mixer_cos = cos_table[phase_index];
                    mixer_sin = sin_table[phase_index];
                    phase_index = phase_index + 2;
                    if (phase_index==16)
                    {
                      phase_index = 0;
                    }*/
                    //mixer_i = adc_data;//*mixer_cos;//s20.19
                    //mixer_q = adc_data;//*mixer_sin;//s20.19
                    //mixer_i_sum += mixer_i;
                    //mixer_q_sum += mixer_q;
                    mixer_i_sum += adc_data ;
                }
                //mixer_i_sum = mixer_i_sum/512; //s20.19
                //mixer_q_sum = mixer_q_sum/512; //s20.19
                //mixer_pwr_i = mixer_i_sum*mixer_i_sum;
                //mixer_pwr_q = mixer_q_sum*mixer_q_sum;
                //mixer_pwr=mixer_i_sum*mixer_i_sum + mixer_q_sum*mixer_q_sum;  //u32.38
                //mixer_pwr = mixer_pwr_i + mixer_pwr_q;
                //if(mixer_pwr > 0x7fffff00000000)
                //  printf("overflow");
                //dc_out[j]=mixer_pwr;

                //search for max dc_out and coresponding tmxcap_sel
                if (dc_out_max < mixer_i_sum)
                {
                    dc_out_max = mixer_i_sum;
                    tmxcap_sel[i * 39] = data ;
                    //RF_PRINTF("txm[%d] %d\n",i, tmxcap_sel[i]);
                }
            }
            tmxcap_sel_cnt[tmxcap_sel[i * 39]] += 1;
        }

        tmxcap_sel[i * 39] = 15;
        for (j = 15; j > 2; j--)
        {
            if (tmxcap_sel_cnt[j] > tmxcap_sel_cnt[tmxcap_sel[i * 39]])
                tmxcap_sel[i * 39] = j;
        }
    }

#ifdef ENABLE_RF_ATE
    hwp_bt_rfc->RSVD_REG1 = tmxcap_sel[78] + (tmxcap_sel[0] << 4) + (tmxcap_sel[39] << 8);
#endif

    for (i = 1; i < 39; i++)
    {
        uint32_t interp = (39 - i) * tmxcap_sel[0] + tmxcap_sel[39] * i;
        tmxcap_sel[i] = interp / 39;
        if (interp % 39 >= 20)
            tmxcap_sel[i] += 1;
    }
    for (i = 40; i < 78; i++)
    {
        uint32_t interp = (78 - i) * tmxcap_sel[39] + tmxcap_sel[78] * (i - 39);
        tmxcap_sel[i] = interp / 39;
        if (interp % 39 >= 20)
            tmxcap_sel[i] += 1;
    }
    ///////////////////////////////////////////////////
    //            dpsk gain calibration
    ///////////////////////////////////////////////////
    /*    #ifndef ENABLE_RF_ATE
        //hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0x22222222;
        //config tmxcap_sel
        hwp_bt_rfc->EDR_CAL_REG1  &= ~BT_RFC_EDR_CAL_REG1_BRF_TRF_EDR_TMXCAP_SEL_LV;
        hwp_bt_rfc->EDR_CAL_REG1  |= tmxcap_sel[i] << BT_RFC_EDR_CAL_REG1_BRF_TRF_EDR_TMXCAP_SEL_LV_Pos;
        _HAL_Delay_us(10);


                uint32_t d0,d1,d2;
                if(i==0) //2402 channel as initial
                {

                    //set dpsk gain

          hwp_bt_phy->TX_DC_CAL_CFG2 = 0x5E;//for HDK

                    //calculate adc out
          hwp_dmac2->CNDTR8 = 1024;
          while (!(hwp_dmac2->ISR & DMAC_ISR_TCIF8));
          hwp_dmac2->IFCR |= DMAC_IFCR_CTCIF8;
          mixer_i_sum = 0;
          for (k = 0; k < 1024; k++)
          {
            mem_data = read_memory(BT_RFC_TXDC_DMA_ADDR + k * 4);
            adc_data = (mem_data & 0x3ff);
            mixer_i_sum += adc_data ;
          }
                    //store reference
          pwr_ref = mixer_i_sum;
                    //update addr to next channel
                    reg_addr += 4;

                    //store dpsk gain for current channel
                  dpsk_gain[i] = 0x5E; //for HDK
                  //dpsk_gain[i] = 0x56; //for ZhouHai
              } else { //other channels
                    //search for dpsk gain whose tx power is closest to channel[0]
                    dc_out_min = 0x7fffffffffffffff;
          for (j = 0; j < 32; j++)
          {
                      //set dpsk gain

                        hwp_bt_phy->TX_DC_CAL_CFG2 = 0x40+j*2;
                      //calculate adc out
            hwp_dmac2->CNDTR8 = 1024;
            while (!(hwp_dmac2->ISR & DMAC_ISR_TCIF8));
            hwp_dmac2->IFCR |= DMAC_IFCR_CTCIF8;
            mixer_i_sum = 0;
            for (k = 0; k < 1024; k++)
            {
              mem_data = read_memory(BT_RFC_TXDC_DMA_ADDR + k * 4);
              adc_data = (mem_data & 0x3ff);
              mixer_i_sum += adc_data ;
            }

                        //compare result
                        if(mixer_i_sum > pwr_ref)
                        {
                            mixer_pwr = mixer_i_sum - pwr_ref;
                        }else{
                            mixer_pwr = pwr_ref - mixer_i_sum;
                        }
                        //dc_out[j] = mixer_pwr;
                        if(mixer_pwr < dc_out_min)
                        {
                            dc_out_min = mixer_pwr;
                            dpsk_gain[i] = 0x40+j*2;
                        }
          }
                    //average with initial gain
                    //dpsk_gain[i] = sqrt(dpsk_gain[i]*dpsk_gain[0]);
                    //update addr to next channel
          reg_addr += 4;
              }
    #endif //ndef ENABLE_RF_ATE
        */



    //store tmxcap_sel cal result and dpsk_gain cal result
    reg_addr = hwp_bt_rfc->CAL_ADDR_REG2;
    reg_addr >>= 16;
    uint32_t d0, d1, d2;
    dpsk_gain[0] = 0x5E;
    for (i = 0; i < 79; i++)
    {

        dpsk_gain[i] = (dpsk_gain[0] - (i * 14) / 78);
        //dig gain saturation
        if (dpsk_gain[i] > 0x5E) dpsk_gain[i] = 0x5E;

        data = read_memory(BT_RFC_MEM_BASE + reg_addr);
        //data &= 0xFFFFFFF;
        data &= 0x1F77FFF;
        d0 = (dpsk_gain[i] >> 1) & 0x1;
        d0 = d0 << 15;
        d1 = (dpsk_gain[i] >> 1) & 0x2;
        d1 = d1 << 18;
        d2 = (dpsk_gain[i] >> 1) & 0x1c;
        d2 = d2 << 23;
        data |= d0 | d1 | d2;
        data |= tmxcap_sel[i] << 28;
        write_memory(BT_RFC_MEM_BASE + reg_addr, data);
        reg_addr += 4;
    }

    //disable sine wave tx and dc cal module
    hwp_bt_phy->TX_DC_CAL_CFG0 &= ~BT_PHY_TX_DC_CAL_CFG0_TX_DC_CAL_EN ;



    //force to BR mode
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_NBT_BLE;
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_NBT_BLE_VAL;

    //force channel to 2440MHz
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_CHANNEL;
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_SYNCWORD;


    //release force rx on
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_RX_VAL;
    _HAL_Delay_us(20);
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_RX;
    _HAL_Delay_us(20);

    //release tx on
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_TX_VAL;
    _HAL_Delay_us(20);
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_TX ;
    _HAL_Delay_us(20);

    //set BLE/BR tx to polar modulation
    hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MOD_METHOD_BLE & ~BT_PHY_TX_CTRL_MOD_METHOD_BR;


    //restore gain setting
    hwp_bt_phy->TX_IF_MOD_CFG2 &= ~BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_BR_FRC_EN;
    hwp_bt_phy->TX_DPSK_CFG1   &= ~BT_PHY_TX_DPSK_CFG1_TX_DPSK_GAIN_FRC_EN;


    //temp add for zero crc error @13dBm test
    //hwp_bt_phy->TX_IF_MOD_CFG2 |= BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_BLE_FRC_EN|BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_BR_FRC_EN |BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_EDR_FRC_EN;
    //hwp_bt_phy->TX_DPSK_CFG1   |= BT_PHY_TX_DPSK_CFG1_TX_DPSK_GAIN_FRC_EN;
    //hwp_bt_rfc->TRF_EDR_REG1.BRF_TRF_EDR_PA_BM_LV setting which need software to take care:
    //edr  0  dBm: 0x0
    //edr  3  dBm: 0x5
    //edr  6  dBm: 0xC
    //edr  9  dBm: 0xE
    //edr  13 dBm: 0x1B
    //gfsk 13 dBm: 0x1B
    //gfsk 16 dBm: 0x1F
    //gfsk 19 dBm: 0x1F
    //edr test 13 dBm: 0x1F


    hwp_bt_phy->TX_IF_MOD_CFG4 = 0x8055555E;
    hwp_bt_phy->TX_IF_MOD_CFG6 = 0x6855555E;
    hwp_bt_phy->TX_IF_MOD_CFG7 = 0x5E5E5E5E;
    hwp_bt_phy->TX_IF_MOD_CFG8 = 0x5050505E;
    hwp_bt_phy->TX_DPSK_CFG2   = 0x5E5E5E5E;
    hwp_bt_phy->TX_DPSK_CFG3   = 0x5050505E;

    if (bt_is_in_BQB_mode())
        hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0xF8775433;
    else
        hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0xF8775433;
    //hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0x66666666;


    //restor mixer
    hwp_bt_phy->MIXER_CFG1 &= ~BT_PHY_MIXER_CFG1_RX_MIXER_PHASE_1;
    hwp_bt_phy->MIXER_CFG1 |= 0x40 << BT_PHY_MIXER_CFG1_RX_MIXER_PHASE_1_Pos;//TODO

    //restore
    hwp_bt_rfc->RBB_REG3 |= 0x2 << BT_RFC_RBB_REG3_BRF_RVGA_VCMREF_LV_Pos | 0x2 << BT_RFC_RBB_REG3_BRF_RVGA_VSTART_LV_Pos;

    //disable rx path
#if 0
    hwp_bt_rfc->TRF_EDR_REG2 &= ~BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_EN_LV ;
    //hwp_bt_rfc->TRF_EDR_REG2 &= ~BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_BM_LV & ~BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_GC_LV;
    //hwp_bt_rfc->TRF_EDR_REG2 |= 2 << BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_GC_LV_Pos;
    //hwp_bt_rfc->TRF_EDR_REG2 |= 0x1 << BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_BM_LV_Pos ;
    //hwp_bt_rfc->TRF_EDR_REG2 |= BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_OS_LV | BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PWRMTR_OS_PN_LV;

    hwp_bt_rfc->RBB_REG5 &= ~(BT_RFC_RBB_REG5_BRF_RVGA_TX_LPBK_EN_LV);
    _HAL_Delay_us(5);//5us
    hwp_bt_rfc->RBB_REG5 &= ~(BT_RFC_RBB_REG5_BRF_EN_IARRAY_LV);


    hwp_bt_rfc->RBB_REG1 &= ~BT_RFC_RBB_REG1_BRF_EN_LDO_RBB_LV;
    hwp_bt_rfc->RBB_REG2 &= ~BT_RFC_RBB_REG2_BRF_EN_RVGA_I_LV ;

    hwp_bt_rfc->ADC_REG  &= ~(BT_RFC_ADC_REG_BRF_EN_ADC_I_LV | BT_RFC_ADC_REG_BRF_EN_LDO_ADCREF_LV | BT_RFC_ADC_REG_BRF_EN_LDO_ADC_LV);
#endif
    hwp_bt_rfc->AGC_REG  &= ~BT_RFC_AGC_REG_VGA_GAIN_FRC_EN;


    hwp_bt_rfc->MISC_CTRL_REG &= ~(BT_RFC_MISC_CTRL_REG_ADC_CLK_EN_FRC_EN | BT_RFC_MISC_CTRL_REG_ADC_CLK_EN);

    hwp_bt_phy->DCCAL_MPT_CFG &= ~(BT_PHY_DCCAL_MPT_CFG_TX_DC_CAL | BT_PHY_DCCAL_MPT_CFG_DC_EST_EN);
    //hwp_bt_phy->DCCAL_MPT_CFG &= ~BT_PHY_DCCAL_MPT_CFG_DC_EST_MU;
    hwp_bt_phy->RX_CTRL1 &= ~BT_PHY_RX_CTRL1_FORCE_RX_ON;


    //store txdc cal result
    hwp_bt_rfc->CAL_ADDR_REG3 = rslt_start_addr;

    reg_addr = rslt_start_addr;


    for (i = 0; i < 9; i++)
    {
        uint8_t m;
        m = i;
        if (m > 4) m = i - 1; //use 13dBm cal result for higher power level,to be verified
#if defined(ENABLE_RF_ATE)
        m = 6;
#endif
        //if (m == 4 && bt_is_in_BQB_mode())
        //    m = 8;
        data = coef0[m] + (coef1[m] << BT_RFC_TXDC_CAL_REG1_TX_DC_CAL_COEF1_Pos);
        write_memory(BT_RFC_MEM_BASE + reg_addr, data);
        //rt_kprintf("rf_cal data0:0x%x\n", data);
        reg_addr += 4;
        data = offset_q[m] + (offset_i[m] << BT_RFC_TXDC_CAL_REG2_TX_DC_CAL_OFFSET_I_Pos);
        write_memory(BT_RFC_MEM_BASE + reg_addr, data);
        //rt_kprintf("rf_cal data1:0x%x\n", data);
        reg_addr += 4;
    }

    //replace edr cal related cmd in bt_bton_cmd with wait cmd
    reg_addr = hwp_bt_rfc->CU_ADDR_REG3;
    reg_addr &= 0xFFFF;
    for (i = 0; i < 10; i++)
    {
        data = 0x50045004;
        write_memory(BT_RFC_MEM_BASE + reg_addr + 30 * 4, data);
        reg_addr += 4;
    }

#if defined(ENABLE_RF_ATE)
    //19dBm setting for ate
    //level 7 : gfsk tx 19dBm
    hwp_bt_phy->TX_CTRL |= BT_PHY_TX_CTRL_MOD_METHOD_BLE | BT_PHY_TX_CTRL_MOD_METHOD_BR;
    hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV & ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_TMXBUF_IBLD_LV;
    hwp_bt_rfc->TRF_EDR_REG1 |= 0x1F << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PA_BM_LV_Pos;
    hwp_bt_phy->TX_DC_CAL_CFG2 = 0x80;
    hwp_bt_phy->EDR_TMXBUF_GC_CFG2 &= ~BT_PHY_EDR_TMXBUF_GC_CFG2_EDR_TMXBUF_GC_0 ;
    hwp_bt_phy->EDR_TMXBUF_GC_CFG2 = 0xFFFFFFFF;
#endif
    HAL_RCC_LCPU_SetDiv(div, -1, -1);
    return reg_addr;
}




static void bt_ful_cal(uint32_t addr)
{
    addr  = bt_rfc_lo_cal(addr);
#if defined(ENABLE_EDR_5G)
    addr = bt_rfc_edrlo_5g_cal(addr);
#elif defined(ENABLE_EDR_3G)
#if !defined(CFG_FACTORY_DEBUG)
    addr = bt_rfc_edrlo_3g_cal(addr);
#endif
#elif defined(ENABLE_EDR_2G)
    addr = bt_rfc_edrlo_2g_cal(addr);
#endif

    hwp_lpsys_rcc->CSR &= ~LPSYS_RCC_CSR_SEL_SYS;
    hwp_lpsys_rcc->CSR |= 1 << LPSYS_RCC_CSR_SEL_SYS_Pos;
#if !defined(CFG_FACTORY_DEBUG)
    addr = bt_rfc_txdc_cal(addr);
#endif
#ifdef ENABLE_IQ_MODULE
    //hwp_bt_phy->TX_CTRL |= BT_PHY_TX_CTRL_MOD_METHOD_BLE | BT_PHY_TX_CTRL_MOD_METHOD_BR;
#else
    //hwp_bt_phy->TX_CTRL &= ~(BT_PHY_TX_CTRL_MOD_METHOD_BLE | BT_PHY_TX_CTRL_MOD_METHOD_BR);
#endif

    //hwp_bt_rfc->TXDC_CAL_REG1 = 0x10003a80;
    //hwp_bt_rfc->TXDC_CAL_REG2 = 0x000f0000;

}


__WEAK uint8_t bt_is_in_BQB_mode(void)
{
    return 0;
}

__WEAK int8_t bt_rf_get_max_tx_pwr(void)
{
    return 10;
}

__WEAK int8_t bt_rf_get_init_tx_pwr(void)
{
    return 0;
}

__WEAK int8_t bt_rf_get_min_tx_pwr(void)
{
    return 0;
}



#if 1
void bt_rf_opt_cal(void)
{
    // PA config
    hwp_bt_rfc->TRF_REG1 &= ~BT_RFC_TRF_REG1_BRF_PA_PM_LV_Msk;
    hwp_bt_rfc->TRF_REG1 &= ~BT_RFC_TRF_REG1_BRF_PA_CAS_BP_LV_Msk;

    hwp_bt_rfc->TRF_REG2 &= ~BT_RFC_TRF_REG2_BRF_PA_UNIT_SEL_LV_Msk;
    hwp_bt_rfc->TRF_REG2 &= ~BT_RFC_TRF_REG2_BRF_PA_MCAP_LV_Msk;

#if 1
    hwp_bt_rfc->TRF_REG1 |= 0x01 << BT_RFC_TRF_REG1_BRF_PA_PM_LV_Pos;
    hwp_bt_rfc->TRF_REG1 |= 0x01 << BT_RFC_TRF_REG1_BRF_PA_CAS_BP_LV_Pos;

    hwp_bt_rfc->TRF_REG2 |= 0x01 << BT_RFC_TRF_REG2_BRF_PA_UNIT_SEL_LV_Pos;
    hwp_bt_rfc->TRF_REG2 |= 0x0 << BT_RFC_TRF_REG2_BRF_PA_MCAP_LV_Pos;
#else

    hwp_bt_rfc->TRF_REG1 |= 0x00 << BT_RFC_TRF_REG1_BRF_PA_PM_LV_Pos;
    hwp_bt_rfc->TRF_REG1 |= 0x01 << BT_RFC_TRF_REG1_BRF_PA_CAS_BP_LV_Pos;

    hwp_bt_rfc->TRF_REG2 |= 0x1F << BT_RFC_TRF_REG2_BRF_PA_UNIT_SEL_LV_Pos;
    hwp_bt_rfc->TRF_REG2 |= 0x01 << BT_RFC_TRF_REG2_BRF_PA_MCAP_LV_Pos;
#endif
    hwp_bt_phy->TX_PA_CFG &= ~BT_PHY_TX_PA_CFG_PA_CTRL_TARGET_Msk;
    hwp_bt_phy->TX_PA_CFG |= 0x3D << BT_PHY_TX_PA_CFG_PA_CTRL_TARGET_Pos;


    // For debug, MAC do not control power
    //hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MAC_PWR_CTRL_EN_Msk;

    // RF CBPF
    hwp_bt_rfc->RBB_REG1 &= ~(BT_RFC_RBB_REG1_BRF_PKDET_VTH1I_BT_Msk | BT_RFC_RBB_REG1_BRF_PKDET_VTH1Q_BT_Msk
                              | BT_RFC_RBB_REG1_BRF_PKDET_VTH2I_BT_Msk | BT_RFC_RBB_REG1_BRF_PKDET_VTH2Q_BT_Msk);
    hwp_bt_rfc->RBB_REG1 |= (0x03 << BT_RFC_RBB_REG1_BRF_PKDET_VTH1I_BT_Pos | 0x03 << BT_RFC_RBB_REG1_BRF_PKDET_VTH1Q_BT_Pos
                             | 0x03 << BT_RFC_RBB_REG1_BRF_PKDET_VTH2I_BT_Pos | 0x03 << BT_RFC_RBB_REG1_BRF_PKDET_VTH2Q_BT_Pos);

    hwp_bt_rfc->RBB_REG2 &= ~BT_RFC_RBB_REG2_BRF_CBPF_FC_LV_Msk;
    hwp_bt_rfc->RBB_REG2 |= 0x3 << BT_RFC_RBB_REG2_BRF_CBPF_FC_LV_Pos;
    hwp_bt_rfc->RBB_REG4 &= ~(BT_RFC_RBB_REG4_BRF_PKDET_VTH1I_LV_Msk | BT_RFC_RBB_REG4_BRF_PKDET_VTH1Q_LV_Msk
                              | BT_RFC_RBB_REG4_BRF_PKDET_VTH2I_LV_Msk | BT_RFC_RBB_REG4_BRF_PKDET_VTH2Q_LV_Msk);
    hwp_bt_rfc->RBB_REG4 |= (0x0A << BT_RFC_RBB_REG4_BRF_PKDET_VTH1I_LV_Pos) | (0x0A << BT_RFC_RBB_REG4_BRF_PKDET_VTH1Q_LV_Pos)
                            | (0x0A << BT_RFC_RBB_REG4_BRF_PKDET_VTH2I_LV_Pos) | (0x0A << BT_RFC_RBB_REG4_BRF_PKDET_VTH2Q_LV_Pos);



    // Mixer
    hwp_bt_phy->MIXER_CFG1 &= ~BT_PHY_MIXER_CFG1_RX_MIXER_PHASE_1_Msk;
    hwp_bt_phy->MIXER_CFG1 &= ~BT_PHY_MIXER_CFG1_RX_MIXER_PHASE_2_Msk;
    hwp_bt_phy->MIXER_CFG1 |= (0xA6 << BT_PHY_MIXER_CFG1_RX_MIXER_PHASE_1_Pos) | (0x80 << BT_PHY_MIXER_CFG1_RX_MIXER_PHASE_2_Pos);

    // MMDIV_OFFSET
    hwp_bt_phy->LFP_MMDIV_CFG0 &= ~BT_PHY_LFP_MMDIV_CFG0_RX_MMDIV_OFFSET_1M_Msk;
    hwp_bt_phy->LFP_MMDIV_CFG1 &= ~BT_PHY_LFP_MMDIV_CFG1_RX_MMDIV_OFFSET_2M_Msk;
    hwp_bt_phy->LFP_MMDIV_CFG0 |= 0x1AAE1 << BT_PHY_LFP_MMDIV_CFG0_RX_MMDIV_OFFSET_1M_Pos;
    hwp_bt_phy->LFP_MMDIV_CFG1 |= 0x18000 << BT_PHY_LFP_MMDIV_CFG1_RX_MMDIV_OFFSET_2M_Pos;

    // BLE DEMOD
    hwp_bt_phy->DEMOD_CFG1 &= ~(BT_PHY_DEMOD_CFG1_BLE_DEMOD_G_Msk | BT_PHY_DEMOD_CFG1_BLE_MU_DC_Msk | BT_PHY_DEMOD_CFG1_BLE_MU_ERR_Msk);
    hwp_bt_phy->DEMOD_CFG1 |= (0xB0 << BT_PHY_DEMOD_CFG1_BLE_DEMOD_G_Pos) | (0x22 << BT_PHY_DEMOD_CFG1_BLE_MU_DC_Pos) | (0x168 << BT_PHY_DEMOD_CFG1_BLE_MU_ERR_Pos);
    hwp_bt_phy->DEMOD_CFG8 &= ~(BT_PHY_DEMOD_CFG8_BR_DEMOD_G_Msk | BT_PHY_DEMOD_CFG8_BR_MU_DC_Msk | BT_PHY_DEMOD_CFG8_BR_MU_ERR_Msk);
    hwp_bt_phy->DEMOD_CFG8 |= (0x50 << BT_PHY_DEMOD_CFG8_BR_DEMOD_G_Pos) | (0x10 << BT_PHY_DEMOD_CFG8_BR_MU_DC_Pos) | (0x120 << BT_PHY_DEMOD_CFG8_BR_MU_ERR_Pos);

    // INTERPROLATOR
    //hwp_bt_phy->TED_CFG1 &= ~(BT_PHY_TED_CFG1_TED_MU_F_C_Msk | BT_PHY_TED_CFG1_TED_MU_P_C_Msk | BT_PHY_TED_CFG1_TED_MU_F_U_Msk | BT_PHY_TED_CFG1_TED_MU_P_U_Msk);
    hwp_bt_phy->TED_CFG1 = (0x02 << BT_PHY_TED_CFG1_TED_MU_F_U_Pos) | (0x04 << BT_PHY_TED_CFG1_TED_MU_P_U_Pos) |
                           (0x03 << BT_PHY_TED_CFG1_TED_MU_F_BR_Pos) | (0x05 << BT_PHY_TED_CFG1_TED_MU_P_BR_Pos);



    // EDR PA
#if 0
    hwp_bt_rfc->TRF_EDR_REG1 &= ~(BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_TMXBUF_IBLD_LV | BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_LOBIAS_BM_LV
                                  | BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PACAS_BM_LV);

    hwp_bt_rfc->TRF_EDR_REG2 &= ~BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PAPMOS_BM_LV;

    hwp_bt_rfc->TRF_EDR_REG1 |= (0x0 << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_TMXBUF_IBLD_LV_Pos) | (0x0 << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_LOBIAS_BM_LV_Pos)
                                | (0x02 << BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_PACAS_BM_LV_Pos);

    hwp_bt_rfc->TRF_EDR_REG2 |= 0x07 << BT_RFC_TRF_EDR_REG2_BRF_TRF_EDR_PAPMOS_BM_LV_Pos;
#endif
    // PKT detect threadhold
    hwp_bt_phy->PKTDET_CFG2 &= ~BT_PHY_PKTDET_CFG2_BR_PKTDET_THD_Msk;
    hwp_bt_phy->PKTDET_CFG2 |= 0x500 << BT_PHY_PKTDET_CFG2_BR_PKTDET_THD_Pos;

#if 0
    // TX RCOS
    hwp_bt_phy->RCOS_CFG0 = (0x000 << BT_PHY_RCOS_CFG0_RCOS_COEF_0_Pos) | (0xFFF << BT_PHY_RCOS_CFG0_RCOS_COEF_1_Pos);
    hwp_bt_phy->RCOS_CFG1 = (0xFFD << BT_PHY_RCOS_CFG1_RCOS_COEF_2_Pos) | (0xFFC << BT_PHY_RCOS_CFG1_RCOS_COEF_3_Pos);
    hwp_bt_phy->RCOS_CFG2 = (0xFFC << BT_PHY_RCOS_CFG2_RCOS_COEF_4_Pos) | (0xFFE << BT_PHY_RCOS_CFG2_RCOS_COEF_5_Pos);
    hwp_bt_phy->RCOS_CFG3 = (0x005 << BT_PHY_RCOS_CFG3_RCOS_COEF_6_Pos) | (0x010 << BT_PHY_RCOS_CFG3_RCOS_COEF_7_Pos);
    hwp_bt_phy->RCOS_CFG4 = (0x01D << BT_PHY_RCOS_CFG4_RCOS_COEF_8_Pos) | (0x02B << BT_PHY_RCOS_CFG4_RCOS_COEF_9_Pos);
    hwp_bt_phy->RCOS_CFG5 = (0x037 << BT_PHY_RCOS_CFG5_RCOS_COEF_10_Pos) | (0x03C << BT_PHY_RCOS_CFG5_RCOS_COEF_11_Pos);
#endif
    //Tx GFSK modulation index
    hwp_bt_phy->TX_GAUSSFLT_CFG &= ~(BT_PHY_TX_GAUSSFLT_CFG_GAUSS_GAIN_2_Msk | BT_PHY_TX_GAUSSFLT_CFG_GAUSS_GAIN_1_Msk);
    hwp_bt_phy->TX_GAUSSFLT_CFG |= 0xF7 << BT_PHY_TX_GAUSSFLT_CFG_GAUSS_GAIN_2_Pos;
    hwp_bt_phy->TX_GAUSSFLT_CFG |= 0xFD << BT_PHY_TX_GAUSSFLT_CFG_GAUSS_GAIN_1_Pos;

    //NOTCH
    hwp_bt_phy->NOTCH_CFG1 &= ~BT_PHY_NOTCH_CFG1_NOTCH_B1_1_Msk;
    hwp_bt_phy->NOTCH_CFG1 |= 0x3000 << BT_PHY_NOTCH_CFG1_NOTCH_B1_1_Pos;

    hwp_bt_phy->NOTCH_CFG7 &= ~BT_PHY_NOTCH_CFG7_CHNL_NOTCH_EN1_1_Msk;
    hwp_bt_phy->NOTCH_CFG7 |= 0x00004000 << BT_PHY_NOTCH_CFG7_CHNL_NOTCH_EN1_1_Pos;

    hwp_bt_phy->NOTCH_CFG10 &= ~BT_PHY_NOTCH_CFG10_CHNL_NOTCH_EN1_2_Msk;
    hwp_bt_phy->NOTCH_CFG10 |= 0x00004000 << BT_PHY_NOTCH_CFG10_CHNL_NOTCH_EN1_2_Pos;


    /* below config come from rom patch  */
    hwp_bt_phy->RX_CTRL1 |= BT_PHY_RX_CTRL1_ADC_Q_EN_1;
    hwp_bt_phy->RX_CTRL1 &= ~BT_PHY_RX_CTRL1_FRC_ADC_24M_Msk;
    hwp_bt_phy->RX_CTRL1 &= ~BT_PHY_RX_CTRL1_PHY_RX_DUMP_EN_Msk;


    hwp_bt_phy->EDRSYNC_CFG1   |= BT_PHY_EDRSYNC_CFG1_EDRSYNC_METHOD;
    hwp_bt_phy->EDRDEMOD_CFG1  &= (~BT_PHY_EDRDEMOD_CFG1_EDR2_MU_DC);
    hwp_bt_phy->EDRDEMOD_CFG1  |= (0x40UL << BT_PHY_EDRDEMOD_CFG1_EDR2_MU_DC_Pos);
    hwp_bt_phy->EDRDEMOD_CFG1  &= (~BT_PHY_EDRDEMOD_CFG1_EDR2_MU_ERR);
    hwp_bt_phy->EDRDEMOD_CFG1  |= (0x100UL << BT_PHY_EDRDEMOD_CFG1_EDR2_MU_ERR_Pos);
    hwp_bt_phy->EDRDEMOD_CFG2  &= (~BT_PHY_EDRDEMOD_CFG2_EDR3_MU_DC);
    hwp_bt_phy->EDRDEMOD_CFG2  |= (0x40UL << BT_PHY_EDRDEMOD_CFG2_EDR3_MU_DC_Pos);
    hwp_bt_phy->EDRDEMOD_CFG2  &= (~BT_PHY_EDRDEMOD_CFG2_EDR3_MU_ERR);
    hwp_bt_phy->EDRDEMOD_CFG2  |= (0x140UL << BT_PHY_EDRDEMOD_CFG2_EDR3_MU_ERR_Pos);

    hwp_bt_phy->EDRTED_CFG1 = 0xA6;
    hwp_bt_phy->EDRTED_CFG1  &= (~BT_PHY_EDRTED_CFG1_TED_EDR3_MU_P);
    hwp_bt_phy->EDRTED_CFG1  |= (0x6UL << BT_PHY_EDRTED_CFG1_TED_EDR3_MU_P_Pos);
    hwp_bt_phy->EDRTED_CFG1  &= (~BT_PHY_EDRTED_CFG1_TED_EDR3_MU_F);
    hwp_bt_phy->EDRTED_CFG1  |= (0xAUL << BT_PHY_EDRTED_CFG1_TED_EDR3_MU_F_Pos);

    hwp_bt_phy->INTERP_CFG1 |= BT_PHY_INTERP_CFG1_INTERP_METHOD_U;
    // Force Polar
    hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MAC_PWR_CTRL_EN_Msk;

    // Fixed abnormal signal in spectrum for BLE
    hwp_bt_phy->TX_PA_CFG &= ~BT_PHY_TX_PA_CFG_PA_RAMP_FACTOR_IDX_Msk;
    hwp_bt_phy->TX_PA_CFG |= 0x06 << BT_PHY_TX_PA_CFG_PA_RAMP_FACTOR_IDX_Pos;
}
#endif

#define BR_BQB_COCHANNEL_CASE  0
//#define VDDIOB_V33

void bt_rf_cal(void)
{
    HAL_RCC_ResetBluetoothRF();
#if 1
    // QFN
#if 1 //def BPS_V33_HAL
#ifdef ENABLE_RF_ATE
    hwp_bt_rfc->TRF_EDR_REG1 |= BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_TMXCAS_SEL_LV;
#else
#ifndef VDDIOB_V33
    hwp_bt_rfc->TRF_EDR_REG1 |= BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_TMXCAS_SEL_LV;
#else
    hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_TMXCAS_SEL_LV;
#endif
#endif
#else //BPS_V33_HAL
    if (__HAL_SYSCFG_GET_PID() == 0)
        hwp_bt_rfc->TRF_EDR_REG1 &= ~BT_RFC_TRF_EDR_REG1_BRF_TRF_EDR_TMXCAS_SEL_LV;
#endif

    uint32_t addr = bt_rfc_init();
    bt_ful_cal(addr);

    bt_rf_opt_cal();

#if BR_BQB_COCHANNEL_CASE
    hwp_bt_phy->DEMOD_CFG8 &= ~(BT_PHY_DEMOD_CFG8_BR_DEMOD_G_Msk | BT_PHY_DEMOD_CFG8_BR_MU_DC_Msk | BT_PHY_DEMOD_CFG8_BR_MU_ERR_Msk);
    hwp_bt_phy->DEMOD_CFG8 |= (0x10 << BT_PHY_DEMOD_CFG8_BR_DEMOD_G_Pos) | (0x02 << BT_PHY_DEMOD_CFG8_BR_MU_DC_Pos) | (0x60 << BT_PHY_DEMOD_CFG8_BR_MU_ERR_Pos);
    hwp_bt_phy->DEMOD_CFG16 |= BT_PHY_DEMOD_CFG16_BR_HADAPT_EN;
#endif
    HAL_Set_backup(RTC_BACKUP_BT_TXPWR, RF_PWR_PARA(bt_rf_get_max_tx_pwr(), bt_rf_get_min_tx_pwr(), bt_rf_get_init_tx_pwr(), (0x80 | bt_is_in_BQB_mode())));
#endif
}
char *g_rf_ful_ver = "1.1.9_2841";
char *rf_ful_ver(uint8_t *cal_en)
{
    *cal_en = 0xFF;
    return g_rf_ful_ver;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

