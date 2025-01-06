/**
  ******************************************************************************
  * @file   drv_audcodec.c
  * @author Sifli software development team
  * @brief   Audio Process driver adaption layer
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "board.h"
#include "drv_config.h"

#if defined (SYS_HEAP_IN_PSRAM)
    #undef calloc
    #undef free
    #undef malloc
    extern void *app_sram_alloc(rt_size_t size);
    extern void *app_sram_calloc(rt_size_t count, rt_size_t size);
    extern void *app_sram_free(void *ptr);
    #define  malloc(s)      app_sram_alloc(s)
    #define  calloc(c, s)   app_sram_calloc(c, s)
    #define  free(p)        app_sram_free(p)
#endif

enum PLL_SET_GRADE
{
    PLL_ADD_ONE_HUND_PPM,
    PLL_ADD_TWO_HUND_PPM,
    PLL_SUB_ONE_HUND_PPM,
    PLL_SUB_TWO_HUND_PPM,
};

void pll_freq_fine_tuning(int delta)
{
    uint32_t  pll_value = 0;
    int sdin = 0, cfw = 0;

    RT_ASSERT((delta >= -0xFFFFF) && (delta <= 0xFFFFF));
    pll_value = READ_REG(hwp_audcodec_lp->PLL_CFG3);
    sdin = (int32_t)(pll_value & AUDCODEC_LP_PLL_CFG3_SDIN_Msk);
    cfw = (int32_t)((pll_value & AUDCODEC_LP_PLL_CFG3_FCW_Msk) >> AUDCODEC_LP_PLL_CFG3_FCW_Pos);
    //rt_kprintf("pll original sdin:%d, cfw:%d\n", sdin, cfw);
    if ((sdin + delta) < 0)
    {
        cfw -= 1;
        sdin = 0xFFFFF + sdin + delta;
    }
    else if ((sdin + delta) >= 0xFFFFF)
    {
        cfw += 1;
        sdin = sdin + delta - 0xFFFFF;
    }
    else // 0 <=  (sdin + delta) < 0xFFFFF
    {
        sdin = sdin + delta;
    }
    //rt_kprintf("pll after sdin:%d, cfw:%d\n", sdin, cfw);
    MODIFY_REG(hwp_audcodec_lp->PLL_CFG3, AUDCODEC_LP_PLL_CFG3_SDIN_Msk, \
               MAKE_REG_VAL(sdin, AUDCODEC_LP_PLL_CFG3_SDIN_Msk, AUDCODEC_LP_PLL_CFG3_SDIN_Pos));
    MODIFY_REG(hwp_audcodec_lp->PLL_CFG3, AUDCODEC_LP_PLL_CFG3_FCW_Msk, \
               MAKE_REG_VAL(cfw, AUDCODEC_LP_PLL_CFG3_FCW_Msk, AUDCODEC_LP_PLL_CFG3_FCW_Pos));
    MODIFY_REG(hwp_audcodec_lp->PLL_CFG3, AUDCODEC_LP_PLL_CFG3_SDM_UPDATE_Msk, \
               MAKE_REG_VAL(1, AUDCODEC_LP_PLL_CFG3_SDM_UPDATE_Msk, AUDCODEC_LP_PLL_CFG3_SDM_UPDATE_Pos));
    /*
    pll_value = READ_REG(hwp_audcodec_lp->PLL_CFG3);
    sdin = (int32_t)(pll_value & AUDCODEC_LP_PLL_CFG3_SDIN_Msk);
    cfw = (int32_t)((pll_value & AUDCODEC_LP_PLL_CFG3_FCW_Msk) >> AUDCODEC_LP_PLL_CFG3_FCW_Pos);
    rt_kprintf("pll check sdin:%d, cfw:%d\n", sdin, cfw);
    */
}

void pll_freq_grade_set(uint8_t gr)
{
    uint32_t  pll_value = 0;
    int delta = 0;
    double grade = 0.0, cfw = 0.0, sdin = 0.0;

    pll_value = READ_REG(hwp_audcodec_lp->PLL_CFG3);
    sdin = (double)(pll_value & AUDCODEC_LP_PLL_CFG3_SDIN_Msk);
    cfw = (double)((pll_value & AUDCODEC_LP_PLL_CFG3_FCW_Msk) >> AUDCODEC_LP_PLL_CFG3_FCW_Pos);

    grade = (cfw + 3.0 + sdin / 0xFFFFF);
    switch (gr)
    {
    case PLL_ADD_ONE_HUND_PPM:
        grade = grade * 0xFFFFF / 10000; //100PPM
        break;
    case PLL_ADD_TWO_HUND_PPM:
        grade = grade * 0xFFFFF / 5000; //200PPM
        break;
    case PLL_SUB_ONE_HUND_PPM:
        grade = -grade * 0xFFFFF / 10000;
        break;
    case PLL_SUB_TWO_HUND_PPM:
        grade = -grade * 0xFFFFF / 5000;
        break;
    default:
        RT_ASSERT(0);
    }

    delta = round(grade);
    pll_freq_fine_tuning(delta);
}


//just add rt_kprintf
void update_pll_after_enable(int16_t delta)
{
    uint32_t value = READ_REG(hwp_audcodec_lp->PLL_CFG3);
    int32_t sdin;


    sdin = (int32_t)(value & AUDCODEC_LP_PLL_CFG3_SDIN_Msk) + (int32_t)delta;

    RT_ASSERT((sdin >= 0) && (sdin <= 0xFFFFF));

    MODIFY_REG(hwp_audcodec_lp->PLL_CFG3, AUDCODEC_LP_PLL_CFG3_SDIN_Msk, \
               MAKE_REG_VAL(sdin, AUDCODEC_LP_PLL_CFG3_SDIN_Msk, AUDCODEC_LP_PLL_CFG3_SDIN_Pos));
    MODIFY_REG(hwp_audcodec_lp->PLL_CFG3, AUDCODEC_LP_PLL_CFG3_SDM_UPDATE_Msk, \
               MAKE_REG_VAL(1, AUDCODEC_LP_PLL_CFG3_SDM_UPDATE_Msk, AUDCODEC_LP_PLL_CFG3_SDM_UPDATE_Pos));
}
int updata_pll_freq(uint8_t type) //type 0: 16k 1024 series  1:44.1k 1024 series 2:16k 1000 series 3: 44.1k 1000 series
{
    hwp_audcodec_lp->PLL_CFG2 |= AUDCODEC_LP_PLL_CFG2_RSTB;
    // wait 50us
    HAL_Delay_us(50);
    if (0 == type)// set pll to 49.152M   [(fcw+3)+sdin/2^20]*6M
    {
        hwp_audcodec_lp->PLL_CFG3 = (201327 << AUDCODEC_LP_PLL_CFG3_SDIN_Pos) |
                                    (5 << AUDCODEC_LP_PLL_CFG3_FCW_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDM_UPDATE_Pos) |
                                    (1 << AUDCODEC_LP_PLL_CFG3_SDMIN_BYPASS_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDM_MODE_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_EN_SDM_DITHER_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDM_DITHER_Pos) |
                                    (1 << AUDCODEC_LP_PLL_CFG3_EN_SDM_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDMCLK_POL_Pos);
    }
    else if (1 == type)// set pll to 45.1584M
    {

        hwp_audcodec_lp->PLL_CFG3 = (551970 << AUDCODEC_LP_PLL_CFG3_SDIN_Pos) |
                                    (4 << AUDCODEC_LP_PLL_CFG3_FCW_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDM_UPDATE_Pos) |
                                    (1 << AUDCODEC_LP_PLL_CFG3_SDMIN_BYPASS_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDM_MODE_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_EN_SDM_DITHER_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDM_DITHER_Pos) |
                                    (1 << AUDCODEC_LP_PLL_CFG3_EN_SDM_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDMCLK_POL_Pos);
    }
    else if (2 == type)// set pll to 48M
    {

        hwp_audcodec_lp->PLL_CFG3 = (0 << AUDCODEC_LP_PLL_CFG3_SDIN_Pos) |
                                    (5 << AUDCODEC_LP_PLL_CFG3_FCW_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDM_UPDATE_Pos) |
                                    (1 << AUDCODEC_LP_PLL_CFG3_SDMIN_BYPASS_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDM_MODE_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_EN_SDM_DITHER_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDM_DITHER_Pos) |
                                    (1 << AUDCODEC_LP_PLL_CFG3_EN_SDM_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDMCLK_POL_Pos);
    }
    else if (3 == type)// set pll to 44.1M
    {

        hwp_audcodec_lp->PLL_CFG3 = (0x5999A << AUDCODEC_LP_PLL_CFG3_SDIN_Pos) |
                                    (4 << AUDCODEC_LP_PLL_CFG3_FCW_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDM_UPDATE_Pos) |
                                    (1 << AUDCODEC_LP_PLL_CFG3_SDMIN_BYPASS_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDM_MODE_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_EN_SDM_DITHER_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDM_DITHER_Pos) |
                                    (1 << AUDCODEC_LP_PLL_CFG3_EN_SDM_Pos) |
                                    (0 << AUDCODEC_LP_PLL_CFG3_SDMCLK_POL_Pos);
    }
    else
    {
        RT_ASSERT(0);
    }
    hwp_audcodec_lp->PLL_CFG3 |= AUDCODEC_LP_PLL_CFG3_SDM_UPDATE;
    hwp_audcodec_lp->PLL_CFG3 &= ~AUDCODEC_LP_PLL_CFG3_SDMIN_BYPASS;
    hwp_audcodec_lp->PLL_CFG2 &= ~AUDCODEC_LP_PLL_CFG2_RSTB;
    // wait 50us
    HAL_Delay_us(50);
    hwp_audcodec_lp->PLL_CFG2 |= AUDCODEC_LP_PLL_CFG2_RSTB;
    // check pll lock
    //wait(2500);
    HAL_Delay_us(50);

    hwp_audcodec_lp->PLL_CFG1 |= AUDCODEC_LP_PLL_CFG1_CSD_EN | AUDCODEC_LP_PLL_CFG1_CSD_RST;
    HAL_Delay_us(50);
    hwp_audcodec_lp->PLL_CFG1 &= ~AUDCODEC_LP_PLL_CFG1_CSD_RST;
    if (hwp_audcodec_lp->PLL_STAT & AUDCODEC_LP_PLL_STAT_UNLOCK_Msk)
    {
        rt_kprintf("pll lock fail! freq_type:%d\n", type);
        return -1;
    }
    else
    {
        rt_kprintf("pll lock! freq_type:%d\n", type);
        hwp_audcodec_lp->PLL_CFG1 &= ~AUDCODEC_LP_PLL_CFG1_CSD_EN;
        set_pll_freq_type(type);
    }
    return 0;
}
#if 0
/**
 * @brief  enable PLL function
 * @param freq - frequency
 * @param type - 0:1024 series, 1:1000 series
 * @return
 */
int bf0_enable_pll(uint32_t freq, uint8_t type)//just add rt_kprintf
{
    uint8_t freq_type;
    uint8_t test_result = -1;
    uint32_t rdata;
    uint32_t pll_cnt;
    uint32_t xtal_cnt;
    uint32_t fc_vco;
    uint32_t fc_vco_min;
    uint32_t fc_vco_max;
    uint32_t delta_cnt;
    uint32_t delta_cnt_min;
    uint32_t delta_cnt_max;
    uint32_t delta_fc_vco;
    uint32_t target_cnt = 1838;

    fc_vco = 16;
    delta_fc_vco = 8;
    test_result = 0;

    freq_type = type << 1;
    if ((freq == 44100) || (freq == 22050) || (freq == 11025))
    {
        freq_type += 1;
    }

    rt_kprintf("PLL_ENABLE pll_state:%d, freq_type:%d\n", get_pll_state(), freq_type);

    if (0 == get_pll_state())
    {
        hwp_pmuc->HXT_CR1 |= PMUC_HXT_CR1_BUF_AUD_EN;
        hwp_hpsys_rcc->ENR2 |= HPSYS_RCC_ENR2_AUDCODEC;
        hwp_lpsys_rcc->ENR1 |= LPSYS_RCC_ENR1_AUDCODEC;

        //-----------------step 2------------------//
// turn on bandgap
// turn on BG sample clock
        hwp_audcodec_lp->BG_CFG1 = 48000;
        hwp_audcodec_lp->BG_CFG2 = 48000000;
// turn on bandgap
        hwp_audcodec_lp->BG_CFG0 &= ~AUDCODEC_LP_BG_CFG0_EN_RCFLT;
        hwp_audcodec_lp->BG_CFG0 |= AUDCODEC_LP_BG_CFG0_EN;
        HAL_Delay_us(100);
        //hwp_audcodec_lp->BG_CFG0 |= AUDCODEC_LP_BG_CFG0_EN_SMPL;

//-----------------step 3------------------//
// pll calibration
// wait 100us
        //wait(5000);
        HAL_Delay_us(100);

        hwp_audcodec_lp->PLL_CFG0 |= AUDCODEC_LP_PLL_CFG0_EN_IARY;
        hwp_audcodec_lp->PLL_CFG0 |= AUDCODEC_LP_PLL_CFG0_EN_VCO;
        hwp_audcodec_lp->PLL_CFG0 |= AUDCODEC_LP_PLL_CFG0_EN_ANA;
        hwp_audcodec_lp->PLL_CFG2 |= AUDCODEC_LP_PLL_CFG2_EN_DIG;
        hwp_audcodec_lp->PLL_CFG3 |= AUDCODEC_LP_PLL_CFG3_EN_SDM;
        hwp_audcodec_lp->PLL_CFG4 |= AUDCODEC_LP_PLL_CFG4_EN_CLK_DIG;
// wait 50us
        //wait(2500);
        HAL_Delay_us(50);
// VCO freq calibration
        hwp_audcodec_lp->PLL_CFG0 |= AUDCODEC_LP_PLL_CFG0_OPEN;
        hwp_audcodec_lp->PLL_CFG2 |= AUDCODEC_LP_PLL_CFG2_EN_LF_VCIN;
        hwp_audcodec_lp->PLL_CAL_CFG = (0    << AUDCODEC_LP_PLL_CAL_CFG_EN_Pos) |
                                       (2000 << AUDCODEC_LP_PLL_CAL_CFG_LEN_Pos);
// setup calibration and run
// target pll_cnt = ceil(46MHz/48MHz*2000)+1 = 1918
// target difference between pll_cnt and xtal_cnt should be less than 1
        while (delta_fc_vco != 0)
        {
            hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_FC_VCO;
            hwp_audcodec_lp->PLL_CFG0 |= (fc_vco << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
            hwp_audcodec_lp->PLL_CAL_CFG |= AUDCODEC_LP_PLL_CAL_CFG_EN;
            while (!(hwp_audcodec_lp->PLL_CAL_CFG & AUDCODEC_LP_PLL_CAL_CFG_DONE_Msk));
            pll_cnt = (hwp_audcodec_lp->PLL_CAL_RESULT >> AUDCODEC_LP_PLL_CAL_RESULT_PLL_CNT_Pos);
            xtal_cnt = (hwp_audcodec_lp->PLL_CAL_RESULT & AUDCODEC_LP_PLL_CAL_RESULT_XTAL_CNT_Msk);
            hwp_audcodec_lp->PLL_CAL_CFG &= ~AUDCODEC_LP_PLL_CAL_CFG_EN;
            if (pll_cnt < target_cnt)
            {
                fc_vco = fc_vco + delta_fc_vco;
                delta_cnt = target_cnt - pll_cnt;
            }
            else if (pll_cnt > target_cnt)
            {
                fc_vco = fc_vco - delta_fc_vco;
                delta_cnt = pll_cnt - target_cnt;
            }
            delta_fc_vco = delta_fc_vco >> 1;
        }
        rt_kprintf("call par CFG1(%x)\r\n", hwp_audcodec_lp->PLL_CFG1);
        fc_vco_min = fc_vco - 1;
        if (fc_vco == 31)
        {
            fc_vco_max = fc_vco;
        }
        else
        {
            fc_vco_max = fc_vco + 1;
        }

        hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_FC_VCO;
        hwp_audcodec_lp->PLL_CFG0 |= (fc_vco_min << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
        hwp_audcodec_lp->PLL_CAL_CFG |= AUDCODEC_LP_PLL_CAL_CFG_EN;
        rt_kprintf("fc %d, xtal %d, pll %d\r\n", fc_vco, xtal_cnt, pll_cnt);
        while (!(hwp_audcodec_lp->PLL_CAL_CFG & AUDCODEC_LP_PLL_CAL_CFG_DONE_Msk));
        pll_cnt = (hwp_audcodec_lp->PLL_CAL_RESULT >> AUDCODEC_LP_PLL_CAL_RESULT_PLL_CNT_Pos);
        hwp_audcodec_lp->PLL_CAL_CFG &= ~AUDCODEC_LP_PLL_CAL_CFG_EN;
        if (pll_cnt < target_cnt)
        {
            delta_cnt_min = target_cnt - pll_cnt;
        }
        else if (pll_cnt > target_cnt)
        {
            delta_cnt_min = pll_cnt - target_cnt;
        }

        hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_FC_VCO;
        hwp_audcodec_lp->PLL_CFG0 |= (fc_vco_max << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
        hwp_audcodec_lp->PLL_CAL_CFG |= AUDCODEC_LP_PLL_CAL_CFG_EN;
        while (!(hwp_audcodec_lp->PLL_CAL_CFG & AUDCODEC_LP_PLL_CAL_CFG_DONE_Msk));
        pll_cnt = (hwp_audcodec_lp->PLL_CAL_RESULT >> AUDCODEC_LP_PLL_CAL_RESULT_PLL_CNT_Pos);
        hwp_audcodec_lp->PLL_CAL_CFG &= ~AUDCODEC_LP_PLL_CAL_CFG_EN;
        if (pll_cnt < target_cnt)
        {
            delta_cnt_max = target_cnt - pll_cnt;
        }
        else if (pll_cnt > target_cnt)
        {
            delta_cnt_max = pll_cnt - target_cnt;
        }

        hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_FC_VCO;
#ifdef SOC_SF32LB58X
        if (delta_cnt_min <= delta_cnt && delta_cnt_min <= delta_cnt_max)
        {
            fc_vco_min  = (fc_vco_min  < 2) ? 2 : fc_vco_min;
            hwp_audcodec_lp->PLL_CFG0 |= ((fc_vco_min - 2) << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
        }
        else if (delta_cnt_max <= delta_cnt && delta_cnt_max <= delta_cnt_min)
        {
            fc_vco_max  = (fc_vco_max  < 2) ? 2 : fc_vco_max;
            hwp_audcodec_lp->PLL_CFG0 |= ((fc_vco_max - 2) << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
        }
        else
        {
            fc_vco  = (fc_vco  < 2) ? 2 : fc_vco;
            hwp_audcodec_lp->PLL_CFG0 |= ((fc_vco - 2) << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
        }
#else
        if (delta_cnt_min <= delta_cnt && delta_cnt_min <= delta_cnt_max)
        {
            hwp_audcodec_lp->PLL_CFG0 |= (fc_vco_min << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
        }
        else if (delta_cnt_max <= delta_cnt && delta_cnt_max <= delta_cnt_min)
        {
            hwp_audcodec_lp->PLL_CFG0 |= (fc_vco_max << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
        }
        else
        {
            hwp_audcodec_lp->PLL_CFG0 |= (fc_vco << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
        }
#endif
        hwp_audcodec_lp->PLL_CFG2 &= ~AUDCODEC_LP_PLL_CFG2_EN_LF_VCIN;
        hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_OPEN;

//-----------------step 4------------------//
// set pll to 49.152M
// wait 50us
        //wait(2500);
        HAL_Delay_us(50);

        do
        {
            test_result = updata_pll_freq(freq_type);
        }
        while (test_result != 0);

        set_pll_state(1);
        set_pll_freq_type(freq_type);
    }
    else if (freq_type != get_pll_freq_type())
    {
        do
        {
            test_result = updata_pll_freq(freq_type);
        }
        while (test_result != 0);
        set_pll_freq_type(freq_type);
    }


    return test_result;
}
#endif

typedef struct pll_vco
{
    uint32_t freq;
    uint32_t vco_value;
    uint32_t target_cnt;
} pll_vco_t;
pll_vco_t g_pll_vco_tab[2] =
{
    {48, 0, 2001},
    {44, 0, 1834},
};
int bf0_pll_calibration()
{
    uint32_t pll_cnt;
    uint32_t xtal_cnt;
    uint32_t fc_vco;
    uint32_t fc_vco_min;
    uint32_t fc_vco_max;
    uint32_t delta_cnt;
    uint32_t delta_cnt_min;
    uint32_t delta_cnt_max;
    uint32_t delta_fc_vco;
    uint32_t target_cnt;

    HAL_PMU_EnableAudio(1);
    HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_HP);
    HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_LP);

    HAL_TURN_ON_PLL();


// VCO freq calibration
    hwp_audcodec_lp->PLL_CFG0 |= AUDCODEC_LP_PLL_CFG0_OPEN;
    hwp_audcodec_lp->PLL_CFG2 |= AUDCODEC_LP_PLL_CFG2_EN_LF_VCIN;
    hwp_audcodec_lp->PLL_CAL_CFG = (0    << AUDCODEC_LP_PLL_CAL_CFG_EN_Pos) |
                                   (2000 << AUDCODEC_LP_PLL_CAL_CFG_LEN_Pos);
    for (int i = 0; i < sizeof(g_pll_vco_tab) / sizeof(g_pll_vco_tab[0]); i++)
    {
        target_cnt = g_pll_vco_tab[i].target_cnt;
        fc_vco = 16;
        delta_fc_vco = 8;
        // setup calibration and run
        // target pll_cnt = ceil(46MHz/48MHz*2000)+1 = 1918
        // target difference between pll_cnt and xtal_cnt should be less than 1
        while (delta_fc_vco != 0)
        {
            hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_FC_VCO;
            hwp_audcodec_lp->PLL_CFG0 |= (fc_vco << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
            hwp_audcodec_lp->PLL_CAL_CFG |= AUDCODEC_LP_PLL_CAL_CFG_EN;
            while (!(hwp_audcodec_lp->PLL_CAL_CFG & AUDCODEC_LP_PLL_CAL_CFG_DONE_Msk));
            pll_cnt = (hwp_audcodec_lp->PLL_CAL_RESULT >> AUDCODEC_LP_PLL_CAL_RESULT_PLL_CNT_Pos);
            xtal_cnt = (hwp_audcodec_lp->PLL_CAL_RESULT & AUDCODEC_LP_PLL_CAL_RESULT_XTAL_CNT_Msk);
            hwp_audcodec_lp->PLL_CAL_CFG &= ~AUDCODEC_LP_PLL_CAL_CFG_EN;
            if (pll_cnt < target_cnt)
            {
                fc_vco = fc_vco + delta_fc_vco;
                delta_cnt = target_cnt - pll_cnt;
            }
            else if (pll_cnt > target_cnt)
            {
                fc_vco = fc_vco - delta_fc_vco;
                delta_cnt = pll_cnt - target_cnt;
            }
            delta_fc_vco = delta_fc_vco >> 1;
        }
        rt_kprintf("call par CFG1(%x)\r\n", hwp_audcodec_lp->PLL_CFG1);
        if (fc_vco == 0)
        {
            fc_vco_min = 0;
        }
        else
        {
            fc_vco_min = fc_vco - 1;
        }
        if (fc_vco == 31)
        {
            fc_vco_max = fc_vco;
        }
        else
        {
            fc_vco_max = fc_vco + 1;
        }

        hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_FC_VCO;
        hwp_audcodec_lp->PLL_CFG0 |= (fc_vco_min << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
        hwp_audcodec_lp->PLL_CAL_CFG |= AUDCODEC_LP_PLL_CAL_CFG_EN;
        rt_kprintf("fc %d, xtal %d, pll %d\r\n", fc_vco, xtal_cnt, pll_cnt);
        while (!(hwp_audcodec_lp->PLL_CAL_CFG & AUDCODEC_LP_PLL_CAL_CFG_DONE_Msk));
        pll_cnt = (hwp_audcodec_lp->PLL_CAL_RESULT >> AUDCODEC_LP_PLL_CAL_RESULT_PLL_CNT_Pos);
        hwp_audcodec_lp->PLL_CAL_CFG &= ~AUDCODEC_LP_PLL_CAL_CFG_EN;
        if (pll_cnt < target_cnt)
        {
            delta_cnt_min = target_cnt - pll_cnt;
        }
        else if (pll_cnt > target_cnt)
        {
            delta_cnt_min = pll_cnt - target_cnt;
        }

        hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_FC_VCO;
        hwp_audcodec_lp->PLL_CFG0 |= (fc_vco_max << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
        hwp_audcodec_lp->PLL_CAL_CFG |= AUDCODEC_LP_PLL_CAL_CFG_EN;
        while (!(hwp_audcodec_lp->PLL_CAL_CFG & AUDCODEC_LP_PLL_CAL_CFG_DONE_Msk));
        pll_cnt = (hwp_audcodec_lp->PLL_CAL_RESULT >> AUDCODEC_LP_PLL_CAL_RESULT_PLL_CNT_Pos);
        hwp_audcodec_lp->PLL_CAL_CFG &= ~AUDCODEC_LP_PLL_CAL_CFG_EN;
        if (pll_cnt < target_cnt)
        {
            delta_cnt_max = target_cnt - pll_cnt;
        }
        else if (pll_cnt > target_cnt)
        {
            delta_cnt_max = pll_cnt - target_cnt;
        }

#ifdef SOC_SF32LB58X
        if (delta_cnt_min <= delta_cnt && delta_cnt_min <= delta_cnt_max)
        {
            fc_vco_min  = (fc_vco_min  < 2) ? 2 : fc_vco_min;
            g_pll_vco_tab[i].vco_value = fc_vco_min - 2;
        }
        else if (delta_cnt_max <= delta_cnt && delta_cnt_max <= delta_cnt_min)
        {
            fc_vco_max  = (fc_vco_max  < 2) ? 2 : fc_vco_max;
            g_pll_vco_tab[i].vco_value = fc_vco_max - 2;
        }
        else
        {
            fc_vco  = (fc_vco  < 2) ? 2 : fc_vco;
            g_pll_vco_tab[i].vco_value = fc_vco - 2;
        }
#else
        if (delta_cnt_min <= delta_cnt && delta_cnt_min <= delta_cnt_max)
        {
            g_pll_vco_tab[i].vco_value = fc_vco_min;
        }
        else if (delta_cnt_max <= delta_cnt && delta_cnt_max <= delta_cnt_min)
        {
            g_pll_vco_tab[i].vco_value = fc_vco_max;
        }
        else
        {
            g_pll_vco_tab[i].vco_value = fc_vco;
        }
#endif
    }
    hwp_audcodec_lp->PLL_CFG2 &= ~AUDCODEC_LP_PLL_CFG2_EN_LF_VCIN;
    hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_OPEN;

//-----------------step 4------------------//
// set pll to 49.152M
// wait 50us
    //wait(2500);
    //HAL_Delay_us(50);

    HAL_TURN_OFF_PLL();

    return 0;
}
/**
 * @brief  enable PLL function
 * @param freq - frequency
 * @param type - 0:1024 series, 1:1000 series
 * @return
 */
int bf0_enable_pll(uint32_t freq, uint8_t type)//just add rt_kprintf
{
    uint8_t freq_type;
    uint8_t test_result = -1;
    uint8_t vco_index = 0;

    rt_kprintf("enable pll \n");
    freq_type = type << 1;
    if ((freq == 44100) || (freq == 22050) || (freq == 11025))
    {
        vco_index = 1;
        freq_type += 1;
    }

    HAL_TURN_ON_PLL();

    hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_FC_VCO;
    hwp_audcodec_lp->PLL_CFG0 |= (g_pll_vco_tab[vco_index].vco_value << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);

    rt_kprintf("new PLL_ENABLE vco:%d, freq_type:%d\n", g_pll_vco_tab[vco_index].vco_value, freq_type);
    do
    {
        test_result = updata_pll_freq(freq_type);
    }
    while (test_result != 0);

    return test_result;
}

int bf0_update_pll(uint32_t freq, uint8_t type)//just add rt_kprintf
{
    uint8_t freq_type;
    uint8_t test_result = -1;
    uint8_t vco_index = 0;

    freq_type = type << 1;
    if ((freq == 44100) || (freq == 22050) || (freq == 11025))
    {
        vco_index = 1;
        freq_type += 1;
    }

    hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_FC_VCO;
    hwp_audcodec_lp->PLL_CFG0 |= (g_pll_vco_tab[vco_index].vco_value << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);

    rt_kprintf("new PLL_ENABLE vco:%d, freq_type:%d\n", g_pll_vco_tab[vco_index].vco_value, freq_type);
    do
    {
        test_result = updata_pll_freq(freq_type);
    }
    while (test_result != 0);

    return test_result;
}

static void clear_pll_enable_flag();
void bf0_disable_pll()
{
    HAL_TURN_OFF_PLL();
    clear_pll_enable_flag();
    //set_pll_state(0);
    rt_kprintf("PLL disable\n");
}
INIT_COMPONENT_EXPORT(bf0_pll_calibration);

void bf0_audcodec_reset()
{
    HAL_RCC_ResetModule(RCC_MOD_AUDCODEC_HP);
    HAL_RCC_ResetModule(RCC_MOD_AUDCODEC_LP);

    //HAL_RCC_DisableModule(RCC_MOD_AUDCODEC_HP);
    //HAL_RCC_DisableModule(RCC_MOD_AUDCODEC_LP);

    //HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_HP);
    //HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_LP);
    rt_kprintf("bf0_audcodec_reset\n");
}


#if defined(BSP_ENABLE_AUD_CODEC) ||defined(_SIFLI_DOXYGEN_)

#define DBG_TAG              "drv.audcodec"
//#define DBG_LEVEL                DBG_LOG
#include "drv_log.h"

#define AUDCODEC_DMA_RBF_NUM  10

typedef enum AUDIO_PLL_STATE_TAG
{
    AUDIO_PLL_CLOSED,
    AUDIO_PLL_OPEN,
    AUDIO_PLL_ENABLE,
} AUDIO_PLL_STATE;

struct bf0_audio_codec
{
    struct rt_audio_device audio_device;    /*!< parent  audio device registerd to OS*/

    AUDCODEC_HandleTypeDef audcodec;
    //uint32_t slot_valid;
    uint8_t *queue_buf[HAL_AUDCODEC_INSTANC_CNT];
    uint8_t tx_instanc;
    uint8_t rx_instanc;
    bool    tx_rbf_enable;
    bool    rx_rbf_enable;
    AUDIO_PLL_STATE    pll_state;
    uint32_t pll_samplerate;
    uint8_t rbf_tx_pool[AUDCODEC_DMA_RBF_NUM];
    uint8_t rbf_rx_pool[AUDCODEC_DMA_RBF_NUM];
    struct rt_ringbuffer *rbf_tx_instanc;
    struct rt_ringbuffer *rbf_rx_instanc;
};


const AUDCODE_DAC_CLK_CONFIG_TYPE   codec_dac_clk_config[9] =
{
#if ALL_CLK_USING_PLL
    {48000, 1, 10, 0, 0x14D, 1,  5, 4, 2, 20, 20},
    {32000, 1, 10, 1, 0x14D, 1,  5, 4, 2, 20, 20},
    {24000, 1, 20, 0, 0x14D, 1, 10, 2, 2, 10, 10},
    {16000, 1, 10, 2, 0x14D, 1,  5, 4, 2, 20, 20},
    {12000, 1, 40, 0, 0x14D, 1, 20, 2, 1,  5,  5},
    { 8000, 1, 20, 2, 0x14D, 1, 10, 2, 2, 10, 10},
#else
    {48000, 0, 10, 0, 0x14D, 0,  5, 4, 2, 20, 20},
    {32000, 0, 10, 1, 0x14D, 0,  5, 4, 2, 20, 20},
    {24000, 0, 20, 0, 0x14D, 0, 10, 2, 2, 10, 10},
    {16000, 0, 10, 2, 0x14D, 0,  5, 4, 2, 20, 20},//{16000, 0, 20, 1, 0x14D, 0, 10, 2, 2, 10, 10},
    {12000, 0, 40, 0, 0x14D, 0, 20, 2, 1,  5,  5},
    { 8000, 0, 20, 2, 0x14D, 0, 10, 2, 2, 10, 10},
#endif
    {44100, 1, 10, 0, 0x14D, 1,  5, 4, 2, 20, 20},
    {22050, 1, 20, 0, 0x14D, 1, 10, 2, 2, 10, 10},
    {11025, 1, 40, 0, 0x14D, 1, 20, 2, 1,  5,  5},
};


const AUDCODE_ADC_CLK_CONFIG_TYPE   codec_adc_clk_config[9] =
{
#if ALL_CLK_USING_PLL
    {48000, 1,  5, 0, 1, 1, 5, 0},
    {32000, 1,  5, 1, 1, 1, 5, 0},
    {24000, 1, 10, 0, 1, 0, 5, 2},
    {16000, 1, 10, 1, 1, 0, 5, 2},
    {12000, 1, 10, 2, 1, 0, 5, 2},
    { 8000, 1, 10, 3, 1, 0, 5, 2},
#else
    {48000, 0,  5, 0, 0, 1, 5, 0},
    {32000, 0,  5, 1, 0, 1, 5, 0},
    {24000, 0, 10, 0, 0, 0, 5, 2},
    {16000, 0, 10, 1, 0, 0, 5, 2},
    {12000, 0, 10, 2, 0, 0, 5, 2},
    { 8000, 0, 10, 3, 0, 0, 5, 2},
#endif
    {44100, 1,  5, 0, 1, 1, 5, 1},
    {22050, 1,  5, 0, 1, 1, 5, 1},
    {11025, 1, 10, 2, 1, 0, 5, 3},
};


static struct bf0_audio_codec h_aud_codec;

AUDCODEC_HandleTypeDef *get_audcodec_handle()
{
    return &h_aud_codec.audcodec;
}
static void clear_pll_enable_flag()
{
    h_aud_codec.pll_state = AUDIO_PLL_CLOSED;
}
static void ADCODEC_DUMP_REG()
{
    volatile uint32_t *ptr = (volatile uint32_t *)AUDCODEC_HP_BASE; //LP_BASE
    int i;
    for (i = 0; i < 40; i++)
    {
        LOG_RAW("0x%08x ", *ptr);
        ptr++;
        if ((i & 3) == 3)
            LOG_RAW("\r\n");
    }
}

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
void AUDCODEC_DAC0_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("AUDCODEC_DAC0_DMA_IRQHandler");

    HAL_DMA_IRQHandler(h_aud_codec.audcodec.hdma[0]);

    /* leave interrupt */
    rt_interrupt_leave();
}

void AUDCODEC_DAC1_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("AUDCODEC_DAC1_DMA_IRQHandler");

    HAL_DMA_IRQHandler(h_aud_codec.audcodec.hdma[1]);

    /* leave interrupt */
    rt_interrupt_leave();
}

void AUDCODEC_ADC0_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("AUDCODEC_ADC0_DMA_IRQHandler");

    HAL_DMA_IRQHandler(h_aud_codec.audcodec.hdma[2]);

    /* leave interrupt */
    rt_interrupt_leave();
}

void AUDCODEC_ADC1_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("AUDCODEC_ADC1_DMA_IRQHandler");

    HAL_DMA_IRQHandler(h_aud_codec.audcodec.hdma[3]);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

static rt_err_t bf0_audio_getcaps(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t result = RT_EOK;

    return result;
}

static rt_err_t bf0_audio_configure(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t result = RT_EOK;
    struct bf0_audio_codec *audcodec = (struct bf0_audio_codec *) audio;
    AUDCODEC_HandleTypeDef *haudcodec = (AUDCODEC_HandleTypeDef *) & (audcodec->audcodec);


    if (audio == NULL || caps == NULL)
        return RT_ERROR;

    switch (caps->main_type)
    {
    case AUDIO_TYPE_INPUT:
    {
        uint8_t i;

        for (i = 0; i < 9; i++)
        {
            if (caps->udata.config.samplerate == codec_adc_clk_config[i].samplerate)
            {
                haudcodec->Init.samplerate_index = i;
                haudcodec->Init.adc_cfg.adc_clk = (AUDCODE_ADC_CLK_CONFIG_TYPE *)&codec_adc_clk_config[i];
                break;
            }
        }

        if (caps->sub_type & (1 << (HAL_AUDCODEC_ADC_CH0)))
        {
#ifdef BSP_AUDCODEC_ADC0_DMA
            if (haudcodec->buf[HAL_AUDCODEC_ADC_CH0] == NULL)
            {
                haudcodec->buf[HAL_AUDCODEC_ADC_CH0] = calloc(1, haudcodec->bufSize);
                RT_ASSERT(haudcodec->buf[HAL_AUDCODEC_ADC_CH0]);
                if (haudcodec->buf[HAL_AUDCODEC_ADC_CH0] == NULL)
                    return RT_ERROR_MEMFAULT;
            }
#endif
            audcodec->rx_instanc = HAL_AUDCODEC_ADC_CH0;
        }

        if (caps->sub_type & (1 << (HAL_AUDCODEC_ADC_CH1)))
        {
#ifdef BSP_AUDCODEC_ADC1_DMA
            if (haudcodec->buf[HAL_AUDCODEC_ADC_CH1] == NULL)
            {
                haudcodec->buf[HAL_AUDCODEC_ADC_CH1] = calloc(1, haudcodec->bufSize);
                RT_ASSERT(haudcodec->buf[HAL_AUDCODEC_ADC_CH1]);
                if (haudcodec->buf[HAL_AUDCODEC_ADC_CH1] == NULL)
                    return RT_ERROR_MEMFAULT;
            }
#endif
            audcodec->rx_instanc = HAL_AUDCODEC_ADC_CH1;
        }

    }
    break;
    case AUDIO_TYPE_OUTPUT:
    {
        uint8_t i;

        for (i = 0; i < 9; i++)
        {
            if (caps->udata.config.samplerate == codec_dac_clk_config[i].samplerate)
            {
                haudcodec->Init.samplerate_index = i;
                haudcodec->Init.dac_cfg.dac_clk = (AUDCODE_DAC_CLK_CONFIG_TYPE *)&codec_dac_clk_config[i];
                break;
            }
        }

        if (caps->sub_type & (1 << (HAL_AUDCODEC_DAC_CH0)))
        {
#ifdef BSP_AUDCODEC_DAC0_DMA
            if (haudcodec->buf[HAL_AUDCODEC_DAC_CH0] == NULL)
            {
                haudcodec->buf[HAL_AUDCODEC_DAC_CH0] = calloc(1, haudcodec->bufSize);
                RT_ASSERT(haudcodec->buf[HAL_AUDCODEC_DAC_CH0]);
                if (haudcodec->buf[HAL_AUDCODEC_DAC_CH0] == NULL)
                    return RT_ERROR_MEMFAULT;

            }
            audcodec->queue_buf[HAL_AUDCODEC_DAC_CH0] = haudcodec->buf[HAL_AUDCODEC_DAC_CH0];
#endif
            HAL_AUDCODEC_Config_TChanel(haudcodec, 0, &haudcodec->Init.dac_cfg);

            audcodec->tx_instanc = HAL_AUDCODEC_DAC_CH0;
        }

        if (caps->sub_type & (1 << (HAL_AUDCODEC_DAC_CH1)))
        {
#ifdef BSP_AUDCODEC_DAC1_DMA
            if (haudcodec->buf[HAL_AUDCODEC_DAC_CH1] == NULL)
            {
                haudcodec->buf[HAL_AUDCODEC_DAC_CH1] = calloc(1, haudcodec->bufSize);
                RT_ASSERT(haudcodec->buf[HAL_AUDCODEC_DAC_CH1]);
                if (haudcodec->buf[HAL_AUDCODEC_DAC_CH1] == NULL)
                    return RT_ERROR_MEMFAULT;
            }
            audcodec->queue_buf[HAL_AUDCODEC_DAC_CH1] = haudcodec->buf[HAL_AUDCODEC_DAC_CH1];

#endif
            HAL_AUDCODEC_Config_TChanel(haudcodec, 1, &haudcodec->Init.dac_cfg);

            audcodec->tx_instanc = HAL_AUDCODEC_DAC_CH1;
        }

    }
    break;

    default:
        result = -RT_ERROR;
        break;
    }

    return result;

}

static rt_err_t bf0_audio_init(struct rt_audio_device *audio)
{
    struct bf0_audio_codec *audcodec = (struct bf0_audio_codec *) audio;
    AUDCODEC_HandleTypeDef *haudcodec = (AUDCODEC_HandleTypeDef *) & (audcodec->audcodec);

    // init dma handle and request, other parameters configure in HAL driver
#ifdef BSP_AUDCODEC_DAC0_DMA
    haudcodec->hdma[HAL_AUDCODEC_DAC_CH0] = malloc(sizeof(DMA_HandleTypeDef));

    if (NULL == haudcodec->hdma[HAL_AUDCODEC_DAC_CH0])
    {
        return RT_ENOMEM;
    }
    memset(haudcodec->hdma[HAL_AUDCODEC_DAC_CH0], 0, sizeof(DMA_HandleTypeDef));

    haudcodec->hdma[HAL_AUDCODEC_DAC_CH0]->Instance                 = AUDCODEC_DAC0_DMA_INSTANCE;
    haudcodec->hdma[HAL_AUDCODEC_DAC_CH0]->Init.Request             = AUDCODEC_DAC0_DMA_REQUEST;
#endif

#ifdef BSP_AUDCODEC_DAC1_DMA
    haudcodec->hdma[HAL_AUDCODEC_DAC_CH1] = malloc(sizeof(DMA_HandleTypeDef));

    if (NULL == haudcodec->hdma[HAL_AUDCODEC_DAC_CH1])
    {
        return RT_ENOMEM;
    }
    memset(haudcodec->hdma[HAL_AUDCODEC_DAC_CH1], 0, sizeof(DMA_HandleTypeDef));

    haudcodec->hdma[HAL_AUDCODEC_DAC_CH1]->Instance                 = AUDCODEC_DAC1_DMA_INSTANCE;
    haudcodec->hdma[HAL_AUDCODEC_DAC_CH1]->Init.Request             = AUDCODEC_DAC1_DMA_REQUEST;
#endif

#ifdef BSP_AUDCODEC_ADC0_DMA
    haudcodec->hdma[HAL_AUDCODEC_ADC_CH0] = malloc(sizeof(DMA_HandleTypeDef));

    if (NULL == haudcodec->hdma[HAL_AUDCODEC_ADC_CH0])
    {
        return RT_ENOMEM;
    }
    memset(haudcodec->hdma[HAL_AUDCODEC_ADC_CH0], 0, sizeof(DMA_HandleTypeDef));

    haudcodec->hdma[HAL_AUDCODEC_ADC_CH0]->Instance                 = AUDCODEC_ADC0_DMA_INSTANCE;
    haudcodec->hdma[HAL_AUDCODEC_ADC_CH0]->Init.Request             = AUDCODEC_ADC0_DMA_REQUEST;
#endif

#ifdef BSP_AUDCODEC_ADC1_DMA
    haudcodec->hdma[HAL_AUDCODEC_ADC_CH1] = malloc(sizeof(DMA_HandleTypeDef));

    if (NULL == haudcodec->hdma[HAL_AUDCODEC_ADC_CH1])
    {
        return RT_ENOMEM;
    }
    memset(haudcodec->hdma[HAL_AUDCODEC_ADC_CH1], 0, sizeof(DMA_HandleTypeDef));

    haudcodec->hdma[HAL_AUDCODEC_ADC_CH1]->Instance                 = AUDCODEC_ADC1_DMA_INSTANCE;
    haudcodec->hdma[HAL_AUDCODEC_ADC_CH1]->Init.Request             = AUDCODEC_ADC1_DMA_REQUEST;
#endif

    // set clock
    haudcodec->Init.en_dly_sel = 0;
    haudcodec->Init.dac_cfg.opmode = 1;
    haudcodec->Init.adc_cfg.opmode = 1;


    haudcodec->bufSize = CFG_AUDIO_RECORD_PIPE_SIZE * 2;

    int i;
    for (i = 0; i < HAL_AUDCODEC_INSTANC_CNT; i++)
    {
        audcodec->queue_buf[i] = NULL;
    }
    audcodec->rbf_tx_instanc = malloc(sizeof(struct rt_ringbuffer));
    audcodec->rbf_rx_instanc = malloc(sizeof(struct rt_ringbuffer));
    rt_ringbuffer_init(audcodec->rbf_tx_instanc, audcodec->rbf_tx_pool, AUDCODEC_DMA_RBF_NUM);
    rt_ringbuffer_init(audcodec->rbf_rx_instanc, audcodec->rbf_rx_pool, AUDCODEC_DMA_RBF_NUM);

    //bf0_enable_pll(44100, 1); //RCC ENABLE
    HAL_PMU_EnableAudio(1);
    HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_HP);
    HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_LP);
    //hwp_audcodec_lp->ADC_ANA_CFG &= ~AUDCODEC_LP_ADC_ANA_CFG_MICBIAS_CHOP_EN;
//__asm__("B .");
    int res = HAL_AUDCODEC_Init(haudcodec);
    //HAL_AUDCODEC_Config_DACPath(haudcodec, 1);


    LOG_I("HAL_AUDCODEC_Init res %d\n", res);

    return RT_EOK;
}

static rt_err_t bf0_audio_shutdown(struct rt_audio_device *audio)
{
    struct bf0_audio_codec *audcodec = (struct bf0_audio_codec *) audio;
    AUDCODEC_HandleTypeDef *haudcodec = (AUDCODEC_HandleTypeDef *) & (audcodec->audcodec);

#ifdef BSP_AUDCODEC_DAC0_DMA
    if (haudcodec->buf[HAL_AUDCODEC_DAC_CH0] != NULL)
    {
        free(haudcodec->buf[HAL_AUDCODEC_DAC_CH0]);
        haudcodec->buf[HAL_AUDCODEC_DAC_CH0] = NULL;
    }
#endif

#ifdef BSP_AUDCODEC_DAC1_DMA
    if (haudcodec->buf[HAL_AUDCODEC_DAC_CH1] != NULL)
    {
        free(haudcodec->buf[HAL_AUDCODEC_DAC_CH1]);
        haudcodec->buf[HAL_AUDCODEC_DAC_CH1] = NULL;
    }
#endif

#ifdef BSP_AUDCODEC_ADC0_DMA
    if (haudcodec->buf[HAL_AUDCODEC_ADC_CH0] != NULL)
    {
        free(haudcodec->buf[HAL_AUDCODEC_ADC_CH0]);
        haudcodec->buf[HAL_AUDCODEC_ADC_CH0] = NULL;
    }
#endif

#ifdef BSP_AUDCODEC_ADC1_DMA
    if (haudcodec->buf[HAL_AUDCODEC_ADC_CH1] != NULL)
    {
        free(haudcodec->buf[HAL_AUDCODEC_ADC_CH1]);
        haudcodec->buf[HAL_AUDCODEC_ADC_CH1] = NULL;
    }
#endif

    return RT_EOK;
}

static void bf0_audio_pll_config(struct bf0_audio_codec *audcodec, const AUDCODE_ADC_CLK_CONFIG_TYPE *adc_cfg,
                                 const AUDCODE_DAC_CLK_CONFIG_TYPE *dac_cfg, int stream)
{
    if (((stream & 0xff) == AUDIO_STREAM_REPLAY) || ((stream & 0xff) == AUDIO_STREAM_RXandTX))
    {
        if ((stream & 0xff) == AUDIO_STREAM_RXandTX)
        {
            RT_ASSERT(adc_cfg->samplerate == dac_cfg->samplerate);
        }

        if (dac_cfg->clk_src_sel) //pll
        {
            if (audcodec->pll_state == AUDIO_PLL_CLOSED)
            {
                bf0_enable_pll(dac_cfg->samplerate, 1);
                audcodec->pll_state = AUDIO_PLL_ENABLE;
                audcodec->pll_samplerate = dac_cfg->samplerate;
            }
            else
            {
                bf0_update_pll(dac_cfg->samplerate, 1);
                audcodec->pll_state = AUDIO_PLL_ENABLE;
                audcodec->pll_samplerate = dac_cfg->samplerate;
            }
        }
        else //xtal
        {
            if (audcodec->pll_state == AUDIO_PLL_CLOSED)
            {
                HAL_TURN_ON_PLL();
                audcodec->pll_state = AUDIO_PLL_OPEN;
            }
        }
    }
    else if ((stream & 0xff) == AUDIO_STREAM_RECORD)
    {
        if (adc_cfg->clk_src_sel) //pll
        {
            if (audcodec->pll_state == AUDIO_PLL_CLOSED)
            {
                bf0_enable_pll(adc_cfg->samplerate, 1);
                audcodec->pll_state = AUDIO_PLL_ENABLE;
                audcodec->pll_samplerate = adc_cfg->samplerate;
            }
            else
            {
                bf0_update_pll(adc_cfg->samplerate, 1);
                audcodec->pll_state = AUDIO_PLL_ENABLE;
                audcodec->pll_samplerate = adc_cfg->samplerate;
            }
        }
        else //xtal
        {
            if (audcodec->pll_state == AUDIO_PLL_CLOSED)
            {
                HAL_TURN_ON_PLL();
                audcodec->pll_state = AUDIO_PLL_OPEN;
            }
        }
    }
    else
    {
        RT_ASSERT(0);
    }
    //LOG_I("pll config state:%d, samplerate:%d \n", audcodec->pll_state, audcodec->pll_samplerate);
}

/**
  * @brief  Start audio device for recording/playback.
  * @param[in]  audio: audio device handle.
  * @param[in]  stream: lower 8 bit for playback/record, high 8 bit for slot number.
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */
static rt_err_t bf0_audio_start(struct rt_audio_device *audio, int stream)
{
    struct bf0_audio_codec *audcodec = (struct bf0_audio_codec *) audio;
    AUDCODEC_HandleTypeDef *haudcodec = (AUDCODEC_HandleTypeDef *) & (audcodec->audcodec);
    HAL_StatusTypeDef res = HAL_OK;
    uint8_t rx_dma_num = 0;
    uint8_t tx_dma_num = 0;

    bf0_audio_pll_config(audcodec, &codec_adc_clk_config[haudcodec->Init.samplerate_index],
                         &codec_dac_clk_config[haudcodec->Init.samplerate_index], stream);


    if (((stream & 0xff) == AUDIO_STREAM_RECORD) || ((stream & 0xff) == AUDIO_STREAM_RXandTX))
    {
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDCODEC_ADC_CH0) << 8)))
        {
            HAL_AUDCODEC_Config_RChanel(haudcodec, 0, &haudcodec->Init.adc_cfg);

#ifdef AUDCODEC_ADC0_DMA_INSTANCE
            res = HAL_AUDCODEC_Receive_DMA(haudcodec, haudcodec->buf[HAL_AUDCODEC_ADC_CH0], haudcodec->bufSize, HAL_AUDCODEC_ADC_CH0);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(AUDCODEC_ADC0_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif
            rx_dma_num++;
            audcodec->rx_instanc = HAL_AUDCODEC_ADC_CH0;
            haudcodec->channel_ref |= (1 << HAL_AUDCODEC_ADC_CH0);
        }
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDCODEC_ADC_CH1) << 8)))
        {
            HAL_AUDCODEC_Config_RChanel(haudcodec, 1, &haudcodec->Init.adc_cfg);

#ifdef AUDCODEC_ADC1_DMA_INSTANCE
            res = HAL_AUDCODEC_Receive_DMA(haudcodec, haudcodec->buf[HAL_AUDCODEC_ADC_CH1], haudcodec->bufSize, HAL_AUDCODEC_ADC_CH1);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(AUDCODEC_ADC1_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif
            rx_dma_num++;
            audcodec->rx_instanc = HAL_AUDCODEC_ADC_CH1;
            haudcodec->channel_ref |= (1 << HAL_AUDCODEC_ADC_CH1);
        }

    }

    if (((stream & 0xff) == AUDIO_STREAM_REPLAY) || ((stream & 0xff) == AUDIO_STREAM_RXandTX))
    {
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDCODEC_DAC_CH0) << 8)))
        {
#ifdef AUDCODEC_DAC0_DMA_INSTANCE
            res = HAL_AUDCODEC_Transmit_DMA(haudcodec, haudcodec->buf[HAL_AUDCODEC_DAC_CH0], haudcodec->bufSize, HAL_AUDCODEC_DAC_CH0);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(AUDCODEC_DAC0_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif
            tx_dma_num++;
            audcodec->tx_instanc = HAL_AUDCODEC_DAC_CH0;
            haudcodec->channel_ref |= (1 << HAL_AUDCODEC_DAC_CH0);
        }
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDCODEC_DAC_CH1) << 8)))
        {
#ifdef AUDCODEC_DAC1_DMA_INSTANCE
            res = HAL_AUDCODEC_Transmit_DMA(haudcodec, haudcodec->buf[HAL_AUDCODEC_DAC_CH1], haudcodec->bufSize, HAL_AUDCODEC_DAC_CH1);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(AUDCODEC_DAC1_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif
            tx_dma_num++;
            audcodec->tx_instanc = HAL_AUDCODEC_DAC_CH1;
            haudcodec->channel_ref |= (1 << HAL_AUDCODEC_DAC_CH1);
        }


    }

    if (tx_dma_num > 1)
    {
        audcodec->tx_rbf_enable = true;
        rt_ringbuffer_reset(audcodec->rbf_tx_instanc);
    }
    else
    {
        audcodec->tx_rbf_enable = false;
    }

    if (rx_dma_num > 1)
    {
        audcodec->rx_rbf_enable = true;
        rt_ringbuffer_reset(audcodec->rbf_rx_instanc);
    }
    else
    {
        audcodec->rx_rbf_enable = false;
    }


    if (((stream & 0xff) == AUDIO_STREAM_REPLAY) || ((stream & 0xff) == AUDIO_STREAM_RXandTX))
    {
        /* enable AUDCODEC at last*/
        __HAL_AUDCODEC_HP_ENABLE(haudcodec);

        HAL_AUDCODEC_Config_DACPath(haudcodec, 1);
        HAL_AUDCODEC_Config_Analog_DACPath(haudcodec->Init.dac_cfg.dac_clk);
        HAL_AUDCODEC_Config_DACPath(haudcodec, 0);
    }
    if (((stream & 0xff) == AUDIO_STREAM_RECORD) || ((stream & 0xff) == AUDIO_STREAM_RXandTX))
    {


        HAL_AUDCODEC_Config_Analog_ADCPath(haudcodec->Init.adc_cfg.adc_clk);

        /* enable AUDCODEC at last*/
        __HAL_AUDCODEC_LP_ENABLE(haudcodec);
    }

    return RT_EOK;
}

/**
  * @brief  Stop audio device for recording/playback.
  * @param[in]  audio: audio device handle.
  * @param[in]  stream: lower 8 bit for playback/record, high 8 bit for slot number.
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */
static rt_err_t bf0_audio_stop(struct rt_audio_device *audio, int stream)
{
    struct bf0_audio_codec *audcodec = (struct bf0_audio_codec *) audio;
    AUDCODEC_HandleTypeDef *haudcodec = (AUDCODEC_HandleTypeDef *) & (audcodec->audcodec);
    rt_err_t ret = RT_EOK;

    if ((stream == AUDIO_STREAM_REPLAY || stream == AUDIO_STREAM_RECORD) && !haudcodec->channel_ref)
    {
        return RT_EOK;
    }

    LOG_I("bf0_audio_stop 0x%x\n", stream);

    if (((stream & 0xff) == AUDIO_STREAM_RECORD) || ((stream & 0xff) == AUDIO_STREAM_RXandTX))  // rx
    {
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDCODEC_ADC_CH0) << 8)))
        {
#ifdef AUDCODEC_ADC0_DMA_INSTANCE
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(AUDCODEC_ADC0_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            ret = HAL_AUDCODEC_DMAStop(haudcodec, HAL_AUDCODEC_ADC_CH0);
#endif
            haudcodec->channel_ref &= ~(1 << HAL_AUDCODEC_ADC_CH0);
            haudcodec->State[HAL_AUDCODEC_ADC_CH0] = HAL_AUDCODEC_STATE_READY;
        }

        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDCODEC_ADC_CH1) << 8)))
        {
#ifdef AUDCODEC_ADC1_DMA_INSTANCE
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(AUDCODEC_ADC1_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            ret = HAL_AUDCODEC_DMAStop(haudcodec, HAL_AUDCODEC_ADC_CH1);
#endif
            haudcodec->channel_ref &= ~(1 << HAL_AUDCODEC_ADC_CH1);
            haudcodec->State[HAL_AUDCODEC_ADC_CH1] = HAL_AUDCODEC_STATE_READY;
        }
    }

    if (((stream & 0xff) == AUDIO_STREAM_REPLAY) || ((stream & 0xff) == AUDIO_STREAM_RXandTX)) //tx
    {
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDCODEC_DAC_CH0) << 8)))
        {
#ifdef AUDCODEC_DAC0_DMA_INSTANCE
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(AUDCODEC_DAC0_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            ret = HAL_AUDCODEC_DMAStop(haudcodec, HAL_AUDCODEC_DAC_CH0);
#endif
            haudcodec->channel_ref &= ~(1 << HAL_AUDCODEC_DAC_CH0);
            haudcodec->State[HAL_AUDCODEC_DAC_CH0] = HAL_AUDCODEC_STATE_READY;
        }

        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDCODEC_DAC_CH1) << 8)))
        {
#ifdef AUDCODEC_DAC1_DMA_INSTANCE
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(AUDCODEC_DAC1_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            ret = HAL_AUDCODEC_DMAStop(haudcodec, HAL_AUDCODEC_DAC_CH1);
#endif
            haudcodec->channel_ref &= ~(1 << HAL_AUDCODEC_DAC_CH1);
            haudcodec->State[HAL_AUDCODEC_DAC_CH1] = HAL_AUDCODEC_STATE_READY;
        }
    }

    if (((stream & 0xff) == AUDIO_STREAM_REPLAY) || ((stream & 0xff) == AUDIO_STREAM_RXandTX))
    {
        uint16_t dac_mask = (1 << HAL_AUDCODEC_DAC_CH0) | (1 << HAL_AUDCODEC_DAC_CH1);
        if ((haudcodec->channel_ref & dac_mask) == 0)
        {
            LOG_I("audcodec close dac");
            HAL_AUDCODEC_Config_DACPath(haudcodec, 1);
            HAL_AUDCODEC_Close_Analog_DACPath();
            __HAL_AUDCODEC_HP_DISABLE(haudcodec);
            HAL_AUDCODEC_Clear_All_Channel(haudcodec, 1);
        }
        else
        {
            LOG_I("audcodec channel_ref=%d", haudcodec->channel_ref);
        }
    }
    if (((stream & 0xff) == AUDIO_STREAM_RECORD) || ((stream & 0xff) == AUDIO_STREAM_RXandTX) && !haudcodec->channel_ref)
    {
        uint16_t adc_mask = (1 << HAL_AUDCODEC_ADC_CH0) | (1 << HAL_AUDCODEC_ADC_CH1);
        if ((haudcodec->channel_ref & adc_mask) == 0)
        {
            LOG_I("audcodec close adc");
            __HAL_AUDCODEC_LP_DISABLE(haudcodec);
            HAL_AUDCODEC_Close_Analog_ADCPath();
            HAL_AUDCODEC_Clear_All_Channel(haudcodec, 2);
        }
        else
        {
            LOG_I("audcodec channel_ref=%d", haudcodec->channel_ref);
        }
    }

    LOG_I("bf0_audio_stop 0x%x done\n", stream);

    return ret;
}

static rt_err_t bf0_audio_suspend(struct rt_audio_device *audio, int stream)
{
    rt_err_t ret = RT_ERROR;

    return ret;
}

static rt_err_t bf0_audio_resume(struct rt_audio_device *audio, int stream)
{
    rt_err_t ret = RT_ERROR;
    //hwp_audcodec_lp->ADC_ANA_CFG &= ~AUDCODEC_LP_ADC_ANA_CFG_MICBIAS_CHOP_EN;
    return ret;
}


static rt_err_t bf0_audio_control(struct rt_audio_device *audio, int cmd, void *args)
{
    struct bf0_audio_codec *audcodec = (struct bf0_audio_codec *) audio;
    AUDCODEC_HandleTypeDef *haudcodec = (AUDCODEC_HandleTypeDef *) & (audcodec->audcodec);
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case AUDIO_CTL_SETOUTPUT:
    {
        uint32_t intf = (uint32_t)args;
        if (intf == AUDPRC_TX_TO_CODEC)
        {
            haudcodec->Init.dac_cfg.opmode = 0;
        }
        else
        {
            haudcodec->Init.dac_cfg.opmode = 1;
        }
        LOG_D("AUDCODEC set source %d\n", haudcodec->Init.dac_cfg.opmode);
        break;
    }
    case AUDIO_CTL_SETINPUT:
    {
        uint32_t intf = (uint32_t)args;
        if (intf == AUDPRC_RX_FROM_CODEC)
        {
            haudcodec->Init.adc_cfg.opmode = 0;
        }
        else
        {
            haudcodec->Init.adc_cfg.opmode = 1;
        }
        LOG_D("AUDCODEC set dest %d\n", haudcodec->Init.adc_cfg.opmode);
        break;
    }
    case AUDIO_CTL_SETVOLUME:
    {
#define AUDPRC_MIN_VOLUME       -18
#define AUDCODEC_MIN_VOLUME     -36
#define AUDCODEC_MAX_VOLUME     54

        int volume = (int)args;
        int audcodec_volume;
        int audprc_volume;
        if (volume > AUDCODEC_MAX_VOLUME * 2)
            volume = AUDCODEC_MAX_VOLUME * 2;
        if (volume < (AUDPRC_MIN_VOLUME + AUDCODEC_MIN_VOLUME) * 2)
            volume = (AUDPRC_MIN_VOLUME + AUDCODEC_MIN_VOLUME) * 2;

        if (volume >= AUDCODEC_MIN_VOLUME * 2) //Q31.1
        {
            audcodec_volume = volume;
            audprc_volume = 0;
        }
        else
        {
            audcodec_volume = AUDCODEC_MIN_VOLUME * 2;
            audprc_volume = (volume - AUDCODEC_MIN_VOLUME * 2) / 2;
        }
        LOG_I("dac set volume: prc=%d codec=%d\n", audprc_volume, audcodec_volume);
        HAL_AUDCODEC_Config_DACPath_Volume(haudcodec, 0, audcodec_volume);
        HAL_AUDCODEC_Config_DACPath_Volume(haudcodec, 1, audcodec_volume);
#if defined(BSP_ENABLE_AUD_PRC)
        {
            extern void hal_audprc_set_dac_volume(int volume);
            hal_audprc_set_dac_volume(audprc_volume);
        }
#endif

#undef AUDPRC_MIN_VOLUME
#undef AUDCODEC_MIN_VOLUME
#undef AUDCODEC_MAX_VOLUME

        break;
    }
    case AUDIO_CTL_MUTE:
    {
        int mute = (int)args;

        if (mute)
        {
            HAL_AUDCODEC_Mute_DACPath(haudcodec, 1);
        }
        else
        {
            HAL_AUDCODEC_Mute_DACPath(haudcodec, 0);
        }
        LOG_I("codec dac mute set:%d\n", mute);
        break;
    }
    case RT_DEVICE_CTRL_SUSPEND:
    {
        struct bf0_audio_codec *audcodec = (struct bf0_audio_codec *) audio;
        AUDCODEC_HandleTypeDef *haudcodec = (AUDCODEC_HandleTypeDef *) & (audcodec->audcodec);
        HAL_AUDCODEC_DeInit(haudcodec);
        set_pll_state(0);
        break;
    }
    case RT_DEVICE_CTRL_RESUME:
    {
        struct bf0_audio_codec *audcodec = (struct bf0_audio_codec *) audio;
        AUDCODEC_HandleTypeDef *haudcodec = (AUDCODEC_HandleTypeDef *) & (audcodec->audcodec);
        HAL_AUDCODEC_Init(haudcodec);
        //bf0_enable_pll(44100, 1); //RCC ENABLE
        //hwp_pmuc->HXT_CR1 |= PMUC_HXT_CR1_BUF_AUD_EN;
        //hwp_hpsys_rcc->ENR2 |= HPSYS_RCC_ENR2_AUDCODEC;
        //hwp_lpsys_rcc->ENR1 |= LPSYS_RCC_ENR1_AUDCODEC;
        HAL_PMU_EnableAudio(1);
        HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_HP);
        HAL_RCC_EnableModule(RCC_MOD_AUDCODEC_LP);
        audcodec->pll_state = AUDIO_PLL_CLOSED;
        break;
    }
    default:
        result = -RT_ERROR;
        break;
    }

    return result;
}

static rt_size_t bf0_audio_trans(struct rt_audio_device *audio, const void *writeBuf, void *readBuf, rt_size_t size)
{
    struct bf0_audio_codec *audcodec = (struct bf0_audio_codec *) audio;
    AUDCODEC_HandleTypeDef *haudcodec = (AUDCODEC_HandleTypeDef *) & (audcodec->audcodec);
    uint8_t tx_ins = audcodec->tx_instanc;


    if (writeBuf != NULL)
    {
        RT_ASSERT(tx_ins < HAL_AUDCODEC_ADC_CH0);
//#ifdef AUDCODEC_DAC0_DMA_INSTANCE
        if (audcodec->queue_buf[tx_ins] != NULL) //HAL_AUDCODEC_DAC_CH0
        {
            memcpy(audcodec->queue_buf[tx_ins], writeBuf, size);
            if (audcodec->tx_rbf_enable)
            {
                audcodec->tx_instanc = 0xFF;
            }
        }
        else
        {
            RT_ASSERT(false);
        }
//#endif
    }

    if (readBuf != NULL)
    {
#ifdef AUDCODEC_ADC0_DMA_INSTANCE
        //HAL_AUDCODEC_Receive_DMA(haudcodec, haudcodec->buf[HAL_AUDCODEC_ADC_CH0], haudcodec->bufSize, HAL_AUDCODEC_ADC_CH0);
        //HAL_NVIC_EnableIRQ(AUDCODEC_ADC0_DMA_IRQ);
#endif
    }

    return size;
}


static const struct rt_audio_ops       _g_audio_ops =
{
    .getcaps    = bf0_audio_getcaps,
    .configure  = bf0_audio_configure,

    .init       = bf0_audio_init,
    .shutdown   = bf0_audio_shutdown,
    .start      = bf0_audio_start,
    .stop       = bf0_audio_stop,
    .suspend    = bf0_audio_suspend,
    .resume     = bf0_audio_resume,
    .control    = bf0_audio_control,
    .transmit   = bf0_audio_trans,
};


/**
* @brief  Audio Process devices initialization
*/
int rt_bf0_audio_codec_init(void)
{
    int result;

    memset(&h_aud_codec, 0, sizeof(h_aud_codec));

    h_aud_codec.audcodec.Instance_hp = hwp_audcodec_hp;
    h_aud_codec.audcodec.Instance_lp = hwp_audcodec_lp;
    h_aud_codec.audio_device.ops = (struct rt_audio_ops *)&_g_audio_ops;

    result = rt_audio_register((struct rt_audio_device *)&h_aud_codec,
                               "audcodec", RT_DEVICE_FLAG_RDWR, NULL);

    rt_device_init((rt_device_t)(&h_aud_codec.audio_device));
    return result;
}

INIT_DEVICE_EXPORT(rt_bf0_audio_codec_init);

void HAL_AUDCODEC_TxCpltCallback(AUDCODEC_HandleTypeDef *haprc, int cid)
{
    struct bf0_audio_codec *haudio = rt_container_of(haprc, struct bf0_audio_codec, audcodec);
    struct rt_audio_device *audio = &(haudio->audio_device);

    if (audio != NULL)
    {
        haudio->queue_buf[cid] = (rt_uint8_t *)((uint32_t)(haprc->buf[cid]) + haprc->bufSize / 2);
        rt_audio_tx_complete(audio, haudio->queue_buf[cid]);

        if (haudio->tx_rbf_enable)
        {
            rt_size_t putsize;
            uint8_t putdata;
            putdata = (1 << 4) | cid;
            putsize = rt_ringbuffer_put(haudio->rbf_tx_instanc, &putdata, 1);
            RT_ASSERT(putsize == 1);
        }
    }
}

void HAL_AUDCODEC_TxHalfCpltCallback(AUDCODEC_HandleTypeDef *haprc, int cid)
{
    struct bf0_audio_codec *haudio = rt_container_of(haprc, struct bf0_audio_codec, audcodec);
    struct rt_audio_device *audio = &(haudio->audio_device);

    if (audio != NULL)
    {
        haudio->queue_buf[cid] = haprc->buf[cid];
        rt_audio_tx_complete(audio, haudio->queue_buf[cid]);
        if (haudio->tx_rbf_enable)
        {
            rt_size_t putsize;
            uint8_t putdata;
            putdata = cid;
            putsize = rt_ringbuffer_put(haudio->rbf_tx_instanc, &putdata, 1);
            RT_ASSERT(putsize == 1);
        }
    }
}

void HAL_AUDCODEC_RxCpltCallback(AUDCODEC_HandleTypeDef *haprc, int cid)
{
    struct bf0_audio_codec *haudio = rt_container_of(haprc, struct bf0_audio_codec, audcodec);
    struct rt_audio_device *audio = &(haudio->audio_device);

    if (audio != NULL)
    {
        haudio->queue_buf[cid] = (rt_uint8_t *)((uint32_t)(haprc->buf[cid]) + haprc->bufSize / 2);
        rt_audio_rx_done(audio, haudio->queue_buf[cid], haprc->bufSize / 2);

        if (haudio->rx_rbf_enable)
        {
            rt_size_t putsize;
            uint8_t putdata;
            putdata = (1 << 4) | cid;
            putsize = rt_ringbuffer_put(haudio->rbf_rx_instanc, &putdata, 1);
            RT_ASSERT(putsize == 1);
        }
    }
}
void HAL_AUDCODEC_RxHalfCpltCallback(AUDCODEC_HandleTypeDef *haprc, int cid)
{
    struct bf0_audio_codec *haudio = rt_container_of(haprc, struct bf0_audio_codec, audcodec);
    struct rt_audio_device *audio = &(haudio->audio_device);
    if (audio != NULL)
    {
        haudio->queue_buf[cid] = haprc->buf[cid];
        rt_audio_rx_done(audio, haudio->queue_buf[cid], haprc->bufSize / 2);
        if (haudio->rx_rbf_enable)
        {
            rt_size_t putsize;
            uint8_t putdata;
            putdata = cid;
            putsize = rt_ringbuffer_put(haudio->rbf_rx_instanc, &putdata, 1);
            RT_ASSERT(putsize == 1);
        }
    }
}

uint8_t bf0_audcodec_get_tx_channel()
{

    if (h_aud_codec.tx_rbf_enable)
    {
        rt_size_t getsize;
        uint8_t getdata;
        getsize = rt_ringbuffer_get(h_aud_codec.rbf_tx_instanc, &getdata, 1);
        RT_ASSERT(getsize == 1);
        h_aud_codec.tx_instanc = getdata & 0xF;
    }

    return h_aud_codec.tx_instanc;
}

void bf0_audcodec_set_tx_channel(uint8_t chan)
{
    RT_ASSERT(chan < HAL_AUDCODEC_ADC_CH0);
    h_aud_codec.tx_instanc = chan;
}


uint8_t bf0_audcodec_get_rx_channel()
{

    if (h_aud_codec.rx_rbf_enable)
    {
        rt_size_t getsize;
        uint8_t getdata;
        getsize = rt_ringbuffer_get(h_aud_codec.rbf_rx_instanc, &getdata, 1);
        RT_ASSERT(getsize == 1);
        h_aud_codec.rx_instanc = getdata & 0xF;
    }

    return h_aud_codec.rx_instanc;
}

void bf0_audcodec_set_rx_channel(uint8_t chan)
{
    RT_ASSERT((chan >= HAL_AUDCODEC_ADC_CH0) && (chan < HAL_AUDCODEC_INSTANC_CNT));
    h_aud_codec.rx_instanc = chan;
}

void bf0_audcodec_device_write(rt_device_t dev, rt_off_t    pos, const void *buffer, rt_size_t   size) /*para is same to rt_device_write*/
//(struct rt_audio_device *audio, const void *writeBuf, void *readBuf, rt_size_t size)
{
    struct rt_audio_device *audio = (struct rt_audio_device *)dev;

    bf0_audio_trans(audio, buffer, NULL, size);
}


#define TEST_PASS       0x1
#define TEST_UNFINISHED 0x2

uint8_t codec_hp_sin1k_test()
{
    uint8_t test_result = TEST_UNFINISHED;
    uint32_t rdata;
    uint32_t pll_cnt;
    uint32_t xtal_cnt;
    uint32_t fc_vco;
    uint32_t fc_vco_min;
    uint32_t fc_vco_max;
    uint32_t delta_cnt;
    uint32_t delta_cnt_min;
    uint32_t delta_cnt_max;
    uint32_t delta_fc_vco;
    uint32_t target_cnt = 1838;

    fc_vco = 16;
    delta_fc_vco = 8;

    test_result = TEST_PASS;
// setup clock

//-----------------step 1------------------//
// wait xtal clock ready
    while (!(hwp_hpsys_aon->ACR & HPSYS_AON_ACR_HXT48_RDY_Msk));
    hwp_hpsys_rcc->CSR |= 1 << HPSYS_RCC_CSR_SEL_SYS_Pos;     //select xtal 48MHz
    hwp_hpsys_rcc->CFGR = (1 << HPSYS_RCC_CFGR_HDIV_Pos) |
                          (1 << HPSYS_RCC_CFGR_PDIV1_Pos) |
                          (7 << HPSYS_RCC_CFGR_PDIV2_Pos);
    hwp_pmuc->HXT_CR1 |= PMUC_HXT_CR1_BUF_AUD_EN;
    hwp_hpsys_rcc->ENR2 |= HPSYS_RCC_ENR2_AUDCODEC;
    hwp_lpsys_rcc->ENR1 |= LPSYS_RCC_ENR1_AUDCODEC;

//-----------------step 2------------------//
// turn on bandgap
// turn on BG sample clock
    hwp_audcodec_lp->BG_CFG1 = 48000;
    hwp_audcodec_lp->BG_CFG2 = 48000000;
// turn on bandgap
    hwp_audcodec_lp->BG_CFG0 &= ~AUDCODEC_LP_BG_CFG0_EN_RCFLT;
    hwp_audcodec_lp->BG_CFG0 |= AUDCODEC_LP_BG_CFG0_EN;

//-----------------step 3------------------//
// pll calibration
// wait 100us
    //wait(5000);
    HAL_Delay_us(100);
    hwp_audcodec_lp->PLL_CFG0 |= AUDCODEC_LP_PLL_CFG0_EN_IARY;
    hwp_audcodec_lp->PLL_CFG0 |= AUDCODEC_LP_PLL_CFG0_EN_VCO;
    hwp_audcodec_lp->PLL_CFG0 |= AUDCODEC_LP_PLL_CFG0_EN_ANA;
    hwp_audcodec_lp->PLL_CFG2 |= AUDCODEC_LP_PLL_CFG2_EN_DIG;
    hwp_audcodec_lp->PLL_CFG3 |= AUDCODEC_LP_PLL_CFG3_EN_SDM;
    hwp_audcodec_lp->PLL_CFG4 |= AUDCODEC_LP_PLL_CFG4_EN_CLK_DIG;
// wait 50us
    //wait(2500);
    HAL_Delay_us(50);
// VCO freq calibration
    hwp_audcodec_lp->PLL_CFG0 |= AUDCODEC_LP_PLL_CFG0_OPEN;
    hwp_audcodec_lp->PLL_CFG2 |= AUDCODEC_LP_PLL_CFG2_EN_LF_VCIN;
    hwp_audcodec_lp->PLL_CAL_CFG = (0    << AUDCODEC_LP_PLL_CAL_CFG_EN_Pos) |
                                   (2000 << AUDCODEC_LP_PLL_CAL_CFG_LEN_Pos);
// setup calibration and run
// target pll_cnt = ceil(44.1MHz/48MHz*2000)+1 = 1838
// target difference between pll_cnt and xtal_cnt should be less than 1
    while (delta_fc_vco != 0)
    {
        hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_FC_VCO;
        hwp_audcodec_lp->PLL_CFG0 |= (fc_vco << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
        hwp_audcodec_lp->PLL_CAL_CFG |= AUDCODEC_LP_PLL_CAL_CFG_EN;
        while (!(hwp_audcodec_lp->PLL_CAL_CFG & AUDCODEC_LP_PLL_CAL_CFG_DONE_Msk));
        pll_cnt = (hwp_audcodec_lp->PLL_CAL_RESULT >> AUDCODEC_LP_PLL_CAL_RESULT_PLL_CNT_Pos);
        xtal_cnt = (hwp_audcodec_lp->PLL_CAL_RESULT & AUDCODEC_LP_PLL_CAL_RESULT_XTAL_CNT_Msk);
        hwp_audcodec_lp->PLL_CAL_CFG &= ~AUDCODEC_LP_PLL_CAL_CFG_EN;
        if (pll_cnt < target_cnt)
        {
            fc_vco = fc_vco + delta_fc_vco;
            delta_cnt = target_cnt - pll_cnt;
        }
        else if (pll_cnt > target_cnt)
        {
            fc_vco = fc_vco - delta_fc_vco;
            delta_cnt = pll_cnt - target_cnt;
        }
        delta_fc_vco = delta_fc_vco >> 1;
    }

    fc_vco_min = fc_vco - 1;
    if (fc_vco == 31)
    {
        fc_vco_max = fc_vco;
    }
    else
    {
        fc_vco_max = fc_vco + 1;
    }

    hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_FC_VCO;
    hwp_audcodec_lp->PLL_CFG0 |= (fc_vco_min << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
    hwp_audcodec_lp->PLL_CAL_CFG |= AUDCODEC_LP_PLL_CAL_CFG_EN;
    while (!(hwp_audcodec_lp->PLL_CAL_CFG & AUDCODEC_LP_PLL_CAL_CFG_DONE_Msk));
    pll_cnt = (hwp_audcodec_lp->PLL_CAL_RESULT >> AUDCODEC_LP_PLL_CAL_RESULT_PLL_CNT_Pos);
    hwp_audcodec_lp->PLL_CAL_CFG &= ~AUDCODEC_LP_PLL_CAL_CFG_EN;
    if (pll_cnt < target_cnt)
    {
        delta_cnt_min = target_cnt - pll_cnt;
    }
    else if (pll_cnt > target_cnt)
    {
        delta_cnt_min = pll_cnt - target_cnt;
    }

    hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_FC_VCO;
    hwp_audcodec_lp->PLL_CFG0 |= (fc_vco_max << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
    hwp_audcodec_lp->PLL_CAL_CFG |= AUDCODEC_LP_PLL_CAL_CFG_EN;
    while (!(hwp_audcodec_lp->PLL_CAL_CFG & AUDCODEC_LP_PLL_CAL_CFG_DONE_Msk));
    pll_cnt = (hwp_audcodec_lp->PLL_CAL_RESULT >> AUDCODEC_LP_PLL_CAL_RESULT_PLL_CNT_Pos);
    hwp_audcodec_lp->PLL_CAL_CFG &= ~AUDCODEC_LP_PLL_CAL_CFG_EN;
    if (pll_cnt < target_cnt)
    {
        delta_cnt_max = target_cnt - pll_cnt;
    }
    else if (pll_cnt > target_cnt)
    {
        delta_cnt_max = pll_cnt - target_cnt;
    }

    hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_FC_VCO;
    if (delta_cnt_min <= delta_cnt && delta_cnt_min <= delta_cnt_max)
    {
        hwp_audcodec_lp->PLL_CFG0 |= ((fc_vco_min - 2) << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
    }
    else if (delta_cnt_max <= delta_cnt && delta_cnt_max <= delta_cnt_min)
    {
        hwp_audcodec_lp->PLL_CFG0 |= ((fc_vco_max - 2) << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
    }
    else
    {
        hwp_audcodec_lp->PLL_CFG0 |= ((fc_vco - 2) << AUDCODEC_LP_PLL_CFG0_FC_VCO_Pos);
    }

    hwp_audcodec_lp->PLL_CFG2 &= ~AUDCODEC_LP_PLL_CFG2_EN_LF_VCIN;
    hwp_audcodec_lp->PLL_CFG0 &= ~AUDCODEC_LP_PLL_CFG0_OPEN;
//-----------------step 4------------------//
// set pll to 44.1MHz
    hwp_audcodec_lp->PLL_CFG2 |= AUDCODEC_LP_PLL_CFG2_RSTB;
// wait 50us
    //wait(2500);
    HAL_Delay_us(50);

    hwp_audcodec_lp->PLL_CFG3 = (0x5999A << AUDCODEC_LP_PLL_CFG3_SDIN_Pos) |
                                (4 << AUDCODEC_LP_PLL_CFG3_FCW_Pos) |
                                (0 << AUDCODEC_LP_PLL_CFG3_SDM_UPDATE_Pos) |
                                (1 << AUDCODEC_LP_PLL_CFG3_SDMIN_BYPASS_Pos) |
                                (0 << AUDCODEC_LP_PLL_CFG3_SDM_MODE_Pos) |
                                (0 << AUDCODEC_LP_PLL_CFG3_EN_SDM_DITHER_Pos) |
                                (0 << AUDCODEC_LP_PLL_CFG3_SDM_DITHER_Pos) |
                                (1 << AUDCODEC_LP_PLL_CFG3_EN_SDM_Pos) |
                                (0 << AUDCODEC_LP_PLL_CFG3_SDMCLK_POL_Pos);

    hwp_audcodec_lp->PLL_CFG3 |= AUDCODEC_LP_PLL_CFG3_SDM_UPDATE;
    hwp_audcodec_lp->PLL_CFG3 &= ~AUDCODEC_LP_PLL_CFG3_SDMIN_BYPASS;
    hwp_audcodec_lp->PLL_CFG2 &= ~AUDCODEC_LP_PLL_CFG2_RSTB;
// wait 50us
    //wait(2500);
    HAL_Delay_us(50);
    hwp_audcodec_lp->PLL_CFG2 |= AUDCODEC_LP_PLL_CFG2_RSTB;
// wait 50us
// check pll lock
    //wait(2500);
    HAL_Delay_us(50);
    hwp_audcodec_lp->PLL_CFG1 |= AUDCODEC_LP_PLL_CFG1_CSD_EN | AUDCODEC_LP_PLL_CFG1_CSD_RST;
    hwp_audcodec_lp->PLL_CFG1 &= ~AUDCODEC_LP_PLL_CFG1_CSD_RST;
    if (hwp_audcodec_lp->PLL_STAT & AUDCODEC_LP_PLL_STAT_UNLOCK_Msk)
    {
        rt_kprintf("pll lock fail!\n");
    }
    else
    {
        rt_kprintf("pll lock!\n");
        hwp_audcodec_lp->PLL_CFG1 &= ~AUDCODEC_LP_PLL_CFG1_CSD_EN;
    }

//-----------------step 5------------------//
// turn on refgen
    hwp_audcodec_lp->BG_CFG0 &= ~AUDCODEC_LP_BG_CFG0_VREF_SEL;
    hwp_audcodec_lp->BG_CFG0 |= (7 << AUDCODEC_LP_BG_CFG0_VREF_SEL_Pos);  // AVDD = 3.3V
    //hwp_audcodec_lp->BG_CFG0 |= (2 << AUDCODEC_LP_BG_CFG0_VREF_SEL_Pos);  // AVDD = 1.8V
    hwp_audcodec_lp->REFGEN_CFG &= ~AUDCODEC_LP_REFGEN_CFG_EN_CHOP;
    hwp_audcodec_lp->REFGEN_CFG |= AUDCODEC_LP_REFGEN_CFG_EN;
    hwp_audcodec_lp->PLL_CFG5 = AUDCODEC_LP_PLL_CFG5_EN_CLK_CHOP_BG |
                                AUDCODEC_LP_PLL_CFG5_EN_CLK_CHOP_REFGEN;

//-----------------step 6------------------//
// turn on hp dac1 and dac2 analog
// dac1 and dac2 clock
    hwp_audcodec_lp->PLL_CFG4 = (2  << AUDCODEC_LP_PLL_CFG4_DIVB_CLK_CHOP_DAC_Pos) |
                                (2  << AUDCODEC_LP_PLL_CFG4_DIVA_CLK_CHOP_DAC_Pos) |
                                (1  << AUDCODEC_LP_PLL_CFG4_EN_CLK_CHOP_DAC_Pos) |
                                (10 << AUDCODEC_LP_PLL_CFG4_DIVA_CLK_DAC_Pos) |
                                (1  << AUDCODEC_LP_PLL_CFG4_EN_CLK_DAC_Pos) |
                                (0  << AUDCODEC_LP_PLL_CFG4_SEL_CLK_DAC_SOURCE_Pos) | // select xtal
                                (0  << AUDCODEC_LP_PLL_CFG4_SEL_CLK_DIG_Pos) |
                                (1  << AUDCODEC_LP_PLL_CFG4_CLK_DIG_STR_Pos) |
                                (2  << AUDCODEC_LP_PLL_CFG4_DIVA_CLK_DIG_Pos) |
                                (1  << AUDCODEC_LP_PLL_CFG4_EN_CLK_DIG_Pos);

    hwp_audcodec_lp->PLL_CFG5 = (2  << AUDCODEC_LP_PLL_CFG5_DIVB_CLK_CHOP_BG_Pos) |
                                (10 << AUDCODEC_LP_PLL_CFG5_DIVA_CLK_CHOP_BG_Pos) |
                                (1  << AUDCODEC_LP_PLL_CFG5_EN_CLK_CHOP_BG_Pos) |
                                (2  << AUDCODEC_LP_PLL_CFG5_DIVB_CLK_CHOP_REFGEN_Pos) |
                                (10 << AUDCODEC_LP_PLL_CFG5_DIVA_CLK_CHOP_REFGEN_Pos) |
                                (1  << AUDCODEC_LP_PLL_CFG5_EN_CLK_CHOP_REFGEN_Pos) |
                                (2  << AUDCODEC_LP_PLL_CFG5_DIVB_CLK_CHOP_DAC2_Pos) |
                                (2  << AUDCODEC_LP_PLL_CFG5_DIVA_CLK_CHOP_DAC2_Pos) |
                                (1  << AUDCODEC_LP_PLL_CFG5_EN_CLK_CHOP_DAC2_Pos) |
                                (10 << AUDCODEC_LP_PLL_CFG5_DIVA_CLK_DAC2_Pos) |
                                (1  << AUDCODEC_LP_PLL_CFG5_EN_CLK_DAC2_Pos);
    // dac1 and dac2 power
    hwp_audcodec_lp->DAC1_CFG |= AUDCODEC_LP_DAC1_CFG_LP_MODE;
    hwp_audcodec_lp->DAC2_CFG |= AUDCODEC_LP_DAC2_CFG_LP_MODE;
    hwp_audcodec_lp->DAC1_CFG &= ~AUDCODEC_LP_DAC1_CFG_EN_OS_DAC;
    hwp_audcodec_lp->DAC2_CFG &= ~AUDCODEC_LP_DAC2_CFG_EN_OS_DAC;
    hwp_audcodec_lp->DAC1_CFG |= AUDCODEC_LP_DAC1_CFG_EN_VCM;
#ifdef BSP_ENABLE_DAC2
    hwp_audcodec_lp->DAC2_CFG |= AUDCODEC_LP_DAC2_CFG_EN_VCM;
#else
    hwp_audcodec_lp->DAC2_CFG &= ~AUDCODEC_LP_DAC2_CFG_EN_VCM;
#endif
    // wait 5us
    //wait(250);
    HAL_Delay_us(5);
    hwp_audcodec_lp->DAC1_CFG |= AUDCODEC_LP_DAC1_CFG_EN_AMP;
#ifdef BSP_ENABLE_DAC2
    hwp_audcodec_lp->DAC2_CFG |= AUDCODEC_LP_DAC2_CFG_EN_AMP;
#else
    hwp_audcodec_lp->DAC2_CFG &= ~AUDCODEC_LP_DAC2_CFG_EN_AMP;
#endif
    // wait 1us
    //wait(50);
    HAL_Delay_us(1);
    hwp_audcodec_lp->DAC1_CFG |= AUDCODEC_LP_DAC1_CFG_EN_OS_DAC;
    hwp_audcodec_lp->DAC2_CFG |= AUDCODEC_LP_DAC2_CFG_EN_OS_DAC;
    // wait 10us
    //wait(500);
    HAL_Delay_us(10);
    hwp_audcodec_lp->DAC1_CFG |= AUDCODEC_LP_DAC1_CFG_EN_DAC;
#ifdef BSP_ENABLE_DAC2
    hwp_audcodec_lp->DAC2_CFG |= AUDCODEC_LP_DAC2_CFG_EN_DAC;
#else
    hwp_audcodec_lp->DAC2_CFG &= ~AUDCODEC_LP_DAC2_CFG_EN_DAC;
#endif
    // wait 10us
    //wait(500);
    HAL_Delay_us(10);
    hwp_audcodec_lp->DAC1_CFG &= ~AUDCODEC_LP_DAC1_CFG_SR;
    hwp_audcodec_lp->DAC2_CFG &= ~AUDCODEC_LP_DAC2_CFG_SR;


//-----------------step 6------------------//
// start ch0 & ch1 test
// setup CODEC
    hwp_audcodec_hp->CFG |= (1 << AUDCODEC_HP_CFG_EN_DLY_SEL_Pos) |
                            (1 << AUDCODEC_HP_CFG_DAC_1K_MODE_Pos);
    hwp_audcodec_hp->DAC_CFG = (2  << AUDCODEC_HP_DAC_CFG_OSR_SEL_Pos) |
                               (0  << AUDCODEC_HP_DAC_CFG_OP_MODE_Pos) |
                               (0  << AUDCODEC_HP_DAC_CFG_PATH_RESET_Pos) |
                               (0  << AUDCODEC_HP_DAC_CFG_CLK_SRC_SEL_Pos) |  // select xtal
                               (20 << AUDCODEC_HP_DAC_CFG_CLK_DIV_Pos);

    hwp_audcodec_hp->DAC_CH0_CFG_EXT = (0  << AUDCODEC_HP_DAC_CH0_CFG_EXT_RAMP_EN_Pos) |
                                       (1  << AUDCODEC_HP_DAC_CH0_CFG_EXT_RAMP_MODE_Pos) |
                                       (0  << AUDCODEC_HP_DAC_CH0_CFG_EXT_ZERO_ADJUST_EN_Pos) |
                                       (20 << AUDCODEC_HP_DAC_CH0_CFG_EXT_RAMP_INTERVAL_Pos) |
                                       (0  << AUDCODEC_HP_DAC_CH0_CFG_EXT_RAMP_STAT_Pos);

    hwp_audcodec_hp->DAC_CH0_CFG = (1   << AUDCODEC_HP_DAC_CH0_CFG_ENABLE_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH0_CFG_DOUT_MUTE_Pos) |
                                   (2   << AUDCODEC_HP_DAC_CH0_CFG_DEM_MODE_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH0_CFG_DMA_EN_Pos) |
                                   (6   << AUDCODEC_HP_DAC_CH0_CFG_ROUGH_VOL_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH0_CFG_FINE_VOL_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH0_CFG_DATA_FORMAT_Pos) |
                                   (256 << AUDCODEC_HP_DAC_CH0_CFG_SINC_GAIN_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH0_CFG_DITHER_GAIN_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH0_CFG_DITHER_EN_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH0_CFG_CLK_ANA_POL_Pos);

    hwp_audcodec_hp->DAC_CH1_CFG_EXT = (0 << AUDCODEC_HP_DAC_CH1_CFG_EXT_RAMP_EN_Pos) |
                                       (1 << AUDCODEC_HP_DAC_CH1_CFG_EXT_RAMP_MODE_Pos) |
                                       (1 << AUDCODEC_HP_DAC_CH1_CFG_EXT_ZERO_ADJUST_EN_Pos) |
                                       (0 << AUDCODEC_HP_DAC_CH1_CFG_EXT_RAMP_INTERVAL_Pos) |
                                       (0 << AUDCODEC_HP_DAC_CH1_CFG_EXT_RAMP_STAT_Pos);

    hwp_audcodec_hp->DAC_CH1_CFG = (1   << AUDCODEC_HP_DAC_CH1_CFG_ENABLE_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH1_CFG_DOUT_MUTE_Pos) |
                                   (2   << AUDCODEC_HP_DAC_CH1_CFG_DEM_MODE_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH1_CFG_DMA_EN_Pos) |
                                   (6   << AUDCODEC_HP_DAC_CH1_CFG_ROUGH_VOL_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH1_CFG_FINE_VOL_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH1_CFG_DATA_FORMAT_Pos) |
                                   (256 << AUDCODEC_HP_DAC_CH1_CFG_SINC_GAIN_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH1_CFG_DITHER_GAIN_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH1_CFG_DITHER_EN_Pos) |
                                   (0   << AUDCODEC_HP_DAC_CH1_CFG_CLK_ANA_POL_Pos);

    hwp_audcodec_hp->CFG |= AUDCODEC_HP_CFG_ENABLE;

    return test_result;
}

MSH_CMD_EXPORT(codec_hp_sin1k_test, audio_codec);

int codec_dac_set_vol(int argc, char *argv[])
{
    rt_thread_t thread;
    int volume;

    if (argc != 2)
    {
        rt_kprintf("arg para num error\n");
        return -1;
    }
    volume = strtol(argv[1], NULL, 10);
    rt_kprintf("set volume=%d\n", volume);

    HAL_AUDCODEC_Config_DACPath_Volume(&(h_aud_codec.audcodec), 0, volume);
    HAL_AUDCODEC_Config_DACPath_Volume(&(h_aud_codec.audcodec), 1, volume);


    return 0;
}

MSH_CMD_EXPORT(codec_dac_set_vol, set codec dac volume);

int update_pll(int argc, char *argv[])
{
    int freq;

    if (argc != 2)
    {
        rt_kprintf("arg para num error\n");
        return -1;
    }
    freq = strtol(argv[1], NULL, 10);
    rt_kprintf("freq delta=%d\n", freq);

    update_pll_after_enable(freq);


    return 0;
}

MSH_CMD_EXPORT(update_pll, update PLL freq);


#endif  //BSP_ENABLE_AUD_PRC
