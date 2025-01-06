/**
  ******************************************************************************
  * @file   drv_io.c
  * @author Sifli software development team
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

#include "board.h"
#include "string.h"
#include "rtconfig.h"
#ifdef BSP_USING_RTTHREAD
    #include "drv_common.h"
    #include "drv_psram.h"
    #include "drv_flash.h"
    //#define DEBUG
    #include "log.h"
#endif
#include "drv_io.h"
#include "bf0_sys_cfg.h"

#ifdef HAL_DSI_MODULE_ENABLED
    #include "bf0_hal_dsi.h"
#endif /* HAL_DSI_MODULE_ENABLED */



#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

#ifndef LXT_LP_CYCLE
    #define LXT_LP_CYCLE 200
#endif


#define LCD_BOARD_POWER_PIN     (38)      // GPIO_B38
#define QSPI2_CLK_PIN           (60)      // GPIO_A60
#define LCD_RESET_PIN           (17)      // GPIO_B17

#define TP_RESET_PIN            (0)       // GPIO_A00
#define TP_EN_PIN               (3)       // GPIO_A06

#define QSPI3_POWER_PIN         (41)      // GPIO_A60
#define QSPI_POWER_PIN          (58)      // GPIO_A58

//#define ENABLE_EFUSE_SYSCONF

static uint16_t flash1_div = 1;
static uint16_t flash2_div = 1;
static uint16_t flash4_div = 1;
static uint16_t flash3_div = 4;
static uint32_t otp_flash_addr = SYSCFG_FACTORY_ADDRESS;


uint16_t BSP_GetFlash1DIV(void)
{
    return flash1_div;
}

uint16_t BSP_GetFlash2DIV(void)
{
    return flash2_div;
}

void BSP_SetFlash1DIV(uint16_t div)
{
    flash1_div = div;
}

void BSP_SetFlash2DIV(uint16_t div)
{
    flash2_div = div;
}

uint16_t BSP_GetFlash4DIV(void)
{
    return flash4_div;
}

void BSP_SetFlash4DIV(uint16_t div)
{
    flash4_div = div;
}

uint16_t BSP_GetFlash3DIV(void)
{
    return flash3_div;
}

void BSP_SetFlash3DIV(uint16_t div)
{
    flash3_div = div;
}

uint32_t BSP_GetOtpBase(void)
{
    return otp_flash_addr;
}


#ifdef SOC_BF0_HCPU
#define HXT_DELAY_EXP_VAL 1000
static void LRC_init(void)
{
    HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
    uint32_t ref_cnt = HAL_RC_CAL_get_reference_cycle_on_48M();
    uint32_t cycle_t = (uint32_t)ref_cnt / (48 * LXT_LP_CYCLE);

    HAL_PMU_SET_HXT3_RDY_DELAY((HXT_DELAY_EXP_VAL / cycle_t + 1));
}

#endif

void HAL_PreInit(void)
{
#ifdef SOC_BF0_HCPU
    // To avoid somebody cancel request.
    HAL_HPAON_EnableXT48();

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_UART1, RCC_CLK_USART_HXT48);

    if (PM_STANDBY_BOOT != SystemPowerOnModeGet()) // Except Standby mode, all other mode need to re-init
    {
        // Halt LCPU first to avoid LCPU in running state
        HAL_HPAON_WakeCore(CORE_ID_LCPU);
        HAL_RCC_Reset_and_Halt_LCPU(1);
#ifdef BSP_USING_SPI_FLASH
        {
#ifndef USE_ATE_MODE
            // if not define EFUSE CONFIG or efuse configure read fail, check flash otp configure
            BSP_System_Config();
#endif
        }
#endif
        HAL_PMU_EnableDLL(1);
        // disable it when ATE?
        HAL_PMU_SWITCH_VRET_LOWER();

        HAL_HPAON_StartGTimer();
#ifdef LXT_DISABLE
        HAL_PMU_LpCLockSelect(PMU_LPCLK_RC10);
        MODIFY_REG(hwp_pmuc->LRC_CR, PMUC_LRC_CR_CMPBM1_Msk, 3 << PMUC_LRC_CR_CMPBM1_Pos);
        MODIFY_REG(hwp_pmuc->LRC_CR, PMUC_LRC_CR_CMPBM2_Msk, 1 << PMUC_LRC_CR_CMPBM2_Pos);
#else
        HAL_PMU_LpCLockSelect(PMU_LPCLK_XT32);
#endif
        //HAL_PMU_SET_HPSYS_LDO_VREF(0xB);

        HAL_PMU_SET_HPSYS_LDO_VREF2(0);

        /* Configure in LCPU. */
        HAL_LPAON_EnableXT48();

        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);

        HAL_PMU_EnableBuck2();
        HAL_PMU_SELECT_LPSYS_PWR(PMU_LPSYS_PSW_BUCK2);
        HAL_PMU_DISABLE_LPSYS_LDO();
#if 1
        HAL_RCC_LCPU_SetDiv(1, 1, 3);

        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_UART3, RCC_CLK_USART_HXT48);
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_UART5, RCC_CLK_USART_HXT48);
        HAL_RCC_LCPU_enable2(LPSYS_RCC_ENR2_QSPI4, 1);
#endif
        if (HAL_PMU_LXT_DISABLED())
        {
            LRC_init();
        }
    }

    HAL_RCC_HCPU_SetDiv(1, 2, 5);

    flash1_div = 1;
    flash2_div = 4;
    //HAL_QSPI_ENABLE_WDT();

    HAL_RCC_HCPU_EnableDLL2(96000000);

    // delayed version of Rx sample en set to 0x6
    // delayed version of Rx clock set to 0x6
    //hwp_qspi1->MISCR = 0x36;
    HAL_QSPI_SET_RXDELAY(0, 6, 6);
    HAL_QSPI_SET_RXDELAY(1, 6, 6);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_SRC_DLL2);

    //HAL_RCC_HCPU_EnableDLL3(144000000);
    //HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH3, RCC_CLK_SRC_DLL3);

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    HAL_RCC_HCPU_EnableDLL1(240000000);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_MOD_SYS);

    // Reset sysclk used by HAL_Delay_us
    HAL_Delay_us(0);

    /* Init the low level hardware */
    HAL_MspInit();

#if defined (BSP_USING_PSRAM)
#if defined (BSP_USING_EXT_PSRAM)
    HAL_RCC_HCPU_EnableDLL3(192000000);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM, RCC_CLK_SRC_DLL3);
#else
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM, RCC_CLK_PSRAM_SYSCLK);
#endif  //BSP_USING_EXT_PSRAM
    //TODO: enable it when PSRAM is available
    rt_psram_init();
#endif




#elif defined(SOC_BF0_LCPU)
    HAL_LPAON_EnableXT48();

    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_UART3, RCC_CLK_USART_HXT48);
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_UART5, RCC_CLK_USART_HXT48);

    HAL_RCC_LCPU_SetDiv(1, 1, 3);
    HAL_RCC_LCPU_enable2(LPSYS_RCC_ENR2_QSPI4, 1);

    if (HAL_PMU_LXT_DISABLED() && PM_STANDBY_BOOT != SystemPowerOnModeGet())
    {
        HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
    }

    HAL_MspInit();

#ifdef BSP_USING_PSRAM
    HAL_LPAON_ENABLE_PAD();
    rt_psram_init();
#endif /* BSP_USING_PSRAM */
#endif
}



void BSP_GPIO_Set(int pin, int val, int is_porta)
{
    GPIO_TypeDef *gpio = (is_porta) ? hwp_gpio1 : hwp_gpio2;
    GPIO_InitTypeDef GPIO_InitStruct;

    // set sensor pin to output mode
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);

    // set sensor pin to high == power on sensor board
    HAL_GPIO_WritePin(gpio, pin, (GPIO_PinState)val);
}

void BSP_LCD_PowerUp(void)
{
    BSP_GPIO_Set(LCD_BOARD_POWER_PIN, 1, 0);
}

void BSP_LCD_PowerDown(void)
{
    BSP_GPIO_Set(LCD_BOARD_POWER_PIN, 0, 0);
    BSP_LCD_Reset(0);
}


void BSP_LCD_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 0);
}

void BSP_TP_PowerUp(void)
{
    BSP_GPIO_Set(TP_EN_PIN, 1, 1);  //TP_EN
#if 0
    HAL_PIN_SetMode(PAD_PA00, 1, PIN_DIGITAL_OUTPUT_NORMAL);
    HAL_PIN_Set(PAD_PA00, GPIO_A0, PIN_NOPULL, 1);
    BSP_GPIO_Set(TP_RESET_PIN, 1, 1);  //TP_RST
#else
    BSP_GPIO_Set(TP_RESET_PIN, 1, 1);  //TP_RST
#endif

}

void BSP_TP_PowerDown(void)
{
    //BSP_GPIO_Set(TP_RESET_PIN, 0, 1);
#if 0
    HAL_GPIO_DeInit(hwp_gpio1, TP_RESET_PIN);
    HAL_PIN_Set(PAD_PA00, GPIO_A0, PIN_NOPULL, 1);
    HAL_PIN_SetMode(PAD_PA00, 1, PIN_ANALOG_INPUT);
#else
    BSP_GPIO_Set(TP_RESET_PIN, 0, 1);
#endif
    BSP_GPIO_Set(TP_EN_PIN, 0, 1);
}

void BSP_TP_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(TP_RESET_PIN, high1_low0, 1);
}

void BSP_QSPI_PowerUp(void)
{
    BSP_GPIO_Set(QSPI_POWER_PIN, 1, 1);
//    HAL_PIN_Set(PAD_PA60, QSPI2_CLK, PIN_NOPULL, 1);

    HAL_GPIO_DeInit(hwp_gpio1, 61);
    HAL_PIN_Set(PAD_PA61, QSPI2_CS, PIN_NOPULL, 1);
}

void BSP_QSPI_PowerDown(void)
{
//    HAL_PIN_Set(PAD_PA60, GPIO_A60, PIN_NOPULL, 1);
//    BSP_GPIO_Set(QSPI2_CLK_PIN, 0, 1);
//    BSP_GPIO_Set(QSPI_POWER_PIN, 0, 1);
#ifndef BSP_USING_EXT_PSRAM
    HAL_PIN_Set(PAD_PA37, GPIO_A37, PIN_NOPULL, 1);
    BSP_GPIO_Set(37, 1, 1);
#endif
    HAL_PIN_Set(PAD_PA61, GPIO_A61, PIN_NOPULL, 1);
    BSP_GPIO_Set(61, 1, 1);
}



void BSP_Power_Up(bool is_deep_sleep)
{
    //TODO: need change
#ifdef SOC_BF0_HCPU
    if (is_deep_sleep)
    {
#if defined (BSP_USING_PSRAM)
        //rt_psram_exit_low_power("psram0");
#endif
    }
    BSP_QSPI_PowerUp();

    HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_PULLUP, 1);

#elif defined(SOC_BF0_LCPU)
    {
        //BSP_LCD_PowerUp();
    }
#endif

}

extern void BSP_PIN_Init(void);
void BSP_IO_Init(void)
{
    BSP_PIN_Init();
    BSP_Power_Up(true);
}


//#define LP_DEBUG
#ifdef LP_DEBUG
#define _WWORD(reg,value) \
{ \
    volatile uint32_t * p_reg=(uint32_t *) reg; \
    *p_reg=value; \
}

void debug_lp(void)
{
    _WWORD(0x4000e018, 0x200);   //PA00
    _WWORD(0x4000e01c, 0x200);   //PA01
    _WWORD(0x4000e020, 0x200);   //PA02
    _WWORD(0x4000e024, 0x200);   //PA03

    _WWORD(0x4000e028, 0x200);   //PA04
    _WWORD(0x4000e02c, 0x200);   //PA05
    _WWORD(0x4000e030, 0x200);   //PA06
    _WWORD(0x4000e034, 0x200);   //PA07
    _WWORD(0x4000e038, 0x200);   //PA08
    _WWORD(0x4000e03c, 0x200);   //PA09

    _WWORD(0x4000e040, 0x200);   //PA10
    _WWORD(0x4000e044, 0x200);   //PA11
    _WWORD(0x4000e048, 0x200);   //PA12
    _WWORD(0x4000e04c, 0x200);   //PA13
    _WWORD(0x4000e050, 0x200);   //PA14
    _WWORD(0x4000e054, 0x200);   //PA15
    _WWORD(0x4000e058, 0x200);   //PA16
    _WWORD(0x4000e05c, 0x200);   //PA17
    _WWORD(0x4000e060, 0x200);   //PA18
    _WWORD(0x4000e064, 0x200);   //PA19
    _WWORD(0x4000e068, 0x200);   //PA20
    _WWORD(0x4000e06c, 0x200);   //PA21
    _WWORD(0x4000e070, 0x200);   //PA22
    _WWORD(0x4000e074, 0x200);   //PA23


    //_WWORD(0x4000e078, 0x200);   //PA24
    //_WWORD(0x4000e07c, 0x200);   //PA25
    _WWORD(0x4000e080, 0x200);   //PA26
    _WWORD(0x4000e084, 0x200);   //PA27
    _WWORD(0x4000e088, 0x200);   //PA28
    _WWORD(0x4000e08c, 0x200);   //PA29
    _WWORD(0x4000e090, 0x200);   //PA30
    _WWORD(0x4000e094, 0x200);   //PA31
    _WWORD(0x4000e098, 0x200);   //PA32
    _WWORD(0x4000e09c, 0x200);   //PA33
    //_WWORD(0x4000e0a0, 0x200);   //PA34
    //_WWORD(0x4000e0a4, 0x200);   //PA35
    _WWORD(0x4000e0a8, 0x200);   //PA36

    _WWORD(0x4000e0ac, 0x200);   //PA37
    _WWORD(0x4000e0b0, 0x200);   //PA38
    _WWORD(0x4000e0b4, 0x200);   //PA39

    _WWORD(0x4004e000, 0x200);   //PB00
    _WWORD(0x4004e004, 0x200);   //PB01
    _WWORD(0x4004e008, 0x200);   //PB02
    _WWORD(0x4004e00c, 0x200);   //PB03
    _WWORD(0x4004e010, 0x200);   //PB04
    _WWORD(0x4004e014, 0x200);   //PB05
    _WWORD(0x4004e018, 0x200);   //PB06
    _WWORD(0x4004e01c, 0x200);   //PB07
    _WWORD(0x4004e020, 0x200);   //PB08
    _WWORD(0x4004e024, 0x200);   //PB09
    _WWORD(0x4004e028, 0x200);   //PB10
    _WWORD(0x4004e02c, 0x200);   //PB11
    _WWORD(0x4004e030, 0x200);   //PB12
    _WWORD(0x4004e034, 0x200);   //PB13
    _WWORD(0x4004e038, 0x200);   //PB14
    _WWORD(0x4004e03c, 0x200);   //PB15
    _WWORD(0x4004e040, 0x200);   //PB16
    _WWORD(0x4004e044, 0x200);   //PB17
    _WWORD(0x4004e048, 0x200);   //PB18
    _WWORD(0x4004e04c, 0x200);   //PB19
    _WWORD(0x4004e050, 0x200);   //PB20

    _WWORD(0x4004e054, 0x200);   //PB21
    _WWORD(0x4004e058, 0x200);   //PB22
    _WWORD(0x4004e05c, 0x200);   //PB23
    _WWORD(0x4004e060, 0x200);   //PB24
    _WWORD(0x4004e064, 0x200);   //PB25
    _WWORD(0x4004e068, 0x200);   //PB26
    _WWORD(0x4004e06c, 0x200);   //PB27
    _WWORD(0x4004e070, 0x200);   //PB28
    _WWORD(0x4004e074, 0x200);   //PB29

    _WWORD(0x4004e078, 0x200);   //PB30
    _WWORD(0x4004e07c, 0x200);   //PB31
    _WWORD(0x4004e080, 0x200);   //PB32
    _WWORD(0x4004e084, 0x200);   //PB33
    _WWORD(0x4004e088, 0x200);   //PB34
    _WWORD(0x4004e08c, 0x200);   //PB35


    _WWORD(0x4001e010, 0xf);    //PA DOER0
    _WWORD(0x4001e008, 0x0);    //PA DOR0
    _WWORD(0x4001e014, 0x80);   //PA DOER1
    _WWORD(0x4001e00c, 0x80);   //PA DOR1
    _WWORD(0x40058010, 0x00400200);//PB DOER0
    _WWORD(0x40058008, 0x00);    //PB DOR0

    _WWORD(0x40030024, 0xC000000F);    //HPSYS ACR


    hwp_lpsys_rcc->RSTR = 0x00000001; //reset LCPU
    hwp_hpsys_rcc->CSR = 0;
    hwp_pmuc->HPSYS_SWR |= PMUC_HPSYS_SWR_NORET;
    hwp_lpsys_aon->ISSR = 0x00000200;
    hwp_lpsys_aon->WER = 0;//LPSYS_AON_WER_LPTIM2;
    hwp_lpsys_aon->SBCR = 0x7f; //no retention
    hwp_lpsys_aon->PMR = 0x80000002;
    __disable_irq();
    hwp_hpsys_aon->WER = 0;//HPSYS_AON_WER_LPTIM1;
    hwp_hpsys_aon->SBCR = 0x1ff; //no retention
    hwp_hpsys_aon->ISSR &= ~HPSYS_AON_ISSR_HP_ACTIVE;
    hwp_hpsys_aon->PMR = 0x00000003;
    __WFI();
    while (1);
}
#else
#define debug_lp()
#endif

void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)
{
    int i;
#ifdef SOC_BF0_HCPU
    if (coreid == CORE_ID_HCPU)
    {
        //BSP_TP_PowerDown();
        //BSP_LCD_PowerDown();
        //BSP_QSPI_PowerDown();

        HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_NOPULL, 1);

        if (is_deep_sleep)
        {
#if defined (BSP_USING_PSRAM)
#if defined (BSP_USING_EXT_PSRAM)
            HAL_RCC_HCPU_DisableDLL3();
#endif

#endif
        }

        debug_lp();
    }
    else
    {
    }
#else
    //BSP_LCD_PowerDown();
#endif
}


#ifdef HAL_PSRAM_MODULE_ENABLED

#if 0

void HAL_PSRAM_MspInit(PSRAM_HandleTypeDef *hpsram)
{

    uint32_t state;
    uint32_t rdata;

    // for fpga setting
    //set o_clk delay
    hpsram->Instance->CLK_CTRL = 0x0;
    //set o_dqs & i_dqs_delay
    hpsram->Instance->DQS_CTRL = 0x0;

    // read phy state
    do
    {
        state = hpsram->Instance->PSRAM_FREE &  PSRAMC_PSRAM_FREE_PSRAM_FREE_Msk;
    }
    while (!state);

    //write to globle reset psram
    hpsram->Instance->POWER_UP |= PSRAMC_POWER_UP_HW_POWER_PULSE;
    // read init state
    do
    {
        state = hpsram->Instance->POWER_UP & PSRAMC_POWER_UP_INIT_DONE_STATE_Msk;
    }
    while (!state);

    if (0 != (hpsram->Instance->PSRAM_CFG & PSRAMC_PSRAM_CFG_XCCELA_PSRAM_EN_Msk))
    {
        /* set write latency to 0 to ensure MR0 and MR8 is set correctly */
        hpsram->Instance->CTRL_TIME &= ~PSRAMC_CTRL_TIME_WL_Msk;

        //change xccela psram wrap size
        rdata = hpsram->Instance->MR0 ;
        //rdata[1:0] = 'h0;
        //rdata[17:16] = 'h0;
        rdata &= 0xFFFCFFFC;
        hpsram->Instance->MR0 = rdata;
        rdata = hpsram->Instance->MR8 ;
        //rdata[2:0]   = 'h3;
        //rdata[18:16] = 'h3;
        rdata &= 0xFFF8FFF8;
        rdata |= 0x00030003;
        hpsram->Instance->MR8 = rdata;

        //write wl code for xccela psram
        hpsram->Instance->CTRL_TIME |= (7 << PSRAMC_CTRL_TIME_WL_Pos) ;
    }
}
#endif
#endif /* HAL_PSRAM_MODULE_ENABLED */




/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
