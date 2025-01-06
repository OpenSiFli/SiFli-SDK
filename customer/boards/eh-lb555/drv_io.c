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

#ifdef PMIC_CTRL_ENABLE
    #include "pmic_controller.h"
#endif /* PMIC_CTRL_ENABLE */


#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

#ifndef LXT_LP_CYCLE
    #define LXT_LP_CYCLE 200
#endif

#ifndef RT_ASSERT
    #define RT_ASSERT(expr)
#endif

#define LCD_BOARD_POWER_PIN     (11)      // GPIO_B11
//#define QSPI2_CLK_PIN           (60)      // GPIO_A60
#define LCD_RESET_PIN           (17)      // GPIO_B17

#define TP_RESET_PIN            (53)       // GPIO_A53
//#define TP_EN_PIN               (6)       // GPIO_A06

//#define QSPI3_POWER_PIN         (41)      // GPIO_A60
#define QSPI_POWER_PIN          (58)      // GPIO_A58

//#define ENABLE_EFUSE_SYSCONF

//#define CHECK_ADC_BEFORE_START      (1)

static uint16_t flash1_div = 1;
static uint16_t flash2_div = 1;
static uint16_t flash3_div = 6;
static uint16_t flash4_div = 4;
static uint32_t otp_flash_addr = SYSCFG_FACTORY_ADDRESS;

int rt_psram_init(void);


#if defined(TOUCH_WAKEUP_SUPPORT)
static uint8_t g_touch_wakeup_check_eanble = 0;

uint8_t drv_tp_get_wakeup_check_enable(void)
{
    return g_touch_wakeup_check_eanble;
}
void drv_tp_set_wakeup_check_enable(uint8_t is_enable)
{
    g_touch_wakeup_check_eanble = is_enable;
}
#endif

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

uint16_t BSP_GetFlash3DIV(void)
{
    return flash3_div;
}

uint16_t BSP_GetFlash4DIV(void)
{
    return flash4_div;
}

void BSP_SetFlash3DIV(uint16_t div)
{
    flash3_div = div;
}

void BSP_SetFlash4DIV(uint16_t div)
{
    flash4_div = div;
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

#ifdef CHECK_ADC_BEFORE_START
int boot_adc_calibration(uint32_t value1, uint32_t value2,
                         uint32_t vol1, uint32_t vol2, float *offset, float *ratio)
{
    float gap1, gap2;

    if (offset == NULL || ratio == NULL)
        return 0;

    if (value1 == 0 || value2 == 0)
        return 0;

    gap1 = value1 > value2 ? value1 - value2 : value2 - value1; // register value gap
    gap2 = vol1 > vol2 ? vol1 - vol2 : vol2 - vol1; // voltage gap -- mv

    if (gap1 != 0)
    {
        *ratio = gap2 * 1000 / gap1; // gap2 * 10 for 0.1mv, gap2 * 100 for 0.01mv
    }
    else //
        return 0;

    *offset = value1 - vol1 * 1000 / *ratio;

    return 1;
}

static int get_boot_voltage(float offset, float ratio)
{
    ADC_HandleTypeDef hadc;
    ADC_ChannelConfTypeDef ADC_ChanConf;
    uint32_t lslot = 0;
    HAL_StatusTypeDef ret = HAL_OK;
    uint32_t vol_offset = (uint32_t)offset; //199;
    uint32_t vol_ratio = (uint32_t)ratio; //3930/3; // TODO: 3930 is offset for x3 mode , need /3 for x1 mode

    // TODO : set adc pinmux with correct pin
    HAL_PIN_Set_Analog(PAD_PB10, 0);       // Battery Voltage ADC

    // TODO: add 300 ms delay to make sure voltage stable
    HAL_Delay(300);

    // TODO, default use slot 1
    lslot = 1;  // set slot to test

    // initial adc handle
    hadc.Instance = hwp_gpadc1;
    hadc.Init.clk_div = 31;
    hadc.Init.adc_se = 1;   // single channel
    hadc.Init.adc_force_on = 0;
    hadc.Init.atten3 = 0;
    hadc.Init.dma_en = 0;   // no dma
    hadc.Init.en_slot = 0;  // default slot, update by enable and configure
    hadc.Init.op_mode = 0;  // single mode, not continous
    HAL_ADC_Init(&hadc);

    // enable slot
    HAL_ADC_EnableSlot(&hadc, lslot, 1);

    // Channel to select register, pchnl_sel to choose which pin used, here use the same number
    rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));
    ADC_ChanConf.Channel = lslot;
    ADC_ChanConf.pchnl_sel = lslot;
    ADC_ChanConf.slot_en = 1;
    ADC_ChanConf.acc_num = 0;
    HAL_ADC_ConfigChannel(&hadc, &ADC_ChanConf);

    /* start ADC */
    HAL_ADC_Start(&hadc);

// TODO, read counter, change it if need do average
#define TEST_ADC_CNT        (2)
    int i, res;
    uint32_t data[TEST_ADC_CNT];
    uint32_t value;

    for (i = 0; i < TEST_ADC_CNT; i++)
    {
        // triger sw start,
        __HAL_ADC_START_CONV(&hadc);

        /* Wait for the ADC to convert */
        res = HAL_ADC_PollForConversion(&hadc, 100);
        if (res != HAL_OK)
        {
            HAL_ADC_Stop(&hadc);
            return 0;
        }

        /* get ADC value */
        data[i] = (rt_uint32_t)HAL_ADC_GetValue(&hadc, lslot);
        // Add a delay between each read to make voltage stable
        HAL_Delay(5);
    }
    HAL_ADC_Stop(&hadc);

    // TODO: USE average or media ? use the latest one
    value = data[TEST_ADC_CNT - 1];
    return (value - vol_offset) * vol_ratio;
}
#endif //CHECK_ADC_BEFORE_START

#endif

void HAL_PreInit(void)
{
#ifdef SOC_BF0_HCPU
    // To avoid somebody cancel request.
    HAL_HPAON_EnableXT48();

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_UART1, RCC_CLK_USART_HXT48);

#ifdef  BSP_USING_UART4
    // Assume LCPU is not in low power mode.
    RT_ASSERT(HAL_HPAON_IS_LP_ACTIVE());
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_UART4, RCC_CLK_USART_HXT48);
#endif

    if (PM_STANDBY_BOOT != SystemPowerOnModeGet()) // Except Standby mode, all other mode need to re-init
    {
        // Halt LCPU first to avoid LCPU in running state
        HAL_HPAON_WakeCore(CORE_ID_LCPU);
        HAL_RCC_Reset_and_Halt_LCPU(1);
#ifdef BSP_USING_SPI_FLASH
        {
            // if not define EFUSE CONFIG or efuse configure read fail, check flash otp configure
            BSP_System_Config();
        }
#endif

#ifdef CHECK_ADC_BEFORE_START
        if (1)
        {
            HAL_LCPU_CONFIG_ADC_T cfg;
            float off, rat;
            uint32_t vol1, vol2;
            int adc_range;
            int res = 0;
            uint16_t len = (uint16_t)sizeof(HAL_LCPU_CONFIG_ADC_T);
            if (HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_ADC_CALIBRATION, (uint8_t *)&cfg, &len) == 0)
            {
                if (cfg.vol10 == 0 || cfg.vol25 == 0) // not valid paramter
                {
                    //LOG_I("Get ADC configure invalid : %d, %d\n", cfg.vol10, cfg.vol25);
                }
                else
                {
                    if ((cfg.vol10 & (1 << 15)) && (cfg.vol25 & (1 << 15))) // small range, use X1 mode
                    {
                        cfg.vol10 &= 0x7fff;
                        cfg.vol25 &= 0x7fff;
                        vol1 = 300;
                        vol2 = 800;
                        adc_range = 1;
                    }
                    else // big range , use X3 mode
                    {
                        vol1 = 1000;
                        vol2 = 2500;
                        adc_range = 0;
                    }
                    res = boot_adc_calibration(cfg.vol10, cfg.vol25, vol1, vol2, &off, &rat);
                    if (adc_range == 0) // big range ratio need change to small range ratio
                        rat /= 3;

                }
            }
            if (res == 0)   // get cal fail
            {
                off = 199;
                rat = 3930 / 3; // TODO: 3930 is offset for x3 mode , need /3 for x1 mode
            }
            int vol = get_boot_voltage(off, rat);   // based on 0.001 mv
            int mul = (1000 + 220) / 220;   // TODO: fix it as real board configure.
            int threshold = 3300; // TODO:  boot up voltage threshold
            while (vol * mul < threshold * 1000)
            {
                HAL_Delay(1000 * 60);   // 60 sec
                vol = get_boot_voltage(off, rat);
            }
        }
#endif //CHECK_ADC_BEFORE_START
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
#ifdef BSP_USING_SPI_CAMERA
    HAL_RCC_HCPU_SetDiv(1, 1, 5);
#else
    HAL_RCC_HCPU_SetDiv(1, 2, 5);
#endif
    flash1_div = 1;
    flash2_div = 4;
    flash3_div = 4;
    //HAL_QSPI_ENABLE_WDT();

    HAL_RCC_HCPU_EnableDLL2(96000000);

    // delayed version of Rx sample en set to 0x6
    // delayed version of Rx clock set to 0x6
    //hwp_qspi1->MISCR = 0x36;
    HAL_QSPI_SET_RXDELAY(0, 6, 6);
    HAL_QSPI_SET_RXDELAY(1, 6, 6);
    HAL_QSPI_SET_RXDELAY(2, 6, 6);
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
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_PSRAM, RCC_CLK_PSRAM_SYSCLK);
    bsp_psramc_init();
#endif


#ifdef PMIC_CTRL_ENABLE
    BSP_PMIC_Init();
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

/**
 * @brief LCD and TP use same power controller.

            Turn off power while both LCD and TP are closed,
            and on if anyone is opened.

 * @param on - turn on power
 * @param tp0_lcd1 - who invoke function, TP or LCD
 * @return
 */
static void BSP_LCD_TP_Power(uint32_t on, uint32_t tp0_lcd1)
{
    static uint8_t s_lcd_power = 0;
    static uint8_t s_tp_power  = 0;

#ifdef BSP_USING_RTTHREAD
    rt_base_t level;
    level = rt_hw_interrupt_disable();
#endif /* BSP_USING_RTTHREAD */
    if (0 == tp0_lcd1)
    {
        if (on)
        {
            HAL_ASSERT(s_tp_power < 255);
            s_tp_power++;
        }
        else
        {
            HAL_ASSERT(s_tp_power > 0);
            s_tp_power--;
        }
    }
    else
    {
        if (on)
        {
            HAL_ASSERT(s_lcd_power < 255);
            s_lcd_power++;
        }
        else
        {
            HAL_ASSERT(s_lcd_power > 0);
            s_lcd_power--;
        }
    }
#ifdef BSP_USING_RTTHREAD
    rt_hw_interrupt_enable(level);
#endif /* BSP_USING_RTTHREAD */


    if (on)
    {
#ifdef PMIC_CTRL_ENABLE
        pmic_device_control(PMIC_OUT_1V8_LVSW100_4, 1, 1); //LCD_1V8 power
        pmic_device_control(PMIC_OUT_LDO33_VOUT, 1, 1);    //LCD_3V3 power
#endif /* PMIC_CTRL_ENABLE */

    }
    else if ((0 == s_lcd_power) && (0 == s_tp_power)) //Turn off power if both LCD and TP are closed.
    {
#ifdef PMIC_CTRL_ENABLE
        pmic_device_control(PMIC_OUT_1V8_LVSW100_4, 0, 1); //LCD_1V8 power
        pmic_device_control(PMIC_OUT_LDO33_VOUT, 0, 1);    //LCD_3V3 power
#endif /* PMIC_CTRL_ENABLE */

    }
}


void BSP_LCD_PowerUp(void)
{
    BSP_LCD_TP_Power(1, 1);

    BSP_GPIO_Set(LCD_BOARD_POWER_PIN, 1, 0);
}

void BSP_LCD_PowerDown(void)
{
#if defined(TOUCH_WAKEUP_SUPPORT)
    if (drv_tp_get_wakeup_check_enable())
    {
        return;
    }
#endif

    BSP_LCD_TP_Power(0, 1);
    BSP_GPIO_Set(LCD_BOARD_POWER_PIN, 0, 0);

    BSP_LCD_Reset(0);
}


void BSP_LCD_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 0);
}

void BSP_TP_PowerUp(void)
{
    BSP_LCD_TP_Power(1, 0);

    //BSP_GPIO_Set(TP_EN_PIN, 1, 1);  //TP_EN
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
#if defined(TOUCH_WAKEUP_SUPPORT)
    if (drv_tp_get_wakeup_check_enable())
    {
        return;
    }
#endif

    BSP_LCD_TP_Power(0, 0);

    //BSP_GPIO_Set(TP_RESET_PIN, 0, 1);
#if 0
    HAL_GPIO_DeInit(hwp_gpio1, TP_RESET_PIN);
    HAL_PIN_Set(PAD_PA00, GPIO_A0, PIN_NOPULL, 1);
    HAL_PIN_SetMode(PAD_PA00, 1, PIN_ANALOG_INPUT);
#else
    BSP_GPIO_Set(TP_RESET_PIN, 0, 1);
#endif
    //BSP_GPIO_Set(TP_EN_PIN, 0, 1);
}

void BSP_TP_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(TP_RESET_PIN, high1_low0, 1);
}

void BSP_QSPI3_PowerUp(void)
{
    //BSP_GPIO_Set(QSPI3_POWER_PIN, 1, 1);
}

void BSP_QSPI3_PowerDown(void)
{
    //BSP_GPIO_Set(QSPI3_POWER_PIN, 0, 1);
}

void BSP_QSPI_PowerUp(void)
{
    BSP_GPIO_Set(QSPI_POWER_PIN, 1, 1);
//    HAL_PIN_Set(PAD_PA60, QSPI2_CLK, PIN_NOPULL, 1);
    HAL_GPIO_DeInit(hwp_gpio1, 49);
#ifdef BSP_ENABLE_QSPI3
    HAL_PIN_Set(PAD_PA49, QSPI3_DIO1, PIN_PULLDOWN, 1);
#else
    HAL_PIN_Set(PAD_PA49, SD2_DIO1, PIN_PULLUP, 1);
#endif
    HAL_GPIO_DeInit(hwp_gpio1, 61);
    HAL_PIN_Set(PAD_PA61, QSPI2_CS, PIN_NOPULL, 1);

    HAL_Delay_us(0);
#if 0 //def BSP_ENABLE_QSPI1
    FLASH_HandleTypeDef hflash1;
    hflash1.Instance = FLASH1;
    HAL_FLASH_RELEASE_DPD(&hflash1);
#endif

#ifdef BSP_ENABLE_QSPI2
#if (BSP_QSPI2_MODE == SPI_MODE_NOR)

    FLASH_HandleTypeDef hflash2;
    hflash2.Instance = FLASH2;
#ifdef BSP_QSPI2_DUAL_CHIP
    int dual = HAL_FLASH_GET_DUAL_MODE(&hflash2);
    HAL_FLASH_SET_DUAL_MODE(&hflash2, 1);
#endif //BSP_QSPI2_DUAL_CHIP

    HAL_FLASH_RELEASE_DPD(&hflash2);

#ifdef BSP_QSPI2_DUAL_CHIP
    HAL_FLASH_SET_DUAL_MODE(&hflash2, dual);
#endif  //BSP_QSPI2_DUAL_CHIP
#endif  //BSP_QSPI2_MODE
#endif  //BSP_ENABLE_QSPI2

    // wait tRES1
    HAL_Delay_us(8);
}

void BSP_QSPI_PowerDown(void)
{
#ifdef BSP_ENABLE_QSPI2
    FLASH_HandleTypeDef hflash2;
    hflash2.Instance = FLASH2;
    HAL_FLASH_DEEP_PWRDOWN(&hflash2);
#endif
#if 0 //def BSP_ENABLE_QSPI1
    FLASH_HandleTypeDef hflash1;
    hflash1.Instance = FLASH1;
    HAL_FLASH_DEEP_PWRDOWN(&hflash1);
#endif
    // wait tDP
    HAL_Delay_us(3);

//    HAL_PIN_Set(PAD_PA60, GPIO_A60, PIN_NOPULL, 1);
//    BSP_GPIO_Set(QSPI2_CLK_PIN, 0, 1);
//    BSP_GPIO_Set(QSPI_POWER_PIN, 0, 1);
#ifndef BSP_USING_EXT_PSRAM
    HAL_PIN_Set(PAD_PA37, GPIO_A37, PIN_NOPULL, 1);
    BSP_GPIO_Set(37, 1, 1);
#endif
    HAL_PIN_Set(PAD_PA49, GPIO_A49, PIN_NOPULL, 1);
    BSP_GPIO_Set(49, 1, 1);
    HAL_PIN_Set(PAD_PA61, GPIO_A61, PIN_NOPULL, 1);
    BSP_GPIO_Set(61, 1, 1);
}



void BSP_Power_Up(bool is_deep_sleep)
{
#ifdef SOC_BF0_HCPU
    if (is_deep_sleep)
    {
#if defined (BSP_USING_PSRAM)
        // rt_psram_exit_low_power("psram0");
#endif
    }
    BSP_QSPI_PowerUp();
    BSP_QSPI3_PowerUp();
#if 0
    HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_PULLUP, 1);
#endif

#elif defined(SOC_BF0_LCPU)
    {
        ;
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
        BSP_QSPI_PowerDown();

        HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_NOPULL, 1);

        if (is_deep_sleep)
        {
#if defined (BSP_USING_PSRAM)
            // rt_psram_enter_low_power("psram0");
#endif
        }

        debug_lp();
    }
    else
    {
    }
#else
    {
        ;
    }
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
