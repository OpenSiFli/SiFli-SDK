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

#include "bsp_board.h"
#include "drv_io.h"

#ifdef PMIC_CTRL_ENABLE
    #include "pmic_controller.h"
#endif

#define LCD_BACKLIGHT_POWER     (70)      // GPIO_A70
#define LCD_BOARD_POWER_PIN     (55)      // GPIO_A55
#define LCD_RESET_PIN           (78)      // GPIO_A78
#define TP_RESET_PIN            (47)      // GPIO_A47
#define QSPI_POWER_PIN          (58)      // GPIO_A58
#define QSPI2_CLK_PIN           (60)      // GPIO_A60
#define MOTOR_POWER_PIN         (44)      // GPIO_A44
#define MOTOR_PWM_PIN           (45)      // GPIO_A45

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

#ifndef LXT_LP_CYCLE
    #define LXT_LP_CYCLE 200
#endif

//#define ENABLE_EFUSE_SYSCONF

#ifdef SOC_BF0_HCPU
    #define IS_HCPU     1
    #define PAD_BASE    PAD_PA00
    #define GPIO_BASE   GPIO_A0
    #define HWP         hwp_gpio1
#else
    #define IS_HCPU     0
    #define PAD_BASE    PAD_PB00
    #define GPIO_BASE   GPIO_B0
    #define HWP         hwp_gpio2
#endif

#define OUTPUT_0(pin_num) \
    do \
    { \
        HAL_PIN_Set(PAD_BASE + pin_num , GPIO_BASE + pin_num, PIN_NOPULL, IS_HCPU); \
        HAL_PIN_SetMode(PAD_BASE + pin_num, IS_HCPU, PIN_DIGITAL_O_NORMAL); \
        BSP_GPIO_Set(pin_num, 0, IS_HCPU); \
    } while(0)


#define INPUT_Z(pin_num) \
    do \
    { \
        HAL_GPIO_DeInit(HWP, pin_num); \
        HAL_PIN_Set(PAD_BASE + pin_num, GPIO_BASE + pin_num, PIN_NOPULL, IS_HCPU); \
        HAL_PIN_SetMode(PAD_BASE + pin_num, IS_HCPU, PIN_ANALOG_INPUT); \
        HAL_GPIO_DeInit(HWP, pin_num); \
    } while(0)


#define OUTPUT_1_PULLUP(pin_num) \
        do \
        { \
            HAL_PIN_Set(PAD_BASE + pin_num , GPIO_BASE + pin_num, PIN_PULLUP, IS_HCPU); \
            HAL_PIN_SetMode(PAD_BASE + pin_num, IS_HCPU, PIN_DIGITAL_O_PULLUP); \
            BSP_GPIO_Set(pin_num, 1, IS_HCPU); \
        } while(0)

static uint16_t flash1_div = 1;
static uint16_t flash2_div = 1;
static uint16_t flash3_div = 4;
static uint16_t flash4_div = 4;
static uint32_t otp_flash_addr = SYSCFG_FACTORY_ADDRESS;


#ifdef PMIC_CTRL_ENABLE
    static uint16_t g_lcd_tp_power = 0;
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

#define LDO_DELAY_EXP_VAL 300
#define HXT_DELAY_EXP_VAL 1000
static void LRC_init(void)
{
    HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
    uint32_t ref_cnt = HAL_RC_CAL_get_reference_cycle_on_48M();
    uint32_t cycle_t = (uint32_t)ref_cnt / (48 * LXT_LP_CYCLE);

    HAL_PMU_SET_LDO_RDY_DELAY((LDO_DELAY_EXP_VAL / cycle_t + 1));
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
            // if not define EFUSE CONFIG or efuse configure read fail, check flash otp configure
            BSP_System_Config();
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

#ifndef PM_USE_RC48
        /* Configure in LCPU. */
        HAL_LPAON_EnableXT48();

        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
#endif /* !PM_USE_RC48 */


        HAL_RCC_LCPU_SetDiv(1, 1, 3);

#ifndef PM_USE_RC48
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_UART3, RCC_CLK_USART_HXT48);
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_UART5, RCC_CLK_USART_HXT48);
#else
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_UART3, RCC_CLK_USART_HRC48);
        HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_UART5, RCC_CLK_USART_HRC48);
#endif /* !PM_USE_RC48 */


        if (HAL_PMU_LXT_DISABLED())
        {
            LRC_init();
        }
    }

    HAL_RCC_HCPU_SetDiv(1, 2, 5);

    flash1_div = 1;
    flash2_div = 4;
    //HAL_QSPI_ENABLE_WDT();

#if 1
    HAL_RCC_HCPU_EnableDLL2(96000000);

    // delayed version of Rx sample en set to 0x6
    // delayed version of Rx clock set to 0x6
    //hwp_qspi1->MISCR = 0x36;
    HAL_QSPI_SET_RXDELAY(0, 6, 6);
    HAL_QSPI_SET_RXDELAY(1, 6, 6);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_SRC_DLL2);
#endif

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




#elif defined(SOC_BF0_LCPU)

#ifndef PM_USE_RC48
    HAL_LPAON_EnableXT48();
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_UART3, RCC_CLK_USART_HXT48);
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_UART5, RCC_CLK_USART_HXT48);
#else
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HRC48);
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_UART3, RCC_CLK_USART_HRC48);
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_UART5, RCC_CLK_USART_HRC48);
    HAL_LPAON_DisableXT48();
#endif /* !PM_USE_RC48 */

    HAL_RCC_LCPU_SetDiv(1, 1, 3);

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


void BSP_LCD_PowerUp(void)
{
    /*Keep LCD_RESET output low during LCD analog poweron*/
    HAL_PIN_Set(PAD_PA20, GPIO_A20, PIN_PULLDOWN, 1); //LCDC1_SPI_CLK
    HAL_PIN_Set(PAD_PA31, GPIO_A31, PIN_PULLDOWN, 1); //LCDC1_SPI_CS
    HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_PULLDOWN, 1); //LCDC1_SPI_DIO0
    HAL_PIN_Set(PAD_PA36, GPIO_A36, PIN_PULLDOWN, 1); //LCDC1_SPI_DIO1
    HAL_PIN_Set(PAD_PA38, GPIO_A38, PIN_PULLDOWN, 1); //LCDC1_SPI_DIO2
    HAL_PIN_Set(PAD_PA42, GPIO_A42, PIN_PULLDOWN, 1); //LCDC1_SPI_DIO3
    HAL_PIN_Set(PAD_PA77, GPIO_A77, PIN_PULLDOWN, 1); //LCDC1_SPI_TE
    BSP_LCD_Reset(0);


    HAL_Delay(2); //Delay > 1ms
#ifdef PMIC_CTRL_ENABLE
    pmic_device_control(PMIC_OUT_1V8_LVSW100_4 | PMIC_OUT_LDO33_VOUT, 1, 1);
    HAL_Delay(2);
    rt_base_t level;
    level = rt_hw_interrupt_disable();
    g_lcd_tp_power++;
    rt_hw_interrupt_enable(level);
#else
    BSP_GPIO_Set(LCD_BOARD_POWER_PIN, 1, 1);
#endif
    HAL_Delay(2); //Delay > 1ms


    HAL_PIN_Set(PAD_PA20, LCDC1_SPI_CLK, PIN_NOPULL, 1);        // LCDC 1  QAD-SPI mode
    HAL_PIN_Set(PAD_PA31, LCDC1_SPI_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA34, LCDC1_SPI_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA36, LCDC1_SPI_DIO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, LCDC1_SPI_DIO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA42, LCDC1_SPI_DIO3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA77, LCDC1_SPI_TE, PIN_NOPULL, 1);
#ifndef BPS_V33
    HAL_PIN_Set_DS0(PAD_PA20, 1, 1);
    HAL_PIN_Set_DS1(PAD_PA20, 1, 1);
#endif
}

void BSP_LCD_PowerDown(void)
{
#ifdef PMIC_CTRL_ENABLE
    rt_base_t level;
    uint16_t  cnt;
    level = rt_hw_interrupt_disable();
    cnt = g_lcd_tp_power;
    if (g_lcd_tp_power > 0)
        g_lcd_tp_power--;
    rt_hw_interrupt_enable(level);
    if (1 == cnt)
        pmic_device_control(PMIC_OUT_1V8_LVSW100_4 | PMIC_OUT_LDO33_VOUT, 0, 1);
#else
    BSP_GPIO_Set(LCD_BOARD_POWER_PIN, 0, 1);
#endif
    HAL_PIN_Set(PAD_PA20, GPIO_A20, PIN_PULLDOWN, 1); //LCDC1_SPI_CLK
    HAL_PIN_Set(PAD_PA31, GPIO_A31, PIN_PULLDOWN, 1); //LCDC1_SPI_CS
    HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_PULLDOWN, 1); //LCDC1_SPI_DIO0
    HAL_PIN_Set(PAD_PA36, GPIO_A36, PIN_PULLDOWN, 1); //LCDC1_SPI_DIO1
    HAL_PIN_Set(PAD_PA38, GPIO_A38, PIN_PULLDOWN, 1); //LCDC1_SPI_DIO2
    HAL_PIN_Set(PAD_PA42, GPIO_A42, PIN_PULLDOWN, 1); //LCDC1_SPI_DIO3
    HAL_PIN_Set(PAD_PA77, GPIO_A77, PIN_PULLDOWN, 1); //LCDC1_SPI_TE
    HAL_PIN_Set(PAD_PA78, GPIO_A78, PIN_NOPULL, 1);   //LCDC1_SPI_RSTB
    BSP_LCD_Reset(0);
    OUTPUT_0(LCD_BACKLIGHT_POWER);
}


void BSP_LCD_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 1);
}

#ifdef SOC_BF0_HCPU
void BSP_TP_PowerUp(void)
{
#ifdef PMIC_CTRL_ENABLE
    rt_base_t level;
    level = rt_hw_interrupt_disable();
    g_lcd_tp_power++;
    rt_hw_interrupt_enable(level);
#endif
    HAL_PIN_SetMode(PAD_PA10, IS_HCPU, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_SetMode(PAD_PA14, IS_HCPU, PIN_DIGITAL_IO_NORMAL);
    HAL_PIN_SetMode(PAD_PA79, IS_HCPU, PIN_DIGITAL_IO_NORMAL);

    HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_PULLUP, 1);             // I2C1 scl, touch screen
    HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_PULLUP, 1);             // I2C1 sda, touch screen

    BSP_GPIO_Set(TP_RESET_PIN, 1, 1);
}

void BSP_TP_PowerDown(void)
{
#ifdef PMIC_CTRL_ENABLE
    rt_base_t level;
    uint16_t  cnt;
    level = rt_hw_interrupt_disable();
    cnt = g_lcd_tp_power;
    if (g_lcd_tp_power > 0)
        g_lcd_tp_power--;
    rt_hw_interrupt_enable(level);
    if (1 == cnt)
        pmic_device_control(PMIC_OUT_1V8_LVSW100_4 | PMIC_OUT_LDO33_VOUT, 0, 1);
#endif
    BSP_GPIO_Set(TP_RESET_PIN, 0, 1);
    //TP_INT
    INPUT_Z(79);
#if 1
    INPUT_Z(10);    // I2C1 scl, touch screen
    INPUT_Z(14);    // I2C1 sda, touch screen
#else
    HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_PULLDOWN, 1);             // I2C1 scl, touch screen
    HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_PULLDOWN, 1);             // I2C1 sda, touch screen
#endif

}
#endif

void BSP_TP_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(TP_RESET_PIN, high1_low0, 1);
}



void BSP_QSPI_PowerUp(void)
{
#ifndef PMIC_CTRL_ENABLE
    BSP_GPIO_Set(QSPI_POWER_PIN, 1, 1);
#endif
    HAL_PIN_Set(PAD_PA60, QSPI2_CLK, PIN_NOPULL, 1);
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

    OUTPUT_0(60); //qspi clk
    OUTPUT_0(61); //qspi cs

    //input pull down
    HAL_PIN_Set(PAD_PA63, QSPI2_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA65, QSPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA66, QSPI2_DIO2, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA68, QSPI2_DIO3, PIN_PULLDOWN, 1);
#ifdef PMIC_CTRL_ENABLE
    pmic_device_control(PMIC_OUT_1V8_LVSW100_1, 0, 1);
#else
#ifndef HDK_U4O5
    /* for U4O5, SIP PSRAM is also controlled by PA58, it cannot be powered down */
    OUTPUT_0(58); //QSPI2_EN norflash
#endif /* HDK_U4O5 */
#endif
}


void BSP_Power_Up(bool is_deep_sleep)
{
#ifdef SOC_BF0_HCPU
    if (is_deep_sleep)
    {
#if defined (BSP_USING_PSRAM)
        extern int rt_psram_exit_low_power(char *name);
        rt_psram_exit_low_power("psram0");
#endif
    }
#ifndef PMIC_CTRL_ENABLE
    BSP_GPIO_Set(MOTOR_POWER_PIN, 0, 1); //default, disable motor
#endif
    BSP_GPIO_Set(MOTOR_PWM_PIN, 0, 1); //default, disable motor
#ifdef PMIC_CTRL_ENABLE
    BSP_PMIC_Init();
    BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_1     // vdd_Sip or Ext.Flash
                     //| PMIC_OUT_1V8_LVSW100_2 //controlled by gsensor
                     //| PMIC_OUT_1V8_LVSW100_3 // hr
                     //| PMIC_OUT_1V8_LVSW100_4 //controlled by lcd
                     //| PMIC_OUT_1V8_LVSW100_5 // controlled by gps
                     //|PMIC_OUT_LDO28_VOUT  //controlled by hr
                     //|PMIC_OUT_LDO33_VOUT   //controlled by lcd
                     //|PMIC_OUT_LDO30_VOUT   //motor used only, controlled in vibrator.c
                     //| PMIC_OUT_VBAT_HVSW150_1  //controlled by bt
                     //| PMIC_OUT_VBAT_HVSW150_2  //controlled by hr
                     , 1, 1);
#endif

    BSP_QSPI_PowerUp();

    HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_PULLUP, 1);
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

#elif defined(SOC_BF0_LCPU)
    {
    }
#endif

}

extern void BSP_PIN_Init(void);
void BSP_IO_Init(void)
{
    BSP_PIN_Init();
    BSP_Power_Up(true);
}




static void BSP_OTHER_PowerDown(void)
{
#ifdef SOC_BF0_HCPU
    //PORT A

    INPUT_Z(1);  //BT, external BT power enable
    INPUT_Z(3);  //

    //same as pinmux, external has pullup, can't use pulldown
    //HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_NOPULL, 1);             // I2C1 scl, touch screen
    //HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_NOPULL, 1);             // I2C1 sda, touch screen
#ifndef PMIC_CTRL_ENABLE
    OUTPUT_0(44); // MOTOR_EN (hig-->open)
#endif
    OUTPUT_0(45); // MOTOR PWM

    //OUTPUT_0(49); // UART1 TX
    INPUT_Z(49);
    //HAL_PIN_Set(PAD_PA51, USART1_RXD, PIN_PULLDOWN, 1);        // UART1 RX
    INPUT_Z(51);
#ifdef PMIC_CTRL_ENABLE
    INPUT_Z(58);
#endif

    //INPUT_Z(80); //external BT interrupt input

#else //SOC_BF0_HCPU
    //PORT B
    //HR, should control by app, disable it temporary
    //INPUT_Z(3); //HR_EN, I2C4_EN
    //external has pullup, keep nopull as pinmux. or use INPUT_Z
    //HAL_PIN_Set(PAD_PB04, I2C4_SCL, PIN_NOPULL, 0);  // I2C4_SCL (Heart rate sensor)
    //HAL_PIN_Set(PAD_PB05, I2C4_SDA, PIN_NOPULL, 0);  // I2C4_SDA (Heart rate sensor)
    //HAL_PIN_Set(PAD_PB43, GPIO_B43, PIN_NOPULL, 0);  // I2C4_INT

    //INPUT_Z(8);    // CALIB

    //charging: no current consumption, aways enable charing
    //INPUT_Z(10); // BAT_ADC
    //INPUT_Z(47); // CHG_DET
    //INPUT_Z(24); // CHG_IND

    //Gesensor, should control by app, disable it temporary
    //INPUT_Z(1);  //SPI3_EN, controled by user app
    INPUT_Z(13); //SPI3_CLK
    INPUT_Z(23); //SPI3_CS
    INPUT_Z(16); //SPI3_DO
    INPUT_Z(19); //SPI3_DI
    INPUT_Z(44); //SPI3_INT


    INPUT_Z(25);      // encode key 1
    INPUT_Z(29);      // encode key 2

    //INPUT_Z(41);      // swclk
    //INPUT_Z(42);      // swdio

    //OUTPUT_0(45);   // if connected with uart2usb chip, can't output 0
    //HAL_PIN_Set(PAD_PB46, USART3_RXD, PIN_PULLDOWN, 0);        //UART3 RX
    INPUT_Z(45);    // UART3 TX
    INPUT_Z(46);


    //HAL_PIN_Set(PAD_PB48, GPIO_B48, PIN_NOPULL, 0);            //FPC_KEY, wakeup key, set by sensor_service.c
#endif //SOC_BF0_HCPU

}

void BSP_IO_Power_Down(int coreid, bool is_deep_sleep)
{
    int i;
#ifdef SOC_BF0_HCPU
    if (coreid == CORE_ID_HCPU)
    {

        BSP_QSPI_PowerDown();
        BSP_LCD_PowerDown();
        BSP_TP_PowerDown();
        BSP_OTHER_PowerDown();
        if (is_deep_sleep)
        {
#if defined (BSP_USING_PSRAM)
            extern int rt_psram_enter_low_power(char *name);
            rt_psram_enter_low_power("psram0");
#endif
        }

    }
#else
    BSP_OTHER_PowerDown();
#endif
}




/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
