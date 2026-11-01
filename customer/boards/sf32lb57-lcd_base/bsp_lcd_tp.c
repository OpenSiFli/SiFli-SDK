/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_board.h"
#include "aw9523.h"

#define EPD_GMODE_PIN           (54)         // GPIO_A54

#define EPD_VDD33_EN            (52)         // GPIO_A52
#define CTP_VDD33_EN            (53)         // GPIO_A53
#define DPI_VSYNC_PIN   (51)          // GPIO_A51

extern void BSP_GPIO_Set(int pin, int val, int is_porta);

void BSP_LCD_Reset(uint8_t high1_low0)
{
    aw9523_ensure_init();
    aw9523_set_pin(AW9523_PIN_LCD_RST, high1_low0);
}


void BSP_LCD_PowerDown(void)
{
    BSP_LCD_Reset(0);
}
static void QSPI_PIN_Set(int pad, pin_function func, int flags, int hcpu)
{
    HAL_PIN_Set(pad, func, flags, hcpu);
#ifdef BSP_LCDC_USING_DDR_QADSPI
    // Set the pin driver strength to 12mA
    HAL_PIN_SetDS(pad, PIN_DS_12MA);
#else
    // Set the pin driver strength to 8mA
    HAL_PIN_SetDS(pad, PIN_DS_8MA);
#endif /* BSP_LCDC_USING_DDR_QADSPI */
}

void BSP_LCD_PowerUp(void)
{
#if defined(BSP_USING_DCMI)
    /*
     * The panel and the DVP camera share PA32/PA33/PA43/PA44/PA45 on this
     * devkit, so only one of them can own those pads. Power the camera down
     * before the LCD takes them over.
     */
    BSP_CAMERA_PowerDown();
#endif /* BSP_USING_DCMI */

#ifdef BSP_LCDC_USING_DPI
    /* R channel */
    HAL_PIN_Set(PAD_PA24, LCDC1_DPI_R0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA25, LCDC1_DPI_R1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA26, LCDC1_DPI_R2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA27, LCDC1_DPI_R3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA28, LCDC1_DPI_R4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, LCDC1_DPI_R5, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA30, LCDC1_DPI_R6, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA31, LCDC1_DPI_R7, PIN_NOPULL, 1);

    /* G channel */
    HAL_PIN_Set(PAD_PA32, LCDC1_DPI_G0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA33, LCDC1_DPI_G1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA37, LCDC1_DPI_G2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, LCDC1_DPI_G3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA39, LCDC1_DPI_G4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA40, LCDC1_DPI_G5, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA41, LCDC1_DPI_G6, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA42, LCDC1_DPI_G7, PIN_NOPULL, 1);

    /* B channel */
    HAL_PIN_Set(PAD_PA43, LCDC1_DPI_B0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA44, LCDC1_DPI_B1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA45, LCDC1_DPI_B2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA52, LCDC1_DPI_B3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA53, LCDC1_DPI_B4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA54, LCDC1_DPI_B5, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA46, LCDC1_DPI_B6, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA47, LCDC1_DPI_B7, PIN_NOPULL, 1);

    /* Timing control */
    HAL_PIN_Set(PAD_PA48, LCDC1_DPI_CLK,   PIN_NOPULL, 1);
    // Set the clk pin driver strength to 8mA
    HAL_PIN_SetDS(PAD_PA48, PIN_DS_8MA);
    HAL_PIN_Set(PAD_PA49, LCDC1_DPI_HSYNC, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA51, LCDC1_DPI_VSYNC, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA50, LCDC1_DPI_DE,    PIN_NOPULL, 1);

    /* Backlight */
    HAL_PIN_Set(PAD_PA11, GPTIM1_CH4, PIN_NOPULL, 1);   // LCDC1_BL_PWM_CTRL, LCD backlight PWM

#elif defined(BSP_LCDC_USING_DBI)
    HAL_PIN_Set(PAD_PA24, LCDC1_8080_TE,    PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA25, LCDC1_8080_DIO15, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA26, LCDC1_8080_CS,    PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA27, LCDC1_8080_DIO14, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA28, LCDC1_8080_DC,    PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, LCDC1_8080_DIO13, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA30, LCDC1_8080_WR,    PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA31, LCDC1_8080_DIO12, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA32, LCDC1_8080_RD,    PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA33, LCDC1_8080_DIO11, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA37, LCDC1_8080_DIO10, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA39, LCDC1_8080_DIO9, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA41, LCDC1_8080_DIO8, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA43, LCDC1_8080_DIO7, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA44, LCDC1_8080_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA45, LCDC1_8080_DIO6, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA46, LCDC1_8080_DIO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA47, LCDC1_8080_DIO5, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA48, LCDC1_8080_DIO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA49, LCDC1_8080_DIO4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA50, LCDC1_8080_DIO3, PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_PA20, GPTIM1_CH4, PIN_NOPULL, 1);   // LCDC1_BL_PWM_CTRL, LCD backlight PWM

    //Unused pins
    HAL_PIN_Set(PAD_PA38, GPIO_A38, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA40, GPIO_A40, PIN_NOPULL, 1);

#elif defined(BSP_LCDC_USING_DIRECT_EPD)
    BSP_GPIO_Set(EPD_VDD33_EN, 1, 1);
    BSP_GPIO_Set(CTP_VDD33_EN, 1, 1);

    HAL_PIN_Set(PAD_PA25, LCDC1_TCON_GDSP,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA27, LCDC1_TCON_GDCLK,  PIN_NOPULL, 1);
    // Set the pin driver strength to 2mA?
    HAL_PIN_SetDS(PAD_PA27, PIN_DS_2MA);
    HAL_PIN_Set(PAD_PA28, LCDC1_TCON_SDCLK,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, LCDC1_TCON_SDAT15, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA30, LCDC1_TCON_SDLE,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA31, LCDC1_TCON_SDAT14, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA32, LCDC1_TCON_SDOE,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA33, LCDC1_TCON_SDAT13, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA37, LCDC1_TCON_SDAT12, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, LCDC1_TCON_SDSTL,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA39, LCDC1_TCON_SDAT11, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA40, LCDC1_TCON_SDAT0,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA41, LCDC1_TCON_SDAT10, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA42, LCDC1_TCON_SDAT1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA43, LCDC1_TCON_SDAT9, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA44, LCDC1_TCON_SDAT2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA45, LCDC1_TCON_SDAT8, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA46, LCDC1_TCON_SDAT3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA47, LCDC1_TCON_SDAT7, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA48, LCDC1_TCON_SDAT4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA49, LCDC1_TCON_SDAT6, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA50, LCDC1_TCON_SDAT5, PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_PA00 + EPD_GMODE_PIN, GPIO_A0 + EPD_GMODE_PIN, PIN_PULLUP, 1);  //GMODE

    HAL_PIN_Set(PAD_PA11, GPTIM1_CH4, PIN_NOPULL, 1);   // LCDC1_BL_PWM_CTRL, LCD backlight PWM

    //TPS chip control pins
    HAL_PIN_Set(PAD_PA00 + EPD_TPS_WAKEUP_PIN, GPIO_A0 + EPD_TPS_WAKEUP_PIN, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA00 + EPD_TPS_PWRCOM_PIN, GPIO_A0 + EPD_TPS_PWRCOM_PIN, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA00 + EPD_TPS_PWRUP_PIN, GPIO_A0 + EPD_TPS_PWRUP_PIN, PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_PA22, I2C1_SDA, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA23, I2C1_SCL, PIN_PULLUP, 1);
#ifdef EPD_TPS_GOOD_PIN
    HAL_PIN_Set(PAD_PA00 + EPD_TPS_GOOD_PIN, GPIO_A0 + EPD_TPS_GOOD_PIN, PIN_NOPULL, 1);
#endif

#elif defined(BSP_LCDC_USING_QADSPI)
    /*
     * QSPI routing of the sf32lb57-lcd devkit.
     *
     * The devkit shares its LCD connector with the sf32lb58-lcd devkit, so the
     * panel signals come out on the pads that carry the same connector pin
     * here. Each line below is verified against the DPI assignment above:
     *   DIO0 PA39 (DPI G4)   DIO1 PA38 (DPI G3)   DIO2 PA37 (DPI G2)
     *   DIO3 PA32 (DPI G0)   CLK  PA33 (DPI G1)   CS   PA31 (DPI R7)
     *   TE   PA30 (DPI R6)
     * The panel reset is not a SoC pad, it is driven by the AW9523 (P1_7).
     */
    QSPI_PIN_Set(PAD_PA39, LCDC1_MATRIX_SPI_DIO0, PIN_NOPULL, 1);
    QSPI_PIN_Set(PAD_PA38, LCDC1_MATRIX_SPI_DIO1, PIN_NOPULL, 1);
    QSPI_PIN_Set(PAD_PA37, LCDC1_MATRIX_SPI_DIO2, PIN_NOPULL, 1);
    QSPI_PIN_Set(PAD_PA32, LCDC1_MATRIX_SPI_DIO3, PIN_NOPULL, 1);
    QSPI_PIN_Set(PAD_PA33, LCDC1_MATRIX_SPI_CLK,  PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA31, LCDC1_MATRIX_SPI_CS,    PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA30, LCDC1_MATRIX_SPI_TE,    PIN_NOPULL, 1);

    HAL_PIN_Set(PAD_PA45, GPTIM1_CH4, PIN_NOPULL, 1);   // LCDC1_BL_PWM_CTRL, LCD backlight PWM

#endif /* BSP_LCDC_USING_QADSPI */

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_LCDC, RCC_CLK_LCDC_SYSCLK);

    /* Release the panel reset. On this board it is driven by the AW9523
       (P1_7) and drv_lcd never calls BSP_LCD_Reset() on its own, so without
       this the panel stays in reset and never answers the ID read. */
    BSP_LCD_Reset(1);
}

void BSP_LCD_GMODE_Set(uint8_t high1_low0)
{
    BSP_GPIO_Set(EPD_GMODE_PIN, high1_low0, 1);
}


void BSP_TP_PowerUp(void)
{
    aw9523_ensure_init();
    aw9523_set_pin(AW9523_PIN_CTP_RST, 1);

    //TP irq pin
    HAL_PIN_Set(PAD_PA00 + TOUCH_IRQ_PIN, GPIO_A0 + TOUCH_IRQ_PIN, PIN_PULLUP, 1);
}

void BSP_TP_PowerDown(void)
{
    aw9523_ensure_init();
    aw9523_set_pin(AW9523_PIN_CTP_RST, 0);
    //TP irq pin
    HAL_PIN_Set(PAD_PA00 + TOUCH_IRQ_PIN, GPIO_A0 + TOUCH_IRQ_PIN, PIN_NOPULL, 1);
}

void BSP_TP_Reset(uint8_t high1_low0)
{
    aw9523_ensure_init();
    aw9523_set_pin(AW9523_PIN_CTP_RST, high1_low0);
}

uint32_t BSP_GET_DPI_AUX_VSYNC_PIN(void)
{
    return DPI_VSYNC_PIN;
}