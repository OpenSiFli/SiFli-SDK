#include "bsp_board.h"

#define PBR2                    (2)
#define LCD_VCC_EN              (PBR2)      // PBR2

#if defined(BSP_USING_BOARD_EH_SS6700XXX)
    #define LCD_RESET_PIN   (4)                    // GPIO_B4
#else
    #define LCD_RESET_PIN   (5)                    // GPIO_A5
#endif

//#define TP_VCC_EN           (12)          // NO pin
//#define TP_VIO_EN           (13)          // NO pin

#define TP_RESET            (25)            // GPIO_B25
#define TP_INT              (50)            // GPIO_A50


extern void BSP_GPIO_Set(int pin, int val, int is_porta);

void BSP_LCD_Reset(uint8_t high1_low0)
{
#ifdef BSP_USING_BOARD_EH_SS6700XXX
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 0);
#else
    BSP_GPIO_Set(LCD_RESET_PIN, high1_low0, 1);
#endif
}

void BSP_LCD_PowerDown(void)
{
#ifdef PMIC_CTRL_ENABLE
    pmic_device_control(PMIC_OUT_1V8_LVSW100_4, 0, 1); //LCD_1V8 power
    pmic_device_control(PMIC_OUT_LDO30_VOUT, 0, 1);    //LCD_3V3 power
#endif /* PMIC_CTRL_ENABLE */

#ifdef LCD_VCC_EN
    HAL_PBR_WritePin(LCD_VCC_EN, 0);         //PBR2 set 0
#endif

    BSP_LCD_Reset(0);
}

void BSP_LCD_PowerUp(void)
{
#ifdef PMIC_CTRL_ENABLE
    pmic_device_control(PMIC_OUT_1V8_LVSW100_4, 1, 1); //LCD_1V8 power
    pmic_device_control(PMIC_OUT_LDO30_VOUT, 1, 1);    //LCD_3V3 power
#endif /* PMIC_CTRL_ENABLE */

#ifdef LCD_VCC_EN
    HAL_PBR_WritePin(LCD_VCC_EN, 1);        //PBR2 set 1
#endif
}

void BSP_TP_PowerUp(void)
{
#ifdef TP_VCC_EN
    BSP_GPIO_Set(TP_VCC_EN, 1, 1);
#endif
#ifdef TP_VIO_EN
    BSP_GPIO_Set(TP_VIO_EN, 1, 1);
#endif
    BSP_GPIO_Set(TP_RESET,  1, 0);
}

void BSP_TP_PowerDown(void)
{
#ifdef TP_VCC_EN
    BSP_GPIO_Set(TP_VCC_EN, 0, 1);
#endif
#ifdef TP_VIO_EN
    BSP_GPIO_Set(TP_VIO_EN, 0, 1);
#endif
    BSP_GPIO_Set(TP_RESET,  0, 0);
}

void BSP_TP_Reset(uint8_t high1_low0)
{
    BSP_GPIO_Set(TP_RESET, high1_low0, 0);
}



#ifdef HAL_LCD_MODULE_ENABLED
void BSP_GPIO_Set2(int pin, uint32_t mode, int val, int is_porta)
{
    GPIO_TypeDef *gpio = (is_porta) ? hwp_gpio1 : hwp_gpio2;
    GPIO_InitTypeDef GPIO_InitStruct;

    // set sensor pin to output mode
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);

    // set sensor pin to high == power on sensor board
    HAL_GPIO_WritePin(gpio, pin, (GPIO_PinState)val);
}

void HAL_LCDC_SoftSpiInit(SOFT_SPI_PIN_Def pin, SOFT_SPI_IO_Def inout, uint32_t high1low0)
{
    uint32_t mode = ((inout == SW_SPI_INPUT) ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT);
    switch (pin)
    {
    case SW_SPI_CS:
        BSP_GPIO_Set2(36, mode, high1low0, 1);
        HAL_PIN_Set(PAD_PA36, GPIO_A36, PIN_PULLDOWN, 1); //LCDC1_SPI_CS
        break;

    case SW_SPI_CLK:
        BSP_GPIO_Set2(37, mode, high1low0, 1);
        HAL_PIN_Set(PAD_PA37, GPIO_A37, PIN_PULLDOWN, 1); //LCDC1_SPI_CLK
        break;

    case SW_SPI_D0:
        BSP_GPIO_Set2(38, mode, high1low0, 1);
        HAL_PIN_Set(PAD_PA38, GPIO_A38, PIN_PULLDOWN, 1); //LCDC1_SPI_DIO0
        break;

    case SW_SPI_DCX:
    case SW_SPI_D1:
        BSP_GPIO_Set2(39, mode, high1low0, 1);
        HAL_PIN_Set(PAD_PA39, GPIO_A39, PIN_PULLDOWN, 1); //LCDC1_SPI_DIO1
        break;

    default:
        HAL_ASSERT(0);
        break;
    }
}
void HAL_LCDC_SoftSpiDeinit(SOFT_SPI_PIN_Def pin)
{
    switch (pin)
    {
    case SW_SPI_CS:
        HAL_PIN_Set(PAD_PA36, LCDC1_SPI_CS, PIN_NOPULL, 1);
        break;

    case SW_SPI_CLK:
        HAL_PIN_Set(PAD_PA37, LCDC1_SPI_CLK, PIN_NOPULL, 1);
        break;

    case SW_SPI_D0:
        HAL_PIN_Set(PAD_PA38, LCDC1_SPI_DIO0, PIN_PULLDOWN, 1);
        break;

    case SW_SPI_DCX:
    case SW_SPI_D1:
        HAL_PIN_Set(PAD_PA39, LCDC1_SPI_DIO1, PIN_NOPULL, 1);
        break;

    default:
        HAL_ASSERT(0);
        break;
    }
}
uint32_t HAL_LCDC_SoftSpiGetPin(SOFT_SPI_PIN_Def pin)
{
    GPIO_PinState pin_state;
    switch (pin)
    {
    case SW_SPI_D0:
        pin_state = HAL_GPIO_ReadPin(hwp_gpio1, 38);
        break;

    case SW_SPI_D1:
        pin_state = HAL_GPIO_ReadPin(hwp_gpio1, 39);
        break;

    default:
        HAL_ASSERT(0);
        break;
    }

    return (GPIO_PIN_SET == pin_state) ? 1 : 0;
}
void HAL_LCDC_SoftSpiSetPin(SOFT_SPI_PIN_Def pin, uint32_t high1low0)
{
    GPIO_PinState pin_state = (1 == high1low0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    switch (pin)
    {
    case SW_SPI_CS:
        HAL_GPIO_WritePin(hwp_gpio1, 36, pin_state);
        break;

    case SW_SPI_CLK:
        HAL_GPIO_WritePin(hwp_gpio1, 37, pin_state);
        break;

    case SW_SPI_D0:
        HAL_GPIO_WritePin(hwp_gpio1, 38, pin_state);
        break;

    case SW_SPI_DCX:
    case SW_SPI_D1:
        HAL_GPIO_WritePin(hwp_gpio1, 39, pin_state);
        break;

    default:
        HAL_ASSERT(0);
        break;
    }
}
#endif /* HAL_LCD_MODULE_ENABLED */

