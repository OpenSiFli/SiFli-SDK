/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal.h"
#include "flash_table.h"
#include "drv_io.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "pmic_controller.h"

#define LCD_BOARD_POWER_PIN     (79)      // GPIO_A79
#define LCD_RESET_PIN           (78)      // GPIO_A78
#define TP_RESET_PIN            (1)       // GPIO_A01
#define TP_EN_PIN               (3)       // GPIO_A03 
#define QSPI_POWER_PIN          (58)      // GPIO_A58
#define QSPI2_CLK_PIN           (60)      // GPIO_A60

extern void BSP_GPIO_Set(int pin, int val, int is_porta);

UART_HandleTypeDef UartHandle;

static uint16_t flash1_div = 1;
static uint16_t flash2_div = 1;

static void SystemClock_Config(void);
static void Error_Handler(void);

#if defined(CFG_FACTORY_DEBUG)
extern bool user_pmic_cfg();
extern void user_pin_cfg();
extern bool get_user_sd0_cfg(uint32_t *pAddr, int8_t *pPinIdx, int8_t *pInitIdx);
#endif

void debug_print(char *str)
{
    HAL_UART_Transmit(&UartHandle, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}


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

static void JLINK_DRV_BSP_PIN_Init(void)
{
    int pad;
    pin_function func;

    //MODIFY_REG(hwp_qspi1->WDTR, QSPI_WDTR_TIMEOUT_Msk, QSPI_WDTR_TIMEOUT_Msk);
    //MODIFY_REG(hwp_qspi2->WDTR, QSPI_WDTR_TIMEOUT_Msk, QSPI_WDTR_TIMEOUT_Msk);
    //MODIFY_REG(hwp_qspi3->WDTR, QSPI_WDTR_TIMEOUT_Msk, QSPI_WDTR_TIMEOUT_Msk);

    /* initialize as digital input pulldown */
    func = GPIO_A0;

    for (pad = PAD_PA00; pad <= PAD_PA80; pad++, func++)
    {
        HAL_PIN_Set(pad, func, PIN_PULLDOWN, 1);
    }

    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_NOPULL, 1);              // USB_DP/PDM1_CLK/TP_RESET
    HAL_PIN_Set(PAD_PA03, GPIO_A3, PIN_NOPULL, 1);              // USB_DM/PDM1_DATA/TP_EN
    BSP_GPIO_Set(3, 1, 1);  //PA03 flash2 powerup for moyang 551
    HAL_PIN_Set(PAD_PA10, I2C1_SCL, PIN_PULLUP, 1);             // I2C1(Touch)
    HAL_PIN_Set(PAD_PA14, I2C1_SDA, PIN_PULLUP, 1);

    HAL_PIN_Set(PAD_PA20, LCDC1_SPI_CLK, PIN_NOPULL, 1);        // LCDC 1  QAD-SPI mode
    HAL_PIN_Set(PAD_PA31, LCDC1_SPI_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA34, LCDC1_SPI_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA36, LCDC1_SPI_DIO1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA38, LCDC1_SPI_DIO2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA42, LCDC1_SPI_DIO3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA77, LCDC1_SPI_TE, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA78, GPIO_A78, PIN_NOPULL, 1);            //LCD reset pin

    HAL_PIN_Set(PAD_PA58, GPIO_A58, PIN_PULLUP, 1);             // QSPI2/QSPI3 Power
    HAL_PIN_Set(PAD_PA60, QSPI2_CLK, PIN_NOPULL, 1);            // QSPI2
    HAL_PIN_Set(PAD_PA61, QSPI2_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA63, QSPI2_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA65, QSPI2_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA66, QSPI2_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA68, QSPI2_DIO3, PIN_PULLUP, 1);
#ifdef JLINK_FLASH_3
    HAL_PIN_Set(PAD_PA41, GPIO_A41, PIN_PULLUP, 1);            // QSPI3_EN

    HAL_PIN_Set(PAD_PA44, QSPI3_CLK, PIN_NOPULL, 1);            // QSPI3
    HAL_PIN_Set(PAD_PA45, QSPI3_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA47, QSPI3_DIO0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA49, QSPI3_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA51, QSPI3_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA55, QSPI3_DIO3, PIN_PULLUP, 1);
#else
    HAL_PIN_Set(PAD_PA47, QSPI2_DIO4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA49, QSPI2_DIO5, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA51, QSPI2_DIO6, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA55, QSPI2_DIO7, PIN_PULLUP, 1);
#endif
    HAL_PIN_Set(PAD_PA70, GPIO_A70, PIN_NOPULL, 1);           // LCD backlight
    HAL_PIN_Set(PAD_PA79, GPIO_A79, PIN_NOPULL, 1);             // LCD Power, LCDC_SPI_EN

    /* initialize as digital input pulldown */
    func = GPIO_B0;
    for (pad = PAD_PB00; pad <= PAD_PB48; pad++, func++)
    {
        /* skip SWD pad */
        if ((PAD_PB31 != pad) && (PAD_PB34 != pad))
        {
            HAL_PIN_Set(pad, func, PIN_PULLDOWN, 0);
        }
    }

    HAL_PIN_Set(PAD_PB20, GPIO_B20, PIN_PULLUP, 0);         //QSPI4_EN for 557 evb
    BSP_GPIO_Set(20, 1, 0);

    BSP_GPIO_Set(1, 0, 0);                                     //PB01 flash4 powerup for moyang 555
    HAL_PIN_Set(PAD_PB06, GPIO_B6, PIN_NOPULL, 0);             //PB06 flash2 powerup for moyang 555
    BSP_GPIO_Set(6, 1, 0);                                     //PB06 flash2 powerup for moyang 555
    HAL_PIN_Set(PAD_PB04, I2C4_SCL, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB05, I2C4_SDA, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB08, GPIO_B8, PIN_NOPULL, 0);              // Temperature ADC
    HAL_PIN_Select(PAD_PB08, 10, 0);
    HAL_PIN_Set(PAD_PB10, GPIO_B10, PIN_NOPULL, 0);             // Battery Voltage ADC
    HAL_PIN_Select(PAD_PB10, 10, 0);

    HAL_PIN_Set(PAD_PB13, SPI3_CLK, PIN_PULLDOWN, 0);             // SPI3 (GSensor)
    HAL_PIN_Set(PAD_PB16, SPI3_DO, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB19, SPI3_DI, PIN_PULLDOWN, 0);
    HAL_PIN_Set(PAD_PB23, SPI3_CS, PIN_PULLUP, 0);

    //HAL_PIN_Set(PAD_PB24, GPIO_B24, PIN_NOPULL, 0);             // SYS_EN/I2C5_INT/PMIC control
    HAL_PIN_Set(PAD_PB25, GPIO_B25, PIN_NOPULL, 0);             // SPI3_EN
    HAL_PIN_Select(PAD_PB25, 10, 0);
    HAL_PIN_Set(PAD_PB29, GPIO_B29, PIN_NOPULL, 0);             // I2C1(Touch) interrupt
    // PB31 SWCLK,
    // PB34 SWDIO
    HAL_PIN_Set(PAD_PB43, I2C5_SCL, PIN_PULLUP, 0);             // I2C5(PMIC control)
    HAL_PIN_Set(PAD_PB44, I2C5_SDA, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB45, USART3_TXD, PIN_NOPULL, 0);           // USART3 TX/SPI3_INT
    HAL_PIN_Set(PAD_PB46, USART3_RXD, PIN_PULLUP, 0);           // USART3 RX
    //HAL_PIN_Set(PAD_PB47, GPIO_B47, PIN_PULLUP, 0);             // (USB_DET)Charger INT
    HAL_PIN_Set(PAD_PB48, GPIO_B48, PIN_NOPULL, 0);             // Key1


    HAL_PIN_SetMode(PAD_PA01, 1, PIN_DIGITAL_O_NORMAL);
    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_NOPULL, 1);
//    BSP_GPIO_Set(TP_RESET_PIN, 1, 1);  //TP_RST
//    BSP_GPIO_Set(TP_EN_PIN, 1, 1);  //TP_EN
    HAL_PIN_Set(PAD_PA60, QSPI2_CLK, PIN_NOPULL, 1);
//    BSP_GPIO_Set(LCD_BOARD_POWER_PIN, 1, 1);
    BSP_GPIO_Set(QSPI_POWER_PIN, 1, 1);

#if defined(CFG_FACTORY_DEBUG)
    if (user_pmic_cfg() == false)
    {
        BSP_PMIC_Init(-1, -1);
        BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_1, 1, 1); //LCD_1V8 power
        BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_2, 1, 1);
        BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_3, 1, 1);
        BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_4, 1, 1);
        BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_5, 1, 1); 	
        BSP_PMIC_Control(PMIC_OUT_LDO33_VOUT, 1, 1);
    }

    user_pin_cfg();
#endif


    {
        //Disable WDTs
        WDT_HandleTypeDef hwdt;

        hwdt.Instance = hwp_wdt1;
        __HAL_WDT_PROTECT(&hwdt, 0);
        __HAL_WDT_STOP(&hwdt);

        hwdt.Instance = hwp_wdt2;
        __HAL_WDT_PROTECT(&hwdt, 0);
        __HAL_WDT_STOP(&hwdt);

        hwdt.Instance = hwp_iwdt;
        __HAL_WDT_PROTECT(&hwdt, 0);
        __HAL_WDT_STOP(&hwdt);
    }

    // delay 6ms to wait for flash power stable
    //HAL_Delay_us(0);
    //HAL_Delay_us(50000);
    {
        uint32_t sysFreq = HAL_RCC_GetSysCLKFreq(CORE_ID_HCPU);
        volatile uint32_t lootMs = 20 * (sysFreq / 1000) / 5;

        if (lootMs == 0)
        {
            lootMs = 480000;
        }

        while (lootMs-- > 0);
    }
    
}


void HAL_MspInit(void)
{
    HAL_RCC_Reset_and_Halt_LCPU(1);

    spi_nor_table_init();
    spi_nand_table_init();

    JLINK_DRV_BSP_PIN_Init();

    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART configured as follows:
        - Word Length = 8 Bits (7 data bit + 1 parity bit) :
       BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
        - Stop Bit    = One Stop bit
        - Parity      = ODD parity
        - BaudRate    = 9600 baud
        - Hardware flow control disabled (RTS and CTS signals) */

    memset(&UartHandle, 0, sizeof(UART_HandleTypeDef));

#ifndef SOC_BF_Z0
    UartHandle.Instance        = USART3;
#else   //
    UartHandle.Instance        = USART4;
#endif
    UartHandle.Init.BaudRate   = 1000000;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        /* Initialization Error */
        HAL_ASSERT(0);
    }
    //debug_print("Init\r\n");
}


HAL_StatusTypeDef HAL_EFUSE_Init()
{
    return HAL_OK;
}

//HAL_StatusTypeDef HAL_HPAON_StartGTimer(void)
//{
//    return HAL_OK;
//}

/* Private functions ---------------------------------------------------------*/

#ifdef JLINK
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    /* HAL library initialization:
         - Configure the Flash prefetch, instruction and Data caches
         - Systick timer is configured by default as source of time base, but user
           can eventually implement his proper time base source (a general purpose
           timer for example or other time source), keeping in mind that Time base
           duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
           handled in milliseconds basis.
         - Set NVIC Group Priority to 4
         - Low Level Initialization: global MSP (MCU Support Package) initialization
       */
    /* Initialize system resurces, such as MPU, SCB, efuse etc. */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

uint32_t SystemCoreClock = 48000000;

void SystemInit(void)
{
}

pm_power_on_mode_t SystemPowerOnModeGet(void)
{
    return PM_COLD_BOOT;
}

/**
  * @}
  */

/**
  * @}
  */
