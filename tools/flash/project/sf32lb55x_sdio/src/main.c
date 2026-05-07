/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "pmic_controller.h"

#define SD1_RESET_PIN       (37)
#define SD1_EN_PIN          (1)
#define QSPI_POWER_PIN          (58)      // GPIO_A58


extern void BSP_GPIO_Set(int pin, int val, int is_porta);

UART_HandleTypeDef UartHandle;

static uint16_t flash3_div = 5;
static uint16_t flash4_div = 5;
static uint16_t flash5_div = 2;


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


void BSP_GPIO_Set(int pin, int val, int is_porta)
{
    GPIO_TypeDef *gpio = (is_porta) ? (GPIO_TypeDef *)hwp_gpio1 : (GPIO_TypeDef *)hwp_gpio2;
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
    int8_t idx = -1;
    
    get_user_sd0_cfg(NULL, &idx, NULL);
	
	if(idx <= 0)
	{
	    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_PULLDOWN, 1);       // SD1_EN
	    BSP_GPIO_Set(1, 1, 1);
    
	    HAL_PIN_Set(PAD_PA28, SD1_DIO0, PIN_PULLUP, 1);       // SDIO1
	    HAL_PIN_Set(PAD_PA29, SD1_DIO1, PIN_PULLUP, 1);
	    HAL_PIN_Set(PAD_PA30, SD1_DIO2, PIN_PULLUP, 1);
	    HAL_PIN_Set(PAD_PA31, SD1_DIO3, PIN_PULLUP, 1);
	    HAL_PIN_Set(PAD_PA47, SD1_DIO4, PIN_PULLUP, 1);
	    HAL_PIN_Set(PAD_PA49, SD1_DIO5, PIN_PULLUP, 1);
	    HAL_PIN_Set(PAD_PA51, SD1_DIO6, PIN_PULLUP, 1);
	    HAL_PIN_Set(PAD_PA55, SD1_DIO7, PIN_PULLUP, 1);
	    HAL_PIN_Set(PAD_PA34, SD1_CLK, PIN_NOPULL, 1);
	    HAL_PIN_Set(PAD_PA36, SD1_CMD, PIN_PULLUP, 1);

	    HAL_PIN_Set(PAD_PA37, GPIO_A37, PIN_PULLDOWN, 1);     // SDIO1 RESET
	    HAL_Delay_us(100);
	    HAL_PIN_Set(PAD_PA37, GPIO_A37, PIN_PULLUP, 1);

	    //HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_PULLUP, 1);     // SD1 EN
	}
	else
	{
	    HAL_PIN_Set(PAD_PA70, GPIO_A70, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA41, GPIO_A41, PIN_PULLDOWN, 1);

        HAL_PIN_Set(PAD_PA63, SD1_DIO0, PIN_PULLUP, 1);       // SDIO1
        HAL_PIN_Set(PAD_PA65, SD1_DIO1, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA66, SD1_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA68, SD1_DIO3, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA60, SD1_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA61, SD1_CMD, PIN_PULLUP, 1);
	}


    HAL_PIN_Set(PAD_PB45, USART3_TXD, PIN_NOPULL, 0);           // USART3 TX/SPI3_INT
    HAL_PIN_Set(PAD_PB46, USART3_RXD, PIN_PULLUP, 0);           // USART3 RX


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
    UartHandle.Instance        = USART3;
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
