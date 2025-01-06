/* Includes ------------------------------------------------------------------*/
#include <rtconfig.h>
#include "bf0_hal.h"
#include "boot_board.h"
#include "string.h"

static void boot_uart_tx(USART_TypeDef *uart, uint8_t *data, int len)
{
    int i;

    for (i = 0; i < len; i++)
    {
        while ((uart->ISR & UART_FLAG_TXE) == 0);
        uart->TDR = (uint32_t)data[i];
    }
}

/**
* Initializes the Global MSP.
*/
#ifdef TARMAC
    #define BOOT_MODE_DELAY 1000
#else
    #define BOOT_MODE_DELAY 1000000
#endif
void HAL_MspInit(void)
{
    HAL_Delay_us(0);                    // Initialize sysclk_m
#ifdef CFG_BOOTROM
    char *boot_tag = "SFBL\n";
    boot_uart_tx(hwp_usart1, (uint8_t *)boot_tag, strlen(boot_tag));
    HAL_Delay_us(BOOT_MODE_DELAY);      // Wait for boot_mode options.
#endif
    __HAL_WDT_DISABLE();
}

#if 0
void mpu_config(void)
{
    // Do nothing
}

void cache_enable(void)
{
    // Do nothing
}
#endif

