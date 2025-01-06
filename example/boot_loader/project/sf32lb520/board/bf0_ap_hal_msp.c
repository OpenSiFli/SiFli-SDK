/* Includes ------------------------------------------------------------------*/
#include <rtconfig.h>
#include "bf0_hal.h"
#include "board.h"
#include "string.h"

void boot_uart_tx(USART_TypeDef *uart, uint8_t *data, int len)
{
    int i;

    for (i = 0; i < len; i++)
    {
        while ((uart->ISR & UART_FLAG_TXE) == 0);
        uart->TDR = (uint32_t)data[i];
    }
}

void boot_error(unsigned char code)
{
    int retry;

    boot_uart_tx(hwp_usart1, &code, 1);
    while (1);
}

/**
* Initializes the Global MSP.
*/
void HAL_MspInit(void)
{
    __HAL_WDT_DISABLE();
}

void mpu_config(void)
{
    // Do nothing
}

void cache_enable(void)
{
    // Do nothing
}

