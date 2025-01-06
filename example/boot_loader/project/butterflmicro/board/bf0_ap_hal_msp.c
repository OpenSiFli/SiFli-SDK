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

#define MAX_RETRY 3
void boot_error(unsigned char code)
{
    int retry;

    boot_uart_tx(hwp_usart1, &code, 1);

    retry = hwp_pmuc->CAU_RSVD & MAX_RETRY;
    retry++;
    if (retry <= MAX_RETRY)
    {
        hwp_pmuc->CAU_RSVD &= ~MAX_RETRY;
        hwp_pmuc->CAU_RSVD |= retry;
        HAL_PMU_Reboot();
    }
    else
    {
        while (1);
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
    __HAL_WDT_DISABLE();
#ifdef CFG_BOOTROM
    char *boot_tag = "SFBL\n";
    boot_uart_tx(hwp_usart1, (uint8_t *)boot_tag, strlen(boot_tag));
    if (__HAL_SYSCFG_GET_REVID() < HAL_CHIP_REV_ID_A4 ||
            (hwp_pmuc->CR & (PMUC_CR_HIBER_EN | PMUC_CR_REBOOT)) == 0)
    {
        HAL_Delay_us(BOOT_MODE_DELAY);      // Wait for boot_mode options.
    }
#endif
}

void mpu_config(void)
{
    // Do nothing
}

void cache_enable(void)
{
    // Do nothing
}

