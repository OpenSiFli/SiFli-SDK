#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"


/**
  * @brief  Systick interrupt handler
  *     HAL_Init initialize systick for system timing.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* User code start from here --------------------------------------------------------*/
#ifdef CONSOLE_UART1
    #define UART_INSTANCE           hwp_usart1
#elif defined(CONSOLE_UART3)
    #define UART_INSTANCE           hwp_usart3
#elif defined(CONSOLE_UART4)
    #define UART_INSTANCE           hwp_usart4
#else
    #error RT_CONSOLE_DEVICE_NAME is not supported yet in this app.
#endif



/* UART handler declaration */
UART_HandleTypeDef UartHandle;

#ifdef __CC_ARM                                                 /*ARMCC*/
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) /*ARMCLANG*/
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif __ICCARM__                                                /*IAR**/
#error Not support yet
#else                                                           /*GCC*/
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
int _write(int fd, char *ptr, int len)
{
    if (fd > 2)
        return -1;
    HAL_UART_Transmit(&UartHandle, ptr, len, 0xFFFF);
    return len;
}
#endif

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART1 and Loop until the end of transmission */
    HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* HAL library initialization: */
    HAL_Init();

    /* Configure the UART peripheral  */
    /* UART configured as follows:
        - Word Length = 8 Bits (8 data bit + 0 parity bit) :
            BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
        - Stop Bit    = One Stop bit
        - Parity      = No parity
        - BaudRate    = 1000000 baud
        - Hardware flow control disabled (RTS and CTS signals) */
    UartHandle.Instance        = UART_INSTANCE;
    UartHandle.Init.BaudRate   = 1000000;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&UartHandle) != HAL_OK)
        HAL_ASSERT(0);  /* Initialization Error */

    /* Output a message on console using printf function */
    printf("Hello world!\n");

    /* Infinite loop */
    while (1)
    {
    }
    return 0;
}

