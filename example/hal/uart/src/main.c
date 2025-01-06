#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"

#include "uart_config.h"
#include "dma_config.h"



#if defined(BSP_USING_BOARD_EM_LB525XXX)
    #define UART2_DMA_RX_IRQHandler          DMAC1_CH6_IRQHandler
#elif defined (BSP_USING_BOARD_EM_LB587XXX)
    #define UART2_DMA_RX_IRQHandler          DMAC1_CH5_IRQHandler
#endif
#define UART2_RX_DMA_RCC                 0
#define UART2_RX_DMA_INSTANCE            DMA1_Channel6
#define UART2_RX_DMA_REQUEST             DMA_REQUEST_7
#define UART2_RX_DMA_IRQ                 DMAC1_CH6_IRQn

#define UART_INSTANCE           hwp_usart2
#define UART_INTERRUPT          USART2_IRQn
#define UART_IRQ_HANDLER        USART2_IRQHandler
#define UART_RX_DMA_INSTANCE    UART2_RX_DMA_INSTANCE
#define UART_RX_DMA_REQUEST     UART2_RX_DMA_REQUEST
#define UART_RX_DMA_IRQ         UART2_RX_DMA_IRQ
#define UART_RX_DMA_IRQ_HANDLER UART2_DMA_RX_IRQHandler


/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* Private function prototypes -----------------------------------------------*/

#ifdef __CC_ARM
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif __ICCARM__

#else
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
int _write(int fd, char *ptr, int len)
{

    if (fd > 2)
    {
        return -1;
    }
    HAL_UART_Transmit(&UartHandle, ptr, len, 0xFFFF);
    return len;
}
#endif

#define BUFF_LEN 256
static DMA_HandleTypeDef dma_rx_handle;
static uint8_t buffer[BUFF_LEN];

void UART_IRQ_HANDLER(void)
{
    rt_interrupt_enter();
    if ((__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_IDLE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&UartHandle, UART_IT_IDLE) != RESET))
    {
        static int last_index = 0;
        int recv_total_index = BUFF_LEN - __HAL_DMA_GET_COUNTER(&dma_rx_handle);
        int recv_len = recv_total_index - last_index;

        if (recv_len < 0)
            recv_len += BUFF_LEN;
        if (recv_len)
        {
            int i;
            rt_kprintf("Receive %d\n", recv_len);
            for (i = 0; i < recv_len; i++)
            {
                rt_kprintf("%02X ", buffer[(i + last_index) % BUFF_LEN]);
            }
            rt_kprintf("\n");
        }
        last_index = recv_total_index;
        __HAL_UART_CLEAR_IDLEFLAG(&UartHandle);
    }
    rt_interrupt_leave();
}

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
void UART_RX_DMA_IRQ_HANDLER(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&dma_rx_handle);
    rt_interrupt_leave();
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

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
    HAL_StatusTypeDef  ret = HAL_OK;

    /* Output a message on console using printf function */
    rt_kprintf("Start uart demo!\n");

    /*##-1- Configure the UART peripheral ######################################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
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
    if (UartHandle.Init.Parity)
    {
        // If parity is odd/even, parity should +1
        UartHandle.Init.WordLength = UART_WORDLENGTH_9B;
    }

    /* 1, pinmux set to uart mode */

#if defined(BSP_USING_BOARD_EM_LB525XXX)
    HAL_PIN_Set(PAD_PA20, USART2_RXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA27, USART2_TXD, PIN_PULLUP, 1);
#elif defined (BSP_USING_BOARD_EM_LB587XXX)
    HAL_PIN_Set(PAD_PA29, USART2_RXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA28, USART2_TXD, PIN_PULLUP, 1);
#endif
    /* 2, open uart2 clock source  */
    HAL_RCC_EnableModule(RCC_MOD_USART2);
    if (HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        /* Initialization Error */
        HAL_ASSERT(0);
    }

    // Start RX DMA
    __HAL_LINKDMA(&(UartHandle), hdmarx, dma_rx_handle);
    dma_rx_handle.Instance = UART_RX_DMA_INSTANCE;
    dma_rx_handle.Init.Request = UART_RX_DMA_REQUEST;
    HAL_UART_DmaTransmit(&UartHandle, buffer, BUFF_LEN, DMA_PERIPH_TO_MEMORY); /* DMA_PERIPH_TO_MEMORY */

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    // Enable DMA IRQ
    HAL_NVIC_SetPriority(UART_RX_DMA_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(UART_RX_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

    {
        // For RX DMA, also need to enable UART IRQ.
        __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_IDLE); /* Set to generates interrupts when UART idle */
        HAL_NVIC_SetPriority(UART_INTERRUPT, 1, 0);
        HAL_NVIC_EnableIRQ(UART_INTERRUPT);
    }
    /* Output a message on Hyperterminal using printf function */
    printf("uart2 hal demo!\n");
    /* Output a message on Hyperterminal using hal function */
    uint8_t ptr[] = {'u', 'a', 'r', 't', '2', '\n'};
    int len = 6 ;
    HAL_UART_Transmit(&UartHandle, ptr, len, 0xFFFF);

    rt_kprintf("uart demo end!\n");
    while (1);
    return 0;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

