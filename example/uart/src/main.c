#include "rtconfig.h"
#include "bf0_hal.h"
#include "uart_config.h"
#include "dma_config.h"
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
    #define UART_INTERRUPT          USART1_IRQn
    #define UART_IRQ_HANDLER        USART1_IRQHandler
    #define UART_RX_DMA_INSTANCE    UART1_RX_DMA_INSTANCE
    #define UART_RX_DMA_REQUEST     UART1_RX_DMA_REQUEST
    #define UART_RX_DMA_IRQ         UART1_RX_DMA_IRQ
    #define UART_RX_DMA_IRQ_HANDLER UART1_DMA_RX_IRQHandler
#elif defined(CONSOLE_UART3)
    #define UART_INSTANCE           hwp_usart3
    #define UART_INTERRUPT          USART3_IRQn
    #define UART_IRQ_HANDLER        USART3_IRQHandler
    #define UART_RX_DMA_INSTANCE    UART3_RX_DMA_INSTANCE
    #define UART_RX_DMA_REQUEST     UART3_RX_DMA_REQUEST
    #define UART_RX_DMA_IRQ         UART3_RX_DMA_IRQ
    #define UART_RX_DMA_IRQ_HANDLER UART3_DMA_RX_IRQHandler
#elif defined(CONSOLE_UART4)
    #define UART_INSTANCE           hwp_usart4
    #define UART_INTERRUPT          USART4_IRQn
    #define UART_IRQ_HANDLER        USART4_IRQHandler
    #define UART_RX_DMA_INSTANCE    UART4_RX_DMA_INSTANCE
    #define UART_RX_DMA_REQUEST     UART4_RX_DMA_REQUEST
    #define UART_RX_DMA_IRQ         UART4_RX_DMA_IRQ
    #define UART_RX_DMA_IRQ_HANDLER UART4_DMA_RX_IRQHandler
#else
    #error RT_CONSOLE_DEVICE_NAME is not supported yet in this app.
#endif


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
            printf("Receive %d\n", recv_len);
            for (i = 0; i < recv_len; i++)
            {
                printf("%02X ", buffer[(i + last_index) % BUFF_LEN]);
            }
            printf("\n");
        }
        last_index = recv_total_index;
        __HAL_UART_CLEAR_IDLEFLAG(&UartHandle);
    }
}


void UART_RX_DMA_IRQ_HANDLER(void)
{
    HAL_DMA_IRQHandler(&dma_rx_handle);
}


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    /* HAL library initialization */
    HAL_Init();



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
    if (HAL_UART_Init(&UartHandle) != HAL_OK)
    {
        /* Initialization Error */
        HAL_ASSERT(0);
    }

    /* Output a message on Hyperterminal using printf function */
    printf("UART Printf Example: retarget the C library printf function to the UART\n");
    printf("** Test finished successfully. ** \n");

    // Start RX DMA
    __HAL_LINKDMA(&(UartHandle), hdmarx, dma_rx_handle);
    dma_rx_handle.Instance = UART_RX_DMA_INSTANCE;
    dma_rx_handle.Init.Request = UART_RX_DMA_REQUEST;
    HAL_UART_DmaTransmit(&UartHandle, buffer, BUFF_LEN, DMA_PERIPH_TO_MEMORY);

    // Enable DMA IRQ
    HAL_NVIC_SetPriority(UART_RX_DMA_IRQ, 0, 0);
    HAL_NVIC_EnableIRQ(UART_RX_DMA_IRQ);

    {
        // For RX DMA, also need to enable UART IRQ.
        __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_IDLE);
        HAL_NVIC_SetPriority(UART_INTERRUPT, 1, 0);
        HAL_NVIC_EnableIRQ(UART_INTERRUPT);
    }
    /* Infinite loop */
    while (1)
    {
    }
}

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

