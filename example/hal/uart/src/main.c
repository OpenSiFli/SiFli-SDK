#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"

#include "bf0_hal_uart.h"
#include "bf0_hal_dma.h"

#include "uart_config.h"
#include "dma_config.h"



#if defined(BSP_USING_BOARD_SF32LB52_LCD_N16R8)
    #define UART2_DMA_RX_IRQHandler          DMAC1_CH6_IRQHandler
#elif defined (BSP_USING_BOARD_SF32LB58_LCD_N16R64N4)
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

#define BUFFER_SIZE 256
#define HALF_BUFFER_SIZE (BUFFER_SIZE/2)
static DMA_HandleTypeDef dma_rx_handle;
static uint8_t buffer[BUFFER_SIZE];




static int last_index = 0;
static int last_index2 = 0;

// 定义接收状态枚举
typedef enum
{
    STATE_UNFULL,
    STATE_HALF_FULL,
    STATE_FULL
} ReceiveState;
// 当前接收状态
ReceiveState currentState = STATE_UNFULL;


// 数据处理函数
void processData(uint8_t *data, uint16_t start, uint16_t length)
{
    printf("rev:");
    for (uint16_t i = start; i < length; i++)
    {
        uint8_t byte = data[i];
        // 打印字符
        printf("%c", byte);
    }
}

void UART_IRQ_HANDLER(void)
{

    if ((__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_IDLE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&UartHandle, UART_IT_IDLE) != RESET))
    {


        // printf("UART_IRQ_HANDLER");

        int recv_total_index = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&dma_rx_handle);
        int recv_len = recv_total_index - last_index;
        if (recv_len < 0)
            recv_len += BUFFER_SIZE;
        if (recv_len)
        {

            int count = __HAL_DMA_GET_COUNTER(&dma_rx_handle);



            switch (currentState)
            {
            case STATE_UNFULL:
                processData(buffer, last_index, recv_total_index);
                last_index2 = recv_total_index;//由空闲态进入全满要及时更新这一次停留的索引
                break;
            case STATE_HALF_FULL:
                if (last_index == 0)
                {
                    last_index2 = recv_total_index;
                }
                else if (count == HALF_BUFFER_SIZE) //刚好结束半满再进入全满则从中间开始打印
                {
                    last_index2 = HALF_BUFFER_SIZE;
                }
                else//超过半满后再进入全满就沿用上一次索引打印
                {
                    last_index2 = last_index;
                }
                processData(buffer, HALF_BUFFER_SIZE, recv_total_index);
                break;
            case STATE_FULL:
                processData(buffer, 0, recv_total_index);
                break;
            }
        }
        currentState = STATE_UNFULL;

        __HAL_UART_CLEAR_IDLEFLAG(&UartHandle);

        // 重置缓冲区和DMA计数器以准备下一次接收
        last_index = recv_total_index;



    }

}

#if defined(BSP_USING_NO_OS) || !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void UART_RX_DMA_IRQ_HANDLER(void)
{

    HAL_DMA_IRQHandler(&dma_rx_handle);

}
#endif

//半满操作
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == hwp_usart2)
    {
        // 实现半满处理逻辑，例如将前半部分数据写入FIFO或进行初步处理
        if (currentState == STATE_UNFULL)
        {
            uint16_t halfLength = HALF_BUFFER_SIZE;
            processData(buffer, last_index, halfLength);//半满则处理从之前的索引开始到中间部分
        }
        else if (currentState == STATE_FULL)
        {
            uint16_t halfLength = HALF_BUFFER_SIZE;
            processData(buffer, 0, halfLength);//半满则处理从之前的索引开始到中间部分
        }
        currentState = STATE_HALF_FULL;
        last_index2 = HALF_BUFFER_SIZE;
        __HAL_DMA_CLEAR_FLAG(&dma_rx_handle, DMA_FLAG_HT6);
    }

}
//全满
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == hwp_usart2)
    {

        // 实现全满处理逻辑，例如将中间至最后的数据写入FIFO或进行初步处理
        uint16_t fullLength = BUFFER_SIZE;

        processData(buffer, last_index2, fullLength);
        currentState = STATE_FULL;
        __HAL_DMA_CLEAR_FLAG(&dma_rx_handle, DMA_FLAG_TC6);
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

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    HAL_StatusTypeDef  ret = HAL_OK;

    /* Output a message on console using printf function */


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

#if defined(BSP_USING_BOARD_SF32LB52_LCD_N16R8)
    HAL_PIN_Set(PAD_PA20, USART2_RXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA27, USART2_TXD, PIN_PULLUP, 1);
#elif defined (BSP_USING_BOARD_SF32LB58_LCD_N16R64N4)
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
    HAL_UART_DmaTransmit(&UartHandle, buffer, BUFFER_SIZE, DMA_PERIPH_TO_MEMORY); /* DMA_PERIPH_TO_MEMORY */

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
    printf("Start uart demo!\n");
    printf("uart2 hal demo!\n");
    /* Output a message on Hyperterminal using hal function */
    uint8_t ptr[] = {'u', 'a', 'r', 't', '2', '\n'};
    int len = 6 ;
    HAL_UART_Transmit(&UartHandle, ptr, len, 0xFFFF);

    printf("uart demo end!\n");
    while (1);
    return 0;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

