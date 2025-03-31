# UART

串口（UART）模块提供了一种灵活的方式，可以与需要行业标准 NRZ 异步串行数据格式的外部设备进行全双工数据交换。 UART 使用可编程波特率发生器提供非常广泛的波特率范围。它支持调制解调器操作 (CTS/RTS) 和 DMA（直接存储器访问）以进行高速通信。

UART主要特点
 - 可配置 16 或 8 的过采样方法，以在速度和时钟容差之间提供灵活性。
 - 高达 3 Mbit/s 的通用可编程发送和接收波特率
 - 可编程数据字长（7、8 或 9 位）
 - 可配置的停止位（1 或 2 个停止位）
 - 使用 DMA 的连续通信 
 -  通信控制/错误检测标志 
 - 奇偶校验控制：发送奇偶校验位，检查接收数据字节的奇偶校验
 
请注意，SiFli 芯片组中的 UART FIFO 大小为 1 个字节。 强烈建议在 RX 方向使用 DMA。


## 使用UART
以下示例显示了 UART TX 和 RX。 UART DMA 的使用，请参考 RTOS Sifli BSP 文件夹（ _rtos/rtthread/bsp/sifli/drivers_ ）中的 `drv_usart.c` 为例。

```c
{
    #include "bf0_hal.h"

    void USART4_IRQHandler(void)                                    // Implement for UART interrupt handler
    {
        if ((__HAL_UART_GET_FLAG(&(uart->handle), UART_FLAG_RXNE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&(uart->handle), UART_IT_RXNE) != RESET))
            printf("Get UART %c", __HAL_UART_GETC(&(uart->handle)));
    }

    
    ...
    
    UART_HandleTypeDef UartHandle;
    
    UartHandle.Instance        = USART4;

    UartHandle.Init.BaudRate   = 1000000;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits   = UART_STOPBITS_1;
    UartHandle.Init.Parity     = UART_PARITY_ODD;
    UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode       = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    if (UartHandle.Init.Parity)     
        UartHandle.Init.WordLength = UART_WORDLENGTH_9B;                // If parity is odd/even, parity should +1
    if (HAL_UART_Init(&UartHandle) == HAL_OK)                           // Initialize UART
    {
        HAL_UART_Transmit(&UartHandle, "UART Tx example", 15, 0xFFFF);  // UART TX
    }
    
    NVIC_EnableIRQ(USART4_IRQn);                                        // Enable UART interrupt 
    __HAL_UART_ENABLE_IT(&(uart->handle), UART_IT_RXNE);                // Enable UART RXNE interrupt source.
    
    // Type on other side of UART will trigger UART interrupt for RX.
    ...
}    
```


## API参考
[](#hal-uart)

