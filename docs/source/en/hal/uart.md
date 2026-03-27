# UART

The Universal Asynchronous Receiver Transmitter (UART) module provides a
flexible way to perform full-duplex data exchange with external devices
requiring industry-standard NRZ asynchronous serial data format. The UART offers
a very wide range of baud rates using a programmable baud rate generator. It
supports modem operations (CTS/RTS) and DMA (Direct Memory Access) for
high-speed communication.

Key Features
 - Configurable oversampling (by 16 or 8) provides flexibility between
   transmission speed and clock tolerance.
 - Programmable baud rates up to 3 Mbit/s.
 - Programmable data word length (7, 8, or 9 bits).
 - Configurable stop bits (1 or 2 bits).
 - Continuous communication via DMA.
 -  Communication control and error detection flags.
 - Parity control: Supports parity bit generation for transmission and parity
   checking for received data.

Note: The UART FIFO size in SiFli chipsets is 1 byte. It is highly recommended
to use DMA for RX operations.


## UART Key Features
Note that the UART FIFO size in SiFli chipsets is 1 byte. It is strongly
recommended to use DMA in the RX direction.

```c
{
    #include "bf0_hal.h"

    void USART4_IRQHandler(void)                                    // UART interrupt handler implementation
    {
        if ((__HAL_UART_GET_FLAG(&amp;(uart-&gt;handle), UART_FLAG_RXNE) != RESET) &amp;&amp;
            (__HAL_UART_GET_IT_SOURCE(&amp;(uart-&gt;handle), UART_IT_RXNE) != RESET))
            printf("Get UART %c", __HAL_UART_GETC(&amp;(uart-&gt;handle)));
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
        UartHandle.Init.WordLength = UART_WORDLENGTH_9B;                // If parity is enabled, word length must be incremented.
    if (HAL_UART_Init(&amp;UartHandle) == HAL_OK)                           // Initialize UART
    {
        HAL_UART_Transmit(&amp;UartHandle, "UART Tx example", 15, 0xFFFF);  // UART TX
    }

    NVIC_EnableIRQ(USART4_IRQn);                                        // Enable UART interrupt 
    __HAL_UART_ENABLE_IT(&amp;(uart-&gt;handle), UART_IT_RXNE);                // Enable UART RXNE interrupt source.

    // Data received from the peer device will trigger the UART RX interrupt.
    ...
}
```


## Using UART
The following example shows UART TX and RX. For UART DMA usage, please refer to
`drv_usart.c` in the RTOS Sifli BSP folder (_rtos/rtthread/bsp/sifli/drivers_)
as an example.

