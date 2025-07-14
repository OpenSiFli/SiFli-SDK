# SPI

Supports 4 SPI interfaces, with 2 on HCPU (SPI1, SPI2) and 2 on LCPU (SPI3, SPI4).

## Key Features
 - Provides receive-only, transmit-only, and simultaneous transmit-receive modes, with each mode supporting polling, interrupt, and DMA methods

## Configurable Options
 - Master/slave
 - Shared data line for transmit/receive, or separate 2 data lines for transmit/receive
 - Data width: supports 8-bit/16-bit
 - Clock polarity, data sampling edge
 - Frame format: supports Motorola Serial Peripheral Interface (SPI), Texas Instruments Synchronous Serial Protocol (SSP), National Semiconductor Microwire

## Example 1
SPI1 as master, transmitting 1000 bytes of data
```c

static void spi1_send_example(void)
{
    SPI_HandleTypeDef spi_Handle = {0};
    uint32_t baudRate = 1200000;
    uint8_t * txBuff = 0x2000c000;
    uint32_t txBuffLenInBytes = 1000;
    uint32_t tx_timeout_tick = 5000;
    
    
    rt_uint32_t SPI_APB_CLOCK = HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);

    spi_Handle.Instance = SPI1;
    spi_Handle.Init.Direction = SPI_DIRECTION_2LINES;
    spi_Handle.Init.Mode = SPI_MODE_MASTER;
    spi_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    spi_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    spi_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
    spi_Handle.Init.BaudRatePrescaler = (SPI_APB_CLOCK + baudRate / 2) / baudRate;
    spi_Handle.Init.FrameFormat = SPI_FRAME_FORMAT_SPI;
    spi_Handle.State = HAL_SPI_STATE_RESET;

    HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
    NVIC_EnableIRQ(SPI1_IRQn);

    if (HAL_SPI_Init(&spi_Handle) == HAL_OK)
    {
        if(HAL_OK ==  HAL_SPI_Transmit(&spi_Handle, txBuff, txBuffLenInBytes, tx_timeout_tick))
        {
            //Transmit done.
        }
    }

    NVIC_DisableIRQ(SPI1_IRQn);
}

```

## API Reference
[](../api/hal/spi.md)