# SPI

支持4个SPI 其中2个在HCPU(SPI1,SPI2), 2个在LCPU(SPI3,SPI4).

## 主要功能
 - 提供单收、单发、同时收发3种模式，每种模式均支持轮询、中断、DMA3种方法

## 可配置项
 - master/slave
 - 收发共用数据线，或者收发分2根数据线
 - 数据宽度 支持8bit/16bit. 
 - 时钟极性、数据采样沿
 - 帧格式  支持Motorola* Serial Peripheral Interface (SPI)， Texas Instruments* Synchronous Serial Protocol (SSP) National Semiconductor Microwire


## 示例1
SPI1 做master, 发送1000字节的数据
```c

static void spi1_send_example(void)
{
    SPI_HandleTypeDef spi_Handle = {0};
    uint32_t baundRate = 1200000;
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
    spi_Handle.Init.BaudRatePrescaler = (SPI_APB_CLOCK + baundRate / 2) / baundRate;
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


## API参考
[](../api/hal/spi.md)

