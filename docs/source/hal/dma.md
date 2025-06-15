# DMA


DMA 有2个一个在HCPU,一个在LCPU, 分别都支持内存-内存，内存-外设，外设-内存，外设-外设之间的传输.


## 主要特性
 - 8个独立的可配置通道
 - 每个通道的DMA请求可在16个硬件请求中选择1个，或由软件请求
 - 每个通道支持4档优先级配置，优先级相同时依照通道编号大小判决
 - 支持内存-内存，内存-外设，外设-内存，外设-外设传输
 - 源地址和目标地址均独立支持单字节/双字节/4字节访问，并都可以独立选择地址是否自动递增
 - 支持循环缓冲模式，单次传输完成后自动重新启动
 - 每个通道支持3种事件标志-传输完成，过半传输，传输出错，并能各自独立产生中断请求
 - 单次配置最大传输单元数为65536，每单元根据不同配置分别为单字节/双字节/4字节传输
```{only} SF32LB55X or SF32LB56X or SF32LB58X
 - 单次配置传输地址范围限制在1M字节以内，即传输过程中源地址和目标地址均不可越过1M字节边界(地址的bit31-bit20不可改变)
```

## DMAC对应的外设request id

|req_sel		|55x DMAC1			|55x DMAC2			|58x DMAC1			|58x DMAC2			|58x DMAC3			|56x DMAC1			|56x DMAC2			|52x DMAC1			|52x DMAC2			|
|---------------|-------------------|-------------------|-------------------|-------------------|-------------------|-------------------|-------------------|-------------------|-------------------|
|0			|qspi1			|usart3_tx			|mpi1			|i2s1_tx			|usart4_tx			|mpi1			|usart4_tx			|mpi1			|usart4_tx|
|1			|qspi2			|usart3_rx			|mpi2/i2c4			|i2s1_rx			|usart4_rx			|mpi2			|usart4_rx			|mpi2			|usart4_rx|
|2			|qspi3			|usart4_tx			|mpi3			|i2s2_tx			|usart5_tx			|mpi3			|usart5_tx			|/			|usart5_tx|
|3			|/			|usart4_rx			|mpi4			|i2s2_rx			|usart5_rx			|i2c4			|usart5_rx			|i2c4			|usart5_rx|
|4			|usart1_tx			|usart5_tx			|usart1_tx			|pdm1_rx_l			|usart6_tx			|usart1_tx			|usart6_tx			|usart1_tx			|/|
|5			|usart1_rx			|usart5_rx			|usart1_rx			|pdm1_rx_r			|usart6_rx			|usart1_rx			|usart6_rx			|usart1_rx			|/|
|6			|usart2_tx			|btim3			|usart2_tx			|pdm2_rx_l			|btim3			|usart2_tx			|btim3			|usart2_tx			|btim3|
|7			|usart2_rx			|btim4			|usart2_rx			|pdm2_rx_r			|btim4			|usart2_rx			|btim4			|usart2_rx			|btim4|
|8			|gptim1_update			|gptim3_update			|gptim1_update			|/			|gptim3_update			|gptim1_update			|gptim3_update			|gptim1_update			|/|
|9			|gptim1_trigger			|gptim3_trigger			|gptim1_trigger			|dac0			|gptim3_trigger			|gptim1_trigger			|gptim3_trigger			|gptim1_trigger			|/|
|10			|gptim1_cc1			|gptim3_cc1			|gptim1_cc1			|dac1			|gptim3_cc1			|gptim1_cc1			|gptim3_cc1			|gptim1_cc1			|/|
|11			|gptim1_cc2			|gptim3_cc2			|gptim1_cc2			|gptim2_update			|gptim3_cc2			|gptim1_cc2			|gptim3_cc2			|gptim1_cc2			|/|
|12			|gptim1_cc3			|gptim3_cc3			|gptim1_cc3			|gptim2_trigger			|gptim3_cc3			|gptim1_cc3			|gptim3_cc3			|gptim1_cc3			|/|
|13			|gptim1_cc4			|gptim3_cc4			|gptim1_cc4			|gptim2_cc1			|gptim3_cc4			|gptim1_cc4			|gptim3_cc4			|gptim1_cc4			|/|
|14			|btim1			|gptim5_update			|btim1			|audprc_tx_out_ch1			|gptim5_update			|btim1			|gptim5_update			|btim1			|/|
|15			|btim2			|gptim5_trigger			|btim2			|audprc_tx_out_ch0			|gptim5_trigger			|btim2			|gptim5_trigger			|btim2			|/|
|16			|/			|spi3_tx			|atim1_update			|audprc_tx_ch3			|spi3_tx			|atim1_update			|spi3_tx			|atim1_update			|/|
|17			|i2c3			|spi3_rx			|atim1_trigger			|audprc_tx_ch2			|spi3_rx			|atim1_trigger			|spi3_rx			|atim1_trigger			|/|
|18			|i2s1_tx/pdm1_l			|spi4_tx			|atim1_cc1			|audprc_tx_ch1			|spi4_tx			|atim1_cc1			|spi4_tx			|atim1_cc1			|/|
|19			|i2s1_rx/pdm1_r			|spi4_rx			|atim1_cc2			|audprc_tx_ch0			|spi4_rx			|atim1_cc2			|spi4_rx			|atim1_cc2			|/|
|20			|i2s2_tx/pdm2_l			|qspi4			|atim1_cc3			|audprc_rx_ch1			|mpi5			|atim1_cc3			|mpi5			|atim1_cc3			|/|
|21			|i2s2_rx/pdm2_r			|i2c4			|atim1_cc4			|audprc_rx_ch0			|i2c5			|atim1_cc4			|i2c5			|atim1_cc4			|/|
|22			|i2c1			|i2c5			|i2c1			|gptim2_cc2			|i2c6			|i2c1			|i2c6			|i2c1			|/|
|23			|i2c2			|i2c6			|i2c2			|gptim2_cc3			|i2c7			|i2c2			|i2c7			|i2c2			|/|
|24			|gptim2_update			|gptim4_update			|i2c3			|gptim2_cc4			|gptim4_update			|i2c3			|gptim4_update			|i2c3			|/|
|25			|gptim2_trigger			|gptim4_trigger			|atim1_com			|atim2_update			|gptim4_trigger			|atim1_com			|gptim4_trigger			|atim1_com			|/|
|26			|gptim2_cc1			|gptim4_cc1			|usart3_tx			|atim2_trigger			|i2s3_rx/gptim4_cc1			|usart3_tx			|gptim4_cc1			|usart3_tx			|/|
|27			|gptim2_cc2			|gptim4_cc2			|usart3_rx			|atim2_cc1			|i2s3_tx/gptim4_cc2			|usart3_rx			|gptim4_cc2			|usart3_rx			|/|
|28			|spi1_tx			|gptim4_cc3			|spi1_tx			|atim2_cc2			|audadc_ch0/gptim4_cc3			|spi1_tx			|audadc_ch0/gptim4_cc3			|spi1_tx			|/|
|29			|spi1_rx			|gptim4_cc4			|spi1_rx			|atim2_cc3			|audadc_ch1/gptim4_cc4			|spi1_rx			|audadc_ch1/gptim4_cc4			|spi1_rx			|/|
|30			|spi2_tx			|gpadc			|spi2_tx			|atim2_cc4			|gpadc			|spi2_tx			|gpadc			|spi2_tx			|/|
|31			|spi2_rx			|sdadc			|spi2_rx			|atim2_com			|sdadc			|spi2_rx			|/			|spi2_rx			|/|
|32			|	/		|	/		|	/		|	/		|	/		|i2s1_tx			|	/		|i2s1_tx			|/|
|33			|	/		|	/		|	/		|	/		|	/		|i2s1_rx			|	/		|i2s1_rx			|/|
|34			|	/		|	/		|	/		|	/		|	/		|sci_tx			|	/		|/			|/|
|35			|	/		|	/		|	/		|	/		|	/		|sci_rx			|	/		|/			|/|
|36			|	/		|	/		|	/		|	/		|	/		|pdm1_rx_l			|	/		|pdm1_rx_l			|/|
|37			|	/		|	/		|	/		|	/		|	/		|pdm1_rx_r			|	/		|pdm1_rx_r			|/|
|38			|	/		|	/		|	/		|	/		|	/		|pdm2_rx_l			|	/		|gpadc			|/|
|39			|	/		|	/		|	/		|	/		|	/		|pdm2_rx_r			|	/		|adc0			|/|
|40			|	/		|	/		|	/		|	/		|	/		|/			|	/		|adc1			|/|
|41			|	/		|	/		|	/		|	/		|	/		|dac0			|	/		|dac0			|/|
|42			|	/		|	/		|	/		|	/		|	/		|dac1			|	/		|dac1			|/|
|43			|	/		|	/		|	/		|	/		|	/		|gptim2_update			|	/		|gptim2_update			|/|
|44			|	/		|	/		|	/		|	/		|	/		|gptim2_trigger			|	/		|gptim2_trigger			|/|
|45			|	/		|	/		|	/		|	/		|	/		|gptim2_cc1			|	/		|gptim2_cc1			|/|
|46			|	/		|	/		|	/		|	/		|	/		|audprc_tx_out_ch1			|	/		|audprc_tx_out_ch1			|/|
|47			|	/		|	/		|	/		|	/		|	/		|audprc_tx_out_ch0			|	/		|audprc_tx_out_ch0			|/|
|48			|	/		|	/		|	/		|	/		|	/		|audprc_tx_ch3			|	/		|audprc_tx_ch3			|/|
|49			|	/		|	/		|	/		|	/		|	/		|audprc_tx_ch2			|	/		|audprc_tx_ch2			|/|
|50			|	/		|	/		|	/		|	/		|	/		|audprc_tx_ch1			|	/		|audprc_tx_ch1			|/|
|51			|	/		|	/		|	/		|	/		|	/		|audprc_tx_ch0			|	/		|audprc_tx_ch0			|/|
|52			|	/		|	/		|	/		|	/		|	/		|audprc_rx_ch1			|	/		|audprc_rx_ch1			|/|
|53			|	/		|	/		|	/		|	/		|	/		|audprc_rx_ch0			|	/		|audprc_rx_ch0			|/|
|54			|	/		|	/		|	/		|	/		|	/		|gptim2_cc2			|	/		|gptim2_cc2			|/|
|55			|	/		|	/		|	/		|	/		|	/		|gptim2_cc3			|	/		|gptim2_cc3			|/|
|56			|	/		|	/		|	/		|	/		|	/		|gptim2_cc4			|	/		|gptim2_cc4			|/|
|57			|	/		|	/		|	/		|	/		|	/		|sdmmc2			|	/		|sdmmc1			|/|
|58			|	/		|	/		|	/		|	/		|	/		|/			|	/		|/			|/|
|59			|	/		|	/		|	/		|	/		|	/		|/			|	/		|/			|/|
|60			|	/		|	/		|	/		|	/		|	/		|/			|	/		|/			|/|
|61			|	/		|	/		|	/		|	/		|	/		|/			|	/		|/			|/|
|62			|	/		|	/		|	/		|	/		|	/		|/			|	/		|/			|/|
|63			|	/		|	/		|	/		|	/		|	/		|/			|	/		|/			|/|



## 使用 DMA示例1
	内存-内存 4字节传输 4096字节.
```c

    DMA_HandleTypeDef hdma;
    HAL_StatusTypeDef err;

    uint32_t SrcAddress = 0x10000000;
    uint32_t DstAddress = 0x20000000;
    uint32_t Counts     = 1024;  //Transfer unit is word, so Counts= 4096 / 4
    
    /* Init DMA configure*/
    hdma.Instance                 = DMA1_Channel6;
    hdma.Init.Request             = 0;                     //Request id is useless while memory to memory, just set to 0
    hdma.Init.Direction           = DMA_MEMORY_TO_MEMORY;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma.Init.PeriphInc           = DMA_PINC_ENABLE;
    hdma.Init.MemInc              = DMA_MINC_ENABLE;
    hdma.Init.Mode                = DMA_NORMAL;
    hdma.Init.Priority            = DMA_PRIORITY_MEDIUM;

    hdma.XferHalfCpltCallback = DMA_Xfer_Half_Callback_Func;
    hdma.XferCpltCallback     = DMA_Xfer_Complete_Callback_Func;
    hdma.XferErrorCallback    = DMA_Xfer_Error_Callback_Func;

    err = HAL_DMA_Init(&hdma);
    if (err != HAL_OK)
        return err;

    err = HAL_DMA_Start_IT(hadc->DMA_Handle, SrcAddress, DstAddress, Counts);
    if (err != HAL_OK)
        return err;

```


## 使用 DMA示例2
 ADC模块-内存 4字节传输 4096字节.
```c

    DMA_HandleTypeDef hdma;
    HAL_StatusTypeDef err;

    uint32_t SrcAddress = 0x10000000;
    uint32_t DstAddress = 0x20000000;
    uint32_t Counts     = 1024;  //Transfer unit is word, so Counts= 4096 / 4
    
    /* Init DMA configure*/
    hdma.Instance                 = DMA1_Channel6;
    hdma.Init.Request             = DMA_REQUEST_12;
    hdma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma.Init.PeriphInc           = DMA_PINC_DISABLE;      //Periphral address NOT auto-increment
    hdma.Init.MemInc              = DMA_MINC_ENABLE;
    hdma.Init.Mode                = DMA_NORMAL;
    hdma.Init.Priority            = DMA_PRIORITY_MEDIUM;

    hdma.XferHalfCpltCallback = DMA_Xfer_Half_Callback_Func;
    hdma.XferCpltCallback     = DMA_Xfer_Complete_Callback_Func;
    hdma.XferErrorCallback    = DMA_Xfer_Error_Callback_Func;

    err = HAL_DMA_Init(&hdma);
    if (err != HAL_OK)
        return err;

    err = HAL_DMA_Start_IT(hadc->DMA_Handle, SrcAddress, DstAddress, Counts);
    if (err != HAL_OK)
        return err;

```



## 使用 DMA示例3
内存->FLASH1模块 1字节传输 4096字节.
```c

    DMA_HandleTypeDef hdma;
    HAL_StatusTypeDef err;

    uint32_t SrcAddress = 0x20000000;
    uint32_t DstAddress = hflash->Instance->DR;
    uint32_t Counts = 4096;  //Transfer unit is byte, so Counts= 4096 / 1
    
    /* Init DMA configure*/
    hdma.Instance                 = DMA1_Channel6;
    hdma.Init.Request             = DMA_REQUEST_0;
    hdma.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma.Init.PeriphInc           = DMA_PINC_DISABLE;      //Periphral address NOT auto-increment
    hdma.Init.MemInc              = DMA_MINC_ENABLE;
    hdma.Init.Mode                = DMA_NORMAL;
    hdma.Init.Priority            = DMA_PRIORITY_MEDIUM;

    hdma.XferHalfCpltCallback = DMA_Xfer_Half_Callback_Func;
    hdma.XferCpltCallback = DMA_Xfer_Complete_Callback_Func;
    hdma.XferErrorCallback = DMA_Xfer_Error_Callback_Func;

    err = HAL_DMA_Init(&hdma);
    if (err != HAL_OK)
        return err;

    err = HAL_DMA_Start_IT(hadc->DMA_Handle, SrcAddress, DstAddress, Counts);
    if (err != HAL_OK)
        return err;

```

## API参考
[](/api/hal/crc.md)

