# PDM

The PDM (Pulse Density Modulation) module is used to filter PDM signals output from external PDM microphones, convert them to PCM signals, and provide them to the CPU for subsequent processing.

## Supported Configurations
 - Mono (left/right), stereo
 - Sample rates: 8/12/16/24/32/48KHz
 - 24-bit sampling depth
 - Left and right channel gain adjustment
 - Circular buffer/single buffer

## Output Data Formats
  - 8-bit + mono
  - 16-bit + mono
  - 32-bit + mono: Upper 8 bits are 0, actual valid data is lower 24 bits
  - 16-bit + stereo: Left and right channels each occupy 16 bits forming one 32-bit data (supports left/right channel swapping)
  - 32-bit + stereo: Only supports left and right channels stored in separate 2 buffers

## Example
PDM1 mono 16KHz, 16-bit depth, using DMA to read 1024 bytes of data

```c
static PDM_HandleTypeDef PDM_Handle = {0};
static DMA_HandleTypeDef DMA_Handle = {0};
static uint32_t DMA_cplt_flag = 0;

void DMAC1_CH3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(PDM_Handle.hdmarx);
}

void HAL_PDM_RxCpltCallback(PDM_HandleTypeDef *hpdm)
{
    if(hpdm == &PDM_Handle)
        DMA_cplt_flag = 1;
}

void pdm_receive(void)
{
    DMA_Handle.Instance = DMA1_Channel3;
    DMA_Handle.Init.Request = DMA_REQUEST_18;

    PDM_Handle.hdmarx = &DMA_Handle;
    PDM_Handle.Instance = hwp_pdm1;
    PDM_Handle.Init.Mode = PDM_MODE_LOOP; //Ring buffer mode
    PDM_Handle.Init.Channels = PDM_CHANNEL_LEFT_ONLY;
    PDM_Handle.Init.SampleRate = PDM_SAMPLE_16KHZ;
    PDM_Handle.Init.ChannelDepth = PDM_CHANNEL_DEPTH_16BIT;
    PDM_Handle.RxXferSize = 1024;
    PDM_Handle.pRxBuffPtr = (uint8_t *) 0x2000c000;

    HAL_PDM_Init(&PDM_Handle);
    HAL_NVIC_EnableIRQ(DMAC1_CH3_IRQn);

    HAL_PDM_Receive_DMA(&PDM_Handle, PDM_Handle.pRxBuffPtr, PDM_Handle.RxXferSize);

    while(DMA_cplt_flag == 0);

    HAL_NVIC_DisableIRQ(DMAC1_CH3_IRQn);
    HAL_PDM_DMAStop(&PDM_Handle);
    HAL_PDM_DeInit(&PDM_Handle);
}
...
```

## API Reference
[](../api/hal/pdm.md)
