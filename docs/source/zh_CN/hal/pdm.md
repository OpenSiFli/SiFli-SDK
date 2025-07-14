# PDM

 PDM (Pulse Density Modulation)模块是用于将外部pdm麦克风输出的pdm信号进行滤波转化为PCM信号并提供给CPU进行后续的处理.

## 支持的配置
 - 单声道(左/右), 立体声
 - 采样率 8/12/16/24/32/48KHz
 - 24bit 采样深度
 - 左右声道 增益调节
 - 循环buffer/ 单次buffer


## 输出数据格式
  - 8bit  + 单声道
  - 16bit + 单声道
  - 32bit + 单声道 :  高8位为0, 实际有效数据为低24bit
  - 16bit + 双声道 :  左右声道各占16bit组成一个32bit数据(支持左右声道对换)
  - 32bit + 双声道 :  仅支持左右声道分开2个buffer存储


## 示例
PDM1单声道16KHz   , 16bit位深，用DMA读取1024字节数据

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


void pdm_recieve(void)
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


## API参考
[](../api/hal/pdm.md)

