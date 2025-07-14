# AUDPRC

AUDPRC (audio processor) completes specific data processing functions for audio paths. Main functions include:\
- Audio sampling rate conversion\
- Audio equalizer EQ for matching output audio devices\
- Audio mixing function, mixing audio from different sources for output, supports mixing audio of different sampling rates\
- Supports data input/output from I2S/MEM/AUDCODEC\
- Supports mono/stereo various data formats, including interleaving and de-interleaving of left and right channel data

## Using AUDPRC HAL Driver:
HAL AUDPRC sample for TX:

```c

const audprc_src_table_t src_table[] =
{
    {8000,  11025, 0, 0, 0, 0, 0, 0, 1, 0x2e709c60},
    {8000,  12000, 0, 0, 0, 0, 0, 0, 1, 0x2aaaaaa8},
    {8000,  16000, 1, 0, 0, 0, 0, 0, 0, 0},
    {8000,  22050, 1, 0, 0, 0, 0, 0, 1, 0x2e706d80},
    {8000,  24000, 1, 0, 1, 0, 0, 0, 1, 0x55553700},
    {8000,  32000, 1, 0, 1, 0, 0, 0, 0, 0},
    {8000,  44100, 1, 0, 1, 0, 0, 0, 1, 0x2e702f00},
    {8000,  48000, 1, 0, 1, 0, 0, 0, 1, 0x2aaa5d00},
    {11025,   8000, 0, 0, 0, 0, 0, 0, 1, 0x583313d9},
    {11025, 12000, 0, 0, 0, 0, 0, 0, 1, 0x3accb7e6},
    {11025, 16000, 0, 0, 0, 0, 0, 0, 1, 0x2c197464},
    {11025, 22050, 1, 0, 0, 0, 0, 0, 0, 0},
    {11025, 24000, 1, 0, 0, 0, 0, 0, 1, 0x3accb7e6},
    {11025, 32000, 1, 0, 0, 0, 0, 0, 1, 0x2c197464},
    {11025, 44100, 1, 0, 1, 0, 0, 0, 0, 0},
    {11025, 48000, 1, 0, 1, 0, 0, 0, 1, 0x3acc61c4},
    {12000,   8000, 1, 1, 0, 0, 0, 0, 1, 0x2fffeef0},
    {12000, 11025, 0, 0, 0, 0, 0, 0, 1, 0x45a8d320},
    {12000, 16000, 0, 0, 0, 0, 0, 0, 1, 0x2fffd780},
    {12000, 22050, 1, 0, 0, 0, 0, 0, 1, 0x45a8a440},
    {12000, 24000, 1, 0, 0, 0, 0, 0, 0, 0},
    {12000, 32000, 1, 0, 0, 0, 0, 0, 1, 0x2fffd780},
    {12000, 44100, 1, 0, 1, 0, 0, 0, 1, 0x45a84680},
    {12000, 48000, 1, 0, 1, 0, 0, 0, 0, 0},
    {16000,   8000, 1, 1, 0, 0, 0, 0, 0, 0},
    {16000, 11025, 0, 0, 0, 0, 0, 0, 1, 0x5ce11980},
    {16000, 12000, 0, 0, 0, 0, 0, 0, 1, 0x55553700},
    {16000, 22050, 0, 0, 0, 0, 0, 0, 1, 0x2e706d80},
    {16000, 24000, 0, 0, 0, 0, 0, 0, 1, 0x2aaa9b80},
    {16000, 32000, 1, 0, 0, 0, 0, 0, 0, 0},
    {16000, 44100, 1, 0, 0, 0, 0, 0, 1, 0x2e702f00},
    {16000, 48000, 1, 0, 0, 0, 0, 0, 1, 0x2aaa5d00},
    {22050,   8000, 1, 1, 0, 0, 0, 0, 1, 0x583313d9},
    {22050, 11025, 1, 1, 0, 0, 0, 0, 0, 0},
    {22050, 12000, 1, 1, 0, 0, 0, 0, 1, 0x3accb7e6},
    {22050, 16000, 0, 0, 0, 0, 0, 0, 1, 0x5832e8c8},
    {22050, 24000, 0, 0, 0, 0, 0, 0, 1, 0x3accb7e6},
    {22050, 32000, 0, 0, 0, 0, 0, 0, 1, 0x2c197464},
    {22050, 44100, 1, 0, 0, 0, 0, 0, 0, 0},
    {22050, 48000, 1, 0, 0, 0, 0, 0, 1, 0x3acc61c4},
    {24000,   8000, 1, 1, 0, 0, 0, 0, 1, 0x5fffdde0},
    {24000, 11025, 1, 1, 0, 0, 0, 0, 1, 0x45a8d320},
    {24000, 12000, 1, 1, 0, 0, 0, 0, 0, 0},
    {24000, 16000, 1, 1, 0, 0, 0, 0, 1, 0x2fffd780},
    {24000, 22050, 0, 0, 0, 0, 0, 0, 1, 0x45a8a440},
    {24000, 32000, 0, 0, 0, 0, 0, 0, 1, 0x2fffd780},
    {24000, 44100, 1, 0, 0, 0, 0, 0, 1, 0x45a84680},
    {24000, 48000, 1, 0, 0, 0, 0, 0, 0, 0},
    {32000,   8000, 1, 1, 1, 1, 0, 0, 0, 0},
    {32000, 11025, 1, 1, 0, 0, 0, 0, 1, 0x5ce11980},
    {32000, 12000, 1, 1, 0, 0, 0, 0, 1, 0x55553700},
    {32000, 16000, 1, 1, 0, 0, 0, 0, 0, 0},
    {32000, 22050, 1, 1, 0, 0, 0, 0, 1, 0x2e706d80},
    {32000, 24000, 0, 0, 0, 0, 0, 0, 1, 0x55553700},
    {32000, 44100, 0, 0, 0, 0, 0, 0, 1, 0x2e702f00},
    {32000, 48000, 0, 0, 0, 0, 0, 0, 1, 0x2aaa5d00},
    {44100,   8000, 1, 1, 1, 1, 0, 0, 1, 0x583313d9},
    {44100, 11025, 1, 1, 1, 1, 0, 0, 0, 0},
    {44100, 12000, 1, 1, 1, 1, 0, 0, 1, 0x3accb7e6},
    {44100, 16000, 1, 1, 0, 0, 0, 0, 1, 0x5832e8c8},
    {44100, 22050, 1, 1, 0, 0, 0, 0, 0, 0},
    {44100, 24000, 1, 1, 0, 0, 0, 0, 1, 0x3accb7e6},
    {44100, 32000, 0, 0, 0, 0, 0, 0, 1, 0x5832e8c8},
    {44100, 48000, 0, 0, 0, 0, 0, 0, 1, 0x3acc61c4},
    {48000,   8000, 1, 1, 1, 1, 1, 1, 1, 0x2fffeef0},
    {48000, 11025, 1, 1, 1, 1, 0, 0, 1, 0x45a8d320},
    {48000, 12000, 1, 1, 1, 1, 0, 0, 0, 0},
    {48000, 16000, 1, 1, 0, 0, 0, 0, 1, 0x5fffaf00},
    {48000, 22050, 1, 1, 0, 0, 0, 0, 1, 0x45a8a440},
    {48000, 24000, 1, 1, 0, 0, 0, 0, 0, 0},
    {48000, 32000, 1, 1, 0, 0, 0, 0, 1, 0x2fffd780},
    {48000, 44100, 0, 0, 0, 0, 0, 0, 1, 0x45a84680},
};
AUDPRC_HandleTypeDef *haprc = &aprc;
AUDPRC_ChnlCfgTypeDef cfg;

int res = HAL_AUDPRC_Init(haprc);

cfg.dma_mask = 0;
cfg.en = 1;
cfg.format = caps->udata.config.samplefmt == 16 ? 0 : 1;
if (cfg.format == 0) // only 16 bit support stereo
    cfg.mode = caps->udata.config.channels == 1 ? 0 : 1;
else
    cfg.mode = 0;
HAL_AUDPRC_Config_TChanel(haudprc, 3, &cfg);

haudprc->Init.dac_cfg.src_hbf3_mode = src_table[i].hbf3_mode;
haudprc->Init.dac_cfg.src_hbf3_en = src_table[i].hbf3_en;
haudprc->Init.dac_cfg.src_hbf2_mode = src_table[i].hbf2_mode;
haudprc->Init.dac_cfg.src_hbf2_en = src_table[i].hbf2_en;
haudprc->Init.dac_cfg.src_hbf1_mode = src_table[i].hbf1_mode;
haudprc->Init.dac_cfg.src_hbf1_en = src_table[i].hbf1_en;
haudprc->Init.dac_cfg.src_sinc_en = src_table[i].sinc_en;
haudprc->Init.dac_cfg.sinc_ratio = src_table[i].sinc_ratio;
haudprc->Init.dac_cfg.src_ch_en = 3;
HAL_AUDPRC_Config_DACPath(haudprc, &(haudprc->Init.dac_cfg));

res = HAL_AUDPRC_Transmit_DMA(haudprc, haudprc->buf[HAL_AUDPRC_TX_CH0], haudprc->bufSize, HAL_AUDPRC_TX_CH0);
HAL_NVIC_EnableIRQ(AUDPRC_TX0_DMA_IRQ);

/* enable AUDPRC at last*/
__HAL_AUDPRC_ENABLE(haudprc);
    
```
HAL AUDPRC sample for RX:

```c
AUDPRC_HandleTypeDef haprc;
AUDPRC_ChnlCfgTypeDef cfg;

int res = HAL_AUDPRC_Init(haprc);

cfg.dma_mask = 0;
cfg.en = 1;
cfg.format = caps->udata.config.samplefmt == 16 ? 0 : 1;
if (cfg.format == 0) // only 16 bit support stereo
    cfg.mode = caps->udata.config.channels == 1 ? 0 : 1;
else
    cfg.mode = 0;
HAL_AUDPRC_Config_RChanel(haudprc, 0, &cfg);

haudprc->Init.adc_cfg.src_hbf3_mode = src_table[i].hbf3_mode;
haudprc->Init.adc_cfg.src_hbf3_en = src_table[i].hbf3_en;
haudprc->Init.adc_cfg.src_hbf2_mode = src_table[i].hbf2_mode;
haudprc->Init.adc_cfg.src_hbf2_en = src_table[i].hbf2_en;
haudprc->Init.adc_cfg.src_hbf1_mode = src_table[i].hbf1_mode;
haudprc->Init.adc_cfg.src_hbf1_en = src_table[i].hbf1_en;
haudprc->Init.adc_cfg.src_sinc_en = src_table[i].sinc_en;
haudprc->Init.adc_cfg.sinc_ratio = src_table[i].sinc_ratio;
haudprc->Init.adc_cfg.src_ch_en = 3;
HAL_AUDPRC_Config_ADCPath(haudprc, &(haudprc->Init.adc_cfg));

res = HAL_AUDPRC_Receive_DMA(haudprc, haudprc->buf[HAL_AUDPRC_RX_CH0], haudprc->bufSize, HAL_AUDPRC_RX_CH0);
HAL_NVIC_EnableIRQ(AUDPRC_RX0_DMA_IRQ);

/* enable AUDPRC at last*/
__HAL_AUDPRC_ENABLE(haudprc);
    
```

## API Reference
[](/api/hal/audprc.md)