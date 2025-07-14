
# I2S

I2S HAL provides basic APIs for accessing I2S peripheral registers. SF32LB55X has two instances I2S1/I2S2 (both in HPSYS), SF32LB58X has three instances I2S1/I2S2/I2S3 (I2S1 and I2S2 in HPSYS, I2S3 in LPSYS). 
For I2S1, it only supports RX functionality,
For I2S2/I2S3, it supports both RX and TX. When using I2S2 RX, its clock comes from its TX, so if using I2S2 as RX, its TX also needs to be enabled to provide the clock.
SF32LB56X has only one full-featured I2S1, equivalent to I2S2 of SF32LB58X, supporting both RX and TX.
The clock of SF32LB55X can only be xtal48M, while SF32LB58X and SF32LB56X clocks can be either xtal48M or PLL.
```{note}
SF32LB52X is the same as SF32LB56X.
```

Main features include:
- Multiple instance support.
- Receive/transmit support.
- DMA/interrupt mode support.
- Master/slave mode support.

## Using I2S HAL Driver
The following description uses a full-featured I2S as an example. When I2S is in master mode, the RX clock is provided by TX, so slave_mode in tx_cfg should be configured as 0, and slave_mode in rx_cfg should be configured as 1.
When I2S is in SLAVE mode, the clock is provided by external I2S, so slave_mode in both tx_cfg and rx_cfg should be configured as 1.
When using xtal48M as clock source, the bclk and lrclk timing may not meet the requirements of the peer I2S (this needs attention). When using PLL as clock source, the audcodec module needs to be enabled, and all audio modules must have consistent clocks.
You can refer to the implementation in drv_i2s.

```c
    #define CLOCK_USING_XTAL 0  //PLL clock need open audcodec
#if CLOCK_USING_XTAL //crystal
static CLK_DIV_T  txrx_clk_div[9]  = {{48000, 125, 125,  5}, {44100, 136, 136,  4}, {32000, 185, 190,  5}, {24000, 250, 250, 10}, {22050, 272, 272,  8},
    {16000, 384, 384, 12}, {12000, 500, 500, 20}, {11025, 544, 544, 16}, { 8000, 750, 750, 30}
};
#else  //PLL
//PLL 16k 49.152M  44.1k  45.1584M
//lrclk_duty_high:PLL/spclk_div/samplerate/2: 64=49.152M/48k/8/2
//bclk:lrclk_duty_high/32
static CLK_DIV_T  txrx_clk_div[9]  = {{48000, 64, 64,  2}, {44100, 64, 64,  2}, {32000, 96, 96,  3}, {24000, 128, 128, 4}, {22050, 128, 128,  4},
    {16000, 192, 192, 6}, {12000, 256, 256, 8}, {11025, 256, 256, 8}, { 8000, 384, 384, 12}
};
#endif
    /* initial I2S controller */
    #define EXAMPLE_I2S_TRANS_SIZE          (480)
    static I2S_HandleTypeDef i2s_handle;
    static uint8_t pData[EXAMPLE_I2S_TRANS_SIZE];

    I2S_HandleTypeDef *hi2s = &i2s_handle;
    HAL_StatusTypeDef ret;

    hi2s->Instance = hwp_i2s2;
    HAL_RCC_EnableModule(RCC_MOD_I2S2);

    /* Initial tx configure*/
    hi2s->Init.tx_cfg.data_dw = 16; // bit width 16
    hi2s->Init.tx_cfg.pcm_dw = 16;
    hi2s->Init.tx_cfg.bus_dw = 32;
    hi2s->Init.tx_cfg.slave_mode = 0;   // master mode
    hi2s->Init.tx_cfg.track = 0;        // default stereo
    hi2s->Init.tx_cfg.vol = 4;     // default set to mute(15) or 0 db (4)
    hi2s->Init.tx_cfg.balance_en = 0;
    hi2s->Init.tx_cfg.balance_vol = 0;
    hi2s->Init.tx_cfg.chnl_sel = 0;
    hi2s->Init.tx_cfg.lrck_invert = 0;
    hi2s->Init.tx_cfg.sample_rate = 16000;
    hi2s->Init.tx_cfg.extern_intf = 0;
    hi2s->Init.tx_cfg.clk_div_index = 5;//for 16k samplerate
    hi2s->Init.tx_cfg.clk_div = &txrx_clk_div[hi2s->Init.tx_cfg.clk_div_index];

    /* Initial rx configure*/
    hi2s->Init.rx_cfg.data_dw = 16;
    hi2s->Init.rx_cfg.pcm_dw = 16;
    hi2s->Init.rx_cfg.bus_dw = 32;
    hi2s->Init.rx_cfg.slave_mode = 1;   // slave mode
    hi2s->Init.rx_cfg.chnl_sel = 0;     // left/right all set to left
    hi2s->Init.rx_cfg.sample_rate = 16000;
    hi2s->Init.rx_cfg.chnl_sel = 0;        // default stereo
    hi2s->Init.rx_cfg.lrck_invert = 0;
    hi2s->Init.rx_cfg.clk_div_index = 5;//for 16k samplerate
    hi2s->Init.rx_cfg.clk_div = &txrx_clk_div[hi2s->Init.rx_cfg.clk_div_index];
            

#if CLOCK_USING_XTAL
    __HAL_I2S_CLK_XTAL(hi2s);   // xtal use 48M for asic
    __HAL_I2S_SET_SPCLK_DIV(hi2s, 4);   // set to 12M to i2s
#else
    __HAL_I2S_CLK_PLL(hi2s); //PLL
    __HAL_I2S_SET_SPCLK_DIV(hi2s, 8);   // set to 6.144M to i2s   PLL
    bf0_enable_pll(hi2s->Init.tx_cfg.sample_rate, 0);
#endif

    /*Initial I2S controller */
    HAL_I2S_Init(hi2s);

    /*Start I2S TX test */
    /* reconfigure I2S TX before start if any changed*/
    HAL_I2S_Config_Transmit(hi2s, &(hi2s->Init.tx_cfg));

    /* Start I2S transmit with polling mode */
    ret = HAL_I2S_Transmit(hi2s, pData, EXAMPLE_I2S_TRANS_SIZE, 100);
    /*End I2S TX test */

    /*Start I2S RX test */
    /* reconfigure I2S RX before start if any changed*/
    HAL_I2S_Config_Receive(hi2s, &(hi2s->Init.rx_cfg));

    /* For I2S2, RX clock from TX, so need enable TX when start RX */
    __HAL_I2S_TX_ENABLE(hi2s);
    /* Start I2S Receive with polling mode */
    ret = HAL_I2S_Receive(hi2s, pData, EXAMPLE_I2S_TRANS_SIZE, 100);
    
    /*Check received data */
    // pData 
```

## API Reference
[](#hal-i2s)
