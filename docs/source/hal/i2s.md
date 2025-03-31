# I2S

I2S HAL 提供用于访问 I2S 外设寄存器的基本 API。 SF32LB55X有两个实例I2S1/I2S2（都在HPSYS），SF32LB58X有三个实例I2S1/I2S2/I2S3（I2S1和I2S2在HPSYS,I2S3在LPSYS）。 
对于 I2S1，它只支持 RX 功能，
对于 I2S2/I2S3，它支持 RX 和 TX。 当使用 I2S2 RX 时，它的时钟来自它的 TX，所以如果使用 I2S2 作为 RX，它的 TX 也需要使能提供时钟。
SF32LB56X只有一个全功能的I2S1，相当于SF32LB58X的I2S2，支持RX和TX。
SF32LB55X的时钟只能是xtal48M，SF32LB58X和SF32LB56X的时钟可以是xtal48M或PLL。
```{note}
SF32LB52X与SF32LB56X相同。
```

主要功能包括：
- 多个实例支持。
- 接收/发送支持。
- DMA/中断模式支持。
- 主/从模式支持。

## 使用 I2S HAL 驱动程序
以下以全功能的I2S为例进行描述，当I2S作为maser模式时，RX的时钟是由TX进行提供的，因此tx_cfg中的slave_mode要配置成0，rx_cfg中的slave_mode配置成1。
当I2S作为SLAVE模式时，时钟是由外部的I2S提供，因此tx_cfg和rx_cfg中的slave_mode都要配置成1。
使用xtal48M作为时钟源时，bclk和lrclk的时序可能无法满足对方I2S的需求（这个需要注意），使用PLL作为时钟源时，需要打开audcodec模块，并且保证所有音频模块的时钟一致。
可以参考drv_i2s中的实现。

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

## API参考
[](#hal-i2s)