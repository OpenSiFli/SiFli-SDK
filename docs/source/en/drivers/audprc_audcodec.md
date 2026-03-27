# Audprc/Audcodec Audio Device
The chip contains Audprc and Audcodec. Audprc is used for digital processing,
while Audcodec is for digital-to-analog conversion. The common audio data flow
is Audprc --> Audcodec --> Speaker. Another possible configuration is Audprc -->
I2S --> External Codec chip. Main functionalities include: <br>
- Input and output sample rate settings
- Input and output channel count settings
- Bit depth setting for each sample
- Volume control settings

The playback code explanation, with specific examples, can be found in the
`drv_audprc.c` file and the `example/rt_device/audprc` directory.

```c
static rt_event_t g_tx_ev;

static rt_err_t speaker_tx_done(rt_device_t dev, void *buffer)
{
    // This function is called when one frame of data has been sent, through a DMA interrupt.
    rt_event_send(g_tx_ev, 1);
    return RT_EOK;
}
#define DMA_BUF_SIZE    1600

void test_demo()
{
    g_tx_ev = rt_event_create("audio_tx_evt", RT_IPC_FLAG_FIFO);
    // 1. Open devices
    int err;
    rt_device_t audprc_dev;
    rt_device_t audcodec_dev;
    audprc_dev = rt_device_find("audprc");
    audcodec_dev = rt_device_find("audcodec");
    RT_ASSERT(audprc_dev && audcodec_dev);

    err = rt_device_open(audprc_dev, RT_DEVICE_FLAG_RDWR);
    RT_ASSERT(RT_EOK == err);
    err = rt_device_open(audcodec_dev, RT_DEVICE_FLAG_WRONLY);
    RT_ASSERT(RT_EOK == err);

    // 2. Set the callback for transmission completion
    rt_device_set_tx_complete(audprc_dev, speaker_tx_done);

    // 3. Set the size of one DMA buffer, the underlying driver uses two such buffers for ping-pong buffering
    rt_device_control(audprc_dev, AUDIO_CTL_SET_TX_DMA_SIZE, (void *)DMA_BUF_SIZE);

    // 4. Output audio to the codec, if using I2S, set AUDPRC_TX_TO_I2S
    rt_device_control(audcodec_dev, AUDIO_CTL_SETOUTPUT, (void *)AUDPRC_TX_TO_CODEC);
    rt_device_control(audprc_dev,   AUDIO_CTL_SETOUTPUT, (void *)AUDPRC_TX_TO_CODEC);

    // 5. Configure codec parameters
    struct rt_audio_caps caps;

    caps.main_type = AUDIO_TYPE_OUTPUT;
    caps.sub_type = (1 << HAL_AUDCODEC_DAC_CH0);
#if BSP_ENABLE_DAC2
    caps.sub_type |= (1 << HAL_AUDCODEC_DAC_CH1);
#endif
    caps.udata.config.channels   = 1; // Final output with one channel
    caps.udata.config.samplerate = 16000; // Sample rate, options: 8000/11025/12000/16000/22050/24000/32000/44100/48000
    caps.udata.config.samplefmt = 16; // Bit depth: 8, 16, 24, or 32
    rt_device_control(audcodec_dev, AUDIO_CTL_CONFIGURE, &caps);

    struct rt_audio_sr_convert cfg;
    cfg.channel = 2; // Source data channels, if 2, data will be in interleave format: LRLRLR...
    cfg.source_sr = 16000; // Source data sample rate
    cfg.dest_sr = 16000;   // Output sample rate  
    rt_device_control(audprc_dev, AUDIO_CTL_OUTPUTSRC, (void *)(&cfg));

    // Data selection. For a single source, this configuration is sufficient. For multiple sources, refer to the "Audio Path Mix & Mux Function Explanation" document.
    caps.main_type = AUDIO_TYPE_SELECTOR;
    caps.sub_type = 0xFF;
    caps.udata.value = 0x5050;
    rt_device_control(audprc_dev, AUDIO_CTL_CONFIGURE, &caps);
    caps.main_type = AUDIO_TYPE_MIXER;
    caps.sub_type = 0xFF;
    caps.udata.value   = 0x5050;
    rt_device_control(audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    // Source data format specification
    caps.main_type = AUDIO_TYPE_OUTPUT;
    caps.sub_type = HAL_AUDPRC_TX_CH0;
    caps.udata.config.channels   = 2;
    caps.udata.config.samplerate = 16000;
    caps.udata.config.samplefmt = 16;
    rt_device_control(audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    // Set the volume from the EQ configuration table to the codec. vol_level ranges from 0 to 15
    uint8_t vol_level = 15;
    int volumex2 = eq_get_music_volumex2(vol_level);
    if (caps.udata.config.samplerate == 16000 || caps.udata.config.samplerate == 8000)
       volumex2 = eq_get_tel_volumex2(vol_level);
    rt_device_control(audcodec_dev, AUDIO_CTL_SETVOLUME, (void *)volumex2);

    // Start playback
    int stream = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDPRC_TX_CH0) << 8);
    rt_device_control(audprc_dev, AUDIO_CTL_START, (void *)&stream);
    stream = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
    rt_device_control(audcodec_dev, AUDIO_CTL_START, &stream);
    rt_uint32_t evt;
    int16_t tx_pipe_data[4096];
    while (1)
    {
        rt_event_recv(g_tx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
#if 1
        rt_device_write(audprc_dev, 0, tx_pipe_data, DMA_BUF_SIZE);
#else
        // Faster alternative:
        bf0_audprc_device_write(audprc_dev, 0, tx_pipe_data, DMA_BUF_SIZE);
#endif
    }
    rt_device_close(audcodec_dev);
    rt_device_close(audprc_dev);
    rt_event_delete(g_tx_ev);
}
```
Receiving Code Explanation:

```c
static rt_event_t g_rx_ev;
static rt_err_t mic_rx_ind(rt_device_t dev, rt_size_t size)
{
    // Called in interrupt context
    rt_event_send(g_rx_ev, 1);
    return RT_EOK;
}
void test_demo()
{
    g_rx_ev = rt_event_create("audio_evt", RT_IPC_FLAG_FIFO);
    // 1. Open devices
    int err;
    rt_device_t audprc_dev;
    rt_device_t audcodec_dev;
    audprc_dev = rt_device_find("audprc");
    audcodec_dev = rt_device_find("audcodec");
    RT_ASSERT(audprc_dev && audcodec_dev);

    err = rt_device_open(audprc_dev, RT_DEVICE_FLAG_RDWR);
    RT_ASSERT(RT_EOK == err);
    err = rt_device_open(audcodec_dev, RT_DEVICE_FLAG_WRONLY);
    RT_ASSERT(RT_EOK == err);

    // Set the callback for reception of one frame
    rt_device_set_rx_indicate(audprc_dev, mic_rx_ind);

    // Configure ADC
    struct rt_audio_caps caps;
    int stream;

    // 2. Set input source to audcodec --> audprc. If using I2S, set to AUDPRC_RX_FROM_I2S
    rt_device_control(audcodec_dev, AUDIO_CTL_SETINPUT, (void *)AUDPRC_RX_FROM_CODEC);
    caps.main_type = AUDIO_TYPE_INPUT;
    caps.sub_type = 1 << HAL_AUDCODEC_ADC_CH0;
    caps.udata.config.channels   = 1; // One channel
    caps.udata.config.samplerate = 16000; // Sample rate
    caps.udata.config.samplefmt = 16; // Bit depth: 8, 16, 24, or 32
    rt_device_control(audcodec_dev, AUDIO_CTL_CONFIGURE, &caps);
    rt_device_control(audprc_dev, AUDIO_CTL_SETINPUT, (void *)AUDPRC_RX_FROM_CODEC);

    caps.main_type = AUDIO_TYPE_INPUT;
    caps.sub_type = HAL_AUDPRC_RX_CH0 - HAL_AUDPRC_RX_CH0; // From RX0
    caps.udata.config.channels   = 1;
    caps.udata.config.samplerate = 16000;
    caps.udata.config.samplefmt = 16;
    rt_device_control(audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    // 3. EQ-related code in drv_audprc.c. You can use the EQ tool to generate this.
    int stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDCODEC_ADC_CH0) << 8);
    rt_device_control(audcodec_dev, AUDIO_CTL_START, &stream);
    stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDPRC_RX_CH0) << 8);
    rt_device_control(audprc_dev, AUDIO_CTL_START, &stream);
    rt_uint32_t evt;

    uint8_t g_pipe_data[CFG_AUDIO_RECORD_PIPE_SIZE];
    while (1)
    {
        rt_event_recv(g_rx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        rt_device_read(audprc_dev, 0, g_pipe_data, CFG_AUDIO_RECORD_PIPE_SIZE);
    }
    rt_device_close(audcodec_dev);
    rt_device_close(audprc_dev);
    rt_event_delete(g_rx_ev);
}
```
## Driver Configuration
Enable in {menuselection}:

[audio]:
https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/audio/audio
## RT-Thread Documentation

- [AUDIO Device Documentation][audio]

