# Audprc/Audcodec Audio设备
芯片内部有Audprc和Audcodec，Audprc用于数字处理，Audcodec是数字转模拟。
常用的音频数据流程是Audprc-->Audcodec-->Speaker
还有的方式是Audprc-->I2S-->芯片外围Codec芯片<br>
主要功能包括： <br>
- 输入和输出采样率设置
- 输入和输出通道数设置
- 每个采样的位深设置
- 音量大小的设置

播放的代码说明，具体例子在drv_audprc.c文件后面和example/rt_device/audprc里都有

```c
static rt_event_t g_tx_ev;

static rt_err_t speaker_tx_done(rt_device_t dev, void *buffer)
{
    //此函数在发送一帧完成的dma中断里，表示发送一次完成
    rt_event_send(g_tx_ev, 1);
    return RT_EOK;
}
#define DMA_BUF_SIZE    1600

void test_demo()
{
    g_tx_ev = rt_event_create("audio_tx_evt", RT_IPC_FLAG_FIFO);
    //1. 打开设备
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

    //2. 设置发送完成的回到函数
    rt_device_set_tx_complete(audprc_dev, speaker_tx_done);
    
    //3. 设置一次DMA buffer的大小，底层驱动里会使用2个这样的DMA buffer做ping/pong buffer
    rt_device_control(audprc_dev, AUDIO_CTL_SET_TX_DMA_SIZE, (void *)DMA_BUF_SIZE);

    //4. 音频输出到CODEC, 如果到I2S，可以设置AUDPRC_TX_TO_I2S
    rt_device_control(audcodec_dev, AUDIO_CTL_SETOUTPUT, (void *)AUDPRC_TX_TO_CODEC);
    rt_device_control(audprc_dev,   AUDIO_CTL_SETOUTPUT, (void *)AUDPRC_TX_TO_CODEC);
    
    //5. 设置codec参数
    struct rt_audio_caps caps;

    caps.main_type = AUDIO_TYPE_OUTPUT;
    caps.sub_type = (1 << HAL_AUDCODEC_DAC_CH0);
#if BSP_ENABLE_DAC2
    caps.sub_type |= (1 << HAL_AUDCODEC_DAC_CH1);
#endif
    caps.udata.config.channels   = 1; //最后的输出为一个声道
    caps.udata.config.samplerate = 16000; //采样率, 8000/11025/12000/16000/22050/24000/32000/44100/48000
    caps.udata.config.samplefmt = 16; //位深度8 16 24 or 32
    rt_device_control(audcodec_dev, AUDIO_CTL_CONFIGURE, &caps);
    
    struct rt_audio_sr_convert cfg;
    cfg.channel = 2; //源数据的通道个数，如果是2，则数据传入的格式位LRLRLR....的interleave格式
    cfg.source_sr = 16000; //源数据的采样率
    cfg.dest_sr = 16000;   //播放时的采样率  
    rt_device_control(audprc_dev, AUDIO_CTL_OUTPUTSRC, (void *)(&cfg));

    //数据选择，一路源数据就这样配置就行了，多路源数据的处理参考《音频通路mix&mux功能说明.docx》
    caps.main_type = AUDIO_TYPE_SELECTOR;
    caps.sub_type = 0xFF;
    caps.udata.value = 0x5050;
    rt_device_control(audprc_dev, AUDIO_CTL_CONFIGURE, &caps);
    caps.main_type = AUDIO_TYPE_MIXER;
    caps.sub_type = 0xFF;
    caps.udata.value   = 0x5050;
    rt_device_control(audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    //源数据格式说明
    caps.main_type = AUDIO_TYPE_OUTPUT;
    caps.sub_type = HAL_AUDPRC_TX_CH0;
    caps.udata.config.channels   = 2;
    caps.udata.config.samplerate = 16000;
    caps.udata.config.samplefmt = 16;
    rt_device_control(audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    //从EQ配置表中获得音量并设置到codec，vol_level为0 ~ 15
    uint8_t vol_level = 15;
    int volumex2 = eq_get_music_volumex2(vol_level);
    if (caps.udata.config.samplerate == 16000 || caps.udata.config.samplerate == 8000)
       volumex2 = eq_get_tel_volumex2(vol_level);
    rt_device_control(audcodec_dev, AUDIO_CTL_SETVOLUME, (void *)volumex2);

    //开始播放
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
        //为了更快，也可以用
        bf0_audprc_device_write(audprc_dev, 0, tx_pipe_data, DMA_BUF_SIZE);
#endif
    }
    rt_device_close(audcodec_dev);
    rt_device_close(audprc_dev);
    rt_event_delete(g_tx_ev);
}
```
接收的代码说明

```c
static rt_event_t g_rx_ev;
static rt_err_t mic_rx_ind(rt_device_t dev, rt_size_t size)
{
    //in inturrupt
    rt_event_send(g_rx_ev, 1);
    return RT_EOK;
}
void test_demo()
{
    g_rx_ev = rt_event_create("audio_evt", RT_IPC_FLAG_FIFO);
    //1. 打开设备
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
    
    // 设置接受一帧完成的回到函数
    rt_device_set_rx_indicate(audprc_dev, mic_rx_ind);

    //config ADC
    struct rt_audio_caps caps;
    int stream;

    //2. 设置输入源自audcodec-->audprc，如果是I2S-->audprc, 这设置为AUDPRC_RX_FROM_I2S
    rt_device_control(audcodec_dev, AUDIO_CTL_SETINPUT, (void *)AUDPRC_RX_FROM_CODEC);
    caps.main_type = AUDIO_TYPE_INPUT;
    caps.sub_type = 1 << HAL_AUDCODEC_ADC_CH0;
    caps.udata.config.channels   = 1; //一个声道
    caps.udata.config.samplerate = 16000; //采样率
    caps.udata.config.samplefmt = 16; // 位深 8 16 24 or 32
    rt_device_control(audcodec_dev, AUDIO_CTL_CONFIGURE, &caps);
    rt_device_control(audprc_dev, AUDIO_CTL_SETINPUT, (void *)AUDPRC_RX_FROM_CODEC);

    caps.main_type = AUDIO_TYPE_INPUT;
    caps.sub_type = HAL_AUDPRC_RX_CH0 - HAL_AUDPRC_RX_CH0; //来自RX0
    caps.udata.config.channels   = 1;
    caps.udata.config.samplerate = 16000;
    caps.udata.config.samplefmt = 16;
    rt_device_control(audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    //3. EQ相关的在drv_audprc.c前面那段代码，那个可以用EQ工具生成的，里面g_adc_volume为mic的增益
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
## 驱动配置
在{menuselection}打开
`On-Chip Peripheral RTOS Drivers --> Enable Audio codec Driver`
`On-Chip Peripheral RTOS Drivers --> Enable Audio Process driver --> Enable AUDPRC TX Channel 0 DMA`
`On-Chip Peripheral RTOS Drivers --> Enable Audio Process driver --> Enable AUDPRC RX Channel 0 DMA`

## 使用audprc/audcodec录音再播放的例子

# I2S Audio设备

音频驱动程序包括两层：用于 I2S 的硬件访问层 (HAL) 和用于 RT-Thread 的适配层。<br>
硬件访问层提供用于访问 I2S 外设寄存器的基本 API。 有关详细信息，请参阅 I2S 的 API 文档。<br>
适配层提供对 RT-Thread 驱动框架的支持。 用户可以使用 RT-Thread POSIX 驱动程序接口进行音频编程。 请参阅 RT-Thread 驱动程序的 API 文档。<br>
主要功能包括： <br>
- 麦克风设备和扬声器设备支持
- 用于音频捕获和播放的 DMA
- 音频捕获转储工具支持并保存在 PC 中
- 两路I2S硬件支持，其中I2S1只用来输入， I2S2 既支持输入也支持输出

## 驱动配置

在{menuselection}`On-Chip Peripheral RTOS Drivers --> Enable I2S Audio Driver`菜单中选择要使用的I2S设备

下面宏开关表示使能了I2S_MIC和I2S_CODEC两个设备
```c
#define BSP_USING_DMA 1
#define RT_USING_AUDIO 1
#define BSP_USING_I2S 1
#define BSP_ENABLE_I2S_MIC 1
#define BSP_ENABLE_I2S_CODEC 1
```

## 设备名称
- `i2s<x>`，
其中x为设备编号，如`i2s1`、`i2s2`，与操作的外设编号对应


## 使用音频驱动程序

适配器层注册 RT-Thread 请求的硬件支持功能，并使用 I2S HAL 实现这些功能。 I2S HAL 的 API 详见 [I2S](#hal-i2s)

## 对于使用 RT-Thread 麦克风设备进行音频捕获的用户，可以使用以下代码作为示例：

```c

uint8_t g_pipe_data[512];

// Find and open device
rt_device_t g_mic = rt_device_find("i2s1");
rt_err_t err = rt_device_open(g_mic, RT_DEVICE_FLAG_RDONLY);

// Configure Microphone deivce, sample rate 16000
struct rt_audio_caps caps;
caps.main_type = AUDIO_TYPE_INPUT;
caps.sub_type = AUDIO_DSP_SAMPLERATE;
caps.udata.value =16000;
rt_device_control(g_mic, AUDIO_CTL_CONFIGURE, &caps);;

// Start capture
int stream = 1; // record = 1, playback = 0
rt_device_set_rx_indicate(g_mic, audio_rx_ind);
rt_device_control(g_mic, AUDIO_CTL_START, &stream);


...

rt_err_t audio_rx_ind(rt_device_t dev, rt_size_t size)
{
    // Processing audio data. Please note this is in interrupt context.
    // User might need to start a thread to read and process data, call  rt_device_read(g_mic, 0, g_pipe_data, AUDIO_BUF_SIZE);
}

```

## 对于使用 RT-Thread 喇叭/耳机设备进行音频播放的用户，可以使用以下代码作为示例：

```c

uint8_t g_pipe_data[512];

// Find and open device
rt_device_t g_i2s = rt_device_find("i2s2");
rt_err_t err = rt_device_open(g_i2s, RT_DEVICE_FLAG_RDWR);


// Configure speaker deivce, sample rate 16000
struct rt_audio_caps caps;
caps.main_type = AUDIO_TYPE_INPUT;  //AUDIO_TYPE_OUTPUT// for I2S2, configure RX will configure RX+TX
caps.sub_type = AUDIO_DSP_SAMPLERATE;
caps.udata.value =16000;
rt_device_control(g_i2s, AUDIO_CTL_CONFIGURE, &caps);;

// Start capture
int stream = 0; // record = 1, playback = 0
rt_device_set_tx_complete(g_i2s, audio_tx_done);
rt_device_control(g_i2s, AUDIO_CTL_START, &stream);


...

rt_err_t audio_tx_done(rt_device_t dev, void *buffer)
{
    // Processing audio data. Please note this is in interrupt context.
    // User might need to start a thread to fill data, call  rt_device_write(g_i2s, 0, g_pipe_data, AUDIO_BUF_SIZE)
}

```

[audio]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/audio/audio
## RT-Thread参考文档

- [AUDIO设备][audio]

