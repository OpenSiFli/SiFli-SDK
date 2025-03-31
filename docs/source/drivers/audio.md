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

