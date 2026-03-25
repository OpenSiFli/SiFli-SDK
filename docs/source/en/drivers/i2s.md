# I2S Audio Device

- i2s samplerate
- i2s channel
- i2s sample bitwidth
- i2s slave/master select

The audio driver includes two layers: a hardware access layer (HAL) for I2S and
an adapter layer for RT-Thread. The hardware access layer provides basic APIs
for accessing I2S peripheral registers. Detailed information is available in the
I2S API documentation. The adapter layer supports the RT-Thread driver
framework, enabling audio programming via RT-Thread POSIX driver interfaces.
Refer to the RT-Thread driver API documentation for details. Main
functionalities include: <br>
- Microphone and speaker device support
- DMA for audio capture and playback
- The audio capture dump tool is supported and saves data to the PC.
- Some chips feature multiple I2S hardware instances. On the 58x series, I2S1 is
  dedicated exclusively to input; please refer to the specific chip datasheet
  for details.

## Driver Configuration

Select the desired I2S device in the {menuselection} `On-Chip Peripheral RTOS
Drivers --> Enable I2S Audio Driver` menu.

The following macro switches enable the I2S_MIC and I2S_CODEC devices:
```c
#define BSP_USING_DMA 1
#define RT_USING_AUDIO 1
#define BSP_USING_I2S 1
#define BSP_ENABLE_I2S_MIC 1
#define BSP_ENABLE_I2S_CODEC 1
```

## Device Names
- `i2s<x>`, where x is the device number, such as `i2s1`, `i2s2`, corresponding
  to the peripheral number. Refer to the definitions in audio_config.h. You can
  see that the device name is i2s2, but it actually corresponds to the hardware
  i2s1 (hwp_i2s1). For chips with only one I2S, or if I2s1 is actually used,
  note that hwp_i2s2 should be changed to hwp_i2s1. If configuring
  BF0_MIC_CONFIG, the I2S used as a mic will use drv_i2s_mic.c. If using
  BSP_ENABLE_I2S_CODEC, drv_i2s_audio.c is generally used for playback data.
  Note that rt_device_find("i2s1") or rt_device_find("i2s2") should match the
  .name below. You need to specifically check which one is used in the
  configuration below.

struct i2s_audio_cfg_t bf0_i2s_audio_obj[] = { ... }

```c
#ifdef BSP_USING_I2S
#ifdef BSP_ENABLE_I2S_MIC
#ifndef BF0_MIC_CONFIG
#define BF0_MIC_CONFIG \
    {                                           \
       .name                    = "i2s1",       \
       .dma_handle              = MIC_DMA_INSTANCE, \
       .dma_request             = I2S1_RX_DMA_REQUEST,  \
       .is_record               = 1,            \
       .i2s_handle              = hwp_i2s1,      \
       .reqdma_tx               = I2S1_TX_DMA_REQUEST, \
       .hdma_tx                 = MIC_TX_DMA_INSTANCE, \
    }
#endif /* BF0_MIC_CONFIG */
#endif /* BSP_ENABLE_I2S_MIC */

#ifdef BSP_ENABLE_I2S_CODEC
#ifndef BF0_I2S2_CONFIG
#define BF0_I2S2_CONFIG \
    {                                           \
       .name                    = "i2s2",       \
       .dma_handle              = I2S_RX_DMA_INSTANCE, \
       .dma_request             = I2S2_RX_DMA_REQUEST,  \
       .is_record               = 1,            \
       .i2s_handle              = hwp_i2s2,      \
       .reqdma_tx               = I2S2_TX_DMA_REQUEST, \
       .hdma_tx                 = I2S_TX_DMA_INSTANCE, \
    }
#endif /* BF0_I2S2_CONFIG */
#endif /* BSP_ENABLE_I2S_CODEC */
```
## Usage Process
### 1. Open the I2S Device
```c
/* Choose i2s2 or i2s1 based on the configuration in audio_config.h */
	rt_device_t i2s = rt_device_find("i2s2");
	if (i2s)
	{
		rt_device_open(i2s, RT_DEVICE_FLAG_RDWR);}
```
### 2. Configure Parameters
```c
struct rt_audio_caps caps;
	caps.main_type = AUDIO_TYPE_INPUT;
	caps.sub_type = AUDIO_DSP_PARAM;
	/* 1 for mono, 2 for stereo */
	caps.udata.config.channels = 2;
	/* Number of bits per sample */
	caps.udata.config.samplefmt = 16;
	/* Sample rate */
	caps.udata.config.samplerate = 44100;
	rt_device_control(i2s, AUDIO_CTL_CONFIGURE, &caps);
	caps.main_type = AUDIO_TYPE_INPUT;
	caps.sub_type = AUDIO_DSP_MODE;
	/* Mode setting, 0 --- master mode; 1 --- slave mode */
	caps.udata.value = 0;
	rt_device_control(i2s, AUDIO_CTL_CONFIGURE, &caps);
```
By default, I2S outputs directly to peripherals. There is also an advanced usage
where I2S inputs from audprc module. This is not commonly used. If needed,
configure it as follows: **if you don't know what is audprc->i2s, skip this**
```c
/* i2s input from audprc*/
	rt_uint32_t inter = 1;
	rt_device_control(i2s, AUDIO_CTL_SETINPUT, (void *)inter);
```
### 3. Set Callback Functions for Receiving or Sending
```c
static rt_err_t audio_rx_ind(rt_device_t dev, rt_size_t size)
{
    LOG_I("audio_rx_ind %d\n", size);
    /*
	here can notify read thread to use rt_device_read(i2s) to read i2s
	*/
    return RT_EOK;
}
rt_err_t audio_tx_done(rt_device_t dev, void *buffer)
{
    LOG_I("audio_tx_done \n");
    return RT_EOK;
}

    rt_device_set_rx_indicate(i2s, audio_rx_ind);
    rt_device_set_tx_complete(i2s, audio_tx_done);
```
### 4. I2S Start
```c
int stream = 0;
    //select one for stream
#if playback_only
    stream = AUDIO_STREAM_REPLAY; /*playback only*/
#else if capture_only
    stream = AUDIO_STREAM_RECORD; /*record only*/
#else if capture_and_playback   
    stream = AUDIO_STREAM_RXandTX; /*record &amp; playback*/
#endif
    rt_device_control(i2s, AUDIO_CTL_START, &amp;stream);

    rt_thread_mdelay(xxxx);

    rt_device_close(i2s);
```
### 5. config size of dma ping-pong buffer
```c
#define AUDIO_DATA_SIZE 640

ALIGN(4) static uint8_t audio_data[AUDIO_DATA_SIZE];
ALIGN(4) static uint8_t audio_tx_data[AUDIO_DATA_SIZE];
```
AUDIO_DATA_SIZE is size of DMA ping-pong buffer，one DMA frame size is
(AUDIO_DATA_SIZE / 2), when read or write，data size should be (AUDIO_DATA_SIZE /
2).


## Using the Audio Driver

The adapter layer registers hardware support functions for RT-Thread and
implements these functions using the I2S HAL. The I2S HAL API is detailed in the
[I2S documentation](#hal-i2s).

## Example Code for Audio Capture Using RT-Thread Microphone Device:

```c
uint8_t g_pipe_data[512];

// Find and open device
rt_device_t g_mic = rt_device_find("i2s1");
rt_err_t err = rt_device_open(g_mic, RT_DEVICE_FLAG_RDONLY);

// Configure Microphone device, sample rate 16000
struct rt_audio_caps caps;
caps.main_type = AUDIO_TYPE_INPUT;
caps.sub_type = AUDIO_DSP_SAMPLERATE;
caps.udata.value = 16000;
rt_device_control(g_mic, AUDIO_CTL_CONFIGURE, &caps);

// Start capture
int stream = 1; // record = 1, playback = 0
rt_device_set_rx_indicate(g_mic, audio_rx_ind);
rt_device_control(g_mic, AUDIO_CTL_START, &stream);
...
```

## Example Code for Audio Playback Using RT-Thread Speaker/Headphone Device:

```c
uint8_t g_pipe_data[512];

// Find and open device
rt_device_t g_i2s = rt_device_find("i2s2");
rt_err_t err = rt_device_open(g_i2s, RT_DEVICE_FLAG_RDWR);

// Configure speaker device, sample rate 16000
struct rt_audio_caps caps;
caps.main_type = AUDIO_TYPE_INPUT;
caps.sub_type = AUDIO_DSP_SAMPLERATE;
caps.udata.value = 16000;
rt_device_control(g_i2s, AUDIO_CTL_CONFIGURE, &caps);

// Start playback
int stream = 0; // record = 1, playback = 0
rt_device_set_tx_complete(g_i2s, audio_tx_done);
rt_device_control(g_i2s, AUDIO_CTL_START, &stream);
...
```
## other exmaple for i2s

$(sdk_root)\example\rt_device\i2s

## shell cmd example
There are command lines at the end of the drv_i2s_audio.c and drv_i2s_mic.c
files, which can also be used as a reference.


[audio]:
https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/audio/audio
## RT-Thread Documentation

- [AUDIO Device Documentation][audio]

