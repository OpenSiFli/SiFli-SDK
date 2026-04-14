# Audio document
## Standard
   When developing third-party libraries, avoid directly calling interfaces like audio_open() in the library, as header files and structures may change. Instead, encapsulate an adaptor to invoke multimedia interfaces. After the library is released, some structures in the header files may be modified. Only recompile the adaptor code without recompiling the released library.
## 1 Audio
In the RT-Thread device driver documentation for audio drivers
[audprc device driver](/drivers/audprc_audcodec.md)，In practical scenario, multi audio stream may work together, if not support audio mix, only one reqeust can use hardware according to priority, request may resume playback after being interrupted, or mix and play together. To meet these requirements, an audio server was implemented. ref[audio server](/extra/audio_server_en.pdf).use client/server design pattern.

audio_server_init() will be run at kernel start up,.

see API in $(sdk_root)/middleware/audio/include/audio_server.h

**1.1 create a client**
```c
#include <audio_server.h>
audio_client_t audio_open(audio_type_t audio_type,
                          audio_rwflag_t rwflag,
                          audio_parameter_t *paramter,
                          audio_server_callback_func callback,
                          void *callback_userdata);  
/*
function:
    create a audio client handle
parameters:
    audio_type - [in] audio type， audio_server.c use this to decide which one can play or be suspended
                      AUDIO_TYPE_BT_VOICE used as BT voice，
                      AUDIO_TYPE_BT_MUSIC used as local is A2DP sink
                      others used as other app
    rwflag     - [in] AUDIO_TX， this client can play audio
                      AUDIO_RX， this client can capture audio
                      AUDIO_TXRX， play and capture
                      only one client can capture in audio server
    callback   - [in] call back for playing and capture.
    callback_userdata - [in] use as input parameter for callback
*/
```
**1.2 Data structure**

```c
typedef struct
{
    uint8_t codec; /** deprecated*/
    uint8_t tsco;  /** deprecated*/

    /** samplerate for playing */
    uint32_t write_samplerate;

    /** software buffer cache size for playing, max is 0x7FFF(limit by  * rt_ring_buf)
    */
    uint32_t write_cache_size;

    /** channle for PCM data 1 means mono，2 means stereo
     *  data layout should be LRLRLR.... for stereo PCM
     *  one sample data is 16bits little-end PCM
     */
    uint8_t  write_channnel_num;

    /** should be 16 now.
     */
    uint8_t  write_bits_per_sample;

    /** is need audio ans, echo cancellation, agc在audio_open()
        only for speech talk, example HFP BT voice
     */
    uint8_t  is_need_3a;

    /** set 1 if no need uplink agc， only valid when enable is_need_3a*/
    uint8_t  disable_uplink_agc;

    /** samplerate for audio capture
     */
    uint32_t read_samplerate;
    uint32_t read_cache_size; /* no use now, capure data in callback */

    /**
        audio capture channles, 1 is commonly used
    */
    uint8_t  read_channnel_num;

    uint8_t  read_bits_per_sample; /** should be 16 now */
} audio_parameter_t;
```
**1.3 another client create function**
```c
typedef enum
{
    AUDIO_DEVICE_NO_INIT       = 255, //internal use
    AUDIO_DEVICE_NONE          = 254, //internal use
    AUDIO_DEVICE_AUTO          = 253, //internal use
    AUDIO_DEVICE_SPEAKER       = 0, //audio output to speaker, input from mic
    AUDIO_DEVICE_A2DP_SINK     = 1, //audio output to tws
    AUDIO_DEVICE_HFP           = 2, //local is AG, audio output to tws HFP
    AUDIO_DEVICE_I2S1          = 3, //audio output to I2s1
    AUDIO_DEVICE_I2S2          = 4, //audio output to I2s2
    AUDIO_DEVICE_PDM1          = 5, //no use now
    AUDIO_DEVICE_PDM2          = 6, //no use now
    AUDIO_DEVICE_BLE_BAP_SINK  = 7, //local is ble bap src, output to ble bap sink
    AUDIO_DEVICE_NUMBER             //internal use
} audio_device_e;

audio_client_t audio_open2(audio_type_t audio_type,
                           audio_rwflag_t rwflag,
                           audio_parameter_t *paramter,
                           audio_server_callback_func callback,
                           void *callback_userdata,
                           audio_device_e fixed_device);
/**
  this has fixed_device than audio_open()
  fixed_device means this client always use fixed_device for audio  
*/

```
**1.4 callback**

audio tranfered by DMA.

```c
//reveived one frame data
typedef struct
{
    const uint8_t *data;    /* data address*/
    uint32_t      data_len; /* data lenght bytes*/
    uint32_t      reserved;
} audio_server_coming_data_t;

typedef enum
{
    /* receive after audio_open(), usually does not require processing */
    as_callback_cmd_opened           = 0,

    /* receive after audio_cllose(), usually does not require processing */
    as_callback_cmd_closed           = 1,
    as_callback_cmd_muted            = 2, /* not used now */

    /*should be processed，means cache is half empty, can call audio_write()
      to send more data to cache */
    as_callback_cmd_cache_half_empty = 3,
    
    /*cache is full empty, should be processed, can call  audio_write()*/
    as_callback_cmd_cache_empty      = 4,
    
    /* this client is interrupted by a higher priority audio client.
       this client will be suspended, write data to cache will be droped.
    */
    as_callback_cmd_suspended        = 5,

    /* thi client is resumed from suspended state */
    as_callback_cmd_resumed          = 6,

    /* only for audio record, indicating that one frame of recording data
     *  has arrived, the 'reserved' input parameter in callback is a pointer
     *  of type audio_ server _coming_data_t
     */
    as_callback_cmd_data_coming      = 7,

    /* Used for the audio_mp3ctrl.h, indicating that all mp3 data has been played.
     * The interface that directly uses audio_open() will not be affected */
    as_callback_cmd_play_to_end      = 8,

    /*avrcp, Next button pressed on peer side*/
    as_callback_cmd_play_to_next     = 9,
    /*avrcp, Prev button pressed on peer side*/
    as_callback_cmd_play_to_prev     = 10,
    /*avrcp, Resume button pressed on peer side*/
    as_callback_cmd_play_resume      = 11,
    /*avrcp, Pause button pressed on peer side*/
    as_callback_cmd_play_pause       = 12,
    
    /*User defined command, defined by the app itself for what purpose.
     *The MP3 player plugin uses this to notify the app of the time it has
     *been played*/
    as_callback_cmd_user             = 100,
} audio_server_callback_cmt_t;


typedef int (*audio_server_callback_func)(audio_server_callback_cmt_t cmd,
             void *callback_userdata, /* 'callback_userdata' parameter in audio_open() */
             uint32_t reserved
             );
/*
  reserved:
    1. cmd is as_callback_cmd_data_coming, reserved is a pointer of
       audio_server_coming_data_t
    2. ...

*/
```
** **
**exmaple for mp3 record**
在$(sdk_root)\middleware\audio\audio_local_music\audio_recorder.c
** **

**1.5 play pcm file**
```c
#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include "audio_server.h"

#define DBG_TAG           "audio"
#define DBG_LVL           LOG_LVL_INFO
#include "log.h"

static int fd;
static in is_cache_full;
#define WRITE_CACHE_SIZE    8192

/*
   look TX_DMA_SIZE in audio_server.c,
   WRITE_CACHE_SIZE should >= TX_DMA_SIZE * 2;
*/

int16_t pcm_buffer[WRITE_CACHE_SIZE/2/sizeof(int16_t)];

static int speaker_callback(audio_server_callback_cmt_t cmd,
                    void *callback_userdata,
                    uint32_t reserved)
{
    audio_client_t client = *((audio_client_t *)callback_userdata);
    if (cmd == as_callback_cmd_cache_half_empty
           ||cmd == as_callback_cmd_cache_empty)
    {
        if (!is_cache_full)
        {
            memset(pcm_buffer, 0, sizeof(pcm_buffer));
            int len = read(fd, pcm_buffer, sizeof(pcm_buffer));
            if (len < sizeof(pcm_buffer) && len >= 0)
            {
                //file end;
            }
        }
        int ret = audio_write(client, (uint8_t *)&pcm_buffer[0], sizeof(pcm_buffer));
        if (ret == 0)
        {
            is_cache_full = 1;
            LOG_I("cache full");
        }
    }
    return 0;
}
static void speaker(uint8_t argc, char **argv)
{
    uint32_t record_seconds = 0;
    is_cache_full = 0;
    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;
    pa.write_channnel_num = 1;
    pa.write_samplerate = 16000;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;
    pa.read_samplerate = 16000;
    pa.write_cache_size = WRITE_CACHE_SIZE;
    fd = open("/test.pcm", O_RDONLY | O_BINARY);
    RT_ASSERT(fd >= 0);
    audio_client_t client = NULL;
    audio_server_set_private_volume(AUDIO_TYPE_LOCAL_MUSIC, 15);
    client = audio_open(AUDIO_TYPE_LOCAL_MUSIC,
                        AUDIO_TX,
                        &pa,
                        speaker_callback,
                        &client);
    RT_ASSERT(client);

    while (record_seconds < 10)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }
    audio_close(client); //cannot be called in mic2speaker_callback, it will cause a deadlock.
    close(fd);
}

MSH_CMD_EXPORT(speaker, pcm2speaker test);
```

**1.6 Example of collecting mic data and playing it back to the speaker**

```c
#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include "audio_server.h"

#define DBG_TAG           "audio"
#define DBG_LVL           LOG_LVL_INFO
#include "log.h"
#define WRITE_CACHE_SIZE    4096
static int mic2speaker_callback(audio_server_callback_cmt_t cmd,
                    void *callback_userdata,
                    uint32_t reserved)
{
    audio_client_t client = *((audio_client_t *)callback_userdata);
    if (cmd == as_callback_cmd_data_coming)
    {
        //use mic data to write to device
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        audio_write(client, (uint8_t *)p->data, p->data_len);
        /* If it's recording, you can save your own data here. 
         * This function runs in the audio server thread.
         * The thread stack size is defined by the AUDIO_SERVER_STACK_SIZE macro in audio_server.c
         * The default DMA size 320. For 16k mono, it collects data every 10ms
         * Some microphones may have about 200ms of noise at the start. You drop the first 20 frames.
         */
    }
    else if (cmd == as_callback_cmd_cache_half_empty
           ||cmd == as_callback_cmd_cache_empty)
    {
        /*
          if not use mic data, here should audio_write() next frame data
          using other PCM data source. here is in audio server thread，
          it's better to nority app writting thread to call audio_write().
        */
        //audio_write(client, other_pcm_data, WRITE_CACHE_SIZE / 2);
    }
    return 0;
}
static void mic2speaker(uint8_t argc, char **argv)
{
    uint32_t record_seconds = 0;
    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;
    pa.write_channnel_num = 1;
    pa.write_samplerate = 16000;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;
    pa.read_samplerate = 16000;
    pa.write_cache_size = WRITE_CACHE_SIZE;

    audio_client_t client = NULL;
    audio_server_set_private_volume(AUDIO_TYPE_LOCAL_MUSIC, 15);
    client = audio_open(AUDIO_TYPE_LOCAL_MUSIC,
                        AUDIO_TXRX,
                        &pa,
                        mic2speaker_callback,
                        &client);
    RT_ASSERT(client);

    while (record_seconds < 10)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }
    audio_close(client); //cannot be called in mic2speaker_callback, it will cause a deadlock.
}

MSH_CMD_EXPORT(mic2speaker, mic2speaker test);
```

**1.7 audio write**

if data_len is larger than the write cache size, it will not be written and will keep returning 0.
```c
int audio_write(audio_client_t handle, uint8_t *data, uint32_t data_len);
/*
input:
    handle   - [in] handle of audio client
    data     - [in] 16bit PCM little-end PCM data, if stereo, data layout as LRLRLR.....
    data_len - [in] data lenght in bytes, should less than write cache size,
                    best is half of write cache size
return:
    -2: invalid parameter
    -1: clent is suppended or paused, drop audio data
    > 0: is eq to data_len, data was putted in to write cache
    0:  write cache is full, should sleep than try again
*/
```
**1.8 audio ioctl**
```c
int audio_ioctl(audio_client_t handle, int cmd, void *parameter);
/*
input:
    handle   - [in] handle of audio client
    cmd      - [in] command
    parameter - [in/out] parameter for command
return:
    0     - success
    other - failed

cmd value and parameter:
    AUDIO_IOCTL_FADE_OUT_START - fade out，volume gradually decreases，parameter no use
    AUDIO_IOCTL_FLUSH_TIME_MS  - parameter type 'uint32_t *'，return how many milliseconds of data
                                 in the cache can be played by paramter
    AUDIO_IOCTL_IS_FADE_OUT_DONE - is fade out finished， parameter no use
    AUDIO_IOCTL_BYTES_IN_CACHE   - parameter type is 'uint32_t *'，return data size bytes in cache 
    AUDIO_IOCTL_ENABLE_CPU_LOW_SPEED - parameter eq 1, allow cpu freq scale down.
                                       parameter eq 0，cpu freq scale up to normal.
*/

```
**1.9 audio close**
```c
int audio_close(audio_client_t handle);
```
Except for audio_write(), the functions above are synchronous and are best called within a single thread. Especially audio_close(), this function cannot be called in the callback function of audio_open(), as it will cause a deadlock.

**1.10 hardware device**
```c
//Legacy code, only for the system MP3 playback plugin, do not use in the app
bool audio_device_is_a2dp_sink()
```
```c
/* return current audio_device_e,
  Legacy issue, the app should not be used, needs improvement,
  does not support scenarios where multiple devices operate simultaneously.
*/
uint8_t get_server_current_device(void);
```
```c
int audio_server_select_public_audio_device(audio_device_e audio_device);
/*
   Selecting the corresponding hardware devices for all music types,
   will definitely succeed if called after audio_server_init().
 */

```


```c
**this is not supported now, Not recommended for use**
/*
    Select specific hardware devices for certain types of music.
    This function is deprecated. If you want to use this function,
    you need to modify its internal implementation by changing #if 1 to #if 0.
    The places where this function is called require system-level maintenance, 
    especially during TWS disconnects and connections, to ensure proper switching.
*/
int audio_server_select_private_audio_device(audio_type_t audio_type,
            audio_device_e audio_device);

int audio_server_select_private_audio_device(audio_type_t audio_type,
            audio_device_e device_type)
{
#if 1
    UNUSED(audio_type);
    return audio_server_select_public_audio_device(device_type);
#else
    uint32_t val = 0;
    if (device_type >= AUDIO_DEVICE_NUMBER
         || !g_server.is_server_inited
         || audio_type >= AUDIO_TYPE_NUMBER)
    {
        return -1;
    }

    lock();
    do
    {
        audio_server_cmt_t *cmd = audio_mem_calloc(1, sizeof(audio_server_cmt_t));
        RT_ASSERT(cmd);
        cmd->cmd = AUDIO_CMD_DEVICE_PRIVATE;
        val = (uint32_t)audio_type;
        val = (val << 8);
        val = val | ((uint32_t)device_type);
        cmd->client = (audio_client_t)val;
        rt_slist_append(&g_server.command_slist, &cmd->snode);
        send_cmd_event_to_server();
    }
    while (0);
    unlock();
    return 0;
#endif
}
```
**1.11 register a hardware device in audio server**
This app does not require attention; it is already implemented within the system. It mainly registers some hardware or virtual hardware devices such as speakers and BT, for selecting devices during playback. If necessary, refer to the places in the system where it is invoked.
```c
int audio_server_register_audio_device(audio_device_e audio_device,
                             const struct audio_device *p_audio_device);
audio_device -    [in] audio device Type
p_audio_device -  [in] device config and callback
```
**1.12 audio volume**

1.12.1 **Get Max Volume**
Currently, the volume level ranges from 0 to 15, and the following function will return 15.
```c
//return Max voluem 15
uint8_t audio_server_get_max_volume(void);
```
The volume level is divided into public and private settings. Each music type can have its own private volume, allowing different music types to have different volume settings during playback. Public is the volume shared by all music types, similar to C++ overloading, while private can override the public value. If a music type sets its own private volume, this private volume will be used when playing that type of music. If a music type has not set a private volume, the public volume will be used. On the hardware side, volume changes are checked each time a DMA data frame is sent. If a change is detected, the hardware is updated. Currently, volume changes are achieved by setting the codec gain; the software does not alter the volume. To change the volume via software, system-level modifications are required. The hardware codec gain remains unchanged, and the PCM amplitude is adjusted in audio_write().
```c
//volume from 0 ~ 15

//set public volume
int audio_server_set_public_volume(uint8_t volume);

// Set the private volume for a certain volume type
int audio_server_set_private_volume(audio_type_t audio_type,
                                    uint8_t volume);
/*
 Get the private volume of a certain volume type. 
 If it returns 0xFF, it means the private volume has never been set.
*/
uint8_t audio_server_get_private_volume(audio_type_t audio_type);

//Set the microphone to mute or unmute;
int audio_server_set_public_mic_mute(uint8_t is_mute);

//get public mic mute setting
//return 1 if mutted, 0 umutted
uint8_t audio_server_get_public_mic_mute(void);

//Set speaker to mute or unmute;
int audio_server_set_public_speaker_mute(uint8_t is_mute);
//get speaker mute setting
//return 1 if mutted, 0 umutted
uint8_t audio_server_get_public_speaker_mute(void);
```
**1.13 one DMA frame size**
The change in data length currently does not provide an interface for dynamically modifying the recording size; it is configured by macro definitions.

The CFG_AUDIO_RECORD_PIPE_SIZE macro in audio.h represents the data size of a single DMA interrupt during recording. If it is 320, then for a 16kHz mono microphone, data is generated every 10ms, and the audio_open callback is triggered.In audio_server.c, CODEC_DATA_UNIT_LEN should have the same value as CFG_AUDIO_RECORD_PIPE_SIZE. Currently, there is no interface provided to modify it dynamically.

### Special Purpose Interface
**It can only be used in product scenarios where the mic bias pin is treated as a regular GPIO, rather than as the mic power supply of the codec.**
```c
need config
CONFIG_MICBIAS_USING_AS_PDM_POWER=y
API
void micbias_power_on(); //open micbias 
void micbias_power_off(); //close micbias
```
## Add music genre and change priority
If you want to add a music type, you must add the priority in this array in audio_server.c. A higher value indicates higher priority. Also, specify whether mixing is allowed. The mixing configuration is quite complex. Currently, BT voice is exclusive with the highest priority and does not participate in mixing, while others can be mixed. If you want to customize mixing rules, you need to check how is_can_mix() is implemented and determines this.
```c
static const audio_mix_policy_t mix_policy[AUDIO_TYPE_NUMBER] =
```
## config
```c
#audio
CONFIG_AUDIO=y
#audio server
CONFIG_AUDIO_USING_MANAGER=y
#local music
CONFIG_AUDIO_LOCAL_MUSIC=y
#use codec on chip
CONFIG_AUDIO_USING_AUDPROC=y
```
```c
/*
Choose one of the three below to indicate where the speaker 
device's audio input comes from. If it is I2S or PDM, 
you also need to enable the corresponding driver in RT-Thread.
*/
//speaker device's mic data from codec on chip
CONFIG_AUDIO_MIC_USING_CODEC=y
# from I2S
CONFIG_AUDIO_RX_USING_I2S=y
# from PDM
CONFIG_AUDIO_RX_USING_PDM=y
```
If outputting to I2S, first configure I2S according to the [I2S driver interface](/drivers/i2s.md), then in the audio server, refer to that configuration to set up the i2s_config(my, 1); check its internal implementation, and track whether speaker_tx_done is triggered.

```c
static void config_tx(audio_device_speaker_t *my, audio_client_t client)
{
#if defined(AUDIO_TX_USING_I2S)
    i2s_config(my, 1);
    rt_device_set_tx_complete(my->i2s, speaker_tx_done);
#else
    ....
#endif
```

```python
Choose one of the two options below to indicate which hardware
the speaker device uses for audio output. If it's I2S, you also
need to enable the corresponding driver in RT-Thread.

# output to codec
AUDIO_SPEAKER_USING_CODEC=y
# output to I2S
CONFIG_AUDIO_TX_USING_I2S=y
```
```python
# If you want multiple clients to work together, for example,
# playing music while recording, you need to configure this
# This way, the two clients created separately with AUDIO_TX and
# AUDIO_RX can work together
CONFIG_MULTI_CLIENTS_AT_WORKING=y
# If both clients are AUDIO_TX and rely on the audio server to mix,
# then you need to enable this
CONFIG_SOFTWARE_TX_MIX_ENABLE=y

```

## MP3 playback API
The audio playback interface is in audio_mp3ctlr.h. 
MP3 playback baseon audio server.
There are quite a few examples, and you can search for example code in the SDK. Currently, it automatically recognizes mp3 and wav files, but wav files only support 16-bit PCM format.At the end of the audio_mp3ctrl.c file, there is an mp3 playback command. The code is commented out by default.
Change
#define MP3_TEST_CMD 0 to
#define MP3_TEST_CMD 1
and you can use the mp3 command to play files.
mp3 command help in comment at end of audio_mp3ctlr.c

## resample
  Implemented a simple resampling, mainly for internal use, especially during mixing.
```c
 /*
   create a resampel handle
parameter:
   channels - [in]channels of source data，1 - mono, 2, stereo
                  if stereo, data layout is LRLRLR.....
   src_samplerate - [in] samplerate before resampling
   dst_samplerate - [in] samplerate after resampling
return: hanle of resample
 */
sifli_resample_t *sifli_resample_open(uint8_t channels,
                            uint32_t src_samplerate,
                            uint32_t dst_samplerate);

/*
    process resample
parameter:
    p - [in] hanle of resample
    src - [in] source data address
    src_bytes - [in] source data size in bytes
    is_last_packet - [in] should always be zero if has more data to 
                          process later
*/
uint32_t sifli_resample_process(sifli_resample_t *p,
                int16_t *src, uint32_t src_bytes,
                uint8_t is_last_packet);

/*
    return data process result after call sifli_resample_process(),
*/
int16_t *sifli_resample_get_output(sifli_resample_t *p);

/*
 close handle to release resource
*/
void sifli_resample_close(sifli_resample_t *p);
```
## Integration of data processing algorithms
Currently, the supported voice processing algorithm is WebRTC. If you want to integrate a third-party algorithm, you need to implement the following functions yourself, refer to audio_3a.c. Or you can delete audio_3a.c to see what is missing. You can also choose not to use WebRTC and configure the following to use the Anyka speech algorithm:
CONFIG_PKG_USING_ANYKA=y
You can also refer to this algorithm to replace audio_3a.c,
source code at $(sdk_root)/middleware/audio/anyka

```c
/*
    BT hfp, downlink data, 
parameter
    fifo - [in] data address,
    size - [in] data size in bytes
  */
void audio_3a_downlink(uint8_t *fifo, uint8_t size);
/*
    audio server put refrence data for echo cancellation
*/
void audio_3a_far_put(uint8_t *fifo, uint16_t fifo_size);
/*
    init audio 3a
parameter:
    samplerate - [in] speech samplereate NB 8000 or WB 16000
    is_bt_voice- [in] 1, is used for HFP, will send result to HFP
                      0, not for HFP, will send result to app
    disable_uplink_agc - [in] enable uplink agc or not
*/
void audio_3a_open(uint32_t samplerate,
                 uint8_t is_bt_voice,
                 uint8_t disable_uplink_agc);
void audio_3a_close();

```
## Hardware Amplifier Adaptation
The amplifier before the speaker may require different hardware amplifiers on different platforms. The following three functions need to be implemented in $(sdk_root)/middleware/audio/audio_port/audio_pa_api.c
```c
RT_WEAK void audio_hardware_pa_init()
{
    // Initialization when PA powers on, pin configuration, etc.
}
RT_WEAK void audio_hardware_pa_start(uint32_t samplerate,
                                    uint32_t reserved)
{
    // Turn on PA
}
RT_WEAK void audio_hardware_pa_stop(void)
{
    // Turn off PA
}
```

## EQ Switch Settings
If using the chip's codec, there is a hardware EQ on the chip. Currently, the software settings only support 44.1k and 16k.You can modify this file to decide whether to turn the EQ on or off for a certain music type.
Located at $(sdk_root)/middleware/audio/audio_port/audio_eq_config.c
Specifically, during factory test frequency response mode, the EQ should be turned off. You can add a check in the get_eq_config() function to return 0 to disable the EQ if it is factory mode.
```c
/*
return:
        0 - not enable eq for audio type 'type'
        1 - enable eq for audio type 'type'
*/
RT_WEAK uint8_t get_eq_config(audio_type_t type);
```

## EQ Parameter Settings
The EQ tool [Sifli_EQ](https://wiki.sifli.com/tools/index.html)
The EQ parameters are in drv_audprc.c. They should ideally be generated using the tool, but sometimes it is convenient to directly modify the code for debugging.The following commands can be used to adjust the mic gain.  You can also refer to the code implementation:
mic_gain
pdm_gain (if PDM is configured)
Sometimes, volume is manually adjusted. Volume testing must meet hardware requirements; the maximum volume should not exceed the speaker's power to avoid damaging it. The maximum volume should be determined through hardware testing to meet power requirements.
```c
int8_t g_adc_volume = 0; // mic gain
/*Maximum volume protection for phone calls, unit: 0.5db.
If the values in g_tel_vol_level[] multiplied by 2 exceed this value, this value will be used as the speaker gain.
*/
int8_t g_tel_max_vol = -2;// 16 volume levels for phone calls, unit: 1dB
int8_t g_tel_vol_level[16] = {-36, -34, -32, -30, -28, -26, -24, -22, -20, -17, -14, -11, -10, -8, -6, -4};
/*Maximum volume protection for music, unit: 0.5db.
If the values in g_music_vol_level[] multiplied by 2 exceed this value, this value will be used as the speaker gain.
*/
int8_t g_music_max_vol = 0;
int8_t g_music_vol_level[16] = {-55, -34, -32, -30, -28, -26, -24, -22, -20, -17, -14, -11, -10, -8, -6, -4};
```