# 音频文档
## 规范
   第三方lib开发，不要直接在lib里调用audio_open()等接口，因为头文件和结构体可能更改，要封装一个adaptor，在adaptor里调用多媒体的接口，因为lib发布后，头文件里一些结构体可能更改，需要重新编译,这样只编译adaptor的代码，不用重新编译发布lib

## 音频接口
音频的驱动接口的RT-Thread 设备驱动说明里，[Audprc设备级驱动接口](/drivers/audprc_audcodec.md)，在实际使用时，涉及电话时带回声消除的，播放音乐过程来通知音的，就是存在多个音频都请求设备驱动工作的场景.可能的需求是, 按优先级互相打断，被打断后可以恢复播放，或者混音一起播放，为了满足这个需求，实现了audio server,参考[audio server使用说明](/extra/audio_server.pdf)，管理多个用户的音频设备的使用，采用client/server的设计模式，每个音频播放请求为一个client。
开机会自动调audio_server_init()启动audio server， 使用了audio server, 音频相关的都应该基于audio server作为audio与底层设备的交互。

接口在$(sdk_root)/middleware/audio/include/audio_server.h里定义。
使用流程如下:

**1.1创建一个client**
```c
#include <audio_server.h>
audio_client_t audio_open(audio_type_t audio_type,
                          audio_rwflag_t rwflag,
                          audio_parameter_t *paramter,
                          audio_server_callback_func callback,
                          void *callback_userdata);  
/*
功能:
    创建一个audio client句柄，后续用这个句柄向audio server发起请求
参数:
    audio_type - [in] 播放的音乐类型， audio_server.c里靠这个类型决定互相打断的优先级.
                      其中AUDIO_TYPE_BT_VOICE专门用来区分是不是蓝牙通话用，
                      AUDIO_TYPE_BT_MUSIC专门用来表示本地是A2DP sink
                      其他看系统整体按需分配，也可以增加类型，这个后面再讲
    rwflag     - [in] AUDIO_TX， 这个client只用来audio play
                      AUDIO_RX， 这个client只用来audio record
                      AUDIO_TXRX， 这个client只用来audio play and record
                      按实际需要打开，audio server里只支持一个client进行audio record，
                      不要随便带audio record，会影响client是否有优先使用权.
    callback   - [in] 如果不为NULL，audio server用这个函数指针向client回调事件用.
    callback_userdata - [in] callback时会带回这个值
*/
```
**1.2 重要结构体说明**

```c
typedef struct
{
    uint8_t codec; /** deprecated，历史遗留，已废除 */
    uint8_t tsco;  /** deprecated，历史遗留，已废除 */

    /** 输出音频时的采样率，audio_open()时rwflag为AUDIO_TX或AUDIO_TXRX时才有效 */
    uint32_t write_samplerate;

    /** 输出音频缓存的大小，最大为0x7FFF，太小了audio_write()写不进去，应该至少为一次
     *  audio_write写入的数据量的2倍，这样可以做ping-pong写入数据
     *  */
    uint32_t write_cache_size;

    /** 输出的通道数，与实际写入的数据要一致， 1为单声道，2为双声道，如果是双声道，
     *  audio_write传入的数据为左右声道交错: LRLRLR...., 每个L或R为16为little-end的
     *  PCM数据
    */
    uint8_t  write_channnel_num;

    /** 每个PCM采样的位数，目前只支持设置为16.
     */
    uint8_t  write_bits_per_sample;

    /** 是否需要audio 3a算法，在audio_open()时rwflag为AUDIO_TXRX有效，一般是双向
     *  语音通话传输时需要，比如BT HFP电话，VoIP， 做回声消除处理。
     *  算法一般都是只支持16k或8k的回声消除，其他采样率是不行的
     */
    uint8_t  is_need_3a;

    /** is_need_3a为1时，这个表示上行要不要做音量agc放大处理 */
    uint8_t  disable_uplink_agc;

    /** 输出音频时的采样率，audio_open()时rwflag为AUDIO_RX或AUDIO_TXRX时才有效,
     *  常用的为16000或8000
     */
    uint32_t read_samplerate;
    uint32_t read_cache_size; /** 没使用，输入数据没缓存，靠回调函数输入数据 */

    /** 输入的通道数，1为单声道，2为双声道，如果是双声道，
     *  输入的数据为左右声道交错: LRLRLR...., 每个L或R为16为little-end的
     *  PCM数据
    */
    uint8_t  read_channnel_num;

    uint8_t  read_bits_per_sample; /** should be 16 now */
} audio_parameter_t;
```
**1.3 另外一个创建client的函数**
```c
typedef enum
{
    AUDIO_DEVICE_NO_INIT       = 255, //内部使用，API不可以使用
    AUDIO_DEVICE_NONE          = 254, //内部使用，API不可以使用
    AUDIO_DEVICE_AUTO          = 253, //内部使用，API不可以使用
    AUDIO_DEVICE_SPEAKER       = 0, //audio output to speaker, input from mic
    AUDIO_DEVICE_A2DP_SINK     = 1, //audio output to tws
    AUDIO_DEVICE_HFP           = 2, //local is AG, audio output to tws HFP
    AUDIO_DEVICE_NUMBER             //内部使用，API不可以使用
} audio_device_e;

audio_client_t audio_open2(audio_type_t audio_type,
                           audio_rwflag_t rwflag,
                           audio_parameter_t *paramter,
                           audio_server_callback_func callback,
                           void *callback_userdata,
                           audio_device_e fixed_device);
/**
  这个函数同audio_open()比就是最后多了个参数fixed_device，
  它表示这个client要固定使用哪个设备播放，不参加输出设备的切换，
  他的输入范围在audio_device_e定义的实体设备，
  按上面的定义，区间范围是[AUDIO_DEVICE_SPEAKER, AUDIO_DEVICE_NUMBER)，
  目前没有app这样用，可能的用法是使用AUDIO_DEVICE_SPEAKER，
  这样无论是否连上tws耳机，这个client一直固定从speaker输出.
*/

```
**1.4 回调函数说明**

底下数据使用DMA播放处理的，每帧数据是固定大小。

```c

typedef struct
{
    const uint8_t *data;    /* audio record一帧数据到达的数据地址*/
    uint32_t      data_len; /* audio record一帧数据到达的字节个数*/
    uint32_t      reserved;
} audio_server_coming_data_t;

typedef enum
{
    /* 这个audio_open()后client创建成功时会回调这个命令， 一般不需要处理 */
    as_callback_cmd_opened           = 0,

    /* 这个audio_cllose()client要释放时会回调这个命令， 一般不需要处理 */
    as_callback_cmd_closed           = 1,
    as_callback_cmd_muted            = 2, /* not used now */

    /* 必须处理，表示缓存有一半空间空闲，可以用audio_write()送入下一帧数据 */
    as_callback_cmd_cache_half_empty = 3,
    
    /* 必须处理，表示缓存数据不够一帧了，可以用audio_write()送入下一帧数据 */
    as_callback_cmd_cache_empty      = 4,
    
    /* 表示这个client被更高优先级的音频client打断了，audio_write()数据会被直接丢弃
     * 一般可以收到suspend后再通知app是否需要处理UI和行为，如果是audio record，
     * 不会再收到as_callback_cmd_data_coming录音数据到达的命令了
    */
    as_callback_cmd_suspended        = 5,

    /* 和as_callback_cmd_suspended相反，表示这个client恢复处理，继续处理音频流 */
    as_callback_cmd_resumed          = 6,

    /* only for audio record,  表示录音一帧数据到达了，
     * callback里的reserved输入参数为audio_server_coming_data_t类型指针
    */
    as_callback_cmd_data_coming      = 7,

    /* 给播放插件用的，表示播放完所有数据，对直接使用audio_open()的接口不会来 */
    as_callback_cmd_play_to_end      = 8,

    /* 音频播放到tws耳机时，按耳机的next键，client会收到这个命令 */
    as_callback_cmd_play_to_next     = 9,
    /* 音频播放到tws耳机时，按耳机的prev键，client会收到这个命令 */
    as_callback_cmd_play_to_prev     = 10,
    /* 音频播放到tws耳机时，按耳机的resume键，client会收到这个命令 */
    as_callback_cmd_play_resume      = 11,
    /* 音频播放到tws耳机时，按耳机的pause键，client会收到这个命令 */
    as_callback_cmd_play_pause       = 12,
    
    /* 用户自定义命令，由app自己定义做什么用，MP3播放插件用了这个通知app已经播放的时间 */
    as_callback_cmd_user             = 100,
} audio_server_callback_cmt_t;


typedef int (*audio_server_callback_func)(audio_server_callback_cmt_t cmd,
             void *callback_userdata, /*audio_open()时传入的callback_userdata*/
             uint32_t reserved
             );
/*
  reserved的含义:
    1. cmd is as_callback_cmd_data_coming, reserved is  a pointer of
       audio_server_coming_data_t
    2. ...

*/
```
** **
**一个mp3录音的例子**

在$(sdk_root)\middleware\audio\audio_local_music\audio_recorder.c
** **

**1.5 播放pcm数据的例子**
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
   见audio_server.c 里TX_DMA_SIZE宏定义大小，这个WRITE_CACHE_SIZE应至少是TX_DMA_SIZE的2倍
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
    audio_close(client); //这个不可以在mic2speaker_callback里调，会死锁
    close(fd);
}

MSH_CMD_EXPORT(speaker, pcm2speaker test);
```

**1.6 采集mic数据再播放到喇叭的例子**
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
        //如果是录音， 这里可以保存自己的数据，这个函数在audio server线程里，
        //线程栈的大小见audio_server.c里AUDIO_SERVER_STACK_SIZE宏定义
        //默认一次录音DMA大小为320，16k单声道的话默认是10ms进来一次，
        //有的mic开始有200ms左右噪声，可以在这里把前面20帧丢弃不处理
    }
    else if (cmd == as_callback_cmd_cache_half_empty
           ||cmd == as_callback_cmd_cache_empty)
    {
        /*
          if not use mic data, here should audio_write() next frame data
          using other PCM data source.
          这里是audio server的线程里，理想的程序模型是向一个write线程发消息，
          write线程里调audio_write(), 并需要判断是否写进去了。
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
    audio_close(client); //这个不可以在mic2speaker_callback里调，会死锁
}

MSH_CMD_EXPORT(mic2speaker, mic2speaker test);
```

**1.7 写入数据**

目前写入的数据长度如果比write cache size大，会一直写不进去，会一直返回0
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
**1.8 控制client**
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

cmd的值及对应的parameter：
    AUDIO_IOCTL_FADE_OUT_START - 表示开始淡出，音量逐渐减小，parameter可以为NULL
    AUDIO_IOCTL_FLUSH_TIME_MS  - parameter为uint32_t *， 返回cache中数据可以播放多少毫秒
    AUDIO_IOCTL_IS_FADE_OUT_DONE - 查询是否淡出完成， parameter可以为NULL
    AUDIO_IOCTL_BYTES_IN_CACHE   - parameter为uint32_t *，返回cache中剩余多少字节数据没播放 
    AUDIO_IOCTL_ENABLE_CPU_LOW_SPEED - parameter为1, 允许cpu降频. parameter为0， cpu升频.
*/

```
**1.9 关闭一个client**
```c
int audio_close(audio_client_t handle);
```
上面一个函数除了audio_write(),其他是同步的，最好在一个线程里调，
特别是audio_close(),这个函数不可以在audio_open()的回调函数里调，会死锁

**1.10 硬件设备**
```c
//历史遗留，只给系统mp3播放插件用，app不要使用
bool audio_device_is_a2dp_sink()
```
```c
//返回当前的audio_device_e, 历史遗留，app不要用，需要改进，不支持多设备同时工作的场景
uint8_t get_server_current_device(void);
```
```c
int audio_server_select_public_audio_device(audio_device_e audio_device);
/*
   功能：
       选择所有音乐类型的对应的硬件设备，在audio_server_init()之后调用一定会成功。   
 */


```


```c
**下面这个新版本已关闭，不推荐使用**
/*
    功能：
        为特定音乐类型选择使用特定的硬件设备，这个函数已不推荐使用，
        如果要使用这个功能，要修改函数内部实现，把#if 1改为 #if 0
        调用地方需要系统级修改维护，特别是tws断开和链接的地方，要保证切换正确。
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
**1.11 注册一个硬件设备**
这个app不需要关注，系统内部已实现，主要是喇叭和BT等注册一些硬件或虚拟硬件设备，
供播放时切换选择设备用.如果需要，搜索系统里调用的地方进行参考.
```c
int audio_server_register_audio_device(audio_device_e audio_device,
                             const struct audio_device *p_audio_device);
audio_device -    [in] 设备类型
p_audio_device -  [in] 注册设备必须的一些回调函数
```
**1.12 音量相关**

1.12.1 **获得系统最大音量**
目前音量等级时从0到15，下面这个函数会返回15
```c
//返回最大音量等级，目前是15
uint8_t audio_server_get_max_volume(void);
```
音量的大小设置分public和private，每种音乐类型可以设置有自己的private音量, 这样不同音乐类型在播放时可以有不同的音量设置。public为所有音乐类型公用的音量，和c++重载类似，private可以重载public的值，如果某种音乐类型设置了自己的private音量，那么播放这个类型音乐时就使用它的private音量，如果某种音乐类型没有设置过private音量，那它就使用public音量。硬件上音量的改变，是在每次送一帧DMA数据时，会读一下音量是否改变，改变的话就会设置硬件，目前音量的改变是靠设置codec增益完成的，软件没有改变音量，如果要软件改变音量，需要系统级修改，硬件codec增益不变，在audio_write()时改变pcm的幅度.

```c
//volume的值从0到15

//设置public音量大小
int audio_server_set_public_volume(uint8_t volume);

//设置某个音量类型的private音量
int audio_server_set_private_volume(audio_type_t audio_type,
                                    uint8_t volume);
//获得某个音量类型的private音量，如果返回0xFF则表示没设置过private音量
uint8_t audio_server_get_private_volume(audio_type_t audio_type);

//设置mic静音或非静音，开机默认值不静音
int audio_server_set_public_mic_mute(uint8_t is_mute);

//获得mic是否mute
uint8_t audio_server_get_public_mic_mute(void);

//设置speaker静音或非静音，开机默认值不静音
int audio_server_set_public_speaker_mute(uint8_t is_mute);
//获得speaker是否mute
uint8_t audio_server_get_public_speaker_mute(void);
```
**1.13 数据长度的更改，目前没提供动态修改录音DMA一帧大小的接口，是宏定义配置的**

在audio.h里的这个CFG_AUDIO_RECORD_PIPE_SIZE宏，表示一次录音的dma中断的数据大小，
如果是320，则对16k单声道的mic，是10ms来一次数据，并调audio_open的callback

audio_server.c，CODEC_DATA_UNIT_LEN应该与CFG_AUDIO_RECORD_PIPE_SIZE相同的值，
目前没提供接口动态修改。

### 特殊用途接口
**对micbias pin当作普通GPIO， 而不用codec上的mic的产品场景才能用**
```c
需要配置
CONFIG_MICBIAS_USING_AS_PDM_POWER=y
void micbias_power_on(); //打开micbias
void micbias_power_off(); //关闭micbias
```
## 添加音乐类型和更改优先级
如果要添加音乐类型，一定要在audio_server.c里这个数组里添加优先级，优先级的值越大表示优先级越高。并填好是否可以混音，混音配置比较复杂，目前BT voice是独占的切优先级最高，不参合混音，其他都是可以混音的，如果要定制混音规则，需要看is_can_mix()是如何实现和判断的。
```c
static const audio_mix_policy_t mix_policy[AUDIO_TYPE_NUMBER] =
```
## 配置
```
//支持audio
CONFIG_AUDIO=y
//支持audio server
CONFIG_AUDIO_USING_MANAGER=y
//支持本地音乐播放
CONFIG_AUDIO_LOCAL_MUSIC=y
//使用芯片的codec
CONFIG_AUDIO_USING_AUDPROC=y
```
```c
/*
下面三个选一个，表示speaker设备音频输入来自哪里， 如果是I2S或PDM，
还得打开rt thread里对应的驱动
*/

# 来自芯片codec
CONFIG_AUDIO_MIC_USING_CODEC=y
# 来自I2S
CONFIG_AUDIO_RX_USING_I2S=y
# 来自PDM
CONFIG_AUDIO_RX_USING_PDM=y
```
```c
下面两个选一个，表示speaker设备音频输出使用哪个硬件，
如果是I2S还得打开rt thread里对应的驱动

# 输出到芯片codec
AUDIO_SPEAKER_USING_CODEC=y
# 输出到I2S
CONFIG_AUDIO_TX_USING_I2S=y
```
如果输出到I2S, 先根据[I2S设备级驱动接口](/drivers/i2s.md)调通I2S, 再audio server里要参考那个配置好下面的i2s_config(my, 1);内部的实现，并跟踪下speaker_tx_done是否到达

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
#如果要多个client一起工作，比如录音时还可以播放音乐，需要配置这个
#这样分别用AUDIO_TX和AUDIO_RX创建的两个client就可以一起工作
CONFIG_MULTI_CLIENTS_AT_WORKING=y
#如果两个client都是AUDIO_TX， 靠audio server混音，则需要打开这个
CONFIG_SOFTWARE_TX_MIX_ENABLE=y
```
## MP3播放插件
音频播放插件接口在audio_mp3ctlr.h里，使用audio server, 这个例子比较多，可以搜索sdk的example代码。
目前自动识别mp3文件和wav文件的，wav文件只支持16位pcm格式

在audio_mp3ctrl.c文件后面有个mp3播放命令，代码默认被注释了，
把
#define MP3_TEST_CMD 0
改为
#define MP3_TEST_CMD 1
就可以使用mp3命令播放文件了， MP3命令的使用帮助在audio_mp3ctrl.c文件的mp3命令实现的地方。
## 重采样
    实现了一个简单重采样，主要是内部使用的，特别是混音时
```c
 /*
   创建一个重采样的句柄，
parameter:
   channels - [in] 原始数据的通道数，1或2，对应mono或stereo，
                   如果时stereo， 数据格式时LRLRLR.....
   src_samplerate - [in] 数据的重采样前的采样率
   dst_samplerate - [in] 数据的重采样后的采样率
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
                            int16_t *src,
                            uint32_t src_bytes,
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

## 数据处理的算法集成
  目前支持的语音处理算法为webrtc，如果要集成第三方算法，需要自己实现如下函数, refence audio_3a.c. 或删除audio_3a.c就可以看到缺哪些东西了。
  也可以不使用webrtc，配置下面这个就使用anyka speech算法
  CONFIG_PKG_USING_ANYKA=y
  也可以参考这个算法替换audio_3a.c的，
  代码在sdk\middleware\audio\anyka
  也都会实现这个函数

```c
/*
    设置算法关闭还是打开，一般工厂频想测试时需要关闭算法。历史遗留，
    可以自己实现，mic和down没用
*/
void audio_3a_set_bypass(uint8_t is_bypass, uint8_t mic, uint8_t down);
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
## 硬件功放的适配
喇叭前的功放，不同平台要更改硬件功放，需要实现下面三个函数,
在$(sdk_root)/middleware/audio/audio_port/audio_pa_api.c
```c
RT_WEAK void audio_hardware_pa_init()
{
    //PA开机的初始化，管脚配置等
}
RT_WEAK void audio_hardware_pa_start(uint32_t samplerate,
                                     uint32_t reserved)
{
    //打开PA
}
RT_WEAK void audio_hardware_pa_stop(void)
{
    //关闭PA
}
```
## EQ开关设置
如果使用芯片的codec，芯片上有硬件EQ，目前软件设置上只支持44.1k和16k，
可以修改这个文件决定某种音乐类型是否打开或关闭EQ。
在$(sdk_root)/middleware/audio/audio_port/audio_eq_config.c
特别是工厂测试频响测试模式时，要关闭EQ， 可在get_eq_config()这个函数里加判断，如果时工厂模式就返回0关闭EQ。

```c
/*
    return:
            0 - not enable eq for audio type 'type'
            1 - enable eq for audio type 'type'
*/
RT_WEAK uint8_t get_eq_config(audio_type_t type);

```

## EQ参数设置

EQ工具参考[Sifli_EQ](https://wiki.sifli.com/tools/index.html)

EQ的参数在drv_audprc.c， 应该用工具生成，有时为了方便直接修改代码调试。

下面命令可以调试mic增益，调好后要修改编译时初始值，可以看看代码的实现
mic_gain
pdm_gain (如果配置了PDM)

有时也手动修改音量， 音量测试要满足硬件要求，最大音量不能超过喇叭功率把喇叭烧了， 最大音量需要经过硬件测试，满足功率要求。

```c
int8_t g_adc_volume = 0; //mic增益

/*
电话的最大音量保护，单位为0.5db, 
如果 g_tel_vol_level[]里的值乘以2比这个值大的话，就会用这个值作为喇叭增益。
*/
int8_t g_tel_max_vol = -2;

//电话的16个音量等级，单位为1db
int8_t g_tel_vol_level[16] = {-36, -34, -32, -30, -28, -26, -24, -22, -20, -17, -14, -11, -10, -8, -6, -4};


/*
音乐的最大音量保护，单位为0.5db, 
如果 g_music_vol_level[]里的值乘以2比这个值大的话，就会用这个值作为喇叭增益。
*/
int8_t g_music_max_vol = 0;
int8_t g_music_vol_level[16] = {-55, -34, -32, -30, -28, -26, -24, -22, -20, -17, -14, -11, -10, -8, -6, -4};

```
