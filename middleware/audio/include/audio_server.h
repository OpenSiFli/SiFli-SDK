#ifndef AUDIO_SERVER_H
#define AUDIO_SERVER_H  1
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "audioproc.h"
#include "audio_mem.h"

#define AUDIO_DBG_LVL           LOG_LVL_INFO

#define AUDIO_MAX_VOLUME        (15)

/*
    !!!!! notice!!!!
    all API should called after INIT_ENV_EXPORT(audio_server_init)
*/
#if SOFTWARE_TX_MIX_ENABLE
#define TWS_MIX_ENABLE              1
#endif

typedef struct
{
    const uint8_t *data;
    uint32_t      data_len;
    uint32_t      reserved;
} audio_server_coming_data_t;

typedef struct
{
    uint8_t codec;
    uint8_t tsco;
    uint16_t sample_rate;
} audio_param_bt_voice_t;

typedef struct
{
    struct rt_ringbuffer *device_rb;
    uint16_t tx_sample_rate;
    uint16_t tx_channels;
} device_open_parameter_t;

typedef enum
{
    AUDIO_TYPE_BT_VOICE     = 0, //local is hfp
    AUDIO_TYPE_BT_MUSIC     = 1, //local is a2dp sink
    AUDIO_TYPE_ALARM        = 2,
    AUDIO_TYPE_NOTIFY       = 3,
    AUDIO_TYPE_LOCAL_MUSIC  = 4,
    AUDIO_TYPE_LOCAL_RING   = 5,
    AUDIO_TYPE_LOCAL_RECORD = 6,
    AUDIO_TYPE_MODEM_VOICE  = 7,
    AUDIO_TYPE_TEL_RING     = 8,
    AUDIO_TYPE_NUMBER,
    AUDIO_MANAGER_TYPE_INVALID = 0xFF, // only for BT, should delete later, BT should use internal value
} audio_type_t;

typedef enum
{
    AUDIO_TX    = (1 << 0),
    AUDIO_RX    = (1 << 1),
    AUDIO_TXRX  = AUDIO_TX | AUDIO_RX,
} audio_rwflag_t;

typedef enum
{
    as_callback_cmd_opened           = 0,
    as_callback_cmd_closed           = 1,
    as_callback_cmd_muted            = 2,
    as_callback_cmd_cache_half_empty = 3,  //only for for write
    as_callback_cmd_cache_empty      = 4,  //only for write
    as_callback_cmd_suspended        = 5,  //for write/read
    as_callback_cmd_resumed          = 6,  //for write/read
    as_callback_cmd_data_coming      = 7,  //only for read
    as_callback_cmd_play_to_end      = 8,  //file end
    as_callback_cmd_play_to_next     = 9,  //a2dp device to AG
    as_callback_cmd_play_to_prev     = 10, //a2dp device to AG
    as_callback_cmd_play_resume      = 11, //a2dp device to AG, only notify app to chagen UI, app do not resume again
    as_callback_cmd_play_pause       = 12, //a2dp device to AG, only notify app to chage UI, app do not pause again
    as_callback_cmd_user             = 100,
} audio_server_callback_cmt_t;

typedef struct audio_client_base_t *audio_client_t;

typedef struct
{
    uint8_t codec;
    uint8_t tsco;
    // write parameter, only invalid when rwflag is AUDIO_TX/AUDIO_TXRX
    //int      output_device; // now only codec
    uint32_t write_samplerate; //only effecty when rwflag is AUDIO_TX or AUDIO_TXRX
    uint32_t write_cache_size;
    uint8_t  write_channnel_num;
    uint8_t  write_bits_per_sample;
    uint8_t  is_need_3a;
    uint8_t  disable_uplink_agc;
    // read paramter, only invalid when rwflag is AUDIO_RX/AUDIO_TXRX
    uint32_t read_samplerate;
    uint32_t read_cache_size;
    uint8_t  read_channnel_num;
    uint8_t  read_bits_per_sample;
} audio_parameter_t;


typedef enum
{
    AUDIO_DEVICE_NO_INIT       = 255,
    AUDIO_DEVICE_NONE          = 254,
    AUDIO_DEVICE_AUTO          = 253,
    AUDIO_DEVICE_SPEAKER       = 0, //audio output to speaker, or input from mic
    AUDIO_DEVICE_A2DP_SINK     = 1, //audio output to tws
    AUDIO_DEVICE_HFP           = 2, //local is AG, audio output to tws HFP
    AUDIO_DEVICE_I2S1          = 3, //audio output to
    AUDIO_DEVICE_I2S2          = 4,
    AUDIO_DEVICE_PDM1          = 5,
    AUDIO_DEVICE_PDM2          = 6,
    AUDIO_DEVICE_BLE_BAP_SINK  = 7, //local is ble audio src, output to ble bap sink device
    AUDIO_DEVICE_NUMBER
} audio_device_e;

typedef enum
{
    AUDIO_DC_TIME,
    AUDIO_RAMPIN_TIME,
    AUDIO_ANS1_TIME,
    AUDIO_AEC_TIME,
    AUDIO_UPAGC_TIME,
    AUDIO_ANS2_TIME,
    AUDIO_RAMPOUT_TIME,
    AUDIO_MSBC_ENCODE_TIME,
    AUDIO_UPLINK_TIME,
    AUDIO_MSBC_DECODE_TIME,
    AUDIO_DNAGC_TIME,
    AUDIO_DNLINK_TIME,
    AUDIO_TIME_MAX,
} AUDIO_TIME_TYPE;

void audio_tick_in(uint8_t type);
void audio_tick_out(uint8_t type);
void audio_time_print(void);
void audio_uplink_time_print(void);
void audio_dnlink_time_print(void);
typedef int (*audio_device_input_callback)(audio_server_callback_cmt_t cmd, const uint8_t *buffer, uint32_t size);
typedef void (*pcm_data_fun_cb)(const int16_t *data, uint32_t len, audio_parameter_t *param);

struct audio_device
{
    /*
        int (*input)(audio_server_callback_cmt_t cmd, const uint8_t *buffer, uint32_t size);
        cmd
            only only support as_callback_cmd_data_coming and as_callback_cmd_cache_half_empty for a2dp source
        buffer & size
            only effect when cmd is as_callback_cmd_data_coming

    */
    int (*open)(void *user_data, audio_device_input_callback callback);
    int (*close)(void *user_data);
    uint32_t (*output)(void *user_data, struct rt_ringbuffer *rb);
    void *user_data;
    int (*ioctl)(void *user_data, int cmd, void *val);
};

typedef int (*audio_server_callback_func)(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved);

audio_client_t audio_open(audio_type_t audio_type, audio_rwflag_t rwflag, audio_parameter_t *paramter, audio_server_callback_func callback, void *callback_userdata);
audio_client_t audio_open2(audio_type_t audio_type, audio_rwflag_t rwflag, audio_parameter_t *paramter, audio_server_callback_func callback, void *callback_userdata, audio_device_e fixed_device);

/**
  * @brief  write pcm data to cache
  * @param  handle value return by audio_open
  * @param  data Point to pcm data
  * @param  data_len length of data, should less than cache size
  * @retval int
  *          the retval can be one of the following values:
  *            -2: invalid parameter
  *            -1: the output for this type of audio was suspended by highest priority audio
  *            >=0: the number of bytes was write to cache, caller should check it and try write remain data lator
  */
int audio_write(audio_client_t handle, uint8_t *data, uint32_t data_len);

int audio_read(audio_client_t handle, uint8_t *buf, uint32_t buf_size);

int audio_ioctl(audio_client_t handle, int cmd, void *parameter);

int audio_close(audio_client_t handle);


/**
  * @brief  set public device for all autio type. if not called, default is AUDIO_DEVICE_SPEAKER
            should call by system UI
  * @param  audio_device, audio_device_e
  * @retval int  0 success, other failed
  */
int audio_server_select_public_audio_device(audio_device_e audio_device);

#ifdef AUDIO_USING_MANAGER
/**
* @brief  set audio_type's private audio device to overload audio_type's public audio_device
should call by system UI or app UI
* @param  audio_type audio type
* @param  audio_device  audio_type's audio device
* @retval int  0 success, other failed
*/
int audio_server_select_private_audio_device(audio_type_t audio_type, audio_device_e audio_device);
#else
#define  audio_server_select_private_audio_device(audio_type, audio_device) 0
#endif
/**
  * @brief  register audio device, must not be called before INIT_ENV_EXPORT(audio_server_init)
  * @param  audio_device audio_device_e
  * @param  p_audio_device device parameter
  * @retval int 0 suscess, otehr failed
  */
int audio_server_register_audio_device(audio_device_e audio_device, const struct audio_device *p_audio_device);



/**
  * @brief  Get the maximum volume
  * @retval uint8_t maximum volume
  */
uint8_t audio_server_get_max_volume(void);



/**
  * @brief  set public volume for all audio type
  * @param  volume, should be [0, 15]
  * @retval int 0 suscess, otehr failed
  */
int audio_server_set_public_volume(uint8_t volume);

/**
  * @brief  set private volume for all audio type
  * @param  volume, should be [0, 15]
  * @retval int 0 suscess, otehr failed
  */
int audio_server_set_private_volume(audio_type_t audio_type, uint8_t volume);
uint8_t audio_server_get_private_volume(audio_type_t audio_type);

/**
  * @brief  set mic mute or not
  * @param  is_mute is_mute=1, mute mic, is_mute=0 unmute mic
  * @retval int 0 success
  */
int audio_server_set_public_mic_mute(uint8_t is_mute);

/**
  * @brief  get mic mute state
  * @param  is_mute is_mute=1, mute mic, is_mute=0 unmute mic
  * @retval 1:mute mic, 0:unmute mic
*/
uint8_t audio_server_get_public_mic_mute(void);

int audio_server_set_public_speaker_mute(uint8_t is_mute);
uint8_t audio_server_get_public_speaker_mute(void);

typedef void (*audio_server_listener_func)(uint32_t message, uint32_t reserved);
/**
  * @brief  register server message listener. must not be called before INIT_ENV_EXPORT(audio_server_init) was ran;
  * @param  func callback
  * @param  what_to_listen the messages what to listen, 0--local music message
  * @param  reserved
  * @retval void
  *          if what_to_listen is 0, message parameter in audio_server_listener_func is:
  *            0: local music can't play
  *            1: local music can play
  *          other what_to_listen is not defined now
  */
void audio_server_register_listener(audio_server_listener_func func, uint32_t what_to_listen, uint32_t reserved);

/**
  * @brief  bt voice data coming indication
  * @param  fifo: data pointer
  * @param  len: data length
  * @retval whether or not need downlink processing algorithm
  */
uint8_t audio_server_bt_voice_ind(uint8_t *fifo, uint8_t len);
/**
  * @brief  write pcm data to uplink cache buffer
  * @param  handle value return by audio_open
  * @param  data Point to pcm data
  * @param  data_len length of data, 240 for 16k samplerate, 120 for 8k samplerate
  * @retval int
  *          the retval can be one of the following values:
  *            -2: invalid parameter
  *            -1: the output for this type of audio was suspended by highest priority audio
  *            >=0: the number of bytes was write to cache, caller should check it and try write remain data lator
  */
int audio_hfp_uplink_write(audio_client_t handle, uint8_t *data, uint32_t data_len);

void audio_set_pcm_callback(pcm_data_fun_cb fun);

void auido_gain_pcm(int16_t *data, rt_size_t data_size, uint8_t shift);
void bt_rx_event_to_audio_server(); //only for bt
void bt_tx_event_to_audio_server(); //only for bt


int is_a2dp_working(void);
#ifndef _WIN32
bool audio_device_is_a2dp_sink();
#endif /* _WIN32 */
uint8_t get_server_current_device(void);
uint8_t get_server_current_play_status(void);
uint8_t get_eq_config(audio_type_t type);
void audio_3a_set_bypass(uint8_t is_bypass, uint8_t mic, uint8_t down);

/*micbias using as GPIO power only*/
void micbias_power_off();
void micbias_power_on();

#define BAP_BROADCAST_SINK_CACHE_SIZE        (960 * 4)  // 10ms * 4
void audio_server_seup_ble_bap_src(struct rt_ringbuffer *rb, void (*ble_src_callback)());

#ifdef __cplusplus
}
#endif

#endif

