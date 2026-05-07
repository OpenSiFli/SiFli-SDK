/*
 * SPDX-FileCopyrightText: 2022-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __AUDIO_SERVER_INTERNAL_H__
#define __AUDIO_SERVER_INTERNAL_H__

int get_pdm_volume();

#if defined(AUDIO_RX_USING_I2S) || defined(AUDIO_TX_USING_I2S)
    #if !BSP_ENABLE_I2S_CODEC
        #error "should enable BSP_ENABLE_I2S_CODEC"
    #endif
#endif
#ifdef AUDIO_RX_USING_PDM
    #if !defined(BSP_USING_PDM1) && !defined(BSP_USING_PDM2)
        #error "should enable BSP_USING_PDM1 or BSP_USING_PDM2"
    #endif
#endif

#define AUDIO_SPEAKER_NAME      "audprc"
#define AUDIO_PRC_CODEC_NAME    "audcodec"

#define PRIVATE_DEFAULT_VOLUME              10

#define PDM_NOISE_DROP_FRAMES               6
#ifdef PKG_USING_SOUNDPLUS
    #define MIC_NOISE_DROP_FRAMES           16
    #define CODEC_DATA_UNIT_LEN             (480)
    #define AUDIO_SERVER_STACK_SIZE         (4096)
#else
    #define MIC_NOISE_DROP_FRAMES           25
    #define CODEC_DATA_UNIT_LEN             (320)
    #define AUDIO_SERVER_STACK_SIZE         (9 * 1024)
#endif

#if PKG_USING_ANYKA
    #define DOWNLINK_STACK_SIZE             3000
#else
    #define DOWNLINK_STACK_SIZE             2000
#endif

#define FADE_VOLUME_STEP        4
#define FADE_INTERVAL_MS        10


#define AUDIO_API
#define AUDIO_CLIENT_MAGIC   0x66778899

#define  AUDIO_SERVER_EVENT_CMD             (1 << 0)
#define  AUDIO_SERVER_EVENT_TX_HALF_EMPTY   (1 << 1)
#define  AUDIO_SERVER_EVENT_RX              (1 << 2)
#define  AUDIO_SERVER_EVENT_BT_DOWNLINK     (1 << 3)
#define  AUDIO_SERVER_EVENT_BT_UPLINK       (1 << 4)
#define  AUDIO_SERVER_EVENT_TX_A2DP_SINK    (1 << 5)
#define  AUDIO_SERVER_EVENT_TX_HFP          (1 << 6)
#define  AUDIO_SERVER_EVENT_A2DP_NEXT       (1 << 7) //a2dp to AG
#define  AUDIO_SERVER_EVENT_A2DP_PREV       (1 << 8) //a2dp to AG
#define  AUDIO_SERVER_EVENT_A2DP_PAUSE      (1 << 9) //a2dp to AG
#define  AUDIO_SERVER_EVENT_A2DP_RESUME     (1 << 10)//a2dp to AG
#define  AUDIO_SERVER_EVENT_DOWN_START      (1 << 11)
#define  AUDIO_SERVER_EVENT_DOWN_END        (1 << 12)
#define  AUDIO_SERVER_EVENT_TX_FULL_EMPTY   (1 << 13)


#define AUDIO_SERVER_EVENT_ALL  ( \
                                AUDIO_SERVER_EVENT_CMD| \
                                AUDIO_SERVER_EVENT_TX_HALF_EMPTY| \
                                AUDIO_SERVER_EVENT_TX_FULL_EMPTY| \
                                AUDIO_SERVER_EVENT_RX| \
                                AUDIO_SERVER_EVENT_TX_A2DP_SINK| \
                                AUDIO_SERVER_EVENT_TX_HFP| \
                                AUDIO_SERVER_EVENT_A2DP_NEXT| \
                                AUDIO_SERVER_EVENT_A2DP_PREV| \
                                AUDIO_SERVER_EVENT_A2DP_PAUSE| \
                                AUDIO_SERVER_EVENT_A2DP_RESUME| \
                                0 \
                                )

#define TYPE_TO_MIX_BIT(type)   (1<<(type))
#define AUDIO_MIX_MASK          ((1<<AUDIO_TYPE_NUMBER) - 1)
#define AUDIO_BT_VOICE_BIT      TYPE_TO_MIX_BIT(AUDIO_TYPE_BT_VOICE)
#define AUDIO_BT_MUSIC_BIT      TYPE_TO_MIX_BIT(AUDIO_TYPE_BT_MUSIC)
#define AUDIO_ALARM_BIT         TYPE_TO_MIX_BIT(AUDIO_TYPE_ALARM)
#define AUDIO_NOTIFY_BIT        TYPE_TO_MIX_BIT(AUDIO_TYPE_NOTIFY)
#define AUDIO_LOCAL_MUSIC_BIT   TYPE_TO_MIX_BIT(AUDIO_TYPE_LOCAL_MUSIC)
#define AUDIO_LOCAL_RING_BIT    TYPE_TO_MIX_BIT(AUDIO_TYPE_LOCAL_RING)
#define AUDIO_MODEM_VOICE_BIT   TYPE_TO_MIX_BIT(AUDIO_TYPE_MODEM_VOICE)
#define AUDIO_MIX_ALL           0xFFFF

#define BT_VOICE_MIX_WITH       (0)
#define LOCAL_RING_MIX_WITH     (AUDIO_MIX_ALL)
#define ALARM_MIX_WITH          (AUDIO_MIX_ALL)
#define NOTIFY_MIX_WITH         (AUDIO_MIX_ALL)
#define LOCAL_MUSIC_MIX_WITH    (AUDIO_MIX_ALL)
#define MODEM_VOICE_MIX_WITH    0
#define BT_MUSIC_MIX_WITH       (AUDIO_MIX_ALL)
#define LOCAL_RECORD_MIX_WITH   (AUDIO_MIX_ALL)

#define AUDIO_DATA_LEN          CODEC_DATA_UNIT_LEN


#define PDM_DELAY_SAMPLES       7
#define PDM_STEREO_DELAY_BYTES (PDM_DELAY_SAMPLES * 2 * 2)
#define PDM1_AFTER_PDM2         1 //delete this if PDM2 after PDM1

#define PDM_MOMO_FRAME_SIZE     CODEC_DATA_UNIT_LEN
#define PDM_STEREO_FRAME_SIZE   (PDM_MOMO_FRAME_SIZE * 2)
#define PDM_4MIC_FRAME_SIZE     (PDM_STEREO_FRAME_SIZE * 2)

#define PDM1_DEVICE_NAME    "pdm1"
#define PDM2_DEVICE_NAME    "pdm2"
#define DELAY_SAMPLE    10


enum
{
    DUMP_NONE,
    DUMP_BLE,
    DUMP_UART,
};

typedef struct
{
    uint32_t magic;
    uint16_t type;
    uint16_t len;
    uint32_t no;
    uint8_t  data[AUDIO_DATA_LEN];
} audio_data_t;

typedef struct
{
    void (*func)(void *);
    void *user_data;
} audio_10ms_callback_t;

#define AUDIO_DATA_HEADER_LEN   ((uint32_t)(((audio_data_t *) 0)->data))

struct audio_client_base_t
{
    rt_list_t                   node;

    uint32_t                    magic;
    rt_tick_t                   fade_out_start_tick;
    const char                  *name; //only for debug use
    rt_event_t                  api_event;
    audio_server_callback_func  callback;
    void                        *user_data;
    audio_parameter_t           parameter;
    struct rt_ringbuffer        ring_buf;
    uint8_t                     *ring_pool;
#if SOFTWARE_TX_MIX_ENABLE
    sifli_resample_t            *resample;
    int16_t                     resample_dst[TX_DMA_SIZE];
    uint32_t                    resample_dst_samplerate;
    uint8_t                     resample_dst_ch;
#endif
    audio_rwflag_t              rw_flag;
    audio_type_t                audio_type;
    audio_device_e              device_specified;
    audio_device_e              device_using;
    fade_state_e                fade_out_state;
    uint8_t                     fade_out_index;
    uint8_t                     is_3a_opened;
    uint8_t                     is_suspended;
    uint8_t                     debug_full;
};

#define OPEN_MAP_TX             (1 << 0)
#define OPEN_MAP_RX             (1 << 1)
#define OPEN_MAP_TXRX           (OPEN_MAP_TX|OPEN_MAP_RX)

#define FRAME_DEBUG_MAX         128

#define PDM1_DEVICE_NAM             "pdm1" /* hardware device interface for software pdm device 1 */
#define PDM2_DEVICE_NAME            "pdm2" /* hardware device interface for software pdm device 2 */

typedef struct
{
    struct _audio_device_ctrl_t *parent;
    rt_event_t                  event;
    rt_device_t                 audprc_dev;
    rt_device_t                 audcodec_dev;
    rt_device_t                 pdm1_dev; /* software pdm device 1 */
    rt_device_t                 pdm2_dev; /* software pdm device 2 */
    rt_device_t                 i2s;
    struct rt_ringbuffer        *mic0_rb;
    struct rt_ringbuffer        *mic1_rb;
    uint8_t                     *mixed_4_channel;
    uint8_t                     *tx_data_tmp;
    uint8_t                     *rx_data_tmp;
    uint8_t                     *rx_data_tmp2;
    uint32_t                    tx_samplerate;
    uint32_t                    rx_samplerate;
    int                         last_volume;
    uint32_t                    tx_dma_size;
    uint16_t                    rx_drop_cnt;
    uint8_t                     is_need_3a;
    uint8_t                     tx_channels;
    uint8_t                     rx_channels;
    uint8_t                     opened_map_flag;
    uint8_t                     rx_ready;
    uint8_t                     tx_ready;
    uint8_t                     tx_ref;
    uint8_t                     rx_ref;
    uint8_t                     need_pdm_rx;
    uint8_t                     need_adc_rx;
    uint8_t                     need_i2s_rx;
    uint8_t                     tx_empty_occur;
    uint8_t                     tx_full_occur;
    uint8_t                     tx_enable;
    uint8_t                     rx_uplink_send_start;
    uint8_t                     is_eq_mute_volume;
    uint8_t                     tx_empty_cnt;
    uint8_t                     is_wait_rx_start;
    mics_t                      mic_used;
    uint8_t                     mic0_got;
    uint8_t                     mic1_got;
    uint8_t                     all_mic_channels;
#if DEBUG_FRAME_SYNC
    uint32_t                    debug_tx_index;
    uint32_t                    debug_rx_index;
    uint32_t                    debug_tx_tick[FRAME_DEBUG_MAX];
    uint32_t                    debug_rx_tick[FRAME_DEBUG_MAX];
#endif

} audio_device_speaker_t;

typedef struct _audio_device_ctrl_t
{
    struct audio_device     device; //must be first member
    audio_client_t          opening_client;
    audio_client_t          closing_client;
    rt_list_t               running_client_list;
    audio_device_e          device_type;
    uint32_t                rx_samplerate;
#if SOFTWARE_TX_MIX_ENABLE
    uint8_t                 *tx_mixed_pool;
    struct rt_ringbuffer    tx_mixed_rb;
#endif
    uint32_t                tx_mix_dst_samplerate;
    uint8_t                 tx_mix_dst_channel;
    uint8_t                 is_tx_need_mix;
    uint8_t                 is_busy;
    uint8_t                 tx_count;
    uint8_t                 rx_count;
    uint8_t                 is_registerd;
} audio_device_ctrl_t;

typedef struct
{
    audio_server_listener_func  local_music_listener;
    struct rt_event     event;
    struct rt_event     down_event;
    struct rt_mutex     mutex;
    rt_slist_t          command_slist;
    rt_list_t           suspend_client_list;
    uint8_t             volume;//0~AUDIO_MAX_VOLUME
    uint8_t             private_volume[AUDIO_TYPE_NUMBER];
    uint8_t             is_bt_3a;
    uint8_t             is_bt_music_working;
    uint8_t             is_server_inited;
    uint8_t             public_is_rx_mute;
    uint8_t             public_is_tx_mute;
    uint8_t             is_micbias_using_as_power_on;
    audio_device_e      private_device[AUDIO_TYPE_NUMBER];
    audio_device_e      public_device;

    audio_device_ctrl_t devices_ctrl[AUDIO_DEVICE_NUMBER];

    audio_device_speaker_t device_speaker_private;
#if !MULTI_CLIENTS_AT_WORKING
    audio_client_t      only_one_client;
#endif
} audio_server_t;

typedef enum
{
    AUDIO_CMD_OPEN              = 0,
    AUDIO_CMD_CLOSE             = 1,
    AUDIO_CMD_PAUSE             = 2,
    AUDIO_CMD_MUTE              = 3,
    AUDIO_CMD_DEVICE_PUBLIC     = 4,
    AUDIO_CMD_DEVICE_PRIVATE    = 5,
    AUDIO_CMD_RESUME            = 6,
    AUDIO_CMD_FADE_OUT          = 7,
    AUDIO_CMD_MICBIAS_ON        = 8,
    AUDIO_CMD_MICBIAS_OFF       = 9,
} audio_server_cmd_e;

typedef struct
{
    rt_slist_t              snode;
    audio_client_t          client;
    audio_server_cmd_e      cmd;
} audio_server_cmt_t;

typedef struct
{
    uint8_t     priority;
    uint16_t    can_mix_with;
} audio_mix_policy_t;

#endif
