/**
  ******************************************************************************
  * @file   audio_server.c
  * @author Sifli software development team
  * @brief SIFLI Manage different audio source.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2022 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rtthread.h>

#ifdef SOC_BF0_HCPU
#include <string.h>
#include <stdlib.h>
#include <rtdevice.h>
#include <drv_config.h>
#include "board.h"
#include "audioproc.h"
#include "audio_server.h"
#include "audio_mem.h"
#include "ipc/ringbuffer.h"
#include "bf0_mbox_common.h"
#include "ipc/dataqueue.h"
#include "drivers/audio.h"
#include <audio_server.h>
#include "sifli_resample.h"
#include <gui_app_pm.h>
#include "bf0_pm.h"
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif

#ifdef WEBRTC_ANS_FIX
    #include "webrtc/modules/audio_processing/ns/include/noise_suppression_x.h"
#endif
#ifdef WEBRTC_AECM
    #include "webrtc/modules/audio_processing/aecm/include/echo_control_mobile.h"
#endif
#ifdef WEBRTC_AGC_FIX
    #include "webrtc/modules/audio_processing/agc/legacy/gain_control.h"
#endif

#ifdef PKG_USING_SOUNDPLUS
    #include "Soundplus_adapter.h"
#endif

#define DBG_TAG         "audio"
#define DBG_LVL          LOG_LVL_INFO
#include "log.h"

#include "bf0_hal_audprc.h"
#include "drv_audprc.h"
#include "drv_i2s.h"

/* ---------------------audio server config start-------------------------- */

#define START_RX_IN_TX_INTERUPT     1
#define PDM_DEVICE_NAME             "pdm1"
#undef audio_mem_malloc
#undef audio_mem_free
#undef audio_mem_calloc
#define audio_mem_malloc    rt_malloc
#define audio_mem_free      rt_free
#define audio_mem_calloc    rt_calloc

#define PRIVATE_DEFAULT_VOLUME              10

#ifdef PKG_USING_SOUNDPLUS
    #define CODEC_DATA_UNIT_LEN             (480)
    #define AUDIO_SERVER_STACK_SIZE         (4096)
#else
    #define CODEC_DATA_UNIT_LEN             (320)
    #define AUDIO_SERVER_STACK_SIZE         (9 * 1024)
#endif

#define FADE_VOLUME_STEP        4
#define FADE_INTERVAL_MS        10

#define AUDIO_DATA_CAPTURE_UART

#if defined (AUDIO_DATA_CAPTURE_UART) && defined (RT_USING_FINSH)

#define AUDIO_DATA_LEN          CODEC_DATA_UNIT_LEN

/* ---------------------audio server config end -------------------------- */

typedef struct
{
    uint32_t magic;
    uint16_t type;
    uint16_t len;
    uint32_t no;
    uint8_t  data[AUDIO_DATA_LEN];
} audio_data_t;

#define AUDIO_DATA_HEADER_LEN   ((uint32_t)(((audio_data_t *) 0)->data))

static audio_data_t *p_audio_dump;
static uint32_t     audio_dump_len;
static uint32_t     audio_dump_pos;
static uint32_t     audio_dump_no[ADUMP_NUM];
static rt_timer_t   simu_audio_data_timer;
static bool         audio_dump_log = false;
static rt_sem_t     audio_dump_sem;

static int  audio_data_write_uart(void);
static void audio_data_save_buf(uint8_t type, uint8_t *buf, uint32_t size);
static void audio_data_stop(void);
#endif

static audio_device_e current_audio_device;
static uint8_t current_play_status;
#define g_hardware_mix_enable    0 //mix is left + right, make big volume

enum
{
    DUMP_NONE,
    DUMP_BLE,
    DUMP_UART,
};

/*------------------   define ----------------------*/
#define m_max(a, b)  ((a) > (b) ? (a ): (b))

#if defined(SOFTWARE_TX_MIX_ENABLE) || defined(AUDIO_RX_USING_I2S) || defined(AUDIO_TX_USING_I2S)
    #define TX_DMA_SIZE         (CODEC_DATA_UNIT_LEN)
#else
    #if defined(BT_BAP_BROADCAST_SINK)
        #define TX_DMA_SIZE         (CODEC_DATA_UNIT_LEN)     //48k mono 10ms/3
    #elif defined(BT_BAP_BROADCAST_SOURCE)
        #define TX_DMA_SIZE         (CODEC_DATA_UNIT_LEN * 3) //48k mono 10ms
    #else
        #define TX_DMA_SIZE         (CODEC_DATA_UNIT_LEN * 5)
    #endif
#endif

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
#define  AUDIO_SERVER_EVENT_TX_I2S1         (1 << 14)
#define  AUDIO_SERVER_EVENT_TX_I2S2         (1 << 15)
#define  AUDIO_SERVER_EVENT_TX_BLE_SINK     (1 << 16)

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
                                AUDIO_SERVER_EVENT_TX_I2S1 | \
                                AUDIO_SERVER_EVENT_TX_I2S2 | \
                                AUDIO_SERVER_EVENT_TX_BLE_SINK | \
                                0 \
                                )

#define SPEAKER_TX_BUF_SIZE     (32 * 300) //300ms

/* --------------device name config --------------------------- */
#define AUDIO_SPEAKER_NAME      "audprc"
#define AUDIO_PRC_CODEC_NAME    "audcodec"



/* -------------------------------------config------------------------------------------------------*/

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


struct audio_client_base_t
{
    rt_list_t                   node;

    uint32_t                    magic;
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
    uint8_t                     is_3a_opened;
    uint8_t                     is_fade_vol; // 1--fade out, 2 fade in
    uint8_t                     is_fade_end;
    uint8_t                     fade_vol_steps;
    uint8_t                     is_suspended;
    uint8_t                     is_factory_loopback;
    uint8_t                     debug_full;
};

#define OPEN_MAP_TX             (1 << 0)
#define OPEN_MAP_RX             (1 << 1)
#define OPEN_MAP_TXRX           (OPEN_MAP_TX|OPEN_MAP_RX)

#define FRAME_DEBUG_MAX         128

typedef struct
{
    struct _audio_device_ctrl_t *parent;
#if START_RX_IN_TX_INTERUPT
    rt_event_t                  event;
#endif
    rt_device_t                 audprc_dev;
    rt_device_t                 audcodec_dev;
    rt_device_t                 pdm;
    rt_device_t                 i2s;
    uint8_t                     *tx_data_tmp;
    uint8_t                     *rx_data_tmp;
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
    uint8_t                     rx_channel_num;
    uint8_t                     is_eq_mute_volume;
    uint8_t                     tx_empty_cnt;

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
    uint8_t             volume;//0~15
    uint8_t             private_volume[AUDIO_TYPE_NUMBER];
    uint8_t             is_bt_3a;
    uint8_t             is_bt_music_working;
    uint8_t             is_server_inited;
    uint8_t             public_is_rx_mute;
    uint8_t             public_is_tx_mute;
    audio_device_e      private_device[AUDIO_TYPE_NUMBER];
    audio_device_e      public_device;

    audio_device_ctrl_t devices_ctrl[AUDIO_DEVICE_NUMBER];

    rt_tick_t           last_tick;
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

/* ---------------global var-------------*/
static int audio_pm_debug = 0;
static audio_server_t g_server;
static uint32_t audio_server_stack[AUDIO_SERVER_STACK_SIZE / 4];
static uint32_t bt_downvoice_stack[500];
static struct rt_thread audio_server_tid;
static struct rt_thread bt_downvoice_tid;
static uint8_t *hfp_dev_input_buf;
static uint32_t hfp_dev_input_buf_offset;
static uint8_t g_ae_log = 0;


/* dump debug control*/

audio_dump_ctrl_t audio_dump_debug[ADUMP_NUM];

static bool a2dp_sink_need_trigger = 1;

static void (*ble_tx_dma_callback)();
static device_open_parameter_t g_ble_bap_sink_parameter;
static bool ble_bap_src_enabled = 0;
struct rt_ringbuffer *ble_bap_src_enabled_ring;
static uint32_t ble_sink_low_water_level_times;
static uint32_t ble_sink_high_water_level_times;
static int ble_sink_pll_speed_up;
static rt_tick_t ble_sink_change_ticks;

void audio_server_seup_ble_bap_src(struct rt_ringbuffer *rb, void (*ble_src_callback)())
{
    ble_bap_src_enabled_ring = rb;
    ble_bap_src_enabled = 1;
    ble_tx_dma_callback = ble_src_callback;
}

/*
 if could not mix, than check priority, if priority is same, then new audio suspend old audio
*/
#if SOFTWARE_TX_MIX_ENABLE
static const audio_mix_policy_t mix_policy[AUDIO_TYPE_NUMBER] =
{
    [AUDIO_TYPE_BT_VOICE]      = {100, BT_VOICE_MIX_WITH},
    [AUDIO_TYPE_MODEM_VOICE]   = {94, MODEM_VOICE_MIX_WITH},
    [AUDIO_TYPE_LOCAL_RECORD]  = {90, LOCAL_RECORD_MIX_WITH},
    [AUDIO_TYPE_BT_MUSIC]      = {90, BT_MUSIC_MIX_WITH},
    [AUDIO_TYPE_LOCAL_MUSIC]   = {80, LOCAL_MUSIC_MIX_WITH},
    [AUDIO_TYPE_ALARM]         = {70, ALARM_MIX_WITH},
    [AUDIO_TYPE_TEL_RING]      = {91, LOCAL_RING_MIX_WITH},
    [AUDIO_TYPE_NOTIFY]        = {70, NOTIFY_MIX_WITH},
    [AUDIO_TYPE_LOCAL_RING]    = {70, LOCAL_RING_MIX_WITH},
};
#else
static const audio_mix_policy_t mix_policy[AUDIO_TYPE_NUMBER] =
{
    [AUDIO_TYPE_BT_VOICE]      = {99, 0},
    [AUDIO_TYPE_BT_MUSIC]      = {11, 0},
    [AUDIO_TYPE_ALARM]         = {88, 0},
    [AUDIO_TYPE_NOTIFY]        = {80, 0},
    [AUDIO_TYPE_LOCAL_MUSIC]   = {10, 0},
    [AUDIO_TYPE_LOCAL_RING]    = {92, 0},
    [AUDIO_TYPE_LOCAL_RECORD]  = {93, 0},
    [AUDIO_TYPE_TEL_RING]      = {94, 0},
    [AUDIO_TYPE_MODEM_VOICE]   = {99, 0},
};
#endif

/*------------------local function------*/
static rt_err_t speaker_tx_done(rt_device_t dev, void *buffer);
static rt_err_t mic_rx_ind(rt_device_t dev, rt_size_t size);
static void start_rx(audio_device_speaker_t *my);
static audio_client_t device_get_tx_in_running(audio_device_ctrl_t *device, int index);
static audio_client_t device_get_rx_in_running(audio_device_ctrl_t *device);
static void audio_device_change(audio_server_t *server);
static int audio_write_resample(audio_client_t c, uint8_t *data, uint32_t data_size);

RT_WEAK rt_err_t pm_scenario_start(pm_scenario_name_t scenario)
{
    return 0;
}
RT_WEAK rt_err_t pm_scenario_stop(pm_scenario_name_t scenario)
{
    return 0;
}

static uint8_t is_can_mix(audio_client_t client_old, audio_client_t client_new, audio_device_ctrl_t *device)
{
#if !MULTI_CLIENTS_AT_WORKING
    return 0;
#endif
    RT_ASSERT(client_old && client_new);
    RT_ASSERT(client_old->audio_type < AUDIO_TYPE_NUMBER && client_new->audio_type < AUDIO_TYPE_NUMBER);
    if (device->device_type != AUDIO_DEVICE_SPEAKER
#if TWS_MIX_ENABLE
            && device->device_type != AUDIO_DEVICE_A2DP_SINK
#endif
            && 1)
    {
        LOG_I("device %d not support mix", device->device_type);
        return 0;
    }

    if ((client_old->rw_flag & AUDIO_RX) && (client_new->rw_flag & AUDIO_RX))
    {
        LOG_I("dennied mix two rx");
        return 0;
    }

    if (!(client_old->rw_flag & AUDIO_TX) || !(client_new->rw_flag & AUDIO_TX))
    {
        LOG_I("allow mix rx & tx");
        return 1;
    }

#if SOFTWARE_TX_MIX_ENABLE
    LOG_D("t1=%d mw=0x%x b2=0x%x t2=%d mw=0x%x b1=0x%x", client_old->audio_type,
          mix_policy[client_old->audio_type].can_mix_with,
          TYPE_TO_MIX_BIT(client_old->audio_type),
          client_new->audio_type,
          mix_policy[client_new->audio_type].can_mix_with,
          client_new->audio_type);

    if (mix_policy[client_old->audio_type].can_mix_with & TYPE_TO_MIX_BIT(client_new->audio_type))
    {
        LOG_I("allow, mix %s(%d) with %s(%d)", client_old->name, client_old->audio_type, client_new->name, client_new->audio_type);
        return 1;
    }
#endif
    LOG_I("dennied, mix %s(%d) with %s(%d)", client_old->name, client_old->audio_type, client_new->name, client_new->audio_type);
    return 0;
}

static inline audio_server_t *get_server()
{
    return &g_server;
}
uint8_t get_server_current_device(void)
{
    if (ble_bap_src_enabled)
        return AUDIO_DEVICE_BLE_BAP_SINK;

    return current_audio_device;
}
uint8_t get_server_current_play_status(void)
{
    return current_play_status;
}
static inline void lock()
{
    rt_mutex_take(&g_server.mutex, RT_WAITING_FOREVER);
}
static inline void unlock()
{
    rt_mutex_release(&g_server.mutex);
}

/* -----------------speaker device start----------------- */
static void inline speaker_update_volume(audio_device_speaker_t *my, int16_t spframe[], uint16_t len)
{
    audio_type_t audio_type;
    int8_t  decrease_level = 0;
    int     volx2;
    uint8_t vol = g_server.volume;
    audio_client_t first;
#if 0 //SOFTWARE_TX_MIX_ENABLE
    RT_ASSERT(0); //todo
    first = NULL;
    audio_type = AUDIO_TYPE_LOCAL_MUSIC;
#else
    first = device_get_tx_in_running(my->parent, 0);
    audio_type = first->audio_type;
#endif

    if (!my->parent->is_busy || !my->tx_ref || !first)
    {
        return;
    }
    if (g_server.private_volume[audio_type] != 0xFF)
    {
        vol = g_server.private_volume[audio_type];
    }
#if VOLUME_0_MUTE
    if (vol == 0)
    {
        memset(spframe, 0, len);
    }
#endif
    if (eq_is_working())
    {
        volx2 = eq_get_default_volumex2();
        if (volx2 == MUTE_UNDER_MIN_VOLUME)
        {
            my->is_eq_mute_volume = 1;
            my->last_volume = MUTE_UNDER_MIN_VOLUME;
        }
        else if (my->audcodec_dev)
        {
            if (my->last_volume != volx2)
            {
                LOG_I("eq change volume=%d", volx2);
                rt_device_control(my->audcodec_dev, AUDIO_CTL_SETVOLUME, (void *)volx2);
            }
            my->is_eq_mute_volume = 0;
            my->last_volume = volx2;
        }

    }
    else
    {
        static uint16_t debug_volume = 0;
        if (debug_volume == 0)
        {
            LOG_I("server volume=%d, mute=%d eq_mute=%d",
                  vol,
                  g_server.public_is_tx_mute,
                  my->is_eq_mute_volume);
        }
        debug_volume++;

        if (g_server.public_is_tx_mute)
        {
            memset(spframe, 0, len * 2);
            return;
        }
        else if (my->audcodec_dev)
        {
#if SOFTWARE_TX_MIX_ENABLE
            //todo: multi stream fade out
#else
            if (first->is_fade_vol && !first->is_fade_end)
            {
                rt_tick_t tick = rt_tick_get_millisecond();
                if (tick - g_server.last_tick > FADE_INTERVAL_MS)
                {
                    g_server.last_tick = tick;
                    if (first->is_fade_vol == 1) //fade out
                    {
                        if (first->fade_vol_steps <= vol)
                        {
                            //2 volume step, using 1 is slowly
                            first->fade_vol_steps += FADE_VOLUME_STEP;
                            if (first->fade_vol_steps >= vol)
                            {
                                vol = 0;
                            }
                            else
                            {
                                vol = vol - first->fade_vol_steps;
                            }
                        }
                        else
                        {
                            vol = 0;
                            first->is_fade_end = 1;
                        }
                    }
                    else //fade in
                    {
                        if (first->fade_vol_steps < vol)
                        {
                            first->fade_vol_steps += FADE_VOLUME_STEP;
                            if (first->fade_vol_steps < vol)
                            {
                                vol = first->fade_vol_steps ;
                            }
                        }
                        else
                        {
                            first->is_fade_vol = 0; //fade in end
                            first->is_fade_end = 1;
                        }
                    }
                }
                else
                {
                    return;
                }
            }
            else if (first->is_fade_vol == 1 && first->is_fade_end)
            {
                vol = 0;
            }
#endif
            if (audio_type == AUDIO_TYPE_BT_VOICE)
                volx2 = eq_get_tel_volumex2(vol);
            else
                volx2 = eq_get_music_volumex2(vol);


            if (volx2 == MUTE_UNDER_MIN_VOLUME)
            {
                my->is_eq_mute_volume = 1;

            }
            else
            {
                if (my->last_volume != volx2)
                {
                    LOG_I("not eq change volume=%d", volx2);
                    rt_device_control(my->audcodec_dev, AUDIO_CTL_SETVOLUME, (void *)volx2);
                }
                my->is_eq_mute_volume = 0;
                my->last_volume = volx2;
            }
        }
    }

    if (my->is_eq_mute_volume && !first->is_fade_vol)
    {
        memset(spframe, 0, len * 2);
        return;
    }

    if (vol == 15)
    {
        if (audio_type == AUDIO_TYPE_BT_VOICE)
            decrease_level = eq_get_decrease_level(1, eq_get_tel_volumex2(15));
        else
            decrease_level = eq_get_decrease_level(1, eq_get_music_volumex2(15));

        if (decrease_level == 0)
            return;

        int32_t d1, d2, d3, d4, d5, d6, d0;
        uint16_t n;

        for (n = 0; n < len; n++)
        {
            d1 = (int) spframe[n] << 11;
            d2 = (int) spframe[n] << 10;
            d3 = (int) spframe[n] << 9;
            d4 = (int) spframe[n] << 8;
            d5 = (int) spframe[n] << 7;
            d6 = (int) spframe[n] << 6;

            switch (decrease_level)
            {
            case 1:
                d0 = d3 + d5;
                break;
            case 2:
                d0 = d2 + d4;
                break;
            case 3:
                d0 = d2 + d3 + d4 + d5;
                break;
            case 4:
                d0 = d1 + d4 + d5 + d6;
                break;
            case 5:
                d0 = d1 + d2 + d6;
                break;
            case 6:
                d0 = d1 + d2 + d3 + d5;
                break;
            default:
                d0 = 0;
                break;
            }
            spframe[n] = (int16_t)(((((int32_t)spframe[n]) << 16) - d0) >> 16);
        }
    }
}


static inline void process_speaker_tx(audio_server_t *server, audio_device_speaker_t *my)
{
    struct rt_ringbuffer  *p_rb;
    rt_uint32_t  getnum;
    uint8_t is_suspended;
    uint8_t has_callback;

    audio_client_t first = device_get_tx_in_running(my->parent, 0);
    if ((my->opened_map_flag & OPEN_MAP_TX) == 0 || !my->tx_ref || !first)
    {
        //LOG_I("invlide tx ref=%d map=%d first=%p", my->tx_ref, (my->opened_map_flag & OPEN_MAP_TX), first);
        return;
    }
    is_suspended = first->is_suspended;
    has_callback = (first->callback != NULL);

#if SOFTWARE_TX_MIX_ENABLE
    audio_client_t second = device_get_tx_in_running(my->parent, 1);
    if (second)
    {
        is_suspended = (is_suspended && second->is_suspended);
        has_callback = (has_callback || (second->callback != NULL));
    }
    p_rb = &my->parent->tx_mixed_rb;
#else
    p_rb = &first->ring_buf;
#endif
#if START_RX_IN_TX_INTERUPT
    if (my->tx_ready == 1)
    {
        my->tx_ready++;
        start_rx(my);
        rt_event_send(my->event, 1);
    }
#endif

    if (my->tx_enable == 0)
    {
        memset(my->tx_data_tmp, 0, my->tx_dma_size);
        if (my->is_need_3a)
        {
            audio_3a_far_put(my->tx_data_tmp, CODEC_DATA_UNIT_LEN);
        }
#if defined(AUDIO_TX_USING_I2S)
        bf0_i2s_device_write(my->i2s, 0, my->tx_data_tmp, my->tx_dma_size);
#else
        bf0_audprc_device_write(my->audprc_dev, 0, my->tx_data_tmp, my->tx_dma_size);
#endif
    }
    else
    {
        if (rt_ringbuffer_data_len(p_rb) < my->tx_dma_size)
        {
            memset(my->tx_data_tmp, 0, my->tx_dma_size);
            if (my->is_need_3a)
            {
                audio_3a_far_put(my->tx_data_tmp, CODEC_DATA_UNIT_LEN);
            }

#if defined(AUDIO_TX_USING_I2S)
            bf0_i2s_device_write(my->i2s, 0, my->tx_data_tmp, my->tx_dma_size);
#else
            bf0_audprc_device_write(my->audprc_dev, 0, my->tx_data_tmp, my->tx_dma_size);
#endif
            my->tx_empty_cnt++;
            if (!is_suspended && g_ae_log)
            {
                // Audio Empty
                rt_kprintf("AE times %d\r\n", my->tx_empty_cnt);
            }
#if SOFTWARE_TX_MIX_ENABLE
            rt_event_send(&server->event, AUDIO_SERVER_EVENT_TX_HALF_EMPTY);
#endif
        }
        else
        {
            my->tx_empty_cnt = 0;
            getnum = rt_ringbuffer_get(p_rb, my->tx_data_tmp, my->tx_dma_size);
            RT_ASSERT(getnum == my->tx_dma_size);
            speaker_update_volume(my, (int16_t *)my->tx_data_tmp, my->tx_dma_size / 2);
            if (my->is_need_3a)
            {
#if DEBUG_FRAME_SYNC
                if (my->debug_tx_index < FRAME_DEBUG_MAX)
                {
                    my->debug_tx_tick[my->debug_tx_index++] = HAL_DBG_DWT_GetCycles();
                }
                else
                {
                    my->debug_tx_index = 0;
                }
#endif

                audio_3a_far_put(my->tx_data_tmp, my->tx_dma_size);
                my->rx_uplink_send_start = 1;
            }
            //audprc_get_tx_channel();
#if defined(AUDIO_TX_USING_I2S)
            bf0_i2s_device_write(my->i2s, 0, my->tx_data_tmp, my->tx_dma_size);
#else
            bf0_audprc_set_tx_channel(0);
            bf0_audprc_device_write(my->audprc_dev, 0, my->tx_data_tmp, my->tx_dma_size);
#endif
        }
#if SOFTWARE_TX_MIX_ENABLE
        if (rt_ringbuffer_space_len(p_rb) >= rt_ringbuffer_get_size(p_rb) / 2)
        {
            rt_event_send(&server->event, AUDIO_SERVER_EVENT_TX_HALF_EMPTY);
        }
#else
        if (has_callback && !is_suspended) //no need wakeup audio server
        {
            if (rt_ringbuffer_data_len(p_rb) < my->tx_dma_size)
            {
                rt_event_send(&server->event, AUDIO_SERVER_EVENT_TX_FULL_EMPTY);
            }
            else if (rt_ringbuffer_space_len(p_rb) >= rt_ringbuffer_get_size(p_rb) / 2)
            {
                rt_event_send(&server->event, AUDIO_SERVER_EVENT_TX_HALF_EMPTY);
            }
        }
#endif
    }
}

#if DEBUG_FRAME_SYNC
void save_mic_tick()
{
    audio_device_speaker_t *my = &g_server.device_speaker_private;
    if (my->debug_rx_index < FRAME_DEBUG_MAX)
    {
        my->debug_rx_tick[my->debug_rx_index++] = HAL_DBG_DWT_GetCycles();
    }
    else
        my->debug_rx_index = 0;
}
#endif
static inline void process_speaker_rx(audio_server_t *server, audio_device_speaker_t *my)
{
    audio_client_t client = device_get_rx_in_running(my->parent);
    if ((my->opened_map_flag & OPEN_MAP_RX) == 0 || !client)
    {
        LOG_I("invlide rx event");
        return;
    }

#if defined (AUDIO_DATA_CAPTURE_UART) && defined (RT_USING_FINSH)
    if (p_audio_dump && audio_dump_len)
    {
        audio_data_write_uart();
    }
#endif
    rt_size_t datasize, putsize, readlen = CODEC_DATA_UNIT_LEN;
    rt_size_t len;

    audio_tick_in(AUDIO_UPLINK_TIME);

    readlen = CODEC_DATA_UNIT_LEN;
#if defined(AUDIO_RX_USING_I2S)
    len = rt_device_read(my->i2s, 0, my->rx_data_tmp, readlen);
#elif defined(AUDIO_RX_USING_PDM)
    len = rt_device_read(my->pdm, 0, my->rx_data_tmp, readlen);
#else
    len = rt_device_read(my->audprc_dev, 0, my->rx_data_tmp, readlen);
#endif
    if (len != readlen)
    {
        LOG_I("read error len=%d", len);
        goto Exit;
    }

    LOG_D("mic_rx_ind readlen:%d", len);
    if (my->is_need_3a)
    {
        if (my->rx_drop_cnt < 25)
        {
            my->rx_drop_cnt++;
            memset(my->rx_data_tmp, 0, readlen);
        }

        audio_3a_uplink(my->rx_data_tmp, readlen, server->public_is_rx_mute, server->is_bt_3a);
    }
    else
    {
        audio_dump_data(ADUMP_AUDPRC, my->rx_data_tmp, len);
    }

    if (client->is_factory_loopback)
    {
        uint8_t gain = client->is_factory_loopback & 0x7F;
        auido_gain_pcm((int16_t *)my->rx_data_tmp, len, gain);
        rt_ringbuffer_put(&client->ring_buf, my->rx_data_tmp, len);
    }

    if (!server->is_bt_3a)
    {
        audio_server_coming_data_t data;
        data.data = my->rx_data_tmp;
        data.data_len = len;
        data.reserved = 0;
        if (server->public_is_rx_mute)
        {
            memset(my->rx_data_tmp, 0, len);
        }
        if (client->callback)
        {
            client->callback(as_callback_cmd_data_coming, client->user_data, (uint32_t)&data);
        }
    }
Exit:
    audio_tick_out(AUDIO_UPLINK_TIME);
    audio_uplink_time_print();
}

static uint8_t threshold_last;
extern void pll_freq_grade_set(uint8_t gr);
static uint8_t pll_add = 1, pll_sub = 1;

#if ALL_CLK_USING_PLL
void audio_pll_dynamic_regulation(audio_device_speaker_t *my, uint16_t fifo_size)
{
    struct rt_ringbuffer *rb = NULL;
#if SOFTWARE_TX_MIX_ENABLE
    rb = &my->parent->tx_mixed_rb;
#else
    audio_client_t first = device_get_tx_in_running(my->parent, 0);
    rb = &first->ring_buf;
#endif
    uint32_t rb_size = 0, rb_used = 0;
    uint8_t threshold_all = 0, threshold_cur = 0;

    rb_size = rt_ringbuffer_get_size(rb);
    rb_used = rt_ringbuffer_data_len(rb);

    threshold_all = rb_size / fifo_size;
    threshold_cur = rb_used / fifo_size;

    if ((threshold_cur >= threshold_all * 5 / 8) && (threshold_last < threshold_all * 5 / 8) && pll_add)
    {
        pll_freq_grade_set(1); //PLL_ADD_TWO_HUND_PPM
        pll_add = 0;
        pll_sub = 1;
        //LOG_I("PLL_ADD_TWO_HUND_PPM all:%d, cur:%d, last:%d", threshold_all, threshold_cur, threshold_last);
    }

    if ((threshold_cur <= threshold_all * 3 / 8) && (threshold_last > threshold_all * 3 / 8) && pll_sub)
    {
        pll_freq_grade_set(3); //PLL_SUB_TWO_HUND_PPM
        pll_sub = 0;
        pll_add = 1;
        //LOG_I("PLL_SUB_TWO_HUND_PPM all:%d, cur:%d, last:%d", threshold_all, threshold_cur, threshold_last);
    }

    threshold_last = threshold_cur;
}
#endif

void speaker_ring_put(uint8_t *fifo, uint16_t fifo_size)
{
    struct rt_ringbuffer *rb;
    rt_size_t putsize;
    audio_device_ctrl_t *device = &g_server.devices_ctrl[AUDIO_DEVICE_SPEAKER];
#if SOFTWARE_TX_MIX_ENABLE
    audio_client_t client = device_get_tx_in_running(device, 0);
    if (!client || client->audio_type != AUDIO_TYPE_BT_VOICE)
    {
        client = device_get_tx_in_running(device, 1);
    }
#else
    audio_client_t client = device_get_tx_in_running(device, 0);
#endif
    if (!client || client->audio_type != AUDIO_TYPE_BT_VOICE || !device->is_busy)
    {
        return;
    }

    rb = &client->ring_buf;

    audio_device_speaker_t *my = &g_server.device_speaker_private;


#if ALL_CLK_USING_PLL
    //audio_pll_dynamic_regulation(my, fifo_size);
#endif

    if (my->tx_full_occur == 0 && my->tx_empty_occur == 0)
    {
        rt_size_t space = rt_ringbuffer_space_len(rb);
        rt_size_t data_len = rt_ringbuffer_data_len(rb);

        if (space < fifo_size)
        {
            LOG_I("speaker buffer full");
            my->tx_full_occur = 1;
            my->tx_empty_occur = 0;
            my->tx_enable = 1;
            return;
        }
        else if (data_len < fifo_size)
        {
            my->tx_full_occur = 0;
            my->tx_empty_occur = 1;
            my->tx_enable = 0;
            rt_ringbuffer_put(rb, fifo, fifo_size);
            return;
        }
        else
        {
            rt_ringbuffer_put(rb, fifo, fifo_size);
            my->tx_enable = 1;
            return;
        }
    }

    if (my->tx_full_occur)
    {
        if (rt_ringbuffer_space_len(rb) >= rt_ringbuffer_get_size(rb) / 2)
        {
            my->tx_full_occur = 0;
            my->tx_enable = 1;
        }
        return;
    }

    if (my->tx_empty_occur)
    {
        rt_ringbuffer_put(rb, fifo, fifo_size);
        if (rt_ringbuffer_data_len(rb) >= rt_ringbuffer_get_size(rb) / 2)
        {
            my->tx_empty_occur = 0;
            my->tx_enable = 1;
        }
        return;
    }
}

#if defined(AUDIO_RX_USING_I2S) || defined(AUDIO_TX_USING_I2S)

static void i2s_config(audio_device_speaker_t *my, bool is_tx)
{
    my->i2s = rt_device_find("i2s2");
    if (!my->i2s)
    {
        LOG_I("i2s not find");
        RT_ASSERT(0);
    }
    if (RT_EOK != rt_device_open(my->i2s, RT_DEVICE_OFLAG_RDWR))
    {
        RT_ASSERT(0);
    }

    struct rt_audio_caps caps =
    {
        .main_type = AUDIO_TYPE_INPUT,
        .sub_type = AUDIO_DSP_PARAM,
        .udata.config.channels = my->rx_channels,
        .udata.config.samplefmt = 16,
        .udata.config.samplerate = my->rx_samplerate,
    };
    if (is_tx)
    {
        caps.udata.config.channels = my->tx_channels;
        caps.udata.config.samplerate = my->tx_samplerate;
    }
    LOG_I("i2s: samplerate=%d ch=%d", caps.udata.config.samplerate, caps.udata.config.channels);
    if (RT_EOK != rt_device_control(my->i2s, AUDIO_CTL_CONFIGURE, &caps))
    {
        LOG_E("Fail to control i2s fmt");
    }

    caps.main_type = AUDIO_TYPE_INPUT;
    caps.sub_type = AUDIO_DSP_MODE;
    if (is_tx)
        caps.udata.value = 0;   /*set 0, because i2s speaker use master mode */
    else
        caps.udata.value = 1;   //0 master mode, 1 slave mode
    if (RT_EOK != rt_device_control(my->i2s, AUDIO_CTL_CONFIGURE, &caps))
    {
        LOG_E("Fail to control i2s mode");
    }

    int inter = 0; // 0:dma 1:audprc
    rt_device_control(my->i2s, AUDIO_CTL_SETINPUT, (void *)inter);
    inter = 0;
    rt_device_control(my->i2s, AUDIO_CTL_SETOUTPUT, (void *)inter);
}
#endif

static void config_tx(audio_device_speaker_t *my, audio_client_t client)
{
#if defined(AUDIO_TX_USING_I2S)
    i2s_config(my, 1);
    rt_device_set_tx_complete(my->i2s, speaker_tx_done);
#else
    LOG_I("config tx--set callback");
    rt_device_set_tx_complete(my->audprc_dev, speaker_tx_done);

    rt_device_control(my->audprc_dev, AUDIO_CTL_SET_TX_DMA_SIZE, (void *)my->tx_dma_size);

#define     mixer_sel  0x5050
    struct rt_audio_caps caps;
    struct rt_audio_sr_convert cfg;
    int stream;
    /* mix left & right channel to mono channel output, too big volume sometime */
    int     out_sel = 0x5050;
    if (my->tx_channels == 2 && g_hardware_mix_enable)
    {
        out_sel = 0x5010; //mix left & right to speaker.  speaker pcm = left pcm + right pcm
    }

    /*set output: codec/mem/i2s*/
    rt_device_control(my->audcodec_dev, AUDIO_CTL_SETOUTPUT, (void *)AUDPRC_TX_TO_CODEC);

    caps.main_type = AUDIO_TYPE_OUTPUT;

    caps.sub_type = (1 << HAL_AUDCODEC_DAC_CH0);
#if BSP_ENABLE_DAC2
    caps.sub_type |= (1 << HAL_AUDCODEC_DAC_CH1);
#endif

    caps.udata.config.channels   = 1; // L,R,L,R,L,R, ......
    caps.udata.config.samplerate = my->tx_samplerate;
    caps.udata.config.samplefmt = 16; //8 16 24 or 32
    LOG_I("prc_codec : sub_type=%d channel %d, samplerate %d, bits %d", caps.sub_type, caps.udata.config.channels,
          caps.udata.config.samplerate, caps.udata.config.samplefmt);
    rt_device_control(my->audcodec_dev, AUDIO_CTL_CONFIGURE, &caps);

    //LOG_I("audprc output to audcodec");
    rt_device_control(my->audprc_dev, AUDIO_CTL_SETOUTPUT, (void *)AUDPRC_TX_TO_CODEC);

    cfg.channel = my->tx_channels;
    cfg.source_sr = my->tx_samplerate;
    cfg.dest_sr = my->tx_samplerate;
    LOG_I("speaker OUTPUTSRC channel=%d in_rate=%d out_rate=%d", cfg.channel, cfg.source_sr, cfg.dest_sr);
    rt_device_control(my->audprc_dev, AUDIO_CTL_OUTPUTSRC, (void *)(&cfg));

    LOG_I("speaker select=0x%x mixer=0x%x", out_sel, mixer_sel);
    caps.main_type = AUDIO_TYPE_SELECTOR;
    caps.sub_type = 0xFF;
    caps.udata.value   = out_sel;
    rt_device_control(my->audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    caps.main_type = AUDIO_TYPE_MIXER;
    caps.sub_type = 0xFF;
    caps.udata.value   = mixer_sel;
    rt_device_control(my->audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    // data source format
    caps.main_type = AUDIO_TYPE_OUTPUT;
    caps.sub_type = HAL_AUDPRC_TX_CH0;
    caps.udata.config.channels   = my->tx_channels;
    caps.udata.config.samplerate = my->tx_samplerate;
    caps.udata.config.samplefmt = 16;
    LOG_I("tx[0]: sub_type %d, ch %d, samrate %d, bits %d\n",
          caps.sub_type,
          caps.udata.config.channels,
          caps.udata.config.samplerate,
          caps.udata.config.samplefmt);

    rt_device_control(my->audprc_dev, AUDIO_CTL_CONFIGURE, &caps);

    if (eq_is_working())
    {
        int volx2 = eq_get_default_volumex2();
        my->is_eq_mute_volume = 0;
        my->last_volume = volx2;
        if (volx2 == MUTE_UNDER_MIN_VOLUME)
        {
            my->is_eq_mute_volume = 1;
        }
        LOG_I("eq init volume=%d", volx2);
        rt_device_control(my->audcodec_dev, AUDIO_CTL_SETVOLUME, (void *)volx2);
    }
    else
    {
        uint8_t vol_level  = audio_server_get_private_volume(client->audio_type);
        int volumex2 = eq_get_music_volumex2(vol_level);
        if (my->tx_samplerate == 16000 || my->tx_samplerate == 8000)
            volumex2 = eq_get_tel_volumex2(vol_level);

        LOG_I("no eq init volume=%d", volumex2);
        my->last_volume = MUTE_UNDER_MIN_VOLUME;
        rt_device_control(my->audcodec_dev, AUDIO_CTL_SETVOLUME, (void *)volumex2);
    }
#endif
}

static void config_rx(audio_device_speaker_t *my)
{
#if defined(AUDIO_RX_USING_PDM)
    //config PDM
    LOG_I("config pdm");

    my->pdm = rt_device_find(PDM_DEVICE_NAME);
    if (my->pdm)
    {
        extern int get_pdm_volume();

#if MICBIAS_USING_AS_PDM_POWER
        micbias_power_on();
#endif

        rt_device_init(my->pdm);
        rt_device_open(my->pdm, RT_DEVICE_FLAG_RDONLY);
        rt_device_set_rx_indicate(my->pdm, mic_rx_ind);
        struct rt_audio_caps caps;
        caps.main_type = AUDIO_TYPE_INPUT;
        caps.sub_type = AUDIO_DSP_PARAM;
        caps.udata.config.samplefmt = PDM_CHANNEL_DEPTH_16BIT;
        caps.udata.config.samplerate = PDM_SAMPLE_16KHZ;
        caps.udata.config.channels = 1; //2
        rt_device_control(my->pdm, AUDIO_CTL_CONFIGURE, &caps);
        int val_db = get_pdm_volume();
        LOG_I("pdm gain=%d * 0.5db", val_db);
        rt_device_control(my->pdm, AUDIO_CTL_SETVOLUME, (void *)val_db);
        int stream = AUDIO_STREAM_PDM_PRESTART;
        LOG_I("pdm rx pre start=0x%x", stream);
        rt_device_control(my->pdm, AUDIO_CTL_START, &stream);
        my->need_pdm_rx = 1;
    }
    else
    {
        RT_ASSERT(0);
    }
#elif defined(AUDIO_RX_USING_I2S)
    i2s_config(my, 0);
    rt_device_set_rx_indicate(my->i2s, mic_rx_ind);
    my->need_i2s_rx = 1;
#else
    LOG_I("config rx--set callback");
    rt_device_set_rx_indicate(my->audprc_dev, mic_rx_ind);

    //config ADC
    struct rt_audio_caps caps;
    int stream;
    rt_device_control(my->audcodec_dev, AUDIO_CTL_SETINPUT, (void *)AUDPRC_RX_FROM_CODEC);
    caps.main_type = AUDIO_TYPE_INPUT;
    caps.sub_type = 1 << HAL_AUDCODEC_ADC_CH0;
    caps.udata.config.channels   = my->rx_channels;
    caps.udata.config.samplerate = my->rx_samplerate;
    caps.udata.config.samplefmt = 16; //8 16 24 or 32
    rt_device_control(my->audcodec_dev, AUDIO_CTL_CONFIGURE, &caps);

    LOG_I("codec input parameter:sub_type=%d channels %d, rate %d, bits %d", caps.sub_type, caps.udata.config.channels,
          caps.udata.config.samplerate, caps.udata.config.samplefmt);

    rt_device_control(my->audprc_dev, AUDIO_CTL_SETINPUT, (void *)AUDPRC_RX_FROM_CODEC);

    caps.main_type = AUDIO_TYPE_INPUT;
    caps.sub_type = HAL_AUDPRC_RX_CH0 - HAL_AUDPRC_RX_CH0;
    caps.udata.config.channels   = my->rx_channels;
    caps.udata.config.samplerate = my->rx_samplerate;
    caps.udata.config.samplefmt = 16;
    LOG_I("mic input:rx channel %d, channels %d, rate %d, bitwidth %d", 0, caps.udata.config.channels,
          caps.udata.config.samplerate, caps.udata.config.samplefmt);
    rt_device_control(my->audprc_dev, AUDIO_CTL_CONFIGURE, &caps);
    my->need_adc_rx = 1;
#endif
}

static void start_rx(audio_device_speaker_t *my)
{
    int stream;
    LOG_I("%s need_adc_rx=%d", __FUNCTION__, my->need_adc_rx);
    if (my->need_adc_rx)
    {
        my->need_adc_rx = 0;
        stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDCODEC_ADC_CH0) << 8);
        rt_device_control(my->audcodec_dev, AUDIO_CTL_START, &stream);
        stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDPRC_RX_CH0) << 8);
        rt_device_control(my->audprc_dev, AUDIO_CTL_START, &stream);
    }

    if (my->need_pdm_rx)
    {
        my->need_pdm_rx = 0;
        stream = AUDIO_STREAM_PDM_START;
        rt_device_control(my->pdm, AUDIO_CTL_START, &stream);
    }
    if (my->need_i2s_rx)
    {
        stream = AUDIO_STREAM_RECORD;
        rt_device_control(my->i2s, AUDIO_CTL_START, &stream);
    }
}

static void start_txrx(audio_device_speaker_t *my)
{
    int stream;
#if defined(AUDIO_RX_USING_PDM)
    //7 DAC start
    stream = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDPRC_TX_CH0) << 8);
    rt_device_control(my->audprc_dev, AUDIO_CTL_START, (void *)&stream);
    stream = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
    rt_device_control(my->audcodec_dev, AUDIO_CTL_START, &stream);
    my->opened_map_flag |= OPEN_MAP_TX;
    my->tx_ready = 1;
#if START_RX_IN_TX_INTERUPT
    //wait rx start
    rt_err_t got = rt_event_recv(my->event, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 1000, NULL) ;
    LOG_I("got rx start %d", got);
#else
    if (my->need_pdm_rx)
    {
        my->need_pdm_rx = 0;
        stream = AUDIO_STREAM_PDM_START;
        rt_device_control(my->pdm, AUDIO_CTL_START, &stream);
    }
#endif
#elif defined(AUDIO_RX_USING_I2S)
    if (my->need_i2s_rx)
    {
        my->need_i2s_rx = 0;
        stream = AUDIO_STREAM_RECORD;
        rt_device_control(my->i2s, AUDIO_CTL_START, &stream);
        my->opened_map_flag |= OPEN_MAP_TX;
        my->tx_ready = 1;
    }
#else
    //6. DAC mute
    rt_device_control(my->audcodec_dev, AUDIO_CTL_MUTE, (void *)1);

#if START_RX_IN_TX_INTERUPT
    //7 DAC start
    stream = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDPRC_TX_CH0) << 8);
    LOG_I("speaker START stream=0x%x", stream);
    rt_device_control(my->audprc_dev, AUDIO_CTL_START, (void *)&stream);
    stream = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
    LOG_I("codec START stream=0x%x", stream);
    rt_device_control(my->audcodec_dev, AUDIO_CTL_START, &stream);
    my->opened_map_flag |= OPEN_MAP_TX;
    my->tx_ready = 1;
    //wait rx start
    LOG_I("wait rx start");
    rt_err_t got = rt_event_recv(my->event, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 1000, NULL) ;
    LOG_I("got rx start %d", got);
#else
    LOG_I("start txrx");
    int stream_audprc, stream_audcodec;
    rt_base_t level = rt_hw_interrupt_disable();
    stream_audcodec = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
    stream_audprc   = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDPRC_TX_CH0) << 8);
    if (my->need_adc_rx)
    {
        my->need_adc_rx = 0;
        stream_audcodec = AUDIO_STREAM_RXandTX | ((1 << HAL_AUDCODEC_ADC_CH0) << 8) | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
        stream_audprc   = AUDIO_STREAM_RXandTX | ((1 << HAL_AUDPRC_RX_CH0) << 8) | ((1 << HAL_AUDPRC_TX_CH0) << 8);
    }
    rt_device_control(my->audcodec_dev, AUDIO_CTL_START, &stream_audcodec);
    rt_device_control(my->audprc_dev, AUDIO_CTL_START, &stream_audprc);
    rt_hw_interrupt_enable(level);
    rt_thread_mdelay(10);
    my->opened_map_flag |= OPEN_MAP_TX;
    my->tx_ready = 1;
#endif //START_RX_IN_TX_INTERUPT
#endif //AUDIO_RX_USING_PDM
}

static rt_err_t micbias_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_kprintf("micbias interrupt\n");
    return 0;
}

AUDIO_API void micbias_power_on()
{
    int stream;
    rt_err_t err;
    audio_server_t *server = get_server();
    audio_device_speaker_t *my = &server->device_speaker_private;
    LOG_I("%s 0x%p", __FUNCTION__, my->audcodec_dev);
    if (!my->audprc_dev)
    {
        my->audprc_dev = rt_device_find(AUDIO_SPEAKER_NAME);
        RT_ASSERT(my->audprc_dev);
        {
            err = rt_device_open(my->audprc_dev, RT_DEVICE_FLAG_RDWR);
            RT_ASSERT(RT_EOK == err);
        }

        my->audcodec_dev = rt_device_find(AUDIO_PRC_CODEC_NAME);
        RT_ASSERT(my->audcodec_dev);
        err = rt_device_open(my->audcodec_dev, RT_DEVICE_FLAG_WRONLY);
        RT_ASSERT(RT_EOK == err);
    }
    {
        rt_device_set_rx_indicate(my->audprc_dev, micbias_rx_ind);
        //config ADC
        struct rt_audio_caps caps;
        int stream;
        rt_device_control(my->audcodec_dev, AUDIO_CTL_SETINPUT, (void *)AUDPRC_RX_FROM_CODEC);
        caps.main_type = AUDIO_TYPE_INPUT;
        caps.sub_type = 1 << HAL_AUDCODEC_ADC_CH0;
        caps.udata.config.channels   = 1;
        caps.udata.config.samplerate = 8000;
        caps.udata.config.samplefmt = 16;
        rt_device_control(my->audcodec_dev, AUDIO_CTL_CONFIGURE, &caps);

        LOG_I("codec input parameter:sub_type=%d channels %d, rate %d, bits %d", caps.sub_type, caps.udata.config.channels,
              caps.udata.config.samplerate, caps.udata.config.samplefmt);

        rt_device_control(my->audprc_dev, AUDIO_CTL_SETINPUT, (void *)AUDPRC_RX_FROM_CODEC);

        caps.main_type = AUDIO_TYPE_INPUT;
        caps.sub_type = HAL_AUDPRC_RX_CH0 - HAL_AUDPRC_RX_CH0;
        caps.udata.config.channels   = 1;
        caps.udata.config.samplerate = 8000;
        caps.udata.config.samplefmt = 16;
        LOG_I("mic input:rx channel %d, channels %d, rate %d, bitwidth %d", 0, caps.udata.config.channels,
              caps.udata.config.samplerate, caps.udata.config.samplefmt);
        rt_device_control(my->audprc_dev, AUDIO_CTL_CONFIGURE, &caps);
        stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDCODEC_ADC_CH0) << 8);
        rt_device_control(my->audcodec_dev, AUDIO_CTL_START, &stream);
        stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDPRC_RX_CH0) << 8);
        rt_device_control(my->audprc_dev, AUDIO_CTL_START, &stream);
        HAL_NVIC_DisableIRQ(AUDPRC_RX0_DMA_IRQ);
    }
}

AUDIO_API void micbias_power_off()
{
    audio_server_t *server = get_server();
    audio_device_speaker_t *my = &server->device_speaker_private;
    int stream_audcodec = AUDIO_STREAM_RECORD | ((1 << HAL_AUDCODEC_ADC_CH0) << 8);
    int stream_audprc = AUDIO_STREAM_RECORD | ((1 << HAL_AUDPRC_RX_CH0) << 8);
    LOG_I("%s 0x%p", __FUNCTION__, my->audprc_dev);
    if (my->audprc_dev)
    {
        rt_device_control(my->audcodec_dev, AUDIO_CTL_STOP, &stream_audcodec);
        rt_device_control(my->audprc_dev, AUDIO_CTL_STOP, &stream_audprc);
        bf0_disable_pll();
        rt_device_close(my->audcodec_dev);
        rt_device_close(my->audprc_dev);
        my->audcodec_dev = NULL;
        my->audprc_dev = NULL;
    }
}

static int audio_device_speaker_open(void *user_data, audio_device_input_callback callback)
{
    int stream;
    uint8_t need_tx_init = 0, need_rx_init = 0;
    rt_err_t err;
    audio_device_ctrl_t *device = (audio_device_ctrl_t *)user_data;
    audio_server_t *server = get_server();
    audio_client_t client = device->opening_client;

    RT_ASSERT(client);
    RT_ASSERT(device == &server->devices_ctrl[AUDIO_DEVICE_SPEAKER]);

    audio_device_speaker_t *my  = &server->device_speaker_private;
    if (!my->event)
    {
        my->event = rt_event_create("speaker", RT_IPC_FLAG_FIFO);
        RT_ASSERT(my->event);
    }

    LOG_I("%s in r=%d t=%d c=0x%p f=%d", __FUNCTION__, my->rx_ref, my->tx_ref, client, client->rw_flag);
    if (client->rw_flag & AUDIO_TX)
    {
        if (!my->tx_ref)
        {
            RT_ASSERT(!device->tx_count);
            need_tx_init = 1;
        }

        my->tx_ref++;
    }
    if (client->rw_flag & AUDIO_RX)
    {
        RT_ASSERT(!my->rx_ref);
        my->rx_ref++;
        RT_ASSERT(!device->rx_count);
        need_rx_init = 1;
        my->rx_drop_cnt = 0;
    }
    LOG_I("need init tx=%d rx=%d", need_tx_init, need_rx_init);

    client->is_suspended = 0;

    my->is_need_3a = client->parameter.is_need_3a;

    if (client->audio_type == AUDIO_TYPE_BT_MUSIC)
    {
        server->is_bt_music_working = 1;
    }

    // 2. prepare hardware memory
    if (need_tx_init)
    {
        my->tx_dma_size = TX_DMA_SIZE;
        my->tx_channels    = client->parameter.write_channnel_num;
        my->tx_samplerate  = client->parameter.write_samplerate;
        my->tx_empty_occur = 1;
        my->tx_enable      = 1;
        RT_ASSERT(!my->tx_data_tmp);
        // prepare audio 3a
        if (client->audio_type == AUDIO_TYPE_BT_VOICE || my->is_need_3a)
        {
            my->is_need_3a = 1; //audio_open(AUDIO_TYPE_BT_VOICE, ) not set is_need_3a now
            my->tx_dma_size = CODEC_DATA_UNIT_LEN;
            if (client->audio_type == AUDIO_TYPE_BT_VOICE)
            {
                RT_ASSERT(need_rx_init);
                my->tx_enable = 0;
            }
            if (my->tx_samplerate == 0)
            {
                my->tx_samplerate = 16000;
                my->rx_samplerate = 16000;
                LOG_W("warning! no samplerate");
            }
            audio_3a_open(my->tx_samplerate, (uint8_t)(client->audio_type == AUDIO_TYPE_BT_VOICE), client->parameter.disable_uplink_agc);
            client->is_3a_opened = 1;
        }

        my->tx_data_tmp = audio_mem_malloc(my->tx_dma_size);
        RT_ASSERT(my->tx_data_tmp);
    }
    if (need_rx_init)
    {
        my->rx_channels    = client->parameter.read_channnel_num;
        my->rx_samplerate  = client->parameter.read_samplerate;
        RT_ASSERT(!my->rx_data_tmp);
        my->rx_data_tmp = audio_mem_malloc(CODEC_DATA_UNIT_LEN);
        RT_ASSERT(my->rx_data_tmp);

    }

    if (!device->rx_count && need_tx_init)
        bf0_audprc_eq_enable_offline(get_eq_config(client->audio_type));

#ifdef PKG_USING_AUDIO_TEST_API
    {
        extern uint8_t audio_test_api_eq_is_enable();
        uint8_t eq = audio_test_api_eq_is_enable();
        if (eq < 2)
        {
            bf0_audprc_eq_enable_offline(eq);
        }
    }
#endif

    LOG_I("(tx rx): ch[%d,%d] rate(%d %d)",
          my->tx_channels,
          my->rx_channels,
          my->tx_samplerate,
          my->rx_samplerate);

    //3. open hardware
#if ((!defined(AUDIO_RX_USING_I2S) && !defined(AUDIO_RX_USING_PDM)) || !defined(AUDIO_TX_USING_I2S))
    if (!my->audprc_dev)
    {
        my->audprc_dev = rt_device_find(AUDIO_SPEAKER_NAME);
        RT_ASSERT(my->audprc_dev);
        {
            err = rt_device_open(my->audprc_dev, RT_DEVICE_FLAG_RDWR);
            RT_ASSERT(RT_EOK == err);
        }

        my->audcodec_dev = rt_device_find(AUDIO_PRC_CODEC_NAME);
        RT_ASSERT(my->audcodec_dev);
        err = rt_device_open(my->audcodec_dev, RT_DEVICE_FLAG_WRONLY);
        RT_ASSERT(RT_EOK == err);
    }
#endif

#if DEBUG_FRAME_SYNC
    if (!HAL_DBG_DWT_IsInit())
        HAL_DBG_DWT_Init();
    HAL_DBG_DWT_Reset();
#endif

    //4. config TX
    if (need_tx_init)
    {
        config_tx(my, client);
    }

    //5. config RX, will reset audcodec PLL_CFG2, TX can't start before config RX
    if (need_rx_init)
    {
        config_rx(my);
    }

    if (need_tx_init && !need_rx_init) // tx only
    {
#if !defined(AUDIO_TX_USING_I2S)
        if (my->rx_ref > 0)
        {
            bf0_audprc_stop(); //avoid dac background noise
        }
        //6. DAC mute
        rt_device_control(my->audcodec_dev, AUDIO_CTL_MUTE, (void *)1);
        //7 DAC start
        stream = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDPRC_TX_CH0) << 8);
        LOG_I("speaker START stream=0x%x", stream);
        rt_device_control(my->audprc_dev, AUDIO_CTL_START, (void *)&stream);
        stream = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
        LOG_I("codec START stream=0x%x", stream);
        rt_device_control(my->audcodec_dev, AUDIO_CTL_START, &stream);
        rt_thread_mdelay(10);
#else
        stream  = AUDIO_STREAM_REPLAY;
        rt_device_control(my->i2s, AUDIO_CTL_START, &stream);
#endif
        my->opened_map_flag |= OPEN_MAP_TX;
        my->tx_ready = 1;
    }
    else if (!need_tx_init && need_rx_init) // rx only
    {
        //6 ADC start
        rt_base_t level = rt_hw_interrupt_disable();
        start_rx(my);
        rt_hw_interrupt_enable(level);
    }
    else if (need_tx_init && need_rx_init)
    {
        start_txrx(my);
    }
    //7. open PA, DAC unmute
    if (need_tx_init)
    {
        LOG_I("open PA, unmute DAC");
        audio_hardware_pa_start(my->tx_samplerate, 0);
#if !defined(AUDIO_TX_USING_I2S)
        rt_device_control(my->audcodec_dev, AUDIO_CTL_MUTE, (void *)0);
#endif
    }
    if (need_rx_init)
    {
        my->opened_map_flag |= OPEN_MAP_RX;
        my->rx_ready = 1;
    }
    if (client->audio_type == AUDIO_TYPE_BT_VOICE)
    {
        server->is_bt_3a = 1;
    }
Exit:
    LOG_I("%s out", __FUNCTION__);
    return 0;
}

static int audio_device_speaker_close(void *user_data)
{
    audio_client_t client;
    uint8_t need_tx_deinit = 0, need_rx_deinit = 0;
    audio_device_ctrl_t *device = (audio_device_ctrl_t *)user_data;
    audio_server_t *server = get_server();
    RT_ASSERT(device == &server->devices_ctrl[AUDIO_DEVICE_SPEAKER]);
    audio_device_speaker_t *my  = &server->device_speaker_private;
    client = device->closing_client;
    LOG_I("%s in t=%d r=%d c=0x%p f=%d", __FUNCTION__, my->tx_ref, my->rx_ref, client, client->rw_flag);

    if (my->opened_map_flag == 0)
    {
        LOG_I("close many times");
        return 0;
    }

    if (client->rw_flag & AUDIO_TX)
    {
        RT_ASSERT(my->tx_ref > 0);
        my->tx_ref--;
        if (!my->tx_ref)
        {
            need_tx_deinit = 1;
            my->tx_ready = 0;
        }
    }

    if (client->rw_flag & AUDIO_RX)
    {
        RT_ASSERT(my->rx_ref == 1);
        my->rx_ref--;

        if (!my->rx_ref)
        {
            need_rx_deinit = 1;
            my->rx_ready = 0;
        }
    }
    LOG_I("%s ref(t=%d r=%d) deinit(t=%d r=%d)", __FUNCTION__, my->tx_ref, my->rx_ref, need_tx_deinit, need_rx_deinit);

    //1. not allow process data
    //2. close PA
    int stream_audprc, stream_audcodec;
    if (need_tx_deinit && need_rx_deinit)
    {
        LOG_I("close pa txrx");
        audio_hardware_pa_stop();
        stream_audcodec = AUDIO_STREAM_RXandTX | ((1 << HAL_AUDCODEC_ADC_CH0) << 8) | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
        stream_audprc   = AUDIO_STREAM_RXandTX | ((1 << HAL_AUDPRC_RX_CH0) << 8) | ((1 << HAL_AUDPRC_TX_CH0) << 8);
        rt_base_t txrx = rt_hw_interrupt_disable();
        my->opened_map_flag  &= ~OPEN_MAP_TXRX;
        rt_hw_interrupt_enable(txrx);
#if !defined(AUDIO_TX_USING_I2S)
        rt_device_control(my->audcodec_dev, AUDIO_CTL_MUTE, (void *)1);
#endif
    }
    else if (need_tx_deinit)
    {
        LOG_I("close tx & pa");
        audio_hardware_pa_stop();
        //3. DAC/ADC
        rt_device_control(my->audcodec_dev, AUDIO_CTL_MUTE, (void *)1);
        stream_audcodec = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
        stream_audprc = AUDIO_STREAM_REPLAY | ((1 << HAL_AUDPRC_TX_CH0) << 8);
        rt_base_t tx = rt_hw_interrupt_disable();
        my->opened_map_flag  &= ~OPEN_MAP_TX;
        rt_hw_interrupt_enable(tx);
    }
    else if (need_rx_deinit)
    {
        LOG_I("close rx");
        stream_audcodec = AUDIO_STREAM_RECORD | ((1 << HAL_AUDCODEC_ADC_CH0) << 8);
        stream_audprc = AUDIO_STREAM_RECORD | ((1 << HAL_AUDPRC_RX_CH0) << 8);
        rt_base_t rx = rt_hw_interrupt_disable();
        my->opened_map_flag  &= ~OPEN_MAP_RX;
        rt_hw_interrupt_enable(rx);
    }
    else
    {
        LOG_I("%s close nothing", __FUNCTION__);
        goto Exit;
    }

    if (need_tx_deinit || need_rx_deinit)
    {

        if (my->i2s)
        {
            int stream = AUDIO_STREAM_REPLAY;
            rt_device_control(my->i2s, AUDIO_CTL_STOP, &stream);
            stream = AUDIO_STREAM_RECORD;
            rt_device_control(my->i2s, AUDIO_CTL_STOP, &stream);
        }

        if (my->audcodec_dev)
            rt_device_control(my->audcodec_dev, AUDIO_CTL_STOP, &stream_audcodec);
        if (my->audprc_dev)
            rt_device_control(my->audprc_dev, AUDIO_CTL_STOP, &stream_audprc);

        if (my->pdm)
        {
            rt_device_close(my->pdm);
            my->pdm = NULL;
#if MICBIAS_USING_AS_PDM_POWER
            micbias_power_off();
#endif
        }
    }

    if (!my->tx_ref && !my->rx_ref)
    {
        LOG_I("close device & pll");
        if (my->audprc_dev)
        {
            //stream_audcodec = AUDIO_STREAM_RXandTX | ((1 << HAL_AUDCODEC_ADC_CH0) << 8) | ((1 << HAL_AUDCODEC_DAC_CH0) << 8);
            //stream_audprc = AUDIO_STREAM_RXandTX | ((1 << HAL_AUDPRC_RX_CH0) << 8) | ((1 << HAL_AUDPRC_TX_CH0) << 8);
            //rt_device_control(my->audcodec_dev, AUDIO_CTL_STOP, &stream_audcodec);
            //rt_device_control(my->audprc_dev, AUDIO_CTL_STOP, &stream_audprc);

            bf0_disable_pll();
            rt_device_close(my->audcodec_dev);
            rt_device_close(my->audprc_dev);
        }
        my->audcodec_dev = NULL;
        my->audprc_dev = NULL;
#if START_RX_IN_TX_INTERUPT
        rt_event_delete(my->event);
        my->event = NULL;
#endif
#if DEBUG_FRAME_SYNC
        LOG_I("tx=%d rx=%d", my->debug_tx_index, my->debug_rx_index);
        for (int i = 0; i < FRAME_DEBUG_MAX; i++)
        {
            LOG_I("[txrx %d]=[%d %d]", i, my->debug_tx_tick[i], my->debug_rx_tick[i] - my->debug_tx_tick[i]);
        }
#endif

    }
    //4. free memory
    if (my->rx_data_tmp && !my->rx_ref)
    {
        RT_ASSERT((my->opened_map_flag  & OPEN_MAP_RX) == 0);
        audio_mem_free(my->rx_data_tmp);
        my->rx_data_tmp = NULL;
    }

    if (my->tx_data_tmp && !my->tx_ref)
    {
        RT_ASSERT((my->opened_map_flag  & OPEN_MAP_TX) == 0);
        audio_mem_free(my->tx_data_tmp);
        my->tx_data_tmp = NULL;
    }
Exit:
    LOG_I("%s out", __FUNCTION__);
    return 0;
}

/* -----------------speaker device end----------------- */

static int is_in_list(rt_list_t *list, rt_list_t *l_node)
{
    rt_list_t *node = list->next;
    for (; node != list; node = node->next)
    {
        if (node == (rt_list_t *)l_node)
            break;
    }
    return (node != list);
}

static inline void send_cmd_event_to_server()
{
    rt_event_send(&get_server()->event, AUDIO_SERVER_EVENT_CMD);
}

static int is_node_in_list(rt_list_t *list, rt_list_t *l_node)
{
    rt_list_t *node = list->next;
    for (; node != list; node = node->next)
    {
        if (node == (rt_list_t *)l_node)
            break;
    }
    return (node != list);
}

int is_a2dp_working(void)
{
    return g_server.is_bt_music_working;
}

static int a2dp_device_input_callback(audio_server_callback_cmt_t cmd, const uint8_t *buffer, uint32_t size)
{
    //LOG_I("a2dp sink cmd=%d", cmd);
    if (cmd == as_callback_cmd_cache_empty || cmd == as_callback_cmd_cache_half_empty)
    {
        rt_event_send(&get_server()->event, AUDIO_SERVER_EVENT_TX_A2DP_SINK);
    }
    else  if (cmd == as_callback_cmd_play_to_next)
    {
        rt_event_send(&get_server()->event, AUDIO_SERVER_EVENT_A2DP_NEXT);
    }
    else if (cmd == as_callback_cmd_play_to_prev)
    {
        rt_event_send(&get_server()->event, AUDIO_SERVER_EVENT_A2DP_PREV);
    }
    else if (cmd == as_callback_cmd_play_pause)
    {
        rt_event_send(&get_server()->event, AUDIO_SERVER_EVENT_A2DP_PAUSE);
    }
    else if (cmd == as_callback_cmd_play_resume)
    {
        rt_event_send(&get_server()->event, AUDIO_SERVER_EVENT_A2DP_RESUME);
    }
    return 0;
}

static int ble_bap_sink_device_input_callback(audio_server_callback_cmt_t cmd, const uint8_t *buffer, uint32_t size)
{
    //LOG_I("ble sink cmd=%d", cmd);
    if (cmd == as_callback_cmd_cache_empty || cmd == as_callback_cmd_cache_half_empty)
    {
        rt_event_send(&get_server()->event, AUDIO_SERVER_EVENT_TX_BLE_SINK);
    }
    return 0;
}


static int hfp_device_input_callback(audio_server_callback_cmt_t cmd, const uint8_t *buffer, uint32_t size)
{
    if (cmd == as_callback_cmd_cache_empty || cmd == as_callback_cmd_cache_half_empty)
    {
        rt_event_send(&get_server()->event, AUDIO_SERVER_EVENT_TX_HFP);
    }
    return 0;
}

static int i2s1_device_input_callback(audio_server_callback_cmt_t cmd, const uint8_t *buffer, uint32_t size)
{
    if (cmd == as_callback_cmd_cache_empty || cmd == as_callback_cmd_cache_half_empty)
    {
        rt_event_send(&get_server()->event, AUDIO_SERVER_EVENT_TX_I2S1);
    }
    return 0;
}
static int i2s2_device_input_callback(audio_server_callback_cmt_t cmd, const uint8_t *buffer, uint32_t size)
{
    if (cmd == as_callback_cmd_cache_empty || cmd == as_callback_cmd_cache_half_empty)
    {
        rt_event_send(&get_server()->event, AUDIO_SERVER_EVENT_TX_I2S2);
    }
    return 0;
}

static int hardware_device_open(audio_device_ctrl_t *device, audio_client_t client)
{
    int ret;

    LOG_I("%s in: r=%d t=%d b=%d d=%d c=0x%p f=%d", __FUNCTION__,
          device->rx_count, device->tx_count, device->is_busy, device->device_type, client, client->rw_flag);

    RT_ASSERT(device->device.open);

    client->device_using = device->device_type;
    device->opening_client = client;
#if SOFTWARE_TX_MIX_ENABLE
    if (client->rw_flag & AUDIO_TX)
    {
        if (!device->tx_mixed_pool)
        {
            uint16_t size = TX_DMA_SIZE * 4;
#if TWS_MIX_ENABLE
            if (device->device_type == AUDIO_DEVICE_A2DP_SINK)
            {
                size = 30000;
            }
#endif
            device->tx_mixed_pool = resample_malloc(size);
            RT_ASSERT(device->tx_mixed_pool);
            rt_ringbuffer_init(&device->tx_mixed_rb, device->tx_mixed_pool, size);
        }
    }
#endif
    switch (device->device_type)
    {
    case AUDIO_DEVICE_SPEAKER:
        ret = device->device.open(device->device.user_data, NULL);
        audio_device_ctrl_t *sink = &g_server.devices_ctrl[AUDIO_DEVICE_BLE_BAP_SINK];
        if (sink->is_registerd && !device->tx_count && ble_bap_src_enabled)
        {
            LOG_I("speaker to ble");
            g_ble_bap_sink_parameter.tx_sample_rate = client->parameter.write_samplerate;
            g_ble_bap_sink_parameter.tx_channels = client->parameter.write_channnel_num;
            sink->device.open(&g_ble_bap_sink_parameter, NULL);
        }
        break;
    case AUDIO_DEVICE_A2DP_SINK:
        if (device->tx_count)
        {
            LOG_I("a2dp share open");
            ret = 0;
            break;
        }
        current_play_status = 1;
        a2dp_sink_need_trigger = 1;
        device->tx_mix_dst_channel = 2;
        device->tx_mix_dst_samplerate = 44100;
        ret = device->device.open(device->device.user_data, a2dp_device_input_callback);
        break;
    case AUDIO_DEVICE_HFP:
        hfp_dev_input_buf = audio_mem_malloc(640);
        RT_ASSERT(hfp_dev_input_buf);
        hfp_dev_input_buf_offset = 0;
        ret = device->device.open(device->device.user_data, hfp_device_input_callback);
        break;
    case AUDIO_DEVICE_PDM1:
    case AUDIO_DEVICE_PDM2:
        ret = device->device.open(device->device.user_data, NULL);
        break;
    case AUDIO_DEVICE_I2S1:
        device->device.open(device->device.user_data, i2s1_device_input_callback);
        break;
    case AUDIO_DEVICE_I2S2:
        device->device.open(device->device.user_data, i2s2_device_input_callback);
        break;
    case AUDIO_DEVICE_BLE_BAP_SINK:
        if (device->tx_count)
        {
            LOG_I("ble share open");
            break;
        }
        g_ble_bap_sink_parameter.device_rb = &client->ring_buf;
        g_ble_bap_sink_parameter.tx_sample_rate = client->parameter.write_samplerate;
        g_ble_bap_sink_parameter.tx_channels = client->parameter.write_channnel_num;
        ret = device->device.open(&g_ble_bap_sink_parameter, ble_bap_sink_device_input_callback);
        break;
    default:
        RT_ASSERT(0);
        break;
    }
    RT_ASSERT(0 == ret);
    if (client->rw_flag & AUDIO_TX)
    {
        device->tx_count++;
    }
    if (client->rw_flag & AUDIO_RX)
    {
        device->rx_count++;
    }
    device->is_busy = 1;
    LOG_I("%s out: rx=%d tx=%d b=%d", __FUNCTION__, device->rx_count, device->tx_count, device->is_busy);
    return ret;
}

static void hardware_device_close(audio_client_t client)
{
    audio_server_t *server = get_server();
    audio_device_ctrl_t *device;
    RT_ASSERT(client);
    RT_ASSERT(client->device_using >= AUDIO_DEVICE_SPEAKER && client->device_using < AUDIO_DEVICE_NUMBER);

    device = &server->devices_ctrl[client->device_using];
    device->closing_client = client;

    LOG_I("%s in: d=%d r=%d t=%d b=%d f=%d", __FUNCTION__, device->device_type, device->rx_count, device->tx_count, device->is_busy, client->rw_flag);

    RT_ASSERT(device->rx_count || device->tx_count);

    if (client->rw_flag & AUDIO_TX)
    {
        device->tx_count--;
    }
    if (client->rw_flag & AUDIO_RX)
    {
        device->rx_count--;
    }
    if (!device->rx_count && !device->tx_count)
    {
        device->is_busy = 0;
    }

    if (device->device_type == AUDIO_DEVICE_SPEAKER)
    {
        RT_ASSERT(device->device.close);
        device->device.close(device->device.user_data);
    }
    else
    {
        if (!device->rx_count && !device->tx_count && device->device.close)
        {
            device->device.close(device->device.user_data);
        }
    }
#if SOFTWARE_TX_MIX_ENABLE
    if (!device->tx_count && device->tx_mixed_pool)
    {
        resample_free(device->tx_mixed_pool);
        device->tx_mixed_pool = NULL;
    }
#endif
    client->device_using = AUDIO_DEVICE_NONE;

    if (device->device_type == AUDIO_DEVICE_HFP && hfp_dev_input_buf)
    {
        audio_mem_free(hfp_dev_input_buf);
        hfp_dev_input_buf = NULL;
        hfp_dev_input_buf_offset = 0;
    }

    LOG_I("%s out: r=%d t=%d b=%d", __FUNCTION__, device->rx_count, device->tx_count, device->is_busy);
}

static void device_print_current_client(audio_device_ctrl_t *device)
{
    rt_list_t *pos = NULL;
    audio_client_t c = NULL;

    LOG_I("device %d info reg=%d busy=%d:", device->device_type, device->is_registerd, device->is_busy);
    rt_list_for_each(pos, &device->running_client_list)
    {
        c = rt_list_entry(pos, struct audio_client_base_t, node);
        LOG_I("running: d=%d h=%x name=%s type=%d rw=%d device_s=%d device_u=%d prio=%d",
              device->device_type, c, c->name, c->audio_type, c->rw_flag,
              c->device_specified, c->device_using, mix_policy[c->audio_type].priority);
    }
    rt_list_for_each(pos, &get_server()->suspend_client_list)
    {
        c = rt_list_entry(pos, struct audio_client_base_t, node);
        LOG_I("suspend: h=%x name=%s type=%d  rw=%d device_s=%d device_u=%d prio=%d", c, c->name, c->audio_type,
              c->device_specified, c->device_using, mix_policy[c->audio_type].priority);
    }
    LOG_I("device %d info end", device->device_type);
}

static void device_suspend_one_client(audio_device_ctrl_t *device, audio_client_t c)
{
    LOG_I("suspend: d=%d h=%x name=%s type=%d prio=%d", device->device_type, c, c->name, c->audio_type, mix_policy[c->audio_type].priority);
    rt_list_insert_before(&get_server()->suspend_client_list, &c->node);
    c->is_suspended = 1;
    if (c->is_3a_opened)
    {
        audio_3a_close();
        c->is_3a_opened = 0;
    }
    if (c->callback)
    {
        c->callback(as_callback_cmd_suspended, c->user_data, 0);
        LOG_I("callback suspend old ok");
    }
}

static void device_suspend_one_running_client(audio_device_ctrl_t *device, audio_client_t c)
{
    hardware_device_close(c);

    rt_base_t hw = rt_hw_interrupt_disable();
    rt_list_remove(&c->node);
    rt_hw_interrupt_enable(hw);
    device_suspend_one_client(device, c);
#if 0
    /*
        if mp3 is playing, wait mp3 thread suspend.
        noise occur when mp3 decode thread is interrupt by new client thread
    */
    rt_thread_mdelay(100);
#endif
}

static void device_suspend_all_running_client(audio_device_ctrl_t *device)
{
    rt_list_t *pos, *pos_next;
    audio_client_t c = NULL;

    rt_list_for_each_safe(pos, pos_next, &device->running_client_list)
    {
        c = rt_list_entry(pos, struct audio_client_base_t, node);
        device_suspend_one_running_client(device, c);
    }
}

static audio_client_t device_get_rx_in_running(audio_device_ctrl_t *device)
{
    bool ret = false;
    rt_list_t *pos = NULL;
    audio_client_t c = NULL;

    rt_list_for_each(pos, &device->running_client_list)
    {
        c = rt_list_entry(pos, struct audio_client_base_t, node);
        if (c->rw_flag & AUDIO_RX)
        {
            return c;
        }
    }
    return NULL;
}

static audio_client_t device_get_tx_in_running(audio_device_ctrl_t *device, int index)
{
    bool ret = false;
    rt_list_t *pos = NULL;
    audio_client_t c = NULL;
    int n = 0;
    rt_list_for_each(pos, &device->running_client_list)
    {
        c = rt_list_entry(pos, struct audio_client_base_t, node);
        if (c->rw_flag & AUDIO_TX)
        {
            if (n == index)
                return c;
            n++;
        }
    }
    return NULL;
}

static int device_get_tx_num_in_running(audio_device_ctrl_t *device)
{
    int n = 0;
    rt_list_t *pos = NULL;
    audio_client_t c = NULL;

    rt_list_for_each(pos, &device->running_client_list)
    {
        c = rt_list_entry(pos, struct audio_client_base_t, node);
        if (c->rw_flag & AUDIO_TX)
        {
            n++;
        }
    }
    return n;
}

static inline int client_compare_priority(audio_client_t c1, audio_client_t c2, audio_device_e type)
{
    UNUSED(type);
    uint8_t v1 = mix_policy[c1->audio_type].priority;
    uint8_t v2 = mix_policy[c2->audio_type].priority;
    if (v1 == v2)
    {
        return 0;
    }
    else if (v1 > v2)
    {
        return 1;
    }
    return -1;
}

static void audio_device_open(audio_server_t *server, audio_client_t client)
{
    uint8_t need_start = 0;
    int diff;
    int ret = 0;
    audio_device_ctrl_t *device;
    audio_client_t low = NULL;
    audio_client_t hight = NULL;

    audio_device_e want_device = server->private_device[client->audio_type];
    if (want_device == AUDIO_DEVICE_NO_INIT)
    {
        want_device = server->public_device;
    }
    if (client->device_specified != AUDIO_DEVICE_AUTO)
    {
        want_device = client->device_specified;
    }
    device = &server->devices_ctrl[want_device];

    if (!device->is_registerd)
    {
        if (client->device_specified != AUDIO_DEVICE_AUTO)
        {
            LOG_W("device %d not find, using speaker", want_device);
            device = &server->devices_ctrl[AUDIO_DEVICE_SPEAKER];
        }
        else
        {
            LOG_W("device %d not find, suspend c=0x%p, d=%d", client, want_device);
            goto suspend_new_exit;
        }
    }

    LOG_I("%s in d=%d busy=%d reg=%d c=%p", __FUNCTION__, device->device_type, device->is_busy, device->is_registerd, client);

    device_print_current_client(device);

#if !MULTI_CLIENTS_AT_WORKING
    if (server->only_one_client)
    {
        int diff = client_compare_priority(client, server->only_one_client, server->only_one_client->device_using);
        if (diff >= 0)
        {
            LOG_I("server suspend old stream");
            audio_device_ctrl_t *device_old = &server->devices_ctrl[server->only_one_client->device_using];
            device_suspend_one_running_client(device_old, server->only_one_client);
            server->only_one_client = NULL;
        }
        else
        {
            LOG_I("server suspend new stream");
            goto suspend_new_exit;
        }
    }
    else
    {
        LOG_I("server first stream");
    }
#endif

    //step 1: check rx client, only one rx client can use this device
    if (rt_list_isempty(&device->running_client_list))
    {
        LOG_I("device %d first stream", want_device);
        device->rx_samplerate = client->parameter.read_samplerate;
        device->tx_mix_dst_channel = client->parameter.write_channnel_num;
        device->tx_mix_dst_samplerate = client->parameter.write_samplerate;

        RT_ASSERT(!device->is_busy);
        RT_ASSERT(!device->tx_count);
        RT_ASSERT(!device->rx_count);
    }
    else if (client->audio_type == AUDIO_TYPE_BT_VOICE || client->audio_type == AUDIO_TYPE_MODEM_VOICE)
    {
        device_suspend_all_running_client(device);
        RT_ASSERT(!device->is_busy);
        RT_ASSERT(!device->tx_count);
        RT_ASSERT(!device->rx_count);
    }
    else if (client->rw_flag & AUDIO_RX)
    {
        hight = device_get_rx_in_running(device);
        if (hight && !is_can_mix(hight, client, device))
        {
            diff = client_compare_priority(client, hight, device->device_type);
            if (diff >= 0)
            {
                device_suspend_one_running_client(device, hight);
            }
            else
            {
                goto suspend_new_exit;
            }
        }
    }

    //step 2: check tx client to mix, only support two tx mix
    if (client->rw_flag & AUDIO_TX)
    {
        int tx_num = device_get_tx_num_in_running(device);
        RT_ASSERT(tx_num <= 2);
        if (tx_num == 1)
        {
            hight = device_get_tx_in_running(device, 0);
            if (!hight)
            {
                RT_ASSERT(0);
                goto Exit;
            }
            goto tx_mix_check;
        }
        else if (tx_num == 2)
        {
            audio_client_t tmp;
            low = device_get_tx_in_running(device, 0);
            hight = device_get_tx_in_running(device, 1);
            if (!low || !hight)
            {
                RT_ASSERT(0);
                goto Exit;
            }

            diff = client_compare_priority(hight, low, device->device_type);
            if (diff < 0)
            {
                tmp = hight;
                hight = low;
                low = tmp;
            }
            diff = client_compare_priority(client, low, device->device_type);
            if (diff >= 0)
            {
                device_suspend_one_running_client(device, low);
tx_mix_check:
                if (!is_can_mix(hight, client, device))
                {
                    diff = client_compare_priority(client, hight, device->device_type);
                    if (diff >= 0)
                    {
                        device_suspend_one_running_client(device, hight);
                    }
                    else
                    {
                        goto suspend_new_exit;
                    }
                }
            }
            else
            {
                goto suspend_new_exit;
            }
        }
    }

    //step 3 start new client
    LOG_I("device count tx=%d rx=%d busy=%d", device->tx_count, device->rx_count, device->is_busy);
#if !MULTI_CLIENTS_AT_WORKING
    server->only_one_client = client;
    RT_ASSERT(!device->is_busy);
#endif
    if (!device->is_busy)
    {
        RT_ASSERT(device_get_tx_num_in_running(device) == 0);
        RT_ASSERT(device_get_rx_in_running(device) == NULL);
        rt_list_insert_before(&device->running_client_list, &client->node);

        device->tx_mix_dst_channel = client->parameter.write_channnel_num;
        device->tx_mix_dst_samplerate = client->parameter.write_samplerate;
        hardware_device_open(device, client);
    }
    else
    {
        rt_list_insert_before(&device->running_client_list, &client->node);
        hardware_device_open(device, client);
    }

    if (client->callback)
    {
        client->callback(as_callback_cmd_opened, client->user_data, 0);
        LOG_I("callback opened ok");
    }
    goto Exit;
suspend_new_exit:
    device_suspend_one_client(device, client);
Exit:
    LOG_I("%s out", __FUNCTION__);
    device_print_current_client(device);
}

static audio_client_t get_highest_priority_handle_in_suspend(audio_server_t *server)
{
    if (rt_list_isempty(&server->suspend_client_list))
    {
        return NULL;
    }
    rt_list_t *pos = NULL;
    audio_client_t client = NULL;
    audio_client_t highest = NULL;
    rt_list_for_each(pos, &server->suspend_client_list)
    {
        client = rt_list_entry(pos, struct audio_client_base_t, node);
        LOG_I("suspend list h=%x n=%s t=%d pri=%d", client, client->name, client->audio_type, mix_policy[client->audio_type].priority);
        if (highest == NULL || mix_policy[client->audio_type].priority > mix_policy[highest->audio_type].priority)
        {
            highest = client;
        }
    }
    return highest;
}

static void audio_device_close(audio_server_t *server, audio_client_t client)
{
    uint8_t need_start = 0;
    int diff;
    int ret = 0;
    audio_device_ctrl_t *device;
    audio_client_t low = NULL;
    audio_client_t hight = NULL;
    device = &server->devices_ctrl[client->device_using];

    LOG_I("%s in d=%d busy=%d reg=%d c=%p", __FUNCTION__, device->device_type, device->is_busy, device->is_registerd, client);

    device_print_current_client(device);
    RT_ASSERT(is_in_list(&device->running_client_list, &client->node));

    //step 3 start new client
    LOG_I("device count tx=%d rx=%d busy=%d", device->tx_count, device->rx_count, device->is_busy);
    RT_ASSERT(device->is_busy);

    hardware_device_close(client);
    rt_base_t hw = rt_hw_interrupt_disable();
    rt_list_remove(&client->node);
    rt_hw_interrupt_enable(hw);

#if !MULTI_CLIENTS_AT_WORKING
    server->only_one_client = NULL;
#endif
    LOG_I("running some suspend");

    audio_client_t suspend1 = get_highest_priority_handle_in_suspend(server);
    if (suspend1)
    {
        LOG_I("resume h=0x%x, n=%s t=%d", suspend1, suspend1->name, suspend1->audio_type);
        rt_list_remove(&suspend1->node);
        if (suspend1->parameter.is_need_3a)
        {
            audio_3a_open(suspend1->parameter.read_bits_per_sample, (uint8_t)(suspend1->audio_type == AUDIO_TYPE_BT_VOICE), suspend1->parameter.disable_uplink_agc);
            suspend1->is_3a_opened = 1;
        }
        audio_device_open(server, suspend1);
        suspend1->is_suspended = 0;
        if (suspend1->callback && suspend1->magic == AUDIO_CLIENT_MAGIC)
        {
            suspend1->callback(as_callback_cmd_resumed, suspend1->user_data, 0);
            if (mix_policy[suspend1->audio_type].priority <= mix_policy[AUDIO_TYPE_LOCAL_MUSIC].priority)
            {
                if (server->local_music_listener)
                {
                    LOG_I("notify listern(1, 0)");
                    server->local_music_listener(1, 0);
                }
            }
            LOG_I("callback resume old ok");
        }
    }
    else
    {
        LOG_I("suspend empty");
        if (server->local_music_listener)
        {
            LOG_I("notify listern(1, 0)");
            server->local_music_listener(1, 0);
        }
    }

    LOG_I("%s out", __FUNCTION__);
    device_print_current_client(device);
}

static void audio_device_clean_cache(audio_client_t client)
{
    if (client)
    {
        rt_base_t level;
        uint32_t cache_drop_bytes;
        uint8_t data[576];
        level = rt_hw_interrupt_disable();
        while (rt_ringbuffer_get(&client->ring_buf, data, sizeof(data)))
        {
            ;
        }
        rt_hw_interrupt_enable(level);
    }
}

static inline bool change_device(audio_client_t running, audio_device_ctrl_t *dev_old, audio_device_ctrl_t *dev_new)
{

    LOG_I("device %d change from %d(using %d) to %d", dev_old->device_type, dev_old->device_type, running->device_using, dev_new->device_type);
    LOG_I("device count tx=%d rx=%d busy=%d", dev_old->tx_count, dev_old->rx_count, dev_old->is_busy);

    if ((running->device_using == dev_new->device_type)
            || (running->device_specified != AUDIO_DEVICE_AUTO)
            || ((running->rw_flag & AUDIO_TXRX) == AUDIO_RX))
    {
        LOG_I("no need change");
        return false;
    }

    if (((running->rw_flag & AUDIO_TXRX) == AUDIO_RX)
            || !dev_new->is_registerd)
    {
        LOG_I("change denied: audio_type=%d rw=%d reg=%d", running->audio_type, running->rw_flag, dev_new->is_registerd);
        return false;
    }

    //step 1: check rx client, only one rx client can use this device
    if (rt_list_isempty(&dev_new->running_client_list))
    {
        LOG_I("device %d first stream", dev_new->device_type);
    }
    else if (running->audio_type == AUDIO_TYPE_BT_VOICE || running->audio_type == AUDIO_TYPE_MODEM_VOICE)
    {
        LOG_I("call change");
        device_suspend_all_running_client(dev_new);
        RT_ASSERT(!dev_new->is_busy);
        RT_ASSERT(!dev_new->tx_count);
        RT_ASSERT(!dev_new->rx_count);
    }
    else if (running->rw_flag & AUDIO_RX)
    {
        LOG_I("dennied: rw=%d contain rx", running->rw_flag);
        return false;
    }

    //step 2: check tx client to mix, only support two tx mix
    if (running->rw_flag & AUDIO_TX)
    {
        int tx_num = device_get_tx_num_in_running(dev_new);
        RT_ASSERT(tx_num <= 2);
        if (tx_num == 2)
        {
            LOG_I("dennied: two tx");
            device_suspend_one_client(dev_new, running);
            return false;
        }
        else if (tx_num == 1)
        {
            audio_client_t old = device_get_tx_in_running(dev_new, 0);
            if (!is_can_mix(running, old, dev_new))
            {
                LOG_I("dennied: can't mix");
                device_suspend_one_running_client(dev_new, running);
                return false;
            }
        }
    }

    LOG_I("change new dev count tx=%d rx=%d busy=%d", dev_new->tx_count, dev_new->rx_count, dev_new->is_busy);

    current_audio_device = dev_new->device_type;
    if (dev_new->device_type == AUDIO_DEVICE_A2DP_SINK)
    {
        a2dp_sink_need_trigger = 1;
    }
    hardware_device_close(running);
    rt_base_t hw = rt_hw_interrupt_disable();
    rt_list_remove(&running->node);
    rt_list_insert_before(&dev_new->running_client_list, &running->node);
    rt_hw_interrupt_enable(hw);

    if (!dev_new->is_busy)
    {
        dev_new->rx_samplerate = running->parameter.read_samplerate;
        dev_new->tx_mix_dst_channel = running->parameter.write_channnel_num;
        dev_new->tx_mix_dst_samplerate = running->parameter.write_samplerate;
    }
    hardware_device_open(dev_new, running);
    audio_device_clean_cache(running); //avoid noise, may different samplerate data in cache

    device_print_current_client(dev_new);
    LOG_I("%s out", __FUNCTION__);
    return true;
}

static void audio_device_change(audio_server_t *server)
{
    audio_device_ctrl_t *device;
    audio_device_e type_new;
    audio_client_t running;
    rt_list_t  *pos, *n;
    LOG_I("%s in", __FUNCTION__);
    for (int i = 0; i < AUDIO_DEVICE_NUMBER; i++)
    {
        device = &server->devices_ctrl[i];
        device_print_current_client(device);

        rt_list_for_each_safe(pos, n, &device->running_client_list)
        {
            running = rt_list_entry(pos, struct audio_client_base_t, node);
            if (!running)
                continue;
            type_new = server->private_device[running->audio_type];
            if (type_new == AUDIO_DEVICE_NO_INIT)
            {
                LOG_I("dev change public");
                type_new = server->public_device;
            }
            else
            {
                LOG_I("dev change private");
            }
            change_device(running, device, &server->devices_ctrl[type_new]);
        }
    }
}


bool audio_device_is_a2dp_sink()
{
#if (SOFTWARE_TX_MIX_ENABLE && TWS_MIX_ENABLE)
    return false; //resampe when audio write, upper layer should not resample
#endif

    if (g_server.devices_ctrl[AUDIO_DEVICE_A2DP_SINK].is_busy)
    {
        return true;
    }
    return false;
}

int is_device_a2dp_sink()
{
    return (int)audio_device_is_a2dp_sink();
}
inline static void audio_client_start(audio_client_t client)
{
    audio_server_t *server = get_server();
    LOG_I("%s h=%x n=%s t=%d f=%d d=%d", __FUNCTION__, client, client->name, client->audio_type, client->rw_flag, client->device_specified);
#ifdef RT_USING_PM
    audio_pm_debug++;
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_start();
#ifdef SF32LB52X
    pm_scenario_start(PM_SCENARIO_AUDIO);
    if (client->audio_type == AUDIO_TYPE_BT_VOICE)
    {
        HAL_HPAON_WakeCore(CORE_ID_LCPU);
        LOG_I("audio wakup lcpu"); //avoid access lcpu memory when lcpu is sleeping
    }
#endif
#endif

    if (client->audio_type == AUDIO_TYPE_BT_VOICE)
    {
        rt_uint32_t evt;
        rt_event_send(&server->down_event, AUDIO_SERVER_EVENT_DOWN_START);
        rt_event_recv(&server->event, AUDIO_SERVER_EVENT_DOWN_START, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt) ;
    }

    RT_ASSERT(client->audio_type < AUDIO_TYPE_NUMBER);

    if (client->audio_type != AUDIO_TYPE_LOCAL_MUSIC)
    {
#if !SOFTWARE_TX_MIX_ENABLE
        if (client->rw_flag & AUDIO_TX)
        {
            LOG_I("notify listern(0, 0)");
            if (server->local_music_listener)
                server->local_music_listener(0, 0);
        }
#endif
    }

    audio_device_open(server, client);

    LOG_I("%s out", __FUNCTION__);
    rt_event_send(client->api_event, (1 << 1));
}

inline static void audio_client_stop(audio_client_t client)
{
    audio_type_t audio_type;
    audio_server_t *server = get_server();
    RT_ASSERT(client);
    audio_type = client->audio_type;
    LOG_I("audio_client_stop 0x%x %s pm_debug=%d", client, client->name, audio_pm_debug);
    client->is_suspended = 1;
    if (client->callback)
    {
        client->callback(as_callback_cmd_closed, client->user_data, 0);
        LOG_I("callback closed ok");
    }

    if (client->audio_type == AUDIO_TYPE_BT_MUSIC)
    {
        server->is_bt_music_working = 0;
    }
    else if (client->audio_type == AUDIO_TYPE_BT_VOICE)
    {
        rt_uint32_t evt;
        rt_event_send(&server->down_event, AUDIO_SERVER_EVENT_DOWN_END);
        rt_event_recv(&server->event, AUDIO_SERVER_EVENT_DOWN_END, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt) ;
        server->is_bt_3a = 0;
        audio_3a_close();
        client->is_3a_opened = 0;
    }
    else if (client->is_3a_opened)
    {
        audio_3a_close();
        client->is_3a_opened = 0;
    }

    if (is_in_list(&server->suspend_client_list, &client->node))
    {
        LOG_I("stop in suspendlist");
        rt_list_remove(&client->node);
        rt_ringbuffer_reset(&client->ring_buf);
        audio_mem_free(client->ring_pool);
        client->magic = 0;
        rt_event_send(client->api_event, 1);
        audio_mem_free(client);
        goto Exit;
    }

    audio_device_close(server, client);

    audio_mem_free(client->ring_pool);
    client->magic = 0;
    rt_event_send(client->api_event, 1);
#if SOFTWARE_TX_MIX_ENABLE
    if (client->resample)
    {
        sifli_resample_close(client->resample);
        client->resample = NULL;
    }

#endif
    audio_mem_free(client);

Exit:

#ifdef RT_USING_PM
    audio_pm_debug--;
#ifdef SF32LB52X
    if (audio_type == AUDIO_TYPE_BT_VOICE)
    {
        HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
        LOG_I("audio release lcpu, 3a=%d", server->is_bt_3a);
    }

    if (audio_pm_debug == 0)
    {
        pm_scenario_stop(PM_SCENARIO_AUDIO);
        current_play_status = 0;
    }
#endif
    rt_pm_hw_device_stop();
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif

    RT_ASSERT(audio_pm_debug >= 0);
    LOG_I("audio_client_stop out pm=%d", audio_pm_debug);
}

inline static int audio_process_cmd(audio_server_t *server)
{
    int ret = 0;
    uint32_t val;
    audio_type_t   audio_type;
    audio_device_e device_type;

    lock();
    do
    {
        audio_server_cmd_e     cmd_e;
        audio_server_cmt_t     *p_cmd;
        audio_client_t          client;
        rt_slist_t *first = rt_slist_first(&server->command_slist);
        if (!first)
            break;

        p_cmd = rt_slist_entry(first, audio_server_cmt_t, snode);
        rt_slist_remove(&server->command_slist, first);
        cmd_e = p_cmd->cmd;
        client = p_cmd->client;
        audio_mem_free(p_cmd);
        p_cmd = NULL;
        LOG_I("audio cmd=%d", cmd_e);
        if (cmd_e == AUDIO_CMD_OPEN)
        {
            RT_ASSERT(client && client->magic == AUDIO_CLIENT_MAGIC);
            audio_client_start(client);
            if (a2dp_sink_need_trigger == 0)
            {
                a2dp_sink_need_trigger = 1;
            }
        }
        else if (cmd_e == AUDIO_CMD_CLOSE)
        {
            RT_ASSERT(client && client->magic == ~AUDIO_CLIENT_MAGIC);
            audio_client_stop(client);
        }
        else if (cmd_e == AUDIO_CMD_DEVICE_PUBLIC)
        {
            val = (uint32_t)client;
            device_type = (audio_device_e)val;
            server->public_device = device_type;
            current_audio_device = device_type;
            if (device_type == AUDIO_DEVICE_A2DP_SINK)
            {
                a2dp_sink_need_trigger = 1;
            }
            audio_device_change(server);
        }
        else if (cmd_e == AUDIO_CMD_DEVICE_PRIVATE)
        {
            val = (uint32_t)client;
            audio_type = (audio_type_t)((val >> 8) & 0xFFFF);
            device_type = (audio_device_e)(val & 0xFFFF);
            server->private_device[audio_type] = device_type;
            current_audio_device = device_type;
            if ((device_type == AUDIO_DEVICE_A2DP_SINK) && (a2dp_sink_need_trigger == 0))
            {
                a2dp_sink_need_trigger = 1;
            }
            audio_device_change(server);
        }
    }
    while (0);

    if (!rt_slist_isempty(&server->command_slist))
    {
        ret = 1;
    }
    unlock();
    return ret;
}

extern void notify_dma_done_to_a2dp();
RT_WEAK void notify_dma_done_to_a2dp()
{
}
static rt_err_t speaker_tx_done(rt_device_t dev, void *buffer)
{
    //in inturrupt
    audio_server_t *server = get_server();
    //rt_kprintf("-tx done\n");
    process_speaker_tx(server, &server->device_speaker_private);
#if BT_BAP_BROADCAST_SOURCE
    if (ble_tx_dma_callback)
    {
        ble_tx_dma_callback();
    }
    notify_dma_done_to_a2dp();
#endif
    return RT_EOK;
}

static rt_err_t mic_rx_ind(rt_device_t dev, rt_size_t size)
{
    //in inturrupt
    rt_event_send(&g_server.event, AUDIO_SERVER_EVENT_RX);
    return RT_EOK;
}

void auido_gain_pcm(int16_t *p, rt_size_t len, uint8_t shift)
{
    int pcm;
    RT_ASSERT(p);
    for (int i = 0; i < len / 2; i++)
    {
        pcm = (int)p[i];
        pcm = pcm << shift;
        if (pcm > 65535)
        {
            pcm = 65535;
        }
        else if (pcm < -65536)
        {
            pcm = -65536;
        }
        p[i] = pcm;
    }
}

/**
  * @brief  bt voice data coming indication
  * @param  fifo: data pointer
  *         len: data length
  * @retval whether or not need downlink processing algorithm
  */
uint8_t audio_server_bt_voice_ind(uint8_t *fifo, uint8_t len)
{
    uint8_t ret = 1;
    rt_size_t putsize;
    audio_server_t *server = get_server();
    audio_device_ctrl_t *device = &server->devices_ctrl[AUDIO_DEVICE_HFP];

    if (device->is_busy && device->device.output)
    {
        audio_client_t client = rt_list_first_entry(&device->running_client_list, struct audio_client_base_t, node);
        RT_ASSERT(client);
#if 0 //AUDIO_BOX_EN
        struct rt_ringbuffer *rb = &client->ring_buf;
        if (rt_ringbuffer_space_len(rb) >= len)
        {
            putsize = rt_ringbuffer_put(rb, fifo, len);
            RT_ASSERT(putsize == len);
        }
        else
        {
            LOG_I("server client buffer full");
        }
        device->device.output(device->device.user_data, rb);
#else
        memcpy(hfp_dev_input_buf + hfp_dev_input_buf_offset, fifo, len);
        hfp_dev_input_buf_offset += len;
        if (hfp_dev_input_buf_offset >= 320)
        {
            audio_server_coming_data_t data;
            data.data = hfp_dev_input_buf;
            data.data_len = 320;
            data.reserved = 0;
            if (client->callback)
            {
                client->callback(as_callback_cmd_data_coming, client->user_data, (uint32_t)&data);
            }
            hfp_dev_input_buf_offset -= 320;
            memcpy(hfp_dev_input_buf, hfp_dev_input_buf + 320, hfp_dev_input_buf_offset);
        }

        //uplink send
        device->device.output(device->device.user_data, &client->ring_buf);
#endif
        ret = 0;
    }

    return ret;
}
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
AUDIO_API int audio_hfp_uplink_write(audio_client_t handle, uint8_t *data, uint32_t data_len)
{
    uint32_t len;
    if (!handle || handle->magic != AUDIO_CLIENT_MAGIC || !data || !data_len)
    {

        LOG_I("hfp_write invalid parameter");
        return -2;

    }
    if (handle->is_suspended)
    {
        LOG_D("hfp_write is suspend %d", handle->audio_type);
        return -1;
    }


#ifdef BLUETOOTH
    msbc_encode_process(data, data_len);
#endif

    return data_len;
}

static inline bool has_device_busy(audio_server_t *server)
{
    for (int d = 0; d < AUDIO_DEVICE_NUMBER; d++)
    {
        if (server->devices_ctrl[d].is_busy)
        {
            return true;
        }
    }
    return false;
}

static inline void client_debug_full(audio_client_t c)
{
    c->debug_full++;
    if (/*handle->debug_full == 1 ||*/ (c->debug_full & 0x7f) == 0)
    {
        LOG_I("audio_write: ring buf full %d times", c->debug_full);
    }
}

#if SOFTWARE_TX_MIX_ENABLE

static void client_callback_to_user(audio_client_t c)
{
    if (c && c->callback)
    {
        struct rt_ringbuffer *rb = &c->ring_buf;
        if (rt_ringbuffer_space_len(rb) < TX_DMA_SIZE)
        {
            c->callback(as_callback_cmd_cache_empty, c->user_data, 0);
        }
        else if (rt_ringbuffer_space_len(rb) >= rt_ringbuffer_get_size(rb) / 2)
        {
            c->callback(as_callback_cmd_cache_half_empty, c->user_data, 0);
        }
    }
}

static inline void mono2stereo(int16_t *mono, uint32_t samples, int16_t *stereo)
{
    for (int i = 0; i < samples; i++)
    {
        *stereo++ = *mono;
        *stereo++ = *mono++;
    }
}
static inline void stereo2mono(int16_t *stereo, uint32_t samples, int16_t *mono)
{
    for (int i = 0; i < samples / 2; i++)
    {
        *mono++ = *stereo++;
        stereo++;
    }
}

static int audio_write_resample(audio_client_t c,        uint8_t *data, uint32_t data_len)
{
    audio_server_t *server = get_server();
    uint32_t out_bytes;
    audio_device_e d_type;
    d_type = c->device_using;
    if (!(d_type >= AUDIO_DEVICE_SPEAKER && d_type < AUDIO_DEVICE_NUMBER))
    {
        return 0;
    }
    uint32_t src_len = data_len;

    audio_device_ctrl_t *d = &server->devices_ctrl[d_type];

    if (c->parameter.write_samplerate == d->tx_mix_dst_samplerate
            && c->parameter.write_channnel_num == d->tx_mix_dst_channel)
    {
        if (rt_ringbuffer_space_len(&c->ring_buf) < data_len)
        {
            client_debug_full(c);
            return 0;
        }
        c->debug_full = 0;
        rt_ringbuffer_put(&c->ring_buf, data, data_len);
        return data_len;
    }

    if (c->resample_dst_samplerate != d->tx_mix_dst_samplerate
            || c->resample_dst_ch != d->tx_mix_dst_channel)
    {
        LOG_I("c=%p d_r=%d to %d ch %d to %d", c, c->resample_dst_samplerate, d->tx_mix_dst_samplerate, c->resample_dst_ch, d->tx_mix_dst_channel);
        c->resample_dst_samplerate = d->tx_mix_dst_samplerate;
        c->resample_dst_ch = d->tx_mix_dst_channel;
        if (c->resample)
        {
            sifli_resample_close(c->resample);
        }
        c->resample = sifli_resample_open(c->resample_dst_ch, c->parameter.write_samplerate, c->resample_dst_samplerate);
        RT_ASSERT(c->resample);
    }

    uint8_t dst_ch = c->resample_dst_ch;
    uint8_t ch =  c->parameter.write_channnel_num;
    uint32_t new_size =  data_len * c->resample->ratio;

    if (ch == 1 && dst_ch == 2)
    {
        new_size = new_size * 2 + 16;
        if (rt_ringbuffer_space_len(&c->ring_buf) < new_size)
        {
            client_debug_full(c);
            return 0;
        }

        for (int i = 0; i < src_len / TX_DMA_SIZE; i++)
        {
            mono2stereo((int16_t *)data, TX_DMA_SIZE / 2, &c->resample_dst[0]);
            out_bytes = sifli_resample_process(c->resample, c->resample_dst, TX_DMA_SIZE * 2, 0);
            rt_ringbuffer_put(&c->ring_buf, (uint8_t *)c->resample->dst, out_bytes);
            data_len -= TX_DMA_SIZE;
            data += TX_DMA_SIZE;
        }
        if (data_len > 0)
        {
            RT_ASSERT(data_len < TX_DMA_SIZE);
            mono2stereo((int16_t *)data, data_len / 2, &c->resample_dst[0]);
            out_bytes = sifli_resample_process(c->resample, c->resample_dst, data_len * 2, 0);
            rt_ringbuffer_put(&c->ring_buf, (uint8_t *)c->resample->dst, out_bytes);
        }
    }
    else if (ch == 2 && dst_ch == 1)
    {
        new_size = (new_size >> 1) + 16;
        if (rt_ringbuffer_space_len(&c->ring_buf) < new_size)
        {
            client_debug_full(c);
            return 0;
        }

        for (int i = 0; i < src_len / TX_DMA_SIZE; i++)
        {
            stereo2mono((int16_t *)data, TX_DMA_SIZE / 2, &c->resample_dst[0]);
            out_bytes = sifli_resample_process(c->resample, c->resample_dst, TX_DMA_SIZE / 2, 0);
            rt_ringbuffer_put(&c->ring_buf, (uint8_t *)c->resample->dst, out_bytes);
            data_len -= TX_DMA_SIZE;
            data += TX_DMA_SIZE;
        }
        if (data_len > 0)
        {
            stereo2mono((int16_t *)data, data_len / 2, &c->resample_dst[0]);
            out_bytes = sifli_resample_process(c->resample, c->resample_dst, data_len / 2, 0);
            rt_ringbuffer_put(&c->ring_buf, (uint8_t *)c->resample->dst, out_bytes);
        }
    }
    else
    {
        new_size += 16;
        if (rt_ringbuffer_space_len(&c->ring_buf) < new_size)
        {
            client_debug_full(c);
            return 0;
        }

        for (int i = 0; i < data_len / TX_DMA_SIZE; i++)
        {
            out_bytes = sifli_resample_process(c->resample, (int16_t *)data, TX_DMA_SIZE, 0);
            rt_ringbuffer_put(&c->ring_buf, (uint8_t *)c->resample->dst, out_bytes);
            data_len -= TX_DMA_SIZE;
            data += TX_DMA_SIZE;
        }
        if (data_len > 0)
        {
            out_bytes = sifli_resample_process(c->resample, (int16_t *)data, data_len, 0);
            rt_ringbuffer_put(&c->ring_buf, (uint8_t *)c->resample->dst, out_bytes);
        }
    }
    return src_len;
}

static void client_mix_process(audio_client_t c1, audio_client_t c2, audio_device_ctrl_t *d)
{
    uint8_t ch;
    uint8_t ch_dst;
    uint16_t len1 = 0, len2 = 0;
    struct rt_ringbuffer *p_mix_rb = &d->tx_mixed_rb;

    int16_t m1[TX_DMA_SIZE / 2];
    int16_t m2[TX_DMA_SIZE / 2];

    // 1. mix it
    if (c1 && c2)
    {
        len1 = rt_ringbuffer_data_len(&c1->ring_buf);
        len2 = rt_ringbuffer_data_len(&c2->ring_buf);
        memset(m1, 0, sizeof(m1));
        memset(m2, 0, sizeof(m2));
        if (len1 >= TX_DMA_SIZE && len2 >= TX_DMA_SIZE)
        {
            rt_ringbuffer_get(&c1->ring_buf, (rt_uint8_t *)m1, TX_DMA_SIZE);
            rt_ringbuffer_get(&c2->ring_buf, (rt_uint8_t *)m2, TX_DMA_SIZE);

            //tws stream average, can use other mix algorithm
            for (int i = 0; i < TX_DMA_SIZE / 2; i++)
            {
                m1[i] = (m1[i] >> 1) + (m2[i] >> 1);
            }

            rt_ringbuffer_put(p_mix_rb, (rt_uint8_t *)m1, TX_DMA_SIZE);
        }
        else if (len1 < TX_DMA_SIZE && len2 < TX_DMA_SIZE)
        {
            //LOG_I("mix all empty");
        }
        else if (len1 < TX_DMA_SIZE)
        {
            //LOG_I("mix empty c=0x%p t=%d n=%s", c1, c1->audio_type, c1->name);
            rt_ringbuffer_get(&c2->ring_buf, (rt_uint8_t *)m2, TX_DMA_SIZE);
            rt_ringbuffer_put(p_mix_rb, (rt_uint8_t *)m2, TX_DMA_SIZE);
        }
        else
        {
            //LOG_I("mix empty c=0x%p t=%d n=%s", c2, c2->audio_type, c2->name);
            rt_ringbuffer_get(&c1->ring_buf, (rt_uint8_t *)m1, TX_DMA_SIZE);
            rt_ringbuffer_put(p_mix_rb, (rt_uint8_t *)m1, TX_DMA_SIZE);
        }
    }
    else if (c1)
    {
        len1 = rt_ringbuffer_data_len(&c1->ring_buf);
        if (len1 >= TX_DMA_SIZE)
        {
            rt_ringbuffer_get(&c1->ring_buf, (rt_uint8_t *)m1, TX_DMA_SIZE);
            rt_ringbuffer_put(p_mix_rb, (rt_uint8_t *)m1, TX_DMA_SIZE);
        }
    }

    if (rt_ringbuffer_space_len(p_mix_rb) >= rt_ringbuffer_get_size(p_mix_rb) / 2
            && (rt_ringbuffer_data_len(&c1->ring_buf) >= TX_DMA_SIZE || rt_ringbuffer_data_len(&c2->ring_buf) >= TX_DMA_SIZE)
            && (len1 >= 3 * TX_DMA_SIZE || len2 >= 3 * TX_DMA_SIZE))
    {
        //continue do mix
        audio_server_t *server = get_server();
        if (d->device_type == AUDIO_DEVICE_A2DP_SINK)
            rt_event_send(&server->event, AUDIO_SERVER_EVENT_TX_A2DP_SINK);
        else
            rt_event_send(&server->event, AUDIO_SERVER_EVENT_TX_HALF_EMPTY);
    }

    client_callback_to_user(c1);
    client_callback_to_user(c2);
}
#endif

static void avrcp_process(audio_client_t c1, audio_client_t c2, rt_uint32_t evt)
{
    audio_client_t first = c1;
    if (c2 && c1)
    {
        if (c2->audio_type == AUDIO_TYPE_LOCAL_MUSIC && c1->audio_type != AUDIO_TYPE_LOCAL_MUSIC)
        {
            first = c2;
        }
    }

    if (first && first->callback)
    {
        if (evt & AUDIO_SERVER_EVENT_A2DP_NEXT)
        {
            first->callback(as_callback_cmd_play_to_next, first->user_data, 0);
        }

        if (evt & AUDIO_SERVER_EVENT_A2DP_PREV)
        {
            first->callback(as_callback_cmd_play_to_prev, first->user_data, 0);
        }
        if (evt & AUDIO_SERVER_EVENT_A2DP_RESUME)
        {
            first->callback(as_callback_cmd_play_resume, first->user_data, 0);
        }
        if (evt & AUDIO_SERVER_EVENT_A2DP_PAUSE)
        {
            first->callback(as_callback_cmd_play_pause, first->user_data, 0);
        }
    }
}

void audio_server_entry()
{
    rt_uint32_t  evt;
    audio_device_ctrl_t *speaker;
    audio_device_ctrl_t *a2dp_sink;
    audio_device_ctrl_t *ble_bap_sink;
    audio_device_ctrl_t *hfp;
    audio_client_t first, second;
    audio_server_t *server = get_server();
    LOG_I("audio server run");
    speaker = &server->devices_ctrl[AUDIO_DEVICE_SPEAKER];
    a2dp_sink = &server->devices_ctrl[AUDIO_DEVICE_A2DP_SINK];
    hfp = &server->devices_ctrl[AUDIO_DEVICE_HFP];
    ble_bap_sink = &server->devices_ctrl[AUDIO_DEVICE_BLE_BAP_SINK];

    while (1)
    {

        evt = 0;
        //using rt_mb_recv() ?
        if (rt_event_recv(&server->event, AUDIO_SERVER_EVENT_ALL, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt) == RT_EOK)
        {
            //LOG_I("evt=0x%x", evt);

            if (evt & AUDIO_SERVER_EVENT_CMD)
            {
                while (1)
                {
                    if (audio_process_cmd(server) == 0)
                        break;
                }
            }
            if (!has_device_busy(server))
            {
                continue;
            }

            if ((evt & (AUDIO_SERVER_EVENT_TX_HALF_EMPTY | AUDIO_SERVER_EVENT_TX_FULL_EMPTY))
                    && speaker->tx_count)
            {
                first = device_get_tx_in_running(speaker, 0);
#if SOFTWARE_TX_MIX_ENABLE
                second = device_get_tx_in_running(speaker, 1);
                client_mix_process(first, second, speaker);
#else
                audio_server_callback_cmt_t cmd = as_callback_cmd_cache_empty;
                if (evt & AUDIO_SERVER_EVENT_TX_FULL_EMPTY)
                {
                    cmd = as_callback_cmd_cache_half_empty;
                }

                if (first && first->callback)
                    first->callback(as_callback_cmd_cache_half_empty, first->user_data, 0);
#endif
            }
            if ((evt & AUDIO_SERVER_EVENT_RX) && speaker->rx_count)
            {
                process_speaker_rx(server, &server->device_speaker_private);
            }
            if (a2dp_sink->device.output
                    && a2dp_sink->is_busy)
            {
#if (SOFTWARE_TX_MIX_ENABLE && TWS_MIX_ENABLE)
                first = device_get_tx_in_running(a2dp_sink, 0);
                second = device_get_tx_in_running(a2dp_sink, 1);
                if (first)
                {
                    if (a2dp_sink_need_trigger)
                    {
                        a2dp_sink->device.output(a2dp_sink->device.user_data, &a2dp_sink->tx_mixed_rb);
                        a2dp_sink_need_trigger = 0;
                    }
                    client_mix_process(first, second, a2dp_sink);
                }
                avrcp_process(first, second, evt);
#else
                first = device_get_tx_in_running(a2dp_sink, 0);
                if (evt & AUDIO_SERVER_EVENT_TX_A2DP_SINK)
                {
                    if (first)
                    {
                        if (a2dp_sink_need_trigger)
                        {
                            a2dp_sink->device.output(a2dp_sink->device.user_data, &first->ring_buf);
                            a2dp_sink_need_trigger = 0;
                        }
                        if (first->callback)
                            first->callback(as_callback_cmd_cache_half_empty, first->user_data, 0);
                    }
                }
                avrcp_process(first, NULL, evt);
#endif
            }

            if ((evt & AUDIO_SERVER_EVENT_TX_BLE_SINK)
                    && ble_bap_sink->device.output
                    && ble_bap_sink->is_busy)
            {
                first = device_get_tx_in_running(ble_bap_sink, 0);
                if (first && first->callback)
                {
                    first->callback(as_callback_cmd_cache_half_empty, first->user_data, 0);
                }
            }

            if ((evt & AUDIO_SERVER_EVENT_BT_DOWNLINK) && hfp->tx_count && hfp->is_busy)
            {
                bt_voice_downlink_process(1);
            }
        }
    }
    rt_mutex_detach(&server->mutex);
    rt_event_detach(&server->event);
    LOG_I("audio server exit");
}

void audio_btdownlink_entry()
{
    int          is_started = 0;
    rt_uint32_t  evt;
    audio_server_t *server = get_server();
    LOG_I("audio_btdownlink run");
    while (1)
    {
        evt = 0;
        if (rt_event_recv(&server->down_event,
                          AUDIO_SERVER_EVENT_BT_DOWNLINK | AUDIO_SERVER_EVENT_DOWN_START | AUDIO_SERVER_EVENT_DOWN_END,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt) == RT_EOK)
        {
            if (evt & AUDIO_SERVER_EVENT_DOWN_START)
            {
                is_started = 1;
                rt_event_send(&get_server()->event, AUDIO_SERVER_EVENT_DOWN_START);
            }
            if (evt & AUDIO_SERVER_EVENT_DOWN_END)
            {
                is_started = 0;
                rt_event_send(&get_server()->event, AUDIO_SERVER_EVENT_DOWN_END);
            }
#ifdef BLUETOOTH
            if ((evt & AUDIO_SERVER_EVENT_BT_DOWNLINK) && is_started)
            {
                audio_tick_in(AUDIO_DNLINK_TIME);
                bt_voice_uplink_send();

                bt_voice_downlink_process(server->is_bt_3a);
                audio_tick_out(AUDIO_DNLINK_TIME);
                audio_dnlink_time_print();
            }
#endif
        }
    }
    LOG_I("audio_btdownlink exit");
    rt_event_detach(&server->down_event);
}
int audio_server_init(void)
{
    //rt_kprintf("audio_server_init------\r\n");
    memset(audio_dump_debug, 0, sizeof(audio_dump_debug));
    for (int i = 0; i < ADUMP_NUM; i++)
    {
        audio_dump_debug[i].fd = -1;
        audio_dump_debug[i].dump_end = 1;
    }

    audio_hardware_pa_init();

    audio_server_t *server = get_server();
    memset(server, 0, sizeof(audio_server_t));
    rt_mutex_init(&server->mutex, "audio_svr", RT_IPC_FLAG_FIFO);
    rt_slist_init(&server->command_slist);
    rt_event_init(&server->event, "e_audio", RT_IPC_FLAG_FIFO);
    rt_event_init(&server->down_event, "e_downlink", RT_IPC_FLAG_FIFO);

    rt_list_init(&server->suspend_client_list);
    for (int i = 0; i < AUDIO_DEVICE_NUMBER; i++)
    {
        rt_list_init(&server->devices_ctrl[i].running_client_list);
        server->devices_ctrl[i].device_type = i;
    }

    server->volume = PRIVATE_DEFAULT_VOLUME;

    for (int i = 0; i < AUDIO_TYPE_NUMBER; i++)
    {
        server->private_volume[i] = 0xFF; //default not using private volume
        server->private_device[i] = AUDIO_DEVICE_NO_INIT;
    }

    rt_err_t err = rt_thread_init(&audio_server_tid, "audiosvr",
                                  audio_server_entry, RT_NULL,  audio_server_stack, sizeof(audio_server_stack),
                                  (RT_THREAD_PRIORITY_HIGH + RT_THREAD_PRIORITY_HIGHER), RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(RT_EOK == err);
    rt_thread_startup(&audio_server_tid);

    err = rt_thread_init(&bt_downvoice_tid, "bt_downvoice",
                         audio_btdownlink_entry, RT_NULL, bt_downvoice_stack, sizeof(bt_downvoice_stack),
                         (RT_THREAD_PRIORITY_HIGH + RT_THREAD_PRIORITY_HIGHER), RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(RT_EOK == err);
    rt_thread_startup(&bt_downvoice_tid);

    server->is_server_inited = 1;

    /*server init finished, do other init depend on server init*/
    struct audio_device speaker = {0};

    speaker.open  = audio_device_speaker_open;
    speaker.close = audio_device_speaker_close;
    speaker.output = NULL;
    speaker.user_data = &server->devices_ctrl[AUDIO_DEVICE_SPEAKER];
    server->device_speaker_private.parent = speaker.user_data;

    audio_server_register_audio_device(AUDIO_DEVICE_SPEAKER, &speaker);
    server->public_device = AUDIO_DEVICE_SPEAKER;
    return 0;
}

INIT_ENV_EXPORT(audio_server_init); //must call after all audio_proc_create which use INIT_COMPONENT_EXPORT

static audio_client_t audio_client_init(audio_type_t audio_type, audio_rwflag_t rwflag, audio_parameter_t *parameter, audio_server_callback_func callback, void *callback_userdata, audio_device_e device)
{
    uint32_t tx_ring_size;
    RT_ASSERT(parameter);
    LOG_I("audio_open type=%d d=%d rw=%d tx cache=%d rx cache=%d", audio_type, device, rwflag, parameter->write_cache_size, parameter->read_cache_size);

    RT_ASSERT(audio_type < AUDIO_TYPE_NUMBER);
    audio_client_t handle = audio_mem_calloc(1, sizeof(struct audio_client_base_t));
    RT_ASSERT(handle);
    handle->device_specified = device;
    handle->device_using = AUDIO_DEVICE_NONE;
    handle->api_event = rt_event_create("audcli", RT_IPC_FLAG_FIFO);
    RT_ASSERT(handle->api_event);
    tx_ring_size = parameter->write_cache_size;
    if (audio_type == AUDIO_TYPE_BT_VOICE)
    {
        tx_ring_size = SPEAKER_TX_BUF_SIZE;
    }
    else
    {
#if SOFTWARE_TX_MIX_ENABLE
        uint32_t resampled_ring_size;
        float size = (float)tx_ring_size * 96000.0f / handle->parameter.write_samplerate + 2048;
        resampled_ring_size = (uint32_t)size ;
        if (resampled_ring_size > 32000)
        {
            resampled_ring_size = 32000;
        }
        tx_ring_size = resampled_ring_size;
        LOG_I("audio resamped cache size=%d", tx_ring_size);
#endif
    }
    if (tx_ring_size < TX_DMA_SIZE * 2 + RT_ALIGN_SIZE)
    {
        tx_ring_size = TX_DMA_SIZE * 2 + RT_ALIGN_SIZE;
    }

    handle->magic       = AUDIO_CLIENT_MAGIC;
    handle->callback    = callback;
    handle->user_data   = callback_userdata;
    handle->audio_type  = audio_type;
    handle->rw_flag     = rwflag;
    handle->ring_pool   = audio_mem_calloc(1, tx_ring_size + RT_ALIGN_SIZE);
    RT_ASSERT(handle->ring_pool);
    rt_ringbuffer_init(&handle->ring_buf, handle->ring_pool, tx_ring_size);

    //todo, if rxflag has RD flag, alloc record ring buffer, now BT use other way to record

    rt_list_init(&handle->node);
    //only for nice debug log
    switch (audio_type)
    {
    case AUDIO_TYPE_BT_VOICE:
        handle->name = "hfp";
        break;
    case AUDIO_TYPE_BT_MUSIC:
        handle->name = "a2dp";
        break;
    case AUDIO_TYPE_NOTIFY:
        handle->name = "notify";
        break;
    case AUDIO_TYPE_ALARM:
        handle->name = "alarm";
        break;
    case AUDIO_TYPE_LOCAL_MUSIC:
        handle->name = "music";
        break;
    case AUDIO_TYPE_LOCAL_RING:
        handle->name = "ring";
        break;
    case AUDIO_TYPE_LOCAL_RECORD:
        handle->name = "record";
        break;
    case AUDIO_TYPE_MODEM_VOICE:
        handle->name = "modem";
        break;
    default:
        handle->name = "unknow";
        break;
    }

    lock();

    do
    {
        memcpy(&handle->parameter, parameter, sizeof(audio_parameter_t));
        audio_server_cmt_t *cmd = audio_mem_calloc(1, sizeof(audio_server_cmt_t));
        RT_ASSERT(cmd);
        cmd->cmd = AUDIO_CMD_OPEN;
        cmd->client = handle;
        rt_slist_append(&g_server.command_slist, &cmd->snode);
        send_cmd_event_to_server();
    }
    while (0);

    unlock();

    rt_uint32_t evt;
    rt_event_recv(handle->api_event, (1 << 1), RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
    LOG_I("audio_open ret=0x%x", handle);
    return handle;
}

AUDIO_API audio_client_t audio_open(audio_type_t audio_type, audio_rwflag_t rwflag, audio_parameter_t *parameter, audio_server_callback_func callback, void *callback_userdata)
{
    audio_device_e device = AUDIO_DEVICE_AUTO;
    if ((audio_type != AUDIO_TYPE_BT_VOICE) && ((rwflag & AUDIO_TXRX) == AUDIO_RX))
    {
        //auto record device only support mic device, pmd record should use audio_open2()
        device = AUDIO_DEVICE_SPEAKER;
        LOG_I("switch to mic record");
    }
    if (audio_type == AUDIO_TYPE_BT_VOICE || parameter->is_need_3a)
    {
        device = AUDIO_DEVICE_SPEAKER;
        LOG_I("3a voice always use speaker");
    }

    return audio_client_init(audio_type, rwflag, parameter, callback, callback_userdata, device);
}

AUDIO_API audio_client_t audio_open2(audio_type_t audio_type,
                                     audio_rwflag_t rwflag,
                                     audio_parameter_t *parameter,
                                     audio_server_callback_func callback,
                                     void *callback_userdata,
                                     audio_device_e device)
{
    return audio_client_init(audio_type, rwflag, parameter, callback, callback_userdata, device);
}

#if BT_BAP_BROADCAST_SINK
#define WATER_LEVEL_LOW_THRESHOLD       (BAP_BROADCAST_SINK_CACHE_SIZE/3)
#define WATER_LEVEL_HIGH_THRESHOLD      (BAP_BROADCAST_SINK_CACHE_SIZE * 2 / 3)
#define WATER_LEVEL_CONTIMUE_LIMITS     6
#define PLL_ADJUST_TIME_LIMITS          3   //100ppm
#define PLL_ADJUST_INTERVAL_LIMITS      30   //seconds

static inline void ble_sink_adjust_pll(struct rt_ringbuffer *rb)
{
    uint32_t len, size;
    len = rt_ringbuffer_data_len(rb);
    size = rt_ringbuffer_get_size(rb);

    //rt_kprintf("\r\nlen=%d/%d l=%d h=%d\r\n", len, size, ble_sink_low_water_level_times, ble_sink_high_water_level_times);

    if (len <= WATER_LEVEL_LOW_THRESHOLD)
    {
        ble_sink_high_water_level_times = 0;
        ble_sink_low_water_level_times++;
        //rt_kprintf("\r\nlen=%d/%d, l=%d\r\n", len, size, ble_sink_low_water_level_times);
    }

    if (len >= WATER_LEVEL_HIGH_THRESHOLD)
    {
        ble_sink_high_water_level_times++;
        ble_sink_low_water_level_times = 0;
        //rt_kprintf("\r\nlen=%d/%d, h=%d\r\n", len, size, ble_sink_high_water_level_times);
    }

    if (rt_tick_get_millisecond() - ble_sink_change_ticks < (1000 * PLL_ADJUST_INTERVAL_LIMITS))
    {
        return;
    }

    if (ble_sink_high_water_level_times > WATER_LEVEL_CONTIMUE_LIMITS)
    {
        ble_sink_high_water_level_times = 0;
        //rt_kprintf("pll add need add, cur=%d\r\n", ble_sink_pll_speed_up);
        if (ble_sink_pll_speed_up < PLL_ADJUST_TIME_LIMITS)
        {
            ble_sink_change_ticks = rt_tick_get_millisecond();
            ble_sink_pll_speed_up++;
            pll_freq_grade_set(PLL_ADD_TEN_PPM);
            rt_kprintf("pll add %d * 10 PPM\r\n", ble_sink_pll_speed_up);
        }
    }
    if (ble_sink_low_water_level_times > WATER_LEVEL_CONTIMUE_LIMITS)
    {
        ble_sink_low_water_level_times = 0;
        //rt_kprintf("pll add need sub, cur=%d\r\n", ble_sink_pll_speed_up);
        if (ble_sink_pll_speed_up > -PLL_ADJUST_TIME_LIMITS)
        {
            ble_sink_change_ticks = rt_tick_get_millisecond();
            ble_sink_pll_speed_up--;
            pll_freq_grade_set(PLL_SUB_TEN_PPM);
            rt_kprintf("pll add to %d * 10 PPM\r\n", ble_sink_pll_speed_up);
        }
    }
}
#endif


/**
  * @brief  write pcm data to downlink cache buffer
  * @param  handle value return by audio_open
  * @param  data Point to pcm data
  * @param  data_len length of data, should less than cache size
  * @retval int
  *          the retval can be one of the following values:
  *            -2: invalid parameter
  *            -1: the output for this type of audio was suspended by highest priority audio
  *            >=0: the number of bytes was write to cache, caller should check it and try write remain data lator
  */
AUDIO_API int audio_write(audio_client_t handle, uint8_t *data, uint32_t data_len)
{
    uint32_t len;
    if (!handle || handle->magic != AUDIO_CLIENT_MAGIC || !data || !data_len)
    {

        LOG_I("audio_write invalid parameter");
        return -2;

    }
    if (handle->is_suspended)
    {
        LOG_D("audio_write is suspend %d", handle->audio_type);
        return -1;
    }

#if SOFTWARE_TX_MIX_ENABLE
#if !TWS_MIX_ENABLE
    if (handle->device_using == AUDIO_DEVICE_A2DP_SINK)
    {
        goto put_raw;
    }
#endif
    int ret = audio_write_resample(handle, data, data_len);
    if (ret > 0)
    {
        handle->debug_full = 0;
    }
    return ret;
#endif

put_raw:
    if (rt_ringbuffer_space_len(&handle->ring_buf) < data_len)
    {
        client_debug_full(handle);
        return 0;
    }
#if MIX_STEREO_TO_MONO
    /*mix left & right channel for single speaker*/
    if (handle->parameter.write_channnel_num == 2)
    {
        int16_t *p = (int16_t *)data;
        int16_t left, right;
        uint32_t samples = data_len >> 1;
        for (uint32_t i = 0; i + 1 < samples; i += 2)
        {
            left = p[i];
            right = p[i + 1];
            p[i] = (left >> 1) + (right >> 1);
            p[i + 1] = p[i];
        }
    }
#endif
    handle->debug_full = 0;

#if BT_BAP_BROADCAST_SINK
    ble_sink_adjust_pll(&handle->ring_buf);
#endif

    len = rt_ringbuffer_put(&handle->ring_buf, data, data_len);
#if defined(BT_BAP_BROADCAST_SINK) || defined(BT_BAP_BROADCAST_SOURCE)
    if (len != data_len)
    {
        LOG_I("--spk cache full");
    }
#endif
    return data_len;
}

AUDIO_API int audio_read(audio_client_t handle, uint8_t *buf, uint32_t buf_size)
{
    if (!handle || handle->magic != AUDIO_CLIENT_MAGIC || !buf || !buf_size)
    {

        LOG_I("audio_read invalid parameter");
        return -2;

    }
    if (handle->is_suspended)
    {
        LOG_I("audio_read is suspend %d", handle->audio_type);
        return -1;
    }
    return 0;
}

AUDIO_API int audio_ioctl(audio_client_t handle, int cmd, void *parameter)
{
    int ret = 0;
    uint32_t gain = (uint32_t)parameter;
    if (!handle || handle->magic != AUDIO_CLIENT_MAGIC)
    {
        return -1;
    }
    LOG_D("audio_ioctl: cmd=%d", cmd);
    if (cmd == 0)
    {
        handle->is_factory_loopback = gain | 0x80;
    }
    else if (cmd == 1)
    {
        uint32_t *time_ms = (uint32_t *)parameter;
        ret = -1;
        if (parameter)
        {
            uint32_t bytes_per_second = handle->parameter.write_samplerate * handle->parameter.write_channnel_num * 2;
            if (bytes_per_second)
            {
                *time_ms = rt_ringbuffer_data_len(&handle->ring_buf)  * 1000 / bytes_per_second;
                ret = 0;
            }
        }
    }
    else if (cmd == 2)
    {
#if !SOFTWARE_TX_MIX_ENABLE
        ret = -1;
        if (handle->is_fade_vol && handle->is_fade_end)
        {
            ret = 0;
        }
#endif
    }
    else if (cmd == 3)
    {
        uint32_t *byte_left = (uint32_t *)parameter;
        ret = -1;
        if (parameter)
        {
            *byte_left = rt_ringbuffer_data_len(&handle->ring_buf);
            ret = 0;
        }
    }
    else if (cmd == -1)
    {
#if !SOFTWARE_TX_MIX_ENABLE
        lock();
        if (!handle->is_suspended)
        {
            handle->is_fade_vol = 1;
            handle->is_fade_end = 0;
            handle->fade_vol_steps = 0;
            g_server.last_tick = rt_tick_get_millisecond();
        }
        unlock();
#endif
    }
    LOG_D("audio_ioctl: cmd=%d ret=%d", cmd, ret);
    return ret;
}

AUDIO_API int audio_close(audio_client_t handle)
{
    rt_event_t event;
    rt_uint32_t evt;

    if (!handle || handle->magic != AUDIO_CLIENT_MAGIC)
    {

        LOG_I("audio_read invalid parameter");
        return -2;

    }
    LOG_I("audio_close type=%d n=%s h=0x%x", handle->audio_type, handle->name, handle);
    handle->magic = ~AUDIO_CLIENT_MAGIC;
    event = handle->api_event;

    lock();

    do
    {
        audio_server_cmt_t *cmd = audio_mem_calloc(1, sizeof(audio_server_cmt_t));
        RT_ASSERT(cmd);
        cmd->cmd = AUDIO_CMD_CLOSE;
        cmd->client = handle;
        rt_slist_append(&g_server.command_slist, &cmd->snode);
        send_cmd_event_to_server();
    }
    while (0);

    unlock();

    rt_event_recv(event, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
    rt_event_delete(event);
    LOG_I("audio_close done");
    return 0;
}

AUDIO_API void audio_server_register_listener(audio_server_listener_func func, uint32_t what_to_listen, uint32_t reserved)
{
    (void)reserved;

    //must run after audio_server_init
    RT_ASSERT(g_server.is_server_inited);

    lock();
    g_server.local_music_listener = func;
    unlock();
}

AUDIO_API void bt_rx_event_to_audio_server()
{
    audio_server_t *server = get_server();
    if (server->devices_ctrl[AUDIO_DEVICE_HFP].is_busy)
    {
#if 0//AUDIO_BOX_EN
        rt_event_send(&server->down_event, AUDIO_SERVER_EVENT_BT_DOWNLINK);
#else
        rt_event_send(&server->event, AUDIO_SERVER_EVENT_BT_DOWNLINK);
#endif
    }
    else
    {
        rt_event_send(&server->down_event, AUDIO_SERVER_EVENT_BT_DOWNLINK);
    }
}

AUDIO_API void bt_tx_event_to_audio_server()
{
    LOG_I("---------uplink");
}

int audio_server_select_public_audio_device(audio_device_e device_type)
{
    if (device_type >= AUDIO_DEVICE_NUMBER || !g_server.is_server_inited)
    {
        return -1;
    }

    lock();

    do
    {
        audio_server_cmt_t *cmd = audio_mem_calloc(1, sizeof(audio_server_cmt_t));
        RT_ASSERT(cmd);
        cmd->cmd = AUDIO_CMD_DEVICE_PUBLIC;
        cmd->client = (audio_client_t)((uint32_t)device_type);
        rt_slist_append(&g_server.command_slist, &cmd->snode);
        send_cmd_event_to_server();
    }
    while (0);

    unlock();

    return 0;
}

int audio_server_select_private_audio_device(audio_type_t audio_type, audio_device_e device_type)
{
#if 1
    UNUSED(audio_type);
    return audio_server_select_public_audio_device(device_type);
#else
    uint32_t val = 0;
    if (device_type >= AUDIO_DEVICE_NUMBER || !g_server.is_server_inited || audio_type >= AUDIO_TYPE_NUMBER)
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

int audio_server_set_public_volume(uint8_t volume)
{
    LOG_I("public volume=%d", volume);
    if (volume > 15)
        volume = 15;
    if (g_server.is_server_inited)
    {
        g_server.volume = volume;
        return 0;
    }
    return -1;
}
int audio_server_set_private_volume(audio_type_t audio_type, uint8_t volume)
{
    LOG_I("private volume[%d]=%d", audio_type, volume);
    if (volume > 15)
        volume = 15;
    if (g_server.is_server_inited && audio_type < AUDIO_TYPE_NUMBER)
    {
        g_server.private_volume[audio_type] = volume;
        return 0;
    }
    return -1;
}

uint8_t audio_server_get_private_volume(audio_type_t audio_type)
{
    if (!g_server.is_server_inited)
    {
        return PRIVATE_DEFAULT_VOLUME;
    }
    if (g_server.private_volume[audio_type] == 0xFF)
    {
        return g_server.volume;
    }
    return g_server.private_volume[audio_type];
}

int audio_server_set_public_speaker_mute(uint8_t is_mute)
{
    LOG_I("spk mute=%d", is_mute);
    g_server.public_is_tx_mute = is_mute;
    return !g_server.is_server_inited;
}

uint8_t audio_server_get_public_speaker_mute(void)
{
    return g_server.public_is_tx_mute;
}
int audio_server_set_public_mic_mute(uint8_t is_mute)
{
    LOG_I("mic mute=%d", is_mute);
    g_server.public_is_rx_mute = is_mute;
    return !g_server.is_server_inited;
}

uint8_t audio_server_get_public_mic_mute(void)
{
    return g_server.public_is_rx_mute;
}

uint8_t audio_server_get_max_volume(void)
{
    return AUDIO_MAX_VOLUME;
}

uint8_t a2dp_set_speaker_volume(uint8_t volume)
{
    uint8_t new_vol;
    audio_server_t *server = get_server();

    if (volume == 0)
    {
        new_vol = 0;
    }
    else if (volume == 0x7F)
    {
        new_vol = 15;
    }
    else if (volume < 9)
    {
        new_vol = 1;
    }
    else if (volume < 17)
    {
        new_vol = 2;
    }
    else if (volume < 26)
    {
        new_vol = 3;
    }
    else if (volume < 35)
    {
        new_vol = 4;
    }
    else if (volume < 44)
    {
        new_vol = 5;
    }
    else if (volume < 53)
    {
        new_vol = 6;
    }
    else if (volume < 62)
    {
        new_vol = 7;
    }
    else if (volume < 71)
    {
        new_vol = 8;
    }
    else if (volume < 80)
    {
        new_vol = 9;
    }
    else if (volume < 88)
    {
        new_vol = 10;
    }
    else if (volume < 97)
    {
        new_vol = 11;
    }
    else if (volume < 105)
    {
        new_vol = 12;
    }
    else if (volume < 113)
    {
        new_vol = 13;
    }
    else //if (volume < 121)
    {
        new_vol = 14;
    }

    LOG_I("a2dp set speaker volume: %d-->%d\n", volume, new_vol);
    server->private_volume[AUDIO_TYPE_BT_MUSIC] = new_vol;
    return new_vol;
}

/*
  only called by HFP
*/
void set_speaker_volume(uint8_t volume)
{
    uint8_t new_vol;

    if (volume == 0)
    {
        new_vol = 0;
    }
    else
    {
        new_vol = volume;
        if (new_vol > 15)
            new_vol = 15;
    }


    LOG_I("set speaker volume: %d-->%d\n", volume, new_vol);
    g_server.private_volume[AUDIO_TYPE_BT_VOICE] = new_vol;
}
int audio_server_register_audio_device(audio_device_e device_type, const struct audio_device *p_audio_device)
{
    audio_device_ctrl_t *dev;
    RT_ASSERT(g_server.is_server_inited && p_audio_device);
    RT_ASSERT(device_type  >= 0 && device_type < AUDIO_DEVICE_NUMBER);
    dev = &g_server.devices_ctrl[device_type];

    RT_ASSERT(dev->is_registerd == 0);
    dev->is_busy = 0;
    dev->is_registerd = 1;
    memcpy((void *)dev, p_audio_device, sizeof(struct audio_device));

    return 0;
}

int is_audio_dump_enable()
{
#if defined (AUDIO_DATA_CAPTURE_UART) && defined (RT_USING_FINSH)
    if (p_audio_dump && audio_dump_len)
    {
        return 1;
    }
#endif
    return 0;
}
int is_audio_dump_enable_type(audio_dump_type_t type)
{
    if (is_audio_dump_enable() && (DUMP_UART == audio_dump_debug[type].dump_enable))
        return 1;
    return 0;
}
void audio_dump_data(audio_dump_type_t type, uint8_t *fifo, uint32_t size)
{
    audio_dump_ctrl_t *p = &audio_dump_debug[type];

    //rt_kprintf("%s: type %d enable %d dump_end %d fd %d\n", __func__, type, p->dump_enable, p->dump_end, p->fd);

#if defined (AUDIO_DATA_CAPTURE_UART) && defined (RT_USING_FINSH)
    if (DUMP_UART == p->dump_enable)
    {
        audio_data_save_buf(type, fifo, size);
        return;
    }
#endif
}


#if defined (AUDIO_DATA_CAPTURE_UART) && defined (RT_USING_FINSH)
extern void log_pause(rt_bool_t pause);

static rt_err_t uart_tx_done(rt_device_t dev, void *buffer)
{
    if (audio_dump_sem)
    {
        rt_sem_release(audio_dump_sem);
    }

    return RT_EOK;
}


static void audio_data_start(uint32_t dump_num, bool log)
{
#ifdef RT_CONSOLE_DEVICE_NAME
    rt_device_t pDev = rt_device_find(RT_CONSOLE_DEVICE_NAME);
    if (!pDev || !(pDev->flag & RT_DEVICE_FLAG_DMA_TX))
    {
        rt_kprintf("%s: enable failed.  [%s] RT_DEVICE_FLAG_DMA_TX must be opened!!!\n", __func__, RT_CONSOLE_DEVICE_NAME);
        RT_ASSERT(pDev && (pDev->flag & RT_DEVICE_FLAG_DMA_TX));
    }
    rt_device_set_tx_complete(pDev, uart_tx_done);
#endif
    audio_dump_len = (dump_num + 2) * sizeof(audio_data_t);
    p_audio_dump = calloc(2, audio_dump_len);
    RT_ASSERT(p_audio_dump);
    rt_kprintf("%s: enable %d %d\n", __func__, dump_num, audio_dump_len);
    if (!log) log_pause(true);
    memset(audio_dump_no, 0x00, sizeof(audio_dump_no));
    if (!audio_dump_sem)
    {
        audio_dump_sem = rt_sem_create("adump", 1, RT_IPC_FLAG_FIFO);
        RT_ASSERT(audio_dump_sem);
    }
}

static void audio_data_stop(void)
{
    int i = 0;
    rt_sem_t temp = NULL;
    rt_base_t level;

    while (i < ADUMP_NUM) audio_dump_debug[i++].dump_enable = 0;
    if (p_audio_dump) free(p_audio_dump);
    p_audio_dump = NULL;
    audio_dump_len = 0;
    audio_dump_pos = 0;
    log_pause(false);
    rt_kprintf("%s: stop\n", __func__);
    if (simu_audio_data_timer)
    {
        rt_timer_stop(simu_audio_data_timer);
        rt_timer_delete(simu_audio_data_timer);
        simu_audio_data_timer = NULL;
    }

    level = rt_hw_interrupt_disable();
    if (audio_dump_sem)
    {
        temp = audio_dump_sem;
        audio_dump_sem = NULL;
    }
    rt_hw_interrupt_enable(level);

    if (temp)
    {
        rt_sem_delete(temp);
    }
}

static audio_data_t *audio_data_get_buf(bool exchange)
{
    static int i;
    audio_data_t *p = NULL;

    if (p_audio_dump)
    {
        p = (audio_data_t *)((uint8_t *)(&p_audio_dump[0]) + i * audio_dump_len);
    }

    if (exchange) i = (i + 1) & 0x01;

    return p;
}

static int audio_data_write_uart(void)
{
    audio_data_t *p = audio_data_get_buf(true);
    if (!p) return -1;
#ifdef RT_CONSOLE_DEVICE_NAME
    rt_device_t pDev = rt_device_find(RT_CONSOLE_DEVICE_NAME);
    if (pDev && (audio_dump_pos > 0))
    {
        rt_err_t err;
        err = rt_sem_take(audio_dump_sem, RT_WAITING_FOREVER);
        RT_ASSERT(RT_EOK == err);
        pDev->open_flag &= ~RT_DEVICE_FLAG_STREAM;
        pDev->open_flag |= RT_DEVICE_FLAG_DMA_TX;
        rt_device_write(pDev, 0, p, audio_dump_pos);
        if (audio_dump_log) rt_kprintf("%s: %d\n", __func__, audio_dump_pos);
    }
#endif
    audio_dump_pos = 0;
    return 0;
}

static void audio_data_save_buf(uint8_t type, uint8_t *buf, uint32_t size)
{
    const char *magic = "AUDI";
    audio_data_t *pingpong = audio_data_get_buf(false);
    if (!pingpong) return;
    if (audio_dump_log) rt_kprintf("%s: type %d pos %d %d %d\n", __func__, type, audio_dump_pos, audio_dump_pos + size + AUDIO_DATA_HEADER_LEN, audio_dump_len);
    RT_ASSERT(size == AUDIO_DATA_LEN);
    if (audio_dump_pos + size + AUDIO_DATA_HEADER_LEN > audio_dump_len)
    {
        return;
    }
    audio_data_t *p = (audio_data_t *)(((uint8_t *) pingpong) + audio_dump_pos);
    p->type = type;
    p->len = size;
    p->magic = *((uint32_t *) magic);
    p->no = audio_dump_no[type]++;
    memcpy((uint8_t *) p + AUDIO_DATA_HEADER_LEN, buf, size);
    audio_dump_pos += size + AUDIO_DATA_HEADER_LEN;
#if defined(PSRAM_CACHE_WB)
    SCB_CleanInvalidateDCache();
#endif
}

static void simu_audio_data_timer_handle(void *param)
{
    if (!p_audio_dump) return;
    static int type = 0;
    while (1)
    {
        if (audio_dump_debug[type].dump_enable)
        {
            if (audio_dump_pos + sizeof(audio_data_t) <= audio_dump_len)
            {
                uint8_t fifo[AUDIO_DATA_LEN];
                for (int i = 0; i < AUDIO_DATA_LEN; i++) fifo[i] = i;
                audio_data_save_buf(type, fifo, AUDIO_DATA_LEN);
            }
            else if (0 < audio_dump_pos)
            {
                int tick = rt_tick_get_millisecond();
                audio_data_write_uart();
                if (audio_dump_log) rt_kprintf("audio_data_write_uart: %d\n", rt_tick_get_millisecond() - tick);
                break;
            }
        }

        type = (type + 1) % ADUMP_NUM;
    }
}
bool audio_data_capture(void)
{
    return false;
}
int audio_data_cmd(int argc, char **argv)
{
    int  i = 0;
    bool log = false;
    int  buf_pos = 0;
    int  type;

    audio_dump_log = false;

    while (++i < argc)
    {
        if (0 == strcmp(argv[i], "-off") || 0 == strcmp(argv[i], "-stop"))
        {
            audio_data_stop();
            return 0;
        }
        else if (0 == strcmp(argv[i], "-test"))
        {
            simu_audio_data_timer = rt_timer_create("simu_audio_data_timer", simu_audio_data_timer_handle, (void *)0, rt_tick_from_millisecond(10), RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
            RT_ASSERT(simu_audio_data_timer);
            rt_timer_start(simu_audio_data_timer);
#if 0
            rt_kprintf("%s: test %d %d %d\n", __func__, audio_dump_pos, sizeof(audio_data_t), audio_dump_len);
            int x = rand() % ADUMP_NUM;
            if (DUMP_UART == audio_dump_debug[x].dump_enable)
            {
                if (audio_dump_pos + sizeof(audio_data_t) <= audio_dump_len)
                {
                    uint8_t fifo[AUDIO_DATA_LEN];
                    for (int i = 0; i < AUDIO_DATA_LEN; i++) fifo[i] = i;
                    audio_data_save_buf(x, fifo, AUDIO_DATA_LEN);
                }
                else if (0 < audio_dump_pos)
                {
                    //int tick = rt_tick_get_millisecond();
                    audio_data_write_uart();
                    //rt_kprintf("audio_data_write_uart: %d\n", rt_tick_get_millisecond() - tick);
                }
            }
#endif
            return 0;
        }
        else if (0 == strcmp(argv[i], "-log"))
        {
            log = 1;
            if (1 == argv[i + 1][0])
            {
                audio_dump_log = true;
                i++;
            }
        }
        else
        {
            if (0 == strcmp(argv[i], "-downlink"))
            {
                type = ADUMP_DOWNLINK;
            }
            else if (0 == strcmp(argv[i], "-downlink_agc"))
            {
                type = ADUMP_DOWNLINK_AGC;
                rt_kprintf("dump enable: dagc\n");
            }
            else if (0 == strcmp(argv[i], "-audprc"))
            {
                type = ADUMP_AUDPRC;
                rt_kprintf("dump enable: audprc\n");
            }
            else if (0 == strcmp(argv[i], "-dc_out"))
            {
                type = ADUMP_DC_OUT;
            }
            else if (0 == strcmp(argv[i], "-ramp_in_out"))
            {
                type = ADUMP_RAMP_IN_OUT;
            }
            else if (0 == strcmp(argv[i], "-aecm_input1"))
            {
                type = ADUMP_AECM_INPUT1;
                rt_kprintf("dump enable: input1\n");
            }
            else if (0 == strcmp(argv[i], "-aecm_input2"))
            {
                type = ADUMP_AECM_INPUT2;
                rt_kprintf("dump enable: input2\n");
            }
            else if (0 == strcmp(argv[i], "-aecm_out"))
            {
                type = ADUMP_AECM_OUT;
                rt_kprintf("dump enable: aecm_out\n");
            }
            else if (0 == strcmp(argv[i], "-ans_out"))
            {
                type = ADUMP_ANS_OUT;
            }
            else if (0 == strcmp(argv[i], "-agc_out"))
            {
                type = ADUMP_AGC_OUT;
            }
            else if (0 == strcmp(argv[i], "-ramp_out_out"))
            {
                type = ADUMP_RAMP_OUT_OUT;
            }
            else if (0 == strcmp(argv[i], "-pdm"))
            {
                type = ADUMP_PDM_RX;
            }
            else
            {
                continue;
            }

            if (p_audio_dump)
            {
                rt_kprintf("%s: was enabled!!! please stop it first\n", __func__);
                return 0;
            }

            audio_dump_debug[type].dump_enable = DUMP_UART;
        }
    }

    int dump_num = 0;
    audio_dump_pos = 0;
    i = 0;
    while (i < ADUMP_NUM)
    {
        if (audio_dump_debug[i++].dump_enable) dump_num++;
    }

    if (dump_num)
    {
        audio_data_start(dump_num, log);
    }

    return 0;
}

MSH_CMD_EXPORT_ALIAS(audio_data_cmd, audio_data, audio_data);
#endif

#if defined (RT_USING_FINSH)
int ae_log(int argc, char **argv)
{
    if (argc == 1)
    {
        LOG_I("ae log=%d", g_ae_log);
    }
    else
    {
        g_ae_log = (argv[1][0] == '0') ? 0 : 1;
        LOG_I("ae log=%d", g_ae_log);
    }
    return 0;
}
MSH_CMD_EXPORT_ALIAS(ae_log, ae_log, ae_log);
#endif

#endif // SOC_BF0_HCPU

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

