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

#define LOG_TAG         "audio"
#define DBG_TAG         "audio"
#define DBG_LVL          LOG_LVL_INFO
#include "log.h"


#include "bf0_hal_audprc.h"
#include "drv_audprc.h"
#ifdef SOLUTION_WATCH
    #include "app_comm.h"
#endif /* SOLUTION_WATCH */

#define PDM_DEVICE_NAME                     "pdm1"
#define PRIVATE_DEFAULT_VOLUME              10

#if defined(PKG_USING_3MICS)
    #define CODEC_DATA_UNIT_LEN             (480)
    #define AUDIO_SERVER_STACK_SIZE         (4096)
#else
    #define CODEC_DATA_UNIT_LEN             (320)
    #define AUDIO_SERVER_STACK_SIZE         (9 * 1024)
#endif

#define DEBUGT_RX_INT_COUNT     8

#define AUDIO_DATA_CAPTURE_UART

#if defined (AUDIO_DATA_CAPTURE_UART) && defined (RT_USING_FINSH)

#define AUDIO_DATA_LEN          CODEC_DATA_UNIT_LEN

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

static uint8_t current_audio_device;
static uint8_t current_play_status;

enum
{
    DUMP_NONE,
    DUMP_BLE,
    DUMP_UART,
};

/*------------------   define ----------------------*/
#define m_max(a, b)  ((a) > (b) ? (a ): (b))

#define AUDIO_API
#define AUDIO_CLIENT_MAGIC   0x66778899

#define  AUDIO_SERVER_EVENT_CMD             (1 << 0)
#define  AUDIO_SERVER_EVENT_TX              (1 << 1)
#define  AUDIO_SERVER_EVENT_RX              (1 << 2)
#define  AUDIO_SERVER_EVENT_BT_DOWNLINK     (1 << 3)
#define  AUDIO_SERVER_EVENT_BT_UPLINK       (1 << 4)
#define  AUDIO_SERVER_EVENT_TX_A2DP_SINK    (1 << 5)
#define  AUDIO_SERVER_EVENT_TX_HFP          (1 << 6)
#define  AUDIO_SERVER_EVENT_A2DP_NEXT       (1 << 7) //a2dp to AG
#define  AUDIO_SERVER_EVENT_A2DP_PREV       (1 << 8) //a2dp to AG
#define  AUDIO_SERVER_EVENT_A2DP_PAUSE      (1 << 9) //a2dp to AG
#define  AUDIO_SERVER_EVENT_A2DP_RESUME     (1 << 10) //a2dp to AG
#define  AUDIO_SERVER_EVENT_DOWN_START      (1 << 11)
#define  AUDIO_SERVER_EVENT_DOWN_END        (1 << 12)
#define  AUDIO_SERVER_EVENT_PDM_RX          (1 << 13)
#define  AUDIO_SERVER_EVENT_3MICS           (1 << 14)

#define AUDIO_SERVER_EVENT_ALL  ( \
                                AUDIO_SERVER_EVENT_CMD| \
                                AUDIO_SERVER_EVENT_TX| \
                                AUDIO_SERVER_EVENT_RX| \
                                AUDIO_SERVER_EVENT_TX_A2DP_SINK| \
                                AUDIO_SERVER_EVENT_TX_HFP| \
                                AUDIO_SERVER_EVENT_A2DP_NEXT| \
                                AUDIO_SERVER_EVENT_A2DP_PREV| \
                                AUDIO_SERVER_EVENT_A2DP_PAUSE| \
                                AUDIO_SERVER_EVENT_A2DP_RESUME| \
                                AUDIO_SERVER_EVENT_PDM_RX | \
                                AUDIO_SERVER_EVENT_3MICS | \
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

#define BT_VOICE_MIX_WITH       (AUDIO_ALARM_BIT | AUDIO_NOTIFY_BIT | AUDIO_LOCAL_RING_BIT)
#define LOCAL_RING_MIX_WITH     (AUDIO_BT_VOICE_BIT | AUDIO_ALARM_BIT | AUDIO_NOTIFY_BIT | AUDIO_LOCAL_MUSIC_BIT | AUDIO_BT_MUSIC_BIT)
#define ALARM_MIX_WITH          (AUDIO_BT_VOICE_BIT | AUDIO_LOCAL_RING_BIT | AUDIO_NOTIFY_BIT | AUDIO_LOCAL_MUSIC_BIT | AUDIO_BT_MUSIC_BIT)
#define NOTIFY_MIX_WITH         (AUDIO_BT_VOICE_BIT | AUDIO_LOCAL_RING_BIT | AUDIO_ALARM_BIT | AUDIO_LOCAL_MUSIC_BIT | AUDIO_BT_MUSIC_BIT)
#define LOCAL_MUSIC_MIX_WITH    (AUDIO_LOCAL_RING_BIT | AUDIO_ALARM_BIT | AUDIO_NOTIFY_BIT)
#define BT_MUSIC_MIX_WITH       (AUDIO_LOCAL_RING_BIT | AUDIO_ALARM_BIT | AUDIO_NOTIFY_BIT)


struct audio_client_base_t
{
    rt_list_t                   node;

    uint32_t                    magic;
    const char                  *name; //only for debug use
    rt_slist_t                  snode;
    rt_event_t                  api_event;
    audio_server_callback_func  callback;
    void                        *user_data;
    audio_parameter_t           parameter;
    struct rt_ringbuffer        ring_buf;
    uint8_t                     *ring_pool;
    audio_rwflag_t              rw_flag;
    audio_type_t                audio_type;
    uint8_t                     is_suspended;
    uint8_t                     is_factory_loopback;
    uint8_t                     debug_full;
};

#define OPEN_MAP_TX             (1 << 0)
#define OPEN_MAP_RX             (1 << 1)

typedef struct
{
    rt_device_t                 audprc_dev;
    rt_device_t                 audcodec_dev;
    rt_device_t                 pdm;
    uint8_t                     *tx_data_tmp;
    uint8_t                     *adc_data_tmp;
    uint8_t                     *pdm_data_tmp;
    uint32_t                    tx_samplerate;
    uint32_t                    rx_samplerate;
    int                         last_volume;
    uint8_t                     tx_channels;
    uint8_t                     rx_channels;
    uint8_t                     opened_map_flag;

    uint8_t                     tx_drop_num;
    uint8_t                     tx_empty_occur;
    uint8_t                     tx_full_occur;
    uint8_t                     tx_enable;
    uint8_t                     rx_uplink_send_start;
    uint8_t                     rx_channel_num;
    uint8_t                     is_eq_mute_volume;
    uint8_t                     tx_interrupt_cnt_debug;
    uint8_t                     tx_empty_cnt;
} audio_device_speaker_t;

typedef struct
{
    struct audio_device device; //must be first member
    uint8_t         is_busy;
    uint8_t         is_registerd;
} audio_device_ctrl_t;

typedef struct
{
    audio_server_listener_func  local_music_listener;
    struct rt_ringbuffer        *p_ring_buf;
    struct rt_event     event;
    struct rt_event     down_event;
    struct rt_mutex     mutex;
    rt_slist_t          command_slist;
    rt_list_t           suspend_list;
    audio_client_t      client;
    uint8_t             volume;//0~15
    uint8_t             private_volume[AUDIO_TYPE_NUMBER];
    uint8_t             is_need_3a;
    uint8_t             is_bt_music_working;
    uint8_t             is_server_inited;
    uint8_t             public_is_rx_mute;
    uint8_t             public_is_tx_mute;
    uint8_t             private_device[AUDIO_TYPE_NUMBER]; //audio_device_e
    uint8_t             public_device;  //audio_device_e
    uint8_t             last_device;    //audio_device_e

    audio_device_ctrl_t device_a2dp_sink;
    audio_device_ctrl_t device_speaker;
    audio_device_ctrl_t device_hfp;
    audio_device_ctrl_t *p_device_current;

    audio_device_speaker_t device_speaker_private;
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
static uint8_t audio_server_stack[AUDIO_SERVER_STACK_SIZE];
static uint8_t bt_downvoice_stack[2048];
static struct rt_thread audio_server_tid;
static struct rt_thread bt_downvoice_tid;
#ifdef SOLUTION_WATCH
    static uint8_t ped_dog_timer;
#endif /* SOLUTION_WATCH */


/* dump debug control*/

audio_dump_ctrl_t audio_dump_debug[ADUMP_NUM];

/*
 if could not mix, than check priority, if priority is same, then new audio suspend old audio
*/
static const audio_mix_policy_t mix_policy[AUDIO_TYPE_NUMBER] =
{
    [AUDIO_TYPE_BT_VOICE]      = {94, BT_VOICE_MIX_WITH},
    [AUDIO_TYPE_BT_MUSIC]      = {11, BT_MUSIC_MIX_WITH},
    [AUDIO_TYPE_ALARM]         = {88, ALARM_MIX_WITH},
    [AUDIO_TYPE_NOTIFY]        = {80, NOTIFY_MIX_WITH},
    [AUDIO_TYPE_LOCAL_MUSIC]   = {10, LOCAL_MUSIC_MIX_WITH},
    [AUDIO_TYPE_LOCAL_RING]    = {92, LOCAL_RING_MIX_WITH},
    [AUDIO_TYPE_LOCAL_RECORD]  = {93, 0},
    [AUDIO_TYPE_MODEM_VOICE]   = {94, 0},
};

uint8_t g_3mics_log;
uint8_t g_3mics_time;
uint8_t g_is_3mic_closing;
uint8_t g_need_3mic_sync;
uint8_t g_need_2mic_sync;
uint8_t g_need_audcodec_rx;
uint8_t g_need_audprc_rx;
uint8_t g_need_pdm_rx;
uint8_t g_pdm_int_set;
uint8_t g_mic_int_set;
uint32_t g_coming_mic;
uint32_t g_coming_pdm;
uint32_t g_coming_tx;
uint32_t g_debug_mic;
uint32_t g_mic_int_cycle[DEBUGT_RX_INT_COUNT];
uint32_t g_pdm_int_cycle[DEBUGT_RX_INT_COUNT];
uint32_t g_tx_int_cycle[DEBUGT_RX_INT_COUNT];
uint32_t g_mic_last2_int_cycle[2];
uint32_t g_pdm_last2_int_cycle[2];
uint32_t g_tx_last2_int_cycle[2];


/*------------------local function------*/
static rt_err_t speaker_tx_done(rt_device_t dev, void *buffer);
static rt_err_t mic_rx_ind(rt_device_t dev, rt_size_t size);
static rt_err_t pdm_rx_ind(rt_device_t dev, rt_size_t size);


RT_WEAK rt_err_t pm_scenario_start(pm_scenario_name_t scenario)
{
    return 0;
}
RT_WEAK rt_err_t pm_scenario_stop(pm_scenario_name_t scenario)
{
    return 0;
}

static uint8_t is_can_mix(audio_client_t client1, audio_client_t client2)
{
    //todo checke output samplerate is same
    RT_ASSERT(client1->audio_type < AUDIO_TYPE_NUMBER && client1->audio_type < AUDIO_TYPE_NUMBER);
#if 0
    LOG_D("t1=%d mw=0x%x b2=0x%x t2=%d mw=0x%x b1=0x%x", client1->audio_type,
          mix_policy[client1->audio_type].can_mix_with,
          TYPE_TO_MIX_BIT(client2->audio_type),
          client2->audio_type,
          mix_policy[client2->audio_type].can_mix_with,
          client1->audio_type);
    if ((mix_policy[client1->audio_type].can_mix_with & TYPE_TO_MIX_BIT(client2->audio_type))
            && (mix_policy[client2->audio_type].can_mix_with & TYPE_TO_MIX_BIT(client1->audio_type)))
    {
        LOG_I("allow, mix %s(%d) with %s(%d)", client1->name, client1->audio_type, client2->name, client2->audio_type);
        return 1;
    }
#endif
    LOG_I("dennied, mix %s(%d) with %s(%d)", client1->name, client1->audio_type, client2->name, client2->audio_type);
    return 0;
}

static inline audio_server_t *get_server()
{
    return &g_server;
}
uint8_t get_server_last_device(void)
{
    audio_server_t *server = get_server();
    return server->last_device;
}
uint8_t get_server_current_device(void)
{
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

extern int is_down_max_volume();

/* -----------------speaker device start----------------- */
static void inline speaker_update_volume(audio_device_speaker_t *my, int16_t spframe[], uint16_t len)
{
    audio_type_t audio_type;
    int8_t  decrease_level = 0;
    int     volx2;
    uint8_t vol = g_server.volume;
    if (g_server.p_device_current != &g_server.device_speaker
            || !g_server.device_speaker.is_busy)
    {
        return;
    }
    audio_type = g_server.client->audio_type;
    if (g_server.private_volume[audio_type] != 0xFF)
    {
        vol = g_server.private_volume[audio_type];
    }

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
            if (audio_type == AUDIO_TYPE_BT_VOICE)
                volx2 = eq_get_tel_volumex2(vol);
            else
                volx2 = eq_get_music_volumex2(vol);

            if (is_down_max_volume())
            {
                volx2 = eq_get_tel_volumex2(15);
            }

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

    if (my->is_eq_mute_volume)
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
    uint8_t chan_hw, tx_empty_cnt = 0;
    rt_uint32_t  datanum, getnum;

    if ((my->opened_map_flag & OPEN_MAP_TX) == 0 || !server->p_ring_buf || !my->tx_data_tmp)
    {
        LOG_I("invlide tx event");
        return;
    }
#ifdef SOLUTION_WATCH
    ped_dog_timer++;
    if (0 == ped_dog_timer)
    {
        app_watchdog_pet();
    }
#endif /* SOLUTION_WATCH */

    datanum = rt_ringbuffer_data_len(server->p_ring_buf);
    getnum = 0;
    if (my->tx_interrupt_cnt_debug == 0)
    {
        LOG_I("tx[0]  len:%d", datanum);
        my->tx_interrupt_cnt_debug++;
    }

    if (my->tx_enable == 0)
    {
        memset(my->tx_data_tmp, 0, CODEC_DATA_UNIT_LEN);
        if (server->is_need_3a)
        {
            audio_3a_far_put(my->tx_data_tmp, CODEC_DATA_UNIT_LEN);
        }

        bf0_audprc_device_write(my->audprc_dev, 0, my->tx_data_tmp, CODEC_DATA_UNIT_LEN);
        my->tx_empty_cnt++;
        if (!server->client->is_suspended)
        {
            LOG_I("speaker tx buffer empty %d times", my->tx_empty_cnt);
        }
    }
    else
    {
        if (rt_ringbuffer_data_len(server->p_ring_buf) < CODEC_DATA_UNIT_LEN)
        {
            memset(my->tx_data_tmp, 0, CODEC_DATA_UNIT_LEN);
            if (server->is_need_3a)
            {
                audio_3a_far_put(my->tx_data_tmp, CODEC_DATA_UNIT_LEN);
            }

            bf0_audprc_device_write(my->audprc_dev, 0, my->tx_data_tmp, CODEC_DATA_UNIT_LEN);
            my->tx_empty_cnt++;
            if (!server->client->is_suspended)
            {
                LOG_I("speaker tx buffer empty %d times", my->tx_empty_cnt);
            }
        }
        else
        {
            my->tx_empty_cnt = 0;
            getnum = rt_ringbuffer_get(server->p_ring_buf, my->tx_data_tmp, CODEC_DATA_UNIT_LEN);
            RT_ASSERT(getnum == CODEC_DATA_UNIT_LEN);
            speaker_update_volume(my, (int16_t *)my->tx_data_tmp, CODEC_DATA_UNIT_LEN / 2);
            if (server->is_need_3a)
            {
                audio_3a_far_put(my->tx_data_tmp, CODEC_DATA_UNIT_LEN);
                my->rx_uplink_send_start = 1;
            }
            //audprc_get_tx_channel();
            bf0_audprc_set_tx_channel(0);
            bf0_audprc_device_write(my->audprc_dev, 0, my->tx_data_tmp, CODEC_DATA_UNIT_LEN);
        }
        if (rt_ringbuffer_space_len(server->p_ring_buf) >= rt_ringbuffer_get_size(server->p_ring_buf) / 2)
        {
            if (server->client->callback)
                server->client->callback(as_callback_cmd_cache_half_empty, server->client->user_data, 0);
        }
    }
}

static inline void process_speaker_rx(audio_server_t *server, audio_device_speaker_t *my)
{
    if ((my->opened_map_flag & OPEN_MAP_RX) == 0 || !server->p_ring_buf || !my->adc_data_tmp)
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

    rt_size_t readlen = CODEC_DATA_UNIT_LEN;
    rt_size_t len;
    len = rt_device_read(my->audprc_dev, 0, my->adc_data_tmp, readlen);
    RT_ASSERT(len == readlen);

    //if (server->is_need_3a)
    //{
    //    audio_3a_uplink(my->adc_data_tmp, readlen, server->public_is_rx_mute, 0);
    //}

    if (server->client && server->client->is_factory_loopback)
    {
        audio_client_t client = server->client;
        uint8_t gain = client->is_factory_loopback & 0x7F;
        auido_gain_pcm((int16_t *)my->adc_data_tmp, len, gain);
        rt_ringbuffer_put(&server->client->ring_buf, my->adc_data_tmp, len);
    }

    audio_server_coming_data_t data;
    data.data = my->adc_data_tmp;
    data.data_len = len;
    data.reserved = 0;
    if (server->public_is_rx_mute)
    {
        memset(my->adc_data_tmp, 0, len);
    }
    if (server->client && server->client->callback && server->client->rw_flag != AUDIO_TX)
        server->client->callback(as_callback_cmd_data_coming, server->client->user_data, (uint32_t)&data);

}

#if !PKG_USING_3MICS_WITHOUT_ADC
static inline void process_3mics_rx(audio_server_t *server, audio_device_speaker_t *my)
{
    if ((my->opened_map_flag & OPEN_MAP_RX) == 0 || !server->p_ring_buf)
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


    rt_size_t readlen = CODEC_DATA_UNIT_LEN;
    rt_size_t len;

    readlen = CODEC_DATA_UNIT_LEN << 1;
    len = rt_device_read(my->pdm, 0, my->pdm_data_tmp, readlen);
    RT_ASSERT(len == readlen);
    audio_3a_save_pdm(my->pdm_data_tmp, len);

    readlen = CODEC_DATA_UNIT_LEN;
    len = rt_device_read(my->audprc_dev, 0, my->adc_data_tmp, readlen);
    RT_ASSERT(len == readlen);

    audio_3a_uplink2(my->adc_data_tmp, server->public_is_rx_mute);

}
#endif

static inline void process_pdm_rx(audio_server_t *server, audio_device_speaker_t *my)
{
    if ((my->opened_map_flag & OPEN_MAP_RX) == 0 || !server->p_ring_buf || !my->pdm_data_tmp)
    {
        LOG_I("invlide rx event");
        return;
    }

    rt_size_t readlen = CODEC_DATA_UNIT_LEN;
    rt_size_t len;

#if PKG_USING_3MICS_WITHOUT_ADC

#if defined (AUDIO_DATA_CAPTURE_UART) && defined (RT_USING_FINSH)
    if (p_audio_dump && audio_dump_len)
    {
        audio_data_write_uart();
    }
#endif

#endif

    readlen = CODEC_DATA_UNIT_LEN;
    if ((server->client->parameter.tsco & 0x04) || server->is_need_3a)
    {
        //has right channel, only left channel using single pdm channel
        readlen <<= 1;
    }
    //rt_kprintf("---------pdmlen=%d\r\n", readlen);
    len = rt_device_read(my->pdm, 0, my->pdm_data_tmp, readlen);
    RT_ASSERT(len == readlen);

#if PKG_USING_3MICS_WITHOUT_ADC
    if (server->is_need_3a)
    {
        audio_3a_uplink(my->pdm_data_tmp, readlen, server->public_is_rx_mute, 1);
    }
#endif

    if (server->client && server->client->callback)
    {
        //todo: put data to client receive cache buffe?
        audio_server_coming_data_t data;
        data.data = my->pdm_data_tmp;
        data.data_len = len;
        data.reserved = 1; //pdm
        if (server->public_is_rx_mute)
        {
            memset(my->pdm_data_tmp, 0, len);
        }
        if (server->client->rw_flag != AUDIO_TX)
            server->client->callback(as_callback_cmd_data_coming, server->client->user_data, (uint32_t)&data);
    }
}


static uint8_t threshold_last;
extern void pll_freq_grade_set(uint8_t gr);
static uint8_t pll_add = 1, pll_sub = 1;

#if 0
void audio_pll_dynamic_regulation(uint16_t fifo_size)
{
    struct rt_ringbuffer *rb = NULL;
    uint32_t rb_size = 0, rb_used = 0;
    uint8_t threshold_all = 0, threshold_cur = 0;

    rb = g_server.p_ring_buf;
    rb_size = rt_ringbuffer_get_size(rb);
    rb_used = rt_ringbuffer_data_len(rb);

    threshold_all = rb_size / fifo_size;
    threshold_cur = rb_used / fifo_size;

    if ((threshold_cur >= threshold_all * 5 / 8) && (threshold_last < threshold_all * 5 / 8) && pll_add)
    {
        pll_freq_grade_set(1); //PLL_ADD_TWO_HUND_PPM
        pll_add = 0;
        pll_sub = 1;
        LOG_I("pll add:%d, cur:%d, last:%d", threshold_all, threshold_cur, threshold_last);
    }

    if ((threshold_cur <= threshold_all * 3 / 8) && (threshold_last > threshold_all * 3 / 8) && pll_sub)
    {
        pll_freq_grade_set(3); //PLL_SUB_TWO_HUND_PPM
        pll_sub = 0;
        pll_add = 1;
        LOG_I("pll sub:%d, cur:%d, last:%d", threshold_all, threshold_cur, threshold_last);
    }

    threshold_last = threshold_cur;
}
#endif

void speaker_ring_put(uint8_t *fifo, uint16_t fifo_size)
{
    struct rt_ringbuffer *rb;
    rt_size_t putsize;

    RT_ASSERT(g_server.client && (g_server.p_device_current == &g_server.device_speaker));
    rb = g_server.p_ring_buf;

    audio_device_speaker_t *my = &g_server.device_speaker_private;

#if 0 //#ifndef PKG_USING_3MICS
    audio_pll_dynamic_regulation(fifo_size);
    RT_ASSERT(0); //not using now for 3mics sync
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

extern int get_pdm_volume();
static int audio_device_speaker_open(void *user_data, audio_device_input_callback callback)
{
    LOG_I("audio_device_speaker_open");
    struct rt_audio_caps caps;
    struct rt_audio_sr_convert cfg;
    rt_err_t err;
    audio_device_ctrl_t *device = (audio_device_ctrl_t *)user_data;
    audio_server_t *server = get_server();
    audio_client_t client = server->client;

    audio_device_speaker_t *my  = &server->device_speaker_private;
    memset(my, 0, sizeof(audio_device_speaker_t));
    my->tx_channels    = client->parameter.write_channnel_num;
    my->rx_channels    = client->parameter.read_channnel_num;
    my->rx_samplerate  = client->parameter.read_samplerate;
    my->tx_samplerate  = client->parameter.write_samplerate;
    my->tx_empty_occur = 1;
    my->tx_enable      = 1;

    RT_ASSERT(client);
    RT_ASSERT(device == &server->device_speaker);

    client->is_suspended = 0;
    g_is_3mic_closing    = 0;
    g_need_2mic_sync     = 0;
    g_need_3mic_sync     = 0;
    g_need_audcodec_rx   = 0;
    g_need_audprc_rx     = 0;
    g_need_pdm_rx        = 0;
    g_coming_mic         = 0;
    g_coming_pdm         = 0;
    g_coming_tx          = 0;
    g_pdm_int_set        = 0;
    g_mic_int_set        = 0;

    // 1. prepare audio 3a
    if (client->audio_type == AUDIO_TYPE_BT_VOICE)
    {
        my->tx_enable = 0;

        my->tx_drop_num = 0; // (250 - 150) / 10; // drop (250ms - 150ms) data

        if (my->tx_samplerate == 0)
        {
            my->tx_samplerate = 16000;
            my->rx_samplerate = 16000;
            LOG_W("warning! no samplerate");
        }
        audio_3a_open(my->tx_samplerate);
    }
    else if (client->audio_type == AUDIO_TYPE_BT_MUSIC)
    {
        server->is_bt_music_working = 1;
    }

    /*set eq before device open*/
    if (client->audio_type == AUDIO_TYPE_BT_VOICE)
        bf0_audprc_eq_enable_offline(0);
    else if (client->audio_type == AUDIO_TYPE_BT_MUSIC)
        bf0_audprc_eq_enable_offline(0);
    else
        bf0_audprc_eq_enable_offline(0);

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


    // 2. prepare hardware memory
    if (client->rw_flag != AUDIO_TX)
    {
        int d;
        my->pdm_data_tmp = audio_mem_malloc(CODEC_DATA_UNIT_LEN * 2);
        my->adc_data_tmp = audio_mem_malloc(CODEC_DATA_UNIT_LEN);
        d = my->pdm_data_tmp && my->adc_data_tmp;
        RT_ASSERT(d);
    }

    if (client->rw_flag != AUDIO_RX)
    {
        my->tx_data_tmp = audio_mem_malloc(CODEC_DATA_UNIT_LEN);
        RT_ASSERT(my->tx_data_tmp);
    }

    if (client->rw_flag == AUDIO_TXRX)
    {
        //pll not support diffrent samplereate now
        RT_ASSERT(my->tx_samplerate == my->rx_samplerate);
    }

    LOG_I("ch: tx=%d rx=%d,rate: tx=%d rx=%d",
          my->tx_channels,
          my->rx_channels,
          my->tx_samplerate,
          my->rx_samplerate);

    //3. open hardware
    my->audprc_dev = rt_device_find(AUDIO_SPEAKER_NAME);
    RT_ASSERT(my->audprc_dev);
    err = rt_device_open(my->audprc_dev, RT_DEVICE_FLAG_RDWR);
    RT_ASSERT(RT_EOK == err);

    my->audcodec_dev = rt_device_find(AUDIO_PRC_CODEC_NAME);
    RT_ASSERT(my->audcodec_dev);
    err = rt_device_open(my->audcodec_dev, RT_DEVICE_FLAG_WRONLY);
    RT_ASSERT(RT_EOK == err);

    //4. config hardware
    if (client->rw_flag != AUDIO_RX)
    {
        LOG_I("set tx callback");
        rt_device_set_tx_complete(my->audprc_dev, speaker_tx_done);
    }
    if (client->rw_flag != AUDIO_TX)
    {
        LOG_I("set rx callback");
        rt_device_set_rx_indicate(my->audprc_dev, mic_rx_ind);
    }

    //LOG_I("audcodec output to codec");
    int stream;
    if (client->rw_flag & AUDIO_TX)
    {
        //config TX
#define     mixer_sel  0x5150
#define     out_sel    0x5050

        /*set output: codec/mem/i2s*/
        rt_device_control(my->audcodec_dev, AUDIO_CTL_SETOUTPUT, (void *)AUDPRC_TX_TO_CODEC);

        caps.main_type = AUDIO_TYPE_OUTPUT;
        caps.sub_type = 1 << HAL_AUDCODEC_DAC_CH0;
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
        caps.udata.value   = out_sel;
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
            int volumex2 = eq_get_music_volumex2(PRIVATE_DEFAULT_VOLUME);
            if (my->tx_samplerate == 16000 || my->tx_samplerate == 8000)
                volumex2 = eq_get_tel_volumex2(PRIVATE_DEFAULT_VOLUME);

            LOG_I("no eq init volume=%d", volumex2);
            my->last_volume = MUTE_UNDER_MIN_VOLUME;
            rt_device_control(my->audcodec_dev, AUDIO_CTL_SETVOLUME, (void *)volumex2);
        }

        rt_device_control(my->audcodec_dev, AUDIO_CTL_MUTE, (void *)1);

        stream = AUDIO_STREAM_REPLAY;
        stream |= ((1 << HAL_AUDPRC_TX_CH0)  << 8);
        LOG_I("speaker START stream=0x%x", stream);
        rt_device_control(my->audprc_dev, AUDIO_CTL_START, (void *)&stream);

        stream = AUDIO_STREAM_REPLAY;
        stream |= ((1 << HAL_AUDPRC_TX_CH0)  << 8);
        LOG_I("codec START stream=0x%x", stream);
        rt_device_control(my->audcodec_dev, AUDIO_CTL_START, &stream);
    }

    if (client->rw_flag & AUDIO_RX)
    {
        //config RX, pdm need mic_bias
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
        g_need_audcodec_rx = 1;
        g_need_audprc_rx = 1;

        if (client->audio_type == AUDIO_TYPE_BT_VOICE
                || ((client->parameter.codec == 0xFF) && (client->parameter.tsco & 0x06)))
        {
            LOG_I("config pdm");
            my->pdm = rt_device_find(PDM_DEVICE_NAME);
            if (my->pdm)
            {
                rt_device_init(my->pdm);
                rt_device_open(my->pdm, RT_DEVICE_FLAG_RDONLY);
                rt_device_set_rx_indicate(my->pdm, pdm_rx_ind);
                struct rt_audio_caps caps;
                caps.main_type = AUDIO_TYPE_INPUT;
                caps.sub_type = AUDIO_DSP_PARAM;
                caps.udata.config.channels = PDM_CHANNEL_STEREO;
                caps.udata.config.samplefmt = PDM_CHANNEL_DEPTH_16BIT;
                caps.udata.config.samplerate = PDM_SAMPLE_16KHZ;
                if (client->parameter.codec == 0xFF)
                {
                    if ((client->parameter.tsco & 0x06) == 0x02)
                    {
                        caps.udata.config.channels = 1;
                    }
                    else
                    {
                        caps.udata.config.channels = 2;
                    }
                }
                rt_device_control(my->pdm, AUDIO_CTL_CONFIGURE, &caps);
                int val_db = get_pdm_volume();
                LOG_I("pdm gain=%d * 0.5db", val_db);
                rt_device_control(my->pdm, AUDIO_CTL_SETVOLUME, (void *)val_db);;
                stream = AUDIO_STREAM_PDM_PRESTART;
                LOG_I("pdm rx pre start=0x%x", stream);
                rt_device_control(my->pdm, AUDIO_CTL_START, &stream);
                g_need_pdm_rx = 1;
            }
            else
            {
                RT_ASSERT(0);
            }
        }

        if (client->audio_type != AUDIO_TYPE_BT_VOICE)
        {
            if (g_need_audcodec_rx == 1)
            {
                g_need_audcodec_rx = 0;
                stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDCODEC_ADC_CH0) << 8);
                rt_device_control(my->audcodec_dev, AUDIO_CTL_START, &stream);
            }
            if (g_need_audprc_rx == 1)
            {
                g_need_audprc_rx = 0;
                stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDPRC_RX_CH0) << 8);
                LOG_I("audprc rx start=0x%x", stream);
                rt_device_control(my->audprc_dev, AUDIO_CTL_START, &stream);
            }

            if (g_need_pdm_rx == 1)
            {
                g_need_pdm_rx = 0;
                LOG_I("pdm rx start=0x%x", stream);
                stream = AUDIO_STREAM_PDM_START;
                rt_device_control(my->pdm, AUDIO_CTL_START, &stream);
            }
            my->opened_map_flag |= OPEN_MAP_RX;
        }
    }
    if (client->rw_flag & AUDIO_TX)
    {
        audio_hardware_pa_start();
        rt_device_control(my->audcodec_dev, AUDIO_CTL_MUTE, (void *)0);
        my->opened_map_flag |= OPEN_MAP_TX;
    }
    if (client->audio_type == AUDIO_TYPE_BT_VOICE)
    {
        server->is_need_3a = 1;
#if PKG_USING_3MICS_WITHOUT_ADC
        g_need_2mic_sync = 1;
#else
        g_need_3mic_sync = 1;
#endif
    }
    return 0;
}

static int audio_device_speaker_close(void *user_data)
{
    LOG_I("%s in", __FUNCTION__);
    audio_device_ctrl_t *device = (audio_device_ctrl_t *)user_data;
    audio_server_t *server = get_server();

    RT_ASSERT(device == &server->device_speaker);

    audio_device_speaker_t *my  = &server->device_speaker_private;

    g_is_3mic_closing = 1;

    if (my->opened_map_flag == 0)
    {
        return 0;
    }
    //1. free memory
    if (my->pdm_data_tmp)
    {
        audio_mem_free(my->pdm_data_tmp);
        my->pdm_data_tmp = NULL;
    }
    if (my->adc_data_tmp)
    {
        audio_mem_free(my->adc_data_tmp);
        my->adc_data_tmp = NULL;
    }
    if (my->tx_data_tmp)
    {
        audio_mem_free(my->tx_data_tmp);
        my->tx_data_tmp = NULL;
    }

    //2. close PA
    int stream;
    if (my->opened_map_flag & OPEN_MAP_TX)
    {
        LOG_I("close PA");
        audio_hardware_pa_stop();
    }

    LOG_I("mute audcodec output");
    rt_device_control(my->audcodec_dev, AUDIO_CTL_MUTE, (void *)1);

    stream = AUDIO_STREAM_REPLAY;
    stream |= ((1 << HAL_AUDPRC_TX_CH0)  << 8);

    //LOG_I("stop replay codec=0x%x", stream);
    rt_device_control(my->audcodec_dev, AUDIO_CTL_STOP, &stream);

    stream = AUDIO_STREAM_RXandTX;
    stream |= ((1 << HAL_AUDPRC_TX_CH0) << 8);
    stream |= ((1 << HAL_AUDPRC_RX_CH0) << 8);

    //LOG_I("stop audprc=0x%x", stream);
    rt_device_control(my->audprc_dev, AUDIO_CTL_STOP, &stream);

    stream = AUDIO_STREAM_RECORD;
    stream |= ((1 << HAL_AUDCODEC_ADC_CH0) << 8);
    //LOG_I("stop record=0x%x", stream);
    rt_device_control(my->audcodec_dev, AUDIO_CTL_STOP, &stream);

    bf0_disable_pll();

    rt_device_close(my->audcodec_dev);
    rt_device_close(my->audprc_dev);
    my->audcodec_dev = NULL;
    my->audprc_dev = NULL;

    if (my->pdm)
    {
        rt_device_close(my->pdm);
        my->pdm = NULL;
    }

    my->opened_map_flag  = 0;

    LOG_I("%s out", __FUNCTION__);
    for (int i = 0; i < DEBUGT_RX_INT_COUNT; i++)
    {
        LOG_I("mic int[%d]=0x%x", i, g_mic_int_cycle[i]);
        LOG_I("pmd int[%d]=0x%x", i, g_pdm_int_cycle[i]);
        LOG_I("tx  int[%d]=0x%x", i, g_tx_int_cycle[i]);
    }
    for (int i = 0; i < 2; i++)
    {
        LOG_I("mic last int[%d]=0x%x", i, g_mic_last2_int_cycle[i]);
        LOG_I("pmd last int[%d]=0x%x", i, g_pdm_last2_int_cycle[i]);
        LOG_I("tx  last int[%d]=0x%x", i, g_tx_last2_int_cycle[i]);
    }
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

static inline int client_is_in_running(audio_client_t client, audio_server_t *server)
{
    if (client && client == server->client)
        return 1;
    return 0;
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

static int hfp_device_input_callback(audio_server_callback_cmt_t cmd, const uint8_t *buffer, uint32_t size)
{
    if (cmd == as_callback_cmd_cache_empty || cmd == as_callback_cmd_cache_half_empty)
    {
        rt_event_send(&get_server()->event, AUDIO_SERVER_EVENT_TX_HFP);
    }
    return 0;
}
static inline audio_device_ctrl_t *get_device_ctrl(audio_server_t *server, audio_device_e device_type)
{
    audio_device_ctrl_t *dev_ctrl;
    if (device_type == AUDIO_DEVICE_A2DP_SINK)
    {
        dev_ctrl = &server->device_a2dp_sink;
    }
    else if (device_type == AUDIO_DEVICE_HFP)
    {
        dev_ctrl = &server->device_hfp;
    }
    else
    {
        dev_ctrl = &server->device_speaker;
    }
    return dev_ctrl;
}

static void audio_device_open(audio_server_t *server, audio_client_t client)
{
    int ret = 0;
    audio_device_ctrl_t *device;

    uint8_t device_type = server->private_device[client->audio_type];
    if (device_type == 0xFF)
    {
        device_type = server->public_device;
    }
    device = get_device_ctrl(server, device_type);

    LOG_I("audio_device_open in busy=%d reg=%d c=%p", device->is_busy, device->is_registerd, client);
    RT_ASSERT(!device->is_busy)

    if (!device->is_registerd)
    {
        LOG_W("device %d no find", device_type);
        device = &server->device_speaker;
        device_type = AUDIO_DEVICE_SPEAKER;
    }
    server->client = client;
    server->last_device = device_type;
    device->is_busy = 1;
    server->p_device_current = device;

    RT_ASSERT(device->device.open);
    server->p_ring_buf = &client->ring_buf;
    if (device == &server->device_speaker)
    {
        ret = device->device.open(device->device.user_data, NULL);
    }
    else if (device == &server->device_a2dp_sink)
    {
        ret = device->device.open(device->device.user_data, a2dp_device_input_callback);
    }
    else if (device == &server->device_hfp)
    {
        ret = device->device.open(device->device.user_data, hfp_device_input_callback);
    }
    RT_ASSERT(0 == ret);

    LOG_I("audio_device_open out");
}

static void audio_device_close(audio_server_t *server)
{
    audio_device_ctrl_t *ctrl = server->p_device_current;
    LOG_I("audio_device_close in");
    if (!ctrl || !ctrl->is_busy)
    {
        LOG_I("audio_device_close do nothing");
        return;
    }
    if (ctrl->device.close)
    {
        ctrl->device.close(ctrl->device.user_data);
    }


    ctrl->is_busy = 0;
    server->p_device_current = NULL;
    LOG_I("audio_device_close out");
}

inline static void audio_client_start(audio_client_t client)
{
    uint8_t stream_busy = 0;
    audio_server_t *server = get_server();
    LOG_I("audio_client_start h=%x n=%s t=%d", client, client->name, client->audio_type);
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
        LOG_I("notify listern(0, 0)");
        if (server->local_music_listener)
            server->local_music_listener(0, 0);
    }
    if (server->client)
    {
        stream_busy = 1;
        RT_ASSERT(server->client);
        LOG_I("old h=%x name=%s type=%d", server->client, server->client->name, server->client->audio_type);
    }

    RT_ASSERT(!client_is_in_running(client, server));
    RT_ASSERT(!is_node_in_list(&server->suspend_list, &client->node));

    if (stream_busy == 0)
    {
        LOG_I("start first stream");

        current_play_status = 1;
        audio_device_close(server);
        audio_device_open(server, client);

        if (client->callback)
        {
            client->callback(as_callback_cmd_opened, client->user_data, 0);
        }
        goto Exit;

    }

    if (stream_busy)
    {
        //cat't mix now
        uint8_t new_pri, old_pri;

        new_pri = mix_policy[client->audio_type].priority;
        old_pri = mix_policy[server->client->audio_type].priority;
        if (new_pri >= old_pri)
        {
            audio_type_t type = server->client->audio_type;

            LOG_I("suspend old h=%x n=%s user=0x%x, only play h=%x n=%s t=%d",
                  server->client,
                  server->client->name,
                  server->client->user_data,
                  client, client->name,
                  client->audio_type);

            server->client->is_suspended = 1;

            if (server->client->callback)
            {
                server->client->callback(as_callback_cmd_suspended, server->client->user_data, 0);
                LOG_I("callback suspend old ok");
            }

            rt_list_insert_before(&server->suspend_list, &server->client->node);
            audio_device_close(server);
            audio_device_open(server, client);
        }
        else
        {
            LOG_I("suspend new h=%x n=%s t=%d new_pri=%d old_pri=%d", client, client->name, client->audio_type, new_pri, old_pri);
            client->is_suspended = 1;
            rt_list_insert_after(&server->suspend_list, &client->node);
            if (client->callback)
            {
                client->callback(as_callback_cmd_suspended, client->user_data, 0);
                LOG_I("callback suspend old ok");
            }
            goto Exit;
        }
    }
Exit:
    LOG_I("audio_client_start out");
    rt_event_send(client->api_event, (1 << 1));
}


static audio_client_t get_highest_priority_handle_in_suspend(audio_server_t *server)
{
    if (rt_list_isempty(&server->suspend_list))
    {
        return NULL;
    }
    rt_list_t *pos = NULL;
    audio_client_t client = NULL;
    audio_client_t highest = NULL;
    rt_list_for_each(pos, &server->suspend_list)
    {
        client = rt_list_entry(pos, struct audio_client_base_t, node);
        LOG_I("suspend list h=%x t=%d pri=%d", client, client->name, client->audio_type, mix_policy[client->audio_type].priority);
        if (highest == NULL || mix_policy[client->audio_type].priority > mix_policy[highest->audio_type].priority)
        {
            highest = client;
        }
    }
    return highest;
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

        server->is_need_3a = 0;
        audio_3a_close();
    }

    if (is_in_list(&server->suspend_list, &client->node))
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

    RT_ASSERT(client == server->client);

    audio_device_close(server);

    audio_mem_free(client->ring_pool);
    client->magic = 0;
    rt_event_send(client->api_event, 1);
    audio_mem_free(client);
    server->client = NULL;
    server->p_ring_buf = NULL;

    if (rt_list_isempty(&server->suspend_list))
    {
        LOG_I("suspend empty");
        if (server->local_music_listener)
        {
            LOG_I("notify listern(1, 0)");
            server->local_music_listener(1, 0);
        }
        goto Exit;
    }

    LOG_I("running some suspend");

    audio_client_t suspend1 = get_highest_priority_handle_in_suspend(server);
    if (suspend1)
    {
        LOG_I("resume h=0x%x, n=%s t=%d", suspend1, suspend1->name, suspend1->audio_type);
        rt_list_remove(&suspend1->node);
        audio_device_open(server, suspend1);

        suspend1->is_suspended = 0;
        if (suspend1->callback && suspend1->magic == AUDIO_CLIENT_MAGIC)
        {
            suspend1->callback(as_callback_cmd_resumed, suspend1->user_data, 0);
            LOG_I("callback resume old ok");
        }
    }

    /*todo: check recording work client numbers, only BT voice has highest priority to use mic*/
Exit:

#ifdef RT_USING_PM
    audio_pm_debug--;
#ifdef SF32LB52X
    if (audio_type == AUDIO_TYPE_BT_VOICE)
    {
        HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
        LOG_I("audio release lcpu, 3a=%d", server->is_need_3a);
    }

    if (audio_pm_debug == 0)
    {
        pm_scenario_stop(PM_SCENARIO_AUDIO);
    }
#endif
    rt_pm_hw_device_stop();
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif

    RT_ASSERT(audio_pm_debug >= 0);
    LOG_I("audio_client_stop out");
}

int flag = 1;
inline static int audio_process_cmd(audio_server_t *server)
{
    int ret = 0;
    audio_device_ctrl_t *dev_new;
    audio_device_ctrl_t *dev_old;
    uint32_t val;
    uint8_t audio_type;
    uint8_t device_type;

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
            if (flag == 0)
            {
                flag = 1;
            }
        }
        else if (cmd_e == AUDIO_CMD_CLOSE)
        {
            RT_ASSERT(client && client->magic == ~AUDIO_CLIENT_MAGIC);
            audio_client_stop(client);
        }
        else if (cmd_e == AUDIO_CMD_DEVICE_PUBLIC)
        {
            server->public_device = (uint8_t)((uint32_t)client);;
            if (server->public_device == server->last_device)
            {
                current_audio_device = device_type;
                break;
            }
            dev_new = get_device_ctrl(server, server->public_device);
            dev_old = get_device_ctrl(server, server->last_device);
            if (server->client && (server->private_device[server->client->audio_type] != 0xFF))
            {
                LOG_W("using private device");
                break;
            }
            current_audio_device = device_type;
            if (server->client && dev_old->is_busy)
            {
                audio_device_close(server);
                audio_device_open(server, server->client);
            }
        }
        else if (cmd_e == AUDIO_CMD_DEVICE_PRIVATE)
        {
            val = (uint32_t)client;
            audio_type = (val >> 8) & 0xFF;
            device_type = val & 0xFF;
            server->private_device[audio_type] = device_type;
            if (device_type == server->last_device)
            {
                current_audio_device = device_type;
                break;
            }
            dev_new = get_device_ctrl(server, device_type);
            dev_old = get_device_ctrl(server, server->last_device);
            current_audio_device = device_type;
            rt_kprintf("client = %p,is_busy = %d,play_status = %d\n", server->client, dev_old->is_busy, get_server_current_play_status());
            if (server->client && dev_old->is_busy)
            {
                audio_device_close(server);
                audio_device_open(server, server->client);
            }
        }
        else if (cmd_e == AUDIO_CMD_PAUSE || cmd_e == AUDIO_CMD_RESUME)
        {
            if (&server->device_a2dp_sink == server->p_device_current
                    && server->device_a2dp_sink.device.ioctl)
            {
                // 0--pause
                // 1-- resume
                int cmd = (cmd_e == AUDIO_CMD_PAUSE) ? 0 : 1;
                LOG_I("a2dp device ioctl %d", cmd);
                server->device_a2dp_sink.device.ioctl(server->device_a2dp_sink.device.user_data, cmd, NULL);

                if (cmd == 1)
                {
                    flag = 1;
                }
            }

            if (cmd_e == AUDIO_CMD_PAUSE)
            {
                current_play_status = 0;
            }
            else
            {
                current_play_status = 1;
            }
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

static rt_err_t speaker_tx_done(rt_device_t dev, void *buffer)
{
    //in inturrupt
    //rt_kprintf("---tx done\r\n");
    RT_ASSERT(g_server.p_device_current == &g_server.device_speaker);
#if PKG_USING_3MICS_WITHOUT_ADC
    if (g_need_2mic_sync == 1)
    {
        g_need_2mic_sync++;
#else
    if (g_need_3mic_sync == 1)
    {
        g_need_3mic_sync++;
#endif
        audio_device_speaker_t *my  = &g_server.device_speaker_private;
        if ((my->opened_map_flag & OPEN_MAP_RX) == 0)
        {
            int stream;
            //rt_kprintf("\r\n---tx done config rx\r\n");
            rt_base_t level = rt_hw_interrupt_disable();
            if (g_need_audcodec_rx == 1)
            {
                g_need_audcodec_rx = 0;
                stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDCODEC_ADC_CH0) << 8);
                //rt_kprintf("int start audcodec rx=0x%x\r\n", stream);
                rt_device_control(my->audcodec_dev, AUDIO_CTL_START, &stream);
            }

            if (g_need_audprc_rx == 1)
            {
                g_need_audprc_rx = 0;
                stream = AUDIO_STREAM_RECORD | ((1 << HAL_AUDPRC_RX_CH0) << 8);
                //rt_kprintf("int start audprc rx=0x%x\r\n", stream);
                rt_device_control(my->audprc_dev, AUDIO_CTL_START, &stream);
            }

            if (g_need_pdm_rx == 1)
            {
                g_need_pdm_rx = 0;
                //rt_kprintf("int start pdm rx=0x%x\r\n", stream);
                stream = AUDIO_STREAM_PDM_START;
                rt_device_control(my->pdm, AUDIO_CTL_START, &stream);
            }
            rt_hw_interrupt_enable(level);
            my->opened_map_flag |= OPEN_MAP_RX;
        }
        else
        {
            //rt_kprintf("---tx done no need config rx\r\n");
        }
    }

#if defined(DEBUG_FRAME_SYNC) && !defined(PKG_USING_3MICS_WITHOUT_ADC)
    else if (g_need_3mic_sync == 2 && g_coming_tx < DEBUGT_RX_INT_COUNT)
    {
        g_tx_int_cycle[g_coming_tx++] = HAL_DBG_DWT_GetCycles();
    }
    g_tx_last2_int_cycle[0] = g_tx_last2_int_cycle[1];
    g_tx_last2_int_cycle[1] = HAL_DBG_DWT_GetCycles();
#endif

    rt_event_send(&g_server.event, AUDIO_SERVER_EVENT_TX);

    return RT_EOK;
}



static rt_err_t mic_rx_ind(rt_device_t dev, rt_size_t size)
{
    //in inturrupt
#if PKG_USING_3MICS_WITHOUT_ADC
    return RT_EOK;
#endif

#ifdef DEBUG_FRAME_SYNC
    if (g_coming_mic < DEBUGT_RX_INT_COUNT)
    {
        g_mic_int_cycle[g_coming_mic] = HAL_DBG_DWT_GetCycles();
    }
    g_mic_last2_int_cycle[0] = g_mic_last2_int_cycle[1];
    g_mic_last2_int_cycle[1] = HAL_DBG_DWT_GetCycles();;
#endif
    g_coming_mic++;
    g_mic_int_set = 1;

    if (g_need_3mic_sync)
    {
        if (g_mic_int_set && g_pdm_int_set)
        {
            g_mic_int_set = 0;
            g_pdm_int_set = 0;
            rt_event_send(&g_server.event, AUDIO_SERVER_EVENT_3MICS);
        }
        else
        {
#ifdef DEBUG_FRAME_SYNC
            int d = g_coming_mic - g_coming_pdm;
            if (d != 1 && d != -1 && !g_is_3mic_closing)
            {
                log_pause(false);
                rt_kprintf("mic: mic=%d pdm=%d\r\n", g_coming_mic, g_coming_pdm);
                for (int i = 0; i < DEBUGT_RX_INT_COUNT; i++)
                {
                    rt_kprintf("mic int[%d]=0x%x\r\n", i, g_mic_int_cycle[i]);
                    rt_kprintf("pmd int[%d]=0x%x\r\n", i, g_pdm_int_cycle[i]);
                }
                for (int i = 0; i < 2; i++)
                {
                    rt_kprintf("mic last int[%d]=0x%x\r\n", i, g_mic_last2_int_cycle[i]);
                    rt_kprintf("pmd last int[%d]=0x%x\r\n", i, g_pdm_last2_int_cycle[i]);
                }
                RT_ASSERT(0);
            }
#endif
        }
    }
    else
    {
        rt_event_send(&g_server.event, AUDIO_SERVER_EVENT_RX);
    }
    return RT_EOK;
}

static rt_err_t pdm_rx_ind(rt_device_t dev, rt_size_t size)
{
    //rt_kprintf("pdmlen rx=%d\r\n",size);

    //in inturrupt
#ifdef DEBUG_FRAME_SYNC
    if (g_coming_pdm < DEBUGT_RX_INT_COUNT)
    {
        g_pdm_int_cycle[g_coming_pdm] = HAL_DBG_DWT_GetCycles();
    }
    g_pdm_last2_int_cycle[0] = g_pdm_last2_int_cycle[1];
    g_pdm_last2_int_cycle[1] = HAL_DBG_DWT_GetCycles();
#endif
    g_coming_pdm++;
    g_pdm_int_set = 1;

#if !PKG_USING_3MICS_WITHOUT_ADC
    if (g_need_3mic_sync)
    {
        if (g_mic_int_set && g_pdm_int_set)
        {
            g_mic_int_set = 0;
            g_pdm_int_set = 0;
            rt_event_send(&g_server.event, AUDIO_SERVER_EVENT_3MICS);
        }
        else
        {
#ifdef DEBUG_FRAME_SYNC
            int d = g_coming_mic - g_coming_pdm;
            if (d != 1 && d != -1 &&  !g_is_3mic_closing)
            {
                log_pause(false);
                rt_kprintf("pdm: mic=%d pdm=%d\r\n", g_coming_mic, g_coming_pdm);
                for (int i = 0; i < DEBUGT_RX_INT_COUNT; i++)
                {
                    rt_kprintf("mic int[%d]=0x%x\r\n", i, g_mic_int_cycle[i]);
                    rt_kprintf("pmd int[%d]=0x%x\r\n", i, g_pdm_int_cycle[i]);
                }
                for (int i = 0; i < 2; i++)
                {
                    rt_kprintf("mic last int[%d]=0x%x\r\n", i, g_mic_last2_int_cycle[i]);
                    rt_kprintf("pmd last int[%d]=0x%x\r\n", i, g_pdm_last2_int_cycle[i]);
                }
                RT_ASSERT(0);
            }
#endif
        }
    }
    else
#endif
    {
        rt_event_send(&g_server.event, AUDIO_SERVER_EVENT_PDM_RX);
    }
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
    audio_device_ctrl_t *device = get_device_ctrl(server, AUDIO_DEVICE_HFP);
    struct rt_ringbuffer *rb = &(server->client->ring_buf);

    if (device->is_busy && device->device.output)
    {
        ret = 0;
        if (rt_ringbuffer_space_len(rb) >= len)
        {
            putsize = rt_ringbuffer_put(rb, fifo, len);
            RT_ASSERT(putsize == len);
        }
        else
        {
            LOG_I("server client buffer full");
        }
        server->device_hfp.device.output(device->device.user_data, rb);
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

        LOG_I("audio_write invalid parameter");
        return -2;

    }
    if (handle->is_suspended)
    {
        LOG_I("audio_write is suspend %d", handle->audio_type);
        return -1;
    }


    msbc_encode_process(data, data_len);

    return data_len;
}

void audio_server_entry()
{
    rt_uint32_t  evt;
    audio_server_t *server = get_server();
    LOG_I("audio server run");
    while (1)
    {
        evt = 0;
        if (rt_event_recv(&server->event, AUDIO_SERVER_EVENT_ALL, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt) == RT_EOK)
        {
            LOG_D("evt=0x%x", evt);

            if (evt & AUDIO_SERVER_EVENT_CMD)
            {
                while (1)
                {
                    if (audio_process_cmd(server) == 0)
                        break;
                }
            }

            if ((evt & AUDIO_SERVER_EVENT_TX) && server->p_device_current == &server->device_speaker)
            {
                process_speaker_tx(server, &server->device_speaker_private);
            }
            if ((evt & AUDIO_SERVER_EVENT_3MICS) && server->p_device_current == &server->device_speaker)
            {
#if !PKG_USING_3MICS_WITHOUT_ADC
                process_3mics_rx(server, &server->device_speaker_private);
#endif
            }
            else
            {
                if ((evt & AUDIO_SERVER_EVENT_RX) && server->p_device_current == &server->device_speaker)
                {
                    process_speaker_rx(server, &server->device_speaker_private);
                }

                if ((evt & AUDIO_SERVER_EVENT_PDM_RX) && server->p_device_current == &server->device_speaker)
                {
                    process_pdm_rx(server, &server->device_speaker_private);
                }
            }
            if (server->device_a2dp_sink.device.output
                    && server->p_device_current == &server->device_a2dp_sink)
            {
                if (evt & AUDIO_SERVER_EVENT_TX_A2DP_SINK)
                {
                    if (flag)
                    {
                        server->device_a2dp_sink.device.output(server->device_a2dp_sink.device.user_data, &server->client->ring_buf);
                        flag = 0;
                    }
                    if (rt_ringbuffer_space_len(&server->client->ring_buf) > server->client->parameter.write_cache_size / 2)
                    {
                        if (server->client->callback)
                            server->client->callback(as_callback_cmd_cache_half_empty, server->client->user_data, 0);
                    }
                }
                if (server->client->callback)
                {
                    if (evt & AUDIO_SERVER_EVENT_A2DP_NEXT)
                    {
                        server->client->callback(as_callback_cmd_play_to_next, server->client->user_data, 0);
                    }

                    if (evt & AUDIO_SERVER_EVENT_A2DP_PREV)
                    {
                        server->client->callback(as_callback_cmd_play_to_prev, server->client->user_data, 0);
                    }
                    if (evt & AUDIO_SERVER_EVENT_A2DP_RESUME)
                    {
                        server->client->callback(as_callback_cmd_play_resume, server->client->user_data, 0);
                    }
                    if (evt & AUDIO_SERVER_EVENT_A2DP_PAUSE)
                    {
                        server->client->callback(as_callback_cmd_play_pause, server->client->user_data, 0);
                    }
                }
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

            if ((evt & AUDIO_SERVER_EVENT_BT_DOWNLINK) && is_started)
            {
                audio_tick_in(AUDIO_DNLINK_TIME);
                bt_voice_uplink_send();

                bt_voice_downlink_process(server->is_need_3a);
                audio_tick_out(AUDIO_DNLINK_TIME);
                audio_dnlink_time_print();
            }
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
#ifdef PA_USING_AW87390
    sifli_aw87390_init();
#elif defined(PA_USING_AW882XX)
    rt_aw882xx_init();
#elif defined(PA_USING_SIA8150)
    sifli_sia8150_init();
#endif

    audio_server_t *server = get_server();
    memset(server, 0, sizeof(audio_server_t));
    rt_mutex_init(&server->mutex, "audio_svr", RT_IPC_FLAG_FIFO);
    rt_list_init(&server->suspend_list);
    rt_slist_init(&server->command_slist);
    rt_event_init(&server->event, "e_audio", RT_IPC_FLAG_FIFO);
    rt_event_init(&server->down_event, "e_downlink", RT_IPC_FLAG_FIFO);

    server->volume = PRIVATE_DEFAULT_VOLUME;

    for (int i = 0; i < AUDIO_TYPE_NUMBER; i++)
    {
        server->private_volume[i] = 0xFF; //default not using private volume
        server->private_device[i] = 0xFF;
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

    server->p_device_current = &server->device_speaker;
    speaker.open  = audio_device_speaker_open;
    speaker.close = audio_device_speaker_close;
    speaker.output = NULL;
    speaker.user_data = &server->device_speaker;

    audio_server_register_audio_device(AUDIO_DEVICE_SPEAKER, &speaker);
    server->public_device = server->last_device = AUDIO_DEVICE_SPEAKER;
    return 0;
}

INIT_ENV_EXPORT(audio_server_init); //must call after all audio_proc_create which use INIT_COMPONENT_EXPORT

AUDIO_API audio_client_t audio_open(audio_type_t audio_type, audio_rwflag_t rwflag, audio_parameter_t *parameter, audio_server_callback_func callback, void *callback_userdata)
{
    uint32_t tx_ring_size;
    RT_ASSERT(parameter);
    LOG_I("audio_open type=%d rwflag=%d tx cache=%d rx cache=%d", audio_type, rwflag, parameter->write_cache_size, parameter->read_cache_size);

    RT_ASSERT((uint32_t)&audio_server_stack[0] < 0x60000000); //check stack in sram
    RT_ASSERT((uint32_t)&bt_downvoice_stack[0] < 0x60000000);

    if (audio_type >= AUDIO_TYPE_NUMBER)
        return NULL;
    audio_client_t handle = audio_mem_calloc(1, sizeof(struct audio_client_base_t));
    RT_ASSERT(handle);
    handle->api_event = rt_event_create("audcli", RT_IPC_FLAG_FIFO);
    RT_ASSERT(handle->api_event);
    tx_ring_size = parameter->write_cache_size;
    if (audio_type == AUDIO_TYPE_BT_VOICE)
    {
        tx_ring_size = SPEAKER_TX_BUF_SIZE;
#ifdef DEBUG_FRAME_SYNC
        HAL_DBG_DWT_Init();
#endif
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

    rt_slist_init(&handle->snode);
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
        LOG_I("audio_write is suspend %d", handle->audio_type);
        return -1;
    }


    if (rt_ringbuffer_space_len(&handle->ring_buf) < data_len)
    {
        handle->debug_full++;
        if (/*handle->debug_full == 1 ||*/ (handle->debug_full & 0x7f) == 0)
        {
            LOG_I("audio_write: ring buf full %d times", handle->debug_full);
        }
        return 0;
    }

    handle->debug_full = 0;
    len = rt_ringbuffer_put(&handle->ring_buf, data, data_len);
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
    uint8_t gain = (uint8_t)parameter;
    if (!handle || handle->magic != AUDIO_CLIENT_MAGIC)
    {
        return -1;
    }
    LOG_I("audio_ioctl: cmd=%d", cmd);
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
    if (cmd == 2 || cmd == 3)
    {
        lock();

        audio_server_cmt_t *p_cmd = audio_mem_calloc(1, sizeof(audio_server_cmt_t));
        RT_ASSERT(p_cmd);
        p_cmd->cmd = (cmd == 2) ? AUDIO_CMD_PAUSE : AUDIO_CMD_RESUME;
        p_cmd->client = handle;
        rt_slist_append(&g_server.command_slist, &p_cmd->snode);
        send_cmd_event_to_server();

        unlock();
    }
    LOG_I("audio_ioctl: ret=%d", ret);
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
    rt_event_send(&get_server()->down_event, AUDIO_SERVER_EVENT_BT_DOWNLINK);
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
int audio_server_register_audio_device(audio_device_e device_type, struct audio_device *p_audio_device)
{
    audio_device_ctrl_t *dev;
    RT_ASSERT(g_server.is_server_inited && p_audio_device);

    if (AUDIO_DEVICE_A2DP_SINK == device_type)
    {
        dev = (void *)&g_server.device_a2dp_sink;
    }
    else if (AUDIO_DEVICE_SPEAKER == device_type)
    {
        dev = (void *)&g_server.device_speaker;
    }
    else if (AUDIO_DEVICE_HFP == device_type)
    {
        dev = (void *)&g_server.device_hfp;
    }
    else
    {
        return -1;
    }
    RT_ASSERT(dev->is_registerd == 0);
    dev->is_busy = 0;
    dev->is_registerd = 1;
    memcpy((void *)dev, p_audio_device, sizeof(struct audio_device));

    return 0;
}

bool audio_data_capture(void)
{
    return false;
}
void audio_dump_stop()
{
}
void audio_dump_start()
{
}
void audio_dump_data(audio_dump_type_t type, uint8_t *fifo, uint32_t size)
{
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
    rt_kprintf("%s: enable %d %d dma=0x%p\n", __func__, dump_num, audio_dump_len, p_audio_dump);
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
    RT_ASSERT(audio_dump_pos + size + AUDIO_DATA_HEADER_LEN <= audio_dump_len && size == AUDIO_DATA_LEN);
    audio_data_t *p = (audio_data_t *)(((uint8_t *) pingpong) + audio_dump_pos);
    p->type = type;
    p->len = size;
    p->magic = *((uint32_t *) magic);
    p->no = audio_dump_no[type]++;
    memcpy((uint8_t *) p + AUDIO_DATA_HEADER_LEN, buf, size);
    audio_dump_pos += size + AUDIO_DATA_HEADER_LEN;
#if defined(SYS_HEAP_IN_PSRAM) && defined(PSRAM_CACHE_WB)
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

int audio_data_cmd(int argc, char **argv)
{
    int  i = 0;
    bool log = false;
    int  buf_pos = 0;
    int  type;

    audio_dump_log = false;

    while (++i < argc)
    {
        if (0 == strcmp(argv[i], "3mics_logon"))
        {
            g_3mics_log = 1;
        }
        else if (0 == strcmp(argv[i], "3mics_logoff"))
        {
            g_3mics_log = 0;
        }
        else if (0 == strcmp(argv[i], "3mics_timeon"))
        {
            g_3mics_time = 1;
        }
        else if (0 == strcmp(argv[i], "3mics_timeoff"))
        {
            g_3mics_time = 0;
        }
        else if (0 == strcmp(argv[i], "-off") || 0 == strcmp(argv[i], "-stop"))
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


#endif // SOC_BF0_HCPU

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

