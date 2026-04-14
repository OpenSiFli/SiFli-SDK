/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <audio_server.h>
#include "bap_broadcast_src_api.h"
#include "lc3.h"

#define BLE_AUDIO_API
#define BEL_SRC_THREAD_NAME     "BleSrcPcm"

#define BAP_SRC_USING_10MS_TIMER        0
#define BAP_SRC_AUDIO_CACHE_SIZE        (SPEAKER_10MS_DMA_SIZE * 4)
//#define BAP_SRC_AUDIO_CACHE_SIZE        (32000)
#define LC3_ENCODER_STACK_SIZE          (4 * 4096)
#define LC3_ENCODER_PRIORITY            10
static rt_timer_t  g_timer;
static rt_event_t g_run_event;
#define BAP_EVENT_THREAD_START          (1<<0)
#define BAP_EVENT_THREAD_EXIT           (1<<1)
#define BAP_EVENT_THREAD_EXIT_DONE      (1<<2)
#define BAP_EVENT_SEND                  (1<<3)


BUILD_ASSERT(strlen(CONFIG_BROADCAST_CODE) <= BT_AUDIO_BROADCAST_CODE_SIZE,
             "Invalid broadcast code");

/* Zephyr Controller works best while Extended Advertising interval to be a multiple
 * of the ISO Interval minus 10 ms (max. advertising random delay). This is
 * required to place the AUX_ADV_IND PDUs in a non-overlapping interval with the
 * Broadcast ISO radio events.
 *
 * I.e. for a 7.5 ms ISO interval use 90 ms minus 10 ms ==> 80 ms advertising
 * interval.
 * And, for 10 ms ISO interval, can use 90 ms minus 10 ms ==> 80 ms advertising
 * interval.
 */
#define BT_LE_EXT_ADV_CUSTOM BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV, 0x0080, 0x0080, NULL)

/* When BROADCAST_ENQUEUE_COUNT > 1 we can enqueue enough buffers to ensure that
 * the controller is never idle
 */
#define BROADCAST_ENQUEUE_COUNT 6U


#define TOTAL_BUF_NEEDED        (BROADCAST_ENQUEUE_COUNT * CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT)

BUILD_ASSERT(CONFIG_BT_ISO_TX_BUF_COUNT >= TOTAL_BUF_NEEDED,
             "CONFIG_BT_ISO_TX_BUF_COUNT should be at least "
             "BROADCAST_ENQUEUE_COUNT * CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT");

#if defined(CONFIG_BAP_BROADCAST_16_2_1)

static struct bt_bap_lc3_preset preset_active = BT_BAP_LC3_BROADCAST_PRESET_16_2_1(
            BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT,
            BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

#define BROADCAST_SAMPLE_RATE 16000

#elif defined(CONFIG_BAP_BROADCAST_24_2_1)

static struct bt_bap_lc3_preset preset_active = BT_BAP_LC3_BROADCAST_PRESET_24_2_1(
            BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT,
            BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

#define BROADCAST_SAMPLE_RATE 24000

#elif defined(BAP_BROADCAST_48_2_1)
#define CONFIG_BAP_BROADCAST_48_2_1 BAP_BROADCAST_48_2_1

static struct bt_bap_lc3_preset preset_active = BT_BAP_LC3_BROADCAST_PRESET_48_2_1(
            BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT,
            BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED);

#define BROADCAST_SAMPLE_RATE 48000

#endif

#if defined(CONFIG_BAP_BROADCAST_16_2_1)
    #define MAX_SAMPLE_RATE 16000
#elif defined(CONFIG_BAP_BROADCAST_24_2_1)
    #define MAX_SAMPLE_RATE 24000
#elif defined(CONFIG_BAP_BROADCAST_48_2_1)
    #define MAX_SAMPLE_RATE 48000
#endif

#define MAX_FRAME_DURATION_US 10000
#define MAX_NUM_SAMPLES       ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)

static struct broadcast_source_stream
{
    struct bt_bap_stream stream;
    uint16_t seq_num;
    size_t sent_cnt;
    lc3_encoder_t lc3_encoder;
#if defined(CONFIG_BAP_BROADCAST_16_2_1)
    lc3_encoder_mem_16k_t lc3_encoder_mem;
#elif defined(CONFIG_BAP_BROADCAST_24_2_1) || defined(CONFIG_BAP_BROADCAST_48_2_1)
    lc3_encoder_mem_48k_t lc3_encoder_mem;
#endif
} streams[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];

static struct bt_bap_broadcast_source *broadcast_source;

NET_BUF_POOL_FIXED_DEFINE(tx_pool, TOTAL_BUF_NEEDED, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
                          CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);


static int16_t send_pcm_data[MAX_NUM_SAMPLES * 2]; //may dual channel
static bool stopping;
static bool busy;

static K_SEM_DEFINE(sem_started, 0U, ARRAY_SIZE(streams));
static K_SEM_DEFINE(sem_stopped, 0U, ARRAY_SIZE(streams));
static K_SEM_DEFINE(lc3_encoder_sem, 0U, TOTAL_BUF_NEEDED);

static int freq_hz;
static int frame_duration_us;
static int frames_per_sdu;
static int octets_per_frame;
static uint8_t g_tx_channels;
static audio_device_input_callback g_callback;


static uint32_t time_cnt;
static struct bt_le_ext_adv *adv;

static uint8_t rb_cache_pool[BAP_SRC_AUDIO_CACHE_SIZE];
static struct rt_ringbuffer rb_cache;

static int bap_broadcast_src_init(void);

static int send_data(struct broadcast_source_stream *source_stream, int channel_index, bool src_is_dual_channel)
{
    struct bt_bap_stream *stream = &source_stream->stream;
    struct net_buf *buf;
    int ret = -1;
    int step = src_is_dual_channel ? 2 : 1;
    if (!src_is_dual_channel)
    {
        channel_index = 0;
    }
    if (stopping)
    {
        return ret;
    }

    //buf = net_buf_alloc(&tx_pool, K_FOREVER);
    buf = net_buf_alloc(&tx_pool, K_NO_WAIT);

    if (buf == NULL)
    {
        printk("%s Could not allocate buffer when sending on %p\n", __FUNCTION__, stream);
        return ret;
    }

    net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

    uint8_t lc3_encoded_buffer[preset_active.qos.sdu];

    if (source_stream->lc3_encoder == NULL)
    {
        printk("%s LC3 encoder not setup, cannot encode data.\n", __FUNCTION__);
        net_buf_unref(buf);
        return ret;
    }

    ret = lc3_encode(source_stream->lc3_encoder, LC3_PCM_FORMAT_S16, &send_pcm_data[channel_index], step,
                     octets_per_frame, lc3_encoded_buffer);


    if (ret == -1)
    {
        printk("%s LC3 encoder failed - wrong parameters?: %d", __FUNCTION__, ret);
        net_buf_unref(buf);
        return ret;
    }

    net_buf_add_mem(buf, lc3_encoded_buffer, preset_active.qos.sdu);


    ret = bt_bap_stream_send(stream, buf, source_stream->seq_num++);
    if (ret < 0)
    {
        /* This will end broadcasting on this stream. */
        printk("Unable to broadcast data on %p: %d\n", stream, ret);
        net_buf_unref(buf);
        return ret;
    }

    source_stream->sent_cnt++;

    if ((source_stream->sent_cnt % 1000U) == 0U)
    {
        printk("Stream %p: Sent %u total ISO packets\n", stream, source_stream->sent_cnt);
    }

    return 0;
}

static void send_timeout_handler(void *parameter)
{
    uint32_t lvl = rt_hw_interrupt_disable();
    time_cnt++;
    rt_hw_interrupt_enable(lvl);
    //printk("timer=%d\n", rt_tick_get());
    rt_event_send(g_run_event, BAP_EVENT_SEND);
}

static void init_lc3_thread(void *arg1)
{
    const struct bt_audio_codec_cfg *codec_cfg = &preset_active.codec_cfg;
    int ret;
    printk("%s\n", __FUNCTION__);
#if BAP_SRC_USING_10MS_TIMER
    g_timer = rt_timer_create("bapsend",
                              send_timeout_handler,
                              NULL,
                              rt_tick_from_millisecond(MAX_FRAME_DURATION_US / 1000),
                              RT_TIMER_FLAG_PERIODIC);
    rt_timer_start(g_timer);
    //extern void hal_timer_start();
    //hal_timer_start(10000);
#else
    audio_register_10ms_tx_dma_callback(send_timeout_handler, NULL);
#endif
    ret = bt_audio_codec_cfg_get_freq(codec_cfg);
    if (ret > 0)
    {
        freq_hz = bt_audio_codec_cfg_freq_to_freq_hz(ret);
        printk("freq_hz=%d\n", freq_hz);
    }
    else
    {
        printk("Error: get freq error\n");
        goto Error;
    }

    ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
    if (ret > 0)
    {
        frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
        printk("frame_duration_us=%d\n", frame_duration_us);
    }
    else
    {
        printk("Error: Frame duration not set, cannot start codec.\n");
        goto Error;
    }

    octets_per_frame = bt_audio_codec_cfg_get_octets_per_frame(codec_cfg);
    frames_per_sdu = bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec_cfg, true);

    printk("octets_per_frame=%d, frames_per_sdu=%d\n", octets_per_frame, frames_per_sdu);

    if (freq_hz < 0)
    {
        printk("Error: Codec frequency not set, cannot start codec.\n");
        goto Error;
    }

    if (frame_duration_us < 0)
    {
        printk("Error: Frame duration not set, cannot start codec.\n");
        goto Error;
    }

    if (octets_per_frame < 0)
    {
        printk("Error: Octets per frame not set, cannot start codec.\n");
        goto Error;
    }


    /* Create the encoder instance. This shall complete before stream_started() is called. */
    for (size_t i = 0U; i < ARRAY_SIZE(streams); i++)
    {
        printk("Initializing lc3 encoder for stream %d\n", i);
        streams[i].lc3_encoder = lc3_setup_encoder(frame_duration_us, freq_hz, 0,
                                 &streams[i].lc3_encoder_mem);

        if (streams[i].lc3_encoder == NULL)
        {
            printk("ERROR: Failed to setup LC3 encoder - wrong parameters?\n");
        }
    }
    printk("wait start\n");
    rt_event_recv(g_run_event, BAP_EVENT_THREAD_START, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, NULL);

    uint32_t cache_read_len, readed;

    cache_read_len = lc3_frame_samples(frame_duration_us, freq_hz) * sizeof(int16_t);
    RT_ASSERT(cache_read_len <= sizeof(send_pcm_data));

    if (g_tx_channels == 2)
    {
        cache_read_len <<= 1;
    }
    printk("\r\nch=%d %d size_before_encode\n", g_tx_channels, cache_read_len);
    while (!stopping)
    {
        rt_uint32_t evt;
        rt_event_recv(g_run_event, BAP_EVENT_SEND | BAP_EVENT_THREAD_EXIT, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);

        if (evt & BAP_EVENT_SEND)
        {
            uint8_t is_retrigger = 0;
            uint32_t lvl = rt_hw_interrupt_disable();
            if (time_cnt > 1)
                is_retrigger = 1;

            if (time_cnt != 0)
                time_cnt--;

            rt_hw_interrupt_enable(lvl);
            if (is_retrigger)
                rt_event_send(g_run_event, BAP_EVENT_SEND);
        }

        if (evt & BAP_EVENT_THREAD_EXIT)
        {
            rt_event_send(g_run_event, BAP_EVENT_THREAD_EXIT);
            break;
        }

        readed = -1;
        rt_enter_critical();

        if (rt_ringbuffer_data_len(&rb_cache) < cache_read_len)
        {
            memset(send_pcm_data, 0, sizeof(send_pcm_data));
            readed = 0;
        }
        else
        {
            readed = rt_ringbuffer_get(&rb_cache, (uint8_t *)send_pcm_data, cache_read_len);
        }

        rt_exit_critical();

        if (readed == 0)
        {
            static uint8_t debug_ble = 0;
            if (debug_ble == 0)
            {
                printk("--no audio, send zero pcm\r\n");
            }
            debug_ble++;
        }
        else
        {
            RT_ASSERT(readed == cache_read_len);
        }
        for (size_t i = 0U; i < ARRAY_SIZE(streams); i++)
        {
            k_sem_take(&lc3_encoder_sem, K_FOREVER);
        }
        for (size_t i = 0U; i < ARRAY_SIZE(streams); i++)
        {
            //printk("send a stream t=%d\n", rt_tick_get());
            if (1 == ARRAY_SIZE(streams))
                ret = send_data(&streams[i], 1, (g_tx_channels == 2));
            else
                ret = send_data(&streams[i], i, (g_tx_channels == 2));
            if (ret != 0)
                k_sem_give(&lc3_encoder_sem);
        }
    }

Error:
    if (g_timer)
    {
        rt_timer_delete(g_timer);
        g_timer = NULL;
    }
    rt_event_recv(g_run_event, BAP_EVENT_THREAD_EXIT, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, NULL);
    rt_event_send(g_run_event, BAP_EVENT_THREAD_EXIT_DONE);
}



static void stream_started_cb(struct bt_bap_stream *stream)
{
    struct broadcast_source_stream *source_stream =
        CONTAINER_OF(stream, struct broadcast_source_stream, stream);

    source_stream->seq_num = 0U;
    source_stream->sent_cnt = 0U;
    k_sem_give(&sem_started);
}

static void stream_stopped_cb(struct bt_bap_stream *stream, uint8_t reason)
{
    k_sem_give(&sem_stopped);
}

static void stream_sent_cb(struct bt_bap_stream *stream)
{
    k_sem_give(&lc3_encoder_sem);
}

static struct bt_bap_stream_ops stream_ops =
{
    .started = stream_started_cb, .stopped = stream_stopped_cb, .sent = stream_sent_cb
};

static int setup_broadcast_source(struct bt_bap_broadcast_source **source)
{
    struct bt_bap_broadcast_source_stream_param
        stream_params[CONFIG_BT_BAP_BROADCAST_SRC_STREAM_COUNT];
    struct bt_bap_broadcast_source_subgroup_param
        subgroup_param[CONFIG_BT_BAP_BROADCAST_SRC_SUBGROUP_COUNT];
    struct bt_bap_broadcast_source_param create_param = {0};
    const size_t streams_per_subgroup = ARRAY_SIZE(stream_params) / ARRAY_SIZE(subgroup_param);
    uint8_t left[] = {BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
                                          BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_LEFT))
                     };
    uint8_t right[] = {BT_AUDIO_CODEC_DATA(BT_AUDIO_CODEC_CFG_CHAN_ALLOC,
                                           BT_BYTES_LIST_LE32(BT_AUDIO_LOCATION_FRONT_RIGHT))
                      };
    int err;

    for (size_t i = 0U; i < ARRAY_SIZE(subgroup_param); i++)
    {
        subgroup_param[i].params_count = streams_per_subgroup;
        subgroup_param[i].params = stream_params + i * streams_per_subgroup;
        subgroup_param[i].codec_cfg = &preset_active.codec_cfg;
    }

    for (size_t j = 0U; j < ARRAY_SIZE(stream_params); j++)
    {
        stream_params[j].stream = &streams[j].stream;
        stream_params[j].data = j == 0 ? left : right;
        stream_params[j].data_len = j == 0 ? sizeof(left) : sizeof(right);
        bt_bap_stream_cb_register(stream_params[j].stream, &stream_ops);
    }

    create_param.params_count = ARRAY_SIZE(subgroup_param);
    create_param.params = subgroup_param;
    create_param.qos = &preset_active.qos;
    create_param.encryption = strlen(CONFIG_BROADCAST_CODE) > 0;
    create_param.packing = BT_ISO_PACKING_INTERLEAVED; //BT_ISO_PACKING_SEQUENTIAL;

    if (create_param.encryption)
    {
        memcpy(create_param.broadcast_code, CONFIG_BROADCAST_CODE,
               strlen(CONFIG_BROADCAST_CODE));
    }

    printk("Creating broadcast source with %d subgroups with %d streams\n",
           ARRAY_SIZE(subgroup_param), ARRAY_SIZE(subgroup_param) * streams_per_subgroup);

    err = bt_bap_broadcast_source_create(&create_param, source);
    if (err != 0)
    {
        printk("Unable to create broadcast source: %d\n", err);
        return err;
    }

    return 0;
}

BLE_AUDIO_API int bap_broadcast_src_start()
{
    int err = 0;

    if (strlen(CONFIG_BROADCAST_CODE) > BT_AUDIO_BROADCAST_CODE_SIZE)
    {
        RT_ASSERT(0);
    }

    if (CONFIG_BT_ISO_TX_BUF_COUNT < TOTAL_BUF_NEEDED)
    {
        RT_ASSERT(0);
    }
    if (busy)
        return -1;

    busy = 1;

    {
        printk("send thread start\n");
        rt_event_send(g_run_event, BAP_EVENT_THREAD_START);
    }

    audio_server_seup_ble_bap_src(1);

    rt_thread_t encode_thread = rt_thread_create(BEL_SRC_THREAD_NAME, init_lc3_thread, NULL, LC3_ENCODER_STACK_SIZE, LC3_ENCODER_PRIORITY, RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(encode_thread);
    rt_thread_startup(encode_thread);

    do
    {
        /* Broadcast Audio Streaming Endpoint advertising data */
        NET_BUF_SIMPLE_DEFINE(ad_buf, BT_UUID_SIZE_16 + BT_AUDIO_BROADCAST_ID_SIZE);
        NET_BUF_SIMPLE_DEFINE(base_buf, 128);
        struct bt_data ext_ad[2];
        struct bt_data per_ad;
        uint32_t broadcast_id;

        /* Create a connectable advertising set */
        err = bt_le_ext_adv_create(BT_LE_EXT_ADV_CUSTOM, NULL, &adv);
        if (err != 0)
        {
            printk("Unable to create extended advertising set: %d\n", err);
            break;
        }

        /* Set periodic advertising parameters */
        err = bt_le_per_adv_set_param(adv, BT_LE_PER_ADV_DEFAULT);
        if (err)
        {
            printk("Failed to set periodic advertising parameters (err %d)\n", err);
            break;
        }

        printk("Creating broadcast source\n");
        err = setup_broadcast_source(&broadcast_source);
        if (err != 0)
        {
            printk("Unable to setup broadcast source: %d\n", err);
            break;
        }

        err = bt_bap_broadcast_source_get_id(broadcast_source, &broadcast_id);
        if (err != 0)
        {
            printk("Unable to get broadcast ID: %d\n", err);
            break;
        }

        /* Setup extended advertising data */
        net_buf_simple_add_le16(&ad_buf, BT_UUID_BROADCAST_AUDIO_VAL);
        net_buf_simple_add_le24(&ad_buf, broadcast_id);
        ext_ad[0].type = BT_DATA_SVC_DATA16;
        ext_ad[0].data_len = ad_buf.len;
        ext_ad[0].data = ad_buf.data;
        ext_ad[1] = (struct bt_data)BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
                                            sizeof(CONFIG_BT_DEVICE_NAME) - 1);
        err = bt_le_ext_adv_set_data(adv, ext_ad, 2, NULL, 0);
        if (err != 0)
        {
            printk("Failed to set extended advertising data: %d\n", err);
            break;
        }

        /* Setup periodic advertising data */
        err = bt_bap_broadcast_source_get_base(broadcast_source, &base_buf);
        if (err != 0)
        {
            printk("Failed to get encoded BASE: %d\n", err);
            break;
        }

        per_ad.type = BT_DATA_SVC_DATA16;
        per_ad.data_len = base_buf.len;
        per_ad.data = base_buf.data;
        err = bt_le_per_adv_set_data(adv, &per_ad, 1);
        if (err != 0)
        {
            printk("Failed to set periodic advertising data: %d\n", err);
            break;
        }

        /* Start extended advertising */
        err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
        if (err)
        {
            printk("Failed to start extended advertising: %d\n", err);
            break;
        }

        /* Enable Periodic Advertising */
        err = bt_le_per_adv_start(adv);
        if (err)
        {
            printk("Failed to enable periodic advertising: %d\n", err);
            break;
        }

        printk("Starting broadcast source\n");
        stopping = false;
        err = bt_bap_broadcast_source_start(broadcast_source, adv);
        if (err != 0)
        {
            printk("Unable to start broadcast source: %d\n", err);
            break;
        }

        /* Wait for all to be started */
        for (size_t i = 0U; i < ARRAY_SIZE(streams); i++)
        {
            k_sem_take(&sem_started, K_FOREVER);
        }
        printk("Broadcast source started\n");

        /* Initialize sending */
        for (size_t i = 0U; i < ARRAY_SIZE(streams); i++)
        {
            for (unsigned int j = 0U; j < BROADCAST_ENQUEUE_COUNT; j++)
            {
                stream_sent_cb(&streams[i].stream);
            }
        }
    }
    while (0);
    if (err != 0)
    {
        printk("ble audio start failed");
        rt_event_send(g_run_event, BAP_EVENT_THREAD_EXIT);
        rt_event_recv(g_run_event, BAP_EVENT_THREAD_EXIT_DONE, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, NULL);
        while (rt_thread_find(BEL_SRC_THREAD_NAME))
        {
            printk("wait thread %s exit", BEL_SRC_THREAD_NAME);
            rt_thread_mdelay(10);
        }
        k_sem_reset(&lc3_encoder_sem);
        k_sem_reset(&sem_started);
        k_sem_reset(&lc3_encoder_sem);
        busy = 0;
    }
    printk("ble audio start err=%d", err);
    return err;
}

BLE_AUDIO_API void bap_broadcast_src_stop()
{
    int err = 0;

    audio_server_seup_ble_bap_src(0);

    if (!busy)
        return;

    printk("Stopping broadcast source\n");

    rt_event_send(g_run_event, BAP_EVENT_THREAD_EXIT);

    do
    {
        err = bt_bap_broadcast_source_stop(broadcast_source);
        if (err != 0)
        {
            printk("Unable to stop broadcast source: %d\n", err);
            break;
        }

        /* Wait for all to be stopped */
        for (size_t i = 0U; i < ARRAY_SIZE(streams); i++)
        {
            k_sem_take(&sem_stopped, K_FOREVER);
        }
        printk("Broadcast source stopped\n");

        printk("Deleting broadcast source\n");
        err = bt_bap_broadcast_source_delete(broadcast_source);
        if (err != 0)
        {
            printk("Unable to delete broadcast source: %d\n", err);
            break;
        }
        printk("Broadcast source deleted\n");
        broadcast_source = NULL;

        err = bt_le_per_adv_stop(adv);
        if (err)
        {
            printk("Failed to stop periodic advertising (err %d)\n", err);
            break;
        }

        err = bt_le_ext_adv_stop(adv);
        if (err)
        {
            printk("Failed to stop extended advertising (err %d)\n", err);
            break;
        }

        err = bt_le_ext_adv_delete(adv);
        if (err)
        {
            printk("Failed to delete extended advertising (err %d)\n", err);
            break;
        }
    }
    while (0);
    rt_event_send(g_run_event, BAP_EVENT_THREAD_EXIT);
    rt_event_recv(g_run_event, BAP_EVENT_THREAD_EXIT_DONE, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, NULL);
    while (rt_thread_find(BEL_SRC_THREAD_NAME))
    {
        printk("wait %s exit", BEL_SRC_THREAD_NAME);
        rt_thread_mdelay(10);
    }
    k_sem_reset(&lc3_encoder_sem);
    k_sem_reset(&sem_started);
    k_sem_reset(&lc3_encoder_sem);
    rt_event_control(g_run_event, RT_IPC_CMD_RESET, NULL);
    busy = 0;
}

BLE_AUDIO_API void ble_src_send(uint8_t *data, uint32_t len)
{
    if (!busy)
        return;

    RT_ASSERT(SPEAKER_10MS_DMA_SIZE == len);
    uint32_t space = rt_ringbuffer_space_len(&rb_cache);
    if (rt_ringbuffer_space_len(&rb_cache) < len)
    {
        printk("ble src cache full space=%d\n", space);
    }
    rt_ringbuffer_put(&rb_cache, data, len);
}

BLE_AUDIO_API uint8_t bap_broadcast_src_is_busy(void)
{
    return busy;
}

static int bap_broadcast_src_init(void)
{
    g_tx_channels = 1;
    g_run_event = rt_event_create("bap_run", RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_run_event);
    rt_ringbuffer_init(&rb_cache, rb_cache_pool, BAP_SRC_AUDIO_CACHE_SIZE);
    return 0;
}
INIT_PRE_APP_EXPORT(bap_broadcast_src_init);


