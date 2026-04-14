/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bap_broadcast_sink_api.h"
#include <audio_server.h>
#include <liblc3/common.h>
#include "log.h"
#include <zephyr/logging/log.h>

BUILD_ASSERT(IS_ENABLED(CONFIG_SCAN_SELF) || IS_ENABLED(CONFIG_SCAN_OFFLOAD),
             "Either SCAN_SELF or SCAN_OFFLOAD must be enabled");

#define SEM_TIMEOUT                 K_SECONDS(10)
#define BROADCAST_ASSISTANT_TIMEOUT K_SECONDS(120) /* 2 minutes */

#define LOG_INTERVAL 1000U

#if defined(CONFIG_SCAN_SELF)
    #define ADV_TIMEOUT K_SECONDS(CONFIG_SCAN_DELAY)
#else /* !CONFIG_SCAN_SELF */
    #define ADV_TIMEOUT K_FOREVER
#endif /* CONFIG_SCAN_SELF */

#define PA_SYNC_INTERVAL_TO_TIMEOUT_RATIO 5 /* Set the timeout relative to interval */
#define PA_SYNC_SKIP                5
#define NAME_LEN                    sizeof(CONFIG_TARGET_BROADCAST_NAME) + 1
#define BROADCAST_DATA_ELEMENT_SIZE sizeof(int16_t)

#if defined(CONFIG_LIBLC3)
#define LC3_MAX_SAMPLE_RATE        48000U
#define LC3_MAX_FRAME_DURATION_US  10000U
#define LC3_MAX_NUM_SAMPLES_MONO   ((LC3_MAX_FRAME_DURATION_US * LC3_MAX_SAMPLE_RATE)              \
                    / USEC_PER_SEC)
#define LC3_MAX_NUM_SAMPLES_STEREO (LC3_MAX_NUM_SAMPLES_MONO * 2)

#define LC3_ENCODER_STACK_SIZE  4096
#define LC3_ENCODER_PRIORITY    5
#endif /* defined(CONFIG_LIBLC3) */

#define RUN_EVT_EXIT            (1<<0)
#define RUN_EVT_PA_SYNC_LOST    (1<<1)
#define RUN_EVT_EXIT_DONE       (1<<2)
#define BAP_THREAD_NAME         "bapsink"

static K_SEM_DEFINE(sem_broadcast_sink_stopped, 0U, 1U);
static K_SEM_DEFINE(sem_connected, 0U, 1U);
static K_SEM_DEFINE(sem_disconnected, 0U, 1U);
static K_SEM_DEFINE(sem_broadcaster_found, 0U, 1U);
static K_SEM_DEFINE(sem_pa_synced, 0U, 1U);
static K_SEM_DEFINE(sem_base_received, 0U, 1U);
static K_SEM_DEFINE(sem_syncable, 0U, 1U);
static K_SEM_DEFINE(sem_broadcast_code_received, 0U, 1U);
static K_SEM_DEFINE(sem_pa_request, 0U, 1U);
static K_SEM_DEFINE(sem_past_request, 0U, 1U);
static K_SEM_DEFINE(sem_bis_sync_requested, 0U, 1U);
static K_SEM_DEFINE(sem_stream_connected, 0U, CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT);
static K_SEM_DEFINE(sem_stream_started, 0U, CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT);
static K_SEM_DEFINE(sem_big_synced, 0U, 1U);

/* Sample assumes that we only have a single Scan Delegator receive state */
static const struct bt_bap_scan_delegator_recv_state *req_recv_state;
static struct bt_bap_broadcast_sink *broadcast_sink;
static struct bt_le_scan_recv_info broadcaster_info;
static bt_addr_le_t broadcaster_addr;
static struct bt_le_per_adv_sync *pa_sync;
static uint32_t broadcaster_broadcast_id;
static int g_ply_stream_idx;
static rt_event_t g_run_event;
static int g_exit;
static rt_tick_t last_write_tick;
static uint8_t zero[80] = {0}; // 1/12 *10ms
static struct broadcast_sink_stream
{
    struct bt_bap_stream stream;
    size_t recv_cnt;
    size_t loss_cnt;
    size_t error_cnt;
    size_t valid_cnt;
#if defined(CONFIG_LIBLC3)
    struct net_buf *in_buf;
    struct k_work_delayable lc3_decode_work;

    /* LC3 config values */
    enum bt_audio_location chan_allocation;
    uint16_t lc3_octets_per_frame;
    uint8_t lc3_frames_blocks_per_sdu;

    /* Internal lock for protecting net_buf from multiple access */
    struct k_mutex lc3_decoder_mutex;
    lc3_decoder_t lc3_decoder;
    lc3_decoder_mem_48k_t lc3_decoder_mem;
    bool need_plc;
#endif /* defined(CONFIG_LIBLC3) */
} streams[CONFIG_BT_BAP_BROADCAST_SNK_STREAM_COUNT];

static struct bt_bap_stream *streams_p[ARRAY_SIZE(streams)];
static volatile bool big_synced;
static volatile bool base_received;
static struct bt_conn *broadcast_assistant_conn;
static struct bt_le_ext_adv *ext_adv;
static volatile uint8_t stream_count;

static const struct bt_audio_codec_cap codec_cap = BT_AUDIO_CODEC_CAP_LC3(
            BT_AUDIO_CODEC_CAP_FREQ_16KHZ | BT_AUDIO_CODEC_CAP_FREQ_24KHZ,
            BT_AUDIO_CODEC_CAP_DURATION_10, BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1), 40u, 60u,
            CONFIG_MAX_CODEC_FRAMES_PER_SDU,
            (BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_MEDIA));

/* Create a mask for the maximum BIS we can sync to using the number of streams
 * we have. We add an additional 1 since the bis indexes start from 1 and not
 * 0.
 */
static const uint32_t bis_index_mask = BIT_MASK(ARRAY_SIZE(streams) + 1U);
static uint32_t requested_bis_sync;
static uint32_t bis_index_bitfield;
audio_client_t client;
static uint8_t sink_broadcast_code[BT_AUDIO_BROADCAST_CODE_SIZE];

uint64_t total_rx_iso_packet_count; /* This value is exposed to test code */

static int stop_adv(void);

#if defined(CONFIG_LIBLC3)
static K_SEM_DEFINE(lc3_decoder_sem, 0, 1);

static void decode_and_write(struct broadcast_sink_stream *stream, const void *data, const uint16_t octets_per_frame, size_t stream_index);
static void do_lc3_decode(lc3_decoder_t decoder, const void *in_data, uint8_t octets_per_frame,
                          int16_t out_data[LC3_MAX_NUM_SAMPLES_MONO]);
static void lc3_decoder_thread(void *arg1, void *arg2, void *arg3);
K_THREAD_DEFINE(decoder_tid, LC3_ENCODER_STACK_SIZE, lc3_decoder_thread,
                NULL, NULL, NULL, LC3_ENCODER_PRIORITY, 0, -1);


static int audio_callback_play(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
    }
    return 0;
}

/* Consumer thread of the decoded stream data */
static void lc3_decoder_thread(void *arg1, void *arg2, void *arg3)
{
    while (true)
    {
        k_sem_take(&lc3_decoder_sem, K_FOREVER);
        //printk("---decode a frame=%d\n", rt_tick_get());
        size_t stream_index = 0;

        for (size_t i = 0; i < ARRAY_SIZE(streams); i++)
        {
            stream_index = i;
            struct broadcast_sink_stream *stream = &streams[i];
            const uint8_t frames_blocks_per_sdu = stream->lc3_frames_blocks_per_sdu;
            const uint16_t octets_per_frame = stream->lc3_octets_per_frame;

            uint16_t frames_per_block;
            struct net_buf *buf;

            k_mutex_lock(&stream->lc3_decoder_mutex, K_FOREVER);

            if (stream->in_buf == NULL && !stream->need_plc)
            {
                k_mutex_unlock(&stream->lc3_decoder_mutex);

                continue;
            }
            if (stream->need_plc)
            {
                RT_ASSERT(stream->in_buf == NULL);
                stream->need_plc = false;
                frames_per_block = bt_audio_get_chan_count(stream->chan_allocation);
                k_mutex_unlock(&stream->lc3_decoder_mutex);
                for (uint8_t i = 0U; i < frames_blocks_per_sdu; i++)
                {
                    for (uint16_t j = 0U; j < frames_per_block; j++)
                    {
                        decode_and_write(stream, NULL, octets_per_frame, stream_index);
                    }
                }
                continue;
            }

            buf = net_buf_ref(stream->in_buf);
            net_buf_unref(stream->in_buf);
            stream->in_buf = NULL;
            k_mutex_unlock(&stream->lc3_decoder_mutex);

            frames_per_block = bt_audio_get_chan_count(stream->chan_allocation);
            LOG_DBG("buf_len=%d, octets_per_frame=%d, blocks_per_sdu=%d frames_per_block=%d\n", buf->len, octets_per_frame, frames_blocks_per_sdu, frames_per_block);

            if (buf->len !=
                    (frames_per_block * octets_per_frame * frames_blocks_per_sdu))
            {
                LOG_D("Expected %u frame blocks with %u frames of size %u, but "
                      "length is %u\n",
                      frames_blocks_per_sdu, frames_per_block, octets_per_frame,
                      buf->len);

                net_buf_unref(buf);

                continue;
            }
            /* Dummy behavior: Decode and discard data */
            LOG_DBG("--decode buf=0x%p\n", buf);

            for (uint8_t i = 0U; i < frames_blocks_per_sdu; i++)
            {
                for (uint16_t j = 0U; j < frames_per_block; j++)
                {
                    const void *data = net_buf_pull_mem(buf, octets_per_frame);
                    decode_and_write(stream, data, octets_per_frame, stream_index);
                }
            }

            LOG_DBG("--decode buf=0x%p end\n", buf);
            net_buf_unref(buf);
        }
    }

    audio_close(client);
    client = NULL;
}

/** Decode LC3 data on a stream and returns true if successful */
static void do_lc3_decode(lc3_decoder_t decoder, const void *in_data, uint8_t octets_per_frame,
                          int16_t out_data[LC3_MAX_NUM_SAMPLES_MONO])
{
    int err;

    err = lc3_decode(decoder, in_data, octets_per_frame, LC3_PCM_FORMAT_S16, out_data, 1);
    if (err == 1)
    {
        printk("  decoder performed PLC\n");
    }
    else if (err < 0)
    {
        printk("  decoder failed - wrong parameters? (err = %d)\n", err);
    }
}

static int lc3_enable(struct broadcast_sink_stream *sink_stream)
{
    size_t chan_alloc_bit_cnt;
    size_t sdu_size_required;
    int frame_duration_us;
    int freq_hz;
    int ret;

    printk("Enable: stream with codec %p\n", sink_stream->stream.codec_cfg);

    ret = bt_audio_codec_cfg_get_freq(sink_stream->stream.codec_cfg);
    if (ret > 0)
    {
        freq_hz = bt_audio_codec_cfg_freq_to_freq_hz(ret);
    }
    else
    {
        printk("Error: Codec frequency not set, cannot start codec.");
        return -1;
    }

    ret = bt_audio_codec_cfg_get_frame_dur(sink_stream->stream.codec_cfg);
    if (ret > 0)
    {
        frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
    }
    else
    {
        printk("Error: Frame duration not set, cannot start codec.");
        return ret;
    }

    ret = bt_audio_codec_cfg_get_chan_allocation(sink_stream->stream.codec_cfg,
            &sink_stream->chan_allocation, true);
    if (ret != 0)
    {
        printk("Error: Channel allocation not set, invalid configuration for LC3");
        return ret;
    }

    ret = bt_audio_codec_cfg_get_octets_per_frame(sink_stream->stream.codec_cfg);
    if (ret > 0)
    {
        sink_stream->lc3_octets_per_frame = (uint16_t)ret;
    }
    else
    {
        printk("Error: Octets per frame not set, invalid configuration for LC3");
        return ret;
    }

    ret = bt_audio_codec_cfg_get_frame_blocks_per_sdu(sink_stream->stream.codec_cfg, true);
    if (ret > 0)
    {
        sink_stream->lc3_frames_blocks_per_sdu = (uint8_t)ret;
    }
    else
    {
        printk("Error: Frame blocks per SDU not set, invalid configuration for LC3");
        return ret;
    }

    /* An SDU can consist of X frame blocks, each with Y frames (one per channel) of size Z in
     * them. The minimum SDU size required for this is X * Y * Z.
     */
    chan_alloc_bit_cnt = bt_audio_get_chan_count(sink_stream->chan_allocation);
    sdu_size_required = chan_alloc_bit_cnt * sink_stream->lc3_octets_per_frame *
                        sink_stream->lc3_frames_blocks_per_sdu;
    if (sdu_size_required < sink_stream->stream.qos->sdu)
    {
        printk("With %zu channels and %u octets per frame and %u frames per block, SDUs "
               "shall be at minimum %zu, but the stream has been configured for %u",
               chan_alloc_bit_cnt, sink_stream->lc3_octets_per_frame,
               sink_stream->lc3_frames_blocks_per_sdu, sdu_size_required,
               sink_stream->stream.qos->sdu);

        return -EINVAL;
    }

    printk("Enabling LC3 decoder with frame duration %uus, frequency %uHz and with channel "
           "allocation 0x%08X, %u octets per frame and %u frame blocks per SDU\n",
           frame_duration_us, freq_hz, sink_stream->chan_allocation,
           sink_stream->lc3_octets_per_frame, sink_stream->lc3_frames_blocks_per_sdu);

    sink_stream->lc3_decoder = lc3_setup_decoder(frame_duration_us, freq_hz, 0,
                               &sink_stream->lc3_decoder_mem);

    if (sink_stream->lc3_decoder == NULL)
    {
        printk("ERROR: Failed to setup LC3 decoder - wrong parameters?\n");
        return -1;
    }

    k_thread_start(decoder_tid);

    return 0;
}
#endif /* defined(CONFIG_LIBLC3) */

static void stream_connected_cb(struct bt_bap_stream *stream)
{
    printk("Stream %p connected\n", stream);

    k_sem_give(&sem_stream_connected);
}

static void stream_disconnected_cb(struct bt_bap_stream *stream, uint8_t reason)
{
    int err;

    printk("Stream %p disconnected with reason 0x%02X\n", stream, reason);

    err = k_sem_take(&sem_stream_connected, K_NO_WAIT);
    if (err != 0)
    {
        printk("Failed to take sem_stream_connected: %d\n", err);
    }
}

static void stream_started_cb(struct bt_bap_stream *stream)
{
    struct broadcast_sink_stream *sink_stream =
        CONTAINER_OF(stream, struct broadcast_sink_stream, stream);

    printk("Stream %p started\n", stream);

    if (!client)
    {
        int freq_hz = 48000;
        int ret = bt_audio_codec_cfg_get_freq(sink_stream->stream.codec_cfg);
        if (ret > 0)
        {
            freq_hz = bt_audio_codec_cfg_freq_to_freq_hz(ret);
        }
        else
        {
            printk("Error: Codec frequency not set, cannot start codec.");
        }
        audio_parameter_t pa = {0};
        pa.is_bap_sink = 1;
        pa.write_bits_per_sample = 16;
        pa.write_channnel_num = 1;
        pa.read_bits_per_sample = 16;
        pa.write_cache_size = BAP_BROADCAST_SINK_CACHE_SIZE;;
        pa.write_samplerate = (uint32_t)freq_hz;
        client = audio_open(AUDIO_TYPE_BT_MUSIC, AUDIO_TX, &pa, audio_callback_play, &client);
        RT_ASSERT(client);
    }

    total_rx_iso_packet_count = 0U;
    sink_stream->recv_cnt = 0U;
    sink_stream->loss_cnt = 0U;
    sink_stream->valid_cnt = 0U;
    sink_stream->error_cnt = 0U;

#if defined(CONFIG_LIBLC3)
    int err;

    if (stream->codec_cfg != 0 && stream->codec_cfg->id != BT_HCI_CODING_FORMAT_LC3)
    {
        /* No subgroups with LC3 was found */
        printk("Did not parse an LC3 codec\n");
        return;
    }
    err = lc3_enable(sink_stream);
    if (err < 0)
    {
        printk("Error: cannot enable LC3 codec: %d", err);
        return;
    }
#endif /* CONFIG_LIBLC3 */

    k_sem_give(&sem_stream_started);

    printk("lc3_enable donec\n");
    printk("sem_stream_started=%d c=%d\n", k_sem_count_get(&sem_stream_started), stream_count);
    if (k_sem_count_get(&sem_stream_started) >= stream_count)
    {
        big_synced = true;
        printk("BIG synced\n");
        k_sem_give(&sem_big_synced);
    }
}

static void stream_stopped_cb(struct bt_bap_stream *stream, uint8_t reason)
{
    int err;

    printk("Stream %p stopped with reason 0x%02X\n", stream, reason);
    if (client)
    {
        audio_close(client);
        client = NULL;
    }
    err = k_sem_take(&sem_stream_started, K_NO_WAIT);
    if (err != 0)
    {
        printk("Failed to take sem_stream_started: %d\n", err);
    }

    if (k_sem_count_get(&sem_stream_started) != stream_count)
    {
        big_synced = false;
    }
}

static void stream_recv_cb(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info,
                           struct net_buf *buf)
{
    struct broadcast_sink_stream *sink_stream =
        CONTAINER_OF(stream, struct broadcast_sink_stream, stream);

    if (info->flags & BT_ISO_FLAGS_ERROR)
    {
        sink_stream->error_cnt++;
        printk("---recv a error\n");
    }

    if (info->flags & BT_ISO_FLAGS_LOST)
    {
        sink_stream->loss_cnt++;
        printk("---recv a lost, %x\n", info->flags);
    }

    if (info->flags & BT_ISO_FLAGS_VALID)
    {
        LOG_DBG("---recv seq=%d sink=%p, stream=%p, s0=%p s1=%p\n", info->seq_num, sink_stream, stream, &streams[0].stream, &streams[1].stream);
        sink_stream->valid_cnt++;
#if defined(CONFIG_LIBLC3)
        k_mutex_lock(&sink_stream->lc3_decoder_mutex, K_FOREVER);
        sink_stream->need_plc = false;
        if (sink_stream->in_buf != NULL)
        {
            net_buf_unref(sink_stream->in_buf);
            printk("---del old=%p\n", sink_stream->in_buf);
            sink_stream->in_buf = NULL;
        }

        sink_stream->in_buf = net_buf_ref(buf);
        k_mutex_unlock(&sink_stream->lc3_decoder_mutex);
        //printk("---recv a frame=%d\n", rt_tick_get());
        k_sem_give(&lc3_decoder_sem);
#endif /* defined(CONFIG_LIBLC3) */
    }
    else
    {
#if defined(CONFIG_LIBLC3)
        k_mutex_lock(&sink_stream->lc3_decoder_mutex, K_FOREVER);
        if (sink_stream->in_buf != NULL)
        {
            net_buf_unref(sink_stream->in_buf);
            sink_stream->in_buf = NULL;
        }

        sink_stream->need_plc = true;
        k_mutex_unlock(&sink_stream->lc3_decoder_mutex);
        k_sem_give(&lc3_decoder_sem);
#endif
    }

    total_rx_iso_packet_count++;
    sink_stream->recv_cnt++;
    if ((sink_stream->recv_cnt % LOG_INTERVAL) == 0U)
    {
        printk("Stream %p: received %u total ISO packets: Valid %u | Error %u | Loss %u\n",
               &sink_stream->stream, sink_stream->recv_cnt, sink_stream->valid_cnt,
               sink_stream->error_cnt, sink_stream->loss_cnt);
    }
}

static struct bt_bap_stream_ops stream_ops =
{
    .connected = stream_connected_cb,
    .disconnected = stream_disconnected_cb,
    .started = stream_started_cb,
    .stopped = stream_stopped_cb,
    .recv = stream_recv_cb,
};

#if defined(CONFIG_TARGET_BROADCAST_CHANNEL)
struct find_valid_bis_data
{
    struct
    {
        uint8_t index;
        enum bt_audio_location chan_allocation;
    } bis[BT_ISO_BIS_INDEX_MAX];

    uint8_t cnt;
};

/**
 * This is called for each BIS in a subgroup
 *
 * It returns `false` if the current BIS contains all of the channels we are looking for,
 * or if it does not contain any and we are looking for BT_AUDIO_LOCATION_MONO_AUDIO. This stops
 * the iteration of the remaining BIS in the subgroup.
 *
 * It returns `true` if the BIS either contains none or some of the channels we are looking for.
 * If it contains some, then that is being stored in the user_data, so that the calling function
 * can check if a combination of the BIS satisfy the channel allocations we want.
 */
static bool find_valid_bis_cb(const struct bt_bap_base_subgroup_bis *bis,
                              void *user_data)
{
    struct find_valid_bis_data *data = user_data;
    struct bt_audio_codec_cfg codec_cfg = {0};
    enum bt_audio_location chan_allocation;
    int err;

    err = bt_bap_base_subgroup_bis_codec_to_codec_cfg(bis, &codec_cfg);
    if (err != 0)
    {
        printk("Could not get codec configuration for BIS: %d\n", err);
        return true;
    }

    err = bt_audio_codec_cfg_get_chan_allocation(&codec_cfg, &chan_allocation, true);
    if (err != 0)
    {
        printk("Could not find channel allocation for BIS: %d\n", err);

        if (err == -ENODATA && strlen(CONFIG_TARGET_BROADCAST_NAME) > 0U)
        {
            /* Accept no channel allocation data available
             * if TARGET_BROADCAST_NAME defined. Use current index.
             */
            data->bis[0].index = bis->index;
            data->bis[0].chan_allocation = chan_allocation;
            data->cnt = 1;

            return false;
        }
    }
    else
    {
        if ((chan_allocation & CONFIG_TARGET_BROADCAST_CHANNEL) ==
                CONFIG_TARGET_BROADCAST_CHANNEL)
        {
            /* Found single BIS with all channels we want - keep as only and stop
             * parsing
             */
            data->bis[0].index = bis->index;
            data->bis[0].chan_allocation = chan_allocation;
            data->cnt = 1;

            return false;
        }
        else if ((chan_allocation & CONFIG_TARGET_BROADCAST_CHANNEL) != 0)
        {
            /* BIS contains part of what we are looking for - Store and see if there are
             * other BIS that may fill the gaps
             */
            data->bis[data->cnt].index = bis->index;
            data->bis[data->cnt].chan_allocation = chan_allocation;
            data->cnt++;
        }
    }

    return true;
}

/**
 * This function searches all the BIS in a subgroup for a set of BIS indexes that satisfy
 * CONFIG_TARGET_BROADCAST_CHANNEL
 *
 * Returns `true` if the right channels were found, otherwise `false`.
 */
static bool find_valid_bis_in_subgroup_bis(const struct bt_bap_base_subgroup *subgroup,
        uint32_t *bis_indexes)
{
    struct find_valid_bis_data data = {0};
    int err;

    err = bt_bap_base_subgroup_foreach_bis(subgroup, find_valid_bis_cb, &data);
    if (err == -ECANCELED)
    {
        /* We found what we are looking for in a single BIS */

        *bis_indexes = BIT(data.bis[0].index);

        return true;
    }
    else if (err == 0)
    {
        /* We are finished parsing all BIS - Try to find a combination that satisfy our
         * channel allocation. For simplicity this is using a greedy approach, rather than
         * an optimal one.
         */
        enum bt_audio_location chan_allocation = BT_AUDIO_LOCATION_MONO_AUDIO;
        *bis_indexes = 0;

        for (uint8_t i = 0U; i < data.cnt; i++)
        {
            chan_allocation |= data.bis[i].chan_allocation;
            *bis_indexes |= BIT(data.bis[i].index);

            if ((chan_allocation & CONFIG_TARGET_BROADCAST_CHANNEL) ==
                    CONFIG_TARGET_BROADCAST_CHANNEL)
            {
                return true;
            }
        }
    }

    /* Some error occurred or we did not find expected channel allocation */
    return false;
}

/**
 * Called for each subgroup in the BASE. Will populate the 32-bit bitfield of BIS indexes if the
 * subgroup contains it.
 *
 * The channel allocation may
 *  - Not exist at all, implicitly meaning BT_AUDIO_LOCATION_MONO_AUDIO
 *  - Exist only in the subgroup codec configuration
 *  - Exist only in the BIS codec configuration
 *  - Exist in both the subgroup and BIS codec configuration, in which case, the BIS codec
 *    configuration overwrites the subgroup values
 *
 * This function returns `true` if the subgroup does not support the channels in
 * CONFIG_TARGET_BROADCAST_CHANNEL which makes it iterate over the next subgroup, and returns
 * `false` if this subgroup satisfies our CONFIG_TARGET_BROADCAST_CHANNEL.
 */
static bool find_valid_bis_in_subgroup_cb(const struct bt_bap_base_subgroup *subgroup,
        void *user_data)
{
    enum bt_audio_location chan_allocation;
    struct bt_audio_codec_cfg codec_cfg;
    uint32_t *bis_indexes = user_data;
    int err;

    /* We only want indexes from a single subgroup, so reset between each of them*/
    *bis_indexes = 0U;

    err = bt_bap_base_subgroup_codec_to_codec_cfg(subgroup, &codec_cfg);
    if (err != 0)
    {
        printk("Could not get codec configuration: %d\n", err);

        return true;
    }

    err = bt_audio_codec_cfg_get_chan_allocation(&codec_cfg, &chan_allocation, false);
    if (err != 0)
    {
        printk("Could not find subgroup channel allocation: %d - Looking in the BISes\n",
               err);

        /* Find chan alloc in BIS */
        if (find_valid_bis_in_subgroup_bis(subgroup, bis_indexes))
        {
            /* Found BISes with correct channel allocation */
            return false;
        }
    }
    else
    {
        /* If the subgroup contains a single channel, then we just grab the first BIS index
         */
        if (bt_audio_get_chan_count(chan_allocation) == 1 &&
                chan_allocation == CONFIG_TARGET_BROADCAST_CHANNEL)
        {
            uint32_t subgroup_bis_indexes;

            /* Set bis_indexes to the first bit set */
            err = bt_bap_base_subgroup_get_bis_indexes(subgroup, &subgroup_bis_indexes);
            if (err != 0)
            {
                /* Should never happen as that would indicate an invalid
                 * subgroup If it does, we just parse the next subgroup
                 */
                return true;
            }

            /* We found the BIS index we want, stop parsing*/
            *bis_indexes = BIT(find_lsb_set(subgroup_bis_indexes) - 1);

            return false;
        }
        else if ((chan_allocation & CONFIG_TARGET_BROADCAST_CHANNEL) ==
                 CONFIG_TARGET_BROADCAST_CHANNEL)
        {
            /* The subgroup contains all channels we are looking for/
             * We continue searching each BIS to get the minimal amount of BIS that
             * satisfy CONFIG_TARGET_BROADCAST_CHANNEL.
             */

            if (find_valid_bis_in_subgroup_bis(subgroup, bis_indexes))
            {
                /* Found BISes with correct channel allocation */
                return false;
            }
        }
    }

    return true;
}

/**
 * This function gets a 32-bit bitfield of BIS indexes that cover the channel allocation values in
 * CONFIG_TARGET_BROADCAST_CHANNEL.
 */
static int base_get_valid_bis_indexes(const struct bt_bap_base *base, uint32_t *bis_indexes)
{
    int err;

    err = bt_bap_base_foreach_subgroup(base, find_valid_bis_in_subgroup_cb, bis_indexes);
    if (err != -ECANCELED)
    {
        printk("Failed to parse subgroups: %d\n", err);
        return err != 0 ? err : -ENOENT;
    }

    return 0;
}
#endif /* CONFIG_TARGET_BROADCAST_CHANNEL */

static void base_recv_cb(struct bt_bap_broadcast_sink *sink, const struct bt_bap_base *base,
                         size_t base_size)
{
    uint32_t base_bis_index_bitfield = 0U;
    int err;

    if (base_received)
    {
        return;
    }

    printk("Received BASE with %d subgroups from broadcast sink %p\n",
           bt_bap_base_get_subgroup_count(base), sink);

#if defined(CONFIG_TARGET_BROADCAST_CHANNEL)
    err = base_get_valid_bis_indexes(base, &base_bis_index_bitfield);
    if (err != 0)
    {
        printk("Failed to find a valid BIS\n");
        return;
    }
#else
    err = bt_bap_base_get_bis_indexes(base, &base_bis_index_bitfield);
    if (err != 0)
    {
        printk("Failed to BIS indexes: %d\n", err);
        return;
    }
#endif /* CONFIG_TARGET_BROADCAST_CHANNEL */

    bis_index_bitfield = base_bis_index_bitfield & bis_index_mask;

    if (broadcast_assistant_conn == NULL)
    {
        /* No broadcast assistant requesting anything */
        requested_bis_sync = BT_BAP_BIS_SYNC_NO_PREF;
        k_sem_give(&sem_bis_sync_requested);
    }

    base_received = true;
    k_sem_give(&sem_base_received);
}

static void syncable_cb(struct bt_bap_broadcast_sink *sink, const struct bt_iso_biginfo *biginfo)
{
    printk("Broadcast sink (%p) is syncable, BIG %s\n", (void *)sink,
           biginfo->encryption ? "encrypted" : "not encrypted");

    k_sem_give(&sem_syncable);

    if (!biginfo->encryption)
    {
        k_sem_give(&sem_broadcast_code_received);
    }
}

static struct bt_bap_broadcast_sink_cb broadcast_sink_cbs =
{
    .base_recv = base_recv_cb,
    .syncable = syncable_cb,
};

static void pa_timer_handler(struct k_work *work)
{
    if (req_recv_state != NULL)
    {
        enum bt_bap_pa_state pa_state;

        if (req_recv_state->pa_sync_state == BT_BAP_PA_STATE_INFO_REQ)
        {
            pa_state = BT_BAP_PA_STATE_NO_PAST;
        }
        else
        {
            pa_state = BT_BAP_PA_STATE_FAILED;
        }

        bt_bap_scan_delegator_set_pa_state(req_recv_state->src_id,
                                           pa_state);
    }

    printk("PA timeout\n");
}

static K_WORK_DELAYABLE_DEFINE(pa_timer, pa_timer_handler);

static uint16_t interval_to_sync_timeout(uint16_t pa_interval)
{
    uint16_t pa_timeout;

    if (pa_interval == BT_BAP_PA_INTERVAL_UNKNOWN)
    {
        /* Use maximum value to maximize chance of success */
        pa_timeout = BT_GAP_PER_ADV_MAX_TIMEOUT;
    }
    else
    {
        uint32_t interval_ms;
        uint32_t timeout;

        /* Add retries and convert to unit in 10's of ms */
        interval_ms = BT_GAP_PER_ADV_INTERVAL_TO_MS(pa_interval);
        timeout = (interval_ms * PA_SYNC_INTERVAL_TO_TIMEOUT_RATIO) / 10;

        /* Enforce restraints */
        pa_timeout = CLAMP(timeout, BT_GAP_PER_ADV_MIN_TIMEOUT, BT_GAP_PER_ADV_MAX_TIMEOUT);
    }

    return pa_timeout;
}

static int pa_sync_past(struct bt_conn *conn, uint16_t pa_interval)
{
    struct bt_le_per_adv_sync_transfer_param param = { 0 };
    int err;

    param.skip = PA_SYNC_SKIP;
    param.timeout = interval_to_sync_timeout(pa_interval);

    err = bt_le_per_adv_sync_transfer_subscribe(conn, &param);
    if (err != 0)
    {
        printk("Could not do PAST subscribe: %d\n", err);
    }
    else
    {
        printk("Syncing with PAST\n");
        (void)k_work_reschedule(&pa_timer, K_MSEC(param.timeout * 10));
    }

    return err;
}

static void recv_state_updated_cb(struct bt_conn *conn,
                                  const struct bt_bap_scan_delegator_recv_state *recv_state)
{
    printk("Receive state updated, pa sync state: %u\n", recv_state->pa_sync_state);

    for (uint8_t i = 0; i < recv_state->num_subgroups; i++)
    {
        printk("subgroup %d bis_sync: 0x%08x\n", i, recv_state->subgroups[i].bis_sync);
    }

    req_recv_state = recv_state;
}

static int pa_sync_req_cb(struct bt_conn *conn,
                          const struct bt_bap_scan_delegator_recv_state *recv_state,
                          bool past_avail, uint16_t pa_interval)
{

    printk("Received request to sync to PA (PAST %savailble): %u\n", past_avail ? "" : "not ",
           recv_state->pa_sync_state);

    req_recv_state = recv_state;

    if (recv_state->pa_sync_state == BT_BAP_PA_STATE_SYNCED ||
            recv_state->pa_sync_state == BT_BAP_PA_STATE_INFO_REQ)
    {
        /* Already syncing */
        /* TODO: Terminate existing sync and then sync to new?*/
        return -1;
    }

    if (IS_ENABLED(CONFIG_BT_PER_ADV_SYNC_TRANSFER_RECEIVER) && past_avail)
    {
        int err;

        err = pa_sync_past(conn, pa_interval);
        if (err != 0)
        {
            printk("Failed to subscribe to PAST: %d\n", err);

            return err;
        }

        k_sem_give(&sem_past_request);

        err = bt_bap_scan_delegator_set_pa_state(recv_state->src_id,
                BT_BAP_PA_STATE_INFO_REQ);
        if (err != 0)
        {
            printk("Failed to set PA state to BT_BAP_PA_STATE_INFO_REQ: %d\n", err);

            return err;
        }
    }

    k_sem_give(&sem_pa_request);

    return 0;
}

static int pa_sync_term_req_cb(struct bt_conn *conn,
                               const struct bt_bap_scan_delegator_recv_state *recv_state)
{
    int err;

    printk("PA sync termination req, pa sync state: %u\n", recv_state->pa_sync_state);

    for (uint8_t i = 0; i < recv_state->num_subgroups; i++)
    {
        printk("subgroup %d bis_sync: 0x%08x\n", i, recv_state->subgroups[i].bis_sync);
    }

    req_recv_state = recv_state;

    printk("Delete periodic advertising sync\n");
    err = bt_le_per_adv_sync_delete(pa_sync);
    if (err != 0)
    {
        printk("Could not delete per adv sync: %d\n", err);

        return err;
    }

    return 0;
}

static void broadcast_code_cb(struct bt_conn *conn,
                              const struct bt_bap_scan_delegator_recv_state *recv_state,
                              const uint8_t broadcast_code[BT_AUDIO_BROADCAST_CODE_SIZE])
{
    printk("Broadcast code received for %p\n", recv_state);

    req_recv_state = recv_state;

    (void)memcpy(sink_broadcast_code, broadcast_code, BT_AUDIO_BROADCAST_CODE_SIZE);

    k_sem_give(&sem_broadcast_code_received);
}

static int bis_sync_req_cb(struct bt_conn *conn,
                           const struct bt_bap_scan_delegator_recv_state *recv_state,
                           const uint32_t bis_sync_req[CONFIG_BT_BAP_BASS_MAX_SUBGROUPS])
{
    printk("BIS sync request received for %p: 0x%08x->0x%08x, broadcast id: 0x%06x, (%s)\n",
           recv_state, requested_bis_sync, bis_sync_req[0], recv_state->broadcast_id,
           big_synced ? "BIG synced" : "BIG not synced");

    /* We only care about a single subgroup in this sample */
    if (big_synced && requested_bis_sync != bis_sync_req[0])
    {
        /* If the BIS sync request is received while we are already
         * synced, it means that the requested BIS sync has changed.
         */
        int err;

        /* The stream stopped callback will be called as part of this,
         * and we do not need to wait for any events from the
         * controller. Thus, when this returns, the `big_synced`
         * is back to false.
         */
        err = bt_bap_broadcast_sink_stop(broadcast_sink);
        if (err != 0)
        {
            printk("Failed to stop Broadcast Sink: %d\n", err);

            return err;
        }

        k_sem_give(&sem_broadcast_sink_stopped);
    }

    requested_bis_sync = bis_sync_req[0];
    broadcaster_broadcast_id = recv_state->broadcast_id;
    if (bis_sync_req[0] != 0)
    {
        k_sem_give(&sem_bis_sync_requested);
    }

    return 0;
}

static struct bt_bap_scan_delegator_cb scan_delegator_cbs =
{
    .recv_state_updated = recv_state_updated_cb,
    .pa_sync_req = pa_sync_req_cb,
    .pa_sync_term_req = pa_sync_term_req_cb,
    .broadcast_code = broadcast_code_cb,
    .bis_sync_req = bis_sync_req_cb,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err != 0U)
    {
        printk("Failed to connect to %s %u %s\n", addr, err, bt_hci_err_to_str(err));

        broadcast_assistant_conn = NULL;
        return;
    }
    printk("Connected: %s\n", addr);
    broadcast_assistant_conn = bt_conn_ref(conn);

    k_sem_give(&sem_connected);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (conn != broadcast_assistant_conn)
    {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Disconnected: %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

    bt_conn_unref(broadcast_assistant_conn);
    broadcast_assistant_conn = NULL;

    k_sem_give(&sem_disconnected);
}

BT_CONN_CB_DEFINE(conn_callbacks) =
{
    .connected = connected,
    .disconnected = disconnected,
};

static struct bt_pacs_cap cap =
{
    .codec_cap = &codec_cap,
};

static bool scan_check_and_sync_broadcast(struct bt_data *data, void *user_data)
{
    const struct bt_le_scan_recv_info *info = user_data;
    char le_addr[BT_ADDR_LE_STR_LEN];
    struct bt_uuid_16 adv_uuid;
    uint32_t broadcast_id;

    if (data->type != BT_DATA_SVC_DATA16)
    {
        return true;
    }

    if (data->data_len < BT_UUID_SIZE_16 + BT_AUDIO_BROADCAST_ID_SIZE)
    {
        return true;
    }

    if (!bt_uuid_create(&adv_uuid.uuid, data->data, BT_UUID_SIZE_16))
    {
        return true;
    }

    if (bt_uuid_cmp(&adv_uuid.uuid, BT_UUID_BROADCAST_AUDIO))
    {
        return true;
    }

    broadcast_id = sys_get_le24(data->data + BT_UUID_SIZE_16);

    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

    printk("Found broadcaster with ID 0x%06X and addr %s and sid 0x%02X\n", broadcast_id,
           le_addr, info->sid);

    if (broadcast_assistant_conn == NULL /* Not requested by Broadcast Assistant */ ||
            (req_recv_state != NULL && bt_addr_le_eq(info->addr, &req_recv_state->addr) &&
             info->sid == req_recv_state->adv_sid &&
             broadcast_id == req_recv_state->broadcast_id))
    {

        /* Store info for PA sync parameters */
        memcpy(&broadcaster_info, info, sizeof(broadcaster_info));
        bt_addr_le_copy(&broadcaster_addr, info->addr);
        broadcaster_broadcast_id = broadcast_id;
        printk("broadcaster_broadcast_id = 0x%06X\n", broadcaster_broadcast_id);
        k_sem_give(&sem_broadcaster_found);
    }

    /* Stop parsing */
    return false;
}

static bool is_substring(const char *substr, const char *str)
{
    const size_t str_len = strlen(str);
    const size_t sub_str_len = strlen(substr);

    if (sub_str_len > str_len)
    {
        return false;
    }

    for (size_t pos = 0; pos < str_len; pos++)
    {
        if (pos + sub_str_len > str_len)
        {
            return false;
        }

        if (strncmp(substr, &str[pos], sub_str_len) == 0)
        {
            return true;
        }
    }

    return false;
}

static bool data_cb(struct bt_data *data, void *user_data)
{
    bool *device_found = user_data;
    char name[NAME_LEN] = {0};

    switch (data->type)
    {
    case BT_DATA_NAME_SHORTENED:
    case BT_DATA_NAME_COMPLETE:
    case BT_DATA_BROADCAST_NAME:
        memcpy(name, data->data, MIN(data->data_len, NAME_LEN - 1));

        if (is_substring(CONFIG_TARGET_BROADCAST_NAME, name))
        {
            /* Device found */
            *device_found = true;
            return false;
        }
        return true;
    default:
        return true;
    }
}

static void broadcast_scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *ad)
{
    if (info->interval != 0U)
    {
        /* call to bt_data_parse consumes netbufs so shallow clone for verbose output */

        /* If req_recv_state is not NULL then we have been requested by a broadcast
         * assistant to sync to a specific broadcast source. In that case we do not apply
         * our own broadcast name filter.
         */
        if (req_recv_state == NULL && strlen(CONFIG_TARGET_BROADCAST_NAME) > 0U)
        {
            bool device_found = false;
            struct net_buf_simple buf_copy;

            net_buf_simple_clone(ad, &buf_copy);
            bt_data_parse(&buf_copy, data_cb, &device_found);

            if (!device_found)
            {
                return;
            }
        }
        bt_data_parse(ad, scan_check_and_sync_broadcast, (void *)info);
    }
}

static struct bt_le_scan_cb bap_scan_cb =
{
    .recv = broadcast_scan_recv,
};

static void bap_pa_sync_synced_cb(struct bt_le_per_adv_sync *sync,
                                  struct bt_le_per_adv_sync_synced_info *info)
{
    if (sync == pa_sync ||
            (req_recv_state != NULL && bt_addr_le_eq(info->addr, &req_recv_state->addr) &&
             info->sid == req_recv_state->adv_sid))
    {
        printk("PA sync %p synced for broadcast sink with broadcast ID 0x%06X\n", sync,
               broadcaster_broadcast_id);

        if (pa_sync == NULL)
        {
            pa_sync = sync;
        }

        k_work_cancel_delayable(&pa_timer);
        k_sem_give(&sem_pa_synced);
    }
}

static void bap_pa_sync_terminated_cb(struct bt_le_per_adv_sync *sync,
                                      const struct bt_le_per_adv_sync_term_info *info)
{
    if (sync == pa_sync)
    {
        printk("PA sync %p lost with reason 0x%02X\n", sync, info->reason);
        pa_sync = NULL;

        if (g_run_event)
            rt_event_send(g_run_event, RUN_EVT_PA_SYNC_LOST);

        if (info->reason != BT_HCI_ERR_LOCALHOST_TERM_CONN && req_recv_state != NULL)
        {
            int err;

            if (k_sem_count_get(&sem_stream_connected) > 0)
            {
                err = bt_bap_broadcast_sink_stop(broadcast_sink);
                if (err != 0)
                {
                    printk("Failed to stop Broadcast Sink: %d\n", err);

                    return;
                }
            }

            err = bt_bap_scan_delegator_rem_src(req_recv_state->src_id);
            if (err != 0)
            {
                printk("Failed to remove source: %d\n", err);

                return;
            }

            k_sem_give(&sem_broadcast_sink_stopped);
        }
    }
}

static struct bt_le_per_adv_sync_cb bap_pa_sync_cb =
{
    .synced = bap_pa_sync_synced_cb,
    .term = bap_pa_sync_terminated_cb,
};

static int reset(void)
{
    int err;

    printk("Reset\n");

    bis_index_bitfield = 0U;
    requested_bis_sync = 0U;
    req_recv_state = NULL;
    big_synced = false;
    base_received = false;
    stream_count = 0U;
    (void)memset(sink_broadcast_code, 0, sizeof(sink_broadcast_code));
    (void)memset(&broadcaster_info, 0, sizeof(broadcaster_info));
    (void)memset(&broadcaster_addr, 0, sizeof(broadcaster_addr));
    broadcaster_broadcast_id = BT_BAP_INVALID_BROADCAST_ID;

    if (broadcast_sink != NULL)
    {
        err = bt_bap_broadcast_sink_delete(broadcast_sink);
        if (err)
        {
            printk("Deleting broadcast sink failed (err %d)\n", err);

            return err;
        }

        broadcast_sink = NULL;
    }

    if (pa_sync != NULL)
    {
        bt_le_per_adv_sync_delete(pa_sync);
        if (err)
        {
            printk("Deleting PA sync failed (err %d)\n", err);

            return err;
        }

        pa_sync = NULL;
    }

    k_sem_reset(&sem_broadcaster_found);
    k_sem_reset(&sem_pa_synced);
    k_sem_reset(&sem_base_received);
    k_sem_reset(&sem_syncable);
    k_sem_reset(&sem_broadcast_code_received);
    k_sem_reset(&sem_bis_sync_requested);
    k_sem_reset(&sem_stream_connected);
    k_sem_reset(&sem_stream_started);
    k_sem_reset(&sem_broadcast_sink_stopped);

    return 0;
}

static int start_adv(void)
{
    const struct bt_data ad[] =
    {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                      BT_UUID_16_ENCODE(BT_UUID_BASS_VAL),
                      BT_UUID_16_ENCODE(BT_UUID_PACS_VAL)),
        BT_DATA_BYTES(BT_DATA_SVC_DATA16, BT_UUID_16_ENCODE(BT_UUID_BASS_VAL)),
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
                sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };
    int err;

    /* Create a connectable advertising set */
    err = bt_le_ext_adv_create(BT_LE_EXT_ADV_CONN, NULL, &ext_adv);
    if (err != 0)
    {
        printk("Failed to create advertising set (err %d)\n", err);

        return err;
    }

    err = bt_le_ext_adv_set_data(ext_adv, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err != 0)
    {
        printk("Failed to set advertising data (err %d)\n", err);

        return err;
    }

    err = bt_le_ext_adv_start(ext_adv, BT_LE_EXT_ADV_START_DEFAULT);
    if (err != 0)
    {
        printk("Failed to start advertising set (err %d)\n", err);

        return err;
    }

    return 0;
}

static int stop_adv(void)
{
    int err;

    err = bt_le_ext_adv_stop(ext_adv);
    if (err != 0)
    {
        printk("Failed to stop advertising set (err %d)\n", err);

        return err;
    }

    err = bt_le_ext_adv_delete(ext_adv);
    if (err != 0)
    {
        printk("Failed to delete advertising set (err %d)\n", err);

        return err;
    }

    ext_adv = NULL;

    return 0;
}

static int pa_sync_create(void)
{
    struct bt_le_per_adv_sync_param create_params = {0};

    bt_addr_le_copy(&create_params.addr, &broadcaster_addr);
#if 0 // BT 5.3 Only
    create_params.options = BT_LE_PER_ADV_SYNC_OPT_FILTER_DUPLICATE;
#endif
    create_params.sid = broadcaster_info.sid;
    create_params.skip = PA_SYNC_SKIP;
    create_params.timeout = interval_to_sync_timeout(broadcaster_info.interval);

    return bt_le_per_adv_sync_create(&create_params, &pa_sync);
}

static void sink_thread_entry(void *p)
{
    int err;

    while (!g_exit)
    {
        uint32_t sync_bitfield;

        if (g_exit)
        {
            goto wait_event;
        }

        err = reset();
        if (err != 0)
        {
            printk("Resetting failed: %d - Aborting\n", err);
            rt_thread_mdelay(1000);
            continue;
        }

        if (IS_ENABLED(CONFIG_SCAN_OFFLOAD))
        {
            if (broadcast_assistant_conn == NULL)
            {
                k_sem_reset(&sem_connected);

                printk("Starting advertising\n");
                /* Stop advertising before starting if needed */
                if (ext_adv != NULL)
                {
                    err = stop_adv();
                    if (err != 0)
                    {
                        printk("Unable to stop advertising: %d\n", err);

                        break;
                    }
                }
                err = start_adv();
                if (err != 0)
                {
                    printk("Unable to start advertising connectable: %d\n",
                           err);

                    break;
                }

                printk("Waiting for Broadcast Assistant\n");
                err = k_sem_take(&sem_connected, ADV_TIMEOUT);
                if (err != 0)
                {
                    printk("No Broadcast Assistant connected\n");

                    err = stop_adv();
                    if (err != 0)
                    {
                        printk("Unable to stop advertising: %d\n", err);
                        break;
                    }
                }
            }

            if (broadcast_assistant_conn != NULL)
            {
                k_sem_reset(&sem_pa_request);
                k_sem_reset(&sem_past_request);
                k_sem_reset(&sem_disconnected);

                /* Wait for the PA request to determine if we
                 * should start scanning, or wait for PAST
                 */
                printk("Waiting for PA sync request\n");
                err = k_sem_take(&sem_pa_request,
                                 BROADCAST_ASSISTANT_TIMEOUT);
                if (err != 0)
                {
                    printk("sem_pa_request timed out, resetting\n");
                    continue;
                }

                if (k_sem_take(&sem_past_request, K_NO_WAIT) == 0)
                {
                    goto wait_for_pa_sync;
                } /* else continue with scanning below */
            }
        }

        if (strlen(CONFIG_TARGET_BROADCAST_NAME) > 0U)
        {
            printk("Scanning for broadcast sources containing "
                   "`" CONFIG_TARGET_BROADCAST_NAME "`\n");
        }
        else
        {
            printk("Scanning for broadcast sources\n");
        }

        err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);
        if (err != 0 && err != -EALREADY)
        {
            printk("Unable to start scan for broadcast sources: %d\n",
                   err);
            rt_thread_mdelay(1000);
            continue;
        }

        printk("Waiting for Broadcaster\n");
        err = k_sem_take(&sem_broadcaster_found, SEM_TIMEOUT);
        if (err != 0)
        {
            printk("sem_broadcaster_found timed out, resetting\n");
            rt_thread_mdelay(1000);
            continue;
        }

        err = bt_le_scan_stop();
        if (err != 0)
        {
            printk("bt_le_scan_stop failed with %d, resetting\n", err);
            rt_thread_mdelay(1000);
            continue;
        }

        printk("Attempting to PA sync to the broadcaster with id 0x%06X\n",
               broadcaster_broadcast_id);
        err = pa_sync_create();
        if (err != 0)
        {
            printk("Could not create Broadcast PA sync: %d, resetting\n", err);
            rt_thread_mdelay(1000);
            continue;
        }

wait_for_pa_sync:
        printk("Waiting for PA synced\n");
        err = k_sem_take(&sem_pa_synced, SEM_TIMEOUT);
        if (err != 0)
        {
            printk("sem_pa_synced timed out, resetting\n");
            rt_thread_mdelay(1000);
            continue;
        }

        printk("Broadcast source PA synced, creating Broadcast Sink\n");
        err = bt_bap_broadcast_sink_create(pa_sync, broadcaster_broadcast_id,
                                           &broadcast_sink);
        if (err != 0)
        {
            printk("Failed to create broadcast sink: %d\n", err);
            rt_thread_mdelay(1000);
            continue;
        }

        printk("Broadcast Sink created, waiting for BASE\n");
        err = k_sem_take(&sem_base_received, SEM_TIMEOUT);
        if (err != 0)
        {
            printk("sem_base_received timed out, resetting\n");
            rt_thread_mdelay(1000);
            continue;
        }

        printk("BASE received, waiting for syncable\n");
        err = k_sem_take(&sem_syncable, SEM_TIMEOUT);
        if (err != 0)
        {
            printk("sem_syncable timed out, resetting\n");
            rt_thread_mdelay(1000);
            continue;
        }

        /* sem_broadcast_code_received is also given if the
         * broadcast is not encrypted
         */
        printk("Waiting for broadcast code\n");
        err = k_sem_take(&sem_broadcast_code_received, SEM_TIMEOUT);
        if (err != 0)
        {
            printk("sem_broadcast_code_received timed out, resetting\n");
            rt_thread_mdelay(1000);
            continue;
        }

        printk("Waiting for BIS sync request\n");
        err = k_sem_take(&sem_bis_sync_requested, SEM_TIMEOUT);
        if (err != 0)
        {
            printk("sem_bis_sync_requested timed out, resetting\n");
            rt_thread_mdelay(1000);
            continue;
        }

        sync_bitfield = bis_index_bitfield & requested_bis_sync;
        stream_count = 0;
        for (int i = 0; i < BT_ISO_MAX_GROUP_ISO_COUNT; i++)
        {
            if ((sync_bitfield & BIT(i)) != 0)
            {
                stream_count++;
            }
        }

        printk("Syncing to broadcast with bitfield: 0x%08x = 0x%08x (bis_index) & 0x%08x "
               "(req_bis_sync), stream_count = %u\n",
               sync_bitfield, bis_index_bitfield, requested_bis_sync, stream_count);

        err = bt_bap_broadcast_sink_sync(broadcast_sink, sync_bitfield, streams_p,
                                         sink_broadcast_code);
        if (err != 0)
        {
            printk("Unable to sync to broadcast source: %d\n", err);
            rt_thread_mdelay(1000);
            continue;
        }

        printk("Waiting for stream(s) started\n");
        err = k_sem_take(&sem_big_synced, SEM_TIMEOUT);
        if (err != 0)
        {
            printk("sem_big_synced timed out, resetting\n");
            rt_thread_mdelay(1000);
            continue;
        }

        printk("Waiting for PA disconnected\n");
        rt_uint32_t evt = 0;

wait_event:
        rt_event_recv(g_run_event, RUN_EVT_PA_SYNC_LOST | RUN_EVT_EXIT, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        if (evt & RUN_EVT_PA_SYNC_LOST)
        {
            printk("Waiting for sink to stop\n");
            err = k_sem_take(&sem_broadcast_sink_stopped, K_SECONDS(5));
            if (err != 0)
            {
                printk("sem_broadcast_sink_stopped timed out, resetting\n");
                continue;
            }
        }

        if (evt & RUN_EVT_EXIT)
        {
            err = bt_bap_broadcast_sink_stop(broadcast_sink);
            if (err != 0)
            {
                printk("Failed to stop Broadcast Sink: %d", err);
            }
            break;
        }
    }
    reset();
    rt_event_send(g_run_event, RUN_EVT_EXIT_DONE);
}

int bap_broadcast_sink_init(void)
{
    int err;
    err = bt_pacs_cap_register(BT_AUDIO_DIR_SINK, &cap);
    if (err)
    {
        printk("Capability register failed (err %d)\n", err);
        return err;
    }

    err = bt_bap_scan_delegator_register(&scan_delegator_cbs);
    if (err)
    {
        printk("Scan delegator register failed (err %d)\n", err);
        return err;
    }

    bt_bap_broadcast_sink_register_cb(&broadcast_sink_cbs);
    bt_le_per_adv_sync_cb_register(&bap_pa_sync_cb);
    bt_le_scan_cb_register(&bap_scan_cb);

    for (size_t i = 0U; i < ARRAY_SIZE(streams); i++)
    {
        streams[i].stream.ops = &stream_ops;
    }
    return 0;
}

int bap_broadcast_sink_start(int stream_idx)
{
    int err;
    if (!IS_ENABLED(CONFIG_SCAN_SELF) && !IS_ENABLED(CONFIG_SCAN_OFFLOAD))
    {
        RT_ASSERT(0);
    }
    if (g_run_event)
    {
        return -1;
    }

    g_ply_stream_idx = stream_idx;

    for (size_t i = 0U; i < ARRAY_SIZE(streams_p); i++)
    {
        streams_p[i] = &streams[i].stream;
#if defined(CONFIG_LIBLC3)
        k_mutex_init(&streams[i].lc3_decoder_mutex);
#endif /* defined(CONFIG_LIBLC3) */
    }

    g_run_event = rt_event_create("bapsink", RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_run_event);
    rt_thread_t tid = rt_thread_create(BAP_THREAD_NAME, sink_thread_entry, NULL, 2048, RT_THREAD_PRIORITY_LOW, 10);
    RT_ASSERT(tid);
    rt_thread_startup(tid);
    return 0;
}

int bap_broadcast_sink_stop(void)
{
    if (!g_run_event)
    {
        return -1;
    }
    g_exit = 1;

    k_sem_give(&sem_broadcaster_found);
    k_sem_give(&sem_pa_synced);
    k_sem_give(&sem_base_received);
    k_sem_give(&sem_syncable);
    k_sem_give(&sem_broadcast_code_received);
    k_sem_give(&sem_bis_sync_requested);
    k_sem_give(&sem_stream_connected);
    k_sem_give(&sem_stream_started);
    k_sem_give(&sem_broadcast_sink_stopped);

    rt_event_send(g_run_event, RUN_EVT_EXIT);
    rt_event_recv(g_run_event, RUN_EVT_EXIT_DONE, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, NULL);
    rt_event_delete(g_run_event);

    while (rt_thread_find(BAP_THREAD_NAME))
    {
        printk("wait thread %s exit\n", BAP_THREAD_NAME);
        rt_thread_mdelay(10);
    }

    g_run_event = NULL;
    g_exit = 0;
    printk("%s exit\n", __FUNCTION__);
    return 0;
}

uint8_t bap_broadcast_sink_is_busy(void)
{
    return (g_run_event != NULL);
}
/*
  decode a frame and write to audio device
*/
static void decode_and_write(struct broadcast_sink_stream *stream, const void *data, const uint16_t octets_per_frame, size_t stream_index)
{
    static int16_t lc3_audio_buf[LC3_MAX_NUM_SAMPLES_MONO];
    do_lc3_decode(stream->lc3_decoder, data, octets_per_frame,
                  lc3_audio_buf);

    if (stream_index == g_ply_stream_idx)
    {
        uint32_t num_of_sample = LC3_NS(stream->lc3_decoder->dt, stream->lc3_decoder->sr_pcm);
        RT_ASSERT(client);
        rt_tick_t cur = rt_tick_get_millisecond();
        if (cur - last_write_tick > 200)
        {
            printk("sink got new data after mute\r\n");
            uint32_t bytes = BAP_BROADCAST_SINK_CACHE_SIZE * 3 / 8;
            while (1)
            {
                audio_ioctl(client, 3, &bytes);
                if (bytes < BAP_BROADCAST_SINK_CACHE_SIZE  * 3 / 8)
                {
                    audio_write(client, zero, sizeof(zero));
                    bytes += sizeof(zero);
                }
                break;
            }
        }
        if (rt_tick_get_millisecond() - last_write_tick > 200)
        {
            uint32_t bytes = 0;
            printk("cache tigger\r\n");
            while (bytes < BAP_BROADCAST_SINK_CACHE_SIZE * 3 / 8)
            {
                audio_write(client, zero, sizeof(zero));
                bytes += sizeof(zero);
            }
        }
        audio_write(client, (uint8_t *)lc3_audio_buf, num_of_sample * 2);
        last_write_tick = rt_tick_get_millisecond();
    }
}
