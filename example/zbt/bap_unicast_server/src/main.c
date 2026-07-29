/*
 * Copyright (c) 2021-2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <audio_server.h>
#include "common.h"
#include <zephyr/autoconf.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/lc3.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys_clock.h>
#include <zephyr/types.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#define LOG_INTERVAL 1000U

#define AVAILABLE_SINK_CONTEXT  (BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | \
                 BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | \
                 BT_AUDIO_CONTEXT_TYPE_MEDIA | \
                 BT_AUDIO_CONTEXT_TYPE_GAME | \
                 BT_AUDIO_CONTEXT_TYPE_INSTRUCTIONAL)

#define AVAILABLE_SOURCE_CONTEXT (BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | \
                  BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | \
                  BT_AUDIO_CONTEXT_TYPE_MEDIA | \
                  BT_AUDIO_CONTEXT_TYPE_GAME)

#define ENABLE_SERVER_SOURCE 0
#define ADVERTISED_SOURCE_CONTEXT (ENABLE_SERVER_SOURCE ? AVAILABLE_SOURCE_CONTEXT : 0)

#define ADV_START_RETRY_COUNT 5
#define ADV_START_RETRY_DELAY K_MSEC(100)

NET_BUF_POOL_FIXED_DEFINE(tx_pool, CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT,
                          BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
                          CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

static const struct bt_audio_codec_cap lc3_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
            BT_AUDIO_CODEC_CAP_FREQ_ANY, BT_AUDIO_CODEC_CAP_DURATION_10,
            BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1), 40u, 120u, 1u,
            (BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_MEDIA));

static struct bt_conn *default_conn;
static struct k_work_delayable audio_send_work;

static struct audio_sink
{
    struct bt_bap_stream stream;
    size_t recv_cnt;
    size_t loss_cnt;
    size_t error_cnt;
    size_t valid_cnt;
    size_t bytes_cnt;
} sink_streams[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static audio_client_t client = NULL;

#if defined(CONFIG_LIBLC3)
static int audio_callback_play(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    if (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty)
    {
        /* Audio server may request more data here. LC3 data is pushed from ISO RX path. */
    }

    return 0;
}
#endif

static struct audio_source
{
    struct bt_bap_stream stream;
    uint16_t seq_num;
    uint16_t max_sdu;
    size_t len_to_send;
    bool started;
    bool in_flight;
} source_streams[CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT];
static size_t configured_source_stream_count;

static const struct bt_bap_qos_cfg_pref qos_pref =
    BT_BAP_QOS_CFG_PREF(true, BT_GAP_LE_PHY_2M, 0x02, 10, 40000, 40000, 40000, 40000);

static K_SEM_DEFINE(sem_disconnected, 0, 1);

static uint8_t unicast_server_addata[] =
{
    BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL), /* ASCS UUID */
    BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED, /* Target Announcement */
    BT_BYTES_LIST_LE16(AVAILABLE_SINK_CONTEXT),
    BT_BYTES_LIST_LE16(ADVERTISED_SOURCE_CONTEXT),
    0x00, /* Metadata length */
};

/* TODO: Expand with BAP data */
static const struct bt_data ad[] =
{
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL)),
    BT_DATA(BT_DATA_SVC_DATA16, unicast_server_addata, ARRAY_SIZE(unicast_server_addata)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static int start_advertising(struct bt_le_ext_adv *adv)
{
    int err;

    for (uint8_t i = 0U; i < ADV_START_RETRY_COUNT; i++)
    {
        err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
        if (err == 0)
        {
            printk("Advertising successfully started\n");
            return 0;
        }

        if (err != -ENOMEM)
        {
            printk("Failed to start advertising set (err %d)\n", err);
            return err;
        }

        printk("No conn resource for advertising, retry %u/%u\n",
               i + 1U, ADV_START_RETRY_COUNT);
        k_sleep(ADV_START_RETRY_DELAY);
    }

    printk("Failed to start advertising set (err %d)\n", err);
    return err;
}

#define AUDIO_DATA_TIMEOUT_US 1000000UL /* Send data every 1 second */
#define SDU_INTERVAL_US       10000UL   /* 10 ms SDU interval */

static uint16_t get_and_incr_seq_num(const struct bt_bap_stream *stream)
{
    for (size_t i = 0U; i < configured_source_stream_count; i++)
    {
        if (stream == &source_streams[i].stream)
        {
            uint16_t seq_num;

            seq_num = source_streams[i].seq_num;

            if (IS_ENABLED(CONFIG_LIBLC3))
            {
                source_streams[i].seq_num++;
            }
            else
            {
                source_streams[i].seq_num += (AUDIO_DATA_TIMEOUT_US /
                                              SDU_INTERVAL_US);
            }

            return seq_num;
        }
    }

    printk("Could not find endpoint from stream %p\n", stream);

    return 0;
}

#if defined(CONFIG_LIBLC3)

    #include "lc3.h"

    #define MAX_SAMPLE_RATE         48000
    #define MAX_FRAME_DURATION_US   10000
    #define MAX_NUM_SAMPLES         ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)

    static int16_t audio_buf[MAX_NUM_SAMPLES];
    static lc3_decoder_t lc3_decoder;
    static lc3_decoder_mem_48k_t lc3_decoder_mem;
    static int frames_per_sdu;
    static uint16_t lc3_octets_per_frame;
    static size_t audio_write_cnt;

#endif

void print_hex(const uint8_t *ptr, size_t len)
{
    while (len-- != 0)
    {
        printk("%02x", *ptr++);
    }
}

static bool print_cb(struct bt_data *data, void *user_data)
{
    const char *str = (const char *)user_data;

    printk("%s: type 0x%02x value_len %u\n", str, data->type, data->data_len);
    print_hex(data->data, data->data_len);
    printk("\n");

    return true;
}

static void print_codec_cfg(const struct bt_audio_codec_cfg *codec_cfg)
{
    printk("codec_cfg 0x%02x cid 0x%04x vid 0x%04x count %u\n", codec_cfg->id, codec_cfg->cid,
           codec_cfg->vid, codec_cfg->data_len);

    if (codec_cfg->id == BT_HCI_CODING_FORMAT_LC3)
    {
        enum bt_audio_location chan_allocation;
        int ret;

        /* LC3 uses the generic LTV format - other codecs might do as well */

        bt_audio_data_parse(codec_cfg->data, codec_cfg->data_len, print_cb, "data");

        ret = bt_audio_codec_cfg_get_freq(codec_cfg);
        if (ret > 0)
        {
            printk("  Frequency: %d Hz\n", bt_audio_codec_cfg_freq_to_freq_hz(ret));
        }

        ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
        if (ret > 0)
        {
            printk("  Frame Duration: %d us\n",
                   bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret));
        }

        ret = bt_audio_codec_cfg_get_chan_allocation(codec_cfg, &chan_allocation, false);
        if (ret == 0)
        {
            printk("  Channel allocation: 0x%x\n", chan_allocation);
        }

        printk("  Octets per frame: %d (negative means value not pressent)\n",
               bt_audio_codec_cfg_get_octets_per_frame(codec_cfg));
        printk("  Frames per SDU: %d\n",
               bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec_cfg, true));
    }
    else
    {
        print_hex(codec_cfg->data, codec_cfg->data_len);
    }

    bt_audio_data_parse(codec_cfg->meta, codec_cfg->meta_len, print_cb, "meta");
}

static void print_qos(const struct bt_bap_qos_cfg *qos)
{
    printk("QoS: interval %u framing 0x%02x phy 0x%02x sdu %u "
           "rtn %u latency %u pd %u\n",
           qos->interval, qos->framing, qos->phy, qos->sdu,
           qos->rtn, qos->latency, qos->pd);
}

/**
 * @brief Send audio data on timeout
 *
 * This will send an increasing amount of audio data, starting from 1 octet.
 * The data is just mock data, and does not actually represent any audio.
 *
 * First iteration : 0x00
 * Second iteration: 0x00 0x01
 * Third iteration : 0x00 0x01 0x02
 *
 * And so on, until it wraps around the configured MTU (CONFIG_BT_ISO_TX_MTU)
 *
 * @param work Pointer to the work structure
 */
static void audio_timer_timeout(struct k_work *work)
{
    int ret;
    static uint8_t buf_data[CONFIG_BT_ISO_TX_MTU];
    static bool data_initialized;
    struct net_buf *buf;

    if (!data_initialized)
    {
        /* TODO: Actually encode some audio data */
        for (size_t i = 0U; i < ARRAY_SIZE(buf_data); i++)
        {
            buf_data[i] = (uint8_t)i;
        }

        data_initialized = true;
    }

    /* We configured the sink streams to be first in `streams`, so that
     * we can use `stream[i]` to select sink streams (i.e. streams with
     * data going to the server)
     */
    for (size_t i = 0; i < configured_source_stream_count; i++)
    {
        struct bt_bap_stream *stream = &source_streams[i].stream;

        if (!source_streams[i].started || source_streams[i].in_flight ||
                source_streams[i].max_sdu == 0U)
        {
            continue;
        }

        buf = net_buf_alloc(&tx_pool, K_NO_WAIT);
        if (buf == NULL)
        {
            printk("Failed to allocate TX buffer\n");
            /* Break and retry later */
            break;
        }
        net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

        net_buf_add_mem(buf, buf_data, ++source_streams[i].len_to_send);

        ret = bt_bap_stream_send(stream, buf, get_and_incr_seq_num(stream));
        if (ret < 0)
        {
            printk("Failed to send audio data on streams[%zu] (%p): (%d)\n",
                   i, stream, ret);
            net_buf_unref(buf);
        }
        else
        {
            source_streams[i].in_flight = true;
            LOG_DBG("Sending mock data with len %zu on streams[%zu] (%p)\n",
                    source_streams[i].len_to_send, i, stream);
        }

        if (source_streams[i].len_to_send >= source_streams[i].max_sdu)
        {
            source_streams[i].len_to_send = 0;
        }
    }

#if defined(CONFIG_LIBLC3)
    k_work_schedule(&audio_send_work, K_USEC(MAX_FRAME_DURATION_US));
#else
    k_work_schedule(&audio_send_work, K_USEC(AUDIO_DATA_TIMEOUT_US));
#endif
}

static enum bt_audio_dir stream_dir(const struct bt_bap_stream *stream)
{
    for (size_t i = 0U; i < ARRAY_SIZE(source_streams); i++)
    {
        if (stream == &source_streams[i].stream)
        {
            return BT_AUDIO_DIR_SOURCE;
        }
    }

    for (size_t i = 0U; i < ARRAY_SIZE(sink_streams); i++)
    {
        if (stream == (const struct bt_bap_stream *)&sink_streams[i])
        {
            return BT_AUDIO_DIR_SINK;
        }
    }

    __ASSERT(false, "Invalid stream %p", stream);
    return 0;
}

static struct bt_bap_stream *stream_alloc(enum bt_audio_dir dir)
{
    if (dir == BT_AUDIO_DIR_SOURCE)
    {
        for (size_t i = 0; i < ARRAY_SIZE(source_streams); i++)
        {
            struct bt_bap_stream *stream = &source_streams[i].stream;

            if (!stream->conn)
            {
                return stream;
            }
        }
    }
    else
    {
        for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++)
        {
            struct bt_bap_stream *stream = (struct bt_bap_stream *)&sink_streams[i];

            if (!stream->conn)
            {
                return stream;
            }
        }
    }

    return NULL;
}

static int lc3_config(struct bt_conn *conn, const struct bt_bap_ep *ep, enum bt_audio_dir dir,
                      const struct bt_audio_codec_cfg *codec_cfg, struct bt_bap_stream **stream,
                      struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
    printk("ASE Codec Config: conn %p ep %p dir %u\n", conn, ep, dir);

    print_codec_cfg(codec_cfg);

    *stream = stream_alloc(dir);
    if (*stream == NULL)
    {
        printk("No streams available\n");
        *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_NO_MEM, BT_BAP_ASCS_REASON_NONE);

        return -ENOMEM;
    }

    printk("ASE Codec Config stream %p\n", *stream);

    if (dir == BT_AUDIO_DIR_SOURCE)
    {
        configured_source_stream_count++;
    }

    *pref = qos_pref;

#if defined(CONFIG_LIBLC3)
    /* Nothing to free as static memory is used */
    lc3_decoder = NULL;
#endif

    return 0;
}

static int lc3_reconfig(struct bt_bap_stream *stream, enum bt_audio_dir dir,
                        const struct bt_audio_codec_cfg *codec_cfg,
                        struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
    printk("ASE Codec Reconfig: stream %p\n", stream);

    print_codec_cfg(codec_cfg);

#if defined(CONFIG_LIBLC3)
    /* Nothing to free as static memory is used */
    lc3_decoder = NULL;
#endif

    *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_UNSUPPORTED, BT_BAP_ASCS_REASON_NONE);

    /* We only support one QoS at the moment, reject changes */
    return -ENOEXEC;
}

static int lc3_qos(struct bt_bap_stream *stream, const struct bt_bap_qos_cfg *qos,
                   struct bt_bap_ascs_rsp *rsp)
{
    printk("QoS: stream %p qos %p\n", stream, qos);

    print_qos(qos);

    for (size_t i = 0U; i < configured_source_stream_count; i++)
    {
        if (stream == &source_streams[i].stream)
        {
            source_streams[i].max_sdu = qos->sdu;
            break;
        }
    }

    return 0;
}

static int lc3_enable(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
                      struct bt_bap_ascs_rsp *rsp)
{
    printk("Enable: stream %p meta_len %zu\n", stream, meta_len);

#if defined(CONFIG_LIBLC3)
    {
        int frame_duration_us;
        int freq;
        int ret;

        ret = bt_audio_codec_cfg_get_freq(stream->codec_cfg);
        if (ret > 0)
        {
            freq = bt_audio_codec_cfg_freq_to_freq_hz(ret);
        }
        else
        {
            printk("Error: Codec frequency not set, cannot start codec.");
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
                                   BT_BAP_ASCS_REASON_CODEC_DATA);
            return ret;
        }

        ret = bt_audio_codec_cfg_get_frame_dur(stream->codec_cfg);
        if (ret > 0)
        {
            frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
        }
        else
        {
            printk("Error: Frame duration not set, cannot start codec.");
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
                                   BT_BAP_ASCS_REASON_CODEC_DATA);
            return ret;
        }

        frames_per_sdu =
            bt_audio_codec_cfg_get_frame_blocks_per_sdu(stream->codec_cfg, true);
        if (frames_per_sdu <= 0)
        {
            printk("Error: Frame blocks per SDU not set, cannot start codec.");
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
                                   BT_BAP_ASCS_REASON_CODEC_DATA);
            return frames_per_sdu;
        }

        ret = bt_audio_codec_cfg_get_octets_per_frame(stream->codec_cfg);
        if (ret > 0)
        {
            lc3_octets_per_frame = (uint16_t)ret;
        }
        else
        {
            printk("Error: Octets per frame not set, cannot start codec.");
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
                                   BT_BAP_ASCS_REASON_CODEC_DATA);
            return ret;
        }

        lc3_decoder = lc3_setup_decoder(frame_duration_us,
                                        freq,
                                        0, /* No resampling */
                                        &lc3_decoder_mem);

        if (lc3_decoder == NULL)
        {
            printk("ERROR: Failed to setup LC3 decoder - wrong parameters?\n");
            *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
                                   BT_BAP_ASCS_REASON_CODEC_DATA);
            return -1;
        }
    }
#endif

    return 0;
}

static int lc3_start(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    printk("Start: stream %p\n", stream);

    for (size_t i = 0U; i < configured_source_stream_count; i++)
    {
        if (stream == &source_streams[i].stream)
        {
            source_streams[i].seq_num = 0U;
            source_streams[i].len_to_send = 0U;
            source_streams[i].in_flight = false;
            break;
        }
    }

    if (configured_source_stream_count > 0 &&
            !k_work_delayable_is_pending(&audio_send_work))
    {

        /* Start send timer */
        k_work_schedule(&audio_send_work, K_MSEC(0));
    }

    return 0;
}

static bool data_func_cb(struct bt_data *data, void *user_data)
{
    struct bt_bap_ascs_rsp *rsp = (struct bt_bap_ascs_rsp *)user_data;

    if (!BT_AUDIO_METADATA_TYPE_IS_KNOWN(data->type))
    {
        printk("Invalid metadata type %u or length %u\n", data->type, data->data_len);
        *rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_METADATA_REJECTED, data->type);

        return -EINVAL;
    }

    return true;
}

static int lc3_metadata(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
                        struct bt_bap_ascs_rsp *rsp)
{
    printk("Metadata: stream %p meta_len %zu\n", stream, meta_len);

    return bt_audio_data_parse(meta, meta_len, data_func_cb, rsp);
}

static int lc3_disable(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    printk("Disable: stream %p\n", stream);

    return 0;
}

static int lc3_stop(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    printk("Stop: stream %p\n", stream);

    return 0;
}

static int lc3_release(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
    printk("Release: stream %p\n", stream);
    return 0;
}

static struct bt_bap_unicast_server_register_param param =
{
    CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT,
    ENABLE_SERVER_SOURCE ? CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT : 0
};

static const struct bt_bap_unicast_server_cb unicast_server_cb =
{
    .config = lc3_config,
    .reconfig = lc3_reconfig,
    .qos = lc3_qos,
    .enable = lc3_enable,
    .start = lc3_start,
    .metadata = lc3_metadata,
    .disable = lc3_disable,
    .stop = lc3_stop,
    .release = lc3_release,
};


#if defined(CONFIG_LIBLC3)

static void stream_recv_lc3_codec(struct bt_bap_stream *stream,
                                  const struct bt_iso_recv_info *info,
                                  struct net_buf *buf)
{
    struct audio_sink *sink_stream = (struct audio_sink *) stream;
    const uint8_t *in_buf = NULL;
    bool need_plc = false;
    int err = 0;

    if (lc3_decoder == NULL)
    {
        printk("LC3 decoder not setup, cannot decode data.\n");
        return;
    }

    sink_stream->recv_cnt++;
    if (info->flags & BT_ISO_FLAGS_LOST)
    {
        sink_stream->loss_cnt++;
        need_plc = true;
    }
    else if (info->flags & BT_ISO_FLAGS_ERROR)
    {
        sink_stream->error_cnt++;
        need_plc = true;
    }
    else if ((info->flags & (BT_ISO_FLAGS_VALID)) == 0)
    {
        printk("Bad packet: 0x%02X\n", info->flags);
        sink_stream->error_cnt++;
        need_plc = true;
    }
    else
    {
        sink_stream->valid_cnt++;
        in_buf = buf->data;
        sink_stream->bytes_cnt += buf->len;

        if (buf->len == 0U)
        {
            /* Some controllers/peers can deliver empty ISO SDUs around stream startup.
             * They do not contain an LC3 frame, so ignore them instead of treating them
             * as a decode error.
             */
            return;
        }
    }
    if ((sink_stream->recv_cnt % LOG_INTERVAL) == 0U)
    {
        printk("Stream %p: received %u total ISO packets: Valid %u | Error %u | Loss %u | bytes %u\n",
               &sink_stream->stream, sink_stream->recv_cnt, sink_stream->valid_cnt,
               sink_stream->error_cnt, sink_stream->loss_cnt, sink_stream->bytes_cnt);
    }

    if (frames_per_sdu <= 0 || lc3_octets_per_frame == 0)
    {
        printk("LC3 parameters not ready: frames_per_sdu=%d octets=%u\n",
               frames_per_sdu, lc3_octets_per_frame);
        return;
    }

    if (need_plc)
    {
        err = lc3_decode(lc3_decoder, NULL, lc3_octets_per_frame,
                         LC3_PCM_FORMAT_S16, audio_buf, 1);
    }
    else if (buf->len == (lc3_octets_per_frame * frames_per_sdu))
    {
        int offset = 0;

        for (int frame_idx = 0; frame_idx < frames_per_sdu; frame_idx++)
        {
            err = lc3_decode(lc3_decoder, in_buf + offset, lc3_octets_per_frame,
                             LC3_PCM_FORMAT_S16, audio_buf, 1);
            offset += lc3_octets_per_frame;
        }
    }
    else
    {
        printk("Unexpected LC3 SDU len %u, expected %d\n",
               buf->len, lc3_octets_per_frame * frames_per_sdu);
        return;
    }

    if (err == 1)
    {
        printk("  decoder performed PLC\n");
    }
    else if (err < 0)
    {
        printk("  decoder failed - wrong parameters?\n");
        return;
    }

    if (client)
    {
        uint32_t ns = LC3_NS(lc3_decoder->dt, lc3_decoder->sr_pcm);
        uint32_t bytes = ns * sizeof(int16_t);

        audio_write(client, (uint8_t *)audio_buf, bytes);
        audio_write_cnt++;

        if ((audio_write_cnt % LOG_INTERVAL) == 0U)
        {
            printk("LC3 audio_write %u times, %u bytes each\n", audio_write_cnt, bytes);
        }
    }
}

#else

static void stream_recv(struct bt_bap_stream *stream,
                        const struct bt_iso_recv_info *info,
                        struct net_buf *buf)
{
    if (info->flags & BT_ISO_FLAGS_VALID)
    {
        printk("Incoming audio on stream %p len %u\n", stream, buf->len);
    }
}

#endif

static void stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
    bool any_source_started = false;

    printk("Audio Stream %p stopped with reason 0x%02X\n", stream, reason);

    if (stream_dir(stream) == BT_AUDIO_DIR_SOURCE)
    {
        for (size_t i = 0U; i < configured_source_stream_count; i++)
        {
            if (stream == &source_streams[i].stream)
            {
                source_streams[i].started = false;
                source_streams[i].len_to_send = 0U;
                source_streams[i].in_flight = false;
            }

            any_source_started = any_source_started || source_streams[i].started;
        }

        if (!any_source_started)
        {
            k_work_cancel_delayable(&audio_send_work);
        }
    }
    else if (client)
    {
        audio_close(client);
        client = NULL;

#if defined(CONFIG_LIBLC3)
        audio_write_cnt = 0;
#endif
    }
}

static void stream_started(struct bt_bap_stream *stream)
{
    printk("Audio Stream %p started\n", stream);
    if (stream_dir(stream) == BT_AUDIO_DIR_SINK)
    {
        struct audio_sink *sink_stream = (struct audio_sink *)stream;
#if defined(CONFIG_LIBLC3)
        if (!client)
        {
            int freq_hz = 48000;
            int ret = bt_audio_codec_cfg_get_freq(stream->codec_cfg);

            if (ret > 0)
            {
                freq_hz = bt_audio_codec_cfg_freq_to_freq_hz(ret);
            }
            else
            {
                printk("Error: Codec frequency not set, use default %d Hz.\n", freq_hz);
            }

            audio_parameter_t pa = {0};
            pa.is_bap_sink = 1;
            pa.write_bits_per_sample = 16;
            pa.write_channnel_num = 1;
            pa.write_samplerate = (uint32_t)freq_hz;
            pa.read_bits_per_sample = 16;
            pa.read_cache_size = 0;
            pa.write_cache_size = 16000;

            printk("Opening audio output for BAP sink: %d Hz, 1 channel, 16 bits\n", freq_hz);
            client = audio_open(AUDIO_TYPE_BT_MUSIC, AUDIO_TX, &pa, audio_callback_play, &client);
            RT_ASSERT(client);
        }
#endif
        sink_stream->recv_cnt = 0;
        sink_stream->loss_cnt = 0;
        sink_stream->error_cnt = 0;
        sink_stream->valid_cnt = 0;
        sink_stream->bytes_cnt = 0;
    }
    else
    {
        for (size_t i = 0U; i < configured_source_stream_count; i++)
        {
            if (stream == &source_streams[i].stream)
            {
                source_streams[i].started = true;
                source_streams[i].in_flight = false;
                break;
            }
        }
    }
}

static void stream_enabled_cb(struct bt_bap_stream *stream)
{
    /* The unicast server is responsible for starting sink ASEs after the
     * client has enabled them.
     */
    if (stream_dir(stream) == BT_AUDIO_DIR_SINK)
    {
        const int err = bt_bap_stream_start(stream);

        if (err != 0)
        {
            printk("Failed to start stream %p: %d", stream, err);
        }
    }
}

static struct bt_bap_stream_ops stream_ops =
{
#if defined(CONFIG_LIBLC3)
    .recv = stream_recv_lc3_codec,
#else
    .recv = stream_recv,
#endif
    .stopped = stream_stopped,
    .started = stream_started,
    .enabled = stream_enabled_cb,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err != 0)
    {
        printk("Failed to connect to %s %u %s\n", addr, err, bt_hci_err_to_str(err));

        default_conn = NULL;
        return;
    }

    printk("Connected: %s\n", addr);
    default_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (conn != default_conn)
    {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Disconnected: %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

    bt_conn_unref(default_conn);
    default_conn = NULL;

    k_sem_give(&sem_disconnected);
}

BT_CONN_CB_DEFINE(conn_callbacks) =
{
    .connected = connected,
    .disconnected = disconnected,
};

static void bond_print_cb(const struct bt_bond_info *info, void *user_data)
{
    size_t *bond_count = user_data;
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(&info->addr, addr, sizeof(addr));
    printk("Bond[%u]: %s\n", (unsigned int)(*bond_count), addr);
    (*bond_count)++;
}

static void print_bonds(const char *tag)
{
    size_t bond_count = 0U;

    printk("Bond list %s:\n", tag);
    bt_foreach_bond(BT_ID_DEFAULT, bond_print_cb, &bond_count);
    printk("Bond list %s: total %u\n", tag, (unsigned int)bond_count);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("Pairing completed: %s bonded=%u\n", addr, bonded);
    print_bonds("after pairing");
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    int err;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("Pairing failed: %s reason=%u\n", addr, reason);

    if (reason == BT_SECURITY_ERR_PIN_OR_KEY_MISSING ||
            reason == BT_SECURITY_ERR_AUTH_REQUIREMENT ||
            reason == BT_SECURITY_ERR_AUTH_FAIL)
    {
        printk("Removing stale bond for %s\n", addr);

        err = bt_unpair(BT_ID_DEFAULT, bt_conn_get_dst(conn));
        if (err != 0)
        {
            printk("Failed to remove stale bond for %s (err %d)\n", addr, err);
        }
        else
        {
            print_bonds("after stale bond removal");
        }
    }
}

static void bond_deleted(uint8_t id, const bt_addr_le_t *peer)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(peer, addr, sizeof(addr));
    printk("Bond deleted: id=%u peer=%s\n", id, addr);
}

static struct bt_conn_auth_info_cb auth_info_cb =
{
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed,
    .bond_deleted = bond_deleted,
};

static struct bt_pacs_cap cap_sink =
{
    .codec_cap = &lc3_codec_cap,
};

static struct bt_pacs_cap cap_source =
{
    .codec_cap = &lc3_codec_cap,
};

static int set_location(void)
{
    int err;

    if (IS_ENABLED(CONFIG_BT_PAC_SNK_LOC))
    {
        err = bt_pacs_set_location(BT_AUDIO_DIR_SINK,
                                   BT_AUDIO_LOCATION_FRONT_CENTER);
        if (err != 0)
        {
            printk("Failed to set sink location (err %d)\n", err);
            return err;
        }
    }

    if (ENABLE_SERVER_SOURCE && IS_ENABLED(CONFIG_BT_PAC_SRC_LOC))
    {
        err = bt_pacs_set_location(BT_AUDIO_DIR_SOURCE,
                                   (BT_AUDIO_LOCATION_FRONT_LEFT |
                                    BT_AUDIO_LOCATION_FRONT_RIGHT));
        if (err != 0)
        {
            printk("Failed to set source location (err %d)\n", err);
            return err;
        }
    }

    printk("Location successfully set\n");

    return 0;
}

static int set_supported_contexts(void)
{
    int err;

    if (IS_ENABLED(CONFIG_BT_PAC_SNK))
    {
        err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SINK,
                                             AVAILABLE_SINK_CONTEXT);
        if (err != 0)
        {
            printk("Failed to set sink supported contexts (err %d)\n",
                   err);

            return err;
        }
    }

    if (ENABLE_SERVER_SOURCE && IS_ENABLED(CONFIG_BT_PAC_SRC))
    {
        err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SOURCE,
                                             AVAILABLE_SOURCE_CONTEXT);
        if (err != 0)
        {
            printk("Failed to set source supported contexts (err %d)\n",
                   err);

            return err;
        }
    }

    printk("Supported contexts successfully set\n");

    return 0;
}

static int set_available_contexts(void)
{
    int err;

    if (IS_ENABLED(CONFIG_BT_PAC_SNK))
    {
        err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SINK,
                                             AVAILABLE_SINK_CONTEXT);
        if (err != 0)
        {
            printk("Failed to set sink available contexts (err %d)\n", err);
            return err;
        }
    }

    if (ENABLE_SERVER_SOURCE && IS_ENABLED(CONFIG_BT_PAC_SRC))
    {
        err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SOURCE,
                                             AVAILABLE_SOURCE_CONTEXT);
        if (err != 0)
        {
            printk("Failed to set source available contexts (err %d)\n", err);
            return err;
        }
    }

    printk("Available contexts successfully set\n");
    return 0;
}

int main(void)
{
    struct bt_le_ext_adv *adv;
    int err;

    err = bt_enable(NULL);
    if (err != 0)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }
    printk("Bluetooth initialized\n");

    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        err = settings_load();
        if (err != 0)
        {
            printk("Failed to load settings (err %d)\n", err);
            return 0;
        }

        printk("Settings loaded\n");
        print_bonds("after settings_load");
    }

    err = bt_conn_auth_info_cb_register(&auth_info_cb);
    if (err != 0)
    {
        printk("Failed to register auth info callbacks (err %d)\n", err);
    }

    bt_bap_unicast_server_register(&param);
    bt_bap_unicast_server_register_cb(&unicast_server_cb);

    bt_pacs_cap_register(BT_AUDIO_DIR_SINK, &cap_sink);
    if (ENABLE_SERVER_SOURCE)
    {
        bt_pacs_cap_register(BT_AUDIO_DIR_SOURCE, &cap_source);
    }

    for (size_t i = 0; i < ARRAY_SIZE(sink_streams); i++)
    {
        bt_bap_stream_cb_register((struct bt_bap_stream *)&sink_streams[i], &stream_ops);
    }

    for (size_t i = 0; ENABLE_SERVER_SOURCE && i < ARRAY_SIZE(source_streams); i++)
    {
        bt_bap_stream_cb_register(&source_streams[i].stream,
                                  &stream_ops);
    }

    err = set_location();
    if (err != 0)
    {
        return 0;
    }

    err = set_supported_contexts();
    if (err != 0)
    {
        return 0;
    }

    err = set_available_contexts();
    if (err != 0)
    {
        return 0;
    }

    /* Create a connectable advertising set */
    err = bt_le_ext_adv_create(BT_LE_EXT_ADV_CONN, NULL, &adv);
    if (err)
    {
        printk("Failed to create advertising set (err %d)\n", err);
        return 0;
    }

    err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
    {
        printk("Failed to set advertising data (err %d)\n", err);
        return 0;
    }

    while (true)
    {
        struct k_work_sync sync;

        err = start_advertising(adv);
        if (err)
        {
            break;
        }

        if (ENABLE_SERVER_SOURCE && CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT > 0)
        {
            /* Start send timer */
            k_work_init_delayable(&audio_send_work, audio_timer_timeout);
        }

        err = k_sem_take(&sem_disconnected, K_FOREVER);
        if (err != 0)
        {
            printk("failed to take sem_disconnected (err %d)\n", err);
            break;
        }

        /* reset data */
        if (configured_source_stream_count)
        {
            configured_source_stream_count = 0U;
            k_work_cancel_delayable_sync(&audio_send_work, &sync);
        }

        for (size_t i = 0; i < ARRAY_SIZE(source_streams); i++)
        {
            source_streams[i].seq_num = 0U;
            source_streams[i].max_sdu = 0U;
            source_streams[i].len_to_send = 0U;
            source_streams[i].started = false;
            source_streams[i].in_flight = false;
        }

        if (client)
        {
            audio_close(client);
            client = NULL;
        }
    }
    if (client)
    {
        audio_close(client);
    }
    return 0;
}
