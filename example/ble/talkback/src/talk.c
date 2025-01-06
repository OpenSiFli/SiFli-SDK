#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>
#include "os_adaptor.h"

#include "main.h"

#if defined(PKG_LIB_OPUS) && defined(RT_USING_FINSH) && defined(AUDIO_USING_MANAGER)
#include "opus_types.h"
#include "opus_multistream.h"
#include "os_support.h"
#include "audio_server.h"

#undef LOG_TAG

#define LOG_TAG           "opus"
#define DBG_TAG           "opus"
#define DBG_LVL           LOG_LVL_INFO
#include "log.h"

//#define SENDER_DEBUG_MIC2SPEAKER    1  //voice sender debug, loop mic to local speaker before sending to peer

#define RECORD_USING_WEBRTC         0
#define OPUS_USING_20MS             1
#define CODEC_DATA_UNIT_LEN         320
#define OPUS_STACK_SIZE             220000

#define OPUS_PACKET_HEADER          0x88
#define TALK_THREAD_NAME            "opusble"
#define OPUS_THREAD_PRIORITY        (RT_THREAD_PRIORITY_MIDDLE + RT_THREAD_PRIORITY_HIGHER)
#define OPUS_EVENT_MIC_RX           (1 << 0)
#define OPUS_EVENT_SPK_TX           (1 << 1)
#define OPUS_EVENT_BLE_DOWNLINK     (1 << 2)
#define OPUS_ENCODE_QUEUE_NUM       3
#define OPUS_SAMPLERATE             8000 //16000

#if OPUS_SAMPLERATE == 16000
    #define OPUS_DOWNLINK_QUEUE_NUM     64
#else
    #define OPUS_DOWNLINK_QUEUE_NUM     32
#endif

#if OPUS_USING_20MS
    #if OPUS_SAMPLERATE == 16000
        #define OPUS_ENCODE_DATA_LEN    (CODEC_DATA_UNIT_LEN * 2)
    #else
        #define OPUS_ENCODE_DATA_LEN    (CODEC_DATA_UNIT_LEN)
    #endif
#else
    #if OPUS_SAMPLERATE == 16000
        #define OPUS_ENCODE_DATA_LEN    (CODEC_DATA_UNIT_LEN)
    #else
        #define OPUS_ENCODE_DATA_LEN    (CODEC_DATA_UNIT_LEN/2)
    #endif
#endif

#define OPUS_EVENT_ALL  (OPUS_EVENT_MIC_RX | OPUS_EVENT_SPK_TX | OPUS_EVENT_BLE_DOWNLINK)

#pragma pack(push, 1)
typedef struct
{
    rt_slist_t  slist;
    uint8_t     id;
    uint8_t     len;
    uint8_t     data[OPUS_ENCODE_DATA_LEN];
} ble_opus_packet_t;

typedef struct
{
    rt_slist_t  node;
    uint8_t     *data;
    uint16_t    data_len;
    uint16_t    size;
} opus_decode_queue_t;

typedef enum
{
    OPUS_WAIT_ID,
    OPUS_WAIT_LEN,
    OPUS_WAIT_DATA,
} opus_packet_state_t;

#pragma pack(pop)
typedef struct
{
    rt_slist_t              downlink_decode_busy;
    rt_slist_t              downlink_decode_idle;
    rt_slist_t              encode_slist;
    ble_opus_packet_t       encode_one[OPUS_ENCODE_QUEUE_NUM];
    uint8_t                 send_data[OPUS_ENCODE_DATA_LEN * OPUS_ENCODE_QUEUE_NUM + 10 * OPUS_ENCODE_QUEUE_NUM];

    opus_decode_queue_t     downlink_queue[OPUS_DOWNLINK_QUEUE_NUM];
    uint8_t                 downlink_data[OPUS_ENCODE_DATA_LEN];
    uint16_t                downlink_data_len;
    uint16_t                downlink_data_pos;
    uint8_t                 downlink_decode_out[OPUS_ENCODE_DATA_LEN];
    struct rt_ringbuffer    *rbuf_out;
    os_thread_t             opus_thread;
    audio_client_t          client;
    OpusEncoder             *encoder;
    OpusDecoder             *decoder;
    struct rt_ringbuffer    *rb_opus_encode_input;
    struct rt_ringbuffer    *rb_opus_decode_input;
    rt_event_t              event;
    opus_packet_state_t     state;
    audio_rwflag_t          flag;
    uint16_t                speaker_cached_packets;
    uint8_t                 is_to_speaker_enable;
    uint8_t                 is_first_encoded;
    uint8_t                 mic_rx_count;
    uint8_t                 send_id;
    uint8_t                 recv_id;
    uint8_t                 readed_id;
    uint8_t                 is_got_first_id;
    uint8_t                 is_tx_enable;
    uint8_t                 is_rx_enable;
    uint8_t                 is_exit;
    uint8_t                 is_inited;
} ble_talker_t;

static ble_talker_t *g_talker = NULL;

static  uint32_t g_opus_stack[OPUS_STACK_SIZE / sizeof(uint32_t)];


#if RECORD_USING_WEBRTC
#include "webrtc/modules/audio_processing/ns/include/noise_suppression_x.h"
#include "webrtc/modules/audio_processing/agc/legacy/gain_control.h"
static NsxHandle               *pNS_inst;
static void                    *agcInst;
static uint8_t *frame0;
static uint8_t *frame1;
static uint8_t *in;
static uint8_t *out;
static void app_recorder_ans_proc(NsxHandle *h, int16_t spframe[160], int16_t outframe[160])
{
    int16_t *spframe_p[1] = {&spframe[0]};
    int16_t *outframe_p[1] = {&outframe[0]};
    if (h)
    {
        WebRtcNsx_Process(h, (const int16_t *const *)spframe_p, 1, outframe_p);
    }
}

static void app_recorder_agc_proc(void *h, int16_t spframe[160], int16_t outframe[160])
{
    int32_t micLevelIn = 0;
    int32_t micLevelOut = 0;
    uint8_t saturationWarning;
    uint16_t u16_frame_len = 160;
    int16_t *spframe_p[1] = {&spframe[0]};
    int16_t *outframe_p[1] = {&outframe[0]};
    if (h && 0 != WebRtcAgc_Process(h, (const int16_t *const *)spframe_p, 1, u16_frame_len, (int16_t *const *)outframe_p, micLevelIn, &micLevelOut, 0, &saturationWarning))
    {
        LOG_W("WebRtcAgc_Process error !\n");
    }
}

static void webrtc_process_frame(const uint8_t *p, uint32_t data_len)
{
    app_recorder_ans_proc(pNS_inst, (int16_t *)p, (int16_t *)frame0);
    app_recorder_agc_proc(agcInst, (int16_t *)frame0, (int16_t *)frame1);
#if OPUS_SAMPLERATE == 8000
    app_recorder_ans_proc(pNS_inst, (int16_t *)(p + 160), (int16_t *)(frame0 + 160));
    app_recorder_agc_proc(agcInst, (int16_t *)(frame0 + 160), (int16_t *)(frame1 + 160));
#endif
}

static void webrtc_open()
{
    pNS_inst = WebRtcNsx_Create();
    RT_ASSERT(pNS_inst);

    if (0 != WebRtcNsx_Init(pNS_inst, OPUS_SAMPLERATE))
    {
        RT_ASSERT(0);
    }
    else if (0 != WebRtcNsx_set_policy(pNS_inst, 2))
    {
        RT_ASSERT(0);
    }
    WebRtcAgcConfig agcConfig;
    agcConfig.compressionGaindB = 19;
    agcConfig.limiterEnable = 1;
    agcConfig.targetLevelDbfs = 3;
    agcConfig.thrhold = 14;
    agcInst = WebRtcAgc_Create();
    RT_ASSERT(agcInst);
    if (0 != WebRtcAgc_Init(agcInst, 0, 255, 3, OPUS_SAMPLERATE)) // 3 --> kAgcModeFixedDigital
    {
        RT_ASSERT(0);
    }
    if (0 != WebRtcAgc_set_config(agcInst, agcConfig))
    {
        RT_ASSERT(0);
    }
    frame0 = rt_malloc(320);
    RT_ASSERT(frame0);
    frame1 = rt_malloc(320);
    RT_ASSERT(frame1);
}

static void webrtc_close()
{
    if (pNS_inst)
        WebRtcNsx_Free(pNS_inst);
    if (agcInst)
        WebRtcAgc_Free(agcInst);

    if (frame0)
        rt_free(frame0);

    if (frame1)
        rt_free(frame1);

    frame0   = NULL;
    frame1   = NULL;
    pNS_inst = NULL;
    agcInst  = NULL;
}

#endif


/*
    call in audio server thread, see audio_server.c
*/
static int audio_callback(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    ble_talker_t *thiz = (ble_talker_t *)callback_userdata;

    if (thiz->is_tx_enable && (cmd == as_callback_cmd_data_coming))
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
#if RECORD_USING_WEBRTC
        RT_ASSERT(p->data_len == 320);
        webrtc_process_frame(p->data, p->data_len);
#endif
        rt_ringbuffer_put(thiz->rb_opus_encode_input, p->data, p->data_len);
        thiz->mic_rx_count++;
#if OPUS_SAMPLERATE == 16000
        if (thiz->mic_rx_count > 1)
#endif
        {
            thiz->mic_rx_count = 0;
            rt_event_send(thiz->event, OPUS_EVENT_MIC_RX);
        }
    }
    else if (thiz->is_rx_enable && (cmd == as_callback_cmd_cache_half_empty || cmd == as_callback_cmd_cache_empty))
    {
    }
    return 0;
}

void ble_talk_downlink(uint8_t *data, uint16_t data_len)
{
    /*
        todo:
            this ble thread should using critical with opus_thread_entry for access g_talker
    */

    if (!g_talker || !data_len || !g_talker->is_rx_enable || !g_talker->is_inited)
    {
        return;
    }

    ble_talker_t *thiz = (ble_talker_t *)g_talker;

    //LOG_HEX("recv:", 16, data,data_len);

    while (g_talker && g_talker->is_inited && data_len > 0)
    {
        switch (thiz->state)
        {
        case OPUS_WAIT_ID:
            thiz->recv_id = *data++;
            thiz->state = OPUS_WAIT_LEN;
            data_len--;
            //LOG_I("id: new=%d old=%d", thiz->recv_id, thiz->readed_id);
            break;

        case OPUS_WAIT_LEN:
        {
            thiz->downlink_data_len = *data++;;
            thiz->state = OPUS_WAIT_DATA;
            data_len--;
            if (thiz->downlink_data_len == 0)
            {
                LOG_I("downlink err len=%02X", thiz->downlink_data_len);
                thiz->state = OPUS_WAIT_ID;
            }
            thiz->downlink_data_pos = 0;
            break;
        }
        case OPUS_WAIT_DATA:
        {
            uint32_t remain;
            remain = thiz->downlink_data_len - thiz->downlink_data_pos;
            if (remain > data_len)
            {
                memcpy(thiz->downlink_data + thiz->downlink_data_pos, data, data_len);
                thiz->downlink_data_pos += data_len;
                data_len = 0;
                data += data_len;
                break;
            }
            memcpy(thiz->downlink_data + thiz->downlink_data_pos, data, remain);
            data_len -= remain;
            data += remain;
            thiz->downlink_data_pos = 0;
            thiz->state = OPUS_WAIT_ID;
            int is_old = 0;
            //LOG_HEX("got opus", 16, thiz->downlink_data, thiz->downlink_data_len);
            if (thiz->is_got_first_id)
            {
                if (thiz->recv_id == (uint8_t)(thiz->readed_id + 1))
                {
                    //right packet
                    thiz->readed_id = thiz->recv_id;
                    //LOG_I("new=%d", thiz->readed_id);
                }
                else
                {
                    int is_lost = 1;
                    for (int i = 0; i < OPUS_ENCODE_QUEUE_NUM - 1; i++)
                    {
                        if ((uint8_t)(thiz->recv_id + i) == (uint8_t)thiz->readed_id)
                        {
                            is_lost = 0; //old packet
                            is_old = 1;
                            break;
                        }
                    }
                    if (is_lost)
                    {
                        LOG_I("lost %d packet, recv=%d readed=%d", (uint8_t)(thiz->recv_id - thiz->readed_id), thiz->recv_id, thiz->readed_id);
                        thiz->readed_id = thiz->recv_id;
                    }
                }
            }
            else
            {
                thiz->is_got_first_id = 1;
                thiz->readed_id = thiz->recv_id;
            }
            if (is_old)
            {
                break;
            }
            rt_slist_t *idle;
            rt_enter_critical();
            idle = rt_slist_first(&thiz->downlink_decode_idle);
            rt_exit_critical();
            if (idle)
            {
                opus_decode_queue_t *queue = rt_container_of(idle, opus_decode_queue_t, node);
                if (queue->size < thiz->downlink_data_len)
                {
                    rt_free(queue->data);
                    queue->size = thiz->downlink_data_len;
                    queue->data = rt_malloc(queue->size);
                    RT_ASSERT(queue->data);
                }
                queue->data_len = thiz->downlink_data_len;
                memcpy(queue->data, thiz->downlink_data, queue->data_len);

                rt_enter_critical();
                rt_slist_remove(&thiz->downlink_decode_idle, idle);
                rt_slist_append(&thiz->downlink_decode_busy, idle);
                rt_exit_critical();
                if (thiz->is_to_speaker_enable)
                {
                    rt_event_send(thiz->event, OPUS_EVENT_BLE_DOWNLINK);
                }
                else
                {
                    thiz->speaker_cached_packets++;
                    if (thiz->speaker_cached_packets > OPUS_DOWNLINK_QUEUE_NUM / 2)
                    {
                        thiz->is_to_speaker_enable = 1;
                    }
                }
            }
            else
            {
                LOG_I("drop: no recv buf");
            }
            break;
        }
        default:
            RT_ASSERT(0);
        }
    }
}

static void opus_thread_entry(void *p)
{
    ble_talker_t *thiz = (ble_talker_t *)p;
    int err;

    rt_kprintf("opus runing stack var address =0x%x\r\n", &thiz);
    rt_slist_init(&thiz->downlink_decode_idle);
    rt_slist_init(&thiz->downlink_decode_busy);
    for (int i = 0; i < OPUS_DOWNLINK_QUEUE_NUM; i++)
    {
        thiz->downlink_queue[i].size = 32;
        thiz->downlink_queue[i].data = rt_malloc(thiz->downlink_queue[i].size);
        RT_ASSERT(thiz->downlink_queue[i].data);
        rt_slist_append(&thiz->downlink_decode_idle, &thiz->downlink_queue[i].node);
    }

    rt_slist_init(&thiz->encode_slist);
    for (int i = 0; i < OPUS_ENCODE_QUEUE_NUM; i++)
    {
        rt_slist_append(&thiz->encode_slist, &thiz->encode_one[i].slist);
    }
    thiz->state = OPUS_WAIT_ID;
    thiz->downlink_data_pos = 0;
    thiz->rb_opus_decode_input  = rt_ringbuffer_create(160 * 8);
    thiz->rb_opus_encode_input  = rt_ringbuffer_create(640 * 3); // three 20ms frame

    bool dbg = thiz->rb_opus_decode_input
               && thiz->rb_opus_encode_input;

    RT_ASSERT(dbg);

    thiz->event = rt_event_create("opusble", RT_IPC_FLAG_FIFO);
    RT_ASSERT(thiz->event);

    thiz->encoder = opus_encoder_create(OPUS_SAMPLERATE, 1, OPUS_APPLICATION_VOIP, &err);
    RT_ASSERT(thiz->encoder);
#if OPUS_USING_20MS
    opus_encoder_ctl(thiz->encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_20_MS));
#else
    opus_encoder_ctl(thiz->encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
#endif

    opus_encoder_ctl(thiz->encoder, OPUS_SET_VBR(1));
    opus_encoder_ctl(thiz->encoder, OPUS_SET_VBR_CONSTRAINT(1));

    opus_encoder_ctl(thiz->encoder, OPUS_SET_BITRATE(OPUS_SAMPLERATE));
    opus_encoder_ctl(thiz->encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
    opus_encoder_ctl(thiz->encoder, OPUS_SET_COMPLEXITY(0));
    opus_encoder_ctl(thiz->encoder, OPUS_SET_LSB_DEPTH(24));

    opus_encoder_ctl(thiz->encoder, OPUS_SET_DTX(0));
    opus_encoder_ctl(thiz->encoder, OPUS_SET_INBAND_FEC(0));
    opus_encoder_ctl(thiz->encoder, OPUS_SET_PACKET_LOSS_PERC(0));
    opus_encoder_ctl(thiz->encoder, OPUS_SET_PREDICTION_DISABLED(0));

    opus_encoder_ctl(thiz->encoder, OPUS_SET_MAX_BANDWIDTH(OPUS_BANDWIDTH_WIDEBAND));
    opus_encoder_ctl(thiz->encoder, OPUS_SET_BANDWIDTH(OPUS_AUTO));

    thiz->decoder = opus_decoder_create(OPUS_SAMPLERATE, 1, &err);
    RT_ASSERT(thiz->decoder);

    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;
    pa.write_channnel_num = 1;
    pa.write_samplerate = OPUS_SAMPLERATE;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;
    pa.read_samplerate = OPUS_SAMPLERATE;
    pa.read_cache_size = 0;
    pa.write_cache_size = 8192;

#if RECORD_USING_WEBRTC
    webrtc_open();
#endif


    thiz->client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_TXRX, &pa, audio_callback, thiz);

    thiz->is_inited = 1;

    while (!thiz->is_exit)
    {
        rt_uint32_t evt = 0;
        rt_event_recv(thiz->event, OPUS_EVENT_ALL, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        if (evt & OPUS_EVENT_MIC_RX)
        {
            int16_t buf[OPUS_ENCODE_DATA_LEN / 2];
            rt_ringbuffer_get(thiz->rb_opus_encode_input, (uint8_t *)&buf[0], OPUS_ENCODE_DATA_LEN);

            rt_slist_t *first = rt_slist_first(&thiz->encode_slist);
            RT_ASSERT(first);
            rt_slist_remove(&thiz->encode_slist, first);
            rt_slist_append(&thiz->encode_slist, first);
            ble_opus_packet_t *cur = rt_container_of(first, ble_opus_packet_t, slist);
            opus_int32 len = opus_encode(thiz->encoder, (const opus_int16 *)buf, OPUS_ENCODE_DATA_LEN / 2, cur->data, OPUS_ENCODE_DATA_LEN);
            if (len < 0 || len > OPUS_ENCODE_DATA_LEN)
            {
                RT_ASSERT(0);
            }

            cur->id = thiz->send_id++;
            cur->len = (uint8_t)len;
            RT_ASSERT(len < 256);

#if SENDER_DEBUG_MIC2SPEAKER
            audio_write(thiz->client, (uint8_t *)&buf[0], OPUS_ENCODE_DATA_LEN);
#endif
            //LOG_I("id=%d", cur->id);

            if (!thiz->is_first_encoded)
            {
                thiz->is_first_encoded = 1;
                rt_slist_t *head_list;
                ble_opus_packet_t *head;
                for (int i = 0; i < OPUS_ENCODE_QUEUE_NUM - 1; i++)
                {
                    head_list = rt_slist_first(&thiz->encode_slist);
                    RT_ASSERT(head_list);
                    rt_slist_remove(&thiz->encode_slist, head_list);
                    head = rt_container_of(head_list, ble_opus_packet_t, slist);
                    memcpy(head, cur, sizeof(ble_opus_packet_t));
                    rt_slist_append(&thiz->encode_slist, head_list);
                }
            }
            int d = 0, pos = 0;
            rt_slist_t *iter;
            rt_slist_for_each(iter, &thiz->encode_slist)
            {
                ble_opus_packet_t *cur = rt_container_of(iter, ble_opus_packet_t, slist);
                d++;
                thiz->send_data[pos++] = cur->id;
                thiz->send_data[pos++] = cur->len;
                memcpy(&thiz->send_data[pos], cur->data, cur->len);
                pos += cur->len;
            }
            RT_ASSERT(d == OPUS_ENCODE_QUEUE_NUM);
            //LOG_HEX("packet", 16, thiz->send_data, pos);
            app_send_voice_data(pos, thiz->send_data);

            if (rt_ringbuffer_data_len(thiz->rb_opus_encode_input) >= OPUS_ENCODE_DATA_LEN)
            {
                rt_event_send(thiz->event, OPUS_EVENT_MIC_RX);
            }
        }

        if (evt & OPUS_EVENT_SPK_TX)
        {
        }

        if (evt & OPUS_EVENT_BLE_DOWNLINK)
        {
            rt_slist_t *decode;
            rt_enter_critical();
            decode = rt_slist_first(&thiz->downlink_decode_busy);
            rt_exit_critical();
            if (decode)
            {
                opus_decode_queue_t *queue = rt_container_of(decode, opus_decode_queue_t, node);
                opus_int32 res = opus_decode(thiz->decoder, (const uint8_t *)queue->data, queue->data_len, (opus_int16 *)&thiz->downlink_decode_out[0], OPUS_ENCODE_DATA_LEN, 0);
                if (res != OPUS_ENCODE_DATA_LEN / 2)
                {
                    LOG_I("decode out samples=%d\n", res);
                    RT_ASSERT(0);
                }
                int ret = audio_write(thiz->client, thiz->downlink_decode_out, OPUS_ENCODE_DATA_LEN);
                if (ret != OPUS_ENCODE_DATA_LEN)
                {
                    LOG_I("audio write=%d", ret);
                }

                rt_enter_critical();
                rt_slist_remove(&thiz->downlink_decode_busy, decode);
                rt_slist_append(&thiz->downlink_decode_idle, decode);
                rt_exit_critical();
            }
            if (rt_slist_first(&thiz->downlink_decode_busy))
            {
                rt_event_send(thiz->event, OPUS_EVENT_BLE_DOWNLINK);
            }
        }

    }
    thiz->is_inited = 0;
    audio_close(thiz->client);
    rt_ringbuffer_destroy(thiz->rb_opus_decode_input);
    rt_ringbuffer_destroy(thiz->rb_opus_encode_input);

    if (thiz->encoder)
        opus_encoder_destroy(thiz->encoder);

    if (thiz->decoder)
        opus_decoder_destroy(thiz->decoder);

#if RECORD_USING_WEBRTC
    webrtc_close();
#endif

    rt_event_delete(thiz->event);
    for (int i = 0; i < OPUS_DOWNLINK_QUEUE_NUM; i++)
    {
        if (thiz->downlink_queue[i].data)
        {
            rt_free(thiz->downlink_queue[i].data);
            thiz->downlink_queue[i].data = NULL;
        }
    }
    rt_kprintf("---opus test exit---\r\n");
}

int talk_init(audio_rwflag_t flag)
{
    if (g_talker)
    {
        return -1;
    }
    ble_talker_t *thiz;
    g_talker = rt_calloc(1, sizeof(ble_talker_t));
    RT_ASSERT(g_talker);
    thiz = g_talker;
    thiz->flag = flag;
    if (flag & AUDIO_TX)
    {
        thiz->is_rx_enable = 1;
    }
    if (flag & AUDIO_RX)
    {
        thiz->is_tx_enable = 1;
    }

    thiz->is_inited = 0;
    /*
        may need two thread:
            decode thread and encode thread
    */
    thiz->opus_thread = os_thread_create_int(TALK_THREAD_NAME, opus_thread_entry, thiz,
                        g_opus_stack, sizeof(g_opus_stack),
                        OPUS_THREAD_PRIORITY,
                        RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(thiz->opus_thread);
    return 0;
}

int talk_deinit(void)
{
    if (g_talker)
    {
        ble_talker_t *thiz = g_talker;
        thiz->is_exit = 1;
        while (rt_thread_find(TALK_THREAD_NAME))
        {
            LOG_I("wait thread %s eixt", TALK_THREAD_NAME);
            os_delay(100);
        }
        os_thread_delete(thiz->opus_thread);
        /*notice,
            should close ble first, ble_talk_downlink() may use g_talker
         */
        g_talker = NULL;
        rt_free(thiz);
    }
    return 0;
}

static int ble_talk(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "start") == 0)
        {
            if (argc > 2)
            {
                if (!strcmp(argv[2], "rx"))
                {
                    /*rx ble and to local AUDIO_TX*/
                    return talk_init(AUDIO_TX);
                }
                if (!strcmp(argv[2], "tx"))
                {
                    /*local mic AUDIO_RX and tx tol ble*/
                    return talk_init(AUDIO_RX);
                }
            }
            return talk_init(AUDIO_TXRX);
        }
        else if (strcmp(argv[1], "stop") == 0)
        {
            if (!g_talker)
            {
                return 0;
            }

            ble_talker_t *thiz = g_talker;

            LOG_I("!!!!\nis ble closed?\n");
            LOG_I("ble may still call ble_talk_downlink(), may use invalid g_talker");
            if (argc > 2)
            {
                if (!strcmp(argv[2], "rx"))
                {
                    LOG_I("talk disable rx");
                    thiz->is_rx_enable = 0;
                }
                if (!strcmp(argv[2], "tx"))
                {
                    LOG_I("talk disable tx");
                    thiz->is_tx_enable = 0;
                }
            }
            else
            {
                thiz->is_rx_enable = 0;
                thiz->is_tx_enable = 0;
            }
            if (!thiz->is_rx_enable && !thiz->is_tx_enable)
            {
                LOG_I("talk disable txrx");
                return talk_deinit();
            }
        }
        else
        {
            LOG_I("!!!cmd error?\n example: ble_talk start tx\nexample: ble_talk start rx\nble_talk_stop\n");
        }
    }

    return 0;
}
MSH_CMD_EXPORT(ble_talk, ble talk);

#endif

