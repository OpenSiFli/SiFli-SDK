#if 0

#include <rtthread.h>
#define DBG_TAG         "vad"
#define DBG_LVL          LOG_LVL_INFO
#include "log.h"
#include "audio_server.h"
#include "webrtc/common_audio/vad/include/webrtc_vad.h"

static int g_is_voice_start;
static int g_not_voice_times; //every 10ms

#define NOT_VOICE_SECINDS(x)  (1000/10 * x)


static int audio_callback_record(audio_server_callback_cmt_t cmd, void *callback_userdata, uint32_t reserved)
{
    int ret;
    VadInst *handle = (VadInst *)callback_userdata;
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;
        
        RT_ASSERT(p->data_len == 320);
        ret = WebRtcVad_Process(handle, 16000, (int16_t*)p->data, p->data_len/2);
        
        if (ret == 1)
        {
            LOG_I("is voice");
            g_is_voice_start = 1;
            
        }
        else if (ret == 0)
        {
            LOG_I("not voice");
            if (g_is_voice_start)
            {
                g_not_voice_times++;
                if (g_not_voice_times < NOT_VOICE_SECINDS(2))
                {
                    LOG_I("2 sencod not speak after first speaking");
                    g_is_voice_start = 0;
                    g_not_voice_times = 0;
                }
            }
        }
        else
        {
            LOG_I("vad error");
        }
    }
    return 0;
}

static void vadtest(uint8_t argc, char **argv)
{
    uint32_t record_seconds = 0;
    audio_parameter_t pa = {0};
    pa.write_bits_per_sample = 16;
    pa.write_channnel_num = 1;
    pa.write_samplerate = 16000;
    pa.read_bits_per_sample = 16;
    pa.read_channnel_num = 1;
    pa.read_samplerate = 16000;
    pa.read_cache_size = 0;
    pa.write_cache_size = 2048;
    int ret;
    VadInst *handle;
    ret = WebRtcVad_Create(&handle);
    ret = WebRtcVad_Init(handle);
    ret = WebRtcVad_set_mode(handle, 2);

    g_is_voice_start = 0;
    g_not_voice_times = 0;

    audio_client_t client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &pa, audio_callback_record, (void *)handle);
    RT_ASSERT(client);

    while (record_seconds < 50)
    {
        rt_thread_mdelay(1000);
        record_seconds++;
    }
    audio_close(client);
    WebRtcVad_Free(handle);
}

MSH_CMD_EXPORT(vadtest, vadtest);

#endif
