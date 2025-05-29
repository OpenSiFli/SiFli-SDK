#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <neaacdec.h>
#include <rtthread.h>
#include <stdbool.h>
#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif

#include "audio_server.h"

#define cache_size 4096
static char file_name[64];
static uint8_t *cache;
static uint32_t cache_used = 0;
static uint32_t cache_remain = 0;
static audio_client_t client = NULL;
static int fd;
static int aacplayer(int object_type, int downMatrix)
{
    cache = rt_malloc(cache_size);
    RT_ASSERT(cache);
    char *faad_id_string = NULL;
    char *faad_copyright_string = NULL;
    NeAACDecHandle hDecoder;
    NeAACDecFrameInfo frameInfo;
    NeAACDecConfigurationPtr config;
    unsigned long samplerate;
    unsigned char channels;
    void *sample_buffer;
    int ret;
    cache_remain = cache_size;
    cache_used = 0;
    fd = open(file_name, O_RDONLY | O_BINARY);
    if (fd < 0)
    {
        rt_kprintf("file not found\n");
        return -1;
    }
    read(fd, cache, cache_size);

    if (0 == NeAACDecGetVersion(&faad_id_string, &faad_copyright_string))
    {
        rt_kprintf("version: %s\n", faad_id_string);
        rt_kprintf("%s\n", faad_copyright_string);
    }
    unsigned long cap = NeAACDecGetCapabilities();
    if (cap & FIXED_POINT_CAP)
        rt_kprintf("Fixed point version\n");
    else
        rt_kprintf("Floating point version\n");
    // Check if decoder has the needed capabilities
    // Open the library
    NeAACDecHandle hAac = NeAACDecOpen();
    // Get the current config
    NeAACDecConfigurationPtr conf =
        NeAACDecGetCurrentConfiguration(hAac);
    conf->defObjectType = object_type;
    conf->outputFormat = FAAD_FMT_16BIT;
    conf->downMatrix = downMatrix;
    conf->useOldADTSFormat = 0;
    // If needed change some of the values in conf
    //
    // Set the new configuration
    NeAACDecSetConfiguration(hAac, conf);
    // Initialise the library using one of the initialization functions
    char err = NeAACDecInit(hAac, (unsigned char*)cache, cache_size, &samplerate,
        &channels);
    if (err != 0)
    {
        rt_kprintf("init error\n");
        return -1;
    }
    unsigned long frame_index = 0;
    frameInfo.bytesconsumed = 0;

    rt_tick_t t0 = rt_tick_get_millisecond();

    // Loop until decoding finished
    do {
        sample_buffer = NeAACDecDecode(hAac, &frameInfo, (unsigned char*)cache,  cache_size);

        cache_used = frameInfo.bytesconsumed;
        cache_remain = cache_size - cache_used;

        if ((frameInfo.error == 0) && (frameInfo.samples > 0))
        {
            frame_index++;

            if (!client)
            {
                audio_parameter_t pa = {0};
                pa.write_cache_size = 30000;
                pa.write_channnel_num = frameInfo.channels;
                pa.write_samplerate = frameInfo.samplerate;
                client = audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &pa, NULL, 0);
                RT_ASSERT(client);
            }
            while (1)
            {
                int w = audio_write(client, (uint8_t *)sample_buffer, frameInfo.samples * 2);
                if (w == 0)
                {
                    rt_thread_mdelay(5);
                    continue;
                }
                break;
            }

        }
        else if (frameInfo.error != 0)
        {
            rt_kprintf("decode error at frame %d\n", frame_index);
        }
        memcpy(cache, cache + cache_used, cache_remain);
        int readed = read(fd, cache + cache_remain, cache_used);

        if (readed < cache_used)
        {
            rt_kprintf("file end\n");
            break;
        }
        cache_remain = cache_size;
    } while (sample_buffer != NULL);

Exit:
    NeAACDecClose(hAac);

    rt_tick_t t1 = rt_tick_get_millisecond();
    uint32_t fps = (uint32_t)((uint64_t)frame_index * 10000 / (t1 - t0));
    rt_kprintf("frames=%d, fps = %d.%d\n", frame_index, fps/10, fps%10);
    audio_close(client);
    if (fd >=0)
        close(fd);
    return 0;
}


static void aac_decode(void *p)
{
    aacplayer(LC, 1);

    return;
}
int aactest(int argc, char *argv[])
{
    strncpy(file_name, argv[1], sizeof(file_name) - 1);
    rt_thread_t thread = rt_thread_create("aac", aac_decode, NULL, 64000, 14, 10); //stack < 55K will overflow
    rt_kprintf("thread=%d\n", thread);
    rt_thread_mdelay(100);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    return 0;
}

MSH_CMD_EXPORT(aactest, aac test);
