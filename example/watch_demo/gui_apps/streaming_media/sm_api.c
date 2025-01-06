#include "sm_internal.h"
#include "sm_api.h"

#define MEDIAPLAYER_USING_ACPU  0

#define DEBUG_FFMPEG_FPS        1
#define USING_FAST_YUV2RGB      1


static sm_device_info_t devinfo;
static char *device_token_ptr = RT_NULL;
static media_queue_t *queue_ptr;
static rt_tick_t g_last_tick;
static uint64_t g_last_bytes;
sm_device_info_t *sm_get_device_info(void)
{
    return &devinfo;
}
#define MAX_TOKEN_LEN   80
int streaming_media_start(char *app_name, char *app_id, int fmt)
{
    int order_id = 7903053;
    int ret = 0;
    LOG_I("appid=[%s], token=[%s]", app_id, app_name);
    if (app_name && strlen(app_name) > 0)
    {
        if (device_token_ptr)
        {
            free(device_token_ptr);
            device_token_ptr = RT_NULL;
        }
        sm_net_apply_for_device(app_name, &device_token_ptr);
        if (!device_token_ptr || strlen(device_token_ptr) == 0)
        {
            LOG_I("token err: %s", app_name);
            return -1;
        }
    }
    else
    {
        return -1;
    }

    if (!device_token_ptr)
    {
        device_token_ptr = calloc(MAX_TOKEN_LEN + 1, 1);
        RT_ASSERT(device_token_ptr);
        strncpy(device_token_ptr, app_name, MAX_TOKEN_LEN);
    }

    queue_ptr = media_queue_open(sm_packet_free);

    strcpy(devinfo.app_id, app_id);
    ret = sm_net_order(order_id, app_id, device_token_ptr);

    if (devinfo.videowidth > 240 || devinfo.videoheight > 320)
    {
        rt_kprintf("unsurport size %dx%d\n", devinfo.videowidth, devinfo.videoheight);
        return -1;
    }

    //sm_net_reset_device(app_name,device_token_ptr,1);

    if (ret == 0)
    {
        sm_ctrl_create(0);
        sm_send_heart(true);
        g_last_tick = rt_tick_get_millisecond();
        g_last_bytes = 0;
        sm_download_open(queue_ptr);
        streaming_callback_t callbacks = {0};
        callbacks.mem_malloc = ffmpeg_alloc;
        callbacks.mem_free = ffmpeg_free;
        callbacks.notify = NULL;
        sm_decode_start(queue_ptr, fmt, &callbacks);
        return 0;
    }

    return ret;

}

void streaming_media_stop(void)
{
    sm_download_close();

    sm_decode_stop();

    sm_ctrl_destry();

    media_queue_close(queue_ptr);
    queue_ptr = NULL;

    if (device_token_ptr)
    {
        free(device_token_ptr);
        device_token_ptr = RT_NULL;
    }
}
extern void sm_debug_get_decode_info(uint32_t *dv, uint32_t *da);
uint32_t sm_debug_get_info(uint32_t *v, uint32_t *a, uint32_t *dv, uint32_t *da)
{
    uint32_t ret = 0;
    uint64_t bytes;
    rt_tick_t now = rt_tick_get_millisecond();
    if (queue_ptr)
    {
        *v = queue_ptr->debug_v - queue_ptr->debug_last_v;
        *a = queue_ptr->debug_a - queue_ptr->debug_last_a;
        queue_ptr->debug_last_v = queue_ptr->debug_v;
        queue_ptr->debug_last_a = queue_ptr->debug_a;
        sm_debug_get_decode_info(dv, da);
        bytes = queue_ptr->bytes_download;
        if (now != g_last_tick)
        {
            float kbits = (double)(bytes - g_last_bytes) * 8.0f / (now - g_last_tick);
            g_last_tick = now;
            ret = (uint32_t)kbits;
            g_last_bytes = bytes;
        }
    }
    return ret;
}



