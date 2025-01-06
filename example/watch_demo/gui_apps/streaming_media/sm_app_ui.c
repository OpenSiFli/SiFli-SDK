#include "sm_api.h"

#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"

#include "app_clock_main.h"
#include "app_mem.h"

#define VIDEO_FPS       15
#define DECODE_TASK_PERIOD  1000/VIDEO_FPS/2

#ifndef MIN
    #define MIN(x,y) (((x)<(y))?(x):(y))
#endif

LV_IMG_DECLARE(clock_simple_bg);

extern void *ffmpeg_alloc(size_t nbytes);
extern void ffmpeg_free(void *p);

#define APP_DOUYIN    "com.ss.android.ugc.aweme"

#define TOKEN_SINAWEIBO "4C45C5BC89324F5380E8D1217C2536ECBB6D625152884FF5B455749F3816A919"
#define TOKEN_DOUYIN    "D84718E0962C48FC8D574C9D583A40541D5F1FAD0A6B448CABA946E432B6FD6D"
#define TOKEN_BILILI    "A02926EFDDC1426F9B6459EEDD78D4F19324A7B7A95D4050A9025609CC2FB703"
#define WEB_APP_ID      "1F73C8287BCED97D"

#define CLOCK_VIDEO_FULL_SCREEN_OVERLAP 1

typedef struct
{
    lv_obj_t           *parent;
    lv_timer_t         *decode_task;
    lv_img_dsc_t        img_desc;
    lv_obj_t           *img;
    uint8_t             is_loop;
    uint8_t             volume;
    uint16_t            period;
    uint16_t            real_img_w;
    uint16_t            real_img_h;
} clock_video_player_t;

typedef struct
{
    lv_timer_t *refresh_task;
    clock_video_player_t video_player;
    lv_obj_t *bg_img;
    bool      is_bg_img_hidden;
    bool      is_img_hidden;
} clock_video_t;


static clock_video_t *p_clk_video = NULL;
static int g_task_run_times;
static int g_debug;

static const lv_area_t titok_invalid_area[] =
{
    {319, 109, 388, 375},
};

static bool is_in_valid_area(lv_point_t now)
{
#if 0
    /*check for titok*/
    for (int i = 0; i < sizeof(titok_invalid_area) / sizeof(titok_invalid_area[0]); i++)
    {
        const lv_area_t *p_area = &titok_invalid_area[i];
        if (now.x > p_area->x1
                && now.x < p_area->x2
                && now.y > p_area->y1
                && now.y < p_area->y2)
        {
            LOG_I("tp invalid\n");
            return false;
        }
    }
#endif
    return true;
}
static void ui_event_cb(lv_event_t *e)
{
    if (!p_clk_video)
        return;
    clock_video_player_t *player = &p_clk_video->video_player;
    lv_point_t original_point;
    lv_indev_t *indev = lv_indev_get_act();
    lv_indev_get_point(indev, &original_point);
    lv_event_code_t event = lv_event_get_code(e);

    uint32_t x_offset = (LV_HOR_RES_MAX - player->real_img_w) / 2;
    uint32_t y_offset = (LV_VER_RES_MAX - player->real_img_h) / 2;

    if (original_point.x < x_offset)
    {
        original_point.x = x_offset;
    }
    if (original_point.y < y_offset)
    {
        original_point.y = y_offset;
    }

    if (LV_EVENT_PRESSED == event)
    {
        if (!is_in_valid_area(original_point))
        {
            return;
        }
        sm_ctrl_send_tp(SM_TOUCH_PRESS, (float)(original_point.x - x_offset) / player->real_img_w, (float)(original_point.y - y_offset) / player->real_img_h);
    }
    else if (LV_EVENT_PRESSING == event)
    {
        sm_ctrl_send_tp(SM_TOUCH_PRESSING, (float)(original_point.x - x_offset) / player->real_img_w, (float)(original_point.y - y_offset) / player->real_img_h);
    }
    else if (LV_EVENT_RELEASED == event || LV_EVENT_DEFOCUSED == event)
    {
        if (!is_in_valid_area(original_point))
        {
            return;
        }
        sm_ctrl_send_tp(SM_TOUCH_RELEASE, (float)(original_point.x - x_offset) / player->real_img_w, (float)(original_point.y - y_offset) / player->real_img_h);
    }

}


/*
    call this if want to delete pop up window in media stream
*/
void back_key_event_cb()
{
    sm_ctrl_send_tp(SM_BACK, 0, 0);
}


static void video_init_img(clock_video_player_t *player, int width, int height)
{
    player->img_desc.data_size = width * height * IMG_PIXEL_SIZE;
    player->img_desc.header.cf = IMG_LV_FMT;
    player->img_desc.header.w = width;
    player->img_desc.header.h = height;
    if (LV_IMG_CF_YUV420_PLANAR2 == IMG_LV_FMT)
    {
        player->img_desc.data_size = sizeof(uint8_t *) * 3;
    }
    player->img_desc.data = sm_packet_malloc(player->img_desc.data_size);
    memset((void *)player->img_desc.data, 0, player->img_desc.data_size);

    player->img = lv_img_create(player->parent);
    lv_img_set_src(player->img, &player->img_desc);
    lv_obj_align_to(player->img, NULL, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_event_cb(player->img, ui_event_cb, LV_EVENT_ALL, player);
    lv_obj_add_flag(player->img, LV_OBJ_FLAG_CLICKABLE);

#if CLOCK_VIDEO_FULL_SCREEN_OVERLAP
    int zoom_w = 256 * MIN(LV_HOR_RES_MAX, width * 2)  / width;
    int zoom_h = 256 * MIN(LV_VER_RES_MAX, height * 2) / height;

    if (zoom_w > zoom_h)
    {
        lv_img_set_zoom(player->img, zoom_h);
        player->real_img_w = width * zoom_h / 256;
        player->real_img_h = height * zoom_h / 256;
    }
    else
    {
        lv_img_set_zoom(player->img, zoom_w);
        player->real_img_w = width * zoom_w / 256;
        player->real_img_h = height * zoom_w / 256;
    }
#endif

    //lv_obj_move_background(player->img);
    //lv_obj_add_flag(player->img, LV_OBJ_FLAG_HIDDEN);
    //p_clk_video->is_img_hidden = true;
}

static void video_decode_task(lv_timer_t *task)
{
    int width, height;
    uint8_t *data;
    clock_video_player_t *player = (clock_video_player_t *)task->user_data;

    if ((g_debug & 0xFF) == 0)
    {
        extern float lv_get_fps(void);
        extern float cpu_get_usage(void);

        float perf_fps = lv_get_fps();
        float perf_cpu = cpu_get_usage();
        uint32_t net_v, net_a, decode_v, decode_a;
        uint32_t bits = sm_debug_get_info(&net_v, &net_a, &decode_v, &decode_a);

        LOG_I("fps=%.1f,cpu=%.1f,net[kbits=%d v=%d,a=%d],decode[v=%d,a=%d,unused=%dk]",
              perf_fps,
              perf_cpu,
              bits,
              net_v,
              net_a,
              decode_v,
              decode_a,
              (uint32_t)(sm_cache_unused() / 1000));
    }
    g_debug++;

    g_task_run_times++;

    if (DECODE_TASK_PERIOD * g_task_run_times > (1 * 60 * 1000))
    {
        g_task_run_times = 0;
        LOG_I("need send heart");
        sm_send_heart(false);
    }
    if (!player->img)
    {
        if (sm_decode_dim(&width, &height) == RT_EOK)
            video_init_img(player, width, height);
        else
            return;
    }
    if (player->img && RT_EOK == sm_video_get((uint8_t *)player->img_desc.data))
    {
        if (p_clk_video->is_img_hidden)
        {
            lv_obj_clear_flag(player->img, LV_OBJ_FLAG_HIDDEN);
            p_clk_video->is_img_hidden = false;
        }
        lv_img_cache_invalidate_src(&player->img_desc);

        lv_obj_invalidate(player->img);
        if (p_clk_video->bg_img && !p_clk_video->is_bg_img_hidden)
        {
            lv_obj_add_flag(p_clk_video->bg_img, LV_OBJ_FLAG_HIDDEN);
            p_clk_video->is_bg_img_hidden = true;
        }
    }
}


static int16_t clock_video_draw_video_area(lv_obj_t *parent, clock_video_player_t *player)
{
    player->parent = parent;

    return 0;
}

static void clock_video_page_init(lv_obj_t *parent)
{
    lv_obj_t *bg_img = lv_img_create(parent);

    const lv_img_dsc_t *p_src = LV_EXT_IMG_GET(clock_simple_bg);
    int zoom_w = 256 * MIN(LV_HOR_RES_MAX, p_src->header.w * 2)  / p_src->header.w;
    int zoom_h = 256 * MIN(LV_VER_RES_MAX, p_src->header.h * 2) / p_src->header.h;

    if (zoom_w > zoom_h)
    {
        lv_img_set_zoom(bg_img, zoom_h);
    }
    else
    {
        lv_img_set_zoom(bg_img, zoom_w);
    }

    lv_img_set_src(bg_img, p_src);
    lv_obj_align_to(bg_img, parent, LV_ALIGN_CENTER, 0, 0);
    p_clk_video->bg_img = bg_img;
    p_clk_video->is_bg_img_hidden = false;
    clock_video_draw_video_area(parent, &p_clk_video->video_player);

}

static rt_int32_t on_init(lv_obj_t *parent)
{
    p_clk_video = (clock_video_t *) rt_malloc(sizeof(clock_video_t));
    RT_ASSERT(p_clk_video);
    rt_memset(p_clk_video, 0, sizeof(*p_clk_video));

// Boost HCPU to run at higher freqency.
#if defined(SF32LB56X)
    HAL_RCC_HCPU_ConfigHCLK(312);
#endif
    clock_video_page_init(parent);
    //lv_obj_data_subscribe(parent, WIFI_APP_STREAM_MEDIA_CONNECT_FAIL, stream_video_connect_datasubs_cb);

    return RT_EOK;
}


static rt_int32_t on_pause(void)
{
    if (p_clk_video)
    {
        if (p_clk_video->video_player.decode_task)
        {
            lv_timer_del(p_clk_video->video_player.decode_task);
            //lv_task_set_prio(p_stream_app_data->delay_task, LV_TASK_PRIO_OFF);
            p_clk_video->video_player.decode_task = NULL;
            streaming_media_stop();
        }
        if (p_clk_video->refresh_task)
        {
            lv_timer_del(p_clk_video->refresh_task);
            p_clk_video->refresh_task = NULL;
        }
        if (p_clk_video->video_player.img)
        {
            lv_obj_del(p_clk_video->video_player.img);
            p_clk_video->video_player.img = NULL;
        }
        if (p_clk_video->video_player.img_desc.data)
        {
            sm_packet_free((void *)p_clk_video->video_player.img_desc.data);
            p_clk_video->video_player.img_desc.data = NULL;
        }
        if (p_clk_video->bg_img)
        {
            lv_obj_clear_flag(p_clk_video->bg_img, LV_OBJ_FLAG_HIDDEN);
            p_clk_video->is_bg_img_hidden = false;
        }
        lv_img_cache_invalidate_src(NULL);
    }

    return RT_EOK;

}


static rt_int32_t on_resume(void)
{
    g_task_run_times = 0;
    g_debug = 0;
    p_clk_video->video_player.volume = 10;
    set_video_order_config(240, 320, 600000);
    if (RT_EOK == streaming_media_start(APP_DOUYIN, WEB_APP_ID, IMG_DESC_FMT))
    {
        if (NULL == p_clk_video->video_player.decode_task)
        {
            p_clk_video->video_player.decode_task = lv_timer_create(video_decode_task, DECODE_TASK_PERIOD, (void *)&p_clk_video->video_player);
        }
    }

    return RT_EOK;

}


static rt_int32_t on_deinit(void)
{
    if (p_clk_video)
    {
        if (p_clk_video->refresh_task)
        {
            lv_timer_del(p_clk_video->refresh_task);
            p_clk_video->refresh_task = NULL;
        }

        rt_free(p_clk_video);
        p_clk_video = NULL;
    }
#if defined(SF32LB56X)
    HAL_RCC_HCPU_ConfigHCLK(240);
#endif

    return RT_EOK;
}


static const app_clock_ops_t ops =
{
    .init   = on_init,
    .pause  = on_pause,
    .resume = on_resume,
    .deinit = on_deinit,
};

void app_clock_streamingmedia_register(void)
{
    app_clock_register("streammedia", &ops);
}





