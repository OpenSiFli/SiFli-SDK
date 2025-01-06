#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "gui_app_fwk.h"

#include "app_clock_main.h"
#include "app_mem.h"


#ifdef PKG_USING_FFMPEG
#include "media_dec.h"

LV_IMG_DECLARE(clock_simple_bg);

extern void *ffmpeg_alloc(size_t nbytes);
extern void ffmpeg_free(void *p);


#ifndef MIN
    #define MIN(x,y) (((x)<(y))?(x):(y))
#endif


#define CLOCK_VIDEO_FULL_SCREEN_OVERLAP 1


const char *VIDEO_SRC[] =
{
#ifdef CONFIG_MJPEG_DECODER
    "mjpeg.mp4",
    "mjpeg2.mp4",
#endif /* CONFIG_MJPEG_DECODER */
#ifdef CONFIG_FFMPEG_HTTP
    //"http://113.204.105.154:19000/ota-file/video_example.mp4",
#endif
    "video_example.mp4",
    //"ezip.avi",
};

#define VIDEO_SRC_LEN  (sizeof(VIDEO_SRC)/sizeof(VIDEO_SRC[0]))

typedef struct
{
    lv_obj_t *time_txt;
    lv_obj_t *filename;
    app_clock_time_t time;
} clock_video_calendar_t;

typedef struct
{
    lv_obj_t *conditions_icons;
    lv_obj_t *average_txt;

} clock_video_weather_t;

typedef struct
{
    lv_obj_t           *parent;
    ffmpeg_handle       ffmpeg;
    lv_timer_t          *decode_task;
    lv_img_dsc_t        img_desc;
    lv_img_dsc_t        img_desc_clone;
    lv_obj_t           *img;
    uint8_t            *yuv[3];
    uint8_t             is_loop;
    uint8_t             volume;
    sifli_gpu_fmt_t     gpu_pic_fmt;
    uint16_t            period;
} clock_video_player_t;

typedef struct
{
    lv_timer_t *refresh_task;
    clock_video_calendar_t calendar;
    clock_video_player_t video_player;
    lv_obj_t *bg_img;
} clock_video_t;


static clock_video_t *p_clk_video = NULL;
static uint32_t play_video_index = 0;

static int ffmpeg_notify(uint32_t user_data, ffmpeg_cmd_e cmd, uint32_t val)
{
    (void)user_data; //user_data is arg3 from ffmpeg_open(x, x, user_data)
    (void)val;
    if (cmd == e_ffmpeg_play_to_end)
    {
        rt_kprintf("play end\n");
    }
    else if (cmd == e_ffmpeg_suspended)
    {
        rt_kprintf("play suspened\n");
    }
    else if (e_ffmpeg_resumed)
    {
        rt_kprintf("play resumed\n");
    }
    return 0;
}

static void img_event_callback(lv_event_t *e)
{
    lv_obj_t *img = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

#ifndef DISABLE_LVGL_V8

    if (LV_EVENT_CLICKED == event)
    {
        if (lv_obj_has_flag(p_clk_video->calendar.time_txt, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(p_clk_video->calendar.time_txt, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(p_clk_video->calendar.filename, LV_OBJ_FLAG_HIDDEN);

            lv_img_set_zoom(img, 256);
        }
        else
        {
#if CLOCK_VIDEO_FULL_SCREEN_OVERLAP
            uint32_t width  = p_clk_video->video_player.img_desc.header.w;
            uint32_t height = p_clk_video->video_player.img_desc.header.h;


            int zoom_w = 256 * LV_HOR_RES_MAX  / width;
            int zoom_h = 256 * LV_VER_RES_MAX / height;

            if (zoom_w > zoom_h)
                lv_img_set_zoom(img, zoom_h);
            else
                lv_img_set_zoom(img, zoom_w);

            lv_obj_align_to(img, NULL, LV_ALIGN_CENTER, 0, 0);
#endif

            lv_obj_add_flag(p_clk_video->calendar.time_txt, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(p_clk_video->calendar.filename, LV_OBJ_FLAG_HIDDEN);
        }

    }
#endif /* DISABLE_LVGL_V8 */

}


static void video_init_img(clock_video_player_t *player, int width, int height)
{
    player->img_desc.header.w = width;
    player->img_desc.header.h = height;

    if (player->gpu_pic_fmt == e_sifli_fmt_ezip)
    {
        player->img_desc.header.cf = LV_IMG_CF_RAW;
        player->img_desc.header.always_zero = 0;
    }
    else
    {
        player->img_desc.data_size = width * height * IMG_PIXEL_SIZE;
        player->img_desc.header.cf = IMG_LV_FMT;
        player->img_desc.data = ffmpeg_alloc(player->img_desc.data_size);
        RT_ASSERT(player->img_desc.data);
        memset((void *)player->img_desc.data, 0, player->img_desc.data_size);
    }
    memcpy((void *)&player->img_desc_clone, (void *)&player->img_desc, sizeof(player->img_desc));

    player->img = lv_img_create(player->parent);
    lv_img_set_src(player->img, &player->img_desc_clone);
    lv_obj_align_to(player->img, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(player->img, img_event_callback, LV_EVENT_ALL, player);
    lv_obj_add_flag(player->img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_move_background(player->img);
    lv_obj_add_flag(player->img, LV_OBJ_FLAG_HIDDEN);//Invalid image, hide it.

}

static void video_decode_task(lv_timer_t *task)
{
    bool first_time = false;
    clock_video_player_t *player = (clock_video_player_t *)task->user_data;

    if (!ffmpeg_is_video_available(player->ffmpeg))
    {
        return;
    }

    if (!player->img)
    {
        uint32_t width, height;
        video_info_t info;
        if (ffmpeg_get_video_info(player->ffmpeg, &width, &height, &info) == 0)
        {
            player->gpu_pic_fmt = info.gpu_pic_fmt;
            video_init_img(player, width, height);
            if (info.period == 0)
            {
                info.period = 36; //30fps
            }
            player->period = info.period;
            lv_timer_set_period(task, player->period / 2);
            {
                //Show video dimension and fps
                char file_info[128];
                snprintf(file_info, sizeof(file_info) - 1,
                         "%s\n(w:%d h:%d period:%d)", VIDEO_SRC[play_video_index],
                         width, height, player->period);

                lv_label_set_text(p_clk_video->calendar.filename, file_info);
            }
            lv_obj_clear_flag(player->img, LV_OBJ_FLAG_HIDDEN);
            if (p_clk_video->bg_img)
                lv_obj_add_flag(p_clk_video->bg_img, LV_OBJ_FLAG_HIDDEN);

            first_time = true;
        }
        else
            return;
    }
    int ret;
    if (player->gpu_pic_fmt == e_sifli_fmt_ezip)
    {
        uint8_t *frame_data[3];
        ret = ffmpeg_next_video_frame(player->ffmpeg, (uint8_t *)frame_data);
        player->img_desc.data = (const uint8_t *)frame_data[0];
        player->img_desc.data_size = (uint32_t)frame_data[1];
        RT_ASSERT(frame_data[2] == (uint8_t *)IMG_DESC_FMT_EZIP);
        memcpy((void *)&player->img_desc_clone, (void *)&player->img_desc, sizeof(player->img_desc));
    }
    else
    {
        ret = ffmpeg_next_video_frame(player->ffmpeg, (uint8_t *)player->img_desc.data);
    }
    RT_ASSERT(!ret);

#ifdef CONFIG_MJPEG_DECODER
#ifdef SOC_SF32LB58X
    if (player->gpu_pic_fmt == e_sifli_fmt_argb8888)
    {
        //EPIC not support YUV and NANOD return ARGB8888 fromat directly.
        player->img_desc_clone.data = *((uint8_t **)player->img_desc.data);
    }
#endif /* SOC_SF32LB58X */
#endif /* CONFIG_MJPEG_DECODER */
    lv_img_cache_invalidate_src(&player->img_desc);
    lv_img_cache_invalidate_src(&player->img_desc_clone);

    lv_obj_invalidate(player->img);


    if (first_time)
    {
        lv_event_send(player->img, LV_EVENT_CLICKED, NULL);//Show video full screen
    }
}



static void clock_video_time_info_tick(void)
{
    app_clock_time_t current_time;

    app_clock_main_get_current_time(&current_time);


    if ((p_clk_video->calendar.time.s != current_time.s) || \
            (p_clk_video->calendar.time.m != current_time.m) || \
            (p_clk_video->calendar.time.h != current_time.h))
    {
        p_clk_video->calendar.time = current_time;

        char time_str[32];
        snprintf(time_str, sizeof(time_str) - 1, "%02d:%02d:%02d", current_time.h, current_time.m, current_time.s);

        lv_label_set_text(p_clk_video->calendar.time_txt, time_str);
    }
}


static void clock_video_page_refresh(lv_timer_t *task)
{
    clock_video_time_info_tick();
}

static int16_t clock_video_draw_video_area(lv_obj_t *parent, clock_video_player_t *player)
{
    player->parent = parent;

    return 0;
}


static lv_obj_t *clock_video_draw_time_area(lv_obj_t *parent)
{
    lv_obj_t *timer_txt = lv_label_create(parent);
    lv_ext_set_local_font(timer_txt, FONT_BIGL, LV_COLOR_WHITE);
    lv_label_set_text(timer_txt, "00:00");
    lv_obj_align_to(timer_txt, parent, LV_ALIGN_TOP_LEFT, 40, 10);
    p_clk_video->calendar.time_txt = timer_txt;


    lv_obj_t *filename = lv_label_create(parent);
    lv_ext_set_local_font(filename, FONT_TITLE, LV_COLOR_WHITE);
    lv_label_set_text(filename, VIDEO_SRC[play_video_index]);
    lv_obj_align_to(filename, timer_txt, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
    p_clk_video->calendar.filename = filename;

    return timer_txt;
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

    clock_video_draw_video_area(parent, &p_clk_video->video_player);

    clock_video_draw_time_area(parent);

}

static rt_int32_t on_init(lv_obj_t *parent)
{
    p_clk_video = (clock_video_t *) rt_malloc(sizeof(clock_video_t));
    RT_ASSERT(p_clk_video);
    rt_memset(p_clk_video, 0, sizeof(*p_clk_video));

    clock_video_page_init(parent);

    return RT_EOK;
}


static rt_int32_t on_pause(void)
{
    if (p_clk_video)
    {
        if (p_clk_video->video_player.decode_task)
        {
            lv_timer_del(p_clk_video->video_player.decode_task);
            p_clk_video->video_player.decode_task = NULL;
            ffmpeg_close(p_clk_video->video_player.ffmpeg);
            p_clk_video->video_player.ffmpeg = NULL;
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
        if (p_clk_video->video_player.img_desc.data
                && p_clk_video->video_player.gpu_pic_fmt != e_sifli_fmt_ezip)
        {
            ffmpeg_free((void *)p_clk_video->video_player.img_desc.data);
            p_clk_video->video_player.img_desc.data = NULL;
        }
        if (p_clk_video->bg_img)
            lv_obj_clear_flag(p_clk_video->bg_img, LV_OBJ_FLAG_HIDDEN);
        lv_img_cache_invalidate_src(NULL);
    }

    return RT_EOK;

}


static rt_int32_t on_resume(void)
{
    const char *filename;

    filename = VIDEO_SRC[play_video_index];

    p_clk_video->video_player.volume = 1;
    ffmpeg_config_t cfg = {0};
    cfg.src             = e_src_localfile;
    cfg.fmt             = IMG_DESC_FMT;
    cfg.is_loop         = 1; //loop again if play to end
    cfg.audio_enable    = 1; //play audio if exist
    cfg.video_enable    = 1; //play video if exist
    cfg.file_path       = filename; //cfg.file_path is reused for loop play, filename must be valid until ffmpeg_close(0)
    cfg.mem_malloc      = ffmpeg_alloc;
    cfg.mem_free        = ffmpeg_free;
    cfg.notify          = ffmpeg_notify;

    if (0 == ffmpeg_open(&p_clk_video->video_player.ffmpeg, &cfg, (uint32_t)NULL))
    {
        if (NULL == p_clk_video->video_player.decode_task)
        {
            p_clk_video->video_player.decode_task = lv_timer_create(video_decode_task, 50, (void *)&p_clk_video->video_player);
        }
    }

    if (NULL == p_clk_video->refresh_task)
    {
        p_clk_video->refresh_task = lv_timer_create(clock_video_page_refresh, 1000, (void *)0);
    }
    clock_video_page_refresh(NULL);

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
    play_video_index = (play_video_index + 1) % VIDEO_SRC_LEN;

    return RT_EOK;
}


static const app_clock_ops_t ops =
{
    .init   = on_init,
    .pause  = on_pause,
    .resume = on_resume,
    .deinit = on_deinit,
};

void app_clock_video_audio_register(void)
{
    app_clock_register("video", &ops);
}



#endif /* PKG_USING_FFMPEG */
