#include <rtthread.h>
#include <rtdevice.h>
#include "bf0_hal.h"
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_font.h"
#ifndef _WIN32
    #include "drv_flash.h"
#endif /* _WIN32 */
#include "dfs_fs.h"
#include "media_dec.h"


LV_IMG_DECLARE(clock_simple_bg);

extern void ffmpeg_heap_init(void);
extern void *ffmpeg_alloc(size_t nbytes);
extern void ffmpeg_free(void *p);
extern void lv_ex_data_pool_init(void);

const char *VIDEO_SRC[] =
{
    "video_example.mp4",
    "video_example1.mjpeg",
    "video_example2.mjpeg",
};

#define VIDEO_SRC_LEN (sizeof(VIDEO_SRC)/sizeof(VIDEO_SRC[0]))
typedef struct
{
    lv_obj_t           *parent;
    ffmpeg_handle       ffmpeg;
    lv_timer_t          *decode_task;
    lv_img_dsc_t        img_desc;
    lv_img_dsc_t        img_desc_clone;
    lv_obj_t           *img;
    lv_obj_t           *filename;
    sifli_gpu_fmt_t     gpu_pic_fmt;
    uint8_t            *yuv[3];
    uint8_t             is_loop;
    uint8_t             volume;
    uint16_t            period;
} video_player_t;

static video_player_t   g_player;
static uint32_t         play_video_index = 0;
static uint32_t         play_end = 1;
static int ffmpeg_notify(uint32_t user_data, ffmpeg_cmd_e cmd, uint32_t val)
{
    (void)user_data; //user_data is arg3 from ffmpeg_open(x, x, user_data)
    (void)val;
    if (cmd == e_ffmpeg_play_to_end)
    {
        //Play next one
        play_video_index = (play_video_index + 1) % VIDEO_SRC_LEN;
        play_end = 1;
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

    if (LV_EVENT_CLICKED == event)
    {
        if (lv_obj_has_flag(g_player.filename, LV_OBJ_FLAG_HIDDEN))
        {
            lv_obj_clear_flag(g_player.filename, LV_OBJ_FLAG_HIDDEN);
            lv_img_set_zoom(img, 256);
        }
        else
        {
            uint32_t width  = g_player.img_desc.header.w;
            uint32_t height = g_player.img_desc.header.h;
            int zoom_w = 256 * LV_HOR_RES_MAX  / width;
            int zoom_h = 256 * LV_VER_RES_MAX / height;

            if (zoom_w > zoom_h)
                lv_img_set_zoom(img, zoom_h);
            else
                lv_img_set_zoom(img, zoom_w);

            lv_obj_align_to(img, NULL, LV_ALIGN_CENTER, 0, 0);
            lv_obj_add_flag(g_player.filename, LV_OBJ_FLAG_HIDDEN);
        }
        lv_obj_invalidate(lv_disp_get_scr_act(NULL));
    }
}


static void video_init_img(video_player_t *p_player, int width, int height)
{
    p_player->img_desc.header.w = width;
    p_player->img_desc.header.h = height;
    if (p_player->gpu_pic_fmt == e_sifli_fmt_ezip)
    {
        p_player->img_desc.header.cf = LV_IMG_CF_RAW;
        p_player->img_desc.header.always_zero = 0;
    }
    else
    {
        p_player->img_desc.data_size = width * height * IMG_PIXEL_SIZE;
        p_player->img_desc.header.cf = IMG_LV_FMT;
        p_player->img_desc.data = ffmpeg_alloc(p_player->img_desc.data_size);
        RT_ASSERT(p_player->img_desc.data != NULL);
        memset((void *)p_player->img_desc.data, 0, p_player->img_desc.data_size);
    }



    memcpy((void *)&p_player->img_desc_clone, (void *)&p_player->img_desc, sizeof(p_player->img_desc));

    p_player->img = lv_img_create(p_player->parent);
    lv_img_set_src(p_player->img, &p_player->img_desc_clone);
    lv_obj_align_to(p_player->img, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(p_player->img, img_event_callback, LV_EVENT_ALL, p_player);
    lv_obj_add_flag(p_player->img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_move_background(p_player->img);
    lv_obj_add_flag(p_player->img, LV_OBJ_FLAG_HIDDEN);//Invalid image, hide it.

}

static void video_decode_task(lv_timer_t *task)
{
    video_player_t *p_player = (video_player_t *)task->user_data;

    if (!ffmpeg_is_video_available(p_player->ffmpeg))
        return;

    if (!p_player->img)
    {
        uint32_t width, height;
        video_info_t info;
        if (ffmpeg_get_video_info(p_player->ffmpeg, &width, &height, &info) == 0)
        {
            p_player->gpu_pic_fmt = info.gpu_pic_fmt;
            video_init_img(p_player, width, height);
            if (info.period == 0)
            {
                info.period = 36; //30fps
            }
            p_player->period = info.period;
            lv_timer_set_period(task, p_player->period / 2);
            {
                //Show video dimension and fps
                char file_info[128];
                snprintf(file_info, sizeof(file_info) - 1,
                         "%s\n(w:%d h:%d period:%d)", VIDEO_SRC[play_video_index],
                         width, height, p_player->period);
            }
            lv_obj_clear_flag(p_player->img, LV_OBJ_FLAG_HIDDEN);
        }
        else
            return;
    }
    int ret;
    if (p_player->gpu_pic_fmt == e_sifli_fmt_ezip)
    {
        uint8_t *frame_data[3];
        ret = ffmpeg_next_video_frame(p_player->ffmpeg, (uint8_t *)frame_data);
        p_player->img_desc.data = (const uint8_t *)frame_data[0];
        p_player->img_desc.data_size = (uint32_t)frame_data[1];
        RT_ASSERT(frame_data[2] == (uint8_t *)IMG_DESC_FMT_EZIP);
        memcpy((void *)&p_player->img_desc_clone, (void *)&p_player->img_desc, sizeof(p_player->img_desc));
    }
    else
    {
        ret = ffmpeg_next_video_frame(p_player->ffmpeg, (uint8_t *)p_player->img_desc.data);
    }
    RT_ASSERT(!ret);

#ifdef CONFIG_MJPEG_DECODER
#ifdef SOC_SF32LB58X
    if (p_player->gpu_pic_fmt == e_sifli_fmt_argb8888)
    {
        //EPIC not support YUV and NANOD return ARGB8888 fromat directly.
        p_player->img_desc_clone.data = *((uint8_t **)p_player->img_desc.data);
    }
#endif /* SOC_SF32LB58X */
#endif /* CONFIG_MJPEG_DECODER */
    lv_img_cache_invalidate_src(&p_player->img_desc);
    lv_img_cache_invalidate_src(&p_player->img_desc_clone);

    lv_obj_invalidate(p_player->img);
}



static void player_init()
{

}

static rt_int32_t player_stop(void)
{
    if (g_player.decode_task)
    {
        lv_timer_del(g_player.decode_task);
        g_player.decode_task = NULL;
        ffmpeg_close(g_player.ffmpeg);
        g_player.ffmpeg = NULL;
    }
    if (g_player.img)
    {
        lv_obj_del(g_player.img);
        g_player.img = NULL;
    }
    if (g_player.img_desc.data
            && g_player.gpu_pic_fmt != e_sifli_fmt_ezip)
    {
        ffmpeg_free((void *)g_player.img_desc.data);
        g_player.img_desc.data = NULL;
    }
    if (g_player.filename)
    {
        lv_obj_del(g_player.filename);
        g_player.filename = NULL;
    }
    return RT_EOK;
}


static rt_int32_t player_start(void)
{
    const char *filename;

    filename = VIDEO_SRC[play_video_index];


    //Create file name lable
    lv_obj_t *parent = lv_disp_get_scr_act(NULL);
    lv_obj_t *filename_label = lv_label_create(parent);
    lv_obj_set_style_bg_color(parent, LV_COLOR_BLACK, 0);
    g_player.parent = parent;
    lv_ext_set_local_font(filename_label, FONT_TITLE, LV_COLOR_WHITE);
    lv_label_set_text(filename_label, filename);
    lv_obj_align(filename_label, LV_ALIGN_BOTTOM_MID, 0, 0);
    g_player.filename = filename_label;




    g_player.volume = 1;
    ffmpeg_config_t cfg = {0};
    cfg.src             = e_src_localfile;
    cfg.fmt             = IMG_DESC_FMT;
    cfg.is_loop         = 0; //loop again if play to end
    cfg.audio_enable    = 1; //play audio if exist
    cfg.video_enable    = 1; //play video if exist
    cfg.file_path       = filename; //cfg.file_path is reused for loop play, filename must be valid until ffmpeg_close(0)
    cfg.mem_malloc      = ffmpeg_alloc;
    cfg.mem_free        = ffmpeg_free;
    cfg.notify          = ffmpeg_notify;

    if (0 == ffmpeg_open(&g_player.ffmpeg, &cfg, (uint32_t)NULL) && NULL == g_player.decode_task)
    {
        g_player.decode_task = lv_timer_create(video_decode_task, 50, (void *)&g_player);
        play_end = 0;
        return RT_EOK;
    }
    else
    {
        //Try next video
        play_video_index = (play_video_index + 1) % VIDEO_SRC_LEN;
        return -RT_ERROR;
    }

}

#ifndef _WIN32
#ifndef FS_REGION_START_ADDR
    #error "Need to define file system start address!"
#endif
int mnt_init(void)
{
    char *name[2];

    rt_kprintf("===auto_mnt_init===\n");

    memset(name, 0, sizeof(name));

#ifdef RT_USING_SDIO
    //Waitting for SD Card detection done.
    int sd_state = mmcsd_wait_cd_changed(3000);
    if (MMCSD_HOST_PLUGED == sd_state)
    {
        rt_kprintf("SD-Card plug in\n");
        name[0] = "sd0";
    }
    else
    {
        rt_kprintf("No SD-Card detected, state: %d\n", sd_state);
    }
#endif /* RT_USING_SDIO */


    name[1] = "flash0";
    register_mtd_device(FS_REGION_START_ADDR, FS_REGION_SIZE, name[1]);



    for (uint32_t i = 0; i < sizeof(name) / sizeof(name[0]); i++)
    {
        if (NULL == name[i]) continue;

        if (dfs_mount(name[i], "/", "elm", 0, 0) == 0) // fs exist
        {
            rt_kprintf("mount fs on %s to root success\n", name[i]);
            break;
        }
        else
        {
            rt_kprintf("mount fs on %s to root fail\n", name[i]);
        }
    }

    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init);
#endif /* _WIN32 */

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_err_t ret = RT_EOK;
    rt_uint32_t ms;

    /* init littlevGL */
    ret = littlevgl2rtt_init("lcd");
    if (ret != RT_EOK)
    {
        return ret;
    }
    lv_ex_data_pool_init();
    ffmpeg_heap_init();
    player_init();
    set_display_fps_and_cpu_load(1);
    player_start();
    while (1)
    {
        if (play_end)
        {
            player_stop();
            player_start();
        }
        ms = lv_task_handler();
        rt_thread_mdelay(ms);
    }
    return RT_EOK;

}

