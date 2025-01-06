/**
 * @file lvsf_rlottie.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "rtconfig.h"
#include "lvgl.h"
#if PKG_USING_RLOTTIE != 0

#include "lvsf_rlottie.h"
#include <rlottie_capi.h>
#include "app_mem.h"
#ifdef RT_USING_DFS
    #include "dfs_posix.h"
#endif
#ifdef WIN32
    #define open rt_open
    #define close rt_close
    #define read rt_read
    #define write rt_write
    #define lseek rt_lseek
#endif

#ifdef DISABLE_LVGL_V8
    extern uint32_t lv_img_buf_get_img_size(lv_coord_t w, lv_coord_t h, lv_img_cf_t cf);
#endif
// rlottie internal is using ARGB
#define ARGB888_PIXEL_SIZE 4

/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_rlottie_class

/**********************
 *      TYPEDEFS
 **********************/
typedef struct
{
    lv_img_t img;
    Lottie_Animation *animation;
    lv_timer_t *task;
    lv_img_dsc_t imgdsc;
    size_t total_frames;
    size_t current_frame;
    size_t framerate;
    uint32_t *allocated_buf;
    size_t allocated_buffer_size;
    size_t scanline_width;
    void *file_data;
} lvsf_rlottie_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_rlottie_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void lv_rlottie_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj);
static void next_frame_task_cb(lv_timer_t *t);

/**********************
 *  STATIC VARIABLES
 **********************/
const lv_obj_class_t lv_rlottie_class =
{
    .constructor_cb = lv_rlottie_constructor,
    .destructor_cb = lv_rlottie_destructor,
    .instance_size = sizeof(lvsf_rlottie_t),
    .base_class = &lv_img_class
};

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

static void common_rlottie_setup(lvsf_rlottie_t *ext, lv_obj_t *parent)
{
    lv_obj_update_layout((const lv_obj_t *) ext);
    ext->total_frames = lottie_animation_get_totalframe(ext->animation);
    ext->framerate = lottie_animation_get_framerate(ext->animation);
    ext->current_frame = 0;

    lv_coord_t obj_width = lv_obj_get_width(parent);
    lv_coord_t obj_height = lv_obj_get_height(parent);

    ext->scanline_width = obj_width * ARGB888_PIXEL_SIZE;
    size_t allocaled_buf_size = (obj_width * obj_height * ARGB888_PIXEL_SIZE);
    ext->allocated_buf = app_cache_alloc(allocaled_buf_size, IMAGE_CACHE_PSRAM);
    if (ext->allocated_buf != NULL)
    {
        ext->allocated_buffer_size = allocaled_buf_size;
        memset(ext->allocated_buf, 0, allocaled_buf_size);
    }

    ext->imgdsc.header.always_zero = 0;
    ext->imgdsc.header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA;
    ext->imgdsc.header.h = obj_height;
    ext->imgdsc.header.w = obj_width;
    ext->imgdsc.data = (const uint8_t *)ext->allocated_buf;
    ext->imgdsc.data_size = lv_img_buf_get_img_size(ext->imgdsc.header.w, ext->imgdsc.header.h, ext->imgdsc.header.cf);

    lv_img_set_src((lv_obj_t *)ext, &ext->imgdsc);

    int period = (int)1000.0 / ext->framerate;
}

lv_obj_t *lv_rlottie_create(lv_obj_t *parent)
{
    lv_obj_t *obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    return obj;
}

int lv_rlottie_file(lv_obj_t *lottie, const char *path)
{
    int r = -RT_ERROR;

#ifdef RT_USING_DFS
    lvsf_rlottie_t *ext = (lvsf_rlottie_t *)lottie;

    int fd = open(path, O_RDONLY);

    if (fd > 0)
    {
        {
            struct stat stat;
            dfs_file_stat(path, &stat);
            if (ext->file_data)
                app_cache_free(ext->file_data);
            ext->file_data = app_cache_alloc(stat.st_size, IMAGE_CACHE_PSRAM);
            if (ext->file_data)
                read(fd, ext->file_data, stat.st_size);
            close(fd);
        }
        if (ext->file_data)
        {
            r = lv_rlottie_raw(lottie, ext->file_data);
            if (r != RT_EOK)
            {
                app_cache_free(ext->file_data);
                ext->file_data = NULL;
            }
        }
    }
#endif
    return r;
}

int lv_rlottie_raw(lv_obj_t *lottie, const char *rlottie_desc)
{
    lvsf_rlottie_t *ext = (lvsf_rlottie_t *)lottie;


#ifdef LOTTIE_JSON_SUPPORT
    ext->animation = lottie_animation_from_rodata(rlottie_desc, strlen(rlottie_desc), "");
#else
    ext->animation = lottie_animation_from_data(rlottie_desc, rlottie_desc, "");
#endif
    if (ext->animation == NULL) return -RT_ERROR;
    common_rlottie_setup(ext,  lottie);
    return RT_EOK;
}

int lv_rlottie_play(lv_obj_t *lottie, int enable)
{
    lvsf_rlottie_t *ext = (lvsf_rlottie_t *)lottie;

    if (enable && ext->task == NULL && ext->animation)
    {
        int period = (int)1000.0 / ext->framerate;
        ext->task = lv_timer_create(next_frame_task_cb, period, ext);
    }
    else if (enable == 0 && ext->task)
    {
        lv_timer_del(ext->task);
        ext->task = NULL;
    }
    else
        return -RT_ERROR;
    return RT_EOK;
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_rlottie_constructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
}

static void lv_rlottie_destructor(const lv_obj_class_t *class_p, lv_obj_t *obj)
{
    LV_UNUSED(class_p);
    lvsf_rlottie_t *ext = (lvsf_rlottie_t *)obj;

    if (ext->task)
        lv_timer_del(ext->task);
    if (ext->animation)
        lottie_animation_destroy(ext->animation);
    lv_img_cache_invalidate_src(&ext->imgdsc);
    if (ext->allocated_buf)
        app_cache_free(ext->allocated_buf);
    if (ext->file_data)
        app_cache_free(ext->file_data);
}


#if LV_COLOR_DEPTH == 16
static void convert_to_rgba5658(uint32_t *pix, const size_t width, const size_t height)
{
    /* rlottie draws in ARGB32 format, but LVGL only deal with RGB565 format with (optional 8 bit alpha channel)
       so convert in place here the received buffer to LVGL format. */
    uint8_t *dest = (uint8_t *)pix;
    uint32_t *src = pix;
    for (size_t y = 0; y < height; y++)
    {
        /* Convert a 4 bytes per pixel in format ARGB to R5G6B5A8 format
            naive way:
                        r = ((c & 0xFF0000) >> 19)
                        g = ((c & 0xFF00) >> 10)
                        b = ((c & 0xFF) >> 3)
                        rgb565 = (r << 11) | (g << 5) | b
                        a = c >> 24;
            That's 3 mask, 6 bitshift and 2 or operations

            A bit better:
                        r = ((c & 0xF80000) >> 8)
                        g = ((c & 0xFC00) >> 5)
                        b = ((c & 0xFF) >> 3)
                        rgb565 = r | g | b
                        a = c >> 24;
            That's 3 mask, 3 bitshifts and 2 or operations */
        for (size_t x = 0; x < width; x++)
        {
            uint32_t in = src[x];
#if LV_COLOR_16_SWAP == 0
            uint16_t r = (uint16_t)(((in & 0xF80000) >> 8) | ((in & 0xFC00) >> 5) | ((in & 0xFF) >> 3));
#else
            /* We want: rrrr rrrr GGGg gggg bbbb bbbb => gggb bbbb rrrr rGGG */
            uint16_t r = (uint16_t)(((in & 0xF80000) >> 16) | ((in & 0xFC00) >> 13) | ((in & 0x1C00) << 3) | ((in & 0xF8) << 5));
#endif

            memcpy(dest, &r, sizeof(r));
            dest[sizeof(r)] = (uint8_t)(in >> 24);
            dest += LV_IMG_PX_SIZE_ALPHA_BYTE;
        }
        src += width;
    }
}
#endif

static void next_frame_task_cb(lv_timer_t *t)
{
    static uint32_t counter = 0;
    if (counter > 0)
    {
        counter--;
        return;
    }

    lvsf_rlottie_t *ext = (lvsf_rlottie_t *)t->user_data;
    if (ext->current_frame == ext->total_frames)
        ext->current_frame = 0;
    else
        ++ext->current_frame;

    lottie_animation_render(
        ext->animation,
        ext->current_frame,
        ext->allocated_buf,
        ext->imgdsc.header.w,
        ext->imgdsc.header.h,
        ext->scanline_width
    );

#if LV_COLOR_DEPTH == 16
    convert_to_rgba5658(ext->allocated_buf, ext->imgdsc.header.w, ext->imgdsc.header.h);
#endif

#ifdef DISABLE_LVGL_V9
    lv_event_send((lv_obj_t *)ext, LV_EVENT_LEAVE, NULL);
#else
    lv_obj_send_event((lv_obj_t *)ext, LV_EVENT_LEAVE, NULL);
#endif
    lv_obj_invalidate((lv_obj_t *)ext);
}

#else
lv_obj_t *lv_rlottie_create(lv_obj_t *parent)
{
    return NULL;
}

int lv_rlottie_file(lv_obj_t *lottie, const char *path)
{
    return -RT_ERROR;
}

int lv_rlottie_raw(lv_obj_t *lottie, const char *rlottie_desc)
{
    return -RT_ERROR;
}

int lv_rlottie_play(lv_obj_t *lottie, int enable)
{
    return -RT_ERROR;
}
#endif

