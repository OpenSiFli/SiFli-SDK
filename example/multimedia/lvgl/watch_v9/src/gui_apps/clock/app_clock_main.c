
#include "rtconfig.h"
#include <time.h>
#include "app_clock_main.h"
#include "app_clock_status_bar.h"
// #include "lvsf.h"
#ifdef RT_USING_XIP_MODULE
    #include "dlmodule.h"
    #include "dlfcn.h"
    #include "dfs_posix.h"
#endif /* RT_USING_XIP_MODULE */

LV_IMG_DECLARE(img_clock);

#define APP_ID  "clock"
#define APP_CLOCK_ID_MAX_LEN 8
#define CLOCK_UPDATE_INTERVAL_IN_MS 100

typedef enum
{
    STATE_DEINIT = 0,
    STATE_PAUSED, /*clock be inited and paused*/
    STATE_ACTIVE,
} CLOCK_STATE;

/**
 *  description of one clock
 *
 */
typedef struct
{
    lv_obj_t *parent;               //!< clock's root parent obj
    char id[APP_CLOCK_ID_MAX_LEN + 1]; //!< clock's name
    const app_clock_ops_t   *ops;          //!< clock UI state cbk func
    uint8_t state;
    rt_list_t node;                  //!< list node for link all clocks
    void *mod;
} app_clock_desc_t;

/**
 *
 *
 */
typedef struct
{
    lv_point_t *p_tileview_valid_pos;  //!< tileview for app clock framework
    rt_uint32_t app_clock_list_len;    //!< clock list length
    rt_list_t list;                    //!< head of all cocks list

    rt_timer_t soft_timer;             //!<  template vaiable for simulator
} app_clock_main_t;

#ifndef BSP_USING_LVGL_INPUT_AGENT
    static
#endif
app_clock_main_t *p_app_clock_main = NULL;
static rt_uint32_t total_milliseconds = 0;
static clock_t latched_clock;
static uint16_t last_active_clock = 0;

static rt_uint16_t get_active_tile_col(lv_obj_t *tileview)
{
    lv_tileview_t *tv = (lv_tileview_t *)tileview;
    lv_obj_t *active_tile = tv->tile_act; // 获取当前活跃的 tile 对象

    if (!active_tile)
    {
        return 0; // 默认返回第一页
    }

    int32_t w = lv_obj_get_content_width(tileview); // 获取 Tileview 单个 tile 的宽度
    int32_t x = lv_obj_get_x(active_tile); // 获取当前活跃 tile 的 X 坐标

    // 计算当前 tile 所在列
    rt_uint16_t col_id = (x + w / 2) / w;

    return col_id;
}

void app_clock_main_get_current_time(app_clock_time_t *t)
{
    if (t)
    {
#if 0

        rt_uint32_t cur_seconds;

        cur_seconds = total_milliseconds / 1000;
        t->ms = total_milliseconds % 1000;
        t->s = cur_seconds % 60;
        t->h = cur_seconds / 3600;
        t->m = (cur_seconds / 60) % 60;
#elif 1


        uint32_t ms;
        rt_uint32_t cur_seconds;
        clock_t clk = clock();
        clock_t elp;

        if (clk >= latched_clock)
        {
            elp = clk - latched_clock;
        }
        else
        {
            elp = INT32_MAX - latched_clock + 1 + clk;
        }
        latched_clock = clk;


        ms = (uint64_t)elp * 1000 / RT_TICK_PER_SECOND + total_milliseconds;
        cur_seconds = ms / 1000;

        //cur_seconds *= 10; //speed 10x

        t->ms = ms % 1000;
        t->s = cur_seconds % 60;
        t->h = (cur_seconds / 3600) % 24;
        t->m = (cur_seconds / 60) % 60;

        total_milliseconds = ms;


#else
        uint32_t ms;
        uint32_t cur_seconds;

        ms = cpu_get_hw_us() / 1000;
        cur_seconds = ms / 1000;
        t->ms = ms % 1000;
        t->s = cur_seconds % 60;
        t->h = cur_seconds / 3600;
        t->m = (cur_seconds / 60) % 60;

#endif


#ifdef GRAPHIC_REFRESH_TIME_ANALYSIS
        if (refer_ana_enable)
        {
            /*
                clock analysis, fix clock time to 10:10:37

                and invalid screen
            */
            t->h = 10;
            t->m = 10;
            t->s = 37;


            {
                lv_disp_t *disp;
                lv_area_t scr_area;

                disp = lv_disp_get_default();

                scr_area.x1 = 0;
                scr_area.y1 = 0;
                scr_area.x2 = lv_disp_get_hor_res(disp) - 1;
                scr_area.y2 = lv_disp_get_ver_res(disp) - 1;

                _lv_inv_area(disp, &scr_area);
            }

        }
#endif

    }
}

#if 0
void app_clock_main_tick(void *param)
{
    total_milliseconds += CLOCK_UPDATE_INTERVAL_IN_MS;
}
#endif

static const char *app_clock_state_to_name(uint8_t state)
{
#define STATE_TO_NAME_CASE(e) case e: return #e
    switch (state)
    {
        STATE_TO_NAME_CASE(STATE_DEINIT);
        STATE_TO_NAME_CASE(STATE_PAUSED);
        STATE_TO_NAME_CASE(STATE_ACTIVE);

    default:
        return "UNKNOW";


    }
}

static char *change_context;
static lv_obj_t *clk_parent;
char *app_clock_change_context(void)
{
    return change_context;
}

lv_obj_t *gui_app_get_clock_parent(void)
{
    return clk_parent;
}

static void app_clock_change_state(app_clock_desc_t *p_clock, uint8_t new_state)
{
    if (p_clock->state == new_state) return;


    rt_kprintf("app_clock_change_state[%s] %s -> %s\n", p_clock->id,
               app_clock_state_to_name(p_clock->state),
               app_clock_state_to_name(new_state));


    change_context = p_clock->id;
    clk_parent = p_clock->parent;

    switch (new_state)
    {
    case STATE_DEINIT:
    {
        if (STATE_ACTIVE == p_clock->state)
        {
            if (p_clock->ops->pause)
            {
                rt_kprintf("STATE_DEINIT: clock[%s] pause\n", p_clock->id);
                p_clock->ops->pause();
            }
        }
        if (p_clock->ops->deinit)
        {
            rt_kprintf("STATE_DEINIT: clock[%s] deinit\n", p_clock->id);
            p_clock->ops->deinit();
        }

        lv_obj_clean(p_clock->parent);
    }
    break;

    case STATE_PAUSED:
    {
        if (STATE_ACTIVE == p_clock->state)
        {
            if (p_clock->ops->pause)
            {
                rt_kprintf("STATE_PAUSED: clock[%s] pause\n", p_clock->id);
                p_clock->ops->pause();
            }
        }
        else if (p_clock->ops->init)
        {
            rt_kprintf("STATE_PAUSED: clock[%s] init\n", p_clock->id);
            p_clock->ops->init(p_clock->parent);
        }
    }
    break;

    case STATE_ACTIVE:
    {
        if (STATE_DEINIT == p_clock->state)
        {
            if (p_clock->ops->init)
            {
                rt_kprintf("STATE_ACTIVE: clock[%s] init\n", p_clock->id);
                p_clock->ops->init(p_clock->parent);
            }
        }

        if (p_clock->ops->resume)
        {
            rt_kprintf("STATE_ACTIVE: clock[%s] resume\n", p_clock->id);
            p_clock->ops->resume();
        }
    }
    break;


    default:
        break;
    }

    p_clock->state = new_state;
}


static void app_clock_change_state_by_id(uint16_t idx, uint8_t new_state)
{
    rt_kprintf("Change state for clock at index %d to %s\n", idx, app_clock_state_to_name(new_state));
    uint16_t i = 0;
    rt_list_t *pos;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        app_clock_desc_t *clk_desc = rt_list_entry(pos, app_clock_desc_t, node);

        if (idx == i)
        {
            app_clock_change_state(clk_desc, new_state);
            break;
        }
        i++;
    }
}

static void app_clock_main_select(uint16_t clock_idx)
{
    rt_uint16_t left_clock_idx, right_clock_idx, i;
    rt_list_t *pos;
    app_clock_desc_t *clk_desc;

    if (clock_idx >= p_app_clock_main->app_clock_list_len)
        clock_idx = p_app_clock_main->app_clock_list_len - 1;

    if (clock_idx > 0)
        left_clock_idx = clock_idx - 1;
    else
        left_clock_idx = p_app_clock_main->app_clock_list_len; //invalid left clock

    right_clock_idx = clock_idx + 1;

    if (right_clock_idx > p_app_clock_main->app_clock_list_len)
        right_clock_idx = p_app_clock_main->app_clock_list_len; //invalid right clock

    /*deinit all other clock , to free memory*/
    i = 0;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        clk_desc = rt_list_entry(pos, app_clock_desc_t, node);

        if ((i != clock_idx) && (i != left_clock_idx) && (i != right_clock_idx))
        {
            app_clock_change_state(clk_desc, STATE_DEINIT);
        }
        i++;
    }

    /*pause left&right clock , to free memory*/
    i = 0;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        clk_desc = rt_list_entry(pos, app_clock_desc_t, node);

        if ((i == left_clock_idx) || (i == right_clock_idx))
        {
            app_clock_change_state(clk_desc, STATE_PAUSED);
        }
        i++;
    }

    /* active selected clock */
    i = 0;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        clk_desc = rt_list_entry(pos, app_clock_desc_t, node);

        if (i == clock_idx)
        {
            app_clock_change_state(clk_desc, STATE_ACTIVE);
        }

        i++;
    }

    last_active_clock = clock_idx;
}

static void app_clock_main_drag_begin(uint16_t clock_idx)
{
    rt_uint16_t left_clock_idx, right_clock_idx, i;
    rt_list_t *pos;
    app_clock_desc_t *clk_desc;

    if (clock_idx >= p_app_clock_main->app_clock_list_len)
        clock_idx = p_app_clock_main->app_clock_list_len - 1;

    if (clock_idx > 0)
        left_clock_idx = clock_idx - 1;
    else
        left_clock_idx = p_app_clock_main->app_clock_list_len; //invalid left clock

    right_clock_idx = clock_idx + 1;

    if (right_clock_idx > p_app_clock_main->app_clock_list_len)
        right_clock_idx = p_app_clock_main->app_clock_list_len; //invalid right clock


    /* active the left&right clock*/
    i = 0;
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        clk_desc = rt_list_entry(pos, app_clock_desc_t, node);

        if ((i == left_clock_idx) || (i == right_clock_idx))
        {
            app_clock_change_state(clk_desc, STATE_ACTIVE);
        }

        i++;
    }
}


static void tileview_event_cb_t(lv_event_t *event)
{
    //if (event->code != LV_EVENT_PRESSING)
    //    rt_kprintf("tileview_event_cb_t %s\n", lv_event_to_name(event->code));

    switch (event->code)
    {
    case LV_EVENT_VALUE_CHANGED:
    {
        rt_uint16_t active_pos = get_active_tile_col(lv_event_get_current_target(event));
        rt_kprintf("tileview_event_cb_t LV_EVENT_VALUE_CHANGED, active_pos=%d\n", active_pos);

        if (gui_app_is_actived(APP_ID)) //value_changed could be sent after app paused
            app_clock_main_select(active_pos);
        else
            last_active_clock = active_pos;
    }
    break;

#if (LV_HOR_RES_MAX == 240) && (LV_HOR_RES_MAX == 240) //Active neighbor clock may cause malloc mem failure in high resolution
    case LV_EVENT_SCROLL_BEGIN:
    {
        app_clock_main_drag_begin(last_active_clock);
    }
    break;
#endif

    default:
        break;
    }
}


static void app_clock_main_init(void)
{
    rt_uint16_t i;
    lv_coord_t scr_hor_res, scr_ver_res;
    rt_uint16_t last_active_clock_bak; /*when tileview created, LV_EVENT_VALUE_CHANGED (idx is 0) will be send */

    scr_hor_res = lv_disp_get_hor_res(lv_disp_get_default());
    scr_ver_res = lv_disp_get_ver_res(lv_disp_get_default());

    last_active_clock_bak = last_active_clock;

    lv_obj_t *tileview;

    //1. create title view
    tileview = lv_tileview_create(lv_scr_act());
    lv_obj_set_scrollbar_mode(tileview, LV_SCROLLBAR_MODE_OFF);
    p_app_clock_main->p_tileview_valid_pos = (lv_point_t *) rt_malloc(sizeof(lv_point_t) * p_app_clock_main->app_clock_list_len);
    rt_kprintf("app_clock_main_init: app_clock_list_len=%d\n", p_app_clock_main->app_clock_list_len);

    lv_obj_set_style_bg_color(tileview, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(tileview, LV_OPA_COVER, LV_PART_MAIN);

    for (i = 0; i < p_app_clock_main->app_clock_list_len; i++)
    {
        p_app_clock_main->p_tileview_valid_pos[i].x = i;
        p_app_clock_main->p_tileview_valid_pos[i].y = 0;
    }

    //2. load clocks into tileview
    rt_list_t *pos;
    i = 0;
    //rt_list_for_each_entry(clk_desc,&p_app_clock_main->list,node)
    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        app_clock_desc_t *clk_desc = rt_list_entry(pos, app_clock_desc_t, node);
        rt_kprintf("Page[%u]: %s, state=%s\n", i, clk_desc->id, app_clock_state_to_name(clk_desc->state));
        lv_obj_t *page;

        page = lv_tileview_add_tile(tileview, i, 0, LV_DIR_HOR);
        lv_obj_set_size(page, scr_hor_res, scr_ver_res);
        lv_obj_set_pos(page, scr_hor_res * i, 0);
        lv_obj_set_scrollbar_mode(page, LV_SCROLLBAR_MODE_OFF);

        clk_desc->parent = page;
        i++;
        rt_kprintf("app_clock_main_init: add clock[%s] to tileview, idx=%d\n", clk_desc->id, i - 1);
    }

    lv_obj_add_event_cb(tileview, tileview_event_cb_t, LV_EVENT_ALL, NULL);

    if (last_active_clock_bak < p_app_clock_main->app_clock_list_len)
        lv_obj_set_tile_id(tileview, last_active_clock_bak, 0, false);
    else
        last_active_clock_bak = 0;

    //3.create status bar at top
    app_clock_main_status_bar_init(lv_scr_act(), tileview);
}

#ifdef RT_USING_XIP_MODULE
static void app_clock_load_one_dyn_wf(const char *module_name, const char *path)
{
    struct rt_dlmodule *mod;
    uint32_t old_len;
    app_clock_desc_t *clk_desc;

    old_len = p_app_clock_main->app_clock_list_len;

    mod = dlrun(module_name, path);

    if (mod && (old_len != p_app_clock_main->app_clock_list_len))
    {
        clk_desc = rt_list_tail_entry(&p_app_clock_main->list, app_clock_desc_t, node);
        clk_desc->mod = mod;
    }
}

static void app_clock_load_dyn_wf(void)
{
    DIR *dir;
    struct dirent *dir_entry;
    struct stat *ent_stat;
    char *full_path;
    const char *path = "watchface";
    uint32_t name_len;

    dir = opendir(path);
    if (!dir)
    {
        return;

    }
    ent_stat = rt_malloc(sizeof(*ent_stat));
    RT_ASSERT(stat);

    do
    {
        dir_entry = readdir(dir);
        if (!dir_entry)
        {
            break;
        }

        memset(ent_stat, 0, sizeof(*ent_stat));

        /* build full path for each file */
        full_path = dfs_normalize_path(path, dir_entry->d_name);
        if (full_path == NULL)
        {
            break;
        }

        if (stat(full_path, ent_stat) == 0)
        {
            if (!S_ISDIR(ent_stat->st_mode))
            {
                name_len = strlen(dir_entry->d_name);
                if (('m' == dir_entry->d_name[name_len - 1])
                        && ('.' == dir_entry->d_name[name_len - 2])) /* ending with .m */
                {
                    /* remove suffix .m */
                    dir_entry->d_name[name_len - 2] = 0;
                    app_clock_load_one_dyn_wf(dir_entry->d_name, path);
                }
            }
        }
        rt_free(full_path);
    }
    while (true);

    rt_free(ent_stat);
    closedir(dir);
}

#endif /* RT_USING_XIP_MODULE */

/**********************regist app clock to app manager****************************/
extern void app_clock_rotate_bg_register(void);
#if 1//!(defined(PKG_USING_MICROPYTHON)||defined(PKG_USING_QUICKJS))
    extern void app_clock_simple_register(void);
    extern void app_clock_dial_register(void);

#endif /* defined(PKG_USING_MICROPYTHON)||defined(PKG_USING_QUICKJS)*/

#ifdef PKG_USING_FFMPEG
    extern void app_clock_video_audio_register(void);
#endif /* PKG_USING_FFMPEG */
#if PKG_USING_STREAMING_MEDIA_DEMO_APP
    extern void app_clock_streamingmedia_register(void);
#endif
void app_clock_reset_time(void)
{
    struct tm *time_info;

#ifdef WIN32
    __time32_t raw_time;
    _time32(&raw_time);
    time_info = _localtime32(&raw_time);
#else
    time_t raw_time;
    time(&raw_time);
    time_info = localtime(&raw_time);
#endif
    latched_clock = clock();
    total_milliseconds = (((time_info->tm_hour * 60) + time_info->tm_min) * 60 + time_info->tm_sec) * 1000;

    rt_kprintf("service_reset_time:  %d:%d:%d - %d:%d:%d - %d\n", time_info->tm_year, time_info->tm_mon, time_info->tm_mday, time_info->tm_hour, time_info->tm_min, time_info->tm_sec, time_info->tm_wday);
}


static void on_start(void)
{
    /* init list*/
    p_app_clock_main = (app_clock_main_t *) rt_malloc(sizeof(app_clock_main_t));
    memset(p_app_clock_main, 0, sizeof(app_clock_main_t));
    rt_list_init(&p_app_clock_main->list);
#if (LV_HOR_RES_MAX < 512)&&(LV_VER_RES_MAX < 512)
    app_clock_rotate_bg_register();
#endif

#if 1//!(defined(PKG_USING_MICROPYTHON)||defined(PKG_USING_QUICKJS))
    app_clock_simple_register();
    app_clock_dial_register();

#endif /* defined(PKG_USING_MICROPYTHON)||defined(PKG_USING_QUICKJS) */
    gui_script_watch_face_register();

#ifdef RT_USING_XIP_MODULE
    app_clock_load_dyn_wf();
#endif /* RT_USING_XIP_MODULE */

    app_clock_reset_time();

    /* first resume after launched*/
    app_clock_main_init();
}

static void on_resume(void)
{

    app_clock_main_select(last_active_clock);
}

static void on_pause(void)
{
    rt_list_t *pos;
    uint16_t i = 0;

    rt_list_for_each(pos, (&p_app_clock_main->list))
    {
        app_clock_desc_t *clk_desc;
        clk_desc = rt_list_entry(pos, app_clock_desc_t, node);
        app_clock_change_state(clk_desc, STATE_DEINIT);
    }
}

static void on_stop(void)
{
    if (p_app_clock_main)
    {

        rt_list_t *pos;

        rt_list_for_each(pos, (&p_app_clock_main->list))
        {
            app_clock_desc_t *clk_desc;
            clk_desc = rt_list_entry(pos, app_clock_desc_t, node);

            app_clock_change_state(clk_desc, STATE_DEINIT);

#ifdef RT_USING_XIP_MODULE
            if (clk_desc->mod)
            {
                dlclose(clk_desc->mod);
            }
#endif  /* RT_USING_XIP_MODULE */
            rt_free(clk_desc);
        }
        rt_free(p_app_clock_main->p_tileview_valid_pos);
        rt_free(p_app_clock_main);
        p_app_clock_main = NULL;

        app_clock_main_status_bar_deinit();
    }
}



static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start();
        break;

    case GUI_APP_MSG_ONRESUME:
        on_resume();
        break;

    case GUI_APP_MSG_ONPAUSE:
        on_pause();
        break;

    case GUI_APP_MSG_ONSTOP:
        on_stop();
        break;
    default:
        break;
    }
}


static int app_main(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID, msg_handler);

    uint32_t active = intent_get_uint32(i, "active", 0xFFFF);
    if (active != 0xFFFF)
        last_active_clock = active;

    return 0;
}


BUILTIN_APP_EXPORT(LV_EXT_STR_ID(clock), LV_EXT_IMG_GET(img_clock), APP_ID, app_main);


/**********************app clocks manager**************************/
int32_t app_clock_register(const char *id, const app_clock_ops_t *operations)
{
    app_clock_desc_t *new_clock;
    uint16_t id_len;

    if ((!id) || (!operations)) return RT_EINVAL;

    new_clock = (app_clock_desc_t *) rt_malloc(sizeof(app_clock_desc_t));

    id_len = strlen(id);
    if (id_len > APP_CLOCK_ID_MAX_LEN)
        id_len = APP_CLOCK_ID_MAX_LEN;

    memcpy(new_clock->id, id, id_len);
    new_clock->id[id_len] = '\0';

    new_clock->ops = operations;
    new_clock->state = STATE_DEINIT;
    new_clock->mod = NULL;

    rt_list_init(&new_clock->node);
    rt_list_insert_before(&p_app_clock_main->list, &new_clock->node);

    p_app_clock_main->app_clock_list_len++;

    return 0;
}

#if 0
/**
 * duplicate an image to SRAM to improve drawn performance
 * \n
 *
 * @return
 * @param copy
 * \n
 * @see
 */
lv_img_dsc_t *app_clock_img_cache_malloc(const void *copy)
{
    lv_res_t res;
    lv_img_cache_entry_t *cache_entry;

    if (NULL == copy) return NULL;

    /* Allocate image descriptor */
    lv_img_dsc_t *dsc = lv_mem_alloc(sizeof(lv_img_dsc_t));
    if (dsc == NULL)
        return NULL;

    cache_entry = _lv_img_cache_open(copy, LV_COLOR_BLACK);
    RT_ASSERT(cache_entry);

    memcpy(&dsc->header, &cache_entry->dec_dsc.header, sizeof(dsc->header));

    /* Allocate raw buffer */
    dsc->data = lv_mem_alloc(cache_entry->dec_dsc.img_data_size);
    if (dsc->data == NULL)
    {
        lv_mem_free(dsc);
        return NULL;
    }

    memcpy((uint8_t *)dsc->data, (uint8_t *)cache_entry->dec_dsc.img_data, cache_entry->dec_dsc.img_data_size);

    return dsc;
}

void app_clock_img_cache_free(lv_img_dsc_t *p_img)
{
    if (NULL != p_img)
    {
        lv_img_cache_invalidate_src(p_img);

        if (NULL != p_img->data)
            lv_mem_free(p_img->data);

        lv_mem_free(p_img);
    }
}
#endif


