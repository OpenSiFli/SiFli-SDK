/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf_comp.h"
#include "gui_app_fwk.h"
#include "lv_ext_resource_manager.h"
#include "lv_ex_data.h"
#include "sensor.h"
#include "../../service/hr_service.h"
#include "ui_datasrv_subscriber.h"


static void close_btn_event_cb(lv_event_t *e)
{
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_CLICKED == event)
    {
        gui_app_self_exit();
    }
}

#if 1
LV_IMG_DECLARE(img_activity);

LV_IMG_DECLARE(img_ecg);
LV_IMG_DECLARE(img_red_heart);
LV_IMG_DECLARE(img_left_arrow);
LV_IMG_DECLARE(up_arrow);
LV_IMG_DECLARE(down_arrow);
LV_IMG_DECLARE(hr_region);



#define HEART_RATE_CANVAS_WIDTH    150
#define HEART_RATE_CANVAS_HEIGHT   150
#define HEART_RATE_CANVAS_BUF_SIZE LV_CANVAS_BUF_SIZE_TRUE_COLOR(HEART_RATE_CANVAS_WIDTH, HEART_RATE_CANVAS_HEIGHT)
#define HEART_RATE_REDRAW_INTERVAL_MS 100

#define APP_ID "health"

typedef struct
{
    lv_ex_data_t *hr_data;  // page 1
    lv_ex_data_t *max_data;
    lv_ex_data_t *min_data;
    lv_ex_data_t *cur_rhr;
    lv_ex_data_t *ul_data;  // page 2
    lv_ex_data_t *ana_data;
    lv_ex_data_t *aer_data;
    lv_ex_data_t *hiit_data;
    lv_ex_data_t *warmu_data;
    lv_ex_data_t *ave_rhr;      // page 3
    lv_ex_data_t *max_rhr;
    lv_ex_data_t *min_rhr;
    datac_handle_t data_handle;
    bool active;
} app_hr_data_ctx_t;

static app_hr_data_ctx_t app_hr_data_ctx;

static custom_hr_data_table_t app_hr_data_table;

#ifndef ABS
    #define ABS(_x) ((_x) < 0 ? -(_x) : (_x))
#endif
#define CHART_WIDTH  (LV_HOR_RES_MAX * 5 /6)
#define CHART_HEIGTH (LV_VER_RES_MAX / 2)


static rt_uint8_t *scr_heart_rate_canvas_buffer = RT_NULL;
static lv_obj_t *measure_heart_rate_canvas;
static lv_obj_t *measure_heart_rate_ecg;
static lv_obj_t *measure_heart_rate_label;
static lv_obj_t *max_heart_rate_label;
static lv_obj_t *min_heart_rate_label;
static lv_obj_t *resting_heart_rate_label;
static lv_obj_t *measure_heart_rate_chart;

static lv_obj_t *heart_rate_label_ul;
static lv_obj_t *heart_rate_label_ana;
static lv_obj_t *heart_rate_label_aer;
static lv_obj_t *heart_rate_label_hiit;
static lv_obj_t *heart_rate_label_warmu;

static lv_obj_t *average_rhr_label;
static lv_obj_t *max_rhr_label;
static lv_obj_t *min_rhr_label;
static lv_obj_t *rhr_month_chart;


static lv_coord_t window_x0, window_x;
static lv_timer_t *heart_rate_redraw_test_task;
//static rt_uint32_t
static rt_uint32_t heart_rate_bpm = 0;
static lv_chart_type_t chart_type = LV_CHART_TYPE_LINE;

static int hr_data_callback(data_callback_arg_t *arg)
{
#define MAC_BPM_STR_LEN (30)

    if ((!app_hr_data_ctx.active)
            && (MSG_SERVICE_SUBSCRIBE_RSP != arg->msg_id))
    {
        return 0;
    }

    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        break;
    }
    case MSG_SERVICE_DATA_NTF_IND:
    {
        struct rt_sensor_data *data;
        RT_ASSERT(arg->data);
        data = (struct rt_sensor_data *)arg->data;
        RT_ASSERT(arg->data_len == sizeof(*data));

        if ((data->data.hr >= 0) && (data->data.hr != heart_rate_bpm) && app_hr_data_ctx.hr_data)
        {
            char *s = lv_mem_alloc(MAC_BPM_STR_LEN);
            RT_ASSERT(s);

            //app_hr_data_ctx.hr_data->updated = false;
            //rt_snprintf(s, MAC_BPM_STR_LEN, LV_EXT_STR_GET_BY_KEY(hr_value_display_text, "%d BPM"), data->data.hr);
            if (data->data.hr == 0)
                rt_snprintf(s, MAC_BPM_STR_LEN, "   --  BPM");
            else
                rt_snprintf(s, MAC_BPM_STR_LEN, "    %d BPM", data->data.hr);
            //lv_ex_data_set_value(app_hr_data_ctx.hr_data, (void *)s);
            lv_label_set_text(measure_heart_rate_label, (const char *)s);
            heart_rate_bpm = data->data.hr;
            lv_mem_free(s);
            if (app_hr_data_table.max < data->data.hr)
                app_hr_data_table.max = data->data.hr;
            if (app_hr_data_table.min > data->data.hr)
                app_hr_data_table.min = data->data.hr;
        }

        break;
    }
    case MSG_SERVICE_HR_MAX_MIN_RSP:
    {
        uint8_t *value = (uint8_t *)arg->data;
        char *s = lv_mem_alloc(MAC_BPM_STR_LEN);
        //rt_kprintf("MAXMIN len %d\n",arg->data_len);
        if (arg->data_len == HRS_MAX_MIN_LEN) // get max in
        {
            app_hr_data_table.max = value[0];
            app_hr_data_table.min = value[1];
            app_hr_data_table.rhr = value[2];

            //app_hr_data_ctx.max_data->updated = false;
            rt_snprintf(s, 4, "%d", app_hr_data_table.max);
            //lv_ex_data_set_value(app_hr_data_ctx.max_data, (void *)s);
            lv_label_set_text(max_heart_rate_label, (const char *)s);

            //app_hr_data_ctx.min_data->updated = false;
            rt_snprintf(s, 4, "%d", app_hr_data_table.min);
            //lv_ex_data_set_value(app_hr_data_ctx.min_data, (void *)s);
            lv_label_set_text(min_heart_rate_label, (const char *)s);

            //app_hr_data_ctx.cur_rhr->updated = false;
            rt_snprintf(s, 4, "%d", app_hr_data_table.rhr);
            //lv_ex_data_set_value(app_hr_data_ctx.cur_rhr, (void *)s);
            lv_label_set_text(resting_heart_rate_label, (const char *)s);
            //rt_kprintf("Get max min %d: %d, rhr %d\n", app_hr_data_table.max, app_hr_data_table.min, app_hr_data_table.rhr);
        }
        lv_mem_free(s);
        break;
    }
    case MSG_SERVICE_HR_DAY_TABLE_RSP:
    {
        uint8_t *value = (uint8_t *)arg->data;
        //rt_kprintf("DAY len %d\n",arg->data_len);

        if (arg->data_len == HRS_DAY_TABLE_LEN)   // get day table
        {
            int i;

            for (i = 0; i < HRS_DAY_TABLE_LEN; i++)
            {
                //if (value[i] == 0)
                //    app_hr_data_table.today[i] = LV_COORD_MIN;
                //else
                {
                    app_hr_data_table.today[i] = (uint16_t)value[i];
                    //rt_kprintf("%d: %d\n", i, app_hr_data_table.today[i]);
                }
            }

            lv_chart_series_t *ser1 = lv_chart_add_series(measure_heart_rate_chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
            for (i = 0; i < HRS_DAY_TABLE_LEN; i++)
            {
                ser1->y_points[i] = app_hr_data_table.today[i];
            }
            lv_chart_refresh(measure_heart_rate_chart); /*Required after direct set*/
        }
        break;
    }
    case MSG_SERVICE_HR_MON_TABLE_RSP:
    {
        uint8_t *value = (uint8_t *)arg->data;
        //rt_kprintf("MON len %d\n",arg->data_len);

        if (arg->data_len == HRS_MON_TABLE_LEN)   // get day table
        {
            int i;

            for (i = 0; i < HRS_MON_TABLE_LEN; i++)
            {
                //if (value[i] == 0)
                //    app_hr_data_table.mon[i] = LV_COORD_MIN;
                //else
                {
                    app_hr_data_table.mon[i] = (uint16_t)value[i];
                    //rt_kprintf("%d: %d\n", i, app_hr_data_table.today[i]);
                }
            }
            lv_chart_series_t *ser1 = lv_chart_add_series(rhr_month_chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
            //lv_chart_set_points(rhr_month_chart, ser1, (lv_coord_t *)app_hr_data_table.mon);
            for (i = 0; i < HRS_MON_TABLE_LEN; i++)
            {
                ser1->y_points[i] = app_hr_data_table.mon[i];
            }
            lv_chart_refresh(rhr_month_chart); /*Required after direct set*/
        }
        break;
    }
    case MSG_SERVICE_RHR_VALUE_RSP:
    {
        uint8_t *value = (uint8_t *)arg->data;
        char *s = lv_mem_alloc(MAC_BPM_STR_LEN);
        //rt_kprintf("RHR len %d\n",arg->data_len);
        if (arg->data_len == HRS_RHR_HIST_LEN)   // get day table
        {
            app_hr_data_table.max_rhr = value[0];
            app_hr_data_table.min_rhr = value[1];
            app_hr_data_table.ave_rhr = value[2];

            //app_hr_data_ctx.ave_rhr->updated = false;
            rt_snprintf(s, MAC_BPM_STR_LEN - 1, "AVERAGE: %d", app_hr_data_table.ave_rhr);
            //lv_ex_data_set_value(app_hr_data_ctx.ave_rhr, (void *)s);
            lv_label_set_text(average_rhr_label, (const char *)s);

            //app_hr_data_ctx.max_rhr->updated = false;
            rt_snprintf(s, 4, "%d", app_hr_data_table.max_rhr);
            //lv_ex_data_set_value(app_hr_data_ctx.max_rhr, (void *)s);
            lv_label_set_text(max_rhr_label, (const char *)s);

            //app_hr_data_ctx.min_rhr->updated = false;
            rt_snprintf(s, 4, "%d", app_hr_data_table.min_rhr);
            //lv_ex_data_set_value(app_hr_data_ctx.min_rhr, (void *)s);
            lv_label_set_text(min_rhr_label, (const char *)s);
        }
        lv_mem_free(s);
        break;
    }
    case MSG_SERVICE_HR_REGION_RSP:
    {
        uint8_t *value = (uint8_t *)arg->data;
        char *s = lv_mem_alloc(MAC_BPM_STR_LEN);
        //rt_kprintf("REGION len %d\n",arg->data_len);
        if (arg->data_len == HRS_REGION_LEN)   // get day table
        {
            int i;
            for (i = 0; i < 5; i++)
                app_hr_data_table.region[i] = value[i];

            //app_hr_data_ctx.ul_data->updated = false;
            rt_snprintf(s, 4, "%d", app_hr_data_table.region[0]);
            //lv_ex_data_set_value(app_hr_data_ctx.ul_data, (void *)s);
            lv_label_set_text(heart_rate_label_ul, (const char *)s);

            //app_hr_data_ctx.ana_data->updated = false;
            rt_snprintf(s, 4, "%d", app_hr_data_table.region[1]);
            //lv_ex_data_set_value(app_hr_data_ctx.ana_data, (void *)s);
            lv_label_set_text(heart_rate_label_ana, (const char *)s);

            //app_hr_data_ctx.aer_data->updated = false;
            rt_snprintf(s, 4, "%d", app_hr_data_table.region[2]);
            //lv_ex_data_set_value(app_hr_data_ctx.aer_data, (void *)s);
            lv_label_set_text(heart_rate_label_aer, (const char *)s);

            //app_hr_data_ctx.hiit_data->updated = false;
            rt_snprintf(s, 4, "%d", app_hr_data_table.region[3]);
            //lv_ex_data_set_value(app_hr_data_ctx.hiit_data, (void *)s);
            lv_label_set_text(heart_rate_label_hiit, (const char *)s);

            //app_hr_data_ctx.warmu_data->updated = false;
            rt_snprintf(s, 4, "%d", app_hr_data_table.region[4]);
            //lv_ex_data_set_value(app_hr_data_ctx.warmu_data, (void *)s);
            lv_label_set_text(heart_rate_label_warmu, (const char *)s);
        }
        lv_mem_free(s);
        break;
    }
    }
    return 0;
}



void heart_rate_ecg_anim(void *obj, int32_t x)
{
    lv_obj_set_x((lv_obj_t *)obj, x);
    lv_img_set_offset_x((lv_obj_t *)obj, x);
}

static void measure_heart_rate_cavans_event_cb(lv_event_t *e)
{
    char buff[20];
    lv_event_code_t event = lv_event_get_code(e);

    //rt_kprintf("measure_heart_rate_cavans_event_cb %s\n",lv_event_to_name(event));

    switch (event)
    {
    case LV_EVENT_DEFOCUSED:
        heart_rate_bpm = 0;
        rt_snprintf(buff, sizeof(buff), LV_EXT_STR_GET_BY_KEY(hr_value_display_text, "%d BPM"), heart_rate_bpm);
        lv_label_set_text(measure_heart_rate_label, buff);
        //TODO
        //lv_obj_realign(measure_heart_rate_label);
        break;

    case LV_EVENT_CLICKED:
    {
#ifndef DISABLE_LVGL_V8
        //setup animation of ecg
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, measure_heart_rate_ecg);
        lv_anim_set_exec_cb(&a, heart_rate_ecg_anim);    /*Set the animator function and variable to animate*/
        lv_anim_set_time(&a, 1500);

        _lv_img_cache_entry_t *cache_entry;

        cache_entry = _lv_img_cache_open(LV_EXT_IMG_GET(img_ecg), lv_color_black(), 0);
        RT_ASSERT(cache_entry);

        lv_anim_set_values(&a, lv_obj_get_x(measure_heart_rate_ecg) - cache_entry->dec_dsc.header.w, cache_entry->dec_dsc.header.w);             /*Set start and end values. E.g. 0, 150*/

        lv_anim_set_path_cb(&a, lv_anim_path_linear);
        lv_anim_set_repeat_delay(&a, 2000);            /*Enable repeat of teh animation with `wait_time` delay. Can be compiled with playback*/

        lv_anim_start(&a);                             /*Start the animation*/

#else


        window_x0 = lv_obj_get_x(measure_heart_rate_ecg) - img_ecg.header.w;
        window_x = window_x0;
        lv_img_set_offset_x(measure_heart_rate_ecg, window_x);
        lv_obj_set_x(measure_heart_rate_ecg, window_x);

#endif

#if 0
        heart_rate_bpm = 60 + ((heart_rate_bpm + 10) % 60);

        rt_sprintf(buff, "%d BPM", heart_rate_bpm);
        lv_label_set_text(measure_heart_rate_label, buff);
        lv_obj_realign(measure_heart_rate_label);
#endif
    }
    break;

    case LV_EVENT_LONG_PRESSED:
        break;

    case LV_EVENT_LONG_PRESSED_REPEAT:
        break;

    case LV_EVENT_RELEASED:
        break;
    default:
        break;
    }
}

void measure_heart_rate_redraw(lv_timer_t *task)
{
    static rt_uint32_t zoom = 0;


    if (heart_rate_bpm)
        zoom = LV_IMG_ZOOM_NONE + ((zoom + (heart_rate_bpm * HEART_RATE_REDRAW_INTERVAL_MS / 600)) % 100);
    else
        zoom = LV_IMG_ZOOM_NONE;

    //rt_kprintf("measure_heart_rate_redraw zoom:%d  window_x:%d\n",zoom,window_x);

    if (RT_NULL != scr_heart_rate_canvas_buffer)
    {
        memset(scr_heart_rate_canvas_buffer, 0, HEART_RATE_CANVAS_BUF_SIZE);
        lvsf_canvas_rotate(measure_heart_rate_canvas, (lv_img_dsc_t *)LV_EXT_IMG_GET(img_red_heart), 0, zoom, 0, 0, 0, 0);
    }

#if 0
    char buff[10];
    if (window_x <= (lv_coord_t)img_ecg.header.w)
        window_x += 8;
    else
        window_x = window_x0;

    lv_obj_set_x(measure_heart_rate_ecg, window_x);
    lv_img_set_offset_x(measure_heart_rate_ecg, window_x);


    rt_sprintf(buff, "%d BPM", ABS(window_x));

    lv_label_set_text(measure_heart_rate_label, buff);
#endif
}


void create_measure_heart_rate_page(lv_obj_t *parent)
{
    lv_obj_t *title_container;
    lv_obj_t *title;
    //lv_obj_t *table;
    lv_obj_t *close_btn;
    lv_obj_t *max;
    lv_obj_t *min;
    lv_obj_t *rhr;
    lv_ex_binding_t binding;

    title_container = lv_obj_create(parent);
    //TODO
    //lv_obj_set_auto_realign(title_container, true);
    lv_obj_set_size(title_container, LV_HOR_RES_MAX, (LV_VER_RES_MAX / 6));
    lv_obj_align(title_container, LV_ALIGN_TOP_MID, 0, 0);


    title = lv_label_create(title_container);
    lv_label_set_text(title, LV_EXT_STR_GET_BY_KEY(hr_title, "HR Title"));
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);
#if 0
    //create a canvas for scale heart
    measure_heart_rate_canvas = lv_canvas_create(parent, NULL);
    if (RT_NULL == scr_heart_rate_canvas_buffer)
    {
        scr_heart_rate_canvas_buffer = lv_mem_alloc(HEART_RATE_CANVAS_BUF_SIZE);
        RT_ASSERT(RT_NULL != scr_heart_rate_canvas_buffer);
    }

    const lv_img_dsc_t *img;
    img = LV_EXT_IMG_GET(img_red_heart);
    if (img)
    {
        lv_canvas_set_buffer(measure_heart_rate_canvas, scr_heart_rate_canvas_buffer, img->header.w, img->header.h, LV_IMG_CF_TRUE_COLOR);
    }
    lv_obj_align(measure_heart_rate_canvas, LV_ALIGN_CENTER, 0, 0);

    //and a image for ecg
    measure_heart_rate_ecg = lv_img_create(measure_heart_rate_canvas, NULL);
    lv_img_set_src(measure_heart_rate_ecg, LV_EXT_IMG_GET(img_ecg));
    lv_obj_align_to(measure_heart_rate_ecg, measure_heart_rate_canvas, LV_ALIGN_CENTER, 0, 0);

    //add a label at bottom
    measure_heart_rate_label = lv_label_create(parent, NULL);
    lv_label_set_text(measure_heart_rate_label, LV_EXT_STR_GET_BY_KEY(hr_value_display_text_init, "- BPM"));
    lv_obj_align(measure_heart_rate_label, LV_ALIGN_IN_BOTTOM_MID, 0, -20);

    app_hr_data_ctx.hr_data = lv_ex_data_create("hr.hr_val", LV_EX_DATA_STRING);
    binding.target = measure_heart_rate_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.hr_data, &binding);
    app_hr_data_ctx.data_handle = datac_subscribe("HR", hr_data_callback, 0);
#else
    //and a image for ecg
    //table = lv_img_create(parent, NULL);
    //lv_img_set_src(table, LV_EXT_IMG_GET(hr_table));
    //lv_obj_align(table, NULL, LV_ALIGN_CENTER, 0, 20);

#if 1

    measure_heart_rate_chart = lv_chart_create(parent);
    lv_obj_set_size(measure_heart_rate_chart, CHART_WIDTH, CHART_HEIGTH);
    lv_obj_align_to(measure_heart_rate_chart, parent, LV_ALIGN_CENTER, -20, 20);
    //TODO
    //lv_obj_set_parent_event(measure_heart_rate_chart, true);
    //lv_obj_set_drag_parent(chart, true);

    /*Add a faded are effect*/
    lv_obj_set_style_bg_opa(measure_heart_rate_chart, LV_OPA_50, LV_PART_ITEMS | LV_STATE_DEFAULT); /*Max. opa.*/
    lv_obj_set_style_bg_grad_dir(measure_heart_rate_chart, LV_GRAD_DIR_VER, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(measure_heart_rate_chart, 255, LV_PART_ITEMS | LV_STATE_DEFAULT);    /*Max opa on the top*/
    lv_obj_set_style_bg_grad_stop(measure_heart_rate_chart, 0, LV_PART_ITEMS | LV_STATE_DEFAULT);      /*Transparent on the bottom*/

    //lv_chart_set_type(chart, LV_CHART_TYPE_AREA|LV_CHART_TYPE_POINT);   /*Show lines and points too*/
    lv_chart_set_type(measure_heart_rate_chart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/
    lv_chart_set_div_line_count(measure_heart_rate_chart, 3, 3);
    //lv_chart_set_y_tick_length(measure_heart_rate_chart, 1, 1);
    //lv_chart_set_x_tick_length(measure_heart_rate_chart, 1, 1);
    /*margin for axes*/
    // TODO: HG_701, lv_chart_set_margin(chart, 40);
    lv_obj_set_style_text_font(measure_heart_rate_chart, LV_EXT_FONT_GET(FONT_SMALL), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(measure_heart_rate_chart, 40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(measure_heart_rate_chart, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    //lv_obj_set_style_local_line_dash_gap(measure_heart_rate_chart,  LV_CHART_PART_BG, LV_STATE_DEFAULT, 0);
    //lv_obj_set_style_local_line_dash_width(measure_heart_rate_chart,  LV_CHART_PART_BG, LV_STATE_DEFAULT, 0);
    //lv_obj_set_style_local_text_color(chart, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);

    //lv_obj_set_style_local_line_color(measure_heart_rate_chart, LV_CHART_PART_BG, LV_STATE_DEFAULT, LV_COLOR_YELLOW);
    lv_obj_set_style_line_color(measure_heart_rate_chart, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN | LV_STATE_DEFAULT);
    //lv_obj_set_style_local_line_blend_mode(measure_heart_rate_chart, LV_CHART_PART_SERIES_BG, LV_STATE_DEFAULT, 1);
    lv_obj_set_style_line_opa(measure_heart_rate_chart, 80, LV_PART_MAIN | LV_STATE_DEFAULT);


#ifndef DISABLE_LVGL_V8
    //const char *x_tick_text = "00:00\n06:00\n12:00\n18:00\n24:00";
    //lv_chart_set_x_tick_texts(measure_heart_rate_chart, x_tick_text, 5, LV_CHART_AXIS_SKIP_LAST_TICK);
    lv_chart_set_axis_tick(measure_heart_rate_chart, LV_CHART_AXIS_PRIMARY_X, 10, 5, 5, 6, true, 40);

    //const char *y_tick_text = " \n50\n100\n150\n200";
    //lv_chart_set_y_tick_texts(measure_heart_rate_chart, y_tick_text, 5, LV_CHART_AXIS_DRAW_LAST_TICK | LV_CHART_AXIS_INVERSE_LABELS_ORDER);

    lv_chart_set_axis_tick(measure_heart_rate_chart, LV_CHART_AXIS_PRIMARY_Y, 10, 1, 4, 1, true, 40);
#endif

    /*Add two data series*/
    lv_chart_set_range(measure_heart_rate_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 200);
    lv_chart_set_range(measure_heart_rate_chart, LV_CHART_AXIS_PRIMARY_X, 0, 24);
    lv_chart_set_point_count(measure_heart_rate_chart, 24);
    //TODO
    //lv_obj_glue_obj(measure_heart_rate_chart, true);

#if 0
    lv_chart_series_t *ser1 = lv_chart_add_series(chart, LV_COLOR_RED);

    int16_t value[24] = {40, 62, 64, 66, 65, 67, 69, 75,
                         80, 85, 90, 100, 80, 60, 60, 80,
                         90, 110, 115, 90, 70, 68, 66, 64
                        };
    //lv_chart_set_points(chart, ser1, app_hr_data_table.today);
    lv_chart_set_points(chart, ser1, value);
#endif
    lv_chart_refresh(measure_heart_rate_chart); /*Required after direct set*/
#endif
    //add a label at top
    measure_heart_rate_label = lv_label_create(parent);
    lv_label_set_text(measure_heart_rate_label, LV_EXT_STR_GET_BY_KEY(hr_value_display_text_init, "- BPM"));
    lv_obj_align_to(measure_heart_rate_label, title_container, LV_ALIGN_OUT_BOTTOM_RIGHT, 20, 0);

    app_hr_data_ctx.hr_data = lv_ex_data_create("hr.hr_val", LV_EX_DATA_STRING);
    binding.target = measure_heart_rate_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.hr_data, &binding);
    //app_hr_data_ctx.data_handle = datac_subscribe("HR", hr_data_callback, 0);



    //add a label at bottom
    max = lv_img_create(parent);
    lv_img_set_src(max, LV_EXT_IMG_GET(up_arrow));
    lv_obj_align(max, LV_ALIGN_BOTTOM_LEFT, 60, -20);
    max_heart_rate_label = lv_label_create(parent);
    lv_label_set_text(max_heart_rate_label, "0");
    lv_obj_align_to(max_heart_rate_label, max, LV_ALIGN_OUT_RIGHT_MID, 2, 0);
    app_hr_data_ctx.max_data = lv_ex_data_create("max_val", LV_EX_DATA_STRING);
    binding.target = max_heart_rate_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.max_data, &binding);

    min = lv_img_create(parent);
    lv_img_set_src(min, LV_EXT_IMG_GET(down_arrow));
    lv_obj_align(min, LV_ALIGN_BOTTOM_RIGHT, -80, -20);
    min_heart_rate_label = lv_label_create(parent);
    lv_label_set_text(min_heart_rate_label, "0");
    lv_obj_align_to(min_heart_rate_label, min, LV_ALIGN_OUT_RIGHT_MID, 2, 0);
    app_hr_data_ctx.min_data = lv_ex_data_create("min_val", LV_EX_DATA_STRING);
    binding.target = min_heart_rate_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.min_data, &binding);

    rhr = lv_label_create(parent);
    lv_label_set_text(rhr, "RHR");
    lv_obj_align(rhr, LV_ALIGN_BOTTOM_MID, 0, 0);
    resting_heart_rate_label = lv_label_create(parent);
    lv_label_set_text(resting_heart_rate_label, "0");
    lv_obj_align_to(resting_heart_rate_label, rhr, LV_ALIGN_OUT_RIGHT_MID, 4, 0);
    app_hr_data_ctx.cur_rhr = lv_ex_data_create("cur_rhr", LV_EX_DATA_STRING);
    binding.target = resting_heart_rate_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.cur_rhr, &binding);

    //datac_rx(app_hr_data_ctx.data_handle, 2, NULL);
    //datac_rx(app_hr_data_ctx.data_handle, 26, NULL);


#endif

    //add close button
    close_btn = lv_img_create(title_container);
    lv_img_set_src(close_btn, LV_EXT_IMG_GET(img_left_arrow));
    lv_obj_align_to(close_btn, title_container, LV_ALIGN_LEFT_MID, (LV_HOR_RES_MAX >> 5), 0);
    lv_obj_add_flag(close_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(close_btn, close_btn_event_cb, LV_EVENT_ALL, 0);


    //lv_obj_set_event_cb(parent, measure_heart_rate_cavans_event_cb);
}


#endif

#if 1

void create_heart_rate_region_page(lv_obj_t *parent)
{
    lv_obj_t *title_container;
    lv_obj_t *title;
    lv_obj_t *table;
    lv_ex_binding_t binding;

    title_container = lv_obj_create(parent);
    //TODO
    //lv_obj_set_auto_realign(title_container, true);
    lv_obj_set_size(title_container, LV_HOR_RES_MAX, (LV_VER_RES_MAX / 6));
    lv_obj_align(title_container, LV_ALIGN_TOP_MID, 0, 0);


    title = lv_label_create(title_container);
    lv_label_set_text(title, LV_EXT_STR_GET_BY_KEY(hr_region, "Region"));
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);

    //and a image for ecg
    table = lv_img_create(parent);
    lv_img_set_src(table, LV_EXT_IMG_GET(hr_region));
    lv_obj_align(table, LV_ALIGN_CENTER, 0, 20);


    heart_rate_label_ul = lv_label_create(parent);
    lv_label_set_text(heart_rate_label_ul, "0");
    app_hr_data_ctx.ul_data = lv_ex_data_create("ul_val", LV_EX_DATA_STRING);
    binding.target = heart_rate_label_ul;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.ul_data, &binding);
    lv_obj_align_to(heart_rate_label_ul, table, LV_ALIGN_TOP_RIGHT, -40, 10);

    heart_rate_label_ana = lv_label_create(parent);
    lv_label_set_text(heart_rate_label_ana, "0");
    app_hr_data_ctx.ana_data = lv_ex_data_create("ana_val", LV_EX_DATA_STRING);
    binding.target = heart_rate_label_ana;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.ana_data, &binding);
    lv_obj_align_to(heart_rate_label_ana, table, LV_ALIGN_TOP_RIGHT, -40, 48);

    heart_rate_label_aer = lv_label_create(parent);
    lv_label_set_text(heart_rate_label_aer, "0");
    app_hr_data_ctx.aer_data = lv_ex_data_create("aer_val", LV_EX_DATA_STRING);
    binding.target = heart_rate_label_aer;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.aer_data, &binding);
    lv_obj_align_to(heart_rate_label_aer, table, LV_ALIGN_RIGHT_MID, -40, -6);

    heart_rate_label_hiit = lv_label_create(parent);
    lv_label_set_text(heart_rate_label_hiit, "0");
    app_hr_data_ctx.hiit_data = lv_ex_data_create("hiit_val", LV_EX_DATA_STRING);
    binding.target = heart_rate_label_hiit;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.hiit_data, &binding);
    lv_obj_align_to(heart_rate_label_hiit, table, LV_ALIGN_RIGHT_MID, -40, 28);

    heart_rate_label_warmu = lv_label_create(parent);
    lv_label_set_text(heart_rate_label_warmu, "0");
    app_hr_data_ctx.warmu_data = lv_ex_data_create("warmu_val", LV_EX_DATA_STRING);
    binding.target = heart_rate_label_warmu;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.warmu_data, &binding);
    lv_obj_align_to(heart_rate_label_warmu, table, LV_ALIGN_BOTTOM_RIGHT, -40, -26);

    //app_hr_data_ctx.data_handle = datac_subscribe("HR", hr_data_callback, 0);

    //datac_rx(app_hr_data_ctx.data_handle, 5, NULL);

}

void create_heart_rate_lastmonth_page(lv_obj_t *parent)
{
    lv_obj_t *title_container;
    lv_obj_t *title;
    //lv_obj_t *table;
    lv_obj_t *max;
    //lv_obj_t *max_val;
    lv_obj_t *min;
    //lv_obj_t *min_val;
    //lv_obj_t *average;
    lv_ex_binding_t binding;

    title_container = lv_obj_create(parent);
    //TODO
    //lv_obj_set_auto_realign(title_container, true);
    lv_obj_set_size(title_container, LV_HOR_RES_MAX, (LV_VER_RES_MAX / 4));
    lv_obj_align_to(title_container, NULL, LV_ALIGN_TOP_MID, 0, 0);


    title = lv_label_create(title_container);
    lv_label_set_text(title, LV_EXT_STR_GET_BY_KEY(hr_past_30_day, "Past 30 Days"));
    lv_obj_align_to(title, title_container, LV_ALIGN_TOP_MID, 0, 0);

    average_rhr_label = lv_label_create(parent);
    lv_label_set_text(average_rhr_label, "AVERAGE: 60");
    app_hr_data_ctx.ave_rhr = lv_ex_data_create("ave_rhr", LV_EX_DATA_STRING);
    binding.target = average_rhr_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.ave_rhr, &binding);
    lv_obj_align_to(average_rhr_label, title_container, LV_ALIGN_BOTTOM_MID, 0, 0);

    rhr_month_chart = lv_chart_create(parent);
    lv_obj_set_size(rhr_month_chart, CHART_WIDTH, CHART_HEIGTH);
    lv_obj_align_to(rhr_month_chart, parent, LV_ALIGN_CENTER, -10, 20);
    //TODO
    //lv_obj_set_parent_event(rhr_month_chart, true);
    //TODO
    //lv_page_glue_obj(rhr_month_chart, true);

    /*Add a faded are effect*/
    lv_obj_set_style_bg_opa(rhr_month_chart, LV_OPA_50, LV_PART_ITEMS | LV_STATE_DEFAULT); /*Max. opa.*/
    lv_obj_set_style_bg_grad_dir(rhr_month_chart, LV_GRAD_DIR_VER, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(rhr_month_chart, 255, LV_PART_ITEMS | LV_STATE_DEFAULT);    /*Max opa on the top*/
    lv_obj_set_style_bg_grad_stop(rhr_month_chart, 0, LV_PART_ITEMS | LV_STATE_DEFAULT);      /*Transparent on the bottom*/

    //lv_chart_set_type(chart, LV_CHART_TYPE_AREA|LV_CHART_TYPE_POINT);   /*Show lines and points too*/
    lv_chart_set_type(rhr_month_chart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/
    lv_chart_set_div_line_count(rhr_month_chart, 4, 2);
    //lv_chart_set_y_tick_length(rhr_month_chart, 1, 1);
    //lv_chart_set_x_tick_length(rhr_month_chart, 1, 1);
    /*margin for axes*/
    // TODO: HG_701, lv_chart_set_margin(chart, 40);
    lv_obj_set_style_text_font(rhr_month_chart, LV_EXT_FONT_GET(FONT_SMALL), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(rhr_month_chart, 40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(rhr_month_chart, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_dash_gap(rhr_month_chart, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_color(rhr_month_chart, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(rhr_month_chart, 80, LV_PART_MAIN | LV_STATE_DEFAULT);
    //lv_obj_set_style_local_line_blend_mode(measure_heart_rate_chart, LV_CHART_PART_SERIES_BG, LV_STATE_DEFAULT, 1);

#ifndef DISABLE_LVGL_V8
    //const char *x_tick_text = "30 Days\n15 Dyas\nYesterday  ";
    //lv_chart_set_x_tick_texts(rhr_month_chart, x_tick_text, 3, LV_CHART_AXIS_DRAW_LAST_TICK);
    lv_chart_set_axis_tick(rhr_month_chart, LV_CHART_AXIS_PRIMARY_X, 10, 5, 4, 10, true, 40);

    //const char *y_tick_text = " \n30\n60\n90\n120";
    //lv_chart_set_y_tick_texts(rhr_month_chart, y_tick_text, 5, LV_CHART_AXIS_INVERSE_LABELS_ORDER);
    lv_chart_set_axis_tick(rhr_month_chart, LV_CHART_AXIS_PRIMARY_Y, 10, 1, 4, 1, true, 40);
#endif

    /*Add two data series*/
    lv_chart_set_range(rhr_month_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 120);
    lv_chart_set_range(rhr_month_chart, LV_CHART_AXIS_PRIMARY_X, 0, 30);
    lv_chart_set_point_count(rhr_month_chart, 30);
    /*
       lv_chart_series_t *ser1 = lv_chart_add_series(chart, LV_COLOR_RED);

       int16_t value[30] = {0, 0, 0, 0, 0, 0, 0, 0, LV_CHART_POINT_DEF, LV_CHART_POINT_DEF,
                               66, 77, 88, 99, 88, 77, 66, 66, LV_CHART_POINT_DEF, LV_CHART_POINT_DEF,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
       //lv_chart_set_points(chart, ser1, app_hr_data_table.today);
       lv_chart_set_points(chart, ser1, value);
    */
    lv_chart_refresh(rhr_month_chart); /*Required after direct set*/


    //add a label at bottom
    max = lv_img_create(parent);
    lv_img_set_src(max, LV_EXT_IMG_GET(up_arrow));
    lv_obj_align(max, LV_ALIGN_BOTTOM_LEFT, 60, -20);
    max_rhr_label = lv_label_create(parent);
    lv_label_set_text(max_rhr_label, "72");
    app_hr_data_ctx.max_rhr = lv_ex_data_create("max_rhr", LV_EX_DATA_STRING);
    binding.target = max_rhr_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.max_rhr, &binding);
    lv_obj_align_to(max_rhr_label, max, LV_ALIGN_OUT_RIGHT_MID, 2, 0);


    min = lv_img_create(parent);
    lv_img_set_src(min, LV_EXT_IMG_GET(down_arrow));
    lv_obj_align(min, LV_ALIGN_BOTTOM_RIGHT, -80, -20);
    min_rhr_label = lv_label_create(parent);
    lv_label_set_text(min_rhr_label, "65");
    app_hr_data_ctx.min_rhr = lv_ex_data_create("min_rhr", LV_EX_DATA_STRING);
    binding.target = min_rhr_label;
    binding.arg_type = LV_EX_DATA_STRING;
    binding.setter = (void *)lv_label_set_text;
    lv_ex_bind_data(app_hr_data_ctx.min_rhr, &binding);
    lv_obj_align_to(min_rhr_label, min, LV_ALIGN_OUT_RIGHT_MID, 2, 0);

    //datac_rx(app_hr_data_ctx.data_handle, 33, NULL);

}


#endif

#if 0
static void heart_rate_statistics_event_cb(lv_obj_t *obj, lv_event_t event)
{
    switch (event)
    {
    case LV_EVENT_PRESSED:

        break;

    case LV_EVENT_CLICKED:

        chart_type += 1;

        if (chart_type > LV_CHART_TYPE_COLUMN)
        {
            chart_type = LV_CHART_TYPE_LINE;
        }

        rt_kprintf("heart_rate_statistics_event_cb set chart_type=%d\n", chart_type);
        lv_chart_set_type(obj, chart_type);   /*Show lines and points too*/
        break;

    case LV_EVENT_LONG_PRESSED:
        break;

    case LV_EVENT_LONG_PRESSED_REPEAT:
        break;

    case LV_EVENT_RELEASED:
        break;
    }
}


void create_heart_rate_statistics_page(lv_obj_t *parent)
{
    lv_obj_t *title_container;
    lv_obj_t *title;
    lv_obj_t *chart;
    lv_obj_t *close_btn;



    title_container = lv_cont_create(parent, NULL);
    lv_obj_set_auto_realign(title_container, true);
    lv_obj_set_size(title_container, LV_HOR_RES_MAX, (LV_VER_RES_MAX / 6));
    lv_obj_align(title_container, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);


    title = lv_label_create(title_container, NULL);
    lv_label_set_text(title, LV_EXT_STR_GET_BY_KEY(hr_heart_rate_stat, "Heartrate Stat"));
    lv_obj_align(title, NULL, LV_ALIGN_CENTER, 0, 0);

    chart = lv_chart_create(parent, NULL);
    lv_obj_set_size(chart, CHART_WIDTH, CHART_HEIGTH);
    lv_obj_align(chart, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_parent_event(chart, true);
    lv_obj_set_drag_parent(chart, true);

    //lv_chart_set_type(chart, LV_CHART_TYPE_AREA|LV_CHART_TYPE_POINT);   /*Show lines and points too*/
    lv_chart_set_type(chart, LV_CHART_TYPE_COLUMN);   /*Show lines and points too*/
    lv_chart_set_div_line_count(chart, 3, 0);
    lv_chart_set_y_tick_length(chart, 10, 2);
    lv_chart_set_x_tick_length(chart, 6, 2);
    /*margin for axes*/
    // TODO: HG_701, lv_chart_set_margin(chart, 40);


    /*Add two data series*/
    lv_chart_set_range(chart, 0, 200);
    lv_chart_set_point_count(chart, 24);
    lv_chart_series_t *ser1 = lv_chart_add_series(chart, LV_COLOR_RED);

    int16_t value[24] = {62, 62, 64, 66, 65, 67, 69, 71,
                         73, 73, 75, 74, 74, 73, 73, 72,
                         72, 73, 71, 70, 69, 68, 66, 64
                        };
    lv_chart_set_points(chart, ser1, value);
#if 0
    /*Set the next points on 'dl1'*/
    lv_chart_set_next(chart, ser1, 10);
    lv_chart_set_next(chart, ser1, 30);
    lv_chart_set_next(chart, ser1, 20);
    lv_chart_set_next(chart, ser1, 40);
    lv_chart_set_next(chart, ser1, 10);
    lv_chart_set_next(chart, ser1, 50);
    lv_chart_set_next(chart, ser1, 10);
    lv_chart_set_next(chart, ser1, 30);
    lv_chart_set_next(chart, ser1, 70);
    lv_chart_set_next(chart, ser1, 90);
#endif
#if 0
    lv_chart_series_t *ser2 = lv_chart_add_series(chart, LV_COLOR_GREEN);
    /*Directly set points on 'dl2'*/
    ser2->points[0] = 90;
    ser2->points[1] = 70;
    ser2->points[2] = 65;
    ser2->points[3] = 65;
    ser2->points[4] = 65;
    ser2->points[5] = 65;
    ser2->points[6] = 65;
    ser2->points[7] = 65;
    ser2->points[8] = 65;
    ser2->points[9] = 65;
#endif



    const char *x_tick_text = "AA\nBB\nCC\nDD\nEE";
    lv_chart_set_x_tick_texts(chart, x_tick_text, 5, LV_CHART_AXIS_SKIP_LAST_TICK);
    lv_obj_set_style_text_color(char, LV_OBJ_PART_MAIN, LV_COLOR_WHITE);

    const char *y_tick_text = "11\n22\n33\n44\n55";
    lv_chart_set_y_tick_texts(chart, y_tick_text, 5, LV_CHART_AXIS_INVERSE_LABELS_ORDER);

    lv_chart_refresh(chart); /*Required after direct set*/

    lv_obj_set_event_cb(chart, heart_rate_statistics_event_cb);


    //add close button
    close_btn = lv_img_create(title_container, NULL);
    lv_img_set_src(close_btn, LV_EXT_IMG_GET(img_left_arrow));
    lv_obj_align(close_btn, title_container, LV_ALIGN_IN_LEFT_MID, (LV_HOR_RES_MAX >> 5), 0);
    lv_obj_set_click(close_btn, true);
    lv_obj_set_event_cb(close_btn, close_btn_event_cb);

}
#endif


//const static lv_point_t valid_pos[] = {{0, 0}, {1, 0}, {2, 0} };
const static lv_point_t valid_pos[] = {{0, 0}, {0, 1}, {0, 2} };

void heart_rate_init(void)
{
    rt_uint16_t i;
    lv_obj_t *tileview;
    lv_obj_t *pages[3];

    tileview = lv_tileview_create(lv_scr_act());

    for (i = 0; i < 3; i++)
    {
        if (0 == i)
        {
            pages[i] = lv_tileview_add_tile(tileview, 0, i, LV_DIR_BOTTOM);
        }
        else if (2 == i)
        {
            pages[i] = lv_tileview_add_tile(tileview, 0, i, LV_DIR_TOP);
        }
        else
        {
            pages[i] = lv_tileview_add_tile(tileview, 0, i, LV_DIR_VER);
        }
        //TODO
        //lv_obj_set_drag_parent(pages[i], true);
    }

    create_measure_heart_rate_page(pages[0]);
    create_heart_rate_region_page(pages[1]);
    create_heart_rate_lastmonth_page(pages[2]);
    //create_measure_heart_rate_tile(pages[2]);

}


static void on_start(void)
{
    heart_rate_redraw_test_task = NULL;
    app_hr_data_ctx.active = true;
    heart_rate_init();




    if (DATA_CLIENT_INVALID_HANDLE != app_hr_data_ctx.data_handle)
    {
        app_hr_data_ctx.data_handle = datac_open();
        RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != app_hr_data_ctx.data_handle);
        ui_datac_subscribe(app_hr_data_ctx.data_handle, "HR", hr_data_callback, 0);
    }

    if (DATA_CLIENT_INVALID_HANDLE != app_hr_data_ctx.data_handle)
    {
        data_msg_t msg;
        uint8_t *msg_payload;

        msg_payload = data_service_init_msg(&msg, MSG_SERVICE_HR_DAY_TABLE_REQ, 1);
        msg_payload[0] = HRS_DAY_TABLE_LEN;
        datac_send_msg(app_hr_data_ctx.data_handle, &msg);

        msg_payload = data_service_init_msg(&msg, MSG_SERVICE_HR_MAX_MIN_REQ, 1);
        msg_payload[0] = HRS_MAX_MIN_LEN;
        datac_send_msg(app_hr_data_ctx.data_handle, &msg);



        msg_payload = data_service_init_msg(&msg, MSG_SERVICE_HR_REGION_REQ, 1);
        msg_payload[0] = HRS_REGION_LEN;
        datac_send_msg(app_hr_data_ctx.data_handle, &msg);



        msg_payload = data_service_init_msg(&msg, MSG_SERVICE_HR_MON_TABLE_REQ, 1);
        msg_payload[0] = HRS_MON_TABLE_LEN;
        datac_send_msg(app_hr_data_ctx.data_handle, &msg);

        msg_payload = data_service_init_msg(&msg, MSG_SERVICE_RHR_VALUE_REQ, 1);
        msg_payload[0] = HRS_RHR_HIST_LEN;
        datac_send_msg(app_hr_data_ctx.data_handle, &msg);
    }

}

static void on_pause(void)
{
    if (heart_rate_redraw_test_task)
    {
        lv_timer_del(heart_rate_redraw_test_task);
        heart_rate_redraw_test_task = NULL;
    }
}

static void on_resume(void)
{
    if (NULL == heart_rate_redraw_test_task)
    {
        heart_rate_redraw_test_task = lv_timer_create(measure_heart_rate_redraw,
                                      HEART_RATE_REDRAW_INTERVAL_MS, (void *)0);
    }
}

static void on_stop(void)
{
    lv_ex_data_t *data;
    //app_hr_data_ctx.hr_data = lv_ex_data_create("hr.hr_val", LV_EX_DATA_STRING);
    if (app_hr_data_ctx.hr_data)
    {
        data = app_hr_data_ctx.hr_data;
        app_hr_data_ctx.hr_data = NULL;
        lv_ex_data_delete(data);
    }
    if (app_hr_data_ctx.max_data)
    {
        data = app_hr_data_ctx.max_data;
        app_hr_data_ctx.max_data = NULL;
        lv_ex_data_delete(data);
    }
    if (app_hr_data_ctx.min_data)
    {
        data = app_hr_data_ctx.min_data;
        app_hr_data_ctx.min_data = NULL;
        lv_ex_data_delete(data);
    }
    if (app_hr_data_ctx.cur_rhr)
    {
        data = app_hr_data_ctx.cur_rhr;
        app_hr_data_ctx.cur_rhr = NULL;
        lv_ex_data_delete(data);
    }
    if (app_hr_data_ctx.ul_data)
    {
        data = app_hr_data_ctx.ul_data;
        app_hr_data_ctx.ul_data = NULL;
        lv_ex_data_delete(data);
    }
    if (app_hr_data_ctx.ana_data)
    {
        data = app_hr_data_ctx.ana_data;
        app_hr_data_ctx.ana_data = NULL;
        lv_ex_data_delete(data);
    }
    if (app_hr_data_ctx.aer_data)
    {
        data = app_hr_data_ctx.aer_data;
        app_hr_data_ctx.aer_data = NULL;
        lv_ex_data_delete(data);
    }
    if (app_hr_data_ctx.hiit_data)
    {
        data = app_hr_data_ctx.hiit_data;
        app_hr_data_ctx.hiit_data = NULL;
        lv_ex_data_delete(data);
    }
    if (app_hr_data_ctx.warmu_data)
    {
        data = app_hr_data_ctx.warmu_data;
        app_hr_data_ctx.warmu_data = NULL;
        lv_ex_data_delete(data);
    }
    if (app_hr_data_ctx.ave_rhr)
    {
        data = app_hr_data_ctx.ave_rhr;
        app_hr_data_ctx.ave_rhr = NULL;
        lv_ex_data_delete(data);
    }
    if (app_hr_data_ctx.max_rhr)
    {
        data = app_hr_data_ctx.max_rhr;
        app_hr_data_ctx.max_rhr = NULL;
        lv_ex_data_delete(data);
    }
    if (app_hr_data_ctx.min_rhr)
    {
        data = app_hr_data_ctx.min_rhr;
        app_hr_data_ctx.min_rhr = NULL;
        lv_ex_data_delete(data);
    }



    if (scr_heart_rate_canvas_buffer)
    {
        lv_mem_free(scr_heart_rate_canvas_buffer);
        scr_heart_rate_canvas_buffer = RT_NULL;
    }
    heart_rate_bpm = 0;
    app_hr_data_ctx.active = false;
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

    return 0;
}

BUILTIN_APP_EXPORT(LV_EXT_STR_ID(heart_rate), LV_EXT_IMG_GET(img_activity), APP_ID, app_main);


