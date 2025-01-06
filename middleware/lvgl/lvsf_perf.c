
#include "littlevgl2rtt.h"

#ifdef LV_USE_LVSF
#include "lvsf.h"

static uint8_t obj_detail = 0;

typedef struct _lv_obj_name_t

{
    const char *name;
    const lv_obj_class_t *class_p;
} obj2nameTypedef;

const static obj2nameTypedef obj2name_array[] =
{

    {"obj", &lv_obj_class},
#if LV_USE_BTN
    {"btn", &lv_btn_class},
#endif
#if  LV_USE_IMG
    {"img", &lv_img_class},
#endif /* LV_USE_IMG */

#if LV_USE_LINE
    {"line", &lv_line_class},
#endif

#if LV_USE_BTNMATRIX
    {"btnmatrix", &lv_btnmatrix_class},
#endif

#if LV_USE_BAR
    {"bar", &lv_bar_class},
#endif

#if LV_USE_SLIDER
    {"slider", &lv_slider_class},
#endif

#if LV_USE_TABLE
    {"table", &lv_table_class},
#endif

#if LV_USE_CHECKBOX
    {"checkbox", &lv_checkbox_class},
#endif

#if LV_USE_SWITCH
    {"switch", &lv_switch_class},
#endif

#if LV_USE_CHART
    {"chart", &lv_chart_class},
#endif

#if LV_USE_ROLLER
    {"roller", &lv_roller_class},
#endif

#if LV_USE_DROPDOWN
    {"dropdown", &lv_dropdown_class},
    {"dropdownlist", &lv_dropdownlist_class},
#endif

#if LV_USE_ARC
    {"arc", &lv_arc_class},
#endif


#if LV_USE_SPINNER
    {"spinner", &lv_spinner_class},
#endif

#if LV_USE_TEXTAREA
    {"textarea", &lv_textarea_class},

#endif

#if LV_USE_CALENDAR
    {"calendar", &lv_calendar_class},

#endif

#if LV_USE_CALENDAR_HEADER_ARROW
    {"calendar_header_arrow", &lv_calendar_header_arrow_class},

#endif

#if LV_USE_CALENDAR_HEADER_DROPDOWN
    {"calendar_header_dropdown", &lv_calendar_header_dropdown_class},

#endif

#if LV_USE_KEYBOARD
    {"keyboard", &lv_keyboard_class},

#endif
#if LV_USE_LIST
    {"list", &lv_list_class},

    {"list_text", &lv_list_text_class},

    {"list_btn", &lv_list_btn_class},

#endif
#if LV_USE_MENU
    {"menu", &lv_menu_class},

    {"menu_sidebar_cont", &lv_menu_sidebar_cont_class},

    {"menu_main_cont", &lv_menu_main_cont_class},

    {"menu_cont", &lv_menu_cont_class},

    {"menu_sidebar_header_cont", &lv_menu_sidebar_header_cont_class},
    {"menu_main_header_cont", &lv_menu_main_header_cont_class},

    {"menu_page", &lv_menu_page_class},

    {"menu_section", &lv_menu_section_class},

    {"menu_separator", &lv_menu_separator_class},

#endif
#if LV_USE_MSGBOX
    {"msgbox", &lv_msgbox_class},

    {"msgbox_backdrop", &lv_msgbox_backdrop_class},

#endif
#if LV_USE_SPINBOX
    {"spinbox", &lv_spinbox_class},

#endif
#if LV_USE_TILEVIEW
    {"tileview", &lv_tileview_class},

    {"tileview_tile", &lv_tileview_tile_class},

#endif

#if LV_USE_TABVIEW
    {"tabview", &lv_tabview_class},

#endif

#if LV_USE_WIN
    {"win", &lv_win_class},

#endif

#if LV_USE_COLORWHEEL
    {"colorwheel", &lv_colorwheel_class},

#endif

#if LV_USE_LED
    {"led", &lv_led_class},

#endif
#if LVSF_USE_HEADER
    {"lvsfheader", &lv_lvsfheader_class},

#endif
#if LVSF_USE_COMPOSITE
    {"lvsfcomp", &lv_lvsfcomp_class},

#endif
#if LVSF_USE_CORNER
    {"lvsfcorner", &lv_lvsfcorner_class},

#endif


};


const char *get_obj_name(const lv_obj_t *obj)
{
    for (uint32_t i = 0; i < (sizeof(obj2name_array) / sizeof(obj2name_array[0])); i++)
        if (lv_obj_check_type(obj, obj2name_array[i].class_p))
            return obj2name_array[i].name;


    return "Unknow";
}

#ifdef PKG_USING_SYSTEMVIEW
#include "SEGGER_SYSVIEW.h"
#include "lv_gc.h"
#include "lv_refr.h"
#include "lv_img.h"
#include "lv_indev.h"
#include "rthw.h"
extern void lv_enter_func(U32 id, const char *task_name, U32 param1, U32 param2);
extern void lv_exit_func(U32 id);
static bool systemview_lv_debug_en = false;


void _cbSendLVTaskinfo(const lv_timer_t *t)
{
    SEGGER_SYSVIEW_TASKINFO Info;

    register rt_base_t level;
    char task_name[20];

    rt_sprintf(task_name, "lv_task %x", (U32)t->timer_cb);

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    rt_memset(&Info, 0, sizeof(Info));
    Info.TaskID = (U32)t->timer_cb;
    Info.sName = &task_name[0];
    Info.Prio = 0;//t->prio;

    SEGGER_SYSVIEW_SendTaskInfo(&Info);
    /* enable interrupt */
    rt_hw_interrupt_enable(level);
}

void _cbSendLvTaskList(void)
{
    lv_timer_t *tmp;

    /*Create task lists in order of priority from high to low*/
    tmp = _lv_ll_get_head(&LV_GC_ROOT(_lv_timer_ll));
    while (tmp != NULL)
    {
        //_cbSendLVTaskinfo(tmp);       Display running task only
        tmp = _lv_ll_get_next(&LV_GC_ROOT(_lv_timer_ll), tmp);
    }
}

void lv_debug_task_create(const lv_timer_t *t)
{
    if (!systemview_lv_debug_en) return;

    _cbSendLVTaskinfo(t);
}

void lv_debug_task_terminate(const lv_timer_t *t)
{
    if (!systemview_lv_debug_en) return;

    //SEGGER_SYSVIEW_OnTaskTerminate((U32) t);
}
void lv_debug_task_start_exec(const lv_timer_t *t)
{
    char task_name[20];
    if (!systemview_lv_debug_en) return;

    rt_sprintf(task_name, "lv_task %x", (U32)t->timer_cb);

    lv_enter_func((U32)t->timer_cb, &task_name[0], 0, 0);
}

void lv_debug_task_stop_exec(const lv_timer_t *t)
{
    if (!systemview_lv_debug_en) return;

    lv_exit_func((U32)t->timer_cb);
}


void lv_debug_obj_start_draw(const lv_obj_t *obj, const lv_area_t *mask)
{
    if (!systemview_lv_debug_en) return;

    if (obj_detail)
    {
        char task_name[32];
        char str[32];
        register rt_base_t level;
        U32 param1 = 0, param2 = 0;
        rt_sprintf(task_name, "%s [%d,%d,%d,%d]",
                   get_obj_name(obj),
                   obj->coords.x1,
                   obj->coords.y1,
                   obj->coords.x2,
                   obj->coords.y2);

        rt_sprintf(str, "mask[%d,%d,%d,%d]",
                   mask->x1,
                   mask->y1,
                   mask->x2,
                   mask->y2);

#if  LV_USE_IMG
        if (lv_obj_check_type(obj, &lv_img_class))
        {
            param1 = (U32)lv_img_get_src((lv_obj_t *)obj);
            param2 = (U32)lv_img_get_angle((lv_obj_t *)obj);
        }
#endif /* LV_USE_IMG */

        /* disable interrupt */
        level = rt_hw_interrupt_disable();
        lv_enter_func((U32)obj, &task_name[0], param1, param2);
        SEGGER_SYSVIEW_Print(str);
        /* enable interrupt */
        rt_hw_interrupt_enable(level);
    }
}
void lv_debug_obj_stop_draw(const lv_obj_t *obj, const lv_area_t *mask)
{
    if (!systemview_lv_debug_en) return;

    if (obj_detail)
    {
        lv_exit_func((U32)obj);
    }
}



void lv_debug_mark_start(uint32_t id, const char *desc)
{
    if (!systemview_lv_debug_en) return;

    SEGGER_SYSVIEW_OnUserStart(id);
    if (desc) SEGGER_SYSVIEW_Print(desc);

}


void lv_debug_mark_stop(uint32_t id)
{
    if (!systemview_lv_debug_en) return;

    //SEGGER_SYSVIEW_OnTaskStopReady((U32) id, 1);

    //SEGGER_SYSVIEW_OnTaskStartExec((U32) id);
    //SEGGER_SYSVIEW_OnTaskStopExec();


    SEGGER_SYSVIEW_OnUserStop(id);
}


void lv_debug_vdb_start_flush(const lv_color_t *color_p, const lv_area_t *area)
{
    char str[64];
    if (!systemview_lv_debug_en) return;

    SEGGER_SYSVIEW_OnUserStart((U32) color_p);

    rt_sprintf(str, "vdb buf=%x area=%d,%d,%d,%d",
               color_p,
               area->x1,
               area->y1,
               area->x2,
               area->y2);
    SEGGER_SYSVIEW_Print(str);
}

void lv_debug_vdb_stop_flush(const lv_color_t *color_p, const lv_area_t *area)
{
    if (!systemview_lv_debug_en) return;

    SEGGER_SYSVIEW_OnUserStop((U32) color_p);
}


void lv_debug_gpu_start(uint8_t type, int16_t p1, uint32_t p2, const lv_area_t *output_coords)
{
    char str[64];
    if (!systemview_lv_debug_en) return;


    switch (type)
    {
    case 0: //DRV_EPIC_COLOR_BLEND
    case 4: //DRV_EPIC_LETTER_BLEND
        rt_sprintf(str, "EPIC Blend area=%d,%d,%d,%d",
                   output_coords->x1,
                   output_coords->y1,
                   output_coords->x2,
                   output_coords->y2);
        break;

    case 1: //DRV_EPIC_COLOR_FILL
        rt_sprintf(str, "EPIC Fill opa=%d, color=0x%x area=%d,%d,%d,%d", p1, p2,
                   output_coords->x1,
                   output_coords->y1,
                   output_coords->x2,
                   output_coords->y2);
        break;

    case 2: //DRV_EPIC_IMG_ROT
        rt_sprintf(str, "EPIC Rotate angle=%d, scale=%d, area=%d,%d,%d,%d", p1, p2,
                   output_coords->x1,
                   output_coords->y1,
                   output_coords->x2,
                   output_coords->y2);
        break;
    case 3: //DRV_EPIC_IMG_COPY
        rt_sprintf(str, "EPIC copy angle=%d, scale=%d, area=%d,%d,%d,%d", p1, p2,
                   output_coords->x1,
                   output_coords->y1,
                   output_coords->x2,
                   output_coords->y2);
        break;

    default:
        RT_ASSERT(0);//Fix me.
        break;
    }

    lv_debug_mark_start(0xAAAAAAAA, str);

}

void lv_debug_gpu_stop(void)
{
    if (!systemview_lv_debug_en) return;

    lv_debug_mark_stop(0xAAAAAAAA);

}

void lv_debug_lcd_flush_start(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    char str[32];
    if (!systemview_lv_debug_en) return;

    rt_sprintf(str, "lcd flush %d,%d,%d,%d", x1, y1, x2, y2);
    lv_debug_mark_start(0xBBBBBBBB, str);
}
void lv_debug_lcd_flush_stop(void)
{
    if (!systemview_lv_debug_en) return;

    lv_debug_mark_stop(0xBBBBBBBB);
}

void lv_debug_enable(bool enable)
{
    systemview_lv_debug_en = enable;

    if (enable)
    {
        _cbSendLvTaskList();
    }
}

void lv_debug_init(void)
{

}


#elif defined(USING_PROFILER)
#include "profiler.h"

typedef struct
{
    uint16_t task_create_evt_id;
    uint16_t task_terminate_evt_id;
    uint16_t task_exec_evt_id;
    uint16_t obj_draw_evt_id;
    uint16_t vdb_flush_evt_id;
    uint16_t gpu_rot_evt_id;
    uint16_t lcd_flush_evt_id;
} _lv_debug_ctx_t;

static _lv_debug_ctx_t _lv_debug_ctx;

#include "../lv_misc/lv_gc.h"
#include "lv_refr.h"
#include "lv_indev.h"

void lv_debug_task_create(const lv_timer_t *t)
{
    struct log_event_buf buf;
    char str[32];

    profiler_log_start(&buf);
    profiler_log_add_mem_address(&buf, (void *)(uint32_t)_lv_debug_ctx.task_create_evt_id);

    if (_lv_disp_refr_task == t->timer_cb)
        rt_snprintf(str, sizeof(str), "lv_refr %x", t->user_data);
    else if (_lv_indev_read_task == t->timer_cb)
        rt_snprintf(str, sizeof(str), "lv_indev %x", t->user_data);
    else
        rt_snprintf(str, sizeof(str), "lv_task %x", t->timer_cb);

    profiler_log_add_string(&buf, str);

    profiler_log_encode_u32(&buf, (uint32_t)t);
    profiler_log_encode_u32(&buf, 0/*t->prio*/);
    profiler_log_send(&buf, _lv_debug_ctx.task_create_evt_id);
}

void lv_debug_task_terminate(const lv_timer_t *t)
{
    struct log_event_buf buf;

    profiler_log_start(&buf);
    profiler_log_add_mem_address(&buf, (void *)(uint32_t)_lv_debug_ctx.task_terminate_evt_id);
    profiler_log_encode_u32(&buf, (uint32_t)t);
    profiler_log_send(&buf, _lv_debug_ctx.task_terminate_evt_id);
}

void lv_debug_task_start_exec(const lv_timer_t *t)
{
    struct log_event_buf buf;
    uint32_t task_type;

    profiler_log_start(&buf);
    profiler_log_add_mem_address(&buf, (void *)(uint32_t)_lv_debug_ctx.task_exec_evt_id);
    profiler_log_encode_u32(&buf, (uint32_t)t);
    if (_lv_disp_refr_task == t->timer_cb)
    {
        task_type = 0;
    }
    else if (_lv_indev_read_task == t->timer_cb)
    {
        task_type = 1;
    }
    else
    {
        task_type = 2;
    }
    profiler_log_encode_u32(&buf, task_type);

    profiler_log_send(&buf, _lv_debug_ctx.task_exec_evt_id);

    profiler_log_event_execution(_lv_debug_ctx.task_exec_evt_id, true);
}

void lv_debug_task_stop_exec(const lv_timer_t *t)
{
    profiler_log_event_execution(_lv_debug_ctx.task_exec_evt_id, false);
}


void lv_debug_obj_start_draw(lv_obj_t *obj, const lv_area_t *mask)
{
    lv_obj_type_t type;
    struct log_event_buf buf;

    lv_obj_get_type(obj, &type);

    profiler_log_start(&buf);
    profiler_log_add_mem_address(&buf, (void *)(uint32_t)_lv_debug_ctx.obj_draw_evt_id);
    profiler_log_add_string(&buf, type.type[0]);
    profiler_log_encode_u32(&buf, (uint32_t)obj);
    profiler_log_encode_u32(&buf, obj->coords.x1);
    profiler_log_encode_u32(&buf, obj->coords.y1);
    profiler_log_encode_u32(&buf, obj->coords.x2);
    profiler_log_encode_u32(&buf, obj->coords.y2);
    profiler_log_encode_u32(&buf, mask->x1);
    profiler_log_encode_u32(&buf, mask->y1);
    profiler_log_encode_u32(&buf, mask->x2);
    profiler_log_encode_u32(&buf, mask->y2);

    profiler_log_send(&buf, _lv_debug_ctx.obj_draw_evt_id);
    profiler_log_event_execution(_lv_debug_ctx.obj_draw_evt_id, true);
}

void lv_debug_obj_stop_draw(lv_obj_t *obj, const lv_area_t *mask)
{
    profiler_log_event_execution(_lv_debug_ctx.obj_draw_evt_id, false);
}

void lv_debug_vdb_start_flush(lv_color_t *color_p, const lv_area_t *area)
{
    char str[64];

    struct log_event_buf buf;

    profiler_log_start(&buf);
    profiler_log_add_mem_address(&buf, (void *)(uint32_t)_lv_debug_ctx.vdb_flush_evt_id);
    profiler_log_add_mem_address(&buf, color_p);
    profiler_log_encode_u32(&buf, area->x1);
    profiler_log_encode_u32(&buf, area->y1);
    profiler_log_encode_u32(&buf, area->x2);
    profiler_log_encode_u32(&buf, area->y2);

    profiler_log_send(&buf, _lv_debug_ctx.vdb_flush_evt_id);
    profiler_log_event_execution(_lv_debug_ctx.vdb_flush_evt_id, true);
}

void lv_debug_vdb_stop_flush(lv_color_t *color_p, const lv_area_t *area)
{
    profiler_log_event_execution(_lv_debug_ctx.vdb_flush_evt_id, false);
}

void lv_debug_gpu_start(uint8_t type, int16_t angle, uint32_t scale, const lv_area_t *output_coords)
{
    struct log_event_buf buf;

    profiler_log_start(&buf);
    profiler_log_add_mem_address(&buf, (void *)(uint32_t)_lv_debug_ctx.gpu_rot_evt_id);
    profiler_log_encode_u32(&buf, (uint32_t)type);
    profiler_log_encode_u32(&buf, (uint32_t)angle);
    profiler_log_encode_u32(&buf, scale);
    profiler_log_encode_u32(&buf, output_coords->x1);
    profiler_log_encode_u32(&buf, output_coords->y1);
    profiler_log_encode_u32(&buf, output_coords->x2);
    profiler_log_encode_u32(&buf, output_coords->y2);

    profiler_log_send(&buf, _lv_debug_ctx.gpu_rot_evt_id);
    profiler_log_event_execution(_lv_debug_ctx.gpu_rot_evt_id, true);
}

void lv_debug_gpu_stop(void)
{
    profiler_log_event_execution(_lv_debug_ctx.gpu_rot_evt_id, false);
}

void lv_debug_lcd_flush_start(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    lv_obj_type_t type;
    struct log_event_buf buf;

    profiler_log_start(&buf);
    profiler_log_add_mem_address(&buf, (void *)(uint32_t)_lv_debug_ctx.lcd_flush_evt_id);
    profiler_log_encode_u32(&buf, x1);
    profiler_log_encode_u32(&buf, y1);
    profiler_log_encode_u32(&buf, x2);
    profiler_log_encode_u32(&buf, y2);

    profiler_log_send(&buf, _lv_debug_ctx.lcd_flush_evt_id);
    profiler_log_event_execution(_lv_debug_ctx.lcd_flush_evt_id, true);
}

void lv_debug_lcd_flush_stop(void)
{
    profiler_log_event_execution(_lv_debug_ctx.lcd_flush_evt_id, false);
}


static void _lv_debug_register_task_create_evt(void)
{
    const char *labels[] = {"mem_address", "task", "id", "prio"};
    enum profiler_arg types[] = {PROFILER_ARG_U32, PROFILER_ARG_STRING, PROFILER_ARG_U32, PROFILER_ARG_U32};

    _lv_debug_ctx.task_create_evt_id = profiler_register_event_type(
                                           "task_create",
                                           labels, types, 4);
}

static void _lv_debug_register_task_terminate_evt(void)
{
    const char *labels[] = {"mem_address", "user_data"};
    enum profiler_arg types[] = {PROFILER_ARG_U32, PROFILER_ARG_U32};

    _lv_debug_ctx.task_terminate_evt_id = profiler_register_event_type(
            "task_terminate",
            labels, types, 2);
}

static void _lv_debug_register_task_exec_evt(void)
{
    const char *labels[] = {"mem_address", "task_id", "task_type"};
    enum profiler_arg types[] = {PROFILER_ARG_U32, PROFILER_ARG_U32, PROFILER_ARG_U32};

    _lv_debug_ctx.task_exec_evt_id = profiler_register_event_type(
                                         "task_exec",
                                         labels, types, 3);
}

static void _lv_debug_register_obj_draw_evt(void)
{
    const char *labels[] = {"mem_address", "obj_type", "obj_addr",
                            "x0", "y0", "x1", "y1",
                            "m_x0", "m_y0", "m_x1", "m_y1"
                           };
    enum profiler_arg types[] = {PROFILER_ARG_U32, PROFILER_ARG_STRING, PROFILER_ARG_U32,
                                 PROFILER_ARG_S32, PROFILER_ARG_S32, PROFILER_ARG_S32, PROFILER_ARG_S32,
                                 PROFILER_ARG_S32, PROFILER_ARG_S32, PROFILER_ARG_S32, PROFILER_ARG_S32
                                };

    _lv_debug_ctx.obj_draw_evt_id = profiler_register_event_type(
                                        "obj_draw",
                                        labels, types, sizeof(types) / sizeof(types[0]));
}

static void _lv_debug_register_vdb_flush_evt(void)
{
    const char *labels[] = {"mem_address", "buf_act",
                            "x0", "y0", "x1", "y1"
                           };
    enum profiler_arg types[] = {PROFILER_ARG_U32, PROFILER_ARG_U32,
                                 PROFILER_ARG_S32, PROFILER_ARG_S32, PROFILER_ARG_S32, PROFILER_ARG_S32
                                };

    _lv_debug_ctx.vdb_flush_evt_id = profiler_register_event_type(
                                         "vdb_flush",
                                         labels, types, sizeof(types) / sizeof(types[0]));
}

static void _lv_debug_register_gpu_evt(void)
{
    const char *labels[] = {"mem_address", "type", "angle", "zoom",
                            "x0", "y0", "x1", "y1"
                           };
    enum profiler_arg types[] = {PROFILER_ARG_U32, PROFILER_ARG_U32, PROFILER_ARG_S32, PROFILER_ARG_U32,
                                 PROFILER_ARG_S32, PROFILER_ARG_S32, PROFILER_ARG_S32, PROFILER_ARG_S32
                                };

    _lv_debug_ctx.gpu_rot_evt_id = profiler_register_event_type(
                                       "gpu",
                                       labels, types, sizeof(types) / sizeof(types[0]));
}

static void _lv_debug_register_lcd_flush_evt(void)
{
    const char *labels[] = {"mem_address", "x0", "y0", "x1", "y1"};
    enum profiler_arg types[] = {PROFILER_ARG_U32,
                                 PROFILER_ARG_S32, PROFILER_ARG_S32, PROFILER_ARG_S32, PROFILER_ARG_S32
                                };

    _lv_debug_ctx.lcd_flush_evt_id = profiler_register_event_type(
                                         "lcd_flush",
                                         labels, types, sizeof(types) / sizeof(types[0]));
}


void lv_debug_enable(bool enable)
{
}

void lv_debug_init(void)
{
    profiler_init();

    _lv_debug_register_task_create_evt();
    _lv_debug_register_task_terminate_evt();
    _lv_debug_register_task_exec_evt();
    _lv_debug_register_obj_draw_evt();
    _lv_debug_register_vdb_flush_evt();
    _lv_debug_register_gpu_evt();
    _lv_debug_register_lcd_flush_evt();
}

#else

void lv_debug_enable(bool enable)
{
}

void lv_debug_init(void)
{
}

#endif


#if defined(FINSH_USING_MSH)&&!defined(PY_GEN)
#include <finsh.h>

static rt_err_t perf_cfg(int argc, char **argv)
{
    if (strcmp(argv[1], "obj") == 0)
    {
        if (argc > 2)
        {
            obj_detail = (uint8_t)strtoul(argv[2], 0, 10);
        }
        else
        {
        }
        rt_kprintf("obj_detail = %d\n", obj_detail);
    }

    return 0;
}

MSH_CMD_EXPORT(perf_cfg, perfromance log config);

static rt_err_t en_lv_perf_debug(int argc, char **argv)
{
    bool en = (strcmp(argv[1], "1") == 0) ? true : false;

    lv_debug_enable(en);
    return 0;
}

MSH_CMD_EXPORT(en_lv_perf_debug, Enable lvgl perfomance debug log);
#endif

#endif /* LV_USE_LVSF */
