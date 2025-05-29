/*********************
 *      INCLUDES
 *********************/
#include <rtthread.h>
#include <rtdevice.h>
#include "littlevgl2rtt.h"
#include "lvgl.h"
#include "lvsf.h"
#include "gui_app_fwk.h"
#ifdef DL_APP_SUPPORT
    #include "gui_dl_app_utils.h"
    #include "dlfcn.h"
#endif
#define DBG_LEVEL  DBG_LOG

//#include "EventRecorder.h"
#include "lv_ext_resource_manager.h"
#include "cell_transform.h"
#include "log.h"
#include "custom_trans_anim.h"


LV_IMG_DECLARE(human1);
LV_IMG_DECLARE(human2);
LV_IMG_DECLARE(eco);
LV_IMG_DECLARE(weather);
LV_IMG_DECLARE(house);
LV_IMG_DECLARE(clock_80);


LV_IMG_DECLARE(img_activity);
LV_IMG_DECLARE(img_alarm);
LV_IMG_DECLARE(img_alarm_2);
LV_IMG_DECLARE(img_calendar);
LV_IMG_DECLARE(img_camera);
LV_IMG_DECLARE(img_clock);
LV_IMG_DECLARE(img_group);
LV_IMG_DECLARE(img_itunes);
LV_IMG_DECLARE(img_mail);
LV_IMG_DECLARE(img_maps);
LV_IMG_DECLARE(img_messages);
LV_IMG_DECLARE(img_passbook);
LV_IMG_DECLARE(img_phone);
LV_IMG_DECLARE(img_photos);
LV_IMG_DECLARE(img_remote);
LV_IMG_DECLARE(img_settings);
LV_IMG_DECLARE(img_stocks);
LV_IMG_DECLARE(img_stopwatch);
LV_IMG_DECLARE(img_workout);
LV_IMG_DECLARE(img_world_clock);
//LV_IMG_DECLARE(celluar);


#define APP_ID "Main"

//#define DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE

/*a virtual circle include gap between icons*/
#if (LV_VER_RES_MAX > LV_HOR_RES_MAX)
    #define ICON_OUTER_RADIUS  (LV_VER_RES_MAX / 9)
#else
    #define ICON_OUTER_RADIUS  (LV_HOR_RES_MAX / 9)
#endif

#define ICON_OUTER_DIAMETER  (ICON_OUTER_RADIUS * 2)
/*full fill with picture*/
#define ICON_INNER_RADIUS  ((ICON_OUTER_RADIUS * 8)/9)
#define ICON_INNER_DIAMETER  (ICON_INNER_RADIUS * 2)


#define MAX_APP_ROW_NUM   16
#define MAX_APP_COL_NUM   16

#define ICON_IMG_WIDTH      ICON_OUTER_DIAMETER //ICON_INNER_DIAMETER
#define ICON_IMG_HEIGHT     ICON_OUTER_DIAMETER //ICON_INNER_DIAMETER

#if (MAX_APP_ROW_NUM > MAX_APP_COL_NUM)
    #define PAGE_SCRL_WIDTH   ((ICON_OUTER_DIAMETER * (MAX_APP_ROW_NUM - 1)) + LV_HOR_RES_MAX)
#else
    #define PAGE_SCRL_WIDTH   ((ICON_OUTER_DIAMETER * (MAX_APP_COL_NUM - 1)) + LV_HOR_RES_MAX)
#endif

#define PAGE_SCRL_HEIGHT  (PAGE_SCRL_WIDTH * 10 / 7)

/* Columun0 Row0 icon pivot coordinate*/
#define C0R0_COORD_X (PAGE_SCRL_WIDTH >> 1)
#define C0R0_COORD_Y (0)


//#define DEBUG_APP_MAINMENU_DISPLAY_ICON_PARAM

#ifndef DEBUG_APP_MAINMENU_DISPLAY_ICON_PARAM
    #define   LIMIT_RECT_WIDTH   (LV_HOR_RES_MAX - 16)
    #define   LIMIT_RECT_HEIGHT  (LV_VER_RES_MAX - 20)
    #if (LV_VER_RES_MAX > LV_HOR_RES_MAX)
        #define   LIMIT_ROUND_RADIUS (LV_VER_RES_MAX >> 1)
    #else
        #define   LIMIT_ROUND_RADIUS (LV_HOR_RES_MAX >> 1)
    #endif
#else

    uint16_t LIMIT_RECT_WIDTH   = (LV_HOR_RES_MAX - 16);
    uint16_t LIMIT_RECT_HEIGHT  = (LV_VER_RES_MAX - 20);
    uint16_t LIMIT_ROUND_RADIUS = (LV_VER_RES_MAX >> 1);

    uint16_t LIMIT_ENABLE = 1;

#endif /* DEBUG_APP_MAINMENU_DISPLAY_ICON_PARAM */


#if defined(LCD_USING_ROUND_TYPE1) || defined(LCD_USING_ROUND_TYPE2_EVB_Z0) || defined(LCD_USING_ROUND_TYPE1_EVB_Z0)
    #define APP_MAINMENU_ROUND_SCREEN
#endif


/**
 * iterate over builtin app list
 */
extern const builtin_app_desc_t *gui_builtin_app_list_open(void);
extern const builtin_app_desc_t *gui_builtin_app_list_get_next(const builtin_app_desc_t *ptr_app);
extern void gui_builtin_app_list_close(const builtin_app_desc_t *ptr_app);


/**
 ***********************  Dynamic load app description ************************************************
 */
/**
 * system registered dl-app file description
 */
typedef struct
{
    char id[GUI_APP_ID_MAX_LEN];                    //!< dl_app id,an unique character id of an app (both built-in app and dl app)
    char dir[GUI_DL_APP_MAX_FILE_PATH_LEN];         //!< dl_app store root directory
    char name[GUI_APP_NAME_MAX_LEN];                //!< dl_app display name
    char icon[GUI_DL_APP_MAX_FILE_PATH_LEN];        //!< dl_app display icon relative path, base on it's root directory
    char exe_file[GUI_DL_APP_MAX_FILE_PATH_LEN];    //!< dl_app executable file relative path, base on root directory
} dl_app_reg_desc_t;

/**
 * iterate over registed dl_app_list
 */
extern const char *gui_dl_app_list_open(void);
extern rt_err_t gui_dl_app_list_get_next(const char **ppf_buff, dl_app_reg_desc_t *record);
extern void gui_dl_app_list_close(const char *ptr_app);

#define GUI_APP_CMD_MAX_LEN 32

typedef struct
{
    char name[GUI_APP_NAME_MAX_LEN];
    lv_img_dsc_t icon;
    char cmd[GUI_APP_CMD_MAX_LEN];

    uint8_t row;   //!< TODO: remove me
    uint8_t col;  //!< TODO: remove me


    rt_list_t node;
} app_mainmenu_item_t;

typedef struct
{
    uint8_t num;
    lv_obj_t *scr;
    lv_obj_t *pg_obj;

    uint8_t row_num;

    lv_obj_t **list;
#ifdef DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE
    lv_obj_t **label_list;
    lv_obj_t *param_ctrl[4];
#endif

    //Pivot before transformed
    lv_point_t *icon_pivot;

    /* current screen centern icon*/
    lv_obj_t *cicon;

    /*Focus/Zoom anim obj*/
    lv_obj_t *anim_obj;

    /*
        RANGE(0 ~ 2)

        = 1  no zoom
        < 1  zoom out
        > 1  zoom in

        NOTE:
        Icons only transform while zoom belong (0, 1],
        and zoom will be applied as tranformation ratio.
    */
    float zoom;
    float last_zoom;
    lv_point_t comp_vect;

    lv_indev_t *indev;
    bool springback_open;

    /*
        We NOT use lv_obj scroll, so here are scroll states
    */
    bool scroll_actived;
    lv_point_t scroll_sum;
} app_mainmenu_ctx_t;

#ifndef BSP_USING_LVGL_INPUT_AGENT
    static
#endif
app_mainmenu_ctx_t app_mainmenu_ctx;
//static rt_list_t app_list;

static void icon_event_callback(lv_event_t *e);
static void page_event_callback(lv_event_t *e);
static void get_icon_col_row(uint16_t idx, uint16_t *p_col, uint16_t *p_row);
static int layout_icon_transform(uint32_t row_idx, uint32_t col_idx, float *p_float_x, float *p_float_y, float *p_float_icon_w, float *p_float_icon_h, float *p_float_pivot_r);
static void app_mainmenu_icons_transform(bool force_refresh);


static bool limit_square(lv_area_t *parent_area, float *x, float *y, float *icon_r)
{
    float res_x1, res_x2, res_y1, res_y2;


    /* Get the smaller area from 'a1_p' and 'a2_p' */
    res_x1 = LV_MAX((float) parent_area->x1, *x - *icon_r);
    res_y1 = LV_MAX((float) parent_area->y1, *y - *icon_r);
    res_x2 = LV_MIN((float) parent_area->x2, *x + *icon_r);
    res_y2 = LV_MIN((float) parent_area->y2, *y + *icon_r);

    /*If x1 or y1 greater then x2 or y2 then the areas union is empty*/
    bool union_ok = true;
    if ((res_x1 > res_x2) || (res_y1 > res_y2))
    {
        *icon_r = 0;
        union_ok = false;
    }
    else
    {
        float new_w, new_h;

        new_w = res_x2 - res_x1 + 1;
        new_h = res_y2 - res_y1 + 1;

        *icon_r = LV_MIN((new_w / 2), (new_h / 2));
        *x = res_x1 + (new_w / 2);
        *y = res_y1 + (new_h / 2);
    }

    return union_ok;
}

static int cal_dist(uint16_t x, uint16_t y, lv_point_t *pivot)
{
    int r;

    r = (x - pivot->x) * (x - pivot->x) + (y - pivot->y) * (y - pivot->y);
    {
        lv_sqrt_res_t ds;
        lv_sqrt(r, &ds, 0x8000);
        r = ds.i;
    }
    return r;
}

static void limit_round(uint16_t limit_r, lv_point_t *zoom_pivot, lv_coord_t *x, lv_coord_t *y, uint16_t *icon_r, uint16_t pivot_r)
{
    if (pivot_r == 0)
        pivot_r = cal_dist(*x, *y, zoom_pivot);
    if (pivot_r + *icon_r > limit_r)
    {
        if (pivot_r - *icon_r < limit_r)
        {
            int32_t new_pivot_r;
            int32_t old_w, old_h;

            old_w = *x - zoom_pivot->x;
            old_h = *y - zoom_pivot->y;

            *icon_r = (limit_r - (pivot_r - *icon_r)) >> 1;
            new_pivot_r = limit_r - *icon_r;

            *x += (old_w * new_pivot_r / pivot_r) - old_w;
            *y += (old_h * new_pivot_r / pivot_r) - old_h;

        }
        else
            *icon_r = 0;
    }
}

static void limit_round2(float limit_r, lv_point_t *zoom_pivot, float *x, float *y, float *icon_r, float pivot_r)
{
    if (pivot_r + *icon_r > limit_r)
    {
        if (pivot_r - *icon_r < limit_r)
        {
            float new_pivot_r;
            float old_w, old_h;

            old_w = *x - (float) zoom_pivot->x;
            old_h = *y - (float) zoom_pivot->y;

            *icon_r = (limit_r - (pivot_r - *icon_r)) / 2;
            new_pivot_r = limit_r - *icon_r;

            *x += (old_w * new_pivot_r / pivot_r) - old_w;
            *y += (old_h * new_pivot_r / pivot_r) - old_h;

        }
        else
            *icon_r = 0;
    }
}

static lv_obj_t **get_icon_obj(uint32_t row_idx, uint32_t col_idx)
{
    if ((row_idx  >= MAX_APP_ROW_NUM) || (col_idx >= MAX_APP_COL_NUM))
    {
        return NULL;
    }

    return &(app_mainmenu_ctx.list[col_idx * MAX_APP_ROW_NUM + row_idx]);
}


static lv_point_t *get_icon_pivot(uint32_t row_idx, uint32_t col_idx)
{
    if ((row_idx  >= MAX_APP_ROW_NUM) || (col_idx >= MAX_APP_COL_NUM))
    {
        return NULL;
    }

    return &(app_mainmenu_ctx.icon_pivot[col_idx * MAX_APP_ROW_NUM + row_idx]);
}


/**
 * get icon colume and row by index as below(idx 0 row=(MAX_APP_ROW_NUM >> 1), col=(MAX_APP_COL_NUM >> 1)):
 *
 *     19___20___21___22
 *      |              \
 *      |  7___8___9   23
 *      |  |        \    \
 *     18  | 1___2   10   24
 *    /    |  \   \   \    \
 *   17    6   0   3   11   25
 *    \     \     /   /     /
 *     16    5___4   12    26
 *      \           /      /
 *       15___14___13    27
 * \n
 *
 * @param i
 * @param p_col
 * @param p_row
 * \n
 * @see
 */
static void layout_get_icon_col_row_by_idx(uint16_t idx, uint16_t *p_col, uint16_t *p_row)
{
    int16_t col, row;
    uint16_t i, total, hexagon_r;

    uint16_t one_edge_icons, hexagon_icons;

    if (0 != idx)
    {
        //find which hexagon is this icon on
        total = 0, hexagon_r = 0, hexagon_icons = 1;
        while (total + hexagon_icons - 1 < idx)
        {
            total += hexagon_icons;
            hexagon_r++;
            hexagon_icons = hexagon_r * 6;
        }

        //icons on one edge of this hexagon
        one_edge_icons = hexagon_r + 1;
        //first icon's  row&col num of this hexagon
        row = 0;
        col = 0 - hexagon_r;

        //calculate row&col from first one to idx
        for (i = 0; i < hexagon_icons; i++)
        {
            if (total + i == idx)
                break;

            switch (i / (one_edge_icons - 1))
            {
            case 0:
                col++;
                row--;
                break;

            case 1:
                col++;
                break;

            case 2:
                row++;
                break;

            case 3:
                col--;
                row++;
                break;

            case 4:
                col--;
                break;

            case 5:
                row--;
                break;

            default:
                RT_ASSERT(0);
                break;
            }

        }
    }
    else
    {
        col = 0;
        row = 0;
    }

    //rt_kprintf("icon %d, \t [%d,%d]\n",idx, row,col);
    *p_col = col + (MAX_APP_COL_NUM >> 1);
    *p_row = row + (MAX_APP_ROW_NUM >> 1);
}

static rt_err_t get_icon_col_row_by_lv_obj(lv_obj_t *obj, uint8_t *row_idx, uint8_t *col_idx)
{
    uint8_t i, j;

    if (NULL == obj)
    {
        return RT_EEMPTY;
    }

    for (i = 0; i < MAX_APP_ROW_NUM; i++)
        for (j = 0; j < MAX_APP_COL_NUM; j++)
        {
            if (obj == *get_icon_obj(i, j))
            {
                *row_idx = i;
                *col_idx = j;
                return RT_EOK;
            }
        }

    return RT_EEMPTY;
}

static void printf_icon_col_row(lv_obj_t *obj)
{
    uint8_t row_idx, col_idx;

    if (RT_EOK == get_icon_col_row_by_lv_obj(obj, &row_idx, &col_idx))
    {
        //LOG_I("printf_icon_col_row %x [%d,%d]", obj, row_idx, col_idx);
    }
}


static lv_obj_t *get_nearest_icon(lv_point_t *target)
{
    lv_obj_t *ret_v = NULL;
    uint8_t row_idx, col_idx;
    uint8_t first = 1;
    float min_delta = 0, pivot_r;

//    rt_kprintf("get_nearest_icon_vect  target %d, %d\n", target->x, target->y);
    for (row_idx = 0; row_idx < MAX_APP_ROW_NUM; row_idx++)
        for (col_idx = 0; col_idx < MAX_APP_COL_NUM; col_idx++)
        {
            uint32_t temp;
            lv_coord_t x, y;

            lv_sqrt_res_t ds;

            x = get_icon_pivot(row_idx, col_idx)->x;
            y = get_icon_pivot(row_idx, col_idx)->y;

            temp = (x - target->x) * (x - target->x) + (y - target->y) * (y - target->y);

            lv_sqrt(temp, &ds, 0x8000);

            pivot_r = ds.i + ds.f / 256;

            if (((pivot_r < min_delta) || first) && (NULL != *get_icon_obj(row_idx, col_idx)))
            {
                //vect2target->x =  target->x - x;
                //vect2target->y =  target->y - y;

                ret_v = *get_icon_obj(row_idx, col_idx);

                //rt_kprintf("get_nearest_icon_vect  [%d,%d] %d,%d vect %d, %d, %.1f\n", row_idx, col_idx, x, y, vect2target->x, vect2target->y, pivot_r);
                min_delta = pivot_r;

                first = 0;
            }
        }

    LV_ASSERT_NULL(ret_v);
    return  ret_v;
}


static lv_obj_t *add_app_icon(lv_obj_t *parent, const char *cmd, const void *img, uint8_t row_idx, uint8_t col_idx)
{
    lv_obj_t *icon;
    uint16_t s_len;
    char *cmd_str;
    if ((row_idx  >= MAX_APP_ROW_NUM) || (col_idx >= MAX_APP_COL_NUM))
    {
        return NULL;
    }

    icon = lv_img_create(parent);

    s_len = strlen(cmd) + 1;
    cmd_str = lv_mem_alloc(s_len);
    memcpy(cmd_str, cmd, s_len - 1);
    cmd_str[s_len - 1] = 0;

    lv_obj_set_user_data(icon, (void *)cmd_str);
    lv_obj_add_flag(icon, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(icon, LV_OBJ_FLAG_PRESS_LOCK);
    lv_obj_add_event_cb(icon, icon_event_callback, LV_EVENT_ALL, 0);
    lv_img_set_src(icon, img); //lv_img_set_src(icon, LV_EXT_IMG_GET(img_mail)); //
    //lv_obj_set_style_img_opa(icon, LV_OPA_80, LV_PART_MAIN | LV_STATE_DEFAULT);

    //lv_page_glue_obj(icon, true);
    //lv_obj_set_parent_event(icon, true);
    *get_icon_obj(row_idx, col_idx) = icon;

    LV_ASSERT(lv_obj_get_self_width(icon) != 0);
    LV_ASSERT(lv_obj_get_self_height(icon) != 0);

#ifdef DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE
    {
        lv_obj_t *label;

        label = lv_label_create(parent, NULL);
        lv_ext_set_local_font(label, FONT_SMALL, LV_COLOR_WHITE);

        *get_label_obj(row_idx, col_idx) = label;
    }
#endif

    return icon;
}

static void get_icons_init_coordinate(uint32_t row_idx, uint32_t col_idx,    lv_coord_t *x, lv_coord_t *y)
{
    /*
       r3r2r1r0    c0c1c2c3
        \ \ \ \    / / / /
         \ \ \ \  / / / /
          \ \ \ \/ / / /
           \ \ \/\/ / /
            \ \/\/\/ /
             \/\/\/\/
             /\/\/\/\
            / /\/\/\ \
           / / /\/\ \ \
          / / / /\ \ \ \
         / / / /  \ \ \ \
        / / / /    \ \ \ \
       /
      /
     /  60 degree
    /________________________

    * (r0,c0) as (0,0) default

    */


    *x = ((col_idx - row_idx) * lv_trigo_cos(60) * ICON_OUTER_RADIUS) >> (LV_TRIGO_SHIFT - 1);
    *y = ((col_idx + row_idx) * lv_trigo_sin(60) * ICON_OUTER_RADIUS) >> (LV_TRIGO_SHIFT - 1);

    *x += C0R0_COORD_X;
    *y += C0R0_COORD_Y;


}




static int layout_icon_transform(uint32_t row_idx, uint32_t col_idx, float *p_float_x, float *p_float_y, float *p_float_icon_w, float *p_float_icon_h, float *p_float_pivot_r)
{
    float float_x = *p_float_x;
    float float_y = *p_float_y;
    float float_icon_r = (*p_float_icon_w) / 2;
    float pivot_r;      //Icon pivot to screen pivot

    lv_point_t scr_center;

    scr_center.x = LV_HOR_RES_MAX >> 1;
    scr_center.y = LV_VER_RES_MAX >> 1;

    lv_area_t parent_area;
    parent_area.x1 = (LV_HOR_RES_MAX - LIMIT_RECT_WIDTH) >> 1;
    parent_area.y1 = (LV_VER_RES_MAX - LIMIT_RECT_HEIGHT) >> 1;
    parent_area.x2 = parent_area.x1 + LIMIT_RECT_WIDTH - 1;
    parent_area.y2 = parent_area.y1 + LIMIT_RECT_HEIGHT - 1;


    //calculate draw pivot and radius
    if (get_icon_transform_param(float_x,
                                 float_y,
                                 float_icon_r,
                                 &float_x, &float_y, &float_icon_r, &pivot_r, LV_HOR_RES_MAX, LV_VER_RES_MAX))
    {


#ifdef DEBUG_APP_MAINMENU_DISPLAY_ICON_PARAM
        if (LIMIT_ENABLE)
#endif /* DEBUG_APP_MAINMENU_DISPLAY_ICON_PARAM */
        {
#ifndef APP_MAINMENU_ROUND_SCREEN
            limit_round2(LIMIT_ROUND_RADIUS, &scr_center, &float_x, &float_y, &float_icon_r, pivot_r);
            limit_square(&parent_area, &float_x, &float_y, &float_icon_r);


#else //round screen
            limit_round2(LV_HOR_RES >> 1, &scr_center, &float_x, &float_y, &float_icon_r, pivot_r);
            //limit_round(LV_HOR_RES >> 1, &scr_center, &x, &y, &icon_r, pivot_r);
#endif
        }


    }
    else
    {
        float_icon_r = 0;
        float_x = 0;
        float_y = 0;

    }

#if 1 //Gap

    if (float_icon_r >= (ICON_OUTER_RADIUS - ICON_INNER_RADIUS))
    {
        float_icon_r = float_icon_r - (ICON_OUTER_RADIUS - ICON_INNER_RADIUS);
    }
    else
    {
        float_icon_r = 0;
    }
#endif /* 0 */



    *p_float_x = float_x ;
    *p_float_y = float_y ;
    *p_float_icon_w = float_icon_r * 2;
    *p_float_pivot_r = pivot_r;      //Icon pivot to screen pivot


    return (0 == float_icon_r) ? 0 : 1;
}





#ifdef DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE
static void app_mainmenu_draw_icon_label(uint8_t row_idx, uint8_t col_idx, lv_coord_t pi_x, lv_coord_t pi_y, uint16_t r)
{
    lv_obj_t *label, *icon;
    char buff[50];

    label = app_mainmenu_ctx.label_list[row_idx][col_idx];
    icon = app_mainmenu_ctx.list[row_idx][col_idx];

    rt_sprintf(buff, "%d,%d,%d\n%d,%d,%d", lv_obj_get_x(icon), lv_obj_get_y(icon), lv_img_get_zoom(icon),
               pi_x, pi_y, r);
    lv_label_set_text(label, buff);
    lv_obj_align(label, icon, LV_ALIGN_CENTER, 0, 0);
}
#endif

static int32_t app_mainmenu_draw_icon(lv_obj_t *obj, float pi_x, float pi_y, float w, float h)
{
    uint16_t zoom;

    if ((w != 0) && (h != 0))
    {
        lv_coord_t img_w = lv_obj_get_self_width(obj);
        lv_coord_t img_h = lv_obj_get_self_height(obj);

        //lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
        obj->flags &= (~LV_OBJ_FLAG_HIDDEN);


        //Updata zoom
        zoom = (uint16_t)(w * 256 / (float)img_w);
        //lv_img_set_zoom(obj, zoom);
#ifndef DISABLE_LVGL_V8
        ((lv_img_t *)obj)->zoom = zoom;
#else
        ((lv_img_t *)obj)->scale_x = zoom;
        ((lv_img_t *)obj)->scale_y = zoom;
#endif

        //Move icon
        {
            int32_t pi_x_10p8 = pi_x * 256;
            int32_t pi_y_10p8 = pi_y * 256;

            //rt_kprintf("app_mainmenu_draw_icon %p:  [%.3f,%.3f]->[%d, %d]   %d\n", obj, pi_x, pi_y, pi_x_10p8, pi_y_10p8, zoom);

            pi_x_10p8 -= ((img_w >> 1) << 8);
            pi_y_10p8 -= ((img_h >> 1) << 8);
            //lv_obj_set_pos(obj, ((lv_coord_t)(pi_x_10p8 >> 8)) + lv_obj_get_scroll_x(app_mainmenu_ctx.pg_obj),
            //               ((lv_coord_t)(pi_y_10p8 >> 8)) + lv_obj_get_scroll_y(app_mainmenu_ctx.pg_obj));


            lv_obj_move_to(obj, ((lv_coord_t)(pi_x_10p8 >> 8)) + lv_obj_get_scroll_x(app_mainmenu_ctx.pg_obj),
                           ((lv_coord_t)(pi_y_10p8 >> 8)) + lv_obj_get_scroll_y(app_mainmenu_ctx.pg_obj));

#ifndef DISABLE_LVGL_V8
            lv_img_set_x_frac(obj, (uint16_t)((pi_x_10p8 << 8)) & 0xFFFF);
            lv_img_set_y_frac(obj, (uint16_t)((pi_y_10p8 << 8)) & 0xFFFF);
#endif
        }
        //srt_kprintf("app_mainmenu_draw_icon %p:  %d,%d   %d\n",obj, pi_x, pi_y, zoom);
    }
    else
    {
        //lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
        obj->flags |= LV_OBJ_FLAG_HIDDEN;
        zoom = 0;
    }
    return zoom;
}

static void app_mainmenu_init_icons_coordinate(void)
{
    lv_coord_t x, y;
    uint8_t row_idx, col_idx;

    for (row_idx = 0; row_idx < MAX_APP_ROW_NUM; row_idx++)
        for (col_idx = 0; col_idx < MAX_APP_COL_NUM; col_idx++)
        {
            lv_obj_t *icon;
            get_icons_init_coordinate(row_idx, col_idx, &x, &y);

            get_icon_pivot(row_idx, col_idx)->x = x;
            get_icon_pivot(row_idx, col_idx)->y = y;

            icon = *get_icon_obj(row_idx, col_idx);
            if (icon)
            {
                lv_coord_t img_w = lv_obj_get_self_width(icon);
                lv_coord_t img_h = lv_obj_get_self_height(icon);
                uint16_t zoom = (uint16_t)(ICON_IMG_WIDTH * 256 / img_w);

                //rt_kprintf("init icon [%d,%d]  %d\n", x, y, zoom);

                lv_obj_set_pos(icon, x - (img_w >> 1), y - (img_h >> 1));
                lv_img_set_pivot(icon, (img_w >> 1), (img_h >> 1));
                lv_img_set_zoom(icon, zoom);

            }
        }

}



#if 0

static void app_mainmenu_click_icon_anim_ready_callback(lv_anim_t *a)
{
    if (app_mainmenu_ctx.anim_obj)
    {
        char *cmd = (char *)lv_obj_get_user_data(app_mainmenu_ctx.anim_obj);

        if (cmd)
        {
            rt_kprintf("app mainmenu icon click anim cbk\n");
            gui_app_run(cmd);
        }
        app_mainmenu_ctx.anim_obj = NULL;
    }
}


static void app_mainmenu_focus_icon(lv_obj_t *obj, uint32_t max_anim_ms, lv_anim_ready_cb_t cbk)
{
    //lv_coord_t c0r0_x, c0r0_y;
    lv_coord_t sx, sy, dx, dy;
    lv_obj_t *pg_scrl;
    uint8_t row_idx, col_idx;
    lv_area_t parent_area;


    LOG_I("app_mainmenu_focus_icon");
    if (RT_EOK != get_icon_col_row_by_lv_obj(obj, &row_idx, &col_idx))
        return;

    LOG_I("app_mainmenu_focus_icon: row %d, col %d", row_idx, col_idx);

    parent_area.x1 = 0;
    parent_area.y1 = 0;
    parent_area.x2 = parent_area.x1 + LV_HOR_RES - 1;
    parent_area.y2 = parent_area.y1 + LV_VER_RES - 1;

    pg_scrl = lv_page_get_scrl(app_mainmenu_ctx.pg_obj);
    sx = lv_obj_get_x(pg_scrl);
    sy = lv_obj_get_y(pg_scrl);


    dx = sx + ((parent_area.x2 + 1 - parent_area.x1) >> 1) - get_icon_pivot(row_idx, col_idx)->x;
    dy = sy + ((parent_area.y2 + 1 - parent_area.y1) >> 1) - get_icon_pivot(row_idx, col_idx)->y;

    /*
        rt_kprintf("pg_scrl=[%d,%d]\n icon=[%d,%d]\n ciotsc=[%d,%d]\n", lv_obj_get_x(pg_scrl),lv_obj_get_y(pg_scrl),
                                        get_icon_pivot(row_idx,col_idx)->x,
                                        get_icon_pivot(row_idx,col_idx)->y,
                                        app_mainmenu_ctx.ciotsc.x,
                                        app_mainmenu_ctx.ciotsc.y

                                        );
    */

    app_mainmenu_ctx.anim_obj = NULL;

    lv_anim_del(pg_scrl, NULL);

    if (max_anim_ms)
    {
        lv_anim_t ax;
        lv_anim_path_t path;
        uint16_t anim_duration;
        lv_sqrt_res_t ds;
        uint32_t abs_x, abs_y;

        if (app_mainmenu_ctx.indev)
        {
            app_mainmenu_ctx.indev->proc.types.pointer.drag_throw_vect.x = 0;
            app_mainmenu_ctx.indev->proc.types.pointer.drag_throw_vect.y = 0;
        }
        abs_x = (dx > sx) ? (dx - sx) : (sx - dx);
        abs_y = (dy > sy) ? (dy - sy) : (sy - dy);

        _lv_sqrt(abs_x * abs_x + abs_y * abs_y, &ds, 0x8000);

        /*
             Start animation except central icon
        */
        {
            anim_duration = max_anim_ms * ds.i / LV_HOR_RES_MAX ;
            anim_duration = LV_MIN(anim_duration, max_anim_ms);

            //rt_kprintf("app_mainmenu_focus_icon anim_duration %d ms\n", anim_duration);

            lv_anim_init(&ax);
            lv_anim_set_time(&ax, anim_duration);
            lv_anim_path_init(&path);
            lv_anim_set_path(&ax, &path);
            lv_anim_path_set_cb(&path, lv_anim_path_ease_in_out);

            lv_anim_set_var(&ax, pg_scrl);
            lv_anim_set_exec_cb(&ax, (lv_anim_exec_xcb_t) lv_obj_set_x);
            lv_anim_set_values(&ax, sx, dx);
            lv_anim_start(&ax);

            lv_anim_set_exec_cb(&ax, (lv_anim_exec_xcb_t) lv_obj_set_y);
            lv_anim_set_values(&ax, sy, dy);
            lv_anim_set_ready_cb(&ax, cbk);
            lv_anim_start(&ax);
            app_mainmenu_ctx.anim_obj = obj;

            return;
        }
    }
    lv_obj_set_pos(pg_scrl, dx, dy);

    if (cbk)
    {
        lv_anim_t ax;
        ax.var = obj;
        cbk(&ax);
    }


}
#endif

static void icon_event_callback(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    //rt_kprintf("icon_event_callback %s\n",lv_event_to_name(event));
    if (app_mainmenu_ctx.scroll_actived && (LV_EVENT_RELEASED == event \
                                            || LV_EVENT_PRESS_LOST == event))
    {
        //Not to clear scroll_actived before icon receieve click event
        lv_event_stop_bubbling(e);
    }
    else if ((LV_EVENT_SHORT_CLICKED == event) && (!app_mainmenu_ctx.scroll_actived))
    {
        rt_kprintf("app mainmenu icon clickd\n");

        lv_obj_add_flag(app_mainmenu_ctx.pg_obj, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_scroll_to_view(obj, LV_ANIM_ON);
        app_mainmenu_ctx.anim_obj = obj;

        //There is no scroll animation if the clicked icon was just at center of pg_obj
        if (NULL == lv_anim_get(app_mainmenu_ctx.pg_obj, NULL))
        {
            //Send SCROLL_END msg manually
#ifndef  DISABLE_LVGL_V8
            lv_event_send(app_mainmenu_ctx.pg_obj, LV_EVENT_SCROLL_END, NULL);
#else
            lv_obj_send_event(app_mainmenu_ctx.pg_obj, LV_EVENT_SCROLL_END, NULL);
#endif
        }

    }
    else if (LV_EVENT_DELETE == event)
    {
        char *cmd = (char *)lv_obj_get_user_data(obj);
        if (cmd)
            lv_mem_free(cmd);
    }
}



static lv_obj_t *predict_focus_icon(void)
{
    lv_point_t scr_center;
    lv_point_t vect;
    uint8_t scroll_throw;
    uint32_t anim_duration;

    lv_indev_t *indev = lv_indev_get_act();

    scr_center.x = LV_HOR_RES_MAX >> 1;
    scr_center.y = LV_VER_RES_MAX >> 1;


    lv_indev_get_vect(indev, &vect);

    LOG_I("scroll_throw %d,%d", vect.x, vect.y);
#ifndef DISABLE_LVGL_V8
    scroll_throw = indev->driver->scroll_throw;
#else
    scroll_throw = 0;
#endif
    anim_duration = 0;

    while (vect.x != 0  || vect.y != 0)
    {
        /*Reduce the vectors*/
        vect.x = vect.x * (100 - scroll_throw) / 100;
        vect.y = vect.y * (100 - scroll_throw) / 100;

        scr_center.x = scr_center.x - vect.x;
        scr_center.y = scr_center.y - vect.y;

        anim_duration += LV_INDEV_DEF_READ_PERIOD * 4;
    }

    /*
       Get the compensation vector of nearest icon to new screen center
    */
    //update_icon_pivot();
    //get_nearest_icon_vect(&scr_center, &app_mainmenu_ctx.comp_vect);

    return    get_nearest_icon(&scr_center);
}


static void page_event_callback(lv_event_t *e)
{

    lv_obj_t *obj = lv_event_get_target(e);
    lv_event_code_t event = lv_event_get_code(e);


    //LOG_I("page_event_callback:event %s", lv_event_to_name(event));
    if (LV_EVENT_PRESSED == event)
    {
        //Clear anim obj in case stop scroll to been aborted
        app_mainmenu_ctx.anim_obj = NULL;
        lv_obj_clear_flag(app_mainmenu_ctx.pg_obj, LV_OBJ_FLAG_SCROLLABLE);

        if (0 != lv_anim_count_running())
        {
            lv_anim_del(app_mainmenu_ctx.pg_obj, NULL);
            //lv_anim_del(lv_page_get_scrl(app_mainmenu_ctx.pg_obj), NULL);
        }
        app_mainmenu_ctx.springback_open = false;
    }
    else if (LV_EVENT_RELEASED == event \
             || LV_EVENT_PRESS_LOST == event || LV_EVENT_CLICKED == event)
    {
        lv_obj_add_flag(app_mainmenu_ctx.pg_obj, LV_OBJ_FLAG_SCROLLABLE);

        if (app_mainmenu_ctx.scroll_actived)
        {
            app_mainmenu_ctx.springback_open = true;

            LOG_I("predict focus icon:");
            printf_icon_col_row(predict_focus_icon());

            lv_obj_scroll_to_view(predict_focus_icon(), LV_ANIM_ON);
        }

        app_mainmenu_ctx.scroll_actived = false;
        app_mainmenu_ctx.scroll_sum.x = 0;
        app_mainmenu_ctx.scroll_sum.y = 0;

    }
    else if (LV_EVENT_PRESSING == event)
    {
        lv_indev_t *indev = lv_indev_get_act();
        lv_point_t p;

        lv_indev_get_vect(indev, &p);

        app_mainmenu_ctx.scroll_sum.x += p.x;
        app_mainmenu_ctx.scroll_sum.y += p.y;

#ifndef DISABLE_LVGL_V8
        if ((LV_ABS(app_mainmenu_ctx.scroll_sum.x) > indev->driver->scroll_limit)
                || (LV_ABS(app_mainmenu_ctx.scroll_sum.y) > indev->driver->scroll_limit)
                || app_mainmenu_ctx.scroll_actived)
        {
            app_mainmenu_ctx.scroll_actived = true;
            //_lv_obj_scroll_by_raw(app_mainmenu_ctx.pg_obj, p.x, p.y); scroll once before draw to speed up
            lv_obj_invalidate(app_mainmenu_ctx.pg_obj);
        }
#endif

#ifdef DISABLE_LVGL_V8



        if (app_mainmenu_ctx.scroll_actived)
        {
            _lv_obj_scroll_by_raw(app_mainmenu_ctx.pg_obj,
                                  app_mainmenu_ctx.scroll_sum.x,
                                  app_mainmenu_ctx.scroll_sum.y);

            app_mainmenu_ctx.scroll_sum.x = 0;
            app_mainmenu_ctx.scroll_sum.y = 0;
        }
        app_mainmenu_icons_transform(false);
        printf_icon_col_row(app_mainmenu_ctx.cicon);
#endif /* DISABLE_LVGL_V8 */

    }
#ifndef DISABLE_LVGL_V8
    else if (LV_EVENT_DRAW_MAIN_BEGIN == event)
    {


        if (app_mainmenu_ctx.scroll_actived)
        {
            _lv_obj_scroll_by_raw(app_mainmenu_ctx.pg_obj,
                                  app_mainmenu_ctx.scroll_sum.x,
                                  app_mainmenu_ctx.scroll_sum.y);

            app_mainmenu_ctx.scroll_sum.x = 0;
            app_mainmenu_ctx.scroll_sum.y = 0;
        }
        app_mainmenu_icons_transform(false);
        printf_icon_col_row(app_mainmenu_ctx.cicon);
    }
#endif /* DISABLE_LVGL_V8 */
    else if (LV_EVENT_SCROLL_END == event)
    {
        if (app_mainmenu_ctx.anim_obj)
        {
            char *cmd = (char *)lv_obj_get_user_data(app_mainmenu_ctx.anim_obj);

            if (cmd)
            {
                rt_kprintf("app mainmenu icon click anim cbk\n");
                gui_app_run(cmd);
            }
            app_mainmenu_ctx.anim_obj = NULL;
        }
    }


}


static inline int16_t reorder_clock_icon(int16_t idx, int16_t clock_idx, const builtin_app_desc_t *builtin_app, lv_obj_t *page)
{
    uint16_t col, row;

    //Fix 1st icon for clock
    if (0 == strcmp("clock", builtin_app->id))
    {
        lv_obj_t *icon;
        layout_get_icon_col_row_by_idx(clock_idx, &col, &row);

        icon = add_app_icon(page, "clock", builtin_app->icon, row, col);

        if (icon)        lv_obj_move_to_index(icon, 0);
    }
    else if (0 == strcmp(APP_ID, builtin_app->id)) //skip main menu icon
    {

    }
    else if (NULL != builtin_app->icon)
    {
        if (idx == clock_idx)
            idx++;

        layout_get_icon_col_row_by_idx(idx, &col, &row);
        add_app_icon(page, builtin_app->id, builtin_app->icon, row, col);
        idx++;
    }

    return idx;
}



static void app_mainmenu_read_app_icons(lv_obj_t *page)
{
    uint16_t col, row;
    uint16_t idx = 1; /* As idx 0 is reserved for clock app, other apps idx start from 1*/
    uint16_t clock_idx = 0; /* 0 - reserved for clock app*/
    const builtin_app_desc_t *p_builtin_app;
    int mainmenu_icon_style;


    mainmenu_icon_style = 0x00;


    /*1. load builtin app list*/
    p_builtin_app = gui_builtin_app_list_open();
    if (p_builtin_app)
    {
        do
        {
            //Fix 1st icon for clock
            idx = reorder_clock_icon(idx, clock_idx, p_builtin_app, page);
            p_builtin_app = gui_builtin_app_list_get_next(p_builtin_app);
        }
        while (p_builtin_app);

        while (1)
        {
            //Fix 1st icon for clock
            p_builtin_app = gui_script_app_list_get_next(p_builtin_app);
            if (p_builtin_app == NULL)
                break;
            idx = reorder_clock_icon(idx, clock_idx, p_builtin_app, page);
        }

        gui_builtin_app_list_close(p_builtin_app);
        p_builtin_app = NULL;
    }

#if 1//for demo, full fill screen
    {
        uint16_t i;
        const void *dummy_icons[] =
        {
            LV_EXT_IMG_GET(img_passbook),
            LV_EXT_IMG_GET(img_mail), LV_EXT_IMG_GET(img_calendar), LV_EXT_IMG_GET(img_camera),
            LV_EXT_IMG_GET(img_phone), LV_EXT_IMG_GET(img_alarm_2), LV_EXT_IMG_GET(img_maps),
            LV_EXT_IMG_GET(img_photos), LV_EXT_IMG_GET(img_remote), LV_EXT_IMG_GET(img_workout),
            LV_EXT_IMG_GET(img_world_clock), LV_EXT_IMG_GET(img_stocks),
            LV_EXT_IMG_GET(img_alarm), LV_EXT_IMG_GET(img_stocks),
            LV_EXT_IMG_GET(img_passbook),
            LV_EXT_IMG_GET(img_mail), LV_EXT_IMG_GET(img_calendar), LV_EXT_IMG_GET(img_camera),
            LV_EXT_IMG_GET(img_phone), LV_EXT_IMG_GET(img_alarm_2), LV_EXT_IMG_GET(img_maps),
            LV_EXT_IMG_GET(img_photos), LV_EXT_IMG_GET(img_remote), LV_EXT_IMG_GET(img_workout),
            LV_EXT_IMG_GET(img_world_clock), LV_EXT_IMG_GET(img_stocks),
            LV_EXT_IMG_GET(img_alarm), LV_EXT_IMG_GET(img_stocks),
            LV_EXT_IMG_GET(img_passbook),
            LV_EXT_IMG_GET(img_mail), LV_EXT_IMG_GET(img_calendar), LV_EXT_IMG_GET(img_camera),
            LV_EXT_IMG_GET(img_phone), LV_EXT_IMG_GET(img_alarm_2), LV_EXT_IMG_GET(img_maps),
            LV_EXT_IMG_GET(img_photos), LV_EXT_IMG_GET(img_remote), LV_EXT_IMG_GET(img_workout),
            LV_EXT_IMG_GET(img_world_clock), LV_EXT_IMG_GET(img_stocks),
            LV_EXT_IMG_GET(img_alarm), LV_EXT_IMG_GET(img_stocks),
            LV_EXT_IMG_GET(img_passbook),
            LV_EXT_IMG_GET(img_mail), LV_EXT_IMG_GET(img_calendar), LV_EXT_IMG_GET(img_camera),
            LV_EXT_IMG_GET(img_phone), LV_EXT_IMG_GET(img_alarm_2), LV_EXT_IMG_GET(img_maps),
            LV_EXT_IMG_GET(img_photos), LV_EXT_IMG_GET(img_remote), LV_EXT_IMG_GET(img_workout),
            LV_EXT_IMG_GET(img_world_clock), LV_EXT_IMG_GET(img_stocks),
            LV_EXT_IMG_GET(img_alarm), LV_EXT_IMG_GET(img_stocks),
        };

        for (i = 0; i < sizeof(dummy_icons) / sizeof(dummy_icons[0]); i++, idx++)
        {
            layout_get_icon_col_row_by_idx(idx, &col, &row);
            lv_obj_t *p_obj = add_app_icon(page, "none", dummy_icons[i], row, col);

            lv_obj_set_style_img_opa(p_obj, LV_OPA_50, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
#endif

}
/**
 * move whole icons map together
 * \n
 *
 * @param c0r0_x - col 0, row 0 item coordinate x
 * @param c0r0_y - col 0, row 0 item coordinate y
 * \n
 * @see
 */
#define MM_ZOOM(A,B,zoom) ((A) + (((B) - (A)) * (zoom)))
static void app_mainmenu_icons_transform(bool force_refresh)
{
    lv_coord_t x_offset, y_offset;
    uint8_t row_idx, col_idx;
    float min_delta;
    static lv_coord_t c0r0_x, c0r0_y;

    lv_coord_t cur_c0r0_x = C0R0_COORD_X - lv_obj_get_scroll_x(app_mainmenu_ctx.pg_obj);
    lv_coord_t cur_c0r0_y = C0R0_COORD_Y - lv_obj_get_scroll_y(app_mainmenu_ctx.pg_obj);

    if (c0r0_x == cur_c0r0_x \
            && c0r0_y == cur_c0r0_y \
            && app_mainmenu_ctx.last_zoom == app_mainmenu_ctx.zoom \
            && !force_refresh)
    {
        return;
    }

    //rt_kprintf("app_mainmenu_icons_transform:zoom %f last_zoom %f force_refresh\n",app_mainmenu_ctx.zoom, app_mainmenu_ctx.last_zoom, force_refresh);

    c0r0_x = cur_c0r0_x;
    c0r0_y = cur_c0r0_y;

    x_offset = c0r0_x - get_icon_pivot(0, 0)->x;
    y_offset = c0r0_y - get_icon_pivot(0, 0)->y;

    min_delta = LV_VER_RES;

    //uint32_t start = rt_tick_get();

    for (row_idx = 0; row_idx < MAX_APP_ROW_NUM; row_idx++)
        for (col_idx = 0; col_idx < MAX_APP_COL_NUM; col_idx++)
        {
            //offset icon pivot
            get_icon_pivot(row_idx, col_idx)->x += x_offset;
            get_icon_pivot(row_idx, col_idx)->y += y_offset;

            if (*get_icon_obj(row_idx, col_idx))
            {
                float float_x = (float)get_icon_pivot(row_idx, col_idx)->x;
                float float_y = (float)get_icon_pivot(row_idx, col_idx)->y;
                float float_icon_w = (float) ICON_IMG_WIDTH;
                float float_icon_h = (float) ICON_IMG_HEIGHT;

                float pivot_r; //Icon pivot to screen pivot
                float zoom;

                //rt_kprintf("transf_before[%.1f,%.1f] w=%.1f,h=%.1f\n", float_x, float_y, float_icon_w, float_icon_h);

                if (app_mainmenu_ctx.zoom < 1)
                {
                    zoom = app_mainmenu_ctx.zoom;
                    zoom = zoom * zoom;
                    float_x = MM_ZOOM(LV_HOR_RES_MAX >> 1, float_x, zoom);
                    float_y = MM_ZOOM(LV_VER_RES_MAX >> 1, float_y, zoom);
                    float_icon_w *= zoom;
                    float_icon_h *= zoom;
                }
                if (app_mainmenu_ctx.zoom > 0)
                {
                    float float_x_before = float_x;
                    float float_y_before = float_y;
                    float float_icon_w_before = float_icon_w;
                    float float_icon_h_before = float_icon_h;

                    //calculate draw pivot and radius
                    if (layout_icon_transform(row_idx, col_idx, &float_x, &float_y, &float_icon_w, &float_icon_h, &pivot_r))
                    {
                        //Record the nearest icon to center
                        if (pivot_r < min_delta)
                        {
                            //rt_kprintf("center[%d,%d] pivot[%.1f, %.1f], float_pivot_r =%.2f  \n", row_idx, col_idx, float_x, float_y, pivot_r);

                            min_delta = pivot_r;
                            //app_mainmenu_ctx.ciotsc.x = float_x - scr_center.x;
                            //app_mainmenu_ctx.ciotsc.y = float_y - scr_center.y;
                            app_mainmenu_ctx.cicon = *get_icon_obj(row_idx, col_idx);
                        }

                        if ((app_mainmenu_ctx.zoom > 0) && (app_mainmenu_ctx.zoom <= 1))
                        {
                            zoom = app_mainmenu_ctx.zoom;
                            float_x = MM_ZOOM(float_x_before, float_x, zoom);
                            float_y = MM_ZOOM(float_y_before, float_y, zoom);

                            float_icon_w = MM_ZOOM(float_icon_w_before, float_icon_w, zoom);
                            float_icon_h = MM_ZOOM(float_icon_h_before, float_icon_w, zoom);
                        }
                    }
                    else
                    {
                        float_icon_w = 0;
                        float_icon_h = 0;
                        float_x = 0;
                        float_y = 0;
                    }
                }

                if (app_mainmenu_ctx.zoom > 1)
                {
                    zoom = 1 / (2 - app_mainmenu_ctx.zoom);

                    float_x = MM_ZOOM(LV_HOR_RES_MAX >> 1, float_x, zoom);
                    float_y = MM_ZOOM(LV_VER_RES_MAX >> 1, float_y, zoom);
                    float_icon_w *= zoom;
                    float_icon_h *= zoom;
                }

                //draw icon
                //rt_kprintf("transf_after[%.1f,%.1f] w=%.1f,h=%.1f\n", float_x, float_y, float_icon_w, float_icon_h);
                app_mainmenu_draw_icon(*get_icon_obj(row_idx, col_idx), float_x, float_y, float_icon_w, float_icon_h);

#ifdef DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE
                app_mainmenu_draw_icon_label(row_idx, col_idx, float_x, float_y, float_icon_w, float_icon_h, pivot_r);
#endif
            }
        }


    //rt_kprintf("app_mainmenu_icons_transform cost %d ms \n",rt_tick_get() - start);
}
/*
static void app_mainmenu_drag_springback(void)
{
    lv_point_t icons_pivot, dis;
    lv_obj_t *icons = app_mainmenu_ctx.cicon;
    icons_pivot.x = (lv_coord_t)((icons->coords.x2 - icons->coords.x1) >> 1) + icons->coords.x1;
    icons_pivot.y = (lv_coord_t)((icons->coords.y2 - icons->coords.y1) >> 1) + icons->coords.y1;

    dis.x = abs(icons_pivot.x - (LV_HOR_RES_MAX >> 1));
    dis.y = abs(icons_pivot.y - (LV_VER_RES_MAX >> 1));

    if (dis.x > lv_obj_get_width(app_mainmenu_ctx.cicon) \
            || dis.y > lv_obj_get_height(app_mainmenu_ctx.cicon))
    {
        app_mainmenu_ctx.springback_open = false;
        if (app_mainmenu_ctx.indev)
        {
            app_mainmenu_ctx.indev->proc.types.pointer.drag_throw_vect.x = 0;
            app_mainmenu_ctx.indev->proc.types.pointer.drag_throw_vect.y = 0;
        }
        if (0 != lv_anim_count_running())
        {
            lv_anim_del(app_mainmenu_ctx.pg_obj, NULL);
            lv_anim_del(lv_page_get_scrl(app_mainmenu_ctx.pg_obj), NULL);
        }
        //LOG_L1("app_mainmenu_drag_springback");
        app_mainmenu_focus_icon(app_mainmenu_ctx.cicon, 300, NULL);
    }
}


static lv_design_res_t mm_scrl_design(lv_obj_t *scrl, const lv_area_t *clip_area, lv_design_mode_t mode)
{
    lv_res_t res;

    if (mode == LV_DESIGN_DRAW_MAIN)
    {
        app_mainmenu_icons_transform(false);
        if (app_mainmenu_ctx.springback_open)
        {
            app_mainmenu_drag_springback();
        }
    }
    res = app_mainmenu_ctx.scrl_design(scrl, clip_area, mode);
    return res;
}

*/


void app_mainmenu_ui_init(void *param)
{
    lv_obj_t *page = lv_obj_create(lv_scr_act()); //lv_img_create(lv_scr_act()); //
    lv_obj_set_size(page, LV_HOR_RES_MAX, LV_VER_RES_MAX);
    lv_obj_set_style_bg_color(page, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(page, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_scroll_snap_x(page, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scroll_snap_y(page, LV_SCROLL_SNAP_CENTER);
    //lv_img_set_src(page,LV_EXT_IMG_GET(celluar));


    app_mainmenu_ctx.pg_obj = page;

    lv_obj_set_scrollbar_mode(page, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(page, page_event_callback, LV_EVENT_ALL, 0);

    //app_mainmenu_ctx.scrl_design = lv_obj_get_design_cb(lv_page_get_scrl(page));

    //lv_obj_set_design_cb(lv_page_get_scrl(page), mm_scrl_design);

    app_mainmenu_read_app_icons(page);

    /* disable autofit so that scrollable part would not be auto repositioned if changing children size */
    //lv_page_set_scrllable_fit4(page, LV_FIT_NONE, LV_FIT_NONE, LV_FIT_NONE, LV_FIT_NONE);

#if 0//def OLD_LAYOUT
    /* add margin in the right */
    lv_page_set_scrl_width(page, lv_page_get_scrl_width(page) + lv_obj_get_width(app_mainmenu_ctx.list[0]));
#else
    /* add margin in the right */
    //lv_page_set_scrl_width(page, PAGE_SCRL_WIDTH);
    /* add margin at the bottom */
    //lv_page_set_scrl_height(page, PAGE_SCRL_HEIGHT);
#endif

    app_mainmenu_init_icons_coordinate();

    /*
        {
            uint16_t col, row;
            get_icon_col_row(0, &col, &row);
            app_mainmenu_focus_icon(*get_icon_obj(row, col), 0, NULL);
        }
        page_event_callback(app_mainmenu_ctx.pg_obj, LV_EVENT_RELEASED);

    */



}


static void destroy(void)
{

}

static lv_obj_t *get_main_win(void)
{
    return NULL;
}

static app_mainmenu_item_t *new_app_item(char *name, const lv_img_dsc_t *icon, char *cmd, uint8_t row, uint8_t col)
{
    app_mainmenu_item_t *item;
    rt_size_t len;

    RT_ASSERT((name != NULL) && (icon != NULL) && (cmd != NULL));

    item = (app_mainmenu_item_t *)rt_malloc(sizeof(app_mainmenu_item_t));

    if (item != NULL)
    {
        memset(item, 0, sizeof(app_mainmenu_item_t));

        len = strlen(name);
        RT_ASSERT(len <= (GUI_APP_NAME_MAX_LEN - 1));
        strncpy(item->name, name, len);
        item->name[len] = '\0';


        len = strlen(cmd);
        RT_ASSERT(len <= (GUI_APP_CMD_MAX_LEN - 1));
        strncpy(item->cmd, cmd, len);
        item->cmd[len] = '\0';

        memcpy(&item->icon, icon, sizeof(lv_img_dsc_t));

        item->row = row;
        item->col = col;
    }

    return item;
}
#ifdef APP_TRANS_ANIMATION_SCALE
    #define MM_CUST_TRAN_ANIMATION
#endif /* APP_TRANS_ANIMATION_SCALE */

#ifdef MM_CUST_TRAN_ANIMATION


static CUST_ANIM_TYPE_E cust_anim_type = CUST_ANIM_TYPE_0;
static void mm_trans_anim_init(void)
{
    //cust_anim_type = CUST_ANIM_TYPE_3; //Fix trans animation
    cust_trans_anim_config(cust_anim_type++, NULL);
//Avoid animation crash
#if (LV_HOR_RES_MAX > 512)||(LV_VER_RES_MAX > 512)
    if (cust_anim_type == CUST_ANIM_TYPE_3)  cust_anim_type++;
#endif

    if (cust_anim_type >= CUST_ANIM_TYPE_MAX) cust_anim_type = CUST_ANIM_TYPE_0;
}


#else
static void mm_trans_anim_init(void)
{
    gui_app_trans_anim_t enter_anim_cfg, exit_anim_cfg;

    gui_app_trans_anim_init_cfg(&enter_anim_cfg, GUI_APP_TRANS_ANIM_ZOOM_OUT);
    gui_app_trans_anim_init_cfg(&exit_anim_cfg, GUI_APP_TRANS_ANIM_ZOOM_IN);


    enter_anim_cfg.cfg.zoom.zoom_start = LV_IMG_ZOOM_NONE >> 2;
    enter_anim_cfg.cfg.zoom.zoom_end = LV_IMG_ZOOM_NONE;
    enter_anim_cfg.cfg.zoom.opa_start = LV_OPA_0;
    enter_anim_cfg.cfg.zoom.opa_end = LV_OPA_COVER;

    exit_anim_cfg.cfg.zoom.zoom_start = LV_IMG_ZOOM_NONE;
    exit_anim_cfg.cfg.zoom.zoom_end = LV_IMG_ZOOM_NONE << 2;
    exit_anim_cfg.cfg.zoom.opa_start = LV_OPA_COVER;
    exit_anim_cfg.cfg.zoom.opa_end = LV_OPA_0;


    gui_app_set_enter_trans_anim(&enter_anim_cfg);
    gui_app_set_exit_trans_anim(&exit_anim_cfg);

}
#endif /* MM_CUST_TRAN_ANIMATION */
static void on_start(void)
{
    uint16_t max_icons;

    memset(&app_mainmenu_ctx, 0, sizeof(app_mainmenu_ctx));

    app_mainmenu_ctx.zoom = 1;
    app_mainmenu_ctx.last_zoom = 1;

    max_icons = MAX_APP_COL_NUM * MAX_APP_ROW_NUM;
    app_mainmenu_ctx.list = rt_malloc(max_icons * sizeof(lv_obj_t *));
    RT_ASSERT(app_mainmenu_ctx.list != NULL);
    memset(app_mainmenu_ctx.list, 0, max_icons * sizeof(lv_obj_t *));
#ifdef DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE
    app_mainmenu_ctx.label_list = rt_malloc(max_icons * sizeof(lv_obj_t *));
    RT_ASSERT(app_mainmenu_ctx.label_list != NULL);
    memset(app_mainmenu_ctx.list, 0, max_icons * sizeof(lv_obj_t *));
#endif
    app_mainmenu_ctx.icon_pivot = rt_malloc(max_icons * sizeof(lv_point_t));
    RT_ASSERT(app_mainmenu_ctx.icon_pivot != NULL);
    memset(app_mainmenu_ctx.icon_pivot, 0, max_icons * sizeof(lv_point_t));

    app_mainmenu_ui_init(NULL);
    //lv_obj_set_parent(app_mainmenu_ctx.pg_obj, lv_scr_act());


    app_mainmenu_icons_transform(true);

    /*Update the buttons position manually for first*/
#ifndef  DISABLE_LVGL_V8
    lv_event_send(app_mainmenu_ctx.pg_obj, LV_EVENT_SCROLL, NULL);
#else
    lv_obj_send_event(app_mainmenu_ctx.pg_obj, LV_EVENT_SCROLL, NULL);
#endif

    /*Be sure the fist button is in the middle*/
    lv_obj_scroll_to_view(lv_obj_get_child(app_mainmenu_ctx.pg_obj, 0), LV_ANIM_OFF);

}

static void on_resume(void)
{


}
static void on_pause(void)
{
    mm_trans_anim_init();
#ifdef AUTO_CIRCLE_ANIM
    lv_anim_del(app_mainmenu_ctx.pg_obj, app_mainmenu_auto_circle);
#endif

    if (app_mainmenu_ctx.anim_obj)
    {
        lv_anim_del(app_mainmenu_ctx.anim_obj, NULL);
        app_mainmenu_ctx.anim_obj = NULL;
    }

}
static void on_stop(void)
{
    rt_free(app_mainmenu_ctx.list);
    app_mainmenu_ctx.list = NULL;
#ifdef DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE
    rt_free(app_mainmenu_ctx.label_list);
    app_mainmenu_ctx.label_list = NULL;
#endif
    rt_free(app_mainmenu_ctx.icon_pivot);
    app_mainmenu_ctx.icon_pivot = NULL;
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


static int app_mainmenu(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID, msg_handler);

    return 0;
}



BUILTIN_APP_EXPORT(LV_EXT_STR_ID(mainmenu), NULL, APP_ID, app_mainmenu);


