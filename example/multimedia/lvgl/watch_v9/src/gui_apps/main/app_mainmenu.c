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

#include "bf0_lib.h"
#include "cell_transform.h"
#include "log.h"
#include "custom_trans_anim.h"
#include "lv_display.h"
#include "lv_types.h"

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
/*------------------------------------mainmenu_layout_celluar-----------------------------*/


#include "lv_math.h"

/**********Private macros**************/
/*a virtual circle include gap between icons*/
#if (LV_VER_RES_MAX > LV_HOR_RES_MAX)//LV_VER_RES_MAX: 448 LV_HOR_RES_MAX: 368
    #define ICON_OUTER_RADIUS  (LV_VER_RES_MAX / 9)//ICON_OUTER_RADIUS (49)
#else
    #define ICON_OUTER_RADIUS  (LV_HOR_RES_MAX / 9)
#endif

#define ICON_OUTER_DIAMETER  (ICON_OUTER_RADIUS * 2)//ICON_OUTER_DIAMETER (98)
/*full fill with picture*/
#define ICON_INNER_RADIUS  ((ICON_OUTER_RADIUS * 8)/9)//ICON_INNER_RADIUS (43)
#define ICON_INNER_DIAMETER  (ICON_INNER_RADIUS * 2) //ICON_INNER_DIAMETER (86)

/**********Export macros(used in app_mainmenu.c)**************/
#define MAX_APP_ROW_NUM   16
#define MAX_APP_COL_NUM   16

#define ICON_IMG_WIDTH      ICON_OUTER_DIAMETER
#define ICON_IMG_HEIGHT     ICON_OUTER_DIAMETER

#if (MAX_APP_ROW_NUM > MAX_APP_COL_NUM)// PAGE_SCRL_WIDTH 1838
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








//for layout_icon_transform
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
//for limit_round
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
//没调用
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

//for layout_icon_transform
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


static void layout_get_icons_init_coordinate(int32_t row_idx, int32_t col_idx,    lv_coord_t *x, lv_coord_t *y)
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



    //cos(60):16384, sin(60):28378 ICON_OUTER_RADIUS 49
    *x = ((col_idx - row_idx) * lv_trigo_cos(60) * ICON_OUTER_RADIUS) >> (LV_TRIGO_SHIFT - 1);
    *y = ((col_idx + row_idx) * lv_trigo_sin(60) * ICON_OUTER_RADIUS) >> (LV_TRIGO_SHIFT - 1);

    *x += C0R0_COORD_X;
    *y += C0R0_COORD_Y;



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

    *p_col = col + (MAX_APP_COL_NUM >> 1);
    *p_row = row + (MAX_APP_ROW_NUM >> 1);
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
    if (get_icon_transform_param(float_x, float_y, float_icon_r, &float_x, &float_y, &float_icon_r, &pivot_r, LV_HOR_RES_MAX, LV_VER_RES_MAX))
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
        // rt_kprintf("layout_icon_transform failed\n");
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



/*------------------------------------mainmenu_cellular-----------------------------*/

/********************
 *      INCLUDES
 *********************/
#include "gui_app_fwk.h"
#define _MODULE_NAME_ "mainmenu"

#define APP_ID "Main"

//#define DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE
// #define APPLICATION_LIST_SORT
#ifdef APPLICATION_LIST_SORT
static const app_list_sort_t mainmenu_app_sort_table[] =
{
#ifdef APP_TLV_USED
    "Tileview",
#endif
    "Setting",
};
uint16_t mainmenu_app_sort_cnt()
{
    return sizeof(mainmenu_app_sort_table) / sizeof(mainmenu_app_sort_table[0]);
}
#endif


typedef struct
{
    lv_obj_t *pg_obj;
    lv_obj_t *encoder;
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
    /*
        The final state zoom
        using to create an animation from current zoom to
        target_zoom
        it avaiable value maybe:
            0.6 - overview state
            1.0 - normal mode
    */
    float target_zoom;
    lv_point_t comp_vect;
    lv_indev_t *indev;
    uint8_t row_idx;
    uint8_t col_idx;
    /*
        We NOT use lv_obj scroll, so here are scroll states
    */
    bool scroll_actived;
    lv_point_t scroll_sum;

} mainmenu_cell_t;

static mainmenu_cell_t *p_menu_cell = NULL;
static void mainmenu_cell_icons_event_cb(lv_event_t *e);
static int32_t keypad_handler(lv_key_t key, lv_indev_state_t event);

static lv_obj_t **mainmenu_cell_get_icon_obj(uint32_t row_idx, uint32_t col_idx)
{
    if ((row_idx  >= MAX_APP_ROW_NUM) || (col_idx >= MAX_APP_COL_NUM))
    {
        LOG_E("Array index out of bounds: row=%d, col=%d", row_idx, col_idx);
        return NULL;
    }

    return &(p_menu_cell->list[col_idx * MAX_APP_ROW_NUM + row_idx]);
}

static lv_point_t *mainmenu_cell_get_icon_pivot(uint32_t row_idx, uint32_t col_idx)
{
    if ((row_idx  >= MAX_APP_ROW_NUM) || (col_idx >= MAX_APP_COL_NUM))
    {
        return NULL;
    }

    return &(p_menu_cell->icon_pivot[col_idx * MAX_APP_ROW_NUM + row_idx]);
}


#ifdef DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE
static lv_obj_t **mainmenu_cell_get_label_obj(uint32_t row_idx, uint32_t col_idx)
{
    if ((row_idx  >= MAX_APP_ROW_NUM) || (col_idx >= MAX_APP_COL_NUM))
        return NULL;

    return &(p_menu_cell->label_list[col_idx * MAX_APP_ROW_NUM + row_idx]);
}
static void mainmenu_cell_draw_icon_label(uint8_t row_idx, uint8_t col_idx, float pi_x, float pi_y, float w, float h, float r)
{
    lv_obj_t *label, *icon;
    char buff[50];

    label = *mainmenu_cell_get_label_obj(row_idx, col_idx);
    icon = *mainmenu_cell_get_icon_obj(row_idx, col_idx);
    snprintf(buff, sizeof(buff) - 1, "[%d,%0.0f,%0.0f]",
             mainmenu_cell_get_icon_pivot(row_idx, col_idx)->y, pi_y, w);
    lv_label_set_text(label, buff);
    lv_obj_align_to(label, icon, LV_ALIGN_CENTER, 0, 0);
}

static void mainmenu_cell_debug_slider_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_current_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (event == LV_EVENT_VALUE_CHANGED)
    {
        float *p_v = lv_obj_get_user_data(obj);
        if (p_v)
        {
            *p_v = lv_slider_get_value(obj);

            LOG_I("debug_slider_cb %d, %d, %d, %d",
                  lv_slider_get_value(p_menu_cell->param_ctrl[0]),
                  lv_slider_get_value(p_menu_cell->param_ctrl[1]),
                  lv_slider_get_value(p_menu_cell->param_ctrl[2]),
                  lv_slider_get_value(p_menu_cell->param_ctrl[3]));
            //Update scren
            lv_obj_invalidate(lv_layer_top());
        }
    }
}

void mainmenu_cell_debug_set_params(uint8_t idx, int16_t min, int16_t max, lv_obj_user_data_t p_data)
{
    lv_slider_set_range(p_menu_cell->param_ctrl[idx], min, max);
    lv_obj_set_user_data(p_menu_cell->param_ctrl[idx], p_data);

    float *p_v = (float *) p_data;
    lv_bar_set_value(p_menu_cell->param_ctrl[idx], (int16_t) *p_v, LV_ANIM_ON);
}
#endif /* DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE */

static void mainmenu_cell_img_set_zoom(lv_obj_t *obj, uint16_t zoom)
{
    lv_image_t *img = (lv_image_t *)obj;
    uint16_t img_zoom = lv_img_get_zoom(obj);
    if (zoom == img_zoom) return;

    if (zoom == 0) zoom = 1;

    lv_coord_t w = lv_obj_get_width(obj);
    lv_coord_t h = lv_obj_get_height(obj);
    lv_area_t a;
    lv_image_buf_get_transformed_area(&a, w, h, 0, img_zoom >> 8, img_zoom >> 8, &img->pivot);
    a.x1 += obj->coords.x1 - 1;
    a.y1 += obj->coords.y1 - 1;
    a.x2 += obj->coords.x1 + 1;
    a.y2 += obj->coords.y1 + 1;
    lv_obj_invalidate_area(obj, &a);


    img->scale_x = zoom;
    img->scale_y = zoom;

    /* Disable invalidations because lv_obj_refresh_ext_draw_size would invalidate
    * the whole ext draw area */
    lv_disp_t *disp = lv_obj_get_disp(obj);
    lv_disp_enable_invalidation(disp, false);
    lv_obj_refresh_ext_draw_size(obj);
    lv_disp_enable_invalidation(disp, true);

    lv_image_buf_get_transformed_area(&a, w, h, 0, img_zoom, img_zoom, &img->pivot);
    a.x1 += obj->coords.x1 - 1;
    a.y1 += obj->coords.y1 - 1;
    a.x2 += obj->coords.x1 + 1;
    a.y2 += obj->coords.y1 + 1;
    lv_obj_invalidate_area(obj, &a);
}

static lv_obj_t *mainmenu_cell_add_icons(lv_obj_t *parent, const char *cmd, const void *img, uint8_t row_idx, uint8_t col_idx)
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
    cmd_str = lv_malloc(s_len);
    memcpy(cmd_str, cmd, s_len - 1);
    cmd_str[s_len - 1] = 0;

    lv_obj_set_user_data(icon, (void *)cmd_str);
    lv_obj_add_flag(icon, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_EVENT_BUBBLE | LV_OBJ_FLAG_PRESS_LOCK);

    lv_obj_clear_flag(icon, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(icon, mainmenu_cell_icons_event_cb, LV_EVENT_ALL, 0);
    lv_img_set_src(icon, img);
    *mainmenu_cell_get_icon_obj(row_idx, col_idx) = icon;

#ifdef DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE
    lv_obj_t *label = lv_label_create(parent);
    lv_ext_set_local_font(label, FONT_SMALL, LV_COLOR_WHITE);

    *mainmenu_cell_get_label_obj(row_idx, col_idx) = label;
#endif
    return icon;
}

static int32_t mainmenu_cell_draw_icons(lv_obj_t *obj, float pi_x, float pi_y, float w, float h)
{
    uint16_t zoom;

    if ((w != 0) && (h != 0))
    {
        lv_coord_t img_w = lv_obj_get_self_width(obj);
        lv_coord_t img_h = lv_obj_get_self_height(obj);

        // lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
        obj->flags &= (~LV_OBJ_FLAG_HIDDEN);

        //Updata zoom
        zoom = (uint16_t)(w * 256 / (float)img_w);
        mainmenu_cell_img_set_zoom(obj, zoom);

        //Move icon
        int32_t pi_x_10p8 = pi_x * 256;
        int32_t pi_y_10p8 = pi_y * 256;


        pi_x_10p8 -= ((img_w >> 1) << 8);
        pi_y_10p8 -= ((img_h >> 1) << 8);


        lv_obj_set_pos(obj, ((lv_coord_t)(pi_x_10p8 >> 8)) + lv_obj_get_scroll_x(p_menu_cell->pg_obj),
                       ((lv_coord_t)(pi_y_10p8 >> 8)) + lv_obj_get_scroll_y(p_menu_cell->pg_obj));

    }
    else
    {
        // lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
        obj->flags |= LV_OBJ_FLAG_HIDDEN;
        zoom = 0;
    }
    return zoom;
}

static void mainmenu_cell_icons_coord_init(void)
{
    lv_coord_t x, y;
    uint8_t row_idx, col_idx;

    for (row_idx = 0; row_idx < MAX_APP_ROW_NUM; row_idx++)
        for (col_idx = 0; col_idx < MAX_APP_COL_NUM; col_idx++)
        {
            rt_kprintf("mainmenu_cell_icons_coord_init: row_idx=%d, col_idx=%d\n", row_idx, col_idx);
            layout_get_icons_init_coordinate(row_idx, col_idx, &x, &y);

            rt_kprintf("init coordinate_B: x=%d, y=%d\n", x, y);
            mainmenu_cell_get_icon_pivot(row_idx, col_idx)->x = x;
            mainmenu_cell_get_icon_pivot(row_idx, col_idx)->y = y;

            lv_obj_t *icon = *mainmenu_cell_get_icon_obj(row_idx, col_idx);
            if (icon)
            {
                lv_coord_t img_w = lv_obj_get_self_width(icon);
                lv_coord_t img_h = lv_obj_get_self_height(icon);
                uint16_t zoom = (uint16_t)(ICON_IMG_WIDTH * 256 / img_w);



                lv_obj_set_pos(icon, x - (img_w >> 1), y - (img_h >> 1));
                lv_img_set_pivot(icon, (img_w >> 1), (img_h >> 1));
                lv_img_set_zoom(icon, zoom);
            }
        }
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
static void mainmainmenu_cell_transform(bool force_refresh)
{
    lv_coord_t x_offset, y_offset;
    uint8_t row_idx, col_idx;
    float min_delta;
    static lv_coord_t c0r0_x, c0r0_y;
    //rt_uint32_t level = rt_hw_interrupt_disable();

    //uint32_t tick = (uint32_t)HAL_GTIMER_READ();
    lv_coord_t cur_c0r0_x = C0R0_COORD_X - lv_obj_get_scroll_x(p_menu_cell->pg_obj);
    lv_coord_t cur_c0r0_y = C0R0_COORD_Y - lv_obj_get_scroll_y(p_menu_cell->pg_obj);

    if (c0r0_x == cur_c0r0_x \
            && c0r0_y == cur_c0r0_y \
            && p_menu_cell->target_zoom == p_menu_cell->zoom \
            && !force_refresh)
    {
        return;
    }


    c0r0_x = cur_c0r0_x;
    c0r0_y = cur_c0r0_y;

    x_offset = c0r0_x - mainmenu_cell_get_icon_pivot(0, 0)->x;
    y_offset = c0r0_y - mainmenu_cell_get_icon_pivot(0, 0)->y;

    min_delta = LV_VER_RES;

    for (row_idx = 0; row_idx < MAX_APP_ROW_NUM; row_idx++)
        for (col_idx = 0; col_idx < MAX_APP_COL_NUM; col_idx++)
        {
            //offset icon pivot
            mainmenu_cell_get_icon_pivot(row_idx, col_idx)->x += x_offset;
            mainmenu_cell_get_icon_pivot(row_idx, col_idx)->y += y_offset;

            if (*mainmenu_cell_get_icon_obj(row_idx, col_idx))
            {
                float float_x = (float)mainmenu_cell_get_icon_pivot(row_idx, col_idx)->x;
                float float_y = (float)mainmenu_cell_get_icon_pivot(row_idx, col_idx)->y;
                float float_icon_w = (float) ICON_IMG_WIDTH;
                float float_icon_h = (float) ICON_IMG_HEIGHT;
                float pivot_r; //Icon pivot to screen pivot
                float zoom;

                if (p_menu_cell->zoom < 1)
                {
                    // rt_kprintf("zoom < 1 %f\n", p_menu_cell->zoom);
                    zoom = p_menu_cell->zoom;
                    zoom = zoom * zoom;
                    float_x = MM_ZOOM(LV_HOR_RES_MAX >> 1, float_x, zoom);
                    float_y = MM_ZOOM(LV_VER_RES_MAX >> 1, float_y, zoom);
                    float_icon_w *= zoom;
                    float_icon_h *= zoom;
                }
                if (p_menu_cell->zoom > 0)
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

                            min_delta = pivot_r;
                            //p_menu_cell->ciotsc.x = float_x - scr_center.x;
                            //p_menu_cell->ciotsc.y = float_y - scr_center.y;
                            p_menu_cell->cicon = *mainmenu_cell_get_icon_obj(row_idx, col_idx);
                        }

                        if ((p_menu_cell->zoom > 0) && (p_menu_cell->zoom <= 1))
                        {
                            zoom = p_menu_cell->zoom;
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

                if (p_menu_cell->zoom > 1)
                {
                    // rt_kprintf("zoom > 1 %f\n", p_menu_cell->zoom);
                    zoom = 1 / (2 - p_menu_cell->zoom);

                    float_x = MM_ZOOM(LV_HOR_RES_MAX >> 1, float_x, zoom);
                    float_y = MM_ZOOM(LV_VER_RES_MAX >> 1, float_y, zoom);
                    float_icon_w *= zoom;
                    float_icon_h *= zoom;
                }
                //draw icon
                mainmenu_cell_draw_icons(*mainmenu_cell_get_icon_obj(row_idx, col_idx), float_x, float_y, float_icon_w, float_icon_h);
                //     *mainmenu_cell_get_icon_obj(row_idx, col_idx), float_x, float_y, float_icon_w, float_icon_h, zoom);

#ifdef DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE
                mainmenu_cell_draw_icon_label(row_idx, col_idx, float_x, float_y, float_icon_w, float_icon_h, pivot_r);
#endif
            }
        }

    //uint32_t tick1 = (uint32_t)HAL_GTIMER_READ() - tick;
    // LOG_I("%s:tick1 %d  ms %d", __func__, tick1, tick1 / 10);
    //rt_hw_interrupt_enable(level);
}

static void mainmenu_cell_set_zoom(lv_obj_t *obj, lv_coord_t z)
{
    p_menu_cell->zoom = ((float)z) / 1000;
    lv_obj_invalidate(obj);
}

static rt_err_t mainmenu_cell_get_icon_col_row(lv_obj_t *obj, uint8_t *row_idx, uint8_t *col_idx)
{
    uint8_t i, j;

    if (NULL == obj)
        return RT_EEMPTY;

    for (i = 0; i < MAX_APP_ROW_NUM; i++)
        for (j = 0; j < MAX_APP_COL_NUM; j++)
        {
            if (obj == *mainmenu_cell_get_icon_obj(i, j))
            {
                *row_idx = i;
                *col_idx = j;
                return RT_EOK;
            }
        }

    return RT_EEMPTY;
}



# if 1
static void mainmenu_cell_focus_icon(lv_obj_t *obj, uint32_t max_anim_ms, lv_anim_ready_cb_t cbk)
{
    lv_coord_t sx, sy, dx, dy;
    uint8_t row_idx, col_idx;
    lv_area_t parent_area;

    //LOG_I("app_mainmenu_focus_icon");
    if (RT_EOK != mainmenu_cell_get_icon_col_row(obj, &row_idx, &col_idx))
        return;

    LOG_I("app_mainmenu_focus_icon: row %d, col %d", row_idx, col_idx);
    p_menu_cell->row_idx = row_idx;
    p_menu_cell->col_idx = col_idx;

    parent_area.x1 = 0;
    parent_area.y1 = 0;
    parent_area.x2 = parent_area.x1 + LV_HOR_RES - 1;
    parent_area.y2 = parent_area.y1 + LV_VER_RES - 1;

    sx = lv_obj_get_scroll_x(p_menu_cell->pg_obj);
    sy = lv_obj_get_scroll_y(p_menu_cell->pg_obj);

    dx = sx + ((parent_area.x2 + 1 - parent_area.x1) >> 1) - mainmenu_cell_get_icon_pivot(row_idx, col_idx)->x;
    dy = sy + ((parent_area.y2 + 1 - parent_area.y1) >> 1) - mainmenu_cell_get_icon_pivot(row_idx, col_idx)->y;



    p_menu_cell->anim_obj = NULL;

    lv_anim_delete(obj, NULL);

    if (max_anim_ms)
    {
        lv_anim_t ax;
        uint16_t anim_duration;
        lv_sqrt_res_t ds;
        uint32_t abs_x, abs_y;

        if (p_menu_cell->indev)
        {
            p_menu_cell->indev->pointer.scroll_throw_vect.x = 0;
            p_menu_cell->indev->pointer.scroll_throw_vect.y = 0;
        }
        abs_x = (dx > sx) ? (dx - sx) : (sx - dx);
        abs_y = (dy > sy) ? (dy - sy) : (sy - dy);

        lv_sqrt(abs_x * abs_x + abs_y * abs_y, &ds, 0x8000);

        /*
            Start animation except central icon
        */
        {
            anim_duration = max_anim_ms * ds.i / LV_HOR_RES_MAX ;
            anim_duration = LV_MIN(anim_duration, max_anim_ms);

            //rt_kprintf("app_mainmenu_focus_icon anim_duration %d ms\n", anim_duration);

            lv_anim_init(&ax);
            lv_anim_set_time(&ax, anim_duration);

            lv_anim_set_var(&ax, obj);
            lv_anim_set_exec_cb(&ax, (lv_anim_exec_xcb_t) lv_obj_set_x);
            lv_anim_set_values(&ax, sx, dx);
            lv_anim_start(&ax);

            lv_anim_set_exec_cb(&ax, (lv_anim_exec_xcb_t) lv_obj_set_y);
            lv_anim_set_values(&ax, sy, dy);
            lv_anim_set_ready_cb(&ax, cbk);
            lv_anim_start(&ax);
            p_menu_cell->anim_obj = obj;

            if (cbk)
            {
                keypad_handler_register(keypad_handler);
            }

            return;
        }
    }
    lv_obj_scroll_to(obj, dx, dy, LV_ANIM_OFF);

    if (cbk)
    {
        lv_anim_t ax;
        ax.var = obj;
        cbk(&ax);
    }
    keypad_handler_register(NULL);
}

#endif

static void mainmenu_cell_print_col_row(lv_obj_t *obj)
{
    uint8_t row_idx, col_idx;

    if (RT_EOK == mainmenu_cell_get_icon_col_row(obj, &row_idx, &col_idx))
    {
        //LOG_I("mainmenu_cell_print_col_row %x [%d,%d]", obj, row_idx, col_idx);
    }
}

static lv_obj_t *mainmenu_cell_get_nearest_icons(lv_point_t *target)
{
    lv_obj_t *ret_v = NULL;
    uint8_t row_idx, col_idx;
    uint8_t first = 1;
    float min_delta = 0, pivot_r;

    for (row_idx = 0; row_idx < MAX_APP_ROW_NUM; row_idx++)
        for (col_idx = 0; col_idx < MAX_APP_COL_NUM; col_idx++)
        {
            uint32_t temp;
            lv_coord_t x, y;

            lv_sqrt_res_t ds;

            x = mainmenu_cell_get_icon_pivot(row_idx, col_idx)->x;
            y = mainmenu_cell_get_icon_pivot(row_idx, col_idx)->y;

            temp = (x - target->x) * (x - target->x) + (y - target->y) * (y - target->y);

            lv_sqrt(temp, &ds, 0x8000);

            pivot_r = ds.i + ds.f / 256;

            if (((pivot_r < min_delta) || first) && (NULL != *mainmenu_cell_get_icon_obj(row_idx, col_idx)))
            {
                //vect2target->x =  target->x - x;
                //vect2target->y =  target->y - y;

                ret_v = *mainmenu_cell_get_icon_obj(row_idx, col_idx);

                //rt_kprintf("get_nearest_icon_vect  [%d,%d] %d,%d vect %d, %d, %.1f\n", row_idx, col_idx, x, y, vect2target->x, vect2target->y, pivot_r);
                min_delta = pivot_r;

                first = 0;
            }
        }

    LV_ASSERT_NULL(ret_v);
    return  ret_v;
}

static lv_obj_t *mainmenu_cell_predict_focus_icon(void)
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
    scroll_throw = indev->scroll_throw;

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
    //get_nearest_icon_vect(&scr_center, &p_menu_cell->comp_vect);

    return mainmenu_cell_get_nearest_icons(&scr_center);
}

static void refr_start_cb(lv_event_t *e)
{
    if (p_menu_cell->scroll_actived)
    {
        _lv_obj_scroll_by_raw(p_menu_cell->pg_obj,
                              p_menu_cell->scroll_sum.x,
                              p_menu_cell->scroll_sum.y);

        p_menu_cell->scroll_sum.x = 0;
        p_menu_cell->scroll_sum.y = 0;

        mainmainmenu_cell_transform(false);

        mainmenu_cell_print_col_row(p_menu_cell->cicon);
    }

}

static void mainmenu_cell_page_event_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_current_target(e);
    lv_event_code_t event = lv_event_get_code(e);

    if (LV_EVENT_PRESSED == event)
    {
        //Clear anim obj in case stop scroll to been aborted
        p_menu_cell->anim_obj = NULL;
        lv_obj_clear_flag(p_menu_cell->pg_obj, LV_OBJ_FLAG_SCROLLABLE);
        lv_anim_delete(p_menu_cell->pg_obj, NULL);
    }
    else if (LV_EVENT_RELEASED == event \
             || LV_EVENT_PRESS_LOST == event \
             || LV_EVENT_CLICKED == event)
    {
        lv_obj_add_flag(p_menu_cell->pg_obj, LV_OBJ_FLAG_SCROLLABLE);

        if (p_menu_cell->scroll_actived)
        {
            mainmenu_cell_print_col_row(mainmenu_cell_predict_focus_icon());
            lv_obj_scroll_to_view(mainmenu_cell_predict_focus_icon(), LV_ANIM_ON);
        }

        p_menu_cell->scroll_actived = false;
        p_menu_cell->scroll_sum.x = 0;
        p_menu_cell->scroll_sum.y = 0;

    }
    else if (LV_EVENT_PRESSING == event)
    {
        lv_indev_t *indev = lv_indev_get_act();
        lv_point_t p;

        lv_indev_get_vect(indev, &p);

        p_menu_cell->scroll_sum.x += p.x;
        p_menu_cell->scroll_sum.y += p.y;

        if ((LV_ABS(p_menu_cell->scroll_sum.x) > indev->scroll_limit)
                || (LV_ABS(p_menu_cell->scroll_sum.y) > indev->scroll_limit)
                || p_menu_cell->scroll_actived)
        {
            p_menu_cell->scroll_actived = true;
            //_lv_obj_scroll_by_raw(p_menu_cell->pg_obj, p.x, p.y); //scroll once before draw to speed up
            lv_obj_invalidate(p_menu_cell->pg_obj);
        }

    }
    else if (LV_EVENT_SCROLL == event)
    {
        mainmainmenu_cell_transform(false);
    }
    /*else if (LV_EVENT_DRAW_MAIN_BEGIN == event)//原文件是注释的
    {
        if (p_menu_cell->scroll_actived)
        {
            _lv_obj_scroll_by_raw(p_menu_cell->pg_obj,
                                p_menu_cell->scroll_sum.x,
                                p_menu_cell->scroll_sum.y);

            p_menu_cell->scroll_sum.x = 0;
            p_menu_cell->scroll_sum.y = 0;
        }
        mainmainmenu_cell_transform(false);
        mainmenu_cell_print_col_row(p_menu_cell->cicon);
    }*/


    else if (LV_EVENT_SCROLL_END == event)
    {
        if (p_menu_cell->anim_obj)
        {
            char *cmd = (char *)lv_obj_get_user_data(p_menu_cell->anim_obj);

            if (cmd)
            {
                LOG_I("app mainmenu icon click anim cbk\n");
// #if defined(APP_MENU_EXT_USED)
//                 for (uint8_t idx = 0; idx < menu_ext_icons_table_size_get(); idx++)
//                 {
//                     if (0 == strcmp(cmd, mainmenu_ext_icons_table[idx].page_id))
//                     {
//                         gui_app_run_subpage("setting", mainmenu_ext_icons_table[idx].page_id, NULL);
//                         p_menu_cell->anim_obj = NULL;
//                         return;
//                     }
//                 }
// #endif
                gui_app_run(cmd);
            }
            p_menu_cell->anim_obj = NULL;
        }
    }
    else if (event == LV_EVENT_DRAW_MAIN)
    {
        // rt_kprintf("mainmenu_cell_icons_event_cb\n");
    }


}

static void mainmenu_cell_icons_event_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_current_target(e);
    lv_event_code_t event = lv_event_get_code(e);
    bool monkey = false;
#if defined (BSP_USING_LVGL_INPUT_AGENT)
    monkey = (0 != monkey_mode()) ? true : false;
#endif

    //LOG_I("icon_event_callback %s\n", lv_event_to_name(event));
    if ((LV_EVENT_RELEASED == event \
            || LV_EVENT_PRESS_LOST == event) \
            && p_menu_cell->scroll_actived)
    {
        //Not to clear scroll_actived before icon receieve click event
        lv_event_stop_bubbling(e);
    }
    else if ((LV_EVENT_SHORT_CLICKED == event) \
             && (monkey || !p_menu_cell->scroll_actived))
    {
        LOG_I("app mainmenu icon clickd: monkey %d \n", monkey);
        lv_obj_add_flag(p_menu_cell->pg_obj, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_scroll_to_view(obj, LV_ANIM_ON);
        p_menu_cell->anim_obj = obj;

        bool ready_to_jump = true;
        if (NULL != lv_anim_get(p_menu_cell->pg_obj, NULL))
            ready_to_jump = (NULL != lv_anim_get(p_menu_cell->pg_obj, (lv_anim_exec_xcb_t)mainmenu_cell_set_zoom)) ? true : false;

        ready_to_jump = monkey ? true : ready_to_jump;
        //There is no scroll animation if the clicked icon was just at center of pg_obj
        if (ready_to_jump)
        {
            lv_anim_delete(p_menu_cell->pg_obj, NULL);
            //Send SCROLL_END msg manually
            lv_obj_send_event(p_menu_cell->pg_obj, LV_EVENT_SCROLL_END, NULL);

        }
    }
    else if (LV_EVENT_DELETE == event)
    {
        char *cmd = (char *)lv_obj_get_user_data(obj);
        if (cmd)
            lv_free(cmd);
    }
}




static inline int16_t mainmenu_cell_reorder_wf_icon(int16_t idx, int16_t clock_idx, const builtin_app_desc_t *builtin_app, lv_obj_t *page)
{
    uint16_t col, row;

    //Fix 1st icon for clock
    if (0 == strcmp("clock", builtin_app->id))
    {
        lv_obj_t *icon;
        layout_get_icon_col_row_by_idx(clock_idx, &col, &row);

        icon = mainmenu_cell_add_icons(page, "clock", builtin_app->icon, row, col);
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
        mainmenu_cell_add_icons(page, builtin_app->id, builtin_app->icon, row, col);
        idx++;
    }

    return idx;
}

static void mainmenu_cell_read_app_icons(lv_obj_t *page)
{
    uint16_t col, row;
    uint16_t idx = 1, clock_idx;
    const builtin_app_desc_t *p_builtin_app;
    int mainmenu_icon_style;

    clock_idx = 0; /* 0 - reserved for clock app*/
    mainmenu_icon_style = 1; // 0/1 is first style

    /*1. load builtin app list*/
    // BUILTIN_APP_LIST_OPEN(mainmenu_icon_style, p_builtin_app);
    p_builtin_app = gui_builtin_app_list_open();
    if (p_builtin_app)
    {
        while (p_builtin_app)
        {
            //Fix 1st icon for clock
            idx = mainmenu_cell_reorder_wf_icon(idx, clock_idx, p_builtin_app, page);
            LOG_I("find3 app %s", p_builtin_app->id); // 先打印再移动指针

            p_builtin_app = gui_builtin_app_list_get_next(p_builtin_app);
        }

        while (1)
        {
            //Fix 1st icon for clock
            p_builtin_app = gui_script_app_list_get_next(p_builtin_app);
            if (p_builtin_app == NULL)
                break;
            idx = mainmenu_cell_reorder_wf_icon(idx, clock_idx, p_builtin_app, page);
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

            lv_obj_t *p_obj = mainmenu_cell_add_icons(page, "none", dummy_icons[i], row, col);

            lv_obj_set_style_img_opa(p_obj, LV_OPA_50, LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
#endif

}

static void mainmenu_cell_redraw_app(lv_obj_t *parent)
{

    mainmenu_cell_read_app_icons(parent);
    mainmenu_cell_icons_coord_init();
    lv_obj_scroll_to_view(lv_obj_get_child(p_menu_cell->pg_obj, 0), LV_ANIM_OFF);
    uint16_t col, row;
    if (0 == p_menu_cell->row_idx && 0 == p_menu_cell->col_idx)
    {
        layout_get_icon_col_row_by_idx(0, &col, &row);
    }
    else
    {
        row = p_menu_cell->row_idx;
        col = p_menu_cell->col_idx;
    }
    mainmenu_cell_focus_icon(*mainmenu_cell_get_icon_obj(row, col), 0, NULL);

    lv_obj_send_event(p_menu_cell->pg_obj, LV_EVENT_RELEASED, NULL);

}

static void mainmenu_cell_clean_app(lv_obj_t *parent)
{
    lv_obj_clean(parent);
}
static void mainmenu_cell_ui_init(void *param)
{

    lv_obj_t *page = lv_obj_create(lv_scr_act());

    lv_obj_set_size(page, LV_HOR_RES_MAX, LV_VER_RES_MAX);

    lv_obj_set_style_bg_color(page, LV_COLOR_BLACK, LV_PART_MAIN);

    lv_obj_align(page, LV_ALIGN_CENTER, 0, 0);


    lv_obj_set_style_border_width(page, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_scroll_snap_x(page, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scroll_snap_y(page, LV_SCROLL_SNAP_CENTER);

    p_menu_cell->pg_obj = page;

    lv_obj_set_scrollbar_mode(page, LV_SCROLLBAR_MODE_OFF);

    lv_obj_add_event_cb(page, mainmenu_cell_page_event_cb, LV_EVENT_ALL, NULL);

    lv_display_add_event_cb(lv_display_get_default(), refr_start_cb, LV_EVENT_REFR_START, NULL);
}

static void on_start(void)
{


    uint16_t max_icons;
    p_menu_cell = (mainmenu_cell_t *)rt_calloc(1, sizeof(mainmenu_cell_t));
    RT_ASSERT(p_menu_cell);

    p_menu_cell->zoom = 1;
    p_menu_cell->target_zoom = 1;


    max_icons = MAX_APP_COL_NUM * MAX_APP_ROW_NUM;
    p_menu_cell->list = rt_calloc(1, max_icons * sizeof(lv_obj_t *));
    RT_ASSERT(p_menu_cell->list);


#ifdef DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE
    p_menu_cell->label_list = rt_calloc(1, max_icons * sizeof(lv_obj_t *));
    RT_ASSERT(p_menu_cell->label_list);
#endif

    p_menu_cell->icon_pivot = rt_calloc(1, max_icons * sizeof(lv_point_t));
    RT_ASSERT(p_menu_cell->icon_pivot);

    mainmenu_cell_ui_init(NULL);
    rt_kprintf("mainmenu_cell_ui_init\n");


#if (LV_FB_LINE_NUM != LV_VER_RES_MAX)
    mainmenu_cell_redraw_app(p_menu_cell->pg_obj);
#endif
}

static void on_resume(void)
{
#if (LV_FB_LINE_NUM == LV_VER_RES_MAX)

    mainmenu_cell_redraw_app(p_menu_cell->pg_obj);
#endif

    mainmainmenu_cell_transform(true);
    lv_obj_send_event(p_menu_cell->pg_obj, LV_EVENT_SCROLL, NULL);

#ifdef  BSP_USING_LVGL_INPUT_AGENT
    if (monkey_mode())
    {
        uint16_t col, row;
        layout_get_icon_col_row_by_idx(0, &col, &row);
        mainmenu_cell_focus_icon(*mainmenu_cell_get_icon_obj(row, col), 0, NULL);
    }
#endif


}

static void on_pause(void)
{
    if (p_menu_cell->anim_obj)
    {
        lv_anim_delete(p_menu_cell->anim_obj, NULL);
        p_menu_cell->anim_obj = NULL;
    }
#if (LV_FB_LINE_NUM == LV_VER_RES_MAX)
    mainmenu_cell_clean_app(p_menu_cell->pg_obj);
#endif

}

static void on_stop(void)
{
    if (p_menu_cell->list)
        rt_free(p_menu_cell->list);
    if (p_menu_cell->icon_pivot)
        rt_free(p_menu_cell->icon_pivot);
#ifdef DEBUG_APP_MAINMENU_DISPLAY_ICON_COORDINATE
    if (p_menu_cell->label_list)
        rt_free(p_menu_cell->label_list);
#endif
    if (p_menu_cell->anim_obj)
        lv_anim_delete(p_menu_cell->anim_obj, NULL);
    rt_free(p_menu_cell);
    p_menu_cell = NULL;
}
#if 1
static int32_t keypad_handler(lv_key_t key, lv_indev_state_t event)
{
    if (LV_KEY_HOME == key)
    {
        /*Block go back event*/
        return LV_BLOCK_EVENT;
    }
    else
    {
        return LV_POST_EVENT;
    }
}
#endif

static void msg_handler(gui_app_msg_type_t msg, void *param)
{
    static uint8_t start = 0;

    switch (msg)
    {
    case GUI_APP_MSG_ONSTART:
        on_start();//
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
        ;
    }

}


static int app_mainmenu(intent_t i)
{
    gui_app_regist_msg_handler(APP_ID, msg_handler);

    return 0;
}



BUILTIN_APP_EXPORT(LV_EXT_STR_ID(mainmenu), NULL, APP_ID, app_mainmenu);




