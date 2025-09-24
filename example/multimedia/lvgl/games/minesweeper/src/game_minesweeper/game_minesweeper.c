#include "game_minesweeper.h"

LV_IMG_DECLARE(game_minesweeper_icon);
LV_IMG_DECLARE(game_minesweeper_img_space);
LV_IMG_DECLARE(game_minesweeper_img_boom);
LV_IMG_DECLARE(game_minesweeper_img_cover);
LV_IMG_DECLARE(game_minesweeper_img_pointer);
LV_IMG_DECLARE(game_minesweeper_img_red_boom);
LV_IMG_DECLARE(game_minesweeper_img_flag);
LV_IMG_DECLARE(game_minesweeper_img_flag_big);
LV_IMG_DECLARE(game_minesweeper_img_smile);
LV_IMG_DECLARE(game_minesweeper_img_cry);
LV_IMG_DECLARE(game_minesweeper_img_cool);

LV_FONT_DECLARE(game_minesweeper_font_24);

static const char *language_ch[] = {
    "扫雷",
    "地雷数量\n重开生效",
    "单击标记\n\n双击排雷\n\n长按设置\n\n笑脸重开"};

static const char *language_en[] = {
    "Minsweeper",
    "Number of bomb\nNext round takes effect",
    "click to tag\n\ndouble click to open\n\nlong press to setting\n\nclick the smile to restart"};

enum
{
    string_name,
    string_setting,
    string_info,
};

typedef enum
{
    pix_sta_space, // Space has been uncovered
    pix_sta_boom,  //  Display bomb
    pix_sta_flag,  //  Display flag
    pix_sta_cover  // Display cover
} pix_sta_t;

typedef enum
{
    game_sta_start,   // First run
    game_sta_end,     // Game over
    game_sta_success, // Success
    game_sta_playing  // Game in progress
} game_sta_t;

typedef enum
{
    bcd_none,
    bcd_del_texts,      // Delete texts
    bcd_down,           // Down
    bcd_up,             // Up
    bcd_back,           // Back
    bcd_ref_flags,      // Refresh flags
    bcd_smile,          // Smile
    bcd_del_first_text, // Delete first text
} user_flag_bcd_t;

typedef struct
{
    lv_indev_t *indev_sys;          // Original system input device
    lv_indev_t *indev_mouse;        // Mouse device
    lv_indev_drv_t indev_mouse_drv; // Physical driver interface
    lv_obj_t *point_cuser_obj;      // Pointer icon
    lv_point_t mouse;               // Mouse coordinates
    lv_point_t mouse_start;         // Mouse press coordinates
    lv_point_t tp_rel_start;        // Touch screen real press coordinates
    lv_point_t choose_point;        // Selected game cell coordinates
    uint8_t offx;                   // Offset X
    uint8_t offy;                   // Offset Y
    uint16_t hor_num;               // Number of horizontal cells
    uint16_t ver_num;               // Number of vertical cells
    lv_obj_t **pix_obj;             // Cell objects
    lv_obj_t *scr;                  // Game screen
    lv_obj_t *game_box;             // Game cell window
    uint8_t slide_flag : 1;         // Slide flag
    uint8_t time_cnt_en : 1;        // Game time counter enable
    uint8_t click_flag : 1;         // Click flag
    lv_timer_t *click_timer;        // Double-click recognition timer
    lv_timer_t *game_time_timer;    // Game time timer
    lv_obj_t *back_scr;             // Backup interface before entering game
    lv_obj_t *label_time;           // Game time display
    lv_obj_t *label_flag;           // Game flag count display
    lv_obj_t *img_smile;            // Smiley face

} game_minesweeper_paras_t;

typedef struct
{
    uint8_t *pix_array_boom_sta;  // Bomb status
    uint8_t *pix_array_cover_sta; // Cover status
    uint16_t flag_cnt;            // Flag count
    uint16_t used_time_cnt;       // Game time used
    uint16_t removed_pix_cnt;     // Number of removed cells
    game_sta_t game_sta;          // Game status
    uint16_t boom_num;            // Bomb count
    float point_sensity;          // Pointer sensitivity
} game_minesweeper_paras_save_t;

static const uint32_t number_color[] = {0x0803ED, 0x08760E, 0xF10204, 0x081B7F, 0x6C0B10, 0x0E7972, 0x0B082C, 0x817E7F};
static game_minesweeper_paras_t *paras;
static game_minesweeper_paras_save_t *paras_save;
#if MINESWEEPER_LANGUAGE_DEFAULT
static const char **language = language_en; // english
#else
static const char **language = language_ch; // 中文
#endif

static pix_sta_t pix_get_sta(uint8_t line, uint8_t row);              // get current status
static void pix_set_sta(uint8_t line, uint8_t row, pix_sta_t sta);    // set current status
static uint16_t pix_has_boom(uint8_t line, uint8_t row);              // get whether it is a bomb
static void pix_set_boom(uint8_t line, uint8_t row, uint8_t sta);     // set whether it is a bomb
static size_t get_pix_around_booms(uint8_t line, uint8_t row);        // get the number of bombs around
static void create_boom_srand(uint8_t boom_num);                      // create random bombs
static void refresh_show();                                           // refresh display
static void change_pix_sta(uint8_t line, uint8_t row, pix_sta_t sta); // update cell status
static void change_smile_img(lv_obj_t *obj);                          // refresh smile display
static void create_around_booms_label(uint8_t line, uint8_t row);     // create display number of bombs around
static void img_event(lv_event_t *e);                                 // game image event
static void lv_obj_set_bcd(lv_obj_t *obj, uint8_t bcd);               // set status code
static uint8_t lv_obj_get_bcd(lv_obj_t *obj);                         // clear status code
static void create_pix(uint8_t line, uint8_t row, pix_sta_t sta);     // create cell
static void game_restart();                                           // restart
static void space_grow(uint8_t line, uint8_t row);                    // deep search for blank
static void chek_success();                                           // check if the game is over
static void click_timer_cb(lv_timer_t *e);                            // click judgment timer
static void time_cnt_cb(lv_timer_t *e);                               // game time counter
static void timer_setting_cb(lv_timer_t *e);                          // setting timer
static void game_minesweeper_load();

static void mem_err_ret()
{
    rt_kprintf("game minesweeper memery alloc failed!\n");
}

static void change_smile_img(lv_obj_t *obj) // update smile icon
{
    if (paras_save->game_sta == game_sta_end) // game over
        lv_img_set_src(obj, &game_minesweeper_img_cry);
    else if (paras_save->game_sta == game_sta_success) // pass
        lv_img_set_src(obj, &game_minesweeper_img_cool);
    else
        lv_img_set_src(obj, &game_minesweeper_img_smile); // normal
}

static void setting_click_event(lv_event_t *e)
{
    user_flag_bcd_t bcd = lv_obj_get_bcd(e->target); // get event code

    if (bcd == bcd_up) // add
    {

        if (paras_save->boom_num < (paras->hor_num * paras->ver_num))
            paras_save->boom_num++;

        lv_label_set_text_fmt((lv_obj_t *)e->user_data, "%d", paras_save->boom_num);
        return;
    }
    else if (bcd == bcd_down) // subtract
    {
        if (paras_save->boom_num > 0)
            paras_save->boom_num--;
        lv_label_set_text_fmt((lv_obj_t *)e->user_data, "%d", paras_save->boom_num);
        return;
    }
    else if (bcd == bcd_back) // back
    {
        lv_obj_set_bcd(paras->game_box, bcd_del_texts);
        lv_obj_clear_flag(paras->game_box, LV_OBJ_FLAG_HIDDEN);
        lv_event_send(paras->game_box, LV_EVENT_RELEASED, NULL);
        lv_indev_enable(paras->indev_sys, 0);

        return;
    }
}

static void smile_click_event(lv_event_t *e)
{
    game_restart();
}

static void icon_click_event(lv_event_t *e)
{
    game_minesweeper_load();
}

static void mouse_read_cb(lv_indev_drv_t *indev, lv_indev_data_t *data)
{
    static uint8_t pre_cnt = 0;
    lv_indev_data_t indev_data;
    static lv_indev_data_t last_indev_data;
    paras->indev_sys->driver->read_cb(&paras->indev_mouse_drv, &indev_data);

    if (indev_data.state == LV_INDEV_STATE_PR)
    {
        if (pre_cnt == 0)
        {
            pre_cnt++;
            paras->mouse_start = paras->mouse;
            paras->tp_rel_start = indev_data.point;
            paras->slide_flag = 0;
        }
        else
        {
            int cha_x = indev_data.point.x - paras->tp_rel_start.x;
            int cha_y = indev_data.point.y - paras->tp_rel_start.y;
            if (cha_x > MOUSE_SHAKE_AREA || cha_x < -MOUSE_SHAKE_AREA || cha_y > MOUSE_SHAKE_AREA || cha_y < -MOUSE_SHAKE_AREA)
                paras->slide_flag = 1;

            paras->mouse.x = paras->mouse_start.x + (indev_data.point.x - paras->tp_rel_start.x) * paras_save->point_sensity;
            paras->mouse.y = paras->mouse_start.y + (indev_data.point.y - paras->tp_rel_start.y) * paras_save->point_sensity;
            if (paras->mouse.x < 0)
                paras->mouse.x = 0;
            if (paras->mouse.x >= LV_HOR_RES)
                paras->mouse.x = LV_HOR_RES - 1;
            if (paras->mouse.y < 0)
                paras->mouse.y = 0;
            if (paras->mouse.y >= LV_VER_RES)
                paras->mouse.y = LV_VER_RES - 1;
        }
    }
    else
    {
        pre_cnt = 0;
    }

    data->point.x = paras->mouse.x;
    data->point.y = paras->mouse.y;
    data->state = indev_data.state;
}

static void img_event(lv_event_t *e)
{

    static lv_timer_t *setting_timer = NULL;
    user_flag_bcd_t bcd = lv_obj_get_bcd(e->target); // get event code

    if (bcd == bcd_del_first_text) // delete the first prompt event
    {

        paras->indev_sys = lv_event_get_indev(e); // get system input device

        lv_indev_drv_init(&paras->indev_mouse_drv);
        paras->indev_mouse_drv.type = LV_INDEV_TYPE_POINTER;
        paras->indev_mouse_drv.read_cb = mouse_read_cb;
        paras->indev_mouse = lv_indev_drv_register(&paras->indev_mouse_drv);

        paras->point_cuser_obj = lv_img_create(lv_layer_sys());
        lv_img_set_src(paras->point_cuser_obj, &game_minesweeper_img_pointer);
        lv_indev_set_cursor(paras->indev_mouse, paras->point_cuser_obj); // register pointer

        paras->mouse.x = LV_HOR_RES / 2;
        paras->mouse.y = LV_VER_RES / 2;

        lv_indev_enable(paras->indev_sys, 0); // disable system input device

        lv_obj_set_bcd(e->target, bcd_none);         // clear event code
        while (lv_obj_get_child_cnt(paras->scr) > 1) // clear all content except the empty game scene
            lv_obj_del(lv_obj_get_child(paras->scr, 1));
        lv_obj_clear_flag(paras->game_box, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_event_cb(paras->scr, img_event);

        return;
    }

    if (bcd == bcd_del_texts) // start game
    {

        lv_obj_set_bcd(e->target, bcd_none);         // clear event code
        while (lv_obj_get_child_cnt(paras->scr) > 1) // clear all content except the empty game scene
            lv_obj_del(lv_obj_get_child(paras->scr, 1));

        lv_obj_t *flag = lv_img_create(paras->scr);
        lv_img_set_src(flag, &game_minesweeper_img_flag_big); // show flag icon
        lv_obj_align(flag, LV_ALIGN_TOP_LEFT, AMOLED_RADIUS_PIX + 20, paras->offy - game_minesweeper_img_flag_big.header.h - 10);

        paras->label_flag = lv_label_create(paras->scr); // show flag number
        lv_label_set_text_fmt(paras->label_flag, "%d", paras_save->flag_cnt);

        lv_obj_set_style_text_font(paras->label_flag, &game_minesweeper_font_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(paras->label_flag, lv_color_hex(0xff0000), LV_PART_MAIN);
        lv_obj_align_to(paras->label_flag, flag, LV_ALIGN_OUT_RIGHT_MID, 20, 0);

        paras->img_smile = lv_img_create(paras->scr);                                     // show smile icon
        lv_obj_add_flag(paras->img_smile, LV_OBJ_FLAG_CLICKABLE);                         // smile icon can be clicked
        lv_obj_add_event_cb(paras->img_smile, smile_click_event, LV_EVENT_CLICKED, NULL); // add click event
        change_smile_img(paras->img_smile);                                               // show smile

        paras->label_time = lv_label_create(paras->scr); // show game time

        lv_label_set_text_fmt(paras->label_time, "%d", paras_save->used_time_cnt);

        lv_obj_set_style_text_font(paras->label_time, &game_minesweeper_font_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(paras->label_time, lv_color_hex(0xff0000), LV_PART_MAIN);

        lv_obj_align(paras->img_smile, LV_ALIGN_TOP_RIGHT, -AMOLED_RADIUS_PIX - 20, paras->offy - game_minesweeper_img_smile.header.h - 10);
        lv_obj_align_to(paras->label_time, paras->img_smile, LV_ALIGN_OUT_LEFT_MID, -20, 0);

        paras->time_cnt_en = 1; // allow game time counting
        return;
    }

    if (bcd == bcd_ref_flags)
    {
        lv_obj_set_bcd(e->target, bcd_none);
        lv_label_set_text_fmt(paras->label_flag, "%d", paras_save->flag_cnt);
    }

    if (e->code == LV_EVENT_PRESSED)
    {
        setting_timer = lv_timer_create(timer_setting_cb, 1500, &setting_timer);
        lv_timer_set_repeat_count(setting_timer, 1);
        return;
    }

    if (e->code == LV_EVENT_RELEASED)
    {
        if (setting_timer != NULL && setting_timer != (void *)1)
        {
            lv_timer_del(setting_timer);
            setting_timer = NULL;
        }
        return;
    }

    if (paras_save->game_sta != game_sta_playing)
        return;

    if (setting_timer == (void *)1)
        return;

    // CLICKED

    paras->choose_point = paras->mouse;

    // release double click judgment
    if (paras->slide_flag)
        return;

    if (paras->choose_point.x < paras->offx && paras->choose_point.x >= (paras->offx + 24 * paras->hor_num))
        return;
    if (paras->choose_point.y < paras->offy && paras->choose_point.y >= (paras->offy + 24 * paras->ver_num))
        return;
    paras->choose_point.x -= paras->offx;
    paras->choose_point.y -= paras->offy;
    paras->choose_point.x = paras->choose_point.x * paras->hor_num / (24 * paras->hor_num);
    paras->choose_point.y = paras->choose_point.y * paras->ver_num / (24 * paras->ver_num);

    if (paras->click_flag == 0) // start double click timing
    {
        paras->click_timer = lv_timer_create(click_timer_cb, 300, &setting_timer); // 300ms no click judgment is double click
        lv_timer_set_repeat_count(paras->click_timer, 1);
        paras->click_flag = 1;
        return;
    }
    else // double click
    {
        lv_timer_del(paras->click_timer); // delete single click timer
        paras->click_timer = NULL;
        paras->click_flag = 0;

        if (pix_get_sta(paras->choose_point.x, paras->choose_point.y) != pix_sta_cover)
            return;
        if (paras_save->game_sta == game_sta_end)
            return;

        if (pix_has_boom(paras->choose_point.x, paras->choose_point.y))
        {
            change_pix_sta(paras->choose_point.x, paras->choose_point.y, pix_sta_boom);
            return;
        }

        change_pix_sta(paras->choose_point.x, paras->choose_point.y, pix_sta_space);
        if (get_pix_around_booms(paras->choose_point.x, paras->choose_point.y))
        {
            chek_success();
            return;
        }

        space_grow(paras->choose_point.x, paras->choose_point.y);
        chek_success();
    }
}

static void lv_obj_set_bcd(lv_obj_t *obj, uint8_t bcd)
{
    if (bcd & 1)
        lv_obj_add_flag(obj, LV_OBJ_FLAG_USER_1);
    else
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_USER_1);
    if (bcd & 2)
        lv_obj_add_flag(obj, LV_OBJ_FLAG_USER_2);
    else
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_USER_2);
    if (bcd & 4)
        lv_obj_add_flag(obj, LV_OBJ_FLAG_USER_3);
    else
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_USER_3);
    if (bcd & 8)
        lv_obj_add_flag(obj, LV_OBJ_FLAG_USER_4);
    else
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_USER_4);
}

static uint8_t lv_obj_get_bcd(lv_obj_t *obj)
{
    uint8_t bcd = 0;
    if (lv_obj_has_flag(obj, LV_OBJ_FLAG_USER_1))
        bcd |= 1;
    if (lv_obj_has_flag(obj, LV_OBJ_FLAG_USER_2))
        bcd |= 2;
    if (lv_obj_has_flag(obj, LV_OBJ_FLAG_USER_3))
        bcd |= 4;
    if (lv_obj_has_flag(obj, LV_OBJ_FLAG_USER_4))
        bcd |= 8;
    return bcd;
}

static uint16_t get_line_row_boom(uint8_t line, uint8_t row)
{
    return (line + (row >> 3) * paras->hor_num);
}

static uint16_t get_line_row_cover(uint8_t line, uint8_t row)
{
    return (line + (row >> 2) * paras->hor_num);
}

static pix_sta_t pix_get_sta(uint8_t line, uint8_t row)
{
    return (pix_sta_t)((paras_save->pix_array_cover_sta[get_line_row_cover(line, row)] >> ((row % 4) << 1)) & 0x03);
}

static void pix_set_sta(uint8_t line, uint8_t row, pix_sta_t sta)
{
    paras_save->pix_array_cover_sta[get_line_row_cover(line, row)] &= ~(0x03 << ((row % 4) << 1));
    paras_save->pix_array_cover_sta[get_line_row_cover(line, row)] |= ((uint32_t)sta << ((row % 4) << 1));
}

static uint16_t pix_has_boom(uint8_t line, uint8_t row)
{
    return (paras_save->pix_array_boom_sta[get_line_row_boom(line, row)] & (1 << (row % 8)));
}

static void pix_set_boom(uint8_t line, uint8_t row, uint8_t sta)
{
    if (sta)
        paras_save->pix_array_boom_sta[get_line_row_boom(line, row)] |= (1 << (row % 8));
    else
        paras_save->pix_array_boom_sta[get_line_row_boom(line, row)] &= ~(1 << (row % 8));
}

static size_t get_pix_around_booms(uint8_t line, uint8_t row)
{
    size_t cnt = 0;

    int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

    for (size_t i = 0; i < 8; i++)
    {
        int x = line + dx[i];
        int y = row + dy[i];
        if (x < 0 || x >= paras->hor_num || y < 0 || y >= paras->ver_num)
            continue;
        if (pix_has_boom(x, y))
            cnt++;
    }
    return cnt;
}

static void chek_success()
{
    if ((paras_save->removed_pix_cnt + paras_save->boom_num) == (paras->hor_num * paras->ver_num))
    {
        paras_save->game_sta = game_sta_success;
        change_smile_img(paras->img_smile);
    }
}

static void create_around_booms_label(uint8_t line, uint8_t row)
{
    uint8_t cnt = get_pix_around_booms(line, row);
    if (cnt == 0)
        return;
    lv_obj_t **pix = paras->pix_obj + row * paras->hor_num + line;
    *pix = lv_label_create(paras->game_box);
    lv_obj_set_style_text_font(*pix, &game_minesweeper_font_24, LV_PART_MAIN);
    lv_label_set_text_fmt(*pix, "%d", cnt);
    lv_obj_set_pos(*pix, line * 24 + 4, row * 24 - 2);
    lv_obj_set_style_text_color(*pix, lv_color_hex(number_color[cnt - 1]), LV_PART_MAIN);
}

static void create_pix(uint8_t line, uint8_t row, pix_sta_t sta)
{
    lv_obj_t **pix = paras->pix_obj + row * paras->hor_num + line;

    if (*pix != NULL) //  pix has been created
        return;

    pix_set_sta(line, row, sta); // sync sta

    if (sta != pix_sta_space)
        *pix = lv_img_create(paras->game_box);

    switch (sta)
    {
    case pix_sta_cover:
        lv_img_set_src(*pix, &game_minesweeper_img_cover);
        break;

    case pix_sta_flag:
        lv_img_set_src(*pix, &game_minesweeper_img_flag);
        break;

    case pix_sta_boom:
        lv_img_set_src(*pix, &game_minesweeper_img_red_boom);
        break;

    case pix_sta_space:
        if (pix_has_boom(line, row))
        {

            *pix = lv_img_create(paras->game_box);
            lv_img_set_src(*pix, &game_minesweeper_img_boom);
        }
        else
        {
            create_around_booms_label(line, row);
            return;
        }
        break;
    default:
        break;
    }
    lv_obj_set_pos(*pix, line * 24, row * 24);
}

static void change_pix_sta(uint8_t line, uint8_t row, pix_sta_t sta)
{
    pix_sta_t pix_sta = pix_get_sta(line, row);

    if (pix_sta == sta) //  no change
        return;
    if (pix_sta == pix_sta_space) // has been opened
        return;
    if (pix_sta == pix_sta_boom) //  is red boom
        return;

    lv_obj_t **pix = paras->pix_obj + row * paras->hor_num + line;
    uint8_t send_event_code = 0;

    if (sta == pix_sta_flag) // add flag
    {
        paras_save->flag_cnt++;
        lv_img_set_src(*pix, &game_minesweeper_img_flag);
        send_event_code = 1;
    }
    else if (sta == pix_sta_cover) // remove flag
    {
        paras_save->flag_cnt--;
        lv_img_set_src(*pix, &game_minesweeper_img_cover);
        send_event_code = 1;
    }
    else if (sta == pix_sta_boom) //  boom
    {
        paras->time_cnt_en = 0;                               //  stop time
        paras_save->game_sta = game_sta_end;                  //  end game
        lv_img_set_src(*pix, &game_minesweeper_img_red_boom); // show red boom
        pix_set_sta(line, row, sta);
        for (size_t i = 0; i < paras->hor_num; i++)
        {
            for (size_t j = 0; j < paras->ver_num; j++)
            {
                if (pix_has_boom(i, j))
                    change_pix_sta(i, j, pix_sta_space);
            }
        }
        change_smile_img(paras->img_smile);
    }
    else //  open space
    {
        paras_save->removed_pix_cnt++;
        if (pix_has_boom(line, row))
        {
            lv_img_set_src(*pix, &game_minesweeper_img_boom);
        }
        else
        {
            lv_obj_del(*pix);
            create_around_booms_label(line, row);
        }
    }
    if (send_event_code == 1) //  update flag cnt
    {
        lv_obj_set_bcd(paras->game_box, bcd_ref_flags);
        lv_event_send(paras->game_box, LV_EVENT_RELEASED, NULL);
    }

    pix_set_sta(line, row, sta);
}

static void create_boom_srand(uint8_t boom_num)
{
    memset(paras_save->pix_array_boom_sta, 0, paras->hor_num * ((paras->ver_num + 4) / 8)); // clear boom

    srand(MINESWEEPER_GET_MS_TIC); // create boom

    while (boom_num)
    {
        uint8_t row = rand() % paras->ver_num;
        uint8_t line = rand() % paras->hor_num;
        if (pix_has_boom(line, row))
            continue;

        pix_set_boom(line, row, 1);
        boom_num--;
    }

    while (lv_obj_get_child_cnt(paras->game_box)) // clear all obj
        lv_obj_del(lv_obj_get_child(paras->game_box, 0));

    for (size_t i = 0; i < paras->hor_num * paras->ver_num; i++) // clear all pointer
        *(paras->pix_obj + i) = NULL;

    for (size_t i = 0; i < paras->hor_num; i++) // reset all point sta
    {
        for (size_t j = 0; j < paras->ver_num; j++)
            pix_set_sta(i, j, pix_sta_cover);
    }
}

static void refresh_show()
{
    for (size_t i = 0; i < paras->hor_num; i++)
    {
        for (size_t j = 0; j < paras->ver_num; j++)
            create_pix(i, j, pix_get_sta(i, j));
    }
}

static void game_restart()
{
    paras_save->removed_pix_cnt = 0;
    paras_save->flag_cnt = 0;
    paras_save->used_time_cnt = 0;
    create_boom_srand(paras_save->boom_num);
    refresh_show();
    paras_save->game_sta = game_sta_playing;
    lv_obj_set_bcd(paras->game_box, bcd_del_texts); // clear and create
    lv_event_send(paras->game_box, LV_EVENT_RELEASED, NULL);
}

static void time_cnt_cb(lv_timer_t *e) //  game time
{
    if (paras_save->game_sta != game_sta_playing) //  not playing
        return;

    if (paras->time_cnt_en != 1) //  enable not start
        return;

    if (paras_save->used_time_cnt < 9999) // overflow
        paras_save->used_time_cnt++;
    else
        return;

    //  refresh show
    lv_label_set_text_fmt(paras->label_time, "%d", paras_save->used_time_cnt);
    lv_obj_align_to(paras->label_time, paras->img_smile, LV_ALIGN_OUT_LEFT_MID, -20, 0);
}

static void click_timer_cb(lv_timer_t *e) //   click timer
{
    paras->click_flag = 0;
    paras->click_timer = NULL;

    if (*(lv_timer_t **)e->user_data != NULL) // long press
        return;

    if (pix_get_sta(paras->choose_point.x, paras->choose_point.y) == pix_sta_cover) //  change sta
        change_pix_sta(paras->choose_point.x, paras->choose_point.y, pix_sta_flag);
    else if (pix_get_sta(paras->choose_point.x, paras->choose_point.y) == pix_sta_flag)
        change_pix_sta(paras->choose_point.x, paras->choose_point.y, pix_sta_cover);
}

static void timer_setting_cb(lv_timer_t *e) //  long press start setting
{
    *(lv_timer_t **)e->user_data = (void *)1; // clear timer

    if (paras->slide_flag)
        return;

    paras->time_cnt_en = 0; // game time stop

    while (lv_obj_get_child_cnt(paras->scr) > 1) //  clear left and right
        lv_obj_del(lv_obj_get_child(paras->scr, 1));

    //  create menu
    lv_obj_add_flag(paras->game_box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *label = lv_label_create(paras->scr);
    lv_label_set_text(label, language[string_setting]);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(label, &game_minesweeper_font_24, LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 100);

    lv_obj_t *label_num = lv_label_create(paras->scr);
    lv_obj_set_style_text_color(label_num, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text_fmt(label_num, "%d", paras_save->boom_num);
    lv_obj_set_style_text_font(label_num, &game_minesweeper_font_24, LV_PART_MAIN);
    lv_obj_align(label_num, LV_ALIGN_CENTER, 0, 0);

    label = lv_label_create(paras->scr);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(label, "一");
    lv_obj_set_style_text_font(label, &game_minesweeper_font_24, LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, -100, 0);
    lv_obj_add_flag(label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_bcd(label, bcd_down);
    lv_obj_add_event_cb(label, setting_click_event, LV_EVENT_CLICKED, label_num);
    lv_obj_set_ext_click_area(label, 40);

    label = lv_label_create(paras->scr);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(label, "十");
    lv_obj_set_style_text_font(label, &game_minesweeper_font_24, LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 100, 0);
    lv_obj_add_flag(label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_bcd(label, bcd_up);
    lv_obj_add_event_cb(label, setting_click_event, LV_EVENT_CLICKED, label_num);
    lv_obj_set_ext_click_area(label, 40);

    label = lv_label_create(paras->scr);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(label, "X");
    lv_obj_set_style_text_font(label, &game_minesweeper_font_24, LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -100);
    lv_obj_add_flag(label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_bcd(label, bcd_back);
    lv_obj_set_ext_click_area(label, 40);

    lv_obj_add_event_cb(label, setting_click_event, LV_EVENT_CLICKED, label_num);

    lv_indev_enable(paras->indev_sys, 1);
}

static uint8_t isValid(int16_t line, int16_t row)
{

    return line >= 0 && line < paras->hor_num && row >= 0 && row < paras->ver_num;
}

static void grow_visited_set(uint8_t *grow_array, uint8_t line, uint8_t row)
{
    grow_array[get_line_row_boom(line, row)] |= (1 << (row % 8));
}

static uint8_t grow_visited_get(uint8_t *grow_array, uint8_t line, uint8_t row)
{
    return (grow_array[get_line_row_boom(line, row)] & (1 << (row % 8)));
}

static void space_grow(uint8_t line, uint8_t row)
{
    uint8_t *grow_array = MINESWEEPER_MEMERY_MALLOC(paras->hor_num * ((paras->ver_num + 4) >> 3));
    if (grow_array == NULL)
    {
        mem_err_ret();
        return;
    }
    memset(grow_array, 0, paras->hor_num * ((paras->ver_num + 4) >> 3));

    if (!isValid(line, row) || grow_visited_get(grow_array, line, row) || pix_has_boom(line, row) || pix_get_sta(line, row) == pix_sta_flag)
        return; //  return if out of range, visited or boom

    grow_visited_set(grow_array, line, row); // mark as visited

    struct buf
    {
        int16_t line;
        int16_t row;
    };

    struct buf *buffer = MINESWEEPER_MEMERY_MALLOC(sizeof(struct buf) * paras->hor_num * paras->ver_num);
    if (buffer == NULL)
    {
        MINESWEEPER_MEMERY_FREE(grow_array);
        mem_err_ret();
        return;
    }
    int16_t cnt = 0;

    buffer[cnt] = (struct buf){line, row};

    while (cnt >= 0)
    {
        struct buf current = buffer[cnt--];
        if (get_pix_around_booms(current.line, current.row) == 0)
        {
            int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
            int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

            for (int i = 0; i < 8; ++i)
            {
                int newX = current.line + dx[i];
                int newY = current.row + dy[i];
                if (!isValid(newX, newY) || grow_visited_get(grow_array, newX, newY) || pix_has_boom(newX, newY) || pix_get_sta(newX, newY) == pix_sta_flag)
                    continue;
                buffer[++cnt] = (struct buf){newX, newY};
                grow_visited_set(grow_array, newX, newY); // mark as visited
            }
        }
        change_pix_sta(current.line, current.row, pix_sta_space); // clear the point
    }

    MINESWEEPER_MEMERY_FREE(buffer);
    MINESWEEPER_MEMERY_FREE(grow_array);
}

static void game_minesweeper_load()
{
    static uint8_t first_flg = 1;

    paras->hor_num = LV_HOR_RES / 24;
    paras->ver_num = (LV_VER_RES - AMOLED_RADIUS_PIX * 2) / 24;
    paras->offx = (LV_HOR_RES - paras->hor_num * 24) / 2;
    paras->offy = (LV_VER_RES - paras->ver_num * 24) / 2;

    if (first_flg)
    {
        paras_save = MINESWEEPER_MEMERY_MALLOC(sizeof(game_minesweeper_paras_t));
        if (paras_save == NULL)
        {
            MINESWEEPER_MEMERY_FREE(paras);
            mem_err_ret();
            return;
        }
        memset(paras_save, 0, sizeof(game_minesweeper_paras_t));
        paras_save->point_sensity = MOUSE_SENSITY_DEFAULT;
        paras_save->boom_num = paras->hor_num * paras->ver_num / 10;

        paras_save->pix_array_boom_sta = MINESWEEPER_MEMERY_MALLOC(paras->hor_num * ((paras->ver_num + 4) / 8));
        if (paras_save->pix_array_boom_sta == NULL)
        {
            MINESWEEPER_MEMERY_FREE(paras);
            mem_err_ret();
            return;
        }

        paras_save->pix_array_cover_sta = MINESWEEPER_MEMERY_MALLOC(paras->hor_num * ((paras->ver_num + 4) / 8) * 2);
        if (paras_save->pix_array_cover_sta == NULL)
        {
            MINESWEEPER_MEMERY_FREE(paras_save->pix_array_boom_sta);
            MINESWEEPER_MEMERY_FREE(paras);
            mem_err_ret();
            return;
        }

        memset(paras_save->pix_array_boom_sta, 0, paras->hor_num * ((paras->ver_num + 4) / 8));
        memset(paras_save->pix_array_cover_sta, 0, paras->hor_num * ((paras->ver_num + 4) / 8) * 2);
    }

    paras->pix_obj = MINESWEEPER_MEMERY_MALLOC(sizeof(lv_obj_t *) * paras->hor_num * paras->ver_num); // apply for object memory

    if (paras->pix_obj == NULL)
    {
        MINESWEEPER_MEMERY_FREE(paras_save->pix_array_boom_sta);
        MINESWEEPER_MEMERY_FREE(paras_save->pix_array_cover_sta);
        MINESWEEPER_MEMERY_FREE(paras);
        mem_err_ret();
        return;
    }

    for (size_t i = 0; i < paras->hor_num * paras->ver_num; i++)
        *(paras->pix_obj + i) = NULL;

    paras->scr = lv_obj_create(NULL);                      // create screen object
    lv_obj_clear_flag(paras->scr, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_bg_color(paras->scr, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(paras->scr, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    paras->game_box = lv_img_create(paras->scr); //  create empty map
    lv_img_set_src(paras->game_box, &game_minesweeper_img_space);
    lv_obj_set_size(paras->game_box, paras->hor_num * 24, paras->ver_num * 24);
    lv_obj_set_pos(paras->game_box, paras->offx, paras->offy);
    lv_obj_add_flag(paras->game_box, LV_OBJ_FLAG_HIDDEN);

    if (first_flg) // restore
    {
        create_boom_srand(paras_save->boom_num);
        paras_save->game_sta = game_sta_playing;
    }

    refresh_show(); // update display

    // create prompt
    lv_obj_t *label = lv_label_create(paras->scr);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(label, &game_minesweeper_font_24, LV_PART_MAIN);
    lv_label_set_text(label, language[string_info]);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_flag(paras->game_box, LV_OBJ_FLAG_CLICKABLE); //  enable map click
    lv_obj_set_bcd(paras->game_box, bcd_del_texts);
    lv_obj_add_event_cb(paras->game_box, img_event, LV_EVENT_CLICKED, NULL);  // click event
    lv_obj_add_event_cb(paras->game_box, img_event, LV_EVENT_PRESSED, NULL);  // press event
    lv_obj_add_event_cb(paras->game_box, img_event, LV_EVENT_RELEASED, NULL); // release event

    lv_obj_set_bcd(paras->scr, bcd_del_first_text);                      //  click to delete the prompt first
    lv_obj_add_event_cb(paras->scr, img_event, LV_EVENT_RELEASED, NULL); //  callback
    lv_scr_load_anim(paras->scr, LV_SCR_LOAD_ANIM_FADE_ON, 0, 100, 1);   //  load APP interface

    paras->game_time_timer = lv_timer_create(time_cnt_cb, 1000, NULL); //   game time counting
    paras->time_cnt_en = 0;                                            //   no timing when there is a prompt

    first_flg = 0;
}

void game_minesweeper_shutdown()
{
    lv_indev_delete(paras->indev_mouse);     // delete mouse input
    lv_obj_del(paras->point_cuser_obj);      // delete mouse icon
    lv_indev_enable(paras->indev_sys, true); // restore system touch

    lv_timer_del(paras->game_time_timer); //  delete game time counting

    if (paras->click_timer != NULL)
        lv_timer_del(paras->click_timer);

    MINESWEEPER_MEMERY_FREE(paras->pix_obj); //  free cache
    MINESWEEPER_MEMERY_FREE(paras);

    lv_scr_load_anim(paras->back_scr, LV_SCR_LOAD_ANIM_FADE_ON, 0, 100, 1); // load interface
}

void game_minesweeper_install(void)
{

    paras = MINESWEEPER_MEMERY_MALLOC(sizeof(game_minesweeper_paras_t));

    if (paras == NULL)
    {
        mem_err_ret();
        return;
    }

    memset(paras, 0, sizeof(game_minesweeper_paras_t));

    paras->back_scr = lv_scr_act(); // record the current screen

    paras->scr = lv_obj_create(NULL);                      // create screen object
    lv_obj_clear_flag(paras->scr, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_bg_color(paras->scr, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(paras->scr, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *img = lv_img_create(paras->scr);
    lv_img_set_src(img, &game_minesweeper_icon);
    lv_obj_add_flag(img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(img, icon_click_event, LV_EVENT_CLICKED, NULL);
    lv_img_set_zoom(img, 2 * 256);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *label = lv_label_create(paras->scr);
    lv_label_set_text(label, language[string_name]);
    lv_obj_set_style_text_font(label, &game_minesweeper_font_24, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align_to(label, img, LV_ALIGN_OUT_BOTTOM_MID, 0, 30);
    lv_scr_load_anim(paras->scr, LV_SCR_LOAD_ANIM_FADE_ON, 0, 100, 0); //  load APP interface
}
