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

typedef enum
{
    pix_sta_space, // 空格已移出遮盖
    pix_sta_boom,  // 显示炸弹
    pix_sta_flag,  // 显示旗子
    pix_sta_cover  // 显示遮盖
} pix_sta_t;

typedef enum
{
    game_sta_start,   // 第一次运行
    game_sta_end,     // G了
    game_sta_success, // 成功
    game_sta_playing  // 游戏中
} game_sta_t;

typedef enum
{
    bcd_none,
    bcd_del_texts,
    bcd_down,
    bcd_up,
    bcd_back,
    bcd_ref_flags,
    bcd_smile,
    bcd_del_first_text,
} user_flag_bcd_t;

typedef struct
{
    lv_indev_t *indev_sys;          // 系统原输入设备
    lv_indev_t *indev_mouse;        // 鼠标设备
    lv_indev_drv_t indev_mouse_drv; // 物理驱动接口
    lv_obj_t *point_cuser_obj;      // 指针图标
    lv_point_t mouse;               // 鼠标坐标
    lv_point_t mouse_start;         // 鼠标按下坐标
    lv_point_t tp_rel_start;        // 触摸屏真实按下坐标
    lv_point_t choose_point;        // 选中游戏格子坐标
    uint8_t offx;                   // 偏移量
    uint8_t offy;                   // 偏移量
    uint16_t hor_num;               // 水平格子数
    uint16_t ver_num;               // 垂直格子数
    lv_obj_t **pix_obj;             // 格子对象
    lv_obj_t *scr;                  // 游戏界面
    lv_obj_t *game_box;             // 游戏格子窗口
    uint8_t slide_flag : 1;         // 滑动标志
    uint8_t time_cnt_en : 1;        // 游戏时长计时使能
    uint8_t click_flag : 1;         // 单击标志
    lv_timer_t *click_timer;        // 双击识别定时器
    lv_timer_t *game_time_timer;    // 游戏时长定时器
    lv_obj_t *back_scr;             // 备份进入游戏前界面
    lv_obj_t *label_time;           // 游戏时长显示
    lv_obj_t *label_flag;           // 游戏旗帜数量显示
    lv_obj_t *img_smile;            // 嘻嘻
} game_minesweeper_paras_t;

typedef struct
{
    uint8_t *pix_array_boom_sta;  // 炸弹状态
    uint8_t *pix_array_cover_sta; // 遮盖状态
    uint16_t flag_cnt;            // 旗帜数量
    uint16_t used_time_cnt;       // 游戏时长
    uint16_t removed_pix_cnt;     // 移除的格子数量
    game_sta_t game_sta;          // 游戏状态
    uint16_t boom_num;            // 炸弹数量
    float point_sensity;          // 指针灵敏度
} game_minesweeper_paras_save_t;

static const uint32_t number_color[] = {0x0803ED, 0x08760E, 0xF10204, 0x081B7F, 0x6C0B10, 0x0E7972, 0x0B082C, 0x817E7F};
static game_minesweeper_paras_t *paras;
static game_minesweeper_paras_save_t *paras_save;

static pix_sta_t pix_get_sta(uint8_t line, uint8_t row);              // 获取点当前状态
static void pix_set_sta(uint8_t line, uint8_t row, pix_sta_t sta);    // 设置点当前状态
static uint16_t pix_has_boom(uint8_t line, uint8_t row);              // 获取点是否为炸弹
static void pix_set_boom(uint8_t line, uint8_t row, uint8_t sta);     // 设置点是否为炸弹
static size_t get_pix_around_booms(uint8_t line, uint8_t row);        // 获取周围炸弹数量
static void create_boom_srand(uint8_t boom_num);                      // 创建随机炸弹
static void refresh_show();                                           // 刷新显示
static void change_pix_sta(uint8_t line, uint8_t row, pix_sta_t sta); // 更新点状态
static void change_smile_img(lv_obj_t *obj);                          // 刷新笑脸显示
static void create_around_booms_label(uint8_t line, uint8_t row);     // 创建显示周围炸弹数量
static void img_event(lv_event_t *e);                                 // 游戏整图事件
static void lv_obj_set_bcd(lv_obj_t *obj, uint8_t bcd);               // 设置状态码
static uint8_t lv_obj_get_bcd(lv_obj_t *obj);                         // 清除状态码
static void create_pix(uint8_t line, uint8_t row, pix_sta_t sta);     // 创建点
static void game_restart();                                           // 重开
static void space_grow(uint8_t line, uint8_t row);                    // 深度查找空白
static void chek_success();                                           // 判断游戏是否结束
static void click_timer_cb(lv_timer_t *e);                            // 单击判断定时
static void time_cnt_cb(lv_timer_t *e);                               // 游戏时长定时
static void timer_setting_cb(lv_timer_t *e);                          // 设置定时

static void mem_err_ret()
{
    rt_kprintf("game minesweeper memery alloc failed!\n");
}

static void change_smile_img(lv_obj_t *obj) // 更新笑脸图标
{
    if (paras_save->game_sta == game_sta_end) // 游戏失败
        lv_img_set_src(obj, &game_minesweeper_img_cry);
    else if (paras_save->game_sta == game_sta_success) // 通关
        lv_img_set_src(obj, &game_minesweeper_img_cool);
    else
        lv_img_set_src(obj, &game_minesweeper_img_smile); // 平时
}

static void setting_click_event(lv_event_t *e)
{
    user_flag_bcd_t bcd = lv_obj_get_bcd(e->target); // 获取事件码

    if (bcd == bcd_up) // 加
    {

        if (paras_save->boom_num < (paras->hor_num * paras->ver_num))
            paras_save->boom_num++;

        lv_label_set_text_fmt((lv_obj_t *)e->user_data, "%d", paras_save->boom_num);
        return;
    }
    else if (bcd == bcd_down) // 减
    {
        if (paras_save->boom_num > 0)
            paras_save->boom_num--;
        lv_label_set_text_fmt((lv_obj_t *)e->user_data, "%d", paras_save->boom_num);
        return;
    }
    else if (bcd == bcd_back) // 返回
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
    user_flag_bcd_t bcd = lv_obj_get_bcd(e->target); // 获取事件码

    if (bcd == bcd_del_first_text) // 删除第一个提示语的事件
    {

        paras->indev_sys = lv_event_get_indev(e); // 获取系统输入设备

        lv_indev_drv_init(&paras->indev_mouse_drv);
        paras->indev_mouse_drv.type = LV_INDEV_TYPE_POINTER;
        paras->indev_mouse_drv.read_cb = mouse_read_cb;
        paras->indev_mouse = lv_indev_drv_register(&paras->indev_mouse_drv);

        paras->point_cuser_obj = lv_img_create(lv_layer_sys());
        lv_img_set_src(paras->point_cuser_obj, &game_minesweeper_img_pointer);
        lv_indev_set_cursor(paras->indev_mouse, paras->point_cuser_obj); // 注册指针

        paras->mouse.x = LV_HOR_RES / 2;
        paras->mouse.y = LV_VER_RES / 2;

        lv_indev_enable(paras->indev_sys, 0); // 禁用系统输入设备

        lv_obj_set_bcd(e->target, bcd_none);         // 清除事件码
        while (lv_obj_get_child_cnt(paras->scr) > 1) // 清除空游戏场景以外所有内容
            lv_obj_del(lv_obj_get_child(paras->scr, 1));
        lv_obj_clear_flag(paras->game_box, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_event_cb(paras->scr, img_event);

        return;
    }

    if (bcd == bcd_del_texts) // 开始游戏
    {

        lv_obj_set_bcd(e->target, bcd_none);         // 清除事件码
        while (lv_obj_get_child_cnt(paras->scr) > 1) // 清除空游戏场景以外所有内容
            lv_obj_del(lv_obj_get_child(paras->scr, 1));

        lv_obj_t *flag = lv_img_create(paras->scr);
        lv_img_set_src(flag, &game_minesweeper_img_flag_big); // 显示旗帜图标
        lv_obj_align(flag, LV_ALIGN_TOP_LEFT, OLED_RADIUS_PIX + 20, paras->offy - game_minesweeper_img_flag_big.header.h - 10);

        paras->label_flag = lv_label_create(paras->scr); // 显示旗帜数量
        lv_label_set_text_fmt(paras->label_flag, "%d", paras_save->flag_cnt);

        lv_obj_set_style_text_font(paras->label_flag, &game_minesweeper_font_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(paras->label_flag, lv_color_hex(0xff0000), LV_PART_MAIN);
        lv_obj_align_to(paras->label_flag, flag, LV_ALIGN_OUT_RIGHT_MID, 20, 0);

        paras->img_smile = lv_img_create(paras->scr);                                     // 显示笑脸图标
        lv_obj_add_flag(paras->img_smile, LV_OBJ_FLAG_CLICKABLE);                         // 笑脸图标可点击
        lv_obj_add_event_cb(paras->img_smile, smile_click_event, LV_EVENT_CLICKED, NULL); // 添加点击事件
        change_smile_img(paras->img_smile);                                               // 显示笑脸

        paras->label_time = lv_label_create(paras->scr); // 显示游戏时长

        lv_label_set_text_fmt(paras->label_time, "%d", paras_save->used_time_cnt);

        lv_obj_set_style_text_font(paras->label_time, &game_minesweeper_font_24, LV_PART_MAIN);
        lv_obj_set_style_text_color(paras->label_time, lv_color_hex(0xff0000), LV_PART_MAIN);

        lv_obj_align(paras->img_smile, LV_ALIGN_TOP_RIGHT, -OLED_RADIUS_PIX - 20, paras->offy - game_minesweeper_img_smile.header.h - 10);
        lv_obj_align_to(paras->label_time, paras->img_smile, LV_ALIGN_OUT_LEFT_MID, -20, 0);

        paras->time_cnt_en = 1; // 允许游戏时长计时
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

    // 松手双击判定
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

    if (paras->click_flag == 0) // 开始双击计时
    {
        paras->click_timer = lv_timer_create(click_timer_cb, 300, &setting_timer); // 300ms后无点击判定为双击
        lv_timer_set_repeat_count(paras->click_timer, 1);
        paras->click_flag = 1;
        return;
    }
    else // 双击
    {
        lv_timer_del(paras->click_timer); // 删除单击定时
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

    if (*pix != NULL) // 已存在点
        return;

    pix_set_sta(line, row, sta); // 同步状态

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

    if (pix_sta == sta) // 状态无改变
        return;
    if (pix_sta == pix_sta_space) // 已经是空地
        return;
    if (pix_sta == pix_sta_boom) // 是结束的红色炸弹
        return;

    lv_obj_t **pix = paras->pix_obj + row * paras->hor_num + line;
    uint8_t send_event_code = 0;

    if (sta == pix_sta_flag) // 加旗子
    {
        paras_save->flag_cnt++;
        lv_img_set_src(*pix, &game_minesweeper_img_flag);
        send_event_code = 1;
    }
    else if (sta == pix_sta_cover) // 去旗子
    {
        paras_save->flag_cnt--;
        lv_img_set_src(*pix, &game_minesweeper_img_cover);
        send_event_code = 1;
    }
    else if (sta == pix_sta_boom) // 点到炸弹
    {
        paras->time_cnt_en = 0;                               // 停止计时
        paras_save->game_sta = game_sta_end;                  // 结束游戏
        lv_img_set_src(*pix, &game_minesweeper_img_red_boom); // 显示红色炸弹
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
    else // 打开空地
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
    if (send_event_code == 1) // 更新旗子数量
    {
        lv_obj_set_bcd(paras->game_box, bcd_ref_flags);
        lv_event_send(paras->game_box, LV_EVENT_RELEASED, NULL);
    }

    pix_set_sta(line, row, sta);
}

static void create_boom_srand(uint8_t boom_num)
{
    memset(paras_save->pix_array_boom_sta, 0, paras->hor_num * ((paras->ver_num + 4) / 8)); // 清除炸弹

    srand(MINESWEEPER_GET_MS_TIC); // 生成炸弹

    while (boom_num)
    {
        uint8_t row = rand() % paras->ver_num;
        uint8_t line = rand() % paras->hor_num;
        if (pix_has_boom(line, row))
            continue;

        pix_set_boom(line, row, 1);
        boom_num--;
    }

    while (lv_obj_get_child_cnt(paras->game_box)) // 清除屏幕所有对象
        lv_obj_del(lv_obj_get_child(paras->game_box, 0));

    for (size_t i = 0; i < paras->hor_num * paras->ver_num; i++) // 清空指针
        *(paras->pix_obj + i) = NULL;

    for (size_t i = 0; i < paras->hor_num; i++) // 重置所有点状态
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
    lv_obj_set_bcd(paras->game_box, bcd_del_texts); // 清除显示重新生成
    lv_event_send(paras->game_box, LV_EVENT_RELEASED, NULL);
}

static void time_cnt_cb(lv_timer_t *e) // 游戏时长定时
{
    if (paras_save->game_sta != game_sta_playing) // 没玩游戏不能计时
        return;

    if (paras->time_cnt_en != 1) // 使能没开不能计时
        return;

    if (paras_save->used_time_cnt < 9999) // 大于999
        paras_save->used_time_cnt++;
    else
        return;

    // 刷新显示
    lv_label_set_text_fmt(paras->label_time, "%d", paras_save->used_time_cnt);
    lv_obj_align_to(paras->label_time, paras->img_smile, LV_ALIGN_OUT_LEFT_MID, -20, 0);
}

static void click_timer_cb(lv_timer_t *e) // 单击判断定时
{
    paras->click_flag = 0;
    paras->click_timer = NULL;

    if (*(lv_timer_t **)e->user_data != NULL) // 长按中
        return;

    if (pix_get_sta(paras->choose_point.x, paras->choose_point.y) == pix_sta_cover) // 翻转标记状态
        change_pix_sta(paras->choose_point.x, paras->choose_point.y, pix_sta_flag);
    else if (pix_get_sta(paras->choose_point.x, paras->choose_point.y) == pix_sta_flag)
        change_pix_sta(paras->choose_point.x, paras->choose_point.y, pix_sta_cover);
}

static void timer_setting_cb(lv_timer_t *e) // 长按开启设置定时
{
    *(lv_timer_t **)e->user_data = (void *)1; // 定时器清除

    if (paras->slide_flag)
        return;

    paras->time_cnt_en = 0; // 游戏时长暂停

    while (lv_obj_get_child_cnt(paras->scr) > 1) // 清除左右两侧内容
        lv_obj_del(lv_obj_get_child(paras->scr, 1));

    // 创建菜单
    lv_obj_add_flag(paras->game_box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *label = lv_label_create(paras->scr);
    lv_label_set_text(label, "地雷数量  重开生效");
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(label, &game_minesweeper_font_24, LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 60);

    lv_obj_t *label_num = lv_label_create(paras->scr);
    lv_obj_set_style_text_color(label_num, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text_fmt(label_num, "%d", paras_save->boom_num);
    lv_obj_set_style_text_font(label_num, &game_minesweeper_font_24, LV_PART_MAIN);
    lv_obj_align(label_num, LV_ALIGN_CENTER, 0, 0);

    label = lv_label_create(paras->scr);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(label, "一");
    lv_obj_set_style_text_font(label, &game_minesweeper_font_24, LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, -60, 0);
    lv_obj_add_flag(label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_bcd(label, bcd_down);
    lv_obj_add_event_cb(label, setting_click_event, LV_EVENT_CLICKED, label_num);
    lv_obj_set_ext_click_area(label, 20);

    label = lv_label_create(paras->scr);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(label, "十");
    lv_obj_set_style_text_font(label, &game_minesweeper_font_24, LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 60, 0);
    lv_obj_add_flag(label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_bcd(label, bcd_up);
    lv_obj_add_event_cb(label, setting_click_event, LV_EVENT_CLICKED, label_num);
    lv_obj_set_ext_click_area(label, 20);

    label = lv_label_create(paras->scr);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(label, "X");
    lv_obj_set_style_text_font(label, &game_minesweeper_font_24, LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -60);
    lv_obj_add_flag(label, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_bcd(label, bcd_back);
    lv_obj_set_ext_click_area(label, 20);

    lv_obj_add_event_cb(label, setting_click_event, LV_EVENT_CLICKED, label_num);

    lv_indev_enable(paras->indev_sys, 1);
}

uint8_t isValid(int16_t line, int16_t row)
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
        return; // 越界、已访问或为地雷则返回

    grow_visited_set(grow_array, line, row); // 标记为已访问

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
                grow_visited_set(grow_array, newX, newY); // 标记为已访问
            }
        }
        change_pix_sta(current.line, current.row, pix_sta_space); // 清除点
    }

    MINESWEEPER_MEMERY_FREE(buffer);
    MINESWEEPER_MEMERY_FREE(grow_array);
}

void game_minesweeper_load()
{
    static uint8_t first_flg = 1;

    paras = MINESWEEPER_MEMERY_MALLOC(sizeof(game_minesweeper_paras_t));

    if (paras == NULL)
    {
        mem_err_ret();
        return;
    }

    memset(paras, 0, sizeof(game_minesweeper_paras_t));
    paras->hor_num = LV_HOR_RES / 24;
    paras->ver_num = (LV_VER_RES - OLED_RADIUS_PIX * 2) / 24;
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

    paras->pix_obj = MINESWEEPER_MEMERY_MALLOC(sizeof(lv_obj_t *) * paras->hor_num * paras->ver_num); // 申请对象内存

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

    paras->scr = lv_obj_create(NULL);                      // 创建屏幕对象
    lv_obj_clear_flag(paras->scr, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_bg_color(paras->scr, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(paras->scr, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    paras->game_box = lv_img_create(paras->scr); // 创建空地图
    lv_img_set_src(paras->game_box, &game_minesweeper_img_space);
    lv_obj_set_size(paras->game_box, paras->hor_num * 24, paras->ver_num * 24);
    lv_obj_set_pos(paras->game_box, paras->offx, paras->offy);
    lv_obj_add_flag(paras->game_box, LV_OBJ_FLAG_HIDDEN);

    if (first_flg) // 恢复
    {
        create_boom_srand(paras_save->boom_num);
        paras_save->game_sta = game_sta_playing;
    }

    refresh_show(); // 更新显示

    // 创建提示语
    lv_obj_t *label = lv_label_create(paras->scr);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(label, &game_minesweeper_font_24, LV_PART_MAIN);
    lv_label_set_text(label, "单击标记\n\n双击排雷\n\n长按设置\n\n笑脸重开");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_flag(paras->game_box, LV_OBJ_FLAG_CLICKABLE); // 开启地图点击使能
    lv_obj_set_bcd(paras->game_box, bcd_del_texts);
    lv_obj_add_event_cb(paras->game_box, img_event, LV_EVENT_CLICKED, NULL);  // 点击事件
    lv_obj_add_event_cb(paras->game_box, img_event, LV_EVENT_PRESSED, NULL);  // 按下事件
    lv_obj_add_event_cb(paras->game_box, img_event, LV_EVENT_RELEASED, NULL); // 松开事件

    lv_obj_set_bcd(paras->scr, bcd_del_first_text);                      // 点击后首先删除提示语
    lv_obj_add_event_cb(paras->scr, img_event, LV_EVENT_RELEASED, NULL); // 回调
    paras->back_scr = lv_scr_act();                                      // 记录当前屏幕
    lv_scr_load_anim(paras->scr, LV_SCR_LOAD_ANIM_FADE_ON, 0, 100, 0);   // 加载APP界面

    paras->game_time_timer = lv_timer_create(time_cnt_cb, 1000, NULL); // 游戏时长计时
    paras->time_cnt_en = 0;                                            // 有提示语不允许计时

    first_flg = 0;
}

void game_minesweeper_shutdown()
{
    lv_indev_delete(paras->indev_mouse);     // 删除鼠标输入
    lv_obj_del(paras->point_cuser_obj);      // 删除鼠标图标
    lv_indev_enable(paras->indev_sys, true); // 还原系统触摸

    lv_timer_del(paras->game_time_timer); // 删除游戏时长计时

    if (paras->click_timer != NULL)
        lv_timer_del(paras->click_timer);

    MINESWEEPER_MEMERY_FREE(paras->pix_obj); // 释放缓存
    MINESWEEPER_MEMERY_FREE(paras);

    lv_scr_load_anim(paras->back_scr, LV_SCR_LOAD_ANIM_FADE_ON, 0, 100, 1); // 加载界面
}
