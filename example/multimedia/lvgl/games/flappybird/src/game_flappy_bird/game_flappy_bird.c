/**
 * SPDX-FileCopyrightText: 启凡科创
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include "game_flappy_bird.h"

LV_IMG_DECLARE(game_flappy_bird_icon);

LV_IMG_DECLARE(game_flappy_bird_img_bird);
LV_IMG_DECLARE(game_flappy_bird_img_bird_0);
LV_IMG_DECLARE(game_flappy_bird_img_bird_1);
LV_IMG_DECLARE(game_flappy_bird_img_bird_2);
LV_IMG_DECLARE(game_flappy_bird_img_bg);
LV_IMG_DECLARE(game_flappy_bird_img_bg_grass);
LV_IMG_DECLARE(game_flappy_bird_img_pipeline);
LV_IMG_DECLARE(game_flappy_bird_img_pipeline_bottom);
LV_IMG_DECLARE(game_flappy_bird_img_pipeline_top);
LV_IMG_DECLARE(game_flappy_bird_img_bg_grass_all);
LV_IMG_DECLARE(game_flappy_bird_img_pipeline_long);
LV_IMG_DECLARE(game_flappy_bird_img_bg_up);
LV_IMG_DECLARE(game_flappy_bird_img_bg_down);

LV_FONT_DECLARE(game_flappy_bird_font_24);
LV_FONT_DECLARE(game_flappy_bird_font_22);

typedef enum
{
    bird_ready,
    bird_flying,
    bird_dead,
} bird_sta_t;

typedef struct
{
    int16_t point_top_x;
    int16_t point_top_y;
    lv_obj_t **objs;
    uint8_t chek_en;
} area_type_t;

static const void *bird_imgs[] = {&game_flappy_bird_img_bird_0, &game_flappy_bird_img_bird_1, &game_flappy_bird_img_bird_2, &game_flappy_bird_img_bird_1};
static uint8_t bird_imgs_cnt = 0;
static bird_sta_t bird_sta = bird_ready;
static int16_t bird_now_y = 0;
static float bird_set_y = -4;
static uint8_t scr_act = 0;
static lv_obj_t *score = NULL;
static uint16_t score_num = 0;
static uint16_t score_best = 0;
static lv_obj_t *home_page = NULL;
static uint32_t seed_cnt = 0;

static void ui_event(lv_event_t *e);             // EVENT
static void app_load_cb(void *arg);              // LOAD GAME
static void icon_click_event(lv_event_t *e);     // ICON CLICK EVENT
static void create_score(lv_obj_t *parent);      // SHOW SCORE
static void timer_cb_grass(lv_timer_t *e);       // REFRESH GRASS
static void timer_cb_bird_stanby(lv_timer_t *e); // REFRESH BIRD STANBY ANIM
static void timer_cb_flaying(lv_timer_t *e);     // REFRESH BIRD FLYING ANIM
static void pipeline_align(lv_obj_t **objs);     // ALIGN PIPELINES
static void pipeline_move(lv_timer_t *e);        // MOVE PIPELINES ANIM
static void create_pipeline(lv_timer_t *e);      // CREATE NEW PIPELINE
static void game_flappy_bird_exit();             // EXIT GAME

void game_flappy_bird_install(); // INSTALL GAME

static void icon_click_event(lv_event_t *e)
{
    app_load_cb(NULL);
}

static void ui_event(lv_event_t *e)
{
    if (e->code == LV_EVENT_CLICKED)
    {
        lv_obj_del((lv_obj_t *)e->user_data);
        lv_obj_del(e->target);
        bird_sta = bird_ready;
        lv_obj_t *img = lv_obj_get_child(lv_obj_get_child(lv_scr_act(), 0), 0);
        lv_img_set_src(img, &game_flappy_bird_img_bird);
        lv_img_set_angle(img, 0);
        lv_obj_set_y(img, 0);
        bird_set_y = -4;
        bird_now_y = 0;
        return;
    }

    if (e->code == LV_EVENT_GESTURE)
    {
        lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
        lv_scr_load_anim_t anim;
        if (dir == LV_DIR_LEFT)
            anim = LV_SCR_LOAD_ANIM_MOVE_LEFT;
        else if (dir == LV_DIR_RIGHT)
            anim = LV_SCR_LOAD_ANIM_MOVE_RIGHT;
        else
            return;
        game_flappy_bird_exit();
        return;
    }

    if (bird_sta == bird_ready)
    {
        bird_sta = bird_flying;
        score_num = 0;
        lv_obj_clear_flag(score, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(score, "0");
    }
    else if (bird_sta == bird_flying)
    {
        bird_set_y = -4;
    }
}

static void create_score(lv_obj_t *parent)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_style_bg_color(cont, lv_color_hex(0xDED895), 0); //
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);            /// Flags
    lv_obj_set_size(cont, 80, 100);
    lv_obj_align(cont, LV_ALIGN_CENTER, 0, -40);

    lv_obj_t *label = lv_label_create(cont);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text_fmt(label, "SCORE\n%d\nBEST\n%d", score_num, score_best);
    lv_obj_set_style_text_line_space(label, 5, 0);
    lv_obj_set_style_text_letter_space(label, 1, 0);
    lv_obj_set_style_text_font(label, &game_flappy_bird_font_22, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0x0), 0);
    lv_obj_center(label);

    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 100, 40);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0xE86101), 0);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 60);
    lv_obj_set_style_border_color(btn, lv_color_hex(0xffffff), 0);
    lv_obj_set_style_border_width(btn, 5, 0);
    lv_obj_set_style_outline_color(btn, lv_color_hex(0x542F00), 0);
    lv_obj_set_style_outline_width(btn, 2, 0);
    lv_obj_set_style_radius(btn, 0, 0);

    label = lv_label_create(btn);
    lv_label_set_text(label, "RESTART");
    lv_obj_set_style_text_font(label, &game_flappy_bird_font_22, 0);
    lv_obj_set_style_text_letter_space(label, 1, 0);
    lv_obj_center(label);
    lv_obj_set_style_text_color(label, lv_color_hex(0x0), 0);

    lv_obj_add_event_cb(btn, ui_event, LV_EVENT_CLICKED, cont);
}

static void timer_cb_grass(lv_timer_t *e)
{
    seed_cnt++;

    if (scr_act == 0)
        lv_timer_del(e);

    static int8_t offset = 9;

    if (bird_sta == bird_dead)
        return;

    lv_img_set_offset_x((lv_obj_t *)e->user_data, offset);
    offset--;
    if (offset == -1)
        offset = 9;
}

static void timer_cb_bird_stanby(lv_timer_t *e)
{
    if (scr_act == 0)
        lv_timer_del(e);

    if (bird_sta == bird_dead)
        return;

    if (bird_sta == bird_flying)
    {
        if (bird_set_y > 3)
            return;
        static uint8_t cnt = 0;
        cnt++;
        if (cnt < 3)
            return;
        cnt = 0;
        lv_img_set_src((lv_obj_t *)e->user_data, bird_imgs[bird_imgs_cnt++]);
        if (bird_imgs_cnt == 4)
            bird_imgs_cnt = 0;

        return;
    }

    if (bird_sta == bird_ready)
    {
        static int8_t offset = 0;
        static uint8_t dir = 0;
        static uint8_t cnt = 0;
        cnt++;
        if (cnt < 2)
            return;
        cnt = 0;
        lv_obj_set_y((lv_obj_t *)e->user_data, offset);
        if (dir)
            offset++;
        else
            offset--;

        if (offset == 3 || offset == -3)
            dir = !dir;
    }
}

static void timer_cb_flaying(lv_timer_t *e)
{
    if (scr_act == 0)
        lv_timer_del(e);
    if (bird_sta != bird_flying)
        return;

    static uint8_t img_change_flag = 0;
    static int16_t angle = -250;

    if (bird_set_y < 3)
    {
        lv_img_set_angle((lv_obj_t *)e->user_data, -250);
        img_change_flag = 1;
        angle -= 30;
        if (angle < -250)
            angle = -250;
    }
    else
    {
        if (img_change_flag)
        {
            img_change_flag = 0;
            lv_img_set_src((lv_obj_t *)e->user_data, &game_flappy_bird_img_bird);
        }

        angle += 45;
        if (angle > 900)
            angle = 900;

        lv_img_set_angle((lv_obj_t *)e->user_data, angle);
    }

    bird_set_y += 0.25;
    bird_now_y += bird_set_y;

    if (bird_now_y >= 100)
    {
        bird_now_y = 100;
        bird_sta = bird_dead;
        lv_obj_add_flag(score, LV_OBJ_FLAG_HIDDEN);
        if (score_num > score_best)
            score_best = score_num;
        create_score(lv_scr_act());
    }

    lv_obj_set_y((lv_obj_t *)e->user_data, bird_now_y);
}

static void pipeline_align(lv_obj_t **objs)
{
    lv_obj_align_to(objs[1], objs[0], LV_ALIGN_OUT_BOTTOM_MID, 0, -1);
    lv_obj_align_to(objs[2], objs[1], LV_ALIGN_OUT_BOTTOM_MID, 0, 73 + 17);
    lv_obj_align_to(objs[3], objs[2], LV_ALIGN_OUT_TOP_MID, 0, 0);
}

static void pipeline_move(lv_timer_t *e)
{
    area_type_t *area = e->user_data;

    if (scr_act == 0)
    {
        free(area->objs);
        free(area);
        lv_timer_del(e);
        return;
    }

    if (bird_sta == bird_ready)
    {
        for (size_t i = 0; i < 4; i++)
        {
            lv_obj_del(area->objs[i]);
        }
        free(area->objs);
        free(area);
        lv_timer_del(e);
        return;
    }

    if (bird_sta != bird_flying)
        return;

    int16_t x = lv_obj_get_x(area->objs[0]);
    lv_obj_set_x(area->objs[0], --x);
    pipeline_align(area->objs);
    if (x == -37)
    {
        for (size_t i = 0; i < 4; i++)
        {
            lv_obj_del(area->objs[i]);
        }
        free(area->objs);
        free(area);
        lv_timer_del(e);
        return;
    }
    if (x == 120)
    {
        area->chek_en = 1;
    }
    if (area->chek_en)
    {
        int16_t real_y = bird_now_y + 140 - 9;
        if (real_y < area->point_top_y || real_y > (area->point_top_y + 73 - 19))
        {
            bird_sta = bird_dead;
            lv_obj_add_flag(score, LV_OBJ_FLAG_HIDDEN);
            if (score_num > score_best)
                score_best = score_num;
            create_score(lv_scr_act());
            return;
        }
    }
    if (x == (120 - 37 - 23))
    {
        area->chek_en = 0;
        score_num++;
        lv_label_set_text_fmt(score, "%d", score_num);
    }
}

static void create_pipeline(lv_timer_t *e)
{
    static uint8_t cnt = 255 - 8;

    if (scr_act == 0)
    {
        lv_timer_del(e);
        return;
    }

    if (bird_sta != bird_flying)
    {
        cnt = 255 - 8;
        return;
    }

    cnt++;
    if (cnt == 16)
    {
        cnt = 0;

        lv_obj_t *scr = e->user_data;

        area_type_t *area = malloc(sizeof(area_type_t));
        area->objs = malloc(sizeof(lv_obj_t *) * 4);

        srand(bird_now_y + bird_set_y + seed_cnt);

        uint16_t pipeline_y = (rand() % 112) + 30;

        area->objs[0] = lv_img_create(scr);
        lv_img_set_src(area->objs[0], &game_flappy_bird_img_pipeline_long);
        lv_obj_set_height(area->objs[0], pipeline_y - 17);
        lv_obj_set_pos(area->objs[0], 240, 0);

        area->objs[1] = lv_img_create(scr);
        lv_img_set_src(area->objs[1], &game_flappy_bird_img_pipeline_top);

        area->objs[2] = lv_img_create(scr);
        lv_img_set_src(area->objs[2], &game_flappy_bird_img_pipeline_long);
        lv_obj_set_height(area->objs[2], 280 - 73 - 17 - 35 - pipeline_y);

        area->objs[3] = lv_img_create(scr);
        lv_img_set_src(area->objs[3], &game_flappy_bird_img_pipeline_bottom);

        pipeline_align(area->objs);

        area->point_top_x = 240;
        area->point_top_y = pipeline_y;
        area->chek_en = 0;

        lv_timer_t *timer = lv_timer_create(pipeline_move, 16, area);
        lv_timer_set_repeat_count(timer, -1);
    }
}

static void app_load_cb(void *arg)
{

    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE); /// Flags

    lv_obj_t *img = lv_img_create(scr);
    lv_img_set_src(img, &game_flappy_bird_img_bg);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_size(img, 240, 280);

    lv_obj_t *bird = lv_img_create(img);
    lv_img_set_src(bird, &game_flappy_bird_img_bird);
    lv_obj_align(bird, LV_ALIGN_CENTER, -13, 0);

    lv_timer_t *timer = lv_timer_create(timer_cb_bird_stanby, 50, bird);
    lv_timer_set_repeat_count(timer, -1);
    timer = lv_timer_create(timer_cb_flaying, 16, bird);
    lv_timer_set_repeat_count(timer, -1);

    lv_obj_t *grass = lv_img_create(img);
    lv_img_set_src(grass, &game_flappy_bird_img_bg_grass_all);
    lv_obj_align(grass, LV_ALIGN_CENTER, 0, 109);
    timer = lv_timer_create(timer_cb_grass, 16, grass);
    lv_timer_set_repeat_count(timer, -1);

    timer = lv_timer_create(create_pipeline, 150, img);
    lv_timer_set_repeat_count(timer, -1);

    score = lv_label_create(scr);
    lv_label_set_text(score, "0");
    lv_obj_set_style_text_font(score, &game_flappy_bird_font_22, 0);
    lv_obj_set_style_text_color(score, lv_color_hex(0x0), 0);
    lv_obj_align(score, LV_ALIGN_TOP_MID, 0, 52 - 20 / 2);
    lv_obj_add_flag(score, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_event_cb(scr, ui_event, LV_EVENT_PRESSED, bird);
    lv_obj_add_event_cb(scr, ui_event, LV_EVENT_GESTURE, NULL);

    lv_scr_load_anim(scr, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, 0); //  load APP interface

    bird_sta = bird_ready;
    bird_set_y = -4;
    bird_now_y = 0;

    scr_act = 1;
}

void game_flappy_bird_install()
{

    home_page = lv_obj_create(NULL);                      // create screen object
    lv_obj_clear_flag(home_page, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    lv_obj_set_style_bg_color(home_page, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(home_page, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *img = lv_img_create(home_page);
    lv_img_set_src(img, &game_flappy_bird_icon);
    lv_obj_add_flag(img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(img, icon_click_event, LV_EVENT_CLICKED, NULL);
    lv_img_set_zoom(img, 1.5 * 256);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *label = lv_label_create(home_page);
    lv_label_set_text(label, "Flappy Bird");
    lv_obj_set_style_text_font(label, &game_flappy_bird_font_24, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align_to(label, img, LV_ALIGN_OUT_BOTTOM_MID, 0, 30);
    lv_scr_load_anim(home_page, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, 0); //  load home page
}

static void game_flappy_bird_exit()
{
    scr_act = 0;
    lv_scr_load_anim(home_page, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, 1); //  load home page
}
