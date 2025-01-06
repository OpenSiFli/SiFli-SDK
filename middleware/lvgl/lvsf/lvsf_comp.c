/**
  ******************************************************************************
  * @file   lvsf_comp.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "stdio.h"
#include "lv_ext_resource_manager.h"
#if defined SOLUTION_WATCH
    #include "app_mem.h"
#endif

#define lvsf_get_font(size) LV_EXT_FONT_GET(size)

int lvsf_font_height(int font_size)
{
    return lvsf_get_font(font_size)->line_height;
}

int lvsf_font_width(int font_size, char c)
{
    char text[2];

    if (c == '\0')
        c = 'A';
    text[0] = c;
    text[1] = '\0';
    return lv_font_get_glyph_width(lvsf_get_font(font_size), *text, *(text + 1));
}

lv_obj_t *lvsf_text_create2(lv_obj_t *parent, char *text, int x, int y, int angle, lv_color_t color, int size)
{
    uint8_t *cbuf;
    lv_img_dsc_t *img;
    lv_obj_t *canvas;
    int font_width, font_height;
    lv_draw_label_dsc_t dsc;

    canvas = lv_canvas_create(parent);
    lv_draw_label_dsc_init(&dsc);
    dsc.flag = LV_TEXT_FLAG_EXPAND;
#ifdef DISABLE_LVGL_V9
    dsc.flag |= LV_TEXT_FLAG_RECOLOR;
#endif
    dsc.font = LV_EXT_FONT_GET(size);
    dsc.color = color;
    font_width = lv_font_get_glyph_width(dsc.font, *text, *(text + 1));
    font_width *= (int)strlen(text);
    font_height = lv_font_get_line_height(dsc.font);
#if defined SOLUTION_WATCH && (IMAGE_CACHE_IN_PSRAM_SIZE > 0 || IMAGE_CACHE_IN_SRAM_SIZE > 0)
    cbuf = (uint8_t *)app_canvas_mem_alloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(font_width, font_height));
#else
    cbuf = (uint8_t *)lv_mem_alloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(font_width, font_height));
#endif
    if (!cbuf)
    {
        rt_kprintf("lvsf_text_create2: size %d font_width %d font_height %d \n", size, font_width, font_height);
        RT_ASSERT(cbuf);
    }
    memset(cbuf, 0, LV_CANVAS_BUF_SIZE_TRUE_COLOR(font_width, font_height));
    lv_canvas_set_buffer(canvas, cbuf, font_width, font_height, LV_IMG_CF_TRUE_COLOR);
#ifdef DISABLE_LVGL_V9
    lv_canvas_draw_text(canvas, 0, 0, font_width, &dsc, text);
#endif
    img = lv_canvas_get_img(canvas);
    lvsf_canvas_rotate(parent, img, angle * 10, LV_IMG_ZOOM_NONE, x, y, 0, 0);
#if defined SOLUTION_WATCH && (IMAGE_CACHE_IN_PSRAM_SIZE > 0 || IMAGE_CACHE_IN_SRAM_SIZE > 0)
    app_canvas_mem_free(cbuf);
#else
    lv_mem_free(cbuf);
#endif
    lv_obj_del(canvas);
    return parent;
}

static uint8_t hex_char_to_num(char hex)
{
    uint8_t result = 0;

    if (hex >= '0' && hex <= '9')
    {
        result = hex - '0';
    }
    else
    {
        if (hex >= 'a') hex -= 'a' - 'A'; /*Convert to upper case*/

        switch (hex)
        {
        case 'A':
            result = 10;
            break;
        case 'B':
            result = 11;
            break;
        case 'C':
            result = 12;
            break;
        case 'D':
            result = 13;
            break;
        case 'E':
            result = 14;
            break;
        case 'F':
            result = 15;
            break;
        default:
            result = 0;
            break;
        }
    }
    return result;
}

char *next_char(char *text, char *ch, lv_color_t *color)
{

    static bool recolor = false;
    if (*text == '#')
    {
        if (*(text + 1) == '#')
        {
            *ch = '#';
            text += 2;
        }
        else
        {
            int r, g, b;
            if (recolor)
            {
                recolor = false;
                text = next_char(text + 1, ch, color);
            }
            else
            {
                text++;
                recolor = true;
                if (strlen(text) > 8)
                {
                    r       = (hex_char_to_num(text[0]) << 4) + hex_char_to_num(text[1]);
                    g       = (hex_char_to_num(text[2]) << 4) + hex_char_to_num(text[3]);
                    b       = (hex_char_to_num(text[4]) << 4) + hex_char_to_num(text[5]);
                    *color = lv_color_make(r, g, b);
                    *ch = text[7];
                    text += 8;
                }
            }
        }
    }
    else
    {
        *ch = *text;
        if (*ch == '\0')
            recolor = false;
        else
            text++;
    }
    return text;
}


void lvsf_curve_draw_text(lv_obj_t *parent, char *text, int corner_startx, int corner_starty, int angle, int r, lv_color_t color, int size)
{
    int delta_angle = TEXT_ANGLE;       // Quad 3,4 , show text anti clock-wise
    if (angle < 180)
        delta_angle = -TEXT_ANGLE;     // Quad 1,2 , show text clock-wise

    if (text)
    {
        int startx, starty;
        uint8_t *cbuf;
        lv_img_dsc_t *img;
        lv_obj_t *canvas;
        int font_width, font_height;
        lv_draw_label_dsc_t dsc;

        canvas = lv_canvas_create(parent);
        lv_draw_label_dsc_init(&dsc);
        dsc.flag = LV_TEXT_FLAG_EXPAND;
#ifdef DISABLE_LVGL_V9
        dsc.flag |= LV_TEXT_FLAG_RECOLOR ;
#endif
        dsc.font = LV_EXT_FONT_GET(size);
        dsc.color = color;


        font_width = lv_font_get_glyph_width(dsc.font, '#', '#');
        font_height = lv_font_get_line_height(dsc.font);
#if defined SOLUTION_WATCH && (IMAGE_CACHE_IN_PSRAM_SIZE > 0 || IMAGE_CACHE_IN_SRAM_SIZE > 0)
        cbuf = (uint8_t *)app_canvas_mem_alloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR((font_width << 1), (font_height << 1))); //Enlarge mem, make sure lager than any character
#else
        cbuf = (uint8_t *)lv_mem_alloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR((font_width << 1), (font_height << 1))); //Enlarge mem, make sure lager than any character
#endif
        if (!cbuf)
        {
            rt_kprintf("lvsf_curve_text: size %d font_width %d font_height %d \n", size, font_width, font_height);
            RT_ASSERT(cbuf);
        }
        lv_canvas_set_buffer(canvas, cbuf, font_width, font_height, LV_IMG_CF_TRUE_COLOR);
        img = lv_canvas_get_img(canvas);

        while (*text)
        {
            char mytext[2];
            text = next_char(text, &mytext[0], &color);
            mytext[1] = '\0';

            if ((mytext[0] != ' ') && (mytext[0] != '\0'))
            {
                dsc.color = color;
                font_width = lv_font_get_glyph_width(dsc.font, mytext[0], mytext[1]);
                lv_canvas_set_buffer(canvas, cbuf, font_width, font_height, LV_IMG_CF_TRUE_COLOR);


                memset(cbuf, 0, LV_CANVAS_BUF_SIZE_TRUE_COLOR(font_width, font_height));
#ifdef DISABLE_LVGL_V9
                lv_canvas_draw_text(canvas, 0, 0, font_width, &dsc, mytext);
#endif
                if (angle < 180)
                {
                    startx = r - corner_startx + ((lv_trigo_cos(angle) * (2 * r + font_height)) >> (LV_TRIGO_SHIFT + 1));
                    starty = r - corner_starty - ((lv_trigo_sin(angle) * (2 * r + font_height)) >> (LV_TRIGO_SHIFT + 1));
                    lvsf_canvas_rotate(parent, img, (90 - angle) * 10, LV_IMG_ZOOM_NONE, startx, starty, 0, 0);
                }
                else
                {
                    startx = r - corner_startx + ((lv_trigo_cos(angle) * (2 * r - font_height)) >> (LV_TRIGO_SHIFT + 1));
                    starty = r - corner_starty - ((lv_trigo_sin(angle) * (2 * r - font_height)) >> (LV_TRIGO_SHIFT + 1));
                    lvsf_canvas_rotate(parent, img, (270 - angle) * 10, LV_IMG_ZOOM_NONE, startx, starty, 0, 0);
                }
            }
            angle += delta_angle;
        }

#if defined SOLUTION_WATCH && (IMAGE_CACHE_IN_PSRAM_SIZE > 0 || IMAGE_CACHE_IN_SRAM_SIZE > 0)
        app_canvas_mem_free(cbuf);
#else
        lv_mem_free(cbuf);
#endif
        lv_obj_del(canvas);
    }

}

/*
    Text should show in either upper half or lower half, could not accross 0/180 degree.
    r - text center point to pivot
    anlge - first character angle
*/
lv_obj_t *lvsf_curve_text(lv_obj_t *parent, char *text, int pivot_x, int pivot_y, int angle, int r, lv_color_t color, int size)
{
    int corner_startx, corner_starty; //top-left coordinate of canvas
    int font_height = lvsf_font_height(size);

    int font_width = lvsf_font_width(size, *text);
    int text_angles;
    int end_angle;
    text_angles = TEXT_ANGLE * (int)strlen(text);
    if (angle < 90)
    {
        end_angle = angle - text_angles;
        corner_startx = ((lv_trigo_cos(angle) * (2 * r - font_height)) >> (LV_TRIGO_SHIFT + 1)) + r;
        corner_starty = r - ((lv_trigo_sin(angle) * (2 * r + font_height)) >> (LV_TRIGO_SHIFT + 1));
    }
    else if (angle < 180)
    {
        end_angle = angle - text_angles;
        corner_startx = ((lv_trigo_cos(angle) * (2 * r + font_height)) >> (LV_TRIGO_SHIFT + 1)) + r;
        corner_starty = (end_angle < 90) ?
                        - (font_height >> 1) :
                        r - ((lv_trigo_sin(end_angle) * (2 * r + font_height) + lv_trigo_cos(end_angle) * (2 * font_width)) >> (LV_TRIGO_SHIFT + 1));
    }
    else if (angle < 270)
    {
        end_angle = angle + text_angles;
        corner_startx = ((lv_trigo_cos(angle) * (2 * r + font_height)) >> (LV_TRIGO_SHIFT + 1)) + r;
        corner_starty = (270 - angle > end_angle - 270) ?
                        r - ((lv_trigo_sin(angle) * (2 * r - font_height)) >> (LV_TRIGO_SHIFT + 1)) :
                        r - ((lv_trigo_sin(end_angle) * (2 * r - font_height)/* + lv_trigo_cos(end_angle) * (2 * font_width)*/) >> (LV_TRIGO_SHIFT + 1));
    }
    else
    {
        end_angle = angle + text_angles;
        corner_startx = ((lv_trigo_cos(angle) * (2 * r - font_height)) >> (LV_TRIGO_SHIFT + 1)) + r;
        corner_starty = r - ((lv_trigo_sin(end_angle) * (2 * r - font_height) /*+ lv_trigo_cos(end_angle) * (2 * font_width)*/) >> (LV_TRIGO_SHIFT + 1));
    }
    {
        lv_coord_t left = pivot_x + corner_startx - r;
        lv_coord_t top = pivot_y + corner_starty - r;
        lv_obj_set_pos(parent, left, top);
    }

    lvsf_curve_draw_text(parent, text, corner_startx, corner_starty, angle, r, color, size);

    return parent;
}


void lvsf_canvas_rotate(lv_obj_t *canvas, lv_img_dsc_t *img, int16_t angle, int32_t scale, lv_coord_t offset_x, lv_coord_t offset_y,
                        int32_t pivot_x, int32_t pivot_y)
{

#if 1//LV_USE_GPU
    lv_draw_img_dsc_t draw_dsc;

    lv_draw_img_dsc_init(&draw_dsc);
    draw_dsc.pivot.x = pivot_x;
    draw_dsc.pivot.y = pivot_y;
#ifdef DISABLE_LVGL_V9
    draw_dsc.zoom = scale;
    draw_dsc.angle = angle;
    lv_canvas_draw_img(canvas, offset_x, offset_y, img, &draw_dsc);
#endif
#else
    lv_canvas_transform(canvas, img, angle, 256, offset_x, offset_y, pivot_x, pivot_y, true);
#endif
    return;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
