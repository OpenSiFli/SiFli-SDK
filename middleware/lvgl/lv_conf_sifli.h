/**
  ******************************************************************************
  * @file   lv_conf_sifli.h
  * @author Sifli software development team
  * @brief  SiFli configuration for both LVGL V8 & V9
  * @attention
  ******************************************************************************
*/
/*
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

#ifndef LV_CONF_SIFLI_H
#define LV_CONF_SIFLI_H

#define LV_USE_DEV_VERSION

#if 1//def __RTTHREAD__

    #define LV_RTTHREAD_INCLUDE <rtthread.h>
    #include LV_RTTHREAD_INCLUDE

    #define CONFIG_LV_CONF_SKIP

    #define CONFIG_SILFI_PERF_MONITOR  0 //1- lv_refr task will include empty frame(which has no invalid area).

    #ifdef LV_RES_AUTO
        #undef LV_HOR_RES_MAX
        #undef LV_VER_RES_MAX
        #ifdef LCD_HOR_RES_MAX
            #define LV_HOR_RES_MAX LCD_HOR_RES_MAX
            #define LV_VER_RES_MAX LCD_VER_RES_MAX
        #else
            #define LV_HOR_RES_MAX 368
            #define LV_VER_RES_MAX 448
        #endif
    #endif

    #ifdef DISABLE_LVGL_V9
        #define LV_USE_LVSF
    #endif /* DISABLE_LVGL_V9 */
    /*=========================
    MEMORY SETTINGS
    *=========================*/

    #ifdef RT_USING_HEAP
        #define LV_MEM_CUSTOM 1
        #define LV_MEM_CUSTOM_INCLUDE LV_RTTHREAD_INCLUDE
        #define LV_MEM_CUSTOM_ALLOC   rt_malloc
        #define LV_MEM_CUSTOM_FREE    rt_free
        #define LV_MEM_CUSTOM_REALLOC rt_realloc
    #endif


    #ifdef LV_FRAME_BUF_ALIGNED_FOR_VGLITE
        #if (16 == LV_COLOR_DEPTH)
            #define FB_ALIGN_BYTE   (64)
            #define FB_ALIGNED_HOR_RES  RT_ALIGN(LV_HOR_RES_MAX * 2 , FB_ALIGN_BYTE) / 2
        #elif (24 == LV_COLOR_DEPTH)
            #define FB_ALIGN_BYTE   (48)
            #define FB_ALIGNED_HOR_RES  RT_ALIGN(LV_HOR_RES_MAX * 3 , FB_ALIGN_BYTE) / 3
        #elif (32 == LV_COLOR_DEPTH)
            #define FB_ALIGN_BYTE   (64)
            #define FB_ALIGNED_HOR_RES  RT_ALIGN(LV_HOR_RES_MAX * 4 , FB_ALIGN_BYTE) / 4
        #else
            #define FB_ALIGNED_HOR_RES  LV_HOR_RES_MAX
        #endif  /* 16 ==LV_COLOR_DEPTH */
    #else
        #define FB_ALIGN_BYTE       (1)
        #define FB_ALIGNED_HOR_RES  (LV_HOR_RES_MAX)
    #endif /* LV_FRAME_BUF_ALIGNED_FOR_VGLITE */

    /*====================
    HAL SETTINGS
    *====================*/

    #define LV_TICK_CUSTOM 1
    #define LV_TICK_CUSTOM_INCLUDE LV_RTTHREAD_INCLUDE
    #define LV_TICK_CUSTOM_SYS_TIME_EXPR (rt_tick_get_millisecond())    /*Expression evaluating to current system time in ms*/

    #ifndef PKG_LVGL_DISP_REFR_PERIOD
        #define PKG_LVGL_DISP_REFR_PERIOD 33
    #endif /* PKG_LVGL_DISP_REFR_PERIOD */


    #ifndef LV_DISP_DEF_REFR_PERIOD
        #define LV_DISP_DEF_REFR_PERIOD   PKG_LVGL_DISP_REFR_PERIOD
    #endif

    #if (defined(SF32LB55X)||defined(SF32LB56X)||defined(SF32LB58X))
        #define FT_BPP    4
    #else
        #define FT_BPP    2
    #endif

    #ifndef _MSC_VER
        #define LV_USE_GPU_SIFLI_EPIC 1

        #ifdef LV_DPI_DEF
            #undef LV_DPI_DEF /* Replace LVGL DPI configuration with LCD_DPI */
        #endif /* LV_DPI_DEF */
        #define LV_DPI_DEF LCD_DPI

        #define COMPATIBLE_WITH_SIFLI_EPIC_FILL  /*Compatible lvgl fill with sifli EPIC fill*/

        #if ((!defined(SF32LB55X) && (4 == FT_BPP)) || (!(defined(SF32LB55X)||defined(SF32LB56X)||defined(SF32LB58X)) && (2 == FT_BPP))) && LV_USING_FREETYPE_ENGINE
            #define COMPATIBLE_WITH_SIFLI_EPIC_Ax  1 /*Attach dummy pixels for every row to align to 1 byte for A2/A4 color format*/
        #else
            #define COMPATIBLE_WITH_SIFLI_EPIC_Ax  0 /*55x not support Ax*/
        #endif /* SF32LB55X */
    #else
        #define LV_USE_GPU_SIFLI_EPIC 0
        #ifndef LV_DPI_DEF
            #define LV_DPI_DEF 315
        #endif
        #define COMPATIBLE_WITH_SIFLI_EPIC_Ax  0
    #endif /* !_MSC_VER */

    #define LV_USE_GPU       LV_USE_GPU_SIFLI_EPIC
    #define LV_USE_DRAW_EPIC LV_USE_GPU_SIFLI_EPIC
    /*=======================
    * FEATURE CONFIGURATION
    *=======================*/

    /*-------------
    * Asserts
    *-----------*/
    #undef LV_ASSERT_HANDLER_INCLUDE
    #define LV_ASSERT_HANDLER_INCLUDE LV_RTTHREAD_INCLUDE
    #define LV_ASSERT_HANDLER RT_ASSERT(0);

    /*-------------
    * Others
    *-----------*/

    #define LV_SPRINTF_CUSTOM 1
    #define LV_SPRINTF_INCLUDE LV_RTTHREAD_INCLUDE
    #ifndef DISABLE_LVGL_V8
        #define lv_snprintf  rt_snprintf
        #define lv_vsnprintf rt_vsnprintf
    #endif
    #define LV_SPRINTF_USE_FLOAT 0

    /*=====================
    *  COMPILER SETTINGS
    *====================*/

    #ifdef ARCH_CPU_BIG_ENDIAN
        #define LV_BIG_ENDIAN_SYSTEM 1
    #else
        #define LV_BIG_ENDIAN_SYSTEM 0
    #endif

    #define LV_ATTRIBUTE_MEM_ALIGN ALIGN(4)

    /*==================
    * EXAMPLES
    *==================*/

    #ifdef PKG_LVGL_USING_EXAMPLES
        #define LV_BUILD_EXAMPLES 1
    #endif


    #ifdef LV_USING_FREETYPE_ENGINE
        #define LV_THEME_DEFAULT_FONT_SMALL        LV_EXT_FONT_GET(FONT_SMALL)
        #define LV_THEME_DEFAULT_FONT_NORMAL       LV_EXT_FONT_GET(FONT_NORMAL)
        #define LV_THEME_DEFAULT_FONT_SUBTITLE     LV_EXT_FONT_GET(FONT_SUBTITLE)
        #define LV_THEME_DEFAULT_FONT_TITLE        LV_EXT_FONT_GET(FONT_TITLE)
        #define LV_THEME_DEFAULT_FONT_BIGL         LV_EXT_FONT_GET(FONT_BIGL)
    #else
        #if LV_HOR_RES_MAX > 350
            #define LV_THEME_DEFAULT_FONT_SMALL         &lv_font_montserrat_16
            #define LV_THEME_DEFAULT_FONT_NORMAL        &lv_font_montserrat_20
            #define LV_THEME_DEFAULT_FONT_SUBTITLE      &lv_font_montserrat_24
            #define LV_THEME_DEFAULT_FONT_TITLE         &lv_font_montserrat_28
            #define LV_THEME_DEFAULT_FONT_BIGL          &lv_font_montserrat_36
        #else
            #define LV_THEME_DEFAULT_FONT_SMALL         &lv_font_montserrat_12
            #define LV_THEME_DEFAULT_FONT_NORMAL        &lv_font_montserrat_16
            #define LV_THEME_DEFAULT_FONT_SUBTITLE      &lv_font_montserrat_20
            #define LV_THEME_DEFAULT_FONT_TITLE         &lv_font_montserrat_24
            #define LV_THEME_DEFAULT_FONT_BIGL          &lv_font_montserrat_28
        #endif
    #endif

    /*--END OF LV_CONF_SIFLI_H--*/

#endif /*__RTTHREAD__*/



#ifdef DISABLE_LVGL_V8
    #ifdef RT_USING_HEAP
        #define LV_USE_STDLIB_MALLOC    LV_STDLIB_RTTHREAD
    #endif

    //Use assembled memcpy/memset
    #undef LV_USE_STDLIB_STRING
    #define LV_USE_STDLIB_STRING    LV_STDLIB_CLIB

    #undef LV_USE_STDLIB_SPRINTF
    #define LV_USE_STDLIB_SPRINTF   LV_STDLIB_CLIB

    #ifndef __FILENAME__
        #define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
    #endif
    /*-------------
    * Asserts
    *-----------*/

    #define LV_ASSERT_HANDLER_INCLUDE LV_RTTHREAD_INCLUDE
    #define LV_ASSERT_HANDLER RT_ASSERT(0);

    /*=====================
    *  COMPILER SETTINGS
    *====================*/

    #ifdef ARCH_CPU_BIG_ENDIAN
        #define LV_BIG_ENDIAN_SYSTEM 1
    #else
        #define LV_BIG_ENDIAN_SYSTEM 0
    #endif




    /* Compatible for V8/V9*/
    #define always_zero reserved_2


    #define LV_IMG_CF_TRUE_COLOR_ALPHA LV_COLOR_FORMAT_NATIVE_WITH_ALPHA
    #define LV_IMG_CF_TRUE_COLOR  LV_COLOR_FORMAT_NATIVE


    #define LV_IMG_CF_ALPHA_8BIT  LV_COLOR_FORMAT_A8
    #define LV_IMG_CF_INDEXED_1BIT LV_COLOR_FORMAT_I1

    #define LV_IMG_CF_RAW        LV_COLOR_FORMAT_RAW
    #define LV_IMG_CF_RAW_ALPHA  LV_COLOR_FORMAT_RAW_ALPHA

    #define LV_IMAGE_FLAGS_EZIP  LV_IMAGE_FLAGS_USER1

    #ifndef LV_USE_BIN
        #define LV_USE_BTN LV_USE_BUTTON
    #endif
    #ifndef LV_USE_IMG
        #define LV_USE_IMG LV_USE_IMAGE
    #endif
    typedef unsigned char lv_img_cf_t;
    #define LV_COLOR_CYAN lv_color_make(0x00, 0xFF, 0xFF)
    #define lv_img_cache_invalidate_src(img) lv_image_cache_drop((const void * )img)

    //#ifdef LV_USE_LVSF


    #if 1//(0 == LVGL_VERSION_MINOR) && (0== LVGL_VERSION_PATCH) //v9.0.0
        //#define lv_btnmatrix_ctrl_t lv_buttonmatrix_ctrl_t
        #define lv_btnmatrix_get_btn_text lv_buttonmatrix_get_button_text
        #define LV_COLOR_WHITE lv_color_make(0xFF, 0xFF, 0xFF)
        #define LV_COLOR_SILVER lv_color_make(0xC0, 0xC0, 0xC0)
        #define LV_COLOR_GRAY lv_color_make(0x80, 0x80, 0x80)
        #define LV_COLOR_BLACK lv_color_make(0x00, 0x00, 0x00)
        #define LV_COLOR_RED lv_color_make(0xFF, 0x00, 0x00)
        #define LV_COLOR_MAROON lv_color_make(0x80, 0x00, 0x00)
        #define LV_COLOR_YELLOW lv_color_make(0xFF, 0xFF, 0x00)
        #define LV_COLOR_OLIVE lv_color_make(0x80, 0x80, 0x00)
        #define LV_COLOR_LIME lv_color_make(0x00, 0xFF, 0x00)
        #define LV_COLOR_GREEN lv_color_make(0x00, 0x80, 0x00)
        #define LV_COLOR_CYAN lv_color_make(0x00, 0xFF, 0xFF)
        #define LV_COLOR_AQUA LV_COLOR_CYAN
        #define LV_COLOR_TEAL lv_color_make(0x00, 0x80, 0x80)
        #define LV_COLOR_BLUE lv_color_make(0x00, 0x00, 0xFF)
        #define LV_COLOR_NAVY lv_color_make(0x00, 0x00, 0x80)
        #define LV_COLOR_MAGENTA lv_color_make(0xFF, 0x00, 0xFF)
        #define LV_COLOR_PURPLE lv_color_make(0x80, 0x00, 0x80)
        #define LV_COLOR_ORANGE lv_color_make(0xFF, 0xA5, 0x00)
        #define LV_IMG_ZOOM_NONE 256
        #define lv_mem_alloc lv_malloc
        #define lv_mem_free lv_free
        #define LV_CANVAS_BUF_SIZE_TRUE_COLOR(w,h) LV_CANVAS_BUF_SIZE(w,h,LV_COLOR_DEPTH,1)
        #define lv_canvas_get_img lv_canvas_get_image
        #define lv_draw_img_dsc_t lv_draw_image_dsc_t
        #define lv_draw_img_dsc_init lv_draw_image_dsc_init
        #define lv_img_t lv_image_t
        #define lv_img_class lv_image_class
        #define lv_img_decoder_t lv_image_decoder_t
        #define lv_img_header_t lv_image_header_t
        #define lv_img_src_t lv_image_src_t
        #define lv_img_src_get_type lv_image_src_get_type

        #define lv_img_decoder_dsc_t lv_image_decoder_dsc_t
        #define lv_img_decoder_get_info lv_image_decoder_get_info
        #define lv_img_decoder_create lv_image_decoder_create
        #define lv_img_decoder_set_info_cb lv_image_decoder_set_info_cb
        #define lv_img_decoder_set_open_cb  lv_image_decoder_set_open_cb
        #define lv_img_decoder_set_read_line_cb  lv_image_decoder_set_read_line_cb
        #define lv_img_decoder_set_close_cb lv_image_decoder_set_close_cb
        #define LV_IMG_SRC_FILE LV_IMAGE_SRC_FILE
        #define LV_INDEV_DEF_READ_PERIOD LV_DEF_REFR_PERIOD
        #define lv_list_btn_class lv_list_button_class
        #define LV_DISP_ROT_NONE LV_DISPLAY_ROTATION_0
        #define LV_DISP_ROT_180 LV_DISPLAY_ROTATION_180
        #define lv_scr_load lv_screen_load
        #define lv_btn_class lv_button_class
        #define lv_btnmatrix_class lv_buttonmatrix_class
        #define lv_disp_drv_t lv_display_t
        #define lv_disp_drv_init lv_display_init
        #define lv_disp_dpx lv_display_dpx
        #define LV_STYLE_PROP_ALL LV_STYLE_PROP_FLAG_ALL
        #define LV_IMG_PX_SIZE_ALPHA_BYTE _LV_COLOR_NATIVE_WITH_ALPHA_SIZE
        #define lv_style_set_bg_img_src lv_style_set_bg_image_src
        #define lv_label_set_text_sel_start lv_label_set_text_selection_start
        #define lv_label_set_text_sel_end lv_label_set_text_selection_end
        #define lv_image_get_size_mode(obj) 0
        #define lv_image_set_size_mode(obj,mode)
        #define lv_label_get_recolor(obj) 1
        #define lv_label_set_recolor(obj,en)
        #define lv_indev_drv_t lv_indev_t
    #endif /* 0 */

    #if (2 == LVGL_VERSION_MINOR) && (0== LVGL_VERSION_PATCH) //v9.2.0
        #define lv_img_class lv_image_class
        #define lv_btn_class lv_button_class
        #define lv_btnmatrix_class lv_buttonmatrix_class
        #define lv_list_btn_class lv_list_button_class

        #define lv_theme_get_font_bigl lv_theme_get_font_large
        #define lv_theme_get_font_title lv_theme_get_font_normal
        #define lv_theme_get_font_subtitle lv_theme_get_font_small

        #define lv_disp_drv_t lv_display_t
        #define lv_img_header_t lv_image_header_t
    #endif



    //#endif /* LV_USE_LVSF */

    #if LV_COLOR_DEPTH == 1
        #define LV_COLOR_SIZE 8
    #elif LV_COLOR_DEPTH == 8
        #define LV_COLOR_SIZE 8
    #elif LV_COLOR_DEPTH == 16
        #define LV_COLOR_SIZE 16
    #elif LV_COLOR_DEPTH == 24
        #define LV_COLOR_SIZE 24
    #elif LV_COLOR_DEPTH == 32
        #define LV_COLOR_SIZE 32
    #else
        #error "Invalid LV_COLOR_DEPTH in lv_conf.h! Set it to 1, 8, 16 or 32!"
    #endif

#endif

#endif /*LV_CONF_SIFLI_H*/
