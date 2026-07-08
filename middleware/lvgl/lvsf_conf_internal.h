/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LVSF_CONF_INTERNAL_H
#define LVSF_CONF_INTERNAL_H

#include "lvgl.h"

/*********************** SDK components *************************/

#ifndef LVSF_USE_ANALOGCLK
    #ifdef CONFIG_LVSF_USE_ANALOGCLK
        #define LVSF_USE_ANALOGCLK CONFIG_LVSF_USE_ANALOGCLK
    #else
        #define LVSF_USE_ANALOGCLK 1
    #endif
#endif

#ifndef LVSF_USE_BARCODE
    #ifdef CONFIG_LVSF_USE_BARCODE
        #define LVSF_USE_BARCODE CONFIG_LVSF_USE_BARCODE
    #else
        #define LVSF_USE_BARCODE 1
    #endif
#endif

#ifndef LVSF_USE_COMPOSITE
    #ifdef CONFIG_LVSF_USE_COMPOSITE
        #define LVSF_USE_COMPOSITE CONFIG_LVSF_USE_COMPOSITE
    #else
        #define LVSF_USE_COMPOSITE 1
    #endif
#endif

#ifndef LVSF_USE_CORNER
    #ifdef CONFIG_LVSF_USE_CORNER
        #define LVSF_USE_CORNER CONFIG_LVSF_USE_CORNER
    #else
        #define LVSF_USE_CORNER 1
    #endif
#endif

#ifndef LVSF_USE_IDXIMG
    #ifdef CONFIG_LVSF_USE_IDXIMG
        #define LVSF_USE_IDXIMG CONFIG_LVSF_USE_IDXIMG
    #else
        #define LVSF_USE_IDXIMG 1
    #endif
#endif

#ifndef LVSF_USE_HEADER
    #ifdef CONFIG_LVSF_USE_HEADER
        #define LVSF_USE_HEADER CONFIG_LVSF_USE_HEADER
    #else
        #define LVSF_USE_HEADER 1
    #endif
#endif

#ifndef LVSF_USE_CURVE
    #ifdef CONFIG_LVSF_USE_CURVE
        #define LVSF_USE_CURVE CONFIG_LVSF_USE_CURVE
    #else
        #define LVSF_USE_CURVE 1
    #endif
#endif

/*********************** Solution components *************************/

#ifndef LVSF_USE_ANIM
    #ifdef CONFIG_LVSF_USE_ANIM
        #define LVSF_USE_ANIM CONFIG_LVSF_USE_ANIM
    #else
        #define LVSF_USE_ANIM 1
    #endif
#endif

#ifndef LVSF_USE_BATTERY
    #ifdef CONFIG_LVSF_USE_BATTERY
        #define LVSF_USE_BATTERY CONFIG_LVSF_USE_BATTERY
    #else
        #define LVSF_USE_BATTERY 1
    #endif
#endif

#ifndef LVSF_USE_FUNCBTN
    #ifdef CONFIG_LVSF_USE_FUNCBTN
        #define LVSF_USE_FUNCBTN CONFIG_LVSF_USE_FUNCBTN
    #else
        #define LVSF_USE_FUNCBTN 1
    #endif
#endif

#ifndef LVSF_USING_FOOTER
    #ifdef CONFIG_LVSF_USING_FOOTER
        #define LVSF_USING_FOOTER CONFIG_LVSF_USING_FOOTER
    #else
        #define LVSF_USING_FOOTER 1
    #endif
#endif

#ifndef LVSF_USING_GRIDVIEW
    #ifdef CONFIG_LVSF_USING_GRIDVIEW
        #define LVSF_USING_GRIDVIEW CONFIG_LVSF_USING_GRIDVIEW
    #else
        #define LVSF_USING_GRIDVIEW 1
    #endif
#endif

#ifndef LVSF_USING_INFOBTN
    #ifdef CONFIG_LVSF_USING_INFOBTN
        #define LVSF_USING_INFOBTN CONFIG_LVSF_USING_INFOBTN
    #else
        #define LVSF_USING_INFOBTN 1
    #endif
#endif

#ifndef LVSF_USING_LIST
    #ifdef CONFIG_LVSF_USING_LIST
        #define LVSF_USING_LIST CONFIG_LVSF_USING_LIST
    #else
        #define LVSF_USING_LIST 1
    #endif
#endif

#ifndef LVSF_USING_PAGE
    #ifdef CONFIG_LVSF_USING_PAGE
        #define LVSF_USING_PAGE CONFIG_LVSF_USING_PAGE
    #else
        #define LVSF_USING_PAGE 1
    #endif
#endif

#ifndef LVSF_USING_RECT
    #ifdef CONFIG_LVSF_USING_RECT
        #define LVSF_USING_RECT CONFIG_LVSF_USING_RECT
    #else
        #define LVSF_USING_RECT 1
    #endif
#endif

#ifndef LVSF_USE_TITLE
    #ifdef CONFIG_LVSF_USE_TITLE
        #define LVSF_USE_TITLE CONFIG_LVSF_USE_TITLE
    #else
        #define LVSF_USE_TITLE 1
    #endif
#endif

#ifndef LVSF_USE_LIST
    #ifdef CONFIG_LVSF_USE_LIST
        #define LVSF_USE_LIST CONFIG_LVSF_USE_LIST
    #else
        #define LVSF_USE_LIST 1
    #endif
#endif


#ifndef _MAINMENU_LIST
    #define _MAINMENU_LIST 1
#endif


#ifndef LVSF_USE_POPUP
    #ifdef CONFIG_LVSF_USE_POPUP
        #define LVSF_USE_POPUP CONFIG_LVSF_USE_POPUP
    #else
        #define LVSF_USE_POPUP 1
    #endif
#endif

#ifndef LVSF_USE_QRCODE
    #ifdef CONFIG_LVSF_USE_QRCODE
        #define LVSF_USE_QRCODE CONFIG_LVSF_USE_QRCODE
    #else
        #define LVSF_USE_QRCODE 1
    #endif
#endif

#define LVSF_IMG_CF_JPEG    (LV_IMG_CF_USER_ENCODED_0)

#ifndef LVSF_USING_SJPG
    #ifdef CONFIG_LVSF_USING_SJPG
        #define LVSF_USING_SJPG CONFIG_LVSF_USING_SJPG
    #else
        #define LVSF_USING_SJPG 1
    #endif
#endif

#ifndef LVSF_USING_ENCODER
    #ifdef CONFIG_LVSF_USING_ENCODER
        #define LVSF_USING_ENCODER CONFIG_LVSF_USING_ENCODER
    #else
        #define LVSF_USING_ENCODER 1
    #endif
#endif

#ifndef LVSF_USE_MULTLIST
    #ifdef CONFIG_LVSF_USE_MULTLIST
        #define LVSF_USE_MULTLIST CONFIG_LVSF_USE_MULTLIST
    #else
        #define LVSF_USE_MULTLIST 1
    #endif
#endif

#ifndef LVSF_USE_MULTOBJ
    #ifdef CONFIG_LVSF_USE_MULTOBJ
        #define LVSF_USE_MULTOBJ CONFIG_LVSF_USE_MULTOBJ
    #else
        #define LVSF_USE_MULTOBJ 1
    #endif
#endif

#ifndef LVSF_USE_MULTSWIPE
    #ifdef CONFIG_LVSF_USE_MULTSWIPE
        #define LVSF_USE_MULTSWIPE CONFIG_LVSF_USE_MULTSWIPE
    #else
        #define LVSF_USE_MULTSWIPE 1
    #endif
#endif

#ifndef LVSF_USE_MULTANIM
    #ifdef CONFIG_LVSF_USE_MULTANIM
        #define LVSF_USE_MULTANIM CONFIG_LVSF_USE_MULTANIM
    #else
        #define LVSF_USE_MULTANIM 1
    #endif
#endif

#ifndef LVSF_USING_SWITCHANIM
    #ifdef CONFIG_LVSF_USING_SWITCHANIM
        #define LVSF_USING_SWITCHANIM 1
    #else
        #define LVSF_USING_SWITCHANIM 0
    #endif
#endif

#ifndef LVSF_USE_TXTIMG
    #ifdef CONFIG_LVSF_USE_TXTIMG
        #define LVSF_USE_TXTIMG CONFIG_LVSF_USE_TXTIMG
    #else
        #define LVSF_USE_TXTIMG 1
    #endif
#endif

/*********** Migrated gui_widgets (2026-06) — expose their guarded headers ***********/
/* These widgets are compiled into the prebuilt gui_widgets lib; defining their
 * guard macros here makes the public header APIs visible to apps/examples. */

#ifndef LVSF_USE_OBJ_EXT
    #ifdef CONFIG_LVSF_USE_OBJ_EXT
        #define LVSF_USE_OBJ_EXT CONFIG_LVSF_USE_OBJ_EXT
    #else
        #define LVSF_USE_OBJ_EXT 1
    #endif
#endif

#ifndef LVSF_USE_BASECHART
    #ifdef CONFIG_LVSF_USE_BASECHART
        #define LVSF_USE_BASECHART CONFIG_LVSF_USE_BASECHART
    #else
        #define LVSF_USE_BASECHART 1
    #endif
#endif

#ifndef LVSF_USE_SECTOR
    #ifdef CONFIG_LVSF_USE_SECTOR
        #define LVSF_USE_SECTOR CONFIG_LVSF_USE_SECTOR
    #else
        #define LVSF_USE_SECTOR 1
    #endif
#endif

#ifndef LVSF_USE_SELECT
    #ifdef CONFIG_LVSF_USE_SELECT
        #define LVSF_USE_SELECT CONFIG_LVSF_USE_SELECT
    #else
        #define LVSF_USE_SELECT 1
    #endif
#endif

#ifndef LVSF_USE_IMGBAR
    #ifdef CONFIG_LVSF_USE_IMGBAR
        #define LVSF_USE_IMGBAR CONFIG_LVSF_USE_IMGBAR
    #else
        #define LVSF_USE_IMGBAR 1
    #endif
#endif

#ifndef LVSF_USE_BASEIMG
    #ifdef CONFIG_LVSF_USE_BASEIMG
        #define LVSF_USE_BASEIMG CONFIG_LVSF_USE_BASEIMG
    #else
        #define LVSF_USE_BASEIMG 1
    #endif
#endif

#ifndef LVSF_USE_IMGARRAY
    #ifdef CONFIG_LVSF_USE_IMGARRAY
        #define LVSF_USE_IMGARRAY CONFIG_LVSF_USE_IMGARRAY
    #else
        #define LVSF_USE_IMGARRAY 1
    #endif
#endif

#ifndef LVSF_USE_BASELABEL
    #ifdef CONFIG_LVSF_USE_BASELABEL
        #define LVSF_USE_BASELABEL CONFIG_LVSF_USE_BASELABEL
    #else
        #define LVSF_USE_BASELABEL 1
    #endif
#endif

#ifndef LVSF_USE_MULROLLER
    #ifdef CONFIG_LVSF_USE_MULROLLER
        #define LVSF_USE_MULROLLER CONFIG_LVSF_USE_MULROLLER
    #else
        #define LVSF_USE_MULROLLER 1
    #endif
#endif

#ifndef LVSF_USE_ARCIMG
    #ifdef CONFIG_LVSF_USE_ARCIMG
        #define LVSF_USE_ARCIMG CONFIG_LVSF_USE_ARCIMG
    #else
        #define LVSF_USE_ARCIMG 1
    #endif
#endif

#ifndef LVSF_USE_MULTEDGE
    #ifdef CONFIG_LVSF_USE_MULTEDGE
        #define LVSF_USE_MULTEDGE CONFIG_LVSF_USE_MULTEDGE
    #else
        #define LVSF_USE_MULTEDGE 1
    #endif
#endif

#ifndef LVSF_USE_MULTSLIDER
    #ifdef CONFIG_LVSF_USE_MULTSLIDER
        #define LVSF_USE_MULTSLIDER CONFIG_LVSF_USE_MULTSLIDER
    #else
        #define LVSF_USE_MULTSLIDER 1
    #endif
#endif

#ifndef LVSF_USE_SCROLLBAR
    #ifdef CONFIG_LVSF_USE_SCROLLBAR
        #define LVSF_USE_SCROLLBAR CONFIG_LVSF_USE_SCROLLBAR
    #else
        #define LVSF_USE_SCROLLBAR 1
    #endif
#endif

#ifndef LVSF_USING_KEYMANAGE
    #ifdef CONFIG_LVSF_USING_KEYMANAGE
        #define LVSF_USING_KEYMANAGE CONFIG_LVSF_USING_KEYMANAGE
    #else
        #define LVSF_USING_KEYMANAGE 1
    #endif
#endif

#ifndef LVSF_USING_TIMELINE
    #ifdef CONFIG_LVSF_USING_TIMELINE
        #define LVSF_USING_TIMELINE CONFIG_LVSF_USING_TIMELINE
    #else
        #define LVSF_USING_TIMELINE 1
    #endif
#endif


#endif /*LVSF_CONF_INTERNAL_H*/
