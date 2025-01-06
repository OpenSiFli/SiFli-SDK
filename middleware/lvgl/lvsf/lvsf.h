#ifndef LVSF_H
#define LVSF_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "rtconfig.h"
#include <stdint.h>
#include "lvgl.h"
#include "lvsf_conf_internal.h"
//#include "lv_ex_conf.h"
#include "lvsf_utils.h"
#include "lvsf_comp.h"
#include "lv_ex_data.h"
#include "lvsf_header.h"
#include "lvsf_popup.h"
#include "lvsf_input.h"
#include "lvsf_gesture.h"
#include "lvsf_composite.h"
#include "lvsf_barcode.h"
#include "lvsf_analogclk.h"
#include "lvsf_idximg.h"
#include "lvsf_curvetext.h"
#include "lvsf_ezipa.h"
#include "lvsf_rlottie.h"
#include "lvsf_emoji.h"
#include "lvsf_theme_1.h"

#if !defined(PYCPARSER)&&!defined(PY_GEN)
#include "section.h"
#include "lvsf_font.h"
#ifndef LV_USING_FREETYPE_ENGINE
#include "lv_freetype.h"
#endif
#endif

#include "lvsf_perf.h"

// For script generation as lv obj member.
void lv_obj_set_local_font(lv_obj_t *obj, uint16_t size, lv_color_t color);
void lv_obj_set_page_glue(lv_obj_t *obj, bool glue);
lv_coord_t lv_get_ver_max();
lv_coord_t lv_get_hor_max();

#include "gui_app_fwk2.h"

/*********************
 *      DEFINES
 *********************/

#ifdef PKG_USING_MICROPYTHON
#if defined(_MSC_VER)
#pragma section("LV_NAMED_IMG$f",read)

#define LV_NAMED_IMG_DECLARE(name)                                \
    extern const lv_img_dsc_t name; \
    SECTION("LV_NAMED_IMG$f") \
    RT_USED static const named_img_var_t __named_images##name =                     \
    {   #name,   \
        &name,    \
    }

#pragma comment(linker, "/merge:LV_NAMED_IMG=mytext")

#else
#define LV_NAMED_IMG_DECLARE(name)                                \
    extern const lv_img_dsc_t name; \
    RT_USED static const named_img_var_t __named_images##name                 \
    SECTION("LV_NAMED_IMGTAB") =                                                    \
    {                                                                          \
        #name, \
        (void*) &name,    \
    }
#endif

#endif

/**********************
 *      TYPEDEFS
 **********************/
typedef struct tag_named_img_var
{
    const char *name;
    void *var;
} named_img_var_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/


/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVSF_UTILS_H*/

