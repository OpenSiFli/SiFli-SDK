/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
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
#include "lv_ex_data.h"
#include "lvsf_input.h"
#include "lvsf_ezipa.h"

#include "lv_ext_resource_manager.h"


#if !defined(PYCPARSER)&&!defined(PY_GEN)
#include "section.h"
#endif

#include "lvsf_perf.h"

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
