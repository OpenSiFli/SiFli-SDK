/**
  ******************************************************************************
  * @file   gui_app_fwk.h
  * @author Sifli software development team
  * @brief Sifli GUI application framework, provide basic functions for user to start/stop/pause/resume
  *  application. It also provide navigation among applications and transition of GUI.
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

#ifndef __GUI_APP_FWK_H__
#define __GUI_APP_FWK_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>

#include "lvgl.h"

#include "intent.h"

/**
 ****************************************************************************************
* @addtogroup gui_app_fwk GUI Application Framework
* @ingroup middleware
* @brief Sifli GUI application framework, provide basic functions for user to start/stop/pause/resume
*  application. It also provide navigation among applications and transition of GUI.
* @{
****************************************************************************************
*/


#define GUI_APP_NAME_MAX_LEN            16  //!< App display name max length
#define GUI_APP_ID_MAX_LEN              16  //!< An unique character id of an app (both bult-in app and dl app)
#define GUI_DL_APP_MAX_FILE_PATH_LEN    64  //!< Max file path length for dynamic loading applications
#if defined(_MSC_VER)
    #define MSC_APP_STRUCT_MAGIC_HEAD 0xAB
#endif


typedef int (*gui_app_entry_func_ptr_t)(intent_t i);
typedef int16_t gui_anim_value_t;
typedef lv_obj_t *gui_anim_obj_t;
typedef int16_t gui_coord_t;
typedef struct
{
    gui_coord_t x;
    gui_coord_t y;
} gui_point_t;


/**
   built-in app description
 */
typedef struct
{
#if defined(_MSC_VER)
    uint8_t  magic_flag;  //!< Maigc struct header for simulator
#endif

#ifdef LV_USING_EXT_RESOURCE_MANAGER
    uint32_t name;   //!< app display name string id
#else
    char name[GUI_APP_NAME_MAX_LEN];   //!< app display name
#endif
    const void *icon;                  //!< app display icon
    char id[GUI_APP_ID_MAX_LEN];       //!< an unique character id of an app (both bult-in app and dl app)
    gui_app_entry_func_ptr_t entry;    //!< app entry function
} builtin_app_desc_t;


/**
    Define a build-in GUI applications:
    name - name which will display on mainmenu
    icon - icon which will display on mainmenu too.
    id   - let other app find this app by id
    entry- app's entry function
*/
#if defined(_MSC_VER)
#define BUILTIN_APP_EXPORT(name, icon, id, entry)                                \
    SECTION("BuiltinAppTab$1") \
    RT_USED static const builtin_app_desc_t __builtinapp =                     \
    {                           \
        MSC_APP_STRUCT_MAGIC_HEAD,        \
        name,    \
        icon,    \
        id,    \
        (gui_app_entry_func_ptr_t)entry     \
    }
#else
#define BUILTIN_APP_EXPORT(name, icon, id, entry)                                \
    RT_USED static const builtin_app_desc_t __builtinapp                       \
    SECTION("BuiltinAppTab") =                                                    \
    {                                                                          \
        name,    \
        icon,    \
        id,    \
        (gui_app_entry_func_ptr_t)entry     \
    }
#endif

#include "gui_app_fwk2.h"


typedef void (*gui_page_msg_cb_t)(gui_app_msg_type_t msg, void *param);

/**
    @brief register an application's root page message handler
    @param[in] id Identification of application
    @param[in] handler The main subpage message handler.
    @param[in] usr_data User data of this app
    @retval RT_EOK if successful, otherwise return error number < 0.
 */
int gui_app_regist_msg_handler_ext(const char *id, gui_page_msg_cb_t handler, void *usr_data);
#define gui_app_regist_msg_handler(id,handler) gui_app_regist_msg_handler_ext(id,handler, NULL)

/**
    @brief Create a subgpage in current actived app, and present it.
    @param[in] pg_id Subpage id (should be unique in one app )
    @param[in] handler This subpage message handler
    @param[in] usr_data User data of this subpage
    @retval RT_EOK if successful, otherwise return error number < 0.
 */
int gui_app_create_page_ext(const char *pg_id, gui_page_msg_cb_t handler, void *usr_data);
#define gui_app_create_page(pg_id,pg_handler) gui_app_create_page_ext(pg_id, pg_handler, NULL)

/**
    @brief Create a subgpage for specified app, and present it.
    @param[in] app_id Identification of an application
    @param[in] pg_id Subpage id (should be unique in one app )
    @param[in] handler This subpage message handler
    @param[in] usr_data User data of this subpage
    @retval RT_EOK if successful, otherwise return error number < 0.
 */
int gui_app_create_page_for_app_ext(const char *app_id, const char *pg_id, gui_page_msg_cb_t handler, void *usr_data);
#define gui_app_create_page_for_app(app_id,pg_id,pg_handler) gui_app_create_page_for_app_ext(app_id,pg_id,pg_handler,NULL)

/************************ Application transfer animation *******************************************************/



typedef enum
{
    GUI_APP_TRANS_ANIM_NONE,                                        //!<Disable app's trans anim
    GUI_APP_TRANS_ANIM_PUSH_RIGHT_IN,                               //!<
    GUI_APP_TRANS_ANIM_PUSH_RIGHT_OUT,                              //!<
    GUI_APP_TRANS_ANIM_PUSH_LEFT_IN,                                //!<
    GUI_APP_TRANS_ANIM_PUSH_LEFT_OUT,                               //!<
    GUI_APP_TRANS_ANIM_ZOOM_IN,                                     //!<
    GUI_APP_TRANS_ANIM_ZOOM_OUT,                                    //!<
    GUI_APP_TRANS_ANIM_CUSTOM,                                     //!<Customized tansform animation
    GUI_APP_TRANS_ANIM_ENTER_DEFAULT = 0xFFFE,   //!<Use system default animation
    GUI_APP_TRANS_ANIM_EXIT_DEFAULT  = 0xFFFF,   //!<Use system default animation
} gui_app_trans_anim_type_t;

#define MANUAL_TRAN_ANIM_MAX_PROCESS 10000
#define FLAG_TRANS_ANIM_REVERSE  0x01    //Goback animation
#define FLAG_TRANS_ANIM_FG       0x02    //Foreground animation(on_resumed screen, or on_paused screen if it's goback animation)
typedef void (*cust_anim_cb_t)(gui_anim_obj_t img, void *user_data, uint32_t flag, gui_anim_value_t process);


typedef struct
{
    gui_point_t pivot;                       //!< Animation zoom pivot
    gui_anim_value_t zoom_start;             //!< zoom start value
    gui_anim_value_t zoom_end;               //!< zoom end value
    gui_anim_value_t opa_start;              //!< opaque start value
    gui_anim_value_t opa_end;                //!< opaque end value
} gui_app_trans_anim_zoom_t;

typedef struct
{
    gui_anim_value_t opa_start;              //!< opaque start value
    gui_anim_value_t opa_end;                //!< opaque end value
    gui_coord_t      x_start;                //!< push start x-coordinate
    gui_coord_t      x_end;                  //!< push end x-coordinate
} gui_app_trans_anim_push_t;

typedef struct
{
    cust_anim_cb_t exe_cb;
    void *user_data;
} gui_app_trans_anim_custom_t;

typedef struct
{
    gui_app_trans_anim_type_t type;         //!< Animation type
    union
    {
        gui_app_trans_anim_zoom_t zoom;     //!< Configuration of zoom animation
        gui_app_trans_anim_push_t push;     //!< Configuration of psuh animation
        gui_app_trans_anim_custom_t cust;   //!< Configuration of customized animation
    } cfg;
} gui_app_trans_anim_t;


typedef struct
{
    gui_app_trans_anim_t a_enter;
    gui_app_trans_anim_t a_exit;
    int8_t prio_up; //parent
    int8_t prio_down;//children
} gui_app_trans_anim_group_t;


/**
* @brief get default configuration of __type__ animation
* \n
*
* @param cfg - output config
* @param type - animation type id
* \n
* @see
*/
void gui_app_trans_anim_init_cfg(gui_app_trans_anim_t *cfg, gui_app_trans_anim_type_t type);
/**
* @brief set app's enter transform animation
* \n
*
* @param cfg
* \n
* @see
*/
void gui_app_set_enter_trans_anim(gui_app_trans_anim_t *cfg);
/**
* @brief set app's exit transform animation
* \n
*
* @param cfg
* \n
* @see
*/
void gui_app_set_exit_trans_anim(gui_app_trans_anim_t *cfg);

void gui_app_set_trans_anim_prio(int8_t up, int8_t down);


rt_err_t gui_app_manual_animation_start(uint32_t process);
rt_err_t gui_app_manual_animation_update(uint32_t process);
rt_err_t gui_app_manual_animation_stop(uint32_t process);



extern const builtin_app_desc_t *gui_builtin_app_list_open(void);

extern const builtin_app_desc_t *gui_builtin_app_list_get_next(const builtin_app_desc_t *ptr_app);

#ifdef PKG_USING_MICROPYTHON
    #define gui_script_app_list_get_next gui_python_app_list_get_next
    #define gui_script_watch_face_register gui_python_watch_face_register
    extern const builtin_app_desc_t *gui_python_app_list_get_next(const builtin_app_desc_t *ptr_app);
    extern void gui_python_watch_face_register(void);
#elif defined(PKG_USING_QUICKJS)
    #define gui_script_app_list_get_next gui_qjs_app_list_get_next
    #define gui_script_watch_face_register gui_qjs_watch_face_register
    extern const builtin_app_desc_t *gui_qjs_app_list_get_next(const builtin_app_desc_t *ptr_app);
    extern void gui_qjs_watch_face_register(void);
#else
    #define gui_script_app_list_get_next(app) NULL
    #define gui_script_watch_face_register()
#endif

extern void gui_builtin_app_list_close(const builtin_app_desc_t *ptr_app);

#ifdef PKG_USING_MICROPYTHON
    extern void pyapp_msg_handler(gui_app_msg_type_t msg, void *param);
#endif

/// @}  gui_app_fwk

#endif  /* __GUI_APP_FWK_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
