/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lvsf_file_explorer.h
 *
 */

#ifndef _LVSF_FILE_EXPLORER_H
#define _LVSF_FILE_EXPLORER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include "lvgl.h"
#include "rtthread.h"

#define LV_FILE_EXPLORER_PATH_MAX_LEN 128

#ifndef LVSF_USING_FILE_EXPLORER
    #define LVSF_USING_FILE_EXPLORER 1
#endif

#if LVSF_USING_FILE_EXPLORER != 0

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

typedef void(*file_explorer_select_cb_t)(lv_obj_t *obj, char *full_path);


/*Reserved*/
typedef enum
{
    LV_EXPLORER_SORT_NONE = 0,
    LV_EXPLORER_SORT_KIND,
} lv_file_explorer_sort_t;

typedef enum
{
    LV_EXPLORER_FILTER_FILE = 0,    /**< filter files which suffixs set by lv_file_explorer_set_filter*/
    LV_EXPLORER_FILTER_DIR,         /**< filter directory */
} lv_file_filter_t;

typedef enum
{
    LV_EXPLORER_DIR,
    LV_EXPLORER_IMG,
    LV_EXPLORER_AUDIO,
    LV_EXPLORER_VIDEO,
    LV_EXPLORER_FILE,
} lv_file_type_t;

typedef struct
{
    lv_file_type_t type;
    const char *str_fn;
    rt_list_t list;
} lv_file_explorer_node_t;

typedef struct
{
    lv_file_explorer_node_t *node;
    lv_obj_t *obj;
    const char *sel_fn;
} lv_file_explorer_select_t;

/*Data of canvas*/
typedef struct
{
    lv_obj_t obj;
    lv_obj_t *cont;
    lv_obj_t *head_area;
    lv_obj_t *back_label;
    lv_obj_t *sure_label;
    lv_obj_t *browser_area;
    lv_obj_t *file_table_list;
    lv_obj_t *path_label;
    const char *select_item_path;
    char current_path[LV_FILE_EXPLORER_PATH_MAX_LEN];
    lv_file_explorer_sort_t sort;
    lv_file_filter_t filter;
    lv_file_explorer_select_t select;

    char *main_dir;
    char **suffix;
    uint16_t file_cnt;
    rt_list_t list;
    file_explorer_select_cb_t select_cb;
    file_explorer_select_cb_t backspace_cb;
    file_explorer_select_cb_t copy_cb;
    file_explorer_select_cb_t del_cb;
} lv_file_explorer_t;

extern const lv_obj_class_t lv_file_explorer_class;

/***********************
 * GLOBAL VARIABLES
 ***********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
lv_obj_t *lv_file_explorer_create(lv_obj_t *parent);

/*=====================
 * Setter functions
 *====================*/

/**
 * Set file_explorer sort, Reserved
 * @param obj   pointer to a file explorer object
 * @param sort  the sort from 'lv_file_explorer_sort_t' enum.
 */
void lv_file_explorer_set_sort(lv_obj_t *obj, lv_file_explorer_sort_t sort);

/*=====================
 * Getter functions
 *====================*/

/**
 * Get file explorer Selected file
 * @param obj   pointer to a file explorer object
 * @return      pointer to the file explorer selected file name
 */
const char *lv_file_explorer_get_selected_file_name(lv_obj_t *obj);

/**
 * Get file explorer cur path
 * @param obj   pointer to a file explorer object
 * @return      pointer to the file explorer cur path
 */
const char *lv_file_explorer_get_current_path(lv_obj_t *obj);

/**
 * Get file explorer head area obj
 * @param obj   pointer to a file explorer object
 * @return      pointer to the file explorer head area obj(lv_obj)
 */
lv_obj_t *lv_file_explorer_get_header(lv_obj_t *obj);


/**
 * Get file explorer path obj(label)
 * @param obj   pointer to a file explorer object
 * @return      pointer to the file explorer path obj(lv_label)
 */
lv_obj_t *lv_file_explorer_get_path_label(lv_obj_t *obj);

/**
 * Get file explorer file list obj(lvsf_multlist)
 * @param obj   pointer to a file explorer object
 * @return      pointer to the file explorer file table obj(lv_table)
 */
lv_obj_t *lv_file_explorer_get_file_table(lv_obj_t *obj);

/**
 * Set file_explorer sort, reseved
 * @param obj   pointer to a file explorer object
 * @return the current mode from 'lv_file_explorer_sort_t'
 */
lv_file_explorer_sort_t lv_file_explorer_get_sort(lv_obj_t *obj);

/*=====================
 * Other functions
 *====================*/

/**
 * Open a specified path
 * @param obj   pointer to a file explorer object
 * @param dir   pointer to the path
 */
void lv_file_explorer_open_dir(lv_obj_t *obj, const char *dir);

/**
 * Set main directory, it only save address of param dir
 * @param obj   pointer to a file explorer object
 * @param dir   pointer to the path
 */
void lv_file_explorer_set_main_dir(lv_obj_t *obj, const char *dir);


/**
 * Set filter for list file
 * @param obj   pointer to a file explorer object
 * @param filter ser lv_file_filter_t
 * @param suffix string array, must end with NULL, such as {".mp3","mp4", NULL}, valid only for filter is LV_EXPLORER_FILTER_FILE
 */
void lv_file_explorer_set_filter(lv_obj_t *obj, lv_file_filter_t filter, const char **suffix);


/**
 * Set select callback function for target select
 * @param obj   pointer to a file explorer object
 * @param cb  callback
 */
void lv_file_explorer_set_select_cb(lv_obj_t *obj, file_explorer_select_cb_t cb);

/**
 * Set backspace callback function for backspace icon clicked
 * @param obj   pointer to a file explorer object
 * @param cb  callback
 */
void lv_file_explorer_set_backspace_cb(lv_obj_t *obj, file_explorer_select_cb_t cb);

/**
 * Set copy callback function for file item clicked
 * @param obj   pointer to a file explorer object
 * @param cb  callback
 */
void lv_file_explorer_set_copy_cb(lv_obj_t *obj, file_explorer_select_cb_t cb);

/**
 * Set del callback function for file item left swipe delete
 * @param obj pointer to a file explorer object
 * @param cb callback
 */
void lv_file_explorer_set_del_cb(lv_obj_t *obj, file_explorer_select_cb_t cb);

/**
 * Set default select path
 * @param obj pointer to a file explorer object
 * @param cb callback
 */
void lv_file_explorer_set_select_path(lv_obj_t *obj, const char *full_path);

/**********************
 *      MACROS
 **********************/

#endif  /*LVSF_USING_FILE_EXPLORER*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*_LVSF_FILE_EXPLORER_H*/
