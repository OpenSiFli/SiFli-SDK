/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LVSF_KEYMANAGE_H__
#define __LVSF_KEYMANAGE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#if LVSF_USING_KEYMANAGE != 0


/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
typedef void (*lv_key_cb_t)(lv_obj_t *obj, char c, void *user_data);

typedef struct
{
    lv_obj_t                        *obj;
    uint16_t                        index;                      /**< setting menu priority                      */
    lv_key_cb_t                     cb;                         /**< setting menu execute callback              */
    void                            *user_data;                 /**< key node user data                     */
    rt_list_t                       list;                       /**< list link                                  */
} lv_key_node_t;

/*Ext data of footer*/
typedef struct
{
    lv_obj_t                        obj;
    void                            *user_data;
    uint16_t                        select_idx;            /* current highlight elem */
    uint16_t                        elem_num;              /* total elem num */
    rt_list_t                       list;

} lv_keymanage_t;

extern const lv_obj_class_t lv_keymanage_class;

/**********************
 * GLOBAL PROTOTYPES
 **********************/
/**
 * Create a footer object
 * @param parent pointer to a parent object.
 *                  If NULL then a screen will be created
 * @param copy pointer to a base object, if not NULL then the new object will be copied from it
 * @return pointer to the new footer object
 */
lv_obj_t *lv_keymanage_create(lv_obj_t *parent);

/**
 * Insert an node into the timeline
 * @param timeline pointer to a timeline object
 * @param start_value
 * @param end_value
 * @param start_time
 * @param end_time
 * @param xcb
 * @param ready_cb
 * @param user_data
 */
void lv_keymanage_add_element(lv_obj_t *keymanage, lv_obj_t *obj, uint16_t index, lv_key_cb_t cb, void *user_data);

void lv_keymanage_set_select_idx(lv_obj_t *keymanage, uint16_t select_idx);

uint16_t lv_keymanage_get_select_idx(lv_obj_t *keymanage);

uint16_t lv_keymanage_get_elem_num(lv_obj_t *keymanage);

lv_key_node_t *lv_keymanage_get_node_by_select_idx(lv_obj_t *keymanage);

lv_obj_t *lv_keymanage_get_obj_by_index(lv_obj_t *keymanage, uint16_t index);

/**********************
 *      MACROS
 **********************/
#endif /*LVSF_USING_KEYMANAGE*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*__LVSF_KEYMANAGE_H__*/

