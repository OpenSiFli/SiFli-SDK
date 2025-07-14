/**
  ******************************************************************************
  * @file   lv_ex_data.h
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

#ifndef LV_EX_DATA_H
#define LV_EX_DATA_H

#define MAX_EX_DATA_NAME_LEN  32

typedef enum
{
    LV_EX_DATA_STRING,
    LV_EX_DATA_INT32,
    LV_EX_DATA_UINT32,
    LV_EX_DATA_FLOAT,
    LV_EX_DATA_TIME,
    LV_EX_DATA_POINTER,
    LV_EX_DATA_LIST,
} lv_ex_data_type_t;

typedef lv_res_t (* lv_ex_data_target_string_setter_t)(lv_obj_t *target, const char *data);
typedef lv_res_t (* lv_ex_data_target_int32_setter_t)(lv_obj_t *target, int32_t data);
typedef lv_res_t (* lv_ex_data_target_uint32_setter_t)(lv_obj_t *target, uint32_t data);
typedef lv_res_t (* lv_ex_data_target_float_setter_t)(lv_obj_t *target, float data);
typedef lv_res_t (* lv_ex_data_target_time_setter_t)(lv_obj_t *target, uint32_t data);
typedef lv_res_t (* lv_ex_data_target_pointer_setter_t)(lv_obj_t *target, void *data);
typedef lv_res_t (* lv_ex_data_target_list_setter_t)(lv_obj_t *target, void *data);

typedef struct
{
    /** binding target object */
    lv_obj_t *target;
    /** binding target object property setter
     *
     *  prototype of setter is:
     *   lv_res_t setter(lv_obj_t *target, arg_type arg);
     */
    void *setter;
    /** property setter argument type */
    lv_ex_data_type_t arg_type;
} lv_ex_binding_t;

typedef union
{
    int32_t i;
    uint32_t ui;
    float f;
    const char *s;
    void *ptr;
} lv_ex_data_value_t;

typedef struct
{
    char name[MAX_EX_DATA_NAME_LEN];
    lv_ex_data_value_t value;
    lv_ex_data_type_t type;
    /* list of lv objects who listen to the data update and consume the data*/
    lv_ll_t listener;
    bool updated;
} lv_ex_data_t;

// create data object
lv_ex_data_t *lv_ex_data_create(const char *name, lv_ex_data_type_t type);
// delete data object
lv_res_t lv_ex_data_delete(lv_ex_data_t *data);


// bind data object
void *lv_ex_bind_data(lv_ex_data_t *data, lv_ex_binding_t *binding);
// variant: ui_elem doesn't need to be lv obj, it just points to one property of the object
//          according to data type, data value is copied to the memory pointed by ui_elem
//          Then any number of properties of one object can be bound
// limitation: how to invalidate the object
//lv_res_t lv_ex_bind_data(void *ui_elem, lv_ex_data_t *data);

lv_res_t lv_ex_unbind_data(lv_ex_data_t *data, void *handle);


// update data object value and notify listeners
lv_res_t lv_ex_data_set_value(lv_ex_data_t *data, void *value);

void lv_ex_process_data(void);

void lv_ex_data_pool_init(void);


#endif /* LV_EX_DATA_H */


