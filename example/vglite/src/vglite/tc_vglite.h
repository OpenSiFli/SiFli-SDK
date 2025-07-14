/**
  ******************************************************************************
  * @file   tc_vglite.h
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

#ifndef __TC_VGLITE_H__
#define __TC_VGLITE_H__

#include "stdint.h"
#include "stdbool.h"
#include "rtthread.h"
#include "utest.h"


void tc_vglite_print_error(const char *func, size_t line, vg_lite_error_t err);

#define TC_VGLITE_CHECK_ERROR(Function)                     \
    error = Function;                                       \
    if (error > 0)                                          \
    {                                                       \
        tc_vglite_print_error(__FUNCTION__, __LINE__, error);   \
        uassert_true(false);                                \
        goto ErrorHandler;                                  \
    }


void tc_vg_send_data_to_lcd(uint8_t *data, uint32_t width, uint32_t height, uint16_t color_fmt);

rt_err_t tc_vglite_init(void);
rt_err_t tc_vglite_cleanup(void);

const char *tc_vglite_get_buf_fmt_str(vg_lite_buffer_format_t buf_format);

const char *tc_vglite_get_blend_mode_str(vg_lite_blend_t blend_mode);

const char *tc_vglite_get_filter_str(vg_lite_filter_t filter);

const char *tc_vglite_get_layout_str(vg_lite_buffer_layout_t layout);


#endif /* __TC_VGLITE_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
