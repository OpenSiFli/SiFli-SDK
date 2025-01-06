/**
  ******************************************************************************
  * @file   patch.c
  * @author Sifli software development team
  * @brief  patch entry definition
  * @{
  ******************************************************************************
*/
/**
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
#include "btdm_patch.h"

uint32_t g_patch_type[PATCH_TYPE_MAX];

patch_no_parm_no_return_t     patch_no_parm_no_return   = patch_no_parm_no_return_handler;
patch_no_parm_have_return_t   patch_no_parm_have_return = patch_no_parm_have_return_handler;
patch_1_parm_no_return_t      patch_1_parm_no_return    = patch_1_parm_no_return_handler;
patch_1_parm_have_return_t    patch_1_parm_have_return  = patch_1_parm_have_return_handler;
patch_2_parm_no_return_t      patch_2_parm_no_return    = patch_2_parm_no_return_handler;
patch_2_parm_have_return_t    patch_2_parm_have_return  = patch_2_parm_have_return_handler;
patch_3_parm_no_return_t      patch_3_parm_no_return    = patch_3_parm_no_return_handler;
patch_3_parm_have_return_t    patch_3_parm_have_return  = patch_3_parm_have_return_handler;
patch_4_parm_no_return_t      patch_4_parm_no_return    = patch_4_parm_no_return_handler;
patch_4_parm_have_return_t    patch_4_parm_have_return  = patch_4_parm_have_return_handler;
patch_vari_parm_no_return_t      patch_vari_parm_no_return    = patch_vari_parm_no_return_handler;
patch_vari_parm_have_return_t    patch_vari_parm_have_return  = patch_vari_parm_have_return_handler;



void patch_no_parm_no_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret)
{
    *ret = DIRECT_RETURN;
    return;
}
uint32_t patch_no_parm_have_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret)
{
    uint32_t ret_param = 0;

    *ret = DIRECT_RETURN;
    return ret_param;
}
void patch_1_parm_no_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1)
{
    *ret = DIRECT_RETURN;
    return;
}

uint32_t patch_1_parm_have_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1)
{
    uint32_t ret_param = 0;

    *ret = DIRECT_RETURN;
    return ret_param;
}

void patch_2_parm_no_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2)
{
    *ret = DIRECT_RETURN;
    return;
}
uint32_t patch_2_parm_have_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2)
{
    uint32_t ret_param = 0;

    *ret = DIRECT_RETURN;
    return ret_param;
}

void patch_3_parm_no_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2, uint32_t para3)
{
    *ret = DIRECT_RETURN;
    return;
}

uint32_t patch_3_parm_have_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2, uint32_t para3)
{
    uint32_t ret_param = 0;

    *ret = DIRECT_RETURN;
    return ret_param;
}

void patch_4_parm_no_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2, uint32_t para3, uint32_t para4)
{
    *ret = DIRECT_RETURN;
    return;
}

uint32_t patch_4_parm_have_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2, uint32_t para3, uint32_t para4)
{
    uint32_t ret_param = 0;

    *ret = DIRECT_RETURN;
    return ret_param;
}

void patch_vari_parm_no_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, ...)
{
    *ret = DIRECT_RETURN;
    return;
}

uint32_t patch_vari_parm_have_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, ...)
{
    uint32_t ret_param = 0;

    *ret = DIRECT_RETURN;
    return ret_param;
}



