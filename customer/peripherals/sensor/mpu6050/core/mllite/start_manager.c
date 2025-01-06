/**
  ******************************************************************************
  * @file   start_manager.c
  * @author Sifli software development team
  * @brief     Motion Library - Start Manager
 *              Start Manager
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

#include <string.h>
#include "log.h"
#include "start_manager.h"

typedef inv_error_t (*inv_start_cb_func)();
struct inv_start_cb_t
{
    int num_cb;
    inv_start_cb_func start_cb[INV_MAX_START_CB];
};

static struct inv_start_cb_t inv_start_cb;

/** Initilize the start manager. Typically called by inv_start_mpl();
* @return Returns INV_SUCCESS if successful or an error code if not.
*/
inv_error_t inv_init_start_manager(void)
{
    memset(&inv_start_cb, 0, sizeof(inv_start_cb));
    return INV_SUCCESS;
}

/** Removes a callback from start notification
* @param[in] start_cb function to remove from start notification
* @return Returns INV_SUCCESS if successful or an error code if not.
*/
inv_error_t inv_unregister_mpl_start_notification(inv_error_t (*start_cb)(void))
{
    int kk;

    for (kk = 0; kk < inv_start_cb.num_cb; ++kk)
    {
        if (inv_start_cb.start_cb[kk] == start_cb)
        {
            // Found the match
            if (kk != (inv_start_cb.num_cb - 1))
            {
                memmove(&inv_start_cb.start_cb[kk],
                        &inv_start_cb.start_cb[kk + 1],
                        (inv_start_cb.num_cb - kk - 1)*sizeof(inv_start_cb_func));
            }
            inv_start_cb.num_cb--;
            return INV_SUCCESS;
        }
    }
    return INV_ERROR_INVALID_PARAMETER;
}

/** Register a callback to receive when inv_start_mpl() is called.
* @param[in] start_cb Function callback that will be called when inv_start_mpl() is
*            called.
* @return Returns INV_SUCCESS if successful or an error code if not.
*/
inv_error_t inv_register_mpl_start_notification(inv_error_t (*start_cb)(void))
{
    if (inv_start_cb.num_cb >= INV_MAX_START_CB)
        return INV_ERROR_INVALID_PARAMETER;

    inv_start_cb.start_cb[inv_start_cb.num_cb] = start_cb;
    inv_start_cb.num_cb++;
    return INV_SUCCESS;
}

/** Callback all the functions that want to be notified when inv_start_mpl() was
* called.
* @return Returns INV_SUCCESS if successful or an error code if not.
*/
inv_error_t inv_execute_mpl_start_notification(void)
{
    inv_error_t result, first_error;
    int kk;

    first_error = INV_SUCCESS;

    for (kk = 0; kk < inv_start_cb.num_cb; ++kk)
    {
        result = inv_start_cb.start_cb[kk]();
        if (result && (first_error == INV_SUCCESS))
        {
            first_error = result;
        }
    }
    return first_error;
}

/**
 * @}
 *//************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
