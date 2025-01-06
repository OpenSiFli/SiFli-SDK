/**
  ******************************************************************************
  * @file   mpl.c
  * @author Sifli software development team
  * @brief     Motion Library - Start Point
 *              Initializes MPL.
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

#include "storage_manager.h"
#include "log.h"
#include "mpl.h"
#include "start_manager.h"
#include "data_builder.h"
#include "results_holder.h"
#include "mlinclude.h"

/**
 * @brief  Initializes the MPL. Should be called first and once
 * @return Returns INV_SUCCESS if successful or an error code if not.
 */
inv_error_t inv_init_mpl(void)
{
    inv_init_storage_manager();

    /* initialize the start callback manager */
    INV_ERROR_CHECK(inv_init_start_manager());

    /* initialize the data builder */
    INV_ERROR_CHECK(inv_init_data_builder());

    INV_ERROR_CHECK(inv_enable_results_holder());

    return INV_SUCCESS;
}

const char ml_ver[] = "InvenSense MA 5.1.2";

/**
 *  @brief  used to get the MPL version.
 *  @param  version     a string where the MPL version gets stored.
 *  @return INV_SUCCESS if successful or a non-zero error code otherwise.
 */
inv_error_t inv_get_version(char **version)
{
    INVENSENSE_FUNC_START;
    /* cast out the const */
    *version = (char *)&ml_ver;
    return INV_SUCCESS;
}

/**
 *  @brief  Starts the MPL. Typically called after inv_init_mpl() or after a
 *          inv_stop_mpl() to start the MPL back up an running.
 *  @return INV_SUCCESS if successful or a non-zero error code otherwise.
 */
inv_error_t inv_start_mpl(void)
{
    INV_ERROR_CHECK(inv_execute_mpl_start_notification());
    return INV_SUCCESS;
}

/**
 * @}
 *//************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
