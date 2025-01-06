/**
  ******************************************************************************
  * @file   ui_datasrv_subscriber.h
  * @author Sifli software development team
  * @brief Data service client for GUI application. It helps isolate GUI application thread from
 *   data service running.
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

#ifndef __UI_DATASRV_H__
#define __UI_DATASRV_H__
#include "data_service_subscriber.h"

/**
 ****************************************************************************************
* @addtogroup ui_datac GUI Data service
* @ingroup data_service
* @brief Data service client for GUI application. It helps isolate GUI application thread from
*   data service running.
* @{
****************************************************************************************
*/

/**
    @brief Initialize GUI data service client module.
*/

void ui_datac_init(void);

/**
    @brief Subscribe data service in GUI thread context.
    @param[in] handle data client handle
    @param[in] name Data service name
    @param[in] cbk Callback functions for data service.
    @param[in] user_data Callback function parameter. Service provide will call callback with it.
    @retval Pointer to message body.
    */
datac_handle_t ui_datac_subscribe(datac_handle_t handle, char *name, data_callback_t cbk, uint32_t user_data);

///@} ui_datac
#endif /*__UI_DATASRV_H__*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
