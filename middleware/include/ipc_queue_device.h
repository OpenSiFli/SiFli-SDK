/**
  ******************************************************************************
  * @file   ipc_queue_device.h
  * @author Sifli software development team
  * @brief Sifli wrappter device interface for ipc_queue
  * @{
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

#ifndef IPC_QUEUE_DEVICE_H
#define IPC_QUEUE_DEVICE_H
#include "rtthread.h"
#include <stdint.h>
#include <stdbool.h>
#include "ipc_queue.h"

/**
 ****************************************************************************************
* @addtogroup ipc_queue_device IPC Queue Wrapper Device
* @ingroup middleware
* @brief wrapper device interface for ipc_queue
* @{
****************************************************************************************
*/


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief  register wrappter device for specified ipc queue
 *
 * @param[in] device rt device
 * @param[in] name device name
 * @param[in] queue ipc queue handle
 *
 * @retval error code
 */
rt_err_t ipc_queue_device_register(rt_device_t device, const char *name, ipc_queue_handle_t queue);


/**
 * @brief  rx notification callback for ipc queue which is bound with rt device
 *
 * The implemented notification callback would call rx_indicate callback of bound rt device
 *
 * @param[in] queue ipc queue handle
 * @param[in] size size in byte
 *
 * @retval status, 0: no error
 */
int32_t ipc_queue_device_rx_ind(ipc_queue_handle_t queue, size_t size);




/// @}  ipc_queue_device

#ifdef __cplusplus
}
#endif




/// @} file
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
