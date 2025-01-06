/**
  ******************************************************************************
  * @file   ipc_hw_port.h
  * @author Sifli software development team
  * @brief Sifli ipc hw interface
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

#ifndef IPC_HW_PORT_H
#define IPC_HW_PORT_H
#include "register.h"

/**
 ****************************************************************************************
* @addtogroup ipc IPC Library
* @ingroup middleware
* @brief Sifli ipc library interface
* @{
****************************************************************************************
*/


#ifdef __cplusplus
extern "C" {
#endif

#define IPC_HW_CH_NUM      (1)
#define IPC_HW_QUEUE_NUM   (8)

#define IPC_HL_HW_QUEUE_0  (0)
#define IPC_HL_HW_QUEUE_1  (1)
#define IPC_HL_HW_QUEUE_2  (2)
#define IPC_HL_HW_QUEUE_3  (3)
#define IPC_HL_HW_QUEUE_4  (4)
#define IPC_HL_HW_QUEUE_5  (5)
#define IPC_HL_HW_QUEUE_6  (6)
#define IPC_HL_HW_QUEUE_7  (7)

/// @}  file

#ifdef __cplusplus
}
#endif




/// @} button
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
