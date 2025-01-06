/**
  ******************************************************************************
  * @file   ipc_queue.h
  * @author Sifli software development team
  * @brief Sifli ipc_queue library interface
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

#ifndef IPC_QUEUE_H
#define IPC_QUEUE_H
#include <stdint.h>
#include <stdbool.h>
#ifndef BSP_USING_PC_SIMULATOR
    #include "ipc_hw_port.h"
#endif
/**
 ****************************************************************************************
* @addtogroup ipc_queue Inter-Processor Queue Library
* @ingroup middleware
* @brief Sifli ipc_queue library interface
* @{
****************************************************************************************
*/


#ifdef __cplusplus
extern "C" {
#endif

/** Invalid queue handle */
#define IPC_QUEUE_INVALID_HANDLE   (0)

/** IPC queue handle type */
typedef int32_t ipc_queue_handle_t;

/** IPC queue rx callback type */
typedef int32_t (*ipc_queue_rx_ind_t)(ipc_queue_handle_t handle, size_t size);

/** IPC queue configuration */
typedef struct
{
    uint8_t qid;               /**< queue id, starting from 0, numbering across all queues in all channels, @see ipc_hw_port.h */
    uint32_t rx_buf_addr;      /**< rx buffer address from the view of receiver side */
    uint32_t tx_buf_addr;      /**< tx buffer address from the view of sender side */
    uint32_t tx_buf_addr_alias;      /**< tx buffer address from the view of receiver side */
    uint32_t tx_buf_size;      /**< tx buffer size in byte */
    ipc_queue_rx_ind_t rx_ind; /**< rx notification callback  */
    uint32_t user_data;        /**< user data */
} ipc_queue_cfg_t;


/** Initialize one IPC queue according to the q_cfg
 *
 * A valid queue handle is returned if queue is intialized succesfully.
 * The queue with same queue id cannot be initialized twice.
 *
 * @param[in]  q_cfg  queue configuration
 *
 * @return queue handle, #IPC_QUEUE_INVALID_HANDLE denotes initialization failure
 */
ipc_queue_handle_t ipc_queue_init(ipc_queue_cfg_t *q_cfg);

/** Get user data
 *
 * @param[in]      handle    queue handle
 * @param[in,out]  user_data user data is saved in the memory pointed by user_data pointer
 *
 * @return status, 0: success, otherwise: error
 */
int32_t ipc_queue_get_user_data(ipc_queue_handle_t handle, uint32_t *user_data);

/** Set user data
 *
 * @param[in]  handle    queue handle
 * @param[in]  user_data user data
 *
 * @return status, 0: success, otherwise: error
 */
int32_t ipc_queue_set_user_data(ipc_queue_handle_t handle, uint32_t user_data);

/** Open the IPC queue
 *
 * After opened, the queue is ready to receive and send data.
 * As all queues share the same IRQ, receiver side may see the interrupt even if it has not opened the queue,
 * it means the registered callback is not called when interrupt is triggered.
 *
 *
 * @param[in]  handle  queue handle
 *
 * @return status, 0: success, otherwise: error
 */
int32_t ipc_queue_open(ipc_queue_handle_t handle);

/** New implementation for opening the IPC queue to solve rx callback loss issue.
 *
 * After opened, the queue is ready to receive and send data.
 * If receiver has not opened the queue, it would not receive interrupt even if sender has opened the queue and send the data.
 *
 * @param[in]  handle  queue handle
 *
 * @return status, 0: success, otherwise: error
 */
int32_t ipc_queue_open2(ipc_queue_handle_t handle);

/** Check whether the IPC queue is open
 *
 * @param[in]  handle  queue handle
 *
 * @return true: open, false: not open
 */
bool ipc_queue_is_open(ipc_queue_handle_t handle);

/** Close the IPC queue
 *
 * After closing the queue, it cannot receive and send data.
 * The interrupt might still be received if sender doesn't close the queue and send data.
 *
 * @param[in]  handle  queue handle
 *
 * @return status, 0: success, otherwise: error
 */
int32_t ipc_queue_close(ipc_queue_handle_t handle);

/** New implementation for closing the IPC queue to solve unexpected interrupt issue
 *
 * After closing the queue, it cannot receive and send data.
 *
 * @param[in]  handle  queue handle
 *
 * @return status, 0: success, otherwise: error
 */
int32_t ipc_queue_close2(ipc_queue_handle_t handle);

/** Deinitialize the IPC queue
 *
 * After deinitializatio the queue with same queue id can be intialized again.
 *
 * @param[in]  handle  queue handle
 *
 * @return status, 0: success, otherwise: error
 */
int32_t ipc_queue_deinit(ipc_queue_handle_t handle);

/** Read data from IPC queue
 *
 * Data is read from rx buffer to saved in buffer
 *
 * @param[in]  handle  queue handle
 * @param[in]  buffer  point to the buffer to save read data
 * @param[in]  size    size of data to be read
 *
 * @return actual size of data that has been read
 */
size_t ipc_queue_read(ipc_queue_handle_t handle, void *buffer, size_t size);

/** Write data to IPC queue and inform the receiver to read
 *
 * Data is written to tx buffer, interrupt is triggered to inform the receiver.
 * The API is not thread-safe. It should be avoided to write the same queue in different threads.
 *
 * @param[in]  handle  queue handle
 * @param[in]  buffer  data buffer
 * @param[in]  size    size of data to be written
 * @param[in]  timeout time to wait if tx buffer is full, the unit is #HAL_GetTick
 *
 * @return actual size of data that has been written
 */
size_t ipc_queue_write(ipc_queue_handle_t handle, const void *buffer, size_t size, uint32_t timeout);

/** Check whether ipc queue is idle from sender perspective
 *
 * If all tx_buffer are empty, it's idle, even though rx_buffer is not empty
 *
 * @return true: idle, false: not idle
 */
bool ipc_queue_check_idle(void);

/** ipc_queue_check_idle ROM implementation placeholder
 *
 * Weak function, can be replaced by ROM symbol.
 * If not use ROM symbol, it's same as ipc_queue_check_idle.
 *
 * @return true: idle, false: not idle
 */
bool ipc_queue_check_idle_rom(void);


/** Restore IPC queue hardware setting
 *
 * HW setting may get lost if the power domain is powered off,
 * if the software configuration is present,
 * this function can be used to restore the hw setting after power domain is powered on.
 *
 * @return void
 */
void ipc_queue_restore_all(void);

/** ipc_queue_restore_all ROM implementation placeholder
 *
 * Weak function, can be replaced by ROM symbol.
 * If not use ROM symbol, it's same as ipc_queue_restore_all.
 *
 * @return void
 */
void ipc_queue_restore_all_rom(void);

/** Get available data size in rx buffer
 *
 * @param[in]  handle  queue handle
 *
 * @return data size in rx buffer
 */
size_t ipc_queue_get_rx_size(ipc_queue_handle_t handle);


/// @}  ipc_queue

#ifdef __cplusplus
}
#endif




/// @} file
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
