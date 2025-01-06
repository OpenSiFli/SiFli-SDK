/**
  ******************************************************************************
  * @file   data_service_provider.h
  * @author Sifli software development team
  * @brief Sifli Data service provider interface
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

#ifndef DATA_SERVICE_PROVIDER_H
#define DATA_SERVICE_PROVIDER_H
#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
#include "data_service.h"

/**
 ****************************************************************************************
* @addtogroup data_service_provider Data service Provider
* @ingroup data_service
* @brief Sifli Data service provider interface
* @{
****************************************************************************************
*/

#define DATA_SERVICE_INVALID_ID   (0x7F)

/** Data service client */
typedef void *datas_handle_t;


/** service message handler */
typedef int32_t (*data_service_msg_handler_t)(datas_handle_t service, data_msg_t *msg);
typedef bool (*data_filter_t)(data_req_t *config, uint16_t msg_id, uint32_t len, uint8_t *data);

typedef struct data_service_config_tag
{
    /** maximum client number supported by the serivce */
    uint8_t max_client_num;
    /** message queue used by the service,
     *
     * If it's NULL, the service shares the system queue and thread.
     * If it's not NULL, the message would be posted to this queue, so service should create its own thread and
     * retrieve message from this queue.
     * For simlicity the user created thread could use #data_service_entry as thread entry and pass the queue as thread's parameter,
     * such as `rt_thread_init(&thread_handle, "test", data_service_entry, queue, thread_stack, sizeof(thread_stack), 25, 10);`
     *
     */
    rt_mq_t queue;
    /** filter which is called to check whether one message needs to be pushed to the client */
    data_filter_t data_filter;
    /** service message handler */
    data_service_msg_handler_t msg_handler;
} data_service_config_t;


#if defined(BSP_USING_DATA_SVC)||defined(_SIFLI_DOXYGEN_)


    /**
    @brief Trigger service main thread to send customer message to the subscriber
    @param[in] service Handle of data service
    @param[in] msg_id Message id
    @param[in] len Size of data available
    @param[in] data content of data
    @retval RT_EOK if successful, otherwise return error number < 0.
    */
    int32_t datas_push_msg_to_client(datas_handle_t service, uint16_t msg_id, uint32_t len, uint8_t *data);
    #define datas_push_data_to_client(service,len,data)    datas_push_msg_to_client(service,MSG_SERVICE_DATA_NTF_IND,len,data)

    /**
    @brief Trigger service main thread to send customer message to the subscriber,

    #data is not copied and accessed directly by client if serivce and client are in the same core.
    Service cannot free #data after this function call as it would be used by client.
    If service and client are in different core, the message is relayed to the client side, i.e. client would use a copy of #data.
    So #data can be freed after this function call.
    @param[in] svc Handle of data service
    @param[in] msg_id Message id
    @param[in] len Size of data available
    @param[in] data content of data
    @retval RT_EOK if successful, otherwise return error number < 0.
    */
    int32_t datas_push_msg_to_client_no_copy(datas_handle_t svc, uint16_t msg_id, uint32_t len, uint8_t *data);


    /**
    @brief  Inform service that data is available to
    @param[in] service Handle of data service
    @param[in] size size of data in bytes available to service.
    @param[in] data point to data, the pointer is carried by message MSG_SERVICE_DATA_RDY_IND,
    so the receiver is responsible to free the memory
    @retval RT_EOK if successful, otherwise return error number < 0.
    */
    rt_err_t datas_data_ready(datas_handle_t service, uint32_t size, uint8_t *data);
    #define datas_ind_size(service,size) datas_data_ready(service,size,RT_NULL);

    /**
    @brief  Send response message to req message
    @param[in] service Handle of data service
    @param[in] msg_req Request message to respond
    @param[in] result Result of response
    @retval RT_EOK if successful, otherwise return error number < 0.
    */
    rt_err_t datas_send_response(datas_handle_t service, data_msg_t *msg_req, rt_err_t result);

    /**
    @brief  Send response message to req message
    @param[in] service Handle of data service
    @param[in] msg_req Request message to respond
    @param[in] len Size of data available
    @param[in] data content of data
    @retval RT_EOK if successful, otherwise return error number < 0.
    */
    rt_err_t datas_send_response_data(datas_handle_t service, data_msg_t *msg_req, uint32_t len, uint8_t *data);
    /**
    @brief  Register data service provider
    @param[in] name Name of data service
    @param[in] config Data service config
    @retval Service handle.
    */
    datas_handle_t datas_register(const char *name, data_service_config_t *config);
#else
    #define datac_subscribe(name,cbk) -1
    #define datac_unsubscribe(handle) -1
    #define datac_start(handle) -1

#endif

/// @} data_service_provider
/// @} file

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

