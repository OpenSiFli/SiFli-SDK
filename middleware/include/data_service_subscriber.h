/**
  ******************************************************************************
  * @file   data_service_subscriber.h
  * @author Sifli software development team
  * @brief Sifli Data service subscriber interface
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

#ifndef DATA_SERVICE_SUBSCRIBER_H
#define DATA_SERVICE_SUBSCRIBER_H
#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
#include "data_service.h"

/**
 ****************************************************************************************
* @addtogroup data_service_subscriber Data service Subscriber
* @ingroup data_service
* @brief Sifli Data service subscriber interface
* @{
****************************************************************************************
*/

#if defined(BSP_USING_DATA_SVC)||defined(_SIFLI_DOXYGEN_)

/** Data service */
typedef struct data_service_mq_tag
{
    //uint8_t hdl;
    data_callback_t     callback;
    data_callback_arg_t arg;
} data_service_mq_t;

/**
****************************************************************************************
* @addtogroup data_service_subscriber_func Data service Subscriber functions
* @ingroup data_service
* @brief  Data Service subscriber function API
* @{
****************************************************************************************
*/

/**
 * @brief Allocate data client handle
 *
 * @return  data client handle
 */
datac_handle_t datac_open(void);

/**
 * @brief Close data client handle
 * @param[in] handle Handle  of data service
 *
 *  If service is subscribed by the handle, it's unsubscribed autuomatically
 *
 * @retval RT_EOK if successful, otherwise return error number < 0.
 */
rt_err_t datac_close(datac_handle_t handle);


/**
 * @brief Subscribe to service
 * @param[in] handle Handle  of data service
 * @param[in] service_name Service name
 * @param[in] cbk  Callbacks to handel data service
 * @param[in] user_data  Callbacks context, transparent to service provider.
 * @param[in] mq  instead of callback directly, send message to mq to issue callback from different task.
 * @retval   void
 */
void datac_subscribe_ex(datac_handle_t handle, char *service_name,
                        data_callback_t cbk, uint32_t user_data, rt_mq_t mq);
/**
 * @brief Subscribe to service
 * @param[in] handle Handle  of data service
 * @param[in] service_name Service name
 * @param[in] cbk  Callbacks to handel data service
 * @param[in] user_data  Callbacks context, transparent to service provider.
 * @retval   void
 */
#define datac_subscribe(handle,service_name,cbk,user_data) datac_subscribe_ex(handle,service_name,cbk,user_data,NULL)

/**
 * @brief Unsubscribe to service
 * @param[in] handle Handle  of data service
 * @param[in] force if true, will immediatly stop all callbacks to subscriber, including unsubscribe response.
 * @retval RT_EOK if successful, otherwise return error number < 0.
 */
rt_err_t datac_unsubscribe_ex(datac_handle_t handle, bool force);
/**
 * @brief Unsubscribe to service
 * @param[in] handle Handle  of data service
 * @retval RT_EOK if successful, otherwise return error number < 0.
 */
#define datac_unsubscribe(handle) datac_unsubscribe_ex(handle, false)

/**
 * @brief Delayed subscriber call back, this will trigger subscriber callback from user defined task.
 * @param[in] arg_msg msg
 */
void datac_delayed_usr_cbk(data_service_mq_t *arg_msg);

/**
 * @brief Configure the specific data provider,
 * \note data provider will decide which config will be used compare to existing configs.
 * @param[in] handle Handle of data service
 * @param[in] len Length of config
 * @param[in] config Configuration content
 * @retval RT_EOK if successful, otherwise return error number < 0.
 */
rt_err_t datac_config(datac_handle_t handle, uint16_t len, uint8_t *config);

/**
 * @brief Send message to service provider
 * @param[in] handle Handle of data service
 * @param[in] msg Message send to service provider
 * @retval RT_EOK if successful, otherwise return error number < 0.
 */
rt_err_t datac_send_msg(datac_handle_t handle, data_msg_t *msg);

/**
 * @brief Send data to service provider
 * @param[in] handle Handle of data service
 * @param[in] len Length of data
 * @param[in] data service data content
 * @retval RT_EOK if successful, otherwise return error number < 0.
 */
rt_err_t datac_tx(datac_handle_t handle, uint16_t len, uint8_t *data);

/**
 * @brief Read data from service provider
 * @param[in] handle Handle of data service
 * @param[in] len Length of data
 * @param[in,out] data service data content readback
 * @retval RT_EOK if successful, otherwise return error number < 0.
 */
rt_err_t datac_rx(datac_handle_t handle, uint16_t len, uint8_t *data);

/**
 * @brief ping device from service provider
 * @param[in] handle Handle of data service
 * @param[in] mode ping mode
 * @retval RT_EOK if successful, otherwise return error number < 0.
 */
rt_err_t datac_ping(datac_handle_t handle, uint8_t mode);

/// @} data_service_subscriber_func

#else
#define datac_subscribe(name,cbk,user_data) -1
#define datac_unsubscribe(handle) -1
#define datac_start(handle) -1
#define datac_config(handle,len,config) -1
#define datac_send_msg(handle,msg) -1
#define datac_tx(handle,len,data) -1
#define datac_rx(handle,len,data) -1
#define datac_ping(handle, mode)  -1
#define datac_delayed_usr_cbk(arg_msg)  -1
#define datac_subscribe_ex(handle,service_name,cbk,user_data,mq) -1
#endif

/// @} data_service_subscriber
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
