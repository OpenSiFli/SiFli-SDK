/**
  ******************************************************************************
  * @file   data_service.h
  * @author Sifli software development team
  * @brief Sifli Data service interface
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

#ifndef DATA_SERVICE_H
#define DATA_SERVICE_H
#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>

/**
 ****************************************************************************************
* @addtogroup data_service  Data service interface
* @ingroup middleware
* @brief Sifli Data service interface
* @{
*/

/**
 ****************************************************************************************
* @addtogroup data_service_common Data service common interface
* @ingroup data_service
* @brief Sifli Data service common APIs
* @{
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup data_service_message Message enum and types
 * @ingroup data_service
 * @brief  Data Service messages and types
 * @{
 ****************************************************************************************
 */

/*Message exchanged between data service provider and subscriber*/

/** Invalid data client handle */
#define DATA_CLIENT_INVALID_HANDLE   (0)

#define GET_MSG_ID(msg_id)    ((~RSP_MSG_TYPE)&(msg_id))
#define IS_RSP_MSG(msg_id)    ((msg_id) & RSP_MSG_TYPE)
#define RSP_MSG_TYPE (0x8000)
enum
{
    MSG_SERVICE_SUBSCRIBE_REQ = 0x0,
    MSG_SERVICE_SUBSCRIBE_RSP = RSP_MSG_TYPE | MSG_SERVICE_SUBSCRIBE_REQ,
    MSG_SERVICE_UNSUBSCRIBE_REQ = 0x01,
    MSG_SERVICE_UNSUBSCRIBE_RSP = RSP_MSG_TYPE | MSG_SERVICE_UNSUBSCRIBE_REQ,
    MSG_SERVICE_CONFIG_REQ =    0x02,
    MSG_SERVICE_CONFIG_RSP =    RSP_MSG_TYPE | MSG_SERVICE_CONFIG_REQ,
    MSG_SERVICE_TX_REQ =        0x03,
    MSG_SERVICE_TX_RSP =        RSP_MSG_TYPE | MSG_SERVICE_TX_REQ,
    MSG_SERVICE_RX_REQ =        0x04,
    MSG_SERVICE_RX_RSP =        RSP_MSG_TYPE | MSG_SERVICE_RX_REQ,
    MSG_SERVICE_PROXY_IND  =    0x05,
    MSG_SERVICE_START_REQ  =    0x06,
    MSG_SERVICE_START_RSP  =    RSP_MSG_TYPE | MSG_SERVICE_START_REQ,
    MSG_SERVICE_STOP_REQ   =    0x07,
    MSG_SERVICE_STOP_RSP   =    RSP_MSG_TYPE | MSG_SERVICE_STOP_REQ,
    MSG_SERVICE_DATA_RDY_IND  = 0x08,
    MSG_SERVICE_DATA_NTF_IND  = RSP_MSG_TYPE | 0x09,
    MSG_SERVICE_PING_REQ      = 0x0A,
    MSG_SERVICE_PING_RSP      = RSP_MSG_TYPE | MSG_SERVICE_PING_REQ,
    MSG_SERVICE_SLEEP_REQ     = 0x0B,
    MSG_SERVICE_SLEEP_RSP     = RSP_MSG_TYPE | MSG_SERVICE_SLEEP_REQ,

    MSG_SERVICE_SYS_ID_END   =  0x2F,
    MSG_SERVICE_CUSTOM_ID_BEGIN   =  MSG_SERVICE_SYS_ID_END + 1,   //0x30

};

typedef uint8_t datac_handle_t;

typedef struct data_callback_arg_tag data_callback_arg_t;

typedef int(*data_callback_t)(data_callback_arg_t *arg);

/*** data service callback type  */
struct data_callback_arg_tag
{
    uint16_t msg_id;                /*!< Message ID, see MSG_SERVICE_XXX */
    uint16_t data_len;              /*!< Parameter length */
    uint8_t  *data;                 /*!< Parameter for message, see data_xxx_t */
    uint32_t user_data;             /*!< User provided context when subscribing*/
};


///Parameter for MSG_SERVICE_SUBSCRIBE_REQ
typedef struct
{
    char service_name[0];
} data_subscribe_req_t;

/// MSG_SERVICE_SUBSCRIBE_RSP body structure
typedef struct
{
    int32_t result;
    datac_handle_t handle;
} data_subscribe_rsp_t;


///Parameter for other request messages
typedef struct
{
    uint16_t len;
    uint8_t data[0];
} data_req_t;

///Parameter for response messages
typedef struct
{
    int32_t result;
} data_rsp_t;

///Parameter for MSG_SERVICE_DATA_RDY_IND
typedef struct
{
    uint32_t len;
    uint8_t *data;
} data_rdy_ind_t;

#define DATA_SVC_THREAD_MB        1     /*!< Data service mailbox thread ID*/
#define DATA_SVC_THREAD_PROC      2     /*!< Data service internal process thread ID*/

/*
    We'll saving short msg body which length <= SHORT_DATA_MSG_BODY_THRESHOLD

    in message header to avoid allocate memory from heap.
*/
#define SHORT_DATA_MSG_BODY_THRESHOLD   (12)

/// data service message structure type
typedef struct
{
    uint16_t src_cid;       // Internal use only
    uint16_t dst_cid;       // Internal use only
    uint16_t msg_id;        /*!< Message ID, see MSG_SERVICE_XXX */
    uint16_t len;            /*!< Parameter length */
    uint32_t no_free: 1;     /**< 1: no need to free memory of long msg, 0: need to free memory of long msg */
    uint32_t reserved: 31;
    uint8_t  body[SHORT_DATA_MSG_BODY_THRESHOLD]; /*!< Saving whole short msg body, which length <= SHORT_DATA_MSG_BODY_THRESHOLD.
                                                       For long msg body which length > SHORT_DATA_MSG_BODY_THRESHOLD,
                                                       we'll allocate a memroy from heap and save a pointer here.
                                                  */
} data_msg_t;

/// @} data_service_message


typedef struct
{
    /** mbox thread stack size in byte */
    uint16_t mbox_thread_stack_size;
    /** mbox thread stack buffer, if not specified, allocated interanlly */
    void *mbox_thread_stack;
    /** mbox thread thread priority */
    uint8_t mbox_thread_priority;

    /** proc thread stack size in byte */
    uint16_t proc_thread_stack_size;
    /** proc thread stack buffer, if not specified, allocated interanlly */
    void *proc_thread_stack;
    /** proc thread thread priority */
    uint8_t proc_thread_priority;
} data_service_init_param_t;


#if defined(BSP_USING_DATA_SVC)||defined(_SIFLI_DOXYGEN_)

    /**
    * @brief Start data service framework
    * @retval success: 0. failure: other value
    */
    int datas_start(data_service_init_param_t *init_param);

    /**
    * @brief Initialize data service message
    * @param[in,out] msg Message pointer to be initialized
    * @param[in] msg_id Message id
    * @param[in] body_len Size of message body
    * @retval Pointer to message body.
    */
    uint8_t *data_service_init_msg(data_msg_t *msg, uint16_t msg_id, uint16_t body_len);

    /**
    * @brief Free data service message
    *
    * no need to call it if msg is sent by data service API, such as datac_send_msg and datac_tx
    *
    * @param[in] msg Message pointer to be freed
    * @return void
    */
    void data_service_deinit_msg(data_msg_t *msg);

    /**
    * @brief Get message body
    * @param[in] msg Message pointer
    * @retval Pointer to message body.
    */
    uint8_t *data_service_get_msg_body(data_msg_t *msg);
    #include "data_service_subscriber.h"
    #include "data_service_provider.h"


    /**
    * @brief service data process entry
    * @param[in] queue queue
    */
    void data_service_entry(void *queue);

    /**
    * @brief Get data service thread object.
    * @param[in] ds_id service id, see DATA_SVC_THREAD_XXX
    * @retval  RTOS Thread object, currently type of rt_thread_t.
    */
    void *data_service_get_thread(uint8_t ds_id);
#else
    #define data_service_init_msg(msg,msg_id,body_len) -1
    #define data_service_get_msg_body(msg)     -1
#endif


/// @} data_service_common

/// @} data_service

/// @}  file


#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
