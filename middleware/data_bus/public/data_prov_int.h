/**
  ******************************************************************************
  * @file   data_service_provider.h
  * @author Sifli software development team
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

#ifndef DATA_PROV_INT_H
#define DATA_PROV_INT_H
#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
#include "data_service.h"

/**
 ****************************************************************************************
* @addtogroup data_service_provider Data service Provider
* @ingroup middleware
* @brief Sifli Data service interface
* @{
****************************************************************************************
*/

/** Data service client */
typedef struct data_service_client_tag
{
    rt_list_t   node;             /*!<List node*/
    uint16_t    src_cid;
    uint8_t     conn_id;
    uint8_t     service_id;         /*!<Service ID*/
    uint8_t    *user_data;
    data_req_t *config;
} data_service_client_t;


/** Data service */
#define MAX_SVC_NAME_LEN        9
typedef struct data_service_tag
{
    bool active;                /*!<Service is active*/
    uint8_t id;                 /*!<Service ID*/
    uint8_t last_clnt;          /*!<Last client ID*/
    char name[MAX_SVC_NAME_LEN];/*!<Service name*/
    rt_list_t clients;          /*!<Service clients connected to this service */
    void **client_list;
    const struct data_service_config_tag *config;  /*!< service config */
    rt_list_t node;             /*!<List node */
    /// data fifo
    void *user_data;            /*!< User data of service */
} data_service_t;


/// @} data_service_provider
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
