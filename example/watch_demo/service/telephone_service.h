/**
  ******************************************************************************
  * @file   telephone_service.h
  * @author Sifli software development team
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

#ifndef TELEPHONE_SERVICE_H
#define TELEPHONE_SERVICE_H
#include <rtthread.h>
#include "data_service.h"

enum
{
    TELEPHONE_MSG_START = MSG_SERVICE_CUSTOM_ID_BEGIN, //0x30
    /*****Request messages*****/
    TELEPHONE_MSG_CALL_INCOMMING,
    TELEPHONE_MSG_CALL_OUTGOING,

    TELEPHONE_MSG_CALL_STATUS_IND,
    TELEPHONE_MSG_ANSWER_CALL,
    TELEPHONE_MSG_END_CALL,





    /*****Response messages*****/
    TELEPHONE_MSG_CALL_OUTGOING_RSP = RSP_MSG_TYPE | TELEPHONE_MSG_CALL_OUTGOING,
    TELEPHONE_MSG_ANSWER_CALL_RSP = RSP_MSG_TYPE | TELEPHONE_MSG_ANSWER_CALL,
    TELEPHONE_MSG_END_CALL_RSP = RSP_MSG_TYPE | TELEPHONE_MSG_END_CALL,
};


typedef enum
{
    CALL_ST_NONE,
    CALL_ST_INCOMMING,
    CALL_ST_OUTGOING,
    CALL_ST_ALERT,
    CALL_ST_ACTIVE,
    CALL_ST_END,
} call_status_t;



typedef union
{
    char callnum[32];
    call_status_t st;
} tel_srv_msg_t;


#endif  /* TELEPHONE_SERVICE */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
