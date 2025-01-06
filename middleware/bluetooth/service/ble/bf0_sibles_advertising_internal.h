/**
  ******************************************************************************
  * @file   bf0_sibles_advertising_internal.h
  * @author Sifli software development team
  * @brief Header file - Sibles interface exposed by BCPU.
 *
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


#ifndef BF0_SIBLES_ADVERTISING_INTERNAL_H_
#define BF0_SIBLES_ADVERTISING_INTERNAL_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "bf0_sibles_internal.h"
#include "bf0_sibles_advertising.h"

enum sibles_adv_state_t
{
    SIBLES_ADV_STATE_IDLE,
    SIBLES_ADV_STATE_READY,
    SIBLES_ADV_STATE_STARTING,
    SIBLES_ADV_STATE_STARTED,
    SIBLES_ADV_STATE_STOPPING,
};

#define SIBLES_ADV_FAST_DURATION                    30
#define SIBLES_ADV_BACKGROUD_DURATION               180
#define SIBLES_MAX_ADV_DATA_LENGTH                  251
#define SIBLES_ADV_FLAG_FILED_LEN                   1
#define SIBLES_ADV_LEN_FILED_LEN                    1
#define SIBLES_ADV_TYPE_HEADER_LEN                  (SIBLES_ADV_FLAG_FILED_LEN + SIBLES_ADV_LEN_FILED_LEN)





uint8_t sibles_advertising_data_flags_compose(uint8_t *adv_data, uint8_t *offset,
        uint8_t flags, uint8_t max_len);

uint8_t sibles_advertising_data_tx_pwr_compose(uint8_t *adv_data, uint8_t *offset,
        uint8_t tw_pwr_level, uint8_t max_len);

uint8_t sibles_advertising_data_apperance_compose(uint8_t *adv_data, uint8_t *offset,
        uint16_t appearance, uint8_t max_len);

uint8_t sibles_advertising_data_adv_interval_compose(uint8_t *adv_data, uint8_t *offset,
        uint16_t adv_interval, uint8_t max_len);

uint8_t sibles_advertising_data_local_name_compose(uint8_t *adv_data, uint8_t *offset, sibles_adv_type_name_t *name,
        uint8_t is_shorted, uint8_t max_len);

uint8_t sibles_advertising_data_service_uuid_compose(uint8_t *adv_data, uint8_t *offset, sibles_adv_type_srv_uuid_t *uuid,
        uint8_t is_completed, uint8_t max_len);


uint8_t sibles_advertising_data_service_data_compose(uint8_t *adv_data, uint8_t *offset, sibles_adv_type_srv_data_t *uuid,
        uint8_t max_len);

uint8_t sibles_advertising_data_conn_interval_compose(uint8_t *adv_data, uint8_t *offset, sibles_adv_type_conn_interval_t *conn_interval,
        uint8_t max_len);



uint8_t sibles_advertising_data_manufacturer_compose(uint8_t *adv_data, uint8_t *offset, sibles_adv_type_manufacturer_data_t *manufacturer_data,
        uint8_t max_len);



#endif //BF0_SIBLES_ADVERTISING_INTERNAL_H_


