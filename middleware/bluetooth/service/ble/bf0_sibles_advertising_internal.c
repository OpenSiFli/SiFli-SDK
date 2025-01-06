/**
  ******************************************************************************
  * @file   bf0_sibles_advertising_internal.c
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


#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "bf0_sibles_advertising.h"
#include "bf0_sibles_advertising_internal.h"

uint8_t sibles_advertising_data_flags_compose(uint8_t *adv_data, uint8_t *offset, uint8_t flags, uint8_t max_len)
{
    if ((*offset + SIBLES_ADV_TYPE_HEADER_LEN + 1) > max_len)
    {
        return SIBLES_ADV_DATA_LENGTH_EXCEED;
    }

    // Length
    adv_data[(*offset)++] = SIBLES_ADV_FLAG_FILED_LEN + 1;
    // Flag
    adv_data[(*offset)++] = BLE_GAP_AD_TYPE_FLAGS;
    // Value
    adv_data[(*offset)++] = flags;

    return SIBLES_ADV_NO_ERR;

}

uint8_t sibles_advertising_data_tx_pwr_compose(uint8_t *adv_data, uint8_t *offset, uint8_t tw_pwr_level, uint8_t max_len)
{
    if ((*offset + SIBLES_ADV_TYPE_HEADER_LEN + 1) > max_len)
    {
        return SIBLES_ADV_DATA_LENGTH_EXCEED;
    }

    // Length
    adv_data[(*offset)++] = SIBLES_ADV_FLAG_FILED_LEN + 1;
    // Flag
    adv_data[(*offset)++] = BLE_GAP_AD_TYPE_TRANSMIT_POWER;
    // Value
    adv_data[(*offset)++] = tw_pwr_level;

    return SIBLES_ADV_NO_ERR;

}

uint8_t sibles_advertising_data_apperance_compose(uint8_t *adv_data, uint8_t *offset, uint16_t appearance, uint8_t max_len)
{
    if ((*offset + SIBLES_ADV_TYPE_HEADER_LEN + 2) > max_len)
    {
        return SIBLES_ADV_DATA_LENGTH_EXCEED;
    }

    // Length
    adv_data[(*offset)++] = SIBLES_ADV_FLAG_FILED_LEN + 2;
    // Flag
    adv_data[(*offset)++] = BLE_GAP_AD_TYPE_APPEARANCE;
    // Value
    sibles_covert_to_raw_data((void *)&appearance, &adv_data[*offset], 2);
    *offset += 2;

    return SIBLES_ADV_NO_ERR;

}

uint8_t sibles_advertising_data_adv_interval_compose(uint8_t *adv_data, uint8_t *offset, uint16_t adv_interval, uint8_t max_len)
{
    if ((*offset + SIBLES_ADV_TYPE_HEADER_LEN + 2) > max_len)
    {
        return SIBLES_ADV_DATA_LENGTH_EXCEED;
    }

    // Length
    adv_data[(*offset)++] = SIBLES_ADV_FLAG_FILED_LEN + 2;
    // Flag
    adv_data[(*offset)++] = BLE_GAP_AD_TYPE_ADV_INTV;
    // Value
    sibles_covert_to_raw_data((void *)&adv_interval, &adv_data[*offset], 2);
    *offset += 2;

    return SIBLES_ADV_NO_ERR;

}

uint8_t sibles_advertising_data_local_name_compose(uint8_t *adv_data, uint8_t *offset, sibles_adv_type_name_t *name,
        uint8_t is_shorted, uint8_t max_len)
{
    if ((*offset + SIBLES_ADV_TYPE_HEADER_LEN + name->name_len) > max_len)
    {
        return SIBLES_ADV_DATA_LENGTH_EXCEED;
    }

    // Length
    adv_data[(*offset)++] = SIBLES_ADV_FLAG_FILED_LEN + name->name_len;
    // Flag
    adv_data[(*offset)++] = is_shorted == 1 ? BLE_GAP_AD_TYPE_SHORTENED_NAME : BLE_GAP_AD_TYPE_COMPLETE_NAME;
    // Value
    memcpy(&adv_data[*offset], name->name, name->name_len);
    *offset += name->name_len;

    return SIBLES_ADV_NO_ERR;
}




static uint8_t sibles_advertising_data_service_uuid_compose_int(uint8_t *adv_data, uint8_t *offset, sibles_adv_type_srv_uuid_t *uuid,
        uint8_t uuid_flag, uint8_t uuid_len, uint8_t is_completed, uint8_t max_len)
{

    uint8_t is_header_prepared = 0, total_len = 0, origin_offset = *offset;
    uint32_t i;


    for (i = 0; i < uuid->count; i++)
    {
        if (uuid->uuid_list[i].uuid_len == uuid_len)
        {
            uint8_t len = is_header_prepared == 1 ? 0 : SIBLES_ADV_TYPE_HEADER_LEN;
            if ((*offset + len + uuid_len) > max_len)
            {
                return SIBLES_ADV_DATA_LENGTH_EXCEED;
            }
            if (is_header_prepared == 0)
            {
                adv_data[++(*offset)] = uuid_flag;
                (*offset)++;
                is_header_prepared = 1;
            }
            memcpy(&adv_data[*offset], uuid->uuid_list[i].uuid.uuid_128, uuid_len);
            *offset += uuid_len;
            total_len += uuid_len;
        }
    }

    if (total_len)
    {
        adv_data[origin_offset] = SIBLES_ADV_FLAG_FILED_LEN + total_len;
    }

    return SIBLES_ADV_NO_ERR;

}


uint8_t sibles_advertising_data_service_uuid_compose(uint8_t *adv_data, uint8_t *offset, sibles_adv_type_srv_uuid_t *uuid,
        uint8_t is_completed, uint8_t max_len)
{

    uint8_t completed_flag[3] = {BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID, BLE_GAP_AD_TYPE_COMPLETE_LIST_32_BIT_UUID,
                                 BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID
                                };
    uint8_t incompleted_flag[3] = {BLE_GAP_AD_TYPE_MORE_16_BIT_UUID, BLE_GAP_AD_TYPE_MORE_32_BIT_UUID,
                                   BLE_GAP_AD_TYPE_MORE_128_BIT_UUID
                                  };

    uint8_t uuid_len[3] = {ATT_UUID_16_LEN, ATT_UUID_32_LEN, ATT_UUID_128_LEN};
    uint8_t *flag = is_completed == 1 ? completed_flag : incompleted_flag;
    uint32_t i;
    uint8_t ret;

    for (i = 0; i < 3; i ++)
    {
        ret = sibles_advertising_data_service_uuid_compose_int(adv_data, offset, uuid, flag[i], uuid_len[i],
                is_completed, max_len);
        if (ret != SIBLES_ADV_NO_ERR)
            break;
    }

    return ret;
}



uint8_t sibles_advertising_data_service_data_compose(uint8_t *adv_data, uint8_t *offset, sibles_adv_type_srv_data_t *uuid, uint8_t max_len)
{

    if ((*offset + SIBLES_ADV_TYPE_HEADER_LEN + uuid->uuid.uuid_len + uuid->data_len) > max_len)
    {
        return SIBLES_ADV_DATA_LENGTH_EXCEED;
    }

    // Length
    adv_data[(*offset)++] = SIBLES_ADV_FLAG_FILED_LEN + uuid->uuid.uuid_len + uuid->data_len;
    // Flag
    if (uuid->uuid.uuid_len == ATT_UUID_16_LEN)
    {
        adv_data[(*offset)++] = BLE_GAP_AD_TYPE_SERVICE_16_BIT_DATA;
        sibles_covert_to_raw_data(
            (void *)&uuid->uuid.uuid.uuid_16, &adv_data[*offset], uuid->uuid.uuid_len);
    }
    else if (uuid->uuid.uuid_len == ATT_UUID_32_LEN)
    {
        adv_data[(*offset)++] = BLE_GAP_AD_TYPE_SERVICE_32_BIT_DATA;
        sibles_covert_to_raw_data(
            (void *)&uuid->uuid.uuid.uuid_32, &adv_data[*offset], uuid->uuid.uuid_len);
    }
    else if (uuid->uuid.uuid_len == ATT_UUID_128_LEN)
    {
        adv_data[(*offset)++] = BLE_GAP_AD_TYPE_SERVICE_128_BIT_DATA;
        memcpy(&adv_data[*offset],
               (void *)&uuid->uuid.uuid.uuid_128, uuid->uuid.uuid_len);
    }

    *offset += uuid->uuid.uuid_len;
    // uuid
    memcpy(&adv_data[*offset], uuid->additional_data, uuid->data_len);
    *offset += uuid->data_len;

    return SIBLES_ADV_NO_ERR;

}

uint8_t sibles_advertising_data_conn_interval_compose(uint8_t *adv_data, uint8_t *offset,
        sibles_adv_type_conn_interval_t *conn_interval, uint8_t max_len)
{
    if ((*offset + SIBLES_ADV_TYPE_HEADER_LEN + sizeof(sibles_adv_type_conn_interval_t)) > max_len)
    {
        return SIBLES_ADV_DATA_LENGTH_EXCEED;
    }

    // Length
    adv_data[(*offset)++] = SIBLES_ADV_FLAG_FILED_LEN + sizeof(sibles_adv_type_conn_interval_t);
    // Flag
    adv_data[(*offset)++] = BLE_GAP_AD_TYPE_SLAVE_CONN_INT_RANGE;
    // Value
    memcpy(&adv_data[*offset], conn_interval, sizeof(sibles_adv_type_conn_interval_t));
    *offset += sizeof(sibles_adv_type_conn_interval_t);

    return SIBLES_ADV_NO_ERR;

}

uint8_t sibles_advertising_data_manufacturer_compose(uint8_t *adv_data, uint8_t *offset,
        sibles_adv_type_manufacturer_data_t *manufacturer_data, uint8_t max_len)
{
    if ((*offset + SIBLES_ADV_TYPE_HEADER_LEN + 2 + manufacturer_data->data_len) > max_len)
    {
        return SIBLES_ADV_DATA_LENGTH_EXCEED;
    }

    // Length
    adv_data[(*offset)++] = SIBLES_ADV_FLAG_FILED_LEN + 2 + manufacturer_data->data_len;
    // Flag
    adv_data[(*offset)++] = BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA;
    // Value
    sibles_covert_to_raw_data((void *)&manufacturer_data->company_id, &adv_data[*offset], 2);
    *offset += 2;
    memcpy(&adv_data[*offset], manufacturer_data->additional_data, manufacturer_data->data_len);
    *offset += manufacturer_data->data_len;

    return SIBLES_ADV_NO_ERR;

}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
