/**
  ******************************************************************************
  * @file   bf0_sibles_eq_config.c
  * @author Sifli software development team
  * @brief Source file - Sibles EQ config.
 *
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2023 - 2023,  Sifli Technology
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


#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "stdlib.h"

#include "os_adaptor.h"
#include "bf0_sibles.h"
#include "bf0_sibles_internal.h"
#include "bf0_sibles_util.h"

#include "bf0_ble_gap.h"
#include "bf0_ble_common.h"
#include "bf0_sibles_serial_trans_service.h"
#include "bf0_sibles_eq_config.h"

#include "log.h"



#if defined(BLE_EQ_CONFIG_SERVER) && defined(BSP_ENABLE_AUD_PRC)
    #define EQ_CONFIG_SERVER
#endif

#if defined(EQ_CONFIG_SERVER) || defined(BLE_EQ_CONFIG_CLIENT)

#ifndef BLE_INVALID_CHANHDL
    #define BLE_INVALID_CHANHDL                      (0xFF)
#endif

#define EQ_U32(data) \
    (uint32_t)((uint32_t)*(data) | (uint32_t)*((data)+1) << 8 | (uint32_t)*((data)+2) << 16 | (uint32_t)*((data)+3) << 24)



typedef enum
{
    BLE_EQ_SET_ADC,
    BLE_EQ_GET_ADC,
    BLE_EQ_SET_DAC,
    BLE_EQ_GET_DAC,
    BLE_EQ_SET_MAX_DAC_LVL,
    BLE_EQ_GET_MAX_DAC_LVL,
    BLE_EQ_SET_VOL_LVL,
    BLE_EQ_GET_VOL_LVL,
    BLE_EQ_SET_DFT_VOL_LVL,
    BLE_EQ_GET_VER,
    BLE_EQ_SET_EQ_PARA,
    BLE_EQ_GET_EQ_PARA,
    BLE_EQ_SET_STATE,
    BLE_EQ_START,
    BLE_EQ_RSP,
} ble_eq_config_type_t;

typedef struct
{
    uint8_t handle;
    uint8_t conn_idx;
    uint16_t remote_handle;
    uint16_t val_hdl;
    uint16_t cccd_hdl;
} ble_eq_config_env_t;

static ble_eq_config_env_t g_eq_config_env;

const static uint8_t g_serial_tran_svc_uid[ATT_UUID_128_LEN] = serial_tran_svc_uuid;
const static uint8_t g_serial_tran_val_uid[ATT_UUID_128_LEN] = serial_tran_data_uuid;


static ble_eq_config_env_t *ble_eq_config_get_env(void)
{
    return &g_eq_config_env;
}

static uint8_t eq_pattern_check(uint8_t *data)
{
    uint8_t ret = 0;
    if (*data == 0xFF && *(data + 1) == 0xEE)
        ret = 1;

    return ret;

}



static eq_state_t ble_eq_config_send_cmd(uint8_t type, uint8_t len, uint8_t *data)
{
    eq_state_t ret = 1;
    if (data == NULL && len != 0)
        return ret;

    ble_eq_config_env_t *env = ble_eq_config_get_env();
    sibles_write_remote_value_t value;
    uint8_t *rsp_data = bt_mem_alloc(len + 7);

    if (rsp_data)
    {
        value.handle = env->val_hdl;
        value.write_type = SIBLES_WRITE_WITHOUT_RSP;
        value.len = len + 7;

        *rsp_data = BLE_EQ_CONFIG_CATEID;
        *(rsp_data + 1) = 0;
        *(rsp_data + 2) = (len + 7) & 0xFF;
        *(rsp_data + 3) = ((uint16_t)(len + 7) & 0xFF00) >> 8;
        *(rsp_data + 4) = 0xFF;
        *(rsp_data + 5) = 0xEE;
        *(rsp_data + 6) =  type;

        memcpy(rsp_data + 7, data, len);

        value.value = rsp_data;
        sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);
        bt_mem_free(rsp_data);
        ret = 0;
    }
    else
    {
        LOG_E("eq config cmd OOM");
        ret = 2;
    }
    return ret;
}


static eq_state_t ble_eq_config_send_rsp(uint8_t type, uint8_t ret, uint8_t len, uint8_t *data)
{
    eq_state_t ret = 1;
    if (data == NULL && len != 0)
        return ret;

    ble_eq_config_env_t *env = ble_eq_config_get_env();
    ble_serial_tran_data_t rsp;
    uint8_t *rsp_data = bt_mem_alloc(len + 5);
    if (rsp_data)
    {
        *rsp_data = 0xFF;
        *(rsp_data + 1) = 0xEE;
        *(rsp_data + 2) = BLE_EQ_RSP;
        *(rsp_data + 3) = type;
        *(rsp_data + 4) = ret;
        rsp.handle = env->handle;
        rsp.cate_id = BLE_EQ_CONFIG_CATEID;
        rsp.len = len + 5;
        memcpy(rsp_data + 5, data, len);
        rsp.data = rsp_data;
        ble_serial_tran_send_data(&rsp);
        bt_mem_free(rsp_data);
        ret = 0;
    }
    else
    {
        LOG_E("eq config rsp OOM");
        ret = 2;
    }

    return ret;
}




#ifdef EQ_CONFIG_SERVER

int8_t bf0_audprc_get_adc_vol(void);
void bf0_audprc_set_adc_vol(int8_t vol);
int8_t bf0_audprc_get_max_call_dac_vol(void);
void bf0_audprc_set_max_call_dac_vol(int8_t vol);
int8_t bf0_audprc_get_max_call_dac_vol_level(void);
void bf0_audprc_set_max_call_dac_vol_level(int8_t lvl);
int32_t bf0_audprc_get_call_dac_vol_level(uint8_t idx, int8_t *lvl);
int32_t bf0_audprc_set_call_dac_vol_level(uint8_t idx, int8_t lvl);
int32_t bf0_audprc_set_default_call_dac_vol_level(uint8_t idx, int8_t lvl);
int8_t bf0_audprc_get_max_music_dac_vol(void);
void bf0_audprc_set_max_music_dac_vol(int8_t vol);
int8_t bf0_audprc_get_max_music_dac_vol_level(void);
void bf0_audprc_set_max_music_dac_vol_level(int8_t lvl);
int32_t bf0_audprc_get_music_dac_vol_level(uint8_t idx, int8_t *lvl);
int32_t bf0_audprc_set_music_dac_vol_level(uint8_t idx, int8_t lvl);
int32_t bf0_audprc_set_default_music_dac_vol_level(uint8_t idx, int8_t lvl);
void bf0_audprc_get_eq_code_ver(char *ver, uint8_t len);
int32_t bf0_audprc_set_music_eq(uint8_t idx, uint8_t *data, uint8_t len);
int32_t bf0_audprc_get_music_eq(uint8_t idx, uint8_t *data, uint8_t len);
int32_t bf0_audprc_set_voice_state(uint8_t state);
int32_t bf0_audprc_set_music_state(uint8_t state);

int32_t bf0_audprc_start_eq(uint8_t is_start);

int32_t bf0_audprc_get_voice_eq(uint8_t idx, uint8_t *data, uint8_t len);
int32_t bf0_audprc_set_voice_eq(uint8_t idx, uint8_t *data, uint8_t len);



static eq_state_t ble_eq_set_adc_server(int8_t val)
{
    bf0_audprc_set_adc_vol(val);
    return 0;
}

static eq_state_t ble_eq_get_adc_server(int8_t *val)
{
    eq_state_t ret = 0;
    if (val)
        *val = bf0_audprc_get_adc_vol();
    else
        ret = 2;
    return 0;
}

static eq_state_t ble_eq_set_dac_server(uint8_t type, int8_t val)
{
    eq_state_t ret = 0;
    if (type == 0)
        bf0_audprc_set_max_call_dac_vol(val);
    else if (type == 1)
        bf0_audprc_set_max_music_dac_vol(val);
    else
        ret = 1;
    return ret;
}

static eq_state_t ble_eq_get_dac_server(uint8_t type, int8_t *val)
{
    eq_state_t ret = 0;

    if (val == NULL)
        return 2;

    if (type == 0)
        *val = bf0_audprc_get_max_call_dac_vol();
    else if (type == 1)
        *val = bf0_audprc_get_max_music_dac_vol();
    else
        ret = 1;
    return ret;
}


static eq_state_t ble_eq_set_max_dac_level_server(uint8_t type, int8_t val)
{
    eq_state_t ret = 0;
    if (type == 0)
        bf0_audprc_set_max_call_dac_vol_level(val);
    else if (type == 1)
        bf0_audprc_set_max_music_dac_vol_level(val);
    else
        ret = 1;
    return ret;
}

static eq_state_t ble_eq_get_max_dac_level_server(uint8_t type, int8_t *val)
{
    eq_state_t ret = 0;

    if (val == NULL)
        return 2;

    if (type == 0)
        *val = bf0_audprc_get_max_call_dac_vol_level();
    else if (type == 1)
        *val = bf0_audprc_get_max_music_dac_vol_level();
    else
        ret = 1;
    return ret;

}


static eq_state_t ble_eq_set_volume_lvl_via_idx_server(uint8_t type, uint8_t idx, int8_t val)
{
    eq_state_t ret = 0;
    if (type == 0)
    {
        if (bf0_audprc_set_call_dac_vol_level(idx, val) != 0)
            ret = 3;
    }
    else if (type == 1)
    {
        if (bf0_audprc_set_music_dac_vol_level(idx, val) != 0)
            ret = 3;
    }
    else
        ret = 1;
    return ret;
}

static eq_state_t ble_eq_get_volume_lvl_via_idx_server(uint8_t type, uint8_t idx, int8_t *val)
{
    eq_state_t ret = 0;

    if (val == NULL)
        return 2;

    if (type == 0)
    {
        if (bf0_audprc_get_call_dac_vol_level(idx, val) != 0)
            ret = 3;
    }
    else if (type == 1)
    {
        if (bf0_audprc_get_music_dac_vol_level(idx, val) != 0)
            ret = 3;
    }
    else
        ret = 1;
    return ret;
}

static eq_state_t ble_eq_set_default_volume_server(uint8_t type, uint8_t idx, int8_t val)
{
    eq_state_t ret = 0;
    if (type == 0)
    {
        if (bf0_audprc_set_default_call_dac_vol_level(idx, val) != 0)
            ret = 3;
    }
    else if (type == 1)
    {
        if (bf0_audprc_set_default_music_dac_vol_level(idx, val) != 0)
            ret = 3;
    }
    else
        ret = 1;
    return ret;
}


static eq_state_t ble_eq_get_version_server(uint8_t *ver, uint8_t len)
{
    bf0_audprc_get_eq_code_ver((char *)ver, len);
    return 0;
}

static eq_state_t ble_eq_set_parameter_server(uint8_t type, uint8_t idx, uint8_t *para, uint8_t len)
{
    eq_state_t ret = 0;
    if (para == NULL)
        return 2;

    if (type == 1)
    {
        if (bf0_audprc_set_music_eq(idx, para, len) != 0)
            ret = 3;
    }
    else if (type == 0)
    {
        if (bf0_audprc_set_voice_eq(idx, para, len) != 0)
            ret = 3;
    }
    else
        ret = 1;

    return ret;
}

static eq_state_t ble_eq_get_parameter_server(uint8_t type, uint8_t idx, uint8_t *para, uint8_t len)
{
    eq_state_t ret = 0;
    if (para == NULL)
        return 2;

    if (type == 1)
    {
        if (bf0_audprc_get_music_eq(idx, para, len) != 0)
            ret = 3;
    }
    else if (type == 0)
    {
        if (bf0_audprc_get_voice_eq(idx, para, len) != 0)
            ret = 3;
    }
    else
        ret = 1;

    return ret;

}

static eq_state_t ble_eq_set_state_server(uint8_t type, uint8_t state)
{
    eq_state_t ret = 0;
    if (type == 0)
    {
        if (bf0_audprc_set_voice_state(state) != 0)
            ret = 3;
    }
    else if (type == 1)
    {
        if (bf0_audprc_set_music_state(state) != 0)
            ret = 3;
    }
    else
        ret = 1;


    return ret;
}

static eq_state_t ble_eq_start_server(uint8_t is_start)
{
    eq_state_t ret = 0;
    if (bf0_audprc_start_eq(is_start) != 0)
        ret = 3;

    return ret;
}
#endif // EQ_CONFIG_SERVER


#ifdef BLE_EQ_CONFIG_CLIENT
static void ble_eq_rsp_handler(uint8_t *para)
{
    uint8_t type = *para;
    uint8_t ret = *(para + 1);
    uint8_t *data = para + 2;
    switch (type)
    {
    case BLE_EQ_SET_ADC:
    {
        if (ret == 0)
            LOG_I("aprc_debug_vol_w success");
        break;
    }
    case BLE_EQ_GET_ADC:
    {
        if (ret == 0)
        {
            int8_t val = *(data);
            LOG_I("aprc_debug_vol %d\n", val);
        }
        break;
    }
    case BLE_EQ_SET_DAC:
    {
        if (ret == 0)
            LOG_I("aprc_debug_vol_w success");
        break;
    }
    case BLE_EQ_GET_DAC:
    {
        if (ret == 0)
        {
            int8_t val = *(data + 1);
            LOG_I("aprc_debug_vol %d\n", val);
        }
        break;
    }
    case BLE_EQ_SET_MAX_DAC_LVL:
    {
        if (ret == 0)
            LOG_I("aprc_debug_vol_w success");
        break;
    }
    case BLE_EQ_GET_MAX_DAC_LVL:
    {
        if (ret == 0)
        {
            int8_t val = *(data + 1);
            LOG_I("aprc_debug_vol %d\n", val);
        }
        break;
    }
    case BLE_EQ_SET_VOL_LVL:
    {
        if (ret == 0)
        {
            LOG_I("aprc_debug_vol_w success");
        }
        break;
    }
    case BLE_EQ_GET_VOL_LVL:
    {
        if (ret == 0)
        {
            int8_t val = *(data + 1);
            LOG_I("aprc_debug_vol %d\n", val);
        }
        break;
    }
    case BLE_EQ_SET_DFT_VOL_LVL:
    {
        if (ret == 0)
        {
            LOG_I("aprc_debug_vol_w success");
        }
        break;
    }
    case BLE_EQ_GET_VER:
    {
        if (ret == 0)
        {
            LOG_I("aprc_debug_version %s\n", data);
        }
        break;
    }
    case BLE_EQ_SET_EQ_PARA:
    {
        if (ret == 0)
        {
            LOG_I("aprc_debug set eq success");
        }
        break;
    }
    case BLE_EQ_GET_EQ_PARA:
    {
        if (ret == 0)
        {
            uint8_t *val = data + 1;
            LOG_I("value:0x%08x,0x%08x,0x%08x,0x%08x,0x%08x",
                  EQ_U32(val),
                  EQ_U32(val + 4),
                  EQ_U32(val + 8),
                  EQ_U32(val + 12),
                  EQ_U32(val + 16));
        }
        break;
    }
    case BLE_EQ_SET_STATE:
    {

        break;
    }
    case BLE_EQ_START:
    {

        break;
    }
    default:
        break;
    }

    if (ret != 0)
    {
        LOG_I("aprc_rw fail");
    }
    else
    {
        LOG_I("aprc_rw success");
    }
}



#endif // BLE_EQ_CONFIG_CLIENT





void ble_eq_config_serial_callback(uint8_t event, uint8_t *data)
{
    if (!data)
        return;

    ble_eq_config_env_t *env = ble_eq_config_get_env();
    switch (event)
    {
    case BLE_SERIAL_TRAN_OPEN:
    {
        ble_serial_open_t *open = (ble_serial_open_t *)data;

        // only set for 1st device
        if (env->handle == BLE_INVALID_CHANHDL)
        {
            env->handle = open->handle;
        }
    }
    break;
    case BLE_SERIAL_TRAN_DATA:
    {
        ble_serial_tran_data_t *t_data = (ble_serial_tran_data_t *)data;
        eq_state_t ret;
        if (env->handle == t_data->handle && t_data->cate_id == BLE_EQ_CONFIG_CATEID)
        {
            if (eq_pattern_check(t_data->data))
            {
                // pattern is 2bytes
                uint8_t type = *(t_data->data + 2);
                uint8_t *para = t_data->data + 3;
                switch (type)
                {
#ifdef EQ_CONFIG_SERVER
                case BLE_EQ_SET_ADC:
                {
                    int8_t val = *para;
                    ret = ble_eq_set_adc_server(val);
                    ble_eq_config_send_rsp(BLE_EQ_SET_ADC, ret, 0, NULL);
                    break;
                }
                case BLE_EQ_GET_ADC:
                {
                    //uint8_t
                    int8_t val;
                    ret = ble_eq_get_adc_server(&val);
                    ble_eq_config_send_rsp(BLE_EQ_GET_ADC, ret, sizeof(val), (uint8_t *) &val);
                    break;
                }
                case BLE_EQ_SET_DAC:
                {
                    uint8_t type = *para;
                    int8_t val = *(para + 1);
                    ret = ble_eq_set_dac_server(type, val);
                    ble_eq_config_send_rsp(BLE_EQ_SET_DAC, ret, sizeof(type), &type);
                    break;
                }
                case BLE_EQ_GET_DAC:
                {
                    uint8_t type = *para;
                    int8_t val;
                    uint8_t rsp[2];
                    ret = ble_eq_get_dac_server(type, &val);
                    rsp[0] = type;
                    rsp[1] = (uint8_t)val;
                    ble_eq_config_send_rsp(BLE_EQ_GET_DAC, ret, sizeof(rsp), (uint8_t *)&rsp);
                    break;
                }
                case BLE_EQ_SET_MAX_DAC_LVL:
                {
                    uint8_t type = *para;
                    int8_t val = *(para + 1);
                    ret = ble_eq_set_max_dac_level_server(type, val);
                    ble_eq_config_send_rsp(BLE_EQ_SET_MAX_DAC_LVL, ret, sizeof(type), &type);
                    break;
                }
                case BLE_EQ_GET_MAX_DAC_LVL:
                {
                    uint8_t type = *para;
                    int8_t val;
                    uint8_t rsp[2];
                    ret = ble_eq_get_max_dac_level_server(type, &val);
                    rsp[0] = type;
                    rsp[1] = (uint8_t)val;
                    ble_eq_config_send_rsp(BLE_EQ_GET_MAX_DAC_LVL, ret, sizeof(rsp), (uint8_t *) &rsp);
                    break;
                }
                case BLE_EQ_SET_VOL_LVL:
                {
                    uint8_t type = *para;
                    uint8_t idx = *(para + 1);
                    int8_t val = *(para + 2);
                    ret = ble_eq_set_volume_lvl_via_idx_server(type, idx, val);
                    ble_eq_config_send_rsp(BLE_EQ_SET_VOL_LVL, ret, sizeof(type), &type);
                    break;
                }
                case BLE_EQ_GET_VOL_LVL:
                {
                    uint8_t type = *para;
                    uint8_t idx = *(para + 1);
                    int8_t val;
                    uint8_t rsp[2];
                    ret = ble_eq_get_volume_lvl_via_idx_server(type, idx, &val);
                    rsp[0] = type;
                    rsp[1] = (uint8_t)val;
                    ble_eq_config_send_rsp(BLE_EQ_GET_VOL_LVL, ret, sizeof(rsp), (uint8_t *)&rsp);
                    break;
                }
                case BLE_EQ_SET_DFT_VOL_LVL:
                {
                    uint8_t type = *para;
                    uint8_t idx = *(para + 1);
                    int8_t val = *(para + 2);
                    ret = ble_eq_set_default_volume_server(type, idx, val);
                    ble_eq_config_send_rsp(BLE_EQ_SET_DFT_VOL_LVL, ret, sizeof(type), &type);
                    break;
                }
                case BLE_EQ_GET_VER:
                {
                    char ver[10];
                    ret = ble_eq_get_version_server((uint8_t *)ver, 10);
                    ble_eq_config_send_rsp(BLE_EQ_GET_VER, ret, sizeof(ver), (uint8_t *)&ver);
                    break;
                }
                case BLE_EQ_SET_EQ_PARA:
                {
                    uint8_t type = *para;
                    uint8_t idx = *(para + 1);
                    uint8_t len = *(para + 2);
                    uint8_t *data = para + 3;
                    ret = ble_eq_set_parameter_server(type, idx, data, len);
                    ble_eq_config_send_rsp(BLE_EQ_SET_EQ_PARA, ret, sizeof(type), &type);
                    break;
                }
                case BLE_EQ_GET_EQ_PARA:
                {
                    uint8_t type = *para;
                    uint8_t idx = *(para + 1);
                    uint8_t rsp[21];
                    ret = ble_eq_get_parameter_server(type, idx, (uint8_t *)&rsp[1], 20);
                    rsp[0] = type;
                    ble_eq_config_send_rsp(BLE_EQ_GET_EQ_PARA, ret, sizeof(rsp), (uint8_t *)&rsp);
                    break;
                }
                case BLE_EQ_SET_STATE:
                {
                    uint8_t type = *para;
                    uint8_t state = *(para + 1);
                    ret = ble_eq_set_state_server(type, state);
                    ble_eq_config_send_rsp(BLE_EQ_SET_STATE, ret, 0, NULL);
                    break;
                }
                case BLE_EQ_START:
                {
                    uint8_t is_start = *para;
                    ret = ble_eq_start_server(is_start);
                    ble_eq_config_send_rsp(BLE_EQ_START, ret, 0, NULL);
                    break;
                }
#endif //EQ_CONFIG_SERVER
#ifdef BLE_EQ_CONFIG_CLIENT
                case BLE_EQ_RSP:
                {
                    ble_eq_rsp_handler(para);
                }
                break;
#endif // BLE_EQ_CONFIG_CLIENT
                default:
                    break;
                }
            }
        }
    }
    break;
    case BLE_SERIAL_TRAN_CLOSE:
    {
        ble_serial_close_t *close = (ble_serial_close_t *)data;
        if (env->handle == close->handle)
        {
            env->handle = BLE_INVALID_CHANHDL;
        }
    }
    break;
    default:
        break;
    }


}


BLE_SERIAL_TRAN_EXPORT(BLE_EQ_CONFIG_CATEID, ble_eq_config_serial_callback);

#ifdef BLE_EQ_CONFIG_CLIENT

eq_state_t ble_eq_set_adc(int8_t val)
{
    return ble_eq_config_send_cmd(BLE_EQ_SET_ADC, sizeof(val), (uint8_t *)&val);
}

eq_state_t ble_eq_get_adc(void)
{
    return ble_eq_config_send_cmd(BLE_EQ_GET_ADC, 0, NULL);
}

eq_state_t ble_eq_set_dac(uint8_t type, int8_t val)
{
    eq_state_t ret = 0;
    if (type > 1)
        ret = 1;
    else
    {
        uint8_t req[2];
        req[0] = type;
        req[1] = (uint8_t)val;
        ret = ble_eq_config_send_cmd(BLE_EQ_SET_DAC, sizeof(req), (uint8_t *)&req);
    }
    return ret;
}

eq_state_t ble_eq_get_dac(uint8_t type)
{
    eq_state_t ret = 0;

    if (type > 1)
        ret = 1;
    else
        ret = ble_eq_config_send_cmd(BLE_EQ_GET_DAC, sizeof(type), &type);

    return ret;
}


eq_state_t ble_eq_set_max_dac_level(uint8_t type, int8_t val)
{
    eq_state_t ret = 0;
    if (type > 1)
        ret = 1;
    else
    {
        uint8_t req[2];
        req[0] = type;
        req[1] = (uint8_t)val;
        ret = ble_eq_config_send_cmd(BLE_EQ_SET_MAX_DAC_LVL, sizeof(req), (uint8_t *)&req);
    }

    return ret;
}

eq_state_t ble_eq_get_max_dac_level(uint8_t type)
{
    eq_state_t ret = 0;
    if (type > 1)
        ret = 1;
    else
        ret = ble_eq_config_send_cmd(BLE_EQ_GET_MAX_DAC_LVL, sizeof(type), &type);

    return ret;

}


eq_state_t ble_eq_set_volume_lvl_via_idx(uint8_t type, uint8_t idx, int8_t val)
{
    eq_state_t ret = 0;
    if (type > 1 || idx > 15)
        ret = 1;
    else
    {
        uint8_t req[3];
        req[0] = type;
        req[1] = idx;
        req[2] = (uint8_t)val;
        ret = ble_eq_config_send_cmd(BLE_EQ_SET_VOL_LVL, sizeof(req), (uint8_t *)&req);
    }

    return ret;
}

eq_state_t ble_eq_get_volume_lvl_via_idx(uint8_t type, uint8_t idx)
{
    eq_state_t ret = 0;

    if (type > 1 || idx > 15)
        ret = 1;
    else
    {
        uint8_t req[2];
        req[0] = type;
        req[1] = idx;
        ret = ble_eq_config_send_cmd(BLE_EQ_GET_VOL_LVL, sizeof(req), (uint8_t *) &req);
    }

    return ret;
}

eq_state_t ble_eq_set_default_volume(uint8_t type, uint8_t idx, int8_t val)
{
    eq_state_t ret = 0;

    if (type > 1 || idx > 15)
        ret = 1;
    else
    {
        uint8_t req[3];
        req[0] = type;
        req[1] = idx;
        req[2] = (uint8_t)val;
        ret = ble_eq_config_send_cmd(BLE_EQ_SET_DFT_VOL_LVL, sizeof(req), (uint8_t *)&req);
    }

    return ret;
}


eq_state_t ble_eq_get_version(void)
{
    return ble_eq_config_send_cmd(BLE_EQ_GET_VER, 0, NULL);
}

eq_state_t ble_eq_set_parameter(uint8_t type, uint8_t idx, uint8_t *para, uint8_t len)
{
    eq_state_t ret = 0;

    if (type > 1 || idx > 15 || len != 20)
        ret = 1;
    else
    {
        uint8_t *req = bt_mem_alloc(3 + len);
        if (req)
        {
            req[0] = type;
            req[1] = idx;
            req[2] = len;
            memcpy(req + 3, para, len);
            ret = ble_eq_config_send_cmd(BLE_EQ_SET_EQ_PARA, 3 + len, (uint8_t *)req);
            bt_mem_free(req);
        }
        else
        {
            LOG_E("eq set para OOM");
            ret = 2;
        }
    }

    return ret;
}

eq_state_t ble_eq_get_parameter(uint8_t type, uint8_t idx)
{
    eq_state_t ret = 0;

    if (type > 1 || idx > 15)
        ret = 1;
    else
    {
        uint8_t req[2];
        req[0] = type;
        req[1] = idx;
        ret = ble_eq_config_send_cmd(BLE_EQ_GET_EQ_PARA, sizeof(req), (uint8_t *)&req);
    }


    return ret;

}

eq_state_t ble_eq_set_state(uint8_t type, uint8_t state)
{
    uint8_t req[2];
    req[0] = type;
    req[1] = state;
    return ble_eq_config_send_cmd(BLE_EQ_SET_STATE, sizeof(req), (uint8_t *)&req);

}

eq_state_t ble_eq_start(uint8_t is_start)
{
    return ble_eq_config_send_cmd(BLE_EQ_START, sizeof(is_start), &is_start);
}


void ble_eq_register(uint8_t conn_idx)
{
    ble_eq_config_env_t *env = ble_eq_config_get_env();
    LOG_I("eq register");
    env->conn_idx = conn_idx;
    sibles_search_service(conn_idx, ATT_UUID_128_LEN, (uint8_t *)g_serial_tran_svc_uid);
}


static void ble_eq_serial_tran_parser_and_callback(sibles_remote_event_ind_t *ind)
{
    uint8_t *data = ind->value;
    LOG_HEX("seral_dump", 16, data, ind->length);
    uint8_t cate_id = *data;
    if (cate_id == BLE_EQ_CONFIG_CATEID)
    {
        uint8_t flag = *(data + 1);
        if (flag == 0)
        {
            uint16_t length = (uint16_t)((uint16_t) * (data + 2)) | (((uint16_t) * (data + 3)) << 8);
            if (eq_pattern_check(data + 4))
            {
                // pattern is 2bytes
                uint8_t type = *(data + 6);
                uint8_t *para = data + 7;
                if (type == BLE_EQ_RSP)
                    ble_eq_rsp_handler(para);
            }
        }
    }
}

static int ble_eq_gattc_event_handler(uint16_t event_id, uint8_t *data, uint16_t len)
{
    ble_eq_config_env_t *env = ble_eq_config_get_env();
    int8_t  res;

    switch (event_id)
    {
    case SIBLES_REGISTER_REMOTE_SVC_RSP:
    {
        sibles_register_remote_svc_rsp_t *rsp = (sibles_register_remote_svc_rsp_t *)data;
        LOG_I("register ret %d\r\n", rsp->status);
        sibles_write_remote_value_t value;
        uint16_t enable = 1;
        value.handle = env->cccd_hdl;
        value.write_type = SIBLES_WRITE;
        value.len = 2;
        value.value = (uint8_t *)&enable;
        res = sibles_write_remote_value(env->remote_handle, env->conn_idx, &value);
        OS_ASSERT(res == SIBLES_WRITE_NO_ERR);
        break;
    }
    case SIBLES_REMOTE_EVENT_IND:
    {
        sibles_remote_event_ind_t *ind = (sibles_remote_event_ind_t *)data;
        LOG_I("eq handle:%d", ind->handle);
        if (ind->handle == env->val_hdl)
        {
            ble_eq_serial_tran_parser_and_callback(ind);
        }
        // Notify upper layer
        break;
    }
    default:
        break;
    }
    return 0;
}


int ble_eq_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context)
{
    ble_eq_config_env_t *env = ble_eq_config_get_env();
    switch (event_id)
    {

    case SIBLES_WRITE_VALUE_RSP:
    {
        sibles_write_value_rsp_t *rsp = (sibles_write_value_rsp_t *)data;
        LOG_I("SIBLES_WRITE_VALUE_RSP %d", rsp->result);
        break;
    }
    case SIBLES_SEARCH_SVC_RSP:
    {
        sibles_svc_search_rsp_t *rsp = (sibles_svc_search_rsp_t *)data;

        // rsp->svc may null
        if (memcmp(rsp->search_uuid, g_serial_tran_svc_uid, rsp->search_svc_len) != 0)
            break;

        if (rsp->result != HL_ERR_NO_ERROR)
        {
            LOG_W("search failed");
            break; // Do nothing
        }

        uint32_t i;
        uint8_t data_check = 0;
        uint16_t offset = 0;
        sibles_svc_search_char_t *chara = (sibles_svc_search_char_t *)rsp->svc->att_db;
        for (i = 0; i < rsp->svc->char_count; i++)
        {
            if (!memcmp(chara->uuid, g_serial_tran_val_uid, chara->uuid_len))
            {
                LOG_W("char found");
                env->val_hdl = chara->pointer_hdl;
                env->cccd_hdl = chara->desc[0].attr_hdl;
                data_check = 1;
                break;
            }
            offset = sizeof(sibles_svc_search_char_t) + chara->desc_count * sizeof(struct sibles_disc_char_desc_ind);
            chara = (sibles_svc_search_char_t *)((uint8_t *)chara + offset);
        }
        if (data_check != 1)
        {
            LOG_W("char not found");
            break;
        }
        LOG_I("EQ search successful");
        //register first
        env->remote_handle = sibles_register_remote_svc(rsp->conn_idx, rsp->svc->hdl_start, rsp->svc->hdl_end, ble_eq_gattc_event_handler);

        break;
    }
    default:
        break;
    }
    return 0;
}
BLE_EVENT_REGISTER(ble_eq_event_handler, NULL);
#endif // BLE_EQ_CONFIG_CLIENT





#endif // defined(EQ_CONFIG_SERVER) || defined(BLE_EQ_CONFIG_CLIENT)





/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
