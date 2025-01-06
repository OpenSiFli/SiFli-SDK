/**
  ******************************************************************************
  * @file   hfp_audio.c
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

//#ifdef HFP_AUDION_INTERFACE
#include "bts2_global.h"
#include "board.h"

#ifdef AUDIO_USING_MANAGER

#include "hfp_type_api.h"
#include "drivers/bt_device.h"

#ifndef BSP_USING_PC_SIMULATOR
    #include "hfp_audio_api.h"
#endif

#define LOG_TAG         "audio_hfp"
//#define DBG_LVL          LOG_LVL_INFO
#include "log.h"


#ifdef BSP_USING_PC_SIMULATOR
void hfp_set_audio_voice_para(BTS2S_HF_AUDIO_INFO *msg)
{
}

void hfp_audio_init(void)
{
}

void hfp_audio_close_path(void)
{
}

void hfp_aduio_open_path(hfp_audio_type_t audioType)
{
}
#else

#ifndef BT_DBG_D
    #define BT_DBG_D(...) bt_log_output(BT_TAG_D, __VA_ARGS__)
#endif

/****************************************weak func start*************************************************/
__WEAK audio_client_t audio_open(audio_type_t audio_type, audio_rwflag_t rwflag,
                                 audio_parameter_t *paramter, audio_server_callback_func callback, void *callback_userdata)
{
    return (audio_client_t)bmalloc(300);
}

__WEAK int audio_write(audio_client_t handle, uint8_t *data, uint32_t data_len)
{
    return 0;
}

__WEAK int audio_read(audio_client_t handle, uint8_t *buf, uint32_t buf_size)
{
    return 0;
}

__WEAK int audio_ioctl(audio_client_t handle, int cmd, void *parameter)
{
    return 0;
}
__WEAK int audio_close(audio_client_t handle)
{

    if (handle)
        bfree(handle);
    return 0;
}

__WEAK void bt_tx_event_to_audio_server()
{
}
/****************************************weak func end*************************************************/

/****************************************global var start*************************************************/
static hfp_audio_env_t g_hfp_audio_env;
/****************************************global var end***************************************************/

/****************************************func define*************************************************/
static int hfp_audio_client_callback(audio_server_callback_cmt_t cmd, void *userdata, uint32_t unused)
{
    audio_type_t type = (audio_type_t)userdata;
    //BT_DBG_D("hfp_audio_client_callback cmd=%d type=%d\r\n", cmd, type);
    return 0;
}

void hfp_audio_init(void)
{
    g_hfp_audio_env.handle = NULL;
    hfp_audio_set_default_para();
}

void hfp_audio_set_default_para(void)
{
    g_hfp_audio_env.voice_para.codec = 0x00;
    g_hfp_audio_env.type_process = AUDIO_MANAGER_TYPE_INVALID;
    g_hfp_audio_env.audioStop = 0xff;
}

void hfp_set_audio_voice_para(BTS2S_HF_AUDIO_INFO *msg, BOOL audio_on, U8 direct_audio_on)
{
    BT_DBG_D("voice para on:%d supportInband %d", audio_on, direct_audio_on);
    if (audio_on)
    {
        g_hfp_audio_env.voice_para.codec = msg->air_mode;
        g_hfp_audio_env.voice_para.tsco = msg->tx_intvl;

        BT_DBG_D("airmode %d\r\n ", msg->air_mode);

        if (msg->air_mode == CVSD_MODE) // CVSD 8K
        {
            g_hfp_audio_env.voice_para.sample_rate = 8000;
        }
        else if (msg->rx_pkt_len == AUDIO_BT_VOICE_MSBC_IN_LEN) //mSBC 7.5ms
        {
            g_hfp_audio_env.voice_para.sample_rate = 16000;
        }
        else//mSBC 3.75ms
        {
            g_hfp_audio_env.voice_para.sample_rate = 16000;
        }

        if (AUDIO_MANAGER_TYPE_INVALID  == g_hfp_audio_env.type_process && direct_audio_on)
        {
            BT_DBG_D("sco open audio path codec: %d, sample_rate:%d", msg->air_mode, g_hfp_audio_env.voice_para.sample_rate);
            hfp_aduio_open_path(AUDIO_TYPE_BT_VOICE);
        }
    }
    else
    {
        hfp_audio_close_path();
    }

}

void hfp_aduio_open_path(hfp_audio_type_t audioType)
{
    if (AUDIO_MANAGER_TYPE_INVALID  != g_hfp_audio_env.type_process)
    {
        return;
    }

    BT_DBG_D("hfp2 open path atype %d sample_rate %x\r\n ", audioType, g_hfp_audio_env.voice_para.sample_rate);

    switch (audioType)
    {
    case AUDIO_TYPE_LOCAL_RING:
        //BT_DBG_D("local  ring.... todo: notify system UI");
        break;

    case AUDIO_TYPE_BT_VOICE:
    {
        audio_parameter_t param = {0};
        param.codec = g_hfp_audio_env.voice_para.codec;
        param.tsco = g_hfp_audio_env.voice_para.tsco;
        param.read_channnel_num = 1;
        param.read_bits_per_sample = 16;
        param.read_samplerate = g_hfp_audio_env.voice_para.sample_rate;
        param.write_bits_per_sample = 16;
        param.write_channnel_num = 1;
        param.write_samplerate = g_hfp_audio_env.voice_para.sample_rate;

        if (g_hfp_audio_env.handle)
        {
            BT_DBG_D("------------error, handle =0x%x", g_hfp_audio_env.handle);
            break;
        }
        g_hfp_audio_env.handle = audio_open(AUDIO_TYPE_BT_VOICE, AUDIO_TXRX,  &param, hfp_audio_client_callback, (void *)AUDIO_TYPE_BT_VOICE);
        BT_DBG_D("hfg audio_open=0x%x\r\n ", g_hfp_audio_env.handle);
        g_hfp_audio_env.type_process = AUDIO_TYPE_BT_VOICE;
        break;
    }
    default:
        ;
    }

}

void hfp_audio_close_path(void)
{
    int  res;
    BT_DBG_D("hfp_audio_close_path stop=%d \n", g_hfp_audio_env.audioStop);
    if (1 == g_hfp_audio_env.audioStop)
    {
        return;
    }

    BT_DBG_D("audio stop type proc %d handle=0x%x\r\n ", g_hfp_audio_env.type_process, g_hfp_audio_env.handle);

    if ((NULL != g_hfp_audio_env.handle) && (AUDIO_MANAGER_TYPE_INVALID != g_hfp_audio_env.type_process))
    {
        g_hfp_audio_env.audioStop = 1;
        res = audio_close(g_hfp_audio_env.handle);
        g_hfp_audio_env.handle = NULL;
    }
    else
    {
        BT_DBG_D("close path do nothing\r\n ");
        return;
    }

    BT_DBG_D("close path res %d\r\n", res);

    if (0 == res)
    {
        g_hfp_audio_env.type_process = AUDIO_MANAGER_TYPE_INVALID;
        g_hfp_audio_env.audioStop = 0xff;
        BT_DBG_D("close path done\r\n");
        hfp_audio_set_default_para();
    }
}
#endif



#define  HFP_AG_AUDIO_POOL_SIZE  0x7fff  //max ringbuffer size for rtthread
typedef struct
{
    audio_client_t       handle;
    BTS2S_HF_AUDIO_INFO  sco_inf;
    audio_device_input_callback input_cbk; //audio box no used
    struct rt_ringbuffer        ring_buf;
    uint8_t                     *ring_pool;
    uint32_t delay_cnt;
    uint32_t tx_cnt;
    uint8_t  tx_en;
    uint8_t  data_len;
} HFP_AG_AUDIO_INFO_T;

HFP_AG_AUDIO_INFO_T g_hfp_ag_audioinfo;
uint32_t g_audio_box_delay = 300;
static int bt_hfpag_register_audio_open(void *user_data, int (*input)(audio_server_callback_cmt_t cmd, const uint8_t *buffer, uint32_t size))
{
    HFP_AG_AUDIO_INFO_T *ag_inf = (HFP_AG_AUDIO_INFO_T *)user_data;
    LOG_I("bt_hfpag_register_audio_open\r\n");

    ag_inf->input_cbk = input;
    ag_inf->tx_en = 0;
    ag_inf->tx_cnt = 0;
    ag_inf->delay_cnt = g_audio_box_delay * 10 / 75;
    ag_inf->ring_pool   = audio_mem_calloc(1, HFP_AG_AUDIO_POOL_SIZE);
    RT_ASSERT(ag_inf->ring_pool);
    rt_ringbuffer_init(&(ag_inf->ring_buf), ag_inf->ring_pool, HFP_AG_AUDIO_POOL_SIZE);
    if (2 == ag_inf->sco_inf.air_mode)
    {
        ag_inf->data_len = 120;
        bt_voice_open(8000);
    }
    else if (3 == ag_inf->sco_inf.air_mode)
    {
        ag_inf->data_len = 240;
        bt_voice_open(16000);
    }
    else
    {
        RT_ASSERT(0);
    }
    return 0;
}

static int bt_hfpag_register_audio_close(void *user_data)
{
    HFP_AG_AUDIO_INFO_T *ag_inf = (HFP_AG_AUDIO_INFO_T *)user_data;
    LOG_I("bt_hfpag_register_audio_close\r\n");
    bt_voice_close();
    ag_inf->input_cbk = NULL;
    ag_inf->tx_en = 0;
    ag_inf->tx_cnt = 0;
    rt_ringbuffer_reset(&(ag_inf->ring_buf));
    audio_mem_free(ag_inf->ring_pool);
    return 0;
}

#if AUDIO_BOX_EN
static uint32_t bt_hfpag_register_audio_output(void *user_data, struct rt_ringbuffer *rb)
{
    HFP_AG_AUDIO_INFO_T *ag_inf = (HFP_AG_AUDIO_INFO_T *)user_data;
    struct rt_ringbuffer *ag_rb_cache = &(ag_inf->ring_buf);
    uint8_t buf[240];
    rt_size_t getsize, putsize;
    int ret;

    if (rt_ringbuffer_space_len(ag_rb_cache) >= ag_inf->data_len)
    {
        if (rt_ringbuffer_data_len(rb) >= ag_inf->data_len)
        {
            getsize = rt_ringbuffer_get(rb, &buf[0], ag_inf->data_len);
            RT_ASSERT(getsize == ag_inf->data_len);
            putsize = rt_ringbuffer_put(ag_rb_cache, &buf[0], ag_inf->data_len);
            RT_ASSERT(putsize == ag_inf->data_len);
            if (ag_inf->tx_en == 0)
            {
                ag_inf->tx_cnt++;
                ag_inf->tx_en = (ag_inf->tx_cnt > ag_inf->delay_cnt) ? 1 : 0;
            }
        }
        else
        {
            LOG_I("ag output callback ringbuf empty");
        }
    }
    else
    {
        LOG_I("aginf ringbuff full");
    }

    memset(&buf[0], 0, sizeof(buf));
    if (ag_inf->tx_en)
    {
        if (rt_ringbuffer_data_len(ag_rb_cache) >= ag_inf->data_len)
        {
            getsize = rt_ringbuffer_get(ag_rb_cache, &buf[0], ag_inf->data_len);
            RT_ASSERT(getsize == ag_inf->data_len);
        }
        else
        {
            LOG_I("ag tx ringbuf empty");
        }
    }
    ret = audio_hfp_uplink_write(ag_inf->handle, &buf[0], ag_inf->data_len);

    return ag_inf->data_len;
}
#else //for 4G
static uint32_t bt_hfpag_register_audio_output(void *user_data, struct rt_ringbuffer *rb)
{
    RT_ASSERT(0);//TODO: concatenate with 4G audio path
}
#endif


static const struct audio_device g_bt_hfpag_audio_dev =
{
    .open = bt_hfpag_register_audio_open,
    .close = bt_hfpag_register_audio_close,
    .user_data = (void *) &g_hfp_ag_audioinfo,
    .output = bt_hfpag_register_audio_output,
    .ioctl = NULL,
};


static int hfp_ag_audio_client_callback(audio_server_callback_cmt_t cmd, void *userdata, uint32_t reserved)
{
    LOG_I("hfp_ag_audio_client_callback cmd=%d \r\n", cmd);

    return 0;
}

void hfp_ag_audio_register(void)
{
    audio_server_register_audio_device(AUDIO_DEVICE_HFP, (struct audio_device *)&g_bt_hfpag_audio_dev);
}

void hfp_ag_audio_opt(BTS2S_HF_AUDIO_INFO *msg, BOOL audio_on)
{
    LOG_I("hfp_ag_audio_opt audio=%d %d\r\n", audio_on, g_hfp_ag_audioinfo.handle);
    if (audio_on)
    {
        audio_server_select_private_audio_device(AUDIO_TYPE_BT_VOICE, AUDIO_DEVICE_HFP);

        {
            audio_parameter_t param = {0};
            param.codec = msg->air_mode;
            param.tsco = msg->tx_intvl;
            param.read_channnel_num = 1;
            param.read_bits_per_sample = 16;
            param.write_bits_per_sample = 16;
            param.write_channnel_num = 1;
            if (msg->air_mode == 3)
            {
                param.read_samplerate = 16000;
                param.write_samplerate = 16000;
            }
            else if (msg->air_mode == 2)
            {
                param.read_samplerate = 8000;
                param.write_samplerate = 8000;
            }
            else
            {
                RT_ASSERT(0);
            }

            memcpy(&(g_hfp_ag_audioinfo.sco_inf), msg, sizeof(BTS2S_HF_AUDIO_INFO));

            RT_ASSERT(g_hfp_ag_audioinfo.handle == NULL);
            g_hfp_ag_audioinfo.handle = audio_open(AUDIO_TYPE_BT_VOICE, AUDIO_TXRX,  &param, hfp_ag_audio_client_callback, (void *)&g_hfp_ag_audioinfo);

        }
    }
    else
    {
        int res;

        if (g_hfp_ag_audioinfo.handle != NULL)
        {
            res = audio_close(g_hfp_ag_audioinfo.handle);
            RT_ASSERT(res == 0);
            g_hfp_ag_audioinfo.handle = NULL;
            audio_server_select_private_audio_device(AUDIO_TYPE_BT_VOICE, AUDIO_DEVICE_SPEAKER);
        }
    }
}

int abox_delay(int argc, char *argv[])
{
    rt_thread_t thread;

    if (argc != 2)
    {
        LOG_E("arg para num error: delay\n");
        return -1;
    }
    g_audio_box_delay = strtol(argv[1], NULL, 10);
    LOG_D("g_audio_box_delay=%d\n", g_audio_box_delay);

    return 0;
}

MSH_CMD_EXPORT(abox_delay,  set audio box delay);


#endif // AUDIO_USING_MANAGER
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
