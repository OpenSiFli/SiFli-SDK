/**
  ******************************************************************************
  * @file   bts2_app_av_snk.c
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

#include "rtthread.h"
#include "bts2_app_inc.h"

#ifdef CFG_AV_SNK


#ifdef CFG_AV_AAC
    #include <neaacdec.h>
#endif

#ifdef RT_USING_BT
    #include "bt_rt_device.h"
#endif
#include "bt_connection_manager.h"
#include "log.h"
#if defined(AUDIO_USING_MANAGER) && defined(AUDIO_BT_AUDIO)
    #include "audio_server.h"
#endif

uint8_t   bts2s_avsnk_openFlag;//0x00:dont open a2dp profile; 0x01:open a2dp profile;
uint8_t   frms_per_payload;

extern bts2_app_stru *bts2g_app_p;


//#include "windows.h"


#define BEGIN_ACCESS_BUF()              rt_sem_take(bt_av_get_inst_data()->snk_data.buf_sem, RT_WAITING_FOREVER)
#define END_ACCESS_BUF()                rt_sem_release(bt_av_get_inst_data()->snk_data.buf_sem)





#ifdef CFG_AV_AAC
    static NeAACDecHandle hAac;
#endif


#define  BIT_RATE_DEAFLUT                  (324)
#define  SINK_DATA_LIST_START_THRESHOLD    (5)
#define  SINK_DATA_LIST_MAX_THRESHOLD      (10)





#if defined(AUDIO_USING_MANAGER) && defined(AUDIO_BT_AUDIO)

static rt_event_t g_playback_evt;
static rt_thread_t g_playback_thread = NULL;
#define  PLAYBACK_GETDATA_EVENT_FLAG       (1 << 0)
#define  PLAYBACK_START_EVENT_FLAG         (1 << 1)
#define  PLAYBACK_STOPPING_EVENT_FLAG      (1 << 2)
#define  PLAYBACK_STOPPED_EVENT_FLAG       (1 << 3)


static uint8_t list_push_back(play_list_t *list, list_hdr_t *hdr)
{
    uint8_t ret;

    RT_ASSERT(hdr != NULL);

    BEGIN_ACCESS_BUF();

    if (list->first == NULL)
    {
        list->first = hdr;
    }
    else
    {
        list->last->next = hdr;
    }
    list->last = hdr;
    hdr->next = NULL;
    list->cnt++;
    ret = (list->cnt >= list->cnt_th) ? 1 : 0;

    //RT_ASSERT(list->cnt <= 100);
    if (list->cnt > SINK_DATA_LIST_MAX_THRESHOLD)
    {
        //USER_TRACE("list->cnt= %d\n", list->cnt);
        list->full_num++;
        ret = 2;
    }
    list->total_num++;
    END_ACCESS_BUF();

    return ret;
}

static list_hdr_t *list_pop_front(play_list_t *list)
{
    list_hdr_t *hdr;

    BEGIN_ACCESS_BUF();

    hdr = list->first;
    if (hdr != NULL)
    {
        list->first = list->first->next;
        if (list->first == NULL)
        {
            list->last = NULL;
        }

        list->cnt--;
    }
    else
    {
        list->empty_num++;
    }

    END_ACCESS_BUF();

    return hdr;
}

static void list_all_free(play_list_t *list)
{
    list_hdr_t *hdr;

    USER_TRACE("all free:%x,cnt:%d\n", list->first, list->cnt);

    while (list->first)
    {
        hdr = list_pop_front(list);
        bfree(hdr);
    }
    list->empty_num = 0;
    list->full_num = 0;
    list->total_num = 0;

}


static int audio_bt_music_client_cb(audio_server_callback_cmt_t cmd, void *userdata, uint32_t unused)
{
    (void)userdata;
    (void)unused;
    if (cmd == as_callback_cmd_cache_empty || cmd == as_callback_cmd_cache_half_empty)
    {
        rt_event_send(g_playback_evt, PLAYBACK_GETDATA_EVENT_FLAG);
    }

    return 0;
}

static U8 *play_data_decode(bts2s_av_inst_data *inst, U16 *out_len)
{
    U8 *frm_ptr, *data, *media_pkt_ptr;
    U16 bytes_left;
    BTS2S_SBC_STREAM bss;
    U8 *ret = NULL;
    play_data_t *pt_data;

    pt_data = (play_data_t *)list_pop_front(&(inst->snk_data.playlist));
    inst->snk_data.pt_curdata = pt_data;

    *out_len = 0;
    data = (U8 *)pt_data;

    if (pt_data == NULL)
    {
        return ret;
    }

    if (pt_data->len <= (AV_FIXED_MEDIA_PKT_HDR_SIZE + 1))
    {
        USER_TRACE("a2dp play decode len:%d\n", pt_data->len);
        bfree(pt_data);
        return ret;
    }

    if (inst->snk_data.codec == AV_SBC)
    {
        frm_ptr = data + AV_FIXED_MEDIA_PKT_HDR_SIZE + 1;
        media_pkt_ptr = data + AV_FIXED_MEDIA_PKT_HDR_SIZE;
        bytes_left = pt_data->len - AV_FIXED_MEDIA_PKT_HDR_SIZE - 1;
        bts2_sbc_cfg *cfg = &inst->con[inst->con_idx].act_cfg;

        if (((*media_pkt_ptr) & 0x0f) > frms_per_payload)
        {
            frms_per_payload = (*media_pkt_ptr) & 0x0f;

            if (inst->snk_data.decode_buf)
            {
                bfree(inst->snk_data.decode_buf);
                inst->snk_data.decode_buf = NULL;
            }

            if (inst->snk_data.decode_buf == NULL)
            {
                U16 decode_buffer_size;
                U16 pcm_samples_per_sbc_frame;

                pcm_samples_per_sbc_frame = bts2_sbc_calculate_pcm_samples_per_sbc_frame(cfg->blocks, cfg->subbands);
                decode_buffer_size = pcm_samples_per_sbc_frame * frms_per_payload * 2;
                USER_TRACE("frms_per_payload = %d,pcm_samples_per_sbc_frame = %d,decode_buffer_size = %d\n",
                           frms_per_payload, pcm_samples_per_sbc_frame, decode_buffer_size);
                inst->snk_data.decode_buf = bmalloc(decode_buffer_size);
                inst->snk_data.decode_buf_len = decode_buffer_size;
            }
            BT_OOM_ASSERT(inst->snk_data.decode_buf);
        }

        bss.dst_len = inst->snk_data.decode_buf_len;
        bss.pdst = inst->snk_data.decode_buf;
        bss.src_len = bytes_left;
        bss.psrc = frm_ptr;

        bts2_sbc_decode(&bss);

        ret = inst->snk_data.decode_buf;
        *out_len = bss.dst_len_used;
        RT_ASSERT(bss.src_len_used == bss.src_len);
    }
    else if (inst->snk_data.codec == AV_MPEG24_AAC)
    {
#ifdef CFG_AV_AAC
        U32 t1 = rt_system_get_time();
        U32 t2;
        frm_ptr = data + AV_FIXED_MEDIA_PKT_HDR_SIZE;
        bytes_left = pt_data->len - AV_FIXED_MEDIA_PKT_HDR_SIZE;

        const unsigned char *fin = frm_ptr;
        int fileread = bytes_left;
        char *faad_id_string = NULL;
        char *faad_copyright_string = NULL;
        NeAACDecFrameInfo frameInfo;
        NeAACDecConfigurationPtr config;
        unsigned long samplerate;
        unsigned char channels;
        void *sample_buffer;

#if 0
        if (0 == NeAACDecGetVersion(&faad_id_string, &faad_copyright_string))
        {
        }
        unsigned long cap = NeAACDecGetCapabilities();
#endif
        // Check if decoder has the needed capabilities
        // Open the library
        char err = NeAACDecInit(hAac, (unsigned char *)fin, fileread, &samplerate,
                                &channels);
        if (err != 0)
        {
            RT_ASSERT(0);
        }
        unsigned long frame_index = 0;
        frameInfo.bytesconsumed = 0;


        // Only one frame
        sample_buffer = NeAACDecDecode(hAac, &frameInfo, (unsigned char *)fin,  fileread);

        fileread -= frameInfo.bytesconsumed;

        if (frameInfo.error != 0)
        {
            USER_TRACE("decode error %d\n", frameInfo.error);
        }

        if (fileread != 0) // < 2048 is safe if, not a 100% frame data at tail
        {
            USER_TRACE("size not enough %d\n", fileread);
        }


        if (frameInfo.error == 0 && fileread == 0)
        {
            ret = sample_buffer;
            *out_len = 2 * frameInfo.samples;
        }


        t2 = rt_system_get_time();
#endif
        //USER_TRACE("ex t %d(%d, %d)\n", t2 - t1, t2, t1);
    }
    else
    {
        RT_ASSERT(0);
    }

    bfree(pt_data);

    return ret;
}


static void decode_playback_thread(void *args)
{
    bts2s_av_inst_data *inst_data;

    rt_uint32_t evt;
    play_data_t *pt_data;
    U8 *decode_data = NULL;
    U16 decode_len = 0;
    U8  is_stopped = 1;
    U8  debug_tx_cnt = 0;
    int  ret_write = 0;
    g_playback_evt = rt_event_create("playback_evt", RT_IPC_FLAG_FIFO);

    while (1)
    {
        evt = 0;
        rt_err_t err = rt_event_recv(g_playback_evt, PLAYBACK_GETDATA_EVENT_FLAG | PLAYBACK_START_EVENT_FLAG | PLAYBACK_STOPPING_EVENT_FLAG, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        inst_data = bt_av_get_inst_data();
        if (evt & PLAYBACK_STOPPING_EVENT_FLAG)
        {
            is_stopped = 1;
            rt_event_send(g_playback_evt, PLAYBACK_STOPPED_EVENT_FLAG);
            continue;
        }

        if (evt & PLAYBACK_START_EVENT_FLAG)
        {
            if (inst_data->snk_data.audio_client)
            {
                USER_TRACE("bt_music: open again--\r\n");
                continue;
            }

            decode_data = play_data_decode(inst_data, &decode_len);
            //interval = decode_len * 1000 / BT_MUSIC_SAMPLERATE / 4; //use interval to rt_event_recv will crash rt_free
            USER_TRACE("bt_music: open len=%d\r\n", decode_len);

            USER_TRACE("sbc decode src_len:%d, dst_len:%d\n", inst_data->snk_data.pt_curdata->len, decode_len);


            audio_parameter_t param = {0};
            param.write_samplerate = inst_data->con[inst_data->con_idx].act_cfg.sample_freq;
            param.write_channnel_num = 2;
            param.write_bits_per_sample = 16;
            param.write_cache_size = 8192;
            debug_tx_cnt = 0;
            inst_data->snk_data.audio_client = audio_open(AUDIO_TYPE_BT_MUSIC, AUDIO_TX, &param, audio_bt_music_client_cb, NULL);
            is_stopped = 0;
        }
        if (evt & PLAYBACK_GETDATA_EVENT_FLAG)
        {
            U16 decode_len_old = decode_len;

            if (debug_tx_cnt == 0)
            {
                USER_TRACE("a2dp get data,total:%d,full:%d,empty:%d, curr %d\r\n", inst_data->snk_data.playlist.total_num,
                           inst_data->snk_data.playlist.full_num, inst_data->snk_data.playlist.empty_num, inst_data->snk_data.playlist.cnt);
            }
            debug_tx_cnt++;

            if (is_stopped == 1 || inst_data->snk_data.play_state == FALSE || inst_data->snk_data.audio_client == NULL)
            {
                USER_TRACE("snk: stop %d %d %x\r\n", is_stopped, inst_data->snk_data.play_state, inst_data->snk_data.audio_client);
                continue;
            }
        }

        if (decode_len == 0)
        {
            //decode_data = play_data_decode(inst_data, &decode_len);
            decode_len = 2560;
            decode_data = inst_data->snk_data.decode_buf;
            memset(decode_data, 0, decode_len);
        }

        while (decode_len > 0)
        {
            ret_write = audio_write(inst_data->snk_data.audio_client, decode_data, decode_len);
            if (ret_write < 0)
            {
                USER_TRACE("playback write ret:%d\n", ret_write);
                break;
            }
            else if (ret_write == 0)
            {
                break;
            }
            else
            {
                decode_data = play_data_decode(inst_data, &decode_len);
            }
        }

    }
}
#if defined(SF32LB52X)

    static rt_uint8_t deplayback_thread_stack[1024];
    static struct rt_thread deplayback_thread;
#endif
static int audio_decode_thread_init(void)
{

#if defined(SF32LB52X)
    rt_thread_init(&deplayback_thread,
                   "deplayback_th",
                   decode_playback_thread,
                   RT_NULL,
                   &deplayback_thread_stack[0],
                   sizeof(deplayback_thread_stack),
                   RT_THREAD_PRIORITY_HIGH,
                   RT_THREAD_TICK_DEFAULT);
    g_playback_thread = &deplayback_thread;
#else
    g_playback_thread = rt_thread_create("deplayback_th", decode_playback_thread, NULL, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);

#endif
    if (g_playback_thread)
    {
        rt_thread_startup(g_playback_thread);
    }
    else
    {
        USER_TRACE("deplayback_th thread fail \n");
    }


    return 0;
}
INIT_PRE_APP_EXPORT(audio_decode_thread_init);


static void stop_audio_playback(bts2s_av_inst_data *inst)
{
    rt_uint32_t evt;

    USER_TRACE("stop_audio_playback state:%d\n", inst->snk_data.play_state);
    if (inst->snk_data.play_state == TRUE)
    {
        rt_event_send(g_playback_evt, PLAYBACK_STOPPING_EVENT_FLAG);
        rt_event_recv(g_playback_evt, PLAYBACK_STOPPED_EVENT_FLAG, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        inst->snk_data.play_state = FALSE;
#if defined(AUDIO_USING_MANAGER) && defined(AUDIO_BT_AUDIO)
        audio_close(inst->snk_data.audio_client);
        inst->snk_data.audio_client = NULL;
#endif
    }

    list_all_free(&(inst->snk_data.playlist));

    if (inst->snk_data.decode_buf)
    {
        bfree(inst->snk_data.decode_buf);
        inst->snk_data.decode_buf = NULL;
    }

}

static void stop_audio_playback_temporarily(bts2s_av_inst_data *inst)
{
    rt_uint32_t evt;

    USER_TRACE("stop_audio_playback_temporarily state:%d\n", inst->snk_data.play_state);
    if (inst->snk_data.play_state == TRUE)
    {
        rt_event_send(g_playback_evt, PLAYBACK_STOPPING_EVENT_FLAG);
        rt_event_recv(g_playback_evt, PLAYBACK_STOPPED_EVENT_FLAG, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        inst->snk_data.play_state = FALSE;
#if defined(AUDIO_USING_MANAGER) && defined(AUDIO_BT_AUDIO)
        audio_close(inst->snk_data.audio_client);
        inst->snk_data.audio_client = NULL;
#endif
    }

    list_all_free(&(inst->snk_data.playlist));

}
#endif

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_avsnk_conn_2_src(BTS2S_BD_ADDR *bd)
{
    BTS2S_BD_ADDR temp = {0xffffff, 0xff, 0xffff};
    bt_err_t res = BT_EOK;

    if (!bd_eq(bd, &temp))
    {
        bt_av_conn(bd, AV_SNK);
        USER_TRACE(">> av snk connect\n");
    }
    else
    {
        res = BT_ERROR_INPARAM;
        USER_TRACE(">> pls input remote device address\n");
    }

    return res;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_avsnk_disc_2_src(BOOL is_close)
{
    bts2s_av_inst_data *inst_data;
    inst_data = bt_av_get_inst_data();
    if (is_close)
    {
        inst_data->close_pending = TRUE;
    }
    for (uint32_t i = 0; i < MAX_CONNS; i++)
    {
        if (inst_data->con[i].cfg == AV_AUDIO_SNK)
            bt_av_disconnect(i);
    }
}

void bt_avsnk_disc_2_src_by_addr(BTS2S_BD_ADDR *bd_addr, BOOL is_close)
{
    bts2s_av_inst_data *inst_data;
    inst_data = bt_av_get_inst_data();
    if (is_close)
    {
        inst_data->close_pending = TRUE;
    }

    for (uint32_t i = 0; i < MAX_CONNS; i++)
    {
        if (bd_eq(bd_addr, &inst_data->con[i].av_rmt_addr))
        {
            bt_av_disconnect(i);
            break;
        }
    }
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_avsnk_suspend_stream(bts2_app_stru *bts2_app_data)
{
    for (uint32_t i = 0; i < MAX_CONNS; i++)
        bt_av_suspend_stream(i);
}
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_avsnk_start_stream(bts2_app_stru *bts2_app_data)
{
    for (uint32_t i = 0; i < MAX_CONNS; i++)
        bt_av_start_stream(i);
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_avsnk_release_stream(bts2_app_stru *bts2_app_data)
{
    for (uint32_t i = 0; i < MAX_CONNS; i++)
        bt_av_release_stream(i);
}

#if 0
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2s_avsnk_inst_data *inst:
 *
 * OUTPUT:
 *      int.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
static int bt_avsnk_get_plyback_conn(bts2s_avsnk_inst_data *inst)
{
    U8 found = FALSE;
    U8 i = 0;
    int res = - 1;

    for (i = 0; i < MAX_CONNS; i++)
    {
        if ((inst->local_seid_info[inst->con[i].local_seid_idx].local_seid.sep_type == AV_SNK) &&
                (inst->local_seid_info[inst->con[i].local_seid_idx].local_seid.in_use == TRUE))
        {
            found = TRUE;
            break;
        }
    }
    if (found)
    {
        res = i;
    }
    return res;
}
#endif
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_sbc_cfg *cfg:
 *      U8 *sbc_ie
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
static void bt_avsnk_init_data(bts2s_avsnk_inst_data *inst, bts2_app_stru *bts2_app_data)
{
    inst->playlist.cnt = 0;
    inst->playlist.cnt_th = SINK_DATA_LIST_START_THRESHOLD;
    inst->playlist.first = NULL;
    inst->playlist.last = NULL;
    inst->play_state = FALSE;
    inst->can_play = 1;
    inst->filter_prompt_enable = 1;
    inst->reveive_start = 0;
#ifndef RT_USING_UTEST
    inst->slience_filter_enable = 1;
    inst->slience_count = 0;
#endif
#if defined(AUDIO_USING_MANAGER) && defined(AUDIO_BT_AUDIO)
    audio_client_t audio_client;
#endif
    inst->decode_buf = NULL;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      U16 r1:
 *      U8 r2:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
static U8 bt_avsnk_role_type_equal(U16 r1, U8 r2)
{
    U8 res;

    if ((r1 == AV_AUDIO_SRC) && (r2 == AV_SRC))
    {
        res = TRUE;
    }
    else if ((r1 == AV_AUDIO_SNK) && (r2 == AV_SNK))
    {
        res = TRUE;
    }
    else
    {
        res = FALSE;
    }
    return res;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      U8 con_idx:
 *
 * OUTPUT:
 *      U8.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 bt_avsnk_prepare_sbc(bts2s_av_inst_data *inst, U8 con_idx)
{
    U8 res;
    U8 bit_pool_alt;
    U8 toggle_period;
    bts2_sbc_cfg *cfg = &inst->con[con_idx].act_cfg;
    cfg->bit_pool = bts2_sbc_calc_bit_pool(&bit_pool_alt,
                                           &toggle_period,
                                           cfg->chnl_mode,
                                           cfg->sample_freq,
                                           cfg->blocks,
                                           cfg->subbands,
                                           BIT_RATE_DEAFLUT);

    if (cfg->bit_pool > cfg->max_bitpool)
    {
        cfg->bit_pool = cfg->max_bitpool;
    }

    res = bts2_sbc_decode_cfg(cfg->chnl_mode,
                              cfg->alloc_method,
                              cfg->sample_freq,
                              cfg->blocks,
                              cfg->subbands,
                              cfg->bit_pool);
    return res;
}

U16 bt_avsnk_calculate_decode_buffer_size(bts2s_av_inst_data *inst, U8 con_idx)
{
    bts2_sbc_cfg *cfg = &inst->con[con_idx].act_cfg;
    U16 sbc_frame_size;
    U16 decode_buffer_size;
    U16 pcm_samples_per_sbc_frame;

    sbc_frame_size = bts2_sbc_calculate_framelen(cfg->chnl_mode,
                     cfg->blocks,
                     cfg->subbands,
                     cfg->bit_pool);
    frms_per_payload = (U16)((AV_MTU_SIZE - 14) / sbc_frame_size + 1);
    pcm_samples_per_sbc_frame = bts2_sbc_calculate_pcm_samples_per_sbc_frame(cfg->blocks, cfg->subbands);
    decode_buffer_size = pcm_samples_per_sbc_frame * frms_per_payload * 2;
    USER_TRACE("sbc_frame_size = %d,pcm_samples_per_sbc_frame = %d,decode_buffer_size = %d\n",
               sbc_frame_size, pcm_samples_per_sbc_frame, decode_buffer_size);
    return decode_buffer_size;
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_avsnk_hdl_disc_handler(bts2s_av_inst_data *inst, uint8_t con_idx)
{

#if defined(AUDIO_USING_MANAGER) && defined(AUDIO_BT_AUDIO)
    stop_audio_playback(inst);
#endif
    bts2_sbc_decode_completed();

}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
int8_t bt_avsnk_hdl_start_cfm(bts2s_av_inst_data *inst, uint8_t con_idx)
{
    int8_t ret = -1;
    if (!bt_avsnk_prepare_sbc(inst, con_idx))
        ret = 0;
    return ret;
}
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/

void bt_avsnk_close_handler(bts2s_av_inst_data *inst, uint8_t con_idx)
{

}

// extern void bt_hid_reset_num_count_drag_down(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
uint8_t bt_avsnk_hdl_start_ind(bts2s_av_inst_data *inst, BTS2S_AV_START_IND *msg, uint8_t con_idx)
{
    U8 res;
    U8 i;
    U8 shdl = 0;
    U8 codec;

    codec = inst->local_seid_info[inst->con[con_idx].local_seid_idx].local_seid.codec;

    res = AV_ACPT;

    for (i = 0; i < msg->list_len; i++)
    {
        shdl = *(&msg->first_shdl + i);

        if (shdl == inst->con[con_idx].stream_hdl)
        {
            if (inst->con[con_idx].st == avconned_open)
            {
                if (codec == AV_SBC)
                {
                    if (!bt_avsnk_prepare_sbc(inst, con_idx))
                    {
                        break;
                    }
                }
                else
                {
                    //TODO: AAC handle
#ifdef CFG_AV_AAC
                    hAac = NeAACDecOpen();
                    // Get the current config
                    NeAACDecConfigurationPtr conf =
                        NeAACDecGetCurrentConfiguration(hAac);
                    conf->defObjectType = LC;
                    conf->outputFormat = FAAD_FMT_16BIT;
                    conf->downMatrix = 1;
                    conf->useOldADTSFormat = 0;
                    // If needed change some of the values in conf
                    //
                    // Set the new configuration
                    NeAACDecSetConfiguration(hAac, conf);
#endif
                }
            }
            else
            {
                res = AV_BAD_ST;
                break;
            }
        }
        else
        {
            res = AV_BAD_ACP_SEID;
            break;
        }
    }

    if ((res == AV_ACPT) && (i == msg->list_len))
    {
        USER_TRACE(">> accept to start the stream\n");
// #ifdef CFG_HID
//         bt_hid_reset_num_count_drag_down();
// #endif

        inst->snk_data.play_rd_idx = inst->snk_data.play_wr_idx = 0;
        inst->snk_data.codec = codec;
#if defined(AUDIO_USING_MANAGER) && defined(AUDIO_BT_AUDIO)
        list_all_free(&(inst->snk_data.playlist));
        if (inst->snk_data.decode_buf == NULL)
        {
            U16 decode_buffer_size;
            decode_buffer_size = bt_avsnk_calculate_decode_buffer_size(inst, con_idx);
            USER_TRACE("decode_buffer_size = %d\n", decode_buffer_size);
            inst->snk_data.decode_buf = bmalloc(decode_buffer_size);
            inst->snk_data.decode_buf_len = decode_buffer_size;
        }
        BT_OOM_ASSERT(inst->snk_data.decode_buf);
#endif
        inst->snk_data.reveive_start = 1;

#ifdef CFG_AVRCP
        bts2_app_stru *bts2_app_data = bts2g_app_p;
        if (bts2_app_data->avrcp_inst.playback_status != 0x01)
        {
            inst->snk_data.can_play = 0;
        }
        else if (bts2_app_data->avrcp_inst.playback_status == 0x01)
        {
            inst->snk_data.can_play = 1;
            inst->snk_data.reveive_start = 0;
        }
        else
        {

        }
#endif
    }

    return res;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_avsnk_hdl_suspend_ind(bts2s_av_inst_data *inst, uint8_t con_idx)
{

    U8 codec = inst->local_seid_info[inst->con[con_idx].local_seid_idx].local_seid.codec;
    USER_TRACE(">> accept to suspend the av stream\n");
    inst->snk_data.reveive_start = 0;
#if defined(AUDIO_USING_MANAGER) && defined(AUDIO_BT_AUDIO)
    stop_audio_playback(inst);
#endif

#ifdef CFG_AV_AAC
    if (codec == AV_MPEG24_AAC)
        NeAACDecClose(hAac);
#endif
}
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_avsnk_hdl_streamdata_ind(bts2s_av_inst_data *inst, uint8_t con_idx, BTS2S_AV_STREAM_DATA_IND *msg)
{
    int ret = -1;
    U8 *frm_ptr;
    U8 codec;
    U16 sbc_frame_size;

    codec = inst->local_seid_info[inst->con[con_idx].local_seid_idx].local_seid.codec;


    if (bt_av_get_slience_filter_enable())
    {
        if (msg->len <= (AV_FIXED_MEDIA_PKT_HDR_SIZE + 1))
        {
            USER_TRACE("a2dp play decode len:%d\n", msg->len);
            bfree(msg->data);
            return;
        }

        frm_ptr = msg->data + AV_FIXED_MEDIA_PKT_HDR_SIZE + 1;
        U8 bitpool = *(frm_ptr + 2);;
        bts2_sbc_cfg *cfg = &inst->con[con_idx].act_cfg;

        sbc_frame_size = bts2_sbc_calculate_framelen(cfg->chnl_mode,
                         cfg->blocks,
                         cfg->subbands,
                         bitpool);

        if (0 == memcmp(frm_ptr, frm_ptr + sbc_frame_size, sbc_frame_size))
        {
            inst->snk_data.slience_count++;
            // USER_TRACE("sbc mute count = %d\n",inst->snk_data.slience_count);
            if (inst->snk_data.slience_count > SINK_DATA_LIST_MAX_THRESHOLD + 2)
            {
                if (inst->snk_data.play_state  == TRUE)
                {
#if defined(AUDIO_USING_MANAGER) && defined(AUDIO_BT_AUDIO)
                    stop_audio_playback_temporarily(inst);
#endif
                }
                bfree(msg->data);
                return;
            }
            else
            {
                if (inst->snk_data.play_state  == FALSE)
                {
#if defined(AUDIO_USING_MANAGER) && defined(AUDIO_BT_AUDIO)
                    list_all_free(&(inst->snk_data.playlist));
#endif
                    bfree(msg->data);
                    return;
                }
            }
        }
        else
        {
            inst->snk_data.slience_count = 0;
        }
    }

    {
#if defined(AUDIO_USING_MANAGER) && defined(AUDIO_BT_AUDIO)
        play_data_t *pt_data = (play_data_t *)msg->data;
        uint8_t ret;

        pt_data->len = msg->len;
        ret = list_push_back(&inst->snk_data.playlist, &(pt_data->hdr));
        if ((inst->snk_data.play_state == FALSE) && (ret == 1))
        {
            inst->snk_data.play_state = TRUE;
            USER_TRACE("av_snk.c open a2dp\r\n");
            rt_event_send(g_playback_evt, PLAYBACK_START_EVENT_FLAG);
        }
        else if (ret == 2)
        {
            list_hdr_t *hdr;
            hdr = list_pop_front(&inst->snk_data.playlist);
            bfree(hdr);
        }
#else
        bfree(msg->data);
#endif
    }


}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_avsnk_init(bts2s_av_inst_data *inst, bts2_app_stru *bts2_app_data)
{
    bt_avsnk_init_data(&inst->snk_data, bts2_app_data);

    //inst->play_handle = NULL;
    inst->snk_data.buf_sem = rt_sem_create("bt_av_sink", 1, RT_IPC_FLAG_FIFO);
    //inst->audio_len = 0;

#ifdef CFG_OPEN_AVSNK
    bts2s_avsnk_openFlag = 1;
#else
    bts2s_avsnk_openFlag = 0;
#endif

    if (1 == bts2s_avsnk_openFlag)
    {
        INFO_TRACE(">> AV sink enabled\n");
        av_enb_req(inst->que_id, AV_AUDIO_SNK); //act the svc
    }
}

void bt_avsnk_open(bts2s_av_inst_data *inst)
{
    USER_TRACE("bt_avsnk_open %d flag\n", bts2s_avsnk_openFlag);
    if (0 == bts2s_avsnk_openFlag)
    {
        bts2s_avsnk_openFlag = 0x01;
        av_enb_req(inst->que_id, AV_AUDIO_SNK); //act the svc
    }
    else
    {

        bt_interface_bt_event_notify(BT_NOTIFY_A2DP, BT_NOTIFY_AVSNK_OPEN_COMPLETE, NULL, 0);
        INFO_TRACE(">> have open,ura AV sink open\n");
    }
}
void bt_avsnk_close(bts2s_av_inst_data *inst)
{
    USER_TRACE("bt_avsnk_close %d flag\n", bts2s_avsnk_openFlag);
    if (0x01 == bts2s_avsnk_openFlag)
    {
        bts2s_avsnk_openFlag = 0x00;
        av_disb_req(); //disable the svc
    }
    else
    {
        bt_interface_bt_event_notify(BT_NOTIFY_A2DP, BT_NOTIFY_AVSNK_CLOSE_COMPLETE, NULL, 0);
        INFO_TRACE(">> have close,urc AV sink close\n");
    }
}

void bt_avsnk_rel(bts2s_av_inst_data *inst)
{
    rt_sem_delete(inst->snk_data.buf_sem);
}

#if 0
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
static void bt_avsnk_hdl_sts_ind(bts2_app_stru *bts2_app_data)
{
    BTS2S_AV_STS_IND *msg;

    msg = (BTS2S_AV_STS_IND *)bts2_app_data->recv_msg;
    switch (msg->st_type)
    {
    case BTS2MU_AV_ENB_CFM:
        bt_avsnk_hdl_enb_cfm(bts2_app_data);
        break;

    case BTS2MU_AV_DISB_CFM:
        //hdl_AV_DISB_CFM(bts2_app_data);
        break;

    case BTS2MU_AV_CONN_CFM:
        bt_avsnk_hdl_conn_cfm(bts2_app_data);
        break;

    case BTS2MU_AV_CONN_IND:
        bt_avsnk_hdl_conn_ind(bts2_app_data);
        break;

    default:
        break;
    }
}
#endif

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_avsnk_msg_handler(bts2_app_stru *bts2_app_data)
{
    U16 *msg_type;
    msg_type = (U16 *)bts2_app_data->recv_msg;
    switch (*msg_type)
    {
    case BTS2MU_AV_SECU_CTRL_CFM:
    {
        INFO_TRACE("<< receive av sercurity control confirmation \n");
        break;
    }
    case BTS2MU_AV_SECU_CTRL_IND:
    {
        INFO_TRACE("<< receive av security control indication\n");
        break;
    }
#if 0
    case BTS2MU_AV_STS_IND:
    {
        bt_avsnk_hdl_sts_ind(bts2_app_data);
        break;
    }
#endif
    default:
        break;
    }
}

#if 0
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_avsnk_recfg(bts2_app_stru *bts2_app_data)
{
    bts2s_avsnk_inst_data *inst;
    U8 app_serv_cap[SBC_MEDIA_CODEC_SC_SIZE];
    U8 i;
    inst = global_inst;
    if (inst->con[inst->con_idx].st == avconned_open)
    {
        app_serv_cap[0] = AV_SC_MEDIA_CODEC;
        app_serv_cap[1] = SBC_MEDIA_CODEC_SC_SIZE - 2;
        app_serv_cap[2] = AV_AUDIO << 4;
        app_serv_cap[3] = AV_SBC;
        for (i = 0; i < SBC_MEDIA_CODEC_SC_SIZE - 4; i++)
        {
            app_serv_cap[4 + i] = avsnk_sbc_cfg[1][i];/*choose the other sbc code mothod*/
        }
        bt_avsnk_store_sbc_cfg(&inst->con[inst->con_idx].act_cfg, app_serv_cap + 4);
        USER_TRACE(">> reset the configuration of the stream\n");
        av_recfg_req(inst->con[inst->con_idx].stream_hdl, ASSIGN_TLABEL, SBC_MEDIA_CODEC_SC_SIZE, app_serv_cap);
    }
    else
    {
        if (inst->con[inst->con_idx].st == avconned_streaming)
        {
            USER_TRACE(" -- the stream is streaming, please supped it!\n");
        }
        else
        {
            USER_TRACE(" -- the stream is not in use!\n");
        }
    }
}
#endif
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_avsnk_abort_stream(bts2_app_stru *bts2_app_data)
{
    for (uint32_t i = 0; i < MAX_CONNS; i++)
        bt_av_abort_stream(i);
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_avsnk_get_cfg(bts2_app_stru *bts2_app_data)
{
    for (uint32_t i = 0; i < MAX_CONNS; i++)
        bt_av_get_cfg(i);
}


#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
