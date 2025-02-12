/**
  ******************************************************************************
  * @file   bts2_app_av_src.c
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
#include "ipc/ringbuffer.h"
#ifdef CFG_AV_SRC

#define LOG_TAG         "audio_av_src"
//#define DBG_LVL          LOG_LVL_INFO
#include "log.h"

#define MAX_FRM_PER_PAYLOAD 10

//#define AUDIO_DATA_TEST
//#define AUDIO_DATA_FROM_PSRAM 1
#ifdef AUDIO_DATA_TEST

    #if AUDIO_DATA_FROM_PSRAM
        #define INPUT_ADDRESS  0x60100000
        #define INPUT_END_ADDRESS  0x60400000
        static uint8_t *datain, *dataend;
    #endif

    static FILE *stream;
#endif // AUDIO_DATA_TEST

#ifdef AUDIO_USING_MANAGER
    #include "audio_server.h"
#endif // AUDIO_USING_MANAGER


#if defined(AUDIO_DATA_TEST)
    static void bt_avsrc_send_data(U16 m, void *bts2_app_data);
#elif defined(AUDIO_USING_MANAGER)
    #define A2DP_CMD_PAUSE          0
    #define A2DP_CMD_RESUME         1

    static uint16_t bt_avsrc_send_data(struct rt_ringbuffer *rb);
    static int bt_avsrc_register_audio_open(void *user_data, audio_device_input_callback callback);
    static int bt_avsrc_register_audio_close(void *user_data);
    static uint32_t bt_avsrc_register_audio_output(void *user_data, struct rt_ringbuffer *rb);
    static int bt_avsrc_register_audio_ioctl(void *user_data, int cmd, void *val);
#endif


#ifdef AUDIO_USING_MANAGER
static const struct audio_device g_bt_avsrc_audio_dev =
{
    .open = bt_avsrc_register_audio_open,
    .close = bt_avsrc_register_audio_close,
    .user_data = NULL,
    .output = bt_avsrc_register_audio_output,
    .ioctl = bt_avsrc_register_audio_ioctl,
};


static int bt_avsrc_register_audio_open(void *user_data, int (*input)(audio_server_callback_cmt_t cmd, const uint8_t *buffer, uint32_t size))
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    inst->src_data.input_cb = input;
    inst->src_data.audio_state = AVSRC_AUDIO_SER_OPEN;
    if (get_server_current_play_status())
    {
        for (uint32_t i = 0; i < MAX_CONNS; i++)
        {
            if (inst->con[i].cfg == AV_AUDIO_SRC)
                bt_av_start_stream_by_audio_server(i);
        }
    }

    return 0;
}

//zhengyu:avoid timer timeout after audio close
static U8 send_timer_added = 0;
static int bt_avsrc_register_audio_close(void *user_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    if (send_timer_added == 1)
    {
        bts2_timer_ev_cancel(inst->src_data.tid, NULL, NULL);
        inst->src_data.tid = 0;
        send_timer_added = 0;
    }
    inst->src_data.input_cb = NULL;
    inst->src_data.audio_state = AVSRC_AUDIO_SER_IDLE;
    for (uint32_t i = 0; i < MAX_CONNS; i++)
    {
        if (inst->con[i].cfg == AV_AUDIO_SRC)
            bt_av_suspend_stream_by_audio_server(i);
    }
    return 0;
}

static uint32_t bt_avsrc_register_audio_output(void *user_data, struct rt_ringbuffer *rb)
{
    uint16_t rd_len = bt_avsrc_send_data(rb);
    return rd_len;
}


static int bt_avsrc_register_audio_ioctl(void *user_data, int cmd, void *val)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    if (cmd == A2DP_CMD_PAUSE)
    {
        if (send_timer_added == 1)
        {
            bts2_timer_ev_cancel(inst->src_data.tid, NULL, NULL);
            inst->src_data.tid = 0;
            send_timer_added = 0;
        }
        inst->src_data.input_cb = NULL;
        inst->src_data.audio_state = AVSRC_AUDIO_SER_IDLE;
        for (uint32_t i = 0; i < MAX_CONNS; i++)
            bt_av_suspend_stream_by_audio_server(i);
    }
    else if (cmd == A2DP_CMD_RESUME)
    {
        inst->src_data.audio_state = AVSRC_AUDIO_SER_OPEN;
        for (uint32_t i = 0; i < MAX_CONNS; i++)
            bt_av_start_stream_by_audio_server(i);
    }
    return 0;
}


static int bt_avsrc_register_audio_service(bts2_app_stru *bts2_app_data)
{
    return audio_server_register_audio_device(AUDIO_DEVICE_A2DP_SINK, (struct audio_device *)&g_bt_avsrc_audio_dev);
}



static void bt_avsrc_send_cb(uint16_t m, void *para)
{
    struct rt_ringbuffer *rb = (struct rt_ringbuffer *)para;
    send_timer_added = 0;
    bt_avsrc_send_data(rb);
}
#endif // AUDIO_USING_MANAGER

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
void bt_avsrc_conn_2_snk(BTS2S_BD_ADDR *bd)
{
    bt_av_conn(bd, AV_SNK);
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
void bt_avsrc_get_cfg(bts2_app_stru *bts2_app_data)
{
    for (uint32_t i = 0; i < MAX_CONNS; i++)
        bt_av_get_cfg(i);
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
void bt_avsrc_recfg(bts2_app_stru *bts2_app_data)
{
// TODO: recfg
#if 0
    bts2s_avsrc_inst_data *inst;
    U8 app_serv_cap[SBC_MEDIA_CODEC_SC_SIZE];
    U8 i;
    inst = global_inst;
#ifdef TP_SIG_SMG_BI_14_C_1
    if (inst->con[inst->con_idx].st == avconned_open)
    {
        /*  case1  invalid category */
        app_serv_cap[0] = 0xFF;
        app_serv_cap[1] = 0xFF;
        app_serv_cap[2] = 0xFF;
        app_serv_cap[3] = 0xFF;
        app_serv_cap[4] = 0xFF;
        app_serv_cap[5] = 0xFF;
        app_serv_cap[6] = 0xFF;
        app_serv_cap[7] = 0xFF;
        av_recfg_req(inst->con[inst->con_idx].stream_hdl, ASSIGN_TLABEL, SBC_MEDIA_CODEC_SC_SIZE, app_serv_cap);
    }
#elif defined(TP_SIG_SMG_BI_14_C_3)
    if (inst->con[inst->con_idx].st == avconned_open)
    {
        /*  case3  valid but not support service category */
        app_serv_cap[0] = AV_SC_MEDIA_CODEC;
        app_serv_cap[1] = SBC_MEDIA_CODEC_SC_SIZE - 2;
        app_serv_cap[2] = AV_AUDIO << 4;
        app_serv_cap[3] = AV_MPEG12_AUDIO;/*mp3*/
        app_serv_cap[4] = 0x32;
        app_serv_cap[5] = 0x41;
        app_serv_cap[6] = 0x89;
        app_serv_cap[7] = 0x32;
        av_recfg_req(inst->con[inst->con_idx].stream_hdl, ASSIGN_TLABEL, SBC_MEDIA_CODEC_SC_SIZE, app_serv_cap);
    }
#else
    if (inst->con[inst->con_idx].st == avconned_open)
    {
        app_serv_cap[0] = AV_SC_MEDIA_CODEC;
        app_serv_cap[1] = SBC_MEDIA_CODEC_SC_SIZE - 2;
        app_serv_cap[2] = AV_AUDIO << 4;
        app_serv_cap[3] = AV_SBC;
        for (i = 0; i < SBC_MEDIA_CODEC_SC_SIZE - 4; i++)
        {
            app_serv_cap[4 + i] = sbc_cfg[1][i];/*choose the other sbc code mothod*/
        }
        bt_avsrc_store_sbc_cfg(&inst->con[inst->con_idx].act_cfg, app_serv_cap + 4);
        INFO_TRACE(">> reset the configuration of the stream\n");
        av_recfg_req(inst->con[inst->con_idx].stream_hdl, ASSIGN_TLABEL, SBC_MEDIA_CODEC_SC_SIZE, app_serv_cap);
    }
    else
    {
        if (inst->con[inst->con_idx].st == avconned_streaming)
        {
            INFO_TRACE(" -- the stream is streaming, please supped it!\n");
        }
        else
        {
            INFO_TRACE(" -- the stream is not in use!\n");
        }
    }
#endif
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
void bt_avsrc_disc_2_snk(bts2_app_stru *bts2_app_data)
{
    for (uint32_t i = 0; i < MAX_CONNS; i++)
        bt_av_disconnect(i);
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
void bt_avsrc_suspend_stream(bts2_app_stru *bts2_app_data)
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
void bt_avsrc_abort_stream(bts2_app_stru *bts2_app_data)
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
void bt_avsrc_start_stream(bts2_app_stru *bts2_app_data)
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
void bt_avsrc_release_stream(bts2_app_stru *bts2_app_data)
{
    for (uint32_t i = 0; i < MAX_CONNS; i++)
        bt_av_release_stream(i);

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
int bt_avsrc_get_plyback_conn(bts2s_av_inst_data *inst)
{
    U8 found = FALSE;
    U8 i = 0;
    int res = - 1;

    for (i = 0; i < MAX_CONNS; i++)
    {
        if ((inst->local_seid_info[inst->con[i].local_seid_idx].local_seid.sep_type == AV_SRC) &&
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
static void bt_avsrc_init_data(bts2s_av_inst_data *inst, bts2_app_stru *bts2_app_data)
{
    /*for IOPT test*/
#ifdef AUDIO_USING_MANAGER
    inst->src_data.audio_state = AVSRC_AUDIO_SER_IDLE;
    inst->src_data.tid = 0;
    inst->src_data.stream_frm_time_begin = 0;
#endif // AUDIO_USING_MANAGER
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
static U8 bt_avsrc_role_type_equal(U16 r1, U8 r2)
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
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
extern U8 BQB_TEST_CASE;
U8 bt_avsrc_prepare_sbc(bts2s_av_inst_data *inst, U8 con_idx)
{
    U8 bit_pool_alt;
    U8 toggle_period;
    bts2_sbc_cfg *cfg = &inst->con[con_idx].act_cfg;
    cfg->bit_pool = bts2_sbc_calc_bit_pool(&bit_pool_alt,
                                           &toggle_period,
                                           cfg->chnl_mode,
                                           cfg->sample_freq,
                                           cfg->blocks,
                                           cfg->subbands,
                                           224);

    if (cfg->bit_pool > cfg->max_bitpool)
    {
        cfg->bit_pool = cfg->max_bitpool;
    }

    LOG_D("chnl_mode = %d,alloc_method = %d,sample_freq = %d\n", cfg->chnl_mode,
          cfg->alloc_method,
          cfg->sample_freq);
    LOG_D("blocks = %d,subbands = %d,bit_pool = %d\n", cfg->blocks,
          cfg->subbands,
          cfg->bit_pool);

#ifdef BSP_BQB_TEST
    switch (BQB_TEST_CASE)
    {
    case A2DP_SRC_CC_BV_09_I:
        cfg->chnl_mode = SBC_JOINT_STEREO;
        cfg->sample_freq = 44100;
        cfg->blocks = 16;
        cfg->subbands = 8;
        cfg->bit_pool = 53;
        cfg->alloc_method = SBC_METHOD_LOUDNESS;
        break;
    case A2DP_SRC_CC_BV_10_I:
        cfg->chnl_mode = SBC_JOINT_STEREO;
        cfg->sample_freq = 48000;
        cfg->blocks = 16;
        cfg->subbands = 8;
        cfg->bit_pool = 51;
        cfg->alloc_method = SBC_METHOD_LOUDNESS;
        break;
    case A2DP_SRC_SET_BV_04_I:
    case A2DP_SRC_SET_BV_06_I:
        cfg->chnl_mode = SBC_MONO;
        cfg->sample_freq = 48000;
        cfg->blocks = 16;
        cfg->subbands = 4;
        cfg->bit_pool = 18;
        cfg->alloc_method = SBC_METHOD_SNR;
        break;
    default:
        break;
    }
#endif

    cfg->frmsize = bts2_sbc_encode_cfg(cfg->chnl_mode,
                                       cfg->alloc_method,
                                       cfg->sample_freq,
                                       cfg->blocks,
                                       cfg->subbands,
                                       cfg->bit_pool);

    LOG_D("frmsize = %d\n", cfg->frmsize);

    if (cfg->frmsize != 0)
    {
        /* calculate how many sbc frms fit into maximum size payload pkt */
        cfg->frms_per_payload = (U16)((inst->max_frm_size - 14) / cfg->frmsize);
        //cfg->frms_per_payload = 7;

        if (cfg->frms_per_payload > MAX_FRM_PER_PAYLOAD)
            cfg->frms_per_payload = MAX_FRM_PER_PAYLOAD;

        /* calculate the amount of sample bytes used per SBC encode call */
        cfg->bytes_per_encoding = (U16)(cfg->blocks * cfg->chnls * cfg->subbands << 1);

        cfg->samples_per_l2c_frm = cfg->frms_per_payload * (cfg->subbands * cfg->blocks);
        cfg->bytes_to_rd = (2 * cfg->chnls) * cfg->samples_per_l2c_frm;
        inst->src_data.m_sec_per_pkt = ((cfg->samples_per_l2c_frm * 1000 * MILLISECOND) / cfg->sample_freq);

        inst->src_data.u_sec_per_pkt = ((cfg->samples_per_l2c_frm * 1000000) / cfg->sample_freq) % 1000;
        inst->src_data.m_sec_time_4_next_pkt = (S32)inst->src_data.m_sec_per_pkt;
        inst->src_data.u_sec_per_pkt_sum = inst->src_data.u_sec_per_pkt;
        return TRUE;
    }
    else
    {
        /* unable to cfg SBC with the given pars */
        return FALSE;
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
void bt_avsrc_hdl_disc_handler(bts2s_av_inst_data *inst, uint8_t con_idx)
{
#ifdef AUDIO_DATA_TEST
    if (stream)
    {
        fclose(stream);
    }
#endif // AUDIO_DATA_TEST
    inst->src_data.stream_frm_time_begin = 0;
    inst->suspend_pending = FALSE;

#ifdef AUDIO_USING_MANAGER
    if (send_timer_added == 1)
    {
        bts2_timer_ev_cancel(inst->src_data.tid, NULL, NULL);
        inst->src_data.tid = 0;
        send_timer_added = 0;
    }
#endif

    bt_avsrc_set_start_flag(FALSE);

    bts2_sbc_encode_completed();
}



int8_t bt_avsrc_hdl_streaming_start(bts2s_av_inst_data *inst, uint8_t con_idx)
{
#ifdef AUDIO_USING_MANAGER
    if ((inst->con[con_idx].st == avconned_streaming) && (inst->src_data.input_cb != NULL))
        inst->src_data.input_cb(as_callback_cmd_cache_empty, NULL, 0);
#endif
    return 0;

}

BOOL bt_avsrc_start_flag = FALSE;
void bt_avsrc_set_start_flag(BOOL flag)
{
    bt_avsrc_start_flag = flag;
}

BOOL bt_avsrc_get_start_flag(void)
{
    return bt_avsrc_start_flag;
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
int8_t bt_avsrc_hdl_start_cfm(bts2s_av_inst_data *inst, uint8_t con_idx)
{
    int8_t ret = -1;


    if (bt_avsrc_prepare_sbc(inst, con_idx))
    {
#ifdef AUDIO_DATA_TEST
#if AUDIO_DATA_FROM_PSRAM
        datain = (uint8_t *)INPUT_ADDRESS;

#else
        if (!stream)
        {
            stream = fopen("Test\\D.wav", "rb");

        }
        if (stream)
#endif
        {
            INFO_TRACE("\n -- Begin to send audio data to the peer device!\n");
            bts2_timer_ev_add((U32)inst->src_data.m_sec_per_pkt, bt_avsrc_send_data, 0, inst);
            inst->src_data.stream_frm_time_begin = BTS2_GET_TIME();
        }
#endif // AUDIO_DATA_TEST

        ret = 0;
    }

#if defined(CFG_AV_SRC) && defined(AUDIO_USING_MANAGER)
    if (inst->src_data.audio_state != AVSRC_AUDIO_SER_OPEN)
    {
        ret = -1;
    }
#endif

    if (!ret)
    {
#if defined(CFG_AV)
        bt_interface_bt_event_notify(BT_NOTIFY_A2DP, BT_NOTIFY_A2DP_START_CFM, NULL, 0);
#endif
    }

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
void bt_avsrc_hdl_abort_ind(bts2s_av_inst_data *inst, uint8_t con_idx)
{
#ifdef AUDIO_DATA_TEST
    if (stream)
    {
        fclose(stream);
    }
#endif //AUDIO_DATA_TEST
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
void bt_avsrc_hdl_suspend_ind(bts2s_av_inst_data *inst, uint8_t con_idx)
{
    // Do nothing
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
 *      AV source endcode the audio data and send it to the remote sink,
 *      int the APP, the audio data got from file,
 *
 *----------------------------------------------------------------------------*/
uint8_t bt_avsrc_hdl_start_ind(bts2s_av_inst_data *inst, BTS2S_AV_START_IND *msg, uint8_t con_idx)
{
    U8 res;
    U8 i;
    U8 shdl = 0;


    res = AV_ACPT;

    for (i = 0; i < msg->list_len; i++)
    {
        shdl = *(&msg->first_shdl + i);

        if (shdl == inst->con[con_idx].stream_hdl)
        {
            if (inst->con[con_idx].st == avconned_open)
            {
                /*do prepare again, MTU size might be updated since last time. it should not fail since
                   the SBC pars have alrdy been chked when this fn was called first time */
                if (!bt_avsrc_prepare_sbc(inst, con_idx))
                {
                    break;
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
#ifdef AUDIO_DATA_TEST
        stream = fopen("Test\\D.wav", "rb");
        INFO_TRACE("\n Begain to send audio data to the peer device!\n");
        bts2_timer_ev_add(inst->src_data.m_sec_per_pkt, bt_avsrc_send_data, 0, inst);
        inst->src_data.stream_frm_time_begin = BTS2_GET_TIME();
#endif //AUDIO_DATA_TEST
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

void bt_avrc_close_handler(bts2s_av_inst_data *inst, uint8_t con_idx)
{
#ifdef AUDIO_DATA_TEST
    if (stream)
    {
        fclose(stream);
    }
#endif // AUDIO_DATA_TEST
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
static void bt_avsrc_hdl_close_cfm(bts2_app_stru *bts2_app_data)
{
    bts2s_avsrc_inst_data *inst = global_inst;
    BTS2S_AV_CLOSE_CFM *msg;
    msg = (BTS2S_AV_CLOSE_CFM *)bts2_app_data->recv_msg;
    if (msg->res == AV_ACPT)
    {
        INFO_TRACE(">> accept to release the av stream\n");
        inst->local_seid_info[inst->con[inst->con_idx].local_seid_idx].local_seid.in_use = FALSE;
        inst->con[inst->con_idx].st = avconned;
    }
    else
    {
        INFO_TRACE(">> reject to release the av stream\n");
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

#ifdef AUDIO_DATA_TEST
static void bt_avsrc_send_data(U16 m, void *data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    U8 *pkt_ptr = NULL;
    U8 *payload_ptr  = NULL;
    U8 *sbc_frm_ptr = NULL;
    U8 *samples_ptr = NULL;
    U8 *tmp_ptr = NULL;
    U8 frms;
    U16 payload_size;
    size_t bytes_rdy;
    U32 actual_time_expired;
    U32 actual_timer_delay;
    int con_idx;
    int numrd;
    bts2_sbc_cfg *act_cfg  = NULL;
    U16 i;
    U16 payload_len;
    con_idx = bt_avsrc_get_plyback_conn(inst);

    if (con_idx == - 1)
        return;

    if (inst->con[con_idx].st == avconned_streaming)
    {
        U32 pcm_pkg_pos;
        U32 sb_pos = 0;
        U32 rd_len = 0;
        U16 pl;

        act_cfg = &inst->con[con_idx].act_cfg;
        payload_size = act_cfg->frms_per_payload * act_cfg->frmsize;
        pkt_ptr = bmalloc(payload_size + AV_FIXED_MEDIA_PKT_HDR_SIZE + 1);
        payload_ptr = sbc_frm_ptr = pkt_ptr + AV_FIXED_MEDIA_PKT_HDR_SIZE;
        frms = 0;
        bytes_rdy = act_cfg->bytes_to_rd;
        tmp_ptr = bmalloc(bytes_rdy);
#if AUDIO_DATA_FROM_PSRAM
        if (((uint32_t)dataend - (uint32_t)datain) < bytes_rdy)
        {
            datain = (uint8_t *)INPUT_ADDRESS;
        }
        memcpy(tmp_ptr, datain, bytes_rdy);
        numrd = bytes_rdy;
        datain += bytes_rdy;
#else
        numrd = fread(tmp_ptr, 1, bytes_rdy, stream); // need some
#endif
        samples_ptr = tmp_ptr;
        *sbc_frm_ptr++ = 0; /*reserve space for payload header */

        pcm_pkg_pos = AV_FIXED_MEDIA_PKT_HDR_SIZE + 1;
        pl = payload_size;

        while (numrd > 0)
        {
            BTS2S_SBC_STREAM bss;

            bss.dst_len = pl;
            bss.pdst = &pkt_ptr[pcm_pkg_pos];
            bss.src_len = numrd;
            bss.psrc = &samples_ptr[sb_pos];

            bts2_sbc_encode(&bss);

            sb_pos += bss.src_len_used;
            rd_len -= bss.src_len_used;

            samples_ptr += bss.src_len_used;
            numrd -= bss.src_len_used;


            pcm_pkg_pos += bss.dst_len_used;
            pl -= bss.dst_len_used;
            frms += bss.dst_len_used / act_cfg->frmsize;
        }

        bfree(tmp_ptr);
        payload_len = (U16)(frms * act_cfg->frmsize + 1 + AV_FIXED_MEDIA_PKT_HDR_SIZE);
        if (frms > 0)
        {
            *payload_ptr = frms;
            for (i = 0; i < MAX_CONNS; i++)
            {
                if (inst->con[i].st == avconned_streaming)
                {
                    U8 *send_pkt_ptr;
                    send_pkt_ptr = bmalloc(payload_len);
                    memcpy(send_pkt_ptr, pkt_ptr, payload_len);
                    av_stream_data_req(inst->con[i].stream_hdl,
                                       FALSE,
                                       FALSE,
                                       96, //any dynamic PT in the range 96 - 127
                                       inst->time_stamp,
                                       payload_len,
                                       send_pkt_ptr); /**/
                    //  INFO_TRACE("con_idx = %d\n", i);
                }
            }
            bfree(pkt_ptr);
            inst->time_stamp += frms * act_cfg->blocks * act_cfg->subbands;
        }
        inst->src_data.stream_frm_time_end = BTS2_GET_TIME();

        actual_time_expired = inst->src_data.stream_frm_time_end - inst->src_data.stream_frm_time_begin;
        actual_timer_delay = actual_time_expired - inst->src_data.m_sec_time_4_next_pkt;
        inst->src_data.m_sec_time_4_next_pkt = inst->src_data.m_sec_per_pkt - actual_timer_delay;
        bts2_timer_ev_add(inst->src_data.m_sec_time_4_next_pkt, bt_avsrc_send_data, 0, inst);
        inst->src_data.stream_frm_time_begin = BTS2_GET_TIME();
    }
}
#else

static uint16_t bt_avsrc_send_data(struct rt_ringbuffer *rb)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    U8 *pkt_ptr = NULL;
    U8 *payload_ptr  = NULL;
    U8 *sbc_frm_ptr = NULL;
    U8 *samples_ptr = NULL;
    U8 *tmp_ptr = NULL;
    U8 frms;
    U16 payload_size;
    size_t bytes_rd = 0;
    U32 actual_time_expired;
    U32 actual_timer_delay;
    int con_idx;
    int numrd;
    bts2_sbc_cfg *act_cfg  = NULL;
    U16 i;
    U16 payload_len;

    uint8_t is_empty = 0;

    con_idx = bt_avsrc_get_plyback_conn(inst);


    if (con_idx == - 1)
        return 0;

    //zhengyu:avoid timer timeout while waiting for suspend cfm
    if (inst->con[con_idx].st == avconned_streaming && !inst->suspend_pending)
    {
        do
        {
            U32 pcm_pkg_pos;
            U32 sb_pos = 0;
            U32 rd_len = 0;
            U16 pl;
            U16 buffer_count = av_get_stream_buffize();


            if (inst->src_data.stream_frm_time_begin != 0)
            {

                inst->src_data.stream_frm_time_end = BTS2_GET_TIME();

                actual_time_expired = inst->src_data.stream_frm_time_end - inst->src_data.stream_frm_time_begin;
                actual_timer_delay = actual_time_expired - inst->src_data.m_sec_time_4_next_pkt;
                inst->src_data.u_sec_per_pkt_sum += inst->src_data.u_sec_per_pkt;
                inst->src_data.m_sec_time_4_next_pkt = inst->src_data.m_sec_per_pkt - actual_timer_delay;

                if (inst->src_data.u_sec_per_pkt_sum >= (inst->src_data.m_sec_per_pkt * 1000))
                {
                    //USER_TRACE("Cal us %d, ori %d, target %d\n", inst->src_data.u_sec_per_pkt_sum, inst->src_data.m_sec_per_pkt, inst->src_data.m_sec_time_4_next_pkt);
#if 0
                    inst->src_data.m_sec_time_4_next_pkt += inst->src_data.m_sec_per_pkt - 1;
                    inst->src_data.u_sec_per_pkt_sum = 0;
#else
                    inst->src_data.m_sec_time_4_next_pkt += inst->src_data.m_sec_per_pkt;
                    inst->src_data.u_sec_per_pkt_sum = inst->src_data.u_sec_per_pkt_sum % 1000;
#endif
                }


            }

            inst->src_data.stream_frm_time_begin = BTS2_GET_TIME();

            if (buffer_count >= av_get_max_stream_buffer_cnt() / 2)
            {
                LOG_I("a2dp buffer more than half, should send buffered frames,count = %d\n", buffer_count);
                break;
            }


            uint16_t len = rt_ringbuffer_data_len((struct rt_ringbuffer *)rb);

            act_cfg = &inst->con[con_idx].act_cfg;

            // Just return if length is not enough
            if (len < act_cfg->bytes_to_rd)
            {
#ifdef AUDIO_USING_MANAGER
                LOG_I("a2dp_src: buffer empty\n");

                if (inst->src_data.input_cb == NULL)
                {
                    LOG_I("input_cb = NULL!!!!!!\n");
                    return 0;
                }
                inst->src_data.input_cb(as_callback_cmd_cache_half_empty, NULL, 0);
#endif
                is_empty = 1;
            }

            uint8_t *ptr = bcalloc(1, act_cfg->bytes_to_rd);
            if (!ptr)
                break;

            payload_size = act_cfg->frms_per_payload * act_cfg->frmsize;
            pkt_ptr = bmalloc(payload_size + AV_FIXED_MEDIA_PKT_HDR_SIZE + 1);

            if (!pkt_ptr)
            {
                bfree(ptr);
                break;
            }

            if (!is_empty)
            {
                rt_ringbuffer_get((struct rt_ringbuffer *)rb, ptr, act_cfg->bytes_to_rd);

                len = rt_ringbuffer_data_len((struct rt_ringbuffer *)rb);
#ifdef AUDIO_USING_MANAGER
                if ((len <= rt_ringbuffer_get_size((struct rt_ringbuffer *)rb) / 2)
                        || (len < act_cfg->bytes_to_rd))
                {
                    if (inst->src_data.input_cb)
                    {
                        inst->src_data.input_cb(as_callback_cmd_cache_empty, NULL, 0);
                    }
                }
#endif
            }

            payload_ptr = sbc_frm_ptr = pkt_ptr + AV_FIXED_MEDIA_PKT_HDR_SIZE;
            frms = 0;
            numrd = bytes_rd = act_cfg->bytes_to_rd;

            samples_ptr = ptr;
            *sbc_frm_ptr++ = 0; /*reserve space for payload header */

            pcm_pkg_pos = AV_FIXED_MEDIA_PKT_HDR_SIZE + 1;
            pl = payload_size;

            while (numrd > 0)
            {
                BTS2S_SBC_STREAM bss;

                bss.dst_len = pl;
                bss.pdst = &pkt_ptr[pcm_pkg_pos];
                bss.src_len = numrd;
                bss.psrc = &samples_ptr[sb_pos];

                bts2_sbc_encode(&bss);

                sb_pos += bss.src_len_used;
                rd_len -= bss.src_len_used;

                samples_ptr += bss.src_len_used;
                numrd -= bss.src_len_used;


                pcm_pkg_pos += bss.dst_len_used;
                pl -= bss.dst_len_used;
                frms += bss.dst_len_used / act_cfg->frmsize;
            }

            bfree(ptr);
            payload_len = (U16)(frms * act_cfg->frmsize + 1 + AV_FIXED_MEDIA_PKT_HDR_SIZE);
            if (frms > 0)
            {
                *payload_ptr = frms;
                for (i = 0; i < MAX_CONNS; i++)
                {
                    if (inst->con[i].st == avconned_streaming)
                    {
                        //U8 *send_pkt_ptr;
                        //send_pkt_ptr = bmalloc(payload_len);
                        //memcpy(send_pkt_ptr, pkt_ptr, payload_len);
                        av_stream_data_req(inst->con[i].stream_hdl,
                                           FALSE,
                                           FALSE,
                                           96, //any dynamic PT in the range 96 - 127
                                           inst->time_stamp,
                                           payload_len,
                                           pkt_ptr); /**/
                        //  INFO_TRACE("con_idx = %d\n", i);
                    }
                }
                inst->time_stamp += frms * act_cfg->blocks * act_cfg->subbands;
            }

            //bfree(pkt_ptr);
        }
        while (0);

#ifdef AUDIO_USING_MANAGER
        // LOG_D("m_sec_time_4_next_pkt = %d\n", inst->src_data.m_sec_time_4_next_pkt);
        if (inst->src_data.m_sec_time_4_next_pkt > 0)
        {
            send_timer_added = 1;
            inst->src_data.tid = bts2_timer_ev_add(inst->src_data.m_sec_time_4_next_pkt, bt_avsrc_send_cb, 0, (void *)rb);
        }
        else
        {
            send_timer_added = 1;
            inst->src_data.tid = bts2_timer_ev_add(0, bt_avsrc_send_cb, 0, (void *)rb);
        }
#endif
    }

    return bytes_rd;
}


BOOL bt_avrc_check_stream_state(void)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    int con_idx;
    con_idx = bt_avsrc_get_plyback_conn(inst);

    if (con_idx == - 1)
        return FALSE;

    if (inst->con[con_idx].st == avconned_streaming)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}


#endif

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
static void bt_avsrc_hdl_sts_ind(bts2_app_stru *bts2_app_data)
{
    BTS2S_AV_STS_IND *msg;

    msg = (BTS2S_AV_STS_IND *)bts2_app_data->recv_msg;

    switch (msg->st_type)
    {
    case BTS2MU_AV_ENB_CFM:
        bt_avsrc_hdl_enb_cfm(bts2_app_data);
        break;

    case BTS2MU_AV_DISB_CFM:
        /* bt_avsrc_hdl_disb_cfm(bts2_app_data); */
        break;

    case BTS2MU_AV_CONN_CFM:
        /*bt_avsrc_hdl_conn_cfm(bts2_app_data);*/
        break;

    case BTS2MU_AV_CONN_IND:
        bt_avsrc_hdl_conn_ind(bts2_app_data);
        break;

    default:
        INFO_TRACE(" unhandled STS_IND type \n");
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
void bt_avsrc_init(bts2s_av_inst_data *inst, bts2_app_stru *bts2_app_data)
{
    bt_avsrc_init_data(inst, bts2_app_data);
#ifdef AUDIO_USING_MANAGER
    bt_avsrc_register_audio_service(bts2_app_data);
#endif // AUDIO_USING_MANAGER

#if AUDIO_DATA_FROM_PSRAM
    memcpy((void *)INPUT_ADDRESS, (void *)0x14120000, 0x1AEAA0);
    dataend = (uint8_t *)INPUT_ADDRESS + 0x1AEAA0;
#endif // AUDIO_DATA_FROM_PSRAM
    INFO_TRACE(">> AV source enabled \n");
    av_enb_req(inst->que_id, AV_AUDIO_SRC); //act the svc
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
void bt_avsrc_msg_handler(bts2_app_stru *bts2_app_data)
{
    U16 *msg_type;
    msg_type = (U16 *)bts2_app_data->recv_msg;
    switch (*msg_type)
    {
    case BTS2MU_AV_SECU_CTRL_CFM:
    {
        INFO_TRACE("<< receive av security contrla confirmation\n");
        break;
    }
    case BTS2MU_AV_SECU_CTRL_IND:
    {
        INFO_TRACE("<< receive av security control indication\n");
        break;
    }
    case BTS2MU_AV_STREAM_DATA_IND:
    {
        INFO_TRACE("<< receive av stream data indication\n");
        break;
    }
#if 0
    case BTS2MU_AV_STS_IND:
    {
        bt_avsrc_hdl_sts_ind(bts2_app_data);
        break;
    }
#endif
    default:
        break;
    }
}
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
