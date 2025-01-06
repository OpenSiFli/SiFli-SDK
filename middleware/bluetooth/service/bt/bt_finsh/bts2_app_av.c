/**
  ******************************************************************************
  * @file   bts2_app_av.c
  * @author Sifli software development team
  * @brief SIFLI BT a2dp app common file.
 *
  ******************************************************************************
*/
/**
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

#include "bts2_app_inc.h"
#include "rtthread.h"


#ifdef RT_USING_BT
    #include "bt_rt_device.h"
#endif

#ifdef AUDIO_USING_MANAGER
    #include "audio_server.h"
#endif // AUDIO_USING_MANAGER

#if defined(CFG_AV)
#include "bts2_mem.h"
#define DBG_TAG             "btapp.av"
//#define DBG_LVL             DBG_INFO
#include "rtdbg.h"


const static U8 av_sbc_capabilities[4] = {0x3F, 0xFF, 0x02, 0x35 }; /*support all */
/* AAC support capabilities */
const static U8 av_aac_capabilities[6] = {0x80, 0x01, 0x8C, 0x84, 0xE2, 0x00};


static const U8 av_cap_rsp[2][AAC_MEDIA_CODEC_SC_SIZE] =
{
    {AV_SC_MEDIA_CODEC, SBC_MEDIA_CODEC_SC_SIZE - 2, AV_AUDIO << 4, AV_SBC, 0x3F, 0xFF, 0x02, 0x35, 0x0, 0x0},
    {AV_SC_MEDIA_CODEC, AAC_MEDIA_CODEC_SC_SIZE - 2, AV_AUDIO << 4, AV_MPEG24_AAC, 0x80, 0x01, 0x8C, 0x84, 0xE2, 0x0}
};

static U8 av_sbc_cfg[4][4] =
{
    { 0x12, 0x15, 0x02, 0xFA },  /* stereo, LOUDNESS, 8 bands, 16 blocks, 44.1k_hz */
    //{ 0x28, 0x15, 0x02, 0xFA },  /* for test */
    { 0x11, 0x16, 0x02, 0xFA },  /* joint, SNR, 8 bands, 16 blocks, 48k_hz */
    { 0x22, 0x1A, 0x02, 0x80 },  /* stereo, SNR, 4 bands, 16 blocks, 32k_hz */
    { 0x48, 0x2A, 0x02, 0x30 }   /* mono, SNR, 4 bands, 12 blocks, 32k_hz */
};



static bts2s_av_inst_data *global_inst;
U8 BQB_TEST_CASE = BQB_TEST_RESET;

extern uint8_t   bts2s_avsnk_openFlag;//0x00:dont open a2dp profile; 0x01:open a2dp profile;

static int bt_av_wait_var_set(U8 *p)
{
    int ret = -1;
    for (int i = 0; i < 300; i++)
    {
        if (*p)
        {
            ret = 0;
            break;
        }
        rt_thread_mdelay(10);
    }
    return ret;
}
static void bt_av_init_data(bts2s_av_inst_data *inst, bts2_app_stru *bts2_app_data)
{
    U8 i = 0;
    inst->que_id = bts2_app_data->phdl;

#ifdef CFG_AV_SNK
    for (; i < MAX_NUM_LOCAL_SNK_SEIDS; i++)
    {
        inst->local_seid_info[i].is_enbd = TRUE;
        inst->local_seid_info[i].local_seid.acp_seid = i + 1;
        inst->local_seid_info[i].local_seid.in_use = FALSE;
        inst->local_seid_info[i].local_seid.media_type = AV_AUDIO;
        inst->local_seid_info[i].local_seid.sep_type = AV_SNK; /*IS_SNK */
#ifdef CFG_AV_AAC
        inst->local_seid_info[i].local_seid.codec = i < (MAX_NUM_LOCAL_SNK_SEIDS - 1) ? AV_SBC : AV_MPEG24_AAC;
#else
        inst->local_seid_info[i].local_seid.codec = AV_SBC;
#endif
    }
#endif // CFG_AV_SNK

#ifdef CFG_AV_SRC
    for (; i < MAX_NUM_LOCAL_SRC_SEIDS + MAX_NUM_LOCAL_SNK_SEIDS; i++)
    {
        inst->local_seid_info[i].is_enbd = TRUE; // What is the usage of the varaible
        inst->local_seid_info[i].local_seid.acp_seid = i + 1;
        inst->local_seid_info[i].local_seid.in_use = FALSE;
        inst->local_seid_info[i].local_seid.media_type = AV_AUDIO;
        inst->local_seid_info[i].local_seid.sep_type = AV_SRC;//AV_SRC; /* IS_SRC */
        inst->local_seid_info[i].local_seid.codec = AV_SBC;
    }
#endif //CFG_AV_SRC

    inst->con_idx = 0;
    for (i = 0; i < MAX_CONNS; i++)
    {
        inst->con[i].st = avidle;
        inst->con[i].role = ACPTOR; //TODO :how to set the role and use it
        inst->con[i].local_seid_idx = 0xFF;
        inst->con[i].in_use = FALSE;
        inst->con[i].cfg = AV_AUDIO_NO_ROLE;
    }



    inst->max_frm_size = 672; /*tmp. init. */
    inst->time_stamp = 0;
    inst->close_pending = FALSE;
    inst->suspend_pending = FALSE;


}


//TODO: check the dedicated role. For MP, just recovery all seid.
static void bt_av_recovery_local_seid(bts2s_av_inst_data *inst, uint16_t role)
{
    U8 i = 0;

#ifdef CFG_AV_SNK
    if (role == AV_AUDIO_SNK)
    {
        for (; i < MAX_NUM_LOCAL_SNK_SEIDS; i++)
        {
            inst->local_seid_info[i].local_seid.in_use = FALSE;
        }
    }
#endif // CFG_AV_SNK

#ifdef CFG_AV_SRC
    if (role == AV_AUDIO_SRC)
    {
        for (; i < MAX_NUM_LOCAL_SRC_SEIDS + MAX_NUM_LOCAL_SNK_SEIDS; i++)
        {
            inst->local_seid_info[i].local_seid.in_use = FALSE;
        }
    }
#endif //CFG_AV_SRC
}

U8 bt_av_get_idx_from_cid(bts2s_av_inst_data *inst, U16 sought_cid)
{
    U8 iter;

    for (iter = 0; iter < MAX_CONNS; ++ iter)
    {
        if (inst->con[iter].conn_id == sought_cid)
        {
            return iter;
        }
    }
    return MAX_CONNS + 1;
}

U8 bt_av_get_idx_from_shdl(bts2s_av_inst_data *inst, U8 shdl)
{
    U8 iter;

    for (iter = 0; iter < MAX_CONNS; ++ iter)
    {
        if (inst->con[iter].stream_hdl == shdl)
            return iter;
    }
    return MAX_CONNS + 1;
}

static uint16_t bt_av_get_role_from_sep_type(uint8_t sep)
{
    uint16_t role = AV_AUDIO_NO_ROLE;
    if (sep == AV_SNK)
        role = AV_AUDIO_SNK;
    else if (sep == AV_SRC)
        role = AV_AUDIO_SRC;

    return role;
}

static uint8_t bt_av_get_local_seid(bts2s_av_inst_data *inst, uint16_t cfg, uint8_t codec)
{
    uint32_t i = 0;
    uint8_t local_id = 0xFF;
#ifdef CFG_AV_SNK
    if (cfg == AV_AUDIO_SNK)
        for (; i < MAX_NUM_LOCAL_SNK_SEIDS; i++)
            if (inst->local_seid_info[i].is_enbd && !inst->local_seid_info[i].local_seid.in_use
                    && (inst->local_seid_info[i].local_seid.codec == codec))
            {
                local_id = i;
                break;
            }
#endif // CFG_AV_SNK

#ifdef CFG_AV_SRC
    if (cfg == AV_AUDIO_SRC)
        for (i = MAX_NUM_LOCAL_SNK_SEIDS; i < MAX_NUM_LOCAL_SNK_SEIDS + MAX_NUM_LOCAL_SNK_SEIDS; i++)
            if (inst->local_seid_info[i].is_enbd && !inst->local_seid_info[i].local_seid.in_use
                    && (inst->local_seid_info[i].local_seid.codec == codec))
            {
                local_id = i;
                break;
            }
#endif //CFG_AV_SRC
    return local_id;
}

void bt_av_store_sbc_cfg(bts2_sbc_cfg *cfg, U8 *sbc_ie)
{
    U8 i, tmp, mask;

    tmp = (U8)((*sbc_ie) & SBC_IE_SAMPLE_FREQ_MASK);

    switch (tmp)
    {
    case 0x80:
        cfg->sample_freq = 16000;
        break;

    case 0x40:
        cfg->sample_freq = 32000;
        break;

    case 0x20:
        cfg->sample_freq = 44100;
        break;

    default:
        cfg->sample_freq = 48000;
        break;
    }

    tmp = (U8)((*sbc_ie) & SBC_IE_CHNL_MODE_MASK);
    switch (tmp)
    {
    case 0x01:
        cfg->chnl_mode = SBC_JOINT_STEREO;
        break;
    case 0x02:
        cfg->chnl_mode = SBC_STEREO;
        break;
    case 0x04:
        cfg->chnl_mode = SBC_DUAL;
        break;
    case 0x08:
        cfg->chnl_mode = SBC_MONO;
        break;
    default:
        /*this should not be possible! */
        break;
    }

    cfg->chnls = 1;
    if (cfg->chnl_mode != SBC_MONO)
    {
        cfg->chnls = 2;
    }

    sbc_ie++ ;

    tmp = (U8)((*sbc_ie) & SBC_IE_BLOCK_MASK);
    mask = 0x80;
    for (i = 0; i < 4; i++)
    {
        if (tmp == mask)
        {
            cfg->blocks = (U8)((1 + i) << 2);
            break;
        }
        mask >>= 1;
    }
    cfg->subbands = (U8)(12 - ((*sbc_ie) & SBC_IE_SUBBAND_MASK));
    cfg->alloc_method = SBC_METHOD_SNR;
    if (((*sbc_ie) & SBC_IE_ALLOC_METHOD_MASK) == 1)
    {
        cfg->alloc_method = SBC_METHOD_LOUDNESS;
    }

    sbc_ie++ ;
    cfg->min_bitpool = *sbc_ie;
    sbc_ie++ ;
    cfg->max_bitpool = *sbc_ie;
}



static void bt_av_hdl_enb_cfm(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_ENB_CFM *msg;
    U8 i = 0;
    U8 found = FALSE;

    msg = (BTS2S_AV_ENB_CFM *)bts2_app_data->recv_msg;


    USER_TRACE("enable(%d) ret %d", msg->enable_role, msg->res);

    if (msg->enable_role == AV_AUDIO_SRC)
    {
        INFO_TRACE(">> AUDIO SOURCE ENB\n");
    }
    else if (msg->enable_role == AV_AUDIO_SNK)
    {
        INFO_TRACE(">> AUDIO SINK ENB\n");
#if defined(CFG_AV)
        bt_interface_bt_event_notify(BT_NOTIFY_A2DP, BT_NOTIFY_AVSNK_OPEN_COMPLETE, NULL, 0);
#else
        LOG_I("URC BT avsnk open complete ind");
#endif
    }
}


static void bt_av_hdl_discover_cfm(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_DISCOVER_CFM *msg;
    BTS2S_AV_SEID_INFO seid_info;
    U8 i;
    U8 con_idx;

    msg = (BTS2S_AV_DISCOVER_CFM *)bts2_app_data->recv_msg;

    con_idx = bt_av_get_idx_from_cid(inst, msg->conn_id);

    if (con_idx > MAX_CONNS)
    {
        return;
    }
    else
    {
        INFO_TRACE("connect index %d\n", con_idx);
    }

    if (msg->res == AV_ACPT)
    {

#ifdef BSP_BQB_TEST
        switch (BQB_TEST_CASE)
        {
        case A2DP_SRC_AS_BV_01_I:
        case A2DP_SRC_AS_BI_01_I:
        case A2DP_SRC_REL_BV_01_I:
        case A2DP_SRC_REL_BV_02_I:
        case A2DP_SRC_SET_BV_01_I:
        case A2DP_SRC_SET_BV_02_I:
        case A2DP_SRC_SET_BV_03_I:
        case A2DP_SRC_SET_BV_04_I:
        case A2DP_SRC_SET_BV_05_I:
        case A2DP_SRC_SET_BV_06_I:
        case A2DP_SRC_SUS_BV_01_I:
        case A2DP_SRC_SUS_BV_02_I:
            USER_TRACE("BQB test for A2DP/SRC/AS/BV-01-I\n");
            inst->con[con_idx].cfg = AV_AUDIO_SRC;
            break;
        default:
            break;
        }
#endif

        for (i = 0; i < MAX_NUM_RMT_SEIDS; i++)
        {
            inst->con[con_idx].rmt_seid[i] = 0;
        }
        inst->con[con_idx].rmt_seid_idx = 0;

        /*chk availability of audio stream endpoint and select seid */
        for (i = 0; (i < msg->list_len) && (i < MAX_NUM_RMT_SEIDS); i++)
        {
            memcpy(&seid_info, ((U8 *) & (msg->first_seid_info)) + i * sizeof(BTS2S_AV_SEID_INFO), sizeof(BTS2S_AV_SEID_INFO));

            /*looking for an audio snk not in use */
            if ((seid_info.media_type == AV_AUDIO)
                    && seid_info.in_use == FALSE)
            {
                if (((inst->con[con_idx].cfg == AV_AUDIO_SNK) && (seid_info.sep_type == AV_SRC)) ||
                        ((inst->con[con_idx].cfg == AV_AUDIO_SRC) && (seid_info.sep_type == AV_SNK)))
                {
                    inst->con[con_idx].rmt_seid[inst->con[con_idx].rmt_seid_idx] = seid_info.acp_seid;
                    inst->con[con_idx].rmt_seid_idx++ ;
                }
            }
        }

        if (inst->con[con_idx].rmt_seid[0] != 0)
        {
            /*ask for the end - points capabilities */
            INFO_TRACE("<< got available audio end-point\n");
            INFO_TRACE(">> request to get capabilities of the end-point\n");
            av_get_capabilities_req(inst->con[con_idx].conn_id, inst->con[con_idx].rmt_seid[0], ASSIGN_TLABEL);
            inst->con[con_idx].rmt_seid_idx = 0;
        }
        else
        {
            INFO_TRACE("<< remote device not have available audio end-points\n");
        }
    }
    else
    {
        INFO_TRACE("<< stream discover rejected, errorcode is %x\n", msg->res);
    }
}


static void bt_av_hdl_discover_ind(bts2_app_stru *bts2_app_data)
{
    BTS2S_AV_DISCOVER_IND *msg;
    BTS2S_AV_SEID_INFO seid_info[MAX_NUM_LOCAL_SEIDS];
    U8 iter;
    U8 len = 0;
    U8 found = FALSE;
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    msg = (BTS2S_AV_DISCOVER_IND *)bts2_app_data->recv_msg;
    INFO_TRACE(" << indication to discover seid \n ");
    for (iter = 0; iter < MAX_NUM_LOCAL_SEIDS; iter++)
    {
        if (!inst->local_seid_info[iter].local_seid.in_use)
        {
            seid_info[len] = inst->local_seid_info[iter].local_seid;
            len++;
        }
    }
    if (len > 0)
    {
        INFO_TRACE(">> accept the discover indication\n");
        av_discover_rsp_acp(msg->conn_id, msg->tlabel, len, seid_info);
    }
    else
    {
        av_discover_rsp_rej(msg->conn_id, msg->tlabel, AV_SEP_IN_USE);
    }
}


static void bt_av_sbc_cfg_para_select(bts2s_av_inst_data *inst, uint16_t con_idx, uint8_t *app_serv_cap, uint8_t *serv_cap)
{
    uint32_t i;
    inst->con[con_idx].act_cfg.min_bitpool = *(serv_cap + 6);
    inst->con[con_idx].act_cfg.max_bitpool = *(serv_cap + 7);

    /*build app. svc capabilities (media and if supp cont protection) */
    app_serv_cap[0] = AV_SC_MEDIA_CODEC;
    app_serv_cap[1] = SBC_MEDIA_CODEC_SC_SIZE - 2;
    app_serv_cap[2] = AV_AUDIO << 4;
    app_serv_cap[3] = AV_SBC;

    for (i = 0; i < (SBC_MEDIA_CODEC_SC_SIZE - 6); i++)
    {
        app_serv_cap[4 + i] = av_sbc_cfg[0][i]; /*first cfg setting is our default */
    }


    /*select a cfguration from the capabilities (ours and the peers) */
    do
    {
        if ((*(serv_cap + 4) == 0x6a) && (*(serv_cap + 7) == 0x2c))
        {
            /*... a workaround for sonorix (has very bad sound unless using 32k_hz) */
            app_serv_cap[4] = 0x40;
            break;
        }

        if (*(serv_cap + 4) & 0x20 & av_sbc_capabilities[0])
        {
            app_serv_cap[4] = (0x10 << 1);
            break;
        }

        if (*(serv_cap + 4) & 0x10 & av_sbc_capabilities[0])
        {
            app_serv_cap[4] = 0x10;
            break;
        }
    }
    while (0);

    for (i = 0; i < 4; i++)
    {
        if (*(serv_cap + 4) & (0x01 << i) & av_sbc_capabilities[0])
        {
            app_serv_cap[4] |= (0x01 << i);
            break;
        }
    }
    // app_serv_cap[4] |= 0x02;
    for (i = 0; i < 4; i++)
    {
        if (*(serv_cap + 5) & (0x10 << i) & av_sbc_capabilities[1])
        {
            app_serv_cap[5] = (0x10 << i);
            break;
        }
    }
    for (i = 0; i < 2; i++)
    {
        if (*(serv_cap + 5) & (0x04 << i) & av_sbc_capabilities[1])
        {
            app_serv_cap[5] |= (0x04 << i);
            break;
        }
    }
    for (i = 0; i < 2; i++)
    {
        if (*(serv_cap + 5) & (0x01 << i) & av_sbc_capabilities[1])
        {
            app_serv_cap[5] |= (0x01 << i);
            break;
        }
    }

    /*take the min/max bitpool from peer rsp */
    app_serv_cap[6] = inst->con[con_idx].act_cfg.min_bitpool;
    app_serv_cap[7] = inst->con[con_idx].act_cfg.max_bitpool;

    bt_av_store_sbc_cfg(&inst->con[con_idx].act_cfg, app_serv_cap + 4);

}

static void bt_av_find_sep_and_set_cfg(bts2s_av_inst_data *inst, uint16_t con_idx)
{
#ifdef CFG_AV_AAC
    uint8_t prefer_codec[2] = {AV_MPEG24_AAC, AV_SBC};
#else
    uint8_t prefer_codec[1] = {AV_SBC};
#endif
    uint8_t *serv_cap;
    uint16_t idx = 0;
    BOOL found = FALSE;

#ifdef CFG_AV_AAC
    for (uint32_t t = 0; t < 2; t++)
#else
    for (uint32_t t = 0; t < 1; t++)
#endif
    {
        for (uint32_t j = 0; j < MAX_NUM_RMT_SEIDS; j++)
        {
            if (found == TRUE)
                break;

            if (inst->con[con_idx].rmt_seid[j] == 0)
                break;

            if (inst->con[con_idx].rmt_capa[j].serv_cap_data == NULL)
                continue;

            serv_cap = av_get_svc_cap(AV_SC_MEDIA_CODEC, inst->con[con_idx].rmt_capa[j].serv_cap_data, inst->con[con_idx].rmt_capa[j].serv_cap_len, &idx);
            if (serv_cap != NULL)
            {
                if ((*(serv_cap + 2) >> 4 == AV_AUDIO) && (*(serv_cap + 3) == prefer_codec[t]))
                {
                    uint8_t app_serv_cap[SBC_MEDIA_CODEC_SC_SIZE];
                    uint16_t app_serv_cap_len;
                    if (prefer_codec[t] == AV_SBC)
                    {
                        app_serv_cap_len = SBC_MEDIA_CODEC_SC_SIZE;
                        bt_av_sbc_cfg_para_select(inst, con_idx, app_serv_cap, serv_cap);
                    }
#ifdef CFG_AV_AAC
                    else if (prefer_codec[t] == AV_MPEG24_AAC)
                    {
                        //TODO: AAC codec select
                    }
#endif
                    else
                    {
                        // Not support others yet
                        RT_ASSERT(0);
                    }

                    INFO_TRACE("<< get the audio end-point capabilities successfully\n");
                    INFO_TRACE(">> request to set configuration of the stream\n");

                    found = TRUE;
                    uint8_t local_seid_idx = bt_av_get_local_seid(inst, inst->con[con_idx].cfg, prefer_codec[t]);
                    // Find rmt seid but local is not enough, won't start set cfg
                    if (local_seid_idx == 0xFF)
                    {
                        break;
                    }

                    inst->con[con_idx].local_seid_idx = local_seid_idx;
                    inst->local_seid_info[local_seid_idx].local_seid.in_use = TRUE;
                    av_set_cfg_req(inst->con[con_idx].conn_id,
                                   ASSIGN_TLABEL,
                                   inst->con[con_idx].rmt_seid[j],
                                   inst->local_seid_info[local_seid_idx].local_seid.acp_seid,
                                   app_serv_cap_len,
                                   app_serv_cap,
                                   inst->con[con_idx].rmt_capa[j].serv_cap_len,
                                   inst->con[con_idx].rmt_capa[j].serv_cap_data);
                    break;
                }
            }
        }

    }

    // Free related memory
    for (uint32_t j = 0; j < MAX_NUM_RMT_SEIDS; j++)
    {
        if (inst->con[con_idx].rmt_seid[j] == 0)
            break;

        if (inst->con[con_idx].rmt_capa[j].serv_cap_data)
        {
            bfree(inst->con[con_idx].rmt_capa[j].serv_cap_data);
            inst->con[con_idx].rmt_capa[j].serv_cap_data =  NULL;
        }
        inst->con[con_idx].rmt_capa[j].serv_cap_len = 0;
        inst->con[con_idx].rmt_seid[j] = 0;
    }

}

static void bt_av_hdl_get_cap_cfm(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_GET_CAPABILITIES_CFM *msg;
    U8 found = FALSE;
    U16 idx = 0;
    U8 con_idx;

    msg = (BTS2S_AV_GET_CAPABILITIES_CFM *)bts2_app_data->recv_msg;
    con_idx = bt_av_get_idx_from_cid(inst, msg->conn_id);

    INFO_TRACE("AVSINK GET CAP CFM");
    if (msg->res == AV_ACPT)
    {
        if (inst->con[con_idx].rmt_seid[inst->con[con_idx].rmt_seid_idx] != 0)
        {
            inst->con[con_idx].rmt_capa[inst->con[con_idx].rmt_seid_idx].serv_cap_data = msg->serv_cap_data;
            inst->con[con_idx].rmt_capa[inst->con[con_idx].rmt_seid_idx].serv_cap_len = msg->serv_cap_len;
        }

        // Connect all capability to select.
        {
            inst->con[con_idx].rmt_seid_idx++ ;
            if ((inst->con[con_idx].rmt_seid_idx < MAX_NUM_RMT_SEIDS) && (inst->con[con_idx].rmt_seid[inst->con[con_idx].rmt_seid_idx] != 0))
            {
                /*ask for the next end - point capabilities */
                INFO_TRACE(">> request to get capabilities of the end-point\n");
                av_get_capabilities_req(inst->con[con_idx].conn_id, inst->con[con_idx].rmt_seid[inst->con[con_idx].rmt_seid_idx], ASSIGN_TLABEL);
            }
            else
            {
                bt_av_find_sep_and_set_cfg(inst, con_idx);
            }
        }
    }
    else
    {
        bt_av_find_sep_and_set_cfg(inst, con_idx);
    }
}


static void bt_av_hdl_get_cap_ind(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_GET_CAPABILITIES_IND *msg;
    U8 i = 0;
    U8 con_idx;
    U8 found = FALSE;
    msg = (BTS2S_AV_GET_CAPABILITIES_IND *)bts2_app_data->recv_msg;

#if defined(TP_SIG_SMG_BI_04_C)
    av_get_capabilities_rsp_rej(msg->conn_id, msg->tlabel, AV_BAD_LEN);
#elif defined(TP_SIG_SMG_BI_06_C)
    av_get_capabilities_rsp_rej(msg->conn_id, msg->tlabel, (U8)0xE1);
#else
    con_idx = bt_av_get_idx_from_cid(inst, msg->conn_id);

    while ((i < MAX_NUM_LOCAL_SEIDS) && (!found))
    {
        if (inst->local_seid_info[i].local_seid.acp_seid == msg->acp_seid)
        {
            found = TRUE;
            break;
        }
        i++ ;
    }


    if (found)
    {
        if (inst->local_seid_info[i].local_seid.codec == AV_SBC)
        {

            if (inst->con[con_idx].local_seid_idx == 0xff)
                inst->con[con_idx].local_seid_idx = i;


            U8 *cap_data;
            // U8 cap_len = av_cap_rsp[0][1] + 2 + AV_DELAY_REPORT_SC_SIZE + AV_MEDIA_TRASPORT_SIZE + AV_CONTENT_PROTECTION_SIZE;
            U8 cap_len = av_cap_rsp[0][1] + 2 + AV_MEDIA_TRASPORT_SIZE;
            cap_data = bmalloc(cap_len);
            BT_OOM_ASSERT(cap_data);
            if (cap_data)
            {
                cap_data[0] = AV_SC_MEDIA_TRS;
                cap_data[1] = 0;

                bmemcpy(cap_data + 2, &av_cap_rsp[0], av_cap_rsp[0][1] + 2);

                // *(cap_data + cap_len - 6) = AV_SC_CONT_PROTECTION;
                // *(cap_data + cap_len - 5) = 2;
                // *(cap_data + cap_len - 4) = 2;
                // *(cap_data + cap_len - 3) = 0;



                // *(cap_data + cap_len - 2) = AV_SC_DELAY_REPORTING;
                // *(cap_data + cap_len - 1) = 0;



                INFO_TRACE(">> accept the get capabilities indication\n");

                av_get_capabilities_rsp_acp(msg->conn_id,
                                            msg->tlabel,
                                            cap_len,
                                            cap_data);
                bfree(cap_data);
            }
        }
        else if (inst->local_seid_info[i].local_seid.codec == AV_MPEG24_AAC)
        {
            //TODO
            // Before competed, just reject in this case
            av_get_capabilities_rsp_rej(msg->conn_id, msg->tlabel, AV_BAD_ACP_SEID);
        }
        else
        {
            av_get_capabilities_rsp_rej(msg->conn_id, msg->tlabel, AV_BAD_ACP_SEID);
        }
    }
    else
    {
        INFO_TRACE(">> reject the get capabilities indication\n");
        av_get_capabilities_rsp_rej(msg->conn_id, msg->tlabel, AV_BAD_ACP_SEID);
    }

#endif
}

static void bt_av_hdl_get_all_cap_ind(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_GET_ALL_CAPABILITIES_IND *msg;
    U8 i = 0;
    U8 con_idx;
    U8 found = FALSE;
    msg = (BTS2S_AV_GET_ALL_CAPABILITIES_IND *)bts2_app_data->recv_msg;

#if defined(TP_SIG_SMG_BI_04_C)
    av_get_all_capabilities_rsp_rej(msg->conn_id, msg->tlabel, AV_BAD_LEN);
#elif defined(TP_SIG_SMG_BI_06_C)
    av_get_all_capabilities_rsp_rej(msg->conn_id, msg->tlabel, (U8)0xE1);
#else
    con_idx = bt_av_get_idx_from_cid(inst, msg->conn_id);

    while ((i < MAX_NUM_LOCAL_SEIDS) && (!found))
    {
        if (inst->local_seid_info[i].local_seid.acp_seid == msg->acp_seid)
        {
            found = TRUE;
            break;
        }
        i++ ;
    }

    if (found)
    {
#if 0
        U8 app_serv_cap[SBC_MEDIA_CODEC_SC_SIZE];

        if (inst->con[con_idx].local_seid_idx == - 1)
            inst->con[con_idx].local_seid_idx = i;

        app_serv_cap[0] = AV_SC_MEDIA_CODEC;
        app_serv_cap[1] = SBC_MEDIA_CODEC_SC_SIZE - 2;
        app_serv_cap[2] = AV_AUDIO << 4;
        app_serv_cap[3] = AV_SBC;
        for (i = 0; i < (SBC_MEDIA_CODEC_SC_SIZE - 4); i++)
        {
            app_serv_cap[4 + i] = avsnk_sbc_capabilities[i];
        }
#endif
        U8 codec_sel = inst->local_seid_info[i].local_seid.codec == AV_SBC ? 0 : 1;

        //inst->con[con_idx].act_cfg.min_bitpool = app_serv_cap[6];
        //inst->con[con_idx].act_cfg.max_bitpool = app_serv_cap[7];

        if (codec_sel == 0)
        {
            inst->con[con_idx].act_cfg.min_bitpool = av_cap_rsp[codec_sel][6];
            inst->con[con_idx].act_cfg.max_bitpool = av_cap_rsp[codec_sel][7];
        }

        INFO_TRACE(">> accept the get all capabilities indication\n");

        // add delay report capability
        U8 *cap_data;
        // U8 cap_len = av_cap_rsp[codec_sel][1] + 2 + AV_DELAY_REPORT_SC_SIZE + AV_MEDIA_TRASPORT_SIZE + AV_CONTENT_PROTECTION_SIZE;
        U8 cap_len = av_cap_rsp[codec_sel][1] + 2 + AV_MEDIA_TRASPORT_SIZE;
        cap_data = bmalloc(cap_len);
        BT_OOM_ASSERT(cap_data);
        if (cap_data)
        {
            cap_data[0] = AV_SC_MEDIA_TRS;
            cap_data[1] = 0;

            bmemcpy(cap_data + 2, &av_cap_rsp[codec_sel], av_cap_rsp[codec_sel][1] + 2);

            // *(cap_data + cap_len - 6) = AV_SC_CONT_PROTECTION;
            // *(cap_data + cap_len - 5) = 2;
            // *(cap_data + cap_len - 4) = 2;
            // *(cap_data + cap_len - 3) = 0;



            // *(cap_data + cap_len - 2) = AV_SC_DELAY_REPORTING;
            // *(cap_data + cap_len - 1) = 0;

            av_get_all_capabilities_rsp_acp(msg->conn_id,
                                            msg->tlabel,
                                            cap_len,
                                            cap_data);
            bfree(cap_data);
        }
    }
    else
    {
        INFO_TRACE(">> reject the get all capabilities indication\n");
        av_get_all_capabilities_rsp_rej(msg->conn_id, msg->tlabel, AV_BAD_ACP_SEID);
    }
#endif
}


static void bt_av_get_sbc_cfg(uint8_t *app_serv_cap, uint8_t app_serv_length, bts2_sbc_cfg *act_cfg)
{

    RT_ASSERT(app_serv_length == SBC_MEDIA_CODEC_SC_SIZE);

    app_serv_cap[0] = AV_SC_MEDIA_CODEC;
    app_serv_cap[1] = SBC_MEDIA_CODEC_SC_SIZE - 2;
    app_serv_cap[2] = AV_AUDIO << 4;
    app_serv_cap[3] = AV_SBC;
    switch (act_cfg->sample_freq)
    {
    case 16000:
        app_serv_cap[4] = 0x80;
        break;
    case 32000:
        app_serv_cap[4] = 0x40;
        break;
    case 44100:
        app_serv_cap[4] = 0x20;
        break;
    case 48000:
        app_serv_cap[4] = 0x10;
        break;
    }
    switch (act_cfg->chnl_mode)
    {
    case SBC_MONO:
        app_serv_cap[4] += 0x08;
        break;
    case SBC_DUAL:
        app_serv_cap[4] += 0x04;
        break;
    case SBC_STEREO:
        app_serv_cap[4] += 0x02;
        break;
    case SBC_JOINT_STEREO:
        app_serv_cap[4] += 0x01;
        break;
    }
    switch (act_cfg->blocks)
    {
    case 4:
        app_serv_cap[5] = 0x80;
        break;
    case 8:
        app_serv_cap[5] = 0x40;
        break;
    case 12:
        app_serv_cap[5] = 0x20;
        break;
    case 16:
        app_serv_cap[5] = 0x10;
        break;
    }
    if (act_cfg->subbands == 8)
    {
        app_serv_cap[5] += 0x04;
    }
    else if (act_cfg->subbands == 8)
    {
        app_serv_cap[5] += 0x08;
    }
    app_serv_cap[5] += act_cfg->alloc_method;
    app_serv_cap[6] = act_cfg->min_bitpool;
    app_serv_cap[7] = act_cfg->max_bitpool;

}


static uint8_t bt_av_prepare_sbc(bts2s_av_inst_data *inst, U8 con_idx)
{
    uint8_t ret = 0x1;

#ifdef CFG_AV_SNK
    if (inst->con[con_idx].cfg == AV_AUDIO_SNK
            && bt_avsnk_prepare_sbc(inst, con_idx) != 0)
        ret = 0;
#endif //CFG_AV_SNK

#ifdef CFG_AV_SRC
    if (inst->con[con_idx].cfg == AV_AUDIO_SRC &&
            bt_avsrc_prepare_sbc(inst, con_idx) != 0)
        ret = 0;
#endif // CFG_AV_SRC
    return ret;
}

static void bt_av_hdl_get_cfg_cfm(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_GET_CFG_CFM *msg;
    msg = (BTS2S_AV_GET_CFG_CFM *)bts2_app_data->recv_msg;

    if (msg->res == AV_ACPT)
    {
        INFO_TRACE("get the configuration of the stream\n");
        bfree(msg->serv_cap_data);
    }
    else
    {
        INFO_TRACE("<< not get the configuration of the stream\n");
    }
}


static void bt_av_hdl_get_cfg_ind(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_GET_CFG_IND *msg;
    U8 con_idx;
    msg = (BTS2S_AV_GET_CFG_IND *)bts2_app_data->recv_msg;
    con_idx = bt_av_get_idx_from_shdl(inst, msg->shdl);
    INFO_TRACE(">> indicate to get configuration of the stream\n");

#ifdef TP_SIG_SMG_BI_10_C
    av_get_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_BAD_ACP_SEID);
#elif defined(TP_SIG_SMG_BI_12_C)
    av_get_cfg_rsp_rej(msg->shdl, msg->tlabel, 0xE1);
#else
    if (inst->con[con_idx].stream_hdl == msg->shdl)
    {
        if ((inst->con[con_idx].st == avconned_open) || (inst->con[con_idx].st == avconned_streaming))
        {
            U8 app_serv_cap[SBC_MEDIA_CODEC_SC_SIZE];
            uint8_t app_serv_length = 0;
            uint8_t codec = inst->local_seid_info[inst->con[con_idx].local_seid_idx].local_seid.codec;
            if (codec == AV_SBC)
            {
                app_serv_length = SBC_MEDIA_CODEC_SC_SIZE;
                bt_av_get_sbc_cfg(app_serv_cap, app_serv_length, &inst->con[con_idx].act_cfg);
            }
#ifdef CFG_AV_AAC
            else if (codec == AV_MPEG24_AAC)
            {
                //TODO :AAC configure get
            }
#endif // CFG_AV_AAC
            else
                RT_ASSERT(0);

            if (app_serv_length != 0)
            {
                av_get_cfg_rsp_acp(msg->shdl, msg->tlabel, app_serv_length, app_serv_cap);
            }
            else
                av_get_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_INSUFFICIENT_RESRCS);
        }
        else
        {
            av_get_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_BAD_ST);
        }
    }
    else
    {
        INFO_TRACE(">> reject to the get configuration request\n");
        av_get_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_BAD_ACP_SEID);
    }
#endif
}



static void bt_av_hdl_set_cfg_cfm(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    U8 con_idx;
    BTS2S_AV_SET_CFG_CFM *msg;

    msg = (BTS2S_AV_SET_CFG_CFM *)bts2_app_data->recv_msg;
    con_idx = bt_av_get_idx_from_cid(inst, msg->conn_id);

    if (msg->res == AV_ACPT)
    {
        uint8_t is_start = 0;
        inst->con[con_idx].stream_hdl = msg->shdl;
        if (inst->local_seid_info[inst->con[con_idx].local_seid_idx].local_seid.codec == AV_SBC)
            if (!bt_av_prepare_sbc(inst, con_idx))
                is_start = 1;

        if (is_start)
        {
            inst->local_seid_info[inst->con[con_idx].local_seid_idx].local_seid.in_use = TRUE;

            /*rdy to open stream */
            INFO_TRACE(">> request to open the stream \n");
#ifdef TP_SIG_SMG_BI_20_C
            av_start_req(1, ASSIGN_TLABEL, &inst->con[inst->con_idx].stream_hdl);
#else
            av_open_req(inst->con[con_idx].stream_hdl, ASSIGN_TLABEL);
#endif
        }
        else
        {
            INFO_TRACE(">> Try open failed\n");
        }
    }
    else
    {
        INFO_TRACE("<< peer device reject the stream confirguration\n");
    }
}

static void bt_av_hdl_set_cfg_ind(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_SET_CFG_IND *msg;
    U8 res;
    U16 idx;
    U8 *serv_cap;
    U8 con_idx;
    U8 found = FALSE;
    U8 i = 0;
    U8 delay_report_enable = 0;
    U8 codec;
    msg = (BTS2S_AV_SET_CFG_IND *)bts2_app_data->recv_msg;
    USER_TRACE("<< indicate to set stream configuration\r\n");
    con_idx = bt_av_get_idx_from_cid(inst, msg->conn_id);
    if (con_idx >= MAX_CONNS)
    {
        return;
    }
    inst->con[con_idx].stream_hdl = msg->shdl; /*get tmp. shdl */
    serv_cap = msg->serv_cap_data;

#if defined(TP_SIG_SMG_BI_07_C)
    av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_SEP_IN_USE, *msg->serv_cap_data);
#elif defined(TP_SIG_SMG_BI_09_C)
    av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, 0xE1, *msg->serv_cap_data);
#else

#ifdef BSP_BQB_TEST
    switch (BQB_TEST_CASE)
    {
    case A2DP_SRC_AVP_BI_10_C:
        USER_TRACE("A2DP/SRC/AVP/BI-10-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_INVLD_OBJ_TYPE, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case A2DP_SRC_AVP_BI_11_C:
        USER_TRACE("A2DP/SRC/AVP/BI-11-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_INVLD_CHNLS, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case BQB_BI_03_C:
    case A2DP_SRC_AVP_BI_12_C:
        USER_TRACE("BQB_BI_03_C or A2DP/SRC/AVP/BI-12-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_INVLD_SAMPLING_FREQ, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case A2DP_SRC_AVP_BI_14_C:
        USER_TRACE("A2DP/SRC/AVP/BI-14-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_NOT_SUPP_VBR, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case A2DP_SRC_AVP_BI_16_C:
        USER_TRACE("A2DP/SRC/AVP/BI-16-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_NOT_SUPP_OBJ_TYPE, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case A2DP_SRC_AVP_BI_17_C:
        USER_TRACE("A2DP/SRC/AVP/BI-17-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_NOT_SUPP_CHNLS, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case BQB_BI_08_C:
    case A2DP_SRC_AVP_BI_18_C:
        USER_TRACE("BQB_BI_08_C or A2DP/SRC/AVP/BI-18-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_NOT_SUPP_SAMPLING_FREQ, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case BQB_BI_10_C:
    case A2DP_SRC_AVP_BI_20_C:
        USER_TRACE("BQB_BI_10_C or A2DP/SRC/AVP/BI-20-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_INVLD_CODEC_TYPE, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case BQB_BI_20_C:
    case A2DP_SRC_AVP_BI_30_C:
        USER_TRACE("BQB_BI_20_C or A2DP/SRC/AVP/BI-30-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_NOT_SUPP_CODEC_TYPE, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case BQB_BI_11_C:
    case A2DP_SRC_AVP_BI_21_C:
        USER_TRACE("BQB_BI_11_C or A2DP/SRC/AVP/BI-21-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_INVLD_CHNL_MODE, *msg->serv_cap_data);
        return;
    case A2DP_SRC_AVP_BI_31_C:
        USER_TRACE("A2DP/SRC/AVP/BI-31-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_NOT_SUPP_CHNL_MODE, *msg->serv_cap_data);
        return;
    case BQB_BI_12_C:
        USER_TRACE("BQB_BI_12_C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_INVLD_SUBBANDS, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case A2DP_SRC_AVP_BI_32_C:
        USER_TRACE("A2DP/SRC/AVP/BI-32-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_NOT_SUPP_SUBBANDS, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case BQB_BI_13_C:
        USER_TRACE("BQB_BI_13_C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_INVLD_ALLOC_METHOD, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case A2DP_SRC_AVP_BI_33_C:
        USER_TRACE("A2DP/SRC/AVP/BI-33-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_NOT_SUPP_ALLOC_METHOD, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case BQB_BI_14_C:
    case A2DP_SRC_AVP_BI_24_C:
        USER_TRACE("BQB_BI_14_C or A2DP/SRC/AVP/BI-24-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_INVLD_MIN_BITPOOL, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case BQB_BI_15_C:
    case A2DP_SRC_AVP_BI_25_C:
        USER_TRACE("BQB_BI_15_C or A2DP/SRC/AVP/BI-25-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_INVLD_MAX_BITPOOL, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;
    case BQB_BI_16_C:
    case A2DP_SRC_AVP_BI_26_C:
        USER_TRACE("BQB_BI_16_C or A2DP/SRC/AVP/BI-26-C test!!!!!\n");
        av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_INVLD_BLOCK_LEN, *msg->serv_cap_data);
        bt_av_hdl_reset_bqb_test();
        return;

    default:
        break;
    }
#endif

    while ((i < MAX_NUM_LOCAL_SEIDS) && (!found))
    {
        if (inst->local_seid_info[i].local_seid.acp_seid == msg->acp_seid)
        {
            found = TRUE;
            break;
        }
        i++ ;
    }
    if (found)
    {
        if (inst->local_seid_info[i].local_seid.in_use == FALSE)
        {
            LOG_I("idx(%d) sep %d cfg %d", i, inst->local_seid_info[i].local_seid.sep_type, inst->con[con_idx].cfg);
            inst->con[con_idx].local_seid_idx = i;
            if (inst->con[con_idx].cfg == AV_AUDIO_NO_ROLE)
                inst->con[con_idx].cfg = bt_av_get_role_from_sep_type(inst->local_seid_info[i].local_seid.sep_type);
            else
            {
                if ((inst->con[con_idx].cfg == AV_AUDIO_SNK && inst->local_seid_info[i].local_seid.sep_type == AV_SRC)
                        || (inst->con[con_idx].cfg == AV_AUDIO_SRC && inst->local_seid_info[i].local_seid.sep_type == AV_SNK))
                {
                    LOG_E("Local role is unexpected!!!");
                    inst->con[con_idx].cfg = AV_AUDIO_NO_ROLE;
                }
            }

            if (inst->con[con_idx].cfg == AV_AUDIO_NO_ROLE)
            {
                av_set_cfg_rsp_rej(msg->shdl, msg->tlabel, AV_BAD_ST, *msg->serv_cap_data);
                bfree(msg->serv_cap_data);
                return;
            }
            idx = 0;
            res = AV_ACPT;
            codec = inst->local_seid_info[inst->con[con_idx].local_seid_idx].local_seid.codec;
            if (codec == AV_SBC)
            {
                while (serv_cap != NULL)
                {
                    if (*serv_cap == AV_SC_MEDIA_CODEC)
                    {
                        if (((*(serv_cap + 2)) >> 4 == AV_AUDIO) && (*(serv_cap + 3) == AV_SBC))
                        {
                            res = AV_ACPT;
                        }
                        else
                        {
                            res = AV_UNSUPP_CFG;
                            break;
                        }
                    }
                    else if (*serv_cap == AV_SC_CONT_PROTECTION)
                    {
                        /*cont protection is not supp by this app */
                        // res = AV_UNSUPP_CFG;
                        // break;
                    }
                    else if (*serv_cap == AV_SC_DELAY_REPORTING)
                    {
                        delay_report_enable = 1;
                    }
                    else
                    {
                        res = av_vldate_svc_cap(serv_cap);
                        if (res != AV_ACPT)
                        {
                            break;
                        }
                    }
                    serv_cap = av_get_svc_cap(AV_SC_NEXT, msg->serv_cap_data, msg->serv_cap_len, &idx);
                }
            }
            else if (codec == AV_MPEG24_AAC)
            {
                //TODO: Parser AAC strcture
            }
            else
                res = AV_UNSUPP_CFG;

            USER_TRACE("CODEC %d\n", codec);
            if (res == AV_ACPT)
            {
                idx = 0;
                U8 p_ret = 1;
                if (codec == AV_SBC)
                {
                    serv_cap = av_get_svc_cap(AV_SC_MEDIA_CODEC, msg->serv_cap_data, msg->serv_cap_len, &idx);

                    bt_av_store_sbc_cfg(&inst->con[con_idx].act_cfg, serv_cap + 4);

                    p_ret = bt_av_prepare_sbc(inst, con_idx);
                }
                else if (codec == AV_MPEG24_AAC)
                {
                    //TODO: Prepare AAC
                }
                else
                    RT_ASSERT(0);

                if (p_ret == 0)
                {
                    inst->local_seid_info[inst->con[con_idx].local_seid_idx].local_seid.in_use = TRUE;
                    USER_TRACE(">> accept the stream configuration indication\n");
                    av_set_cfg_rsp_acp(msg->shdl, msg->tlabel);
                    bfree(msg->serv_cap_data);

                    if (delay_report_enable == 1 && inst->con[con_idx].cfg == AV_AUDIO_SNK)
                    {
                        // TODO: set real delay report value
                        av_delay_report_req(msg->shdl, msg->tlabel + 1, msg->int_seid, 1900);
                    }
                    return;
                }
                else
                {
                    /*if we fail to initialize SBC, some pars must be wrong */
                    res = AV_BAD_MEDIA_TRS_FMT;
                }
            }
        }
        else
        {
            res = AV_SEP_IN_USE;
            INFO_TRACE(" --- AV_SEP_IN_USE \n");
        }
    }
    else
    {
        res = AV_BAD_ACP_SEID;
    }
    /*reaching here means cfguration can not be approved, send rej rsp */
    INFO_TRACE(">> reject the stream configuration indication\n");
    av_set_cfg_rsp_rej(inst->con[con_idx].stream_hdl,
                       msg->tlabel,
                       res,
                       *serv_cap);
    inst->con[con_idx].stream_hdl = 0; /*not vld anymore */
#endif
    bfree(msg->serv_cap_data);
}

static void bt_av_hdl_cfg_cfm(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_CFG_CFM *msg;
    msg = (BTS2S_AV_CFG_CFM *) bts2_app_data->recv_msg;
    if (msg->res == AV_ACPT)
    {
        USER_TRACE("<< reconfigurate the stream success\n");
    }
    else
    {
        USER_TRACE("<< failed reconfigurate the stream \n");
    }
}


static void bt_av_hdl_cfg_ind(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_CFG_IND *msg;
    U8 res;
    U16 idx;
    U8 *serv_cap;
    U8 con_idx;

    msg = (BTS2S_AV_CFG_IND *)bts2_app_data->recv_msg;

#ifdef TP_SIG_SMG_BI_13_C
    INFO_TRACE(">> reject to reconfigurate the stream\n");
    av_recfg_rsp_rej(msg->shdl, msg->tlabel, AV_SEP_NOT_IN_USE, *msg->serv_cap_data);
    /*  av_recfg_rsp_rej(msg->shdl, msg->tlabel,AV_BAD_SERV_CATEGORY,*msg->serv_cap_data);*/
#elif defined(TP_SIG_SMG_BI_15_C)
    av_recfg_rsp_rej(msg->shdl, msg->tlabel, 0xE1, *msg->serv_cap_data);
#else
    con_idx = bt_av_get_idx_from_shdl(inst, msg->shdl);

    serv_cap = msg->serv_cap_data;

    if (inst->con[con_idx].stream_hdl == msg->shdl)
    {
        if (inst->local_seid_info[inst->con[con_idx].local_seid_idx].local_seid.in_use == TRUE)
        {
            idx = 0;
            res = AV_ACPT;

            while (serv_cap != NULL)
            {
                if (*serv_cap == AV_SC_MEDIA_CODEC)
                {
                    if (((*(serv_cap + 2)) >> 4 == AV_AUDIO) && ((*(serv_cap + 3) == AV_SBC)
#ifdef CFG_AV_AAC
                            || (*(serv_cap + 3) == AV_MPEG24_AAC)
#endif
                                                                ))
                    {
                        res = AV_ACPT;
                    }
                    else
                    {
                        res = AV_UNSUPP_CFG;
                        break;
                    }
                }
                else if (*serv_cap == AV_SC_MEDIA_TRS)
                {
                    /*media trs is not allowed in recfg */
                    res = AV_INVLD_CAPABILITY;
                    break;
                }
                else if (*serv_cap > AV_SC_MEDIA_CODEC)
                {
                    /*not defined in spec */
                    res = AV_BAD_SERV_CATEGORY;
                    break;
                }
                else
                {
                    /*the rest is unsupp */
                    res = AV_UNSUPP_CFG;
                    break;
                }
                serv_cap = av_get_svc_cap(AV_SC_NEXT, msg->serv_cap_data, msg->serv_cap_len, &idx);
            }
        }
        else
        {
            res = AV_SEP_NOT_IN_USE;
        }

        if (res == AV_ACPT)
        {
            idx = 0;
            U8 p_ret = 1;
            uint8_t codec = inst->local_seid_info[inst->con[con_idx].local_seid_idx].local_seid.codec;
            if (codec == AV_SBC)
            {
                serv_cap = av_get_svc_cap(AV_SC_MEDIA_CODEC, msg->serv_cap_data, msg->serv_cap_len, &idx);

                bt_av_store_sbc_cfg(&inst->con[con_idx].act_cfg, serv_cap + 4);
                p_ret = bt_av_prepare_sbc(inst, con_idx);
            }
            else if (codec == AV_MPEG24_AAC)
            {
                //TODO: Prepare AAC
            }
            else
                RT_ASSERT(0);

            if (p_ret == 0)
            {
                INFO_TRACE(">> accept to reconfigurate the stream\n");
                av_recfg_rsp_acp(msg->shdl, msg->tlabel);
                bfree(msg->serv_cap_data);
                return;
            }
            else
            {
                /*if we fail to initialize SBC, some pars must be wrong */
                res = AV_BAD_MEDIA_TRS_FMT;
            }
        }
    }
    else
    {
        res = AV_BAD_ACP_SEID;
    }

    /*reaching here means cfguration can not be approved, send rej rsp */
    INFO_TRACE(">> reject to reconfigurate the stream\n");
    av_recfg_rsp_rej(msg->shdl,
                     msg->tlabel,
                     res,
                     *serv_cap);
#endif

    bfree(msg->serv_cap_data);
}



static void bt_av_hdl_open_cfm(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    U8 con_idx;
    BTS2S_AV_OPEN_CFM *msg;

    msg = (BTS2S_AV_OPEN_CFM *)bts2_app_data->recv_msg;
    con_idx = bt_av_get_idx_from_shdl(inst, msg->shdl);

    if (msg->res == AV_ACPT)
    {
        INFO_TRACE("<< peer device accept to open stream\n");
        // INFO_TRACE(">> request to start the av stream\n");
        /*av_start_req(1, ASSIGN_TLABEL, &inst->con[con_idx].stream_hdl);*/
        inst->con[con_idx].st = avconned_open;
    }
    else
    {
        INFO_TRACE("<< peer device reject to open stream\n");
    }
}

static void bt_av_hdl_open_ind(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_OPEN_IND *msg;
    U8 con_idx;
    msg = (BTS2S_AV_OPEN_IND *)bts2_app_data->recv_msg;
    con_idx = bt_av_get_idx_from_shdl(inst, msg->shdl);
    if (con_idx > MAX_CONNS)
    {
        return;
    }
    else
    {
        INFO_TRACE("connect index %d\n", con_idx);
    }

#ifdef TP_SIG_SMG_BI_16_C
    av_open_rsp_rej(msg->shdl, msg->tlabel, AV_BAD_ST);
#elif defined(TP_SIG_SMG_BI_18_C)
    av_open_rsp_rej(msg->shdl, msg->tlabel, 0xE1);
#else
    INFO_TRACE(">> indicate to open stream\n");
    if (inst->con[con_idx].stream_hdl == msg->shdl) // AV profile record the local seid
    {
        INFO_TRACE(">> accept to open the av stream\n");
        av_open_rsp_acp(msg->shdl, msg->tlabel);
        inst->con[con_idx].st = avconned_open;
    }
    else
    {
        INFO_TRACE(">> reject to open the av stream\n");
        av_open_rsp_rej(inst->con[con_idx].stream_hdl, msg->tlabel, AV_BAD_ACP_SEID);
    }
#endif
}

static void bt_av_hdl_start_cfm(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_START_CFM *msg;
    U8 con_idx;

    msg = (BTS2S_AV_START_CFM *)bts2_app_data->recv_msg;
    con_idx = bt_av_get_idx_from_shdl(inst, msg->rej_shdl);
    if (con_idx > MAX_CONNS)
    {
        return;
    }
    else
    {
        INFO_TRACE("connect index %d\n", con_idx);
    }

    if (msg->res == AV_ACPT)
    {
        int8_t ret = -1;
        USER_TRACE(">> peer device accept to start stream \n");
#ifdef CFG_AV_SNK
        if (inst->con[con_idx].cfg == AV_AUDIO_SNK)
            ret = bt_avsnk_hdl_start_cfm(inst, con_idx);
#endif // CFG_AV_SNK

#ifdef CFG_AV_SRC
        if (inst->con[con_idx].cfg == AV_AUDIO_SRC)
            ret = bt_avsrc_hdl_start_cfm(inst, con_idx);
#endif //CFG_AV_SRC

        if (!ret)
        {
            inst->con[con_idx].st = avconned_streaming;
#ifdef CFG_AV_SRC
            bt_avsrc_set_start_flag(TRUE);
            bt_avsrc_hdl_streaming_start(inst, con_idx);
            U8 play_status = bt_av_get_a2dp_stream_state();
#ifdef CFG_AVRCP
            bt_avrcp_change_play_status(bts2_app_data, play_status);
#ifdef BSP_BQB_TEST
            bt_avrcp_track_changed_register_response(bts2_app_data, AVRCP_CR_CHANGED, 0);
#endif
#endif
#endif // CFG_AV_SRC
        }
    }
    else
    {
        USER_TRACE(">> peer device reject to start stream \n");
    }
#ifdef CFG_AV_SRC
    if (inst->con[con_idx].cfg == AV_AUDIO_SRC)
    {
        inst->con[con_idx].is_start_cfg = 1;
    }
#endif //CFG_AV_SRC
}

static void bt_av_hdl_start_ind(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_START_IND *msg;
    U8 res = AV_BAD_ST;
    U8 shdl = 0;
    U8 con_idx;
    msg = (BTS2S_AV_START_IND *)bts2_app_data->recv_msg;

#ifdef TP_SIG_SMG_BI_19_C
    av_start_rsp_rej(*(msg->first_shdl), msg->tlabel, AV_BAD_ST, msg->list_len, msg->first_shdl);
#elif defined(TP_SIG_SMG_BI_21_C)
    av_start_rsp_rej(*(msg->first_shdl), msg->tlabel, 0xE1, msg->list_len, msg->first_shdl);
#else
    con_idx = bt_av_get_idx_from_shdl(inst, msg->first_shdl);

    if (con_idx > MAX_CONNS)
    {
        av_start_rsp_rej(shdl, msg->tlabel, AV_BAD_ST, (U8)msg->list_len, &msg->first_shdl);
        return;
    }
    else
    {
        INFO_TRACE("connect index %d\n", con_idx);
    }


#ifdef CFG_AV_SNK
    if (inst->con[con_idx].cfg == AV_AUDIO_SNK)
        res = bt_avsnk_hdl_start_ind(inst, msg, con_idx);
#endif // CFG_AV_SNK

#ifdef CFG_AV_SRC
    if (inst->con[con_idx].cfg == AV_AUDIO_SRC)
        res = bt_avsrc_hdl_start_ind(inst, msg, con_idx);
#endif // CFG_AV_SRC

    if (res == AV_ACPT)
    {
        inst->con[con_idx].st = avconned_streaming;
        av_start_rsp_acp(msg->tlabel, (U8)msg->list_len, &msg->first_shdl);
    }
    else
    {
        INFO_TRACE(">> reject to start the stream\n");
        av_start_rsp_rej(shdl, msg->tlabel, res, (U8)msg->list_len, &msg->first_shdl);
    }
#endif
}


static void bt_av_hdl_streamdata_ind(bts2_app_stru *bts2_app_data)
{

    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_STREAM_DATA_IND *msg;
    U8 con_idx;
    int ret = -1;

    msg = (BTS2S_AV_STREAM_DATA_IND *)bts2_app_data->recv_msg;

    con_idx = bt_av_get_idx_from_shdl(inst, (U8)msg->shdl);

    if (inst->con[con_idx].st != avconned_streaming)
    {
        bfree(msg->data);
        return;
    }
    else
    {
#ifdef CFG_AV_SNK
        if (inst->con[con_idx].cfg == AV_AUDIO_SNK)
            bt_avsnk_hdl_streamdata_ind(inst, con_idx, msg);
        else // Should only receive stream in snk role
#endif
        {
            LOG_W("current role %d", inst->con[con_idx].cfg);
            bfree(msg->data);
        }
    }


}



static void bt_av_hdl_suspend_cfm(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_SUSPEND_CFM *msg;
    uint8_t con_idx;
    INFO_TRACE("<< peer device response to suspend av stream\n");
    msg = (BTS2S_AV_SUSPEND_CFM *)bts2_app_data->recv_msg;

    con_idx = bt_av_get_idx_from_shdl(inst, msg->rej_shdl);

    if (con_idx > MAX_CONNS)
    {
        return;
    }
    else
    {
        INFO_TRACE("suspend connect index %d\n", con_idx);
    }

    if (msg->res == AV_ACPT)
    {
        INFO_TRACE("<< peer deviece agree to suspend stream\n");
        inst->con[con_idx].st = avconned_open;
#ifdef CFG_AV_SRC
        inst->src_data.stream_frm_time_begin = 0;
        U8 play_status = bt_av_get_a2dp_stream_state();
#ifdef  CFG_AVRCP
        bt_avrcp_change_play_status(bts2_app_data, play_status);
#endif
#endif
    }
    else
    {
        INFO_TRACE("<< peer deviece reject to suspend stream\n");
    }

    inst->suspend_pending = FALSE;
#ifdef CFG_AV_SRC
    if (inst->con[con_idx].cfg == AV_AUDIO_SRC)
    {
        inst->con[con_idx].is_suspend_cfg = 1;
    }
#endif //CFG_AV_SRC
}


static void bt_av_hdl_suspend_ind(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_SUSPEND_IND *msg;
    U8 con_idx;
    INFO_TRACE("<< peer device indicate to suspend av stream\n");
    msg = (BTS2S_AV_SUSPEND_IND *)bts2_app_data->recv_msg;
#ifdef TP_SIG_SMG_BI_25_C
    av_suspend_rsp_rej(msg->first_shdl, msg->tlabel, AV_BAD_ST, msg->list_len, &(msg->first_shdl));
#elif defined(TP_SIG_SMG_BI_27_C)
    av_suspend_rsp_rej(msg->first_shdl, msg->tlabel, 0XE1, msg->list_len, &(msg->first_shdl));
#else
    con_idx = bt_av_get_idx_from_shdl(inst, msg->first_shdl);

    if (con_idx > MAX_CONNS)
    {
        return;
    }
    else
    {
        INFO_TRACE("connect index %d\n", con_idx);
    }

    if (inst->con[con_idx].stream_hdl == msg->first_shdl)
    {
        INFO_TRACE(">> accept to suspend the av stream\n");
#ifdef CFG_AV_SNK
        if (inst->con[con_idx].cfg == AV_AUDIO_SNK)
            bt_avsnk_hdl_suspend_ind(inst, con_idx);
#endif // CFG_AV_SNK
#ifdef CFG_AV_SRC
        if (inst->con[con_idx].cfg == AV_AUDIO_SRC)
            bt_avsrc_hdl_suspend_ind(inst, con_idx);
#endif //CFG_AV_SRC
        av_suspend_rsp_acp(msg->tlabel, msg->list_len, &(msg->first_shdl));
        inst->con[con_idx].st = avconned_open;
        inst->local_seid_info[inst->con[con_idx].local_seid_idx].local_seid.in_use = TRUE;
    }
    else
    {
        INFO_TRACE(">> reject to suspend the av stream\n");
        av_suspend_rsp_rej(inst->con[con_idx].stream_hdl, msg->tlabel, AV_BAD_ACP_SEID, msg->list_len, &(msg->first_shdl));
    }
#endif
}


static void bt_av_hdl_abort_ind(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_ABORT_IND *msg;
    U8 con_idx;
    INFO_TRACE("<< peer device indicate to abort av stream\n");
    msg = (BTS2S_AV_ABORT_IND *)bts2_app_data->recv_msg;
    con_idx = bt_av_get_idx_from_shdl(inst, msg->shdl);

    if (con_idx > MAX_CONNS)
    {
        return;
    }
    else
    {
        INFO_TRACE("abort index %d\n", con_idx);
    }

    if (inst->con[con_idx].stream_hdl == msg->shdl)
    {
        INFO_TRACE(">> accept to abort the av stream\n");
#ifdef CFG_AV_SRC
        if (inst->con[con_idx].cfg == AV_AUDIO_SRC)
            bt_avsrc_hdl_abort_ind(inst, con_idx);
#endif
        av_abort_rsp(msg->shdl, msg->tlabel);
    }
    else
    {
    }
}

static void bt_av_hdl_close_ind(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_CLOSE_IND *msg;
    U8 con_idx;
    msg = (BTS2S_AV_CLOSE_IND *)bts2_app_data->recv_msg;

#ifdef TP_SIG_SMG_BI_22_C
    av_close_rsp_rej(msg->shdl, msg->tlabel, AV_BAD_ST);

#elif defined(TP_SIG_SMG_BI_24_C)
    av_close_rsp_rej(msg->shdl, msg->tlabel, 0xE1);
#else

    USER_TRACE("<< peer device indicate to realease av stream\n");
    con_idx = bt_av_get_idx_from_shdl(inst, msg->shdl);
    if (con_idx > MAX_CONNS)
    {
        return;
    }
    else
    {
        INFO_TRACE("connect index %d\n", con_idx);
    }
    if (inst->con[con_idx].stream_hdl == msg->shdl)
    {
        USER_TRACE(">> accept to release the av stream\n");
        if (inst->con[con_idx].st == avconned_streaming)
        {
#ifdef CFG_AV_SNK
            if (inst->con[con_idx].cfg == AV_AUDIO_SNK)
                bt_avsnk_close_handler(inst, con_idx);
#endif // CFG_AV_SNK

#ifdef CFG_AV_SRC
            if (inst->con[con_idx].cfg == AV_AUDIO_SRC)
                bt_avrc_close_handler(inst, con_idx);
#endif //CFG_AV_SRC
        }
        av_close_rsp_acp(msg->shdl, msg->tlabel);
        inst->con[con_idx].st = avconned;
        inst->local_seid_info[inst->con[con_idx].local_seid_idx].local_seid.in_use = FALSE;

        bt_av_recovery_local_seid(inst, inst->con[con_idx].cfg);
    }
    else
    {
        USER_TRACE(">> reject to release the av stream\n");
        av_close_rsp_rej(inst->con[con_idx].stream_hdl, msg->tlabel, AV_BAD_ACP_SEID);
    }
#endif
}

static void bt_av_hdl_stream_mtu_size_ind(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_STREAM_MTU_SIZE_IND *msg;
    U8 con_idx;

    msg = (BTS2S_AV_STREAM_MTU_SIZE_IND *)bts2_app_data->recv_msg;
    inst->max_frm_size = msg->rmt_mtu_size;
    INFO_TRACE("<< recive remote av MUT information,max_frm_size = %d\n", inst->max_frm_size);

    U16 dft_mtu_size = bts2_get_l2cap_default_mtu_size();
    /*if def_max_MTU < rmt MTU < (2 *def_max_MTU + 2), use def_max_MTU which
       matches the payload size of a DH5 pkt */
    if ((msg->rmt_mtu_size < ((dft_mtu_size   << 1) + 2))
            && (msg->rmt_mtu_size > dft_mtu_size))
    {
        inst->max_frm_size = dft_mtu_size ;
    }

    con_idx = bt_av_get_idx_from_shdl(inst, msg->shdl);

#ifdef RT_USING_UTEST
    if (inst->con[con_idx].cfg == AV_AUDIO_SRC)
    {
#ifdef AUDIO_USING_MANAGER
        audio_server_select_private_audio_device(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_DEVICE_A2DP_SINK);
#endif
    }
#endif

#if defined(CFG_AV)
    bt_notify_profile_state_info_t profile_state;
    bt_addr_convert(&inst->con[con_idx].av_rmt_addr, profile_state.mac.addr);
    profile_state.profile_type = BT_NOTIFY_A2DP;
    profile_state.res = BTS2_SUCC;
    bt_interface_bt_event_notify(BT_NOTIFY_A2DP, BT_NOTIFY_A2DP_PROFILE_CONNECTED,
                                 &profile_state, sizeof(bt_notify_profile_state_info_t));
#endif
}

static void bt_av_hdl_delay_report_ind(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    BTS2S_AV_DELAY_REPORT_IND *msg;

    INFO_TRACE("<< recive remote av delay report\n");
    msg = (BTS2S_AV_DELAY_REPORT_IND *)bts2_app_data->recv_msg;


    av_delay_report_rsp_acp(msg->shdl, msg->tlabel);
}

static void bt_av_hdl_conn_cfm(bts2_app_stru *bts2_app_data)
{
    U8 i = 0;
    U8 j = 0;
    U8 found = FALSE;
    BTS2S_AV_CONN_CFM *msg;
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    msg = (BTS2S_AV_CONN_CFM *)bts2_app_data->recv_msg;
    if (msg->res != AV_ACPT)
    {
#if defined(CFG_AV)
        bt_notify_profile_state_info_t profile_state;
        bt_addr_convert(&msg->bd, profile_state.mac.addr);
        profile_state.profile_type = BT_NOTIFY_A2DP;
        profile_state.res = msg->res;
        bt_interface_bt_event_notify(BT_NOTIFY_A2DP, BT_NOTIFY_A2DP_PROFILE_DISCONNECTED,
                                     &profile_state, sizeof(bt_notify_profile_state_info_t));
#endif
        USER_TRACE(" -- a2dp connect failed %x\n", msg->res);
        return;
    }
    while ((i < MAX_CONNS) && (!found))
    {
        if (inst->con[i].in_use == FALSE)
        {
            found = TRUE;
            break;
        }
        i++ ;
    }

    if (found)
    {
        inst->con[i].in_use = TRUE;
        inst->con[i].role = INITIATOR;
        inst->con[i].conn_id = msg->conn_id;
        inst->con[i].st = avconned;
        inst->con[i].cfg = msg->local_role;
        bd_copy(&inst->con[i].av_rmt_addr, &msg->bd);
        USER_TRACE("<< av connect success,cfg = %d\n", inst->con[i].cfg);

        INFO_TRACE("URC av conn,cfm\n");

        INFO_TRACE(">> discover the remote device's audio end point\n");

        av_discover_req(inst->con[i].conn_id, ASSIGN_TLABEL);
    }
    else
    {
        INFO_TRACE(" -- no free connection\n");
    }
}


static void bt_av_hdl_conn_ind(bts2_app_stru *bts2_app_data)
{
    BTS2S_AV_CONN_IND *msg;
    U8 i = 0;
    U8 found = FALSE;
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    msg = (BTS2S_AV_CONN_IND *)bts2_app_data->recv_msg;
    USER_TRACE("<< indicate to connect to remote device\n");
    while ((i < MAX_CONNS) && (!found))
    {
        if (inst->con[i].in_use == FALSE)
        {
            found = TRUE;
            break;
        }
        i++ ;
    }
    if (found)
    {
        inst->con[i].in_use = TRUE;
        inst->con[i].role = ACPTOR;
        inst->con[i].conn_id = msg->conn_id;
        inst->con[i].st = avconned;
        bd_copy(&inst->con[i].av_rmt_addr, &msg->bd);

        INFO_TRACE("URC av conn,ind\n");

#ifdef BSP_BQB_TEST
        switch (BQB_TEST_CASE)
        {
        case A2DP_SRC_AS_BV_01_I:
        case A2DP_SRC_AS_BI_01_I:
        case A2DP_SRC_REL_BV_01_I:
        case A2DP_SRC_REL_BV_02_I:
        case A2DP_SRC_SET_BV_01_I:
        case A2DP_SRC_SET_BV_03_I:
        case A2DP_SRC_SET_BV_05_I:
        case A2DP_SRC_SUS_BV_01_I:
        case A2DP_SRC_SUS_BV_02_I:
            USER_TRACE("BQB test for A2DP/SRC/AS/BV-01-I or A2DP/SRC/AS/BI-01-I\n");
            av_discover_req(inst->con[i].conn_id, ASSIGN_TLABEL);
            break;
        default:
            break;
        }
#endif
    }
    else
    {
        INFO_TRACE(" -- no free connection\n");
    }
}


static void bt_av_hdl_disc_ind(bts2_app_stru *bts2_app_data)
{
    BTS2S_AV_DISC_IND *msg;
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    U8 con_idx;
    int local_seid_idx;
    msg = (BTS2S_AV_DISC_IND *)bts2_app_data->recv_msg;
    con_idx = bt_av_get_idx_from_cid(inst, msg->conn_id);


    if (con_idx > MAX_CONNS)
    {
        return;
    }
    else
    {
        INFO_TRACE("connect index %d\n", con_idx);
    }


    USER_TRACE("<< av indicate to disconnect to remote device\n");
    if (inst->con[con_idx].in_use == TRUE)
    {
        inst->con[con_idx].in_use = FALSE;
        inst->con[con_idx].st = avdisced;
        local_seid_idx = inst->con[con_idx].local_seid_idx;
        bd_set_empty(&inst->con[con_idx].av_rmt_addr);

        if (local_seid_idx < MAX_NUM_LOCAL_SEIDS)
        {
            /* inst->local_seid_info[local_seid_idx].is_enbd = FALSE;*/
            inst->local_seid_info[local_seid_idx].local_seid.in_use = FALSE;
        }

        bt_av_recovery_local_seid(inst, inst->con[con_idx].cfg);

        inst->con[con_idx].local_seid_idx = -1;

#ifdef CFG_AV_SNK
        if (inst->con[con_idx].cfg == AV_AUDIO_SNK)
        {
#if defined(CFG_AV)
            bt_notify_profile_state_info_t profile_state;
            bt_addr_convert(&msg->bd, profile_state.mac.addr);
            profile_state.profile_type = BT_NOTIFY_A2DP;
            profile_state.res = msg->res;
            bt_interface_bt_event_notify(BT_NOTIFY_A2DP, BT_NOTIFY_A2DP_PROFILE_DISCONNECTED,
                                         &profile_state, sizeof(bt_notify_profile_state_info_t));
            USER_TRACE("<< urc av disc\n");
#endif
            bt_avsnk_hdl_disc_handler(inst, con_idx);
        }
#endif // CFG_AV_SNK

#ifdef CFG_AV_SRC
        if (inst->con[con_idx].cfg == AV_AUDIO_SRC)
        {
#if defined(CFG_AV)
            bt_notify_profile_state_info_t profile_state;
            bt_addr_convert(&msg->bd, profile_state.mac.addr);
            profile_state.profile_type = BT_NOTIFY_A2DP;
            profile_state.res = msg->res;
            bt_interface_bt_event_notify(BT_NOTIFY_A2DP, BT_NOTIFY_A2DP_PROFILE_DISCONNECTED,
                                         &profile_state, sizeof(bt_notify_profile_state_info_t));
            USER_TRACE("<< urc av disc\n");
#endif
            bt_avsrc_hdl_disc_handler(inst, con_idx);
        }
#endif // CFG_AV_SRC
        inst->con[con_idx].cfg = AV_AUDIO_NO_ROLE;

    }

    if (inst->close_pending == TRUE)
    {
        bt_avsnk_close(inst);
    }
}




bts2s_av_inst_data *bt_av_get_inst_data(void)
{
    // global_inst  should already init
    RT_ASSERT(global_inst != NULL);
    return global_inst;
}


void bt_av_init(bts2_app_stru *bts2_app_data)
{
    bts2s_av_inst_data *inst;

    inst = bcalloc(1, sizeof(bts2s_av_inst_data));
    global_inst = inst;
    bt_av_init_data(inst, bts2_app_data);

#ifdef CFG_AV_SNK
    bt_avsnk_init(inst, bts2_app_data);
#endif

#ifdef CFG_AV_SRC
    bt_avsrc_init(inst, bts2_app_data);
#endif
}

U8 bt_av_conn_check(void)
{
    U8 i = 0;
    U8 found = FALSE;

    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    for (i = 0 ; i < MAX_CONNS; i++)
    {
        if ((avidle != inst->con[i].st) && (avdisced != inst->con[i].st))
        {
            found = TRUE;
            break;
        }
    }
    return found;
}


#ifdef CFG_AV_SNK
void bt_av_snk_open(void)
{
    bts2s_av_inst_data *inst;
    inst = global_inst;
    bt_avsnk_open(inst);
}
void bt_av_snk_close(void)
{
    bts2s_av_inst_data *inst;
    inst = global_inst;
    bt_avsnk_close(inst);
}
#endif

void bt_av_unregister_sdp(U16 local_role)
{
    av_unregister_sdp(local_role); //disable the svc
}

void bt_av_register_sdp(U16 local_role)
{
    av_register_sdp(local_role); //disable the svc
}

void bt_av_rel(void)
{
#ifdef CFG_AV_SNK
    bt_avsnk_rel(global_inst);
#endif

    bfree(global_inst);
    global_inst = NULL;
}

void bt_av_snk_close_boundary_condition(U16 type)
{
    switch (type)
    {
    case BTS2MU_AV_DISB_CFM:
    {
#if defined(CFG_AV)
        bt_interface_bt_event_notify(BT_NOTIFY_A2DP, BT_NOTIFY_AVSNK_CLOSE_COMPLETE, NULL, 0);
#endif
        INFO_TRACE("<< URC av had been disabled \n");
        bts2s_av_inst_data *inst = bt_av_get_inst_data();
        inst->close_pending = FALSE;

        break;
    }

    default:
        ;
    }
}
void bt_av_msg_handler(bts2_app_stru *bts2_app_data)
{
    U16 *msg_type;
    msg_type = (U16 *)bts2_app_data->recv_msg;
#ifdef CFG_OPEN_AVSNK
    if (0x00 == bts2s_avsnk_openFlag)
    {
        bt_av_snk_close_boundary_condition(*msg_type);
        return;
    }
#endif
    switch (*msg_type)
    {
    case BTS2MU_AV_ERROR_IND:
    {
        break;
    }
    case BTS2MU_AV_ENB_CFM:
    {
        bt_av_hdl_enb_cfm(bts2_app_data);
        INFO_TRACE("<< av had been enabled \n");
        break;
    }
    case BTS2MU_AV_DISB_CFM:
    {
        INFO_TRACE("<< -- av had been disabled \n");
        bts2s_av_inst_data *inst = bt_av_get_inst_data();
        inst->close_pending = FALSE;
        break;
    }
    case BTS2MU_AV_CONN_CFM :
    {
        bt_av_hdl_conn_cfm(bts2_app_data);
        break;
    }
    case BTS2MU_AV_CONN_IND:
    {
        bt_av_hdl_conn_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_DISC_IND:
    {
        bt_av_hdl_disc_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_DISCOVER_CFM :
    {
        bt_av_hdl_discover_cfm(bts2_app_data);
        break;
    }
    case BTS2MU_AV_DISCOVER_IND:
    {
        bt_av_hdl_discover_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_GET_CAPABILITIES_CFM:
    {
        bt_av_hdl_get_cap_cfm(bts2_app_data);
        break;
    }
    case BTS2MU_AV_GET_CAPABILITIES_IND:
    {
        bt_av_hdl_get_cap_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_GET_ALL_CAPABILITIES_IND:
    {
        bt_av_hdl_get_all_cap_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_GET_CFG_CFM:
    {
        bt_av_hdl_get_cfg_cfm(bts2_app_data);
        break;
    }
    case BTS2MU_AV_GET_CFG_IND:
    {
        INFO_TRACE("<< receive av get configuration indication\n");
        bt_av_hdl_get_cfg_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_SET_CFG_CFM:
    {
        bt_av_hdl_set_cfg_cfm(bts2_app_data);
        break;
    }
    case BTS2MU_AV_SET_CFG_IND:
    {
        bt_av_hdl_set_cfg_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_CFG_CFM:
    {
        bt_av_hdl_cfg_cfm(bts2_app_data);
        break;
    }
    case BTS2MU_AV_CFG_IND:
    {
        bt_av_hdl_cfg_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_OPEN_CFM:
    {
        bt_av_hdl_open_cfm(bts2_app_data);
        break;
    }
    case BTS2MU_AV_OPEN_IND:
    {
        bt_av_hdl_open_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_START_CFM:
    {
        bt_av_hdl_start_cfm(bts2_app_data);
        break;
    }
    case BTS2MU_AV_START_IND:
    {
#if defined(CFG_AV)
        bt_interface_bt_event_notify(BT_NOTIFY_A2DP, BT_NOTIFY_A2DP_START_IND, NULL, 0);
#endif

        bt_av_hdl_start_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_QOS_IND:
    {
        BTS2S_AV_QOS_IND *msg;
        msg = (BTS2S_AV_QOS_IND *)bts2_app_data->recv_msg;
        break;
    }
    case BTS2MU_AV_STREAM_DATA_IND:
    {
        bt_av_hdl_streamdata_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_SUSPEND_CFM:
    {
        bt_av_hdl_suspend_cfm(bts2_app_data);
        break;
    }
    case BTS2MU_AV_SUSPEND_IND:
    {
        bt_av_hdl_suspend_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_ABORT_CFM:
    {
        BTS2S_AV_ABORT_CFM *msg;
        msg = (BTS2S_AV_ABORT_CFM *)bts2_app_data->recv_msg;
        INFO_TRACE(" -- the stream had been aborted \n");
        break;
    }
    case BTS2MU_AV_ABORT_IND:
    {
        bt_av_hdl_abort_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_CLOSE_CFM:
    {
        BTS2S_AV_CLOSE_CFM *msg;
        U8 con_idx;
        bts2s_av_inst_data *inst = bt_av_get_inst_data();
        msg = (BTS2S_AV_CLOSE_CFM *)bts2_app_data->recv_msg;
        INFO_TRACE("<< receive av close stream confirmation\n");

        con_idx = bt_av_get_idx_from_shdl(inst, msg->shdl);
        if (con_idx > MAX_CONNS)
        {
            return;
        }
        else
        {
            INFO_TRACE("connect index %d\n", con_idx);
        }

        if ((inst->con[con_idx].stream_hdl == msg->shdl) && (msg->res == AV_ACPT))
        {
            inst->con[con_idx].st = avconned;
            inst->local_seid_info[inst->con[con_idx].local_seid_idx].local_seid.in_use = FALSE;

            bt_av_recovery_local_seid(inst, inst->con[con_idx].cfg);
        }

        U8 play_status = bt_av_get_a2dp_stream_state();
#ifdef CFG_AVRCP
        bt_avrcp_change_play_status(bts2_app_data, play_status);
#endif
        break;
    }
    case BTS2MU_AV_CLOSE_IND:
    {
        bt_av_hdl_close_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_STREAM_MTU_SIZE_IND:
    {
        bt_av_hdl_stream_mtu_size_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AV_DELAY_REPORT_IND:
    {
        bt_av_hdl_delay_report_ind(bts2_app_data);
        break;
    }
    default:
        break;
    }

}


/**********************************************************
*
*          External function
*
***********************************************************/


void bt_av_conn(BTS2S_BD_ADDR *bd_addr, uint8_t peer_role)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();
    uint16_t local_role = AV_AUDIO_NO_ROLE;
    uint16_t peer_role_1 = AV_AUDIO_NO_ROLE;

#ifdef CFG_AV_SNK
    if (peer_role == AV_SRC)
    {
        local_role = AV_AUDIO_SRC;
        peer_role_1 = AV_AUDIO_SNK;
    }
#endif //CFG_AV_SNK

#ifdef CFG_AV_SRC
    if (peer_role == AV_SNK)
    {
        local_role = AV_AUDIO_SNK;
        peer_role_1 = AV_AUDIO_SRC;
    }
#endif // CFG_AV_SRC
    USER_TRACE(" -- av conn rmt device... peer_role:%d local_role:%d\n", peer_role_1, local_role);
    USER_TRACE(" -- address: %04X:%02X:%06lX\n",
               bd_addr->nap,
               bd_addr->uap,
               bd_addr->lap);

    if (local_role == AV_AUDIO_NO_ROLE
            || peer_role_1 == AV_AUDIO_NO_ROLE)
        LOG_E("Wrongly role!");
    else
        av_conn_req(bts2_task_get_app_task_id(), *bd_addr, peer_role_1, local_role);
}


void bt_av_disconnect(uint8_t con_idx)
{
    int local_seid_idx;
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    USER_TRACE(">> av dis st%d  local_seid_idx %x\n", inst->con[con_idx].st, inst->con[con_idx].local_seid_idx);

    if ((inst->con[con_idx].st != avdisced) && (avidle != inst->con[con_idx].st))
    {
        USER_TRACE(" -- disconnect to the source\n");
        av_disc_req(inst->con[con_idx].conn_id);
        inst->con[con_idx].st = avdisced;
        //inst->con[con_idx].in_use = FALSE;
        local_seid_idx = inst->con[con_idx].local_seid_idx;
        /*inst->local_seid_info[local_seid_idx].is_enbd = FALSE;*/
        if (local_seid_idx < MAX_NUM_LOCAL_SEIDS)
        {
            inst->local_seid_info[local_seid_idx].local_seid.in_use = FALSE;
        }

        bt_av_recovery_local_seid(inst, inst->con[con_idx].cfg);
    }
}


void bt_av_open_stream(void)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    INFO_TRACE(">> request to open the stream \n");
    av_open_req(inst->con[0].stream_hdl, ASSIGN_TLABEL);
}


void bt_av_suspend_stream(uint8_t con_idx)
{

    bts2s_av_inst_data *inst = bt_av_get_inst_data();
#ifdef  TP_SIG_SMG_BI_26_C
    av_suspend_req((U8)1, inst->tlabel, &(inst->con[con_idx].stream_hdl));
#else
    if (inst->con[con_idx].st == avconned_streaming)
    {
        USER_TRACE(">> suspend the av stream \n");
        ASSIGN_TLABEL;
        av_suspend_req((U8)1, inst->tlabel, &(inst->con[con_idx].stream_hdl));
        inst->suspend_pending = TRUE;
    }
    else
    {
        USER_TRACE(" -- the stream is not streaming, can not be suspend!\n");
    }
#endif
}

void bt_av_start_stream(uint8_t con_idx)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    if (inst->con[con_idx].st == avconned_open)
    {
        USER_TRACE(">> start the av stream\n");
        av_start_req(1, ASSIGN_TLABEL, &inst->con[con_idx].stream_hdl);
        inst->suspend_pending = FALSE;
    }
    else
    {
        if (inst->con[con_idx].st == avconned_streaming)
        {
            USER_TRACE(" -- the stream is already started !\n");
        }
        else
        {
            USER_TRACE(" -- the stream(%d) can not start due to status error!\n", con_idx);
        }
    }
}

#ifdef AUDIO_USING_MANAGER
void bt_av_start_stream_by_audio_server(uint8_t con_idx)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    if (inst->con[con_idx].st == avconned_open)
    {
        if (inst->con[con_idx].cfg == AV_AUDIO_SRC)
        {
            inst->con[con_idx].is_start_cfg = 0;
        }
    }

    bt_av_start_stream(con_idx);

    if (inst->con[con_idx].st == avconned_open)
    {
        if (inst->con[con_idx].cfg == AV_AUDIO_SRC)
        {
            bt_av_wait_var_set(&inst->con[con_idx].is_start_cfg);
        }
    }
}


void bt_av_suspend_stream_by_audio_server(uint8_t con_idx)
{

    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    if (inst->con[con_idx].st == avconned_streaming)
    {
        //fix bug: fast click pause/play, tws disconnect with me
        if (inst->con[con_idx].cfg == AV_AUDIO_SRC)
        {
            inst->con[con_idx].is_suspend_cfg = 0;
        }
    }

    bt_av_suspend_stream(con_idx);

    if (inst->con[con_idx].st == avconned_streaming)
    {
        //fix bug: fast click pause/play, tws disconnect with me
        if (inst->con[con_idx].cfg == AV_AUDIO_SRC)
        {
            bt_av_wait_var_set(&inst->con[con_idx].is_suspend_cfg);
        }
    }
}
#endif


void bt_av_release_stream(uint8_t con_idx)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    if ((inst->con[con_idx].st == avconned_open) || (inst->con[con_idx].st == avconned_streaming))
    {
        // inst->con[inst->con_idx].st = avconned;
        USER_TRACE(">> to release the av stream\n");
        av_close_req(inst->con[inst->con_idx].stream_hdl, ASSIGN_TLABEL);
    }
    else
    {
        USER_TRACE(" -- the stream(%d) is not opened/streaming and can not be released!\n", con_idx);
    }
}

void bt_av_abort_stream(uint8_t con_idx)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    INFO_TRACE(">> request to abort the av stream \n");
    if (inst->con[con_idx].st != avdisced
            && inst->con[con_idx].st != avidle)
    {
        ASSIGN_TLABEL;
        av_abort_req(inst->con[con_idx].stream_hdl, inst->tlabel);
        inst->con[con_idx].st = avdisced;
    }
}

void bt_av_get_cfg(uint8_t con_idx)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    if (inst->con[con_idx].st == avconned_open)
    {
        USER_TRACE(">> get the configuration of the stream\n");
        av_get_cfg_req(inst->con[con_idx].stream_hdl, ASSIGN_TLABEL);
    }
    else
    {
        if (inst->con[con_idx].st == avconned_streaming)
        {
            USER_TRACE(" -- the stream is streaming!\n");
        }
        else
        {
            USER_TRACE(" -- the stream(%d) could not get configure!\n", con_idx);
        }
    }
}



void bt_av_set_can_play(void)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    inst->snk_data.can_play = 1;
    inst->snk_data.reveive_start = 1;
}

void bt_av_set_filter_prompt_enable(U8 enable)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    inst->snk_data.filter_prompt_enable = enable;
}


U8 bt_av_get_filter_prompt_enable(void)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    return inst->snk_data.filter_prompt_enable;
}


void bt_av_set_slience_filter_enable(U8 enable)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    inst->snk_data.slience_filter_enable = enable;
}


U8 bt_av_get_slience_filter_enable(void)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    return inst->snk_data.slience_filter_enable;
}

U8 bt_av_get_receive_a2dp_start(void)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    return inst->snk_data.reveive_start;
}

#ifndef CFG_AVRCP
    #define AVRCP_PLAY_STATUS_STOP 0x00
    #define AVRCP_PLAY_STATUS_PLAYING 0x01
    #define AVRCP_PLAY_STATUS_PAUSED 0x02
#endif
U8 bt_av_get_a2dp_stream_state(void)
{
    bts2s_av_inst_data *inst = bt_av_get_inst_data();

    if (inst->con[0].cfg == AV_AUDIO_SRC)
    {
        if (inst->con[0].st == avconned_streaming)
        {
            return AVRCP_PLAY_STATUS_PLAYING;
        }
        else if (inst->con[0].st == avconned_open)
        {
            return AVRCP_PLAY_STATUS_PAUSED;
        }
    }

    return AVRCP_PLAY_STATUS_STOP;
}

void bt_av_hdl_set_bqb_test(U8 value)
{
    LOG_I("BQB_TEST_CASE = %d\n", value);
    BQB_TEST_CASE = value;
    return;
}

void bt_av_hdl_reset_bqb_test(void)
{
    BQB_TEST_CASE = BQB_TEST_RESET;
    return;
}


#endif // defined(CFG_AV_SNK) || defined(CFG_AV_SRC)




/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
