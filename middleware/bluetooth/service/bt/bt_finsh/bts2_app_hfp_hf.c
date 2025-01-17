/**
******************************************************************************
* @file   bts2_app_hfp_hf.c
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

#include "bts2_app_inc.h"
#include "rtthread.h"

#ifdef RT_USING_BT
    #include "bt_rt_device.h"
#endif

#ifdef CFG_HFP_HF

#define LOG_TAG         "btapp_hf"
#include "log.h"


#if defined(AUDIO_USING_MANAGER) && !defined(BT_USING_HF)
    #include "hfp_audio_api.h"
#endif

#define HFP_HF_LOCAL_FEATURES        (  HFP_HF_FEAT_ECNR  | \
                                        HFP_HF_FEAT_3WAY  | \
                                        HFP_HF_FEAT_CLI   | \
                                        HFP_HF_FEAT_VREC  | \
                                        HFP_HF_FEAT_VOL   | \
                                        HFP_HF_FEAT_ECS   | \
                                        HFP_HF_FEAT_ECC   | \
                                        HFP_HF_FEAT_CODEC | \
                                        HFP_HF_FEAT_ESCO  )

// sco handle: Packet_Status_Flag inside if any.
// static void hfp_hf_audio_cb_fn(U16 sco_hdl, U8 sco_len, U8 *data)
// {
// }

static bts2_hfp_hf_inst_data *bt_hfp_hf_get_context()
{
    bts2_app_stru *bts2_app_data = getApp();
    return bts2_app_data->hfp_hf_ptr;
}

static U8 bt_hfp_is_support_feature(U16 feature)
{
    bts2_hfp_hf_inst_data *hfp_context = bt_hfp_hf_get_context();
    if (hfp_context->peer_features & feature)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static void bt_hfp_hf_app_service_state_update(bts2_hfp_st new_state)
{
    bts2_hfp_hf_inst_data *hfp_context = bt_hfp_hf_get_context();

    USER_TRACE("hfp hf profile service new_date:%d, old_state:%d", new_state, hfp_context->st);

    hfp_context->st = new_state;
}
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_hfp_hf_init(bts2_app_stru *bts2_app_data)
{
    bts2_app_data->hfp_hf_ptr = &bts2_app_data->hfp_hf_inst;
    bts2_app_data->hfp_hf_inst.voice_flag = 0;
    bts2_app_data->esco_flag = FALSE;
    bts2_app_data->hfp_hf_inst.conn_type = HF_CONN;
    bts2_app_data->hfp_hf_inst.peer_features = 0x0000;
    bts2_app_data->hfp_hf_inst.sco_hdl = 0xffff;
    memset(&bts2_app_data->hfp_hf_inst.cind_status, 0x00, sizeof(bts2_hfp_hf_cind));
    bts2_app_data->hfp_hf_inst.srv_chnl = 0xff;//initial value
    bt_hfp_hf_app_service_state_update(hfp_idle);

#if defined(AUDIO_USING_MANAGER) && !defined(BT_USING_HF)
    hfp_audio_init();
#endif

}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_start_enb(bts2_app_stru *bts2_app_data)
{
    bts2_hfp_hf_inst_data *hfp_context = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (hfp_context->st)
    {
    case hfp_idle:
    case hfp_disb:
    {
        bt_hfp_hf_app_service_state_update(hfp_enbd);
        hfp_hf_enb_req(HFP_HF_LOCAL_FEATURES);
        ret = BT_EOK;
        break;
    }
    default:
    {
        if (hfp_context->st != hfp_enbd)
        {
            USER_TRACE(">> Hf enable fail\n");
        }
        break;
    }
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_start_disb(bts2_app_stru *bts2_app_data)
{
    bts2_hfp_hf_inst_data *hfp_context = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;
    switch (hfp_context->st)
    {
    case hfp_enbd:
    {
        hfp_hf_disb_req();
        hfp_context->voice_flag = 0;
        ret = BT_EOK;
        bt_hfp_hf_app_service_state_update(hfp_disb);
        USER_TRACE(">> Hf disable\n");
        break;
    }
    default:
    {
        USER_TRACE(">> Hf disable failed\n");
        break;
    }
    }
    return ret;
}

void bt_hfp_hf_clean_flag()
{
    bts2_hfp_hf_inst_data *hfp_context = bt_hfp_hf_get_context();
    hfp_context->voice_flag = 0;
    hfp_context->sco_hdl = 0xffff;
    hfp_context->peer_features = 0x0000;
    bt_hfp_hf_app_service_state_update(hfp_enbd);
}

U8 bt_hfp_hf_get_ring_type(void)
{
    return bt_hfp_is_support_feature(HFP_AG_FEAT_INBAND);
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_start_connecting(BTS2S_BD_ADDR *bd)
{
    bts2_hfp_hf_inst_data *hfp_context = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    if (bd_is_empty(bd))
    {
        return BT_ERROR_INPARAM;
    }

    switch (hfp_context->st)
    {
    case hfp_enbd:
    {
        hfp_hf_conn_req(bd, HF_CONN);
        ret = BT_EOK;
        break;
    }
    default:
    {
        break;
    }
    }
    USER_TRACE("bt_hfp_hf_start_connecting 0x%2x\n", ret);
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_start_disc(BTS2S_BD_ADDR *bd)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    case hfp_calling:
    {
        hfp_hf_disc_req();
        ptr->voice_flag = 0;
        ret = BT_EOK;
        bt_hfp_hf_app_service_state_update(hfp_enbd);
        USER_TRACE(">> Hf disconnect send\n");
        break;
    }
    default:
    {
        USER_TRACE(">> Hf disconnect fail");
        break;
    }
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_audio_transfer(U8 type)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    case hfp_calling:
    {
        // type 0:connect audio type 1 :disconnect audio
        if (type == 0)
        {
            hfp_hf_audio_transfer_req(1);//connect audio
            ret = BT_EOK;
        }
        else if (type == 1)
        {
            hfp_hf_audio_transfer_req(0);//disconect audio
            ret = BT_EOK;
        }
        else
        {
            ret = BT_ERROR_INPARAM;
        }
        break;
    }
    default:
    {
        USER_TRACE(">> Transfer audio path fail\n");
        break;
    }
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_voice_recog_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_voice_recog_send(U8 active)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    {
        if (active == 0 || active == 1)
        {
            hfp_hf_send_at_bvra_api(active);
            ret = BT_EOK;
        }
        else
        {
            ret = BT_ERROR_INPARAM;
        }
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_dial_by_mem_send(U16 memory)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    case hfp_calling:
    {
        char data[6];
        int at_len = 0;
        at_len = snprintf(data, sizeof(data), ">%u;", memory);
        hfp_hf_send_at_atd_api((U8 *)data, (U8) at_len);
        ret = BT_EOK;
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_last_num_dial_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_last_num_dial_send(void)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    case hfp_calling:
    {
        hfp_hf_send_at_bldn_api();
        ret = BT_EOK;
        //ptr->st = hfp_calling;
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_make_call_by_number_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_make_call_by_number_send(U8 *payload, U8 payload_len)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    //USER_TRACE(" out_going_call_req  ptr->st%d", ptr->st);
    switch (ptr->st)
    {
    case hfp_conned:
    case hfp_calling:
    {
        char *data;
        int p_payload_len = payload_len + 1;
        data = (char *)bmalloc(p_payload_len);
        if (data)
        {
            bmemcpy(data, payload, payload_len);
            data[payload_len] = ';';
            hfp_hf_send_at_atd_api((U8 *)data, (U8) p_payload_len);
            USER_TRACE("data %s len %d input_len %d", data, p_payload_len, payload_len);
            ret = BT_EOK;
            bfree(data);
        }
        else
            ret = BT_ERROR_OUT_OF_MEMORY;
        break;
    }
    default:
    {
        USER_TRACE(">> dial out fail\n");
        break;
    }
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Start_hf_answer_req_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_answer_call_send(void)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    {
        hfp_hf_send_at_ata_api();
        USER_TRACE(">> Answer the incoming call\n");
        ret = BT_EOK;
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_hangup_call_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_hangup_call_send(void)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    case hfp_calling:
    {
        hfp_hf_send_at_chup_api();
        ptr->voice_flag = 0;
        USER_TRACE(">> hfp_hf reject terminal the call\n");
        ret = BT_EOK;
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_update_spk_vol
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_update_spk_vol(U8 vol)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    case hfp_calling:
    {
        if (0 <= vol && vol <= 15)
        {
            hfp_hf_send_at_vgs_api((U8)vol); //just send 0---15
            ret = BT_EOK;
        }
        else
        {
            ret = BT_ERROR_INPARAM;
        }
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_update_mic_vol
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_update_mic_vol(U8 vol)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    case hfp_calling:
    {
        if (0 <= vol && vol <= 15)
        {
            hfp_hf_send_at_vgm_api(vol); //just send 0---15
            ret = BT_EOK;
        }
        else
        {
            ret = BT_ERROR_INPARAM;
        }
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_at_btrh_query_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_at_btrh_query_send(void)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    {
        hfp_hf_send_at_btrh_api();
        ret = BT_EOK;
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_at_btrh_cmd_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_at_btrh_cmd_send(U8 mode)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    USER_TRACE(">> enter \n");

    switch (ptr->st)
    {
    case hfp_conned:
    case hfp_calling:
    {
        hfp_hf_send_at_btrh_mode_api(mode);
        ret = BT_EOK;
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_at_binp_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_at_binp_send(void)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    {
        //Attach a Phone Number to a Voice Tag.
        hfp_hf_send_at_binp_api();
        ret = BT_EOK;
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_at_clip_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_at_clip_send(U8 enable)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    {
        //Enable calling Line Identification (CLI) Notification.
        if (enable == 0 || enable == 1)
        {
            hfp_hf_send_at_clip_api(enable);
            ret = BT_EOK;
        }
        else
        {
            ret = BT_ERROR_INPARAM;
        }
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_at_cmee_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_at_cmee_send(BOOL val)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    {
        if (val == 0 || val == 1)
        {
            hfp_hf_send_at_cmee_api(val);
            ret = BT_EOK;
        }
        else
        {
            ret = BT_ERROR_INPARAM;
        }
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Start_at_cnum_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_at_cnum_send(void)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    {
        hfp_hf_send_at_cnum_api();
        ret = BT_EOK;
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_at_ccwa_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_at_ccwa_send(BOOL val)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    {
        if (val == 0 || val == 1)
        {
            hfp_hf_send_at_ccwa_api(val); //active
            ret = BT_EOK;
        }
        else
        {
            ret = BT_ERROR_INPARAM;
        }
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_at_chld_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_at_chld_send(U8 *payload, U8 payload_len)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    case hfp_calling:
    {
        hfp_hf_send_at_chld_control_api(payload, payload_len);
        ret = BT_EOK;
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_at_clcc_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_at_clcc_send(void)
{
    bt_err_t ret = BT_ERROR_STATE;
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();

    switch (ptr->st)
    {
    case hfp_conned:
    case hfp_calling:
    {
        hfp_hf_send_at_clcc_api();
        // ok
        //during a call process, solution can send clcc the get info. so ptr->st maybe hfp_conned or hfp_calling
        //ptr->st = hfp_conned;
        USER_TRACE(">> List current call status\n");
        ret = BT_EOK;
        break;
    }
    default:
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_at_cops_cmd_send(void)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    if (ptr->st == hfp_conned)
    {
        //hfp_hf_copp_srv_req(COPSMODE, COPSFMTE);
        char *payload =  "3,0";
        U8 payload_len = strlen(payload);
        hfp_hf_send_at_cops_cmd_api((U8 *)payload, payload_len);
        ret = BT_EOK;
        USER_TRACE(">> set the cops information\n");
    }
    else
    {
        USER_TRACE(">> Not in connected state\n");
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_at_dtmf_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_at_dtmf_send(char key)
{
    //U8 *data;
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    case hfp_calling:
    {
        hfp_hf_send_at_vts_api(key);
        ret = BT_EOK;
        break;
    }
    default:
        USER_TRACE("-- Hf state error, current state is %d\n", ptr->st);
        break;
    }
    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Start_ccwa_req_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_at_nrec_send(void)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    {
        //The HF may disable the echo canceling and noise reduction functions resident in the AG via the AT+NREC command.
        hfp_hf_send_at_nrec_api();
        ret = BT_EOK;
        break;
    }
    default:
        break;
    }

    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_update_batt_send
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bt_err_t bt_hfp_hf_update_batt_send(U8 batt_val)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    bt_err_t ret = BT_ERROR_STATE;

    switch (ptr->st)
    {
    case hfp_conned:
    case hfp_calling:
    {
        if (0 <= batt_val && batt_val <= 9)
        {
            char data[8];
            int at_len = 0;
            at_len = snprintf(data, sizeof(data), "1,1,%d", batt_val);
            hfp_hf_send_at_batt_update_api((U8 *)data, (U8) at_len); //just 0~9
            ret = BT_EOK;
        }
        else
        {
            ret = BT_ERROR_INPARAM;
        }
        break;
    }
    default:
        break;
    }

    return ret;
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
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
void bt_hfp_hf_rfc_conn_accept_hdl(void)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    hfp_hf_conn_rsp(&ptr->hfp_bd, ptr->srv_chnl, TRUE);
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
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
void bt_hfp_hf_rfc_conn_rej_hdl(void)
{
    bts2_hfp_hf_inst_data *ptr = bt_hfp_hf_get_context();
    hfp_hf_conn_rsp(&ptr->hfp_bd, ptr->srv_chnl, FALSE);
}

static void bt_hfp_hf_at_cmd_cfm_hdl(BTS2S_HF_AT_CMD_CFM *msg)
{

    USER_TRACE("bt_hfp_at_cmd_cfm cmd_id:0x%2x  res:0x%2x", msg->at_cmd_id, msg->res);

    switch (msg->at_cmd_id)
    {
    case HFP_HF_AT_BVRA:
    case HFP_HF_AT_CLCC:
    case HFP_HF_AT_ATD:
    case HFP_HF_AT_BLDN:
    case HFP_HF_AT_VTS:
    case HFP_HF_AT_VGS:
    {
        break;
    }
    case HFP_HF_AT_BCC:
    case HFP_HF_AT_CIND_STATUS:
    case HFP_HF_AT_CMER:
    case HFP_HF_AT_CHLD_CMD:
    case HFP_HF_AT_CMEE:
    case HFP_HF_AT_BIA:
    case HFP_HF_AT_CLIP:
    case HFP_HF_AT_CCWA:
    case HFP_HF_AT_COPS_CMD:
    case HFP_HF_AT_VGM:
    case HFP_HF_AT_ATA:
    case HFP_HF_AT_CHUP:
    case HFP_HF_AT_BTRH:
    case HFP_HF_AT_BTRH_MODE:
    case HFP_HF_AT_CNUM:
    case HFP_HF_AT_NREC:
    case HFP_HF_AT_BINP:
    {
        break;
    }

    default:
        break;
    }

    bt_notify_at_cmd_cfm_t at_cmd_cfm;
    at_cmd_cfm.at_cmd_id = msg->at_cmd_id;
    at_cmd_cfm.res = msg->res;
    bt_interface_bt_event_notify(BT_NOTIFY_HFP_HF, BT_NOTIFY_HF_AT_CMD_CFM, &at_cmd_cfm, sizeof(bt_notify_at_cmd_cfm_t));
}

U8 bt_hfp_hf_get_ciev_info(BTS2S_HF_CIEV_IND *msg)
{
    int ptr;
    char *str1 = "call";
    char *str2 = "callheld";
    char *str3 = "callsetup";
    bt_notify_cind_ind_t ind = {0};
    bts2_hfp_hf_inst_data *inst_data = bt_hfp_hf_get_context();

    ptr = strcmp(msg->name, str1);//call
    if (0 == ptr)
    {
        ind.type = HFP_AG_CIND_CALL_TYPE;
        ind.val = msg->val;
        USER_TRACE(">>ciev call st %d callStatus %d\n", inst_data->st, msg->val);
    }

    ptr = strcmp(msg->name, str2);//callheld
    if (0 == ptr)
    {
        ind.type = HFP_AG_CIND_CALLHELD_TYPE;
        ind.val = msg->val;
        USER_TRACE(">>ciev call hold st %d callStatus%d\n", inst_data->st, msg->val);
    }

    ptr = strcmp(msg->name, str3);//callsetup
    if (0 == ptr)
    {
        ind.type = HFP_AG_CIND_CALLSETUP_TYPE;
        ind.val = msg->val;
        USER_TRACE(">>ciev call setup callStatus %d\n", msg->val);
    }

    bt_interface_bt_event_notify(BT_NOTIFY_HFP_HF, BT_NOTIFY_HF_INDICATOR_UPDATE,
                                 &ind, sizeof(bt_notify_cind_ind_t));
    return msg->val;
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      bt_hfp_hf_msg_hdl
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_hfp_hf_msg_hdl(bts2_app_stru *bts2_app_data)
{
    U16 *msg_type;
    bts2_hfp_hf_inst_data *inst_data;
    inst_data = bts2_app_data->hfp_hf_ptr;

    msg_type = (U16 *)bts2_app_data->recv_msg;
    USER_TRACE("hfp msg *msg_type %x inst_data->st %d\n", *msg_type, inst_data->st);

    switch (*msg_type)
    {
    case BTS2MU_HF_DISB_CFM:
    {
        BTS2S_HF_DISB_CFM *msg;
        msg = (BTS2S_HF_DISB_CFM *)bts2_app_data->recv_msg;
        inst_data->voice_flag = 0;
        break;
    }
    case BTS2MU_HF_ENB_CFM:
    {
        BTS2S_HF_ENB_CFM *msg;
        msg = (BTS2S_HF_ENB_CFM *)bts2_app_data->recv_msg;
        inst_data->voice_flag = 0;
        if (msg->conn_type == HF_CONN && msg->res == BTS2_SUCC)
        {
            USER_TRACE(">> Handfree enable success\n");
        }

        if (msg->conn_type == HS_CONN && msg->res == BTS2_SUCC)
        {
            USER_TRACE(">> Headset enable success\n");
        }
        break;
    }
    case BTS2MU_HF_CONN_IND:
    {
        BTS2S_HF_CONN_IND *msg;
        msg = (BTS2S_HF_CONN_IND *)bts2_app_data->recv_msg;

        bts2_app_data->hfp_hf_inst.srv_chnl = msg->srv_chnl;
        inst_data->hfp_bd = msg->bd;
        bts2_app_data->menu_id = menu_hfp_hf;
        bt_hfp_hf_rfc_conn_accept_hdl();

        //20220727:add last_conn_bd
        bts2_app_data->last_conn_bd = msg->bd;
#ifdef BTS2_APP_MENU
        bt_disply_menu(bts2_app_data);
#endif
        break;
    }
    case BTS2MU_HF_CONN_CFM:
    {
        BTS2S_HF_CONN_CFM *msg;
        msg = (BTS2S_HF_CONN_CFM *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            inst_data->conn_type = msg->conn_type;
            bt_hfp_hf_app_service_state_update(hfp_conned);
            bts2_app_data->menu_id = menu_hfp_hf;
            if (bt_hfp_is_support_feature(HFP_AG_FEAT_ECNR))
            {
                hfp_hf_send_at_nrec_api();
                inst_data->peer_features &= (~HFP_AG_FEAT_ECNR);
            }

            bt_notify_all_call_status call_info;
            call_info.call_status = inst_data->cind_status.callStatus;
            call_info.callsetup_status = inst_data->cind_status.callHeldStatus;
            call_info.callheld_status = inst_data->cind_status.callHeldStatus;
            bt_interface_bt_event_notify(BT_NOTIFY_HFP_HF, BT_NOTIFY_HF_CALL_STATUS_UPDATE,
                                         &call_info, sizeof(bt_notify_all_call_status));

            bt_notify_profile_state_info_t profile_state;
            bt_addr_convert(&msg->bd, profile_state.mac.addr);
            profile_state.profile_type = BT_NOTIFY_HFP_HF;
            profile_state.res = BTS2_SUCC;
            bt_interface_bt_event_notify(BT_NOTIFY_HFP_HF, BT_NOTIFY_HF_PROFILE_CONNECTED,
                                         &profile_state, sizeof(bt_notify_profile_state_info_t));

            memset(&(inst_data->cind_status), 0x00, sizeof(bts2_hfp_hf_cind));
            bts2_app_data->bd_list[bts2_app_data->dev_idx] = msg->bd;
#ifdef BTS2_APP_MENU
            bt_disply_menu(bts2_app_data);
#endif

            if (msg->conn_type == HF_CONN)
            {
                USER_TRACE("<< HF Conneted success\n");
            }
            else
            {
                USER_TRACE("<< HS Conneted success\n");
            }

            bts2_app_data->last_conn_bd = msg->bd;

            USER_TRACE("conn_cfm bd: %04X:%04X:%04X\n",
                       msg->bd.lap, msg->bd.uap, msg->bd.nap);
            //bts2_app_stru bt_app_data = {0};
            //bt_app_data.menu_id = menu_gen_7;
            //bt_app_data.input_str[0] = '0';
            //bt_hdl_menu(&bt_app_data);
        }
        else
        {
            bt_hfp_hf_clean_flag();

            bt_notify_profile_state_info_t profile_state;
            bt_addr_convert(&msg->bd, profile_state.mac.addr);
            profile_state.profile_type = BT_NOTIFY_HFP_HF;
            profile_state.res = msg->res;
            bt_interface_bt_event_notify(BT_NOTIFY_HFP_HF, BT_NOTIFY_HF_PROFILE_DISCONNECTED,
                                         &profile_state, sizeof(bt_notify_profile_state_info_t));
        }

        break;
    }

    case BTS2MU_HF_DISC_CFM:
    {
        BTS2S_HF_DISC_CFM *msg;
        msg = (BTS2S_HF_DISC_CFM *)bts2_app_data->recv_msg;
        bt_hfp_hf_clean_flag();

        bt_notify_profile_state_info_t profile_state;
        bt_addr_convert(&msg->cur_bd, profile_state.mac.addr);
        profile_state.profile_type = BT_NOTIFY_HFP_HF;
        profile_state.res = msg->res;
        bt_interface_bt_event_notify(BT_NOTIFY_HFP_HF, BT_NOTIFY_HF_PROFILE_DISCONNECTED,
                                     &profile_state, sizeof(bt_notify_profile_state_info_t));

#if defined(AUDIO_USING_MANAGER) && !defined(BT_USING_HF)
        hfp_audio_close_path();
#endif // AUDIO_USING_MANAGER

        //USER_TRACE("<< cfm Disconnet sucess\n");
        //bts2_app_data->last_conn_bd.lap = CFG_BD_LAP;
        //bts2_app_data->last_conn_bd.uap = CFG_BD_UAP;
        //bts2_app_data->last_conn_bd.nap = CFG_BD_NAP;
        break;
    }
    case BTS2MU_HF_DISC_IND:
    {
        BTS2S_HF_DISC_IND *msg;
        msg = (BTS2S_HF_DISC_IND *)bts2_app_data->recv_msg;
        bt_hfp_hf_clean_flag();

        bt_notify_profile_state_info_t profile_state;
        bt_addr_convert(&msg->cur_bd, profile_state.mac.addr);
        profile_state.profile_type = BT_NOTIFY_HFP_HF;
        profile_state.res = msg->res;
        bt_interface_bt_event_notify(BT_NOTIFY_HFP_HF, BT_NOTIFY_HF_PROFILE_DISCONNECTED,
                                     &profile_state, sizeof(bt_notify_profile_state_info_t));

#if defined(AUDIO_USING_MANAGER) && !defined(BT_USING_HF)
        hfp_audio_close_path();
#endif // AUDIO_USING_MANAGER

        USER_TRACE("<< Disconnet sucess %x\n", msg->res);
        //bts2_app_data->last_conn_bd.lap = CFG_BD_LAP;
        //bts2_app_data->last_conn_bd.uap = CFG_BD_UAP;
        //bts2_app_data->last_conn_bd.nap = CFG_BD_NAP;
        break;
    }

    case BTS2MU_HF_AT_DATA_IND:
    {
        BTS2S_HF_AT_DATA_IND *msg;
        int i;
        msg = (BTS2S_HF_AT_DATA_IND *)bts2_app_data->recv_msg;

        USER_TRACE("<< Receive Externed AT Command from AG:");
        for (i = 0; i < msg->payload_len; ++i)
        {
            USER_TRACE("%c", msg->payload[i]);
        }

        if (NULL != (msg->payload))
        {
            bfree(msg->payload);
        }
        USER_TRACE("\n");
        break;
    }

    case BTS2MU_HF_SPK_GAIN_IND:
    {
        BTS2S_HF_SPK_GAIN_IND *msg;
        msg = (BTS2S_HF_SPK_GAIN_IND *)bts2_app_data->recv_msg;
        USER_TRACE("<< AG change speaker volume to be %d\n", msg->gain);
        bt_interface_bt_event_notify(BT_NOTIFY_HFP_HF, BT_NOTIFY_HF_VOLUME_CHANGE,
                                     &msg->gain, 1);

#if defined(AUDIO_USING_MANAGER) && !defined(BT_USING_HF)
        audio_server_set_private_volume(AUDIO_TYPE_BT_VOICE, msg->gain);
#endif
        break;
    }

    case BTS2MU_HF_MIC_GAIN_IND:
    {
        BTS2S_HF_MIC_GAIN_IND *msg;
        msg = (BTS2S_HF_MIC_GAIN_IND *)bts2_app_data->recv_msg;
        USER_TRACE("<< AG change microphone volume to be %d\n", msg->gain);
        break;
    }

    case BTS2MU_HF_VOICE_RECOG_IND:
    {
        BTS2S_HF_VOICE_RECOG_IND *msg;
        msg = (BTS2S_HF_VOICE_RECOG_IND *)bts2_app_data->recv_msg;

        if (msg->val == TRUE)
        {
            USER_TRACE("<< Activate Voice Recognition\n");
        }
        else
        {
            USER_TRACE("<< Deactivate Voice Recognition\n");
        }
        bt_interface_bt_event_notify(BT_NOTIFY_HFP_HF, BT_NOTIFY_HF_VOICE_RECOG_STATUS_CHANGE, &msg->val, sizeof(uint8_t));
        break;
    }
    case BTS2MU_HF_AUDIO_CFM:
    {
        BTS2S_HF_AUDIO_CFM *msg;
        msg = (BTS2S_HF_AUDIO_CFM *)bts2_app_data->recv_msg;

        if (msg->audio_on == TRUE)
        {
            USER_TRACE(">> Transfer audio to Hands-free\n");
        }
        else
        {
            USER_TRACE(">> Transfer audio to AG\n");
        }
        break;
    }
    case BTS2MU_HF_AUDIO_IND:
    {
        BTS2S_HF_AUDIO_INFO *msg;
        msg = (BTS2S_HF_AUDIO_INFO *)bts2_app_data->recv_msg;
        inst_data->voice_flag = msg->audio_on;

        if (msg->link_type >= 2)
        {
            bts2_app_data->esco_flag = TRUE;
        }
        else
        {
            bts2_app_data->esco_flag = FALSE;
        }
        if (msg->audio_on == TRUE)
        {
            USER_TRACE("<< Audio connected\n");
            // sco handle: Packet_Status_Flag inside if any.
            inst_data->sco_hdl = msg->sco_hdl;
            // gap_reg_sco_callback(msg->sco_hdl, hfp_hf_audio_cb_fn);

            bt_notify_device_sco_info_t sco_info;
            sco_info.sco_type = BT_NOTIFY_HFP_HF;
            sco_info.sco_res = BTS2_SUCC;
            bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_SCO_CONNECTED,
                                         &sco_info, sizeof(bt_notify_device_sco_info_t));

#if defined(AUDIO_USING_MANAGER) && !defined(BT_USING_HF)
            hfp_set_audio_voice_para(msg, msg->audio_on, 1);
#endif // AUDIO_USING_MANAGER

        }
        else
        {
            //inst_data->st = hfp_conned;
            USER_TRACE("<< Audio disconnect\n");
            inst_data->sco_hdl = 0xffff;

#if defined(AUDIO_USING_MANAGER) && !defined(BT_USING_HF)
            hfp_audio_close_path();
#endif // AUDIO_USING_MANAGER

            bt_notify_device_sco_info_t sco_info;
            sco_info.sco_type = BT_NOTIFY_HFP_HF;
            sco_info.sco_res = BTS2_SUCC;
            bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_SCO_DISCONNECTED,
                                         &sco_info, sizeof(bt_notify_device_sco_info_t));
            gap_unreg_sco_callback(msg->sco_hdl);
        }
        break;
    }
    case BTS2MU_HF_AT_CMD_CFM:
    {
        BTS2S_HF_AT_CMD_CFM *msg;
        msg = (BTS2S_HF_AT_CMD_CFM *) bts2_app_data->recv_msg;
        bt_hfp_hf_at_cmd_cfm_hdl(msg);
        break;
    }

    case BTS2MU_HF_RING_IND:
    {
        BTS2S_HF_RING_IND *msg;
        msg = (BTS2S_HF_RING_IND *) bts2_app_data->recv_msg;
        USER_TRACE("<< Ring......\n");
        break;
    }
    case BTS2MU_HF_STS_IND:
    {
        BTS2S_HF_ST_IND *msg;
        msg = (BTS2S_HF_ST_IND *) bts2_app_data->recv_msg;

        switch (msg->st_ev)
        {
        case 0:
        {
            USER_TRACE(">> Active mode\n");
            break;
        }
        case 1:
        {
            USER_TRACE(">> Hold mode\n");
            break;
        }
        case 2:
        {
            USER_TRACE(">> Sniff mode\n");
            break;
        }
        case 3:
        {
            USER_TRACE(">> Park mode\n");
            break;
        }
        default:
            break;
        }
        break;
    }
    case BTS2MU_HF_CIEV_IND:
    {
        BTS2S_HF_CIEV_IND *msg;
        U8 callStatus = 0xff;
        msg = (BTS2S_HF_CIEV_IND *) bts2_app_data->recv_msg;
        USER_TRACE(">> \"%s\":%d\n", msg->name, msg->val);
        callStatus = bt_hfp_hf_get_ciev_info(msg);
        break;
    }
    case BTS2MU_HF_CHLD_IND:
    {
        BTS2S_HF_CHLD_IND *msg;
        msg = (BTS2S_HF_CHLD_IND *) bts2_app_data->recv_msg;
        USER_TRACE(">> \"%s\":%d\n", msg->chld_str, msg->supp);
        break;
    }

    case BTS2MU_HF_CLCC_IND:
    {
        BTS2S_HF_CLCC_IND *msg;
        msg = (BTS2S_HF_CLCC_IND *)bts2_app_data->recv_msg;
        /*The numbering (starting with 1) of the call given by the
        sequence of setting up or receiving the calls (active, held
        or waiting) as seen by the served subscriber.*/
        USER_TRACE("<< The index is:%d\n", msg->idx);
        if (msg->dir == 1)
        {
            USER_TRACE("<< It's an incoming call\n");
        }
        else
        {
            USER_TRACE("<< It's an outgoing call\n");
        }
        switch (msg->st)
        {
        case 0:
        {
            USER_TRACE("<< Active\n");
            break;
        }
        case 1:
        {
            USER_TRACE("<< Held\n");
            break;
        }
        case 2:
        {
            USER_TRACE("<< Dialing(outgoing calls only)\n");
            break;
        }
        case 3:
        {
            USER_TRACE("<< Alerting (outgoing calls only)\n");
            break;
        }
        case 4:
        {
            USER_TRACE("<< Incoming (incoming calls only)\n");
            break;
        }
        case 5:
        {
            USER_TRACE("<< Waiting (incoming calls only)\n");
            break;
        }
        default:
        {
            break;
        }
        }
        switch (msg->mode)
        {
        case 0:
        {
            USER_TRACE("<< Voice\n");
            break;
        }
        case 1:
        {
            USER_TRACE("<< Data\n");
            break;
        }
        case 2:
        {
            USER_TRACE("<< Fax\n");
            break;
        }
        default:
        {
            break;
        }
        }
        if (msg->mpty)
        {
            USER_TRACE("<< Multiparty\n");
        }
        else
        {
            USER_TRACE("<< Not Multiparty\n");
        }

        if (0 != msg->data_len)
        {

            bt_notify_clcc_ind_t ind = {0};
            ind.dir = msg->dir;
            ind.mode = msg->mode;
            ind.mpty = msg->mpty;
            ind.st = msg->st;
            ind.phone_number_type = msg->type;
            ind.number_size = msg->data_len;
            ind.idx = msg->idx;
            bmemcpy(&ind.number, msg->data, ind.number_size);
            bt_interface_bt_event_notify(BT_NOTIFY_HFP_HF, BT_NOTIFY_HF_REMOTE_CALL_INFO_IND,
                                         &ind, sizeof(bt_notify_clcc_ind_t));
        }

        if (NULL != msg->data)
        {
            bfree(msg->data);
        }

        if (NULL != msg->body)
        {
            bfree(msg->body);
        }
        break;
    }

    case BTS2MU_HF_CNUM_IND:
    {
        BTS2S_HF_CNUM_IND *msg;
        msg = (BTS2S_HF_CNUM_IND *)bts2_app_data->recv_msg;
        USER_TRACE(">> Phone number \"%s\",phone type <%d>\n", msg->phone_number, msg->phone_type);

        bt_interface_bt_event_notify(BT_NOTIFY_HFP_HF, BT_NOTIFY_HF_LOCAL_PHONE_NUMBER,
                                     msg->phone_number, msg->phone_len);
        break;
    }
    case BTS2MU_HF_CIND_IND:
    {
        BTS2S_HF_CALL_STATUS_IND  *msg;
        msg = (BTS2S_HF_CALL_STATUS_IND *)bts2_app_data->recv_msg;
        USER_TRACE("cind call status %d callHeldStatus %d callSetupStatus %d",
                   msg->cind_status.callStatus, msg->cind_status.callHeldStatus, msg->cind_status.callSetupStatus);
        if (inst_data->st < hfp_conned)
        {
            inst_data->cind_status.callStatus = msg->cind_status.callStatus;
            inst_data->cind_status.callHeldStatus = msg->cind_status.callHeldStatus;
            inst_data->cind_status.callSetupStatus = msg->cind_status.callSetupStatus;
            break;
        }

        bt_notify_all_call_status call_info;
        call_info.call_status = msg->cind_status.callStatus;
        call_info.callsetup_status = msg->cind_status.callHeldStatus;
        call_info.callheld_status = msg->cind_status.callHeldStatus;
        bt_interface_bt_event_notify(BT_NOTIFY_HFP_HF, BT_NOTIFY_HF_CALL_STATUS_UPDATE,
                                     &call_info, sizeof(bt_notify_all_call_status));
        break;
    }

    case BTS2MU_HF_BTRH_IND:
    {
        BTS2S_HF_BTRH_IND *msg;
        msg = (BTS2S_HF_BTRH_IND *)bts2_app_data->recv_msg;
        USER_TRACE("URC call 3way btrh ind\n");
        break;
    }

    case BTS2MU_HF_CLIP_IND:
    {
        BTS2S_HF_CLIP_IND *msg;
        int i = 0;
        msg = (BTS2S_HF_CLIP_IND *)bts2_app_data->recv_msg;
        USER_TRACE("CLIP  call phone number: %s,phone type %d", msg->phone_number, msg->phone_type);
        break;
    }

    case BTS2MU_HF_CCWA_IND:
    {
        BTS2S_HF_CCWA_IND *msg;
        int i = 0;
        msg = (BTS2S_HF_CCWA_IND *)bts2_app_data->recv_msg;
        USER_TRACE("CCWA Phone number: %s phone type: %d", msg->phone_number, msg->phone_type);

        break;
    }

    case BTS2MU_HF_BINP_IND:
    {
        BTS2S_HF_BINP_IND *msg;

        msg = (BTS2S_HF_BINP_IND *)bts2_app_data->recv_msg;
        if (msg->res == BTS2_SUCC)
        {
            int i = 0;
            USER_TRACE("-- Phone number: ");
            for (i = 0; i < msg->phone_len; i++)
            {
                USER_TRACE("%c", msg->phone_number[i]);
            }
            USER_TRACE("-- phone type <%d>\n", msg->phone_type);
        }
        else
        {
            USER_TRACE("-- ERROR\n");
        }
        break;
    }

    case BTS2MU_HF_CMEE_IND:
    {
        BTS2S_HF_CMEE_IND *msg;
        msg = (BTS2S_HF_CMEE_IND *)bts2_app_data->recv_msg;
        USER_TRACE(">> Error code is: %d\n", msg->cmee_err_code);
        break;
    }

    case BTS2MU_HF_COPP_SRV_QUERY_IND:
    {
        BTS2MD_HF_COPP_SRV_QUERY_IND *msg;
        int i = 0;

        msg = (BTS2MD_HF_COPP_SRV_QUERY_IND *)bts2_app_data->recv_msg;
        USER_TRACE("<< Network operator's name is: ");
        for (i = 0; i < msg->data_len; i++)
        {
            USER_TRACE("%c", msg->data[i]);
        }
        USER_TRACE("\n");
        break;
    }
    case BTS2MU_HF_BSIR_IND:
    {
        BTS2S_HF_COMMON_CFM *msg;
        msg = (BTS2S_HF_COMMON_CFM *)bts2_app_data->recv_msg;
        // USER_TRACE("BTS2MU_HF_BSIR_IND %d\n", msg->res);

        if (msg->res)
        {
            inst_data->peer_features |= HFP_AG_FEAT_INBAND;
        }
        else
        {
            inst_data->peer_features &= (~HFP_AG_FEAT_INBAND);
        }

        break;
    }

    case BTS2MU_HF_BRSF_IND:
    {
        U8 supp_voice_reg;
        BTS2S_HF_BRSF_IND *msg;
        msg = (BTS2S_HF_BRSF_IND *)bts2_app_data->recv_msg;
        bts2_app_data->hfp_hf_inst.peer_features = msg->supp_feature;
        supp_voice_reg = bt_hfp_is_support_feature(HFP_AG_FEAT_VREC);
        bt_interface_bt_event_notify(BT_NOTIFY_HFP_HF, BT_NOTIFY_HF_VOICE_RECOG_CAP_UPDATE, &supp_voice_reg, sizeof(uint8_t));
        break;
    }
    default:
    {
        USER_TRACE("<< Unexpected message %x\n", *msg_type);
        break;
    }
    }
}

#endif



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
