/**
  ******************************************************************************
  * @file   bts2_app_avrcp.c
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
#ifdef  CFG_MS
    #include <Windows.h>
#endif

#ifdef RT_USING_BT
    #include "bt_rt_device.h"
#endif

#include "drivers/bt_device.h"
#include "bt_connection_manager.h"

#ifdef CFG_AVRCP

#define LOG_TAG         "audio_avrcp"
//#define DBG_LVL          LOG_LVL_INFO
#include "log.h"

#ifdef CFG_MS
    HINSTANCE plyer_hdl = NULL;
    PAVRCP_ctrl_API avrcp_target = NULL;
#endif
static int tlabel = 0;
#undef ASSIGN_TLABEL
#define ASSIGN_TLABEL ((U8)(tlabel++ % 16))

#ifdef AUDIO_USING_MANAGER
    extern uint8_t a2dp_set_speaker_volume(uint8_t volume);
#endif
extern bts2_app_stru *bts2g_app_p;
uint8_t   bts2s_avrcp_openFlag;//0x00:dont open avrcp profile; 0x01:open avrcp profile;
#ifdef BSP_BQB_TEST
    extern U8 BQB_TEST_CASE;
#endif


#define BEGIN_ACCESS_VAR()              {rt_sem_take(bts2g_app_p->avrcp_inst.volume_change_sem, RT_WAITING_FOREVER);  \
                                         LOG_D("enter modifly tgRegStatus\n");}
#define END_ACCESS_VAR()                {rt_sem_release(bts2g_app_p->avrcp_inst.volume_change_sem); \
                                         LOG_D("exit modifly tgRegStatus\n");}


// #define SDK_AVRCP_USE_PASS_THROUGH      1

typedef struct
{
    uint32_t size;
    uint8_t song_name[BT_MAX_SONG_NAME_LEN];
} bt_avrcp_mp3_song_name_t;

typedef struct
{
    uint32_t size;
    uint8_t singer_name[BT_MAX_SINGER_NAME_LEN];
} bt_avrcp_mp3_singer_name_t;


typedef struct
{
    uint32_t size;
    uint8_t album_name[BT_MAX_ALBUM_INFO_LEN];
} bt_avrcp_mp3_album_info_t;

typedef struct
{
    uint32_t size;
    uint8_t play_time[BT_MAX_PLAY_TIME_LEN];//ascii code  ,unit:ms
} bt_avrcp_mp3_play_time_t;

typedef struct
{
    uint32_t  song_total_size;          /**< the song's total length */
    bt_avrcp_mp3_play_time_t duration;                  /**< the song's total duration */
    bt_avrcp_mp3_song_name_t song_name;          /**< the song's name */
    bt_avrcp_mp3_singer_name_t singer_name;      /**< the song's singer name */
    bt_avrcp_mp3_album_info_t album_info;        /**< the song's album name */
    uint16_t          character_set_id;  //UTF-8 0x006A; other??
} bt_avrcp_mp3_detail_info_t;

typedef struct
{
    uint8_t  track_id;
    uint8_t  attri_req;
    bt_avrcp_mp3_detail_info_t detail_info;
} bt_avrcp_mp3_detail_t;


bt_avrcp_mp3_detail_t mp3_detail_info;

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
void bt_avrcp_int(bts2_app_stru *bts2_app_data)
{
    bts2_app_data->avrcp_inst.st = avrcp_idle;
    bts2_app_data->avrcp_inst.release_type = 0x00;
    bts2_app_data->avrcp_inst.avrcp_time_handle = NULL;
    bts2_app_data->avrcp_inst.avrcp_vol_time_handle = NULL;
    bts2_app_data->avrcp_inst.volume_change_sem = rt_sem_create("bt_avrcp_vol_change", 1, RT_IPC_FLAG_FIFO);
    bts2_app_data->avrcp_inst.tgRegStatus = 0;
    bts2_app_data->avrcp_inst.tgRegStatus1 = 0;
    bts2_app_data->avrcp_inst.abs_volume_pending = 0;
    bts2_app_data->avrcp_inst.playback_status = 0;
    bts2_app_data->avrcp_inst.ab_volume = 20;//default value;
#ifdef CFG_OPEN_AVRCP
    bts2s_avrcp_openFlag = 1;
#else
    bts2s_avrcp_openFlag = 0;
#endif
    if (1 == bts2s_avrcp_openFlag)
    {
        avrcp_enb_req(bts2_app_data->phdl, AVRCP_CT);
    }

    memset(&mp3_detail_info, 0x00, sizeof(bt_avrcp_mp3_detail_t));
    mp3_detail_info.track_id = 0xff;
}

void bt_avrcp_open(void)
{
    bts2_app_stru *bts2_app_data = bts2g_app_p;

    USER_TRACE("bt_avrcp_open %d flag\n", bts2s_avrcp_openFlag);

    if (0 == bts2s_avrcp_openFlag)
    {
        bts2s_avrcp_openFlag = 0x01;
        avrcp_enb_req(bts2_app_data->phdl, AVRCP_CT);
    }
    else
    {
#if defined(CFG_AVRCP)
        bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_OPEN_COMPLETE, NULL, 0);
        INFO_TRACE(">> URC AVRCP open,alreay open\n");
#endif
    }
}

void bt_avrcp_close(void)
{
    USER_TRACE("bt_avrcp_close %d flag\n", bts2s_avrcp_openFlag);

    if (0x01 == bts2s_avrcp_openFlag)
    {
        bts2s_avrcp_openFlag = 0x00;
        avrcp_disb_req(); //disable avrcp
    }
    else
    {
#if defined(CFG_AVRCP)
        bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_CLOSE_COMPLETE, NULL, 0);

        INFO_TRACE(">> alreay close,urc AVRCP close\n");
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
void bt_avrcp_conn_2_dev(BTS2S_BD_ADDR *bd, BOOL is_target)
{
    bts2_app_stru *bts2_app_data = bts2g_app_p;

    if (bts2_app_data->avrcp_inst.st == avrcp_idle)
    {
        USER_TRACE(" -- avrcp conn remote device...\n");
        USER_TRACE(" -- address: %04X:%02X:%06lX\n",
                   bd->nap,
                   bd->uap,
                   bd->lap);

        if (!is_target)
        {
            avrcp_conn_req(bts2_app_data->phdl, *bd, AVRCP_TG, AVRCP_CT);
        }
        else
        {
            avrcp_conn_req(bts2_app_data->phdl, *bd, AVRCP_CT, AVRCP_TG);
        }
    }
    else
    {
        USER_TRACE(" -- already connected with remote device\n");
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
void bt_avrcp_disc_2_dev(BTS2S_BD_ADDR *bd_addr)
{
    bts2_app_stru *bts2_app_data = getApp();
    if (avrcp_conned == bts2_app_data->avrcp_inst.st)
    {
        USER_TRACE(">> disconnect avrcp with remote device...\n");
        avrcp_disc_req();
        bts2_app_data->avrcp_inst.st = avrcp_idle;
    }
    else
    {
        USER_TRACE(">> disconnect avrcp,already idle\n");
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
void bt_avrcp_pop(bts2_app_stru *bts2_app_data, U8 stateOpe)
{
    U8 data[3] = {0x7c, stateOpe, 0x00}; /* pop*/
    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_CTRL,
                       AVRCP_PASS_THROUGH_SUBUNIT_TYPE,
                       AVRCP_PASS_THROUGH_SUBUNIT_ID,
                       3,
                       data);
}

void bt_avrcp_delay_pop(bts2_app_stru *bts2_app_data, int type)
{
    bts2_app_data->avrcp_inst.release_type = type;

    if (!bts2_app_data->avrcp_inst.avrcp_time_handle)
    {
        bts2_app_data->avrcp_inst.avrcp_time_handle = rt_timer_create("avrcp_ti", bt_avrcp_timeout_handler, (void *)bts2_app_data,
                rt_tick_from_millisecond(100), RT_TIMER_FLAG_SOFT_TIMER);
    }
    else
    {
        rt_timer_stop(bts2_app_data->avrcp_inst.avrcp_time_handle);
    }
    rt_timer_start(bts2_app_data->avrcp_inst.avrcp_time_handle);
}

void bt_avrcp_timeout_handler(void *parameter)
{
    bts2_app_stru *bts2_app_data = (bts2_app_stru *)parameter;
    U8 data[3] = {0x7c, 0x00, 0x00};

    data[1] = 0x80 | bts2_app_data->avrcp_inst.release_type;

    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_CTRL,
                       AVRCP_PASS_THROUGH_SUBUNIT_TYPE,
                       AVRCP_PASS_THROUGH_SUBUNIT_ID,
                       3,
                       data);
}

void bt_avrcp_vol_timeout_handler(void *parameter)
{
    USER_TRACE("can't receive interim notify,re-try register volume changed\n");
    bts2_app_stru *bts2_app_data = (bts2_app_stru *)parameter;
    bt_avrcp_volume_register_request(bts2_app_data);
}


void bt_avrcp_ply(bts2_app_stru *bts2_app_data)
{
    U8 data[3] = {0x7c, 0x44, 0x00};/* ply*/

    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_CTRL,
                       AVRCP_PASS_THROUGH_SUBUNIT_TYPE,
                       AVRCP_PASS_THROUGH_SUBUNIT_ID,
                       3,
                       data);


    /*U8 *record;
      U16 num_rec_bytes;
      num_rec_bytes = sizeof(bts2s_sds_pan_panu_svc_record);
      record = (U8 *) bmalloc(num_rec_bytes);
      bmemcpy(record, bts2s_sds_pan_panu_svc_record, num_rec_bytes);
      gap_sds_reg_req(bts2_app_data->phdl, record, num_rec_bytes); */
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
void bt_avrcp_rewind(bts2_app_stru *bts2_app_data)
{
    U8 data[3] = {0x7c, 0x48, 0x00}; /*rewind*/
    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_CTRL,
                       AVRCP_PASS_THROUGH_SUBUNIT_TYPE,
                       AVRCP_PASS_THROUGH_SUBUNIT_ID,
                       3,
                       data);


    /*U8 *record;
     U16 num_rec_bytes;
     num_rec_bytes = sizeof(bts2s_sds_pan_gn_svc_record);
     record = (U8 *) bmalloc(num_rec_bytes);
     bmemcpy(record, bts2s_sds_pan_gn_svc_record, num_rec_bytes);
     gap_sds_reg_req(bts2_app_data->phdl, record, num_rec_bytes);*/
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
void bt_avrcp_record(bts2_app_stru *bts2_app_data)
{
    U8 data[3] = {0x7c, 0x47, 0x00}; /*record*/
    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_CTRL,
                       AVRCP_PASS_THROUGH_SUBUNIT_TYPE,
                       AVRCP_PASS_THROUGH_SUBUNIT_ID,
                       3,
                       data);


    /*U8 *record;
      U16 num_rec_bytes;
      num_rec_bytes = sizeof(bts2s_sds_pan_nap_svc_record);
      record = (U8 *) bmalloc(num_rec_bytes);
      bmemcpy(record, bts2s_sds_pan_nap_svc_record, num_rec_bytes);
      gap_sds_reg_req(bts2_app_data->phdl, record, num_rec_bytes);*/
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
void bt_avrcp_select_sound(bts2_app_stru *bts2_app_data)
{
    U8 data[3] = {0x7c, 0x33, 0x00}; //record
    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_CTRL,
                       AVRCP_PASS_THROUGH_SUBUNIT_TYPE,
                       AVRCP_PASS_THROUGH_SUBUNIT_ID,
                       3,
                       data);
}

void bt_avrcp_volume_up(bts2_app_stru *bts2_app_data)
{
    U8 data[3] = {0x7c, 0x41, 0x00}; //ply
    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_CTRL,
                       AVRCP_PASS_THROUGH_SUBUNIT_TYPE,
                       AVRCP_PASS_THROUGH_SUBUNIT_ID,
                       3,
                       data);

    bt_avrcp_delay_pop(bts2_app_data, 0x41);
}

void bt_avrcp_volume_down(bts2_app_stru *bts2_app_data)
{
    U8 data[3] = {0x7c, 0x42, 0x00}; //ply
    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_CTRL,
                       AVRCP_PASS_THROUGH_SUBUNIT_TYPE,
                       AVRCP_PASS_THROUGH_SUBUNIT_ID,
                       3,
                       data);


    bt_avrcp_delay_pop(bts2_app_data, 0x42);
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
void bt_avrcp_stop(bts2_app_stru *bts2_app_data)
{
    U8 data[3] = {0x7c, 0x45, 0x00}; //stop

    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_CTRL,
                       AVRCP_PASS_THROUGH_SUBUNIT_TYPE,
                       AVRCP_PASS_THROUGH_SUBUNIT_ID,
                       3,
                       data);


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
void bt_avrcp_pause(bts2_app_stru *bts2_app_data)
{
    U8 data[3] = {0x7c, 0x46, 0x00}; //pause
    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_CTRL,
                       AVRCP_PASS_THROUGH_SUBUNIT_TYPE,
                       AVRCP_PASS_THROUGH_SUBUNIT_ID,
                       3,
                       data);
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
void bt_avrcp_forward(bts2_app_stru *bts2_app_data)
{
    U8 data[3] = {0x7c, 0x4b, 0x00}; //forward

    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_CTRL,
                       AVRCP_PASS_THROUGH_SUBUNIT_TYPE,
                       AVRCP_PASS_THROUGH_SUBUNIT_ID,
                       3,
                       data);


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
void bt_avrcp_backward(bts2_app_stru *bts2_app_data)
{
    //backward
    U8 data[3] = {0x7c, 0x4c, 0x00};

    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_CTRL,
                       AVRCP_PASS_THROUGH_SUBUNIT_TYPE,
                       AVRCP_PASS_THROUGH_SUBUNIT_ID,
                       3,
                       data);



    /* test for get capbality */
    /*U8 data[9] = {0x00, 0x00, 0x19, 0x58, 0x10, 0x00, 0x00, 0x01, 0x01};
    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_CTRL,
                       AVRCP_PASS_THROUGH_SUBUNIT_TYPE,
                       AVRCP_PASS_THROUGH_SUBUNIT_ID,
                       9,
                       data);*/

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
void bt_avrcp_unitinfo(bts2_app_stru *bts2_app_data)
{
    U8 data[64] = {0x30, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_STS,
                       AVRCP_UNIT_INFO_SUBUNIT_TYPE,
                       AVRCP_UNIT_INFO_SUBUNIT_ID,
                       64,
                       data);
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
void bt_avrcp_subunitinfo(bts2_app_stru *bts2_app_data)
{
    U8 data[6] = {0x31, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_STS,
                       AVRCP_SUBUNIT_INFO_SUBUNIT_TYPE,
                       AVRCP_SUBUNIT_INFO_SUBUNIT_ID,
                       6,
                       data);
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
void bt_avrcp_deint(bts2_app_stru *bts2_app_data)
{

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
static void bt_avrcp_hdl_subunitinfo_cmd_ind(bts2_app_stru *bts2_app_data)
{
    BTS2S_AVRCP_SUBUNIT_INFO_CMD_CFM *avrcmsg;
    U8 data[6] = {0x31, 0x07, 0x48, 0xff, 0xff, 0xff};
    avrcmsg = (BTS2S_AVRCP_SUBUNIT_INFO_CMD_CFM *)bts2_app_data->recv_msg;
    INFO_TRACE("<< recive avrcp subunit info command indication\n");
    if (avrcmsg->c_type == AVRCP_CR_STS)
    {
        if (avrcmsg->prof_id == BT_UUID_AVRCP_CT)
        {
            U8 vld_cmd = TRUE;
//          if ((avrcmsg->subunit_id != AVRCP_SUBUNIT_INFO_SUBUNIT_TYPE)||
//              (avrcmsg->subunit_type != AVRCP_SUBUNIT_INFO_SUBUNIT_ID))
//          {
//              vld_cmd = FALSE;
//          }
            /*send response */
            if (vld_cmd == TRUE)
            {
                avrcp_cmd_data_rsp(bts2_app_data->phdl,
                                   avrcmsg->tlabel,
                                   avrcmsg->prof_id,
                                   AVRCP_CR_STABLE,
                                   avrcmsg->subunit_type,
                                   avrcmsg->subunit_id,
                                   (U16)6,
                                   data);
            }
        }
        else
        {
            avrcp_cmd_data_rsp(bts2_app_data->phdl,
                               avrcmsg->tlabel,
                               avrcmsg->prof_id,
                               AVRCP_CR_INVLD_PID,
                               avrcmsg->subunit_type,
                               avrcmsg->subunit_id,
                               0,
                               NULL);
        }
    }

    if (avrcmsg->data != NULL)
    {
        bfree(avrcmsg->data);
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
static void bt_avrcp_hdl_unitinfo_cmd_ind(bts2_app_stru *bts2_app_data)
{
    BTS2S_AVRCP_UNIT_INFO_CMD_CFM *avrcmsg;
    U8 data[64] = {0x30, 0x07, 0x48, 0xFF, 0xFF, 0xFF};
    avrcmsg = (BTS2S_AVRCP_UNIT_INFO_CMD_CFM *)bts2_app_data->recv_msg;
    INFO_TRACE("<< recive avrcp unit info command indication\n");

    if (avrcmsg->c_type == AVRCP_CR_STS)
    {
        if (avrcmsg->prof_id == BT_UUID_AVRCP_CT)
        {
            U8 vld_cmd = TRUE;
//          if ((avrcmsg->subunit_id != AVRCP_UNIT_INFO_SUBUNIT_TYPE)||
//              (avrcmsg->subunit_type != AVRCP_UNIT_INFO_SUBUNIT_ID))
//          {
//              vld_cmd = FALSE;
//          }
            /*send response */
            if (vld_cmd == TRUE)
            {
                avrcp_cmd_data_rsp(bts2_app_data->phdl,
                                   avrcmsg->tlabel,
                                   avrcmsg->prof_id,
                                   AVRCP_CR_STABLE,
                                   avrcmsg->subunit_type,
                                   avrcmsg->subunit_id,
                                   (U16)64,
                                   data);
            }
        }
        else
        {
            avrcp_cmd_data_rsp(bts2_app_data->phdl,
                               avrcmsg->tlabel,
                               avrcmsg->prof_id,
                               AVRCP_CR_INVLD_PID,
                               avrcmsg->subunit_type,
                               avrcmsg->subunit_id,
                               0,
                               NULL);
        }
    }

    if (avrcmsg->data != NULL)
    {
        bfree(avrcmsg->data);
    }
}

U8 VENDOR_DEPENDENT_BLUETOOTH_SIG_ID[4] = {0x00, 0x00, 0x19, 0x58};

void bt_avrcp_get_capabilities_request(bts2_app_stru *bts2_app_data)
{
    U8 data_len = 9;
    U8 data[9];
    memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_CAPABILITIES;
    *(data + 5) = 0;

    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 1;

    // parameter
    *(data + 8) = AVRCP_VENDOR_DEPENDENT_EVENT_CAPABILITY_FOR_EVENTS;
    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_STS,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}

void bt_avrcp_get_capabilities_response(bts2_app_stru *bts2_app_data, int tlabel)
{
    U8 data_len = 12;
    U8 data[12];

    memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_CAPABILITIES;
    *(data + 5) = 0;

    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 3;

    // parameter
    // capability
    *(data + 8) = AVRCP_VENDOR_DEPENDENT_EVENT_CAPABILITY_FOR_EVENTS;
    // capability count
    *(data + 9) = 2;
    // event ID volume changed
    *(data + 10) = 0x01;
    *(data + 11) = 0x0d;

    avrcp_cmd_data_rsp(bts2_app_data->phdl,
                       tlabel,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_STABLE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}

void bt_avrcp_get_play_status_response(bts2_app_stru *bts2_app_data, int tlabel)
{
    U8 data_len = 17;
    U8 data[17];
    memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_PLAY_STATUS;
    *(data + 5) = 0;
    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 9;
    // parameter
    // song length
    *(data + 8) = 0xFF;
    *(data + 9) = 0xFF;
    *(data + 10) = 0xFF;
    *(data + 11) = 0xFF;
    // position
    *(data + 12) = 0xFF;
    *(data + 13) = 0xFF;
    *(data + 14) = 0xFF;
    *(data + 15) = 0xFF;
    // play status
    *(data + 16) = bt_av_get_a2dp_stream_state();
    avrcp_cmd_data_rsp(bts2_app_data->phdl,
                       tlabel,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_STABLE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}
void bt_avrcp_get_company_id_capabilities_response(bts2_app_stru *bts2_app_data, int tlabel)
{
    U8 data_len = 13;
    U8 data[13];
    memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_CAPABILITIES;
    *(data + 5) = 0;
    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 5;
    // parameter
    // capability
    *(data + 8) = AVRCP_VENDOR_DEPENDENT_EVENT_CAPABILITY_COMPANY_ID;
    // capability count
    *(data + 9) = 1;
    // company id
    *(data + 10) = 0x00;
    *(data + 11) = 0x19;
    *(data + 12) = 0x58;
    avrcp_cmd_data_rsp(bts2_app_data->phdl,
                       tlabel,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_STABLE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}
void bt_avrcp_get_element_attributes_response(bts2_app_stru *bts2_app_data, int tlabel)
{
    U16 data_len = 494;
    U8 *data = bmalloc(data_len);
    BT_OOM_ASSERT(data);
    if (data)
    {
        memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
        *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_ELEMENT_ATTRIBUTES;
        *(data + 5) = 1;
        // parameter length
        *(data + 6) = 0x01;
        *(data + 7) = 0xF6;
        // number of attributes
        *(data + 8) = 1;
        // attribute id (title)
        *(data + 9) = 0;
        *(data + 10) = 0;
        *(data + 11) = 0;
        *(data + 12) = 1;
        // character set id utf8
        *(data + 13) = 0;
        *(data + 14) = 0x6a;
        // attirbute value length
        *(data + 15) = 2;
        *(data + 16) = 0x08;
        // value
        *(data + 17) = 0x4d;
        *(data + 18) = 0x49;
        *(data + 19) = 0x52;
        *(data + 20) = 0x52;
        *(data + 21) = 0x4f;
        *(data + 22) = 0x52;
        avrcp_cmd_data_rsp(bts2_app_data->phdl,
                           tlabel,
                           BT_UUID_AVRCP_CT,
                           AVRCP_CR_STABLE,
                           AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                           AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                           data_len,
                           data);
        bfree(data);
    }
}
void bt_avrcp_get_element_attributes_response_continue(bts2_app_stru *bts2_app_data, int tlabel)
{
    U16 data_len = 35;
    U8 data[35];
    memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_ELEMENT_ATTRIBUTES;
    *(data + 5) = 3;
    // parameter length
    *(data + 6) = 0x00;
    *(data + 7) = 0x1b;
    *(data + 8) = 0x4d;
    *(data + 9) = 0x4d;
    avrcp_cmd_data_rsp(bts2_app_data->phdl,
                       tlabel,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_STABLE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}
void bt_avrcp_get_element_attributes_response_abort(bts2_app_stru *bts2_app_data, int tlabel)
{
    U16 data_len = 8;
    U8 data[8];
    memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_ABORT_CONTINUING_RESPONSE;
    *(data + 5) = 0;
    // parameter length
    *(data + 6) = 0x00;
    *(data + 7) = 0x00;
    avrcp_cmd_data_rsp(bts2_app_data->phdl,
                       tlabel,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_ACPT,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}
void bt_avrcp_reject_response(bts2_app_stru *bts2_app_data, int tlabel, int command, int error_code)
{
    U8 data_len = 9;
    U8 data[9];
    memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
    *(data + 4) = command;
    *(data + 5) = 0;
    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 1;
    // parameter
    // capability
    *(data + 8) = error_code;
    avrcp_cmd_data_rsp(bts2_app_data->phdl,
                       tlabel,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_REJECT,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}
void bt_avrcp_playback_register_request(bts2_app_stru *bts2_app_data)
{
    U8 data_len = 13;
    U8 data[13];
    bmemcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);

    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_REGISTER_NOTIFICATION;
    *(data + 5) = 0;

    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 5;

    // parameter
    *(data + 8) = AVRCP_VENDOR_DEPENDENT_EVENT_PLAYBACK_STATUS_CHANGED;
    bmemset(data + 9, 0, 4);

    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_NOTIFY,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}

void bt_avrcp_playback_pos_register_request(bts2_app_stru *bts2_app_data)
{
    U8 data_len = 13;
    U8 data[13];
    bmemcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);

    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_REGISTER_NOTIFICATION;
    *(data + 5) = 0;

    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 5;

    // parameter
    *(data + 8) = AVRCP_VENDOR_DEPENDENT_EVENT_PLAYBACK_POS_CHANGED;
    bmemset(data + 9, 0, 4);
    data[12] = 1;//interval :1s

    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_NOTIFY,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}



void bt_avrcp_track_register_request(bts2_app_stru *bts2_app_data)
{
    U8 data_len = 13;
    U8 data[13];
    bmemcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);

    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_REGISTER_NOTIFICATION;
    *(data + 5) = 0;

    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 5;

    // parameter
    *(data + 8) = AVRCP_VENDOR_DEPENDENT_EVENT_TRACK_CHANGED;
    bmemset(data + 9, 0, 4);

    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_NOTIFY,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}

void bt_avrcp_get_capabilities_confirm(bts2_app_stru *bts2_app_data, BTS2S_AVRCP_VENDOR_DEPEND_CMD_CFM *avrcmsg)
{
    U16 paramter_length;
    bmemcpy(&paramter_length, avrcmsg->data + 6, sizeof(U16));

    if (paramter_length > 2)
    {
        U8 capability = avrcmsg->data[8];
        if (capability == AVRCP_VENDOR_DEPENDENT_EVENT_CAPABILITY_FOR_EVENTS)
        {
            U8 capability_count = avrcmsg->data[9];

            U8 *events_id = NULL;

            if (capability_count > 0)
            {
                events_id = bmalloc(capability_count);
                BT_OOM_ASSERT(events_id);
            }
            if (events_id)
            {
                bmemcpy(events_id, avrcmsg->data + 10, capability_count);

                for (int i = 0; i < capability_count; i++)
                {
                    U8 event = *(events_id + i);
                    if (event == AVRCP_VENDOR_DEPENDENT_EVENT_PLAYBACK_STATUS_CHANGED)
                    {
                        bt_avrcp_playback_register_request(bts2_app_data);
                    }
                    else if (event == AVRCP_VENDOR_DEPENDENT_EVENT_TRACK_CHANGED)
                    {
                        bt_avrcp_track_register_request(bts2_app_data);
                    }
                    else if (event == AVRCP_VENDOR_DEPENDENT_EVENT_VOLUME_CHANGED)
                    {
                        bt_avrcp_volume_register_request(bts2_app_data);
                    }
                    else if (AVRCP_VENDOR_DEPENDENT_EVENT_PLAYBACK_POS_CHANGED == event)
                    {
                        bt_avrcp_playback_pos_register_request(bts2_app_data);
                    }
                }

                // TODO: update remote capability for events;
                bfree(events_id);
            }
        }
    }
}

bt_err_t bt_avrcp_change_volume(bts2_app_stru *bts2_app_data, U8 volume)
{
    BEGIN_ACCESS_VAR();
    USER_TRACE("absolute volume tgRegStatus%x volume%x \n", bts2_app_data->avrcp_inst.tgRegStatus, volume);

    if (1 == bts2_app_data->avrcp_inst.tgRegStatus)
    {
        bts2_app_data->avrcp_inst.ab_volume = volume;
        bts2_app_data->avrcp_inst.tgRegStatus = 0;//the life cycle of volume change:(begin:CT register; end:TG changed)
        bt_avrcp_volume_register_response(bts2_app_data, AVRCP_CR_CHANGED, volume);
        END_ACCESS_VAR();
        return BT_EOK;
    }
    else
    {
        END_ACCESS_VAR();
        return BT_ERROR_AVRCP_NO_REG;
        /* if (volume > (bts2_app_data->avrcp_inst.ab_volume))
         {
             USER_TRACE("volume up\n");
             bt_avrcp_volume_up(bts2_app_data);
         }
         else if (volume < (bts2_app_data->avrcp_inst.ab_volume))
         {
             USER_TRACE("volume down\n");
             bt_avrcp_volume_down(bts2_app_data);
         }
         else
         {
             return;
         }*/
    }
}

bt_err_t bt_avrcp_change_play_status(bts2_app_stru *bts2_app_data, U8 play_status)
{
    BEGIN_ACCESS_VAR();
    USER_TRACE("play status tgRegStatus%x play_status%d \n", bts2_app_data->avrcp_inst.tgRegStatus1, play_status);

    if (1 == bts2_app_data->avrcp_inst.tgRegStatus1)
    {
        bts2_app_data->avrcp_inst.tgRegStatus1 = 0;//the life cycle of volume change:(begin:CT register; end:TG changed)
        bt_avrcp_play_status_changed_register_response(bts2_app_data, AVRCP_CR_CHANGED, play_status);
        END_ACCESS_VAR();
        return BT_EOK;
    }
    else
    {
        END_ACCESS_VAR();
        return BT_ERROR_AVRCP_NO_REG;
        /* if (volume > (bts2_app_data->avrcp_inst.ab_volume))
         {
             USER_TRACE("volume up\n");
             bt_avrcp_volume_up(bts2_app_data);
         }
         else if (volume < (bts2_app_data->avrcp_inst.ab_volume))
         {
             USER_TRACE("volume down\n");
             bt_avrcp_volume_down(bts2_app_data);
         }
         else
         {
             return;
         }*/
    }
}


static void bt_avrcp_get_element_attributes_request(bts2_app_stru *bts2_app_data, U8 media_attribute)
{
    U8 data_len = 21;
    U8 data[21];
    bmemcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);

    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_ELEMENT_ATTRIBUTES;
    *(data + 5) = 0;

    // parameter length
    *(data + 6) = 0;

    U8 para_len = 13;
    *(data + 7) = para_len;

    // parameter
    U8 para[13] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0};
    bmemcpy(data + 8, para, para_len);

    *(data + 20) = media_attribute;

    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_STS,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}



void bt_avrcp_get_play_status_request(bts2_app_stru *bts2_app_data)
{
    U8 data_len = 8;
    U8 data[8];
    bmemcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);

    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_PLAY_STATUS;
    *(data + 5) = 0;

    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 0;

    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_STS,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}

static void bt_avrcp_get_element_attributes_confirm(bts2_app_stru *bts2_app_data, BTS2S_AVRCP_VENDOR_DEPEND_CMD_CFM *avrcmsg)
{
    U16 paramter_length;
    paramter_length = ((avrcmsg->data[6] << 8) | (avrcmsg->data[7]));


    U8 attribute_rej;
    if (paramter_length == 1)
    {
        attribute_rej = *(avrcmsg->data + 8);
        USER_TRACE("avrcp attr_cfm reject %d\n", attribute_rej);
        return;
    }
    if (paramter_length < 9)
    {
        USER_TRACE("avrcp attr_cfm short len %d\n", paramter_length);
        return;
    }

    //data[8]:number of attribute,is 1 general.jump
    U32 media_attribute;
    media_attribute = ((avrcmsg->data[9] << 24) | (avrcmsg->data[10] << 16) | (avrcmsg->data[11] << 8) | (avrcmsg->data[12]));
    USER_TRACE("avrcp media_attribute %x  req_attr %x\n", media_attribute, mp3_detail_info.attri_req);

    mp3_detail_info.detail_info.character_set_id = ((avrcmsg->data[13] << 8) | (avrcmsg->data[14]));
    USER_TRACE("avrcp character_set_id %x\n", mp3_detail_info.detail_info.character_set_id);

    U16 value_length;
    value_length = ((avrcmsg->data[15] << 8) | (avrcmsg->data[16]));
    USER_TRACE("avrcp value_length %x\n", value_length);

    U8 *value;

    if (media_attribute != mp3_detail_info.attri_req)
    {
        return;
    }

    switch (media_attribute)
    {
    case AVRCP_MEDIA_ATTRIBUTES_GENRE:
    {
        if (value_length != 0)
        {
            // TODO: process value
#if 0
            value = bmalloc(value_length);
            bmemcpy(value, avrcmsg->data + 17, value_length);
            bfree(value);
#endif
        }
        mp3_detail_info.attri_req = AVRCP_MEDIA_ATTRIBUTES_ARTIST;
        bt_avrcp_get_element_attributes_request(bts2_app_data, AVRCP_MEDIA_ATTRIBUTES_ARTIST);
        break;
    }
    case AVRCP_MEDIA_ATTRIBUTES_ARTIST:
    {
        if (value_length > BT_MAX_SINGER_NAME_LEN)
        {
            value_length = BT_MAX_SINGER_NAME_LEN;
        }

        if (value_length != 0)
        {
            value = (U8 *)(avrcmsg->data + 17);

            mp3_detail_info.detail_info.singer_name.size = value_length;
            memcpy(mp3_detail_info.detail_info.singer_name.singer_name, (const void *) value, value_length);

            // TODO: process value
#if 0
            value = bmalloc(value_length);
            bmemcpy(value, avrcmsg->data + 17, value_length);
            bfree(value);
#endif
        }
        else
        {
            mp3_detail_info.detail_info.singer_name.size = 0;
        }
        mp3_detail_info.attri_req = AVRCP_MEDIA_ATTRIBUTES_ALBUM;
        bt_avrcp_get_element_attributes_request(bts2_app_data, AVRCP_MEDIA_ATTRIBUTES_ALBUM);
        break;
    }
    case AVRCP_MEDIA_ATTRIBUTES_ALBUM:
    {
        if (value_length > BT_MAX_ALBUM_INFO_LEN)
        {
            value_length = BT_MAX_ALBUM_INFO_LEN;
        }

        if (value_length != 0)
        {
            value = (U8 *)(avrcmsg->data + 17);

            mp3_detail_info.detail_info.album_info.size = value_length;
            memcpy(mp3_detail_info.detail_info.album_info.album_name, (const void *) value, value_length);
            // TODO: process value
#if 0
            value = bmalloc(value_length);
            bmemcpy(value, avrcmsg->data + 17, value_length);
            bfree(value);
#endif
        }
        else
        {
            mp3_detail_info.detail_info.album_info.size = 0;
        }
        mp3_detail_info.attri_req = AVRCP_MEDIA_ATTRIBUTES_TITLE;
        bt_avrcp_get_element_attributes_request(bts2_app_data, AVRCP_MEDIA_ATTRIBUTES_TITLE);
        break;
    }
    case AVRCP_MEDIA_ATTRIBUTES_TITLE:
    {
        if (value_length > BT_MAX_SONG_NAME_LEN)
        {
            value_length = BT_MAX_SONG_NAME_LEN;
        }

        if (value_length != 0)
        {
            value = (U8 *)(avrcmsg->data + 17);

            mp3_detail_info.detail_info.song_name.size = value_length;
            memcpy(mp3_detail_info.detail_info.song_name.song_name, (const void *) value, value_length);
            // TODO: process value
#if 0
            value = bmalloc(value_length);
            bmemcpy(value, avrcmsg->data + 17, value_length);
            bfree(value);
#endif
        }
        else
        {
            mp3_detail_info.detail_info.song_name.size = 0;
        }
        mp3_detail_info.attri_req = AVRCP_MEDIA_ATTRIBUTES_PLAYTIME;
        bt_avrcp_get_element_attributes_request(bts2_app_data, AVRCP_MEDIA_ATTRIBUTES_PLAYTIME);
        break;
    }
    case AVRCP_MEDIA_ATTRIBUTES_PLAYTIME:
    {
        if (value_length > BT_MAX_PLAY_TIME_LEN)
        {
            value_length = BT_MAX_PLAY_TIME_LEN;
        }

        if (value_length != 0)
        {
            value = (U8 *)(avrcmsg->data + 17);

            mp3_detail_info.detail_info.duration.size = value_length;
            bmemcpy(mp3_detail_info.detail_info.duration.play_time, avrcmsg->data + 17, value_length);
            // TODO: process value
#if 0
            value = bmalloc(value_length);
            bmemcpy(value, avrcmsg->data + 17, value_length);
            bfree(value);
#endif
        }
        else
        {
            mp3_detail_info.detail_info.duration.size = 0;
        }

        mp3_detail_info.attri_req = 0x00;

        if ((0 == mp3_detail_info.detail_info.singer_name.size) && (0 == mp3_detail_info.detail_info.album_info.size))
        {
            break;
        }
        USER_TRACE("URC BT mp3 detail end %x\n", mp3_detail_info.track_id);
#if defined(CFG_AVRCP)
        bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_MP3_DETAIL_INFO, &mp3_detail_info.detail_info, sizeof(bt_mp3_detail_info_t));
#endif
        break;
    }
    default:
        ;
    }
}

void bt_avrcp_get_play_status_confirm(bts2_app_stru *bts2_app_data, BTS2S_AVRCP_VENDOR_DEPEND_CMD_CFM *avrcmsg)
{
    U8 play_status = 0;
    play_status = avrcmsg->data[16];

    rt_kprintf("get play_status = %d\n", play_status);

#if defined(CFG_AVRCP)
    //solution 0x00:playing ;0x01:paused
    uint8_t play_status_notify = play_status - 1;
    bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_PLAY_STATUS, &play_status_notify, sizeof(uint8_t));
#endif
}

// register absolute voulume notify
void bt_avrcp_volume_register_request(bts2_app_stru *bts2_app_data)
{
    U8 data_len = 13;
    U8 data[13];
    bmemcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);

    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_REGISTER_NOTIFICATION;
    *(data + 5) = 0;

    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 5;

    // parameter
    *(data + 8) = AVRCP_VENDOR_DEPENDENT_EVENT_VOLUME_CHANGED;
    bmemset(data + 9, 0, 4);

    avrcp_cmd_data_req(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_NOTIFY,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}

// response should be AVRCP_CR_INTERIM or AVRCP_CR_CHANGED
void bt_avrcp_volume_register_response(bts2_app_stru *bts2_app_data, U8 response, U8 volume)
{
    U8 data_len = 10;
    U8 data[10];

    memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_REGISTER_NOTIFICATION;
    *(data + 5) = 0;

    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 2;

    // parameter
    *(data + 8) = AVRCP_VENDOR_DEPENDENT_EVENT_VOLUME_CHANGED;
    *(data + 9) = volume;
    avrcp_cmd_data_rsp(bts2_app_data->phdl,
                       bts2_app_data->avrcp_inst.tgTlable,
                       BT_UUID_AVRCP_CT,
                       response,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}

void bt_avrcp_track_changed_register_response(bts2_app_stru *bts2_app_data, U8 response, U8 track_changed)
{
    U8 data_len = 17;
    U8 data[17];
    memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_REGISTER_NOTIFICATION;
    *(data + 5) = 0;
    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 9;
    // parameter
    *(data + 8) = AVRCP_VENDOR_DEPENDENT_EVENT_TRACK_CHANGED;
    if (track_changed == 0)
    {
        memset(data + 9, 0xff, 8);
    }
    else
    {
        memset(data + 9, 0, 8);
    }
    avrcp_cmd_data_rsp(bts2_app_data->phdl,
                       ASSIGN_TLABEL,
                       BT_UUID_AVRCP_CT,
                       response,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}


void bt_avrcp_play_status_changed_register_response(bts2_app_stru *bts2_app_data, U8 response, U8 play_status)
{
    U8 data_len = 10;
    U8 data[10];
    memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_REGISTER_NOTIFICATION;
    *(data + 5) = 0;
    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 2;
    // parameter
    *(data + 8) = AVRCP_VENDOR_DEPENDENT_EVENT_PLAYBACK_STATUS_CHANGED;
    *(data + 9) = play_status;

    avrcp_cmd_data_rsp(bts2_app_data->phdl,
                       bts2_app_data->avrcp_inst.tgTlable_1,
                       BT_UUID_AVRCP_CT,
                       response,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}

bt_err_t bt_avrcp_set_absolute_volume_request(bts2_app_stru *bts2_app_data, U8 volume)
{
    if (bts2_app_data->avrcp_inst.abs_volume_pending == 1)
    {
        return BT_ERROR_STATE;
    }
    else
    {
        U8 data_len = 9;
        U8 *data = malloc(data_len);

        memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
        *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_SET_ABSOLUTE_VOLUME;
        *(data + 5) = 0;

        // parameter length
        *(data + 6) = 0;
        *(data + 7) = 1;

        // parameter
        *(data + 8) = volume;

        avrcp_cmd_data_req(bts2_app_data->phdl,
                           ASSIGN_TLABEL,
                           BT_UUID_AVRCP_CT,
                           AVRCP_CR_CTRL,
                           AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                           AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                           data_len,
                           data);
        bts2_app_data->avrcp_inst.abs_volume_pending = 1;
        free(data);
        return BT_EOK;
    }
}

void bt_avrcp_set_absolute_volume_response(bts2_app_stru *bts2_app_data, U8 volume)
{
    BTS2S_AVRCP_VENDOR_DEPEND_CMD_CFM *avrcmsg;
    avrcmsg = (BTS2S_AVRCP_VENDOR_DEPEND_CMD_CFM *)bts2_app_data->recv_msg;

    U8 data_len = 9;
    U8 data[9];
    memcpy(data, VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, 4);
    *(data + 4) = AVRCP_VENDOR_DEPENDENT_PDU_ID_SET_ABSOLUTE_VOLUME;
    *(data + 5) = 0;

    // parameter length
    *(data + 6) = 0;
    *(data + 7) = 1;

    // parameter
    *(data + 8) = volume;
    avrcp_cmd_data_rsp(bts2_app_data->phdl,
                       avrcmsg->tlabel,
                       BT_UUID_AVRCP_CT,
                       AVRCP_CR_ACPT,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_TYPE,
                       AVRCP_VENDOR_DEPENDENT_SUBUNIT_ID,
                       data_len,
                       data);
}

static void bt_avrcp_hdl_vendor_depend_cmd_ind(bts2_app_stru *bts2_app_data)
{
    BTS2S_AVRCP_VENDOR_DEPEND_CMD_CFM *avrcmsg;
    avrcmsg = (BTS2S_AVRCP_VENDOR_DEPEND_CMD_CFM *)bts2_app_data->recv_msg;
    INFO_TRACE("<<avrcp vendor cmd ind\n");

    if (avrcmsg->c_type == AVRCP_CR_NOTIFY)
    {
        if (avrcmsg->prof_id == BT_UUID_AVRCP_CT)
        {
            U8 op_code = avrcmsg->data[0];
            U8 pdu_id = avrcmsg->data[4];

            U8 company_id[4];
            bmemcpy(&company_id, avrcmsg->data, sizeof(U32));

            if (bmemcmp(VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, company_id, 4) != 0)
            {
                // TODO: error not bluetooth sig vendor dependent command
                if (avrcmsg->data != NULL)
                {
                    bfree(avrcmsg->data);
                }
                return;
            }

            if (pdu_id == AVRCP_VENDOR_DEPENDENT_PDU_ID_REGISTER_NOTIFICATION)
            {
                U8 event_id = avrcmsg->data[8];
                switch (event_id)
                {
                case AVRCP_VENDOR_DEPENDENT_EVENT_VOLUME_CHANGED:
                {
                    BEGIN_ACCESS_VAR();
                    U8 current_volume = bts2_app_data->avrcp_inst.ab_volume;

                    bts2_app_data->avrcp_inst.tgTlable  = avrcmsg->tlabel;

                    USER_TRACE("<< register notificaiton volume changed %d\n", current_volume);

                    bts2_app_data->avrcp_inst.tgRegStatus = 1;
                    bt_avrcp_volume_register_response(bts2_app_data, AVRCP_CR_INTERIM, current_volume);
                    END_ACCESS_VAR();

#if defined(CFG_AVRCP)
                    bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_VOLUME_CHANGED_REGISTER, NULL, 0);
#endif
                    break;
                }
                case AVRCP_VENDOR_DEPENDENT_EVENT_TRACK_CHANGED:
                {
#ifdef BSP_BQB_TEST
                    switch (BQB_TEST_CASE)
                    {
                    case AVRCP_TG_NFY_BV_05_C:
                    case AVRCP_TG_NFY_BV_08_C:
                        bt_avrcp_track_changed_register_response(bts2_app_data, AVRCP_CR_INTERIM, 1);
                        break;
                    default:
                        bt_avrcp_track_changed_register_response(bts2_app_data, AVRCP_CR_INTERIM, 0);
                        break;
                    }
#else
                    bt_avrcp_track_changed_register_response(bts2_app_data, AVRCP_CR_INTERIM, 1);
#endif
                    break;
                }
                case AVRCP_VENDOR_DEPENDENT_EVENT_PLAYBACK_STATUS_CHANGED:
                {
                    BEGIN_ACCESS_VAR();
                    U8 play_status = bt_av_get_a2dp_stream_state();

                    bts2_app_data->avrcp_inst.tgTlable_1  = avrcmsg->tlabel;

                    USER_TRACE("<< register play status changed %d\n", play_status);

                    bts2_app_data->avrcp_inst.tgRegStatus1 = 1;
                    bt_avrcp_play_status_changed_register_response(bts2_app_data, AVRCP_CR_INTERIM, play_status);
                    END_ACCESS_VAR();
                    break;
                }
                default:
                {
                    bt_avrcp_reject_response(bts2_app_data, avrcmsg->tlabel, AVRCP_VENDOR_DEPENDENT_PDU_ID_REGISTER_NOTIFICATION, AVRCP_REJECT_ERROR_INVALID_PARAMETER);
                }

                }
            }
        }
        else
        {
            avrcp_cmd_data_rsp(bts2_app_data->phdl,
                               avrcmsg->tlabel,
                               avrcmsg->prof_id,
                               AVRCP_CR_INVLD_PID,
                               avrcmsg->subunit_type,
                               avrcmsg->subunit_id,
                               0,
                               NULL);
        }
    }
    else if (avrcmsg->c_type == AVRCP_CR_CTRL)
    {
        if (avrcmsg->prof_id == BT_UUID_AVRCP_CT)
        {
            U8 op_code = avrcmsg->data[0];
            U8 pdu_id = avrcmsg->data[4];

            U8 company_id[4];
            U16 para_len;
            memcpy(&para_len, &avrcmsg->data[6], 2);
            bmemcpy(&company_id, avrcmsg->data, sizeof(U32));

            if (bmemcmp(VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, company_id, 4) != 0)
            {
                // TODO: error not bluetooth sig vendor dependent command
                if (avrcmsg->data != NULL)
                {
                    bfree(avrcmsg->data);
                }
                return;
            }

            switch (pdu_id)
            {
            case AVRCP_VENDOR_DEPENDENT_PDU_ID_SET_ABSOLUTE_VOLUME:
            {
                U8 volume = avrcmsg->data[8];

                if (para_len == 0)
                {
                    bt_avrcp_reject_response(bts2_app_data, avrcmsg->tlabel, AVRCP_VENDOR_DEPENDENT_PDU_ID_SET_ABSOLUTE_VOLUME, AVRCP_REJECT_ERROR_INVALID_PARAMETER);
                    break;
                }

                // TODO: set volume
                bts2_app_data->avrcp_inst.ab_volume = volume;
#ifdef AUDIO_USING_MANAGER
                uint8_t relative_volume = 0;
                relative_volume = a2dp_set_speaker_volume(volume);

#if defined(CFG_AVRCP)
                bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_ABSOLUTE_VOLUME, &relative_volume, sizeof(uint8_t));
#endif
                USER_TRACE("<< set absolute volume %d relative_volume:%d\n", volume, relative_volume);
#endif

                bt_avrcp_set_absolute_volume_response(bts2_app_data, volume);
                break;
            }
            case AVRCP_VENDOR_DEPENDENT_PDU_ID_REQUEST_CONTINUING_RESPONSE:
            {
                bt_avrcp_get_element_attributes_response_continue(bts2_app_data, avrcmsg->tlabel);
                break;
            }
            case AVRCP_VENDOR_DEPENDENT_PDU_ID_ABORT_CONTINUING_RESPONSE:
            {
                bt_avrcp_get_element_attributes_response_abort(bts2_app_data, avrcmsg->tlabel);
                break;
            }
            }
        }
    }
    else if (avrcmsg->c_type == AVRCP_CR_STS)
    {
        if (avrcmsg->prof_id == BT_UUID_AVRCP_CT)
        {
            U8 op_code = avrcmsg->data[0];
            U8 pdu_id = avrcmsg->data[4];

            U8 company_id[4];
            bmemcpy(&company_id, avrcmsg->data, sizeof(U32));

            if (bmemcmp(VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, company_id, 4) != 0)
            {
                // TODO: error not bluetooth sig vendor dependent command
                if (avrcmsg->data != NULL)
                {
                    bfree(avrcmsg->data);
                }
                return;
            }

            switch (pdu_id)
            {
            case AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_CAPABILITIES:
            {
                //U8 volume = avrcmsg->data[8];

                // TODO: set volume
                U8 capability_id = avrcmsg->data[8];
                switch (capability_id)
                {
                case AVRCP_VENDOR_DEPENDENT_EVENT_CAPABILITY_COMPANY_ID:
                {
                    bt_avrcp_get_company_id_capabilities_response(bts2_app_data, avrcmsg->tlabel);
                    break;
                }
                case AVRCP_VENDOR_DEPENDENT_EVENT_CAPABILITY_FOR_EVENTS:
                {
                    bt_avrcp_get_capabilities_response(bts2_app_data, avrcmsg->tlabel);
                    break;
                }
                default:
                    bt_avrcp_reject_response(bts2_app_data, avrcmsg->tlabel, AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_CAPABILITIES, AVRCP_REJECT_ERROR_INVALID_PARAMETER);
                }
                break;
            }
            case AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_PLAY_STATUS:
            {
                bt_avrcp_get_play_status_response(bts2_app_data, avrcmsg->tlabel);
                break;
            }
            case AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_ELEMENT_ATTRIBUTES:
            {
                bt_avrcp_get_element_attributes_response(bts2_app_data, avrcmsg->tlabel);
                break;
            }
            default:
                bt_avrcp_reject_response(bts2_app_data, avrcmsg->tlabel, pdu_id, AVRCP_REJECT_ERROR_INVALID_COMMAND);
            }
        }
    }

    if (avrcmsg->data != NULL)
    {
        bfree(avrcmsg->data);
    }
}

static void bt_avrcp_hdl_pass_through_cmd_cfm(bts2_app_stru *bts2_app_data)
{
    BTS2S_AVRCP_PASS_THROUGH_CMD_CFM *avrcmsg;
    avrcmsg = (BTS2S_AVRCP_PASS_THROUGH_CMD_CFM *)bts2_app_data->recv_msg;

    INFO_TRACE("<< avrcp pass through cmd cfm,c_type = %x\n", avrcmsg->c_type);
    if (avrcmsg->c_type == AVRCP_CR_ACPT && !(avrcmsg->data[1] & 0x80))
    {
        U8 statePop = 0x80 + avrcmsg->data[1];//0x80:pop
        bt_avrcp_pop(bts2_app_data, statePop);
    }

#ifndef RT_USING_UTEST
    if (avrcmsg->data != NULL)
    {
        bfree(avrcmsg->data);
    }
#endif
}

static void bt_avrcp_hdl_vendor_depend_cmd_cfm(bts2_app_stru *bts2_app_data)
{
    BTS2S_AVRCP_VENDOR_DEPEND_CMD_CFM *avrcmsg;
    avrcmsg = (BTS2S_AVRCP_VENDOR_DEPEND_CMD_CFM *)bts2_app_data->recv_msg;
    INFO_TRACE("<< avrcp vndor cmd cfm\n");

    if (avrcmsg->c_type == AVRCP_CR_INTERIM || avrcmsg->c_type == AVRCP_CR_CHANGED)
    {
        if (avrcmsg->prof_id == BT_UUID_AVRCP_CT)
        {
            U8 op_code = avrcmsg->data[0];
            U8 pdu_id = avrcmsg->data[4];

            U8 company_id[4];
            bmemcpy(&company_id, avrcmsg->data, sizeof(U32));

            if (bmemcmp(VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, company_id, 4) != 0)
            {
                // TODO: error not bluetooth sig vendor dependent command
                if (avrcmsg->data != NULL)
                {
                    bfree(avrcmsg->data);
                }
                return;
            }

            U16 paramter_length;
            bmemcpy(&paramter_length, avrcmsg->data + 6, sizeof(U16));

            if (pdu_id == AVRCP_VENDOR_DEPENDENT_PDU_ID_REGISTER_NOTIFICATION)
            {
                U8 event_id = avrcmsg->data[8];

                INFO_TRACE("<< avrcp vndor cmd cfm event_id %x\n", event_id);

                switch (event_id)
                {
                case AVRCP_VENDOR_DEPENDENT_EVENT_VOLUME_CHANGED:
                {
                    U8 volume = avrcmsg->data[9];
                    // TODO: remote update volume, set our

                    INFO_TRACE("<< VOLUME_CHANGED  volume%x\n", volume);

#ifdef AUDIO_USING_MANAGER
                    uint8_t relative_volume = a2dp_set_speaker_volume(volume);
#if defined(CFG_AVRCP)
                    bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_ABSOLUTE_VOLUME, &relative_volume, sizeof(uint8_t));
#endif
#endif

                    if (avrcmsg->c_type == AVRCP_CR_CHANGED)
                    {
                        bt_avrcp_volume_register_request(bts2_app_data);

                        if (!bts2_app_data->avrcp_inst.avrcp_vol_time_handle)
                        {
                            bts2_app_data->avrcp_inst.avrcp_vol_time_handle = rt_timer_create("avrcp_vol", bt_avrcp_vol_timeout_handler, (void *)bts2_app_data,
                                    rt_tick_from_millisecond(500), RT_TIMER_FLAG_SOFT_TIMER);
                        }
                        else
                        {
                            rt_timer_stop(bts2_app_data->avrcp_inst.avrcp_vol_time_handle);
                        }
                        rt_timer_start(bts2_app_data->avrcp_inst.avrcp_vol_time_handle);
                    }
                    else if (avrcmsg->c_type == AVRCP_CR_INTERIM)
                    {
                        if (bts2_app_data->avrcp_inst.avrcp_vol_time_handle)
                        {
                            rt_timer_stop(bts2_app_data->avrcp_inst.avrcp_vol_time_handle);
                            bts2_app_data->avrcp_inst.avrcp_vol_time_handle = NULL;
                        }
                    }
                    break;
                }
                case AVRCP_VENDOR_DEPENDENT_EVENT_PLAYBACK_STATUS_CHANGED:
                {
                    U8 play_status = avrcmsg->data[9];
                    // TODO: remote update play status

                    // 0x01 playing, 0x02 paused

                    if (avrcmsg->c_type == AVRCP_CR_CHANGED)
                    {
                        bt_avrcp_playback_register_request(bts2_app_data);
#if defined(CFG_AVRCP)
                        //solution 0x00:playing ;0x01:paused
                        uint8_t play_status_notify = play_status - 1;
                        bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_PLAY_STATUS, &play_status_notify, sizeof(uint8_t));
#endif
                    }

                    bts2_app_stru *bts2_app_data = bts2g_app_p;
                    bts2_app_data->avrcp_inst.playback_status = play_status;
#ifdef CFG_AV_SRC
                    if (bt_av_get_receive_a2dp_start())
                    {
                        bt_av_set_can_play();
                    }
#endif
                    INFO_TRACE("<<PLAYBACK_STATUS_CHANGED  play_status%x\n", play_status);

                    break;
                }
                case AVRCP_VENDOR_DEPENDENT_EVENT_PLAYBACK_POS_CHANGED:
                {
                    U32 play_pos;
                    play_pos = ((avrcmsg->data[9] << 24) | (avrcmsg->data[10] << 16) | (avrcmsg->data[11] << 8) | (avrcmsg->data[12]));

                    if (avrcmsg->c_type == AVRCP_CR_CHANGED)
                    {
                        bt_avrcp_playback_pos_register_request(bts2_app_data);
                    }
                    if (0xffffffff == play_pos)
                    {
                        break;
                    }
#if defined(CFG_AVRCP)
                    bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_SONG_PROGREAS_STATUS, &play_pos, sizeof(U32));
#endif

                    break;
                }
                case AVRCP_VENDOR_DEPENDENT_EVENT_TRACK_CHANGED:
                {
                    U8 identifier[8];
                    U8 identifier_len = 8;

                    bmemcpy(identifier, &avrcmsg->data[9], identifier_len);

                    U8 identifier_result = 2;
                    U8 value;
                    for (int i = 0; i < identifier_len; i++)
                    {
                        value = *(identifier + i);
                        if (value == 0x00)
                        {
                            identifier_result = 0;
                        }
                        else if (value == 0xFF)
                        {
                            identifier_result = 1;
                        }
                        else
                        {
                            identifier_result = 2;
                            break;
                        }
                    }

                    INFO_TRACE("<<track_CHANGED identifier_result %x track_id_old %x  track_id_new%x\n", identifier_result, mp3_detail_info.track_id, value);

                    if (identifier_result != 1 && avrcmsg->c_type == AVRCP_CR_INTERIM)
                    {
                        // playing, should get element attributes
                        memset(&mp3_detail_info, 0x00, sizeof(bt_avrcp_mp3_detail_t));
                        mp3_detail_info.track_id = value;
                        mp3_detail_info.attri_req = AVRCP_MEDIA_ATTRIBUTES_GENRE;
                        bt_avrcp_get_element_attributes_request(bts2_app_data, AVRCP_MEDIA_ATTRIBUTES_GENRE);
                    }
                    else if (identifier_result == 1)
                    {
                        // not playing
                    }

                    if (avrcmsg->c_type == AVRCP_CR_CHANGED)
                    {
                        bt_avrcp_track_register_request(bts2_app_data);
                    }
                    break;
                }
                }
            }
        }
    }
    else if (avrcmsg->c_type == AVRCP_CR_STABLE)
    {
        U8 op_code = avrcmsg->data[0];
        U8 pdu_id = avrcmsg->data[4];

        U8 company_id[4];
        bmemcpy(&company_id, avrcmsg->data, sizeof(U32));

        if (bmemcmp(VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, company_id, 4) != 0)
        {
            // TODO: error not bluetooth sig vendor dependent command
            if (avrcmsg->data != NULL)
            {
                bfree(avrcmsg->data);
            }
            return;
        }

        if (pdu_id == AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_CAPABILITIES)
        {
            bt_avrcp_get_capabilities_confirm(bts2_app_data, avrcmsg);
        }
        else if (pdu_id == AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_ELEMENT_ATTRIBUTES)
        {
            bt_avrcp_get_element_attributes_confirm(bts2_app_data, avrcmsg);
        }
        else if (pdu_id == AVRCP_VENDOR_DEPENDENT_PDU_ID_GET_PLAY_STATUS)
        {
            //todo
            bt_avrcp_get_play_status_confirm(bts2_app_data, avrcmsg);
        }

    }
    else if (avrcmsg->c_type == AVRCP_CR_ACPT)
    {
        U8 op_code = avrcmsg->data[0];
        U8 pdu_id = avrcmsg->data[4];

        U8 company_id[4];
        bmemcpy(&company_id, avrcmsg->data, sizeof(U32));

        if (bmemcmp(VENDOR_DEPENDENT_BLUETOOTH_SIG_ID, company_id, 4) != 0)
        {
            // TODO: error not bluetooth sig vendor dependent command
            if (avrcmsg->data != NULL)
            {
                bfree(avrcmsg->data);
            }
            return;
        }

        if (pdu_id == AVRCP_VENDOR_DEPENDENT_PDU_ID_SET_ABSOLUTE_VOLUME)
        {
            INFO_TRACE("set absolute volume success\n");
            bts2_app_data->avrcp_inst.abs_volume_pending = 0;
        }
    }

    if (avrcmsg->data != NULL)
    {
        bfree(avrcmsg->data);
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
static void bt_avrcp_hdl_pass_through_cmd_ind(bts2_app_stru *bts2_app_data)
{
    BTS2S_AVRCP_PASS_THROUGH_CMD_IND *avrcmsg;

    avrcmsg = (BTS2S_AVRCP_PASS_THROUGH_CMD_IND *)bts2_app_data->recv_msg;

    INFO_TRACE("<< recive avrcp subunit info command indication\n");

    if (avrcmsg->c_type == AVRCP_CR_CTRL)
    {
        if (avrcmsg->prof_id == BT_UUID_AVRCP_CT)
        {
            U8 vld_cmd = TRUE;
            switch (avrcmsg->data[1] & 0x7f)
            {
            case PLY_PASS_THROUGH:
            {
                if (avrcmsg->data[1] & 0x80)
                {
                    USER_TRACE("PLAY OFF\n");
                    //input_ev(inst->input, EV_KEY, KEY_PLY, 0);

                    bts2s_av_inst_data *inst = bt_av_get_inst_data();
                    int con_idx;

                    con_idx = bt_avsrc_get_plyback_conn(inst);


                    if (con_idx == - 1)
                        break;



#if defined(AUDIO_USING_MANAGER) && defined(SDK_AVRCP_USE_PASS_THROUGH)
                    if (inst->con[con_idx].st == avconned_open && bt_avsrc_get_start_flag() && (inst->src_data.input_cb != NULL))
                        inst->src_data.input_cb(as_callback_cmd_play_resume, NULL, 0);
#elif defined(CFG_AVRCP)
                    //solution 0x00:playing ;0x01:paused
                    uint8_t play_status_notify = 0;
                    bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_PLAY_STATUS, &play_status_notify, sizeof(uint8_t));
#endif
                }
                else
                {
                    USER_TRACE("PLAY ON\n");
                    //input_ev(inst->input, EV_KEY, KEY_PLY, 1);
                    /*  avrcp_target(BT_AVRCP_PLY, NULL); */
                }
                break;
            }
            case STOP_PASS_THROUGH:
            {
                if (avrcmsg->data[1] & 0x80)
                {
                    USER_TRACE("STOP OFF\n");
                    // input_ev(inst->input, EV_KEY, KEY_STOP, 0);
#if defined(AUDIO_USING_MANAGER) && defined(SDK_AVRCP_USE_PASS_THROUGH)
                    if (inst->con[con_idx].st == avconned_streaming && (inst->src_data.input_cb != NULL))
                        inst->src_data.input_cb(as_callback_cmd_play_pause, NULL, 0);
#elif defined(CFG_AVRCP)
                    //solution 0x00:playing ;0x01:paused
                    uint8_t play_status_notify = 1;
                    bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_PLAY_STATUS, &play_status_notify, sizeof(uint8_t));
#endif
                }
                else
                {
                    USER_TRACE("STOP ON\n");
                    //input_ev(inst->input, EV_KEY, KEY_STOP, 1);
                    /* avrcp_target(BT_AVRCP_STOP, NULL); */
                }
                break;
            }
            case PAUSE_PASS_THROUGH:
            {
                if (avrcmsg->data[1] & 0x80)
                {
                    USER_TRACE("PAUSE OFF\n");
                    //input_ev(inst->input, EV_KEY, KEY_PLYPAUSE, 0);

                    bts2s_av_inst_data *inst = bt_av_get_inst_data();
                    int con_idx;

                    con_idx = bt_avsrc_get_plyback_conn(inst);


                    if (con_idx == - 1)
                        break;

#if defined(AUDIO_USING_MANAGER) && defined(SDK_AVRCP_USE_PASS_THROUGH)
                    if (inst->con[con_idx].st == avconned_streaming && (inst->src_data.input_cb != NULL))
                        inst->src_data.input_cb(as_callback_cmd_play_pause, NULL, 0);
#elif defined(CFG_AVRCP)
                    //solution 0x00:playing ;0x01:paused
                    uint8_t play_status_notify = 1;
                    bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_PLAY_STATUS, &play_status_notify, sizeof(uint8_t));
#endif
                }
                else
                {
                    USER_TRACE("PAUSE ON\n");
                    //input_ev(inst->input, EV_KEY, KEY_PLYPAUSE, 1);
                    /*avrcp_target(BT_AVRCP_PAUSE, NULL); */
                }
                break;
            }
            case FORWARD_PASS_THROUGH:
            {
                if (avrcmsg->data[1] & 0x80)
                {
                    USER_TRACE("FORWARD OFF\n");
                    // input_ev(inst->input, EV_KEY, KEY_NEXTSONG, 0);
                    /*  avrcp_target(BT_AVRCP_FORWARD, NULL);*/
                    bts2s_av_inst_data *inst = bt_av_get_inst_data();
                    int con_idx;

                    con_idx = bt_avsrc_get_plyback_conn(inst);


                    if (con_idx == - 1)
                        break;

#if defined(AUDIO_USING_MANAGER) && defined(SDK_AVRCP_USE_PASS_THROUGH)
                    if (inst->con[con_idx].st == avconned_streaming && (inst->src_data.input_cb != NULL))
                        inst->src_data.input_cb(as_callback_cmd_play_to_next, NULL, 0);
#elif defined(CFG_AVRCP)
                    //solution 0x00:previous ;0x01:next
                    uint8_t track_change = 1;
                    bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_TRACK_CHANGE_STATUS, &track_change, sizeof(uint8_t));
#endif
                }
                else
                {
                    USER_TRACE("FORWARD ON\n");
                    //input_ev(inst->input, EV_KEY, KEY_NEXTSONG, 1);
                }

                break;
            }
            case BACKWARD_PASS_THROUGH:
            {

                if (avrcmsg->data[1] & 0x80)
                {
                    USER_TRACE("BACKWARD OFF\n");
                    //input_ev(inst->input, EV_KEY, KEY_PRESONG, 0);

                    bts2s_av_inst_data *inst = bt_av_get_inst_data();
                    int con_idx;

                    con_idx = bt_avsrc_get_plyback_conn(inst);


                    if (con_idx == - 1)
                        break;

#if defined(AUDIO_USING_MANAGER) && defined(SDK_AVRCP_USE_PASS_THROUGH)
                    if (inst->con[con_idx].st == avconned_streaming && (inst->src_data.input_cb != NULL))
                        inst->src_data.input_cb(as_callback_cmd_play_to_prev, NULL, 0);
#elif defined(CFG_AVRCP)
                    //solution 0x00:previous ;0x01:next
                    uint8_t track_change = 0;
                    bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_TRACK_CHANGE_STATUS, &track_change, sizeof(uint8_t));

#endif
                }
                else
                {
                    USER_TRACE("BACKWARD ON\n");
                    //input_ev(inst->input, EV_KEY, KEY_PRESONG, 1);
                    /* avrcp_target(BT_AVRCP_BACKWARD, NULL); */
                }
                break;
            }

            default:
            {
                /*unknown key */
                INFO_TRACE(" -- unknown key commond:0x%x\n", (avrcmsg->data[1] & 0x7f));
                vld_cmd = TRUE;
                break;
            }
            }

            /*send response */
            if (vld_cmd == TRUE)
            {
                avrcp_cmd_data_rsp(bts2_app_data->phdl,
                                   avrcmsg->tlabel,
                                   avrcmsg->prof_id,
                                   AVRCP_CR_ACPT,
                                   avrcmsg->subunit_type,
                                   avrcmsg->subunit_id,
                                   avrcmsg->data_len,
                                   avrcmsg->data);
            }
        }
        else
        {
            avrcp_cmd_data_rsp(bts2_app_data->phdl,
                               avrcmsg->tlabel,
                               avrcmsg->prof_id,
                               AVRCP_CR_INVLD_PID,
                               avrcmsg->subunit_type,
                               avrcmsg->subunit_id,
                               0,
                               NULL);
        }
    }

#ifndef RT_USING_UTEST
    if (avrcmsg->data != NULL)
    {
        bfree(avrcmsg->data);
    }
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
static void bt_avrcp_hdl_conn_cfm(bts2_app_stru *bts2_app_data)
{
    BTS2S_AVRCP_CONN_CFM *msg;
    msg = (BTS2S_AVRCP_CONN_CFM *)bts2_app_data->recv_msg;
    if (msg->res == BTS2_SUCC)
    {
        USER_TRACE("<< confirmation connect successed \n");
        bts2_app_data->avrcp_inst.st = avrcp_conned;
#if defined(CFG_AVRCP)
        bt_notify_profile_state_info_t profile_state;
        bt_addr_convert(&msg->bd, profile_state.mac.addr);
        profile_state.profile_type = BT_NOTIFY_AVRCP;
        profile_state.res = BTS2_SUCC;
        bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_PROFILE_CONNECTED,
                                     &profile_state, sizeof(bt_notify_profile_state_info_t));
        INFO_TRACE("URC avrcp conn,cfm\n");
#endif

        bt_cm_conn_info_t *bonded_dev = bt_cm_find_bonded_dev_by_addr(profile_state.mac.addr);
        if (bonded_dev->role == BT_CM_SLAVE)
        {
            bt_avrcp_get_capabilities_request(bts2_app_data);
            bt_avrcp_get_play_status_request(bts2_app_data);
            bts2_app_data->avrcp_inst.abs_volume_pending = 0;
        }

    }
    else
    {
        USER_TRACE("<< confirmation connect failed \n");
#if defined(CFG_AVRCP)
        bt_notify_profile_state_info_t profile_state;
        bt_addr_convert(&msg->bd, profile_state.mac.addr);
        profile_state.profile_type = BT_NOTIFY_AVRCP;
        profile_state.res = msg->res;
        bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED,
                                     &profile_state, sizeof(bt_notify_profile_state_info_t));
#endif
    }

}

void bt_avrcp_close_boundary_condition(bts2_app_stru *bts2_app_data)
{
    U16 *msg_type;
    msg_type = (U16 *)bts2_app_data->recv_msg;

    switch (*msg_type)
    {
    case BTS2MU_AVRCP_DISB_CFM:
    {
        bts2_app_data->avrcp_inst.tgRegStatus = 0;
        bts2_app_data->avrcp_inst.tgRegStatus1 = 0;
#if defined(CFG_AVRCP)
        bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_CLOSE_COMPLETE, NULL, 0);
        INFO_TRACE("<< URC av had been disabled \n");
#endif
        break;
    }

    default:
        INFO_TRACE("<< URC av had been disabled,discard msg %x \n", *msg_type);
        break;
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
void bt_avrcp_msg_handler(bts2_app_stru *bts2_app_data)
{
    U16 *msg_type;
    msg_type = (U16 *)bts2_app_data->recv_msg;

    if (0x00 == bts2s_avrcp_openFlag)
    {
        bt_avrcp_close_boundary_condition(bts2_app_data);
        return;
    }

    switch (*msg_type)
    {
    case BTS2MU_AVRCP_ERROR_IND:
    {
        break;
    }
    case BTS2MU_AVRCP_ENB_CFM:
    {
        BTS2S_AVRCP_ENB_CFM *msg;
        msg = (BTS2S_AVRCP_ENB_CFM *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE(">> AVCTP enabled\n");
#if defined(CFG_AVRCP)
            bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_OPEN_COMPLETE, NULL, 0);
            INFO_TRACE(">> URC AVRCP open\n");
#endif
        }
        break;
    }
    case BTS2MU_AVRCP_CONN_CFM:
    {
        bt_avrcp_hdl_conn_cfm(bts2_app_data);
        break;
    }
    case BTS2MU_AVRCP_PASS_THROUGH_CMD_IND:
    {
        bt_avrcp_hdl_pass_through_cmd_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AVRCP_CONN_IND:
    {
        BTS2S_AVRCP_CONN_IND *msg;
        msg = (BTS2S_AVRCP_CONN_IND *)bts2_app_data->recv_msg;
        bts2_app_data->avrcp_inst.rmt_bd.lap = msg->bd.lap;
        bts2_app_data->avrcp_inst.rmt_bd.nap = msg->bd.nap;
        bts2_app_data->avrcp_inst.rmt_bd.uap = msg->bd.uap;

        bts2_app_data->avrcp_inst.st = avrcp_conned;
        USER_TRACE("<< avrcp indicate to connect with remote device\n");
#if defined(CFG_AVRCP)
        bt_notify_profile_state_info_t profile_state;
        bt_addr_convert(&msg->bd, profile_state.mac.addr);
        profile_state.profile_type = BT_NOTIFY_AVRCP;
        profile_state.res = BTS2_SUCC;
        bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_PROFILE_CONNECTED,
                                     &profile_state, sizeof(bt_notify_profile_state_info_t));
        INFO_TRACE("URC avrcp conn,ind\n");
#endif
        bt_cm_conn_info_t *bonded_dev = bt_cm_find_bonded_dev_by_addr(profile_state.mac.addr);
        if (bonded_dev->role == BT_CM_SLAVE)
        {
            bt_avrcp_get_capabilities_request(bts2_app_data);
            bt_avrcp_get_play_status_request(bts2_app_data);
            bts2_app_data->avrcp_inst.abs_volume_pending = 0;
        }
        break;
    }
    case BTS2MU_AVRCP_DISC_IND:
    {
        BTS2S_AVRCP_DISC_IND *msg;
        bts2_app_data->avrcp_inst.st = avrcp_idle;
        msg = (BTS2S_AVRCP_DISC_IND *)bts2_app_data->recv_msg;
        bts2_app_data->avrcp_inst.rmt_bd.lap = msg->bd.lap;
        bts2_app_data->avrcp_inst.rmt_bd.nap = msg->bd.nap;
        bts2_app_data->avrcp_inst.rmt_bd.uap = msg->bd.uap;

        bts2_app_data->avrcp_inst.tgRegStatus = 0;
        bts2_app_data->avrcp_inst.tgRegStatus1 = 0;
        bts2_app_data->avrcp_inst.abs_volume_pending = 0;
        USER_TRACE("bd : %4lx %4x %4x\n", msg->bd.lap, msg->bd.nap, msg->bd.uap);
        USER_TRACE("<< avrcp indicate to disconnect with remote device\n");
#if defined(CFG_AVRCP)
        bt_notify_profile_state_info_t profile_state;
        bt_addr_convert(&msg->bd, profile_state.mac.addr);
        profile_state.profile_type = BT_NOTIFY_AVRCP;
        profile_state.res = msg->res;
        bt_interface_bt_event_notify(BT_NOTIFY_AVRCP, BT_NOTIFY_AVRCP_PROFILE_DISCONNECTED,
                                     &profile_state, sizeof(bt_notify_profile_state_info_t));
        USER_TRACE("<< urc AVRCP disc\n");
#endif
        break;
    }
    case BTS2MU_AVRCP_UNIT_INFO_CMD_IND:
    {
        bt_avrcp_hdl_unitinfo_cmd_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AVRCP_SUBUNIT_INFO_CMD_IND:
    {
        bt_avrcp_hdl_subunitinfo_cmd_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AVRCP_VENDOR_DEPEND_CMD_IND:
    {
        //INFO_TRACE("BTS2MU_AVRCP_VENDOR_DEPEND_CMD_IND\n");
        bt_avrcp_hdl_vendor_depend_cmd_ind(bts2_app_data);
        break;
    }
    case BTS2MU_AVRCP_DISB_CFM:
    {
        bts2_app_data->avrcp_inst.tgRegStatus = 0;
        bts2_app_data->avrcp_inst.tgRegStatus1 = 0;
        INFO_TRACE("BTS2MU_AVRCP_DISB_CFM\n");
        break;
    }
    case BTS2MU_AVRCP_UNIT_INFO_CMD_CFM:
    {
        INFO_TRACE("BTS2MU_AVRCP_UNIT_INFO_CMD_CFM\n");
        break;
    }
    case BTS2MU_AVRCP_SUBUNIT_INFO_CMD_CFM:
    {
        INFO_TRACE("BTS2MU_AVRCP_SUBUNIT_INFO_CMD_CFM\n");
        break;
    }
    case BTS2MU_AVRCP_VENDOR_DEPEND_CMD_CFM:
    {
        //INFO_TRACE("BTS2MU_AVRCP_VENDOR_DEPEND_CMD_CFM\n");
        bt_avrcp_hdl_vendor_depend_cmd_cfm(bts2_app_data);
        break;
    }
    case BTS2MU_AVRCP_PASS_THROUGH_CMD_CFM:
    {
        INFO_TRACE("BTS2MU_AVRCP_PASS_THROUGH_CMD_CFM\n");
        bt_avrcp_hdl_pass_through_cmd_cfm(bts2_app_data);
        break;
    }
    default:
        break;
    }
}

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
