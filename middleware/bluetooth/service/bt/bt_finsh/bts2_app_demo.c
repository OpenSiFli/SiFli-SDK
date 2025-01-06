/**
  ******************************************************************************
  * @file   bts2_app_demo.c
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
#include "bt_connection_manager.h"
#include "bf0_ble_common.h"

#include "log.h"

bts2_app_stru *bts2g_app_p = NULL;

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
void init_inst_data(bts2_app_stru *bts2_app_data)
{
    U8 i = 0;

    bts2_app_data->recv_msg = NULL;
    bts2_app_data->bd_list_num = 0;
    bts2_app_data->inquiry_list_num = 0;
    bts2_app_data->inquiry_flag = FALSE;
    bts2_app_data->dev_idx = 0;

    for (i = 0; i < MAX_DISCOV_RESS; i++)
    {
        bts2_app_data->bd_list[i].lap = CFG_BD_LAP;
        bts2_app_data->inquiry_list[i].lap = CFG_BD_LAP;
        bts2_app_data->bd_list[i].nap = CFG_BD_NAP;
        bts2_app_data->inquiry_list[i].nap = CFG_BD_NAP;
        bts2_app_data->bd_list[i].uap = CFG_BD_UAP;
        bts2_app_data->inquiry_list[i].uap = CFG_BD_UAP;
    }
    //20220727
    bts2_app_data->last_conn_bd.lap = CFG_BD_LAP;
    bts2_app_data->last_conn_bd.nap = CFG_BD_NAP;
    bts2_app_data->last_conn_bd.uap = CFG_BD_UAP;

    bts2_app_data->phdl = bts2_task_get_app_task_id();
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Initialize && Enable BTS2 profile service.
 *
 * INPUT:
 *      bts2_app_data: application struct.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Here initialize BTS2 profile program.
 *
 *----------------------------------------------------------------------------*/
void bt_init_profile(bts2_app_stru *bts2_app_data)
{
    /*SPP */
#ifdef CFG_SPP_CLT
    bt_spp_clt_init(bts2_app_data);
#endif
    /*HFP HF */
#ifdef CFG_HFP_HF
    bt_hfp_hf_init(bts2_app_data);
    bt_hfp_hf_start_enb(bts2_app_data);
#endif

    /*for sdk config do not change BT_USING_AG*/
#ifdef CFG_HFP_AG
    bt_hfp_ag_app_init(bts2_app_data);
    bt_hfp_start_profile_service(bts2_app_data);
#endif

#ifdef BT_FINSH_PAN
    bt_pan_init(bts2_app_data);
    bt_pan_reg(bts2_app_data);
#endif
    /*A2DP SNK */
#if (defined(CFG_AV_SNK) || defined(CFG_AV_SRC))
    bt_av_init(bts2_app_data);
#endif

#ifdef CFG_AVRCP
    bt_avrcp_int(bts2_app_data);
#endif

#ifdef CFG_HID
    bt_hid_init(bts2_app_data);
#endif

#ifdef CFG_PBAP_CLT
    bt_pbap_clt_init(bts2_app_data);
#endif
#ifdef CFG_SPP_SRV
    bt_spp_srv_init(bts2_app_data);
    bt_spp_srv_start_enb(bts2_app_data);
#endif
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      void **pp:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void app_fn_init(void **pp)
{
    bts2_app_stru *bts2_app_data;

    /*app */
    *pp = (void *)bmalloc(sizeof(bts2_app_stru));
    RT_ASSERT(*pp);
    memset(*pp, 0, sizeof(bts2_app_stru));
    bts2_app_data = (bts2_app_stru *)*pp;
    bts2g_app_p = bts2_app_data;
#ifdef BTS2_APP_MENU
    bts2_app_data->menu_id = menu_start;
    bt_disply_menu(bts2_app_data);
    bts2_app_data->menu_id = menu_main;
    bt_disply_menu(bts2_app_data);
#endif
    bts2_app_data->input_str_len = 0;
    bts2_app_data->pin_code_len = strlen(CFG_PIN_CODE);
    strcpy((char *)bts2_app_data->pin_code, CFG_PIN_CODE);
    init_inst_data(bts2_app_data);
#ifdef BSP_BT_CONNECTION_MANAGER
    init_bt_cm();
#endif // BSP_BT_CONNECTION_MANAGER

    gap_rd_local_bd_req(bts2_app_data->phdl);
    /*Send read local BD request,if receive confirm message,then BTS2 initialization is OK*/
    gap_app_init_req(bts2_app_data->phdl);


    //bts2_timer_ev_add(KEYB_CHK_TIMEOUT, key_msg_svc, 0, NULL);
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      void **pp:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void app_fn_rel(void **pp)
{
    U16 msg_type;
    void *msg_data = NULL;
#if defined(CFG_SPP_CLT) || defined(CFG_SPP_SRV)
    U8  i;
#endif
    bts2_app_stru *bts2_app_data;

    bts2_app_data = (bts2_app_stru *)(*pp);
    if (!bts2_app_data)
    {
        DBG_OUT()
        return;
    }

    /*get a msg from the demo application task*/
    while (bts2_msg_get(bts2_task_get_app_task_id(), &msg_type, &msg_data))
    {
        switch (msg_type)
        {
#ifdef CFG_SPP_CLT
        case BTS2M_SPP_CLT:
        {
            U16 *msg_type;

            msg_type = (U16 *)msg_data;
            switch (*msg_type)
            {

            case BTS2MU_SPP_CLT_DATA_IND:
            {
                BTS2S_SPP_CLT_DATA_IND *msg;

                msg = (BTS2S_SPP_CLT_DATA_IND *)msg_data;
                bfree(msg->payload);
                break;
            }
            }
            bfree(msg_data);

            /*SPP */
            for (i = 0; i < SPP_CLT_MAX_CONN_NUM; i++)
            {
                if (bts2_app_data->spp_inst[i].timer_flag)
                {
                    bts2_timer_ev_cancel(bts2_app_data->spp_inst[i].time_id, NULL, NULL);
                }

                if (bts2_app_data->spp_inst[i].cur_file_hdl != NULL)
                {
                    fclose(bts2_app_data->spp_inst[i].cur_file_hdl);
                }

                if (bts2_app_data->spp_inst[i].wr_file_hdl != NULL)
                {
                    fclose(bts2_app_data->spp_inst[i].wr_file_hdl);
                }
            }
            break;
        }
#endif
#ifdef CFG_SPP_SRV
        case BTS2M_SPP_SRV:
        {
            U16 *msg_type;

            msg_type = (U16 *)msg_data;
            switch (*msg_type)
            {

            case BTS2MU_SPP_SRV_DATA_IND:
            {
                BTS2S_SPP_SRV_DATA_IND *msg;

                msg = (BTS2S_SPP_SRV_DATA_IND *)msg_data;
                bfree(msg->payload);
                break;
            }

            }
            bfree(msg_data);

            /*SPP */
            for (i = 0; i < CFG_MAX_ACL_CONN_NUM; i++)
            {
                bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
                bts2_spp_service_list *spp_service_list = NULL;

                bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[i];

                if (bts2_spp_srv_inst->service_list != 0)
                {
                    bts2_spp_service_list *spp_service_list = bts2_spp_srv_inst->spp_service_list;
                    while (spp_service_list)
                    {
                        if (spp_service_list->timer_flag)
                        {
                            bts2_timer_ev_cancel(spp_service_list->time_id, NULL, NULL);
                        }

#if RT_USING_DFS
                        if (spp_service_list->cur_file_hdl != NULL)
                        {
                            fclose(spp_service_list->cur_file_hdl);
                        }

                        if (spp_service_list->wr_file_hdl != NULL)
                        {
                            fclose(spp_service_list->wr_file_hdl);
                        }
#endif
                        spp_service_list = (bts2_spp_service_list *)spp_service_list->next_struct;
                    }
                }
            }
            break;
        }
#endif

#ifdef CFG_AV
        case  BTS2M_AV:
        {
#if (defined(CFG_AV_SNK) || defined(CFG_AV_SRC))
            bt_av_rel();
#endif
            break;
        }
#endif
            /* PBAP  client*/
#ifdef CFG_PBAP_CLT
        case BTS2M_PBAP_CLT:
        {
            bt_pbap_clt_rel(bts2_app_data, msg_data);
            break;
        }
#endif
        }
    }

#ifdef CFG_PBAP_CLT
    bt_pbap_clt_free_inst();
#endif
    bfree(bts2_app_data);
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Demo application handler function, all message sent to
 *      app will recived here
 *
 * INPUT:
 *      void **pp:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/


void app_fn_hdl_ext(void **pp)
{
    U16 msg_cls;
    void *msg;

    bts2_msg_get(bts2_task_get_app_task_id(), &msg_cls, &msg);
    bt_event_publish(msg_cls, *((U16 *)msg), msg);
    bfree(msg);
}


void app_event_init()
{


}


int app_event_hdl(U16 type, U16 event_id, uint8_t *msg, uint32_t context)
{
    bts2_app_stru *bts2_app_data = (bts2_app_stru *)(*(uint32_t *)context);

    bts2_app_data->recv_msg = msg;

    switch (type)
    {
#ifdef CFG_SPP_CLT
    case BTS2M_SPP_CLT:
    {
        bt_spp_clt_msg_hdl(bts2_app_data);
        break;
    }
#endif
#ifdef CFG_SPP_SRV
    case BTS2M_SPP_SRV:
    {
        bt_spp_srv_msg_hdl(bts2_app_data);
        break;
    }
#endif
#ifdef CFG_HFP_HF
    case BTS2M_HFP_HF:
    {
        bt_hfp_hf_msg_hdl(bts2_app_data);
        break;
    }
#endif

#ifdef CFG_HFP_AG
    case BTS2M_HFP_AG:
    {
        bt_hfp_ag_msg_hdl(bts2_app_data);
        break;
    }
#endif
    case BTS2M_AV:
    {
#if (defined(CFG_AV_SNK) || defined(CFG_AV_SRC))
        bt_av_msg_handler(bts2_app_data);
#endif
        break;
    }
#ifdef  CFG_AVRCP
    case BTS2M_AVRCP:
    {
        bt_avrcp_msg_handler(bts2_app_data);
        break;
    }
#endif

#ifdef  CFG_HID
    case BTS2M_HID:
    {
        bt_hid_msg_handler(bts2_app_data);
        break;
    }
#endif

    case BTS2M_GAP :
    {
        bt_hdl_gap_msg(bts2_app_data);
        break;
    }

#ifdef CFG_BQB_SDC_SSA
    case BTS2M_SDP:
    {
        bt_hdl_sdp_msg(bts2_app_data);
        break;
    }
#endif

#ifdef BT_FINSH_PAN
    case BTS2M_PAN:
    {
        bt_hdl_pan_msg(bts2_app_data);
        break;
    }
#endif
#ifdef CFG_PBAP_CLT
    case BTS2M_PBAP_CLT:
    {
        bt_pbap_clt_hdl_msg(bts2_app_data);
        break;
    }
#endif

#ifdef CFG_BR_GATT_SRV
    case BTS2M_BR_GATT:
    {
        bt_bt_gatt_msg_hdl(bts2_app_data);
        break;
    }
#endif

#ifdef CFG_BT_DIS
    case BTS2M_BT_DIS:
    {
        bt_dis_app_msg_hdl(bts2_app_data);
        break;
    }
#endif

#ifdef CFG_BT_L2CAP_PROFILE
    case BTS2M_BT_L2CAP_PROFILE:
    {
        bt_l2cap_profile_app_msg_hdl(bts2_app_data);
        break;
    }

    case BTS2M_HCI_CMD:
    {
        bt_l2cap_profile_hci_msg_hdl(bts2_app_data);
        break;
    }
#endif

    case BTS2M_SC:
    {
        bt_hdl_sc_msg(bts2_app_data);
        break;
    }
#ifdef BTS2_APP_MENU
    case KEY_MSG:
    {
        U8 c;
        c = *((U8 *)bts2_app_data->recv_msg);
#if 0 // Not handle 'q' case
        /* it should release memory here. Because it will happen memory leak when function
           hdl_keyb_msg quit as a result of receiving character 'q'.
         */
        bfree(bts2_app_data->recv_msg);
#endif
        hdl_keyb_msg(c, bts2_app_data);
        break;
    }
#endif //BTS2_APP_MENU
    default:
    {
        INFO_TRACE("Unknown message class: %x\n", type);
        break;
    }
    }

    return 0;

}
BT_EVENT_REGISTER(app_event_hdl, &bts2g_app_p);




#if 0
void app_fn_hdl(void **pp)
{
    bts2_app_stru *bts2_app_data;
    U16 msg_cls;
    void *msg;
    /* get a msg from the demo task*/
    bts2_app_data = (bts2_app_stru *)(*pp);

    bts2_app_data->recv_msg = msg;

    switch (msg_cls)
    {
#ifdef CFG_SPP_CLT
    case BTS2M_SPP_CLT:
    {
        bt_spp_clt_msg_hdl(bts2_app_data);
        break;
    }
#endif
#ifdef CFG_SPP_SRV
    case BTS2M_SPP_SRV:
    {
        bt_spp_srv_msg_hdl(bts2_app_data);
        break;
    }
#endif
#ifdef CFG_HFP_HF
    case BTS2M_HFP_HF:
    {
        bt_hfp_hf_msg_hdl(bts2_app_data);
        break;
    }
#endif
    case BTS2M_AV:
    {
#ifdef CFG_AV_SNK
        bt_avsnk_msg_handler(bts2_app_data);
#endif
#ifdef CFG_AV_SRC
        bt_avsrc_msg_handler(bts2_app_data);
#endif
#ifdef CFG_VDP_SNK
        bt_vdpsnk_msg_handler(bts2_app_data);
#endif
#ifdef CFG_VDP_SRC
        bt_vdpsrc_msg_handler(bts2_app_data);
#endif
        break;
    }
#ifdef  CFG_AVRCP
    case BTS2M_AVRCP:
    {
        bt_avrcp_msg_handler(bts2_app_data);
        break;
    }
#endif

    case BTS2M_GAP :
    {
        bt_hdl_gap_msg(bts2_app_data);
        break;
    }

#ifdef CFG_BQB_SDC_SSA
    case BTS2M_SDP:
    {
        bt_hdl_sdp_msg(bts2_app_data);
        break;
    }
#endif

#ifdef BT_FINSH_PAN
    case BTS2M_PAN:
    {
        bt_hdl_pan_msg(bts2_app_data);
        break;
    }
#endif
#ifdef CFG_PBAP_CLT
    case BTS2M_PBAP_CLT:
    {
        bt_pbap_clt_hdl_msg(bts2_app_data);
        break;
    }
#endif

    case BTS2M_SC:
    {
        bt_hdl_sc_msg(bts2_app_data);
        break;
    }
    case KEY_MSG:
    {
        U8 c;
        c = *((U8 *)bts2_app_data->recv_msg);
        /* it should release memory here. Because it will happen memory leak when function
           hdl_keyb_msg quit as a result of receiving character 'q'.
         */
        bfree(bts2_app_data->recv_msg);
        hdl_keyb_msg(c, bts2_app_data);
        return;
    }
    default:
    {
        INFO_TRACE("Unknown message class: %x\n", msg_cls);
        break;
    }
    }
    bfree(bts2_app_data->recv_msg);
}

#endif


bts2_app_stru *getApp(void)
{
    return bts2g_app_p;
}


#ifdef BTS2_APP_MENU

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      U16 m:
 *      void *data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void key_msg_svc(void)
{
    while (1)
    {
#ifdef CFG_MS
        if (1)
        {
            int ch;
            void *key_stroke_data;
            ch = _getch();
            key_stroke_data = bmalloc(sizeof(unsigned char));
            *((unsigned char *)key_stroke_data) = (unsigned char)ch;
            bts2_msg_put(bts2_task_get_app_task_id(), KEY_MSG, key_stroke_data);
        }
#endif
    }
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 * INPUT:
 *      U8 c:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void hdl_keyb_msg(U8 c, bts2_app_stru *bts2_app_data)
{
    if ((c == RETURN_KEY) || (c == RETURN_ENTER))
    {
        USER_TRACE("\n");
        bts2_app_data->input_str[bts2_app_data->input_str_len] = '\0';
        bt_hdl_menu(bts2_app_data);
        bmemset(bts2_app_data->input_str, 0x00, bts2_app_data->input_str_len);
        bts2_app_data->input_str_len = 0;
        USER_TRACE("\n");
    }
    else if (c == BACKSPACE_KEY)
    {
        if (bts2_app_data->input_str_len > 0)
        {
            bts2_app_data->input_str_len -- ;
            bts2_app_data->input_str[bts2_app_data->input_str_len] = c;
            USER_TRACE("\b \b");
        }
    }
    else if ((c >= 32) && (c < 127))
    {
        bts2_app_data->input_str[bts2_app_data->input_str_len++ ] = c;
        USER_TRACE("%c", c);
    }
    else
    {
        if ((bts2_app_data->menu_id == menu_gen_5_1)
                || (bts2_app_data->menu_id == menu_sc_input)
                || (bts2_app_data->menu_id == menu_sc_yesno)
                || (bts2_app_data->menu_id == menu_sc_passkey_notifi)
                || (bts2_app_data->menu_id == menu_sc_oobdata)
           )
        {
            /*for chinese character input */
            bts2_app_data->input_str[bts2_app_data->input_str_len++ ] = c;
            USER_TRACE("%c", c);
        }
    }
}





int btskey(int argc, char **argv)
{

    if (argc > 1)
    {
        char *key_stroke_data;
        int i = 0;

        for (int j = 1; j < argc; j ++)
        {
            i = 0;

            while (argv[j][i])
            {
                key_stroke_data = bmalloc(1);
                *key_stroke_data = argv[j][i];
                bts2_msg_put(bts2_task_get_app_task_id(), KEY_MSG, key_stroke_data);
                i++;
            }
        }

        key_stroke_data = bmalloc(1);
        *key_stroke_data = '\n';
        bts2_msg_put(bts2_task_get_app_task_id(), KEY_MSG, key_stroke_data);
    }

    return 0;
}
MSH_CMD_EXPORT(btskey, Send a key to BTS2 application.);
#endif // BTS2_APP_MENU

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
