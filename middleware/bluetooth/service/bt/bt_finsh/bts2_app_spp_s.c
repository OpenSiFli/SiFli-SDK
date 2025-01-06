/**
  ******************************************************************************
  * @file   bts2_app_spp_s.c
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
#include "bt_connection_manager.h"

#ifdef RT_USING_BT
    #include "bt_rt_device.h"
#endif

#define LOG_TAG         "btapp_spps"
#include "log.h"



#ifdef CFG_SPP_SRV




#define SPP_DEFAULT_NAME    "SPP_TEST"

static U8 write_into_file = 0;

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/


#ifdef BSP_USING_PC_SIMULATOR
#define bt_spp_srv_test_enable() FALSE
#else
__WEAK BOOL bt_spp_srv_test_enable(void)
{
    return FALSE;
}
#endif



//!zhengyu:Customers need to implement this weak function to add custom uuid
__WEAK void bt_spp_srv_add_uuid_list(void)
{
    // zhengyu:Add a custom 16-bit, 32-bit or 128-bit UUID
    // U8 uuid_1[] =
    // {
    //     // 0xC9, 0x77, 0x0A, 0x18, 0x4C, 0x3D, 0x45, 0x3A, 0x8A, 0xAF, 0xD7, 0xEC, 0x7B, 0xBD, 0x27, 0x85
    //     0x30, 0x01
    // };

    // U8 uuid_2[] =
    // {
    //     0x30, 0x02
    // };

    // U8 uuid_3[] =
    // {
    //     0x30, 0x03
    // };

    // spp_add_uuid_list_node(uuid_1, sizeof(uuid_1), "aaaa");
    // spp_add_uuid_list_node(uuid_2, sizeof(uuid_2), "bbbb");
    // spp_add_uuid_list_node(uuid_3, sizeof(uuid_3), "cccc");
}



void bt_spp_srv_init(bts2_app_stru *bts2_app_data)
{
    U8 i = 0;

    bts2_app_data->select_device_id = 0;
    bts2_app_data->select_srv_chnl = 0;
    bts2_app_data->spp_srv_conn_nums = 0;
    bts2_app_data->spp_srv_inst_ptr = &bts2_app_data->spp_srv_inst[0];

    for (i = 0; i < CFG_MAX_ACL_CONN_NUM; i++)
    {
        bts2_app_data->spp_srv_inst[i].device_id = 0xff;
        bd_set_empty(&bts2_app_data->spp_srv_inst[i].bd_addr);
        bts2_app_data->spp_srv_inst[i].cur_link_mode = 0;
        bts2_app_data->spp_srv_inst[i].exit_sniff_pending = FALSE;
        bts2_app_data->spp_srv_inst[i].cod = 0;
        bts2_app_data->spp_srv_inst[i].service_list = 0;
        bts2_app_data->spp_srv_inst[i].spp_service_list = NULL;
    }

    //!zhengyu:add custom uuid
    bt_spp_srv_add_uuid_list();
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send an L2CA_REG_REQ message.
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_start_enb(bts2_app_stru *bts2_app_data)
{
    U8 i;

    for (i = 0; i < SPP_SRV_MAX_CONN_NUM; i++)
    {
        spp_srv_enb_req(i, SPP_DEFAULT_NAME);
    }
    USER_TRACE(">> SPP enabled\n");
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send an L2CA_REG_REQ message.
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_mode_change_req(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chnl, U8 mode)
{
    USER_TRACE(">> SPP change mode\n");

    spp_srv_mode_change_req_ext(device_id, srv_chnl, mode);
    switch (mode)
    {
    case 0:
        USER_TRACE("Change to ACT_MODE\n");
        break;
    case 1:
        USER_TRACE("Change to HOLD_MODE\n");
        break;
    case 2:
        USER_TRACE("Change to SNIFF_MODE\n");
        break;
    case 3:
        USER_TRACE("Change to PARK_MODE\n");
        break;
    }
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send an L2CA_REG_REQ message.
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
//!zhengyu:disable all spp channel
void bt_spp_srv_start_disb(bts2_app_stru *bts2_app_data)
{
    //!zhengyu:Function parameters are only for compatibility with old versions
    spp_srv_disb_req(0);
    USER_TRACE(">> SPP disable\n");
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      send data to special channel
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_sending_data_by_device_id_and_srv_chnl(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl, U8 *payload, U16 payload_len)
{
    bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;

    bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[device_id];

    if ((bts2_spp_srv_inst->cur_link_mode != 0) && (bts2_spp_srv_inst->exit_sniff_pending == FALSE))
    {
        bts2_spp_srv_inst->exit_sniff_pending = TRUE;
        bt_spp_srv_mode_change_req(bts2_app_data, device_id, srv_chl, 0);
    }
    spp_srv_data_req_ext(device_id, srv_chl, payload_len, payload);
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      get mtu size of special channel
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U16 bt_spp_srv_get_mtu_size(bts2_app_stru *bts2_app_data, U8 device_id)
{
    bts2_spp_srv_inst_data *inst_data;

    inst_data = &(bts2_app_data->spp_srv_inst[device_id]);
    return inst_data->spp_srv_mfs;
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *      U16 U16:
 *      bts2_app_stru *bts2_app_data:
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_time_out(U8 spp_id, bts2_app_stru *bts2_app_data)
{
    bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
    bts2_spp_service_list *spp_service_list = NULL;
    U8 device_id = 0xff;
    U8 srv_chl = 0xff;

    device_id = spp_id / SPP_SRV_MAX_CONN_NUM - 1;
    srv_chl = spp_id % SPP_SRV_MAX_CONN_NUM;

    if ((device_id < CFG_MAX_ACL_CONN_NUM) && (bts2_app_data->spp_srv_inst[device_id].service_list != 0))
    {
        bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[device_id];
        spp_service_list = bt_spp_srv_get_service_list_by_srv_chl(bts2_spp_srv_inst, srv_chl);

        if (spp_service_list)
        {
            spp_service_list->timer_flag = FALSE;

            if (spp_service_list->wr_file_hdl != NULL)
            {
                fclose(spp_service_list->wr_file_hdl);
                spp_service_list->wr_file_hdl = NULL;

                if (spp_service_list->counter > 0)
                {
                    USER_TRACE("[SPP:%04X:%02X:%06lX-%d RX]:\n",
                               bts2_spp_srv_inst->bd_addr.nap,
                               bts2_spp_srv_inst->bd_addr.uap,
                               bts2_spp_srv_inst->bd_addr.lap,
                               spp_service_list->srv_chnl);
                    if (spp_service_list->counter < 2000)
                    {
                        USER_TRACE(">> SPP server recive %ld character from peer\n", spp_service_list->counter);
                    }
                    else
                    {
                        USER_TRACE(">> File %s copied with speed %.3f Kbyte/s\n", spp_service_list->rd_file_name, (((double)spp_service_list->counter) / (((double)SUB(spp_service_list->last_time, spp_service_list->start_timer)) / 1000.0)) / 1000);
                    }
                    spp_service_list->counter = 0;
                }
            }
            else
            {
                if (spp_service_list->counter > 0)
                {
                    USER_TRACE("[SPP:%04X:%02X:%06lX-%d RX]:\n",
                               bts2_spp_srv_inst->bd_addr.nap,
                               bts2_spp_srv_inst->bd_addr.uap,
                               bts2_spp_srv_inst->bd_addr.lap,
                               spp_service_list->srv_chnl);
                    if (spp_service_list->counter < 2000)
                    {
                        USER_TRACE(">> SPP server recive %ld character from peer\n", spp_service_list->counter);
                    }
                    else
                    {
                        USER_TRACE(">>SPP received %d bytes with speed %.3f Kbyte/s\n", spp_service_list->counter, (((double)spp_service_list->counter) / (((double)SUB(spp_service_list->last_time, spp_service_list->start_timer)) / 1000.0)) / 1024);
                    }
                    spp_service_list->counter = 0;
                }
                spp_service_list->receive_first_packet = 0;
            }
        }
    }
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
void bt_spp_srv_data_ind_hdl(bts2_app_stru *bts2_app_data)
{
    BTS2S_SPP_SRV_DATA_IND *my_msg;
    U16 *msg_type;
    bts2_spp_srv_inst_data *inst_data;
    U8 spp_data_ind_id;
    U8 spp_id = 0xff;
    bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
    bts2_spp_service_list *spp_service_list = NULL;

    msg_type = (U16 *) bts2_app_data->recv_msg;
    my_msg = (BTS2S_SPP_SRV_DATA_IND *) bts2_app_data->recv_msg;
    bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[my_msg->device_id];
    spp_service_list = bt_spp_srv_get_service_list_by_srv_chl(bts2_spp_srv_inst, my_msg->srv_chl);


#ifndef SOLUTION_WATCH
    spp_id = my_msg->srv_chl + (bts2_spp_srv_inst->device_id + 1) * SPP_SRV_MAX_CONN_NUM;
    spp_srv_data_rsp_ext(my_msg->device_id, my_msg->srv_chl);
#endif


    if (write_into_file)
    {
        if (spp_service_list)
        {
            if (spp_service_list->wr_file_hdl == NULL)
            {
                if ((spp_service_list->wr_file_hdl = fopen(spp_service_list->rd_file_name, "wb")) == NULL)
                {
                    USER_TRACE(" -- SPP open the file %s failed\n", spp_service_list->rd_file_name);
                }
                else
                {
                    spp_service_list->counter = my_msg->payload_len;
                    spp_service_list->start_timer = BTS2_GET_TIME();
                    spp_service_list->timer_flag = TRUE;
                    spp_service_list->time_id = bts2_timer_ev_add(TIME_WITH_NO_DATA, (void (*)(U16, void *)) bt_spp_srv_time_out, (U16)spp_id, (void *) bts2_app_data);
                    fwrite(my_msg->payload, sizeof(U8), my_msg->payload_len, spp_service_list->wr_file_hdl);
                }
            }
            else
            {
                bts2_timer_ev_cancel(spp_service_list->time_id, 0, NULL);
                spp_service_list->counter = my_msg->payload_len + spp_service_list->counter;
                spp_service_list->timer_flag = TRUE;
                spp_service_list->time_id = bts2_timer_ev_add(TIME_WITH_NO_DATA, (void (*)(U16, void *)) bt_spp_srv_time_out, (U16)spp_id, (void *) bts2_app_data);
                fwrite(my_msg->payload, sizeof(U8), my_msg->payload_len, spp_service_list->wr_file_hdl);
            }
        }
    }
    else
    {
#ifndef SOLUTION_WATCH
        {
            if (spp_service_list)
            {
                if (spp_service_list->receive_first_packet == 0)
                {
                    spp_service_list->counter = my_msg->payload_len;
                    spp_service_list->start_timer = BTS2_GET_TIME();
                    spp_service_list->timer_flag = TRUE;
                    spp_service_list->receive_first_packet = 1;
                    spp_service_list->time_id = bts2_timer_ev_add(TIME_WITH_NO_DATA, (void (*)(U16, void *)) bt_spp_srv_time_out, (U16)spp_id, (void *) bts2_app_data);
                }
                else
                {
                    bts2_timer_ev_cancel(spp_service_list->time_id, 0, NULL);
                    spp_service_list->counter = my_msg->payload_len + spp_service_list->counter;
                    spp_service_list->timer_flag = TRUE;
                    spp_service_list->time_id = bts2_timer_ev_add(TIME_WITH_NO_DATA, (void (*)(U16, void *)) bt_spp_srv_time_out, (U16)spp_id, (void *) bts2_app_data);
                }
                // bt_spp_srv_sending_data_by_device_id_and_srv_chnl(bts2_app_data, my_msg->device_id, my_msg->srv_chl, my_msg->payload, my_msg->payload_len);
            }
        }
#else
        {
            if (spp_service_list)
            {
#ifdef CFG_SPP_SRV
                bt_notify_spp_data_ind_t *spp_data_info = (bt_notify_spp_data_ind_t *)bmalloc(sizeof(bt_notify_spp_data_ind_t) + my_msg->payload_len);
                bt_addr_convert(&bts2_spp_srv_inst->bd_addr, spp_data_info->mac.addr);
                spp_data_info->srv_chl = my_msg->srv_chl;
                spp_data_info->uuid_len = my_msg->uuid_len;
                if (spp_data_info->uuid_len == 2)
                {
                    bmemcpy(spp_data_info->uuid.uuid_16, my_msg->uuid, spp_data_info->uuid_len);
                }
                else if (spp_data_info->uuid_len == 4)
                {
                    bmemcpy(spp_data_info->uuid.uuid_32, my_msg->uuid, spp_data_info->uuid_len);
                }
                else if (spp_data_info->uuid_len == 16)
                {
                    bmemcpy(spp_data_info->uuid.uuid_128, my_msg->uuid, spp_data_info->uuid_len);
                }
                spp_data_info->payload_len = my_msg->payload_len;
                bmemcpy(&(spp_data_info->payload), my_msg->payload, my_msg->payload_len);

                bt_interface_bt_event_notify(BT_NOTIFY_SPP, BT_NOTIFY_SPP_DATA_IND, spp_data_info, sizeof(spp_data_info));
                bfree(spp_data_info);
#endif
            }
        }
#endif
    }

    if (spp_service_list)
        spp_service_list->last_time = BTS2_GET_TIME();


    // //!zhengyu:only for debug
    // if (my_msg->payload_len <= 20)
    // {

    //     USER_TRACE("[SPP:%04X:%02X:%06X-%d-0x%x:0x%x RX]:",
    //                bts2_spp_srv_inst->bd_addr.nap,
    //                bts2_spp_srv_inst->bd_addr.uap,
    //                bts2_spp_srv_inst->bd_addr.lap,
    //                my_msg->srv_chl,
    //                my_msg->uuid[0],
    //                my_msg->uuid[1]);
    //     for (int i = 0; i < my_msg->payload_len; i++)
    //     {
    //         rt_kprintf("0x%x ", my_msg->payload[i]);
    //     }
    //     rt_kprintf("\n");
    // }
    // else
    // {
    //     USER_TRACE("[SPP:%04X:%02X:%06lX-%d-%02X:%02X RX]:receive data length = %d\n",
    //                bts2_spp_srv_inst->bd_addr.nap,
    //                bts2_spp_srv_inst->bd_addr.uap,
    //                bts2_spp_srv_inst->bd_addr.lap,
    //                my_msg->srv_chl,
    //                my_msg->uuid[0],
    //                my_msg->uuid[1],
    //                my_msg->payload_len);
    // }

    bfree(my_msg->payload);
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
void bt_spp_srv_rfc_conn_accept_hdl(bts2_app_stru *bts2_app_data, U8 srv_chl, BTS2S_BD_ADDR bd)
{
    //!zhengyu:Just to be compatible with the old version
    spp_srv_conn_rsp(0, srv_chl, bd, TRUE);
    USER_TRACE("<< BD addreess: %04X:%02X:%06lX\n", bd.nap, bd.uap, bd.lap);
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
void bt_spp_srv_rfc_conn_rej_hdl(bts2_app_stru *bts2_app_data, U8 srv_chl, BTS2S_BD_ADDR bd)
{
    //!zhengyu:Just to be compatible with the old version
    spp_srv_conn_rsp(0, srv_chl, bd, FALSE);
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
void bt_spp_srv_test_req(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl)
{
    U8 *test_data = (U8 *)"Hello BTS2!";
    U16 test_len = strlen((char *)test_data);

    spp_srv_test_req_ext(device_id, srv_chl, test_len, test_data);
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
void bt_spp_srv_linest_req(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl)
{
    /* L1 =1 L2=0 L3 =1 L4 =0 */
    spp_srv_line_st_req_ext(device_id, srv_chl, FALSE, 0x90);
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
void bt_spp_srv_switch_role(bts2_app_stru *bts2_app_data)
{
    /* HCI_MASTER = 0 HCI_SLAVE = 1 */
    gap_switch_role_req(bts2_app_data->pair_bd, 1);
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
/*Apply to temporarily use,because set the muxID to be fixed number 0*/
void bt_spp_srv_portneg_req(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl)
{
    BTS2S_PORT_PAR   *port_par;

    port_par = (BTS2S_PORT_PAR *)bmalloc(sizeof(BTS2S_PORT_PAR));

    port_par->port_speed = 0x07;
    port_par->data_bit = 0x03;
    port_par->stop_bit = 0x00;
    port_par->parity = 0x01;
    port_par->parity_type = 0x02;
    port_par->flow_ctrl_mask = 0x04;
    port_par->xon = 0x11;
    port_par->xoff = 0x11;
    port_par->par_mask = 0x3F7F;

    spp_srv_portneg_req_ext(device_id, srv_chl, TRUE, port_par);
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
//addd for rfcomm bqb
void bt_spp_srv_disc_req(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl)
{
    bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
    bts2_spp_service_list *spp_service_list = NULL;

    if ((device_id < CFG_MAX_ACL_CONN_NUM) && (bts2_app_data->spp_srv_inst[device_id].service_list != 0))
    {
        bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[device_id];
        spp_service_list = bt_spp_srv_get_service_list_by_srv_chl(bts2_spp_srv_inst, srv_chl);

        if (spp_service_list)
        {
            USER_TRACE(">> SPP serve disconnect addr: %04X:%02X:%06lX,srv_chl = %d\n",
                       bts2_spp_srv_inst->bd_addr.nap,
                       bts2_spp_srv_inst->bd_addr.uap,
                       bts2_spp_srv_inst->bd_addr.lap,
                       spp_service_list->srv_chnl);
            spp_srv_disc_req_ext(device_id, srv_chl);
        }
    }
}


void bt_spp_srv_disc_req_all(bts2_app_stru *bts2_app_data)
{
    U8 device_id = 0xff;
    U8 srv_chl = 0xff;
    bts2_spp_srv_inst_data *sub_inst;

    for (U8 i = 0; i < CFG_MAX_ACL_CONN_NUM; i++)
    {
        sub_inst = &bts2_app_data->spp_srv_inst[i];
        if (sub_inst->device_id != 0xff)
        {
            bts2_spp_service_list *curr_list = sub_inst->spp_service_list;

            while (curr_list)
            {
                bt_spp_srv_disc_req(bts2_app_data, sub_inst->device_id, curr_list->srv_chnl);
                curr_list = (bts2_spp_service_list *)curr_list->next_struct;
            }
        }
    }
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Handle SPP message.
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
void bt_spp_srv_msg_hdl(bts2_app_stru *bts2_app_data)
{
    U16 *msg_type;
    bts2_spp_srv_inst_data *inst_data;

    msg_type = (U16 *) bts2_app_data->recv_msg;
    switch (*msg_type)
    {
    case BTS2MU_SPP_SRV_ENB_CFM:
    {
        BTS2S_SPP_SRV_ENB_CFM *my_msg;
        my_msg = (BTS2S_SPP_SRV_ENB_CFM *) bts2_app_data->recv_msg;

        if (my_msg->res == BTS2_SUCC)
        {
            INFO_TRACE("spp enable success\n");
        }
        else
        {
            INFO_TRACE("Warning,spp enable fail\n");
        }
        break;
    }
    case BTS2MU_SPP_SRV_CONN_CFM:
    {
        BTS2S_SPP_SRV_CONN_CFM *my_msg = NULL;
        U8 mdm_setting = 0;
        U8 break_sig = 0;
        U8 idx = 0xff;
        bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
        bts2_spp_service_list *spp_service_list = NULL;
        my_msg = (BTS2S_SPP_SRV_CONN_CFM *) bts2_app_data->recv_msg;

        if (my_msg->res == BTS2_SUCC)
        {
            if (bt_spp_srv_check_addr_is_connected(bts2_app_data, &my_msg->bd, &idx))
            {
                INFO_TRACE("[BT_SPP_DEBUG]the addr is connected,then add service channel\n");
                bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[idx];
                bts2_spp_srv_inst->service_list |= (1 << my_msg->srv_chnl);
                spp_service_list = bt_spp_srv_add_new_service_list(bts2_spp_srv_inst, my_msg->srv_chnl);
                spp_service_list->mtu_size = bts2_spp_srv_inst->spp_srv_mfs;
            }
            else
            {
                idx = bt_spp_srv_get_available_sub_inst(bts2_app_data);

                if (idx != 0xff)
                {
                    INFO_TRACE("[BT_SPP_DEBUG]add new addr:%04X:%02X:%06lX\n", my_msg->bd.nap, my_msg->bd.uap, my_msg->bd.lap);
                    bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[idx];
                    bts2_spp_srv_inst->device_id = idx;
                    bmemcpy(&bts2_spp_srv_inst->bd_addr, &my_msg->bd, sizeof(BTS2S_BD_ADDR));
                    bts2_spp_srv_inst->service_list |= (1 << my_msg->srv_chnl);
                    bts2_spp_srv_inst->cur_link_mode = ACT_MODE;
                    bts2_spp_srv_inst->spp_srv_mfs = my_msg->mfs;
                    spp_service_list = bt_spp_srv_add_new_service_list(bts2_spp_srv_inst, my_msg->srv_chnl);
                    spp_service_list->mtu_size = bts2_spp_srv_inst->spp_srv_mfs;
                }
                else
                {
                    BT_SPP_DEBUG("[BT_SPP_DEBUG]Why can't assign a new acl!!!!!!!\n");
                    RT_ASSERT(0);
                }
            }

            mdm_setting = MDM_CTS_MASK | MDM_DCD_MASK | MDM_DSR_MASK;

            spp_srv_ctrl_req_ext(bts2_spp_srv_inst->device_id, my_msg->srv_chnl, mdm_setting, break_sig);

            USER_TRACE(">> Current device is %d,service channel %d\n", bts2_spp_srv_inst->device_id, my_msg->srv_chnl);

            bts2_app_data->spp_srv_conn_nums++;

            USER_TRACE(">> SPP server connect success,mtu_size = %d\n", my_msg->mfs);

            bt_spp_srv_dump_all_spp_connect_information(bts2_app_data);

#ifdef CFG_SPP_SRV

            bt_notify_profile_state_info_t profile_state;
            bt_addr_convert(&bts2_spp_srv_inst->bd_addr, profile_state.mac.addr);
            profile_state.profile_type = BT_NOTIFY_SPP;
            profile_state.res = BTS2_SUCC;
            bt_interface_bt_event_notify(BT_NOTIFY_SPP, BT_NOTIFY_SPP_PROFILE_CONNECTED,
                                         &profile_state, sizeof(bt_notify_profile_state_info_t));

            bt_notify_spp_conn_ind_t spp_conn_ind;
            spp_conn_ind.srv_chl = my_msg->srv_chnl;
            spp_conn_ind.uuid_len = my_msg->uuid_len;
            if (spp_conn_ind.uuid_len == 2)
            {
                bmemcpy(spp_conn_ind.uuid.uuid_16, my_msg->uuid, spp_conn_ind.uuid_len);
            }
            else if (spp_conn_ind.uuid_len == 4)
            {
                bmemcpy(spp_conn_ind.uuid.uuid_32, my_msg->uuid, spp_conn_ind.uuid_len);
            }
            else if (spp_conn_ind.uuid_len == 16)
            {
                bmemcpy(spp_conn_ind.uuid.uuid_128, my_msg->uuid, spp_conn_ind.uuid_len);
            }
            spp_conn_ind.mfs = my_msg->mfs;
            bt_addr_convert(&bts2_spp_srv_inst->bd_addr, spp_conn_ind.mac.addr);
            bt_interface_bt_event_notify(BT_NOTIFY_SPP, BT_NOTIFY_SPP_CONN_IND, &spp_conn_ind, sizeof(spp_conn_ind));

#endif
        }
        else
        {
            BT_SPP_DEBUG("[BT_SPP_DEBUG]Why spp connect fail!!!!!!!\n");
        }
        break;
    }
    case BTS2MU_SPP_SRV_DISB_CFM:
    {
        BTS2S_SPP_SRV_DISB_CFM *my_msg;
        my_msg = (BTS2S_SPP_SRV_DISB_CFM *) bts2_app_data->recv_msg;

        if (my_msg->res == BTS2_SUCC)
        {
            INFO_TRACE("spp disable success\n");
        }
        else
        {
            INFO_TRACE("Warning,spp disable fail\n");
        }
        break;
    }
    case BTS2MU_SPP_SRV_DATA_IND:
    {
        bt_spp_srv_data_ind_hdl(bts2_app_data);
        break;
    }
    case BTS2MU_SPP_SRV_DATA_CFM:
    {
        BTS2S_SPP_SRV_DATA_CFM *my_msg;
        my_msg = (BTS2S_SPP_SRV_DATA_CFM *) bts2_app_data->recv_msg;

        bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
        bts2_spp_service_list *spp_service_list = NULL;

        bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[my_msg->device_id];
        spp_service_list = bt_spp_srv_get_service_list_by_srv_chl(bts2_spp_srv_inst, my_msg->srv_chl);

        if (spp_service_list)
        {
#ifndef SOLUTION_WATCH
            {
                if (spp_service_list->cur_file_hdl != NULL)
                {
#if RT_USING_DFS
                    bt_spp_srv_sending_next_pieceof_file(bts2_app_data, my_msg->device_id, spp_service_list->srv_chnl);
#endif
                }
                else if (spp_service_list->is_rand_going != 0)
                {
                    if ((bts2_spp_srv_inst->cur_link_mode != 0) && (bts2_spp_srv_inst->exit_sniff_pending == FALSE))
                    {
                        bts2_spp_srv_inst->exit_sniff_pending = TRUE;
                        bt_spp_srv_mode_change_req(bts2_app_data, bts2_spp_srv_inst->device_id, my_msg->srv_chl, 0);
                    }
#if 1
                    if (my_msg->credit > 2)
                    {
                        for (U32 i = 0; i < my_msg->credit - 1; i++);
                        bt_spp_srv_sending_random_data_next(spp_service_list, my_msg->device_id);
                    }
                    else
#endif
                        bt_spp_srv_sending_random_data_next(spp_service_list, my_msg->device_id);

                }
            }
#else
            {
#ifdef CFG_SPP_SRV
                bt_notify_spp_data_cfm_t spp_data_cfm;
                spp_data_cfm.srv_chl = my_msg->srv_chl;
                spp_data_cfm.uuid_len = my_msg->uuid_len;
                if (spp_data_cfm.uuid_len == 2)
                {
                    bmemcpy(spp_data_cfm.uuid.uuid_16, my_msg->uuid, spp_data_cfm.uuid_len);
                }
                else if (spp_data_cfm.uuid_len == 4)
                {
                    bmemcpy(spp_data_cfm.uuid.uuid_32, my_msg->uuid, spp_data_cfm.uuid_len);
                }
                else if (spp_data_cfm.uuid_len == 16)
                {
                    bmemcpy(spp_data_cfm.uuid.uuid_128, my_msg->uuid, spp_data_cfm.uuid_len);
                }
                bt_addr_convert(&bts2_spp_srv_inst->bd_addr, spp_data_cfm.mac.addr);

                bt_interface_bt_event_notify(BT_NOTIFY_SPP, BT_NOTIFY_SPP_DATA_CFM, &spp_data_cfm, sizeof(spp_data_cfm));
#endif
            }
#endif
        }
        break;
    }
    case BTS2MU_SPP_SRV_CTRL_IND:
    {
        break;
    }
    case BTS2MU_SPP_SRV_DISC_CFM:
    {
        BTS2S_SPP_SRV_DISC_IND *my_msg;

        my_msg = (BTS2S_SPP_SRV_DISC_IND *) bts2_app_data->recv_msg;


        bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
        bts2_spp_service_list *spp_service_list = NULL;

        BT_SPP_DEBUG("device_id = %d\n", my_msg->device_id);
        bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[my_msg->device_id];
        spp_service_list = bt_spp_srv_get_service_list_by_srv_chl(bts2_spp_srv_inst, my_msg->srv_chl);

        if (spp_service_list)
        {
            spp_service_list->sending = FALSE;
            spp_service_list->counter = 0;
            if (spp_service_list->cur_file_hdl != NULL)
            {
                fclose(spp_service_list->cur_file_hdl);
                spp_service_list->cur_file_hdl = NULL;
            }
            if (spp_service_list->wr_file_hdl != NULL)
            {
                bts2_timer_ev_cancel(spp_service_list->time_id, 0, NULL);
                spp_service_list->timer_flag = FALSE;
                fclose(spp_service_list->wr_file_hdl);
                spp_service_list->wr_file_hdl = NULL;
            }

            bt_spp_srv_delete_service_list_by_srv_chl(bts2_spp_srv_inst, my_msg->srv_chl);

            bts2_app_data->spp_srv_conn_nums--;

            INFO_TRACE(">>SPP server disconnect with peer addr:%04X:%02X:%06X,srv_chl = %d\n", my_msg->bd.nap, my_msg->bd.uap, my_msg->bd.lap, my_msg->srv_chl);

            bts2_spp_srv_inst->service_list &= ~(1 << my_msg->srv_chl);
            if (bts2_spp_srv_inst->service_list == 0)
            {
                bts2_spp_srv_inst->device_id = 0xff;
                bd_set_empty(&bts2_spp_srv_inst->bd_addr);
            }

            bt_spp_srv_dump_all_spp_connect_information(bts2_app_data);

#ifdef CFG_SPP_SRV
            bt_notify_profile_state_info_t profile_state;
            bt_addr_convert(&my_msg->bd, profile_state.mac.addr);
            profile_state.profile_type = BT_NOTIFY_SPP;
            profile_state.res = my_msg->res;
            bt_interface_bt_event_notify(BT_NOTIFY_SPP, BT_NOTIFY_SPP_PROFILE_DISCONNECTED,
                                         &profile_state, sizeof(bt_notify_profile_state_info_t));

            bt_notify_spp_disc_ind_t spp_disc_ind;
            spp_disc_ind.srv_chl = my_msg->srv_chl;
            spp_disc_ind.uuid_len = my_msg->uuid_len;
            if (spp_disc_ind.uuid_len == 2)
            {
                bmemcpy(spp_disc_ind.uuid.uuid_16, my_msg->uuid, spp_disc_ind.uuid_len);
            }
            else if (spp_disc_ind.uuid_len == 4)
            {
                bmemcpy(spp_disc_ind.uuid.uuid_32, my_msg->uuid, spp_disc_ind.uuid_len);
            }
            else if (spp_disc_ind.uuid_len == 16)
            {
                bmemcpy(spp_disc_ind.uuid.uuid_128, my_msg->uuid, spp_disc_ind.uuid_len);
            }
            bt_addr_convert(&my_msg->bd, spp_disc_ind.mac.addr);

            bt_interface_bt_event_notify(BT_NOTIFY_SPP, BT_NOTIFY_SPP_DISC_IND, &spp_disc_ind, sizeof(spp_disc_ind));
#endif
        }
        break;
    }
    case BTS2MU_SPP_SRV_PORTNEG_IND:
    {
        break;
    }
    case BTS2MU_SPP_SRV_MODE_CHANGE_IND:
    {
        BTS2S_SPP_SRV_MODE_CHANGE_IND *msg;
        msg = (BTS2S_SPP_SRV_MODE_CHANGE_IND *)bts2_app_data->recv_msg;

        bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
        bts2_spp_service_list *spp_service_list = NULL;

        bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[msg->device_id];
        spp_service_list = bt_spp_srv_get_service_list_by_srv_chl(bts2_spp_srv_inst, msg->srv_chl);

        if (spp_service_list)
        {
            switch (msg->mode)
            {
            case 0:
                INFO_TRACE("Receive change to ACT_MODE indicator\n");
                break;
            case 2:
                INFO_TRACE("Receive change to SNIFF_MODE indicator\n");
                break;
            }
            bts2_spp_srv_inst->cur_link_mode = msg->mode;
            bts2_spp_srv_inst->exit_sniff_pending = FALSE;
        }
        else
        {
            INFO_TRACE("receive un-connected spp channel mode change ind\n");
        }
        break;
    }

    case BTS2MU_SPP_SRV_CONN_IND:
    {
        BTS2S_SPP_SRV_CONN_IND *my_msg;
        my_msg = (BTS2S_SPP_SRV_CONN_IND *)bts2_app_data->recv_msg;

        INFO_TRACE(">>New connection addr:%04X:%02X:%06X,srv_chl = %d\n", my_msg->bd.nap, my_msg->bd.uap, my_msg->bd.lap, my_msg->srv_chnl);

        uint8_t addr1[6];
        bt_addr_convert(&my_msg->bd, addr1);
        bt_cm_conn_info_t *bonded_dev = bt_cm_find_bonded_dev_by_addr(addr1);
        if (bonded_dev->role == BT_CM_MASTER)
        {
            INFO_TRACE(">>earphone connect spp,should reject\n");
            bt_spp_srv_rfc_conn_rej_hdl(bts2_app_data, my_msg->srv_chnl, my_msg->bd);
        }
        else
        {
            bts2_app_data->menu_id = menu_spp_srv;

            bt_spp_srv_rfc_conn_accept_hdl(bts2_app_data, my_msg->srv_chnl, my_msg->bd);
        }
        break;
    }

    default:
    {
        INFO_TRACE("<< Unhandled message %x in SPP\n", *msg_type);
        break;
    }
    }
}



/*-----------------------------------PUBLIC API-----------------------------------------*/
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *
 *
 * INPUT:
 *
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
BOOL bt_spp_srv_check_addr_is_connected(bts2_app_stru *bts2_app_data, BTS2S_BD_ADDR *bd, U8 *idx)
{
    BOOL is_connected = FALSE;

    BT_SPP_DEBUG("[BT_SPP_DEBUG]bt_spp_srv_check_addr_is_connected\n");

    for (int i = 0; i < CFG_MAX_ACL_CONN_NUM; i++)
    {
        if (0 == memcmp(&(bts2_app_data->spp_srv_inst[i].bd_addr), bd, sizeof(BTS2S_BD_ADDR)))
        {
            is_connected = TRUE;
            *idx = i;
            break;
        }
    }

    if (!is_connected)
    {
        bt_spp_srv_dump_all_spp_connect_information(bts2_app_data);
    }
    return is_connected;
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
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 bt_spp_srv_get_available_sub_inst(bts2_app_stru *bts2_app_data)
{
    U8 idx = 0xff;

    for (int i = 0 ; i < CFG_MAX_ACL_CONN_NUM; i++)
    {
        if (bts2_app_data->spp_srv_inst[i].device_id == 0xff)
        {
            idx = i;
            break;
        }
    }
    return idx;
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
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bts2_spp_service_list *bt_spp_srv_add_new_service_list(bts2_spp_srv_inst_data *spp_srv_inst_ptr, U8 srv_chl)
{
    bts2_spp_service_list *spp_service_list = (bts2_spp_service_list *)malloc(sizeof(bts2_spp_service_list));

    if (!spp_service_list)
    {
        return NULL;
    }

    spp_service_list->srv_chnl = srv_chl;
    spp_service_list->sending = FALSE;
    spp_service_list->cur_file_hdl = NULL;
    spp_service_list->wr_file_hdl = NULL;
    spp_service_list->cur_file_pos = 0;
    spp_service_list->cur_file_size = 0;
    sprintf(spp_service_list->file_name, "");
    sprintf(spp_service_list->rd_file_name, "SPP%02d.txt", srv_chl + (spp_srv_inst_ptr->device_id + 1) * SPP_SRV_MAX_CONN_NUM);
    spp_service_list->start_timer = 0;
    spp_service_list->last_time = 0;
    spp_service_list->time_id = 0;
    spp_service_list->timer_flag = FALSE;
    spp_service_list->counter = 0;
    spp_service_list->rand_total_len = 0;
    spp_service_list->rand_packet_len = 0;
    spp_service_list->rand_buf = NULL;
    spp_service_list->is_rand_going = 0;
    spp_service_list->receive_first_packet = 0;
    spp_service_list->mtu_size = 0;;
    spp_service_list->next_struct = NULL;;


    if (spp_srv_inst_ptr->spp_service_list == NULL)
    {
        spp_srv_inst_ptr->spp_service_list = spp_service_list;
    }
    else
    {
        bts2_spp_service_list *curr_list = spp_srv_inst_ptr->spp_service_list;
        while (curr_list->next_struct != NULL)
        {
            curr_list = (bts2_spp_service_list *)curr_list->next_struct;
        }

        if (curr_list->next_struct == NULL)
        {
            curr_list->next_struct = spp_service_list;
        }
    }
    BT_SPP_DEBUG("[BT_SPP_DEBUG]spp_service_list = %p,srv_chl = %d\n", spp_service_list, srv_chl);
    return spp_service_list;
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
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
bts2_spp_service_list *bt_spp_srv_get_service_list_by_srv_chl(bts2_spp_srv_inst_data *spp_srv_inst_ptr, U8 srv_chl)
{
    bts2_spp_service_list *spp_service_list = NULL;
    bts2_spp_service_list *curr_list = spp_srv_inst_ptr->spp_service_list;
    while (curr_list)
    {
        if (curr_list->srv_chnl == srv_chl)
        {
            spp_service_list = curr_list;
            break;
        }
        else
        {
            curr_list = (bts2_spp_service_list *)curr_list->next_struct;
        }
    }

    if (spp_service_list == NULL)
    {
        BT_SPP_DEBUG("[BT_SPP_DEBUG]why can't find specified service channel,service_list = %d\n", spp_srv_inst_ptr->service_list);
    }

    return spp_service_list;
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
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_delete_service_list_by_srv_chl(bts2_spp_srv_inst_data *spp_srv_inst_ptr, U8 srv_chl)
{
    bts2_spp_service_list *curr_list_pre = NULL;
    bts2_spp_service_list *curr_list = spp_srv_inst_ptr->spp_service_list;

    while (curr_list)
    {
        if (curr_list->srv_chnl == srv_chl)
        {
            if (curr_list_pre == NULL)
            {
                free(curr_list);
                spp_srv_inst_ptr->spp_service_list = curr_list->next_struct;
            }
            else
            {
                curr_list_pre->next_struct = curr_list->next_struct;
                free(curr_list);
            }
            break;
        }
        else
        {
            curr_list_pre = curr_list;
            curr_list = (bts2_spp_service_list *)curr_list->next_struct;
        }
    }
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
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 bt_spp_srv_get_srv_chl_by_uuid(U8 *uuid, U8 uuid_len)
{
    U8 srv_chl = 0;
    BTS2S_SPP_UUID_LIST_EXT *spp_uuid_header;

    spp_uuid_header = spp_get_uuid_list_by_uuid(uuid, uuid_len, &srv_chl);

    return srv_chl;
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
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_dump_all_spp_connect_information(bts2_app_stru *bts2_app_data)
{
    INFO_TRACE("[spp_debug]*****************spp connect info***************************\n");

    for (int i = 0; i < CFG_MAX_ACL_CONN_NUM; i++)
    {
        bts2_spp_srv_inst_data *sub_inst = &bts2_app_data->spp_srv_inst[i];
        INFO_TRACE("[spp_debug]device_id = %d,bd = %04X:%02X:%06X,connect server channel = %x\n",
                   sub_inst->device_id, sub_inst->bd_addr.nap, sub_inst->bd_addr.uap, sub_inst->bd_addr.lap, sub_inst->service_list);
    }

    INFO_TRACE("[spp_debug]************************************************************\n");
}



/************************ TEST API *****************************/

#if RT_USING_DFS
void bt_spp_srv_set_write_into_file(bts2_app_stru *bts2_app_data, U8 on_or_off)
{
    write_into_file = on_or_off;
    USER_TRACE(">> set write_into_file %d\n", write_into_file);
}
#endif

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send an L2CA_REG_REQ message.
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_set_instance_index(bts2_app_stru *bts2_app_data)
{
    U8 device_id = 0xff;
    U8 srv_chl = 0xff;
    bts2_spp_srv_inst_data *sub_inst;

    device_id = bts2_app_data->input_str[1] - '0';
    srv_chl = bts2_app_data->input_str[2] - '0';

    if (device_id > CFG_MAX_ACL_CONN_NUM)
    {
        USER_TRACE(">> Error,please input 0 ~ %d\n", (U8)CFG_MAX_ACL_CONN_NUM);
    }
    else
    {
        sub_inst = &bts2_app_data->spp_srv_inst[device_id];

        if ((sub_inst->service_list & (1 << srv_chl)) == 0)
        {
            USER_TRACE(">> Error,please input service channel in service_list = %x\n", sub_inst->service_list);
        }
        else
        {
            bts2_app_data->select_device_id = device_id;
            bts2_app_data->select_srv_chnl = srv_chl;
        }
    }
}



/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send an L2CA_REG_REQ message.
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
#define MAX_SPP_SEND_DATA 0x800000
void bt_spp_srv_sending_random_data(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl)
{
    bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
    bts2_spp_service_list *spp_service_list = NULL;

    if ((device_id < CFG_MAX_ACL_CONN_NUM) && (bts2_app_data->spp_srv_inst[device_id].service_list != 0))
    {
        bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[device_id];
        spp_service_list = bt_spp_srv_get_service_list_by_srv_chl(bts2_spp_srv_inst, srv_chl);

        if (spp_service_list)
        {
            if (spp_service_list->sending == FALSE)
            {
                spp_service_list->sending = TRUE;
                spp_service_list->is_rand_going = 1;
                spp_service_list->rand_total_len = MAX_SPP_SEND_DATA;
                spp_service_list->rand_packet_len = atoi((const char *)&bts2_app_data->input_str[2]);
                if (spp_service_list->rand_packet_len > spp_service_list->mtu_size)
                {
                    spp_service_list->rand_packet_len = spp_service_list->mtu_size;
                }
                if (!spp_service_list->rand_buf)
                    spp_service_list->rand_buf = bmalloc(spp_service_list->rand_packet_len);
                for (U32 i = 0; i < spp_service_list->rand_packet_len; i++)
                    spp_service_list->rand_buf[i] = (U8)rand();

                // Just treat send out.
                spp_service_list->rand_total_len -= spp_service_list->rand_packet_len;
                if ((bts2_spp_srv_inst->cur_link_mode != 0) && (bts2_spp_srv_inst->exit_sniff_pending == FALSE))
                {
                    bts2_spp_srv_inst->exit_sniff_pending = TRUE;
                    bt_spp_srv_mode_change_req(bts2_app_data, device_id, srv_chl, 0);
                }
                spp_srv_data_req_ext(device_id, srv_chl, spp_service_list->rand_packet_len, spp_service_list->rand_buf);
                // rfcomm will release this buffer
                spp_service_list->rand_buf = NULL;
            }
        }
        else
        {
            INFO_TRACE("Please enter a valid value\n");
        }
    }
    else
    {
        INFO_TRACE("Please enter a valid value\n");
    }
}


U8 g_t_count = 0;
U8 bt_spp_srv_sending_random_data_next(bts2_spp_service_list *spp_service_list, U8 device_id)
{
    U8 is_end = 1;
    if (spp_service_list)
    {
        if (spp_service_list->rand_total_len != 0)
        {
            is_end = 0;
            if (spp_service_list->rand_total_len >= spp_service_list->rand_packet_len)
                spp_service_list->rand_total_len -= spp_service_list->rand_packet_len;
            else
            {
                spp_service_list->rand_packet_len = spp_service_list->rand_total_len;
                spp_service_list->rand_total_len = 0;
            }
            if (spp_service_list->rand_buf == NULL)
                spp_service_list->rand_buf = bmalloc(spp_service_list->rand_packet_len);
            spp_service_list->rand_buf[0] = g_t_count++;
            for (U32 i = 1; i < spp_service_list->rand_packet_len; i++)
                spp_service_list->rand_buf[i] = (U8)(i % 256);
            //instance->rand_buf[i] = (U8)rand();
            //memset(instance->rand_buf, g_t_count++, instance->rand_packet_len);
            spp_srv_data_req_ext(device_id, spp_service_list->srv_chnl, spp_service_list->rand_packet_len, spp_service_list->rand_buf);
            // RFCOMM will release this buffer
            spp_service_list->rand_buf = NULL;
        }
    }

    if (is_end)
    {
        if (spp_service_list->rand_buf)
            bfree(spp_service_list->rand_buf);
        spp_service_list->is_rand_going = 0;
        spp_service_list->rand_buf = NULL;
        spp_service_list->rand_packet_len = 0;
        spp_service_list->rand_total_len = 0;
        spp_service_list->sending = FALSE;
    }
    return is_end;
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send an L2CA_REG_REQ message.
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
#if RT_USING_DFS
void bt_spp_srv_select_file_to_send(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl)
{
    bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
    bts2_spp_service_list *spp_service_list = NULL;

    if ((device_id < CFG_MAX_ACL_CONN_NUM) && (bts2_app_data->spp_srv_inst[device_id].service_list != 0))
    {
        bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[device_id];
        spp_service_list = bt_spp_srv_get_service_list_by_srv_chl(bts2_spp_srv_inst, srv_chl);

        if (spp_service_list)
        {
            if (spp_service_list->sending == FALSE)
            {
                int i;

                for (i = 0; i < MAX_ONLY_FILE_NAME_LEN; i++)
                {
                    spp_service_list->file_name[i] = '\0';
                }
                bstrcpy(spp_service_list->file_name, (char *)&bts2_app_data->input_str[2]);
                bt_spp_srv_open_the_selected_file(bts2_app_data, device_id, srv_chl);
            }
        }
        else
        {
            INFO_TRACE("Please enter a valid value\n");
        }
    }
    else
    {
        INFO_TRACE("Please enter a valid value\n");
    }
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send an L2CA_REG_REQ message.
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_sending_the_selected_file(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl)
{
    U32 bytes_to_rd;
    bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
    bts2_spp_service_list *spp_service_list = NULL;

    if ((device_id < CFG_MAX_ACL_CONN_NUM) && (bts2_app_data->spp_srv_inst[device_id].service_list != 0))
    {
        bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[device_id];
        spp_service_list = bt_spp_srv_get_service_list_by_srv_chl(bts2_spp_srv_inst, srv_chl);

        if (spp_service_list)
        {
            if ((spp_service_list->cur_file_size - spp_service_list->cur_file_pos) <= spp_service_list->mtu_size)
            {
                bytes_to_rd = spp_service_list->cur_file_size - spp_service_list->cur_file_pos;
            }
            else
            {
                bytes_to_rd = spp_service_list->mtu_size;
            }

            if (bytes_to_rd > 0)
            {
                U16 bytes_rd;
                U8 *body;

                body = (U8 *) bmalloc(bytes_to_rd);
                bytes_rd = fread(body, 1, bytes_to_rd, spp_service_list->cur_file_hdl);
                //USER_TRACE(">> send data %d\n",bytes_to_rd);
                spp_service_list->cur_file_pos += bytes_rd;
                if ((bts2_spp_srv_inst->cur_link_mode != 0) && (bts2_spp_srv_inst->exit_sniff_pending == FALSE))
                {
                    bts2_spp_srv_inst->exit_sniff_pending = TRUE;
                    bt_spp_srv_mode_change_req(bts2_app_data, device_id, srv_chl, 0);
                }
                spp_srv_data_req_ext(device_id, srv_chl, bytes_rd, body);

                if (bytes_to_rd < spp_service_list->mtu_size)
                {
                    USER_TRACE(">> SPP server send file finish\n");
                }


            }
            if (spp_service_list->cur_file_pos >= spp_service_list->cur_file_size)
            {
                fclose(spp_service_list->cur_file_hdl);
                spp_service_list->cur_file_pos = 0;
                spp_service_list->cur_file_hdl = NULL;
                spp_service_list->sending = FALSE;
            }
        }
        else
        {
            INFO_TRACE("Please enter a valid value\n");
        }
    }
    else
    {
        INFO_TRACE("Please enter a valid value\n");
    }
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send an L2CA_REG_REQ message.
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_open_the_selected_file(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl)
{
    bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
    bts2_spp_service_list *spp_service_list = NULL;

    if ((device_id < CFG_MAX_ACL_CONN_NUM) && (bts2_app_data->spp_srv_inst[device_id].service_list != 0))
    {
        bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[device_id];
        spp_service_list = bt_spp_srv_get_service_list_by_srv_chl(bts2_spp_srv_inst, srv_chl);

        if (spp_service_list)
        {
            if (spp_service_list->cur_file_hdl != NULL)
            {
                fclose(spp_service_list->cur_file_hdl);
                spp_service_list->cur_file_hdl = NULL;
            }

            spp_service_list->cur_file_pos = 0;

            if (spp_service_list->cur_file_hdl == NULL)
            {
                if ((spp_service_list->cur_file_hdl = fopen(spp_service_list->file_name, "rb")) == NULL)
                {
                    USER_TRACE(">> SPP open the file %s failed\n", spp_service_list->file_name);
                }
                else
                {
                    spp_service_list->cur_file_size = bt_get_file_size(spp_service_list->cur_file_hdl);
                    spp_service_list->sending = TRUE;
                    bt_spp_srv_sending_the_selected_file(bts2_app_data, device_id, srv_chl);
                }
            }
        }
        else
        {
            INFO_TRACE("Please enter a valid value\n");
        }
    }
    else
    {
        INFO_TRACE("Please enter a valid value\n");
    }
}


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send an L2CA_REG_REQ message.
 *
 * INPUT:
 *      void *msg: message.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bt_spp_srv_sending_next_pieceof_file(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl)
{
    bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;
    bts2_spp_service_list *spp_service_list = NULL;

    if ((device_id < CFG_MAX_ACL_CONN_NUM) && (bts2_app_data->spp_srv_inst[device_id].service_list != 0))
    {
        bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[device_id];
        spp_service_list = bt_spp_srv_get_service_list_by_srv_chl(bts2_spp_srv_inst, srv_chl);

        if (spp_service_list)
        {
            if (spp_service_list->cur_file_hdl != NULL)
            {
                bt_spp_srv_sending_the_selected_file(bts2_app_data, device_id, srv_chl);
            }
            else
            {
                if (spp_service_list->cur_file_hdl != NULL)
                {
                    fclose(spp_service_list->cur_file_hdl);
                    spp_service_list->cur_file_hdl = NULL;
                    spp_service_list->sending = FALSE;
                }
                spp_service_list->cur_file_pos = 0;
            }
        }
    }
}
#endif

void bt_spp_srv_sending_data_to_peer(bts2_app_stru *bts2_app_data, U8 device_id, U8 srv_chl)
{
    U8 *body;
    bts2_spp_srv_inst_data *bts2_spp_srv_inst = NULL;

    bts2_spp_srv_inst = &bts2_app_data->spp_srv_inst[device_id];

    body = (U8 *)bmalloc(bts2_app_data->input_str_len - 2);
    bmemcpy(body, &bts2_app_data->input_str[2], bts2_app_data->input_str_len - 2);

    if ((bts2_spp_srv_inst->cur_link_mode != 0) && (bts2_spp_srv_inst->exit_sniff_pending == FALSE))
    {
        bts2_spp_srv_inst->exit_sniff_pending = TRUE;
        bt_spp_srv_mode_change_req(bts2_app_data, device_id, srv_chl, 0);
    }
    spp_srv_data_req_ext(device_id, srv_chl, bts2_app_data->input_str_len - 2, body);
}

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
