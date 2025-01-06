/**
  ******************************************************************************
  * @file   bts2_app_spp_c.c
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

#ifdef CFG_SPP_CLT

#define LOG_TAG         "btapp_sppc"
#include "log.h"


static void bt_spp_clt_init_single_inst(bts2_app_stru *bts2_app_data, U8 spp_id)
{
    bts2_app_data->spp_inst[spp_id].spp_clt_st = spp_clt_idle;
    bts2_app_data->spp_inst[spp_id].spp_id = spp_id;
    bts2_app_data->spp_inst[spp_id].mfs = 0;
    bts2_app_data->spp_inst[spp_id].sending = FALSE;
    bts2_app_data->spp_inst[spp_id].cur_file_hdl = NULL;
    bts2_app_data->spp_inst[spp_id].wr_file_hdl = NULL;
    bts2_app_data->spp_inst[spp_id].cur_file_pos = 0;
    sprintf(bts2_app_data->spp_inst[spp_id].file_name, "");
    sprintf(bts2_app_data->spp_inst[spp_id].rd_file_name, "SPP%02d.txt", spp_id);
    bts2_app_data->spp_inst[spp_id].start_timer = 0;
    bts2_app_data->spp_inst[spp_id].last_time = 0;
    bts2_app_data->spp_inst[spp_id].time_id = 0;
    bts2_app_data->spp_inst[spp_id].timer_flag = FALSE;
    bts2_app_data->spp_inst[spp_id].counter = 0;
}

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
void bt_spp_clt_init(bts2_app_stru *bts2_app_data)
{
    U8 i = 0;

    bts2_app_data->select_spp_clt_id = 0;
    bts2_app_data->spp_conn_nums = 0;
    bts2_app_data->inst_ptr = &bts2_app_data->spp_inst[0];

    for (i = 0; i < SPP_CLT_MAX_CONN_NUM; i++)
    {
        bt_spp_clt_init_single_inst(bts2_app_data, i);
    }
}

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
void bt_spp_clt_set_instance_index(bts2_app_stru *bts2_app_data)
{
    if ((bts2_app_data->input_str[0] - '0' < SPP_CLT_MAX_CONN_NUM) && !(bts2_app_data->input_str[1]))
    {
        bts2_app_data->select_spp_clt_id = bts2_app_data->input_str[0] - '0';
        USER_TRACE(">> Select SPP index is:%d\n", bts2_app_data->select_spp_clt_id);
    }
    else
    {
        USER_TRACE(">> Error,please input 0 ~ %d\n", (U8)SPP_CLT_MAX_CONN_NUM);
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
void bt_spp_clt_start_w4conn(bts2_app_stru *bts2_app_data)
{
    spp_clt_conn_req(&(bts2_app_data->bd_list[bts2_app_data->dev_idx]),
                     0xff,
                     FALSE,
                     NULL);
    USER_TRACE(">> SPP connect device address: %04X:%02X:%06lX\n",
               bts2_app_data->bd_list[bts2_app_data->dev_idx].nap,
               bts2_app_data->bd_list[bts2_app_data->dev_idx].uap,
               bts2_app_data->bd_list[bts2_app_data->dev_idx].lap);
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
void bt_spp_clt_disc_req(bts2_app_stru *bts2_app_data)
{
    bts2_app_data->inst_ptr = &(bts2_app_data->spp_inst[bts2_app_data->select_spp_clt_id]);

    if (bts2_app_data->inst_ptr->spp_clt_st == spp_clt_conned)
    {
        USER_TRACE(">> SPP disconnect with peer id %d\n", bts2_app_data->select_spp_clt_id);
        spp_clt_disc_req(bts2_app_data->select_spp_clt_id);
        bts2_app_data->inst_ptr->spp_clt_st = spp_clt_idle;
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
void bt_spp_clt_mode_change_req(bts2_app_stru *bts2_app_data)
{
    bts2_spp_inst_data *ptr;
    ptr = bts2_app_data->inst_ptr;

    if (ptr->spp_clt_st == spp_clt_conned)
    {
        USER_TRACE(">> SPP change mode\n");
        spp_clt_mode_change_req(bts2_app_data->select_spp_clt_id, (U8)(bts2_app_data->input_str[0] - '0'));

        switch (bts2_app_data->input_str[0])
        {
        case '0':
            USER_TRACE("Change to ACT_MODE\n");
            break;
        case '1':
            USER_TRACE("Change to HOLD_MODE\n");
            break;
        case '2':
            USER_TRACE("Change to SNIFF_MODE\n");
            break;
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
void bt_spp_clt_select_file_to_send(bts2_app_stru *bts2_app_data)
{
    bts2_spp_inst_data *ptr;

    ptr = &(bts2_app_data->spp_inst[bts2_app_data->select_spp_clt_id]);

    if (ptr->spp_clt_st == spp_clt_conned)
    {
        if (ptr->sending == FALSE)
        {
            int i;

            bts2_app_data->file_name_len = 0;
            for (i = 0; i < MAX_ONLY_FILE_NAME_LEN; i++)
            {
                ptr->file_name[i] = '\0';
            }
            bstrcpy(ptr->file_name, (char *)bts2_app_data->input_str);
            bt_spp_clt_open_the_selected_file(bts2_app_data);
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
void bt_spp_clt_sending_data_to_peer(bts2_app_stru *bts2_app_data)
{
    bts2_spp_inst_data *inst_data;
    U8 *body;

    inst_data = bts2_app_data->inst_ptr;
    body = (U8 *)bmalloc(bts2_app_data->input_str_len);
    BT_OOM_ASSERT(body);
    if (body)
    {
        bmemcpy(body, bts2_app_data->input_str, bts2_app_data->input_str_len);
        spp_clt_data_req(inst_data->spp_id, bts2_app_data->input_str_len, body);
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
void bt_spp_clt_sending_the_selected_file(bts2_app_stru *bts2_app_data, bts2_spp_inst_data *inst_data)
{
    U32 bytes_to_rd;

    if ((inst_data->cur_file_size - inst_data->cur_file_pos) <= inst_data->mfs)
    {
        bytes_to_rd = inst_data->cur_file_size - inst_data->cur_file_pos;
    }
    else
    {
        bytes_to_rd = inst_data->mfs;
    }

    if (bytes_to_rd > 0)
    {
        U16 bytes_rd;
        U8 *body;

        body = (U8 *) bmalloc(bytes_to_rd);
        BT_OOM_ASSERT(body);
        if (body)
        {
            bytes_rd = fread(body, 1, bytes_to_rd, inst_data->cur_file_hdl);
            inst_data->cur_file_pos += bytes_rd;
            USER_TRACE(">> send data %ld\n", bytes_to_rd);
            spp_clt_data_req(inst_data->spp_id, bytes_rd, body);
        }

        if (bytes_to_rd < inst_data->mfs)
        {
            USER_TRACE(">> SPP client send file finish\n");
        }
    }
    if (inst_data->cur_file_pos >= inst_data->cur_file_size)
    {
        fclose(inst_data->cur_file_hdl);
        inst_data->cur_file_pos = 0;
        inst_data->cur_file_hdl = NULL;
        inst_data->sending = FALSE;
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
void bt_spp_clt_open_the_selected_file(bts2_app_stru *bts2_app_data)
{
    bts2_spp_inst_data *ptr;

    ptr = &(bts2_app_data->spp_inst[bts2_app_data->select_spp_clt_id]);

    ptr->cur_file_pos = 0;

    if ((ptr->spp_clt_st == spp_clt_conned) && ptr->cur_file_hdl == NULL)
    {
        if ((ptr->cur_file_hdl = fopen(ptr->file_name, "rb")) == NULL)
        {
            USER_TRACE(" -- SPP open the file %s failed\n", ptr->file_name);
        }
        else
        {
            ptr->cur_file_size = bt_get_file_size(ptr->cur_file_hdl);
            USER_TRACE("File size %ld bytes\n", ptr->cur_file_size);
            ptr->sending = TRUE;
            bt_spp_clt_sending_the_selected_file(bts2_app_data, ptr);
        }
    }
    else
    {
        if (ptr->cur_file_hdl != NULL)
        {
            fclose(ptr->cur_file_hdl);
            ptr->cur_file_hdl = NULL;
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
void bt_spp_clt_sending_next_pieceof_file(bts2_app_stru *bts2_app_data, bts2_spp_inst_data *inst_data)
{
    if (inst_data->cur_file_hdl != NULL && inst_data->spp_clt_st == spp_clt_conned)
    {
        bt_spp_clt_sending_the_selected_file(bts2_app_data, inst_data);
    }
    else
    {
        if (inst_data->cur_file_hdl != NULL)
        {
            fclose(inst_data->cur_file_hdl);
            inst_data->cur_file_hdl = NULL;
            inst_data->sending = FALSE;
        }
        inst_data->cur_file_pos = 0;
    }
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
void bt_spp_clt_time_out(U8 spp_id, bts2_app_stru *bts2_app_data)
{
    bts2_spp_inst_data *inst_data;
    // bts2_app_data->cur_spp_clt_id = spp_id;

    if (spp_id != SPP_INSTANCE_UNDEFINED)
    {
        inst_data = &bts2_app_data->spp_inst[spp_id];
    }
    else
    {
        USER_TRACE(">> No instance data time out\n");
        return;
    }

    inst_data->timer_flag = FALSE;

    if (inst_data->wr_file_hdl != NULL)
    {
        fclose(inst_data->wr_file_hdl);
        inst_data->wr_file_hdl = NULL;

        if (inst_data->counter > 0)
        {
            if (inst_data->counter < 2000)
            {
                USER_TRACE(">> Recive %ld character from peer id %d\n", inst_data->counter, spp_id);
            }
            else
            {
                USER_TRACE("File %s copied with speed %.3f Kbyte/s from peer %d\n", inst_data->rd_file_name, (((double)inst_data->counter) / (((double)SUB(inst_data->last_time, inst_data->start_timer)) / 1000.0)) / 1000, spp_id);
            }
            inst_data->counter = 0;
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
void bt_spp_clt_data_ind_hdl(bts2_app_stru *bts2_app_data)
{
    BTS2S_SPP_CLT_DATA_IND *my_msg;
    bts2_spp_inst_data *inst_data;

    my_msg = (BTS2S_SPP_CLT_DATA_IND *) bts2_app_data->recv_msg;
    inst_data = &(bts2_app_data->spp_inst[my_msg->spp_id]);
    if (inst_data == NULL)
    {
        USER_TRACE(">> No instance data return\n");
        return;
    }
    spp_clt_data_rsp(my_msg->spp_id);
#if 0
    if (inst_data->wr_file_hdl == NULL)
    {
        if ((inst_data->wr_file_hdl = fopen(inst_data->rd_file_name, "wb")) == NULL)
        {
            USER_TRACE(">> SPP open the file %s failed\n", inst_data->rd_file_name);
        }
        else
        {
            inst_data->counter = my_msg->payload_len;
            inst_data->start_timer = BTS2_GET_TIME();
            inst_data->timer_flag = TRUE;
            inst_data->time_id = bts2_timer_ev_add(TIME_WITH_NO_DATA, (void (*)(U16, void *)) bt_spp_clt_time_out, (U16)(my_msg->spp_id), (void *) bts2_app_data);
            fwrite(my_msg->payload, sizeof(U8), my_msg->payload_len, inst_data->wr_file_hdl);
        }
    }
    else
    {
        bts2_timer_ev_cancel(inst_data->time_id, 0, NULL);
        inst_data->counter = my_msg->payload_len + inst_data->counter;
        inst_data->timer_flag = TRUE;
        inst_data->time_id = bts2_timer_ev_add(TIME_WITH_NO_DATA, (void (*)(U16, void *)) bt_spp_clt_time_out, (U16)(my_msg->spp_id), (void *) bts2_app_data);
        fwrite(my_msg->payload, sizeof(U8), my_msg->payload_len, inst_data->wr_file_hdl);
    }
#else
    {
        inst_data->counter = my_msg->payload_len;
        DBG_DATA(my_msg->payload, inst_data->counter, "SPP_RX:");
    }
#endif
    inst_data->last_time = BTS2_GET_TIME();
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
void bt_spp_clt_test_req(bts2_app_stru *bts2_app_data)
{
    U8 *test_data = (U8 *)"Hello BTS2!";
    U16 test_len = strlen((char *)test_data);

    bts2_spp_inst_data *inst_data = &(bts2_app_data->spp_inst[bts2_app_data->select_spp_clt_id]);
    spp_clt_test_req(bts2_app_data->select_spp_clt_id, test_len, test_data);
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
void bt_spp_clt_linest_req(bts2_app_stru *bts2_app_data)
{
    bts2_spp_inst_data *inst_data = &(bts2_app_data->spp_inst[bts2_app_data->select_spp_clt_id]);
    /* L1 =1 L2=0 L3 =1 L4 =0 */
    spp_clt_line_st_req(bts2_app_data->select_spp_clt_id, FALSE, 0x90);
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
void bt_spp_clt_switch_role(bts2_app_stru *bts2_app_data)
{
    bts2_spp_inst_data *inst_data = &(bts2_app_data->spp_inst[bts2_app_data->select_spp_clt_id]);

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
void bt_spp_clt_portneg_req(bts2_app_stru *bts2_app_data)
{
    BTS2S_PORT_PAR   *port_par;
    bts2_spp_inst_data *inst_data = &(bts2_app_data->spp_inst[bts2_app_data->select_spp_clt_id]);

    port_par = (BTS2S_PORT_PAR *)bmalloc(sizeof(BTS2S_PORT_PAR));
    BT_OOM_ASSERT(port_par);
    if (port_par)
    {
        port_par->port_speed = 0x07;
        port_par->data_bit = 0x03;
        port_par->stop_bit = 0x00;
        port_par->parity = 0x01;
        port_par->parity_type = 0x02;
        port_par->flow_ctrl_mask = 0x04;
        port_par->xon = 0x11;
        port_par->xoff = 0x11;
        port_par->par_mask = 0x3F7F;

        spp_clt_port_neg_req(bts2_app_data->select_spp_clt_id, TRUE, port_par);
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
void bt_spp_clt_msg_hdl(bts2_app_stru *bts2_app_data)
{
    U16 *msg_type;
    bts2_spp_inst_data *inst_data;

    msg_type = (U16 *) bts2_app_data->recv_msg;
    switch (*msg_type)
    {
    case BTS2MU_SPP_CLT_CONN_CFM:
    {
        BTS2S_SPP_CLT_CONN_CFM *my_msg = NULL;
        U8    mdm_setting = 0;
        U8 break_sig = 0;
        my_msg = (BTS2S_SPP_CLT_CONN_CFM *) bts2_app_data->recv_msg;

        inst_data = &(bts2_app_data->spp_inst[my_msg->spp_id]);

        if (inst_data == NULL)
        {
            INFO_TRACE(">> No spp instance get BTS2MU_SPP_CLT_ENB_CFM\n");
            return;
        }

        if (my_msg->res == BTS2_SUCC)
        {
            inst_data->spp_clt_st = spp_clt_conned;
            inst_data->mfs = my_msg->mfs;
            inst_data->sending = FALSE;

            mdm_setting = MDM_RTS_MASK | MDM_DTR_MASK;

            spp_clt_ctrl_req(my_msg->spp_id, mdm_setting, break_sig);
            bts2_app_data->inst_ptr = &(bts2_app_data->spp_inst[my_msg->spp_id]);
            bts2_app_data->spp_conn_nums++;

            USER_TRACE(">> SPP client connect id %d success\n", my_msg->spp_id);
            USER_TRACE(">> Remote device server chanle is:%d\n", my_msg->rmt_srv_chnl);
        }
        else
        {
            USER_TRACE(">> SPP client connect id %d fail\n", my_msg->spp_id);
            inst_data->spp_clt_st = spp_clt_idle;
        }
        break;
    }
    case BTS2MU_SPP_CLT_DATA_IND:
    {
        bt_spp_clt_data_ind_hdl(bts2_app_data);
        break;
    }
    case BTS2MU_SPP_CLT_DATA_CFM:
    {
        BTS2S_SPP_CLT_DATA_CFM *my_msg;
        my_msg = (BTS2S_SPP_CLT_DATA_CFM *) bts2_app_data->recv_msg;

        inst_data = &(bts2_app_data->spp_inst[my_msg->spp_id]);

        INFO_TRACE("Remote credit:%d\n", my_msg->credit);

        if (inst_data == NULL)
        {
            INFO_TRACE(">> No spp instance get BTS2MU_SPP_CLT_DATA_CFM\n");
            return;
        }
        else
        {
            bt_spp_clt_sending_next_pieceof_file(bts2_app_data, inst_data);
        }
        break;
    }
    case BTS2MU_SPP_CLT_CTRL_IND:
    {
        break;
    }
    case BTS2MU_SPP_CLT_DISC_CFM:
    {
        BTS2S_SPP_CLT_DISC_CFM *my_msg;

        my_msg = (BTS2S_SPP_CLT_DISC_CFM *) bts2_app_data->recv_msg;

        inst_data = &(bts2_app_data->spp_inst[my_msg->spp_id]);
        if (inst_data == NULL)
        {
            INFO_TRACE(">> No spp instance get BTS2MU_SPP_CLT_DISC_IND\n");
            return;
        }

        if (inst_data->cur_file_hdl != NULL)
        {
            fclose(inst_data->cur_file_hdl);
            inst_data->cur_file_hdl = NULL;
        }
        if (inst_data->wr_file_hdl != NULL)
        {
            bts2_timer_ev_cancel(inst_data->time_id, 0, NULL);
            inst_data->timer_flag = FALSE;
            fclose(inst_data->wr_file_hdl);
            inst_data->wr_file_hdl = NULL;
        }

        bt_spp_clt_init_single_inst(bts2_app_data, my_msg->spp_id);
        bts2_app_data->inst_ptr = &(bts2_app_data->spp_inst[0]);
        bts2_app_data->spp_conn_nums--;

        USER_TRACE(">>SPP client disconnect with peer id %d\n", my_msg->spp_id);
        break;
    }
    case BTS2MU_SPP_CLT_PORTNEG_IND:
    {
        break;
    }
    case BTS2MU_SPP_CLT_MODE_CHANGE_IND:
    {
        BTS2S_SPP_CLT_MODE_CHANGE_IND *msg;
        msg = (BTS2S_SPP_CLT_MODE_CHANGE_IND *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            switch (msg->mode)
            {
            case 0:
                INFO_TRACE("Receive change to ACT_MODE indicator\n");
                break;
            case 1:
                INFO_TRACE("Receive change to HOLD_MODE indicator\n");
                break;
            case 2:
                INFO_TRACE("Receive change to SNIFF_MODE indicator\n");
                break;
            case 3:
                INFO_TRACE("Receive change to PARK_MODE indicator\n");
                break;
            }
        }
        else
        {
            INFO_TRACE(">> SPP change mode failed\n");
        }
        break;
    }
    default:
    {
        INFO_TRACE("<< unhandled message %x in SPP\n", *msg_type);
        break;
    }
    }
}

#endif/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
