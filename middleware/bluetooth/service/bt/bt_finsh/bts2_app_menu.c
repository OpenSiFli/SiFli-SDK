/**
  ******************************************************************************
  * @file   bts2_app_menu.c
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

#ifndef BSP_USING_PC_SIMULATOR
    #undef printf
    #define printf rt_kprintf
#endif

#ifdef BTS2_APP_MENU

#include "log.h"

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
static void bt_disply_menu_start(void)
{
    char ver[BTS2_VER_LEN];
    bts2_get_ver(ver);
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##               BTS2 Demo Application              ##\n");
    printf("##                                                  ##\n");
    printf("##     Release at:                                  ##\n");
    printf("##                     %s %s %s       ##\n", __DATE__, ",", __TIME__);
    printf("##     BTS2 version:                                ##\n");
    printf("##                     %s        ##\n", ver);
    printf("##                                                  ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Build and send an L2CA_REG_REQ message.
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
static void bt_disply_menu_main(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           BTS2 Demo Main Menu                    ##\n");
    printf("##   1. Generic Command                             ##\n");
#ifdef CFG_SPP_CLT
    printf("##   2. SPP Client                                  ##\n");
#endif
#ifdef CFG_SPP_SRV
    printf("##   3. SPP Server                                  ##\n");
#endif
#ifdef CFG_HFP_HF
    printf("##   4. HFP HF                                      ##\n");
#endif
#ifdef CFG_AV_SNK
    printf("##   6. A2DP Sink                                   ##\n");
#endif
#ifdef CFG_AV_SRC
    printf("##   7. A2DP Source                                 ##\n");
#endif
    //add for l2cap bqb test
    printf("##   8. L2CAP bqb test                              ##\n");

#ifdef CFG_BT_DIS
    printf("##   9. bt_dis                                      ##\n");
#endif

#ifdef CFG_BT_L2CAP_PROFILE
    printf("##   a. bt_l2cap_profile                            ##\n");
#endif

#ifdef BT_FINSH_PAN
    printf("##   h. PAN                                         ##\n");
#endif
#ifdef CFG_PBAP_CLT
    printf("##   m. PBAP Client                                 ##\n");
#endif
#ifdef CFG_VDP_SNK
    printf("##   n. VDP Sink                                    ##\n");
#endif
#ifdef CFG_VDP_SRC
    printf("##   o. VDP Source                                  ##\n");
#endif
#ifdef CFG_AVRCP
    printf("##   p. AVRCP                                       ##\n");
#endif

#ifdef CFG_BR_GATT_SRV
    printf("##   x.  bt_gatt                                    ##\n");
#endif

#ifdef CFG_HFP_AG
    printf("##   y. HFP AG                                      ##\n");
#endif

#ifdef CFG_HID
    printf("##   z. hid                                         ##\n");
#endif


    printf("##   s. Show Menu                                   ##\n");
    printf("##   q. Exit                                        ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_main(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
    {
        bts2_app_data->menu_id = menu_gen;
        break;
    }
#ifdef CFG_SPP_CLT
    case '2':
    {
        bts2_app_data->menu_id = menu_spp_clt;
        break;
    }
#endif
#ifdef CFG_SPP_SRV
    case '3':
    {
        bts2_app_data->menu_id = menu_spp_srv;
        break;
    }
#endif
#ifdef CFG_HFP_HF
    case '4':
    {
        bts2_app_data->menu_id = menu_hfp_hf;
        break;
    }
#endif

#ifdef CFG_AV_SNK
    case '6':
    {
        bts2_app_data->menu_id = menu_av_snk;
        break;
    }
#endif
#ifdef CFG_AV_SRC
    case '7':
    {
        bts2_app_data->menu_id = menu_av_src;
        break;
    }
#endif

//add for l2cap bqb test
    case '8':
    {
        bts2_app_data->menu_id = menu_l2cap_bqb;
        break;
    }

#ifdef CFG_BT_DIS
    case '9':
    {
        bts2_app_data->menu_id = menu_bt_dis;
        break;
    }
#endif

#ifdef CFG_BT_L2CAP_PROFILE
    case 'a':
    {
        bts2_app_data->menu_id = menu_bt_l2cap_profile;
        break;
    }
#endif

#ifdef BT_FINSH_PAN
    case 'h':
    {
        bts2_app_data->menu_id = menu_pan_g;
        break;
    }
#endif
#ifdef CFG_PBAP_CLT
    case 'm':
    {
        bts2_app_data->menu_id = menu_pbap_c;
        break;
    }
#endif
#ifdef CFG_VDP_SNK
    case 'n':
    {
        bts2_app_data->menu_id = menu_vdp_snk;
        break;
    }
#endif
#ifdef CFG_VDP_SRC
    case 'o':
    {
        bts2_app_data->menu_id = menu_vdp_src;
        break;
    }
#endif
#ifdef CFG_AVRCP
    case 'p':
    {
        bts2_app_data->menu_id = menu_avrcp;
        break;
    }
#endif

#ifdef CFG_BR_GATT_SRV
    case 'x':
    {
        bts2_app_data->menu_id = menu_bt_gatt_srv;
        break;
    }
#endif

#ifdef CFG_HFP_AG
    case 'y':
    {
        bts2_app_data->menu_id = menu_hfp_ag;
        break;
    }
#endif

#ifdef CFG_HID
    case 'z':
    {
        bts2_app_data->menu_id = menu_hid;
        break;
    }
#endif
    case 's':
    {
        break;
    }
    case 'q':
    {
#ifdef BT_FINSH_PAN
#ifdef CFG_GNU
        Inter_Done();
#endif
#endif
#ifdef CFG_MS
        bt_close_kb_thread();
#endif
        bts2_rel();
        bts2_stop();
#ifdef CFG_DBG_MEM_LEAK
        mem_leak_dump();
#endif
#ifdef BSP_USING_PC_SIMULATOR
        exit(0);
#endif
        break;
    }
    default:
        break;
    }
    bt_disply_menu(bts2_app_data);
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
static void bt_disply_menu_gen(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Generic Command Menu                   ##\n");
    printf("##   1. Inquiry start                               ##\n");
    printf("##   2. Inquiry cancel                              ##\n");
    printf("##   3. Select device from Inquiry list             ##\n");
    printf("##   4. Sc menu                                     ##\n");
    printf("##   5. Local device info                           ##\n");
    printf("##   6. Get remote device info                      ##\n");
    printf("##   7. Scan mode                                   ##\n");
    printf("##   8. Link menu                                   ##\n");
    printf("##   9. Service Browse                              ##\n");
    printf("##   a. Set IO Settings                             ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_gen(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
    {
        bt_start_inquiry(bts2_app_data);
        break;
    }
    case '2':
    {
        bt_stop_inquiry(bts2_app_data);
        break;
    }

    case '3':
        bts2_app_data->menu_id = menu_gen_3;
        bt_disply_menu(bts2_app_data);
        break;
    case '4':
        bts2_app_data->menu_id = menu_gen_4;
        bt_disply_menu(bts2_app_data);
        break;
    case '5':
        bts2_app_data->menu_id = menu_gen_5;
        bt_disply_menu(bts2_app_data);
        break;
    case '6':
        bts2_app_data->menu_id = menu_gen_6;
        bt_disply_menu(bts2_app_data);
        break;
    case '7':
        bts2_app_data->menu_id = menu_gen_7;
        bt_disply_menu(bts2_app_data);
        break;
    case '8':
        bts2_app_data->menu_id = menu_gen_8;
        bt_disply_menu(bts2_app_data);
        break;
    case '9':
        bts2_app_data->menu_id = menu_gen_9;
        bt_disply_menu(bts2_app_data);
        break;
    case 'a':
        bts2_app_data->menu_id = menu_gen_a;
        bt_disply_menu(bts2_app_data);
        break;

    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
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
static void bt_disply_menu_gen_3(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Device Select Menu                     ##\n");
    printf("##   0 ~ 9. Input Device Number of Inquiry List     ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_gen_3(bts2_app_stru *bts2_app_data)
{
    if (BT_IN(bts2_app_data->input_str[0] - '0', 0, bts2_app_data->bd_list_num))
    {
        bts2_app_data->dev_idx = bts2_app_data->input_str[0] - '0';
        printf("<< Select %d device successfully!\n", bts2_app_data->dev_idx);
        bt_cm_connect_req(&bts2_app_data->bd_list[bts2_app_data->dev_idx], 0);
    }
    else if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_gen;
        bt_disply_menu(bts2_app_data);
    }
    else
        printf(">> Input again:");
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
static void bt_disply_menu_gen_4_3(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##            Set Security Mode                     ##\n");
    printf("##   1. Set mode 1                                  ##\n");
    printf("##   2. Set mode 2                                  ##\n");
    printf("##   3. Set mode 3                                  ##\n");
    printf("##   4. Set mode 4                                  ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_gen_4_3(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
    case '2':
    case '3':
    case '4':
        bt_set_secu_mode(bts2_app_data);
        break;
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_gen_4;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
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
static void bt_disply_menu_gen_4(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##                SC Menu                           ##\n");
    printf("##   1. Pair with remote device                     ##\n");
    printf("##   2. Unpair with remote device                   ##\n");
    printf("##   3. Set security mode                           ##\n");
    printf("##   4. Read paired device db                       ##\n");
    printf("##   5. Delete all paired records                   ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_gen_4(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
        bt_start_pairing(bts2_app_data);
        break;
    case '2':
        bt_start_unpairing(bts2_app_data);
        break;
    case '3':
        bts2_app_data->menu_id = menu_gen_4_3;
        bt_disply_menu(bts2_app_data);
        break;
    case '4':
        sc_rd_paired_dev_record_req(bts2_task_get_app_task_id());
        break;
    case '5':
        sc_unpair_req(bts2_app_data->phdl, NULL);
        break;
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_gen;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
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
static void bt_disply_menu_gen_5(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Local device info Menu                 ##\n");
    printf("##   1. Set local device name (length < 50)         ##\n");
    printf("##   2. Read local device name                      ##\n");
    printf("##   3. Read local address                          ##\n");
    printf("##   4. Write class of device                       ##\n");
    printf("##   5. Read class of device                        ##\n");
    printf("##   6. Read local version                          ##\n");
    printf("##   7. Set fix pin code                            ##\n");
    printf("##   8. Write page timeout                          ##\n");
    printf("##   9. Enable device under test mode               ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_gen_5(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
    {
        bts2_app_data->menu_id = menu_gen_5_1;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '2':
        gap_rd_local_name_req(bts2_app_data->phdl);
        break;
    case '3':
        gap_rd_local_bd_req(bts2_app_data->phdl);
        break;
    case '4':
    {
        bts2_app_data->menu_id = menu_gen_5_4;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '5':
        gap_rd_dev_cls_req(bts2_app_data->phdl);
        break;
    case '6':
        gap_rd_local_version_req(bts2_app_data->phdl);
        break;
    case '7':
    {
        bts2_app_data->menu_id = menu_gen_5_7;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '8':
    {
        bts2_app_data->menu_id = menu_gen_5_8;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '9':
        gap_enb_dut_mode_req(bts2_app_data->phdl);
        break;
    case 'a':
    {
        bts2_app_data->menu_id = menu_gen_5_a;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_gen;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
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
static void bt_disply_menu_gen_5_1(void)
{
    printf("-- Please input new name: ");
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
static void bt_hdl_menu_gen_5_1(bts2_app_stru *bts2_app_data)
{
    bt_set_local_name(bts2_app_data);
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
static void bt_disply_menu_gen_5_4(void)
{
    printf("-- Please input class of device(0x******): ");
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
static void bt_hdl_menu_gen_5_4(bts2_app_stru *bts2_app_data)
{
    bt_wr_dev_cls(bts2_app_data);
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
static void bt_disply_menu_gen_5_7(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Set Fix Pin Code Menu                  ##\n");
    printf("##   Input Pin Code (length between 4 and 16)       ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_gen_5_7(bts2_app_stru *bts2_app_data)
{
    int i = 0;
    if (bts2_app_data->input_str_len == 1 && bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_gen_5;
        bt_disply_menu(bts2_app_data);
    }
    else if (bts2_app_data->input_str_len > HCI_MAX_PIN_LEN)
    {
        printf("<< Pin code is too long! Please input again.\n");
    }
    else
    {
        memset(bts2_app_data->pin_code, 0, HCI_MAX_PIN_LEN);
        memcpy(bts2_app_data->pin_code, bts2_app_data->input_str, bts2_app_data->input_str_len);
        bts2_app_data->pin_code_len = strlen((const char *)bts2_app_data->input_str);
        printf("<< Pin Code has been set to \"");
        for (i = 0; i < bts2_app_data->input_str_len; i++)
        {
            printf("%c", bts2_app_data->pin_code[i]);
        }
        printf("\".\n");
        bts2_app_data->menu_id = menu_gen_5;
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
static void bt_disply_menu_gen_5_8(void)
{
    printf("-- Please input 0x0000 ~ 0xFFFF: ");
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
static void bt_hdl_menu_gen_5_8(bts2_app_stru *bts2_app_data)
{
    bt_wr_page_timeout(bts2_app_data);
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
static void bt_disply_menu_gen_6(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Get Remote Device Info Menu            ##\n");
    printf("##   1. Role discovery                              ##\n");
    printf("##   2. Get remote device name                      ##\n");
    printf("##   3. Read remote version                         ##\n");
    printf("##   s. Show menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_gen_6(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
        bts2_app_data->menu_id = menu_gen_6_1;
        bt_disply_menu(bts2_app_data);
        break;
    case '2':
        bts2_app_data->menu_id = menu_gen_6_2;
        bt_disply_menu(bts2_app_data);
        break;
    case '3':
        bts2_app_data->menu_id = menu_gen_6_3;
        bt_disply_menu(bts2_app_data);
        break;
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_gen;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
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
static void bt_disply_menu_gen_6_1(void)
{
    printf("-- Role discovery, please input 0 ~ 9: ");
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
static void bt_hdl_menu_gen_6_1(bts2_app_stru *bts2_app_data)
{
    bt_get_role_discov(bts2_app_data);
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
static void bt_disply_menu_gen_6_2(void)
{
    printf("-- Get remote device name, please input 0 ~ 9: ");
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
static void bt_hdl_menu_gen_6_2(bts2_app_stru *bts2_app_data)
{
    bt_get_rmt_dev_name(bts2_app_data);
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
static void bt_disply_menu_gen_6_3(void)
{
    printf("-- Get remote version, please input 0 ~ 9: ");
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
static void bt_hdl_menu_gen_6_3(bts2_app_stru *bts2_app_data)
{
    bt_get_rmt_version(bts2_app_data);
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
static void bt_disply_menu_gen_7(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Scan Mode Select Menu                  ##\n");
    printf("##   0. No scans enabled                            ##\n");
    printf("##   1. Inquiry scan enabled and Page scan disabled ##\n");
    printf("##   2. Page scan enabled and Inquiry scan disabled ##\n");
    printf("##   3. Inquiry scan and Page scan enabled          ##\n");
    printf("##   4. Read scan mode                              ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_gen_7(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '0':
        gap_wr_scan_enb_req(bts2_app_data->phdl, FALSE, FALSE);
        break;
    case '1':
        gap_wr_scan_enb_req(bts2_app_data->phdl, TRUE,  FALSE);
        break;
    case '2':
        gap_wr_scan_enb_req(bts2_app_data->phdl, FALSE, TRUE);
        break;
    case '3':
        gap_wr_scan_enb_req(bts2_app_data->phdl, TRUE,  TRUE);
        break;
    case '4':
        gap_rd_scan_enb_req(bts2_app_data->phdl);
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_gen;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
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
static void bt_disply_menu_gen_8(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##             Link      Menu                       ##\n");
    printf("##   1. Write link supervision timeout              ##\n");
    printf("##   2. Read transmit power level                   ##\n");
    printf("##   3. Get link quality                            ##\n");
    printf("##   4. Read clock                                  ##\n");
    printf("##   5. Get RSSI                                    ##\n");
    printf("##   6. Read link policy                            ##\n");
    printf("##   7. Write link policy                           ##\n");
    printf("##   8. Enter sniff mode                            ##\n");
    printf("##   9. Exit sniff mode                             ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_gen_8(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
    {
        bts2_app_data->menu_id = menu_gen_8_1;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '2':
    {
        bts2_app_data->menu_id = menu_gen_8_2;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '3':
        gap_get_link_qa_req(bts2_app_data->phdl, bts2_app_data->pair_bd);
        break;
    case '4':
    {
        bts2_app_data->menu_id = menu_gen_8_4;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '5':
    {
        bts2_app_data->menu_id = menu_gen_8_5;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '6':
        gap_rd_link_policy_req(bts2_app_data->phdl, bts2_app_data->pair_bd);
        break;
    case '7':
    {
        bts2_app_data->menu_id = menu_gen_8_7;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '8':
    {
        bts2_app_data->menu_id = menu_gen_8_8;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '9':
    {
        bts2_app_data->menu_id = menu_gen_8_9;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_gen;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
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
static void bt_disply_menu_gen_8_1(void)
{
    printf("-- Please set link supervision timeout(0x0001 ~ 0xffff): ");
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
static void bt_hdl_menu_gen_8_1(bts2_app_stru *bts2_app_data)
{
    bt_wr_link_supvisn_timeout(bts2_app_data);
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
static void bt_disply_menu_gen_8_2(void)
{
    printf("-- Please input 1 or 2(1: Current level; 2: Maximum level): ");
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
static void bt_hdl_menu_gen_8_2(bts2_app_stru *bts2_app_data)
{
    bt_rd_transmit_power_level(bts2_app_data);
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
static void bt_disply_menu_gen_8_4(void)
{
    printf("-- Please input clock(0: Local clock; 1: Piconet clock): ");
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
static void bt_hdl_menu_gen_8_4(bts2_app_stru *bts2_app_data)
{
    bt_rd_clock(bts2_app_data);
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
static void bt_disply_menu_gen_8_5(void)
{
    printf("-- Please input 0 ~ 9: ");
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
static void bt_hdl_menu_gen_8_5(bts2_app_stru *bts2_app_data)
{
    bt_get_rssi(bts2_app_data);
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
static void  bt_disply_menu_gen_8_7(void)
{
    printf("-- Please input link policy, leave a space between parameters.\n\n");
    printf("-- 1. Link policy settings (0xXXXX)\n\n");
    printf("-- 2. Sniff settings: max interval   (0xXXXX)\n");
    printf("--                    min interval   (0xXXXX)\n");
    printf("--                    attempt        (0xXXXX)\n");
    printf("--                    timeout        (0xXXXX)\n\n");
    printf("-- 3. Set up link policy(Y/N)\n");
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
static void bt_hdl_menu_gen_8_7(bts2_app_stru *bts2_app_data)
{
    bt_wr_link_policy(bts2_app_data);
}

static void  bt_disply_menu_gen_8_8(void)
{
    printf("-- Enter sniff mode(max_inv_min_inv_n_temp_timeout).\n\n");
}

static void bt_hdl_menu_gen_8_8(bts2_app_stru *bts2_app_data)
{
    bt_etner_sniff_mode(bts2_app_data);
}

static void  bt_disply_menu_gen_8_9(void)
{
    printf("-- Exit sniff mode.\n\n");
}

static void bt_hdl_menu_gen_8_9(bts2_app_stru *bts2_app_data)
{
    bt_exit_sniff_mode(bts2_app_data);
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
static void bt_disply_menu_gen_9(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           SDP Service Browse Menu                ##\n");
    printf("##   0 ~ 9. Input Device Number of Inquiry List     ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_disply_menu_gen_a(void)
{
    printf("\n");
    printf("#####################################################\n");
    printf("##                                                 ##\n");
    printf("##              IO  Settings  Menu                 ##\n");
    printf("## IO CAPABILITY:                                  ##\n");
    printf("##     00: DISPLAY ONLY                            ##\n");
    printf("##     01: DISPLAY YES_NO                          ##\n");
    printf("##     02: KEYBOARD ONLY                           ##\n");
    printf("##     03: NO_INPUT NO OUTPUT                      ##\n");
    printf("##                                                 ##\n");
    printf("## IO AUTH REQUIRE:                                ##\n");
    printf("##     00: MITM_PROTECT_SINGLE_PROFILE_NOT_REQUIRED##\n");
    printf("##     01: MITM_PROTECT_SINGLE_PROFILE_REQUIRED    ##\n");
    printf("##     02: MITM_PROTECT_ALL_PROFILE_NOT_REQUIRED   ##\n");
    printf("##     03: MITM_PROTECT_ALL_PROFILE_REQUIRED       ##\n");
    printf("##                                                 ##\n");
    printf("## OOB DATA PRESENT:                               ##\n");
    printf("##     00: OOB_AUTH_DATA_NOT_PRESENT               ##\n");
    printf("##     01: OOB_AUTH_DATA_FROM_RMT_DEVICE_PRESENT   ##\n");
    printf("##     r. Return to last menu                      ##\n");
    printf("##                                                 ##\n");
    printf("#####################################################\n");
    printf("\n");
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
static void bt_hdl_menu_gen_9(bts2_app_stru *bts2_app_data)
{
    U32 svc = 0x00000100;

    U16 attr_id = 0x0001;/*TP/SERVER/SSA/BV-01-C*/
    /*SVC_RECORD_ST_ATTRUTE_ID  TP/SERVER/SSA/BV-07-C*/

#ifdef CFG_BQB_SDC_SSA
    U16 svc_list_size;
    U16 attr_list_size;
    U8 *attr_list = NULL;
    U8 *srch_ptr = NULL;
#endif

    // if (BT_IN(bts2_app_data->input_str[0] - '0', 0, bts2_app_data->bd_list_num))
    // {
    //     bts2_app_data->dev_idx = bts2_app_data->input_str[0] - '0';

    // }
    switch (bts2_app_data->input_str[0])
    {
    case '1':
    {
        sdpa_sdc_open_srch_req(bts2_task_get_app_task_id(), &bts2_app_data->bd_list[bts2_app_data->dev_idx]);
        break;
    }
    case '2':
    {
        GAP_BT_UUID uuid_list;
        uuid_list.len = LEN_UUID_16;
        uuid_list.uu.uuid16 = 0x0019;
        sdpa_sdc_svc_srch_req(bts2_task_get_app_task_id(),
                              &bts2_app_data->bd_list[bts2_app_data->dev_idx],
                              1,
                              &uuid_list,
                              0x100);
        break;
    }
    case '3':
    {
        U16 attr_list = 0x0100;
        sdpa_sdc_svc_attrute_req(bts2_task_get_app_task_id(),
                                 &bts2_app_data->bd_list[bts2_app_data->dev_idx],
                                 0x00010002,  // service_handle
                                 1,
                                 &attr_list,
                                 0x80);
        break;
    }
    case '4':
    {
        extern void gap_sdc_svc_srch_req_send(U16 app_hdl,
                                              BTS2S_BD_ADDR bd,
                                              U16 num_uuid,
                                              GAP_BT_UUID * uuid_list,
                                              U16 total_uuid_len,
                                              U8 local_srv_chnl);
        GAP_BT_UUID uuid_list;
        uuid_list.len = LEN_UUID_16;
        uuid_list.uu.uuid16 = 0x0019;
        gap_sdc_svc_srch_req_send(bts2_task_get_app_task_id(),
                                  bts2_app_data->bd_list[bts2_app_data->dev_idx],
                                  1, &uuid_list, 0, 0);
        break;
    }
    case '5':
    {
        extern void  gap_sdc_attrute_req(U32 tid,
                                         U16 attrute_indentifier,
                                         U16 max_bytes_to_return);
        U16 attr_list = 0x0100;
        gap_sdc_attrute_req(0x00010002, attr_list, MAX_FRIENDLY_NAME_LEN);
        break;
    }
    case 's':
    {
        bt_disply_menu(bts2_app_data);
        break;
    }
    case 'r':
    {
        bts2_app_data->menu_id = menu_gen;
        bt_disply_menu(bts2_app_data);
        break;
    }
    default:
        break;
    }
// #ifdef CFG_BQB_SDC_SSA
//         GAP_BT_UUID uuid_list;
//         uuid_list.len = LEN_UUID_16;
//         uuid_list.uu.uuid16 = 0x0019;

//         sdpa_sdc_svc_srch_req(bts2_task_get_app_task_id(),
//                               &bts2_app_data->bd_list[bts2_app_data->dev_idx],
//                               1,
//                               &uuid_list,
//                               0x100);
// #else
//         // gap_sdc_srch_req(bts2_task_get_app_task_id(), bts2_app_data->bd_list[bts2_app_data->dev_idx], &svc, 1);
// #endif
//     }
//     else if (bts2_app_data->input_str[0] == 'r')        //return menu
//     {
//         bts2_app_data->menu_id = menu_gen;
//         bt_disply_menu(bts2_app_data);
//     }
//     else if (bts2_app_data->input_str[0] == 's')        //show menu again
//     {
//         bt_disply_menu(bts2_app_data);
//     }
//     else
//         printf(" -- input again:");
}

static void bt_disply_menu_cur_sec_mode(void)
{
    printf("Please  set Current  Security Mode[1,2,3,4]:\n");
}

static void bt_hdl_menu_gen_a(bts2_app_stru *bts2_app_data)
{
    U8   uio_capability = 0;
    U8   uio_auth_require = 0;
    U8   uoob_data_present = 0 ;
    U8   i = 0;

    if (bts2_app_data->input_str_len >= 0x08)
    {
        uio_capability = (bts2_app_data->input_str[0] - 0x30) * 10 + (bts2_app_data->input_str[1] - 0x30);
        if (uio_capability)

            for (i = 2 ; i < bts2_app_data->input_str_len; i++)
            {
                if (bts2_app_data->input_str[i] == ' ')
                    continue;
                else
                    break;
            }
        uio_auth_require = (bts2_app_data->input_str[i] - 0x30) * 10 + (bts2_app_data->input_str[i + 1] - 0x30);

        for (i = i + 2; i < bts2_app_data->input_str_len; i++)
        {
            if (bts2_app_data->input_str[i] == ' ')
                continue;
            else
                break;
        }
        uoob_data_present = (bts2_app_data->input_str[i] - 0x30) * 10 + (bts2_app_data->input_str[i + 1] - 0x30);

        printf(">> Note: Set io_capability:%d,oob_data_present:%d,io_auth_require:%d\n", uio_capability, uoob_data_present, uio_auth_require);
    }
    else if ((bts2_app_data->input_str_len == 0x01) && (bts2_app_data->input_str[0] == 'r'))
    {
        bts2_app_data->menu_id = menu_gen;
        bt_disply_menu(bts2_app_data);
    }
    else if ((bts2_app_data->input_str_len == 0x01) && (bts2_app_data->input_str[0] == 's'))
    {
        bt_disply_menu(bts2_app_data);
    }
}

#ifdef BT_FINSH_PAN
void bt_disply_menu_pan(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           PAN Menu                               ##\n");
    printf("##   1. Connecing                                   ##\n");
    printf("##   2. Disconnect                                  ##\n");
    printf("##   3. Register                                    ##\n");
    printf("##   4. void                                        ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

static void bt_hdl_menu_pan(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
    {
        bt_pan_conn(&(bts2_app_data->last_conn_bd));
        break;
    }
    case '2':
    {
        bt_pan_disc(&(bts2_app_data->last_conn_bd));
        break;
    }
    case '3':
    {
        bt_pan_reg(bts2_app_data);
        break;
    }
    case '4':
    {
        break;
    }
    case 's':
    {
        bt_disply_menu(bts2_app_data);
        break;
    }
    case 'r':
    {
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    }
    default:
        break;
    }
}
#endif
#ifdef CFG_HFP_HF
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
void bt_disply_menu_hfp_hf(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##                  HFP HF Menu                     ##\n");
    printf("##   1. HF conn to the selected AG                  ##\n");
    printf("##   2. HF disc with AG or vice versa               ##\n");
    printf("##   3. Accept the remote side conn request         ##\n");
    printf("##   4. Reject the remote side conn request         ##\n");
    printf("##   5. Dial out                                    ##\n");
    printf("##   6. Memory dial                                 ##\n");
    printf("##   7. Last number redial                          ##\n");
    printf("##   8. Voice Recognition                           ##\n");
    printf("##   9. DTMF                                        ##\n");
    printf("##   a. 3 way                                       ##\n");
    printf("##   b. Call transfer                               ##\n");
    printf("##   c. Answer the incoming call                    ##\n");
    printf("##   d. Reject or Terminate the call                ##\n");
    printf("##   e. Speaker volume control                      ##\n");
    printf("##   f. Microphone volume control                   ##\n");
    printf("##   g. CCWA                                        ##\n");
#if 0
    printf("##   g. Phone book download                         ##\n");
    printf("##   h. Short msg                                   ##\n");
#endif
    printf("##   h. Network operator info                       ##\n");
    printf("##   i. List current call status                    ##\n");
    printf("##   j. CMEE                                        ##\n");
    printf("##   k. CNUM                                        ##\n");
    printf("##   l. BTRH                                        ##\n");
    printf("##   m. Turning of AG's EC and NR                   ##\n");
    printf("##   n. CLIP                                        ##\n");
    printf("##   o. BINP                                        ##\n");
    printf("##   p. hs connect                                  ##\n");
    printf("##   q. ckpd                                        ##\n");
    printf("##   t. set wbs                                     ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_hfp_hf(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {

    case '1':
    {
        //bt_hfp_hf_start_connecting(bts2_app_data);
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);

        hfp_hf_conn_req(&bd, HF_CONN);
        break;
    }
    case '2':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);
        bt_hfp_hf_start_disc(&bd);
        break;
    }
    case '3':
    {
        bt_hfp_hf_rfc_conn_accept_hdl();
        break;
    }
    case '4':
    {
        bt_hfp_hf_rfc_conn_rej_hdl();
        //smsInit(bts2_app_data);
        break;
    }
    case '5':
    {
        bts2_app_data->menu_id = menu_hfp_hf_5;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '6':
    {
        int memory = atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_hf_dial_by_mem_send((U16)memory);
        break;
    }
    case '7':
    {
        bt_hfp_hf_last_num_dial_send();
        break;
    }
    case '8':
    {
        int active = atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_hf_voice_recog_send((U8)active);
        break;
    }
    case '9':
    {
        bt_hfp_hf_at_dtmf_send(bts2_app_data->input_str[1]);
        break;
    }
    case 'a':
    {
        bts2_app_data->menu_id = menu_hfp_hf_a;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case 'b':
    {
        int voice_flag = atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_hf_audio_transfer((U8)voice_flag);
        break;
    }
    case 'c':
    {
        bt_hfp_hf_answer_call_send();
        break;
    }
    case 'd':
    {
        bt_hfp_hf_hangup_call_send();
        break;
    }
    case 'e':
    {
        int vol = atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_hf_update_spk_vol((U8)vol);
        break;
    }
    case 'f':
    {
        int vol = atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_hf_update_mic_vol((U8)vol);
        break;
    }

    case 'g':
    {
        int val = atoi((const char *)bts2_app_data->input_str + 1);
        hfp_hf_send_at_ccwa_api((BOOL)val);
        break;
    }
    case 'h':
    {
        bts2_app_data->menu_id = menu_hfp_hf_h;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case 'i':
    {
        bt_hfp_hf_at_clcc_send();
        break;
    }
    case 'j':
    {
        int val = atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_hf_at_cmee_send(val);
        break;
    }
    case 'k':
    {
        bt_hfp_hf_at_cnum_send();
        break;
    }
    case 'l':
    {
        bts2_app_data->menu_id = menu_hfp_hf_l;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case 'm':
    {
        bt_hfp_hf_at_nrec_send();
        break;
    }
    case 'n':
    {
        int enable = atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_hf_at_clip_send((U8)enable);
        break;
    }
    case 'o':
    {
        bt_hfp_hf_at_binp_send();
        break;
    }
    case 'p':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);

        hfp_hf_conn_req(&bd, HS_CONN);
        break;
    }
    case 'q':
    {
        hfp_hs_send_at_ckpd_api();
        break;
    }
    case 't':
    {
        int wbs_flag = atoi((const char *)bts2_app_data->input_str + 1);
        hfp_hf_set_wbs((U8) wbs_flag);
        break;
    }
    case 's':
    {
        bt_disply_menu(bts2_app_data);
        break;
    }
    case 'r':
    {
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    }
    default:
    {
        break;
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
static void bt_disply_menu_hfp_hf_5(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Dial Menu                              ##\n");
    printf("##   Input phone numbers (Digitals)                 ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_hfp_hf_5(bts2_app_stru *bts2_app_data)
{
    U32 i;
    U8 isdigitflag = 0;

    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_hfp_hf;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        for (i = 0; i < (U32)(bts2_app_data->input_str_len); i++)
        {
            if (isdigit(bts2_app_data->input_str[i]))
                continue;
            else
                break;
        }
        if (i == (U32)(bts2_app_data->input_str_len))
            isdigitflag = 1;

        if (!(U32)(bts2_app_data->input_str_len))
            isdigitflag = 0;

        if ((isdigitflag) && (U32)((bts2_app_data->input_str_len) < 13))
        {

            bt_hfp_hf_make_call_by_number_send(bts2_app_data->input_str, bts2_app_data->input_str_len);
        }
        else
        {
            printf(">> input err...\n");
        }
    }
}

/*----------------------------------------------------------------------------*hfp_hf_send_at_chld_control_api((U8
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
static void bt_disply_menu_hfp_hf_a_1(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           AT+CHLD=1x Menu                        ##\n");
    printf("##   Input number(0 ~ 9)                            ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_hfp_hf_a_1(bts2_app_stru *bts2_app_data)
{
    int i;

    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_hfp_hf_a;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        for (i = bts2_app_data->input_str_len; i >= 0; i--)
        {
            bts2_app_data->input_str[i + 1] = bts2_app_data->input_str[i];
        }
        bts2_app_data->input_str[0] = 1 + '0';
        bts2_app_data->input_str_len++;
        bt_hfp_hf_at_chld_send(bts2_app_data->input_str, bts2_app_data->input_str_len);
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
static void bt_disply_menu_hfp_hf_a_2(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           AT+CHLD=2x Menu                        ##\n");
    printf("##   Input number(0 ~ 9)                            ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_hfp_hf_a_2(bts2_app_stru *bts2_app_data)
{
    int i;

    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_hfp_hf_a;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        for (i = bts2_app_data->input_str_len; i >= 0; i--)
        {
            bts2_app_data->input_str[i + 1] = bts2_app_data->input_str[i];
        }
        bts2_app_data->input_str[0] = 2 + '0';
        bts2_app_data->input_str_len++;
        bt_hfp_hf_at_chld_send(bts2_app_data->input_str, bts2_app_data->input_str_len);
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
static void bt_disply_menu_hfp_hf_a(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           3Way Select Menu                       ##\n");
    printf("##   0. AT+CHLD=0                                   ##\n");
    printf("##   1. AT+CHLD=1                                   ##\n");
    printf("##   2. AT+CHLD=2                                   ##\n");
    printf("##   3. AT+CHLD=3                                   ##\n");
    printf("##   4. AT+CHLD=4                                   ##\n");
    printf("##   5. AT+CHLD=1x                                  ##\n");
    printf("##   6. AT+CHLD=2x                                  ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_hfp_hf_a(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    {
        bt_hfp_hf_at_chld_send(bts2_app_data->input_str, bts2_app_data->input_str_len);
        break;
    }
    case '5':
    {
        bts2_app_data->menu_id = menu_hfp_hf_a_1;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '6':
    {
        bts2_app_data->menu_id = menu_hfp_hf_a_2;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case 'r':
    {
        bts2_app_data->menu_id = menu_hfp_hf;
        bt_disply_menu(bts2_app_data);
        break;
    }
    default:
    {
        printf(">> input err...\n");
        break;
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
static void bt_disply_menu_hfp_hf_h(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##          	 Network operator Menu              ##\n");
    printf("##   0.	 Query Network Operator info                ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_hfp_hf_h(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '0':
        bt_hfp_hf_at_cops_cmd_send();
        break;
    case 'r':
        bts2_app_data->menu_id = menu_hfp_hf;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        printf(">> input error\n");
        break;
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
static void bt_disply_menu_hfp_hf_l(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##      Bluetooth reponse and hold feature          ##\n");
    printf("##   0.	Put incoming call on hold                   ##\n");
    printf("##   1.	Accept held incoming call on hold           ##\n");
    printf("##   2.	Reject held incoming call                   ##\n");
    printf("##   3.	Read current reponse and hold status        ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_hfp_hf_l(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '0':
        printf(">> enter 0...\n");
        bts2_app_data->input_str[0] = 0;
        bt_hfp_hf_at_btrh_cmd_send(bts2_app_data->input_str[0]);
        break;
    case '1':

        bts2_app_data->input_str[0] = 1;
        bt_hfp_hf_at_btrh_cmd_send(bts2_app_data->input_str[0]);
        break;
    case '2':

        bts2_app_data->input_str[0] = 2;
        bt_hfp_hf_at_btrh_cmd_send(bts2_app_data->input_str[0]);
        break;
    case '3':

        bts2_app_data->input_str[0] = 3;
        bt_hfp_hf_at_btrh_query_send();
        break;
    case 'r':
        bts2_app_data->menu_id = menu_hfp_hf;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        printf(">> input err...\n");
        break;
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
static void bt_display_menu_hfp_hf_p(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Send short message                     ##\n");
    printf("##   input send data                                ##\n");
    printf("##   r. return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_hfp_hf_p(bts2_app_stru *bts2_app_data)
{
    bts2_hfp_hf_inst_data *ptr;
    ptr = bts2_app_data->hfp_hf_ptr;

    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_hfp_hf;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
//        smsSendMessage(bts2_app_data);
    }
}
#endif
#ifdef CFG_HFP_AG

static void  bt_display_menu_hfp_ag(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##                  HFP AG Menu                     ##\n");
    printf("##   1. AG CONNNECT HF DEVICE                       ##\n");
    printf("##   2. AG DISCONNECT HF DEVICE                     ##\n");
    printf("##   3. Ag start connect audio                      ##\n");
    printf("##   4. AG stop  disconnect audio                   ##\n");
    printf("##   5. AG spk vol                                  ##\n");
    printf("##   6. AG mic vol                                  ##\n");
    printf("##   7. AG changed inband                           ##\n");
    printf("##   8. Voice Recognition                           ##\n");
    printf("##   9. stop AG service                             ##\n");
    printf("##   a. start AG service                            ##\n");
    printf("##   b. cind service update                         ##\n");
    printf("##   c. cind call   update                          ##\n");
    printf("##   d. cind callsetup update                       ##\n");
    printf("##   e. cind batt update                            ##\n");
    printf("##   f. cind signal update                          ##\n");
    printf("##   g. cind roam update                            ##\n");
    printf("##   h. cind callheld update                        ##\n");
    printf("##   i. send clip cmd                               ##\n");
    printf("##   j. dial a call                                 ##\n");
    printf("##   k. call active                                 ##\n");
    printf("##   l. no call                                     ##\n");
    printf("##   m. outgoing call dialing                       ##\n");
    printf("##   n. outgoing call alter                         ##\n");
    printf("##   o. send AT result                              ##\n");
    printf("##   p. send code id                                ##\n");
    printf("##   q. send ring                                   ##\n");
    printf("##   t. not auto response                           ##\n");
    printf("##   x. all indicator 1:call ative 2:hold & active  ##\n");
    printf("##   y. set sco retry                               ##\n");
    printf("##   s. Show ag Menu                                ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

static void bt_hdl_menu_hfp_ag(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {

    case '1':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);
        bt_hfp_connect_profile(&bd);
        break;
    }
    case '2':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);

        bt_hfp_disconnect_profile(&bd);
        break;
    }
    case '3':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);
        bt_hfp_connect_audio(&bd);
        break;
    }
    case '4':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);

        bt_hfp_disconnect_audio(&bd);
        break;
    }
    case '5':
    {
        int value = atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_ag_spk_vol_control(value);
        break;
    }
    case '6':
    {
        int value = atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_ag_mic_vol_control(value);
        break;
    }
    case '7':
    {
        int value = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_ag_set_inband(value);
        break;
    }
    case '8':
    {
        int value = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_ag_brva_response(value);
        break;
    }
    case '9':
    {
        bt_hfp_stop_profile_service(bts2_app_data);
        break;
    }
    case 'a':
    {
        bt_hfp_start_profile_service(bts2_app_data);
        break;
    }
    case 'b':
    {
        int value = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_ag_ind_status_update(HFP_AG_CIND_SERVICE_TYPE, value);
        break;
    }
    case 'c':
    {
        int value = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_ag_ind_status_update(HFP_AG_CIND_CALL_TYPE, value);
        break;
    }
    case 'd':
    {
        int value = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_ag_ind_status_update(HFP_AG_CIND_CALLSETUP_TYPE, value);
        break;
    }
    case 'e':
    {
        U8 value = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_ag_ind_status_update(HFP_AG_CIND_BATT_TYPE, value);
        break;
    }
    case 'f':
    {
        int value = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_ag_ind_status_update(HFP_AG_CIND_SIGNAL_TYPE, value);
        break;
    }

    case 'g':
    {
        U8 value = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_ag_ind_status_update(HFP_AG_CIND_ROAM_TYPE, value);
        break;
    }
    case 'h':
    {
        int value = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_ag_ind_status_update(HFP_AG_CIND_CALLHELD_TYPE, value);
        break;
    }
    case 'i':
    {
        hfp_phone_number_t remote_phone_num;
        char *str = "1234567";
        bmemcpy(&remote_phone_num.phone_number, str, strlen(str));
        remote_phone_num.type = 0x81;
        bt_hfp_ag_clip_response(&remote_phone_num);
        break;
    }
    case 'j':
    {
        HFP_CALL_INFO_T call_info;
        call_info.num_active = 0;
        call_info.num_held = 0;
        call_info.callsetup_state = 1;
        char *str = "18182307981";
        bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
        call_info.phone_type = 0x81;
        call_info.phone_len = strlen(str) + 1;
        bt_hfp_ag_app_call_status_change((char *)&call_info.phone_number, call_info.phone_len, call_info.num_active, call_info.callsetup_state);
        bt_hfp_ag_call_state_update_listener(&call_info);
        break;
    }
    case 'k':
    {
        HFP_CALL_INFO_T call_info;
        call_info.num_active = 1;
        call_info.num_held = 0;
        call_info.callsetup_state = 0;
        char *str = "18182307981";
        bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
        call_info.phone_type = 0x81;
        call_info.phone_len = strlen(str) + 1;
        bt_hfp_ag_app_call_status_change((char *)&call_info.phone_number, call_info.phone_len, call_info.num_active, call_info.callsetup_state);
        bt_hfp_ag_call_state_update_listener(&call_info);
        break;
    }
    case 'l':
    {
        HFP_CALL_INFO_T call_info;
        call_info.num_active = 0;
        call_info.num_held = 0;
        call_info.callsetup_state = 0;
        char *str = "18182307981";
        bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
        call_info.phone_type = 0x81;
        call_info.phone_len = strlen(str) + 1;
        bt_hfp_ag_app_call_status_change((char *)&call_info.phone_number, call_info.phone_len, call_info.num_active, call_info.callsetup_state);
        bt_hfp_ag_call_state_update_listener(&call_info);
        break;
    }
    case 'm':
    {
        HFP_CALL_INFO_T call_info;
        call_info.num_active = 0;
        call_info.num_held = 0;
        call_info.callsetup_state = 2;
        char *str = "18182307981";
        bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
        call_info.phone_type = 0x81;
        call_info.phone_len = strlen(str) + 1;
        bt_hfp_ag_app_call_status_change((char *)&call_info.phone_number, call_info.phone_len, call_info.num_active, call_info.callsetup_state);
        bt_hfp_ag_call_state_update_listener(&call_info);
        break;
    }
    case 'n':
    {
        HFP_CALL_INFO_T call_info;
        call_info.num_active = 0;
        call_info.num_held = 0;
        call_info.callsetup_state = 3;
        char *str = "18182307981";
        bmemcpy(&call_info.phone_number, str, strlen(str) + 1);
        call_info.phone_type = 0x81;
        call_info.phone_len = strlen(str) + 1;
        bt_hfp_ag_app_call_status_change((char *)&call_info.phone_number, call_info.phone_len, call_info.num_active, call_info.callsetup_state);
        bt_hfp_ag_call_state_update_listener(&call_info);
        break;
    }
    case 'o':
    {
        int value = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_ag_at_result_res(value);
        break;
    }
    case 'p':
    {
        int value = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        bt_hfp_ag_set_bcs(value);
        break;
    }

    case 'q':
    {
        extern void hfp_ag_app_send_ring();
        hfp_ag_app_send_ring();
        break;
    }
    case 't':
    {
#ifndef BT_USING_AG
        int value = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        extern U8 g_flag_auto_answer_call;
        g_flag_auto_answer_call = value;
#endif
        break;
    }
    case 'x':
    {
        int value = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        hfp_cind_status_t cind_status;
        cind_status.service_status = 1;
        cind_status.batt_level = 5;
        cind_status.signal = 3;
        cind_status.roam_status = 1;

        if (value == 1)
        {
            cind_status.call = 1;
            cind_status.callsetup = 0;
            cind_status.callheld = 0;
        }
        else if (value == 2)
        {
            cind_status.call = 1;
            cind_status.callsetup = 0;
            cind_status.callheld = 1;
        }
        bt_hfp_ag_cind_response(&cind_status);
        break;
    }
    case 'y':
    {
        extern void hfp_set_sco_retry_flag(U8 enable);
        int enable = (U8)atoi((const char *)bts2_app_data->input_str + 1);
        hfp_set_sco_retry_flag(enable);
        break;
    }

    case 's':
    {
        bt_display_menu_hfp_ag();
        break;
    }
    case 'r':
    {
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    }
    default:
    {
        break;
    }
    }
}
static void bt_disply_menu_hfp_hf_ag_1(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Dial Menu                              ##\n");
    printf("##   Input phone numbers (Digitals)&type            ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}
static void bt_hdl_menu_hfp_hf_ag_1(bts2_app_stru *bts2_app_data)
{

}
#endif

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
#if 0
static void disply_menu_hfp_hf_g(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           PhoneBook Download Menu                ##\n");
    printf("##   1. SIM card phonebook                          ##\n");
    printf("##   2. Mobile phone phonebook                      ##\n");
    printf("##   3. Last dialing list                           ##\n");
    printf("##   4. Miss calls list                             ##\n");
    printf("##   5. Received calls list                         ##\n");
    printf("##   n. Load all records (include 1 ~ 5)            ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void hdl_menu_hfp_hf_g(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
        break;
    case '2':
        break;
    case '3':
        break;
    case '4':
        break;
    case 's':
        disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_hfp_hf;
        disply_menu(bts2_app_data);
        break;
    default:
        break;
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
static void disply_menu_hfp_hf_h(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Short Message Menu                     ##\n");
    printf("##   0. List received unrd msgs-- REC UNRD          ##\n");
    printf("##   1. List received rd msgs  -- REC RD            ##\n");
    printf("##   2. List stored unsent msgs  -- STO UNSENT      ##\n");
    printf("##   3. List stored sent msgs    -- STO SENT        ##\n");
    printf("##   4. List all msg             -- ALL             ##\n");
    printf("##   5. Delete msg                                  ##\n");
    printf("##   6. Send msg                                    ##\n");
    printf("##   7. Set Receive Side Mobile Phone Number        ##\n");
#if 0
    printf("##   8. Set SIM card as Message Store Area          ##\n");
    printf("##   9. Set Mobile Phone as Message Store Area      ##\n");
#endif
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_hfp_hf_h(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
        break;
    case '2':
        break;
    case '3':
        break;
    case '4':
        break;
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_hfp_hf;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
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
static void bt_disply_menu_hfp_hf_h_5(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Delete Short Message Menu              ##\n");
    printf("##   Input the deleted msg idx (Digitals)           ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_hfp_hf_h_5(bts2_app_stru *bts2_app_data)
{
    if (bts2_app_data->input_str_len == 1 && bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_hfp_hf;
        bt_disply_menu(bts2_app_data);
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
static void bt_disply_menu_hfp_hf_h_6(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Send Short Message Menu                ##\n");
    printf("##   Input msg string except the first is 'r'       ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_hfp_hf_h_6(bts2_app_stru *bts2_app_data)
{
    if (bts2_app_data->input_str_len == 1 && bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_hfp_hf;
        bt_disply_menu(bts2_app_data);
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
static void bt_disply_menu_hfp_hf_h_7(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Set Receive Side Phone Number          ##\n");
    printf("##   Input phone number                             ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_hfp_hf_h_7(bts2_app_stru *bts2_app_data)
{
    if (bts2_app_data->input_str_len == 1 && bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_hfp_hf;
        bt_disply_menu(bts2_app_data);
    }
}
#endif

#ifdef CFG_BQB

void bt_disply_menu_l2cap_bqb(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##                  L2CAP BQB Menu                  ##\n");
    printf("##   1. disconnect l2cap channel                    ##\n");
    printf("##   2. l2cap config req                            ##\n");
    printf("##   3. l2cap connect req                           ##\n");
    printf("##   4. l2cap echo req                              ##\n");
    printf("##   5. l2cap data req                              ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}


extern void l2ca_disc_req(U16 cid);
extern void l2ca_cfg_req(U16 cid,
                         U16 option,
                         U16 incoming_mtu,
                         BTS2S_QOS_FLOW *outgoing_flow,
                         U16 outgoing_flush_timeout,
                         BOOL more);
extern void l2ca_conn_req(BTS2S_BD_ADDR *bd, U16 local_psm, U16 rmt_psm);
extern void l2ca_ping_req(BTS2S_BD_ADDR *bd, U8 *data, U16 len, U16 local_psm);
extern void l2ca_datawr_req(U16 cid, U16 len, void *data);

//extern void l2ca_get_info_req(BTS2S_BD_ADDR *bd, U16 info_type, U16 local_psm);

static void bt_hdl_menu_l2cap(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
    {
        l2ca_disc_req(0xff);//disc the first l2cap
        break;
    }

    case '2':
    {
        l2ca_cfg_req(0xff, 0, 0, NULL, 0, FALSE);
        break;
    }

    case '3':
    {
        BTS2S_BD_ADDR temp = {0xffffff, 0xff, 0xffff};

        if (0 != memcmp(&(bts2_app_data->bd_list[bts2_app_data->dev_idx]), &temp, sizeof(BTS2S_BD_ADDR)))
        {
            l2ca_conn_req(&(bts2_app_data->bd_list[bts2_app_data->dev_idx]), BT_PSM_SDP, BT_PSM_SDP);
        }
        else if (0 != memcmp(&(bts2_app_data->last_conn_bd), &temp, sizeof(BTS2S_BD_ADDR)))
        {
            l2ca_conn_req(&(bts2_app_data->last_conn_bd), BT_PSM_SDP, BT_PSM_SDP);
            bts2_app_data->last_conn_bd = temp;
        }

        break;
    }

    case '4':
    {
        BTS2S_BD_ADDR temp = {0xffffff, 0xff, 0xffff};

        U8 *tmp = bmalloc(10);
        bmemcpy(tmp, "echo test!", 10);

        if (0 != memcmp(&(bts2_app_data->bd_list[bts2_app_data->dev_idx]), &temp, sizeof(BTS2S_BD_ADDR)))
        {
            l2ca_ping_req(&(bts2_app_data->bd_list[bts2_app_data->dev_idx]), tmp, 10, BT_PSM_SDP);
        }
        else if (0 != memcmp(&(bts2_app_data->last_conn_bd), &temp, sizeof(BTS2S_BD_ADDR)))
        {
            l2ca_ping_req(&(bts2_app_data->last_conn_bd), tmp, 10, BT_PSM_SDP);
            bts2_app_data->last_conn_bd = temp;
        }


        break;
    }

    case '5':
    {
        U8 *tmp = bmalloc(48);
        memset(tmp, 0x01, 48);
        l2ca_datawr_req(0xff, 48, tmp);
        break;
    }

    case 'r':
    {
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    }
    default:
        break;
    }
}

#endif


#ifdef CFG_VDP_SNK
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
static void bt_disply_menu_vdp_snk(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           VDP Sink Menu                          ##\n");
    printf("##   1. VDP conn to the selected Source             ##\n");
    printf("##   2. VDP disc with Source                        ##\n");
    printf("##   3. Suspend vedio stream                        ##\n");
    printf("##   4. Start vedio stream                          ##\n");
    printf("##   5. Rlease vedio stream                         ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}
static void bt_hdl_menu_vdp_snk(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
        bt_vdpsnk_conn_2_src(bts2_app_data);
        break;
    case '2':
        bt_vdpsnk_disc_2_src(bts2_app_data);
        break;
    case '3':
        bt_vdpsnk_suspend_stream(bts2_app_data);
        break;
    case '4':
        bt_vdpsnk_start_stream(bts2_app_data);
        break;
    case '5':
        bt_vdpsnk_release_stream(bts2_app_data);
        break;
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
    }
}
#endif


#ifdef CFG_AV_SNK

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
static void bt_disply_menu_av_snk(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           AV Sink Menu                           ##\n");
    printf("##   1. AV conn to the selected Source              ##\n");
    printf("##   2. AV disc with Source                         ##\n");
    printf("##   3. Suspend audio stream                        ##\n");
    printf("##   4. Start audio stream                          ##\n");
    printf("##   5. Rlease audio stream                         ##\n");
    printf("##   6. enable a2dp                                 ##\n");
    printf("##   7. disable a2dp                                ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_av_snk(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {

    case '1':
        bt_avsnk_conn_2_src(&(bts2_app_data->last_conn_bd));
        break;
    case '2':
        bt_avsnk_disc_2_src(FALSE);
        break;
    case '3':
        bt_avsnk_suspend_stream(bts2_app_data);
        break;
    case '4':
        bt_avsnk_start_stream(bts2_app_data);
        break;
    case '5':
        bt_avsnk_release_stream(bts2_app_data);
        break;
    case '6':
        bt_av_snk_open();
        break;
    case '9':
        bt_av_open_stream();
        break;
    case '7':
        if (bt_av_conn_check())
        {
            bt_avsnk_disc_2_src(TRUE);
        }
        else
        {
            bt_av_snk_close();
        }
        break;
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
    }
}
#endif


#ifdef CFG_AVRCP

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
static void bt_disply_menu_avrcp(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##             AVRCP Menu                           ##\n");
    printf("##   4. Record                                      ##\n");
    printf("##   5. Rewind                                      ##\n");
    printf("##   6. Play                                        ##\n");
    printf("##   7. Stop                                        ##\n");
    printf("##   8. Pause                                       ##\n");
    printf("##   9. Forward                                     ##\n");
    printf("##   a. Backward                                    ##\n");
    printf("##   b. Speaker volume control                      ##\n");
    printf("##   c. AVRCP connect                               ##\n");
    printf("##   d. AVRCP disconnect                            ##\n");
    printf("##   e. AVRCP uint infomation                       ##\n");
    printf("##   f. AVRCP subunot information                   ##\n");
    printf("##   g. enable avrcp profile                        ##\n");
    printf("##   h. disable avrcp profile                       ##\n");
    printf("##   j. get play status                             ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_disply_menu_avrcp_a(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Speaker Volume Control Menu            ##\n");
    printf("##    Input Speaker volumes (0 ~ 100)               ##\n");
    printf("##   2. volume   up                                 ##\n");
    printf("##   3. volume   down                               ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_avrcp(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '4':
        bt_avrcp_record(bts2_app_data);
        break;
    case '5':
        bt_avrcp_rewind(bts2_app_data);
        break;
    case '6':
        bt_avrcp_ply(bts2_app_data);
        break;
    case '7':
        bt_avrcp_stop(bts2_app_data);
        break;
    case '8':
        bt_avrcp_pause(bts2_app_data);
        break;
    case '9':
        bt_avrcp_forward(bts2_app_data);
        break;
    case 'a':
        bt_avrcp_backward(bts2_app_data);
        break;
    case 'b':
        bts2_app_data->menu_id = menu_avrcp_a;
        bt_disply_menu_avrcp_a();
        break;
    case 'c':
        bt_avrcp_conn_2_dev(&(bts2_app_data->last_conn_bd), FALSE);
        break;
    case 'd':
        bt_avrcp_disc_2_dev(&(bts2_app_data->last_conn_bd));
        break;
    case 'e':
        bt_avrcp_unitinfo(bts2_app_data);
        break;
    case 'f':
        bt_avrcp_subunitinfo(bts2_app_data);
        break;
    case 'g':
        avrcp_enb_req(bts2_app_data->phdl, AVRCP_CT);
        break;
    case 'h':
        avrcp_disb_req();
        break;
    case 'j':
        bt_avrcp_get_play_status_request(bts2_app_data);
        break;
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
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
static void bt_hdl_menu_avrcp_a(bts2_app_stru *bts2_app_data)
{
    if (bts2_app_data->input_str_len == 1 && bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_avrcp;
        bt_disply_menu(bts2_app_data);
    }
    else if (bts2_app_data->input_str_len == 1 && bts2_app_data->input_str[0] == '2') //volume up
    {
        bt_avrcp_volume_up(bts2_app_data);
    }
    else if (bts2_app_data->input_str_len == 1 && bts2_app_data->input_str[0] == '3') //volume down
    {
        bt_avrcp_volume_down(bts2_app_data);
    }
    else  //set absolute volume
    {
        U8   volume;
        volume  = atoi((const char *)bts2_app_data->input_str);

        bt_avrcp_change_volume(bts2_app_data, volume);
    }
}
#endif


#ifdef CFG_HID
static void bt_disply_menu_hid(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##               HID Menu                           ##\n");
    printf("##   c. HID connect                                 ##\n");
    printf("##   D. HID disconnect                              ##\n");
    printf("##   g. HID enable                                 ##\n");
    printf("##   h. HID disable                                 ##\n");
    printf("##   w. HID mouse +Y_direction movement             ##\n");
    printf("##   a. HID mouse -X_direction movement             ##\n");
    printf("##   s. HID mouse -Y_direction movement             ##\n");
    printf("##   d. HID mouse +X_direction movement             ##\n");
    printf("##   L. control mobile left click                   ##\n");
    printf("##   R. control mobile right click                  ##\n");
    printf("##   2. control mobile drag up                      ##\n");
    printf("##   3. control mobile drag down                    ##\n");
    printf("##   4. control mobile double left click            ##\n");
    printf("##   5. control mobile double right click           ##\n");
    printf("##   m. set device IOS                              ##\n");
    printf("##   n. set device Android                          ##\n");
    printf("##   +. control mobile take a picture               ##\n");
    printf("##   S. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##   e. exit sniff mode                             ##\n");
    printf("##   b. enter sniff mode                            ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

static void bt_hdl_menu_hid(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case 'c':
        bt_hid_conn_2_dev(&(bts2_app_data->last_conn_bd));
        break;
    case 'D':
        bt_hid_disc_2_dev(&(bts2_app_data->last_conn_bd));
        break;
    case 'g':
        hid_enb_req(bts2_app_data->phdl, HID_Device);
        break;
    case 'h':
        hid_disb_req();
        break;
    case 'w':
    case 'a':
    case 's':
    case 'd':
    case 'L':
    case 'R':
    case 'O':
        bt_hid_mouse_test1(bts2_app_data);
        break;
    case '2':
        bt_hid_mouse_test2(bts2_app_data);
        break;
    case '3':
        bt_hid_mouse_test3(bts2_app_data);
        break;
    case '4':
        bt_hid_mouse_test4(bts2_app_data);
        break;
    case '5':
        bt_hid_mouse_test5(bts2_app_data);
        break;
    case '6':
        bt_hid_mouse_test6(bts2_app_data);
        break;
    case 'S':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    case 'm':
        bt_hid_mouse_test7(bts2_app_data);
        break;
    case 'n':
        bt_hid_mouse_test8(bts2_app_data);
        break;
    case '7':
    case '8':
    case '9':
    case '-':
    case '+':
        bt_hid_mouse_test9(bts2_app_data);
        break;
    case 'e':
        bt_exit_sniff_mode(bts2_app_data);
        break;
    case 'b':
        bt_etner_sniff_mode(bts2_app_data);
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

#ifdef CFG_VDP_SRC

static void bt_disply_menu_vdp_src(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           VDP Source Menu                        ##\n");
    printf("##   1. VDP conn to the selected Sink               ##\n");
    printf("##   2. VDP disc with Sink                          ##\n");
    printf("##   3. Suspend vedio stream                        ##\n");
    printf("##   4. Start vedio stream                          ##\n");
    printf("##   5. Release vedio stream                        ##\n");
    printf("##   6. Get configuration of the stream             ##\n");
    printf("##   7. Reconfigurate the audio stream              ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

static void bt_hdl_menu_vdp_src(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
        bt_vdpsrc_conn_2_snk(bts2_app_data);
        break;
    case '2':
        bt_vdpsrc_disc_2_snk(bts2_app_data);
        break;
    case '3':
        bt_vdpsrc_suspend_stream(bts2_app_data);
        break;
    case '4':
        bt_vdpsrc_start_stream(bts2_app_data);
        break;
    case '5':
        bt_vdpsrc_release_stream(bts2_app_data);
        break;
    case '6':
        bt_vdpsrc_get_cfg(bts2_app_data);
        break;
    case '7':
        bt_vdpsrc_recfg(bts2_app_data);
        break;
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
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
#ifdef CFG_AV_SRC

static void bt_disply_menu_av_src(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           AV Source Menu                         ##\n");
    printf("##   1. AV conn to the selected Sink                ##\n");
    printf("##   2. AV disc with Sink                           ##\n");
    printf("##   3. Suspend audio stream                        ##\n");
    printf("##   4. Start audio stream                          ##\n");
    printf("##   5. Release audio stream                        ##\n");
    printf("##   6. Get configuration of the stream             ##\n");
    printf("##   7. Reconfigurate the audio stream              ##\n");
    printf("##   8. AV SOURCE BQB TEST                          ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

static void bt_hdl_menu_av_src(bts2_app_stru *bts2_app_data)
{

    switch (bts2_app_data->input_str[0])
    {
    case '1':
        bt_avsrc_conn_2_snk(&(bts2_app_data->last_conn_bd));
        break;
    case '2':
        bt_avsrc_disc_2_snk(bts2_app_data);
        break;
    case '3':
        bt_avsrc_suspend_stream(bts2_app_data);
        break;
    case '4':
        bt_avsrc_start_stream(bts2_app_data);
        break;
    case '5':
        bt_avsrc_release_stream(bts2_app_data);
        break;
    case '6':
        bt_avsrc_get_cfg(bts2_app_data);
        break;
    case '7':
        bt_avsrc_recfg(bts2_app_data);
        break;
    case '8':
        bts2_app_data->menu_id = menu_av_src_bqb;
        bt_disply_menu(bts2_app_data);
        break;
    case '+':
        bt_interface_set_audio_device(0);
        break;
    case '-':
        bt_interface_set_audio_device(1);
        break;
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
    }
}

static void bt_disply_menu_av_src_bqb(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           AV Source BQB Menu                     ##\n");
    printf("##    Input BQB TEST NUMBER                         ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

static void bt_hdl_menu_av_src_bqb(bts2_app_stru *bts2_app_data)
{
    if (bts2_app_data->input_str_len == 1 && bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_av_src;
        bt_disply_menu(bts2_app_data);
    }
    else if (bts2_app_data->input_str_len == 1 && bts2_app_data->input_str[0] == 's')
    {
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        U8   bqb_num;
        bqb_num  = atoi((const char *)bts2_app_data->input_str);

        bt_interface_set_a2dp_bqb_test(bqb_num);
    }
}

#endif
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
#ifdef CFG_SPP_CLT
static void bt_disply_menu_spp_clt(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           SPP Client Menu                        ##\n");
    printf("##   0. Set SPP instance index                      ##\n");
    printf("##   1. SPP conn to the selected server             ##\n");
    printf("##   2. SPP disc with server                        ##\n");
    printf("##   3. Input send data                             ##\n");
    printf("##   4. Transfer a file                             ##\n");
    printf("##   5. Mode change                                 ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_spp_clt(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '0':
        bts2_app_data->menu_id = menu_spp_clt_0;
        bt_disply_menu(bts2_app_data);
        break;
    case '1':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);
        spp_clt_conn_req(&bd, 0xff, FALSE, NULL);
        break;
    }
    case '2':
        bt_spp_clt_disc_req(bts2_app_data);
        break;
    case '3':
        bts2_app_data->menu_id = menu_spp_clt_3;
        bt_disply_menu(bts2_app_data);
        break;
    case '4':
        bts2_app_data->menu_id = menu_spp_clt_4;
        bt_disply_menu(bts2_app_data);
        break;
    case '5':
        bts2_app_data->menu_id = menu_spp_clt_5;
        bt_disply_menu(bts2_app_data);
        break;
    case '6':
        // smsInit(bts2_app_data);
        break;
    case '7':
//          smsSendMessage(bts2_app_data);
        break;
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
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
static void bt_disply_menu_spp_clt_0(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Set SPP A index menu                   ##\n");
    printf("##   input index id(0 ~ %d)                         ##\n", (U8)SPP_CLT_MAX_CONN_NUM);
    printf("##   r. return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_spp_clt_0(bts2_app_stru *bts2_app_data)
{
    bts2_spp_inst_data *ptr;
    ptr = bts2_app_data->inst_ptr;

    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_spp_clt;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        bt_spp_clt_set_instance_index(bts2_app_data);
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
static void bt_disply_menu_spp_clt_3(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           SPP send data menu                     ##\n");
    printf("##   input send data                                ##\n");
    printf("##   r. return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_spp_clt_3(bts2_app_stru *bts2_app_data)
{
    bts2_spp_inst_data *ptr;
    ptr = bts2_app_data->inst_ptr;

    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_spp_clt;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        bt_spp_clt_sending_data_to_peer(bts2_app_data);
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
static void bt_disply_menu_spp_clt_4(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           SPP Send File Menu                     ##\n");
    printf("##   Input a file name                              ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_spp_clt_4(bts2_app_stru *bts2_app_data)
{
    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_spp_clt;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        bt_spp_clt_select_file_to_send(bts2_app_data);
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
static void bt_disply_menu_spp_clt_5(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           SPP mode change                        ##\n");
    printf("##   0. Exit PARK/SNIFF MODE                        ##\n");
    printf("##   1. HOLD_MODE                                   ##\n");
    printf("##   2. SNIFF_MODE                                  ##\n");
    printf("##   3. PARK_MODE                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_spp_clt_5(bts2_app_stru *bts2_app_data)
{
    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_spp_clt;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        switch (bts2_app_data->input_str[0])
        {
        case '0':
        // gap_exit_park_mode(&(bts2_app_data->pair_bd));
        //gap_exit_sniff_mode(&(bts2_app_data->pair_bd));
        case '1':
        case '2':
        case '3':
        {
            bt_spp_clt_mode_change_req(bts2_app_data);
            break;
        }
        default:
            printf(">> Input error\n");
            break;
        }
    }
}

#endif

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
#ifdef CFG_SPP_SRV
static void bt_disply_menu_spp_srv(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           SPP Server Menu                        ##\n");
    printf("##   0. Set SPP instance index                      ##\n");
    printf("##   1. Accept the remote side conn request         ##\n");
    printf("##   2. Reject the remote side conn request         ##\n");
    printf("##   3. Input send data                             ##\n");
    printf("##   4. Transfer a file                             ##\n");
    printf("##   5. Mode change                                 ##\n");
    printf("##   6. Rand send                                   ##\n");
    printf("##   7. spp disconnect                              ##\n");
    printf("##          Please specify device_id and srv_chl:   ##\n");
    printf("##          can get connect information by btskey d ##\n");
    printf("##          exp:btskey 7 0 2                        ##\n");
    printf("##              disconnect device 0 with channel 2  ##\n");
    printf("##   8. disconnect all spp connect                  ##\n");
    printf("##   9. received data is written into a file        ##\n");
    printf("##   x. received data is not written into a file    ##\n");
    printf("##   d. dump connection information                 ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_spp_srv(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '0':
    {
        bts2_app_data->menu_id = menu_spp_srv_0;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '1':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);

        U8 srv_chl = bts2_app_data->input_str[2] - '0';

        bt_spp_srv_rfc_conn_accept_hdl(bts2_app_data, srv_chl, bd);
        break;
    }
    case '2':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);

        U8 srv_chl = bts2_app_data->input_str[2] - '0';

        bt_spp_srv_rfc_conn_rej_hdl(bts2_app_data, srv_chl, bd);
        break;
    }
    case '3':
    {
        bts2_app_data->menu_id = menu_spp_srv_3;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '4':
    {
        bts2_app_data->menu_id = menu_spp_srv_4;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '5':
    {
        bts2_app_data->menu_id = menu_spp_srv_5;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '6':
    {
        bts2_app_data->menu_id = menu_spp_srv_6;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '7':
    {
        U8 device_id = 0xff;
        U8 srv_chl = 0xff;
        bts2_spp_srv_inst_data *sub_inst;

        device_id = bts2_app_data->input_str[1] - '0';
        srv_chl = bts2_app_data->input_str[2] - '0';

        rt_kprintf("device_id = %d,srv_chl = %d\n", device_id, srv_chl);

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
            bt_spp_srv_disc_req(bts2_app_data, bts2_app_data->select_device_id, bts2_app_data->select_srv_chnl);
        }
        break;
    }
    case '8':
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
        break;
    }
    case '9':
    {
        bt_spp_srv_set_write_into_file(bts2_app_data, 1);
        break;
    }
    case 'x':
    {
        bt_spp_srv_set_write_into_file(bts2_app_data, 0);
        break;
    }
    case 'd':
    {
        bt_spp_srv_dump_all_spp_connect_information(bts2_app_data);
        break;
    }
    case 's':
    {
        bt_disply_menu(bts2_app_data);
        break;
    }
    case 'r':
    {
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    }
    default:
    {
        break;
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
static void bt_disply_menu_spp_srv_0(bts2_app_stru *bts2_app_data)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Set SPP index menu                     ##\n");
    printf("##   input device id(On which device to send data?) ##\n");
    printf("##   device id(min 0 -- max %d)                     ##\n", CFG_MAX_ACL_CONN_NUM - 1);
    printf("##   device_id-addr-service_list                    ##\n");
    for (int i = 0; i < CFG_MAX_ACL_CONN_NUM; i++)
    {
        printf("##   %d -- %04lX:%02X:%06lX -- %lx                 ##\n", bts2_app_data->spp_srv_inst[i].device_id,
               (unsigned long)bts2_app_data->spp_srv_inst[i].bd_addr.nap, bts2_app_data->spp_srv_inst[i].bd_addr.uap, bts2_app_data->spp_srv_inst[i].bd_addr.lap, bts2_app_data->spp_srv_inst[i].service_list);
    }
    printf("##                                                  ##\n");
    printf("##   input service channel                          ##\n");
    printf("##   Is the index of bit with 1 in service_list     ##\n");
    printf("##                                                  ##\n");
    printf("##   r. return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_spp_srv_0(bts2_app_stru *bts2_app_data)
{
    bts2_spp_srv_inst_data *ptr;
    ptr = bts2_app_data->spp_srv_inst_ptr;

    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_spp_srv;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        bt_spp_srv_set_instance_index(bts2_app_data);
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
static void bt_disply_menu_spp_srv_3(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           SPP send data menu                     ##\n");
    printf("##   Please set spp instance index before:          ##\n");
    printf("##          Please specify device_id and srv_chl:   ##\n");
    printf("##          can get connect information by btskey d ##\n");
    printf("##          exp:btskey 0 2                          ##\n");
    printf("##            send data to device 0 with channel 2  ##\n");
    printf("##                                                  ##\n");
    printf("##   input send data                                ##\n");
    printf("##   r. return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_spp_srv_3(bts2_app_stru *bts2_app_data)
{
    bts2_spp_srv_inst_data *ptr;
    ptr = bts2_app_data->spp_srv_inst_ptr;

    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_spp_srv;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        U8 device_id = 0xff;
        U8 srv_chl = 0xff;

        device_id = bts2_app_data->input_str[0] - '0';
        srv_chl = bts2_app_data->input_str[1] - '0';
        bt_spp_srv_sending_data_to_peer(bts2_app_data, device_id, srv_chl);
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
static void bt_disply_menu_spp_srv_4(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           SPP Send File Menu                     ##\n");
    printf("##   Please set spp instance index before:          ##\n");
    printf("##          Please specify device_id and srv_chl:   ##\n");
    printf("##          can get connect information by btskey d ##\n");
    printf("##          exp:btskey 0 2                          ##\n");
    printf("##            send file to device 0 with channel 2  ##\n");
    printf("##                                                  ##\n");
    printf("##   Input a file name                              ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_hdl_menu_spp_srv_4(bts2_app_stru *bts2_app_data)
{
    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_spp_srv;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        U8 device_id = 0xff;
        U8 srv_chl = 0xff;

        device_id = bts2_app_data->input_str[0] - '0';
        srv_chl = bts2_app_data->input_str[1] - '0';
        bt_spp_srv_select_file_to_send(bts2_app_data, device_id, srv_chl);
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
static void bt_disply_menu_spp_srv_5(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           SPP mode change                        ##\n");
    printf("##   Please set spp instance index before:          ##\n");
    printf("##          Please specify device_id and srv_chl:   ##\n");
    printf("##          can get connect information by btskey d ##\n");
    printf("##          exp:btskey 0 2                          ##\n");
    printf("##          mode change to device 0 with channel 2  ##\n");
    printf("##                                                  ##\n");
    printf("##   0. Exit PARK/SNIFF MODE                        ##\n");
    printf("##   1. HOLD_MODE                                   ##\n");
    printf("##   2. SNIFF_MODE                                  ##\n");
    printf("##   3. PARK_MODE                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
static void bt_disply_menu_spp_srv_6(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           SPP Send 64K rand number Menu          ##\n");
    printf("##   Please set spp instance index before:          ##\n");
    printf("##          Please specify device_id and srv_chl:   ##\n");
    printf("##          can get connect information by btskey d ##\n");
    printf("##          exp:btskey 0 2                          ##\n");
    printf("##     send random data to device 0 with channel 2  ##\n");
    printf("##                                                  ##\n");
    printf("##   Input one packet size                          ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");

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
static void bt_hdl_menu_spp_srv_5(bts2_app_stru *bts2_app_data)
{
    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_spp_srv;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        switch (bts2_app_data->input_str[0])
        {
        case '0':
        // gap_exit_park_mode(&(bts2_app_data->pair_bd));
        //gap_exit_sniff_mode(&(bts2_app_data->pair_bd));
        case '1':
        case '2':
        case '3':
        {
            U8 device_id = 0xff;
            U8 srv_chl = 0xff;

            device_id = bts2_app_data->input_str[0] - '0';
            srv_chl = bts2_app_data->input_str[1] - '0';
            bt_spp_srv_mode_change_req(bts2_app_data, device_id, srv_chl, (U8)(bts2_app_data->input_str[0] - '0'));
            break;
        }
        default:
            printf(">> Input error\n");
            break;
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
static void bt_hdl_menu_spp_srv_6(bts2_app_stru *bts2_app_data)
{
    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_spp_srv;
        bt_disply_menu(bts2_app_data);
    }
    else
    {

        U8 device_id = 0xff;
        U8 srv_chl = 0xff;

        device_id = bts2_app_data->input_str[0] - '0';
        srv_chl = bts2_app_data->input_str[1] - '0';
        bt_spp_srv_sending_random_data(bts2_app_data, device_id, srv_chl);
    }
}


#endif

#ifdef CFG_PBAP_CLT
static void bt_disply_menu_pbap_clt(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           PBAP Client Menu                       ##\n");
    printf("##   1. Initiate a connection                       ##\n");
    printf("##   2. Disconnect a connection                     ##\n");
    printf("##   3. Goto local phone book                       ##\n");
    printf("##   4. Goto SIM1 phone book                        ##\n");
    printf("##   5. Pull phonebook                              ##\n");
    printf("##   6. Pull vcard entry                            ##\n");
    printf("##   7. Pull vcard list                             ##\n");
    printf("##   8. Abort current operation                     ##\n");
    printf("##   9. Initial connect with authentication         ##\n");
    printf("##   s. Show Menu                                   ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

static void bt_display_menu_pbap_clt_3()
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           PBAP Set phone book Menu               ##\n");
    printf("##   1. Goto main phone book                        ##\n");
    printf("##   2. Goto incoming calls history                 ##\n");
    printf("##   3. Goto outgoing calls history                 ##\n");
    printf("##   4. Goto missed calls history                   ##\n");
    printf("##   5. Goto combined calls history                 ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

static void bt_display_menu_pbap_clt_6()
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           PBAP Get phone book entry              ##\n");
    printf("##   Please input the phone book entry number       ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

static void bt_hdl_menu_pbap_clt(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);

        bt_pbap_client_connect(&bd, FALSE);
        break;
    }
    case '2':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);
        bt_pbap_client_disconnect(&bd);
        break;
    }
    case '3':
    {
        bts2_app_data->menu_id = menu_pbap_c_3;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '4':
    {
        bts2_app_data->menu_id = menu_pbap_c_4;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '5':
    {
        bt_pbap_client_pull_pb(0, 1, 0x00);
        break;
    }
    case '6':
    {
        bts2_app_data->menu_id = menu_pbap_c_6;
        bt_disply_menu(bts2_app_data);
        break;
    }
    case '7':
    {
        // bt_pbap_client_pull_vcard_list(bts2_app_data);
        char *str = "10086";
        bt_pbap_client_get_name_by_number(str, strlen(str));
        break;
    }
    case '8':
    {
        pbap_clt_abort_req();
        break;
    }
    case '9':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);

        bt_pbap_client_connect(&bd, TRUE);
        break;
    }
    case 's':
    {
        bt_disply_menu(bts2_app_data);
        break;
    }
    case 'r':
    {
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    }
    default:
        break;
    }
}

static void bt_display_menu_pbap_clt_1(void)
{
    printf("-- Please input the authentication password:>");
}

static void bt_hdl_menu_pbap_clt_1(bts2_app_stru *bts2_app_data)
{
    bt_pbap_client_auth(bts2_app_data->input_str, bts2_app_data->input_str_len);
    bts2_app_data->menu_id = menu_pbap_c;
}

static void bt_hdl_menu_pbap_clt_3(bts2_app_stru *bts2_app_data)
{
    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_pbap_c;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        if (bts2_app_data->input_str[0] >= '1' && bts2_app_data->input_str[0] <= '5')
        {
            if (bts2_app_data->menu_id == menu_pbap_c_3)
                bt_pbap_client_set_pb(PBAP_LOCAL, bts2_app_data->input_str[0] - '0');
            else
                bt_pbap_client_set_pb(PBAP_SIM1, bts2_app_data->input_str[0] - '0');
        }
        else
        {
            USER_TRACE(">> Invalid input\n");
        }
    }
}

static void bt_hdl_menu_pbap_clt_6(bts2_app_stru *bts2_app_data)
{
    if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_pbap_c;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        bts2_app_data->pin_code_len = strlen((const char *)bts2_app_data->input_str);
        bt_pbap_client_pull_vcard(bts2_app_data->input_str, bts2_app_data->pin_code_len);
    }
}

#endif

#ifdef CFG_BR_GATT_SRV
static void  bt_display_menu_bt_gatt_srv(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##                  bt_gatt Menu                    ##\n");
    printf("##   1. bt_gatt connect device                      ##\n");
    printf("##   2. bt_gatt disconnect device                   ##\n");
    printf("##   3. bt_gatt send_data                           ##\n");
    printf("##   4. bt_gatt change_mtu                          ##\n");
    printf("##   5. bt_gatt sdp_reg_gatt                        ##\n");
    printf("##   s. Show ag Menu                                ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

//U8 payload1 [] ={0x09, 0x07, 0x02, 0x00, 0x02, 0x03, 0x00, 0x00, 0x2a, 0x04, 0x00, 0x02, 0x05, 0x00, 0x01, 0x2a, 0x06, 0x00, 0x02, 0x07, 0x00, 0xa6, 0x2a};
U8 att_uuid = 0xff;

static void bt_hdl_menu_bt_gatt_srv(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);

        bt_gatt_conn_req(&bd);
        break;
    }
    case '2':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);

        bt_gatt_disconn_req(&bd);
        break;
    }
    case '3':
    {
        bt_gatt_send_data_req((char *)bts2_app_data->input_str + 2, bts2_app_data->input_str_len - 2);
        break;
    }
    break;
    case '4':
        bt_gatt_change_mtu_req(512);
        break;
    case '5':
        bt_gatt_create_sdp_record(1, &att_uuid, 01, 02);
        break;
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
    }

}
#endif

#ifdef CFG_BT_DIS
static void  bt_display_menu_bt_dis_srv(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##                  bt_dis Menu                     ##\n");
    printf("##   1. bt_dis sdp reg info                         ##\n");
    printf("##   2. bt_dis sdp unreg info                       ##\n");
    printf("##   s. Show bt_dis Menu                            ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

//U8 payload1 [] ={0x09, 0x07, 0x02, 0x00, 0x02, 0x03, 0x00, 0x00, 0x2a, 0x04, 0x00, 0x02, 0x05, 0x00, 0x01, 0x2a, 0x06, 0x00, 0x02, 0x07, 0x00, 0xa6, 0x2a};

static void bt_hdl_menu_bt_dis_srv(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
    {
        bt_dis_app_sdp_reg();
        break;
    }
    case '2':
    {
        bt_dis_app_sdp_unreg();
        break;
    }
    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
    }

}
#endif

#ifdef CFG_BT_L2CAP_PROFILE
static void  bt_display_menu_bt_l2cap_profile(void)
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           bt_l2cap_profile Menu                  ##\n");
    printf("##   1. bt_l2cap_profile  reg req                   ##\n");
    printf("##   2. bt_l2cap_profile  unreg req                 ##\n");
    printf("##   3. bt_l2cap_profile  conn req                  ##\n");
    printf("##   4. bt_l2cap_profile  discon req                ##\n");
    printf("##   5. bt_l2cap_profile  send data                 ##\n");
    printf("##   6. bt_l2cap_profile  sco req                   ##\n");
    printf("##   7. bt_l2cap_profile  sco discon req            ##\n");
    printf("##   8. bt_l2cap_profile  sco req res               ##\n");
    printf("##   s. Show bt_dis Menu                            ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
}

//U8 payload1 [] ={0x09, 0x07, 0x02, 0x00, 0x02, 0x03, 0x00, 0x00, 0x2a, 0x04, 0x00, 0x02, 0x05, 0x00, 0x01, 0x2a, 0x06, 0x00, 0x02, 0x07, 0x00, 0xa6, 0x2a};

static void bt_hdl_menu_bt_l2cap_profile(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
    {
        bt_l2cap_profile_app_reg_service(0xffee, 1);
        break;
    }
    case '2':
    {
        bt_l2cap_profile_app_unreg_service(0xffee);
        break;
    }
    case '3':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);

        bt_l2cap_profile_app_connect_req(&bd, 0xffee, 0xffee);
        break;
    }
    case '4':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);

        bt_l2cap_profile_app_disconnect_req(&bd, 0xffee);
        break;
    }
    case '5':
    {
        char *data = "data";
        int val = atoi((const char *)bts2_app_data->input_str + 1);
        bt_l2cap_profile_app_send_data_req(val, strlen(data), data);
        break;
    }
    case '6':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);
        bt_l2cap_profile_app_connect_sco_req(&bd);
        break;
    }
    case '7':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 1, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);
        bt_l2cap_profile_app_disconnect_sco_req(&bd);

        break;
    }
    case '8':
    {
        BTS2S_BD_ADDR bd;
        int reuslt = 0;
        uint32_t mac[6];
        reuslt = sscanf((const char *)bts2_app_data->input_str + 2, "%02X%02X%02X%02X%02X%02X", \
                        &mac[0], &mac[1], \
                        &mac[2], &mac[3], \
                        &mac[4], &mac[5]);
        bd.lap = (mac[3] << 16) | (mac[4] << 8) | mac[5];
        bd.uap = (U8)mac[2];
        bd.nap = (U16)((mac[0] << 8) | mac[1]);


        int val = atoi((const char *)bts2_app_data->input_str + 1);

        hcia_sync_conn_res(&bd, val, BT_SYNC_S4_TX_BANDWIDTH,
                           BT_SYNC_S4_RX_BANDWIDTH,
                           BT_SYNC_S4_MAX_LATENCY,
                           BT_SYNC_S4_VOICE_SETTING,
                           BT_SYNC_S4_RE_TX_EFFORT,
                           BT_SYNC_S4_PACKET_TYPE,
                           BT_HCI_CONN_REJED_DUE_TO_LIMITED_RESRCS);
        break;
    }

    case 's':
        bt_disply_menu(bts2_app_data);
        break;
    case 'r':
        bts2_app_data->menu_id = menu_main;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        break;
    }

}
#endif

static void bt_disply_menu_sc_input(void)
{
    printf(">> Please input your secrety value:\n");
}
static void bt_disply_menu_sc_yesno(void)
{
    printf(">> Please select your [Y|N] items:\n");
}

static void bt_disply_menu_sc_oobdata(void)
{
    printf(">> Please select your enter oobdata:\n");
}


static void bt_disply_menu_sc_pair()
{
    printf("\n");
    printf("######################################################\n");
    printf("##                                                  ##\n");
    printf("##           Input PinCode Menu                     ##\n");
    printf("##   1. Use default Pin Code                        ##\n");
    printf("##   2. Cancel              	                    ##\n");
    printf("##   r. Return to last menu                         ##\n");
    printf("##                                                  ##\n");
    printf("######################################################\n");
    printf("\n");
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
void bt_hdl_sc_pincode(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '1':
        sc_passkey_rsp(TRUE, &(bts2_app_data->pair_bd), bts2_app_data->pin_code_len, bts2_app_data->pin_code, TRUE, TRUE);
        INFO_TRACE(">> response pincode indication with: %s\n", bts2_app_data->pin_code);
        break;
    case '2':
        sc_passkey_rsp(FALSE, &(bts2_app_data->pair_bd), 0, bts2_app_data->input_str, TRUE, TRUE);
        INFO_TRACE(">> reject pin code indication\n");
        break;
    case 'r':
        bts2_app_data->menu_id = menu_gen_4;
        bt_disply_menu(bts2_app_data);
        break;
    default:
        printf(">> input err...\n");
        break;
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
void bt_hdl_menu(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->menu_id)
    {
    case menu_main:
    {
        bt_hdl_menu_main(bts2_app_data);
        break;
    }

    case menu_gen:
    {
        bt_hdl_menu_gen(bts2_app_data);
        break;
    }
    case menu_gen_3:
    {
        bt_hdl_menu_gen_3(bts2_app_data);
        break;
    }
    case menu_gen_4:
    {
        bt_hdl_menu_gen_4(bts2_app_data);
        break;
    }
    case menu_gen_4_3:
    {
        bt_hdl_menu_gen_4_3(bts2_app_data);
        break;
    }
    case menu_gen_5:
    {
        bt_hdl_menu_gen_5(bts2_app_data);
        break;
    }
    case menu_gen_5_1:
    {
        bt_hdl_menu_gen_5_1(bts2_app_data);
        break;
    }
    case menu_gen_5_4:
    {
        bt_hdl_menu_gen_5_4(bts2_app_data);
        break;
    }
    case menu_gen_5_7:
    {
        bt_hdl_menu_gen_5_7(bts2_app_data);
        break;
    }
    case menu_gen_5_8:
    {
        bt_hdl_menu_gen_5_8(bts2_app_data);
        break;
    }
    case menu_gen_6:
    {
        bt_hdl_menu_gen_6(bts2_app_data);
        break;
    }
    case menu_gen_6_1:
    {
        bt_hdl_menu_gen_6_1(bts2_app_data);
        break;
    }
    case menu_gen_6_2:
    {
        bt_hdl_menu_gen_6_2(bts2_app_data);
        break;
    }
    case menu_gen_6_3:
    {
        bt_hdl_menu_gen_6_3(bts2_app_data);
        break;
    }
    case menu_gen_7:
    {
        bt_hdl_menu_gen_7(bts2_app_data);
        break;
    }
    case menu_gen_8:
    {
        bt_hdl_menu_gen_8(bts2_app_data);
        break;
    }
    case menu_gen_8_1:
    {
        bt_hdl_menu_gen_8_1(bts2_app_data);
        break;
    }
    case menu_gen_8_2:
    {
        bt_hdl_menu_gen_8_2(bts2_app_data);
        break;
    }
    case menu_gen_8_4:
    {
        bt_hdl_menu_gen_8_4(bts2_app_data);
        break;
    }
    case menu_gen_8_5:
    {
        bt_hdl_menu_gen_8_5(bts2_app_data);
        break;
    }
    case menu_gen_8_7:
    {
        bt_hdl_menu_gen_8_7(bts2_app_data);
        break;
    }
    case menu_gen_8_8:
    {
        bt_hdl_menu_gen_8_8(bts2_app_data);
        break;
    }
    case menu_gen_8_9:
    {
        bt_hdl_menu_gen_8_9(bts2_app_data);
        break;
    }
    case menu_gen_9:
    {
        bt_hdl_menu_gen_9(bts2_app_data);
        break;
    }
    case menu_gen_a:
    {
        bt_hdl_menu_gen_a(bts2_app_data);
        break;
    }
    case menu_gen_b:
    {
        bt_set_secu_mode(bts2_app_data);
        break;
    }

#ifdef CFG_SPP_CLT
    case menu_spp_clt:
    {
        bt_hdl_menu_spp_clt(bts2_app_data);
        break;
    }
    case menu_spp_clt_0:
    {
        bt_hdl_menu_spp_clt_0(bts2_app_data);
        break;
    }
    case menu_spp_clt_3:
    {
        bt_hdl_menu_spp_clt_3(bts2_app_data);
        break;
    }
    case menu_spp_clt_4:
    {
        bt_hdl_menu_spp_clt_4(bts2_app_data);
        break;
    }
    case menu_spp_clt_5:
    {
        bt_hdl_menu_spp_clt_5(bts2_app_data);
        break;
    }
#endif

#ifdef CFG_SPP_SRV
    case menu_spp_srv:
    {
        bt_hdl_menu_spp_srv(bts2_app_data);
        break;
    }
    case menu_spp_srv_0:
    {
        bt_hdl_menu_spp_srv_0(bts2_app_data);
        break;
    }
    case menu_spp_srv_3:
    {
        bt_hdl_menu_spp_srv_3(bts2_app_data);
        break;
    }
    case menu_spp_srv_4:
    {
        bt_hdl_menu_spp_srv_4(bts2_app_data);
        break;
    }
    case menu_spp_srv_5:
    {
        bt_hdl_menu_spp_srv_5(bts2_app_data);
        break;
    }
    case menu_spp_srv_6:
    {
        bt_hdl_menu_spp_srv_6(bts2_app_data);
        break;
    }
#endif

#ifdef BT_FINSH_PAN
    case menu_pan_g:
    {
        bt_hdl_menu_pan(bts2_app_data);
        break;
    }
#endif
#ifdef CFG_HFP_HF
    case menu_hfp_hf:
    {
        bt_hdl_menu_hfp_hf(bts2_app_data);
        break;
    }

    case menu_hfp_hf_5:
    {
        bt_hdl_menu_hfp_hf_5(bts2_app_data);
        break;
    }

    case menu_hfp_hf_a:
    {
        bt_hdl_menu_hfp_hf_a(bts2_app_data);
        break;
    }
    case menu_hfp_hf_a_1:
    {
        bt_hdl_menu_hfp_hf_a_1(bts2_app_data);
        break;
    }
    case menu_hfp_hf_a_2:
    {
        bt_hdl_menu_hfp_hf_a_2(bts2_app_data);
        break;
    }
    case menu_hfp_hf_h:
    {
        bt_hdl_menu_hfp_hf_h(bts2_app_data);
        break;
    }
    case menu_hfp_hf_l:
    {
        bt_hdl_menu_hfp_hf_l(bts2_app_data);
        break;
    }
    case menu_hfp_hf_p:
    {
        bt_hdl_menu_hfp_hf_p(bts2_app_data);
        break;
    }
#endif

#ifdef CFG_BQB
    case menu_l2cap_bqb:
    {
        bt_hdl_menu_l2cap(bts2_app_data);
        break;
    }
#endif

#ifdef CFG_AV_SNK
    case menu_av_snk:
    {
        bt_hdl_menu_av_snk(bts2_app_data);
        break;
    }
#endif

#ifdef CFG_AV_SRC
    case menu_av_src:
    {
        bt_hdl_menu_av_src(bts2_app_data);
        break;
    }
    case menu_av_src_bqb:
    {
        bt_hdl_menu_av_src_bqb(bts2_app_data);
        break;
    }
#endif

#ifdef CFG_VDP_SNK
    case menu_vdp_snk:
    {
        bt_hdl_menu_vdp_snk(bts2_app_data);
        break;
    }
#endif

#ifdef CFG_AVRCP
    case menu_avrcp:
    {
        bt_hdl_menu_avrcp(bts2_app_data);
        break;
    }
    case menu_avrcp_a:
    {
        bt_hdl_menu_avrcp_a(bts2_app_data);
        break;
    }

#endif

#ifdef CFG_HFP_AG
    case menu_hfp_ag:
    {
        bt_hdl_menu_hfp_ag(bts2_app_data);
        break;
    }
#endif
#ifdef CFG_BR_GATT_SRV
    case menu_bt_gatt_srv:
    {
        bt_hdl_menu_bt_gatt_srv(bts2_app_data);
        break;
    }
#endif

#ifdef CFG_BT_DIS
    case menu_bt_dis:
    {
        bt_hdl_menu_bt_dis_srv(bts2_app_data);
        break;
    }
#endif
#ifdef CFG_BT_L2CAP_PROFILE
    case menu_bt_l2cap_profile:
    {
        bt_hdl_menu_bt_l2cap_profile(bts2_app_data);
        break;
    }
#endif
#ifdef CFG_HID
    case menu_hid:
    {
        bt_hdl_menu_hid(bts2_app_data);
        break;
    }
#endif

#ifdef CFG_VDP_SRC
    case menu_vdp_src:
    {
        bt_hdl_menu_vdp_src(bts2_app_data);
        break;
    }
#endif

#ifdef CFG_PBAP_CLT
    case menu_pbap_c:
    {
        bt_hdl_menu_pbap_clt(bts2_app_data);
        break;
    }
    case menu_pbap_c_1:
    {
        bt_hdl_menu_pbap_clt_1(bts2_app_data);
        break;
    }
    case menu_pbap_c_3:
    case menu_pbap_c_4:
    {
        bt_hdl_menu_pbap_clt_3(bts2_app_data);
        break;
    }
    case menu_pbap_c_6:
    {
        bt_hdl_menu_pbap_clt_6(bts2_app_data);
        break;
    }
#endif

    case menu_sc_passkey_notifi:
    {
        bt_hdl_sc_passkey_notifi(bts2_app_data);
        break;
    }
    case menu_sc_input:
    {
        bt_hdl_sc_input(bts2_app_data);
        break;
    }
    case menu_sc_yesno:
    {
        bt_hdl_sc_yesno(bts2_app_data);
        break;
    }
    case menu_sc_oobdata:
    {
        bt_hdl_sc_oobdata(bts2_app_data);
        break;
    }
    case menu_sc_pair:
    {
        bt_hdl_sc_pincode(bts2_app_data);
        break;
    }
    default:
        break;
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
void bt_disply_menu(bts2_app_stru *bts2_app_data)
{
#ifndef RT_USING_BT
    switch (bts2_app_data->menu_id)
    {
    case menu_start:
    {
        break;
    }

    case menu_main:
    {
        bt_disply_menu_main();
        break;
    }

    case menu_gen:
    {
        bt_disply_menu_gen();
        break;
    }
    case menu_gen_3:
    {
        bt_disply_menu_gen_3();
        break;
    }
    case menu_gen_4:
    {
        bt_disply_menu_gen_4();
        break;
    }
    case menu_gen_4_3:
    {
        bt_disply_menu_gen_4_3();
        break;
    }
    case menu_gen_5:
    {
        bt_disply_menu_gen_5();
        break;
    }
    case menu_gen_5_1:
    {
        bt_disply_menu_gen_5_1();
        break;
    }
    case menu_gen_5_4:
    {
        bt_disply_menu_gen_5_4();
        break;
    }
    case menu_gen_5_7:
    {
        bt_disply_menu_gen_5_7();
        break;
    }
    case menu_gen_5_8:
    {
        bt_disply_menu_gen_5_8();
        break;
    }
    case menu_gen_6:
    {
        bt_disply_menu_gen_6();
        break;
    }
    case menu_gen_6_1:
    {
        bt_disply_menu_gen_6_1();
        break;
    }
    case menu_gen_6_2:
    {
        bt_disply_menu_gen_6_2();
        break;
    }
    case menu_gen_6_3:
    {
        bt_disply_menu_gen_6_3();
        break;
    }
    case menu_gen_7:
    {
        bt_disply_menu_gen_7();
        break;
    }
    case menu_gen_8:
    {
        bt_disply_menu_gen_8();
        break;
    }
    case menu_gen_8_1:
    {
        bt_disply_menu_gen_8_1();
        break;
    }
    case menu_gen_8_2:
    {
        bt_disply_menu_gen_8_2();
        break;
    }
    case menu_gen_8_4:
    {
        bt_disply_menu_gen_8_4();
        break;
    }
    case menu_gen_8_5:
    {
        bt_disply_menu_gen_8_5();
        break;
    }
    case menu_gen_8_7:
    {
        bt_disply_menu_gen_8_7();
        break;
    }

    case menu_gen_8_8:
    {
        bt_disply_menu_gen_8_8();
        break;
    }
    case menu_gen_8_9:
    {
        bt_disply_menu_gen_8_9();
        break;
    }
    case menu_gen_9:
    {
        bt_disply_menu_gen_9();
        break;
    }
    case menu_gen_a:
    {
        bt_disply_menu_gen_a();
        break;
    }
    case menu_gen_b:
    {
        bt_disply_menu_cur_sec_mode();
        break;
    }

#ifdef CFG_SPP_CLT
    case menu_spp_clt:
    {
        bt_disply_menu_spp_clt();
        break;
    }
    case menu_spp_clt_0:
    {
        bt_disply_menu_spp_clt_0();
        break;
    }
    case menu_spp_clt_3:
    {
        bt_disply_menu_spp_clt_3();
        break;
    }
    case menu_spp_clt_4:
    {
        bt_disply_menu_spp_clt_4();
        break;
    }
    case menu_spp_clt_5:
    {
        bt_disply_menu_spp_clt_5();
        break;
    }
#endif
#ifdef CFG_SPP_SRV
    case menu_spp_srv:
    {
        bt_disply_menu_spp_srv();
        break;
    }
    case menu_spp_srv_0:
    {
        bt_disply_menu_spp_srv_0(bts2_app_data);
        break;
    }
    case menu_spp_srv_3:
    {
        bt_disply_menu_spp_srv_3();
        break;
    }
    case menu_spp_srv_4:
    {
        bt_disply_menu_spp_srv_4();
        break;
    }
    case menu_spp_srv_5:
    {
        bt_disply_menu_spp_srv_5();
        break;
    }
    case menu_spp_srv_6:
    {
        bt_disply_menu_spp_srv_6();
        break;
    }
#endif
#ifdef BT_FINSH_PAN
    case menu_pan_g:
    {
        bt_disply_menu_pan();
        break;
    }
#endif

#ifdef CFG_HFP_HF
    case menu_hfp_hf:
    {
        bt_disply_menu_hfp_hf();
        break;
    }

    case menu_hfp_hf_5:
    {
        bt_disply_menu_hfp_hf_5();
        break;
    }

    case menu_hfp_hf_a:
    {
        bt_disply_menu_hfp_hf_a();
        break;
    }
    case menu_hfp_hf_a_1:
    {
        bt_disply_menu_hfp_hf_a_1();
        break;
    }
    case menu_hfp_hf_a_2:
    {
        bt_disply_menu_hfp_hf_a_2();
        break;
    }

    case menu_hfp_hf_h:
    {
        bt_disply_menu_hfp_hf_h();
        break;
    }
    //add 20220728
    case menu_hfp_hf_l:
    {
        bt_disply_menu_hfp_hf_l();
        break;
    }

    case menu_hfp_hf_p:
    {
        bt_display_menu_hfp_hf_p();
        break;
    }
#endif

#ifdef CFG_HFP_AG
    case menu_hfp_ag:
    {
        bt_display_menu_hfp_ag();
        break;
    }
#endif

#ifdef CFG_AV_SNK
    case menu_av_snk:
    {
        bt_disply_menu_av_snk();
        break;
    }
#endif

#ifdef CFG_AV_SRC
    case menu_av_src:
    {
        bt_disply_menu_av_src();
        break;
    }
    case menu_av_src_bqb:
    {
        bt_disply_menu_av_src_bqb();
        break;
    }
#endif
#ifdef CFG_VDP_SNK
    case menu_vdp_snk:
    {
        bt_disply_menu_vdp_snk();
        break;
    }
#endif

#ifdef CFG_VDP_SRC
    case menu_vdp_src:
    {
        bt_disply_menu_vdp_src();
        break;
    }
#endif

#ifdef CFG_AVRCP
    case menu_avrcp:
    {
        bt_disply_menu_avrcp();
        break;
    }
#endif

#ifdef CFG_HID
    case menu_hid:
    {
        bt_disply_menu_hid();
        break;
    }
#endif

#ifdef CFG_BQB
    case menu_l2cap_bqb:
    {
        bt_disply_menu_l2cap_bqb();
        break;
    }
#endif


        /*****************               PBAP CLIENT             **************/
#ifdef CFG_PBAP_CLT
    case menu_pbap_c:
    {
        bt_disply_menu_pbap_clt();
        break;
    }
    case menu_pbap_c_1:
    {
        bt_display_menu_pbap_clt_1();
        break;
    }
    case menu_pbap_c_3:
    case menu_pbap_c_4:
    {
        bt_display_menu_pbap_clt_3();
        break;
    }
    case menu_pbap_c_6:
    {
        bt_display_menu_pbap_clt_6();
        break;
    }
#endif
#ifdef CFG_BR_GATT_SRV
    case menu_bt_gatt_srv:
    {
        bt_display_menu_bt_gatt_srv();
        break;
    }
#endif

#ifdef CFG_BT_DIS
    case menu_bt_dis:
    {
        bt_display_menu_bt_dis_srv();
        break;
    }
#endif

#ifdef CFG_BT_L2CAP_PROFILE
    case menu_bt_l2cap_profile:
    {
        bt_display_menu_bt_l2cap_profile();
        break;
    }
#endif
    case menu_sc_input:
    {
        bt_disply_menu_sc_input();
        break;
    }

    case menu_sc_yesno:
    {
        bt_disply_menu_sc_yesno();
        break;
    }

    case menu_sc_oobdata:
    {
        bt_disply_menu_sc_oobdata();
        break;
    }
    case menu_sc_pair:
    {
        bt_disply_menu_sc_pair();
        break;
    }
    default:
    {
        break;
    }
    }
#endif
}
#endif //BTS2_APP_MENU
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
