/**
  ******************************************************************************
  * @file   bts2_app_generic.c
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
#include "bf0_ble_common.h"
#include "bf0_ble_err.h"
#include "bf0_sibles.h"

#define LOG_TAG         "btapp_ge"
#include "log.h"

#ifdef RT_USING_BT
    #include "bt_rt_device.h"
#endif


#ifdef  CFG_MS
    #include <Windows.h>
#endif

extern BTS2S_ETHER_ADDR bd2etheraddr(const BTS2S_BD_ADDR *bd);

#define CTRLLER_NAME_LEN (HCI_NAME_LEN + 1)

#ifdef CFG_MS
    static HANDLE kb_thrd;
    DWORD thrd_id;
#endif


extern U8 bts2s_local_hash_c[16];
extern U8 bts2s_local_rand_r[16];
#ifdef BT_FINSH_PAN
    //add for tcpip route
    BTS2S_ETHER_ADDR   bts2_local_ether_addr;
    //BTS2S_ETHER_ADDR   bts2_remote_ether_addr;
#endif
extern bts2_app_stru *bts2g_app_p;
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
void bt_pincode_indi(bts2_app_stru *bts2_app_data)
{
    if (bts2_app_data->pin_code_len == 0)
    {
        INFO_TRACE(">> Reject pin code indication\n");
        sc_passkey_rsp(FALSE, &(bts2_app_data->pair_bd), 0, bts2_app_data->input_str, TRUE, TRUE);
    }
    else
    {
        INFO_TRACE(">> Response pincode indication with: %s\n", bts2_app_data->pin_code);
        sc_passkey_rsp(TRUE, &(bts2_app_data->pair_bd), bts2_app_data->pin_code_len, bts2_app_data->pin_code, TRUE, TRUE);
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
void bt_start_inquiry(bts2_app_stru *bts2_app_data)
{
    BTS2S_CPL_FILTER inq_filter;
    U8 i = 0;

    inq_filter.filter = BTS2_INQ_FILTER_CLEAR;
    inq_filter.dev_mask_cls = BT_DEVCLS_AUDIO;
    bts2_app_data->bd_list_num = 0;
    bts2_app_data->inquiry_list_num = 0;
    bts2_app_data->inquiry_flag = TRUE;
    for (i = 0; i < MAX_DISCOV_RESS; i++)
    {
        bts2_app_data->inquiry_list[i].lap = CFG_BD_LAP;
        bts2_app_data->inquiry_list[i].nap = CFG_BD_NAP;
        bts2_app_data->inquiry_list[i].uap = CFG_BD_UAP;
    }
    gap_discov_req(bts2_app_data->phdl, MAX_DISCOV_RESS, 60, &inq_filter, TRUE);
    USER_TRACE(">> inquiry start...\n");
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
void bt_stop_inquiry(bts2_app_stru *bts2_app_data)
{
    gap_esc_discov_req(bts2_app_data->phdl);
    USER_TRACE(">> inquiry esc...\n");
}


void bt_register_receive_earphone_connect_req_handler(BOOL (*cb)(BTS2S_BD_ADDR *p_bd))
{
    hcia_register_receive_earphone_connect_req_handler(cb);
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
void bt_start_pairing(bts2_app_stru *bts2_app_data)
{
    BTS2S_CONN_INFO the_conn_info;

    sc_pair_req(bts2_app_data->phdl,
                &(bts2_app_data->bd_list[bts2_app_data->dev_idx]),
                &the_conn_info);
    USER_TRACE(">> Initiating pairing...\n");
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
void bt_start_unpairing(bts2_app_stru *bts2_app_data)
{
    /* void sc_debond_req(U16 phdl, BTS2S_BD_ADDR *bd);  */
    sc_unpair_req(bts2_app_data->phdl,
                  &(bts2_app_data->bd_list[bts2_app_data->dev_idx]));
    USER_TRACE(">> Start unpairing...\n");
}


void bt_io_capability_rsp(BTS2S_BD_ADDR *bd,
                          BTS2E_SC_IO_CAPABILITY io_capability,
                          U8 mitm,
                          U8 bonding)
{
    sc_io_capability_rsp(bd, io_capability, mitm, bonding);
}



__WEAK uint32_t bt_get_class_of_device(void)
{
    return 0x240704;
}

__WEAK void bt_sc_io_capability_rsp(BTS2S_BD_ADDR *bd)
{
    bt_io_capability_rsp(bd, IO_CAPABILITY_DISPLAY_YES_NO, FALSE, TRUE);
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
void bt_init_local_ctrller(bts2_app_stru *bts2_app_data)
{
    U8 tmp[MAX_FRIENDLY_NAME_LEN + 1];

    memset(tmp, 0, sizeof(BTS2S_DEV_NAME));
#ifdef CFG_MS
    BTS2S_DEV_NAME ctrller_name;
    U32 name_len = sizeof(BTS2S_DEV_NAME);
    /*
    write controller name with local computer name
    */
    memset(ctrller_name, 0, sizeof(ctrller_name));
    GetUserName(ctrller_name, &name_len);

    /* convert ANSI str to UNICODE str */
    bt_multibyte_to_widechar(CP_ACP, 0, ctrller_name, - 1, (void *)tmp, sizeof(tmp));
    /* convert UNICODE str to UTF8 str */
    memset(ctrller_name, 0, sizeof(ctrller_name));
    bt_widechar_to_multibyte(CP_UTF8, 0, (void *)tmp, - 1, (void *)ctrller_name, sizeof(ctrller_name), NULL, NULL);
#else
    uint8_t ret;
    bd_addr_t addr;
    ret = ble_get_public_address(&addr);
    if (ret == HL_ERR_NO_ERROR)
        rt_snprintf((char *)tmp, MAX_FRIENDLY_NAME_LEN, "SifliDemo-%x-%x-%x-%x-%x-%x", addr.addr[0], addr.addr[1], addr.addr[2], addr.addr[3], addr.addr[4], addr.addr[5]);
    else
        strcpy((char *)tmp, "SifliDemo");

#endif

    /* set local name */
    gap_set_local_name_req(bts2_app_data->phdl, (S8 *)tmp);

    /*
    write device class
    */
#if 0
    gap_wr_dev_cls_req(bts2_task_get_app_task_id(), BT_SRVCLS_TELEPHONE
                       | BT_SRVCLS_AUDIO
                       | BT_SRVCLS_OBJECT
                       | BT_SRVCLS_CAPTURE
                       | BT_SRVCLS_RENDER
                       | BT_SRVCLS_NETWORK
                       | BT_DEVCLS_PHONE
                       | BT_PHONECLS_CELLULAR);
#endif
    //gap_wr_dev_cls_req(bts2_task_get_app_task_id(), 0x800006);
    // gap_wr_dev_cls_req(bts2_task_get_app_task_id(), 0x240404);

    gap_wr_dev_cls_req(bts2_task_get_app_task_id(), bt_get_class_of_device());

    gap_rd_local_version_req(bts2_app_data->phdl);

    /*
    enable inquiry scan and page scan at the last step of BTS2 initializing process !!!
    */
    //gap_rd_scan_enb_req(bts2_app_data->phdl);

    U8 *eir = bmalloc(2 + strlen((const char *)tmp));
    BT_OOM_ASSERT(eir);
    if (eir)
    {
        *eir = 1 + strlen((const char *)tmp);
        *(eir + 1) = 9;
        memcpy((void *)(eir + 2), (void *)tmp, strlen((const char *)tmp));
        gap_wr_eir_req(0, eir, strlen((const char *)tmp) + 2);
        bfree(eir);
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
void bt_set_local_name(bts2_app_stru *bts2_app_data)
{
    BTS2S_DEV_NAME  tmp;
    BTS2S_DEV_NAME  name;
    memset(tmp, 0, sizeof(BTS2S_DEV_NAME));
    memset(name, 0, sizeof(BTS2S_DEV_NAME));

    if (bts2_app_data->input_str_len <= MAX_FRIENDLY_NAME_LEN)
    {
#ifdef CFG_MS
        bt_multibyte_to_widechar(CP_ACP, 0, bts2_app_data->input_str, - 1, (LPWSTR)tmp, sizeof(tmp));
        bt_widechar_to_multibyte(CP_UTF8, 0, (void *)tmp, - 1, (LPSTR)name, sizeof(name), NULL, NULL);
#else
        strcpy((char *)name, (const char *)bts2_app_data->input_str);
#endif
        gap_set_local_name_req(bts2_app_data->phdl, name);
        U8 *eir = bmalloc(2 + bts2_app_data->input_str_len);
        BT_OOM_ASSERT(eir);
        if (eir)
        {
            *eir = 1 + bts2_app_data->input_str_len;
            *(eir + 1) = 9;
            memcpy((void *)(eir + 2), (void *)bts2_app_data->input_str, bts2_app_data->input_str_len);
            gap_wr_eir_req(0, eir, bts2_app_data->input_str_len + 2);
        }
        bts2_app_data->menu_id = menu_gen_5;
    }
    else
    {
        INFO_TRACE(" -- name is too long, input again: ");
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
void bt_wr_dev_cls(bts2_app_stru *bts2_app_data)
{
    int num;
    U24 cod;
    char *end;

    num = bts2_app_data->input_str_len;
    cod = strtol((const char *)bts2_app_data->input_str, &end, 16);

    INFO_TRACE("\n");

    if (num <= 6 && cod > 0)
    {
        gap_wr_dev_cls_req(bts2_app_data->phdl, cod);
        bts2_app_data->menu_id = menu_gen_5;
    }
    else
    {
        INFO_TRACE(" -- COD is too long or wrong, input again: ");
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
void bt_wr_page_timeout(bts2_app_stru *bts2_app_data)
{
    int num;
    U16 timeout;
    char *end;

    num = bts2_app_data->input_str_len;
    timeout = (U16)strtol((const char *)bts2_app_data->input_str, &end, 16);

    INFO_TRACE("\n");

    if (num <= 4 && timeout > 0)
    {
        gap_wr_page_timeout_req(bts2_app_data->phdl, timeout);
        bts2_app_data->menu_id = menu_gen_5;
    }
    else
    {
        INFO_TRACE(" -- timeout is wrong, input again: ");
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
void bt_get_rmt_dev_name(bts2_app_stru *bts2_app_data)
{
    int num = 0;
    char c;

    c = bts2_app_data->input_str[0];
    INFO_TRACE("\n");

    if (BT_IN(c - '0', 0, bts2_app_data->bd_list_num))
    {
        num = c - '0';
        gap_rd_rmt_name_req(bts2_app_data->phdl, bts2_app_data->bd_list[num]);
        bts2_app_data->menu_id = menu_gen_6;
    }
    else
    {
        INFO_TRACE(" -- input again:");
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
void bt_get_role_discov(bts2_app_stru *bts2_app_data)
{
    int num = 0;
    char c;

    c = bts2_app_data->input_str[0];
    INFO_TRACE("\n");

    if (BT_IN(c - '0', 0, bts2_app_data->bd_list_num))
    {
        num = c - '0';
        gap_role_discov_req(bts2_app_data->phdl, bts2_app_data->bd_list[num]);
        bts2_app_data->menu_id = menu_gen_6;
    }
    else
    {
        INFO_TRACE(" -- input again:");
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
void bt_get_rmt_version(bts2_app_stru *bts2_app_data)
{
    int num = 0;
    char c;

    c = bts2_app_data->input_str[0];

    INFO_TRACE("\n");

    if (BT_IN(c - '0', 0, bts2_app_data->bd_list_num))
    {
        num = c - '0';
        gap_rd_rmt_version_req(bts2_app_data->phdl, bts2_app_data->bd_list[num]);
        bts2_app_data->menu_id = menu_gen_6;
    }
    else
    {
        INFO_TRACE(" -- input again:");
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
void bt_wr_link_supvisn_timeout(bts2_app_stru *bts2_app_data)
{
    int num;
    U16 timeout;
    char *end;

    num = bts2_app_data->input_str_len;
    timeout = (U16)strtol((const char *)bts2_app_data->input_str, &end, 16);

    INFO_TRACE("\n");

    if (num <= 4 && timeout >= 0)
    {
        gap_wr_super_vision_timeout_req(bts2_app_data->phdl, bts2_app_data->bd_list[bts2_app_data->dev_idx], timeout);
        bts2_app_data->menu_id = menu_gen_8;
    }
    else
    {
        INFO_TRACE(" -- timeout is wrong, input again: ");
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
void bt_rd_transmit_power_level(bts2_app_stru *bts2_app_data)
{
    char c;
    c = bts2_app_data->input_str[0];

    switch (c)
    {
    case '1':
        gap_rd_tx_power_level_req(bts2_app_data->phdl, bts2_app_data->bd_list[bts2_app_data->dev_idx], 0x00);
        bts2_app_data->menu_id = menu_gen_8;
        break;
    case '2':
        gap_rd_tx_power_level_req(bts2_app_data->phdl, bts2_app_data->bd_list[bts2_app_data->dev_idx], 0x01);
        bts2_app_data->menu_id = menu_gen_8;
        break;
    default:
        INFO_TRACE("level type is error, input again: ");
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
void bt_get_rssi(bts2_app_stru *bts2_app_data)
{
#ifdef BTS2_APP_MENU
    if (BT_IN(bts2_app_data->input_str[0] - '0', 0, bts2_app_data->bd_list_num))
    {
        gap_rd_rssi_req(bts2_app_data->phdl, bts2_app_data->bd_list[bts2_app_data->dev_idx]);
        bts2_app_data->menu_id = menu_gen_8;
    }
    else if (bts2_app_data->input_str[0] == 'r')
    {
        bts2_app_data->menu_id = menu_gen;
        bt_disply_menu(bts2_app_data);
    }
    else
        INFO_TRACE(" -- input again:");
#endif // BTS2_APP_MENU
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
void bt_wr_link_policy(bts2_app_stru *bts2_app_data)
{
    char *delim = " ";
    char *p;
    char c;

    U16 the_link_policy_setting;
    BTS2S_SNIFF_SETTINGS the_bts2s_sniff_setting;

    U8 setup_link_policy_setting;

#ifdef BTS2_APP_MENU
    memset(&the_bts2s_sniff_setting, 0, sizeof(BTS2S_PARK_SETTINGS));

    if (bts2_app_data->input_str_len < 8)
    {
        INFO_TRACE(" -- please input more pars!\n");
        bts2_app_data->menu_id = menu_gen_8;
        bt_disply_menu(bts2_app_data);
    }
    else
    {
        p = strtok((char *)bts2_app_data->input_str, delim);
        the_link_policy_setting = (U16)strtol(p, NULL, 16);


        p = strtok(NULL, delim);
        the_bts2s_sniff_setting.max_intvl = (U16)strtol(p, NULL, 16);
        p = strtok(NULL, delim);
        the_bts2s_sniff_setting.min_intvl = (U16)strtol(p, NULL, 16);
        p = strtok(NULL, delim);
        the_bts2s_sniff_setting.attmpt = (U16)strtol(p, NULL, 16);
        p = strtok(NULL, delim);
        the_bts2s_sniff_setting.timeout = (U16)strtol(p, NULL, 16);


        p = strtok(NULL, delim);
        c = p[0];

        switch (c)
        {
        case 'y':
        case 'Y':
            setup_link_policy_setting = TRUE;
            break;
        case 'N':
        case 'n':
            setup_link_policy_setting = FALSE;
            break;
        default:
            setup_link_policy_setting = TRUE;
            break;
        }
        gap_wr_link_policy_req(bts2_app_data->phdl,
                               bts2_app_data->pair_bd,
                               the_link_policy_setting,
                               setup_link_policy_setting,
                               NULL,
                               NULL);
        bts2_app_data->menu_id = menu_gen_8;
    }
#endif //BTS2_APP_MENU
}



void bt_etner_sniff_mode(bts2_app_stru *bts2_app_data)
{
    // char *delim = "_";
    // char *p;

    // p = strtok((char *)bts2_app_data->input_str, delim);
    // U16 max_intvl = (U16)strtol(p, NULL, 16);
    // p = strtok(NULL, delim);
    // U16 min_intvl = (U16)strtol(p, NULL, 16);
    // p = strtok(NULL, delim);
    // U16 attmpt = (U16)strtol(p, NULL, 16);
    // p = strtok(NULL, delim);
    // U16 timeout = (U16)strtol(p, NULL, 16);

    // USER_TRACE("mx inv %d, min inv %d, attmpt %d, timeout %d\n", max_intvl, min_intvl, attmpt, timeout);
    // USER_TRACE("sniff addr %x-%x-%x\n", bts2_app_data->last_conn_bd.lap, bts2_app_data->last_conn_bd.uap, bts2_app_data->last_conn_bd.nap);
    // hcia_sniff_mode(&bts2_app_data->last_conn_bd, max_intvl, min_intvl, attmpt, timeout, NULL);

    // bts2_app_data->menu_id = menu_gen_8;

    hcia_sniff_mode(&bts2_app_data->last_conn_bd, 800, 800, 1, 1, NULL);
}

void bt_exit_sniff_mode(bts2_app_stru *bts2_app_data)
{
    hcia_exit_sniff_mode(&bts2_app_data->last_conn_bd, NULL);
    // bts2_app_data->menu_id = menu_gen_8;
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
void bt_rd_clock(bts2_app_stru *bts2_app_data)
{
    switch (bts2_app_data->input_str[0])
    {
    case '0':
        gap_rd_clock_req(bts2_app_data->phdl, 0, bts2_app_data->pair_bd);
        bts2_app_data->menu_id = menu_gen_8;
        break;
    case '1':
        gap_rd_clock_req(bts2_app_data->phdl, 1, bts2_app_data->pair_bd);
        bts2_app_data->menu_id = menu_gen_8;
        break;
    default:
        INFO_TRACE(" -- clock is wrong, input again: ");
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
void bt_set_secu_mode(bts2_app_stru *bts2_app_data)
{
    sc_set_secu_level_req(bts2_app_data->phdl, (U8)(bts2_app_data->input_str[0] - '0'));
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
void bt_hdl_sc_msg(bts2_app_stru *bts2_app_data)
{
    U16 *msg_type;

    msg_type = (U16 *)bts2_app_data->recv_msg;
    switch (*msg_type)
    {
    case BTS2MU_SC_PASSKEY_IND:
    {
        BTS2S_SC_PASSKEY_IND *msg;

        msg = (BTS2S_SC_PASSKEY_IND *)bts2_app_data->recv_msg;

        bts2_app_data->pair_bd = msg->bd;
        USER_TRACE("<< pincode indication recived from device: %04X:%02X:%06lX\n",
                   msg->bd.nap,
                   msg->bd.uap,
                   msg->bd.lap);
        bt_pincode_indi(bts2_app_data);
        break;
    }
    case BTS2MU_SC_PAIR_CFM:
    {
        BTS2S_SC_PAIR_CFM *msg;

        msg = (BTS2S_SC_PAIR_CFM *)bts2_app_data->recv_msg;
        switch (msg->res)
        {
        case 0:
            USER_TRACE("<< Pairing result is successful!\n");
            break;
        case 1:
            USER_TRACE("<< Timeout!(the timer applied to the procedure is has expired!)\n");
            break;
        case 2:
            USER_TRACE("<< Pairing failed!(the auth failed!)\n");
            break;
        case 3:
            USER_TRACE("<< Pairing not allowed!(the security mode can not be changed, so auth. is not possible!)\n");
            break;
        default:
            break;
        }

        if (msg->added_to_sc_db_list)
        {
            USER_TRACE("<< The device is added to the security manager's device db.\n");
        }

        break;
    }
    case BTS2MU_SC_PAIR_IND:
    {
        BTS2S_SC_PAIR_IND *msg;
        msg = (BTS2S_SC_PAIR_IND *)bts2_app_data->recv_msg;

        bt_notify_device_base_info_t device_info;
        bt_addr_convert(&msg->bd, device_info.mac.addr);
        device_info.res = msg->res;
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_PAIR_IND, &device_info, sizeof(bt_notify_device_base_info_t));

        uint8_t state = 1;
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_PAIR_STATE, &state, 1);
        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE("-- Pair with device: %04X:%02X:%06lX\n",
                       msg->bd.nap,
                       msg->bd.uap,
                       msg->bd.lap);
        }
        else
        {
            USER_TRACE("-- Device : %04X:%02X:%06lX pair fail\n",
                       msg->bd.nap,
                       msg->bd.uap,
                       msg->bd.lap);
        }
        break;
    }
    case BTS2MU_SC_UNPAIR_CFM:
    {
        BTS2S_SC_UNPAIR_CFM *msg;
        msg = (BTS2S_SC_UNPAIR_CFM *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE("<< Unpair with device: %04X:%02X:%06lX\n",
                       msg->bd.nap,
                       msg->bd.uap,
                       msg->bd.lap);
        }
        break;
    }
    case BTS2MU_SC_AUTHORISE_IND:
    {
        BTS2S_SC_AUTHORISE_IND *msg;

        msg = (BTS2S_SC_AUTHORISE_IND *)bts2_app_data->recv_msg;
        USER_TRACE("<< Authorise indication from device: %04X:%02X:%06lX\n",
                   msg->bd.nap,
                   msg->bd.uap,
                   msg->bd.lap);
        sc_authorise_rsp(TRUE, &(msg->bd));
        break;
    }
    case BTS2MU_SC_RD_DEV_RECORD_IND:
    {
        BTS2S_SC_RD_DEV_RECORD_IND *msg;
        msg = (BTS2S_SC_RD_DEV_RECORD_IND *)bts2_app_data->recv_msg;

        USER_TRACE("<< %ld\n", msg->dev_num);
        break;
    }

    case BTS2MU_SC_RD_DEV_RECORD_CFM:
    {
        BTS2S_SC_RD_DEV_RECORD_CFM *msg;

        msg = (BTS2S_SC_RD_DEV_RECORD_CFM *)bts2_app_data->recv_msg;

        USER_TRACE("<< %ld\n", msg->total_dev_num);
        break;
    }

    case BTS2MU_SC_RD_PAIRED_DEV_RECORD_CFM:
    {
        U32 count;
        BTS2S_SC_RD_PAIRED_DEV_RECORD_CFM *msg;

        msg = (BTS2S_SC_RD_PAIRED_DEV_RECORD_CFM *)bts2_app_data->recv_msg;

        USER_TRACE("<< Total paired device: %ld\n", msg->total_dev_num);

        for (count = 0; count < msg->total_dev_num; count++)
        {
            USER_TRACE("<< bd[%ld]: %04X:%02X:%06lX\n",
                       count,
                       msg->bd[count].nap,
                       msg->bd[count].uap,
                       msg->bd[count].lap);
        }

        break;
    }
    case BTS2MU_SC_SET_SECU_LEVEL_CFM:
    {
        BTS2S_SC_SET_SECU_LEVEL_CFM *msg;

        msg = (BTS2S_SC_SET_SECU_LEVEL_CFM *)bts2_app_data->recv_msg;
        if (msg->res == 0x00)
        {
            USER_TRACE("<< set security level successful!\n");
        }
        else
        {
            USER_TRACE("<< failed to set security level!\n");
        }
        break;
    }
    case BTS2MU_SC_IO_CAPABILITY_REQ_IND:
    {
        BTS2S_SC_IO_CAPABILITY_REQ_IND *msg;

        msg = (BTS2S_SC_IO_CAPABILITY_REQ_IND *)bts2_app_data->recv_msg;

#ifdef BT_USING_PAIRING_CONFIRMATION
        uint8_t addr[6];
        bt_addr_convert(&msg->bd, addr);
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_IO_CAPABILITY_IND, addr, 6);
#else
#if 1
        bt_sc_io_capability_rsp(&msg->bd);
#else
        sc_io_capability_rsp(&msg->bd, IO_CAPABILITY_REJECT_REQ, FALSE, TRUE);
#endif
#endif
        break;
    }
    case BTS2MU_SC_REMOTE_IO_CAPABILITY_IND:
    {
        BTS2S_SC_REMOTE_IO_CAPABILITY_IND *msg;
        uint8_t state = 0;
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_PAIR_STATE, &state, 1);

        msg = (BTS2S_SC_REMOTE_IO_CAPABILITY_IND *)bts2_app_data->recv_msg;
        switch (msg->io_capability)
        {
        case IO_CAPABILITY_DISPLAY_ONLY:
            USER_TRACE("-- Remote device have display capability\n");
            break;
        case IO_CAPABILITY_DISPLAY_YES_NO:
            USER_TRACE("-- Remote device have display and input capability\n");
            break;
        case IO_CAPABILITY_KEYBOARD_ONLY:
            USER_TRACE("-- Remote device have input capability\n");
            break;
        case IO_CAPABILITY_NO_INPUT_NO_OUTPUT:
            USER_TRACE("-- Remote device haven't display and input capability\n");
            break;
        default:
            USER_TRACE("-- Unnkown Remote device capability\n");
            break;
        }
        break;
    }
    case BTS2MU_SC_REQ_USER_CONFIRM_IND:
    {
        /*33 event Numeric Comparison*/
        BTS2S_SC_USER_CONF_CFM *msg = (BTS2S_SC_USER_CONF_CFM *) bts2_app_data->recv_msg;
        bts2_app_data->pair_bd.lap = msg->bd.lap;
        bts2_app_data->pair_bd.uap = msg->bd.uap;
        bts2_app_data->pair_bd.nap = msg->bd.nap;
        USER_TRACE(">> The Numeric_Value is:%ld\n", msg->num_val);
#ifdef BT_USING_PAIRING_CONFIRMATION
        bt_notify_pair_confirm_t info;
        bt_addr_convert_to_general(&msg->bd, (bd_addr_t *)&info.mac);
        info.num_val = msg->num_val;
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_USER_CONFIRM_IND, &info, sizeof(bt_notify_pair_confirm_t));
#else
        sc_user_cfm_rsp(&(bts2_app_data->pair_bd), TRUE);
        // Yier: Just reply automaticlly.
        //bt_get_sele_yesno(bts2_app_data);
#endif
        break;
    }

    case BTS2MU_SC_REQ_USER_PASSKEY_IND:
    {
        /*34 event Passkey Entry*/
        BTS2S_SC_USER_PASSKY_CFM *msg = (BTS2S_SC_USER_PASSKY_CFM *) bts2_app_data->recv_msg;
        bts2_app_data->pair_bd.lap = msg->bd.lap;
        bts2_app_data->pair_bd.uap = msg->bd.uap;
        bts2_app_data->pair_bd.nap = msg->bd.nap;
        USER_TRACE("BTS2MU_SC_USER_PASSKY_CFM\n");
        bt_get_user_psky_value(bts2_app_data);
        break;
    }
    case BTS2MU_SC_RMT_OOB_DATA_CFM:
    {
        /*35 event Out of Band*/
        BTS2S_SC_RMT_OOB_DATA_CFM *msg = (BTS2S_SC_RMT_OOB_DATA_CFM *) bts2_app_data->recv_msg;
        bts2_app_data->pair_bd.lap = msg->bd.lap;
        bts2_app_data->pair_bd.uap = msg->bd.uap;
        bts2_app_data->pair_bd.nap = msg->bd.nap;
        USER_TRACE("BTS2MU_SC_RMT_OOB_DATA_CFM\n");
        bt_get_oob_data(bts2_app_data);
        break;
    }
    case BTS2MU_SC_USER_PASSKEY_NOTIFICATION_IND:
    {
        /*3b event*/
        BTS2S_SC_PASSKEY_NOTIFI *msg = (BTS2S_SC_PASSKEY_NOTIFI *) bts2_app_data->recv_msg;
        bts2_app_data->pair_bd.lap = msg->bd.lap;
        bts2_app_data->pair_bd.uap = msg->bd.uap;
        bts2_app_data->pair_bd.nap = msg->bd.nap;
        USER_TRACE(">> The passkey is:%ld\n", msg->passkey);
        bt_get_passkey_notifi(bts2_app_data);
        break;
    }
    case BTS2MU_SC_PAIRED_DEV_KEY_DELETE_CFM:
    {
        BTS2S_SC_PAIRED_DEV_DELETE_KEY_CFM  *msg = (BTS2S_SC_PAIRED_DEV_DELETE_KEY_CFM *) bts2_app_data->recv_msg;
        USER_TRACE("BTS2MU_SC_PAIRED_DEV_KEY_DELETE_CFM");
        uint8_t addr[6];
        bt_addr_convert(&msg->bd, addr);
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_KEY_OVERLAID, addr, 6);
        break;
    }

    default:
    {
        //INFO_TRACE("<< Unhandle message %x in SC\n", *msg_type);
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
void print_serice_name_from_svc_clsIDto_str(U16 clsID)
{
    INFO_TRACE(" ¨N service class name:");
    switch ((int)clsID)
    {
    case BT_UUID_SERVER_SERVICE:
        INFO_TRACE("service discov srv service\n");
        break;
    case BT_UUID_BROWSE_GROUP:
        INFO_TRACE("browse group desp service\n");
        break;
    case  BT_UUID_PUBLIC_BROWSE_GROUP:
        INFO_TRACE("public browse grou\n");
        break;
    case BT_UUID_SERIAL_PORT:
        INFO_TRACE("SPP PROF\n");
        break;
    case BT_UUID_LAN_ACCESS:
        INFO_TRACE("LAP PROF\n");
        break;
    case BT_UUID_DIALUP_NET:
        INFO_TRACE("DUN PROF\n");
        break;
    case BT_UUID_IRMC_SYNC:
        INFO_TRACE("OBEX IR MC SYNC SVC\n");
        break;
    case BT_UUID_OBEX_OBJ_PUSH:
        INFO_TRACE("OBEX OBJ PUSH SVC UUID\n");
        break;
    case BT_UUID_OBEX_FILE_TRANS:
        INFO_TRACE("OBEX FILE TRANSFER UUID\n");
        break;
    case BT_UUID_IRMC_SYNC_CMD:
        INFO_TRACE("ir MC sync cmd\n");
        break;
    case BT_UUID_HEADSET:
        INFO_TRACE("head set PROFIL\n");
        break;
    case BT_UUID_CORDLESS_TELE:
        INFO_TRACE("CTP PROF\n");
        break;
    case BT_UUID_AUDIO_SOURCE:
        INFO_TRACE("AUDIO SRC\n");
        break;
    case BT_UUID_AUDIO_SINK:
        INFO_TRACE("AUDIO SNK\n");
        break;
    case BT_UUID_AVRCP_TG:
        INFO_TRACE("AV RMT CTRL TARGET\n");
        break;
    case BT_UUID_ADV_AUDIO_DISTRIB:
        INFO_TRACE("ADVANCED AUDIO PROF\n");
        break;
    case BT_UUID_AVRCP_CT:
        INFO_TRACE("AV RMT CTRL UUID\n");
        break;
    case BT_UUID_VIDEO_CONFERENCE:
        INFO_TRACE("video conferencing\n");
        break;
    case BT_UUID_INTERCOM:
        INFO_TRACE("ICP PROF\n");
        break;
    case BT_UUID_FAX:
        INFO_TRACE("FAX PROF\n");
        break;
    case BT_UUID_HEADSET_AG:
        INFO_TRACE("HEADSET AG SVC\n");
        break;
    case BT_UUID_WAP:
        INFO_TRACE("WAP\n");
        break;
    case BT_UUID_WAP_CLIENT:
        INFO_TRACE("WAP clt\n");
    case BT_UUID_PAN_PANU:
        INFO_TRACE("PAN PANU PROF\n");
        break;
    case BT_UUID_PAN_NAP:
        INFO_TRACE("PAN NAP PROFIL\n");
        break;
    case BT_UUID_PAN_GN:
        INFO_TRACE("PAN GN PROF\n");
        break;
    case BT_UUID_DIRECT_PRINT:
        INFO_TRACE("DIRECT PRINTING\n");
        break;
    case BT_UUID_REF_PRINT:
        INFO_TRACE("REFERENCE PRINTING\n");
        break;
    case BT_UUID_IMAGING:
        INFO_TRACE("OBEX IMAGING\n");
        break;
    case BT_UUID_IMAG_RESPONDER:
        INFO_TRACE("OBEX IMAGING RESPONDER\n");
        break;
    case BT_UUID_IMAG_AUTO_ARCH:
        INFO_TRACE("imaging automatic_archive\n");
        break;
    case BT_UUID_IMAG_REF_OBJ:
        INFO_TRACE("imaging referenced obj\n");
        break;
    case BT_UUID_HF:
        INFO_TRACE("HF PROF\n");
        break;
    case BT_UUID_HF_AG:
        INFO_TRACE("HFG PROF\n");
        break;
    case BT_UUID_DIRECT_PRINT_REF_OBJ:
        INFO_TRACE("direct printing reference objects\n");
        break;
    case BT_UUID_REFLECTED_UI:
        INFO_TRACE("reflectedUI\n");
        break;
    case BT_UUID_BASIC_PRINT:
        INFO_TRACE("basic printing\n");
        break;
    case BT_UUID_PRINT_ST:
        INFO_TRACE("printing st\n");
        break;
    case BT_UUID_HID:
        INFO_TRACE("human ui device service\n");
        break;
    case BT_UUID_HCRP:
        INFO_TRACE("hardcp cable replacement\n");
        break;
    case BT_UUID_HCR_PRINT:
        INFO_TRACE("HCR print\n");
        break;
    case BT_UUID_HCR_SCAN:
        INFO_TRACE("HCR scan\n");
        break;
    case BT_UUID_COMMON_ISDN_ACCESS:
        INFO_TRACE("common ISDN access\n");
        break;
    case BT_UUID_VIDEO_CONFERENCE_GW:
        INFO_TRACE("video conferencing GW\n");
        break;
    case BT_UUID_UDI_MT:
        INFO_TRACE("UDI MT\n");
        break;
    case BT_UUID_UDI_TA:
        INFO_TRACE("UDI TA\n");
        break;
    case BT_UUID_AUDIO_VIDEO:
        INFO_TRACE("auido/video\n");
        break;
    case BT_UUID_SIM_ACCESS:
        INFO_TRACE("SIM access\n");
        break;
    case BT_UUID_PB_ACCESS_PCE:
        INFO_TRACE("phone book access - PCE\n");
        break;
    case BT_UUID_PB_ACCESS_PSE:
        INFO_TRACE("phone book access - PSE\n");
        break;
    case BT_UUID_PB_ACCESS:
        INFO_TRACE("phone book access\n");
        break;
    case BT_UUID_PNP_INFO:
        INFO_TRACE("pnp information\n");
        break;
    case BT_UUID_GENERIC_NET:
        INFO_TRACE("gen network\n");
        break;
    case BT_UUID_GENERIC_FILE_TRANS:
        INFO_TRACE("gen file transfer\n");
        break;
    case BT_UUID_GENERIC_AUDIO:
        INFO_TRACE("gen audio\n");
        break;
    case BT_UUID_GENERIC_TELE:
        INFO_TRACE("gen telephony\n");
        break;
    case BT_UUID_UPNP_SVC:
        INFO_TRACE("UPNP service\n");
        break;
    case BT_UUID_UPNP_IP_SVC:
        INFO_TRACE("UPNP IP service\n");
        break;
    case BT_UUID_ESDP_UPNP_IP_PAN:
        INFO_TRACE("ESDP UPNP IP PAN\n");
        break;
    case BT_UUID_ESDP_UPNP_IP_LAP:
        INFO_TRACE("ESDP UPNP IP LAP\n");
        break;
    case BT_UUID_ESDP_UPNP_L2C:
        INFO_TRACE("ESDP UPNP L2C\n");
        break;
    case BT_UUID_VIDEO_SRC:
        INFO_TRACE("VDP SRC\n");
        break;
    case BT_UUID_VIDEO_SNK:
        INFO_TRACE("VDP SNK\n");
        break;
    case BT_UUID_VIDEO_DISTRIBUTION:
        INFO_TRACE("video distribution profile\n");
        break;
    default:
        INFO_TRACE("unknown service\n ");
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
void bt_get_svc_res(bts2_app_stru *bts2_app_data)
{
#ifdef CFG_BQB_SDC_SSA
    BTS2S_SDC_SVC_SRCH_CFM *msg;
    U16 attr_id;
    msg = (BTS2S_SDC_SVC_SRCH_CFM *)bts2_app_data->recv_msg;

    attr_id =   /*0x0001;  SVC_ID_ATTRUTE_ID;               TP/SERVER/SA/BV-04-C*/
        /* 0x0004;PROT_DESP_LIST_ATTRUTE_ID; surpport       TP/SERVER/SA/BV-05-C*/
        /*0x0002;SVC_RECORD_ST_ATTRUTE_ID;          TP/SERVER/SA/BV-06-C*/
        /* 0x0007;                                  TP/SERVER/SA/BV-07-C*/
        /* 0x0005;                                  TP/SERVER/SA/BV-08-C*/
        0x0006;                                     /*TP/SERVER/SA/BV-09-C*/
    /*0x0008;                                   TP/SERVER/SA/BV-10-C*/
    /*0x000C;                                   TP/SERVER/SA/BV-11-C*/
    /*0x0100;  SVC_NAME_ATTRUTE_ID; suuport     TP/SERVER/SA/BV-12-C*/
    /*0x0101;SVC_DESP_ATTRUTE_ID;    s          TP/SERVER/SA/BV-13-C*/
    /*0x0102 SVC_PROVIDER_NAME;                 TP/SERVER/SA/BV-14-C*/
    /*0x0200;                                   TP/SERVER/SA/BV-15-C*/
    /*0x0201;                                   TP/SERVER/SA/BV-16-C*/
    /*0x0009; BLUETOOTH_PROF_DESP_LIST_ATTRUTE_ID;      TP/SERVER/SA/BV-17-C*/
    /*0x000A;                                   TP/SERVER/SA/BV-18-C*/
    /*0x000B;                                   TP/SERVER/SA/BV-19-C*/
    /*0x1234;                                   TP/SERVER/SA/BV-20-C*/
    /*
    0x0001
    0x0004
    0x0009
    0x0100
    0x0102
    */

    sdpa_sdc_svc_attrute_req(bts2_task_get_app_task_id(),
                             &bts2_app_data->bd_list[bts2_app_data->dev_idx],
                             0x00010002,
                             1,
                             &attr_id,
                             0x80);
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
void bt_close_kb_thread(void)
{
#ifdef CFG_MS
    CloseHandle(kb_thrd);
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
void bt_hdl_gap_msg(bts2_app_stru *bts2_app_data)
{
    U16 *msg_type;

    msg_type = (U16 *)bts2_app_data->recv_msg;
    switch (*msg_type)
    {
    /*inq res recvd */
    case BTS2MU_GAP_DISCOV_RES_IND:
    {
        BTS2S_DEV_NAME d_name;
        BTS2S_DEV_NAME tmp;
        BTS2S_GAP_DISCOV_RES_IND *msg;
        memset(d_name, 0, sizeof(d_name));
        memset(tmp, 0, sizeof(tmp));

        msg = (BTS2S_GAP_DISCOV_RES_IND *)bts2_app_data->recv_msg;



#ifdef CFG_MS
        bt_multibyte_to_widechar(CP_UTF8, 0, (void *)msg->dev_disp_name, - 1, (void *)d_name, sizeof(d_name));
        bt_widechar_to_multibyte(CP_ACP, 0x00000400, (void *)d_name, - 1, (void *)tmp, sizeof(tmp), NULL, FALSE);
#else
        strcpy((char *)tmp, (const char *)msg->dev_disp_name);
#endif
        bt_notify_remote_device_info_t remote_info;
        bmemset(&remote_info, 0, sizeof(bt_notify_remote_device_info_t));
        bt_addr_convert(&msg->bd, remote_info.mac.addr);
        strcpy((char *)remote_info.bt_name, (const char *)msg->dev_disp_name);
        remote_info.name_size = sizeof(remote_info.bt_name);
        remote_info.dev_cls = msg->dev_cls;
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_DISCOVER_IND, &remote_info, sizeof(bt_notify_remote_device_info_t));

        if (bts2_app_data->inquiry_flag == TRUE)
        {
            bts2_app_data->inquiry_list[bts2_app_data->inquiry_list_num] = msg->bd;
            USER_TRACE("<< idx: %02d--address: %04X:%02X:%06lX--class: %06lX--name: %s\n",
                       bts2_app_data->inquiry_list_num,
                       msg->bd.nap,
                       msg->bd.uap,
                       msg->bd.lap,
                       msg->dev_cls,
                       tmp);


            //temp
            //USER_TRACE(">>BTS2MU_GAP_DISCOV_RES_IND num%d \n",bts2_app_data->bd_list_num);

            if (bts2_app_data->inquiry_list_num == MAX_DISCOV_RESS - 1)
            {
                // gap_esc_discov_req(bts2_app_data->phdl);
                // USER_TRACE(">> inquiry stop, since max res recvd...\n");
                bts2_app_data->inquiry_list_num = 0;
            }
            else
            {
                bts2_app_data->inquiry_list_num++ ;
            }
        }

        break;
    }
    case BTS2MU_GAP_DISCOV_CFM:
    {
        BTS2S_GAP_DISCOV_CFM *msg;
        msg = (BTS2S_GAP_DISCOV_CFM *)bts2_app_data->recv_msg;
        bts2_app_data->inquiry_flag = FALSE;
        USER_TRACE(">> inquiry finished\n");
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_INQUIRY_CMP, NULL, 0);
        for (U8 i = 0; i < bts2_app_data->inquiry_list_num; i++)
        {
            USER_TRACE("<< idx: %02d--address: %04X:%02X:%06lX",
                       i,
                       bts2_app_data->inquiry_list[i].nap,
                       bts2_app_data->inquiry_list[i].uap,
                       bts2_app_data->inquiry_list[i].lap);
        }
        break;
    }

    case BTS2MU_GAP_ESC_DISCOV_CFM:
    {
        BTS2S_GAP_ESC_DISCOV_CFM *msg;

        msg = (BTS2S_GAP_ESC_DISCOV_CFM *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            bts2_app_data->inquiry_flag = FALSE;
            USER_TRACE(">> inquiry esced\n");
        }
        else
        {
            gap_esc_discov_req(bts2_app_data->phdl);
        }
        break;
    }

    case BTS2MU_GAP_SET_LOCAL_NAME_CFM:
    {
        BTS2S_GAP_SET_LOCAL_NAME_CFM *msg;

        msg = (BTS2S_GAP_SET_LOCAL_NAME_CFM *)bts2_app_data->recv_msg;
        gap_rd_local_name_req(bts2_app_data->phdl);

        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE("<< Local device name changed!\n");
        }
        else
        {
            USER_TRACE("<< Failed to set local device name!\n");
        }
        break;
    }
    case BTS2MU_GAP_RD_LOCAL_NAME_CFM:
    {
        BTS2S_GAP_RD_LOCAL_NAME_CFM *msg = (BTS2S_GAP_RD_LOCAL_NAME_CFM *)bts2_app_data->recv_msg;

        USER_TRACE("<< Local device name: %s\n", &msg->local_name);
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_BT_STACK_READY, NULL, 0);
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_LOCAL_NAME_RSP, &msg->local_name, sizeof(BTS2S_DEV_NAME));
        USER_TRACE("bt stack ready\n");
        break;
    }
    case BTS2MU_GAP_RD_LOCAL_BD_ADDR_CFM:

    {
        BTS2S_GAP_RD_LOCAL_BD_ADDR_CFM *msg;

        msg = (BTS2S_GAP_RD_LOCAL_BD_ADDR_CFM *)bts2_app_data->recv_msg;
        USER_TRACE("<< Local device addreess: %04X:%02X:%06lX\n",
                   msg->bd.nap,
                   msg->bd.uap,
                   msg->bd.lap);

        memcpy(&bts2_app_data->local_bd, &msg->bd, sizeof(BTS2S_BD_ADDR));

#ifdef BT_FINSH_PAN
        //add for tcpip route
        // USER_TRACE("lwip: enter local ether address generic app pointer %x\n",bts2_app_data);
        bts2_local_ether_addr =  bd2etheraddr(&(msg->bd));
        //bts2_app_data->local_ether_addr = bd2etheraddr(&(msg->bd));
        //USER_TRACE("lwip: Local device addreess: %04X:%04X:%04X\n",
        //                  bts2_local_ether_addr.w[0],
        //              bts2_local_ether_addr.w[1],
        //               bts2_local_ether_addr.w[2]);
#endif

        U8     addr[6];
        bt_addr_convert(&msg->bd, addr);
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_LOCAL_ADDR_RSP, addr, 6);
        break;
    }
    case BTS2MU_GAP_APP_INIT_CFM:
    {
        if (bts2_app_data->state == BTS_APP_IDLE)
        {
            bt_init_profile(bts2_app_data);

            bt_init_local_ctrller(bts2_app_data);


#ifdef CFG_MS
            /* Create and start keyboard thread */
            kb_thrd = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) key_msg_svc, NULL, 0, &thrd_id);
            if (kb_thrd == (HANDLE)(-1))
            {
                INFO_TRACE("Create keyboard thread fail\r\n");
            }
#endif
            bts2_app_data->state = BTS_APP_READY;
        }
        break;
    }
    case BTS2MU_GAP_KEYMISSING_IND:
    {
        BTS2S_GAP_KEYMISSING_IND *msg;

        msg = (BTS2S_GAP_KEYMISSING_IND *)bts2_app_data->recv_msg;
        U8     addr[6];
        bt_addr_convert(&msg->bd, addr);
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_KEY_MISSING, addr, 6);
        USER_TRACE("BTS2MU_GAP_KEYMISSING_IND\n");
        break;
    }
    case BTS2MU_GAP_WR_COD_CFM:
    {
        BTS2S_GAP_WR_COD_CFM *msg;

        msg = (BTS2S_GAP_WR_COD_CFM *)bts2_app_data->recv_msg;
        gap_rd_dev_cls_req(bts2_app_data->phdl);
        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE("<< Class of device has been changed!\n");
        }
        else
        {
            USER_TRACE("<< Failed to write class of device!\n");
        }
        break;
    }
    case BTS2MU_GAP_RD_COD_CFM:
    {
        BTS2S_GAP_RD_COD_CFM *msg;

        msg = (BTS2S_GAP_RD_COD_CFM *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE("<< Class of device: 0x%06lX\n", msg->dev_cls);
        }
        else
        {
            USER_TRACE("<< Failed to read class of device!\n");
        }
        break;
    }
    case BTS2MU_GAP_ACL_OPEN_IND:
    {
        BTS2S_GAP_ACL_OPEN_IND *msg;
        msg = (BTS2S_GAP_ACL_OPEN_IND *)bts2_app_data->recv_msg;
        INFO_TRACE(("<< Remote side want to establish an ACL link\n"));
        break;
    }
    case BTS2MU_GAP_RD_LOCAL_VERSION_CFM:
    {
        BTS2S_GAP_RD_LOCAL_VERSION_CFM *msg;

        msg = (BTS2S_GAP_RD_LOCAL_VERSION_CFM *)bts2_app_data->recv_msg;

        USER_TRACE("<< Local LMP version: %d\n", msg->lmp_version);
        USER_TRACE("<< Local LMP subversion: %d\n", msg->lmp_subversion);
        USER_TRACE("<< Local controller manufacturer name id: %d\n", msg->manufacturer_name);

        break;
    }
    case BTS2MU_GAP_WR_PAGE_TO_CFM:
    {
        BTS2S_GAP_WR_PAGE_TO_CFM *msg;

        msg = (BTS2S_GAP_WR_PAGE_TO_CFM *)bts2_app_data->recv_msg;

        INFO_TRACE("<< Write page timeout successful!\n");
        break;
    }
    case BTS2MU_GAP_ENB_DUT_MODE_CFM:
    {
        BTS2S_GAP_ENB_DUT_MODE_CFM *msg;

        msg = (BTS2S_GAP_ENB_DUT_MODE_CFM *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            INFO_TRACE("<< enb DUT mode : %d\n", msg->step_num);
        }
        else
        {
            INFO_TRACE("<< failed to enb DUT mode!\n");
        }
        break;
    }

    case BTS2MU_GAP_RD_RMT_NAME_CFM:
    {
        BTS2S_GAP_RD_RMT_NAME_CFM *msg;

        msg = (BTS2S_GAP_RD_RMT_NAME_CFM *)bts2_app_data->recv_msg;

        bt_notify_rmt_name_t device_name;
        bt_addr_convert(&msg->bd, device_name.mac.addr);
        device_name.res = msg->res;
        strcpy((char *)device_name.bt_name, (const char *)msg->dev_disp_name);
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_REMOTE_NAME_RSP, &device_name, sizeof(bt_notify_rmt_name_t));
        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE("<< remote device name: %s\n", msg->dev_disp_name);
        }
        else
        {
            USER_TRACE("<< failed to read remote device name!\n");
        }
        break;
    }
    case BTS2MU_GAP_RD_RMT_VERSION_CFM:
    {
        BTS2S_GAP_RD_RMT_VERSION_CFM *msg;

        msg = (BTS2S_GAP_RD_RMT_VERSION_CFM *)bts2_app_data->recv_msg;
        bt_notify_rmt_version_t version;
        version.res = msg->res;
        if (msg->res    == BTS2_SUCC)
        {
            USER_TRACE("<< remote device version \n");
            USER_TRACE("       LMP version:         %d\n", msg->lmp_version);
            USER_TRACE("       LMP subversion:      %d\n", msg->lmp_subversion);
            USER_TRACE("       manufacturer name:   %d\n", msg->manufacturer_name);
            version.lmp_subversion = msg->lmp_subversion;
            version.lmp_version = msg->lmp_version;
            version.manufacturer_name = msg->manufacturer_name;
        }
        else
        {
            USER_TRACE("<< failed to read remote device version!\n");
        }
        bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_RMT_VERSION_IND, &version, sizeof(bt_notify_rmt_version_t));
        break;
    }
    case BTS2MU_GAP_RD_SCAN_ENB_CFM:
    {
        BTS2S_GAP_RD_SCAN_ENB_CFM *msg;

        msg = (BTS2S_GAP_RD_SCAN_ENB_CFM *)bts2_app_data->recv_msg;

        if (msg->res    == BTS2_SUCC)
        {
            USER_TRACE("<< Scan mode: %d\n", msg->scan_enb);
            switch (msg->scan_enb)
            {
            case 0:
                USER_TRACE("<<            No scans enbd.\n");
                break;
            case 1:
                USER_TRACE("<<            Inquiry scan enbd.\n<<            Page scan disbd.\n");
                break;
            case 2:
                USER_TRACE("<<            Inquiry scan disbd.\n<<           Page scan enbd.\n");
                break;
            case 3:
                USER_TRACE("<<            Inquiry scan enbd.\n<<            Page scan enbd.\n");
                break;
            }
        }
        else
        {
            USER_TRACE("<< Failed to read scan mode!\n");
        }
        bts2_app_data->scan_mode = msg->scan_enb;
        break;
    }
    case BTS2MU_GAP_WR_SCAN_ENB_CFM:
    {
        BTS2S_GAP_WR_SCAN_ENB_CFM *msg;

        msg = (BTS2S_GAP_WR_SCAN_ENB_CFM *)bts2_app_data->recv_msg;
        gap_rd_scan_enb_req(bts2_task_get_app_task_id());
        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE("<< Write scan enable success\n");
        }
        else
        {
            USER_TRACE("<< Write scan enable fail\n");
        }
        break;
    }
    case BTS2MU_GAP_ROLE_DISCOV_CFM:
    {
        BTS2S_GAP_ROLE_DISCOV_CFM *msg;

        msg = (BTS2S_GAP_ROLE_DISCOV_CFM *)bts2_app_data->recv_msg;

        USER_TRACE("device address: %04X:%02X:%06lX",
                   msg->bd.nap,
                   msg->bd.uap,
                   msg->bd.lap);

        switch (msg->role)
        {
        case 0:
            USER_TRACE("    role: master\n");
            break;
        case 1:
            USER_TRACE("    role: slave\n");
            break;
        default:
            USER_TRACE("    role: %d(no link with this device)\n", msg->role);
            break;
        }
        break;
    }
    case BTS2MU_GAP_WR_LINK_SUPERV_TIMEOUT_CFM:
    {
        BTS2S_GAP_WR_LINK_SUPERV_TIMEOUT_CFM *msg;

        msg = (BTS2S_GAP_WR_LINK_SUPERV_TIMEOUT_CFM *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE("<< Write link supvisn timeout successful!\n");
        }
        else
        {
            //USER_TRACE("<< Failed to write link supvisn timeout!\n");
        }
        break;
    }
    case BTS2MU_GAP_RD_TX_POWER_LEVEL_CFM:
    {
        BTS2S_GAP_RD_TX_POWER_LEVEL_CFM *msg;

        msg = (BTS2S_GAP_RD_TX_POWER_LEVEL_CFM *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE("<< tx power level: %d d_bm\n", msg->power_level);
        }
        else
        {
            USER_TRACE("<< failed to read tx power level!\n");
        }
        break;
    }
    case BTS2MU_GAP_GET_LINK_QA_CFM:
    {
        BTS2S_GAP_GET_LINK_QA_CFM *msg;

        msg = (BTS2S_GAP_GET_LINK_QA_CFM *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE("<< link qa with remote device: 0x%X\n", msg->link_qa);
        }
        else
        {
            USER_TRACE("<< failed to get link qa!\n");
        }
        break;
    }
    case BTS2MU_GAP_RD_CLOCK_CFM:
    {
        BTS2S_GAP_RD_CLOCK_CFM *msg;

        msg = (BTS2S_GAP_RD_CLOCK_CFM *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE("<< clock    : 0x%08lX\n", msg->clock);
            USER_TRACE("<< accuracy : 0x%04X\n", msg->accuracy);
        }
        else
        {
            USER_TRACE("<< failed to read clock!\n");
        }
        break;
    }
    case BTS2MU_GAP_RD_RSSI_CFM:
    {
        BTS2S_GAP_RD_RSSI_CFM *msg;

        msg = (BTS2S_GAP_RD_RSSI_CFM *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            bt_interface_bt_event_notify(BT_NOTIFY_COMMON, BT_NOTIFY_COMMON_LOCAL_RSSI_RSP, &msg->rssi, sizeof(S8));
            USER_TRACE("<< remote device RSSI: %d dB\n", msg->rssi);
        }
        else
        {
            USER_TRACE("<< failed to read RSSI!\n");
        }
        break;
    }
    case BTS2MU_GAP_RD_LINK_POLICY_CFM:
    {
        BTS2S_GAP_RD_LINK_POLICY_CFM *msg;

        msg = (BTS2S_GAP_RD_LINK_POLICY_CFM *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE("<< actual mode: 0x%04X\n", msg->actual_mode);
            switch (msg->link_policy_setting)
            {
            case 0x00:
                USER_TRACE("<< link policy settings: disb all LM modes default.\n");
                break;
            case 0x01:
                USER_TRACE("<< link policy settings: enb role switch.\n");
                break;
            case 0x02:
                USER_TRACE("<< link policy settings: enb hold mode.\n");
                break;
            case 0x04:
                USER_TRACE("<< link policy settings: enb BTS2S_SNIFF mode.\n");
                break;
            case 0x08:
                USER_TRACE("<< link policy settings: enb BTS2S_PARK st.\n");
                break;
            default:
                USER_TRACE("<< link policy settings: 0x%04X\n", msg->link_policy_setting);
                break;
            }
            USER_TRACE("<< BTS2S_HOLD settings\n");
            USER_TRACE("<<     max BTS2S_HOLD intvl: 0x%X\n", msg->hold_setting.max_intvl);
            USER_TRACE("<<     min BTS2S_HOLD intvl: 0x%X\n", msg->hold_setting.min_intvl);

            USER_TRACE("<< BTS2S_SNIFF settings\n");
            USER_TRACE("<<     max BTS2S_SNIFF intvl: 0x%X\n", msg->sniff_setting.max_intvl);
            USER_TRACE("<<     min BTS2S_SNIFF intvl: 0x%X\n", msg->sniff_setting.min_intvl);
            USER_TRACE("<<     BTS2S_SNIFF attmpt:    0x%X\n", msg->sniff_setting.attmpt);
            USER_TRACE("<<     BTS2S_SNIFF timeout:   0x%X\n", msg->sniff_setting.timeout);
        }
        else
        {
            USER_TRACE("<< failed to read link policy!\n");
        }
        break;
    }
    case BTS2MU_GAP_WR_LINK_POLICY_ERR_IND:
    {
        BTS2S_GAP_WR_LINK_POLICY_ERR_IND *msg;

        msg = (BTS2S_GAP_WR_LINK_POLICY_ERR_IND *)bts2_app_data->recv_msg;

        if (msg->res == BTS2_SUCC)
        {
            USER_TRACE("<< Write link policy successful!\n");
        }
        else
        {
            USER_TRACE("<< Wailed to write link policy!\n");
        }
        break;
    }

    case BTS2MU_GAP_SDC_SVC_RECORD:
    {
        U16 idx;
        BTS2S_GAP_SDC_SVC_RECORD *msg;
        msg = (BTS2S_GAP_SDC_SVC_RECORD *)bts2_app_data->recv_msg;
        if (msg->svc_name_len > 0)
        {
            USER_TRACE("\nservice name: %s\n", msg->svc_name);
            bfree(msg->svc_name);
        }

        USER_TRACE("service record handle:0x%lx\n", msg->svc_hdl);
        USER_TRACE("Uuid number is:%d\n", msg->uuid_num);
        if (msg->uuid_num > 0)
        {
            for (idx = 0; idx < msg->uuid_num; idx++)
            {
                print_serice_name_from_svc_clsIDto_str((U16)(*(msg->uuid_list + idx)));
            }
            bfree(msg->uuid_list);
        }
        if (msg->rfc_chl != 0xFF)
        {
            USER_TRACE("rfcomm chl = 0x%x\n", msg->rfc_chl);
        }

        break;
    }
    case BTS2MU_GAP_SDC_SERVICE_RECORD_HANDLE:
    {
        BTS2S_GAP_SDC_SRCH_IND *msg;
        msg = (BTS2S_GAP_SDC_SRCH_IND *)bts2_app_data->recv_msg;
        break;

    }
    case BTS2MU_GAP_SDC_SRCH_CFM:
    {
        BTS2S_GAP_SDC_SRCH_CFM *msg;
        msg = (BTS2S_GAP_SDC_SRCH_CFM *)bts2_app_data->recv_msg;
        gap_sdc_close_req(bts2_app_data->phdl, msg->bd);
        break;
    }

    case BTS2MU_GAP_SDC_CLOSE_IND:
    {
        BTS2S_GAP_SDC_CLOSE_IND *msg;
        msg = (BTS2S_GAP_SDC_CLOSE_IND *)bts2_app_data->recv_msg;
        if (msg->res != BTS2_SUCC)
        {
            USER_TRACE(">> Browse service fail.\n");
        }
        else
        {
            USER_TRACE(">> Browse service complete\n");
        }
        break;
    }

    case BTS2MU_GAP_SCO_CONNECT_REQ_IND:
    {
        BTS2S_GAP_SCO_REMOTE_CONN_REQ *msg;
        msg = (BTS2S_GAP_SCO_REMOTE_CONN_REQ *)bts2_app_data->recv_msg;

        if (msg->code_id == 0)
        {
            gap_sync_conn_res(&msg->bd, msg->code_id, 0);
        }
        else
        {
            gap_sync_conn_res(&msg->bd, msg->code_id, 1);
        }
        break;
    }


#if 0
    case BTS2MU_GAP_HCI_RESET_CFM:
        /* set application inst data flag to BTS2_BT_READY */
        break;
#endif
    default:
    {
        //INFO_TRACE("<< unhandled message %x in CM\n", *msg_type);
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
#ifdef CFG_BQB_SDC_SSA
void bt_hdl_sdp_msg(bts2_app_stru *bts2_app_data)
{
    U16 *msg_type;

    msg_type = (U16 *)bts2_app_data->recv_msg;
    switch (*msg_type)
    {
    case BTS2M_SDC_SVC_SRCH_ATTRUTE_CFM:
    {
        BTS2S_SDC_SVC_SRCH_ATTRUTE_CFM *msg;
        msg = (BTS2S_SDC_SVC_SRCH_ATTRUTE_CFM *)bts2_app_data->recv_msg;
        INFO_TRACE("BTS2M_SDC_SVC_SRCH_ATTRUTE_CFM\n");
        INFO_TRACE("msg->rsp %x\n", msg->rsp);
        INFO_TRACE("msg->err_code %x\n", msg->err_code);
        break;
    }
    case BTS2M_SDC_SVC_SRCH_CFM:
    {
        INFO_TRACE("BTS2M_SDC_SVC_SRCH_CFM\n");
        /*bt_get_svc_res(bts2_app_data);*/
        break;
    }
    case BTS2M_SDC_SVC_ATTRUTE_CFM:
    {
        BTS2S_SDC_SVC_ATTRUTE_CFM *msg;
        msg = bts2_app_data->recv_msg;
        INFO_TRACE("BTS2M_SDC_SVC_ATTRUTE_CFM\n");
        if (msg->rsp == 0x0000 && msg->size_attr_list == 0xe && msg->attr_list[2] == 0x06)
        {
            U16 attr_list = 0x0100;
            /*{0x35,0x03,0x09, 0x01,0x02};*/
            sdpa_sdc_svc_attrute_req(bts2_task_get_app_task_id(),
                                     &bts2_app_data->bd_list[bts2_app_data->dev_idx],
                                     0x00010002,
                                     1,
                                     &attr_list,
                                     0x80);
        }
    }
    default:
    {
        break;
    }
    }
}
#endif




U8 bt_hdl_sc_passkey_notifi(bts2_app_stru *bts2_app_data)
{
    U32 uValue;
    BTS2S_SC_PASSKEY_NOTIFI *msg = (BTS2S_SC_PASSKEY_NOTIFI *) bts2_app_data->recv_msg;
    uValue  =  atoi((const char *)bts2_app_data->input_str);
    if (bts2_app_data->input_str_len > 0)
    {
        gap_user_passkey_request_reply(TRUE, &(bts2_app_data->pair_bd), uValue);
    }
    else
    {
        gap_user_passkey_request_reply(FALSE, &(bts2_app_data->pair_bd), uValue);
    }
    bts2_app_data->menu_id  = bts2_app_data->pre_menu_id;
    return  0;
}

U8 bt_hdl_sc_input(bts2_app_stru *bts2_app_data)
{
    U32 uValue;
    BTS2S_SC_USER_PASSKY_CFM *msg = (BTS2S_SC_USER_PASSKY_CFM *) bts2_app_data->recv_msg;
    uValue = atoi((const char *)bts2_app_data->input_str);
    if (bts2_app_data->input_str_len > 0)
    {
        gap_user_passkey_request_reply(TRUE, &(bts2_app_data->pair_bd), uValue);
    }
    else
    {
        gap_user_passkey_request_reply(FALSE, &(bts2_app_data->pair_bd), uValue);
    }
    bts2_app_data->menu_id  = bts2_app_data->pre_menu_id;
    return 0;
}

U8 bt_hdl_sc_yesno(bts2_app_stru *bts2_app_data)
{
    if ((strcmp((const char *)bts2_app_data->input_str, "y") == 0) || (strcmp((const char *)bts2_app_data->input_str, "Y") == 0))
    {
        sc_user_cfm_rsp(&(bts2_app_data->pair_bd), TRUE);
    }
    else if ((strcmp((const char *)bts2_app_data->input_str, "n") == 0) || (strcmp((const char *)bts2_app_data->input_str, "N") == 0))
    {
        sc_user_cfm_rsp(&(bts2_app_data->pair_bd), FALSE);
    }
    else
    {
        printf(">> Input again...\n");
    }
    bts2_app_data->menu_id  = bts2_app_data->pre_menu_id;
    return 0;
}

U8 bt_hdl_sc_oobdata(bts2_app_stru *bts2_app_data)
{
    BTS2S_SC_RMT_OOB_DATA_CFM *msg = (BTS2S_SC_RMT_OOB_DATA_CFM *) bts2_app_data->recv_msg;
    U8 bFlag = 1;

    //bmemset(bts2s_local_hash_c,0,sizeof(bts2s_local_hash_c));
    //bmemset(bts2s_local_rand_r,0,sizeof(bts2s_local_rand_r));

    if (bFlag)
    {
        gap_rmt_oob_data_request_reply(TRUE,
                                       &(bts2_app_data->pair_bd),
                                       bts2s_local_hash_c,
                                       bts2s_local_rand_r);
    }
    else
    {
        gap_rmt_oob_data_request_reply(FALSE,
                                       &(bts2_app_data->pair_bd),
                                       bts2s_local_hash_c,
                                       bts2s_local_rand_r);

    }

    return  0;
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
U8 bt_get_user_psky_value(bts2_app_stru *app_data)
{
#ifdef BTS2_APP_MENU
    app_data->pre_menu_id = app_data->menu_id;
    app_data->menu_id = menu_sc_input;
    bt_disply_menu(app_data);
#endif // BTS2_APP_MENU
    return 0;
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
U8 bt_get_passkey_notifi(bts2_app_stru *app_data)
{
#ifdef BTS2_APP_MENU
    app_data->pre_menu_id  = app_data->menu_id;
    app_data->menu_id = menu_sc_passkey_notifi;
    bt_disply_menu(app_data);
#endif
    return 0;
}

/*------------------------------------------------------------------------------*
 * getselectionyesno:

 * description: select yes/no option
 -------------------------------------------------------------------------------*/

void bt_get_sele_yesno(bts2_app_stru *app_data)
{
#ifdef BTS2_APP_MENU
    app_data->pre_menu_id  = app_data->menu_id;
    app_data->menu_id = menu_sc_yesno;
    bt_disply_menu(app_data);
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
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------*
 * germtoobdata:

 * description: input your remote oob data
 -------------------------------------------------------------------------------*/

U8 bt_get_oob_data(bts2_app_stru *app_data)
{
#ifdef BTS2_APP_MENU
    app_data->pre_menu_id = app_data->menu_id;
    app_data->menu_id = menu_sc_oobdata;
    bt_disply_menu(app_data);
#endif
    return  0;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
