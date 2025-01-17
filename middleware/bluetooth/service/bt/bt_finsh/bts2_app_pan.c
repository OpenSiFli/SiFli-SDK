/*
*********************************************************************************************************
* Copyright (C) 2006-2021 Lianway Corporation
*
* Introduction:
*       The purpose of design is to provide PAN app.
*
* File : bts2_app_pan.c
*
* History:
*
*********************************************************************************************************
*/
#include "bts2_app_inc.h"
#include "bts2_app_pan.h"
#include "bts2_task.h"
#include "rtdef.h"

#ifdef RT_USING_BT
    #include "bt_rt_device_urc.h"
#endif

static int running_flag = 0;
struct rt_bt_pan_instance  bt_pan_instance[MAX_PAN_INSTANCE_NUM];

#ifdef BT_FINSH_PAN

#define LOG_TAG         "btapp_pan"
#include "log.h"



#include "bt_prot.h"
#ifdef CFG_GNU
#define FIFO "/data/myfifo"
static void bt_pan_wr_data(char *string)
{
    int fd;
    int nwrite;

    if (fd == -1)
    {
        if (errno == ENXIO)
        {
            printf("open error; no reading process\n");
        }
    }

    fd = open(FIFO, O_WRONLY | O_NONBLOCK, 0);

    if ((nwrite = write(fd, string, 60)) == -1)
    {
        if (errno == EAGAIN)
        {
            printf("The FIFO has not been read yet.Please try later\n");
        }
    }
    else
    {
        printf("write %s to the FIFO\n", string);
    }
}

void bt_pan_set_ip_addr(char *string)
{
    char ipaddress[60];
    sprintf(ipaddress, "ifconfig btn0 address %s up", string);
    bt_pan_wr_data(ipaddress);
}

void bt_pan_set_netmask(char *string)
{
    char netmask[60];
    sprintf(netmask, "ifconfig btn0 netmask %s", string);
    bt_pan_wr_data(netmask);
}

void bt_pan_set_gw(char *string)
{
    char gw[60];
    sprintf(gw, "route add default gw %s", string);
    bt_pan_wr_data(gw);
}

void bt_pan_set_dns1(char *string)
{
    char gw[60];
    sprintf(gw, "setprop net.dns1 %s", string);
    bt_pan_wr_data(gw);
}

void bt_pan_set_dns2(char *string)
{
    char gw[60];
    sprintf(gw, "setprop net.dns2 %s", string);
    bt_pan_wr_data(gw);
}

void bt_pan_scan_proc_net_dev(void)
{
    FILE *proc_net_f;
    char linebuf[512];
    int linenum;
    unsigned char *p;
    char name[512]; /* XXX - pick a size */
    char *q, *saveq;

    proc_net_f = fopen("/proc/net/dev", "r");
    if (proc_net_f == NULL)
        return (0);

    for (linenum = 1; fgets(linebuf, sizeof linebuf, proc_net_f) != NULL; linenum++)
    {
        /*
         * Skip the first two lines - they're headers.
         */
        if (linenum <= 2)
            continue;

        p = &linebuf[0];

        /*
         * Skip leading white space.
         */
        while (*p != '\0' && isspace(*p))
            p++;
        if (*p == '\0' || *p == '\n')
            continue;   /* blank line */

        /*
         * Get the interface name.
         */
        q = &name[0];
        while (*p != '\0' && !isspace(*p))
        {
            if (*p == ':')
            {
                /*
                 * This could be the separator between a
                 * name and an alias number, or it could be
                 * the separator between a name with no
                 * alias number and the next field.
                 *
                 * If there's a colon after digits, it
                 * separates the name and the alias number,
                 * otherwise it separates the name and the
                 * next field.
                 */
                saveq = q;
                while (isdigit(*p))
                    *q++ = *p++;
                if (*p != ':')
                {
                    /*
                     * That was the next field,
                     * not the alias number.
                     */
                    q = saveq;
                }
                break;
            }
            else
                *q++ = *p++;
        }
        *q = '\0';

        /*
         * Get the flags for this interface, and skip it if
         * it's not up.
         */

        //strncpy(ifrflags.ifr_name, name, sizeof(ifrflags.ifr_name));
        USER_TRACE("Interface name:%s\n", name);
    }

    (void)fclose(proc_net_f);
}

void bt_pan_set_nap_route(char *string)
{
    char nap_route[60];
    bt_pan_wr_data("sh /data/iptables-open.sh");
    bt_pan_wr_data("sh /data/firwall.sh");
    sprintf(nap_route, "iptables -t nat -A POSTROUTING -o %s -s 192.168.1.0/24 -j SNAT --to 192.168.0.171", string);
    bt_pan_wr_data(nap_route);
}
#endif

static U8 bt_pan_get_idx(bts2_app_stru *bts2_app_data)
{
    U8 i = 0xff;
    bts2_pan_inst_data *ptr = NULL;
    ptr = bts2_app_data->pan_inst_ptr;

    for (i = 0; i < PAN_MAX_NUM; i++)
    {
        if (bd_is_empty(&(ptr->pan_sdp[i].bd_addr)))
        {
            return i;
        }
    }
    return i;
}

static U8 bt_pan_get_idx_by_bd(bts2_app_stru *bts2_app_data, const BTS2S_BD_ADDR *bd)
{
    U8 i = 0xff;
    bts2_pan_inst_data *ptr = NULL;
    ptr = bts2_app_data->pan_inst_ptr;

    for (i = 0; i < PAN_MAX_NUM; i++)
    {
        if (bd_eq(&(ptr->pan_sdp[i].bd_addr), bd))
        {
            return i;
        }
    }
    return i;
}

void bt_pan_init(bts2_app_stru *bts2_app_data)
{
    U8 i;
    bts2_app_data->pan_inst_ptr = &bts2_app_data->pan_inst;

    bts2_app_data->pan_inst.pan_st = PAN_REG_ST;
    bts2_app_data->pan_inst.id = 0xffff;
    bts2_app_data->pan_inst.local_role = PAN_NO_ROLE;
    bts2_app_data->pan_inst.rmt_role = PAN_NO_ROLE;
    bts2_app_data->pan_inst.mode = ACT_MODE;

    for (i = 0; i < PAN_MAX_NUM; i++)
    {
        bd_set_empty(&(bts2_app_data->pan_inst.pan_sdp[i].bd_addr));
        bts2_app_data->pan_inst.pan_sdp[i].gn_sdp_pending = FALSE;
        bts2_app_data->pan_inst.pan_sdp[i].gn_sdp_fail = FALSE;
        bts2_app_data->pan_inst.pan_sdp[i].nap_sdp_pending = FALSE;
        bts2_app_data->pan_inst.pan_sdp[i].nap_sdp_fail = FALSE;
    }
}

void bt_pan_reg(bts2_app_stru *bts2_app_data)
{
    bts2_pan_inst_data *ptr = NULL;
    ptr = bts2_app_data->pan_inst_ptr;

    if (ptr->pan_st == PAN_REG_ST)
    {
        /*
        void pan_reg_req(U16 the_conn_phdl, U16 the_data_phdl);
        */
        pan_reg_req(bts2_task_get_app_task_id(), bts2_task_get_pan_task_id(), bts2_app_data->local_bd);
        ptr->pan_st = PAN_IDLE_ST;
        bt_pan_enable(bts2_app_data);
        USER_TRACE(">> PAN register start\n");
    }
    else
    {
        USER_TRACE(">> PAN register fail\n");
    }
}

void bt_pan_enable(bts2_app_stru *bts2_app_data)
{
    bts2_pan_inst_data *ptr = NULL;
    ptr = bts2_app_data->pan_inst_ptr;

    if (ptr->pan_st == PAN_IDLE_ST)
    {
        /*
        void pan_enb_req(BOOL the_single_user, U16  the_local_role, U16  the_rmt_role);
        */
        //pan_enb_req(FALSE,PAN_NAP_ROLE,PAN_PANU_ROLE);
        //modified the pan role,local is panu
        pan_enb_req(FALSE, PAN_PANU_ROLE, PAN_NAP_ROLE);
        ptr->pan_st = PAN_IDLE_ST;
        USER_TRACE(">> PAN enable\n");
    }
    else
    {
        USER_TRACE(">> PAN enable fail\n");
    }
}

void bt_pan_conn(BTS2S_BD_ADDR *bd)
{
    bts2_app_stru *bts2_app_data = getApp();
    bts2_pan_inst_data *ptr = NULL;
    U8 idx, idx2;

    ptr = bts2_app_data->pan_inst_ptr;

    USER_TRACE(" pan st %x\n", ptr->pan_st);

    if (ptr->pan_st == PAN_IDLE_ST)
    {
        /*Before send connection,send service search requst*/
        //modified for product type

        idx = bt_pan_get_idx_by_bd(bts2_app_data, bd);
        if (idx != PAN_MAX_NUM)
        {
            if (ptr->pan_sdp[idx].gn_sdp_pending == TRUE)
            {
                USER_TRACE("SDP is in progress,connect pan later\n");
            }
            else
            {
                ptr->pan_sdp[idx].gn_sdp_pending = TRUE;
                pan_svc_srch_req(bts2_task_get_app_task_id(), bd, PAN_NAP_ROLE);
                USER_TRACE(">> PAN connect\n");
            }
        }
        else
        {
            idx2 = bt_pan_get_idx(bts2_app_data);

            if (idx2 != PAN_MAX_NUM)
            {
                bd_copy(&(ptr->pan_sdp[idx2].bd_addr), bd);
                ptr->pan_sdp[idx2].gn_sdp_pending = TRUE;
                pan_svc_srch_req(bts2_task_get_app_task_id(), bd, PAN_NAP_ROLE);
                USER_TRACE(">> PAN connect\n");
            }
            else
            {
                USER_TRACE("No pan resources are available\n");
            }
        }
    }
    else
    {
        USER_TRACE(">> PAN connect fail\n");
    }
}


extern bts2_app_stru *bts2g_app_p;
void bt_pan_conn_by_addr(BTS2S_BD_ADDR *remote_addr)
{
    bts2_app_stru *bts2_app_data = bts2g_app_p;
    bts2_pan_inst_data *ptr = NULL;
    U8 idx, idx2;

    ptr = bts2_app_data->pan_inst_ptr;

    USER_TRACE(" pan st %x\n", ptr->pan_st);

    if (ptr->pan_st == PAN_IDLE_ST)
    {
        /*Before send connection,send service search requst*/
        //modified for product type
        idx = bt_pan_get_idx_by_bd(bts2_app_data, remote_addr);
        if (idx != PAN_MAX_NUM)
        {
            if (ptr->pan_sdp[idx].gn_sdp_pending == TRUE)
            {
                USER_TRACE("SDP is in progress,connect pan later\n");
            }
            else
            {
                ptr->pan_sdp[idx].gn_sdp_pending = TRUE;
                pan_svc_srch_req(bts2_task_get_app_task_id(), remote_addr, PAN_NAP_ROLE);
                USER_TRACE(">> PAN connect\n");
            }
        }
        else
        {
            idx2 = bt_pan_get_idx(bts2_app_data);

            if (idx2 != PAN_MAX_NUM)
            {
                bd_copy(&(ptr->pan_sdp[idx2].bd_addr), remote_addr);
                ptr->pan_sdp[idx2].gn_sdp_pending = TRUE;
                pan_svc_srch_req(bts2_task_get_app_task_id(), remote_addr, PAN_NAP_ROLE);
                USER_TRACE(">> PAN connect\n");
            }
            else
            {
                USER_TRACE("No pan resources are available\n");
            }
        }
    }
    else
    {
        if (ptr->pan_st == PAN_BUSY_ST)
        {
            USER_TRACE(">> PAN already connected\n");
        }
        else
        {
            USER_TRACE(">> PAN connect fail\n");
        }
    }
}

extern BTS2S_ETHER_ADDR bd2etheraddr(const BTS2S_BD_ADDR *bd);

void bt_pan_update_addr(BTS2S_BD_ADDR *bd_addr)
{
    bts2_local_ether_addr =  bd2etheraddr(bd_addr);
    return;
}

void bt_pan_disc(BTS2S_BD_ADDR *bd)
{
    bts2_app_stru *bts2_app_data = getApp();
    bts2_pan_inst_data *ptr = NULL;
    ptr = bts2_app_data->pan_inst_ptr;

    if (ptr->pan_st == PAN_BUSY_ST)
    {
        /*
        void pan_disc_req (U16 the_id);
        void pan_disc_res (U16 the_id);
        */
        pan_disc_req(ptr->id);
        USER_TRACE(">> PAN disconnect\n");
    }
    else
    {
        USER_TRACE(">> PAN disconnect fail");
    }
}

void bt_lwip_pan_send(struct rt_bt_pan_instance *bt_instance, void *buff, int len)
{
    bts2_pan_inst_data *ptr = NULL;
    void  *p;
    U8   *eth_header;
    ptr = bt_instance->bts2_app_data->pan_inst_ptr;

    if (ptr->pan_st == PAN_BUSY_ST)
    {
        if (ptr->mode == SNIFF_MODE)
            bt_exit_sniff_mode(bt_instance->bts2_app_data);

        BTS2S_PAN_DATA_REQ *msg;
        msg = (BTS2S_PAN_DATA_REQ *)bmalloc(sizeof(BTS2S_PAN_DATA_REQ));
        BT_OOM_ASSERT(msg);
        if (msg)
        {
            msg->type = BTS2MD_PAN_DATA_REQ;
            eth_header = buff;
            msg->ether_type = (eth_header[12] << 8) + eth_header[13];
            msg->len = len - 14;
            buff = eth_header + 14;

            //msg->dst_addr = bt_pan_get_remote_mac_address(bt_instance);
            msg->dst_addr.w[0] = (((U16)eth_header[0]) << 8) | (U16)eth_header[1];
            msg->dst_addr.w[1] = (((U16)eth_header[2]) << 8) | (U16)eth_header[3];
            msg->dst_addr.w[2] = (((U16)eth_header[4]) << 8) | (U16)eth_header[5];
            msg->src_addr = bt_pan_get_mac_address(bt_instance);
            p = bmalloc(msg->len);
            BT_OOM_ASSERT(p);
            if (p == NULL)
            {
                bfree(msg);
                return;
            }
            memcpy(p, buff, msg->len);
            msg->payload = p;
            bts2_msg_put(bts2_task_get_pan_task_id(), BTS2M_PAN, msg);
            //USER_TRACE("bt_lwip_pan_send\n");
        }
    }
    else
    {
        USER_TRACE(">> PAN data send fail\n");
    }
}
void rt_lwip_instance_register_event_handler(struct rt_bt_pan_instance *bt_instance, rt_bt_instance_event_t event, rt_bt_instance_event_handler handler)
{
    int i = 0;
    for (i = 0; i < MAX_PAN_INSTANCE_NUM; i++)
    {
        bt_instance->handler_table[event][i] = handler;
    }
}
struct rt_bt_instance_ops instance_ops =
{
    RT_NULL,
    RT_NULL,
    bt_lwip_pan_send
};

void bt_lwip_pan_control_tcpip(bts2_app_stru *bts2_app_data)
{
    bt_pan_instance[0].bts2_app_data =  bts2_app_data;
    bt_pan_instance[0].ops =  &instance_ops;
    rt_bt_prot_attach_pan_instance(&bt_pan_instance[0]);
}

void bt_lwip_pan_detach_tcpip(bts2_app_stru *bts2_app_data)
{
    bt_pan_instance[0].bts2_app_data =  bts2_app_data;
    bt_pan_instance[0].ops =  &instance_ops;
    rt_bt_prot_detach_pan_instance(&bt_pan_instance[0]);
}


void bt_hdl_pan_msg(bts2_app_stru *bts2_app_data)
{

    U16 *msg_type = NULL;
    bts2_pan_inst_data *ptr;

    ptr = bts2_app_data->pan_inst_ptr;
    msg_type = (U16 *)bts2_app_data->recv_msg;

    switch (*msg_type)
    {
    case BTS2MU_PAN_ENB_CFM:
    {
        BTS2S_PAN_ENB_CFM *msg;

        msg = (BTS2S_PAN_ENB_CFM *)bts2_app_data->recv_msg;
        if (msg->res == BTS2_FAILED)
        {

            ptr->pan_st = PAN_IDLE_ST;
            USER_TRACE(">> Enable PAN fail\n");
        }

        USER_TRACE(">> PAN enable success\n");
        //bt_lwip_pan_control_tcpip(bts2_app_data);
        break;
    }
    case BTS2MU_PAN_CONN_IND:
    {
        BTS2S_PAN_CONN_IND *msg;
#ifdef CFG_GNU
        BOOL init_ret;
#endif
        BTS2S_BD_ADDR local_bd;
        BTS2S_ETHER_ADDR local_eth;

        local_bd.lap = bts2_app_data->local_bd.lap;
        local_bd.nap = bts2_app_data->local_bd.nap;
        local_bd.uap = bts2_app_data->local_bd.uap;

        local_eth = bconvbd2etherbig(&local_bd);

        msg = (BTS2S_PAN_CONN_IND *)bts2_app_data->recv_msg;

        USER_TRACE("<< PAN local device addreess: %04X:%02X:%06lX, result=%d\n",
                   bts2_app_data->local_bd.nap,
                   bts2_app_data->local_bd.uap,
                   bts2_app_data->local_bd.lap,
                   msg->res);

        if (msg->res == BTS2_SUCC)
        {
            ptr->id = msg->id;
            ptr->bd_addr = msg->bd_addr;
            ptr->local_role = msg->local_role;
            ptr->rmt_role = msg->rmt_role;
#ifdef CFG_BNEP_DBG
            INFO_TRACE(">> Remote device bd_addr: %04X:%02X:%06X\n", msg->bd_addr.nap, msg->bd_addr.uap, msg->bd_addr.lap);
            INFO_TRACE("\n>> enable finish,now regeist a network card\n");
#endif

            bt_notify_profile_state_info_t profile_state;
            bt_addr_convert(&msg->bd_addr, profile_state.mac.addr);
            profile_state.profile_type = BT_NOTIFY_PAN;
            profile_state.res = BTS2_SUCC;
            bt_interface_bt_event_notify(BT_NOTIFY_PAN, BT_NOTIFY_PAN_PROFILE_CONNECTED,
                                         &profile_state, sizeof(bt_notify_profile_state_info_t));

#ifdef CFG_GNU
            if (0 == running_flag)
            {
                init_ret = Inter_Init((U8 *)(&local_eth));
            }

            if (1 == init_ret)
            {
                INFO_TRACE("init netcard successful\n");
            }

            running_flag ++ ;
#endif
            ptr->pan_st = PAN_BUSY_ST;

            // bts2_remote_ether_addr = bd2etheraddr(&(msg->bd_addr));
            INFO_TRACE(">> PAN connect successfully, running_flag = %d\n", running_flag);
            lwip_sys_init();
            bt_lwip_pan_control_tcpip(bts2_app_data);
        }
        USER_TRACE(" BTS2MU_PAN_CONN_IND\n");

#ifdef CFG_BQB
#ifdef TP_BNEP_BV_20_C
        struct rt_bt_pan_instance *bt_instance;
        U8 buff[50];
        int len = 50;
        bt_instance = &bt_pan_instance[0];
        bt_instance->bts2_app_data = bts2_app_data;
        bt_instance->bts2_app_data->pan_inst_ptr =  ptr;
        memset(buff, 0xbb, 50);
        buff[12] = 0x08;
        buff[13] = 0x00;//ether_type ipv4
        extern BTS2S_ETHER_ADDR   bts2_local_ether_addr;
        bts2_local_ether_addr.w[0] = 0xaaaa;
        bts2_local_ether_addr.w[1] = 0xaaaa;
        bts2_local_ether_addr.w[2] = 0xaaaa;
        bt_lwip_pan_send(bt_instance, buff, len);
#endif
#ifdef TP_PANU_AUTONET_BV_01_I
        struct rt_bt_pan_instance *bt_instance;
        //raw data:28byte
        //header 14byte;
        U8 buff[42] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00\
                       , 0x00, 0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0xab, 0x89, 0x56, 0x34, 0x12, 0x00\
                       , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xa8, 0x01, 0x1e
                      };
        int len = 42;
        bt_instance = &bt_pan_instance[0];
        bt_instance->bts2_app_data = bts2_app_data;
        bt_instance->bts2_app_data->pan_inst_ptr =  ptr;
        buff[12] = 0x08;
        buff[13] = 0x06;//ether_type ETHER_TYPE_ARP
        extern BTS2S_ETHER_ADDR   bts2_local_ether_addr;
        bts2_local_ether_addr.w[0] = 0xab89;
        bts2_local_ether_addr.w[1] = 0x5634;
        bts2_local_ether_addr.w[2] = 0x1200;
        bt_lwip_pan_send(bt_instance, buff, len);
#endif
#endif

        break;
    }
    case BTS2MU_PAN_DATA_IND:
    {
        BTS2S_PAN_DATA_IND *msg;
        msg = (BTS2S_PAN_DATA_IND *)bts2_app_data->recv_msg;

        if (ptr->mode == SNIFF_MODE)
            bt_exit_sniff_mode(bts2_app_data);

#ifdef CFG_GNU
        tx_data(msg);
#endif
        //USER_TRACE(" BTS2MU_PAN_DATA_IND\n");
        msg->len = msg->len + 14;
        memcpy(msg->payload, msg->dst_addr.w, 6);
        memcpy(msg->payload + 6, msg->src_addr.w, 6);
        msg->payload[12] = (msg->ether_type >> 8);
        msg->payload[13] = (msg->ether_type & 0xff);
        if (0x86dd == msg->ether_type)
        {
            bfree(msg->payload);
            break;
        }
        rt_bt_instance_transfer_prot(&bt_pan_instance[0], (void *)msg->payload, msg->len);
        bfree(msg->payload);
        break;
    }
    case BTS2MU_PAN_DISC_IND:
    {
        BTS2S_PAN_DISC_IND *msg;
        msg = (BTS2S_PAN_DISC_IND *)bts2_app_data->recv_msg;
        ptr->pan_st = PAN_IDLE_ST;
        ptr->id = 0xffff;
        ptr->local_role = PAN_NO_ROLE;
        ptr->rmt_role = PAN_NO_ROLE;
        bd_set_empty(&(ptr->bd_addr));
        bt_lwip_pan_detach_tcpip(bts2_app_data);

        bt_notify_profile_state_info_t profile_state;
        bt_addr_convert(&msg->bd, profile_state.mac.addr);
        profile_state.profile_type = BT_NOTIFY_PAN;
        profile_state.res = msg->res;
        bt_interface_bt_event_notify(BT_NOTIFY_PAN, BT_NOTIFY_PAN_PROFILE_DISCONNECTED,
                                     &profile_state, sizeof(bt_notify_profile_state_info_t));

        lwip_system_uninit();
        INFO_TRACE(" BTS2MU_PAN_DISC_IND\n");
        break;
    }
    case BTS2MU_PAN_STS_IND:
    {
        BTS2S_PAN_STS_IND *msg;
        msg = (BTS2S_PAN_STS_IND *)bts2_app_data->recv_msg;
#ifdef CFG_BNEP_DBG
        INFO_TRACE("id=%d\n", msg->id);
        INFO_TRACE("ev=%d\n", msg->ev);
        INFO_TRACE("sts=%d\n", msg->sts);
#endif
        if (msg->ev == PAN_LINK_ST_EV)
            ptr->mode = msg->sts;
        INFO_TRACE(" BTS2MU_PAN_STS_IND\n");
        break;
    }
    case BTS2MU_PAN_SVC_SRCH_CFM:
    {
        BTS2S_PAN_SVC_SRCH_CFM *msg;
        BTS2S_BD_ADDR rem_addr;
        U8 idx;

        msg = (BTS2S_PAN_SVC_SRCH_CFM *)bts2_app_data->recv_msg;

        rem_addr.nap = msg->bd_addr.nap;
        rem_addr.uap = msg->bd_addr.uap;
        rem_addr.lap = msg->bd_addr.lap;


        if (msg->res != BTS2_SUCC)
        {
            idx = bt_pan_get_idx_by_bd(bts2_app_data, &rem_addr);
            if (idx != PAN_MAX_NUM)
            {
                if (ptr->pan_sdp[idx].gn_sdp_pending)
                {
                    ptr->pan_sdp[idx].gn_sdp_fail = TRUE;
                    ptr->pan_sdp[idx].gn_sdp_pending = FALSE;
                }

                if (ptr->pan_sdp[idx].nap_sdp_pending)
                {
                    ptr->pan_sdp[idx].nap_sdp_fail = TRUE;
                    ptr->pan_sdp[idx].nap_sdp_pending = FALSE;
                    bd_set_empty(&(ptr->pan_sdp[idx].bd_addr));
                    INFO_TRACE("Remote device has no gn or nap service\n");
                    break;
                }

                if (ptr->pan_sdp[idx].gn_sdp_fail)
                {
                    ptr->pan_sdp[idx].gn_sdp_fail = FALSE;
                    ptr->pan_sdp[idx].nap_sdp_pending = TRUE;
                    pan_svc_srch_req(bts2_task_get_app_task_id(), &rem_addr, PAN_NAP_ROLE);
                }
            }
            else
            {
                INFO_TRACE("error,the address is wrong\n");
            }
        }
        else
        {
            INFO_TRACE("SDP service search finish successfully\n");
        }
        break;
    }
    case BTS2MU_PAN_SVC_SRCH_RSP_IND:
    {
        /*When receive this message,indicate that the service search finish,so can send connect request now*/
        U8 idx;
        BTS2S_PAN_SVC_SRCH_RSP_IND *msg;
        msg = (BTS2S_PAN_SVC_SRCH_RSP_IND *)bts2_app_data->recv_msg;

        INFO_TRACE("BTS2MU_PAN_SVC_SRCH_RSP_IND bd: %04X:%02X:%06X\n", msg->bd_addr.nap, msg->bd_addr.uap, msg->bd_addr.lap);
        INFO_TRACE("BTS2MU_PAN_SVC_SRCH_RSP_IND role:%x\n", msg->srch_role);

        if ((msg->srch_role == PAN_GN_ROLE) || (msg->srch_role == PAN_NAP_ROLE))
        {
            /*
            void pan_conn_req(BTS2S_BD_ADDR *the_bd_addr, U16 the_local_role, U16 the_rmt_role);
            */
            idx = bt_pan_get_idx_by_bd(bts2_app_data, &msg->bd_addr);

            if (idx != PAN_MAX_NUM)
            {
                bd_set_empty(&(bts2_app_data->pan_inst.pan_sdp[idx].bd_addr));
                bts2_app_data->pan_inst.pan_sdp[idx].gn_sdp_pending = FALSE;
                bts2_app_data->pan_inst.pan_sdp[idx].gn_sdp_fail = FALSE;
                bts2_app_data->pan_inst.pan_sdp[idx].nap_sdp_pending = FALSE;
                bts2_app_data->pan_inst.pan_sdp[idx].nap_sdp_fail = FALSE;
                pan_conn_req(&(msg->bd_addr), PAN_PANU_ROLE, msg->srch_role);
            }
            else
            {
                INFO_TRACE("error,the address is wrong\n");
            }
        }
        else
        {
            INFO_TRACE("BTS2MU_PAN_SVC_SRCH_RSP_IND: fail\n");
        }
        break;
    }
    default:
    {
        INFO_TRACE("Default %x\n", *msg_type);
        break;
    }
    }
}

#endif
