/*
*********************************************************************************************************
* Copyright (C) 2006-2021 Lianway Corporation
*
* Introduction:
*       The purpose of design is to provide PAN app.
*
* File : bts2_app_pan.h
*
* History:
*
*
*********************************************************************************************************
*/

#ifndef _BTS2_APP_PAN_H_
#define _BTS2_APP_PAN_H_

#include "bts2_app_demo.h"

#ifdef __cplusplus
extern "C" {
#endif



#define MAX_PAN_INSTANCE_NUM       1



typedef enum
{
    RT_BT_INSTANCE_EVT_INIT_DONE = 0,
    RT_BT_INSTANCE_EVT_CONNECT,
    RT_BT_INSTANCE_EVT_DISCONNECT,
    RT_BT_INSTANCE_EVT_MAX,
} rt_bt_instance_event_t;

struct rt_bt_pan_instance;

typedef void (*rt_bt_instance_event_handler)(struct rt_bt_pan_instance *bt_instance, rt_bt_instance_event_t event);


struct rt_bt_instance_ops
{
    rt_err_t (*bt_init)(struct rt_bt_pan_instance *bt_instance);
    int (*bt_recv)(struct rt_bt_pan_instance *bt_instance, void *buff, int len);
    void (*bt_send)(struct rt_bt_pan_instance *bt_instance, void *buff, int len);
};

struct rt_bt_pan_instance
{
    bts2_app_stru     *bts2_app_data;
    void *prot;
    rt_bt_instance_event_handler handler_table[RT_BT_INSTANCE_EVT_MAX][MAX_PAN_INSTANCE_NUM];
    struct rt_bt_instance_ops *ops;
    void *user_data;
};




#ifdef BT_FINSH_PAN

void bt_pan_init(bts2_app_stru *bts2_app_data);
void bt_pan_reg(bts2_app_stru *bts2_app_data);
void bt_pan_enable(bts2_app_stru *bts2_app_data);
void bt_pan_conn(BTS2S_BD_ADDR *bd);
void bt_pan_conn_by_addr(BTS2S_BD_ADDR *remote_addr);
void bt_pan_update_addr(BTS2S_BD_ADDR *bd_addr);
void bt_pan_disc(BTS2S_BD_ADDR *bd);
void bt_hdl_pan_msg(bts2_app_stru *bts2_app_data);

void bt_pan_set_ip_addr(char *string);
void bt_pan_set_netmask(char *string);
void bt_pan_set_gw(char *string);
void bt_pan_set_dns(char *string1, char *string2);
void bt_pan_scan_proc_net_dev(void);
void bt_pan_set_nap_route(char *string);
void bt_pan_set_dns1(char *string);
void bt_pan_set_dns2(char *string);


void bt_lwip_pan_send(struct rt_bt_pan_instance *bt_instance, void *buff, int len);
void rt_lwip_instance_register_event_handler(struct rt_bt_pan_instance *bt_instance, rt_bt_instance_event_t event, rt_bt_instance_event_handler handler);
void bt_lwip_pan_control_tcpip(bts2_app_stru *bts2_app_data);
void bt_lwip_pan_detach_tcpip(bts2_app_stru *bts2_app_data);

extern rt_err_t rt_bt_prot_attach_pan_instance(struct rt_bt_pan_instance *panInstance);
extern rt_err_t rt_bt_instance_transfer_prot(struct rt_bt_pan_instance *bt_instance, void *buff, int len);
//extern BTS2S_ETHER_ADDR bt_pan_get_remote_mac_address(struct rt_bt_pan_instance *bt_instance);
extern BTS2S_ETHER_ADDR bt_pan_get_mac_address(struct rt_bt_pan_instance *bt_instance);

extern void lwip_sys_init(void);
extern void lwip_system_uninit(void);
extern BTS2S_ETHER_ADDR bconvbd2etherbig(BTS2S_BD_ADDR *bd);
extern void pan_reg_req(U16 conn_hdl, U16 data_hdl, BTS2S_BD_ADDR local_bd);





#endif

#ifdef __cplusplus
}
#endif
#endif
