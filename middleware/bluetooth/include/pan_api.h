/*
*********************************************************************************************************
* Copyright (C) 2006-2021 Lianway Corporation
*
* Introduction:
*       The purpose of design is to provide pan API functions.
*
* File : pan_api.h
*
* History:
*
*********************************************************************************************************
*/

#ifndef _PAN_API_H_
#define _PAN_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#define PAN_NO_ROLE                 ((U16) (0x00))
#define PAN_NAP_ROLE                ((U16) (0x01))
#define PAN_GN_ROLE                 ((U16) (0x02))
#define PAN_PANU_ROLE               ((U16) (0x04))

#define PAN_ID_DEST_ADDR            ((U16) (0x01))
#define PAN_ID_MULTI_TO_LOCAL       ((U16) (0x01))
#define PAN_ID_MULTI_NO_LOCAL       ((U16) (0x00))

#define NETW_PKT_TYPE_NUM    50

enum
{
    BTS2MD_PAN_REG_REQ = BTS2MD_START,
    BTS2MD_PAN_ENB_REQ,
    BTS2MD_PAN_CONN_REQ,
    BTS2MD_PAN_DATA_REQ,
    BTS2MD_PAN_MULTICAST_DATA_REQ,
    BTS2MD_PAN_DISC_REQ,
    BTS2MD_PAN_DISC_RSP,
    BTS2MD_PAN_SVC_SRCH_REQ,

    BTS2MU_PAN_ENB_CFM = BTS2MU_START,
    BTS2MU_PAN_CONN_IND,
    BTS2MU_PAN_DATA_IND,
    BTS2MU_PAN_DISC_IND,
    BTS2MU_PAN_STS_IND,
    BTS2MU_PAN_SVC_SRCH_CFM,
    BTS2MU_PAN_SVC_SRCH_RSP_IND
};

#define PAN_LINK_ST_EV           ((U8) (0))
#define PAN_SWITCH_ROLE_EV       ((U8) (1))

#define BTS2MD_PAN_MSG_NUM        (BTS2MD_PAN_SVC_SRCH_REQ - BTS2MD_START + 1)
#define BTS2MU_PAN_MSG_NUM        (BTS2MU_PAN_SVC_SRCH_RSP_IND - BTS2MU_START + 1)

typedef struct
{
    U8 load_factor;
    U16 prot_version;
    U16 bluetooth_prof_version;
    U16 secu_desp;
    U16 num_of_language_elems;
    U16 num_of_supp_netw_pkt_types;
    U16 net_access_type;
    U16 supp_netw_pkt_type[NETW_PKT_TYPE_NUM];
    BOOL net_access_type_included;
    BOOL max_net_access_rate_included;
    BOOL load_factor_included;
    BOOL ipv4_subnet_included;
    BOOL ipv6_subnet_included;
    U32 max_net_access_rate;
    BTS2S_DEV_NAME svc_name;
    BTS2S_DEV_NAME svc_desp;
    BTS2S_DEV_NAME ipv4subnet;
    BTS2S_DEV_NAME ipv6subnet;
    BTS2S_LANGUAGE_ELEM language_used[ELEM_LANGUAGE_MAX_NUM];
} BTS2S_PAN_SRCH_RECORD;

typedef struct
{
    U16 type;
} BTS2S_PAN_RESET;

typedef struct
{
    U16 type;
    U16 conn_phdl;
    U16 data_phdl;
    BTS2S_BD_ADDR  local_bd;
} BTS2S_PAN_REG_REQ;

typedef struct
{
    U16 type;
    BOOL single_user;
    U16 local_role;
    U16 rmt_role;
} BTS2S_PAN_ENB_REQ;

typedef struct
{
    U16 type;
    U8 res;
} BTS2S_PAN_ENB_CFM;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd_addr;
    U16 local_role;
    U16 rmt_role;
} BTS2S_PAN_CONN_REQ;

typedef struct
{
    U16 type;
    U8 res;
    U16 id;
    BTS2S_BD_ADDR bd_addr;
    U16 local_role;
    U16 rmt_role;
} BTS2S_PAN_CONN_IND;

typedef struct
{
    U16 type;
    U16 id;
} BTS2S_PAN_DISC_REQ;

typedef struct
{
    U16 type;
    U16 id;
} BTS2S_PAN_DISC_RSP;

typedef struct
{
    U16 type;
    U16 id;
    U8 res;
    BTS2S_BD_ADDR bd;
} BTS2S_PAN_DISC_IND;

typedef struct
{
    U16 type;
    U16 id;
    U16 ether_type;
    U16 len;
    BTS2S_ETHER_ADDR dst_addr;
    BTS2S_ETHER_ADDR src_addr;
    U8 *payload;
} BTS2S_PAN_DATA_REQ;

typedef struct
{
    U16 type;
    U16 id;
    U16 ether_type;
    BTS2S_ETHER_ADDR dst_addr;
    BTS2S_ETHER_ADDR src_addr;
    U16 len;
    U8 *payload;
} BTS2S_PAN_DATA_IND;

typedef struct
{
    U16 type;
    U16 id_not;
    U16 ether_type;
    BTS2S_ETHER_ADDR dst_addr;
    BTS2S_ETHER_ADDR src_addr;
    U16 len;
    U8 *payload;
} BTS2S_PAN_MULTICAST_DATA_REQ;

typedef struct
{
    U16 type;
    U16 id;
    U8 ev;
    U8 sts;
} BTS2S_PAN_STS_IND;

typedef struct
{
    U16 type;
    U16 phdl;
    BTS2S_BD_ADDR bd_addr;
    U16 srch_role;
} BTS2S_PAN_SVC_SRCH_REQ;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd_addr;
    U8 res;
} BTS2S_PAN_SVC_SRCH_CFM;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd_addr;
    BOOL more_ress;
    U16 srch_role;
    BTS2S_PAN_SRCH_RECORD srch_res;
} BTS2S_PAN_SVC_SRCH_RSP_IND;

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to registe application task id to PAN profile.
 *
 * INPUT:
 *      U16 conn_hdl: Always BTS2T_APP
 *      U16 data_hdl: Always BTS2T_PAN
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      None
 *
 *----------------------------------------------------------------------------*/
void pan_reg_req(U16 conn_hdl, U16 data_hdl, BTS2S_BD_ADDR local_bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to enable PAN service.
 *
 * INPUT:
 *      BOOL single_user_mode: single mode or multiple mode.
 *      U16  local_role: local device role(panu,gn or nap)
 *      U16  rmt_role: remote device role(panu,gn or nap)
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PAN_ENB_CFM with structure
 *      BTS2S_PAN_ENB_CFM will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void pan_enb_req(BOOL single_user_mode,
                 U16  local_role,
                 U16  rmt_role);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to connect remote PAN device.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd_addr: remote BT device address.
 *      U16  local_role: local device role(panu,gn or nap)
 *      U16  rmt_role: remote device role(panu,gn or nap)
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PAN_CONN_IND with structure
 *      BTS2S_PAN_CONN_IND will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void pan_conn_req(BTS2S_BD_ADDR *bd_addr,
                  U16 local_role,
                  U16 rmt_role);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to disconnet with remote PAN device
 *
 * INPUT:
 *      U16 conn_hdl: connect device id.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PAN_DISC_IND with structure
 *      BTS2S_PAN_DISC_IND will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void pan_disc_req(U16 conn_hdl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to send data to remote device
 *
 * INPUT:
 *      U16 conn_hdl: Always 1
 *      U16 ethernet_type: ethernet type.
 *      BTS2S_ETHER_ADDR *dst_addr: desternation address.
 *      BTS2S_ETHER_ADDR *src_addr: source address.
 *      U16 len: the length of data.
 *      U8 *payload: pointer of data.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PAN_DATA_IND with structure
 *      BTS2S_PAN_DATA_IND will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void pan_data_req(U16 conn_hdl,
                  U16 ethernet_type,
                  BTS2S_ETHER_ADDR *dst_addr,
                  BTS2S_ETHER_ADDR *src_addr,
                  U16 len,
                  U8 *payload);
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function is used to inqury remote pan device service.
 *
 * INPUT:
 *      U16 conn_hdl: Always BTS2T_APP
 *      BTS2S_ETHER_ADDR *bd_addr: desternation address.
 *      U16 role: remote device service.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      Message BTS2MU_PAN_SVC_SRCH_CFM with structure
 *      BTS2S_PAN_SVC_SRCH_CFM will be received as a confirmation.
 *      Message BTS2MU_PAN_SVC_SRCH_RSP_IND with structure
 *      BTS2S_PAN_SVC_SRCH_RSP_IND will be received as a confirmation.
 *
 *----------------------------------------------------------------------------*/
void pan_svc_srch_req(U16 conn_hdl,
                      BTS2S_BD_ADDR *bd_addr,
                      U16 role);


#ifdef __cplusplus
}
#endif

#endif

