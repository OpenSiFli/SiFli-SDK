/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _NETIF_DEF_H_
#define _NETIF_DEF_H_

#include "co_list.h"
#include "rtos_port.h"
#include"aic_plat_net.h"

/* net_if param define */
#define NX_NB_L2_FILTER         (2)
//#define NET_IF_UAP_NCPYBUF      (1)

typedef enum
{
    TCPIP_PKTTYPE_NULL = 0,
    TCPIP_PKTTYPE_IP,                   /* packet is encapsulated as IP */
    TCPIP_PKTTYPE_ETHER,                /* packet is encapsulated as Ethernet */
    TCPIP_PKTTYPE_PPP,                  /* packet is encapsulated as PPP */
    TCPIP_PKTTYPE_IP6,                  /* packet is encapsulated as IPv6 only */
    TCPIP_PKTTYPE_VOIP,                 /* bug 323266 for IMS service */
    TCPIP_PKTTYPE_FILTER,               /* for XCAP SS, share the same PDN with AP */
    TCPIP_PKTTYPE_SIPCVOLTEAUDIO,
    TCPIP_PKTTYPE_IMSBR,                /* packet is from ims bridge */
    TCPIP_PKTTYPE_MAX = 0xFF
} TCPIP_PACKET_TYPE_E;

typedef uint32_t                        TCPIP_NETID_T;
typedef struct packet_info_tag
{
    #ifdef CONFIG_SDIO_ADMA
    uint8_t*                data_oprt;
    #endif
    uint8_t*                data_ptr;       /* data pointer */
    uint32_t                data_len;       /* data length - full packet encapsulation length */
    TCPIP_PACKET_TYPE_E     pkt_type;       /* packet type */
    TCPIP_NETID_T           net_id;         /* net interface ID */
    //uint32_t                link_handle;    /* link layer handle */
    struct packet_info_tag *frag_next; /* pointer to the next frag, if it has the frag */
} TCPIP_PACKET_INFO_T;

typedef struct netif                    net_if_t;
struct l2_filter_tag
{
    net_if_t *net_if;
    int       sock;
    uint16_t  ethertype;
    struct    fhost_cntrl_link *link;
};

struct net_tx_buf_tag
{
    /// Chained list element
    struct co_list_hdr  hdr;
    TCPIP_PACKET_INFO_T pkt_hdr;
    #if NET_IF_UAP_NCPYBUF
    uint8_t *buf;
    void    *pmsg;
    #else
    uint8_t  buf[1600];
    #endif
};

struct aic_netif_dev
{
    bool                 net_inited;                //set false
    char                 netif_num;
    rtos_mutex           net_tx_buf_mutex;
    struct co_list       net_tx_buf_free_list;

    net_if_t            *wifi_uap_netif;

    /* wpa l2 packet */
    rtos_mutex           l2_mutex;
    rtos_semaphore       l2_semaphore;
    rtos_semaphore       net_tx_buf_sema;
    struct l2_filter_tag l2_filter[NX_NB_L2_FILTER];

    #ifdef CONFIG_RX_WAIT_LWIP
    rtos_semaphore       fhost_rx_lwip_sema;
    #endif
};

/* ASR Platform ADMA Struct */
#ifdef CONFIG_RX_NOCOPY
typedef struct asr_lwip_buf
{
    void *buf;
    void *msg;
} ASR_LWIP_BUF;
#endif /* CONFIG_RX_NOCOPY */

typedef int                 net_buf_rx_t;
typedef TCPIP_PACKET_INFO_T net_buf_tx_t;
typedef void (*net_buf_free_fn)(void *net_buf);

#endif /* _NETIF_DEF_H_ */