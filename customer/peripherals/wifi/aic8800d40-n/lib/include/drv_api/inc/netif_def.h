/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NETIF_DEF_H_
#define _NETIF_DEF_H_

#include "co_list.h"
#include "rtos_port.h"
#include "aic_netbuf.h"  /* net_if_t, net_buf_tx_t */

/* Forward declaration - opaque, defined in host library */
struct fhost_cntrl_link;

/* net_if param define */
#define NX_NB_L2_FILTER         (2)
/* [PLATFORM] nocopy TX 开关: 1 = tx_buf 直接引用 LwIP pbuf（省 1600B 中间拷贝），
 * 依赖平台 LwIP 的 PBUF_LINK_ENCAPSULATION_HLEN >= 28 预留 hostdesc headroom
 * （hostdesc 写入起始于 payload-14 处，须落在 pbuf 内存内；
 *  若平台未满足该配置，逐帧判定 headroom 不足自动回退 copy，
 *  功能不受影响但无零拷贝收益，可用 aicwf_platform_nocopy_aggr_copy 命中情况确认） */
#define NET_IF_UAP_NCPYBUF      (1)

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
    uint8_t                *data_oprt;
#endif
    uint8_t                *data_ptr;       /* data pointer */
    uint32_t                data_len;       /* data length - full packet encapsulation length */
    TCPIP_PACKET_TYPE_E     pkt_type;       /* packet type */
    TCPIP_NETID_T           net_id;         /* net interface ID */
    //uint32_t                link_handle;    /* link layer handle */
    struct packet_info_tag *frag_next; /* nocopy TX: pbuf 链指针（透传到 SDIO 聚合 scatter copy）；
                                          copy 模式: 未使用，恒为 NULL */
} TCPIP_PACKET_INFO_T;

/* net_if_t is typedef void * — defined in aic_netbuf.h */
struct l2_filter_tag
{
    net_if_t  net_if;   /* opaque handle (void *) */
    int       sock;
    uint16_t  ethertype;
    struct    fhost_cntrl_link *link;
};

struct net_tx_buf_tag
{
    /// Chained list element
    struct co_list_hdr  hdr;
    net_buf_tx_t        pkt_hdr;
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
    uint16_t             net_tx_buf_cnt;
    rtos_mutex           net_tx_buf_mutex;
    struct co_list       net_tx_buf_free_list;

    net_if_t             wifi_uap_netif;  /* opaque handle (void *) */

    /* wpa l2 packet */
    rtos_mutex           l2_mutex;
    rtos_semaphore       l2_semaphore;
    rtos_semaphore       net_tx_buf_sema;
    struct l2_filter_tag l2_filter[NX_NB_L2_FILTER];

#ifdef CONFIG_RX_WAIT_LWIP
    rtos_semaphore       fhost_rx_lwip_sema;
#endif

    uint32_t        used_ip;
    uint32_t        netmask;
};

typedef void (*net_buf_free_fn)(void *net_buf);

#endif /* _NETIF_DEF_H_ */

