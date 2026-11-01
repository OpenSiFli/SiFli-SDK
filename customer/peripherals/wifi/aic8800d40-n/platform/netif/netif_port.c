/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//-------------------------------------------------------------------
// Driver Header Files
//-------------------------------------------------------------------
#include "co_math.h"
#include "netif_api.h"
#include "wifi_api.h"
#include "rtos_port.h"
#include "netif_port.h"
#include "aic_log.h"
#include "aic_assert.h"
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"
#include "lwip/init.h"
/* [AIC8800-PORT][SiFli] lwIP headers used directly by route 1: included
 * transitively in the baseline, not by SiFli lwIP-2.1.2; options enabled in lwipopts.h */
#include "lwip/netif.h"      /* struct netif, netif_linkoutput_fn */
#include "lwip/netifapi.h"   /* netifapi_netif_add, netifapi_dhcp_start/stop/release */
#include "lwip/etharp.h"     /* etharp_output, ETHARP_HWADDR_LEN */
#include "lwip/dhcp.h"       /* dhcp_supplied_address */

#ifdef AIC8800_USE_RT_WLAN
    #include "wlan_dev.h"  /* rt_wlan_dev_report_data, struct rt_wlan_device */
    extern struct rt_wlan_device *aic8800_wlan_get_device(void);
    extern struct rt_wlan_device *aic8800_wlan_get_ap_device(void);
#endif

/* host layer byte-order / inet utilities (aic_sock.c) */
extern int aic_inet_aton(const char *cp, void *addr);

#define NETIF_PKT_DHCP  0
#define NETIF_PKT_PING  1

/* net_if buffer */
#define NET_HEAD_OFFSET         (14)
#define ERR_OK                  (0)


#if NET_IF_UAP_NCPYBUF
    #define NET_TXBUF_CNT           (32*2)
#else
    #define NET_TXBUF_CNT           (28)
#endif
#define NET_TXBUF_TH            (NET_TXBUF_CNT/2-2)
#define NET_RX_LWIP_CNT         (40)

/**
 * @brief DHCP packet tracing
 *
 * Parses a contiguous Ethernet frame layer by layer per protocol
 * (ETH+IPv4+UDP+DHCP), validates the magic cookie, then scans option 53 for
 * the msg type and prints xid/yiaddr/chaddr.
 * Replaces the old hard-coded offset implementation (breaks on frame length change).
 *
 * @param data frame start (contiguous; caller linearizes/skips chained pbufs)
 * @param len  frame length
 * @param tag  log tag (e.g. "send"/"recv")
 */
static void aicwf_dhcp_trace(uint8_t *f, uint32_t len, const char *tag)
{
    static const char *const type_str[] =
    {
        "", "DISCOVER", "OFFER", "REQUEST", "DECLINE",
        "ACK", "NAK", "RELEASE", "INFORM"
    };
    uint32_t ip, udp, dhcp, opt, xid, yiaddr;
    uint16_t dport;
    uint8_t ihl, msg_type = 0;

    if (f == NULL || len < 46)
        return;
    if (f[12] != 0x08 || f[13] != 0x00)         /* ETH_P_IP */
        return;
    ihl = (uint8_t)(f[14] & 0x0f) * 4;
    if (ihl < 20 || f[14 + 9] != 17)            /* proto != UDP */
        return;
    ip = 14;
    udp = ip + ihl;
    if (len < udp + 8 + 240)                    /* BOOTP fixed header 236 + cookie 4 */
        return;
    dport = (uint16_t)((f[udp + 2] << 8) | f[udp + 3]);
    if (dport != 67 && dport != 68)             /* DHCP server/client */
        return;
    dhcp = udp + 8;
    if (f[dhcp + 236] != 0x63 || f[dhcp + 237] != 0x82 ||
            f[dhcp + 238] != 0x53 || f[dhcp + 239] != 0x63)  /* magic cookie */
        return;
    xid = ((uint32_t)f[dhcp + 4] << 24) | ((uint32_t)f[dhcp + 5] << 16) |
          ((uint32_t)f[dhcp + 6] << 8) | f[dhcp + 7];
    yiaddr = ((uint32_t)f[dhcp + 16] << 24) | ((uint32_t)f[dhcp + 17] << 16) |
             ((uint32_t)f[dhcp + 18] << 8) | f[dhcp + 19];
    /* option 53 (msg type): scan after the cookie, skip Pad/other options */
    opt = dhcp + 240;
    while (opt + 1 < len)
    {
        if (f[opt] == 0)                        /* Pad */
        {
            opt++;
            continue;
        }
        if (f[opt] == 255)                      /* End */
            break;
        if (f[opt] == 53 && opt + 2 < len)
        {
            msg_type = f[opt + 2];
            break;
        }
        opt += 2 + f[opt + 1];
    }
    if (msg_type >= 1 && msg_type <= 8)
    {
        DBG_NETAL_INF("%s DHCP %s xid=0x%08x yiaddr=0x%08x chaddr=%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                      tag, type_str[msg_type], xid, yiaddr,
                      f[dhcp + 28], f[dhcp + 29], f[dhcp + 30],
                      f[dhcp + 31], f[dhcp + 32], f[dhcp + 33]);
    }
}

//-------------------------------------------------------------------
// Driver Variables define
//-------------------------------------------------------------------
static bool volatile net_out_ing = false;
struct aic_netif_dev netif_dev = {0};

/*
 * Platform-internal netif pool.
 *
 * net_if_t (void*) stores struct netif* directly (a slot in s_netif_pool).
 * No extra handle→slot mapping array is needed.
 * Index 0 = STA vif (default).
 *
 * Allocation and type conversion rules:
 *  - netif_alloc_for_handle(&h): if *p_h is not already inside the pool, allocates
 *    a free slot, stores the slot address in *p_h and returns that slot.
 *  - netif_from_handle(h): checks whether h falls inside s_netif_pool and, if so,
 *    returns it cast directly to (struct netif*).
 *  - netif_handle_in_pool(h): checks whether h falls inside the pool.
 */
#define PLAT_NETIF_POOL_SIZE  4
static struct netif s_netif_pool[PLAT_NETIF_POOL_SIZE];

/**
 * @brief Check whether the handle points to a slot in s_netif_pool
 *
 * Compares the pointer address range, no mapping table walk needed.
 *
 * @param h netif handle (actually a struct netif*)
 * @return true=inside the pool range, false=not inside
 */
static bool netif_handle_in_pool(net_if_t h)
{
    struct netif *nif = (struct netif *)h;
    return (nif >= &s_netif_pool[0] &&
            nif <  &s_netif_pool[PLAT_NETIF_POOL_SIZE]);
}

/**
 * @brief Type conversion: net_if_t → struct netif*
 *
 * The handle itself is a pointer to a pool slot; cast it directly once the
 * range check passes. Returns NULL when out of range.
 */
static struct netif *netif_from_handle(net_if_t h)
{
    if (netif_handle_in_pool(h))
        return (struct netif *)h;
    DBG_NETAL_ERR("%s: handle %p not in pool\r\n", __func__, h);
    return NULL;
}

/**
 * @brief Allocate a free slot from the pool and write its address to *p_h
 *
 * Logic:
 *  1. If *p_h is already inside the pool → already allocated, return that slot.
 *  2. If not inside the pool → find a free slot (all zero), clear it and store
 *     its address in *p_h.
 *
 * Allocation runs single-threaded during driver init, so no lock is needed.
 *
 * @param p_h  pointer to the handle variable (in/out: set to slot address on success)
 * @return allocated netif slot pointer, NULL when the pool is full
 */
static struct netif *netif_alloc_for_handle(net_if_t *p_h)
{
    int i;

    if (!p_h)
        return NULL;

    /* 1. *p_h already in the pool → return it directly */
    if (netif_handle_in_pool(*p_h))
        return (struct netif *)*p_h;

    /* 2. Allocate a free slot */
    for (i = 0; i < PLAT_NETIF_POOL_SIZE; i++)
    {
        /* Free slot test: not initialized (num==0 and output==NULL) */
        if (s_netif_pool[i].output == NULL)
        {
            memset(&s_netif_pool[i], 0, sizeof(struct netif));
            *p_h = (net_if_t)&s_netif_pool[i];
            return &s_netif_pool[i];
        }
    }

    DBG_NETAL_ERR("%s: pool full\n", __func__);
    return NULL;
}

//-------------------------------------------------------------------
// Driver Import functions
//-------------------------------------------------------------------
/* Platform functions */


//-------------------------------------------------------------------
// Driver platform functions
//-------------------------------------------------------------------
int aicwf_tx_eth_data_process(uint8_t *data, uint32_t len, void *msg);
int aicwf_tx_eth_data_process_if(net_if_t net_if, uint8_t *data, uint32_t len, void *msg);
/**
 * @brief Allocate an RX network buffer (pbuf)
 *
 * Allocates a raw packet of the requested size from the LwIP pbuf pool.
 *
 * @param length requested buffer size (bytes)
 * @return pbuf pointer, NULL on failure
 */
void *aicwf_platform_net_buf_rx_alloc(uint32_t length)
{
    struct pbuf *p_buf = pbuf_alloc(PBUF_RAW, length, PBUF_POOL);

    if (p_buf)
    {
        return (void *)p_buf;
    }
    return NULL;
}

/**
 * @brief RX zero-copy fallback: allocate a pbuf and copy the Ethernet frame into its payload
 *
 * The host layer does not own the lwip headers, so the platform layer performs
 * alloc + copy. Returns a pbuf pointer (same semantics as
 * aicwf_platform_net_buf_rx_alloc), NULL on failure.
 *
 * @param eth_data Ethernet frame start
 * @param len      frame length
 * @return pbuf pointer, NULL on failure
 */
void *aicwf_platform_net_buf_rx_alloc_copy(const uint8_t *eth_data, uint32_t len)
{
    struct pbuf *p_buf = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

    if (p_buf == NULL)
    {
        DBG_NETAL_ERR("%s alloc fail len:%u\n", __func__, len);
        return NULL;
    }
    memcpy(p_buf->payload, eth_data, len);
    return (void *)p_buf;
}

/**
 * @brief Get the descriptor pointer preceding the buffer
 *
 * @param buffer data buffer address
 * @return descriptor pointer (buffer - sizeof(void*))
 */
void *aicwf_platform_net_buf_rx_desc(uint8_t *buffer)
{
    return (void *)(buffer - sizeof(void *));
}

/**
 * @brief Free an RX network buffer (pbuf)
 *
 * Note: pbuf_free() may only be called here when the upstream path has
 *       already called pbuf_ref() to bump the reference count. If upstream
 *       did not call pbuf_ref(), the pbuf is owned by LwIP.
 *
 * @param ref pbuf pointer
 */

void aicwf_platform_net_buf_tx_free(void *ref)
{
#if NET_IF_UAP_NCPYBUF
    /* nocopy: pbuf_ref() was called on TX, release the driver-side reference.
     * Only called for frames already queued (pmsg); drop/fallback copy paths must not call it */
    if (ref)
    {
        pbuf_free((struct pbuf *)ref);
    }
#else
    /* no matching pbuf_ref(), must not free to avoid a double free */
    // pbuf_free((struct pbuf *)ref);
    (void)ref;
#endif
}

#if NET_IF_UAP_NCPYBUF
/**
 * @brief nocopy TX aggregation-stage copy hook (called by host lib sdio_co.c)
 *
 * Single-segment pbuf: plain memcpy; chained pbuf: scatter copy per segment.
 * src points to the start of the contiguous logical view of
 * [hostdesc(28B) + payload without the Ethernet header],
 * where [src+14, src+28) overlaps the first 14B (Ethernet header) of the first
 * pbuf segment; fhost_tx has already written the hostdesc there before the
 * call, so copying len+14 for the first segment is enough.
 *
 * @param pmsg pbuf chain head pointer
 * @param src  logical contiguous view start (= first segment payload - 14)
 * @param len  total length (= tot_len + 14)
 * @param dst  aggregation buf write position
 */
void aicwf_platform_nocopy_aggr_copy(void *pmsg, const uint8_t *src,
                                     uint32_t len, uint8_t *dst)
{
    struct pbuf *p = (struct pbuf *)pmsg;

    if (p->next == NULL)
    {
        /* single segment: memory is already contiguous */
        memcpy(dst, src, len);
        return;
    }

    /* chained: first segment [src, src + p->len + 14) covers hostdesc + all first-segment data */
    uint32_t first_len = p->len + NET_HEAD_OFFSET;
    memcpy(dst, src, first_len);
    dst += first_len;

    struct pbuf *q;
    for (q = p->next; q != NULL; q = q->next)
    {
        memcpy(dst, q->payload, q->len);
        dst += q->len;
    }
}
#endif /* NET_IF_UAP_NCPYBUF */

#ifdef CONFIG_RX_NOCOPY
#if LWIP_SUPPORT_CUSTOM_PBUF
/* host layer layer_common symbols (implemented in hal_co.c; platform layer does not include hal_co.h) */
extern void aicwf_hal_buf_ref(void *node);
extern void aicwf_hal_buf_unref(void *node);

/* [PLATFORM] RX zero-copy: small pool of custom pbuf headers. Not written
 * into the SDIO shared buffer body (control area is read-only), allocated from
 * a dedicated static pool; pbuf_custom has no arg field, so a wrapper carries node. */
#define AICWF_RX_ZCOPY_HDR_CNT  96   /* >= SDIO_RX_BUF_COUNT_DEFAULT, avoid running dry before the node pool */

struct aicwf_rx_zcopy_hdr
{
    struct pbuf_custom pc;
    void *node;               /* SDIO RX node holding a reference */
    struct aicwf_rx_zcopy_hdr *next;  /* free list link */
};

static struct aicwf_rx_zcopy_hdr rx_zcopy_hdr_pool[AICWF_RX_ZCOPY_HDR_CNT];
static struct aicwf_rx_zcopy_hdr *rx_zcopy_free_list;
static rtos_mutex rx_zcopy_mutex;
static uint8_t rx_zcopy_inited;
static uint32_t rx_zcopy_fallback_cnt;  /* fallback-to-copy count when the header pool runs dry */

static void aicwf_rx_zcopy_init(void)
{
    int i;

    if (rx_zcopy_inited)
    {
        return;
    }
    rx_zcopy_free_list = NULL;
    for (i = AICWF_RX_ZCOPY_HDR_CNT - 1; i >= 0; i--)
    {
        rx_zcopy_hdr_pool[i].next = rx_zcopy_free_list;
        rx_zcopy_free_list = &rx_zcopy_hdr_pool[i];
    }
    aicwf_rtos_mutex_create(&rx_zcopy_mutex, "rx_zcopy.mutex");
    rx_zcopy_inited = 1;
}

/* Called back when lwip finishes consuming the pbuf (ref==0): reclaim the header
 * and release the node reference; may run in any context, unref is locked. */
static void aicwf_rx_zcopy_pbuf_free(struct pbuf *p)
{
    struct aicwf_rx_zcopy_hdr *hdr = (struct aicwf_rx_zcopy_hdr *)p;

    aicwf_hal_buf_unref(hdr->node);

    aicwf_rtos_mutex_lock(rx_zcopy_mutex, -1);
    hdr->node = NULL;
    hdr->pc.custom_free_function = NULL;
    hdr->next = rx_zcopy_free_list;
    rx_zcopy_free_list = hdr;
    aicwf_rtos_mutex_unlock(rx_zcopy_mutex);
}

/**
 * @brief RX zero-copy hook: build a custom pbuf pointing at the Ethernet frame in the SDIO buffer
 *
 * @param eth_data frame start (Ethernet header), inside the SDIO RX node buffer
 * @param len      frame length
 * @param node     SDIO RX node owning the frame (void*, may be NULL under USB config → fallback)
 * @return pbuf pointer; NULL on failure, the caller then falls back to the copy path
 */
void *aicwf_platform_rx_zero_copy(uint8_t *eth_data, uint32_t len, void *node)
{
    struct aicwf_rx_zcopy_hdr *hdr;
    struct pbuf *p;

    if (!rx_zcopy_inited)
    {
        aicwf_rx_zcopy_init();
    }
    if (!eth_data || !len || !node)
    {
        return NULL;
    }

    aicwf_rtos_mutex_lock(rx_zcopy_mutex, -1);
    hdr = rx_zcopy_free_list;
    if (hdr)
    {
        rx_zcopy_free_list = hdr->next;
    }
    aicwf_rtos_mutex_unlock(rx_zcopy_mutex);

    if (!hdr)
    {
        rx_zcopy_fallback_cnt++;
        if ((rx_zcopy_fallback_cnt & 0xFF) == 1)
        {
            DBG_NETAL_WRN("rx_zcopy hdr pool dry, fallback cnt=%u\n",
                          rx_zcopy_fallback_cnt);
        }
        return NULL;
    }

    hdr->node = node;
    hdr->pc.custom_free_function = aicwf_rx_zcopy_pbuf_free;
    /* PBUF_RAW: no protocol header offset, payload points at the Ethernet header; len must fit u16 */
    p = pbuf_alloced_custom(PBUF_RAW, (u16_t)len, PBUF_RAM, &hdr->pc,
                            eth_data, (u16_t)len);
    if (p == NULL)
    {
        /* bad parameters (only when length>payload_mem_len): return the header and fall back */
        aicwf_rtos_mutex_lock(rx_zcopy_mutex, -1);
        hdr->node = NULL;
        hdr->pc.custom_free_function = NULL;
        hdr->next = rx_zcopy_free_list;
        rx_zcopy_free_list = hdr;
        aicwf_rtos_mutex_unlock(rx_zcopy_mutex);
        return NULL;
    }

    aicwf_hal_buf_ref(node);
    return (void *)p;
}

/**
 * @brief RX zero-copy deinit: free the custom pbuf header pool + mutex + state reset
 *
 * Must be called before the SDIO/USB node pool is freed (aicwf_sdio_buf_deinit);
 * in the normal sequence all custom pbufs have been consumed by lwip at this
 * point (custom_free_fn returns the headers), so the header pool should be fully
 * back on free_list. If any header is still in use, some pbuf was not consumed;
 * freeing the node pool then would make custom_free_fn later unref a dangling
 * node → use-after-free.
 */
void aicwf_rx_zcopy_deinit(void)
{
    int free_cnt = 0;
    struct aicwf_rx_zcopy_hdr *h;

    if (!rx_zcopy_inited)
    {
        return;
    }

    /* Verify the header pool is fully idle: free_list should hold all AICWF_RX_ZCOPY_HDR_CNT headers */
    h = rx_zcopy_free_list;
    while (h)
    {
        free_cnt++;
        h = h->next;
    }
    if (free_cnt != AICWF_RX_ZCOPY_HDR_CNT)
    {
        DBG_NETAL_WRN("rx_zcopy deinit: hdr not all free (%d/%d), node free may UAF!\n",
                      free_cnt, AICWF_RX_ZCOPY_HDR_CNT);
    }

    aicwf_rtos_mutex_delete(rx_zcopy_mutex);
    rx_zcopy_free_list = NULL;
    rx_zcopy_inited = 0;
    rx_zcopy_fallback_cnt = 0;
}
#else /* !LWIP_SUPPORT_CUSTOM_PBUF: platform lwip lacks custom pbuf, always fall back to copy */
/* [PLATFORM] RX zero-copy hook fallback: platform lwip does not enable
 * LWIP_SUPPORT_CUSTOM_PBUF, so no custom pbuf can be built; always return NULL
 * to make the host take the alloc+memcpy copy path. */
void *aicwf_platform_rx_zero_copy(uint8_t *eth_data, uint32_t len, void *node)
{
    (void)eth_data;
    (void)len;
    (void)node;
    return NULL;
}

void aicwf_rx_zcopy_deinit(void)
{
}
#endif /* LWIP_SUPPORT_CUSTOM_PBUF */
#endif /* CONFIG_RX_NOCOPY */

void aicwf_platform_net_buf_rx_free(void *ref)
{
    pbuf_free((struct pbuf *)ref);
}

/**
 * @brief Pass the packet to the LwIP stack input
 *
 * Injects the pbuf into LwIP through the netif->input callback.
 *
 * @param data  data pointer (not used directly)
 * @param len   data length (not used directly)
 * @param desc  pbuf descriptor
 * @param netif netif handle
 * @return ERR_OK(0)=success, other=LwIP error code
 */
static int __aicwf_aicwifi_pkt_send_to_platform(uint8_t *data, uint32_t len, uint8_t *desc, net_if_t netif)
{
    struct pbuf *p_buf = (struct pbuf *)desc;
    struct netif *nif = netif_from_handle(netif);
    int errcode;

    //p_buf->payload = data;
    //p_buf->len = len;

#if LWIP_TCPIP_CORE_LOCKING
    /* [PLATFORM] The RX direct-inject path runs in the fhost task context and
     * races with the tcpip thread; with LWIP_TCPIP_CORE_LOCKING enabled the core
     * lock must be held to enter the stack (see lwipopts.md) */
    LOCK_TCPIP_CORE();
#endif /* LWIP_TCPIP_CORE_LOCKING */
    errcode = nif->input(p_buf, nif);
#if LWIP_TCPIP_CORE_LOCKING
    UNLOCK_TCPIP_CORE();
#endif /* LWIP_TCPIP_CORE_LOCKING */
    if (errcode != ERR_OK)
    {
        DBG_NETAL_ERR("netif input failed, errcode = %d/n", errcode);
    }
    return errcode;
}

/**
 * @brief Send data to the WiFi UAP in zero-copy mode
 *
 * Forwards directly to aicwf_tx_eth_data_process.
 *
 * @param data data pointer
 * @param len  data length
 * @param msg  associated pbuf message
 * @return processing result
 */
static unsigned long aicwf_netif_send_to_wifi_uap_nocpy(uint8_t *data, uint32_t len, void *msg)
{
    return aicwf_tx_eth_data_process(data, len, msg);
}

/**
 * @brief LwIP netif linkoutput callback
 *
 * Called by the LwIP ARP layer to send a packet to the WiFi driver.
 *
 * @param net_if netif handle
 * @param p_buf  pbuf to send
 * @return processing result
 */
static err_t __aicwf_net_if_output(net_if_t net_if, struct pbuf *p_buf)
{
    /* Chained pbuf (e.g. [head pbuf]→[REF payload]) must pass tot_len:
     * the first segment len is only the header length; using it would make the
     * chip send a short MSDU (truncated frame) */
    /* net_if pass-through: in concurrent mode SoftAP/P2P (vif2) downlink uses its own interface */
    return aicwf_tx_eth_data_process_if(net_if, p_buf->payload, p_buf->tot_len, p_buf);
}


//-------------------------------------------------------------------
// Driver netif base API define
//-------------------------------------------------------------------
/**
 * @brief Get the netif name
 *
 * Type conversion: net_if_t → struct netif* (via netif_from_handle).
 * Format is "wl" + numeric index.
 *
 * @param net_if netif handle
 * @param buf    output buffer
 * @param len    buffer size
 * @return name length (always 3)
 */
int aicwf_net_if_get_name(net_if_t net_if, char *buf, int len)
{
    struct netif *nif;
    nif = netif_from_handle(net_if);
    if (!nif)
        return 0;
    if (len > 0)
        buf[0] = nif->name[0];
    if (len > 1)
        buf[1] = nif->name[1];
    if (len > 2)
        buf[2] = nif->num + '0';
    if (len > 3)
        buf[3] = '\0';

    return 3;
}

/**
 * @brief Check whether the netif is up
 *
 * Type conversion: net_if_t → struct netif* (via netif_from_handle).
 *
 * @param net_if netif handle
 * @return true=up, false=down or invalid parameter
 */
uint8_t aicwf_net_if_is_up(net_if_t net_if)
{
#ifdef AIC8800_USE_RT_WLAN
    /* Route 2: the netif belongs to the rt_wlan framework; the pool slot was never
     * registered with lwIP (flags stay 0), so its up state is meaningless. The
     * framework already did netif_set_up after eth_device_init (wlan_lwip.c) */
    (void)net_if;
    return true;
#else
    struct netif *nif;
    if (!netif_handle_in_pool(net_if))
    {
        DBG_NETAL_ERR("%s: handle %p not in pool\r\n", __func__, net_if);
        return false;
    }
    nif = netif_from_handle(net_if);
    return netif_is_up(nif);
#endif
}

/**
 * @brief Bring the netif up
 *
 * Type conversion: net_if_t → struct netif* (via netif_from_handle).
 *
 * @param net_if netif handle
 */
void aicwf_net_if_up(net_if_t net_if)
{
#ifdef AIC8800_USE_RT_WLAN
    /* Route 2: the netif is created and brought up by the rt_wlan framework; an
     * unregistered pool slot must not be touched */
    (void)net_if;
    DBG_NETAL_INF("%s: skipped (rt_wlan owns netif)\r\n", __func__);
#else
    struct netif *nif;
    if (!netif_handle_in_pool(net_if))
    {
        DBG_NETAL_ERR("%s: handle %p not in pool, skip\r\n", __func__, net_if);
        return;
    }
    nif = netif_from_handle(net_if);
    netif_set_up(nif);
#endif
}

/**
 * @brief Bring the netif down
 *
 * Type conversion: net_if_t → struct netif* (via netif_from_handle).
 *
 * @param net_if netif handle
 */
void aicwf_net_if_down(net_if_t net_if)
{
#ifdef AIC8800_USE_RT_WLAN
    /* Route 2: the netif is managed by the rt_wlan framework; the pool slot was
     * never registered with lwIP, so no set_down here */
    (void)net_if;
    DBG_NETAL_INF("%s: skipped (rt_wlan owns netif)\r\n", __func__);
    return;
#else
    struct netif *nif;
    int i;
    /* [BUGFIX] (common: netif lifecycle) NULL now means "bring down all active
     * netifs in the pool"; the old implementation skipped NULL as not-in-pool,
     * i.e. did nothing */
    if (net_if == NULL)
    {
        for (i = 0; i < PLAT_NETIF_POOL_SIZE; i++)
        {
            if (s_netif_pool[i].output != NULL)
                netif_set_down(&s_netif_pool[i]);
        }
        return;
    }

    if (!netif_handle_in_pool(net_if))
    {
        DBG_NETAL_ERR("%s: handle %p not in pool, skip\r\n", __func__, net_if);
        return;
    }
    nif = netif_from_handle(net_if);
    netif_set_down(nif);
#endif
}

/**
 * @brief Set the default netif
 *
 * Type conversion: net_if_t → struct netif* (via netif_from_handle).
 *
 * @param net_if netif handle
 */
void aicwf_net_if_set_default(net_if_t net_if)
{
#ifdef AIC8800_USE_RT_WLAN
    /* Route 2: the pool slot was never registered with lwIP and cannot serve as
     * the default interface; the default route is set by the framework in
     * netif_add (for the first interface) */
    (void)net_if;
    DBG_NETAL_INF("%s: skipped (rt_wlan owns netif)\r\n", __func__);
#else
    if (!netif_handle_in_pool(net_if))
    {
        DBG_NETAL_ERR("%s: handle %p not in pool\r\n", __func__, net_if);
        return;
    }
    netif_set_default(netif_from_handle(net_if));
#endif
}

/**
 * @brief Set the netif IP address, netmask and gateway
 *
 * Type conversion: net_if_t → struct netif* (via netif_from_handle).
 *
 * @param net_if netif handle
 * @param ip     IPv4 address
 * @param mask   netmask
 * @param gw     gateway address
 */
void aicwf_net_if_set_ip(net_if_t net_if, uint32_t ip, uint32_t mask, uint32_t gw)
{
#ifdef AIC8800_USE_RT_WLAN
    /* Route 2: the netif belongs to the rt_wlan framework; the pool slot was
     * never registered with lwIP. The STA address is obtained by the framework
     * DHCP client; the SoftAP address is written by the framework dhcpd_start()
     * (DHCPD_SERVER_IP), so the pool slot must not be touched here. */
    (void)net_if;
    (void)ip;
    (void)mask;
    (void)gw;
    DBG_NETAL_INF("%s: skipped (rt_wlan owns netif)\r\n", __func__);
#else
    struct netif *nif;

    if (!netif_handle_in_pool(net_if))
    {
        DBG_NETAL_ERR("%s: handle %p not in pool, skip\r\n", __func__, net_if);
        return;
    }

    nif = netif_from_handle(net_if);
    if (!nif)
        return;

    ip4_addr_t netif_ip;
    ip4_addr_t netif_mask;
    ip4_addr_t netif_gw;
    netif_ip.addr = ip;
    netif_mask.addr = mask;
    netif_gw.addr = gw;
    netif_set_addr(nif, &netif_ip, &netif_mask, &netif_gw);
#endif /* AIC8800_USE_RT_WLAN */
}

/**
 * @brief Get the current netif IP address, netmask and gateway
 *
 * Type conversion: net_if_t → struct netif* (via netif_from_handle).
 *
 * @param net_if_h netif handle (NULL=default interface)
 * @param ip       output IPv4 address (may be NULL)
 * @param mask     output netmask (may be NULL)
 * @param gw       output gateway address (may be NULL)
 * @return 0=success, -1=failure
 */
int aicwf_net_if_get_ip(net_if_t net_if_h, uint32_t *ip, uint32_t *mask, uint32_t *gw)
{
    struct netif *net_if;

    /* [BUGFIX] (common: netif lookup) NULL means the default interface: look it up
     * by the real STA vif (idx 0) instead of assuming pool[0]
     * (on driver re-init/APSTA with AP started first, pool[0] is not the STA) */
    if (net_if_h == NULL)
    {
        net_if_h = net_if_find_from_wifi_idx(0);
        if (!net_if_h)
            return -1;
    }

    if (!netif_handle_in_pool(net_if_h))
    {
        DBG_NETAL_ERR("%s: handle %p not in pool\r\n", __func__, net_if_h);
        return -1;
    }

    net_if = netif_from_handle(net_if_h);
    if (!net_if)
    {
        DBG_NETAL_ERR("%s: netif_from_handle fail\r\n", __func__);
        return -1;
    }

    if (ip)
        *ip = net_if->ip_addr.addr;
    if (mask)
        *mask = net_if->netmask.addr;
    if (gw)
        *gw = net_if->gw.addr;

    return 0;
}

static err_t aicwf_net_if_init(struct netif *net_if);

/**
 * @brief Add a netif to LwIP
 *
 * Allocates a slot from the pool via netif_alloc_for_handle, writes the slot
 * address to *net_if_h, then registers it with LwIP.
 *
 * @param net_if_h pointer to the netif handle (in/out: set to the pool slot address on success)
 * @param ipaddr   IPv4 address
 * @param netmask  netmask
 * @param gw       gateway address
 * @param vif      virtual interface pointer (fhost_vif)
 * @return 0=success, -1=failure
 */
int aicwf_net_if_add(net_if_t *net_if_h, const uint32_t *ipaddr, const uint32_t *netmask, const uint32_t *gw, void *vif)
{
    err_t status;
    struct netif *net_if;

    if (!net_if_h)
    {
        DBG_NETAL_INF("%s net_if_h is NULL\r\n", __func__);
        return -1;
    }

    /* 1. Allocate a slot from the pool; *net_if_h is set to the slot address */
    net_if = netif_alloc_for_handle(net_if_h);
    DBG_NETAL_INF("%s net_if=%p\r\n", __func__, net_if);
    if (!net_if)
    {
        DBG_NETAL_ERR("%s pool full\r\n", __func__);
        return -1;
    }

    /* 2. Check whether it is already registered with LwIP (output is set in the
     * init callback, consistent with the pool free check; num cannot be used —
     * num is fully assigned by lwIP and is no longer a registration flag) */
    if (net_if->output != NULL)
    {
        DBG_NETAL_INF("%s already added to LwIP\r\n", __func__);
        return 0;
    }

    /* 3. Register the interface with the LwIP stack */
#ifndef AIC8800_USE_RT_WLAN
    status = netifapi_netif_add(net_if,
                                (const ip4_addr_t *)ipaddr,
                                (const ip4_addr_t *)netmask,
                                (const ip4_addr_t *)gw,
                                vif,
                                aicwf_net_if_init,
                                tcpip_input);
#else
    /* M2: in AIC8800_USE_RT_WLAN mode the netif is created by the rt_wlan
     * framework (wlan_lwip.c), so the driver skips creating its own netif; the
     * pool slot is still allocated (to satisfy host lib internal assumptions) */
    status = ERR_OK;
#endif

    DBG_NETAL_INF("%s status=%d\r\n", __func__, status);

    return (status == ERR_OK ? 0 : -1);
}

/**
 * @brief LwIP netif init callback
 *
 * Sets the interface name, MAC address, output callback and flags.
 *
 * @param net_if LwIP netif structure
 * @return 0=success
 */
static err_t aicwf_net_if_init(struct netif *net_if)
{
    //err_t status = ERR_OK;
    void *fvif;
    const uint8_t *vmac;

    /* netif_add() already stored the passed-in fhost_vif in state; fall back to vif[0] only when empty */
    fvif = net_if->state;
    if (!fvif)
        fvif = aic_get_fhost_default_vif();
    net_if->state = fvif;

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    net_if->hostname = "AicWlan";
#endif

    net_if->name[ 0 ] = 'w';
    net_if->name[ 1 ] = 'l';

    /* lwIP version visible at boot (logged once for both VIFs): the nocopy TX
     * type-check branch follows the version, >=2.1 uses the type_internal bit
     * check, <=2.0/1.x uses the type enum check */
    {
        static bool lwip_ver_logged = false;
        if (!lwip_ver_logged)
        {
            lwip_ver_logged = true;
            DBG_NETAL_INF("lwip version: %s\n", LWIP_VERSION_STRING);
        }
    }

    net_if->output = etharp_output;
    net_if->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_IGMP;
    net_if->hwaddr_len = ETHARP_HWADDR_LEN;
    net_if->mtu = 1500;
    /* [AIC8800-PORT][SiFli] __aicwf_net_if_output takes net_if_t(void*) as its
     * first argument while lwIP linkoutput takes struct netif*; both are the same
     * value at runtime (the handle is passed through netif_from_handle), but GCC14
     * promotes the incompatible pointer assignment to an error, hence the cast */
    net_if->linkoutput = (netif_linkoutput_fn)__aicwf_net_if_output;

    /* [BUGFIX] (common: single MAC source) The single MAC source is
     * fhost_vif->mac_addr (written by set_mac on the startup path: STA=base,
     * AP/P2P=base+vif_idx); here it is only mirrored into netif->hwaddr and not
     * recomputed. Fallback: if set_mac was not called first, fall back to base
     * to keep hwaddr non-zero */
    vmac = aic_fhost_vif_get_mac(fvif);
    if ((vmac[0] | vmac[1] | vmac[2] | vmac[3] | vmac[4] | vmac[5]) == 0)
    {
        aic_fhost_vif_set_mac(fvif, aic_get_mac_address());
        vmac = aic_fhost_vif_get_mac(fvif);
    }
    memcpy(net_if->hwaddr, vmac, ETHARP_HWADDR_LEN);


    /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
    net_if->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_IGMP;
    //net_if->flags = NETIF_FLAG_IGMP | NETIF_FLAG_ETHERNET | NETIF_FLAG_ETHARP;
    /* [BUGFIX] (common: unique interface name) Do not overwrite net_if->num:
     * lwIP netif_add assigns a globally unique num (name wl<num>), and unique
     * interface names are the premise for fhost_vif_idx_from_name() to look up a
     * vif by name; overwriting it with vif_idx once made two netifs share the
     * name "wl2" in STA/AP concurrency, binding the AP's WPA iface to the STA
     * vif0 (concurrent add_iface failed) */

    return 0;
}

/**
 * @brief Refresh the netif hardware address mirror (single MAC source)
 *
 * Called to refresh the mirror when the netif already exists after set_mac
 * (the first add is synced by init); ensures the lwip ARP sender MAC / Ethernet
 * frame source MAC match the firmware vif TX/RX MAC.
 */
void aicwf_net_if_update_mac(net_if_t net_if)
{
    struct netif *nif = netif_from_handle(net_if);
    if (!nif || !nif->state)
        return;
    memcpy(nif->hwaddr, aic_fhost_vif_get_mac(nif->state), ETHARP_HWADDR_LEN);
}

/**
 * @brief Reclaim all pool netifs (called on driver deinit)
 *
 * [BUGFIX] (common: netif lifecycle) Fixes the slot leak caused by netifs only
 * ever being added: the old implementation had no netif_remove after driver
 * re-init (fhost_init clears net_if_handle to NULL), so old netifs permanently
 * occupied netif_list while pool slots with output!=NULL could not be reused,
 * names drifted (wl0→wl1→...) and the pool was exhausted after 4 re-inits.
 * Called from fhost_deinit: takes each slot down / out of lwIP and clears it
 * back to the pool (output=NULL).
 */
void aicwf_net_if_release_all(void)
{
    int i;

    for (i = 0; i < PLAT_NETIF_POOL_SIZE; i++)
    {
        struct netif *nif = &s_netif_pool[i];
        if (nif->output == NULL)
            continue;
        DBG_NETAL_INF("%s: remove netif %c%c%d\n", __func__,
                      nif->name[0], nif->name[1], nif->num);
        netif_set_down(nif);
        netif_remove(nif);
        /* Return the slot to the pool: netif_remove does not clear output, zero it out to make it allocatable */
        memset(nif, 0, sizeof(struct netif));
    }
}

/**
 * @brief Initialize the network subsystem
 *
 * Creates the L2 semaphore/mutex, the TX buffer pool and the RX semaphore.
 * Can be called multiple times; returns immediately when already initialized.
 *
 * @return 0=success
 */
int aicwf_net_init(void)
{
    uint8_t i;
    struct aic_netif_dev *netifdev = &netif_dev;

    if (netifdev->net_inited)
    {
        return 0;
    }

    if (aicwf_rtos_semaphore_create(&netifdev->l2_semaphore, "l2_semaphore", 1, 0))
    {
        AIC_ASSERT_ERR(0);
    }

    if (aicwf_rtos_mutex_create(&netifdev->l2_mutex, "l2_mutex"))
    {
        AIC_ASSERT_ERR(0);
    }

#ifdef CONFIG_RX_WAIT_LWIP
    if (aicwf_rtos_semaphore_create(&netifdev->fhost_rx_lwip_sema, "fhost_rx_lwip_sema", NET_RX_LWIP_CNT, 0))
    {
        AIC_ASSERT_ERR(0);
    }
#endif /* CONFIG_RX_WAIT_LWIP */

    // Initial free tx buf
    co_list_init(&netifdev->net_tx_buf_free_list);
    if (aicwf_rtos_semaphore_create(&netifdev->net_tx_buf_sema, "net_tx_buf_sema", NET_TXBUF_CNT, 0))
    {
        AIC_ASSERT_ERR(0);
    }

    netifdev->net_tx_buf_cnt = 0;
    for (i = 0; i < NET_TXBUF_CNT; i++)
    {
        struct net_tx_buf_tag *net_tx_buffer = aicwf_rtos_malloc(sizeof(struct net_tx_buf_tag));
        if (net_tx_buffer)
        {
            //AIC_LOG_PRINTF("%s net_buf %d %x\n", __func__, i, (net_tx_buffer));
            co_list_push_back(&netifdev->net_tx_buf_free_list, (struct co_list_hdr *)(net_tx_buffer));
            netifdev->net_tx_buf_cnt++;
            aicwf_rtos_semaphore_signal(netifdev->net_tx_buf_sema, 0);
        }
    }

    DBG_NETAL_INF("net_tx_buf_sema initial count:%d\n", aicwf_rtos_semaphore_get_count(netifdev->net_tx_buf_sema));
    if (aicwf_rtos_mutex_create(&netifdev->net_tx_buf_mutex, "net_tx_buf_mutex"))
    {
        AIC_ASSERT_ERR(0);
    }

    netifdev->net_inited = true;

    return 0;
}

/**
 * @brief Deinitialize the network subsystem
 *
 * Frees the TX buffer pool, deletes all semaphores and mutexes and brings the
 * netif down.
 */
void aicwf_net_deinit(void)
{
    //uint8_t i;
    //struct net_tx_buf_tag *tx_buf;
    struct aic_netif_dev *netifdev = &netif_dev;

#ifndef CONFIG_DRIVER_ORM
    for (i = 0; i < NET_TXBUF_CNT; i++)
    {
        aicwf_rtos_mutex_lock(netifdev->net_tx_buf_mutex, -1);
        tx_buf = (struct net_tx_buf_tag *)co_list_pop_front(&netifdev->net_tx_buf_free_list);
        netifdev->net_tx_buf_cnt--;
        aicwf_rtos_mutex_unlock(netifdev->net_tx_buf_mutex);
        aicwf_rtos_free(tx_buf);
    }
#endif

    aicwf_rtos_semaphore_delete(netifdev->net_tx_buf_sema);
    aicwf_rtos_mutex_delete(netifdev->net_tx_buf_mutex);
    aicwf_rtos_semaphore_delete(netifdev->l2_semaphore);
    aicwf_rtos_mutex_delete(netifdev->l2_mutex);
#ifdef CONFIG_RX_WAIT_LWIP
    aicwf_rtos_semaphore_delete(netifdev->fhost_rx_lwip_sema);
#endif /* CONFIG_RX_WAIT_LWIP */

    RTOS_RES_NULL(netifdev->net_tx_buf_sema);
    RTOS_RES_NULL(netifdev->net_tx_buf_mutex);
    RTOS_RES_NULL(netifdev->l2_semaphore);
    RTOS_RES_NULL(netifdev->l2_mutex);
#ifdef CONFIG_RX_WAIT_LWIP
    RTOS_RES_NULL(netifdev->fhost_rx_lwip_sema);
#endif /* CONFIG_RX_WAIT_LWIP */

    memset(netifdev->l2_filter, 0, sizeof(netifdev->l2_filter));
    aicwf_net_if_down(NULL);
    netifdev->net_inited = false;
}

/**
 * @brief Network subsystem pre-deinit
 *
 * Waits for outstanding TX to finish and brings the netif down, preparing for
 * the real deinit.
 */
void aicwf_net_predeinit(void)
{
    netif_dev.net_inited = false;
    DBG_NETAL_INF("[netif] down and preinit enter\n");
    while (net_out_ing)
    {
        aicwf_rtos_msleep(1);
    }

    //down wifi netif
    aicwf_net_if_down(NULL);
    DBG_NETAL_INF("[netif] down and preinit exit\n");
}

/**
 * @brief Allocate a TX network buffer
 *
 * Allocates memory and copies the payload data, supporting SDIO ADMA alignment.
 *
 * @param payload data pointer
 * @param length  data length
 * @return net_buf_tx_t pointer, NULL on failure
 */
net_buf_tx_t *aicwf_net_buf_tx_alloc(const uint8_t *payload, uint32_t length)
{
    unsigned int reserved_len = aic_offsetof();
    uint8_t ext_len = 0, offset = 0;
#ifdef CONFIG_SDIO_ADMA
    ext_len = 8 + 64;
    offset = 4 + 32;
#endif
    void *obuf = aicwf_rtos_malloc(CO_ALIGN4_HI(sizeof(net_buf_tx_t)) + CO_ALIGN4_HI(length + reserved_len) + ext_len);
    if (!obuf)
    {
        DBG_NETAL_ERR("%s tx buffer null\n", __func__);
        return NULL;
    }
#ifdef  CONFIG_SDIO_ADMA
    void *nbuf = (void *)CO_ALIGN4_HI((uint32_t)obuf);
#else
    void *nbuf = obuf;
#endif
    net_buf_tx_t *buf = (net_buf_tx_t *)nbuf;
    memset(buf, 0, (sizeof(net_buf_tx_t)));
    uint8_t *payload_buf = (uint8_t *)buf + CO_ALIGN4_HI(sizeof(net_buf_tx_t)) + offset;
    memcpy((payload_buf + reserved_len), payload, length);
    buf->data_ptr = (payload_buf + reserved_len);
    buf->data_len = length;
    buf->pkt_type = TCPIP_PKTTYPE_MAX;
#ifdef  CONFIG_SDIO_ADMA
    buf->data_oprt = obuf;
#endif
    //AIC_LOG_PRINTF("%s tcpip %p _buf %p %p\n", __func__, buf, payload_buf, buf->data_ptr);

    return buf;
}

/**
 * @brief Free a TX network buffer
 *
 * Frees according to the packet type: TCPIP packets are freed directly, ETHER
 * packets are returned to the free list.
 *
 * @param buf TX buffer pointer
 */
void aicwf_net_buf_tx_free(net_buf_tx_t *buf)
{
    struct aic_netif_dev *netifdev = &netif_dev;

    if (!buf)
    {
        DBG_NETAL_ERR("Err tx buf is null\n");
        return ;
    }

    if (TCPIP_PKTTYPE_MAX == buf->pkt_type)
    {
        //AIC_LOG_PRINTF("nbtf:%p/%p\n", buf, buf->data_ptr);
#ifdef CONFIG_SDIO_ADMA
        aicwf_rtos_free(buf->data_oprt);
#else
        aicwf_rtos_free(buf);
#endif
        buf = NULL;
    }
    else if (TCPIP_PKTTYPE_ETHER == buf->pkt_type)
    {
        //buf->data_ptr -= (SDIO_HOSTDESC_SIZE);
        //buf->data_ptr -= sizeof(struct co_list_hdr);
        uint8_t *list_hdr = (uint8_t *)buf;
        list_hdr -= sizeof(struct co_list_hdr);
        //AIC_LOG_PRINTF("%s buf %p, net_buf %p\n", __func__, list_hdr, buf->data_ptr);
#if NET_IF_UAP_NCPYBUF
        /* [PLATFORM] nocopy TX free: pmsg==buf → free the linearized heap buffer
         * (rtos_free); pmsg!=buf → pmsg is a referenced pbuf (pbuf_free) */
        {
            struct net_tx_buf_tag *tag = (struct net_tx_buf_tag *)list_hdr;
            if (tag->pmsg != NULL)
            {
                if (tag->pmsg == tag->buf)
                {
                    aicwf_rtos_free(tag->pmsg);
                }
                else
                {
                    aicwf_platform_net_buf_tx_free(tag->pmsg);
                }
                tag->pmsg = NULL;
            }
        }
#endif
        aicwf_rtos_mutex_lock(netifdev->net_tx_buf_mutex, -1);
        co_list_push_back(&netifdev->net_tx_buf_free_list, (struct co_list_hdr *)(list_hdr));
        netifdev->net_tx_buf_cnt++;
        aicwf_rtos_mutex_unlock(netifdev->net_tx_buf_mutex);
        aicwf_rtos_semaphore_signal(netifdev->net_tx_buf_sema, 0);

    }
    else
    {
        DBG_NETAL_INF("%s free what? net_buf %p %d\n", __func__, buf->data_ptr, buf->pkt_type);
    }
}

/**
 * @brief Get the number of free TX buffers
 *
 * @return number of free buffers
 */
static uint32_t __aicwf_net_buf_tx_free_list_cnt(void)
{
    struct aic_netif_dev *netifdev = &netif_dev;
#if 1
    aicwf_rtos_mutex_lock(netifdev->net_tx_buf_mutex, -1);
    uint32_t cnt = co_list_cnt(&netifdev->net_tx_buf_free_list);
    aicwf_rtos_mutex_unlock(netifdev->net_tx_buf_mutex);
    return cnt;
#else
    return netifdev->net_tx_buf_cnt;
#endif
}

/**
 * @brief Check whether free TX buffers are running low
 *
 * Returns true when the free count is <= the threshold.
 *
 * @return true=low, false=enough
 */
bool aicwf_net_buf_tx_free_list_is_full(void)
{
    uint32_t cnt = __aicwf_net_buf_tx_free_list_cnt();

    return (cnt <= NET_TXBUF_TH);
}

/**
 * @brief Allocate an RX data buffer
 *
 * Allocates a pbuf and records it in the resource tracking list.
 *
 * @param length requested buffer size
 * @return pbuf pointer, NULL on failure
 */
uint8_t *aicwf_net_buf_rx_alloc(uint32_t length)
{
    void *pbuf = aicwf_platform_net_buf_rx_alloc(length);
    void *pmsg;

    if (pbuf == NULL)
    {
        DBG_NETAL_ERR("%s alloc tx_buf failed\n", __func__);

        return NULL;
    }

    pmsg = pbuf;//aicwf_platform_net_buf_rx_desc(pbuf);

    return pbuf;
}

/**
 * @brief Free an RX data buffer
 *
 * Frees the pbuf and removes it from the resource tracking list.
 *
 * @param msg pbuf message pointer
 */
void aicwf_net_buf_rx_free(void *msg)
{
    aicwf_platform_net_buf_rx_free(msg);
}


/* Internal copy core: platform copies data into allocated pbuf */
/**
 * @brief Process received Ethernet data (copy version)
 *
 * Allocates a new pbuf, copies the data and passes it to LwIP.
 *
 * @param pdata received data pointer
 * @param len   data length
 * @param netif netif handle
 * @return 0=success, negative=failure
 */
static int aicwf_rx_eth_data_process_copy(unsigned char *pdata,
        unsigned short len, net_if_t netif)
{
    int ret;
    void *msg;
    unsigned char *pmem = NULL;

    /* Unified per-vif gating for concurrent/single mode: pass as soon as the vif
     * bound to this netif is ready, independent of the global mode (in APSTA one
     * side disconnecting does not affect the other) */
    if (!aic_net_if_vif_ready(netif))
    {
        DBG_NETAL_INF("rx:vif not ready\n");
        goto exit;
    }

    if (len == 0)
    {
        DBG_NETAL_INF("lwip_wifi_in_buf len 0\n");
        goto exit;
    }

#if NETIF_PKT_DHCP
    aicwf_dhcp_trace(pdata, len, "netif-in");

    uint8_t *rdata = (uint8_t *)pdata;
    if (rdata[35] == 0x44 && rdata[37] == 0x43)
    {
        DBG_NETAL_INF("DHCP:");
        aicwf_dump_buffer("dhcp_payload", (uint8_t *)rdata, len);
    }
#endif

#if NETIF_PKT_PING
#if !NETIF_PKT_DHCP
    uint8_t *rdata = (uint8_t *)pdata;  /* reuse when the DHCP block is declared, avoiding a duplicate declaration in the same scope */
#endif
    if (rdata[12] == 0x08 && rdata[13] == 0x00 && rdata[34] == 0x08 && rdata[35] == 0x00)
    {
        DBG_NETAL_INF("NET-IN  ICMP: %d[%d]\n", (rdata[40] << 8) | rdata[41], aicwf_rtos_now(0));
        //aicwf_dump_buffer("rx_payload", (uint8_t *)rdata, len);
    }
#endif

    {
        //no rorder net buf don't push res list
        //pmsg 16 ||wifi extend 108||mac eth head || IP data ||....
        msg = aicwf_platform_net_buf_rx_alloc(len + AICWF_WIFI_HDR);
        if (msg == NULL)
        {
            DBG_NETAL_ERR("lwip_wifi_in_buf_alloc fail\n");

            return -2;
        }

        pmem = (uint8_t *)(((struct pbuf *)msg)->payload);

        if (pmem == NULL)
        {
            DBG_NETAL_INF("lwip_wifi_in_buf pmsg is null\n");
            return -3;
        }

        memcpy(pmem + AICWF_WIFI_HDR, pdata, len);
        ret = __aicwf_aicwifi_pkt_send_to_platform(pmem + AICWF_WIFI_HDR, len, msg, netif);

        //ret !0 free buf wifi module  else lwip
        if (ret != 0)
        {
            DBG_NETAL_ERR("lwip_wifi input fail2 ret:%d len:%d\n!!!!", ret, len);
            aicwf_platform_net_buf_rx_free(msg);
        }

    }
    return ret;

exit:
    return -1;
}

/**
 * @brief Common RX data entry
 *
 * 5-parameter entry. Under CONFIG_RX_NOCOPY: the reorder pass-through path
 * (is_reord) and the zero-copy path (lwip_msg is a custom pbuf whose payload
 * points at pdata) inject directly; otherwise alloc+copy is used.
 *
 * @param pdata    data pointer
 * @param len      data length
 * @param netif    netif handle
 * @param lwip_msg pbuf; non-NULL for the zero-copy/reorder pass-through path
 * @param is_reord reorder frame flag
 * @return processing result
 */
int aicwf_rx_eth_data_process(uint8_t *pdata, uint16_t len, net_if_t netif,
                              void *lwip_msg, uint8_t is_reord)
{
#ifdef AIC8800_USE_RT_WLAN
    /* M2: RX is bridged to the rt_wlan framework (the framework owns the netif,
     * the driver only delivers raw frames). STA and AP are two independent
     * rt_wlan devices; pick the target by the vif owning the netif. */
    struct rt_wlan_device *wlan = aic8800_wlan_get_device();
    int ap_vif = aic_wifi_get_ap_vif_idx();

    (void)lwip_msg;
    (void)is_reord;
    if (ap_vif >= 0 && netif != NULL &&
            netif == net_if_find_from_wifi_idx((unsigned int)ap_vif))
    {
        wlan = aic8800_wlan_get_ap_device();
    }
    rt_wlan_dev_report_data(wlan, pdata, (int)len);
    return 0;
#else
#ifdef CONFIG_RX_NOCOPY
    /* reorder pass-through / zero-copy: lwip_msg is already built (payload points
     * at pdata); if injection fails, pbuf_free triggers custom_free_fn to return
     * the header and release the node reference */
    if (is_reord || lwip_msg)
    {
        int ret = __aicwf_aicwifi_pkt_send_to_platform(pdata, len, lwip_msg, netif);
        if (ret != 0)
        {
            DBG_NETAL_ERR("lwip_wifi zcopy input fail ret:%d len:%d\n", ret, len);
            aicwf_platform_net_buf_rx_free(lwip_msg);
        }
        return ret;
    }
#endif /* CONFIG_RX_NOCOPY */
    return aicwf_rx_eth_data_process_copy((unsigned char *)pdata, len, netif);
#endif /* AIC8800_USE_RT_WLAN */
}

#ifdef CONFIG_FILTER_TCP_ACK
#define MAX_TCP_ACK             80
/**
 * @brief Send Ethernet data (with TCP ACK filtering)
 *
 * Allocates a buffer from the TX buffer pool, copies the data and sends it
 * through the WiFi driver. When CONFIG_FILTER_TCP_ACK is enabled, small TCP ACK
 * packets are filtered.
 *
 * @param data data pointer
 * @param len  data length
 * @param msg  associated pbuf message
 * @return 0=success
 */
int aicwf_tx_eth_data_no_tcpack_process(UINT8 *data, UINT32 len, void *msg)
{
    net_buf_tx_t *sent_pkt;
    void *fhost_vif;
    net_if_t net_if = aic_get_vif_net_if();
    struct aic_netif_dev *netifdev = &netif_dev;

    //dwmp data for test
    //rwnx_data_dump("__aicwf_net_if_output", data, len);

    /* Unified per-vif gating for concurrent/single mode (same as the RX path) */
    if (!aic_net_if_vif_ready(net_if))
    {
        DBG_NETAL_INF("tx:vif not ready\n");
        goto exit;
    }

    net_out_ing = 1;
    do
    {
        unsigned int reserved_len = aic_offsetof();//offsetof(struct hostdesc, cfm_cb);

#ifndef CONFIG_RX_WAIT_LWIP
#ifndef PROTECT_TX_ADMA
        int ret = aicwf_rtos_semaphore_wait(netifdev->net_tx_buf_sema, 100);
#else
        int ret = aicwf_rtos_semaphore_wait(netifdev->net_tx_buf_sema, 200);
#endif /* CONFIG_ADMA_PROTECT */
#else
        int ret = aicwf_rtos_semaphore_wait(netifdev->net_tx_buf_sema, 200);
#endif /* CONFIG_RX_WAIT_LWIP */

        struct net_tx_buf_tag *tx_buf = NULL;

        if (ret == 0)
        {
            aicwf_rtos_mutex_lock(netifdev->net_tx_buf_mutex, -1);
            tx_buf = (struct net_tx_buf_tag *)co_list_pop_front(&netifdev->net_tx_buf_free_list);
            netifdev->net_tx_buf_cnt--;
            aicwf_rtos_mutex_unlock(netifdev->net_tx_buf_mutex);
        }

        if (!tx_buf)
        {
            DBG_NETAL_INF("lwip_wifi_out_buf:tx full\n");
            /* nocopy: pbuf_ref has not been called yet, ownership stays with LwIP, must not free */
#if !NET_IF_UAP_NCPYBUF
            aicwf_platform_net_buf_tx_free(msg);
#endif
            goto end;
        }

#if NET_IF_UAP_NCPYBUF
        /* [PLATFORM] nocopy TX: reference the pbuf directly when headroom is
         * sufficient and the chain head has the Ethernet header; else fall back to
         * copy. pmsg/frag_next carry the pbuf chain to the SDIO aggregation scatter
         * copy. The decision uses data, the actual hostdesc write position (must
         * overlap the pbuf); ADMA/USB have no scatter hook (pkt_data must be
         * contiguous), so chained pbufs always linearize. Type check: PBUF_REF/ROM
         * payloads point outside, so [payload-14, payload) is not pbuf-owned
         * headroom and passing the address check would corrupt memory before it;
         * only RAM/POOL chain heads allow nocopy (one-sided address check, no upper
         * bound); chained also needs first-segment len >= NET_HEAD_OFFSET so that
         * [payload, payload+14) stays inside the first payload */
        struct pbuf *p_ref = (struct pbuf *)msg;
#if defined(PBUF_TYPE_FLAG_STRUCT_DATA_CONTIGUOUS)
        bool pbuf_own_buf = (p_ref->type_internal & PBUF_TYPE_FLAG_STRUCT_DATA_CONTIGUOUS) != 0;
#else
        bool pbuf_own_buf = true; /* lwip 1.x has no type flags, keep the old behavior */
#endif
#if defined(CONFIG_SDIO_ADMA) || defined(CONFIG_USB_SUPPORT)
        bool nocopy_ok = pbuf_own_buf
                         && (((uint8_t *)data - reserved_len)
                             >= ((uint8_t *)p_ref + sizeof(struct pbuf)))
                         && (p_ref->next == NULL);
#else
        bool nocopy_ok = pbuf_own_buf
                         && (((uint8_t *)data - reserved_len)
                             >= ((uint8_t *)p_ref + sizeof(struct pbuf)))
                         && ((p_ref->next == NULL) || (p_ref->len >= NET_HEAD_OFFSET));
#endif
        if (nocopy_ok)
        {
            pbuf_ref(p_ref);
            tx_buf->buf = data - reserved_len;
            tx_buf->pmsg = msg;
            /* chained frame length is self-consistent: always use the whole chain length, not the caller value */
            len = p_ref->tot_len;
        }
        else
        {
            /* Fallback linearize: in nocopy mode net_tx_buf_tag has no inline buf,
             * so malloc on demand; mark tag->buf=NULL + pmsg=heap pointer, and the
             * free path frees accordingly. The pbuf is owned by LwIP, no tx_free */
            uint32_t tot = p_ref->tot_len;
            tx_buf->buf = aicwf_rtos_malloc(reserved_len + tot);
            if (!tx_buf->buf)
            {
                DBG_NETAL_INF("lwip_wifi_out_buf:fb alloc fail\n");
                aicwf_rtos_mutex_lock(netifdev->net_tx_buf_mutex, -1);
                co_list_push_back(&netifdev->net_tx_buf_free_list, (struct co_list_hdr *)tx_buf);
                netifdev->net_tx_buf_cnt++;
                aicwf_rtos_mutex_unlock(netifdev->net_tx_buf_mutex);
                aicwf_rtos_semaphore_signal(netifdev->net_tx_buf_sema, 0);
                goto end;
            }
            tx_buf->pmsg = tx_buf->buf;
            pbuf_copy_partial(p_ref, tx_buf->buf + reserved_len, tot, 0);
            len = tot;
        }
#else
        memcpy((tx_buf->buf + reserved_len), data, len);
        aicwf_platform_net_buf_tx_free(msg);
#endif

        sent_pkt = &(tx_buf->pkt_hdr);
#if NET_IF_UAP_NCPYBUF
        sent_pkt->data_ptr = tx_buf->buf + reserved_len;
        sent_pkt->frag_next = nocopy_ok ? msg : NULL;
#else
        sent_pkt->data_ptr = tx_buf->buf + reserved_len;
        sent_pkt->frag_next = NULL;
#endif
        sent_pkt->data_len = len;
        sent_pkt->net_id   = 0;
        sent_pkt->pkt_type = TCPIP_PKTTYPE_ETHER;

#if NETIF_PKT_DHCP
        aicwf_dhcp_trace(data, len, "netif-out");
#endif

#if NETIF_PKT_PING
        uint8_t *rdata = (uint8_t *)sent_pkt->data_ptr;
        if (rdata[12] == 0x08 && rdata[13] == 0x00 && rdata[34] == 0x00 && rdata[35] == 0x00)
        {
            DBG_NETAL_INF("NET-OUT ICMP: %d", (rdata[40] << 8) | rdata[41]);
            aicwf_dump_buffer("tx_payload", rdata, len);
        }
#endif

        aic_tx_start(net_if, sent_pkt, NULL, NULL);

    }
    while (0);

end:
    net_out_ing = 0;
    return 0;

exit:
    /* nocopy: the drop path did not pbuf_ref, ownership stays with LwIP, must not free */
#if !NET_IF_UAP_NCPYBUF
    aicwf_platform_net_buf_tx_free(msg);
#endif
    return 0;
}
#endif

/**
 * @brief Send Ethernet data to the WiFi driver (default vif0 interface)
 *
 * Compatibility entry: callers without a netif context (e.g. tcp_ack
 * reinjection) use vif0.
 *
 * @param data data pointer
 * @param len  data length
 * @param msg  associated pbuf message
 * @return 0=success
 */
int aicwf_tx_eth_data_process(uint8_t *data, uint32_t len, void *msg)
{
    return aicwf_tx_eth_data_process_if(aic_get_vif_net_if(), data, len, msg);
}

/**
 * @brief Send Ethernet data to the WiFi driver (specified netif)
 *
 * Allocates a buffer from the TX buffer pool, copies the data and sends it via
 * aic_tx_start. net_if is passed through from the lwIP linkoutput callback
 * (per-vif), so in concurrent mode SoftAP/P2P (vif2) downlink data uses its own
 * interface instead of always vif0.
 * Supports STA/AP/P2P/APSTA/APP2P mode checks and TCP ACK filtering.
 *
 * @param net_if netif handle (NULL falls back to vif0)
 * @param data data pointer
 * @param len  data length
 * @param msg  associated pbuf message
 * @return 0=success
 */
int aicwf_tx_eth_data_process_if(net_if_t net_if, uint8_t *data, uint32_t len, void *msg)
{
    net_buf_tx_t *sent_pkt;
    //void *fhost_vif;
    struct aic_netif_dev *netifdev = &netif_dev;

    if (net_if == NULL)
        net_if = aic_get_vif_net_if();

    //dwmp data for test
    //rwnx_data_dump("__aicwf_net_if_output", data, len);

    /* Unified per-vif gating for concurrent/single mode: netif is passed through
     * from the lwIP linkoutput, so SoftAP/P2P (vif2) downlink is gated precisely
     * by its own vif state */
    if (!aic_net_if_vif_ready(net_if))
    {
        DBG_NETAL_INF("tx:vif not ready\n");
        goto exit;
    }

    aicwf_plat_gpio_trigger(DRV_TRG_NET_TX, 1);

#ifdef CONFIG_FILTER_TCP_ACK
#if TCP_ACK_LOG_ENABLE
    tcp_check_addr(data, len, "tx_uap");
#endif
    {
        if (len <= MAX_TCP_ACK)
        {
            if (aic_filter_send_tcp_ack(data, len, msg))
            {
#if TCP_ACK_LOG_ENABLE
                DBG_NETAL_INF("%s: filter\n", __func__);
#endif
                return 0;
            }
            else
            {
#if TCP_ACK_LOG_ENABLE
                DBG_NETAL_INF("%s: NOT\n", __func__);
#endif
            }
        }
    }
#endif

    net_out_ing = 1;
    do
    {
        unsigned int reserved_len = aic_offsetof();//offsetof(struct hostdesc, cfm_cb);

#ifndef CONFIG_RX_WAIT_LWIP
#ifndef PROTECT_TX_ADMA
        int ret = aicwf_rtos_semaphore_wait(netifdev->net_tx_buf_sema, 100);
#else
        int ret = aicwf_rtos_semaphore_wait(netifdev->net_tx_buf_sema, 200);
#endif /* CONFIG_ADMA_PROTECT */
#else
        int ret = aicwf_rtos_semaphore_wait(netifdev->net_tx_buf_sema, 200);
#endif /* CONFIG_RX_WAIT_LWIP */

        struct net_tx_buf_tag *tx_buf = NULL;

        if (ret == 0)
        {
            aicwf_rtos_mutex_lock(netifdev->net_tx_buf_mutex, -1);
            tx_buf = (struct net_tx_buf_tag *)co_list_pop_front(&netifdev->net_tx_buf_free_list);
            netifdev->net_tx_buf_cnt--;
            aicwf_rtos_mutex_unlock(netifdev->net_tx_buf_mutex);
        }

        if (!tx_buf)
        {
            DBG_NETAL_INF("lwip_wifi_out_buf:tx full\n");
            /* nocopy: pbuf_ref has not been called yet, ownership stays with LwIP, must not free */
#if !NET_IF_UAP_NCPYBUF
            aicwf_platform_net_buf_tx_free(msg);
#endif
            goto end;
        }

#if NET_IF_UAP_NCPYBUF
        /* [PLATFORM] nocopy TX: same as aicwf_tx_eth_data_no_tcpack_process,
         * single/chained segments both reference the pbuf with scatter copy at the
         * aggregation stage; fall back to linearize when headroom is insufficient.
         * The decision uses data, the actual hostdesc write position (must overlap
         * the pbuf); ADMA/USB have no scatter hook (pkt_data must be contiguous),
         * so chained pbufs always linearize. Type check: PBUF_REF/ROM payloads
         * point outside, so [payload-14, payload) is not pbuf-owned headroom and
         * passing the address check would corrupt memory before it; only RAM/POOL
         * chain heads allow nocopy (one-sided address check, no upper bound);
         * chained also needs first-segment len >= NET_HEAD_OFFSET so that
         * [payload, payload+14) stays inside the first payload */
        struct pbuf *p_ref = (struct pbuf *)msg;
        bool nocopy_ok = false;

        if (p_ref == NULL)
        {
            /* [PLATFORM] Route 2: the rt_wlan framework hands over raw Ethernet
             * frames via wlan_send; msg==NULL means there is no pbuf context
             * (wlan_lwip.c has already linearized chained pbufs), so data/len must
             * be copied directly; interpreting NULL as a pbuf would trigger the
             * "invalid buf" assert in pbuf_copy_partial. */
            tx_buf->buf = aicwf_rtos_malloc(reserved_len + len);
            if (!tx_buf->buf)
            {
                DBG_NETAL_INF("lwip_wifi_out_buf:fb alloc fail\n");
                aicwf_rtos_mutex_lock(netifdev->net_tx_buf_mutex, -1);
                co_list_push_back(&netifdev->net_tx_buf_free_list, (struct co_list_hdr *)tx_buf);
                netifdev->net_tx_buf_cnt++;
                aicwf_rtos_mutex_unlock(netifdev->net_tx_buf_mutex);
                aicwf_rtos_semaphore_signal(netifdev->net_tx_buf_sema, 0);
                goto end;
            }
            memcpy(tx_buf->buf + reserved_len, data, len);
            /* pmsg==buf marks a linearized heap buffer; the free path frees it directly */
            tx_buf->pmsg = tx_buf->buf;
        }
        else
        {
#if defined(PBUF_TYPE_FLAG_STRUCT_DATA_CONTIGUOUS)
            bool pbuf_own_buf = (p_ref->type_internal & PBUF_TYPE_FLAG_STRUCT_DATA_CONTIGUOUS) != 0;
#else
            bool pbuf_own_buf = true; /* lwip 1.x has no type flags, keep the old behavior */
#endif
            nocopy_ok = pbuf_own_buf
                        && (((uint8_t *)data - reserved_len)
                            >= ((uint8_t *)p_ref + sizeof(struct pbuf)))
                        && (p_ref->next == NULL);
            if (nocopy_ok)
            {
                pbuf_ref(p_ref);
                tx_buf->buf = data - reserved_len;
                tx_buf->pmsg = msg;
                /* chained frame length is self-consistent: always use the whole chain length, not the caller value */
                len = p_ref->tot_len;
            }
            else
            {
                uint32_t tot = p_ref->tot_len;
                tx_buf->buf = aicwf_rtos_malloc(reserved_len + tot);
                if (!tx_buf->buf)
                {
                    DBG_NETAL_INF("lwip_wifi_out_buf:fb alloc fail\n");
                    aicwf_rtos_mutex_lock(netifdev->net_tx_buf_mutex, -1);
                    co_list_push_back(&netifdev->net_tx_buf_free_list, (struct co_list_hdr *)tx_buf);
                    netifdev->net_tx_buf_cnt++;
                    aicwf_rtos_mutex_unlock(netifdev->net_tx_buf_mutex);
                    aicwf_rtos_semaphore_signal(netifdev->net_tx_buf_sema, 0);
                    goto end;
                }
                tx_buf->pmsg = tx_buf->buf;
                pbuf_copy_partial(p_ref, tx_buf->buf + reserved_len, tot, 0);
                len = tot;
            }
        }
#else
        struct pbuf *q;
        struct pbuf *p_buf = (struct pbuf *)msg;
        uint32_t offset = 0;
        uint8_t *buf = tx_buf->buf + reserved_len;

        for (q = p_buf; q != NULL; q = q->next)
        {
            memcpy((void *)(buf + offset), (void *)q->payload, q->len);
            offset += q->len;
            //AIC_LOG_PRINTF("NET-OUT: %d",  q->len);
        }
        len = offset;
        aicwf_platform_net_buf_tx_free(msg);
#endif

        sent_pkt = &(tx_buf->pkt_hdr);
        sent_pkt->data_ptr = tx_buf->buf + reserved_len;
#if NET_IF_UAP_NCPYBUF
        sent_pkt->frag_next = nocopy_ok ? msg : NULL;
#else
        sent_pkt->frag_next = NULL;
#endif
        sent_pkt->data_len = len;
        sent_pkt->net_id   = 0;
        sent_pkt->pkt_type = TCPIP_PKTTYPE_ETHER;

#if NETIF_PKT_DHCP
        aicwf_dhcp_trace(data, len, "netif-out");
#endif

#if NETIF_PKT_PING
        uint8_t *rdata = (uint8_t *)sent_pkt->data_ptr;
        if (rdata[12] == 0x08 && rdata[13] == 0x00 && rdata[34] == 0x00 && rdata[35] == 0x00)
        {
            DBG_NETAL_INF("NET-OUT ICMP: %d[%d]\n", (rdata[40] << 8) | rdata[41], aicwf_rtos_now(0));
            //aicwf_dump_buffer("tx_payload", rdata, len);
        }
#endif

        aic_tx_start(net_if, sent_pkt, NULL, NULL);

    }
    while (0);

end:
    net_out_ing = 0;
    aicwf_plat_gpio_trigger(DRV_TRG_NET_TX, 0);
    return 0;

exit:
    /* nocopy: the drop path did not pbuf_ref, ownership stays with LwIP, must not free */
#if !NET_IF_UAP_NCPYBUF
    aicwf_platform_net_buf_tx_free(msg);
#endif

    return 0;
}

//-------------------------------------------------------------------
// Driver other API define
//-------------------------------------------------------------------
static int net_dhcp_started = 0;
void aicwf_net_dhcp_stop(net_if_t net_if);
int aicwf_net_dhcp_address_obtained(net_if_t net_if);
/**
 * @brief Start the DHCP client
 *
 * @param net_if netif handle
 * @return 0=success, -1=failure
 */
int aicwf_net_dhcp_start(net_if_t net_if)
{
#ifdef AIC8800_USE_RT_WLAN
    /* Route 2: the DHCP client is started by the framework on the netif
     * connected event (wlan_lwip.c); calling dhcp_start on a pool slot that was
     * never registered with lwIP would hit output=NULL and crash */
    (void)net_if;
    DBG_NETAL_INF("%s: skipped (rt_wlan owns dhcp)\r\n", __func__);
    return 0;
#else
#if LWIP_IPV4 && LWIP_DHCP
    if (netifapi_dhcp_start(netif_from_handle(net_if)) ==  ERR_OK)
    {
        net_dhcp_started = 1;
        return 0;
    }
#endif //LWIP_IPV4 && LWIP_DHCP

    return -1;
#endif
}


/**
 * @brief Check whether DHCP has obtained an address
 *
 * @param act unused
 * @return 1=obtained, 0=not obtained
 */
uint8_t aicwf_net_if_dhcp_done(uint8_t act)
{
    /* [BUGFIX] (common: netif lookup) Look up the netif by the real STA vif (idx 0);
     * the pool[0] assumption that STA is always the first slot does not hold on
     * driver re-init/APSTA with AP started first, and would query the wrong interface */
    net_if_t net_if = net_if_find_from_wifi_idx(0);
    if (!net_if)
        return 0;
    int ret = aicwf_net_dhcp_address_obtained(net_if);
    if (ret == 0)
        return 1;
    else
        return 0;
}

//switch netif mode host or slave
/**
 * @brief Switch the netif DHCP mode
 *
 * mode=1 starts the DHCP client, otherwise stops it.
 *
 * @param mode 1=start DHCP, other=stop DHCP
 */
void aicwf_netif_mode_switch(uint8_t mode)
{
    /* [BUGFIX] (common: netif lookup) Same as aicwf_net_if_dhcp_done: look up the
     * real STA netif by vif idx 0 instead of assuming pool[0] */
    net_if_t net_if = net_if_find_from_wifi_idx(0);
    if (!net_if)
        return;
    if (mode == 1)
    {
        aicwf_net_dhcp_start(net_if);
        /* [BUGFIX] (common: default route) lwIP 2.1.2 netif_add() does not set the
         * default interface automatically; without netif_default, off-subnet packets
         * make ip4_route() return NULL (ERR_RTE), which shows up as same-subnet
         * traffic working while ping 8.8.8.8/public network fails to send. STA is
         * the default interface. */
        aicwf_net_if_set_default(net_if);
        DBG_NETAL_INF("__aicwf_net_dhcp_start\n");
    }
    else
    {
        aicwf_net_dhcp_stop(net_if);
        DBG_NETAL_INF("__aicwf_net_dhcp_stop\n");
    }
}

/**
 * @brief Stop the DHCP client
 *
 * @param net_if netif handle
 */
void aicwf_net_dhcp_stop(net_if_t net_if)
{
#ifdef AIC8800_USE_RT_WLAN
    /* Route 2: DHCP is managed by the framework (net_dhcp_started stays 0; this is explicit only) */
    (void)net_if;
    DBG_NETAL_INF("%s: skipped (rt_wlan owns dhcp)\r\n", __func__);
#else
    if (net_dhcp_started == 0)
        return;
#if LWIP_IPV4 && LWIP_DHCP
    netifapi_dhcp_stop(netif_from_handle(net_if));
    net_dhcp_started = 0;
#endif //LWIP_IPV4 && LWIP_DHCP
#endif
}

/**
 * @brief Get the DHCP start status
 *
 * @return 1=started, 0=not started
 */
int aicwf_net_dhcp_start_status(void)
{
    return net_dhcp_started;
}

/**
 * @brief Release the DHCP address
 *
 * @param net_if netif handle
 * @return 0=success, -1=failure
 */
int aicwf_net_dhcp_release(net_if_t net_if)
{
#ifdef AIC8800_USE_RT_WLAN
    (void)net_if;
    DBG_NETAL_INF("%s: skipped (rt_wlan owns dhcp)\r\n", __func__);
    return 0;
#else
#if LWIP_IPV4 && LWIP_DHCP
    if (netifapi_dhcp_release(netif_from_handle(net_if)) ==  ERR_OK)
        return 0;
#endif //LWIP_IPV4 && LWIP_DHCP
    return -1;
#endif
}

/**
 * @brief Check whether DHCP has assigned an address
 *
 * @param net_if netif handle
 * @return 0=assigned, -1=not assigned
 */
int aicwf_net_dhcp_address_obtained(net_if_t net_if)
{
#ifdef AIC8800_USE_RT_WLAN
    /* Route 2: the address lives on the framework netif and can never be found on
     * the pool slot; use the IPv4 address of the default interface (the first
     * registered STA netif, see netif_set_default in ethernetif.c), which also
     * holds for static IP setups */
    (void)net_if;
    if (netif_default != NULL &&
            !ip4_addr_isany_val(*netif_ip4_addr(netif_default)))
        return 0;
    return -1;
#else
#if LWIP_IPV4 && LWIP_DHCP
    if (dhcp_supplied_address(netif_from_handle(net_if)))
        return 0;
#endif //LWIP_IPV4 && LWIP_DHCP
    return -1;
#endif
}

/**
 * @brief Set the DNS server address
 *
 * @param dns_server IPv4 DNS server address
 * @return 0=success, -1=failure (LWIP_DNS not enabled)
 */
int aicwf_net_set_dns(uint32_t dns_server)
{
#if LWIP_DNS
    ip_addr_t ip;
    ip_addr_set_ip4_u32(&ip, dns_server);
    dns_setserver(0, &ip);
    return 0;
#else
    return -1;
#endif
}

/**
 * @brief Get the DNS server address
 *
 * @param dns_server output DNS server IPv4 address
 * @return 0=success, -1=failure
 */
int aicwf_net_get_dns(uint32_t *dns_server)
{
#if LWIP_DNS
    const ip_addr_t *ip;

    if (dns_server == NULL)
        return -1;

    ip = dns_getserver(0);
    *dns_server = ip_addr_get_ip4_u32(ip);

    return 0;
#else
    return -1;
#endif
}
/**
 * @brief Start the DHCP server
 *
 * @param net_if netif handle
 */
void aicwf_net_if_dhcp_server_start(net_if_t net_if)
{
    //uint8_t    addr_start[4] = {192, 168, 13, 20};
    //uint8_t    addr_end[4] = {192, 168, 13, 30};
    //dhcp_addr_start.addr  = lwip_ipv4_addr(addr_start);
    //dhcp_addr_end.addr        = lwip_ipv4_addr(addr_end);
    //dhcp_server_start(&gnetif[0], &dhcp_addr_start, &dhcp_addr_end);
#if LWIP_DHCPS
    struct netif *nif = netif_from_handle(net_if);
    char name[3];
    memset(name, 0, 3);
    memcpy(name, nif->name, sizeof(nif->name));
    dhcpd_start(name);
#endif
}

/**
 * @brief Stop the DHCP server
 *
 * @param net_if netif handle
 */
void aicwf_net_if_dhcp_server_stop(net_if_t net_if)
{
#if LWIP_DHCPS
    struct netif *nif = netif_from_handle(net_if);
    char name[3];
    memset(name, 0, 3);
    memcpy(name, nif->name, sizeof(nif->name));
    dhcpd_stop(nif->name);
#endif
}



/**
 * @brief TCP/IP stack init-done callback
 *
 * @param arg unused
 */
static void aicwf_tcpip_init_done(void *arg)
{
    (void)arg;
    DBG_NETAL_INF("netif_plat_init done\r\n");
}


/**
 * @brief Initialize the platform network layer
 *
 * Calls LwIP tcpip_init to start the TCP/IP stack.
 */
void aicwf_net_if_plat_init(void)
{
    tcpip_init(aicwf_tcpip_init_done, NULL);
}

/**
 * @brief Get the used IP address
 * @return IPv4 address
 */
uint32_t aicwf_net_if_get_tousedip(void)
{
    return netif_dev.used_ip;
}

/**
 * @brief Set the used IP address
 * @param ip IPv4 address
 */
void aicwf_net_if_set_tousedip(uint32_t ip)
{
    netif_dev.used_ip = ip;
}

/**
 * @brief Get the netmask
 * @return netmask
 */
uint32_t aicwf_net_if_get_netmask(void)
{
    return netif_dev.netmask;
}
/**
 * @brief Set the netmask
 * @param netmask netmask
 */
void aicwf_net_if_set_netmask(uint32_t netmask)
{
    netif_dev.netmask = netmask;
}

/**
 * @brief Get the netif handle for the given index
 *
 * @param index netif pool index
 * @return netif handle (struct netif*), slot 0 when the index is out of range
 */
net_if_t aicwf_net_if_get(uint8_t index)
{
    if (index < PLAT_NETIF_POOL_SIZE)
        return (net_if_t)&s_netif_pool[index];
    return (net_if_t)&s_netif_pool[0];
}

/**
 * @brief Get the WiFi index of the netif
 *
 * @param net_if netif handle
 * @return WiFi interface index
 */
int aicwf_net_if_get_wifi_idx(net_if_t net_if)
{
    return netif_get_index(netif_from_handle(net_if));
}

//-------------------------------------------------------------------
// System function wrappers (referenced by the host closed-source lib / wpa_supplicant)
//-------------------------------------------------------------------
/**
 * @brief htons() wrapper — referenced by wpa_supplicant prebuilt code
 *
 * Byte-swap implementation (Cortex-M33 little-endian: host=LE, network=BE).
 * [AIC8800-PORT][SiFli] This file includes lwip/def.h, where `#define htons lwip_htons`
 * macro-expands the function name htons below into lwip_htons, leaving the bare
 * symbol htons referenced by the host prebuilt lib still undefined (link error:
 * undefined reference to htons). Hence #undef first to restore the real name.
 * weak: if the platform provides htons elsewhere, let that one win (SKILL porting
 * guide: htons multiple definition → weak).
 */
#undef htons
uint16_t __attribute__((weak)) htons(uint16_t hostshort)
{
    return (uint16_t)(((hostshort & 0xff00) >> 8) | ((hostshort & 0x00ff) << 8));
}

/**
 * @brief inet_aton() wrapper — referenced by wpa_supplicant prebuilt code
 *
 * Forwards to the host layer aic_inet_aton() implementation.
 */
int inet_aton(const char *cp, void *addr)
{
    return aic_inet_aton(cp, addr);
}


//-------------------------------------------------------------------
// DHCP Server stubs (callers are guarded by the LWIP_DHCPS macro)
//-------------------------------------------------------------------
#if LWIP_DHCPS
/**
 * @brief DHCP Server start stub
 *
 * Placeholder implementation for when the SDK does not provide dhcpd_start.
 * Replace with the real DHCP Server start logic when porting.
 */
void dhcpd_start(const char *name)
{
    (void)name;
    DBG_NETAL_INF("dhcpd_start: stub — %s\n", name);
}

/**
 * @brief DHCP Server stop stub
 *
 * Placeholder implementation for when the SDK does not provide dhcpd_stop.
 */
void dhcpd_stop(const char *name)
{
    (void)name;
    DBG_NETAL_INF("dhcpd_stop: stub — %s\n", name);
}
#endif
