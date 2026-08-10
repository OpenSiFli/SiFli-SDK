/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "mac_frame.h"
#include "fhost_tx.h"
#include "fhost_cntrl.h"
#include "netif_port.h"
#include "aic_plat_log.h"
#include "netal_sock.h"

#define ERR_OK                  (0)

net_if_t fvif_netifs[NX_VIRT_DEV_MAX];

//-------------------------------------------------------------------
// Driver netif base API define
//-------------------------------------------------------------------
net_if_t *net_if_find_from_name(const char *name)
{
    //aic_dbg("net_if_find_from_name %s \n", name);
    //return netif_find_by_name((char *)name);

    return fhost_env.vif[0].net_if;
}

const uint8_t *net_if_get_mac_addr(net_if_t *net_if)
{
    return (uint8_t *)net_if->hwaddr;
}

struct fhost_vif_tag *net_if_vif_info(net_if_t *net_if)
{
    return (struct fhost_vif_tag *)net_if->state;
}

net_if_t *net_if_find_from_wifi_idx(unsigned int idx)
{
    if ((idx >= NX_VIRT_DEV_MAX) || (fhost_env.vif[idx].mac_vif == NULL))
        return NULL;


    if (fhost_env.vif[idx].mac_vif->type != VIF_UNKNOWN)
    {
        return fhost_env.vif[idx].net_if;
    }

    return NULL;
}

net_if_t *net_if_from_fvif_idx(unsigned int idx)
{
    net_if_t *net_if = NULL;
    if (idx < NX_VIRT_DEV_MAX) {
        net_if = &fvif_netifs[idx];
    }
    return net_if;
}

//-------------------------------------------------------------------
// Driver netif l2 API define
//-------------------------------------------------------------------
static void net_l2_send_cfm(uint32_t frame_id, bool acknowledged, void *arg)
{
    if (arg)
        *((bool *)arg) = acknowledged;
    rtos_semaphore_signal(netif_dev.l2_semaphore, false);
}

int net_l2_send(net_if_t *net_if, const uint8_t *data, int data_len, uint16_t ethertype,
                const uint8_t *dst_addr, bool *ack)
{
    int res;
    net_buf_tx_t *net_buf;
    struct aic_netif_dev *netifdev = &netif_dev;

	if(!netifdev->net_inited)
	{
		aic_dbg("%s:net is not init\n",__func__);
		return -2;
	}

    if (net_if == NULL || data == NULL /* || data_len >= net_if->mtu || !netif_is_up(net_if) */)
        return -1;

    net_buf = net_buf_tx_alloc((data - sizeof(struct mac_eth_hdr)), (data_len + sizeof(struct mac_eth_hdr)));
    if (net_buf == NULL)
        return 0;

    if (dst_addr)
    {
        // Need to add ethernet header as fhost_tx_start is called directly
        struct mac_eth_hdr* ethhdr;
        ethhdr = (struct mac_eth_hdr*)net_buf->data_ptr;
        ethhdr->type = htons(ethertype);
        memcpy(&ethhdr->da, dst_addr, 6);
        memcpy(&ethhdr->sa, net_if->hwaddr, 6);
    }

    // Ensure no other thread will program a L2 transmission while this one is waiting
    // for its confirmation
    rtos_mutex_lock(netifdev->l2_mutex, -1);

    //AIC_LOG_PRINTF("%s pkt %x %x, t:%x\n", __func__, net_buf, net_buf->data_ptr, net_buf->pkt_type);
    // In order to implement this function as blocking until the completion of the frame
    // transmission, directly call fhost_tx_start with a confirmation callback.
    res = fhost_tx_start(net_if, net_buf, net_l2_send_cfm, ack);

    // Wait for the transmission completion
    rtos_semaphore_wait(netifdev->l2_semaphore, AIC_RTOS_WAIT_FOREVEVR);

    // Now new L2 transmissions are possible
    rtos_mutex_unlock(netifdev->l2_mutex);

    return res;
}

int net_l2_socket_create(net_if_t *net_if, uint16_t ethertype)
{
    int i;
    struct fhost_cntrl_link *link;
    struct l2_filter_tag *filter = NULL;
    struct aic_netif_dev *netifdev = &netif_dev;

    /* First find free filter and check that socket for this ethertype/net_if couple
       doesn't already exists */
    for (i = 0; i < NX_NB_L2_FILTER; i++)
    {
        if ((netifdev->l2_filter[i].net_if == net_if) &&
            (netifdev->l2_filter[i].ethertype == ethertype))
        {
            return -1;
        }
        else if ((filter == NULL) && (netifdev->l2_filter[i].net_if == NULL))
        {
            filter = &netifdev->l2_filter[i];
        }
    }

    if (!filter)
        return -1;
    // Open link with cntrl task to send cfgrwnx commands and retrieve events
    link = fhost_cntrl_cfgrwnx_link_open();
    if (link == NULL)
        return -1;

    filter->link = link;
    filter->sock = link->sock_recv;
    if (filter->sock == -1)
        return -1;

    filter->net_if = net_if;
    filter->ethertype = ethertype;

    return filter->sock;
}

int net_l2_socket_delete(int sock)
{
    int i;
    struct aic_netif_dev *netifdev = &netif_dev;

    for (i = 0; i < NX_NB_L2_FILTER; i++)
    {
        if ((netifdev->l2_filter[i].net_if != NULL) &&
            (netifdev->l2_filter[i].sock == sock))
        {
            netifdev->l2_filter[i].net_if = NULL;
            fhost_cntrl_cfgrwnx_link_close(netifdev->l2_filter[i].link);
            netifdev->l2_filter[i].sock = -1;
            return 0;
        }
    }

    return -1;
}

//-------------------------------------------------------------------
// Driver EAP pkt API define
//-------------------------------------------------------------------
err_t net_eth_receive(unsigned char *pdata, unsigned short len, net_if_t *netif)
{
    int i;
    struct l2_filter_tag *filter = NULL;
    struct mac_eth_hdr* ethhdr = (struct mac_eth_hdr*)pdata;
    uint16_t ethertype = ntohs(ethhdr->type);
    struct aic_netif_dev *netifdev = &netif_dev;

	if(!netifdev->net_inited)
	{
		aic_dbg("%s:net is not init\n",__func__);
		return -2;
	}

    for (i = 0; i < NX_NB_L2_FILTER; i++)
    {
        if ((netifdev->l2_filter[i].net_if == netif) &&
            (netifdev->l2_filter[i].ethertype == ethertype))
        {
            filter = &netifdev->l2_filter[i];
            break;
        }
    }

    if (!filter)
        return -1;

    if (netal_send(filter->link->sock_send, pdata, len, 0) < 0)
    {
        AIC_LOG_PRINTF("Err: %s len %d\n", __func__, len);
        return -1;
    }
    return ERR_OK;
}

//-------------------------------------------------------------------
// Driver other API define
//-------------------------------------------------------------------
char* aic_inet_ntoa(struct in_addr addr)
{
  	return inet_ntoa(addr);
}

