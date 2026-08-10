/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


//-------------------------------------------------------------------
// Driver Header Files
//-------------------------------------------------------------------
#include "co_math.h"
#include "dbg_assert.h"
#include "wifi_al.h"
#include "netif_al.h"
#include "netif_def.h"
#include "rtos_port.h"
#include "netif_port.h"
#include "aic_plat_log.h"

#include "wifi_port.h"

/* net_if buffer */
#define NET_IF_UAP_NCPYBUF      (0)
#define NET_HEAD_OFFSET         (14)
#define ERR_OK                  (0)


#if NET_IF_UAP_NCPYBUF
#define NET_TXBUF_CNT           (32*2)
#else
#define NET_TXBUF_CNT           (28)
#endif
#define NET_TXBUF_TH            (NET_TXBUF_CNT/2-2)
#define NET_RX_LWIP_CNT         (40)

#ifdef WIFI_USING_LOOPBACK_NETDEV
#define MAX_ADDR_LEN    6

struct loop_net_device
{
    /* inherit from ethernet device */
    struct eth_device parent;

    /* interface address info. */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN]; /* hw address   */
};

/* control the interface */
static rt_err_t loop_net_control(rt_device_t dev, int cmd, void *args)
{
    struct loop_net_device *aicwf_net = (struct loop_net_device *)dev;
    switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if (args)
            rt_memcpy(args, aicwf_net->dev_addr, MAX_ADDR_LEN);
        else
            return -RT_ERROR;
        break;

    default :
        break;
    }

    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops aicwf_net_ops =
{
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    loop_net_control
};
#endif

static struct loop_net_device loop_net_dev = {};
#endif

//-------------------------------------------------------------------
// Driver Variables define
//-------------------------------------------------------------------
static bool volatile net_out_ing = false;
struct aic_netif_dev netif_dev = {0};

//-------------------------------------------------------------------
// Driver Import functions
//-------------------------------------------------------------------

//-------------------------------------------------------------------
// Driver netif base API define
//-------------------------------------------------------------------
int net_if_get_name(net_if_t *net_if, char *buf, int len)
{
    if (len > 0)
        buf[0] = net_if->name[0];
    if (len > 1)
        buf[1] = net_if->name[1];
    if (len > 2)
        buf[2] = net_if->num + '0';
    if ( len > 3)
        buf[3] = '\0';

    return 3;
}

#if 0
void net_if_up(net_if_t *net_if)
{
    netifapi_netif_set_up(netif_dev.wifi_uap_netif);
}

void net_if_down(net_if_t *net_if)
{
    netifapi_netif_set_down(netif_dev.wifi_uap_netif);
	//AIC_LOG_PRINTF("netif status:%d",(netif_is_ready(netif_dev.wifi_uap_netif)));
}

void net_if_set_default(net_if_t *net_if)
{
    netifapi_netif_set_default(netif_dev.wifi_uap_netif);
}

void net_if_set_ip(net_if_t *net_if, uint32_t ip, uint32_t mask, uint32_t gw)
{
    if (!net_if)
        return;

    netif_set_addr(netif_dev.wifi_uap_netif, (ip_addr_t *)&ip, (ip_addr_t *)&mask,
                   (ip_addr_t *)&gw);

}

int net_if_get_ip(net_if_t *net_if, uint32_t *ip, uint32_t *mask, uint32_t *gw)
{
    struct aic_netif_dev *netifdev = &netif_dev;

    if (!net_if)
        return -1;

    #if 0
    if (ip)
        *ip = netif_get_ipaddr(netifdev->wifi_uap_netif)->addr;
    if (mask)
        *mask = netif_get_netmask(netifdev->wifi_uap_netif)->addr;
    if (gw)
        *gw = netif_get_gateway(netifdev->wifi_uap_netif)->addr;
    #endif

    return 0;
}
#endif

int net_if_add(net_if_t *net_if, const uint32_t *ipaddr, const uint32_t *netmask, const uint32_t *gw, void *state_vif)
{
    err_t status = ERR_OK;

	aic_dbg("net_if_add %s return %d\n", net_if->name, status);
	net_if->num  = netif_dev.netif_num++;
    net_if->state = state_vif;

    return (status == ERR_OK ? 0 : -1);
}

err_t net_if_init(net_if_t *net_if)
{
    err_t status = ERR_OK;
    struct macaddr *vif_mac = NULL;
    struct aic_netif_dev *netifdev = &netif_dev;

	net_if->hostname = "AicWlan";
	net_if->name[ 0 ] = 'w';
	net_if->name[ 1 ] = 'l';

    vif_mac = (struct macaddr *)aic_get_vif_mac_addr();
    memcpy(vif_mac, aic_get_mac_address(), 6);
    netifdev->netif_num  = 0;

    aic_dbg("net_if_init %s %s\n", net_if->name, net_if->hostname);
    memcpy(net_if->hwaddr, vif_mac, 6);

    return status;
}

int net_init(void)
{
    uint8_t i;
    struct aic_netif_dev *netifdev = &netif_dev;

    if (netifdev->net_inited)
	{
        return 0;
    }

    if (rtos_semaphore_create(&netifdev->l2_semaphore, "l2_semaphore", 1, 0))
    {
        AIC_ASSERT_ERR(0);
    }

	if (rtos_mutex_create(&netifdev->l2_mutex, "l2_mutex"))
	{
	    AIC_ASSERT_ERR(0);
	}

    #ifdef CONFIG_RX_WAIT_LWIP
    if (rtos_semaphore_create(&netifdev->fhost_rx_lwip_sema, "FhostRxLwipSema", NET_RX_LWIP_CNT, 0)) {
        AIC_ASSERT_ERR(0);
    }
    #endif /* CONFIG_RX_WAIT_LWIP */

    // Initial free tx buf
    co_list_init(&netifdev->net_tx_buf_free_list);
    if (rtos_semaphore_create(&netifdev->net_tx_buf_sema, "net_tx_buf_sema", NET_TXBUF_CNT, 0)) {
        AIC_ASSERT_ERR(0);
    }

    for(i = 0; i < NET_TXBUF_CNT; i++)
    {
        struct net_tx_buf_tag *net_tx_buffer = rtos_malloc(sizeof(struct net_tx_buf_tag));
        if(net_tx_buffer) {
            //AIC_LOG_PRINTF("%s net_buf %d %x\n", __func__, i, (net_tx_buffer));
            co_list_push_back(&netifdev->net_tx_buf_free_list, (struct co_list_hdr *)(net_tx_buffer));
            rtos_semaphore_signal(netifdev->net_tx_buf_sema, 0);
        }
    }

    AIC_LOG_PRINTF("net_tx_buf_sema initial count:%d\n", rtos_semaphore_get_count(netifdev->net_tx_buf_sema));
    if (rtos_mutex_create(&netifdev->net_tx_buf_mutex, "net_tx_buf_mutex"))
    {
        AIC_ASSERT_ERR(0);
    }

    netifdev->net_inited = true;

    return 0;
}

void net_deinit(void)
{
    int txbuf_cnt = 0;
    struct net_tx_buf_tag *tx_buf;
    struct aic_netif_dev *netifdev = &netif_dev;

    #ifndef CONFIG_DRIVER_ORM
    if (netifdev->net_tx_buf_mutex) {
        rtos_mutex_lock(netifdev->net_tx_buf_mutex, -1);
    }
    tx_buf = (struct net_tx_buf_tag *)co_list_pop_front(&netifdev->net_tx_buf_free_list);
    while (tx_buf) {
        rtos_free(tx_buf);
        txbuf_cnt++;
        tx_buf = (struct net_tx_buf_tag *)co_list_pop_front(&netifdev->net_tx_buf_free_list);
    }
    if (netifdev->net_tx_buf_mutex) {
        rtos_mutex_unlock(netifdev->net_tx_buf_mutex);
    }
    AIC_LOG_PRINTF("%s txbuf_cnt: %d, %d\n", __func__, txbuf_cnt, NET_TXBUF_CNT);
    #endif

    if (netifdev->net_tx_buf_sema) {
        rtos_semaphore_delete(netifdev->net_tx_buf_sema);
        netifdev->net_tx_buf_sema = NULL;
    }
    if (netifdev->net_tx_buf_mutex) {
        rtos_mutex_delete(netifdev->net_tx_buf_mutex);
        netifdev->net_tx_buf_mutex = NULL;
    }
    if (netifdev->l2_semaphore) {
        rtos_semaphore_delete(netifdev->l2_semaphore);
        netifdev->l2_semaphore = NULL;
    }
    if (netifdev->l2_mutex) {
        rtos_mutex_delete(netifdev->l2_mutex);
        netifdev->l2_mutex = NULL;
    }
	memset(netifdev->l2_filter, 0, sizeof(netifdev->l2_filter));
	//net_if_down(NULL);
}

void net_predeinit(void)
{
	netif_dev.net_inited = false;

	while(net_out_ing)
	{
		rtos_msleep(1);
	}
}

net_buf_tx_t *net_buf_tx_alloc(const uint8_t *payload, uint32_t length)
{
    unsigned int reserved_len = aic_offsetof();
	uint8_t ext_len = 0,offset = 0;
#ifdef CONFIG_SDIO_ADMA
	ext_len = 8 + 64;
	offset = 4+32;
#endif
    void *obuf = rtos_malloc(CO_ALIGN4_HI(sizeof(net_buf_tx_t)) + CO_ALIGN4_HI(length + reserved_len)+ext_len);
    if(!obuf) {
		AIC_LOG_ERROR("%s tx buffer null\n", __func__);
        return NULL;
    }
#ifdef	CONFIG_SDIO_ADMA
	void *nbuf = (void *)CO_ALIGN4_HI((uint32_t)obuf);
#else
	void *nbuf = obuf;
#endif
	net_buf_tx_t *buf = (net_buf_tx_t *)nbuf;
    memset(buf, 0, (sizeof(net_buf_tx_t)));
    uint8_t *payload_buf = (uint8_t *)buf + CO_ALIGN4_HI(sizeof(net_buf_tx_t))+offset;
    memcpy((payload_buf + reserved_len), payload, length);
    buf->data_ptr = (payload_buf + reserved_len);
    buf->data_len = length;
    buf->pkt_type = TCPIP_PKTTYPE_MAX;
#ifdef	CONFIG_SDIO_ADMA
	buf->data_oprt = obuf;
#endif
    //AIC_LOG_PRINTF("%s tcpip %p _buf %p %p\n", __func__, buf, payload_buf, buf->data_ptr);

    return buf;
}

void net_buf_tx_free(net_buf_tx_t *buf)
{
    struct aic_netif_dev *netifdev = &netif_dev;

    if (!buf) {
        AIC_LOG_PRINTF("Err tx buf is null\n");
        return ;
    }

    if (TCPIP_PKTTYPE_MAX == buf->pkt_type)
	{
        //AIC_LOG_PRINTF("nbtf:%p/%p\n", buf, buf->data_ptr);
        #ifdef CONFIG_SDIO_ADMA
        rtos_free(buf->data_oprt);
        #else
        rtos_free(buf);
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
#if	NET_IF_UAP_NCPYBUF
		{
			struct net_tx_buf_tag * tag = (struct net_tx_buf_tag *)list_hdr;
			if(((buf->data_ptr-AICWF_WIFI_HDR_LEN) != tag->pmsg) && ((buf->data_ptr-AICWF_WIFI_HDR_LEN - NET_HEAD_OFFSET) != tag->pmsg))
			{
				AIC_LOG_PRINTF("%s free what? why? net_buf %p %p\n", __func__, buf->data_ptr,tag->pmsg);
			}
			else
			{
                platform_net_buf_rx_free(tag->pmsg);
				rtos_net_mempop(tag->pmsg,RES_TYPE_MEM_NET_TX);
			}
		}
#endif
        rtos_mutex_lock(netifdev->net_tx_buf_mutex, -1);
        co_list_push_back(&netifdev->net_tx_buf_free_list, (struct co_list_hdr *)(list_hdr));
        rtos_mutex_unlock(netifdev->net_tx_buf_mutex);
        rtos_semaphore_signal(netifdev->net_tx_buf_sema, 0);

    }
	else
	{
		AIC_LOG_PRINTF("%s free what? net_buf %p %d\n", __func__, buf->data_ptr,buf->pkt_type);
	}
}

uint32_t net_buf_tx_free_list_cnt(void)
{
    struct aic_netif_dev *netifdev = &netif_dev;
    rtos_mutex_lock(netifdev->net_tx_buf_mutex, -1);
    uint32_t cnt = co_list_cnt(&netifdev->net_tx_buf_free_list);
    rtos_mutex_unlock(netifdev->net_tx_buf_mutex);
    return cnt;
}

bool net_buf_tx_free_list_is_full(void)
{
    uint32_t cnt = net_buf_tx_free_list_cnt();

    return (cnt <= NET_TXBUF_TH);
}

#ifdef CONFIG_RX_NOCOPY
uint8_t *net_buf_rx_alloc(uint32_t length)
{
    uint8_t*pbuf = platform_net_buf_rx_alloc(length);
	void *pmsg;

	if (pbuf == NULL) {
        aic_dbg("%s alloc tx_buf failed\n", __func__);

        return NULL;
    }

	pmsg = platform_net_buf_rx_desc(pbuf);

	rtos_net_mempush(pbuf,length,pmsg,RES_TYPE_MEM_NET_RX);

    return pbuf;
}

void net_buf_rx_free(void *msg)
{
	if(rtos_res_find_res(msg,RES_TYPE_MEM_NET_RX) == NULL)
		return;

    platform_net_buf_rx_free(msg);
	rtos_net_mempop(msg,RES_TYPE_MEM_NET_RX);
}
#endif

#ifdef CONFIG_RX_NOCOPY
int rx_eth_data_process(unsigned char *pdata, unsigned short len,
                     net_if_t *netif, void *lwip_msg, uint8_t is_reord)
{
    int ret;
    void *msg;
    unsigned char *pmem = NULL;
    struct aic_netif_dev *netifdev = &netif_dev;

    if(!netifdev->net_inited)
    {
        aic_dbg("rx:netif is not ready\n");
        //platform_net_buf_rx_free(lwip_msg);
        if((lwip_msg)&&(is_reord))
            net_buf_rx_free(lwip_msg);
        return -5;
    }

    if(len == 0)
    {
        aic_dbg("lwip_wifi_in_buf len 0\n");
        //platform_net_buf_rx_free(lwip_msg);
        if((lwip_msg)&&(is_reord))
            net_buf_rx_free(lwip_msg);
        return -1;
    }

    if (is_reord) {
		ret = aicwifi_pkt_send_to_platform(pdata,len,lwip_msg);

		if(((uint32_t)pdata - (uint32_t)lwip_msg) != 124)
			aic_dbg("lwip_wifi input %p %p %d \n!!!!",pdata,lwip_msg,len);

        //ret !0 free buf wifi module  else lwip
        if(ret != 0)
        {
            aic_dbg("lwip_wifi input fail1 ret:%d len:%d\n!!!!",ret,len);

            #ifndef CONFIG_RX_WAIT_LWIP
            net_buf_rx_free(lwip_msg);
            #else
            rtos_semaphore_wait(netifdev->fhost_rx_lwip_sema, 100);
            ret = aicwifi_pkt_send_to_platform(pdata,len,lwip_msg);
            if(ret != 0)
            {
                aic_dbg("fail1 ret:%d", ret);
                net_buf_rx_free(lwip_msg);
            }
            #endif /* CONFIG_RX_WAIT_LWIP */
        }
		else
		{
			rtos_net_mempop(lwip_msg,RES_TYPE_MEM_NET_RX);
		}
    }
    else
    {
    	//no rorder net buf don't push res list
    	//pmsg 16 ||wifi extend 108||mac eth head || IP data ||....
    	pmem = platform_net_buf_rx_alloc(len + AICWF_WIFI_HDR);
    	if(pmem == NULL)
    	{
    		aic_dbg("lwip_wifi_in_buf_alloc fail\n");

    		return -2;
    	}

    	msg = platform_net_buf_rx_desc(pmem);

    	if(msg == NULL)
    	{
    		aic_dbg("lwip_wifi_in_buf pmsg is null\n");
    		return -3;
    	}

        //aic_dbg("lwip_wifi_in_buf_pmsg:%p t:%d l:%d n:%d\n",
        //          pmem,((pmsg*)msg)->type,((pmsg*)msg)->layer,((pmsg*)msg)->rnum);

        #if 0
        uint8_t *rdata = (uint8_t *)pdata;
        if (rdata[12] == 0x08 && rdata[13] == 0x00 && rdata[34] == 0x08 && rdata[35] == 0x00) {
            AIC_LOG_PRINTF("NET-IN  ICMP: %d", (rdata[40]<<8) | rdata[41]);
        }
        //rwnx_data_dump("payload", (uint8_t *)buf->payload + 20, buf->info.vect.frmlen - 20);
        #endif

    	memcpy(pmem + AICWF_WIFI_HDR, pdata,len);
    	ret = aicwifi_pkt_send_to_platform(pmem + AICWF_WIFI_HDR, len,msg);

    	//ret !0 free buf wifi module  else lwip
    	if(ret != 0)
    	{
    		aic_dbg("lwip_wifi input fail2 ret:%d len:%d\n!!!!",ret,len);
            platform_net_buf_rx_free(msg);
    	}

    }
    return ret;

}
#else
int rx_eth_data_process(unsigned char *pdata,
                     unsigned short len, net_if_t *netif)
{
    int ret;
    void *msg;
    unsigned char *pmem = NULL;

    if(!netif_dev.net_inited)
    {
        return -5;
    }

    if(len == 0)
    {
        aic_dbg("lwip_wifi_in_buf len 0\n");
        return -1;
    }

    {
        int fhost_vif_idx = 0;
        struct rt_wlan_device *wlan_dev=NULL;
        int wifi_mode = aic_wifi_fvif_mode_get(fhost_vif_idx);
        if ((wifi_mode == WIFI_MODE_STA) ||
            (wifi_mode == WIFI_MODE_AP)) {
            wlan_dev = (wifi_mode == WIFI_MODE_STA) ? s_wlan_dev : s_ap_dev;
            if (rt_wlan_dev_report_data(wlan_dev, pdata, len)){
                aic_dbg("wifi mode %d input pkt fail\n", wifi_mode);
                return -2;
            }
        } else {
            aic_dbg("invalid wifi mode: %d\n", wifi_mode);
            return -3;
        }
    }

    return 0;

}
#endif /* CONFIG_RX_NOCOPY */

int tx_eth_data_process(uint8_t *data, uint32_t len,void*msg)
{
    TCPIP_PACKET_INFO_T *sent_pkt;
    struct fhost_vif_tag *fhost_vif;
    struct aic_netif_dev *netifdev = &netif_dev;
    net_if_t *net_if = aic_get_vif_net_if();

	//dwmp data for test
	//rwnx_data_dump("net_if_output", data, len);

	if(!netifdev->net_inited)
	{
        aic_dbg("tx:netif is not ready\n");
        //platform_net_buf_rx_free(msg);
		return 0;
	}

	net_out_ing = 1;
	do
	{
		unsigned int reserved_len = aic_offsetof();//offsetof(struct hostdesc, cfm_cb);

        #ifndef CONFIG_RX_WAIT_LWIP
        #ifndef PROTECT_TX_ADMA
        int ret = rtos_semaphore_wait(netifdev->net_tx_buf_sema, 100);
        #else
        int ret = rtos_semaphore_wait(netifdev->net_tx_buf_sema, 200);
        #endif /* CONFIG_ADMA_PROTECT */
        #else
        int ret = rtos_semaphore_wait(netifdev->net_tx_buf_sema, 200);
        #endif /* CONFIG_RX_WAIT_LWIP */

		struct net_tx_buf_tag *tx_buf = NULL, *tx_flowctl = NULL;

		if (ret == 0)
		{
			rtos_mutex_lock(netifdev->net_tx_buf_mutex, -1);
			tx_buf = (struct net_tx_buf_tag *)co_list_pop_front(&netifdev->net_tx_buf_free_list);
			rtos_mutex_unlock(netifdev->net_tx_buf_mutex);
		}

        if (!tx_buf)
        {
            aic_dbg("lwip_wifi_out_buf:tx full\n");
            //platform_net_buf_rx_free(msg);
            goto end;
        }

        #if NET_IF_UAP_NCPYBUF
        tx_buf->buf = data-reserved_len;
        tx_buf->pmsg = msg;
        //aic_dbg("lwip_wifi_out_buf rev:%d p:%p\n",reserved_len,data);
        if(reserved_len > AICWF_WIFI_HDR)
        {
            aic_dbg("lwip_wifi_out_buf:%d\n",reserved_len);
        }
		rtos_net_mempush(data,len,msg,RES_TYPE_MEM_NET_TX);
        #else
        memcpy((tx_buf->buf + reserved_len), data, len);
        //platform_net_buf_rx_free(msg);
        #endif

		sent_pkt = (TCPIP_PACKET_INFO_T *)&(tx_buf->pkt_hdr);
		sent_pkt->data_ptr = tx_buf->buf + reserved_len;
		sent_pkt->data_len = len;
		sent_pkt->net_id   = 0;
		sent_pkt->pkt_type = TCPIP_PKTTYPE_ETHER;

        #if 0
        uint8_t *rdata = (uint8_t *)sent_pkt->data_ptr;
        if (rdata[12] == 0x08 && rdata[13] == 0x00 && rdata[34] == 0x00 && rdata[35] == 0x00) {
            AIC_LOG_PRINTF("NET-OUT ICMP: %d", (rdata[40]<<8) | rdata[41]);
        }
        //rwnx_data_dump("net_if_output_uap", rdata, len);
        #endif

        aic_tx_start(net_if, sent_pkt, NULL, NULL);

    }while(0);

end:
    net_out_ing = 0;
    return 0;
}

int tx_mon_data_process(uint8_t *pkt, uint32_t len)
{
    int ret = 0;
    net_if_t *net_if = aic_get_vif_net_if();
    ret = aic_monitor_tx(net_if, pkt, len);
    return ret;
}

//-------------------------------------------------------------------
// Driver other API define
//-------------------------------------------------------------------
int net_dhcp_start(int net_id)
{
    return 0;
}

#ifdef WIFI_USING_LOOPBACK_NETDEV
void net_loopback_netdev_register(void)
{
    int err;
    #ifdef RT_USING_DEVICE_OPS
    loop_net_dev.parent.parent.ops     = &aicwf_net_ops;
    #else
    loop_net_dev.parent.parent.init    = NULL;
    loop_net_dev.parent.parent.open    = NULL;
    loop_net_dev.parent.parent.close   = NULL;
    loop_net_dev.parent.parent.read    = NULL;
    loop_net_dev.parent.parent.write   = NULL;
    loop_net_dev.parent.parent.control = loop_net_control;
    #endif
    err = af_unix_eth_device_init(&loop_net_dev.parent, "lo");
    if (err) {
        AIC_LOG_PRINTF("AF_UNIX eth dev init err=%d\n", err);
    }
}
#endif
