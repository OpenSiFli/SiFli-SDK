/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-13     Zhaoshimin   first version
 */

#include <rtthread.h>
#include <drivers/usb_host.h>
#include <drivers/ndis.h>
#include "rndis_dev.h"
#include "usbh_rndis.h"
#ifdef RT_USING_LWIP
    /* RT-Thread LWIP ethernet interface */
    #include <netif/ethernetif.h>
    #include <netdev.h>
#endif

#ifdef RT_USBH_RNDIS_DEV
#define RNDIS_DEV_DEBUG
#ifdef  RNDIS_DEV_DEBUG
    #define RNDIS_DEV_PRINTF                rt_kprintf("[RNDIS_DEV] "); rt_kprintf
#else
    #define RNDIS_DEV_PRINTF(...)
#endif /* RNDIS_DEBUG */

#define RNDIS_NET_DEV_NAME  "u0"
#define MAX_ADDR_LEN         6
/* rndis device keepalive time 5000ms*/
#define RNDIS_DEV_KEEPALIVE_TIMEOUT    5000
/*should be the usb Integer multiple of maximum packet length  N*64*/
#define  RNDIS_ETH_BUFFER_LEN   (sizeof(struct rndis_packet_msg) + USB_ETH_MTU + 42)
struct rt_rndis_eth
{
    /* inherit from ethernet device */
    struct eth_device parent;
    struct uhintf *intf;
    rt_mutex_t     rndis_mutex;
    /* interface address info */
    rt_uint8_t   dev_addr[MAX_ADDR_LEN];
    rt_uint16_t  res;
    rt_uint32_t  rndis_speed;
    rt_uint32_t  res32;

    rt_uint8_t tx_buffer[RNDIS_ETH_BUFFER_LEN];
    rt_uint8_t rx_bufferA[RNDIS_ETH_BUFFER_LEN];
    rt_uint8_t rx_bufferB[RNDIS_ETH_BUFFER_LEN];
    rt_size_t  rx_lengthA;
    rt_size_t  rx_lengthB;
    rt_uint8_t *rx_buf_ptr;
    rt_uint32_t frame_debug;
    rt_uint32_t send_packet_counter;
    rt_uint32_t recv_packet_counter;


};
typedef struct rt_rndis_eth *rt_rndis_eth_t;

static struct rt_rndis_eth usbh_rndis_eth_device;


/**
 * This function send rndis msg and receive msg response.
 *
 * @param intf the usb interface instance.
 *
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t rt_rndis_msg_send_recv(struct uhintf *intf, rt_uint8_t *send_buf, rt_uint32_t send_len,
                                       rt_uint8_t *recv_buf, rt_uint32_t recv_size, rt_uint32_t *recv_len)
{
    int ret = 0;
    rt_uint32_t rndis_avial[2] = {0};

    ret = rt_usbh_rndis_control_send(intf, send_buf, send_len, USB_TIMEOUT_BASIC);

    if (ret == send_len)
    {
        /*waite for the interrupt ep*/
        ret = rt_usbh_rndis_interrupt_recv(intf, (rt_uint8_t *)rndis_avial, sizeof(rndis_avial), USB_TIMEOUT_BASIC);
        if ((ret == sizeof(rndis_avial)) && (rndis_avial[0] == RESPONSE_AVAILABLE) &&
                rndis_avial[1] == 0)
        {

            ret = rt_usbh_rndis_control_recv(intf, recv_buf, recv_size, USB_TIMEOUT_BASIC);
            if (ret > 0)
            {
                *recv_len = ret;
                return RT_EOK;
            }

        }
        ret = -RT_ERROR;

    }

    return ret;

}

/**
 * This function send the rndis init msg.
 *
 * @param intf the usb interface instance.
 *
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t rt_rndis_msg_init(struct uhintf *intf)
{
    struct rndis_init_msg   init_msg;
    struct rndis_init_cmplt init_cmplt;
    rt_uint32_t recv_len = 0, request_id = 0;
    int ret = 0;
    urndis_t rndis = RT_NULL;

    rndis = intf->user_data;
    if ((intf ==  RT_NULL) || (rndis == RT_NULL))
    {
        return -RT_ERROR;
    }

    init_msg.MessageType = REMOTE_NDIS_INITIALIZE_MSG;
    init_msg.MessageLength = sizeof(init_msg);
    request_id = rndis->rndis_request_id++;
    init_msg.RequestId = request_id;
    init_msg.MajorVersion = RNDIS_MAJOR_VERSION;
    init_msg.MinorVersion = RNDIS_MINOR_VERSION;
    init_msg.MaxTransferSize = 0x4000;

    ret = rt_rndis_msg_send_recv(intf, (rt_uint8_t *)&init_msg, sizeof(init_msg),
                                 (rt_uint8_t *)&init_cmplt, sizeof(init_cmplt), &recv_len);

    if ((ret == RT_EOK) && (recv_len == sizeof(init_cmplt))
            && (init_cmplt.MessageType == REMOTE_NDIS_INITIALIZE_CMPLT) &&
            (request_id == init_cmplt.RequestId) &&
            (init_cmplt.Status == RNDIS_STATUS_SUCCESS))
    {
        return RT_EOK;
    }
    else
    {
        return -RT_ERROR;
    }

}

/**
 * This function send the rndis init msg.
 *
 * @param intf the usb interface instance.
 *
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t rt_rndis_msg_query(struct uhintf *intf, rt_uint32_t oid, rt_uint32_t query_len,
                                   rt_uint8_t *query_result, rt_uint32_t query_result_size, rt_uint32_t *query_reslut_len)
{
    rndis_query_msg_t    pquery_msg = RT_NULL;
    rndis_query_cmplt_t  pquery_cmplt = RT_NULL;
    rt_uint32_t recv_len = 256, request_id = 0;
    int ret = 0;
    rt_uint32_t oid_buf_len = 0;
    urndis_t rndis = RT_NULL;

    rndis = intf->user_data;
    if ((intf ==  RT_NULL) || (rndis == RT_NULL))
    {
        return -RT_ERROR;
    }

    oid_buf_len = query_len;
    query_len = query_len + sizeof(struct rndis_query_msg);
    pquery_msg = (rndis_query_msg_t)rt_malloc(query_len);
    if (pquery_msg == RT_NULL)
    {
        return -RT_ERROR;
    }

    pquery_msg->MessageType = REMOTE_NDIS_QUERY_MSG;
    pquery_msg->MessageLength = query_len;
    request_id = rndis->rndis_request_id++;
    pquery_msg->RequestId = request_id;
    pquery_msg->Oid = oid;
    pquery_msg->InformationBufferLength = oid_buf_len;
    pquery_msg->InformationBufferOffset = RNDIS_INFO_BUFFER_OFFSET;
    pquery_msg->Reserved = 0;

    if (oid_buf_len > 0)
    {
        rt_memset(pquery_msg->OIDInputBuffer, 0, oid_buf_len);
    }

    pquery_cmplt = (rndis_query_cmplt_t)rt_malloc(recv_len);
    if (pquery_cmplt == RT_NULL)
    {
        return -RT_ERROR;
    }
    rt_memset(pquery_cmplt, 0, recv_len);

    ret = rt_rndis_msg_send_recv(intf, (rt_uint8_t *)pquery_msg, query_len,
                                 (rt_uint8_t *)pquery_cmplt, recv_len, &recv_len);

    rt_free(pquery_msg);
    pquery_msg = RT_NULL;

    if ((ret == RT_EOK) && (recv_len > 0) &&
            (pquery_cmplt->MessageType == REMOTE_NDIS_QUERY_CMPLT) &&
            (request_id == pquery_cmplt->RequestId) &&
            (pquery_cmplt->Status == RNDIS_STATUS_SUCCESS))
    {

        ret =  RT_EOK;
        *query_reslut_len = pquery_cmplt->InformationBufferLength;
        if (pquery_cmplt->InformationBufferLength > 0)
        {
            if (pquery_cmplt->InformationBufferLength <= query_result_size)
            {
                rt_memcpy(query_result, pquery_cmplt->OIDInputBuffer, *query_reslut_len);
            }
            else
            {
                ret =  -RT_ERROR;
            }
        }

    }
    else
    {
        *query_reslut_len = 0;
        ret = -RT_ERROR;
    }
    rt_free(pquery_cmplt);
    rt_mutex_take(usbh_rndis_eth_device.rndis_mutex, RT_WAITING_FOREVER);
    if (rndis->keepalive_timer)
    {
        rt_timer_start(rndis->keepalive_timer);
    }
    rt_mutex_release(usbh_rndis_eth_device.rndis_mutex);
    return ret;

}

/**
 * This function send the rndis set msg.
 *
 * @param intf the usb interface instance.
 *
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t rt_rndis_msg_set(struct uhintf *intf, rt_uint32_t oid,
                                 rt_uint8_t *set_buf, rt_uint32_t set_len)
{
    rndis_set_msg_t     pset_msg = RT_NULL;
    struct rndis_set_cmplt   set_cmplt;
    rt_uint32_t  set_msg_len = 0;
    rt_uint32_t  recv_len = 0, request_id = 0;
    int ret = 0;
    urndis_t rndis = RT_NULL;

    rndis = intf->user_data;
    if ((intf ==  RT_NULL) || (rndis == RT_NULL))
    {
        return -RT_ERROR;
    }

    set_msg_len = sizeof(struct rndis_set_msg) + set_len;
    pset_msg = (rndis_set_msg_t)rt_malloc(set_msg_len);
    if (pset_msg == RT_NULL)
    {
        return -RT_ERROR;
    }
    pset_msg->MessageType = REMOTE_NDIS_SET_MSG;
    pset_msg->MessageLength = set_msg_len;
    request_id = rndis->rndis_request_id++;
    pset_msg->RequestId = request_id;
    pset_msg->Oid = oid;
    pset_msg->InformationBufferLength = set_len;
    pset_msg->InformationBufferOffset = RNDIS_INFO_BUFFER_OFFSET;
    pset_msg->Reserved = 0;
    rt_memcpy(pset_msg->OIDInputBuffer, set_buf, set_len);

    ret = rt_rndis_msg_send_recv(intf, (rt_uint8_t *)pset_msg, set_msg_len,
                                 (rt_uint8_t *)&set_cmplt, sizeof(set_cmplt), &recv_len);

    if ((ret == RT_EOK) && (recv_len == sizeof(set_cmplt))
            && (set_cmplt.MessageType == REMOTE_NDIS_SET_CMPLT) &&
            (request_id == set_cmplt.RequestId) &&
            (set_cmplt.Status == RNDIS_STATUS_SUCCESS))
    {

        ret =  RT_EOK;
    }
    else
    {
        ret = -RT_ERROR;
    }
    rt_free(pset_msg);
    rt_mutex_take(usbh_rndis_eth_device.rndis_mutex, RT_WAITING_FOREVER);
    if (rndis->keepalive_timer)
    {
        rt_timer_start(rndis->keepalive_timer);

    }
    rt_mutex_release(usbh_rndis_eth_device.rndis_mutex);
    return ret;

}

/**
 * This function send the rndis data.
 *
 * @param intf the usb interface instance.
 *
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t rt_rndis_msg_data_send(struct uhintf *intf, rt_uint8_t *buffer, int nbytes)
{
    int ret = 0;
    urndis_t rndis = RT_NULL;

    rndis = intf->user_data;

    if ((rndis == RT_NULL) || (intf == RT_NULL))
    {
        return -RT_ERROR;
    }

    ret = rt_usbh_rndis_bulk_send(intf, buffer, nbytes, USB_TIMEOUT_BASIC);
    rt_mutex_take(usbh_rndis_eth_device.rndis_mutex, RT_WAITING_FOREVER);
    if (rndis->keepalive_timer)
    {
        rt_timer_start(rndis->keepalive_timer);
    }
    rt_mutex_release(usbh_rndis_eth_device.rndis_mutex);

    return ret;

}


/**
 * This function recv the rndis data.
 *
 * @param intf the usb interface instance.
 *
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t rt_rndis_msg_data_recv(struct uhintf *intf, rt_uint8_t *buffer, int nbytes)
{
    int ret = 0;
    urndis_t rndis = RT_NULL;

    rndis = intf->user_data;

    if ((rndis == RT_NULL) || (intf == RT_NULL))
    {
        return -RT_ERROR;
    }


    ret = rt_usbh_rndis_bulk_recv(intf, buffer, nbytes, 0);
    rt_mutex_take(usbh_rndis_eth_device.rndis_mutex, RT_WAITING_FOREVER);
    if (rndis->keepalive_timer)
    {
        rt_timer_start(rndis->keepalive_timer);
    }
    rt_mutex_release(usbh_rndis_eth_device.rndis_mutex);

    return ret;

}



/**
 * This function send the rndis set msg.
 *
 * @param intf the usb interface instance.
 *
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t rt_rndis_keepalive_msg(struct uhintf *intf)
{

    rt_uint32_t  recv_len = 0;
    int ret = 0;
    struct rndis_keepalive_msg msg;
    struct rndis_keepalive_cmplt cmplt;
    urndis_t rndis = RT_NULL;

    rndis = intf->user_data;
    if ((intf ==  RT_NULL) || (rndis == RT_NULL))
    {
        return -RT_ERROR;
    }

    msg.MessageType = REMOTE_NDIS_KEEPALIVE_MSG;
    msg.RequestID = rndis->rndis_request_id++;
    msg.MessageLength = sizeof(msg);

    ret = rt_rndis_msg_send_recv(intf, (rt_uint8_t *)&msg, sizeof(msg),
                                 (rt_uint8_t *)&cmplt, sizeof(cmplt), &recv_len);
    /*when the usb device run about 4hours, it maybe return RT_ERROR, so the next
      keepalive query will return the last request id.
      solution: we donot check the request id equal only! zhaoshimin 20210518
      fix this bug zhaoshimin because the low level usb driver bug 20211124*/
    if ((ret == RT_EOK) && (recv_len == sizeof(cmplt))
            && (cmplt.MessageType == REMOTE_NDIS_KEEPALIVE_CMPLT)
            && (cmplt.Status == RNDIS_STATUS_SUCCESS) && (cmplt.RequestID == msg.RequestID))
    {

        ret =  RT_EOK;
    }
    else
    {
        ret = -RT_ERROR;
    }

    return ret;

}

/**
 * This function will send the bulk data to the usb device instance,
 *
 * @param device the usb device instance.
 * @param type the type of descriptor bRequest.
 * @param buffer the data buffer to save requested data
 * @param nbytes the size of buffer
 *
 * @return the error code, RT_EOK on successfully.
 */
void rt_usbh_rndis_data_recv(void *pdata)
{
    int ret = 0;
    struct uhintf *intf = (struct uhintf *)pdata;
    urndis_t rndis = RT_NULL;
    rt_rndis_eth_t device = RT_NULL;
    rndis_packet_msg_t pmsg = RT_NULL;

    rndis = intf->user_data;
    device = (rt_rndis_eth_t)rndis->rndis_dev;

    if ((pdata == RT_NULL) || (rndis == RT_NULL) ||
            (device == RT_NULL))
    {
        return;
    }

    while (1)
    {
        ret = rt_rndis_msg_data_recv(intf, device->rx_buf_ptr, RNDIS_ETH_BUFFER_LEN);
        if (ret > 0)
        {
            pmsg = (rndis_packet_msg_t)device->rx_buf_ptr;

            if (device->frame_debug == RT_TRUE)
            {
                //hex_data_print("rndis eth rx", device->rx_buf_ptr, ret);
            }
            if (device->rx_buf_ptr == device->rx_bufferA)
            {
                if (device->rx_lengthA)
                {
                    RNDIS_DEV_PRINTF("Rndis deivce rx bufferA overwrite!\n");
                }
                device->rx_lengthA = ret;
                device->rx_buf_ptr = device->rx_bufferB;
            }
            else
            {
                if (device->rx_lengthB)
                {
                    RNDIS_DEV_PRINTF("Rndis deivce rx bufferB overwrite!\n");
                }
                device->rx_lengthB = ret;
                device->rx_buf_ptr = device->rx_bufferA;
            }

            if ((pmsg->MessageType == REMOTE_NDIS_PACKET_MSG) && (pmsg->MessageLength == ret))
            {
                device->recv_packet_counter++;
                eth_device_ready((struct eth_device *)device);
            }
            else
            {
                RNDIS_DEV_PRINTF("Rndis deivce recv data error!,ret=%d\n", ret);
            }

        }
        else if (ret == 0)
        {
            RNDIS_DEV_PRINTF("Rndis deivce recv len:%d!\n", ret);
            rt_thread_mdelay(2000);
        }
        else
        {
            RNDIS_DEV_PRINTF("Rndis deivce recv error:%d!\n", ret);
            rt_thread_mdelay(10);
        }

    }
}

/**
 * This function power off the rndis device and power up it again.
 *
 * @param intf the usb interface instance.
 *
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t rt_rndis_dev_power(struct uhintf *intf, rt_uint32_t time)
{
    /*power off the rndis device*/
    rt_usbh_hub_clear_port_feature(intf->device->parent_hub, intf->device->port, PORT_FEAT_POWER);
    if (time)
    {
        rt_thread_mdelay(time);
        /*power up the rndis device */
        rt_usbh_hub_set_port_feature(intf->device->parent_hub, intf->device->port, PORT_FEAT_POWER);
    }

    return RT_EOK;

}

void rndis_dev_keepalive_timeout(void *pdata)
{
    struct uhintf *intf = (struct uhintf *)pdata;
    urndis_t rndis = RT_NULL;
    static rt_uint32_t keepalive_error = 0;


    rndis = (urndis_t)intf->user_data;
    if ((intf == RT_NULL) || (rndis == RT_NULL))
    {
        return;
    }

    if (RT_EOK == rt_rndis_keepalive_msg(intf))
    {
        RNDIS_DEV_PRINTF("rndis dev keepalive success!\n");
        keepalive_error = 0;
        rt_mutex_take(usbh_rndis_eth_device.rndis_mutex, RT_WAITING_FOREVER);
        if (rndis->keepalive_timer)
        {
            rt_timer_start(rndis->keepalive_timer);
        }
        rt_mutex_release(usbh_rndis_eth_device.rndis_mutex);
    }
    else
    {
        keepalive_error++;
        RNDIS_DEV_PRINTF("rndis dev keepalive timeout!\n");
        if (keepalive_error > 3)
        {
            keepalive_error = 0;
            rt_rndis_dev_power(intf, RNDIS_DEV_POWER_OFF_TIME);
            rndis->rndis_state = RNDIS_BUS_INITIALIZED;
        }
    }


}

/**
 * This function will run rndis driver when usb disk is detected.
 *
 * @param intf the usb interface instance.
 *
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_rndis_run(struct uhintf *intf)
{
    rt_err_t ret = 0;
    urndis_t rndis = RT_NULL;
    rt_uint8_t *recv_buf = RT_NULL, *psupport_oid_list = RT_NULL;
    uint8_t time_out_num = 0;
    rt_uint32_t recv_len = 256;
    rt_uint32_t poid = 0;
    rt_uint32_t *pquery_rlt = RT_NULL;
    rt_uint32_t  i = 0, j = 0, oid_len = 0;
    struct netdev *netdev = RT_NULL;

    /* check parameter */
    RT_ASSERT(intf != RT_NULL);

    rndis = intf->user_data;

    /*rndis device is connected to the host via the usb bus, the host is not yet
      configured to exchanged any RNDIS message*/
    rndis->rndis_state = RNDIS_BUS_UNINITIALIZED;

    /*set config*/
    ret = rt_usbh_set_configure(intf->device, 1);
    if (ret != RT_EOK)
    {
        RNDIS_DEV_PRINTF("set config faile!\n");
        goto __exit;
    }

    /* set interface */
    ret = rt_usbh_set_interface(intf->device, intf->intf_desc->bInterfaceNumber);
    if (ret != RT_EOK)
    {
        RNDIS_DEV_PRINTF("set interface faile!\n");
        goto __exit;
    }

    /*The host has successfully mapped and initialized
      the RNDIS protocol data and control channels on the bus transport*/
    rndis->rndis_state = RNDIS_BUS_INITIALIZED;


    ret = rt_rndis_msg_init(intf);
    if (ret != RT_EOK)
    {
        RNDIS_DEV_PRINTF("rt_rndis_msg_init faile!\n");
        goto __exit;
    }

    /*The host is configured to send and receive any of the RNDIS control messages for suitably
     configuring or querying the device, to receive status indications from the device,
     to reset the device, or to tear down the data and control channels*/
    rndis->rndis_state = RNDIS_INITIALIZED;
#if 0
    rndis->keepalive_timer = rt_timer_create("keeplive", rndis_dev_keepalive_timeout,
                             intf,
                             RT_TICK_PER_SECOND * RNDIS_DEV_KEEPALIVE_TIMEOUT / 1000,
                             RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);

    if (rndis->keepalive_timer == RT_NULL)
    {
        ret = -RT_ENOMEM;
        goto __exit;
    }
#endif

    recv_buf = rt_malloc(recv_len);
    if (recv_buf == RT_NULL)
    {
        ret = -RT_ENOMEM;
        goto __exit;
    }

    /*OID_GEN_SUPPORTED_LIST = 0x00010101*/
    ret = rt_rndis_msg_query(intf, OID_GEN_SUPPORTED_LIST, 0, recv_buf, recv_len, &oid_len);

    if (ret != RT_EOK)
    {
        RNDIS_DEV_PRINTF("query the oid OID_GEN_SUPPORTED_LIST error!\n");
        goto __exit;
    }

    oid_len = oid_len - 4;
    psupport_oid_list = rt_malloc(oid_len);
    if (psupport_oid_list == RT_NULL)
    {
        ret = -RT_ENOMEM;
        goto __exit;
    }


    rndis->rndis_dev = (struct rt_device *)&usbh_rndis_eth_device;
    usbh_rndis_eth_device.intf = intf;

    rndis->rndis_recv = rt_thread_create("rndis",
                                         (void (*)(void *parameter))rt_usbh_rndis_data_recv,
                                         intf,
                                         1024 + 512,
                                         6,
                                         20);

    if (rndis->rndis_recv == RT_NULL)
    {

        ret = -RT_ENOMEM;
        goto __exit;
    }


    /*get the support oid list, the first is OID_GEN_SUPPORTED_LIST, donnot remeber it*/
    rt_memcpy(psupport_oid_list, &recv_buf[4], oid_len);

    /* query and set the support oid list*/
    for (i = 0; i < oid_len; i = i + 4)
    {
        poid =  psupport_oid_list[i + 3] << 24;
        poid += psupport_oid_list[i + 2] << 16;
        poid += psupport_oid_list[i + 1] << 8;
        poid += psupport_oid_list[i];
        recv_len = 256;
        pquery_rlt = (rt_uint32_t *)recv_buf;
        RNDIS_DEV_PRINTF("%s %d *poid=0x%x\n", __func__, __LINE__, poid);
        switch (poid)
        {
        case OID_GEN_PHYSICAL_MEDIUM:
            time_out_num = 5;
            while (time_out_num --)
            {
                ret = rt_rndis_msg_query(intf, poid, 4, recv_buf, recv_len, &recv_len);
                if ((ret == RT_EOK) && (recv_len == 4))
                {
                    /*physical mdium is NDIS_MEDIUM_802_3*/
                    break;
                }
                else
                {
                    RNDIS_DEV_PRINTF("query the oid OID_GEN_PHYSICAL_MEDIUM error!\n");
                    ret = -RT_ERROR;
                }
            }
            break;

        case OID_GEN_MAXIMUM_FRAME_SIZE:
            time_out_num = 5;
            while (time_out_num --)
            {
                ret = rt_rndis_msg_query(intf, poid, 4, recv_buf, recv_len, &recv_len);
                if ((ret == RT_EOK) && (recv_len == 4))
                {
                    /*the maximum frame size(MTU) is 0x05dc=1500*/
                    break;
                }
                else
                {
                    RNDIS_DEV_PRINTF("query the oid OID_GEN_MAXIMUM_FRAME_SIZE error!\n");
                    ret = -RT_ERROR;
                }
            }
            break;

        case OID_GEN_LINK_SPEED:
            time_out_num = 5;
            while (time_out_num --)
            {
                ret = rt_rndis_msg_query(intf, poid, 4, recv_buf, recv_len, &recv_len);
                if ((ret == RT_EOK) && (recv_len == 4))
                {
                    /*the LINK SPEED is 100Mbps*/
                    RNDIS_DEV_PRINTF("query =%d\n", *pquery_rlt);
                    usbh_rndis_eth_device.rndis_speed = *pquery_rlt;
                    break;
                }
                else
                {
                    RNDIS_DEV_PRINTF("query the oid OID_GEN_LINK_SPEED error!\n");
                    ret = -RT_ERROR;
                }
            }
            break;

        case OID_GEN_MEDIA_CONNECT_STATUS:
            time_out_num = 5;
            while (time_out_num --)
            {
                ret = rt_rndis_msg_query(intf, poid, 4, recv_buf, recv_len, &recv_len);
                if ((ret == RT_EOK) && (recv_len == 4))
                {
                    if (NDIS_MEDIA_STATE_CONNECTED == *pquery_rlt)
                    {
                        eth_device_linkchange(&usbh_rndis_eth_device.parent, RT_TRUE);
                    }
                    else
                    {
                        eth_device_linkchange(&usbh_rndis_eth_device.parent, RT_FALSE);
                    }
                    break;
                }
                else
                {
                    RNDIS_DEV_PRINTF("query the oid OID_GEN_MEDIA_CONNECT_STATUS error!\n");
                    ret = -RT_ERROR;
                }
                //eth_device_linkchange(&usbh_rndis_eth_device.parent, RT_TRUE);
            }
            break;

        case OID_802_3_MAXIMUM_LIST_SIZE:
            time_out_num = 5;
            while (time_out_num --)
            {
                ret = rt_rndis_msg_query(intf, poid, 4, recv_buf, recv_len, &recv_len);
                if ((ret == RT_EOK) && (recv_len == 4))
                {
                    break;
                }
                else
                {
                    RNDIS_DEV_PRINTF("query the oid OID_802_3_MAXIMUM_LIST_SIZE error!\n");
                    ret = -RT_ERROR;
                }
            }
            break;

        case OID_802_3_PERMANENT_ADDRESS:
        case OID_802_3_CURRENT_ADDRESS:
            time_out_num = 5;
            while (time_out_num --)
            {
                ret = rt_rndis_msg_query(intf, poid, MAX_ADDR_LEN, recv_buf, recv_len, &recv_len);
                if ((ret == RT_EOK) && (recv_len == MAX_ADDR_LEN))
                {
                    for (j = 0; j < MAX_ADDR_LEN; j++)
                    {
                        usbh_rndis_eth_device.dev_addr[j] = recv_buf[j];
                    }
                    /* update the mac addr to netif interface */
                    rt_device_control((rt_device_t)&usbh_rndis_eth_device.parent, NIOCTL_GADDR,
                                      usbh_rndis_eth_device.parent.netif->hwaddr);
                    netdev = netdev_get_by_name(RNDIS_NET_DEV_NAME);
                    if (netdev)
                    {
                        rt_memcpy(netdev->hwaddr, recv_buf, MAX_ADDR_LEN);

                    }
                    break;
                }
                else
                {
                    RNDIS_DEV_PRINTF("query the oid OID_802_3_CURRENT_ADDRESS error!\n");
                    ret = -RT_ERROR;
                }
            }
            break;
        case OID_GEN_CURRENT_PACKET_FILTER:
            time_out_num = 5;
            while (time_out_num --)
            {
                *pquery_rlt = 0x0b;
                if (RT_EOK == rt_rndis_msg_set(intf, poid, (rt_uint8_t *)pquery_rlt, 4))
                {
                    break;
                }
                else
                {
                    RNDIS_DEV_PRINTF("set the oid OID_GEN_CURRENT_PACKET_FILTER error!\n");
                }
            }
            break;

        case OID_802_3_MULTICAST_LIST:
            time_out_num = 5;
            while (time_out_num --)
            {
                /*set multicast list it is a mac list*/
                recv_buf[0] = 0x01;
                recv_buf[1] = 0x00;
                recv_buf[2] = 0x5E;
                recv_buf[3] = 0x00;
                recv_buf[4] = 0x00;
                recv_buf[5] = 0x01;

                ret = rt_rndis_msg_set(intf, poid, recv_buf, 6);
                if (RT_EOK == ret)
                {
                    break;
                }
                else
                {
                    RNDIS_DEV_PRINTF("set the oid OID_802_3_MULTICAST_LIST error!\n");

                }
            }
            break;

#if 0
        case OID_GEN_CURRENT_PACKET_FILTER:
            *pquery_rlt = 0x0f;
            ret = rt_rndis_msg_set(intf, poid, (rt_uint8_t *)pquery_rlt, 4);
            if (ret == RT_EOK)
            {
            }
            else
            {
                RNDIS_DEV_PRINTF("set the oid OID_GEN_CURRENT_PACKET_FILTER error!\n");

            }
            break;
#endif
        case OID_GEN_MAXIMUM_TOTAL_SIZE:
            time_out_num = 5;
            while (time_out_num --)
            {
                ret = rt_rndis_msg_query(intf, poid, 4, recv_buf, recv_len, &recv_len);
                if ((ret == RT_EOK) && (recv_len == 4))
                {
                    break;
                }
                else
                {
                    RNDIS_DEV_PRINTF("query the oid OID_GEN_MAXIMUM_TOTAL_SIZE error!\n");
                    ret = -RT_ERROR;
                }
            }
            break;

        default:
            break;
        }

        if (ret == -RT_ERROR)
        {
            break;
        }
    }
__exit:
    if (ret == RT_EOK)
    {
        /*This state is entered after the host has received REMOTE_NDIS_SET_CMPLT
        messages from the device in response to the REMOTE_NDIS_SET_MSG
        that it had sent earlier to the device with all the OIDs required to configure the device for data transfer.
        When the host is in this state, apart from the control messages,
        it can exchange REMOTE_NDIS_PACKET_MSG messages for network data transfer with the device on the data channel*/
        rndis->rndis_state = RNDIS_DATA_INITIALIZED;
        rt_thread_startup(rndis->rndis_recv);
        RNDIS_DEV_PRINTF("rndis dev start!\n");

    }
    else
    {
        RNDIS_DEV_PRINTF("rndis dev faile!\n");
        /*rndis device run error, power off the device, try it agin*/
        rt_rndis_dev_power(intf, RNDIS_DEV_POWER_OFF_TIME);

    }

    if (recv_buf)
    {
        rt_free(recv_buf);
        recv_buf = RT_NULL;
    }
    if (psupport_oid_list)
    {
        rt_free(psupport_oid_list);
        psupport_oid_list = RT_NULL;
    }
    return ret;
}

rt_err_t rt_rndis_stop(struct uhintf *intf)
{
    urndis_t rndis = RT_NULL;

    rndis = intf->user_data;

    if (rndis->rndis_recv)
    {
        rt_thread_delete(rndis->rndis_recv);
        rndis->rndis_recv = RT_NULL;
    }
    eth_device_linkchange(&usbh_rndis_eth_device.parent, RT_FALSE);
    usbh_rndis_eth_device.intf = RT_NULL;


    /*disable the other thread etx call the rt_timer_start(rndis->keepalive_timer) cause the RT_ASSERT(rt_object_get_type(&timer->parent) == RT_Object_Class_Timer)*/
    rt_mutex_take(usbh_rndis_eth_device.rndis_mutex, RT_WAITING_FOREVER);
    if (rndis->keepalive_timer)
    {
        rt_timer_stop(rndis->keepalive_timer);
        rt_timer_delete(rndis->keepalive_timer);
        rndis->keepalive_timer = RT_NULL;
    }
    rt_mutex_release(usbh_rndis_eth_device.rndis_mutex);

    rndis->rndis_state = RNDIS_BUS_UNINITIALIZED;

    RNDIS_DEV_PRINTF("rndis dev stop!\n");
    return RT_EOK;
}

/**
 * This function rndis eth device.
 *
 * @param intf the usb interface instance.
 *
 * @return the error code, RT_EOK on successfully.
 */
#ifdef RT_USING_LWIP
/* initialize the interface */
static rt_err_t rt_rndis_eth_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t rt_rndis_eth_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t rt_rndis_eth_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t rt_rndis_eth_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t rt_rndis_eth_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}
static rt_err_t rt_rndis_eth_control(rt_device_t dev, int cmd, void *args)
{
    rt_rndis_eth_t rndis_eth_dev = (rt_rndis_eth_t)dev;
    switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if (args)
        {
            rt_memcpy(args, rndis_eth_dev->dev_addr, MAX_ADDR_LEN);
        }
        else
        {
            return -RT_ERROR;
        }
        break;

    case NIOTCTL_GTXCOUNTER:
        if (args)
        {
            *(rt_uint32_t *)args = rndis_eth_dev->send_packet_counter;
        }
        else
        {
            return -RT_ERROR;
        }
        break;

    case NIOTCTL_GRXCOUNTER:
        if (args)
        {
            *(rt_uint32_t *)args = rndis_eth_dev->recv_packet_counter;
        }
        else
        {
            return -RT_ERROR;
        }
        break;
    default :
        break;
    }

    return RT_EOK;
}

/* ethernet device interface */


/* reception packet. */
struct pbuf *rt_rndis_eth_rx(rt_device_t dev)
{
    struct pbuf *p = RT_NULL;
    rt_uint32_t offset = 0;
    rt_rndis_eth_t device = (rt_rndis_eth_t)dev;
    rt_uint32_t  recv_len = 0;


    rndis_packet_msg_t pmsg = RT_NULL;

    if (device->rx_buf_ptr == device->rx_bufferA)
    {
        pmsg = (rndis_packet_msg_t)device->rx_bufferB;
        recv_len = device->rx_lengthB;
    }
    else
    {
        pmsg = (rndis_packet_msg_t)device->rx_bufferA;
        recv_len = device->rx_lengthA;
    }

    if ((recv_len == 0) || (pmsg->DataLength == 0))
    {
        return RT_NULL;
    }


    /* allocate buffer */
    p = pbuf_alloc(PBUF_LINK, pmsg->DataLength, PBUF_RAM);
    if (p != RT_NULL)
    {
        struct pbuf *q;

        for (q = p; q != RT_NULL; q = q->next)
        {
            /* Copy the received frame into buffer from memory pointed by the current ETHERNET DMA Rx descriptor */
            memcpy(q->payload,
                   (rt_uint8_t *)((pmsg->data) + offset),
                   q->len);
            offset += q->len;
        }
    }

    if (device->rx_buf_ptr == device->rx_bufferA)
    {
        device->rx_lengthB = 0;
    }
    else
    {
        device->rx_lengthA = 0;
    }

    return p;
}

/* transmit packet. */
rt_err_t rt_rndis_eth_tx(rt_device_t dev, struct pbuf *p)
{
    struct pbuf *q;
    rt_uint8_t *buffer = RT_NULL;
    rt_err_t result = RT_EOK;
    rt_rndis_eth_t device = (rt_rndis_eth_t)dev;
    rndis_packet_msg_t msg = RT_NULL;

    if (!device->parent.link_status)
    {
        RNDIS_DEV_PRINTF("linkdown, drop pkg\r\n");
        return RT_EOK;
    }

    RT_ASSERT((p->tot_len + sizeof(struct rndis_packet_msg)) < sizeof(device->tx_buffer));
    if (p->tot_len > sizeof(device->tx_buffer))
    {
        RNDIS_DEV_PRINTF("RNDIS MTU is:%d, but the send packet size is %d\r\n",
                         sizeof(device->tx_buffer), p->tot_len);
        p->tot_len = sizeof(device->tx_buffer);
    }

    msg = (rndis_packet_msg_t)&device->tx_buffer;
    msg->MessageType = REMOTE_NDIS_PACKET_MSG;
    msg->DataOffset = sizeof(struct rndis_packet_msg) - 8;
    msg->DataLength = p->tot_len;
    msg->OOBDataLength = 0;
    msg->OOBDataOffset = 0;
    msg->NumOOBDataElements = 0;
    msg->PerPacketInfoOffset = 0;
    msg->PerPacketInfoLength = 0;
    msg->VcHandle = 0;
    msg->Reserved = 0;
    msg->MessageLength = sizeof(struct rndis_packet_msg) + p->tot_len;

    buffer = msg->data;
    for (q = p; q != NULL; q = q->next)
    {
        memcpy(buffer, q->payload, q->len);
        buffer += q->len;
    }

    /* send */
    if ((msg->MessageLength & 0x3F) == 0)
    {
        /* pad a dummy. */
        msg->MessageLength += 1;
    }

    if (device->frame_debug == RT_TRUE)
    {
        //hex_data_print("rndis eth tx", (rt_uint8_t *)msg, msg->MessageLength);
    }
    result = rt_rndis_msg_data_send(device->intf, (rt_uint8_t *)msg, msg->MessageLength);
    device->send_packet_counter++;

    return result;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops rndis_device_ops =
{
    rt_rndis_eth_init,
    rt_rndis_eth_open,
    rt_rndis_eth_close,
    rt_rndis_eth_read,
    rt_rndis_eth_write,
    rt_rndis_eth_control
}
#endif
#endif

int usbh_rndis_eth_device_init(void)
{

    /* OUI 00-00-00, only for test. */
    usbh_rndis_eth_device.dev_addr[0] = 0xFF;
    usbh_rndis_eth_device.dev_addr[1] = 0xFF;
    usbh_rndis_eth_device.dev_addr[2] = 0xFF;
    /* generate random MAC. */
    usbh_rndis_eth_device.dev_addr[3] = 0xFF;
    usbh_rndis_eth_device.dev_addr[4] = 0xFF;
    usbh_rndis_eth_device.dev_addr[5] = 0xFF;

    usbh_rndis_eth_device.rndis_mutex = rt_mutex_create("rndis", RT_IPC_FLAG_PRIO);

    if (usbh_rndis_eth_device.rndis_mutex == RT_NULL)
    {
        RNDIS_DEV_PRINTF("Rndis mutex creat faile!\r\n");
    }


#ifdef RT_USING_DEVICE_OPS
    usbh_rndis_eth_device.parent.parent.ops           = &rndis_device_ops;
#else
    usbh_rndis_eth_device.parent.parent.init          = rt_rndis_eth_init;
    usbh_rndis_eth_device.parent.parent.open          = rt_rndis_eth_open;
    usbh_rndis_eth_device.parent.parent.close         = rt_rndis_eth_close;
    usbh_rndis_eth_device.parent.parent.read          = rt_rndis_eth_read;
    usbh_rndis_eth_device.parent.parent.write         = rt_rndis_eth_write;
    usbh_rndis_eth_device.parent.parent.control       = rt_rndis_eth_control;
#endif
    usbh_rndis_eth_device.parent.parent.user_data     = RT_NULL;

    usbh_rndis_eth_device.parent.eth_rx               = rt_rndis_eth_rx;
    usbh_rndis_eth_device.parent.eth_tx               = rt_rndis_eth_tx;

    /* register eth device */
    usbh_rndis_eth_device.rx_lengthA = 0;
    usbh_rndis_eth_device.rx_lengthB = 0;
    usbh_rndis_eth_device.rx_buf_ptr = usbh_rndis_eth_device.rx_bufferA;
    usbh_rndis_eth_device.frame_debug = RT_FALSE;

    usbh_rndis_eth_device.send_packet_counter = 0;
    usbh_rndis_eth_device.recv_packet_counter = 0;


    eth_device_init(&usbh_rndis_eth_device.parent, RNDIS_NET_DEV_NAME);
    netif_set_up(usbh_rndis_eth_device.parent.netif);
    netif_set_link_up(usbh_rndis_eth_device.parent.netif);

    eth_device_linkchange(&usbh_rndis_eth_device.parent, RT_TRUE);

    return RT_EOK;
}
INIT_APP_EXPORT(usbh_rndis_eth_device_init);

int rndis_lwip_init(void)
{
    int lwip_system_init(void);
    lwip_system_init();
    return RT_EOK;
}
INIT_PREV_EXPORT(rndis_lwip_init);


/*********************************************************************************************************
** Function name        eth_rndis_frame_debug()
** Descriptions:        rndis frame print
** input parameters
** output parameters     None
** Returned value:      RT_EOK or RT_ERROR
*********************************************************************************************************/
static void eth_rndis_frame_debug(int argc, char **argv)
{


    if (argc != 2)
    {
        rt_kprintf("Please check the command you enter, it like this: rndis_debug on/off!\n");
    }
    else
    {


        if (rt_strcmp(argv[1], "on") == 0)
        {
            usbh_rndis_eth_device.frame_debug = RT_TRUE;
        }
        else
        {
            usbh_rndis_eth_device.frame_debug = RT_FALSE;
        }
    }
}

#ifdef FINSH_USING_MSH
    #include <finsh.h>
    MSH_CMD_EXPORT_ALIAS(eth_rndis_frame_debug, rndis_debug, set eth rndis frame print);
#endif /* FINSH_USING_MSH */


#endif

