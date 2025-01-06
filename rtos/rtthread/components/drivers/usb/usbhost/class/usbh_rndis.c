/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-12     Zhaoshimin   first version
 */

#include <rtthread.h>
#include <drivers/usb_host.h>
#include "usbh_rndis.h"
#include "rndis_dev.h"

#ifdef RT_USBH_RNDIS

extern rt_err_t rt_rndis_run(struct uhintf *intf);
extern rt_err_t rt_rndis_stop(struct uhintf *intf);

static struct uclass_driver rndis_driver;

/**
 * This function will do send control out data to the usb device instance,
 *
 * @param device the usb device instance.
 * @param type the type of descriptor bRequest.
 * @param buffer the data buffer to save requested data
 * @param nbytes the size of buffer
 *
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_usbh_rndis_control_send(struct uhintf *intf, rt_uint8_t *buffer, int nbytes, int timeout)
{
    struct urequest setup;
    struct uinstance *device;

    RT_ASSERT(intf != RT_NULL);

    setup.request_type = USB_REQ_TYPE_DIR_OUT | USB_REQ_TYPE_CLASS |
                         USB_REQ_TYPE_INTERFACE;
    setup.bRequest = 0;
    setup.wValue = 0;
    setup.wIndex = 0;
    setup.wLength = nbytes;

    /* get usb device instance from the interface instance */
    device = intf->device;

    /* send setup*/
    if (rt_usb_hcd_setup_xfer(device->hcd, device->pipe_ep0_out, &setup, timeout) == 8)
    {
        /*send out token and data*/
        if (rt_usb_hcd_pipe_xfer(device->hcd, device->pipe_ep0_out, buffer, nbytes, timeout) == nbytes)
        {
            /* Send 0 bytes out packet */
            if (rt_usb_hcd_pipe_xfer(device->hcd, device->pipe_ep0_in, RT_NULL, 0, timeout) == 0)
            {
                return nbytes;
            }
        }
    }
    return -RT_ERROR;
}

/**
 * This function will recv the control request from the usb device instance,
 *
 * @param device the usb device instance.
 * @param type the type of descriptor bRequest.
 * @param buffer the data buffer to save requested data
 * @param nbytes the size of buffer
 *
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_usbh_rndis_control_recv(struct uhintf *intf, rt_uint8_t *buffer, int nbytes, int timeout)
{
    struct urequest setup;
    struct uinstance *device;
    int read_len = 0;

    RT_ASSERT(intf != RT_NULL);

    setup.request_type = USB_REQ_TYPE_DIR_IN | USB_REQ_TYPE_CLASS |
                         USB_REQ_TYPE_INTERFACE;
    setup.bRequest = USB_REQ_TYPE_INTERFACE;
    setup.wValue = 0;
    setup.wIndex = 0;
    setup.wLength = 0x100;

    /* get usb device instance from the interface instance */
    device = intf->device;

    /* send setup*/
    if (rt_usb_hcd_setup_xfer(device->hcd, device->pipe_ep0_out, &setup, timeout) == 8)
    {
        /*send IN token and read data*/
        read_len = rt_usb_hcd_pipe_xfer(device->hcd, device->pipe_ep0_in, buffer, nbytes, timeout);
        if (read_len > 0)
        {
            /* Send 0 bytes out packet */
            if (rt_usb_hcd_pipe_xfer(device->hcd, device->pipe_ep0_out, RT_NULL, 0, timeout) == 0)
            {
                return read_len;
            }
        }
    }
    return -RT_ERROR;
}

/**
 * This function will recv the interrupt data from the usb device instance,
 *
 * @param device the usb device instance.
 * @param type the type of descriptor bRequest.
 * @param buffer the data buffer to save requested data
 * @param nbytes the size of buffer
 *
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_usbh_rndis_interrupt_recv(struct uhintf *intf, rt_uint8_t *buffer, int nbytes, int timeout)
{

    struct uinstance *device;
    int read_len = 0;
    urndis_t rndis = RT_NULL;

    RT_ASSERT(intf != RT_NULL);

    /* get usb device instance from the interface instance */
    device = intf->device;
    rndis = (urndis_t)intf->user_data;


    /*send IN token and read data*/
    read_len = rt_usb_hcd_pipe_xfer(device->hcd, rndis->pipe_int, buffer, nbytes, timeout);
    if (read_len > 0)
    {
        return read_len;
    }
    else
    {

        return -RT_ERROR;
    }
}

/**
 * This function will recv the bulk data from the usb device instance,
 *
 * @param device the usb device instance.
 * @param type the type of descriptor bRequest.
 * @param buffer the data buffer to save requested data
 * @param nbytes the size of buffer
 *
 * @return the error code, RT_EOK on successfully.
 */
rt_err_t rt_usbh_rndis_bulk_recv(struct uhintf *intf, rt_uint8_t *buffer, int nbytes, int timeout)
{

    struct uinstance *device;
    int read_len = 0;
    urndis_t rndis = RT_NULL;

    RT_ASSERT(intf != RT_NULL);

    /* get usb device instance from the interface instance */
    device = intf->device;
    rndis = (urndis_t)intf->user_data;


    /*send IN token and read data*/
    read_len = rt_usb_hcd_pipe_xfer(device->hcd, rndis->pipe_in, buffer, nbytes, timeout);
    if (read_len >= 0)
    {
        return read_len;
    }
    else
    {

        return -RT_ERROR;
    }
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
rt_err_t rt_usbh_rndis_bulk_send(struct uhintf *intf, rt_uint8_t *buffer, int nbytes, int timeout)
{

    struct uinstance *device;
    int send_len = 0;
    urndis_t rndis = RT_NULL;

    RT_ASSERT(intf != RT_NULL);

    /* get usb device instance from the interface instance */
    device = intf->device;
    rndis = (urndis_t)intf->user_data;


    /*send IN token and read data*/
    send_len = rt_usb_hcd_pipe_xfer(device->hcd, rndis->pipe_out, buffer, nbytes, timeout);
    if (send_len == nbytes)
    {
        return send_len;
    }
    else
    {

        return -RT_ERROR;
    }
}



/**
 * This function will run rndis class driver when usb device is detected
 * and identified as a mass storage class device, it will continue to do the enumulate
 * process.
 *
 * @param arg the argument.
 *
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t rt_usbh_rndis_enable(void *arg)
{
    int i = 0;
    urndis_t rndis = RT_NULL;
    struct uhintf **intf = arg;
    uep_desc_t ep_desc = RT_NULL;


    /* parameter check */
    if (intf == RT_NULL)
    {
        rt_kprintf("the interface is not available\n");
        return -RT_EIO;
    }

    RT_DEBUG_LOG(RT_DEBUG_USB, ("subclass %d, protocal %d\n",
                                intf[0]->intf_desc->bInterfaceSubClass,
                                intf[0]->intf_desc->bInterfaceProtocol));

    RT_DEBUG_LOG(RT_DEBUG_USB, ("rt_usbh_rndis_enable\n"));

    /* only support SCSI subclass and bulk only protocal */

    rndis = rt_malloc(sizeof(struct urndis));
    RT_ASSERT(rndis != RT_NULL);

    /* initilize the data structure */
    rt_memset(rndis, 0, sizeof(struct urndis));
    rndis->rndis_recv = RT_NULL;
    rndis->rndis_request_id = 1;
    intf[0]->user_data = (void *)rndis;
    /* get endpoint descriptor from interface descriptor */
    rt_usbh_get_endpoint_descriptor(intf[0]->intf_desc, i, &ep_desc);
    if (ep_desc == RT_NULL)
    {
        rt_kprintf("rt_usb_get_endpoint_descriptor error\n");
        return -RT_ERROR;
    }

    /* the endpoint type of mass storage class should be BULK */
    if ((ep_desc->bmAttributes & USB_EP_ATTR_TYPE_MASK) != USB_EP_ATTR_INT)
        return -RT_ERROR;

    /* allocate pipes according to the endpoint type */
    if (ep_desc->bEndpointAddress & USB_DIR_IN)
    {
        /* alloc an in pipe for the storage instance */
        rndis->pipe_int = rt_usb_instance_find_pipe(intf[0]->device, ep_desc->bEndpointAddress);
    }


    intf[1]->user_data = RT_NULL;
    ep_desc = RT_NULL;

    for (i = 0; i < intf[1]->intf_desc->bNumEndpoints; i++)
    {

        /* get endpoint descriptor from interface descriptor */
        rt_usbh_get_endpoint_descriptor(intf[1]->intf_desc, i, &ep_desc);
        if (ep_desc == RT_NULL)
        {
            rt_kprintf("rt_usb_get_endpoint_descriptor error\n");
            return -RT_ERROR;
        }

        /* the endpoint type of mass storage class should be BULK */
        if ((ep_desc->bmAttributes & USB_EP_ATTR_TYPE_MASK) != USB_EP_ATTR_BULK)
            continue;

        /* allocate pipes according to the endpoint type */
        if (ep_desc->bEndpointAddress & USB_DIR_IN)
        {
            /* alloc an in pipe for the storage instance */
            rndis->pipe_in = rt_usb_instance_find_pipe(intf[1]->device, ep_desc->bEndpointAddress);

        }
        else
        {
            /* alloc an output pipe for the storage instance */
            rndis->pipe_out = rt_usb_instance_find_pipe(intf[1]->device, ep_desc->bEndpointAddress);
        }
    }

    /* check pipes infomation */
    if (rndis->pipe_in == RT_NULL || rndis->pipe_out == RT_NULL ||
            rndis->pipe_int == RT_NULL)
    {
        rt_kprintf("pipe error, unsupported device\n");
        return -RT_ERROR;
    }

    /* should implement as callback */
    return rt_rndis_run(intf[0]);


}

/**
 * This function will be invoked when usb device plug out is detected and it would clean
 * and release rndis class related resources.
 *
 * @param arg the argument.
 *
 * @return the error code, RT_EOK on successfully.
 */
static rt_err_t rt_usbh_rndis_disable(void *arg)
{
    urndis_t rndis = RT_NULL;
    struct uhintf **intf = (struct uhintf **)arg;

    /* parameter check */
    RT_ASSERT(intf != RT_NULL);
    RT_ASSERT(intf[0]->user_data != RT_NULL);
    RT_ASSERT(intf[0]->device != RT_NULL);

    RT_DEBUG_LOG(RT_DEBUG_USB, ("rt_usbh_rndis_stop\n"));

    /* get storage instance from interface instance */
    rndis = (urndis_t)intf[0]->user_data;

    rt_rndis_stop(intf[0]);


    /* free storage instance */
    if (rndis != RT_NULL)
    {
        rt_free(rndis);
        intf[0]->user_data = RT_NULL;
    }
    return RT_EOK;
}

/**
 * This function will register mrndis class driver to the usb class driver manager.
 * and it should be invoked in the usb system initialization.
 *
 * @return the error code, RT_EOK on successfully.
 */
ucd_t rt_usbh_class_driver_rndis(void)
{
    rndis_driver.class_code = USB_CLASS_WIRELESS;

    rndis_driver.enable = rt_usbh_rndis_enable;
    rndis_driver.disable = rt_usbh_rndis_disable;

    return &rndis_driver;
}

#endif

