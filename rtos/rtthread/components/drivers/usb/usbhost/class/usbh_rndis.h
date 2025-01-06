/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2021-04-12     Zhao shimin   first version
 */

#ifndef __USBH_RNDIS_H__
#define __USBH_RNDIS_H__

#include <rtthread.h>



struct urndis
{
    upipe_t pipe_in;
    upipe_t pipe_out;
    upipe_t pipe_int;

    struct rt_device *rndis_dev;

    rt_thread_t  rndis_recv;
    rt_timer_t   keepalive_timer;
    rt_uint32_t  rndis_request_id;
    rt_uint32_t  rndis_state;

};
typedef struct urndis *urndis_t;

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
rt_err_t rt_usbh_rndis_control_send(struct uhintf *intf, rt_uint8_t *buffer, int nbytes, int timeout);


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
rt_err_t rt_usbh_rndis_control_recv(struct uhintf *intf, rt_uint8_t *buffer, int nbytes, int timeout);


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
rt_err_t rt_usbh_rndis_interrupt_recv(struct uhintf *intf, rt_uint8_t *buffer, int nbytes, int timeout);


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
rt_err_t rt_usbh_rndis_bulk_recv(struct uhintf *intf, rt_uint8_t *buffer, int nbytes, int timeout);
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
rt_err_t rt_usbh_rndis_bulk_send(struct uhintf *intf, rt_uint8_t *buffer, int nbytes, int timeout);

ucd_t rt_usbh_class_driver_rndis(void);

#endif
