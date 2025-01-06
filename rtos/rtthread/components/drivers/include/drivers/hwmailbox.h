/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */
#ifndef __HWMAILBOX_H__
#define __HWMAILBOX_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct rt_hwmailbox_device
{
    struct rt_device parent;
    const struct rt_hwmailbox_ops *ops;
    struct rt_ringbuffer *ring_buffer;
    /** buffer address from the view of sender */
    uint32_t tx_buffer_addr;
    /** buffer address from the view of receiver */
    uint32_t rx_buffer_addr;
    uint32_t buffer_size;
    rt_device_t owner;
    rt_size_t data_len;

} rt_hwmailbox_t;

typedef struct rt_bidir_hwmailbox_device
{
    struct rt_device parent;
    rt_hwmailbox_t *rx_device;
    rt_hwmailbox_t *tx_device;
} rt_bidir_hwmailbox_t;


struct rt_hwmailbox_ops
{
    void (*init)(struct rt_hwmailbox_device *mailbox);
    void (*trigger)(struct rt_hwmailbox_device *mailbox);
    rt_err_t (*control)(struct rt_hwmailbox_device *mailbox, rt_uint32_t cmd, void *args);
    void (*lock)(struct rt_hwmailbox_device *mailbox, uint8_t lock);
};

rt_err_t rt_device_hwmailbox_register(rt_hwmailbox_t *mailbox, const char *name, rt_uint32_t flag, void *user_data);
void rt_device_hwmailbox_isr(rt_hwmailbox_t *mailbox);
rt_err_t rt_device_bidir_hwmailbox_register(rt_bidir_hwmailbox_t *mailbox, const char *name, rt_uint32_t flag, void *user_data);

#ifdef __cplusplus
}
#endif

#endif
