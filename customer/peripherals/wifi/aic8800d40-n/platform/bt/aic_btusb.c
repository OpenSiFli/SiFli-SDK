/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * AIC8800D80 USB HCI bring-up for RT-Thread.
 *
 * Linux reference: drivers/aic_btusb (class e0/01/01 on PID 0x8d81).
 * Phase-2: LE advertise + minimal ATT so phones can complete a connect.
 */

#include <rtthread.h>
#include <usb.h>
#include <string.h>
#include <stdio.h>
#include "aic_btusb.h"
#include "aic_bt_usb.h"
#include "aic_log.h"

#ifndef CONFIG_USB_SUPPORT
int aic_btusb_drv_register(void)
{
    return -1;
}
void aic_btusb_drv_unregister(void) {}
int aic_btusb_try_bind(struct usb_interface *intf)
{
    (void)intf;
    return -1;
}
int aic_btusb_ready(void)
{
    return 0;
}
int aic_bt_hci_cmd(const uint8_t *c, uint16_t n, uint8_t *r, uint16_t m,
                   uint16_t *l, uint32_t t)
{
    (void)c;
    (void)n;
    (void)r;
    (void)m;
    (void)l;
    (void)t;
    return -1;
}
int aic_ble_adv_start(const char *n)
{
    (void)n;
    return -1;
}
int aic_ble_adv_stop(void)
{
    return -1;
}
int aic_ble_test_get(char *b, int n)
{
    (void)b;
    (void)n;
    return -1;
}
int aic_ble_test_set(const char *d, int n)
{
    (void)d;
    (void)n;
    return -1;
}
int aic_bt_usb_start(void)
{
    return -1;
}
int aic_bt_usb_send(const uint8_t *h4, uint32_t len)
{
    (void)h4;
    (void)len;
    return -1;
}
void aic_bt_usb_set_rx_cb(aic_hci_tr_rx_cb_t cb)
{
    (void)cb;
}
int aic_bt_usb_ready(void)
{
    return 0;
}
#else

#define AIC_VENDOR_ID              0xa69c
#define AIC_PRODUCT_ID_D80_WIFI_BT 0x8d81
#define AIC_PRODUCT_ID_D80N_WIFI_BT 0x8d45   /* AIC8800D80N (Linux USB PID) */

#define HCI_OP_SET_EVENT_MASK      0x0c01
#define HCI_OP_RESET               0x0c03
#define HCI_OP_LE_SET_EVENT_MASK   0x2001
#define HCI_OP_LE_SET_ADV_PARAM    0x2006
#define HCI_OP_LE_SET_ADV_DATA     0x2008
#define HCI_OP_LE_SET_SCAN_RSP     0x2009
#define HCI_OP_LE_SET_ADV_ENABLE   0x200a
#define HCI_OP_LE_SET_RAND_ADDR    0x2005

#define BTUSB_EVT_BUF_SIZE         256
#define BTUSB_INTR_SIZE            64
#define BTUSB_ACL_SIZE             512
#define L2CAP_CID_ATT              0x0004
#define BT_WORK_MQ_DEPTH           8
#define BT_WORK_ACL_MAX            256

#define BT_WORK_ACL                1
#define BT_WORK_CONNECTED          2
#define BT_WORK_DISCONNECTED       3
#define BT_WORK_H4                 4

struct bt_work_msg
{
    uint8_t type;
    uint8_t pad;
    uint16_t handle;
    uint16_t len;
    uint8_t data[BT_WORK_ACL_MAX];
};

struct aic_btusb
{
    struct usb_device *udev;
    struct usb_interface *intf;
    struct usb_endpoint_descriptor *intr_ep;
    struct usb_endpoint_descriptor *bulk_tx_ep;
    struct usb_endpoint_descriptor *bulk_rx_ep;
    struct urb *intr_urb;
    struct urb *bulk_rx_urb;
    uint8_t *intr_buf;
    uint8_t *bulk_rx_buf;
    struct rt_semaphore evt_sem;
    struct rt_mutex lock;
    uint8_t evt_buf[BTUSB_EVT_BUF_SIZE];
    uint16_t evt_len;
    uint16_t wait_opcode;
    volatile int ready;
    volatile int evt_pending;
    volatile uint16_t conn_handle; /* 0xffff = none */
    uint16_t att_mtu;
    char adv_name[28];
    uint8_t test_val[20];
    uint8_t test_len;
    struct rt_messagequeue work_mq;
    struct rt_thread work_thread;
    rt_uint8_t work_stack[2048];
    struct bt_work_msg work_mq_pool[BT_WORK_MQ_DEPTH];
    int work_started;
};

static struct aic_btusb g_bt;
static int g_drv_reg;
static aic_hci_tr_rx_cb_t s_usb_hci_rx_cb;

static uint16_t hci_opcode(const uint8_t *cmd)
{
    return (uint16_t)cmd[0] | ((uint16_t)cmd[1] << 8);
}

static int btusb_submit_intr(struct aic_btusb *bt);
static int btusb_submit_bulk_rx(struct aic_btusb *bt);
static void btusb_handle_att(struct aic_btusb *bt, const uint8_t *att, uint16_t alen);

/* ISR-safe: queue work for btusb thread (no blocking USB I/O in ISR) */
static void btusb_work_post(struct aic_btusb *bt, uint8_t type, uint16_t handle,
                            const uint8_t *data, uint16_t len)
{
    struct bt_work_msg msg;

    if (!bt->work_started)
        return;
    memset(&msg, 0, sizeof(msg));
    msg.type = type;
    msg.handle = handle;
    if (data && len)
    {
        if (len > BT_WORK_ACL_MAX)
            len = BT_WORK_ACL_MAX;
        memcpy(msg.data, data, len);
        msg.len = len;
    }
    if (rt_mq_send(&bt->work_mq, &msg, sizeof(msg)) != RT_EOK)
    {
        /* drop if full; never block in ISR */
    }
}

static void btusb_h4_post(struct aic_btusb *bt, uint8_t h4_type,
                          const uint8_t *data, uint16_t len)
{
    uint8_t h4[BT_WORK_ACL_MAX];
    uint16_t n;

    if (s_usb_hci_rx_cb == NULL || data == NULL || len == 0)
        return;
    n = len;
    if ((uint32_t)n + 1u > BT_WORK_ACL_MAX)
        n = (uint16_t)(BT_WORK_ACL_MAX - 1u);
    h4[0] = h4_type;
    memcpy(&h4[1], data, n);
    btusb_work_post(bt, BT_WORK_H4, 0, h4, (uint16_t)(n + 1u));
}

static int btusb_acl_send(struct aic_btusb *bt, uint16_t handle,
                          const uint8_t *l2cap_payload, uint16_t plen)
{
    uint8_t *buf;
    unsigned int pipe;
    int ret, actual = 0;
    uint16_t acl_len;

    if (!bt->ready || !bt->bulk_tx_ep || handle == 0xffff)
        return -1;
    /* HCI ACL: handle(2) + dlen(2) + L2CAP(len+cid+payload) */
    acl_len = (uint16_t)(4 + plen);
    buf = (uint8_t *)usb_dma_buffer_alloc(acl_len < 64 ? 64 : 128, bt->udev->bus);
    if (!buf)
        return -ENOMEM;

    buf[0] = (uint8_t)(handle & 0xff);
    buf[1] = (uint8_t)((handle >> 8) & 0x0f); /* PB/BC = 0 (start) */
    buf[2] = (uint8_t)((plen + 4) & 0xff);    /* ACL data len = L2CAP hdr+payload */
    buf[3] = (uint8_t)(((plen + 4) >> 8) & 0xff);
    /* L2CAP header */
    buf[4] = (uint8_t)(plen & 0xff);
    buf[5] = (uint8_t)((plen >> 8) & 0xff);
    buf[6] = (uint8_t)(L2CAP_CID_ATT & 0xff);
    buf[7] = (uint8_t)((L2CAP_CID_ATT >> 8) & 0xff);
    if (plen && l2cap_payload)
        memcpy(buf + 8, l2cap_payload, plen);

    pipe = usb_sndbulkpipe(bt->udev, bt->bulk_tx_ep->bEndpointAddress & 0x0f);
    ret = usb_bulk_msg(bt->udev, pipe, buf, (int)(8 + plen), &actual, 2000);
    usb_dma_buffer_free(buf, bt->udev->bus);
    if (ret < 0)
    {
        rt_kprintf("[btusb] ACL TX fail %d\n", ret);
        return ret;
    }
    return 0;
}

static void btusb_att_error(struct aic_btusb *bt, uint8_t req_op,
                            uint16_t handle, uint8_t err)
{
    uint8_t rsp[5];
    rsp[0] = 0x01; /* Error Response */
    rsp[1] = req_op;
    rsp[2] = (uint8_t)(handle & 0xff);
    rsp[3] = (uint8_t)(handle >> 8);
    rsp[4] = err;
    btusb_acl_send(bt, bt->conn_handle, rsp, 5);
}

static void btusb_handle_att(struct aic_btusb *bt, const uint8_t *att, uint16_t alen)
{
    uint8_t op;
    uint8_t rsp[64];
    uint16_t nlen;
    uint16_t start, end, uuid, h;
    uint16_t vlen;
    int i;

    if (!att || alen < 1)
        return;
    op = att[0];
    nlen = (uint16_t)strlen(bt->adv_name);
    if (nlen > 20)
        nlen = 20;

    /*
     * GATT DB:
     *  0x0001 PrimSvc GAP(0x1800)          group ..0x0003
     *  0x0002 CharDecl DeviceName(0x2A00)  Read     val@0x0003
     *  0x0003 DeviceName value
     *  0x0004 PrimSvc Test(0xFFF0)         group ..0x0006
     *  0x0005 CharDecl TestData(0xFFF1)    R|W|WnR  val@0x0006
     *  0x0006 TestData value (RW buffer)
     */
    switch (op)
    {
    case 0x02: /* Exchange MTU Request */
        if (alen < 3)
            return;
        {
            uint16_t mtu = (uint16_t)att[1] | ((uint16_t)att[2] << 8);
            if (mtu < 23)
                mtu = 23;
            if (mtu > 247)
                mtu = 247;
            bt->att_mtu = mtu;
            rsp[0] = 0x03;
            rsp[1] = 23;
            rsp[2] = 0;
            btusb_acl_send(bt, bt->conn_handle, rsp, 3);
            rt_kprintf("[btusb] ATT MTU peer=%u\n", mtu);
        }
        break;

    case 0x10: /* Read By Group Type Request */
        if (alen < 7)
            return;
        start = (uint16_t)att[1] | ((uint16_t)att[2] << 8);
        end   = (uint16_t)att[3] | ((uint16_t)att[4] << 8);
        uuid  = (uint16_t)att[5] | ((uint16_t)att[6] << 8);
        if (alen == 7 && uuid == 0x2800)
        {
            if (start <= 0x0001 && end >= 0x0001)
            {
                rsp[0] = 0x11;
                rsp[1] = 6;
                rsp[2] = 0x01;
                rsp[3] = 0x00;
                rsp[4] = 0x03;
                rsp[5] = 0x00;
                rsp[6] = 0x00;
                rsp[7] = 0x18;
                btusb_acl_send(bt, bt->conn_handle, rsp, 8);
                rt_kprintf("[btusb] ATT PrimSvc GAP\n");
            }
            else if (start <= 0x0004 && end >= 0x0004)
            {
                rsp[0] = 0x11;
                rsp[1] = 6;
                rsp[2] = 0x04;
                rsp[3] = 0x00;
                rsp[4] = 0x06;
                rsp[5] = 0x00;
                rsp[6] = 0xf0;
                rsp[7] = 0xff; /* 0xFFF0 Test */
                btusb_acl_send(bt, bt->conn_handle, rsp, 8);
                rt_kprintf("[btusb] ATT PrimSvc Test(FFF0)\n");
            }
            else
            {
                btusb_att_error(bt, op, start, 0x0a);
            }
        }
        else
        {
            btusb_att_error(bt, op, start, 0x0a);
        }
        break;

    case 0x08: /* Read By Type Request */
        if (alen < 7)
            return;
        start = (uint16_t)att[1] | ((uint16_t)att[2] << 8);
        end   = (uint16_t)att[3] | ((uint16_t)att[4] << 8);
        uuid  = (uint16_t)att[5] | ((uint16_t)att[6] << 8);
        if (uuid == 0x2803)   /* Characteristic Declaration */
        {
            if (start <= 0x0002 && end >= 0x0002)
            {
                rsp[0] = 0x09;
                rsp[1] = 7;
                rsp[2] = 0x02;
                rsp[3] = 0x00;
                rsp[4] = 0x02; /* Read */
                rsp[5] = 0x03;
                rsp[6] = 0x00;
                rsp[7] = 0x00;
                rsp[8] = 0x2a;
                btusb_acl_send(bt, bt->conn_handle, rsp, 9);
                rt_kprintf("[btusb] ATT CharDecl DeviceName\n");
            }
            else if (start <= 0x0005 && end >= 0x0005)
            {
                rsp[0] = 0x09;
                rsp[1] = 7;
                rsp[2] = 0x05;
                rsp[3] = 0x00;
                rsp[4] = 0x0e; /* Read|Write|WriteCmd */
                rsp[5] = 0x06;
                rsp[6] = 0x00;
                rsp[7] = 0xf1;
                rsp[8] = 0xff; /* 0xFFF1 */
                btusb_acl_send(bt, bt->conn_handle, rsp, 9);
                rt_kprintf("[btusb] ATT CharDecl TestData(FFF1) RW\n");
            }
            else
            {
                btusb_att_error(bt, op, start, 0x0a);
            }
        }
        else if (uuid == 0x2a00)
        {
            if (start <= 0x0003 && end >= 0x0003)
            {
                rsp[0] = 0x09;
                rsp[1] = (uint8_t)(2 + nlen);
                rsp[2] = 0x03;
                rsp[3] = 0x00;
                memcpy(&rsp[4], bt->adv_name, nlen);
                btusb_acl_send(bt, bt->conn_handle, rsp, (uint16_t)(4 + nlen));
            }
            else
            {
                btusb_att_error(bt, op, start, 0x0a);
            }
        }
        else if (uuid == 0xfff1)
        {
            if (start <= 0x0006 && end >= 0x0006)
            {
                rsp[0] = 0x09;
                rsp[1] = (uint8_t)(2 + bt->test_len);
                rsp[2] = 0x06;
                rsp[3] = 0x00;
                memcpy(&rsp[4], bt->test_val, bt->test_len);
                btusb_acl_send(bt, bt->conn_handle, rsp,
                               (uint16_t)(4 + bt->test_len));
            }
            else
            {
                btusb_att_error(bt, op, start, 0x0a);
            }
        }
        else
        {
            btusb_att_error(bt, op, start, 0x0a);
        }
        break;

    case 0x0a: /* Read Request */
        if (alen < 3)
            return;
        h = (uint16_t)att[1] | ((uint16_t)att[2] << 8);
        if (h == 0x0003)
        {
            rsp[0] = 0x0b;
            memcpy(&rsp[1], bt->adv_name, nlen);
            btusb_acl_send(bt, bt->conn_handle, rsp, (uint16_t)(1 + nlen));
            rt_kprintf("[btusb] ATT Read DeviceName\n");
        }
        else if (h == 0x0006)
        {
            rsp[0] = 0x0b;
            memcpy(&rsp[1], bt->test_val, bt->test_len);
            btusb_acl_send(bt, bt->conn_handle, rsp, (uint16_t)(1 + bt->test_len));
            rt_kprintf("[btusb] ATT Read TestData len=%u \"", bt->test_len);
            for (i = 0; i < bt->test_len; i++)
                rt_kprintf("%c", bt->test_val[i] >= 32 && bt->test_val[i] < 127 ?
                           bt->test_val[i] : '.');
            rt_kprintf("\"\n");
        }
        else if (h == 0x0001)
        {
            rsp[0] = 0x0b;
            rsp[1] = 0x00;
            rsp[2] = 0x18;
            btusb_acl_send(bt, bt->conn_handle, rsp, 3);
        }
        else if (h == 0x0004)
        {
            rsp[0] = 0x0b;
            rsp[1] = 0xf0;
            rsp[2] = 0xff;
            btusb_acl_send(bt, bt->conn_handle, rsp, 3);
        }
        else if (h == 0x0002)
        {
            rsp[0] = 0x0b;
            rsp[1] = 0x02;
            rsp[2] = 0x03;
            rsp[3] = 0x00;
            rsp[4] = 0x00;
            rsp[5] = 0x2a;
            btusb_acl_send(bt, bt->conn_handle, rsp, 6);
        }
        else if (h == 0x0005)
        {
            rsp[0] = 0x0b;
            rsp[1] = 0x0e;
            rsp[2] = 0x06;
            rsp[3] = 0x00;
            rsp[4] = 0xf1;
            rsp[5] = 0xff;
            btusb_acl_send(bt, bt->conn_handle, rsp, 6);
        }
        else
        {
            btusb_att_error(bt, op, h, 0x0a);
        }
        break;

    case 0x12: /* Write Request */
    case 0x52: /* Write Command (no response) */
        if (alen < 3)
            return;
        h = (uint16_t)att[1] | ((uint16_t)att[2] << 8);
        vlen = (uint16_t)(alen - 3);
        if (h == 0x0006)
        {
            if (vlen > sizeof(bt->test_val))
                vlen = sizeof(bt->test_val);
            memcpy(bt->test_val, att + 3, vlen);
            bt->test_len = (uint8_t)vlen;
            rt_kprintf("[btusb] ATT Write TestData len=%u \"", bt->test_len);
            for (i = 0; i < bt->test_len; i++)
                rt_kprintf("%c", bt->test_val[i] >= 32 && bt->test_val[i] < 127 ?
                           bt->test_val[i] : '.');
            rt_kprintf("\"\n");
            if (op == 0x12)
            {
                rsp[0] = 0x13; /* Write Response */
                btusb_acl_send(bt, bt->conn_handle, rsp, 1);
            }
        }
        else
        {
            btusb_att_error(bt, op, h, 0x03); /* Write Not Permitted */
        }
        break;

    case 0x04: /* Find Information Request */
        if (alen < 5)
            return;
        start = (uint16_t)att[1] | ((uint16_t)att[2] << 8);
        end   = (uint16_t)att[3] | ((uint16_t)att[4] << 8);
        if (start <= 0x0001 && end >= 0x0001)
        {
            rsp[0] = 0x05;
            rsp[1] = 0x01;
            rsp[2] = 0x01;
            rsp[3] = 0x00;
            rsp[4] = 0x00;
            rsp[5] = 0x28;
            btusb_acl_send(bt, bt->conn_handle, rsp, 6);
        }
        else if (start <= 0x0002 && end >= 0x0002)
        {
            rsp[0] = 0x05;
            rsp[1] = 0x01;
            rsp[2] = 0x02;
            rsp[3] = 0x00;
            rsp[4] = 0x03;
            rsp[5] = 0x28;
            btusb_acl_send(bt, bt->conn_handle, rsp, 6);
        }
        else if (start <= 0x0003 && end >= 0x0003)
        {
            rsp[0] = 0x05;
            rsp[1] = 0x01;
            rsp[2] = 0x03;
            rsp[3] = 0x00;
            rsp[4] = 0x00;
            rsp[5] = 0x2a;
            btusb_acl_send(bt, bt->conn_handle, rsp, 6);
        }
        else if (start <= 0x0004 && end >= 0x0004)
        {
            rsp[0] = 0x05;
            rsp[1] = 0x01;
            rsp[2] = 0x04;
            rsp[3] = 0x00;
            rsp[4] = 0x00;
            rsp[5] = 0x28;
            btusb_acl_send(bt, bt->conn_handle, rsp, 6);
        }
        else if (start <= 0x0005 && end >= 0x0005)
        {
            rsp[0] = 0x05;
            rsp[1] = 0x01;
            rsp[2] = 0x05;
            rsp[3] = 0x00;
            rsp[4] = 0x03;
            rsp[5] = 0x28;
            btusb_acl_send(bt, bt->conn_handle, rsp, 6);
        }
        else if (start <= 0x0006 && end >= 0x0006)
        {
            rsp[0] = 0x05;
            rsp[1] = 0x01;
            rsp[2] = 0x06;
            rsp[3] = 0x00;
            rsp[4] = 0xf1;
            rsp[5] = 0xff;
            btusb_acl_send(bt, bt->conn_handle, rsp, 6);
        }
        else
        {
            btusb_att_error(bt, op, start, 0x0a);
        }
        break;

    default:
        rt_kprintf("[btusb] ATT op=0x%02x len=%u -> not supported\n", op, alen);
        btusb_att_error(bt, op, 0x0001, 0x06);
        break;
    }
}

static void btusb_handle_acl(struct aic_btusb *bt, uint8_t *data, uint16_t len)
{
    uint16_t l2_len, cid;

    if (len < 8)
        return;
    l2_len = (uint16_t)(data[4] | (data[5] << 8));
    cid = (uint16_t)(data[6] | (data[7] << 8));
    if (cid == L2CAP_CID_ATT && l2_len >= 1 && (8 + l2_len) <= len)
        btusb_handle_att(bt, data + 8, l2_len);
    else
        rt_kprintf("[btusb] ACL cid=0x%04x l2=%u\n", cid, l2_len);
}

static void btusb_work_entry(void *arg)
{
    struct aic_btusb *bt = (struct aic_btusb *)arg;
    struct bt_work_msg msg;

    while (1)
    {
        if (rt_mq_recv(&bt->work_mq, &msg, sizeof(msg), RT_WAITING_FOREVER) != RT_EOK)
            continue;
        switch (msg.type)
        {
        case BT_WORK_CONNECTED:
            rt_kprintf("[btusb] LE connected handle=0x%04x (worker)\n", msg.handle);
            (void)btusb_submit_bulk_rx(bt);
            break;
        case BT_WORK_DISCONNECTED:
            rt_kprintf("[btusb] HCI disconnect handle=0x%04x reason=0x%02x\n",
                       msg.handle, msg.data[0]);
            break;
        case BT_WORK_ACL:
            btusb_handle_acl(bt, msg.data, msg.len);
            break;
        case BT_WORK_H4:
            if (s_usb_hci_rx_cb != NULL && msg.len > 0)
                s_usb_hci_rx_cb(msg.data, msg.len);
            break;
        default:
            break;
        }
    }
}

static int btusb_work_start(struct aic_btusb *bt)
{
    if (bt->work_started)
        return 0;
    if (rt_mq_init(&bt->work_mq, "bt_work",
                   bt->work_mq_pool,
                   sizeof(struct bt_work_msg),
                   sizeof(bt->work_mq_pool),
                   RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("[btusb] work mq init fail\n");
        return -1;
    }
    if (rt_thread_init(&bt->work_thread, "bt_work",
                       btusb_work_entry, bt,
                       bt->work_stack, sizeof(bt->work_stack),
                       12, 20) != RT_EOK)
    {
        rt_kprintf("[btusb] work thread init fail\n");
        return -1;
    }
    bt->work_started = 1;
    rt_thread_startup(&bt->work_thread);
    return 0;
}

static void btusb_handle_le_meta(struct aic_btusb *bt, const uint8_t *data, uint16_t len)
{
    uint8_t sub;
    uint8_t status;
    uint16_t handle;

    if (len < 3)
        return;
    sub = data[2];
    if (sub == 0x01 && len >= 21)
    {
        status = data[3];
        handle = (uint16_t)(data[4] | (data[5] << 8));
        if (status == 0)
        {
            bt->conn_handle = handle;
            bt->att_mtu = 23;
            btusb_work_post(bt, BT_WORK_CONNECTED, handle, NULL, 0);
        }
    }
    else if (sub == 0x0a && len >= 13)
    {
        status = data[3];
        handle = (uint16_t)(data[4] | (data[5] << 8));
        if (status == 0)
        {
            bt->conn_handle = handle;
            bt->att_mtu = 23;
            btusb_work_post(bt, BT_WORK_CONNECTED, handle, NULL, 0);
        }
    }
    else if (sub == 0x14)
    {
        /* Channel Selection Algorithm — ignore */
    }
    /* other LE meta: ignore in ISR */
}

static void btusb_bulk_rx_complete(struct urb *urb)
{
    struct aic_btusb *bt = (struct aic_btusb *)urb->context;

    if (!bt)
        return;
    if (urb->status == 0 && urb->actual_length >= 4)
    {
        if (s_usb_hci_rx_cb != NULL)
            btusb_h4_post(bt, 0x02, urb->transfer_buffer,
                          (uint16_t)urb->actual_length);
        else
            btusb_work_post(bt, BT_WORK_ACL, bt->conn_handle,
                            urb->transfer_buffer, (uint16_t)urb->actual_length);
    }

    if (bt->ready)
        (void)btusb_submit_bulk_rx(bt);
}

static int btusb_submit_bulk_rx(struct aic_btusb *bt)
{
    int pipe;

    if (!bt->bulk_rx_urb || !bt->bulk_rx_buf || !bt->bulk_rx_ep)
        return -1;
    pipe = usb_rcvbulkpipe(bt->udev, bt->bulk_rx_ep->bEndpointAddress & 0x0f);
    usb_fill_bulk_urb(bt->bulk_rx_urb, bt->udev, pipe,
                      bt->bulk_rx_buf, BTUSB_ACL_SIZE,
                      btusb_bulk_rx_complete, bt);
    return usb_submit_urb(bt->bulk_rx_urb, 0);
}

static void btusb_intr_complete(struct urb *urb)
{
    struct aic_btusb *bt = (struct aic_btusb *)urb->context;
    uint8_t *data;
    uint16_t len;
    uint16_t opcode;
    int pipe;

    if (!bt)
        return;

    if (urb->status == 0 && urb->actual_length >= 2)
    {
        data = urb->transfer_buffer;
        len = (uint16_t)urb->actual_length;
        if (s_usb_hci_rx_cb != NULL)
        {
            btusb_h4_post(bt, 0x04, data, len);
        }
        else if (data[0] == 0x0e && len >= 6)
        {
            opcode = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
            if (bt->wait_opcode == 0 || bt->wait_opcode == opcode)
            {
                if (len > BTUSB_EVT_BUF_SIZE)
                    len = BTUSB_EVT_BUF_SIZE;
                memcpy(bt->evt_buf, data, len);
                bt->evt_len = len;
                bt->evt_pending = 1;
                rt_sem_release(&bt->evt_sem);
            }
        }
        else if (data[0] == 0x0f && len >= 5)
        {
            opcode = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
            if (bt->wait_opcode == 0 || bt->wait_opcode == opcode)
            {
                if (len > BTUSB_EVT_BUF_SIZE)
                    len = BTUSB_EVT_BUF_SIZE;
                memcpy(bt->evt_buf, data, len);
                bt->evt_len = len;
                bt->evt_pending = 1;
                rt_sem_release(&bt->evt_sem);
            }
        }
        else if (data[0] == 0x05 && len >= 6)
        {
            /* Disconnection Complete */
            uint16_t h = (uint16_t)(data[3] | (data[4] << 8));
            uint8_t reason = data[5];
            if (h == bt->conn_handle)
                bt->conn_handle = 0xffff;
            btusb_work_post(bt, BT_WORK_DISCONNECTED, h, &reason, 1);
        }
        else if (data[0] == 0x3e)
        {
            btusb_handle_le_meta(bt, data, len);
        }
        /* other events ignored in ISR */
    }

    if (!bt->ready || !bt->intr_ep)
        return;

    pipe = usb_rcvintpipe(bt->udev, bt->intr_ep->bEndpointAddress & 0x0f);
    usb_fill_int_urb(bt->intr_urb, bt->udev, pipe,
                     bt->intr_buf, BTUSB_INTR_SIZE,
                     btusb_intr_complete, bt,
                     bt->intr_ep->bInterval ? bt->intr_ep->bInterval : 1);
    if (usb_submit_urb(bt->intr_urb, 0) != 0)
        DBG_USB_ERR("btusb: resubmit intr failed\n");
}

static int btusb_submit_intr(struct aic_btusb *bt)
{
    int pipe;

    if (!bt->intr_urb || !bt->intr_buf || !bt->intr_ep)
        return -1;
    pipe = usb_rcvintpipe(bt->udev, bt->intr_ep->bEndpointAddress & 0x0f);
    usb_fill_int_urb(bt->intr_urb, bt->udev, pipe,
                     bt->intr_buf, BTUSB_INTR_SIZE,
                     btusb_intr_complete, bt,
                     bt->intr_ep->bInterval ? bt->intr_ep->bInterval : 1);
    return usb_submit_urb(bt->intr_urb, 0);
}

int aic_btusb_ready(void)
{
    return g_bt.ready;
}

void
aic_bt_usb_set_rx_cb(aic_hci_tr_rx_cb_t cb)
{
    s_usb_hci_rx_cb = cb;
}

int
aic_bt_usb_ready(void)
{
    return aic_btusb_ready();
}

int
aic_bt_usb_start(void)
{
    return aic_btusb_ready() ? 0 : -1;
}

int
aic_bt_usb_send(const uint8_t *h4, uint32_t len)
{
    struct aic_btusb *bt = &g_bt;
    const uint8_t *payload;
    uint32_t plen;
    uint8_t *tx;
    unsigned int pipe;
    int ret, actual = 0;

    if (h4 == NULL || len < 1u || !bt->ready || !bt->udev)
        return -1;
    payload = h4 + 1;
    plen = len - 1u;
    if (h4[0] == 0x01u)
    {
        if (plen < 3u)
            return -1;
        tx = (uint8_t *)usb_dma_buffer_alloc(64, bt->udev->bus);
        if (!tx)
            return -1;
        if (plen > 64u)
            plen = 64u;
        memcpy(tx, payload, plen);
        ret = usb_control_msg(bt->udev, usb_sndctrlpipe(bt->udev, 0),
                              0, USB_TYPE_CLASS,
                              0, 0, tx, (int)plen, 2000);
        usb_dma_buffer_free(tx, bt->udev->bus);
        return (ret < 0) ? -1 : 0;
    }
    if (h4[0] == 0x02u)
    {
        if (!bt->bulk_tx_ep || plen == 0u)
            return -1;
        tx = (uint8_t *)usb_dma_buffer_alloc(plen < 64u ? 64u : plen,
                                             bt->udev->bus);
        if (!tx)
            return -1;
        memcpy(tx, payload, plen);
        pipe = usb_sndbulkpipe(bt->udev, bt->bulk_tx_ep->bEndpointAddress & 0x0f);
        ret = usb_bulk_msg(bt->udev, pipe, tx, (int)plen, &actual, 2000);
        usb_dma_buffer_free(tx, bt->udev->bus);
        return (ret < 0) ? -1 : 0;
    }
    return -1;
}

static int btusb_evt_matches(const uint8_t *evt, int len, uint16_t opcode)
{
    uint16_t op;

    if (len < 5)
        return 0;
    if (evt[0] == 0x0e && len >= 6)
    {
        op = (uint16_t)evt[3] | ((uint16_t)evt[4] << 8);
        return op == opcode;
    }
    if (evt[0] == 0x0f)
    {
        op = (uint16_t)evt[3] | ((uint16_t)evt[4] << 8);
        return op == opcode;
    }
    return 0;
}

int aic_bt_hci_cmd(const uint8_t *cmd, uint16_t cmd_len,
                   uint8_t *rsp, uint16_t rsp_max, uint16_t *rsp_len,
                   uint32_t timeout_ms)
{
    struct aic_btusb *bt = &g_bt;
    uint8_t *tx = RT_NULL;
    int ret;
    uint16_t opcode;

    if (!bt->ready || !bt->udev || !cmd || cmd_len < 3)
        return -1;

    opcode = hci_opcode(cmd);
    tx = (uint8_t *)usb_dma_buffer_alloc(64, bt->udev->bus);
    if (!tx)
        return -ENOMEM;
    if (cmd_len > 64)
        cmd_len = 64;
    memcpy(tx, cmd, cmd_len);

    rt_mutex_take(&bt->lock, RT_WAITING_FOREVER);
    bt->wait_opcode = opcode;
    bt->evt_pending = 0;
    bt->evt_len = 0;
    while (rt_sem_trytake(&bt->evt_sem) == RT_EOK)
        ;

    ret = usb_control_msg(bt->udev, usb_sndctrlpipe(bt->udev, 0),
                          0, USB_TYPE_CLASS,
                          0, 0, tx, cmd_len, 2000);
    if (ret < 0)
    {
        rt_kprintf("[btusb] HCI CMD 0x%04x ctrl fail %d\n", opcode, ret);
        bt->wait_opcode = 0;
        rt_mutex_release(&bt->lock);
        usb_dma_buffer_free(tx, bt->udev->bus);
        return ret;
    }
    rt_kprintf("[btusb] HCI CMD 0x%04x ctrl ok (%d bytes)\n", opcode, ret);

    ret = rt_sem_take(&bt->evt_sem, rt_tick_from_millisecond(timeout_ms));
    if (ret != RT_EOK)
    {
        uint8_t *rx = (uint8_t *)usb_dma_buffer_alloc(BTUSB_INTR_SIZE, bt->udev->bus);
        unsigned int ipipe;
        int actual = 0;
        uint32_t waited = 0;

        if (bt->intr_urb)
            usb_kill_urb(bt->intr_urb);

        if (rx && bt->intr_ep)
        {
            ipipe = usb_rcvintpipe(bt->udev, bt->intr_ep->bEndpointAddress & 0x0f);
            while (waited < 1000)
            {
                actual = 0;
                ret = usb_interrupt_msg(bt->udev, ipipe, rx, BTUSB_INTR_SIZE,
                                        &actual, 500);
                waited += 500;
                if (ret < 0)
                    continue;
                if (btusb_evt_matches(rx, actual, opcode))
                {
                    if (actual > BTUSB_EVT_BUF_SIZE)
                        actual = BTUSB_EVT_BUF_SIZE;
                    memcpy(bt->evt_buf, rx, actual);
                    bt->evt_len = (uint16_t)actual;
                    break;
                }
            }
            usb_dma_buffer_free(rx, bt->udev->bus);
        }
        else if (rx)
        {
            usb_dma_buffer_free(rx, bt->udev->bus);
        }
        (void)btusb_submit_intr(bt);

        if (bt->evt_len == 0)
        {
            rt_kprintf("[btusb] HCI CMD 0x%04x timeout\n", opcode);
            bt->wait_opcode = 0;
            rt_mutex_release(&bt->lock);
            usb_dma_buffer_free(tx, bt->udev->bus);
            return -ETIMEDOUT;
        }
    }

    if (rsp && rsp_len)
    {
        uint16_t n = bt->evt_len;
        if (n > rsp_max)
            n = rsp_max;
        memcpy(rsp, bt->evt_buf, n);
        *rsp_len = n;
    }

    if (bt->evt_len >= 6 && bt->evt_buf[0] == 0x0e && bt->evt_buf[5] != 0)
    {
        rt_kprintf("[btusb] HCI CMD 0x%04x status=0x%02x\n",
                   opcode, bt->evt_buf[5]);
        bt->wait_opcode = 0;
        rt_mutex_release(&bt->lock);
        usb_dma_buffer_free(tx, bt->udev->bus);
        return -EIO;
    }

    bt->wait_opcode = 0;
    rt_mutex_release(&bt->lock);
    usb_dma_buffer_free(tx, bt->udev->bus);
    return 0;
}

static int hci_cmd_u16(uint16_t opcode, const uint8_t *params, uint8_t plen)
{
    uint8_t buf[64];
    if ((uint16_t)plen + 3 > sizeof(buf))
        return -EINVAL;
    buf[0] = (uint8_t)(opcode & 0xff);
    buf[1] = (uint8_t)(opcode >> 8);
    buf[2] = plen;
    if (plen && params)
        memcpy(buf + 3, params, plen);
    return aic_bt_hci_cmd(buf, (uint16_t)(3 + plen), NULL, 0, NULL, 3000);
}

int aic_ble_adv_stop(void)
{
    uint8_t en = 0;
    if (!aic_btusb_ready())
        return -1;
    return hci_cmd_u16(HCI_OP_LE_SET_ADV_ENABLE, &en, 1);
}

int aic_ble_adv_start(const char *name)
{
    struct aic_btusb *bt = &g_bt;
    uint8_t param[32];
    uint8_t adv[32];
    uint8_t mask[8];
    uint8_t nlen;
    int ret;

    rt_kprintf("[btusb] ble-connect build: adv + mini ATT\n");

    if (!aic_btusb_ready())
    {
        rt_kprintf("[btusb] not ready (wait WiFi FW → PID 8d81 BT if)\n");
        return -1;
    }
    if (!name || !name[0])
        name = "AIC-BLE";
    strncpy(bt->adv_name, name, sizeof(bt->adv_name) - 1);
    bt->adv_name[sizeof(bt->adv_name) - 1] = '\0';
    bt->conn_handle = 0xffff;
    bt->att_mtu = 23;
    /* default RW test payload */
    memcpy(bt->test_val, "hello", 5);
    bt->test_len = 5;

    ret = hci_cmd_u16(HCI_OP_RESET, NULL, 0);
    if (ret)
    {
        rt_kprintf("[btusb] HCI_Reset fail %d\n", ret);
        return ret;
    }
    rt_thread_mdelay(100);

    /* Enable common events incl. Disconnection Complete */
    memset(mask, 0xff, sizeof(mask));
    ret = hci_cmd_u16(HCI_OP_SET_EVENT_MASK, mask, 8);
    if (ret)
        rt_kprintf("[btusb] Set_Event_Mask warn %d\n", ret);

    /* LE meta: conn complete, enh conn, etc. */
    memset(mask, 0xff, sizeof(mask));
    ret = hci_cmd_u16(HCI_OP_LE_SET_EVENT_MASK, mask, 8);
    if (ret)
        rt_kprintf("[btusb] LE_Set_Event_Mask warn %d\n", ret);

    (void)btusb_submit_bulk_rx(bt);

    /* Use static random address to avoid public-addr issues */
    param[0] = 0xC1;
    param[1] = 0x22;
    param[2] = 0x33;
    param[3] = 0x44;
    param[4] = 0x55;
    param[5] = 0x66;
    ret = hci_cmd_u16(HCI_OP_LE_SET_RAND_ADDR, param, 6);
    if (ret)
        rt_kprintf("[btusb] LE_Set_Random_Addr warn %d\n", ret);

    memset(param, 0, sizeof(param));
    param[0] = 0xa0;
    param[1] = 0x00;
    param[2] = 0xa0;
    param[3] = 0x00;
    param[4] = 0x00; /* ADV_IND connectable */
    param[5] = 0x01; /* own addr: random */
    param[13] = 0x07;
    param[14] = 0x00;
    ret = hci_cmd_u16(HCI_OP_LE_SET_ADV_PARAM, param, 15);
    if (ret)
    {
        rt_kprintf("[btusb] LE_Set_Adv_Param fail %d\n", ret);
        return ret;
    }

    nlen = (uint8_t)strlen(bt->adv_name);
    if (nlen > 26)
        nlen = 26;
    memset(adv, 0, sizeof(adv));
    adv[1] = 0x02;
    adv[2] = 0x01;
    adv[3] = 0x06;
    adv[4] = (uint8_t)(nlen + 1);
    adv[5] = 0x09;
    memcpy(&adv[6], bt->adv_name, nlen);
    adv[0] = (uint8_t)(3 + 2 + nlen);
    ret = hci_cmd_u16(HCI_OP_LE_SET_ADV_DATA, adv, 32);
    if (ret)
    {
        rt_kprintf("[btusb] LE_Set_Adv_Data fail %d\n", ret);
        return ret;
    }

    memset(adv, 0, sizeof(adv));
    (void)hci_cmd_u16(HCI_OP_LE_SET_SCAN_RSP, adv, 32);

    param[0] = 0x01;
    ret = hci_cmd_u16(HCI_OP_LE_SET_ADV_ENABLE, param, 1);
    if (ret)
    {
        rt_kprintf("[btusb] LE_Set_Adv_Enable fail %d\n", ret);
        return ret;
    }

    rt_kprintf("[btusb] LE advertising ON name=\"%s\" (GAP+TestFFF0 RW)\n",
               bt->adv_name);
    rt_kprintf("[btusb] Test char UUID=0xFFF1 handle=0x0006 default=\"hello\"\n");
    return 0;
}

int aic_ble_test_get(char *buf, int buflen)
{
    struct aic_btusb *bt = &g_bt;
    int n;

    if (!buf || buflen <= 0)
        return -1;
    n = bt->test_len;
    if (n >= buflen)
        n = buflen - 1;
    memcpy(buf, bt->test_val, n);
    buf[n] = '\0';
    return n;
}

int aic_ble_test_set(const char *data, int len)
{
    struct aic_btusb *bt = &g_bt;

    if (!data || len < 0)
        return -1;
    if (len > (int)sizeof(bt->test_val))
        len = (int)sizeof(bt->test_val);
    memcpy(bt->test_val, data, len);
    bt->test_len = (uint8_t)len;
    rt_kprintf("[btusb] TestData set len=%d\n", len);
    return len;
}

static int aic_btusb_probe(struct usb_interface *intf,
                           const struct usb_device_id *id)
{
    (void)id;
    return aic_btusb_try_bind(intf);
}

int aic_btusb_try_bind(struct usb_interface *intf)
{
    struct usb_host_interface *alt;
    struct usb_endpoint_descriptor *ep;
    struct aic_btusb *bt = &g_bt;
    int i;

    if (!intf || !intf->altsetting)
        return -ENODEV;

    alt = &intf->altsetting[0];
    if (alt->desc.bInterfaceClass != USB_CLASS_WIRELESS_CONTROLLER ||
            alt->desc.bInterfaceSubClass != 0x01 ||
            alt->desc.bInterfaceProtocol != 0x01)
    {
        return -ENODEV;
    }
    if (alt->desc.bInterfaceNumber != 0)
        return 0;

    if (bt->ready)
    {
        DBG_USB_INF("btusb: already claimed\n");
        return 0;
    }

    memset(bt, 0, sizeof(*bt));
    bt->udev = intf->usb_dev;
    bt->intf = intf;
    bt->conn_handle = 0xffff;
    bt->att_mtu = 23;
    rt_sem_init(&bt->evt_sem, "bt_evt", 0, RT_IPC_FLAG_FIFO);
    rt_mutex_init(&bt->lock, "bt_hci", RT_IPC_FLAG_PRIO);

    for (i = 0; i < alt->desc.bNumEndpoints; i++)
    {
        ep = &alt->endpoint[i].desc;
        if (!bt->intr_ep && usb_endpoint_is_int_in(ep))
            bt->intr_ep = ep;
        else if (!bt->bulk_tx_ep && usb_endpoint_is_bulk_out(ep))
            bt->bulk_tx_ep = ep;
        else if (!bt->bulk_rx_ep && usb_endpoint_is_bulk_in(ep))
            bt->bulk_rx_ep = ep;
    }

    if (!bt->intr_ep || !bt->bulk_tx_ep || !bt->bulk_rx_ep)
    {
        DBG_USB_ERR("btusb: missing eps\n");
        return -ENODEV;
    }

    bt->intr_buf = rt_malloc(BTUSB_INTR_SIZE);
    bt->bulk_rx_buf = rt_malloc(BTUSB_ACL_SIZE);
    bt->intr_urb = usb_alloc_urb(0, 0);
    bt->bulk_rx_urb = usb_alloc_urb(0, 0);
    if (!bt->intr_buf || !bt->bulk_rx_buf || !bt->intr_urb || !bt->bulk_rx_urb)
    {
        DBG_USB_ERR("btusb: alloc fail\n");
        return -ENOMEM;
    }

    usb_set_intfdata(intf, bt);
    bt->ready = 1;
    if (btusb_work_start(bt) != 0)
    {
        bt->ready = 0;
        return -EIO;
    }
    if (btusb_submit_intr(bt) != 0)
    {
        DBG_USB_ERR("btusb: submit intr fail\n");
        bt->ready = 0;
        return -EIO;
    }
    (void)btusb_submit_bulk_rx(bt);

    rt_kprintf("[btusb] USB HCI ready if#%u intr=0x%02x bulk_tx=0x%02x bulk_rx=0x%02x\n",
               alt->desc.bInterfaceNumber,
               bt->intr_ep->bEndpointAddress,
               bt->bulk_tx_ep->bEndpointAddress,
               bt->bulk_rx_ep->bEndpointAddress);
    return 0;
}

static void aic_btusb_disconnect(struct usb_interface *intf)
{
    struct aic_btusb *bt = &g_bt;

    (void)intf;
    bt->ready = 0;
    bt->conn_handle = 0xffff;
    if (bt->intr_urb)
        usb_kill_urb(bt->intr_urb);
    if (bt->bulk_rx_urb)
        usb_kill_urb(bt->bulk_rx_urb);
    usb_set_intfdata(intf, RT_NULL);
    if (bt->intr_urb)
    {
        usb_free_urb(bt->intr_urb);
        bt->intr_urb = NULL;
    }
    if (bt->bulk_rx_urb)
    {
        usb_free_urb(bt->bulk_rx_urb);
        bt->bulk_rx_urb = NULL;
    }
    if (bt->intr_buf)
    {
        rt_free(bt->intr_buf);
        bt->intr_buf = NULL;
    }
    if (bt->bulk_rx_buf)
    {
        rt_free(bt->bulk_rx_buf);
        bt->bulk_rx_buf = NULL;
    }
    rt_kprintf("[btusb] disconnected\n");
}

static const struct usb_device_id aic_btusb_ids[] =
{
    {
        USB_DEVICE_AND_INTERFACE_INFO(AIC_VENDOR_ID, AIC_PRODUCT_ID_D80_WIFI_BT,
                                      USB_CLASS_WIRELESS_CONTROLLER, 0x01, 0x01)
    },
    {
        USB_DEVICE_AND_INTERFACE_INFO(AIC_VENDOR_ID, AIC_PRODUCT_ID_D80N_WIFI_BT,
                                      USB_CLASS_WIRELESS_CONTROLLER, 0x01, 0x01)
    },
    { USB_INTERFACE_INFO(USB_CLASS_WIRELESS_CONTROLLER, 0x01, 0x01) },
    { }
};

static struct usb_driver aic_btusb_driver =
{
    .name = "aic_btusb",
    .probe = aic_btusb_probe,
    .disconnect = aic_btusb_disconnect,
    .id_table = aic_btusb_ids,
};

int aic_btusb_drv_register(void)
{
    int ret;
    if (g_drv_reg)
        return 0;
    ret = usb_register_driver(&aic_btusb_driver);
    if (ret)
    {
        DBG_USB_ERR("btusb: register fail %d\n", ret);
        return ret;
    }
    g_drv_reg = 1;
    DBG_USB_INF("btusb: driver registered\n");
    return 0;
}

void aic_btusb_drv_unregister(void)
{
    if (!g_drv_reg)
        return;
    usb_deregister(&aic_btusb_driver);
    g_drv_reg = 0;
}

#endif /* CONFIG_USB_SUPPORT */
