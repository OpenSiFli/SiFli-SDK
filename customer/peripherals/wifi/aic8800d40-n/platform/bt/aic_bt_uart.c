/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BT HCI UART passthrough backend (chosen in aic_hci.c via AIC_HCI_OVER_UART).
 * Two sections in this file:
 *   [AIC8800-PORT] platform section -- UART HAL glue, replaced wholesale on port;
 *   generic transport section -- H4 reassembly state machine, no HAL calls.
 */

#include "aic_bt_uart.h"
#include "rtos_port.h"
#include "aic_log.h"

/* ============================================================
 * [AIC8800-PORT] platform section -- UART HAL glue (replace on port)
 * Depends on platform header uart.h: xUartOpen / vUartInit / iUartRead / iUartWrite
 * ============================================================ */
#include "uart.h"

/** UART port used by BT (matches the chip-side hardware wiring) */
#define BT_UART_PORT_ID     UART_ID1
/** BT UART baud rate (HCI H4 passthrough convention) */
#define BT_UART_BAUDRATE    921600
/** Per read/write blocking limit (ticks) */
#define BT_UART_RW_TIMEOUT  pdMS_TO_TICKS(20)

static UartPort_t *uap = NULL;

/**
 * @brief Open the BT UART (reuse if already open)
 * @return 0 on success, -1 on failure
 */
static int bt_uart_plat_open(void)
{
    if (uap == NULL)
    {
        uap = xUartOpen(BT_UART_PORT_ID);
        if (uap == NULL)
        {
            DBG_BT_ERR("xUartOpen(%d) fail\n", (int)BT_UART_PORT_ID);
            return -1;
        }
        vUartInit(uap, BT_UART_BAUDRATE, 0);
        DBG_BT_INF("uart opened: id=%d baud=%d\n", (int)BT_UART_PORT_ID, BT_UART_BAUDRATE);
    }
    return 0;
}

/**
 * @brief Read BT data from UART (platform adaptation)
 * @param buf receive buffer
 * @param len expected read length
 * @return bytes actually read, 0 = timeout with no data, <0 = failure
 */
static int bt_uart_plat_read(uint8_t *buf, uint32_t len)
{
    int ret = iUartRead(uap, buf, len, BT_UART_RW_TIMEOUT);

    if (ret < 0)
    {
        DBG_BT_ERR("iUartRead failed, ret = %d\n", ret);
    }
    return ret;
}

/**
 * @brief Send BT data over UART (platform adaptation)
 * @param buf transmit buffer
 * @param len transmit length
 * @return bytes actually sent, 0 = timeout with nothing sent, <0 = failure
 */
static int bt_uart_plat_write(uint8_t *buf, uint32_t len)
{
    int ret = iUartWrite(uap, buf, len, BT_UART_RW_TIMEOUT);

    if (ret < 0)
    {
        DBG_BT_ERR("iUartWrite failed, ret = %d\n", ret);
    }
    return ret;
}

/* ============================================================
 * Generic transport section -- H4 frame reassembly, TX and RX (platform independent)
 * ============================================================ */

static rtos_queue bt_driver_server_queue = NULL, bt_server_driver_queue = NULL;
static rtos_task_handle bt_uart_server_driver_task_handle;
/* UART H4 reassembly task: the legacy and hci entry points share one instance;
 * two tasks reading the same UART concurrently split the byte stream and thus
 * corrupt frames */
static rtos_task_handle s_rx_reasm_task;
static aic_bt_uart_rx_cb_t s_hci_rx_cb;
static int s_uart_hci_started;

/**
 * @brief BT UART Server->Driver direction task
 *
 * Reads HCI packets (CMD/ACL/SCO) from the BT Server queue,
 * parses the HCI packet length and sends them to the BT chip over UART.
 *
 * @param args task argument (unused)
 */
static void aicwf_bt_uart_server_driver_task(void *args)
{
    uint8_t server_buffer[BT_SOCKET_BUFFER_SIZE] = {0};
    uint16_t hci_len = 0;
    uint16_t sent_len = 0;
    int ret = 0;

    while (1)
    {
        if (aicwf_rtos_queue_read(bt_server_driver_queue, 0, server_buffer, -1, 0) == pdTRUE)
        {
            DBG_BT_INF("server -> driver: %d\n", server_buffer[0]);
            switch (server_buffer[0])
            {
            case HCI_CMD_PKT:
                hci_len = (uint16_t)server_buffer[3] + 4;
                break;
            case HCI_ACL_PKT:
                hci_len = ((uint16_t)(server_buffer[3] & 0xFF) | (((uint16_t)(server_buffer[4] & 0xFF)) << 8)) + 5;
                break;
            case HCI_SCO_PKT:
                hci_len = (uint16_t)server_buffer[3] + 4;
                break;
            default:
                break;
            }
            sent_len = 0;
            while (sent_len < hci_len)
            {
                ret = bt_uart_plat_write(server_buffer + sent_len, hci_len - sent_len);
                if (ret <= 0)
                {
                    /* write failure (incl. timeout): stop advancing, drop this packet */
                    DBG_BT_ERR("uart write fail, ret = %d\n", ret);
                    break;
                }
                sent_len += (uint16_t)ret;
                DBG_BT_INF("driver -> uart: %d\n", ret);
            }
        }
    }

    DBG_BT_INF("aicwf_bt_uart_server_driver_task exit\n");

    aicwf_rtos_task_delete(NULL);
}

/**
 * @brief BT UART Driver->Server direction task
 *
 * Reads HCI packets (EVT/ACL/SCO) of the BT chip from UART,
 * parses the HCI headers and payload with a state machine, then sends the
 * packets to the BT Server through the queue.
 *
 * @param args task argument (unused)
 */
static void aicwf_bt_uart_driver_server_task(void *args)
{
    DBG_BT_INF("reasm task enter\n");

    while (1)
    {
        uint8_t driver_buffer[BT_SOCKET_BUFFER_SIZE] = {0};
        int ret_len = -1;
        bool read_done = false;
        uint8_t tmp_buf[256];
        uint32_t len_type, len_header, len_data, offset = 0;
        int rd = 0;
        uint8_t evt_len = 0;
        uint16_t acl_len = 0;
        uint8_t sco_len = 0;
        uint16_t check_offset = 0;
        HCI_RXST_T state = HCI_MSGTYPE_ST;

        while (!read_done)
        {
            switch (state)
            {
            case HCI_MSGTYPE_ST:
                rd = bt_uart_plat_read(tmp_buf, HCI_TYPE_LEN);
                if (rd < 0)
                {
                    /* real error (HAL failure); timeout (=0) means link idle */
                    DBG_BT_ERR("read len_type error, ret=%d\n", rd);
                    read_done = true;
                    break;
                }
                if (rd != HCI_TYPE_LEN)
                {
                    /* timeout with no data = link idle (e.g. the chip stays
                     * silent during BLE idle advertising); silently wait for
                     * the next byte, not treated as an error */
                    break;
                }
                len_type = (uint32_t)HCI_TYPE_LEN;
                driver_buffer[0] = tmp_buf[0];
                state = HCI_LEN_ST;
                offset = 0;
                break;
            case HCI_LEN_ST:
                if (driver_buffer[0] == HCI_EVT_PKT)
                {
                    len_header = bt_uart_plat_read((tmp_buf + offset), (HCI_LE_EVT_HEADER_LEN - offset));
                    if (!len_header)
                    {
                        DBG_BT_ERR("read evt len_header error\n");
                        read_done = true;
                        break;
                    }
                    if ((len_header + offset) < HCI_LE_EVT_HEADER_LEN)
                    {
                        DBG_BT_INF("read evt len_header continue %d\n", len_header);
                        offset += len_header;
                        break;
                    }
                    evt_len = tmp_buf[1];
                }
                else if (driver_buffer[0] == HCI_ACL_PKT)
                {
                    len_header = bt_uart_plat_read((tmp_buf + offset), (HCI_LE_ACL_HEADER_LEN - offset));
                    if (!len_header)
                    {
                        DBG_BT_ERR("read acl len_header error\n");
                        read_done = true;
                        break;
                    }
                    if ((len_header + offset) < HCI_LE_ACL_HEADER_LEN)
                    {
                        DBG_BT_INF("read acl len_header continue %d\n", len_header);
                        offset += len_header;
                        break;
                    }
                    acl_len = (uint16_t)tmp_buf[2] | ((uint16_t)tmp_buf[3] << 8);
                    DBG_BT_INF("acl_len = %d\n", acl_len);
                }
                else if (driver_buffer[0] == HCI_SCO_PKT)
                {
                    len_header = bt_uart_plat_read((tmp_buf + offset), (HCI_SCO_HEADER_LEN - offset));
                    if (!len_header)
                    {
                        DBG_BT_ERR("read sco len_header error\n");
                        read_done = true;
                        break;
                    }
                    if ((len_header + offset) < HCI_SCO_HEADER_LEN)
                    {
                        DBG_BT_INF("read sco len_header continue %d\n", len_header);
                        offset += len_header;
                        break;
                    }
                    sco_len = tmp_buf[2];
                    DBG_BT_INF("sco_len = %d\n", sco_len);
                }
                else
                {
                    DBG_BT_ERR("hci len st error, type=0x%02x\n", driver_buffer[0]);
                    read_done = true;
                    break;
                }
                len_header += offset;
                memcpy(driver_buffer + len_type, tmp_buf, len_header);
                state = HCI_DATA_ST;
                offset = 0;
                break;
            case HCI_DATA_ST:
                if (driver_buffer[0] == HCI_EVT_PKT)
                {
                    len_data = bt_uart_plat_read(tmp_buf, (evt_len - check_offset));
                    if (!len_data)
                    {
                        DBG_BT_ERR("read evt len_data error, len_data = %d\n", len_data);
                        read_done = true;
                        break;
                    }
                    if (len_data + check_offset < evt_len)
                    {
                        memcpy(driver_buffer + len_type + len_header + check_offset, tmp_buf, len_data);
                        check_offset += len_data;
                        break;
                    }
                    memcpy(driver_buffer + len_type + len_header + check_offset, tmp_buf, len_data);
                    len_data = evt_len;
                }
                else if (driver_buffer[0] == HCI_ACL_PKT)
                {
                    len_data = bt_uart_plat_read(tmp_buf, (acl_len - check_offset));
                    if (!len_data)
                    {
                        DBG_BT_ERR("read acl len_data error, len_data = %d\n", len_data);
                        /* the original code missed read_done here, so a read timeout spun in a 20ms busy loop */
                        read_done = true;
                        break;
                    }
                    if (len_data + check_offset < acl_len)
                    {
                        memcpy(driver_buffer + len_type + len_header + check_offset, tmp_buf, len_data);
                        check_offset += len_data;
                        break;
                    }
                    memcpy(driver_buffer + len_type + len_header + check_offset, tmp_buf, len_data);
                    len_data = acl_len;
                }
                else if (driver_buffer[0] == HCI_SCO_PKT)
                {
                    len_data = bt_uart_plat_read(tmp_buf, (sco_len - check_offset));
                    if (!len_data)
                    {
                        DBG_BT_ERR("read sco len_data error, len_data = %d\n", len_data);
                        read_done = true;
                        break;
                    }
                    if (len_data + check_offset < sco_len)
                    {
                        memcpy(driver_buffer + len_type + len_header + check_offset, tmp_buf, len_data);
                        check_offset += len_data;
                        break;
                    }
                    memcpy(driver_buffer + len_type + len_header + check_offset, tmp_buf, len_data);
                    len_data = sco_len;
                }
                else
                {
                    DBG_BT_ERR("hci data st error\n");
                    read_done = true;
                    break;
                }
                state = HCI_DONE_ST;
                check_offset = 0;
                break;
            case HCI_DONE_ST:
                read_done = true;
                ret_len = len_type + len_header + len_data;
                break;
            default:
                break;
            }
        }
        if (ret_len > 0)
        {
            DBG_BT_INF("driver -> server: type=0x%02x len=%d\n", driver_buffer[0], ret_len);
            if (s_hci_rx_cb != NULL)
            {
                s_hci_rx_cb(driver_buffer, (uint32_t)ret_len);
            }
            else if (bt_driver_server_queue != NULL)
            {
                aicwf_rtos_queue_write(bt_driver_server_queue, 0, driver_buffer,
                                       portMAX_DELAY, 0);
            }
        }
    }

    DBG_BT_INF("bt_uart_stop\n");
    aicwf_rtos_task_delete(NULL);
}

/**
 * @brief Start the UART H4 reassembly task (single instance)
 *
 * The legacy and hci entry points share one reassembly task; if it is
 * already running it is reused directly, so that two tasks never read the
 * same UART concurrently and split the byte stream.
 *
 * @return 0 on success, -1 on failure
 */
static int bt_uart_reasm_task_start(void)
{
    if (s_rx_reasm_task != NULL)
    {
        return 0;
    }
    if (aicwf_rtos_task_create(aicwf_bt_uart_driver_server_task, "bt_uart_rx", 0,
                               4096, NULL, BT_DRIVER_TASK_PRIORITY,
                               &s_rx_reasm_task) != 0)
    {
        s_rx_reasm_task = NULL;
        DBG_BT_ERR("reasm task create fail\n");
        return -1;
    }
    DBG_BT_INF("reasm task created\n");
    return 0;
}

/**
 * @brief Start BT UART communication (legacy dual-queue BT Server entry)
 *
 * Creates the bidirectional message queues, opens the UART (921600 baud),
 * and creates the Server->Driver and Driver->Server data transfer tasks.
 *
 * Legacy dual-queue BT Server entry. NimBLE/AT bt_uart_start is not in this file.
 */
int aic_bt_uart_legacy_start(void)
{
    aicwf_rtos_queue_create(20, BT_SOCKET_BUFFER_SIZE, &bt_driver_server_queue, NULL);
    if (bt_driver_server_queue == NULL)
    {
        DBG_BT_ERR("drv->srv queue create fail\n");
        return -1;
    }

    aicwf_rtos_queue_create(20, BT_SOCKET_BUFFER_SIZE, &bt_server_driver_queue, NULL);
    if (bt_server_driver_queue == NULL)
    {
        DBG_BT_ERR("srv->drv queue create fail\n");
        aicwf_rtos_queue_delete(bt_driver_server_queue);
        bt_driver_server_queue = NULL;
        return -1;
    }

    if (bt_uart_plat_open() != 0)
    {
        return -1;
    }

    if (bt_uart_reasm_task_start() != 0)
    {
        return -1;
    }
    if (aicwf_rtos_task_create(aicwf_bt_uart_server_driver_task, "bt_uart_sd_task", 0, 4096, NULL, BT_DRIVER_TASK_PRIORITY, &bt_uart_server_driver_task_handle) != 0)
    {
        DBG_BT_ERR("sd task create fail\n");
        return -1;
    }
    DBG_BT_INF("legacy start ok\n");

    return 0;
}

void
aic_bt_uart_set_rx_cb(aic_bt_uart_rx_cb_t cb)
{
    s_hci_rx_cb = cb;
}

int
aic_bt_uart_ready(void)
{
    return (uap != NULL) ? 1 : 0;
}

int
aic_bt_uart_send(const uint8_t *h4, uint32_t len)
{
    uint32_t sent = 0u;
    int ret;

    if (uap == NULL || h4 == NULL || len == 0u)
    {
        DBG_BT_ERR("tx rejected: uap=%p h4=%p len=%u\n", (void *)uap, (const void *)h4, len);
        return -1;
    }
    DBG_BT_INF("tx: type=0x%02x len=%u\n", h4[0], len);
    while (sent < len)
    {
        ret = bt_uart_plat_write((uint8_t *)(h4 + sent), len - sent);
        if (ret <= 0)
        {
            DBG_BT_ERR("tx fail at %u/%u, ret=%d\n", sent, len, ret);
            return -1;
        }
        sent += (uint32_t)ret;
    }
    DBG_BT_INF("tx done: %u bytes\n", sent);
    return 0;
}

int
aic_bt_uart_hci_start(void)
{
    if (s_uart_hci_started)
    {
        return (uap != NULL) ? 0 : -1;
    }
    if (bt_uart_plat_open() != 0)
    {
        return -1;
    }
    if (bt_uart_reasm_task_start() != 0)
    {
        return -1;
    }
    s_uart_hci_started = 1;
    DBG_BT_INF("uart hci started\n");
    return 0;
}
