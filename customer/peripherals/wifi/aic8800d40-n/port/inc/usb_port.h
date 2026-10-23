/*
 * Copyright (C) 2018-2025 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB port header: platform USB interface declarations.
 *
 * Architecture (cf. the sdio_co.c <-> sdio_port.c pairing on the SDIO side):
 *
 *   ┌──────────────────────────────────────────────────────────┐
 *   │ Upper layer (wifi_if / fhost_tx / fhost_rx)              │
 *   │   calls aicwf_hal_* (hal_co.c)                           │
 *   └────────────────────────┬─────────────────────────────────┘
 *                            │ #ifdef CONFIG_USB_SUPPORT
 *   ┌────────────────────────┴─────────────────────────────────┐
 *   │ CO layer (usb_co.c) -- protocol logic                    │
 *   │   msg pack / TX aggr / RX unpack / flow ctrl / buf mgmt  │
 *   │   calls Port layer via APIs below (CO -> Port down)      │
 *   │   gives RX/TX completion via callbacks (Port -> CO up)   │
 *   └────────────────────────┬─────────────────────────────────┘
 *                            │
 *   ┌────────────────────────┴─────────────────────────────────┐
 *   │ Port layer (usb_port.c) -- platform specific             │
 *   │   URB submit / URB pool mgmt / endpoint parse            │
 *   │   URB complete callback -> aicwf_usb_rx_complete         │
 *   └──────────────────────────────────────────────────────────┘
 *
 * Compared with the Linux reference driver (aicwf_usb.c), USB does not need
 * the following SDIO concepts:
 *   - CMD52/CMD53 register access (USB uses bulk URB)
 *   - claim_host/release_host (no SDIO host concept on USB)
 *   - readsb/writesb (USB uses URB bulk in/out)
 *   - release_ep2 (SDIO function 2 release, not applicable to USB)
 *   - host_enable_isr/set_irq_handler/interrupt_init/isr (USB uses URB completion)
 *
 * The platform must implement the following functions (with CONFIG_USB_SUPPORT).
 * See platform/hal/usb_port.c for the stub implementation.
 */

#ifndef _USB_PORT_H_
#define _USB_PORT_H_

#include "rtos_port.h"
#include "co_list.h"
#include "aic_usb_types.h"
#include "usb_def.h"

#define CONFIG_RXTASK_INUSB    0
#define CONFIG_RX_POLLING      0
#define CONFIG_IRQ_SUS         0

/* USB TX endpoint selection (counterpart of the SDIO FUNC_1/FUNC_2 split) */
#define USB_EP_DATA            0    /* data endpoint (bulk OUT data) */
#define USB_EP_MSG             1    /* message endpoint (bulk OUT msg, optional) */

/* Export variables (provided by platform or stubs)
 * The global device instance is named g_aic_usb_dev: some platform USB host
 * stacks already have `typedef u8 usb_dev`, which would clash (the aic_bus_dev
 * macro in aic_bus.h refers to the same name) */
extern int  msgcfm_poll_en;
extern uint8_t usb_unexcept_test;
extern struct aic_usb_dev g_aic_usb_dev;

/* ================================================================ */
/*  CO -> Port downlink APIs (called by CO, implemented by Port)    */
/*  SDIO counterpart: aicwf_sdio_writesb / aicwf_sdio_readsb etc.   */
/* ================================================================ */

/*
 * USB function init: parse endpoints, allocate the URB pool, init sync primitives.
 * Does not submit RX URB (done by aicwf_usb_rx_submit_all).
 * Counterpart of Linux aicwf_usb_init + aicwf_parse_usb.
 * Configures no registers (USB has no SDIO function registers).
 *
 * @chipid: chip ID (PRODUCT_ID_AIC8801/DC/DW/D80 etc.)
 * @usbdev: USB device struct pointer
 * Return: 0 on success, negative on failure
 */
int  aicwf_usb_func_init(uint16_t chipid, struct aic_usb_dev *usbdev);

/*
 * USB driver registration: register the driver with the platform USB core.
 * Counterpart of Linux usb_register(&aicwf_usbdrvr).
 * An RTOS platform may call func_init directly at init time instead.
 */
int  aicwf_usb_drv_register(void);

/*
 * Submit a TX bulk OUT URB (asynchronous).
 * Counterpart of SDIO aicwf_sdio_writesb_msg / aicwf_sdio_writesb.
 * Counterpart of usb_submit_urb in Linux aicwf_usb_bus_txmsg / aicwf_usb_bus_txdata.
 *
 * CO layer calls this API after packing the msg header to submit the URB.
 * Port layer does the URB allocation, fill_bulk_urb and submit_urb.
 * After URB completion the Port layer calls aicwf_usb_tx_complete to notify CO.
 *
 * @ep: USB_EP_DATA or USB_EP_MSG (selects the bulk OUT endpoint)
 * @buf: data buffer to send (already contains the 4B usb_header)
 * @len: data length (bytes)
 * Return: 0 on success (URB submitted), negative on failure
 */
int  aicwf_usb_submit_tx_urb(uint8_t ep, uint8_t *buf, uint32_t len);

/*
 * DATA single-packet TX: prepends the 4B USB header (len+type) during the Port copy.
 * Used by the non-AGGR path of aicwf_usb_send; payload is hostdesc+frame, no USB header.
 */
int  aicwf_usb_submit_tx_data(uint8_t *payload, uint32_t payload_len);

/*
 * Wait synchronously for a TX message to complete (msg path only).
 * Counterpart of wait_event_timeout(msg_wait) in Linux aicwf_usb_bus_txmsg.
 *
 * CO layer calls this API after aicwf_usb_submit_tx_urb(USB_EP_MSG, ...).
 * Port layer wakes up the waiter in the msg URB complete callback.
 *
 * @timeout_ms: timeout (milliseconds)
 * Return: 0 on success (URB completed), negative on timeout or failure
 */
int  aicwf_usb_wait_tx_msg_complete(uint32_t timeout_ms);

/*
 * Query the number of free TX URBs (flow control watermark).
 * Counterpart of Linux aicwf_usb_dev.tx_free_count.
 *
 * CO layer aicwf_usb_send_check calls this API to decide whether TX is allowed.
 * When the return value is below LOW_WATER, the CO layer should stop sending.
 *
 * Return: number of free TX URBs
 */
uint32_t aicwf_usb_tx_free_count(void);

/*
 * Submit all RX bulk IN URBs (always resident).
 * Counterpart of Linux aicwf_usb_rx_submit_all_urb.
 *
 * CO layer aicwf_usb_host_init calls this API to start the RX path.
 * Port layer takes all URBs from rx_free_list and calls usb_submit_urb.
 * After URB completion the Port layer calls aicwf_usb_rx_complete to report data.
 *
 * Return: 0 on success, negative on failure
 */
int  aicwf_usb_rx_submit_all(void);

/*
 * Called before FW start_app: the chip soft-disconnects and re-enumerates as 8d81/8d83.
 * Then call aicwf_usb_wait_reconnect() to wait for the new device to be ready.
 */
void aicwf_usb_expect_reconnect(void);
int  aicwf_usb_wait_reconnect(uint32_t timeout_ms);

/*
 * Cancel all pending TX/RX URBs.
 * Counterpart of Linux usb_kill_urb + usb_unlink_urb.
 *
 * CO layer aicwf_usb_host_predeinit/deinit calls these APIs to stop TX/RX.
 */
void aicwf_usb_kill_all_tx_urb(void);
void aicwf_usb_kill_all_rx_urb(void);

#ifdef CONFIG_OOB
    /* USB normally does not use OOB, interface kept for special platforms */
    void aicwf_usb_oob_enable(void);
#endif

/* ================================================================ */
/*  Port -> CO uplink callbacks (CO implements, Port calls)         */
/*  SDIO side: SDIO ISR trigger -> aicwf_sdio_rx_task               */
/*  USB side: URB complete callback -> aicwf_usb_rx_complete        */
/* ================================================================ */

/*
 * RX URB completion callback (called by Port layer).
 * Counterpart of Linux aicwf_usb_rx_complete.
 *
 * Port layer calls this API when a bulk IN URB completes, reporting data to CO.
 * CO layer parses the 4B header, unpacks, queues to rxq and wakes rx_task.
 *
 * @buf: receive data buffer (URB transfer_buffer)
 * @actual_len: URB actual_length (bytes actually received)
 */
void aicwf_usb_rx_complete(uint8_t *buf, uint32_t actual_len);

/*
 * TX URB completion callback (called by Port layer).
 * Counterpart of waking msg_wait in the Linux TX URB complete callback.
 *
 * Port layer calls this API when a bulk OUT URB completes, notifying CO.
 * CO layer (or Port layer) releases the URB to tx_free_list and wakes msg waiters.
 *
 * @ep: USB_EP_DATA or USB_EP_MSG
 * @status: URB completion status (0 success, negative failure)
 */
void aicwf_usb_tx_complete(uint8_t ep, int status);

#endif /* _USB_PORT_H_ */
