/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file main.c
 * @brief USB CDC ACM Host Demo Application
 * @details This file implements a USB CDC ACM host demo that demonstrates
 *          how to use CherryUSB host stack with RT-Thread to communicate
 *          with USB CDC ACM devices.
 * @author SiFli
 * @date 2025-11-28
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "usbh_core.h"
#include "usbh_cdc_acm.h"
#include "usbh_serial.h"

/** @brief USB CDC ACM device handle */
static rt_device_t ttyACM0 = RT_NULL;

/** @brief Receive buffer for USB CDC ACM data */
char recv_buffer[4096] = {0};

/**
 * @brief USB CDC ACM receive callback function
 * @details This callback is invoked when data is received from the USB CDC ACM device.
 *          It reads the received data into the receive buffer and prints it to console.
 * @param[in] dev RT-Thread device handle
 * @param[in] size Number of bytes received
 * @return RT_EOK on success
 */
rt_err_t rx_cb(rt_device_t dev, rt_size_t size)
{
    rt_kprintf("rx callback, size:%d\n", size);
    rt_device_read(dev, 0, recv_buffer, size);
    rt_kprintf("receive data: ");
    for (rt_size_t i = 0; i < size; i++)
    {
        rt_kprintf("%c", recv_buffer[i]);
    }
    rt_kprintf("\n");
    return RT_EOK;
}

/**
 * @brief Main program entry point
 * @details This function initializes the USB host stack, waits for CDC ACM device
 *          connection, opens the device, and sets up receive callback.
 *          The main loop runs indefinitely to keep the application running.
 *
 * @note The function performs the following steps:
 *       1. Initialize USB host controller with base address
 *       2. Wait 2 seconds for device enumeration
 *       3. Find ttyACM0 device
 *       4. Open device with read/write and interrupt receive flags
 *       5. Register receive callback function
 *       6. Set CDC ACM line state (DTR=1, RTS=0)
 *       7. Enter infinite loop
 *
 * @return 0 on success, otherwise failure number
 */
int main(void)
{
    rt_kprintf("cherryusb host demo!\n");

    /* Initialize USB host controller */
    usbh_initialize(0, (uintptr_t)USBC_BASE, RT_NULL);

    /* Wait for device enumeration */
    rt_thread_mdelay(2000);

    /* Find CDC ACM device */
    ttyACM0 = rt_device_find("ttyACM0");
    if (!ttyACM0)
    {
        rt_kprintf("ttyACM0 not found!\n");
    }
    else
    {
        rt_kprintf("ttyACM0 found!\n");
        /* Open device with read/write and interrupt receive mode */
        rt_device_open(ttyACM0, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
        /* Set receive callback function */
        rt_device_set_rx_indicate(ttyACM0, rx_cb);
        /* Set line state: DTR=1, RTS=0 */
        usbh_serial_control((struct usbh_serial *)ttyACM0->user_data, USBH_SERIAL_CMD_IOCMBIS,
                            (void *)USBH_SERIAL_TIOCM_DTR);
    }

    /* Main loop */
    while (1)
    {
        rt_thread_mdelay(1000);
    }
    return 0;
}
