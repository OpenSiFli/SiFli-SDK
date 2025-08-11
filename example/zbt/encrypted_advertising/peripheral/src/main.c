/* main.c - Application main entry point */

/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/conn.h>

#include <zephyr/logging/log.h>

#include "../../include/common.h"

#define BTN_COUNT 1

LOG_MODULE_DECLARE(ead_peripheral_sample, CONFIG_BT_EAD_LOG_LEVEL);

extern int run_peripheral_sample(int get_passkey_confirmation(struct bt_conn *conn));

static struct k_poll_signal button_pressed_signal;

static void button_pressed(uint32_t pins)
{
    printk("Button pressed...");

    k_poll_signal_raise(&button_pressed_signal, 0);
    k_sleep(K_SECONDS(1));
    k_poll_signal_reset(&button_pressed_signal);
}

static void ead_btn(uint8_t argc, char **argv)
{
    if (argc > 1)
    {
        uint8_t sel = strtol(argv[1], NULL, 0);
        if (sel > BTN_COUNT || sel == 0)
            printk("Usage: otc [1-4]");
        else
            button_pressed(sel);
    }
}
MSH_CMD_EXPORT(ead_btn, enter button for OTC example);


static int get_passkey_confirmation(struct bt_conn *conn)
{
    int err;

    await_signal(&button_pressed_signal);

    err = bt_conn_auth_passkey_confirm(conn);
    if (err)
    {
        printk("Failed to confirm passkey.");
        return -1;
    }

    printk("Passkey confirmed.\n");

    return 0;
}

static int setup_btn(void)
{
    int ret;

    k_poll_signal_init(&button_pressed_signal);

    return 0;
}

int main(void)
{
    int err;

    err = setup_btn();
    if (err)
    {
        return 0;
    }

    printk("Starting peripheral sample...");

    (void)run_peripheral_sample(get_passkey_confirmation);
    return 0;
}
