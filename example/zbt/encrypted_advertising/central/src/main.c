/* main.c - Application main entry point */

/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/ead.h>
#include <zephyr/bluetooth/conn.h>

#include <zephyr/logging/log.h>

#include "common.h"

LOG_MODULE_DECLARE(ead_central_sample, CONFIG_BT_EAD_LOG_LEVEL);

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */

extern int run_central_sample(int get_passkey_confirmation(struct bt_conn *conn),
                              uint8_t *received_data, size_t received_data_size,
                              struct key_material *keymat);

static struct k_poll_signal button_pressed_signal;

static int get_passkey_confirmation(struct bt_conn *conn)
{
    int err;

    printk("Confirm passkey by enter conf\n");

    err = bt_conn_auth_passkey_confirm(conn);
    if (err)
    {
        LOG_DBG("Failed to confirm passkey.");
        return -1;
    }

    printk("Passkey confirmed.\n");

    return 0;
}


int main(void)
{
    int err;

    printk("Starting central sample...");

    (void)run_central_sample(get_passkey_confirmation, NULL, 0, NULL);

    return 0;
}
