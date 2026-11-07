/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * bt_peer instrument firmware: application entry.
 *
 * The board does nothing on its own. It brings up the Bluetooth stack and
 * waits for the host to drive it over the `peer` MSH command, so a log line
 * printed here must never start with '@'.
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "peer.h"
#include "bf0_sibles.h"

static void peer_app_init(void)
{
    /* The output lock must exist before any BLE callback can emit a line. */
    peer_proto_init();

    peer_ble_init();
    peer_gatt_client_init();
    peer_gatt_server_init();
    peer_sec_init();
    peer_periodic_init();
}

int main(void)
{
    peer_app_init();

    rt_kprintf("bt_peer: %s ready, proto %d\n", PEER_FW_ID, PEER_PROTO_VERSION);

    sifli_ble_enable();

    while (1)
        rt_thread_mdelay(1000);

    return RT_EOK;
}
