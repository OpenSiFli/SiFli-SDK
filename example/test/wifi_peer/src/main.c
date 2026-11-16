/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * wifi_peer instrument firmware: application entry.
 *
 * One image, two jobs:
 *
 *  - It is a real SoftAP, built the way example/rt_device/wifi/ap is built: it
 *    brings up SiFli_AP / 12345678 by itself and runs a TCP echo server on
 *    port 8888, so the board can be joined, pinged and exercised by hand.
 *  - It is the `wifi_peer` test instrument: the `peer` MSH command (see
 *    peer.h) drives the same AP over the line protocol the suites use, so a
 *    case picks the SSID, key and BSSID it wants to assert against.
 *
 * The Wi-Fi driver starts itself before main (INIT_APP_EXPORT in the AIC8800
 * integration), so main only has to bind the AP interface, start the network
 * and bring up the echo server.
 *
 * A log line printed here must never start with '@': the host parser treats
 * that character as the start of a protocol line.
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <sys/socket.h>

#include <wlan_mgnt.h>

#include "peer.h"

#define ECHO_BUF_SIZE       512
/* How long the listening thread waits before trying to bind again. The AP
 * interface has no address until the first SoftAP is up, so a bind at boot is
 * expected to fail on a board whose AP comes up later. */
#define ECHO_RETRY_MS       2000

static rt_thread_t echo_tid = RT_NULL;

static void echo_server_thread(void *param)
{
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    char buf[ECHO_BUF_SIZE];
    int recv_len;
    int opt = 1;

    (void)param;

    /* One loop that never returns: failing to listen is a state to retry, not
     * a reason to leave the image without the service it promises. */
    while (1)
    {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0)
        {
            peer_log("[AP Echo] socket create failed, retry in %d s",
                     ECHO_RETRY_MS / 1000);
            rt_thread_mdelay(ECHO_RETRY_MS);
            continue;
        }

        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
        {
            peer_log("[AP Echo] setsockopt failed");
            closesocket(server_fd);
            rt_thread_mdelay(ECHO_RETRY_MS);
            continue;
        }

        rt_memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(PEER_AP_ECHO_PORT);

        if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
            /* Normal until an AP interface has an address. */
            peer_log("[AP Echo] bind port %d failed, retry in %d s",
                     PEER_AP_ECHO_PORT, ECHO_RETRY_MS / 1000);
            closesocket(server_fd);
            rt_thread_mdelay(ECHO_RETRY_MS);
            continue;
        }

        if (listen(server_fd, 1) < 0)
        {
            peer_log("[AP Echo] listen failed, retry in %d s",
                     ECHO_RETRY_MS / 1000);
            closesocket(server_fd);
            rt_thread_mdelay(ECHO_RETRY_MS);
            continue;
        }

        peer_log("[AP Echo] server listening on port %d", PEER_AP_ECHO_PORT);

        while (1)
        {
            client_addr_len = sizeof(client_addr);
            client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_addr_len);
            if (client_fd < 0)
            {
                rt_thread_mdelay(100);
                continue;
            }

            peer_log("[AP Echo] client connected: %d.%d.%d.%d:%d",
                     client_addr.sin_addr.s_addr & 0xff,
                     (client_addr.sin_addr.s_addr >> 8) & 0xff,
                     (client_addr.sin_addr.s_addr >> 16) & 0xff,
                     (client_addr.sin_addr.s_addr >> 24) & 0xff,
                     ntohs(client_addr.sin_port));

            while (1)
            {
                recv_len = recv(client_fd, buf, sizeof(buf) - 1, 0);
                if (recv_len <= 0)
                {
                    peer_log("[AP Echo] client disconnected");
                    break;
                }

                buf[recv_len] = '\0';
                peer_log("[AP Echo] recv(%d): %s", recv_len, buf);

                if (send(client_fd, buf, recv_len, 0) < 0)
                {
                    peer_log("[AP Echo] send failed");
                    break;
                }
            }

            closesocket(client_fd);
        }
    }
}

void peer_echo_ensure(void)
{
    if (echo_tid != RT_NULL)
        return;

    echo_tid = rt_thread_create("ap_echo", echo_server_thread, RT_NULL,
                                4096, 20, 10);
    if (echo_tid != RT_NULL)
    {
        rt_thread_startup(echo_tid);
    }
    else
    {
        peer_log("[AP Echo] create echo server thread failed");
    }
}

/**
 * Start the SoftAP by hand: `ap_start [ssid] [key]`.
 *
 * Same path the instrument uses, so what a human sees here is what a case
 * sees. Without arguments it serves PEER_AP_SSID / PEER_AP_PASSWORD.
 */
static int ap_start(int argc, char **argv)
{
    const char *ssid = (argc > 1) ? argv[1] : PEER_AP_SSID;
    const char *key = (argc > 2) ? argv[2] : PEER_AP_PASSWORD;

    if (peer_ap_start_simple(ssid, key, RT_NULL) != 0)
    {
        peer_log("[AP] start softap \"%s\" failed", ssid);
        return -1;
    }

    /* A hand-started AP gets the echo service too. */
    peer_echo_ensure();

    peer_log("[AP] softap \"%s\" is up", ssid);
    return 0;
}
MSH_CMD_EXPORT(ap_start, start softap: ap_start [ssid] [key]);

static int ap_stop(int argc, char **argv)
{
    peer_reply_t reply;

    (void)argc;
    (void)argv;

    rt_memset(&reply, 0, sizeof(reply));
    if (peer_ap_stop(&reply) != 0)
    {
        peer_log("[AP] stop softap failed: %s", reply.text);
        return -1;
    }

    peer_log("[AP] softap stopped");
    return 0;
}
MSH_CMD_EXPORT(ap_stop, stop softap);

int main(void)
{
    /* The output lock must exist before any event can be emitted. */
    peer_proto_init();

    /* Bind the AP interface before anything reads the MAC or starts the AP. */
    peer_ap_init();

    /* peer_log(), not rt_kprintf(): a host may send `peer info` the moment the
     * console answers, and both must not interleave mid-line. */
    peer_log("wifi_peer: %s ready, proto %d", PEER_FW_ID, PEER_PROTO_VERSION);
    peer_log("wifi_peer: auto start softap \"%s\", echo port %d",
             PEER_AP_SSID, PEER_AP_ECHO_PORT);

    /* The echo service belongs to the image, not to the boot-time AP: start it
     * even when the AP does not come up, so any later ap.start leaves a
     * listening port behind instead of a promise that is off for this run. */
    peer_echo_ensure();

    /* An AP that fails to start is not fatal: the suites can still drive it
     * with `peer req ... ap.start`, and a failing boot log says why. */
    peer_ap_autostart();

    while (1)
        rt_thread_mdelay(10000);

    return RT_EOK;
}
