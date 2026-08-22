/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtthread.h"
#include <string.h>
#include <wlan_mgnt.h>
#include <sys/socket.h>

/* ============================================================
 *  User Configuration - Modify the macros below
 * ============================================================ */
#define P2P_GO_SSID         "DIRECT-SiFli"      /* P2P GO SSID */
#define P2P_GO_PASSWORD     "12345678"          /* P2P GO password (min 8 chars) */

#define ECHO_SERVER_PORT    8888                /* TCP echo server port */
#define ECHO_BUF_SIZE       256                 /* Echo buffer size */


/**
 * @brief  TCP echo server thread - echoes back all received data
 */
static void p2p_echo_server_thread(void *param)
{
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    char buf[ECHO_BUF_SIZE];
    int recv_len;

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0)
    {
        rt_kprintf("[P2P Echo] socket create failed\n");
        return;
    }

    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
        rt_kprintf("[P2P Echo] setsockopt failed\n");
        closesocket(server_fd);
        return;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(ECHO_SERVER_PORT);

    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        rt_kprintf("[P2P Echo] bind port %d failed\n", ECHO_SERVER_PORT);
        closesocket(server_fd);
        return;
    }

    if (listen(server_fd, 1) < 0)
    {
        rt_kprintf("[P2P Echo] listen failed\n");
        closesocket(server_fd);
        return;
    }

    rt_kprintf("[P2P Echo] server listening on port %d\n", ECHO_SERVER_PORT);

    while (1)
    {
        client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_addr_len);
        if (client_fd < 0)
        {
            rt_thread_mdelay(100);
            continue;
        }

        rt_kprintf("[P2P Echo] client connected: %d.%d.%d.%d:%d\n",
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
                rt_kprintf("[P2P Echo] client disconnected\n");
                break;
            }
            buf[recv_len] = '\0';
            rt_kprintf("[P2P Echo] recv(%d): %s\n", recv_len, buf);
            send(client_fd, buf, recv_len, 0);
        }
        closesocket(client_fd);
    }
}


/**
 * @brief  P2P GO start and echo server thread entry
 */
static void wifi_p2p_auto_go_thread(void *parameter)
{
    rt_err_t ret;
    rt_thread_t tid;

    rt_kprintf("\n[P2P] Starting P2P GO: SSID=\"%s\"\n", P2P_GO_SSID);
    ret = rt_wlan_p2p_go_start(P2P_GO_SSID, P2P_GO_PASSWORD);
    if (ret != RT_EOK)
    {
        rt_kprintf("[P2P] P2P GO start failed! error: %d\n", ret);
        return;
    }

    rt_kprintf("[P2P] P2P GO started\n");

    tid = rt_thread_create("p2p_echo",
                           p2p_echo_server_thread,
                           RT_NULL,
                           4096,
                           20,
                           10);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        rt_kprintf("[P2P] Failed to create echo server thread!\n");
    }
}


static int p2p_stop(int argc, char **argv)
{
    rt_err_t ret;

    if (!rt_wlan_p2p_go_is_active())
    {
        rt_kprintf("[P2P] P2P GO is not active\n");
        return 0;
    }

    ret = rt_wlan_p2p_go_stop();
    if (ret != RT_EOK)
    {
        rt_kprintf("[P2P] P2P GO stop failed! error: %d\n", ret);
        return -1;
    }

    rt_kprintf("[P2P] P2P GO stopped\n");
    return 0;
}
MSH_CMD_EXPORT(p2p_stop, stop P2P GO mode and TCP echo server);


int main(void)
{
    rt_thread_t tid;
    rt_wlan_set_mode("wlan0", RT_WLAN_AP);

    tid = rt_thread_create("p2p_go",
                           wifi_p2p_auto_go_thread,
                           RT_NULL,
                           4096,
                           20,
                           10);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        rt_kprintf("[P2P] Failed to create P2P GO thread!\n");
    }

    while (1)
    {
        rt_thread_mdelay(10000);
    }
    return 0;
}
