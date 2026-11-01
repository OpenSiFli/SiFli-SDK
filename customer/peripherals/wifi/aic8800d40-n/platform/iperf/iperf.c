/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * iperf2-style network performance tool (TCP/UDP, server/client).
 *
 * Ported from t3x shell/cmds/net/cmds_iperf.c (iperf 2.0.9 compatible).
 * TCP/UDP TX/RX logic, wire format and report format are identical to the
 * platform implementation; only platform dependencies (FreeRTOS/systick/
 * shell/newlib getopt) are replaced by the aicwf_rtos_* abstraction layer.
 * Entry point is changed from shell command to iperf_parse_argv/iperf_start/
 * iperf_stop (wifi_port_cmd.c: iwpriv iperf start/stop).
 */

#include "iperf_internal.h"

#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "iperf.h"          /* lwip/netif.h + lwip/inet.h + rtos_port.h */
#include "plat_port.h"      /* TCPIP_PRIORITY */
#include "lwip/ip4_addr.h"
#include "lwip/sockets.h"

/* ============================================================
 * Platform adaptation layer (t3x FreeRTOS/shell deps -> aicwf_rtos_* abstraction)
 * ============================================================ */
#ifndef unlikely
    #define unlikely(x)                     (x)
#endif

#define iperf_msleep(ms)                aicwf_rtos_msleep(ms)
/* Abstraction layer only provides an ms timebase; µs timestamps are ms x1000 (1ms jitter resolution) */
#define iperf_timer_get_time()          ((uint64_t)aicwf_rtos_now(0) * 1000ULL)
/* pacer token bucket timebase: 1 tick = 1ms */
#define IPERF_TICKS_PER_SECOND          1000U
#define iperf_tick_now()                ((uint32_t)aicwf_rtos_now(0))

/* Single-instance tool, no nested critical sections; keep the IRQ mask in a static */
static uint32_t s_iperf_crit_level;
#define taskENTER_CRITICAL()            ((s_iperf_crit_level = aicwf_rtos_critical_enter()))
#define taskEXIT_CRITICAL()             (aicwf_rtos_critical_exit(s_iperf_crit_level))

#define IPERF_IP_TYPE_IPV4          0
#define IPERF_IP_TYPE_IPV6          1

#define IPERF_FLAG_CLIENT           (1U)
#define IPERF_FLAG_SERVER           (1U << 1)
#define IPERF_FLAG_TCP              (1U << 2)
#define IPERF_FLAG_UDP              (1U << 3)
#define IPERF_FLAG_DUAL             (1U << 4)



#define IPERF_DEFAULT_PORT          5001
#define IPERF_DEFAULT_INTERVAL      1
#define IPERF_DEFAULT_TIME          10

#define IPERF_TRAFFIC_TASK_NAME     "iperf_traffic"
/* Synced back from SDK measurements: the original TCPIP_PRIORITY - 1 put the
   traffic task below the tcpip thread. Under LWIP_TCPIP_CORE_LOCKING, socket
   recv runs synchronously in the caller task holding the core lock, so a
   priority-2 traffic task queues at the tail of the protocol task chain,
   buffer draining / window return lags, and TCP throughput is clearly lower
   than the SDK shell implementation of the same lineage (whose priority
   equals tcpip). Raise it to the same level to align. */
#define IPERF_TRAFFIC_TASK_PRIORITY (TCPIP_PRIORITY)
/* Raised from t3x's 2048: report lines use float printf, keep stack headroom */
#define IPERF_TRAFFIC_TASK_STACK_DEPTH 4096
#define IPERF_REPORT_TASK_NAME      "iperf_report"
#define IPERF_REPORT_TASK_STACK_DEPTH 2048
#define IPERF_MAX_TRAFFIC_TASK_PRIORITY (TCPIP_PRIORITY + 2U)

#define IPERF_UDP_TX_LEN            (1470)
#define IPERF_UDP_RX_LEN            (1470)
#define IPERF_TCP_TX_LEN            (4 << 10)
#define IPERF_TCP_RX_LEN            (4 << 10)
#define RECV_DUAL_BUF_LEN           (4 * 1024)

#define IPERF_SOCKET_POLL_TIMEOUT_SEC 1
#define IPERF_DEFAULT_UDP_RATE_BITS_PER_SECOND 1048576ULL
#define IPERF_BYTES_PER_MIBYTE      (1024U * 1024U)
#define IPERF_BITS_PER_MEGABIT      1000000ULL
#define IPERF_MAX_TIME_SECONDS      (UINT32_MAX / 1000U)
#define IPERF_UDP_FIN_RETRY_COUNT   10U
#define IPERF_UDP_FIN_TIMEOUT_MS    250U
#define IPERF_UDP_ACK_FIN_TIMEOUT_MS 1000U
#define IPERF_PACER_DELAY_CHUNK_TICKS 2U
#define IPERF_UDP_MIN_CLIENT_DATAGRAM_LEN \
    (sizeof(iperf_udp_legacy_datagram_wire_t) + sizeof(iperf_udp_legacy_client_header_wire_t))

typedef enum
{
    IPERF_RATE_BITS_PER_SECOND = 0,
    IPERF_RATE_PACKETS_PER_SECOND,
} iperf_rate_unit_t;

typedef struct
{
    uint32_t flag;
    union
    {
        uint32_t destination_ip4;
        char *destination_ip6;
    };
    union
    {
        uint32_t source_ip4;
        char *source_ip6;
    };
    uint8_t type;
    uint16_t dport;
    uint16_t sport;
    uint32_t interval;
    uint32_t time;
    uint32_t len_buf;
    uint32_t window_size;
    uint8_t tos;
    uint8_t traffic_task_priority;
    uint64_t num_bytes;
    uint64_t rate;
    iperf_rate_unit_t rate_unit;
    bool end_by_bytes;
    bool rate_set;
} iperf_cfg_t;

#define IPERF_V6             0


typedef int iperf_err_t;

#define IPERF_OK             0  /*!< iperf_err_t value indicating success (no error) */
#define IPERF_FAIL           -1 /*!< Generic iperf_err_t code indicating failure */

#define IPERF_LOGE(tag, format, ...)                       \
    do {                                                   \
        (void)tag;                                         \
        printf("[%s] " format "\r\n", tag, ##__VA_ARGS__); \
    } while (0)

#define IPERF_LOGW(tag, format, ...)                       \
    do {                                                   \
        (void)tag;                                         \
        printf("[%s] " format "\r\n", tag, ##__VA_ARGS__); \
    } while (0)

#define IPERF_LOGI(tag, format, ...)                       \
    do {                                                   \
        (void)tag;                                         \
        printf("[%s] " format "\r\n", tag, ##__VA_ARGS__); \
    } while (0)

#define IPERF_LOGD(tag, format, ...)                       \
    do {                                                   \
        (void)tag;                                         \
        printf("[%s] " format "\r\n", tag, ##__VA_ARGS__); \
    } while (0)

#define IPERF_LOGV(tag, format, ...)                       \
    do {                                                   \
        (void)tag;                                         \
        printf("[%s] " format "\r\n", tag, ##__VA_ARGS__); \
    } while (0)

#define IPERF_GOTO_ON_FALSE(a, err_code, goto_tag, log_tag, format, ...) \
    do {                                                                 \
        if (unlikely(!(a))) {                                            \
            IPERF_LOGE(log_tag, format, ##__VA_ARGS__);                  \
            ret = err_code;                                              \
            goto goto_tag;                                               \
        }                                                                \
    } while (0)

#define NL "\r\n"

typedef struct
{
    iperf_cfg_t cfg;
    volatile bool finish;
    uint64_t tot_len;
    uint64_t start_time_us;
    uint64_t end_time_us;
    uint32_t buffer_len;
    uint8_t *buffer;
    int sockfd;
    volatile rtos_task_handle report_task;
    volatile rtos_task_handle dual_task;
    iperf_udp_stats_t udp_stats;
    iperf_pacer_t pacer;
    bool transfer_started;
    bool transfer_completed;
} iperf_ctrl_t;

static volatile bool s_iperf_is_running = false;
static iperf_ctrl_t s_iperf_ctrl;
static const char *TAG = "iperf";

static inline bool iperf_is_udp_client(void)
{
    return ((s_iperf_ctrl.cfg.flag & IPERF_FLAG_CLIENT) &&
            (s_iperf_ctrl.cfg.flag & IPERF_FLAG_UDP));
}

static inline bool iperf_is_udp_server(void)
{
    return ((s_iperf_ctrl.cfg.flag & IPERF_FLAG_SERVER) &&
            (s_iperf_ctrl.cfg.flag & IPERF_FLAG_UDP));
}

static inline bool iperf_is_tcp_client(void)
{
    return ((s_iperf_ctrl.cfg.flag & IPERF_FLAG_CLIENT) &&
            (s_iperf_ctrl.cfg.flag & IPERF_FLAG_TCP));
}

static inline bool iperf_is_tcp_dual_client(void)
{
    return ((s_iperf_ctrl.cfg.flag & IPERF_FLAG_CLIENT) &&
            (s_iperf_ctrl.cfg.flag & IPERF_FLAG_TCP) &&
            (s_iperf_ctrl.cfg.flag & IPERF_FLAG_DUAL));
}

static inline bool iperf_is_tcp_server(void)
{
    return ((s_iperf_ctrl.cfg.flag & IPERF_FLAG_SERVER) &&
            (s_iperf_ctrl.cfg.flag & IPERF_FLAG_TCP));
}

static uint64_t iperf_account_transfer(size_t transfer_len)
{
    uint64_t total_len;

    taskENTER_CRITICAL();
    s_iperf_ctrl.tot_len += transfer_len;
    total_len = s_iperf_ctrl.tot_len;
    taskEXIT_CRITICAL();
    return total_len;
}

static uint64_t iperf_snapshot_total_len(void)
{
    uint64_t total_len;

    taskENTER_CRITICAL();
    total_len = s_iperf_ctrl.tot_len;
    taskEXIT_CRITICAL();
    return total_len;
}

static void iperf_publish_udp_stats(const iperf_udp_stats_t *stats)
{
    taskENTER_CRITICAL();
    s_iperf_ctrl.udp_stats = *stats;
    taskEXIT_CRITICAL();
}

static void iperf_snapshot_udp_stats(iperf_udp_stats_t *stats_out)
{
    taskENTER_CRITICAL();
    *stats_out = s_iperf_ctrl.udp_stats;
    taskEXIT_CRITICAL();
}

static void iperf_begin_transfer(void)
{
    uint64_t now_us = iperf_timer_get_time();

    taskENTER_CRITICAL();
    if (!s_iperf_ctrl.transfer_started)
    {
        s_iperf_ctrl.start_time_us = now_us;
        s_iperf_ctrl.transfer_started = true;
    }
    taskEXIT_CRITICAL();
}

static void iperf_mark_transfer_complete(void)
{
    uint64_t now_us = iperf_timer_get_time();

    taskENTER_CRITICAL();
    if (s_iperf_ctrl.transfer_started && !s_iperf_ctrl.transfer_completed)
    {
        s_iperf_ctrl.end_time_us = now_us;
        s_iperf_ctrl.transfer_completed = true;
    }
    taskEXIT_CRITICAL();
}

static uint64_t iperf_transfer_elapsed_us(void)
{
    uint64_t start_time_us;
    uint64_t end_time_us;
    bool transfer_started;
    bool transfer_completed;

    taskENTER_CRITICAL();
    start_time_us = s_iperf_ctrl.start_time_us;
    end_time_us = s_iperf_ctrl.end_time_us;
    transfer_started = s_iperf_ctrl.transfer_started;
    transfer_completed = s_iperf_ctrl.transfer_completed;
    taskEXIT_CRITICAL();

    if (!transfer_started)
    {
        return 0;
    }
    if (!transfer_completed)
    {
        end_time_us = iperf_timer_get_time();
    }
    return end_time_us >= start_time_us ? end_time_us - start_time_us : 0;
}

static bool iperf_transfer_bytes_complete(void)
{
    return s_iperf_ctrl.cfg.end_by_bytes &&
           iperf_snapshot_total_len() >= s_iperf_ctrl.cfg.num_bytes;
}

static bool iperf_transfer_time_expired(void)
{
    uint64_t time_limit_us;

    if (s_iperf_ctrl.cfg.end_by_bytes)
    {
        return false;
    }
    time_limit_us = (uint64_t)s_iperf_ctrl.cfg.time * 1000000ULL;
    return iperf_transfer_elapsed_us() >= time_limit_us;
}

static bool iperf_should_stop_transfer(void)
{
    return s_iperf_ctrl.finish || iperf_transfer_bytes_complete() ||
           iperf_transfer_time_expired();
}

static size_t iperf_limit_send_len(size_t requested_len)
{
    uint64_t total_len;
    uint64_t remaining_len;

    if (!s_iperf_ctrl.cfg.end_by_bytes)
    {
        return requested_len;
    }

    total_len = iperf_snapshot_total_len();
    if (total_len >= s_iperf_ctrl.cfg.num_bytes)
    {
        return 0;
    }
    remaining_len = s_iperf_ctrl.cfg.num_bytes - total_len;
    return remaining_len < requested_len ? (size_t)remaining_len : requested_len;
}

static int iperf_get_socket_error_code(int sockfd)
{
    (void)sockfd;
    return errno;
}

static int iperf_show_socket_error_reason(const char *str, int sockfd)
{
    int err = errno;

    (void)sockfd;
    if (err != 0)
    {
        IPERF_LOGW(TAG, "%s error, error code: %d, reason: %s", str, err, strerror(err));
    }

    return err;
}

static bool iperf_socket_timed_out(int err)
{
    return err == EAGAIN || err == EWOULDBLOCK;
}

static bool iperf_socket_error_is_transient(int err)
{
    return err == ENOMEM || err == ENOBUFS || iperf_socket_timed_out(err);
}

static iperf_err_t iperf_set_recv_timeout_ms(int sockfd, uint32_t timeout_ms)
{
    struct timeval timeout =
    {
        .tv_sec = timeout_ms / 1000U,
        .tv_usec = (timeout_ms % 1000U) * 1000U,
    };

    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0)
    {
        iperf_show_socket_error_reason("set receive timeout", sockfd);
        return IPERF_FAIL;
    }

    return IPERF_OK;
}

/*
 * UDP receive wait that can be interrupted by stop.
 * A single blocking recvfrom waits the whole fixed timeout (250ms/1000ms) at
 * once and is unaware of s_iperf_ctrl.finish; if the lwIP port does not really
 * implement SO_RCVTIMEO it may even block forever. Here the wait is split into
 * 50ms short-timeout slices, checking finish between slices so that the stop
 * command can interrupt promptly.
 *
 * @param timeout_ms total wait budget (must be > 0)
 * @return >0 bytes received; 0 interrupted by finish; -1 overall timeout; -2 hard socket error
 */
static int iperf_recvfrom_interruptible(int sockfd,
                                        void *buf,
                                        size_t len,
                                        struct sockaddr *addr,
                                        socklen_t *addr_len,
                                        uint32_t timeout_ms)
{
    const uint32_t chunk_ms = 50U;
    uint32_t waited_ms = 0;

    while (waited_ms < timeout_ms)
    {
        uint32_t step = (timeout_ms - waited_ms) > chunk_ms ?
                        chunk_ms : (timeout_ms - waited_ms);
        struct timeval timeout;
        int ret;

        timeout.tv_sec = step / 1000U;
        timeout.tv_usec = (step % 1000U) * 1000U;
        if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0)
        {
            iperf_show_socket_error_reason("set receive timeout", sockfd);
            return -2;
        }
        ret = recvfrom(sockfd, buf, len, 0, addr, addr_len);
        if (ret >= 0)
        {
            return ret;
        }
        if (s_iperf_ctrl.finish)
        {
            return 0;
        }
        if (!iperf_socket_timed_out(errno))
        {
            iperf_show_socket_error_reason("udp intercept recv", sockfd);
            return -2;
        }
        waited_ms += step;
    }

    return -1;
}

static iperf_err_t iperf_set_recv_timeout(int sockfd)
{
    return iperf_set_recv_timeout_ms(sockfd, IPERF_SOCKET_POLL_TIMEOUT_SEC * 1000U);
}

static iperf_err_t iperf_configure_socket(int sockfd)
{
    int tos = (int)s_iperf_ctrl.cfg.tos;

    if (setsockopt(sockfd, IPPROTO_IP, IP_TOS, &tos, sizeof(tos)) != 0)
    {
        iperf_show_socket_error_reason("set IP TOS", sockfd);
        return IPERF_FAIL;
    }
    if (s_iperf_ctrl.cfg.window_size > 0U)
    {
        int requested_window = (int)s_iperf_ctrl.cfg.window_size;
        int actual_window = 0;
        socklen_t actual_window_len = sizeof(actual_window);

        if (setsockopt(sockfd,
                       SOL_SOCKET,
                       SO_RCVBUF,
                       &requested_window,
                       sizeof(requested_window)) != 0)
        {
            iperf_show_socket_error_reason("set receive window", sockfd);
            return IPERF_FAIL;
        }
        if (getsockopt(sockfd,
                       SOL_SOCKET,
                       SO_RCVBUF,
                       &actual_window,
                       &actual_window_len) != 0)
        {
            iperf_show_socket_error_reason("verify receive window", sockfd);
            return IPERF_FAIL;
        }
        if (actual_window < requested_window)
        {
            IPERF_LOGW(TAG,
                       "receive window limited to %d bytes (requested %d bytes)",
                       actual_window,
                       requested_window);
        }
    }

    return IPERF_OK;
}

static int iperf_accept_client(int listen_socket,
                               struct sockaddr *remote_addr,
                               socklen_t *addr_len)
{
    int client_socket;

    while (!s_iperf_ctrl.finish)
    {
        client_socket = accept(listen_socket, remote_addr, addr_len);
        if (client_socket >= 0)
        {
            return client_socket;
        }
        if (!iperf_socket_timed_out(errno))
        {
            iperf_show_socket_error_reason("accept", listen_socket);
            break;
        }
    }

    return -1;
}

static void iperf_notify_report_task(void)
{
    /* Task notification is a stub in this project; the report task polls at 100ms, no explicit wakeup needed */
}

static void iperf_print_udp_report_line(double start_seconds,
                                        double end_seconds,
                                        uint64_t transfer_bytes,
                                        const iperf_udp_stats_t *stats)
{
    double duration_seconds = end_seconds - start_seconds;
    double transfer_mbytes = transfer_bytes / (double)IPERF_BYTES_PER_MIBYTE;
    double bandwidth_mbps = 0.0;

    if (duration_seconds > 0.0)
    {
        bandwidth_mbps = (double)transfer_bytes * 8.0 /
                         duration_seconds / IPERF_BITS_PER_MEGABIT;
    }
    printf("[%3d] %5.1f-%5.1f sec  %.2f MByte  %.2f Mbits/sec  "
           "%.3f ms  %llu/%llu (%.2f%%)\r\n",
           s_iperf_ctrl.sockfd,
           start_seconds,
           end_seconds,
           transfer_mbytes,
           bandwidth_mbps,
           stats->jitter_us / 1000.0,
           (unsigned long long)stats->lost_datagrams,
           (unsigned long long)stats->next_packet_id,
           iperf_udp_stats_loss_percent(stats));
}

static void iperf_print_transfer_report_line(double start_seconds,
        double end_seconds,
        uint64_t transfer_bytes)
{
    double duration_seconds = end_seconds - start_seconds;
    double transfer_mbytes = transfer_bytes / (double)IPERF_BYTES_PER_MIBYTE;
    double bandwidth_mbps = 0.0;

    if (duration_seconds > 0.0)
    {
        bandwidth_mbps = (double)transfer_bytes * 8.0 /
                         duration_seconds / IPERF_BITS_PER_MEGABIT;
    }
    printf("[%3d] %5.1f-%5.1f sec  %.2f MByte  %.2f Mbits/sec\r\n",
           s_iperf_ctrl.sockfd,
           start_seconds,
           end_seconds,
           transfer_mbytes,
           bandwidth_mbps);
}

/* Report task periodic wait: 100ms polling so it exits promptly after iperf stop */
static void iperf_interval_sleep(uint32_t total_ms)
{
    uint32_t elapsed = 0;

    while (elapsed < total_ms && !s_iperf_ctrl.finish)
    {
        uint32_t step = (total_ms - elapsed) > 100U ? 100U : (total_ms - elapsed);

        iperf_msleep(step);
        elapsed += step;
    }
}

static void iperf_report_task(void *arg)
{
    uint32_t interval = s_iperf_ctrl.cfg.interval;
    uint32_t delay_interval_ms = interval * 1000U;
    uint64_t previous_elapsed_us = 0;
    uint64_t previous_total_len = 0;

    (void)arg;
    if (delay_interval_ms == 0)
    {
        delay_interval_ms = 1;
    }
    if (iperf_is_udp_server())
    {
        printf("[ ID] Interval       Transfer     Bandwidth       Jitter   "
               "Lost/Total Datagrams\r\n");
    }
    else
    {
        printf("[ ID] Interval       Transfer     Bandwidth\r\n");
    }
    while (!s_iperf_ctrl.finish)
    {
        iperf_interval_sleep(delay_interval_ms);
        if (s_iperf_ctrl.finish)
        {
            break;
        }
        {
            uint64_t elapsed_us = iperf_transfer_elapsed_us();
            uint64_t total_len = iperf_snapshot_total_len();
            uint64_t interval_len = total_len - previous_total_len;

            if (elapsed_us <= previous_elapsed_us)
            {
                continue;
            }
            if (iperf_is_udp_server())
            {
                iperf_udp_stats_t stats;

                iperf_snapshot_udp_stats(&stats);
                iperf_print_udp_report_line(previous_elapsed_us / 1000000.0,
                                            elapsed_us / 1000000.0,
                                            interval_len,
                                            &stats);
            }
            else
            {
                iperf_print_transfer_report_line(previous_elapsed_us / 1000000.0,
                                                 elapsed_us / 1000000.0,
                                                 interval_len);
            }
            previous_elapsed_us = elapsed_us;
            previous_total_len = total_len;
        }
    }

    s_iperf_ctrl.report_task = NULL;
    aicwf_rtos_task_delete(NULL);
}

static void iperf_print_final_report(void)
{
    uint64_t total_len;
    uint64_t elapsed_us;

    taskENTER_CRITICAL();
    if (!s_iperf_ctrl.transfer_started)
    {
        taskEXIT_CRITICAL();
        return;
    }
    taskEXIT_CRITICAL();

    total_len = iperf_snapshot_total_len();
    elapsed_us = iperf_transfer_elapsed_us();
    if (iperf_is_udp_server())
    {
        iperf_udp_stats_t stats;

        iperf_snapshot_udp_stats(&stats);
        iperf_print_udp_report_line(0.0, elapsed_us / 1000000.0, total_len, &stats);
    }
    else
    {
        iperf_print_transfer_report_line(0.0, elapsed_us / 1000000.0, total_len);
    }
    printf("iperf done\r\n");
}

static iperf_err_t iperf_start_report(void)
{
    rtos_task_handle report_task = NULL;
    int ret;

    iperf_begin_transfer();
    if (s_iperf_ctrl.report_task != NULL)
    {
        return IPERF_OK;
    }

    ret = aicwf_rtos_task_create(iperf_report_task,
                                 IPERF_REPORT_TASK_NAME,
                                 0,
                                 IPERF_REPORT_TASK_STACK_DEPTH,
                                 NULL,
                                 s_iperf_ctrl.cfg.traffic_task_priority + 1U,
                                 &report_task);

    if (ret != AIC_RTOS_SUCCESS)
    {
        s_iperf_ctrl.report_task = NULL;
        IPERF_LOGE(TAG, "create task %s failed", IPERF_REPORT_TASK_NAME);
        return IPERF_FAIL;
    }
    s_iperf_ctrl.report_task = report_task;

    return IPERF_OK;
}

static void socket_recv_tcp(int recv_socket)
{
    bool iperf_recv_start = true;
    uint8_t *buffer;
    int actual_recv = 0;

    buffer = s_iperf_ctrl.buffer;
    while (!s_iperf_ctrl.finish)
    {
        size_t want_recv = iperf_limit_send_len(s_iperf_ctrl.buffer_len);

        if (want_recv == 0U)
        {
            break;
        }
        actual_recv = recv(recv_socket, buffer, want_recv, 0);
        if (actual_recv < 0)
        {
            if (iperf_socket_timed_out(errno))
            {
                if (iperf_transfer_time_expired())
                {
                    break;
                }
                continue;
            }
            iperf_show_socket_error_reason("tcp server recv", recv_socket);
            s_iperf_ctrl.finish = true;
            break;
        }
        else if (actual_recv == 0)
        {
            break;
        }
        else
        {
            if (iperf_recv_start)
            {
                if (iperf_start_report() != IPERF_OK)
                {
                    s_iperf_ctrl.finish = true;
                    break;
                }
                iperf_recv_start = false;
            }
            (void)iperf_account_transfer((size_t)actual_recv);
            if (iperf_should_stop_transfer())
            {
                break;
            }
        }
    }
}

static void socket_recv_dual(int recv_socket, struct sockaddr_storage listen_addr)
{
    uint8_t *buffer;
    int want_recv = 0;
    int actual_recv = 0;
    socklen_t socklen = sizeof(struct sockaddr_in);

    buffer = aicwf_rtos_malloc(RECV_DUAL_BUF_LEN);
    want_recv = RECV_DUAL_BUF_LEN;
    if (!buffer)
    {
        return;
    }
    while (!s_iperf_ctrl.finish)
    {
        actual_recv = recvfrom(recv_socket,
                               buffer,
                               want_recv,
                               0,
                               (struct sockaddr *)&listen_addr,
                               &socklen);
        if (actual_recv < 0 && iperf_socket_timed_out(errno))
        {
            continue;
        }
        if (actual_recv <= 0)
        {
            break;
        }
    }
    aicwf_rtos_free(buffer);
}

typedef struct
{
    uint64_t total_len;
    uint64_t duration_us;
    uint64_t jitter_us;
    uint64_t error_count;
    uint64_t out_of_order_count;
    uint64_t datagrams;
} iperf_udp_server_result_t;

typedef union
{
    iperf_udp_server_report_wire_t modern;
    iperf_udp_legacy_server_report_full_wire_t legacy_full;
    iperf_udp_legacy_server_report_wire_t legacy_short;
} iperf_udp_server_report_buffer_t;

static uint32_t iperf_client_header_flags(bool dual_mode)
{
    uint32_t flags = 0;

    if (dual_mode)
    {
        flags |= IPERF_HEADER_VERSION_1 | IPERF_HEADER_RUN_NOW;
    }
    if (s_iperf_ctrl.cfg.rate_unit == IPERF_RATE_PACKETS_PER_SECOND)
    {
        flags |= IPERF_HEADER_UNITS_PPS;
    }
    return flags;
}

static uint32_t iperf_client_header_amount(void)
{
    if (s_iperf_ctrl.cfg.end_by_bytes)
    {
        return (uint32_t)s_iperf_ctrl.cfg.num_bytes;
    }
    return (uint32_t)(-(int32_t)(s_iperf_ctrl.cfg.time * 100U));
}

static void iperf_build_client_header(iperf_udp_legacy_client_header_wire_t *header,
                                      bool dual_mode)
{
    memset(header, 0, sizeof(*header));
    header->flags = htonl(iperf_client_header_flags(dual_mode));
    header->num_threads = htonl(1U);
    header->port = htonl(s_iperf_ctrl.cfg.sport);
    header->buffer_len = htonl(s_iperf_ctrl.buffer_len);
    header->window_size = htonl(s_iperf_ctrl.cfg.window_size);
    header->amount = htonl(iperf_client_header_amount());
    header->rate = htonl((uint32_t)s_iperf_ctrl.cfg.rate);
    header->rate_units = htonl(s_iperf_ctrl.cfg.rate_unit ==
                               IPERF_RATE_PACKETS_PER_SECOND ?
                               IPERF_HEADER_UNITS_PPS : 0U);
}

static void send_dual_header(int sock, struct sockaddr *addr, socklen_t socklen)
{
    iperf_udp_legacy_client_header_wire_t header;

    iperf_build_client_header(&header, true);
    if (sendto(sock, &header, sizeof(header), 0, addr, socklen) != (int)sizeof(header))
    {
        iperf_show_socket_error_reason("send dual-test header", sock);
    }
}

static void iperf_prepare_udp_legacy_datagram(uint8_t *buffer,
        int32_t packet_id,
        uint64_t time_us)
{
    iperf_udp_legacy_datagram_wire_t *datagram =
        (iperf_udp_legacy_datagram_wire_t *)buffer;

    datagram->id = htonl((uint32_t)packet_id);
    datagram->tv_sec = htonl((uint32_t)(time_us / 1000000ULL));
    datagram->tv_usec = htonl((uint32_t)(time_us % 1000000ULL));
}

static void iperf_prepare_udp_extended_datagram(uint8_t *buffer,
        int64_t packet_id,
        uint64_t time_us)
{
    iperf_udp_datagram_wire_t *datagram = (iperf_udp_datagram_wire_t *)buffer;
    uint64_t encoded_packet_id = (uint64_t)packet_id;

    datagram->id = htonl((uint32_t)encoded_packet_id);
    datagram->tv_sec = htonl((uint32_t)(time_us / 1000000ULL));
    datagram->tv_usec = htonl((uint32_t)(time_us % 1000000ULL));
    datagram->id_high = htonl((uint32_t)(encoded_packet_id >> 32));
}

static int64_t iperf_decode_udp_packet_id(uint32_t id_low,
        uint32_t id_high,
        bool sequence64)
{
    uint64_t encoded_packet_id;

    if (!sequence64)
    {
        return (int32_t)id_low;
    }

    encoded_packet_id = ((uint64_t)id_high << 32) | id_low;
    if (encoded_packet_id <= INT64_MAX)
    {
        return (int64_t)encoded_packet_id;
    }

    return -1 - (int64_t)(UINT64_MAX - encoded_packet_id);
}

static bool iperf_udp_datagram_uses_64bit_sequence(const uint8_t *buffer,
        size_t buffer_len)
{
    const iperf_udp_datagram_wire_t *datagram;
    const iperf_udp_client_header_wire_t *header;

    if (buffer == NULL || buffer_len < sizeof(*datagram))
    {
        return false;
    }

    datagram = (const iperf_udp_datagram_wire_t *)buffer;
    if ((int32_t)ntohl(datagram->id) < 0 && ntohl(datagram->id_high) == UINT32_MAX)
    {
        return true;
    }

    if (buffer_len < sizeof(*datagram) + sizeof(header->base.flags))
    {
        return false;
    }
    header = (const iperf_udp_client_header_wire_t *)(buffer + sizeof(*datagram));
    return (ntohl(header->base.flags) & IPERF_HEADER_SEQNO_64BIT) != 0;
}

static bool iperf_parse_udp_datagram(const uint8_t *buffer,
                                     size_t buffer_len,
                                     bool sequence64,
                                     int64_t *packet_id_out,
                                     uint64_t *sent_time_us_out)
{
    const iperf_udp_legacy_datagram_wire_t *legacy_datagram;
    uint32_t id_high = 0;

    if (buffer == NULL || buffer_len < sizeof(*legacy_datagram) ||
            packet_id_out == NULL || sent_time_us_out == NULL)
    {
        return false;
    }
    if (sequence64 && buffer_len < sizeof(iperf_udp_datagram_wire_t))
    {
        return false;
    }

    legacy_datagram = (const iperf_udp_legacy_datagram_wire_t *)buffer;
    if (sequence64)
    {
        const iperf_udp_datagram_wire_t *datagram =
            (const iperf_udp_datagram_wire_t *)buffer;

        id_high = ntohl(datagram->id_high);
    }
    *packet_id_out = iperf_decode_udp_packet_id(ntohl(legacy_datagram->id),
                     id_high,
                     sequence64);
    *sent_time_us_out = (uint64_t)ntohl(legacy_datagram->tv_sec) * 1000000ULL +
                        ntohl(legacy_datagram->tv_usec);
    return true;
}

static bool iperf_prepare_udp_legacy_client_header(uint8_t *buffer, size_t buffer_len)
{
    iperf_udp_legacy_client_header_wire_t *header;
    size_t header_offset = sizeof(iperf_udp_legacy_datagram_wire_t);

    if (buffer == NULL || buffer_len < header_offset + sizeof(*header))
    {
        return false;
    }

    header = (iperf_udp_legacy_client_header_wire_t *)(buffer + header_offset);
    iperf_build_client_header(header, false);
    return true;
}

static void iperf_delay_after_transient_error(uint32_t failure_streak)
{
    iperf_msleep(iperf_pacer_backoff_ticks(failure_streak));
}

static iperf_err_t iperf_start_pacer(size_t packet_size_bytes)
{
    uint64_t rate_bits_per_second;
    uint64_t packet_bits;

    if (!s_iperf_ctrl.cfg.rate_set)
    {
        return IPERF_OK;
    }
    if (s_iperf_ctrl.cfg.rate_unit == IPERF_RATE_PACKETS_PER_SECOND)
    {
        if (packet_size_bytes == 0U || packet_size_bytes > SIZE_MAX / 8U)
        {
            IPERF_LOGE(TAG, "UDP packets-per-second rate overflows the pacer");
            return IPERF_FAIL;
        }
        packet_bits = (uint64_t)packet_size_bytes * 8U;
        if (s_iperf_ctrl.cfg.rate > UINT64_MAX / packet_bits)
        {
            IPERF_LOGE(TAG, "UDP packets-per-second rate overflows the pacer");
            return IPERF_FAIL;
        }
        rate_bits_per_second = s_iperf_ctrl.cfg.rate * packet_bits;
    }
    else
    {
        rate_bits_per_second = s_iperf_ctrl.cfg.rate;
    }

    iperf_pacer_init(&s_iperf_ctrl.pacer,
                     rate_bits_per_second,
                     packet_size_bytes,
                     IPERF_TICKS_PER_SECOND,
                     iperf_tick_now());
    return IPERF_OK;
}

static bool iperf_wait_for_pacer(size_t packet_size_bytes)
{
    uint32_t wait_ticks;

    while (!iperf_should_stop_transfer())
    {
        if (iperf_pacer_try_consume(&s_iperf_ctrl.pacer,
                                    packet_size_bytes,
                                    iperf_tick_now(),
                                    &wait_ticks))
        {
            return true;
        }
        if (wait_ticks > IPERF_PACER_DELAY_CHUNK_TICKS)
        {
            wait_ticks = IPERF_PACER_DELAY_CHUNK_TICKS;
        }
        iperf_msleep(wait_ticks);
    }

    return false;
}

static void iperf_send_tcp(int send_socket,
                           struct sockaddr_storage dest_addr)
{
    uint8_t *buffer = s_iperf_ctrl.buffer;
    uint32_t transient_failures = 0;
    uint32_t failure_streak = 0;
#if IPERF_V6
    const socklen_t socklen = (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6) ?
                              sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
#else
    const socklen_t socklen = sizeof(struct sockaddr_in);
#endif

    if (iperf_start_report() != IPERF_OK)
    {
        s_iperf_ctrl.finish = true;
        return;
    }
    if (iperf_start_pacer(s_iperf_ctrl.buffer_len) != IPERF_OK)
    {
        s_iperf_ctrl.finish = true;
        return;
    }
    if (iperf_is_tcp_dual_client())
    {
        send_dual_header(send_socket, (struct sockaddr *)&dest_addr, socklen);
    }

    while (!iperf_should_stop_transfer())
    {
        size_t send_len = iperf_limit_send_len(s_iperf_ctrl.buffer_len);
        size_t sent_len = 0;

        if (send_len == 0U)
        {
            break;
        }
        while (sent_len < send_len && !iperf_should_stop_transfer())
        {
            int actual_send;
            int err;

            if (!iperf_wait_for_pacer(send_len - sent_len))
            {
                break;
            }
            actual_send = sendto(send_socket,
                                 buffer + sent_len,
                                 send_len - sent_len,
                                 0,
                                 (struct sockaddr *)&dest_addr,
                                 socklen);
            if (actual_send > 0)
            {
                failure_streak = 0;
                sent_len += (size_t)actual_send;
                (void)iperf_account_transfer((size_t)actual_send);
                continue;
            }

            err = iperf_get_socket_error_code(send_socket);
            if (iperf_socket_error_is_transient(err))
            {
                transient_failures++;
                failure_streak++;
                iperf_delay_after_transient_error(failure_streak);
                continue;
            }
            iperf_show_socket_error_reason("tcp client send", send_socket);
            return;
        }
    }

    if (transient_failures > 0)
    {
        IPERF_LOGW(TAG, "TCP send transient retries: %" PRIu32, transient_failures);
    }
}

static uint64_t iperf_send_udp_data(int send_socket,
                                    struct sockaddr_storage dest_addr)
{
    uint8_t *buffer = s_iperf_ctrl.buffer;
    uint64_t packet_id = 0;
    uint32_t transient_failures = 0;
    uint32_t failure_streak = 0;
#if IPERF_V6
    const socklen_t socklen = (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6) ?
                              sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
#else
    const socklen_t socklen = sizeof(struct sockaddr_in);
#endif

    if (iperf_start_report() != IPERF_OK)
    {
        s_iperf_ctrl.finish = true;
        return 0;
    }
    if (!iperf_prepare_udp_legacy_client_header(buffer, s_iperf_ctrl.buffer_len) ||
            iperf_start_pacer(s_iperf_ctrl.buffer_len) != IPERF_OK)
    {
        s_iperf_ctrl.finish = true;
        return 0;
    }

    while (!iperf_should_stop_transfer())
    {
        size_t want_send = iperf_limit_send_len(s_iperf_ctrl.buffer_len);
        int actual_send;
        int err;

        if (want_send == 0U)
        {
            break;
        }
        if (want_send < sizeof(iperf_udp_legacy_datagram_wire_t))
        {
            IPERF_LOGW(TAG, "UDP transfer amount leaves a short datagram");
            break;
        }
        if (packet_id >= (uint64_t)INT32_MAX - IPERF_UDP_FIN_RETRY_COUNT)
        {
            IPERF_LOGW(TAG, "UDP packet sequence exhausted");
            break;
        }
        if (!iperf_wait_for_pacer(want_send))
        {
            break;
        }
        iperf_prepare_udp_legacy_datagram(buffer,
                                          (int32_t)packet_id,
                                          iperf_timer_get_time());
        actual_send = sendto(send_socket,
                             buffer,
                             want_send,
                             0,
                             (struct sockaddr *)&dest_addr,
                             socklen);
        if (actual_send == (int)want_send)
        {
            failure_streak = 0;
            (void)iperf_account_transfer((size_t)actual_send);
            packet_id++;
            continue;
        }

        err = iperf_get_socket_error_code(send_socket);
        if (iperf_socket_error_is_transient(err))
        {
            transient_failures++;
            failure_streak++;
            iperf_delay_after_transient_error(failure_streak);
            continue;
        }
        iperf_show_socket_error_reason("udp client send", send_socket);
        break;
    }

    if (transient_failures > 0)
    {
        IPERF_LOGW(TAG, "UDP send transient retries: %" PRIu32, transient_failures);
    }
    return packet_id;
}

static void iperf_parse_udp_server_header(
    const iperf_udp_server_header_v1_wire_t *header,
    const iperf_udp_server_extension2_wire_t *extension2,
    iperf_udp_server_result_t *result_out)
{
    uint32_t flags = ntohl(header->flags);
    uint64_t error_count_high = 0;
    uint64_t out_of_order_count_high = 0;
    uint64_t datagrams_high = 0;

    if (extension2 != NULL && (flags & IPERF_HEADER_SEQNO_64BIT) != 0)
    {
        error_count_high = ntohl(extension2->error_count_high);
        out_of_order_count_high = ntohl(extension2->out_of_order_count_high);
        datagrams_high = ntohl(extension2->datagrams_high);
    }

    result_out->total_len = ((uint64_t)ntohl(header->total_len_high) << 32) |
                            ntohl(header->total_len_low);
    result_out->duration_us = (uint64_t)ntohl(header->stop_sec) * 1000000ULL +
                              ntohl(header->stop_usec);
    result_out->error_count = (error_count_high << 32) | ntohl(header->error_count);
    result_out->out_of_order_count = (out_of_order_count_high << 32) |
                                     ntohl(header->out_of_order_count);
    result_out->datagrams = (datagrams_high << 32) | ntohl(header->datagrams);
    result_out->jitter_us = (uint64_t)ntohl(header->jitter_sec) * 1000000ULL +
                            ntohl(header->jitter_usec);
}

static bool iperf_parse_udp_server_report(const uint8_t *buffer,
        size_t buffer_len,
        iperf_udp_server_result_t *result_out)
{
    const iperf_udp_server_report_wire_t *report;
    const iperf_udp_legacy_server_report_full_wire_t *legacy_full_report;
    const iperf_udp_legacy_server_report_wire_t *legacy_short_report;

    if (buffer == NULL || result_out == NULL)
    {
        return false;
    }

    if (buffer_len >= sizeof(*report))
    {
        report = (const iperf_udp_server_report_wire_t *)buffer;
        if ((ntohl(report->server.base.flags) & IPERF_HEADER_VERSION_1) != 0)
        {
            iperf_parse_udp_server_header(&report->server.base,
                                          &report->server.extension2,
                                          result_out);
            return true;
        }
    }

    if (buffer_len >= sizeof(*legacy_full_report))
    {
        legacy_full_report = (const iperf_udp_legacy_server_report_full_wire_t *)buffer;
        if ((ntohl(legacy_full_report->server.base.flags) & IPERF_HEADER_VERSION_1) != 0 &&
                (ntohl(legacy_full_report->server.base.flags) & IPERF_HEADER_SEQNO_64BIT) == 0)
        {
            iperf_parse_udp_server_header(&legacy_full_report->server.base, NULL, result_out);
            return true;
        }
    }

    if (buffer_len < sizeof(*legacy_short_report))
    {
        return false;
    }
    legacy_short_report = (const iperf_udp_legacy_server_report_wire_t *)buffer;
    if ((ntohl(legacy_short_report->server.flags) & IPERF_HEADER_VERSION_1) == 0 ||
            (ntohl(legacy_short_report->server.flags) & IPERF_HEADER_SEQNO_64BIT) != 0)
    {
        return false;
    }
    iperf_parse_udp_server_header(&legacy_short_report->server, NULL, result_out);
    return true;
}

static void iperf_print_udp_server_result(const iperf_udp_server_result_t *result)
{
    iperf_udp_stats_t stats;

    iperf_udp_stats_init(&stats);
    stats.jitter_us = (double)result->jitter_us;
    stats.lost_datagrams = result->error_count;
    stats.out_of_order_datagrams = result->out_of_order_count;
    stats.next_packet_id = result->datagrams;

    printf("Server Report:\r\n");
    printf("[ ID] Interval       Transfer     Bandwidth       Jitter   "
           "Lost/Total Datagrams\r\n");
    iperf_print_udp_report_line(0.0,
                                result->duration_us / 1000000.0,
                                result->total_len,
                                &stats);
    if (result->out_of_order_count > 0)
    {
        printf("[iperf] Out-of-order datagrams: %llu\r\n",
               (unsigned long long)result->out_of_order_count);
    }
}

static iperf_err_t iperf_send_udp_fin(int send_socket,
                                      struct sockaddr_storage dest_addr,
                                      uint64_t packet_count)
{
    uint8_t *buffer = s_iperf_ctrl.buffer;
    uint32_t attempt;
#if IPERF_V6
    const socklen_t socklen = (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6) ?
                              sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
#else
    const socklen_t socklen = sizeof(struct sockaddr_in);
#endif

    if (packet_count > (uint64_t)INT32_MAX - IPERF_UDP_FIN_RETRY_COUNT)
    {
        IPERF_LOGE(TAG, "UDP packet sequence cannot be represented by iperf 2.0.9");
        return IPERF_FAIL;
    }

    for (attempt = 0; attempt < IPERF_UDP_FIN_RETRY_COUNT; attempt++)
    {
        if (s_iperf_ctrl.finish)
        {
            break;
        }
        struct sockaddr_storage report_addr = { 0 };
        socklen_t report_addr_len = sizeof(report_addr);
        iperf_udp_server_report_buffer_t report_buffer;
        iperf_udp_server_result_t result;
        uint64_t fin_sequence = packet_count > 0U ? packet_count + attempt : attempt + 1U;
        int32_t fin_packet_id = -(int32_t)fin_sequence;
        int actual_send;
        int actual_recv;

        iperf_prepare_udp_legacy_datagram(buffer, fin_packet_id, iperf_timer_get_time());
        actual_send = sendto(send_socket,
                             buffer,
                             s_iperf_ctrl.buffer_len,
                             0,
                             (struct sockaddr *)&dest_addr,
                             socklen);
        if (actual_send != (int)s_iperf_ctrl.buffer_len)
        {
            int err = iperf_get_socket_error_code(send_socket);

            if (iperf_socket_error_is_transient(err))
            {
                iperf_delay_after_transient_error(attempt + 1U);
                continue;
            }
            iperf_show_socket_error_reason("udp client FIN send", send_socket);
            return IPERF_FAIL;
        }

        actual_recv = iperf_recvfrom_interruptible(send_socket,
                      &report_buffer,
                      sizeof(report_buffer),
                      (struct sockaddr *)&report_addr,
                      &report_addr_len,
                      IPERF_UDP_FIN_TIMEOUT_MS);
        if (actual_recv == 0)
        {
            break;
        }
        if (actual_recv > 0)
        {
            if (iperf_parse_udp_server_report((const uint8_t *)&report_buffer,
                                              (size_t)actual_recv,
                                              &result))
            {
                iperf_print_udp_server_result(&result);
                return IPERF_OK;
            }
            continue;
        }
        if (actual_recv == -2)
        {
            return IPERF_FAIL;
        }
    }

    IPERF_LOGW(TAG,
               "did not receive UDP server report after %u FIN attempts",
               IPERF_UDP_FIN_RETRY_COUNT);
    return IPERF_FAIL;
}

static void iperf_build_udp_server_header(iperf_udp_server_header_v1_wire_t *header,
        const iperf_udp_stats_t *stats,
        uint64_t duration_us,
        uint32_t flags)
{
    uint64_t jitter_us = (uint64_t)(stats->jitter_us + 0.5);

    header->flags = htonl(flags);
    header->total_len_high = htonl((uint32_t)(stats->received_bytes >> 32));
    header->total_len_low = htonl((uint32_t)stats->received_bytes);
    header->stop_sec = htonl((uint32_t)(duration_us / 1000000ULL));
    header->stop_usec = htonl((uint32_t)(duration_us % 1000000ULL));
    header->error_count = htonl((uint32_t)stats->lost_datagrams);
    header->out_of_order_count = htonl((uint32_t)stats->out_of_order_datagrams);
    header->datagrams = htonl((uint32_t)stats->next_packet_id);
    header->jitter_sec = htonl((uint32_t)(jitter_us / 1000000ULL));
    header->jitter_usec = htonl((uint32_t)(jitter_us % 1000000ULL));
}

static void iperf_encode_udp_time(int64_t time_us,
                                  uint32_t *seconds_out,
                                  uint32_t *microseconds_out)
{
    uint64_t nonnegative_time_us = time_us > 0 ? (uint64_t)time_us : 0U;

    *seconds_out = htonl((uint32_t)(nonnegative_time_us / 1000000ULL));
    *microseconds_out = htonl((uint32_t)(nonnegative_time_us % 1000000ULL));
}

static void iperf_encode_udp_double_time(double time_us,
        uint32_t *seconds_out,
        uint32_t *microseconds_out)
{
    int64_t rounded_time_us;

    if (time_us <= 0.0)
    {
        rounded_time_us = 0;
    }
    else if (time_us >= (double)INT64_MAX)
    {
        rounded_time_us = INT64_MAX;
    }
    else
    {
        rounded_time_us = (int64_t)(time_us + 0.5);
    }
    iperf_encode_udp_time(rounded_time_us, seconds_out, microseconds_out);
}

static void iperf_build_udp_server_extension(iperf_udp_server_extension_wire_t *extension,
        const iperf_udp_stats_t *stats,
        uint64_t duration_us)
{
    uint64_t ipg_count = 0;

    memset(extension, 0, sizeof(*extension));
    iperf_encode_udp_time(stats->min_transit_us,
                          &extension->min_transit_sec,
                          &extension->min_transit_usec);
    iperf_encode_udp_time(stats->max_transit_us,
                          &extension->max_transit_sec,
                          &extension->max_transit_usec);
    iperf_encode_udp_time(stats->sum_transit_us,
                          &extension->sum_transit_sec,
                          &extension->sum_transit_usec);
    iperf_encode_udp_double_time(stats->mean_transit_us,
                                 &extension->mean_transit_sec,
                                 &extension->mean_transit_usec);
    iperf_encode_udp_double_time(stats->m2_transit_us,
                                 &extension->m2_transit_sec,
                                 &extension->m2_transit_usec);
    iperf_encode_udp_double_time(stats->variance_transit_us,
                                 &extension->variance_transit_sec,
                                 &extension->variance_transit_usec);
    extension->transit_count = htonl((uint32_t)stats->transit_count);
    if (duration_us > 0U)
    {
        if (stats->received_datagrams > UINT64_MAX / 1000000ULL)
        {
            ipg_count = UINT64_MAX;
        }
        else
        {
            ipg_count = stats->received_datagrams * 1000000ULL / duration_us;
        }
    }
    extension->ipg_count = htonl((uint32_t)ipg_count);
    extension->ipg_sum = htonl(duration_us > 0U ? 1U : 0U);
}

static size_t iperf_build_udp_server_report(iperf_udp_server_report_buffer_t *report,
        const iperf_udp_stats_t *stats,
        uint64_t duration_us,
        bool sequence64)
{
    uint64_t report_sequence = stats->next_packet_id > 0 ? stats->next_packet_id : 1U;
    int64_t report_packet_id;
    uint64_t now_us = iperf_timer_get_time();

    if (sequence64 && report_sequence > INT64_MAX)
    {
        report_sequence = INT64_MAX;
    }
    else if (!sequence64 && report_sequence > (uint64_t)INT32_MAX)
    {
        report_sequence = INT32_MAX;
    }
    report_packet_id = -(int64_t)report_sequence;
    memset(report, 0, sizeof(*report));

    if (sequence64)
    {
        iperf_prepare_udp_extended_datagram((uint8_t *)&report->modern,
                                            report_packet_id,
                                            now_us);
        iperf_build_udp_server_header(
            &report->modern.server.base,
            stats,
            duration_us,
            IPERF_HEADER_VERSION_1 | IPERF_HEADER_SEQNO_64BIT);
        iperf_build_udp_server_extension(&report->modern.server.extension, stats, duration_us);
        report->modern.server.extension2.error_count_high =
            htonl((uint32_t)(stats->lost_datagrams >> 32));
        report->modern.server.extension2.out_of_order_count_high =
            htonl((uint32_t)(stats->out_of_order_datagrams >> 32));
        report->modern.server.extension2.datagrams_high =
            htonl((uint32_t)(stats->next_packet_id >> 32));
        return sizeof(report->modern);
    }

    iperf_prepare_udp_legacy_datagram((uint8_t *)&report->legacy_full,
                                      (int32_t)report_packet_id,
                                      now_us);
    iperf_build_udp_server_header(&report->legacy_full.server.base,
                                  stats,
                                  duration_us,
                                  IPERF_HEADER_VERSION_1);
    iperf_build_udp_server_extension(&report->legacy_full.server.extension,
                                     stats,
                                     duration_us);
    return sizeof(report->legacy_full);
}

static void iperf_send_udp_ack_fin(int server_socket,
                                   const struct sockaddr_storage *client_addr,
                                   socklen_t client_addr_len,
                                   const iperf_udp_stats_t *stats,
                                   uint64_t duration_us,
                                   bool sequence64)
{
    iperf_udp_server_report_buffer_t report;
    size_t report_len;
    uint32_t attempt;

    report_len = iperf_build_udp_server_report(&report, stats, duration_us, sequence64);

    for (attempt = 0; attempt < IPERF_UDP_FIN_RETRY_COUNT; attempt++)
    {
        if (s_iperf_ctrl.finish)
        {
            break;
        }
        struct sockaddr_storage repeat_addr = { 0 };
        socklen_t repeat_addr_len = sizeof(repeat_addr);
        uint64_t sent_time_us;
        int64_t packet_id;
        int actual_send;
        int actual_recv;

        actual_send = sendto(server_socket,
                             &report,
                             report_len,
                             0,
                             (const struct sockaddr *)client_addr,
                             client_addr_len);
        if (actual_send != (int)report_len)
        {
            int err = iperf_get_socket_error_code(server_socket);

            if (iperf_socket_error_is_transient(err))
            {
                iperf_delay_after_transient_error(attempt + 1U);
                continue;
            }
            iperf_show_socket_error_reason("udp server AckFIN send", server_socket);
            break;
        }
        actual_recv = iperf_recvfrom_interruptible(server_socket,
                      s_iperf_ctrl.buffer,
                      s_iperf_ctrl.buffer_len,
                      (struct sockaddr *)&repeat_addr,
                      &repeat_addr_len,
                      IPERF_UDP_ACK_FIN_TIMEOUT_MS);
        if (actual_recv < 1)
        {
            break;
        }
        if (!iperf_parse_udp_datagram(s_iperf_ctrl.buffer,
                                      (size_t)actual_recv,
                                      sequence64,
                                      &packet_id,
                                      &sent_time_us) || packet_id >= 0)
        {
            break;
        }
    }
}

static void iperf_recv_udp(int recv_socket)
{
    iperf_udp_stats_t stats;
    iperf_udp_sequence_tracker_t sequence_tracker;
    bool report_started = false;
    bool sequence64 = false;

    iperf_udp_stats_init(&stats);
    iperf_udp_sequence_tracker_init(&sequence_tracker);
    iperf_publish_udp_stats(&stats);

    while (!s_iperf_ctrl.finish)
    {
        struct sockaddr_storage client_addr = { 0 };
        socklen_t client_addr_len = sizeof(client_addr);
        uint64_t arrival_time_us;
        uint64_t sent_time_us;
        int64_t packet_id;
        int actual_recv;

        actual_recv = iperf_recvfrom_interruptible(recv_socket,
                      s_iperf_ctrl.buffer,
                      s_iperf_ctrl.buffer_len,
                      (struct sockaddr *)&client_addr,
                      &client_addr_len,
                      IPERF_SOCKET_POLL_TIMEOUT_SEC * 1000U);
        arrival_time_us = iperf_timer_get_time();
        if (actual_recv == 0)
        {
            break;
        }
        if (actual_recv < 0)
        {
            if (actual_recv == -1)
            {
                if (iperf_transfer_time_expired())
                {
                    break;
                }
                continue;
            }
            iperf_show_socket_error_reason("udp server recv", recv_socket);
            break;
        }
        if (!sequence64 &&
                iperf_udp_datagram_uses_64bit_sequence(s_iperf_ctrl.buffer,
                        (size_t)actual_recv))
        {
            sequence64 = true;
        }
        if (!iperf_parse_udp_datagram(s_iperf_ctrl.buffer,
                                      (size_t)actual_recv,
                                      sequence64,
                                      &packet_id,
                                      &sent_time_us))
        {
            IPERF_LOGW(TAG, "ignored short UDP datagram (%d bytes)", actual_recv);
            continue;
        }

        if (packet_id < 0)
        {
            uint64_t expected_datagrams = (uint64_t)(-(packet_id + 1)) + 1U;
            uint64_t duration_us = stats.sequence_initialized ?
                                   arrival_time_us - stats.first_arrival_us : 0;

            iperf_udp_stats_finish(&stats, expected_datagrams);
            iperf_publish_udp_stats(&stats);
            iperf_mark_transfer_complete();
            if (stats.out_of_order_datagrams > 0)
            {
                printf("[iperf] Out-of-order datagrams: %llu\r\n",
                       (unsigned long long)stats.out_of_order_datagrams);
            }
            if (stats.duplicate_datagrams > 0)
            {
                printf("[iperf] Duplicate datagrams: %llu\r\n",
                       (unsigned long long)stats.duplicate_datagrams);
            }
            iperf_send_udp_ack_fin(recv_socket,
                                   &client_addr,
                                   client_addr_len,
                                   &stats,
                                   duration_us,
                                   sequence64);
            break;
        }

        if (!report_started)
        {
            if (client_addr.ss_family == AF_INET)
            {
                const struct sockaddr_in *client_addr4 =
                    (const struct sockaddr_in *)&client_addr;

                IPERF_LOGI(TAG,
                           "UDP client %s:%u connected",
                           inet_ntoa(client_addr4->sin_addr),
                           (unsigned int)ntohs(client_addr4->sin_port));
            }
            if (iperf_start_report() != IPERF_OK)
            {
                s_iperf_ctrl.finish = true;
                break;
            }
            report_started = true;
        }

        if (!iperf_udp_stats_record(&stats,
                                    &sequence_tracker,
                                    (uint64_t)packet_id,
                                    sent_time_us,
                                    arrival_time_us,
                                    (size_t)actual_recv))
        {
            iperf_publish_udp_stats(&stats);
            continue;
        }
        iperf_publish_udp_stats(&stats);
        (void)iperf_account_transfer((size_t)actual_recv);
        if (iperf_should_stop_transfer())
        {
            break;
        }
    }
}

static iperf_err_t iperf_run_tcp_server(void)
{
    int listen_socket = -1;
    int client_socket = -1;
    int opt = 1;
    int err = 0;
    iperf_err_t ret = IPERF_OK;
    struct sockaddr_in remote_addr = { 0 };
    socklen_t addr_len = sizeof(remote_addr);
#if IPERF_V6
    struct sockaddr_in6 listen_addr6 = { 0 };
#endif
    struct sockaddr_in listen_addr4 = { 0 };

    IPERF_GOTO_ON_FALSE((s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6 ||
                         s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4),
                        IPERF_FAIL,
                        exit,
                        TAG,
                        "Invalid address family");

#if IPERF_V6
    if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6)
    {
        /* Listen on all IPv6 addresses. */
        inet6_aton("::", &listen_addr6.sin6_addr);
        listen_addr6.sin6_family = AF_INET6;
        listen_addr6.sin6_port = htons(s_iperf_ctrl.cfg.sport);

        listen_socket = socket(AF_INET6, SOCK_STREAM, IPPROTO_IPV6);
        IPERF_GOTO_ON_FALSE((listen_socket >= 0),
                            IPERF_FAIL,
                            exit,
                            TAG,
                            "Unable to create socket: errno %d",
                            errno);

        setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        setsockopt(listen_socket, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));

        IPERF_LOGI(TAG, "Socket created");

        err = bind(listen_socket, (struct sockaddr *)&listen_addr6, sizeof(listen_addr6));
        IPERF_GOTO_ON_FALSE((err == 0),
                            IPERF_FAIL,
                            exit,
                            TAG,
                            "Socket unable to bind: errno %d, IPPROTO: %d",
                            errno,
                            AF_INET6);
        err = listen(listen_socket, 1);
        IPERF_GOTO_ON_FALSE((err == 0),
                            IPERF_FAIL,
                            exit,
                            TAG,
                            "Error occurred during listen: errno %d",
                            errno);

    }
    else
#endif
        if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4)
        {
            listen_addr4.sin_family = AF_INET;
            listen_addr4.sin_port = htons(s_iperf_ctrl.cfg.sport);
            listen_addr4.sin_addr.s_addr = s_iperf_ctrl.cfg.source_ip4;

            listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            IPERF_GOTO_ON_FALSE((listen_socket >= 0),
                                IPERF_FAIL,
                                exit,
                                TAG,
                                "Unable to create socket: errno %d",
                                errno);

            setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            if (iperf_configure_socket(listen_socket) != IPERF_OK)
            {
                ret = IPERF_FAIL;
                goto exit;
            }

            IPERF_LOGI(TAG, "Socket created");

            err = bind(listen_socket, (struct sockaddr *)&listen_addr4, sizeof(listen_addr4));
            IPERF_GOTO_ON_FALSE((err == 0),
                                IPERF_FAIL,
                                exit,
                                TAG,
                                "Socket unable to bind: errno %d, IPPROTO: %d",
                                errno,
                                AF_INET);

            err = listen(listen_socket, 5);
            IPERF_GOTO_ON_FALSE((err == 0),
                                IPERF_FAIL,
                                exit,
                                TAG,
                                "Error occurred during listen: errno %d",
                                errno);
        }

    if (iperf_set_recv_timeout(listen_socket) != IPERF_OK)
    {
        ret = IPERF_FAIL;
        goto exit;
    }

    client_socket = iperf_accept_client(listen_socket,
                                        (struct sockaddr *)&remote_addr,
                                        &addr_len);
    if (client_socket < 0)
    {
        if (!s_iperf_ctrl.finish)
        {
            ret = IPERF_FAIL;
        }
        goto exit;
    }
    IPERF_LOGI(TAG,
               "accept: %s,%u",
               inet_ntoa(remote_addr.sin_addr),
               (unsigned int)ntohs(remote_addr.sin_port));

    if (iperf_configure_socket(client_socket) != IPERF_OK)
    {
        ret = IPERF_FAIL;
        goto exit;
    }
    if (iperf_set_recv_timeout(client_socket) != IPERF_OK)
    {
        ret = IPERF_FAIL;
        goto exit;
    }

    s_iperf_ctrl.sockfd = client_socket;
    socket_recv_tcp(client_socket);
exit:
    if (client_socket != -1)
    {
        closesocket(client_socket);
    }

    if (listen_socket != -1)
    {
        shutdown(listen_socket, 0);
        closesocket(listen_socket);
        IPERF_LOGI(TAG, "TCP Socket server is closed.");
    }
    s_iperf_ctrl.finish = true;
    return ret;
}

static void iperf_tcp_dual_server_task(void *pvParameters)
{
    int listen_socket = -1;
    int client_socket = -1;
    int opt = 1;
    int err = 0;
    iperf_err_t ret = IPERF_OK;
    struct sockaddr_in remote_addr = { 0 };
    socklen_t addr_len = sizeof(remote_addr);
    struct sockaddr_storage listen_addr = { 0 };
    struct sockaddr_in listen_addr4 = { 0 };

    (void)pvParameters;
    (void)ret;
    if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4)
    {
        listen_addr4.sin_family = AF_INET;
        listen_addr4.sin_port = htons(s_iperf_ctrl.cfg.sport);
        listen_addr4.sin_addr.s_addr = s_iperf_ctrl.cfg.source_ip4;

        listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        IPERF_GOTO_ON_FALSE((listen_socket >= 0),
                            IPERF_FAIL,
                            exit,
                            TAG,
                            "Unable to create socket: errno %d",
                            errno);

        setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        if (iperf_configure_socket(listen_socket) != IPERF_OK)
        {
            ret = IPERF_FAIL;
            goto exit;
        }

        IPERF_LOGI(TAG, "Socket created");

        err = bind(listen_socket, (struct sockaddr *)&listen_addr4, sizeof(listen_addr4));
        IPERF_GOTO_ON_FALSE((err == 0),
                            IPERF_FAIL,
                            exit,
                            TAG,
                            "Socket unable to bind: errno %d, IPPROTO: %d",
                            errno,
                            AF_INET);

        err = listen(listen_socket, 5);
        IPERF_GOTO_ON_FALSE((err == 0),
                            IPERF_FAIL,
                            exit,
                            TAG,
                            "Error occurred during listen: errno %d",
                            errno);
        memcpy(&listen_addr, &listen_addr4, sizeof(listen_addr4));
    }

    if (iperf_set_recv_timeout(listen_socket) != IPERF_OK)
    {
        ret = IPERF_FAIL;
        goto exit;
    }

    client_socket = iperf_accept_client(listen_socket,
                                        (struct sockaddr *)&remote_addr,
                                        &addr_len);
    if (client_socket < 0)
    {
        if (!s_iperf_ctrl.finish)
        {
            ret = IPERF_FAIL;
        }
        goto exit;
    }
    IPERF_LOGI(TAG,
               "accept: %s,%u",
               inet_ntoa(remote_addr.sin_addr),
               (unsigned int)ntohs(remote_addr.sin_port));

    if (iperf_configure_socket(client_socket) != IPERF_OK)
    {
        ret = IPERF_FAIL;
        goto exit;
    }
    if (iperf_set_recv_timeout(client_socket) != IPERF_OK)
    {
        ret = IPERF_FAIL;
        goto exit;
    }

    socket_recv_dual(client_socket, listen_addr);
exit:
    if (client_socket != -1)
    {
        closesocket(client_socket);
    }

    if (listen_socket != -1)
    {
        shutdown(listen_socket, 0);
        closesocket(listen_socket);
        IPERF_LOGI(TAG, "TCP Socket server is closed.");
    }

    if (ret != IPERF_OK)
    {
        s_iperf_ctrl.finish = true;
    }
    s_iperf_ctrl.dual_task = NULL;
    aicwf_rtos_task_delete(NULL);
}

static iperf_err_t iperf_run_tcp_client(void)
{
    int client_socket = -1;
    int err = 0;
    int task_ret;
    iperf_err_t ret = IPERF_OK;
    struct sockaddr_storage dest_addr = { 0 };
#if IPERF_V6
    struct sockaddr_in6 dest_addr6 = { 0 };
#endif
    struct sockaddr_in dest_addr4 = { 0 };

    IPERF_GOTO_ON_FALSE((s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6 ||
                         s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4),
                        IPERF_FAIL,
                        exit,
                        TAG,
                        "Invalid address family");

    if (iperf_is_tcp_dual_client())
    {
        rtos_task_handle dual_task = NULL;

        task_ret = aicwf_rtos_task_create(iperf_tcp_dual_server_task,
                                          "iperf_dual_rx",
                                          0,
                                          IPERF_TRAFFIC_TASK_STACK_DEPTH,
                                          NULL,
                                          s_iperf_ctrl.cfg.traffic_task_priority,
                                          &dual_task);
        if (task_ret != AIC_RTOS_SUCCESS)
        {
            s_iperf_ctrl.dual_task = NULL;
            IPERF_LOGE(TAG, "create dual receive task failed");
            ret = IPERF_FAIL;
            goto exit;
        }
        s_iperf_ctrl.dual_task = dual_task;
        iperf_msleep(100);
        if (s_iperf_ctrl.finish)
        {
            ret = IPERF_FAIL;
            goto exit;
        }
    }
#if IPERF_V6
    if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6)
    {
        client_socket = socket(AF_INET6, SOCK_STREAM, IPPROTO_IPV6);
        IPERF_GOTO_ON_FALSE((client_socket >= 0),
                            IPERF_FAIL,
                            exit,
                            TAG,
                            "Unable to create socket: errno %d",
                            errno);
        if (iperf_configure_socket(client_socket) != IPERF_OK)
        {
            ret = IPERF_FAIL;
            goto exit;
        }

        inet6_aton(s_iperf_ctrl.cfg.destination_ip6, &dest_addr6.sin6_addr);
        dest_addr6.sin6_family = AF_INET6;
        dest_addr6.sin6_port = htons(s_iperf_ctrl.cfg.dport);

        err = connect(client_socket,
                      (struct sockaddr *)&dest_addr6,
                      sizeof(struct sockaddr_in6));
        IPERF_GOTO_ON_FALSE((err == 0),
                            IPERF_FAIL,
                            exit,
                            TAG,
                            "Socket unable to connect: errno %d",
                            errno);
        IPERF_LOGI(TAG, "Successfully connected");
        memcpy(&dest_addr, &dest_addr6, sizeof(dest_addr6));
    }
    else
#endif
        if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4)
        {
            client_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            IPERF_GOTO_ON_FALSE((client_socket >= 0),
                                IPERF_FAIL,
                                exit,
                                TAG,
                                "Unable to create socket: errno %d",
                                errno);
            if (iperf_configure_socket(client_socket) != IPERF_OK)
            {
                ret = IPERF_FAIL;
                goto exit;
            }

            dest_addr4.sin_family = AF_INET;
            dest_addr4.sin_port = htons(s_iperf_ctrl.cfg.dport);
            dest_addr4.sin_addr.s_addr = s_iperf_ctrl.cfg.destination_ip4;
            err = connect(client_socket,
                          (struct sockaddr *)&dest_addr4,
                          sizeof(struct sockaddr_in));
            IPERF_GOTO_ON_FALSE((err == 0),
                                IPERF_FAIL,
                                exit,
                                TAG,
                                "Socket unable to connect: errno %d",
                                errno);
            IPERF_LOGI(TAG, "Successfully connected");
            memcpy(&dest_addr, &dest_addr4, sizeof(dest_addr4));
        }

    s_iperf_ctrl.sockfd = client_socket;
    iperf_send_tcp(client_socket, dest_addr);
exit:
    if (client_socket != -1)
    {
        shutdown(client_socket, 0);
        closesocket(client_socket);
        IPERF_LOGI(TAG, "TCP Socket client is closed.");
    }
    s_iperf_ctrl.finish = true;
    return ret;
}

static iperf_err_t iperf_run_udp_server(void)
{
    int listen_socket = -1;
    int opt = 1;
    int err = 0;
    iperf_err_t ret = IPERF_OK;
#if IPERF_V6
    struct sockaddr_in6 listen_addr6 = { 0 };
#endif
    struct sockaddr_in listen_addr4 = { 0 };

    IPERF_GOTO_ON_FALSE((s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6 ||
                         s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4),
                        IPERF_FAIL,
                        exit,
                        TAG,
                        "Invalid address family");

#if IPERF_V6
    if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6)
    {
        /* Listen on all IPv6 addresses. */
        inet6_aton("::", &listen_addr6.sin6_addr);
        listen_addr6.sin6_family = AF_INET6;
        listen_addr6.sin6_port = htons(s_iperf_ctrl.cfg.sport);

        listen_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
        IPERF_GOTO_ON_FALSE((listen_socket >= 0),
                            IPERF_FAIL,
                            exit,
                            TAG,
                            "Unable to create socket: errno %d",
                            errno);
        IPERF_LOGI(TAG, "Socket created");

        setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        if (iperf_configure_socket(listen_socket) != IPERF_OK)
        {
            ret = IPERF_FAIL;
            goto exit;
        }

        err = bind(listen_socket,
                   (struct sockaddr *)&listen_addr6,
                   sizeof(struct sockaddr_in6));
        IPERF_GOTO_ON_FALSE((err == 0),
                            IPERF_FAIL,
                            exit,
                            TAG,
                            "Socket unable to bind: errno %d",
                            errno);
        IPERF_LOGI(TAG,
                   "Socket bound, port %u",
                   (unsigned int)ntohs(listen_addr6.sin6_port));

    }
    else
#endif
        if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4)
        {
            listen_addr4.sin_family = AF_INET;
            listen_addr4.sin_port = htons(s_iperf_ctrl.cfg.sport);
            listen_addr4.sin_addr.s_addr = s_iperf_ctrl.cfg.source_ip4;

            listen_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            IPERF_GOTO_ON_FALSE((listen_socket >= 0),
                                IPERF_FAIL,
                                exit,
                                TAG,
                                "Unable to create socket: errno %d",
                                errno);
            IPERF_LOGI(TAG, "Socket created");

            setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            if (iperf_configure_socket(listen_socket) != IPERF_OK)
            {
                ret = IPERF_FAIL;
                goto exit;
            }

            err = bind(listen_socket,
                       (struct sockaddr *)&listen_addr4,
                       sizeof(struct sockaddr_in));
            IPERF_GOTO_ON_FALSE((err == 0),
                                IPERF_FAIL,
                                exit,
                                TAG,
                                "Socket unable to bind: errno %d",
                                errno);
            IPERF_LOGI(TAG,
                       "Socket bound, port %u",
                       (unsigned int)ntohs(listen_addr4.sin_port));
        }

    if (iperf_set_recv_timeout(listen_socket) != IPERF_OK)
    {
        ret = IPERF_FAIL;
        goto exit;
    }

    s_iperf_ctrl.sockfd = listen_socket;
    iperf_recv_udp(listen_socket);
exit:
    if (listen_socket != -1)
    {
        shutdown(listen_socket, 0);
        closesocket(listen_socket);
    }
    IPERF_LOGI(TAG, "Udp socket server is closed.");
    s_iperf_ctrl.finish = true;
    return ret;
}

static iperf_err_t iperf_run_udp_client(void)
{
    int client_socket = -1;
    int opt = 1;
    uint64_t packet_count = 0;
    iperf_err_t ret = IPERF_OK;
    struct sockaddr_storage dest_addr = { 0 };
#if IPERF_V6
    struct sockaddr_in6 dest_addr6 = { 0 };
#endif
    struct sockaddr_in dest_addr4 = { 0 };

    IPERF_GOTO_ON_FALSE((s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6 ||
                         s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4),
                        IPERF_FAIL,
                        exit,
                        TAG,
                        "Invalid address family");

#if IPERF_V6
    if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6)
    {
        inet6_aton(s_iperf_ctrl.cfg.destination_ip6, &dest_addr6.sin6_addr);
        dest_addr6.sin6_family = AF_INET6;
        dest_addr6.sin6_port = htons(s_iperf_ctrl.cfg.dport);

        client_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_IPV6);
        IPERF_GOTO_ON_FALSE((client_socket >= 0),
                            IPERF_FAIL,
                            exit,
                            TAG,
                            "Unable to create socket: errno %d",
                            errno);
        IPERF_LOGI(TAG,
                   "Socket created, sending to %s:%d",
                   s_iperf_ctrl.cfg.destination_ip6,
                   s_iperf_ctrl.cfg.dport);

        setsockopt(client_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        if (iperf_configure_socket(client_socket) != IPERF_OK)
        {
            ret = IPERF_FAIL;
            goto exit;
        }
        memcpy(&dest_addr, &dest_addr6, sizeof(dest_addr6));
    }
    else
#endif
        if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4)
        {
            dest_addr4.sin_family = AF_INET;
            dest_addr4.sin_port = htons(s_iperf_ctrl.cfg.dport);
            dest_addr4.sin_addr.s_addr = s_iperf_ctrl.cfg.destination_ip4;

            client_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            IPERF_GOTO_ON_FALSE((client_socket >= 0),
                                IPERF_FAIL,
                                exit,
                                TAG,
                                "Unable to create socket: errno %d",
                                errno);
            IPERF_LOGI(TAG,
                       "Socket created, sending to %s:%u",
                       inet_ntoa(s_iperf_ctrl.cfg.destination_ip4),
                       (unsigned int)s_iperf_ctrl.cfg.dport);

            setsockopt(client_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            if (iperf_configure_socket(client_socket) != IPERF_OK)
            {
                ret = IPERF_FAIL;
                goto exit;
            }
            memcpy(&dest_addr, &dest_addr4, sizeof(dest_addr4));
        }

    s_iperf_ctrl.sockfd = client_socket;
    packet_count = iperf_send_udp_data(client_socket, dest_addr);
    iperf_mark_transfer_complete();
    if (!s_iperf_ctrl.finish && packet_count > 0U &&
            iperf_send_udp_fin(client_socket, dest_addr, packet_count) != IPERF_OK)
    {
        ret = IPERF_FAIL;
    }
exit:
    if (client_socket != -1)
    {
        shutdown(client_socket, 0);
        closesocket(client_socket);
    }
    s_iperf_ctrl.finish = true;
    IPERF_LOGI(TAG, "UDP Socket client is closed");
    return ret;
}

static void iperf_task_traffic(void *arg)
{
    (void)arg;

    if (iperf_is_udp_client())
    {
        iperf_run_udp_client();
    }
    else if (iperf_is_udp_server())
    {
        iperf_run_udp_server();
    }
    else if (iperf_is_tcp_client())
    {
        iperf_run_tcp_client();
    }
    else
    {
        iperf_run_tcp_server();
    }

    iperf_mark_transfer_complete();
    s_iperf_ctrl.finish = true;
    iperf_notify_report_task();
    while (s_iperf_ctrl.report_task != NULL || s_iperf_ctrl.dual_task != NULL)
    {
        iperf_msleep(1);
    }

    iperf_print_final_report();

    if (s_iperf_ctrl.buffer)
    {
        aicwf_rtos_free(s_iperf_ctrl.buffer);
        s_iperf_ctrl.buffer = NULL;
    }
    printf("iperf exit\r\n");
    s_iperf_is_running = false;
    aicwf_rtos_task_delete(NULL);
}

static uint32_t iperf_get_buffer_len(void)
{
    if (iperf_is_udp_client())
    {
        return (s_iperf_ctrl.cfg.len_buf == 0 ?
                IPERF_UDP_TX_LEN : s_iperf_ctrl.cfg.len_buf);
    }
    else if (iperf_is_udp_server())
    {
        return (s_iperf_ctrl.cfg.len_buf == 0 ?
                IPERF_UDP_RX_LEN : s_iperf_ctrl.cfg.len_buf);
    }
    else if (iperf_is_tcp_client())
    {
        return (s_iperf_ctrl.cfg.len_buf == 0 ?
                IPERF_TCP_TX_LEN : s_iperf_ctrl.cfg.len_buf);
    }
    else
    {
        return (s_iperf_ctrl.cfg.len_buf == 0 ?
                IPERF_TCP_RX_LEN : s_iperf_ctrl.cfg.len_buf);
    }
    return 0;
}

static void iperf_print_header(const iperf_cfg_t *cfg)
{
    printf("------------------------------------------------------------\r\n");
    if (iperf_is_udp_server())
    {
        printf("Server listening on UDP port %u\r\n", (unsigned int)cfg->sport);
    }
    else if (iperf_is_tcp_server())
    {
        printf("Server listening on TCP port %u\r\n", (unsigned int)cfg->sport);
    }
    else if (iperf_is_udp_client())
    {
        uint32_t datagram_len = cfg->len_buf == 0 ? IPERF_UDP_TX_LEN : cfg->len_buf;

        printf("Client connecting to %s, UDP port %u\r\n",
               inet_ntoa(cfg->destination_ip4),
               (unsigned int)cfg->dport);
        if (cfg->end_by_bytes)
        {
            printf("Sending %llu bytes in %" PRIu32 " byte datagrams\r\n",
                   (unsigned long long)cfg->num_bytes,
                   datagram_len);
        }
        else
        {
            printf("Sending %" PRIu32 " byte datagrams\r\n", datagram_len);
        }
    }
    else if (iperf_is_tcp_client())
    {
        printf("Client connecting to %s, TCP port %u\r\n",
               inet_ntoa(cfg->destination_ip4),
               (unsigned int)cfg->dport);
    }
    printf("------------------------------------------------------------\r\n");
}

static iperf_err_t iperf_engine_start(const iperf_cfg_t *cfg)
{
    rtos_task_handle traffic_task = NULL;
    int ret;

    if (!cfg)
    {
        return IPERF_FAIL;
    }

    if (s_iperf_is_running)
    {
        IPERF_LOGW(TAG, "iperf is running");
        return IPERF_FAIL;
    }

    memset(&s_iperf_ctrl, 0, sizeof(s_iperf_ctrl));
    memcpy(&s_iperf_ctrl.cfg, cfg, sizeof(*cfg));
    s_iperf_is_running = true;
    s_iperf_ctrl.finish = false;
    s_iperf_ctrl.sockfd = -1;
    s_iperf_ctrl.buffer_len = iperf_get_buffer_len();
    if (s_iperf_ctrl.buffer_len == 0U || s_iperf_ctrl.buffer_len > INT_MAX)
    {
        IPERF_LOGE(TAG, "invalid buffer length: %" PRIu32, s_iperf_ctrl.buffer_len);
        s_iperf_ctrl.finish = true;
        s_iperf_is_running = false;
        return IPERF_FAIL;
    }
    s_iperf_ctrl.buffer = (uint8_t *)aicwf_rtos_malloc(s_iperf_ctrl.buffer_len);
    if (!s_iperf_ctrl.buffer)
    {
        IPERF_LOGE(TAG, "create buffer: not enough memory");
        s_iperf_ctrl.finish = true;
        s_iperf_is_running = false;
        return IPERF_FAIL;
    }
    memset(s_iperf_ctrl.buffer, 0, s_iperf_ctrl.buffer_len);
    ret = aicwf_rtos_task_create(iperf_task_traffic,
                                 IPERF_TRAFFIC_TASK_NAME,
                                 0,
                                 IPERF_TRAFFIC_TASK_STACK_DEPTH,
                                 NULL,
                                 s_iperf_ctrl.cfg.traffic_task_priority,
                                 &traffic_task);
    if (ret != AIC_RTOS_SUCCESS)
    {
        IPERF_LOGE(TAG, "create task %s failed", IPERF_TRAFFIC_TASK_NAME);
        aicwf_rtos_free(s_iperf_ctrl.buffer);
        s_iperf_ctrl.buffer = NULL;
        s_iperf_ctrl.finish = true;
        s_iperf_is_running = false;
        return IPERF_FAIL;
    }
    if (cfg->window_size > 0U)
    {
        IPERF_LOGI(TAG, "-w controls SO_RCVBUF only; lwIP does not implement SO_SNDBUF");
    }
    iperf_print_header(cfg);
    return IPERF_OK;
}

static iperf_err_t iperf_engine_stop(void)
{
    if (s_iperf_is_running)
    {
        s_iperf_ctrl.finish = true;
        iperf_notify_report_task();
    }
    else
    {
        IPERF_LOGW(TAG, "iperf is not running");
    }

    return IPERF_OK;
}

static bool iperf_parse_u32(const char *text,
                            uint32_t min_value,
                            uint32_t max_value,
                            uint32_t *value_out)
{
    char *end;
    unsigned long value;

    if (text == NULL || text[0] == '\0' || text[0] == '-' || value_out == NULL)
    {
        return false;
    }

    errno = 0;
    value = strtoul(text, &end, 10);
    if (errno == ERANGE || end == text || *end != '\0' ||
            value < min_value || value > max_value)
    {
        return false;
    }

    *value_out = (uint32_t)value;
    return true;
}

static bool iperf_parse_unit_multiplier(char suffix, uint64_t *multiplier_out)
{
    switch (suffix)
    {
    case '\0':
        *multiplier_out = 1U;
        return true;
    case 'K':
        *multiplier_out = 1024ULL;
        return true;
    case 'M':
        *multiplier_out = 1024ULL * 1024ULL;
        return true;
    case 'G':
        *multiplier_out = 1024ULL * 1024ULL * 1024ULL;
        return true;
    case 'k':
        *multiplier_out = 1000ULL;
        return true;
    case 'm':
        *multiplier_out = 1000ULL * 1000ULL;
        return true;
    case 'g':
        *multiplier_out = 1000ULL * 1000ULL * 1000ULL;
        return true;
    default:
        return false;
    }
}

static bool iperf_is_pps_suffix(const char *suffix)
{
    return suffix != NULL &&
           tolower((unsigned char)suffix[0]) == 'p' &&
           tolower((unsigned char)suffix[1]) == 'p' &&
           tolower((unsigned char)suffix[2]) == 's' &&
           suffix[3] == '\0';
}

static bool iperf_parse_scaled_u64(const char *text,
                                   bool allow_pps,
                                   uint64_t max_value,
                                   uint64_t *value_out,
                                   iperf_rate_unit_t *rate_unit_out)
{
    char *suffix;
    double number;
    double scaled_number;
    uint64_t multiplier;

    if (text == NULL || text[0] == '\0' || value_out == NULL ||
            (!isdigit((unsigned char)text[0]) && text[0] != '.'))
    {
        return false;
    }

    errno = 0;
    number = strtod(text, &suffix);
    if (errno == ERANGE || suffix == text || number < 0.0)
    {
        return false;
    }
    if (allow_pps && iperf_is_pps_suffix(suffix))
    {
        multiplier = 1U;
        if (rate_unit_out != NULL)
        {
            *rate_unit_out = IPERF_RATE_PACKETS_PER_SECOND;
        }
    }
    else
    {
        if (!iperf_parse_unit_multiplier(*suffix, &multiplier) ||
                (*suffix != '\0' && suffix[1] != '\0'))
        {
            return false;
        }
        if (rate_unit_out != NULL)
        {
            *rate_unit_out = IPERF_RATE_BITS_PER_SECOND;
        }
    }

    if (number > (double)max_value / (double)multiplier)
    {
        return false;
    }
    scaled_number = number * (double)multiplier;
    if (scaled_number < 1.0 || scaled_number > (double)max_value)
    {
        return false;
    }

    *value_out = (uint64_t)scaled_number;
    return true;
}

#define ML(s) s NL
#define IPERF_USAGE                                                   \
    ML("Usage: iperf (-s | -c server_addr) [options]")               \
    ML(" -c server_addr: run in client mode")                        \
    ML(" -s: run in server mode")                                    \
    ML(" -u: use UDP instead of TCP")                                \
    ML(" -p port: specify port")                                     \
    ML(" -l length: set read/write buffer size (K/M/G units supported)") \
    ML(" -i interval: seconds between bandwidth reports")            \
    ML(" -t time: time in seconds to run")                           \
    ML(" -n bytes: send/receive bytes instead of using -t")          \
    ML(" -b bandwidth: limit TCP/UDP rate ([KMG] bit/s or UDP pps)") \
    ML(" -w window: set receive window (K/M/G units supported)")    \
    ML(" -S tos: IP type of service")                                \
    ML(" -P priority: FreeRTOS traffic task priority (not parallel streams)") \
    ML(" -d: TCP client dual mode")                                  \
    ML(" -a: abort running iperf")                                   \
    ML(" -h: show this help")                                        \
    ML(" -v: show version")

static void iperf_print_usage(void)
{
    printf("%s", IPERF_USAGE);
}

static bool iperf_parse_option(const char *option_name,
                               const char *text,
                               uint32_t min_value,
                               uint32_t max_value,
                               uint32_t *value_out)
{
    if (iperf_parse_u32(text, min_value, max_value, value_out))
    {
        return true;
    }

    printf("iperf: invalid %s: %s\r\n", option_name, text != NULL ? text : "");
    return false;
}

/* ---- Compact reentrant getopt (replaces newlib getopt_r) ----
 * Same semantics: with a leading ':' in the optstring, a missing argument
 * returns ':' and an unknown option returns '?'; supports both -c1.2.3.4
 * attached and -c 1.2.3.4 separated argument forms. */
typedef struct
{
    const char *optarg;
    int optind;
    int optopt;
    const char *next;
} getopt_data;

#define GETOPT_DATA_INITIALIZER { NULL, 1, 0, NULL }

static int getopt_r(int argc, char **argv, const char *optstring, getopt_data *d)
{
    const char *spec;

    if (d->next == NULL || *d->next == '\0')
    {
        const char *arg;

        if (d->optind >= argc)
        {
            return -1;
        }
        arg = argv[d->optind];
        if (arg[0] != '-' || arg[1] == '\0')
        {
            return -1;      /* non-option argument, stop parsing */
        }
        d->optind++;
        if (arg[1] == '-' && arg[2] == '\0')
        {
            return -1;      /* "--" terminates */
        }
        d->next = arg + 1;
    }

    d->optopt = (unsigned char) * d->next++;
    if (d->optopt == ':')
    {
        return '?';
    }
    spec = strchr(optstring, d->optopt);
    if (spec == NULL)
    {
        return '?';
    }
    if (spec[1] == ':')
    {
        if (*d->next != '\0')
        {
            d->optarg = d->next;
            d->next = NULL;
        }
        else if (d->optind < argc)
        {
            d->optarg = argv[d->optind++];
            d->next = NULL;
        }
        else
        {
            d->optarg = NULL;
            return (optstring[0] == ':') ? ':' : '?';
        }
    }
    else
    {
        d->optarg = NULL;
    }
    return d->optopt;
}

/* Parse command line into cfg_out; 0=parsed OK, -1=bad args, -2=handled in place (stop/help/version) */
static int iperf_cmd(int argc, char **argv, iperf_cfg_t *cfg_out)
{
    int opt;
    getopt_data opt_env = GETOPT_DATA_INITIALIZER;
    bool is_client = false;
    bool is_server = false;
    bool use_udp = false;
    bool abort_requested = false;
    bool dual_mode = false;
    uint32_t port = IPERF_DEFAULT_PORT;
    uint32_t buffer_len = 0;
    uint32_t interval = IPERF_DEFAULT_INTERVAL;
    uint32_t duration = IPERF_DEFAULT_TIME;
    uint32_t tos = 0;
    uint32_t priority = IPERF_TRAFFIC_TASK_PRIORITY;
    uint64_t amount_bytes = 0;
    uint64_t rate = 0;
    uint64_t parsed_value;
    iperf_rate_unit_t rate_unit = IPERF_RATE_BITS_PER_SECOND;
    bool end_by_bytes = false;
    bool rate_set = false;
    ip4_addr_t destination_ip4 = { 0 };
    iperf_cfg_t cfg = { 0 };

    while ((opt = getopt_r(argc, argv, ":c:sup:l:i:t:n:b:w:S:P:adhv", &opt_env)) != -1)
    {
        switch (opt)
        {
        case 'c':
            if (!ip4addr_aton(opt_env.optarg, &destination_ip4))
            {
                if (strchr(opt_env.optarg, ':') != NULL)
                {
                    printf("iperf: IPv6 is not supported\r\n");
                }
                else
                {
                    printf("iperf: invalid IPv4 address: %s\r\n", opt_env.optarg);
                }
                iperf_print_usage();
                return -1;
            }
            is_client = true;
            break;
        case 's':
            is_server = true;
            break;
        case 'u':
            use_udp = true;
            break;
        case 'p':
            if (!iperf_parse_option("port", opt_env.optarg, 1, UINT16_MAX, &port))
            {
                iperf_print_usage();
                return -1;
            }
            break;
        case 'l':
            if (strcmp(opt_env.optarg, "0") == 0)
            {
                buffer_len = 0;
            }
            else if (!iperf_parse_scaled_u64(opt_env.optarg,
                                             false,
                                             INT_MAX,
                                             &parsed_value,
                                             NULL))
            {
                printf("iperf: invalid buffer length: %s\r\n", opt_env.optarg);
                iperf_print_usage();
                return -1;
            }
            else
            {
                buffer_len = (uint32_t)parsed_value;
            }
            break;
        case 'i':
            if (!iperf_parse_option("report interval",
                                    opt_env.optarg,
                                    1,
                                    IPERF_MAX_TIME_SECONDS,
                                    &interval))
            {
                iperf_print_usage();
                return -1;
            }
            break;
        case 't':
            if (!iperf_parse_option("run time",
                                    opt_env.optarg,
                                    1,
                                    IPERF_MAX_TIME_SECONDS,
                                    &duration))
            {
                iperf_print_usage();
                return -1;
            }
            break;
        case 'S':
            if (!iperf_parse_option("TOS", opt_env.optarg, 0, UINT8_MAX, &tos))
            {
                iperf_print_usage();
                return -1;
            }
            break;
        case 'n':
            if (!iperf_parse_scaled_u64(opt_env.optarg,
                                        false,
                                        INT32_MAX,
                                        &amount_bytes,
                                        NULL))
            {
                printf("iperf: invalid transfer amount: %s\r\n", opt_env.optarg);
                iperf_print_usage();
                return -1;
            }
            end_by_bytes = true;
            break;
        case 'b':
            if (!iperf_parse_scaled_u64(opt_env.optarg,
                                        true,
                                        UINT32_MAX,
                                        &rate,
                                        &rate_unit))
            {
                printf("iperf: invalid bandwidth: %s\r\n", opt_env.optarg);
                iperf_print_usage();
                return -1;
            }
            rate_set = true;
            break;
        case 'w':
            if (!iperf_parse_scaled_u64(opt_env.optarg,
                                        false,
                                        INT_MAX,
                                        &parsed_value,
                                        NULL))
            {
                printf("iperf: invalid window size: %s\r\n", opt_env.optarg);
                iperf_print_usage();
                return -1;
            }
            cfg.window_size = (uint32_t)parsed_value;
            break;
        case 'P':
            if (!iperf_parse_option("task priority",
                                    opt_env.optarg,
                                    0,
                                    IPERF_MAX_TRAFFIC_TASK_PRIORITY,
                                    &priority))
            {
                iperf_print_usage();
                return -1;
            }
            break;
        case 'd':
            dual_mode = true;
            break;
        case 'a':
            abort_requested = true;
            break;
        case 'h':
            iperf_print_usage();
            return -2;
        case 'v':
            printf("iperf version 2.0.9 (FreeRTOS/lwIP)\r\n");
            return -2;
        case ':':
            printf("iperf: option -%c requires an argument\r\n", opt_env.optopt);
            iperf_print_usage();
            return -1;
        case '?':
            if (opt_env.optopt == 'r')
            {
                printf("iperf: -r tradeoff mode is not supported\r\n");
            }
            else if (opt_env.optopt == 'V')
            {
                printf("iperf: IPv6 is not supported\r\n");
            }
            else if (opt_env.optopt == 'f' || opt_env.optopt == 'y')
            {
                printf("iperf: CSV and alternate output formats are not supported\r\n");
            }
            else if (opt_env.optopt == 'F' || opt_env.optopt == 'o')
            {
                printf("iperf: file input/output is not supported\r\n");
            }
            else
            {
                printf("iperf: unsupported option -%c\r\n", opt_env.optopt);
            }
            iperf_print_usage();
            return -1;
        default:
            printf("iperf: invalid command line\r\n");
            iperf_print_usage();
            return -1;
        }
    }

    if (opt_env.optind < argc)
    {
        printf("iperf: unexpected argument: %s\r\n", argv[opt_env.optind]);
        iperf_print_usage();
        return -1;
    }

    if (abort_requested)
    {
        iperf_engine_stop();
        return -2;
    }
    if (is_client == is_server)
    {
        printf("iperf: exactly one of -c and -s is required\r\n");
        iperf_print_usage();
        return -1;
    }
    if (dual_mode && (!is_client || use_udp))
    {
        printf("iperf: -d is only valid for a TCP client\r\n");
        iperf_print_usage();
        return -1;
    }
    if (rate_set && rate_unit == IPERF_RATE_PACKETS_PER_SECOND && !use_udp)
    {
        printf("iperf: pps bandwidth is only valid for UDP\r\n");
        iperf_print_usage();
        return -1;
    }
    if (use_udp && buffer_len > 0U &&
            buffer_len < sizeof(iperf_udp_legacy_datagram_wire_t))
    {
        printf("iperf: UDP buffer length must be at least %u bytes\r\n",
               (unsigned int)sizeof(iperf_udp_legacy_datagram_wire_t));
        iperf_print_usage();
        return -1;
    }
    if (is_client && use_udp)
    {
        uint32_t udp_buffer_len = buffer_len == 0U ? IPERF_UDP_TX_LEN : buffer_len;

        if (udp_buffer_len < IPERF_UDP_MIN_CLIENT_DATAGRAM_LEN)
        {
            printf("iperf: UDP client buffer length must be at least %u bytes\r\n",
                   (unsigned int)IPERF_UDP_MIN_CLIENT_DATAGRAM_LEN);
            iperf_print_usage();
            return -1;
        }
        if (end_by_bytes && amount_bytes < IPERF_UDP_MIN_CLIENT_DATAGRAM_LEN)
        {
            printf("iperf: UDP transfer amount must be at least %u bytes\r\n",
                   (unsigned int)IPERF_UDP_MIN_CLIENT_DATAGRAM_LEN);
            iperf_print_usage();
            return -1;
        }
        if (end_by_bytes && amount_bytes > udp_buffer_len &&
                (amount_bytes % udp_buffer_len) < sizeof(iperf_udp_legacy_datagram_wire_t))
        {
            printf("iperf: UDP transfer amount leaves a datagram shorter than %u bytes\r\n",
                   (unsigned int)sizeof(iperf_udp_legacy_datagram_wire_t));
            iperf_print_usage();
            return -1;
        }
    }

    cfg.type = IPERF_IP_TYPE_IPV4;
    if (is_client)
    {
        cfg.destination_ip4 = ip4_addr_get_u32(&destination_ip4);
        cfg.flag |= IPERF_FLAG_CLIENT;
    }
    else
    {
        cfg.flag |= IPERF_FLAG_SERVER;
    }
    if (use_udp)
    {
        cfg.flag |= IPERF_FLAG_UDP;
    }
    else
    {
        cfg.flag |= IPERF_FLAG_TCP;
    }
    if (dual_mode)
    {
        cfg.flag |= IPERF_FLAG_DUAL;
    }

    cfg.len_buf = buffer_len;
    cfg.sport = (uint16_t)port;
    cfg.dport = (uint16_t)port;
    cfg.interval = interval;
    cfg.time = duration;
    cfg.tos = (uint8_t)tos;
    cfg.num_bytes = amount_bytes;
    cfg.end_by_bytes = end_by_bytes;
    cfg.rate = rate_set ? rate : (use_udp ? IPERF_DEFAULT_UDP_RATE_BITS_PER_SECOND : 0U);
    cfg.rate_unit = rate_set ? rate_unit : IPERF_RATE_BITS_PER_SECOND;
    cfg.rate_set = rate_set || use_udp;
    cfg.traffic_task_priority = (uint8_t)priority;

    *cfg_out = cfg;
    return 0;
}

/* ============================================================
 * Public API glue layer (wifi_port_cmd.c: iwpriv iperf start/stop)
 * Single instance, handle is always 0; iperf_parse_argv return values:
 *   0  = arguments parsed, config staged for iperf_start
 *   -1 = argument error (usage already printed)
 *   -2 = handled in place (-Q/-h/-v/-a), no need to start tasks
 * ============================================================ */
static iperf_cfg_t s_pending_cfg;

int iperf_parse_argv(int argc, char *argv[])
{
    memset(&s_pending_cfg, 0, sizeof(s_pending_cfg));

    /* Legacy command format: "-Q" argument of iperf stop */
    if (argc >= 1 && strcmp(argv[0], "-Q") == 0)
    {
        iperf_engine_stop();
        return -2;
    }

    if (argc == 1)
    {
        iperf_print_usage();
        return -1;
    }

    return iperf_cmd(argc, argv, &s_pending_cfg);
}

int iperf_start(struct netif *nif, int handle)
{
    (void)nif;

    if (handle != 0)
    {
        IPERF_LOGW(TAG, "only handle 0 is supported");
        return -1;
    }

    return (iperf_engine_start(&s_pending_cfg) == IPERF_OK) ? 0 : -1;
}

int iperf_stop(char *arg)
{
    (void)arg;

    return (iperf_engine_stop() == IPERF_OK) ? 0 : -1;
}

int iperf_handle_free(int handle)
{
    (void)handle;

    return 0;
}

int iperf_handle_start(struct netif *nif, int handle)
{
    return iperf_start(nif, handle);
}

/* ============================================================
 * UDP pacer / statistics / sequence tracking (formerly cmds_iperf_internal.c,
 * merged into this file because the platform project only builds a single
 * file; contents are identical to t3x)
 * ============================================================ */
static uint64_t iperf_saturating_multiply_u64(uint64_t left, uint64_t right)
{
    if (left == 0 || right == 0)
    {
        return 0;
    }
    if (left > UINT64_MAX / right)
    {
        return UINT64_MAX;
    }
    return left * right;
}

static uint64_t iperf_saturating_add_u64(uint64_t left, uint64_t right)
{
    if (left > UINT64_MAX - right)
    {
        return UINT64_MAX;
    }
    return left + right;
}

static int64_t iperf_saturating_add_i64(int64_t left, int64_t right)
{
    if (right > 0 && left > INT64_MAX - right)
    {
        return INT64_MAX;
    }
    if (right < 0 && left < INT64_MIN - right)
    {
        return INT64_MIN;
    }
    return left + right;
}

static uint64_t iperf_pacer_packet_units(size_t packet_size_bytes, uint32_t ticks_per_second)
{
    return iperf_saturating_multiply_u64(
               iperf_saturating_multiply_u64((uint64_t)packet_size_bytes, 8U),
               ticks_per_second);
}

void iperf_pacer_init(iperf_pacer_t *pacer,
                      uint64_t rate_bits_per_second,
                      size_t packet_size_bytes,
                      uint32_t ticks_per_second,
                      uint32_t now_tick)
{
    uint64_t packet_units;
    uint64_t rate_burst_units;
    uint64_t packet_burst_units;

    if (pacer == NULL)
    {
        return;
    }

    memset(pacer, 0, sizeof(*pacer));
    pacer->last_tick = now_tick;
    if (rate_bits_per_second == 0 || ticks_per_second == 0)
    {
        return;
    }

    packet_units = iperf_pacer_packet_units(packet_size_bytes, ticks_per_second);
    rate_burst_units = iperf_saturating_multiply_u64(rate_bits_per_second,
                       IPERF_PACER_BURST_TICKS);
    packet_burst_units = iperf_saturating_multiply_u64(packet_units,
                         IPERF_PACER_MIN_BURST_PACKETS);

    pacer->rate_bits_per_second = rate_bits_per_second;
    pacer->capacity_units = rate_burst_units > packet_burst_units ?
                            rate_burst_units : packet_burst_units;
    pacer->token_units = pacer->capacity_units;
    pacer->ticks_per_second = ticks_per_second;
    pacer->enabled = true;
}

bool iperf_pacer_try_consume(iperf_pacer_t *pacer,
                             size_t packet_size_bytes,
                             uint32_t now_tick,
                             uint32_t *wait_ticks_out)
{
    uint32_t elapsed_ticks;
    uint64_t packet_units;
    uint64_t missing_units;
    uint64_t wait_ticks;

    if (wait_ticks_out != NULL)
    {
        *wait_ticks_out = 0;
    }
    if (pacer == NULL || !pacer->enabled)
    {
        return true;
    }

    elapsed_ticks = now_tick - pacer->last_tick;
    if (elapsed_ticks > 0)
    {
        uint64_t available_units;
        uint64_t added_units;

        if (pacer->token_units > pacer->capacity_units)
        {
            pacer->token_units = pacer->capacity_units;
        }
        available_units = pacer->capacity_units - pacer->token_units;
        added_units = iperf_saturating_multiply_u64(elapsed_ticks,
                      pacer->rate_bits_per_second);
        if (added_units >= available_units)
        {
            pacer->token_units = pacer->capacity_units;
        }
        else
        {
            pacer->token_units = iperf_saturating_add_u64(pacer->token_units,
                                 added_units);
        }
        pacer->last_tick = now_tick;
    }

    packet_units = iperf_pacer_packet_units(packet_size_bytes, pacer->ticks_per_second);
    if (pacer->token_units >= packet_units)
    {
        pacer->token_units -= packet_units;
        return true;
    }

    missing_units = packet_units - pacer->token_units;
    wait_ticks = missing_units / pacer->rate_bits_per_second;
    if ((missing_units % pacer->rate_bits_per_second) != 0U)
    {
        wait_ticks++;
    }
    if (wait_ticks == 0)
    {
        wait_ticks = 1;
    }
    else if (wait_ticks > UINT32_MAX)
    {
        wait_ticks = UINT32_MAX;
    }
    if (wait_ticks_out != NULL)
    {
        *wait_ticks_out = (uint32_t)wait_ticks;
    }
    return false;
}

uint32_t iperf_pacer_backoff_ticks(uint32_t failure_streak)
{
    uint32_t shift;
    uint32_t backoff_ticks;

    if (failure_streak == 0)
    {
        return 1;
    }

    shift = failure_streak - 1U;
    if (shift > 3U)
    {
        shift = 3U;
    }
    backoff_ticks = 1U << shift;
    return backoff_ticks > IPERF_PACER_MAX_BACKOFF_TICKS ?
           IPERF_PACER_MAX_BACKOFF_TICKS : backoff_ticks;
}

void iperf_udp_stats_init(iperf_udp_stats_t *stats)
{
    if (stats != NULL)
    {
        memset(stats, 0, sizeof(*stats));
    }
}

void iperf_udp_sequence_tracker_init(iperf_udp_sequence_tracker_t *tracker)
{
    if (tracker != NULL)
    {
        memset(tracker, 0, sizeof(*tracker));
    }
}

static void iperf_udp_sequence_clear(iperf_udp_sequence_tracker_t *tracker,
                                     uint64_t packet_id)
{
    uint32_t bit_index = (uint32_t)(packet_id % IPERF_UDP_SEQUENCE_WINDOW_BITS);
    uint32_t word_index = bit_index / 32U;
    uint32_t bit_mask = (uint32_t)1U << (bit_index % 32U);

    tracker->received_bitmap[word_index] &= ~bit_mask;
}

static void iperf_udp_sequence_advance(iperf_udp_sequence_tracker_t *tracker,
                                       uint64_t packet_id)
{
    uint64_t new_window_start;
    uint64_t evicted_count;
    uint64_t evicted_packet_id;

    if (packet_id - tracker->window_start_packet_id <
            IPERF_UDP_SEQUENCE_WINDOW_BITS)
    {
        return;
    }

    new_window_start = packet_id - IPERF_UDP_SEQUENCE_WINDOW_BITS + 1U;
    evicted_count = new_window_start - tracker->window_start_packet_id;
    if (evicted_count >= IPERF_UDP_SEQUENCE_WINDOW_BITS)
    {
        memset(tracker->received_bitmap, 0, sizeof(tracker->received_bitmap));
    }
    else
    {
        for (evicted_packet_id = tracker->window_start_packet_id;
                evicted_packet_id < new_window_start;
                evicted_packet_id++)
        {
            iperf_udp_sequence_clear(tracker, evicted_packet_id);
        }
    }
    tracker->window_start_packet_id = new_window_start;
}

static bool iperf_udp_sequence_mark_received(iperf_udp_sequence_tracker_t *tracker,
        uint64_t packet_id)
{
    uint32_t bit_index;
    uint32_t word_index;
    uint32_t bit_mask;

    if (packet_id < tracker->window_start_packet_id)
    {
        return false;
    }

    iperf_udp_sequence_advance(tracker, packet_id);
    bit_index = (uint32_t)(packet_id % IPERF_UDP_SEQUENCE_WINDOW_BITS);
    word_index = bit_index / 32U;
    bit_mask = (uint32_t)1U << (bit_index % 32U);
    if ((tracker->received_bitmap[word_index] & bit_mask) != 0)
    {
        return false;
    }

    tracker->received_bitmap[word_index] |= bit_mask;
    return true;
}

bool iperf_udp_stats_record(iperf_udp_stats_t *stats,
                            iperf_udp_sequence_tracker_t *tracker,
                            uint64_t packet_id,
                            uint64_t sent_time_us,
                            uint64_t arrival_time_us,
                            size_t packet_size_bytes)
{
    int64_t transit_us;

    if (stats == NULL || tracker == NULL)
    {
        return false;
    }
    if (!iperf_udp_sequence_mark_received(tracker, packet_id))
    {
        stats->duplicate_datagrams++;
        return false;
    }

    if (!stats->sequence_initialized)
    {
        stats->next_packet_id = 0;
        stats->first_arrival_us = arrival_time_us;
        stats->sequence_initialized = true;
    }

    if (packet_id >= stats->next_packet_id)
    {
        stats->lost_datagrams += packet_id - stats->next_packet_id;
        stats->next_packet_id = packet_id + 1U;
    }
    else
    {
        stats->out_of_order_datagrams++;
        if (stats->lost_datagrams > 0)
        {
            stats->lost_datagrams--;
        }
    }

    stats->received_bytes += packet_size_bytes;
    stats->received_datagrams++;
    stats->last_arrival_us = arrival_time_us;

    if (arrival_time_us > (uint64_t)INT64_MAX || sent_time_us > (uint64_t)INT64_MAX)
    {
        transit_us = arrival_time_us >= sent_time_us ? INT64_MAX : INT64_MIN;
    }
    else
    {
        transit_us = (int64_t)arrival_time_us - (int64_t)sent_time_us;
    }
    if (stats->transit_initialized)
    {
        double delta_us = (double)transit_us - (double)stats->previous_transit_us;
        double delta_mean;
        double delta_mean_after;

        if (delta_us < 0)
        {
            delta_us = -delta_us;
        }
        stats->jitter_us += ((double)delta_us - stats->jitter_us) / 16.0;

        stats->transit_count++;
        stats->min_transit_us = transit_us < stats->min_transit_us ?
                                transit_us : stats->min_transit_us;
        stats->max_transit_us = transit_us > stats->max_transit_us ?
                                transit_us : stats->max_transit_us;
        stats->sum_transit_us = iperf_saturating_add_i64(stats->sum_transit_us,
                                transit_us);
        delta_mean = (double)transit_us - stats->mean_transit_us;
        stats->mean_transit_us += delta_mean / (double)stats->transit_count;
        delta_mean_after = (double)transit_us - stats->mean_transit_us;
        stats->m2_transit_us += delta_mean * delta_mean_after;
        if (stats->transit_count > 1U)
        {
            stats->variance_transit_us = stats->m2_transit_us /
                                         (double)(stats->transit_count - 1U);
        }
    }
    else
    {
        stats->transit_initialized = true;
        stats->min_transit_us = transit_us;
        stats->max_transit_us = transit_us;
        stats->sum_transit_us = transit_us;
        stats->mean_transit_us = (double)transit_us;
        stats->transit_count = 1U;
    }
    stats->previous_transit_us = transit_us;
    return true;
}

void iperf_udp_stats_finish(iperf_udp_stats_t *stats, uint64_t expected_datagrams)
{
    if (stats == NULL)
    {
        return;
    }

    if (!stats->sequence_initialized)
    {
        stats->sequence_initialized = true;
    }
    if (expected_datagrams > stats->next_packet_id)
    {
        stats->lost_datagrams += expected_datagrams - stats->next_packet_id;
        stats->next_packet_id = expected_datagrams;
    }
}

double iperf_udp_stats_loss_percent(const iperf_udp_stats_t *stats)
{
    if (stats == NULL || stats->next_packet_id == 0)
    {
        return 0.0;
    }

    return (double)stats->lost_datagrams * 100.0 / stats->next_packet_id;
}
