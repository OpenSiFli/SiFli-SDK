/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IPERF_INTERNAL_H
#define IPERF_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define IPERF_HEADER_VERSION_1          ((uint32_t)0x80000000UL)
#define IPERF_HEADER_EXTEND             ((uint32_t)0x40000000UL)
#define IPERF_HEADER_SEQNO_64BIT        ((uint32_t)0x08000000UL)
#define IPERF_HEADER_RUN_NOW            ((uint32_t)0x00000001UL)
#define IPERF_HEADER_UNITS_PPS          ((uint32_t)0x00000002UL)
#define IPERF_PACER_BURST_TICKS         2U
#define IPERF_PACER_MIN_BURST_PACKETS   4U
#define IPERF_PACER_MAX_BACKOFF_TICKS   8U
#define IPERF_UDP_SEQUENCE_WINDOW_BITS  4096U
#define IPERF_UDP_SEQUENCE_BITMAP_WORDS (IPERF_UDP_SEQUENCE_WINDOW_BITS / 32U)

typedef struct
{
    uint32_t id;
    uint32_t tv_sec;
    uint32_t tv_usec;
} iperf_udp_legacy_datagram_wire_t;

typedef struct
{
    uint32_t id;
    uint32_t tv_sec;
    uint32_t tv_usec;
    uint32_t id_high;
} iperf_udp_datagram_wire_t;

typedef struct
{
    uint32_t flags;
    uint32_t num_threads;
    uint32_t port;
    uint32_t buffer_len;
    uint32_t window_size;
    uint32_t amount;
} iperf_udp_client_header_v1_wire_t;

/* The 2.0.9 client header is 36 bytes and follows the UDP datagram header. */
typedef struct
{
    uint32_t flags;
    uint32_t num_threads;
    uint32_t port;
    uint32_t buffer_len;
    uint32_t window_size;
    uint32_t amount;
    uint32_t rate;
    uint32_t rate_units;
    uint32_t realtime;
} iperf_udp_legacy_client_header_wire_t;

typedef struct
{
    uint32_t type;
    uint32_t length;
    uint32_t flags;
    uint32_t version_upper;
    uint32_t version_lower;
    uint32_t reserved;
    uint32_t rate;
    uint32_t rate_units;
    uint32_t realtime;
} iperf_udp_client_header_extension_wire_t;

typedef struct
{
    uint16_t test_flags;
    uint16_t tlv_offset;
    uint32_t version_upper;
    uint32_t version_lower;
} iperf_udp_client_tests_wire_t;

typedef union
{
    iperf_udp_client_header_extension_wire_t extension;
    iperf_udp_client_tests_wire_t udp_tests;
} iperf_udp_client_header_payload_wire_t;

typedef struct
{
    iperf_udp_client_header_v1_wire_t base;
    iperf_udp_client_header_payload_wire_t payload;
} iperf_udp_client_header_wire_t;

typedef struct
{
    uint32_t flags;
    uint32_t total_len_high;
    uint32_t total_len_low;
    uint32_t stop_sec;
    uint32_t stop_usec;
    uint32_t error_count;
    uint32_t out_of_order_count;
    uint32_t datagrams;
    uint32_t jitter_sec;
    uint32_t jitter_usec;
} iperf_udp_server_header_v1_wire_t;

typedef struct
{
    uint32_t min_transit_sec;
    uint32_t min_transit_usec;
    uint32_t max_transit_sec;
    uint32_t max_transit_usec;
    uint32_t sum_transit_sec;
    uint32_t sum_transit_usec;
    uint32_t mean_transit_sec;
    uint32_t mean_transit_usec;
    uint32_t m2_transit_sec;
    uint32_t m2_transit_usec;
    uint32_t variance_transit_sec;
    uint32_t variance_transit_usec;
    uint32_t transit_count;
    uint32_t ipg_count;
    uint32_t ipg_sum;
} iperf_udp_server_extension_wire_t;

typedef struct
{
    uint32_t error_count_high;
    uint32_t out_of_order_count_high;
    uint32_t datagrams_high;
} iperf_udp_server_extension2_wire_t;

typedef struct
{
    iperf_udp_server_header_v1_wire_t base;
    iperf_udp_server_extension_wire_t extension;
    iperf_udp_server_extension2_wire_t extension2;
} iperf_udp_server_header_wire_t;

typedef struct
{
    iperf_udp_datagram_wire_t datagram;
    iperf_udp_server_header_wire_t server;
} iperf_udp_server_report_wire_t;

/* The complete 2.0.9 server header is 25 network-order words (100 bytes). */
typedef struct
{
    iperf_udp_server_header_v1_wire_t base;
    iperf_udp_server_extension_wire_t extension;
} iperf_udp_legacy_server_header_wire_t;

typedef struct
{
    iperf_udp_legacy_datagram_wire_t datagram;
    iperf_udp_legacy_server_header_wire_t server;
} iperf_udp_legacy_server_report_full_wire_t;

typedef struct
{
    iperf_udp_legacy_datagram_wire_t datagram;
    iperf_udp_server_header_v1_wire_t server;
} iperf_udp_legacy_server_report_wire_t;

typedef struct
{
    uint64_t rate_bits_per_second;
    uint64_t token_units;
    uint64_t capacity_units;
    uint32_t ticks_per_second;
    uint32_t last_tick;
    bool enabled;
} iperf_pacer_t;

typedef struct
{
    uint64_t window_start_packet_id;
    uint32_t received_bitmap[IPERF_UDP_SEQUENCE_BITMAP_WORDS];
} iperf_udp_sequence_tracker_t;

typedef struct
{
    uint64_t received_bytes;
    uint64_t first_arrival_us;
    uint64_t last_arrival_us;
    int64_t previous_transit_us;
    double jitter_us;
    uint64_t next_packet_id;
    uint64_t received_datagrams;
    uint64_t lost_datagrams;
    uint64_t out_of_order_datagrams;
    uint64_t duplicate_datagrams;
    int64_t min_transit_us;
    int64_t max_transit_us;
    int64_t sum_transit_us;
    double mean_transit_us;
    double m2_transit_us;
    double variance_transit_us;
    uint64_t transit_count;
    bool sequence_initialized;
    bool transit_initialized;
} iperf_udp_stats_t;

void iperf_pacer_init(iperf_pacer_t *pacer,
                      uint64_t rate_bits_per_second,
                      size_t packet_size_bytes,
                      uint32_t ticks_per_second,
                      uint32_t now_tick);
bool iperf_pacer_try_consume(iperf_pacer_t *pacer,
                             size_t packet_size_bytes,
                             uint32_t now_tick,
                             uint32_t *wait_ticks_out);
uint32_t iperf_pacer_backoff_ticks(uint32_t failure_streak);

void iperf_udp_stats_init(iperf_udp_stats_t *stats);
void iperf_udp_sequence_tracker_init(iperf_udp_sequence_tracker_t *tracker);
bool iperf_udp_stats_record(iperf_udp_stats_t *stats,
                            iperf_udp_sequence_tracker_t *tracker,
                            uint64_t packet_id,
                            uint64_t sent_time_us,
                            uint64_t arrival_time_us,
                            size_t packet_size_bytes);
void iperf_udp_stats_finish(iperf_udp_stats_t *stats, uint64_t expected_datagrams);
double iperf_udp_stats_loss_percent(const iperf_udp_stats_t *stats);

#endif
