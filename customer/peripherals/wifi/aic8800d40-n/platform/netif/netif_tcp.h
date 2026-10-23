/*
 * Copyright (C) 2018-2025 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _NETIF_TCP_H_
#define _NETIF_TCP_H_

#include "aic_porting_types.h"

/*
 *  TCP option
 */
#define TCPOPT_NOP      1   /* Padding */
#define TCPOPT_EOL      0   /* End of options */
#define TCPOPT_MSS      2   /* Segment size negotiating */
#define TCPOPT_WINDOW       3   /* Window scaling */
#define TCPOPT_SACK_PERM        4       /* SACK Permitted */
#define TCPOPT_SACK             5       /* SACK Block */
#define TCPOPT_TIMESTAMP    8   /* Better RTT estimations/PAWS */
#define TCPOPT_MD5SIG       19  /* MD5 Signature (RFC2385) */
#define TCPOPT_FASTOPEN     34  /* Fast open (RFC7413) */
#define TCPOPT_EXP      254 /* Experimental */


#define CPU_LITTLE_ENDIAN 1
/*
 *  kind of define for ipv4 header
 *  definition from netlink/ipof FreeBSD
 *  this definition is different to ip_hdr of lwip/ip.h
 */
struct iphdr
{
#if CPU_LITTLE_ENDIAN // little endian
    __u8    ihl: 4,
            version: 4;
#else // big endian
    __u8    version: 4,
            ihl: 4;
#endif
    __u8    tos;
    __be16  tot_len;
    __be16  id;
    __be16  frag_off;
//#define   IP_RF 0x8000            /* reserved fragment flag */
//#define   IP_DF 0x4000            /* dont fragment flag */
//#define   IP_MF 0x2000            /* more fragments flag */
//#define   IP_OFFMASK 0x1fff       /* mask for fragmenting bits */
    __u8    ttl;
    __u8    protocol;
    __sum16 check;
    __be32  saddr;
    __be32  daddr;
    /*The options start here. */
} __attribute__((packed));


/*
 *  kind of define for TCP header
 *  definition from netlink/tcp.h FreeBSD
 *  this definition is different to tcp_hdr of lwip/tcp_impl.h
 */
struct tcphdr
{
    __be16  source;
    __be16  dest;
    __be32  seq;
    __be32  ack_seq;
#if CPU_LITTLE_ENDIAN // little endian
    __u16   res1: 4,
            doff: 4,
            fin: 1,
            syn: 1,
            rst: 1,
            psh: 1,
            ack: 1,
            urg: 1,
            ece: 1,
            cwr: 1;
#else // big endian
    __u16   doff: 4,
            res1: 4,
            cwr: 1,
            ece: 1,
            urg: 1,
            ack: 1,
            psh: 1,
            rst: 1,
            syn: 1,
            fin: 1;
#endif
    __be16  window;
    __sum16 check;
    __be16  urg_ptr;
} __attribute__((packed));

#endif
