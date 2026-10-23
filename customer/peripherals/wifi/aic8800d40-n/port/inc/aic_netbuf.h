/*
 * Copyright (C) 2018-2024 AICSemi Ltd.
 *
 * All Rights Reserved
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unified net_buf_tx_t definition.
 *
 * This is the SINGLE canonical definition of net_buf_tx_t.
 *
 * It lives in a standalone, minimal-dependency header so that:
 *   - netif_port.h   (port/inc/)  can reference net_buf_tx_t / net_if_t
 *   - netif_def.h    (drv_api/inc/) can alias it to TCPIP_PACKET_INFO_T
 *
 * None of these three files need circular-include guards anymore —
 * they simply #include "aic_netbuf.h" unconditionally.
 *
 * The struct layout is pointer-compatible with TCPIP_PACKET_INFO_T
 * (defined in netif_def.h for internal use).
 */

#ifndef _AIC_NETBUF_H_
#define _AIC_NETBUF_H_

#include "aic_porting_types.h"

typedef void *net_if_t;  /* opaque network interface handle */

typedef struct net_buf_tx
{
    uint8_t *data_ptr;
    uint32_t data_len;
    uint32_t pkt_type;
    uint32_t net_id;
    void    *frag_next;  /* [PLATFORM] nocopy TX (NET_IF_UAP_NCPYBUF=1): pbuf chain
                            pointer, passed through to the SDIO aggregation scatter
                            copy; always NULL in copy mode */
#ifdef CONFIG_SDIO_ADMA
    /* In ADMA mode the TX buffer is provided by the platform as a
     * "whole-block allocation + alignment": data_ptr points to the aligned
     * payload, the block start address (for freeing) is recorded here, and
     * platform netif_port.c aicwf_net_buf_tx_free() passes it to aicwf_rtos_free().
     * Same semantics as TCPIP_PACKET_INFO_T.data_oprt in host drv_api/inc/netif_def.h;
     * without this field the ADMA branch in platform/netif/netif_port.c does not
     * build (#136 no field). */
    uint8_t *data_oprt;
#endif
} net_buf_tx_t;

#endif /* _AIC_NETBUF_H_ */
