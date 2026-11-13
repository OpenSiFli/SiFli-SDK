/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PEER_CLASSIC_H__
#define __PEER_CLASSIC_H__

#include "peer.h"

#ifdef PEER_CLASSIC

/**
 * Register for Classic stack events and take the server (pageable) role.
 *
 * Must run before sifli_ble_enable(): the stack only delivers events to
 * callbacks that are already registered when it starts.
 */
void peer_classic_init(void);

/**
 * Whether peer_classic_init() completed and the Classic operations can work.
 *
 * The capability is only announced in `@info` when this is true: a build that
 * could not create its IPC objects must not hand the host operations that can
 * only time out.
 */
rt_bool_t peer_classic_available(void);

/** Drop every link and forget the state the host is about to leave. */
void peer_classic_reset(void);

/**
 * Wait for the Classic stack and report the local address and name.
 *
 * @param timeout_ms how long the stack may take to come up
 */
int peer_classic_local(uint32_t timeout_ms, peer_reply_t *reply);

/**
 * Start an inquiry and report every device heard as a `btc.dev` event.
 *
 * The terminal reply is sent only after the inquiry has finished, carrying
 * the device count and the event sequence number from before it started.
 */
int peer_classic_inquiry(uint32_t duration_ms, uint8_t max_rsp,
                         peer_reply_t *reply);

/**
 * Page @p addr and open an SPP connection for @p uuid.
 *
 * @param uuid     UUID in display order (SPP is 0x11, 0x01)
 * @param uuid_len 2, 4 or 16
 */
int peer_classic_connect(const uint8_t *addr, const uint8_t *uuid,
                         uint8_t uuid_len, uint32_t timeout_ms,
                         peer_reply_t *reply);

/**
 * Pair (bond) with @p addr before opening an SPP connection.
 *
 * The peer's SPP service asks for authentication, which an unpaired link
 * cannot satisfy; this must run before peer_classic_connect().
 */
int peer_classic_pair(const uint8_t *addr, uint32_t timeout_ms,
                      peer_reply_t *reply);

/** Request teardown; `btc.disc` confirms it later. */
int peer_classic_disconnect(uint8_t link_id, peer_reply_t *reply);

/** Send one SPP payload; the stack takes ownership of the copy. */
int peer_classic_send(uint8_t link_id, const uint8_t *data, uint16_t len,
                      peer_reply_t *reply);

#endif /* PEER_CLASSIC */

#endif /* __PEER_CLASSIC_H__ */
