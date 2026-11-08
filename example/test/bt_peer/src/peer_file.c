/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * File push for the bt_peer instrument.
 *
 * The DUT runs the watchface download server of the `sifli_sd`
 * serial-transport service; the instrument is the GATT client that would
 * otherwise be a phone. Driving it is a multi-step exchange, so one `file.push`
 * op owns the link until the last response or the timeout.
 *
 * Wire format, both directions (bf0_sibles_serial_trans_service.h):
 *   [cate_id = 0x04][flag][length u16 LE][payload]
 * `flag` 0 is a complete packet; 1/2/3 are the fragments of a longer message
 * (only the first fragment carries the total length).
 *
 * The payload of a request is a watchface message with a length field
 * (bf0_sibles_watchface.h):
 *   [message_id u16 LE][length u16 LE][data...]
 * Responses come back with the id followed directly by the id-specific body,
 * with no length word: the serial frame's length already says how long the
 * message is. The layouts below are therefore read from the DUT's senders,
 * e.g. ble_watchface_send_start_rsp_file_info(), not from a symmetric
 * encoder/decoder pair.
 *
 * Requests: START_REQ -> FILE_SEND_START_REQ -> FILE_SEND_DATA (one per
 * chunk) -> FILE_SEND_END_REQ -> END_REQ. Each is answered by the response
 * with the next odd id; a non-zero result stops the push.
 */

#include <rtthread.h>
#include <string.h>

#include "peer.h"
#include "bf0_sibles.h"

/* Serial-transport framing. */
#define PEER_FILE_CATE_ID           0x04
#define PEER_FILE_FLAG_COMPLETE     0x00

/* Watchface message ids. */
#define PEER_FILE_START_REQ         0
#define PEER_FILE_START_RSP         1
#define PEER_FILE_FILE_START_REQ    2
#define PEER_FILE_FILE_START_RSP    3
#define PEER_FILE_DATA_REQ          4
#define PEER_FILE_DATA_RSP          5
#define PEER_FILE_END_REQ           6
#define PEER_FILE_END_RSP           7
#define PEER_FILE_FINISH_REQ        8
#define PEER_FILE_FINISH_RSP        9

/** One step may wait for a connection interval or two. */
#define PEER_FILE_TIMEOUT_MS        10000
/**
 * Largest chunk written per request. The real limit is the negotiated ATT MTU
 * (`mtu - 15`: 3 bytes of ATT, 4 of serial frame, 4 of message header and 4 of
 * chunk index). The cap also keeps a `file.push` command well inside the
 * shell's usable line length: a line over ~300 characters arrives truncated,
 * and the hex payload is two characters per byte.
 */
#define PEER_FILE_CHUNK_MAX         48
/** Longest file name accepted; the DUT's path buffer is 256 bytes. */
#define PEER_FILE_NAME_MAX          64
#define PEER_FILE_NO_CONN           0xFF

/* State of the push in flight; the notification callback writes the response
 * side of it, the shell thread the request side. */
static uint8_t g_push_active;
static uint8_t g_push_conn = PEER_FILE_NO_CONN;
static uint16_t g_push_handle;
static uint16_t g_push_expect;
static uint8_t g_push_rsp[PEER_MAX_PAYLOAD];
static uint16_t g_push_rsp_len;
static int g_push_rsp_rc;

/* Reassembly of a message the DUT had to fragment to fit the ATT MTU. */
static uint8_t g_push_asm[PEER_MAX_PAYLOAD];
static uint16_t g_push_asm_len;
static uint8_t g_push_asm_on;

static uint16_t peer_file_le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static void peer_file_put16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFF);
    p[1] = (uint8_t)(value >> 8);
}

static void peer_file_put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value & 0xFF);
    p[1] = (uint8_t)((value >> 8) & 0xFF);
    p[2] = (uint8_t)((value >> 16) & 0xFF);
    p[3] = (uint8_t)((value >> 24) & 0xFF);
}

/** CRC-32 exactly as the DUT computes it (polynomial_crc_calculate()). */
static uint32_t peer_file_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    uint32_t i;
    uint8_t bit;

    for (i = 0; i < len; i++)
    {
        crc ^= (uint32_t)data[i] << 24;
        for (bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }
    }

    return crc;
}

void peer_file_on_notify(uint8_t conn_idx, uint16_t handle,
                         const uint8_t *data, uint16_t len)
{
    const uint8_t *payload;
    uint16_t payload_len;
    uint8_t flag;
    uint16_t msg_id;

    if (!g_push_active || conn_idx != g_push_conn || handle != g_push_handle)
        return;
    if (data == RT_NULL || len < 2 || data[0] != PEER_FILE_CATE_ID)
        return;

    flag = data[1];
    if (flag == PEER_FILE_FLAG_COMPLETE)
    {
        if (len < 4)
            return;
        payload_len = peer_file_le16(data + 2);
        /* The frame is authoritative: a sender that forgot the length still
         * gets its bytes delivered instead of a silently truncated message. */
        if ((uint32_t)payload_len + 4 > len)
            payload_len = (uint16_t)(len - 4);
        payload = data + 4;
        g_push_asm_on = 0;
    }
    else if (flag == 1)
    {
        if (len < 4)
            return;
        payload_len = (uint16_t)(len - 4);
        if (payload_len > sizeof(g_push_asm))
            return;
        memcpy(g_push_asm, data + 4, payload_len);
        g_push_asm_len = payload_len;
        g_push_asm_on = 1;
        return;
    }
    else if (flag == 2 || flag == 3)
    {
        if (!g_push_asm_on || len < 2)
            return;
        payload_len = (uint16_t)(len - 2);
        if ((uint32_t)g_push_asm_len + payload_len > sizeof(g_push_asm))
        {
            g_push_asm_on = 0;
            return;
        }
        memcpy(g_push_asm + g_push_asm_len, data + 2, payload_len);
        g_push_asm_len = (uint16_t)(g_push_asm_len + payload_len);
        if (flag != 3)
            return;
        payload = g_push_asm;
        payload_len = g_push_asm_len;
        g_push_asm_on = 0;
    }
    else
    {
        return;
    }

    if (payload_len < 2)
        return;

    msg_id = peer_file_le16(payload);
    if (msg_id != g_push_expect)
        return;

    /* Responses carry the body straight after the id (see the file comment). */
    payload += 2;
    payload_len = (uint16_t)(payload_len - 2);
    if (payload_len > sizeof(g_push_rsp))
        return;

    memcpy(g_push_rsp, payload, payload_len);
    g_push_rsp_len = payload_len;
    g_push_rsp_rc = 0;
    peer_ble_signal(PEER_BIT_FILE_RSP);
}

void peer_file_on_disconnect(uint8_t conn_idx)
{
    if (!g_push_active || conn_idx != g_push_conn)
        return;

    g_push_rsp_rc = -2;
    peer_ble_signal(PEER_BIT_FILE_RSP);
}

/**
 * Send one request and wait for its response.
 *
 * On success the response body is in the module's buffer; the caller checks
 * the id-specific result. @p step names the exchange in every failure text.
 */
static int peer_file_request(peer_link_t *link, uint16_t handle,
                             uint16_t msg_id, const uint8_t *data,
                             uint16_t data_len, uint16_t rsp_id,
                             const char *step, peer_reply_t *reply)
{
    static uint8_t frame[PEER_MAX_PAYLOAD + 16];
    uint16_t msg_len = (uint16_t)(4 + data_len);
    uint16_t frame_len = (uint16_t)(4 + msg_len);
    char detail[sizeof(reply->text)];

    if ((uint32_t)msg_len + 4 > sizeof(frame))
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "file.push %s size", step);
        return -1;
    }

    frame[0] = PEER_FILE_CATE_ID;
    frame[1] = PEER_FILE_FLAG_COMPLETE;
    peer_file_put16(frame + 2, msg_len);
    peer_file_put16(frame + 4, msg_id);
    peer_file_put16(frame + 6, data_len);
    if (data_len > 0)
        memcpy(frame + 8, data, data_len);

    g_push_expect = rsp_id;
    g_push_rsp_len = 0;
    g_push_rsp_rc = -1;
    peer_ble_clear_bits(PEER_BIT_FILE_RSP);

    if (peer_gatt_client_write_raw(link, handle, frame, frame_len, 1, reply) < 0)
    {
        /* Keep the write path's reason (stack/ATT result) and add the step it
         * belonged to. Copy first: reply_err formats into reply->text. */
        rt_strncpy(detail, reply->text, sizeof(detail) - 1);
        detail[sizeof(detail) - 1] = '\0';
        peer_reply_err(reply, reply->code, "file.push %s %s", step, detail);
        return -1;
    }

    if (peer_ble_wait(PEER_BIT_FILE_RSP, PEER_FILE_TIMEOUT_MS) != RT_EOK)
    {
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "file.push %s", step);
        return -1;
    }

    if (g_push_rsp_rc != 0)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "file.push %s disconnected", step);
        return -1;
    }

    if (g_push_rsp_len < 2)
    {
        peer_reply_err(reply, PEER_ERR_PROTO, "file.push %s response", step);
        return -1;
    }

    return 0;
}

/** The DUT's result word, at the start of the response body. */
static int peer_file_result(const char *step, peer_reply_t *reply)
{
    uint16_t result;

    if (g_push_rsp_len < 2)
    {
        peer_reply_err(reply, PEER_ERR_PROTO, "file.push %s short", step);
        return -1;
    }

    result = peer_file_le16(g_push_rsp);
    if (result != 0)
    {
        peer_reply_err(reply, PEER_ERR_FILE_FAILED, "file.push %s result=%u",
                       step, result);
        return -1;
    }

    return 0;
}

int peer_file_push(peer_link_t *link, uint16_t handle, const char *name,
                   const uint8_t *data, uint16_t len, peer_reply_t *reply)
{
    static uint8_t file[PEER_MAX_PAYLOAD + 4];
    /* Holds the largest request body this op builds: a data chunk plus its
     * index, or a file name plus its length. */
    static uint8_t req[PEER_FILE_CHUNK_MAX + 4];
    uint16_t name_len;
    uint32_t file_len;
    uint32_t crc;
    uint32_t chunk_max;
    uint32_t offset;
    uint32_t index;
    uint16_t peer_max;
    int rc = -1;

    if (link == RT_NULL || name == RT_NULL || data == RT_NULL ||
            name[0] == '\0')
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "file.push");
        return -1;
    }

    /* The DUT rejects an unaligned length, and the CRC lives in the last four
     * bytes, so a payload the caller cannot frame is refused here. */
    if (len == 0 || (len & 3) != 0)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "file.push len");
        return -1;
    }

    name_len = (uint16_t)rt_strlen(name);
    if (name_len > PEER_FILE_NAME_MAX)
    {
        peer_reply_err(reply, PEER_ERR_INVALID_ARG, "file.push name");
        return -1;
    }

    if (link->rmt_svc_hdl == SIBLES_ERROR_REMOTE_HANDLE)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "file.push discover");
        return -1;
    }

    if (link->gatt_busy)
    {
        peer_reply_err(reply, PEER_ERR_BUSY, "file.push gatt");
        return -1;
    }

    /* The trailing four bytes are the CRC of the payload, and all_files_len
     * counts them: the DUT compares it with what it received. The DUT rebuilds
     * the word with the last byte as the high byte, so it goes out little
     * endian (file_recv_check_crc32()). */
    memcpy(file, data, len);
    crc = peer_file_crc32(data, len);
    file[len] = (uint8_t)(crc & 0xFF);
    file[len + 1] = (uint8_t)((crc >> 8) & 0xFF);
    file[len + 2] = (uint8_t)((crc >> 16) & 0xFF);
    file[len + 3] = (uint8_t)(crc >> 24);
    file_len = (uint32_t)len + 4;

    g_push_active = 1;
    g_push_conn = link->conn_idx;
    g_push_handle = handle;
    g_push_expect = 0xFFFF;
    g_push_asm_on = 0;
    link->gatt_busy = 1;

    /* START_REQ: watchface file type 0, phone type unknown, total length. */
    req[0] = 0;
    req[1] = 0;
    req[2] = 0;
    peer_file_put32(req + 3, file_len);
    if (peer_file_request(link, handle, PEER_FILE_START_REQ, req, 7,
                          PEER_FILE_START_RSP, "start_rsp", reply) < 0)
        goto out;
    if (peer_file_result("start_rsp", reply) < 0)
        goto out;

    /* START_RSP body: result u16, max_data_len u16, version u16, ... */
    peer_max = 20;
    if (g_push_rsp_len >= 4)
        peer_max = peer_file_le16(g_push_rsp + 2);

    /* FILE_SEND_START_REQ: aligned length, name length, name. */
    peer_file_put32(req, file_len);
    peer_file_put16(req + 4, name_len);
    memcpy(req + 6, name, name_len);
    if (peer_file_request(link, handle, PEER_FILE_FILE_START_REQ, req,
                          (uint16_t)(6 + name_len), PEER_FILE_FILE_START_RSP,
                          "file_start_rsp", reply) < 0)
        goto out;
    if (peer_file_result("file_start_rsp", reply) < 0)
        goto out;

    /* A chunk of 4 bytes also keeps the CRC from crossing two packets, which
     * the DUT assumes when it accumulates it. */
    chunk_max = peer_max;
    if (chunk_max > PEER_FILE_CHUNK_MAX)
        chunk_max = PEER_FILE_CHUNK_MAX;
    if (link->mtu > 15 && chunk_max > (uint32_t)(link->mtu - 15))
        chunk_max = (uint32_t)(link->mtu - 15);
    chunk_max &= ~3UL;
    if (chunk_max < 4)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "file.push mtu=%u", link->mtu);
        goto out;
    }

    offset = 0;
    index = 0;
    while (offset < file_len)
    {
        uint32_t chunk = file_len - offset;

        if (chunk > chunk_max)
            chunk = chunk_max;

        peer_file_put32(req, index);
        memcpy(req + 4, file + offset, chunk);
        if (peer_file_request(link, handle, PEER_FILE_DATA_REQ, req,
                              (uint16_t)(4 + chunk), PEER_FILE_DATA_RSP,
                              "data_rsp", reply) < 0)
            goto out;
        if (peer_file_result("data_rsp", reply) < 0)
            goto out;

        offset += chunk;
        index++;
    }

    if (peer_file_request(link, handle, PEER_FILE_END_REQ, RT_NULL, 0,
                          PEER_FILE_END_RSP, "end_rsp", reply) < 0)
        goto out;
    if (peer_file_result("end_rsp", reply) < 0)
        goto out;

    if (peer_file_request(link, handle, PEER_FILE_FINISH_REQ, RT_NULL, 0,
                          PEER_FILE_FINISH_RSP, "finish_rsp", reply) < 0)
        goto out;
    if (peer_file_result("finish_rsp", reply) < 0)
        goto out;

    peer_reply_ok(reply, "name=%s bytes=%u", name, len);
    rc = 0;

out:
    g_push_active = 0;
    g_push_conn = PEER_FILE_NO_CONN;
    g_push_handle = 0;
    g_push_expect = 0xFFFF;
    g_push_asm_on = 0;
    link->gatt_busy = 0;
    return rc;
}
