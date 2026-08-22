/**
 * @file dfu_protocol.c
 * @brief DFU V2 Frame Protocol — Parser, Builder, CRC16
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <string.h>
#include "dfu_protocol.h"

#define LOG_TAG "dfu.proto"
#include <log.h>

/*============================================================================
 * CRC16-CCITT (Polynomial 0x1021, Init 0xFFFF)
 *============================================================================*/

/**
 * Identical algorithm to dfu_core_crc16_calculate() in the existing SDK.
 * Duplicated here so that dfu_protocol module has zero dependency on
 * dfu_core (self-contained and unit-testable).
 */
uint16_t dfu_proto_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/*============================================================================
 * Incremental CRC16 helper (feed one byte at a time)
 *============================================================================*/

static inline uint16_t crc16_feed_byte(uint16_t crc, uint8_t byte)
{
    crc ^= ((uint16_t)byte << 8);
    for (uint8_t j = 0; j < 8; j++)
    {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
    return crc;
}

/*============================================================================
 * Frame Builder
 *============================================================================*/

int dfu_proto_frame_build(dfu_cmd_id_t cmd_id,
                          const uint8_t *payload,
                          uint16_t payload_len,
                          uint8_t *buf,
                          uint32_t buf_size)
{
    uint32_t total = DFU_PROTO_OVERHEAD + payload_len;

    if (!buf || buf_size < total)
        return -1;

    if (payload_len > DFU_PROTO_MAX_PAYLOAD)
        return -2;

    uint16_t msg_id = (uint16_t)cmd_id;

    /* Header */
    buf[0] = DFU_PROTO_HEADER_0;
    buf[1] = DFU_PROTO_HEADER_1;

    /* MSG_ID (2 bytes, little-endian) */
    buf[2] = (uint8_t)(msg_id & 0xFF);
    buf[3] = (uint8_t)((msg_id >> 8) & 0xFF);

    /* LENGTH (2 bytes, little-endian) */
    buf[4] = (uint8_t)(payload_len & 0xFF);
    buf[5] = (uint8_t)((payload_len >> 8) & 0xFF);

    /* PAYLOAD */
    if (payload_len > 0 && payload != NULL)
        memcpy(&buf[6], payload, payload_len);

    /* CRC16 over MSG_ID(2B) + LENGTH(2B) + PAYLOAD(NB) = bytes [2..5+payload_len] */
    uint16_t crc = dfu_proto_crc16(&buf[2], 2 + 2 + payload_len);
    buf[6 + payload_len]     = (uint8_t)(crc & 0xFF);
    buf[6 + payload_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

    return (int)total;
}

uint8_t *dfu_proto_frame_alloc_build(dfu_cmd_id_t cmd_id,
                                     const uint8_t *payload,
                                     uint16_t payload_len,
                                     uint32_t *out_len)
{
    uint32_t total = DFU_PROTO_OVERHEAD + payload_len;
    uint8_t *buf = (uint8_t *)rt_malloc(total);

    if (!buf)
        return NULL;

    int ret = dfu_proto_frame_build(cmd_id, payload, payload_len, buf, total);
    if (ret < 0)
    {
        rt_free(buf);
        return NULL;
    }

    if (out_len)
        *out_len = (uint32_t)ret;

    return buf;
}

/*============================================================================
 * Frame Parser — Internal State
 *============================================================================*/

typedef enum
{
    PARSE_WAIT_HEADER_0 = 0,
    PARSE_WAIT_HEADER_1,
    PARSE_WAIT_MSGID_0,
    PARSE_WAIT_MSGID_1,
    PARSE_WAIT_LENGTH_0,
    PARSE_WAIT_LENGTH_1,
    PARSE_WAIT_PAYLOAD,
    PARSE_WAIT_CRC_0,
    PARSE_WAIT_CRC_1,
} parse_state_t;

struct dfu_proto_parser
{
    /* State machine */
    parse_state_t       state;

    /* Frame assembly */
    uint16_t            msg_id;
    uint16_t            payload_len;
    uint16_t            payload_pos;
    uint8_t             payload_buf[DFU_PROTO_MAX_PAYLOAD];
    uint16_t            recv_crc;

    /* Incremental CRC computed while receiving (covers MSG_ID+LENGTH+PAYLOAD) */
    uint16_t            running_crc;

    /* Callback */
    dfu_proto_on_frame_t on_frame;
    void                *user_data;
};

/*============================================================================
 * Frame Parser — Public API
 *============================================================================*/

dfu_proto_parser_t *dfu_proto_parser_create(dfu_proto_on_frame_t on_frame,
                                            void *user_data)
{
    dfu_proto_parser_t *p = (dfu_proto_parser_t *)rt_malloc(sizeof(dfu_proto_parser_t));
    if (!p)
        return NULL;

    memset(p, 0, sizeof(*p));
    p->state     = PARSE_WAIT_HEADER_0;
    p->on_frame  = on_frame;
    p->user_data = user_data;
    return p;
}

void dfu_proto_parser_destroy(dfu_proto_parser_t *parser)
{
    if (parser)
        rt_free(parser);
}

void dfu_proto_parser_reset(dfu_proto_parser_t *parser)
{
    if (!parser)
        return;
    parser->state       = PARSE_WAIT_HEADER_0;
    parser->msg_id      = 0;
    parser->payload_len = 0;
    parser->payload_pos = 0;
    parser->recv_crc    = 0;
    parser->running_crc = 0;
}

int dfu_proto_parser_feed(dfu_proto_parser_t *parser,
                          const uint8_t *data,
                          uint32_t len)
{
    if (!parser || !data || len == 0)
        return -1;

    for (uint32_t i = 0; i < len; i++)
    {
        uint8_t byte = data[i];

        switch (parser->state)
        {
        /* ---- Wait for 0xAA ---- */
        case PARSE_WAIT_HEADER_0:
            if (byte == DFU_PROTO_HEADER_0)
                parser->state = PARSE_WAIT_HEADER_1;
            break;

        /* ---- Wait for 0x55 ---- */
        case PARSE_WAIT_HEADER_1:
            if (byte == DFU_PROTO_HEADER_1)
            {
                parser->running_crc = 0xFFFF;   /* Init CRC for this frame */
                parser->state = PARSE_WAIT_MSGID_0;
            }
            else if (byte == DFU_PROTO_HEADER_0)
            {
                /* Another 0xAA — might be start of new frame */
                parser->state = PARSE_WAIT_HEADER_1;
            }
            else
            {
                parser->state = PARSE_WAIT_HEADER_0;
            }
            break;

        /* ---- MSG_ID low byte ---- */
        case PARSE_WAIT_MSGID_0:
            parser->msg_id = byte;
            parser->running_crc = crc16_feed_byte(parser->running_crc, byte);
            parser->state = PARSE_WAIT_MSGID_1;
            break;

        /* ---- MSG_ID high byte ---- */
        case PARSE_WAIT_MSGID_1:
            parser->msg_id |= ((uint16_t)byte << 8);
            parser->running_crc = crc16_feed_byte(parser->running_crc, byte);
            parser->state = PARSE_WAIT_LENGTH_0;
            break;

        /* ---- LENGTH low byte ---- */
        case PARSE_WAIT_LENGTH_0:
            parser->payload_len = byte;
            parser->running_crc = crc16_feed_byte(parser->running_crc, byte);
            parser->state = PARSE_WAIT_LENGTH_1;
            break;

        /* ---- LENGTH high byte ---- */
        case PARSE_WAIT_LENGTH_1:
            parser->payload_len |= ((uint16_t)byte << 8);
            parser->running_crc = crc16_feed_byte(parser->running_crc, byte);
            parser->payload_pos = 0;

            if (parser->payload_len > DFU_PROTO_MAX_PAYLOAD)
            {
                LOG_E("frame payload too large: %u", parser->payload_len);
                parser->state = PARSE_WAIT_HEADER_0;
            }
            else if (parser->payload_len == 0)
            {
                parser->state = PARSE_WAIT_CRC_0;
            }
            else
            {
                parser->state = PARSE_WAIT_PAYLOAD;
            }
            break;

        /* ---- Accumulate payload ---- */
        case PARSE_WAIT_PAYLOAD:
            parser->payload_buf[parser->payload_pos++] = byte;
            parser->running_crc = crc16_feed_byte(parser->running_crc, byte);
            if (parser->payload_pos >= parser->payload_len)
                parser->state = PARSE_WAIT_CRC_0;
            break;

        /* ---- CRC low byte ---- */
        case PARSE_WAIT_CRC_0:
            parser->recv_crc = byte;
            parser->state = PARSE_WAIT_CRC_1;
            break;

        /* ---- CRC high byte → frame complete ---- */
        case PARSE_WAIT_CRC_1:
        {
            parser->recv_crc |= ((uint16_t)byte << 8);

            if (parser->running_crc != parser->recv_crc)
            {
                LOG_E("CRC mismatch: calc=0x%04x recv=0x%04x",
                      parser->running_crc, parser->recv_crc);
                parser->state = PARSE_WAIT_HEADER_0;
                break;
            }

            /* CRC OK — fire callback */
            if (parser->on_frame)
            {
                parser->on_frame((dfu_cmd_id_t)parser->msg_id,
                                 parser->payload_buf,
                                 parser->payload_len,
                                 parser->user_data);
            }
            parser->state = PARSE_WAIT_HEADER_0;
            break;
        }

        default:
            parser->state = PARSE_WAIT_HEADER_0;
            break;
        }
    }

    return 0;
}
