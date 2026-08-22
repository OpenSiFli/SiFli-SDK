/**
 * @file dfu_protocol.h
 * @brief DFU V2 Frame Protocol Module
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_PROTOCOL_H__
#define __DFU_PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Frame Constants
 *============================================================================*/

#define DFU_PROTO_HEADER_0          0xAA
#define DFU_PROTO_HEADER_1          0x55
#define DFU_PROTO_HEADER_SIZE       2   /**< 0xAA 0x55 */
#define DFU_PROTO_MSGID_SIZE        2   /**< Message ID (little-endian, uint16_t) */
#define DFU_PROTO_LENGTH_SIZE       2   /**< Payload length (little-endian) */
#define DFU_PROTO_CRC_SIZE          2   /**< CRC16 (little-endian) */

/** Fixed overhead per frame = header(2) + msg_id(2) + length(2) + crc(2) = 8 */
#define DFU_PROTO_OVERHEAD          (DFU_PROTO_HEADER_SIZE + \
                                     DFU_PROTO_MSGID_SIZE  + \
                                     DFU_PROTO_LENGTH_SIZE + \
                                     DFU_PROTO_CRC_SIZE)

/** Maximum payload bytes in one frame */
#define DFU_PROTO_MAX_PAYLOAD       4096

/*============================================================================
 * Command IDs (must match existing dfu_protocol_msg_id_t values)
 *============================================================================*/

typedef enum
{
    /* --- Legacy image mode --- */
    DFU_CMD_INIT_REQ                = 0,
    DFU_CMD_INIT_RSP                = 1,
    DFU_CMD_INIT_COMPLETE_IND       = 2,
    DFU_CMD_RESUME_REQ              = 3,
    DFU_CMD_RESUME_RSP              = 4,
    DFU_CMD_RESUME_COMPLETE_IND     = 5,
    DFU_CMD_IMG_SEND_START          = 6,
    DFU_CMD_IMG_SEND_START_RSP      = 7,
    DFU_CMD_IMG_SEND_END            = 8,
    DFU_CMD_IMG_SEND_END_RSP        = 9,
    DFU_CMD_IMG_SEND_PKT            = 10,
    DFU_CMD_IMG_SEND_PKT_RSP        = 11,
    DFU_CMD_TRANS_END               = 12,
    DFU_CMD_END_IND                 = 13,
    DFU_CMD_FORCE_INIT_REQ          = 14,
    DFU_CMD_CONN_PRIORITY           = 15,
    DFU_CMD_RETRANS_REQ             = 16,
    DFU_CMD_RETRANS_RSP             = 17,
    DFU_CMD_READ_VER_REQ            = 18,
    DFU_CMD_READ_VER_RSP            = 19,
    DFU_CMD_POWER_OFF               = 20,

    /* --- File mode --- */
    DFU_CMD_FILE_INIT_REQ           = 21,
    DFU_CMD_FILE_INIT_RSP           = 22,
    DFU_CMD_FILE_INIT_COMPLETE      = 23,
    DFU_CMD_FILE_START              = 24,
    DFU_CMD_FILE_START_RSP          = 25,
    DFU_CMD_FILE_PKT                = 26,
    DFU_CMD_FILE_PKT_RSP            = 27,
    DFU_CMD_FILE_END                = 28,
    DFU_CMD_FILE_END_RSP            = 29,
    DFU_CMD_FILE_TOTAL_END          = 30,
    DFU_CMD_FILE_TOTAL_END_RSP      = 31,

    /* --- Extended init --- */
    DFU_CMD_INIT_REQ_EXT            = 32,
    DFU_CMD_INIT_RSP_EXT            = 33,
    DFU_CMD_INIT_COMPLETE_EXT       = 34,
    DFU_CMD_LINK_LOSE_CHECK_REQ     = 35,
    DFU_CMD_LINK_LOSE_CHECK_RSP     = 36,
    DFU_CMD_ABORT                   = 37,

    /* --- Package mode (bulk transfer) --- */
    DFU_CMD_PKG_START_REQ           = 38,
    DFU_CMD_PKG_START_RSP           = 39,
    DFU_CMD_PKG_PKT_REQ             = 40,
    DFU_CMD_PKG_PKT_RSP             = 41,
    DFU_CMD_PKG_END_REQ             = 42,
    DFU_CMD_PKG_END_RSP             = 43,
} dfu_cmd_id_t;

/*============================================================================
 * Protocol Error Codes (wire-level, sent to host in response frames)
 *============================================================================*/

typedef enum
{
    DFU_PROTO_ERR_NONE              = 0,
    DFU_PROTO_ERR_GENERAL           = 1,
    DFU_PROTO_ERR_PARAM_INVALID     = 2,
    DFU_PROTO_ERR_SPACE_NOT_ENOUGH  = 3,
    DFU_PROTO_ERR_NOT_READY         = 4,
    DFU_PROTO_ERR_HW_VER            = 5,
    DFU_PROTO_ERR_SDK_VER           = 6,
    DFU_PROTO_ERR_FW_VER            = 7,
    DFU_PROTO_ERR_FW_INVALID        = 8,
    DFU_PROTO_ERR_CTRL_PKT_INVALID  = 9,
    DFU_PROTO_ERR_OTA_ONGOING       = 10,
    DFU_PROTO_ERR_USER_REJECT       = 11,
    DFU_PROTO_ERR_UNEXPECT_STATE    = 12,
    DFU_PROTO_ERR_INDEX_ERROR       = 13,
    DFU_PROTO_ERR_UPDATING          = 14,
    DFU_PROTO_ERR_MISS_PACKET       = 15,
    DFU_PROTO_ERR_FLASH             = 16,
} dfu_proto_err_t;

/*============================================================================
 * Frame Parser
 *============================================================================*/

/** Opaque parser context (allocated internally) */
typedef struct dfu_proto_parser dfu_proto_parser_t;

/**
 * @brief Callback when a complete frame has been assembled
 *
 * @param cmd_id    Command ID from the frame
 * @param payload   Payload pointer (valid only during callback)
 * @param length    Payload length in bytes
 * @param user_data User context
 */
typedef void (*dfu_proto_on_frame_t)(dfu_cmd_id_t cmd_id,
                                     const uint8_t *payload,
                                     uint16_t length,
                                     void *user_data);

/**
 * @brief Create a frame parser instance
 *
 * @param on_frame   Callback for complete frames
 * @param user_data  User context for callback
 * @return Parser handle, or NULL on allocation failure
 */
dfu_proto_parser_t *dfu_proto_parser_create(dfu_proto_on_frame_t on_frame,
                                            void *user_data);

/**
 * @brief Destroy a parser instance
 * @param parser  Handle (NULL-safe)
 */
void dfu_proto_parser_destroy(dfu_proto_parser_t *parser);

/**
 * @brief Feed raw bytes into the parser
 *
 * Input may contain partial frames, exactly one frame, or multiple
 * concatenated frames. The parser's internal state machine handles
 * all cases. For each complete frame found, on_frame callback fires.
 *
 * @param parser  Parser handle
 * @param data    Raw byte stream
 * @param len     Number of bytes
 * @return 0 on success, negative on error (e.g. CRC failure)
 */
int dfu_proto_parser_feed(dfu_proto_parser_t *parser,
                          const uint8_t *data,
                          uint32_t len);

/**
 * @brief Reset parser state machine
 *
 * Discards any partially assembled frame. Use after errors or
 * when starting a new session.
 *
 * @param parser  Parser handle
 */
void dfu_proto_parser_reset(dfu_proto_parser_t *parser);

/*============================================================================
 * Frame Builder
 *============================================================================*/

/**
 * @brief Build a complete protocol frame
 *
 * Assembles: 0xAA 0x55 | msg_id(2B) | length(2B) | payload(N) | CRC16(2B)
 *
 * @param cmd_id       Command ID for the response
 * @param payload      Payload data (may be NULL if payload_len == 0)
 * @param payload_len  Payload length
 * @param[out] buf     Output buffer (caller provides)
 * @param buf_size     Output buffer capacity
 * @return Total frame length (positive), or negative if buffer too small
 */
int dfu_proto_frame_build(dfu_cmd_id_t cmd_id,
                          const uint8_t *payload,
                          uint16_t payload_len,
                          uint8_t *buf,
                          uint32_t buf_size);

/**
 * @brief Convenience: allocate buffer + build frame
 *
 * Allocates a buffer from heap, builds the frame, and returns it.
 * Caller must free the buffer with rt_free() when done.
 *
 * @param cmd_id       Command ID
 * @param payload      Payload data (may be NULL)
 * @param payload_len  Payload length
 * @param[out] out_len Total frame length
 * @return Allocated frame buffer, or NULL on failure
 */
uint8_t *dfu_proto_frame_alloc_build(dfu_cmd_id_t cmd_id,
                                     const uint8_t *payload,
                                     uint16_t payload_len,
                                     uint32_t *out_len);

/*============================================================================
 * CRC Utility
 *============================================================================*/

/**
 * @brief Compute CRC16 (same algorithm as existing host tool)
 *
 * @param data  Input data
 * @param len   Data length
 * @return CRC16 value
 */
uint16_t dfu_proto_crc16(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_PROTOCOL_H__ */