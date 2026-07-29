/**
 * @file dfu_transport.h
 * @brief DFU V2 Transport — Common Interface Types
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_TRANSPORT_H__
#define __DFU_TRANSPORT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Pull Transport (for Device-Initiated Mode)
 *============================================================================*/

/**
 * @brief Pull Transport vtable
 *
 * Device-Initiated Mode calls these to fetch firmware from a remote source.
 * Each pull-type Transport (PAN HTTP, WiFi HTTP, file system, etc.)
 * provides one static instance of this struct.
 */
typedef struct
{
    /**
     * @brief Open the data source
     *
     * For HTTP transports, this establishes the connection and sends
     * the GET request. The offset parameter enables HTTP Range for resume.
     *
     * @param target  Data source identifier (URL, file path, etc.)
     * @param offset  Byte offset to start from (0 = beginning, >0 = resume)
     * @return 0 on success, negative on failure
     */
    int (*open)(const char *target, uint32_t offset);

    /**
     * @brief Read data (blocking)
     *
     * Blocks until data is available or EOF/error occurs.
     *
     * @param buf      Destination buffer
     * @param max_len  Maximum bytes to read
     * @return Positive = bytes actually read
     *         0 = EOF (all data received)
     *         Negative = error
     */
    int (*read)(uint8_t *buf, uint32_t max_len);

    /**
     * @brief Close the data source
     *
     * Releases connection resources. Safe to call multiple times.
     */
    void (*close)(void);
} dfu_pull_transport_t;

/*============================================================================
 * Push Transport (for Host-Initiated Mode)
 *============================================================================*/

/**
 * @brief Push Transport vtable
 *
 * Host-Initiated Mode calls send() to transmit response frames to host.
 * The data passed to send() is already a complete protocol frame
 * (with AA55 header and CRC16), built by the Protocol module.
 * Transport sends it as raw bytes without modification.
 */
typedef struct
{
    /**
     * @brief Send data to host
     *
     * @param data  Frame bytes to send (complete protocol frame)
     * @param len   Number of bytes
     * @return 0 on success, negative on failure
     */
    int (*send)(const uint8_t *data, uint16_t len);
} dfu_push_transport_t;

#ifdef __cplusplus
}
#endif

#endif /* __DFU_TRANSPORT_H__ */
