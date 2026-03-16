/**
 * @file dfu_sec.h
 * @brief DFU V2 — Security primitives for ctrl_packet decryption
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_SEC_H__
#define __DFU_SEC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize V2 security subsystem
 *
 * Checks if secure boot is enabled. If not, installs fake eFuse keys
 * for development boards. Must be called once at startup.
 */
void dfu_sec_init(void);

/**
 * @brief Decrypt data with AES-CTR and verify SHA-256 hash
 *
 * Used by dfu_transport_ble.c to decrypt the ctrl_packet from
 * INIT_REQ_EXT before parsing.
 *
 * @param key       AES key (NULL = use root key from eFuse)
 * @param offset    AES-CTR initial offset (0 for ctrl_packet)
 * @param in_data   Input ciphertext
 * @param out_data  Output plaintext buffer (may equal in_data)
 * @param size      Data length
 * @param hash      Expected SHA-256 hash (first 8 bytes compared)
 * @return out_data on success, NULL on verification failure
 */
uint8_t *dfu_dec_verify(uint8_t *key, uint32_t offset,
                        uint8_t *in_data, uint8_t *out_data,
                        int size, uint8_t *hash);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_SEC_H__ */
