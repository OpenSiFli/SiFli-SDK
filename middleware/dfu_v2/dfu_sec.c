/**
 * @file dfu_sec.c
 * @brief DFU V2 — Minimal security primitives for ctrl_packet decryption
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include "board.h"

#ifdef PKG_SIFLI_MBEDTLS_BOOT
    #ifndef BSP_USING_HW_AES
        #include "mbedtls/aes.h"
    #endif
    #include "mbedtls/sha256.h"
#endif

#define LOG_TAG "dfu.v2sec"
#include <log.h>

/*============================================================================
 * Constants (matching dfu.h definitions)
 *============================================================================*/

#ifndef DFU_UID_SIZE
#define DFU_UID_SIZE            16
#endif

#ifndef DFU_KEY_SIZE
#define DFU_KEY_SIZE            32
#endif

#ifndef DFU_SIG_HASH_SIZE
#define DFU_SIG_HASH_SIZE       8
#endif

#ifndef DFU_IV_LEN
#define DFU_IV_LEN              16
#endif

/* eFuse key IDs */
#ifndef EFUSE_UID
#define EFUSE_UID               1
#endif

#ifndef EFUSE_ID_ROOT
#define EFUSE_ID_ROOT           2
#endif

#ifndef EFUSE_ID_SIG_HASH
#define EFUSE_ID_SIG_HASH       3
#endif

#ifndef EFUSE_ID_SECURE_ENABLED
#define EFUSE_ID_SECURE_ENABLED 4
#endif

#ifndef DFU_SECURE_SIZE
#define DFU_SECURE_SIZE         1
#endif

#ifndef DFU_SECURE_PATTERN
#define DFU_SECURE_PATTERN      0xA5
#endif

/*============================================================================
 * eFuse hook — allows fake keys for dev boards without eFuse programmed
 *============================================================================*/

typedef int (*dfu_efuse_read_hook_t)(uint8_t id, uint8_t *data, int size);

/**
 * Fake keys for development boards without eFuse programmed.
 * These MUST match the keys used by imgtoolv37 (sifli01/ directory).
 * In production, real keys come from eFuse hardware.
 */
static const uint8_t g_fake_uid[DFU_UID_SIZE] =
{
    0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08,
    0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00
};

static const uint8_t g_fake_sig_hash[DFU_SIG_HASH_SIZE] =
{
    0xF0, 0xD7, 0x77, 0x7F, 0x61, 0x6F, 0x7C, 0x89
};

static const uint8_t g_fake_root_key[DFU_KEY_SIZE] =
{
    0xAB, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88,
    0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static int dfu_fake_efuse_hook(uint8_t id, uint8_t *data, int size)
{
    if (id == EFUSE_UID)
    {
        memcpy(data, g_fake_uid, DFU_UID_SIZE);
        return DFU_UID_SIZE;
    }
    else if (id == EFUSE_ID_SIG_HASH)
    {
        memcpy(data, g_fake_sig_hash, DFU_SIG_HASH_SIZE);
        return DFU_SIG_HASH_SIZE;
    }
    else if (id == EFUSE_ID_ROOT)
    {
        memcpy(data, g_fake_root_key, DFU_KEY_SIZE);
        return DFU_KEY_SIZE;
    }
    return 0;
}

/*============================================================================
 * eFuse read — tries real eFuse first, falls back to fake hook
 *============================================================================*/

/* Global hook — set to fake_efuse_hook if secure boot is not enabled */
static dfu_efuse_read_hook_t g_efuse_hook = NULL;

static int dfu_efuse_read(uint8_t id, uint8_t *data, int size)
{
    if (g_efuse_hook)
    {
        int r = g_efuse_hook(id, data, size);
        if (r != 0)
            return r;
    }

#ifdef HW_EFUSE
    /* Read from real eFuse hardware */
    #define EFUSE_OFFSET_UID        0
    #define EFUSE_OFFSET_SIG_HASH   128
    #define EFUSE_OFFSET_ROOT       768

    if (id == EFUSE_UID)
        return HAL_EFUSE_Read(EFUSE_OFFSET_UID, data, DFU_UID_SIZE);
    else if (id == EFUSE_ID_SIG_HASH)
        return HAL_EFUSE_Read(EFUSE_OFFSET_SIG_HASH, data, DFU_SIG_HASH_SIZE);
    else if (id == EFUSE_ID_ROOT)
        return HAL_EFUSE_Read(EFUSE_OFFSET_ROOT, data, DFU_KEY_SIZE);
#endif

    return 0;
}

/*============================================================================
 * AES-CTR counter construction
 *============================================================================*/

ALIGN(4)
static uint8_t g_aes_ctr_iv[DFU_IV_LEN];

/**
 * @brief Build AES-CTR initial counter value
 *
 * Layout (16 bytes):
 *   [0..7]   sig_hash from eFuse (8 bytes)
 *   [8..11]  zeros (4 bytes)
 *   [12..15] offset/16 in big-endian (4 bytes)
 *
 * This matches imgtoolv37.py:
 *   cnt_prefix = sig_hash_bin[0:8] + b'\x00'*4
 *   Counter.new(32, prefix=cnt_prefix, initial_value=0)
 */
static uint8_t *dfu_get_counter(uint32_t offset)
{
    dfu_efuse_read(EFUSE_ID_SIG_HASH, g_aes_ctr_iv, DFU_SIG_HASH_SIZE);
    memset(&g_aes_ctr_iv[8], 0, 8);

    offset >>= 4;  /* Counter increments every 16 bytes */
    for (int i = 15; i >= 12 && offset > 0; i--, offset >>= 8)
    {
        g_aes_ctr_iv[i] = offset & 0xff;
    }
    return g_aes_ctr_iv;
}

/*============================================================================
 * AES-CTR decryption
 *============================================================================*/

ALIGN(4)
static uint8_t g_root_key[DFU_KEY_SIZE];

/**
 * @brief AES-CTR decrypt
 *
 * If key is NULL, reads root key from eFuse (or fake key on dev boards).
 * Uses HW AES accelerator if available, otherwise mbedtls software AES.
 *
 * Matches sifli_hw_dec() in efuse.c exactly.
 */
static int dfu_aes_ctr_decrypt(uint8_t *key, uint8_t *in_data,
                                   uint8_t *out_data, int size,
                                   uint32_t init_offset)
{
#if defined(OTA_55X) || defined(OTA_56X_NAND)
    uint32_t offset = 0;

    /* If no key provided, read root key from eFuse */
    if (g_efuse_hook && !key)
    {
        g_efuse_hook(EFUSE_ID_ROOT, g_root_key, DFU_KEY_SIZE);
        key = g_root_key;
    }

#ifdef BSP_USING_HW_AES
    {
        #define V2_AES_BLOCK_SIZE 512
        static uint8_t temp[V2_AES_BLOCK_SIZE];
        memset(temp, 0, V2_AES_BLOCK_SIZE);
        while (offset < (uint32_t)size)
        {
            int len = (size - (int)offset) < V2_AES_BLOCK_SIZE
                      ? (size - (int)offset) : V2_AES_BLOCK_SIZE;
            memcpy(temp, in_data + offset, len);
            HAL_AES_init((uint32_t *)key, DFU_KEY_SIZE,
                         (uint32_t *)dfu_get_counter(init_offset + offset),
                         AES_MODE_CTR);
            HAL_AES_run(AES_DEC, temp, out_data + offset, len);
            offset += len;
        }
    }
#else
    {
#ifdef PKG_SIFLI_MBEDTLS_BOOT
        static mbedtls_aes_context ctx;
        static uint8_t stream_block[16];
        mbedtls_aes_init(&ctx);
        mbedtls_aes_setkey_enc(&ctx, key, DFU_KEY_SIZE * 8);
        mbedtls_aes_crypt_ctr(&ctx, size, (size_t *)&offset,
                              dfu_get_counter(init_offset),
                              stream_block, in_data, out_data);
#endif
    }
#endif
    return offset;
#else
    (void)key; (void)in_data; (void)out_data; (void)size; (void)init_offset;
    LOG_E("AES decrypt not available (OTA_55X/OTA_56X_NAND not defined)");
    return 0;
#endif
}

/*============================================================================
 * Public API: dfu_dec_verify — decrypt + SHA-256 verify
 *============================================================================*/

/**
 * @brief Decrypt data with AES-CTR and verify SHA-256 hash
 *
 * This is the V2 equivalent of dfu_dec_verify() in dfu_sec.c.
 * Called by translate_init_req() to decrypt the control packet.
 *
 * @param key       AES key (NULL = use root key from eFuse)
 * @param offset    AES-CTR initial offset (0 for ctrl_packet)
 * @param in_data   Input ciphertext (will be overwritten if in_data == out_data)
 * @param out_data  Output plaintext buffer
 * @param size      Data length
 * @param hash      Expected SHA-256 hash (first 8 bytes compared)
 * @return out_data on success, NULL on verification failure
 */
uint8_t *dfu_dec_verify(uint8_t *key, uint32_t offset,
                        uint8_t *in_data, uint8_t *out_data,
                        int size, uint8_t *hash)
{
#ifdef PKG_SIFLI_MBEDTLS_BOOT
    uint8_t *r = NULL;

    /* Allocate aligned temp buffer for AES (DMA requirement) */
    int align_size = (size / 16 + 1) * 16;
    uint8_t *temp_out = rt_malloc(align_size);
    if (!temp_out)
    {
        LOG_E("dfu_dec_verify: malloc(%d) failed", align_size);
        return NULL;
    }

    SCB_InvalidateDCache_by_Addr((uint32_t *)temp_out, align_size);
    SCB_InvalidateICache_by_Addr((uint32_t *)temp_out, align_size);

    /* AES-CTR decrypt */
    dfu_aes_ctr_decrypt(key, in_data, temp_out, size, offset);
    memcpy(out_data, temp_out, size);
    rt_free(temp_out);

    /* SHA-256 verify (compare first DFU_SIG_HASH_SIZE=8 bytes) */
    mbedtls_sha256_context ctx;
    uint8_t computed_hash[32] = {0};

    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);  /* SHA-256, not SHA-224 */
    mbedtls_sha256_update(&ctx, out_data, size);
    mbedtls_sha256_finish(&ctx, computed_hash);
    mbedtls_sha256_free(&ctx);

    if (memcmp(computed_hash, hash, DFU_SIG_HASH_SIZE) == 0)
    {
        r = out_data;
        LOG_I("ctrl_packet hash verified OK");
    }
    else
    {
        LOG_E("ctrl_packet hash mismatch");
        LOG_E("  expected: %02x%02x%02x%02x%02x%02x%02x%02x",
              hash[0], hash[1], hash[2], hash[3],
              hash[4], hash[5], hash[6], hash[7]);
        LOG_E("  computed: %02x%02x%02x%02x%02x%02x%02x%02x",
              computed_hash[0], computed_hash[1], computed_hash[2], computed_hash[3],
              computed_hash[4], computed_hash[5], computed_hash[6], computed_hash[7]);
    }

    return r;
#else
    LOG_E("dfu_dec_verify: PKG_SIFLI_MBEDTLS_BOOT not enabled");
    return NULL;
#endif
}

/*============================================================================
 * Init — must be called once at startup
 *============================================================================*/

/**
 * @brief Initialize V2 security subsystem
 *
 * Checks if secure boot is enabled. If not, installs the fake eFuse
 * hook so that dev boards work with the default imgtoolv37 keys.
 *
 * Call this from dfu_init() or early in main().
 */
void dfu_sec_init(void)
{
#ifdef HW_EFUSE
    /* Check if secure boot is enabled in eFuse */
    uint8_t pattern = 0;
    int len = 0;

    /* Try reading eFuse directly first (before hook is set) */
    #ifndef EFUSE_OFFSET_SECURE
    #define EFUSE_OFFSET_SECURE 192
    #endif

    uint32_t temp_data = 0;
    len = HAL_EFUSE_Read(EFUSE_OFFSET_SECURE, (uint8_t *)&temp_data, 4);
    if (len == 4)
        pattern = (uint8_t)(temp_data & 0xFF);

    if (pattern != DFU_SECURE_PATTERN)
    {
        /* Secure boot NOT enabled → use fake keys for dev board */
        g_efuse_hook = dfu_fake_efuse_hook;
        LOG_I("V2 sec: using dev keys (secure boot not enabled)");
    }
    else
    {
        LOG_I("V2 sec: secure boot enabled, using eFuse keys");
    }
#else
    /* No eFuse hardware → always use fake keys */
    g_efuse_hook = dfu_fake_efuse_hook;
    LOG_I("V2 sec: using dev keys (no HW_EFUSE)");
#endif
}
