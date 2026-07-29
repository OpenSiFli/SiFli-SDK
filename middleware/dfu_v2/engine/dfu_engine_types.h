/**
 * @file dfu_engine_types.h
 * @brief DFU V2 Engine — Type Definitions
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_ENGINE_TYPES_H__
#define __DFU_ENGINE_TYPES_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Constants
 *============================================================================*/

/** Maximum number of images in one control packet (matches existing protocol) */
#define DFU_MAX_IMAGES           4

/** Firmware encryption key size in bytes */
#define DFU_KEY_SIZE             32

/** Firmware signature size in bytes */
#define DFU_SIG_SIZE             256

/*============================================================================
 * Engine State Machine
 *============================================================================*/

/**
 * @brief Engine state enumeration
 *
 * Transition map (trigger in parentheses):
 *
 *   IDLE ──(begin_session)──> SESSION
 *   SESSION ──(erase)──> ERASING ──(done)──> TRANSFERRING
 *   TRANSFERRING ──(write_data)──> TRANSFERRING   [loop]
 *   TRANSFERRING ──(verify)──> VERIFYING
 *   VERIFYING ──(success+commit)──> COMMITTING ──(done)──> COMPLETE
 *   VERIFYING ──(failure)──> ERROR
 *   Any ──(abort)──> IDLE
 *   Any ──(unrecoverable error)──> ERROR
 */
typedef enum
{
    DFU_ENGINE_IDLE = 0,        /**< No upgrade in progress */
    DFU_ENGINE_SESSION,         /**< Session established, awaiting erase */
    DFU_ENGINE_ERASING,         /**< Erasing target flash region */
    DFU_ENGINE_TRANSFERRING,    /**< Receiving and writing data */
    DFU_ENGINE_VERIFYING,       /**< Verifying CRC and optional signature */
    DFU_ENGINE_COMMITTING,      /**< Writing firmware info / update flags */
    DFU_ENGINE_COMPLETE,        /**< Upgrade completed successfully */
    DFU_ENGINE_ERROR,           /**< Unrecoverable error */
} dfu_engine_state_t;

/*============================================================================
 * Error Codes
 *============================================================================*/

/**
 * @brief Engine error codes
 *
 * DFU_ENGINE_ERR_NONE means success. All others are error conditions.
 * When state == ERROR, last_error in dfu_engine_status_t holds the cause.
 */
typedef enum
{
    DFU_ENGINE_ERR_NONE = 0,        /**< Success / no error */
    DFU_ENGINE_ERR_INVALID_STATE,   /**< API called in wrong state */
    DFU_ENGINE_ERR_INVALID_PARAM,   /**< NULL pointer or bad argument */
    DFU_ENGINE_ERR_FLASH_ERASE,     /**< Flash erase failed */
    DFU_ENGINE_ERR_FLASH_WRITE,     /**< Flash write failed */
    DFU_ENGINE_ERR_FLASH_READ,      /**< Flash read failed */
    DFU_ENGINE_ERR_CRC_MISMATCH,    /**< Running CRC != expected CRC */
    DFU_ENGINE_ERR_SIG_VERIFY,      /**< Firmware signature invalid */
    DFU_ENGINE_ERR_ABORTED,         /**< Abort flag was set */
    DFU_ENGINE_ERR_PROGRESS_DB,     /**< FlashDB read/write failed */
    DFU_ENGINE_ERR_OVERFLOW,        /**< Received data exceeds total_length */
} dfu_engine_err_t;

/*============================================================================
 * Image Descriptor
 *============================================================================*/

/**
 * @brief Single image descriptor within a firmware package
 *
 * One firmware package may contain multiple images (HCPU, LCPU, RES, etc.).
 * Each image has its own flash target and verification signature.
 */
typedef struct
{
    uint8_t     img_id;                     /**< Image ID (matches dfu_img_id_t in protocol) */
    uint16_t    flag;                       /**< Image flags (compress, encrypt, etc.) */
    uint32_t    img_length;                 /**< Image data length in bytes */
    uint32_t    flash_addr;                 /**< Target flash write address */
    uint32_t    flash_size;                 /**< Target flash partition size (for erase) */
    uint32_t    img_crc;                    /**< Per-image CRC32 (for verify) */
    uint8_t     sig[DFU_SIG_SIZE];       /**< Image signature (zeroed if unsigned) */
} dfu_image_info_t;

/*============================================================================
 * File Information (input to begin_session)
 *============================================================================*/

/**
 * @brief Firmware file descriptor
 *
 * Passed by Mode layer to dfu_engine_begin_session().
 * Contains everything Engine needs to manage one upgrade session.
 *
 * Multi-image upgrade: Mode calls begin_session() once per image,
 * populating only images[0] with img_count=1. Mode layer orchestrates
 * the image sequence; Engine handles one image at a time.
 */
typedef struct
{
    /* --- Identification (used for resume matching) --- */
    uint32_t    total_length;               /**< Total data length in bytes */
    uint32_t    total_packets;              /**< Total expected packet count */
    uint32_t    file_crc;                   /**< CRC32 of the complete data */

    /* --- Version info (for compatibility check) --- */
    uint32_t    hw_version;                 /**< Required hardware version */
    uint32_t    sdk_version;                /**< Required SDK version */
    uint32_t    fw_version;                 /**< New firmware version */

    /* --- Security --- */
    uint8_t     fw_key[DFU_KEY_SIZE];    /**< Encryption key (zeroed if unencrypted) */

    /* --- Transfer parameters --- */
    uint8_t     dfu_id;                     /**< DFU type (code / mix / res / etc.) */
    uint16_t    block_size;                 /**< Transfer block size */

    /* --- Image list --- */
    uint8_t     img_count;                  /**< Number of images (1 for per-image session) */
    dfu_image_info_t images[DFU_MAX_IMAGES]; /**< Image descriptors */
} dfu_file_info_t;

/*============================================================================
 * Resume Information (output from begin_session)
 *============================================================================*/

/**
 * @brief Resume status returned by begin_session()
 *
 * Engine compares (total_length, total_packets, file_crc) against FlashDB.
 * If all three match, the previous session was for the same firmware file
 * and resume is possible.
 */
typedef struct
{
    bool        can_resume;         /**< true = matching snapshot found */
    uint32_t    completed_packets;  /**< Packets already confirmed written */
    uint32_t    completed_bytes;    /**< Bytes already written to flash */
    uint8_t     current_img_id;     /**< Image ID where transfer was interrupted */
    uint8_t     current_img_index;  /**< Index into images[] to resume from */
} dfu_resume_info_t;

/*============================================================================
 * Progress Information
 *============================================================================*/

/**
 * @brief Transfer progress snapshot (read-only)
 */
typedef struct
{
    uint32_t    bytes_received;     /**< Total bytes received in this session */
    uint32_t    bytes_total;        /**< Total bytes expected */
    uint32_t    packets_received;   /**< Packets received so far */
    uint32_t    packets_total;      /**< Packets expected */
    uint8_t     percent;            /**< Completion percentage 0-100 */
} dfu_progress_t;

/*============================================================================
 * Combined Status (for APP layer query)
 *============================================================================*/

/**
 * @brief Complete engine status snapshot
 *
 * Returned by dfu_engine_get_status(). Contains everything APP layer
 * needs for LOG output or UI display.
 */
typedef struct
{
    dfu_engine_state_t  state;          /**< Current engine state */
    dfu_engine_err_t    last_error;     /**< Error code (valid when state==ERROR) */
    const char         *mode_name;      /**< "device-initiated" or "host-initiated" */
    const char         *transport_name; /**< "cdc", "pan", "ble", "uart", etc. */
    dfu_progress_t      progress;       /**< Current transfer progress */
} dfu_engine_status_t;

/*============================================================================
 * Callback Types
 *============================================================================*/

/**
 * @brief State change notification callback
 *
 * Invoked synchronously in the caller's thread context.
 * Keep the implementation short (e.g., rt_mb_send, set flag).
 *
 * @param old_state  State before transition
 * @param new_state  State after transition
 * @param error      Error code (meaningful only when new_state == ERROR)
 * @param user_data  User context pointer set at init time
 */
typedef void (*dfu_engine_state_cb_t)(dfu_engine_state_t old_state,
                                      dfu_engine_state_t new_state,
                                      dfu_engine_err_t error,
                                      void *user_data);

/**
 * @brief Progress notification callback
 *
 * Invoked periodically during TRANSFERRING state.
 * Engine throttles internally — NOT called on every write_data().
 *
 * @param progress   Progress snapshot
 * @param user_data  User context pointer set at init time
 */
typedef void (*dfu_engine_progress_cb_t)(const dfu_progress_t *progress,
                                         void *user_data);

/*============================================================================
 * Engine Init Configuration
 *============================================================================*/

/**
 * @brief Engine initialization parameters
 *
 * All fields optional. Zeroed struct means: no callbacks, default interval.
 */
typedef struct
{
    dfu_engine_state_cb_t       on_state_change;    /**< State change callback (or NULL) */
    dfu_engine_progress_cb_t    on_progress;        /**< Progress callback (or NULL) */
    void                       *user_data;          /**< Context for both callbacks */
    uint32_t                    progress_interval;  /**< Callback interval in bytes (0 = 4096) */
} dfu_engine_config_t;

#ifdef __cplusplus
}
#endif

#endif /* __DFU_ENGINE_TYPES_H__ */