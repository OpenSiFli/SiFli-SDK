/**
 * @file dfu_engine.h
 * @brief DFU V2 Engine — Core Interface
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DFU_ENGINE_H__
#define __DFU_ENGINE_H__

#include "dfu_engine_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Lifecycle
 *============================================================================*/

/**
 * @brief Initialize the DFU Engine
 *
 * Call once at startup before any other Engine API.
 * Internally initializes: Flash abstraction layer, FlashDB for progress
 * persistence, NAND cache module, and internal state machine.
 *
 * @param config  Configuration (callbacks, progress interval). May be NULL
 *                for defaults (no callbacks, 4096-byte progress interval).
 * @return DFU_ENGINE_ERR_NONE on success
 */
dfu_engine_err_t dfu_engine_init(const dfu_engine_config_t *config);

/**
 * @brief Deinitialize the DFU Engine
 *
 * Releases all resources. If a session is active, aborts it first.
 * Safe to call in any state, including before init (no-op).
 */
void dfu_engine_deinit(void);

/*============================================================================
 * Session Management
 *============================================================================*/

/**
 * @brief Begin a new upgrade session for one image
 *
 * State: IDLE/COMPLETE -> SESSION
 *
 * 1. Receives file metadata from Mode layer.
 * 2. Checks FlashDB for a matching progress snapshot:
 *    Match key = (total_length, total_packets, file_crc).
 *    - Match found:  can_resume=true, completed_* fields populated.
 *    - No match:     can_resume=false, old progress cleared.
 * 3. Stores mode_name and transport_name for status queries.
 *
 * @param file_info       File metadata for this image (must not be NULL)
 * @param mode_name       String identifier, e.g. "device-initiated" (stored as pointer)
 * @param transport_name  String identifier, e.g. "cdc" (stored as pointer)
 * @param[out] resume     Resume info output (must not be NULL)
 * @return DFU_ENGINE_ERR_NONE on success
 *         DFU_ENGINE_ERR_INVALID_STATE if not in IDLE or COMPLETE
 *         DFU_ENGINE_ERR_INVALID_PARAM if file_info or resume is NULL
 */
dfu_engine_err_t dfu_engine_begin_session(const dfu_file_info_t *file_info,
                                           const char *mode_name,
                                           const char *transport_name,
                                           dfu_resume_info_t *resume);

/*============================================================================
 * Flash Operations
 *============================================================================*/

/**
 * @brief Erase the target flash region
 *
 * State: SESSION -> ERASING -> TRANSFERRING
 *
 * Uses the flash_addr and flash_size from the current image's
 * dfu_image_info_t. Automatically detects NOR/NAND and aligns erase
 * boundaries. Initializes NAND cache if target is on NAND flash.
 *
 * If resuming (can_resume was true) and target region is already erased,
 * skips the erase and transitions directly to TRANSFERRING.
 *
 * @return DFU_ENGINE_ERR_NONE on success
 *         DFU_ENGINE_ERR_INVALID_STATE if not in SESSION
 *         DFU_ENGINE_ERR_FLASH_ERASE on erase failure
 *         DFU_ENGINE_ERR_ABORTED if abort was requested
 */
dfu_engine_err_t dfu_engine_erase(void);

/**
 * @brief Write a chunk of firmware data to flash
 *
 * State: stays in TRANSFERRING
 *
 * Internally:
 *   1. Checks abort flag — returns ERR_ABORTED if set.
 *   2. Writes data via dfu_flash_write() (NOR/NAND transparent).
 *   3. Updates running CRC32 and bytes_received counter.
 *   4. Increments packets_received.
 *   5. Periodically persists progress to FlashDB.
 *   6. Calls progress callback if interval threshold is reached.
 *
 * @param data    Pointer to data buffer (must not be NULL)
 * @param len     Number of bytes to write (must be > 0)
 * @return DFU_ENGINE_ERR_NONE on success
 *         DFU_ENGINE_ERR_INVALID_STATE if not in TRANSFERRING
 *         DFU_ENGINE_ERR_FLASH_WRITE on write failure
 *         DFU_ENGINE_ERR_OVERFLOW if total would exceed total_length
 *         DFU_ENGINE_ERR_ABORTED if abort was requested
 */
dfu_engine_err_t dfu_engine_write_data(const uint8_t *data, uint32_t len);

/*============================================================================
 * Verification and Commit
 *============================================================================*/

/**
 * @brief Verify firmware integrity
 *
 * State: TRANSFERRING -> VERIFYING -> (success: awaiting commit / failure: ERROR)
 *
 * Steps:
 *   1. Flushes NAND cache (if applicable).
 *   2. Compares running CRC32 with expected_crc.
 *   3. If secure boot is enabled, performs signature verification
 *      using the image signature and secboot module.
 *   4. On failure, transitions to ERROR.
 *
 * @param expected_crc  Expected CRC32 of the transferred data.
 *                      For Host Mode, this comes from IMG_SEND_END payload.
 *                      For Device Mode, this comes from file_info.file_crc.
 * @return DFU_ENGINE_ERR_NONE if all verifications passed
 *         DFU_ENGINE_ERR_CRC_MISMATCH if CRC check failed
 *         DFU_ENGINE_ERR_SIG_VERIFY if signature check failed
 *         DFU_ENGINE_ERR_INVALID_STATE if not in TRANSFERRING
 */
dfu_engine_err_t dfu_engine_verify(uint32_t expected_crc);

/**
 * @brief Get the finalized running CRC32 of data written so far
 *
 * Returns the CRC32 that Engine has accumulated during write_data() calls,
 * finalized for comparison. This is primarily used by BLE Transport,
 * where the legacy protocol does not provide a per-image CRC from the
 * host, so Transport passes Engine's own CRC back to verify() to allow
 * the verification step to succeed (actual integrity is guaranteed by
 * per-packet hash verification in the BLE Transport layer).
 *
 * @return Finalized CRC32 value
 */
uint32_t dfu_engine_get_running_crc(void);

/**
 * @brief Commit the upgrade
 *
 * State: VERIFYING -> COMMITTING -> COMPLETE
 *
 * Steps:
 *   1. Writes firmware info (image header, update flags) to the
 *      designated flash location.
 *   2. Clears progress record from FlashDB.
 *   3. Deinitializes NAND cache.
 *   4. Transitions to COMPLETE.
 *
 * @return DFU_ENGINE_ERR_NONE on success
 *         DFU_ENGINE_ERR_INVALID_STATE if not in VERIFYING
 *         DFU_ENGINE_ERR_FLASH_WRITE on write failure
 */
dfu_engine_err_t dfu_engine_commit(void);

/*============================================================================
 * Abort and Status (thread-safe)
 *============================================================================*/

/**
 * @brief Abort the current upgrade session
 *
 * Sets an internal abort flag only. The flag is checked on the NEXT
 * Engine API call (write_data, erase, verify, etc.) via engine_check_abort(),
 * which then performs the actual cleanup:
 *   - NAND cache deinitialized
 *   - FlashDB progress saved (for potential resume)
 *   - State reset to IDLE
 *
 * This two-phase design ensures abort() itself is ISR-safe (no blocking
 * operations), while cleanup runs in the main-loop context.
 *
 * Safe to call from any thread or ISR context.
 *
 * @return DFU_ENGINE_ERR_NONE always
 */
dfu_engine_err_t dfu_engine_abort(void);

/**
 * @brief Synchronously reset the current session back to IDLE
 *
 * Unlike abort() which is async (flag-based), this function immediately
 * cleans up the session and returns to IDLE. FlashDB progress is NOT
 * cleared, so a subsequent begin_session() can still detect resume.
 *
 * NOT safe for ISR — must be called from main-loop context only.
 * Intended for Mode-layer use during resume probing (trying multiple
 * images to find a matching snapshot).
 *
 * @return DFU_ENGINE_ERR_NONE on success (or already IDLE, no-op)
 */
dfu_engine_err_t dfu_engine_reset_session(void);

/**
 * @brief Force-clear the progress snapshot in FlashDB.
 * Used by FORCE_INIT_REQ to discard stale resume state.
 */
void dfu_engine_force_clear_progress(void);

/**
 * @brief Get current engine status snapshot
 *
 * No state transition. Returns a copy of the internal state including
 * current state, last error, mode/transport names, and progress.
 *
 * Safe to call from any thread or ISR context.
 *
 * @param[out] status  Output structure (must not be NULL)
 * @return DFU_ENGINE_ERR_NONE on success
 *         DFU_ENGINE_ERR_INVALID_PARAM if status is NULL
 */
dfu_engine_err_t dfu_engine_get_status(dfu_engine_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_ENGINE_H__ */