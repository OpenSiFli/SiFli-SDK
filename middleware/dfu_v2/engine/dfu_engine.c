/**
 * @file dfu_engine.c
 * @brief DFU V2 Engine — State Machine and Core Operations
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <string.h>
#include "dfu_engine.h"
#include "dfu_engine_flash.h"

/* FlashDB for progress persistence */
#include "flashdb.h"
#include "dfu_fwinfo.h"   /* dfu_fw_info, dfu_core_set_* compat */

/*============================================================================
 * CRC32 — internalized (was dfu_core.h)
 *
 * Polynomial: 0xEDB88320 (reflected), Init: 0xFFFFFFFF, XorOut: 0xFFFFFFFF
 *============================================================================*/

#define CRC32_POLY  0xEDB88320
#define CRC32_INIT  0xFFFFFFFF

static uint32_t s_crc32_table[256];
static int s_crc32_ready = 0;

static void crc32_init(void)
{
    if (s_crc32_ready) return;
    for (int i = 0; i < 256; i++)
    {
        uint32_t c = (uint32_t)i;
        for (int j = 0; j < 8; j++)
            c = (c & 1) ? ((c >> 1) ^ CRC32_POLY) : (c >> 1);
        s_crc32_table[i] = c;
    }
    s_crc32_ready = 1;
}

static uint32_t crc32_update(const uint8_t *data, uint32_t len, uint32_t crc)
{
    if (!s_crc32_ready) crc32_init();
    for (uint32_t i = 0; i < len; i++)
        crc = (crc >> 8) ^ s_crc32_table[(crc ^ data[i]) & 0xFF];
    return crc;
}

static uint32_t crc32_finalize(uint32_t crc)
{
    return crc ^ 0xFFFFFFFF;
}

/* Signature verification (secure boot) */
#ifdef BSP_USING_SECBOOT
#include "dfu_internal.h"   /* dfu_ctrl_ctrl_header_sig_verify_ext() */
#endif

#define LOG_TAG "dfu.engine"
#include <log.h>

/*============================================================================
 * FlashDB Configuration
 *============================================================================*/

#define DFU_DB_NAME          "SIF_DFU2"
#define DFU_DB_PARTITION     "dfu"
#define DFU_PROGRESS_KEY     "dfu2_prog"

/** Default progress interval: persist + callback every 4096 bytes */
#define DFU_DEFAULT_PROGRESS_INTERVAL    4096

/*============================================================================
 * Progress Snapshot (persisted to FlashDB)
 *============================================================================*/

/**
 * Internal progress snapshot stored in FlashDB.
 * Used to detect whether resume is possible on next boot.
 */
typedef struct
{
    /* --- Match key: if all three match, resume is possible --- */
    uint32_t    total_length;
    uint32_t    total_packets;
    uint32_t    file_crc;

    /* --- Resume position --- */
    uint32_t    completed_bytes;
    uint32_t    completed_packets;
    uint32_t    write_addr;             /**< Next flash write address */
    uint32_t    running_crc;            /**< CRC32 state at this point */
    uint8_t     current_img_id;
    uint8_t     current_img_index;

    /* --- Validity marker --- */
    uint32_t    magic;                  /**< 0xDFE2A55A = valid snapshot */
} dfu_progress_snapshot_t;

#define DFU_SNAPSHOT_MAGIC   0xDFE2A55A

/*============================================================================
 * Engine Context (singleton)
 *============================================================================*/

typedef struct
{
    /* --- Lifecycle --- */
    bool                    initialized;

    /* --- State machine --- */
    dfu_engine_state_t      state;
    dfu_engine_err_t        last_error;
    volatile uint8_t        abort_flag;     /**< Set from any thread */

    /* --- Session data (valid when state >= SESSION) --- */
    dfu_file_info_t         file_info;
    const char             *mode_name;
    const char             *transport_name;
    bool                    resuming;       /**< True if begin_session found a match */

    /* --- Transfer tracking --- */
    uint32_t                write_addr;     /**< Next flash address to write */
    uint32_t                bytes_received;
    uint32_t                packets_received;
    uint32_t                running_crc;    /**< Streaming CRC32 accumulator */
    uint32_t                verified_crc;   /**< Per-image CRC confirmed by verify() */

    /* --- Current image info shortcut --- */
    uint8_t                 img_index;      /**< Index into file_info.images[] */
    const dfu_image_info_t *cur_img;        /**< Pointer into file_info.images[img_index] */

    /* --- Callbacks --- */
    dfu_engine_state_cb_t   on_state_change;
    dfu_engine_progress_cb_t on_progress;
    void                   *user_data;
    uint32_t                progress_interval;
    uint32_t                last_progress_bytes; /**< bytes_received at last callback */

    /* --- FlashDB --- */
    struct fdb_kvdb         kvdb;
    bool                    db_ready;
} dfu_engine_ctx_t;

static dfu_engine_ctx_t g_engine;

/*============================================================================
 * Internal Helpers — Forward Declarations
 *============================================================================*/

static bool engine_check_abort(void);
static void engine_cleanup(bool clear_progress);

/*============================================================================
 * Internal Helpers
 *============================================================================*/

/**
 * @brief Transition state and fire callback
 */
static void engine_set_state(dfu_engine_state_t new_state, dfu_engine_err_t err)
{
    dfu_engine_state_t old = g_engine.state;
    if (old == new_state)
        return;

    g_engine.state = new_state;
    if (new_state == DFU_ENGINE_ERROR)
        g_engine.last_error = err;

    LOG_I("state: %d -> %d (err=%d)", old, new_state, err);

    if (g_engine.on_state_change)
        g_engine.on_state_change(old, new_state, err, g_engine.user_data);
}

/**
 * @brief Check if abort was requested
 */
static inline bool engine_is_aborted(void)
{
    return (g_engine.abort_flag != 0);
}

/**
 * @brief Compute current progress percentage
 */
static inline uint8_t engine_calc_percent(void)
{
    if (g_engine.file_info.total_length == 0)
        return 0;
    return (uint8_t)((uint64_t)g_engine.bytes_received * 100
                     / g_engine.file_info.total_length);
}

/**
 * @brief Fill a progress struct from current state
 */
static void engine_fill_progress(dfu_progress_t *p)
{
    p->bytes_received  = g_engine.bytes_received;
    p->bytes_total     = g_engine.file_info.total_length;
    p->packets_received = g_engine.packets_received;
    p->packets_total   = g_engine.file_info.total_packets;
    p->percent         = engine_calc_percent();
}

/*============================================================================
 * FlashDB Progress Persistence
 *============================================================================*/

static dfu_engine_err_t progress_db_init(void)
{
#ifdef PKG_USING_FLASHDB
    static const uint8_t default_prog_data[] = {0x0};
    struct fdb_default_kv_node default_kv_table[] = {
        {DFU_PROGRESS_KEY, (void *)default_prog_data, sizeof(default_prog_data)},
    };
    struct fdb_default_kv default_kv = {
        .kvs = default_kv_table,
        .num = sizeof(default_kv_table) / sizeof(default_kv_table[0]),
    };

    /* V1.5 pattern: memset kvdb struct to zero, then init directly.
     * Do NOT call fdb_kvdb_control — V1.5 doesn't in partition mode,
     * and calling it under PKG_FDB_USING_AUTO_MODE can trigger file_mode path. */
    memset(&g_engine.kvdb, 0, sizeof(g_engine.kvdb));

    fdb_err_t ret = fdb_kvdb_init(&g_engine.kvdb, DFU_DB_NAME, DFU_DB_PARTITION, &default_kv, NULL);
    if (ret != FDB_NO_ERR)
    {
        LOG_E("FlashDB init failed: %d", ret);
        return DFU_ENGINE_ERR_PROGRESS_DB;
    }
    g_engine.db_ready = true;
    return DFU_ENGINE_ERR_NONE;
#else
    #error "FlashDB (PKG_USING_FLASHDB) must be enabled for DFU V2"
#endif
}

static dfu_engine_err_t progress_save(void)
{
    if (!g_engine.db_ready)
        return DFU_ENGINE_ERR_PROGRESS_DB;

    dfu_progress_snapshot_t snap = {0};
    snap.total_length     = g_engine.file_info.total_length;
    snap.total_packets    = g_engine.file_info.total_packets;
    snap.file_crc         = g_engine.file_info.file_crc;
    snap.completed_bytes  = g_engine.bytes_received;
    snap.completed_packets = g_engine.packets_received;
    snap.write_addr       = g_engine.write_addr;
    snap.running_crc      = g_engine.running_crc;
    snap.current_img_id   = g_engine.cur_img ? g_engine.cur_img->img_id : 0;
    snap.current_img_index = g_engine.img_index;
    snap.magic            = DFU_SNAPSHOT_MAGIC;

    struct fdb_blob blob;
    fdb_err_t err = fdb_kv_set_blob(&g_engine.kvdb, DFU_PROGRESS_KEY,
                                     fdb_blob_make(&blob, &snap, sizeof(snap)));
    if (err != FDB_NO_ERR)
    {
        LOG_E("progress save failed: %d", err);
        return DFU_ENGINE_ERR_PROGRESS_DB;
    }
    return DFU_ENGINE_ERR_NONE;
}

static dfu_engine_err_t progress_load(dfu_progress_snapshot_t *snap)
{
    if (!g_engine.db_ready)
        return DFU_ENGINE_ERR_PROGRESS_DB;

    struct fdb_blob blob;
    size_t read_len = fdb_kv_get_blob(&g_engine.kvdb, DFU_PROGRESS_KEY,
                                       fdb_blob_make(&blob, snap, sizeof(*snap)));
    if (read_len != sizeof(*snap) || snap->magic != DFU_SNAPSHOT_MAGIC)
    {
        memset(snap, 0, sizeof(*snap));
        return DFU_ENGINE_ERR_PROGRESS_DB;
    }
    return DFU_ENGINE_ERR_NONE;
}

static void progress_clear(void)
{
    if (!g_engine.db_ready)
        return;

    dfu_progress_snapshot_t empty = {0};
    struct fdb_blob blob;
    fdb_kv_set_blob(&g_engine.kvdb, DFU_PROGRESS_KEY,
                     fdb_blob_make(&blob, &empty, sizeof(empty)));
}

/*============================================================================
 * Session Cleanup Helper
 *============================================================================*/

static void engine_cleanup(bool clear_progress)
{
    dfu_ef_cache_flush();
    dfu_ef_cache_deinit();

    if (clear_progress)
        progress_clear();

    g_engine.abort_flag = 0;
    g_engine.resuming = false;
    g_engine.cur_img = NULL;
    g_engine.img_index = 0;
    g_engine.write_addr = 0;
    g_engine.bytes_received = 0;
    g_engine.packets_received = 0;
    g_engine.running_crc = CRC32_INIT;
    g_engine.verified_crc = 0;
    g_engine.last_progress_bytes = 0;
}

/*============================================================================
 * Public API: Lifecycle
 *============================================================================*/

dfu_engine_err_t dfu_engine_init(const dfu_engine_config_t *config)
{
    if (g_engine.initialized)
        return DFU_ENGINE_ERR_NONE;

    memset(&g_engine, 0, sizeof(g_engine));
    g_engine.state = DFU_ENGINE_IDLE;
    g_engine.running_crc = CRC32_INIT;

    /* Apply configuration */
    if (config)
    {
        g_engine.on_state_change = config->on_state_change;
        g_engine.on_progress     = config->on_progress;
        g_engine.user_data       = config->user_data;
        g_engine.progress_interval = config->progress_interval;
    }
    if (g_engine.progress_interval == 0)
        g_engine.progress_interval = DFU_DEFAULT_PROGRESS_INTERVAL;

    /* Init CRC32 table */
    crc32_init();

    /* Init flash abstraction layer */
    dfu_engine_err_t err = dfu_ef_init();
    if (err != DFU_ENGINE_ERR_NONE)
    {
        LOG_E("flash init failed");
        return err;
    }

    /* Init FlashDB for progress persistence */
    err = progress_db_init();
    if (err != DFU_ENGINE_ERR_NONE)
    {
        LOG_E("progress DB init failed");
        return err;
    }

    g_engine.initialized = true;
    LOG_I("engine initialized");
    return DFU_ENGINE_ERR_NONE;
}

void dfu_engine_deinit(void)
{
    if (!g_engine.initialized)
        return;

    /* Abort any active session — deinit always runs in main thread,
       so we can do cleanup directly (unlike ISR-safe abort()). */
    if (g_engine.state != DFU_ENGINE_IDLE)
    {
        engine_cleanup(true);
        g_engine.state = DFU_ENGINE_IDLE;
    }

    dfu_ef_deinit();
    g_engine.initialized = false;
    LOG_I("engine deinitialized");
}

/*============================================================================
 * Public API: begin_session
 *============================================================================*/

dfu_engine_err_t dfu_engine_begin_session(const dfu_file_info_t *file_info,
                                           const char *mode_name,
                                           const char *transport_name,
                                           dfu_resume_info_t *resume)
{
    if (!file_info || !resume)
        return DFU_ENGINE_ERR_INVALID_PARAM;

    if (g_engine.state != DFU_ENGINE_IDLE &&
        g_engine.state != DFU_ENGINE_COMPLETE)
        return DFU_ENGINE_ERR_INVALID_STATE;

    /* If coming from COMPLETE (multi-image switch), reset to IDLE first */
    if (g_engine.state == DFU_ENGINE_COMPLETE)
    {
        LOG_I("begin_session: resetting from COMPLETE for next image");
        g_engine.state = DFU_ENGINE_IDLE;
    }

    /* Store session parameters */
    memcpy(&g_engine.file_info, file_info, sizeof(dfu_file_info_t));
    g_engine.mode_name = mode_name;
    g_engine.transport_name = transport_name;

    /* Default: first image in the list */
    g_engine.img_index = 0;
    g_engine.cur_img = &g_engine.file_info.images[0];

    /* Reset transfer tracking */
    g_engine.write_addr = g_engine.cur_img->flash_addr;
    g_engine.bytes_received = 0;
    g_engine.packets_received = 0;
    g_engine.running_crc = CRC32_INIT;
    g_engine.verified_crc = 0;
    g_engine.last_progress_bytes = 0;
    g_engine.abort_flag = 0;
    g_engine.resuming = false;

    /* Initialize resume output */
    memset(resume, 0, sizeof(dfu_resume_info_t));

    /* Check FlashDB for matching progress snapshot */
    dfu_progress_snapshot_t snap;
    if (progress_load(&snap) == DFU_ENGINE_ERR_NONE)
    {
        /* Compare match key: (total_length, total_packets, file_crc) */
        if (snap.total_length  == file_info->total_length  &&
            snap.total_packets == file_info->total_packets  &&
            snap.file_crc      == file_info->file_crc       &&
            snap.completed_bytes > 0)
        {
            /* Resume possible */
            resume->can_resume       = true;
            resume->completed_packets = snap.completed_packets;
            resume->completed_bytes  = snap.completed_bytes;
            resume->current_img_id   = snap.current_img_id;
            resume->current_img_index = snap.current_img_index;

            /* Restore internal state to resume point */
            g_engine.write_addr       = snap.write_addr;
            g_engine.bytes_received   = snap.completed_bytes;
            g_engine.packets_received = snap.completed_packets;
            g_engine.running_crc      = snap.running_crc;
            g_engine.img_index        = snap.current_img_index;
            g_engine.cur_img          = &g_engine.file_info.images[snap.current_img_index];
            g_engine.resuming         = true;

            LOG_I("resume: bytes=%u pkts=%u img=%d",
                  snap.completed_bytes, snap.completed_packets, snap.current_img_id);
        }
        else
        {
            /* Different firmware file — discard old progress */
            progress_clear();
            LOG_I("no resume: snapshot mismatch, cleared");
        }
    }
    else
    {
        /* No valid snapshot (empty or corrupted) — clear to be safe */
        progress_clear();
        LOG_I("no resume: no valid snapshot, cleared");
    }

    engine_set_state(DFU_ENGINE_SESSION, DFU_ENGINE_ERR_NONE);
    return DFU_ENGINE_ERR_NONE;
}

/*============================================================================
 * Public API: erase
 *============================================================================*/

dfu_engine_err_t dfu_engine_erase(void)
{
    if (g_engine.state != DFU_ENGINE_SESSION)
        return DFU_ENGINE_ERR_INVALID_STATE;

    if (engine_check_abort())
        return DFU_ENGINE_ERR_ABORTED;

    engine_set_state(DFU_ENGINE_ERASING, DFU_ENGINE_ERR_NONE);

    /* If resuming and region was already erased, skip */
    if (g_engine.resuming && g_engine.bytes_received > 0)
    {
        LOG_I("resume: skip erase (already done)");
    }
    else
    {
        uint32_t addr = g_engine.cur_img->flash_addr;
        uint32_t size = g_engine.cur_img->flash_size;

        LOG_I("erasing: addr=0x%08x size=%u", addr, size);

        dfu_engine_err_t err = dfu_ef_erase(addr, size);
        if (err != DFU_ENGINE_ERR_NONE)
        {
            engine_set_state(DFU_ENGINE_ERROR, DFU_ENGINE_ERR_FLASH_ERASE);
            return DFU_ENGINE_ERR_FLASH_ERASE;
        }
    }

    /* Initialize NAND cache if target is NAND */
    dfu_ef_cache_init(g_engine.cur_img->flash_addr);

    engine_set_state(DFU_ENGINE_TRANSFERRING, DFU_ENGINE_ERR_NONE);
    return DFU_ENGINE_ERR_NONE;
}

/*============================================================================
 * Public API: write_data
 *============================================================================*/

dfu_engine_err_t dfu_engine_write_data(const uint8_t *data, uint32_t len)
{
    if (!data || len == 0)
        return DFU_ENGINE_ERR_INVALID_PARAM;

    if (g_engine.state != DFU_ENGINE_TRANSFERRING)
        return DFU_ENGINE_ERR_INVALID_STATE;

    /* 1. Check abort flag (deferred cleanup) */
    if (engine_check_abort())
        return DFU_ENGINE_ERR_ABORTED;

    /* 2. Overflow check — truncate last packet if padded beyond total */
    if (g_engine.bytes_received + len > g_engine.file_info.total_length)
    {
        uint32_t remaining = g_engine.file_info.total_length - g_engine.bytes_received;
        if (remaining > 0)
        {
            LOG_I("last packet truncated: %u -> %u bytes", len, remaining);
            len = remaining;
        }
        else
        {
            /* All data received — silently discard trailing padding/duplicates */
            LOG_D("ignoring extra packet: received=%u already equals total=%u",
                  g_engine.bytes_received, g_engine.file_info.total_length);
            return DFU_ENGINE_ERR_NONE;
        }
    }

    /* 3. Write to flash */
    dfu_engine_err_t err = dfu_ef_write(g_engine.write_addr, data, len);
    if (err != DFU_ENGINE_ERR_NONE)
    {
        LOG_E("flash write failed at 0x%08x len=%u", g_engine.write_addr, len);
        engine_set_state(DFU_ENGINE_ERROR, DFU_ENGINE_ERR_FLASH_WRITE);
        return DFU_ENGINE_ERR_FLASH_WRITE;
    }

    /* 4. Update running CRC32 */
    g_engine.running_crc = crc32_update(data, len, g_engine.running_crc);

    /* 5. Update counters */
    g_engine.write_addr += len;
    g_engine.bytes_received += len;
    g_engine.packets_received++;

    /* 6. Periodically persist progress to FlashDB */
    uint32_t since_last = g_engine.bytes_received - g_engine.last_progress_bytes;
    bool is_final = (g_engine.bytes_received >= g_engine.file_info.total_length);

    if (since_last >= g_engine.progress_interval || is_final)
    {
        progress_save();
        g_engine.last_progress_bytes = g_engine.bytes_received;

        /* 7. Fire progress callback */
        if (g_engine.on_progress)
        {
            dfu_progress_t p;
            engine_fill_progress(&p);
            g_engine.on_progress(&p, g_engine.user_data);
        }
    }

    return DFU_ENGINE_ERR_NONE;
}

/*============================================================================
 * Public API: verify
 *============================================================================*/

dfu_engine_err_t dfu_engine_verify(uint32_t expected_crc)
{
    if (g_engine.state != DFU_ENGINE_TRANSFERRING)
        return DFU_ENGINE_ERR_INVALID_STATE;

    if (engine_check_abort())
        return DFU_ENGINE_ERR_ABORTED;

    engine_set_state(DFU_ENGINE_VERIFYING, DFU_ENGINE_ERR_NONE);

    /* 1. Flush NAND cache */
    dfu_engine_err_t err = dfu_ef_cache_flush();
    if (err != DFU_ENGINE_ERR_NONE)
    {
        LOG_E("NAND cache flush failed");
        engine_set_state(DFU_ENGINE_ERROR, DFU_ENGINE_ERR_FLASH_WRITE);
        return DFU_ENGINE_ERR_FLASH_WRITE;
    }

    /* 2. Finalize and compare CRC32 */
    uint32_t final_crc = crc32_finalize(g_engine.running_crc);
    if (final_crc != expected_crc)
    {
        LOG_E("CRC mismatch: calculated=0x%08x expected=0x%08x",
              final_crc, expected_crc);
        engine_set_state(DFU_ENGINE_ERROR, DFU_ENGINE_ERR_CRC_MISMATCH);
        return DFU_ENGINE_ERR_CRC_MISMATCH;
    }
    LOG_I("CRC32 OK: 0x%08x", final_crc);
    g_engine.verified_crc = expected_crc;

    /* 3. Signature verification (if secure boot enabled) */
#ifdef BSP_USING_SECBOOT
    {
        const dfu_image_info_t *img = g_engine.cur_img;
        /* Check if signature is present (not all zeros) */
        bool has_sig = false;
        for (int i = 0; i < DFU_SIG_SIZE; i++)
        {
            if (img->sig[i] != 0)
            {
                has_sig = true;
                break;
            }
        }

        if (has_sig)
        {
            /* Use existing SDK signature verification */
            int8_t sig_ret = dfu_ctrl_ctrl_header_sig_verify_ext(
                (uint8_t *)img->flash_addr, img->img_length, (uint8_t *)img->sig);
            if (sig_ret != 0)
            {
                LOG_E("signature verification failed: %d", sig_ret);
                engine_set_state(DFU_ENGINE_ERROR, DFU_ENGINE_ERR_SIG_VERIFY);
                return DFU_ENGINE_ERR_SIG_VERIFY;
            }
            LOG_I("signature OK");
        }
    }
#endif

    /* Verification passed — stay in VERIFYING, waiting for commit */
    LOG_I("verify passed, awaiting commit");
    return DFU_ENGINE_ERR_NONE;
}

uint32_t dfu_engine_get_running_crc(void)
{
    return crc32_finalize(g_engine.running_crc);
}

/*============================================================================
 * Public API: commit
 *============================================================================*/

dfu_engine_err_t dfu_engine_commit(void)
{
    if (g_engine.state != DFU_ENGINE_VERIFYING)
        return DFU_ENGINE_ERR_INVALID_STATE;

    if (engine_check_abort())
        return DFU_ENGINE_ERR_ABORTED;

    engine_set_state(DFU_ENGINE_COMMITTING, DFU_ENGINE_ERR_NONE);

    /* Skip firmware info write for BLE direct-write mode.
     * Data is already at target flash address, no bootloader action needed. */
#if 0
    const dfu_image_info_t *img = g_engine.cur_img;
    struct dfu_fw_info fw_info;
    memset(&fw_info, 0, sizeof(fw_info));
    fw_info.file_id = img->img_id;
    fw_info.size    = img->img_length;
    fw_info.crc32   = g_engine.verified_crc;
    fw_info.addr    = img->flash_addr;
    fw_info.region_size = img->flash_size;

    int ret = dfu_core_set_firmware_info(0, &fw_info);
    if (ret != 0)
    {
        LOG_E("set firmware info failed for img %d", img->img_id);
        engine_set_state(DFU_ENGINE_ERROR, DFU_ENGINE_ERR_FLASH_WRITE);
        return DFU_ENGINE_ERR_FLASH_WRITE;
    }

    dfu_fwinfo_clear();
#endif

    /* Clear progress from FlashDB */
    progress_clear();

    /* 4. Deinit NAND cache */
    dfu_ef_cache_deinit();

    engine_set_state(DFU_ENGINE_COMPLETE, DFU_ENGINE_ERR_NONE);
    LOG_I("commit done, upgrade complete");
    return DFU_ENGINE_ERR_NONE;
}

/*============================================================================
 * Public API: abort (thread-safe — ISR-safe)
 *============================================================================*/

dfu_engine_err_t dfu_engine_abort(void)
{
    /*
     * ONLY set the flag here. Do NOT call engine_cleanup() or any
     * FlashDB / flash / RTOS API — this function may be called from
     * ISR context (e.g., button press interrupt → rt_mb_send MSG_ABORT).
     *
     * Actual cleanup is deferred to engine_check_abort(), which is
     * called at the entry of every state-mutating API (erase, write_data,
     * verify, commit) in the main-loop thread context.
     */
    g_engine.abort_flag = 1;
    LOG_I("abort flag set");
    return DFU_ENGINE_ERR_NONE;
}

/*============================================================================
 * Public API: reset_session (synchronous, main-loop only)
 *============================================================================*/

dfu_engine_err_t dfu_engine_reset_session(void)
{
    if (g_engine.state == DFU_ENGINE_IDLE)
        return DFU_ENGINE_ERR_NONE;  /* already IDLE, no-op */

    engine_cleanup(false);   /* preserve FlashDB for resume probing */
    engine_set_state(DFU_ENGINE_IDLE, DFU_ENGINE_ERR_NONE);
    LOG_I("session reset to IDLE (sync)");
    return DFU_ENGINE_ERR_NONE;
}

void dfu_engine_force_clear_progress(void)
{
    progress_clear();
    LOG_I("progress snapshot force-cleared");
}

/**
 * @brief Deferred abort cleanup (called from main-loop context only)
 *
 * Checks the abort flag and performs the actual cleanup if set.
 * Called at the entry of erase(), write_data(), verify(), commit().
 *
 * @return true if abort was processed (caller should return ERR_ABORTED)
 */
static bool engine_check_abort(void)
{
    if (!engine_is_aborted())
        return false;

    /* Save latest progress before cleanup so resume can recover
       up to the most recent write_data() call. */
    if (g_engine.state == DFU_ENGINE_TRANSFERRING)
        progress_save();

    engine_cleanup(false);   /* preserve saved progress for resume */
    engine_set_state(DFU_ENGINE_IDLE, DFU_ENGINE_ERR_ABORTED);
    return true;
}

/*============================================================================
 * Public API: get_status (thread-safe)
 *============================================================================*/

dfu_engine_err_t dfu_engine_get_status(dfu_engine_status_t *status)
{
    if (!status)
        return DFU_ENGINE_ERR_INVALID_PARAM;

    status->state          = g_engine.state;
    status->last_error     = g_engine.last_error;
    status->mode_name      = g_engine.mode_name;
    status->transport_name = g_engine.transport_name;
    engine_fill_progress(&status->progress);

    return DFU_ENGINE_ERR_NONE;
}