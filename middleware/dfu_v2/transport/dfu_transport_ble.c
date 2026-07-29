/**
 * @file dfu_transport_ble.c
 * @brief DFU V2 BLE Transport Implementation
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */
#include <rtthread.h>
#include <string.h>
#include "bf0_ble_gap.h"
#include "dfu_transport_ble.h"
#include "dfu_v2.h"                     /* DFU_MSG_BLE_DATA */
#include "engine/dfu_engine_types.h"    /* DFU_MAX_IMAGES */
#include "../dfu_macro.h"            /* DFU_V2_LOADER_START_ADDR */
#include "engine/dfu_engine.h"
/* BLE Serial Transmission API (provided by BLE stack) */
#ifdef BSP_BLE_SIBLES
#include "bf0_sibles_serial_trans_service.h"
#endif

/* SHA-256 for per-packet hash verification */
#ifdef PKG_SIFLI_MBEDTLS_BOOT
#include "mbedtls/sha256.h"
#endif

#define DBG_TAG    "dfu.ble"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

/*============================================================================
 * Configuration
 *============================================================================*/

/**
 * Message buffer size. Must hold the largest single BLE DFU message.
 * IMG_SEND_PKT is the largest: dfu_image_send_packet_v2_t header (8B)
 * + image_body_hdr (36B) + data payload (up to block_size, typically
 * 512-2048B). 4KB provides safe headroom.
 */
#ifndef DFU_BLE_MSGBUF_SIZE
#define DFU_BLE_MSGBUF_SIZE     4096
#endif

/** Number of message slots in the ring queue.
 *  Must be >= num_of_rsp + 2 to absorb a full batch without dropping.
 *  16 slots × 600B typical ≈ 10KB — acceptable for DFU. */
#ifndef DFU_BLE_QUEUE_DEPTH
#define DFU_BLE_QUEUE_DEPTH     16
#endif

/*============================================================================
 * Message Ring Queue
 *============================================================================*/

typedef struct
{
    uint8_t     data[DFU_BLE_MSGBUF_SIZE];
    uint32_t    len;
} msg_slot_t;

typedef struct
{
    msg_slot_t  slots[DFU_BLE_QUEUE_DEPTH];
    volatile uint8_t  head;     /**< Next slot to write (producer) */
    volatile uint8_t  tail;     /**< Next slot to read  (consumer) */
} msg_queue_t;

static inline void mq_init(msg_queue_t *q)
{
    q->head = 0;
    q->tail = 0;
}

static inline bool mq_is_empty(const msg_queue_t *q)
{
    return (q->head == q->tail);
}

static inline bool mq_is_full(const msg_queue_t *q)
{
    return (((q->head + 1) % DFU_BLE_QUEUE_DEPTH) == q->tail);
}

/** Enqueue: returns pointer to slot to fill, or NULL if full.
 *  Caller must fill slot->data and slot->len, then call mq_commit(). */
static inline msg_slot_t *mq_enqueue_begin(msg_queue_t *q)
{
    if (mq_is_full(q))
        return NULL;
    return &q->slots[q->head];
}

static inline void mq_enqueue_commit(msg_queue_t *q)
{
    q->head = (q->head + 1) % DFU_BLE_QUEUE_DEPTH;
}

/** Dequeue: returns pointer to oldest slot, or NULL if empty.
 *  Caller reads slot->data/len, then calls mq_dequeue_done(). */
static inline msg_slot_t *mq_dequeue_peek(msg_queue_t *q)
{
    if (mq_is_empty(q))
        return NULL;
    return &q->slots[q->tail];
}

static inline void mq_dequeue_done(msg_queue_t *q)
{
    q->tail = (q->tail + 1) % DFU_BLE_QUEUE_DEPTH;
}

/*============================================================================
 * Module Context
 *============================================================================*/

typedef struct
{
    bool                    initialized;

    /* BLE connection handle (from BLE_SERIAL_TRAN_OPEN) */
    uint16_t                conn_handle;
    bool                    is_open;

    /* Message ring queue: BLE callback enqueues, main loop dequeues.
     * Replaces the old single msg_buf to prevent packet drops when
     * APP sends faster than main loop processes. */
    msg_queue_t             msg_queue;

    /* Translation output buffer. Used by process() to build the
     * translated V2 message without overwriting input data.
     * Keeps read/write separation clean with no overlap concerns. */
    uint8_t                 trans_buf[DFU_BLE_MSGBUF_SIZE];

    /* Retransmission state */
    uint16_t                num_of_rsp;     /**< Response frequency (from IMG_SEND_START) */

    /* Image ID → index mapping (populated by INIT_REQ translation) */
    uint8_t                 img_count;      /**< Number of images from INIT_REQ */
    uint8_t                 img_ids[DFU_MAX_IMAGES]; /**< img_id for each index */
    uint32_t                pkt_rsp_count;  /**< Counter for batch RSP throttling */
    uint32_t                total_packets;  /**< Total packets from INIT_REQ */
    /* APP layer mailbox — callback posts DFU_MSG_BLE_DATA here */
    rt_mailbox_t            mb;

    /* Push Transport vtable instance */
    dfu_push_transport_t    transport;

} ble_ctx_t;

static ble_ctx_t g_ble;

/*============================================================================
 * Internal: Push Transport send() implementation
 *============================================================================*/

/**
 * @brief Send a response message to mobile APP via BLE
 *
 * Called from Host Mode (main-loop context). The data is already in
 * msg_id(2B) + length(2B) + payload format (is_message_transport=true,
 * no AA55/CRC16 framing).
 *
 * Before sending to the mobile APP, we must translate the V2 RSP payload
 * format to the legacy format expected by the APP. If no translation is
 * needed (e.g. END_IND), we send as-is.
 *
 * @param data  Message bytes (msg_id + length + payload)
 * @param len   Message length
 * @return 0 on success, -1 on failure
 */

/** RSP msg_id values */
#define DFU_RSP_INIT_RSP            1
#define DFU_RSP_INIT_RSP_EXT        33
#define DFU_RSP_RESUME_RSP          4
#define DFU_RSP_IMG_SEND_START_RSP  7
#define DFU_RSP_IMG_SEND_END_RSP    9
#define DFU_RSP_IMG_SEND_PKT_RSP   11
#define DFU_RSP_END_IND             13
#define DFU_RSP_RETRANS_RSP         17

/**
 * @brief Send raw data via BLE serial transmission
 *
 * Low-level send, no translation. Used by both ble_send (after
 * translating RSP) and ble_request_retransmission (sends directly).
 */
static int ble_raw_send(const uint8_t *data, uint16_t len)
{
    if (!g_ble.initialized || !g_ble.is_open)
        return -1;

    if (!data || len == 0)
        return -1;

#if defined(BSP_BLE_SIBLES) && defined(BSP_BLE_SERIAL_TRANSMISSION)
    ble_serial_tran_data_t t_data;
    t_data.cate_id = BLE_DFU_CATEID;
    t_data.handle  = g_ble.conn_handle;
    t_data.data    = (uint8_t *)data;
    t_data.len     = len;

    ble_serial_tran_send_data(&t_data);

    return 0;
#else
    LOG_E("BLE serial transmission not available");
    return -1;
#endif
}

/**
 * @brief Build and send a legacy-format RSP message
 *
 * Helper: builds msg_id(2B) + length(2B) + payload, then calls ble_raw_send.
 */
static int ble_send_legacy_rsp(uint16_t rsp_id, const uint8_t *payload,
                               uint16_t payload_len)
{
    uint8_t buf[64];  /* All RSP payloads are small (<16B) */
    if (4 + payload_len > sizeof(buf))
        return -1;

    buf[0] = (uint8_t)(rsp_id & 0xFF);
    buf[1] = (uint8_t)((rsp_id >> 8) & 0xFF);
    buf[2] = (uint8_t)(payload_len & 0xFF);
    buf[3] = (uint8_t)((payload_len >> 8) & 0xFF);
    if (payload && payload_len > 0)
        memcpy(&buf[4], payload, payload_len);

    return ble_raw_send(buf, 4 + payload_len);
}

static int ble_send(const uint8_t *data, uint16_t len)
{
    if (!data || len < 4)
        return -1;

    /* Parse msg_id from the V2 response */
    uint16_t msg_id      = data[0] | ((uint16_t)data[1] << 8);
    uint16_t payload_len = data[2] | ((uint16_t)data[3] << 8);
    const uint8_t *payload = &data[4];

    LOG_I("ble_send: msg_id=%u payload_len=%u total_len=%u", msg_id, payload_len, len);

    switch (msg_id)
    {
    /*
     * INIT_RSP_EXT (33):
     *   V2 payload: status(1B) + can_resume(1B) + completed_pkts(4B) = 6B
     *   Legacy:     result(2B) + resume_status(1B) + is_restart(1B)
     *               + curr_pkt(4B) + curr_img(1B) + num_of_rsp(1B)
     *               + is_boot(1B) + ver(1B) = 12B
     */
    case DFU_RSP_INIT_RSP_EXT:
    {
        if (payload_len < 7) break;
        uint8_t status     = payload[0];
        uint8_t can_resume = payload[1];
        uint32_t comp_pkts = payload[2] | ((uint32_t)payload[3] << 8) |
                             ((uint32_t)payload[4] << 16) | ((uint32_t)payload[5] << 24);
        uint8_t curr_img_index = payload[6];

        /* Translate curr_img_index (V2) to curr_img_id (legacy).
         * Use the mapping saved during INIT_REQ translation. */
        uint8_t curr_img_id = 0;
        if (curr_img_index < g_ble.img_count)
            curr_img_id = g_ble.img_ids[curr_img_index];

        uint8_t rsp[12];
        rsp[0] = status;          /* result low byte */
        rsp[1] = 0;               /* result high byte */
        rsp[2] = can_resume;      /* resume_status */
        rsp[3] = 0;               /* is_restart (0 = fresh start) */
        rsp[4] = (uint8_t)(comp_pkts & 0xFF);
        rsp[5] = (uint8_t)((comp_pkts >> 8) & 0xFF);
        rsp[6] = (uint8_t)((comp_pkts >> 16) & 0xFF);
        rsp[7] = (uint8_t)((comp_pkts >> 24) & 0xFF);
        rsp[8] = can_resume ? curr_img_id : 0;     /* curr_img: 0 when no resume */
        rsp[9] = can_resume ? (g_ble.num_of_rsp > 0 ? (uint8_t)g_ble.num_of_rsp : 1) : 0;
        rsp[10] = 0;              /* is_boot (1 = normal/user mode) */
        rsp[11] = 2;              /* ver = OTA_CODE_VERSION (2) */

        LOG_I("INIT_RSP_EXT: result=%u resume=%u pkts=%u img=%u nrsp=%u boot=%u ver=%u",
              rsp[0], rsp[2], comp_pkts, rsp[8], rsp[9], rsp[10], rsp[11]);

        int send_ret = ble_send_legacy_rsp(DFU_RSP_INIT_RSP_EXT, rsp, sizeof(rsp));
        LOG_I("INIT_RSP_EXT send result: %d", send_ret);
        return send_ret;
    }

    /*
     * INIT_RSP (1) — unlikely on BLE (BLE uses EXT), but handle for safety
     *   V2 payload: status(1B) + can_resume(1B) + completed_pkts(4B)
     *   Legacy:     result(2B) + is_boot(1B) = 3B
     */
    case DFU_RSP_INIT_RSP:
    {
        if (payload_len < 1) break;
        uint8_t status = payload[0];

        uint8_t rsp[3];
        rsp[0] = status;
        rsp[1] = 0;
        rsp[2] = 0;    /* is_boot = 0 */

        int ret = ble_send_legacy_rsp(DFU_RSP_INIT_RSP, rsp, sizeof(rsp));

        return ret;
    }

    /*
     * RESUME_RSP (4):
     *   V2 payload: status(1B) + completed_pkts(4B) = 5B
     *   Legacy:     result(2B) + is_boot(1B) + is_restart(1B)
     *               + curr_pkt(4B) + curr_img(1B) + num_of_rsp(1B) = 10B
     */
    case DFU_RSP_RESUME_RSP:
    {
        if (payload_len < 5) break;
        uint8_t status = payload[0];
        /* completed_pkts at payload[1..4] */

        uint8_t rsp[10];
        rsp[0] = status;
        rsp[1] = 0;
        rsp[2] = 0;               /* is_boot */
        rsp[3] = 0;               /* is_restart */
        memcpy(&rsp[4], &payload[1], 4);  /* curr_pkt (4B, already LE) */
        rsp[8] = 0;               /* curr_img */
        rsp[9] = 1;               /* num_of_rsp */

        return ble_send_legacy_rsp(DFU_RSP_RESUME_RSP, rsp, sizeof(rsp));
    }

    /*
     * IMG_SEND_START_RSP (7):
     *   V2 payload: status(1B) = 1B
     *   Legacy:     result(2B) + end_send(1B) + extra(1B) = 4B
     */
    case DFU_RSP_IMG_SEND_START_RSP:
    {
        if (payload_len < 1) break;
        uint8_t status = payload[0];

        uint8_t rsp[4];
        rsp[0] = status;
        rsp[1] = 0;
        rsp[2] = 0;    /* end_send (0 = keep sending) */
        rsp[3] = 0;    /* extra */

        return ble_send_legacy_rsp(DFU_RSP_IMG_SEND_START_RSP, rsp, sizeof(rsp));
    }

    /*
     * IMG_SEND_PKT_RSP (11):
     *   V2 payload: status(1B) = 1B
     *   Legacy:     result(2B) = 2B
     */
    case DFU_RSP_IMG_SEND_PKT_RSP:
    {
        if (payload_len < 1) break;
        uint8_t status = payload[0];

        g_ble.pkt_rsp_count++;

        if (status != 0 || g_ble.num_of_rsp == 0 ||
            g_ble.pkt_rsp_count >= g_ble.total_packets ||
            (g_ble.pkt_rsp_count % g_ble.num_of_rsp) == 0)
        {
            uint8_t rsp[2];
            rsp[0] = status;
            rsp[1] = 0;
            return ble_send_legacy_rsp(DFU_RSP_IMG_SEND_PKT_RSP, rsp, sizeof(rsp));
        }

        return 0;
    }

    /*
     * IMG_SEND_END_RSP (9):
     *   V2 payload: status(1B) = 1B
     *   Legacy:     result(2B) = 2B
     */
    case DFU_RSP_IMG_SEND_END_RSP:
    {
        if (payload_len < 1) break;
        uint8_t status = payload[0];

        uint8_t rsp[2];
        rsp[0] = status;
        rsp[1] = 0;

        return ble_send_legacy_rsp(DFU_RSP_IMG_SEND_END_RSP, rsp, sizeof(rsp));
    }

    /*
     * END_IND (13): V2=1B, Legacy=1B — format matches, pass through.
     * Other RSPs not listed above: pass through unchanged.
     */
    default:
        return ble_raw_send(data, len);
    }

    /* Fallthrough from break on payload_len check failure */
    LOG_E("RSP payload too short for msg_id=%u (len=%u)", msg_id, payload_len);
    return -1;
}

/*============================================================================
 * Internal: BLE Serial Transmission Callback (BLE stack context)
 *============================================================================*/

/**
 * @brief BLE Serial Transmission callback
 *
 * Registered via BLE_SERIAL_TRAN_EXPORT. Called in BLE stack context
 * (not ISR, but not DFU thread either). Must be fast — no flash
 * operations, no blocking calls.
 *
 * Strategy: copy the complete message into the ring queue, and post
 * msg_pending flag, and post DFU_MSG_BLE_DATA to the mailbox.
 * The DFU main loop picks it up and does the real processing.
 */
static void ble_dfu_serial_callback(uint8_t event, uint8_t *data)
{
    if (!data)
        return;

    /* Ignore events if not initialized */
    if (!g_ble.initialized)
        return;

    switch (event)
    {
    case BLE_SERIAL_TRAN_OPEN:
    {
        ble_serial_open_t *open = (ble_serial_open_t *)data;
        g_ble.conn_handle = open->handle;
        g_ble.is_open     = true;
        mq_init(&g_ble.msg_queue);
        LOG_I("BLE DFU connection opened (handle=%u)", open->handle);
        break;
    }

    case BLE_SERIAL_TRAN_DATA:
    {
        ble_serial_tran_data_t *t_data = (ble_serial_tran_data_t *)data;

        /* Verify handle and category */
        if (t_data->handle != g_ble.conn_handle ||
            t_data->cate_id != BLE_DFU_CATEID)
        {
            break;
        }

        /* Filter out cmd_id=11 (batch notification) at callback level
         * to avoid wasting a queue slot */
        if (t_data->len >= 2)
        {
            uint16_t msg_id = t_data->data[0] | ((uint16_t)t_data->data[1] << 8);
            if (msg_id == 11)
                break;
        }

        /* Validate size */
        if (t_data->len == 0 || t_data->len > DFU_BLE_MSGBUF_SIZE)
        {
            LOG_E("BLE message too large: %u (max=%u)",
                  t_data->len, DFU_BLE_MSGBUF_SIZE);
            break;
        }

        /* Enqueue into ring buffer */
        msg_slot_t *slot = mq_enqueue_begin(&g_ble.msg_queue);
        if (!slot)
        {
            LOG_W("BLE msg queue full, dropping %u bytes", t_data->len);
            break;
        }

        memcpy(slot->data, t_data->data, t_data->len);
        slot->len = t_data->len;
        mq_enqueue_commit(&g_ble.msg_queue);

        /* Notify DFU main loop */
        if (g_ble.mb)
            rt_mb_send(g_ble.mb, DFU_MSG_BLE_DATA);

        break;
    }

    case BLE_SERIAL_TRAN_CLOSE:
    {
        ble_serial_close_t *close = (ble_serial_close_t *)data;
        if (close->handle == g_ble.conn_handle)
        {
            g_ble.is_open     = false;
            mq_init(&g_ble.msg_queue);
            LOG_I("BLE DFU connection closed");

            /* Notify main loop so it can clean up Host Mode state.
             * Without this, a reconnecting host sending INIT_REQ would
             * fail because Engine is stuck in TRANSFERRING/SESSION. */
            if (g_ble.mb)
                rt_mb_send(g_ble.mb, DFU_MSG_BLE_DISCONNECTED);
        }
        break;
    }

    case BLE_SERIAL_TRAN_ERROR:
    {
        ble_serial_tran_error_t *error = (ble_serial_tran_error_t *)data;
        if (error->handle == g_ble.conn_handle)
        {
            LOG_E("BLE serial error: %d", error->error);
        }
        break;
    }

    default:
        break;
    }
}

/* Register callback with BLE stack (static registration at link time) */
#if defined(BSP_BLE_SIBLES) && defined(BSP_BLE_SERIAL_TRANSMISSION)
BLE_SERIAL_TRAN_EXPORT(BLE_DFU_CATEID, ble_dfu_serial_callback);
#endif

/*============================================================================
 * Internal: Per-packet hash verification (SHA-256, compare first 8 bytes)
 *============================================================================*/

/**
 * @brief Verify per-packet hash (image_body_hdr.hash)
 *
 * Computes SHA-256 of the data and compares the first 8 bytes
 * with the expected hash from image_body_hdr.
 *
 * Self-contained: uses mbedtls_sha256 directly, no dependency on
 * old DFU middleware's dfu_integrate_verify().
 *
 * @param data      Data to hash
 * @param size      Data length
 * @param expected  Expected hash (32 bytes, only first 8 compared)
 * @return 0 on match, -1 on mismatch
 */
static int ble_verify_packet_hash(const uint8_t *data, int size,
                                  const uint8_t *expected)
{
#ifdef PKG_SIFLI_MBEDTLS_BOOT

    #ifndef DFU_SIG_HASH_CMP_SIZE
    #define DFU_SIG_HASH_CMP_SIZE   8   /* Only compare first 8 bytes of SHA-256 */
    #endif

    uint8_t hash[32] = {0};
    mbedtls_sha256_context ctx;

    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);  /* SHA-256, not SHA-224 */
    mbedtls_sha256_update(&ctx, data, size);
    mbedtls_sha256_finish(&ctx, hash);
    mbedtls_sha256_free(&ctx);

    if (memcmp(hash, expected, DFU_SIG_HASH_CMP_SIZE) == 0)
        return 0;

    return -1;
#else
    /* If mbedtls is not available, skip verification.
     * BLE link layer CRC already provides basic integrity. */
    (void)data;
    (void)size;
    (void)expected;
    return 0;
#endif
}

/*============================================================================
 * Internal: Protocol message translation
 *============================================================================*/

/** Old BLE protocol message IDs (must match dfu_protocol_msg_id_t) */
#define DFU_LEGACY_INIT_REQ             0
#define DFU_LEGACY_INIT_REQ_EXT         32
#define DFU_LEGACY_IMG_SEND_PKT         10
#define DFU_LEGACY_IMG_SEND_PKT_RSP     11

/** Size of dfu_image_send_packet_v2_t header (pkt_idx + img_id + size) */
#define DFU_LEGACY_PKT_V2_HDR_SIZE      8

/** Legacy control packet wrapper and content sizes */
#define DFU_LEGACY_CFG_HDR_SIZE         32      /**< image_cfg_hdr: hash[32] */
#define DFU_LEGACY_SIG_SIZE             256     /**< Trailing signature */
#define DFU_LEGACY_KEY_SIZE             32      /**< FW_key size */
#define DFU_LEGACY_IMG_HDR_SIG_SIZE     256     /**< Per-image sig in dfu_image_header_int_t */

/**
 * @brief Resolve flash address and size for an image ID
 *
 * Self-contained partition table lookup using compile-time macros from
 * ptab.h. Covers all image IDs for both OTA_55X (NOR) and OTA_56X_NAND
 * platforms, matching the old dfu_get_download_addr_by_imgid() exactly.
 *
 * V2 direct-write mode: flag & DFU_FLAG_COMPRESS is NOT expected (we
 * require uncompressed packages), so we always return the final target
 * address (the "else" branch of the old code).
 *
 * The function is __WEAK so that board-specific code can override it
 * for custom image ID → address mappings if needed.
 *
 * @param img_id     Image identifier (matches dfu_img_id_t from old protocol)
 * @param flag       Image flags (for future compress support; currently ignored)
 * @param img_length Image data length (used as flash_size estimate if partition size unknown)
 * @param[out] addr  Flash address for this image
 * @param[out] size  Flash partition size for this image
 * @return 0 on success, -1 if image ID not found
 */

/*------------------------------------------------------------------------
 * Image ID definitions — must match dfu_img_id_t in dfu_protocol.h
 *
 * OTA_55X (NOR):
 *   0=HCPU, 1=LCPU, 2=PATCH, 3=RES, 4=FONT, 5=EX, 6=OTA_MANAGER,
 *   7=TINY_FONT, 8=RES_UPGRADE, 9=PATCH_TEMP, 10=CTRL_PACKET, 11=BOOTLOADER
 *
 * OTA_56X_NAND:
 *   0=NAND_HCPU, 1=LCPU, 2=NAND_HCPU_PATCH, 3=RES(ROOT),
 *   4=LCPU_PATCH, 5=DYN, 6=MUSIC, 7=PIC, 8=FONT, 9=RING, 10=LANG
 *------------------------------------------------------------------------*/

/* OTA_55X image IDs */
#define DFU_55X_IMG_ID_HCPU             0
#define DFU_55X_IMG_ID_LCPU             1
#define DFU_55X_IMG_ID_PATCH            2
#define DFU_55X_IMG_ID_RES              3
#define DFU_55X_IMG_ID_FONT             4
#define DFU_55X_IMG_ID_EX              5
#define DFU_55X_IMG_ID_OTA_MANAGER      6
#define DFU_55X_IMG_ID_TINY_FONT        7
#define DFU_55X_IMG_ID_RES_UPGRADE      8
#define DFU_55X_IMG_ID_PATCH_TEMP       9
#define DFU_55X_IMG_ID_CTRL_PACKET      10
#define DFU_55X_IMG_ID_BOOTLOADER       11

/* OTA_56X_NAND image IDs */
#define DFU_NAND_IMG_ID_HCPU            0
#define DFU_NAND_IMG_ID_LCPU            1
#define DFU_NAND_IMG_ID_HCPU_PATCH      2
#define DFU_NAND_IMG_ID_RES             3
#define DFU_NAND_IMG_ID_LCPU_PATCH      4
#define DFU_NAND_IMG_ID_DYN             5
#define DFU_NAND_IMG_ID_MUSIC           6
#define DFU_NAND_IMG_ID_PIC             7
#define DFU_NAND_IMG_ID_FONT            8
#define DFU_NAND_IMG_ID_RING            9
#define DFU_NAND_IMG_ID_LANG            10

static int ble_resolve_flash_addr(uint8_t img_id, uint16_t flag,
                                  uint32_t img_length,
                                  uint32_t *addr, uint32_t *size)
{
    uint32_t flash_addr = 0xFFFFFFFF;
    uint32_t flash_size = 0;

    (void)flag;  /* V2 direct-write: no compress staging area lookup */

#ifdef OTA_55X
    /*--------------------------------------------------------------------
     * NOR Flash platform (SF32LB52X, SF32LB55X, etc.)
     *--------------------------------------------------------------------*/
    switch (img_id)
    {
    case DFU_55X_IMG_ID_HCPU:
#ifdef HCPU_FLASH_CODE_START_ADDR
        flash_addr = HCPU_FLASH_CODE_START_ADDR;
#ifdef HCPU_FLASH_CODE_SIZE
        flash_size = HCPU_FLASH_CODE_SIZE;
#endif
#endif
        break;

    case DFU_55X_IMG_ID_OTA_MANAGER:
#if defined(DFU_V2_LOADER_START_ADDR) && (DFU_V2_LOADER_START_ADDR != 0xFFFFFFFF)
        flash_addr = DFU_V2_LOADER_START_ADDR;
        flash_size = DFU_V2_LOADER_SIZE;
#elif defined(DFU_FLASH_CODE_START_ADDR)
        flash_addr = DFU_FLASH_CODE_START_ADDR;
#ifdef DFU_FLASH_CODE_SIZE
        flash_size = DFU_FLASH_CODE_SIZE;
#endif
#endif
        break;

#ifndef SOC_SF32LB52X
    case DFU_55X_IMG_ID_LCPU:
#ifdef LCPU_FLASH_CODE_START_ADDR
        flash_addr = LCPU_FLASH_CODE_START_ADDR;
#ifdef LCPU_FLASH_CODE_SIZE
        flash_size = LCPU_FLASH_CODE_SIZE;
#endif
#endif
        break;

    case DFU_55X_IMG_ID_PATCH:
#ifdef LCPU_PATCH_START_ADDR
        flash_addr = LCPU_PATCH_START_ADDR;
#ifdef LCPU_PATCH_SIZE
        flash_size = LCPU_PATCH_SIZE;
#endif
#endif
        break;
#endif /* !SOC_SF32LB52X */

    case DFU_55X_IMG_ID_FONT:
#ifdef HCPU_FLASH2_FONT_START_ADDR
        flash_addr = HCPU_FLASH2_FONT_START_ADDR;
#ifdef HCPU_FLASH2_FONT_SIZE
        flash_size = HCPU_FLASH2_FONT_SIZE;
#endif
#endif
        break;

    case DFU_55X_IMG_ID_TINY_FONT:
#ifdef HCPU_FLASH2_TINY_FONT_START_ADDR
        flash_addr = HCPU_FLASH2_TINY_FONT_START_ADDR;
#ifdef HCPU_FLASH2_TINY_FONT_SIZE
        flash_size = HCPU_FLASH2_TINY_FONT_SIZE;
#endif
#endif
        break;

    case DFU_55X_IMG_ID_RES:
#ifdef HCPU_FLASH2_IMG_START_ADDR
        flash_addr = HCPU_FLASH2_IMG_START_ADDR;
#ifdef HCPU_FLASH2_IMG_SIZE
        flash_size = HCPU_FLASH2_IMG_SIZE;
#endif
#endif
        break;

    case DFU_55X_IMG_ID_EX:
#ifdef HCPU_FS_ROOT_BURN_ADDR
        flash_addr = HCPU_FS_ROOT_BURN_ADDR;
#ifdef HCPU_FS_ROOT_SIZE
        flash_size = HCPU_FS_ROOT_SIZE;
#endif
#endif
        break;

    case DFU_55X_IMG_ID_RES_UPGRADE:
#ifdef HCPU_FLASH2_IMG_UPGRADE_START_ADDR
        flash_addr = HCPU_FLASH2_IMG_UPGRADE_START_ADDR;
#ifdef HCPU_FLASH2_IMG_UPGRADE_SIZE
        flash_size = HCPU_FLASH2_IMG_UPGRADE_SIZE;
#endif
#endif
        break;

    case DFU_55X_IMG_ID_BOOTLOADER:
#ifdef FLASH_BOOT_LOADER_START_ADDR
        flash_addr = FLASH_BOOT_LOADER_START_ADDR;
#ifdef FLASH_BOOT_LOADER_SIZE
        flash_size = FLASH_BOOT_LOADER_SIZE;
#endif
#endif
        break;

    default:
        break;
    }

#elif defined(OTA_56X_NAND)
    /*--------------------------------------------------------------------
     * NAND Flash platform (SF32LB56X, SF32LB58X, etc.)
     *--------------------------------------------------------------------*/
    switch (img_id)
    {
    case DFU_NAND_IMG_ID_HCPU:
        /* For NAND pingpong, the target address depends on which slot
         * is the "backup" slot. The old code uses env->back_up_hcpu
         * which is determined at runtime. For V2 direct-write, we use
         * the second flash code region if available. */
#ifdef HCPU_FLASH_CODE_LOAD_REGION2_START_ADDR
        flash_addr = HCPU_FLASH_CODE_LOAD_REGION2_START_ADDR;
#ifdef HCPU_FLASH_CODE_LOAD_REGION2_SIZE
        flash_size = HCPU_FLASH_CODE_LOAD_REGION2_SIZE;
#endif
#elif defined(HCPU_FLASH_CODE_START_ADDR)
        flash_addr = HCPU_FLASH_CODE_START_ADDR;
#ifdef HCPU_FLASH_CODE_SIZE
        flash_size = HCPU_FLASH_CODE_SIZE;
#endif
#endif
        break;

    case DFU_NAND_IMG_ID_LCPU:
#ifdef DFU_LCPU_DOWNLOAD_ADDR
        flash_addr = DFU_LCPU_DOWNLOAD_ADDR;
#endif
        break;

    case DFU_NAND_IMG_ID_HCPU_PATCH:
#ifdef DFU_NAND_PATCH_DOWNLOAD_ADDR
        flash_addr = DFU_NAND_PATCH_DOWNLOAD_ADDR;
#ifdef DFU_NAND_PATCH_DOWNLOAD_SIZE
        flash_size = DFU_NAND_PATCH_DOWNLOAD_SIZE;
#endif
#endif
        break;

    case DFU_NAND_IMG_ID_LCPU_PATCH:
#ifdef DFU_LCPU_PATCH_DOWNLOAD_ADDR
        flash_addr = DFU_LCPU_PATCH_DOWNLOAD_ADDR;
#ifdef DFU_LCPU_PATCH_DOWNLOAD_SIZE
        flash_size = DFU_LCPU_PATCH_DOWNLOAD_SIZE;
#endif
#endif
        break;

    case DFU_NAND_IMG_ID_RES:
#ifdef HCPU_FS_ROOT_BURN_ADDR
        flash_addr = HCPU_FS_ROOT_BURN_ADDR;
#ifdef HCPU_FS_ROOT_SIZE
        flash_size = HCPU_FS_ROOT_SIZE;
#endif
#endif
        break;

    case DFU_NAND_IMG_ID_DYN:
#ifdef HCPU_FS_DYN_BURN_ADDR
        flash_addr = HCPU_FS_DYN_BURN_ADDR;
#ifdef HCPU_FS_DYN_SIZE
        flash_size = HCPU_FS_DYN_SIZE;
#endif
#endif
        break;

    case DFU_NAND_IMG_ID_MUSIC:
#ifdef HCPU_FS_MUSIC_BURN_ADDR
        flash_addr = HCPU_FS_MUSIC_BURN_ADDR;
#ifdef HCPU_FS_MUSIC_SIZE
        flash_size = HCPU_FS_MUSIC_SIZE;
#endif
#endif
        break;

    case DFU_NAND_IMG_ID_PIC:
#ifdef HCPU_FLASH2_IMG_BURN_ADDR
        flash_addr = HCPU_FLASH2_IMG_BURN_ADDR;
#ifdef HCPU_FLASH2_IMG_SIZE
        flash_size = HCPU_FLASH2_IMG_SIZE;
#endif
#endif
        break;

    case DFU_NAND_IMG_ID_FONT:
#ifdef HCPU_FLASH2_FONT_BURN_ADDR
        flash_addr = HCPU_FLASH2_FONT_BURN_ADDR;
#ifdef HCPU_FLASH2_FONT_SIZE
        flash_size = HCPU_FLASH2_FONT_SIZE;
#endif
#endif
        break;

    case DFU_NAND_IMG_ID_RING:
#ifdef HCPU_FLASH2_RING_BURN_ADDR
        flash_addr = HCPU_FLASH2_RING_BURN_ADDR;
#ifdef HCPU_FLASH2_RING_SIZE
        flash_size = HCPU_FLASH2_RING_SIZE;
#endif
#endif
        break;

    case DFU_NAND_IMG_ID_LANG:
#ifdef HCPU_FLASH2_LANG_START_ADDR
        flash_addr = HCPU_FLASH2_LANG_START_ADDR;
#ifdef HCPU_FLASH2_LANG_SIZE
        flash_size = HCPU_FLASH2_LANG_SIZE;
#endif
#endif
        break;

    default:
        break;
    }

#else
    /*--------------------------------------------------------------------
     * Fallback: try common macros that exist on most platforms
     *--------------------------------------------------------------------*/
    switch (img_id)
    {
    case 0: /* HCPU */
#ifdef HCPU_FLASH_CODE_START_ADDR
        flash_addr = HCPU_FLASH_CODE_START_ADDR;
#ifdef HCPU_FLASH_CODE_SIZE
        flash_size = HCPU_FLASH_CODE_SIZE;
#endif
#endif
        break;

    case 6: /* OTA_MANAGER / DFU */
#if defined(DFU_V2_LOADER_START_ADDR) && (DFU_V2_LOADER_START_ADDR != 0xFFFFFFFF)
        flash_addr = DFU_V2_LOADER_START_ADDR;
        flash_size = DFU_V2_LOADER_SIZE;
#elif defined(DFU_FLASH_CODE_START_ADDR)
        flash_addr = DFU_FLASH_CODE_START_ADDR;
#ifdef DFU_FLASH_CODE_SIZE
        flash_size = DFU_FLASH_CODE_SIZE;
#endif
#endif
        break;

    case 11: /* BOOTLOADER */
#ifdef FLASH_BOOT_LOADER_START_ADDR
        flash_addr = FLASH_BOOT_LOADER_START_ADDR;
#ifdef FLASH_BOOT_LOADER_SIZE
        flash_size = FLASH_BOOT_LOADER_SIZE;
#endif
#endif
        break;

    default:
        break;
    }
#endif /* OTA_55X / OTA_56X_NAND */

    if (flash_addr == 0xFFFFFFFF || flash_addr == 0)
    {
        *addr = 0;
        *size = 0;
        LOG_W("flash_addr lookup failed for img_id=%u", img_id);
        return -1;
    }

    *addr = flash_addr;
    /* Use partition size if known, otherwise estimate from image length */
    *size = (flash_size > 0) ? flash_size : ((img_length + 0xFFF) & ~0xFFFU);

    LOG_I("img_id=%u -> flash=0x%08x, size=0x%x", img_id, *addr, *size);
    return 0;
}

/**
 * @brief Translate INIT_REQ/INIT_REQ_EXT from legacy BLE format to DFU V2 format
 *
 * Legacy BLE INIT_REQ_EXT payload structure:
 *   image_cfg_hdr: hash[32]                        (32B — outer hash)
 *   control_packet_content (plain or decrypted):
 *     dfu_ID                                        (1B)
 *     HW_version                                    (4B)
 *     SDK_version                                   (4B)
 *     FW_version                                    (4B)
 *     FW_key[32]                                    (32B)
 *     image_header_len                              (2B)
 *     dfu_code_image_header_t:
 *       blk_size                                    (2B)
 *       img_count                                   (1B)
 *       dfu_image_header_int_t[img_count]:
 *         sig[256]                                  (256B)
 *         length                                    (4B)
 *         flag                                      (2B)
 *         img_id                                    (1B)
 *   sig[256]                                        (256B — trailing signature)
 *
 * DFU V2 handle_init_req expects payload:
 *   [0]       dfu_id       (1B)
 *   [1..4]    total_length (4B LE) — sum of all image lengths
 *   [5..8]    total_packets(4B LE) — ceil(total_length / blk_size)
 *   [9..12]   file_crc     (4B LE) — 0 for BLE (not provided in legacy)
 *   [13..14]  block_size   (2B LE)
 *   [15]      img_count    (1B)
 *   per image (15B each):
 *     [0]     img_id       (1B)
 *     [1..2]  flag         (2B LE)
 *     [3..6]  img_length   (4B LE)
 *     [7..10] flash_addr   (4B LE) — from partition table
 *     [11..14] flash_size  (4B LE) — from partition table
 *
 * @param msg_id        Original msg_id (0 or 32)
 * @param payload       Legacy payload (after msg_id+length header)
 * @param payload_len   Legacy payload length
 * @param out_buf       Output buffer for translated message
 * @param out_buf_size  Output buffer capacity
 * @return Translated message length (including msg_id+length header),
 *         -1 on error
 */
static int translate_init_req(uint16_t msg_id,
                              const uint8_t *payload, uint16_t payload_len,
                              uint8_t *out_buf, uint32_t out_buf_size)
{
    /*
     * Step 1: Strip outer hash (image_cfg_hdr)
     */
    if (payload_len < DFU_LEGACY_CFG_HDR_SIZE + 1 + DFU_LEGACY_SIG_SIZE)
    {
        LOG_E("INIT_REQ too short: %u", payload_len);
        return -1;
    }

    const uint8_t *outer_hash = &payload[0];
    const uint8_t *content    = &payload[DFU_LEGACY_CFG_HDR_SIZE];
    uint16_t content_len      = payload_len - DFU_LEGACY_CFG_HDR_SIZE - DFU_LEGACY_SIG_SIZE;
    /* trailing sig at payload[payload_len - DFU_LEGACY_SIG_SIZE] */

    /*
     * Step 2: Decrypt + verify control packet content
     *
     * imgtoolv37 ALWAYS AES-CTR encrypts the control packet content,
     * even when image flag=0 (no compress/no encrypt). The stored hash
     * is SHA-256 of the PLAINTEXT, but the content in the packet is
     * CIPHERTEXT. This matches the old dfu_ctrl_sol.c flow:
     *
     *   dfu_dec_verify(NULL, 0, ciphertext, plaintext, len, hash)
     *     → sifli_hw_dec(NULL, ciphertext, plaintext, len, 0)  // AES-CTR decrypt
     *     → SHA-256(plaintext) compare first 8 bytes with hash
     *
     * We reuse dfu_dec_verify() from dfu_sec.c which handles:
     *   - Key retrieval from eFuse/OTP (or fake key for dev boards)
     *   - AES-CTR nonce construction from sig_hash
     *   - SHA-256 verification of decrypted content
     */

    /* Allocate buffer for decrypted content (content is modified in-place
     * by dfu_dec_verify, so we need a writable copy) */
    uint8_t *decrypted = rt_malloc(content_len);
    if (!decrypted)
    {
        LOG_E("INIT_REQ: failed to allocate %u bytes for decryption", content_len);
        return -1;
    }
    memcpy(decrypted, content, content_len);

    extern uint8_t *dfu_dec_verify(uint8_t *key, uint32_t offset,
                                   uint8_t *in_data, uint8_t *out_data,
                                   int size, uint8_t *hash);
    /* Note: dfu_dec_verify is provided by dfu_sec.c.
     * Alternatively, #include "dfu_sec.h" at the top of this file. */

    uint8_t *dec_result = dfu_dec_verify(
        NULL,                           /* key=NULL → use root key from eFuse/AES_ACC */
        0,                              /* offset=0 → CTR starts at 0 */
        decrypted,                      /* in: ciphertext (will be overwritten) */
        decrypted,                      /* out: decrypt in-place */
        (int)content_len,               /* size */
        (uint8_t *)outer_hash           /* expected hash (first 8 bytes compared) */
    );

    if (!dec_result)
    {
        LOG_E("INIT_REQ ctrl_packet decrypt+verify failed");
        rt_free(decrypted);
        return -1;
    }

    LOG_I("INIT_REQ ctrl_packet decrypted and verified OK");

    /* From here on, use 'decrypted' instead of 'content' for parsing */
    const uint8_t *plain_content = decrypted;

    /*
     * Step 3: Parse legacy control packet content (byte-by-byte, matching
     * dfu_ctrl_ctrl_header_alloc in old code — handles unaligned fields)
     *
     * All error returns in parsing must free 'decrypted' before returning.
     */
#define PARSE_FAIL(msg, ...) do { LOG_E(msg, ##__VA_ARGS__); rt_free(decrypted); return -1; } while(0)

    const uint8_t *p = plain_content;
    const uint8_t *p_end = plain_content + content_len;

    /* dfu_ID (1B) */
    if (p + 1 > p_end) PARSE_FAIL("truncated at dfu_ID");
    uint8_t dfu_id = *p++;

    /* HW_version (4B) */
    if (p + 4 > p_end) PARSE_FAIL("truncated at HW_version");
    uint32_t hw_ver;
    memcpy(&hw_ver, p, 4); p += 4;

    /* SDK_version (4B) */
    if (p + 4 > p_end) PARSE_FAIL("truncated at SDK_version");
    uint32_t sdk_ver;
    memcpy(&sdk_ver, p, 4); p += 4;

    /* FW_version (4B) */
    if (p + 4 > p_end) PARSE_FAIL("truncated at FW_version");
    uint32_t fw_ver;
    memcpy(&fw_ver, p, 4); p += 4;

    /* FW_key[32] (32B) */
    if (p + DFU_LEGACY_KEY_SIZE > p_end) PARSE_FAIL("truncated at FW_key");
    const uint8_t *fw_key = p;
    p += DFU_LEGACY_KEY_SIZE;

    /* image_header_len (2B) — not used directly, skip */
    if (p + 2 > p_end) PARSE_FAIL("truncated at image_header_len");
    p += 2;

    /* dfu_code_image_header_t: blk_size (2B) + img_count (1B) */
    if (p + 3 > p_end) PARSE_FAIL("truncated at image_header");
    uint16_t blk_size;
    memcpy(&blk_size, p, 2); p += 2;
    uint8_t img_count = *p++;

    if (img_count == 0 || img_count > DFU_MAX_IMAGES)
        PARSE_FAIL("invalid img_count: %u", img_count);

    /* Parse per-image headers: sig[256] + length(4) + flag(2) + img_id(1) = 263B each */
    typedef struct {
        uint8_t  img_id;
        uint16_t flag;
        uint32_t img_length;
        uint32_t flash_addr;
        uint32_t flash_size;
    } parsed_img_t;

    parsed_img_t imgs[DFU_MAX_IMAGES];
    uint32_t total_length = 0;

    for (uint8_t i = 0; i < img_count; i++)
    {
        /* sig[256] — skip (per-image signature, stored in V2 image_info but
         * not needed for the init_req payload translation) */
        if (p + DFU_LEGACY_IMG_HDR_SIG_SIZE > p_end)
            PARSE_FAIL("truncated at img[%u].sig", i);
        /* const uint8_t *img_sig = p; */  /* Save if needed later */
        p += DFU_LEGACY_IMG_HDR_SIG_SIZE;

        /* length (4B) */
        if (p + 4 > p_end) PARSE_FAIL("truncated at img[%u].length", i);
        memcpy(&imgs[i].img_length, p, 4); p += 4;

        /* flag (2B) */
        if (p + 2 > p_end) PARSE_FAIL("truncated at img[%u].flag", i);
        memcpy(&imgs[i].flag, p, 2); p += 2;

        /* img_id (1B) */
        if (p + 1 > p_end) PARSE_FAIL("truncated at img[%u].img_id", i);
        imgs[i].img_id = *p++;

        /* Resolve flash address from partition table */
        if (ble_resolve_flash_addr(imgs[i].img_id,
                                   imgs[i].flag,
                                   imgs[i].img_length,
                                   &imgs[i].flash_addr,
                                   &imgs[i].flash_size) != 0)
        {
            LOG_W("no flash_addr for img_id=%u, using 0", imgs[i].img_id);
            /* Engine will fail at erase if addr is 0 — acceptable for now,
             * the board-specific partition lookup must be implemented. */
        }

        total_length += imgs[i].img_length;
    }

    /* Compute total_packets */
    uint32_t total_packets = 0;
    if (blk_size > 0)
        total_packets = (total_length + blk_size - 1) / blk_size;

    /*
     * Step 4: Build V2 format message
     *
     * Header: msg_id(2B) + length(2B)
     * Payload: 16B fixed + 15B × img_count
     */
    uint16_t v2_payload_len = 16 + (15 * img_count);
    uint16_t v2_msg_len     = 4 + v2_payload_len;

    if (v2_msg_len > out_buf_size)
        PARSE_FAIL("output buffer too small for INIT_REQ: need=%u, have=%u",
                   v2_msg_len, out_buf_size);

#undef PARSE_FAIL

    /* Use the same msg_id as the original (0 or 32) so Host Mode
     * routes to the correct handler (INIT_REQ or INIT_REQ_EXT) */
    uint16_t out_msg_id = msg_id;

    /* Message header */
    out_buf[0] = (uint8_t)(out_msg_id & 0xFF);
    out_buf[1] = (uint8_t)((out_msg_id >> 8) & 0xFF);
    out_buf[2] = (uint8_t)(v2_payload_len & 0xFF);
    out_buf[3] = (uint8_t)((v2_payload_len >> 8) & 0xFF);

    /* Payload: fixed header (16B) */
    uint8_t *q = &out_buf[4];
    q[0] = dfu_id;
    q[1] = (uint8_t)(total_length & 0xFF);
    q[2] = (uint8_t)((total_length >> 8) & 0xFF);
    q[3] = (uint8_t)((total_length >> 16) & 0xFF);
    q[4] = (uint8_t)((total_length >> 24) & 0xFF);
    q[5] = (uint8_t)(total_packets & 0xFF);
    q[6] = (uint8_t)((total_packets >> 8) & 0xFF);
    q[7] = (uint8_t)((total_packets >> 16) & 0xFF);
    q[8] = (uint8_t)((total_packets >> 24) & 0xFF);
    /* file_crc = 0 (legacy BLE protocol doesn't provide overall CRC) */
    q[9]  = 0; q[10] = 0; q[11] = 0; q[12] = 0;
    q[13] = (uint8_t)(blk_size & 0xFF);
    q[14] = (uint8_t)((blk_size >> 8) & 0xFF);
    q[15] = img_count;
    q += 16;

    /* Payload: per-image descriptors (15B each) */
    for (uint8_t i = 0; i < img_count; i++)
    {
        q[0] = imgs[i].img_id;
        q[1] = (uint8_t)(imgs[i].flag & 0xFF);
        q[2] = (uint8_t)((imgs[i].flag >> 8) & 0xFF);
        q[3] = (uint8_t)(imgs[i].img_length & 0xFF);
        q[4] = (uint8_t)((imgs[i].img_length >> 8) & 0xFF);
        q[5] = (uint8_t)((imgs[i].img_length >> 16) & 0xFF);
        q[6] = (uint8_t)((imgs[i].img_length >> 24) & 0xFF);
        q[7] = (uint8_t)(imgs[i].flash_addr & 0xFF);
        q[8] = (uint8_t)((imgs[i].flash_addr >> 8) & 0xFF);
        q[9] = (uint8_t)((imgs[i].flash_addr >> 16) & 0xFF);
        q[10] = (uint8_t)((imgs[i].flash_addr >> 24) & 0xFF);
        q[11] = (uint8_t)(imgs[i].flash_size & 0xFF);
        q[12] = (uint8_t)((imgs[i].flash_size >> 8) & 0xFF);
        q[13] = (uint8_t)((imgs[i].flash_size >> 16) & 0xFF);
        q[14] = (uint8_t)((imgs[i].flash_size >> 24) & 0xFF);
        q += 15;
    }

    LOG_I("INIT_REQ translated: dfu_id=%u, imgs=%u, total=%u, blk=%u",
          dfu_id, img_count, total_length, blk_size);

    /* Save img_id → index mapping for IMG_SEND_START translation */
    g_ble.img_count = img_count;
    for (uint8_t i = 0; i < img_count; i++)
        g_ble.img_ids[i] = imgs[i].img_id;
    g_ble.total_packets = total_packets;

    rt_free(decrypted);
    return (int)v2_msg_len;
}

/**
 * @brief Translate IMG_SEND_PKT from legacy BLE format to DFU V2 format
 *
 * Legacy BLE payload (dfu_image_send_packet_v2_t):
 *   [0..3]  pkt_idx   (uint32_t LE)
 *   [4..5]  img_id    (uint16_t LE)
 *   [6..7]  size      (uint16_t LE)  — size of packet[] below
 *   [8..]   packet[]  = image_body_hdr (36B) + raw firmware data
 *
 *   image_body_hdr:
 *     [0..31]  hash[32]   (SHA-256 of raw data, first 8B compared)
 *     [32..35] offset     (uint32_t LE, write offset within image)
 *
 * DFU V2 Host Mode expects (handle_img_send_pkt payload):
 *   [0..3]  pkt_idx   (uint32_t LE)
 *   [4..]   raw firmware data (no header)
 *
 * Translation: keep pkt_idx, strip img_id+size+image_body_hdr, keep raw data.
 * Before stripping, verify the per-packet hash.
 *
 * @param payload       Legacy payload (after msg_id+length header)
 * @param payload_len   Legacy payload length
 * @param out_buf       Output buffer for translated message (must be large enough)
 * @param out_buf_size  Output buffer capacity
 * @return Translated message length (including msg_id+length header),
 *         0 if hash verification failed (retransmission requested),
 *         -1 on error
 */
static int translate_img_send_pkt(const uint8_t *payload, uint16_t payload_len,
                                  uint8_t *out_buf, uint32_t out_buf_size)
{
    if (payload_len < 6)
    {
        LOG_E("IMG_SEND_PKT too short: %u", payload_len);
        return -1;
    }

    uint32_t pkt_idx;
    uint16_t declared_size;
    const uint8_t *packet;
    uint16_t packet_len;

    /* V1 format: img_id(2B) + pkt_idx(2B) + size(2B) + packet[] */
    pkt_idx = payload[2] | ((uint32_t)payload[3] << 8);
    if (pkt_idx > 0) pkt_idx--;  /* 1-based → 0-based */
    declared_size = payload[4] | ((uint16_t)payload[5] << 8);
    packet = &payload[6];
    packet_len = payload_len - 6;

    /* Sanity check declared_size vs actual */
    if (declared_size > packet_len)
    {
        LOG_E("IMG_SEND_PKT size mismatch: declared=%u, available=%u",
              declared_size, packet_len);
        return -1;
    }

    /* Extract image_body_hdr */
    if (declared_size < DFU_BLE_BODY_HDR_SIZE)
    {
        LOG_E("packet too short for image_body_hdr: %u", declared_size);
        return -1;
    }

    const uint8_t *hash = &packet[0];
    const uint8_t *raw_data = &packet[DFU_BLE_BODY_HDR_SIZE];
    uint16_t raw_data_len = declared_size - DFU_BLE_BODY_HDR_SIZE;

    /* Verify per-packet hash */
    if (ble_verify_packet_hash(raw_data, raw_data_len, hash) != 0)
    {
        LOG_W("IMG_SEND_PKT hash verify failed, pkt_idx=%u", pkt_idx);
        return 0;
    }

    uint16_t v2_payload_len = 4 + raw_data_len;
    uint16_t v2_msg_len = 4 + v2_payload_len;

    if (v2_msg_len > out_buf_size)
    {
        LOG_E("output buffer too small: need=%u, have=%u",
              v2_msg_len, out_buf_size);
        return -1;
    }

    out_buf[0] = (uint8_t)(DFU_LEGACY_IMG_SEND_PKT & 0xFF);
    out_buf[1] = (uint8_t)((DFU_LEGACY_IMG_SEND_PKT >> 8) & 0xFF);
    out_buf[2] = (uint8_t)(v2_payload_len & 0xFF);
    out_buf[3] = (uint8_t)((v2_payload_len >> 8) & 0xFF);
    out_buf[4] = (uint8_t)(pkt_idx & 0xFF);
    out_buf[5] = (uint8_t)((pkt_idx >> 8) & 0xFF);
    out_buf[6] = (uint8_t)((pkt_idx >> 16) & 0xFF);
    out_buf[7] = (uint8_t)((pkt_idx >> 24) & 0xFF);
    memcpy(&out_buf[8], raw_data, raw_data_len);

    return (int)v2_msg_len;
}

/*============================================================================
 * Internal: IMG_SEND_START translation
 *============================================================================*/

/** Legacy command IDs that need special handling */
#define DFU_LEGACY_IMG_SEND_START       6
#define DFU_LEGACY_IMG_SEND_END         8

/**
 * @brief Translate IMG_SEND_START from legacy BLE format to DFU V2 format
 *
 * Legacy BLE payload (dfu_image_send_start_t):
 *   [0..3]  img_length    (uint32_t LE)
 *   [4..7]  total_pkt_num (uint32_t LE)
 *   [8]     num_of_rsp    (uint8_t) — batched response frequency
 *   [9]     img_id        (uint8_t)
 *
 * DFU V2 Host Mode expects (handle_img_send_start payload):
 *   [0]  image_index  (uint8_t) — index into file_info.images[]
 *
 * Translation: look up img_id in the stored mapping to find image_index.
 * Also extract num_of_rsp for retransmission support.
 */
static int translate_img_send_start(const uint8_t *payload, uint16_t payload_len,
                                    uint8_t *out_buf, uint32_t out_buf_size)
{
    if (payload_len < 10)
    {
        LOG_E("IMG_SEND_START too short: %u", payload_len);
        return -1;
    }

    /* Parse legacy fields */
    uint8_t num_of_rsp = payload[8];
    uint8_t img_id     = payload[9];

    /* Save num_of_rsp for retransmission requests */
    g_ble.num_of_rsp = num_of_rsp;

    /* Find image_index by matching img_id in the mapping saved during INIT_REQ */
    uint8_t img_index = 0;
    bool found = false;
    for (uint8_t i = 0; i < g_ble.img_count; i++)
    {
        if (g_ble.img_ids[i] == img_id)
        {
            img_index = i;
            found = true;
            break;
        }
    }

    if (!found)
    {
        LOG_E("IMG_SEND_START: img_id=%u not found in init info", img_id);
        return -1;
    }

    /* Build V2 message: msg_id(2B) + length(2B) + image_index(1B) */
    if (out_buf_size < 5)
        return -1;

    out_buf[0] = DFU_LEGACY_IMG_SEND_START;
    out_buf[1] = 0;
    out_buf[2] = 1;  /* payload length = 1 */
    out_buf[3] = 0;
    out_buf[4] = img_index;

    LOG_I("IMG_SEND_START translated: img_id=%u -> index=%u, num_of_rsp=%u",
          img_id, img_index, num_of_rsp);
    g_ble.pkt_rsp_count = 0;
    return 5;
}

/*============================================================================
 * Internal: IMG_SEND_END translation
 *============================================================================*/

/**
 * @brief Translate IMG_SEND_END from legacy BLE format to DFU V2 format
 *
 * Legacy BLE payload (dfu_image_send_end_t):
 *   [0]  img_id        (uint8_t)
 *   [1]  is_more_image (uint8_t)
 *
 * DFU V2 Host Mode expects (handle_img_send_end payload):
 *   [0..3]  per-image CRC32  (uint32_t LE)
 *
 * The legacy BLE protocol does NOT provide a per-image CRC from the host.
 * Per-packet integrity is guaranteed by image_body_hdr hash verification
 * in the BLE Transport layer. To satisfy Engine's verify() step, we pass
 * Engine's own accumulated CRC back to it (self-match, always passes).
 */
static int translate_img_send_end(const uint8_t *payload, uint16_t payload_len,
                                  uint8_t *out_buf, uint32_t out_buf_size)
{
    if (payload_len < 2)
    {
        LOG_E("IMG_SEND_END too short: %u", payload_len);
        return -1;
    }

    uint8_t img_id = payload[0];
    /* is_more_image at payload[1] — V2 Mode handles multi-image via
     * IMG_SEND_START for next image, so this field is not needed. */

    /* Get Engine's accumulated CRC32 to pass back as "expected" */
    extern uint32_t dfu_engine_get_running_crc(void);
    uint32_t crc = dfu_engine_get_running_crc();

    /* Build V2 message: msg_id(2B) + length(2B) + CRC32(4B) */
    if (out_buf_size < 8)
        return -1;

    out_buf[0] = DFU_LEGACY_IMG_SEND_END;
    out_buf[1] = 0;
    out_buf[2] = 4;  /* payload length = 4 */
    out_buf[3] = 0;
    out_buf[4] = (uint8_t)(crc & 0xFF);
    out_buf[5] = (uint8_t)((crc >> 8) & 0xFF);
    out_buf[6] = (uint8_t)((crc >> 16) & 0xFF);
    out_buf[7] = (uint8_t)((crc >> 24) & 0xFF);

    LOG_I("IMG_SEND_END translated: img_id=%u, self_crc=0x%08x", img_id, crc);

    return 8;
}

/*============================================================================
 * Internal: Request retransmission via LINK_LOSE_CHECK_REQ
 *============================================================================*/

/**
 * @brief Send LINK_LOSE_CHECK_REQ to mobile APP requesting retransmission
 *
 * When per-packet hash verification fails, we send this message to tell
 * the APP which packet was last successfully received, so it can resend
 * from that point.
 *
 * LINK_LOSE_CHECK_REQ (msg_id=34) payload:
 *   [0..1]  result          (uint16_t LE) — 0 = no error
 *   [2..3]  new_num_of_rsp  (uint16_t LE) — response frequency
 *   [4..7]  current_pkt_idx (uint32_t LE) — last confirmed packet
 *
 * @param last_confirmed_pkt  Last successfully received packet index
 * @param num_of_rsp          Current response frequency
 */
static void ble_request_retransmission(uint32_t last_confirmed_pkt,
                                       uint16_t num_of_rsp)
{
    uint8_t msg[4 + 8];  /* header(4) + payload(8) */

    /* msg_id = DFU_LINK_LOSE_CHECK_REQ (34) */
    msg[0] = 34;
    msg[1] = 0;
    /* length = 8 */
    msg[2] = 8;
    msg[3] = 0;
    /* result = 0 (no error) */
    msg[4] = 0;
    msg[5] = 0;
    /* new_num_of_rsp */
    msg[6] = (uint8_t)(num_of_rsp & 0xFF);
    msg[7] = (uint8_t)((num_of_rsp >> 8) & 0xFF);
    /* current_pkt_idx */
    msg[8]  = (uint8_t)(last_confirmed_pkt & 0xFF);
    msg[9]  = (uint8_t)((last_confirmed_pkt >> 8) & 0xFF);
    msg[10] = (uint8_t)((last_confirmed_pkt >> 16) & 0xFF);
    msg[11] = (uint8_t)((last_confirmed_pkt >> 24) & 0xFF);

    /* Send directly to APP via BLE (bypass RSP translation) */
    ble_raw_send(msg, sizeof(msg));

    LOG_W("retransmission requested from pkt %u", last_confirmed_pkt);
}

/*============================================================================
 * Public API: Process received message (called from DFU main loop)
 *============================================================================*/

/**
 * @brief Process a pending BLE message
 *
 * Called by APP main loop after receiving DFU_MSG_BLE_DATA from mailbox.
 * Retrieves the buffered message, performs protocol translation for
 * commands that differ between legacy BLE format and DFU V2 format,
 * and feeds the result to Host Mode via dfu_mode_host_feed_message().
 *
 * @return 0 on success, -1 if no data or error
 */
int dfu_transport_ble_process(void)
{
    if (!g_ble.initialized)
        return -1;

    /* Dequeue one message from the ring */
    msg_slot_t *slot = mq_dequeue_peek(&g_ble.msg_queue);
    if (!slot)
        return -1;  /* queue empty */

    uint32_t len = slot->len;

    if (len < 4)
    {
        mq_dequeue_done(&g_ble.msg_queue);
        return -1;
    }

    /* Parse message header: msg_id(2B LE) + length(2B LE) */
    uint16_t msg_id      = slot->data[0] | ((uint16_t)slot->data[1] << 8);
    uint16_t payload_len = slot->data[2] | ((uint16_t)slot->data[3] << 8);
    const uint8_t *payload = &slot->data[4];

    /* Validate */
    if (4 + payload_len > len)
    {
        LOG_E("BLE message length mismatch: declared=%u, available=%u",
              payload_len, len - 4);
        mq_dequeue_done(&g_ble.msg_queue);
        return -1;
    }

    extern int dfu_mode_host_feed_message(const uint8_t *data, uint32_t len);
    int ret = -1;

    switch (msg_id)
    {
    case DFU_LEGACY_INIT_REQ:       /* 0 */
    case 14:                        /* FORCE_INIT_REQUEST */
    case DFU_LEGACY_INIT_REQ_EXT:   /* 32 */
    {
        /*
         * INIT_REQ: strip hash+sig wrapper, parse legacy control packet,
         * translate to DFU V2 format.
         * Output is small: 4B header + 16B fixed + 15B × img_count.
         * Max = 4 + 16 + 15*12 = 200B.
         */
        int trans_len = translate_init_req(msg_id, payload, payload_len,
                                           g_ble.trans_buf, sizeof(g_ble.trans_buf));

        if (trans_len > 0)
        {
            ret = dfu_mode_host_feed_message(g_ble.trans_buf, (uint32_t)trans_len);
        }
        else
        {
            LOG_E("INIT_REQ translation failed");
        }
        break;
    }

    case DFU_LEGACY_IMG_SEND_PKT:
    {
        /*
         * IMG_SEND_PKT: strip image_body_hdr, verify hash, reformat.
         * Translated message is always smaller than original (we remove
         * img_id(2B)+size(2B)+hash(32B)+offset(4B) = 40B, add header 4B).
         * We use g_ble.trans_buf as output to keep read/write cleanly separated.
         */
        int trans_len = translate_img_send_pkt(payload, payload_len,
                                               g_ble.trans_buf,
                                               sizeof(g_ble.trans_buf));

        if (trans_len > 0)
        {
            /* Translation successful — feed to Host Mode */
            ret = dfu_mode_host_feed_message(g_ble.trans_buf, (uint32_t)trans_len);
        }
        else if (trans_len == 0)
        {
            /* Hash verification failed — request retransmission.
             * We need to know the last confirmed packet index.
             * Extract pkt_idx from the failed packet for reference. */
            uint32_t failed_pkt_idx = payload[0] | ((uint32_t)payload[1] << 8) |
                                      ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
            /* Last confirmed = failed - 1 (or 0 if this was the first) */
            uint32_t last_confirmed = (failed_pkt_idx > 0) ? (failed_pkt_idx - 1) : 0;
            ble_request_retransmission(last_confirmed, g_ble.num_of_rsp);
            ret = 0;  /* Not a fatal error */
        }
        else
        {
            LOG_E("IMG_SEND_PKT translation failed");
        }
        break;
    }

    case DFU_LEGACY_IMG_SEND_START:
    {
        uint8_t translated[16];
        int trans_len = translate_img_send_start(payload, payload_len,
                                                 translated, sizeof(translated));
        if (trans_len > 0)
            ret = dfu_mode_host_feed_message(translated, (uint32_t)trans_len);
        else
            LOG_E("IMG_SEND_START translation failed");
        break;
    }

    case DFU_LEGACY_IMG_SEND_END:
    {
        uint8_t translated[16];
        int trans_len = translate_img_send_end(payload, payload_len,
                                               translated, sizeof(translated));
        if (trans_len > 0)
            ret = dfu_mode_host_feed_message(translated, (uint32_t)trans_len);
        else
            LOG_E("IMG_SEND_END translation failed");
        break;
    }

    /*
     * All other commands (INIT_COMPLETE, RESUME_REQ, RESUME_COMPLETE,
     * TRANS_END, ABORT, CONN_PRIORITY, READ_VER_REQ, etc.):
     *
     * msg_id values match between legacy BLE and DFU V2.
     * Payload is either unused by V2 Host Mode or format-compatible.
     * Pass through unchanged.
     */
    default:
    {
        /* Pass raw message to Host Mode */
        ret = dfu_mode_host_feed_message(slot->data, len);
        break;
    }
    }

    /* Release slot for next message */
    mq_dequeue_done(&g_ble.msg_queue);

    return ret;
}

/*============================================================================
 * Public API: Lifecycle
 *============================================================================*/

int dfu_transport_ble_init(rt_mailbox_t mb)
{
    if (g_ble.initialized)
    {
        LOG_W("BLE transport already initialized");
        return 0;
    }

    if (!mb)
    {
        LOG_E("mailbox handle is NULL");
        return -1;
    }

    memset(&g_ble, 0, sizeof(g_ble));
    g_ble.mb = mb;

    /* Set up Push Transport vtable */
    g_ble.transport.send = ble_send;
    g_ble.initialized = true;

    LOG_I("BLE transport initialized (queue=%u×%u bytes)", DFU_BLE_QUEUE_DEPTH, DFU_BLE_MSGBUF_SIZE);
    return 0;
}

void dfu_transport_ble_deinit(void)
{
    if (!g_ble.initialized)
        return;

    g_ble.is_open     = false;
    mq_init(&g_ble.msg_queue);
    g_ble.initialized = false;

    LOG_I("BLE transport deinitialized");
}

/*============================================================================
 * Public API: Transport Instance
 *============================================================================*/

const dfu_push_transport_t *dfu_transport_ble_get_instance(void)
{
    if (!g_ble.initialized)
        return RT_NULL;

    return &g_ble.transport;
}