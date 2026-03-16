/**
 * @file dfu_mode_host.c
 * @brief DFU V2 Host-Initiated Mode Implementation
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <string.h>
#include <board.h>
#include "dfu_mode_host.h"
#include "dfu_protocol.h"
#include "engine/dfu_engine.h"

#define LOG_TAG "dfu.host"
#include <log.h>

/*============================================================================
 * Internal Context
 *============================================================================*/

typedef struct
{
    /* Lifecycle */
    bool                        initialized;
    bool                        session_active;

    /* Protocol parser (for byte-stream entry) */
    dfu_proto_parser_t         *parser;

    /* Push Transport (for sending response frames to host) */
    const dfu_push_transport_t *transport;
    bool                        is_message_transport; /**< true for BLE (no AA55/CRC framing) */
    const char                 *transport_name;       /**< "cdc", "ble", etc. for logging */

    /* APP layer callback */
    dfu_mode_callback_t         callback;
    void                       *user_data;

    /* Current session state */
    dfu_file_info_t             file_info;
    dfu_resume_info_t           resume_info;
    bool                        is_resume;
    uint8_t                     current_img_index;
    uint32_t                    expected_pkt_index;  /**< Next expected packet index */

    /* Image skip: when the current image cannot be written (e.g. DFU
     * subprogram cannot overwrite itself, user APP cannot overwrite
     * its own HCPU), the image is "skipped" — ACKs are sent to the
     * host normally so it keeps transmitting, but data is discarded
     * instead of written to flash. */
    bool                        skip_current_image;

    /**
     * Self image ID — the img_id of the partition this program is
     * running from. Set by the caller via dfu_mode_host_set_self_img_id().
     * When IMG_SEND_START receives an image with this ID, it enters
     * skip mode. Set to 0xFF to disable (default = no filtering).
     */
    uint8_t                     self_img_id;

    /* Response buffer (avoid per-frame malloc for common responses) */
    uint8_t                     rsp_buf[DFU_PROTO_OVERHEAD + 64];
} host_mode_ctx_t;

static host_mode_ctx_t g_host;

/*============================================================================
 * Internal Helpers — Response Sending
 *============================================================================*/

/**
 * @brief Send a response frame through Push Transport
 */
static int send_response(dfu_cmd_id_t rsp_id,
                         const uint8_t *payload,
                         uint16_t payload_len)
{
    if (!g_host.transport || !g_host.transport->send)
        return -1;

    /*
     * Message-type transports (BLE): send msg_id + length + payload directly.
     * No AA55 header, no CRC16 — BLE handles framing/integrity at link layer.
     * This mirrors the entry path: feed_message() skips AA55 parsing,
     * so the response path should skip AA55 building.
     */
    if (g_host.is_message_transport)
    {
        /* Build: msg_id(2B LE) + length(2B LE) + payload(NB) */
        uint16_t msg_len = 4 + payload_len;

        if (msg_len <= sizeof(g_host.rsp_buf))
        {
            g_host.rsp_buf[0] = (uint8_t)(rsp_id & 0xFF);
            g_host.rsp_buf[1] = (uint8_t)((rsp_id >> 8) & 0xFF);
            g_host.rsp_buf[2] = (uint8_t)(payload_len & 0xFF);
            g_host.rsp_buf[3] = (uint8_t)((payload_len >> 8) & 0xFF);
            if (payload && payload_len > 0)
                memcpy(&g_host.rsp_buf[4], payload, payload_len);
            return g_host.transport->send(g_host.rsp_buf, msg_len);
        }

        /* Large message: heap allocate */
        uint8_t *buf = (uint8_t *)rt_malloc(msg_len);
        if (!buf)
            return -1;
        buf[0] = (uint8_t)(rsp_id & 0xFF);
        buf[1] = (uint8_t)((rsp_id >> 8) & 0xFF);
        buf[2] = (uint8_t)(payload_len & 0xFF);
        buf[3] = (uint8_t)((payload_len >> 8) & 0xFF);
        if (payload && payload_len > 0)
            memcpy(&buf[4], payload, payload_len);
        int ret = g_host.transport->send(buf, msg_len);
        rt_free(buf);
        return ret;
    }

    /*
     * Byte-stream transports (CDC/UART): build full AA55 frame with CRC16.
     */

    /* Try stack buffer for small responses */
    if (payload_len + DFU_PROTO_OVERHEAD <= sizeof(g_host.rsp_buf))
    {
        int frame_len = dfu_proto_frame_build(rsp_id, payload, payload_len,
                                              g_host.rsp_buf, sizeof(g_host.rsp_buf));
        if (frame_len < 0)
            return frame_len;
        return g_host.transport->send(g_host.rsp_buf, (uint16_t)frame_len);
    }

    /* Large response: heap allocate */
    uint32_t out_len = 0;
    uint8_t *frame = dfu_proto_frame_alloc_build(rsp_id, payload, payload_len, &out_len);
    if (!frame)
        return -1;
    int ret = g_host.transport->send(frame, (uint16_t)out_len);
    rt_free(frame);
    return ret;
}

/**
 * @brief Send a simple status response (1-byte error code payload)
 */
static int send_status_response(dfu_cmd_id_t rsp_id, dfu_proto_err_t err)
{
    uint8_t payload[1] = { (uint8_t)err };
    return send_response(rsp_id, payload, 1);
}

/**
 * @brief Map Engine error to Protocol wire error
 */
static dfu_proto_err_t engine_err_to_proto(dfu_engine_err_t err)
{
    switch (err)
    {
    case DFU_ENGINE_ERR_NONE:           return DFU_PROTO_ERR_NONE;
    case DFU_ENGINE_ERR_INVALID_STATE:  return DFU_PROTO_ERR_UNEXPECT_STATE;
    case DFU_ENGINE_ERR_INVALID_PARAM:  return DFU_PROTO_ERR_PARAM_INVALID;
    case DFU_ENGINE_ERR_FLASH_ERASE:    return DFU_PROTO_ERR_FLASH;
    case DFU_ENGINE_ERR_FLASH_WRITE:    return DFU_PROTO_ERR_FLASH;
    case DFU_ENGINE_ERR_FLASH_READ:     return DFU_PROTO_ERR_FLASH;
    case DFU_ENGINE_ERR_CRC_MISMATCH:   return DFU_PROTO_ERR_FW_INVALID;
    case DFU_ENGINE_ERR_SIG_VERIFY:     return DFU_PROTO_ERR_FW_INVALID;
    case DFU_ENGINE_ERR_OVERFLOW:       return DFU_PROTO_ERR_PARAM_INVALID;
    default:                            return DFU_PROTO_ERR_GENERAL;
    }
}

/*============================================================================
 * Internal Helpers — APP Callback
 *============================================================================*/

static void notify_app(dfu_mode_event_t event, dfu_engine_err_t error)
{
    if (!g_host.callback)
        return;

    dfu_mode_event_param_t param;
    memset(&param, 0, sizeof(param));
    param.event = event;
    param.error = error;

    /* Fill progress from engine */
    dfu_engine_status_t status;
    if (dfu_engine_get_status(&status) == DFU_ENGINE_ERR_NONE)
        param.progress = status.progress;

    param.img_id = (g_host.current_img_index < g_host.file_info.img_count)
                   ? g_host.file_info.images[g_host.current_img_index].img_id
                   : g_host.current_img_index;
    g_host.callback(&param, g_host.user_data);
}

/*============================================================================
 * Command Handlers
 *============================================================================*/

/**
 * @brief Handle INIT_REQ / INIT_REQ_EXT — Parse control packet, probe resume
 *
 * The host sends a control packet containing image descriptors.
 * Mode parses the relevant fields into dfu_file_info_t and stores them.
 * Then probes resume by calling begin_session() for the first image —
 * this checks FlashDB for a matching progress snapshot.
 *
 * Actual per-image Engine sessions are started by IMG_SEND_START.
 * For img_idx==0, IMG_SEND_START detects the existing session and
 * skips the redundant begin_session(), only calling erase().
 *
 * Simplified control packet payload (for new protocol):
 *   [0]     dfu_id        (1B)
 *   [1..4]  total_length  (4B LE)
 *   [5..8]  total_packets (4B LE)
 *   [9..12] file_crc      (4B LE)
 *   [13..14] block_size   (2B LE)
 *   [15]    img_count     (1B)
 *   [16..]  image descriptors (variable)
 *
 * Each image descriptor:
 *   [0]     img_id        (1B)
 *   [1..2]  flag          (2B LE)
 *   [3..6]  img_length    (4B LE)
 *   [7..10] flash_addr    (4B LE)
 *   [11..14] flash_size   (4B LE)
 *
 * Note: For compatibility with existing BLE protocol, the actual control
 * packet format may differ (includes hash + signature wrapper). The BLE
 * Transport layer handles wrapper stripping before data reaches here.
 */
static void handle_init_req(dfu_cmd_id_t cmd_id,
                            const uint8_t *payload, uint16_t length)
{
    dfu_cmd_id_t rsp_id = (cmd_id == DFU_CMD_INIT_REQ_EXT)
                          ? DFU_CMD_INIT_RSP_EXT : DFU_CMD_INIT_RSP;

    if (length < 16)
    {
        LOG_E("init_req payload too short: %u", length);
        send_status_response(rsp_id, DFU_PROTO_ERR_PARAM_INVALID);
        return;
    }

    /*
     * Legacy hash+sig wrapper stripping:
     * In the old BLE DFU, INIT_REQ_EXT payload arrives wrapped as:
     *   Hash(32B) | Control packet content | Signature(256B)
     *
     * This unwrapping is handled in dfu_transport_ble.c's
     * ble_translate_legacy_to_v2() — the BLE Transport strips the
     * hash+sig wrapper and passes only the control packet content
     * as the V2 payload. So by the time data reaches Host Mode,
     * it's already clean and can be parsed directly.
     *
     * For CDC Transport, the PC tool sends raw V2 format (no wrapper).
     */

    /* Parse file info from payload */
    memset(&g_host.file_info, 0, sizeof(g_host.file_info));

    /* FORCE_INIT_REQ: discard any stale resume snapshot */
    if (cmd_id == DFU_CMD_FORCE_INIT_REQ)
    {
        dfu_engine_force_clear_progress();
    }

    const uint8_t *p = payload;
    g_host.file_info.dfu_id         = p[0];
    g_host.file_info.total_length   = p[1]  | (p[2] << 8) | (p[3] << 16) | (p[4] << 24);
    g_host.file_info.total_packets  = p[5]  | (p[6] << 8) | (p[7] << 16) | (p[8] << 24);
    g_host.file_info.file_crc       = p[9]  | (p[10] << 8) | (p[11] << 16) | (p[12] << 24);
    g_host.file_info.block_size     = p[13] | (p[14] << 8);
    g_host.file_info.img_count      = p[15];

    if (g_host.file_info.img_count == 0 ||
        g_host.file_info.img_count > DFU_MAX_IMAGES)
    {
        LOG_E("invalid img_count: %u", g_host.file_info.img_count);
        send_status_response(rsp_id, DFU_PROTO_ERR_PARAM_INVALID);
        return;
    }

    /* Parse image descriptors */
    uint16_t offset = 16;
    for (uint8_t i = 0; i < g_host.file_info.img_count; i++)
    {
        if (offset + 15 > length)
        {
            LOG_E("payload too short for image[%u]", i);
            send_status_response(rsp_id, DFU_PROTO_ERR_PARAM_INVALID);
            return;
        }

        const uint8_t *img = &payload[offset];
        g_host.file_info.images[i].img_id      = img[0];
        g_host.file_info.images[i].flag         = img[1] | (img[2] << 8);
        g_host.file_info.images[i].img_length   = img[3] | (img[4] << 8) |
                                                   (img[5] << 16) | (img[6] << 24);
        g_host.file_info.images[i].flash_addr   = img[7] | (img[8] << 8) |
                                                   (img[9] << 16) | (img[10] << 24);
        g_host.file_info.images[i].flash_size   = img[11] | (img[12] << 8) |
                                                   (img[13] << 16) | (img[14] << 24);
        offset += 15;
    }

    /* Probe resume: try each image in order until we find a matching
     * FlashDB snapshot, or fall through to img[0] as default.
     *
     * If the device was interrupted on img[N], the snapshot stores
     * img[N]'s total_length/total_packets. We must probe with img[N]'s
     * info to get a match. After probing, the session stays open on
     * the matched image (or img[0] if no match) for IMG_SEND_START
     * to reuse. */
    g_host.current_img_index = 0;
    g_host.is_resume = false;
    g_host.expected_pkt_index = 0;

    /* static to avoid ~3KB stack allocation (dfu_file_info_t is large) */
    static dfu_file_info_t probe_info;
    bool resume_found = false;

    for (uint8_t i = 0; i < g_host.file_info.img_count; i++)
    {
        memset(&probe_info, 0, sizeof(probe_info));
        const dfu_image_info_t *img = &g_host.file_info.images[i];

        probe_info.total_length  = img->img_length;
        probe_info.total_packets = (g_host.file_info.block_size > 0)
            ? (img->img_length + g_host.file_info.block_size - 1) / g_host.file_info.block_size
            : 0;
        probe_info.file_crc      = g_host.file_info.file_crc;
        probe_info.dfu_id        = g_host.file_info.dfu_id;
        probe_info.block_size    = g_host.file_info.block_size;
        probe_info.img_count     = 1;
        memcpy(&probe_info.images[0], img, sizeof(dfu_image_info_t));

        dfu_engine_err_t err = dfu_engine_begin_session(
            &probe_info, "host-initiated", g_host.transport_name, &g_host.resume_info);

        if (err != DFU_ENGINE_ERR_NONE)
        {
            LOG_E("begin_session probe failed for img[%u]: %d", i, err);
            send_status_response(rsp_id, engine_err_to_proto(err));
            return;
        }

        if (g_host.resume_info.can_resume)
        {
            g_host.current_img_index = i;
            resume_found = true;
            LOG_I("resume found at img[%u], completed=%u",
                  i, g_host.resume_info.completed_packets);
            break;
        }

        /* No resume for this image. Reset session synchronously
           so we can probe the next image immediately. */
        if (i < g_host.file_info.img_count - 1)
        {
            dfu_engine_reset_session();
        }
    }

    if (!resume_found)
    {
        /* No resume match on any image. Session is open on the last
           probed image. For a fresh start, we want img[0]. If the
           last probe was not img[0], reset and re-probe img[0]. */
        if (g_host.file_info.img_count > 1)
        {
            dfu_engine_reset_session();

            memset(&probe_info, 0, sizeof(probe_info));
            const dfu_image_info_t *img0 = &g_host.file_info.images[0];
            probe_info.total_length  = img0->img_length;
            probe_info.total_packets = (g_host.file_info.block_size > 0)
                ? (img0->img_length + g_host.file_info.block_size - 1) / g_host.file_info.block_size
                : 0;
            probe_info.file_crc      = g_host.file_info.file_crc;
            probe_info.dfu_id        = g_host.file_info.dfu_id;
            probe_info.block_size    = g_host.file_info.block_size;
            probe_info.img_count     = 1;
            memcpy(&probe_info.images[0], img0, sizeof(dfu_image_info_t));

            dfu_engine_err_t err = dfu_engine_begin_session(
                &probe_info, "host-initiated", g_host.transport_name, &g_host.resume_info);
            if (err != DFU_ENGINE_ERR_NONE)
            {
                LOG_E("begin_session for img[0] failed: %d", err);
                send_status_response(rsp_id, engine_err_to_proto(err));
                return;
            }
        }
        g_host.current_img_index = 0;
        LOG_I("no resume, fresh start from img[0]");
    }

    g_host.session_active = true;
    notify_app(DFU_MODE_EVT_STARTED, DFU_ENGINE_ERR_NONE);

    /* Build response: status(1B) + can_resume(1B) + completed_packets(4B) + current_img_index(1B) */
    uint8_t rsp_payload[7];
    rsp_payload[0] = (uint8_t)DFU_PROTO_ERR_NONE;
    rsp_payload[1] = g_host.resume_info.can_resume ? 1 : 0;
    rsp_payload[2] = (uint8_t)(g_host.resume_info.completed_packets & 0xFF);
    rsp_payload[3] = (uint8_t)((g_host.resume_info.completed_packets >> 8) & 0xFF);
    rsp_payload[4] = (uint8_t)((g_host.resume_info.completed_packets >> 16) & 0xFF);
    rsp_payload[5] = (uint8_t)((g_host.resume_info.completed_packets >> 24) & 0xFF);
    rsp_payload[6] = g_host.resume_info.can_resume ? g_host.resume_info.current_img_index : 0;

    send_response(rsp_id, rsp_payload, sizeof(rsp_payload));
    LOG_I("init_req OK, resume=%d, completed=%u",
          g_host.resume_info.can_resume, g_host.resume_info.completed_packets);
}

/**
 * @brief Handle RESUME_REQ — Resume interrupted session
 */
static void handle_resume_req(const uint8_t *payload, uint16_t length)
{
    /* Resume uses the same file_info from previous INIT_REQ.
       Host sends RESUME_REQ to confirm it wants to resume. */

    if (!g_host.session_active || !g_host.resume_info.can_resume)
    {
        send_status_response(DFU_CMD_RESUME_RSP, DFU_PROTO_ERR_UNEXPECT_STATE);
        return;
    }

    g_host.is_resume = true;

    /* Reply with resume position */
    uint8_t rsp[5];
    rsp[0] = (uint8_t)DFU_PROTO_ERR_NONE;
    rsp[1] = (uint8_t)(g_host.resume_info.completed_packets & 0xFF);
    rsp[2] = (uint8_t)((g_host.resume_info.completed_packets >> 8) & 0xFF);
    rsp[3] = (uint8_t)((g_host.resume_info.completed_packets >> 16) & 0xFF);
    rsp[4] = (uint8_t)((g_host.resume_info.completed_packets >> 24) & 0xFF);

    send_response(DFU_CMD_RESUME_RSP, rsp, sizeof(rsp));
    LOG_I("resume_req OK, skip to packet %u", g_host.resume_info.completed_packets);
}

/**
 * @brief Handle INIT_COMPLETE_IND / INIT_COMPLETE_EXT / RESUME_COMPLETE_IND
 *        — Host confirms init, device is now ready for IMG_SEND_START
 *
 * In the refactored flow, erase is done in IMG_SEND_START (per-image),
 * so INIT_COMPLETE is just a state confirmation / acknowledgment.
 */
static void handle_init_complete(dfu_cmd_id_t cmd_id)
{
    if (!g_host.session_active)
    {
        LOG_W("init_complete but no session");
        return;
    }

    LOG_I("init_complete received (cmd=%u), ready for IMG_SEND_START", cmd_id);
}

/**
 * @brief Handle IMG_SEND_START — Host begins sending image data
 *
 * Multi-image flow: The host sends IMG_SEND_START for each image.
 * Mode orchestrates per-image Engine sessions:
 *
 *   Probed image (matches INIT_REQ probe result):
 *     - INIT_REQ already called begin_session() for this image.
 *       Engine is in SESSION state. Just call erase() here.
 *
 *   Other images:
 *     - Commit the previous image (Engine: VERIFYING → COMPLETE).
 *     - begin_session() for the new image (Engine: COMPLETE → SESSION).
 *     - erase() for the new image.
 *
 * Payload structure:
 *   [0]  image_index  (1B, index into file_info.images[])
 */
static void handle_img_send_start(const uint8_t *payload, uint16_t length)
{
    if (!g_host.session_active)
    {
        send_status_response(DFU_CMD_IMG_SEND_START_RSP, DFU_PROTO_ERR_UNEXPECT_STATE);
        return;
    }

    /* Parse image index from payload */
    uint8_t img_idx = 0;
    if (length >= 1)
        img_idx = payload[0];

    if (img_idx >= g_host.file_info.img_count)
    {
        LOG_E("img_send_start: invalid img_index=%u (count=%u)",
              img_idx, g_host.file_info.img_count);
        send_status_response(DFU_CMD_IMG_SEND_START_RSP, DFU_PROTO_ERR_INDEX_ERROR);
        return;
    }

    const dfu_image_info_t *img = &g_host.file_info.images[img_idx];

    /*
     * Self-image skip: if this image matches self_img_id, the running
     * program cannot overwrite its own flash. Enter skip mode — ACK
     * everything the host sends for this image, but discard the data.
     * No Engine session is created for skipped images.
     */
    g_host.skip_current_image = false;
    if (g_host.self_img_id != 0xFF && img->img_id == g_host.self_img_id)
    {
        LOG_W("img_send_start: SKIP img_id=%u (self) — cannot overwrite running partition",
              img->img_id);
        g_host.skip_current_image = true;
        g_host.current_img_index = img_idx;
        g_host.expected_pkt_index = 0;
        send_status_response(DFU_CMD_IMG_SEND_START_RSP, DFU_PROTO_ERR_NONE);
        return;
    }
    dfu_engine_err_t err;

    /*
     * Determine if we need a new Engine session.
     * INIT_REQ probe left a session open for g_host.current_img_index.
     * If this IMG_SEND_START matches that index and Engine is in SESSION,
     * reuse the existing session. Otherwise, start a new one.
     */
    dfu_engine_status_t st;
    dfu_engine_get_status(&st);

    bool need_new_session = true;

    if (img_idx == g_host.current_img_index && st.state == DFU_ENGINE_SESSION)
    {
        /* Session already established by INIT_REQ probe for this image */
        need_new_session = false;
        LOG_D("img[%u]: reusing session from INIT_REQ probe", img_idx);
    }

    if (need_new_session)
    {
        /* Commit previous image if in a committable state */
        if (st.state == DFU_ENGINE_VERIFYING)
        {
            err = dfu_engine_commit();
            if (err != DFU_ENGINE_ERR_NONE)
            {
                LOG_E("commit previous image failed: %d", err);
                send_status_response(DFU_CMD_IMG_SEND_START_RSP,
                                     engine_err_to_proto(err));
                notify_app(DFU_MODE_EVT_ERROR, err);
                return;
            }
            /* IMG_COMPLETE was already notified by handle_img_send_end
               after verify passed. Just log the commit here. */
            LOG_I("committed previous image before switching to img[%u]", img_idx);
        }

        /* Build per-image file_info */
        static dfu_file_info_t per_img_info;
        memset(&per_img_info, 0, sizeof(per_img_info));
        per_img_info.total_length  = img->img_length;
        per_img_info.total_packets = (g_host.file_info.block_size > 0)
            ? (img->img_length + g_host.file_info.block_size - 1) / g_host.file_info.block_size
            : 0;
        per_img_info.file_crc      = g_host.file_info.file_crc;  /* for resume matching only */
        per_img_info.dfu_id        = g_host.file_info.dfu_id;
        per_img_info.block_size    = g_host.file_info.block_size;
        per_img_info.img_count     = 1;
        memcpy(&per_img_info.images[0], img, sizeof(dfu_image_info_t));

        /* Begin new Engine session (Engine accepts from IDLE or COMPLETE) */
        dfu_resume_info_t resume;
        err = dfu_engine_begin_session(
            &per_img_info, "host-initiated", g_host.transport_name, &resume);
        if (err != DFU_ENGINE_ERR_NONE)
        {
            LOG_E("begin_session for img[%u] failed: %d", img_idx, err);
            send_status_response(DFU_CMD_IMG_SEND_START_RSP, engine_err_to_proto(err));
            notify_app(DFU_MODE_EVT_ERROR, err);
            return;
        }
    }

    g_host.current_img_index = img_idx;

    /* Erase flash for this image */
    err = dfu_engine_erase();
    if (err != DFU_ENGINE_ERR_NONE)
    {
        LOG_E("erase for img[%u] failed: %d", img_idx, err);
        send_status_response(DFU_CMD_IMG_SEND_START_RSP, engine_err_to_proto(err));
        notify_app(DFU_MODE_EVT_ERROR, err);
        return;
    }

    /* Initialize expected packet counter.
     * For resume scenario, start from where we left off — but ONLY for
     * the specific image that was interrupted (resume_info.current_img_index).
     * Other images in the same session start from packet 0. */
    if (g_host.is_resume && img_idx == g_host.resume_info.current_img_index)
        g_host.expected_pkt_index = g_host.resume_info.completed_packets;
    else
        g_host.expected_pkt_index = 0;

    send_status_response(DFU_CMD_IMG_SEND_START_RSP, DFU_PROTO_ERR_NONE);
    LOG_I("img_send_start OK, img_index=%u, addr=0x%08x, len=%u, pkt_start=%u",
          img_idx, img->flash_addr, img->img_length, g_host.expected_pkt_index);
}

/**
 * @brief Handle IMG_SEND_PKT — Write one data packet to flash
 *
 * Payload structure:
 *   [0..3]  packet_index  (4B LE)
 *   [4..]   data          (variable)
 */
static void handle_img_send_pkt(const uint8_t *payload, uint16_t length)
{
    if (!g_host.session_active)
    {
        send_status_response(DFU_CMD_IMG_SEND_PKT_RSP, DFU_PROTO_ERR_UNEXPECT_STATE);
        return;
    }

    if (length <= 4)
    {
        send_status_response(DFU_CMD_IMG_SEND_PKT_RSP, DFU_PROTO_ERR_PARAM_INVALID);
        return;
    }

    /* Parse and check packet index */
    uint32_t packet_index = payload[0] | ((uint32_t)payload[1] << 8) |
                            ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);

    if (packet_index != g_host.expected_pkt_index)
    {
        LOG_E("packet index mismatch: expected=%u, received=%u",
              g_host.expected_pkt_index, packet_index);
        send_status_response(DFU_CMD_IMG_SEND_PKT_RSP, DFU_PROTO_ERR_MISS_PACKET);
        return;
    }

    /*
     * Skip mode: ACK the packet but don't write to flash.
     * The host sees a normal success response and keeps sending.
     */
    if (g_host.skip_current_image)
    {
        g_host.expected_pkt_index++;
        send_status_response(DFU_CMD_IMG_SEND_PKT_RSP, DFU_PROTO_ERR_NONE);
        return;
    }

    const uint8_t *data = &payload[4];
    uint32_t data_len = length - 4;

    dfu_engine_err_t err = dfu_engine_write_data(data, data_len);
    if (err != DFU_ENGINE_ERR_NONE)
    {
        LOG_E("write_data failed: %d", err);
        send_status_response(DFU_CMD_IMG_SEND_PKT_RSP, engine_err_to_proto(err));

        if (err == DFU_ENGINE_ERR_ABORTED)
            notify_app(DFU_MODE_EVT_ABORTED, err);
        else
            notify_app(DFU_MODE_EVT_ERROR, err);
        return;
    }

    /* Success — advance packet counter and send ACK */
    g_host.expected_pkt_index++;
    send_status_response(DFU_CMD_IMG_SEND_PKT_RSP, DFU_PROTO_ERR_NONE);
}

/**
 * @brief Handle IMG_SEND_END — All packets for current image sent, verify
 *
 * Payload structure:
 *   [0..3]  per-image CRC32  (4B LE, computed by host over this image's data)
 *
 * The CRC is passed to dfu_engine_verify() for comparison with the
 * device's running CRC32 accumulated during write_data() calls.
 */
static void handle_img_send_end(const uint8_t *payload, uint16_t length)
{
    if (!g_host.session_active)
    {
        send_status_response(DFU_CMD_IMG_SEND_END_RSP, DFU_PROTO_ERR_UNEXPECT_STATE);
        return;
    }

    /*
     * Skip mode: no data was written, so no verify/commit needed.
     * Just ACK and clear the skip flag for the next image.
     */
    if (g_host.skip_current_image)
    {
        LOG_I("img_send_end: skipped img_index=%u (self-image)", g_host.current_img_index);
        g_host.skip_current_image = false;
        send_status_response(DFU_CMD_IMG_SEND_END_RSP, DFU_PROTO_ERR_NONE);
        return;
    }

    if (length < 4)
    {
        LOG_E("img_send_end: payload too short for CRC (%u bytes)", length);
        send_status_response(DFU_CMD_IMG_SEND_END_RSP, DFU_PROTO_ERR_PARAM_INVALID);
        return;
    }

    /* Parse per-image expected CRC32 from host */
    uint32_t expected_crc = payload[0] | ((uint32_t)payload[1] << 8) |
                            ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);

    dfu_engine_err_t err = dfu_engine_verify(expected_crc);
    if (err != DFU_ENGINE_ERR_NONE)
    {
        LOG_E("verify failed: %d", err);
        send_status_response(DFU_CMD_IMG_SEND_END_RSP, engine_err_to_proto(err));
        notify_app(DFU_MODE_EVT_ERROR, err);
        return;
    }

    send_status_response(DFU_CMD_IMG_SEND_END_RSP, DFU_PROTO_ERR_NONE);
    notify_app(DFU_MODE_EVT_IMG_COMPLETE, DFU_ENGINE_ERR_NONE);
    LOG_I("img_send_end: verify passed, img_index=%u, crc=0x%08x",
          g_host.current_img_index, expected_crc);
}

/**
 * @brief Handle TRANS_END — All images done, commit and finalize
 */
static void handle_trans_end(const uint8_t *payload, uint16_t length)
{
    if (!g_host.session_active)
    {
        send_status_response(DFU_CMD_END_IND, DFU_PROTO_ERR_UNEXPECT_STATE);
        return;
    }

    dfu_engine_err_t err = dfu_engine_commit();
    if (err != DFU_ENGINE_ERR_NONE)
    {
        LOG_E("commit failed: %d", err);
        send_status_response(DFU_CMD_END_IND, engine_err_to_proto(err));
        notify_app(DFU_MODE_EVT_ERROR, err);
        return;
    }

    g_host.session_active = false;

    send_status_response(DFU_CMD_END_IND, DFU_PROTO_ERR_NONE);
    notify_app(DFU_MODE_EVT_ALL_COMPLETE, DFU_ENGINE_ERR_NONE);
    LOG_I("trans_end: commit complete, upgrade finished");
}

/**
 * @brief Handle ABORT command
 */
static void handle_abort(void)
{
    LOG_I("abort requested by host");
    dfu_engine_abort();
    g_host.session_active = false;
    notify_app(DFU_MODE_EVT_ABORTED, DFU_ENGINE_ERR_ABORTED);
}

/**
 * @brief Handle READ_VER_REQ — Reply with device firmware version
 *
 * Response payload:
 *   [0..3]   hw_version  (4B LE)
 *   [4..7]   sdk_version (4B LE)
 *   [8..11]  fw_version  (4B LE)
 */
static void handle_read_ver_req(void)
{
    uint8_t rsp[12];
    memset(rsp, 0, sizeof(rsp));

    /*
     * Fill from SDK's dfu_get_version() which reads version info from
     * the flash address stored in RTC_BACKUP_NAND_OTA_DES.
     *
     * Layout: [0..3] hw_version  [4..7] sdk_version  [8..11] fw_version
     * Each is 4 bytes LE. dfu_get_version returns a raw byte buffer
     * of DFU_PART_VERSION_LEN (8 bytes) = hw_version(4) + fw_version(4).
     * SDK version is typically not included in the version response
     * for older protocols, so we fill it as zero.
     *
     * The host tool uses this for compatibility check before upgrade.
     */
    /*
     * Version query: read version from flash address stored in RTC backup register.
     * This is the same mechanism used by old SDK dfu_get_version() —
     * works identically for BLE, CDC, and PAN transports.
     *
     * RTC_BACKUP_NAND_OTA_DES stores (flash_addr >> 2).
     * Layout at that address: hw_version(4) + fw_version(4).
     */

#ifndef DFU_PART_VERSION_LEN
#define DFU_PART_VERSION_LEN 8
#endif

    uint8_t ver_buf[DFU_PART_VERSION_LEN];
    memset(ver_buf, 0, sizeof(ver_buf));

    uint32_t ver_addr = HAL_Get_backup(RTC_BACKUP_NAND_OTA_DES);
    ver_addr = ver_addr << 2;
    if (ver_addr != 0)
    {
        memcpy(ver_buf, (uint8_t *)ver_addr, DFU_PART_VERSION_LEN);
    }
    else
    {
        LOG_W("version address not set in RTC backup");
    }

    memcpy(&rsp[0], &ver_buf[0], 4);  /* hw_version */
    /* rsp[4..7] = sdk_version: leave as zero */
    memcpy(&rsp[8], &ver_buf[4], 4);  /* fw_version */

    send_response(DFU_CMD_READ_VER_RSP, rsp, sizeof(rsp));
}

/**
 * @brief Handle LINK_LOSE_CHECK_REQ — Heartbeat / link check
 */
static void handle_link_lose_check(void)
{
    uint8_t rsp[1] = { (uint8_t)DFU_PROTO_ERR_NONE };
    send_response(DFU_CMD_LINK_LOSE_CHECK_RSP, rsp, 1);
}

/*============================================================================
 * Command Dispatcher
 *============================================================================*/

/**
 * @brief Central command dispatcher
 *
 * Called for every complete protocol message (from either parser callback
 * or feed_message). Dispatches to the appropriate handler based on cmd_id.
 */
static void dispatch_command(dfu_cmd_id_t cmd_id,
                             const uint8_t *payload,
                             uint16_t length)
{
    LOG_D("dispatch cmd=%u len=%u", cmd_id, length);

    switch (cmd_id)
    {
    /* --- Session init --- */
    case DFU_CMD_INIT_REQ:
    case DFU_CMD_FORCE_INIT_REQ:
    case DFU_CMD_INIT_REQ_EXT:
        handle_init_req(cmd_id, payload, length);
        break;

    /* --- Resume --- */
    case DFU_CMD_RESUME_REQ:
        handle_resume_req(payload, length);
        break;

    /* --- Init/Resume confirmed by host → erase --- */
    case DFU_CMD_INIT_COMPLETE_IND:
    case DFU_CMD_INIT_COMPLETE_EXT:
    case DFU_CMD_RESUME_COMPLETE_IND:
        handle_init_complete(cmd_id);
        break;

    /* --- Image transfer --- */
    case DFU_CMD_IMG_SEND_START:
        handle_img_send_start(payload, length);
        break;

    case DFU_CMD_IMG_SEND_PKT:
        handle_img_send_pkt(payload, length);
        break;

    case DFU_CMD_IMG_SEND_END:
        handle_img_send_end(payload, length);
        break;

    /* --- Transmission end → commit --- */
    case DFU_CMD_TRANS_END:
        handle_trans_end(payload, length);
        break;

    /* --- Abort --- */
    case DFU_CMD_ABORT:
        handle_abort();
        break;

    /* --- Version query --- */
    case DFU_CMD_READ_VER_REQ:
        handle_read_ver_req();
        break;

    /* --- Link check --- */
    case DFU_CMD_LINK_LOSE_CHECK_REQ:
        handle_link_lose_check();
        break;

    /* --- Connection priority (no-op, just ACK) --- */
    case DFU_CMD_CONN_PRIORITY:
        LOG_D("conn_priority ignored");
        break;

    /* --- Retransmission (not supported in V2, NAK) --- */
    case DFU_CMD_RETRANS_REQ:
        send_status_response(DFU_CMD_RETRANS_RSP, DFU_PROTO_ERR_GENERAL);
        LOG_W("retransmission not supported in V2");
        break;

    /* --- Power off command --- */
    case DFU_CMD_POWER_OFF:
        LOG_I("power_off requested, rebooting...");
        /* Delay briefly for the log to flush, then reboot */
        rt_thread_mdelay(100);
        HAL_PMU_Reboot();
        break;

    default:
        LOG_W("unknown cmd_id: %u", cmd_id);
        break;
    }
}

/*============================================================================
 * Parser Callback (for byte-stream entry)
 *============================================================================*/

/**
 * @brief Called by Protocol Parser when a complete frame is assembled
 */
static void on_frame_received(dfu_cmd_id_t cmd_id,
                              const uint8_t *payload,
                              uint16_t length,
                              void *user_data)
{
    (void)user_data;
    dispatch_command(cmd_id, payload, length);
}

/*============================================================================
 * Public API — Lifecycle
 *============================================================================*/

int dfu_mode_host_init(const dfu_push_transport_t *transport,
                       bool is_message_transport,
                       const char *transport_name,
                       dfu_mode_callback_t callback,
                       void *user_data)
{
    if (!transport || !transport->send)
    {
        LOG_E("transport or transport->send is NULL");
        return -1;
    }

    if (g_host.initialized)
    {
        LOG_W("host mode already initialized, deinit first");
        dfu_mode_host_deinit();
    }

    memset(&g_host, 0, sizeof(g_host));

    /* Create protocol parser */
    g_host.parser = dfu_proto_parser_create(on_frame_received, NULL);
    if (!g_host.parser)
    {
        LOG_E("failed to create parser");
        return -1;
    }

    g_host.transport = transport;
    g_host.is_message_transport = is_message_transport;
    g_host.transport_name = transport_name ? transport_name : "unknown";
    g_host.callback  = callback;
    g_host.user_data = user_data;
    g_host.self_img_id = 0xFF;  /* No filtering by default */
    g_host.skip_current_image = false;
    g_host.initialized = true;

    LOG_I("host mode initialized");
    return 0;
}

/**
 * @brief Set the image ID of the currently running partition
 *
 * When Host Mode receives IMG_SEND_START for an image matching this ID,
 * it enters skip mode: ACKs all packets without writing to flash.
 *
 * Call this after dfu_mode_host_init() but before any upgrade session.
 *
 * @param img_id  The img_id to skip (e.g. DFU_IMG_ID_HCPU=0 in user APP,
 *                DFU_IMG_ID_OTA_MANAGER=6 in DFU subprogram).
 *                Use 0xFF to disable filtering (write all images).
 */
void dfu_mode_host_set_self_img_id(uint8_t img_id)
{
    g_host.self_img_id = img_id;
    LOG_I("self_img_id set to %u (0xFF=disabled)", img_id);
}

void dfu_mode_host_deinit(void)
{
    if (!g_host.initialized)
        return;

    /* Abort any active session */
    if (g_host.session_active)
    {
        dfu_engine_abort();
        g_host.session_active = false;
    }

    /* Destroy parser */
    if (g_host.parser)
    {
        dfu_proto_parser_destroy(g_host.parser);
        g_host.parser = NULL;
    }

    g_host.initialized = false;
    LOG_I("host mode deinitialized");
}

/*============================================================================
 * Public API — Data Entry Points
 *============================================================================*/

int dfu_mode_host_feed_data(const uint8_t *data, uint32_t len)
{
    if (!g_host.initialized)
    {
        LOG_E("host mode not initialized");
        return -1;
    }

    if (!data || len == 0)
        return -1;

    /* Feed bytes to parser; parser fires on_frame_received → dispatch */
    return dfu_proto_parser_feed(g_host.parser, data, len);
}

int dfu_mode_host_feed_message(const uint8_t *data, uint32_t len)
{
    if (!g_host.initialized)
    {
        LOG_E("host mode not initialized");
        return -1;
    }

    if (!data || len < 4)
    {
        LOG_E("message too short: %u", len);
        return -1;
    }

    /*
     * BLE message format (matches dfu_tran_protocol_t):
     *   [0..1] message_id  (uint16_t LE)
     *   [2..3] length      (uint16_t LE)
     *   [4..]  payload     (length bytes)
     *
     * No AA55 header, no CRC — BLE handles framing/integrity.
     */
    uint16_t msg_id      = data[0] | ((uint16_t)data[1] << 8);
    uint16_t payload_len = data[2] | ((uint16_t)data[3] << 8);

    /* Sanity check */
    if ((uint32_t)(4 + payload_len) > len)
    {
        LOG_E("message length mismatch: declared=%u, available=%u",
              payload_len, len - 4);
        return -1;
    }

    const uint8_t *payload = (payload_len > 0) ? &data[4] : NULL;
    dispatch_command((dfu_cmd_id_t)msg_id, payload, payload_len);
    return 0;
}

/*============================================================================
 * Public API — Status Query
 *============================================================================*/

bool dfu_mode_host_is_busy(void)
{
    return g_host.initialized && g_host.session_active;
}