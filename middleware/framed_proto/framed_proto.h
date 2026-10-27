/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file framed_proto.h
 * @brief Framed message protocol: consumer interface.
 *
 * This layer only moves 4-byte-header messages between the peer and the
 * consumer registered through framed_proto_set_handler(). It never parses
 * message contents: firmware update, screen cast and any other feature each
 * supply their own handler on top of this single channel.
 *
 * The peer-side specification calls this protocol V3.
 */

#ifndef __FRAMED_PROTO_H
#define __FRAMED_PROTO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Category id of the firmware-update / screen-cast channel. */
#define FRAMED_PROTO_CATE_ID        0x01

/** Connection-interval threshold: above this a busy link is asked to switch
 *  to the high-performance setting. Unit: 1.25 ms. */
#define FRAMED_PROTO_INTERVAL_SLOW      20

/** Minimum link supervision timeout reported to the peer. Unit: 10 ms. */
#define FRAMED_PROTO_SYNC_DEFAULT_TIMEOUT   1000

/**
 * @brief One complete protocol message.
 *
 * @p length counts only the payload bytes in @p data; the 4-byte header is
 * not included.
 */
typedef struct
{
    uint16_t message_id;
    uint16_t length;
    uint8_t data[0];
} framed_proto_msg_t;

/**
 * @brief Consumer callback set.
 *
 * Every entry is optional; fill in NULL to skip it.
 */
typedef struct
{
    /**
     * A complete message has arrived.
     * @param msg     message, valid only during this callback
     * @param buf_len length of the buffer handed over by the link, header
     *                included
     */
    void (*on_msg)(framed_proto_msg_t *msg, uint16_t buf_len);

    /** The peer has disconnected. */
    void (*on_close)(void);

    /** The link reported a transmission error; codes are in framed_proto_link.h. */
    void (*on_error)(uint8_t error);

    /** The link reported new transmission parameters; see framed_proto_link.h. */
    void (*on_configure)(uint16_t len, uint8_t *data);

    /** The last queued message has been fully transmitted. */
    void (*on_tx_done)(void);

    /**
     * Non-zero while a transfer is running; the link uses it to raise the
     * connection interval to the high-performance setting.
     */
    uint8_t (*is_busy)(void);
} framed_proto_handler_t;

/**
 * @brief Register the consumer. Call once before the link is established.
 */
void framed_proto_set_handler(const framed_proto_handler_t *handler);

/* -------------------------------------------------------------------------
 * Message helpers
 * ---------------------------------------------------------------------- */

/**
 * @brief Allocate a message with @p length payload bytes.
 * @return pointer to the payload, or NULL when allocation fails
 *
 * The returned pointer addresses @c msg->data, not the start of the
 * allocation. Hand it to framed_proto_msg_send(), or release a cancelled
 * message with framed_proto_msg_free().
 */
uint8_t *framed_proto_msg_alloc(uint16_t msg_id, uint16_t length);

/**
 * @brief Release a message obtained from framed_proto_msg_alloc() without
 *        sending it.
 */
void framed_proto_msg_free(uint8_t *msg);

/**
 * @brief Hand a message obtained from framed_proto_msg_alloc() to the link.
 *
 * The buffer is released whatever the return value is.
 * @return 0 on success, -1 when the message cannot be queued
 */
int8_t framed_proto_msg_send(uint8_t *msg);

/* -------------------------------------------------------------------------
 * Response builders
 *
 * What goes on the wire is the caller's decision; these functions only take
 * care of the layout and the framing. Firmware-update and streaming products
 * share the same set.
 * ---------------------------------------------------------------------- */

/** Protocol version this side reports. It has been on the wire since the
 *  channel first existed. */
#define FRAMED_PROTO_CODE_VERSION       6

/** The only result value on the wire that means "no error". */
#define FRAMED_PROTO_RESULT_OK          0

/**
 * @brief Parameters of the file-init response, all decided by the product.
 */
typedef struct
{
    uint16_t result;
    uint16_t resume_state;      /**< 0 = fresh transfer, 1 = resume */
    uint32_t resume_index;      /**< file index the resume starts from */
    uint16_t fs_block;          /**< storage block size told to the peer */
    uint8_t  num_of_rsp_file;   /**< acknowledge every N file packets */
    uint8_t  num_of_rsp_bin;    /**< acknowledge every N image packets */
    uint16_t packet_length_file;/**< resource packet size asked of the peer */
    uint16_t packet_length_bin; /**< image packet size asked of the peer */
} framed_proto_rsp_file_init_t;

/** @brief Answer the version request (message 41). */
void framed_proto_rsp_version(uint16_t result, uint8_t need_reboot);

/** @brief Answer the file-init request (message 44). */
void framed_proto_rsp_file_init(const framed_proto_rsp_file_init_t *p);

/** @brief Answer the file-init-completed request (message 46). */
void framed_proto_rsp_file_init_completed(uint16_t result, uint32_t left_block);

/** @brief Answer the file-start request that carries transfer parameters
 *         (message 48). */
void framed_proto_rsp_file_start_ctrl(uint16_t result, uint8_t resume_state,
                                      uint8_t num_of_rsp, uint32_t packet_index,
                                      uint16_t image_id);

/** @brief Answer a plain file-start request (message 48). */
void framed_proto_rsp_file_start(uint16_t result);

/** @brief Answer one file packet (message 50). */
void framed_proto_rsp_file_packet(uint16_t result, uint32_t new_index);

/** @brief Answer the file-end request (message 52). */
void framed_proto_rsp_file_end(uint16_t result);

/** @brief Answer the whole-transfer-end request (message 54). */
void framed_proto_rsp_file_total_end(uint16_t result);

/** @brief Report a continuous-stream error (message 60). */
void framed_proto_rsp_continual_error(uint16_t error_type);

/** @brief Ask the peer to switch continuous-stream mode (message 61). */
void framed_proto_rsp_continual_mode(uint16_t mode, uint16_t reserved1,
                                     uint16_t reserved2);

/* -------------------------------------------------------------------------
 * Link supervision
 *
 * The link may fall silent while a transfer is running. The product tells the
 * timer which kind of transfer it is waiting for and the timer reports back
 * when nothing arrives in time; what to do about it is up to the product.
 * ---------------------------------------------------------------------- */

/** @brief Kind of transfer being supervised. */
typedef enum
{
    FRAMED_PROTO_SYNC_DOWNLOAD,
    FRAMED_PROTO_SYNC_RSP,
    FRAMED_PROTO_SYNC_CONTINUAL_FILE,
    FRAMED_PROTO_SYNC_CONTINUAL_FILE_LOST,
} framed_proto_sync_type_t;

/** How long an image transfer may stay silent before it counts as stuck.
 *  Unit: ms. */
#ifndef FRAMED_PROTO_SYNC_FILE_DOWNLOAD_TIMEOUT
#define FRAMED_PROTO_SYNC_FILE_DOWNLOAD_TIMEOUT         15000
#endif

/** Maximum gap allowed between two continuous-stream packets. Unit: ms. */
#ifndef FRAMED_PROTO_SYNC_CONTINUAL_FILE_TIMEOUT
#define FRAMED_PROTO_SYNC_CONTINUAL_FILE_TIMEOUT        2000
#endif

/** How long a stuck continuous stream may stay silent before the peer is
 *  considered lost. Unit: ms. */
#ifndef FRAMED_PROTO_SYNC_CONTINUAL_FILE_LOST_TIMEOUT
#define FRAMED_PROTO_SYNC_CONTINUAL_FILE_LOST_TIMEOUT   6000
#endif

/** @brief The supervised transfer timed out; @p sync_type is one of
 *         framed_proto_sync_type_t. */
typedef void (*framed_proto_sync_timeout_cb_t)(uint8_t sync_type);

/** @brief Register the supervision callback. Call once before a transfer
 *         starts. */
void framed_proto_sync_set_timeout_cb(framed_proto_sync_timeout_cb_t cb);

/** @brief Arm the timer for @p sync_type. An already armed timer is not
 *         reset. */
void framed_proto_sync_start(uint8_t sync_type);

/** @brief Stop the timer. Safe to call when it is not armed. */
void framed_proto_sync_end(void);

/** @brief Non-zero while the timer is armed. */
uint8_t framed_proto_sync_is_on(void);

/* -------------------------------------------------------------------------
 * Link -> protocol layer
 * ---------------------------------------------------------------------- */

/**
 * @brief A whole frame has arrived. The link layer has already removed its
 *        own framing header.
 */
void framed_proto_rx_frame(uint8_t cate_id, uint8_t *payload, uint16_t len);

/** @brief The link hit a transmission error. */
void framed_proto_rx_error(uint8_t error);

/** @brief The link has been disconnected. */
void framed_proto_rx_closed(void);

/* -------------------------------------------------------------------------
 * Session and link control
 * ---------------------------------------------------------------------- */

/**
 * @brief Ready callback, invoked once the link layer is usable.
 *
 * framed_proto_init() enables the Bluetooth stack and starts the serial
 * transmission service. It never touches advertising: deciding what to do
 * once the link is ready - for example starting connectable advertising - is
 * the product's business. May be NULL.
 */
typedef void (*framed_proto_ready_cb_t)(void);

/** @brief Register the ready callback. Call once, before framed_proto_init(). */
void framed_proto_set_ready_cb(framed_proto_ready_cb_t cb);

/**
 * @brief Bring the link up: enable the Bluetooth stack and start the
 *        serial-transmission service.
 *
 * Advertising is not started here; use @ref framed_proto_set_ready_cb if the
 * product needs to advertise once the link is ready.
 *
 * @return 0 on success, -1 when the build has no BLE support
 */
int8_t framed_proto_init(void);

/**
 * @brief Ask the peer to disconnect.
 * @return 0 on success, -1 when the build has no BLE support
 */
int8_t framed_proto_session_close(void);

/**
 * @brief Reboot right after the link disconnects, or immediately when it is
 *        already disconnected.
 */
void framed_proto_set_reboot_after_disconnect(void);

/**
 * @brief Flow control around the last message of a transfer.
 * @return 1 when the transmit queue has drained; on 0 the caller should wait
 *         for framed_proto_handler_t::on_tx_done
 */
uint8_t framed_proto_last_packet_wait(void);

/**
 * @brief Record the transmit queue depth before the last message is queued.
 */
uint8_t framed_proto_record_tx(void);

/** @brief Ask the link for a high-performance connection interval. */
void framed_proto_request_connection_priority(void);

/** @brief Hand the connection interval back to the low-power setting. */
void framed_proto_recovery_connection(void);

/** @brief Current supervision timeout of the link. */
uint16_t framed_proto_get_supervision_timeout(void);

/* -------------------------------------------------------------------------
 * Protocol message ids
 *
 * These numbers travel on the wire and are shared with the phone app; never
 * change or renumber an existing entry.
 * ---------------------------------------------------------------------- */

/** @brief Message ids carried in framed_proto_msg_t::message_id. */
typedef enum
{
    FRAMED_PROTO_LINK_LOSE_CHECK_REQ                = 35,
    FRAMED_PROTO_LINK_LOSE_CHECK_RSP                = 36,

    FRAMED_PROTO_VERSION_REQUEST                    = 40,
    FRAMED_PROTO_VERSION_RESPONSE                   = 41,
    FRAMED_PROTO_REBOOT_COMMAND                     = 42,
    FRAMED_PROTO_FILE_INIT_REQUEST                  = 43,
    FRAMED_PROTO_FILE_INIT_RESPONSE                 = 44,
    FRAMED_PROTO_FILE_INIT_COMPLETED_REQUEST        = 45,
    FRAMED_PROTO_FILE_INIT_COMPLETED_RESPONSE       = 46,
    FRAMED_PROTO_FILE_START_REQUEST                 = 47,
    FRAMED_PROTO_FILE_START_RESPONSE                = 48,
    FRAMED_PROTO_FILE_PACKET                        = 49,
    FRAMED_PROTO_FILE_PACKET_RESPONSE               = 50,
    FRAMED_PROTO_FILE_END_REQUEST                   = 51,
    FRAMED_PROTO_FILE_END_RESPONSE                  = 52,
    FRAMED_PROTO_FILE_TOTAL_END_REQUEST             = 53,
    FRAMED_PROTO_FILE_TOTAL_END_RESPONSE            = 54,
    FRAMED_PROTO_FILE_ABORT_COMMAND                 = 55,
    FRAMED_PROTO_CANCEL_REQUEST                     = 56,
    FRAMED_PROTO_CANCEL_RESPONSE                    = 57,
    FRAMED_PROTO_LINK_LOSE_CHECK_FINISH             = 58,
    FRAMED_PROTO_FILE_CONTINUAL_PACKET              = 59,
    FRAMED_PROTO_FILE_CONTINUAL_PACKET_ERROR        = 60,
    FRAMED_PROTO_FILE_CONTINUAL_PACKET_MODE_CHANGE  = 61,
} framed_proto_msg_id_t;

/** @brief Content type declared in the version request. */
typedef enum
{
    FRAMED_PROTO_DATA_TYPE_PHOTO_PREVIEW    = 7,
    FRAMED_PROTO_DATA_TYPE_DFU              = 10,
    FRAMED_PROTO_DATA_TYPE_OTA_MANAGER      = 11,
    FRAMED_PROTO_DATA_TYPE_CONTINUAL_FILE   = 17,
} framed_proto_data_type_t;

/** @brief File type declared in the file-init / file-start request. */
typedef enum
{
    FRAMED_PROTO_FILE_TYPE_RES,
    FRAMED_PROTO_FILE_TYPE_RES_WITHOUT_PATH,
    FRAMED_PROTO_FILE_TYPE_OTA_CTRL,
    FRAMED_PROTO_FILE_TYPE_OTA_BIN,
    FRAMED_PROTO_FILE_TYPE_DIRECT_BIN,
} framed_proto_file_type_t;

/**
 * @brief Transfer progress the protocol engine can read and write.
 *
 * Deliberately free of storage concepts: no image header, no partition, no
 * key. A streaming-only product such as screen cast simply keeps the
 * firmware-related counters at 0.
 */
typedef struct
{
    uint32_t downloaded_count;  /**< files received so far */
    uint32_t all_file_count;    /**< total number of files the peer announced */
    uint32_t all_file_length;   /**< total byte count the peer announced */
    uint32_t res_file_size;     /**< size of the latest resource file */

    uint32_t fw_version;
    uint32_t hw_version;
    uint32_t sdk_version;
    uint32_t reserved;          /**< product-defined flags, never interpreted */

    uint8_t  type;              /**< see framed_proto_data_type_t */
    uint8_t  state;             /**< see framed_proto_state_t */
    uint8_t  fw_state;          /**< see framed_proto_fw_state_t */
    uint8_t  ota_info_id;
} framed_proto_progress_t;

/* -------------------------------------------------------------------------
 * Product-side sink
 *
 * What the protocol engine needs from the product built on top of it. An OTA
 * product registers a sink that writes flash and installs; a screen-cast
 * product registers one that hands the payload to the renderer. The engine
 * itself never touches storage or verification.
 * ---------------------------------------------------------------------- */

typedef struct
{
    /**
     * @brief Store one block of file data.
     * @param img_id image the block belongs to
     * @param offset offset inside that image
     * @return 0 on success; a negative value makes the engine report an error
     *         to the peer
     */
    int (*on_file_data)(uint8_t img_id, uint32_t offset, uint8_t *data, uint32_t len);

    /**
     * @brief One file has been received in full.
     * @param file_type see framed_proto_file_type_t
     * @param total_len total bytes received for this file
     */
    int (*on_file_end)(uint16_t file_type, uint32_t total_len);

    /**
     * @brief Everything is in; install now.
     *
     * A streaming product just returns 0: there is nothing to install.
     */
    int (*on_install)(void);

    /** @brief Verify a received data block (OTA only). May be NULL. */
    int (*on_verify)(uint8_t *data, uint32_t len);

    /** @brief Read the current progress. May be NULL. */
    void (*get_progress)(framed_proto_progress_t *out);

    /** @brief Publish progress; persist it yourself if needed. */
    void (*set_progress)(const framed_proto_progress_t *in);

    /**
     * @brief Non-zero when the peer is streaming (screen cast) rather than
     *        updating firmware.
     *
     * Replaces the dfu_type_is_video_stream() checks that used to be scattered
     * across the OTA engine.
     */
    uint8_t (*is_stream)(void);
} framed_proto_sink_t;

/**
 * @brief Register the product-side implementation. Call once before the link
 *        is established.
 */
void framed_proto_set_sink(const framed_proto_sink_t *sink);

/**
 * @brief Whether the registered sink streams (see framed_proto_sink_t::is_stream).
 *        Returns 0 when no sink is registered.
 */
uint8_t framed_proto_is_stream(void);

/** @brief Currently registered sink, NULL when none is registered. */
const framed_proto_sink_t *framed_proto_get_sink(void);

/* -------------------------------------------------------------------------
 * Control state machine
 *
 * Shared by the protocol engine and the product, and persisted together with
 * the progress record, so the enum values must stay stable. The engine only
 * compares them; what each state means for storage is the product's business.
 * ---------------------------------------------------------------------- */

typedef enum
{
    FRAMED_PROTO_CTRL_IDLE,
    FRAMED_PROTO_CTRL_NEG,
    FRAMED_PROTO_CTRL_PREPARE_START,
    FRAMED_PROTO_CTRL_TRAN_START,
    FRAMED_PROTO_CTRL_INSTALL,
    FRAMED_PROTO_CTRL_UPDATING,
    FRAMED_PROTO_CTRL_FORCE_UPDATE,
    FRAMED_PROTO_CTRL_PREPARE_START_FORCE,
    FRAMED_PROTO_CTRL_UPDATED,
    FRAMED_PROTO_CTRL_OFFLINE_INSTALL_PREPARE,
    FRAMED_PROTO_CTRL_OFFLINE_INSTALL,
    FRAMED_PROTO_CTRL_REBOOT_INSTALL_PREPARE,
    FRAMED_PROTO_CTRL_REBOOT_INSTALL,
    FRAMED_PROTO_CTRL_USB_INSTALL,
    FRAMED_PROTO_CTRL_TF_INSTALL,
    FRAMED_PROTO_CTRL_OFFLINE_INSTALL_LITE,

    FRAMED_PROTO_CTRL_FILE_NEG = 15,
    FRAMED_PROTO_CTRL_FILE_PREPARE_START,
    FRAMED_PROTO_CTRL_FILE_RES_INSTALL,

    FRAMED_PROTO_CTRL_RAM_RUN_INSTALL,
    FRAMED_PROTO_CTRL_RAM_RUN_EXT_INSTALL,
    FRAMED_PROTO_CTRL_NOR_EXT_INSTALL,
} framed_proto_state_t;

typedef enum
{
    FRAMED_PROTO_CTRL_NORMAL_MODE,
    FRAMED_PROTO_CTRL_OTA_MODE,
    FRAMED_PROTO_CTRL_BOOTLOADER,
} framed_proto_mode_t;

typedef enum
{
    FRAMED_PROTO_CTRL_FW_NO_STATE,
    FRAMED_PROTO_CTRL_FW_DOWNLOADING,
    FRAMED_PROTO_CTRL_FW_INSTALLING,
    FRAMED_PROTO_CTRL_FW_INSTALLED,
} framed_proto_fw_state_t;

typedef enum
{
    FRAMED_PROTO_CTRL_IMG_STATE_IDLE,
    FRAMED_PROTO_CTRL_IMG_STATE_DOWNLOADING,
    FRAMED_PROTO_CTRL_IMG_STATE_DOWNLOADED_FAIL,
    FRAMED_PROTO_CTRL_IMG_STATE_DOWNLOADED,
    FRAMED_PROTO_CTRL_IMG_STATE_INSTALLING,
    FRAMED_PROTO_CTRL_IMG_STATE_INSTALLED,
} framed_proto_img_state_t;

/* -------------------------------------------------------------------------
 * Protocol context
 *
 * Everything the protocol state machine has to remember between messages. It
 * carries no storage layout, no image header and no event callback: those
 * belong to the product and reach the engine through the sink.
 * ---------------------------------------------------------------------- */

typedef struct
{
    /* What the peer declared in the version request. */
    uint8_t  phone_type;
    uint16_t remote_version;
    uint8_t  dfu_type;

    /* Per-file progress. */
    uint8_t  bin_resume_status;
    uint8_t  bin_num_of_rsp;
    uint8_t  res_num_of_rsp;
    uint8_t  res_num_of_rsp_set_value;

    uint16_t current_file_type;
    uint32_t current_file_packet_count;
    uint32_t current_file_length;

    uint32_t current_download_file_packet_index;
    uint32_t current_download_file_length;
    uint32_t downloaded_length;

    /* Retransmission. */
    uint8_t  retrans_state;
    uint8_t  retrans_time;
    uint8_t  retrans_image_time;
    uint32_t last_packet_index;

    /* Link supervision. */
    uint8_t  is_lose_check;
    uint32_t sync_size;

    /* File-start request, kept until the response has been assembled. */
    uint8_t  *start_buf;
    uint16_t start_buf_len;

    uint8_t  is_ctrl_received;
    uint8_t  is_reboot;

    volatile uint8_t dfu_work_thread_pending;
    volatile uint8_t dfu_work_pending;
} framed_proto_ctx_t;

#ifdef __cplusplus
}
#endif

#endif /* __FRAMED_PROTO_H */
