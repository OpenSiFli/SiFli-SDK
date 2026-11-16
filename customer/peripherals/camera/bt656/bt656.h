/**
 * @file bt656.h
 * @brief Generic BT.656 video input interface.
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @details
 * Applications depend only on this file, never on a specific decoder chip. The
 * BT.656 SAV/EAV sync codes, DCMI configuration, DMA and multi-buffer lending
 * are all handled here; chip differences (I2C address, register table,
 * supported formats) are registered inside the driver through
 * bt656_decoder_ops (see bt656_decoder.h).
 *
 * @par Frame ownership
 * - bt656_get() lends a frame and bt656_release() returns it; the buffer is not
 *   handed back to the DMA before it is released.
 * - frame->token is an internal token: pass it back unchanged, do not interpret
 *   or modify it.
 * - A slow consumer never builds up a backlog: the pool is fixed and new frames
 *   are dropped when no block is free, counted in bt656_stats()'s dropped.
 *
 * @par Capture model
 * Only continuous capture (stream) exists: DCMI stays enabled and switches to
 * the next buffer after every completed field/frame. Progressive formats use
 * the DCMI FRAME event, interlaced formats use the DMA byte count.
 *
 * @defgroup BT656 BT.656 video input
 * @{
 */

#ifndef __BT656_H__
#define __BT656_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief One BT.656 input format.
 *
 * @note Only geometry that is independent of the decoder chip; the sync codes
 *       are defined by the protocol and are not exposed here.
 */
struct bt656_format
{
    const char *name;       /**< Format name, for example "CVBS NTSC" */
    rt_uint16_t width;      /**< Active pixel width */
    rt_uint16_t height;     /**< Active lines per field/frame */
    rt_uint16_t stride;     /**< Bytes per line inside the buffer (2 * width for YUV422) */
    rt_uint8_t  interlace;  /**< 1: interlaced (single field), 0: progressive (full frame) */
    rt_uint32_t frame_bytes;/**< Bytes per field/frame (stride * height) */
};

/**
 * @brief Capture result of one frame/field.
 *
 * @note data points to a driver-internal buffer and stays valid until the
 *       matching bt656_release().
 */
struct bt656_frame
{
    rt_uint8_t  *data;      /**< Raw YUV422 data (driver-internal buffer) */
    rt_uint16_t  width;     /**< Pixel width */
    rt_uint16_t  height;    /**< Active lines (narrowed to the bytes actually received) */
    rt_uint16_t  stride;    /**< Bytes per line */
    rt_uint8_t   field;     /**< 1 = interlaced single field, 0 = progressive full frame */
    rt_uint32_t  seq;       /**< Frame sequence number */
    rt_uint32_t  bytes;     /**< Bytes actually valid in this frame */
    void        *token;     /**< Lend/return token: pass it back to bt656_release() unchanged */
};

/**
 * @brief Initialize BT.656 capture.
 *
 * @details Calls init() of the registered decoder and sets up DCMI/DMA.
 *          Calling it again when already initialized returns RT_EOK.
 *
 * @return Error code.
 * @retval RT_EOK Initialized, or already initialized.
 * @retval -RT_EIO No decoder registered, or HAL initialization failed.
 * @retval Other Error returned by the chip's init().
 */
int bt656_init(void);

/**
 * @brief Shut down BT.656 capture.
 *
 * @details Stops capture, shuts down DCMI/DMA and calls the chip's deinit().
 *
 * @return Error code; always RT_EOK for now.
 */
int bt656_deinit(void);

/**
 * @brief Start continuous capture.
 *
 * @details Decoder soft reset -> reload register table -> wait for a stable
 *          lock -> start DCMI/DMA. Does nothing when capture is already running.
 *
 * @return Error code.
 * @retval RT_EOK Started.
 * @retval -RT_EIO Not initialized, or no decoder registered.
 * @retval Other Error returned by the chip configuration or the HAL.
 */
int bt656_start(void);

/**
 * @brief Stop continuous capture.
 *
 * @warning Frames lent by bt656_get() but not released yet become invalid after
 *          this call; do not use the old handle afterwards. A bt656_release()
 *          with an old token may clear the in-use bit of a buffer owned by the
 *          next session.
 *
 * @return Error code; always RT_EOK for now.
 */
int bt656_stop(void);

/**
 * @brief Take one frame.
 *
 * @param[out] frame      Output frame information; on success data/width/
 *                        height/stride/token are valid.
 * @param[in]  timeout_ms Wait time: > 0 waits up to timeout_ms milliseconds;
 *                        <= 0 returns immediately.
 *
 * @return Error code.
 * @retval RT_EOK Frame taken.
 * @retval -RT_EINVAL frame is NULL.
 * @retval -RT_EIO bt656_start() was not called, or capture has stopped.
 * @retval -RT_ETIMEOUT No frame arrived before the timeout.
 *
 * @note The frame must be returned with bt656_release().
 * @note Only one consumer is supported: the semaphore and the single ready slot
 *       describe one frame in flight, so concurrent callers would steal each
 *       other's wakeup.
 */
int bt656_get(struct bt656_frame *frame, rt_int32_t timeout_ms);

/**
 * @brief Return one frame.
 *
 * @details Call it after the last read of frame->data. Sets data/token to NULL.
 *
 * @param[in,out] frame Frame returned by bt656_get().
 */
void bt656_release(struct bt656_frame *frame);

/**
 * @brief Read capture statistics.
 *
 * @param[out] frames  Number of frames completed and published to the upper
 *                     layer (may be NULL).
 * @param[out] dropped Number of frames dropped because they were not taken in
 *                     time or no free buffer existed (may be NULL).
 * @param[out] errors  Number of link errors such as OVR/FSM/PACK (may be NULL).
 */
void bt656_stats(rt_uint32_t *frames, rt_uint32_t *dropped,
                 rt_uint32_t *errors);

/**
 * @brief Select the analog input channel.
 *
 * @param[in] source Channel number (meaning is defined by the decoder chip).
 *
 * @return Error code.
 * @retval RT_EOK Set successfully.
 * @retval -RT_EIO No decoder registered.
 */
int bt656_set_source(rt_uint8_t source);

/**
 * @brief Get the video format selected at compile time.
 *
 * @return Format pointer; NULL when not initialized or no chip is registered.
 */
const struct bt656_format *bt656_get_format(void);

/**
 * @brief Read the chip ID.
 *
 * @return Chip ID; 0 when the read fails or no chip is registered.
 */
rt_uint16_t bt656_detect(void);

/**
 * @brief Read the signal lock state.
 *
 * @return 1 when locked, 0 when unlocked or the read failed.
 */
rt_uint8_t bt656_signal_locked(void);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* __BT656_H__ */
