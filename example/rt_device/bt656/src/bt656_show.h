/**
 * @file bt656_show.h
 * @brief BT656 example display and continuous preview interface.
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BT656_SHOW_H__
#define __BT656_SHOW_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Capture mode of the continuous preview.
 *
 * @note Both modes currently use the same continuous capture path; the mode is
 *       only stored so that `camera info` can report which one was requested.
 */
typedef enum
{
    BT656_PREVIEW_FRAME = 0,  /**< Requested as "frame"; same path as stream */
    BT656_PREVIEW_STREAM,     /**< Requested as "stream"; same path as frame */
} camera_preview_mode_t;

/**
 * @brief Capture and show one frame (snapshot).
 *
 * @param[in] timeout_ms Capture timeout in milliseconds; <= 0 uses the default.
 *
 * @return Error code.
 *
 * @pre bt656_init() + bt656_start() must already have been called.
 */
int camera_show_once(rt_int32_t timeout_ms);

/**
 * @brief Start the continuous preview.
 *
 * @details Initializes bt656_init() / bt656_start() internally. Each start is a
 *          new session: decoder soft reset -> reload register table -> wait for
 *          a stable lock -> DCMI init -> start streaming -> establish the LCD
 *          self-scan.
 *
 * @param[in] mode        Preview mode: FRAME / STREAM.
 * @param[in] interval_ms Minimum interval between two frames in milliseconds;
 *                        0 means full speed.
 *
 * @return Error code.
 * @retval RT_EOK Success.
 * @retval -RT_EBUSY Preview is already running.
 * @retval -RT_ENOMEM Thread creation failed.
 */
int camera_preview_start(camera_preview_mode_t mode, rt_int32_t interval_ms);

/**
 * @brief Stop the preview and wait for the thread to exit.
 *
 * @return Error code.
 * @retval RT_EOK Fully stopped.
 * @retval -RT_ETIMEOUT The thread did not exit within 3 seconds.
 */
int camera_preview_stop(void);

/**
 * @brief Query whether the preview is running.
 *
 * @return RT_TRUE when running.
 */
rt_bool_t camera_preview_running(void);

/**
 * @brief Get the current preview mode.
 *
 * @return Preview mode.
 */
camera_preview_mode_t camera_preview_mode(void);

/**
 * @brief Read preview statistics.
 *
 * @param[out] frames Number of captured frames (may be NULL).
 * @param[out] fails  Number of capture failures (may be NULL).
 *
 * @note The statistics are retained after stop.
 */
void camera_preview_stats(rt_uint32_t *frames, rt_uint32_t *fails);

#ifdef __cplusplus
}
#endif

#endif /* __BT656_SHOW_H__ */
