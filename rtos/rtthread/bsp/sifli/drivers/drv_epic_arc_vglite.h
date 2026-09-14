/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef DRV_EPIC_ARC_VGLITE_H
#define DRV_EPIC_ARC_VGLITE_H

#include "drv_epic.h"

/**
 * Opaque handle for a VGLite arc mask rendering job.
 * The internal structure holds the tessellated path data and
 * ping-pong output buffers used during A8 mask generation.
 */
typedef struct arc_job arc_job_t;

/**
 * Prepare a VGLite arc mask rendering job from an EPIC arc operation.
 *
 * Builds the tessellated path (outer arc, inner arc, optional round caps)
 * from the arc geometry parameters and allocates the output A8 buffers.
 *
 * @param arc      Pointer to the EPIC arc operation containing geometry
 * @param job_out  Output pointer receiving the allocated job handle
 * @return         RT_EOK on success, error code otherwise
 */
rt_err_t drv_epic_arc_vglite_prepare(const drv_epic_operation *arc, arc_job_t **job_out);

/**
 * Render a sub-rectangle of the arc A8 mask using VGLite.
 *
 * Draws the tessellated path into one of the ping-pong output buffers
 * selected by target_index, then configures the mask layer descriptor
 * to point at the rendered A8 data for EPIC compositing.
 *
 * @param job          Pointer to the prepared arc job
 * @param left         Left coordinate of the sub-rectangle to render
 * @param top          Top coordinate of the sub-rectangle to render
 * @param width        Width of the sub-rectangle
 * @param height       Height of the sub-rectangle
 * @param target_index Ping-pong buffer index (0 or 1)
 * @param mask         Output mask layer descriptor configured with the A8 data
 * @return             RT_EOK on success, error code otherwise
 */
rt_err_t drv_epic_arc_vglite_render(arc_job_t *job, int32_t left, int32_t top,
        int32_t width, int32_t height, uint8_t target_index,
        EPIC_LayerConfigTypeDef *mask);

/**
 * Release a VGLite arc mask rendering job and free its resources.
 *
 * Waits for any in-flight VGLite operations to complete, then frees
 * the output buffers and path data. If the job cannot be released
 * immediately (VGLite still busy), it is placed in a quarantined state
 * for deferred cleanup.
 *
 * @param job  Pointer to the job to release
 */
void drv_epic_arc_vglite_release(arc_job_t *job);

/**
 * Check whether the VGLite arc mask engine is idle.
 *
 * Returns true when there is no quarantined job pending cleanup,
 * meaning it is safe to close the VGLite hardware.
 *
 * @return  true if idle, false if a job is still pending
 */
bool drv_epic_arc_vglite_idle(void);

#endif
