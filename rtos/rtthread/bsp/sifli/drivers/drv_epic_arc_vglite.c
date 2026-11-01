/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file drv_epic_arc_vglite.c
 * @brief VGLite hardware-accelerated A8 mask generation for EPIC image arcs.
 *
 * This module implements the VGLite backend for arc image mask generation.
 * It tessellates the arc geometry (outer arc, inner arc, optional round caps)
 * into VGLite path commands, renders the path into A8 alpha buffers using
 * ping-pong double buffering, and provides the resulting mask to the EPIC
 * compositor for image arc rendering.
 */
#include "drv_epic_arc_vglite.h"
#include "drv_vglite.h"
#include "math.h"
#include "string.h"

/** Maximum number of path command words (opcode + coordinates) per arc job. */
#define ARC_PATH_WORDS 512
/** Pi constant used for angle-to-radian conversion in path construction. */
#define ARC_PI 3.14159265358979323846f

/**
 * Internal structure representing a VGLite arc mask rendering job.
 *
 * Holds the tessellated path data and ping-pong output buffers used
 * during A8 mask generation. The path is built once during prepare() and
 * reused across multiple render() calls for different sub-rectangles.
 */
struct arc_job
{
    vg_lite_path_t path;       /**< VGLite path descriptor (bounding box, format, data pointer) */
    vg_lite_buffer_t target[2];/**< Ping-pong output A8 buffers for CPU/GPU parallel execution */
    uint32_t count;            /**< Current number of words in the path command buffer */
    union
    {
        uint32_t opcode;       /**< VGLite path opcode (MOVE, LINE, CUBIC, CLOSE, END) */
        float coordinate;      /**< Path coordinate value (x or y) */
    } words[ARC_PATH_WORDS];   /**< Path command buffer storing opcodes and coordinates interleaved */
};

/**
 * Pointer to a job that could not be released immediately because VGLite
 * was still busy. It is retained for deferred cleanup once the hardware
 * becomes idle. NULL when no job is pending release.
 */
static arc_job_t *quarantined_job;

/**
 * Check whether the VGLite arc mask engine is idle.
 *
 * @return true if no quarantined job is pending cleanup, false otherwise
 */
bool drv_epic_arc_vglite_idle(void)
{
    return quarantined_job == NULL;
}

/**
 * Append a path command (opcode + coordinates) to the arc job's path buffer.
 *
 * Also updates the path bounding box to encompass all new coordinates.
 *
 * @param job          Pointer to the arc job
 * @param opcode       VGLite path opcode (VLC_OP_MOVE, VLC_OP_LINE, VLC_OP_CUBIC, etc.)
 * @param coordinates  Array of coordinate values (x, y pairs)
 * @param count        Number of coordinate values
 * @return             true if the command fit in the buffer, false if overflow
 */
static bool path_command(arc_job_t *job, uint32_t opcode, const float *coordinates, uint32_t count)
{
    if (job->count + count + 1 > ARC_PATH_WORDS) return false;
    if (!job->count && count >= 2)
    {
        job->path.bounding_box[0] = job->path.bounding_box[2] = coordinates[0];
        job->path.bounding_box[1] = job->path.bounding_box[3] = coordinates[1];
    }
    for (uint32_t index = 0; index + 1 < count; index += 2)
    {
        job->path.bounding_box[0] = fminf(job->path.bounding_box[0], coordinates[index]);
        job->path.bounding_box[1] = fminf(job->path.bounding_box[1], coordinates[index + 1]);
        job->path.bounding_box[2] = fmaxf(job->path.bounding_box[2], coordinates[index]);
        job->path.bounding_box[3] = fmaxf(job->path.bounding_box[3], coordinates[index + 1]);
    }
    job->words[job->count++].opcode = opcode;
    for (uint32_t index = 0; index < count; index++)
        job->words[job->count++].coordinate = coordinates[index];
    return true;
}

/**
 * Append a single point at a given polar coordinate to the path.
 *
 * Converts the angle from degrees to radians and computes the (x, y)
 * position using the provided center and radius.
 *
 * @param job       Pointer to the arc job
 * @param opcode    VGLite path opcode to use for this point
 * @param center_x  X coordinate of the arc center
 * @param center_y  Y coordinate of the arc center
 * @param radius    Radius from the center
 * @param angle     Angle in degrees (0 = right, clockwise)
 * @return          true on success, false on buffer overflow
 */
static bool path_point(arc_job_t *job, uint32_t opcode, float center_x, float center_y,
                       float radius, float angle)
{
    float radians = angle * ARC_PI / 180.0f;
    float coordinates[2] = {center_x + radius * cosf(radians), center_y + radius * sinf(radians)};
    return path_command(job, opcode, coordinates, 2);
}

/**
 * Approximate an arc segment using cubic Bezier curves.
 *
 * Splits the sweep angle into 30-degree segments, each approximated by
 * a cubic Bezier curve using the standard tangent control point formula.
 * This produces a smooth path that VGLite can tessellate efficiently.
 *
 * @param job       Pointer to the arc job
 * @param center_x  X coordinate of the arc center
 * @param center_y  Y coordinate of the arc center
 * @param radius    Arc radius
 * @param start     Start angle in degrees
 * @param sweep     Sweep angle in degrees (positive = clockwise)
 * @return          true on success, false on buffer overflow
 */
static bool path_arc(arc_job_t *job, float center_x, float center_y, float radius,
                     float start, float sweep)
{
    uint32_t segments = (uint32_t)ceilf(fabsf(sweep) / 30.0f);
    if (!segments || !radius) return true;
    float delta = sweep * ARC_PI / (180.0f * segments);
    float tangent = 4.0f / 3.0f * tanf(delta / 4.0f);
    for (uint32_t index = 0; index < segments; index++)
    {
        float angle = start * ARC_PI / 180.0f + index * delta;
        float next = angle + delta;
        float coordinates[6] = {
            center_x + radius * (cosf(angle) - tangent * sinf(angle)),
            center_y + radius * (sinf(angle) + tangent * cosf(angle)),
            center_x + radius * (cosf(next) + tangent * sinf(next)),
            center_y + radius * (sinf(next) - tangent * cosf(next)),
            center_x + radius * cosf(next), center_y + radius * sinf(next)
        };
        if (!path_command(job, VLC_OP_CUBIC, coordinates, 6)) return false;
    }
    return true;
}

/**
 * Build the complete tessellated path for an arc operation.
 *
 * Constructs a closed path representing the arc annulus:
 * - Outer arc from start to end angle
 * - Radial line connecting outer to inner radius at the end angle
 * - Inner arc from end back to start angle
 * - Radial line connecting inner back to outer radius at the start angle
 * - Optional round caps at both ends (if configured)
 *
 * For full 360-degree arcs, the path consists of two concentric circles.
 *
 * @param job   Pointer to the arc job to populate
 * @param arc   Pointer to the EPIC arc operation with geometry parameters
 * @return      true on success, false on buffer overflow
 */
static bool build_path(arc_job_t *job, const drv_epic_operation *arc)
{
    float center_x = arc->desc.arc.center_x;
    float center_y = arc->desc.arc.center_y;
    float outer = arc->desc.arc.radius;
    float thickness = arc->desc.arc.width > arc->desc.arc.radius ? arc->desc.arc.radius : arc->desc.arc.width;
    float inner = outer - thickness;
    float start = arc->desc.arc.start_angle % 360;
    bool full = arc->desc.arc.start_angle != arc->desc.arc.end_angle &&
                arc->desc.arc.start_angle % 360 == arc->desc.arc.end_angle % 360;
    float sweep = full ? 360 : (arc->desc.arc.end_angle % 360 + 360 - arc->desc.arc.start_angle % 360) % 360;
    float end = start + sweep;
    if (!path_point(job, VLC_OP_MOVE, center_x, center_y, outer, start) ||
            !path_arc(job, center_x, center_y, outer, start, sweep)) return false;
    if (full)
    {
        if (!path_command(job, VLC_OP_CLOSE, NULL, 0)) return false;
        if (inner > 0 && (!path_point(job, VLC_OP_MOVE, center_x, center_y, inner, end) ||
                         !path_arc(job, center_x, center_y, inner, end, -sweep) ||
                         !path_command(job, VLC_OP_CLOSE, NULL, 0))) return false;
    }
    else
    {
        if (!path_point(job, VLC_OP_LINE, center_x, center_y, inner, end) ||
                !path_arc(job, center_x, center_y, inner, end, -sweep) ||
                !path_command(job, VLC_OP_CLOSE, NULL, 0)) return false;
        for (uint32_t cap = 0; cap < 2; cap++)
        {
            if (!(cap ? arc->desc.arc.round_end : arc->desc.arc.round_start)) continue;
            float radians = (cap ? end : start) * ARC_PI / 180.0f;
            float cap_x = center_x + (outer - thickness / 2) * cosf(radians);
            float cap_y = center_y + (outer - thickness / 2) * sinf(radians);
            if (!path_point(job, VLC_OP_MOVE, cap_x, cap_y, thickness / 2, 0) ||
                    !path_arc(job, cap_x, cap_y, thickness / 2, 0, 360) ||
                    !path_command(job, VLC_OP_CLOSE, NULL, 0)) return false;
        }
    }
    return path_command(job, VLC_OP_END, NULL, 0);
}

/**
 * Prepare a VGLite arc mask rendering job from an EPIC arc operation.
 *
 * Allocates the job structure, builds the tessellated path from the arc
 * geometry, initializes the VGLite path descriptor, and allocates the
 * ping-pong A8 output buffers. A8 requires the buffer base address and
 * every row stride to be 64-byte aligned, so the tile width is rounded
 * up to a 64-byte multiple to keep the stride aligned.
 *
 * @param arc      Pointer to the EPIC arc operation containing geometry
 * @param job_out  Output pointer receiving the allocated job handle
 * @return         RT_EOK on success, error code otherwise
 */
rt_err_t drv_epic_arc_vglite_prepare(const drv_epic_operation *arc, arc_job_t **job_out)
{
    if (quarantined_job) return -RT_ERROR;
    if (!arc || !job_out) return -RT_EINVAL;
    *job_out = NULL;
    arc_job_t *job = rt_malloc(sizeof(*job));
    if (!job) return -RT_ENOMEM;
    memset(job, 0, sizeof(*job));
    if (!build_path(job, arc))
    {
        rt_free(job);
        return -RT_ERROR;
    }
    float bounds[4];
    memcpy(bounds, job->path.bounding_box, sizeof(bounds));
    vg_lite_error_t error = vg_lite_init_path(&job->path, VG_LITE_FP32, VG_LITE_HIGH,
                             job->count * sizeof(job->words[0]), job->words,
                             bounds[0], bounds[1], bounds[2], bounds[3]);
    if (error != VG_LITE_SUCCESS)
    {
        vglite_print_error(__func__, __LINE__, error);
        rt_free(job);
        return -RT_ERROR;
    }
    for (uint8_t index = 0; index < 2; index++)
    {
        /* A8 buffers require the base address and the row stride to be
         * 64-byte aligned; round the width up to a multiple of 64 bytes
         * so that the stride meets the alignment requirement. */
        job->target[index].width = RT_ALIGN(ARC_IMG_TILE_WIDTH, 64);
        job->target[index].height = ARC_IMG_TILE_HEIGHT;
        job->target[index].format = VG_LITE_A8;
        job->target[index].tiled = VG_LITE_LINEAR;
        error = vg_lite_allocate(&job->target[index]);
        if (error != VG_LITE_SUCCESS)
        {
            vglite_print_error(__func__, __LINE__, error);
            for (uint8_t release = 0; release < index; release++)
                vg_lite_free(&job->target[release]);
            vg_lite_clear_path(&job->path);
            rt_free(job);
            return -RT_ERROR;
        }
        if (!job->target[index].memory || job->target[index].stride < ARC_IMG_TILE_WIDTH ||
                (job->target[index].stride & 63) ||
                ((uintptr_t)job->target[index].memory & 63) ||
                (job->target[index].address & 63))
        {
            vg_lite_free(&job->target[index]);
            for (uint8_t release = 0; release < index; release++)
                vg_lite_free(&job->target[release]);
            vg_lite_clear_path(&job->path);
            rt_free(job);
            return -RT_ERROR;
        }
    }
    *job_out = job;
    return RT_EOK;
}

/**
 * Render a sub-rectangle of the arc A8 mask using VGLite.
 *
 * Clears the selected ping-pong buffer, draws the tessellated path into it
 * using non-zero fill rule, waits for VGLite to finish, then configures the
 * mask layer descriptor to point at the rendered A8 data for EPIC compositing.
 * A translation matrix is applied so path coordinates map correctly to the
 * sub-rectangle's position.
 *
 * @param job          Pointer to the prepared arc job
 * @param left         Left coordinate of the sub-rectangle
 * @param top          Top coordinate of the sub-rectangle
 * @param width        Width of the sub-rectangle (must be <= ARC_IMG_TILE_WIDTH)
 * @param height       Height of the sub-rectangle (must be <= ARC_IMG_TILE_HEIGHT)
 * @param target_index Ping-pong buffer index (0 or 1)
 * @param mask         Output mask layer descriptor configured with the A8 data
 * @return             RT_EOK on success, error code otherwise
 */
rt_err_t drv_epic_arc_vglite_render(arc_job_t *job, int32_t left, int32_t top,
        int32_t width, int32_t height, uint8_t target_index,
        EPIC_LayerConfigTypeDef *mask)
{
    if (!job || !mask || target_index >= 2 || width <= 0 || width > ARC_IMG_TILE_WIDTH ||
            height <= 0 || height > ARC_IMG_TILE_HEIGHT)
        return -RT_EINVAL;
    vg_lite_buffer_t *target = &job->target[target_index];
    target->width = width;
    target->height = height;
    vg_lite_error_t error;
    vg_lite_matrix_t matrix;
    vg_lite_identity(&matrix);
    vg_lite_translate(-left, -top, &matrix);
    mpu_dcache_clean(target->memory, target->stride * ARC_IMG_TILE_HEIGHT);
    error = vg_lite_clear(target, NULL, 0);
    if (error == VG_LITE_SUCCESS)
        error = vg_lite_draw(target, &job->path, VG_LITE_FILL_NON_ZERO, &matrix,
                             VG_LITE_BLEND_NONE, 0xffffffff);
    vg_lite_error_t finished = vg_lite_finish();
    if (finished != VG_LITE_SUCCESS)
    {
        vglite_print_error(__func__, __LINE__, finished);
        return -RT_ERROR;
    }
    mpu_dcache_invalidate(target->memory, target->stride * height);
    if (error == VG_LITE_SUCCESS)
    {
        HAL_EPIC_LayerConfigInit(mask);
        mask->data = (uint8_t *)target->memory;
        mask->color_mode = EPIC_INPUT_A8;
        mask->width = width;
        mask->height = height;
        mask->total_width = (uint16_t)target->stride;
        mask->x_offset = left;
        mask->y_offset = top;
        mask->color_en = true;
        mask->color_r = 0xff;
        mask->color_g = 0xff;
        mask->color_b = 0xff;
        mask->ax_mode = ALPHA_BLEND_MASK;
        mask->data_size = (uint32_t)target->stride * height;
    }
    if (error != VG_LITE_SUCCESS) vglite_print_error(__func__, __LINE__, error);
    return error == VG_LITE_SUCCESS ? RT_EOK : -RT_ERROR;
}

/**
 * Release a VGLite arc mask rendering job and free its resources.
 *
 * Clears the VGLite path, frees both ping-pong output buffers, and
 * frees the job structure. If the job is the quarantined job (pending
 * deferred cleanup), it is ignored to avoid double-free.
 *
 * @param job  Pointer to the job to release
 */
void drv_epic_arc_vglite_release(arc_job_t *job)
{
    if (!job || job == quarantined_job) return;
    vg_lite_clear_path(&job->path);
    for (uint8_t index = 0; index < 2; index++)
        if (job->target[index].handle) vg_lite_free(&job->target[index]);
    rt_free(job);
}
