/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DRV_VGLITE_H__
#define __DRV_VGLITE_H__

#include <rtdevice.h>
#include "board.h"
#include "vg_lite.h"
#include "vg_lite_platform.h"
#include "lv_img_buf.h"
#include "vg_lite_ex.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef USING_VGLITE

// Floating-point tolerance for zero checks and orthogonal validation (single-precision)
#ifndef VG_LITE_EPSILON
#define VG_LITE_EPSILON  ((float)(1e-6))
#define VG_LITE_IS_EQUAL_TO_ZERO(x)   (((float)(x) < VG_LITE_EPSILON) && ((float)(x) > -VG_LITE_EPSILON))
#endif

#ifndef M_PI
#define M_PI 3.1415926f
#endif

void vglite_print_error(const char *func, size_t line, vg_lite_error_t err);

#define VGLITE_CHECK_ERROR(Function)                     \
    error = Function;                                       \
    if (error > 0)                                          \
    {                                                       \
        vglite_print_error(__FUNCTION__, __LINE__, error);   \
        goto ErrorHandler;                                  \
    }

rt_err_t drv_vglite_open(void);
rt_err_t drv_vglite_close(void);


/**
 * @brief  Allocates GPU-aligned memory (meets VGLite hardware memory alignment requirements)
 * @details 1. Automatically aligns input size up to the next multiple of VGLITE_ALIGNMENT;
 *          2. Allocates memory from a dedicated framebuffer pool with thread safety;
 *          3. Lazy-initializes the framebuffer pool on first call.
 * @param  size: [Input/Output] Requested memory size (bytes) -> Actual aligned allocation size (bytes)
 * @return Success: Pointer to aligned memory; Failure: RT_NULL (invalid size or insufficient pool space)
 * @note   1. Memory allocated via this function MUST be freed with vglite_aligned_free(), not rt_free();
 *         2. Output size may be larger than input size (due to alignment) - use output size for memory operations;
 *         3. Thread-safe: Supports concurrent calls from multiple threads.
 */
void *vglite_aligned_malloc(size_t *size);

/**
 * @brief  Frees memory allocated by vglite_aligned_malloc()
 * @details 1. Validates the memory block to prevent invalid free operations;
 *          2. Merges adjacent free blocks to reduce fragmentation;
 *          3. Ensures thread safety via mutex protection.
 * @param  aligned_ptr: Pointer to memory allocated by vglite_aligned_malloc()
 * @return None
 * @note   1. Do NOT use this function to free memory from rt_malloc() or other allocators;
 *         2. Passing a NULL pointer is safe (no operation performed);
 *         3. Thread-safe: Supports concurrent calls from multiple threads;
 *         4. Debug validation: Detects double-frees and invalid pointers via magic number check.
 */
void vglite_aligned_free(void *aligned_ptr);

/**
 * @brief Calculate the Euclidean norm (magnitude) of a 3D vector
 * @param v Pointer to the input vg_lite_vertex_t vector (must be non-NULL)
 * @return Norm of the vector as a float
 */
float vg_lite_vec3_norm(const vg_lite_vertex_t *v);

/**
 * @brief Normalize a 3D vector to unit length (modifies the vector in-place)
 * @param v Pointer to the vector to normalize (must be non-NULL)
 * @return 0 on success, -1 on failure (zero vector input)
 */
int vg_lite_vec3_normalize(vg_lite_vertex_t *v);

/**
 * @brief Compute the cross product of two 3D vectors (a × b)
 * @param a Pointer to the first input vector (must be non-NULL)
 * @param b Pointer to the second input vector (must be non-NULL)
 * @return Result of the cross product as a vg_lite_vertex_t vector
 */
vg_lite_vertex_t vg_lite_vec3_cross(const vg_lite_vertex_t *a, const vg_lite_vertex_t *b);

/**
 * @brief Print a 4x4 matrix (for debugging/verification purposes)
 * @note Uses printf - disable in production if no console is available
 * @param name Name of the matrix (displayed in output)
 * @param mat Pointer to the vg_lite_matrix_4x4_t matrix to print (must be non-NULL)
 */
void vg_lite_mat4_print(const char *name, const vg_lite_matrix_4x4_t *mat);

void vg_lite_multiply2_4x4(vg_lite_matrix_4x4_t *out, vg_lite_matrix_4x4_t *lt, vg_lite_matrix_4x4_t *rt);


/**
 * @brief Compute the 4x4 homogeneous transformation matrix from reference frame to target frame
 *        (combines rotation and translation for origin offset)
 *
 * Constraints for target frame:
 * - X-axis is parallel to the reference frame's ZX-plane (Y-component = 0)
 * - Z-axis is specified by the input `target_z` (automatically normalized to unit length)
 * - Y-axis is derived via right-hand rule (tz × tx)
 *
 * @param target_z Input: Z-axis vector of the target coordinate system (no prior normalization needed, must be non-NULL)
 * @param origin_offset Input: Origin offset (target origin's coordinates in the reference frame: dx, dy, dz, must be non-NULL)
 * @param transform_mat Output: 4x4 transformation matrix (reference → target, must be non-NULL)
 *
 * @return 0 on success, -1 on failure (NULL parameters or zero target Z-axis)
 */
int vg_lite_get_transform_matrix_4x4(
    const vg_lite_vertex_t *target_z,
    const vg_lite_vertex_t *origin_offset,
    vg_lite_matrix_4x4_t *transform_mat
);

/**
 * @brief Compute the inverse transformation matrix (target frame → reference frame)
 *        Reverses the original transformation (first inverse translation, then inverse rotation)
 *
 * @note Assumes the input matrix is a **rigid body transformation** (3x3 submatrix is orthogonal).
 *       For non-orthogonal matrices (scaling/shearing), use a general inverse implementation.
 *
 * @param mat Input: Original transformation matrix (reference → target, must be non-NULL)
 * @param inv_mat Output: Inverse transformation matrix (target → reference, must be non-NULL)
 *
 * @return 0 on success, -1 on failure (NULL parameters)
 */
int vg_lite_inverse_matrix_4x4(
    const vg_lite_matrix_4x4_t *mat,
    vg_lite_matrix_4x4_t *inv_mat
);

/**
 * @brief Compute a 3x3 perspective projection matrix for projecting a 3D quadrilateral onto a 2D canvas
 *
 * This function handles perspective projection by first calculating the 3D quad's surface normal,
 * then applying perspective scaling based on the field of view (FOV). It supports edge cases
 * where the quad is aligned with specific axes (normal vector components are zero).
 *
 * @param fov Field of view (in degrees, defines the perspective "width")
 * @param canvas_w Width of the target 2D canvas (pixels or normalized units)
 * @param canvas_h Height of the target 2D canvas (pixels or normalized units)
 * @param quad Input: 3D quadrilateral to project (array of 4 vg_lite_vertex_t points, must be non-NULL)
 * @param matrix Output: 3x3 perspective projection matrix (must be non-NULL)
 *
 * @return vg_lite_error_t VG_LITE_SUCCESS on success, error code on failure
 *         (e.g., from vg_lite_perspective2_3x3 if called for non-Z-aligned quads)
 */
vg_lite_error_t vg_lite_perspective3_3x3(
    float fov,
    float canvas_w,
    float canvas_h,
    vg_lite_quad_t quad,
    vg_lite_matrix_t *matrix
);

/**
 * @brief Initialize a 4x4 homogeneous rotation matrix using Rodrigues' rotation formula
 *
 * Creates a matrix that rotates 3D points around an arbitrary unit vector (ux, uy, uz) by a specified angle.
 * The 4x4 matrix retains homogeneous coordinates (last row/column = [0,0,0,1]) for compatibility with
 * translation transformations.
 *
 * Core Math: Rodrigues' Rotation Formula (for arbitrary axis rotation)
 * R = I + s*[u]× + (1-c)*[u]×²
 * Where:
 * - R: 3x3 rotation matrix
 * - I: 3x3 identity matrix
 * - s: sin(theta) (theta = rotation angle in radians)
 * - c: cos(theta)
 * - [u]×: Skew-symmetric cross product matrix of the unit vector u
 * - [u]×²: Square of the skew-symmetric matrix
 *
 * @param matrix Output: 4x4 rotation matrix (initialized to identity first, must be non-NULL)
 * @param ux X-component of the rotation axis vector (will be normalized internally)
 * @param uy Y-component of the rotation axis vector (will be normalized internally)
 * @param uz Z-component of the rotation axis vector (will be normalized internally)
 * @param theta_deg Rotation angle in degrees (converted to radians internally)
 *
 * @note If the input axis vector (ux, uy, uz) is a zero vector (norm < 1e-6), the matrix remains identity
 *       (no rotation applied to avoid division by zero).
 * @note The 4x4 matrix structure:
 *       [ R00 R01 R02 0 ]
 *       [ R10 R11 R12 0 ]
 *       [ R20 R21 R22 0 ]
 *       [  0   0   0  1 ]
 */
void vg_lite_rodrigues_matrix_4x4(vg_lite_matrix_4x4_t *matrix, float ux, float uy, float uz, float theta_deg);

/**
 * @brief Render a textured image to a destination buffer with perspective projection
 *
 * Draws a source texture buffer onto a destination framebuffer using a precomputed
 * projective quad and specified vertical field-of-view (FOV). This function handles
 * perspective correction and texture mapping for 2D/3D rendering pipelines, and applies
 * rotation transformation via the input rotation matrix. It supports block-based image
 * processing through SRAM buffer to optimize rendering frame rate.
 *
 * @param[in]  fov       Vertical field-of-view angle in degrees for perspective projection
 * @param[out] dst       Pointer to the destination framebuffer buffer (must be initialized
 *                       and valid, with format compatible with src)
 * @param[in]  src       Pointer to the source texture buffer (must be initialized and valid
 *                       with RGBA/RGB texture data)
 * @param[in]  rot       Pointer to a 4x4 rotation matrix for texture/projective quad transformation
 * @param[in]  sram_buf  Pointer to a SRAM memory region used for block-based image processing
 *                       to optimize frame rate. Pass NULL to disable block-based processing.
 * @param[in]  buf_size  Size of the sram_buf memory region in bytes. Pass 0 when sram_buf is NULL
 *                       (indicates no block-based processing). Must match the actual SRAM buffer size
 *                       if block-based processing is enabled.
 *
 * @note The source texture buffer (src) and destination framebuffer (dst) must have compatible
 *       pixel formats (RGBA/RGB). Mismatched formats will trigger an error.
 * @note The projective quad used for rendering must contain homogeneous coordinates (w-component)
 *       to ensure correct perspective division and perspective correction.
 * @note Block-based processing via sram_buf is an optimization for frame rate; it splits the image
 *       into smaller blocks for rendering and uses SRAM to reduce memory access latency.
 */
void vg_lite_draw_img_texture(float fov, vg_lite_buffer_t *dst, vg_lite_buffer_t *src, vg_lite_matrix_4x4_t *rot, void *sram_buf, uint32_t buf_size);

#endif

#ifdef __cplusplus
}
#endif
#endif /* __DRV_VGLITE_H__ */

