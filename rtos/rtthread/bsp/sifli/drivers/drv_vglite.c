/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "drv_vglite.h"
#include "string.h"
#include "mem_section.h"
#include "math.h"

#define  DBG_LEVEL            DBG_LOG
#define  LOG_TAG                "drv.vglite"
#include "log.h"

#ifdef SOLUTION
    #include "app_mem.h"
    #define vglite_malloc(size)             app_cache_alloc(size, CACHE_PSRAM)
    #define vglite_free                     app_cache_free
#else
    #define vglite_malloc(size)             rt_malloc(size)
    #define vglite_free                     rt_free
#endif

#ifdef BSP_USING_LCD
    #define VG_WIDTH_DEFAULT   (LCD_HOR_RES_MAX)
    #define VG_HEIGHT_DEFAULT  (LCD_VER_RES_MAX)
#else
    #define VG_WIDTH_DEFAULT   (320)
    #define VG_HEIGHT_DEFAULT  (320)
#endif

#define VG_BUFFER_ALIGN    (64)
#define VG_LITE_OPEND      (1)

static uint8_t vglite_status = !VG_LITE_OPEND;

static char *vglite_error_type[] =
{
    "VG_LITE_SUCCESS",
    "VG_LITE_INVALID_ARGUMENT",
    "VG_LITE_OUT_OF_MEMORY",
    "VG_LITE_NO_CONTEXT",
    "VG_LITE_TIMEOUT",
    "VG_LITE_OUT_OF_RESOURCES",
    "VG_LITE_GENERIC_IO",
    "VG_LITE_NOT_SUPPORT",
};

#define VGLITE_ERROR_TYPE_NUM   (sizeof(vglite_error_type) / sizeof(vglite_error_type[0]))
#define VGLITE_MEM_SIZE   ((320) * 1024)

L1_NON_RET_BSS_SECT_BEGIN(drv_vglite_stack)

L1_NON_RET_BSS_SECT(drv_vglite_stack, ALIGN(64) static uint8_t drv_vglite_pool[VGLITE_MEM_SIZE]);

L1_NON_RET_BSS_SECT_END

#define VG_LITE_CMD_BUF_SIZE    (102<<10)


void vglite_print_error(const char *func, size_t line, vg_lite_error_t err)
{
    if (err < VGLITE_ERROR_TYPE_NUM)
    {
        LOG_I("[%s: %d] failed.error type: %s\n", func, line, vglite_error_type[err]);
    }
    else
    {
        LOG_I("[%s: %d] failed.unknown error type: %d\n", func, line, err);
    }
}

rt_err_t drv_vglite_open(void)
{
    rt_err_t err;

    if (VG_LITE_OPEND == vglite_status)
        return RT_ERROR;

    vg_module_parameters_t param = { 0 };

    param.register_mem_base = V2D_GPU_BASE;
    param.gpu_mem_base[0] = 0;
    param.contiguous_mem_base[0] = &drv_vglite_pool[0];
    param.contiguous_mem_size[0] = VGLITE_MEM_SIZE;
    vg_lite_init_mem(&param);

    RT_ASSERT(VG_LITE_SUCCESS == vg_lite_set_command_buffer_size(VG_LITE_CMD_BUF_SIZE));
    RT_ASSERT(VG_LITE_SUCCESS == vg_lite_init(RT_ALIGN(VG_WIDTH_DEFAULT, 16), RT_ALIGN(VG_HEIGHT_DEFAULT, 16)));

    vglite_status = VG_LITE_OPEND;

    LOG_I("%s,mem_size 0x%x, tess_w %d, tess_h %d, command_buffer 0x%x ", __func__, VGLITE_MEM_SIZE, VG_WIDTH_DEFAULT, VG_HEIGHT_DEFAULT, VG_LITE_CMD_BUF_SIZE);
    return RT_EOK;
}

rt_err_t drv_vglite_close(void)
{
    if ((!VG_LITE_OPEND) == vglite_status)
        return RT_ERROR;

    vg_lite_close();

    /* unnecessary */
#if 0
    vg_module_parameters_t param = { 0 };

    param.register_mem_base = V2D_GPU_BASE;
    param.gpu_mem_base[0] = 0;
    param.contiguous_mem_base[0] = 0;
    param.contiguous_mem_size[0] = 0;
    vg_lite_init_mem(&param);
#endif

    vglite_status = !VG_LITE_OPEND;

    LOG_I("%s.", __func__);

    return RT_EOK;
}


void *vglite_aligned_malloc(size_t *size)
{
    const size_t alignment = VG_BUFFER_ALIGN;
    const size_t offset = alignment - 1 + sizeof(void *);
    *size = *size + offset;
    void *raw_ptr = vglite_malloc(*size);
    if (raw_ptr == NULL)
    {
        return NULL;
    }
    void *aligned_ptr = (void *)(((uintptr_t)raw_ptr + sizeof(void *) + alignment - 1) & ~(alignment - 1));
    *(void **)((uintptr_t)aligned_ptr - sizeof(void *)) = raw_ptr;

    return aligned_ptr;
}

void vglite_aligned_free(void *aligned_ptr)
{
    if (aligned_ptr == NULL)
    {
        return;
    }
    void *raw_ptr = *(void **)((uintptr_t)aligned_ptr - sizeof(void *));
    vglite_free(raw_ptr);
}


/**
 * @brief Calculate the Euclidean norm (magnitude) of a 3D vector
 * @param v Pointer to the input vg_lite_vertex_t vector
 * @return Norm of the vector as a float
 */
float vg_lite_vec3_norm(const vg_lite_vertex_t *v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

/**
 * @brief Normalize a 3D vector to unit length (in-place modification)
 * @param v Pointer to the vector to normalize
 * @return 0 on success, -1 on failure (zero vector input)
 */
int vg_lite_vec3_normalize(vg_lite_vertex_t *v)
{
    float norm = vg_lite_vec3_norm(v);
    if (norm < 1e-6f)   // Avoid division by zero (tolerance for floating-point errors)
    {
        fprintf(stderr, "Error: Target Z-axis is a zero vector. Cannot construct coordinate system!\n");
        return -1;
    }
    v->x /= norm;
    v->y /= norm;
    v->z /= norm;
    return 0;
}

/**
 * @brief Compute the cross product of two 3D vectors (a × b)
 * @param a Pointer to the first input vector
 * @param b Pointer to the second input vector
 * @return Result of the cross product as a vg_lite_vertex_t vector
 */
vg_lite_vertex_t vg_lite_vec3_cross(const vg_lite_vertex_t *a, const vg_lite_vertex_t *b)
{
    vg_lite_vertex_t res;
    res.x = a->y * b->z - a->z * b->y;
    res.y = a->z * b->x - a->x * b->z;
    res.z = a->x * b->y - a->y * b->x;
    return res;
}

/**
 * @brief Print a 4x4 matrix (for debugging/verification purposes)
 * @param name Name of the matrix (displayed in output)
 * @param mat Pointer to the vg_lite_matrix_4x4_t matrix to print
 */
void vg_lite_mat4_print(const char *name, const vg_lite_matrix_4x4_t *mat)
{
    printf("%s:\n", name);
    for (int i = 0; i < 4; i++)
    {
        printf("%.6f  %.6f  %.6f  %.6f\n",
               mat->m[i][0], mat->m[i][1], mat->m[i][2], mat->m[i][3]);
    }
}

void vg_lite_multiply2_4x4(vg_lite_matrix_4x4_t *out, vg_lite_matrix_4x4_t *lt, vg_lite_matrix_4x4_t *rt)
{
    vg_lite_matrix_4x4_t temp;
    int row, column;

    /* Process all rows. */
    for (row = 0; row < 4; row++)
    {
        /* Process all columns. */
        for (column = 0; column < 4; column++)
        {
            /* Compute matrix entry. */
            temp.m[row][column] = (lt->m[row][0] * rt->m[0][column])
                                  + (lt->m[row][1] * rt->m[1][column])
                                  + (lt->m[row][2] * rt->m[2][column])
                                  + (lt->m[row][3] * rt->m[3][column]);
        }
    }

    /* Copy temporary matrix into result. */
    memcpy(out, &temp, sizeof(temp));
}

// -----------------------------------------------------------------------------
// Core Function: Reference Frame → Target Frame Transformation Matrix
// -----------------------------------------------------------------------------

/**
 * @brief Compute the 4x4 homogeneous transformation matrix from reference frame to target frame
 *        (combines rotation and translation for origin offset)
 *
 * @param target_z Input: Z-axis vector of the target coordinate system (no prior normalization needed)
 * @param origin_offset Input: Origin offset between frames (target origin's coordinates in the reference frame: dx, dy, dz)
 * @param transform_mat Output: 4x4 transformation matrix (reference → target)
 *
 * @return 0 on success, -1 on failure (NULL parameters or zero target Z-axis)
 */
int vg_lite_get_transform_matrix_4x4(
    const vg_lite_vertex_t *target_z,
    const vg_lite_vertex_t *origin_offset,
    vg_lite_matrix_4x4_t *transform_mat
)
{
    // Validate input parameters (prevent NULL pointer dereference)
    if (target_z == NULL || origin_offset == NULL || transform_mat == NULL)
    {
        fprintf(stderr, "Error: Input parameters cannot be NULL!\n");
        return -1;
    }

    // Step 1: Initialize matrix as identity matrix (avoids garbage values)
    memset(transform_mat, 0, sizeof(vg_lite_matrix_4x4_t));
    transform_mat->m[0][0] = 1.0f;
    transform_mat->m[1][1] = 1.0f;
    transform_mat->m[2][2] = 1.0f;
    transform_mat->m[3][3] = 1.0f; // Homogeneous component (fixed for affine transformations)

    // Step 2: Normalize the target Z-axis to unit length (tz)
    vg_lite_vertex_t tz = *target_z;
    if (vg_lite_vec3_normalize(&tz) != 0)
    {
        return -1; // Failure: zero vector input
    }

    // Step 3: Construct target X-axis (tx)
    // Constraints:
    // 1. Parallel to reference frame's ZX-plane (Y-component = 0)
    // 2. Orthogonal to target Z-axis (tz)
    vg_lite_vertex_t tx = { 1.0f, 0.0f, 0.0f };
    float tz_x = tz.x;  // X-component of normalized target Z-axis
    float tz_z = tz.z;  // Z-component of normalized target Z-axis
    float tx_candidate_norm = sqrtf(tz_z * tz_z + tz_x * tz_x); // Norm of (tz_z, 0, -tz_x)

    if (tx_candidate_norm > 1e-6f)
    {
        // General case: tx = (tz_z, 0, -tz_x) / tx_candidate_norm
        // Guarantees tx ⊥ tz and Y-component = 0 (parallel to ZX-plane)
        tx.x = tz_z / tx_candidate_norm;
        tx.y = 0.0f;
        tx.z = -tz_x / tx_candidate_norm;
    }

    // Step 4: Construct target Y-axis (ty) using right-hand rule
    // ty = tz × tx (automatically orthogonal to both tz and tx, and unit-length)
    vg_lite_vertex_t ty = vg_lite_vec3_cross(&tz, &tx);
    if (ty.y <= 1e-6f)
    {
        ty.x = -ty.x;
        ty.y = -ty.y;
        ty.z = -ty.z;
    }

    // Step 5: Populate rotation component (top-left 3x3 submatrix)
    // Columns of the matrix are the target axes (tx, ty, tz) in reference frame
    transform_mat->m[0][0] = tx.x;  // Column 0: tx (X-axis of target frame)
    transform_mat->m[1][0] = tx.y;
    transform_mat->m[2][0] = tx.z;

    transform_mat->m[0][1] = ty.x;  // Column 1: ty (Y-axis of target frame)
    transform_mat->m[1][1] = ty.y;
    transform_mat->m[2][1] = ty.z;

    transform_mat->m[0][2] = tz.x;  // Column 2: tz (Z-axis of target frame)
    transform_mat->m[1][2] = tz.y;
    transform_mat->m[2][2] = tz.z;

    // Step 6: Populate translation component (4th column, first 3 rows)
    // Represents target origin's coordinates in the reference frame
    transform_mat->m[0][3] = origin_offset->x;  // X-offset
    transform_mat->m[1][3] = origin_offset->y;  // Y-offset
    transform_mat->m[2][3] = origin_offset->z;  // Z-offset

    return 0;
}

// -----------------------------------------------------------------------------
// Auxiliary Function: Target Frame → Reference Frame Inverse Matrix
// -----------------------------------------------------------------------------

/**
 * @brief Compute the inverse transformation matrix (target frame → reference frame)
 *        Reverses the original transformation (first inverse translation, then inverse rotation)
 *
 * @param mat Input: Original transformation matrix (reference → target)
 * @param inv_mat Output: Inverse transformation matrix (target → reference)
 *
 * @return 0 on success, -1 on failure (NULL parameters)
 */
int vg_lite_inverse_matrix_4x4(
    const vg_lite_matrix_4x4_t *mat,
    vg_lite_matrix_4x4_t *inv_mat
)
{
    // Validate input parameters
    if (mat == NULL || inv_mat == NULL)
    {
        fprintf(stderr, "Error: Input parameters cannot be NULL!\n");
        return -1;
    }

    // Initialize inverse matrix to zero
    memset(inv_mat, 0, sizeof(vg_lite_matrix_4x4_t));
    inv_mat->m[3][3] = 1.0f; // Preserve homogeneous component

    // Step 1: Inverse rotation = transpose of original rotation matrix (orthogonal matrix property)
    const float rotation[3][3] =
    {
        {mat->m[0][0], mat->m[0][1], mat->m[0][2]},
        {mat->m[1][0], mat->m[1][1], mat->m[1][2]},
        {mat->m[2][0], mat->m[2][1], mat->m[2][2]}
    };
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            inv_mat->m[i][j] = rotation[j][i]; // Transpose operation
        }
    }

    // Step 2: Inverse translation = -inverse_rotation × original_translation
    const float translation[3] =
    {
        mat->m[0][3],
        mat->m[1][3],
        mat->m[2][3]
    };
    inv_mat->m[0][3] = - (inv_mat->m[0][0] * translation[0] +
                          inv_mat->m[0][1] * translation[1] +
                          inv_mat->m[0][2] * translation[2]);
    inv_mat->m[1][3] = - (inv_mat->m[1][0] * translation[0] +
                          inv_mat->m[1][1] * translation[1] +
                          inv_mat->m[1][2] * translation[2]);
    inv_mat->m[2][3] = - (inv_mat->m[2][0] * translation[0] +
                          inv_mat->m[2][1] * translation[1] +
                          inv_mat->m[2][2] * translation[2]);

    return 0;
}

vg_lite_error_t vg_lite_perspective3_3x3(float fov, float canvas_w, float canvas_h, vg_lite_quad_t quad, vg_lite_matrix_t *matrix)
{
    vg_lite_vertex_t normal;
    vg_lite_matrix_t affine_mat;
    /* line equation: x = m*y+n */
    float m, n;
    vg_lite_error_t err = VG_LITE_SUCCESS;
    vg_lite_float_t scale = vg_lite_get_perspective_scaling_factor(fov);

    vg_lite_vertex_t *v0 = &quad[0];
    vg_lite_vertex_t *v1 = &quad[1];
    vg_lite_vertex_t *v2 = &quad[2];
    vg_lite_vertex_t *v3 = &quad[3];

    vg_lite_get_normal(v0, v1, v2, &normal);

    if (!VG_LITE_IS_EQUAL_TO_ZERO(normal.z))
    {
        err = vg_lite_perspective2_3x3(scale, canvas_w, canvas_h, v0, v1, v2, v3, matrix);
        if (VG_LITE_SUCCESS != err)
        {
            goto __EXIT;
        }
    }
    else if (VG_LITE_IS_EQUAL_TO_ZERO(normal.y))
    {
        matrix->m[0][0] = 0;
        matrix->m[0][1] = -canvas_w / canvas_h;
        matrix->m[0][2] = v0->x * scale;
        matrix->m[1][0] = -scale;
        matrix->m[1][1] = -1;
        matrix->m[1][2] = 0;
        matrix->m[2][0] = 0;
        matrix->m[2][1] = -2.0f / canvas_h;
        matrix->m[2][2] = 0;
    }
    else if (VG_LITE_IS_EQUAL_TO_ZERO(normal.x))
    {

        matrix->m[0][0] = scale;
        matrix->m[0][1] = -canvas_w / canvas_h;
        matrix->m[0][2] = 0;
        matrix->m[1][0] = 0;
        matrix->m[1][1] = -1;
        matrix->m[1][2] = -v0->y * scale;
        matrix->m[2][0] = 0;
        matrix->m[2][1] = -2.0f / canvas_h;
        matrix->m[2][2] = 0;
    }
    else
    {
        m = -normal.y / normal.x;
        n = (normal.x * v0->x + normal.y * v0->y) / normal.x;
        matrix->m[0][0] = m * scale;
        matrix->m[0][1] = -canvas_w / canvas_h;
        matrix->m[0][2] = n * scale;
        matrix->m[1][0] = -scale;
        matrix->m[1][1] = -1;
        matrix->m[1][2] = 0;
        matrix->m[2][0] = 0;
        matrix->m[2][1] = -2.0f / canvas_h;
        matrix->m[2][2] = 0;
    }
__EXIT:
    return err;
}

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
void vg_lite_rodrigues_matrix_4x4(vg_lite_matrix_4x4_t *matrix, float ux, float uy, float uz, float theta_deg)
{
    // Step 1: Initialize matrix to 4x4 identity (base for homogeneous transformation)
    vg_lite_identity_4x4(matrix);

    // Step 2: Convert rotation angle from degrees to radians
    float theta = theta_deg * M_PI / 180.0f;

    // Step 3: Normalize the rotation axis vector (ux, uy, uz) to unit length
    float axis_norm = sqrtf(ux * ux + uy * uy + uz * uz);
    // Guard against zero vector (avoid division by zero)
    if (axis_norm < 1e-6f)
    {
        // Keep rotation submatrix as identity (no rotation)
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                matrix->m[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        return;
    }
    // Normalize axis components
    ux /= axis_norm;
    uy /= axis_norm;
    uz /= axis_norm;

    // Step 4: Precompute trigonometric terms for Rodrigues' formula
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    float one_minus_cos = 1.0f - cos_theta;

    // Step 5: Skew-symmetric cross product matrix [u]× of the unit axis vector u
    float skew_symm[3][3] =
    {
        {0.0f, -uz,    uy   },
        {uz,    0.0f, -ux   },
        {-uy,   ux,    0.0f }
    };

    // Step 6: Square of the skew-symmetric matrix [u]×²
    float skew_symm_sq[3][3];
    skew_symm_sq[0][0] = -(uy * uy + uz * uz);
    skew_symm_sq[0][1] = ux * uy;
    skew_symm_sq[0][2] = ux * uz;

    skew_symm_sq[1][0] = ux * uy;
    skew_symm_sq[1][1] = -(ux * ux + uz * uz);
    skew_symm_sq[1][2] = uy * uz;

    skew_symm_sq[2][0] = ux * uz;
    skew_symm_sq[2][1] = uy * uz;
    skew_symm_sq[2][2] = -(ux * ux + uy * uy);

    // Step 7: Apply Rodrigues' formula to compute 3x3 rotation submatrix
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            matrix->m[i][j] = (i == j ? 1.0f : 0.0f)  // Identity matrix term
                              + sin_theta * skew_symm[i][j]  // Sine term (rotation)
                              + one_minus_cos * skew_symm_sq[i][j];  // Cosine correction term
        }
    }

    // Step 8: Homogeneous components remain unchanged (last row/column = [0,0,0,1])
    // Already handled by vg_lite_identity_4x4()
}

static void vg_lite_draw_img_texture_ext(float fov, vg_lite_buffer_t *dst, vg_lite_buffer_t *src, vg_lite_quad_t quad_proj)
{
    vg_lite_error_t err = VG_LITE_SUCCESS;

    vg_lite_matrix_t mat_proj;
    float item_w = src->width;
    float item_h = src->height;
    err = vg_lite_get_texture_map_matrix(fov, item_w, item_h, dst->width, dst->height, quad_proj, &mat_proj);
    if (err != VG_LITE_SUCCESS)
    {
        LOG_I("Get texture matrix error.\n");
        return;
    }
    err = vg_lite_blit(dst, src, &mat_proj, VG_LITE_BLEND_SRC_OVER, 0, VG_LITE_FILTER_BI_LINEAR);
    if (err != VG_LITE_SUCCESS)
    {
        LOG_I("blit img error:%d.\n", err);
        return;
    }
    err = vg_lite_finish();
    if (err != VG_LITE_SUCCESS)
    {
        LOG_I("wait blit img error:%d.\n", err);
        return;
    }
}

void vg_lite_draw_img_texture(float fov, vg_lite_buffer_t *dst, vg_lite_buffer_t *src, vg_lite_matrix_4x4_t *rot, void *sram_buf, uint32_t buf_size)
{
    vg_lite_error_t err = VG_LITE_SUCCESS;
    float ref_h = dst->height;
    float src_w = src->width;
    float src_h = src->height;
    float w_f = src->width / ref_h;
    float h_f = src->height / ref_h;

    //buf_size = buf_size >> 1;
    int32_t line_size = src->stride;
    int32_t line_max = buf_size / line_size;

    vg_lite_quad_t v = { { -w_f, h_f, 0}, { -w_f, -h_f, 0}, { w_f, -h_f, 0}, { w_f, h_f, 0} };

    for (uint32_t j = 0; j < 4; j++)
    {
        vg_lite_mat_vertex_multiply2_4x4(rot, &v[j].x, &v[j].y, &v[j].z);
    }

    float sum = 0.0f;
    sum += v[0].x * v[1].y - v[1].x * v[0].y;
    sum += v[1].x * v[2].y - v[2].x * v[1].y;
    sum += v[2].x * v[3].y - v[3].x * v[2].y;
    sum += v[3].x * v[0].y - v[0].x * v[3].y;
    float rate = sum / w_f / h_f / 8.0f;

    if (sram_buf && buf_size > line_size && (HAL_ABS(rate) > 0.5f || (line_size * src_h < buf_size)))
    {
        //rt_kprintf("sram sum:%f rate:%f\n",sum,rate);
        for (int32_t ofs_y = 0; ofs_y < src_h; ofs_y += line_max)
        {
            int32_t next_y = HAL_MIN(src_h, ofs_y + line_max);
            float y0 = h_f - ofs_y * h_f * 2.0f / src_h;
            float y1 = h_f - next_y * h_f * 2.0f / src_h;
            vg_lite_quad_t v = { { -w_f, y0, 0}, { -w_f, y1, 0}, { w_f, y1, 0}, { w_f, y0, 0} };
            float splitpart_h = next_y - ofs_y;
            uint32_t data_size = splitpart_h * line_size;
            uint32_t ofs_size = (ofs_y + 1) * line_size;
            uint8_t *p_buf = (uint8_t *)((uint32_t)src->memory + ofs_size);
            rt_memcpy(sram_buf, p_buf, data_size);

            vg_lite_buffer_t tmp_fb = { 0 };
            err = vg_lite_init_buf(&tmp_fb, src_w, splitpart_h, src->format, sram_buf);
            if (err != VG_LITE_SUCCESS)
            {
                LOG_I("splitpart init error.\n");
                return;
            }

            for (uint32_t j = 0; j < 4; j++)
            {
                vg_lite_mat_vertex_multiply2_4x4(rot, &v[j].x, &v[j].y, &v[j].z);
            }
            vg_lite_draw_img_texture_ext(fov, dst, &tmp_fb, v);
        }
    }
    else
    {
        //rt_kprintf("psram sum:%f rate:%f\n",sum,rate);
        vg_lite_draw_img_texture_ext(fov, dst, src, v);
    }

}

