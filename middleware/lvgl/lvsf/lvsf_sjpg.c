/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lv_sjpg.c
 *
 */

/*----------------------------------------------------------------------------------------------------------------------------------
/    Added normal JPG support [7/10/2020]
/    ----------
/    SJPEG is a custom created modified JPEG file format for small embedded platforms.
/    It will contain multiple JPEG fragments all embedded into a single file with a custom header.
/    This makes JPEG decoding easier using any JPEG library. Overall file size will be almost
/    similar to the parent jpeg file. We can generate sjpeg from any jpeg using a python script
/    provided along with this project.
/                                                                                     (by vinodstanur | 2020 )
/    SJPEG FILE STRUCTURE
/    --------------------------------------------------------------------------------------------------------------------------------
/    Bytes                       |   Value                                                                                           |
/    --------------------------------------------------------------------------------------------------------------------------------
/
/    0 - 7                       |   "_SJPG__" followed by '\0'
/
/    8 - 13                      |   "V1.00" followed by '\0'       [VERSION OF SJPG FILE for future compatibiliby]
/
/    14 - 15                     |   X_RESOLUTION (width)            [little endian]
/
/    16 - 17                     |   Y_RESOLUTION (height)           [little endian]
/
/    18 - 19                     |   TOTAL_FRAMES inside sjpeg       [little endian]
/
/    20 - 21                     |   JPEG BLOCK WIDTH (16 normally)  [little endian]
/
/    22 - [(TOTAL_FRAMES*2 )]    |   SIZE OF EACH JPEG SPLIT FRAGMENTS   (FRAME_INFO_ARRAY)
/
/   SJPEG data                   |   Each JPEG frame can be extracted from SJPEG data by parsing the FRAME_INFO_ARRAY one time.
/
/----------------------------------------------------------------------------------------------------------------------------------
/                   JPEG DECODER
/                   ------------
/   We are using TJpgDec - Tiny JPEG Decompressor library from ELM-CHAN for decoding each split-jpeg fragments.
/   The tjpgd.c and tjpgd.h is not modified and those are used as it is. So if any update comes for the tiny-jpeg,
/   just replace those files with updated files.
/---------------------------------------------------------------------------------------------------------------------------------*/

/*********************
 *      INCLUDES
 *********************/

#include "lvgl.h"
#include "lvsf_conf_internal.h"

#if LVSF_USING_SJPG

#include <dfs_posix.h>
#include <rtthread.h>
#include <string.h>

#include "app_mem.h"
#include "lvsf_sjpg.h"
#include "tjpgd.h"
#include "lvsf_sjpg_algo.h"

#ifndef LV_IMG_CF_JPG
#define LV_IMG_CF_JPG LV_IMG_CF_RAW
#endif

static void *sjpg_alloc_wrap(size_t size)
{
    return app_cache_alloc(size, CACHE_PSRAM);
}

static void sjpg_free_wrap(void *ptr)
{
    app_cache_free(ptr);
}

static void *sjpg_calloc_wrap(size_t count, size_t size)
{
    return app_cache_calloc(count, size, CACHE_PSRAM);
}

#define sjpg_alloc(size)            sjpg_alloc_wrap(size)
#define sjpg_calloc(count, size)    sjpg_calloc_wrap(count, size)
#define sjpg_free(data)             sjpg_free_wrap(data)
#define sjpg_frame_alloc(reuse_anim_buf, size) \
    ((reuse_anim_buf) ? app_anim_alloc(size) : sjpg_alloc(size))
#define sjpg_frame_free(data)       app_anim_free(data)

#ifdef USING_JPEG_DEC
    #include "jpeg_dec.h"
#endif /* USING_JPEG_DEC */

/**
 * Return with the extension of the filename
 * @param fn string with a filename
 * @return pointer to the beginning extension or empty string if no extension
 */
static const char *file_get_ext(const char *fn)
{
    size_t i;
    for (i = strlen(fn); i > 0; i--)
    {
        if (fn[i] == '.')
        {
            return &fn[i + 1];
        }
        else if (fn[i] == '/' || fn[i] == '\\')
        {
            return ""; /*No extension if a '\' or '/' found*/
        }
    }

    return ""; /*Empty string if no '.' in the file name. */
}

/*********************
 *      DEFINES
 *********************/
#if defined(_MSC_VER)
    #define TJPGD_WORKBUFF_SIZE         65536
#elif (3 == JD_FORMAT)
    #define TJPGD_WORKBUFF_SIZE         32768
#else
    #define TJPGD_WORKBUFF_SIZE         24576
#endif

//NEVER EDIT THESE OFFSET VALUES
#define SJPEG_VERSION_OFFSET            8
#define SJPEG_X_RES_OFFSET              14
#define SJPEG_y_RES_OFFSET              16
#define SJPEG_TOTAL_FRAMES_OFFSET       18
#define SJPEG_BLOCK_WIDTH_OFFSET        20
#define SJPEG_FRAME_INFO_ARRAY_OFFSET   22


#if (3 == JD_FORMAT)
    #define SJEPG_OUTPUT_EPIC_FORMAT  LV_IMG_CF_YUV420_PLANAR
#else
    #define SJEPG_OUTPUT_EPIC_FORMAT  LV_IMG_CF_TRUE_COLOR
#endif

/**********************
 *      TYPEDEFS
 **********************/

enum io_source_type
{
    SJPEG_IO_SOURCE_C_ARRAY,
    SJPEG_IO_SOURCE_DISK,
};

typedef struct
{
    lvsf_sjpg_io_source_t algo;
    enum io_source_type type;
    int lv_file;
    uint8_t *raw_sjpg_data;               //Used when type==SJPEG_IO_SOURCE_C_ARRAY.
    uint32_t raw_sjpg_data_size;          //Num bytes pointed to by raw_sjpg_data.
    uint32_t raw_sjpg_data_next_read_pos; //Used for all types.
} io_source_t;


typedef struct
{
    uint8_t *sjpeg_data;
    uint32_t sjpeg_data_size;
    int sjpeg_x_res;
    int sjpeg_y_res;
    int sjpeg_total_frames;
    int sjpeg_single_frame_height;
    int sjpeg_cache_frame_index;
    uint8_t **frame_base_array;         //to save base address of each split frames upto sjpeg_total_frames.
    int *frame_base_offset;             //to save base offset for fseek
    uint8_t *frame_cache;
    uint8_t *workb;                     //JPG work buffer for jpeg library
    JDEC *tjpeg_jd;
    io_source_t io;
} SJPEG;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static lv_res_t decoder_info(lv_img_decoder_t *decoder, const void *src, lv_img_header_t *header);
static lv_res_t decoder_open(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc);
static lv_res_t decoder_read_line(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc, lv_coord_t x, lv_coord_t y,
                                  lv_coord_t len, uint8_t *buf);
static void decoder_close(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc);
static size_t input_func(JDEC *jd, uint8_t *buff, size_t ndata);
static int is_jpg(const uint8_t *raw_data, size_t len);
static void lv_sjpg_cleanup(SJPEG *sjpeg);
static void lv_sjpg_free(SJPEG *sjpeg);
static void lv_sjpg_free_ctrl(SJPEG *sjpeg);
static int get_sjpg_size_from_file(int fd, uint32_t offset, uint16_t *width, uint16_t *height);
static int get_sjpg_size_from_data(const uint8_t *jpeg_data, size_t data_len,
                                   uint16_t *width, uint16_t *height);

/**********************
 *  STATIC VARIABLES
 **********************/
static const lvsf_sjpg_algo_ops_t *g_algo_ops = NULL;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
int lvsf_split_jpeg_init(void)
{
    lv_img_decoder_t *dec = lv_img_decoder_create();
    lv_img_decoder_set_info_cb(dec, decoder_info);
    lv_img_decoder_set_open_cb(dec, decoder_open);
    lv_img_decoder_set_close_cb(dec, decoder_close);
    lv_img_decoder_set_read_line_cb(dec, decoder_read_line);

    g_algo_ops = lvsf_sjpg_algo_get_ops();

    return 0;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * Get info about an SJPG / JPG image
 * @param decoder pointer to the decoder where this function belongs
 * @param src can be file name or pointer to a C array
 * @param header store the info here
 * @return LV_RES_OK: no error; LV_RES_INV: can't get the info
 */
static lv_res_t decoder_info(lv_img_decoder_t *decoder, const void *src, lv_img_header_t *header)
{
    LV_UNUSED(decoder);

    /*Check whether the type `src` is known by the decoder*/
    /* Read the SJPG/JPG header and find `width` and `height` */

    lv_img_src_t src_type = lv_img_src_get_type(src);          /*Get the source type*/


    lv_res_t ret = LV_RES_OK;

    if (src_type == LV_IMG_SRC_VARIABLE)
    {
        lv_img_dsc_t *dsc = (lv_img_dsc_t *)src;
        uint16_t w;
        uint16_t h;

        if (LV_IMG_CF_JPG != dsc->header.cf)  return LV_RES_INV;

        if (0 == dsc->header.w || 0 == dsc->header.h)
        {
            if (0 != get_sjpg_size_from_data((uint8_t *)dsc->data, dsc->data_size, &w, &h))
            {
                return LV_RES_INV;
            }
            dsc->header.w = w;
            dsc->header.h = h;
        }
        else
        {
            w = dsc->header.w;
            h = dsc->header.h;
        }

        {
            int out_w;
            int out_h;
            header->cf = SJEPG_OUTPUT_EPIC_FORMAT;
#if (3 == JD_FORMAT)
            w = JD_ALIGN(w, 2);
            h = JD_ALIGN(h, 2);
#endif
            header->w = w;
            header->h = h;
#if (3 == JD_FORMAT)
            if (!g_algo_ops->calc_decode_size_yuv((int)header->w, (int)header->h, &out_w, &out_h)) return LV_RES_INV;
#else
            if (!g_algo_ops->calc_decode_size_rgb((int)header->w, (int)header->h, &out_w, &out_h)) return LV_RES_INV;
#endif
            header->w = (lv_coord_t)out_w;
            header->h = (lv_coord_t)out_h;
            return ret;
        }
    }
    else if (src_type == LV_IMG_SRC_FILE)
    {
        const char *fn = src;
        const char *ext = file_get_ext(fn);

        if (strcmp(ext, "bin") == 0)
        {
            int file = open(fn, 0);
            if (file < 0) return LV_RES_INV;

            /* judge __seq case, get length and offset. */
            uint32_t length = 0;
            uint32_t offset = lv_img_decoder_get_wf_offset(fn, &length);
            if (0 < offset)
            {
                lseek(file, offset, SEEK_SET);
            }
            int size = read(file, header, sizeof(*header));
            close(file);

            if (size < sizeof(*header) || LV_IMG_CF_JPG != header->cf)
            {
                return LV_RES_INV;
            }
            {
                int out_w;
                int out_h;
                header->cf = SJEPG_OUTPUT_EPIC_FORMAT;
#if (3 == JD_FORMAT)
                header->w = JD_ALIGN(header->w, 2);
                header->h = JD_ALIGN(header->h, 2);
                if (!g_algo_ops->calc_decode_size_yuv((int)header->w, (int)header->h, &out_w, &out_h)) return LV_RES_INV;
#else
                if (!g_algo_ops->calc_decode_size_rgb((int)header->w, (int)header->h, &out_w, &out_h)) return LV_RES_INV;
#endif
                header->w = (lv_coord_t)out_w;
                header->h = (lv_coord_t)out_h;
                //rt_kprintf("%s: %d %d\n", __func__, header->w, header->h);
                return LV_RES_OK;
            }
        }

        if (strcmp(ext, "sjpg") == 0)
        {
            int file = open(fn, 0);
            if (file < 0) return LV_RES_INV;
            uint8_t buff[22];
            memset(buff, 0, sizeof(buff));

            uint32_t rn;
            rn = read(file, buff, 8);
            if (rn < 8)
            {
                close(file);
                return LV_RES_INV;
            }

            if (strcmp((char *)buff, "_SJPG__") == 0)
            {
                lseek(file, 14, SEEK_SET);
                rn = read(file, buff, 4);
                close(file);
                if (rn < 4)
                {
                    return LV_RES_INV;
                }
                header->always_zero = 0;
                header->cf = SJEPG_OUTPUT_EPIC_FORMAT;
                uint8_t *raw_sjpeg_data = buff;
                header->w = *raw_sjpeg_data++;
                header->w |= *raw_sjpeg_data++ << 8;
                header->h = *raw_sjpeg_data++;
                {
                    int out_w;
                    int out_h;
                    header->h |= *raw_sjpeg_data++ << 8;
#if (3 == JD_FORMAT)
                    if (!g_algo_ops->calc_decode_size_yuv((int)header->w, (int)header->h, &out_w, &out_h)) return LV_RES_INV;
#else
                    if (!g_algo_ops->calc_decode_size_rgb((int)header->w, (int)header->h, &out_w, &out_h)) return LV_RES_INV;
#endif
                    header->w = (lv_coord_t)out_w;
                    header->h = (lv_coord_t)out_h;
                    return LV_RES_OK;
                }
            }
            close(file);
        }
        else if (strcmp(ext, "jpg") == 0 || strcmp(ext, "jpeg") == 0)
        {
            int file = open(fn, 0);
            uint16_t w;
            uint16_t h;
            if (file < 0) return LV_RES_INV;

            if (0 == get_sjpg_size_from_file(file, 0, &w, &h))
            {
                int out_w;
                int out_h;
                close(file);
                header->always_zero = 0;
                header->cf = SJEPG_OUTPUT_EPIC_FORMAT;
                header->w = w;
                header->h = h;
#if (3 == JD_FORMAT)
                header->w = JD_ALIGN(header->w, 2);
                header->h = JD_ALIGN(header->h, 2);
                if (!g_algo_ops->calc_decode_size_yuv((int)header->w, (int)header->h, &out_w, &out_h)) return LV_RES_INV;
#else
                if (!g_algo_ops->calc_decode_size_rgb((int)header->w, (int)header->h, &out_w, &out_h)) return LV_RES_INV;
#endif
                header->w = (lv_coord_t)out_w;
                header->h = (lv_coord_t)out_h;
                return LV_RES_OK;
            }
            close(file);
        }
    }
    return LV_RES_INV;
}

#if (3 == JD_FORMAT)
#if (JD_FORMAT == 3)
#define sjpg_sat_u8_from_yuv(sample)   ((uint8_t)(sample))
#else
static inline uint8_t sjpg_sat_u8_from_yuv(jd_yuv_t sample)
{
#if defined(__USAT)
    return (uint8_t)__USAT((int32_t)sample, 8);
#else
    if (sample < 0) return 0;
    if (sample > 255) return 255;
    return (uint8_t)sample;
#endif
}
#endif

static uint32_t single_frame_size(const SJPEG *sjpeg)
{
    uint32_t cache_size = 0;

    if (!sjpeg) return 0;
    if (!g_algo_ops->calc_cache_size_yuv(sjpeg->sjpeg_x_res, sjpeg->sjpeg_single_frame_height, &cache_size)) return 0;
    return cache_size;
}

//Convert YUV444/422/420 to YUV420 planar (and optionally downscale in-place to target cache size)
static int img_data_cb(JDEC *jd, void *data, JRECT *rect)
{
    io_source_t *io = jd->device;
    uint8_t *cache = io->algo.img_cache_buff;

    if (!io || !cache || !data || !rect) return 0;

    {
        const int src_width = io->algo.img_src_x_res > 0 ? io->algo.img_src_x_res : jd->width;
        const int src_height = io->algo.img_src_y_res > 0 ? io->algo.img_src_y_res : jd->height;
        const int dst_width = io->algo.img_cache_x_res;
        const int dst_height = io->algo.img_cache_y_res;
        const int dst_stride = JD_ALIGN(dst_width, 2);
        const int dst_h_aligned = JD_ALIGN(dst_height, 2);
        const uint32_t y_plane_size = (uint32_t)dst_stride * (uint32_t)dst_h_aligned;
        const uint32_t uv_plane_size = y_plane_size >> 2;
        const uint32_t cache_size = y_plane_size + uv_plane_size + uv_plane_size;

        if (cache_size < y_plane_size || cache_size < uv_plane_size) return 0;

        const unsigned int mx = jd->msx * 8U;
        const unsigned int my = jd->msy * 8U;
        const int block_cols = (int)(mx >> 3);
        const jd_yuv_t *src_data = (const jd_yuv_t *)data;

        int valid_w;
        int valid_h;

        uint8_t *y_plane;
        uint8_t *u_plane;
        uint8_t *v_plane;

        if (src_width <= 0 || src_height <= 0 || dst_width <= 0 || dst_height <= 0) return 0;

        valid_w = src_width - (int)rect->left;
        valid_h = src_height - (int)rect->top;
        if (valid_w > (int)mx) valid_w = (int)mx;
        if (valid_h > (int)my) valid_h = (int)my;
        if (valid_w <= 0 || valid_h <= 0) return 1;

        y_plane = cache;
        u_plane = y_plane + y_plane_size;
        v_plane = u_plane + uv_plane_size;

        const int dst_uv_width = dst_stride >> 1;
        const int dst_uv_height = dst_h_aligned >> 1;
        const int src_uv_width = (src_width + 1) >> 1;
        const int src_uv_height = (src_height + 1) >> 1;

        if (jd->msx == 2U && jd->msy == 2U)
        {
            lvsf_sjpg_rect_t algo_rect = {
                .left = rect->left, .right = rect->right,
                .top = rect->top, .bottom = rect->bottom,
            };
            const uint8_t *src_bytes = (const uint8_t *)src_data;

            if (src_width == dst_width && src_height == dst_height)
            {
                return g_algo_ops->fast_copy_420_no_scale(&io->algo, src_bytes, &algo_rect, valid_w, valid_h);
            }

            if (g_algo_ops->is_near_half_scale_yuv(src_width, src_height, dst_width, dst_height))
            {
                return g_algo_ops->fast_copy_420_half_scale(&io->algo, src_bytes, &algo_rect, valid_w, valid_h);
            }

            if (io->algo.y_row_map && io->algo.y_col_map && io->algo.uv_row_map && io->algo.uv_col_map)
            {
                return g_algo_ops->fast_copy_420_mapped(&io->algo, src_bytes, &algo_rect, valid_w, valid_h);
            }
        }

        /* Y plane nearest-neighbor mapping */
        for (int ly = 0; ly < valid_h; ly++)
        {
            int sy = (int)rect->top + ly;
            int dy;
            int by;

            if (!io->algo.y_row_map || (uint32_t)sy >= (uint32_t)src_height) continue;
            dy = io->algo.y_row_map[sy];
            if (dy < 0 || dy >= dst_height) continue;

            by = ly >> 3;
            for (int lx = 0; lx < valid_w; lx++)
            {
                int sx = (int)rect->left + lx;
                int dx;
                int bx;
                int block_idx;
                const jd_yuv_t *src_y;

                if (!io->algo.y_col_map || (uint32_t)sx >= (uint32_t)src_width) continue;
                dx = io->algo.y_col_map[sx];
                if (dx < 0 || dx >= dst_width) continue;

                bx = lx >> 3;
                block_idx = (by * block_cols + bx) * 64 + ((ly & 7) << 3) + (lx & 7);
                src_y = src_data + block_idx;

                {
                    uint32_t y_offset = (uint32_t)dy * (uint32_t)dst_stride + (uint32_t)dx;
                    if (y_offset >= y_plane_size || y_offset >= cache_size) continue;
                    y_plane[y_offset] = sjpg_sat_u8_from_yuv(*src_y);
                }
            }
        }

        /* UV plane nearest-neighbor mapping (output is YUV420 planar) */
        {
            const int rect_uv_left = (int)rect->left >> 1;
            const int rect_uv_top = (int)rect->top >> 1;
            const int src_cx_mul = (jd->msx == 1U) ? 2 : 1;
            const int src_cy_mul = (jd->msy == 1U) ? 2 : 1;
            const jd_yuv_t *src_cb = (const jd_yuv_t *)data + (jd->msx * jd->msy) * 64U;
            const jd_yuv_t *src_cr = src_cb + 64;
            int uv_rows = (valid_h + 1) >> 1;
            int uv_cols = (valid_w + 1) >> 1;

            if (rect_uv_top + uv_rows > src_uv_height) uv_rows = src_uv_height - rect_uv_top;
            if (rect_uv_left + uv_cols > src_uv_width) uv_cols = src_uv_width - rect_uv_left;
            if (uv_rows < 0) uv_rows = 0;
            if (uv_cols < 0) uv_cols = 0;

            for (int uvy = 0; uvy < uv_rows; uvy++)
            {
                int src_uv_y = rect_uv_top + uvy;
                int src_cy = uvy * src_cy_mul;
                int duvy;

                if (src_uv_height <= 0 || dst_uv_height <= 0) continue;
                if (src_cy < 0 || src_cy >= 8) continue;
                if (!io->algo.uv_row_map || (uint32_t)src_uv_y >= (uint32_t)src_uv_height) continue;

                duvy = io->algo.uv_row_map[src_uv_y];
                if (duvy < 0 || duvy >= dst_uv_height) continue;

                for (int uvx = 0; uvx < uv_cols; uvx++)
                {
                    int src_uv_x = rect_uv_left + uvx;
                    int src_cx = uvx * src_cx_mul;
                    int duvx;
                    int src_uv_idx;

                    if (src_uv_width <= 0 || dst_uv_width <= 0) continue;
                    if (src_cx < 0 || src_cx >= 8) continue;
                    if (!io->algo.uv_col_map || (uint32_t)src_uv_x >= (uint32_t)src_uv_width) continue;

                    duvx = io->algo.uv_col_map[src_uv_x];
                    if (duvx < 0 || duvx >= dst_uv_width) continue;

                    src_uv_idx = src_cy * 8 + src_cx;
                    {
                        uint32_t uv_offset = (uint32_t)duvy * (uint32_t)dst_uv_width + (uint32_t)duvx;
                        if (uv_offset >= uv_plane_size) continue;
                        u_plane[uv_offset] = sjpg_sat_u8_from_yuv(src_cb[src_uv_idx]);
                        v_plane[uv_offset] = sjpg_sat_u8_from_yuv(src_cr[src_uv_idx]);
                    }
                }
            }
        }

        return 1;
    }
}

#else
static uint32_t single_frame_size(const SJPEG *sjpeg)
{
    uint32_t cache_size = 0;

    if (!sjpeg) return 0;
    if (!g_algo_ops->calc_cache_size_rgb(sjpeg->sjpeg_x_res, sjpeg->sjpeg_single_frame_height, &cache_size)) return 0;
    return cache_size;
}

// RGB888 tile -> LVGL color buffer with decode-time nearest-neighbor downscale
static int img_data_cb(JDEC *jd, void *data, JRECT *rect)
{
    io_source_t *io = jd->device;
    uint8_t *cache = io->algo.img_cache_buff;
    const int INPUT_PIXEL_SIZE = LV_COLOR_DEPTH >> 3;
    const int SOURCE_PIXEL_SIZE = 3;
    const int src_width = io->algo.img_src_x_res > 0 ? io->algo.img_src_x_res : jd->width;
    const int src_height = io->algo.img_src_y_res > 0 ? io->algo.img_src_y_res : jd->height;
    const int dst_width = io->algo.img_cache_x_res;
    const int dst_height = io->algo.img_cache_y_res;
    const int use_scale = (src_width != dst_width) || (src_height != dst_height);

    const int src_tile_w = (int)rect->right - (int)rect->left + 1;
    const int src_tile_h = (int)rect->bottom - (int)rect->top + 1;

    if (!io || !cache || !data || !rect) return 0;
    if (src_width <= 0 || src_height <= 0 || dst_width <= 0 || dst_height <= 0) return 0;
    if (src_tile_w <= 0 || src_tile_h <= 0) return 0;

    {
        lvsf_sjpg_rect_t algo_rect = {
            .left = rect->left, .right = rect->right,
            .top = rect->top, .bottom = rect->bottom,
        };
        return g_algo_ops->process_rgb(&io->algo, data, &algo_rect, src_tile_w, src_tile_h);
    }
}

#endif

static size_t input_func(JDEC *jd, uint8_t *buff, size_t ndata)
{
    io_source_t *io = jd->device;

    if (!io) return 0;

    if (io->type == SJPEG_IO_SOURCE_C_ARRAY)
    {
        if (io->raw_sjpg_data_next_read_pos > io->raw_sjpg_data_size)
        {
            rt_kprintf("[sjpg][c_array] invalid read pos: next=%u size=%u ndata=%u\n",
                       io->raw_sjpg_data_next_read_pos,
                       io->raw_sjpg_data_size,
                       (uint32_t)ndata);
            return 0;
        }

        const uint32_t bytes_left = io->raw_sjpg_data_size - io->raw_sjpg_data_next_read_pos;
        const uint32_t to_read = ndata <= bytes_left ? (uint32_t)ndata : bytes_left;
        if (to_read == 0)
            return 0;
        if (buff)
        {
            memcpy(buff, io->raw_sjpg_data + io->raw_sjpg_data_next_read_pos, to_read);
        }
        io->raw_sjpg_data_next_read_pos += to_read;
        return to_read;
    }
    else if (io->type == SJPEG_IO_SOURCE_DISK)
    {

        int lv_file_p = io->lv_file;

        if (buff)
        {
            uint32_t rn = read(lv_file_p, buff, (uint32_t)ndata);
            return rn;
        }
        else
        {
            lseek(lv_file_p, ndata,  SEEK_CUR);
            return ndata;
        }
    }
    return 0;
}

static JRESULT jd_decode_frame(lv_img_decoder_dsc_t *dsc, int sjpeg_req_frame_index)
{
    uint32_t start = lv_tick_get();
    if (dsc->src_type == LV_IMG_SRC_VARIABLE)
    {
        SJPEG *sjpeg = (SJPEG *) dsc->user_data;
        JRESULT rc;
        //int sjpeg_req_frame_index = y / sjpeg->sjpeg_single_frame_height;
        /*If line not from cache, refresh cache */
        if (sjpeg_req_frame_index != sjpeg->sjpeg_cache_frame_index)
        {
            sjpeg->io.raw_sjpg_data = sjpeg->frame_base_array[ sjpeg_req_frame_index ];
            if (sjpeg_req_frame_index == (sjpeg->sjpeg_total_frames - 1))
            {
                /*This is the last frame. */
                const uint32_t frame_offset = (uint32_t)(sjpeg->io.raw_sjpg_data - sjpeg->sjpeg_data);
                sjpeg->io.raw_sjpg_data_size = sjpeg->sjpeg_data_size - frame_offset;
            }
            else
            {
                sjpeg->io.raw_sjpg_data_size =
                    (uint32_t)(sjpeg->frame_base_array[sjpeg_req_frame_index + 1] - sjpeg->io.raw_sjpg_data);
            }
            sjpeg->io.raw_sjpg_data_next_read_pos = 0;
            rc = jd_prepare_ex(sjpeg->tjpeg_jd, input_func, sjpeg->workb, (size_t)TJPGD_WORKBUFF_SIZE, &(sjpeg->io), 0);
            if (rc != JDR_OK) return rc;
            rc = jd_decomp(sjpeg->tjpeg_jd, img_data_cb, 0);
            if (rc != JDR_OK) return rc;
            sjpeg->sjpeg_cache_frame_index = sjpeg_req_frame_index;
        }

    }
    else if (dsc->src_type == LV_IMG_SRC_FILE)
    {
        SJPEG *sjpeg = (SJPEG *) dsc->user_data;
        JRESULT rc;
        //int sjpeg_req_frame_index = y / sjpeg->sjpeg_single_frame_height;

        int lv_file_p = sjpeg->io.lv_file;
        if (lv_file_p < 0) return JDR_INP;

        /*If line not from cache, refresh cache */
        if (sjpeg_req_frame_index != sjpeg->sjpeg_cache_frame_index)
        {
            sjpeg->io.raw_sjpg_data_next_read_pos = (int)(sjpeg->frame_base_offset [ sjpeg_req_frame_index ]);
            lseek(sjpeg->io.lv_file, sjpeg->io.raw_sjpg_data_next_read_pos, SEEK_SET);

            rc = jd_prepare_ex(sjpeg->tjpeg_jd, input_func, sjpeg->workb, (size_t)TJPGD_WORKBUFF_SIZE, &(sjpeg->io), 0);
            if (rc != JDR_OK) return rc;

            rc = jd_decomp(sjpeg->tjpeg_jd, img_data_cb, 0);
            if (rc != JDR_OK) return rc;

            sjpeg->sjpeg_cache_frame_index = sjpeg_req_frame_index;
        }
    }
    else
    {
        return JDR_PAR;
    }

    rt_kprintf("%s: %d(ms)\n", __func__, lv_tick_get() - start);

    return JDR_OK;

}
#ifdef USING_JPEG_DEC

/**
 * @brief  Allocate memory.
           The JPEG decoder requires 64-byte address alignment.
 * @param  size Size of the memory to allocate in bytes
 * @retval pointer to the allocated memory
 */
static void *jpeg_aligned_malloc(lv_img_decoder_dsc_t *dsc, size_t size)
{
    const size_t alignment = 64;
    const size_t offset = alignment - 1 + sizeof(void *);
    size = size + offset;
    void *raw_ptr = sjpg_frame_alloc(dsc->reuse_anim_buf, size);
    if (raw_ptr == NULL)
    {
        return NULL;
    }
    void *aligned_ptr = (void *)(((uintptr_t)raw_ptr + sizeof(void *) + alignment - 1) & ~(alignment - 1));
    *(void **)((uintptr_t)aligned_ptr - sizeof(void *)) = raw_ptr;

    return aligned_ptr;
}

/**
 * @brief  Free memory.
 * @param  aligned_ptr the address of memory which will be released.
 */
static void jpeg_aligned_free(void *aligned_ptr)
{
    if (aligned_ptr == NULL)
    {
        return;
    }
    void *raw_ptr = *(void **)((uintptr_t)aligned_ptr - sizeof(void *));
    sjpg_frame_free(raw_ptr);
}

static void sjpg_scale_decoded_buffer(uint8_t *dst, uint32_t dst_w, uint32_t dst_h,
                                      const uint8_t *src, uint32_t src_w, uint32_t src_h,
                                      uint32_t pixel_size)
{
    uint32_t y;

    if (!dst || !src || !dst_w || !dst_h || !src_w || !src_h || !pixel_size)
    {
        return;
    }

    for (y = 0; y < dst_h; y++)
    {
        uint32_t src_y = (uint32_t)(((uint64_t)y * (uint64_t)src_h) / (uint64_t)dst_h);
        uint32_t x;

        if (src_y >= src_h) src_y = src_h - 1U;

        for (x = 0; x < dst_w; x++)
        {
            uint32_t src_x = (uint32_t)(((uint64_t)x * (uint64_t)src_w) / (uint64_t)dst_w);
            const uint8_t *src_px;
            uint8_t *dst_px;

            if (src_x >= src_w) src_x = src_w - 1U;

            src_px = src + ((src_y * src_w) + src_x) * pixel_size;
            dst_px = dst + ((y * dst_w) + x) * pixel_size;
            memcpy(dst_px, src_px, pixel_size);
        }
    }
}

/**
 * Decode SJPG image and return the decoded image.
   Currently, only the RGB565/ARGB888 formats are supported.
 * @param dsc pointer to a descriptor which describes this decoding session
 * @param jpeg_data pointer to the jpeg data,source must in SRAM or PSRAM.
 * @param data_size jpeg data size.
 * @return LV_RES_OK: no error; LV_RES_INV: can't get the info
 */
static lv_res_t lv_jpeg_decode(lv_img_decoder_dsc_t *dsc, uint8_t *jpeg_data, uint32_t data_size)
{
    uint32_t width = 0;
    uint32_t height = 0;
    uint8_t *decode_buf = NULL;
    uint8_t *out_buf = NULL;
    uint32_t jpeg_data_len;
    uint32_t decode_buf_size = 0;
    uint32_t out_buf_size = 0;
    int out_width = 0;
    int out_height = 0;
    int ret;

#if (24 == LV_COLOR_DEPTH)
    const char *decode_format = "ARGB8888";
    uint32_t cf = LV_IMG_CF_RGBA8888;
    uint32_t pixel_size = 4;
#else
    const char *decode_format = "RGB565";
    uint32_t cf = LV_IMG_CF_RGB565;
    uint32_t pixel_size = 2;
#endif
    jpeg_data_len = data_size ;

    SJPEG *sjpeg = (SJPEG *) dsc->user_data;

    dsc->img_data = NULL;
    dsc->img_data_size = 0;

    rt_kprintf("%s: jpg_data %p data_size %d\n", __func__, jpeg_data, data_size);

    if (!jpeg_data || jpeg_data_len == 0U)
    {
        return LV_RES_INV;
    }

    if (!sjpeg)
    {
        sjpeg =  sjpg_calloc(1, sizeof(SJPEG));
        if (!sjpeg) return LV_RES_INV;
        dsc->user_data = sjpeg;
    }

    jpeg_decode_init();

    ret = jpeg_decode_get_dimension(jpeg_data, jpeg_data_len, &width, &height);
    if (0 != ret || width == 0U || height == 0U)
    {
        jpeg_decode_deinit();
        return LV_RES_INV;
    }

    if (!g_algo_ops->calc_decode_size_rgb((int)width, (int)height, &out_width, &out_height) ||
            !g_algo_ops->calc_cache_size_rgb((int)width, (int)height, &out_buf_size))
    {
        jpeg_decode_deinit();
        return LV_RES_INV;
    }

    {
        uint64_t full_decode_size = (uint64_t)width * (uint64_t)height * (uint64_t)pixel_size;
        if (full_decode_size == 0U || full_decode_size > UINT32_MAX)
        {
            jpeg_decode_deinit();
            return LV_RES_INV;
        }
        decode_buf_size = (uint32_t)full_decode_size;
    }

    decode_buf = (uint8_t *)jpeg_aligned_malloc(dsc, decode_buf_size);
    if (!decode_buf)
    {
        jpeg_decode_deinit();
        return LV_RES_INV;
    }

    ret = jpeg_decode2(jpeg_data, jpeg_data_len, decode_format, decode_buf, decode_buf_size);

    jpeg_decode_deinit();

    if ((uint32_t)ret != jpeg_data_len)
    {
        rt_kprintf("decode fail, err=%d jpeg_data_len %d \n", ret, jpeg_data_len);
        jpeg_aligned_free(decode_buf);
        return LV_RES_INV;
    }

    if ((uint32_t)out_width == width && (uint32_t)out_height == height)
    {
        out_buf = decode_buf;
    }
    else
    {
        out_buf = jpeg_aligned_malloc(dsc, out_buf_size);
        if (!out_buf)
        {
            jpeg_aligned_free(decode_buf);
            return LV_RES_INV;
        }
        sjpg_scale_decoded_buffer(out_buf, (uint32_t)out_width, (uint32_t)out_height, decode_buf, width, height, pixel_size);
        jpeg_aligned_free(decode_buf);
    }

    if (sjpeg->frame_cache)
    {
        jpeg_aligned_free(sjpeg->frame_cache);
    }
    sjpeg->frame_cache = out_buf;

    dsc->img_data = out_buf;
    dsc->img_data_size = out_buf_size;
    dsc->header.w = (lv_coord_t)out_width;
    dsc->header.h = (lv_coord_t)out_height;
    dsc->header.cf = cf;

    return LV_RES_OK;
}

/**
 * Open SJPG image and return the decoded image
 * @param decoder pointer to the decoder where this function belongs
 * @param dsc pointer to a descriptor which describes this decoding session
 * @return LV_RES_OK: no error; LV_RES_INV: can't get the info
 */
static lv_res_t decoder_open(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc)
{
    LV_UNUSED(decoder);
    lv_res_t lv_ret = LV_RES_OK;

    if (dsc->src_type == LV_IMG_SRC_VARIABLE)
    {
        lv_img_dsc_t *img_dsc = (lv_img_dsc_t *)dsc->src;
        if (!img_dsc->data || 0 == img_dsc->data_size)  return LV_RES_INV;

        if (0 != strncmp((char *) img_dsc->data, "_SJPG__", strlen("_SJPG__")) && \
                !is_jpg(img_dsc->data, img_dsc->data_size))
        {
            return LV_RES_INV;
        }

        uint8_t *data = (uint8_t *)jpeg_aligned_malloc(dsc, img_dsc->data_size);
        if (!data) return LV_RES_INV;

        memcpy(data, img_dsc->data, img_dsc->data_size);

        lv_ret = lv_jpeg_decode(dsc, data, img_dsc->data_size);

        jpeg_aligned_free(data);

        return lv_ret;

    }
    else if (dsc->src_type == LV_IMG_SRC_FILE)
    {
        const char *fn = dsc->src;
        const char *ext = file_get_ext(fn);

        int lv_file = open(fn, 0);
        if (lv_file < 0)
        {
            return LV_RES_INV;
        }

        /* judge __seq case, get length and offset. */
        uint32_t length = 0;
        uint32_t offset = 0;
        int file_size = lseek(lv_file, 0, SEEK_END);

        if (strcmp(ext, "bin") == 0)
        {
            offset = lv_img_decoder_get_wf_offset(fn, &length);
            if (offset) file_size = length;
            file_size -= sizeof(lv_img_header_t);
            offset += sizeof(lv_img_header_t);
        }

        lseek(lv_file, offset, SEEK_SET);

        if (strcmp(ext, "sjpg") == 0)
        {
            uint8_t buff[22];
            memset(buff, 0, sizeof(buff));

            uint32_t rn = read(lv_file, buff, 22);
            if (rn != 22)
            {
                close(lv_file);
                return LV_RES_INV;
            }
            if (strcmp((char *)buff, "_SJPG__") == 0)
            {
                uint8_t *file_data = (uint8_t *)jpeg_aligned_malloc(dsc, file_size);
                if (!file_data)
                {
                    close(lv_file);
                    return LV_RES_INV;
                }

                int ret = read(lv_file, file_data, file_size);
                close(lv_file);

                if (file_size != ret)
                {
                    jpeg_aligned_free(file_data);
                    return LV_RES_INV;
                }

                lv_ret = lv_jpeg_decode(dsc, file_data, file_size);

                jpeg_aligned_free(file_data);

                return lv_ret;
            }
        }
        else if (strcmp(ext, "jpg") == 0 || strcmp(ext, "jpeg") == 0 || strcmp(ext, "bin") == 0)
        {
            if (file_size <= 0)
            {
                close(lv_file);
                return LV_RES_INV;
            }

            uint8_t *file_data = (uint8_t *)jpeg_aligned_malloc(dsc, file_size);
            if (!file_data)
            {
                close(lv_file);
                return LV_RES_INV;
            }

            int ret = read(lv_file, file_data, file_size);
            close(lv_file);

            if (file_size != ret)
            {
                jpeg_aligned_free(file_data);
                return LV_RES_INV;
            }

            lv_ret = lv_jpeg_decode(dsc, file_data, file_size);

            jpeg_aligned_free(file_data);

            return lv_ret;

        }
        close(lv_file);
    }

    return LV_RES_INV;
}

#else

/**
 * Open SJPG image and return the decided image
 * @param decoder pointer to the decoder where this function belongs
 * @param dsc pointer to a descriptor which describes this decoding session
 * @return LV_RES_OK: no error; LV_RES_INV: can't get the info
 */
static lv_res_t decoder_open(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc)
{
    LV_UNUSED(decoder);
    lv_res_t lv_ret = LV_RES_OK;

    if (dsc->src_type == LV_IMG_SRC_VARIABLE)
    {
        //rt_kprintf("%s_jpg: dsc->src_type %d dsc->src %p\n", __func__, dsc->src_type, dsc->src);

        lv_img_dsc_t *img_dsc = (lv_img_dsc_t *)dsc->src;
        if (!img_dsc->data || 0 == img_dsc->data_size)  return LV_RES_INV;

        uint8_t *data;
        SJPEG *sjpeg = (SJPEG *) dsc->user_data;
        const uint32_t raw_sjpeg_data_size = img_dsc->data_size;

        if (!sjpeg)
        {
            sjpeg =  sjpg_calloc(1, sizeof(SJPEG));
            if (!sjpeg) return LV_RES_INV;

            memset(sjpeg, 0, sizeof(SJPEG));
            dsc->user_data = sjpeg;
        }
        else
        {
            if (sjpeg->sjpeg_data != (uint8_t *)img_dsc->data || sjpeg->sjpeg_data_size != raw_sjpeg_data_size)
            {
                lv_sjpg_free(sjpeg);
                memset(sjpeg, 0, sizeof(SJPEG));
            }
        }

        sjpeg->sjpeg_data = (uint8_t *)img_dsc->data;
        sjpeg->sjpeg_data_size = img_dsc->data_size;

        if (!strncmp((char *) sjpeg->sjpeg_data, "_SJPG__", strlen("_SJPG__")))
        {

            data = sjpeg->sjpeg_data;
            data += 14;

            sjpeg->sjpeg_x_res = *data++;
            sjpeg->sjpeg_x_res |= *data++ << 8;

            sjpeg->sjpeg_y_res = *data++;
            sjpeg->sjpeg_y_res |= *data++ << 8;

            sjpeg->sjpeg_total_frames = *data++;
            sjpeg->sjpeg_total_frames |= *data++ << 8;

            sjpeg->sjpeg_single_frame_height = *data++;
            sjpeg->sjpeg_single_frame_height |= *data++ << 8;

            sjpeg->frame_base_array = sjpg_calloc(1, sizeof(uint8_t *) * sjpeg->sjpeg_total_frames);
            if (! sjpeg->frame_base_array)
            {
                decoder_close(NULL, dsc);
                return LV_RES_INV;
            }

            sjpeg->frame_base_offset = NULL;

            uint8_t *img_frame_base = data +  sjpeg->sjpeg_total_frames * 2;
            sjpeg->frame_base_array[0] = img_frame_base;

            for (int i = 1; i <  sjpeg->sjpeg_total_frames; i++)
            {
                int offset = *data++;
                offset |= *data++ << 8;
                sjpeg->frame_base_array[i] = sjpeg->frame_base_array[i - 1] + offset;
            }
            sjpeg->sjpeg_cache_frame_index = -1;
            sjpeg->frame_cache = (void *)sjpg_frame_alloc(dsc->reuse_anim_buf, single_frame_size(sjpeg));
            if (! sjpeg->frame_cache)
            {
                rt_kprintf("%s: sjpg_frame_alloc failed! %d reuse_anim_buf=%d\n",
                           __func__, single_frame_size(sjpeg), dsc->reuse_anim_buf);
                decoder_close(NULL, dsc);
                return LV_RES_INV;
            }

            dsc->img_data = sjpeg->frame_cache;
            dsc->img_data_size = single_frame_size(sjpeg);
            sjpeg->io.algo.img_cache_buff = sjpeg->frame_cache;
#if (3 == JD_FORMAT)
            if (!g_algo_ops->calc_decode_size_yuv(sjpeg->sjpeg_x_res, sjpeg->sjpeg_single_frame_height, &sjpeg->io.algo.img_cache_x_res, &sjpeg->io.algo.img_cache_y_res))
#else
            if (!g_algo_ops->calc_decode_size_rgb(sjpeg->sjpeg_x_res, sjpeg->sjpeg_single_frame_height, &sjpeg->io.algo.img_cache_x_res, &sjpeg->io.algo.img_cache_y_res))
#endif
            {
                decoder_close(NULL, dsc);
                return LV_RES_INV;
            }
            sjpeg->io.algo.img_src_x_res = sjpeg->sjpeg_x_res;
            sjpeg->io.algo.img_src_y_res = sjpeg->sjpeg_single_frame_height;
            if (!g_algo_ops->prepare_scale_maps(&sjpeg->io.algo))
            {
                decoder_close(NULL, dsc);
                return LV_RES_INV;
            }
            dsc->header.w = sjpeg->io.algo.img_cache_x_res;
            dsc->header.h = sjpeg->io.algo.img_cache_y_res;
            dsc->header.cf = SJEPG_OUTPUT_EPIC_FORMAT;
            sjpeg->workb =   sjpg_alloc(TJPGD_WORKBUFF_SIZE);
            if (! sjpeg->workb)
            {
                decoder_close(NULL, dsc);
                return LV_RES_INV;
            }

            sjpeg->tjpeg_jd =   sjpg_alloc(sizeof(JDEC));
            if (! sjpeg->tjpeg_jd)
            {
                decoder_close(NULL, dsc);
                return LV_RES_INV;
            }

            sjpeg->io.type = SJPEG_IO_SOURCE_C_ARRAY;
            sjpeg->io.lv_file = -1;

            if (jd_decode_frame(dsc, 0) != JDR_OK)
            {
                decoder_close(NULL, dsc);
                return LV_RES_INV;
            }
            return lv_ret;
        }
        else if (is_jpg(sjpeg->sjpeg_data, raw_sjpeg_data_size) == true)
        {

            uint8_t *workb_temp = sjpg_calloc(1, TJPGD_WORKBUFF_SIZE);
            if (! workb_temp)
            {
                decoder_close(NULL, dsc);
                return LV_RES_INV;
            }
            io_source_t io_source_temp;
            io_source_temp.type = SJPEG_IO_SOURCE_C_ARRAY;
            io_source_temp.raw_sjpg_data =  sjpeg->sjpeg_data;
            io_source_temp.raw_sjpg_data_size = sjpeg->sjpeg_data_size;
            io_source_temp.raw_sjpg_data_next_read_pos = 0;

            JDEC jd_tmp;
            JRESULT rc = jd_prepare_ex(&jd_tmp, input_func, workb_temp, (size_t)TJPGD_WORKBUFF_SIZE, &io_source_temp, 0);
            sjpg_free(workb_temp);
            workb_temp = NULL;

            if (rc == JDR_OK)
            {
                sjpeg->sjpeg_x_res = jd_tmp.width;
                sjpeg->sjpeg_y_res = jd_tmp.height;
                sjpeg->sjpeg_total_frames = 1;
                sjpeg->sjpeg_single_frame_height = jd_tmp.height;

                sjpeg->frame_base_array = sjpg_calloc(1, sizeof(uint8_t *) * sjpeg->sjpeg_total_frames);
                if (! sjpeg->frame_base_array)
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }
                sjpeg->frame_base_offset = NULL;

                uint8_t *img_frame_base = sjpeg->sjpeg_data;
                sjpeg->frame_base_array[0] = img_frame_base;

                sjpeg->sjpeg_cache_frame_index = -1;
                sjpeg->frame_cache = (void *)sjpg_frame_alloc(dsc->reuse_anim_buf, single_frame_size(sjpeg));
                if (! sjpeg->frame_cache)
                {
                    rt_kprintf("%s: sjpg_frame_alloc failed! %d reuse_anim_buf=%d\n",
                               __func__, single_frame_size(sjpeg), dsc->reuse_anim_buf);
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }

                dsc->img_data = sjpeg->frame_cache;
                dsc->img_data_size = single_frame_size(sjpeg);
                sjpeg->io.algo.img_cache_buff = sjpeg->frame_cache;
#if (3 == JD_FORMAT)
                if (!g_algo_ops->calc_decode_size_yuv(sjpeg->sjpeg_x_res, sjpeg->sjpeg_single_frame_height, &sjpeg->io.algo.img_cache_x_res, &sjpeg->io.algo.img_cache_y_res))
#else
                if (!g_algo_ops->calc_decode_size_rgb(sjpeg->sjpeg_x_res, sjpeg->sjpeg_single_frame_height, &sjpeg->io.algo.img_cache_x_res, &sjpeg->io.algo.img_cache_y_res))
#endif
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }
                sjpeg->io.algo.img_src_x_res = sjpeg->sjpeg_x_res;
                sjpeg->io.algo.img_src_y_res = sjpeg->sjpeg_single_frame_height;
                if (!g_algo_ops->prepare_scale_maps(&sjpeg->io.algo))
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }
                dsc->header.w = sjpeg->io.algo.img_cache_x_res;
                dsc->header.h = sjpeg->io.algo.img_cache_y_res;
                dsc->header.cf = SJEPG_OUTPUT_EPIC_FORMAT;
                sjpeg->workb =   sjpg_alloc(TJPGD_WORKBUFF_SIZE);
                if (! sjpeg->workb)
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }

                sjpeg->tjpeg_jd =   sjpg_alloc(sizeof(JDEC));
                if (! sjpeg->tjpeg_jd)
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }

                sjpeg->io.type = SJPEG_IO_SOURCE_C_ARRAY;
                sjpeg->io.lv_file = -1;

                if (jd_decode_frame(dsc, 0) != JDR_OK)
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }
                return lv_ret;
            }
            else
            {
                decoder_close(NULL, dsc);
                return LV_RES_INV;
            }

            return lv_ret;
        }
    }
    else if (dsc->src_type == LV_IMG_SRC_FILE)
    {
        const char *fn = dsc->src;
        const char *ext = file_get_ext(fn);

        int lv_file = open(fn, 0);
        if (lv_file < 0)
        {
            return LV_RES_INV;
        }

        /* judge __seq case, get length and offset. */
        uint32_t length = 0;
        uint32_t offset = 0;
        int file_size = lseek(lv_file, 0, SEEK_END);

        if (strcmp(ext, "bin") == 0)
        {
            offset = lv_img_decoder_get_wf_offset(fn, &length);
            if (offset) file_size = length;
            file_size -= sizeof(lv_img_header_t);
            offset += sizeof(lv_img_header_t);
        }

        lseek(lv_file, offset, SEEK_SET);

        /* If all fine, then the file will be kept open */
        uint8_t *data;

        if (strcmp(ext, "sjpg") == 0)
        {

            uint8_t buff[22];
            memset(buff, 0, sizeof(buff));

            uint32_t rn = read(lv_file, buff, 22);
            if (rn != 22)
            {
                decoder_close(NULL, dsc);
                return LV_RES_INV;
            }

            if (strcmp((char *)buff, "_SJPG__") == 0)
            {

                SJPEG *sjpeg = (SJPEG *) dsc->user_data;
                if (sjpeg == NULL)
                {
                    sjpeg = sjpg_calloc(1, sizeof(SJPEG));

                    if (! sjpeg)
                    {
                        decoder_close(NULL, dsc);
                        return LV_RES_INV;
                    }
                    memset(sjpeg, 0, sizeof(SJPEG));

                    dsc->user_data = sjpeg;
                    sjpeg->sjpeg_data = NULL;
                    sjpeg->sjpeg_data_size = 0;
                }

                sjpeg->io.type = SJPEG_IO_SOURCE_DISK;
                sjpeg->io.lv_file = lv_file;
                dsc->img_data = NULL;

                data = buff;
                data += 14;

                sjpeg->sjpeg_x_res = *data++;
                sjpeg->sjpeg_x_res |= *data++ << 8;

                sjpeg->sjpeg_y_res = *data++;
                sjpeg->sjpeg_y_res |= *data++ << 8;

                sjpeg->sjpeg_total_frames = *data++;
                sjpeg->sjpeg_total_frames |= *data++ << 8;

                sjpeg->sjpeg_single_frame_height = *data++;
                sjpeg->sjpeg_single_frame_height |= *data++ << 8;

                sjpeg->frame_base_array = NULL;//sjpg_alloc( sizeof(uint8_t *) * sjpeg->sjpeg_total_frames );
                sjpeg->frame_base_offset = sjpg_calloc(1, sizeof(int) * sjpeg->sjpeg_total_frames);
                if (! sjpeg->frame_base_offset)
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }

                int img_frame_start_offset = (SJPEG_FRAME_INFO_ARRAY_OFFSET + sjpeg->sjpeg_total_frames * 2);
                sjpeg->frame_base_offset[0] = img_frame_start_offset; //pointer used to save integer for now...

                for (int i = 1; i <  sjpeg->sjpeg_total_frames; i++)
                {
                    rn = read(lv_file, buff, 2);
                    if (rn != 2)
                    {
                        decoder_close(NULL, dsc);
                        return LV_RES_INV;
                    }

                    data = buff;
                    int offset = *data++;
                    offset |= *data++ << 8;
                    sjpeg->frame_base_offset[i] = sjpeg->frame_base_offset[i - 1] + offset;
                }

                sjpeg->sjpeg_cache_frame_index = -1; //INVALID AT BEGINNING for a forced compare mismatch at first time.
                sjpeg->frame_cache = (void *)sjpg_frame_alloc(dsc->reuse_anim_buf, single_frame_size(sjpeg));
                if (! sjpeg->frame_cache)
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }

                dsc->img_data = sjpeg->frame_cache;
                dsc->img_data_size = single_frame_size(sjpeg);
                sjpeg->io.algo.img_cache_buff = sjpeg->frame_cache;
#if (3 == JD_FORMAT)
                if (!g_algo_ops->calc_decode_size_yuv(sjpeg->sjpeg_x_res, sjpeg->sjpeg_single_frame_height, &sjpeg->io.algo.img_cache_x_res, &sjpeg->io.algo.img_cache_y_res))
#else
                if (!g_algo_ops->calc_decode_size_rgb(sjpeg->sjpeg_x_res, sjpeg->sjpeg_single_frame_height, &sjpeg->io.algo.img_cache_x_res, &sjpeg->io.algo.img_cache_y_res))
#endif
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }
                sjpeg->io.algo.img_src_x_res = sjpeg->sjpeg_x_res;
                sjpeg->io.algo.img_src_y_res = sjpeg->sjpeg_single_frame_height;
                if (!g_algo_ops->prepare_scale_maps(&sjpeg->io.algo))
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }
                dsc->header.w = sjpeg->io.algo.img_cache_x_res;
                dsc->header.h = sjpeg->io.algo.img_cache_y_res;
                dsc->header.cf = SJEPG_OUTPUT_EPIC_FORMAT;
                sjpeg->workb =   sjpg_alloc(TJPGD_WORKBUFF_SIZE);
                if (! sjpeg->workb)
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }

                sjpeg->tjpeg_jd =    sjpg_alloc(sizeof(JDEC));
                if (! sjpeg->tjpeg_jd)
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }

                if (jd_decode_frame(dsc, 0) != JDR_OK)
                    lv_ret = LV_RES_INV;
                close(lv_file);
                sjpeg->io.lv_file = -1;
                return lv_ret;
            }
        }
        else if (strcmp(ext, "jpg") == 0 || strcmp(ext, "jpeg") == 0 || strcmp(ext, "bin") == 0)
        {
            SJPEG *sjpeg = (SJPEG *) dsc->user_data;
            if (! sjpeg)
            {
                sjpeg = sjpg_calloc(1, sizeof(SJPEG));
                if (! sjpeg)
                {
                    return LV_RES_INV;
                }

                dsc->user_data = sjpeg;
                sjpeg->sjpeg_data = NULL;
                sjpeg->sjpeg_data_size = 0;
            }

            sjpeg->io.type = SJPEG_IO_SOURCE_DISK;
            sjpeg->io.lv_file = lv_file;
            dsc->img_data = NULL;

            uint8_t *workb_temp = sjpg_alloc(TJPGD_WORKBUFF_SIZE);
            if (! workb_temp)
            {
                decoder_close(NULL, dsc);
                return LV_RES_INV;
            }

            io_source_t io_source_temp;
            io_source_temp.type = SJPEG_IO_SOURCE_DISK;
            io_source_temp.raw_sjpg_data_next_read_pos = 0;
            io_source_temp.algo.img_cache_buff = NULL;
            io_source_temp.lv_file = lv_file;

            JDEC jd_tmp;

            JRESULT rc = jd_prepare_ex(&jd_tmp, input_func, workb_temp, (size_t)TJPGD_WORKBUFF_SIZE, &io_source_temp, 0);

            sjpg_free(workb_temp);

            if (rc == JDR_OK)
            {
                int out_w;
                int out_h;
#if (3 == JD_FORMAT)
                g_algo_ops->calc_decode_size_yuv((int)jd_tmp.width, (int)jd_tmp.height, &out_w, &out_h);
#else
                g_algo_ops->calc_decode_size_rgb((int)jd_tmp.width, (int)jd_tmp.height, &out_w, &out_h);
#endif
                RT_ASSERT(dsc->header.w == (lv_coord_t)out_w && dsc->header.h == (lv_coord_t)out_h);
                sjpeg->sjpeg_x_res = jd_tmp.width;
                sjpeg->sjpeg_y_res = jd_tmp.height;
                sjpeg->sjpeg_total_frames = 1;
                sjpeg->sjpeg_single_frame_height = jd_tmp.height;

                sjpeg->frame_base_array = NULL;
                sjpeg->frame_base_offset =  sjpg_calloc(1, sizeof(uint8_t *) * sjpeg->sjpeg_total_frames);
                if (! sjpeg->frame_base_offset)
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }

                int img_frame_start_offset = 0;
                sjpeg->frame_base_offset[0] = img_frame_start_offset;

                sjpeg->sjpeg_cache_frame_index = -1;
                sjpeg->frame_cache = (void *)sjpg_frame_alloc(dsc->reuse_anim_buf, single_frame_size(sjpeg));
                if (! sjpeg->frame_cache)
                {
                    rt_kprintf("%s: sjpg_frame_alloc failed! %d reuse_anim_buf=%d\n",
                               __func__, single_frame_size(sjpeg), dsc->reuse_anim_buf);
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }

                dsc->img_data = sjpeg->frame_cache;
                dsc->img_data_size = single_frame_size(sjpeg);
                sjpeg->io.algo.img_cache_buff = sjpeg->frame_cache;
#if (3 == JD_FORMAT)
                if (!g_algo_ops->calc_decode_size_yuv(sjpeg->sjpeg_x_res, sjpeg->sjpeg_single_frame_height, &sjpeg->io.algo.img_cache_x_res, &sjpeg->io.algo.img_cache_y_res))
#else
                if (!g_algo_ops->calc_decode_size_rgb(sjpeg->sjpeg_x_res, sjpeg->sjpeg_single_frame_height, &sjpeg->io.algo.img_cache_x_res, &sjpeg->io.algo.img_cache_y_res))
#endif
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }
                sjpeg->io.algo.img_src_x_res = sjpeg->sjpeg_x_res;
                sjpeg->io.algo.img_src_y_res = sjpeg->sjpeg_single_frame_height;
                if (!g_algo_ops->prepare_scale_maps(&sjpeg->io.algo))
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }
                dsc->header.w = sjpeg->io.algo.img_cache_x_res;
                dsc->header.h = sjpeg->io.algo.img_cache_y_res;
                dsc->header.cf = SJEPG_OUTPUT_EPIC_FORMAT;
                sjpeg->workb =   sjpg_alloc(TJPGD_WORKBUFF_SIZE);
                if (! sjpeg->workb)
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }

                sjpeg->tjpeg_jd =   sjpg_alloc(sizeof(JDEC));
                if (! sjpeg->tjpeg_jd)
                {
                    decoder_close(NULL, dsc);
                    return LV_RES_INV;
                }


                if (jd_decode_frame(dsc, 0) != JDR_OK)
                {
                    lv_ret = LV_RES_INV;
                }
                close(lv_file);
                sjpeg->io.lv_file = -1;
                return lv_ret;

            }
            else
            {
                decoder_close(NULL, dsc);
            }
        }
        close(lv_file);
    }

    return LV_RES_INV;
}

#endif


/**
 * Decode `len` pixels starting from the given `x`, `y` coordinates and store them in `buf`.
 * Required only if the "open" function can't open the whole decoded pixel array. (dsc->img_data == NULL)
 * @param decoder pointer to the decoder the function associated with
 * @param dsc pointer to decoder descriptor
 * @param x start x coordinate
 * @param y start y coordinate
 * @param len number of pixels to decode
 * @param buf a buffer to store the decoded pixels
 * @return LV_RES_OK: ok; LV_RES_INV: failed
 */

static lv_res_t decoder_read_line(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc, lv_coord_t x, lv_coord_t y,
                                  lv_coord_t len, uint8_t *buf)
{
    return LV_RES_INV;
}


/**
 * Free the allocated resources
 * @param decoder pointer to the decoder where this function belongs
 * @param dsc pointer to a descriptor which describes this decoding session
 */
static void decoder_close(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc)
{
    LV_UNUSED(decoder);

    //rt_kprintf("%s: user_data %p src_type %d\n", __func__, dsc->user_data, dsc->src_type);
    /*Free all allocated data*/
    SJPEG *sjpeg = (SJPEG *) dsc->user_data;
    if (!sjpeg) return;

    switch (dsc->src_type)
    {
    case LV_IMG_SRC_FILE:
        if (sjpeg->io.lv_file >= 0)
        {
            close(sjpeg->io.lv_file);
        }
        lv_sjpg_cleanup(sjpeg);
        break;

    case LV_IMG_SRC_VARIABLE:
        lv_sjpg_cleanup(sjpeg);
        break;

    default:
        ;
    }

    dsc->user_data = NULL;
}


static int is_jpg(const uint8_t *raw_data, size_t len)
{
    const uint8_t jpg_signature_standard[] = {0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46};
    const uint8_t jpg_signature_exif[] = {0xFF, 0xD8, 0xFF, 0xE1, 0x00, 0x18, 0x45, 0x78, 0x69, 0x66};
    const uint8_t jpg_signature_sof[] = {0xFF, 0xD8, 0xFF}; // SOF start

    if (len < sizeof(jpg_signature_standard) || len < sizeof(jpg_signature_exif)) return 0;

    if (memcmp(jpg_signature_standard, raw_data, sizeof(jpg_signature_standard)) == 0)
    {
        return 1;
    }

    if (memcmp(jpg_signature_exif, raw_data, sizeof(jpg_signature_exif)) == 0)
    {
        return 1;
    }

    // Check if it starts with SOF (Start of Frame)
    if (len >= 3 && raw_data[0] == 0xFF && raw_data[1] == 0xD8 && raw_data[2] == 0xFF)
    {
        return 1;
    }

    return 0;
}

static void lv_sjpg_free_ctrl(SJPEG *sjpeg)
{
    if (sjpeg->io.type == SJPEG_IO_SOURCE_C_ARRAY)
    {
        if (sjpeg->frame_base_array) sjpg_free(sjpeg->frame_base_array);
    }

    if (sjpeg->frame_base_offset) sjpg_free(sjpeg->frame_base_offset);
    if (sjpeg->tjpeg_jd) sjpg_free(sjpeg->tjpeg_jd);
    if (sjpeg->workb) sjpg_free(sjpeg->workb);
    if (sjpeg->io.algo.y_row_map) sjpg_free(sjpeg->io.algo.y_row_map);
    if (sjpeg->io.algo.y_col_map) sjpg_free(sjpeg->io.algo.y_col_map);
    if (sjpeg->io.algo.uv_row_map) sjpg_free(sjpeg->io.algo.uv_row_map);
    if (sjpeg->io.algo.uv_col_map) sjpg_free(sjpeg->io.algo.uv_col_map);
    sjpeg->frame_base_array = NULL;
    sjpeg->tjpeg_jd = NULL;
    sjpeg->frame_base_offset = NULL;
    sjpeg->workb = NULL;
    sjpeg->io.algo.y_row_map = NULL;
    sjpeg->io.algo.y_col_map = NULL;
    sjpeg->io.algo.uv_row_map = NULL;
    sjpeg->io.algo.uv_col_map = NULL;
}


static void lv_sjpg_free(SJPEG *sjpeg)
{
    //rt_kprintf("%s: %p %p %p %p %p\n", __func__, sjpeg->frame_cache, sjpeg->frame_base_array, sjpeg->frame_base_offset, sjpeg->tjpeg_jd, sjpeg->workb);
#ifdef USING_JPEG_DEC
    if (sjpeg->frame_cache) jpeg_aligned_free(sjpeg->frame_cache);
#else
    if (sjpeg->frame_cache) sjpg_frame_free(sjpeg->frame_cache);
#endif
    sjpeg->frame_cache = NULL;
    lv_sjpg_free_ctrl(sjpeg);
}

static void lv_sjpg_cleanup(SJPEG *sjpeg)
{
    if (! sjpeg) return;

    lv_sjpg_free(sjpeg);
    sjpg_free(sjpeg);
}

static int is_supported_jpeg_sof_marker(uint16_t marker)
{
    switch (marker)
    {
    case 0xFFC0:
    case 0xFFC1:
    case 0xFFC2:
    case 0xFFC3:
    case 0xFFC5:
    case 0xFFC6:
    case 0xFFC7:
    case 0xFFC9:
    case 0xFFCA:
    case 0xFFCB:
    case 0xFFCD:
    case 0xFFCE:
    case 0xFFCF:
        return 1;
    default:
        return 0;
    }
}

static int get_sjpg_size_from_file(int fd, uint32_t offset, uint16_t *width, uint16_t *height)
{
    uint8_t buf[2];
    uint16_t marker;
    uint16_t seg_len;

    if (fd < 0 || width == NULL || height == NULL) return -1;
    if (lseek(fd, (int)offset, SEEK_SET) < 0) return -1;

    if (read(fd, buf, 2) != 2 || buf[0] != 0xFF || buf[1] != 0xD8) return -1;

    while (1)
    {
        uint8_t c;

        do
        {
            if (read(fd, &c, 1) != 1) return -2;
        }
        while (c != 0xFF);

        do
        {
            if (read(fd, &c, 1) != 1) return -2;
        }
        while (c == 0xFF);

        if (c == 0x00) continue;

        marker = (uint16_t)((0xFFU << 8) | c);
        if (marker == 0xFFD9 || marker == 0xFFDA) return -2;

        if ((marker >= 0xFFD0 && marker <= 0xFFD7) || marker == 0xFFD8 || marker == 0xFF01)
        {
            continue;
        }

        if (read(fd, buf, 2) != 2) return -1;
        seg_len = (uint16_t)((buf[0] << 8) | buf[1]);
        if (seg_len < 2) return -1;

        if (is_supported_jpeg_sof_marker(marker))
        {
            uint8_t sof[5];

            if (seg_len < 7) return -1;
            if (read(fd, sof, sizeof(sof)) != sizeof(sof)) return -1;

            *height = (uint16_t)((sof[1] << 8) | sof[2]);
            *width = (uint16_t)((sof[3] << 8) | sof[4]);
            return 0;
        }

        if (lseek(fd, (int)(seg_len - 2), SEEK_CUR) < 0) return -1;
    }
}

// Parse the width and height from in-memory JPEG data.
// Parameters:
//   jpeg_data: Start pointer of JPEG data buffer
//   data_len: Total length of the data buffer (prevent out-of-bounds access)
//   width/height: Pointers to store the output width and height
// Return values: 0=success, -1=insufficient data/invalid parameters, -2=SOF segment not found
static int get_sjpg_size_from_data(const uint8_t *jpeg_data, size_t data_len,
                                   uint16_t *width, uint16_t *height)
{
    // Check for invalid input parameters or insufficient initial data
    if (jpeg_data == NULL || width == NULL || height == NULL || data_len < 4)
    {
        return -1; // Invalid input or insufficient data
    }

    size_t offset = 0;
    uint16_t marker, seg_len;

    //printf("%s: data_len %d\n", __func__, data_len);

    // Traverse JPEG marker segments until a supported SOF marker is found or data ends
    while (offset + 2 <= data_len) // Only need 2 bytes to read marker
    {
        // Skip FF padding bytes (JPEG standard allows 0xFF padding between markers)
        while (offset < data_len && jpeg_data[offset] == 0xFF)
        {
            offset++;
        }
        if (offset >= data_len) break;

        // Read 2-byte marker (1st byte is 0xFF, 2nd byte is marker type)
        if (offset + 1 >= data_len) return -1;
        marker = (0xFF << 8) | jpeg_data[offset];
        offset++;

        //printf("%s: marker %x offset %d\n", __func__, marker, offset);

        // Handle special markers without length field
        if (marker == 0xFFD8 || marker == 0xFFD9) // SOI (start) / EOI (end)
        {
            continue; // No length field, skip directly
        }

        // Found a supported SOF marker carrying image size information.
        if (is_supported_jpeg_sof_marker(marker))
        {
            // Check if remaining data is enough to read width and height
            if (offset + 5 > data_len) return -1;

            // Skip: 2-byte segment length + 1-byte precision (total 3 bytes)
            offset += 3;

            // Read height (big-endian byte order)
            *height = (jpeg_data[offset] << 8) | jpeg_data[offset + 1];
            offset += 2;

            // Read width (big-endian byte order)
            *width = (jpeg_data[offset] << 8) | jpeg_data[offset + 1];
            return 0; // Parse success
        }

        // Handle normal markers with length field
        if ((marker & 0xFF00) == 0xFF00)
        {
            // Read segment length (2 bytes, big-endian)
            if (offset + 2 > data_len) return -1;
            seg_len = (jpeg_data[offset] << 8) | jpeg_data[offset + 1];
            offset += 2;

            // Skip segment content (seg_len-2 = content length, 2 bytes for length already read)
            if (offset + seg_len - 2 > data_len) return -1;
            offset += seg_len - 2;
        }
        else
        {
            // Non-JPEG marker, parse failed
            return -2;
        }
    }

    return -2; // SOF segment not found after traversing all data
}


bool lvsf_sjpg_is_jpg(const void *src, lv_img_header_t *header)
{
    lv_res_t res = decoder_info(NULL, src, header);

    return res == LV_RES_OK;
}

lv_res_t lvsf_sjpg_decoder_info(const void *src, lv_img_header_t *header)
{
    lv_res_t ret = LV_RES_INV;
    uint16_t w, h;
    lv_img_src_t src_type = lv_img_src_get_type(src);
    /* Note src can't be builtin_res*/
    if (src_type == LV_IMG_SRC_VARIABLE)
    {
        lv_img_dsc_t *dsc = (lv_img_dsc_t *)src;

        if (0 != get_sjpg_size_from_data((uint8_t *) dsc->data, dsc->data_size, &w, &h))
            return LV_RES_INV;

        dsc->header.w = w;
        dsc->header.h = h;
        dsc->header.cf = LV_IMG_CF_JPG;
        header->w = w;
        header->h = h;
        ret = LV_RES_OK;
    }
    else
    {
        ret = decoder_info(NULL, src, header);
    }

    if (ret == LV_RES_OK)
    {
        int out_w;
        int out_h;
#if (3 == JD_FORMAT)
        if (!g_algo_ops->calc_decode_size_yuv((int)header->w, (int)header->h, &out_w, &out_h))
#else
        if (!g_algo_ops->calc_decode_size_rgb((int)header->w, (int)header->h, &out_w, &out_h))
#endif
        {
            rt_kprintf("%s: calc decode size fail!\n", __func__);
            return LV_RES_INV;
        }
        header->w = (lv_coord_t)out_w;
        header->h = (lv_coord_t)out_h;
        header->cf = SJEPG_OUTPUT_EPIC_FORMAT;
    }
    else
    {
        rt_kprintf("%s: decode info fail!\n", __func__);
    }

    return ret;
}

lv_res_t lvsf_sjpg_decoder_open(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc)
{
    lv_res_t res = decoder_open(decoder, dsc);
    return res;
}

void lvsf_sjpg_decoder_close(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc)
{
    decoder_close(decoder, dsc);
}

INIT_APP_EXPORT(lvsf_split_jpeg_init);

#endif /*LVSF_USING_SJPG*/
