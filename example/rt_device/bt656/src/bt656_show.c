/**
 * @file bt656_show.c
 * @brief BT656 local display on the LCD (EPIC/CPU color conversion).
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @brief BT656 -> LCD display (RT-Thread native path, no LVGL dependency).
 *
 * Data flow:
 *   snapshot: bt656_get() borrows a frame -> YUV422 -> RGB565 -> show_fb -> release
 *   frame/stream: the same capture thread loops get -> render -> release
 *
 * Key points:
 *   - The panel is RAMLESS DPI with line-buffer bypass: draw_rect can only cover
 *     the full screen, and the PTC state machine keeps scanning whatever buffer
 *     was handed to it first. So the scan start is pointed at show_fb only once,
 *     then every frame just updates the canvas and writes back the cache;
 *   - RTGRAPHIC_CTRL_SET_BUF_FORMAT(RGB565) must come first, otherwise the layer
 *     data_format is 0 and HAL_LCDC_GetPixelSize() asserts;
 *   - YUV->RGB565 uses EPIC hardware by default (falls back to CPU on failure);
 *   - The source buffer is not overwritten by the DMA before bt656_release(),
 *     so "render, then return" is enough; EPIC is an asynchronous DMA, and
 *     show_epic_blit() returns only after the completion callback.
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>

#include "mem_section.h"
#include "bt656.h"
#include "bt656_show.h"
#include "cpu_usage_profiler.h"

/**
 * EPIC hardware color conversion (YUV422 -> RGB565).
 * Same path LVGL uses on the 57x (EPIC_INPUT_YUV422_PACKED_* +
 * EPIC_COLOR_RGB565); the low-level drv_epic_blend() is called directly here.
 */
#if defined(BSP_USING_EPIC) && !defined(DRV_EPIC_NEW_API)
    #define SHOW_EPIC_ENABLE    1
    #include <drv_epic.h>
#else
    #define SHOW_EPIC_ENABLE    0
#endif

#define DBG_TAG "bt656.show"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifndef LCD_HOR_RES_MAX
    #define LCD_HOR_RES_MAX 1024
#endif
#ifndef LCD_VER_RES_MAX
    #define LCD_VER_RES_MAX 600
#endif

#define FB_LINE_BYTES   (LCD_HOR_RES_MAX * 2)
#define FB_BYTES        (FB_LINE_BYTES * LCD_VER_RES_MAX)

/** Full-screen RGB565 canvas (1024*600*2 = 1.2 MB), non-retained PSRAM section */
L2_NON_RET_BSS_SECT(bt656_show, ALIGN(64) static rt_uint8_t show_fb[FB_BYTES]);

static rt_device_t show_lcd;
static struct rt_semaphore show_done;
static rt_bool_t show_inited = RT_FALSE;
static rt_bool_t show_sem_inited = RT_FALSE;

/** show and preview share the same show_fb and LCD message queue, so they must
 *  be serialized */
static struct rt_mutex show_lock;
static rt_bool_t show_lock_inited = RT_FALSE;

static void show_lock_take(void)
{
    if (show_lock_inited)
    {
        rt_mutex_take(&show_lock, RT_WAITING_FOREVER);
    }
}

static void show_lock_release(void)
{
    if (show_lock_inited)
    {
        rt_mutex_release(&show_lock);
    }
}

/** Dirty line range: show_blit() writes only the lines covered by the image each
 *  frame, and the flush writes back only those lines */
static rt_int32_t show_dirty_y0 = -1;
static rt_int32_t show_dirty_y1 = -1;
static rt_int32_t show_flushed_y0 = -1;
static rt_int32_t show_flushed_y1 = -1;

/** Whether the LCDC continuous scan already points at show_fb (established once) */
static rt_bool_t show_attached = RT_FALSE;

/** EPIC hardware conversion switch: falls back to CPU automatically after a failure */
static rt_bool_t show_use_epic = RT_TRUE;

static rt_err_t show_lcd_tx_done(rt_device_t dev, void *buffer)
{
    (void)dev;
    (void)buffer;
    rt_sem_release(&show_done);
    return RT_EOK;
}

static inline __attribute__((always_inline)) rt_uint8_t show_clamp(rt_int32_t v)
{
    return (v < 0) ? 0 : ((v > 255) ? 255 : (rt_uint8_t)v);
}

/** @brief BT.601 limited range.
 *
 * Must be always_inline: with -Oz the compiler turns it into a real function
 * call that runs twice per pixel pair, and 20 ms+ per 720x240 field would be
 * pure call overhead.
 */
static inline __attribute__((always_inline)) rt_uint16_t
show_yuv2rgb565(rt_uint8_t y, rt_uint8_t u, rt_uint8_t v)
{
    rt_int32_t c = (rt_int32_t)y - 16;
    rt_int32_t d = (rt_int32_t)u - 128;
    rt_int32_t e = (rt_int32_t)v - 128;
    rt_int32_t r = (298 * c + 409 * e + 128) >> 8;
    rt_int32_t g = (298 * c - 100 * d - 208 * e + 128) >> 8;
    rt_int32_t b = (298 * c + 516 * d + 128) >> 8;

    r = show_clamp(r);
    g = show_clamp(g);
    b = show_clamp(b);

    return (rt_uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

/** @brief Line layout detection (progressive formats).
 *
 * AHD buffers still carry the BT.656 timing codes, so the real line pitch must
 * be derived from the code spacing. Only the most frequent adjacent spacing
 * ([2*width, 2*width+8]) within the first 64 KB is used, so false codes inside
 * the image cannot skew the result.
 *
 * @return Line pitch; 0 when no consistent structure was found (the caller
 *         falls back to the format stride / base 0).
 */
static rt_int32_t show_detect_layout(const rt_uint8_t *buf, rt_int32_t bytes,
                                     rt_int32_t sw, rt_int32_t *base)
{
    rt_int32_t pos[32];
    rt_int32_t n = 0;
    rt_int32_t i, j;
    rt_int32_t best_gap = 0, best_cnt = 0, best_prev = -1;
    rt_int32_t limit = (bytes < 65536) ? bytes : 65536;
    rt_int32_t lo = sw * 2, hi = sw * 2 + 8;

    *base = 0;

    for (i = 0; (i + 3 < limit) && (n < 32); i++)
    {
        if ((buf[i] == 0xFF) && (buf[i + 1] == 0x00) && (buf[i + 2] == 0x00) &&
                ((buf[i + 3] & 0x80U) != 0U))
        {
            pos[n++] = i;
            i += 3;
        }
    }

    if (n < 3)
    {
        return 0;
    }

    for (i = 1; i < n; i++)
    {
        rt_int32_t gap = pos[i] - pos[i - 1];
        rt_int32_t cnt = 0;

        if ((gap < lo) || (gap > hi))
        {
            continue;
        }
        for (j = 1; j < n; j++)
        {
            if ((pos[j] - pos[j - 1]) == gap)
            {
                cnt++;
            }
        }
        if (cnt > best_cnt)
        {
            best_cnt = cnt;
            best_gap = gap;
            best_prev = i - 1;
        }
    }

    /* Accept only when at least two gaps agree, so false codes inside the image
     * cannot skew the result */
    if ((best_cnt < 2) || (best_prev < 0))
    {
        return 0;
    }

    *base = pos[best_prev] + 4;
    return best_gap;
}

#if SHOW_EPIC_ENABLE
static struct rt_semaphore show_epic_sem;
static rt_bool_t show_epic_sem_inited = RT_FALSE;

static void show_epic_done(EPIC_HandleTypeDef *epic)
{
    (void)epic;
    rt_sem_release(&show_epic_sem);
}

/** @brief drv_epic_blend() (LVGL's YUV image path) composites the source YUV
 *         layer at alpha=255 onto the output canvas.
 * Returns RT_TRUE on success: show_fb was written by EPIC and invalidated.
 */
static rt_bool_t show_epic_blit(const rt_uint8_t *src, rt_int32_t stride,
                                rt_int32_t dx, rt_int32_t dy,
                                rt_int32_t w, rt_int32_t h)
{
    EPIC_LayerConfigTypeDef fg;
    EPIC_LayerConfigTypeDef out;

    if (!show_use_epic)
    {
        return RT_FALSE;
    }
    if ((src == RT_NULL) || (w <= 0) || (h <= 0) || (stride != (w * 2)))
    {
        return RT_FALSE;
    }
    if ((dx < 0) || (dy < 0) ||
            ((dx + w) > LCD_HOR_RES_MAX) || ((dy + h) > LCD_VER_RES_MAX))
    {
        return RT_FALSE;
    }

    if (!show_epic_sem_inited)
    {
        if (rt_sem_init(&show_epic_sem, "ark_epic", 0, RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            return RT_FALSE;
        }
        show_epic_sem_inited = RT_TRUE;
    }
    while (rt_sem_take(&show_epic_sem, 0) == RT_EOK)
    {
    }

    HAL_EPIC_LayerConfigInit(&fg);
    fg.data = (uint8_t *)src;
    fg.color_mode = EPIC_INPUT_YUV422_PACKED_UYVY;
    fg.width = (uint16_t)w;
    fg.height = (uint16_t)h;
    fg.total_width = (uint16_t)w;
    fg.x_offset = 0;
    fg.y_offset = 0;
    fg.alpha = 255;
    fg.yuv.y_buf = (uint8_t *)src;
    fg.data_size = (uint32_t)h * (uint32_t)stride;

    /*
     * Restrict the output canvas to the video rectangle: a full-screen output
     * would overwrite the white border around the video every frame.
     */
    HAL_EPIC_LayerConfigInit(&out);
    out.data = show_fb + ((rt_uint32_t)dy * LCD_HOR_RES_MAX + (rt_uint32_t)dx) * 2U;
    out.color_mode = EPIC_COLOR_RGB565;
    out.width = (uint16_t)w;
    out.height = (uint16_t)h;
    out.total_width = LCD_HOR_RES_MAX;
    out.x_offset = 0;
    out.y_offset = 0;
    out.data_size = (uint32_t)LCD_HOR_RES_MAX * (uint32_t)h * 2U;
    out.color_en = RT_FALSE;

    if (drv_epic_blend(&fg, 1, &out, show_epic_done) != RT_EOK)
    {
        LOG_W("epic blend: start failed, fallback to cpu");
        show_use_epic = RT_FALSE;
        return RT_FALSE;
    }
    if (rt_sem_take(&show_epic_sem, rt_tick_from_millisecond(100)) != RT_EOK)
    {
        /* Do not call drv_epic_wait_done: it asserts in the debug build */
        LOG_W("epic blend: timeout, fallback to cpu");
        show_use_epic = RT_FALSE;
        return RT_FALSE;
    }

    /* EPIC writes show_fb through DMA; the CPU must drop the stale cache lines
     * for these rows */
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_InvalidateDCache_by_Addr((uint32_t *)(show_fb + dy * FB_LINE_BYTES),
                                 (int32_t)(h * FB_LINE_BYTES));
#endif
    __DSB();
    return RT_TRUE;
}
#else
static rt_bool_t show_epic_blit(const rt_uint8_t *src, rt_int32_t stride,
                                rt_int32_t dx, rt_int32_t dy,
                                rt_int32_t w, rt_int32_t h)
{
    (void)src;
    (void)stride;
    (void)dx;
    (void)dy;
    (void)w;
    (void)h;
    return RT_FALSE;
}
#endif /* SHOW_EPIC_ENABLE */

/** @brief YUV422(UYVY) -> RGB565, centered on the full-screen canvas, no scaling:
 *   source smaller than the panel -> white border around it;
 *   source larger than the panel  -> center crop, the overflow is dropped.
 *
 * Returns RT_TRUE when EPIC did the conversion (no cache write-back needed);
 * RT_FALSE means the CPU path ran and the line range written this frame was
 * recorded.
 */
static rt_bool_t show_blit(const struct bt656_frame *frame)
{
    rt_int32_t sw = frame->width;
    rt_int32_t sh = frame->height;
    const rt_uint8_t *src = frame->data;
    rt_int32_t stride = frame->stride;
    rt_int32_t src_bytes = (rt_int32_t)frame->bytes;
    rt_int32_t code_base = 0;
    rt_int32_t x0, y0, y;

    show_dirty_y0 = -1;
    show_dirty_y1 = -1;

    if ((sw <= 0) || (sh <= 0) || (frame->stride < sw * 2) ||
            (frame->data == RT_NULL))
    {
        return RT_FALSE;
    }

    /*
     * Safety net: no path may read more lines than the bytes actually received
     * for this frame, otherwise it would read past the buffer.
     */
    if ((stride > 0) && (src_bytes > 0))
    {
        rt_int32_t avail = src_bytes / stride;

        if (avail <= 0)
        {
            /* Nothing complete enough to render: keep the previous picture */
            return RT_FALSE;
        }
        sh = (sh > avail) ? avail : sh;
    }

    /*
     * AHD (progressive) buffers still carry the BT.656 timing codes, so the
     * line pitch is detected from the code spacing. CVBS has its SAV/EAV
     * stripped by the FSM and keeps the stride from the format table.
     */
    if (frame->field == 0U)
    {
        rt_int32_t code_stride = show_detect_layout(frame->data, src_bytes,
                                 sw, &code_base);

        if (code_stride > 0)
        {
            rt_int32_t avail = (src_bytes - code_base) / code_stride;

            if (avail <= 0)
            {
                /* Nothing complete enough to render: keep the previous picture */
                return RT_FALSE;
            }

            src = frame->data + code_base;
            stride = code_stride;
            sh = (sh > avail) ? avail : sh;
        }
    }

    x0 = (LCD_HOR_RES_MAX - sw) / 2;
    y0 = (LCD_VER_RES_MAX - sh) / 2;
    x0 &= ~1;   /* Start on an even byte for YUV422 */

    /* Prefer EPIC hardware color conversion (full frame, 1:1, no cropping) */
    if (show_epic_blit(src, stride, x0, y0, sw, sh))
    {
        return RT_TRUE;
    }

    for (y = 0; y < sh; y++)
    {
        rt_int32_t dy = y0 + y;
        rt_int32_t sx, dx, n, i;
        const rt_uint8_t *s;
        rt_uint16_t *d;

        if ((dy < 0) || (dy >= LCD_VER_RES_MAX))
        {
            continue;
        }

        /* Horizontal cropping */
        sx = (x0 < 0) ? -x0 : 0;
        sx &= ~1;
        dx = x0 + sx;
        n = sw - sx;
        if (dx + n > LCD_HOR_RES_MAX)
        {
            n = LCD_HOR_RES_MAX - dx;
        }
        if (n <= 0)
        {
            continue;
        }

        s = src + y * stride + sx * 2;
        d = (rt_uint16_t *)(show_fb + dy * FB_LINE_BYTES) + dx;

        for (i = 0; i + 1 < n; i += 2)
        {
            rt_uint8_t uu = s[0];
            rt_uint8_t ya = s[1];
            rt_uint8_t vv = s[2];
            rt_uint8_t yb = s[3];

            d[i]     = show_yuv2rgb565(ya, uu, vv);
            d[i + 1] = show_yuv2rgb565(yb, uu, vv);
            s += 4;
        }

        if (show_dirty_y0 < 0)
        {
            show_dirty_y0 = dy;
        }
        show_dirty_y1 = dy;
    }

    return RT_FALSE;
}

/** @brief Fill the whole canvas with white: the panel scans show_fb by itself,
 *         so clearing the cache after changing the content is enough.
 */
static void show_canvas_white(void)
{
    memset(show_fb, 0xFF, sizeof(show_fb));
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_CleanDCache_by_Addr((uint32_t *)show_fb, (int32_t)sizeof(show_fb));
#endif
    __DSB();

    show_dirty_y0 = -1;
    show_dirty_y1 = -1;
    show_flushed_y0 = 0;
    show_flushed_y1 = LCD_VER_RES_MAX - 1;
}

/** @brief Write back the line range touched by this frame (and the range touched
 *         by the previous frame) to PSRAM.
 *
 * The panel continuously scans show_fb through LCDC/PTC; pixels written by the
 * CPU into the cache never reach the screen unless they are cleaned.
 */
static void show_fb_flush(void)
{
    rt_int32_t y0 = show_dirty_y0;
    rt_int32_t y1 = show_dirty_y1;

    if (y0 < 0)
    {
        return;
    }
    if (show_flushed_y0 >= 0)
    {
        if (show_flushed_y0 < y0)
        {
            y0 = show_flushed_y0;
        }
        if (show_flushed_y1 > y1)
        {
            y1 = show_flushed_y1;
        }
    }
    if (y0 < 0)
    {
        y0 = 0;
    }
    if (y1 > LCD_VER_RES_MAX - 1)
    {
        y1 = LCD_VER_RES_MAX - 1;
    }

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_CleanDCache_by_Addr((uint32_t *)(show_fb + y0 * FB_LINE_BYTES),
                            (int32_t)((y1 - y0 + 1) * FB_LINE_BYTES));
#endif
    __DSB();

    show_flushed_y0 = show_dirty_y0;
    show_flushed_y1 = show_dirty_y1;
}

static int show_wait_lcd_ready(void)
{
    struct rt_device_graphic_info info;
    int i;

    for (i = 0; i < 60; i++)    /* Up to 3 s */
    {
        memset(&info, 0, sizeof(info));
        if ((rt_device_control(show_lcd, RTGRAPHIC_CTRL_GET_INFO, &info) == RT_EOK) &&
                (info.width > 0) && (info.bits_per_pixel > 0))
        {
            LOG_I("lcd ready: %dx%d %dbpp", info.width, info.height, info.bits_per_pixel);
            return RT_EOK;
        }
        rt_thread_mdelay(50);
    }

    LOG_E("lcd not ready in 3s");
    return -RT_ETIMEOUT;
}

static int show_init(void)
{
    rt_err_t err;
    rt_uint16_t fmt = RTGRAPHIC_PIXEL_FORMAT_RGB565;

    /*
     * Initialize the semaphore/mutex only once: show_init() may be called
     * repeatedly while the LCD is not ready yet, and re-initializing a kernel
     * object corrupts it -> hardfault.
     */
    if (!show_lock_inited)
    {
        rt_mutex_init(&show_lock, "ark_show", RT_IPC_FLAG_PRIO);
        show_lock_inited = RT_TRUE;
    }
    if (!show_sem_inited)
    {
        err = rt_sem_init(&show_done, "ark_show", 0, RT_IPC_FLAG_FIFO);
        if (err != RT_EOK)
        {
            return err;
        }
        show_sem_inited = RT_TRUE;
    }

    show_lcd = rt_device_find("lcd");
    if (show_lcd == RT_NULL)
    {
        LOG_E("can't find lcd device");
        return -RT_EIO;
    }

    if (rt_device_open(show_lcd, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        LOG_W("lcd open busy, continue");
    }
    rt_device_set_tx_complete(show_lcd, show_lcd_tx_done);

    err = show_wait_lcd_ready();
    if (err != RT_EOK)
    {
        return err;
    }

    /* Tell drv_lcd that RGB565 is sent in -- it calls HAL_LCDC_LayerSetFormat()
     * internally */
    rt_device_control(show_lcd, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &fmt);

    show_inited = RT_TRUE;
    LOG_I("ready: lcd %dx%d", LCD_HOR_RES_MAX, LCD_VER_RES_MAX);
    return RT_EOK;
}

/** @brief Point the LCDC continuous scan start at show_fb -- only once per boot.
 *
 * This is a RAMLESS DPI panel: the HAL uses the PTC state machine to move the
 * full-screen data from PSRAM into the SRAM line buffer block by block and
 * keeps scanning once it is set up. So after the first draw_rect, updating
 * show_fb and writing back the cache is enough to show new content.
 *
 * The caller must already hold show_lock.
 */
static int show_attach_locked(void)
{
    const struct rt_device_graphic_ops *ops;

    if (show_attached)
    {
        return RT_EOK;
    }

    ops = rt_graphix_ops(show_lcd);
    if ((ops == RT_NULL) || (ops->set_window == RT_NULL) ||
            ((ops->draw_rect == RT_NULL) && (ops->draw_rect_async == RT_NULL)))
    {
        LOG_E("lcd graphic ops not supported");
        return -RT_ENOSYS;
    }

    /* White background: RGB565 0xFFFF, memset 0xFF byte-wise */
    memset(show_fb, 0xFF, sizeof(show_fb));
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_CleanDCache_by_Addr((uint32_t *)show_fb, (int32_t)sizeof(show_fb));
#endif
    __DSB();

    while (rt_sem_take(&show_done, 0) == RT_EOK)
    {
    }

    ops->set_window(0, 0, LCD_HOR_RES_MAX - 1, LCD_VER_RES_MAX - 1);
    if (ops->draw_rect != RT_NULL)
    {
        ops->draw_rect((const char *)show_fb,
                       0, 0, LCD_HOR_RES_MAX - 1, LCD_VER_RES_MAX - 1);
    }
    else
    {
        ops->draw_rect_async((const char *)show_fb,
                             0, 0, LCD_HOR_RES_MAX - 1, LCD_VER_RES_MAX - 1);
        if (rt_sem_take(&show_done, rt_tick_from_millisecond(1000)) != RT_EOK)
        {
            LOG_W("lcd attach: tx timeout");
        }
    }

    show_dirty_y0 = -1;
    show_dirty_y1 = -1;
    show_flushed_y0 = -1;
    show_flushed_y1 = -1;
    show_attached = RT_TRUE;
    LOG_I("lcd attached: panel now scans show_fb continuously");
    return RT_EOK;
}

static int show_render(const struct bt656_frame *frame)
{
    if (!show_blit(frame))
    {
        /* CPU path: write back only the lines covered by the image; the panel
         * is self-scanned by the PTC state machine */
        show_fb_flush();
    }
    /* The EPIC path already wrote show_fb by DMA and invalidated the cache, so
     * no write-back is needed */
    return RT_EOK;
}

/** @brief Capture and show one frame (snapshot).
 * bt656_init() + bt656_start() must already have been called.
 */
int camera_show_once(rt_int32_t timeout_ms)
{
    struct bt656_frame frame;
    int ret;

    if (!show_inited)
    {
        ret = show_init();
        if (ret != RT_EOK)
        {
            return ret;
        }
    }

    /* show and preview are mutually exclusive: the same show_fb / LCD message
     * queue cannot be used concurrently */
    show_lock_take();

    ret = show_attach_locked();
    if (ret == RT_EOK)
    {
        ret = bt656_get(&frame, (timeout_ms > 0) ? timeout_ms : 1000);
        if (ret == RT_EOK)
        {
            LOG_D("show %dx%d stride=%d bytes=%u",
                  frame.width, frame.height, frame.stride,
                  (unsigned int)frame.bytes);
            ret = show_render(&frame);
        }
        bt656_release(&frame);
    }

    show_lock_release();
    return ret;
}

/* -------------------------------------------------------------------------- */
/* Continuous preview: one thread doing get -> display -> release */
/* -------------------------------------------------------------------------- */

/** @brief Convert a tick delta to milliseconds */
#define PREVIEW_TICKS_TO_MS(ticks)  \
    ((rt_uint32_t)(((rt_uint64_t)(ticks) * 1000U) / RT_TICK_PER_SECOND))

/** @brief Single capture timeout */
#define PREVIEW_WAIT_MS     500
/** @brief Period for printing fps while running */
#define PREVIEW_STAT_MS     2000
/** @brief This many consecutive failures is treated as "no signal"; a hint is
 *         printed but the loop keeps running */
#define PREVIEW_FAIL_LOG    50

static volatile rt_bool_t preview_run = RT_FALSE;
static rt_thread_t preview_thread = RT_NULL;

static camera_preview_mode_t preview_mode = BT656_PREVIEW_FRAME;
static rt_int32_t preview_interval;

static volatile rt_uint32_t preview_ok;
static volatile rt_uint32_t preview_fail;
static volatile rt_uint32_t preview_ms_sum;
static volatile rt_uint32_t preview_ms_max;

/** @brief Capture thread: take a frame -> render (show_render returns after EPIC
 *         completed) -> return it.
 *
 * The driver will not hand this buffer back to the DMA before it is returned,
 * so no extra copy and no second thread are needed. When display is slower than
 * capture the driver simply drops new frames (counted in dropped), so there is
 * no backlog in memory.
 */
static void preview_entry(void *parameter)
{
    rt_tick_t t_start = rt_tick_get();
    rt_tick_t t_stat = t_start;
    rt_uint32_t fail_run = 0U;

    (void)parameter;

    while (preview_run)
    {
        struct bt656_frame frame;
        rt_tick_t t0 = rt_tick_get();
        rt_uint32_t dt;
        int ret;

        ret = bt656_get(&frame, PREVIEW_WAIT_MS);
        if (ret == RT_EOK)
        {
            show_lock_take();
            (void)show_render(&frame);
            show_lock_release();

            bt656_release(&frame);

            dt = PREVIEW_TICKS_TO_MS(rt_tick_get() - t0);
            preview_ok++;
            preview_ms_sum += dt;
            if (dt > preview_ms_max)
            {
                preview_ms_max = dt;
            }
            fail_run = 0U;

            if (preview_interval > 0)
            {
                /* Sleep in slices so a long interval still reacts quickly to
                 * `camera stop` */
                rt_int32_t wait = preview_interval;

                while ((wait > 0) && preview_run)
                {
                    rt_int32_t slice = (wait > 20) ? 20 : wait;

                    rt_thread_mdelay(slice);
                    wait -= slice;
                }
            }
        }
        else
        {
            preview_fail++;
            /*
             * Failures must yield the CPU: back off longer as the failure run
             * grows.
             */
            if (++fail_run == PREVIEW_FAIL_LOG)
            {
                rt_kprintf("[camera] %u captures failed (ret=%d), check signal\n",
                           (unsigned int)fail_run, ret);
            }
            rt_thread_mdelay((fail_run >= PREVIEW_FAIL_LOG) ? 50 : 5);
        }

        if ((rt_tick_get() - t_stat) >= rt_tick_from_millisecond(PREVIEW_STAT_MS))
        {
            rt_int32_t ms = (rt_int32_t)PREVIEW_TICKS_TO_MS(rt_tick_get() - t_start);
            rt_uint32_t ok = preview_ok;

            if (ms <= 0)
            {
                ms = 1;
            }
#if defined(USING_CPU_USAGE_PROFILER)
            rt_kprintf("[camera] %.1f fps, %u ms/frame, fail=%u, cpu %.1f%%\n",
                       (double)ok * 1000.0 / (double)ms,
                       (unsigned int)((ok > 0U) ? (preview_ms_sum / ok) : 0U),
                       (unsigned int)preview_fail,
                       (double)cpu_get_usage());
#else
            rt_kprintf("[camera] %.1f fps, %u ms/frame, fail=%u\n",
                       (double)ok * 1000.0 / (double)ms,
                       (unsigned int)((ok > 0U) ? (preview_ms_sum / ok) : 0U),
                       (unsigned int)preview_fail);
#endif
            t_stat = rt_tick_get();
        }
    }

    {
        rt_int32_t ms = (rt_int32_t)PREVIEW_TICKS_TO_MS(rt_tick_get() - t_start);
        rt_uint32_t ok = preview_ok;

        if (ms <= 0)
        {
            ms = 1;
        }
        rt_kprintf("[camera] stopped: %d ms, ok=%u fail=%u, %.1f fps, %u ms/frame\n",
                   (int)ms, (unsigned int)ok, (unsigned int)preview_fail,
                   (double)ok * 1000.0 / (double)ms,
                   (unsigned int)((ok > 0U) ? (preview_ms_sum / ok) : 0U));
    }

    /* Safety net: if the thread exits without camera_preview_stop(), do not
     * leave the DCMI running */
    (void)bt656_stop();

    preview_thread = RT_NULL;
    /* Return directly: the thread entry's LR is rt_thread_exit, so the kernel
     * reclaims the thread normally */
}

static int camera_preview_start_internal(camera_preview_mode_t mode,
        rt_int32_t interval_ms)
{
    const struct bt656_format *info;
    int ret;

    if (preview_run || (preview_thread != RT_NULL))
    {
        return -RT_EBUSY;
    }
    if (interval_ms < 0)
    {
        interval_ms = 0;
    }

    if (!show_inited)
    {
        ret = show_init();
        if (ret != RT_EOK)
        {
            return ret;
        }
    }

    /* Snapshot, frame and stream all use the same stream capture path */
    ret = bt656_init();
    if (ret != RT_EOK)
    {
        rt_kprintf("[camera] capture init failed: %d\n", ret);
        return ret;
    }

    info = bt656_get_format();
    if (info == RT_NULL)
    {
        return -RT_EINVAL;
    }

    ret = bt656_start();
    if (ret != RT_EOK)
    {
        rt_kprintf("[camera] capture start failed: %d\n", ret);
        return ret;
    }

    /* Point the LCDC scan start at show_fb for the first time (the panel then
     * refreshes itself, no more draw calls) */
    show_lock_take();
    ret = show_attach_locked();
    if (ret == RT_EOK)
    {
        show_canvas_white();
    }
    show_lock_release();
    if (ret != RT_EOK)
    {
        (void)bt656_stop();
        return ret;
    }

    preview_ok = 0U;
    preview_fail = 0U;
    preview_ms_sum = 0U;
    preview_ms_max = 0U;
    preview_mode = mode;
    preview_interval = interval_ms;
    preview_run = RT_TRUE;

    preview_thread = rt_thread_create("ark_prev", preview_entry,
                                      RT_NULL, 4096, 21, 10);
    if (preview_thread == RT_NULL)
    {
        preview_run = RT_FALSE;
        (void)bt656_stop();
        return -RT_ENOMEM;
    }
    rt_thread_startup(preview_thread);

    rt_kprintf("[camera] %s started: %s, interval %d ms\n",
               (mode == BT656_PREVIEW_STREAM) ? "stream" : "frame",
               info->name, (int)interval_ms);
    return RT_EOK;
}

int camera_preview_start(camera_preview_mode_t mode, rt_int32_t interval_ms)
{
    return camera_preview_start_internal(mode, interval_ms);
}

int camera_preview_stop(void)
{
    int i;

    if (!preview_run && (preview_thread == RT_NULL))
    {
        return RT_EOK;
    }

    preview_run = RT_FALSE;

    /* The capture thread waits at most 500 ms in get(); 3 s is plenty */
    for (i = 0; (i < 60) && (preview_thread != RT_NULL); i++)
    {
        rt_thread_mdelay(50);
    }
    if (preview_thread != RT_NULL)
    {
        rt_kprintf("[camera] thread did not exit in 3s\n");
        return -RT_ETIMEOUT;
    }

    (void)bt656_stop();
    return RT_EOK;
}

rt_bool_t camera_preview_running(void)
{
    return (preview_run || (preview_thread != RT_NULL)) ? RT_TRUE : RT_FALSE;
}

camera_preview_mode_t camera_preview_mode(void)
{
    return preview_mode;
}

void camera_preview_stats(rt_uint32_t *frames, rt_uint32_t *fails)
{
    if (frames != RT_NULL)
    {
        *frames = preview_ok;
    }
    if (fails != RT_NULL)
    {
        *fails = preview_fail;
    }
}
