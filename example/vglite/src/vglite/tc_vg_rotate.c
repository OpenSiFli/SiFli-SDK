/*
 Resolution: 320 x 480
 Format: VG_LITE_RGB565
 Transformation: Rotate/Scale/Translate
 Alpha Blending: None
 Related APIs: vg_lite_clear/vg_lite_translate/vg_lite_scale/vg_lite_rotate/vg_lite_blit
 Description: Load outside landscape image, then blit it to blue dest buffer with rotate/scale/translate.
 Image filter type is selected by hardware feature gcFEATURE_BIT_VG_IM_FILTER(ON: VG_LITE_FILTER_BI_LINEAR, OFF: VG_LITE_FILTER_POINT).
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "vg_lite.h"
#include "vg_lite_ex.h"
#include "vg_lite_util.h"
#include "tc_vglite.h"

#define DEFAULT_SIZE   224.0f;

static int   fb_width = 224, fb_height = 240;
static float fb_scale = 1.0f;

static vg_lite_buffer_t buffer;     //offscreen framebuffer object for rendering.
static vg_lite_buffer_t *fb;

static vg_lite_buffer_t image;

#define IMG_WIDTH   (16)
#define IMG_HEIGHT  (198)


ALIGN(64)
static uint8_t img_data[] =
{
#include "second_hand_1.dat"
};

static void cleanup(void)
{
    if (buffer.handle != NULL)
    {
        // Free the buffer memory.
        vg_lite_free(&buffer);
    }

    if (image.handle != NULL)
    {
        // Free the image memory.
        vg_lite_free(&image);
    }

    vg_lite_close();
}

void tc_vglite_rotate(int argc, char **argv)
{
    vg_lite_filter_t filter;
    vg_lite_matrix_t matrix;

    // Initialize vg_lite engine.
    vg_lite_error_t error = VG_LITE_SUCCESS;
    TC_VGLITE_CHECK_ERROR(vg_lite_set_command_buffer_size(16 << 10));
    TC_VGLITE_CHECK_ERROR(vg_lite_init(fb_width, fb_height));

    // Set image filter type according to hardware feature.
    filter = VG_LITE_FILTER_BI_LINEAR;

    TC_VGLITE_CHECK_ERROR(vg_lite_init_buf(&image, IMG_WIDTH, IMG_HEIGHT, VG_LITE_BGRA8888, (void *)img_data));

    fb_scale = (float)fb_width / DEFAULT_SIZE;
    printf("Framebuffer size: %d x %d\n", fb_width, fb_height);

    // Allocate the off-screen buffer.
    buffer.width  = fb_width;
    buffer.height = fb_height;
    buffer.format = VG_LITE_BGR565;

    TC_VGLITE_CHECK_ERROR(vg_lite_allocate(&buffer));
    fb = &buffer;

    for (uint32_t i = 1; i < 3600; i++)
    {
        //uint32_t i = 3;

        rt_kprintf("rot:%d\n", i);
        // Clear the buffer with blue.
        TC_VGLITE_CHECK_ERROR(vg_lite_clear(fb, NULL, 0xFF000000));
        // Build a 33 degree rotation matrix from the center of the buffer.
        vg_lite_identity(&matrix);
        // Translate the matrix to the center of the buffer.
        //vg_lite_translate((fb_width - IMG_WIDTH) / 2.0f, (fb_height - IMG_HEIGHT) / 2.0f, &matrix);
        vg_lite_translate(fb_width / 2.0f, fb_height / 2.0f, &matrix);
        //vg_lite_translate(IMG_WIDTH / 2.0f, IMG_HEIGHT / 2.0f, &matrix);
        // Do a 33 degree rotation.
        vg_lite_rotate(i * 1.0f, &matrix);
        // Translate the matrix again to adjust matrix location.
        vg_lite_translate(IMG_WIDTH / -2.0f, IMG_HEIGHT / -2.0f, &matrix);
        //vg_lite_translate(IMG_WIDTH / -2.0f, IMG_HEIGHT / -2.0f, &matrix);

        // Scale matrix according to buffer size.
        //vg_lite_scale((vg_lite_float_t) fb_width / (vg_lite_float_t) image.width,
        //              (vg_lite_float_t) fb_height / (vg_lite_float_t) image.height, &matrix);

        // Blit the image using the matrix.
        //TC_VGLITE_CHECK_ERROR(vg_lite_blit(fb, &image, &matrix, VG_LITE_BLEND_NONE, 0, filter));
        TC_VGLITE_CHECK_ERROR(vg_lite_blit(fb, &image, &matrix, VG_LITE_BLEND_SRC_OVER, 0, filter));
        //TC_VGLITE_CHECK_ERROR(vg_lite_blit(fb, &image, NULL, VG_LITE_BLEND_SRC_OVER, 0, filter));
        TC_VGLITE_CHECK_ERROR(vg_lite_finish());
        // Save PNG file.
        //vg_lite_save_png("rotate.png", fb);

        tc_vg_send_data_to_lcd(fb->memory, fb->width, fb->height, RTGRAPHIC_PIXEL_FORMAT_RGB565);
        rt_thread_mdelay(160);

    }
ErrorHandler:
    // Cleanup.
    cleanup();
}

UTEST_TC_EXPORT(tc_vglite_rotate, "tc_vglite_0_2", tc_vglite_init, tc_vglite_cleanup, 10);