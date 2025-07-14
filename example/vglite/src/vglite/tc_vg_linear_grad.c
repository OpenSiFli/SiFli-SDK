
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "vg_lite.h"
#include "vg_lite_util.h"
#include "utest.h"
#include "tc_vglite.h"

#define DEFAULT_SIZE   320.0f;
#define DEFAULT_WIDTH 192
#define DEFAULT_HEIGHT 240

static int   fb_width = DEFAULT_WIDTH, fb_height = DEFAULT_HEIGHT;

static vg_lite_buffer_t buffer;     //offscreen framebuffer object for rendering.
static vg_lite_buffer_t *fb;
static vg_lite_buffer_t image;
int frames = 1; //Frames to render
/*
*-----*
/       \
/         \
*           *
|          /
|         X
|          \
*           *
\         /
\       /
*-----*
*/
static char path_data[] =
{
    2, 0, 0, // moveto   -5,-10
    4, 100, 0,  // lineto    5,-10
    4, 70, 50,  // lineto    5,-10
    4, 100, 100,  // lineto   10, -5
    4, 0, 100,    // lineto    0,  0
    4, 40, 80,  // lineto    5,-10
    4, 0, 0,   // lineto   10,  5
    0, // end
};

static vg_lite_path_t path =
{
    {
        0,   0, // left,top
        100, 100
    }, // right,bottom
    VG_LITE_HIGH, // quality
    VG_LITE_S8, // -128 to 127 coordinate range
    {0}, // uploaded
    sizeof(path_data), // path length
    path_data, // path data
    1
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



void tc_vglite_linear_grad(int argc, char **argv)
{
    vg_lite_filter_t filter;
    vg_lite_linear_gradient_t grad;
    uint32_t ramps[] = {0xff000000, 0xffff0000, 0xff00ff00, 0xff0000ff, 0xffffffff};
    uint32_t stops[] = {0, 66, 122, 200, 255};
    vg_lite_matrix_t *matGrad;
    vg_lite_matrix_t matPath;
    int fcount = 0;
    rt_device_t lcd_device;
    uint16_t cf = RTGRAPHIC_PIXEL_FORMAT_RGB565;

    lcd_device = rt_device_find("lcd");

    RT_ASSERT(lcd_device);

    rt_device_open(lcd_device, RT_DEVICE_OFLAG_RDWR);


    /* Initialize vg_lite engine. */
    vg_lite_error_t error = VG_LITE_SUCCESS;
    TC_VGLITE_CHECK_ERROR(vg_lite_set_command_buffer_size(16 << 10));
    TC_VGLITE_CHECK_ERROR(vg_lite_init(fb_width, fb_height));

    filter = VG_LITE_FILTER_POINT;

    /* Allocate the off-screen buffer. */
    buffer.width  = fb_width;
    buffer.height = fb_height;
    buffer.format = VG_LITE_RGB565;

    TC_VGLITE_CHECK_ERROR(vg_lite_allocate(&buffer));
    fb = &buffer;

    printf("Render size: %d x %d, %x\n", fb_width, fb_height, buffer.memory);
    frames = 10;
    while (frames > 0)
    {
        memset(&grad, 0, sizeof(grad));
        if (VG_LITE_SUCCESS != vg_lite_init_grad(&grad))
        {
            printf("Linear gradient is not supported.\n");
            vg_lite_close();
            TC_VGLITE_CHECK_ERROR(false);
        }

        vg_lite_set_grad(&grad, 5, ramps, stops);
        vg_lite_update_grad(&grad);
        matGrad = vg_lite_get_grad_matrix(&grad);
        vg_lite_identity(matGrad);
        //vg_lite_rotate(30.0f * fcount, matGrad);
        //vg_lite_rotate(30.0f , matGrad);

        // Clear the buffer with blue.
        TC_VGLITE_CHECK_ERROR(vg_lite_clear(fb, NULL, 0xffaabbcc));
        vg_lite_identity(&matPath);
        vg_lite_scale(2.0f, 2.0f, &matPath);

        // Fill the path using an image.
        TC_VGLITE_CHECK_ERROR(vg_lite_draw_gradient(fb, &path, VG_LITE_FILL_EVEN_ODD, &matPath, &grad, VG_LITE_BLEND_NONE));
        //TC_VGLITE_CHECK_ERROR(vg_lite_draw(fb, &path, VG_LITE_FILL_EVEN_ODD, &matPath, VG_LITE_BLEND_NONE, 0xff00));
        TC_VGLITE_CHECK_ERROR(vg_lite_finish());
        vg_lite_clear_grad(&grad);
        frames--;
        printf("frame %d done\n", fcount++);
        rt_graphix_ops(lcd_device)->set_window(0, 0, fb_width - 1, fb_height - 1);
        rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &cf);
        rt_graphix_ops(lcd_device)->draw_rect((const char *)fb->memory, 0, 0, fb_width - 1, fb_height - 1);
        rt_thread_mdelay(2000);
    }

    // Save PNG file.
    //vg_lite_save_png("linearGrad.png", fb);

ErrorHandler:
    // Cleanup.
    cleanup();
    rt_device_close(lcd_device);
}

UTEST_TC_EXPORT(tc_vglite_linear_grad, "tc_vglite_0_0", tc_vglite_init, tc_vglite_cleanup, 10);
