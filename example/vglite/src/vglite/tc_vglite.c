
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <rtthread.h>
#include "vg_lite.h"
#include "utest.h"
#include "board.h"
#include "vg_lite_platform.h"

static char *tc_vglite_error_type[] =
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

#define TC_VGLITE_ERROR_TYPE_NUM   (sizeof(tc_vglite_error_type) / sizeof(tc_vglite_error_type[0]))
#define TC_VGLITE_MEM_SIZE   (512 * 1024)

static rt_device_t lcd_device;
static void *tc_vglite_buf;

void tc_vg_send_data_to_lcd(uint8_t *data, uint32_t width, uint32_t height, uint16_t color_fmt)
{
    rt_err_t err;
    struct rt_device_graphic_info lcd_info;
    int tl_x, tl_y;

    err = rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_INFO, &lcd_info);
    RT_ASSERT(RT_EOK == err);

    tl_x = (lcd_info.width - width) >> 1;
    tl_y = (lcd_info.height - height) >> 1;
    tl_x = RT_ALIGN_DOWN(tl_x, lcd_info.draw_align);
    tl_y = RT_ALIGN_DOWN(tl_y, lcd_info.draw_align);


    rt_graphix_ops(lcd_device)->set_window(tl_x, tl_y, tl_x + width - 1, tl_y + height - 1);
    rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &color_fmt);

    rt_graphix_ops(lcd_device)->draw_rect((const char *)data, tl_x, tl_y, tl_x + width - 1, tl_y + height - 1);
}

void tc_vglite_print_error(const char *func, size_t line, vg_lite_error_t err)
{
    if (err < TC_VGLITE_ERROR_TYPE_NUM)
    {
        LOG_I("[%s: %d] failed.error type: %s\n", func, line, tc_vglite_error_type[err]);
    }
    else
    {
        LOG_I("[%s: %d] failed.unknown error type: %d\n", func, line, err);
    }
}

rt_err_t tc_vglite_init(void)
{
    rt_err_t err;
    vg_module_parameters_t param;

    lcd_device = rt_device_find("lcd");
    RT_ASSERT(lcd_device);

    rt_device_open(lcd_device, RT_DEVICE_OFLAG_RDWR);

    tc_vglite_buf = rt_malloc(TC_VGLITE_MEM_SIZE);
    RT_ASSERT(tc_vglite_buf);
    memset((void *)&param, 0, sizeof(param));
    param.register_mem_base = V2D_GPU_BASE;
    param.gpu_mem_base[0] = 0;
    param.contiguous_mem_base[0] = tc_vglite_buf;
    param.contiguous_mem_size[0] = TC_VGLITE_MEM_SIZE;
    vg_lite_init_mem(&param);

    return RT_EOK;
}

rt_err_t tc_vglite_cleanup(void)
{
    vg_module_parameters_t param;

    rt_device_close(lcd_device);
    lcd_device = NULL;

    if (tc_vglite_buf)
    {
        rt_free(tc_vglite_buf);
        tc_vglite_buf = NULL;
        memset((void *)&param, 0, sizeof(param));
        param.register_mem_base = V2D_GPU_BASE;
        param.gpu_mem_base[0] = 0;
        param.contiguous_mem_base[0] = 0;
        param.contiguous_mem_size[0] = 0;
        vg_lite_init_mem(&param);
    }

    return RT_EOK;
}

const char *tc_vglite_get_buf_fmt_str(vg_lite_buffer_format_t buf_format)
{
    const char *s;

    if (VG_LITE_BGRA5658 == buf_format)
    {
        s = "BGRA5658";
    }
    else if (VG_LITE_BGRA8888 == buf_format)
    {
        s = "BGRA8888";
    }
    else if (VG_LITE_BGR565 == buf_format)
    {
        s = "BGR565";
    }
    else if (VG_LITE_BGR888 == buf_format)
    {
        s = "BGR888";
    }
    else if (VG_LITE_RGBA8888_ETC2_EAC == buf_format)
    {
        s = "ETC2";
    }
    else
    {
        s = "UNK";
    }

    return s;
}

const char *tc_vglite_get_blend_mode_str(vg_lite_blend_t blend_mode)
{
    const char *s;

    if (VG_LITE_BLEND_SRC_OVER == blend_mode)
    {
        s = "SRC_OVER";
    }
    else if (VG_LITE_BLEND_NONE == blend_mode)
    {
        s = "NONE";
    }
    else
    {
        s = "UNK";
    }

    return s;
}

const char *tc_vglite_get_filter_str(vg_lite_filter_t filter)
{
    const char *s;

    if (VG_LITE_FILTER_POINT == filter)
    {
        s = "POINT";
    }
    else if (VG_LITE_FILTER_BI_LINEAR == filter)
    {
        s = "BI_LINEAR";
    }
    else
    {
        s = "UNK";
    }

    return s;
}

const char *tc_vglite_get_layout_str(vg_lite_buffer_layout_t layout)
{
    const char *s;

    if (VG_LITE_LINEAR == layout)
    {
        s = "LINEAR";
    }
    else if (VG_LITE_TILED == layout)
    {
        s = "TILED";
    }
    else
    {
        s = "UNK";
    }

    return s;
}

