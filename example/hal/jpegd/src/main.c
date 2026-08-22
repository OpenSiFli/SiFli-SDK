/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"
#include "bf0_hal_jpegd.h"
#include "rtthread.h"

#ifdef JPEGD_USING_LCD
    #include "rtdevice.h"
#endif

#include <stdint.h>
#include <string.h>

#if !defined(SF32LB57X)
    #error "The JPEGD example supports SF32LB57X only."
#endif

#define JPEGD_WORK_CAPACITY 5376U
#define JPEGD_Y_CAPACITY    12544U
#define JPEGD_U_CAPACITY    3136U
#define JPEGD_V_CAPACITY    3136U

#ifdef JPEGD_USING_LCD
    #define JPEGD_RGB565_CAPACITY (100U * 100U * 2U)
#endif

ALIGN(32)
static const uint8_t g_jpeg_data[] =
{
#include "../assets/100x100_jpeg.dat"
};

ALIGN(32)
static uint8_t g_work_buffer[JPEGD_WORK_CAPACITY];
ALIGN(32)
static uint8_t g_output_y[JPEGD_Y_CAPACITY];
ALIGN(32)
static uint8_t g_output_u[JPEGD_U_CAPACITY];
ALIGN(32)
static uint8_t g_output_v[JPEGD_V_CAPACITY];

#ifdef JPEGD_USING_LCD
ALIGN(32)
static uint16_t g_rgb565_buffer[JPEGD_RGB565_CAPACITY / sizeof(uint16_t)];

static uint8_t clamp_color(int32_t color)
{
    if (color < 0)
    {
        return 0;
    }
    if (color > 255)
    {
        return 255;
    }

    return (uint8_t)color;
}

static void yuv420_to_rgb565(int width, int height)
{
    int y_stride = (width + 15) & ~15;
    int uv_stride = y_stride / 2;
    int row;
    int column;

    for (row = 0; row < height; row++)
    {
        for (column = 0; column < width; column++)
        {
            int32_t y = g_output_y[row * y_stride + column];
            int32_t u = g_output_u[(row / 2) * uv_stride + column / 2] - 128;
            int32_t v = g_output_v[(row / 2) * uv_stride + column / 2] - 128;
            uint8_t red = clamp_color(y + ((359 * v) >> 8));
            uint8_t green = clamp_color(y - ((88 * u + 183 * v) >> 8));
            uint8_t blue = clamp_color(y + ((454 * u) >> 8));

            g_rgb565_buffer[row * width + column] =
                (uint16_t)(((red & 0xF8U) << 8) | ((green & 0xFCU) << 3) | (blue >> 3));
        }
    }
}

static int display_on_lcd(int width, int height)
{
    rt_device_t lcd_device;
    struct rt_device_graphic_info info = {0};
    struct rt_device_graphic_ops *ops;
    uint16_t format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
    uint8_t brightness = 100;
    int start_x;
    int start_y;

    lcd_device = rt_device_find("lcd");
    if (lcd_device == RT_NULL)
    {
        rt_kprintf("JPEGD example FAILED: LCD device not found\n");
        return -1;
    }
    if (rt_device_open(lcd_device, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("JPEGD example FAILED: LCD device open failed\n");
        return -1;
    }
    if (rt_device_control(lcd_device, RTGRAPHIC_CTRL_GET_INFO, &info) != RT_EOK)
    {
        rt_kprintf("JPEGD example FAILED: LCD info query failed\n");
        return -1;
    }
    if ((info.width < width) || (info.height < height))
    {
        rt_kprintf("JPEGD example FAILED: LCD %ux%u is smaller than image %dx%d\n", info.width, info.height,
                   width, height);
        return -1;
    }
    if (rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &format) != RT_EOK)
    {
        rt_kprintf("JPEGD example FAILED: LCD RGB565 format setup failed\n");
        return -1;
    }

    ops = rt_graphix_ops(lcd_device);
    if ((ops == RT_NULL) || (ops->set_window == RT_NULL) || (ops->draw_rect_async == RT_NULL))
    {
        rt_kprintf("JPEGD example FAILED: LCD asynchronous drawing is unavailable\n");
        return -1;
    }

    start_x = (info.width - width) / 2;
    start_y = (info.height - height) / 2;
    if (info.draw_align > 1)
    {
        if ((width % info.draw_align) != 0)
        {
            rt_kprintf("JPEGD example FAILED: image width %d does not meet LCD alignment %u\n", width,
                       info.draw_align);
            return -1;
        }
        start_x = (start_x / info.draw_align) * info.draw_align;
    }
    yuv420_to_rgb565(width, height);
    SCB_CleanDCache_by_Addr((uint32_t *)g_rgb565_buffer, width * height * sizeof(uint16_t));
    ops->set_window(start_x, start_y, start_x + width - 1, start_y + height - 1);
    ops->draw_rect_async((const char *)g_rgb565_buffer, start_x, start_y, start_x + width - 1,
                         start_y + height - 1);
    rt_device_control(lcd_device, RTGRAPHIC_CTRL_SET_BRIGHTNESS, &brightness);
    rt_kprintf("LCD: RGB565 image displayed at (%d,%d), size=%dx%d\n", start_x, start_y, width, height);

    return 0;
}
#endif

static uint32_t calculate_checksum(const uint8_t *data, uint32_t size)
{
    uint32_t checksum = 0;
    uint32_t index;

    for (index = 0; index < size; index++)
    {
        checksum += data[index];
    }

    return checksum;
}

int main(void)
{
    JPEGD_HandleTypeDef handle = {0};
    JPEGD_DecodeConfigTypeDef config = {0};
    HAL_StatusTypeDef status;
    int width;
    int height;
    int work_size;
    int y_size;
    int u_size;
    int v_size;

    rt_kprintf("JPEGD AHB polling example\n");
    /*读取JPEG宽高*/
    status = HAL_JPEGD_GetDim((uint8_t *)g_jpeg_data, sizeof(g_jpeg_data), &width, &height);
    if (status != HAL_OK)
    {
        rt_kprintf("JPEGD example FAILED: GetDim returned %d\n", status);
        return -1;
    }
    if ((width != 100) || (height != 100))
    {
        rt_kprintf("JPEGD example FAILED: expected 100x100 baseline JPEG, got %dx%d\n", width, height);
        return -1;
    }
    /*配置 初始化 JPEGD HAL*/
    config.input = (uint8_t *)g_jpeg_data;
    config.input_data_size = sizeof(g_jpeg_data);
    config.output_mode = HAL_JPEGD_OUTPUT_AHB;
    config.work_buffer = g_work_buffer;
    config.output_y = g_output_y;
    config.output_u = g_output_u;
    config.output_v = g_output_v;
    config.start_x = 0;
    config.start_y = 0;
    config.width = width;
    config.height = height;

    handle.Instance = hwp_jpegd;
    status = HAL_JPEGD_Init(&handle, NULL, NULL);
    if (status != HAL_OK)
    {
        rt_kprintf("JPEGD example FAILED: Init returned %d\n", status);
        return -1;
    }

    work_size = HAL_JPEGD_GetBufferSize(&handle, &config);
    if ((work_size <= 0) || ((uint32_t)work_size > sizeof(g_work_buffer)))
    {
        rt_kprintf("JPEGD example FAILED: work buffer needs %d bytes, capacity is %u\n", work_size,
                   (unsigned int)sizeof(g_work_buffer));
        return -1;
    }

    status = HAL_JPEGD_GetOutputSize(&handle, &config, &y_size, &u_size, &v_size);
    if (status != HAL_OK)
    {
        rt_kprintf("JPEGD example FAILED: GetOutputSize returned %d\n", status);
        return -1;
    }
    if ((y_size <= 0) || (u_size <= 0) || (v_size <= 0) || ((uint32_t)y_size > sizeof(g_output_y)) ||
            ((uint32_t)u_size > sizeof(g_output_u)) || ((uint32_t)v_size > sizeof(g_output_v)))
    {
        rt_kprintf("JPEGD example FAILED: output needs Y/U/V=%d/%d/%d bytes, capacities are %u/%u/%u\n", y_size,
                   u_size, v_size, (unsigned int)sizeof(g_output_y), (unsigned int)sizeof(g_output_u),
                   (unsigned int)sizeof(g_output_v));
        return -1;
    }

    rt_kprintf("JPEG: %dx%d, input=%u bytes\n", width, height, (unsigned int)sizeof(g_jpeg_data));
    rt_kprintf("Buffers: work=%d, Y=%d, U=%d, V=%d bytes\n", work_size, y_size, u_size, v_size);

    memset(g_work_buffer, 0, work_size);
    memset(g_output_y, 0, y_size);
    memset(g_output_u, 0, u_size);
    memset(g_output_v, 0, v_size);

    /*JPEG解码*/
    status = HAL_JPEGD_Decode(&handle, &config);
    if (status != HAL_OK)
    {
        rt_kprintf("JPEGD example FAILED: Decode returned %d, error=0x%08x\n", status,
                   (unsigned int)handle.ErrorCode);
        return -1;
    }

#ifdef JPEGD_USING_LCD
    if (display_on_lcd(width, height) != 0)
    {
        HAL_JPEGD_DeInit(&handle);
        return -1;
    }
#endif

    status = HAL_JPEGD_DeInit(&handle);
    if (status != HAL_OK)
    {
        rt_kprintf("JPEGD example FAILED: DeInit returned %d\n", status);
        return -1;
    }

    rt_kprintf("JPEGD example PASSED\n");
    return 0;
}
