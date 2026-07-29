/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "bf0_hal.h"
#include "drv_epic.h"
#include "drv_lcd.h"
#include "mem_section.h"
// 包含图片数据
#include "array.h"

#define BUFFER_SIZE   (LCD_HOR_RES_MAX * LCD_VER_RES_MAX * 2)

#define TEXT_SRC_WIDTH   268
#define TEXT_DRAW_WIDTH  266
#define TEXT_DRAW_HEIGHT 266

// 旋转

#define ROTATE_IMAGE_WIDTH  265
#define ROTATE_IMAGE_HEIGHT 265
//mask
#define MASK_ANGLE 10800
// 分配显存空间到psram
L2_NON_RET_BSS_SECT_BEGIN(epic_buffers)
L2_NON_RET_BSS_SECT(epic_buffers, ALIGN(64) static uint8_t buffer0[BUFFER_SIZE]);
L2_NON_RET_BSS_SECT(epic_buffers, ALIGN(64) static uint8_t buffer1[BUFFER_SIZE]);
L2_NON_RET_BSS_SECT(epic_buffers, ALIGN(64) static uint8_t buffer2[BUFFER_SIZE]);
L2_NON_RET_BSS_SECT_END


static rt_device_t lcd_dev = RT_NULL;
EPIC_AreaTypeDef clear_area = {.x0 = 0, .y0 = 0, .x1 = LCD_HOR_RES_MAX - 1, .y1 = LCD_VER_RES_MAX - 1};
EPIC_AreaTypeDef dst_clear = {.x0 = 0, .y0 = 0, .x1 = LCD_HOR_RES_MAX - 1, .y1 = LCD_VER_RES_MAX - 1};
static uint32_t clear_color = 0xFF000000; // 黑色

void lcd_display_update(uint8_t *buffer)
{
    uint16_t buffer_format = RTGRAPHIC_PIXEL_FORMAT_RGB565;
    rt_device_control(lcd_dev, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &buffer_format);

    struct rt_device_graphic_info info;
    rt_memset(&info, 0, sizeof(info));
    rt_device_control(lcd_dev, RTGRAPHIC_CTRL_GET_INFO, &info);

    rt_kprintf("LCD Info: Width=%d, Height=%d, BPP=%d\n", info.width, info.height, info.bits_per_pixel);
    struct rt_device_graphic_ops *ops = rt_graphix_ops(lcd_dev);
    if (ops && ops->draw_rect)
    {
        ops->set_window(0, 0, LCD_HOR_RES_MAX - 1, LCD_VER_RES_MAX - 1);
        ops->draw_rect((const char *)buffer, 0, 0, LCD_HOR_RES_MAX - 1, LCD_VER_RES_MAX - 1);
    }
    else
    {
        rt_kprintf("draw_rect not available!\n");
    }
}
//混叠
void blend_layers_demo(void)
{
    rt_kprintf("Starting EPIC blend demo...\n");
    drv_epic_fill(EPIC_COLOR_RGB565, buffer2, &dst_clear, &clear_area, clear_color, 0, NULL, NULL, NULL);

    EPIC_AreaTypeDef fill_area = {.x0 = 100, .y0 = 100, .x1 = 250, .y1 = 200};
    EPIC_AreaTypeDef dst_area = {.x0 = 0, .y0 = 0, .x1 = LCD_HOR_RES_MAX - 1, .y1 = LCD_VER_RES_MAX - 1};

    // 填充蓝色矩形到 buffer0
    uint32_t argb_color = 0xFF0000FF; // ARGB8888 蓝色
    rt_err_t ret = drv_epic_fill(EPIC_COLOR_RGB565, buffer0, &dst_area, &fill_area, argb_color, 0, NULL, NULL, NULL);
    if (ret != RT_EOK)
    {
        rt_kprintf("Fill blue rect failed\n");
        return;
    }

    // 填充红色矩形到 buffer1
    ret = drv_epic_fill(EPIC_COLOR_RGB565, buffer1, &dst_area, &fill_area, 0xFFFF0000, 0, NULL, NULL, NULL);
    if (ret != RT_EOK)
    {
        rt_kprintf("Fill red rect failed\n");
        return;
    }
    // Layer 1: 蓝色
    EPIC_LayerConfigTypeDef fg_layer;
    HAL_EPIC_LayerConfigInit(&fg_layer);
    fg_layer.data = buffer0;
    fg_layer.color_mode = EPIC_COLOR_RGB565;
    fg_layer.width = 150;
    fg_layer.height = 100;
    fg_layer.x_offset = 50;
    fg_layer.y_offset = 50;
    fg_layer.total_width = LCD_HOR_RES_MAX;
    fg_layer.color_en = false;
    fg_layer.alpha = 128;
    fg_layer.ax_mode = ALPHA_BLEND_RGBCOLOR;
    HAL_EPIC_LayerSetDataOffset((EPIC_BlendingDataType *)&fg_layer, 150, 150);

    // Layer 2: 红色图层
    EPIC_LayerConfigTypeDef bg_layer;
    HAL_EPIC_LayerConfigInit(&bg_layer);
    bg_layer.data = buffer1;
    bg_layer.color_mode = EPIC_COLOR_RGB565;
    bg_layer.width = 150;
    bg_layer.height = 100;
    bg_layer.x_offset = 100;
    bg_layer.y_offset = 100;
    bg_layer.total_width = LCD_HOR_RES_MAX;
    bg_layer.color_en = false;
    bg_layer.alpha = 128;
    bg_layer.ax_mode = ALPHA_BLEND_RGBCOLOR;
    HAL_EPIC_LayerSetDataOffset((EPIC_BlendingDataType *)&bg_layer, 200, 200);

    EPIC_LayerConfigTypeDef input_layers[] = {fg_layer, bg_layer};
    uint8_t input_layer_num = sizeof(input_layers) / sizeof(EPIC_LayerConfigTypeDef);

    EPIC_LayerConfigTypeDef output_layer;
    HAL_EPIC_LayerConfigInit(&output_layer);
    output_layer.data = buffer2;
    output_layer.color_mode = EPIC_COLOR_RGB565;
    output_layer.width = LCD_HOR_RES_MAX;
    output_layer.height = LCD_VER_RES_MAX;
    output_layer.x_offset = 0;
    output_layer.y_offset = 0;
    output_layer.total_width = LCD_HOR_RES_MAX;
    output_layer.color_en = true;
    output_layer.alpha = 255;

    // 开始混叠
    rt_err_t rett = drv_epic_blend(input_layers, input_layer_num, &output_layer, NULL);
    if (rett != RT_EOK)
    {
        rt_kprintf("EPIC blend failed (%d)\n", rett);
    }
    else
    {
        rt_kprintf("EPIC blend succeeded\n");
        drv_epic_wait_done();
        lcd_display_update(buffer2);
    }
}


void epic_gradient_demo(void)
{
    rt_kprintf("Starting EPIC gradient fill demo...\n");

    // 清空输出缓冲区
    drv_epic_fill(EPIC_COLOR_RGB565, buffer2, &dst_clear, &clear_area, clear_color, 0, NULL, NULL, NULL);

    // 创建渐变填充参数
    EPIC_GradCfgTypeDef grad_param;
    grad_param.start = buffer2;
    grad_param.color_mode = EPIC_COLOR_RGB565;
    grad_param.width = LCD_HOR_RES_MAX;
    grad_param.height = LCD_VER_RES_MAX;
    grad_param.total_width = LCD_HOR_RES_MAX;

    // 设置四种角点颜色，创建彩虹渐变效果
    grad_param.color[0][0].full = 0xFFFF0000; // 左上角 - 红色
    grad_param.color[0][1].full = 0xFFFFFF00; // 右上角 - 黄色
    grad_param.color[1][0].full = 0xFF0000FF; // 左下角 - 蓝色
    grad_param.color[1][1].full = 0xFFFF00FF; // 右下角 - 紫色

    // 执行渐变填充
    rt_err_t ret = drv_epic_fill_grad(&grad_param, NULL);
    if (ret != RT_EOK)
    {
        rt_kprintf("EPIC gradient fill failed (%d)\n", ret);
    }
    else
    {
        rt_kprintf("EPIC gradient fill succeeded\n");
        drv_epic_wait_done();
        lcd_display_update(buffer2);

    }
}


// ezip图片缩放函数， multiple:缩放倍数，width,height:图片宽高
// 注意：1是原始倍数，2是缩放2倍，3是缩放3倍，以此类推
void scale_down_demo(int multiple, int width, int height)
{
    rt_kprintf("Starting image scale down demo...\n");
    drv_epic_fill(EPIC_COLOR_RGB565, buffer0, &dst_clear, &clear_area, clear_color, 0, NULL, NULL, NULL);
    int bei =  1024 * multiple;
    // 创建输入图层配置
    EPIC_LayerConfigTypeDef input_layers[1];
    HAL_EPIC_LayerConfigInit(&input_layers[0]);
    // 设置EZIP数据源
#ifdef SF32LB57X
    // SF32LB57 EZIP 解码窗口限制，使用 -winsize 2048 重新生成数据
    input_layers[0].data = ezip_img_data_57;
#else
    input_layers[0].data = ezip_img_data;
#endif
    input_layers[0].color_mode = EPIC_INPUT_EZIP;
    input_layers[0].width = width;
    input_layers[0].height = height;
    input_layers[0].x_offset = (LCD_HOR_RES_MAX - width) / 2;
    input_layers[0].y_offset = (LCD_VER_RES_MAX - height) / 2;
    input_layers[0].total_width = width;
    input_layers[0].color_en = false;
    input_layers[0].alpha = 255;
    input_layers[0].ax_mode = ALPHA_BLEND_RGBCOLOR;


    input_layers[0].transform_cfg.scale_x = bei;
    input_layers[0].transform_cfg.scale_y = bei;
    input_layers[0].transform_cfg.angle = 0;
    input_layers[0].transform_cfg.pivot_x = width / 2;
    input_layers[0].transform_cfg.pivot_y = height / 2;
    input_layers[0].transform_cfg.h_mirror = 0;
    input_layers[0].transform_cfg.v_mirror = 0;

    // 配置输出图层
    EPIC_LayerConfigTypeDef output_layer;
    HAL_EPIC_LayerConfigInit(&output_layer);
    output_layer.data = buffer0;
    output_layer.color_mode = EPIC_COLOR_RGB565;
    output_layer.width = LCD_HOR_RES_MAX;
    output_layer.height = LCD_VER_RES_MAX;
    output_layer.x_offset = 0;
    output_layer.y_offset = 0;
    output_layer.total_width = LCD_HOR_RES_MAX;
    output_layer.color_en = true;
    output_layer.alpha = 255;


    drv_epic_blend(input_layers, 1, &output_layer, NULL);
    rt_kprintf("show lcd ...\n");
    drv_epic_wait_done();
    lcd_display_update(buffer0);

}


void rotate_and_mask_demo()
{
    uint16_t animation_angle = 0;
    while (animation_angle < MASK_ANGLE)
    {
        animation_angle += 90;
        rt_kprintf("mask start--------------------------- %d...\n", animation_angle);
        drv_epic_fill(EPIC_COLOR_RGB565, buffer0, &dst_clear, &clear_area, clear_color, 0, NULL, NULL, NULL);
        EPIC_LayerConfigTypeDef input_layers[3];

        //背景图层 (input_layers[0])
        HAL_EPIC_LayerConfigInit(&input_layers[0]);
        input_layers[0].data = buffer0;
        input_layers[0].color_mode = EPIC_COLOR_RGB565;
        input_layers[0].width = LCD_HOR_RES_MAX;
        input_layers[0].height = LCD_VER_RES_MAX;
        input_layers[0].x_offset = 0;
        input_layers[0].y_offset = 0;
        input_layers[0].total_width = LCD_HOR_RES_MAX;
        input_layers[0].color_en = false;
        input_layers[0].alpha = 255;
        input_layers[0].ax_mode = ALPHA_BLEND_RGBCOLOR;

        // 图片图层 (input_layers[1])
        HAL_EPIC_LayerConfigInit(&input_layers[1]);
        input_layers[1].data = mask_2_data;
        input_layers[1].color_mode = EPIC_COLOR_RGB565;
        input_layers[1].width = 270;  // 270
        input_layers[1].height = 270; // 270
        input_layers[1].x_offset = (LCD_HOR_RES_MAX - 270) / 2;
        input_layers[1].y_offset = (LCD_VER_RES_MAX - 270) / 2;
        input_layers[1].total_width = 270;
        input_layers[1].color_en = false;
        input_layers[1].alpha = 255;
        input_layers[1].ax_mode = ALPHA_BLEND_RGBCOLOR;
        // 配置变换参数
        input_layers[1].transform_cfg.scale_x = 1024;  // 不缩放
        input_layers[1].transform_cfg.scale_y = 1024;  // 不缩放
        input_layers[1].transform_cfg.angle = animation_angle;   // 设置旋转角度
        input_layers[1].transform_cfg.pivot_x = 270 / 2;
        input_layers[1].transform_cfg.pivot_y = 270 / 2;
        input_layers[1].transform_cfg.h_mirror = 0;    // 不镜像
        input_layers[1].transform_cfg.v_mirror = 0;

        //mask
        HAL_EPIC_LayerConfigInit(&input_layers[2]);
        input_layers[2].data = mask_watch_data;
        input_layers[2].color_mode = EPIC_COLOR_A8;
        input_layers[2].width = 329;
        input_layers[2].height = 330;
        input_layers[2].x_offset = (LCD_HOR_RES_MAX - 329) / 2;
        input_layers[2].y_offset = (LCD_VER_RES_MAX - 330) / 2;
        input_layers[2].total_width = 329;
        input_layers[2].color_en = false;
        input_layers[2].alpha = 255;
        input_layers[2].ax_mode = ALPHA_BLEND_MASK;
        uint8_t pixel_size = HAL_EPIC_GetColorDepth(input_layers[2].color_mode);
        input_layers[2].data_size = ((pixel_size * input_layers[2].total_width * input_layers[2].height) + 7) >> 3;

        // 配置输出图层
        EPIC_LayerConfigTypeDef output_layer;
        HAL_EPIC_LayerConfigInit(&output_layer);
        output_layer.data = buffer0;
        output_layer.color_mode = EPIC_COLOR_RGB565;
        output_layer.width = LCD_HOR_RES_MAX;
        output_layer.height = LCD_VER_RES_MAX;
        output_layer.x_offset = 0;
        output_layer.y_offset = 0;
        output_layer.total_width = LCD_HOR_RES_MAX;
        output_layer.color_en = true;
        output_layer.alpha = 255;

        drv_epic_blend(input_layers, 3, &output_layer, NULL);
        drv_epic_wait_done();
        lcd_display_update(buffer0);
    }
}


void text_blend_demo(void)
{
    rt_kprintf("Starting text blend demo ...\n");

    // 清空缓冲区
    drv_epic_fill(EPIC_COLOR_RGB565, buffer0, &dst_clear, &clear_area, clear_color, 0, NULL, NULL, NULL);
    drv_epic_fill(EPIC_COLOR_RGB565, buffer1, &dst_clear, &clear_area, clear_color, 0, NULL, NULL, NULL);
    drv_epic_fill(EPIC_COLOR_RGB565, buffer2, &dst_clear, &clear_area, clear_color, 0, NULL, NULL, NULL);

    const int SI_TEXT_WIDTH = TEXT_DRAW_WIDTH;
    const int SI_TEXT_HEIGHT = TEXT_DRAW_HEIGHT;
    const int SI_OFFSET_X = (LCD_HOR_RES_MAX - SI_TEXT_WIDTH) / 2 - 50;  // 稍微左移
    const int SI_OFFSET_Y = (LCD_VER_RES_MAX - SI_TEXT_HEIGHT) / 2;

    const int CHE_TEXT_WIDTH = TEXT_DRAW_WIDTH;
    const int CHE_TEXT_HEIGHT = TEXT_DRAW_HEIGHT;
    const int CHE_OFFSET_X = (LCD_HOR_RES_MAX - CHE_TEXT_WIDTH) / 2 + 50;  // 稍微右移
    const int CHE_OFFSET_Y = (LCD_VER_RES_MAX - CHE_TEXT_HEIGHT) / 2;

    EPIC_AreaTypeDef fill_area = {.x0 = 0, .y0 = 0, .x1 = LCD_HOR_RES_MAX - 1, .y1 = LCD_VER_RES_MAX - 1};
    EPIC_AreaTypeDef dst_area = {.x0 = 0, .y0 = 0, .x1 = LCD_HOR_RES_MAX - 1, .y1 = LCD_VER_RES_MAX - 1};
    uint32_t bg_color = 0xFF000000; // ARGB8888 黑色背景

    // 填充黑色背景
    drv_epic_fill(EPIC_COLOR_RGB565, buffer0, &dst_area, &fill_area, bg_color, 0, NULL, NULL, NULL);

    // 第一个文字图层 (si_text_data)
    EPIC_LayerConfigTypeDef si_text_layer;
    HAL_EPIC_LayerConfigInit(&si_text_layer);
    si_text_layer.data = si_text_data;
    si_text_layer.color_mode = EPIC_COLOR_A4;
    si_text_layer.width = SI_TEXT_WIDTH;
    si_text_layer.height = SI_TEXT_HEIGHT;
    si_text_layer.x_offset = SI_OFFSET_X;
    si_text_layer.y_offset = SI_OFFSET_Y;
    si_text_layer.total_width = TEXT_SRC_WIDTH;  // A4格式要2像素对齐
    si_text_layer.color_en = true;
    si_text_layer.color_r = 255;
    si_text_layer.color_g = 255;
    si_text_layer.color_b = 0;  // 黄色
    si_text_layer.alpha = 255;
    si_text_layer.ax_mode = ALPHA_BLEND_MASK;

    uint8_t pixel_size = HAL_EPIC_GetColorDepth(si_text_layer.color_mode);
    si_text_layer.data_size = ((pixel_size * si_text_layer.total_width * si_text_layer.height) + 7) >> 3;

    // 第二个文字图层 (che_text_data)
    EPIC_LayerConfigTypeDef che_text_layer;
    HAL_EPIC_LayerConfigInit(&che_text_layer);
    che_text_layer.data = che_text_data;
    che_text_layer.color_mode = EPIC_COLOR_A4;
    che_text_layer.width = CHE_TEXT_WIDTH;
    che_text_layer.height = CHE_TEXT_HEIGHT;
    che_text_layer.x_offset = CHE_OFFSET_X;
    che_text_layer.y_offset = CHE_OFFSET_Y;
    che_text_layer.total_width = TEXT_SRC_WIDTH;  // A4格式要2像素对齐
    che_text_layer.color_en = true;
    che_text_layer.color_r = 255;
    che_text_layer.color_g = 0;
    che_text_layer.color_b = 0;  // 红色
    che_text_layer.alpha = 255;
    che_text_layer.ax_mode = ALPHA_BLEND_MASK;

    che_text_layer.data_size = ((pixel_size * che_text_layer.total_width * che_text_layer.height) + 7) >> 3;

    // 背景图层
    EPIC_LayerConfigTypeDef bg_layer;
    HAL_EPIC_LayerConfigInit(&bg_layer);
    bg_layer.data = buffer0;
    bg_layer.color_mode = EPIC_COLOR_RGB565;
    bg_layer.width = LCD_HOR_RES_MAX;
    bg_layer.height = LCD_VER_RES_MAX;
    bg_layer.x_offset = 0;
    bg_layer.y_offset = 0;
    bg_layer.total_width = LCD_HOR_RES_MAX;
    bg_layer.color_en = false;
    bg_layer.alpha = 255;
    bg_layer.ax_mode = ALPHA_BLEND_RGBCOLOR;

    // 配置输出图层
    EPIC_LayerConfigTypeDef output_layer;
    HAL_EPIC_LayerConfigInit(&output_layer);
    output_layer.data = buffer2;
    output_layer.color_mode = EPIC_COLOR_RGB565;
    output_layer.width = LCD_HOR_RES_MAX;
    output_layer.height = LCD_VER_RES_MAX;
    output_layer.x_offset = 0;
    output_layer.y_offset = 0;
    output_layer.total_width = LCD_HOR_RES_MAX;
    output_layer.color_en = true;
    output_layer.alpha = 255;

    // 先混合第一个文本
    EPIC_LayerConfigTypeDef input_layers1[] = {bg_layer, si_text_layer};
    rt_err_t ret = drv_epic_cont_blend(input_layers1, 2, &output_layer);
    if (ret != RT_EOK)
    {
        rt_kprintf("First text blend failed: %d\n", ret);
        return;
    }

    // 再混合第二个文本到同一个输出缓冲区
    EPIC_LayerConfigTypeDef input_layers2[] = {output_layer, che_text_layer};
    ret = drv_epic_cont_blend(input_layers2, 2, &output_layer);
    if (ret != RT_EOK)
    {
        rt_kprintf("Second text blend failed: %d\n", ret);
        return;
    }
    drv_epic_wait_done();
    // 重置连续混合状态
    drv_epic_cont_blend_reset();
    lcd_display_update(buffer2);
}
void display_single_text(void)
{
    rt_kprintf("Displaying single text: si_text_data\n");

    // 清空缓冲区
    drv_epic_fill(EPIC_COLOR_RGB565, buffer0, &dst_clear, &clear_area, clear_color, 0, NULL, NULL, NULL);
    drv_epic_fill(EPIC_COLOR_RGB565, buffer1, &dst_clear, &clear_area, clear_color, 0, NULL, NULL, NULL);
    drv_epic_fill(EPIC_COLOR_RGB565, buffer2, &dst_clear, &clear_area, clear_color, 0, NULL, NULL, NULL);

    const int TEXT_WIDTH = TEXT_DRAW_WIDTH;
    const int TEXT_HEIGHT = TEXT_DRAW_HEIGHT;
    const int OFFSET_X = (LCD_HOR_RES_MAX - TEXT_WIDTH) / 2;
    const int OFFSET_Y = (LCD_VER_RES_MAX - TEXT_HEIGHT) / 2; // 居中显示

    // 创建背景图层
    EPIC_AreaTypeDef fill_area = {.x0 = 0, .y0 = 0, .x1 = LCD_HOR_RES_MAX - 1, .y1 = LCD_VER_RES_MAX - 1};
    EPIC_AreaTypeDef dst_area = {.x0 = 0, .y0 = 0, .x1 = LCD_HOR_RES_MAX - 1, .y1 = LCD_VER_RES_MAX - 1};
    uint32_t bg_color = 0xFF000000;

    // 填充黑色背景
    drv_epic_fill(EPIC_COLOR_RGB565, buffer0, &dst_area, &fill_area, bg_color, 0, NULL, NULL, NULL);

    // 文字图层 (si_test_data)
    EPIC_LayerConfigTypeDef text_layer;
    HAL_EPIC_LayerConfigInit(&text_layer);
    text_layer.data = si_text_data;
    text_layer.color_mode = EPIC_COLOR_A4;
    text_layer.width = TEXT_WIDTH;
    text_layer.height = TEXT_HEIGHT;
    text_layer.x_offset = OFFSET_X;
    text_layer.y_offset = OFFSET_Y;
    text_layer.total_width = TEXT_SRC_WIDTH;    //A4格式要2像素对齐
    text_layer.color_en = true;
    text_layer.color_r = 255;
    text_layer.color_g = 255;
    text_layer.color_b = 0;
    text_layer.alpha = 255; // 不透明
    text_layer.ax_mode = ALPHA_BLEND_MASK;

    uint8_t pixel_size = HAL_EPIC_GetColorDepth(text_layer.color_mode);
    text_layer.data_size = ((pixel_size * text_layer.total_width * text_layer.height) + 7) >> 3;

    // 背景图层
    EPIC_LayerConfigTypeDef bg_layer;
    HAL_EPIC_LayerConfigInit(&bg_layer);
    bg_layer.data = buffer0;
    bg_layer.color_mode = EPIC_COLOR_RGB565;
    bg_layer.width = LCD_HOR_RES_MAX;
    bg_layer.height = LCD_VER_RES_MAX;
    bg_layer.x_offset = 0;
    bg_layer.y_offset = 0;
    bg_layer.total_width = LCD_HOR_RES_MAX;
    bg_layer.color_en = false;
    bg_layer.alpha = 255;
    bg_layer.ax_mode = ALPHA_BLEND_RGBCOLOR;

    EPIC_LayerConfigTypeDef input_layers[] = {bg_layer, text_layer};
    uint8_t input_layer_num = sizeof(input_layers) / sizeof(EPIC_LayerConfigTypeDef);

    // 配置输出图层
    EPIC_LayerConfigTypeDef output_layer;
    HAL_EPIC_LayerConfigInit(&output_layer);
    output_layer.data = buffer2;
    output_layer.color_mode = EPIC_COLOR_RGB565;
    output_layer.width = LCD_HOR_RES_MAX;
    output_layer.height = LCD_VER_RES_MAX;
    output_layer.x_offset = 0;
    output_layer.y_offset = 0;
    output_layer.total_width = LCD_HOR_RES_MAX;
    output_layer.color_en = true;
    output_layer.alpha = 255;

    // 执行混叠操作
    drv_epic_cont_blend(input_layers, input_layer_num, &output_layer);
    drv_epic_wait_done();
    drv_epic_cont_blend_reset();
    lcd_display_update(buffer2);
}


void init(void)
{
    // 初始化 GPU，包括中断等资源
    drv_gpu_open();
    rt_kprintf("drv_gpu_open ok\n");
    lcd_dev = rt_device_find("lcd");
    if (!lcd_dev)
    {
        rt_kprintf("Failed to find LCD device\n");
        return;
    }

    if (rt_device_open(lcd_dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Failed to open LCD device\n");
        return;
    }
}

int main(int argc, char *argv[])
{
    rt_kprintf("EZIP Image Display Application started\n");
    init();
    while (1)
    {
        //混叠演示
        blend_layers_demo();
        rt_thread_mdelay(2000);
        //渐变填充演示
        epic_gradient_demo();
        rt_thread_mdelay(2000);
        //缩放演示
        scale_down_demo(1, 205, 208);
        rt_thread_mdelay(1000);
        scale_down_demo(2, 205, 208);
        rt_thread_mdelay(1000);
        scale_down_demo(3, 205, 208);
        rt_thread_mdelay(500);
#ifndef SF32LB55X
        //旋转+遮罩演示
        rotate_and_mask_demo();
        rt_thread_mdelay(500);
        //单文字显示
        display_single_text();
        rt_thread_mdelay(2000);
        //文字混叠演示
        text_blend_demo();
        rt_thread_mdelay(2000);
#endif

    }

    return 0;
}