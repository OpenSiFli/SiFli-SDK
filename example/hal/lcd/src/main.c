/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "mem_section.h"
#include "string.h"
#include "rtthread.h"

#define DBG_TAG  "hal.lcd"
#define DBG_LVL  DBG_INFO
#include "rtdbg.h"

#define LCD_WIDTH       1024
#define LCD_HEIGHT      600

#define FB_PIXEL_BYTES  2                                  /* RGB565 */
#define FB_SIZE_BYTES   (LCD_WIDTH * LCD_HEIGHT * FB_PIXEL_BYTES)
#define FB_NUM          2                                  /* double buffering */

#define LCD_RESET_PIN       18      /* PA18 -> panel RESET  */
#define LCD_BACKLIGHT_PIN   42      /* PA42 -> panel backlight enable */

/* RGB565 helper */
#define RGB565(r, g, b)     ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

L2_NON_RET_BSS_SECT_BEGIN(lcd_fb)
L2_NON_RET_BSS_SECT(lcd_fb, ALIGN(64) static uint8_t g_framebuffer[FB_NUM][FB_SIZE_BYTES]);
L2_NON_RET_BSS_SECT_END

static LCDC_HandleTypeDef hlcdc;
static struct rt_semaphore lcd_flush_sem;

static const LCDC_InitTypeDef lcdc_init_cfg =
{
    .lcd_itf = LCDC_INTF_DPI,                 /* 1024 <= LCDC_DPI_MAX_WIDTH(1024) -> plain DPI */
    .freq = 48 * 1000 * 1000,                 /* Pixel clock 48MHz */
    .color_mode = LCDC_PIXEL_FORMAT_RGB888,   /* LCDC output color format (panel is 24bit RGB) */

    .cfg = {
        .dpi = {
            .PCLK_polarity = 1,
            .DE_polarity   = 0,
            .VS_polarity   = 1,
            .HS_polarity   = 1,
            .PCLK_force_on = 0,

            .VS_width      = 3,     /* VSYNC pulse width, in HSYNC clocks (VLW) */
            .HS_width      = 24,    /* HSYNC pulse width, in pixel clocks (HLW) */

            .VBP = 21,              /* Vertical back porch  */
            .VAH = LCD_HEIGHT,      /* Vertical active height */
            .VFP = 12,              /* Vertical front porch */

            .HBP = 136,             /* Horizontal back porch */
            .HAW = LCD_WIDTH,       /* Horizontal active width */
            .HFP = 160,             /* Horizontal front porch */

            .interrupt_line_num = 1,
        },
    },
};


void LCDC1_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_LCDC_IRQHandler(&hlcdc);
    rt_interrupt_leave();
}

/* Frame transfer complete callback (called from HAL_LCDC_IRQHandler) */
static void lcd_flush_cplt(LCDC_HandleTypeDef *lcdc)
{
    rt_sem_release(&lcd_flush_sem);
}

/* Frame transfer error callback */
static void lcd_flush_error(LCDC_HandleTypeDef *lcdc)
{
    LOG_E("LCDC transfer error, code=0x%x", lcdc->ErrorCode);
    rt_sem_release(&lcd_flush_sem);
}

static void lcd_dpi_pins_init(void)
{
    /* Sync / clock */
    HAL_PIN_Set(PAD_PA12, LCDC1_DPI_CLK,   PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA13, LCDC1_DPI_DE,    PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA14, LCDC1_DPI_HSYNC, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA15, LCDC1_DPI_VSYNC, PIN_NOPULL, 1);

    /* Red[0:7] */
    HAL_PIN_Set(PAD_PA22, LCDC1_DPI_R0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA23, LCDC1_DPI_R1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA24, LCDC1_DPI_R2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA25, LCDC1_DPI_R3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA26, LCDC1_DPI_R4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA27, LCDC1_DPI_R5, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA43, LCDC1_DPI_R6, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA44, LCDC1_DPI_R7, PIN_NOPULL, 1);

    /* Green[0:7] */
    HAL_PIN_Set(PAD_PA45, LCDC1_DPI_G0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA46, LCDC1_DPI_G1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA47, LCDC1_DPI_G2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA48, LCDC1_DPI_G3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA50, LCDC1_DPI_G4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA53, LCDC1_DPI_G5, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA54, LCDC1_DPI_G6, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA55, LCDC1_DPI_G7, PIN_NOPULL, 1);

    /* Blue[0:7] */
    HAL_PIN_Set(PAD_PA56, LCDC1_DPI_B0, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA57, LCDC1_DPI_B1, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA58, LCDC1_DPI_B2, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA61, LCDC1_DPI_B3, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA62, LCDC1_DPI_B4, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA63, LCDC1_DPI_B5, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA65, LCDC1_DPI_B6, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA67, LCDC1_DPI_B7, PIN_NOPULL, 1);

    /* RESET and BACKLIGHT controlled as plain GPIO */
    HAL_PIN_Set(PAD_PA18, GPIO_A18, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA42, GPIO_A42, PIN_NOPULL, 1);
}

/* Generate a reset pulse to the panel */
static void lcd_reset(void)
{
    BSP_GPIO_Set(LCD_RESET_PIN, 0, 1);
    rt_thread_mdelay(10);
    BSP_GPIO_Set(LCD_RESET_PIN, 1, 1);
    rt_thread_mdelay(120);
}

/* Turn the backlight fully on (drive the enable pin high) */
static void lcd_backlight_on(void)
{
    BSP_GPIO_Set(LCD_BACKLIGHT_PIN, 1, 1);
}

/* ------------------------------------------------------------------------- *
 *  LCDC bring-up
 * ------------------------------------------------------------------------- */
static void lcd_hw_init(void)
{
    /* 1. Pin mux for the DPI bus and the control GPIOs */
    lcd_dpi_pins_init();

    /* 2. Initialize the LCDC handle.
          HAL_LCDC_Init() enables and resets the LCDC clock (RCC_MOD_LCDC1)
          and programs the DPI timing taken from hlcdc.Init. */
    memcpy(&hlcdc.Init, &lcdc_init_cfg, sizeof(LCDC_InitTypeDef));
    hlcdc.Instance = LCDC1;
    HAL_LCDC_Init(&hlcdc);

    /* 3. Enable the LCDC1 interrupt (used by HAL_LCDC_SendLayerData_IT) */
    HAL_NVIC_SetPriority(LCDC1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(LCDC1_IRQn);

    /* 4. Reset the panel (HTM-H070A20 needs no register init sequence) */
    lcd_reset();

    /* 5. The drawing region covers the whole panel */
    HAL_LCDC_SetROIArea(&hlcdc, 0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    /* 6. Configure the default layer once (frame buffer format is RGB565) */
    HAL_LCDC_LayerReset(&hlcdc, HAL_LCDC_LAYER_DEFAULT);
    HAL_LCDC_LayerSetFormat(&hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB565);

    /* 7. Bind the transfer callbacks */
    hlcdc.XferCpltCallback  = lcd_flush_cplt;
    hlcdc.XferErrorCallback = lcd_flush_error;

    /* 8. Light up the backlight */
    lcd_backlight_on();
}

/* ------------------------------------------------------------------------- *
 *  Drawing
 * ------------------------------------------------------------------------- */

/* Fill a frame buffer with a single RGB565 color */
static void fb_fill(uint8_t *fb, uint16_t color)
{
    uint16_t *p = (uint16_t *)fb;
    uint32_t  n = LCD_WIDTH * LCD_HEIGHT;

    while (n--)
    {
        *p++ = color;
    }
}

/* Draw 8 classic vertical color bars into a frame buffer */
static void fb_draw_color_bars(uint8_t *fb)
{
    static const uint16_t bar[8] =
    {
        RGB565(255, 255, 255), /* White  */
        RGB565(255, 255,   0), /* Yellow */
        RGB565(0, 255, 255),   /* Cyan   */
        RGB565(0, 255,   0),   /* Green  */
        RGB565(255,   0, 255), /* Magenta*/
        RGB565(255,   0,   0), /* Red    */
        RGB565(0,   0, 255),   /* Blue   */
        RGB565(0,   0,   0),   /* Black  */
    };

    uint16_t *p = (uint16_t *)fb;
    uint32_t  bar_w = LCD_WIDTH / 8;

    for (uint32_t y = 0; y < LCD_HEIGHT; y++)
    {
        uint16_t *line = p + y * LCD_WIDTH;
        for (uint32_t x = 0; x < LCD_WIDTH; x++)
        {
            uint32_t idx = x / bar_w;
            if (idx > 7) idx = 7;
            line[x] = bar[idx];
        }
    }
}


static void lcd_show(uint8_t *fb)
{
    mpu_dcache_clean(fb, FB_SIZE_BYTES);

    HAL_LCDC_LayerSetData(&hlcdc, HAL_LCDC_LAYER_DEFAULT, fb,
                          0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    HAL_LCDC_SendLayerData_IT(&hlcdc);

    /* Wait for the transfer complete / error callback */
    rt_sem_take(&lcd_flush_sem, rt_tick_from_millisecond(1000));
}


int main(void)
{
    LOG_I("HAL LCD(DPI) display example start");
    LOG_I("Panel %dx%d, %d frame buffers @%p (%d bytes each)",
          LCD_WIDTH, LCD_HEIGHT, FB_NUM, g_framebuffer, FB_SIZE_BYTES);

    rt_sem_init(&lcd_flush_sem, "lcdflush", 0, RT_IPC_FLAG_FIFO);

    lcd_hw_init();

    /* content index: 0 = color bars, 1 = red, 2 = green, 3 = blue, 4 = white */
    static const uint16_t solid[] =
    {
        RGB565(255,   0,   0), /* Red    */
        RGB565(0, 255,   0),   /* Green  */
        RGB565(0,   0, 255),   /* Blue   */
        RGB565(255, 255, 255), /* White  */
    };
    const uint32_t content_num = 1 + sizeof(solid) / sizeof(solid[0]);

    uint32_t back = 0;          /* index of the buffer we render into */
    uint32_t content_idx = 0;

    while (1)
    {
        uint8_t *fb = g_framebuffer[back];

        /* Render the next content into the BACK buffer */
        if (content_idx == 0)
        {
            fb_draw_color_bars(fb);
            LOG_I("show color bars");
        }
        else
        {
            fb_fill(fb, solid[content_idx - 1]);
            LOG_I("show solid color %u", content_idx - 1);
        }

        lcd_show(fb);

        /* The buffer just shown is now the front buffer; render into the other
           one next time. */
        back = (back + 1) % FB_NUM;
        content_idx = (content_idx + 1) % content_num;

        rt_thread_mdelay(2000);
    }

    return RT_EOK;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
