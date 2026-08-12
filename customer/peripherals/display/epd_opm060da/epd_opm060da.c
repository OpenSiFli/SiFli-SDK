/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"

#include "epd_tps.h"
#include "mem_section.h"
#include "epd_waveform.h"

/* Panel native (TCON output) resolution */
#define EPD_PANEL_HOR 1024
#define EPD_PANEL_VER 758

/* Rotation mode (from V12 epd_display.c logic) */
enum EpdRotation
{
    EPD_ROT_LANDSCAPE = 0,
    EPD_ROT_PORTRAIT = 1,
    EPD_ROT_INVERTED_LANDSCAPE = 2,
    EPD_ROT_INVERTED_PORTRAIT = 3,
};

/*
 * LVGL: HOR=758, VER=1024 (portrait). Panel: 1024x758 (landscape).
 * EPD_PANEL_HOR(1024) == LCD_VER_RES_MAX(1024) → rotate 90° CW
 */
#if (EPD_PANEL_HOR == LCD_VER_RES_MAX) && (EPD_PANEL_VER == LCD_HOR_RES_MAX)
    #define DISPLAY_ROTATE EPD_ROT_INVERTED_PORTRAIT
#else
    #define DISPLAY_ROTATE EPD_ROT_LANDSCAPE
#endif

static enum EpdRotation display_rotation = EPD_ROT_LANDSCAPE;

/**
  * @brief epd_opm060da chip IDs
  */
#define THE_LCD_ID                  0x19386

#define  DBG_LEVEL            DBG_INFO  //DBG_LOG //
#define LOG_TAG                "epd_opm060da"
#include "log.h"


static const LCDC_InitTypeDef lcdc_int_cfg_edp_8bit =
{
    .lcd_itf = LCDC_INTF_EPD_8BIT,
    .freq = 24 * 1000 * 1000, //sclk frequnecy
    .color_mode = LCDC_PIXEL_FORMAT_F2_SWAP,

    .cfg = {
        .epd = {
            .SDMODE = 0, //Source driver mode
            .SDCLK_polarity = 0, //Source driver clock polarity
            .GDSP_polarity = 0,
            .GDCLK_polarity = 0, //Gate clock polarity

            .LSL = 1, //Line start length
            .LBL = 4, //Line begin length
            .LDL = EPD_PANEL_HOR >> 2, //Line data length (1024 / 4)
                                 .LEL = 9, //Line end length

                                 .GSTA = 7, //Gate STA length

                                 .FSL = 1, //Frame sync length
                                 .FBL = 3, //Frame begin length
                                 .FDL = EPD_PANEL_VER, //Frame data length (758 rows)
                                 .FEL = 5, //Frame end length
        },
    },

};

static LCDC_InitTypeDef lcdc_int_cfg;
static void  LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t parameter[32];

    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_edp_8bit, sizeof(lcdc_int_cfg));
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(1);
    rt_thread_mdelay(10);
    BSP_LCD_Reset(0);//Reset LCD
    rt_thread_mdelay(10);
    BSP_LCD_Reset(1);
    rt_thread_mdelay(10);


    HAL_LCDC_SetROIArea(hlcdc, 0, 0, EPD_PANEL_HOR - 1, EPD_PANEL_VER - 1);

    display_rotation = DISPLAY_ROTATE;

    epd_wave_table();

    tps_init(1000); //-1.00V

    tps_exit_sleep();
    tps_enter_sleep();
}


/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
    return THE_LCD_ID;
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOn(LCDC_HandleTypeDef *hlcdc)
{
    /* Display On */
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOff(LCDC_HandleTypeDef *hlcdc)
{
    /* Display Off */
}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{

}

/**
  * @brief  Writes pixel.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  RGBCode: the RGB pixel color
  * @retval None
  */
static void LCD_WritePixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos, const uint8_t *RGBCode)
{

}

/*
Define a mixed grey framebuffer on PSRAM
high 4 bits for old pixel and low 4 bits for new pixel in every byte.
*/
L2_NON_RET_BSS_SECT_BEGIN(frambuf)
L2_NON_RET_BSS_SECT(frambuf, ALIGN(4) static uint8_t mixed_framebuffer[EPD_PANEL_HOR * EPD_PANEL_VER]);
L2_NON_RET_BSS_SECT_END


/*
 * Mixed grey framebuffer on PSRAM:
 * high 4 bits = old pixel (value currently shown on the panel),
 * low  4 bits = new pixel (value to be shown after this flush).
 * - Only the pixels inside [Xpos0..Xpos1] x [Ypos0..Ypos1] are updated:
 *   the previous low nibble (current on-screen value) is moved to the high
 *   nibble as the new "old", and the new gray value goes to the low nibble.
 * - Pixels outside the region are synced to old==new before the region fill,
 *   so their waveform maps to "do nothing" -> the rest of the panel stays
 *   still. This also clears the stale high nibble left by a previous partial
 *   flush, which would otherwise spuriously re-drive those pixels.
 */

/* Sync old (high nibble) to new (low nibble) for pixels in [x0, x1) of a row. */
static void MixedGraySyncRange(uint8_t *row, uint16_t x0, uint16_t x1)
{
    uint16_t x = x0;

    /* Leading unaligned bytes */
    while ((x < x1) && (x & 3))
    {
        row[x] = (row[x] & 0x0F) | ((row[x] & 0x0F) << 4);
        x++;
    }

    /* 4 pixels at a time */
    for (uint32_t *p = (uint32_t *)(row + x); (x + 4) <= x1; p++, x += 4)
    {
        uint32_t v = *p;
        *p = (v & 0x0F0F0F0F) | ((v & 0x0F0F0F0F) << 4);
    }

    /* Trailing unaligned bytes */
    while (x < x1)
    {
        row[x] = (row[x] & 0x0F) | ((row[x] & 0x0F) << 4);
        x++;
    }
}

/* Sync old (high nibble) to new (low nibble) for the pixels OUTSIDE
 * [rx0..rx1] x [ry0..ry1] (region in panel coordinates). Pixels inside the
 * region are rebuilt by the region fill right after, syncing them here
 * would only be redundant work. */
static void MixedGraySyncOldOutside(uint16_t rx0, uint16_t ry0, uint16_t rx1, uint16_t ry1)
{
    for (uint16_t y = 0; y < EPD_PANEL_VER; y++)
    {
        uint8_t *row = &mixed_framebuffer[(uint32_t)y * EPD_PANEL_HOR];

        if ((y < ry0) || (y > ry1))
        {
            /* Row fully outside the region */
            MixedGraySyncRange(row, 0, EPD_PANEL_HOR);
        }
        else
        {
            MixedGraySyncRange(row, 0, rx0);
            MixedGraySyncRange(row, rx1 + 1, EPD_PANEL_HOR);
        }
    }
}

L1_RET_CODE_SECT(epd_codes, static void CopyToMixedGrayBuffer(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1))
{
    uint32_t total_pixels = LCD_HOR_RES_MAX * LCD_VER_RES_MAX;
    RT_ASSERT(Xpos0 <= Xpos1);
    RT_ASSERT(Ypos0 <= Ypos1);
    RT_ASSERT(Xpos1 < LCD_HOR_RES_MAX);
    RT_ASSERT(Ypos1 < LCD_VER_RES_MAX);
    RT_ASSERT((total_pixels % 4) == 0); // Must be a multiple of 4 pixels

    // Partial update: pixels outside [Xpos0..Xpos1] x [Ypos0..Ypos1] are not
    // filled, so sync them to old==new first; otherwise the stale high nibble
    // left by a previous partial flush would make their waveform drive the panel.
    // Pixels inside the region are skipped here: the region fill below rebuilds
    // them (including their old nibble), syncing them first is redundant.
    if ((Xpos1 - Xpos0 + 1) != LCD_HOR_RES_MAX || (Ypos1 - Ypos0 + 1) != LCD_VER_RES_MAX)
    {
        if (display_rotation == EPD_ROT_INVERTED_PORTRAIT)
        {
            /* LVGL (x, y) -> panel (y, LCD_HOR_RES_MAX - 1 - x) */
            MixedGraySyncOldOutside(Ypos0, LCD_HOR_RES_MAX - 1 - Xpos1,
                                    Ypos1, LCD_HOR_RES_MAX - 1 - Xpos0);
        }
        else
        {
            MixedGraySyncOldOutside(Xpos0, Ypos0, Xpos1, Ypos1);
        }
    }

    //Convert layer data to 4bit gray data
    if (hlcdc->Layer[HAL_LCDC_LAYER_DEFAULT].data_format == LCDC_PIXEL_FORMAT_MONO)
    {
        RT_ASSERT(0);//Not implemented yet
    }
    else if (hlcdc->Layer[HAL_LCDC_LAYER_DEFAULT].data_format == LCDC_PIXEL_FORMAT_A4)
    {
        // Region fill: only update pixels inside
        // [Xpos0..Xpos1] x [Ypos0..Ypos1]. Pixels outside keep old==new
        // in the mixed framebuffer, so their waveform maps to "do nothing"
        // -> true partial update (not just a partial waveform with full
        // screen data).
        const uint8_t *p_src = RGBCode;
        uint8_t *p_dst = mixed_framebuffer;
        if (display_rotation == EPD_ROT_INVERTED_PORTRAIT)
        {
            for (uint16_t x = Xpos0; x <= Xpos1; x++)
            {
                for (uint16_t y = Ypos0; y <= Ypos1; y++)
                {
                    // Rotate 90 degree CW: source (x, y) -> dest (y, VER-1-x)
                    uint16_t dst_x = y;
                    uint16_t dst_y = LCD_HOR_RES_MAX - 1 - x;

                    // Linear index of the pixel in the packed source
                    // (A4 stores 2 pixels per byte, region-relative packing)
                    uint32_t src_index = (y - Ypos0) * (Xpos1 - Xpos0 + 1) + (x - Xpos0);
                    uint8_t src_byte = RGBCode[src_index >> 1];
                    uint8_t pixel_val = (src_index & 1) ? ((src_byte >> 4) & 0x0F) : (src_byte & 0x0F);

                    uint32_t dst_index = dst_y * EPD_PANEL_HOR + dst_x;
                    uint8_t old_val = mixed_framebuffer[dst_index];

                    // Move previous low nibble (current on-screen value) to
                    // the high nibble as the new "old", put new gray low.
                    mixed_framebuffer[dst_index] = ((old_val & 0x0F) << 4) | pixel_val;
                }
            }
        }
        else
        {
            /* 32-bit writes below need 4-pixel alignment */
            RT_ASSERT(((Xpos1 - Xpos0 + 1) % 4) == 0);
            RT_ASSERT((Xpos0 % 4) == 0);

            for (uint16_t y = Ypos0; y <= Ypos1; y++)
            {
                for (uint16_t x = Xpos0; x < Xpos1; x += 4)
                {
                    uint8_t byte0 = RGBCode[((y - Ypos0) * (Xpos1 - Xpos0 + 1) + (x - Xpos0)) / 2];
                    uint8_t byte1 = RGBCode[((y - Ypos0) * (Xpos1 - Xpos0 + 1) + (x - Xpos0) + 2) / 2];

                    // Generate new values for 4 pixels (A4: 2 pixels/byte)
                    uint8_t gray0 = (x % 2) ? ((byte0 >> 4) & 0x0F) : (byte0 & 0x0F);
                    uint8_t gray1 = ((x + 1) % 2) ? ((byte0 >> 4) & 0x0F) : (byte0 & 0x0F);
                    uint8_t gray2 = ((x + 2) % 2) ? ((byte1 >> 4) & 0x0F) : (byte1 & 0x0F);
                    uint8_t gray3 = ((x + 3) % 2) ? ((byte1 >> 4) & 0x0F) : (byte1 & 0x0F);

                    // Move the previous low nibble (current on-screen value)
                    // to the high nibble as the new "old", put new gray low.
                    uint32_t dst_index = y * EPD_PANEL_HOR + x;
                    uint32_t old_val = *((uint32_t *)(p_dst + dst_index));

                    uint32_t new_val =
                        (((old_val >> 0) & 0x0F) << 4)  | gray0 |
                        (((old_val >> 8) & 0x0F) << 12) | (gray1 << 8) |
                        (((old_val >> 16) & 0x0F) << 20) | (gray2 << 16) |
                        (((old_val >> 24) & 0x0F) << 28) | (gray3 << 24);

                    *((uint32_t *)(p_dst + dst_index)) = new_val;
                }
            }
        }
    }
    else if (hlcdc->Layer[HAL_LCDC_LAYER_DEFAULT].data_format == LCDC_PIXEL_FORMAT_RGB565)
    {
        // Calculate grayscale value
        // 0.299*R + 0.587*G + 0.114*B
#define RGB565_TO_GRAY4(rgb)  ( \
        (uint8_t)(( \
        ((((rgb) >> 8) & 0xF8) * 77 + \
         (((rgb) >> 3) & 0xFC) * 150 + \
         (((rgb) << 3) & 0xF8) * 29) >> 8) >> 4) \
        )

        if (display_rotation == EPD_ROT_INVERTED_PORTRAIT)
        {
            // Rotate LVGL portrait 758x1024 → panel landscape 1024x758
            // Source pixel (x, y) → dest pixel (y, 757-x)
            RT_ASSERT(((Ypos1 - Ypos0 + 1) % 4) == 0);
            RT_ASSERT((Ypos0 % 4) == 0);

            const uint16_t *p_src = (const uint16_t *)RGBCode;
            uint8_t *p_dst = mixed_framebuffer;

            for (uint16_t x = Xpos0; x <= Xpos1; x++)
            {
                for (uint16_t y = Ypos0; y < Ypos1; y += 4)
                {
                    uint8_t gray0 = RGB565_TO_GRAY4(p_src[(y - Ypos0) * (Xpos1 - Xpos0 + 1) + (x - Xpos0)]);
                    uint8_t gray1 = RGB565_TO_GRAY4(p_src[(y + 1 - Ypos0) * (Xpos1 - Xpos0 + 1) + (x - Xpos0)]);
                    uint8_t gray2 = RGB565_TO_GRAY4(p_src[(y + 2 - Ypos0) * (Xpos1 - Xpos0 + 1) + (x - Xpos0)]);
                    uint8_t gray3 = RGB565_TO_GRAY4(p_src[(y + 3 - Ypos0) * (Xpos1 - Xpos0 + 1) + (x - Xpos0)]);

                    uint16_t dst_x0 = y;
                    uint16_t dst_y0 = LCD_HOR_RES_MAX - 1 - x;

                    uint32_t dst_index = dst_y0 * EPD_PANEL_HOR + dst_x0;
                    uint32_t old_val = *((uint32_t *)(p_dst + dst_index));

                    uint32_t new_val =
                        (((old_val >> 0) & 0x0F) << 4)  | gray0 |
                        (((old_val >> 8) & 0x0F) << 12) | (gray1 << 8) |
                        (((old_val >> 16) & 0x0F) << 20) | (gray2 << 16) |
                        (((old_val >> 24) & 0x0F) << 28) | (gray3 << 24);

                    *((uint32_t *)(p_dst + dst_index)) = new_val;
                }
            }
        }
        else
        {
            // No rotation: straight mapping. Only fill the requested region
            // Pixels outside the region are left untouched,
            // so they keep old==new and their waveform maps to "do nothing".
            RT_ASSERT(((Xpos1 - Xpos0 + 1) % 4) == 0);
            RT_ASSERT((Xpos0 % 4) == 0);

            const uint16_t *p_src = (const uint16_t *)RGBCode;
            uint8_t *p_dst = mixed_framebuffer;

            for (uint16_t y = Ypos0; y <= Ypos1; y++)
            {
                for (uint16_t x = Xpos0; x < Xpos1; x += 4)
                {
                    uint8_t gray0 = RGB565_TO_GRAY4(p_src[(y - Ypos0) * (Xpos1 - Xpos0 + 1) + (x - Xpos0)]);
                    uint8_t gray1 = RGB565_TO_GRAY4(p_src[(y - Ypos0) * (Xpos1 - Xpos0 + 1) + (x - Xpos0) + 1]);
                    uint8_t gray2 = RGB565_TO_GRAY4(p_src[(y - Ypos0) * (Xpos1 - Xpos0 + 1) + (x - Xpos0) + 2]);
                    uint8_t gray3 = RGB565_TO_GRAY4(p_src[(y - Ypos0) * (Xpos1 - Xpos0 + 1) + (x - Xpos0) + 3]);

                    uint32_t dst_index = y * EPD_PANEL_HOR + x;
                    uint32_t old_val = *((uint32_t *)(p_dst + dst_index));

                    // Move the previous low nibble (current on-screen value)
                    // to the high nibble as the new "old", put new gray low.
                    uint32_t new_val =
                        (((old_val >> 0) & 0x0F) << 4)  | gray0 |
                        (((old_val >> 8) & 0x0F) << 12) | (gray1 << 8) |
                        (((old_val >> 16) & 0x0F) << 20) | (gray2 << 16) |
                        (((old_val >> 24) & 0x0F) << 28) | (gray3 << 24);

                    *((uint32_t *)(p_dst + dst_index)) = new_val;
                }
            }
        }

#undef RGB565_TO_GRAY4
    }
    else if (hlcdc->Layer[HAL_LCDC_LAYER_DEFAULT].data_format == LCDC_PIXEL_FORMAT_RGB888)
    {
        /* This format only supports full screen update */
        RT_ASSERT(((Xpos1 - Xpos0 + 1) == LCD_HOR_RES_MAX) && ((Ypos1 - Ypos0 + 1) == LCD_VER_RES_MAX));

        uint32_t n = total_pixels / 4; // Process 4 pixels (4 bytes) at a time
        uint32_t *p_dst = (uint32_t *)mixed_framebuffer;
        const uint8_t *p_src = (const uint8_t *)RGBCode;

        // Calculate grayscale value
        // 0.299*R + 0.587*G + 0.114*B
#define RGB888_TO_GRAY4(r, g, b)  ( \
        (uint8_t)(( \
        ((r) * 77 + \
         (g) * 150 + \
         (b) * 29) >> 8) >> 4) \
        )

        while (n--)
        {
            uint8_t r0 = *p_src++;
            uint8_t g0 = *p_src++;
            uint8_t b0 = *p_src++;
            uint8_t pixel0 = RGB888_TO_GRAY4(r0, g0, b0);

            uint8_t r1 = *p_src++;
            uint8_t g1 = *p_src++;
            uint8_t b1 = *p_src++;
            uint8_t pixel1 = RGB888_TO_GRAY4(r1, g1, b1);

            uint8_t r2 = *p_src++;
            uint8_t g2 = *p_src++;
            uint8_t b2 = *p_src++;
            uint8_t pixel2 = RGB888_TO_GRAY4(r2, g2, b2);

            uint8_t r3 = *p_src++;
            uint8_t g3 = *p_src++;
            uint8_t b3 = *p_src++;
            uint8_t pixel3 = RGB888_TO_GRAY4(r3, g3, b3);

            // Generate new values for 4 pixels
            uint32_t src_v = ((pixel3 << 24) | (pixel2 << 16) | (pixel1 << 8) | pixel0) & 0x0F0F0F0F;

            // Read old pixels, clear old pixel nibbles, shift new pixels to old pixel position
            uint32_t dst_v = (*p_dst & 0x0F0F0F0F) << 4;

            // Merge new pixels
            *p_dst++ = dst_v | src_v;
        }
    }
    else
        RT_ASSERT(0);
}


static void (* Ori_XferCpltCallback)(struct __LCDC_HandleTypeDef *lcdc);
static uint32_t start_tick;
static uint32_t total_frames; //Total frames need to do
static uint32_t curr_frame; //Current flushing frame index
static HAL_LCDC_PixelFormat ori_format;
static uint32_t lut[HAL_LCDC_LOOKUP_TABLE_SIZE >> 2];

/*
*/
static uint32_t StartFrame(LCDC_HandleTypeDef *hlcdc, uint32_t frame_idx)
{
    if (frame_idx < total_frames)
    {
        epd_wave_table_fill_lut((uint32_t *)&lut[0], frame_idx);
        BSP_LCD_GMODE_Set(0);
        HAL_Delay_us(1);
        BSP_LCD_GMODE_Set(1);
        HAL_LCDC_SendLayerData_IT(hlcdc);

        return 0;
    }
    else
    {
        tps_enter_sleep();
        return 1; //Done
    }
}
static void LCDC_SendLineCpltCbk(LCDC_HandleTypeDef *hlcdc)
{

    if (StartFrame(hlcdc, curr_frame++))
    {
        LOG_I("Take %d ticks", rt_tick_get() - start_tick);

        HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, ori_format); //Restore layer format
        if (Ori_XferCpltCallback) Ori_XferCpltCallback(hlcdc);
        Ori_XferCpltCallback = NULL;
    }
}

static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    start_tick = rt_tick_get();
    ori_format = hlcdc->Layer[HAL_LCDC_LAYER_DEFAULT].data_format;

    LOG_I("LCD_WriteMultiplePixels x0=%d, x1=%d, y0=%d, y1=%d\n", Xpos0, Xpos1 - Xpos0 + 1, Ypos0, Ypos1 - Ypos0 + 1);
    /*Implement grayscale buffer filling with partial‑fill support*/
    CopyToMixedGrayBuffer(hlcdc, RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);

    /* Always send the whole mixed framebuffer: the LCDC fetches layer data
       linearly from the buffer start using data_area width as line stride,
       so a partial data_area would fetch misaligned data (and shrink the
       layer area on the canvas). Partial refresh is achieved by the buffer
       content itself: pixels outside the region keep old==new and their
       waveform maps to "do nothing". */
    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)mixed_framebuffer,
                          0, 0, EPD_PANEL_HOR - 1, EPD_PANEL_VER - 1);
    HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_L8);

    HAL_LCDC_LayerSetLTab(hlcdc, HAL_LCDC_LAYER_DEFAULT, (LCDC_AColorDef *)lut);


    total_frames = epd_wave_table_get_frames(26/*temperature*/, EPD_DRAW_MODE_AUTO);
    curr_frame = 0;
    LOG_I("Done. Start to flush total_frames = %d", total_frames);


    Ori_XferCpltCallback = hlcdc->XferCpltCallback;
    hlcdc->XferCpltCallback = LCDC_SendLineCpltCbk;
    tps_exit_sleep();
    StartFrame(hlcdc, 0);
}


/**
  * @brief  Writes  to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters)
{
    HAL_LCDC_WriteU8Reg(hlcdc, LCD_Reg, Parameters, NbParameters);
}


/**
  * @brief  Reads the selected LCD Register.
  * @param  RegValue: Address of the register to read
  * @param  ReadSize: Number of bytes to read
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize)
{
    uint32_t rd_data = 0;

    HAL_LCDC_ReadU8Reg(hlcdc, RegValue, (uint8_t *)&rd_data, ReadSize);
    return rd_data;
}



static uint32_t LCD_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos)
{
    return 0;
}


static void LCD_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode)
{
}

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{

}

static void IdleModeOn(LCDC_HandleTypeDef *hlcdc)
{
    tps_enter_sleep();
}

static void IdleModeOff(LCDC_HandleTypeDef *hlcdc)
{
    tps_exit_sleep();
}

static const LCD_DrvOpsDef lcd_drv_operations =
{
    LCD_Init,
    LCD_ReadID,
    LCD_DisplayOn,
    LCD_DisplayOff,

    LCD_SetRegion,
    LCD_WritePixel,
    LCD_WriteMultiplePixels,
    NULL,

    LCD_SetColorMode,
    LCD_SetBrightness,
    IdleModeOn,
    IdleModeOff,
    NULL,
    NULL,
    NULL,
    NULL
};

LCD_DRIVER_EXPORT2(epd_opm060da, THE_LCD_ID, &lcdc_int_cfg,
                   &lcd_drv_operations, 1);
