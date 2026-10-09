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
#include "hang_diag.h"

/**
  * @brief epd_opm060da chip IDs
  */
#define THE_LCD_ID                  0x19357

#define  DBG_LEVEL            DBG_INFO  //DBG_LOG //
#define LOG_TAG                "epd_e0470a03_57"
#include "log.h"
enum EpdRotation
{
    EPD_ROT_LANDSCAPE = 0,
    EPD_ROT_PORTRAIT = 1,
    EPD_ROT_INVERTED_LANDSCAPE = 2,
    EPD_ROT_INVERTED_PORTRAIT = 3,
};

/* Panel native (TCON output) resolution */
#define EPD_PANEL_HOR 1216
#define EPD_PANEL_VER 684
#define EPD_POWER_OFF_IDLE_MS 300
#define EPD_GRAY_TILE_WIDTH 16
#define EPD_GRAY_TILE_HEIGHT 32

#if (EPD_PANEL_HOR == LCD_VER_RES_MAX) && (EPD_PANEL_VER == LCD_HOR_RES_MAX)
    #define DISPLAY_ROTATE EPD_ROT_INVERTED_PORTRAIT
#else
    #define DISPLAY_ROTATE EPD_ROT_LANDSCAPE
#endif


static enum EpdRotation display_rotation = EPD_ROT_LANDSCAPE;



static const LCDC_InitTypeDef lcdc_int_cfg_edp_16bit =
{
    .lcd_itf = LCDC_INTF_EPD_8BIT,
    .freq = 24 * 1000 * 1000,
    .color_mode = LCDC_PIXEL_FORMAT_F2_SWAP,

    .cfg = {
        .epd = {
#ifdef EPD_WAVEFORM_USE_BIN
            .SDMODE = 1, //Source driver mode (BIN waveform requires SDMODE=1)
#else
            .SDMODE = 0, //Source driver mode
#endif
            .SDCLK_polarity = 0, //Source driver clock polarity
            .GDSP_polarity = 0,
            .GDCLK_polarity = 0, //Gate clock polarity

            .LSL = 4, //Line start length   300ns
            .LBL = 1, //Line begin length
            .LDL = EPD_PANEL_HOR >> 2, //Line data length:
                                 .LEL = 3, //Line end length

                                 .GSTA = 3, //Gate STA length

                                 .FSL = 1, //Frame sync length
                                 .FBL = 3, //Frame begin length    100ns  ->
                                 .FDL = EPD_PANEL_VER, //Frame data length (684 rows)
                                 .FEL = 1, //Frame end length

        },
    },

};

static LCDC_InitTypeDef lcdc_int_cfg;
static void  LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);
static void epd_power_entry(void *parameter);
static void epd_power_off_locked(void);
static void epd_power_off(void);
static struct rt_semaphore epd_power_wakeup;
static struct rt_semaphore epd_power_ready;
static struct rt_mutex epd_power_lock;
static rt_thread_t epd_power_thread;
static rt_bool_t epd_panel_powered;
static rt_bool_t epd_poweron_pending;
/* The completion ISR publishes these together; power transitions use the mutex. */
static volatile rt_bool_t epd_refresh_active;
static volatile rt_tick_t epd_idle_since;

/* SetRegion and WriteMultiplePixels are serialized by the LCD worker. */
static LCD_AreaDef epd_update_area;
static LCD_AreaDef epd_previous_update_area;
static rt_bool_t epd_update_area_valid;
static rt_bool_t epd_gray_cache_valid;










/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t parameter[32];
    rt_err_t err;

    if (epd_power_thread == RT_NULL)
    {
        err = rt_mutex_init(&epd_power_lock, "epd_pwr", RT_IPC_FLAG_PRIO);
        RT_ASSERT(err == RT_EOK);
        err = rt_sem_init(&epd_power_wakeup, "epd_evt", 0, RT_IPC_FLAG_FIFO);
        RT_ASSERT(err == RT_EOK);
        err = rt_sem_init(&epd_power_ready, "epd_on", 0, RT_IPC_FLAG_FIFO);
        RT_ASSERT(err == RT_EOK);
        /* Run before lcd_task so stabilization delays overlap image conversion. */
        epd_power_thread = rt_thread_create("epd_pwr", epd_power_entry, RT_NULL,
                                            1024, RT_THREAD_PRIORITY_HIGH - 1, 10);
        RT_ASSERT(epd_power_thread != RT_NULL);
        err = rt_thread_startup(epd_power_thread);
        RT_ASSERT(err == RT_EOK);
    }

    err = rt_mutex_take(&epd_power_lock, RT_WAITING_FOREVER);
    RT_ASSERT(err == RT_EOK);
    epd_power_off_locked();

    epd_update_area_valid = RT_FALSE;
    epd_gray_cache_valid = RT_FALSE;

    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_edp_16bit, sizeof(lcdc_int_cfg));
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

#ifdef EPD_WAVEFORM_USE_BIN
    tps_init(2100); //-2.10V (BIN waveform requires VCOM=2100mV)
#else
    tps_init(1000); //-1.00V
#endif
    rt_mutex_release(&epd_power_lock);
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
    epd_power_off();
}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    (void)hlcdc;
    epd_update_area_valid = RT_FALSE;
    if (Xpos0 > Xpos1 || Ypos0 > Ypos1 ||
            Xpos1 >= LCD_HOR_RES_MAX || Ypos1 >= LCD_VER_RES_MAX)
    {
        return;
    }

    /* This window limits conversion only; the panel still scans the full frame. */
    epd_update_area.x0 = Xpos0;
    epd_update_area.y0 = Ypos0;
    epd_update_area.x1 = Xpos1;
    epd_update_area.y1 = Ypos1;
    epd_update_area_valid = RT_TRUE;
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
#ifdef BSP_USING_PSRAM2
    L2_NON_RET_BSS_SECT2_BEGIN(frambuf)
    L2_NON_RET_BSS_SECT2(frambuf, ALIGN(4) static uint8_t mixed_framebuffer[EPD_PANEL_HOR * EPD_PANEL_VER]);
    L2_NON_RET_BSS_SECT2_END
#else
    L2_NON_RET_BSS_SECT_BEGIN(frambuf)
    L2_NON_RET_BSS_SECT(frambuf, ALIGN(4) static uint8_t mixed_framebuffer[EPD_PANEL_HOR * EPD_PANEL_VER]);
    L2_NON_RET_BSS_SECT_END
#endif

/* Column-major gray tile in internal SRAM; LCD transfers are serialized. */
L1_NON_RET_BSS_SECT_BEGIN(epd_gray_tile)
ALIGN(4)
L1_NON_RET_BSS_SECT(epd_gray_tile, static uint32_t epd_gray_tile[EPD_GRAY_TILE_WIDTH][EPD_GRAY_TILE_HEIGHT / 4]);
L1_NON_RET_BSS_SECT_END

L1_RET_CODE_SECT(epd_codes, static void SettlePreviousGrayArea(void))
{
    /* Only the previous update can still contain old != new pixel pairs. */
    const LCD_AreaDef *area = &epd_previous_update_area;
    const uint32_t y0 = (uint32_t)area->y0 & ~3U;
    const uint32_t y_end = ((uint32_t)area->y1 + 4) & ~3U;
    const uint32_t words = (y_end - y0) / 4;
    for (int32_t x = area->x0; x <= area->x1; x++)
    {
        uint32_t *dst = (uint32_t *)(mixed_framebuffer +
                                     (LCD_HOR_RES_MAX - 1 - x) * EPD_PANEL_HOR + y0);
        for (uint32_t word = 0; word < words; word++)
        {
            uint32_t gray = dst[word] & 0x0F0F0F0F;
            dst[word] = gray | (gray << 4);
        }
    }
}

L1_RET_CODE_SECT(epd_codes, static void CopyToMixedGrayBuffer(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1, const LCD_AreaDef *update_area))
{
    RT_ASSERT(LCD_HOR_RES_MAX == (Xpos1 - Xpos0 + 1));
    RT_ASSERT(LCD_VER_RES_MAX == (Ypos1 - Ypos0 + 1));

    rt_kprintf("[EPD] flush: %s, src=%dx%d, update=(%d,%d)-(%d,%d)\n",
               display_rotation == EPD_ROT_INVERTED_PORTRAIT ? "PORTRAIT(rot)" :
               display_rotation == EPD_ROT_LANDSCAPE ? "LANDSCAPE" :
               display_rotation == EPD_ROT_PORTRAIT ? "PORTRAIT" : "INV_LANDSCAPE",
               Xpos1 - Xpos0 + 1, Ypos1 - Ypos0 + 1,
               update_area->x0, update_area->y0, update_area->x1, update_area->y1);

    //Convert layer data to 4bit gray data
    if (hlcdc->Layer[HAL_LCDC_LAYER_DEFAULT].data_format == LCDC_PIXEL_FORMAT_MONO)
    {
        RT_ASSERT(0);//Not implemented yet
    }
    else if (hlcdc->Layer[HAL_LCDC_LAYER_DEFAULT].data_format == LCDC_PIXEL_FORMAT_A4)
    {
        const uint8_t *p_src = RGBCode;
        uint8_t *p_dst = mixed_framebuffer;

        if (display_rotation == EPD_ROT_INVERTED_PORTRAIT)
        {

            uint16_t src_w = LCD_HOR_RES_MAX; // 758

            for (uint16_t x = Xpos0; x <= Xpos1; x++)
            {
                uint16_t sx = x - Xpos0;
                uint8_t *p_dst_row = &p_dst[(LCD_HOR_RES_MAX - 1 - x) * EPD_PANEL_HOR];

                for (uint16_t y = Ypos0; y <= Ypos1; y++)
                {
                    uint16_t sy = y - Ypos0;
                    uint8_t src_byte = p_src[sy * (src_w / 2) + sx / 2];
                    uint8_t gray = (sx & 1) ? (src_byte >> 4) : (src_byte & 0x0F);

                    uint8_t *p_dst_pixel = &p_dst_row[y];
                    *p_dst_pixel = ((*p_dst_pixel & 0x0F) << 4) | gray;
                }
            }
        }
        else
        {
            uint32_t n = LCD_HOR_RES_MAX * LCD_VER_RES_MAX / 4; // 每次处理4像素（4字节）
            uint32_t *p_dst32 = (uint32_t *)(mixed_framebuffer);

            while (n--)
            {
                uint8_t byte0 = *p_src++;
                uint8_t byte1 = *p_src++;

                // 生成4像素的新值
                uint32_t src_v = ((byte1 << 20) | (byte1 << 16) | (byte0 << 4) | byte0) & 0x0F0F0F0F;

                // 读取原像素，旧像素清零，新像素移入老像素
                uint32_t dst_v = (*p_dst32 & 0x0F0F0F0F) << 4;

                // 合并新像素
                *p_dst32++ = dst_v | src_v;
            }

            //16bit 特殊处理
            // uint32_t n = LCD_HOR_RES_MAX * LCD_VER_RES_MAX / 8; // 每次处理8像素（8字节）
            // uint64_t *p_dst64 = (uint64_t *)(mixed_framebuffer);

            // while (n--)
            // {
            //     uint8_t byte0 = *p_src++;
            //     uint8_t byte1 = *p_src++;
            //     uint8_t byte2 = *p_src++;
            //     uint8_t byte3 = *p_src++;


            //     // 生成8像素的新值
            //     uint32_t src_v_1 = ((byte1 << 20) | (byte1 << 16) | (byte0 << 4) | byte0) & 0x0F0F0F0F;
            //     uint32_t src_v_2 = ((byte3 << 20) | (byte3 << 16) | (byte2 << 4) | byte2) & 0x0F0F0F0F;

            //     uint64_t src_v = ((uint64_t)src_v_1 << 32) | src_v_2;
            //     // 读取原像素，旧像素清零，新像素移入老像素
            //     uint64_t dst_v = (*p_dst64 & 0x0F0F0F0F0F0F0F0FULL) << 4;

            //     // 合并新像素
            //     *p_dst64++ = dst_v | src_v;

        }
    }
    else if (hlcdc->Layer[HAL_LCDC_LAYER_DEFAULT].data_format == LCDC_PIXEL_FORMAT_RGB565)
    {
        // 计算灰度值
        // 0.299*R + 0.587*G + 0.114*B
#define RGB565_TO_GRAY4(rgb)  ( \
        (uint8_t)(( \
        ((((rgb) >> 8) & 0xF8) * 77 + \
         (((rgb) >> 3) & 0xFC) * 150 + \
         (((rgb) << 3) & 0xF8) * 29) >> 8) >> 4) \
        )

        if (display_rotation == EPD_ROT_INVERTED_PORTRAIT)
        {
            /* Source (x, y) maps to panel (y, LCD_HOR_RES_MAX - 1 - x). */
            const uint32_t src_w = Xpos1 - Xpos0 + 1;
            const uint32_t src_h = Ypos1 - Ypos0 + 1;
            const uint32_t x_begin = update_area->x0 - Xpos0;
            const uint32_t x_end = update_area->x1 - Xpos0 + 1;
            const uint32_t dirty_y_begin = update_area->y0 - Ypos0;
            const uint32_t dirty_y_end = update_area->y1 - Ypos0 + 1;
            const uint32_t y_begin = dirty_y_begin & ~3U;
            const uint32_t y_end = (dirty_y_end + 3) & ~3U;
            RT_ASSERT((src_h % 4) == 0);
            RT_ASSERT((Ypos0 % 4) == 0);
            RT_ASSERT(x_begin < x_end && x_end <= src_w &&
                      y_begin < y_end && y_end <= src_h);

            const uint16_t *p_src = (const uint16_t *)RGBCode;

            for (uint32_t tile_y = y_begin; tile_y < y_end; tile_y += EPD_GRAY_TILE_HEIGHT)
            {
                const uint32_t tile_h = (y_end - tile_y < EPD_GRAY_TILE_HEIGHT) ?
                                        y_end - tile_y : EPD_GRAY_TILE_HEIGHT;
                for (uint32_t tile_x = x_begin; tile_x < x_end; tile_x += EPD_GRAY_TILE_WIDTH)
                {
                    const uint32_t tile_w = (x_end - tile_x < EPD_GRAY_TILE_WIDTH) ?
                                            x_end - tile_x : EPD_GRAY_TILE_WIDTH;

                    /* Read consecutive RGB pixels; transpose only in SRAM. */
                    for (uint32_t y = 0; y < tile_h; y++)
                    {
                        const uint32_t src_y = tile_y + y;
                        if (src_y < dirty_y_begin || src_y >= dirty_y_end)
                        {
                            /* Preserve cached pixels outside the update window when aligning writes. */
                            for (uint32_t x = 0; x < tile_w; x++)
                            {
                                const uint32_t dst_y = LCD_HOR_RES_MAX - 1 - (Xpos0 + tile_x + x);
                                ((uint8_t *)epd_gray_tile[x])[y] =
                                    mixed_framebuffer[dst_y * EPD_PANEL_HOR + Ypos0 + src_y] & 0x0F;
                            }
                        }
                        else
                        {
                            const uint16_t *src_row = p_src + src_y * src_w + tile_x;
                            for (uint32_t x = 0; x < tile_w; x++)
                            {
                                ((uint8_t *)epd_gray_tile[x])[y] = RGB565_TO_GRAY4(src_row[x]);
                            }
                        }
                    }

                    /* Each tile column becomes consecutive panel pixels. */
                    for (uint32_t x = 0; x < tile_w; x++)
                    {
                        const uint32_t dst_y = LCD_HOR_RES_MAX - 1 - (Xpos0 + tile_x + x);
                        uint32_t *dst = (uint32_t *)(mixed_framebuffer +
                                                     dst_y * EPD_PANEL_HOR + Ypos0 + tile_y);
                        const uint32_t *gray = epd_gray_tile[x];

                        for (uint32_t word = 0; word < tile_h / 4; word++)
                        {
                            dst[word] = ((dst[word] & 0x0F0F0F0F) << 4) | gray[word];
                        }
                    }
                }
            }
        }
        else
        {
            uint32_t n = LCD_HOR_RES_MAX * LCD_VER_RES_MAX / 4; // 每次处理4像素（4字节）
            uint32_t *p_dst = (uint32_t *)(mixed_framebuffer);
            const uint16_t *p_src = (const uint16_t *)RGBCode;

            while (n--)
            {
                uint8_t pixel0 = RGB565_TO_GRAY4(*p_src);
                p_src++;
                uint8_t pixel1 = RGB565_TO_GRAY4(*p_src);
                p_src++;
                uint8_t pixel2 = RGB565_TO_GRAY4(*p_src);
                p_src++;
                uint8_t pixel3 = RGB565_TO_GRAY4(*p_src);
                p_src++;


                // 生成4像素的新值
                uint32_t src_v = ((pixel3 << 24) | (pixel2 << 16) | (pixel1 << 8) | pixel0) & 0x0F0F0F0F;

                // 读取原像素，旧像素清零，新像素移入老像素
                uint32_t dst_v = (*p_dst & 0x0F0F0F0F) << 4;

                // 合并新像素
                *p_dst++ = dst_v | src_v;
            }
        }

#undef RGB565_TO_GRAY4
    }
    else
        RT_ASSERT(0);
}


static void (* Ori_XferCpltCallback)(struct __LCDC_HandleTypeDef *lcdc);
static uint32_t start_tick;
static uint32_t total_frames; //Total frames need to do
static uint32_t curr_frame; //Next waveform frame index to send
static HAL_LCDC_PixelFormat ori_format;
static uint32_t lut[HAL_LCDC_LOOKUP_TABLE_SIZE >> 2];

static void epd_power_off_locked(void)
{
    RT_ASSERT(!epd_refresh_active);
    if (epd_panel_powered)
    {
        /* Keep the lock through rail discharge before allowing another power-up. */
        tps_enter_sleep();
        epd_panel_powered = RT_FALSE;
    }
}

static void epd_power_off(void)
{
    rt_err_t err = rt_mutex_take(&epd_power_lock, RT_WAITING_FOREVER);
    RT_ASSERT(err == RT_EOK);
    epd_power_off_locked();
    rt_mutex_release(&epd_power_lock);
}

static void epd_power_entry(void *parameter)
{
    const rt_tick_t idle_ticks = rt_tick_from_millisecond(EPD_POWER_OFF_IDLE_MS);
    (void)parameter;

    while (1)
    {
        rt_err_t err = rt_sem_take(&epd_power_wakeup, RT_WAITING_FOREVER);
        RT_ASSERT(err == RT_EOK);

        while (1)
        {
            err = rt_mutex_take(&epd_power_lock, RT_WAITING_FOREVER);
            RT_ASSERT(err == RT_EOK);

            if (epd_poweron_pending)
            {
                tps_exit_sleep();
                epd_panel_powered = RT_TRUE;
                epd_poweron_pending = RT_FALSE;
                rt_mutex_release(&epd_power_lock);
                err = rt_sem_release(&epd_power_ready);
                RT_ASSERT(err == RT_EOK);
                break;
            }

            rt_base_t level = rt_hw_interrupt_disable();
            rt_bool_t active = epd_refresh_active;
            rt_tick_t idle_since = epd_idle_since;
            rt_hw_interrupt_enable(level);

            if (!epd_panel_powered || active)
            {
                rt_mutex_release(&epd_power_lock);
                break;
            }

            rt_tick_t elapsed = rt_tick_get() - idle_since;
            if (elapsed >= idle_ticks)
            {
                epd_power_off_locked();
                rt_mutex_release(&epd_power_lock);
                break;
            }

            rt_mutex_release(&epd_power_lock);
            /* Recheck the newest completion time after either a wakeup or timeout. */
            err = rt_sem_take(&epd_power_wakeup, (rt_int32_t)(idle_ticks - elapsed));
            RT_ASSERT(err == RT_EOK || err == -RT_ETIMEOUT);
        }
    }
}

/*
*/
static uint32_t StartFrame(LCDC_HandleTypeDef *hlcdc, uint32_t frame_idx)
{
    if (frame_idx < total_frames)
    {
        hang_diag_mark(HANG_DIAG_LCD, "scan-frame", frame_idx, RT_TRUE);
        epd_wave_table_fill_lut((uint32_t *)&lut[0], frame_idx);
        BSP_LCD_GMODE_Set(0);
        HAL_Delay_us(1);
        BSP_LCD_GMODE_Set(1);
        HAL_LCDC_SendLayerData_IT(hlcdc);

        return 0;
    }
    else
    {
        return 1; //Done
    }
}
static void LCDC_SendLineCpltCbk(LCDC_HandleTypeDef *hlcdc)
{

    if (StartFrame(hlcdc, curr_frame++))
    {
        LOG_I("Take %d ticks", rt_tick_get() - start_tick);

        HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, ori_format); //Restore layer format
        void (*callback)(struct __LCDC_HandleTypeDef *) = Ori_XferCpltCallback;
        Ori_XferCpltCallback = NULL;
        /* The SDK callback checks and clears this pointer before releasing draw_sem. */
        hlcdc->XferCpltCallback = callback;

        rt_base_t level = rt_hw_interrupt_disable();
        epd_idle_since = rt_tick_get();
        epd_refresh_active = RT_FALSE;
        rt_hw_interrupt_enable(level);

        rt_err_t err = rt_sem_release(&epd_power_wakeup);
        RT_ASSERT(err == RT_EOK);
        if (callback)
        {
            callback(hlcdc);
        }
        hang_diag_mark(HANG_DIAG_LCD, "complete", curr_frame, RT_FALSE);
    }
}

static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    start_tick = rt_tick_get();
    hang_diag_mark(HANG_DIAG_LCD, "power-lock", 0, RT_TRUE);
    rt_err_t err = rt_mutex_take(&epd_power_lock, RT_WAITING_FOREVER);
    RT_ASSERT(err == RT_EOK);
    RT_ASSERT(!epd_refresh_active);
    /* Reserve the panel before conversion so an old idle timeout cannot power it off. */
    rt_base_t level = rt_hw_interrupt_disable();
    epd_refresh_active = RT_TRUE;
    rt_hw_interrupt_enable(level);
    rt_bool_t wait_poweron = !epd_panel_powered;
    if (wait_poweron)
    {
        epd_poweron_pending = RT_TRUE;
        err = rt_sem_release(&epd_power_wakeup);
        RT_ASSERT(err == RT_EOK);
    }
    rt_mutex_release(&epd_power_lock);

    ori_format = hlcdc->Layer[HAL_LCDC_LAYER_DEFAULT].data_format;

    LCD_AreaDef update_area = {Xpos0, Ypos0, Xpos1, Ypos1};
    rt_bool_t cache_supported = (ori_format == LCDC_PIXEL_FORMAT_RGB565 &&
                                 display_rotation == EPD_ROT_INVERTED_PORTRAIT &&
                                 Xpos0 == 0 && Ypos0 == 0 &&
                                 Xpos1 == LCD_HOR_RES_MAX - 1 && Ypos1 == LCD_VER_RES_MAX - 1);
    if (cache_supported && epd_gray_cache_valid && epd_update_area_valid)
    {
        update_area = epd_update_area;
    }
    /* A draw without a fresh window must never reuse an old dirty rectangle. */
    epd_update_area_valid = RT_FALSE;

    LOG_I("LCD_WriteMultiplePixels %d pixels to %d, %d", (Xpos1 - Xpos0 + 1) * (Ypos1 - Ypos0 + 1), Xpos0, Ypos0);

    if (cache_supported && epd_gray_cache_valid &&
            (update_area.x0 != Xpos0 || update_area.y0 != Ypos0 ||
             update_area.x1 != Xpos1 || update_area.y1 != Ypos1))
    {
        hang_diag_mark(HANG_DIAG_LCD, "settle-gray", 0, RT_TRUE);
        SettlePreviousGrayArea();
    }
    hang_diag_mark(HANG_DIAG_LCD, "convert-gray", 0, RT_TRUE);
    CopyToMixedGrayBuffer(hlcdc, RGBCode, Xpos0, Ypos0, Xpos1, Ypos1, &update_area);
    /* Publish both settled old pixels and new gray pixels before LCDC reads them. */
    hang_diag_mark(HANG_DIAG_LCD, "clean-gray-cache", 0, RT_TRUE);
    mpu_dcache_clean(mixed_framebuffer, sizeof(mixed_framebuffer));
    epd_gray_cache_valid = cache_supported;
    if (cache_supported) epd_previous_update_area = update_area;
    hang_diag_mark(HANG_DIAG_LCD, "lcdc-setup", 0, RT_TRUE);
    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)mixed_framebuffer, 0, 0, EPD_PANEL_HOR - 1, EPD_PANEL_VER - 1);
    HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_L8);

    HAL_LCDC_LayerSetLTab(hlcdc, HAL_LCDC_LAYER_DEFAULT, (LCDC_AColorDef *)lut);


    hang_diag_mark(HANG_DIAG_LCD, "waveform", 0, RT_TRUE);
    total_frames = epd_wave_table_get_frames(26, EPD_DRAW_MODE_AUTO);
    curr_frame = 0;


    Ori_XferCpltCallback = hlcdc->XferCpltCallback;
    hlcdc->XferCpltCallback = LCDC_SendLineCpltCbk;
    if (wait_poweron)
    {
        /* Conversion and both power-up delays must finish before scanning. */
        hang_diag_mark(HANG_DIAG_LCD, "power-ready", 0, RT_TRUE);
        err = rt_sem_take(&epd_power_ready, RT_WAITING_FOREVER);
        RT_ASSERT(err == RT_EOK);
    }
    LOG_I("Done. Start to flush total_frames = %d", total_frames);
    StartFrame(hlcdc, curr_frame++);
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
    epd_power_off();
}

static void IdleModeOff(LCDC_HandleTypeDef *hlcdc)
{
    /* The next refresh powers up the panel. */
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

LCD_DRIVER_EXPORT2(epd_e0470a03_57, THE_LCD_ID, &lcdc_int_cfg,
                   &lcd_drv_operations, 1);

