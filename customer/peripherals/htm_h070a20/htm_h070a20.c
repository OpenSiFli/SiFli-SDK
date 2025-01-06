#include <rtthread.h>
#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"
#include "log.h"


//#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif


#define LCD_ID                  0x8000
/**
  * @brief  LCD IC Size
  */
#define  LCD_IC_PIXEL_WIDTH    (1024)
#define  LCD_IC_PIXEL_HEIGHT   (600)

static LCDC_InitTypeDef lcdc_int_cfg =
{
    .lcd_itf = AUTO_SELECTED_DPI_INTFACE,
    .freq = 48 * 1000 * 1000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB888,

    .cfg = {
        .dpi = {
            .PCLK_polarity = 1,
            .DE_polarity   = 0,
            .VS_polarity   = 1,
            .HS_polarity   = 1,
            .PCLK_force_on = 0,

            .VS_width      = 3,    //VLW
            .HS_width      = 24,   //HLW

            .VBP = 21,   //VBP
            .VAH = LCD_VER_RES_MAX,
            .VFP = 12,   //VFP

            .HBP = 136,  //HBP
            .HAW = LCD_HOR_RES_MAX,
            .HFP = 160,  //HFP

            .interrupt_line_num = 1,
        },
    },

};


/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];

    /* Initialize ATK7016 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);
    LCD_DRIVER_DELAY_MS(10);
    BSP_LCD_Reset(1);
    LCD_DRIVER_DELAY_MS(500);

    HAL_LCDC_SetROIArea(hlcdc, 0, 0, LCD_HOR_RES_MAX - 1, LCD_VER_RES_MAX - 1);

}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
    return LCD_ID;
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOn(LCDC_HandleTypeDef *hlcdc)
{
    /* Display On */
    //WriteReg(hlcdc, DISPLAY_ON, (uint8_t *)NULL, 0);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOff(LCDC_HandleTypeDef *hlcdc)
{
    /* Display Off */
    //WriteReg(hlcdc, DISPLAY_OFF, (uint8_t *)NULL, 0);
}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{

}




static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint32_t size;

    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);
    HAL_LCDC_SendLayerData_IT(hlcdc);
}


static void LCD_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode)
{
    uint8_t   parameter[2];


    switch (color_mode)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB565;
        break;

    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB888;
        break;

    default:
        return; //unsupport
        break;
    }

    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    rt_device_t device = rt_device_find("lcdlight");
    if (device)
    {
        rt_err_t err = rt_device_open(device, RT_DEVICE_OFLAG_RDWR);
        uint8_t val = br;
        rt_device_write(device, 0, &val, 1);
        rt_device_close(device);
    }
    else
    {
        LOG_E("Can't find device lcdlight!");
    }
}

static const LCD_DrvOpsDef LCD_drv =
{
    .Init = LCD_Init,
    .ReadID = LCD_ReadID,
    .DisplayOn = LCD_DisplayOn,
    .DisplayOff = LCD_DisplayOff,

    .SetRegion = LCD_SetRegion,
    .WritePixel = NULL,
    .WriteMultiplePixels = LCD_WriteMultiplePixels,

    .ReadPixel = NULL,

    .SetColorMode = LCD_SetColorMode,
    .SetBrightness = LCD_SetBrightness,
};


LCD_DRIVER_EXPORT2(htm_h070a20, LCD_ID, &lcdc_int_cfg,
                   &LCD_drv, 1);
