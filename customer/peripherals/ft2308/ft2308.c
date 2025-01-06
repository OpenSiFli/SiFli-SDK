/**
  ******************************************************************************
  * @file   ft2308.c
  * @author software development team
  * @brief   This file includes the LCD driver for ft2308 LCD.
  * @attention
  ******************************************************************************
*/

#include <rtthread.h>
#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"
#include "log.h"









#define ROW_OFFSET  (0x00)
#define COL_OFFSET  (0x00)




/**
  * @brief ft2308 chip IDs
  */
#define THE_LCD_ID                  0x19386





/**
  * @brief  ft2308 Registers
  */
#define REG_SW_RESET           0x0100
#define REG_LCD_ID             0x0400
#define REG_DSI_ERR            0x0500
#define REG_POWER_MODE         0x0A00
#define REG_SLEEP_IN           0x1000
#define REG_SLEEP_OUT          0x1100
#define REG_PARTIAL_DISPLAY    0x1200
#define REG_DISPLAY_INVERSION  0x2100
#define REG_DISPLAY_OFF        0x2800
#define REG_DISPLAY_ON         0x2900
#define REG_WRITE_RAM          0x2C00
#define REG_READ_RAM           0x2E00
#define REG_CASET              0x2A00
#define REG_RASET              0x2B00




#define REG_TEARING_EFFECT     0x3500

#define REG_IDLE_MODE_OFF      0x3800
#define REG_IDLE_MODE_ON       0x3900
#define REG_COLOR_MODE         0x3A00

#define REG_WBRIGHT            0x5100 /* Write brightness*/





































#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif











#ifdef BSP_LCDC_USING_DDR_QADSPI
    #define QAD_SPI_ITF LCDC_INTF_SPI_DCX_DDR_4DATA
    #define QAD_SPI_ITF_FREQ   40000000
#else
    #define QAD_SPI_ITF LCDC_INTF_SPI_DCX_4DATA
    #define QAD_SPI_ITF_FREQ   50000000
#endif

static const LCDC_InitTypeDef lcdc_int_cfg_spi =
{
    .lcd_itf = QAD_SPI_ITF, //LCDC_INTF_SPI_NODCX_1DATA,
    .freq = QAD_SPI_ITF_FREQ,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 1, //0: QAD-SPI/SPI3   1:SPI4
#ifdef LCD_FT2308_VSYNC_ENABLE
            .syn_mode = HAL_LCDC_SYNC_VER, //HAL_LCDC_SYNC_VER,
#else
            .syn_mode = HAL_LCDC_SYNC_DISABLE, //HAL_LCDC_SYNC_VER,
#endif
            .vsyn_polarity = 1,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 0,
            .hsyn_num = 0,
#ifdef LCDC_SUPPORT_DDR_QSPI
            .flags = SPI_LCD_FLAG_DDR_DUMMY_CLOCK,
#endif /* LCDC_SUPPORT_DDR_QSPI */
            .readback_from_Dx = 3,
        },
    },

};

static LCDC_InitTypeDef lcdc_int_cfg;
static void     LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable);











/**
  * @brief  spi read/write mode
  * @param  enable: false - write spi mode |  true - read spi mode
  * @retval None
  */
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable)
{
    if (HAL_LCDC_IS_SPI_IF(lcdc_int_cfg.lcd_itf))
    {
        if (enable)
        {
            HAL_LCDC_SetFreq(hlcdc, 2000000); //read mode min cycle 300ns
        }
        else
        {
            HAL_LCDC_SetFreq(hlcdc, lcdc_int_cfg.freq); //Restore normal frequency
        }
    }
}


/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
#if 1//def LCD_USING_ED_LB5XSPI19701_QADSPI_LB56X_HDK
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t parameter[32];

    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_spi, sizeof(lcdc_int_cfg));
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(1);
    rt_thread_mdelay(10);
    BSP_LCD_Reset(0);//Reset LCD
    rt_thread_mdelay(10);
    BSP_LCD_Reset(1);
    rt_thread_mdelay(10);
    uint16_t num = 0;

    uint8_t data[40] = {0};
    data[0] = 0x23;
    data[1] = 0x08;
    data[2] = 0x01;
    LCD_WriteReg(hlcdc, 0xFF00, data, 3);

    data[0] = 0x23;
    data[1] = 0x08;
    LCD_WriteReg(hlcdc, 0xFF80, data, 2);

//    data[0] = 0xEF;
//    data[1] = 0x00;
//    data[2] = 0x58;
//    LCD_WriteReg(hlcdc, 0xC3B9, data, 3);

//    data[0] = 0xBC;
//    data[1] = 0xD0;
//    data[2] = 0x54;
//    LCD_WriteReg(hlcdc, 0xC3C9, data, 3);

//    data[0] = 0xEF;
//    data[1] = 0x00;
//    data[2] = 0x58;
//    LCD_WriteReg(hlcdc, 0xC3D9, data, 3);

//    data[0] = 0xBC;
//    data[1] = 0xD0;
//    data[2] = 0x54;
//    LCD_WriteReg(hlcdc, 0xC3E9, data, 3);

//    data[0] = 0x01;
//    LCD_WriteReg(hlcdc, 0xC49A, data, 1);

    data[0] = 0x39;
    LCD_WriteReg(hlcdc, 0xF381, data, 1);

    data[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xC990, data, 1);

#ifdef BSP_LCDC_USING_DDR_QADSPI
    //60fps default
#else
#if BSP_USING_PSRAM
    //50fps
    data[0] = 0x6e;
    LCD_WriteReg(hlcdc, 0xC083, data, 1);

    data[0] = 0x6e;
    LCD_WriteReg(hlcdc, 0xC099, data, 1);
#else
    //40fps
    data[0] = 0x01;

    data[1] = 0x04;

    LCD_WriteReg(hlcdc, 0xC082, data, 2);

    data[0] = 0x01;

    data[1] = 0x04;

    LCD_WriteReg(hlcdc, 0xC098, data, 2);
#endif
#endif

    data[0] = 0x81;
    LCD_WriteReg(hlcdc, 0x9600, data, 1);


    if (LCDC_INTF_SPI_DCX_DDR_4DATA == lcdc_int_cfg.lcd_itf)
    {
        data[0] = 0x81;
        LCD_WriteReg(hlcdc, 0xb181, data, 1);
    }

    data[0] = 0x00;
    data[1] = 0x00;
    LCD_WriteReg(hlcdc, 0xFF80, data, 2);


    data[0] = 0x00;
    data[1] = 0x00;
    LCD_WriteReg(hlcdc, 0xFF00, data, 2);


//    data[0] = 0x92;
//    LCD_WriteReg(hlcdc, 0xD804, data, 1);
//    data[0] = 0xff;
//    LCD_WriteReg(hlcdc, 0x5100, data, 1);

//    data[0] = 0xC0;
//    LCD_WriteReg(hlcdc, 0xC470, data, 1);

    if (LCDC_PIXEL_FORMAT_RGB888 == lcdc_int_cfg.color_mode)
        data[0] = 0x77; //24bit rgb
    else if (LCDC_PIXEL_FORMAT_RGB565 == lcdc_int_cfg.color_mode)
        data[0] = 0x55; //16bit rgb
    else
        RT_ASSERT(0); //fix me

    LCD_WriteReg(hlcdc, 0x3A00, data, 1);

    data[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x3500, data, 1);

    LCD_WriteReg(hlcdc, 0x1100, NULL, 0);

    rt_thread_delay(60);

    LCD_WriteReg(hlcdc, 0x2900, NULL, 0);

    rt_thread_delay(10);

//    data[0] = 0x23;
//    data[1] = 0x08;
//    data[2] = 0x01;
//    data[3] = 0x01;
//    LCD_WriteReg(hlcdc, 0xFF00, data, 4);



//    rt_kprintf("LCD_Init done\r\n");
}
#else
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t parameter[32];

    rt_kprintf("LCD_Init\r\n");
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_spi, sizeof(lcdc_int_cfg));

    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    LCD_DRIVER_DELAY_MS(20);
    BSP_LCD_Reset(1);
    LCD_DRIVER_DELAY_MS(30);
    uint16_t num = 0;

    uint8_t data[40] = {0};
    data[0] = 0x23;
    data[1] = 0x08;
    data[2] = 0x01;
    LCD_WriteReg(hlcdc, 0xFF00, data, 3);
    data[0] = 0x23;
    data[1] = 0x08;
    LCD_WriteReg(hlcdc, 0xFF80, data, 2);
    data[0] = 0x63;
    LCD_WriteReg(hlcdc, 0xb282, data, 1);
    data[0] = 0x55;
    LCD_WriteReg(hlcdc, 0x3a00, data, 1);
    data[0] = 0xff;
    data[1] = 0x00;
    LCD_WriteReg(hlcdc, 0x5100, data, 2);
    data[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x3500, data, 1);
    data[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xc990, data, 1);

    LCD_WriteReg(hlcdc, 0x1100, NULL, 0);
    LCD_DRIVER_DELAY_MS(120);

    LCD_WriteReg(hlcdc, 0x2900, NULL, 0);
    data[0] = 0x23;
    data[1] = 0x08;
    data[2] = 0x01;
    data[3] = 0x01;
    LCD_WriteReg(hlcdc, 0xFF00, data, 4);

    rt_kprintf("LCD_Init done\r\n");
}
#endif


/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
    uint32_t data;

    data = LCD_ReadData(hlcdc, 0xda00, 2);
    rt_kprintf("\nft2308_ReadID0 0x%x \n", data);

    data = LCD_ReadData(hlcdc, REG_LCD_ID, 2);
    rt_kprintf("\nft2308_ReadID1 0x%x \n", data);

    data = LCD_ReadData(hlcdc, 0xa800, 4);
    rt_kprintf("\nft2308_ReadID1 0x%x \n", data);

    data = THE_LCD_ID; //GC FAE: NOT support read id now
    return data;

}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOn(LCDC_HandleTypeDef *hlcdc)
{
    /* Display On */
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
static void LCD_DisplayOff(LCDC_HandleTypeDef *hlcdc)
{
    /* Display Off */
    LCD_WriteReg(hlcdc, REG_DISPLAY_OFF, (uint8_t *)NULL, 0);
    LCD_WriteReg(hlcdc, REG_SLEEP_IN, (uint8_t *)NULL, 0);
}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint8_t   parameter[4];

    HAL_LCDC_SetROIArea(hlcdc, Xpos0, Ypos0, Xpos1, Ypos1);

    Xpos0 += COL_OFFSET;
    Xpos1 += COL_OFFSET;

    Ypos0 += ROW_OFFSET;
    Ypos1 += ROW_OFFSET;

    parameter[0] = (Xpos0) >> 8;
    parameter[1] = (Xpos0) & 0xFF;
    parameter[2] = (Xpos1) >> 8;
    parameter[3] = (Xpos1) & 0xFF;
    LCD_WriteReg(hlcdc, REG_CASET, parameter, 4);

    parameter[0] = (Ypos0) >> 8;
    parameter[1] = (Ypos0) & 0xFF;
    parameter[2] = (Ypos1) >> 8;
    parameter[3] = (Ypos1) & 0xFF;
    LCD_WriteReg(hlcdc, REG_RASET, parameter, 4);
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
    uint8_t data = 0;

    /* Set Cursor */
    LCD_SetRegion(hlcdc, Xpos, Ypos, Xpos, Ypos);
    LCD_WriteReg(hlcdc, REG_WRITE_RAM, (uint8_t *)RGBCode, 2);
}

#ifdef LCDC_WRITE_CROSS_TE
static void UpdateTeDelay(LCDC_HandleTypeDef *hlcdc)
{
    uint32_t te_period_us = 1000000 / 50;

    uint32_t lcd_read_time_us = te_period_us * hlcdc->roi.y1 / LCD_VER_RES_MAX;

    uint32_t lcdc_clk_per_pixel = (LCDC_PIXEL_FORMAT_RGB888 == lcdc_int_cfg_spi.color_mode) ? 6 : 4; //QSPI
    uint32_t lcdc_write_time_us = (hlcdc->roi.y1 - hlcdc->roi.y0 + 1) * (hlcdc->roi.x1 - hlcdc->roi.x0 + 1) * lcdc_clk_per_pixel / (lcdc_int_cfg_spi.freq / 1000000);

    //rt_kprintf("lcd_read_time_us %d  lcdc_write_time_us %d  \r\n", lcd_read_time_us, lcdc_write_time_us);


    if (lcdc_write_time_us > lcd_read_time_us)
    {
        hlcdc->Init.cfg.spi.vsyn_delay_us = lcdc_int_cfg_spi.cfg.spi.vsyn_delay_us;
    }
    else
    {
        hlcdc->Init.cfg.spi.vsyn_delay_us = lcdc_int_cfg_spi.cfg.spi.vsyn_delay_us + lcd_read_time_us - lcdc_write_time_us;
    }

    //rt_kprintf("UpdateTeDelay %d  \r\n", hlcdc->Init.cfg.spi.vsyn_delay_us);
}
#endif

static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint32_t size;

    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);
    // rt_kprintf("ft2308_WriteMultiplePixels:RGBCode:%x,x1:%d,y1:%d,x2:%d,y2:%d\r\n", RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);

    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
#ifdef LCDC_WRITE_CROSS_TE
        UpdateTeDelay(hlcdc);
#endif
        HAL_LCDC_SendLayerData2Reg_IT(hlcdc, ((0x32 << 24) | REG_WRITE_RAM), 4);
    }
    else
    {
        HAL_LCDC_SendLayerData2Reg_IT(hlcdc, REG_WRITE_RAM, 1);
    }
}


/**
  * @brief  Writes  to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters)
{
#if 0
    if (LCD_Reg == REG_CASET)
    {
        DEBUG_PRINTF("ft2308_SetX[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
    else if (LCD_Reg == REG_RASET)
    {
        DEBUG_PRINTF("ft2308_SetY[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
#endif

    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        uint32_t cmd;

        // cmd = (0x02 << 24) | (LCD_Reg << 8);
        cmd = (0x02 << 24) | (LCD_Reg);

        if (0 != NbParameters)
        {
            /* Send command's parameters if any */
            HAL_LCDC_WriteU32Reg(hlcdc, cmd, Parameters, NbParameters);
//          rt_kprintf("LCD_WriteReg:cmd:0x%x,data:0x%x,size:%x\r\n",cmd,Parameters,NbParameters);
        }
        else
        {
            uint32_t v = 0;
            HAL_LCDC_WriteU32Reg(hlcdc, cmd, (uint8_t *)&v, 1);
//          rt_kprintf("LCD_WriteReg:cmd:0x%x,0x%x,%x\r\n",cmd,v,1);
        }

    }
    else
    {
        HAL_LCDC_WriteU8Reg(hlcdc, LCD_Reg, Parameters, NbParameters);
    }
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

    LCD_ReadMode(hlcdc, true);
    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        HAL_LCDC_ReadU32Reg(hlcdc, ((0x03 << 24) | RegValue), (uint8_t *)&rd_data, ReadSize);

        rt_kprintf("LCD_ReadData:reg:0x%x,data:0x%x,size:0x%x\r\n", ((0x03 << 24) | RegValue), rd_data, ReadSize);
    }
    else
    {
        HAL_LCDC_ReadU8Reg(hlcdc, RegValue, (uint8_t *)&rd_data, ReadSize);
    }
    LCD_ReadMode(hlcdc, false);
    return rd_data;
}



static uint32_t LCD_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos)
{
    uint8_t  r, g, b;
    uint32_t ret_v, read_value;

    DEBUG_PRINTF("ft2308 NOT support read pixel!");

    return 0;


    DEBUG_PRINTF("ft2308_ReadPixel[%d,%d]\n", Xpos, Ypos);


    LCD_SetRegion(hlcdc, Xpos, Ypos, Xpos, Ypos);

    read_value = LCD_ReadData(hlcdc, REG_READ_RAM, 4);
    DEBUG_PRINTF("result: [%x]\n", read_value);

    b = (read_value >> 0) & 0xFF;
    g = (read_value >> 8) & 0xFF;
    r = (read_value >> 16) & 0xFF;

    DEBUG_PRINTF("r=%d, g=%d, b=%d \n", r, g, b);

    switch (lcdc_int_cfg.color_mode)
    {
    case LCDC_PIXEL_FORMAT_RGB565:
        ret_v = (uint32_t)(((r << 11) & 0xF800) | ((g << 5) & 0x7E0) | ((b >> 3) & 0X1F));
        break;

    /*
       (8bit R + 3bit dummy + 8bit G + 3bit dummy + 8bit B)

    */
    case LCDC_PIXEL_FORMAT_RGB888:
        ret_v = (uint32_t)(((r << 16) & 0xFF0000) | ((g << 8) & 0xFF00) | ((b) & 0XFF));
        break;

    default:
        RT_ASSERT(0);
        break;
    }


    //LCD_WriteReg(hlcdc,REG_COLOR_MODE, parameter, 1);

    return ret_v;
}


static void LCD_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode)
{
    uint8_t   parameter[2];


    /*

    Control interface color format
    ��011�� = 12bit/pixel ��101�� = 16bit/pixel ��110�� = 18bit/pixel ��111�� = 16M truncated

    */
    switch (color_mode)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        /* Color mode 16bits/pixel */
        parameter[0] = 0x55;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB565;
        break;

    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        parameter[0] = 0x77;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB888;
        break;

    default:
        return; //unsupport
        break;
    }

    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    uint32_t data = LCD_ReadData(hlcdc, 0x0c00, 1);
    DEBUG_PRINTF("\nft2308_color_format 0x%x \n", data);

    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

#define ft2308_BRIGHTNESS_MAX 0xFF

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = (uint8_t)((int)ft2308_BRIGHTNESS_MAX * br / 100);
    LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);
}



static void LCD_TimeoutDbg(LCDC_HandleTypeDef *hlcdc)
{
    uint32_t data;

    HAL_LCDC_Init(hlcdc);

    data = LCD_ReadData(hlcdc, 0xda00, 2);
    rt_kprintf("\nft2308_ReadID0 0x%x \n", data);

    data = LCD_ReadData(hlcdc, REG_LCD_ID, 2);
    rt_kprintf("\nft2308_ReadID1 0x%x \n", data);

    data = LCD_ReadData(hlcdc, 0xa800, 4);
    rt_kprintf("\nft2308_ReadID1 0x%x \n", data);
}

static void LCD_TimeoutReset(LCDC_HandleTypeDef *hlcdc)
{
    BSP_LCD_Reset(0);//Reset LCD
    rt_thread_mdelay(100);
    BSP_LCD_Reset(1);
    rt_thread_mdelay(10);

    LCD_Init(hlcdc);
}

static uint32_t LCD_ESDCehck(LCDC_HandleTypeDef *hlcdc)
{
    uint32_t data;

    data = LCD_ReadData(hlcdc, REG_POWER_MODE, 2);
    rt_kprintf("\nft2308_ESDCehck 0x%x \n", data);

    return 0;

}


static const LCD_DrvOpsDef ft2308_drv =
{
    LCD_Init,
    LCD_ReadID,
    LCD_DisplayOn,
    LCD_DisplayOff,

    LCD_SetRegion,
    LCD_WritePixel,
    LCD_WriteMultiplePixels,
    LCD_ReadPixel,

    LCD_SetColorMode,
    LCD_SetBrightness,
    NULL,
    NULL,
    NULL,
    LCD_TimeoutDbg,
    LCD_TimeoutReset,
    LCD_ESDCehck
};

LCD_DRIVER_EXPORT2(ft2308, THE_LCD_ID, &lcdc_int_cfg,
                   &ft2308_drv, 2);
