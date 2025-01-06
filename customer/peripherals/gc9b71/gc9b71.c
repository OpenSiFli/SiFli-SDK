/**
  ******************************************************************************
  * @file   gc9b71.c
  * @author Sifli software development team
  * @brief   This file includes the LCD driver for GC9B71 LCD.
  * @attention
  ******************************************************************************
*/


#include <rtthread.h>
#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"

#define DBG_TAG               "gc9b71"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>










#define ROW_OFFSET  (0x00)
#define COL_OFFSET  (0x00)



/**
  * @brief GC9B71 chip IDs
  */
#define THE_LCD_ID                  0x1190a7

/**
  * @brief  GC9B71 Size
  */
#define  THE_LCD_PIXEL_WIDTH    (320)
#define  THE_LCD_PIXEL_HEIGHT   (390)






/**
  * @brief  GC9B71 Registers
  */
//#define REG_SW_RESET           0x01
#define REG_LCD_ID             0xA1
//#define REG_DSI_ERR            0x05
#define REG_POWER_MODE         0x0A
#define REG_SLEEP_IN           0x10
#define REG_SLEEP_OUT          0x11
//#define REG_PARTIAL_DISPLAY    0x12
//#define REG_DISPLAY_INVERSION  0x21
#define REG_DISPLAY_OFF        0x28
#define REG_DISPLAY_ON         0x29
#define REG_WRITE_RAM          0x2C
#define REG_READ_RAM           0x2E
#define REG_CASET              0x2A
#define REG_RASET              0x2B




#define REG_TEARING_EFFECT     0x35
#define REG_MACTL     0x36
//#define REG_IDLE_MODE_OFF      0x38
//#define REG_IDLE_MODE_ON       0x39
#define REG_COLOR_MODE         0x3A

#define REG_WBRIGHT            0x51 /* Write brightness*/

























//#define LCD_3V3_POWER_PIN               55
//#define LCD_1V8_POWER_PIN               47
//#ifdef BSP_USING_BOARD_EC_SS6700XXX
//    #define LCD_BACKLIGHT_POWER_PIN         31
//#endif
//#define LCD_RESET_PIN                   78     // GPIO_A44  gc9b71


#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

/*gc9b71 start colume & row must can be divided by 2, and roi width&height too.*/
#define LCD_ALIGN2(x) ((x) = (x) & (~1))
#define LCD_ALIGN1(x) ((x) = (0 == ((x) & 1)) ? (x - 1) : x)








#define QAD_SPI_ITF LCDC_INTF_SPI_DCX_4DATA


static const LCDC_InitTypeDef lcdc_int_cfg_qadspi =
{
    .lcd_itf = QAD_SPI_ITF, // LCDC_INTF_SPI_NODCX_1DATA
    .freq = 48000000,        //GC9B71 RGB565 only support 48000000,  RGB888 support 60000000
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,//LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 1,
#ifdef LCD_GC9B71_VSYNC_ENABLE
            .syn_mode = HAL_LCDC_SYNC_VER,
#else
            .syn_mode = HAL_LCDC_SYNC_DISABLE,
#endif /* LCD_GC9B71_VSYNC_ENABLE */
            .vsyn_polarity = 1,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 0,
            .hsyn_num = 0,
        },
    },

};


static LCDC_InitTypeDef lcdc_int_cfg;

static void     LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable);
static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1);









#ifdef QAD_SPI_USE_GPIO_CS
    void gpio_cs_enable(void);
    void gpio_cs_disable(void);
#endif /* QAD_SPI_USE_GPIO_CS */



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

void GC9B71_Clear(LCDC_HandleTypeDef *hlcdc)
{
    /*Clear gram*/
    HAL_LCDC_Next_Frame_TE(hlcdc, 0);
    LCD_SetRegion(hlcdc, 0, 0, THE_LCD_PIXEL_WIDTH, THE_LCD_PIXEL_HEIGHT);
    HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB565);
    HAL_LCDC_LayerDisable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
    HAL_LCDC_SetBgColor(hlcdc, 0, 0, 0);
    HAL_LCDC_SendLayerData2Reg(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
    HAL_LCDC_LayerEnable(hlcdc, HAL_LCDC_LAYER_DEFAULT);

}


static void GC9B71_Drv_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[32];

    /* Initialize GC9B71 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

#if 0
    rt_pin_mode(LCD_3V3_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LCD_3V3_POWER_PIN, 1); /* LCD_3V3 ON */
    rt_pin_mode(LCD_1V8_POWER_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LCD_1V8_POWER_PIN, 0); /* LCD_1V8 ON */
#endif
    //rt_pin_mode(LCD_BACKLIGHT_POWER_PIN, PIN_MODE_OUTPUT);
    //rt_pin_write(LCD_BACKLIGHT_POWER_PIN, 0);

    BSP_LCD_Reset(0);//Reset LCD
    HAL_Delay_us(20);
    BSP_LCD_Reset(1);

    /* Wait for 200ms */
    LCD_DRIVER_DELAY_MS(200);

#ifdef LCD_USING_ED_LB5XSPI18501

    uint8_t data[40] = {0};

    // AppLcdWriteCmd(0x28);
    // AppLcdWriteCmd(0x10);
    // AppTaskDelayMs(100);

    ///AppLcdWriteCmd(0xfe);
    //AppLcdWriteCmd(0xef);
    LCD_WriteReg(hlcdc, 0xfe, (uint8_t *)NULL, 0);
    LCD_WriteReg(hlcdc, 0xef, (uint8_t *)NULL, 0);
    data[0] = 0x11;
    LCD_WriteReg(hlcdc, 0x80, data, 1);

    data[0] = 0x30; //
    LCD_WriteReg(hlcdc, 0x81, data, 1);

    data[0] = 0x09;
    LCD_WriteReg(hlcdc, 0x82, data, 1);

    data[0] = 0x03;
    LCD_WriteReg(hlcdc, 0x83, data, 1);

    data[0] = 0x22; //
    LCD_WriteReg(hlcdc, 0x84, data, 1);

    data[0] = 0x18;
    LCD_WriteReg(hlcdc, 0x89, data, 1);

    data[0] = 0x40;
    LCD_WriteReg(hlcdc, 0x8A, data, 1);

    data[0] = 0x0A;
    LCD_WriteReg(hlcdc, 0x8B, data, 1);

    data[0] = 0x05;
    LCD_WriteReg(hlcdc, 0x3a, data, 1);

    data[0] = 0x40;
    LCD_WriteReg(hlcdc, 0x36, data, 1);

    data[0] = 0x07;
    LCD_WriteReg(hlcdc, 0xEC, data, 1);

    data[0] = 0x01;
    data[1] = 0x80;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    LCD_WriteReg(hlcdc, 0x74, data, 6);

    data[0] = 0x3E;
    LCD_WriteReg(hlcdc, 0x98, data, 1);

    data[0] = 0x3E;
    LCD_WriteReg(hlcdc, 0x99, data, 1);

    data[0] = 0x01;
    data[1] = 0x04;
    LCD_WriteReg(hlcdc, 0xA1, data, 2);

    data[0] = 0x01;
    data[1] = 0x04;
    LCD_WriteReg(hlcdc, 0xA2, data, 2);

    // data[0] = 0x2B;
    // AppLcdWriteData(0xA3, data, 1); // 00 30Hz, 2b 40Hz

    data[0] = 0x02;
    LCD_WriteReg(hlcdc, 0xCB, data, 1);

    data[0] = 0xB6;
    data[1] = 0x24;
    LCD_WriteReg(hlcdc, 0x7C, data, 2);

    data[0] = 0x74;
    LCD_WriteReg(hlcdc, 0xAC, data, 1);

    data[0] = 0x80;
    LCD_WriteReg(hlcdc, 0xF6, data, 1);

    data[0] = 0x09;
    data[1] = 0x09;
    LCD_WriteReg(hlcdc, 0xB5, data, 2);

    data[0] = 0x01;
    data[1] = 0x81; // 81
    LCD_WriteReg(hlcdc, 0xEB, data, 2);

    data[0] = 0x38;
    data[1] = 0x06; // 0c
    data[2] = 0x13;
    data[3] = 0x56;
    LCD_WriteReg(hlcdc, 0x60, data, 4);

    data[0] = 0x38;
    data[1] = 0x08; // e
    data[2] = 0x13;
    data[3] = 0x56;
    LCD_WriteReg(hlcdc, 0x63, data, 4);

    data[0] = 0x3B;
    data[1] = 0x1b;
    data[2] = 0x58;
    data[3] = 0x38;
    LCD_WriteReg(hlcdc, 0x61, data, 4);

    data[0] = 0x3B;
    data[1] = 0x1b;
    data[2] = 0x58;
    data[3] = 0x38;
    LCD_WriteReg(hlcdc, 0x62, data, 4);

    data[0] = 0x38;
    data[1] = 0x0a;
    data[2] = 0x73;
    data[3] = 0x16;
    data[4] = 0x13;
    data[5] = 0x56;
    LCD_WriteReg(hlcdc, 0x64, data, 6);

    data[0] = 0x38;
    data[1] = 0x0b;
    data[2] = 0x73;
    data[3] = 0x17;
    data[4] = 0x13;
    data[5] = 0x56;
    LCD_WriteReg(hlcdc, 0x66, data, 6);

    data[0] = 0x00;
    data[1] = 0x0B;
    data[2] = 0x22;
    data[3] = 0x0B;
    data[4] = 0x22;
    data[5] = 0x1C;
    data[6] = 0x1C;
    LCD_WriteReg(hlcdc, 0x68, data, 7);

    data[0] = 0x00;
    data[1] = 0x0B;
    data[2] = 0x26;
    data[3] = 0x0B;
    data[4] = 0x26;
    data[5] = 0x1C;
    data[6] = 0x1C;
    LCD_WriteReg(hlcdc, 0x69, data, 7);

    data[0] = 0x15;
    data[1] = 0x00;
    LCD_WriteReg(hlcdc, 0x6A, data, 2);

    data[0] = 0x08;
    data[1] = 0x02;
    data[2] = 0x1a;
    data[3] = 0x00;
    data[4] = 0x12;
    data[5] = 0x12;
    data[6] = 0x11;
    data[7] = 0x11;
    data[8] = 0x14;
    data[9] = 0x14;
    data[10] = 0x13;
    data[11] = 0x13;
    data[12] = 0x04;
    data[13] = 0x19;
    data[14] = 0x1e;
    data[15] = 0x1d;
    data[16] = 0x1d;
    data[17] = 0x1e;
    data[18] = 0x19;
    data[19] = 0x04;
    data[20] = 0x0b;
    data[21] = 0x0b;
    data[22] = 0x0c;
    data[23] = 0x0c;
    data[24] = 0x09;
    data[25] = 0x09;
    data[26] = 0x0a;
    data[27] = 0x0a;
    data[28] = 0x00;
    data[29] = 0x1a;
    data[30] = 0x01;
    data[31] = 0x07;
    LCD_WriteReg(hlcdc, 0x6E, data, 32);

    data[0] = 0xCC;
    data[1] = 0x0C;
    data[2] = 0xCC;
    data[3] = 0x84;
    data[4] = 0xCC;
    data[5] = 0x04;
    data[6] = 0x50;
    LCD_WriteReg(hlcdc, 0x6C, data, 7);

    data[0] = 0x72;
    LCD_WriteReg(hlcdc, 0x7D, data, 1);

    data[0] = 0x02;
    data[1] = 0x03;
    data[2] = 0x09;
    data[3] = 0x07;
    data[4] = 0x09;
    data[5] = 0x03;
    data[6] = 0x09;
    data[7] = 0x07;
    data[8] = 0x09;
    data[9] = 0x03;
    LCD_WriteReg(hlcdc, 0x70, data, 10);

    data[0] = 0x06;
    data[1] = 0x06;
    data[2] = 0x05;
    data[3] = 0x06;
    LCD_WriteReg(hlcdc, 0x90, data, 4);

    data[0] = 0x45;
    data[1] = 0xFF;
    data[2] = 0x00;
    LCD_WriteReg(hlcdc, 0x93, data, 3);

    data[0] = 0x15;
    LCD_WriteReg(hlcdc, 0xC3, data, 1);

    data[0] = 0x36;
    LCD_WriteReg(hlcdc, 0xC4, data, 1);

    data[0] = 0x3d;
    LCD_WriteReg(hlcdc, 0xC9, data, 1);

    data[0] = 0x47; // gVR1_N[5:0]
    data[1] = 0x07; // gVR2_N[5:0]
    data[2] = 0x0A; // gVR4_N[4:0]
    data[3] = 0x0A; // gVR6_N[4:0]
    data[4] = 0x00; // gVR0_N[3:0] gVR13_N[3:0] 7
    data[5] = 0x29; // gVR20_N[6:0]
    LCD_WriteReg(hlcdc, 0xF0, data, 6);

    data[0] = 0x47; // gVR1_P[5:0]
    data[1] = 0x07; // gVR2_P[5:0]
    data[2] = 0x0a; // gVR4_P[4:0]
    data[3] = 0x0A; // gVR6_P[4:0]
    data[4] = 0x00; // gVR0_P[3:0] gVR13_P[3:0]
    data[5] = 0x29; // gVR20_P[6:0]
    LCD_WriteReg(hlcdc, 0xF2, data, 6);

    data[0] = 0x42; // gVR43_N[6:0]
    data[1] = 0x91; // gVR27_N[2:0] gVR57_N[4:0]
    data[2] = 0x10; // gVR36_N[2:0] gVR59_N[4:0]
    data[3] = 0x2D; // gVR61_N[5:0]
    data[4] = 0x2F; // gVR62_N[5:0]
    data[5] = 0x6F; // gVR50_N[3:0] gVR63_N[3:0]
    LCD_WriteReg(hlcdc, 0xF1, data, 6);

    data[0] = 0x42; // gVR43_P[6:0]
    data[1] = 0x91; // gVR27_P[2:0] gVR57_P[4:0]
    data[2] = 0x10; // gVR36_P[2:0] gVR59_P[4:0]
    data[3] = 0x2D; // gVR61_P[5:0]
    data[4] = 0x2F; // gVR62_P[5:0]
    data[5] = 0x6F; // gVR50_P[3:0] gVR63_P[3:0]
    LCD_WriteReg(hlcdc, 0xF3, data, 6);

    data[0] = 0x30;
    LCD_WriteReg(hlcdc, 0xF9, data, 1);

    data[0] = 0x11;
    LCD_WriteReg(hlcdc, 0xBE, data, 1);

    data[0] = 0x00;
    data[1] = 0x00;
    LCD_WriteReg(hlcdc, 0xFB, data, 2);

    data[1] = 0x0a;
    LCD_WriteReg(hlcdc, 0xb4, data, 1);

    data[1] = 0x00;
    LCD_WriteReg(hlcdc, 0x35, data, 1);

    data[0] = 0x00;
    data[1] = 0x00;
    LCD_WriteReg(hlcdc, 0x44, data, 2);



    LCD_DRIVER_DELAY_MS(10);

    LCD_WriteReg(hlcdc, REG_SLEEP_OUT, (uint8_t *)NULL, 0);

    LCD_DRIVER_DELAY_MS(120); //Delay 5 ms after sleep out



    HAL_LCDC_Next_Frame_TE(hlcdc, 0);
    LCD_SetRegion(hlcdc, 0, 0, 319, 385);
    HAL_LCDC_LayerSetFormat(hlcdc, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB565);
    HAL_LCDC_LayerDisable(hlcdc, HAL_LCDC_LAYER_DEFAULT);
    HAL_LCDC_SetBgColor(hlcdc, 0, 0, 0);
    HAL_LCDC_SendLayerData2Reg(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
    HAL_LCDC_LayerEnable(hlcdc, HAL_LCDC_LAYER_DEFAULT);

    /* Display ON command */
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);

    LCD_DRIVER_DELAY_MS(50); //Wait TE signal ready

#else

    //Initail
    LCD_WriteReg(hlcdc, 0xfe, parameter, 0); // internal reg enable
    LCD_WriteReg(hlcdc, 0xef, parameter, 0); // internal reg enable


    parameter[0] = 0x11;
    LCD_WriteReg(hlcdc, 0x80, parameter, 1);


    parameter[0] = 0x70;
    LCD_WriteReg(hlcdc, 0x81, parameter, 1);


    parameter[0] = 0x09;
    LCD_WriteReg(hlcdc, 0x82, parameter, 1);


    parameter[0] = 0x03;
    LCD_WriteReg(hlcdc, 0x83, parameter, 1);


    parameter[0] = 0x22;
    LCD_WriteReg(hlcdc, 0x84, parameter, 1);


    parameter[0] = 0x18;
    LCD_WriteReg(hlcdc, 0x89, parameter, 1);


    parameter[0] = 0x40;
    LCD_WriteReg(hlcdc, 0x8A, parameter, 1);


    parameter[0] = 0x0A;
    LCD_WriteReg(hlcdc, 0x8B, parameter, 1);


    parameter[0] = 0x05;
    LCD_WriteReg(hlcdc, 0x3a, parameter, 1);


    parameter[0] = 0x40;
    LCD_WriteReg(hlcdc, 0x36, parameter, 1);


    parameter[0] = 0x07;
    LCD_WriteReg(hlcdc, 0xEC, parameter, 1);


    parameter[0] = 0x01;
    parameter[1] = 0x80;
    parameter[2] = 0x00;
    parameter[3] = 0x00;
    parameter[4] = 0x00;
    parameter[5] = 0x00;
    LCD_WriteReg(hlcdc, 0x74, parameter, 6);


    parameter[0] = 0x3E;
    LCD_WriteReg(hlcdc, 0x98, parameter, 1);


    parameter[0] = 0x3E;
    LCD_WriteReg(hlcdc, 0x99, parameter, 1);



    parameter[0] = 0x01;
    parameter[1] = 0x04;
    LCD_WriteReg(hlcdc, 0xA1, parameter, 2);


    parameter[0] = 0x01;
    parameter[1] = 0x04;
    LCD_WriteReg(hlcdc, 0xA2, parameter, 2);


    parameter[0] = 0x02;
    LCD_WriteReg(hlcdc, 0xCB, parameter, 1);


    parameter[0] = 0x44;
    LCD_WriteReg(hlcdc, 0xAC, parameter, 1);


    parameter[0] = 0xB6;
    parameter[1] = 0x25;
    LCD_WriteReg(hlcdc, 0x7C, parameter, 2);


    parameter[0] = 0x80;
    LCD_WriteReg(hlcdc, 0xF6, parameter, 1);


    parameter[0] = 0x09;
    parameter[1] = 0x09;
    LCD_WriteReg(hlcdc, 0xB5, parameter, 2);


    parameter[0] = 0x01;
    parameter[1] = 0x7B;
    LCD_WriteReg(hlcdc, 0xEB, parameter, 2);


    parameter[0] = 0x58;
    parameter[1] = 0x09;
    parameter[2] = 0x5B;
    parameter[3] = 0x56;
    LCD_WriteReg(hlcdc, 0x60, parameter, 4);


    parameter[0] = 0x5B;
    parameter[1] = 0x05;
    parameter[2] = 0x5B;
    parameter[3] = 0x56;
    LCD_WriteReg(hlcdc, 0x63, parameter, 4);


    parameter[0] = 0x38;
    parameter[1] = 0x0F;
    parameter[2] = 0x73;
    parameter[3] = 0x17;
    parameter[4] = 0x5B;
    parameter[5] = 0x56;
    LCD_WriteReg(hlcdc, 0x64, parameter, 6);


    parameter[0] = 0x38;
    parameter[1] = 0x0F;
    parameter[2] = 0x73;
    parameter[3] = 0x17;
    parameter[4] = 0x5B;
    parameter[5] = 0x56;
    LCD_WriteReg(hlcdc, 0x66, parameter, 6);


    parameter[0] = 0x38;
    parameter[1] = 0x0B;
    parameter[2] = 0x73;
    parameter[3] = 0x13;
    parameter[4] = 0x5B;
    parameter[5] = 0x56;
    LCD_WriteReg(hlcdc, 0x65, parameter, 6);


    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x00;
    parameter[3] = 0x00;
    parameter[4] = 0x00;
    parameter[5] = 0x00;
    parameter[6] = 0x00;
    parameter[7] = 0x01;
    parameter[8] = 0x0D;
    parameter[9] = 0x0F;
    parameter[10] = 0x09;
    parameter[11] = 0x0B;
    parameter[12] = 0x07;
    parameter[13] = 0x00;
    parameter[14] = 0x00;
    parameter[15] = 0x00;
    parameter[16] = 0x00;
    parameter[17] = 0x00;
    parameter[18] = 0x00;
    parameter[19] = 0x08;
    parameter[20] = 0x14;
    parameter[21] = 0x12;
    parameter[22] = 0x10;
    parameter[23] = 0x0E;
    parameter[24] = 0x02;
    parameter[25] = 0x00;
    parameter[26] = 0x00;
    parameter[27] = 0x00;
    parameter[28] = 0x00;
    parameter[29] = 0x00;
    parameter[30] = 0x00;
    parameter[31] = 0x00;
    LCD_WriteReg(hlcdc, 0x6E, parameter, 32);


    parameter[0] = 0xCC;
    parameter[1] = 0x0C;
    parameter[2] = 0xCC;
    parameter[3] = 0x84;
    parameter[4] = 0xCC;
    parameter[5] = 0x04;
    parameter[6] = 0x50;
    LCD_WriteReg(hlcdc, 0x6C, parameter, 7);


    parameter[0] = 0x72;
    LCD_WriteReg(hlcdc, 0x7D, parameter, 1);


    parameter[0] = 0x02;
    parameter[1] = 0x03;
    parameter[2] = 0x09;
    parameter[3] = 0x04;
    parameter[4] = 0x0C;
    parameter[5] = 0x06;
    parameter[6] = 0x09;
    parameter[7] = 0x05;
    parameter[8] = 0x0C;
    parameter[9] = 0x06;
    LCD_WriteReg(hlcdc, 0x70, parameter, 10);


    parameter[0] = 0x06;
    parameter[1] = 0x06;
    parameter[2] = 0x05;
    parameter[3] = 0x06;
    LCD_WriteReg(hlcdc, 0x90, parameter, 4);


    parameter[0] = 0x45;
    parameter[1] = 0xFF;
    parameter[2] = 0x00;
    LCD_WriteReg(hlcdc, 0x93, parameter, 3);


    parameter[0] = 0x40;
    LCD_WriteReg(hlcdc, 0xC3, parameter, 1);


    parameter[0] = 0x60;
    LCD_WriteReg(hlcdc, 0xC4, parameter, 1);


    parameter[0] = 0x3d;
    LCD_WriteReg(hlcdc, 0xC9, parameter, 1);


    parameter[0] = 0x47;
    parameter[1] = 0x07;
    parameter[2] = 0x08;
    parameter[3] = 0x02;
    parameter[4] = 0x00;
    parameter[5] = 0x27;
    LCD_WriteReg(hlcdc, 0xF0, parameter, 6);


    parameter[0] = 0x47;
    parameter[1] = 0x08;
    parameter[2] = 0x07;
    parameter[3] = 0x02;
    parameter[4] = 0x00;
    parameter[5] = 0x27;
    LCD_WriteReg(hlcdc, 0xF2, parameter, 6);


    parameter[0] = 0x41;
    parameter[1] = 0xaA;
    parameter[2] = 0x5a;
    parameter[3] = 0x28;
    parameter[4] = 0x2C;
    parameter[5] = 0xeF;
    LCD_WriteReg(hlcdc, 0xF1, parameter, 6);


    parameter[0] = 0x41;
    parameter[1] = 0x9A;
    parameter[2] = 0x5a;
    parameter[3] = 0x28;
    parameter[4] = 0x2C;
    parameter[5] = 0xeF;
    LCD_WriteReg(hlcdc, 0xF3, parameter, 6);


    parameter[0] = 0x30;
    LCD_WriteReg(hlcdc, 0xF9, parameter, 1);


    parameter[0] = 0x11;
    LCD_WriteReg(hlcdc, 0xBE, parameter, 1);



    parameter[0] = 0x00;
    parameter[1] = 0x02;
    LCD_WriteReg(hlcdc, 0xFB, parameter, 2);


    parameter[0] = 0xb0;
    parameter[1] = 0x00;
    LCD_WriteReg(hlcdc, 0xAA, parameter, 2);



    LCD_WriteReg(hlcdc, REG_SLEEP_OUT, (uint8_t *)NULL, 0);

    /* Wait for 100ms */
    LCD_DRIVER_DELAY_MS(100);
    GC9B71_Clear(hlcdc);
    /* Display ON command */
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);

    LCD_DRIVER_DELAY_MS(150); //Wait TE signal ready

    //LCD_WriteReg(hlcdc,0x23, (uint8_t *)NULL, 0);

    /* Tearing Effect Line On: Option (00h:VSYNC Only, 01h:VSYNC & HSYNC ) */
    parameter[0] = 0x02;
    LCD_WriteReg(hlcdc, REG_TEARING_EFFECT, parameter, 1);
#endif
    return;
}

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
 */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_qadspi, sizeof(lcdc_int_cfg));

    for (uint32_t retry = 0; retry < 10; retry++)
    {
        GC9B71_Drv_Init(hlcdc);
        LOG_I("GC9B71_Init\r\n");
        {
            uint32_t data = 0;

            data = LCD_ReadData(hlcdc, REG_POWER_MODE, 2);
            LOG_I("GC9B71_Read POWER MODE 0x%x \n", data);

            if (data != 0)
            {
                break;//Success
            }
            else
            {
                LOG_I("GC9B71_Init retry %d", retry);
                BSP_LCD_PowerDown();
                LCD_DRIVER_DELAY_MS(100 * retry);
                BSP_LCD_PowerUp();
            }
        }
    }
    return;
}


/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
#ifdef DSI_HW_TEST
    return THE_LCD_ID;
#else

    uint32_t data;

    data = LCD_ReadData(hlcdc, REG_LCD_ID, 4);
    DEBUG_PRINTF("\nGC9B71_ReadID 0x%x \n", data);
    data = ((data << 24) & 0xFF000000)
           | ((data <<  8) & 0x00FF0000)
           | ((data >>  8) & 0x0000FF00)
           | ((data >> 24) & 0x000000FF);


    if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        return THE_LCD_ID;
    }
    return THE_LCD_ID;

    return data;
#endif /* DSI_HW_TEST */

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
}

static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint8_t   parameter[4];
    LCD_ALIGN2(Xpos0);
    LCD_ALIGN2(Ypos0);
    LCD_ALIGN1(Xpos1);
    LCD_ALIGN1(Ypos1);

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


    if ((Xpos >= THE_LCD_PIXEL_WIDTH) || (Ypos >= THE_LCD_PIXEL_HEIGHT))
    {
        return;
    }

    /* Set Cursor */
    LCD_SetRegion(hlcdc, Xpos, Ypos, Xpos, Ypos);
    LCD_WriteReg(hlcdc, REG_WRITE_RAM, (uint8_t *)RGBCode, 2);
}

static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc, const uint8_t *RGBCode, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint32_t size;


    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode, Xpos0, Ypos0, Xpos1, Ypos1);


    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        HAL_LCDC_SendLayerData2Reg_IT(hlcdc, ((0x32 << 24) | (REG_WRITE_RAM << 8)), 4);
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
        DEBUG_PRINTF("GC9B71_SetX[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
    else if (LCD_Reg == REG_RASET)
    {
        DEBUG_PRINTF("GC9B71_SetY[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
#endif

#ifdef QAD_SPI_USE_GPIO_CS
    gpio_cs_enable();
#endif /* QAD_SPI_USE_GPIO_CS */

    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        HAL_LCDC_WriteU32Reg(hlcdc, ((0x02 << 24) | (LCD_Reg << 8)), Parameters, NbParameters);
    }
    else
    {
        HAL_LCDC_WriteU8Reg(hlcdc, LCD_Reg, Parameters, NbParameters);
    }

#ifdef QAD_SPI_USE_GPIO_CS
    HAL_Delay_us(20); //wait lcdc sending single data finished
    gpio_cs_disable();
#endif /* QAD_SPI_USE_GPIO_CS */

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
#ifdef QAD_SPI_USE_GPIO_CS
    gpio_cs_enable();
#endif /* QAD_SPI_USE_GPIO_CS */

    LCD_ReadMode(hlcdc, true);
    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        HAL_LCDC_ReadU32Reg(hlcdc, ((0x03 << 24) | (RegValue << 8)), (uint8_t *)&rd_data, ReadSize);
    }
    else
    {
        HAL_LCDC_ReadU8Reg(hlcdc, RegValue, (uint8_t *)&rd_data, ReadSize);
    }
    LCD_ReadMode(hlcdc, false);
#ifdef QAD_SPI_USE_GPIO_CS
    gpio_cs_disable();
#endif /* QAD_SPI_USE_GPIO_CS */

    return rd_data;
}



static uint32_t LCD_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos, uint16_t Ypos)
{
    uint8_t  r, g, b;
    uint32_t ret_v, read_value;
    DEBUG_PRINTF("GC9B71_ReadPixel[%d,%d]\n", Xpos, Ypos);

    LCD_ALIGN2(Xpos);
    LCD_ALIGN2(Ypos);

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
    ?11?= 12bit/pixel ?01?= 16bit/pixel ?10?= 18bit/pixel ?11?= 16M truncated

    */
    switch (color_mode)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        /* Color mode 16bits/pixel */
        parameter[0] = 0xD5;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB565;
        break;

    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        parameter[0] = 0xF7;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB888;
        break;

    default:
        return; //unsupport
        break;
    }

    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);
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
    LOG_I("GC9B71_SetBrightness\r\n");
}

static void LCD_Rotate(LCDC_HandleTypeDef *hlcdc, LCD_DrvRotateTypeDef degree)
{
    uint8_t parameter;
#if defined(LCDC_SUPPORT_V_MIRROR)&&!defined(LCDC_SUPPORT_H_MIRROR)
    if (LCD_ROTATE_180_DEGREE == degree)
    {
        parameter = 0 << 7; //MY
        parameter = 0 << 6; //MX
        LCD_WriteReg(hlcdc, REG_MACTL, &parameter, 1);
    }
    else
    {
        parameter = 0 << 7; //MY
        parameter = 1 << 6; //MX
        LCD_WriteReg(hlcdc, REG_MACTL, &parameter, 1);
    }
#endif /* LCDC_SUPPORT_H_MIRROR */
}




static const LCD_DrvOpsDef GC9B71_drv =
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
    LCD_Rotate,
};


LCD_DRIVER_EXPORT2(gc9b71, THE_LCD_ID, &lcdc_int_cfg,
                   &GC9B71_drv, 2);








/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
