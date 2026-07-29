/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
  * @brief GC9C01 chip IDs
  */
#define THE_LCD_ID                  0x009C01

/**
  * @brief  GC9C01 Registers
  */
#define REG_SW_RESET           0x01
#define REG_LCD_ID             0x04
#define REG_DSI_ERR            0x05
#define REG_POWER_MODE         0x0A
#define REG_SLEEP_IN           0x10
#define REG_SLEEP_OUT          0x11
#define REG_PARTIAL_DISPLAY    0x12
#define REG_DISPLAY_INVERSION  0x21
#define REG_DISPLAY_OFF        0x28
#define REG_DISPLAY_ON         0x29
#define REG_WRITE_RAM          0x2C
#define REG_READ_RAM           0x2E
#define REG_CASET              0x2A
#define REG_RASET              0x2B

#define REG_TEARING_EFFECT     0x35

#define REG_IDLE_MODE_OFF      0x38
#define REG_IDLE_MODE_ON       0x39
#define REG_COLOR_MODE         0x3A

#define REG_WBRIGHT            0x51 /* Write brightness*/

#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

#define QAD_SPI_ITF LCDC_INTF_SPI_DCX_4DATA

static const LCDC_InitTypeDef lcdc_int_cfg_spi =
{
    .lcd_itf = QAD_SPI_ITF, //LCDC_INTF_SPI_NODCX_1DATA,
    .freq = 24000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 0, //0: QAD-SPI/SPI3   1:SPI4
            .syn_mode = HAL_LCDC_SYNC_DISABLE, //HAL_LCDC_SYNC_VER,
            .vsyn_polarity = 0,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 1000,
            .hsyn_num = 0,
        },
    },

};

static LCDC_InitTypeDef lcdc_int_cfg;
static void     LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable);

//#define GC9C01_LCD_TEST
#ifdef GC9C01_LCD_TEST
static rt_err_t lcd_init(int argc, char **argv)
{
    GC9C01_Init();
    return 0;
}

MSH_CMD_EXPORT(lcd_init, lcd_init);

static rt_err_t lcd_rreg(int argc, char **argv)
{
    uint32_t data;

    uint16_t i, reg, len;

    reg = strtoul(argv[1], 0, 16);
    len = strtoul(argv[2], 0, 16);

    data = LCD_ReadData(reg, len);

    DEBUG_PRINTF("\nGC9C01_Read reg[%x] %d(byte), result=%08x\n", reg, len, data);

    return 0;
}
MSH_CMD_EXPORT(lcd_rreg, lcd_rreg);

static rt_err_t lcd_wreg(int argc, char **argv)
{
    uint8_t   parameter[4];

    uint32_t data;
    uint16_t reg, i;

    reg = strtoul(argv[1], 0, 16);

    for (i = 2; i < argc; i++)
    {
        parameter[i - 2] = strtoul(argv[i], 0, 16);
    }

    LCD_WriteReg(hlcdc, reg, parameter, argc - 2);
    DEBUG_PRINTF("\nGC9C01_Write reg[%x] %d(byte) done.\n", reg, argc - 2);

    return 0;

}
MSH_CMD_EXPORT(lcd_wreg, lcd_wreg);
uint8_t gc9c01_dsip_mode_value = 0;

static rt_err_t lcd_cfg(int argc, char **argv)
{

    if (strcmp(argv[1], "nodcx1d") == 0)
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_NODCX_1DATA;
    else if (strcmp(argv[1], "nodcx2d") == 0)
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_NODCX_2DATA;
    else if (strcmp(argv[1], "nodcx4d") == 0)
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_NODCX_4DATA;

    else if (strcmp(argv[1], "dcx1d") == 0)
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_DCX_1DATA;
    else if (strcmp(argv[1], "dcx2d") == 0)
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_DCX_2DATA;
    else if (strcmp(argv[1], "dcx4d") == 0)
        lcdc_int_cfg_spi.lcd_itf = LCDC_INTF_SPI_DCX_4DATA;

    if (strcmp(argv[2], "rgb565") == 0)
        lcdc_int_cfg_spi.color_mode = LCDC_PIXEL_FORMAT_RGB565;
    else if (strcmp(argv[2], "rgb888") == 0)
        lcdc_int_cfg_spi.color_mode = LCDC_PIXEL_FORMAT_RGB888;

    lcdc_int_cfg_spi.freq = 1000000 * strtoul(argv[3], 0, 10);

    /*
        bit 0:  DUAL SPI MODE enable
        bit[5:4] DAUL SPI MODE Selection
                  00: 1P1T for 1 wire
                  10: 1P1T for 2 wire
                  11: 2P3T for 2 wire
                  01: reserve

    */
    if (strcmp(argv[4], "default") == 0)
        gc9c01_dsip_mode_value = 0x80;
    else if (strcmp(argv[4], "1p1t") == 0)
        gc9c01_dsip_mode_value = 0x81 | (0 << 4);
    else if (strcmp(argv[4], "1p1t_2w") == 0)
        gc9c01_dsip_mode_value = 0x81 | (2 << 4);
    else if (strcmp(argv[4], "2p3t_2w") == 0)
        gc9c01_dsip_mode_value = 0x81 | (3 << 4);

    lcdc_int_cfg_spi.cfg.spi.dummy_clock = strtoul(argv[5], 0, 10);

    DEBUG_PRINTF("\nlcd_cfg itf=%d, colormode=%d, freq=%d, disp_m=%x\n", lcdc_int_cfg_spi.lcd_itf,
                 lcdc_int_cfg_spi.color_mode,
                 lcdc_int_cfg_spi.freq,
                 gc9c01_dsip_mode_value
                );

    return 0;

}

MSH_CMD_EXPORT(lcd_cfg, lcd_cfg);
#endif /* DSI_TEST */

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
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t parameter[32];

    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_spi, sizeof(lcdc_int_cfg));

    /* Initialize GC9C01 low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0);//Reset LCD
    HAL_Delay_us(20);
    BSP_LCD_Reset(1);

    /* Wait for 200ms */
    LCD_DRIVER_DELAY_MS(200);

    //Initail
    LCD_WriteReg(hlcdc, 0xfe, parameter, 0); // internal reg enable
    LCD_WriteReg(hlcdc, 0xef, parameter, 0); // internal reg enable

    parameter[0] = 0x11; //reg_en for 70\74
    LCD_WriteReg(hlcdc, 0x80, parameter, 1);

    parameter[0] = 0x70; //reg_en for 7C\7D\7E
    LCD_WriteReg(hlcdc, 0x81, parameter, 1);

    parameter[0] = 0x09; //reg_en for 90\93
    LCD_WriteReg(hlcdc, 0x82, parameter, 1);

    parameter[0] = 0x03; //reg_en for 98\99
    LCD_WriteReg(hlcdc, 0x83, parameter, 1);

    parameter[0] = 0x20; //reg_en for B5
    LCD_WriteReg(hlcdc, 0x84, parameter, 1);

    parameter[0] = 0x42; //reg_en for B9\BE
    LCD_WriteReg(hlcdc, 0x85, parameter, 1);

    parameter[0] = 0xfc; //reg_en for C2~7
    LCD_WriteReg(hlcdc, 0x86, parameter, 1);

    parameter[0] = 0x09; //reg_en for C8\CB
    LCD_WriteReg(hlcdc, 0x87, parameter, 1);

    parameter[0] = 0x10; //reg_en for EC
    LCD_WriteReg(hlcdc, 0x89, parameter, 1);

    parameter[0] = 0x4f; //reg_en for F0~3\F6
    LCD_WriteReg(hlcdc, 0x8A, parameter, 1);

    parameter[0] = 0x59; //reg_en for 60\63\64\66
    LCD_WriteReg(hlcdc, 0x8C, parameter, 1);

    parameter[0] = 0x51; //reg_en for 68\6C\6E
    LCD_WriteReg(hlcdc, 0x8D, parameter, 1);

    parameter[0] = 0xae; //reg_en for A1~3\A5\A7
    LCD_WriteReg(hlcdc, 0x8E, parameter, 1);

    parameter[0] = 0xf3; //reg_en for AC~F\A8\A9
    LCD_WriteReg(hlcdc, 0x8F, parameter, 1);

    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x36, parameter, 1);

    if (LCDC_PIXEL_FORMAT_RGB888 == lcdc_int_cfg.color_mode)
        parameter[0] = 0x07; //24bit rgb
    else if (LCDC_PIXEL_FORMAT_RGB565 == lcdc_int_cfg.color_mode)
        parameter[0] = 0x05; //16bit rgb
    else if (LCDC_PIXEL_FORMAT_RGB332 == lcdc_int_cfg.color_mode)
    {
        RT_ASSERT(QAD_SPI_ITF == lcdc_int_cfg.lcd_itf);
        parameter[0] = 0x02; //8bit rgb QSPI only
    }
    else
        RT_ASSERT(0); //fix me

    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    parameter[0] = 0x77;
    LCD_WriteReg(hlcdc, 0xEC, parameter, 1); //2COL

    parameter[0] = 0x01;
    parameter[1] = 0x80;
    parameter[2] = 0x00;
    parameter[3] = 0x00;
    parameter[4] = 0x00;
    parameter[5] = 0x00;
    LCD_WriteReg(hlcdc, 0x74, parameter, 6); //rtn 60Hz

    parameter[0] = 0x3E;
    LCD_WriteReg(hlcdc, 0x98, parameter, 1); //bvdd 3x

    parameter[0] = 0x3E;
    LCD_WriteReg(hlcdc, 0x99, parameter, 1); //bvee -2x

    parameter[0] = 0x2A;
    LCD_WriteReg(hlcdc, 0xC3, parameter, 1); //VBP

    parameter[0] = 0x18;
    LCD_WriteReg(hlcdc, 0xC4, parameter, 1); //VBN

    //*********************????*********************//

    parameter[0] = 0x01;
    parameter[1] = 0x04; //SRAM RD OPTION
    LCD_WriteReg(hlcdc, 0xA1, parameter, 2);

    parameter[0] = 0x01;
    parameter[1] = 0x04; //SRAM RD OPTION
    LCD_WriteReg(hlcdc, 0xA2, parameter, 2);

    parameter[0] = 0x1C; //IREF=9.8uA
    LCD_WriteReg(hlcdc, 0xA9, parameter, 1);

    parameter[0] = 0x11; //VDDMA=1.553V
    parameter[1] = 0x09; //VDDML=1.24V
    LCD_WriteReg(hlcdc, 0xA5, parameter, 2);

    parameter[0] = 0x8A; //RTERM=101O
    LCD_WriteReg(hlcdc, 0xB9, parameter, 1);

    parameter[0] = 0x5E; //VBG_BUF=1.003V, DVDD=1.543V
    LCD_WriteReg(hlcdc, 0xA8, parameter, 1);

    parameter[0] = 0x40; //BIAS=10.2uA
    LCD_WriteReg(hlcdc, 0xA7, parameter, 1);

    parameter[0] = 0x73; //VDDSOU=1.715V ,VDDGM=2.002V
    LCD_WriteReg(hlcdc, 0xAF, parameter, 1);

    parameter[0] = 0x44; //VREE=2.475V,VRDD=2.335V
    LCD_WriteReg(hlcdc, 0xAE, parameter, 1);

    parameter[0] = 0x38; //VRGL=1.635V ,VDDSF=2.018V
    LCD_WriteReg(hlcdc, 0xAD, parameter, 1);

    parameter[0] = 0x5D; //OSC=53.7MHz
    LCD_WriteReg(hlcdc, 0xA3, parameter, 1);

    parameter[0] = 0x02; //VREG_VREF=2.805V
    LCD_WriteReg(hlcdc, 0xC2, parameter, 1);

    parameter[0] = 0x11; //VREG1A=5.99V
    LCD_WriteReg(hlcdc, 0xC5, parameter, 1);

    parameter[0] = 0x0E; //VREG1B=1.505V
    LCD_WriteReg(hlcdc, 0xC6, parameter, 1);

    parameter[0] = 0x13; //VREG2A=-2.995V
    LCD_WriteReg(hlcdc, 0xC7, parameter, 1);

    parameter[0] = 0x0D; //VREG2B=1.497V
    LCD_WriteReg(hlcdc, 0xC8, parameter, 1);

    parameter[0] = 0x02; //6.09V
    LCD_WriteReg(hlcdc, 0xCB, parameter, 1); //bvdd ref_ad

    parameter[0] = 0xB6;
    parameter[1] = 0x26; //13.12V
    LCD_WriteReg(hlcdc, 0x7C, parameter, 2);

    parameter[0] = 0x24; //VGLO=-8.35V
    LCD_WriteReg(hlcdc, 0xAC, parameter, 1);

    parameter[0] = 0x80; //EPF=2
    LCD_WriteReg(hlcdc, 0xF6, parameter, 1);

    //*********************????*************************//
    //gip start

    parameter[0] = 0x09; //VFP
    parameter[1] = 0x09; //VBP
    LCD_WriteReg(hlcdc, 0xB5, parameter, 2);

    parameter[0] = 0x38;
    parameter[1] = 0x0B;
    parameter[2] = 0x5B;
    parameter[3] = 0x56;
    LCD_WriteReg(hlcdc, 0x60, parameter, 4); //STV1&2

    parameter[0] = 0x3A;
    parameter[1] = 0xE0; //DE
    parameter[2] = 0x5B; //MAX=0x61
    parameter[3] = 0x56; //MAX=0x61
    LCD_WriteReg(hlcdc, 0x63, parameter, 4); //STV3&4

    parameter[0] = 0x38;
    parameter[1] = 0x0D;
    parameter[2] = 0x72;
    parameter[3] = 0xDD;
    parameter[4] = 0x5B;
    parameter[5] = 0x56;
    LCD_WriteReg(hlcdc, 0x64, parameter, 6); //CLK_group1

    parameter[0] = 0x38;
    parameter[1] = 0x11;
    parameter[2] = 0x72;
    parameter[3] = 0xE1;
    parameter[4] = 0x5B;
    parameter[5] = 0x56;
    LCD_WriteReg(hlcdc, 0x66, parameter, 6); //CLK_group1

    parameter[0] = 0x3B; //FLC12 FREQ
    parameter[1] = 0x08;
    parameter[2] = 0x08;
    parameter[3] = 0x00;
    parameter[4] = 0x08;
    parameter[5] = 0x29;
    parameter[6] = 0x5B;
    LCD_WriteReg(hlcdc, 0x68, parameter, 7); //FLC&FLV 1~2

    parameter[0] = 0x00; //gout1_swap_fw[4:0]
    parameter[1] = 0x00; //gout2_swap_fw[4:0]
    parameter[2] = 0x00; //gout3_swap_fw[4:0]
    parameter[3] = 0x07; //gout4_swap_fw[4:0]
    parameter[4] = 0x01; //gout5_swap_fw[4:0]
    parameter[5] = 0x13; //gout6_swap_fw[4:0]
    parameter[6] = 0x11; //gout7_swap_fw[4:0]
    parameter[7] = 0x0B; //gout8_swap_fw[4:0]
    parameter[8] = 0x09; //gout9_swap_fw[4:0]
    parameter[9] = 0x16; //gout10_swap_fw[4:0]
    parameter[10] = 0x15; //gout11_swap_fw[4:0]
    parameter[11] = 0x1D; //gout12_swap_fw[4:0]
    parameter[12] = 0x1E; //gout13_swap_fw[4:0]
    parameter[13] = 0x00; //gout14_swap_fw[4:0]
    parameter[14] = 0x00; //gout15_swap_fw[4:0]
    parameter[15] = 0x00; //gout16_swap_fw[4:0]
    parameter[16] = 0x00; //gout17_swap_fw[4:0]
    parameter[17] = 0x00; //gout18_swap_fw[4:0]
    parameter[18] = 0x00; //gout19_swap_fw[4:0]
    parameter[19] = 0x1E; //gout20_swap_fw[4:0]
    parameter[20] = 0x1D; //gout21_swap_fw[4:0]
    parameter[21] = 0x15; //gout22_swap_fw[4:0]
    parameter[22] = 0x16; //gout23_swap_fw[4:0]
    parameter[23] = 0x0A; //gout24_swap_fw[4:0]
    parameter[24] = 0x0C; //gout25_swap_fw[4:0]
    parameter[25] = 0x12; //gout26_swap_fw[4:0]
    parameter[26] = 0x14; //gout27_swap_fw[4:0]
    parameter[27] = 0x02; //gout28_swap_fw[4:0]
    parameter[28] = 0x08; //gout29_swap_fw[4:0]
    parameter[29] = 0x00; //gout30_swap_fw[4:0]
    parameter[30] = 0x00; //gout31_swap_fw[4:0]
    parameter[31] = 0x00; //gout32_swap_fw[4:0]
    LCD_WriteReg(hlcdc, 0x6E, parameter, 32); //gout_Mapping

    //*********************?????**********************//

    parameter[0] = 0x11; //SOU_BIAS_FIX=1
    LCD_WriteReg(hlcdc, 0xBE, parameter, 1);

    parameter[0] = 0xCC;
    parameter[1] = 0x0C;
    parameter[2] = 0xCC;
    parameter[3] = 0x84;
    parameter[4] = 0xCC;
    parameter[5] = 0x04;
    parameter[6] = 0x50;
    LCD_WriteReg(hlcdc, 0x6C, parameter, 7); //precharge GATE

    parameter[0] = 0x72;
    LCD_WriteReg(hlcdc, 0x7D, parameter, 1);

    parameter[0] = 0x38; // VGL_BT=1 5X  (BT=0:6X)  RT=0
    LCD_WriteReg(hlcdc, 0x7E, parameter, 1);

    parameter[0] = 0x02; //avdd_clk
    parameter[1] = 0x03; //avee_clk
    parameter[2] = 0x09; //vdh_clk
    parameter[3] = 0x05; //vgh_clk
    parameter[4] = 0x0C; //vgl_clk
    parameter[5] = 0x06; //vcl_clk
    parameter[6] = 0x09; //vdh_clk_porch  0E
    parameter[7] = 0x05; //vgh_clk_porch  0E
    parameter[8] = 0x0C; //vgl_clk_porch  0E
    parameter[9] = 0x06; //vcl_clk_porch  0E
    LCD_WriteReg(hlcdc, 0x70, parameter, 10);

    parameter[0] = 0x06;
    parameter[0] = 0x06;
    parameter[0] = 0x05; //bvdd_clk1_ad1
    parameter[0] = 0x06; //bvdd_clk1_ad1
    LCD_WriteReg(hlcdc, 0x90, parameter, 1);

    parameter[0] = 0x45; //bvdd_shut_ad1
    parameter[1] = 0xFF;
    parameter[2] = 0x00;
    LCD_WriteReg(hlcdc, 0x93, parameter, 3);

    //gamma start
    parameter[0] = 0x45;
    parameter[1] = 0x09;
    parameter[2] = 0x08;
    parameter[3] = 0x08;
    parameter[4] = 0x26;
    parameter[5] = 0x2A;
    LCD_WriteReg(hlcdc, 0xF0, parameter, 6);

    parameter[0] = 0x43;
    parameter[1] = 0x70;
    parameter[2] = 0x72;
    parameter[3] = 0x36;
    parameter[4] = 0x37;
    parameter[5] = 0x6F;
    LCD_WriteReg(hlcdc, 0xF1, parameter, 6);

    parameter[0] = 0x45;
    parameter[1] = 0x09;
    parameter[2] = 0x08;
    parameter[3] = 0x08;
    parameter[4] = 0x26;
    parameter[5] = 0x2A;
    LCD_WriteReg(hlcdc, 0xF2, parameter, 6);

    parameter[0] = 0x43;
    parameter[1] = 0x70;
    parameter[2] = 0x72;
    parameter[3] = 0x36;
    parameter[4] = 0x37;
    parameter[5] = 0x6F;
    LCD_WriteReg(hlcdc, 0xF3, parameter, 6);

    LCD_WriteReg(hlcdc, REG_SLEEP_OUT, (uint8_t *)NULL, 0);

    /* Wait for 100ms */
    LCD_DRIVER_DELAY_MS(100);

    /* Display ON command */
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);

    LCD_DRIVER_DELAY_MS(150); //Wait TE signal ready

    //LCD_WriteReg(hlcdc,0x23, (uint8_t *)NULL, 0);

    /* Tearing Effect Line On: Option (00h:VSYNC Only, 01h:VSYNC & HSYNC ) */
    //parameter[0] = 0x02;
    // LCD_WriteReg(hlcdc,REG_TEARING_EFFECT, parameter, 1);

}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
    uint32_t data;
    /*
        data = LCD_ReadData(hlcdc,REG_CASET, 4);
        DEBUG_PRINTF("\REG_CASET 0x%x \n", data);

        data = LCD_ReadData(hlcdc,REG_RASET, 4);
        DEBUG_PRINTF("\REG_RASET 0x%x \n", data);
    */
    data = LCD_ReadData(hlcdc, REG_LCD_ID, 3);
    DEBUG_PRINTF("\nGC9C01_ReadID 0x%x \n", data);

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
        DEBUG_PRINTF("GC9C01_SetX[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
    else if (LCD_Reg == REG_RASET)
    {
        DEBUG_PRINTF("GC9C01_SetY[%d,%d]\n", ((Parameters[0] << 8) | Parameters[1]),
                     ((Parameters[2] << 8) | Parameters[3]));
    }
#endif

    if (0)
    {
    }
    else if (QAD_SPI_ITF == lcdc_int_cfg.lcd_itf)
    {
        uint32_t cmd;

        cmd = (0x02 << 24) | (LCD_Reg << 8);

        if (0 != NbParameters)
        {
            /* Send command's parameters if any */
            HAL_LCDC_WriteU32Reg(hlcdc, cmd, Parameters, NbParameters);
        }
        else
        {
            uint32_t v = 0;
            HAL_LCDC_WriteU32Reg(hlcdc, cmd, (uint8_t *)&v, 1);
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
        HAL_LCDC_ReadU32Reg(hlcdc, ((0x03 << 24) | (RegValue << 8)), (uint8_t *)&rd_data, ReadSize);
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

    DEBUG_PRINTF("GC9C01 NOT support read pixel!");

    return 0;

    DEBUG_PRINTF("GC9C01_ReadPixel[%d,%d]\n", Xpos, Ypos);

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
        parameter[0] = 0x75;
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

    uint32_t data = LCD_ReadData(hlcdc, 0xc, 1);
    DEBUG_PRINTF("\nGC9C01_color_format 0x%x \n", data);

    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

#define GC9C01_BRIGHTNESS_MAX 0xFF

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = (uint8_t)((int)GC9C01_BRIGHTNESS_MAX * br / 100);
    LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);
}

static const LCD_DrvOpsDef GC9C01_drv =
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
    NULL

};

LCD_DRIVER_EXPORT2(gc9c01, THE_LCD_ID, &lcdc_int_cfg,
                   &GC9C01_drv, 1);

