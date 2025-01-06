/**
  ******************************************************************************
  * @file   nv3041a.c
  * @author Sifli software development team
  * @brief   This file includes the LCD driver for NV3041A LCD.
  * @attention
  ******************************************************************************
*/


#include <rtthread.h>
#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"

#define DBG_TAG               "nv3041a"
#define DBG_LVL               DBG_INFO
#include <rtdbg.h>


#define ROW_OFFSET  (0x00)
#define COL_OFFSET  (0x00)


/**
  * @brief NV3041A chip IDs
  */
#define THE_LCD_ID                  0x60834200

/**
  * @brief  NV3041A Size
  */
#define  THE_LCD_PIXEL_WIDTH    (480)
#define  THE_LCD_PIXEL_HEIGHT   (272)






/**
  * @brief  NV3041A Registers
  */
//#define REG_SW_RESET           0x01
#define REG_LCD_ID             0x04
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















#define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...)   LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif

/*nv3041a start colume & row must can be divided by 2, and roi width&height too.*/
#define LCD_ALIGN2(x) ((x) = (x) & (~1))
#define LCD_ALIGN1(x) ((x) = (0 == ((x) & 1)) ? (x - 1) : x)


#define QAD_SPI_ITF LCDC_INTF_SPI_DCX_4DATA


static const LCDC_InitTypeDef lcdc_int_cfg_qadspi =
{
    .lcd_itf = QAD_SPI_ITF,
    .freq = 36000000,        // 240/7= 34.28Mhz
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,//LCDC_PIXEL_FORMAT_RGB565,

    .cfg = {
        .spi = {
            .dummy_clock = 1,
#ifdef LCD_NV3041A_VSYNC_ENABLE
            .syn_mode = HAL_LCDC_SYNC_VER,
#else
            .syn_mode = HAL_LCDC_SYNC_DISABLE,
#endif /* LCD_NV3041A_VSYNC_ENABLE */
            .vsyn_polarity = 1,
            //default_vbp=2, frame rate=82, delay=115us,
            //TODO: use us to define delay instead of cycle, delay_cycle=115*48
            .vsyn_delay_us = 0,
            .hsyn_num = 0,
            .readback_from_Dx = 0,
        },
    },

};

static LCDC_InitTypeDef lcdc_int_cfg;

static void     LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg, uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue, uint8_t ReadSize);
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable);









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
            HAL_LCDC_SetFreq(hlcdc, 5000000); //read mode min cycle 300ns
        }
        else
        {
            HAL_LCDC_SetFreq(hlcdc, lcdc_int_cfg.freq); //Restore normal frequency
        }
    }

}


static void NV3041A_Drv_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[32];

    /* Initialize NV3041A low level bus layer ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    //rt_pin_mode(LCD_BACKLIGHT_POWER_PIN, PIN_MODE_OUTPUT);
    //rt_pin_write(LCD_BACKLIGHT_POWER_PIN, 0);
    BSP_LCD_Reset(1);
    rt_thread_mdelay(1);
    BSP_LCD_Reset(0);//Reset LCD
    rt_thread_mdelay(10);
    BSP_LCD_Reset(1);

    /* Wait for 200ms */
    rt_thread_mdelay(120);
    //Initail
    //---------Start Initial Code ------//
//    SPI_WriteComm(0xff);
//    SPI_WriteData(0xa5);
    parameter[0] = 0xa5;
    LCD_WriteReg(hlcdc, 0xff, parameter, 1);

    /* bist mode */
#if 0
    parameter[0] = 0x11;
    LCD_WriteReg(hlcdc, 0xf8, parameter, 1);

    parameter[0] = 0x11;
    LCD_WriteReg(hlcdc, 0xf9, parameter, 1);
#endif
    /* bist mode */


//    SPI_WriteComm(0xE7);//TE_output_en
//    SPI_WriteData(0x10);
    parameter[0] = 0x10;
    LCD_WriteReg(hlcdc, 0xE7, parameter, 1);


//    SPI_WriteComm(0x35);//TE_ interface_en
//    SPI_WriteData(0x01);
    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0x35, parameter, 1);

//    SPI_WriteComm(0x3A);
//    SPI_WriteData(0x01);//00---666//01--565
    parameter[0] = 0x01;
    LCD_WriteReg(hlcdc, 0x3A, parameter, 1);

//    SPI_WriteComm(0x40);
//    SPI_WriteData(0x01);
    parameter[0] = 0x01;
    LCD_WriteReg(hlcdc, 0x40, parameter, 1);

//    SPI_WriteComm(0x41);
//    SPI_WriteData(0x01);//01--8bit//03--16bit
    parameter[0] = 0x01;
    LCD_WriteReg(hlcdc, 0x41, parameter, 1);

//    SPI_WriteComm(0x55);
//    SPI_WriteData(0x01);
    parameter[0] = 0x01;
    LCD_WriteReg(hlcdc, 0x55, parameter, 1);

//    SPI_WriteComm(0x44);
//    SPI_WriteData(0x15);
    parameter[0] = 0x15; //TE 62Hz
    LCD_WriteReg(hlcdc, 0x44, parameter, 1);

//    SPI_WriteComm(0x45);
//    SPI_WriteData(0x15);
    parameter[0] = 0x15; //TE 62Hz
    LCD_WriteReg(hlcdc, 0x45, parameter, 1);

//    SPI_WriteComm(0x7d);
//    SPI_WriteData(0x03);
    parameter[0] = 0x03;
    LCD_WriteReg(hlcdc, 0x7d, parameter, 1);

//    SPI_WriteComm(0xc1);
//    SPI_WriteData(0xab);
    parameter[0] = 0xab;
    LCD_WriteReg(hlcdc, 0xc1, parameter, 1);

//    SPI_WriteComm(0xc2);
//    SPI_WriteData(0x17);
    parameter[0] = 0x17;
    LCD_WriteReg(hlcdc, 0xc2, parameter, 1);

//    SPI_WriteComm(0xc3);
//    SPI_WriteData(0x10);
    parameter[0] = 0x10;
    LCD_WriteReg(hlcdc, 0xc3, parameter, 1);

//    SPI_WriteComm(0xc6);
//    SPI_WriteData(0x3a);
    parameter[0] = 0x3a;
    LCD_WriteReg(hlcdc, 0xc6, parameter, 1);

//    SPI_WriteComm(0xc7);
//    SPI_WriteData(0x25);
    parameter[0] = 0x25;
    LCD_WriteReg(hlcdc, 0xc7, parameter, 1);

//    SPI_WriteComm(0xc8);
//    SPI_WriteData(0x11);
    parameter[0] = 0x11;
    LCD_WriteReg(hlcdc, 0xc8, parameter, 1);

//    SPI_WriteComm(0x7a);
//    SPI_WriteData(0x49);
    parameter[0] = 0x49;
    LCD_WriteReg(hlcdc, 0x7a, parameter, 1);

//    SPI_WriteComm(0x6f);
//    SPI_WriteData(0x2f);
    parameter[0] = 0x2f;
    LCD_WriteReg(hlcdc, 0x6f, parameter, 1);

//    SPI_WriteComm(0x78);
//    SPI_WriteData(0x4b);
    parameter[0] = 0x4b;
    LCD_WriteReg(hlcdc, 0x78, parameter, 1);

//    SPI_WriteComm(0x73);
//    SPI_WriteData(0x08);
    parameter[0] = 0x08;
    LCD_WriteReg(hlcdc, 0x73, parameter, 1);

//    SPI_WriteComm(0x74);
//    SPI_WriteData(0x12);
    parameter[0] = 0x11;//0x11 59.5hz //0x12; 62hz
    LCD_WriteReg(hlcdc, 0x74, parameter, 1);

//    SPI_WriteComm(0xc9);
//    SPI_WriteData(0x00);
    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xc9, parameter, 1);

//    SPI_WriteComm(0x67);
//    SPI_WriteData(0x11);
    //gate_ed
    parameter[0] = 0x11;
    LCD_WriteReg(hlcdc, 0x67, parameter, 1);

//    SPI_WriteComm(0x51);
//    SPI_WriteData(0x20);
    parameter[0] = 0x20;
    LCD_WriteReg(hlcdc, 0x51, parameter, 1);

//    SPI_WriteComm(0x52);
//    SPI_WriteData(0x7c);
    parameter[0] = 0x7c;
    LCD_WriteReg(hlcdc, 0x52, parameter, 1);

//    SPI_WriteComm(0x53);
//    SPI_WriteData(0x1c);
    parameter[0] = 0x1c;
    LCD_WriteReg(hlcdc, 0x53, parameter, 1);

//    SPI_WriteComm(0x54);
//    SPI_WriteData(0x77);
    ////sorce old
    parameter[0] = 0x77;
    LCD_WriteReg(hlcdc, 0x54, parameter, 1);

//    SPI_WriteComm(0x46);
//    SPI_WriteData(0x0a);
    parameter[0] = 0x0a;
    LCD_WriteReg(hlcdc, 0x46, parameter, 1);

//    SPI_WriteComm(0x47);
//    SPI_WriteData(0x2a);
    parameter[0] = 0x2a;
    LCD_WriteReg(hlcdc, 0x47, parameter, 1);

//    SPI_WriteComm(0x48);
//    SPI_WriteData(0x0a);
    parameter[0] = 0x0a;
    LCD_WriteReg(hlcdc, 0x48, parameter, 1);

//    SPI_WriteComm(0x49);
//    SPI_WriteData(0x1a);
    parameter[0] = 0x1a;
    LCD_WriteReg(hlcdc, 0x49, parameter, 1);

//    SPI_WriteComm(0x56);
//    SPI_WriteData(0x43);
    parameter[0] = 0x43;
    LCD_WriteReg(hlcdc, 0x56, parameter, 1);

//    SPI_WriteComm(0x57);
//    SPI_WriteData(0x42);
    parameter[0] = 0x42;
    LCD_WriteReg(hlcdc, 0x57, parameter, 1);

//    SPI_WriteComm(0x58);
//    SPI_WriteData(0x3c);
    parameter[0] = 0x3c;
    LCD_WriteReg(hlcdc, 0x58, parameter, 1);

//    SPI_WriteComm(0x59);
//    SPI_WriteData(0x64);
    parameter[0] = 0x64;
    LCD_WriteReg(hlcdc, 0x59, parameter, 1);

//    SPI_WriteComm(0x5a);
//    SPI_WriteData(0x41);
    parameter[0] = 0x41;
    LCD_WriteReg(hlcdc, 0x5a, parameter, 1);

//    SPI_WriteComm(0x5b);
//    SPI_WriteData(0x3c);
    parameter[0] = 0x3c;
    LCD_WriteReg(hlcdc, 0x5b, parameter, 1);;

//    SPI_WriteComm(0x5c);
//    SPI_WriteData(0x02);
    parameter[0] = 0x02;
    LCD_WriteReg(hlcdc, 0x5c, parameter, 1);;

//    SPI_WriteComm(0x5d);
//    SPI_WriteData(0x3c);
    parameter[0] = 0x3c;
    LCD_WriteReg(hlcdc, 0x5d, parameter, 1);

//    SPI_WriteComm(0x5e);
//    SPI_WriteData(0x1f);
    parameter[0] = 0x1f;
    LCD_WriteReg(hlcdc, 0x5e, parameter, 1);

//    SPI_WriteComm(0x60);
//    SPI_WriteData(0x80);
    parameter[0] = 0x80;
    LCD_WriteReg(hlcdc, 0x60, parameter, 1);

//    SPI_WriteComm(0x61);
//    SPI_WriteData(0x3f);
    parameter[0] = 0x3f;
    LCD_WriteReg(hlcdc, 0x61, parameter, 1);

//    SPI_WriteComm(0x62);
//    SPI_WriteData(0x21);
    parameter[0] = 0x21;
    LCD_WriteReg(hlcdc, 0x62, parameter, 1);

//    SPI_WriteComm(0x63);
//    SPI_WriteData(0x07);
    parameter[0] = 0x07;
    LCD_WriteReg(hlcdc, 0x63, parameter, 1);

//    SPI_WriteComm(0x64);
//    SPI_WriteData(0xe0);
    parameter[0] = 0xe0;
    LCD_WriteReg(hlcdc, 0x64, parameter, 1);

//    SPI_WriteComm(0x65);
//    SPI_WriteData(0x01);
    parameter[0] = 0x01;
    LCD_WriteReg(hlcdc, 0x65, parameter, 1);

//    SPI_WriteComm(0xca);
//    SPI_WriteData(0x20);
    parameter[0] = 0x20;
    LCD_WriteReg(hlcdc, 0xca, parameter, 1);

//    SPI_WriteComm(0xcb);
//    SPI_WriteData(0x52);
    parameter[0] = 0x52;
    LCD_WriteReg(hlcdc, 0xcb, parameter, 1);

//    SPI_WriteComm(0xcc);
//    SPI_WriteData(0x10);
    parameter[0] = 0x10;
    LCD_WriteReg(hlcdc, 0xcc, parameter, 1);

//    SPI_WriteComm(0xcD);
//    SPI_WriteData(0x42);
    parameter[0] = 0x42;
    LCD_WriteReg(hlcdc, 0xcD, parameter, 1);

//    SPI_WriteComm(0xD0);
//    SPI_WriteData(0x20);
    parameter[0] = 0x20;
    LCD_WriteReg(hlcdc, 0xD0, parameter, 1);

//    SPI_WriteComm(0xD1);
//    SPI_WriteData(0x52);
    parameter[0] = 0x52;
    LCD_WriteReg(hlcdc, 0xD1, parameter, 1);

//    SPI_WriteComm(0xD2);
//    SPI_WriteData(0x10);
    parameter[0] = 0x10;
    LCD_WriteReg(hlcdc, 0xD2, parameter, 1);

//    SPI_WriteComm(0xD3);
//    SPI_WriteData(0x42);
    parameter[0] = 0x42;
    LCD_WriteReg(hlcdc, 0xD3, parameter, 1);

//    SPI_WriteComm(0xD4);
//    SPI_WriteData(0x0a);
    parameter[0] = 0x0a;
    LCD_WriteReg(hlcdc, 0xD4, parameter, 1);

//    SPI_WriteComm(0xD5);
//    SPI_WriteData(0x32);
    parameter[0] = 0x32;
    LCD_WriteReg(hlcdc, 0xD5, parameter, 1);

//    SPI_WriteComm(0xe5);
//    SPI_WriteData(0x06);
    parameter[0] = 0x06;
    LCD_WriteReg(hlcdc, 0xe5, parameter, 1);

//    SPI_WriteComm(0xe6);
//    SPI_WriteData(0x00);
    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xe6, parameter, 1);

//    SPI_WriteComm(0x6e);
//    SPI_WriteData(0x14);
    //gammma 01
    parameter[0] = 0x14;
    LCD_WriteReg(hlcdc, 0x6e, parameter, 1);

//    SPI_WriteComm(0x80);
//    SPI_WriteData(0x04);
    parameter[0] = 0x04;
    LCD_WriteReg(hlcdc, 0x80, parameter, 1);

//    SPI_WriteComm(0xA0);
//    SPI_WriteData(0x00);
    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xA0, parameter, 1);

//    SPI_WriteComm(0x81);  confirm again
//    SPI_WriteData(0x07);
    parameter[0] = 0x07;
    LCD_WriteReg(hlcdc, 0x81, parameter, 1);

//    SPI_WriteComm(0xA1);
//    SPI_WriteData(0x05);
    parameter[0] = 0x05;
    LCD_WriteReg(hlcdc, 0xA1, parameter, 1);

//    SPI_WriteComm(0x82);
//    SPI_WriteData(0x06);
    parameter[0] = 0x06;
    LCD_WriteReg(hlcdc, 0x82, parameter, 1);

//    SPI_WriteComm(0xA2);
//    SPI_WriteData(0x04);
    parameter[0] = 0x04;
    LCD_WriteReg(hlcdc, 0xA2, parameter, 1);

//    SPI_WriteComm(0x83);
//    SPI_WriteData(0x39);
    parameter[0] = 0x39;
    LCD_WriteReg(hlcdc, 0x83, parameter, 1);

//    SPI_WriteComm(0xA3);
//    SPI_WriteData(0x39);
    parameter[0] = 0x39;
    LCD_WriteReg(hlcdc, 0xA3, parameter, 1);

//    SPI_WriteComm(0x84);
//    SPI_WriteData(0x3a);
    parameter[0] = 0x3a;
    LCD_WriteReg(hlcdc, 0x84, parameter, 1);

//    SPI_WriteComm(0xA4);
//    SPI_WriteData(0x3a);
    parameter[0] = 0x3a;
    LCD_WriteReg(hlcdc, 0xA4, parameter, 1);

//    SPI_WriteComm(0x85);
//    SPI_WriteData(0x3f);
    parameter[0] = 0x3f;
    LCD_WriteReg(hlcdc, 0x85, parameter, 1);

//    SPI_WriteComm(0xA5);
//    SPI_WriteData(0x3f);
    parameter[0] = 0x3f;
    LCD_WriteReg(hlcdc, 0xA5, parameter, 1);

//    SPI_WriteComm(0x86);
//    SPI_WriteData(0x2c);
    parameter[0] = 0x2c;
    LCD_WriteReg(hlcdc, 0x86, parameter, 1);

//    SPI_WriteComm(0xA6);
//    SPI_WriteData(0x2a);
    parameter[0] = 0x2a;
    LCD_WriteReg(hlcdc, 0xA6, parameter, 1);

//    SPI_WriteComm(0x87);
//    SPI_WriteData(0x43);
    parameter[0] = 0x43;
    LCD_WriteReg(hlcdc, 0x87, parameter, 1);

//    SPI_WriteComm(0xA7);
//    SPI_WriteData(0x47);
    parameter[0] = 0x47;
    LCD_WriteReg(hlcdc, 0xA7, parameter, 1);

//    SPI_WriteComm(0x88);
//    SPI_WriteData(0x08);
    parameter[0] = 0x08;
    LCD_WriteReg(hlcdc, 0x88, parameter, 1);

//    SPI_WriteComm(0xA8);
//    SPI_WriteData(0x08);
    parameter[0] = 0x08;
    LCD_WriteReg(hlcdc, 0xA8, parameter, 1);

//    SPI_WriteComm(0x89);
//    SPI_WriteData(0x0f);
    parameter[0] = 0x0f;
    LCD_WriteReg(hlcdc, 0x89, parameter, 1);

//    SPI_WriteComm(0xA9);
//    SPI_WriteData(0x0f);
    parameter[0] = 0x0f;
    LCD_WriteReg(hlcdc, 0xA9, parameter, 1);

//    SPI_WriteComm(0x8a);
//    SPI_WriteData(0x17);
    parameter[0] = 0x17;
    LCD_WriteReg(hlcdc, 0x8a, parameter, 1);

//    SPI_WriteComm(0xAa);
//    SPI_WriteData(0x17);
    parameter[0] = 0x17;
    LCD_WriteReg(hlcdc, 0xAa, parameter, 1);

//    SPI_WriteComm(0x8b);
//    SPI_WriteData(0x10);
    parameter[0] = 0x10;
    LCD_WriteReg(hlcdc, 0x8b, parameter, 1);

//    SPI_WriteComm(0xAb);
//    SPI_WriteData(0x10);
    parameter[0] = 0x10;
    LCD_WriteReg(hlcdc, 0xAb, parameter, 1);

//    SPI_WriteComm(0x8c);
//    SPI_WriteData(0x16);
    parameter[0] = 0x16;
    LCD_WriteReg(hlcdc, 0x8c, parameter, 1);

//    SPI_WriteComm(0xAc);
//    SPI_WriteData(0x16);
    parameter[0] = 0x16;
    LCD_WriteReg(hlcdc, 0xAc, parameter, 1);

//    SPI_WriteComm(0x8d);
//    SPI_WriteData(0x14);
    parameter[0] = 0x14;
    LCD_WriteReg(hlcdc, 0x8d, parameter, 1);

//    SPI_WriteComm(0xAd);
//    SPI_WriteData(0x14);
    parameter[0] = 0x14;
    LCD_WriteReg(hlcdc, 0xAd, parameter, 1);

//    SPI_WriteComm(0x8e);
//    SPI_WriteData(0x11);
    parameter[0] = 0x11;
    LCD_WriteReg(hlcdc, 0x8e, parameter, 1);

//    SPI_WriteComm(0xAe);
//    SPI_WriteData(0x11);
    parameter[0] = 0x11;
    LCD_WriteReg(hlcdc, 0xAe, parameter, 1);

//    SPI_WriteComm(0x8f);
//    SPI_WriteData(0x14);
    parameter[0] = 0x14;
    LCD_WriteReg(hlcdc, 0x8f, parameter, 1);

//    SPI_WriteComm(0xAf);
//    SPI_WriteData(0x14);
    parameter[0] = 0x14;
    LCD_WriteReg(hlcdc, 0xAf, parameter, 1);

//    SPI_WriteComm(0x90);
//    SPI_WriteData(0x06);
    parameter[0] = 0x06;
    LCD_WriteReg(hlcdc, 0x90, parameter, 1);

//    SPI_WriteComm(0xB0);
//    SPI_WriteData(0x06);
    parameter[0] = 0x06;
    LCD_WriteReg(hlcdc, 0xB0, parameter, 1);

//    SPI_WriteComm(0x91);
//    SPI_WriteData(0x0f);
    parameter[0] = 0x0f;
    LCD_WriteReg(hlcdc, 0x91, parameter, 1);

//    SPI_WriteComm(0xB1);
//    SPI_WriteData(0x0f);
    parameter[0] = 0x0f;
    LCD_WriteReg(hlcdc, 0xB1, parameter, 1);

//    SPI_WriteComm(0x92);
//    SPI_WriteData(0x16);
    parameter[0] = 0x16;
    LCD_WriteReg(hlcdc, 0x92, parameter, 1);

//    SPI_WriteComm(0xB2);
//    SPI_WriteData(0x16);
    parameter[0] = 0x16;
    LCD_WriteReg(hlcdc, 0xB2, parameter, 1);

//    SPI_WriteComm(0xff);
//    SPI_WriteData(0x00);
    parameter[0] = 0x00;
    LCD_WriteReg(hlcdc, 0xff, parameter, 1);

//    SPI_WriteComm(0x11);
    LCD_WriteReg(hlcdc, 0x11, parameter, 0); // internal reg enable
    rt_thread_mdelay(60);

//    SPI_WriteComm(0x29);
    LCD_WriteReg(hlcdc, 0x29, parameter, 0); // internal reg enable
    rt_thread_mdelay(120);

    return;
}

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
 */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif  /* RT_USING_PM */

    memcpy(&lcdc_int_cfg, &lcdc_int_cfg_qadspi, sizeof(lcdc_int_cfg));

    HAL_PIN_Set(PAD_PA01, GPIO_A1, PIN_NOPULL, 1);     // LCDC1_BL_PWM_CTRL, LCD backlight PWM
    extern void BSP_GPIO_Set(int pin, int val, int is_porta);
    BSP_GPIO_Set(1, 1, 1);

    NV3041A_Drv_Init(hlcdc);
    rt_kprintf("NV3041A_Init end!\n");
#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif  /* RT_USING_PM */

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

    data = ((data << 24) & 0xFF000000)
           | ((data <<  8) & 0x00FF0000)
           | ((data >>  8) & 0x0000FF00)
           | ((data >> 24) & 0x000000FF);
    DEBUG_PRINTF("\nNV3041A_ReadID 0x%x \n", data);
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

#ifdef QAD_SPI_USE_GPIO_CS
    gpio_cs_enable();
#endif /* QAD_SPI_USE_GPIO_CS */
    LCD_ReadMode(hlcdc, true);
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
    LCD_ReadMode(hlcdc, false);

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
    DEBUG_PRINTF("NV3041A_ReadPixel[%d,%d]\n", Xpos, Ypos);

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
    LOG_I("NV3041A_SetBrightness\r\n");
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




static const LCD_DrvOpsDef NV3041A_drv =
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


LCD_DRIVER_EXPORT2(nv3041a, THE_LCD_ID, &lcdc_int_cfg,
                   &NV3041A_drv, 2);

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
