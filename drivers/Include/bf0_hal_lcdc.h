/**
  ******************************************************************************
  * @file   bf0_hal_lcdc.h
  * @author Sifli software development team
  * @brief   Header file of LCD controller HAL module.
  * @attention
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BF0_HAL_LCDC_H
#define BF0_HAL_LCDC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "bf0_hal_def.h"
#ifdef HAL_DSI_MODULE_ENABLED
#include "bf0_hal_dsi.h"
#endif /* HAL_DSI_MODULE_ENABLED */


/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup LCDC LCD Controller
  * @brief LCDC HAL module driver
  * @{
  */

#define MAX_LCDC_LAYER  1U

#ifdef SOC_BF0_HCPU
#define HAL_RAMLESS_LCD_ENABLED
#endif /* SOC_BF0_HCPU */

#define INVALID_TOTAL_WIDTH 0xFFFF

#ifndef SF32LB55X
#define RAMLESS_AUTO_REFR_CODE_SIZE_IN_WORD 512

#if defined(SF32LB58X) ||defined(SF32LB56X)
#define LCDC_SUPPORT_H_MIRROR
#endif /* SF32LB58X */

#if defined(SF32LB56X) || defined(SF32LB52X)
#define LCDC_SUPPORT_DDR_QSPI
#endif


#define LCDC_SUPPORT_V_MIRROR
#define LCDC_SUPPORT_LINE_DONE_IRQ
#else
#define RAMLESS_AUTO_REFR_CODE_SIZE_IN_WORD 384

#ifdef SOC_BF0_HCPU
//Use PTC(ch6~8)+Busmonitor(ch4) to implement line irq function.
#define LCDC_SUPPORT_LINE_DONE_IRQ
#endif /* SOC_BF0_HCPU */
#endif /* SF32LB55X */

#ifndef SF32LB52X
#ifdef HAL_EXTDMA_MODULE_ENABLED
#define LCDC_SUPPORTED_COMPRESSED_LAYER
#ifdef SF32LB58X
#define LCDC_SUPPORTED_COMPRESSED_LAYER_MAX_WIDTH 1024
#else
#define LCDC_SUPPORTED_COMPRESSED_LAYER_MAX_WIDTH 512
#endif /* SF32LB58X */
#endif /* HAL_EXTDMA_MODULE_ENABLED */

#define LCDC_SUPPORT_DPI

#ifdef SF32LB58X
#define LCDC_DPI_MAX_WIDTH 1024
#else
#define LCDC_DPI_MAX_WIDTH 512
#endif /* SF32LB58X */

#endif /* SF32LB52X */

// 'v', 'a', 'b' have same origin,
// and get new 'v' after swap 'a' and 'b'
#define FLIP_V_BY_AREA(v, a, b) ((a) + (b) - (v))

/* Exported types ------------------------------------------------------------*/

/** @defgroup LCDC_Exported_Types LCDC Exported Types
  * @brief
  * @{
  */
/** @defgroup LCDC_Output_HW_Interface LCDC Output Interface
  * @{
  */
typedef enum
{
    LCDC_INTF_DBI_8BIT_A,                   //!< MIPI DBI type A interface(8080 8-bit  Clocked E mode)
    LCDC_INTF_DBI_8BIT_B,                   //!< MIPI DBI type B interface(8080 8-bit)
    LCDC_INTF_AHB,                   //!< output to AHB buffer (RAM/PSRAM)

    LCDC_INTF_SPI_START,
    LCDC_INTF_SPI_DCX_1DATA = LCDC_INTF_SPI_START,   //!< SPI which has DCX pin, output with 1 data line
    LCDC_INTF_SPI_DCX_2DATA,        //!< SPI which has DCX pin, output with 2 data line
    LCDC_INTF_SPI_DCX_4DATA,        //!< SPI which has DCX pin, output with 4 data line
    LCDC_INTF_SPI_DCX_4DATA_AUX,    //!< SPI which has DCX pin, output with 4 data line(Drive by PTC)
    LCDC_INTF_SPI_DCX_DDR_4DATA,    //!< SPI which has DCX pin, output with 4 data line(DDR)
    LCDC_INTF_SPI_RSVD,
    LCDC_INTF_SPI_NODCX_1DATA,       //!< SPI which has NO DCX pin, output with 1 data line
    LCDC_INTF_SPI_NODCX_2DATA,       //!< SPI which has NO DCX pin, output with 2 data line
    LCDC_INTF_SPI_NODCX_4DATA,       //!< SPI which has NO DCX pin, output with 4 data line
    LCDC_INTF_SPI_RSVD2,
    LCDC_INTF_SPI_END = LCDC_INTF_SPI_RSVD2,

    LCDC_INTF_DSI,                   //!< DSI interface(Command mode)
    LCDC_INTF_DSI_VIDEO,             //!< DSI video interface
    LCDC_INTF_DPI,                   //!< DPI interface
    LCDC_INTF_DPI_AUX,               //!< DPI interface(Drive by PTC)
    LCDC_INTF_JDI_SERIAL,            //!< JDI serial interface
    LCDC_INTF_JDI_PARALLEL,          //!< JDI parallel interface

    LCDC_INTF_NUM,
} HAL_LCDC_IF_TypeDef;

/**
  * @}
  */




/** @defgroup LCDC_Color_Mode LCDC Pixel format
  * @{
  */
typedef enum
{
    LCDC_PIXEL_FORMAT_MONO = 0,

    LCDC_PIXEL_FORMAT_RGB332,
    LCDC_PIXEL_FORMAT_RGB565, /*Byte0{G[2:0]B[4:0]}, Byte1{R[4:0]G[5:3]}*/
    LCDC_PIXEL_FORMAT_RGB666,
    LCDC_PIXEL_FORMAT_RGB888,
    LCDC_PIXEL_FORMAT_ARGB888,
    LCDC_PIXEL_FORMAT_ARGB565,
    LCDC_PIXEL_FORMAT_A8,
    LCDC_PIXEL_FORMAT_L8,
    LCDC_PIXEL_FORMAT_RGB565_SWAP, /*Swap byte: Byte0{R[4:0]G[5:3]}, Byte1{G[2:0]B[4:0]}*/
} HAL_LCDC_PixelFormat;
/**
  * @}
  */

typedef enum
{
    HAL_LCDC_SYNC_DISABLE             = 0x00U,    /*!< disable frame synchronization */
    HAL_LCDC_SYNC_VER                 = 0x01U,    /*!< only vsync signal mode, pulse trigger default    */
    HAL_LCDC_SYNC_VERHOR              = 0x02U,    /*!< vsync mixed hsync signal mode, edge trigger default  */
} HAL_LCDC_SyncTypeDef;


typedef struct
{
    uint8_t RD_polarity : 1;    //!< Read signal pin polarity
    uint8_t WR_polarity : 1;    //!< Write signal pin polarity
    uint8_t RS_polarity : 1;    //!< Reset signal pin polarity
    uint8_t CS_polarity : 1;    //!< CS signal pin polarity
    uint8_t reserved : 4;
    //uint8_t pwh;       //!<    inactive cycles of LCD_WR/LCD_RD for consecutive write/read operation
    //uint8_t pwl;       //!<    active cycles of LCD_WR/LCD_RD
    //uint8_t tah;       //!<    hold cycles, delay from LCD_WR/LCD_RD inactive to LCD_CS inactive
    //uint8_t tas;       //!<    setup cycles, delay from LCD_CS active to LCD_WR/LCD_RD active

    HAL_LCDC_SyncTypeDef syn_mode;   /*!< vsyn only | vsyn + hsyn mixed | disable */
    uint32_t vsyn_polarity;          /*!< TE pin polarity: 0 - high active and 1 - low active*/
    uint32_t vsyn_delay_us;          /*!< The delay us to send frame buffer right after received TE signal*/
    uint32_t hsyn_num;               /*!< Hsync signal num between two vsync signal */
} DBI_LCD_CFG;


typedef struct
{
    uint32_t lcd_mem;                /*!< address for AHB LCD/ AHB RAM */
    uint32_t lcd_o_width;            /*!< AHB RAM address offset for each line */
} AHB_LCD_Cfg;


#define SPI_LCD_FLAG_DDR_DUMMY_CLOCK  0x00000001
typedef struct
{
    uint32_t dummy_clock;            /*!< Dummy clock between cmd & data in read mode.*/

    HAL_LCDC_SyncTypeDef syn_mode;   /*!< vsyn only | vsyn + hsyn mixed | disable */
    uint32_t cs_polarity:    1;      /*!< CS pin polarity: 0 - low active and 1 - high active*/
    uint32_t clk_polarity:   1;      /*!< CLK pin polarity(CPOL): 0 - low  and  1 - high    */
    uint32_t clk_phase:      1;      /*!< CLK pin phase(CPHA):    0 - phase 0 and 1 - phase 1*/
    uint32_t vsyn_polarity:  1;      /*!< TE pin polarity: 0 - falling edge  and 1 - rasing edge*/
    uint32_t reserved:      28;
    uint32_t vsyn_delay_us;          /*!< The delay us to send frame buffer right after received TE signal*/
    uint32_t hsyn_num;               /*!< Hsync signal num between two vsync signal */

    uint32_t bytes_gap_us;           /*!< Set 0 by default. The minimal gap between every byte, but not for multi pixel writting.*/
    uint32_t readback_from_Dx;       /*!< 0 read back data from D0 (HW SPI support), 1~3 read back from D1~D3(Software SPI support before 52x).*/

    uint32_t flags;


    /*****Configuration for SPI_AUX only START*****/
    uint32_t frame_cmd;
    uint32_t frame_gap;     //!< Gap after every frame, In microsecond
    uint32_t porch_cmd;
    uint32_t line_cmd;
    uint32_t porch_line_gap;      //!< Gap after every porch line, In microsecond
    uint32_t data_line_gap;      //!< Gap after every data line, In microsecond
    uint32_t front_porch;   //!< Front porch line
    uint32_t back_porch;    //!< Back porch line(The frame start will be treated as a porch line)

    uint8_t backlight_pwm;       //!< Output pwm output enable percent
    uint8_t backlight_polarity;  //!< Output polarity
    /*****Configuration for SPI_AUX only END*****/
} SPI_LCD_Cfg;

#ifdef HAL_DSI_MODULE_ENABLED
typedef struct
{
    uint32_t VirtualChannelID;             /*!< Virtual channel ID                                                */
    uint32_t Mode;                         //!< Video mode type, see @ref DSI_Video_Mode_Type
    uint32_t ColorCoding;                  /*!< Color coding for LTDC interface
                                              This parameter can be any value of @ref DSI_Color_Coding          */

    uint16_t VS_width;            //!< VSYNC pulse width, in HSYNC clock(VLW).
    uint16_t HS_width;            //!< HSYNC pulse width, in Pixel clock(HLW).
    uint16_t VBP;                 //!< in HSYNC clock
    uint16_t VAH;
    uint16_t VFP;

    uint16_t HBP;                  //!< in Pixel clock
    uint16_t HAW;
    uint16_t HFP;

    uint16_t interrupt_line_num;
} DSI_VID_CFG;

typedef struct
{
    DSI_InitTypeDef           Init;         /*!< DSI Init parameters    */
    DSI_CmdCfgTypeDef CmdCfg;               /*!< DSI Adapted command mode parameters    */
    DSI_PHY_TimerTypeDef PhyTimings;        /*!< DSI PHY Timings parameters    */
    DSI_HOST_TimeoutTypeDef HostTimeouts;   /*!< DSI Host timeouts definition  */
    DSI_LPCmdTypeDef LPCmd;                 /*!< DSI command transmission mode     */
    DSI_VID_CFG     VidCfg;                 /*!< DSI video mode config parameters    */

    uint32_t vsyn_delay_us;                 /*!< The delay us to send frame buffer right after received TE signal */
} DSI_LCD_CFG;
#endif /* HAL_DSI_MODULE_ENABLED */

typedef struct
{
    uint16_t bank_col_head;     /*In pixel, same as below*/
    uint16_t valid_columns;
    uint16_t bank_col_tail;

    uint16_t bank_row_head;
    uint16_t valid_rows;
    uint16_t bank_row_tail;
} JDI_LCD_CFG;


typedef struct
{
    uint8_t PCLK_polarity : 1;    //!< Pixel clock pin polarity
    uint8_t DE_polarity : 1;      //!< Data enable signal pin polarity
    uint8_t VS_polarity : 1;      //!< VSYNC signal pin polarity
    uint8_t HS_polarity : 1;      //!< HSYNC signal pin polarity
    uint8_t PCLK_force_on : 1;    //!< Never stop PCLK during
    uint8_t reserved : 3;

    uint16_t VS_width;            //!< VSYNC pulse width, in HSYNC clock(VLW).
    uint16_t HS_width;            //!< HSYNC pulse width, in Pixel clock(HLW).
    uint16_t VBP;                 //!< in HSYNC clock
    uint16_t VAH;
    uint16_t VFP;

    uint16_t HBP;                  //!< in Pixel clock
    uint16_t HAW;
    uint16_t HFP;

    uint16_t interrupt_line_num;
} DPI_LCD_CFG;


/** @defgroup LCD_init_config_param LCD init configuration data struct
  * @brief LCD Init structure definition
  * @{
  */
typedef struct
{
    HAL_LCDC_IF_TypeDef lcd_itf;     /*!< LCD HW connection interface .*/
    uint32_t freq;                   /*!< Interface clock frequency.*/
    HAL_LCDC_PixelFormat color_mode; /*!< The color format of LCDC output*/

    union
    {
        DBI_LCD_CFG dbi;             /*!< DBI LCD interface config  */
        AHB_LCD_Cfg ahb;             /*!< AHB LCD interface config  */
        SPI_LCD_Cfg spi;             /*!< SPI LCD interface config  */
#ifdef HAL_DSI_MODULE_ENABLED
        DSI_LCD_CFG dsi;         /*!< DSI LCD interface config  */
#endif /* HAL_DSI_MODULE_ENABLED */
        JDI_LCD_CFG jdi;             /*!< JDI LCD interface config  */
        DPI_LCD_CFG dpi;             /*!< DPI LCD interface config  */
    } cfg;                           /*!< LCD interface config union*/
} LCDC_InitTypeDef;
/**
  * @}
  */


/**
  * @brief LCDC color structure definition
  */
typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} LCDC_ColorDef;

/**
  * @brief LCDC alpha color structure definition
  */
typedef struct
{
    uint8_t a;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} LCDC_AColorDef;


/**
  * @brief LCDC area structure definition
  */
typedef struct
{
    uint16_t x0;
    uint16_t y0;
    uint16_t x1;
    uint16_t y1;
} LCDC_AreaDef;

/**
  * @brief LCDC Layer structure definition
  */
typedef struct
{
    uint8_t *data;                      //!< Layer buffer ptr
    HAL_LCDC_PixelFormat data_format;   //!< Layer data pixel format

    LCDC_ColorDef chroma_key;           //!< Layer chroma key color
    LCDC_ColorDef bg_color;             //!< Layer background color for A8 format data only
    const LCDC_AColorDef *lookup_table; //!< Layer lookup table for L8 format, it's length must be 256*sizeof(LCDC_AColorDef)
    LCDC_AreaDef data_area;             //!< Layer data position     (Origin is LCD top-left, same as below)
    uint16_t     total_width;           //!< Layer original line width in pixels. Equals to INVALID_TOTAL_WIDTH if use 'data_area' width instead.

    uint8_t cmpr_rate;                  //!< Layer buffer's compress rate
    uint8_t alpha;                      //!< Layer alpha value

    uint16_t cmpr_en         : 1;       //!< Layer buffer is compressed
    uint16_t chroma_key_en   : 1;       //!< Enable layer chroma key
    uint16_t layer_alpha_en  : 1;       //!< Use layer alpha if enabled
    uint16_t disable         : 1;       //!< Disable layer
    uint16_t h_mirror        : 1;       //!< Horizontal mirror
    uint16_t v_mirror        : 1;       //!< Vertical mirror
    uint16_t reserved        : 10;

} LCDC_LayerCfgTypeDef;

/**
  * @brief  HAL LCDC State structures definition
  */
typedef enum
{
    HAL_LCDC_STATE_RESET             = 0x00U,    /*!< LCDC not yet initialized or disabled       */
    HAL_LCDC_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
    HAL_LCDC_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing              */
    HAL_LCDC_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                               */
    HAL_LCDC_STATE_ERROR             = 0x04U,    /*!< LCDC state error                           */
    HAL_LCDC_STATE_SUSPEND           = 0x05U,    /*!< LCDC process is suspended                  */
    HAL_LCDC_STATE_LOWPOWER          = 0x06U     /*!< LCDC enter low power mode                  */
} HAL_LCDC_StateTypeDef;


typedef enum
{
    HAL_LCDC_LAYER_0 = 0,                           //!< Layer 0, support compressed buffer
    HAL_LCDC_LAYER_DEFAULT = HAL_LCDC_LAYER_0,      //!< Default layer
    HAL_LCDC_LAYER_1,                               //!< Layer 1  is upon layer 1, NOT support compressed buffer
    HAL_LCDC_LAYER_MAX
} HAL_LCDC_LayerDef;

typedef enum
{
    HAL_LCDC_ROTATE_0   = 0,
    HAL_LCDC_ROTATE_180 = 180,
} HAL_LCDC_RotateDef;


typedef enum
{
    SW_SPI_CS,
    SW_SPI_CLK,
    SW_SPI_D0,
    SW_SPI_D1,
    SW_SPI_D2,
    SW_SPI_D3,
    SW_SPI_DCX,
} SOFT_SPI_PIN_Def;

typedef enum
{
    SW_SPI_INPUT,
    SW_SPI_OUTPUT,
} SOFT_SPI_IO_Def;


#ifdef HAL_DSI_MODULE_ENABLED
typedef void (*SetRegion_cb_t)(void *hlcdc, uint16_t Xpos0, uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1);
#endif

typedef struct __LCDC_HandleTypeDef
{
    LCD_IF_TypeDef            *Instance;                                                    /*!< LCDC register base address.              */

    LCDC_InitTypeDef          Init;                                                         /*!< LCDC communication parameters.           */

    LCDC_LayerCfgTypeDef      Layer[HAL_LCDC_LAYER_MAX];                                    /*!< LCDC layer parameters.           */

#ifdef HAL_DSI_MODULE_ENABLED
    DSI_HandleTypeDef         hdsi;                                                        /*!< Wrapper DSI handler.           */
#endif /* HAL_DSI_MODULE_ENABLED */

    void (* XferCpltCallback)(struct __LCDC_HandleTypeDef *lcdc);                          /*!< LCDC transfer complete callback.          */

    void (* XferErrorCallback)(struct __LCDC_HandleTypeDef *lcdc);                         /*!< LCDC transfer error callback.             */
#ifdef LCDC_SUPPORT_LINE_DONE_IRQ
    void (* XferLineCallback)(struct __LCDC_HandleTypeDef *lcdc, uint32_t line_num);       /*!< LCDC transfer line done callback.             */
    uint16_t irq_lines;                                                                    /*!< Call 'XferLineCallback' every 'irq_lines' were transferred.  */
#endif /* LCDC_SUPPORT_LINE_DONE_IRQ */


#ifdef HAL_DSI_MODULE_ENABLED
    /*   dsi patial write  configurations*/
    LCDC_AreaDef part_write_roi_bak;
    //LCDC_AreaDef part_write_data_area_bak;
    //uint8_t     *part_write_data_bak;
    uint16_t     part_write_y;
    uint16_t     part_write_lines;      //!< Cut layer to some continued horizontal lines

    SetRegion_cb_t lcd_rg_cb;   //!<  cbk to set lcd region
    uint8_t dsi_lcd_w_cmd;      //!<  cmd to write GRAM
#endif /* HAL_DSI_MODULE_ENABLED */

#ifdef HAL_RAMLESS_LCD_ENABLED
    uint32_t *ptc_code;
    uint32_t need_cplt_ckb;
    uint8_t  running; //!<  ptc_code is running
    uint8_t *sram_buf0;
    uint8_t *sram_buf1;
    uint32_t sram_buf_bytes;
#endif /* HAL_RAMLESS_LCD_ENABLED */

    LCDC_AreaDef roi;                   //!< Clip area position (Origin is LCD top-left, same as below)
    LCDC_ColorDef bg;                   //!< LCDC default background color

    HAL_LockTypeDef             Lock;                                                         /*!< LCDC lock.                                */

    __IO HAL_LCDC_StateTypeDef State;                                                       /*!< LCDC transfer state.                     */

    __IO uint32_t               ErrorCode;                                                    /*!< LCDC error code.                         */

    uint8_t                     Next_Frame_TE;           /*!< The TE configuration of Next Frame*/

    uint32_t  use_lcdc2_te : 1;                         /*!< Use LCDC2 'TE in LCDC1*/
    uint32_t  reversed     : 31;

    uint32_t  debug_cnt0;
    uint32_t  debug_cnt1;
    uint32_t  debug_cnt2;

} LCDC_HandleTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup LCDC_Exported_Constants LCDC Exported Constants
  * @{
  */


/** @defgroup LCDC_Rest_Pulse_Type LCDC Reset Pulse Type
  * @brief Keep RESTB pin in NEG/POS level for a period of time
  * @{
  */
#define LCDC_RESX_NEG_PULSE  0
#define LCDC_RESX_POS_PULSE  1
/**
  * @}
  */


/** @defgroup LCDC_Error_Code LCDC Error Code
  * @{
  */
#define HAL_LCDC_ERROR_NONE                 0x00000000U    /*!< No error                                */
#define HAL_LCDC_ERROR_TIMEOUT              0x00000001U    /*!< Wait busy timeout error                 */
#define HAL_LCDC_ERROR_OVERFLOW             0x00000002U    /*!< Decompress buf overflow error           */
#define HAL_LCDC_ERROR_UNDERRUN             0x00000004U    /*!< JDI/DPI under run error                 */
#define HAL_LCDC_ERROR_HAL_LOCKED           0x80000000U    /*!< HAL LCDC locked error                   */

/**
  * @}
  */


/**
  * @} LCDC_Exported_Constants
  */


/* Exported macros -----------------------------------------------------------*/
/** @defgroup LCDC_Exported_Macros LCDC Exported Macros
  * @brief
  * @{
  */

#define HAL_LCDC_IS_SPI_IF(lcd_itf) (((lcd_itf) >= LCDC_INTF_SPI_START) && ((lcd_itf) <= LCDC_INTF_SPI_END))
#define HAL_LCDC_IS_AHB_IF(lcd_itf) ((lcd_itf) == LCDC_INTF_AHB)
#define HAL_LCDC_IS_DBI_IF(lcd_itf) (((lcd_itf) == LCDC_INTF_DBI_8BIT_A) || ((lcd_itf) == LCDC_INTF_DBI_8BIT_B))
#define HAL_LCDC_IS_DSI_IF(lcd_itf) (((lcd_itf) == LCDC_INTF_DSI) || ((lcd_itf) == LCDC_INTF_DSI_VIDEO))
#define HAL_LCDC_IS_DSI_CMD_IF(lcd_itf) ((lcd_itf) == LCDC_INTF_DSI)
#define HAL_LCDC_IS_DSI_VID_IF(lcd_itf) ((lcd_itf) == LCDC_INTF_DSI_VIDEO)
#define HAL_LCDC_IS_DPI_IF(lcd_itf) (((lcd_itf) == LCDC_INTF_DPI)||((lcd_itf) == LCDC_INTF_DPI_AUX))
#define HAL_LCDC_IS_JDI_IF(lcd_itf) (((lcd_itf) == LCDC_INTF_JDI_SERIAL) || ((lcd_itf) == LCDC_INTF_JDI_PARALLEL))
#define HAL_LCDC_IS_JDI_PARALLEL_IF(lcd_itf) ((lcd_itf) == LCDC_INTF_JDI_PARALLEL)
#define HAL_LCDC_IS_JDI_SERIAL_IF(lcd_itf) ((lcd_itf) == LCDC_INTF_JDI_SERIAL)
#define HAL_LCDC_IS_PTC_AUX_IF(lcd_itf)  (((lcd_itf) == LCDC_INTF_DPI_AUX) || ((lcd_itf) == LCDC_INTF_SPI_DCX_4DATA_AUX))

#define HAL_LCDC_LOOKUP_TABLE_SIZE          (256*4)

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup LCDC_Exported_Functions LCDC Exported Functions
  * @brief
  * @{
  */

/**
* @brief  Initialize LCD controller hardware
* @param  lcdc LCD controller handle
* @retval HAL status
*/
HAL_StatusTypeDef HAL_LCDC_Init(LCDC_HandleTypeDef *lcdc);

/**
* @brief  De-Initialize LCD controller hardware
* @param  lcdc LCD controller handle
* @retval HAL status
*/
HAL_StatusTypeDef HAL_LCDC_DeInit(LCDC_HandleTypeDef *lcdc);

//Obsolete
HAL_StatusTypeDef HAL_LCDC_ResetLCD(LCDC_HandleTypeDef *lcdc, uint8_t pulse_type, uint32_t pulse_us);

/**
* @brief  Reset LCD controller hardware
* @param  lcdc LCD controller handle
* @retval HAL status
*/
HAL_StatusTypeDef HAL_LCDC_Reset(LCDC_HandleTypeDef *lcdc);

/**
 * @brief Change LCDC output format
 * @param lcdc - LCD controller handle
 * @param format - LCDC output format
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_SetOutFormat(LCDC_HandleTypeDef *lcdc, HAL_LCDC_PixelFormat format);


/**
 * @brief Update LCDC output frequecy
 * @param lcdc - LCD controller handle
 * @param freq - Frequency in Hz
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_SetFreq(LCDC_HandleTypeDef *lcdc, uint32_t freq);


/**
 * @brief Set LCDC output ROI area by an area pointer
 * @param lcdc - LCD controller handle
 * @param area - ROI area
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_SetROI(LCDC_HandleTypeDef *lcdc, LCDC_AreaDef *area);

/**
 * @brief Set LCDC output ROI area by coordinate
 * @param lcdc - LCD controller handle
 * @param x_0  - ROI area left to LCD top-left pixels
 * @param x_1  - ROI area right to LCD top-left pixels
 * @param y_0  - ROI area top to LCD top-left pixels
 * @param y_1  - ROI area bottom to LCD top-left pixels
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_SetROIArea(LCDC_HandleTypeDef *lcdc, uint16_t x_0, uint16_t y_0, uint16_t x_1, uint16_t y_1);

/**
 * @brief Reset layer configuration
 * @param  lcdc LCD controller handle
 * @param layeridx - Layer index
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerReset(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx);

/**
 * @brief Setup layer buffer compression ratio
 * @param  lcdc LCD controller handle
 * @param layeridx - Layer index
 * @param cmpr - Layer buffer compression ratio
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerSetCmpr(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx, uint8_t cmpr);

/**
 * @brief Get layer buffer compression ratio
 * @param  lcdc LCD controller handle
 * @param layeridx - Layer index
 * @return compression ratio
 */
uint8_t HAL_LCDC_LayerGetCmpr(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx);

/**
 * @brief Ser layer buffer color format
 * @param  lcdc LCD controller handle
 * @param layeridx - Layer index
 * @param format - Buffer format
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerSetFormat(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx, HAL_LCDC_PixelFormat format);

/**
 * @brief Set layer buffer data and it's coordinate
 * @param  lcdc LCD controller handle
 * @param pData - Layer data
 * @param x0  - Layer left to LCD top-left pixels
 * @param x1  - Layer right to LCD top-left pixels
 * @param y0  - Layer top to LCD top-left pixels
 * @param y1  - Layer bottom to LCD top-left pixels
 * @param layeridx - Layer index
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerSetData(
    LCDC_HandleTypeDef *lcdc,
    HAL_LCDC_LayerDef layeridx,
    uint8_t *pData,
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1);

/**
 * @brief Set layer buffer data and it's coordinate
 * @param  lcdc LCD controller handle
 * @param pData - Layer data
 * @param x0  - Layer left to LCD top-left pixels
 * @param x1  - Layer right to LCD top-left pixels
 * @param y0  - Layer top to LCD top-left pixels
 * @param y1  - Layer bottom to LCD top-left pixels
 * @param total_width  - Layer original data width in pixels
 * @param layeridx - Layer index
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerSetDataExt(
    LCDC_HandleTypeDef *lcdc,
    HAL_LCDC_LayerDef layeridx,
    uint8_t *pData,
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1,
    uint16_t total_width);


/**
 * @brief Set layer background color (It is visible only while blended with LCDC_PIXEL_FORMAT_A8 layer format for now)
 * @param  lcdc LCD controller handle
 * @param  layeridx - Layer index
 * @param  r  red[0~255]
 * @param  g  green[0~255]
 * @param  b  blue[0~255]
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerSetBgColor(
    LCDC_HandleTypeDef *lcdc,
    HAL_LCDC_LayerDef layeridx,
    uint8_t r, uint8_t g, uint8_t b);



/**
 * @brief Set layer's look up table
 * @param  lcdc LCD controller handle
 * @param layeridx - Layer index
 * @param pLTab - Look up table
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerSetLTab(LCDC_HandleTypeDef *lcdc,    HAL_LCDC_LayerDef layeridx, const LCDC_AColorDef *pLTab);

/**
 * @brief Set layer chroma key color
 * @param  lcdc LCD controller handle
 * @param  layeridx - Layer index
 * @param  r  red[0~255]
 * @param  g  green[0~255]
 * @param  b  blue[0~255]
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerEnableChromaKey(
    LCDC_HandleTypeDef *lcdc,
    HAL_LCDC_LayerDef layeridx,
    uint8_t r, uint8_t g, uint8_t b);


/**
 * @brief Disable layer chroma key
 * @param  lcdc LCD controller handle
 * @param layeridx - Layer index
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerDisableChromaKey(LCDC_HandleTypeDef *lcdc,    HAL_LCDC_LayerDef layeridx);

/**
 * @brief Set layer global alpha value and enable it
 * @param  lcdc LCD controller handle
 * @param layeridx - Layer index
 * @param alpha - Layer alpha value[0~255]
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerEnableAlpha(LCDC_HandleTypeDef *lcdc,    HAL_LCDC_LayerDef layeridx, uint8_t alpha);

/**
 * @brief Disable layer's global alpha
 * @param  lcdc LCD controller handle
 * @param layeridx - Layer index
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerDisableAlpha(LCDC_HandleTypeDef *lcdc,    HAL_LCDC_LayerDef layeridx);


/**
 * @brief Set sepcified layer's rotate angle
 * @param lcdc - LCD controller handle
 * @param layeridx - Layer index
 * @param angle - Rotate angle
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerRotate(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx, HAL_LCDC_RotateDef angle);


/**
 * @brief Enable layer
 * @param  lcdc LCD controller handle
 * @param layeridx - Layer index
 * @param en - enable
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerVMirror(LCDC_HandleTypeDef *lcdc, HAL_LCDC_LayerDef layeridx, bool en);


/**
 * @brief Enable layer
 * @param  lcdc LCD controller handle
 * @param layeridx - Layer index
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerEnable(LCDC_HandleTypeDef *lcdc,    HAL_LCDC_LayerDef layeridx);
/**
 * @brief Disable layer
 * @param  lcdc LCD controller handle
 * @param layeridx - Layer index
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_LayerDisable(LCDC_HandleTypeDef *lcdc,    HAL_LCDC_LayerDef layeridx);
/**
 * @brief Set background color (RGB888 format)
 * @param  lcdc LCD controller handle
 * @param  r  red[0~255]
 * @param  g  green[0~255]
 * @param  b  blue[0~255]
 * @return HAL status
 */
HAL_StatusTypeDef HAL_LCDC_SetBgColor(LCDC_HandleTypeDef *lcdc, uint8_t r, uint8_t g, uint8_t b);

/**
* @brief  LCD controller interrupt handler
* @param  lcdc LCD controller handle
* @retval None
*/
void HAL_LCDC_IRQHandler(LCDC_HandleTypeDef *lcdc);

/**
* @brief  Send layer data when TE comming
* @param  lcdc LCD controller handle
* @retval None
*/
void HAL_LCDC_TE_IRQHandler(LCDC_HandleTypeDef *lcdc);

/**
 * @brief Read data(s) from LCD register
 * @param  lcdc LCD controller handle
 * @param  addr LCD register address(For example: 0x12345678 will be sent as 0x12, 0x34, 0x56, 0x78)
 * @param  addr_len - LCD register address length
 * @param  p_data read back buffer
 * @param  data_len read back buffer size in byte
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_LCDC_ReadDatas(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len, uint8_t *p_data, uint32_t data_len);

/**
 * @brief Read data(s) from LCD register which address length is 8bit
 * @param  lcdc LCD controller handle
 * @param  U8Reg Unsigned 8bit LCD register address
 * @param  p_data read back buffer
 * @param  data_len read back buffer size in byte
 * @retval HAL status
 */
#define HAL_LCDC_ReadU8Reg(lcdc, U8Reg, p_data, data_len) HAL_LCDC_ReadDatas(lcdc, (uint32_t)U8Reg, 1, p_data, data_len)

/**
 * @brief Read data(s) from LCD register which address length is 16bit
 * @param  lcdc LCD controller handle
 * @param  U16Reg Unsigned 16bit LCD register address
 * @param  p_data read back buffer
 * @param  data_len read back buffer size in byte
 * @retval HAL status
 */
#define HAL_LCDC_ReadU16Reg(lcdc, U16Reg, p_data, data_len) HAL_LCDC_ReadDatas(lcdc, (uint32_t)U16Reg, 2, p_data, data_len)

/**
 * @brief Read data(s) from LCD register which address length is 32bit
 * @param  lcdc LCD controller handle
 * @param  U32Reg Unsigned 32bit LCD register address
 * @param  p_data read back buffer
 * @param  data_len read back buffer size in byte
 * @retval HAL status
 */
#define HAL_LCDC_ReadU32Reg(lcdc, U32Reg, p_data, data_len) HAL_LCDC_ReadDatas(lcdc, (uint32_t)U32Reg, 4, p_data, data_len)


/**
 * @brief Write data(s) to LCD register
 * @param  lcdc LCD controller handle
 * @param  addr LCD register address(For example: 0x12345678 will be sent as 0x12, 0x34, 0x56, 0x78)
 * @param  addr_len - LCD address register length
 * @param  p_data read back buffer
 * @param  data_len read back buffer size in byte
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_LCDC_WriteDatas(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len, uint8_t *p_data, uint32_t data_len);

/**
 * @brief Write data(s) to LCD register which address length is 8bit
 * @param  lcdc LCD controller handle
 * @param  U8Reg Unsigned 8bit LCD register address
 * @param  p_data read back buffer
 * @param  data_len read back buffer size in byte
 * @retval HAL status
 */
#define HAL_LCDC_WriteU8Reg(lcdc, U8Reg, p_data, data_len) HAL_LCDC_WriteDatas(lcdc, (uint32_t)U8Reg, 1, p_data, data_len)



/**
 * @brief Set SPI interface configuration
 * @param  lcdc LCD controller handle
 * @param  config SPI interface type, must between LCDC_INTF_SPI_START and LCDC_INTF_SPI_END
 * @retval Previous LCDC interface configuration
 */
HAL_LCDC_IF_TypeDef HAL_LCDC_SetSPICfg(LCDC_HandleTypeDef *lcdc, HAL_LCDC_IF_TypeDef config);


/**
 * @brief Write data(s) to LCD register which address length is 16bit
 * @param  lcdc LCD controller handle
 * @param  U16Reg Unsigned 16bit LCD register address
 * @param  p_data read back buffer
 * @param  data_len read back buffer size in byte
 * @retval HAL status
 */
#define HAL_LCDC_WriteU16Reg(lcdc, U16Reg, p_data, data_len) HAL_LCDC_WriteDatas(lcdc, (uint32_t)U16Reg, 2, p_data, data_len)

/**
 * @brief Write data(s) to LCD register which address length is 32bit
 * @param  lcdc LCD controller handle
 * @param  U32Reg Unsigned 32bit LCD register address
 * @param  p_data read back buffer
 * @param  data_len read back buffer size in byte
 * @retval HAL status
 */
#define HAL_LCDC_WriteU32Reg(lcdc, U32Reg, p_data, data_len) HAL_LCDC_WriteDatas(lcdc, (uint32_t)U32Reg, 4, p_data, data_len)


/**
 * @brief Start write layer data DMA mode directly
 * @param  lcdc LCD controller handle
 * @retval HAL status
 */
#define HAL_LCDC_SendLayerData_IT(lcdc) HAL_LCDC_SendLayerData2Reg_IT(lcdc,0,0)
#define HAL_LCDC_SendLayerData(lcdc)    HAL_LCDC_SendLayerData2Reg(lcdc,0,0)

/**
 * @brief Send an address and layer data to LCD in asynchronized mode
 * @param  lcdc LCD controller handle
 * @param addr - Address to send
 * @param addr_len - Address length in byte
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_LCDC_SendLayerData2Reg_IT(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len);

/**
 * @brief Send an address and layer data to LCD in synchronized mode
 * @param  lcdc LCD controller handle
 * @param addr - Address to send
 * @param addr_len - Address length in byte
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_LCDC_SendLayerData2Reg(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len);

/**
* @brief  LCD controller send layer data complete callback.
* @param  lcdc LCD controller handle
* @retval None
*/
void HAL_LCDC_SendLayerDataCpltCbk(LCDC_HandleTypeDef *lcdc);

/**
* @brief  LCD controller sent lines(Base on HAL_LCDC_SetROIArea's y0) complete callback.
* @param  lcdc LCD controller handle
* @param  line Sent lines
* @retval None
*/
void HAL_LCDC_SendLineCpltCbk(LCDC_HandleTypeDef *lcdc, uint32_t line);

/**
* @brief  Setup next frame's TE, auto restore to default TE after send framebuffer
* @param  lcdc LCD controller handle
* @param  en  enable TE if ture
* @retval None
*/
void HAL_LCDC_Next_Frame_TE(LCDC_HandleTypeDef *lcdc, bool en);
/**
* @brief  Enable/Disable TE
* @param  lcdc LCD controller handle
* @param  en  enable TE if ture
* @retval None
*/
void HAL_LCDC_Enable_TE(LCDC_HandleTypeDef *lcdc, bool en);

/**
 * @brief Let LCDC enter low power mode
 * @param  lcdc LCD controller handle
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_LCDC_Enter_LP(LCDC_HandleTypeDef *lcdc);

/**
 * @brief Wake up LCDC from low power mode
 * @param  lcdc LCD controller handle
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_LCDC_Exit_LP(LCDC_HandleTypeDef *lcdc);

/**
 * @brief Resume LCDC all registers from standby mode
 * @param  lcdc LCD controller handle
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_LCDC_Resume(LCDC_HandleTypeDef *lcdc);

#if !(defined(SF32LB55X) || defined(SOC_BF0_LCPU))
/**
* @brief  Use LCDC2' TE signal on LCDC1
* @param  lcdc LCD controller handle
* @retval None
*/
void HAL_LCDC_UseLCDC2TE(LCDC_HandleTypeDef *lcdc);
#endif

#ifdef HAL_RAMLESS_LCD_ENABLED
/**
 * @brief Initial RAMLESS HW FSM
 * @param  lcdc LCD controller handle
 * @param ptc_buf - Pointer to buffer for saving HW auto refresh code, size is RAMLESS_AUTO_REFR_CODE_SIZE_IN_WORD
 * @param  sram_buf0 - Ping-Pong buffer for DPI
 * @param  sram_buf1 - Ping-Pong buffer for DPI
 * @param  sram_buf_bytes - Ping-Pong buffer size
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_LCDC_RAMLESS_Init_Ext(LCDC_HandleTypeDef *lcdc, uint32_t *ptc_buf,
        uint8_t *sram_buf0, uint8_t *sram_buf1, uint32_t sram_buf_bytes);
#define HAL_LCDC_RAMLESS_Init(hlcdc, ptc_buf) HAL_LCDC_RAMLESS_Init_Ext(hlcdc, ptc_buf, NULL, NULL, 0)

/**
 * @brief Start RAMLESS HW FSM
 * @param  lcdc LCD controller handle
 * @retval HAL status
 */
#define HAL_LCDC_RAMLESS_Start(lcdc)  HAL_LCDC_SendLayerData2Reg_IT(lcdc,0,0)

/**
 * @brief Stop HW FSM
 * @param  lcdc LCD controller handle
 * @retval HAL status
 */
#define HAL_LCDC_RAMLESS_Stop(lcdc)

void HAL_RAMLESS_LCD_IRQHandler(LCDC_HandleTypeDef *lcdc);

/**
 * @brief Commit an reading LCD register action during RAMLESS HW FSM working
 * @param  lcdc LCD controller handle
 * @param  freq The frequency of reading
 * @param  addr LCD register address(For example: 0x12345678 will be sent as 0x12, 0x34, 0x56, 0x78)
 * @param  addr_len - LCD register address length
 * @param  data_len read back buffer size in byte
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_LCDC_RAMLESS_ReadDatasStart(LCDC_HandleTypeDef *lcdc, uint32_t freq, uint32_t addr, uint32_t addr_len, uint32_t data_len);
/**
 * @brief  Get the result of last committed reading action.
 * @param  lcdc LCD controller handle
 * @param  p_data read back buffer
 * @param  data_len read back buffer size in byte
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_LCDC_RAMLESS_ReadDatasEnd(LCDC_HandleTypeDef *lcdc, uint8_t *p_data, uint32_t data_len);
/**
 * @brief Commit an writing register action during RAMLESS HW FSM working
 * @param  lcdc LCD controller handle
 * @param  addr LCD register address(For example: 0x12345678 will be sent as 0x12, 0x34, 0x56, 0x78)
 * @param  addr_len - LCD register address length
 * @param  p_data read back buffer
 * @param  data_len read back buffer size in byte
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_LCDC_RAMLESS_WriteDatasStart(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len, uint8_t *p_data, uint32_t data_len);

/**
 * @brief  Get the result of last committed writing action.
 * @param  lcdc LCD controller handle
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_LCDC_RAMLESS_WriteDatasEnd(LCDC_HandleTypeDef *lcdc);
#endif /* HAL_RAMLESS_LCD_ENABLED */

#if defined(LCDC_SUPPORT_LINE_DONE_IRQ) && defined(SF32LB55X)
void HAL_LCD_LineIRQHandler(LCDC_HandleTypeDef *lcdc);
#endif /* LCDC_SUPPORT_LINE_DONE_IRQ &&  SF32LB55X */

void HAL_LCDC_PTC_IRQHandler(LCDC_HandleTypeDef *lcdc);

void HAL_LCDC_SoftSpiInit(SOFT_SPI_PIN_Def pin, SOFT_SPI_IO_Def inout, uint32_t high1low0);
void HAL_LCDC_SoftSpiDeinit(SOFT_SPI_PIN_Def pin);
uint32_t HAL_LCDC_SoftSpiGetPin(SOFT_SPI_PIN_Def pin);
void HAL_LCDC_SoftSpiSetPin(SOFT_SPI_PIN_Def pin, uint32_t high1low0);
uint32_t HAL_LCDC_SoftSPI_Read(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len, uint32_t data_len);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /* BF0_HAL_MAILBOX_H */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
