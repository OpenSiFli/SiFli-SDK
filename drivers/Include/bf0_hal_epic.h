/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BF0_HAL_EPIC_H
#define BF0_HAL_EPIC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "bf0_hal_def.h"
#include "sifli_matrix.h"


#ifndef SF32LB55X /* Features supported after 55x */
#define EPIC_SUPPORT_MASK
#define EPIC_SUPPORT_MONOCHROME_LAYER
#define EPIC_SUPPORT_A4
#define EPIC_SUPPORT_A8
#define EPIC_SUPPORT_L8
#ifndef SF32LB58X /* Features supported after 58x */
#define EPIC_SUPPORT_YUV
#define EPIC_SUPPORT_DITHER
#ifndef SF32LB56X /* Features supported after 56x */
#define EPIC_SUPPORT_A2
#ifndef SF32LB52X /* Features supported after 52x */
#define EPIC_SUPPORT_JPEGD
#define EPIC_SUPPORT_DECOMP
#define EPIC_SUPPORT_COMP
#define EPIC_SUPPORT_COLOR_MATRIX
// #define HAL_EPICTL_ENABLED
#define EPIC_SUPPORT_L4
#define EPIC_SUPPORT_GREYSCALE
#define EPIC_SUPPORT_OUTPUT_A8
#define EPIC_SUPPORT_TRANS_MATRIX
#endif /* SF32LB52X */
#endif /* SF32LB56X */
#endif /* SF32LB58X */
#endif /* SF32LB55X */


/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup EPIC EPIC
  * @brief EPIC HAL module driver
  * @{
  */

#ifndef EPIC_DEBUG
#define EPIC_DEBUG

//#define EPIC_CONT_BLEND_DEBUG
#endif /* EPIC_DEBUG */

#define MAX_EPIC_LAYER  4U


#define EPIC_COLOR_RGB565          (0)              /**< RGB565 color format */
#define EPIC_COLOR_ARGB8565        (1)              /**< ARGB8565 color format */
#define EPIC_COLOR_RGB888          (2)              /**< RGB888 color format */
#define EPIC_COLOR_ARGB8888        (3)              /**< ARGB8888 color format */
#define EPIC_COLOR_L8              (4)              /**< L8 color format */
#define EPIC_COLOR_A8              (5)              /**< A8 color format */
#define EPIC_COLOR_A4              (6)              /**< A4 color format */
#define EPIC_COLOR_A2              (7)              /**< A2 color format */
#define EPIC_COLOR_MONO            (8)              /**< Monochrome color format */
#define EPIC_COLOR_GRAY8              (9)              /**< Grayscale 8bit color format */
#define EPIC_COLOR_GRAY4              (0xA)             /**< Grayscale 4bit color format */
#define EPIC_COLOR_GRAY2              (0xB)             /**< Grayscale 2bit color format */
#define EPIC_COLOR_L4              (0xC)             /**< L4 color format */

#define EPIC_COLOR_SWAP_FLAG       (0x40)           /**< Color format swap flag */
#define EPIC_COLOR_A4_SWAP         (EPIC_COLOR_SWAP_FLAG | EPIC_COLOR_A4)       /**< A4 color format with swapped pixels */
#define EPIC_COLOR_A2_SWAP         (EPIC_COLOR_SWAP_FLAG | EPIC_COLOR_A2)       /**< A2 color format with swapped pixels */
#define EPIC_COLOR_GRAY4_SWAP         (EPIC_COLOR_SWAP_FLAG | EPIC_COLOR_GRAY4)       /**< Grayscale 4bit color format with swapped pixels */
#define EPIC_COLOR_GRAY2_SWAP         (EPIC_COLOR_SWAP_FLAG | EPIC_COLOR_GRAY2)       /**< Grayscale 2bit color format with swapped pixels */
#define EPIC_COLOR_L4_SWAP         (EPIC_COLOR_SWAP_FLAG | EPIC_COLOR_L4)       /**< L4 color format with swapped pixels */
#define EPIC_COLOR_RGB565_SWAP     (EPIC_COLOR_SWAP_FLAG | EPIC_COLOR_RGB565)  /**< RGB565 color format with swapped bytes */


#define EPIC_COLOR_EZIP_FLAG        (0x80)           /**< EZIP color format flag, supported after A0*/
#define EPIC_COLOR_EZIP             (0x81)           /**< EZIP color format */

/**
   YUV format example:

   1. YUV422_PACKED_YUYV:
           YUYV_YUYV_YUYV_..._YUYV

   2. YUV420_PLANAR(YUV420P):
          YYYY_YYYY_YYYY_..._YYYY
          U_U_U_..._U
          V_V_V_..._V

   4. YUV420_SEMI_PLANAR(YUV420SP):
          YYYY_YYYY_YYYY_..._YYYY
          UV_UV_UV_..._UV

*/
#define EPIC_COLOR_YUV_FLAG                  (0x100)
#define EPIC_COLOR_YUV422_PACKED_YUYV        (0x101)              /**< YUYV */
#define EPIC_COLOR_YUV422_PACKED_UYVY        (0x102)              /**< UYUV */
#define EPIC_COLOR_YUV420_PLANAR             (0x103)              /**< iYUV */
//#define EPIC_COLOR_YUV420_SEMI_PLANAR      (0x104)              /**< Reserved */

#define EPIC_COLOR_JPEG_FLAG                 (0x200)
#define EPIC_COLOR_JPEG                      (0x200)

#define EPIC_COLOR_FLEX_FLAG                 (0x400)           /**< Flex color format flag, supported after 57x*/
#define EPIC_COLOR_F2_332                    (0x401)           /**< Flex 2bit color format */

#define EPIC_IS_EZIP_COLOR_MODE(c)       (EPIC_COLOR_EZIP_FLAG == (EPIC_COLOR_EZIP_FLAG&(c)))
#define EPIC_IS_YUV_COLOR_MODE(c)       (EPIC_COLOR_YUV_FLAG == (EPIC_COLOR_YUV_FLAG&(c)))
#define EPIC_IS_LUT_COLOR_MODE(c)       ((EPIC_COLOR_L8 == (c))||(EPIC_COLOR_L4 == (c))||(EPIC_COLOR_L4_SWAP == (c)))
#define EPIC_IS_GRAY_COLOR_MODE(c)      ((EPIC_COLOR_GRAY8 == ((c) & ~EPIC_COLOR_SWAP_FLAG)) || \
                                         (EPIC_COLOR_GRAY4 == ((c) & ~EPIC_COLOR_SWAP_FLAG)) || \
                                         (EPIC_COLOR_GRAY2 == ((c) & ~EPIC_COLOR_SWAP_FLAG)))
#define EPIC_IS_MASK_COLOR_MODE(c)   ((EPIC_COLOR_A8 == (c)) || (EPIC_COLOR_A4 == ((c))) || (EPIC_COLOR_A4_SWAP == ((c))))

/** @defgroup EPIC_Output_Color_Mode EPIC Output Color Mode
  * @{
  */
#define EPIC_OUTPUT_RGB565         EPIC_COLOR_RGB565                     /*!< RGB565 EPIC output color mode   */
#define EPIC_OUTPUT_ARGB8565       EPIC_COLOR_ARGB8565                   /*!< ARGB8565 EPIC output color mode   */
#define EPIC_OUTPUT_RGB888         EPIC_COLOR_RGB888                     /*!< RGB888 EPIC output color mode   */
#define EPIC_OUTPUT_ARGB8888       EPIC_COLOR_ARGB8888                   /*!< ARGB8888 EPIC output color mode   */
#define EPIC_OUTPUT_A8             EPIC_COLOR_A8                         /*!< A8 EPIC output color mode   */
#define EPIC_OUTPUT_GRAY8             EPIC_COLOR_GRAY8                         /*!< Grayscale 8bit EPIC output color mode   */
#define EPIC_OUTPUT_GRAY4             EPIC_COLOR_GRAY4                         /*!< Grayscale 4bit EPIC output color mode   */
#define EPIC_OUTPUT_GRAY2             EPIC_COLOR_GRAY2                         /*!< Grayscale 2bit EPIC output color mode   */
#define EPIC_OUTPUT_GRAY4_SWAP         EPIC_COLOR_GRAY4_SWAP                   /*!< Grayscale 4bit EPIC output color mode with swapped pixels   */
#define EPIC_OUTPUT_GRAY2_SWAP         EPIC_COLOR_GRAY2_SWAP                   /*!< Grayscale 2bit EPIC output color mode with swapped pixels   */
/**
  * @}
  */
#define EPIC_SUPPROT_OUT_FORMAT(cf)  ((EPIC_OUTPUT_RGB565 == (cf)) || (EPIC_OUTPUT_ARGB8565 == (cf)) \
                                        || (EPIC_OUTPUT_RGB888 == (cf)) || (EPIC_OUTPUT_ARGB8888 == (cf)) \
                                        || (EPIC_OUTPUT_A8 == (cf)) || (EPIC_OUTPUT_GRAY8 == (cf)) \
                                        || (EPIC_OUTPUT_GRAY4 == (cf)) || (EPIC_OUTPUT_GRAY2 == (cf)) \
                                        || (EPIC_OUTPUT_GRAY4_SWAP == (cf)) || (EPIC_OUTPUT_GRAY2_SWAP == (cf)))

#define EPIC_OUTPUT_IS_TRANSLUCENT(cf)  ((EPIC_OUTPUT_ARGB8565 == (cf)) || (EPIC_OUTPUT_ARGB8888 == (cf)) \
                                        || (EPIC_OUTPUT_A8 == (cf)))

/** @defgroup EPIC_Input_Color_Mode EPIC Input Color Mode
  * @{
  */
#define EPIC_INPUT_RGB565            EPIC_COLOR_RGB565              /*!< RGB565 input color mode */
#define EPIC_INPUT_ARGB8565          EPIC_COLOR_ARGB8565            /*!< ARGB8565 input color mode   */
#define EPIC_INPUT_RGB888            EPIC_COLOR_RGB888              /*!< RGB888 input color mode   */
#define EPIC_INPUT_ARGB8888          EPIC_COLOR_ARGB8888            /*!< ARGB8888 input color mode   */
#define EPIC_INPUT_L8                EPIC_COLOR_L8                  /*!< L8 color format */
#define EPIC_INPUT_A8                EPIC_COLOR_A8                  /*!< A8 color format */
#define EPIC_INPUT_A4                EPIC_COLOR_A4                  /*!< A4 color format */
#define EPIC_INPUT_A2                EPIC_COLOR_A2                  /*!< A2 color format */
#define EPIC_INPUT_EZIP              EPIC_COLOR_EZIP                /*!< EZIP color format */
#define EPIC_INPUT_YUV422_PACKED_YUYV   EPIC_COLOR_YUV422_PACKED_YUYV            /**< YUYV */
#define EPIC_INPUT_YUV422_PACKED_UYVY   EPIC_COLOR_YUV422_PACKED_UYVY            /**< UYUV */
#define EPIC_INPUT_YUV420_PLANAR        EPIC_COLOR_YUV420_PLANAR                 /**< iYUV */
#define EPIC_INPUT_MONO                EPIC_COLOR_MONO
#define EPIC_INPUT_JPEG              EPIC_COLOR_JPEG
#define EPIC_INPUT_L4                EPIC_COLOR_L4                  /*!< L4 color format */
#define EPIC_INPUT_L4_SWAP          EPIC_COLOR_L4_SWAP            /*!< L4 color format with swapped pixels */
#define EPIC_INPUT_A4_SWAP          EPIC_COLOR_A4_SWAP            /*!< A4 color format with swapped pixels */
#define EPIC_INPUT_A2_SWAP          EPIC_COLOR_A2_SWAP            /*!< A2 color format with swapped pixels */
#define EPIC_INPUT_RGB565_SWAP    EPIC_COLOR_RGB565_SWAP          /*!< RGB565 color format with swapped bytes */


/**
  * @}
  */

/** @defgroup EPIC_CoengStateFlag EPIC Co-engine state flag
 * @{
*/
#define EPIC_COENG_STATE_EZIP_FLAG         (0x1)
#define EPIC_COENG_STATE_JPEGD_FLAG        (0x2)
/**
 * @}
 */

#define EPIC_LAYER_OPAQUE  (0xFF)

#define EPIC_MAX_LOOKUP_TABLE_CNT        (256)    /*!< Maximum lookup table color numbers */
#define EPIC_LOOKUP_TABLE_SIZE          (EPIC_MAX_LOOKUP_TABLE_CNT*4)    /*!< In bytes */
#ifndef SF32LB55X
#if defined(SF32LB52X) || defined(SF32LB57X)
#define EPIC_LOOKUP_TABLES   1
#else
#define EPIC_LOOKUP_TABLES   2
#endif /* SF32LB52X */
#endif /* SF32LB55X */



#define EPIC_TO_INT32_COORD(INT16,FRAC16)        ((int32_t)  ((((int32_t) (INT16)&0xFFFF) << 16) | ((FRAC16)&0xFFFF)))
#define EPIC_INT32_COORD_GET_INT16(_INT32_)      ((int16_t)    ((_INT32_) >> 16))
#define EPIC_INT32_COORD_GET_FRAC16(_INT32_)      ((uint16_t)  ((_INT32_)&0xFFFF))

/*
* - 1024 or EPIC_INPUT_SCALE_NONE for no scaling
* - >1024: scale down
* - <1024  scale up
* - 2048   half size
* - 512    double size
*/
#define EPIC_INPUT_SCALE_FRAC_SIZE 10
#define EPIC_INPUT_SCALE_NONE 1024

/*
 EPIC input/output maximum coordinates
*/
#if defined(SF32LB52X)||defined(SF32LB57X)
#define EPIC_COORDINATES_MAX 505
#else /* SF32LB52X */
#define EPIC_COORDINATES_MAX 1010
#endif /* !SF32LB52X */
/**
  * @brief EPIC Init structure definition
  */
typedef struct
{
    uint32_t            reserved;
} EPIC_InitTypeDef;

/**
  * @brief  HAL EPIC State structures definition
  */
typedef enum
{
    HAL_EPIC_STATE_RESET             = 0x00U,    /*!< EPIC not yet initialized or disabled       */
    HAL_EPIC_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
    HAL_EPIC_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing              */
    HAL_EPIC_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                               */
    HAL_EPIC_STATE_ERROR             = 0x04U,    /*!< EPIC state error                           */
    HAL_EPIC_STATE_SUSPEND           = 0x05U     /*!< EPIC process is suspended                  */
} HAL_EPIC_StateTypeDef;

/**
  * @brief  The blending behaviors of EPIC A4/A8 layer
  */
typedef enum
{
    ALPHA_BLEND_INVALID            = 0x00U,    /*!< Invalid                                               */
    ALPHA_BLEND_RGBCOLOR           = 0x00U,    /*!< Append a fixed RGB color to be an ARGB layer          */
    ALPHA_BLEND_MASK               = 0x01U,    /*!< Alpha mask - Multiply with background's alpha         */
    ALPHA_BLEND_OVERWRITE          = 0x02U,    /*!< Replace the background's alpha                        */
} Alpha_BlendTypeDef;

typedef enum
{
    EPIC_DITHER_LEVEL_DISABLE = 0,   /**< Dither disable */
    EPIC_DITHER_LEVEL_MIN = 1,   /**< Dither minimal level*/
    EPIC_DITHER_LEVEL_MIDDLE = 2,   /**< Dither middle level */
    EPIC_DITHER_LEVEL_MAX = 3,   /**< Dither maximum level*/
} EPIC_DitherLevelTypeDef;

typedef struct
{
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;
} EPIC_AreaTypeDef;

typedef struct
{
    int16_t x;
    int16_t y;
} EPIC_PointTypeDef;

#define EPIC_SIN_COS_FRAC_BIT   (15)

typedef struct
{
    /*
      !!IMPORTANT!!
       Part1~3 are mutually exclusive, only one part is used each time
    */

    /*****************
     * Part1 for basic 2D transform
     * *********************/
    /** angle in 0.1 degree */
    int16_t angle;
    /** flip left to right */
    int8_t h_mirror;
    /** flip up to down */
    int8_t v_mirror;
    /** x coordinate of pivot relative to source image topleft pixel,
        valid if angle!=0 or scale_x(scale_y) != EPIC_INPUT_SCALE_NONE */
    int16_t pivot_x;
    /** y coordinate of pivot relative to source image topleft pixel,
        valid if angle!=0 or scale_x(scale_y) != EPIC_INPUT_SCALE_NONE */
    int16_t pivot_y;
    /** scaling factor in x direction
     *
     *   scale_x = factor*EPIC_INPUT_SCALE_NONE, EPIC_INPUT_SCALE_NONE means no scaling,
     *   scale_x < EPIC_INPUT_SCALE_NONE means scaling up, scale_x > EPIC_INPUT_SCALE_NONE means scaling down
     */
    uint32_t scale_x;
    /** scaling factor in y direction
     *
     *   scale_y = factor*EPIC_INPUT_SCALE_NONE, EPIC_INPUT_SCALE_NONE means no scaling,
     *   scale_y < EPIC_INPUT_SCALE_NONE means scaling up, scale_y > EPIC_INPUT_SCALE_NONE means scaling down
     */
    uint32_t scale_y;

    /***************
     * Part2 Reserved params for epic_adv APIs
     * *****************/
    uint16_t type; /*0: disable, 1-type1 2-type2*/
    int16_t angle_adv; /** angle in 0.1 degree */
    int16_t pivot_z;
    int16_t z_offset; /*src z offset*/
    int16_t vp_x_offset; /*view x offset*/
    int16_t vp_y_offset; /*view y offset*/
    int16_t dst_z_offset; /*dst z offset*/


    /*******************
     * Part3 for matrix transfrom only
     * *********************/
    sifli_matrix_3x3_t *trans_matrix;
} EPIC_TransformCfgTypeDef;

typedef struct
{
    uint8_t *y_buf;  /**< Y buf in planar/semi-planar mode, or YUV buf in packed mode */
    uint8_t *u_buf;  /**< U(Cb) buf in planar mode, or UV buf in semi-planar mode     */
    uint8_t *v_buf;  /**< V(Cr) buf in planar mode                                    */
} EPIC_YUVCfgTypeDef;

/**
  * @brief  HAL EPIC Blend layer structures definition
  *
  *
  *
  *              |<-------                total_width                               ----->|
  *
  *              +------------------------------------------------------------------------+
  *              |Image buffer                                                            |
  *              |                                                                        |
  *              |                                                                        |
  *              |                         |<-------        width     ----->|             |
  *              |  (x_offset,y_offset)                                                   |
  *              |          |------------->+--------------------------------+   ---       |
  *              |        data             |EPIC layer buffer               |    ^        |
  *              |                         |                                |    |        |
  *              |                         |                                |    |        |
  *              |                         |                                |             |
  *              |                         |                                |  height     |
  *              |                         |                                |             |
  *              |                         |                                |    |        |
  *              |                         |                                |    |        |
  *              |                         |                                |    v        |
  *              |                         +--------------------------------+   ---       |
  *              |                                                                        |
  *              |                                                                        |
  *              |                                                                        |
  *              |                                                                        |
  *              +------------------------------------------------------------------------+
  *
  *       NOTE:  x_offset&y_offset is NOT relative Image buffer's top-left
  */
typedef struct
{
    uint8_t *data;            /**< data, color format is specified by #color_mode,
                                   point to the pixel data corresponding to pixel(#x_offset,#y_offset)
                                   Set to NULL if this layer is NOT actived*/
    uint32_t color_mode;      /**< color mode, refer to #EPIC_COLOR_RGB565,
                                   if ezip is enabled, it must be ARGB8888 for data with alpha channel
                                   or RGB888 for data without alpha channel */
    uint16_t width;           /**< layer width to be processed */
    uint16_t height;          /**< layer heigth to be processed  */
    int16_t x_offset;         /**< topleft x coordinate of the layer */
    int16_t y_offset;         /**< topleft y coordinate of the layer */
    uint16_t x_offset_frac;   /**< x_offset 's fractional, has same sign as x_offset */
    uint16_t y_offset_frac;   /**< y_offset 's fractional, has same sign as y_offset */
    /** layer total width,
     *
     * e.g. #data points to rectangle area (0,0)~(99,99), need to blend area (0,0)~(9,9),
     *   then width can be set to 10 while total_width is set to 100
     */
    uint16_t total_width;
    /** Enabled color in RGB888,
     *
     * 1. used as filter color by RGBxxx or ARGBxxx layer
     * 2. used as background color by 'A4/A8' color_mode layer
     * 3. used as background color by output layer
     */
    bool color_en;
    /** Red, if color_filter_en is true, it's filter color for rotation/scaling layer or background color for output layer */
    uint8_t color_r;
    /** Green */
    uint8_t color_g;
    /** Blue */
    uint8_t color_b;

    Alpha_BlendTypeDef ax_mode; /**< 'A4/A8' color mode data blend mode*/

    uint8_t *lookup_table;      /**< Lookup table for 'L8' mode data*/
    /** data size in bytes, only valid if data is in ezip format and saved in NAND */
    uint32_t data_size;

    EPIC_YUVCfgTypeDef  yuv;  /**< YUV data*/

    uint16_t lookup_table_size;  /**< Lookup table color numbers*/

    uint32_t dither_level : 2;  /**< Dither level for this layer, refer to #EPIC_DITHER_LEVEL_XXX */
    uint32_t reserved  : 30;   /**< Reserved */
    /** 4x5 color matrix (4 rows, 5 columns) :
     *
     *  | Ra    Rb    Rc   Rd   Re|
     *  | Ga    Gb    Gc   Gd   Ge|
     *  | Ba    Bb    Bc   Bd   Be|
     *  | Aa    Ab    Ac   Ad   Ae|
     *
     *  Color transform formula:
     *  R' = Ra*R + Rb*G + Rc*B + Rd*A + Re
     *  G' = Ga*R + Gb*G + Gc*B + Gd*A + Ge
     *  B' = Ba*R + Bb*G + Bc*B + Bd*A + Be
     *  A' = Aa*R + Ab*G + Ac*B + Ad*A + Ae
     *
     *  Only valid for input layer
    */
    float *color_matrix;
} EPIC_BlendingDataType;

typedef struct
{
    uint8_t *data;            /**< data, color format is specified by #color_mode,
                                   point to the pixel data corresponding to pixel(#x_offset,#y_offset)
                                   Set to NULL if this layer is NOT actived*/
    uint32_t color_mode;      /**< color mode, refer to #EPIC_COLOR_RGB565,
                                   if ezip is enabled, it must be ARGB8888 for data with alpha channel
                                   or RGB888 for data without alpha channel */
    uint16_t width;           /**< layer width to be processed */
    uint16_t height;          /**< layer heigth to be processed  */
    int16_t x_offset;         /**< topleft x coordinate of the layer */
    int16_t y_offset;         /**< topleft y coordinate of the layer */
    uint16_t x_offset_frac;   /**< x_offset 's fractional, has same sign as x_offset */
    uint16_t y_offset_frac;   /**< y_offset 's fractional, has same sign as y_offset */
    /** layer total width,
     *
     * e.g. #data points to rectangle area (0,0)~(99,99), need to blend area (0,0)~(9,9),
     *   then width can be set to 10 while total_width is set to 100
     */
    uint16_t total_width;
    /** Enabled color in RGB888,
     *
     * 1. used as filter color by RGBxxx or ARGBxxx layer
     * 2. used as background color by 'A4/A8' color_mode layer
     * 3. used as background color by output layer
     */
    bool color_en;
    /** Red */
    uint8_t color_r;
    /** Green */
    uint8_t color_g;
    /** Blue */
    uint8_t color_b;

    Alpha_BlendTypeDef ax_mode; /**< 'A4/A8' color mode data blend mode*/

    uint8_t *lookup_table;      /**< Lookup table for 'L8' mode data*/
    /** data size in bytes, only valid if data is in ezip format and saved in NAND */
    uint32_t data_size;

    EPIC_YUVCfgTypeDef  yuv;  /**< YUV data*/

    uint16_t lookup_table_size;  /**< Lookup table color numbers, maximum is 'EPIC_MAX_LOOKUP_TABLE_CNT' */

    uint32_t dither_level : 2;  /**< Dither level for this layer, refer to #EPIC_DITHER_LEVEL_XXX */
    uint32_t reserved  : 30;   /**< Reserved */
    /** 4x5 color matrix (4 rows, 5 columns) :
     *
     *  | Ra    Rb    Rc   Rd   Re|
     *  | Ga    Gb    Gc   Gd   Ge|
     *  | Ba    Bb    Bc   Bd   Be|
     *  | Aa    Ab    Ac   Ad   Ae|
     *
     *  Color transform formula:
     *  R' = Ra*R + Rb*G + Rc*B + Rd*A + Re
     *  G' = Ga*R + Gb*G + Gc*B + Gd*A + Ge
     *  B' = Ba*R + Bb*G + Bc*B + Bd*A + Be
     *  A' = Aa*R + Ab*G + Ac*B + Ad*A + Ae
     *
     *  Only valid for input layer
    */
    float *color_matrix;
    /****** Keep above members as same as struct 'EPIC_BlendingDataType'  *************/

    uint8_t alpha;              /**< Layer global alpha*/

    EPIC_TransformCfgTypeDef transform_cfg;
} EPIC_LayerConfigTypeDef;

typedef struct
{
    uint8_t *start;           /**< filling start address, color format is specified by #color_mode
                                   it's the address of the first pixel which should be filled by the specifed color */
    uint32_t color_mode;      /**< color mode, refer to #EPIC_COLOR_RGB565 */
    uint16_t width;           /**< width of rectangle area to be filled */
    uint16_t height;          /**< height of rectangle area to be filled */
    /** buffer total width,
     *
     * e.g. buffer resolution is 100*200, 100 pixels each line, area (10,20)~(49, 69) needs to be filled with color red,
     *      then #width is 40 and 3heigth is 50 while 3total_width is 100, after filling 40 pixels in one line,
     *      remaining 60 pixels position would be skipped and switch to start pixel of next line,
     *      #start should point to the position of pixel (10,20)
     */
    uint16_t total_width;
    /** Red part of filling color, RGB888  */
    uint8_t color_r;
    /** Green part of filling color, RGB888 */
    uint8_t color_g;
    /** Blue part of filling color, RGB888 */
    uint8_t color_b;
    /** alpha value, range: 0~255 */
    uint8_t alpha;
} EPIC_FillingCfgTypeDef;

typedef union
{
    struct
    {
        uint8_t color_b;
        uint8_t color_g;
        uint8_t color_r;
        uint8_t alpha;
    } ch;
    uint32_t full;
} EPIC_ColorDef;

typedef struct
{
    uint8_t *start;           /**< filling start address, color format is specified by #color_mode
                                   it's the address of the first pixel which should be filled by the specifed color */
    uint32_t color_mode;      /**< color mode, refer to #EPIC_COLOR_RGB565 */
    uint16_t width;           /**< width of rectangle area to be filled */
    uint16_t height;          /**< height of rectangle area to be filled */
    /** buffer total width,
     *
     * e.g. buffer resolution is 100*200, 100 pixels each line, area (10,20)~(49, 69) needs to be filled with color red,
     *      then #width is 40 and 3heigth is 50 while 3total_width is 100, after filling 40 pixels in one line,
     *      remaining 60 pixels position would be skipped and switch to start pixel of next line,
     *      #start should point to the position of pixel (10,20)
     */
    uint16_t total_width;

    /** color of rect area 4 pionts.
     * [0][0] - top left
     * [0][1] - top right
     * [1][0] - bottom left
     * [1][1] - bottom right
     */
    EPIC_ColorDef color[2][2];
} EPIC_GradCfgTypeDef;


typedef int16_t (*EPIC_TransPath)(int16_t cur, EPIC_LayerConfigTypeDef *pTransLayer, void *user_data);

#ifdef EPIC_DEBUG
typedef enum
{
    EPIC_OP_ROTATION,
    EPIC_OP_BLENDING,
    EPIC_OP_FILLING,
    EPIC_OP_FillGrad,

    EPIC_OP_BLENDING_EX,
    EPIC_OP_COPY,
    EPIC_OP_CONT_BLENDING,
    EPIC_OP_CONT_BLENDING_REPEAT,
    EPIC_OP_TRANSFORM,
} EPIC_OpTypeDef;

typedef union
{
    struct
    {
        EPIC_TransformCfgTypeDef rot_cfg;
        EPIC_BlendingDataType fg;
        EPIC_BlendingDataType bg;
        EPIC_BlendingDataType dst;
        uint8_t alpha;
    } rot_param;
    struct
    {
        EPIC_BlendingDataType fg;
        EPIC_BlendingDataType bg;
        EPIC_BlendingDataType dst;
        uint8_t alpha;
    } blend_param;
    EPIC_FillingCfgTypeDef fill_param;
    EPIC_GradCfgTypeDef grad_param;
    struct
    {
        uint8_t input_layer_num;
        EPIC_LayerConfigTypeDef input_layer[MAX_EPIC_LAYER];
        EPIC_LayerConfigTypeDef output_layer;
    } blend_ex_param;
    struct
    {
        EPIC_BlendingDataType src;
        EPIC_BlendingDataType dst;
    } copy_param;
    struct
    {
        EPIC_LayerConfigTypeDef input_layer;
        EPIC_LayerConfigTypeDef mask_layer;
        EPIC_LayerConfigTypeDef output_layer;
    } cont_blend_param;
    struct
    {
        EPIC_LayerConfigTypeDef input_layer;
        EPIC_LayerConfigTypeDef mask_layer;
        EPIC_LayerConfigTypeDef output_layer;
    } cont_blend_repeat_param;
    struct
    {
        uint8_t input_layer_num;
        EPIC_LayerConfigTypeDef input_layer[MAX_EPIC_LAYER];
        EPIC_LayerConfigTypeDef output_layer;
        EPIC_TransPath hor_path;
        EPIC_TransPath ver_path;
        void *user_data;
    } transform_param;
} EPIC_OpParamTypeDef;

typedef struct
{
    EPIC_OpTypeDef op;
    uint32_t start_time; //Gtimer
    uint32_t end_time;
    EPIC_OpParamTypeDef param;
} EPIC_OpHistItemTypeDef;

#define EPIC_OP_HIST_DEPTH  (4)
typedef struct
{
    uint8_t idx;
    EPIC_OpHistItemTypeDef hist[EPIC_OP_HIST_DEPTH];
} EPIC_OpHistTypeDef;
#endif

typedef enum
{
    EPIC_API_NORMAL = 0,
    EPIC_API_CONTINUE,   /*!< Continues mode */
    EPIC_API_TRANSFORM,
} EPIC_ApiEnumType;

typedef struct
{
    uint8_t epic_layer[3];            /*!< Continues mode used EPIC layer,         0 - bg,  1 - fg,  2- mask*/
    uint32_t color_format[3];         /*!< layer color foramt,         0 - bg,  1 - fg,  2- mask*/
    uint32_t color_depth[3];          /*!< layer color depth,          0 - bg,  1 - fg,  2- mask*/
} EPIC_ContModeTypeDef;

typedef struct
{
    EPIC_TransPath hor_path;
    EPIC_TransPath ver_path;
    void *user_data;
} EPIC_TransModeTypeDef;


/** EPIC handle structure definition */
typedef struct __EPIC_HandleTypeDef
{
    EPIC_TypeDef               *Instance;                                        /*!< EPIC register base address. */
#ifdef EPIC_SUPPORT_L8
    __IO uint32_t              *LTab[EPIC_LOOKUP_TABLES];
#endif /* EPIC_SUPPORT_L8 */
#ifdef HAL_EZIP_MODULE_ENABLED
    EZIP_HandleTypeDef        *hezip;                                            /**< EZIP handle                 */
#endif /* HAL_EZIP_MODULE_ENABLED */
#ifdef HAL_JPEGD_MODULE_ENABLED
    JPEGD_HandleTypeDef       *hjpegd;                                           /**< JPEGD Handle  */
    uint32_t                  jpegd_work_buf_size;
    uint8_t                   *jpegd_work_buf;
#endif /* HAL_JPEGD_MODULE_ENABLED */
    EPIC_InitTypeDef           Init;                                             /*!< EPIC init parameters.       */
    void (* XferCpltCallback)(struct __EPIC_HandleTypeDef *epic);                /*!< EPIC processing complete callback. */
    __IO HAL_EPIC_StateTypeDef State;                                            /*!< EPIC state.                        */
    __IO uint32_t              ErrorCode;                                       /*!< EPIC error code.                   */
    void (*IntXferCpltCallback)(struct __EPIC_HandleTypeDef *epic);              /*!< Internal EPIC processing complete callback. No need to set by user */

    EPIC_TypeDef               *RamInstance;                                     /*!< EPIC register on RAM. */
    EPIC_TypeDef               *HwInstance;                                      /*!< Backup of EPIC register base address. */
#ifdef EPIC_SUPPORT_L8
    uint8_t              *RamLTab[EPIC_LOOKUP_TABLES];
    uint16_t           RamLTabSize[EPIC_LOOKUP_TABLES];
#endif /* EPIC_SUPPORT_L8 */
    uint8_t            RamInstance_used;

    uint32_t   start_tick;
    uint32_t   end_tick;

    uint32_t    reserved : 32;          /* Reserved bits */

    EPIC_ApiEnumType  api_type;
    union
    {
        EPIC_ContModeTypeDef cont_cfg;                                            /*!< Continues mode configurations*/
        EPIC_TransModeTypeDef trans_cfg;
    } api_cfg;
#ifdef EPIC_DEBUG
    EPIC_OpHistTypeDef *op_hist;
    EPIC_OpHistItemTypeDef *last_hist_item;
#endif
    void *user_data;                                                             /*!< user data */
    __IO uint32_t PerfCnt;                                                       /*!< EPIC total running cycle counter */
    __IO uint32_t WaitCnt;
    __IO uint32_t HalCnt;
    __IO uint32_t coeng_state;                                                   /*!< Co-engine running state, @ref EPIC_CoengStateFlag */
} EPIC_HandleTypeDef;

typedef void (*EPIC_CpltCallback)(EPIC_HandleTypeDef *epic);

/* Exported constants --------------------------------------------------------*/


/**
 * @brief  EPIC Init
 *
 * @param[in]  epic EPIC handle
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EPIC_Init(EPIC_HandleTypeDef *epic);

/**
 * @brief  Init an epic blend struct to default vaules
 *
 * @param[layer]  EPIC layer
 * @retval None
 */
void HAL_EPIC_BlendDataInit(EPIC_BlendingDataType *layer);

/**
 * @brief  Start blending in polling mode
 *
 * @param[in]  epic EPIC handle
 * @param[in]  fg foreground layer configuration
 * @param[in]  bg background layer configuration
 * @param[in]  dst output layer configuration
 * @param[in]  alpha alpha value
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EPIC_BlendStart(EPIC_HandleTypeDef *epic, EPIC_BlendingDataType *fg,
                                      EPIC_BlendingDataType *bg, EPIC_BlendingDataType *dst, uint8_t alpha);

/**
 * @brief  Start blending in interrupt mode
 *
 * @param[in]  epic EPIC handle
 * @param[in]  fg foreground layer configuration
 * @param[in]  bg background layer configuration
 * @param[in]  dst output layer configuration
 * @param[in]  alpha alpha value
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EPIC_BlendStart_IT(EPIC_HandleTypeDef *epic, EPIC_BlendingDataType *fg,
        EPIC_BlendingDataType *bg, EPIC_BlendingDataType *dst, uint8_t alpha);


/**
* @brief  Init an epic rotate cfg struct to default vaules
*
* @param rot_cfg EPIC rotation configuration parameter
* @retval None
*/
void HAL_EPIC_RotDataInit(EPIC_TransformCfgTypeDef *rot_cfg);

/**
 * @brief  Rotate and scale src1 based on the image center, blend with bg and output to dst in polling mode
 *
 * @param[in]  epic EPIC handle
 * @param[in]  rot_cfg rotation configuration
 * @param[in]  fg configuration of foreground layer that's to be rotated
 * @param[in]  bg background layer configuration
 * @param[in]  dst output layer configuration
 * @param[in]  alpha alpha value for the fg.
 *
 * @retval HAL status
 *
 * @note fields in fg, bg and dst would be modified
 */
HAL_StatusTypeDef HAL_EPIC_Rotate(EPIC_HandleTypeDef *epic, EPIC_TransformCfgTypeDef *rot_cfg,
                                  EPIC_BlendingDataType *fg,
                                  EPIC_BlendingDataType *bg, EPIC_BlendingDataType *dst, uint8_t alpha);

/**
* @brief  Rotate and scale src1 based on the image center, blend with bg and output to dst in interrupt mode
*
* @param[in]  epic EPIC handle
* @param[in]  rot_cfg rotation configuration
* @param[in]  fg configuration of foreground layer that's to be rotated
* @param[in]  bg background layer configuration
* @param[in]  dst output layer configuration
* @param[in]  alpha alpha value
*
* @retval HAL status
*
* @note fields in fg, bg and dst would be modified
*
*/
HAL_StatusTypeDef HAL_EPIC_Rotate_IT(EPIC_HandleTypeDef *epic, EPIC_TransformCfgTypeDef *rot_cfg,
                                     EPIC_BlendingDataType *fg,
                                     EPIC_BlendingDataType *bg, EPIC_BlendingDataType *dst, uint8_t alpha);

/**
* @brief  Init an epic fill cfg struct to default vaules
*
* @param param  EPIC fill configuration parameter
* @retval None
*/
void HAL_EPIC_FillDataInit(EPIC_FillingCfgTypeDef *param);

/**
 * @brief  Fill the region with given color in interrupt mode
 *
 *  if alpha is used, filling color would be blended with original color in the buffer
 *  output_color = alpha * filling_color + (255 - alpha) * original_color
 *
 * @param[in]  epic EPIC handle
 * @param[in]  param filling region configuration
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EPIC_FillStart_IT(EPIC_HandleTypeDef *epic, EPIC_FillingCfgTypeDef *param);

/**
 * @brief  Fill the region with given color in polling mode
 *
 *  if alpha is used, filling color would be blended with original color in the buffer
 *  output_color = alpha * filling_color + (255 - alpha) * original_color
 *
 * @param[in]  epic EPIC handle
 * @param[in]  param filling region configuration
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EPIC_FillStart(EPIC_HandleTypeDef *epic, EPIC_FillingCfgTypeDef *param);


/**
 * @brief  Copy src data buffer to dst
 *
 *
 * @param[in]  epic EPIC handle
 * @param[in]  src src layer configuration
 * @param[in]  dst dst layer configuration
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EPIC_Copy_IT(EPIC_HandleTypeDef *epic, EPIC_BlendingDataType *src, EPIC_BlendingDataType *dst);

/**
* @brief  Init an epic fill grad cfg struct to default vaules
*
* @param param EPIC fill grad configuration parameter
* @retval None
*/
void HAL_EPIC_FillGradDataInit(EPIC_GradCfgTypeDef *param);

HAL_StatusTypeDef HAL_EPIC_FillGrad_IT(EPIC_HandleTypeDef *epic, EPIC_GradCfgTypeDef *param);
HAL_StatusTypeDef HAL_EPIC_FillGrad(EPIC_HandleTypeDef *epic, EPIC_GradCfgTypeDef *param);

/**
 * @brief  Init an epic layer config struct to default values
 *
 * @param layer EPIC layer
 * @retval None
 */
void HAL_EPIC_LayerConfigInit(EPIC_LayerConfigTypeDef *layer);


/**  input_layer[0]: bottom layer, input_layer[1] is on top of input_layer[0]
 *   input_layer[2] is on top of input_layer[1], ...
 *
 *   and mask layer will apply to the layer(1 layer only) under it.
 */
HAL_StatusTypeDef HAL_EPIC_BlendStartEx(EPIC_HandleTypeDef *epic,
                                        EPIC_LayerConfigTypeDef *input_layer,
                                        uint8_t input_layer_num,
                                        EPIC_LayerConfigTypeDef *output_layer);

/**  input_layer[0]: bottom layer, input_layer[1] is on top of input_layer[0]
 *   input_layer[2] is on top of input_layer[1], ...
 *
 *   and mask layer will apply to the layer(1 layer only) under it.
 */
HAL_StatusTypeDef HAL_EPIC_BlendStartEx_IT(EPIC_HandleTypeDef *epic,
        EPIC_LayerConfigTypeDef *input_layer,
        uint8_t input_layer_num,
        EPIC_LayerConfigTypeDef *output_layer);



/**
 * @brief  EPIC IRQ Handler
 *
 * @param  epic EPIC handle
 *
 * @retval None
 */
void HAL_EPIC_IRQHandler(EPIC_HandleTypeDef *epic);

HAL_StatusTypeDef HAL_EPIC_ContBlendStart(EPIC_HandleTypeDef *hepic,
        EPIC_LayerConfigTypeDef *input_layer,
        EPIC_LayerConfigTypeDef *mask_layer,
        EPIC_LayerConfigTypeDef *output_layer);


HAL_StatusTypeDef HAL_EPIC_ContBlendRepeat(EPIC_HandleTypeDef *hepic,
        EPIC_LayerConfigTypeDef *cont_layer,
        EPIC_LayerConfigTypeDef *mask_layer,
        EPIC_LayerConfigTypeDef *output_layer);
HAL_StatusTypeDef HAL_EPIC_ContBlendStop(EPIC_HandleTypeDef *hepic);


HAL_StatusTypeDef HAL_EPIC_TransStart(EPIC_HandleTypeDef *hepic,
                                      EPIC_LayerConfigTypeDef *input_layer,
                                      uint8_t input_layer_num,
                                      EPIC_LayerConfigTypeDef *output_layer,
                                      EPIC_TransPath hor_path,
                                      EPIC_TransPath ver_path,
                                      void *user_data);

HAL_StatusTypeDef HAL_EPIC_Adv(EPIC_HandleTypeDef *hepic,
                               EPIC_LayerConfigTypeDef *input_layers,
                               uint8_t input_layer_num,
                               EPIC_LayerConfigTypeDef *output_layer);

bool HAL_EPIC_IsHWBusy(EPIC_HandleTypeDef *hepic);


bool HAL_EPIC_AreaIsIn(const EPIC_AreaTypeDef *ain_p, const EPIC_AreaTypeDef *aholder_p);
bool HAL_EPIC_AreaIntersect(EPIC_AreaTypeDef *res_p, const EPIC_AreaTypeDef *a1_p, const EPIC_AreaTypeDef *a2_p);
void HAL_EPIC_AreaMove(EPIC_AreaTypeDef *area, int16_t x, int16_t y);
static inline int16_t HAL_EPIC_AreaWidth(const EPIC_AreaTypeDef *area_p)
{
    return (int16_t)(area_p->x1 - area_p->x0 + 1);
}

static inline int16_t HAL_EPIC_AreaHeight(const EPIC_AreaTypeDef *area_p)
{
    return (int16_t)(area_p->y1 - area_p->y0 + 1);
}

static inline void HAL_EPIC_AreaCopy(EPIC_AreaTypeDef *area_dst_p, const EPIC_AreaTypeDef *area_src_p)
{
    area_dst_p->x0 = area_src_p->x0;
    area_dst_p->x1 = area_src_p->x1;
    area_dst_p->y0 = area_src_p->y0;
    area_dst_p->y1 = area_src_p->y1;
}

static inline bool HAL_EPIC_AreaIsValid(const EPIC_AreaTypeDef *area_p)
{
    return ((area_p->x0 <= area_p->x1) && (area_p->y0 <= area_p->y1));
}
void HAL_EPIC_AreaJoin(EPIC_AreaTypeDef *a_res_p, const EPIC_AreaTypeDef *a1_p, const EPIC_AreaTypeDef *a2_p);


void HAL_EPIC_LayerSetDataOffset(EPIC_BlendingDataType *layer, int16_t x, int16_t y);

HAL_StatusTypeDef HAL_EPIC_BlendFastStart_Init(EPIC_HandleTypeDef *hepic_s);
HAL_StatusTypeDef HAL_EPIC_BlendFastStart_IT(EPIC_HandleTypeDef *hepic, EPIC_HandleTypeDef *hepic_s);
uint32_t HAL_EPIC_GetColorDepth(uint32_t color_mode);
#define EPIC_SIN_COS_FRAC_BIT   (15)
#define EPIC_TRIGO_SHIFT EPIC_SIN_COS_FRAC_BIT
int16_t EPIC_TrigoSin(int16_t angle);
static inline int32_t EPIC_TrigoCos(int16_t angle)
{
    return EPIC_TrigoSin(angle + 90);
}
void EPIC_TrigoSinCosP1(int16_t angle, int16_t *sin_val, int16_t *cos_val);

void EPIC_GetTransformedArea(EPIC_AreaTypeDef *output, uint16_t w, uint16_t h, int16_t angle,
                             uint32_t scale_x, uint32_t scale_y,
                             const EPIC_PointTypeDef *pivot);
#ifdef EPIC_SUPPORT_TRANS_MATRIX
void EPIC_GetMatrixTransfromedArea(sifli_matrix_3x3_t *p_matrix, EPIC_AreaTypeDef *p_area);
#endif
/* -------------------------------
    EPIC Transfer Layer(EPICTL) APIs
 ------------------------------- */
#ifdef HAL_EPICTL_ENABLED
#include "bf0_hal_epictl.h"
#endif /* HAL_EPICTL_ENABLED */
/**
  * @}
  */

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /* BF0_HAL_EPIC_H */