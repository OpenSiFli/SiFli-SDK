/**
  ******************************************************************************
  * @file   bf0_hal_dsi.c
  * @author Sifli software development team
  * @brief   DSI HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the DSI peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State and Errors functions
  ******************************************************************************
*/
/**
 *
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

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

#ifdef HAL_DSI_MODULE_ENABLED
#include "bf0_hal_dsi.h"

/** @addtogroup DSI
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @addtogroup DSI_Private_Constants
  * @{
  */
#define DSI_TIMEOUT_VALUE ((uint32_t)1000U)  /* 1s */

#define DSI_ERROR_ACK_MASK (DSI_HOST_INT_MASK1_SOT | DSI_HOST_INT_MASK1_SOT_SYNC | DSI_HOST_INT_MASK1_EOT_SYNC | DSI_HOST_INT_MASK1_ESC_ENTRY_CMD | \
                            DSI_HOST_INT_MASK1_LPTX_SYNC | DSI_HOST_INT_MASK1_PERI_TIMEOUT | DSI_HOST_INT_MASK1_FALSE_CTRL | DSI_HOST_INT_MASK1_RSVD_INFO0 | \
                            DSI_HOST_INT_MASK1_SINGLE_ECC | DSI_HOST_INT_MASK1_MULTI_ECC | DSI_HOST_INT_MASK1_LONG_PKT_CHKSUM | DSI_HOST_INT_MASK1_UNDEF_DTYPE | \
                            DSI_HOST_INT_MASK1_INVALID_VCID | DSI_HOST_INT_MASK1_INVALID_TRANS_LEN | DSI_HOST_INT_MASK1_RSVD_INFO1 | DSI_HOST_INT_MASK1_DSI_PROTOCOL_VIO)
#define DSI_ERROR_PHY_MASK (DSI_HOST_INT_MASK1_D0_ERRESC | DSI_HOST_INT_MASK1_D0_ERRSYNCESC | DSI_HOST_INT_MASK1_D0_ERRCTRL | DSI_HOST_INT_MASK1_D0_LP0_CONT | DSI_HOST_INT_MASK1_D0_LP1_CONT)
#define DSI_ERROR_TX_MASK  DSI_HOST_INT_MASK2_HSTX_TIMEOUT
#define DSI_ERROR_RX_MASK  DSI_HOST_INT_MASK2_LPRX_TIMEOUT
#define DSI_ERROR_ECC_MASK (DSI_HOST_INT_MASK2_ECC_SINGLE | DSI_HOST_INT_MASK2_ECC_MULTI)
#define DSI_ERROR_CRC_MASK DSI_HOST_INT_MASK2_CRC
#define DSI_ERROR_PSE_MASK DSI_HOST_INT_MASK2_PACKET_SIZE
#define DSI_ERROR_EOT_MASK DSI_HOST_INT_MASK2_EOTP
#define DSI_ERROR_OVF_MASK (DSI_HOST_INT_MASK2_DBI_CMD_WR|DSI_HOST_INT_MASK2_DBI_PAYLOAD_WR|DSI_HOST_INT_MASK2_DBI_PAYLOAD_RD|DSI_HOST_INT_MASK2_DBI_PAYLOAD_RCV|DSI_HOST_INT_MASK2_DBI_INVALID_CMD)


#define DSI_ERROR_GEN_MASK (DSI_HOST_INT_MASK2_COM_CMD_WR | DSI_HOST_INT_MASK2_COM_PAYLOAD_WR | DSI_HOST_INT_MASK2_COM_PAYLOAD_SEND | DSI_HOST_INT_MASK2_COM_PAYLOAD_RD | DSI_HOST_INT_MASK2_COM_PAYLOAD_RCV)



#define DSI_LOG_D(...)    //HAL_DBG_printf(__VA_ARGS__)
#define DSI_LOG_E(...)   HAL_DBG_printf(__VA_ARGS__)

#define SHOW_BTA_LOG //For debug purpose

/**
  * @}
  */

typedef struct Freq2DLLMap
{
    uint32_t freq;
    uint8_t  in_div2;
    uint16_t stg;
    uint8_t  out_div2;

} *Freq2DLLMap_t;

#ifdef SF32LB55X
static const struct Freq2DLLMap DSI_Freq_DLL_map[] =
{
    {DSI_FREQ_96Mbps,     1,   7,   0},
    {DSI_FREQ_144Mbps,    1,   6,   0},
    {DSI_FREQ_168Mbps,    1,   5,   0},
    {DSI_FREQ_192Mbps,    1,   4,   0},
    {DSI_FREQ_240Mbps,    1,   3,   0},
    {DSI_FREQ_288Mbps,    1,   2,   0},
    {DSI_FREQ_336Mbps,    1,   1,   0},
    {DSI_FREQ_384Mbps,    1,   0,   0},
    {DSI_FREQ_480Mbps,    0,   3,   0},
    {DSI_FREQ_528Mbps,    0,   10,  0}
};

#else

static const struct Freq2DLLMap DSI_Freq_DLL_map[] =
{

    {DSI_FREQ_96Mbps,     1,   3,   0},
    {DSI_FREQ_144Mbps,    1,   5,   0},
    {DSI_FREQ_168Mbps,    1,   6,   0},
    {DSI_FREQ_192Mbps,    1,   7,   0},
    {DSI_FREQ_240Mbps,    1,   9,   0},
    {DSI_FREQ_288Mbps,    1,   11,  0},
    {DSI_FREQ_336Mbps,    1,   13,  0},
    {DSI_FREQ_384Mbps,    1,   15,  0},
    {DSI_FREQ_480Mbps,    0,   9,   0},
    {DSI_FREQ_528Mbps,    0,   10,  0}
};

#endif /* SF32LB55X */

typedef struct Freq2PhyCfgMap
{
    uint32_t freq;
    uint32_t ck_phase_sel; /*CK_LANE_CFG0*/
    uint32_t ck_lpx_cnt;
    uint32_t ck_prep_cnt;
    uint32_t ck_zero_cnt;

    uint32_t ck_pre_cnt; /*CK_LANE_CFG1*/
    uint32_t ck_post_cnt;
    uint32_t ck_trail_cnt;
    uint32_t ck_hs_exit_cnt;

    uint32_t ck_wakeup_cnt; /*CK_LANE_CFG2*/

    uint32_t dx_lpx_cnt;   /*D_LANE0_CFG0*/
    uint32_t dx_hs_prep_cnt;
    uint32_t dx_hs_zero_cnt;
    uint32_t dx_hs_trail_cnt;


    uint32_t dx_tasure_cnt;  /*D_LANE0_CFG1*/
    uint32_t dx_taget_cnt;
    uint32_t dx_tago_cnt;
    uint32_t dx_wakeup_cnt;
} *Freq2PhyCfgMap_t;

static const struct Freq2PhyCfgMap DSI_Freq_PHY_Cfg_map[] =
{
    /*  freq             CK_LANE_CFG0       CK_LANE_CFG1      CK_LANE_CFG2     D_LANE0_CFG0       D_LANE0_CFG1 */
    {DSI_FREQ_240Mbps,  0, 15, 15, 89,      7,   72, 19, 33,     20,           1,  2, 4, 2,        0,  4,  3, 20},
    {DSI_FREQ_288Mbps,  0, 15, 15, 89,      7,   72, 19, 33,     20,           1,  2, 4, 2,        0,  4,  3, 20},
    {DSI_FREQ_336Mbps,  2, 23, 23, 100,     23,  75, 21, 36,     30,           2,  2, 6, 3,        0,  4,  3, 20},
    {DSI_FREQ_384Mbps,  0, 23, 23, 115,     23,  78, 26, 44,     40,           2,  3, 7, 3,        0,  4,  3, 20},
    {DSI_FREQ_480Mbps,  0, 31, 31, 119,     23,  82, 30, 50,     50,           3,  3, 7, 5,        0,  4,  3, 20},
    {DSI_FREQ_528Mbps,  2, 23, 23, 134,     23,  80, 28, 47,     40,           3,  3, 7, 4,        0,  4,  3, 20}
};

/* Private variables ---------------------------------------------------------*/
static uint8_t force_enable_err_report = 0; //Force enable DSI acknowledge

/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void DSI_ConfigPacketHeader(DSI_HOST_TypeDef *DSIx, uint32_t ChannelID, uint32_t DataType, uint32_t Data0, uint32_t Data1);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Generic DSI packet header configuration
  * @param  DSIx  Pointer to DSI register base
  * @param  ChannelID Virtual channel ID of the header packet
  * @param  DataType  Packet data type of the header packet
  *                   This parameter can be any value of :
  *                      @ref DSI_SHORT_WRITE_PKT_Data_Type
  *                   or @ref DSI_LONG_WRITE_PKT_Data_Type
  *                   or @ref DSI_SHORT_READ_PKT_Data_Type
  *                   or DSI_MAX_RETURN_PKT_SIZE
  * @param  Data0  Word count LSB
  * @param  Data1  Word count MSB
  * @retval None
  */
static void DSI_ConfigPacketHeader(DSI_HOST_TypeDef *DSIx,
                                   uint32_t ChannelID,
                                   uint32_t DataType,
                                   uint32_t Data0,
                                   uint32_t Data1)
{
    /* Update the DSI packet header with new information */
    DSIx->COM_INTF = (DataType | (ChannelID << DSI_HOST_COM_INTF_VID_Pos) | (Data0 << DSI_HOST_COM_INTF_LOW_DATA_Pos) | (Data1 << DSI_HOST_COM_INTF_HIGH_DATA_Pos));
}

/* Exported functions --------------------------------------------------------*/
/** @addtogroup DSI_Exported_Functions
  * @{
  */

/** @defgroup DSI_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
                ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize and configure the DSI
      (+) De-initialize the DSI

@endverbatim
  * @{
  */


static Freq2DLLMap_t GetDLLMap(uint32_t freq)
{
    for (uint32_t i = 0; i < (sizeof(DSI_Freq_DLL_map) / sizeof(DSI_Freq_DLL_map[0])); i++)
    {
        if (freq == DSI_Freq_DLL_map[i].freq)
        {
            return (Freq2DLLMap_t)&DSI_Freq_DLL_map[i];
        }
    }

    return NULL;
}

static Freq2PhyCfgMap_t GetPHYCfgMap(uint32_t freq)
{
    for (uint32_t i = 0; i < (sizeof(DSI_Freq_PHY_Cfg_map) / sizeof(DSI_Freq_PHY_Cfg_map[0])); i++)
    {
        if (freq == DSI_Freq_PHY_Cfg_map[i].freq)
        {
            return (Freq2PhyCfgMap_t)&DSI_Freq_PHY_Cfg_map[i];
        }
    }

    return NULL;
}

/**
  * @brief  Initializes the DSI according to the specified
  *         parameters in the DSI_InitTypeDef and create the associated handle.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_Init(DSI_HandleTypeDef *hdsi)
{
    uint32_t tickstart;
    uint32_t unitIntervalx4;
    uint32_t tempIDF;

    /* Check the DSI handle allocation */
    if (hdsi == NULL)
    {
        return HAL_ERROR;
    }

    /* Check function parameters */
    //HAL_ASSERT(IS_DSI_PLL_NDIV(DLLInit->PLLNDIV));
    //HAL_ASSERT(IS_DSI_PLL_IDF(DLLInit->PLLIDF));
    //HAL_ASSERT(IS_DSI_PLL_ODF(DLLInit->PLLODF));
    HAL_ASSERT(IS_DSI_AUTO_CLKLANE_CONTROL(hdsi->Init.AutomaticClockLaneControl));
    HAL_ASSERT(IS_DSI_NUMBER_OF_LANES(hdsi->Init.NumberOfLanes));

    HAL_RCC_EnableModule(RCC_MOD_DSIHOST);
    HAL_RCC_EnableModule(RCC_MOD_DSIPHY);
    HAL_RCC_ResetModule(RCC_MOD_DSIHOST);
    HAL_RCC_ResetModule(RCC_MOD_DSIPHY);

    if (hdsi->State == HAL_DSI_STATE_RESET)
    {
        /* Initialize the low level hardware */
        HAL_DSI_MspInit(hdsi);
    }

    /* Change DSI peripheral state */
    hdsi->State = HAL_DSI_STATE_BUSY;


    /*************************** Set the PHY parameters ***************************/

    /* Clock lane configuration */
    hdsi->Instance->PHY_CTRL &= ~(DSI_HOST_PHY_CTRL_CKLANE_HSTXREQ | DSI_HOST_PHY_CTRL_CKLANE_AUTO_CTRL);
    hdsi->Instance->PHY_CTRL |= (DSI_HOST_PHY_CTRL_CKLANE_HSTXREQ | hdsi->Init.AutomaticClockLaneControl);

    /* Configure the number of active data lanes */
    hdsi->Instance->PHY_CFG1 &= ~DSI_HOST_PHY_CFG1_NUM_ACT_LANE;
    hdsi->Instance->PHY_CFG1 |= hdsi->Init.NumberOfLanes << DSI_HOST_PHY_CFG1_NUM_ACT_LANE_Pos;


    /************************ Set the DSI clock parameters ************************/

    {
        /* Set the TX escape clock division factor */
        hdsi->Instance->CTRL &= ~DSI_HOST_CTRL_ESC_DIV;
        hdsi->Instance->CTRL |= (hdsi->Init.TXEscapeCkdiv << DSI_HOST_CTRL_ESC_DIV_Pos); //escape clk  = dsi_phy_clk / 2(DDR) / 8 / ESC_DIV
    }
#ifdef DSI_FIX_ME  //PHY timing
    /* Calculate the bit period in high-speed mode in unit of 0.25 ns (UIX4) */
    /* The equation is : UIX4 = IntegerPart( (1000/F_PHY_Mhz) * 4 )          */
    /* Where : F_PHY_Mhz = (NDIV * HSE_Mhz) / (IDF * ODF)                    */
    tempIDF = (DLLInit->PLLIDF > 0U) ? DLLInit->PLLIDF : 1U;
    unitIntervalx4 = (4000000U * tempIDF * (1U << DLLInit->PLLODF)) / ((HSE_VALUE / 1000U) * DLLInit->PLLNDIV);

    /* Set the bit period in high-speed mode */
    hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_UIX4;
    hdsi->Instance->WPCR[0U] |= unitIntervalx4;

#else

    /* Enable the DSI host */
    //__HAL_DSI_ENABLE(hdsi);

    /* D-PHY clock and digital enable*/
    hdsi->Instance->PHY_CTRL |= (DSI_HOST_PHY_CTRL_CKLANE_ENABLE | DSI_HOST_PHY_CTRL_PHY_RSTN | DSI_HOST_PHY_CTRL_PLL_EN | DSI_HOST_PHY_CTRL_PHY_PD_N);

    /**************** Turn on the regulator and enable the DSI PLL ****************/

    /* Enable the analog power */
    __HAL_DSI_ANA_ENABLE(hdsi);

    /* wait for 100 us*/
    HAL_Delay_us(100U);

#if 1
    /*start calib */
    hdsi->PHY_Instance->BIAS_CAL |= DSI_PHY_BIAS_CAL_AUTO_CLK_DIV_Msk;
    hdsi->PHY_Instance->BIAS_CAL |= DSI_PHY_BIAS_CAL_START;
    uint32_t start_tick = HAL_GetTick();
    do
    {
        if (hdsi->PHY_Instance->BIAS_CAL & DSI_PHY_BIAS_CAL_DONE) break;
    }
    while (HAL_GetTick() - start_tick < 100);
    HAL_ASSERT(hdsi->PHY_Instance->BIAS_CAL & DSI_PHY_BIAS_CAL_DONE);
#else
    hdsi->PHY_Instance->BIAS_CAL &= ~DSI_PHY_BIAS_CAL_RSET_MANUAL_Msk;
    hdsi->PHY_Instance->BIAS_CAL |= DSI_PHY_BIAS_CAL_RCAL_EN_MANUAL | (0x3 << DSI_PHY_BIAS_CAL_RSET_MANUAL_Pos);
#endif



    // phy power control
    hdsi->PHY_Instance->CK_LANE_CTRL = (1 << DSI_PHY_CK_LANE_CTRL_HSTXPON_Pos) |
                                       (1 << DSI_PHY_CK_LANE_CTRL_LPTXPON_Pos) |
                                       (0 << DSI_PHY_CK_LANE_CTRL_LPRXPONLP_Pos) |
                                       (0 << DSI_PHY_CK_LANE_CTRL_LPRXPONULP_Pos) |
                                       (0 << DSI_PHY_CK_LANE_CTRL_LPRXPONCD_Pos) |
                                       (1 << DSI_PHY_CK_LANE_CTRL_BIASPON_Pos);

    hdsi->PHY_Instance->D_LANE0_CTRL = (1 << DSI_PHY_D_LANE0_CTRL_HSTXPON_Pos) |
                                       (1 << DSI_PHY_D_LANE0_CTRL_LPTXPON_Pos) |
                                       (1 << DSI_PHY_D_LANE0_CTRL_LPRXPONLP_Pos) |
                                       (0 << DSI_PHY_D_LANE0_CTRL_LPRXPONULP_Pos) |
                                       (0 << DSI_PHY_D_LANE0_CTRL_LPRXPONCD_Pos) |
                                       (1 << DSI_PHY_D_LANE0_CTRL_BIASPON_Pos) |
                                       (1 << DSI_PHY_D_LANE0_CTRL_LPTXPU_BYPASS_Pos) |
                                       (0 << DSI_PHY_D_LANE0_CTRL_LPTXPU_SET_Pos);



    if (DSI_TWO_DATA_LANES == hdsi->Init.NumberOfLanes)
    {
        hdsi->PHY_Instance->D_LANE1_CTRL = hdsi->PHY_Instance->D_LANE0_CTRL;
    }

#endif

    /****************************** Error management *****************************/

    /* Disable all error interrupts and reset the Error Mask */
    hdsi->Instance->INT_MASK1 = 0U;
    hdsi->Instance->INT_MASK2 = 0U;
    hdsi->ErrorMsk = 0U;

    /* Initialise the error code */
    hdsi->ErrorCode = HAL_DSI_ERROR_NONE;

    /* Initialize the DSI state*/
    hdsi->State = HAL_DSI_STATE_READY;

    return HAL_OK;
}



/**
  * @brief  De-initializes the DSI peripheral registers to their default reset
  *         values.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_DeInit(DSI_HandleTypeDef *hdsi)
{
    /* Check the DSI handle allocation */
    if (hdsi == NULL)
    {
        return HAL_ERROR;
    }

    /* Change DSI peripheral state */
    hdsi->State = HAL_DSI_STATE_BUSY;

    /* Disable the DSI PHY */
    __HAL_DSI_PHY_DISABLE(hdsi);

    /* Disable the DSI host */
    __HAL_DSI_DISABLE(hdsi);

    /* D-PHY clock and digital disable */
    hdsi->Instance->PHY_CTRL &= ~(DSI_HOST_PHY_CTRL_CKLANE_ENABLE /* | DSI_HOST_PHY_CTRL_PHY_RSTN*/);

    /* Turn off the DSI PLL */
    __HAL_DSI_PLL_DISABLE(hdsi);

    /* Disable the analog power */
    __HAL_DSI_ANA_DISABLE(hdsi);

    /* DeInit the low level hardware */
    HAL_DSI_MspDeInit(hdsi);

    /* Initialise the error code */
    hdsi->ErrorCode = HAL_DSI_ERROR_NONE;

    /* Initialize the DSI state*/
    hdsi->State = HAL_DSI_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Enable the error monitor flags
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  ActiveErrors  indicates which error interrupts will be enabled.
  *                      This parameter can be any combination of @ref DSI_Error_Data_Type.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigErrorMonitor(DSI_HandleTypeDef *hdsi, uint32_t ActiveErrors)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    hdsi->Instance->INT_MASK1 = 0U;
    hdsi->Instance->INT_MASK2 = 0U;

    /* Store active errors to the handle */
    hdsi->ErrorMsk = ActiveErrors;

    if ((ActiveErrors & HAL_DSI_ERROR_ACK) != RESET)
    {
        /* Enable the interrupt generation on selected errors */
        hdsi->Instance->INT_MASK1 |= DSI_ERROR_ACK_MASK;
    }

    if ((ActiveErrors & HAL_DSI_ERROR_PHY) != RESET)
    {
        /* Enable the interrupt generation on selected errors */
        hdsi->Instance->INT_MASK1 |= DSI_ERROR_PHY_MASK;
    }

    if ((ActiveErrors & HAL_DSI_ERROR_TX) != RESET)
    {
        /* Enable the interrupt generation on selected errors */
        hdsi->Instance->INT_MASK2 |= DSI_ERROR_TX_MASK;
    }

    if ((ActiveErrors & HAL_DSI_ERROR_RX) != RESET)
    {
        /* Enable the interrupt generation on selected errors */
        hdsi->Instance->INT_MASK2 |= DSI_ERROR_RX_MASK;
    }

    if ((ActiveErrors & HAL_DSI_ERROR_ECC) != RESET)
    {
        /* Enable the interrupt generation on selected errors */
        hdsi->Instance->INT_MASK2 |= DSI_ERROR_ECC_MASK;
    }

    if ((ActiveErrors & HAL_DSI_ERROR_CRC) != RESET)
    {
        /* Enable the interrupt generation on selected errors */
        hdsi->Instance->INT_MASK2 |= DSI_ERROR_CRC_MASK;
    }

    if ((ActiveErrors & HAL_DSI_ERROR_PSE) != RESET)
    {
        /* Enable the interrupt generation on selected errors */
        hdsi->Instance->INT_MASK2 |= DSI_ERROR_PSE_MASK;
    }

    if ((ActiveErrors & HAL_DSI_ERROR_EOT) != RESET)
    {
        /* Enable the interrupt generation on selected errors */
        hdsi->Instance->INT_MASK2 |= DSI_ERROR_EOT_MASK;
    }

    if ((ActiveErrors & HAL_DSI_ERROR_OVF) != RESET)
    {
        /* Enable the interrupt generation on selected errors */
        hdsi->Instance->INT_MASK2 |= DSI_ERROR_OVF_MASK;
    }

    if ((ActiveErrors & HAL_DSI_ERROR_GEN) != RESET)
    {
        /* Enable the interrupt generation on selected errors */
        hdsi->Instance->INT_MASK2 |= DSI_ERROR_GEN_MASK;
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Initializes the DSI MSP.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
__weak void HAL_DSI_MspInit(DSI_HandleTypeDef *hdsi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdsi);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_DSI_MspInit could be implemented in the user file
     */
}

/**
  * @brief  De-initializes the DSI MSP.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
__weak void HAL_DSI_MspDeInit(DSI_HandleTypeDef *hdsi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdsi);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_DSI_MspDeInit could be implemented in the user file
     */
}

/**
  * @}
  */

/** @defgroup DSI_Group2 IO operation functions
 *  @brief    IO operation functions
 *
@verbatim
 ===============================================================================
                      #####  IO operation functions  #####
 ===============================================================================
    [..]  This section provides function allowing to:
      (+) Handle DSI interrupt request

@endverbatim
  * @{
  */
/**
  * @brief  Handles DSI interrupt request.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
uint32_t prev_IntStatus1 = 0;
uint32_t IntStatus1_cnt = 0;
extern void rt_kprintf(const char *fmt, ...);

void HAL_DSI_IRQHandler(DSI_HandleTypeDef *hdsi)
{
    uint32_t IntStatus1, IntStatus2;


#if 0
    /* Tearing Effect Interrupt management ***************************************/
    if (__HAL_DSI_GET_FLAG(hdsi, DSI_FLAG_TE) != RESET)
    {
        if (__HAL_DSI_GET_IT_SOURCE(hdsi, DSI_IT_TE) != RESET)
        {
            /* Clear the Tearing Effect Interrupt Flag */
            __HAL_DSI_CLEAR_FLAG(hdsi, DSI_FLAG_TE);

            /* Tearing Effect Callback */
            HAL_DSI_TearingEffectCallback(hdsi);
        }
    }

    /* End of Refresh Interrupt management ***************************************/
    if (__HAL_DSI_GET_FLAG(hdsi, DSI_FLAG_ER) != RESET)
    {
        if (__HAL_DSI_GET_IT_SOURCE(hdsi, DSI_IT_ER) != RESET)
        {
            /* Clear the End of Refresh Interrupt Flag */
            __HAL_DSI_CLEAR_FLAG(hdsi, DSI_FLAG_ER);

            /* End of Refresh Callback */
            HAL_DSI_EndOfRefreshCallback(hdsi);
        }
    }
#endif


    /* Error Interrupts management ***********************************************/
    if (hdsi->ErrorMsk != 0U)
    {
        IntStatus1 = hdsi->Instance->INT_STAT1;
        IntStatus1 &= hdsi->Instance->INT_MASK1;
        IntStatus2 = hdsi->Instance->INT_STAT2;
        IntStatus2 &= hdsi->Instance->INT_MASK2;


#if 0
        if ((prev_IntStatus1 != IntStatus1) || (IntStatus1_cnt > 100000))
        {

            rt_kprintf("\nHAL_DSI_IRQHandler ist0=0x%x, ist1=0x%x\n", IntStatus1, IntStatus2);
            IntStatus1_cnt = 1;
            prev_IntStatus1 = IntStatus1;
        }
        else
        {
            IntStatus1_cnt++;
        }
#endif /* 0 */

        if ((IntStatus1 & DSI_ERROR_ACK_MASK) != RESET)
        {
            hdsi->ErrorCode |= HAL_DSI_ERROR_ACK;
            DSI_LOG_E("HAL_DSI_ERROR_ACK");
        }

        if ((IntStatus1 & DSI_ERROR_PHY_MASK) != RESET)
        {
            hdsi->ErrorCode |= HAL_DSI_ERROR_PHY;
            DSI_LOG_E("HAL_DSI_ERROR_PHY");
        }

        if ((IntStatus2 & DSI_ERROR_TX_MASK) != RESET)
        {
            hdsi->ErrorCode |= HAL_DSI_ERROR_TX;
            DSI_LOG_E("HAL_DSI_ERROR_TX");
        }

        if ((IntStatus2 & DSI_ERROR_RX_MASK) != RESET)
        {
            hdsi->ErrorCode |= HAL_DSI_ERROR_RX;
            DSI_LOG_E("HAL_DSI_ERROR_RX");
        }

        if ((IntStatus2 & DSI_ERROR_ECC_MASK) != RESET)
        {
            hdsi->ErrorCode |= HAL_DSI_ERROR_ECC;
            DSI_LOG_E("HAL_DSI_ERROR_ECC");
        }

        if ((IntStatus2 & DSI_ERROR_CRC_MASK) != RESET)
        {
            hdsi->ErrorCode |= HAL_DSI_ERROR_CRC;
            DSI_LOG_E("HAL_DSI_ERROR_CRC");
        }

        if ((IntStatus2 & DSI_ERROR_PSE_MASK) != RESET)
        {
            hdsi->ErrorCode |= HAL_DSI_ERROR_PSE;
            DSI_LOG_E("HAL_DSI_ERROR_PSE");
        }

        if ((IntStatus2 & DSI_ERROR_EOT_MASK) != RESET)
        {
            hdsi->ErrorCode |= HAL_DSI_ERROR_EOT;
            DSI_LOG_E("HAL_DSI_ERROR_EOT");
        }

        if ((IntStatus2 & DSI_ERROR_OVF_MASK) != RESET)
        {
            hdsi->ErrorCode |= HAL_DSI_ERROR_OVF;
            DSI_LOG_E("HAL_DSI_ERROR_OVF");
        }

        if ((IntStatus2 & DSI_ERROR_GEN_MASK) != RESET)
        {
            hdsi->ErrorCode |= HAL_DSI_ERROR_GEN;
            DSI_LOG_E("HAL_DSI_ERROR_GEN");
        }

        /* Check only selected errors */
        if (hdsi->ErrorCode != HAL_DSI_ERROR_NONE)
        {
            DSI_LOG_E("IntStatus1 =%x, IntStatus2 = %x", IntStatus1, IntStatus2);

            /* DSI error interrupt user callback */
            HAL_DSI_ErrorCallback(hdsi);
        }
    }
}



/**
  * @brief  Tearing Effect DSI callback.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
__weak void HAL_DSI_TearingEffectCallback(DSI_HandleTypeDef *hdsi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdsi);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_DSI_TearingEffectCallback could be implemented in the user file
     */
}

/**
  * @brief  End of Refresh DSI callback.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
__weak void HAL_DSI_EndOfRefreshCallback(DSI_HandleTypeDef *hdsi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdsi);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_DSI_EndOfRefreshCallback could be implemented in the user file
     */
}

/**
  * @brief  Operation Error DSI callback.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
__weak void HAL_DSI_ErrorCallback(DSI_HandleTypeDef *hdsi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hdsi);
    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_DSI_ErrorCallback could be implemented in the user file
     */
}

/**
  * @}
  */

/** @defgroup DSI_Group3 Peripheral Control functions
 *  @brief    Peripheral Control functions
 *
@verbatim
 ===============================================================================
                    ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure the Generic interface read-back Virtual Channel ID
      (+) Select video mode and configure the corresponding parameters
      (+) Configure command transmission mode: High-speed or Low-power
      (+) Configure the flow control
      (+) Configure the DSI PHY timer
      (+) Configure the DSI HOST timeout
      (+) Configure the DSI HOST timeout
      (+) Start/Stop the DSI module
      (+) Refresh the display in command mode
      (+) Controls the display color mode in Video mode
      (+) Control the display shutdown in Video mode
      (+) write short DCS or short Generic command
      (+) write long DCS or long Generic command
      (+) Read command (DCS or generic)
      (+) Enter/Exit the Ultra Low Power Mode on data only (D-PHY PLL running)
      (+) Enter/Exit the Ultra Low Power Mode on data only and clock (D-PHY PLL turned off)
      (+) Start/Stop test pattern generation
      (+) Slew-Rate And Delay Tuning
      (+) Low-Power Reception Filter Tuning
      (+) Activate an additional current path on all lanes to meet the SDDTx parameter
      (+) Custom lane pins configuration
      (+) Set custom timing for the PHY
      (+) Force the Clock/Data Lane in TX Stop Mode
      (+) Force LP Receiver in Low-Power Mode
      (+) Force Data Lanes in RX Mode after a BTA
      (+) Enable a pull-down on the lanes to prevent from floating states when unused
      (+) Switch off the contention detection on data lanes

@endverbatim
  * @{
  */

/**
  * @brief  Configure the Generic interface read-back Virtual Channel ID.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  VirtualChannelID  Virtual channel ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_SetGenericVCID(DSI_HandleTypeDef *hdsi, uint32_t VirtualChannelID)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Update the GVCID register */
    hdsi->Instance->COM_CFG &= ~DSI_HOST_COM_CFG_VID;
    hdsi->Instance->COM_CFG |= VirtualChannelID;

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}


/**
  * @brief  Select video mode and configure the corresponding parameters
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  VidCfg pointer to a DSI_VidCfgTypeDef structure that contains
  *                the DSI video mode configuration parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigVideoMode(DSI_HandleTypeDef *hdsi, DSI_VidCfgTypeDef *VidCfg)
{
    /* Process locked */
    __HAL_LOCK(hdsi);


#ifdef SF32LB58X

    uint32_t cf;

    if (DSI_RGB888 == VidCfg->ColorCoding)
        cf = 5;
    else if (DSI_RGB666 == VidCfg->ColorCoding)
        cf = 3;
    else if (DSI_RGB565 == VidCfg->ColorCoding)
        cf = 0;
    else
    {
        HAL_ASSERT(0); //Unknown format
    }


    hwp_dsi_host->DPI_CFG1 = (0 << DSI_HOST_DPI_CFG1_VID_Pos) |
                             (cf << DSI_HOST_DPI_CFG1_FORMAT_Pos) |
                             (0 << DSI_HOST_DPI_CFG1_LOOSE_EN_Pos) |
                             (0 << DSI_HOST_DPI_CFG1_D_EN_POL_Pos) |
                             (0 << DSI_HOST_DPI_CFG1_VSYNC_POL_Pos) |
                             (0 << DSI_HOST_DPI_CFG1_HSYNC_POL_Pos) |
                             (0 << DSI_HOST_DPI_CFG1_SHUTDOWN_POL_Pos) |
                             (0 << DSI_HOST_DPI_CFG1_COLORM_POL_Pos);
    hwp_dsi_host->DPI_CFG2 = (0 << DSI_HOST_DPI_CFG2_INVACT_Pos) |
                             (0 << DSI_HOST_DPI_CFG2_OUTVACT_Pos);




    hwp_dsi_host->VMODE_CFG1 = (VidCfg->Mode  << DSI_HOST_VMODE_CFG1_VMODE_TYPE_Pos) |         // 0: burst mode
                               (1    << DSI_HOST_VMODE_CFG1_LPVSA_EN_Pos) |           // enable LP mode for VSA
                               (1    << DSI_HOST_VMODE_CFG1_LPVBP_EN_Pos) |           // enable LP mode for VBP
                               (1    << DSI_HOST_VMODE_CFG1_LPVFP_EN_Pos) |           // enable LP mode for VFP
                               (1    << DSI_HOST_VMODE_CFG1_LPVACT_EN_Pos) |          // enable LP mode for VACT
                               (0    << DSI_HOST_VMODE_CFG1_LPHBP_EN_Pos) |           // enable LP mode for HBP
                               (0    << DSI_HOST_VMODE_CFG1_LPHFP_EN_Pos) |           // enable LP mode for HFP
                               (0    << DSI_HOST_VMODE_CFG1_BTA_ACK_EN_Pos) |         // no ack at end of frame
                               (1    << DSI_HOST_VMODE_CFG1_LPCMD_EN_Pos) |           // no command transmission in LP mode
                               (0    << DSI_HOST_VMODE_CFG1_VPG_EN_Pos) |             // disable video pattern generation
                               (0    << DSI_HOST_VMODE_CFG1_VPG_MODE_Pos) |           // no vpg mode
                               (0    << DSI_HOST_VMODE_CFG1_VPG_ORIENTATION_Pos) |    // no vpg mode
                               (VidCfg->PacketSize << DSI_HOST_VMODE_CFG1_PKT_SIZE_Pos);        // number of pixels per line in single video packet



    HAL_ASSERT(VidCfg->NullPacketSize <= (DSI_HOST_VMODE_CFG2_NULL_SIZE_Msk >> DSI_HOST_VMODE_CFG2_NULL_SIZE_Pos));
    hwp_dsi_host->VMODE_CFG2 = (VidCfg->NumberOfChunks << DSI_HOST_VMODE_CFG2_NUM_CHUNKS_Pos) |        // video line is transimitted in single packet
                               (VidCfg->NullPacketSize << DSI_HOST_VMODE_CFG2_NULL_SIZE_Pos);         // no null packet
    hwp_dsi_host->VTIMER1 = (VidCfg->HorizontalSyncActive << DSI_HOST_VTIMER1_HSA_Pos) |                     // HSA cycles
                            (VidCfg->HorizontalBackPorch << DSI_HOST_VTIMER1_HBP_Pos);                      // HBP cycles
    hwp_dsi_host->VTIMER2 = (VidCfg->HorizontalLine << DSI_HOST_VTIMER2_HLINE_Pos) |         // HLINE cycles // HSA+HBP+HACT+HFP period
                            (VidCfg->VerticalSyncActive << DSI_HOST_VTIMER2_VSA_Pos);                      // VSA lines
    hwp_dsi_host->VTIMER3 = (VidCfg->VerticalBackPorch << DSI_HOST_VTIMER3_VBP_Pos) |                     // VBP lines
                            (VidCfg->VerticalFrontPorch << DSI_HOST_VTIMER3_VFP_Pos);                      // VFP lines
    hwp_dsi_host->VTIMER4 = (VidCfg->VerticalActive  << DSI_HOST_VTIMER4_VACT_Pos);                   // VACT lines



    /*******
    Generate test pattern
    *****************/
    hwp_dsi_host->VMODE_CFG1 |= (0 << DSI_HOST_VMODE_CFG1_VPG_EN_Pos) |             // Enable video pattern generation
                                (0 << DSI_HOST_VMODE_CFG1_VPG_MODE_Pos) |           // 0: color bar, 1: ber pattern
                                (0 << DSI_HOST_VMODE_CFG1_VPG_ORIENTATION_Pos);     // color bar orientation, 0: vertical  1: horizontal


#endif /* SF32LB58X */

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Select adapted command mode and configure the corresponding parameters
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  CmdCfg  pointer to a DSI_CmdCfgTypeDef structure that contains
  *                 the DSI command mode configuration parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigAdaptedCommandMode(DSI_HandleTypeDef *hdsi, DSI_CmdCfgTypeDef *CmdCfg)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check the parameters */
    HAL_ASSERT(IS_DSI_COLOR_CODING(CmdCfg->ColorCoding));
    //HAL_ASSERT(IS_DSI_TE_SOURCE(CmdCfg->TearingEffectSource));
    //HAL_ASSERT(IS_DSI_TE_POLARITY(CmdCfg->TearingEffectPolarity));
    //HAL_ASSERT(IS_DSI_AUTOMATIC_REFRESH(CmdCfg->AutomaticRefresh));
    //HAL_ASSERT(IS_DSI_VS_POLARITY(CmdCfg->VSyncPol));
    HAL_ASSERT(IS_DSI_TE_ACK_REQUEST(CmdCfg->TEAcknowledgeRequest));
    //HAL_ASSERT(IS_DSI_DE_POLARITY(CmdCfg->DEPolarity));
    //HAL_ASSERT(IS_DSI_VSYNC_POLARITY(CmdCfg->VSPolarity));
    //HAL_ASSERT(IS_DSI_HSYNC_POLARITY(CmdCfg->HSPolarity));

    /* Select command mode by setting CMDM and DSIM bits */
    hdsi->Instance->MODE_CFG |= DSI_HOST_MODE_CFG_CMD_VIDEO_SEL;
#if 0//def DSI_FIX_ME   LCDC config

    hdsi->Instance->WCFGR &= ~DSI_WCFGR_DSIM;
    hdsi->Instance->WCFGR |= DSI_WCFGR_DSIM;
#endif

    /* Select the virtual channel for the LTDC interface traffic */
    hdsi->Instance->DBI_CFG1 &= ~DSI_HOST_DBI_CFG1_VID;
    hdsi->Instance->DBI_CFG1 |= CmdCfg->VirtualChannelID;
#ifndef SF32LB55X
    hdsi->Instance->DCOM_INTF_CFG3 &= ~DSI_HOST_DCOM_INTF_CFG3_ID_Msk;
    hdsi->Instance->DCOM_INTF_CFG3 |= CmdCfg->VirtualChannelID << DSI_HOST_DCOM_INTF_CFG3_ID_Pos;
#endif /* !SF32LB55X */



    hdsi->Instance->DBI_CFG1 &= ~(DSI_HOST_DBI_CFG1_IFORMAT_Msk | DSI_HOST_DBI_CFG1_OFORMAT_Msk | DSI_HOST_DBI_CFG1_LUT_SIZE_Msk);
#ifndef SF32LB55X
    hdsi->Instance->DCOM_INTF_CFG3 &= ~DSI_HOST_DCOM_INTF_CFG3_I_FORMAT_Msk;
#endif /* SF32LB55X */
    if (DSI_RGB888 == CmdCfg->ColorCoding)
    {
        hdsi->Instance->DBI_CFG1 |= (11 << DSI_HOST_DBI_CFG1_IFORMAT_Pos) |  // DBI input format 16-bit 24bpp option 1
                                    (11 << DSI_HOST_DBI_CFG1_OFORMAT_Pos) |  // DBI output format 16-bit 24bpp option 1
                                    (0  << DSI_HOST_DBI_CFG1_LUT_SIZE_Pos);   // 16-bit color display
#ifndef SF32LB55X
        hdsi->Instance->DCOM_INTF_CFG3 |= 1 << DSI_HOST_DCOM_INTF_CFG3_I_FORMAT_Pos;
#endif /* SF32LB55X */
    }
    else if (DSI_RGB565 == CmdCfg->ColorCoding)
    {
        hdsi->Instance->DBI_CFG1 |= (8 << DSI_HOST_DBI_CFG1_IFORMAT_Pos) |  // DBI input format 16-bit 16bpp option 1
                                    (8 << DSI_HOST_DBI_CFG1_OFORMAT_Pos) |  // DBI output format 16-bit 16bpp option 1
                                    (0  << DSI_HOST_DBI_CFG1_LUT_SIZE_Pos);   // 16-bit color display
#ifndef SF32LB55X
        hdsi->Instance->DCOM_INTF_CFG3 |= 0 << DSI_HOST_DCOM_INTF_CFG3_I_FORMAT_Pos;
#endif /* SF32LB55X */

    }
    else
    {
        HAL_ASSERT(0); //fix me
    }

#ifndef SF32LB55X
    hdsi->Instance->DCOM_INTF_CFG3 &= ~DSI_HOST_DCOM_INTF_CFG3_DATA_TYPE_Msk;
    hdsi->Instance->DCOM_INTF_CFG3 |= DSI_DCS_LONG_PKT_WRITE << DSI_HOST_DCOM_INTF_CFG3_DATA_TYPE_Pos;
#endif /* !SF32LB55X */

    hdsi->Instance->DBI_CFG1 &= ~DSI_HOST_DBI_CFG1_PARTITIONING_EN_Msk; //// disable partitioning,  0 -  use automatic partitioning

    hdsi->Instance->DBI_CFG1 |= (1  << 15);   // wr_cont_mode

    //Set default write command size
    hdsi->Instance->DBI_CFG2 &= ~DSI_HOST_DBI_CFG2_WR_SIZE_Msk;
    hdsi->Instance->DBI_CFG2 |= (CmdCfg->CommandSize << DSI_HOST_DBI_CFG2_WR_SIZE_Pos);


    //Allowed command size
    hdsi->Instance->DBI_CFG2 &= ~DSI_HOST_DBI_CFG2_ALLOWED_SIZE_Msk;
    //if (DSI_TWO_DATA_LANES == hdsi->Init.NumberOfLanes)

    hdsi->Instance->DBI_CFG2 |= (163  << DSI_HOST_DBI_CFG2_ALLOWED_SIZE_Pos);

    //else
    //    hdsi->Instance->DBI_CFG2 |= (120  << DSI_HOST_DBI_CFG2_ALLOWED_SIZE_Pos);


    /* Configure the tearing effect acknowledge request */
    hdsi->Instance->CMODE_CFG &= ~DSI_HOST_CMODE_CFG_FMARK_EN;
    if (DSI_TE_DSILINK == CmdCfg->TearingEffectSource) hdsi->Instance->CMODE_CFG |= CmdCfg->TEAcknowledgeRequest;

    /* Enable the Tearing Effect interrupt */
    __HAL_DSI_ENABLE_IT(hdsi, DSI_IT_TE);

    /* Enable the End of Refresh interrupt */
    __HAL_DSI_ENABLE_IT(hdsi, DSI_IT_ER);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Configure command transmission mode: High-speed or Low-power
  *         and enable/disable acknowledge request after packet transmission
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  LPCmd  pointer to a DSI_LPCmdTypeDef structure that contains
  *                the DSI command transmission mode configuration parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigCommand(DSI_HandleTypeDef *hdsi, DSI_LPCmdTypeDef *LPCmd)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    HAL_ASSERT(IS_DSI_LP_GSW0P(LPCmd->LPGenShortWriteNoP));
    HAL_ASSERT(IS_DSI_LP_GSW1P(LPCmd->LPGenShortWriteOneP));
    HAL_ASSERT(IS_DSI_LP_GSW2P(LPCmd->LPGenShortWriteTwoP));
    HAL_ASSERT(IS_DSI_LP_GSR0P(LPCmd->LPGenShortReadNoP));
    HAL_ASSERT(IS_DSI_LP_GSR1P(LPCmd->LPGenShortReadOneP));
    HAL_ASSERT(IS_DSI_LP_GSR2P(LPCmd->LPGenShortReadTwoP));
    HAL_ASSERT(IS_DSI_LP_GLW(LPCmd->LPGenLongWrite));
    HAL_ASSERT(IS_DSI_LP_DSW0P(LPCmd->LPDcsShortWriteNoP));
    HAL_ASSERT(IS_DSI_LP_DSW1P(LPCmd->LPDcsShortWriteOneP));
    HAL_ASSERT(IS_DSI_LP_DSR0P(LPCmd->LPDcsShortReadNoP));
    HAL_ASSERT(IS_DSI_LP_DLW(LPCmd->LPDcsLongWrite));
    HAL_ASSERT(IS_DSI_LP_MRDP(LPCmd->LPMaxReadPacket));
    HAL_ASSERT(IS_DSI_ACK_REQUEST(LPCmd->AcknowledgeRequest));

    /* Select High-speed or Low-power for command transmission */
    hdsi->Instance->CMODE_CFG &= ~(DSI_HOST_CMODE_CFG_COM_SHORT_WR0 | \
                                   DSI_HOST_CMODE_CFG_COM_SHORT_WR1 | \
                                   DSI_HOST_CMODE_CFG_COM_SHORT_WR2 | \
                                   DSI_HOST_CMODE_CFG_COM_SHORT_RD0 | \
                                   DSI_HOST_CMODE_CFG_COM_SHORT_RD1 | \
                                   DSI_HOST_CMODE_CFG_COM_SHORT_RD2 | \
                                   DSI_HOST_CMODE_CFG_COM_LONG_WR  | \
                                   DSI_HOST_CMODE_CFG_SHORT_WR0 | \
                                   DSI_HOST_CMODE_CFG_SHORT_WR1 | \
                                   DSI_HOST_CMODE_CFG_SHORT_RD0 | \
                                   DSI_HOST_CMODE_CFG_LONG_WR  | \
                                   DSI_HOST_CMODE_CFG_MAX_RD_SIZE);
    hdsi->Instance->CMODE_CFG |= (LPCmd->LPGenShortWriteNoP  | \
                                  LPCmd->LPGenShortWriteOneP | \
                                  LPCmd->LPGenShortWriteTwoP | \
                                  LPCmd->LPGenShortReadNoP   | \
                                  LPCmd->LPGenShortReadOneP  | \
                                  LPCmd->LPGenShortReadTwoP  | \
                                  LPCmd->LPGenLongWrite      | \
                                  LPCmd->LPDcsShortWriteNoP  | \
                                  LPCmd->LPDcsShortWriteOneP | \
                                  LPCmd->LPDcsShortReadNoP   | \
                                  LPCmd->LPDcsLongWrite      | \
                                  LPCmd->LPMaxReadPacket);

    /* Configure the acknowledge request after each packet transmission */
    hdsi->Instance->CMODE_CFG &= ~DSI_HOST_CMODE_CFG_ACK_REQ_EN_Msk;

    if (force_enable_err_report)
        hdsi->Instance->CMODE_CFG |= DSI_ACKNOWLEDGE_ENABLE;
    else
        hdsi->Instance->CMODE_CFG |= LPCmd->AcknowledgeRequest;

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
 * @brief Force acknowledge enable for testcase
 * @param hdsi -
 * @return
 */
void HAL_DSI_ForceEnableAck(void)
{
    force_enable_err_report = 1;
}


/**
 * @brief Force acknowledge enable for testcase
 * @param hdsi -
 * @return
 */
uint8_t HAL_DSI_ISForceEnableAck(void)
{
    return force_enable_err_report;
}

/**
  * @brief  Configure the flow control parameters
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  FlowControl  flow control feature(s) to be enabled.
  *                      This parameter can be any combination of @ref DSI_FlowControl.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigFlowControl(DSI_HandleTypeDef *hdsi, uint32_t FlowControl)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check the parameters */
    HAL_ASSERT(IS_DSI_FLOW_CONTROL(FlowControl));

    /* Set the DSI Host Protocol Configuration Register */
    hdsi->Instance->PKT_CFG &= ~DSI_FLOW_CONTROL_ALL;
    hdsi->Instance->PKT_CFG |= FlowControl;

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Configure the DSI PHY timer parameters
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  PhyTimers  DSI_PHY_TimerTypeDef structure that contains
  *                    the DSI PHY timing parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigPhyTimer(DSI_HandleTypeDef *hdsi, DSI_PHY_TimerTypeDef *PhyTimers)
{
    uint32_t maxTime;
    /* Process locked */
    __HAL_LOCK(hdsi);

    maxTime = (PhyTimers->ClockLaneLP2HSTime > PhyTimers->ClockLaneHS2LPTime) ? PhyTimers->ClockLaneLP2HSTime : PhyTimers->ClockLaneHS2LPTime;

    /* Clock lane timer configuration */

    /* In Automatic Clock Lane control mode, the DSI Host can turn off the clock lane between two
       High-Speed transmission.
       To do so, the DSI Host calculates the time required for the clock lane to change from HighSpeed
       to Low-Power and from Low-Power to High-Speed.
       This timings are configured by the HS2LP_TIME and LP2HS_TIME in the DSI Host Clock Lane Timer Configuration Register (DSI_CLTCR).
       But the DSI Host is not calculating LP2HS_TIME + HS2LP_TIME but 2 x HS2LP_TIME.

       Workaround : Configure HS2LP_TIME and LP2HS_TIME with the same value being the max of HS2LP_TIME or LP2HS_TIME.
    */
    hdsi->Instance->PHY_CFG1 &= ~(DSI_HOST_PHY_CFG1_CKLP2HS_TIMER | DSI_HOST_PHY_CFG1_CKHS2LP_TIMER);
    hdsi->Instance->PHY_CFG1 |= ((maxTime << DSI_HOST_PHY_CFG1_CKLP2HS_TIMER_Pos) | ((maxTime) << DSI_HOST_PHY_CFG1_CKHS2LP_TIMER_Pos));

    /* Data lane timer configuration */
    hdsi->Instance->PHY_CFG2 &= ~(DSI_HOST_PHY_CFG2_MAX_RD_TIMER | DSI_HOST_PHY_CFG2_DATALP2HS_TIMER | DSI_HOST_PHY_CFG2_DATAHS2LP_TIMER);
    hdsi->Instance->PHY_CFG2 |= ((PhyTimers->DataLaneMaxReadTime << DSI_HOST_PHY_CFG2_MAX_RD_TIMER_Pos) | ((PhyTimers->DataLaneLP2HSTime) << DSI_HOST_PHY_CFG2_DATALP2HS_TIMER_Pos) | ((PhyTimers->DataLaneHS2LPTime) << DSI_HOST_PHY_CFG2_DATAHS2LP_TIMER_Pos));

    /* Configure the wait period to request HS transmission after a stop state */
    hdsi->Instance->PHY_CFG1 &= ~DSI_HOST_PHY_CFG1_WAIT_TIMER;
    hdsi->Instance->PHY_CFG1 |= ((PhyTimers->StopWaitTime) << DSI_HOST_PHY_CFG1_WAIT_TIMER_Pos);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Configure the DSI HOST timeout parameters
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  HostTimeouts  DSI_HOST_TimeoutTypeDef structure that contains
  *                       the DSI host timeout parameters
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigHostTimeouts(DSI_HandleTypeDef *hdsi, DSI_HOST_TimeoutTypeDef *HostTimeouts)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Set the timeout clock division factor */
    hdsi->Instance->CTRL &= ~DSI_HOST_CTRL_TIMEOUT_DIV;
    hdsi->Instance->CTRL |= ((HostTimeouts->TimeoutCkdiv) << DSI_HOST_CTRL_TIMEOUT_DIV_Pos);  //timeout clk  = dsi_phy_clk / 2(DDR) / 8 / TO_DIV

    /* High-speed transmission timeout */
    hdsi->Instance->ST_CNT_CFG1 &= ~DSI_HOST_ST_CNT_CFG1_HSTX_TIMEOUT;
    hdsi->Instance->ST_CNT_CFG1 |= ((HostTimeouts->HighSpeedTransmissionTimeout) << DSI_HOST_ST_CNT_CFG1_HSTX_TIMEOUT_Pos);

    /* Low-power reception timeout */
    hdsi->Instance->ST_CNT_CFG1 &= ~DSI_HOST_ST_CNT_CFG1_LPRX_TIMEOUT;
    hdsi->Instance->ST_CNT_CFG1 |= HostTimeouts->LowPowerReceptionTimeout << DSI_HOST_ST_CNT_CFG1_LPRX_TIMEOUT_Pos;

    /* High-speed read timeout */
    hdsi->Instance->ST_CNT_CFG2 &= ~DSI_HOST_ST_CNT_CFG2_HSRD_TIMEOUT;
    hdsi->Instance->ST_CNT_CFG2 |= HostTimeouts->HighSpeedReadTimeout << DSI_HOST_ST_CNT_CFG2_HSRD_TIMEOUT_Pos;

    /* Low-power read timeout */
    hdsi->Instance->ST_CNT_CFG2 &= ~DSI_HOST_ST_CNT_CFG2_LPRD_TIMEOUT;
    hdsi->Instance->ST_CNT_CFG2 |= HostTimeouts->LowPowerReadTimeout << DSI_HOST_ST_CNT_CFG2_LPRD_TIMEOUT_Pos;

    /* High-speed write timeout */
    hdsi->Instance->ST_CNT_CFG3 &= ~DSI_HOST_ST_CNT_CFG3_HSWR_TIMEOUT;
    hdsi->Instance->ST_CNT_CFG3 |= HostTimeouts->HighSpeedWriteTimeout << DSI_HOST_ST_CNT_CFG3_HSWR_TIMEOUT_Pos;

    /* Low-speed write timeout */
    hdsi->Instance->ST_CNT_CFG3 &= ~DSI_HOST_ST_CNT_CFG3_LPWR_TIMEOUT;
    hdsi->Instance->ST_CNT_CFG3 |= HostTimeouts->LowPowerWriteTimeout << DSI_HOST_ST_CNT_CFG3_LPWR_TIMEOUT_Pos;

    /* BTA timeout */
    hdsi->Instance->ST_CNT_CFG4 &= ~DSI_HOST_ST_CNT_CFG4_BTA_TIMEOUT;
    hdsi->Instance->ST_CNT_CFG4 |= HostTimeouts->BTATimeout << DSI_HOST_ST_CNT_CFG4_BTA_TIMEOUT_Pos;

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}


/**
  * @brief  Configure the DSI PHY frequency.
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  freq  DSI frequency.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ConfigPhyFreq(DSI_HandleTypeDef *hdsi, uint32_t freq)
{
    uint32_t tickstart;
    uint32_t unitIntervalx4;
    uint32_t tempIDF;

    /* Check the DSI handle allocation */
    if (hdsi == NULL)
    {
        return HAL_ERROR;
    }

    /* Change DSI peripheral state */
    hdsi->State = HAL_DSI_STATE_BUSY;

    /* Disable the DSI PLL */
    __HAL_DSI_PLL_DISABLE(hdsi);

    Freq2PhyCfgMap_t p_phy_cfg = GetPHYCfgMap(freq);
    HAL_ASSERT(p_phy_cfg != NULL);

    // setup phy timing parameter
    hdsi->PHY_Instance->CK_LANE_CFG0 = (p_phy_cfg->ck_phase_sel   << DSI_PHY_CK_LANE_CFG0_PHASE_SEL_Pos) |
                                       (p_phy_cfg->ck_lpx_cnt     << DSI_PHY_CK_LANE_CFG0_TLPX_CNT_Pos) |
                                       (p_phy_cfg->ck_prep_cnt    << DSI_PHY_CK_LANE_CFG0_TCLKPREP_CNT_Pos) |
                                       (p_phy_cfg->ck_zero_cnt    << DSI_PHY_CK_LANE_CFG0_TCLKZERO_CNT_Pos);
    hdsi->PHY_Instance->CK_LANE_CFG1 = (p_phy_cfg->ck_pre_cnt     << DSI_PHY_CK_LANE_CFG1_TCLKPRE_CNT_Pos) |
                                       (p_phy_cfg->ck_post_cnt    << DSI_PHY_CK_LANE_CFG1_TCLKPOST_CNT_Pos) |
                                       (p_phy_cfg->ck_trail_cnt   << DSI_PHY_CK_LANE_CFG1_TCLKTRAIL_CNT_Pos) |
                                       (p_phy_cfg->ck_hs_exit_cnt << DSI_PHY_CK_LANE_CFG1_THSEXIT_CNT_Pos);
    hdsi->PHY_Instance->CK_LANE_CFG2 = (p_phy_cfg->ck_wakeup_cnt  << DSI_PHY_CK_LANE_CFG2_TWAKEUP_CNT_Pos);
    hdsi->PHY_Instance->D_LANE0_CFG0 = (p_phy_cfg->dx_lpx_cnt     << DSI_PHY_D_LANE0_CFG0_TLPX_CNT_Pos) |
                                       (p_phy_cfg->dx_hs_prep_cnt << DSI_PHY_D_LANE0_CFG0_THSPREP_CNT_Pos) |
                                       (p_phy_cfg->dx_hs_zero_cnt << DSI_PHY_D_LANE0_CFG0_THSZERO_CNT_Pos) |
                                       (p_phy_cfg->dx_hs_trail_cnt << DSI_PHY_D_LANE0_CFG0_THSTRAIL_CNT_Pos);
    hdsi->PHY_Instance->D_LANE0_CFG1 = (p_phy_cfg->dx_tasure_cnt  << DSI_PHY_D_LANE0_CFG1_TTASURE_CNT_Pos) |
                                       (p_phy_cfg->dx_taget_cnt   << DSI_PHY_D_LANE0_CFG1_TTAGET_CNT_Pos) |
                                       (p_phy_cfg->dx_tago_cnt    << DSI_PHY_D_LANE0_CFG1_TTAGO_CNT_Pos) |
                                       (p_phy_cfg->dx_wakeup_cnt  << DSI_PHY_D_LANE0_CFG1_TWAKEUP_CNT_Pos);



    //D_LANE1 use same config as D_LANE0
    if (DSI_TWO_DATA_LANES == hdsi->Init.NumberOfLanes)
    {
        hdsi->PHY_Instance->D_LANE1_CFG0 = hdsi->PHY_Instance->D_LANE0_CFG0;
        hdsi->PHY_Instance->D_LANE1_CFG1 = hdsi->PHY_Instance->D_LANE0_CFG1;
    }


    /* Set the PLL division factors */
    Freq2DLLMap_t dll = GetDLLMap(freq);
    HAL_ASSERT(dll != NULL);
    hdsi->PHY_Instance->DLL_CFG &= ~(DSI_PHY_DLL_CFG_DLL0_IN_DIV2_EN |
                                     DSI_PHY_DLL_CFG_DLL0_STG |
                                     DSI_PHY_DLL_CFG_DLL0_OUT_DIV2_EN |
                                     DSI_PHY_DLL_CFG_DLL0_48M_EN);
    hdsi->PHY_Instance->DLL_CFG |= ((dll->stg) << DSI_PHY_DLL_CFG_DLL0_STG_Pos);
    if (1 == dll->in_div2)
    {
        hdsi->PHY_Instance->DLL_CFG |= DSI_PHY_DLL_CFG_DLL0_IN_DIV2_EN;
    }
    if (1 == dll->out_div2)
    {
        hdsi->PHY_Instance->DLL_CFG |= DSI_PHY_DLL_CFG_DLL0_OUT_DIV2_EN;
    }


    if (DSI_FREQ_528Mbps == freq)
    {

        MODIFY_REG(hdsi->PHY_Instance->BIAS_CTRL, DSI_PHY_BIAS_CTRL_BIAS_PROG_Msk,
                   0x659B << DSI_PHY_BIAS_CTRL_BIAS_PROG_Pos);

        MODIFY_REG(hdsi->PHY_Instance->BIAS_CAL, DSI_PHY_BIAS_CAL_RSET_MANUAL_Msk | DSI_PHY_BIAS_CAL_MODE_Msk,
                   (7 << DSI_PHY_BIAS_CAL_RSET_MANUAL_Pos) | (1 << DSI_PHY_BIAS_CAL_MODE_Pos));

    }


    /* Enable the DSI PLL */
    __HAL_DSI_PLL_ENABLE(hdsi);

    /* PLL ready need 10~20us, wait for 50 us*/
    HAL_Delay_us(50U);




    /* Initialize the DSI state*/
    hdsi->State = HAL_DSI_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  Start the DSI module
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_Start(DSI_HandleTypeDef *hdsi)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Enable the DSI host */
    __HAL_DSI_ENABLE(hdsi);

    hdsi->PHY_Instance->PHY_CFG &= ~DSI_PHY_PHY_CFG_INIT_CNT_Msk;
    hdsi->PHY_Instance->PHY_CFG |= (100 << DSI_PHY_PHY_CFG_INIT_CNT_Pos);

    /* Enable the DSI PHY */
    __HAL_DSI_PHY_ENABLE(hdsi);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}


#if 0

/**
  * @brief  Start the DSI module and keep on ULPM mode
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_Start_ULPM(DSI_HandleTypeDef *hdsi)
{
    /* Process locked */
    __HAL_LOCK(hdsi);


    /*1. Bypas all output lanes of DSI-PHY*/
    {
        //hdsi->PHY_Instance->PHY_CFG |= DSI_PHY_PHY_CFG_RESETN;

        hdsi->PHY_Instance->CK_LANE_CTRL |= DSI_PHY_CK_LANE_CTRL_LPTXENA_BYPASS
                                            | DSI_PHY_CK_LANE_CTRL_LPTXENA_SET
                                            | DSI_PHY_CK_LANE_CTRL_LPTXDIN_BYPASS;

        hdsi->PHY_Instance->CK_LANE_CTRL &= ~DSI_PHY_CK_LANE_CTRL_LPTXDIN_SET;



        hdsi->PHY_Instance->D_LANE0_CTRL |= DSI_PHY_D_LANE0_CTRL_LPTXENA_BYPASS
                                            | DSI_PHY_D_LANE0_CTRL_LPTXENA_SET
                                            | DSI_PHY_D_LANE0_CTRL_LPTXDIN_BYPASS;

        hdsi->PHY_Instance->D_LANE0_CTRL &= ~DSI_PHY_D_LANE0_CTRL_LPTXDIN_SET;


        if (DSI_TWO_DATA_LANES == hdsi->Init.NumberOfLanes)
        {
            hdsi->PHY_Instance->D_LANE1_CTRL = hdsi->PHY_Instance->D_LANE0_CTRL;
        }
    }


    /* 2. Enable the DSI host & PHY*/
    {
        __HAL_DSI_ENABLE(hdsi);

        hdsi->PHY_Instance->PHY_CFG &= ~DSI_PHY_PHY_CFG_INIT_CNT_Msk;
        hdsi->PHY_Instance->PHY_CFG |= (100 << DSI_PHY_PHY_CFG_INIT_CNT_Pos);
        /* Enable the DSI PHY */
        __HAL_DSI_PHY_ENABLE(hdsi);

        //hdsi->PHY_Instance->PHY_CFG |= DSI_PHY_PHY_CFG_ENABLE;
    }

    /*3. Enter ULPM, and it will not output to bus*/
    {
        hdsi->Instance->PHY_CTRL &= ~(DSI_HOST_PHY_CTRL_CKLANE_AUTO_CTRL | DSI_HOST_PHY_CTRL_CKLANE_HSTXREQ);
        /* ULPS Request on Clock and Data Lanes */
        hdsi->Instance->PHY_CTRL |= (DSI_HOST_PHY_CTRL_CKLANE_REQULPS | DSI_HOST_PHY_CTRL_DLANE_REQULPS);

        HAL_Delay(1);

        //hdsi->Instance->PHY_CTRL &= ~(DSI_HOST_PHY_CTRL_CKLANE_REQULPS | DSI_HOST_PHY_CTRL_DLANE_REQULPS);
        hdsi->Instance->PHY_CTRL |= hdsi->Init.AutomaticClockLaneControl; //Revert clk lane setting
        //hdsi->Instance->PHY_CTRL |= (DSI_HOST_PHY_CTRL_CKLANE_HSTXREQ | hdsi->Init.AutomaticClockLaneControl);

        HAL_Delay(1);
        //__asm("B .");
    }

    /*4. disable bypass*/
    {
        hdsi->PHY_Instance->CK_LANE_CTRL &= ~(DSI_PHY_CK_LANE_CTRL_LPTXENA_BYPASS | DSI_PHY_CK_LANE_CTRL_LPTXDIN_BYPASS);
        hdsi->PHY_Instance->D_LANE0_CTRL &= ~(DSI_PHY_D_LANE0_CTRL_LPTXENA_BYPASS | DSI_PHY_D_LANE0_CTRL_LPTXDIN_BYPASS);
        hdsi->PHY_Instance->D_LANE1_CTRL &= ~(DSI_PHY_D_LANE1_CTRL_LPTXENA_BYPASS | DSI_PHY_D_LANE1_CTRL_LPTXDIN_BYPASS);
    }

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}
#endif /* 0 */

HAL_StatusTypeDef HAL_DSI_bypass_output(DSI_HandleTypeDef *hdsi, bool bypass)
{
    __IO uint32_t tmpreg;

    /* Process locked */
    __HAL_LOCK(hdsi);

    if (bypass)
    {
        hdsi->PHY_Instance->CK_LANE_CTRL |= DSI_PHY_CK_LANE_CTRL_LPTXENA_BYPASS
                                            | DSI_PHY_CK_LANE_CTRL_LPTXENA_SET
                                            | DSI_PHY_CK_LANE_CTRL_LPTXDIN_BYPASS;

        hdsi->PHY_Instance->CK_LANE_CTRL &= ~DSI_PHY_CK_LANE_CTRL_LPTXDIN_SET;

        tmpreg = hdsi->PHY_Instance->CK_LANE_CTRL;

        hdsi->PHY_Instance->D_LANE0_CTRL |= DSI_PHY_D_LANE0_CTRL_LPTXENA_BYPASS
                                            | DSI_PHY_D_LANE0_CTRL_LPTXENA_SET
                                            | DSI_PHY_D_LANE0_CTRL_LPTXDIN_BYPASS;

        hdsi->PHY_Instance->D_LANE0_CTRL &= ~DSI_PHY_D_LANE0_CTRL_LPTXDIN_SET;

        tmpreg = hdsi->PHY_Instance->D_LANE0_CTRL;
        if (DSI_TWO_DATA_LANES == hdsi->Init.NumberOfLanes)
        {
            hdsi->PHY_Instance->D_LANE1_CTRL = hdsi->PHY_Instance->D_LANE0_CTRL;
            tmpreg = hdsi->PHY_Instance->D_LANE1_CTRL;
        }

    }
    else
    {
        hdsi->PHY_Instance->CK_LANE_CTRL &= ~(DSI_PHY_CK_LANE_CTRL_LPTXENA_BYPASS | DSI_PHY_CK_LANE_CTRL_LPTXDIN_BYPASS);
        tmpreg = hdsi->PHY_Instance->CK_LANE_CTRL;

        hdsi->PHY_Instance->D_LANE0_CTRL &= ~(DSI_PHY_D_LANE0_CTRL_LPTXENA_BYPASS | DSI_PHY_D_LANE0_CTRL_LPTXDIN_BYPASS);
        tmpreg = hdsi->PHY_Instance->D_LANE0_CTRL;

        if (DSI_TWO_DATA_LANES == hdsi->Init.NumberOfLanes)
        {
            hdsi->PHY_Instance->D_LANE1_CTRL = hdsi->PHY_Instance->D_LANE0_CTRL;
            tmpreg = hdsi->PHY_Instance->D_LANE1_CTRL;
        }
    }


    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;

}

/**
  * @brief  Stop the DSI module
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_Stop(DSI_HandleTypeDef *hdsi)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Disable the DSI host */
    __HAL_DSI_DISABLE(hdsi);

    /* Disable the DSI PHY */
    __HAL_DSI_PHY_DISABLE(hdsi);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}
#ifdef DSI_FIX_ME
/**
  * @brief  Refresh the display in command mode
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_Refresh(DSI_HandleTypeDef *hdsi)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Update the display */
    hdsi->Instance->WCR |= DSI_WCR_LTDCEN;

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}


/**
  * @brief  Controls the display color mode in Video mode
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  ColorMode  Color mode (full or 8-colors).
  *                    This parameter can be any value of @ref DSI_Color_Mode
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ColorMode(DSI_HandleTypeDef *hdsi, uint32_t ColorMode)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check the parameters */
    HAL_ASSERT(IS_DSI_COLOR_MODE(ColorMode));

    /* Update the display color mode */
    hdsi->Instance->WCR &= ~DSI_WCR_COLM;
    hdsi->Instance->WCR |= ColorMode;

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Control the display shutdown in Video mode
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  Shutdown  Shut-down (Display-ON or Display-OFF).
  *                   This parameter can be any value of @ref DSI_ShutDown
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_Shutdown(DSI_HandleTypeDef *hdsi, uint32_t Shutdown)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check the parameters */
    HAL_ASSERT(IS_DSI_SHUT_DOWN(Shutdown));

    /* Update the display Shutdown */
    hdsi->Instance->WCR &= ~DSI_WCR_SHTDN;
    hdsi->Instance->WCR |= Shutdown;

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}
#endif

HAL_StatusTypeDef HAL_DSI_SetPaticalSend(DSI_HandleTypeDef *hdsi,
        uint32_t write_size)
{
    /* Process locked */
    __HAL_LOCK(hdsi);


    //Write command size
    hdsi->Instance->DBI_CFG2 &= ~DSI_HOST_DBI_CFG2_WR_SIZE_Msk;
    hdsi->Instance->DBI_CFG2 |= (write_size << DSI_HOST_DBI_CFG2_WR_SIZE_Pos);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;

}


#ifndef SF32LB55X
HAL_StatusTypeDef HAL_DSI_DirectSend(DSI_HandleTypeDef *hdsi,
                                     uint32_t write_size)
{
    /* Process locked */
    __HAL_LOCK(hdsi);


    hdsi->Instance->DCOM_INTF_CFG1 = (10 << DSI_HOST_DCOM_INTF_CFG1_WAIT_CYCLE_Pos) | //Hpsys clcyes
                                     (210 << DSI_HOST_DCOM_INTF_CFG1_PKT_SIZE_Pos);   //Must <=512, large pkt size may enter LP mode

    hdsi->Instance->DCOM_INTF_CFG2 = ((write_size) << DSI_HOST_DCOM_INTF_CFG2_TOTAL_SIZE_Pos);

    // enable direct interface
    hdsi->Instance->DCOM_INTF_CTRL = (1 <<  DSI_HOST_DCOM_INTF_CTRL_START_Pos) |
                                     (0 << DSI_HOST_DCOM_INTF_CTRL_SW_RESET_Pos);


    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;

}


#endif /* !SF32LB55X */

/**
  * @brief  write short DCS or short Generic command
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  ChannelID  Virtual channel ID.
  * @param  Mode  DSI short packet data type.
  *               This parameter can be any value of @ref DSI_SHORT_WRITE_PKT_Data_Type.
  * @param  Param1  DSC command or first generic parameter.
  *                 This parameter can be any value of @ref DSI_DCS_Command or a
  *                 generic command code.
  * @param  Param2  DSC parameter or second generic parameter.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ShortWrite(DSI_HandleTypeDef *hdsi,
                                     uint32_t ChannelID,
                                     uint32_t Mode,
                                     uint32_t Param1,
                                     uint32_t Param2)
{
    uint32_t tickstart;

    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check the parameters */
    HAL_ASSERT(IS_DSI_SHORT_WRITE_PACKET_TYPE(Mode));
    DSI_LOG_D("HAL_DSI_ShortWrite INT_STAT2=%x\n", hdsi->Instance->INT_STAT2);
    DSI_LOG_D("PHY_STAT %x\n", hdsi->Instance->PHY_STAT);

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait for Command FIFO Empty */
    while ((hdsi->Instance->CMD_PKT_STAT & DSI_HOST_CMD_PKT_STAT_COM_CMD_EMPTY) == 0U)
    {
        /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
        {

            DSI_LOG_D("HAL_DSI_ShortWrite time out, cur=%d, %d\n", HAL_GetTick(), tickstart);

            /* Process Unlocked */
            __HAL_UNLOCK(hdsi);

            return HAL_TIMEOUT;
        }
    }

    DSI_LOG_D("HAL_DSI_ShortWrite %x, %x\n", Param1, Param2);

    /* Configure the packet to send a short DCS command with 0 or 1 parameter */
    DSI_ConfigPacketHeader(hdsi->Instance,
                           ChannelID,
                           Mode,
                           Param1,
                           Param2);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  write long DCS or long Generic command
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  ChannelID  Virtual channel ID.
  * @param  Mode  DSI long packet data type.
  *               This parameter can be any value of @ref DSI_LONG_WRITE_PKT_Data_Type.
  * @param  NbParams  Number of parameters.
  * @param  Param1  DSC command or first generic parameter.
  *                 This parameter can be any value of @ref DSI_DCS_Command or a
  *                 generic command code
  * @param  ParametersTable  Pointer to parameter values table.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_LongWrite(DSI_HandleTypeDef *hdsi,
                                    uint32_t ChannelID,
                                    uint32_t Mode,
                                    uint32_t NbParams,
                                    uint32_t Param1,
                                    uint8_t *ParametersTable)
{
    uint32_t uicounter, nbBytes, count;
    uint32_t tickstart;
    uint32_t fifoword;
    uint8_t *pparams = ParametersTable;

    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check the parameters */
    HAL_ASSERT(IS_DSI_LONG_WRITE_PACKET_TYPE(Mode));
    DSI_LOG_D("HAL_DSI_LongWrite INT_STAT2=%x\n", hdsi->Instance->INT_STAT2);
    DSI_LOG_D("PHY_STAT %x\n", hdsi->Instance->PHY_STAT);

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait for Command FIFO Empty */
    while ((hdsi->Instance->CMD_PKT_STAT & DSI_HOST_CMD_PKT_STAT_COM_CMD_EMPTY) == RESET)
    {
        /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
        {
            /* Process Unlocked */
            __HAL_UNLOCK(hdsi);

            return HAL_TIMEOUT;
        }
    }

    DSI_LOG_D("HAL_DSI_LongWrite %x\n", Param1);

    /* Set the DCS code on payload byte 1, and the other parameters on the write FIFO command*/
    fifoword = Param1;
    nbBytes = (NbParams < 3U) ? NbParams : 3U;

    for (count = 0U; count < nbBytes; count++)
    {
        fifoword |= (((uint32_t)(*(pparams + count))) << (8U + (8U * count)));
    }
    hdsi->Instance->COM_PLD = fifoword;

    uicounter = NbParams - nbBytes;
    pparams += nbBytes;
    /* Set the Next parameters on the write FIFO command*/
    while (uicounter != 0U)
    {
        nbBytes = (uicounter < 4U) ? uicounter : 4U;
        fifoword = 0U;
        for (count = 0U; count < nbBytes; count++)
        {
            fifoword |= (((uint32_t)(*(pparams + count))) << (8U * count));
        }
        hdsi->Instance->COM_PLD = fifoword;

        uicounter -= nbBytes;
        pparams += nbBytes;
    }

    /* Configure the packet to send a long DCS command */
    DSI_ConfigPacketHeader(hdsi->Instance,
                           ChannelID,
                           Mode,
                           ((NbParams + 1U) & 0x00FFU),
                           (((NbParams + 1U) & 0xFF00U) >> 8U));

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Read command (DCS or generic)
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  ChannelNbr  Virtual channel ID
  * @param  Array pointer to a buffer to store the payload of a read back operation.
  * @param  Size  Data size to be read (in byte).
  * @param  Mode  DSI read packet data type.
  *               This parameter can be any value of @ref DSI_SHORT_READ_PKT_Data_Type.
  * @param  DCSCmd  DCS get/read command.
  * @param  ParametersTable  Pointer to parameter values table.
  * @retval HAL status
*/
HAL_StatusTypeDef HAL_DSI_Read(DSI_HandleTypeDef *hdsi,
                               uint32_t ChannelNbr,
                               uint8_t *Array,
                               uint32_t Size,
                               uint32_t Mode,
                               uint32_t DCSCmd,
                               uint8_t *ParametersTable)
{
    uint32_t tickstart;
    uint8_t *pdata = Array;
    uint32_t tmp, datasize = Size;

    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check the parameters */
    HAL_ASSERT(IS_DSI_READ_PACKET_TYPE(Mode));

    if (Size > 64)
    {
        return HAL_ERROR;
    }

    DSI_LOG_D("HAL_DSI_Readx start  SIZE=%d\n", datasize);
    DSI_LOG_D("HAL_DSI_Read INT_STAT2=%x\n", hdsi->Instance->INT_STAT2);
    if (datasize > 2U)
    {
        /* set max return packet size */
        /* Get tick */
        tickstart = HAL_GetTick();

        /* Wait for Command FIFO Empty */
        while ((hdsi->Instance->CMD_PKT_STAT & DSI_HOST_CMD_PKT_STAT_COM_CMD_EMPTY) == 0U)
        {
            /* Check for the Timeout */
            if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
            {

                DSI_LOG_D("HAL_DSI_ShortWrite time out, cur=%d, %d\n", HAL_GetTick(), tickstart);

                /* Process Unlocked */
                __HAL_UNLOCK(hdsi);

                return HAL_TIMEOUT;
            }
        }

        /* Configure the packet to send a short DCS command with 0 or 1 parameter */
        DSI_ConfigPacketHeader(hdsi->Instance,
                               ChannelNbr,
                               DSI_MAX_RETURN_PKT_SIZE,
                               ((datasize) & 0xFFU),
                               (((datasize) >> 8U) & 0xFFU));
    }

    DSI_LOG_D("HAL_DSI_Read Mode=%x, cmd=%x\n", Mode, DCSCmd);
    DSI_LOG_D("HAL_DSI_Read INT_STAT2=%x\n", hdsi->Instance->INT_STAT2);
    DSI_LOG_D("HAL_DSI_Read CMODE_CFG=%x\n", hdsi->Instance->CMODE_CFG);


    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait for Command FIFO Empty */
    while ((hdsi->Instance->CMD_PKT_STAT & DSI_HOST_CMD_PKT_STAT_COM_CMD_EMPTY) == 0U)
    {
        /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
        {

            DSI_LOG_D("HAL_DSI_ShortWrite time out, cur=%d, %d\n", HAL_GetTick(), tickstart);

            /* Process Unlocked */
            __HAL_UNLOCK(hdsi);

            return HAL_TIMEOUT;
        }
    }

    /* Configure the packet to read command */
    if (Mode == DSI_DCS_SHORT_PKT_READ)
    {
        DSI_ConfigPacketHeader(hdsi->Instance, ChannelNbr, Mode, DCSCmd, 0U);
    }
    else if (Mode == DSI_GEN_SHORT_PKT_READ_P0)
    {
        DSI_ConfigPacketHeader(hdsi->Instance, ChannelNbr, Mode, 0U, 0U);
    }
    else if (Mode == DSI_GEN_SHORT_PKT_READ_P1)
    {
        DSI_ConfigPacketHeader(hdsi->Instance, ChannelNbr, Mode, ParametersTable[0U], 0U);
    }
    else if (Mode == DSI_GEN_SHORT_PKT_READ_P2)
    {
        DSI_ConfigPacketHeader(hdsi->Instance, ChannelNbr, Mode, ParametersTable[0U], ParametersTable[1U]);
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hdsi);

        return HAL_ERROR;
    }


#ifdef SHOW_BTA_LOG
    {
        /* Get tick */
        tickstart = HAL_GetTick();
        // enter receive mode
        uint32_t rdata = (hdsi->Instance->PHY_STAT & DSI_HOST_PHY_STAT_DIR_Msk);
        while (rdata == (0 << DSI_HOST_PHY_STAT_DIR_Pos))
        {
            rdata = (hdsi->Instance->PHY_STAT & DSI_HOST_PHY_STAT_DIR_Msk);
            /* Check for the Timeout */
            if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
            {
                /* Process Unlocked */
                __HAL_UNLOCK(hdsi);

                return HAL_TIMEOUT;
            }
        }

        DSI_LOG_D("enter receive mode\n");
        /* Get tick */
        tickstart = HAL_GetTick();

        // enter transmitt mode
        rdata = (hdsi->Instance->PHY_STAT & DSI_HOST_PHY_STAT_DIR_Msk);
        while (rdata == (1 << DSI_HOST_PHY_STAT_DIR_Pos))
        {
            rdata = (hdsi->Instance->PHY_STAT & DSI_HOST_PHY_STAT_DIR_Msk);
            /* Check for the Timeout */
            if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
            {
                /* Process Unlocked */
                __HAL_UNLOCK(hdsi);

                return HAL_TIMEOUT;
            }
        }

        DSI_LOG_D("enter transmit mode\n");
    }
#endif

    DSI_LOG_D("HAL_DSI_Read CMD_PKT_STAT=%x\n", hdsi->Instance->CMD_PKT_STAT);

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Check that the payload read FIFO is not empty and transmit is finish*/
    while (((hdsi->Instance->CMD_PKT_STAT & DSI_HOST_CMD_PKT_STAT_COM_RDPLD_EMPTY) == DSI_HOST_CMD_PKT_STAT_COM_RDPLD_EMPTY)
            || ((1 << DSI_HOST_PHY_STAT_DIR_Pos) == (hdsi->Instance->PHY_STAT & DSI_HOST_PHY_STAT_DIR_Msk))
          )
    {
        /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
        {
            DSI_LOG_D("TIMEOUT CMD_PKT_STAT=%x, PHY_STAT=%x\n", hdsi->Instance->CMD_PKT_STAT, hdsi->Instance->PHY_STAT);
            /* Process Unlocked */
            __HAL_UNLOCK(hdsi);

            return HAL_TIMEOUT;
        }
    }


    /* Read payload after transmit done. */
    do
    {
        tmp = hdsi->Instance->COM_PLD;
        for (uint8_t i = 0; (i < 4) && (datasize > 0); i++)
        {
            *pdata = (uint8_t)((tmp >> (i * 8)) & 0xff);
            DSI_LOG_D("HAL_DSI_Read byte=%x\n", *pdata);
            pdata++;
            datasize--;
        }

    }
    while (datasize > 0);


#ifdef SHOW_BTA_LOG
    /* Get tick */
    tickstart = HAL_GetTick();

    /* Get the remaining bytes if any */
    while (((int)(datasize)) > 0)
    {
        if ((hdsi->Instance->CMD_PKT_STAT & DSI_HOST_CMD_PKT_STAT_COM_RDPLD_EMPTY) == 0U)
        {
            *((uint32_t *)pdata) = (hdsi->Instance->COM_PLD);
            DSI_LOG_D("HAL_DSI_Read remaining byte=%x\n", *((uint32_t *)pdata));
            datasize -= 4U;
            pdata += 4U;
        }

        /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
        {
            /* Process Unlocked */
            __HAL_UNLOCK(hdsi);

            return HAL_TIMEOUT;
        }
    }
#endif
    /* Process unlocked */
    __HAL_UNLOCK(hdsi);
    DSI_LOG_D("HAL_DSI_Read end\n");

    return HAL_OK;
}

/**
  * @brief  Enter the ULPM (Ultra Low Power Mode) with the D-PHY PLL running
  *         (only data lanes are in ULPM)
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_EnterULPMData(DSI_HandleTypeDef *hdsi)
{
    uint32_t tickstart;

    /* Process locked */
    __HAL_LOCK(hdsi);

    /* ULPS Request on Data Lanes */
    hdsi->Instance->PHY_CTRL |= DSI_HOST_PHY_CTRL_DLANE_REQULPS;

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait until the D-PHY active lanes enter into ULPM */
    if ((hdsi->Instance->PHY_CFG1 & DSI_HOST_PHY_CFG1_NUM_ACT_LANE) == DSI_ONE_DATA_LANE)
    {
        while ((hdsi->Instance->PHY_STAT & DSI_HOST_PHY_STAT_DLANE0_ULPS_N) != RESET)
        {
            /* Check for the Timeout */
            if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
            {
                /* Process Unlocked */
                __HAL_UNLOCK(hdsi);

                return HAL_TIMEOUT;
            }
        }
    }
    else if ((hdsi->Instance->PHY_CFG1 & DSI_HOST_PHY_CFG1_NUM_ACT_LANE) == DSI_TWO_DATA_LANES)
    {
        while ((hdsi->Instance->PHY_STAT & (DSI_HOST_PHY_STAT_DLANE0_ULPS_N | DSI_HOST_PHY_STAT_DLANE1_ULPS_N)) != RESET)
        {
            /* Check for the Timeout */
            if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
            {
                /* Process Unlocked */
                __HAL_UNLOCK(hdsi);

                return HAL_TIMEOUT;
            }
        }
    }
    else
    {
        /* Process unlocked */
        __HAL_UNLOCK(hdsi);

        return HAL_ERROR;
    }

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Exit the ULPM (Ultra Low Power Mode) with the D-PHY PLL running
  *         (only data lanes are in ULPM)
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ExitULPMData(DSI_HandleTypeDef *hdsi)
{
    uint32_t tickstart;

    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Exit ULPS on Data Lanes */
    hdsi->Instance->PHY_CTRL |= DSI_HOST_PHY_CTRL_DLANE_EXITULPS;

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait until all active lanes exit ULPM */
    if ((hdsi->Instance->PHY_CFG1 & DSI_HOST_PHY_CFG1_NUM_ACT_LANE) == DSI_ONE_DATA_LANE)
    {
        while ((hdsi->Instance->PHY_STAT & DSI_HOST_PHY_STAT_DLANE0_ULPS_N) != DSI_HOST_PHY_STAT_DLANE0_ULPS_N)
        {
            /* Check for the Timeout */
            if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
            {
                /* Process Unlocked */
                __HAL_UNLOCK(hdsi);

                return HAL_TIMEOUT;
            }
        }
    }
    else if ((hdsi->Instance->PHY_CFG1 & DSI_HOST_PHY_CFG1_NUM_ACT_LANE) == DSI_TWO_DATA_LANES)
    {
        while ((hdsi->Instance->PHY_STAT & (DSI_HOST_PHY_STAT_DLANE0_ULPS_N | DSI_HOST_PHY_STAT_DLANE1_ULPS_N)) != (DSI_HOST_PHY_STAT_DLANE0_ULPS_N | DSI_HOST_PHY_STAT_DLANE1_ULPS_N))
        {
            /* Check for the Timeout */
            if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
            {
                /* Process Unlocked */
                __HAL_UNLOCK(hdsi);

                return HAL_TIMEOUT;
            }
        }
    }
    else
    {
        /* Process unlocked */
        __HAL_UNLOCK(hdsi);

        return HAL_ERROR;
    }

    /* wait for 1 ms*/
    HAL_Delay(1U);

    /* De-assert the ULPM requests and the ULPM exit bits */
    hdsi->Instance->PHY_CTRL &= ~(DSI_HOST_PHY_CTRL_DLANE_REQULPS | DSI_HOST_PHY_CTRL_DLANE_EXITULPS | DSI_HOST_PHY_CTRL_CKLANE_REQULPS | DSI_HOST_PHY_CTRL_CKLANE_EXITULPS);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Enter the ULPM (Ultra Low Power Mode) with the D-PHY PLL turned off
  *         (both data and clock lanes are in ULPM)
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_EnterULPM(DSI_HandleTypeDef *hdsi)
{
    uint32_t tickstart;
    HAL_StatusTypeDef ret_v = HAL_OK;

    if (HAL_DSI_IS_VIDEO_MODE(hdsi)) return HAL_OK;

    DSI_LOG_D("HAL_DSI_EnterULPM \n");
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Clock lane configuration: no more HS request */
    hdsi->Instance->PHY_CTRL &= ~DSI_HOST_PHY_CTRL_CKLANE_HSTXREQ;

    /* Use system PLL as byte lane clock source before stopping DSIPHY clock source */
    //__HAL_RCC_DSI_CONFIG(RCC_DSICLKSOURCE_PLLR);
    //hdsi->Instance->PHY_CTRL &= ~(DSI_HOST_PHY_CTRL_CKLANE_AUTO_CTRL);
    HAL_ASSERT(DSI_HOST_PHY_CTRL_CKLANE_AUTO_CTRL == (hdsi->Instance->PHY_CTRL & DSI_HOST_PHY_CTRL_CKLANE_AUTO_CTRL)); //Make sure CLK lane not during HS mode

    hdsi->Instance->PHY_CTRL &= ~(DSI_HOST_PHY_CTRL_CKLANE_AUTO_CTRL);

    /* Get tick */
    tickstart = HAL_GetTick();

    /*Wait clock lane enter LP mode*/
    while (!__HAL_DSI_IS_CKLANE_STOP(hdsi))
    {
        /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
        {
            ret_v =  HAL_TIMEOUT;
            goto _ERR_HANDLE;
        }
    }

    /* ULPS Request on Clock and Data Lanes */
    hdsi->Instance->PHY_CTRL |= (DSI_HOST_PHY_CTRL_CKLANE_REQULPS | DSI_HOST_PHY_CTRL_DLANE_REQULPS);


    /* Wait until all active lanes exit ULPM */
    if ((hdsi->Instance->PHY_CFG1 & DSI_HOST_PHY_CFG1_NUM_ACT_LANE) == DSI_ONE_DATA_LANE)
    {
        while ((hdsi->Instance->PHY_STAT & (DSI_HOST_PHY_STAT_DLANE0_ULPS_N | DSI_HOST_PHY_STAT_CKLANE_ULPS_N)) != RESET)
        {
            /* Check for the Timeout */
            if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
            {
                ret_v =  HAL_TIMEOUT;
                goto _ERR_HANDLE;
            }
        }
    }
    else if ((hdsi->Instance->PHY_CFG1 & DSI_HOST_PHY_CFG1_NUM_ACT_LANE) == DSI_TWO_DATA_LANES)
    {
        while ((hdsi->Instance->PHY_STAT & (DSI_HOST_PHY_STAT_DLANE0_ULPS_N | DSI_HOST_PHY_STAT_DLANE1_ULPS_N | DSI_HOST_PHY_STAT_CKLANE_ULPS_N)) != RESET)
        {
            /* Check for the Timeout */
            if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
            {
                ret_v =  HAL_TIMEOUT;
                goto _ERR_HANDLE;
            }
        }
    }
    else
    {
        ret_v =  HAL_ERROR;
        goto _ERR_HANDLE;
    }

    /* Turn off the DSI PLL */
    __HAL_DSI_PLL_DISABLE(hdsi);

_ERR_HANDLE:
    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    DSI_LOG_D("HAL_DSI_EnterULPM done.\n");

    return ret_v;
}

/**
  * @brief  Exit the ULPM (Ultra Low Power Mode) with the D-PHY PLL turned off
  *         (both data and clock lanes are in ULPM)
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ExitULPM(DSI_HandleTypeDef *hdsi)
{
    uint32_t tickstart;

    if (HAL_DSI_IS_VIDEO_MODE(hdsi)) return HAL_OK;

    DSI_LOG_D("HAL_DSI_ExitULPM \n");

    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Turn on the DSI PLL */
    __HAL_DSI_PLL_ENABLE(hdsi);

    /* PLL ready need 10~20us, wait for 50 us*/
    HAL_Delay_us(50U);


    /* Exit ULPS on Clock and Data Lanes */
    hdsi->Instance->PHY_CTRL |= (DSI_HOST_PHY_CTRL_CKLANE_EXITULPS | DSI_HOST_PHY_CTRL_DLANE_EXITULPS);

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait until all active lanes exit ULPM */
    if ((hdsi->Instance->PHY_CFG1 & DSI_HOST_PHY_CFG1_NUM_ACT_LANE) == DSI_ONE_DATA_LANE)
    {
        while ((hdsi->Instance->PHY_STAT & (DSI_HOST_PHY_STAT_DLANE0_ULPS_N | DSI_HOST_PHY_STAT_CKLANE_ULPS_N)) != (DSI_HOST_PHY_STAT_DLANE0_ULPS_N | DSI_HOST_PHY_STAT_CKLANE_ULPS_N))
        {
            /* Check for the Timeout */
            if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
            {
                /* Process Unlocked */
                __HAL_UNLOCK(hdsi);

                return HAL_TIMEOUT;
            }
        }
    }
    else if ((hdsi->Instance->PHY_CFG1 & DSI_HOST_PHY_CFG1_NUM_ACT_LANE) == DSI_TWO_DATA_LANES)
    {
        while ((hdsi->Instance->PHY_STAT & (DSI_HOST_PHY_STAT_DLANE0_ULPS_N | DSI_HOST_PHY_STAT_DLANE1_ULPS_N | DSI_HOST_PHY_STAT_CKLANE_ULPS_N)) != (DSI_HOST_PHY_STAT_DLANE0_ULPS_N | DSI_HOST_PHY_STAT_DLANE1_ULPS_N | DSI_HOST_PHY_STAT_CKLANE_ULPS_N))
        {
            /* Check for the Timeout */
            if ((HAL_GetTick() - tickstart) > DSI_TIMEOUT_VALUE)
            {
                /* Process Unlocked */
                __HAL_UNLOCK(hdsi);

                return HAL_TIMEOUT;
            }
        }
    }
    else
    {
        /* Process unlocked */
        __HAL_UNLOCK(hdsi);

        return HAL_ERROR;
    }

    /* wait for 1 ms */
    HAL_Delay_us(50U);

    /* De-assert the ULPM requests and the ULPM exit bits */
    hdsi->Instance->PHY_CTRL &= ~(DSI_HOST_PHY_CTRL_DLANE_REQULPS | DSI_HOST_PHY_CTRL_DLANE_EXITULPS | DSI_HOST_PHY_CTRL_CKLANE_REQULPS | DSI_HOST_PHY_CTRL_CKLANE_EXITULPS);

    /* Switch the lanbyteclock source in the RCC from system PLL to D-PHY */
    //__HAL_RCC_DSI_CONFIG(RCC_DSICLKSOURCE_DSIPHY);

    /* Restore clock lane configuration to HS */
    //hdsi->Instance->PHY_CTRL |= DSI_HOST_PHY_CTRL_CKLANE_HSTXREQ;

    hdsi->Instance->PHY_CTRL |= DSI_HOST_PHY_CTRL_CKLANE_HSTXREQ | hdsi->Init.AutomaticClockLaneControl; //Revert clk lane setting
    //HAL_Delay_us(50U);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    DSI_LOG_D("HAL_DSI_ExitULPM done.\n");

    return HAL_OK;
}


HAL_StatusTypeDef HAL_DSI_EnterHSM(DSI_HandleTypeDef *hdsi)
{

    return HAL_OK;
}


HAL_StatusTypeDef HAL_DSI_ExitHSM(DSI_HandleTypeDef *hdsi)
{
    return HAL_OK;
}

HAL_StatusTypeDef HAL_DSI_EnterCKLaneHSM(DSI_HandleTypeDef *hdsi)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    hdsi->Instance->PHY_CTRL &= ~(DSI_HOST_PHY_CTRL_CKLANE_AUTO_CTRL);
    hdsi->Instance->PHY_CTRL |= DSI_HOST_PHY_CTRL_CKLANE_HSTXREQ;

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);


    return HAL_OK;
}


HAL_StatusTypeDef HAL_DSI_ExitCKLaneHSM(DSI_HandleTypeDef *hdsi)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    hdsi->Instance->PHY_CTRL &= ~(DSI_HOST_PHY_CTRL_CKLANE_AUTO_CTRL);
    hdsi->Instance->PHY_CTRL |= (DSI_HOST_PHY_CTRL_CKLANE_HSTXREQ | hdsi->Init.AutomaticClockLaneControl);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);


    return HAL_OK;
}

uint8_t HAL_DSI_IsBusy(DSI_HandleTypeDef *hdsi)
{
    if (!__HAL_DSI_IS_WRITE_FINISH(hdsi)) return 1;
    if (!__HAL_DSI_IS_MASTER_STATE(hdsi)) return 1;

    if (0 == (hdsi->Instance->PHY_CTRL & DSI_HOST_PHY_CTRL_DLANE_REQULPS))
    {
        if (!__HAL_DSI_IS_DLANE0_STOP(hdsi))  return 1;
    }


    return 0;
}


#ifdef DSI_FIX_ME  //PHY timing

/**
  * @brief  Set Slew-Rate And Delay Tuning
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  CommDelay  Communication delay to be adjusted.
  *                    This parameter can be any value of @ref DSI_Communication_Delay
  * @param  Lane  select between clock or data lanes.
  *               This parameter can be any value of @ref DSI_Lane_Group
  * @param  Value  Custom value of the slew-rate or delay
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_SetSlewRateAndDelayTuning(DSI_HandleTypeDef *hdsi, uint32_t CommDelay, uint32_t Lane, uint32_t Value)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check function parameters */
    HAL_ASSERT(IS_DSI_COMMUNICATION_DELAY(CommDelay));
    HAL_ASSERT(IS_DSI_LANE_GROUP(Lane));

    switch (CommDelay)
    {
    case DSI_SLEW_RATE_HSTX:
        if (Lane == DSI_CLOCK_LANE)
        {
            /* High-Speed Transmission Slew Rate Control on Clock Lane */
            hdsi->Instance->WPCR[1U] &= ~DSI_WPCR1_HSTXSRCCL;
            hdsi->Instance->WPCR[1U] |= Value << 16U;
        }
        else if (Lane == DSI_DATA_LANES)
        {
            /* High-Speed Transmission Slew Rate Control on Data Lanes */
            hdsi->Instance->WPCR[1U] &= ~DSI_WPCR1_HSTXSRCDL;
            hdsi->Instance->WPCR[1U] |= Value << 18U;
        }
        else
        {
            /* Process unlocked */
            __HAL_UNLOCK(hdsi);

            return HAL_ERROR;
        }
        break;
    case DSI_SLEW_RATE_LPTX:
        if (Lane == DSI_CLOCK_LANE)
        {
            /* Low-Power transmission Slew Rate Compensation on Clock Lane */
            hdsi->Instance->WPCR[1U] &= ~DSI_WPCR1_LPSRCCL;
            hdsi->Instance->WPCR[1U] |= Value << 6U;
        }
        else if (Lane == DSI_DATA_LANES)
        {
            /* Low-Power transmission Slew Rate Compensation on Data Lanes */
            hdsi->Instance->WPCR[1U] &= ~DSI_WPCR1_LPSRCDL;
            hdsi->Instance->WPCR[1U] |= Value << 8U;
        }
        else
        {
            /* Process unlocked */
            __HAL_UNLOCK(hdsi);

            return HAL_ERROR;
        }
        break;
    case DSI_HS_DELAY:
        if (Lane == DSI_CLOCK_LANE)
        {
            /* High-Speed Transmission Delay on Clock Lane */
            hdsi->Instance->WPCR[1U] &= ~DSI_WPCR1_HSTXDCL;
            hdsi->Instance->WPCR[1U] |= Value;
        }
        else if (Lane == DSI_DATA_LANES)
        {
            /* High-Speed Transmission Delay on Data Lanes */
            hdsi->Instance->WPCR[1U] &= ~DSI_WPCR1_HSTXDDL;
            hdsi->Instance->WPCR[1U] |= Value << 2U;
        }
        else
        {
            /* Process unlocked */
            __HAL_UNLOCK(hdsi);

            return HAL_ERROR;
        }
        break;
    default:
        break;
    }

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Low-Power Reception Filter Tuning
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  Frequency  cutoff frequency of low-pass filter at the input of LPRX
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_SetLowPowerRXFilter(DSI_HandleTypeDef *hdsi, uint32_t Frequency)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Low-Power RX low-pass Filtering Tuning */
    hdsi->Instance->WPCR[1U] &= ~DSI_WPCR1_LPRXFT;
    hdsi->Instance->WPCR[1U] |= Frequency << 25U;

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Activate an additional current path on all lanes to meet the SDDTx parameter
  *         defined in the MIPI D-PHY specification
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  State  ENABLE or DISABLE
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_SetSDD(DSI_HandleTypeDef *hdsi, FunctionalState State)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check function parameters */
    HAL_ASSERT(IS_FUNCTIONAL_STATE(State));

    /* Activate/Disactivate additional current path on all lanes */
    hdsi->Instance->WPCR[1U] &= ~DSI_WPCR1_SDDC;
    hdsi->Instance->WPCR[1U] |= ((uint32_t)State << 12U);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}
#endif /* DSI_FIX_ME */

/**
  * @brief  Custom lane pins configuration
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  CustomLane  Function to be applyed on selected lane.
  *                     This parameter can be any value of @ref DSI_CustomLane
  * @param  Lane  select between clock or data lane 0 or data lane 1.
  *               This parameter can be any value of @ref DSI_Lane_Select
  * @param  State  ENABLE or DISABLE
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_SetLanePinsConfiguration(DSI_HandleTypeDef *hdsi, uint32_t CustomLane, uint32_t Lane, FunctionalState State)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check function parameters */
    HAL_ASSERT(IS_DSI_CUSTOM_LANE(CustomLane));
    HAL_ASSERT(IS_DSI_LANE(Lane));
    HAL_ASSERT(IS_FUNCTIONAL_STATE(State));

    switch (CustomLane)
    {
    case DSI_SWAP_LANE_PINS:
        if (Lane == DSI_CLK_LANE)
        {
            /* Swap pins on clock lane */
            hdsi->PHY_Instance->CK_LANE_CTRL &= ~DSI_PHY_CK_LANE_CTRL_PN_SWAP;
            hdsi->PHY_Instance->CK_LANE_CTRL |= ((uint32_t)State << DSI_PHY_CK_LANE_CTRL_PN_SWAP_Pos);
        }
        else if (Lane == DSI_DATA_LANE0)
        {
            /* Swap pins on data lane 0 */
            hdsi->PHY_Instance->D_LANE0_CTRL &= ~DSI_PHY_D_LANE0_CTRL_PN_SWAP;
            hdsi->PHY_Instance->D_LANE0_CTRL |= ((uint32_t)State << DSI_PHY_D_LANE0_CTRL_PN_SWAP_Pos);
        }
        else if (Lane == DSI_DATA_LANE1)
        {
            /* Swap pins on data lane 1 */
            hdsi->PHY_Instance->D_LANE1_CTRL &= ~DSI_PHY_D_LANE1_CTRL_PN_SWAP;
            hdsi->PHY_Instance->D_LANE1_CTRL |= ((uint32_t)State << DSI_PHY_D_LANE1_CTRL_PN_SWAP_Pos);
        }
        else
        {
            /* Process unlocked */
            __HAL_UNLOCK(hdsi);

            return HAL_ERROR;
        }
        break;
#ifdef DSI_FIX_ME  //NOT suppoert  HS invert?
    case DSI_INVERT_HS_SIGNAL:
        if (Lane == DSI_CLK_LANE)
        {
            /* Invert HS signal on clock lane */
            hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_HSICL;
            hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 9U);
        }
        else if (Lane == DSI_DATA_LANE0)
        {
            /* Invert HS signal on data lane 0 */
            hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_HSIDL0;
            hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 10U);
        }
        else if (Lane == DSI_DATA_LANE1)
        {
            /* Invert HS signal on data lane 1 */
            hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_HSIDL1;
            hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 11U);
        }
        else
        {
            /* Process unlocked */
            __HAL_UNLOCK(hdsi);

            return HAL_ERROR;
        }
        break;
#endif
    default:
        break;
    }

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}
#ifdef  DSI_FIX_ME   //PHY TIMING

/**
  * @brief  Set custom timing for the PHY
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  Timing  PHY timing to be adjusted.
  *                 This parameter can be any value of @ref DSI_PHY_Timing
  * @param  State  ENABLE or DISABLE
  * @param  Value  Custom value of the timing
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_SetPHYTimings(DSI_HandleTypeDef *hdsi, uint32_t Timing, FunctionalState State, uint32_t Value)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check function parameters */
    HAL_ASSERT(IS_DSI_PHY_TIMING(Timing));
    HAL_ASSERT(IS_FUNCTIONAL_STATE(State));

    switch (Timing)
    {
    case DSI_TCLK_POST:
        /* Enable/Disable custom timing setting */
        hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_TCLKPOSTEN;
        hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 27U);

        if (State != DISABLE)
        {
            /* Set custom value */
            hdsi->Instance->WPCR[4U] &= ~DSI_WPCR4_TCLKPOST;
            hdsi->Instance->WPCR[4U] |= Value & DSI_WPCR4_TCLKPOST;
        }

        break;
    case DSI_TLPX_CLK:
        /* Enable/Disable custom timing setting */
        hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_TLPXCEN;
        hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 26U);

        if (State != DISABLE)
        {
            /* Set custom value */
            hdsi->Instance->WPCR[3U] &= ~DSI_WPCR3_TLPXC;
            hdsi->Instance->WPCR[3U] |= (Value << 24U) & DSI_WPCR3_TLPXC;
        }

        break;
    case DSI_THS_EXIT:
        /* Enable/Disable custom timing setting */
        hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_THSEXITEN;
        hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 25U);

        if (State != DISABLE)
        {
            /* Set custom value */
            hdsi->Instance->WPCR[3U] &= ~DSI_WPCR3_THSEXIT;
            hdsi->Instance->WPCR[3U] |= (Value << 16U) & DSI_WPCR3_THSEXIT;
        }

        break;
    case DSI_TLPX_DATA:
        /* Enable/Disable custom timing setting */
        hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_TLPXDEN;
        hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 24U);

        if (State != DISABLE)
        {
            /* Set custom value */
            hdsi->Instance->WPCR[3U] &= ~DSI_WPCR3_TLPXD;
            hdsi->Instance->WPCR[3U] |= (Value << 8U) & DSI_WPCR3_TLPXD;
        }

        break;
    case DSI_THS_ZERO:
        /* Enable/Disable custom timing setting */
        hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_THSZEROEN;
        hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 23U);

        if (State != DISABLE)
        {
            /* Set custom value */
            hdsi->Instance->WPCR[3U] &= ~DSI_WPCR3_THSZERO;
            hdsi->Instance->WPCR[3U] |= Value & DSI_WPCR3_THSZERO;
        }

        break;
    case DSI_THS_TRAIL:
        /* Enable/Disable custom timing setting */
        hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_THSTRAILEN;
        hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 22U);

        if (State != DISABLE)
        {
            /* Set custom value */
            hdsi->Instance->WPCR[2U] &= ~DSI_WPCR2_THSTRAIL;
            hdsi->Instance->WPCR[2U] |= (Value << 24U) & DSI_WPCR2_THSTRAIL;
        }

        break;
    case DSI_THS_PREPARE:
        /* Enable/Disable custom timing setting */
        hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_THSPREPEN;
        hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 21U);

        if (State != DISABLE)
        {
            /* Set custom value */
            hdsi->Instance->WPCR[2U] &= ~DSI_WPCR2_THSPREP;
            hdsi->Instance->WPCR[2U] |= (Value << 16U) & DSI_WPCR2_THSPREP;
        }

        break;
    case DSI_TCLK_ZERO:
        /* Enable/Disable custom timing setting */
        hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_TCLKZEROEN;
        hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 20U);

        if (State != DISABLE)
        {
            /* Set custom value */
            hdsi->Instance->WPCR[2U] &= ~DSI_WPCR2_TCLKZERO;
            hdsi->Instance->WPCR[2U] |= (Value << 8U) & DSI_WPCR2_TCLKZERO;
        }

        break;
    case DSI_TCLK_PREPARE:
        /* Enable/Disable custom timing setting */
        hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_TCLKPREPEN;
        hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 19U);

        if (State != DISABLE)
        {
            /* Set custom value */
            hdsi->Instance->WPCR[2U] &= ~DSI_WPCR2_TCLKPREP;
            hdsi->Instance->WPCR[2U] |= Value & DSI_WPCR2_TCLKPREP;
        }

        break;
    default:
        break;
    }

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Force the Clock/Data Lane in TX Stop Mode
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  Lane  select between clock or data lanes.
  *               This parameter can be any value of @ref DSI_Lane_Group
  * @param  State  ENABLE or DISABLE
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ForceTXStopMode(DSI_HandleTypeDef *hdsi, uint32_t Lane, FunctionalState State)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check function parameters */
    HAL_ASSERT(IS_DSI_LANE_GROUP(Lane));
    HAL_ASSERT(IS_FUNCTIONAL_STATE(State));

    if (Lane == DSI_CLOCK_LANE)
    {
        /* Force/Unforce the Clock Lane in TX Stop Mode */
        hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_FTXSMCL;
        hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 12U);
    }
    else if (Lane == DSI_DATA_LANES)
    {
        /* Force/Unforce the Data Lanes in TX Stop Mode */
        hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_FTXSMDL;
        hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 13U);
    }
    else
    {
        /* Process unlocked */
        __HAL_UNLOCK(hdsi);

        return HAL_ERROR;
    }

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Force LP Receiver in Low-Power Mode
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  State  ENABLE or DISABLE
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ForceRXLowPower(DSI_HandleTypeDef *hdsi, FunctionalState State)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check function parameters */
    HAL_ASSERT(IS_FUNCTIONAL_STATE(State));

    /* Force/Unforce LP Receiver in Low-Power Mode */
    hdsi->Instance->WPCR[1U] &= ~DSI_WPCR1_FLPRXLPM;
    hdsi->Instance->WPCR[1U] |= ((uint32_t)State << 22U);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Force Data Lanes in RX Mode after a BTA
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  State  ENABLE or DISABLE
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_ForceDataLanesInRX(DSI_HandleTypeDef *hdsi, FunctionalState State)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check function parameters */
    HAL_ASSERT(IS_FUNCTIONAL_STATE(State));

    /* Force Data Lanes in RX Mode */
    hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_TDDL;
    hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 16U);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @brief  Enable a pull-down on the lanes to prevent from floating states when unused
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  State  ENABLE or DISABLE
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_SetPullDown(DSI_HandleTypeDef *hdsi, FunctionalState State)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check function parameters */
    HAL_ASSERT(IS_FUNCTIONAL_STATE(State));

    /* Enable/Disable pull-down on lanes */
    hdsi->Instance->WPCR[0U] &= ~DSI_WPCR0_PDEN;
    hdsi->Instance->WPCR[0U] |= ((uint32_t)State << 18U);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}
#endif

/**
  * @brief  Switch off the contention detection on data lanes
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @param  State  ENABLE or DISABLE
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DSI_SetContentionDetectionOff(DSI_HandleTypeDef *hdsi, FunctionalState State)
{
    /* Process locked */
    __HAL_LOCK(hdsi);

    /* Check function parameters */
    HAL_ASSERT(IS_FUNCTIONAL_STATE(State));

    /* Contention Detection on Data Lanes OFF */
    hdsi->PHY_Instance->D_LANE0_CTRL &= ~DSI_PHY_D_LANE0_CTRL_CONTENTION_DET_EN;
    hdsi->PHY_Instance->D_LANE0_CTRL |= ((uint32_t)State << DSI_PHY_D_LANE0_CTRL_CONTENTION_DET_EN_Pos);

    /* Process unlocked */
    __HAL_UNLOCK(hdsi);

    return HAL_OK;
}

/**
  * @}
  */

/** @defgroup DSI_Group4 Peripheral State and Errors functions
 *  @brief    Peripheral State and Errors functions
 *
@verbatim
 ===============================================================================
                  ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DSI state.
      (+) Get error code.

@endverbatim
  * @{
  */

/**
  * @brief  Return the DSI state
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval HAL state
  */
HAL_DSI_StateTypeDef HAL_DSI_GetState(DSI_HandleTypeDef *hdsi)
{
    return hdsi->State;
}

/**
  * @brief  Return the DSI error code
  * @param  hdsi  pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval DSI Error Code
  */
uint32_t HAL_DSI_GetError(DSI_HandleTypeDef *hdsi)
{
    /* Get the error code */
    return hdsi->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_DSI_MODULE_ENABLED */
/**
  * @}
  */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

