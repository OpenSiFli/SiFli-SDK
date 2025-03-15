/**
  ******************************************************************************
  * @file   bf0_hal_pdm.c
  * @author Sifli software development team
  * @brief   PDM HAL module driver.
  *          This file provides firmware functions to manage the following
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

#include "bf0_hal.h"


#if defined(HAL_PDM_MODULE_ENABLED) ||defined(_SIFLI_DOXYGEN_)

static void PDM_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void PDM_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma);
static void PDM_DMAError(DMA_HandleTypeDef *hdma);
static HAL_StatusTypeDef PDM_Config(PDM_HandleTypeDef *hpdm, PDM_ConfigureTypeDef type);
static HAL_StatusTypeDef PDM_ConfigDMA(PDM_HandleTypeDef *hpdm);



HAL_StatusTypeDef HAL_PDM_Init(PDM_HandleTypeDef *hpdm)
{

    if (hpdm == NULL)
    {
        return HAL_ERROR;
    }

    if (hpdm->State == PDM_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hpdm->Lock = HAL_UNLOCKED;

        /* Init the low level hardware : GPIO, CLOCK, NVIC... */
        HAL_PDM_MspInit(hpdm);
    }

    hpdm->State = PDM_STATE_BUSY;

    if (hpdm->Instance == hwp_pdm1)
        HAL_RCC_ResetModule(RCC_MOD_PDM1);
#ifdef hwp_pdm2
    else if (hpdm->Instance == hwp_pdm2)
        HAL_RCC_ResetModule(RCC_MOD_PDM2);
#endif

    __HAL_PDM_DISABLE(hpdm);

    PDM_Config(hpdm, PDM_CFG_CHANNEL | PDM_CFG_SAMPLERATE | PDM_CFG_DEPTH);


#if 0
    hpdm->Instance->COMP_CFG0 |= PDM_COMP_CFG0_COMP_BYPASS;
    hpdm->Instance->COMP_CFG0 &= ~PDM_COMP_CFG0_COMP_COEFF0;
    hpdm->Instance->COMP_CFG0 |= (0x15c << PDM_COMP_CFG0_COMP_COEFF0_Pos);
    hpdm->Instance->COMP_CFG1 &= ~PDM_COMP_CFG1_COMP_COEFF1;
    hpdm->Instance->COMP_CFG1 |= (0xb31 << PDM_COMP_CFG1_COMP_COEFF1_Pos);
    hpdm->Instance->COMP_CFG1 &= ~PDM_COMP_CFG1_COMP_COEFF2;
    hpdm->Instance->COMP_CFG1 |= (0x57e << PDM_COMP_CFG1_COMP_COEFF2_Pos);
#endif

    hpdm->Instance->HPF_CFG &= ~PDM_HPF_CFG_HPF_BYPASS;
    hpdm->Instance->HPF_CFG &= ~PDM_HPF_CFG_HPF_COEFF;
    hpdm->Instance->HPF_CFG |= (0x7 << PDM_HPF_CFG_HPF_COEFF_Pos);

    hpdm->Instance->PGA_CFG &= ~PDM_PGA_CFG_PGA_GAIN_L;
    hpdm->Instance->PGA_CFG |= (0x26 << PDM_PGA_CFG_PGA_GAIN_L_Pos);
    hpdm->Instance->PGA_CFG &= ~PDM_PGA_CFG_PGA_GAIN_R;
    hpdm->Instance->PGA_CFG |= (0x26 << PDM_PGA_CFG_PGA_GAIN_R_Pos);

#if 0
    hpdm->Instance->LPF_CFG0 &= ~PDM_LPF_CFG0_LPF_COEFF00;
    hpdm->Instance->LPF_CFG0 |= (0xffa << PDM_LPF_CFG0_LPF_COEFF00_Pos);
    hpdm->Instance->LPF_CFG0 &= ~PDM_LPF_CFG0_LPF_COEFF01;
    hpdm->Instance->LPF_CFG0 |= (0x005 << PDM_LPF_CFG0_LPF_COEFF01_Pos);

    hpdm->Instance->LPF_CFG1 &= ~PDM_LPF_CFG1_LPF_COEFF02;
    hpdm->Instance->LPF_CFG1 |= (0x028 << PDM_LPF_CFG1_LPF_COEFF02_Pos);
    hpdm->Instance->LPF_CFG1 &= ~PDM_LPF_CFG1_LPF_COEFF03;
    hpdm->Instance->LPF_CFG1 |= (0x01f << PDM_LPF_CFG1_LPF_COEFF03_Pos);

    hpdm->Instance->LPF_CFG2 &= ~PDM_LPF_CFG2_LPF_COEFF04;
    hpdm->Instance->LPF_CFG2 |= (0xfc4 << PDM_LPF_CFG2_LPF_COEFF04_Pos);
    hpdm->Instance->LPF_CFG2 &= ~PDM_LPF_CFG2_LPF_COEFF05;
    hpdm->Instance->LPF_CFG2 |= (0xfaa << PDM_LPF_CFG2_LPF_COEFF05_Pos);

    hpdm->Instance->LPF_CFG3 &= ~PDM_LPF_CFG3_LPF_COEFF06;
    hpdm->Instance->LPF_CFG3 |= (0x050 << PDM_LPF_CFG3_LPF_COEFF06_Pos);
    hpdm->Instance->LPF_CFG3 &= ~PDM_LPF_CFG3_LPF_COEFF07;
    hpdm->Instance->LPF_CFG3 |= (0x0b5 << PDM_LPF_CFG3_LPF_COEFF07_Pos);

    hpdm->Instance->LPF_CFG4 &= ~PDM_LPF_CFG4_LPF_COEFF08;
    hpdm->Instance->LPF_CFG4 |= (0xfa0 << PDM_LPF_CFG4_LPF_COEFF08_Pos);
    hpdm->Instance->LPF_CFG4 &= ~PDM_LPF_CFG4_LPF_COEFF09;
    hpdm->Instance->LPF_CFG4 |= (0xe80 << PDM_LPF_CFG4_LPF_COEFF09_Pos);

    hpdm->Instance->LPF_CFG5 &= ~PDM_LPF_CFG5_LPF_COEFF10;
    hpdm->Instance->LPF_CFG5 |= (0x06b << PDM_LPF_CFG5_LPF_COEFF10_Pos);
    hpdm->Instance->LPF_CFG5 &= ~PDM_LPF_CFG5_LPF_COEFF11;
    hpdm->Instance->LPF_CFG5 |= (0x506 << PDM_LPF_CFG5_LPF_COEFF11_Pos);

    hpdm->Instance->LPF_CFG6 &= ~PDM_LPF_CFG6_LPF_COEFF12;
    hpdm->Instance->LPF_CFG6 |= (0x791 << PDM_LPF_CFG6_LPF_COEFF12_Pos);
#endif

    hpdm->Instance->LPF_CFG6 &= ~PDM_LPF_CFG6_LPF_BYPASS;



    hpdm->ErrorCode = PDM_ERROR_NONE;
    hpdm->State     = PDM_STATE_READY;


    return HAL_OK;
}

HAL_StatusTypeDef HAL_PDM_DeInit(PDM_HandleTypeDef *hpdm)
{
    __HAL_PDM_DISABLE(hpdm);
    hpdm->State = PDM_STATE_RESET;

    return HAL_OK;
}

__weak void HAL_PDM_MspInit(PDM_HandleTypeDef *hpdm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpdm);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_PDM_MspInit should be implemented in the user file
     */
}

__weak void HAL_PDM_MspDeInit(PDM_HandleTypeDef *hpdm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpdm);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_PDM_MspDeInit should be implemented in the user file
     */
}


HAL_StatusTypeDef HAL_PDM_Set_Gain(PDM_HandleTypeDef *hpdm, PDM_ChannelTypeDef channel, uint8_t value)
{
    uint8_t hw_gain = value;
    switch (channel)
    {
    case PDM_CHANNEL_LEFT_ONLY:
        hpdm->Instance->PGA_CFG &= ~PDM_PGA_CFG_PGA_GAIN_L;
        hpdm->Instance->PGA_CFG |= (hw_gain << PDM_PGA_CFG_PGA_GAIN_L_Pos);
        break;

    case PDM_CHANNEL_RIGHT_ONLY:
        hpdm->Instance->PGA_CFG &= ~PDM_PGA_CFG_PGA_GAIN_R;
        hpdm->Instance->PGA_CFG |= (hw_gain << PDM_PGA_CFG_PGA_GAIN_R_Pos);
        break;

    case PDM_CHANNEL_STEREO:
    case PDM_CHANNEL_STEREO_SWAP:
        hpdm->Instance->PGA_CFG &= ~(PDM_PGA_CFG_PGA_GAIN_L | PDM_PGA_CFG_PGA_GAIN_R);
        hpdm->Instance->PGA_CFG |= (hw_gain << PDM_PGA_CFG_PGA_GAIN_L_Pos) | (hw_gain << PDM_PGA_CFG_PGA_GAIN_R_Pos);
        break;

    default:
        HAL_ASSERT(0);
        break;
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PDM_Config(PDM_HandleTypeDef *hpdm, PDM_ConfigureTypeDef type)
{

    if (hpdm == NULL)
    {
        return HAL_ERROR;
    }
    if (hpdm->State != PDM_STATE_READY)
    {
        return  HAL_BUSY;
    }


    return PDM_Config(hpdm, type);
}

HAL_StatusTypeDef HAL_PDM_Receive_DMA(PDM_HandleTypeDef *hpdm, uint8_t *pData, uint32_t Size)
{
    HAL_StatusTypeDef errorcode = HAL_OK;

    uint32_t fifo_addr, transfw;

    /* check rx dma handle */
    HAL_ASSERT(hpdm->hdmarx != NULL);
    HAL_ASSERT(HAL_ALIGN((uint32_t)pData, 0x4));
    HAL_ASSERT(HAL_ALIGN(Size, 0x4));


    /* Process Locked */
    __HAL_LOCK(hpdm);
    __HAL_PDM_DISABLE_IT(hpdm, (PDM_INT_MSK_INT_MASK_R | PDM_INT_MSK_INT_MASK_L));

    if (hpdm->State != PDM_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    if (HAL_OK != PDM_ConfigDMA(hpdm))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Set the transaction information */
    hpdm->State       = PDM_STATE_BUSY_RX;
    hpdm->ErrorCode   = PDM_ERROR_NONE;
    hpdm->pRxBuffPtr  = (uint8_t *)pData;
    hpdm->RxXferSize  = Size;



    /* Set the PDM RxDMA Half transfer complete callback */
    hpdm->hdmarx->XferHalfCpltCallback = PDM_DMAHalfReceiveCplt;

    /* Set the PDM Rx DMA transfer complete callback */
    hpdm->hdmarx->XferCpltCallback = PDM_DMAReceiveCplt;

    /* Set the DMA error callback */
    hpdm->hdmarx->XferErrorCallback = PDM_DMAError;

    /* Set the DMA AbortCpltCallback */
    hpdm->hdmarx->XferAbortCallback = NULL;


    if (PDM_CHANNEL_RIGHT_ONLY == hpdm->Init.Channels)
    {
        fifo_addr = (uint32_t)&hpdm->Instance->DATA_R;
    }
    else
    {
        fifo_addr = (uint32_t)&hpdm->Instance->DATA_L;
    }

#ifdef  SF32LB55X
    if (PDM_CHANNEL_DEPTH_8BIT == hpdm->Init.ChannelDepth)
    {
        /*32bit -> 8bit*/
        fifo_addr += 0x2;
        transfw = hpdm->RxXferSize;
    }
    else
#endif
    {
        transfw = hpdm->RxXferSize >> 2;
    }

    /* Enable the PDM Error Interrupt Bit */
    __HAL_PDM_ENABLE_IT(hpdm, (PDM_INT_MSK_INT_MASK_R | PDM_INT_MSK_INT_MASK_L));

    /* Enable the Rx DMA Stream/Channel  */
    HAL_DMA_Start_IT(hpdm->hdmarx, (uint32_t)fifo_addr, (uint32_t)hpdm->pRxBuffPtr, transfw);

    /* Check if the PDM is already enabled */
    if ((hpdm->Instance->CFG0 & PDM_CFG0_PDMCOREEN) != PDM_CFG0_PDMCOREEN)
    {
        /* Enable PDM peripheral */
        __HAL_PDM_ENABLE(hpdm);
    }


error:
    /* Process Unlocked */
    __HAL_UNLOCK(hpdm);
    return errorcode;
}



/**
  * @brief Stop the DMA Transfer.
  * @param  hpdm pointer to a PDM_HandleTypeDef structure that contains
  *               the configuration information for the specified PDM module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PDM_DMAStop(PDM_HandleTypeDef *hpdm)
{
    /* The Lock is not implemented on this API to allow the user application
       to call the HAL PDM API under callbacks HAL_PDM_TxCpltCallback() or HAL_PDM_RxCpltCallback() or HAL_PDM_TxRxCpltCallback():
       when calling HAL_DMA_Abort() API the DMA TX/RX Transfer complete interrupt is generated
       and the correspond call back is executed HAL_PDM_TxCpltCallback() or HAL_PDM_RxCpltCallback() or HAL_PDM_TxRxCpltCallback()
       */

    /* Abort the PDM DMA rx Stream/Channel  */
    if (hpdm->hdmarx != NULL)
    {
        HAL_DMA_Abort(hpdm->hdmarx);
    }

    /* Disable the PDM DMA Tx & Rx requests */
    //CLEAR_BIT(hpdm->Instance->CR2, PDM_CR2_TXDMAEN | PDM_CR2_RXDMAEN);
    hpdm->State = PDM_STATE_READY;
    return HAL_OK;
}

void HAL_PDM_IRQHandler(PDM_HandleTypeDef *hpdm)
{
    uint32_t err = hpdm->Instance->INT_ST;
    uint32_t err_mask = hpdm->Instance->INT_MSK;

    if (err & PDM_INT_ST_OVERFLOW_R)
    {
        hpdm->Instance->INT_CLR = PDM_INT_CLR_INT_CLR_R;
        if (0 == (err_mask & PDM_INT_MSK_INT_MASK_R))
            hpdm->ErrorCode |= PDM_ERROR_OVERFLOW_R;
        else {
            ;//Ignore
        }
    }

    if (err & PDM_INT_ST_OVERFLOW_L)
    {
        hpdm->Instance->INT_CLR = PDM_INT_CLR_INT_CLR_L;
        if (0 == (err_mask & PDM_INT_MSK_INT_MASK_L))
            hpdm->ErrorCode |= PDM_ERROR_OVERFLOW_L;
        else {
            ;//Ignore
        }
    }
}


__weak void HAL_PDM_RxHalfCpltCallback(PDM_HandleTypeDef *hpdm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpdm);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_PDM_RxHalfCpltCallback should be implemented in the user file
     */
}

__weak void HAL_PDM_RxCpltCallback(PDM_HandleTypeDef *hpdm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpdm);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_PDM_RxCpltCallback should be implemented in the user file
     */
}

__weak void HAL_PDM_ErrorCallback(PDM_HandleTypeDef *hpdm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpdm);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_PDM_ErrorCallback should be implemented in the user file
     */
}

__weak void HAL_PDM_AbortCpltCallback(PDM_HandleTypeDef *hpdm)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hpdm);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_PDM_AbortCpltCallback should be implemented in the user file
     */
}


/**
  * @brief DMA PDM receive process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void PDM_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
    PDM_HandleTypeDef *hpdm = (PDM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    /* DMA Normal Mode */
    if ((hdma->Instance->CCR & DMA_CCR_CIRC) != DMA_CCR_CIRC)
    {
        /* Disable ERR interrupt */
        __HAL_PDM_DISABLE_IT(hpdm, (PDM_INT_MSK_INT_MASK_R | PDM_INT_MSK_INT_MASK_L));


        hpdm->RxXferSize = 0U;
        hpdm->State = PDM_STATE_READY;


        if (hpdm->ErrorCode != PDM_ERROR_NONE)
        {
            HAL_PDM_ErrorCallback(hpdm);
            return;
        }
    }
    HAL_PDM_RxCpltCallback(hpdm);

    if (PDM_MODE_ONCE == hpdm->Init.Mode)
        __HAL_PDM_DISABLE(hpdm);

}

/**
  * @brief  DMA PDM half receive process complete callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void PDM_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma)
{
    PDM_HandleTypeDef *hpdm = (PDM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    HAL_PDM_RxHalfCpltCallback(hpdm);
}


/**
  * @brief  DMA PDM communication error callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void PDM_DMAError(DMA_HandleTypeDef *hdma)
{
    PDM_HandleTypeDef *hpdm = (PDM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    SET_BIT(hpdm->ErrorCode, PDM_ERROR_DMA);
    hpdm->State = PDM_STATE_READY;
    HAL_PDM_ErrorCallback(hpdm);
}


static HAL_StatusTypeDef PDM_Config(PDM_HandleTypeDef *hpdm, PDM_ConfigureTypeDef type)
{
    uint32_t cfg0_value, cfg_fifo, lpf_cfg, sinc_cfg;

    if (hpdm == NULL)
    {
        return HAL_ERROR;
    }
    cfg0_value = hpdm->Instance->CFG0;
    cfg_fifo   = hpdm->Instance->FIFO_CFG;

    cfg_fifo &= ~PDM_FIFO_CFG_PDM_SHIFT_Msk;

    if (type & PDM_CFG_CHANNEL)
    {
        cfg0_value &= ~(PDM_CFG0_LEFT_EN | PDM_CFG0_RIGHT_EN | PDM_CFG0_STEREO_EN | PDM_CFG0_SWAP_EN);
        cfg_fifo &= ~(PDM_FIFO_CFG_BYTE_CON | PDM_FIFO_CFG_RX_DMA_MSK_L | PDM_FIFO_CFG_RX_DMA_MSK_R);
        switch (hpdm->Init.Channels)
        {
        case PDM_CHANNEL_LEFT_ONLY:
            cfg0_value |= PDM_CFG0_LEFT_EN;
            cfg_fifo   |= PDM_FIFO_CFG_RX_DMA_MSK_R; //mask rigth channel
            break;

        case PDM_CHANNEL_RIGHT_ONLY:
            cfg0_value |= PDM_CFG0_RIGHT_EN;
            cfg_fifo   |= PDM_FIFO_CFG_RX_DMA_MSK_L; //mask left channel
            break;

        case PDM_CHANNEL_STEREO:
            cfg_fifo   |= PDM_FIFO_CFG_BYTE_CON;
            cfg0_value |= PDM_CFG0_LEFT_EN | PDM_CFG0_RIGHT_EN | PDM_CFG0_STEREO_EN;
#ifdef SF32LB55X
            if (PDM_CHANNEL_DEPTH_16BIT != hpdm->Init.ChannelDepth) HAL_ASSERT(0);
#endif /* SF32LB55X */
            break;

        case PDM_CHANNEL_STEREO_SWAP:
            cfg_fifo   |= PDM_FIFO_CFG_BYTE_CON;
            cfg0_value |= (PDM_CFG0_LEFT_EN | PDM_CFG0_RIGHT_EN | PDM_CFG0_STEREO_EN | PDM_CFG0_SWAP_EN);
#ifdef SF32LB55X
            if (PDM_CHANNEL_DEPTH_16BIT != hpdm->Init.ChannelDepth) HAL_ASSERT(0);
#endif /* SF32LB55X */
            break;

        default:
            HAL_ASSERT(0);
            break;
        }
    }

    if (type & PDM_CFG_SAMPLERATE)
    {
        /*
            Sample rate = clk_source / clk_div / sinc_rate / down_sample
        */
        lpf_cfg = hpdm->Instance->LPF_CFG6;
        sinc_cfg = hpdm->Instance->SINC_CFG;

        lpf_cfg &= ~PDM_LPF_CFG6_LPF_DS;       /*0: down_sample=2   1: down_sample=4*/
        sinc_cfg &= ~PDM_SINC_CFG_SINC_ORDER_SEL; /*0: sinc_order=3   1: sinc_order=4*/
        sinc_cfg &= ~PDM_SINC_CFG_SINC_RATE;
        cfg_fifo &= ~(PDM_FIFO_CFG_PDM_SHIFT);
        switch (hpdm->Init.SampleRate)
        {
        case PDM_SAMPLE_8KHZ:
            if (hpdm->Init.clkSrc == 9600000)
            {
                cfg0_value &= ~PDM_CFG0_CLK_SEL; //9.6MHz source
                cfg0_value &= ~PDM_CFG0_CLK_DIV;
                cfg0_value |= (4 << PDM_CFG0_CLK_DIV_Pos);
                sinc_cfg |= (75 << PDM_SINC_CFG_SINC_RATE_Pos);
                lpf_cfg |= PDM_LPF_CFG6_LPF_DS;
                //cfg_fifo |= (0x2 << PDM_FIFO_CFG_PDM_SHIFT_Pos); //Align to 24bit
            }
            else  if (hpdm->Init.clkSrc == 3072000)
            {
                cfg0_value |= PDM_CFG0_CLK_SEL; //3.072MHz source
                cfg0_value &= ~PDM_CFG0_CLK_DIV;
                cfg0_value |= (3 << PDM_CFG0_CLK_DIV_Pos);
                sinc_cfg |= (64 << PDM_SINC_CFG_SINC_RATE_Pos);
                //cfg_fifo |= (0x2 << PDM_FIFO_CFG_PDM_SHIFT_Pos); //Align to 24bit
            }
            break;
        case PDM_SAMPLE_16KHZ:
            if (hpdm->Init.clkSrc == 9600000)
            {
                cfg0_value &= ~PDM_CFG0_CLK_SEL; //9.6MHz source
                cfg0_value &= ~PDM_CFG0_CLK_DIV;
                cfg0_value |= (6 << PDM_CFG0_CLK_DIV_Pos);
                sinc_cfg |= (50 << PDM_SINC_CFG_SINC_RATE_Pos);
                //cfg_fifo |= (0x4 << PDM_FIFO_CFG_PDM_SHIFT_Pos);  //Align to 24bit
            }
            else if (hpdm->Init.clkSrc == 3072000)
            {
                cfg0_value |= PDM_CFG0_CLK_SEL; //3.072MHz source
                cfg0_value &= ~PDM_CFG0_CLK_DIV;
                cfg0_value |= (1 << PDM_CFG0_CLK_DIV_Pos);
                sinc_cfg |= (96 << PDM_SINC_CFG_SINC_RATE_Pos);
            }
            break;
        case PDM_SAMPLE_32KHZ:
            if (hpdm->Init.clkSrc == 9600000)
            {
                cfg0_value &= ~PDM_CFG0_CLK_SEL; //9.6MHz source
                cfg0_value &= ~PDM_CFG0_CLK_DIV;
                cfg0_value |= (6 << PDM_CFG0_CLK_DIV_Pos);
                sinc_cfg |= (25 << PDM_SINC_CFG_SINC_RATE_Pos);
                //lpf_cfg |= PDM_LPF_CFG6_LPF_DS;
            }
            else if (hpdm->Init.clkSrc == 3072000)
            {

                cfg0_value |= PDM_CFG0_CLK_SEL; //3.072MHz source
                cfg0_value &= ~PDM_CFG0_CLK_DIV;
                cfg0_value |= (1 << PDM_CFG0_CLK_DIV_Pos);
                sinc_cfg |= (48 << PDM_SINC_CFG_SINC_RATE_Pos);
            }
            break;
        case PDM_SAMPLE_48KHZ:
            if (hpdm->Init.clkSrc == 9600000)
            {
                cfg0_value &= ~PDM_CFG0_CLK_SEL; //9.6MHz source
                cfg0_value &= ~PDM_CFG0_CLK_DIV;
                cfg0_value |= (4 << PDM_CFG0_CLK_DIV_Pos);
                sinc_cfg |= (25 << PDM_SINC_CFG_SINC_RATE_Pos);
                sinc_cfg |= PDM_SINC_CFG_SINC_ORDER_SEL;
                //cfg_fifo |= (0x4 << PDM_FIFO_CFG_PDM_SHIFT_Pos);  //Align to 24bit
            }
            else if (hpdm->Init.clkSrc == 3072000)
            {
                cfg0_value |= PDM_CFG0_CLK_SEL; //3.072MHz source
                cfg0_value &= ~PDM_CFG0_CLK_DIV;
                cfg0_value |= (1 << PDM_CFG0_CLK_DIV_Pos);
                sinc_cfg |= (32 << PDM_SINC_CFG_SINC_RATE_Pos);
            }
            break;
        case PDM_SAMPLE_96KHZ:
        {
            /* Only support audpll */
            /* pcm_clk:3.072M SINCRATE:8 LPF:4 SINC ORDER:4 */
            cfg0_value |= PDM_CFG0_CLK_SEL; //3.072MHz source
            cfg0_value &= ~PDM_CFG0_CLK_DIV;
            cfg0_value |= (1 << PDM_CFG0_CLK_DIV_Pos);
            sinc_cfg |= (8 << PDM_SINC_CFG_SINC_RATE_Pos);
            sinc_cfg |= PDM_SINC_CFG_SINC_ORDER_SEL;
            lpf_cfg |= PDM_LPF_CFG6_LPF_DS;
            /* reg_pga_gain = 0x72,  pdm_shift = 0 */
            // cfg_fifo |= (0x8 << PDM_FIFO_CFG_PDM_SHIFT_Pos);
            hpdm->Instance->PGA_CFG &= ~PDM_PGA_CFG_PGA_GAIN_L;
            hpdm->Instance->PGA_CFG |= (0x72 << PDM_PGA_CFG_PGA_GAIN_L_Pos);
            hpdm->Instance->PGA_CFG &= ~PDM_PGA_CFG_PGA_GAIN_R;
            hpdm->Instance->PGA_CFG |= (0x72 << PDM_PGA_CFG_PGA_GAIN_R_Pos);
        }
        break;
        default:
            HAL_ASSERT(0);
            return HAL_ERROR;
            break;
        }
        hpdm->Instance->SINC_CFG = sinc_cfg;
        hpdm->Instance->LPF_CFG6 = lpf_cfg;
    }

    if (type & PDM_CFG_DEPTH)
    {
        /*Channel depth*/
#ifndef SF32LB55X
        cfg_fifo &= ~PDM_FIFO_CFG_BYTE_TRUNC;
        switch (hpdm->Init.ChannelDepth)
        {
        case PDM_CHANNEL_DEPTH_8BIT:
            cfg_fifo |= (2 << PDM_FIFO_CFG_BYTE_TRUNC_Pos);
            break;
        case PDM_CHANNEL_DEPTH_16BIT:
            cfg_fifo |= (1 << PDM_FIFO_CFG_BYTE_TRUNC_Pos);
            break;
        case PDM_CHANNEL_DEPTH_24BIT:
            cfg_fifo |= (0 << PDM_FIFO_CFG_BYTE_TRUNC_Pos);
            break;
        case PDM_CHANNEL_DEPTH_32BIT:
            cfg_fifo |= (3 << PDM_FIFO_CFG_BYTE_TRUNC_Pos);
            break;
        default:
            HAL_ASSERT(0);
            return HAL_ERROR;
            break;
        }
#else
        if (PDM_CHANNEL_DEPTH_16BIT != hpdm->Init.ChannelDepth)
        {
            cfg_fifo &= ~PDM_FIFO_CFG_BYTE_TRUNC;
        }
        else
        {
            cfg_fifo |= PDM_FIFO_CFG_BYTE_TRUNC;
        }
#endif
    }
    hpdm->Instance->CFG0 = cfg0_value;
    hpdm->Instance->FIFO_CFG = cfg_fifo;

    return HAL_OK;
}


static HAL_StatusTypeDef PDM_ConfigDMA(PDM_HandleTypeDef *hpdm)
{
    /*Init DMA*/
    if (hpdm->hdmarx != NULL)
    {
        hpdm->hdmarx->Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hpdm->hdmarx->Init.PeriphInc           = DMA_PINC_DISABLE;
        hpdm->hdmarx->Init.MemInc              = DMA_MINC_ENABLE;
#ifdef  SF32LB55X
        if (PDM_CHANNEL_DEPTH_8BIT == hpdm->Init.ChannelDepth)
        {
            hpdm->hdmarx->Init.PeriphDataAlignment = DMA_MDATAALIGN_BYTE;
            hpdm->hdmarx->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        }
        else
#endif
        {
            hpdm->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
            hpdm->hdmarx->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        }
        hpdm->hdmarx->Init.Mode                = (PDM_MODE_LOOP == hpdm->Init.Mode) ? DMA_CIRCULAR : DMA_NORMAL;
        hpdm->hdmarx->Init.Priority            = DMA_PRIORITY_LOW;
        hpdm->hdmarx->Parent = (void *)hpdm;

        HAL_DMA_Init(hpdm->hdmarx);
    }
    if (hpdm->hdmarx_r != NULL)
    {
        hpdm->hdmarx_r->Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hpdm->hdmarx_r->Init.PeriphInc           = DMA_PINC_DISABLE;
        hpdm->hdmarx_r->Init.MemInc              = DMA_MINC_ENABLE;
#ifdef  SF32LB55X
        if (PDM_CHANNEL_DEPTH_8BIT == hpdm->Init.ChannelDepth)
        {
            hpdm->hdmarx_r->Init.PeriphDataAlignment = DMA_MDATAALIGN_BYTE;
            hpdm->hdmarx_r->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        }
        else
#endif
        {
            hpdm->hdmarx_r->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
            hpdm->hdmarx_r->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        }
        hpdm->hdmarx_r->Init.Mode                = (PDM_MODE_LOOP == hpdm->Init.Mode) ? DMA_CIRCULAR : DMA_NORMAL;
        hpdm->hdmarx_r->Init.Priority            = DMA_PRIORITY_LOW;
        hpdm->hdmarx_r->Parent = (void *)hpdm;

        HAL_DMA_Init(hpdm->hdmarx_r);
    }

    return HAL_OK;
}

#endif /* HAL_PDM_MODULE_ENABLED */
