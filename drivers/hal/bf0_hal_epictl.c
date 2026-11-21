/**
 ******************************************************************************
 * @file   bf0_hal_epictl.c
 * @author Sifli software development team
 * @brief   EPIC Transfer Layer HAL module driver.
 *          This file provides firmware functions to manage the following functions:
 *     + Initialization and de-initialization functions
 *     + Intterrupt handler
 *     + Data transfer functions in polling and interrupt mode
 *
 ******************************************************************************
 */
/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"
#include "string.h"

#ifdef HAL_EPICTL_ENABLED

#define RETURN_ERROR(hepictl,ret_v) \
        do{ (hepictl)->ErrorCode = __LINE__;  return ret_v; }while(0)

#define __HAL_EPICTL_LOCK(__HANDLE__)                                           \
                                do{                                        \
                                    if(HAL_EPICTL_STATE_BUSY == (__HANDLE__)->State)   \
                                    {                                      \
                                       (__HANDLE__)->ErrorCode = __LINE__;\
                                       HAL_ASSERT(0);                \
                                       return HAL_BUSY;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->State = HAL_EPICTL_STATE_BUSY;    \
                                       EPICTL_TICK_START((__HANDLE__)); \
                                    }                                      \
                                  }while (0)

#define __HAL_EPICTL_UNLOCK(__HANDLE__)                                          \
                                  do{                                       \
                                      HAL_ASSERT(HAL_EPICTL_STATE_BUSY == (__HANDLE__)->State); \
                                      (__HANDLE__)->State = HAL_EPICTL_STATE_READY;    \
                                      EPICTL_TICK_END((__HANDLE__));   \
                                    }while (0)

#define EPICTL_CFG_FMT_RGB565          (0 << EPIC_TL_CFG_FORMAT_Pos)
#define EPICTL_CFG_FMT_RGB888          (1 << EPIC_TL_CFG_FORMAT_Pos)
#define EPICTL_CFG_FMT_ARGB8888        (2 << EPIC_TL_CFG_FORMAT_Pos)
#define EPICTL_CFG_FMT_ARGB8565        (3 << EPIC_TL_CFG_FORMAT_Pos)
#define EPICTL_CFG_FMT_A8              (4 << EPIC_TL_CFG_FORMAT_Pos)

/*===============================================================================
  Functions
 ================================================================================*/

static void EPICTL_TICK_START(EPICTL_HandleTypeDef *hepictl)
{
    hepictl->start_tick = HAL_DBG_DWT_GetCycles();
    hepictl->Instance->TL_PERF_CNT = 0;
}

static void EPICTL_TICK_END(EPICTL_HandleTypeDef *hepictl)
{
    uint32_t hw_cnt = hepictl->Instance->TL_PERF_CNT;
    hepictl->end_tick = HAL_DBG_DWT_GetCycles();
    hepictl->PerfCnt += hw_cnt;
    hepictl->HalCnt  += HAL_GetElapsedTick(hepictl->start_tick, hepictl->end_tick) - hw_cnt;
}

static uint32_t EPICTL_GetLayerColorFormat(uint32_t color_mode)
{
    uint32_t layer_color_format;

    if (EPIC_COLOR_RGB565 == color_mode)
    {
        layer_color_format = EPICTL_CFG_FMT_RGB565;
    }
    else if (EPIC_COLOR_ARGB8565 == color_mode)
    {
        layer_color_format = EPICTL_CFG_FMT_ARGB8565;
    }
    else if (EPIC_COLOR_RGB888 == color_mode)
    {
        layer_color_format = EPICTL_CFG_FMT_RGB888;
    }
    else if (EPIC_COLOR_A8 == color_mode)
    {
        layer_color_format = EPICTL_CFG_FMT_A8;
    }
    else if (EPIC_COLOR_ARGB8888 == color_mode)
    {
        layer_color_format = EPICTL_CFG_FMT_ARGB8888;
    }
    else
    {
        HAL_ASSERT(0);
    }

    return layer_color_format;
}

static HAL_StatusTypeDef EPICTL_Config(EPICTL_HandleTypeDef *epictl, EPICTL_DataType *cfg)
{
    EPIC_TypeDef *hwp_epictl = epictl->Instance;
    uint32_t ouput_stride;
    if (0 == cfg->transfer_width || 0 == cfg->transfer_height)
    {
        return HAL_EPIC_NOTHING_TO_DO;
    }

    //source configuration
    hwp_epictl->TL_CFG = MAKE_REG_VAL(cfg->src_stride, EPIC_TL_CFG_WIDTH_Msk, EPIC_TL_CFG_WIDTH_Pos)
                         | MAKE_REG_VAL(EPICTL_GetLayerColorFormat(cfg->src_color_mode), EPIC_TL_CFG_FORMAT_Msk, EPIC_TL_CFG_FORMAT_Pos);
    hwp_epictl->TL_SRC = (uint32_t)HCPU_MPI_SBUS_ADDR(cfg->src);


    //dstination configuration
    if (cfg->dst_compression_rate > 0)
    {
        uint32_t chunk_size, chunks, tgt_size, cfg0, cfg1;

        chunks = CMPR_CHUNKS(cfg->transfer_width);
        chunk_size = CMPR_CHUNK_SIZE(cfg->transfer_width);
        tgt_size  = HAL_EPICTL_CMPR_Target_size(chunk_size, cfg->dst_color_mode, cfg->dst_compression_rate);

        if (tgt_size > (EPIC_CMPRCR_TGTSIZE_Msk >> EPIC_CMPRCR_TGTSIZE_Pos))
        {
            RETURN_ERROR(epictl, HAL_ERROR);
        }
        hwp_epictl->CMPRCR = MAKE_REG_VAL(chunk_size, EPIC_CMPRCR_LINESIZE_Msk, EPIC_CMPRCR_LINESIZE_Pos)
                             | MAKE_REG_VAL(chunks, EPIC_CMPRCR_CHUNKCNT_Msk,  EPIC_CMPRCR_CHUNKCNT_Pos)
                             | MAKE_REG_VAL(tgt_size, EPIC_CMPRCR_TGTSIZE_Msk,  EPIC_CMPRCR_TGTSIZE_Pos)
                             | EPIC_CMPRCR_CMPREN;


        HAL_EPICTL_CMPR_GetConfig(cfg->dst_color_mode, &cfg0, &cfg1);
        hwp_epictl->CMPRCFG0 = cfg0;
        hwp_epictl->CMPRCFG1 = cfg1;

        ouput_stride = chunks * tgt_size * 6;
    }
    else
    {
        hwp_epictl->CMPRCR = 0;
        ouput_stride = cfg->transfer_width * (HAL_EPIC_GetColorDepth(cfg->dst_color_mode) >> 3);
    }
    hwp_epictl->TL_AHB_CTRL = MAKE_REG_VAL(cfg->dst_color_mode, EPIC_AHB_CTRL_O_FORMAT_Msk, EPIC_AHB_CTRL_O_FORMAT_Pos);
    hwp_epictl->TL_AHB_MEM = (uint32_t)HCPU_MPI_SBUS_ADDR(cfg->dst);
    if (cfg->dst_stride < ouput_stride)
    {
        RETURN_ERROR(epictl, HAL_ERROR);
    }
    hwp_epictl->TL_AHB_STRIDE = cfg->dst_stride - ouput_stride;

    //transfer size configuration
    hwp_epictl->TL_SIZE = MAKE_REG_VAL(cfg->transfer_width, EPIC_TL_SIZE_MAX_COL_Msk, EPIC_TL_SIZE_MAX_COL_Pos)
                          | MAKE_REG_VAL(cfg->transfer_height, EPIC_TL_SIZE_MAX_LINE_Msk, EPIC_TL_SIZE_MAX_LINE_Pos);
    return HAL_OK;
}

//EPICTL init and deinit
void HAL_EPICTL_Init(EPICTL_HandleTypeDef *epictl)
{
    HAL_ASSERT(NULL != epictl);
#ifndef SF32LB55X
    HAL_RCC_EnableModule(RCC_MOD_EPIC);
#endif /* SF32LB55X */
    epictl->ErrorCode = 0;
    epictl->PerfCnt = 0;
    epictl->WaitCnt = 0;
    epictl->State = HAL_EPICTL_STATE_READY;
}

void HAL_EPICTL_DeInit(EPICTL_HandleTypeDef *epictl)
{
    HAL_ASSERT(NULL != epictl);
    epictl->State = HAL_EPICTL_STATE_RESET;
}

__weak void HAL_EPICTL_OverFlowIRQHandler(EPICTL_HandleTypeDef *epictl)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(epictl);

    /* NOTE : This function should not be modified, when the callback is needed,
              it could be implemented in the user file
     */
}

//EPIC Transfer Layer IRQ Handler
void HAL_EPICTL_IRQHandler(EPICTL_HandleTypeDef *epictl)
{
    EPIC_TypeDef *hwp_epictl = epictl->Instance;
    uint32_t irq_status = hwp_epictl->TL_IRQ;

    hwp_epictl->TL_IRQ = irq_status; // Clear the interrupt

    if (irq_status & EPIC_TL_IRQ_OCB_OF_CAUSE) // OCB overflow
    {
        hwp_epictl->TL_SETTING &= ~EPIC_TL_SETTING_OCB_OF_IRQ_MASK;
        HAL_EPICTL_OverFlowIRQHandler(epictl);
    }

    if (irq_status & EPIC_TL_IRQ_EOT_CAUSE)
    {
        void (*cb)(EPICTL_HandleTypeDef * epictl);
        hwp_epictl->TL_SETTING = 0; //Disable all IRQs

        cb = epictl->XferCpltCallback;
        __HAL_EPICTL_UNLOCK(epictl);

        if (cb)
        {
            cb(epictl);
        }
    }
}

/**
 * @brief  Initialize an epictl data struct to default vaules
 *
 * @param param  EPICTL data parameter
 * @retval None
 */
void HAL_EPICTL_DataInit(EPICTL_DataType *param)
{
    memset(param, 0, sizeof(EPICTL_DataType));
}

/**
 * @brief  Start EPIC transfer in interrupt mode
 * This function is used to transfer data from src to dst
 * @param[in]  epic EPICTL handle
 * @param[in]  cfg  transfer configuration
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EPICTL_Start_IT(EPICTL_HandleTypeDef *epictl, EPICTL_DataType *cfg)
{
    HAL_StatusTypeDef ret;
    HAL_ASSERT(NULL != epictl);
    HAL_ASSERT(NULL != cfg);



    __HAL_EPICTL_LOCK(epictl);

    ret = EPICTL_Config(epictl, cfg);
    if (HAL_EPIC_NOTHING_TO_DO == ret)
    {
        /*
            Nothing to do, unlock and call callback directly.
            This is used to handle the case that the transfer size is 0.
        */
        void (*cb)(EPICTL_HandleTypeDef * epictl);

        cb = epictl->XferCpltCallback;
        __HAL_EPICTL_UNLOCK(epictl);

        if (cb)
        {
            cb(epictl);
        }

        return HAL_OK;
    }
    else if (HAL_OK != ret)
    {
        __HAL_EPICTL_UNLOCK(epictl);
        RETURN_ERROR(epictl, ret);
    }
    else
    {
        EPIC_TypeDef *hwp_epictl = epictl->Instance;
        hwp_epictl->TL_SETTING = EPIC_TL_SETTING_OCB_OF_IRQ_MASK | EPIC_TL_SETTING_EOT_IRQ_MASK;
        hwp_epictl->TL_COMMAND = EPIC_TL_COMMAND_START; // Start the transfer

        return HAL_OK;
    }
}

/**
 * @brief  Start EPIC transfer in polling mode
 * This function is used to transfer data from src to dst
 * @param[in]  epic EPICTL handle
 * @param[in]  cfg  transfer configuration
 *
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_EPICTL_Start(EPICTL_HandleTypeDef *epictl, EPICTL_DataType *cfg)
{
    HAL_StatusTypeDef ret;
    HAL_ASSERT(NULL != epictl);
    HAL_ASSERT(NULL != cfg);



    __HAL_EPICTL_LOCK(epictl);

    ret = EPICTL_Config(epictl, cfg);
    if (HAL_EPIC_NOTHING_TO_DO == ret)
    {
        __HAL_EPICTL_UNLOCK(epictl);
        return HAL_OK;
    }
    else if (HAL_OK != ret)
    {
        __HAL_EPICTL_UNLOCK(epictl);
        RETURN_ERROR(epictl, ret);
    }
    else
    {
        EPIC_TypeDef *hwp_epictl = epictl->Instance;
        hwp_epictl->TL_SETTING = 0; // Disable all IRQs
        hwp_epictl->TL_COMMAND = EPIC_TL_COMMAND_START; // Start the transfer

        while (hwp_epictl->TL_STATUS & EPIC_TL_STATUS_TL_BUSY) {;} //Wait for transfer to finish

        __HAL_EPICTL_UNLOCK(epictl);
        return HAL_OK;
    }
}


uint32_t HAL_EPICTL_CMPR_Target_size(uint32_t chunk_pixels, uint32_t color_mode, uint32_t cmpr_rate)
{
    uint32_t target_size = 0;
    HAL_ASSERT((cmpr_rate > 0) && (cmpr_rate <= MAX_CMPR_TBL_SIZE));

    if (EPIC_COLOR_RGB565 == color_mode)
    {
        switch (cmpr_rate)
        {
        case 1:
            target_size = CMPR_1_RGB565_TGT_SIZE(chunk_pixels);
            break;
        case 2:
            target_size = CMPR_2_RGB565_TGT_SIZE(chunk_pixels);
            break;
        case 3:
            target_size = CMPR_3_RGB565_TGT_SIZE(chunk_pixels);
            break;
        }
    }
    else if (EPIC_COLOR_RGB888 == color_mode)
    {
        switch (cmpr_rate)
        {
        case 1:
            target_size = CMPR_1_RGB888_TGT_SIZE(chunk_pixels);
            break;
        case 2:
            target_size = CMPR_2_RGB888_TGT_SIZE(chunk_pixels);
            break;
        case 3:
            target_size = CMPR_3_RGB888_TGT_SIZE(chunk_pixels);
            break;
        }
    }
    else if (EPIC_COLOR_ARGB8888 == color_mode)
    {
        switch (cmpr_rate)
        {
        case 1:
            target_size = CMPR_1_ARGB8888_TGT_SIZE(chunk_pixels);
            break;
        case 2:
            target_size = CMPR_2_ARGB8888_TGT_SIZE(chunk_pixels);
            break;
        case 3:
            target_size = CMPR_3_ARGB8888_TGT_SIZE(chunk_pixels);
            break;
        }
    }
    else
    {
        HAL_ASSERT(0);
    }

    return target_size;
}


#endif /* HAL_EPICTL_ENABLED */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/