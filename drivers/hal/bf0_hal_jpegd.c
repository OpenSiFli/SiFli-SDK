/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"
#include "string.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup JPEGD
  * @{
  */

#if defined(HAL_JPEGD_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

#define IS_REAL_INSTANCE(handle) ((handle)->Instance == (handle)->HwInstance)

/* As JPEGD share some modules from EZIP, EZIP clock needs to be enabled */
#define JPEGD_ENABLE(handle)                          \
    do                                          \
    {                                           \
        if (!IS_REAL_INSTANCE(handle)) break; \
        HAL_RCC_EnableModule(RCC_MOD_JPEGD);    \
        HAL_RCC_EnableModule(RCC_MOD_EZIP);     \
    }                                           \
    while (0)

#define JPEGD_DISABLE(handle)                   \
    do                                          \
    {                                           \
        if (!IS_REAL_INSTANCE(handle)) break; \
        HAL_RCC_DisableModule(RCC_MOD_JPEGD);   \
        HAL_RCC_DisableModule(RCC_MOD_EZIP);    \
    }                                           \
    while (0)


#define JPEGD_CLEAR_INT_STATUS(inst)                                   \
    do                                                                 \
    {                                                                  \
        inst->INT_STA = 0xFFFFFFFF;                                    \
    }                                                                  \
    while (0);

#define LDB_WORD(ptr)       (uint16_t)(((uint16_t)*((uint8_t*)(ptr))<<8)|(uint16_t)*(uint8_t*)((ptr)+1))
/*  This is a timeout value for JPEGD loading header and starting to decode,
    and it is based on JPEGD clock(SYSCLK).
    Set this timeout to about 100ms@240MHz for slow memory
*/
#define ACT_TIME_OUT_TICKS     (100 * (240000000 / 1000) )
static HAL_StatusTypeDef HAL_JPEGD_ConfigDecode(JPEGD_HandleTypeDef *hdl, JPEGD_DecodeConfigTypeDef *config)
{
    uint16_t col_start;
    uint16_t row_start;
    uint32_t col_end;
    uint32_t row_end;
    FLASH_HandleTypeDef *flash_handle;


    /* Check parameters*/
    if ((uint32_t)config->input & 3)
    {
        return HAL_ERROR;
    }

    if ((HAL_JPEGD_OUTPUT_AHB == config->output_mode) &&
            (((uint32_t)config->output_y & 3) || ((uint32_t)config->output_u & 3) || ((uint32_t)config->output_v & 3)))
    {
        return HAL_ERROR;
    }

    /* Configure output*/
    if (HAL_JPEGD_OUTPUT_EPIC == config->output_mode)
    {
        hdl->Instance->JPEGD_PARA = (HAL_JPEGD_OUTPUT_EPIC << JPEGD_JPEGD_PARA_OUT_SEL_Pos);

        /* Configure start/end point*/
        if (config->width > 0)
        {
            col_start = config->start_x;
            col_end = config->start_x + config->width - 1;
        }
        else if (0 == config->width)
        {
            col_start = 1;
            col_end = 0;
        }
        else
        {
            col_start = 0;
            col_end = 0xFFFF;
        }
        if (config->height > 0)
        {
            row_start = config->start_y;
            row_end = config->start_y + config->height - 1;
        }
        else if (0 == config->height)
        {
            row_start = 1;
            row_end = 0;
        }
        else
        {
            row_start = 0;
            row_end = 0xFFFF;
        }
    }
    else if (HAL_JPEGD_OUTPUT_AHB == config->output_mode)
    {
        hdl->Instance->JPEGD_PARA = (HAL_JPEGD_OUTPUT_AHB << JPEGD_JPEGD_PARA_OUT_SEL_Pos) ;
        hdl->Instance->DST_ADDR0 = (uint32_t)config->output_y;
        hdl->Instance->DST_ADDR1 = (uint32_t)config->output_u;
        hdl->Instance->DST_ADDR2 = (uint32_t)config->output_v;

        // AHB mode does not support clip window, need to output all.
        col_start = row_start = 0;
        HAL_JPEGD_GetDim(config->input, config->input_data_size, (int *)&col_end, (int *)&row_end);
        col_end--;
        row_end--;
    }

    /* Configure Input*/
    hdl->Instance->SRC_ADDR = HCPU_MPI_SBUS_ADDR(config->input);
    hdl->Instance->SRC_LEN = config->input_data_size;
    hdl->Instance->BUF_ADDR = HCPU_MPI_SBUS_ADDR(config->work_buffer);

    hdl->Instance->START_POINT = MAKE_REG_VAL(col_start, JPEGD_START_POINT_START_COL_Msk, JPEGD_START_POINT_START_COL_Pos)
                                 | MAKE_REG_VAL(row_start, JPEGD_START_POINT_START_ROW_Msk, JPEGD_START_POINT_START_ROW_Pos);
    hdl->Instance->END_POINT = MAKE_REG_VAL(col_end, JPEGD_END_POINT_END_COL_Msk, JPEGD_END_POINT_END_COL_Pos)
                               | MAKE_REG_VAL(row_end, JPEGD_END_POINT_END_ROW_Msk, JPEGD_END_POINT_END_ROW_Pos);
    hdl->Instance->ACT_TIME = ACT_TIME_OUT_TICKS;
    return HAL_OK;
}


HAL_StatusTypeDef HAL_JPEGD_Init(JPEGD_HandleTypeDef *hdl, JPEGD_CpltCallback cbk, void *user_data)
{
    if (HAL_JPEGD_STATE_RESET != hdl->State)
    {
        return HAL_ERROR;
    }

    HAL_RCC_EnableModule(RCC_MOD_JPEGD);

    hdl->State = HAL_JPEGD_STATE_READY;
    hdl->CpltCallback = cbk;
    hdl->UserData = user_data;
    hdl->RamInstance_used = 0;
    hdl->HwInstance = hdl->Instance;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_JPEGD_DeInit(JPEGD_HandleTypeDef *hdl)
{
    if (HAL_JPEGD_STATE_READY != hdl->State)
    {
        return HAL_ERROR;
    }
    hdl->State = HAL_JPEGD_STATE_RESET;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_JPEGD_IRQHandler(JPEGD_HandleTypeDef *hdl)
{
    uint32_t status;
    bool err = false;
    uint32_t addr;
    bool disabled = false;

    status = hdl->Instance->INT_STA;

    hdl->Instance->INT_STA = status;

    if (JPEGD_INT_STA_JPEGD_END_STA & status)
    {
        hdl->Instance->INT_EN = 0;
        JPEGD_DISABLE(hdl);
        disabled = true;
        hdl->State = HAL_JPEGD_STATE_READY;
        hdl->RunNum++;
        hdl->RunTime += HAL_GTIMER_READ() - hdl->StartTime;

        if (hdl->CpltCallback)
        {
            hdl->CpltCallback(hdl, hdl->UserData);
        }
    }

    if (status &
            (JPEGD_INT_STA_SOI_ERR_STA | JPEGD_INT_STA_SCANACTIVE_ERR_STA | JPEGD_INT_STA_TYPE_ERR_STA |
             JPEGD_INT_STA_CONFG_ERR_STA | JPEGD_INT_STA_SOB_ERR_STA | JPEGD_INT_STA_EOB_ERR_STA |
             JPEGD_INT_STA_LBS_ERR_STA | JPEGD_INT_STA_START_ERR_STA)
       )
        err = true;

    if (err)
    {
        // if (!disabled)
        //     JPEGD_DISABLE(hdl);
        hdl->State = HAL_JPEGD_STATE_ERROR;
        hdl->ErrorCode = status;
        // HAL_ASSERT(0); Let upper layer to handle the error
        return HAL_ERROR;
    }
    else
        return HAL_OK;
}

HAL_StatusTypeDef HAL_JPEGD_Decode(JPEGD_HandleTypeDef *hdl, JPEGD_DecodeConfigTypeDef *config)
{
    HAL_StatusTypeDef status;
    JPEGD_OutputModeTypeDef temp_output_mode = config->output_mode;

    if (HAL_JPEGD_STATE_READY != hdl->State)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    JPEGD_ENABLE(hdl);

    if (0 != hdl->Instance->JPEGD_EN)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    hdl->State = HAL_JPEGD_STATE_BUSY;

    status = HAL_JPEGD_ConfigDecode(hdl, config);

    if (HAL_OK != status)
    {
        hdl->State = HAL_JPEGD_STATE_READY;
        goto __EXIT;
    }

    /* clear old status */
    JPEGD_CLEAR_INT_STATUS(hdl->Instance);

    hdl->Instance->JPEGD_EN = JPEGD_JPEGD_EN_JPEGD_EN;

    if (HAL_JPEGD_OUTPUT_AHB == temp_output_mode)
    {
        while (0 == hdl->Instance->INT_STA)
        {
        }
        if (JPEGD_INT_STA_JPEGD_END_STA & hdl->Instance->INT_STA)
        {
            hdl->State = HAL_JPEGD_STATE_READY;
            status = HAL_OK;
        }
        else
        {
            status = HAL_ERROR;
            HAL_ASSERT(0);
        }
    }
    else
    {
        hdl->State = HAL_JPEGD_STATE_READY;
        status = HAL_OK;
    }

__EXIT:

    JPEGD_DISABLE(hdl);

    return status;

}

HAL_StatusTypeDef HAL_JPEGD_Decode_IT(JPEGD_HandleTypeDef *hdl, JPEGD_DecodeConfigTypeDef *config)
{
    HAL_StatusTypeDef status;

    if (HAL_JPEGD_STATE_READY != hdl->State)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    JPEGD_ENABLE(hdl);

    hdl->State = HAL_JPEGD_STATE_BUSY;

    status = HAL_JPEGD_ConfigDecode(hdl, config);

    if (HAL_OK != status)
    {
        JPEGD_DISABLE(hdl);
        (hdl)->State = HAL_JPEGD_STATE_READY;
        goto __EXIT;
    }

    /* clear old status */
    JPEGD_CLEAR_INT_STATUS(hdl->Instance);

    hdl->StartTime = HAL_GTIMER_READ();

    hdl->Instance->INT_EN = 0x1F;

    hdl->Instance->JPEGD_EN = JPEGD_JPEGD_EN_JPEGD_EN;

    status = HAL_OK;

__EXIT:

    return status;
}


HAL_StatusTypeDef HAL_JPEGD_DecodeFast_IT(JPEGD_HandleTypeDef *hdl)
{
    HAL_StatusTypeDef status;
    HAL_ASSERT(NULL != hdl->HwInstance);
    HAL_ASSERT(NULL != hdl->RamInstance);
    HAL_ASSERT(hdl->RamInstance_used);


    if (HAL_JPEGD_STATE_READY != hdl->State)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    hdl->Instance = hdl->HwInstance;


    JPEGD_ENABLE(hdl);
    hdl->State = HAL_JPEGD_STATE_BUSY;


    hdl->RamInstance_used = 0;
    hdl->Instance = hdl->HwInstance;

    /* clear old status */
    JPEGD_CLEAR_INT_STATUS(hdl->Instance);

    memcpy((void *)&hdl->Instance->BUF_ADDR, (const void *)&hdl->RamInstance->BUF_ADDR,
           ((uint32_t)&hdl->Instance->INT_STA) - ((uint32_t)&hdl->Instance->BUF_ADDR));

    hdl->Instance->JPEGD_EN = JPEGD_JPEGD_EN_JPEGD_EN;

    status = HAL_OK;

__EXIT:

    return status;
}


HAL_StatusTypeDef HAL_JPEGD_DecodeFast_Init(JPEGD_HandleTypeDef *hdl)
{

    hdl->RamInstance_used = 1;
    hdl->Instance = hdl->RamInstance;
    hdl->Instance->INT_EN = 0;

    hdl->State = HAL_JPEGD_STATE_READY;

    return HAL_OK;
}


HAL_StatusTypeDef HAL_JPEGD_CheckReady(JPEGD_HandleTypeDef *hdl)
{
    HAL_StatusTypeDef status;

    if (HAL_JPEGD_STATE_READY != hdl->State)
    {
        status = HAL_BUSY;
        goto __EXIT;
    }

    status = HAL_OK;
__EXIT:

    return status;

}

int HAL_JPEGD_GetBufferSize(JPEGD_HandleTypeDef *hdl, JPEGD_DecodeConfigTypeDef *config)
{
    int r;

    r = config->width;
    // Column size 16 bytes aligned.
    r = ((r + 15) >> 4) << 4;
    if (HAL_JPEGD_OUTPUT_EPIC == config->output_mode)
    {
        // 32 rows fixed,
        // 2 bytes each pixel
        r <<= 6;
    }
    else if (HAL_JPEGD_OUTPUT_AHB == config->output_mode)
    {
        // 32 rows fixed,
        // 1.5 bytes each pixel
        r <<= 5;
        r = r + (r >> 1);
    }
    else
        r = 0;
    return r;
}


HAL_StatusTypeDef HAL_JPEGD_GetOutputSize(JPEGD_HandleTypeDef *hdl, JPEGD_DecodeConfigTypeDef *config, int *y_size, int *u_size, int *v_size)
{
    HAL_StatusTypeDef r = HAL_OK;
    int size;
    int32_t width;
    int32_t height;

    /* padding to 16x16 pixels */
    width = ((config->width + 15) >> 4) << 4;
    height = ((config->height + 15) >> 4) << 4;

    size = width * height;
    if (y_size)
        *y_size = size;
    if (u_size)
        *u_size = size >> 2;
    if (v_size)
        *v_size = size >> 2;
    return r;
}

HAL_StatusTypeDef HAL_JPEGD_GetDim(uint8_t *input, uint32_t size, int *width, int *height)
{
    int w = -1, h = -1, i = 0;
    uint16_t len;

    while (i < size - 4)
    {
        if (input[i] == 0xFF)
        {
            len = LDB_WORD(input + i + 2);
            if (input[i + 1] == 0xD8)   // SOI
            {
                i += 2;
                continue;
            }
            if (input[i + 1] == 0xC0)   /* SOF0 (baseline JPEG) */
            {
                if (i < size - 9)
                {
                    h = LDB_WORD(input + i + 5);
                    w = LDB_WORD(input + i + 7);
                }
                break;
            }
            // Next tag
            i += len + 2;
        }
    }
    if (width)
        *width = w;
    if (height)
        *height = h;

    return HAL_OK;
}

#endif


/**
  * @} JPEGD
  */

/**
  * @} BF0_HAL_Driver
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

