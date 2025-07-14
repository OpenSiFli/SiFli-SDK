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

/** @addtogroup EZIP
  * @{
  */

#if defined(HAL_EZIP_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

#define IS_REAL_INSTANCE(ins) (((uint32_t)(EZIP)) == ((uint32_t)(ins)))


#define EZIP_CLEAR_INT_STATUS(inst)                                    \
    do                                                                 \
    {                                                                  \
        inst->INT_MASK = 0xFFFFFFFF;                                   \
        /* from LITE on, INT_STA is set even if INT_EN is not set */   \
        inst->INT_STA = 0xFFFFFFFF;                                    \
    }                                                                  \
    while (0);


static void EZIP_ENABLE(EZIP_HandleTypeDef *ezip)
{
    if (!IS_REAL_INSTANCE(ezip->Instance)) return;

    HAL_RCC_EnableModule(RCC_MOD_EZIP);
}

static void EZIP_DISABLE(EZIP_HandleTypeDef *ezip)
{
    if (!IS_REAL_INSTANCE(ezip->Instance)) return;

    HAL_RCC_DisableModule(RCC_MOD_EZIP);
}

static void EZIP_DISABLE_CHECK(EZIP_HandleTypeDef *ezip)
{
    if (!IS_REAL_INSTANCE(ezip->Instance)) return;

#ifndef SF32LB55X
    /* if decoding APNG frame not disable EZIP after decoding complete
     * close or suspend API would disable EZIP
     */
    if ((ezip->Instance->DB_DATA3 >> 24) != 5)
#endif
    {
        HAL_RCC_DisableModule(RCC_MOD_EZIP);
    }
}

#ifdef EZIP_DEBUG
static inline EZIP_DecodeConfigTypeDef *EZIP_AllocRecordItem(EZIP_HandleTypeDef *ezip)
{
    ezip->op_hist.idx = (ezip->op_hist.idx + 1) & (EZIP_OP_HIST_DEPTH - 1);
    return &ezip->op_hist.hist[ezip->op_hist.idx];
}
static void EZIP_RecordOp(EZIP_HandleTypeDef *ezip, EZIP_DecodeConfigTypeDef *config)
{
    EZIP_DecodeConfigTypeDef *item;

    item = EZIP_AllocRecordItem(ezip);

    memcpy((void *)item, (void *)config, sizeof(*item));
}
#endif /* EZIP_DEBUG */

static HAL_StatusTypeDef HAL_EZIP_ConfigDecode(EZIP_HandleTypeDef *ezip, EZIP_DecodeConfigTypeDef *config)
{
    uint16_t col_start;
    uint16_t col_end;
    uint16_t row_start;
    uint16_t row_end;
    uint32_t mode;
    FLASH_HandleTypeDef *flash_handle;
    uint32_t flash_addr;
#ifdef HAL_EZIP_NON_CONT_MODE_SUPPORTED
    EZIP_BlockListTypeDef *block_list;
#endif /* HAL_EZIP_NON_CONT_MODE_SUPPORTED */
    EZIP_WorkModeTypeDef data_type;
    uint8_t addr_type;

#ifdef EZIP_DEBUG
    EZIP_RecordOp(ezip, config);
#endif

    data_type = config->work_mode & HAL_EZIP_MODE_DATA_TYPE_MASK;
    if (data_type >= HAL_EZIP_MODE_DATA_TYPE_NUM)
    {
        return HAL_ERROR;
    }

    if ((HAL_EZIP_MODE_EZIP != data_type) && (HAL_EZIP_OUTPUT_AHB != config->output_mode))
    {
        return HAL_ERROR;
    }
    if ((uint32_t)config->input & 3)
    {
        return HAL_ERROR;
    }
    if ((HAL_EZIP_OUTPUT_AHB == config->output_mode) && ((uint32_t)config->output & 3))
    {
        return HAL_ERROR;
    }


#ifdef HAL_EZIP_NON_CONT_MODE_SUPPORTED
    addr_type = config->work_mode & HAL_EZIP_MODE_ADDR_TYPE_MASK;
    if ((HAL_EZIP_MODE_ADDR_NON_CONT_TYPE == addr_type)
            && (!ezip->flash_handle_query_cb))
    {
        return HAL_ERROR;
    }
#else
    addr_type = HAL_EZIP_MODE_ADDR_CONT_TYPE;
#endif /* HAL_EZIP_NON_CONT_MODE_SUPPORTED */

    if (HAL_EZIP_OUTPUT_EPIC == config->output_mode)
    {
        ezip->Instance->EZIP_PARA = (EZIP_PARA_OUT_EPIC << EZIP_EZIP_PARA_OUT_SEL_Pos);
    }
    else if (HAL_EZIP_OUTPUT_AHB == config->output_mode)
    {
        if (HAL_EZIP_MODE_EZIP == data_type)
        {
            mode = EZIP_PARA_MOD_EZIP;
        }
        else if (HAL_EZIP_MODE_GZIP == data_type)
        {
            mode = EZIP_PARA_MOD_GZIP;
        }
        else
        {
            mode = EZIP_PARA_MOD_LZ4;
        }
        ezip->Instance->EZIP_PARA = (EZIP_PARA_OUT_AHB << EZIP_EZIP_PARA_OUT_SEL_Pos) | (mode << EZIP_EZIP_PARA_MOD_SEL_Pos);
        ezip->Instance->DST_ADDR = (uint32_t)config->output;
    }
    else
    {
        // do nothing
    }

    ezip->Instance->EZIP_PARA &= ~EZIP_EZIP_PARA_CACHE_EN;

#ifdef HAL_EZIP_NON_CONT_MODE_SUPPORTED
    ezip->non_cont_block_start_addr = 0;
#endif /* HAL_EZIP_NON_CONT_MODE_SUPPORTED */

    if ((HAL_EZIP_MODE_ADDR_CONT_TYPE == addr_type)
            && (ezip->flash_handle_query_cb))
    {
#ifndef SF32LB55X
        flash_addr = HCPU_MPI_CBUS_ADDR((uint32_t)config->input);
#else
        flash_addr = (uint32_t)config->input;
#endif
        flash_handle = ezip->flash_handle_query_cb(flash_addr);

#ifdef HAL_USE_NAND
        if (flash_handle && flash_handle->isNand)
        {
            if ((uint32_t)config->input & 2047)
            {
                return HAL_ERROR;
            }
            ezip->Instance->EZIP_PARA |= (EZIP_EZIP_PARA_IN_SEL);
            ezip->Instance->SRC_LEN = config->input_data_size;
#ifdef SF32LB58X
            if (flash_handle->Instance == hwp_qspi4)
            {
                ezip->Instance->EZIP_PARA &= ~EZIP_EZIP_PARA_SPI_SEL;
            }
            else
            {
                ezip->Instance->EZIP_PARA |= EZIP_EZIP_PARA_SPI_SEL;
            }
#elif defined(SF32LB56X) || defined(SF32LB52X)
            if (flash_handle->Instance == hwp_qspi1)
            {
                ezip->Instance->EZIP_PARA &= ~EZIP_EZIP_PARA_SPI_SEL;
            }
            else
            {
                ezip->Instance->EZIP_PARA |= EZIP_EZIP_PARA_SPI_SEL;
            }
#else
            if (flash_handle->Instance == hwp_qspi3)
            {
                ezip->Instance->EZIP_PARA &= ~EZIP_EZIP_PARA_SPI_SEL;
            }
            else
            {
                ezip->Instance->EZIP_PARA |= EZIP_EZIP_PARA_SPI_SEL;
            }
#endif   /* SF32LB58X */

            HAL_NAND_SET_CONTINUE(flash_handle, flash_addr, 1, config->input_data_size);
        }
#endif
    }
#ifdef HAL_EZIP_NON_CONT_MODE_SUPPORTED
    else if (HAL_EZIP_MODE_ADDR_NON_CONT_TYPE == addr_type)
    {
        block_list = (EZIP_BlockListTypeDef *)config->input;
        flash_addr = HCPU_MPI_CBUS_ADDR(block_list->base_addr);

        flash_handle = ezip->flash_handle_query_cb(flash_addr);
        if (flash_handle)
        {
            ezip->non_cont_block_start_addr = flash_addr;
            ezip->Instance->EZIP_PARA |= (EZIP_EZIP_PARA_IN_SEL);
            ezip->Instance->SRC_LEN = config->input_data_size;
            if (flash_handle->Instance == hwp_qspi1)
            {
                ezip->Instance->EZIP_PARA &= ~EZIP_EZIP_PARA_SPI_SEL;
            }
            else
            {
                ezip->Instance->EZIP_PARA |= EZIP_EZIP_PARA_SPI_SEL;
            }

            HAL_MPI_FORCE_CONTINUE(flash_handle, block_list->block_offset, block_list->block_num);
        }
        else
        {
            return HAL_ERROR;
        }
    }
#endif /* HAL_EZIP_NON_CONT_MODE_SUPPORTED */

#ifndef SF32LB55X
    if (addr_type == HAL_EZIP_MODE_ADDR_CONT_TYPE)
    {
        ezip->Instance->SRC_ADDR = HCPU_MPI_SBUS_ADDR(config->input);
    }
    else
    {
        ezip->Instance->SRC_ADDR = 0;
    }
#else
    ezip->Instance->SRC_ADDR = (uint32_t)config->input;
#endif /* SF32LB55X */

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

    ezip->Instance->START_POINT = MAKE_REG_VAL(col_start, EZIP_START_POINT_START_COL_Msk, EZIP_START_POINT_START_COL_Pos)
                                  | MAKE_REG_VAL(row_start, EZIP_START_POINT_START_ROW_Msk, EZIP_START_POINT_START_ROW_Pos);

    ezip->Instance->END_POINT = MAKE_REG_VAL(col_end, EZIP_END_POINT_END_COL_Msk, EZIP_END_POINT_END_COL_Pos)
                                | MAKE_REG_VAL(row_end, EZIP_END_POINT_END_ROW_Msk, EZIP_END_POINT_END_ROW_Pos);

    /* workaround: set GREY_PARA to white as no API for setting this register yet */
#ifdef EZIP_GREY_PARA_GREY_PARA_Msk
    MODIFY_REG(ezip->Instance->GREY_PARA, EZIP_GREY_PARA_GREY_PARA_Msk,
               MAKE_REG_VAL(0xFFFFFF, EZIP_GREY_PARA_GREY_PARA_Msk, EZIP_GREY_PARA_GREY_PARA_Pos));
#endif /* EZIP_GREY_PARA_GREY_PARA_Msk */

    return HAL_OK;
}

#ifndef SF32LB55X
static HAL_StatusTypeDef HAL_EZIP_EnableAEZIP(EZIP_HandleTypeDef *ezip)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    if (ezip->Instance->AEZIP_CTRL & EZIP_AEZIP_CTRL_AEZIP_CTRL)
    {
        /* already enabled */
        return HAL_ERROR;
    }

    /* clear old status */
    EZIP_CLEAR_INT_STATUS(ezip->Instance);

    /* start decoding header */
    ezip->Instance->AEZIP_CTRL = EZIP_AEZIP_CTRL_AEZIP_CTRL;

    while (0 == (ezip->Instance->INT_MASK
                 & (EZIP_INT_MASK_END_INT_MASK
                    | EZIP_INT_MASK_ROW_ERR_MASK
                    | EZIP_INT_MASK_BTYPE_ERR_MASK
                    | EZIP_INT_MASK_ETYPE_ERR_MASK)))
    {
        /* wait */
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    }

    if (ezip->Instance->INT_MASK & EZIP_INT_MASK_END_INT_MASK)
    {
        status = HAL_OK;
    }

    if (ezip->Instance->INT_MASK & EZIP_INT_MASK_BTYPE_ERR_MASK)
    {
        status = HAL_ERROR;
        ezip->ErrorCode |= EZIP_INT_MASK_BTYPE_ERR_MASK;
    }

    if (ezip->Instance->INT_MASK & EZIP_INT_MASK_ROW_ERR_MASK)
    {
        status = HAL_ERROR;
        ezip->ErrorCode |= EZIP_INT_MASK_ROW_ERR_MASK;
    }

    if (ezip->Instance->INT_MASK & EZIP_INT_MASK_ETYPE_ERR_MASK)
    {
        status = HAL_ERROR;
        ezip->ErrorCode |= EZIP_INT_MASK_ETYPE_ERR_MASK;
    }

    return status;
}

static HAL_StatusTypeDef HAL_EZIP_DisableAEZIP(EZIP_HandleTypeDef *ezip)
{
    if (0 == (ezip->Instance->AEZIP_CTRL & EZIP_AEZIP_CTRL_AEZIP_CTRL))
    {
        /* already disabled */
        return HAL_ERROR;
    }

    /* clear old status */
    EZIP_CLEAR_INT_STATUS(ezip->Instance);

    ezip->Instance->AEZIP_CTRL &= ~EZIP_AEZIP_CTRL_AEZIP_CTRL;

    while (0 == (ezip->Instance->INT_MASK & (EZIP_INT_MASK_AEZIP_INT_MASK)))
    {
        /* wait */
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
        __NOP();
    }

    return HAL_OK;
}

static void HAL_EZIP_ReadNextFrameInfo(EZIP_HandleTypeDef *ezip,
                                       EZIP_EZIPAFrameInfoTypeDef *frame)
{
    uint32_t val;

    frame->seq_num = READ_REG(ezip->Instance->SEQ_NUM);

    val = ezip->Instance->FRAME_AREA;
    frame->width = GET_REG_VAL(val, EZIP_FRAME_AREA_FRAME_WIDTH_Msk, EZIP_FRAME_AREA_FRAME_WIDTH_Pos);
    frame->height = GET_REG_VAL(val, EZIP_FRAME_AREA_FRAME_HEIGHT_Msk, EZIP_FRAME_AREA_FRAME_HEIGHT_Pos);

    val = ezip->Instance->FRAME_OFFSET;
    frame->x_offset = GET_REG_VAL(val, EZIP_FRAME_OFFSET_OFFSET_COL_Msk, EZIP_FRAME_OFFSET_OFFSET_COL_Pos);
    frame->y_offset = GET_REG_VAL(val, EZIP_FRAME_OFFSET_OFFEST_ROW_Msk, EZIP_FRAME_OFFSET_OFFEST_ROW_Pos);

    val = ezip->Instance->FRAME_DELAY;
    frame->delay_num = GET_REG_VAL(val, EZIP_FRAME_DELAY_DELAY_NUM_Msk, EZIP_FRAME_DELAY_DELAY_NUM_Pos);
    frame->delay_den = GET_REG_VAL(val, EZIP_FRAME_DELAY_DELAY_DEN_Msk, EZIP_FRAME_DELAY_DELAY_DEN_Pos);

    val = ezip->Instance->FRAME_TYPE;
    frame->dispose_op = GET_REG_VAL(val, EZIP_FRAME_TYPE_DISPOSE_OP_Msk, EZIP_FRAME_TYPE_DISPOSE_OP_Pos);
    frame->blend_op = GET_REG_VAL(val, EZIP_FRAME_TYPE_BLEND_OP_Msk, EZIP_FRAME_TYPE_BLEND_OP_Pos);

}
#endif /* SF32LB55X */


HAL_StatusTypeDef HAL_EZIP_Init(EZIP_HandleTypeDef *ezip)
{
    if (HAL_EZIP_STATE_RESET != ezip->State)
    {
        return HAL_ERROR;
    }

#ifndef SF32LB55X
    HAL_RCC_EnableModule(RCC_MOD_EZIP);
#endif /* SF32LB55X */

    ezip->RamInstance_used = 0;
    ezip->HwInstance = ezip->Instance;

    ezip->State = HAL_EZIP_STATE_READY;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_EZIP_DeInit(EZIP_HandleTypeDef *ezip)
{
    if (HAL_EZIP_STATE_READY != ezip->State)
    {
        return HAL_ERROR;
    }
    ezip->RamInstance_used = 0;
    ezip->State = HAL_EZIP_STATE_RESET;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_EZIP_IRQHandler(EZIP_HandleTypeDef *ezip)
{
    uint32_t status;
    bool err = false;
    uint32_t addr;
    bool disabled = false;

    status = ezip->Instance->INT_STA;

    ezip->Instance->INT_STA = status;

    if (EZIP_INT_STA_END_INT_STA & status)
    {
        if (ezip->Instance->EZIP_PARA & EZIP_EZIP_PARA_IN_SEL)
        {
            ezip->Instance->EZIP_PARA &= ~EZIP_EZIP_PARA_IN_SEL;
            if (ezip->flash_handle_query_cb)
            {
#ifdef HAL_EZIP_NON_CONT_MODE_SUPPORTED
                if (ezip->non_cont_block_start_addr) /* it's Non-Continuous Mode */
                {
                    HAL_MPI_QUIT_CONTINUE(ezip->flash_handle_query_cb(ezip->non_cont_block_start_addr));
                }
                else
#endif /* HAL_EZIP_NON_CONT_MODE_SUPPORTED */
                {
#ifdef HAL_USE_NAND
#ifndef SF32LB55X
                    addr = HCPU_MPI_CBUS_ADDR(ezip->Instance->SRC_ADDR);
#else
                    addr = ezip->Instance->SRC_ADDR;
#endif /* SF32LB55X */
                    HAL_NAND_SET_CONTINUE(ezip->flash_handle_query_cb(addr), addr, 0, 0);
#endif
                }
            }
        }
        ezip->Instance->INT_EN = 0;
        EZIP_DISABLE_CHECK(ezip);
        disabled = true;
        ezip->State = HAL_EZIP_STATE_READY;
        if (ezip->CpltCallback)
        {
            ezip->CpltCallback(ezip);
        }
    }

    if (EZIP_INT_STA_ROW_ERR_STA & status)
    {
        err = true;
    }

    if (EZIP_INT_STA_BTYPE_ERR_STA & status)
    {
        err = true;
    }

    if (EZIP_INT_STA_ETYPE_ERR_STA & status)
    {
        err = true;
    }
    if (err)
    {
        if (!disabled)
        {
            EZIP_DISABLE_CHECK(ezip);
        }
        ezip->State = HAL_EZIP_STATE_ERROR;
        ezip->ErrorCode = status;
        //HAL_ASSERT(0);
        return HAL_ERROR;
    }
    else
    {
        return HAL_OK;
    }
}

HAL_StatusTypeDef HAL_EZIP_Decode(EZIP_HandleTypeDef *ezip, EZIP_DecodeConfigTypeDef *config)
{
    HAL_StatusTypeDef status;
    uint32_t flash_addr;
    EZIP_OutputModeTypeDef temp_output_mode = config->output_mode;

    if (HAL_EZIP_STATE_READY != ezip->State)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    EZIP_ENABLE(ezip);

    if (0 != ezip->Instance->EZIP_CTRL)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    ezip->State = HAL_EZIP_STATE_BUSY;

    status = HAL_EZIP_ConfigDecode(ezip, config);

    if (HAL_OK != status)
    {
        ezip->State = HAL_EZIP_STATE_READY;
        goto __EXIT;
    }

    /* clear old status */
    EZIP_CLEAR_INT_STATUS(ezip->Instance);

    ezip->Instance->EZIP_CTRL = EZIP_EZIP_CTRL_EZIP_CTRL;

    if (HAL_EZIP_OUTPUT_AHB == temp_output_mode)
    {
        while (0 == ezip->Instance->INT_MASK)
        {
        }
        if (EZIP_INT_STA_END_INT_STA & ezip->Instance->INT_MASK)
        {
            if (ezip->Instance->EZIP_PARA & EZIP_EZIP_PARA_IN_SEL)
            {
                ezip->Instance->EZIP_PARA &= ~EZIP_EZIP_PARA_IN_SEL;
                if (ezip->flash_handle_query_cb)
                {
#ifdef HAL_EZIP_NON_CONT_MODE_SUPPORTED
                    if (ezip->non_cont_block_start_addr)
                    {
                        HAL_MPI_QUIT_CONTINUE(ezip->flash_handle_query_cb(ezip->non_cont_block_start_addr));
                    }
                    else
#endif /* HAL_EZIP_NON_CONT_MODE_SUPPORTED */
                    {
#ifdef HAL_USE_NAND
#ifndef SF32LB55X
                        flash_addr = HCPU_MPI_CBUS_ADDR((uint32_t)config->input);
#else
                        flash_addr = (uint32_t)config->input;
#endif /*SF32LB55X */
                        HAL_NAND_SET_CONTINUE(ezip->flash_handle_query_cb(flash_addr), flash_addr, 0, 0);
#endif
                    }
                }
            }
            ezip->State = HAL_EZIP_STATE_READY;
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
        ezip->State = HAL_EZIP_STATE_READY;
        status = HAL_OK;
    }

__EXIT:

    EZIP_DISABLE_CHECK(ezip);

    return status;

}

HAL_StatusTypeDef HAL_EZIP_Decode_IT(EZIP_HandleTypeDef *ezip, EZIP_DecodeConfigTypeDef *config)
{
    HAL_StatusTypeDef status;

    if (HAL_EZIP_STATE_READY != ezip->State)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    EZIP_ENABLE(ezip);

    ezip->State = HAL_EZIP_STATE_BUSY;

    status = HAL_EZIP_ConfigDecode(ezip, config);

    if (HAL_OK != status)
    {
        EZIP_DISABLE(ezip);
        ezip->State = HAL_EZIP_STATE_READY;
        goto __EXIT;
    }

    /* clear old status */
    EZIP_CLEAR_INT_STATUS(ezip->Instance);

    ezip->Instance->INT_EN = EZIP_INT_EN_END_INT_EN | EZIP_INT_EN_ROW_ERR_EN
                             | EZIP_INT_EN_BTYPE_ERR_EN | EZIP_INT_EN_ETYPE_ERR_EN;

    ezip->Instance->EZIP_CTRL = EZIP_EZIP_CTRL_EZIP_CTRL;

    status = HAL_OK;

__EXIT:

    return status;
}

HAL_StatusTypeDef HAL_EZIP_CheckReady(EZIP_HandleTypeDef *ezip)
{
    HAL_StatusTypeDef status;

    if (HAL_EZIP_STATE_READY != ezip->State)
    {
        status = HAL_BUSY;
        goto __EXIT;
    }

    status = HAL_OK;
__EXIT:

    return status;

}

#ifndef SF32LB55X
HAL_StatusTypeDef HAL_EZIP_OpenEZIPA(EZIP_HandleTypeDef *ezip, uint8_t *data,
                                     EZIP_EZIPAHeaderTypeDef *header,
                                     EZIP_EZIPAFrameInfoTypeDef *frame)
{
    HAL_StatusTypeDef status;

    if (!header || !frame || !data)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    if (HAL_EZIP_STATE_READY != ezip->State)
    {
        status = HAL_BUSY;
        goto __EXIT;
    }

    EZIP_ENABLE(ezip);

    if (0 != ezip->Instance->EZIP_CTRL)
    {
        status = HAL_BUSY;
        goto __EXIT;
    }

    ezip->State = HAL_EZIP_STATE_BUSY;
    ezip->ErrorCode = 0;

    ezip->Instance->SRC_ADDR = HCPU_MPI_SBUS_ADDR(data);

    ezip->Instance->EZIP_PARA = (EZIP_PARA_MOD_EZIP << EZIP_EZIP_PARA_MOD_SEL_Pos) | (EZIP_PARA_OUT_EPIC << EZIP_EZIP_PARA_OUT_SEL_Pos);
    ezip->Instance->FRAME_START = 1;
    ezip->Instance->PLAY_START = 1;

    /* clear old status */
    EZIP_CLEAR_INT_STATUS(ezip->Instance);

    /* start decoding file header and first frame header */
    status = HAL_EZIP_EnableAEZIP(ezip);

    if (HAL_OK == status)
    {
        header->frame_num = READ_REG(ezip->Instance->FRAME_NUM);
        header->play_num = READ_REG(ezip->Instance->PLAY_NUM);
        header->width = (ezip->Instance->DB_DATA1 >> 16) & 0xFFFF;
        header->height = (ezip->Instance->DB_DATA1 & 0xFFFF);
        header->bit_depth = (ezip->Instance->DB_DATA0 >> 24) & 0xFF;
        header->color_type = (ezip->Instance->DB_DATA0 >> 16) & 0xFF;

        HAL_EZIP_ReadNextFrameInfo(ezip, frame);
        ezip->State = HAL_EZIP_STATE_READY;
    }
    else
    {
        ezip->State = HAL_EZIP_STATE_ERROR;
    }

__EXIT:

    /* not disable EZIP as it may be still accessing bus after END_INT,
     *  ezip will be disabled by HAL_EZIP_CloseEZIPA or HAL_EZIP_SuspendEZIPA
     *
     */
    //EZIP_DISABLE();

    return status;
}


HAL_StatusTypeDef HAL_EZIP_CloseEZIPA(EZIP_HandleTypeDef *ezip)
{
    HAL_StatusTypeDef status;

    if (HAL_EZIP_STATE_READY != ezip->State)
    {
        status = HAL_BUSY;
        goto __EXIT;
    }

    EZIP_ENABLE(ezip);

    if (0 == (ezip->Instance->AEZIP_CTRL & EZIP_AEZIP_CTRL_AEZIP_CTRL))
    {
        status = HAL_OK;
        goto __EXIT;
    }

    ezip->State = HAL_EZIP_STATE_BUSY;

    status = HAL_EZIP_DisableAEZIP(ezip);

    if (HAL_OK == status)
    {
        ezip->State = HAL_EZIP_STATE_READY;
    }
    else
    {
        ezip->State = HAL_EZIP_STATE_ERROR;
    }
__EXIT:
    /* It's safe to disable EZIP after writing aezip_ctrl=0 and END_INT comes */
    EZIP_DISABLE(ezip);
    return status;
}


HAL_StatusTypeDef HAL_EZIP_GetNextFrameInfo(EZIP_HandleTypeDef *ezip,
        EZIP_EZIPAFrameInfoTypeDef *frame)
{
    HAL_StatusTypeDef status;


    if (!frame)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    if (HAL_EZIP_STATE_READY != ezip->State)
    {
        status = HAL_BUSY;
        goto __EXIT;
    }

    EZIP_ENABLE(ezip);

    if (0 == ezip->Instance->AEZIP_CTRL)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    HAL_EZIP_ReadNextFrameInfo(ezip, frame);

    status = HAL_OK;

__EXIT:

    /* not disable EZIP as it may be still accessing bus after END_INT,
     *  ezip will be disabled by HAL_EZIP_CloseEZIPA or HAL_EZIP_SuspendEZIPA
     *
     */
    //EZIP_DISABLE(ezip);

    return status;
}

HAL_StatusTypeDef HAL_EZIP_SuspendEZIPA(EZIP_HandleTypeDef *ezip)
{
    HAL_StatusTypeDef status;

    if (!(ezip && ezip->Instance))
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    if (HAL_EZIP_STATE_READY != ezip->State)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    EZIP_ENABLE(ezip);

    if (0 == (ezip->Instance->AEZIP_CTRL & EZIP_AEZIP_CTRL_AEZIP_CTRL))
    {
        status = HAL_OK;
        goto __EXIT;
    }

    ezip->State = HAL_EZIP_STATE_BUSY;

    status = HAL_EZIP_DisableAEZIP(ezip);

    if (HAL_OK == status)
    {
        ezip->State = HAL_EZIP_STATE_READY;
    }
    else
    {
        ezip->State = HAL_EZIP_STATE_ERROR;
    }

__EXIT:
    /* It's safe to disable EZIP after writing aezip_ctrl=0 and END_INT comes */
    EZIP_DISABLE(ezip);

    return status;
}


HAL_StatusTypeDef HAL_EZIP_ResumeEZIPA(EZIP_HandleTypeDef *ezip, const uint8_t *data,
                                       uint32_t seq_num, uint32_t play_loop_idx)
{
    HAL_StatusTypeDef status;

    if (!(ezip && ezip->Instance))
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    if (HAL_EZIP_STATE_READY != ezip->State)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    EZIP_ENABLE(ezip);

    if (ezip->Instance->AEZIP_CTRL & EZIP_AEZIP_CTRL_AEZIP_CTRL)
    {
        status = HAL_OK;
        goto __EXIT;
    }

    ezip->State = HAL_EZIP_STATE_BUSY;

    ezip->Instance->SRC_ADDR = HCPU_MPI_SBUS_ADDR(data);
    ezip->Instance->FRAME_START = seq_num + 1;
    ezip->Instance->PLAY_START = play_loop_idx + 1;

    /* start decoding header and specified frame header */
    status = HAL_EZIP_EnableAEZIP(ezip);

    if (HAL_OK == status)
    {
        ezip->State = HAL_EZIP_STATE_READY;
    }
    else
    {
        ezip->State = HAL_EZIP_STATE_ERROR;
    }

__EXIT:
    /* not disable EZIP as it may be still accessing bus after END_INT,
     *  ezip will be disabled by HAL_EZIP_CloseEZIPA or HAL_EZIP_SuspendEZIPA
     *
     */
    //EZIP_DISABLE();

    return status;
}


#endif /* SF32LB55X */

HAL_StatusTypeDef HAL_EZIP_DecodeFast_IT(EZIP_HandleTypeDef *hezip)
{
    HAL_StatusTypeDef status;
    HAL_ASSERT(NULL != hezip->HwInstance);
    HAL_ASSERT(NULL != hezip->RamInstance);
    HAL_ASSERT(hezip->RamInstance_used);


    if (HAL_EZIP_STATE_READY != hezip->State)
    {
        status = HAL_ERROR;
        goto __EXIT;
    }

    hezip->Instance = hezip->HwInstance;


    EZIP_ENABLE(hezip);
    hezip->State = HAL_EZIP_STATE_BUSY;


    hezip->RamInstance_used = 0;
    hezip->Instance = hezip->HwInstance;

    /* clear old status */
    EZIP_CLEAR_INT_STATUS(hezip->Instance);

    memcpy((void *)&hezip->Instance->SRC_ADDR, (const void *)&hezip->RamInstance->SRC_ADDR,
           ((uint32_t)&hezip->Instance->DB_SEL) - ((uint32_t)&hezip->Instance->SRC_ADDR));

    hezip->Instance->EZIP_CTRL = EZIP_EZIP_CTRL_EZIP_CTRL;

    status = HAL_OK;

__EXIT:

    return status;
}


HAL_StatusTypeDef HAL_EZIP_DecodeFast_Init(EZIP_HandleTypeDef *hezip)
{

    hezip->RamInstance_used = 1;
    hezip->Instance = hezip->RamInstance;
    hezip->Instance->INT_EN = 0;

    hezip->State = HAL_EZIP_STATE_READY;

    return HAL_OK;
}


#endif


/**
  * @} EZIP
  */

/**
  * @} BF0_HAL_Driver
  */

/**
  * @}
  */