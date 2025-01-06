/**
  ******************************************************************************
  * @file   bf0_hal_aes.c
  * @author Sifli software development team
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

#include <string.h>                 // for memcpy
#include "bf0_hal.h"
#include "register.h"
#include "bf0_hal_aes.h"

static pAESCallback irq_cb = NULL;

HAL_StatusTypeDef HAL_AES_Regist_IT_cb(pAESCallback cb)
{
#if defined(HAL_AES_MODULE_ENABLED)||defined(HAL_HASH_MODULE_ENABLED)
    if (HAL_AES_busy()) return HAL_BUSY;
#endif /* HAL_AES_MODULE_ENABLED */

    irq_cb = cb;

    return HAL_OK;
}

#if defined(HAL_AES_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

int HAL_AES_reset(void)
{
    hwp_aes_acc->COMMAND = AES_ACC_COMMAND_AES_ACC_RESET;
    hwp_aes_acc->COMMAND = 0;
    return 0;
}

__HAL_ROM_USED int HAL_AES_init(uint32_t *key, int key_size, uint32_t *iv, uint32_t mode)
{
    int r = 0;
    int ks;
    uint32_t setting;

    if (HAL_AES_busy())
        HAL_AES_reset();

    if (key == NULL)     // Use root key
    {
        setting = AES_ACC_AES_SETTING_KEY_SEL;
    }
    else
    {
        setting = 0;
    }

    switch (key_size)
    {
    case 16:
        ks = AES_KEY_LEN_128;
        break;
    case 24:
        ks = AES_KEY_LEN_192;
        break;
    case 32:
        ks = AES_KEY_LEN_256;
        break;
    default:
        ks = -1;
    }
    if (ks >= 0)
    {
        volatile uint32_t *p;
        int i;
        if (key)
        {
            p = &(hwp_aes_acc->EXT_KEY_W0);
            for (i = 0; i < (key_size >> 2); i++, p++)
                *p = key[i];
        }
        setting |= (mode | (ks << AES_ACC_AES_SETTING_AES_LENGTH_Pos));
        hwp_aes_acc->AES_SETTING = setting;
        if (mode != AES_MODE_ECB && iv)
        {
            p = &(hwp_aes_acc->IV_W0);
            for (i = 0; i < 4; i++, p++)
                *p = iv[i];
        }
    }
    else
        r = -1;

    return r;
}

static void HAL_AES_run_help(uint8_t enc, uint8_t *in_data, uint8_t *out_data, int size)
{
    hwp_aes_acc-> DMA_IN = (uint32_t)HCPU_MPI_SBUS_ADDR(in_data);
    hwp_aes_acc-> DMA_OUT = (uint32_t)HCPU_MPI_SBUS_ADDR(out_data);
    hwp_aes_acc-> DMA_DATA = ((size + 15) >> 4);
    if (enc)
        hwp_aes_acc-> AES_SETTING |= (enc << AES_ACC_AES_SETTING_AES_OP_MODE_Pos);
    else
        hwp_aes_acc-> AES_SETTING &= ~(enc << AES_ACC_AES_SETTING_AES_OP_MODE_Pos);

    mpu_dcache_clean((void *)in_data, size);
    hwp_aes_acc-> COMMAND |= AES_ACC_COMMAND_START;
}

static HAL_StatusTypeDef  HAL_AES_check(uint32_t in_data, uint32_t out_data)
{
    HAL_StatusTypeDef r = HAL_OK;
    if (HPSYS_RAM_IN_ITCM(in_data))
        r = HAL_ERROR;
    else if (out_data && HPSYS_RAM_IN_ITCM(out_data))
        r = HAL_ERROR;
    return r;
}
__HAL_ROM_USED HAL_StatusTypeDef  HAL_AES_run_IT(uint8_t enc, uint8_t *in_data, uint8_t *out_data, int size)
{
    HAL_StatusTypeDef status;

    status = HAL_AES_check((uint32_t)in_data, (uint32_t)out_data);
    if (status == HAL_OK)
    {
        hwp_aes_acc->IRQ = hwp_aes_acc->IRQ;                // Clear all pending IRQ state
        hwp_aes_acc->SETTING = (AES_ACC_SETTING_DONE_IRQ_MASK | AES_ACC_SETTING_BUS_ERR_IRQ_MASK | AES_ACC_SETTING_SETUP_ERR_IRQ_MASK);
        HAL_AES_run_help(enc, in_data, out_data, size);
    }
    return status;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_AES_run(uint8_t enc, uint8_t *in_data, uint8_t *out_data, int size)
{
    HAL_StatusTypeDef status;

    status = HAL_AES_check((uint32_t)in_data, (uint32_t)out_data);
    if (status == HAL_OK)
    {
        NVIC_DisableIRQ(AES_IRQn);
        hwp_aes_acc->SETTING = 0;
        HAL_AES_run_help(enc, in_data, out_data, size);
        while (hwp_aes_acc->STATUS & AES_ACC_STATUS_BUSY);
        if (hwp_aes_acc->IRQ & (AES_ACC_IRQ_BUS_ERR_STAT | AES_ACC_IRQ_SETUP_ERR_STAT))
            status = HAL_ERROR;
    }
    return status;
}

void HAL_AES_IRQHandler(void)
{
    if (!hwp_aes_acc->IRQ)
    {
        return;
    }
    hwp_aes_acc->IRQ |= (AES_ACC_IRQ_DONE_STAT | AES_ACC_IRQ_BUS_ERR_STAT);

    if (irq_cb)
    {
        pAESCallback  cb = irq_cb;
        irq_cb = NULL;

        if (cb) cb();
    }
}
#endif


#if defined(HAL_HASH_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

//#define DBG_HASH

static const uint8_t HASH_SIZE[] =
{
    HASH_SHA1_SIZE,
    HASH_SHA224_SIZE,
    HASH_SHA256_SIZE,
    HASH_SM3_SIZE,
};

/**
* @brief  Initialize AES Hash hardware block
* @param  iv initial vector, must be 32bit aligned, if NULL, use algorithm default initial vector.
* @param  algo hash algorithm
* @param  previous finished length
* @retval 0 if successful, otherwise -1
*/
int HAL_HASH_init(uint32_t *iv, uint8_t algo, uint32_t length)
{
    uint32_t config = 0;
    int r = -1;

    // Length must be 64bytes aligned.
    if (((length & 0x3f) == 0) && (algo <= HASH_ALGO_MAX))
    {
        HAL_HASH_reset();                                           // 0. Reset Hash module
        config = (algo << AES_ACC_HASH_SETTING_HASH_MODE_Pos);      // 1. Select alogrithm

        if (iv)                                                     // 2. Load IV
        {
            int len = HASH_SIZE[algo];
            memcpy((void *) & (hwp_aes_acc->HASH_IV_H0), iv, len);
            config |= AES_ACC_HASH_SETTING_DFT_IV_SEL;
        }
        config |= AES_ACC_HASH_SETTING_HASH_IV_LOAD;
        hwp_aes_acc->HASH_SETTING = config;

        if (length)                                                 // 3. Then load length.
        {
            hwp_aes_acc->HASH_LEN_L = length;
            config |= AES_ACC_HASH_SETTING_HASH_LEN_LOAD;
        }
        hwp_aes_acc->HASH_SETTING = config;

        r = 0;
        HAL_DBG_printf("Load IV and length HASH_SETTING=%08X, iv length=%d\n", config, length);
    }
    return r;
}

static HAL_StatusTypeDef HAL_HASH_run_help(uint8_t *in_data, int size, int final)
{

    hwp_aes_acc->HASH_DMA_IN = (uint32_t)HCPU_MPI_SBUS_ADDR(in_data);
    hwp_aes_acc->HASH_DMA_DATA = size;

    mpu_dcache_clean((void *)in_data, size);

    // Select padding.
    if (final)
        hwp_aes_acc->HASH_SETTING |= AES_ACC_HASH_SETTING_DO_PADDING;
    HAL_DBG_printf("HASH_SETTING=%08X\n", (uint32_t)hwp_aes_acc->HASH_SETTING);

#ifdef DBG_HASH
    HAL_DBG_printf("HASH_DMA_IN=%08X\n", (uint32_t)hwp_aes_acc->HASH_DMA_IN);
    HAL_DBG_printf("HASH_DMA_DATA=%08X\n", (uint32_t)hwp_aes_acc->HASH_DMA_DATA);
    HAL_DBG_printf("HASH_SETTING=%08X\n", (uint32_t)hwp_aes_acc->HASH_SETTING);
    HAL_DBG_printf("HASH_STATUS Before=%08X\n", (uint32_t)hwp_aes_acc->STATUS);
    HAL_DBG_printf("HASH_IRQ Before=%08X\n", (uint32_t)hwp_aes_acc->IRQ);
    HAL_DBG_print_data((char *)hwp_aes_acc, 0, sizeof(AES_ACC_TypeDef));
#endif
    hwp_aes_acc->COMMAND = AES_ACC_COMMAND_HASH_START;
    //HAL_DBG_printf("HASH_STATUS Before 2=%08X\n", (uint32_t)hwp_aes_acc->STATUS);

    return HAL_OK;

}


/**
* @brief  run Hash hardware
* @param  in_data input data, input data could not in ITCM or Retention memory
* @param  size length of input data in bytes.
* @param  final Last block
* @retval HAL_OK if successful, otherwise HAL_ERROR
*/
HAL_StatusTypeDef HAL_HASH_run(uint8_t *in_data, int size, int final)
{

    HAL_StatusTypeDef status;

    status = HAL_AES_check((uint32_t)in_data, (uint32_t)NULL);
    if (status == HAL_OK)
    {
        NVIC_DisableIRQ(AES_IRQn);
        HAL_HASH_run_help(in_data, size, final);
        while (hwp_aes_acc->STATUS & AES_ACC_STATUS_HASH_BUSY);
        hwp_aes_acc->IRQ = (hwp_aes_acc->IRQ & (AES_ACC_IRQ_HASH_BUS_ERR_STAT | AES_ACC_IRQ_HASH_PAD_ERR_STAT | AES_ACC_IRQ_HASH_DONE_STAT));
#ifdef DBG_HASH
        HAL_DBG_printf("SYNC HASH_STATUS=%08X\n", (uint32_t)hwp_aes_acc->STATUS);
        HAL_DBG_printf("SYNC HASH_IRQ=%08X\n", (uint32_t)hwp_aes_acc->IRQ);
#endif
        if (hwp_aes_acc->IRQ & (AES_ACC_IRQ_HASH_BUS_ERR_STAT | AES_ACC_IRQ_HASH_PAD_ERR_STAT))
            status = HAL_ERROR;
    }
    return status;
}

/**
* @brief  run Hash hardware, return immediately, generate AES HASH interrupt when done.
* @param  in_data input data, input data could not in ITCM or Retention memory
* @param  size length of input data in bytes.
* @param  final Last block
* @retval HAL_OK if successful, otherwise HAL_ERROR
*/
HAL_StatusTypeDef HAL_HASH_run_IT(uint8_t *in_data, int size, int final)
{
    HAL_StatusTypeDef status;

    status = HAL_AES_check((uint32_t)in_data, (uint32_t)NULL);
    if (status == HAL_OK)
    {
        hwp_aes_acc->IRQ = hwp_aes_acc->IRQ;                // Clear all pending IRQ state
        hwp_aes_acc->SETTING = (AES_ACC_SETTING_HASH_DONE_MASK | AES_ACC_SETTING_HASH_BUS_ERR_MASK | AES_ACC_SETTING_HASH_PAD_ERR_MASK);
        NVIC_EnableIRQ(AES_IRQn);
        HAL_HASH_run_help(in_data, size, final);
    }
    return status;
}

/**
* @brief  Reset AES HASH hardware block
* @retval 0 if successful, otherwise -1
*/
int HAL_HASH_reset(void)
{
    hwp_aes_acc->COMMAND = AES_ACC_COMMAND_HASH_RESET;
    hwp_aes_acc->COMMAND = 0;
    return 0;
}

/**
* @brief  Get hash result.
* @param  out_data output data, Size of out_data depends on hash algorithm
*/
void HAL_HASH_result(uint8_t *out_data)
{
    int len = HASH_SIZE[hwp_aes_acc->HASH_SETTING & AES_ACC_HASH_SETTING_HASH_MODE_Msk];
    memcpy(out_data, (const void *) & (hwp_aes_acc->HASH_RESULT_H0), len);
    HAL_DBG_printf("Result len=%d\n", hwp_aes_acc->HASH_RESULT_LEN_L);
}

/**
* @brief  AES HASH IRQ handler
*/
void HAL_HASH_IRQHandler(void)
{
    if (!hwp_aes_acc->IRQ)
    {
        return;
    }
    hwp_aes_acc->IRQ |= (AES_ACC_IRQ_HASH_DONE_STAT | AES_ACC_IRQ_HASH_BUS_ERR_STAT | AES_ACC_IRQ_HASH_PAD_ERR_STAT);

    if (irq_cb)
    {
        pAESCallback  cb = irq_cb;
        irq_cb = NULL;

        if (cb) cb();
    }
}
#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
