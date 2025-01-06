/**
  ******************************************************************************
  * @file   bf0_hal_i2c.c
  * @author Sifli software development team
  * @brief   I2C HAL module driver.
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

//extern  void rt_kprintf(const char *fmt, ...);

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup I2C I2C
  * @brief I2C HAL module driver
  * @{
  */

#if defined(HAL_I2C_MODULE_ENABLED) ||defined(_SIFLI_DOXYGEN_)
#define HAL_I2C_LOG(...)  //HAL_DBG_printf

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/** @defgroup I2C_Private_Define I2C Private Define
  * @{
  */
#define TIMING_CLEAR_MASK   (0xF0FFFFFFU)  /*!< I2C TIMING clear register Mask */
#define I2C_TIMEOUT_ADDR    (1000U)       /*!< 1s, IT/DMA mode send address timeout  */
#define I2C_TIMEOUT_BUSY    (1000U)       /*!< 1s, wait last UB timeout */

#define MAX_NBYTE_SIZE      511U
#define SlaveAddr_SHIFT     7U
#define SlaveAddr_MSK       0x06U

/* Private define for @ref PreviousState usage */
#define I2C_STATE_MSK             ((uint32_t)((uint32_t)((uint32_t)HAL_I2C_STATE_BUSY_TX | (uint32_t)HAL_I2C_STATE_BUSY_RX) & (uint32_t)(~((uint32_t)HAL_I2C_STATE_READY)))) /*!< Mask State define, keep only RX and TX bits            */
#define I2C_STATE_NONE            ((uint32_t)(HAL_I2C_MODE_NONE))                                                        /*!< Default Value                                          */
#define I2C_STATE_MASTER_BUSY_TX  ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_MASTER))            /*!< Master Busy TX, combinaison of State LSB and Mode enum */
#define I2C_STATE_MASTER_BUSY_RX  ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_MASTER))            /*!< Master Busy RX, combinaison of State LSB and Mode enum */
#define I2C_STATE_SLAVE_BUSY_TX   ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_SLAVE))             /*!< Slave Busy TX, combinaison of State LSB and Mode enum  */
#define I2C_STATE_SLAVE_BUSY_RX   ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_SLAVE))             /*!< Slave Busy RX, combinaison of State LSB and Mode enum  */
#define I2C_STATE_MEM_BUSY_TX     ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_MEM))               /*!< Memory Busy TX, combinaison of State LSB and Mode enum */
#define I2C_STATE_MEM_BUSY_RX     ((uint32_t)(((uint32_t)HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | (uint32_t)HAL_I2C_MODE_MEM))               /*!< Memory Busy RX, combinaison of State LSB and Mode enum */


/* Private define to centralize the enable/disable of Interrupts */
#define I2C_XFER_TX_IT          (0x00000001U)
#define I2C_XFER_RX_IT          (0x00000002U)
#define I2C_XFER_LISTEN_IT      (0x00000004U)

#define I2C_XFER_ERROR_IT       (0x00000011U)
#define I2C_XFER_CPLT_IT        (0x00000012U)
#define I2C_XFER_RELOAD_IT      (0x00000012U)

/* Private define Sequential Transfer Options default/reset value */
#define I2C_NO_OPTION_FRAME     (0xFFFF0000U)
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/

#define IS_MEM_MODE(MemAddSize) ((I2C_MEMADD_SIZE_8BIT == MemAddSize)||(I2C_MEMADD_SIZE_16BIT == MemAddSize))
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @defgroup I2C_Private_Functions I2C Private Functions
  * @{
  */
/* Private functions to handle DMA transfer */
static void I2C_DMAMasterTransmitCplt(DMA_HandleTypeDef *hdma);
static void I2C_DMAMasterReceiveCplt(DMA_HandleTypeDef *hdma);
#if HAL_I2CSLAVE_MODULE_ENABLED
    static void I2C_DMASlaveTransmitCplt(DMA_HandleTypeDef *hdma);
    static void I2C_DMASlaveReceiveCplt(DMA_HandleTypeDef *hdma);
#endif
static void I2C_DMAError(DMA_HandleTypeDef *hdma);
static void I2C_DMAAbort(DMA_HandleTypeDef *hdma);

#if HAL_I2CSLAVE_MODULE_ENABLED
    /* Private functions to handle IT transfer */
    static void I2C_ITAddrCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
#endif
#if 0
    static void I2C_ITMasterSequentialCplt(I2C_HandleTypeDef *hi2c);
#endif
static void I2C_ITMasterCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
#if HAL_I2CSLAVE_MODULE_ENABLED
    static void I2C_ITSlaveSequentialCplt(I2C_HandleTypeDef *hi2c);
    static void I2C_ITSlaveCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
    static void I2C_ITListenCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
#endif
static void I2C_ITError(I2C_HandleTypeDef *hi2c, uint32_t ErrorCode);

/* Private functions to handle IT transfer */
//static HAL_StatusTypeDef I2C_RequestMemoryWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart);
//static HAL_StatusTypeDef I2C_RequestMemoryRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart);

/* Private functions for I2C transfer IRQ handler */
static HAL_StatusTypeDef I2C_Master_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);
static HAL_StatusTypeDef I2C_Master_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);
static HAL_StatusTypeDef I2C_Master_DMAReload(struct __I2C_HandleTypeDef *hi2c);

#if HAL_I2CSLAVE_MODULE_ENABLED
    static HAL_StatusTypeDef I2C_Slave_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);
    static HAL_StatusTypeDef I2C_Slave_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);
#endif

/* Private functions to handle flags during polling transfer */
static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart);
#if HAL_I2CSLAVE_MODULE_ENABLED
    static HAL_StatusTypeDef I2C_WaitOnTXISFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
    static HAL_StatusTypeDef I2C_WaitOnRXNEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
    static HAL_StatusTypeDef I2C_WaitOnSTOPFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
#endif
static HAL_StatusTypeDef I2C_WaitOnFlagAndDetectError(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t ErrFlags, uint32_t Timeout, uint32_t Tickstart);

/* Private functions to centralize the enable/disable of Interrupts */
static void I2C_Enable_IRQ(I2C_HandleTypeDef *hi2c, uint16_t InterruptRequest);
static void I2C_Disable_IRQ(I2C_HandleTypeDef *hi2c, uint16_t InterruptRequest);

#if 0
    static HAL_StatusTypeDef I2C_IsAcknowledgeFailed(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
    /* Private function to flush TXDR register */
    static void I2C_Flush_TXDR(I2C_HandleTypeDef *hi2c);

    /* Private function to handle  start, restart or stop a transfer */
    static void I2C_TransferConfig(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request);

    /* Private function to Convert Specific options */
    static void I2C_ConvertOtherXferOptions(I2C_HandleTypeDef *hi2c);
#endif

/* Private functions for I2C master xfer both Memmode and normal mode*/
static HAL_StatusTypeDef I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
static HAL_StatusTypeDef I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
static HAL_StatusTypeDef I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
static HAL_StatusTypeDef I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
static HAL_StatusTypeDef I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
static HAL_StatusTypeDef I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup I2C_Exported_Functions I2C Exported Functions
  * @{
  */

/** @defgroup I2C_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          deinitialize the I2Cx peripheral:

      (+) User must Implement HAL_I2C_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

      (+) Call the function HAL_I2C_Init() to configure the selected device with
          the selected configuration:
        (++) Clock Timing
        (++) Own Address 1
        (++) Addressing mode (Master, Slave)
        (++) Dual Addressing mode
        (++) Own Address 2
        (++) Own Address 2 Mask
        (++) General call mode
        (++) Nostretch mode

      (+) Call the function HAL_I2C_DeInit() to restore the default configuration
          of the selected I2Cx peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the I2C according to the specified parameters
  *         in the I2C_InitTypeDef and initialize the associated handle.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c)
{
    /* Check the I2C handle allocation */
    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    HAL_ASSERT(IS_I2C_ALL_INSTANCE(hi2c->Instance));
    HAL_ASSERT(IS_I2C_OWN_ADDRESS1(hi2c->Init.OwnAddress1));
    HAL_ASSERT(IS_I2C_ADDRESSING_MODE(hi2c->Init.AddressingMode));
    HAL_ASSERT(IS_I2C_GENERAL_CALL(hi2c->Init.GeneralCallMode));


    if (hi2c->Instance == hwp_i2c1)
        HAL_RCC_ResetModule(RCC_MOD_I2C1);
    else if (hi2c->Instance == hwp_i2c2)
        HAL_RCC_ResetModule(RCC_MOD_I2C2);
    else if (hi2c->Instance == hwp_i2c3)
        HAL_RCC_ResetModule(RCC_MOD_I2C3);
    else if (hi2c->Instance == hwp_i2c4)
        HAL_RCC_ResetModule(RCC_MOD_I2C4);
#ifdef hwp_i2c5
    else if (hi2c->Instance == hwp_i2c5)
        HAL_RCC_ResetModule(RCC_MOD_I2C5);
#endif
#ifdef hwp_i2c6
    else if (hi2c->Instance == hwp_i2c6)
        HAL_RCC_ResetModule(RCC_MOD_I2C6);
#endif
    else
        ;



    /*Reset I2C block*/
    //I2C_SoftReset(hi2c);

    if (hi2c->State == HAL_I2C_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hi2c->Lock = HAL_UNLOCKED;

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
        /* Init the I2C Callback settings */
        hi2c->MasterTxCpltCallback = HAL_I2C_MasterTxCpltCallback; /* Legacy weak MasterTxCpltCallback */
        hi2c->MasterRxCpltCallback = HAL_I2C_MasterRxCpltCallback; /* Legacy weak MasterRxCpltCallback */
        hi2c->SlaveTxCpltCallback  = HAL_I2C_SlaveTxCpltCallback;  /* Legacy weak SlaveTxCpltCallback  */
        hi2c->SlaveRxCpltCallback  = HAL_I2C_SlaveRxCpltCallback;  /* Legacy weak SlaveRxCpltCallback  */
        hi2c->ListenCpltCallback   = HAL_I2C_ListenCpltCallback;   /* Legacy weak ListenCpltCallback   */
        hi2c->MemTxCpltCallback    = HAL_I2C_MemTxCpltCallback;    /* Legacy weak MemTxCpltCallback    */
        hi2c->MemRxCpltCallback    = HAL_I2C_MemRxCpltCallback;    /* Legacy weak MemRxCpltCallback    */
        hi2c->ErrorCallback        = HAL_I2C_ErrorCallback;        /* Legacy weak ErrorCallback        */
        hi2c->AbortCpltCallback    = HAL_I2C_AbortCpltCallback;    /* Legacy weak AbortCpltCallback    */
        hi2c->AddrCallback         = HAL_I2C_AddrCallback;         /* Legacy weak AddrCallback         */

        if (hi2c->MspInitCallback == NULL)
        {
            hi2c->MspInitCallback = HAL_I2C_MspInit; /* Legacy weak MspInit  */
        }

        /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
        hi2c->MspInitCallback(hi2c);
#else
        /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
        HAL_I2C_MspInit(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }

    hi2c->State = HAL_I2C_STATE_BUSY;

    /* Disable the selected I2C peripheral */
    __HAL_I2C_DISABLE(hi2c);

    //scl=i2c_clk/(5*(PRESC+1))
#ifdef SF32LB55X
    uint32_t i2c_clk = HAL_RCC_GetPCLKFreq(hi2c->core, 1);
#else
    /* always 48MHz */
    uint32_t i2c_clk = 48000000;
#endif /* SF32LB55X */
    uint32_t cr;
    uint32_t dnf = hi2c->dnf;
    uint32_t flv;

    cr = (dnf << I2C_CR_DNF_Pos)&I2C_CR_DNF_Msk;

    if (hi2c->Mode == HAL_I2C_MODE_MASTER)
        cr |= (I2C_MODE_FS << I2C_CR_MODE_Pos) | I2C_CR_SCLE; // | I2C_CR_IUE;
    else if (hi2c->Mode == HAL_I2C_MODE_SLAVE)
        cr |= (I2C_MODE_FS << I2C_CR_MODE_Pos) | I2C_CR_IUE;
    else
        HAL_ASSERT(0);

    HAL_ASSERT(hi2c->Init.ClockSpeed > 0);

    flv = ((i2c_clk + (hi2c->Init.ClockSpeed >> 1)) / hi2c->Init.ClockSpeed - dnf - 7 + 1/*round up*/) / 2;

    HAL_ASSERT(flv <= (I2C_LCR_FLV_Msk >> I2C_LCR_FLV_Pos));
    MODIFY_REG(hi2c->Instance->LCR, I2C_LCR_FLV_Msk, flv << I2C_LCR_FLV_Pos);
    /* Standard mode only use to send bus reset, use lowest frequence */
    MODIFY_REG(hi2c->Instance->LCR, I2C_LCR_SLV_Msk, I2C_LCR_SLV_Msk);
    hi2c->Instance->CR = cr;
    hi2c->Instance->IER = 0;
    hi2c->Instance->SAR = hi2c->Init.OwnAddress1;
    /* Enable the selected I2C peripheral */
    //__HAL_I2C_ENABLE(hi2c);

#ifndef SF32LB55X
    {
        uint32_t cnt = flv / 2;

        cnt = (cnt < 3) ? 0 : cnt - 3;
        MODIFY_REG(hi2c->Instance->WCR, I2C_WCR_CNT_Msk, cnt << I2C_WCR_CNT_Pos);
    }
#endif /* !SF32LB55X */

    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->PreviousState = I2C_STATE_NONE;

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_DMA_Init(I2C_HandleTypeDef *hi2c, struct dma_config *dma_rx, struct dma_config *dma_tx)
{

    /* Configure the DMA handler for Transmission process */
    hi2c->hdmarx->Instance                  = dma_rx->Instance;
    hi2c->hdmarx->Init.Request              = dma_rx->request;
    hi2c->hdmarx->Init.Direction            = DMA_PERIPH_TO_MEMORY;
    hi2c->hdmarx->Init.PeriphInc            = DMA_PINC_DISABLE;
    hi2c->hdmarx->Init.MemInc               = DMA_MINC_ENABLE;
    hi2c->hdmarx->Init.PeriphDataAlignment  = DMA_PDATAALIGN_BYTE;
    hi2c->hdmarx->Init.MemDataAlignment     = DMA_MDATAALIGN_BYTE;
    hi2c->hdmarx->Init.Mode                 = DMA_NORMAL;
    hi2c->hdmarx->Init.Priority             = DMA_PRIORITY_HIGH;
    {
        uint32_t tmpreg = 0x00U;
        //SET_BIT(RCC->AHB1ENR, spi_config[i].dma_rx->dma_rcc);
        /* Delay after an RCC peripheral clock enabling */
        //tmpreg = READ_BIT(RCC->AHB1ENR, spi_config[i].dma_rx->dma_rcc);
        UNUSED(tmpreg); /* To avoid compiler warnings */
    }

    /* Configure the DMA handler for Transmission process */
    hi2c->hdmatx->Instance                  = dma_tx->Instance;
    hi2c->hdmatx->Init.Request              = dma_tx->request;
    hi2c->hdmatx->Init.Direction            = DMA_MEMORY_TO_PERIPH;
    hi2c->hdmatx->Init.PeriphInc            = DMA_PINC_DISABLE;
    hi2c->hdmatx->Init.MemInc               = DMA_MINC_ENABLE;
    hi2c->hdmatx->Init.PeriphDataAlignment  = DMA_PDATAALIGN_BYTE;
    hi2c->hdmatx->Init.MemDataAlignment     = DMA_MDATAALIGN_BYTE;
    hi2c->hdmatx->Init.Mode                 = DMA_NORMAL;
    hi2c->hdmatx->Init.Priority             = DMA_PRIORITY_LOW;
    {
        uint32_t tmpreg = 0x00U;
        //SET_BIT(RCC->AHB1ENR, spi_config[i].dma_tx->dma_rcc);
        /* Delay after an RCC peripheral clock enabling */
        //tmpreg = READ_BIT(RCC->AHB1ENR, spi_config[i].dma_tx->dma_rcc);
        UNUSED(tmpreg); /* To avoid compiler warnings */
    }

    HAL_DMA_Init(hi2c->hdmarx);

    //HAL_DMA_Init(hi2c->dma_tx);

    return HAL_OK;
}


/**
  * @brief  DeInitialize the I2C peripheral.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c)
{
    /* Check the I2C handle allocation */
    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    HAL_ASSERT(IS_I2C_ALL_INSTANCE(hi2c->Instance));

    hi2c->State = HAL_I2C_STATE_BUSY;

    /* Disable the I2C Peripheral Clock */
    __HAL_I2C_DISABLE(hi2c);

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    if (hi2c->MspDeInitCallback == NULL)
    {
        hi2c->MspDeInitCallback = HAL_I2C_MspDeInit; /* Legacy weak MspDeInit  */
    }

    /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
    hi2c->MspDeInitCallback(hi2c);
#else
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
    HAL_I2C_MspDeInit(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    hi2c->State = HAL_I2C_STATE_RESET;
    hi2c->PreviousState = I2C_STATE_NONE;
    hi2c->Mode = HAL_I2C_MODE_NONE;

    /* Release Lock */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
}

/**
  * @brief Initialize the I2C MSP.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2c);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_I2C_MspInit could be implemented in the user file
     */
}

/**
  * @brief DeInitialize the I2C MSP.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2c);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_I2C_MspDeInit could be implemented in the user file
     */
}

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User I2C Callback
  *         To be used instead of the weak predefined callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_I2C_MASTER_TX_COMPLETE_CB_ID Master Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_MASTER_RX_COMPLETE_CB_ID Master Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_SLAVE_TX_COMPLETE_CB_ID Slave Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_SLAVE_RX_COMPLETE_CB_ID Slave Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_LISTEN_COMPLETE_CB_ID Listen Complete callback ID
  *          @arg @ref HAL_I2C_MEM_TX_COMPLETE_CB_ID Memory Tx Transfer callback ID
  *          @arg @ref HAL_I2C_MEM_RX_COMPLETE_CB_ID Memory Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_ERROR_CB_ID Error callback ID
  *          @arg @ref HAL_I2C_ABORT_CB_ID Abort callback ID
  *          @arg @ref HAL_I2C_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_I2C_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_RegisterCallback(I2C_HandleTypeDef *hi2c, HAL_I2C_CallbackIDTypeDef CallbackID, pI2C_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

        return HAL_ERROR;
    }
    /* Process locked */
    __HAL_LOCK(hi2c);

    if (HAL_I2C_STATE_READY == hi2c->State)
    {
        switch (CallbackID)
        {
        case HAL_I2C_MASTER_TX_COMPLETE_CB_ID :
            hi2c->MasterTxCpltCallback = pCallback;
            break;

        case HAL_I2C_MASTER_RX_COMPLETE_CB_ID :
            hi2c->MasterRxCpltCallback = pCallback;
            break;

        case HAL_I2C_SLAVE_TX_COMPLETE_CB_ID :
            hi2c->SlaveTxCpltCallback = pCallback;
            break;

        case HAL_I2C_SLAVE_RX_COMPLETE_CB_ID :
            hi2c->SlaveRxCpltCallback = pCallback;
            break;

        case HAL_I2C_LISTEN_COMPLETE_CB_ID :
            hi2c->ListenCpltCallback = pCallback;
            break;

        case HAL_I2C_MEM_TX_COMPLETE_CB_ID :
            hi2c->MemTxCpltCallback = pCallback;
            break;

        case HAL_I2C_MEM_RX_COMPLETE_CB_ID :
            hi2c->MemRxCpltCallback = pCallback;
            break;

        case HAL_I2C_ERROR_CB_ID :
            hi2c->ErrorCallback = pCallback;
            break;

        case HAL_I2C_ABORT_CB_ID :
            hi2c->AbortCpltCallback = pCallback;
            break;

        case HAL_I2C_MSPINIT_CB_ID :
            hi2c->MspInitCallback = pCallback;
            break;

        case HAL_I2C_MSPDEINIT_CB_ID :
            hi2c->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (HAL_I2C_STATE_RESET == hi2c->State)
    {
        switch (CallbackID)
        {
        case HAL_I2C_MSPINIT_CB_ID :
            hi2c->MspInitCallback = pCallback;
            break;

        case HAL_I2C_MSPDEINIT_CB_ID :
            hi2c->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hi2c);
    return status;
}

/**
  * @brief  Unregister an I2C Callback
  *         I2C callback is redirected to the weak predefined callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_I2C_MASTER_TX_COMPLETE_CB_ID Master Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_MASTER_RX_COMPLETE_CB_ID Master Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_SLAVE_TX_COMPLETE_CB_ID Slave Tx Transfer completed callback ID
  *          @arg @ref HAL_I2C_SLAVE_RX_COMPLETE_CB_ID Slave Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_LISTEN_COMPLETE_CB_ID Listen Complete callback ID
  *          @arg @ref HAL_I2C_MEM_TX_COMPLETE_CB_ID Memory Tx Transfer callback ID
  *          @arg @ref HAL_I2C_MEM_RX_COMPLETE_CB_ID Memory Rx Transfer completed callback ID
  *          @arg @ref HAL_I2C_ERROR_CB_ID Error callback ID
  *          @arg @ref HAL_I2C_ABORT_CB_ID Abort callback ID
  *          @arg @ref HAL_I2C_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_I2C_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_UnRegisterCallback(I2C_HandleTypeDef *hi2c, HAL_I2C_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hi2c);

    if (HAL_I2C_STATE_READY == hi2c->State)
    {
        switch (CallbackID)
        {
        case HAL_I2C_MASTER_TX_COMPLETE_CB_ID :
            hi2c->MasterTxCpltCallback = HAL_I2C_MasterTxCpltCallback; /* Legacy weak MasterTxCpltCallback */
            break;

        case HAL_I2C_MASTER_RX_COMPLETE_CB_ID :
            hi2c->MasterRxCpltCallback = HAL_I2C_MasterRxCpltCallback; /* Legacy weak MasterRxCpltCallback */
            break;

        case HAL_I2C_SLAVE_TX_COMPLETE_CB_ID :
            hi2c->SlaveTxCpltCallback = HAL_I2C_SlaveTxCpltCallback;   /* Legacy weak SlaveTxCpltCallback  */
            break;

        case HAL_I2C_SLAVE_RX_COMPLETE_CB_ID :
            hi2c->SlaveRxCpltCallback = HAL_I2C_SlaveRxCpltCallback;   /* Legacy weak SlaveRxCpltCallback  */
            break;

        case HAL_I2C_LISTEN_COMPLETE_CB_ID :
            hi2c->ListenCpltCallback = HAL_I2C_ListenCpltCallback;     /* Legacy weak ListenCpltCallback   */
            break;

        case HAL_I2C_MEM_TX_COMPLETE_CB_ID :
            hi2c->MemTxCpltCallback = HAL_I2C_MemTxCpltCallback;       /* Legacy weak MemTxCpltCallback    */
            break;

        case HAL_I2C_MEM_RX_COMPLETE_CB_ID :
            hi2c->MemRxCpltCallback = HAL_I2C_MemRxCpltCallback;       /* Legacy weak MemRxCpltCallback    */
            break;

        case HAL_I2C_ERROR_CB_ID :
            hi2c->ErrorCallback = HAL_I2C_ErrorCallback;               /* Legacy weak ErrorCallback        */
            break;

        case HAL_I2C_ABORT_CB_ID :
            hi2c->AbortCpltCallback = HAL_I2C_AbortCpltCallback;       /* Legacy weak AbortCpltCallback    */
            break;

        case HAL_I2C_MSPINIT_CB_ID :
            hi2c->MspInitCallback = HAL_I2C_MspInit;                   /* Legacy weak MspInit              */
            break;

        case HAL_I2C_MSPDEINIT_CB_ID :
            hi2c->MspDeInitCallback = HAL_I2C_MspDeInit;               /* Legacy weak MspDeInit            */
            break;

        default :
            /* Update the error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (HAL_I2C_STATE_RESET == hi2c->State)
    {
        switch (CallbackID)
        {
        case HAL_I2C_MSPINIT_CB_ID :
            hi2c->MspInitCallback = HAL_I2C_MspInit;                   /* Legacy weak MspInit              */
            break;

        case HAL_I2C_MSPDEINIT_CB_ID :
            hi2c->MspDeInitCallback = HAL_I2C_MspDeInit;               /* Legacy weak MspDeInit            */
            break;

        default :
            /* Update the error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hi2c);
    return status;
}

/**
  * @brief  Register the Slave Address Match I2C Callback
  *         To be used instead of the weak HAL_I2C_AddrCallback() predefined callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pCallback pointer to the Address Match Callback function
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_RegisterAddrCallback(I2C_HandleTypeDef *hi2c, pI2C_AddrCallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

        return HAL_ERROR;
    }
    /* Process locked */
    __HAL_LOCK(hi2c);

    if (HAL_I2C_STATE_READY == hi2c->State)
    {
        hi2c->AddrCallback = pCallback;
    }
    else
    {
        /* Update the error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hi2c);
    return status;
}

/**
  * @brief  UnRegister the Slave Address Match I2C Callback
  *         Info Ready I2C Callback is redirected to the weak HAL_I2C_AddrCallback() predefined callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_UnRegisterAddrCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hi2c);

    if (HAL_I2C_STATE_READY == hi2c->State)
    {
        hi2c->AddrCallback = HAL_I2C_AddrCallback; /* Legacy weak AddrCallback  */
    }
    else
    {
        /* Update the error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hi2c);
    return status;
}

#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group2 Input and Output operation functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the I2C data
    transfers.

    (#) There are two modes of transfer:
       (++) Blocking mode : The communication is performed in the polling mode.
            The status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode : The communication is performed using Interrupts
            or DMA. These functions return the status of the transfer startup.
            The end of the data processing will be indicated through the
            dedicated I2C IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.

    (#) Blocking mode functions are :
        (++) HAL_I2C_Master_Transmit()
        (++) HAL_I2C_Master_Receive()
        (++) HAL_I2C_Slave_Transmit()
        (++) HAL_I2C_Slave_Receive()
        (++) HAL_I2C_Mem_Write()
        (++) HAL_I2C_Mem_Read()
        (++) HAL_I2C_IsDeviceReady()

    (#) No-Blocking mode functions with Interrupt are :
        (++) HAL_I2C_Master_Transmit_IT()
        (++) HAL_I2C_Master_Receive_IT()
        (++) HAL_I2C_Slave_Transmit_IT()
        (++) HAL_I2C_Slave_Receive_IT()
        (++) HAL_I2C_Mem_Write_IT()
        (++) HAL_I2C_Mem_Read_IT()
        (++) HAL_I2C_Master_Sequential_Transmit_IT()
        (++) HAL_I2C_Master_Sequential_Receive_IT()
        (++) HAL_I2C_Slave_Sequential_Transmit_IT()
        (++) HAL_I2C_Slave_Sequential_Receive_IT()
        (++) HAL_I2C_EnableListen_IT()
        (++) HAL_I2C_DisableListen_IT()
        (++) HAL_I2C_Master_Abort_IT()

    (#) No-Blocking mode functions with DMA are :
        (++) HAL_I2C_Master_Transmit_DMA()
        (++) HAL_I2C_Master_Receive_DMA()
        (++) HAL_I2C_Slave_Transmit_DMA()
        (++) HAL_I2C_Slave_Receive_DMA()
        (++) HAL_I2C_Mem_Write_DMA()
        (++) HAL_I2C_Mem_Read_DMA()
        (++) HAL_I2C_Master_Sequential_Transmit_DMA()
        (++) HAL_I2C_Master_Sequential_Receive_DMA()
        (++) HAL_I2C_Slave_Sequential_Transmit_DMA()
        (++) HAL_I2C_Slave_Sequential_Receive_DMA()

    (#) A set of Transfer Complete Callbacks are provided in non Blocking mode:
        (++) HAL_I2C_MasterTxCpltCallback()
        (++) HAL_I2C_MasterRxCpltCallback()
        (++) HAL_I2C_SlaveTxCpltCallback()
        (++) HAL_I2C_SlaveRxCpltCallback()
        (++) HAL_I2C_MemTxCpltCallback()
        (++) HAL_I2C_MemRxCpltCallback()
        (++) HAL_I2C_AddrCallback()
        (++) HAL_I2C_ListenCpltCallback()
        (++) HAL_I2C_ErrorCallback()
        (++) HAL_I2C_AbortCpltCallback()

@endverbatim
  * @{
  */




#define I2C_10BIT_ADDRESS(__ADDRESS__)                     ((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)0x00FF)))
#define I2C_10BIT_HEADER_WRITE(__ADDRESS__)                ((uint8_t)((uint16_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)0x0300)) >> 7) | (uint16_t)0x00F0)))
#define I2C_10BIT_HEADER_READ(__ADDRESS__)                 ((uint8_t)((uint16_t)((uint16_t)(((uint16_t)((__ADDRESS__) & (uint16_t)0x0300)) >> 7) | (uint16_t)(0x00F1))))

#if 0
static HAL_StatusTypeDef I2C_MasterRequestWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Timeout, uint32_t Tickstart)
{
    HAL_StatusTypeDef err = HAL_OK;

    hi2c->Instance->IER = 0; //Disable all interrupt

    // Clear all pending statu.
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_SR_TE | I2C_SR_NACK);
    HAL_I2C_LOG("I2C_MasterRequestWrite0 CR=%x, SR=%x, mode=%x",
                hi2c->Instance->CR, hi2c->Instance->SR, hi2c->Init.AddressingMode);
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
    {
        /* Send slave address */
        hi2c->Instance->DBR = (DevAddress << 1);
        hi2c->Instance->TCR = I2C_TCR_START | I2C_TCR_TB;
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_SR_TE, RESET, Timeout, Tickstart) != HAL_OK)
        {
            err = HAL_TIMEOUT;
        }
    }
    else
    {
        /* 1.Send header of slave address */
        hi2c->Instance->DBR = I2C_10BIT_HEADER_WRITE(DevAddress);
        hi2c->Instance->TCR = I2C_TCR_START | I2C_TCR_TB;

        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_SR_TE, RESET, Timeout, Tickstart) != HAL_OK)
        {
            err = HAL_TIMEOUT;
        }
        else
        {
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_SR_TE | I2C_SR_NACK);

            /* 2.Send slave address */
            hi2c->Instance->DBR = I2C_10BIT_ADDRESS(DevAddress);
            hi2c->Instance->TCR = I2C_TCR_TB;

            if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_SR_TE, RESET, Timeout, Tickstart) != HAL_OK)
            {
                err = HAL_TIMEOUT;
            }
        }
    }

    HAL_I2C_LOG("I2C_MasterRequestWrite2 SR=%x", hi2c->Instance->SR);
    if (HAL_OK == err)
    {
        err = (__HAL_I2C_GET_FLAG(hi2c, I2C_SR_NACK) == RESET) ? HAL_OK : HAL_ERROR;
    }

    return err;
}

static HAL_StatusTypeDef I2C_MasterRequestRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Timeout, uint32_t Tickstart)
{
    HAL_StatusTypeDef err = HAL_OK;

    hi2c->Instance->IER = 0; //Disable all interrupt

    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_SR_TE | I2C_SR_NACK);
    HAL_I2C_LOG("I2C_MasterRequestRead0 CR=%x, SR=%x, mode=%x",
                hi2c->Instance->CR, hi2c->Instance->SR, hi2c->Init.AddressingMode);
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
    {
        /* Send slave address */
        hi2c->Instance->DBR = (DevAddress << 1) | 1;
        hi2c->Instance->TCR = (I2C_TCR_START | I2C_TCR_TB);

        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_SR_TE, RESET, Timeout, Tickstart) != HAL_OK)
        {
            err = HAL_TIMEOUT;
        }
    }
    else
    {
        /* 1.Send header of slave address , write*/
        hi2c->Instance->DBR = I2C_10BIT_HEADER_WRITE(DevAddress);
        hi2c->Instance->TCR = (I2C_TCR_START | I2C_TCR_TB);

        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_SR_TE, RESET, Timeout, Tickstart) != HAL_OK)
        {
            err = HAL_TIMEOUT;
        }
        else
        {
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_SR_TE);

            /* 2.Send slave address */
            hi2c->Instance->DBR = I2C_10BIT_ADDRESS(DevAddress);
            hi2c->Instance->TCR = I2C_TCR_TB;

            if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_SR_TE, RESET, Timeout, Tickstart) != HAL_OK)
            {
                err = HAL_TIMEOUT;
            }

            else
            {
                __HAL_I2C_CLEAR_FLAG(hi2c, I2C_SR_TE);

                /* 3.Send header of slave address , read*/
                hi2c->Instance->DBR = I2C_10BIT_HEADER_READ(DevAddress);
                hi2c->Instance->TCR = (I2C_TCR_START | I2C_TCR_TB);

                if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_SR_TE, RESET, Timeout, Tickstart) != HAL_OK)
                {
                    err = HAL_TIMEOUT;
                }
            }
        }
    }


    HAL_I2C_LOG("I2C_MasterRequestRead2 SR=%x", hi2c->Instance->SR);

    if (HAL_OK == err)
    {
        err = (__HAL_I2C_GET_FLAG(hi2c, I2C_SR_NACK) == RESET) ? HAL_OK : HAL_ERROR;
    }

    return err;
}
#else

/**
  * @brief  Master sends target device address followed by internal memory address for write request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */

static HAL_StatusTypeDef I2C_MasterRequestWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart)
{
    uint8_t sequence[4][2];
    uint8_t seq_len = 0;
    HAL_StatusTypeDef err = HAL_OK;

    HAL_I2C_LOG("I2C_MasterRequestWrite0 CR=%x, SR=%x, addrmode=%x, memmode=%d",
                hi2c->Instance->CR, hi2c->Instance->SR, hi2c->Init.AddressingMode, MemAddSize);

    /*Prepare send sequence*/
    if (IS_MEM_MODE(MemAddSize))
    {
        /*Start write mem addr*/
        sequence[seq_len][0] = (DevAddress << 1);
        sequence[seq_len][1] = (I2C_TCR_START | I2C_TCR_TB);
        seq_len++;

        /* If Memory address size is 8Bit */
        if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
        {
            /* Send Memory Address */
            sequence[seq_len][0] = I2C_MEM_ADD_LSB(MemAddress);
            sequence[seq_len][1] = I2C_TCR_TB;
            seq_len++;
        }
        /* If Memory address size is 16Bit */
        else
        {
            /* Send MSB of Memory Address */
            sequence[seq_len][0] = I2C_MEM_ADD_MSB(MemAddress);
            sequence[seq_len][1] = I2C_TCR_TB;
            seq_len++;

            /* Send LSB of Memory Address */
            sequence[seq_len][0] = I2C_MEM_ADD_LSB(MemAddress);
            sequence[seq_len][1] = I2C_TCR_TB;
            seq_len++;
        }

    }
    else
    {
        if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
        {
            sequence[seq_len][0] = (DevAddress << 1);
            sequence[seq_len][1] = (I2C_TCR_START | I2C_TCR_TB);
            seq_len++;
        }
        else
        {
            sequence[seq_len][0] = I2C_10BIT_HEADER_WRITE(DevAddress);
            sequence[seq_len][1] = (I2C_TCR_START | I2C_TCR_TB);
            seq_len++;

            sequence[seq_len][0] = I2C_10BIT_ADDRESS(DevAddress);
            sequence[seq_len][1] = I2C_TCR_TB;
            seq_len++;
        }
    }


    hi2c->Instance->IER = 0; //Disable all interrupt
    hi2c->Instance->CR &= ~(I2C_CR_DMAEN | I2C_CR_LASTSTOP | I2C_CR_LASTNACK); //Disable DMA cfg


    for (uint8_t i = 0; i < seq_len; i++)
    {
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_SR_TE | I2C_SR_BED);
        hi2c->Instance->DBR = sequence[i][0];
        hi2c->Instance->TCR = sequence[i][1];
        if (I2C_WaitOnFlagAndDetectError(hi2c, I2C_SR_TE, RESET, I2C_FLAG_TX_ERR, Timeout, Tickstart) != HAL_OK)
        {
            HAL_I2C_LOG("idx:%d: data:%02x, %02x", i, sequence[i][0], sequence[i][1]);
            HAL_I2C_LOG("CR=%x, SR=%x", hi2c->Instance->CR, hi2c->Instance->SR);
            hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
            err = HAL_TIMEOUT;
            break;
        }
    }
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_SR_TE | I2C_SR_BED);


    HAL_I2C_LOG("I2C_MasterRequestWrite1 CR=%x, SR=%x, err=%d",
                hi2c->Instance->CR, hi2c->Instance->SR, err);

    return err;
}

/**
  * @brief  Master sends target device address followed by internal memory address for read request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */

static HAL_StatusTypeDef I2C_MasterRequestRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart)
{
    uint8_t sequence[4][2];
    uint8_t seq_len = 0;
    HAL_StatusTypeDef err = HAL_OK;

    HAL_I2C_LOG("I2C_MasterRequestRead0 CR=%x, SR=%x, addrmode=%x, memmode=%d",
                hi2c->Instance->CR, hi2c->Instance->SR, hi2c->Init.AddressingMode, MemAddSize);

    /*Prepare send sequence*/
    if (IS_MEM_MODE(MemAddSize))
    {
        /*Start write mem addr*/
        sequence[seq_len][0] = (DevAddress << 1);
        sequence[seq_len][1] = (I2C_TCR_START | I2C_TCR_TB);
        seq_len++;

        /* If Memory address size is 8Bit */
        if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
        {
            /* Send Memory Address */
            sequence[seq_len][0] = I2C_MEM_ADD_LSB(MemAddress);
            sequence[seq_len][1] = I2C_TCR_TB;
            seq_len++;
        }
        /* If Memory address size is 16Bit */
        else
        {
            /* Send MSB of Memory Address */
            sequence[seq_len][0] = I2C_MEM_ADD_MSB(MemAddress);
            sequence[seq_len][1] = I2C_TCR_TB;
            seq_len++;

            /* Send LSB of Memory Address */
            sequence[seq_len][0] = I2C_MEM_ADD_LSB(MemAddress);
            sequence[seq_len][1] = I2C_TCR_TB;
            seq_len++;
        }

        /*Restart + send slave addr + read*/
        sequence[seq_len][0] = (DevAddress << 1) | 1;
        sequence[seq_len][1] = (I2C_TCR_START | I2C_TCR_TB);
        seq_len++;

    }
    else
    {
        if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
        {
            sequence[seq_len][0] = (DevAddress << 1) | 1;
            sequence[seq_len][1] = (I2C_TCR_START | I2C_TCR_TB);
            seq_len++;
        }
        else
        {
            sequence[seq_len][0] = I2C_10BIT_HEADER_WRITE(DevAddress);
            sequence[seq_len][1] = (I2C_TCR_START | I2C_TCR_TB);
            seq_len++;

            sequence[seq_len][0] = I2C_10BIT_ADDRESS(DevAddress);
            sequence[seq_len][1] = I2C_TCR_TB;
            seq_len++;

            sequence[seq_len][0] = I2C_10BIT_HEADER_READ(DevAddress);
            sequence[seq_len][1] = (I2C_TCR_START | I2C_TCR_TB);
            seq_len++;
        }
    }


    hi2c->Instance->IER = 0; //Disable all interrupt
    hi2c->Instance->CR &= ~(I2C_CR_DMAEN | I2C_CR_LASTSTOP | I2C_CR_LASTNACK); //Disable DMA cfg


    for (uint8_t i = 0; i < seq_len; i++)
    {
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_SR_TE | I2C_SR_BED);
        hi2c->Instance->DBR = sequence[i][0];
        hi2c->Instance->TCR = sequence[i][1];
        if (I2C_WaitOnFlagAndDetectError(hi2c, I2C_SR_TE, RESET, I2C_FLAG_TX_ERR, Timeout, Tickstart) != HAL_OK)
        {
            HAL_I2C_LOG("idx:%d: data:%02x, %02x", i, sequence[i][0], sequence[i][1]);
            HAL_I2C_LOG("CR=%x, SR=%x", hi2c->Instance->CR, hi2c->Instance->SR);
            hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
            err = HAL_TIMEOUT;
            break;
        }
    }
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_SR_TE | I2C_SR_BED);


    HAL_I2C_LOG("I2C_MasterRequestRead1 CR=%x, SR=%x, err=%d",
                hi2c->Instance->CR, hi2c->Instance->SR, err);

    return err;
}


#endif

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart;

    /* Check the parameters */
    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(hi2c);

        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();

        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_SR_UB, SET, I2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
        {
            __HAL_UNLOCK(hi2c);
            return HAL_BUSY;
        }

        /* Send Slave Address (and Memory Address if present) */
        if (I2C_MasterRequestWrite(hi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != HAL_OK)
        {
            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);
            return HAL_ERROR;
        }

        hi2c->State     = HAL_I2C_STATE_BUSY_TX;
        hi2c->Mode      = IS_MEM_MODE(MemAddSize) ? HAL_I2C_MODE_MASTER : HAL_I2C_MODE_MEM;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr  = pData;
        hi2c->XferCount = Size;
        hi2c->XferISR   = NULL;
        hi2c->XferSize = hi2c->XferCount;
        while (hi2c->XferCount > 0U)
        {
            uint32_t errmsk;

            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_SR_TE);

            /* Write data to DBR */
            hi2c->Instance->DBR = *hi2c->pBuffPtr;
            hi2c->Instance->TCR = (hi2c->XferCount == 1) ? (I2C_TCR_TB | I2C_TCR_STOP) : I2C_TCR_TB;
            errmsk = (1 == hi2c->XferCount) ? I2C_FLAG_TXE_ERR : I2C_FLAG_TX_ERR;

            if (I2C_WaitOnFlagAndDetectError(hi2c, I2C_SR_TE, RESET, errmsk, Timeout, tickstart) != HAL_OK)
            {
                goto ERROR_DEL;
            }

            /* Increment Buffer pointer */
            hi2c->pBuffPtr++;

            hi2c->XferCount--;
            hi2c->XferSize--;
        }
        /* Wait until STOP is sent */
        if (I2C_WaitOnFlagAndDetectError(hi2c, I2C_SR_UB, SET, 0, Timeout, tickstart) != HAL_OK)
        {
            hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
            goto ERROR_DEL;
        }

        hi2c->State = HAL_I2C_STATE_READY;
        hi2c->Mode  = HAL_I2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }

ERROR_DEL:
    HAL_I2C_LOG("%s ERROR_DEL SR=%x", __FUNCTION__, hi2c->Instance->SR);
    __HAL_I2C_CLEAR_FLAG(hi2c, hi2c->Instance->SR);
    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    __HAL_UNLOCK(hi2c);

    return HAL_ERROR;
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart;

    /* Check the parameters */
    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }

        __HAL_LOCK(hi2c);           /* Process Locked */

        tickstart = HAL_GetTick();  /* Init tickstart for timeout management*/
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY, tickstart) != HAL_OK)
        {
            __HAL_UNLOCK(hi2c);
            return HAL_BUSY;
        }

        /* Send Slave Address (and Memory Address if present) */
        if (HAL_OK != I2C_MasterRequestRead(hi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart))
        {
            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);
            return HAL_ERROR;
        }

        hi2c->State     = HAL_I2C_STATE_BUSY_RX;
        hi2c->Mode      = IS_MEM_MODE(MemAddSize) ? HAL_I2C_MODE_MASTER : HAL_I2C_MODE_MEM;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr  = pData;
        hi2c->XferCount = Size;
        hi2c->XferISR   = NULL;
        hi2c->XferSize = hi2c->XferCount;
        while (hi2c->XferCount > 0U)
        {
            uint32_t errmsk;
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_SR_RF);
            hi2c->Instance->TCR = (1 == hi2c->XferCount) ? (I2C_TCR_TB | I2C_TCR_STOP | I2C_TCR_NACK) : I2C_TCR_TB;
            errmsk = (1 == hi2c->XferCount) ? I2C_FLAG_RXE_ERR : I2C_FLAG_RX_ERR;
            if (I2C_WaitOnFlagAndDetectError(hi2c, I2C_SR_RF, RESET, errmsk, Timeout, tickstart) != HAL_OK)
            {
                goto ERROR_DEL;
            }
            *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DBR;     /* Read data from RXDR */
            hi2c->pBuffPtr++;                                   /* Increment Buffer pointer */
            hi2c->XferSize--;
            hi2c->XferCount--;
        }

        /* Wait until STOP is sent */
        if (I2C_WaitOnFlagAndDetectError(hi2c, I2C_SR_UB, SET, 0, Timeout, tickstart) != HAL_OK)
        {
            hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
            goto ERROR_DEL;
        }

        hi2c->State = HAL_I2C_STATE_READY;
        hi2c->Mode  = HAL_I2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_OK;
    }
    else
        return HAL_BUSY;

ERROR_DEL:
    HAL_I2C_LOG("%s ERROR_DEL SR=%x", __FUNCTION__, hi2c->Instance->SR);
    __HAL_I2C_CLEAR_FLAG(hi2c, hi2c->Instance->SR);
    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    __HAL_UNLOCK(hi2c);

    return HAL_ERROR;
}


/**
  * @brief  Write an amount of data in non-blocking mode with Interrupt to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
    uint32_t tickstart;
    //uint32_t xfermode;

    /* Check the parameters */
    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }

        if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(hi2c);

        HAL_I2C_LOG("Master_Transmit_IT %d bytes.", Size);

        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();

        /* Send Slave Address and Memory Address */
        if (I2C_MasterRequestWrite(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_ADDR, tickstart) != HAL_OK)
        {
            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);
            return HAL_ERROR;
        }

        hi2c->Mode        = IS_MEM_MODE(MemAddSize) ? HAL_I2C_MODE_MASTER : HAL_I2C_MODE_MEM;
        hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferOptions = I2C_NO_OPTION_FRAME;
        hi2c->XferISR     = I2C_Master_ISR_IT;

        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
            hi2c->XferSize = MAX_NBYTE_SIZE;
            //xfermode = I2C_RELOAD_MODE;
        }
        else
        {
            hi2c->XferSize = hi2c->XferCount;
            //xfermode = I2C_AUTOEND_MODE;
        }

        I2C_Disable_IRQ(hi2c, I2C_IER_TEIE);


        hi2c->State       = HAL_I2C_STATE_BUSY_TX;
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_SR_TE);

        hi2c->Instance->CR |= I2C_CR_MSDE;
        /* Write data to DBR */
        hi2c->Instance->DBR = *hi2c->pBuffPtr;
        if (1 == hi2c->XferSize)
            hi2c->Instance->TCR = I2C_TCR_TB | I2C_TCR_STOP;
        else
            hi2c->Instance->TCR = I2C_TCR_TB;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        hi2c->XferCount--;
        hi2c->XferSize--;


        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */

        I2C_Enable_IRQ(hi2c, I2C_IER_TEIE | I2C_IER_BEDIE | I2C_IER_MSDIE);
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Read an amount of data in non-blocking mode with Interrupt from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
    uint32_t tickstart;
    //uint32_t xfermode;

    /* Check the parameters */
    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }

        if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(hi2c);

        HAL_I2C_LOG("Master_Receive_IT %d bytes.", Size);

        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();

        /* Send Slave Address and Memory Address */
        if (I2C_MasterRequestRead(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_ADDR, tickstart) != HAL_OK)
        {
            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);
            return HAL_ERROR;
        }

        hi2c->Mode        = IS_MEM_MODE(MemAddSize) ? HAL_I2C_MODE_MASTER : HAL_I2C_MODE_MEM;
        hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferOptions = I2C_NO_OPTION_FRAME;
        hi2c->XferISR     = I2C_Master_ISR_IT;

        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
            hi2c->XferSize = MAX_NBYTE_SIZE;
            //xfermode = I2C_RELOAD_MODE;
        }
        else
        {
            hi2c->XferSize = hi2c->XferCount;
            //xfermode = I2C_AUTOEND_MODE;
        }

        I2C_Disable_IRQ(hi2c, I2C_IER_RFIE | I2C_IER_TEIE);

        /*Start receive data*/
        hi2c->State       = HAL_I2C_STATE_BUSY_RX;
        hi2c->Instance->CR |= I2C_CR_MSDE;
        hi2c->Instance->TCR = (1 == hi2c->XferCount) ? (I2C_TCR_TB | I2C_TCR_STOP | I2C_TCR_NACK) : I2C_TCR_TB;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */

        /* Enable ERR, TC, STOP, NACK, RXI interrupt */
        /* possible to enable all of these */
        /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
        //I2C_Enable_IRQ(hi2c, I2C_IER_TEIE);
        I2C_Enable_IRQ(hi2c, I2C_IER_RFIE | I2C_IER_TEIE | I2C_IER_BEDIE | I2C_IER_MSDIE);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}
/**
  * @brief  Write an amount of data in non-blocking mode with DMA to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
    uint32_t tickstart;
    //uint32_t xfermode;
    HAL_StatusTypeDef dmaxferstatus;

    /* Check the parameters */
    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }

        if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(hi2c);

        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();

        hi2c->State       = HAL_I2C_STATE_BUSY_TX;
        hi2c->Mode        = IS_MEM_MODE(MemAddSize) ? HAL_I2C_MODE_MASTER : HAL_I2C_MODE_MEM;
        hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferOptions = I2C_NO_OPTION_FRAME;
        hi2c->XferISR     = I2C_Master_ISR_DMA;

        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
            hi2c->XferSize = MAX_NBYTE_SIZE;
            //xfermode = I2C_RELOAD_MODE;
        }
        else
        {
            hi2c->XferSize = hi2c->XferCount;
            //xfermode = I2C_AUTOEND_MODE;
        }


        if (hi2c->hdmatx != NULL)
        {
            /* Set the I2C DMA transfer complete callback */
            hi2c->hdmatx->XferCpltCallback = I2C_DMAMasterTransmitCplt;

            /* Set the DMA error callback */
            hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

            /* Set the unused DMA callbacks to NULL */
            hi2c->hdmatx->XferHalfCpltCallback = NULL;
            hi2c->hdmatx->XferAbortCallback = NULL;

            /* Enable the DMA channel,

            Set DMA xfer len as XferCount(The total length), as we will restart I2C controller's DMA request
            while this DMADONE IRQ comming,

            and never restart DMA during whole I2C transfer*/
            dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)pData, (uint32_t)&hi2c->Instance->FIFO, hi2c->XferCount);
        }
        else
        {
            /* Update I2C state */
            hi2c->State     = HAL_I2C_STATE_READY;
            hi2c->Mode      = HAL_I2C_MODE_NONE;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }

        if (dmaxferstatus == HAL_OK)
        {
            /* Send Slave Address and Memory Address */
            if (I2C_MasterRequestWrite(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_ADDR, tickstart) != HAL_OK)
            {
                /* Update I2C state */
                hi2c->State     = HAL_I2C_STATE_READY;
                hi2c->Mode      = HAL_I2C_MODE_NONE;

                if (HAL_DMA_Abort_IT(hi2c->hdmatx) != HAL_OK)
                {
                    /* Call Directly hi2c->hdmatx->XferAbortCallback function in case of error */
                    hi2c->hdmatx->XferAbortCallback(hi2c->hdmatx);
                }

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);
                return HAL_ERROR;
            }

            /* Update XferCount value */
            hi2c->XferCount -= hi2c->XferSize;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);


            I2C_Master_DMAReload(hi2c);
        }
        else
        {
            /* Update I2C state */
            hi2c->State     = HAL_I2C_STATE_READY;
            hi2c->Mode      = HAL_I2C_MODE_NONE;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Reads an amount of data in non-blocking mode with DMA from a specific memory address.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be read
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
    uint32_t tickstart;
    //uint32_t xfermode;
    HAL_StatusTypeDef dmaxferstatus;

    /* Check the parameters */
    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }

        if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(hi2c);

        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();

        hi2c->State       = HAL_I2C_STATE_BUSY_RX;
        hi2c->Mode        = IS_MEM_MODE(MemAddSize) ? HAL_I2C_MODE_MASTER : HAL_I2C_MODE_MEM;
        hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferOptions = I2C_NO_OPTION_FRAME;
        hi2c->XferISR     = I2C_Master_ISR_DMA;

        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
            hi2c->XferSize = MAX_NBYTE_SIZE;
            //xfermode = I2C_RELOAD_MODE;
        }
        else
        {
            hi2c->XferSize = hi2c->XferCount;
            //xfermode = I2C_AUTOEND_MODE;
        }


        if (hi2c->hdmarx != NULL)
        {
            /* Set the I2C DMA transfer complete callback */
            hi2c->hdmarx->XferCpltCallback = I2C_DMAMasterReceiveCplt;

            /* Set the DMA error callback */
            hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

            /* Set the unused DMA callbacks to NULL */
            hi2c->hdmarx->XferHalfCpltCallback = NULL;
            hi2c->hdmarx->XferAbortCallback = NULL;

            /* Enable the DMA channel,

            Set DMA xfer len as XferCount(The total length), as we will restart I2C controller's DMA request
            while this DMADONE IRQ comming,

            and never restart DMA during whole I2C transfer*/
            dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->FIFO, (uint32_t)pData, hi2c->XferCount);
        }
        else
        {
            /* Update I2C state */
            hi2c->State     = HAL_I2C_STATE_READY;
            hi2c->Mode      = HAL_I2C_MODE_NONE;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }

        if (dmaxferstatus == HAL_OK)
        {
            /* Send Slave Address and Memory Address */
            if (I2C_MasterRequestRead(hi2c, DevAddress, MemAddress, MemAddSize, I2C_TIMEOUT_ADDR, tickstart) != HAL_OK)
            {
                /* Update I2C state */
                hi2c->State     = HAL_I2C_STATE_READY;
                hi2c->Mode      = HAL_I2C_MODE_NONE;

                if (HAL_DMA_Abort_IT(hi2c->hdmarx) != HAL_OK)
                {
                    /* Call Directly hi2c->hdmarx->XferAbortCallback function in case of error */
                    hi2c->hdmarx->XferAbortCallback(hi2c->hdmarx);
                }

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);
                return HAL_ERROR;
            }

            /* Update XferCount value */
            hi2c->XferCount -= hi2c->XferSize;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            I2C_Master_DMAReload(hi2c);
        }
        else
        {
            /* Update I2C state */
            hi2c->State     = HAL_I2C_STATE_READY;
            hi2c->Mode      = HAL_I2C_MODE_NONE;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}


/**
  * @brief  Transmits in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return I2C_Master_Transmit(hi2c, DevAddress, 0, I2C_MEMADD_SIZE_0BIT, pData, Size, Timeout);
}

/**
  * @brief  Receives in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    return I2C_Master_Receive(hi2c, DevAddress, 0, I2C_MEMADD_SIZE_0BIT, pData, Size, Timeout);
}


#if HAL_I2CSLAVE_MODULE_ENABLED
/**
  * @brief  Transmits in slave mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart;

    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }
        /* Process Locked */
        __HAL_LOCK(hi2c);

        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();

        hi2c->State     = HAL_I2C_STATE_BUSY_TX;
        hi2c->Mode      = HAL_I2C_MODE_SLAVE;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr  = pData;
        hi2c->XferCount = Size;
        hi2c->XferISR   = NULL;

        /* Enable Address Acknowledge */
        hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

        /* Wait until ADDR flag is set */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, RESET, Timeout, tickstart) != HAL_OK)
        {
            /* Disable Address Acknowledge */
            hi2c->Instance->CR2 |= I2C_CR2_NACK;
            return HAL_ERROR;
        }

        /* Clear ADDR flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

        /* If 10bit addressing mode is selected */
        if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_10BIT)
        {
            /* Wait until ADDR flag is set */
            if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, RESET, Timeout, tickstart) != HAL_OK)
            {
                /* Disable Address Acknowledge */
                hi2c->Instance->CR2 |= I2C_CR2_NACK;
                return HAL_ERROR;
            }

            /* Clear ADDR flag */
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
        }

        /* Wait until DIR flag is set Transmitter mode */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_DIR, RESET, Timeout, tickstart) != HAL_OK)
        {
            /* Disable Address Acknowledge */
            hi2c->Instance->CR2 |= I2C_CR2_NACK;
            return HAL_ERROR;
        }

        while (hi2c->XferCount > 0U)
        {
            /* Wait until TXIS flag is set */
            if (I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
            {
                /* Disable Address Acknowledge */
                hi2c->Instance->CR2 |= I2C_CR2_NACK;
                return HAL_ERROR;
            }

            /* Write data to TXDR */
            hi2c->Instance->TXDR = *hi2c->pBuffPtr;

            /* Increment Buffer pointer */
            hi2c->pBuffPtr++;

            hi2c->XferCount--;
        }

        /* Wait until STOP flag is set */
        if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
        {
            /* Disable Address Acknowledge */
            hi2c->Instance->CR2 |= I2C_CR2_NACK;

            if (hi2c->ErrorCode == HAL_I2C_ERROR_AF)
            {
                /* Normal use case for Transmitter mode */
                /* A NACK is generated to confirm the end of transfer */
                hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
            }
            else
            {
                return HAL_ERROR;
            }
        }

        /* Clear STOP flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

        /* Wait until BUSY flag is reset */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, Timeout, tickstart) != HAL_OK)
        {
            /* Disable Address Acknowledge */
            hi2c->Instance->CR2 |= I2C_CR2_NACK;
            return HAL_ERROR;
        }

        /* Disable Address Acknowledge */
        hi2c->Instance->CR2 |= I2C_CR2_NACK;

        hi2c->State = HAL_I2C_STATE_READY;
        hi2c->Mode  = HAL_I2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Receive in slave mode an amount of data in blocking mode
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart;

    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }
        /* Process Locked */
        __HAL_LOCK(hi2c);

        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();

        hi2c->State     = HAL_I2C_STATE_BUSY_RX;
        hi2c->Mode      = HAL_I2C_MODE_SLAVE;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr  = pData;
        hi2c->XferCount = Size;
        hi2c->XferISR   = NULL;

        /* Enable Address Acknowledge */
        hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

        /* Wait until ADDR flag is set */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, RESET, Timeout, tickstart) != HAL_OK)
        {
            /* Disable Address Acknowledge */
            hi2c->Instance->CR2 |= I2C_CR2_NACK;
            return HAL_ERROR;
        }

        /* Clear ADDR flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

        /* Wait until DIR flag is reset Receiver mode */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_DIR, SET, Timeout, tickstart) != HAL_OK)
        {
            /* Disable Address Acknowledge */
            hi2c->Instance->CR2 |= I2C_CR2_NACK;
            return HAL_ERROR;
        }

        while (hi2c->XferCount > 0U)
        {
            /* Wait until RXNE flag is set */
            if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
            {
                /* Disable Address Acknowledge */
                hi2c->Instance->CR2 |= I2C_CR2_NACK;

                /* Store Last receive data if any */
                if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) == SET)
                {
                    /* Read data from RXDR */
                    *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

                    /* Increment Buffer pointer */
                    hi2c->pBuffPtr++;

                    hi2c->XferCount--;
                }

                return HAL_ERROR;
            }

            /* Read data from RXDR */
            *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

            /* Increment Buffer pointer */
            hi2c->pBuffPtr++;

            hi2c->XferCount--;
        }

        /* Wait until STOP flag is set */
        if (I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
        {
            /* Disable Address Acknowledge */
            hi2c->Instance->CR2 |= I2C_CR2_NACK;
            return HAL_ERROR;
        }

        /* Clear STOP flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

        /* Wait until BUSY flag is reset */
        if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, Timeout, tickstart) != HAL_OK)
        {
            /* Disable Address Acknowledge */
            hi2c->Instance->CR2 |= I2C_CR2_NACK;
            return HAL_ERROR;
        }

        /* Disable Address Acknowledge */
        hi2c->Instance->CR2 |= I2C_CR2_NACK;

        hi2c->State = HAL_I2C_STATE_READY;
        hi2c->Mode  = HAL_I2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}
#endif
/**
  * @brief  Transmit in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    return I2C_Master_Transmit_IT(hi2c, DevAddress, 0, I2C_MEMADD_SIZE_0BIT, pData, Size);
}

/**
  * @brief  Receive in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    return I2C_Master_Receive_IT(hi2c, DevAddress, 0, I2C_MEMADD_SIZE_0BIT, pData, Size);
}


#if HAL_I2CSLAVE_MODULE_ENABLED
/**
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        /* Process Locked */
        __HAL_LOCK(hi2c);

        hi2c->State       = HAL_I2C_STATE_BUSY_TX;
        hi2c->Mode        = HAL_I2C_MODE_SLAVE;
        hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

        /* Enable Address Acknowledge */
        hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferSize    = hi2c->XferCount;
        hi2c->XferOptions = I2C_NO_OPTION_FRAME;
        hi2c->XferISR     = I2C_Slave_ISR_IT;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */

        /* Enable ERR, TC, STOP, NACK, TXI interrupt */
        /* possible to enable all of these */
        /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
        I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT | I2C_XFER_LISTEN_IT);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Receive in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        /* Process Locked */
        __HAL_LOCK(hi2c);

        hi2c->State       = HAL_I2C_STATE_BUSY_RX;
        hi2c->Mode        = HAL_I2C_MODE_SLAVE;
        hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

        /* Enable Address Acknowledge */
        hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferSize    = hi2c->XferCount;
        hi2c->XferOptions = I2C_NO_OPTION_FRAME;
        hi2c->XferISR     = I2C_Slave_ISR_IT;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */

        /* Enable ERR, TC, STOP, NACK, RXI interrupt */
        /* possible to enable all of these */
        /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
        I2C_Enable_IRQ(hi2c, I2C_XFER_RX_IT | I2C_XFER_LISTEN_IT);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}
#endif
/**
  * @brief  Transmit in master mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    return I2C_Master_Transmit_DMA(hi2c, DevAddress, 0, I2C_MEMADD_SIZE_0BIT, pData, Size);
}

/**
  * @brief  Receive in master mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    return I2C_Master_Receive_DMA(hi2c, DevAddress, 0, I2C_MEMADD_SIZE_0BIT, pData, Size);
}


#if HAL_I2CSLAVE_MODULE_ENABLED
/**
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef dmaxferstatus;

    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }
        /* Process Locked */
        __HAL_LOCK(hi2c);

        hi2c->State       = HAL_I2C_STATE_BUSY_TX;
        hi2c->Mode        = HAL_I2C_MODE_SLAVE;
        hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferSize    = hi2c->XferCount;
        hi2c->XferOptions = I2C_NO_OPTION_FRAME;
        hi2c->XferISR     = I2C_Slave_ISR_DMA;

        if (hi2c->hdmatx != NULL)
        {
            /* Set the I2C DMA transfer complete callback */
            hi2c->hdmatx->XferCpltCallback = I2C_DMASlaveTransmitCplt;

            /* Set the DMA error callback */
            hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

            /* Set the unused DMA callbacks to NULL */
            hi2c->hdmatx->XferHalfCpltCallback = NULL;
            hi2c->hdmatx->XferAbortCallback = NULL;

            /* Enable the DMA channel */
            dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)pData, (uint32_t)&hi2c->Instance->TXDR, hi2c->XferSize);
        }
        else
        {
            /* Update I2C state */
            hi2c->State     = HAL_I2C_STATE_LISTEN;
            hi2c->Mode      = HAL_I2C_MODE_NONE;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }

        if (dmaxferstatus == HAL_OK)
        {
            /* Enable Address Acknowledge */
            hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            /* Note : The I2C interrupts must be enabled after unlocking current process
                      to avoid the risk of I2C interrupt handle execution before current
                      process unlock */
            /* Enable ERR, STOP, NACK, ADDR interrupts */
            I2C_Enable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

            /* Enable DMA Request */
            hi2c->Instance->CR1 |= I2C_CR1_TXDMAEN;
        }
        else
        {
            /* Update I2C state */
            hi2c->State     = HAL_I2C_STATE_LISTEN;
            hi2c->Mode      = HAL_I2C_MODE_NONE;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Receive in slave mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef dmaxferstatus;

    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }
        /* Process Locked */
        __HAL_LOCK(hi2c);

        hi2c->State       = HAL_I2C_STATE_BUSY_RX;
        hi2c->Mode        = HAL_I2C_MODE_SLAVE;
        hi2c->ErrorCode   = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferSize    = hi2c->XferCount;
        hi2c->XferOptions = I2C_NO_OPTION_FRAME;
        hi2c->XferISR     = I2C_Slave_ISR_DMA;

        if (hi2c->hdmarx != NULL)
        {
            /* Set the I2C DMA transfer complete callback */
            hi2c->hdmarx->XferCpltCallback = I2C_DMASlaveReceiveCplt;

            /* Set the DMA error callback */
            hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

            /* Set the unused DMA callbacks to NULL */
            hi2c->hdmarx->XferHalfCpltCallback = NULL;
            hi2c->hdmarx->XferAbortCallback = NULL;

            /* Enable the DMA channel */
            dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->RXDR, (uint32_t)pData, hi2c->XferSize);
        }
        else
        {
            /* Update I2C state */
            hi2c->State     = HAL_I2C_STATE_LISTEN;
            hi2c->Mode      = HAL_I2C_MODE_NONE;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }

        if (dmaxferstatus == HAL_OK)
        {
            /* Enable Address Acknowledge */
            hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            /* Note : The I2C interrupts must be enabled after unlocking current process
                      to avoid the risk of I2C interrupt handle execution before current
                      process unlock */
            /* Enable ERR, STOP, NACK, ADDR interrupts */
            I2C_Enable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

            /* Enable DMA Request */
            hi2c->Instance->CR1 |= I2C_CR1_RXDMAEN;
        }
        else
        {
            /* Update I2C state */
            hi2c->State     = HAL_I2C_STATE_LISTEN;
            hi2c->Mode      = HAL_I2C_MODE_NONE;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

#endif



/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_ASSERT(IS_I2C_MEMADD_SIZE(MemAddSize));
    return I2C_Master_Transmit(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
}

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_ASSERT(IS_I2C_MEMADD_SIZE(MemAddSize));
    return I2C_Master_Receive(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
}


/**
  * @brief  Write an amount of data in non-blocking mode with Interrupt to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
    HAL_ASSERT(IS_I2C_MEMADD_SIZE(MemAddSize));
    return I2C_Master_Transmit_IT(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
}


/**
  * @brief  Read an amount of data in non-blocking mode with Interrupt from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
    HAL_ASSERT(IS_I2C_MEMADD_SIZE(MemAddSize));
    return I2C_Master_Receive_IT(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
}

/**
  * @brief  Write an amount of data in non-blocking mode with DMA to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
    HAL_ASSERT(IS_I2C_MEMADD_SIZE(MemAddSize));
    return I2C_Master_Transmit_DMA(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
}

/**
  * @brief  Reads an amount of data in non-blocking mode with DMA from a specific memory address.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be read
  * @retval HAL status
  */
__HAL_ROM_USED HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
    HAL_ASSERT(IS_I2C_MEMADD_SIZE(MemAddSize));
    return I2C_Master_Receive_DMA(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
}

#if 0
/**
  * @brief  Checks if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  Trials Number of trials
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
    uint32_t tickstart;

    __IO uint32_t I2C_Trials = 0UL;

    FlagStatus tmp1;
    FlagStatus tmp2;

    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(hi2c);

        hi2c->State = HAL_I2C_STATE_BUSY;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        do
        {
            /* Generate Start */
            hi2c->Instance->CR2 = I2C_GENERATE_START(hi2c->Init.AddressingMode, DevAddress);

            /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
            /* Wait until STOPF flag is set or a NACK flag is set*/
            tickstart = HAL_GetTick();

            tmp1 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF);
            tmp2 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF);

            while ((tmp1 == RESET) && (tmp2 == RESET))
            {
                if (Timeout != HAL_MAX_DELAY)
                {
                    if (((HAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
                    {
                        /* Update I2C state */
                        hi2c->State = HAL_I2C_STATE_READY;

                        /* Update I2C error code */
                        hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;

                        /* Process Unlocked */
                        __HAL_UNLOCK(hi2c);

                        return HAL_ERROR;
                    }
                }

                tmp1 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF);
                tmp2 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF);
            }

            /* Check if the NACKF flag has not been set */
            if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF) == RESET)
            {
                /* Wait until STOPF flag is reset */
                if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_STOPF, RESET, Timeout, tickstart) != HAL_OK)
                {
                    return HAL_ERROR;
                }

                /* Clear STOP Flag */
                __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

                /* Device is ready */
                hi2c->State = HAL_I2C_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);

                return HAL_OK;
            }
            else
            {
                /* Wait until STOPF flag is reset */
                if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_STOPF, RESET, Timeout, tickstart) != HAL_OK)
                {
                    return HAL_ERROR;
                }

                /* Clear NACK Flag */
                __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

                /* Clear STOP Flag, auto generated with autoend*/
                __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
            }

            /* Check if the maximum allowed number of trials has been reached */
            if (I2C_Trials == Trials)
            {
                /* Generate Stop */
                hi2c->Instance->CR2 |= I2C_CR2_STOP;

                /* Wait until STOPF flag is reset */
                if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_STOPF, RESET, Timeout, tickstart) != HAL_OK)
                {
                    return HAL_ERROR;
                }

                /* Clear STOP Flag */
                __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
            }

            /* Increment Trials */
            I2C_Trials++;
        }
        while (I2C_Trials < Trials);

        /* Update I2C state */
        hi2c->State = HAL_I2C_STATE_READY;

        /* Update I2C error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Sequential transmit in master I2C mode an amount of data in non-blocking mode with Interrupt.
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Sequential_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
    uint32_t xfermode;
    uint32_t xferrequest = I2C_GENERATE_START_WRITE;

    /* Check the parameters */
    HAL_ASSERT(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        /* Process Locked */
        __HAL_LOCK(hi2c);

        hi2c->State     = HAL_I2C_STATE_BUSY_TX;
        hi2c->Mode      = HAL_I2C_MODE_MASTER;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferOptions = XferOptions;
        hi2c->XferISR     = I2C_Master_ISR_IT;

        /* If hi2c->XferCount > MAX_NBYTE_SIZE, use reload mode */
        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
            hi2c->XferSize = MAX_NBYTE_SIZE;
            xfermode = I2C_RELOAD_MODE;
        }
        else
        {
            hi2c->XferSize = hi2c->XferCount;
            xfermode = hi2c->XferOptions;
        }

        /* If transfer direction not change and there is no request to start another frame, do not generate Restart Condition */
        /* Mean Previous state is same as current state */
        if ((hi2c->PreviousState == I2C_STATE_MASTER_BUSY_TX) && (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 0))
        {
            xferrequest = I2C_NO_STARTSTOP;
        }
        else
        {
            /* Convert OTHER_xxx XferOptions if any */
            I2C_ConvertOtherXferOptions(hi2c);

            /* Update xfermode accordingly if no reload is necessary */
            if (hi2c->XferCount < MAX_NBYTE_SIZE)
            {
                xfermode = hi2c->XferOptions;
            }
        }

        /* Send Slave Address and set NBYTES to write */
        I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, xferrequest);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */
        I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Sequential transmit in master I2C mode an amount of data in non-blocking mode with DMA.
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Sequential_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
    uint32_t xfermode;
    uint32_t xferrequest = I2C_GENERATE_START_WRITE;
    HAL_StatusTypeDef dmaxferstatus;

    /* Check the parameters */
    HAL_ASSERT(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        /* Process Locked */
        __HAL_LOCK(hi2c);

        hi2c->State     = HAL_I2C_STATE_BUSY_TX;
        hi2c->Mode      = HAL_I2C_MODE_MASTER;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferOptions = XferOptions;
        hi2c->XferISR     = I2C_Master_ISR_DMA;

        /* If hi2c->XferCount > MAX_NBYTE_SIZE, use reload mode */
        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
            hi2c->XferSize = MAX_NBYTE_SIZE;
            xfermode = I2C_RELOAD_MODE;
        }
        else
        {
            hi2c->XferSize = hi2c->XferCount;
            xfermode = hi2c->XferOptions;
        }

        /* If transfer direction not change and there is no request to start another frame, do not generate Restart Condition */
        /* Mean Previous state is same as current state */
        if ((hi2c->PreviousState == I2C_STATE_MASTER_BUSY_TX) && (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 0))
        {
            xferrequest = I2C_NO_STARTSTOP;
        }
        else
        {
            /* Convert OTHER_xxx XferOptions if any */
            I2C_ConvertOtherXferOptions(hi2c);

            /* Update xfermode accordingly if no reload is necessary */
            if (hi2c->XferCount < MAX_NBYTE_SIZE)
            {
                xfermode = hi2c->XferOptions;
            }
        }

        if (hi2c->XferSize > 0U)
        {
            if (hi2c->hdmatx != NULL)
            {
                /* Set the I2C DMA transfer complete callback */
                hi2c->hdmatx->XferCpltCallback = I2C_DMAMasterTransmitCplt;

                /* Set the DMA error callback */
                hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

                /* Set the unused DMA callbacks to NULL */
                hi2c->hdmatx->XferHalfCpltCallback = NULL;
                hi2c->hdmatx->XferAbortCallback = NULL;

                /* Enable the DMA channel */
                dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)pData, (uint32_t)&hi2c->Instance->TXDR, hi2c->XferSize);
            }
            else
            {
                /* Update I2C state */
                hi2c->State     = HAL_I2C_STATE_READY;
                hi2c->Mode      = HAL_I2C_MODE_NONE;

                /* Update I2C error code */
                hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);

                return HAL_ERROR;
            }

            if (dmaxferstatus == HAL_OK)
            {
                /* Send Slave Address and set NBYTES to write */
                I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, xferrequest);

                /* Update XferCount value */
                hi2c->XferCount -= hi2c->XferSize;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);

                /* Note : The I2C interrupts must be enabled after unlocking current process
                          to avoid the risk of I2C interrupt handle execution before current
                          process unlock */
                /* Enable ERR and NACK interrupts */
                I2C_Enable_IRQ(hi2c, I2C_XFER_ERROR_IT);

                /* Enable DMA Request */
                hi2c->Instance->DCR |= I2C_DCR_TXREQ;
            }
            else
            {
                /* Update I2C state */
                hi2c->State     = HAL_I2C_STATE_READY;
                hi2c->Mode      = HAL_I2C_MODE_NONE;

                /* Update I2C error code */
                hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);

                return HAL_ERROR;
            }
        }
        else
        {
            /* Update Transfer ISR function pointer */
            hi2c->XferISR = I2C_Master_ISR_IT;

            /* Send Slave Address */
            /* Set NBYTES to write and generate START condition */
            I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_WRITE);

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            /* Note : The I2C interrupts must be enabled after unlocking current process
                      to avoid the risk of I2C interrupt handle execution before current
                      process unlock */
            /* Enable ERR, TC, STOP, NACK, TXI interrupt */
            /* possible to enable all of these */
            /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
            I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT);
        }

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Sequential receive in master I2C mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Sequential_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
    uint32_t xfermode;
    uint32_t xferrequest = I2C_GENERATE_START_READ;

    /* Check the parameters */
    HAL_ASSERT(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        /* Process Locked */
        __HAL_LOCK(hi2c);

        hi2c->State     = HAL_I2C_STATE_BUSY_RX;
        hi2c->Mode      = HAL_I2C_MODE_MASTER;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferOptions = XferOptions;
        hi2c->XferISR     = I2C_Master_ISR_IT;

        /* If hi2c->XferCount > MAX_NBYTE_SIZE, use reload mode */
        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
            hi2c->XferSize = MAX_NBYTE_SIZE;
            xfermode = I2C_RELOAD_MODE;
        }
        else
        {
            hi2c->XferSize = hi2c->XferCount;
            xfermode = hi2c->XferOptions;
        }

        /* If transfer direction not change and there is no request to start another frame, do not generate Restart Condition */
        /* Mean Previous state is same as current state */
        if ((hi2c->PreviousState == I2C_STATE_MASTER_BUSY_RX) && (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 0))
        {
            xferrequest = I2C_NO_STARTSTOP;
        }
        else
        {
            /* Convert OTHER_xxx XferOptions if any */
            I2C_ConvertOtherXferOptions(hi2c);

            /* Update xfermode accordingly if no reload is necessary */
            if (hi2c->XferCount < MAX_NBYTE_SIZE)
            {
                xfermode = hi2c->XferOptions;
            }
        }

        /* Send Slave Address and set NBYTES to read */
        I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, xferrequest);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */
        I2C_Enable_IRQ(hi2c, I2C_XFER_RX_IT);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Sequential receive in master I2C mode an amount of data in non-blocking mode with DMA
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Sequential_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
    uint32_t xfermode;
    uint32_t xferrequest = I2C_GENERATE_START_READ;
    HAL_StatusTypeDef dmaxferstatus;

    /* Check the parameters */
    HAL_ASSERT(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        /* Process Locked */
        __HAL_LOCK(hi2c);

        hi2c->State     = HAL_I2C_STATE_BUSY_RX;
        hi2c->Mode      = HAL_I2C_MODE_MASTER;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferOptions = XferOptions;
        hi2c->XferISR     = I2C_Master_ISR_DMA;

        /* If hi2c->XferCount > MAX_NBYTE_SIZE, use reload mode */
        if (hi2c->XferCount > MAX_NBYTE_SIZE)
        {
            hi2c->XferSize = MAX_NBYTE_SIZE;
            xfermode = I2C_RELOAD_MODE;
        }
        else
        {
            hi2c->XferSize = hi2c->XferCount;
            xfermode = hi2c->XferOptions;
        }

        /* If transfer direction not change and there is no request to start another frame, do not generate Restart Condition */
        /* Mean Previous state is same as current state */
        if ((hi2c->PreviousState == I2C_STATE_MASTER_BUSY_RX) && (IS_I2C_TRANSFER_OTHER_OPTIONS_REQUEST(XferOptions) == 0))
        {
            xferrequest = I2C_NO_STARTSTOP;
        }
        else
        {
            /* Convert OTHER_xxx XferOptions if any */
            I2C_ConvertOtherXferOptions(hi2c);

            /* Update xfermode accordingly if no reload is necessary */
            if (hi2c->XferCount < MAX_NBYTE_SIZE)
            {
                xfermode = hi2c->XferOptions;
            }
        }

        if (hi2c->XferSize > 0U)
        {
            if (hi2c->hdmarx != NULL)
            {
                /* Set the I2C DMA transfer complete callback */
                hi2c->hdmarx->XferCpltCallback = I2C_DMAMasterReceiveCplt;

                /* Set the DMA error callback */
                hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

                /* Set the unused DMA callbacks to NULL */
                hi2c->hdmarx->XferHalfCpltCallback = NULL;
                hi2c->hdmarx->XferAbortCallback = NULL;

                /* Enable the DMA channel */
                dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->RXDR, (uint32_t)pData, hi2c->XferSize);
            }
            else
            {
                /* Update I2C state */
                hi2c->State     = HAL_I2C_STATE_READY;
                hi2c->Mode      = HAL_I2C_MODE_NONE;

                /* Update I2C error code */
                hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);

                return HAL_ERROR;
            }

            if (dmaxferstatus == HAL_OK)
            {
                /* Send Slave Address and set NBYTES to read */
                I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, xfermode, xferrequest);

                /* Update XferCount value */
                hi2c->XferCount -= hi2c->XferSize;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);

                /* Note : The I2C interrupts must be enabled after unlocking current process
                          to avoid the risk of I2C interrupt handle execution before current
                          process unlock */
                /* Enable ERR and NACK interrupts */
                I2C_Enable_IRQ(hi2c, I2C_XFER_ERROR_IT);

                /* Enable DMA Request */
                hi2c->Instance->CR1 |= I2C_CR1_RXDMAEN;
            }
            else
            {
                /* Update I2C state */
                hi2c->State     = HAL_I2C_STATE_READY;
                hi2c->Mode      = HAL_I2C_MODE_NONE;

                /* Update I2C error code */
                hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);

                return HAL_ERROR;
            }
        }
        else
        {
            /* Update Transfer ISR function pointer */
            hi2c->XferISR = I2C_Master_ISR_IT;

            /* Send Slave Address */
            /* Set NBYTES to read and generate START condition */
            I2C_TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            /* Note : The I2C interrupts must be enabled after unlocking current process
                      to avoid the risk of I2C interrupt handle execution before current
                      process unlock */
            /* Enable ERR, TC, STOP, NACK, TXI interrupt */
            /* possible to enable all of these */
            /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
            I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT);
        }

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Sequential transmit in slave/device I2C mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Sequential_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
    /* Check the parameters */
    HAL_ASSERT(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

    if (((uint32_t)hi2c->State & (uint32_t)HAL_I2C_STATE_LISTEN) == (uint32_t)HAL_I2C_STATE_LISTEN)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }

        /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
        I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_TX_IT);

        /* Process Locked */
        __HAL_LOCK(hi2c);

        /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
        /* and then toggle the HAL slave RX state to TX state */
        if (hi2c->State == HAL_I2C_STATE_BUSY_RX_LISTEN)
        {
            /* Disable associated Interrupts */
            I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);

            /* Abort DMA Xfer if any */
            if ((hi2c->Instance->CR1 & I2C_CR1_RXDMAEN) == I2C_CR1_RXDMAEN)
            {
                hi2c->Instance->CR1 &= ~I2C_CR1_RXDMAEN;

                if (hi2c->hdmarx != NULL)
                {
                    /* Set the I2C DMA Abort callback :
                     will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
                    hi2c->hdmarx->XferAbortCallback = I2C_DMAAbort;

                    /* Abort DMA RX */
                    if (HAL_DMA_Abort_IT(hi2c->hdmarx) != HAL_OK)
                    {
                        /* Call Directly XferAbortCallback function in case of error */
                        hi2c->hdmarx->XferAbortCallback(hi2c->hdmarx);
                    }
                }
            }
        }

        hi2c->State     = HAL_I2C_STATE_BUSY_TX_LISTEN;
        hi2c->Mode      = HAL_I2C_MODE_SLAVE;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Enable Address Acknowledge */
        hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferSize    = hi2c->XferCount;
        hi2c->XferOptions = XferOptions;
        hi2c->XferISR     = I2C_Slave_ISR_IT;

        if (I2C_GET_DIR(hi2c) == I2C_DIRECTION_RECEIVE)
        {
            /* Clear ADDR flag after prepare the transfer parameters */
            /* This action will generate an acknowledge to the Master */
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
        to avoid the risk of I2C interrupt handle execution before current
        process unlock */
        /* REnable ADDR interrupt */
        I2C_Enable_IRQ(hi2c, I2C_XFER_TX_IT | I2C_XFER_LISTEN_IT);

        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

/**
  * @brief  Sequential transmit in slave/device I2C mode an amount of data in non-blocking mode with DMA
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Sequential_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
    HAL_StatusTypeDef dmaxferstatus;

    /* Check the parameters */
    HAL_ASSERT(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

    if (((uint32_t)hi2c->State & (uint32_t)HAL_I2C_STATE_LISTEN) == (uint32_t)HAL_I2C_STATE_LISTEN)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(hi2c);

        /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
        I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_TX_IT);

        /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
        /* and then toggle the HAL slave RX state to TX state */
        if (hi2c->State == HAL_I2C_STATE_BUSY_RX_LISTEN)
        {
            /* Disable associated Interrupts */
            I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);

            if ((hi2c->Instance->CR1 & I2C_CR1_RXDMAEN) == I2C_CR1_RXDMAEN)
            {
                /* Abort DMA Xfer if any */
                if (hi2c->hdmarx != NULL)
                {
                    hi2c->Instance->CR1 &= ~I2C_CR1_RXDMAEN;

                    /* Set the I2C DMA Abort callback :
                     will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
                    hi2c->hdmarx->XferAbortCallback = I2C_DMAAbort;

                    /* Abort DMA RX */
                    if (HAL_DMA_Abort_IT(hi2c->hdmarx) != HAL_OK)
                    {
                        /* Call Directly XferAbortCallback function in case of error */
                        hi2c->hdmarx->XferAbortCallback(hi2c->hdmarx);
                    }
                }
            }
        }
        else if (hi2c->State == HAL_I2C_STATE_BUSY_TX_LISTEN)
        {
            if ((hi2c->Instance->CR1 & I2C_CR1_TXDMAEN) == I2C_CR1_TXDMAEN)
            {
                hi2c->Instance->CR1 &= ~I2C_CR1_TXDMAEN;

                /* Abort DMA Xfer if any */
                if (hi2c->hdmatx != NULL)
                {
                    /* Set the I2C DMA Abort callback :
                     will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
                    hi2c->hdmatx->XferAbortCallback = I2C_DMAAbort;

                    /* Abort DMA TX */
                    if (HAL_DMA_Abort_IT(hi2c->hdmatx) != HAL_OK)
                    {
                        /* Call Directly XferAbortCallback function in case of error */
                        hi2c->hdmatx->XferAbortCallback(hi2c->hdmatx);
                    }
                }
            }
        }
        else
        {
            /* Nothing to do */
        }

        hi2c->State     = HAL_I2C_STATE_BUSY_TX_LISTEN;
        hi2c->Mode      = HAL_I2C_MODE_SLAVE;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Enable Address Acknowledge */
        hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferSize    = hi2c->XferCount;
        hi2c->XferOptions = XferOptions;
        hi2c->XferISR     = I2C_Slave_ISR_DMA;

        if (hi2c->hdmatx != NULL)
        {
            /* Set the I2C DMA transfer complete callback */
            hi2c->hdmatx->XferCpltCallback = I2C_DMASlaveTransmitCplt;

            /* Set the DMA error callback */
            hi2c->hdmatx->XferErrorCallback = I2C_DMAError;

            /* Set the unused DMA callbacks to NULL */
            hi2c->hdmatx->XferHalfCpltCallback = NULL;
            hi2c->hdmatx->XferAbortCallback = NULL;

            /* Enable the DMA channel */
            dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)pData, (uint32_t)&hi2c->Instance->TXDR, hi2c->XferSize);
        }
        else
        {
            /* Update I2C state */
            hi2c->State     = HAL_I2C_STATE_LISTEN;
            hi2c->Mode      = HAL_I2C_MODE_NONE;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }

        if (dmaxferstatus == HAL_OK)
        {
            /* Update XferCount value */
            hi2c->XferCount -= hi2c->XferSize;

            /* Reset XferSize */
            hi2c->XferSize = 0;
        }
        else
        {
            /* Update I2C state */
            hi2c->State     = HAL_I2C_STATE_LISTEN;
            hi2c->Mode      = HAL_I2C_MODE_NONE;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }

        if (I2C_GET_DIR(hi2c) == I2C_DIRECTION_RECEIVE)
        {
            /* Clear ADDR flag after prepare the transfer parameters */
            /* This action will generate an acknowledge to the Master */
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
        to avoid the risk of I2C interrupt handle execution before current
        process unlock */
        /* Enable ERR, STOP, NACK, ADDR interrupts */
        I2C_Enable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

        /* Enable DMA Request */
        hi2c->Instance->CR1 |= I2C_CR1_TXDMAEN;

        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

/**
  * @brief  Sequential receive in slave/device I2C mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Sequential_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
    /* Check the parameters */
    HAL_ASSERT(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

    if (((uint32_t)hi2c->State & (uint32_t)HAL_I2C_STATE_LISTEN) == (uint32_t)HAL_I2C_STATE_LISTEN)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }

        /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
        I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_RX_IT);

        /* Process Locked */
        __HAL_LOCK(hi2c);

        /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
        /* and then toggle the HAL slave TX state to RX state */
        if (hi2c->State == HAL_I2C_STATE_BUSY_TX_LISTEN)
        {
            /* Disable associated Interrupts */
            I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

            if ((hi2c->Instance->CR1 & I2C_CR1_TXDMAEN) == I2C_CR1_TXDMAEN)
            {
                hi2c->Instance->CR1 &= ~I2C_CR1_TXDMAEN;

                /* Abort DMA Xfer if any */
                if (hi2c->hdmatx != NULL)
                {
                    /* Set the I2C DMA Abort callback :
                     will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
                    hi2c->hdmatx->XferAbortCallback = I2C_DMAAbort;

                    /* Abort DMA TX */
                    if (HAL_DMA_Abort_IT(hi2c->hdmatx) != HAL_OK)
                    {
                        /* Call Directly XferAbortCallback function in case of error */
                        hi2c->hdmatx->XferAbortCallback(hi2c->hdmatx);
                    }
                }
            }
        }

        hi2c->State     = HAL_I2C_STATE_BUSY_RX_LISTEN;
        hi2c->Mode      = HAL_I2C_MODE_SLAVE;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Enable Address Acknowledge */
        hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferSize    = hi2c->XferCount;
        hi2c->XferOptions = XferOptions;
        hi2c->XferISR     = I2C_Slave_ISR_IT;

        if (I2C_GET_DIR(hi2c) == I2C_DIRECTION_TRANSMIT)
        {
            /* Clear ADDR flag after prepare the transfer parameters */
            /* This action will generate an acknowledge to the Master */
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
        to avoid the risk of I2C interrupt handle execution before current
        process unlock */
        /* REnable ADDR interrupt */
        I2C_Enable_IRQ(hi2c, I2C_XFER_RX_IT | I2C_XFER_LISTEN_IT);

        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

/**
  * @brief  Sequential receive in slave/device I2C mode an amount of data in non-blocking mode with DMA
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Slave_Sequential_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions)
{
    HAL_StatusTypeDef dmaxferstatus;

    /* Check the parameters */
    HAL_ASSERT(IS_I2C_TRANSFER_OPTIONS_REQUEST(XferOptions));

    if (((uint32_t)hi2c->State & (uint32_t)HAL_I2C_STATE_LISTEN) == (uint32_t)HAL_I2C_STATE_LISTEN)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
            return  HAL_ERROR;
        }

        /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
        I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_RX_IT);

        /* Process Locked */
        __HAL_LOCK(hi2c);

        /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
        /* and then toggle the HAL slave TX state to RX state */
        if (hi2c->State == HAL_I2C_STATE_BUSY_TX_LISTEN)
        {
            /* Disable associated Interrupts */
            I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

            if ((hi2c->Instance->CR1 & I2C_CR1_TXDMAEN) == I2C_CR1_TXDMAEN)
            {
                /* Abort DMA Xfer if any */
                if (hi2c->hdmatx != NULL)
                {
                    hi2c->Instance->CR1 &= ~I2C_CR1_TXDMAEN;

                    /* Set the I2C DMA Abort callback :
                     will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
                    hi2c->hdmatx->XferAbortCallback = I2C_DMAAbort;

                    /* Abort DMA TX */
                    if (HAL_DMA_Abort_IT(hi2c->hdmatx) != HAL_OK)
                    {
                        /* Call Directly XferAbortCallback function in case of error */
                        hi2c->hdmatx->XferAbortCallback(hi2c->hdmatx);
                    }
                }
            }
        }
        else if (hi2c->State == HAL_I2C_STATE_BUSY_RX_LISTEN)
        {
            if ((hi2c->Instance->CR1 & I2C_CR1_RXDMAEN) == I2C_CR1_RXDMAEN)
            {
                hi2c->Instance->CR1 &= ~I2C_CR1_RXDMAEN;

                /* Abort DMA Xfer if any */
                if (hi2c->hdmarx != NULL)
                {
                    /* Set the I2C DMA Abort callback :
                     will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
                    hi2c->hdmarx->XferAbortCallback = I2C_DMAAbort;

                    /* Abort DMA RX */
                    if (HAL_DMA_Abort_IT(hi2c->hdmarx) != HAL_OK)
                    {
                        /* Call Directly XferAbortCallback function in case of error */
                        hi2c->hdmarx->XferAbortCallback(hi2c->hdmarx);
                    }
                }
            }
        }
        else
        {
            /* Nothing to do */
        }

        hi2c->State     = HAL_I2C_STATE_BUSY_RX_LISTEN;
        hi2c->Mode      = HAL_I2C_MODE_SLAVE;
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

        /* Enable Address Acknowledge */
        hi2c->Instance->CR2 &= ~I2C_CR2_NACK;

        /* Prepare transfer parameters */
        hi2c->pBuffPtr    = pData;
        hi2c->XferCount   = Size;
        hi2c->XferSize    = hi2c->XferCount;
        hi2c->XferOptions = XferOptions;
        hi2c->XferISR     = I2C_Slave_ISR_DMA;

        if (hi2c->hdmarx != NULL)
        {
            /* Set the I2C DMA transfer complete callback */
            hi2c->hdmarx->XferCpltCallback = I2C_DMASlaveReceiveCplt;

            /* Set the DMA error callback */
            hi2c->hdmarx->XferErrorCallback = I2C_DMAError;

            /* Set the unused DMA callbacks to NULL */
            hi2c->hdmarx->XferHalfCpltCallback = NULL;
            hi2c->hdmarx->XferAbortCallback = NULL;

            /* Enable the DMA channel */
            dmaxferstatus = HAL_DMA_Start_IT(hi2c->hdmarx, (uint32_t)&hi2c->Instance->RXDR, (uint32_t)pData, hi2c->XferSize);
        }
        else
        {
            /* Update I2C state */
            hi2c->State     = HAL_I2C_STATE_LISTEN;
            hi2c->Mode      = HAL_I2C_MODE_NONE;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_DMA_PARAM;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }

        if (dmaxferstatus == HAL_OK)
        {
            /* Update XferCount value */
            hi2c->XferCount -= hi2c->XferSize;

            /* Reset XferSize */
            hi2c->XferSize = 0;
        }
        else
        {
            /* Update I2C state */
            hi2c->State     = HAL_I2C_STATE_LISTEN;
            hi2c->Mode      = HAL_I2C_MODE_NONE;

            /* Update I2C error code */
            hi2c->ErrorCode |= HAL_I2C_ERROR_DMA;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }

        if (I2C_GET_DIR(hi2c) == I2C_DIRECTION_TRANSMIT)
        {
            /* Clear ADDR flag after prepare the transfer parameters */
            /* This action will generate an acknowledge to the Master */
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
        to avoid the risk of I2C interrupt handle execution before current
        process unlock */
        /* REnable ADDR interrupt */
        I2C_Enable_IRQ(hi2c, I2C_XFER_RX_IT | I2C_XFER_LISTEN_IT);

        /* Enable DMA Request */
        hi2c->Instance->CR1 |= I2C_CR1_RXDMAEN;

        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

/**
  * @brief  Enable the Address listen mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        hi2c->State = HAL_I2C_STATE_LISTEN;
        hi2c->XferISR = I2C_Slave_ISR_IT;

        /* Enable the Address Match interrupt */
        I2C_Enable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Disable the Address listen mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c)
{
    /* Declaration of tmp to prevent undefined behavior of volatile usage */
    uint32_t tmp;

    /* Disable Address listen mode only if a transfer is not ongoing */
    if (hi2c->State == HAL_I2C_STATE_LISTEN)
    {
        tmp = (uint32_t)(hi2c->State) & I2C_STATE_MSK;
        hi2c->PreviousState = tmp | (uint32_t)(hi2c->Mode);
        hi2c->State = HAL_I2C_STATE_READY;
        hi2c->Mode = HAL_I2C_MODE_NONE;
        hi2c->XferISR = NULL;

        /* Disable the Address Match interrupt */
        I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Abort a master I2C IT or DMA process communication with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    if (hi2c->Mode == HAL_I2C_MODE_MASTER)
    {
        /* Process Locked */
        __HAL_LOCK(hi2c);

        /* Disable Interrupts */
        I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);
        I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

        /* Set State at HAL_I2C_STATE_ABORT */
        hi2c->State = HAL_I2C_STATE_ABORT;

        /* Set NBYTES to 1 to generate a dummy read on I2C peripheral */
        /* Set AUTOEND mode, this will generate a NACK then STOP condition to abort the current transfer */
        I2C_TransferConfig(hi2c, DevAddress, 1, I2C_AUTOEND_MODE, I2C_GENERATE_STOP);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                  to avoid the risk of I2C interrupt handle execution before current
                  process unlock */
        I2C_Enable_IRQ(hi2c, I2C_XFER_CPLT_IT);

        return HAL_OK;
    }
    else
    {
        /* Wrong usage of abort function */
        /* This function should be used only in case of abort monitored by master device */
        return HAL_ERROR;
    }
}

/**
  * @}
  */

/** @defgroup I2C_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
 * @{
 */

/**
  * @brief  This function handles I2C event interrupt request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c)
{
    /* Get current IT Flags and IT sources value */
    uint32_t itflags   = READ_REG(hi2c->Instance->ISR);
    uint32_t itsources = READ_REG(hi2c->Instance->CR1);

    /* I2C events treatment -------------------------------------*/
    if (hi2c->XferISR != NULL)
    {
        hi2c->XferISR(hi2c, itflags, itsources);
    }
}

/**
  * @brief  This function handles I2C error interrupt request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c)
{
    uint32_t itflags   = READ_REG(hi2c->Instance->ISR);
    uint32_t itsources = READ_REG(hi2c->Instance->CR1);
    uint32_t tmperror;

    /* I2C Bus error interrupt occurred ------------------------------------*/
    if ((I2C_CHECK_FLAG(itflags, I2C_FLAG_BERR) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_ERRI) != RESET))
    {
        hi2c->ErrorCode |= HAL_I2C_ERROR_BERR;

        /* Clear BERR flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BERR);
    }

    /* I2C Over-Run/Under-Run interrupt occurred ----------------------------------------*/
    if ((I2C_CHECK_FLAG(itflags, I2C_FLAG_OVR) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_ERRI) != RESET))
    {
        hi2c->ErrorCode |= HAL_I2C_ERROR_OVR;

        /* Clear OVR flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_OVR);
    }

    /* I2C Arbitration Loss error interrupt occurred -------------------------------------*/
    if ((I2C_CHECK_FLAG(itflags, I2C_FLAG_ARLO) != RESET) && (I2C_CHECK_IT_SOURCE(itsources, I2C_IT_ERRI) != RESET))
    {
        hi2c->ErrorCode |= HAL_I2C_ERROR_ARLO;

        /* Clear ARLO flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ARLO);
    }

    /* Store current volatile hi2c->ErrorCode, misra rule */
    tmperror = hi2c->ErrorCode;

    /* Call the Error Callback in case of Error detected */
    if ((tmperror & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_OVR | HAL_I2C_ERROR_ARLO)) !=  HAL_I2C_ERROR_NONE)
    {
        I2C_ITError(hi2c, tmperror);
    }
}
#endif
/**
  * @brief  Master Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->MasterTxCpltCallback(hi2c);
#else

    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2c);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_I2C_MasterTxCpltCallback could be implemented in the user file
     */
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
}

/**
  * @brief  Master Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->MasterRxCpltCallback(hi2c);
#else

    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2c);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_I2C_MasterRxCpltCallback could be implemented in the user file
     */
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
}

#if HAL_I2CSLAVE_MODULE_ENABLED
/** @brief  Slave Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2c);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_I2C_SlaveTxCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  Slave Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2c);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_I2C_SlaveRxCpltCallback could be implemented in the user file
     */
}
#endif

/**
  * @brief  Slave Address Match callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  TransferDirection Master request Transfer Direction (Write/Read), value of @ref I2C_XFERDIRECTION
  * @param  AddrMatchCode Address Match Code
  * @retval None
  */
__weak void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2c);
    UNUSED(TransferDirection);
    UNUSED(AddrMatchCode);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_I2C_AddrCallback() could be implemented in the user file
     */
}

/**
  * @brief  Listen Complete callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2c);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_I2C_ListenCpltCallback() could be implemented in the user file
     */
}

/**
  * @brief  Memory Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->MemTxCpltCallback(hi2c);
#else

    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2c);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_I2C_MemTxCpltCallback could be implemented in the user file
     */
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
}

/**
  * @brief  Memory Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->MemRxCpltCallback(hi2c);
#else

    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2c);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_I2C_MemRxCpltCallback could be implemented in the user file
     */
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
}

/**
  * @brief  I2C error callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->ErrorCallback(hi2c);
#else

    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2c);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_I2C_ErrorCallback could be implemented in the user file
     */
#endif
}

/**
  * @brief  I2C abort callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->AbortCpltCallback(hi2c);
#else

    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2c);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_I2C_AbortCpltCallback could be implemented in the user file
     */
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
}

/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group3 Peripheral State, Mode and Error functions
 *  @brief   Peripheral State, Mode and Error functions
 *
@verbatim
 ===============================================================================
            ##### Peripheral State, Mode and Error functions #####
 ===============================================================================
    [..]
    This subsection permit to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the I2C handle state.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL state
  */
__HAL_ROM_USED HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c)
{
    /* Return I2C handle state */
    // for Ready status, need may sure UB status clear
    if (hi2c->State == HAL_I2C_STATE_READY)
    {
        if (__HAL_I2C_GET_FLAG(hi2c, I2C_SR_UB) == SET) // prev action not finish
            return HAL_I2C_STATE_BUSY;
    }
    return hi2c->State;
}

/**
  * @brief  Returns the I2C Master, Slave, Memory or no mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval HAL mode
  */
__HAL_ROM_USED HAL_I2C_ModeTypeDef HAL_I2C_GetMode(I2C_HandleTypeDef *hi2c)
{
    return hi2c->Mode;
}

/**
* @brief  Return the I2C error code.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
* @retval I2C Error Code
*/
__HAL_ROM_USED uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *hi2c)
{
    return hi2c->ErrorCode;
}


/**
* @brief  Send I2C bus reset and reset I2C controller.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
* @retval I2C Error Code
*/
uint32_t  HAL_I2C_Reset(I2C_HandleTypeDef *hi2c)
{
    /*
        Overwrite SLV setting for:
        1. Make sure SLV setting works
        2. Fix A0 LCPU ROM BUG: SLV been set to 0

        Standard mode only use to send bus reset, use lowest frequence
    */
    MODIFY_REG(hi2c->Instance->LCR, I2C_LCR_SLV_Msk, I2C_LCR_SLV_Msk);

    hi2c->Instance->CR |= I2C_CR_UR;
    HAL_Delay_us(1);        // Delay at least 9 cycle.
    hi2c->Instance->CR &= ~I2C_CR_UR;

    /*Reset I2C Bus*/
    hi2c->Instance->CR |= I2C_CR_RSTREQ;
    while (hi2c->Instance->CR & I2C_CR_RSTREQ)
        HAL_Delay_us(1);


    hi2c->Instance->CR |= I2C_CR_UR;
    HAL_Delay_us(1);        // Delay at least 9 cycle.
    hi2c->Instance->CR &= ~I2C_CR_UR;


    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    hi2c->State = HAL_I2C_STATE_READY;

    return 0;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup I2C_Private_Functions
  * @{
  */

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Master_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources)
{
#if 1
    /* Process Locked */
    __HAL_LOCK(hi2c);

    ITFlags = hi2c->Instance->SR;
    hi2c->Instance->SR = ITFlags;
    ITSources = hi2c->Instance->IER;


    if ((HAL_I2C_STATE_BUSY_TX != hi2c->State) && (HAL_I2C_STATE_BUSY_RX != hi2c->State))
    {
        HAL_I2C_LOG("I2C_Master_ISR_IT Err SR=%x, IER=%x.", ITFlags, ITSources);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
    }

    if (ITFlags & (I2C_SR_TE | I2C_SR_RF))
    {
        if (HAL_I2C_STATE_BUSY_RX == hi2c->State)
        {
            if (hi2c->XferCount > 0)
            {
                /* Read data from RXDR */
                *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DBR;

                /* Increment Buffer pointer */
                hi2c->pBuffPtr++;
                hi2c->XferSize--;
                hi2c->XferCount--;

                if (0 == hi2c->XferCount)
                    HAL_I2C_LOG("I2C_Master_ISR_IT Last byte RX Done.");
                else if (1 == hi2c->XferCount)
                    hi2c->Instance->TCR = I2C_TCR_TB | I2C_TCR_STOP | I2C_TCR_NACK;
                else
                    hi2c->Instance->TCR = I2C_TCR_TB;
            }
        }
        else if (HAL_I2C_STATE_BUSY_TX == hi2c->State)
        {
            if (0 == hi2c->XferCount)
            {
                HAL_I2C_LOG("I2C_Master_ISR_IT Last byte TX Done.");
            }
            else
            {
                /* Send next byte*/
                hi2c->Instance->DBR = *hi2c->pBuffPtr;
                if (1 == hi2c->XferCount)
                    hi2c->Instance->TCR = I2C_TCR_TB | I2C_TCR_STOP;
                else
                    hi2c->Instance->TCR = I2C_TCR_TB;
                /* Increment Buffer pointer */
                hi2c->pBuffPtr++;
                hi2c->XferSize--;
                hi2c->XferCount--;
            }

        }
    }





    /*Check xfer errors*/
    if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_STOPF) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_STOPI) != RESET))
    {
        /* Call I2C Master complete process */
        HAL_I2C_LOG("I2C_Master_ISR_IT STOP bit finished. SR=%x", ITFlags);

        if (0 != hi2c->XferCount)
        {
            hi2c->ErrorCode |= HAL_I2C_ERROR_BERR;
            HAL_I2C_LOG("Received STOP while Xfer NOT done! Left %d", hi2c->XferCount);
        }

        I2C_ITMasterCplt(hi2c, ITFlags);
    }
    else if (I2C_CHECK_FLAG(ITFlags, I2C_FLAG_AF) != RESET)
    {
        HAL_I2C_LOG("I2C_Master_ISR_IT NACK.");

        I2C_ITError(hi2c, HAL_I2C_ERROR_AF);
    }


    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

#else
    uint16_t devaddress;

    /* Process Locked */
    __HAL_LOCK(hi2c);

    if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_AF) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_NACKI) != RESET))
    {
        /* Clear NACK Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

        /* Set corresponding Error Code */
        /* No need to generate STOP, it is automatically done */
        /* Error callback will be send during stop flag treatment */
        hi2c->ErrorCode |= HAL_I2C_ERROR_AF;

        /* Flush TX register */
        I2C_Flush_TXDR(hi2c);
    }
    else if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_RXNE) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_RXI) != RESET))
    {
        /* Read data from RXDR */
        *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        hi2c->XferSize--;
        hi2c->XferCount--;
    }
    else if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_TXIS) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_TXI) != RESET))
    {
        /* Write data to TXDR */
        hi2c->Instance->TXDR = *hi2c->pBuffPtr;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        hi2c->XferSize--;
        hi2c->XferCount--;
    }
    else if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_TCR) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_TCI) != RESET))
    {
        if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
        {
            devaddress = (uint16_t)(hi2c->Instance->CR2 & I2C_CR2_SADD);

            if (hi2c->XferCount > MAX_NBYTE_SIZE)
            {
                hi2c->XferSize = MAX_NBYTE_SIZE;
                I2C_TransferConfig(hi2c, devaddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
            }
            else
            {
                hi2c->XferSize = hi2c->XferCount;
                if (hi2c->XferOptions != I2C_NO_OPTION_FRAME)
                {
                    I2C_TransferConfig(hi2c, devaddress, (uint8_t)hi2c->XferSize, hi2c->XferOptions, I2C_NO_STARTSTOP);
                }
                else
                {
                    I2C_TransferConfig(hi2c, devaddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
                }
            }
        }
        else
        {
            /* Call TxCpltCallback() if no stop mode is set */
            if (I2C_GET_STOP_MODE(hi2c) != I2C_AUTOEND_MODE)
            {
                /* Call I2C Master Sequential complete process */
                I2C_ITMasterSequentialCplt(hi2c);
            }
            else
            {
                /* Wrong size Status regarding TCR flag event */
                /* Call the corresponding callback to inform upper layer of End of Transfer */
                I2C_ITError(hi2c, HAL_I2C_ERROR_SIZE);
            }
        }
    }
    else if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_TC) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_TCI) != RESET))
    {
        if (hi2c->XferCount == 0U)
        {
            if (I2C_GET_STOP_MODE(hi2c) != I2C_AUTOEND_MODE)
            {
                /* Generate a stop condition in case of no transfer option */
                if (hi2c->XferOptions == I2C_NO_OPTION_FRAME)
                {
                    /* Generate Stop */
                    hi2c->Instance->CR2 |= I2C_CR2_STOP;
                }
                else
                {
                    /* Call I2C Master Sequential complete process */
                    I2C_ITMasterSequentialCplt(hi2c);
                }
            }
        }
        else
        {
            /* Wrong size Status regarding TC flag event */
            /* Call the corresponding callback to inform upper layer of End of Transfer */
            I2C_ITError(hi2c, HAL_I2C_ERROR_SIZE);
        }
    }
    else
    {
        /* Nothing to do */
    }

    if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_STOPF) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_STOPI) != RESET))
    {
        /* Call I2C Master complete process */
        I2C_ITMasterCplt(hi2c, ITFlags);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);


#endif
    return HAL_OK;
}

#if HAL_I2CSLAVE_MODULE_ENABLED
/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Slave_ISR_IT(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources)
{
    uint32_t tmpoptions = hi2c->XferOptions;

    /* Process locked */
    __HAL_LOCK(hi2c);

    if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_AF) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_NACKI) != RESET))
    {
        /* Check that I2C transfer finished */
        /* if yes, normal use case, a NACK is sent by the MASTER when Transfer is finished */
        /* Mean XferCount == 0*/
        /* So clear Flag NACKF only */
        if (hi2c->XferCount == 0U)
        {
            if ((hi2c->State == HAL_I2C_STATE_LISTEN) && (tmpoptions == I2C_FIRST_AND_LAST_FRAME)) /* Same action must be done for (tmpoptions == I2C_LAST_FRAME) which removed for Warning[Pa134]: left and right operands are identical */
            {
                /* Call I2C Listen complete process */
                I2C_ITListenCplt(hi2c, ITFlags);
            }
            else if ((hi2c->State == HAL_I2C_STATE_BUSY_TX_LISTEN) && (tmpoptions != I2C_NO_OPTION_FRAME))
            {
                /* Clear NACK Flag */
                __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

                /* Flush TX register */
                I2C_Flush_TXDR(hi2c);

                /* Last Byte is Transmitted */
                /* Call I2C Slave Sequential complete process */
                I2C_ITSlaveSequentialCplt(hi2c);
            }
            else
            {
                /* Clear NACK Flag */
                __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
            }
        }
        else
        {
            /* if no, error use case, a Non-Acknowledge of last Data is generated by the MASTER*/
            /* Clear NACK Flag */
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

            /* Set ErrorCode corresponding to a Non-Acknowledge */
            hi2c->ErrorCode |= HAL_I2C_ERROR_AF;

            if ((tmpoptions == I2C_FIRST_FRAME) || (tmpoptions == I2C_NEXT_FRAME))
            {
                /* Call the corresponding callback to inform upper layer of End of Transfer */
                I2C_ITError(hi2c, hi2c->ErrorCode);
            }
        }
    }
    else if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_RXNE) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_RXI) != RESET))
    {
        if (hi2c->XferCount > 0U)
        {
            /* Read data from RXDR */
            *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

            /* Increment Buffer pointer */
            hi2c->pBuffPtr++;

            hi2c->XferSize--;
            hi2c->XferCount--;
        }

        if ((hi2c->XferCount == 0U) && \
                (tmpoptions != I2C_NO_OPTION_FRAME))
        {
            /* Call I2C Slave Sequential complete process */
            I2C_ITSlaveSequentialCplt(hi2c);
        }
    }
    else if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_ADDR) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_ADDRI) != RESET))
    {
        I2C_ITAddrCplt(hi2c, ITFlags);
    }
    else if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_TXIS) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_TXI) != RESET))
    {
        /* Write data to TXDR only if XferCount not reach "0" */
        /* A TXIS flag can be set, during STOP treatment      */
        /* Check if all Datas have already been sent */
        /* If it is the case, this last write in TXDR is not sent, correspond to a dummy TXIS event */
        if (hi2c->XferCount > 0U)
        {
            /* Write data to TXDR */
            hi2c->Instance->TXDR = *hi2c->pBuffPtr;

            /* Increment Buffer pointer */
            hi2c->pBuffPtr++;

            hi2c->XferCount--;
            hi2c->XferSize--;
        }
        else
        {
            if ((tmpoptions == I2C_NEXT_FRAME) || (tmpoptions == I2C_FIRST_FRAME))
            {
                /* Last Byte is Transmitted */
                /* Call I2C Slave Sequential complete process */
                I2C_ITSlaveSequentialCplt(hi2c);
            }
        }
    }
    else
    {
        /* Nothing to do */
    }

    /* Check if STOPF is set */
    if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_STOPF) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_STOPI) != RESET))
    {
        /* Call I2C Slave complete process */
        I2C_ITSlaveCplt(hi2c, ITFlags);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
}
#endif

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with DMA.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Master_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources)
{

    /* Process Locked */
    __HAL_LOCK(hi2c);

    ITFlags = hi2c->Instance->SR;
    hi2c->Instance->SR = ITFlags;
    ITSources = hi2c->Instance->IER;

    if ((HAL_I2C_STATE_BUSY_TX != hi2c->State) && (HAL_I2C_STATE_BUSY_RX != hi2c->State))
    {
        HAL_I2C_LOG("I2C_Master_ISR_DMA Err SR=%x, IER=%x.", ITFlags, ITSources);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
    }



    if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_STOPF) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_STOPI) != RESET))
    {
        /* Call I2C Master complete process */

        HAL_I2C_LOG("I2C_Master_ISR_DMA STOP bit finished.");

        /* Disable DMA Request */
        if (HAL_I2C_STATE_BUSY_TX == hi2c->State)
            hi2c->Instance->TCR &= ~I2C_TCR_TXREQ;
        else
            hi2c->Instance->TCR &= ~I2C_TCR_RXREQ;

#if 0
        HAL_I2C_LOG("SR1 %x ", hi2c->Instance->SR);

        uint32_t tickstart = HAL_GetTick();
        I2C_WaitOnFlagUntilTimeout(hi2c, I2C_SR_UB, SET, 100, tickstart);

        HAL_I2C_LOG("SR1 %x ", hi2c->Instance->SR);
#endif /* 0 */

        I2C_ITMasterCplt(hi2c, ITFlags);

    }
    else if (I2C_CHECK_FLAG(ITFlags, I2C_FLAG_AF) != RESET)
    {
        HAL_I2C_LOG("I2C_Master_ISR_DMA NACK");

        I2C_ITError(hi2c, HAL_I2C_ERROR_AF);
    }
    else if ((I2C_CHECK_FLAG(ITFlags, I2C_SR_DMADONE) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IER_DMADONEIE) != RESET))
    {
        if (hi2c->XferCount > 0)
        {
            /* Update Buffer pointer */
            hi2c->pBuffPtr += hi2c->XferSize;

            /* Set the XferSize to transfer */
            if (hi2c->XferCount > MAX_NBYTE_SIZE)
            {
                hi2c->XferSize = MAX_NBYTE_SIZE;
            }
            else
            {
                hi2c->XferSize = hi2c->XferCount;
            }

            hi2c->XferCount -= hi2c->XferSize;      /* Update XferCount value */


            //Reload dma
            I2C_Master_DMAReload(hi2c);
        }
        else
        {
            HAL_I2C_LOG("DMA xfer Done.");
        }

    }
    else
    {
        HAL_ASSERT(0);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
}

#if HAL_I2CSLAVE_MODULE_ENABLED
/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with DMA.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_Slave_ISR_DMA(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources)
{
    uint32_t tmpoptions = hi2c->XferOptions;
    uint32_t treatdmanack = 0U;

    /* Process locked */
    __HAL_LOCK(hi2c);

    if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_AF) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_NACKI) != RESET))
    {
        /* Check that I2C transfer finished */
        /* if yes, normal use case, a NACK is sent by the MASTER when Transfer is finished */
        /* Mean XferCount == 0 */
        /* So clear Flag NACKF only */
        if ((I2C_CHECK_IT_SOURCE(ITSources, I2C_CR1_TXDMAEN) != RESET) ||
                (I2C_CHECK_IT_SOURCE(ITSources, I2C_CR1_RXDMAEN) != RESET))
        {
            /* Split check of hdmarx, for MISRA compliance */
            if (hi2c->hdmarx != NULL)
            {
                if (__HAL_DMA_GET_COUNTER(hi2c->hdmarx) == 0U)
                {
                    treatdmanack = 1U;
                }
            }

            /* Split check of hdmatx, for MISRA compliance  */
            if (hi2c->hdmatx != NULL)
            {
                if (__HAL_DMA_GET_COUNTER(hi2c->hdmatx) == 0U)
                {
                    treatdmanack = 1U;
                }
            }

            if (treatdmanack == 1U)
            {
                if ((hi2c->State == HAL_I2C_STATE_LISTEN) && (tmpoptions == I2C_FIRST_AND_LAST_FRAME)) /* Same action must be done for (tmpoptions == I2C_LAST_FRAME) which removed for Warning[Pa134]: left and right operands are identical */
                {
                    /* Call I2C Listen complete process */
                    I2C_ITListenCplt(hi2c, ITFlags);
                }
                else if ((hi2c->State == HAL_I2C_STATE_BUSY_TX_LISTEN) && (tmpoptions != I2C_NO_OPTION_FRAME))
                {
                    /* Clear NACK Flag */
                    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

                    /* Flush TX register */
                    I2C_Flush_TXDR(hi2c);

                    /* Last Byte is Transmitted */
                    /* Call I2C Slave Sequential complete process */
                    I2C_ITSlaveSequentialCplt(hi2c);
                }
                else
                {
                    /* Clear NACK Flag */
                    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
                }
            }
            else
            {
                /* if no, error use case, a Non-Acknowledge of last Data is generated by the MASTER*/
                /* Clear NACK Flag */
                __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

                /* Set ErrorCode corresponding to a Non-Acknowledge */
                hi2c->ErrorCode |= HAL_I2C_ERROR_AF;

                if ((tmpoptions == I2C_FIRST_FRAME) || (tmpoptions == I2C_NEXT_FRAME))
                {
                    /* Call the corresponding callback to inform upper layer of End of Transfer */
                    I2C_ITError(hi2c, hi2c->ErrorCode);
                }
            }
        }
        else
        {
            /* Only Clear NACK Flag, no DMA treatment is pending */
            __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
        }
    }
    else if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_ADDR) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_ADDRI) != RESET))
    {
        I2C_ITAddrCplt(hi2c, ITFlags);
    }
    else if ((I2C_CHECK_FLAG(ITFlags, I2C_FLAG_STOPF) != RESET) && (I2C_CHECK_IT_SOURCE(ITSources, I2C_IT_STOPI) != RESET))
    {
        /* Call I2C Slave complete process */
        I2C_ITSlaveCplt(hi2c, ITFlags);
    }
    else
    {
        /* Nothing to do */
    }

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
}


#endif


#if HAL_I2CSLAVE_MODULE_ENABLED
/**
  * @brief  I2C Address complete process callback.
  * @param  hi2c I2C handle.
  * @param  ITFlags Interrupt flags to handle.
  * @retval None
  */
static void I2C_ITAddrCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags)
{
    uint8_t transferdirection;
    uint16_t slaveaddrcode;
    uint16_t ownadd1code;
    uint16_t ownadd2code;

    /* Prevent unused argument(s) compilation warning */
    UNUSED(ITFlags);

    /* In case of Listen state, need to inform upper layer of address match code event */
    if (((uint32_t)hi2c->State & (uint32_t)HAL_I2C_STATE_LISTEN) == (uint32_t)HAL_I2C_STATE_LISTEN)
    {
        transferdirection = I2C_GET_DIR(hi2c);
        slaveaddrcode     = I2C_GET_ADDR_MATCH(hi2c);
        ownadd1code       = I2C_GET_OWN_ADDRESS1(hi2c);
        ownadd2code       = I2C_GET_OWN_ADDRESS2(hi2c);

        /* If 10bits addressing mode is selected */
        if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_10BIT)
        {
            if ((slaveaddrcode & SlaveAddr_MSK) == ((ownadd1code >> SlaveAddr_SHIFT) & SlaveAddr_MSK))
            {
                slaveaddrcode = ownadd1code;
                hi2c->AddrEventCount++;
                if (hi2c->AddrEventCount == 2U)
                {
                    /* Reset Address Event counter */
                    hi2c->AddrEventCount = 0U;

                    /* Clear ADDR flag */
                    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

                    /* Process Unlocked */
                    __HAL_UNLOCK(hi2c);

                    /* Call Slave Addr callback */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
                    hi2c->AddrCallback(hi2c, transferdirection, slaveaddrcode);
#else
                    HAL_I2C_AddrCallback(hi2c, transferdirection, slaveaddrcode);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
                }
            }
            else
            {
                slaveaddrcode = ownadd2code;

                /* Disable ADDR Interrupts */
                I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);

                /* Call Slave Addr callback */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
                hi2c->AddrCallback(hi2c, transferdirection, slaveaddrcode);
#else
                HAL_I2C_AddrCallback(hi2c, transferdirection, slaveaddrcode);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
            }
        }
        /* else 7 bits addressing mode is selected */
        else
        {
            /* Disable ADDR Interrupts */
            I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT);

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            /* Call Slave Addr callback */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
            hi2c->AddrCallback(hi2c, transferdirection, slaveaddrcode);
#else
            HAL_I2C_AddrCallback(hi2c, transferdirection, slaveaddrcode);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
        }
    }
    /* Else clear address flag only */
    else
    {
        /* Clear ADDR flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);
    }
}

/**
  * @brief  I2C Slave sequential complete process.
  * @param  hi2c I2C handle.
  * @retval None
  */
static void I2C_ITSlaveSequentialCplt(I2C_HandleTypeDef *hi2c)
{
    /* Reset I2C handle mode */
    hi2c->Mode = HAL_I2C_MODE_NONE;

    if (hi2c->State == HAL_I2C_STATE_BUSY_TX_LISTEN)
    {
        /* Remove HAL_I2C_STATE_SLAVE_BUSY_TX, keep only HAL_I2C_STATE_LISTEN */
        hi2c->State         = HAL_I2C_STATE_LISTEN;
        hi2c->PreviousState = I2C_STATE_SLAVE_BUSY_TX;

        /* Disable Interrupts */
        I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->SlaveTxCpltCallback(hi2c);
#else
        HAL_I2C_SlaveTxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }

    else if (hi2c->State == HAL_I2C_STATE_BUSY_RX_LISTEN)
    {
        /* Remove HAL_I2C_STATE_SLAVE_BUSY_RX, keep only HAL_I2C_STATE_LISTEN */
        hi2c->State         = HAL_I2C_STATE_LISTEN;
        hi2c->PreviousState = I2C_STATE_SLAVE_BUSY_RX;

        /* Disable Interrupts */
        I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->SlaveRxCpltCallback(hi2c);
#else
        HAL_I2C_SlaveRxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }
    else
    {
        /* Nothing to do */
    }
}


/**
  * @brief  I2C Master sequential complete process.
  * @param  hi2c I2C handle.
  * @retval None
  */
static void I2C_ITMasterSequentialCplt(I2C_HandleTypeDef *hi2c)
{
#if 0
    /* Reset I2C handle mode */
    hi2c->Mode = HAL_I2C_MODE_NONE;

    /* No Generate Stop, to permit restart mode */
    /* The stop will be done at the end of transfer, when I2C_AUTOEND_MODE enable */
    if (hi2c->State == HAL_I2C_STATE_BUSY_TX)
    {
        hi2c->State         = HAL_I2C_STATE_READY;
        hi2c->PreviousState = I2C_STATE_MASTER_BUSY_TX;
        hi2c->XferISR       = NULL;

        /* Disable Interrupts */
        I2C_Disable_IRQ(hi2c, I2C_XFER_TX_IT);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->MasterTxCpltCallback(hi2c);
#else
        HAL_I2C_MasterTxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }
    /* hi2c->State == HAL_I2C_STATE_BUSY_RX */
    else
    {
        hi2c->State         = HAL_I2C_STATE_READY;
        hi2c->PreviousState = I2C_STATE_MASTER_BUSY_RX;
        hi2c->XferISR       = NULL;

        /* Disable Interrupts */
        I2C_Disable_IRQ(hi2c, I2C_XFER_RX_IT);

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->MasterRxCpltCallback(hi2c);
#else
        HAL_I2C_MasterRxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }
#endif
}

#endif


/**
  * @brief  I2C Master complete process.
  * @param  hi2c I2C handle.
  * @param  ITFlags Interrupt flags to handle.
  * @retval None
  */
static void I2C_ITMasterCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags)
{
    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);


    /* Disable Interrupts */
    hi2c->Instance->IER = 0;
    hi2c->Instance->SR = hi2c->Instance->SR;


    /* Reset handle parameters */
    hi2c->PreviousState = I2C_STATE_NONE;
    hi2c->XferISR       = NULL;
    hi2c->XferOptions   = I2C_NO_OPTION_FRAME;

    if (I2C_CHECK_FLAG(ITFlags, I2C_FLAG_AF) != RESET)
    {
        /* Clear NACK Flag */
        HAL_I2C_LOG("I2C_ITMasterCplt detect NACK.");

        /* Set acknowledge error code */
        hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
    }


    /* Call the corresponding callback to inform upper layer of End of Transfer */
    if (hi2c->State == HAL_I2C_STATE_ABORT)
    {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        I2C_ITError(hi2c, hi2c->ErrorCode);
    }
    else if (hi2c->State == HAL_I2C_STATE_BUSY_TX)
    {
        HAL_I2C_LOG("I2C_Master TX Cplt callback.");
        hi2c->State = HAL_I2C_STATE_READY;

        if (hi2c->Mode == HAL_I2C_MODE_MEM)
        {
            hi2c->Mode = HAL_I2C_MODE_NONE;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            /* Call the corresponding callback to inform upper layer of End of Transfer */
            HAL_I2C_MemTxCpltCallback(hi2c);
        }
        else
        {
            hi2c->Mode = HAL_I2C_MODE_NONE;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            /* Call the corresponding callback to inform upper layer of End of Transfer */
            HAL_I2C_MasterTxCpltCallback(hi2c);
        }
    }
    else if (hi2c->State == HAL_I2C_STATE_BUSY_RX)
    {
        HAL_I2C_LOG("I2C_Master RX Cplt callback.");
        hi2c->State = HAL_I2C_STATE_READY;

        if (hi2c->Mode == HAL_I2C_MODE_MEM)
        {
            hi2c->Mode = HAL_I2C_MODE_NONE;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            /* Call the corresponding callback to inform upper layer of End of Transfer */
            HAL_I2C_MemRxCpltCallback(hi2c);
        }
        else
        {
            hi2c->Mode = HAL_I2C_MODE_NONE;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            /* Call the corresponding callback to inform upper layer of End of Transfer */
            HAL_I2C_MasterRxCpltCallback(hi2c);
        }
    }
    else
    {
        /* Nothing to do */
    }
}


#if HAL_I2CSLAVE_MODULE_ENABLED
/**
  * @brief  I2C Slave complete process.
  * @param  hi2c I2C handle.
  * @param  ITFlags Interrupt flags to handle.
  * @retval None
  */
static void I2C_ITSlaveCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags)
{
    uint32_t tmpcr1value = READ_REG(hi2c->Instance->CR1);

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

    /* Disable all interrupts */
    I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_TX_IT | I2C_XFER_RX_IT);

    /* Disable Address Acknowledge */
    hi2c->Instance->CR2 |= I2C_CR2_NACK;

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    /* Flush TX register */
    I2C_Flush_TXDR(hi2c);

    /* If a DMA is ongoing, Update handle size context */
    if (I2C_CHECK_IT_SOURCE(tmpcr1value, I2C_CR1_TXDMAEN) != RESET)
    {
        if (hi2c->hdmatx != NULL)
        {
            hi2c->XferCount = (uint16_t)__HAL_DMA_GET_COUNTER(hi2c->hdmatx);
        }
    }
    else if (I2C_CHECK_IT_SOURCE(tmpcr1value, I2C_CR1_RXDMAEN) != RESET)
    {
        if (hi2c->hdmarx != NULL)
        {
            hi2c->XferCount = (uint16_t)__HAL_DMA_GET_COUNTER(hi2c->hdmarx);
        }
    }
    else
    {
        /* Do nothing */
    }

    /* Store Last receive data if any */
    if (I2C_CHECK_FLAG(ITFlags, I2C_FLAG_RXNE) != RESET)
    {
        /* Read data from RXDR */
        *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        if ((hi2c->XferSize > 0U))
        {
            hi2c->XferSize--;
            hi2c->XferCount--;
        }
    }

    /* All data are not transferred, so set error code accordingly */
    if (hi2c->XferCount != 0U)
    {
        /* Set ErrorCode corresponding to a Non-Acknowledge */
        hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
    }

    hi2c->PreviousState = I2C_STATE_NONE;
    hi2c->Mode = HAL_I2C_MODE_NONE;
    hi2c->XferISR = NULL;

    if (hi2c->ErrorCode != HAL_I2C_ERROR_NONE)
    {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        I2C_ITError(hi2c, hi2c->ErrorCode);

        /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
        if (hi2c->State == HAL_I2C_STATE_LISTEN)
        {
            /* Call I2C Listen complete process */
            I2C_ITListenCplt(hi2c, ITFlags);
        }
    }
    else if (hi2c->XferOptions != I2C_NO_OPTION_FRAME)
    {
        hi2c->XferOptions = I2C_NO_OPTION_FRAME;
        hi2c->State = HAL_I2C_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->ListenCpltCallback(hi2c);
#else
        HAL_I2C_ListenCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }
    /* Call the corresponding callback to inform upper layer of End of Transfer */
    else if (hi2c->State == HAL_I2C_STATE_BUSY_RX)
    {
        hi2c->State = HAL_I2C_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->SlaveRxCpltCallback(hi2c);
#else
        HAL_I2C_SlaveRxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }
    else
    {
        hi2c->State = HAL_I2C_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Call the corresponding callback to inform upper layer of End of Transfer */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
        hi2c->SlaveTxCpltCallback(hi2c);
#else
        HAL_I2C_SlaveTxCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    }
}

/**
  * @brief  I2C Listen complete process.
  * @param  hi2c I2C handle.
  * @param  ITFlags Interrupt flags to handle.
  * @retval None
  */
static void I2C_ITListenCplt(I2C_HandleTypeDef *hi2c, uint32_t ITFlags)
{
    /* Reset handle parameters */
    hi2c->XferOptions = I2C_NO_OPTION_FRAME;
    hi2c->PreviousState = I2C_STATE_NONE;
    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode = HAL_I2C_MODE_NONE;
    hi2c->XferISR = NULL;

    /* Store Last receive data if any */
    if (I2C_CHECK_FLAG(ITFlags, I2C_FLAG_RXNE) != RESET)
    {
        /* Read data from RXDR */
        *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

        /* Increment Buffer pointer */
        hi2c->pBuffPtr++;

        if ((hi2c->XferSize > 0U))
        {
            hi2c->XferSize--;
            hi2c->XferCount--;

            /* Set ErrorCode corresponding to a Non-Acknowledge */
            hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
        }
    }

    /* Disable all Interrupts*/
    I2C_Disable_IRQ(hi2c, I2C_XFER_LISTEN_IT | I2C_XFER_RX_IT | I2C_XFER_TX_IT);

    /* Clear NACK Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    hi2c->ListenCpltCallback(hi2c);
#else
    HAL_I2C_ListenCpltCallback(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
}
#endif

/**
  * @brief  I2C interrupts error process.
  * @param  hi2c I2C handle.
  * @param  ErrorCode Error code to handle.
  * @retval None
  */
static void I2C_ITError(I2C_HandleTypeDef *hi2c, uint32_t ErrorCode)
{
    HAL_I2C_LOG("I2C_ITError %x.", ErrorCode);

    /* Disable Interrupts */
    hi2c->Instance->IER = 0;
    hi2c->XferISR       = NULL;
    hi2c->Instance->SR = hi2c->Instance->SR;//Clear irq

    /* Set acknowledge error code */
    hi2c->ErrorCode |= ErrorCode;
    if (hi2c->State != HAL_I2C_STATE_ABORT)
    {
        hi2c->State = HAL_I2C_STATE_READY;
    }


    /* Abort DMA TX transfer if any */
    if (hi2c->Instance->CR & I2C_CR_DMAEN)
    {
        if (HAL_I2C_STATE_BUSY_TX == hi2c->State)
        {
            hi2c->Instance->TCR &= ~I2C_TCR_TXREQ;
            if (hi2c->hdmatx != NULL)
            {
                /* Set the I2C DMA Abort callback :
                 will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
                hi2c->hdmatx->XferAbortCallback = I2C_DMAAbort;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);

                /* Abort DMA TX */
                if (HAL_DMA_Abort_IT(hi2c->hdmatx) != HAL_OK)
                {
                    /* Call Directly XferAbortCallback function in case of error */
                    hi2c->hdmatx->XferAbortCallback(hi2c->hdmatx);
                }
            }
        }
        /* Abort DMA RX transfer if any */
        else if (HAL_I2C_STATE_BUSY_RX == hi2c->State)
        {
            hi2c->Instance->TCR &= ~I2C_TCR_RXREQ;
            if (hi2c->hdmarx != NULL)
            {
                /* Set the I2C DMA Abort callback :
                  will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
                hi2c->hdmarx->XferAbortCallback = I2C_DMAAbort;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);

                /* Abort DMA RX */
                if (HAL_DMA_Abort_IT(hi2c->hdmarx) != HAL_OK)
                {
                    /* Call Directly hi2c->hdmarx->XferAbortCallback function in case of error */
                    hi2c->hdmarx->XferAbortCallback(hi2c->hdmarx);
                }
            }
        }

    }
    else if (hi2c->State == HAL_I2C_STATE_ABORT)
    {
        hi2c->State = HAL_I2C_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Call the corresponding callback to inform upper layer of End of Transfer */
        HAL_I2C_AbortCpltCallback(hi2c);
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        /* Call the corresponding callback to inform upper layer of End of Transfer */

        HAL_I2C_ErrorCallback(hi2c);
    }
}

#if 0
/**
  * @brief  I2C Tx data register flush process.
  * @param  hi2c I2C handle.
  * @retval None
  */
static void I2C_Flush_TXDR(I2C_HandleTypeDef *hi2c)
{
    /* If a pending TXIS flag is set */
    /* Write a dummy data in TXDR to clear it */
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS) != RESET)
    {
        hi2c->Instance->TXDR = 0x00U;
    }

    /* Flush TX register if not empty */
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXE) == RESET)
    {
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_TXE);
    }
}
#endif

/**
  * @brief  DMA I2C master transmit process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void I2C_DMAMasterTransmitCplt(DMA_HandleTypeDef *hdma)
{
    //I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */

    //HAL_I2C_LOG("I2C_DMAMasterTransmitCplt cnt=%d, state=%d,SR=%x, DNR=%x",
    //            hi2c->XferCount, hi2c->State, hi2c->Instance->SR, hi2c->Instance->DNR);

}

#if HAL_I2CSLAVE_MODULE_ENABLED
/**
  * @brief  DMA I2C slave transmit process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void I2C_DMASlaveTransmitCplt(DMA_HandleTypeDef *hdma)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
    uint32_t tmpoptions = hi2c->XferOptions;

    if ((tmpoptions == I2C_NEXT_FRAME) || (tmpoptions == I2C_FIRST_FRAME))
    {
        /* Disable DMA Request */
        hi2c->Instance->CR1 &= ~I2C_CR1_TXDMAEN;

        /* Last Byte is Transmitted */
        /* Call I2C Slave Sequential complete process */
        I2C_ITSlaveSequentialCplt(hi2c);
    }
    else
    {
        /* No specific action, Master fully manage the generation of STOP condition */
        /* Mean that this generation can arrive at any time, at the end or during DMA process */
        /* So STOP condition should be manage through Interrupt treatment */
    }
}
#endif

/**
  * @brief DMA I2C master receive process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void I2C_DMAMasterReceiveCplt(DMA_HandleTypeDef *hdma)
{
//    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
//    HAL_I2C_LOG("I2C_DMAMasterReceiveCplt cnt=%d, state=%d,SR=%x, DNR=%x",
//                hi2c->XferCount, hi2c->State, hi2c->Instance->SR, hi2c->Instance->DNR);

}

#if HAL_I2CSLAVE_MODULE_ENABLED
/**
  * @brief  DMA I2C slave receive process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
static void I2C_DMASlaveReceiveCplt(DMA_HandleTypeDef *hdma)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
    uint32_t tmpoptions = hi2c->XferOptions;

    if ((__HAL_DMA_GET_COUNTER(hi2c->hdmarx) == 0U) && \
            (tmpoptions != I2C_NO_OPTION_FRAME))
    {
        /* Disable DMA Request */
        hi2c->Instance->CR1 &= ~I2C_CR1_RXDMAEN;

        /* Call I2C Slave Sequential complete process */
        I2C_ITSlaveSequentialCplt(hi2c);
    }
    else
    {
        /* No specific action, Master fully manage the generation of STOP condition */
        /* Mean that this generation can arrive at any time, at the end or during DMA process */
        /* So STOP condition should be manage through Interrupt treatment */
    }
}
#endif

/**
  * @brief  DMA I2C communication error callback.
  * @param hdma DMA handle
  * @retval None
  */
static void I2C_DMAError(DMA_HandleTypeDef *hdma)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */

    /* Disable Acknowledge */
    //hi2c->Instance->CR |= I2C_CR_NACK;

    /* Call the corresponding callback to inform upper layer of End of Transfer */
    I2C_ITError(hi2c, HAL_I2C_ERROR_DMA);
}

/**
  * @brief DMA I2C communication abort callback
  *        (To be called at end of DMA Abort procedure).
  * @param hdma DMA handle.
  * @retval None
  */
static void I2C_DMAAbort(DMA_HandleTypeDef *hdma)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */

    /* Reset AbortCpltCallback */
    hi2c->hdmatx->XferAbortCallback = NULL;
    hi2c->hdmarx->XferAbortCallback = NULL;

    /* Check if come from abort from user */
    if (hi2c->State == HAL_I2C_STATE_ABORT)
    {
        hi2c->State = HAL_I2C_STATE_READY;

        /* Call the corresponding callback to inform upper layer of End of Transfer */
        HAL_I2C_AbortCpltCallback(hi2c);
    }
    else
    {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        HAL_I2C_ErrorCallback(hi2c);
    }
}

static HAL_StatusTypeDef I2C_Master_DMAReload(struct __I2C_HandleTypeDef *hi2c)
{
    HAL_ASSERT(hi2c->XferSize > 0);
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_SR_DMADONE);


    if (HAL_I2C_STATE_BUSY_TX == hi2c->State)
    {
        if (0 == hi2c->XferCount) //The last transfer
        {
            /* Enable TC interrupts */
            I2C_Enable_IRQ(hi2c, I2C_IER_DMADONEIE | I2C_IT_STOPI | I2C_IER_BEDIE);
            hi2c->Instance->CR |= I2C_CR_DMAEN | I2C_CR_LASTSTOP | I2C_CR_MSDE;
        }
        else
        {
            /* Enable TC interrupts */
            I2C_Enable_IRQ(hi2c, I2C_IER_DMADONEIE | I2C_IER_BEDIE);
            hi2c->Instance->CR |= I2C_CR_DMAEN;
        }
        hi2c->Instance->DNR = hi2c->XferSize;
        hi2c->Instance->TCR = I2C_TCR_TXREQ;    /* Enable DMA Request */
    }
    else if (HAL_I2C_STATE_BUSY_RX == hi2c->State)
    {
        if (0 == hi2c->XferCount) //The last transfer
        {
            /* Enable TC interrupts */
            I2C_Enable_IRQ(hi2c, I2C_IER_DMADONEIE | I2C_IT_STOPI | I2C_IER_BEDIE);
            hi2c->Instance->CR |= I2C_CR_DMAEN | I2C_CR_LASTNACK | I2C_CR_LASTSTOP | I2C_CR_MSDE;
        }
        else
        {
            /* Enable TC interrupts */
            I2C_Enable_IRQ(hi2c, I2C_IER_DMADONEIE | I2C_IER_BEDIE);
            hi2c->Instance->CR |= I2C_CR_DMAEN;
        }
        hi2c->Instance->DNR = hi2c->XferSize;
        hi2c->Instance->TCR = I2C_TCR_RXREQ;    /* Enable DMA Request */
    }
    else
    {
        HAL_ASSERT(0);
        return HAL_ERROR;
    }

    return HAL_OK;
}


/**
  * @brief  This function handles I2C Communication Timeout.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Flag Specifies the I2C flag to check.
  * @param  Status The new Flag status (SET or RESET).
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart)
{
    while (__HAL_I2C_GET_FLAG(hi2c, Flag) == Status)
    {
        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
            {
                if (__HAL_I2C_GET_FLAG(hi2c, Flag) == Status) //Return timeout if got status again.
                {
                    hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
                    hi2c->State = HAL_I2C_STATE_READY;
                    hi2c->Mode = HAL_I2C_MODE_NONE;

                    return HAL_ERROR;
                }
            }
        }
    }

#if 0
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_SR_RXACK) == SET)
    {
        hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
        hi2c->State = HAL_I2C_STATE_READY;
        hi2c->Mode = HAL_I2C_MODE_NONE;

        __HAL_UNLOCK(hi2c);
        return HAL_ERROR;
    }
#endif
    return HAL_OK;
}


/**
  * @brief  This function waitting Flag util timeout, but exit immediately if detect Error Flag
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Flag Specifies the I2C flag to check.
  * @param  Status The new Flag status (SET or RESET).
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_WaitOnFlagAndDetectError(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t ErrFlags, uint32_t Timeout, uint32_t Tickstart)
{
    while (__HAL_I2C_GET_FLAG(hi2c, Flag) == Status)
    {
        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
            {
                if (__HAL_I2C_GET_FLAG(hi2c, Flag) == Status) //Return timeout if got status again.
                {
                    hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;

                    return HAL_ERROR;
                }
            }
        }
        /*
                if(ErrFlags&I2C_SR_NACK)
                {
                    if (__HAL_I2C_GET_FLAG(hi2c, I2C_SR_NACK) != RESET)
                    {
                        hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
                        return HAL_ERROR;
                    }
                }
        */
        if (ErrFlags & I2C_SR_EBB)
        {
            if (__HAL_I2C_GET_FLAG(hi2c, I2C_SR_EBB) != RESET)
            {
                hi2c->ErrorCode |= HAL_I2C_ERROR_BERR;
                return HAL_ERROR;
            }
        }

        if (ErrFlags & I2C_SR_BED)
        {
            if (__HAL_I2C_GET_FLAG(hi2c, I2C_SR_BED) != RESET)
            {
                hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
                return HAL_ERROR;
            }
        }
    }


    return HAL_OK;
}


#if 0

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of TXIS flag.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_WaitOnTXISFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
    while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS) == SET)
    {
        /* Check if a NACK is detected */
        //if (I2C_IsAcknowledgeFailed(hi2c, Timeout, Tickstart) != HAL_OK)
        //{
        //    return HAL_ERROR;
        //}

        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
            {
                hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
                hi2c->State = HAL_I2C_STATE_READY;
                hi2c->Mode = HAL_I2C_MODE_NONE;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);

                return HAL_ERROR;
            }
        }
    }

    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF) == SET)
    {
        hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
        hi2c->State = HAL_I2C_STATE_READY;
        hi2c->Mode = HAL_I2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
    }

    return HAL_OK;
}


/**
  * @brief  This function handles I2C Communication Timeout for specific usage of STOP flag.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_WaitOnSTOPFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
    while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF) == RESET)
    {
        /* Check if a NACK is detected */
        if (I2C_IsAcknowledgeFailed(hi2c, Timeout, Tickstart) != HAL_OK)
        {
            return HAL_ERROR;
        }

        /* Check for the Timeout */
        if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
        {
            hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
            hi2c->State = HAL_I2C_STATE_READY;
            hi2c->Mode = HAL_I2C_MODE_NONE;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of RXNE flag.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_WaitOnRXNEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
    while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) == RESET)
    {
        /* Check if a NACK is detected */
        if (I2C_IsAcknowledgeFailed(hi2c, Timeout, Tickstart) != HAL_OK)
        {
            return HAL_ERROR;
        }

        /* Check if a STOPF is detected */
        if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF) == SET)
        {
            /* Check if an RXNE is pending */
            /* Store Last receive data if any */
            if ((__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) == SET) && (hi2c->XferSize > 0U))
            {
                /* Return HAL_OK */
                /* The Reading of data from RXDR will be done in caller function */
                return HAL_OK;
            }
            else
            {
                /* Clear STOP Flag */
                __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

                /* Clear Configuration Register 2 */
                I2C_RESET_CR2(hi2c);

                hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
                hi2c->State = HAL_I2C_STATE_READY;
                hi2c->Mode = HAL_I2C_MODE_NONE;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2c);

                return HAL_ERROR;
            }
        }

        /* Check for the Timeout */
        if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
        {
            hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
            hi2c->State = HAL_I2C_STATE_READY;

            /* Process Unlocked */
            __HAL_UNLOCK(hi2c);

            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

#endif


#if 0
/**
  * @brief  This function handles Acknowledge failed detection during an I2C Communication.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_IsAcknowledgeFailed(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF) == SET)
    {
        /* Wait until STOP Flag is reset */
        /* AutoEnd should be initiate after AF */
        while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF) == RESET)
        {
            /* Check for the Timeout */
            if (Timeout != HAL_MAX_DELAY)
            {
                if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
                {
                    hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
                    hi2c->State = HAL_I2C_STATE_READY;
                    hi2c->Mode = HAL_I2C_MODE_NONE;

                    /* Process Unlocked */
                    __HAL_UNLOCK(hi2c);

                    return HAL_ERROR;
                }
            }
        }

        /* Clear NACKF Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

        /* Clear STOP Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

        /* Flush TX register */
        I2C_Flush_TXDR(hi2c);

        /* Clear Configuration Register 2 */
        I2C_RESET_CR2(hi2c);

        hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
        hi2c->State = HAL_I2C_STATE_READY;
        hi2c->Mode = HAL_I2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
    }
    return HAL_OK;
}
#endif

#if 0
/**
  * @brief  Handles I2Cx communication when starting transfer or during transfer (TC or TCR flag are set).
  * @param  hi2c I2C handle.
  * @param  DevAddress Specifies the slave address to be programmed.
  * @param  Size Specifies the number of bytes to be programmed.
  *   This parameter must be a value between 0 and 255.
  * @param  Mode New state of the I2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg @ref I2C_RELOAD_MODE Enable Reload mode .
  *     @arg @ref I2C_AUTOEND_MODE Enable Automatic end mode.
  *     @arg @ref I2C_SOFTEND_MODE Enable Software end mode.
  * @param  Request New state of the I2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg @ref I2C_NO_STARTSTOP Don't Generate stop and start condition.
  *     @arg @ref I2C_GENERATE_STOP Generate stop condition (Size should be set to 0).
  *     @arg @ref I2C_GENERATE_START_READ Generate Restart for read request.
  *     @arg @ref I2C_GENERATE_START_WRITE Generate Restart for write request.
  * @retval None
  */
static void I2C_TransferConfig(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t Size, uint32_t Mode, uint32_t Request)
{
    /* Check the parameters */
    HAL_ASSERT(IS_I2C_ALL_INSTANCE(hi2c->Instance));
    HAL_ASSERT(IS_TRANSFER_MODE(Mode));
    HAL_ASSERT(IS_TRANSFER_REQUEST(Request));

    /* update CR2 register */
    if (I2C_GENERATE_START_WRITE == Request)
    {
        hi2c->Instance->DBR = (DevAddress << 1);
        hi2c->Instance->TCR = (I2C_TCR_START | I2C_TCR_TB);
    }
    else if (I2C_GENERATE_START_READ == Request)
    {
        hi2c->Instance->DBR = (DevAddress << 1) | 1;
        hi2c->Instance->TCR = (I2C_TCR_START | I2C_TCR_TB);
    }
    else if (I2C_NO_STARTSTOP == Request)
    {
        ;
    }
}
#endif


/**
  * @brief  Manage the enabling of Interrupts.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  InterruptRequest Value of @ref I2C_Interrupt_configuration_definition.
  * @retval None
  */
static void I2C_Enable_IRQ(I2C_HandleTypeDef *hi2c, uint16_t InterruptRequest)
{
    hi2c->Instance->IER |= InterruptRequest;

#if 0
    uint32_t tmpisr = 0U;

    if ((hi2c->XferISR == I2C_Master_ISR_DMA) || \
            (hi2c->XferISR == I2C_Slave_ISR_DMA))
    {
        if ((InterruptRequest & I2C_XFER_LISTEN_IT) == I2C_XFER_LISTEN_IT)
        {
            /* Enable ERR, STOP, NACK and ADDR interrupts */
            tmpisr |= I2C_IT_ADDRI | I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ERRI;
        }

        if ((InterruptRequest & I2C_XFER_ERROR_IT) == I2C_XFER_ERROR_IT)
        {
            /* Enable ERR and NACK interrupts */
            tmpisr |= I2C_IT_ERRI | I2C_IT_NACKI;
        }

        if ((InterruptRequest & I2C_XFER_CPLT_IT) == I2C_XFER_CPLT_IT)
        {
            /* Enable STOP interrupts */
            tmpisr |= I2C_IT_STOPI;
        }

        if ((InterruptRequest & I2C_XFER_RELOAD_IT) == I2C_XFER_RELOAD_IT)
        {
            /* Enable TC interrupts */
            tmpisr |= I2C_IT_TCI;
        }
    }
    else
    {
        if ((InterruptRequest & I2C_XFER_LISTEN_IT) == I2C_XFER_LISTEN_IT)
        {
            /* Enable ERR, STOP, NACK, and ADDR interrupts */
            tmpisr |= I2C_IT_ADDRI | I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ERRI;
        }

        if ((InterruptRequest & I2C_XFER_TX_IT) == I2C_XFER_TX_IT)
        {
            /* Enable ERR, TC, STOP, NACK and RXI interrupts */
            tmpisr |= I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_TXI;
        }

        if ((InterruptRequest & I2C_XFER_RX_IT) == I2C_XFER_RX_IT)
        {
            /* Enable ERR, TC, STOP, NACK and TXI interrupts */
            tmpisr |= I2C_IT_ERRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_RXI;
        }

        if ((InterruptRequest & I2C_XFER_CPLT_IT) == I2C_XFER_CPLT_IT)
        {
            /* Enable STOP interrupts */
            tmpisr |= I2C_IT_STOPI;
        }
    }

    /* Enable interrupts only at the end */
    /* to avoid the risk of I2C interrupt handle execution before */
    /* all interrupts requested done */
    __HAL_I2C_ENABLE_IT(hi2c, tmpisr);
#endif
}

/**
  * @brief  Manage the disabling of Interrupts.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  InterruptRequest Value of @ref I2C_Interrupt_configuration_definition.
  * @retval None
  */
static void I2C_Disable_IRQ(I2C_HandleTypeDef *hi2c, uint16_t InterruptRequest)
{

    hi2c->Instance->IER &= (~(InterruptRequest));

#if 0
    uint32_t tmpisr = 0U;

    if ((InterruptRequest & I2C_XFER_TX_IT) == I2C_XFER_TX_IT)
    {
        /* Disable TC and TXI interrupts */
        tmpisr |= I2C_IT_TCI | I2C_IT_TXI;

        if (((uint32_t)hi2c->State & (uint32_t)HAL_I2C_STATE_LISTEN) != (uint32_t)HAL_I2C_STATE_LISTEN)
        {
            /* Disable NACK and STOP interrupts */
            tmpisr |= I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ERRI;
        }
    }

    if ((InterruptRequest & I2C_XFER_RX_IT) == I2C_XFER_RX_IT)
    {
        /* Disable TC and RXI interrupts */
        tmpisr |= I2C_IT_TCI | I2C_IT_RXI;

        if (((uint32_t)hi2c->State & (uint32_t)HAL_I2C_STATE_LISTEN) != (uint32_t)HAL_I2C_STATE_LISTEN)
        {
            /* Disable NACK and STOP interrupts */
            tmpisr |= I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ERRI;
        }
    }

    if ((InterruptRequest & I2C_XFER_LISTEN_IT) == I2C_XFER_LISTEN_IT)
    {
        /* Disable ADDR, NACK and STOP interrupts */
        tmpisr |= I2C_IT_ADDRI | I2C_IT_STOPI | I2C_IT_NACKI | I2C_IT_ERRI;
    }

    if ((InterruptRequest & I2C_XFER_ERROR_IT) == I2C_XFER_ERROR_IT)
    {
        /* Enable ERR and NACK interrupts */
        tmpisr |= I2C_IT_ERRI | I2C_IT_NACKI;
    }

    if ((InterruptRequest & I2C_XFER_CPLT_IT) == I2C_XFER_CPLT_IT)
    {
        /* Enable STOP interrupts */
        tmpisr |= I2C_IT_STOPI;
    }

    if ((InterruptRequest & I2C_XFER_RELOAD_IT) == I2C_XFER_RELOAD_IT)
    {
        /* Enable TC interrupts */
        tmpisr |= I2C_IT_TCI;
    }

    /* Disable interrupts only at the end */
    /* to avoid a breaking situation like at "t" time */
    /* all disable interrupts request are not done */
    __HAL_I2C_DISABLE_IT(hi2c, tmpisr);
#endif
}

#if 0

/**
  * @brief  Convert I2Cx OTHER_xxx XferOptions to functionnal XferOptions.
  * @param  hi2c I2C handle.
  * @retval None
  */
static void I2C_ConvertOtherXferOptions(I2C_HandleTypeDef *hi2c)
{
    /* if user set XferOptions to I2C_OTHER_FRAME            */
    /* it request implicitly to generate a restart condition */
    /* set XferOptions to I2C_FIRST_FRAME                    */
    if (hi2c->XferOptions == I2C_OTHER_FRAME)
    {
        hi2c->XferOptions = I2C_FIRST_FRAME;
    }
    /* else if user set XferOptions to I2C_OTHER_AND_LAST_FRAME */
    /* it request implicitly to generate a restart condition    */
    /* then generate a stop condition at the end of transfer    */
    /* set XferOptions to I2C_FIRST_AND_LAST_FRAME              */
    else if (hi2c->XferOptions == I2C_OTHER_AND_LAST_FRAME)
    {
        hi2c->XferOptions = I2C_FIRST_AND_LAST_FRAME;
    }
    else
    {
        /* Nothing to do */
    }
}


#endif

/**
  * @}
  */

#endif /* HAL_I2C_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
