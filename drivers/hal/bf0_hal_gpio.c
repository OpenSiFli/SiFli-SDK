/**
  ******************************************************************************
  * @file   bf0_hal_gpio.c
  * @author Sifli software development team
  * @brief   GPIO HAL module driver.
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

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @defgroup GPIO GPIO
  * @brief GPIO HAL module driver
  * @{
  */
/** MISRA C:2012 deviation rule has been granted for following rules:
  * Rule-12.2 - Medium: RHS argument is in interval [0,INF] which is out of
  * range of the shift operator in following API :
  * HAL_GPIO_Init
  * HAL_GPIO_DeInit
  */

#if defined(HAL_GPIO_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

#ifdef SOC_BF0_HCPU
    #define IS_CURRENT_SYS_GPIO(hgpio) ((hgpio) == hwp_gpio1)
#else
    #define IS_CURRENT_SYS_GPIO(hgpio) ((hgpio) == hwp_gpio2)
#endif /* SOC_BF0_HCPU */




#ifndef SF32LB55X

/*
    Open Drain Output mode: (DOR=0,ITR=0,IPSR=1,IPCR=1)
        1)when OD output high,   DOER=0
        2)whne OD output low,    DOER=1
*/
#define IS_OPEN_DRAIN_FLAG(gpiox,mask)   ((0 == (gpiox->DOR & (mask)))&&   \
                                           (0 == (gpiox->ITR & (mask)))&&   \
                                           ((mask) == (gpiox->IPHR & (mask)))&&  \
                                           ((mask) == (gpiox->IPLR & (mask))))


#define CLEAR_OPEN_DRAIN_FLAG(gpiox,mask) do{ \
        gpiox->IPHCR = (mask); \
        gpiox->IPLCR = (mask); \
}while(0)

#define SET_OPEN_DRAIN_FLAG(gpiox,mask) do{ \
        gpiox->DOCR  = (mask);  \
        gpiox->ITCR  = (mask);  \
        gpiox->IPHSR = (mask);  \
        gpiox->IPLSR = (mask);  \
}while(0)

#else

/*
    Open Drain Output mode: (DOR=0,ITR=0,IPSR=1,IPCR=1)
        1)when OD output high,   DOER=0
        2)whne OD output low,    DOER=1
*/

#define IS_OPEN_DRAIN_FLAG(gpiox,mask)   ((0 == (gpiox->DOR & (mask)))&&   \
                                           (0 == (gpiox->ITR & (mask)))&&   \
                                           ((mask) == (gpiox->IPSR & (mask)))&&  \
                                           ((mask) == (gpiox->IPCR & (mask))))


#define CLEAR_OPEN_DRAIN_FLAG(gpiox,mask) do{ \
        gpiox->IPSR &= ~(mask); \
        gpiox->IPCR &= ~(mask); \
}while(0)


#define SET_OPEN_DRAIN_FLAG(gpiox,mask) do{ \
        gpiox->DOR  &= ~(mask); \
        gpiox->ITCR |= (mask);  \
        gpiox->IPSR |= (mask);  \
        gpiox->IPCR |= (mask);  \
}while(0)
#endif /* SF32LB55X */

#if defined(SF32LB55X)||defined(SF32LB58X)||defined(SF32LB56X)
    #define WAIT_ISR_DISABLED()  HAL_Delay_us(1)
#else
    #define WAIT_ISR_DISABLED()
#endif

#ifndef SF32LB55X
#define DISABLE_ISR(gpiox,mask)  do{(gpiox)->IECR |= (mask); \
                                        (gpiox)->IECR_EXT |= (mask); \
                                        WAIT_ISR_DISABLED(); /*Wait IER/IER_EXT deactive*/ \
                                        (gpiox)->ISR = (mask); \
                                        (gpiox)->ISR_EXT = (mask); \
                                        }while(0)
#else
#define DISABLE_ISR(gpiox,mask)  do{(gpiox)->IECR |= (mask); \
                                        WAIT_ISR_DISABLED();  /*Wait IER deactive*/\
                                         (gpiox)->ISR = (mask); \
                                        }while(0)
#endif /* SF32LB55X */


/*The actual avaliable gpios in each group*/
#ifdef SF32LB55X
    #define GPIO1_EXTI_START_PIN  (0)
    #define GPIO1_EXTI_END_PIN    (80)

    #define GPIO2_EXTI_START_PIN  (0)
    #define GPIO2_EXTI_END_PIN    (48)

#elif defined(SF32LB58X)
    #define GPIO1_EXTI_START_PIN  (0)
    #define GPIO1_EXTI_END_PIN    (93)

    #define GPIO2_EXTI_START_PIN  (0)
    #define GPIO2_EXTI_END_PIN    (59)
#elif defined(SF32LB56X)
    #define GPIO1_EXTI_START_PIN  (0)
    #define GPIO1_EXTI_END_PIN    (78)

    #define GPIO2_EXTI_START_PIN  (0)
    #define GPIO2_EXTI_END_PIN    (36)
#elif defined(SF32LB52X)
    // TODO: Revist this
    #define GPIO1_EXTI_START_PIN  (0)
    #define GPIO1_EXTI_END_PIN    (78)

    #define GPIO2_EXTI_START_PIN  (0)
    #define GPIO2_EXTI_END_PIN    (36)
#else
    #error "Fix me!"
#endif /* SF32LB55X */

/* Private typedef -----------------------------------------------------------*/

static GPIO_TypeDef *GPIO_GetInstance(GPIO_TypeDef *hgpio, uint16_t gpio_pin, uint16_t *offset);


/* Exported functions --------------------------------------------------------*/

/** @defgroup GPIO_Exported_Functions GPIO Exported Functions
  * @{
  */

/** @defgroup GPIO_Exported_Functions_Group1 Initialization/de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
 * @brief  Initialize the GPIO peripheral according to the specified parameters in the GPIO_Init.
 * @param  hgpio GPIO handle
 *         This parameter can be one of the following values:
 *            @arg @ref hwp_gpio1 Set hcpu pin register
 *            @arg @ref hwp_gpio2 Set lcpu pin register
 * @param  GPIO_Init: pointer to a GPIO_InitTypeDef structure that contains
 *         the configuration information for the specified GPIO peripheral.
 * @retval None
 */
__HAL_ROM_USED void HAL_GPIO_Init(GPIO_TypeDef *hgpio, GPIO_InitTypeDef *GPIO_Init)
{
    GPIO_TypeDef *gpiox;
    uint16_t offset;

    /* Check the parameters */
    HAL_ASSERT(hgpio);
    HAL_ASSERT(IS_GPIO_MODE(GPIO_Init->Mode));
    HAL_ASSERT(IS_GPIO_PULL(GPIO_Init->Pull));

    gpiox = GPIO_GetInstance(hgpio, GPIO_Init->Pin, &offset);
    if (0 == gpiox)
    {
        return;
    }

    /* In case of Output*/
    if (GPIO_Init->Mode == GPIO_MODE_OUTPUT)
    {
        DISABLE_ISR(gpiox, (1UL << offset));
        CLEAR_OPEN_DRAIN_FLAG(gpiox, (1UL << offset));
        gpiox->DOESR = (1UL << offset);
    }
    else if (GPIO_Init->Mode == GPIO_MODE_OUTPUT_OD)
    {
        DISABLE_ISR(gpiox, (1UL << offset));
        SET_OPEN_DRAIN_FLAG(gpiox, (1UL << offset));
    }
    else if (GPIO_Init->Mode == GPIO_MODE_INPUT)
    {
        DISABLE_ISR(gpiox, (1UL << offset));
        CLEAR_OPEN_DRAIN_FLAG(gpiox, (1UL << offset));
        gpiox->DOECR = (1UL << offset);
    }
    else
    {
        gpiox->DOECR = (1UL << offset); //Disable output

        if (GPIO_Init->Mode == GPIO_MODE_IT_RISING)
        {
            gpiox->ITSR |= (1UL << offset);
#ifndef SF32LB55X
            gpiox->IPHSR = (1UL << offset);
            gpiox->IPLCR = (1UL << offset);
#else
            gpiox->IPSR |= (1UL << offset);
            gpiox->IPCR &= ~(1UL << offset);

#endif /* SF32LB55X */
        }
        else if (GPIO_Init->Mode == GPIO_MODE_IT_FALLING)
        {
            gpiox->ITSR |= (1UL << offset);
#ifndef SF32LB55X
            gpiox->IPHCR = (1UL << offset);
            gpiox->IPLSR = (1UL << offset);
#else
            gpiox->IPSR &= ~(1UL << offset);
            gpiox->IPCR |= (1UL << offset);
#endif   /* SF32LB55X */
        }
        else if (GPIO_Init->Mode == GPIO_MODE_IT_RISING_FALLING)
        {
            gpiox->ITSR |= (1UL << offset);
#ifndef SF32LB55X
            gpiox->IPHSR = (1UL << offset);
            gpiox->IPLSR = (1UL << offset);
#else
            gpiox->IPSR |= (1UL << offset);
            gpiox->IPCR |= (1UL << offset);
#endif  /* SF32LB55X */
        }
        else if (GPIO_Init->Mode == GPIO_MODE_IT_HIGH_LEVEL)
        {
            gpiox->ITCR |= (1UL << offset);
#ifndef SF32LB55X
            gpiox->IPHSR = (1UL << offset);
            gpiox->IPLCR = (1UL << offset);
#else
            gpiox->IPSR |= (1UL << offset);
            gpiox->IPCR &= ~(1UL << offset);
#endif  /* SF32LB55X */
        }
        else if (GPIO_Init->Mode == GPIO_MODE_IT_LOW_LEVEL)
        {
            gpiox->ITCR |= (1UL << offset);
#ifndef SF32LB55X
            gpiox->IPHCR = (1UL << offset);
            gpiox->IPLSR = (1UL << offset);
#else
            gpiox->IPSR &= ~(1UL << offset);
            gpiox->IPCR |= (1UL << offset);
#endif  /* SF32LB55X */
        }

#ifndef SF32LB55X
        if (IS_CURRENT_SYS_GPIO(hgpio))
        {
            gpiox->IESR |= (1 << offset);
        }
        else
        {
            gpiox->IESR_EXT |= (1 << offset);
        }
#else
        gpiox->IESR |= (1 << offset);
#endif /* SF32LB55X */

    }
}

/**
 * @brief  De-initialize the GPIOx peripheral registers to their default reset values.
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin specifies the pin index to be written, starting from 0.
 * @retval None
 */
__HAL_ROM_USED void HAL_GPIO_DeInit(GPIO_TypeDef *hgpio, uint32_t GPIO_Pin)
{
    GPIO_TypeDef *gpiox;
    uint16_t offset;

    /* Check the parameters */
    HAL_ASSERT(hgpio);

    gpiox = GPIO_GetInstance(hgpio, GPIO_Pin, &offset);
    if (0 == gpiox)
    {
        return;
    }

    gpiox->DOECR |= (1UL << offset);
    DISABLE_ISR(gpiox, (1UL << offset));
    CLEAR_OPEN_DRAIN_FLAG(gpiox, (1UL << offset));
}

/**
  * @}
  */

/** @defgroup GPIO_Exported_Functions_Group2 IO operation functions
 *  @brief GPIO Read, Write, Toggle, Lock and EXTI management functions.
 *
@verbatim
 ===============================================================================
                       ##### IO operation functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
 * @brief  Read the specified input port pin.
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin specifies the pin index to be read, starting from 0.
 * @retval The input port pin value.
 */
__HAL_ROM_USED GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin)
{
    GPIO_PinState bitstatus;
    GPIO_TypeDef *gpiox;
    uint16_t offset;
    uint32_t val;

    /* Check the parameters */
    HAL_ASSERT(hgpio);

    gpiox = GPIO_GetInstance(hgpio, GPIO_Pin, &offset);
    if (0 == gpiox)
    {
        return GPIO_PIN_RESET;
    }

    if (IS_OPEN_DRAIN_FLAG(gpiox, (1UL << offset)))
    {
        //output open drain mode
        val = !(gpiox->DOER & (1UL << offset));
    }
    else if (gpiox->DOER & (1UL << offset))
    {
        // output pull push mode
        val = (gpiox->DOR & (1UL << offset));
    }
    else
    {
        // input
        val = (gpiox->DIR & (1UL << offset));
    }

    if (0 != val)
    {
        bitstatus = GPIO_PIN_SET;
    }
    else
    {
        bitstatus = GPIO_PIN_RESET;
    }

    return bitstatus;
}

/**
 * @brief  Set or clear the selected data port bit.
 *
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin specifies the pin index to be written, starting from 0.
 *
 * @param  PinState specifies the value to be written to the selected bit.
 *         This parameter can be one of the GPIO_PinState enum values:
 *            @arg GPIO_PIN_RESET: to clear the port pin
 *            @arg GPIO_PIN_SET: to set the port pin
 * @retval None
 */
__HAL_ROM_USED void HAL_GPIO_WritePin(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    GPIO_TypeDef *gpiox;
    uint16_t offset;

    /* Check the parameters */
    HAL_ASSERT(hgpio);
    HAL_ASSERT(IS_GPIO_PIN_ACTION(PinState));

    gpiox = GPIO_GetInstance(hgpio, GPIO_Pin, &offset);
    if (0 == gpiox)
    {
        return;
    }

    if (IS_OPEN_DRAIN_FLAG(gpiox, (1UL << offset)))
    {
        if (PinState == GPIO_PIN_RESET)
        {
            gpiox->DOESR |= (1UL << offset);
        }
        else
        {
            gpiox->DOECR |= (1UL << offset);
        }
    }
    else
    {
#ifndef SF32LB55X
        if (PinState != GPIO_PIN_RESET)
        {
            gpiox->DOSR = (1UL << offset);

        }
        else
        {
            gpiox->DOCR = (1UL << offset);
        }
#else
        /* Need support input pin write? */
        if (PinState != GPIO_PIN_RESET)
        {
            gpiox->DOR |= (1UL << offset);

        }
        else
        {
            gpiox->DOR &= ~(1UL << offset);
        }
#endif /* SF32LB55X */
    }
}

/**
 * @brief  Toggle the specified GPIO pin.
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin specifies the pin index to be toggoled, starting from 0.
 * @retval None
 */
__HAL_ROM_USED void HAL_GPIO_TogglePin(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin)
{
    GPIO_TypeDef *gpiox;
    uint16_t offset;

    /* Check the parameters */
    HAL_ASSERT(hgpio);

    gpiox = GPIO_GetInstance(hgpio, GPIO_Pin, &offset);
    if (0 == gpiox)
    {
        return;
    }

    if (IS_OPEN_DRAIN_FLAG(gpiox, (1UL << offset)))
    {
        if (gpiox->DOER & (1UL << offset))
        {
            gpiox->DOECR = (1UL << offset);
        }
        else
        {
            gpiox->DOESR = (1UL << offset);
        }
    }
    else
    {
#ifndef SF32LB55X
        if (gpiox->DOR & (1UL << offset))
        {
            gpiox->DOCR = (1UL << offset);
        }
        else
        {
            gpiox->DOSR = (1UL << offset);
        }
#else
        gpiox->DOR ^= (1UL << offset);
#endif /* SF32LB55X */

    }
}



/**
 * @brief  Handle EXTI interrupt request.
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line, starting from 0.
 * @retval None
 */
__HAL_ROM_USED void HAL_GPIO_EXTI_IRQHandler(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin)
{
    GPIO_TypeDef *gpiox;
    uint16_t offset;
    int8_t wakeup_pin;
    uint8_t detect_interrupt = 0;

    gpiox = GPIO_GetInstance(hgpio, GPIO_Pin, &offset);
    if (0 == gpiox)
    {
        return;
    }

#ifndef SF32LB55X
    if (!IS_CURRENT_SYS_GPIO(hgpio))
    {
        //Check EXT ISR only
        if ((gpiox->ISR_EXT & (1UL << offset)) != 0x00UL)
        {
            HAL_ASSERT((gpiox->IER_EXT & (1UL << offset)) != 0x00UL);
            gpiox->ISR_EXT = (1UL << offset);
            detect_interrupt = 1;
        }
    }
    else
#endif /* SF32LB55X */
    {
        //Check ISR
        if ((gpiox->ISR & (1UL << offset)) != 0x00UL)
        {
            HAL_ASSERT((gpiox->IER & (1UL << offset)) != 0x00UL);
            gpiox->ISR = (1UL << offset);
            detect_interrupt = 1;
        }
    }

    /* EXTI line interrupt detected */
    if (1 == detect_interrupt)
    {
        HAL_GPIO_EXTI_Callback(hgpio, GPIO_Pin);

        /* Clear pin WSR if it's used as wakeup pin.
         * AON pin detection always works even if it's not in sleep state.
         * So if edge detection is configured, the corresponding bit in WSR would still be set to 1 if system is awake, and no one will clear the bit.
         */
        if ((GPIO_TypeDef *)hwp_gpio1 == hgpio)
        {
            wakeup_pin = HAL_HPAON_QueryWakeupPin(hgpio, GPIO_Pin);
            if (wakeup_pin >= 0)
            {
                HAL_HPAON_CLEAR_WSR(1UL << (HPSYS_AON_WCR_PIN0_Pos + wakeup_pin));
            }
        }

        if ((GPIO_TypeDef *)hwp_gpio2 == hgpio)
        {
            wakeup_pin = HAL_LPAON_QueryWakeupPin(hgpio, GPIO_Pin);
            if (wakeup_pin >= 0)
            {
                HAL_LPAON_CLEAR_WSR(1UL << (LPSYS_AON_WCR_PIN0_Pos + wakeup_pin));

                HAL_PMU_CLEAR_WSR(1UL << (PMUC_WCR_PIN0_Pos + wakeup_pin));
            }
        }
    }
}

/**
 * @brief  Handle all GPIO interrupt request which belong current group.
 * @param  hgpio GPIO handle
 * @retval None
 */
__HAL_ROM_USED void HAL_GPIO_IRQHandler(GPIO_TypeDef *hgpio)
{
    uint32_t start, end;

    if (hwp_gpio1 == hgpio)
    {
        start = GPIO1_EXTI_START_PIN;
        end   = GPIO1_EXTI_END_PIN;
    }
    else if (hwp_gpio2 == hgpio)
    {
        start = GPIO2_EXTI_START_PIN;
        end   = GPIO2_EXTI_END_PIN;
    }
    else
    {
        HAL_ASSERT(0);//Invalid param
    }

    for (uint32_t i = start; i <= end; i++)
    {
        HAL_GPIO_EXTI_IRQHandler(hgpio, i);
    }

}

/**
 * @brief  EXTI line detection callback.
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line, starting from 0.
 * @retval None
 */
__weak void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin)
{
    /* Prevent unused argument(s) compilation warning */
#if 0  //for INT test
    extern void pin_irq_hdr(int irqno);
    pin_irq_hdr(GPIO_Pin);
#endif
    UNUSED(hgpio);
    UNUSED(GPIO_Pin);

    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_GPIO_EXTI_Callback could be implemented in the user file
     */
}

/**
 * @brief  Get the GPIO mode.
 * @param  hgpio GPIO handle
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line, starting from 0.
 * @retval mode in @ref GPIO_mode, return UINT32_MAX if invalid
 */
__HAL_ROM_USED uint32_t HAL_GPIO_GetMode(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin)
{
    GPIO_TypeDef *gpiox;
    uint16_t offset;
    uint32_t Mode;

    /* Check the parameters */
    HAL_ASSERT(hgpio);

    gpiox = GPIO_GetInstance(hgpio, GPIO_Pin, &offset);
    if (0 == gpiox)
    {
        return UINT32_MAX;
    }

    if (IS_OPEN_DRAIN_FLAG(gpiox, (1UL << offset)))
    {
        Mode = GPIO_MODE_OUTPUT_OD;
    }
    else if (gpiox->DOER & (1UL << offset))
    {
        Mode = GPIO_MODE_OUTPUT;
    }
    else if (gpiox->IER & (1UL << offset))
    {
        if (gpiox->ITR & (1UL << offset)) //Edge
        {
#ifndef SF32LB55X
            if ((gpiox->IPHR & (1UL << offset)) && (gpiox->IPLR & (1UL << offset)))
                Mode = GPIO_MODE_IT_RISING_FALLING;
            else if (gpiox->IPHR & (1UL << offset))
                Mode = GPIO_MODE_IT_RISING;
            else if (gpiox->IPLR & (1UL << offset))
                Mode = GPIO_MODE_IT_FALLING;
            else
                Mode = UINT32_MAX;
#else
            if ((gpiox->IPSR & (1UL << offset)) && (gpiox->IPCR & (1UL << offset)))
                Mode = GPIO_MODE_IT_RISING_FALLING;
            else if (gpiox->IPSR & (1UL << offset))
                Mode = GPIO_MODE_IT_RISING;
            else if (gpiox->IPCR & (1UL << offset))
                Mode = GPIO_MODE_IT_FALLING;
            else
                Mode = UINT32_MAX;
#endif /* SF32LB55Xs */
        }
        else
        {
#ifndef SF32LB55X
            if ((gpiox->IPHR & (1UL << offset)) && (gpiox->IPLR & (1UL << offset)))
                Mode = UINT32_MAX;
            else if (gpiox->IPHR & (1UL << offset))
                Mode = GPIO_MODE_IT_HIGH_LEVEL;
            else if (gpiox->IPLR & (1UL << offset))
                Mode = GPIO_MODE_IT_LOW_LEVEL;
            else
                Mode = UINT32_MAX;
#else
            if ((gpiox->IPSR & (1UL << offset)) && (gpiox->IPCR & (1UL << offset)))
                Mode = UINT32_MAX;
            else if (gpiox->IPSR & (1UL << offset))
                Mode = GPIO_MODE_IT_HIGH_LEVEL;
            else if (gpiox->IPCR & (1UL << offset))
                Mode = GPIO_MODE_IT_LOW_LEVEL;
            else
                Mode = UINT32_MAX;
#endif /* SF32LB55X */
        }
    }
    else
    {
        Mode = GPIO_MODE_INPUT;
    }

    return Mode;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_GPIO_Save(GPIO_TypeDef *hgpio, GPIOxRestore_TypeDef *p_buf, uint32_t buf_cnt)
{
    uint32_t i;
    GPIO_TypeDef *gpiox;

    if ((GPIO_TypeDef *)hwp_gpio1 == hgpio)
    {
        if (buf_cnt != (GPIO1_PIN_NUM / 32)) return HAL_ERROR;
    }
    else
    {
        if (buf_cnt != (GPIO2_PIN_NUM / 32)) return HAL_ERROR;
    }

    gpiox = (GPIO_TypeDef *)hgpio;

    for (i = 0; i < buf_cnt; i++)
    {
        p_buf->DOR  = gpiox->DOR;
        p_buf->DOER = gpiox->DOER;
        p_buf->IER  = gpiox->IER;
        p_buf->ITR  = gpiox->ITR;
#ifndef SF32LB55X
        p_buf->IPHR = gpiox->IPHR;
        p_buf->IPLR = gpiox->IPLR;
        p_buf->IER_EXT  = gpiox->IER_EXT;
#else
        p_buf->IPSR = gpiox->IPSR;
        p_buf->IPCR = gpiox->IPCR;
#endif /* SF32LB55X */

        p_buf++;
        gpiox++;
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_GPIO_Restore(GPIO_TypeDef *hgpio, GPIOxRestore_TypeDef *p_buf, uint32_t buf_cnt)
{
    uint32_t i;
    GPIO_TypeDef *gpiox;

    if ((GPIO_TypeDef *)hwp_gpio1 == hgpio)
    {
        if (buf_cnt != (GPIO1_PIN_NUM / 32)) return HAL_ERROR;
    }
    else
    {
        if (buf_cnt != (GPIO2_PIN_NUM / 32)) return HAL_ERROR;
    }

    gpiox = (GPIO_TypeDef *)hgpio;

    for (i = 0; i < buf_cnt; i++)
    {
        gpiox->DOR  = p_buf->DOR;
        gpiox->DOER = p_buf->DOER;
        gpiox->ITR  = p_buf->ITR;
#ifndef SF32LB55X
        gpiox->IPHSR = p_buf->IPHR;
        gpiox->IPLSR = p_buf->IPLR;
        gpiox->IER_EXT = p_buf->IER_EXT;
#else
        gpiox->IPSR = p_buf->IPSR;
        gpiox->IPCR = p_buf->IPCR;
#endif /* SF32LB55X */

        gpiox->IER  = p_buf->IER;

        p_buf++;
        gpiox++;
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_GPIO_ClearInterrupt(GPIO_TypeDef *hgpio)
{
    GPIO_TypeDef *gpiox;

    if ((hgpio != (GPIO_TypeDef *)hwp_gpio1) && (hgpio != (GPIO_TypeDef *)hwp_gpio2))
    {
        return HAL_ERROR;
    }

    gpiox = (GPIO_TypeDef *)hgpio;
    gpiox[0].ISR = GPIO1_ISR0_IS_Msk;
    gpiox[1].ISR = GPIO1_ISR0_IS_Msk;
    if (hwp_gpio1 == hgpio)
    {
        gpiox[2].ISR = GPIO1_ISR0_IS_Msk;
    }

    return HAL_OK;
}

__HAL_ROM_USED HAL_StatusTypeDef HAL_GPIO_ClearPinInterrupt(GPIO_TypeDef *hgpio, uint16_t GPIO_Pin)
{
    GPIO_TypeDef *gpiox;
    uint16_t offset;

    gpiox = GPIO_GetInstance(hgpio, GPIO_Pin, &offset);
    if (0 == gpiox)
    {
        return HAL_ERROR;
    }

#ifndef SF32LB55X
    if (!IS_CURRENT_SYS_GPIO(hgpio))
    {
        //Check EXT ISR only
        if ((gpiox->ISR_EXT & (1UL << offset)) != 0x00UL)
        {
            HAL_ASSERT((gpiox->IER_EXT & (1UL << offset)) != 0x00UL);
            gpiox->ISR_EXT = (1UL << offset);
        }
    }
    else
#endif /* SF32LB55X */
    {
        //Check ISR
        if ((gpiox->ISR & (1UL << offset)) != 0x00UL)
        {
            HAL_ASSERT((gpiox->IER & (1UL << offset)) != 0x00UL);
            gpiox->ISR = (1UL << offset);
        }
    }



    return HAL_OK;
}


/**
  * @}
  */


/**
  * @}
  */

static GPIO_TypeDef *GPIO_GetInstance(GPIO_TypeDef *hgpio, uint16_t gpio_pin, uint16_t *offset)
{
    uint16_t max_num;
    uint16_t inst_idx;
    GPIO_TypeDef *gpiox;

    if ((GPIO_TypeDef *)hwp_gpio1 == hgpio)
    {
        max_num = GPIO1_PIN_NUM;
    }
    else
    {
        max_num = GPIO2_PIN_NUM;
    }

    HAL_ASSERT(gpio_pin < max_num);

    if (gpio_pin >= max_num)
    {
        return (GPIO_TypeDef *)NULL;
    }

    inst_idx = gpio_pin >> 5;
    *offset = gpio_pin & 31;

    gpiox = (GPIO_TypeDef *)hgpio + inst_idx;

    return gpiox;
}

#endif /* HAL_GPIO_MODULE_ENABLED */


/**
  * @}
  */


/**
  * @}
  */



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
