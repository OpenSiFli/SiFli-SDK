/**
  ******************************************************************************
  * @file   button.h
  * @author Sifli software development team
  * @brief Sifli button library interface
  * @{
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

#ifndef BUTTON_H
#define BUTTON_H
#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
#include "sf_type.h"
#include "drivers/pin.h"
#ifdef BUTTON_SERVICE_ENABLED
    #include "data_service.h"
#endif


/**
 ****************************************************************************************
* @addtogroup button Button Library
* @ingroup middleware
* @brief Sifli button library interface
* @{
****************************************************************************************
*/


#ifdef __cplusplus
extern "C" {
#endif


/** button action type  */
typedef enum
{
    BUTTON_PRESSED  = 0,   /**< Indicates that a button is pressed */
    BUTTON_RELEASED = 1,   /**< Indicates that a button is released */
    BUTTON_LONG_PRESSED = 2,/**< Indicates that a button is long released */
    BUTTON_CLICKED  = 3,    /**< Indicates that a button is clicked */
} button_action_t;

/** button active level */
typedef enum
{
    BUTTON_ACTIVE_LOW  = 0,  /**< state is low when button is pressed */
    BUTTON_ACTIVE_HIGH = 1,  /**< state is high when button is pressed */
} button_active_state_t;


/** Button event handler type. */
typedef void (*button_handler_t)(int32_t pin, button_action_t button_action);

/** Button configuration structure. */
typedef struct
{
    int32_t                pin;              /**< Pin to be used as a button.
                                                  Its range is same as pin argument of rt pin device*/
    button_active_state_t  active_state;     /**< pin state when button is pressed */
    uint32_t               mode;             /**< Pull-up or -down mode,
                                                  such as PIN_MODE_INPUT, PIN_MODE_INPUT_PULLUP
                                                  or PIN_MODE_INPUT_PULLDOWN */
    button_handler_t       button_handler;   /**< Handler to be called when button is pushed. */
    uint16_t               debounce_time;    /**< debounce time in millisecond  */
} button_cfg_t;

#ifdef BUTTON_SERVICE_ENABLED
enum
{
    MSG_SERVICE_BUTTON_IRQ_TRIGGER_REQ             = (MSG_SERVICE_CUSTOM_ID_BEGIN),
    MSG_SERVICE_BUTTON_IRQ_TRIGGER_RSP             = MSG_SERVICE_BUTTON_IRQ_TRIGGER_REQ | RSP_MSG_TYPE,
};
#endif


/** Initialize the button.
 *
 * @param[in]  cfg  button config
 *
 * @return button id. >=0 on success, otherwise an error code.
 */
int32_t button_init(button_cfg_t *cfg);

/** Enable specified button detection.
 *
 * @param[in]  id  button id allocated by button_init
 *
 * @retval SF_EOK button successfully enabled.
 */
sf_err_t button_enable(int32_t id);

/** Disable specified button detection.
 *
 * @param[in]  id  button id allocated by button_init
 *
 * @retval  SF_EOK Button detection successfully disabled. Error code otherwise.
 */
sf_err_t button_disable(int32_t id);

/** Check if specified button is currently being pressed.
 *
 * @param[in]  id button id allocated by button_init
 *
 * @return true: pressed, false: not pressed
 */
bool button_is_pressed(int32_t id);

/** Emulate the pin IRQ
 *
 * @param[in]  pin pin
 *
 * @return void
 */
void button_irq_trigger(int32_t pin);

/** Update button handler
 *
 * @param[in] id button id allocated by button_init
 * @param[in] new_handler new button handler
 *
 * @return SF_EOK handler update succesfully
 */
sf_err_t button_update_handler(int32_t id, button_handler_t new_handler);


#ifdef USING_ADC_BUTTON
typedef void (*adc_button_handler_t)(uint8_t group_idx, int32_t pin, button_action_t button_action);

int32_t button_bind_adc_button(int32_t id, int8_t adc_button_group_id, uint8_t handler_num,
                               adc_button_handler_t *handler);
#endif /* USING_ADC_BUTTON */

/// @}  file

#ifdef __cplusplus
}
#endif




/// @} button
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
