/**
  ******************************************************************************
  * @file   intent.h
  * @author Sifli software development team
  * @brief  An IPC between app&app and app&service
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

#ifndef __INTENT_H__
#define __INTENT_H__

typedef void   *intent_t;


/**
 * @brief Initialize an intent
 * @param action - action name
 * @return pointer to intent
 */
intent_t intent_init(const char *action);

/**
 * @brief Destory intent
 * @param i - Intent
 */
void intent_deinit(intent_t i);

/**
 * @brief Put a string param into intent
 * @param i - Intent pointer
 * @param name - param name
 * @param value - param value
 * @return RT_EOK if no error occurred
 */
int intent_set_string(intent_t i, const char *name, const char *value);

/**
 * @brief Get a string param value form intent
 * @param i - Intent
 * @param name - parameter name
 * @return NULL if param not exist
 */
const char *intent_get_string(intent_t i, const char *name);


/**
 * @brief Put an unsigned int32 param into intent
 * @param i - Intent
 * @param name - parameter name
 * @param value - unsigned int32 value
 * @return
 */
int intent_set_uint32(intent_t i, const char *name, uint32_t value);


/**
 * @brief Get an unsigned int32 param from intent
 * @param i - Intent
 * @param name - Parameter name
 * @param err_value - error_value if param not found at intent
 * @return err_value if not found
 */
uint32_t intent_get_uint32(intent_t i, const char *name, uint32_t err_value);

/**
 * @brief Get action from intent
 * @param i - Intent
 * @return NULL if action not exist
 */
const char *intent_get_action(intent_t i);

/**
 * @brief Run an app by intent
 * @param i - intent
 * @return RT_EOK if no error occurred
 */
int intent_runapp(intent_t i);

/**
 * @brief Print the intent
 * @param i - intent
 * @return void
 */
void printf_intent(intent_t i);

#endif  /* __INTENT_H__ */



