/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INTENT_H__
#define __INTENT_H__



#ifdef __cplusplus
extern "C" {
#endif

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
 * @brief Reinitialize an intent with a new action
 * @param i - Intent
 * @param action - action name
 * @return pointer to intent
 */
intent_t intent_reinit(intent_t i, const char *action);

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

#ifdef __cplusplus
}
#endif
#endif  /* __INTENT_H__ */


