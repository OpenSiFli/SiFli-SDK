/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_GPIO_EX_H
#define __LL_GPIO_EX_H

#include <stdint.h>
#include "register.h"
#include "ll_gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_gpio_ex.h
 * @brief Header-only low-level GPIO APIs for SF32LB52x SDK.
 */

/** @defgroup LL_GPIO_BANK LL GPIO Bank Access */
/** @{ */
/* UM defines GPIO1 bank1 base at GPIO1_BASE + 0x80. Do not derive this offset
 * from sizeof(ll_gpio_bank_t), because the register map contains a reserved gap
 * between bank0 and bank1. */
#define LL_GPIO_BANK_SIZE (0x80U)
#define LL_GPIO1_BANK0 ((ll_gpio_bank_t *)GPIO1_BASE)
#define LL_GPIO1_BANK1 ((ll_gpio_bank_t *)(GPIO1_BASE + LL_GPIO_BANK_SIZE))
#define LL_GPIO2_BANK0 ((ll_gpio_bank_t *)GPIO2_BASE)
/** @} */

/**
 * @brief Get PA bank pointer and pin mask from absolute pin index.
 * @param[in] pin Absolute PA pin index (PA00 as 0).
 * @param[out] pin_mask Output bit mask for the selected pin.
 * @return PA bank pointer containing the pin.
 */
static inline ll_gpio_bank_t *ll_gpio_pa_resolve_bank(uint32_t pin,
        uint32_t *pin_mask)
{
    *pin_mask = 1UL << (pin & 31U);
    if (pin < 32U)
    {
        return LL_GPIO1_BANK0;
    }

    return LL_GPIO1_BANK1;
}

/**
 * @brief Get PB bank pointer and pin mask from absolute pin index.
 * @param[in] pin Absolute PB pin index (PB00 as 0).
 * @param[out] pin_mask Output bit mask for the selected pin.
 * @return PB bank pointer containing the pin.
 */
static inline ll_gpio_bank_t *ll_gpio_pb_resolve_bank(uint32_t pin,
        uint32_t *pin_mask)
{
    *pin_mask = 1UL << (pin & 31U);
    return LL_GPIO2_BANK0;
}

/**
 * @brief Enable PA output driver by absolute pin index.
 * @param[in] pin Absolute PA pin index (PA00 as 0).
 */
static inline void ll_gpio_pa_enable_output(uint32_t pin)
{
    ll_gpio_bank_t *bank;
    uint32_t pin_mask;

    bank = ll_gpio_pa_resolve_bank(pin, &pin_mask);
    ll_gpio_bank_enable_output(bank, pin_mask);
}

/**
 * @brief Disable PA output driver by absolute pin index.
 * @param[in] pin Absolute PA pin index (PA00 as 0).
 */
static inline void ll_gpio_pa_disable_output(uint32_t pin)
{
    ll_gpio_bank_t *bank;
    uint32_t pin_mask;

    bank = ll_gpio_pa_resolve_bank(pin, &pin_mask);
    ll_gpio_bank_disable_output(bank, pin_mask);
}

/**
 * @brief Drive PA pin to high level by absolute pin index.
 * @param[in] pin Absolute PA pin index (PA00 as 0).
 */
static inline void ll_gpio_pa_set_high(uint32_t pin)
{
    ll_gpio_bank_t *bank;
    uint32_t pin_mask;

    bank = ll_gpio_pa_resolve_bank(pin, &pin_mask);
    ll_gpio_bank_set_high(bank, pin_mask);
}

/**
 * @brief Drive PA pin to low level by absolute pin index.
 * @param[in] pin Absolute PA pin index (PA00 as 0).
 */
static inline void ll_gpio_pa_set_low(uint32_t pin)
{
    ll_gpio_bank_t *bank;
    uint32_t pin_mask;

    bank = ll_gpio_pa_resolve_bank(pin, &pin_mask);
    ll_gpio_bank_set_low(bank, pin_mask);
}

/**
 * @brief Toggle PA pin by absolute pin index.
 * @param[in] pin Absolute PA pin index (PA00 as 0).
 * @note This function reads DOR and writes DOSR/DOCR separately. Concurrent
 * updates to the same pins can still race.
 */
static inline void ll_gpio_pa_toggle(uint32_t pin)
{
    ll_gpio_bank_t *bank;
    uint32_t pin_mask;

    bank = ll_gpio_pa_resolve_bank(pin, &pin_mask);
    ll_gpio_bank_toggle(bank, pin_mask);
}

/**
 * @brief Query PA input level by absolute pin index.
 * @param[in] pin Absolute PA pin index (PA00 as 0).
 * @return Non-zero when input level is high.
 */
static inline uint32_t ll_gpio_pa_is_input_high(uint32_t pin)
{
    ll_gpio_bank_t *bank;
    uint32_t pin_mask;

    bank = ll_gpio_pa_resolve_bank(pin, &pin_mask);
    return ll_gpio_bank_is_input_high(bank, pin_mask);
}

/**
 * @brief Enable PA interrupt by absolute pin index.
 * @param[in] pin Absolute PA pin index (PA00 as 0).
 */
static inline void ll_gpio_pa_enable_irq(uint32_t pin)
{
    ll_gpio_bank_t *bank;
    uint32_t pin_mask;

    bank = ll_gpio_pa_resolve_bank(pin, &pin_mask);
    ll_gpio_bank_enable_irq(bank, pin_mask);
}

/**
 * @brief Disable PA interrupt by absolute pin index.
 * @param[in] pin Absolute PA pin index (PA00 as 0).
 */
static inline void ll_gpio_pa_disable_irq(uint32_t pin)
{
    ll_gpio_bank_t *bank;
    uint32_t pin_mask;

    bank = ll_gpio_pa_resolve_bank(pin, &pin_mask);
    ll_gpio_bank_disable_irq(bank, pin_mask);
}

/**
 * @brief Query PA interrupt pending status by absolute pin index.
 * @param[in] pin Absolute PA pin index (PA00 as 0).
 * @return Non-zero when interrupt is pending.
 */
static inline uint32_t ll_gpio_pa_is_irq_pending(uint32_t pin)
{
    ll_gpio_bank_t *bank;
    uint32_t pin_mask;

    bank = ll_gpio_pa_resolve_bank(pin, &pin_mask);
    return (ll_gpio_bank_get_irq_pending(bank) & pin_mask);
}

/**
 * @brief Clear PA interrupt pending status by absolute pin index.
 * @param[in] pin Absolute PA pin index (PA00 as 0).
 */
static inline void ll_gpio_pa_clear_irq_pending(uint32_t pin)
{
    ll_gpio_bank_t *bank;
    uint32_t pin_mask;

    bank = ll_gpio_pa_resolve_bank(pin, &pin_mask);
    ll_gpio_bank_clear_irq_pending(bank, pin_mask);
}

/**
 * @brief Configure PA interrupt trigger by absolute pin index.
 * @param[in] pin Absolute PA pin index (PA00 as 0).
 * @param[in] cfg Pointer to IRQ trigger configuration.
 */
static inline void
ll_gpio_pa_config_irq_trigger(uint32_t pin, const ll_gpio_irq_config_t *cfg)
{
    ll_gpio_bank_t *bank;
    uint32_t pin_mask;

    bank = ll_gpio_pa_resolve_bank(pin, &pin_mask);
    ll_gpio_bank_config_irq_trigger(bank, pin_mask, cfg);
}

/**
 * @brief Enable PB output driver by absolute pin index.
 * @param[in] pin Absolute PB pin index (PB00 as 0).
 */
static inline void ll_gpio_pb_enable_output(uint32_t pin)
{
    ll_gpio_bank_enable_output(LL_GPIO2_BANK0, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Disable PB output driver by absolute pin index.
 * @param[in] pin Absolute PB pin index (PB00 as 0).
 */
static inline void ll_gpio_pb_disable_output(uint32_t pin)
{
    ll_gpio_bank_disable_output(LL_GPIO2_BANK0, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Drive PB pin to high level by absolute pin index.
 * @param[in] pin Absolute PB pin index (PB00 as 0).
 */
static inline void ll_gpio_pb_set_high(uint32_t pin)
{
    ll_gpio_bank_set_high(LL_GPIO2_BANK0, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Drive PB pin to low level by absolute pin index.
 * @param[in] pin Absolute PB pin index (PB00 as 0).
 */
static inline void ll_gpio_pb_set_low(uint32_t pin)
{
    ll_gpio_bank_set_low(LL_GPIO2_BANK0, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Toggle PB pin by absolute pin index.
 * @param[in] pin Absolute PB pin index (PB00 as 0).
 * @note This function reads DOR and writes DOSR/DOCR separately. Concurrent
 * updates to the same pins can still race.
 */
static inline void ll_gpio_pb_toggle(uint32_t pin)
{
    ll_gpio_bank_toggle(LL_GPIO2_BANK0, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Query PB input level by absolute pin index.
 * @param[in] pin Absolute PB pin index (PB00 as 0).
 * @return Non-zero when input level is high.
 */
static inline uint32_t ll_gpio_pb_is_input_high(uint32_t pin)
{
    return ll_gpio_bank_is_input_high(LL_GPIO2_BANK0, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Enable PB interrupt by absolute pin index.
 * @param[in] pin Absolute PB pin index (PB00 as 0).
 */
static inline void ll_gpio_pb_enable_irq(uint32_t pin)
{
    ll_gpio_bank_enable_irq(LL_GPIO2_BANK0, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Disable PB interrupt by absolute pin index.
 * @param[in] pin Absolute PB pin index (PB00 as 0).
 */
static inline void ll_gpio_pb_disable_irq(uint32_t pin)
{
    ll_gpio_bank_disable_irq(LL_GPIO2_BANK0, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Query PB interrupt pending status by absolute pin index.
 * @param[in] pin Absolute PB pin index (PB00 as 0).
 * @return Non-zero when interrupt is pending.
 */
static inline uint32_t ll_gpio_pb_is_irq_pending(uint32_t pin)
{
    return (ll_gpio_bank_get_irq_pending(LL_GPIO2_BANK0) &
            ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Clear PB interrupt pending status by absolute pin index.
 * @param[in] pin Absolute PB pin index (PB00 as 0).
 */
static inline void ll_gpio_pb_clear_irq_pending(uint32_t pin)
{
    ll_gpio_bank_clear_irq_pending(LL_GPIO2_BANK0, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Configure PB interrupt trigger by absolute pin index.
 * @param[in] pin Absolute PB pin index (PB00 as 0).
 * @param[in] cfg Pointer to IRQ trigger configuration.
 */
static inline void
ll_gpio_pb_config_irq_trigger(uint32_t pin, const ll_gpio_irq_config_t *cfg)
{
    ll_gpio_bank_config_irq_trigger(LL_GPIO2_BANK0, ll_gpio_pin_to_mask(pin),
                                    cfg);
}
#ifdef __cplusplus
}
#endif

#endif /* __LL_GPIO_EX_H */