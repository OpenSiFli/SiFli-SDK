/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_GPIO_H
#define __LL_GPIO_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file ll_gpio.h
 * @brief Header-only low-level GPIO APIs for SF32LB52x.
 */

/** @defgroup LL_GPIO_IRQ_TYPE LL GPIO IRQ Trigger Type */
/** @{ */
#define LL_GPIO_IRQ_TYPE_LEVEL 0U
#define LL_GPIO_IRQ_TYPE_EDGE 1U
/** @} */

/** @defgroup LL_GPIO_IRQ_POLARITY LL GPIO IRQ Polarity */
/** @{ */
#define LL_GPIO_IRQ_POL_LOW 1U
#define LL_GPIO_IRQ_POL_HIGH 2U
#define LL_GPIO_IRQ_POL_BOTH 3U
/** @} */

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
 * @brief Single GPIO bank register layout.
 */
typedef struct
{
    __IO uint32_t DIR;   /**< Input data register. */
    __IO uint32_t DOR;   /**< Output data register. */
    __IO uint32_t DOSR;  /**< Output set register. */
    __IO uint32_t DOCR;  /**< Output clear register. */
    __IO uint32_t DOER;  /**< Output enable register. */
    __IO uint32_t DOESR; /**< Output enable set register. */
    __IO uint32_t DOECR; /**< Output enable clear register. */
    __IO uint32_t IER;   /**< Interrupt enable register. */
    __IO uint32_t IESR;  /**< Interrupt enable set register. */
    __IO uint32_t IECR;  /**< Interrupt enable clear register. */
    __IO uint32_t ITR;   /**< Interrupt type register. */
    __IO uint32_t ITSR;  /**< Interrupt type set register. */
    __IO uint32_t ITCR;  /**< Interrupt type clear register. */
    __IO uint32_t IPHR;  /**< High-level or rising-edge enable register. */
    __IO uint32_t IPHSR; /**< High-level or rising-edge set register. */
    __IO uint32_t IPHCR; /**< High-level or rising-edge clear register. */
    __IO uint32_t IPLR;  /**< Low-level or falling-edge enable register. */
    __IO uint32_t IPLSR; /**< Low-level or falling-edge set register. */
    __IO uint32_t IPLCR; /**< Low-level or falling-edge clear register. */
    __IO uint32_t ISR;   /**< Interrupt status register (write 1 to clear). */
    __IO uint32_t
        RSVD[4];         /**< Reserved space for undocumented EXT registers. */
    __IO uint32_t OEMR;  /**< Output mode register. */
    __IO uint32_t OEMSR; /**< Output mode set register. */
    __IO uint32_t OEMCR; /**< Output mode clear register. */
} ll_gpio_bank_t;

/**
 * @brief GPIO IRQ trigger configuration.
 */
typedef struct
{
    uint32_t type; /**< Trigger type, use @ref LL_GPIO_IRQ_TYPE_LEVEL or @ref
                      LL_GPIO_IRQ_TYPE_EDGE. */
    uint32_t polarity; /**< Trigger polarity, use @ref LL_GPIO_IRQ_POL_LOW, @ref
                          LL_GPIO_IRQ_POL_HIGH, or @ref LL_GPIO_IRQ_POL_BOTH. */
} ll_gpio_irq_config_t;

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
 * @brief Convert pin index to bit mask within a bank.
 * @param[in] pin Pin index in the bank.
 * @return Pin bit mask.
 */
static inline uint32_t ll_gpio_pin_to_mask(uint32_t pin)
{
    return (1UL << (pin & 31U));
}

/**
 * @brief Enable output driver for selected pins.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 */
static inline void ll_gpio_bank_enable_output(ll_gpio_bank_t *bank,
                                              uint32_t pin_mask)
{
    bank->DOESR = pin_mask;
}

/**
 * @brief Disable output driver for selected pins.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 */
static inline void ll_gpio_bank_disable_output(ll_gpio_bank_t *bank,
                                               uint32_t pin_mask)
{
    bank->DOECR = pin_mask;
}

/**
 * @brief Query output enable status for selected pins.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 * @return Non-zero for enabled pins.
 */
static inline uint32_t ll_gpio_bank_is_output_enabled(ll_gpio_bank_t *bank,
                                                      uint32_t pin_mask)
{
    return (bank->DOER & pin_mask);
}

/**
 * @brief Drive selected pins to high level.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 */
static inline void ll_gpio_bank_set_high(ll_gpio_bank_t *bank,
                                         uint32_t pin_mask)
{
    bank->DOSR = pin_mask;
}

/**
 * @brief Drive selected pins to low level.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 */
static inline void ll_gpio_bank_set_low(ll_gpio_bank_t *bank, uint32_t pin_mask)
{
    bank->DOCR = pin_mask;
}

/**
 * @brief Toggle selected output pins.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 * @note This function reads DOR and writes DOSR/DOCR separately. Concurrent
 * updates to the same pins can still race.
 */
static inline void ll_gpio_bank_toggle(ll_gpio_bank_t *bank, uint32_t pin_mask)
{
    uint32_t cur_mask;
    uint32_t set_mask;

    cur_mask = bank->DOR & pin_mask;
    set_mask = (~cur_mask) & pin_mask;

    bank->DOSR = set_mask;
    bank->DOCR = cur_mask;
}

/**
 * @brief Read output latch value of the full bank.
 * @param[in] bank GPIO bank pointer.
 * @return 32-bit output latch value.
 */
static inline uint32_t ll_gpio_bank_read_output_port(ll_gpio_bank_t *bank)
{
    return bank->DOR;
}

/**
 * @brief Read input level of the full bank.
 * @param[in] bank GPIO bank pointer.
 * @return 32-bit input level value.
 */
static inline uint32_t ll_gpio_bank_read_input_port(ll_gpio_bank_t *bank)
{
    return bank->DIR;
}

/**
 * @brief Query input level for selected pins.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 * @return Non-zero for pins at high level.
 */
static inline uint32_t ll_gpio_bank_is_input_high(ll_gpio_bank_t *bank,
                                                  uint32_t pin_mask)
{
    return (bank->DIR & pin_mask);
}

/**
 * @brief Enable GPIO interrupt for selected pins.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 */
static inline void ll_gpio_bank_enable_irq(ll_gpio_bank_t *bank,
                                           uint32_t pin_mask)
{
    bank->IESR = pin_mask;
}

/**
 * @brief Disable GPIO interrupt for selected pins.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 */
static inline void ll_gpio_bank_disable_irq(ll_gpio_bank_t *bank,
                                            uint32_t pin_mask)
{
    bank->IECR = pin_mask;
}

/**
 * @brief Query interrupt enable state for selected pins.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 * @return Non-zero for interrupt-enabled pins.
 */
static inline uint32_t ll_gpio_bank_is_irq_enabled(ll_gpio_bank_t *bank,
                                                   uint32_t pin_mask)
{
    return (bank->IER & pin_mask);
}

/**
 * @brief Read interrupt pending flags.
 * @param[in] bank GPIO bank pointer.
 * @return 32-bit pending bitmap.
 */
static inline uint32_t ll_gpio_bank_get_irq_pending(ll_gpio_bank_t *bank)
{
    return bank->ISR;
}

/**
 * @brief Clear interrupt pending flags.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of pending flags to clear.
 */
static inline void ll_gpio_bank_clear_irq_pending(ll_gpio_bank_t *bank,
                                                  uint32_t pin_mask)
{
    bank->ISR = pin_mask;
}

/**
 * @brief Configure interrupt trigger type and polarity.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 * @param[in] cfg Pointer to IRQ trigger configuration.
 */
static inline void
ll_gpio_bank_config_irq_trigger(ll_gpio_bank_t *bank, uint32_t pin_mask,
                                const ll_gpio_irq_config_t *cfg)
{
    if (cfg->type == LL_GPIO_IRQ_TYPE_EDGE)
    {
        bank->ITSR = pin_mask;
    }
    else
    {
        bank->ITCR = pin_mask;
    }

    if ((cfg->polarity & LL_GPIO_IRQ_POL_HIGH) != 0U)
    {
        bank->IPHSR = pin_mask;
    }
    else
    {
        bank->IPHCR = pin_mask;
    }

    if ((cfg->polarity & LL_GPIO_IRQ_POL_LOW) != 0U)
    {
        bank->IPLSR = pin_mask;
    }
    else
    {
        bank->IPLCR = pin_mask;
    }
}

/**
 * @brief Set output mode bit for selected pins.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 */
static inline void ll_gpio_bank_set_output_mode(ll_gpio_bank_t *bank,
                                                uint32_t pin_mask)
{
    bank->OEMSR = pin_mask;
}

/**
 * @brief Clear output mode bit for selected pins.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 */
static inline void ll_gpio_bank_clear_output_mode(ll_gpio_bank_t *bank,
                                                  uint32_t pin_mask)
{
    bank->OEMCR = pin_mask;
}

/**
 * @brief Read output mode register of the full bank.
 * @param[in] bank GPIO bank pointer.
 * @return 32-bit output mode value.
 */
static inline uint32_t ll_gpio_bank_get_output_mode(ll_gpio_bank_t *bank)
{
    return bank->OEMR;
}

/**
 * @brief Enable output driver by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 */
static inline void ll_gpio_bank_enable_output_pin(ll_gpio_bank_t *bank,
                                                  uint32_t pin)
{
    ll_gpio_bank_enable_output(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Disable output driver by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 */
static inline void ll_gpio_bank_disable_output_pin(ll_gpio_bank_t *bank,
                                                   uint32_t pin)
{
    ll_gpio_bank_disable_output(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Query output enable status by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 * @return Non-zero when output is enabled.
 */
static inline uint32_t ll_gpio_bank_is_output_enabled_pin(ll_gpio_bank_t *bank,
                                                          uint32_t pin)
{
    return ll_gpio_bank_is_output_enabled(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Drive pin to high level by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 */
static inline void ll_gpio_bank_set_high_pin(ll_gpio_bank_t *bank, uint32_t pin)
{
    ll_gpio_bank_set_high(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Drive pin to low level by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 */
static inline void ll_gpio_bank_set_low_pin(ll_gpio_bank_t *bank, uint32_t pin)
{
    ll_gpio_bank_set_low(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Toggle pin by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 * @note This function performs a read-modify-write on DOR and is not atomic.
 */
static inline void ll_gpio_bank_toggle_pin(ll_gpio_bank_t *bank, uint32_t pin)
{
    ll_gpio_bank_toggle(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Query input level by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 * @return Non-zero when input level is high.
 */
static inline uint32_t ll_gpio_bank_is_input_high_pin(ll_gpio_bank_t *bank,
                                                      uint32_t pin)
{
    return ll_gpio_bank_is_input_high(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Enable interrupt by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 */
static inline void ll_gpio_bank_enable_irq_pin(ll_gpio_bank_t *bank,
                                               uint32_t pin)
{
    ll_gpio_bank_enable_irq(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Disable interrupt by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 */
static inline void ll_gpio_bank_disable_irq_pin(ll_gpio_bank_t *bank,
                                                uint32_t pin)
{
    ll_gpio_bank_disable_irq(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Query interrupt enable status by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 * @return Non-zero when interrupt is enabled.
 */
static inline uint32_t ll_gpio_bank_is_irq_enabled_pin(ll_gpio_bank_t *bank,
                                                       uint32_t pin)
{
    return ll_gpio_bank_is_irq_enabled(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Query interrupt pending status by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 * @return Non-zero when interrupt is pending.
 */
static inline uint32_t ll_gpio_bank_is_irq_pending_pin(ll_gpio_bank_t *bank,
                                                       uint32_t pin)
{
    return (ll_gpio_bank_get_irq_pending(bank) & ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Clear interrupt pending status by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 */
static inline void ll_gpio_bank_clear_irq_pending_pin(ll_gpio_bank_t *bank,
                                                      uint32_t pin)
{
    ll_gpio_bank_clear_irq_pending(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Configure interrupt trigger by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 * @param[in] cfg Pointer to IRQ trigger configuration.
 */
static inline void
ll_gpio_bank_config_irq_trigger_pin(ll_gpio_bank_t *bank, uint32_t pin,
                                    const ll_gpio_irq_config_t *cfg)
{
    ll_gpio_bank_config_irq_trigger(bank, ll_gpio_pin_to_mask(pin), cfg);
}

/**
 * @brief Set output mode bit by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 */
static inline void ll_gpio_bank_set_output_mode_pin(ll_gpio_bank_t *bank,
                                                    uint32_t pin)
{
    ll_gpio_bank_set_output_mode(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Clear output mode bit by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 */
static inline void ll_gpio_bank_clear_output_mode_pin(ll_gpio_bank_t *bank,
                                                      uint32_t pin)
{
    ll_gpio_bank_clear_output_mode(bank, ll_gpio_pin_to_mask(pin));
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

#endif /* __LL_GPIO_H */
