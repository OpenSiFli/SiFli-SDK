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
    __IO uint32_t IER_EXT;  /**< Extended interrupt enable register. */
    __IO uint32_t IESR_EXT; /**< Extended interrupt enable set register. */
    __IO uint32_t IECR_EXT; /**< Extended interrupt enable clear register. */
    __IO uint32_t ISR_EXT;  /**< Extended interrupt status register. */
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
 * @brief Read the interrupt type register (ITR) of a bank.
 * @param[in] bank GPIO bank pointer.
 * @return Interrupt type bit mask (1 = edge-sensitive, 0 = level-sensitive).
 */
static inline uint32_t ll_gpio_bank_get_irq_type(ll_gpio_bank_t *bank)
{
    return READ_REG(bank->ITR);
}

/**
 * @brief Write the interrupt type register (ITR) of a bank.
 * @param[in] bank     GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 */
static inline void ll_gpio_bank_set_irq_type(ll_gpio_bank_t *bank, uint32_t pin_mask)
{
    WRITE_REG(bank->ITR, pin_mask);
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
 * @brief Read the high-level/rising-edge enable register (IPHR) of a bank.
 * @param[in] bank GPIO bank pointer.
 * @return High-level/rising-edge enable bit mask.
 */
static inline uint32_t ll_gpio_bank_get_irq_high_polarity(ll_gpio_bank_t *bank)
{
    return READ_REG(bank->IPHR);
}

/**
 * @brief Write the high-level/rising-edge enable register (IPHR) of a bank.
 * @param[in] bank     GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 */
static inline void ll_gpio_bank_set_irq_high_polarity(ll_gpio_bank_t *bank,
        uint32_t pin_mask)
{
    WRITE_REG(bank->IPHR, pin_mask);
}

/**
 * @brief Read the low-level/falling-edge enable register (IPLR) of a bank.
 * @param[in] bank GPIO bank pointer.
 * @return Low-level/falling-edge enable bit mask.
 */
static inline uint32_t ll_gpio_bank_get_irq_low_polarity(ll_gpio_bank_t *bank)
{
    return READ_REG(bank->IPLR);
}

/**
 * @brief Write the low-level/falling-edge enable register (IPLR) of a bank.
 * @param[in] bank     GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 */
static inline void ll_gpio_bank_set_irq_low_polarity(ll_gpio_bank_t *bank,
        uint32_t pin_mask)
{
    WRITE_REG(bank->IPLR, pin_mask);
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
 * @brief Query extended interrupt enable state for selected pins.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 * @return Non-zero for pins with extended interrupt enabled.
 */
static inline uint32_t ll_gpio_bank_is_ext_irq_enabled(ll_gpio_bank_t *bank,
        uint32_t pin_mask)
{
    return bank->IER_EXT & pin_mask;
}

/**
 * @brief Enable the extended GPIO interrupt path for selected pins.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 */
static inline void ll_gpio_bank_enable_ext_irq(ll_gpio_bank_t *bank,
        uint32_t pin_mask)
{
    bank->IESR_EXT = pin_mask;
}

/**
 * @brief Disable the extended GPIO interrupt path for selected pins.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of target pins.
 */
static inline void ll_gpio_bank_disable_ext_irq(ll_gpio_bank_t *bank,
        uint32_t pin_mask)
{
    bank->IECR_EXT = pin_mask;
}

/**
 * @brief Read extended interrupt pending flags.
 * @param[in] bank GPIO bank pointer.
 * @return Extended interrupt pending bitmap.
 */
static inline uint32_t ll_gpio_bank_get_ext_irq_pending(ll_gpio_bank_t *bank)
{
    return bank->ISR_EXT;
}

/**
 * @brief Clear extended interrupt pending flags.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin_mask Bit mask of pending flags to clear.
 */
static inline void ll_gpio_bank_clear_ext_irq_pending(ll_gpio_bank_t *bank,
        uint32_t pin_mask)
{
    bank->ISR_EXT = pin_mask;
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
 * @brief Convert pin index to bit mask within a bank.
 * @param[in] pin Pin index in the bank.
 * @return Pin bit mask.
 */
static inline uint32_t ll_gpio_pin_to_mask(uint32_t pin)
{
    return (1UL << (pin & 31U));
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
 * @brief Enable extended interrupt by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 */
static inline void ll_gpio_bank_enable_ext_irq_pin(ll_gpio_bank_t *bank,
        uint32_t pin)
{
    ll_gpio_bank_enable_ext_irq(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Disable extended interrupt by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 */
static inline void ll_gpio_bank_disable_ext_irq_pin(ll_gpio_bank_t *bank,
        uint32_t pin)
{
    ll_gpio_bank_disable_ext_irq(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Query extended interrupt enable status by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 * @return Non-zero when extended interrupt is enabled.
 */
static inline uint32_t ll_gpio_bank_is_ext_irq_enabled_pin(ll_gpio_bank_t *bank,
        uint32_t pin)
{
    return ll_gpio_bank_is_ext_irq_enabled(bank, ll_gpio_pin_to_mask(pin));
}

/**
 * @brief Query extended interrupt pending status by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 * @return Non-zero when extended interrupt is pending.
 */
static inline uint32_t ll_gpio_bank_is_ext_irq_pending_pin(ll_gpio_bank_t *bank,
        uint32_t pin)
{
    return ll_gpio_bank_get_ext_irq_pending(bank) & ll_gpio_pin_to_mask(pin);
}

/**
 * @brief Clear extended interrupt pending status by pin index.
 * @param[in] bank GPIO bank pointer.
 * @param[in] pin Pin index in the bank.
 */
static inline void ll_gpio_bank_clear_ext_irq_pending_pin(ll_gpio_bank_t *bank,
        uint32_t pin)
{
    ll_gpio_bank_clear_ext_irq_pending(bank, ll_gpio_pin_to_mask(pin));
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


#ifdef __cplusplus
}
#endif

#endif /* __LL_GPIO_H */
