/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __LL_PTC_H
#define __LL_PTC_H

#include <stdint.h>
#include "register.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file ll_ptc.h
 * @brief Header-only low-level Peripheral Trigger Controller (PTC) APIs for
 *        SF32LB52x.
 *
 * PTC provides 8 configurable tasks (TASK1..TASK8). Each owns a
 * TCR/TAR/TDR/RCR register pair and performs an operation on an arbitrary
 * peripheral address when triggered.
 *
 * All task-taking APIs use a 0-based @p task index: 0 = TASK1 ... 7 = TASK8.
 * As a low-level layer these APIs do not validate the index; the caller must
 * pass a value in 0..7.
 */

/**
 * @brief Get the register-block word offset for TASK1..TASK8 (TCR1..TCR8).
 * @param[in] task Task index in 0..7 (0 = TASK1 ... 7 = TASK8); not validated.
 * @return Word offset from TCR1 (task*4 stride, since each task owns
 *         TCR/TAR/TDR/RCR).
 */
static inline uint32_t ll_ptc_task_word_offset(uint32_t task)
{
    return task * 4U;
}

/*==============================================================================
 * Task Operation (TCRx.OP)
 *============================================================================*/

/** @defgroup LL_PTC_OP Task Operation Mode */
/** @{ */
#define LL_PTC_OP_DIRECT_WRITE (0x0UL) /**< Direct write data to address. */
#define LL_PTC_OP_XOR          (0x4UL) /**< Read, XOR with data, write back. */
#define LL_PTC_OP_OR           (0x5UL) /**< Read, OR with data, write back. */
#define LL_PTC_OP_AND          (0x6UL) /**< Read, AND with data, write back. */
#define LL_PTC_OP_ADD          (0x7UL) /**< Read, add data, write back. */
/** @} */

/**
 * @brief Get the transfer-error flag of a task (ISR.TEIFx).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 * @return Non-zero when the transfer-error flag is set.
 */
static inline uint32_t ll_ptc_is_active_flag_te(PTC_TypeDef *PTCx, uint32_t task)
{
    volatile uint32_t *isr = &PTCx->ISR;

    /* TEIF1..TEIF8 occupy ISR bits 16..23. */
    return READ_BIT(*isr, (1UL << (task + 16U)));
}

/*==============================================================================
 * Interrupts (ISR / ICR / IER)
 *============================================================================*/

/**
 * @brief Get the task-complete flag of a task (ISR.TCIFx).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 * @return Non-zero when the task-complete flag is set.
 */
static inline uint32_t ll_ptc_is_active_flag_tc(PTC_TypeDef *PTCx, uint32_t task)
{
    volatile uint32_t *isr = &PTCx->ISR;

    /* TCIF1..TCIF8 occupy ISR bits 0..7. */
    return READ_BIT(*isr, (1UL << task));
}

/**
 * @brief Clear the transfer-error flag (ICR.CTEIF, write 1 to clear).
 * @param[in] PTCx PTC instance pointer.
 */
static inline void ll_ptc_clear_flag_te(PTC_TypeDef *PTCx)
{
    SET_BIT(PTCx->ICR, PTC_ICR_CTEIF);
}

/**
 * @brief Clear the task-complete flag (ICR.CTCIFx, write 1 to clear).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 */
static inline void ll_ptc_clear_flag_tc(PTC_TypeDef *PTCx, uint32_t task)
{
    volatile uint32_t *icr = &PTCx->ICR;

    /* CTCIF1..CTCIF8 occupy ICR bits 0..7. */
    SET_BIT(*icr, (1UL << task));
}

/**
 * @brief Enable the transfer-error interrupt (IER.TEIE).
 * @param[in] PTCx PTC instance pointer.
 */
static inline void ll_ptc_enable_it_te(PTC_TypeDef *PTCx)
{
    SET_BIT(PTCx->IER, PTC_IER_TEIE);
}

/**
 * @brief Disable the transfer-error interrupt (IER.TEIE = 0).
 * @param[in] PTCx PTC instance pointer.
 */
static inline void ll_ptc_disable_it_te(PTC_TypeDef *PTCx)
{
    CLEAR_BIT(PTCx->IER, PTC_IER_TEIE);
}

/**
 * @brief Enable the task-complete interrupt (IER.TCIEx).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 */
static inline void ll_ptc_enable_it_tc(PTC_TypeDef *PTCx, uint32_t task)
{
    volatile uint32_t *ier = &PTCx->IER;

    /* TCIE1..TCIE8 occupy IER bits 0..7. */
    SET_BIT(*ier, (1UL << task));
}

/**
 * @brief Disable the task-complete interrupt (IER.TCIEx = 0).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 */
static inline void ll_ptc_disable_it_tc(PTC_TypeDef *PTCx, uint32_t task)
{
    volatile uint32_t *ier = &PTCx->IER;

    /* TCIE1..TCIE8 occupy IER bits 0..7. */
    CLEAR_BIT(*ier, (1UL << task));
}

/**
 * @brief Interrupt after each operation (TCRx.REPIRQ = 0).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 */
static inline void ll_ptc_set_task_irq_each(PTC_TypeDef *PTCx, uint32_t task)
{
    volatile uint32_t *tcr = &PTCx->TCR1;

    CLEAR_BIT(tcr[ll_ptc_task_word_offset(task)], PTC_TCR1_REPIRQ);
}

/**
 * @brief Interrupt after REP operations (TCRx.REPIRQ).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 */
static inline void ll_ptc_set_task_irq_rep(PTC_TypeDef *PTCx, uint32_t task)
{
    volatile uint32_t *tcr = &PTCx->TCR1;

    SET_BIT(tcr[ll_ptc_task_word_offset(task)], PTC_TCR1_REPIRQ);
}

/**
 * @brief Make the task output a trigger every REP operations (TCRx.REPTRIG).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 */
static inline void ll_ptc_enable_task_rep_trigger(PTC_TypeDef *PTCx, uint32_t task)
{
    volatile uint32_t *tcr = &PTCx->TCR1;

    SET_BIT(tcr[ll_ptc_task_word_offset(task)], PTC_TCR1_REPTRIG);
}

/**
 * @brief Make the task output a trigger on every operation (TCRx.REPTRIG = 0).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 */
static inline void ll_ptc_disable_task_rep_trigger(PTC_TypeDef *PTCx, uint32_t task)
{
    volatile uint32_t *tcr = &PTCx->TCR1;

    CLEAR_BIT(tcr[ll_ptc_task_word_offset(task)], PTC_TCR1_REPTRIG);
}

/**
 * @brief Enable task repetition gating (TCRx.REPEN).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 * @note When enabled the task only fires while RCRx.REP is non-zero.
 */
static inline void ll_ptc_enable_task_rep(PTC_TypeDef *PTCx, uint32_t task)
{
    volatile uint32_t *tcr = &PTCx->TCR1;

    SET_BIT(tcr[ll_ptc_task_word_offset(task)], PTC_TCR1_REPEN);
}

/**
 * @brief Disable task repetition gating (TCRx.REPEN = 0).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 */
static inline void ll_ptc_disable_task_rep(PTC_TypeDef *PTCx, uint32_t task)
{
    volatile uint32_t *tcr = &PTCx->TCR1;

    CLEAR_BIT(tcr[ll_ptc_task_word_offset(task)], PTC_TCR1_REPEN);
}

/**
 * @brief Trigger a task by software (TCRx.SWTRIG, auto-cleared).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 */
static inline void ll_ptc_software_trigger_task(PTC_TypeDef *PTCx, uint32_t task)
{
    volatile uint32_t *tcr = &PTCx->TCR1;

    SET_BIT(tcr[ll_ptc_task_word_offset(task)], PTC_TCR1_SWTRIG);
}

/**
 * @brief Set the task trigger polarity (TCRx.TRIGPOL).
 * @param[in] PTCx    PTC instance pointer.
 * @param[in] task    Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 * @param[in] neg_pol Non-zero selects negative edge, zero positive edge.
 */
static inline void ll_ptc_set_task_trig_polarity(PTC_TypeDef *PTCx, uint32_t task,
        uint32_t neg_pol)
{
    volatile uint32_t *tcr = &PTCx->TCR1;

    MODIFY_REG(tcr[ll_ptc_task_word_offset(task)], PTC_TCR1_TRIGPOL,
               neg_pol ? PTC_TCR1_TRIGPOL : 0U);
}

/**
 * @brief Set the task operation mode (TCRx.OP).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 * @param[in] op   Operation, use @ref LL_PTC_OP_DIRECT_WRITE to @ref LL_PTC_OP_ADD.
 */
static inline void ll_ptc_set_task_op(PTC_TypeDef *PTCx, uint32_t task, uint32_t op)
{
    volatile uint32_t *tcr = &PTCx->TCR1;

    MODIFY_REG(tcr[ll_ptc_task_word_offset(task)], PTC_TCR1_OP,
               MAKE_REG_VAL(op, PTC_TCR1_OP_Msk, PTC_TCR1_OP_Pos));
}

/**
 * @brief Set the task trigger source (TCRx.TRIGSEL).
 * @param[in] PTCx    PTC instance pointer.
 * @param[in] task    Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 * @param[in] trigsel Trigger source select (8 bits); 0 = software only.
 */
static inline void ll_ptc_set_task_trig_sel(PTC_TypeDef *PTCx, uint32_t task,
        uint32_t trigsel)
{
    volatile uint32_t *tcr = &PTCx->TCR1;

    MODIFY_REG(tcr[ll_ptc_task_word_offset(task)], PTC_TCR1_TRIGSEL,
               MAKE_REG_VAL(trigsel, PTC_TCR1_TRIGSEL_Msk, PTC_TCR1_TRIGSEL_Pos));
}

/**
 * @brief Set the task peripheral address (TARx.ADDR).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 * @param[in] addr Peripheral address to access.
 */
static inline void ll_ptc_set_task_addr(PTC_TypeDef *PTCx, uint32_t task, uint32_t addr)
{
    volatile uint32_t *tar = &PTCx->TAR1;

    WRITE_REG(tar[ll_ptc_task_word_offset(task)], addr);
}

/**
 * @brief Set the task data value (TDRx.DATA).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 * @param[in] data Data value for the task operation.
 */
static inline void ll_ptc_set_task_data(PTC_TypeDef *PTCx, uint32_t task, uint32_t data)
{
    volatile uint32_t *tdr = &PTCx->TDR1;

    WRITE_REG(tdr[ll_ptc_task_word_offset(task)], data);
}

/**
 * @brief Set the task pre-trigger delay (RCRx.DLY).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 * @param[in] dly  Delay in HCLK cycles (16 bits); 0 = no delay.
 */
static inline void ll_ptc_set_task_delay(PTC_TypeDef *PTCx, uint32_t task, uint32_t dly)
{
    volatile uint32_t *rcr = &PTCx->RCR1;

    MODIFY_REG(rcr[ll_ptc_task_word_offset(task)], PTC_RCR1_DLY,
               MAKE_REG_VAL(dly, PTC_RCR1_DLY_Msk, PTC_RCR1_DLY_Pos));
}

/**
 * @brief Set the task repetition count (RCRx.REP).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] task Task index in range 0..7 (0 = TASK1 ... 7 = TASK8).
 * @param[in] rep  Repetition count (10 bits).
 */
static inline void ll_ptc_set_task_rep(PTC_TypeDef *PTCx, uint32_t task, uint32_t rep)
{
    volatile uint32_t *rcr = &PTCx->RCR1;

    MODIFY_REG(rcr[ll_ptc_task_word_offset(task)], PTC_RCR1_REP,
               MAKE_REG_VAL(rep, PTC_RCR1_REP_Msk, PTC_RCR1_REP_Pos));
}

/*==============================================================================
 * Memory & GPIO Trigger Select (MEM1..4 / GPIO31_0 / GPIO63_32)
 *============================================================================*/

/**
 * @brief Read a PTC scratch memory word (MEMx).
 * @param[in] PTCx PTC instance pointer.
 * @param[in] idx  Memory index in range 1..4.
 * @return Memory word value.
 */
static inline uint32_t ll_ptc_get_mem(PTC_TypeDef *PTCx, uint32_t idx)
{
    volatile uint32_t *mem = &PTCx->MEM1;

    return (idx >= 1U && idx <= 4U) ? READ_REG(mem[idx - 1U]) : 0U;
}

/**
 * @brief Write a PTC scratch memory word (MEMx).
 * @param[in] PTCx  PTC instance pointer.
 * @param[in] idx   Memory index in range 1..4.
 * @param[in] value Word value to store.
 */
static inline void ll_ptc_set_mem(PTC_TypeDef *PTCx, uint32_t idx, uint32_t value)
{
    volatile uint32_t *mem = &PTCx->MEM1;

    if (idx >= 1U && idx <= 4U)
    {
        WRITE_REG(mem[idx - 1U], value);
    }
}

/**
 * @brief Select the GPIO trigger D source (GPIO31_0.SELD).
 * @param[in] PTCx  PTC instance pointer.
 * @param[in] group GPIO group (0 = GPIO31_0, 1 = GPIO63_32).
 * @param[in] gpio  GPIO index to select (0..31).
 */
static inline void ll_ptc_set_gpio_trig_d(PTC_TypeDef *PTCx, uint32_t group, uint32_t gpio)
{
    volatile uint32_t *gpio_reg = (group == 0U) ? &PTCx->GPIO31_0 : &PTCx->GPIO63_32;
    MODIFY_REG(*gpio_reg, PTC_GPIO31_0_SELD,
               MAKE_REG_VAL(gpio, PTC_GPIO31_0_SELD_Msk, PTC_GPIO31_0_SELD_Pos));
}

/**
 * @brief Select the GPIO trigger C source (GPIO31_0.SELC).
 * @param[in] PTCx  PTC instance pointer.
 * @param[in] group GPIO group (0 = GPIO31_0, 1 = GPIO63_32).
 * @param[in] gpio  GPIO index to select (0..31).
 */
static inline void ll_ptc_set_gpio_trig_c(PTC_TypeDef *PTCx, uint32_t group, uint32_t gpio)
{
    volatile uint32_t *gpio_reg = (group == 0U) ? &PTCx->GPIO31_0 : &PTCx->GPIO63_32;
    MODIFY_REG(*gpio_reg, PTC_GPIO31_0_SELC,
               MAKE_REG_VAL(gpio, PTC_GPIO31_0_SELC_Msk, PTC_GPIO31_0_SELC_Pos));
}

/**
 * @brief Select the GPIO trigger B source (GPIO31_0.SELB).
 * @param[in] PTCx  PTC instance pointer.
 * @param[in] group GPIO group (0 = GPIO31_0, 1 = GPIO63_32).
 * @param[in] gpio  GPIO index to select (0..31).
 */
static inline void ll_ptc_set_gpio_trig_b(PTC_TypeDef *PTCx, uint32_t group, uint32_t gpio)
{
    volatile uint32_t *gpio_reg = (group == 0U) ? &PTCx->GPIO31_0 : &PTCx->GPIO63_32;
    MODIFY_REG(*gpio_reg, PTC_GPIO31_0_SELB,
               MAKE_REG_VAL(gpio, PTC_GPIO31_0_SELB_Msk, PTC_GPIO31_0_SELB_Pos));
}

/**
 * @brief Select the GPIO trigger A source (GPIO31_0.SELA).
 * @param[in] PTCx  PTC instance pointer.
 * @param[in] group GPIO group (0 = GPIO31_0, 1 = GPIO63_32).
 * @param[in] gpio  GPIO index to select (0..31).
 */
static inline void ll_ptc_set_gpio_trig_a(PTC_TypeDef *PTCx, uint32_t group, uint32_t gpio)
{
    volatile uint32_t *gpio_reg = (group == 0U) ? &PTCx->GPIO31_0 : &PTCx->GPIO63_32;
    MODIFY_REG(*gpio_reg, PTC_GPIO31_0_SELA,
               MAKE_REG_VAL(gpio, PTC_GPIO31_0_SELA_Msk, PTC_GPIO31_0_SELA_Pos));
}

/*==============================================================================
 * Task Registers (TCRx / TARx / TDRx / RCRx)
 *============================================================================*/

#ifdef __cplusplus
}
#endif

#endif /* __LL_PTC_H */
