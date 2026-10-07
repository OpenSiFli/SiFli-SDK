/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* suite: ptc
 *
 * The suite is the selection unit the host filters on. Naming this file
 * test_ptc.c would derive the same name; it is stated anyway, because the rule
 * ("file name, or the `suite:` hint") is easier to see here than to reconstruct.
 */

/*
 * The real HAL, compiled and executed -- registers in RAM.
 *
 * drivers/hal/bf0_hal_ptc.c is one translation unit with no dependencies: it
 * includes bf0_hal.h, touches no hwp_* pointer and calls nothing outside itself.
 * Its three functions are reached through PTC_HandleTypeDef, which carries the
 * register base as `Instance` -- so a test does not remap anything. It hands the
 * HAL the address of a PTC_TypeDef of its own and every `hptc->Instance->FIELD`
 * in the driver lands in that object:
 *
 *     static PTC_TypeDef g_ptc_regs;                 address chosen by the linker
 *     g_ptc.Instance = &g_ptc_regs;
 *     HAL_PTC_Enable(&g_ptc, 1);
 *     TEST_ASSERT_EQUAL_HEX32(expected, g_ptc_regs.IER);   read back what it wrote
 *
 * The register block is an ordinary variable, so no linker script change, no
 * reserved memory and no address translation is involved, and the carrier's
 * console at 0x40200000 is nowhere near it.
 *
 * Two things this buys that a bench does not. The `Sel`-conditioned writes land
 * in different trigger-pin registers, and a PTC channel's delay field is only
 * touched for channels 0..3 -- states a bench can stage only by wiring a
 * specific pin. And the values written are readable afterwards, so a case can
 * assert the exact word rather than infer it from behaviour.
 *
 * Two things it does not buy, so that no case here is read as more than it is:
 *
 *   * RAM does not distinguish a write-1-to-clear register from a plain one.
 *     The IRQ handler's `ICR |= (1 << channel)` clears an interrupt flag on
 *     silicon and sets a bit in RAM. The case below therefore pins the store the
 *     driver performs, not its effect on hardware.
 *   * Nothing here runs at hardware speed or waits for hardware. Any path that
 *     polls a status bit would spin forever unless the test pre-seeds that bit;
 *     bf0_hal_ptc.c has no such path, which is part of why it is the first
 *     target.
 */

#include <rtthread.h>
#include <stdint.h>

#include "unity.h"

#include "bf0_hal.h"

/*
 * bf0_hal_ptc.c compiles to nothing unless the module is enabled, and then the
 * cases below fail to link with nothing to say why. bf0_hal_conf_hcpu.h enables
 * it for HCPU unconditionally, so this only fires if that changes.
 */
#if !defined(HAL_PTC_ENABLED)
    #error "HAL_PTC_ENABLED is off: bf0_hal_ptc.c would compile to nothing."
#endif

/*
 * HAL_PTC_GetData is the module's only data accessor, and as the SDK shipped it
 * it did not compile: bf0_hal_ptc.h had it as
 *
 *     #define HAL_PTC_GetData(hptc) (*(hptc->Chn->TDR))
 *
 * while PTC_ChnTypeDef.TDR is a plain __IO uint32_t, so expanding it was
 *
 *     error: invalid type argument of unary '*' (have 'uint32_t')
 *
 * Nothing in the SDK called the macro, which is how it stayed that way. This
 * project is the first thing to expand it -- the case at the bottom of this file
 * is what found it -- and the header has since been fixed. The case stays: it is
 * the only one that reads a register back through the HAL rather than writing
 * through it, and re-breaking the macro would take it out again.
 */

/* The register block, and the handle that points at it. One per case, so a case
 * cannot be explained by what the previous one left behind. */
static PTC_TypeDef      g_ptc_regs;
static PTC_HandleTypeDef g_ptc;

/* Channel 2 sits at TCR3/TAR3/TDR3/RCR3: PTC_ChnTypeDef is four words, and the
 * channels start at TCR1. Spelled out because a case below asserts a value in
 * TAR3 while TAR1 stays zero. */
#define PTC_TEST_CHANNEL    2

/*
 * The per-case reset, and not setUp() itself.
 *
 * Unity calls one setUp()/tearDown() pair per case for the whole image --
 * UnityDefaultTestRun() reaches them by name -- and sifli_unity_runner.c defines
 * that pair weak so a project can own it. A project with two suites in one image
 * therefore cannot let each suite define the pair: both definitions are strong,
 * both reach the linker, and it stops at
 *
 *     test_ptc.c:100: multiple definition of `setUp';
 *     test_crc.c:56: first defined here
 *
 * so the pair is image-wide and lives in unity_main.c, which owns the other
 * image-wide concern already. What stays here, beside the state it touches, is
 * the reset that pair calls -- each suite keeps its own registers and handle
 * private, and neither file has to reach into the other's.
 */
void ptc_fixture_reset(void)
{
    rt_memset(&g_ptc_regs, 0, sizeof(g_ptc_regs));
    rt_memset(&g_ptc, 0, sizeof(g_ptc));
    g_ptc.Instance = &g_ptc_regs;
}

/**
 * @brief  Init resolves the channel's register block and fills its address/data.
 *
 * The channel pointer is arithmetic the driver does itself --
 * `(PTC_ChnTypeDef *)&Instance->TCR1 + Channel` -- so this asserts both that the
 * arithmetic is right and that it stays inside the block the test owns. Reading
 * TAR3/TDR3 back out of g_ptc_regs is the part that matters: it is what says the
 * writes went to RAM and not somewhere else.
 */
/* case: init_selects_the_channel_and_fills_it */
void test_ptc_init_selects_the_channel_and_fills_it(void)
{
    g_ptc.Init.Channel = PTC_TEST_CHANNEL;
    g_ptc.Init.Address = 0x12345678u;
    g_ptc.Init.data    = 0xA5A5A5A5u;

    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Init(&g_ptc));

    TEST_ASSERT_EQUAL_PTR((PTC_ChnTypeDef *)&g_ptc_regs.TCR1 + PTC_TEST_CHANNEL,
                          g_ptc.Chn);
    TEST_ASSERT_EQUAL_HEX32(0x12345678u, g_ptc_regs.TAR3);
    TEST_ASSERT_EQUAL_HEX32(0xA5A5A5A5u, g_ptc_regs.TDR3);
    TEST_ASSERT_EQUAL_INT(HAL_PTC_STATE_READY, g_ptc.State);
}

/**
 * @brief  A different channel reaches a different block.
 *
 * The same call with a different Channel. If the driver ignored Channel, or
 * always used the first one, the two would agree -- and this is the half of the
 * arithmetic the previous case cannot see.
 */
/* case: init_channel_zero_is_not_channel_two */
void test_ptc_init_with_channel_zero_leaves_channel_two_alone(void)
{
    g_ptc.Init.Channel = 0;
    g_ptc.Init.Address = 0x0000BEEFu;
    g_ptc.Init.data    = 0x00000001u;

    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Init(&g_ptc));

    TEST_ASSERT_EQUAL_HEX32(0x0000BEEFu, g_ptc_regs.TAR1);
    TEST_ASSERT_EQUAL_HEX32(0x00000000u, g_ptc_regs.TAR3);
}

/**
 * @brief  Enable programmes the interrupt bits and the channel control word.
 *
 * The channel's own TCR is assembled from five Init fields, each shifted to its
 * documented position -- the packing is the logic under test, and the positions
 * are not visible from the register value alone, so they are spelled out.
 */
/* case: enable_programmes_control_registers */
void test_ptc_enable_programmes_the_control_registers(void)
{
    g_ptc.Init.Channel     = PTC_TEST_CHANNEL;
    g_ptc.Init.Sel         = PTC_HCPU_PA31_0_A;
    g_ptc.Init.Operation   = 3;
    g_ptc.Init.Tripol      = 1;
    g_ptc.Init.RepEn       = 0;
    g_ptc.Init.RepTrig     = 0;
    g_ptc.Init.RepIRQ      = 0;
    g_ptc.Init.Delay       = 0;
    g_ptc.Init.Trigger_Pin = 5;

    /*
     * Init before Enable: the order the driver is called in on a board, and not
     * a formality. Enable writes the channel word through hptc->Chn, and Chn is
     * resolved by Init alone -- `(PTC_ChnTypeDef *)&Instance->TCR1 + Channel`.
     * Enable never derives it.
     *
     * Leave this out and the handle still holds the NULL setUp() memsets into
     * it, so `hptc->Chn->TCR = cr` lands at address 0 rather than in TCR3, and
     * the assertion below reads zero out of a register nothing wrote. The ICR
     * and IER assertions above pass either way -- the driver reaches those
     * through Instance, which the test does set -- so the omission is invisible
     * until a case reads back a channel register, which is why it is worth
     * saying out loud here.
     */
    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Init(&g_ptc));

    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Enable(&g_ptc, 1));

    /* The channel bit, plus the "transfer error" flag the driver always adds. */
    TEST_ASSERT_EQUAL_HEX32((1u << PTC_TEST_CHANNEL) | PTC_ICR_CTEIF,
                            g_ptc_regs.ICR);

    /* The channel's completion interrupt, and the same bit shifted up into the
     * error-interrupt half of the register. */
    TEST_ASSERT_EQUAL_HEX32((1u << PTC_TEST_CHANNEL) |
                            (1u << (PTC_TEST_CHANNEL + PTC_IER_TEIE_Pos)),
                            g_ptc_regs.IER);

    TEST_ASSERT_EQUAL_HEX32(PTC_HCPU_PA31_0_A |
                            (3u << PTC_TCR1_OP_Pos) |
                            (1u << PTC_TCR1_TRIGPOL_Pos),
                            g_ptc_regs.TCR3);

    /* Trigger pin 5 is in the low window, so GPIO31_0 takes it and the upper
     * window stays untouched. */
    TEST_ASSERT_EQUAL_HEX32(5u << PTC_GPIO31_0_SELA_Pos, g_ptc_regs.GPIO31_0);
    TEST_ASSERT_EQUAL_HEX32(0u, g_ptc_regs.GPIO63_32);

    TEST_ASSERT_EQUAL_INT(HAL_PTC_STATE_RUNNING, g_ptc.State);
}

/**
 * @brief  A pin in the 32..63 window is written as an offset, not as the pin.
 *
 * GPIO63_32 carries pins 32 onwards, so the driver stores `Trigger_Pin - 32`.
 * Getting that subtraction wrong is invisible on a bench until the wrong pin
 * fires, and it is exactly the kind of thing a case should pin.
 */
/* case: enable_offsets_the_upper_pin_window */
void test_ptc_enable_offsets_the_upper_pin_window(void)
{
    g_ptc.Init.Channel     = PTC_TEST_CHANNEL;
    g_ptc.Init.Sel         = PTC_HCPU_PA63_32_A;
    g_ptc.Init.Operation   = 1;
    g_ptc.Init.Trigger_Pin = 40;   /* 8 pins into the window */
    g_ptc.Init.Delay       = 0;

    /* Init first: Enable writes the channel word through hptc->Chn. See the note
     * in enable_programmes_control_registers. */
    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Init(&g_ptc));

    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Enable(&g_ptc, 1));

    TEST_ASSERT_EQUAL_HEX32(8u << PTC_GPIO63_32_SELA_Pos, g_ptc_regs.GPIO63_32);
    TEST_ASSERT_EQUAL_HEX32(0u, g_ptc_regs.GPIO31_0);
}

/**
 * @brief  The delay field only exists for the first four channels.
 *
 * DLY lives in the channel's repeat register, which only channels 0..3 have --
 * above that the driver must leave the field alone. Stated as two cases because
 * the value alone cannot say which rule produced it: 0x1234 in RCR for channel
 * 1, and RCR untouched for channel 4.
 */
/* case: delay_programmed_below_channel_four */
void test_ptc_delay_is_programmed_below_channel_four(void)
{
    g_ptc.Init.Channel = 1;
    g_ptc.Init.Delay   = 0x1234;
    g_ptc.Init.Sel     = PTC_HCPU_PA31_0_A;

    /* Init first: without it Chn is NULL and the delay below is written to
     * address 0 instead of RCR2. See enable_programmes_control_registers. */
    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Init(&g_ptc));

    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Enable(&g_ptc, 1));

    TEST_ASSERT_EQUAL_HEX32(0x1234u << PTC_RCR1_DLY_Pos, g_ptc_regs.RCR2);
}

/* case: delay_ignored_from_channel_four */
void test_ptc_delay_is_ignored_from_channel_four(void)
{
    g_ptc.Init.Channel = 4;
    g_ptc.Init.Delay   = 0x1234;
    g_ptc.Init.Sel     = PTC_HCPU_PA31_0_A;

    /* Init first, and here it is what gives the case its teeth: with Chn NULL,
     * a driver that had lost the `Channel < 4` guard would still write to
     * address 0 and RCR5 would still read zero, so the case would pass against a
     * driver it is supposed to catch. See enable_programmes_control_registers. */
    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Init(&g_ptc));

    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Enable(&g_ptc, 1));

    TEST_ASSERT_EQUAL_HEX32(0u, g_ptc_regs.RCR5);
}

/**
 * @brief  The repeat count is merged into RCR without disturbing the delay.
 *
 * MODIFY_REG reads, masks and writes back, so a repeat count set after a delay
 * has to leave the delay where it was. Writing the register instead of merging
 * into it would pass a case that only checked the repeat count -- this one
 * checks both fields of the same word.
 */
/* case: repeat_count_keeps_the_delay_field */
void test_ptc_repeat_count_keeps_the_delay_field(void)
{
    g_ptc.Init.Channel = 1;
    g_ptc.Init.Delay   = 0x1234;
    g_ptc.Init.RepEn   = 1;
    g_ptc.Init.Pen     = 0x155;
    g_ptc.Init.Sel     = PTC_HCPU_PA31_0_A;

    /* Init first: Enable merges the repeat count into the channel's RCR, which
     * it can only find through hptc->Chn. See
     * enable_programmes_control_registers. */
    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Init(&g_ptc));

    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Enable(&g_ptc, 1));

    TEST_ASSERT_EQUAL_HEX32((0x1234u << PTC_RCR1_DLY_Pos) |
                            (0x155u << PTC_RCR1_REP_Pos),
                            g_ptc_regs.RCR2);
}

/**
 * @brief  Disabling clears the interrupt enables and leaves the flags alone.
 *
 * The disable path is not the mirror image of the enable path: it clears IER and
 * touches neither ICR nor the channel's control word. Both halves are asserted,
 * because "cleared IER" alone would still pass if the driver also wiped the
 * pending flags.
 */
/* case: disable_clears_only_the_interrupt_enable */
void test_ptc_disable_clears_only_the_interrupt_enable(void)
{
    g_ptc.Init.Channel = PTC_TEST_CHANNEL;
    g_ptc.State        = HAL_PTC_STATE_RUNNING;

    g_ptc_regs.ICR = 0xFFFFFFFFu;
    g_ptc_regs.IER = 0xFFFFFFFFu;

    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Enable(&g_ptc, 0));

    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFu &
                            ~((1u << PTC_TEST_CHANNEL) |
                              (1u << (PTC_TEST_CHANNEL + PTC_IER_TEIE_Pos))),
                            g_ptc_regs.IER);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFu, g_ptc_regs.ICR);
    TEST_ASSERT_EQUAL_INT(HAL_PTC_STATE_READY, g_ptc.State);
}

/**
 * @brief  The handler stores the channel's clear bit.
 *
 * Read the caveat at the top of this file: ICR is write-1-to-clear on silicon, so
 * on hardware this store *clears* the channel's flag while in RAM it sets a bit.
 * The case pins the store -- one bit, the channel's own -- because that is the
 * part the driver is responsible for and the part a change would break. It does
 * not establish what the peripheral then does with it, and cannot here.
 */
/* case: irq_handler_writes_the_channel_clear_bit */
void test_ptc_irq_handler_writes_the_channel_clear_bit(void)
{
    g_ptc.Init.Channel = 3;
    g_ptc_regs.ICR     = 1u << 7;   /* some other channel's flag */

    HAL_PTC_IRQHandler(&g_ptc);

    TEST_ASSERT_EQUAL_HEX32((1u << 7) | (1u << 3), g_ptc_regs.ICR);
}

/**
 * @brief  The data accessor reads the channel the handle points at.
 *
 * The one case where the direction is reversed: the test writes the register and
 * the HAL reads it. That is what makes a primed register meaningful at all -- a
 * driver that polled a status bit would find what the test put there -- and it
 * checks the accessor's register offset, which no write-side case can.
 */
/* case: data_accessor_reads_the_channel_register */
void test_ptc_data_accessor_reads_the_channel_register(void)
{
    g_ptc.Init.Channel = PTC_TEST_CHANNEL;
    g_ptc.Init.data    = 0x11111111u;

    TEST_ASSERT_EQUAL_INT(HAL_OK, HAL_PTC_Init(&g_ptc));

    /* What the peripheral would have captured, as far as this carrier is
     * concerned: the same word the driver wrote, overwritten by the test. */
    g_ptc_regs.TDR3 = 0x0000BEEFu;

    TEST_ASSERT_EQUAL_HEX32(0x0000BEEFu, HAL_PTC_GetData((&g_ptc)));
}
