/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* suite: crc */

/*
 * The real HAL, registers in RAM -- the same mechanism as test_ptc.c, on a module
 * whose interesting parts are a table and a polling loop.
 *
 * Why this module, in two halves:
 *
 *   * HAL_CRC_Setmode writes three registers out of a 16-entry table indexed by
 *     the mode. The table is a transcription of published CRC parameters, so the
 *     assertions here are written against the published values rather than
 *     against the table -- a wrong constant is the bug this can catch, and the
 *     only way to catch it is to state independently what it should be. The
 *     polynomial-size field is checked against a rule derived from the polynomial
 *     itself (its width), which no copy of the table would catch either.
 *
 *   * HAL_CRC_Accumulate is the case that needs RAM rather than merely tolerates
 *     it. It feeds the data register in word-sized steps and waits for the
 *     peripheral after each one:
 *
 *         hcrc->Instance->DR = *(uint32_t *)(&pBuffer[index]);
 *         while (0 == (hcrc->Instance->SR & CRC_SR_DONE));
 *
 *     On a bench, the wait is what the hardware does. Here the test sets
 *     CRC_SR_DONE before the call and the loop reads it straight back, which is
 *     what makes a polling driver testable at all: the status bit is an input the
 *     test supplies, not something to wait for. What that does *not* reproduce is
 *     the wait -- a driver that polled the wrong bit, or never exited, behaves
 *     differently against real silicon, and this case cannot see that.
 *
 * The xor_res column of the table is exercised too, but only through
 * Accumulate's return value: Setmode does not program it anywhere.
 */

#include <rtthread.h>
#include <stdint.h>

#include "unity.h"

#include "bf0_hal.h"

#if !defined(HAL_CRC_MODULE_ENABLED)
    #error "HAL_CRC_MODULE_ENABLED is off: bf0_hal_crc.c would compile to nothing."
#endif

static CRC_TypeDef      g_crc_regs;
static CRC_HandleTypeDef g_crc;

/* The per-case reset. setUp()/tearDown() are image-wide and live in
 * unity_main.c -- see the comment on ptc_fixture_reset() in test_ptc.c for why a
 * second suite cannot define its own. */
void crc_fixture_reset(void)
{
    rt_memset(&g_crc_regs, 0, sizeof(g_crc_regs));
    rt_memset(&g_crc, 0, sizeof(g_crc));
    g_crc.Instance = &g_crc_regs;
}

/*
 * The published polynomial of each named mode, in the order the HAL's
 * HAL_CRC_Mode enumerates them. This is the specification the SDK's private
 * table has to agree with; a mode added to the enum without a row here fails the
 * static assertion below, so the two cannot drift apart quietly.
 */
static const uint32_t g_poly[] =
{
    0x09u,          /* CRC_7_MMC        */
    0x07u,          /* CRC_8            */
    0x07u,          /* CRC_8_ITU        */
    0x07u,          /* CRC_8_ROHC       */
    0x31u,          /* CRC_8_MAXIM      */
    0x8005u,        /* CRC_16_IBM       */
    0x8005u,        /* CRC_16_MAXIM     */
    0x8005u,        /* CRC_16_USB       */
    0x8005u,        /* CRC_16_MODBUS    */
    0x1021u,        /* CRC_16_CCITT     */
    0x1021u,        /* CRC_16_CCITT_FALSE */
    0x1021u,        /* CRC_16_X25       */
    0x1021u,        /* CRC_16_XMODEM    */
    0x3D65u,        /* CRC_16_DNP       */
    0x04C11DB7u,    /* CRC_32           */
    0x04C11DB7u,    /* CRC_32_MPEG_2    */
};

/* The two must stay the same length, or the loop below would silently stop
 * short of the modes it is supposed to cover. */
#define CRC_MODE_COUNT_STATIC_ASSERT(cond) \
    typedef char crc_mode_count_must_match[(cond) ? 1 : -1]

CRC_MODE_COUNT_STATIC_ASSERT(sizeof(g_poly) / sizeof(g_poly[0]) == CRC_MODE_NUM);

/**
 * @brief  The width the polynomial-size field has to state, derived from the
 *         polynomial itself rather than read off the table.
 */
static uint32_t expected_polysize(uint32_t poly)
{
    if (poly > 0xFFFFu)
    {
        return HAL_CRC_POLYSIZE_32;
    }
    if (poly > 0xFFu)
    {
        return HAL_CRC_POLYSIZE_16;
    }
    /* 0x09 is a degree-7 polynomial; every other 8-bit entry is degree 8. */
    return (poly == 0x09u) ? HAL_CRC_POLYSIZE_7 : HAL_CRC_POLYSIZE_8;
}

/**
 * @brief  Every mode programmes its published polynomial, and a size that
 *         matches it.
 *
 * Two assertions per mode, because they fail for different reasons: a wrong
 * polynomial is a bad transcription, while a right polynomial with the wrong
 * POLYSIZE field is a bad shift -- and the second computes the CRC over the
 * wrong number of bits without looking wrong in the register dump.
 */
/* case: setmode_programmes_the_published_polynomial */
void test_crc_setmode_programmes_the_published_polynomial(void)
{
    int mode;

    for (mode = 0; mode < CRC_MODE_NUM; mode++)
    {
        HAL_CRC_Setmode(&g_crc, (HAL_CRC_Mode)mode);

        TEST_ASSERT_EQUAL_HEX32(g_poly[mode], g_crc_regs.POL);
        TEST_ASSERT_EQUAL_HEX32(expected_polysize(g_poly[mode]),
                                (g_crc_regs.CR & CRC_CR_POLYSIZE_Msk) >>
                                CRC_CR_POLYSIZE_Pos);
        TEST_ASSERT_EQUAL_INT(mode, g_crc.Mode);
    }
}

/**
 * @brief  Setmode resets the calculation unit, having programmed it.
 *
 * The driver's last act is __HAL_CRC_DR_RESET, which sets CR's RESET bit. It is
 * asserted separately from the fields above because it is the one bit that is not
 * a table lookup, and leaving it out would let a CRC run against the previous
 * mode's residue.
 */
/* case: setmode_resets_the_calculation_unit */
void test_crc_setmode_resets_the_calculation_unit(void)
{
    HAL_CRC_Setmode(&g_crc, CRC_16_MODBUS);

    TEST_ASSERT_EQUAL_HEX32(CRC_CR_RESET, g_crc_regs.CR & CRC_CR_RESET);
}

/**
 * @brief  The initial value is the published one, per mode.
 *
 * Only the modes whose initial value is not a matter of variant are listed: a
 * CRC-16/CCITT has two defensible seeds and the HAL names one of them, so
 * asserting the other would be asserting a different specification rather than a
 * bug. These four have one answer each.
 */
/* case: setmode_programmes_the_published_initial_value */
void test_crc_setmode_programmes_the_published_initial_value(void)
{
    HAL_CRC_Setmode(&g_crc, CRC_16_MODBUS);         /* seeded all-ones   */
    TEST_ASSERT_EQUAL_HEX32(0x0000FFFFu, g_crc_regs.INIT);

    HAL_CRC_Setmode(&g_crc, CRC_16_XMODEM);         /* seeded zero       */
    TEST_ASSERT_EQUAL_HEX32(0x00000000u, g_crc_regs.INIT);

    HAL_CRC_Setmode(&g_crc, CRC_32);                /* seeded all-ones   */
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFu, g_crc_regs.INIT);

    HAL_CRC_Setmode(&g_crc, CRC_32_MPEG_2);         /* seeded all-ones   */
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFFu, g_crc_regs.INIT);
}

/**
 * @brief  Input and output reflection follow the mode, not a constant.
 *
 * CRC-32 (the Ethernet one) reflects both ends; CRC-32/MPEG-2 reflects neither.
 * They share a polynomial and differ only in these two bits, so a driver that
 * hard-coded either answer would pass every other case in this file.
 */
/* case: reflection_bits_follow_the_mode */
void test_crc_reflection_bits_follow_the_mode(void)
{
    HAL_CRC_Setmode(&g_crc, CRC_32);
    TEST_ASSERT_EQUAL_HEX32(1u, (g_crc_regs.CR & CRC_CR_REV_IN_Msk) >>
                            CRC_CR_REV_IN_Pos);
    TEST_ASSERT_EQUAL_HEX32(1u, (g_crc_regs.CR & CRC_CR_REV_OUT_Msk) >>
                            CRC_CR_REV_OUT_Pos);

    HAL_CRC_Setmode(&g_crc, CRC_32_MPEG_2);
    TEST_ASSERT_EQUAL_HEX32(0u, (g_crc_regs.CR & CRC_CR_REV_IN_Msk) >>
                            CRC_CR_REV_IN_Pos);
    TEST_ASSERT_EQUAL_HEX32(0u, (g_crc_regs.CR & CRC_CR_REV_OUT_Msk) >>
                            CRC_CR_REV_OUT_Pos);
}

/* A word for Accumulate to feed. Aligned on purpose: the driver takes a separate
 * path for a buffer whose first byte is not word aligned, and this case is about
 * the main loop. */
static uint32_t g_word;

/**
 * @brief  Accumulate feeds the data register and returns the residue XORed with
 *         the mode's constant.
 *
 * The status bit is primed rather than waited for -- see the note at the top of
 * this file. With SR reading back as the driver expects, one four-byte buffer
 * goes through the word loop exactly once, so the data register holds precisely
 * what was handed in, and the return value is that word XORed with CRC_8/ITU's
 * 0x55. Both are values the driver computes; neither can be produced by a stub.
 *
 * State and Lock are asserted as well, because they are the driver's bookkeeping
 * and are invisible in the register block: a case that only read the registers
 * back would pass on a driver that never released the handle.
 */
/* case: accumulate_feeds_the_data_register */
void test_crc_accumulate_feeds_the_data_register_and_xors_the_result(void)
{
    g_word = 0xDEADBEEFu;
    g_crc.Mode = CRC_8_ITU;

    /* What the peripheral would have set once the previous word was consumed. */
    g_crc_regs.SR = CRC_SR_DONE;

    TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFu ^ 0x55u,
                            HAL_CRC_Accumulate(&g_crc, (uint8_t *)&g_word,
                                    sizeof(g_word)));

    TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFu, g_crc_regs.DR);

    /* The driver switches the unit to word-sized data before the main loop. */
    TEST_ASSERT_EQUAL_HEX32(HAL_CRC_DATASIZE_32,
                            (g_crc_regs.CR & CRC_CR_DATASIZE_Msk) >>
                            CRC_CR_DATASIZE_Pos);

    /* Never written by the driver: the loop only reads it, which is what made the
     * priming hold for the whole call. */
    TEST_ASSERT_EQUAL_HEX32(CRC_SR_DONE, g_crc_regs.SR);

    TEST_ASSERT_EQUAL_INT(HAL_CRC_STATE_READY, g_crc.State);
    TEST_ASSERT_EQUAL_INT(HAL_UNLOCKED, g_crc.Lock);
}
