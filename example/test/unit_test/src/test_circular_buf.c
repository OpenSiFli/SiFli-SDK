/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "unity.h"
#include "circular_buf.h"

#define POOL_SIZE   16

static uint8_t pool[POOL_SIZE];
static uint8_t scratch[POOL_SIZE];
static uint8_t actual[POOL_SIZE];
static struct circular_buf cb;

static void buf_setup(int16_t size)
{
    memset(&cb, 0, sizeof(cb));
    memset(pool, 0, sizeof(pool));
    circular_buf_init(&cb, pool, size);
}

static void fill_sequence(uint8_t *dst, uint8_t first, size_t count)
{
    size_t i;

    for (i = 0; i < count; i++)
    {
        dst[i] = (uint8_t)(first + i);
    }
}

/* Ordinary use: write a block, read it back, and check the accounting at both
 * ends. Reading an empty buffer, and asking for more than is stored, are the
 * two edges of this. */
void test_circular_buf_fill_and_drain(void)
{
    uint8_t ch = 0;

    buf_setup(POOL_SIZE);

    TEST_ASSERT_EQUAL_INT(POOL_SIZE, circular_buf_get_size(&cb));
    TEST_ASSERT_EQUAL_INT(0, circular_buf_data_len(&cb));
    TEST_ASSERT_EQUAL_INT(POOL_SIZE, circular_buf_space_len(&cb));

    /* Empty: nothing to hand out, and the caller's buffer is left alone. */
    memset(actual, 0xAA, sizeof(actual));
    TEST_ASSERT_EQUAL_INT(0, circular_buf_get(&cb, actual, 4));
    TEST_ASSERT_EQUAL_HEX8(0xAA, actual[0]);
    TEST_ASSERT_EQUAL_INT(0, circular_buf_getchar(&cb, &ch));

    fill_sequence(scratch, 0x01, 10);
    TEST_ASSERT_EQUAL_INT(10, circular_buf_put(&cb, scratch, 10));
    TEST_ASSERT_EQUAL_INT(10, circular_buf_data_len(&cb));
    TEST_ASSERT_EQUAL_INT(POOL_SIZE - 10, circular_buf_space_len(&cb));

    /* Asking for more than is stored returns only what is there. */
    memset(actual, 0, sizeof(actual));
    TEST_ASSERT_EQUAL_INT(10, circular_buf_get(&cb, actual, POOL_SIZE));
    TEST_ASSERT_EQUAL_MEMORY(scratch, actual, 10);
    TEST_ASSERT_EQUAL_INT(0, circular_buf_data_len(&cb));
    TEST_ASSERT_EQUAL_INT(POOL_SIZE, circular_buf_space_len(&cb));
}

/* The interesting case for a ring: indices that run off the end of the pool
 * and continue at the start. Both the block and the byte-at-a-time paths are
 * exercised, because they wrap on different conditions. */
void test_circular_buf_wraparound(void)
{
    uint8_t expected[10];
    uint8_t ch = 0;
    int i;

    /* Block path: advance read and write to 12 of the 16 bytes, so the next
     * write has to split -- 4 bytes at the end of the pool, 4 at the start. */
    buf_setup(POOL_SIZE);

    fill_sequence(scratch, 0xA0, 12);
    TEST_ASSERT_EQUAL_INT(12, circular_buf_put(&cb, scratch, 12));
    memset(actual, 0, sizeof(actual));
    TEST_ASSERT_EQUAL_INT(12, circular_buf_get(&cb, actual, 12));
    TEST_ASSERT_EQUAL_MEMORY(scratch, actual, 12);
    TEST_ASSERT_EQUAL_INT(0, circular_buf_data_len(&cb));

    fill_sequence(expected, 0xB0, 8);
    TEST_ASSERT_EQUAL_INT(8, circular_buf_put(&cb, expected, 8));
    TEST_ASSERT_EQUAL_INT(8, circular_buf_data_len(&cb));

    memset(actual, 0, sizeof(actual));
    TEST_ASSERT_EQUAL_INT(8, circular_buf_get(&cb, actual, 8));
    TEST_ASSERT_EQUAL_MEMORY(expected, actual, 8);
    TEST_ASSERT_EQUAL_INT(0, circular_buf_data_len(&cb));
    TEST_ASSERT_EQUAL_INT(POOL_SIZE, circular_buf_space_len(&cb));

    /* Byte path: fill, drain part of it so the indices sit near the end, then
     * write past the end of the pool one byte at a time. */
    buf_setup(POOL_SIZE);

    for (i = 0; i < 14; i++)
    {
        TEST_ASSERT_EQUAL_INT(1, circular_buf_putchar(&cb, (uint8_t)(0xC0 + i)));
    }
    for (i = 0; i < 10; i++)
    {
        TEST_ASSERT_EQUAL_INT(1, circular_buf_getchar(&cb, &ch));
        TEST_ASSERT_EQUAL_HEX8(0xC0 + i, ch);
    }
    TEST_ASSERT_EQUAL_INT(4, circular_buf_data_len(&cb));

    /* These six wrap: two at the end of the pool, four at the start. */
    for (i = 0; i < 6; i++)
    {
        TEST_ASSERT_EQUAL_INT(1, circular_buf_putchar(&cb, (uint8_t)(0xD0 + i)));
    }
    TEST_ASSERT_EQUAL_INT(10, circular_buf_data_len(&cb));

    /* What comes back must be the tail that was left plus the wrapped bytes,
     * in the order they were written. */
    fill_sequence(expected, 0xCA, 4);
    fill_sequence(expected + 4, 0xD0, 6);
    memset(actual, 0, sizeof(actual));
    for (i = 0; i < 10; i++)
    {
        TEST_ASSERT_EQUAL_INT(1, circular_buf_getchar(&cb, &ch));
        TEST_ASSERT_EQUAL_HEX8(expected[i], ch);
    }
    TEST_ASSERT_EQUAL_INT(0, circular_buf_data_len(&cb));
}

/* The two boundaries the mirror bit exists for: completely full, and
 * completely empty. A full buffer refuses more data (unless forced), an empty
 * one refuses to give any. */
void test_circular_buf_full_and_empty(void)
{
    uint8_t ch = 0;

    /* The pool size is rounded down to a multiple of four. */
    buf_setup(13);
    TEST_ASSERT_EQUAL_INT(12, circular_buf_get_size(&cb));
    TEST_ASSERT_EQUAL_INT(12, circular_buf_space_len(&cb));

    buf_setup(POOL_SIZE);

    fill_sequence(scratch, 0x10, POOL_SIZE);
    TEST_ASSERT_EQUAL_INT(POOL_SIZE, circular_buf_put(&cb, scratch, POOL_SIZE));
    TEST_ASSERT_EQUAL_INT(POOL_SIZE, circular_buf_data_len(&cb));
    TEST_ASSERT_EQUAL_INT(0, circular_buf_space_len(&cb));

    /* Full is not empty: with the same read and write index but different
     * mirrors, the buffer holds a full pool. Further writes are refused. */
    TEST_ASSERT_EQUAL_INT(0, circular_buf_put(&cb, scratch, 1));
    TEST_ASSERT_EQUAL_INT(0, circular_buf_putchar(&cb, 0xFF));
    TEST_ASSERT_EQUAL_INT(POOL_SIZE, circular_buf_data_len(&cb));

    /* putchar_force keeps the length at capacity by dropping the oldest byte. */
    TEST_ASSERT_EQUAL_INT(1, circular_buf_putchar_force(&cb, 0xEE));
    TEST_ASSERT_EQUAL_INT(POOL_SIZE, circular_buf_data_len(&cb));

    memset(actual, 0, sizeof(actual));
    TEST_ASSERT_EQUAL_INT(POOL_SIZE, circular_buf_get(&cb, actual, POOL_SIZE));
    /* Everything but the dropped first byte is still in order ... */
    TEST_ASSERT_EQUAL_MEMORY(scratch + 1, actual, POOL_SIZE - 1);
    /* ... and the forced byte took the freed slot at the end. */
    TEST_ASSERT_EQUAL_HEX8(0xEE, actual[POOL_SIZE - 1]);

    /* Drained back to empty. */
    TEST_ASSERT_EQUAL_INT(0, circular_buf_data_len(&cb));
    TEST_ASSERT_EQUAL_INT(POOL_SIZE, circular_buf_space_len(&cb));
    TEST_ASSERT_EQUAL_INT(0, circular_buf_get(&cb, actual, 1));
    TEST_ASSERT_EQUAL_INT(0, circular_buf_getchar(&cb, &ch));

    /* reset() empties the buffer without touching the pool itself. */
    fill_sequence(scratch, 0x20, 4);
    TEST_ASSERT_EQUAL_INT(4, circular_buf_put(&cb, scratch, 4));
    TEST_ASSERT_EQUAL_INT(4, circular_buf_data_len(&cb));
    circular_buf_reset(&cb);
    TEST_ASSERT_EQUAL_INT(0, circular_buf_data_len(&cb));
    TEST_ASSERT_EQUAL_INT(POOL_SIZE, circular_buf_space_len(&cb));
}
