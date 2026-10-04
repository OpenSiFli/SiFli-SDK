/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include "unity.h"
#include "lz4.h"

/* Highly compressible input: the same 16 byte pattern repeated. */
#define DATA_SIZE       1024
/* LZ4 guarantees success when the destination is at least this large, and
 * for incompressible input it is this large a destination that is needed. */
#define COMPRESSED_MAX  LZ4_COMPRESSBOUND(DATA_SIZE)

static char compressible[DATA_SIZE];
static char incompressible[DATA_SIZE];
static char compressed[COMPRESSED_MAX];
static char restored[DATA_SIZE];

static void fill_compressible(void)
{
    int i;

    for (i = 0; i < DATA_SIZE; i++)
    {
        compressible[i] = (char)('A' + (i % 16));
    }
}

/* xorshift32: deterministic, and for LZ4's purposes incompressible. Every
 * block window differs, so there is no match to encode. */
static void fill_incompressible(void)
{
    uint32_t state = 0x12345678u;
    int i;

    for (i = 0; i < DATA_SIZE; i++)
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        incompressible[i] = (char)(state & 0xFF);
    }
}

/* Repeating data compresses well and must come back byte for byte. */
void test_lz4_roundtrip(void)
{
    int compressed_size;
    int restored_size;

    fill_compressible();

    compressed_size = LZ4_compress_default(compressible, compressed,
                                           DATA_SIZE, (int)sizeof(compressed));
    TEST_ASSERT_GREATER_THAN_INT(0, compressed_size);
    /* The point of the case: a compressible block really does shrink. */
    TEST_ASSERT_LESS_THAN_INT(DATA_SIZE, compressed_size);

    restored_size = LZ4_decompress_safe(compressed, restored,
                                        compressed_size, (int)sizeof(restored));
    TEST_ASSERT_EQUAL_INT(DATA_SIZE, restored_size);
    TEST_ASSERT_EQUAL_MEMORY(compressible, restored, DATA_SIZE);
}

/* The other end of the scale: random data has no match to encode, so the
 * compressed block is not smaller than the input -- and LZ4_compressBound()
 * is the size that must still be enough for it. */
void test_lz4_incompressible(void)
{
    int compressed_size;
    int restored_size;

    fill_incompressible();

    compressed_size = LZ4_compress_default(incompressible, compressed,
                                           DATA_SIZE, (int)sizeof(compressed));
    TEST_ASSERT_GREATER_THAN_INT(0, compressed_size);

    /* The documented worst case bound must hold for any successful result. */
    TEST_ASSERT_TRUE(LZ4_compressBound(DATA_SIZE) >= compressed_size);
    /* And nothing was gained: the output stays close to, in fact slightly
     * above, the input size. */
    TEST_ASSERT_TRUE(compressed_size > DATA_SIZE / 2);

    restored_size = LZ4_decompress_safe(compressed, restored,
                                        compressed_size, (int)sizeof(restored));
    TEST_ASSERT_EQUAL_INT(DATA_SIZE, restored_size);
    TEST_ASSERT_EQUAL_MEMORY(incompressible, restored, DATA_SIZE);
}

/* Empty input, and destinations too small to hold the result -- the encoder
 * must refuse rather than write past the buffer, and the "safe" decoder must
 * report failure rather than overflow. */
void test_lz4_boundaries(void)
{
    int compressed_size;
    int restored_size;

    fill_compressible();

    /* Zero bytes in: the encoder still emits the one byte empty-sequence
     * token, and the decoder turns it back into zero bytes. */
    compressed_size = LZ4_compress_default(compressible, compressed,
                                           0, (int)sizeof(compressed));
    TEST_ASSERT_GREATER_THAN_INT(0, compressed_size);

    restored_size = LZ4_decompress_safe(compressed, restored,
                                        compressed_size, (int)sizeof(restored));
    TEST_ASSERT_EQUAL_INT(0, restored_size);

    /* A full block cannot possibly fit in four bytes: the encoder reports
     * failure instead of overflowing the destination. */
    compressed_size = LZ4_compress_default(compressible, compressed,
                                           DATA_SIZE, 4);
    TEST_ASSERT_EQUAL_INT(0, compressed_size);

    /* And the mirror image: a valid block whose content does not fit the
     * destination the caller offers. */
    compressed_size = LZ4_compress_default(compressible, compressed,
                                           DATA_SIZE, (int)sizeof(compressed));
    TEST_ASSERT_GREATER_THAN_INT(0, compressed_size);

    restored_size = LZ4_decompress_safe(compressed, restored,
                                        compressed_size, 4);
    TEST_ASSERT_LESS_THAN_INT(0, restored_size);
}
