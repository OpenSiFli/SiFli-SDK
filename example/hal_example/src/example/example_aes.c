/**
  ******************************************************************************
  * @file   example_aes.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
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

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "utest.h"
#include "bf0_hal.h"
#include "tc_utils.h"

#ifdef HAL_AES_MODULE_ENABLED

/*
    This example demo:
        1. Generate random content in buffer g_aes_input_data
        2. Encrypt g_aes_input_data with AES CTR algorithm to g_aes_output_data
        3. Decrypt g_aes_output_data using AES CTR algorithm with same key to g_aes_input_data2
        4. Content of g_aes_input_data and g_aes_input_data2 should match
*/

#define BUFFER_SIZE    256

ALIGN(4)
static uint8_t g_aes_input_data[BUFFER_SIZE];
ALIGN(4)
static uint8_t g_aes_input_data2[BUFFER_SIZE];
//static uint8_t *g_aes_input_data2=(uint8_t *)HPSYS_ITCM_BASE;

ALIGN(4)
static uint8_t g_aes_output_data[BUFFER_SIZE];
ALIGN(4)
static uint8_t g_temp[BUFFER_SIZE];

ALIGN(4)
static uint8_t g_key[] =
{
    0x3D, 0xA5, 0xA4, 0x98, 0x6E, 0x90, 0xA7, 0x90,
    0x1D, 0x97, 0x69, 0xAA, 0xF0, 0xDF, 0x32, 0xE4,
    0x55, 0xE9, 0xFC, 0xD6, 0x75, 0x60, 0xBD, 0x33,
    0x5E, 0x20, 0xD0, 0x78, 0x47, 0xF7, 0x8C, 0x4D,
};

ALIGN(4)
static uint8_t g_iv[] =
{
    0xf0, 0xd7, 0x77, 0x7f, 0x61, 0x6f, 0x7c, 0x89,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static void aes_async_cbk(void)
{
    LOG_I("Aes/Hash complete\n");
}

static void testcase(int argc, char **argv)
{
    HAL_StatusTypeDef status;

    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        g_aes_input_data[i] = (rand() & 0xff);
    }

    HAL_AES_init((uint32_t *)g_key, 32, (uint32_t *)g_iv, AES_MODE_CTR);    // Key and IV must be 4 bytes aligned.


    NVIC_EnableIRQ(AES_IRQn);                                               // Enable interrupt
    HAL_NVIC_SetPriority(AES_IRQn, 5, 0);

    HAL_AES_Regist_IT_cb(aes_async_cbk);
    status = HAL_AES_run_IT(AES_ENC, g_aes_input_data, g_aes_output_data, BUFFER_SIZE);
    if (HAL_OK != status) // Encryption Asyn
    {
        LOG_E("AES run (async mode) error\n");
        uassert_true(status == HAL_OK);
    }
    rt_thread_mdelay(10);

    status = HAL_AES_run(AES_DEC, g_aes_output_data, g_aes_input_data2, BUFFER_SIZE);
    if (HAL_OK != status) // Decryption sync
    {
        LOG_E("AES run (sync mode) error\n");
        uassert_true(status == HAL_OK);
    }
    rt_thread_mdelay(10);

    if (memcmp(g_aes_input_data, g_aes_input_data2, BUFFER_SIZE) == 0)       // Decryption result should match with original input
        LOG_I("AES enc/dec matched.");
    else
    {
        LOG_I("AES Error.");
        uassert_true(0);
    }
}
UTEST_TC_EXPORT(testcase, "example_aes", utest_tc_init, utest_tc_cleanup, 10);

#endif

#ifdef HAL_HASH_MODULE_ENABLED
static void testcase_hash(int argc, char **argv)
{
    HAL_StatusTypeDef status;
    uint8_t algo = HASH_ALGO_SHA1;
    int result_len = 20;
    int i;

    if (argc == 0)
    {
        algo = HASH_ALGO_SHA256;
        result_len = 32;
    }
    else if (strcmp(argv[0], "sha1") == 0)
    {
        algo = HASH_ALGO_SHA1;
        result_len = 20;
    }
    else if (strcmp(argv[0], "sha224") == 0)
    {
        algo = HASH_ALGO_SHA224;
        result_len = 32;
    }
    else if (strcmp(argv[0], "sha256") == 0)
    {
        algo = HASH_ALGO_SHA256;
        result_len = 32;
    }
    else if (strcmp(argv[0], "sm3") == 0)
    {
        algo = HASH_ALGO_SM3;
        result_len = 32;
    }
    else
        uassert_true(0);

    {
        int len, multi;

        if (argc < 2)
        {
            memcpy(g_aes_input_data, "0123456789abcdef", 16);
            len = 16;
        }
        // Get data for testing
        else if (strlen(argv[1]) > 2 && argv[1][0] == '0' && (argv[1][1] == 'x' || argv[1][1] == 'X'))
        {
            len = hex2data(argv[1], g_aes_input_data, (uint8_t)(BUFFER_SIZE - 1));
        }
        else
        {
            strcpy((char *)g_aes_input_data, argv[1]);
            len = strlen(argv[1]);
        }

        // Multiple data length for larger data
        multi = (argc > 2) ? atoi(argv[2]) : 0;
        uassert_true(multi * len < BUFFER_SIZE);
        for (i = 0; i < multi; i++)
            memcpy(&g_aes_input_data[(i + 1)*len], &g_aes_input_data[0], len);
        len *= (multi + 1);

        // Test for interrupt mode, finish in 1 run
        {
            HAL_HASH_init(NULL, algo, 0);

            // Enable interrupt
            NVIC_EnableIRQ(AES_IRQn);
            HAL_NVIC_SetPriority(AES_IRQn, 5, 0);

            HAL_HASH_Regist_IT_cb(aes_async_cbk);
            status = HAL_HASH_run_IT(g_aes_input_data, len, 1);
            if (HAL_OK != status) // Encryption Asyn
            {
                LOG_E("AES run (async mode) error\n");
                uassert_true(status == HAL_OK);
            }
            rt_thread_mdelay(10);
            uassert_true(status == HAL_OK);
            HAL_HASH_result(g_temp);
            HAL_DBG_print_data((char *)g_temp, 0, result_len);
        }

        // Test for non-interrupt mode, finish in 1 run
        {
            HAL_HASH_init(NULL, algo, 0);
            status = HAL_HASH_run(g_aes_input_data, len, 1);
            if (HAL_OK != status) // Decryption sync
            {
                LOG_E("AES run (sync mode) error\n");
                uassert_true(status == HAL_OK);
            }
            rt_thread_mdelay(10);
            uassert_true(status == HAL_OK);
            HAL_HASH_result(g_temp);
            HAL_DBG_print_data((char *)g_temp, 0, result_len);
        }

        // Test for non-interrupt mode, finish in multiple runs
        {
#define SPLIT_THRESHOLD 64
            for (i = 0; i < len; i += SPLIT_THRESHOLD)
            {
                if (i == 0)
                    HAL_HASH_init(NULL, algo, 0);
                else
                    // Resume from last hash result.
                    HAL_HASH_init((uint32_t *)g_temp, algo, i);
                if (i + SPLIT_THRESHOLD < len)
                {
                    LOG_I("AES run %d\n", i);
                    status = HAL_HASH_run(&(g_aes_input_data[i]), SPLIT_THRESHOLD, 0);
                }
                else
                {
                    LOG_I("AES run continue %d\n", i);
                    status = HAL_HASH_run(&(g_aes_input_data[i]), len - i, 1);
                }
                if (HAL_OK != status) // Decryption sync
                {
                    LOG_E("AES run (sync mode) error\n");
                    uassert_true(status == HAL_OK);
                }
                HAL_HASH_result(g_temp);
            }
            HAL_DBG_print_data((char *)g_temp, 0, result_len);
        }
    }
}

UTEST_TC_EXPORT(testcase_hash, "example_hash", utest_tc_init, utest_tc_cleanup, 10);

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
