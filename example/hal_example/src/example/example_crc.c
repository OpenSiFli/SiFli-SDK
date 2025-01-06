/**
  ******************************************************************************
  * @file   example_crc.c
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


#include <stdlib.h>
#include <string.h>
#include "utest.h"
#include "bf0_hal.h"

#ifdef HAL_CRC_MODULE_ENABLED

/*
    This example demo:
        1. Use hal CRC funcions to calculate CRC value with different standard mode.
        2. use hal CRC funcions to calculate CRC value with customized initial value and polynomial.
*/
typedef struct
{
    int mode;
    int offset;
    int value;
} T_CRC_TEST;

#define TEST_SIZE 512

static uint8_t g_test_data[TEST_SIZE];
static T_CRC_TEST crc_test[CRC_MODE_NUM] = {{CRC_7_MMC, 0, 0x0d},
    {CRC_8, 0, 0x17},
    {CRC_8_ITU, 0, 0x42},
    {CRC_8_ROHC, 0, 0x88},
    {CRC_8_MAXIM, 0, 0x82},
    {CRC_16_IBM, 0, 0x4631},
    {CRC_16_MAXIM, 0, 0xb9ce},
    {CRC_16_USB, 0, 0x028f},
    {CRC_16_MODBUS, 0, 0xfd70},
    {CRC_16_CCITT, 0, 0x4997},
    {CRC_16_CCITT_FALSE, 0, 0x56ee},
    {CRC_16_X25, 0, 0x9a00},
    {CRC_16_XMODEM, 0, 0x40da},
    {CRC_16_DNP, 0, 0x6ed3},
    {CRC_32, 0, 0x1c613576},
    {CRC_32_MPEG_2, 0, 0xD08D7AD9}
};

static T_CRC_TEST crc_customized_test[CRC_MODE_NUM] = {{CRC_7_MMC, 0, 0x64},
    {CRC_8, 0, 0x93},
    {CRC_8_ITU, 0, 0xC6},
    {CRC_8_ROHC, 0, 0x87},
    {CRC_8_MAXIM, 0, 0x87},
    {CRC_16_IBM, 0, 0x01ED},
    {CRC_16_MAXIM, 0, 0xFE12},
    {CRC_16_USB, 0, 0xFE12},
    {CRC_16_MODBUS, 0, 0x01ED},
    {CRC_16_CCITT, 0, 0x01ED},
    {CRC_16_CCITT_FALSE, 0, 0xE8EB},
    {CRC_16_X25, 0, 0xFE12},
    {CRC_16_XMODEM, 0, 0xE8EB},
    {CRC_16_DNP, 0, 0xFE12},
    {CRC_32, 0, 0xDA5B668F},
    {CRC_32_MPEG_2, 0, 0x06FDE735}
};

/* CRC handler declaration */
CRC_HandleTypeDef   CrcHandle;

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static void testcase(int argc, char **argv)
{
    uint32_t result;

    for (int m = 0; m < TEST_SIZE; m++)
    {
        // initial test data
        g_test_data[m] = (uint8_t)(m & 0xff);
    }

    // initial CRC handle
    CrcHandle.Instance = CRC;

    if (HAL_CRC_Init(&CrcHandle) != HAL_OK)
    {
        uassert_true(RT_FALSE);
        return;
    }

    for (int m = 0; m < CRC_MODE_NUM; m++)
    {
        // use standard mode to initialize
        HAL_CRC_Setmode(&CrcHandle, crc_test[m].mode);
        // calculate CRC result
        result = HAL_CRC_Calculate(&CrcHandle, &(g_test_data[crc_test[m].offset]), TEST_SIZE - crc_test[m].offset);

        LOG_I("mode %d:  result=0x%X, expect=0x%X", m, result, crc_test[m].value);

        uassert_true_ret(result == crc_test[m].value);
    }


    for (int m = 0; m < CRC_MODE_NUM; m++)
    {
        // use customize initial value, polynomial with different mode
        uint32_t init = 0xFF;
        uint32_t poly = 0x1D;
        HAL_CRC_Setmode_Customized(&CrcHandle, init, poly, crc_customized_test[m].mode);
        // calculate CRC result
        result = HAL_CRC_Calculate(&CrcHandle, &(g_test_data[crc_customized_test[m].offset]), TEST_SIZE - crc_customized_test[m].offset);

        LOG_I("mode %d:  init=0xFF, poly=0x1D, result=0x%X, expect=0x%X", m, result, crc_customized_test[m].value, 0);

        uassert_true_ret(result == crc_customized_test[m].value);
    }
}

UTEST_TC_EXPORT(testcase, "example_crc", utest_tc_init, utest_tc_cleanup, 10);

#endif /*BSP_USING_HW_AES*/

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
