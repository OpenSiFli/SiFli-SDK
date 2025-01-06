/**
  ******************************************************************************
  * @file   example_atomic.c
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



static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}


#define ASSIGNED_VALUE 0x5AA5A5A5
#define ASSIGNED_VALUE_H 0x5AA5
#define ASSIGNED_VALUE_B 0xA5

static void testcase(int argc, char **argv)
{
    volatile uint32_t data_l;
    volatile uint16_t data_h;
    volatile uint8_t data_b;

    uint32_t value = (ASSIGNED_VALUE >> 1);
    uint16_t value_h = (uint16_t)(ASSIGNED_VALUE_H >> 1);
    uint8_t value_b = (uint8_t)(ASSIGNED_VALUE_B >> 1);
    rt_kprintf("value=%x, value_h=%x, value_b=%x\n", value, (uint32_t)value_h, (uint32_t)value_b);

#ifdef HAL_ATOMIC_FIX_ENABLED
    // Enable support for atomic instruction on PRO
    hwp_hpsys_cfg->SYS_RSVD |= 3;
#endif /*HAL_ATOMIC_FIX_ENABLED*/

    data_l = __LDREXW(&value);
    data_l = (ASSIGNED_VALUE >> 1);
    if (__STREXW(ASSIGNED_VALUE, &data_l) == 0)
        rt_kprintf("Write success\n");
    else
        rt_kprintf("Write fail\n");
    __utest_assert(data_l == ASSIGNED_VALUE, "STREXW value");

    data_h = __LDREXH(&value_h);
    if (__STREXH(ASSIGNED_VALUE_H, &data_h) == 0)
        rt_kprintf("Write half success\n");
    else
        rt_kprintf("Write half fail\n");
    __utest_assert(data_h == ASSIGNED_VALUE_H, "STREXH value");

    data_b = __LDREXB(&value_b);
    if (__STREXB(ASSIGNED_VALUE_H, &data_b) == 0)
        rt_kprintf("Write byte success\n");
    else
        rt_kprintf("Write byte fail\n");
    __utest_assert(data_b == ASSIGNED_VALUE_B, "STREXB value");

    rt_kprintf("value=%x, value_h=%x, value_b=%x\n", data_l, (uint32_t)data_h, (uint32_t)data_b);

    __CLREX();

}

UTEST_TC_EXPORT(testcase, "example_atomic", utest_tc_init, utest_tc_cleanup, 10);


/* Example to show to how to perform an exclusive read-modify-write using Load-Exclusive and Store-Exclusive instructions */
static void testcase_ex_readwrite(int argc, char **argv)
{
    uint32_t test_var = 0xabcd;
    uint32_t temp;

    temp = HAL_LOCK_Read32(&test_var);
    temp++;
    /* no exception occurs in between, write succeed */
    if (HAL_LOCK_Write32(&test_var, temp))
    {
        rt_kprintf("write1 succ:%x\n", test_var);
    }
    else
    {
        rt_kprintf("write1 fail\n");
        uassert_true_ret(false);
    }

    temp = HAL_LOCK_Read32(&test_var);
    temp++;
    /* wait for systick interrupt, it would result in write fail as exception occurs */
    rt_thread_mdelay(2);
    if (HAL_LOCK_Write32(&test_var, temp))
    {
        rt_kprintf("write2 succ:%x\n", test_var);
        uassert_true_ret(false);
    }
    else
    {
        rt_kprintf("write2 fail, try again\n");
    }

    /* try the entire read-modify-write sequence again */
    temp = HAL_LOCK_Read32(&test_var);
    temp++;
    if (HAL_LOCK_Write32(&test_var, temp))
    {
        rt_kprintf("write3 succ:%x\n", test_var);
    }
    else
    {
        rt_kprintf("write3 fail\n");
        uassert_true_ret(false);
    }

    temp++;
    /* write fails as no read precedes */
    if (HAL_LOCK_Write32(&test_var, temp))
    {
        rt_kprintf("write4 succ:%x\n", test_var);
        uassert_true_ret(false);
    }
    else
    {
        rt_kprintf("write4 fail\n");
    }
}
UTEST_TC_EXPORT(testcase_ex_readwrite, "example_ex_readwrite", NULL, NULL, 10);

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
