/**
  ******************************************************************************
  * @file   example_mailbox_mutex.c
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

#ifdef HAL_MAILBOX_MODULE_ENABLED

/* Example Description:
 *
 * 1. Demonstrate HPSYS mutex usage, lock success and lock fail
 * 2. Demonstrate LPSYS mutex usage, lock success and lock fail
 */

static MUTEX_LockCoreIdTypeDef lock_core_id_map[] =
{
    [CORE_ID_DEFAULT] = MUTEX_LOCK_CORE_INVALID,
    [CORE_ID_HCPU] = MUTEX_HCPU_LOCKED,
    [CORE_ID_LCPU] = MUTEX_LCPU_LOCKED,
    //[CORE_ID_BCPU] = MUTEX_BCPU_LOCKED,
};


static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static void test_mutex(MUTEX_CH_TypeDef *mutex)
{
    MUTEX_HandleTypeDef handle;
    MUTEX_LockCoreIdTypeDef core;
    MUTEX_LockCoreIdTypeDef exp_core_id;

    handle.Instance = mutex;

    /* lock first time, success */
    core = HAL_MAILBOX_Lock(&handle, 0);
    /* check whether succeed to lock */
    uassert_true_ret(core == MUTEX_UNLOCKED);
    /* lock second time, fail, it has been locked before */
    core = HAL_MAILBOX_Lock(&handle, 0);
    /* it's locked by current core */
    exp_core_id = lock_core_id_map[CORE_ID_CURRENT];
    LOG_I("core:%d,%d,%d,%x", core, exp_core_id, CORE_ID_CURRENT, mutex);
    uassert_true_ret(core == exp_core_id);

    /* unlock */
    HAL_MAILBOX_UnLock(&handle, 0);
    /* lock again, success */
    core = HAL_MAILBOX_Lock(&handle, 0);
    uassert_true_ret(core == MUTEX_UNLOCKED);
    core = HAL_MAILBOX_Lock(&handle, 0);
    LOG_I("core:%d,%d,%d,%x", core, exp_core_id, CORE_ID_CURRENT, mutex);
    exp_core_id = lock_core_id_map[CORE_ID_CURRENT];
    uassert_true_ret(core == exp_core_id);
    HAL_MAILBOX_UnLock(&handle, 0);
}

static void test_hmutex(void)
{
    /* test HPSYS mutex channel1 */
    test_mutex(HMUTEX_CH1);
    /* test HPSYS mutex channel2 */
    test_mutex(HMUTEX_CH2);
}


static void test_lmutex(void)
{
    /* test LPSYS mutex channel1 */
    test_mutex(LMUTEX_CH1);
}

static void example_mailbox_mutex(int argc, char **argv)
{
    test_hmutex();
    test_lmutex();
}

UTEST_TC_EXPORT(example_mailbox_mutex, "example_mailbox_mutex", NULL, NULL, 10);

#endif /*HAL_CRC_MODULE_ENABLED*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
