/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>

#include "sifli_unity.h"

/*
 * The case registry is generated, from the test sources this project compiles,
 * by tools/build/test_registry.py -- see the block in src/SConscript. What is
 * left here is only what a person writes.
 */

/*
 * The fixture Unity runs around every case.
 *
 * Unity reaches setUp()/tearDown() by name from UnityDefaultTestRun(), so an
 * image has exactly one pair no matter how many suites it holds, and
 * sifli_unity_runner.c defines the pair weak for a project to replace. Both
 * suites here need their register block zeroed before each case, so the pair
 * lives with the other image-wide concern (the autorun below) and calls the
 * reset each suite keeps beside its own state. A suite that defined setUp()
 * itself would be the second strong definition and the link would fail.
 */
extern void ptc_fixture_reset(void);
extern void crc_fixture_reset(void);

void setUp(void)
{
    ptc_fixture_reset();
    crc_fixture_reset();
}

void tearDown(void)
{
}

/**
 * @brief  Main program
 * @param  None
 * @retval 0 if success, otherwise failure number
 */
int main(void)
{
    rt_kprintf("\nSiFli SDK HAL tests running the real implementations (Unity)\n");
    rt_kprintf("  unity_list                  list the registered cases\n");
    rt_kprintf("  unity_run [suite|case|suite.case]\n");

    /*
     * Report every suite now if the build asked for it (TEST_FWK_AUTORUN),
     * so the host only has to read the console. A no-op otherwise, and the loop
     * below keeps the shell usable for hands-on work either way.
     */
    sifli_unity_autorun();

    /* Infinite loop */
    while (1)
    {
        /* Delay for 1000 ms to yield CPU time to other threads */
        rt_thread_mdelay(1000);
    }
    return 0;
}
