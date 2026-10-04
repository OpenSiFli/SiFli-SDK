/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>

/*
 * The case registry used to be in this file. It is generated now, from the test
 * sources this project compiles, by tools/build/test_registry.py -- see the
 * block in src/SConscript. What is left here is only what a person writes.
 */

/**
 * @brief  Main program
 * @param  None
 * @retval 0 if success, otherwise failure number
 */
int main(void)
{
    /* The host drives the run over the shell; nothing happens here on its own. */
    rt_kprintf("\nSiFli SDK unit tests (Unity)\n");
    rt_kprintf("  unity_list                  list the registered cases\n");
    rt_kprintf("  unity_run [suite|case|suite.case]\n");

    /* Infinite loop */
    while (1)
    {
        /* Delay for 1000 ms to yield CPU time to other threads */
        rt_thread_mdelay(1000);
    }
    return 0;
}
