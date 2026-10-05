/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>

#include "sifli_unity.h"
#include "unity.h"

/**
 * @addtogroup test_fwk
 * @{
 */

/**
 * Unity calls these around every case in UnityDefaultTestRun(). A case is a
 * plain function here, with no per-case fixture support, so there is nothing
 * to do. They are weak so a project that does want fixtures can override them
 * without patching this component.
 */
RT_WEAK void setUp(void)
{
}

RT_WEAK void tearDown(void)
{
}

/**
 * Unity's end-of-line, as configured by unity_config.h.
 *
 * rt_kprintf() sets RT_DEVICE_FLAG_STREAM around its write, and the serial
 * layer expands a '\n' into "\r\n" while that flag is set. The host's result
 * line ends at the newline, so the '\r' would leave every PASS line without a
 * recognizable end and the case would vanish from the report. Write the single
 * LF with the flag cleared instead. Saving and restoring the flag mirrors what
 * rt_kprintf() itself does.
 */
void sifli_unity_put_eol(void)
{
    rt_device_t console;
    rt_uint16_t saved_flag;

    console = rt_console_get_device();
    if (console == RT_NULL)
    {
        /* Nothing to steer: rt_kprintf() would not reach a console either. */
        rt_kprintf("\n");
        return;
    }

    saved_flag = console->open_flag;
    console->open_flag &= ~RT_DEVICE_FLAG_STREAM;
    rt_device_write(console, 0, "\n", 1);
    console->open_flag = saved_flag;
}

/** Number of cases the registry actually holds (terminator excluded). */
static rt_size_t sifli_unity_case_count(void)
{
    rt_size_t count = 0;

    while (sifli_unity_cases[count].func != RT_NULL)
    {
        count++;
    }
    return count;
}

/**
 * Does @p tc answer to @p filter?
 *
 * The filter is a suite name, a case name, or "suite.case"; a case name or a
 * suite name alone deliberately selects more than one entry (all cases of a
 * suite, or a case name shared by several suites).
 */
static rt_bool_t sifli_unity_case_matches(const sifli_unity_case_t *tc,
        const char          *filter)
{
    char qualified[64];

    if (filter == RT_NULL || filter[0] == '\0')
    {
        return RT_TRUE;
    }
    if (tc->suite != RT_NULL && rt_strcmp(filter, tc->suite) == 0)
    {
        return RT_TRUE;
    }
    if (tc->name != RT_NULL && rt_strcmp(filter, tc->name) == 0)
    {
        return RT_TRUE;
    }

    rt_snprintf(qualified, sizeof(qualified), "%s.%s",
                (tc->suite != RT_NULL) ? tc->suite : "",
                (tc->name != RT_NULL) ? tc->name : "");
    return (rt_strcmp(filter, qualified) == 0) ? RT_TRUE : RT_FALSE;
}

int sifli_unity_run(const char *filter)
{
    rt_size_t total;
    rt_size_t selected = 0;
    rt_size_t i;
    int failures;

    total = sifli_unity_case_count();
    if (total == 0)
    {
        rt_kprintf("unity: no test case registered (sifli_unity_cases is empty)\n");
        return -RT_ERROR;
    }

    for (i = 0; i < total; i++)
    {
        if (sifli_unity_case_matches(&sifli_unity_cases[i], filter))
        {
            selected++;
        }
    }

    if (selected == 0)
    {
        /*
         * Refuse to run an empty selection. Handing Unity nothing to do would
         * print "0 Tests 0 Failures 0 Ignored" followed by "OK", which is a
         * green-looking summary that says nothing at all; the host rejects it,
         * and a human reading the console deserves the same honesty.
         */
        rt_kprintf("unity: filter \"%s\" matched none of the %u registered case(s);"
                   " use \"unity_list\" to see them\n",
                   (filter != RT_NULL) ? filter : "", (unsigned int)total);
        return -RT_ERROR;
    }

    /*
     * Unity's result line is "<file>:<line>:<name>:<status>". The name is the
     * argument to UnityDefaultTestRun(), which is the case's fully-qualified
     * "suite.case" -- so the raw transcript identifies a case on its own, and
     * the name the host splits on is the same one unity_run accepts as a
     * filter. The file is Unity.TestFile, which UnitySetTestFile() sets per
     * case from the registry's source file; the argument here is only that
     * field's initial value and is never printed, because every case replaces
     * it before it can appear on a line.
     */
    UnityBegin("sifli_unity_cases");

    for (i = 0; i < total; i++)
    {
        const sifli_unity_case_t *tc = &sifli_unity_cases[i];
        char qualified[64];

        if (!sifli_unity_case_matches(tc, filter))
        {
            continue;
        }

        /*
         * A registry generated before the case carried a file has none; the
         * suite stands in for it, which is what the device used to print, so a
         * stale generated file costs nothing and cannot hand Unity a NULL.
         */
        UnitySetTestFile((tc->file != RT_NULL) ? tc->file : tc->suite);

        rt_snprintf(qualified, sizeof(qualified), "%s.%s",
                    (tc->suite != RT_NULL) ? tc->suite : "",
                    (tc->name != RT_NULL) ? tc->name : "");

        /*
         * Not RUN_TEST(): it stringifies its argument, and here the function
         * comes from a table at run time.
         *
         * The third argument is the case's number, not a source line: Unity
         * prints it on the PASS line, where a failing case prints the
         * assertion's own __LINE__ instead. Nothing reports it as a location.
         */
        UnityDefaultTestRun(tc->func, qualified, (int)(i + 1));
    }

    failures = UnityEnd();
    rt_kprintf("unity: %u case(s) selected, %d failure(s)\n",
               (unsigned int)selected, failures);
    return failures;
}

void sifli_unity_list(void)
{
    rt_size_t total = sifli_unity_case_count();
    rt_size_t i;

    if (total == 0)
    {
        rt_kprintf("unity: no test case registered (sifli_unity_cases is empty)\n");
        return;
    }

    for (i = 0; i < total; i++)
    {
        rt_kprintf("%s.%s\n", sifli_unity_cases[i].suite, sifli_unity_cases[i].name);
    }
}

void sifli_unity_autorun(void)
{
    /*
     * The generated rtconfig.h defines Kconfig names verbatim, without a
     * CONFIG_ prefix -- RT_USING_FINSH, not CONFIG_RT_USING_FINSH.
     */
#ifdef TEST_FWK_AUTORUN
    sifli_unity_run(RT_NULL);
#endif
}

#ifdef RT_USING_FINSH
static void unity_run(int argc, char **argv)
{
    const char *filter = RT_NULL;

    if (argc > 1)
    {
        filter = argv[1];
    }
    if (sifli_unity_run(filter) < 0)
    {
        rt_kprintf("unity_run [suite|case|suite.case]\n");
    }
}
MSH_CMD_EXPORT_ALIAS(unity_run, unity_run, unity_run [suite | case | suite.case] - run the Unity cases);

static void unity_list(void)
{
    sifli_unity_list();
}
MSH_CMD_EXPORT_ALIAS(unity_list, unity_list, unity_list - list the registered Unity cases);
#endif /* RT_USING_FINSH */

/**
 * @}
 */
