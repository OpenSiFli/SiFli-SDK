/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SIFLI_UNITY_H__
#define SIFLI_UNITY_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup test_fwk
 * @{
 */

/**
 * One registered Unity test case.
 *
 * The test project provides the table; this component only walks it. That is
 * a deliberate choice over a linker section: RT-Thread's utest needs a
 * UtestTcTab section to be reserved in every linker script, and this SDK
 * carries 40+ linker templates that already reserve it. Handing registration
 * to a plain array keeps a new test component from having to touch any of
 * them, and keeps it from breaking the templates that already exist.
 *
 * Unity itself registers tests with explicit calls rather than by scanning a
 * section, so an array is also the shape that matches the framework.
 */
typedef struct
{
    const char *suite;          /**< suite name, e.g. "lz4"                 */
    const char *name;           /**< case name,  e.g. "roundtrip"           */
    void (*func)(void);         /**< the test body                          */
    const char *file;           /**< source file, e.g. "test_lz4.c"         */
} sifli_unity_case_t;

/**
 * The registry, defined by the test project.
 *
 * An array of cases terminated by an entry whose func is RT_NULL:
 *
 *     const sifli_unity_case_t sifli_unity_cases[] =
 *     {
 *         { "lz4", "roundtrip", test_lz4_roundtrip, "test_lz4.c" },
 *         { RT_NULL, RT_NULL, RT_NULL, RT_NULL },
 *     };
 *
 * ``file`` is the source the case was found in, which Unity prints as the
 * location of a result line. It is the last field on purpose: a registry
 * generated before it existed still compiles, its three initializers landing
 * in the same three slots, and the runner falls back to the suite name for the
 * file rather than printing a NULL.
 *
 * The runner prints a case as "suite.case", so a name must not contain '.' any
 * more than suite and case names may contain ':' -- Unity prints them between
 * colons and the host parses the line by splitting it on both.
 */
extern const sifli_unity_case_t sifli_unity_cases[];

/**
 * Run every registered case selected by @p filter.
 *
 * @param filter a suite name ("lz4"), a case name ("roundtrip"), or the pair
 *               joined by a dot ("lz4.roundtrip"). NULL or an empty string
 *               selects everything.
 *
 * @return the number of failed cases, or a negative value on error (nothing
 *         matched, or the registry is empty).
 */
int sifli_unity_run(const char *filter);

/**
 * Print every registered case, one "suite.case" per line, over the console.
 */
void sifli_unity_list(void);

/**
 * Run every registered case, if this build asked for that.
 *
 * A no-op unless ``TEST_FWK_AUTORUN`` is set, so a project's ``main()``
 * can call it unconditionally and leave the decision next to the Kconfig it
 * belongs to. Running the cases is all it does: the shell stays usable, so
 * ``unity_run`` still selects a subset by hand afterwards.
 */
void sifli_unity_autorun(void);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* SIFLI_UNITY_H__ */
