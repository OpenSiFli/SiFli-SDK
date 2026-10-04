/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SIFLI_UNITY_CONFIG_H__
#define SIFLI_UNITY_CONFIG_H__

/*
 * Unity configuration for the SiFli SDK.
 *
 * This file is pulled in by external/unity/src/unity_internals.h (line ~12)
 * as soon as UNITY_INCLUDE_CONFIG_H is defined, and it is included *before*
 * every "ifndef <macro>" default block in that header. Anything set here
 * therefore wins over Unity's own defaults, which is how the output contract
 * below is pinned down. The middleware/test_fwk group supplies the define and
 * points the include path here.
 *
 * The contract, as the host-side harness (tools/sdk_test/harness/unity.py)
 * parses it, is plain text:
 *
 *     <file>:<line>:<name>:PASS
 *     <file>:<line>:<name>:FAIL:<message>
 *     <n> Tests <n> Failures <n> Ignored
 *     OK | FAIL
 *
 * Every char Unity prints goes through one of the callbacks below.
 */

#include <rtthread.h>

/*
 * Send Unity's output to the RT-Thread console, so a host that is merely
 * attached to the serial port sees the whole run. Unity calls this once per
 * character; the console device write is what actually puts the byte on the
 * wire, so no extra flush is needed and the byte sequence is what Unity asked
 * for -- with one deliberate exception, the newline, which has its own path
 * below.
 */
#define UNITY_OUTPUT_CHAR(a)        rt_kprintf("%c", (a))

/*
 * End of line, handled separately from UNITY_OUTPUT_CHAR above.
 *
 * rt_kprintf() sets RT_DEVICE_FLAG_STREAM for the duration of its write, and
 * the serial layer expands every '\n' into "\r\n" while that flag is set
 * (rtos/rtthread/components/drivers/serial/serial.c, "_serial_poll_tx"). The
 * host ends a result line at the newline, so a trailing '\r' leaves
 * "<file>:<line>:<name>:PASS" without its end-of-line and the line is dropped
 * from the report entirely -- a passing case would simply be invisible.
 *
 * So the newline gets its own path, which writes one bare LF with the stream
 * flag cleared; everything else still goes through rt_kprintf. The definition
 * lives in sifli_unity_runner.c, which is linked together with this header.
 */
void sifli_unity_put_eol(void);
#define UNITY_PRINT_EOL()           sifli_unity_put_eol()

/*
 * UNITY_OUTPUT_COLOR is deliberately left undefined.
 *
 * Defining it turns PASS/FAIL/OK into ANSI escapes ("\033[42mPASS\033[0m").
 * The host matches a whole line and requires the status to be the last field
 * before the optional ":<message>", so an escape appended after the status
 * makes every result line unparsable and the run silently reports nothing.
 * Keep this output plain text.
 */

/*
 * Unity prints the failure message as "<sep><message>" right after the
 * status. The host's regex expects exactly one ':' there, so pin the
 * separator to its documented default instead of inheriting it -- a future
 * Unity default change must not be able to alter the on-wire format.
 */
#define UNITY_FAILURE_DETAIL_SEPARATOR  ":"

/*
 * Execution timing is disabled by defining the three hooks as no-ops.
 *
 * When UNITY_INCLUDE_EXEC_TIME is set (and Unity finds a clock) it appends
 * " (0 ms)" to every single result line, which again breaks the host's
 * "line ends after status/message" anchor. Defining these here means the
 * suffix cannot appear even if the flag is switched on somewhere in the
 * shared build environment.
 */
#define UNITY_PRINT_EXEC_TIME()
#define UNITY_EXEC_TIME_START()
#define UNITY_EXEC_TIME_STOP()

/*
 * No buffering is used, so there is nothing to flush and nothing to open or
 * close around a run. Defined explicitly to keep Unity from reaching for
 * stdio's fflush().
 */
#define UNITY_OUTPUT_FLUSH()        ((void)0)

/* Hooks Unity would otherwise leave to the integrating project. Nothing to do:
 * the report is a plain console stream, and the host reads it off the serial
 * port rather than from a file. */
#define UNITY_OUTPUT_START()
#define UNITY_OUTPUT_COMPLETE()

#endif /* SIFLI_UNITY_CONFIG_H__ */
