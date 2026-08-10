/*
 * wpa_supplicant/hostapd - Default include files
 * Copyright (c) 2005-2006, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 *
 * This header file is included into all C files so that commonly used header
 * files can be selected with OS specific ifdef blocks in one place instead of
 * having to have OS/C library specific selection in many files.
 */

#ifndef INCLUDES_H
#define INCLUDES_H

/* Include possible build time configuration before including anything else */
#include "build_config.h"

#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#ifndef _WIN32_WCE
#include <signal.h>
//#include <sys/types.h>
//#include <errno.h>
#endif /* _WIN32_WCE */

#ifndef _MSC_VER
//#include <unistd.h>
#endif /* _MSC_VER */

#include "aic_plat_log.h"

#ifdef CONFIG_RWNX
#include "rtos_port.h"
#include "netif_co.h"
#include "compiler.h"
#include "fhost_wpa.h"
//#define vsnprintf dbg_vsnprintf
#ifndef isprint
#define isprint(c)      ((c >= 0x1F) && (c <= 0x7E))
#define isblank(c)      ((c) == ' ' || (c) == '\t')
#define isspace(c)      ((c) == ' ' || (c) == '\f' || (c) == '\n' || (c) == '\r' || (c) == '\t' || (c) == '\v')
#endif
#endif /* CONFIG_RWNX */

#endif /* INCLUDES_H */
