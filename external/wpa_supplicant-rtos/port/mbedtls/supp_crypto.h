/** @file supp_crypto.h
 *
 *  @brief  This file provides mbedtls port layer API for supplicant
 */
/*
 *  Copyright 2024 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifdef RW610
#include "wpa_supp_els_pkc_mbedtls_config.h"
#endif

#ifdef RW610
/* As mbedtls didn't support time check for cert files, we add support for it internally, and only use it temporary.
 * Delete this impl after mbedtls support it.
 */
void supp_set_mbedtls_set_time();

#if defined ( __GNUC__ )
#if defined(MBEDTLS_HAVE_TIME) || defined(MBEDTLS_HAVE_TIME_DATE)
#include <sys/time.h>
int _gettimeofday(struct timeval *tv, void *tzvp);
#endif
#endif
#endif