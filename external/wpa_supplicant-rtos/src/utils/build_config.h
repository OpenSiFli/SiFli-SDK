/*
 * wpa_supplicant/hostapd - Build time configuration defines
 * Copyright (c) 2005-2006, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 *
 * This header file can be used to define configuration defines that were
 * originally defined in Makefile. This is mainly meant for IDE use or for
 * systems that do not have suitable 'make' tool. In these cases, it may be
 * easier to have a single place for defining all the needed C pre-processor
 * defines.
 */

#ifndef BUILD_CONFIG_H
#define BUILD_CONFIG_H

#include <wifi_config_default.h>

#if !CONFIG_WPA_SUPP
#undef CONFIG_WPA_SUPP
#define CONFIG_WPA_SUPP 1
#endif

#if defined(USE_RTOS) && defined(SDK_OS_FREE_RTOS)
#define CONFIG_FREERTOS 1
//#define CONFIG_KSDK_MBEDTLS 0
#endif

#if !CONFIG_IPV6
#undef CONFIG_IPV6
#endif

#define CONFIG_WPA_SUPP_CRYPTO   1
//#define CONFIG_WPA_SUPP_AP       1
//#define CONFIG_HOSTAPD           0

#define WPA_SUPPLICANT_CLEANUP_INTERVAL 120

#define HOSTAPD_CLEANUP_INTERVAL 120

#if !defined(CONFIG_WPA_SUPP_DEBUG_LEVEL)

#define CONFIG_WPA_SUPP_DEBUG_LEVEL 6

#endif

#if CONFIG_WPA_SUPP_DEBUG_LEVEL > 5 && !defined(CONFIG_WPA_SUPP_DPP)

#define CONFIG_NO_STDOUT_DEBUG 1

#endif

#endif /* BUILD_CONFIG_H */
