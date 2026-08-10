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

/* Insert configuration defines, e.g., #define EAP_MD5, here, if needed. */

#ifdef CONFIG_WIN32_DEFAULTS
#define CONFIG_NATIVE_WINDOWS
#define CONFIG_ANSI_C_EXTRA
#define CONFIG_WINPCAP
#define IEEE8021X_EAPOL
#define PKCS12_FUNCS
#define PCSC_FUNCS
#define CONFIG_CTRL_IFACE
#define CONFIG_CTRL_IFACE_NAMED_PIPE
#define CONFIG_DRIVER_NDIS
#define CONFIG_NDIS_EVENTS_INTEGRATED
#define CONFIG_DEBUG_FILE
#define EAP_MD5
#define EAP_TLS
#define EAP_MSCHAPv2
#define EAP_PEAP
#define EAP_TTLS
#define EAP_GTC
#define EAP_OTP
#define EAP_LEAP
#define EAP_TNC
#define _CRT_SECURE_NO_DEPRECATE

#ifdef USE_INTERNAL_CRYPTO
#define CONFIG_TLS_INTERNAL_CLIENT
#define CONFIG_INTERNAL_LIBTOMMATH
#define CONFIG_CRYPTO_INTERNAL
#endif /* USE_INTERNAL_CRYPTO */
#endif /* CONFIG_WIN32_DEFAULTS */

#if 1//def CONFIG_RWNX_LWIP
#include "rwnx_config.h"
#include "aic8800_config.h"

//#define CONFIG_AES_ALT
//#define CONFIG_SHA1_ALT

#define CONFIG_RWNX
//#define CONFIG_LWIP
#define CONFIG_DRIVER_RWNX
#define OS_NO_C_LIB_DEFINES
//#define CONFIG_NO_STDOUT_DEBUG
//#define CONFIG_NO_WPA_MSG
#define CONFIG_NO_HOSTAPD_LOGGER
#define CONFIG_CTRL_IFACE
#define CONFIG_CTRL_IFACE_UDP

#define CONFIG_TLS_INTERNAL_CLIENT
#define CONFIG_INTERNAL_LIBTOMMATH
#define CONFIG_CRYPTO_INTERNAL
#define CONFIG_NO_RANDOM_POOL

#if NX_MFP
#define CONFIG_IEEE80211W
#endif

#if NX_WPS
#define CONFIG_WPS
#define IEEE8021X_EAPOL
#define EAP_WSC
#endif

// for now only used for SAE
#ifdef CONFIG_MBEDTLS
#define CONFIG_SME
#define CONFIG_SAE
#define CONFIG_ECC
#endif

#define CONFIG_ACS

// AP support
#if 1//NX_BEACONING
#define CONFIG_AP
#define NEED_AP_MLME
#define CONFIG_NO_RADIUS
//#ifdef PLATFORM_UNISOC_THREADX
#define CONFIG_NO_ACCOUNTING
#define CONFIG_NO_VLAN
//#endif

#define EAP_SERVER
#define EAP_SERVER_IDENTITY
#if NX_WPS
#define EAP_SERVER_WSC
#endif
#define CONFIG_WNM_AP
#define CONFIG_IEEE80211N
#if NX_VHT
#define CONFIG_IEEE80211AC
#endif
#if NX_HE
#define CONFIG_IEEE80211AX
#endif
#endif // NX_BEACONING

#if NX_P2P
#define CONFIG_P2P
#define CONFIG_GAS
#define CONFIG_OFFCHANNEL
#define CONFIG_WIFI_DISPLAY
#endif

#endif /* CONFIG_RWNX_LWIP */

#endif /* BUILD_CONFIG_H */
