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
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#ifndef _WIN32_WCE
//#include <signal.h>
#ifdef __ZEPHYR__
#include <sys/types.h>
#include <errno.h>
#endif
#endif /* _WIN32_WCE */
#include <ctype.h>

#if !(defined(MSC_VER) || defined(__ZEPHYR__) || (CONFIG_FREERTOS))
#include <unistd.h>
#endif /* _MSC_VER */

#if !((CONFIG_NATIVE_WINDOWS) || defined(__ZEPHYR__) || (CONFIG_FREERTOS))
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#ifndef __vxworks
//#include <sys/uio.h>
#include <sys/time.h>
#endif /* __vxworks */
#endif /* CONFIG_NATIVE_WINDOWS */

#if defined(__ZEPHYR__)
#if (CONFIG_POSIX_API)
#include <zephyr/posix/arpa/inet.h>
#include <zephyr/posix/sys/select.h>
#include <zephyr/posix/sys/socket.h>
#include <zephyr/posix/unistd.h>
#else /* (CONFIG_POSIX_API) */
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#endif /* (CONFIG_POSIX_API) */
#include <zephyr/shell/shell.h>
#endif /* defined(__ZEPHYR__) */

#include <wm_net.h>
#include <lwip/opt.h>
#include <lwip/sys.h>
#include <lwip/tcpip.h>
#include <lwip/api.h>



#if (CONFIG_FREERTOS)

#if (CONFIG_POSIX_API)
#include <posix/arpa/inet.h>
//#include <posix/sys/select.h>
#include <posix/sys/socket.h>
//#include <posix/unistd.h>
#else /* (CONFIG_POSIX_API) */
#include <lwip/inet.h>
#include <lwip/sockets.h>
#endif /* (CONFIG_POSIX_API) */
//#include <zephyr/shell/shell.h>
// #include <fsl_debug_console.h>
#endif /* (CONFIG_FREERTOS) */

#if CONFIG_WPA_SUPP_CRYPTO

#include <mbedtls/version.h>

#ifndef MBEDTLS_NIST_KW_C
#define CONFIG_INTERNAL_AES_WRAP 1
#define CONFIG_INTERNAL_AES_UNWRAP 1
#endif

#ifndef MBEDTLS_CMAC_C
#define CONFIG_INTERNAL_OMAC1_AES 1
#endif

#ifndef MBEDTLS_ARC4_C
#define CONFIG_INTERNAL_RC4 1
#endif

#ifndef MBEDTLS_MD4_C
#define CONFIG_INTERNAL_MD4 1
#endif

#endif

#ifdef CONFIG_FIPS
#define CONFIG_INTERNAL_MD4 1
#define CONFIG_INTERNAL_MD5 1
#define CONFIG_INTERNAL_AES_WRAP 1
#define CONFIG_INTERNAL_AES_UNWRAP 1
#define CONFIG_INTERNAL_DES 1
#endif

#define CONFIG_SHA256 1
#if CONFIG_WPA_SUPP_CRYPTO
#define CONFIG_SHA384 1
#define CONFIG_SHA512 1
#define CONFIG_DES 1
#define CONFIG_AES 1
#endif

#if !CONFIG_AES
#define CONFIG_INTERNAL_AES 1
#endif

#if CONFIG_11R
#define CONFIG_IEEE80211R 1
#if CONFIG_WPA_SUPP_AP
#define CONFIG_IEEE80211R_AP 1
#endif
#endif

#define CONFIG_SME 1
// #define CONFIG_NO_TKIP
#define CONFIG_NO_CONFIG_WRITE 1
#define CONFIG_CTRL_IFACE 1
#define CONFIG_CTRL_IFACE_UDP 0
#define CONFIG_NO_RANDOM_POOL 1
#define CONFIG_WNM 1
#define IEEE8021X_EAPOL

#if CONFIG_WPA_SUPP_AP
#define CONFIG_AP 1
#define CONFIG_NO_RADIUS 1
#define CONFIG_NO_VLAN 1
#define CONFIG_NO_ACCOUNTING 1
#define NEED_AP_MLME
#define HOSTAPD
#define CONFIG_IEEE80211AX 1
#define CONFIG_ACS 1
#define NEED_RSN_AUTHENTICATOR
#define CONFIG_IEEE80211AC 1
#define CONFIG_WNM_AP 1
#define CONFIG_ETH_P_OUI 1
#endif

#if !CONFIG_WPA_SUPP_CRYPTO
#define CONFIG_CRYPTO_INTERNAL 1
#define CONFIG_INTERNAL_OMAC1_AES 1
#define CONFIG_INTERNAL_MD4 1
#define CONFIG_INTERNAL_MD5 1
#define CONFIG_INTERNAL_AES_WRAP 1
#define CONFIG_INTERNAL_AES_UNWRAP 1
#define CONFIG_INTERNAL_DES 1
#define CONFIG_INTERNAL_AES 1
#define CONFIG_INTERNAL_RC4 1
#define CONFIG_INTERNAL_SHA1 1
#define CONFIG_INTERNAL_SHA256 1

//#define CONFIG_TLS_INTERNAL_CLIENT
//#define CONFIG_TLS_INTERNAL_SERVER
#define CONFIG_SHA256 1
//#define CONFIG_SHA384
//#define CONFIG_HMAC_SHA384_KDF
//#define CONFIG_INTERNAL_SHA384
#endif

#if CONFIG_WPA_SUPP_WPA3
#define CONFIG_SAE 1
//#define CONFIG_SAE_PK
#define CONFIG_ECC 1
#endif

#if CONFIG_WPA_SUPP_WPS
#define CONFIG_WPS 1
#define EAP_WSC
#define IEEE8021X_EAPOL
#define NEED_DH_GROUPS
#define NEED_BASE64
#define NEED_AES_CBC
#define EAP_SERVER_WSC
#define EAP_SERVER_IDENTITY
#endif

#if CONFIG_WPA_SUPP_P2P
#define CONFIG_P2P 1
#define CONFIG_GAS 1
#define CONFIG_OFFCHANNEL 1
#endif

#if CONFIG_WPA_SUPP_DPP
#define CONFIG_DPP
#define CONFIG_DPP2
#define CONFIG_DPP3
#define CONFIG_GAS
#define CONFIG_OFFCHANNEL
#define CONFIG_GAS_SERVER
#if CONFIG_WPA_SUPP_AP
#define CONFIG_INTERWORKING
#endif /* CONFIG_WPA_SUPP_AP */
#endif

#if (CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE) || (CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE)

#define CONFIG_SUITEB 1
#define CONFIG_SUITEB192 1

#define CONFIG_TLSV11 1
#define CONFIG_TLSV12 1
#define TLS_FUNCS
#define EAP_TLS_FUNCS
#define MS_FUNCS
#define NEED_DES
#define NEED_MD4
#define NEED_TLS_PRF
#define NEED_TLS_PRF_SHA256
#define NEED_SHA1
#define CHAP
#define NEED_AES_CBC
#define NEED_AES
#define NEED_AES_ENCBLOCK
#define NEED_AES_EAX
#define NEED_FIPS186_2_PRF
#define NEED_T_PRF
#define NEED_SHA384
#define NEED_TLS_PRF_SHA256
#define NEED_TLS_PRF_SHA384
#define NEED_DRAGONFLY
#define NEED_CIPHER
//#define PCSC_FUNCS
#define NEED_ASN1
#define NEED_AES_EAX
#define NEED_AES_SIV
#define NEED_AES_CTR
#define CONFIG_SIM_SIMULATOR 1
#define CONFIG_USIM_SIMULATOR 1
#define NEED_MILENAGE
#define NEED_AES_ENC
#define IEEE8021X_EAPOL

#if CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE
#if CONFIG_EAP_TLS
#define EAP_TLS
#endif
#if CONFIG_EAP_PEAP
#define EAP_PEAP
#endif
#if CONFIG_EAP_TTLS
#define EAP_TTLS
#endif
//#define EAP_MD5
//#define EAP_LEAP
//#define EAP_PSK
#ifndef CONFIG_FIPS
#if CONFIG_EAP_FAST
#define EAP_FAST
#endif
#endif
//#define EAP_PAX
//#define EAP_SAKE
//#define EAP_GPSK
//#define EAP_PWD
//#define EAP_EKE
//#define EAP_IKEv2
#if CONFIG_EAP_SIM
#define EAP_SIM
#define CONFIG_EAP_SIM_COMMON 1
#endif
#if CONFIG_EAP_AKA_PRIME
#define EAP_AKA_PRIME
#if !CONFIG_EAP_AKA
#define CONFIG_EAP_AKA 1
#endif
#endif
#if CONFIG_EAP_AKA
#define EAP_AKA
#define CONFIG_EAP_SIM_COMMON 1
#endif
#if CONFIG_EAP_MSCHAPV2
#define EAP_MSCHAPv2
#endif
#if CONFIG_EAP_GTC
#define EAP_GTC
#endif
#endif

#if CONFIG_WPA_SUPP_AP
#if CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE
#define RADIUS_SERVER
#define EAP_SERVER
#define EAP_SERVER_IDENTITY
#if CONFIG_EAP_TLS
#define EAP_SERVER_TLS
#endif
#if CONFIG_EAP_PEAP
#define EAP_SERVER_PEAP
#endif
#if CONFIG_EAP_TTLS
#define EAP_SERVER_TTLS
#endif
#define EAP_SIM_DB
#if CONFIG_EAP_SIM
#define EAP_SERVER_SIM
#endif
#if CONFIG_EAP_AKA_PRIME
#define EAP_SERVER_AKA_PRIME
#if !CONFIG_EAP_AKA
#define CONFIG_EAP_AKA 1
#endif
#endif
#if CONFIG_EAP_AKA
#define EAP_SERVER_AKA
#define CONFIG_EAP_SIM_COMMON 1
#endif
#ifndef CONFIG_FIPS
#if CONFIG_EAP_FAST
#define EAP_SERVER_FAST
#endif
#endif
#if CONFIG_EAP_MSCHAPV2
#define EAP_SERVER_MSCHAPV2
#endif
#if CONFIG_EAP_GTC
#define EAP_SERVER_GTC
#endif
#endif
#endif

#endif /* CONFIG_WPA_SUPP_CRYPTO_ENTERPRISE || CONFIG_WPA_SUPP_CRYPTO_AP_ENTERPRISE */

#endif /* INCLUDES_H */
