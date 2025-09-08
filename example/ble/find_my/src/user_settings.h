/*
 * SPDX-FileCopyrightText: 2021-2021 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* user_settings.h
wolfCrypt library configuration file

Example Settings for PCA10040 with the following algorithms:
 * DRBG with hardware RNG seed
 * X9.63 SHA2-256 - KDF
 * ECC P-256 / P-224
 * AES-GCM (possible acceleration with AES HW encryption with EasyDMA)
*/

#ifndef WOLFSSL_USER_SETTINGS_H
#define WOLFSSL_USER_SETTINGS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Enable this to enhance performance - will increase code size */
/* 0=None (36,708), 1=Some (37,924), 2=All (57,544) */
#define BUILD_FOR_PERF 0

/* ------------------------------------------------------------------------- */
/* Platform */
/* ------------------------------------------------------------------------- */
#define WOLFCRYPT_ONLY
#define WOLFSSL_GENERAL_ALIGNMENT 4
#define SINGLE_THREADED
#define WOLFSSL_SMALL_STACK
#define WOLFSSL_USER_IO
#define SIZEOF_LONG_LONG 8

/* nRF5 SDK doesn't support strncasecmp - override with strncmp */
#define XSTRNCASECMP(s1,s2,n) strncmp((s1),(s2),(n))


/* ------------------------------------------------------------------------- */
/* Math Configuration */
/* ------------------------------------------------------------------------- */
#define USE_FAST_MATH
#define TFM_TIMING_RESISTANT
#if BUILD_FOR_PERF >= 1
#define TFM_ARM /* Assembly optimization */
#endif

/* Wolf Single Precision Math */
#if BUILD_FOR_PERF >= 2
#define WOLFSSL_SP
#define WOLFSSL_SP_SMALL /* use smaller version of code */
#define WOLFSSL_HAVE_SP_ECC
#define WOLFSSL_SP_ASM
#define WOLFSSL_SP_ARM_CORTEX_M_ASM /* Assembly optimization */
#endif


/* ------------------------------------------------------------------------- */
/* Crypto */
/* ------------------------------------------------------------------------- */
/* ECC */
#define HAVE_ECC

/* Manually define enabled curves */
#define ECC_USER_CURVES /* 256-bit on by default */
//#define NO_ECC256
#define HAVE_ECC224
/* wolfCrypt test needs right keygen size */
#if defined(HAVE_ECC224) && defined(NO_ECC256)
#define ECC_KEYGEN_SIZE 28
#define BENCH_ECC_SIZE  28
#endif

/* Optional ECC calculation method */
/* Note: doubles heap usage, but slightly faster */
#if BUILD_FOR_PERF >= 1
#define ECC_SHAMIR
#endif

/* Reduces heap usage, but slower */
#define ECC_TIMING_RESISTANT

/* Custom fastmath size */
/* MAX = ROUND32(ECC BITS 256) * 2 */
#define FP_MAX_BITS (256 * 2)

/* Speedups specific to curve */
#if BUILD_FOR_PERF >= 2
#ifndef NO_ECC256
#define TFM_ECC256
#endif
#ifdef HAVE_ECC224
#define TFM_ECC224
#endif
#endif

/* AES */
#define HAVE_AESGCM
#define AES_MAX_KEY_SIZE 128

/* GCM Method: None, GCM_SMALL, GCM_WORD32 or GCM_TABLE */
#if BUILD_FOR_PERF >= 2
#define GCM_TABLE
#else
#define GCM_SMALL
#endif

/* Sha256 - On by default */
#if BUILD_FOR_PERF == 0
/* Optional size reduction (not unrolled - ~2k smaller and ~25% slower) */
#define USE_SLOW_SHA256
#endif

/* X963 KDF */
#define HAVE_X963_KDF


/* ------------------------------------------------------------------------- */
/* Benchmark / Test */
/* ------------------------------------------------------------------------- */
/* Use reduced benchmark / test sizes */
#define BENCH_EMBEDDED
#define USE_CERT_BUFFERS_256


/* ------------------------------------------------------------------------- */
/* Debugging */
/* ------------------------------------------------------------------------- */

#undef DEBUG_WOLFSSL
#undef NO_ERROR_STRINGS
#if 0
#define DEBUG_WOLFSSL
#else
#if 0
#define NO_ERROR_STRINGS
#endif
#endif


/* ------------------------------------------------------------------------- */
/* Memory */
/* ------------------------------------------------------------------------- */

/* Memory callbacks */
#if 0
#define USE_WOLFSSL_MEMORY

/* Use this to measure / print heap usage */
#if 0
#define WOLFSSL_TRACK_MEMORY
#define WOLFSSL_DEBUG_MEMORY
#endif
#else
#define NO_WOLFSSL_MEMORY
#endif


/* ------------------------------------------------------------------------- */
/* Port */
/* ------------------------------------------------------------------------- */
/* Override Current Time */
/* Allows custom "custom_time()" function to be used for benchmark */
#define WOLFSSL_USER_CURRTIME
#define WOLFSSL_GMTIME
#define USER_TICKS


/* ------------------------------------------------------------------------- */
/* RNG */
/* ------------------------------------------------------------------------- */
/* uses wc_GenerateSeed in random.c for WOLFSSL_NRF52 */
#define HAVE_HASHDRBG
//extern int nrf_random_generate_seed(uint8_t* output, uint32_t size);
//#define CUSTOM_RAND_GENERATE_SEED nrf_random_generate_seed


/* ------------------------------------------------------------------------- */
/* Enable Features */
/* ------------------------------------------------------------------------- */
//#define WOLFSSL_BASE64_ENCODE
#define WOLFSSL_VALIDATE_ECC_IMPORT /* Validate import */
#define WOLFSSL_VALIDATE_ECC_KEYGEN /* Validate ECC key gen */
#define WOLFSSL_PUBLIC_ECC_ADD_DBL /* Export ECC point functions */
//#define HAVE_COMP_KEY /* ECC Compressed Key Support */


/* ------------------------------------------------------------------------- */
/* Disable Features */
/* ------------------------------------------------------------------------- */
#define NO_RSA
#define NO_DSA
#define NO_DH
#define NO_SHA
//#define NO_AES_DECRYPT
#define NO_AES_CBC
#define NO_RC4
#define NO_HC128
#define NO_RABBIT
#define NO_PSK
#define NO_MD4
#define NO_MD5
#define NO_DES3
#define NO_PKCS12
#define NO_PKCS8
#define NO_PBKDF1
#define NO_PBKDF2
//#define NO_HMAC
#define NO_PWDBASED
#define NO_OLD_TLS
#define NO_STRICT_ECDSA_LEN

#define NO_FILESYSTEM
#define NO_WRITEV
#define NO_MAIN_DRIVER
#define NO_DEV_RANDOM
#define NO_SESSION_CACHE
#define NO_SIG_WRAPPER

/* no time required */
#define NO_ASN_TIME

#define NO_CRYPT_TEST
/* Note: Enabling "-flto" in Makefile causes RTC init issue */
#define NO_CRYPT_BENCHMARK
//#define NO_CERTS
//#define NO_INLINE
//#define NO_CODING

#ifdef __cplusplus
}
#endif

#endif /* WOLFSSL_USER_SETTINGS_H */
