/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ================================================================
 * aic_mbedtls_port_example.c — example external implementation of mbedTLS crypto interface
 *
 * Purpose (USE_BUILTIN_MBEDTLS=0, see "mbedTLS Crypto library selection" in platform_config.mk):
 *   When the platform SDK already ships standard mbedTLS (>=3.6), there is no need to
 *   link the built-in libaic_mbedtls.a; only the 35 aic_mbedtls_* symbols
 *   (22 MPI + 13 ECP) must be provided.
 *
 * Interface mapping (core idea):
 *   aic_mbedtls_* = standard mbedtls API + "aic_" prefix, identical signature, so every
 *   function below strips the aic_ prefix and forwards to the standard mbedtls interface:
 *     aic_mbedtls_mpi_init(X)      → mbedtls_mpi_init(X)
 *     aic_mbedtls_ecp_group_load() → mbedtls_ecp_group_load()
 *   Full contract in host/thirdparty/mbedtls_v3_6_2/include/aic_mbedtls_crypto_if.h
 *
 * Usage:
 *   1. platform_config.mk: USE_BUILTIN_MBEDTLS=0
 *   2. SRC_PLATFORM += $(PLATFORM_PATH)/crypto/aic_mbedtls_port_example.c
 *      (with USE_BUILTIN_MBEDTLS=0 the top-level Makefile includes this file by default;
 *       AIC_MBEDTLS_FWD_EXAMPLE=0 disables it in favor of a fully custom implementation)
 *   3. EXT_INC_DIRS += <platform mbedtls include path> (so #include <mbedtls/bignum.h> works)
 *   4. SDK links the platform mbedtls library (no longer libaic_mbedtls.a)
 *
 * Note: this file targets the platform's own "standard mbedtls" (functions named mbedtls_*).
 *       The driver's vendored mbedtls (host/thirdparty/mbedtls_v3_6_2) already carries the
 *       aic_ prefix in its function names and must not be mixed with this file.
 * ================================================================
 */

#include "mbedtls/bignum.h"
#include "mbedtls/ecp.h"

/* ================================================================
 * Part 1: big number (MPI) interfaces — 22 functions
 * Each function → standard mbedtls interface of the same name without aic_ prefix
 * ================================================================ */

/* --- Lifecycle --- */

/* aic_mbedtls_mpi_init → mbedtls_mpi_init */
void aic_mbedtls_mpi_init(mbedtls_mpi *X)
{
    mbedtls_mpi_init(X);
}

/* aic_mbedtls_mpi_free → mbedtls_mpi_free */
void aic_mbedtls_mpi_free(mbedtls_mpi *X)
{
    mbedtls_mpi_free(X);
}

/* --- Copy and assignment --- */

/* aic_mbedtls_mpi_copy → mbedtls_mpi_copy */
int aic_mbedtls_mpi_copy(mbedtls_mpi *X, const mbedtls_mpi *Y)
{
    return mbedtls_mpi_copy(X, Y);
}

/* aic_mbedtls_mpi_lset → mbedtls_mpi_lset */
int aic_mbedtls_mpi_lset(mbedtls_mpi *X, mbedtls_mpi_sint z)
{
    return mbedtls_mpi_lset(X, z);
}

/* aic_mbedtls_mpi_safe_cond_assign → mbedtls_mpi_safe_cond_assign */
int aic_mbedtls_mpi_safe_cond_assign(mbedtls_mpi *X, const mbedtls_mpi *Y,
                                     unsigned char assign)
{
    return mbedtls_mpi_safe_cond_assign(X, Y, assign);
}

/* --- Serialization --- */

/* aic_mbedtls_mpi_read_binary → mbedtls_mpi_read_binary */
int aic_mbedtls_mpi_read_binary(mbedtls_mpi *X, const unsigned char *buf,
                                size_t buflen)
{
    return mbedtls_mpi_read_binary(X, buf, buflen);
}

/* aic_mbedtls_mpi_write_binary → mbedtls_mpi_write_binary */
int aic_mbedtls_mpi_write_binary(const mbedtls_mpi *X, unsigned char *buf,
                                 size_t buflen)
{
    return mbedtls_mpi_write_binary(X, buf, buflen);
}

/* --- Query --- */

/* aic_mbedtls_mpi_size → mbedtls_mpi_size */
size_t aic_mbedtls_mpi_size(const mbedtls_mpi *X)
{
    return mbedtls_mpi_size(X);
}

/* aic_mbedtls_mpi_bitlen → mbedtls_mpi_bitlen */
size_t aic_mbedtls_mpi_bitlen(const mbedtls_mpi *X)
{
    return mbedtls_mpi_bitlen(X);
}

/* --- Comparison --- */

/* aic_mbedtls_mpi_cmp_mpi → mbedtls_mpi_cmp_mpi */
int aic_mbedtls_mpi_cmp_mpi(const mbedtls_mpi *X, const mbedtls_mpi *Y)
{
    return mbedtls_mpi_cmp_mpi(X, Y);
}

/* aic_mbedtls_mpi_cmp_int → mbedtls_mpi_cmp_int */
int aic_mbedtls_mpi_cmp_int(const mbedtls_mpi *X, mbedtls_mpi_sint z)
{
    return mbedtls_mpi_cmp_int(X, z);
}

/* --- Shift --- */

/* aic_mbedtls_mpi_shift_r → mbedtls_mpi_shift_r */
int aic_mbedtls_mpi_shift_r(mbedtls_mpi *X, size_t count)
{
    return mbedtls_mpi_shift_r(X, count);
}

/* --- Arithmetic --- */

/* aic_mbedtls_mpi_add_mpi → mbedtls_mpi_add_mpi */
int aic_mbedtls_mpi_add_mpi(mbedtls_mpi *X, const mbedtls_mpi *A,
                            const mbedtls_mpi *B)
{
    return mbedtls_mpi_add_mpi(X, A, B);
}

/* aic_mbedtls_mpi_sub_mpi → mbedtls_mpi_sub_mpi */
int aic_mbedtls_mpi_sub_mpi(mbedtls_mpi *X, const mbedtls_mpi *A,
                            const mbedtls_mpi *B)
{
    return mbedtls_mpi_sub_mpi(X, A, B);
}

/* aic_mbedtls_mpi_mul_mpi → mbedtls_mpi_mul_mpi */
int aic_mbedtls_mpi_mul_mpi(mbedtls_mpi *X, const mbedtls_mpi *A,
                            const mbedtls_mpi *B)
{
    return mbedtls_mpi_mul_mpi(X, A, B);
}

/* aic_mbedtls_mpi_div_mpi → mbedtls_mpi_div_mpi */
int aic_mbedtls_mpi_div_mpi(mbedtls_mpi *Q, mbedtls_mpi *R,
                            const mbedtls_mpi *A, const mbedtls_mpi *B)
{
    return mbedtls_mpi_div_mpi(Q, R, A, B);
}

/* aic_mbedtls_mpi_add_int → mbedtls_mpi_add_int */
int aic_mbedtls_mpi_add_int(mbedtls_mpi *X, const mbedtls_mpi *A,
                            mbedtls_mpi_sint b)
{
    return mbedtls_mpi_add_int(X, A, b);
}

/* aic_mbedtls_mpi_sub_int → mbedtls_mpi_sub_int */
int aic_mbedtls_mpi_sub_int(mbedtls_mpi *X, const mbedtls_mpi *A,
                            mbedtls_mpi_sint b)
{
    return mbedtls_mpi_sub_int(X, A, b);
}

/* aic_mbedtls_mpi_sub_abs → mbedtls_mpi_sub_abs */
int aic_mbedtls_mpi_sub_abs(mbedtls_mpi *X, const mbedtls_mpi *A,
                            const mbedtls_mpi *B)
{
    return mbedtls_mpi_sub_abs(X, A, B);
}

/* --- Modular arithmetic --- */

/* aic_mbedtls_mpi_mod_mpi → mbedtls_mpi_mod_mpi */
int aic_mbedtls_mpi_mod_mpi(mbedtls_mpi *R, const mbedtls_mpi *A,
                            const mbedtls_mpi *B)
{
    return mbedtls_mpi_mod_mpi(R, A, B);
}

/* aic_mbedtls_mpi_exp_mod → mbedtls_mpi_exp_mod */
int aic_mbedtls_mpi_exp_mod(mbedtls_mpi *X, const mbedtls_mpi *A,
                            const mbedtls_mpi *E, const mbedtls_mpi *N,
                            mbedtls_mpi *prec_RR)
{
    return mbedtls_mpi_exp_mod(X, A, E, N, prec_RR);
}

/* aic_mbedtls_mpi_inv_mod → mbedtls_mpi_inv_mod */
int aic_mbedtls_mpi_inv_mod(mbedtls_mpi *X, const mbedtls_mpi *A,
                            const mbedtls_mpi *N)
{
    return mbedtls_mpi_inv_mod(X, A, N);
}

/* ================================================================
 * Part 2: elliptic curve (ECP) interfaces — 13 functions
 * Each function → standard mbedtls interface of the same name without aic_ prefix
 * ================================================================ */

/* --- Lifecycle --- */

/* aic_mbedtls_ecp_group_init → mbedtls_ecp_group_init */
void aic_mbedtls_ecp_group_init(mbedtls_ecp_group *grp)
{
    mbedtls_ecp_group_init(grp);
}

/* aic_mbedtls_ecp_group_free → mbedtls_ecp_group_free */
void aic_mbedtls_ecp_group_free(mbedtls_ecp_group *grp)
{
    mbedtls_ecp_group_free(grp);
}

/* aic_mbedtls_ecp_group_load → mbedtls_ecp_group_load */
int aic_mbedtls_ecp_group_load(mbedtls_ecp_group *grp, mbedtls_ecp_group_id id)
{
    return mbedtls_ecp_group_load(grp, id);
}

/* aic_mbedtls_ecp_point_init → mbedtls_ecp_point_init */
void aic_mbedtls_ecp_point_init(mbedtls_ecp_point *pt)
{
    mbedtls_ecp_point_init(pt);
}

/* aic_mbedtls_ecp_point_free → mbedtls_ecp_point_free */
void aic_mbedtls_ecp_point_free(mbedtls_ecp_point *pt)
{
    mbedtls_ecp_point_free(pt);
}

/* --- Point operations --- */

/* aic_mbedtls_ecp_mul → mbedtls_ecp_mul */
int aic_mbedtls_ecp_mul(mbedtls_ecp_group *grp, mbedtls_ecp_point *R,
                        const mbedtls_mpi *m, const mbedtls_ecp_point *P,
                        int (*f_rng)(void *, unsigned char *, size_t),
                        void *p_rng)
{
    return mbedtls_ecp_mul(grp, R, m, P, f_rng, p_rng);
}

/* aic_mbedtls_ecp_muladd → mbedtls_ecp_muladd */
int aic_mbedtls_ecp_muladd(mbedtls_ecp_group *grp, mbedtls_ecp_point *R,
                           const mbedtls_mpi *m, const mbedtls_ecp_point *P,
                           const mbedtls_mpi *n, const mbedtls_ecp_point *Q)
{
    return mbedtls_ecp_muladd(grp, R, m, P, n, Q);
}

/* --- Serialization --- */

/* aic_mbedtls_ecp_point_write_binary → mbedtls_ecp_point_write_binary */
int aic_mbedtls_ecp_point_write_binary(const mbedtls_ecp_group *grp,
                                       const mbedtls_ecp_point *P,
                                       int format, size_t *olen,
                                       unsigned char *buf, size_t buflen)
{
    return mbedtls_ecp_point_write_binary(grp, P, format, olen, buf, buflen);
}

/* aic_mbedtls_ecp_point_read_binary → mbedtls_ecp_point_read_binary */
int aic_mbedtls_ecp_point_read_binary(const mbedtls_ecp_group *grp,
                                      mbedtls_ecp_point *P,
                                      const unsigned char *buf, size_t ilen)
{
    return mbedtls_ecp_point_read_binary(grp, P, buf, ilen);
}

/* --- Checks --- */

/* aic_mbedtls_ecp_check_pubkey → mbedtls_ecp_check_pubkey */
int aic_mbedtls_ecp_check_pubkey(const mbedtls_ecp_group *grp,
                                 const mbedtls_ecp_point *pt)
{
    return mbedtls_ecp_check_pubkey(grp, pt);
}

/* aic_mbedtls_ecp_is_zero → mbedtls_ecp_is_zero */
int aic_mbedtls_ecp_is_zero(mbedtls_ecp_point *pt)
{
    return mbedtls_ecp_is_zero(pt);
}

/* aic_mbedtls_ecp_point_cmp → mbedtls_ecp_point_cmp */
int aic_mbedtls_ecp_point_cmp(const mbedtls_ecp_point *P,
                              const mbedtls_ecp_point *Q)
{
    return mbedtls_ecp_point_cmp(P, Q);
}

/* aic_mbedtls_ecp_set_zero → mbedtls_ecp_set_zero */
int aic_mbedtls_ecp_set_zero(mbedtls_ecp_point *pt)
{
    return mbedtls_ecp_set_zero(pt);
}
