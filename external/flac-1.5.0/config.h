/*
 * SPDX-FileCopyrightText: 2025-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

/* Endianness */
#define CPU_IS_LITTLE_ENDIAN 1

#define FLAC__NO_ASM 1

#define FLAC__HAS_OGG 0

#define HAVE_STDINT_H 1
#define HAVE_STDBOOL_H 1
#define HAVE_STRING_H 1
#define HAVE_STDLIB_H 1
#define HAVE_STDIO_H 1
#define HAVE_INTTYPES_H 1

#define HAVE_LSTAT 0

#ifndef PACKAGE_VERSION
#define PACKAGE_VERSION "1.5.0"
#endif

#define HAVE_LROUND 1

/* Common compiler builtins */
#define HAVE_BSWAP16 1
#define HAVE_BSWAP32 1

