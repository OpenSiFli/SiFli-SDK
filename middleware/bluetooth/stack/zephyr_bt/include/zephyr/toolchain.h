/*
 * Copyright (c) 2010-2014, Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Macros to abstract toolchain specific capabilities
 *
 * This file contains various macros to abstract compiler capabilities that
 * utilize toolchain specific attributes and/or pragmas.
 */

#ifndef ZEPHYR_INCLUDE_TOOLCHAIN_H_
#define ZEPHYR_INCLUDE_TOOLCHAIN_H_

#include <rtt4zephyr.h>

#define Z_STRINGIFY(x) #x
#define STRINGIFY(s) Z_STRINGIFY(s)

/* concatenate the values of the arguments into one */
#define _DO_CONCAT(x, y) x ## y
#define _CONCAT(x, y) _DO_CONCAT(x, y)


#define ___in_section(a, b, c) \
    __attribute__((section("." Z_STRINGIFY(a)           \
                "." Z_STRINGIFY(b)          \
                "." Z_STRINGIFY(c))))
#define __in_section(a, b, c) ___in_section(a, b, c)

#endif /* ZEPHYR_INCLUDE_TOOLCHAIN_H_ */
