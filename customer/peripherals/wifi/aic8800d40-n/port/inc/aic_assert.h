/*
 * Copyright (C) 2018-2026 AICSemi Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Shim header replacing platform/inc/aic_assert.h
 */

#ifndef _AIC_ASSERT_H_
#define _AIC_ASSERT_H_

#include "aic_log.h"

#ifndef AIC_ASSERT_ERR
#define AIC_ASSERT_ERR(cond)        do { if (!(cond)) { \
                                            DBG_CMN_CRT("Assertion failed: %s, file %s, line %d\n", #cond, __FILE__, __LINE__); \
                                            while (1); \
                                        } } while (0)
#define ASSERT(cond)
#endif

#endif /* _AIC_ASSERT_H_ */
