/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef APP_NVM_H
#define APP_NVM_H

#include <stddef.h>
#include "rtthread.h"

#ifdef __cplusplus
extern "C" {
#endif

size_t app_nvm_read(const char *key_name, const void *data, size_t length);
rt_err_t app_nvm_write(const char *key_name, const void *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
