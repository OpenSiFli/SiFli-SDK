/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _LMAC_TYPES_H_
#define _LMAC_TYPES_H_


/**
 ****************************************************************************************
 * @addtogroup CO_INT
 * @ingroup COMMON
 * @brief Common integer standard types (removes use of stdint)
 *
 * @{
 ****************************************************************************************
 */

#include "co_bool.h"
#include "aicwf_types.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#if 0
    typedef unsigned char u8;
    typedef unsigned short u16;
    typedef unsigned int u32;
#endif

typedef unsigned char u8_l;
typedef signed char s8_l;
typedef bool bool_l;
typedef unsigned short u16_l;
typedef signed short s16_l;
typedef unsigned int u32_l;
typedef signed int s32_l;
typedef unsigned long long u64_l;

/// @} CO_INT
#endif // _LMAC_TYPES_H_
