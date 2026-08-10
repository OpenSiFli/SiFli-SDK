/**
 * @attention
 * Copyright (c) 2018-2025 AICSemi Ltd. All rights reserved.
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
 
#ifndef PLATFORM_H
#define PLATFORM_H

#include "includes.h"
#include "common.h"

#define le16_to_cpu		le_to_host16
#define le32_to_cpu		le_to_host32

#define get_unaligned(p)					\
({								\
	struct packed_dummy_struct {				\
		typeof(*(p)) __val;				\
	} __attribute__((packed)) *__ptr = (void *) (p);	\
								\
	__ptr->__val;						\
})
#define get_unaligned_le16(p)	le16_to_cpu(get_unaligned((le16 *)(p)))
#define get_unaligned_le32(p)	le32_to_cpu(get_unaligned((le32 *)(p)))

#endif /* PLATFORM_H */
