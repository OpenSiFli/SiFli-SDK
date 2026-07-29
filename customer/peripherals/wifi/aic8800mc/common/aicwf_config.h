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

#ifndef _AICWF_CONFIG_H_
#define _AICWF_CONFIG_H_

/**
 * @defgroup SOFTWARE CONFIG
 * @brief macro defines
 * @{
 */

#include "user_config.h"

#if (AIC_CFG_RDATA_MODE_EN)
    #define CONFIG_RAWDATA_MODE
#endif
#if (AIC_CFG_VNET_MODE_EN)
    #define CONFIG_VNET_MODE
    #define CONFIG_DATA_TXRX
#endif

//#define CONFIG_RX_REORDER

/**
 * @}
 */

#endif /* _AICWF_CONFIG_H_ */
