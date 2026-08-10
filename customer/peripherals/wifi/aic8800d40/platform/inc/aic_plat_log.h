/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AIC__PLAT_LOG_H_
#define _AIC__PLAT_LOG_H_

#include "plat_port.h"
#include "aic8800_config.h"
#include "osal_debug.h"

#ifdef AIC_LOG_DEBUG_ON
#define AIC_LOG_PRINTF(fmt, ...)    {DBG_OSAL_INF("[AIC_I]"fmt, ##__VA_ARGS__);}
#define AIC_LOG_TRACE(fmt, ...)     {DBG_OSAL_VRB("[AIC_T]"fmt, ##__VA_ARGS__);}
#define AIC_LOG_ERROR(fmt, ...)     {DBG_OSAL_ERR("[AIC_E]"fmt, ##__VA_ARGS__);}

#else
#define AIC_LOG_PRINTF(...)         do { } while(0)
#define AIC_LOG_TRACE(...)          do { } while(0)
#define AIC_LOG_ERROR(...)          do { } while(0)
#endif /* AIC_LOG_DEBUG_ON */

#define aic_dbg                     AIC_LOG_PRINTF


#endif /* _AIC_LOG_H_ */

