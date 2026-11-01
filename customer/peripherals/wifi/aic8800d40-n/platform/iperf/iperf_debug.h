/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _IPERF_DEBUG_H_
#define _IPERF_DEBUG_H_

#if 1 //PRJCONF_NET_EN

#ifndef PLATFORM_NO_FILE
    #include <stdio.h>
#else
    #ifndef EOF
        #define EOF (-1)
    #endif
#endif
#include "aic_log.h"
#ifdef __cplusplus
extern "C" {
#endif

#define sys_abort() while (1)

#define IPERF_DBG_ON    1
#define IPERF_WARN_ON   1
#define IPERF_ERR_ON    1
#define IPERF_ABORT_ON  0

#define IPERF_SYSLOG    AIC_LOG_PRINTF
#define IPERF_ABORT()   sys_abort()

#define IPERF_LOG(flags, fmt, arg...)   \
    do {                                \
        if (flags)                      \
            IPERF_SYSLOG(fmt, ##arg);   \
    } while (0)

#define IPERF_DBG(fmt, arg...)  IPERF_LOG(IPERF_DBG_ON, "[iperf] "fmt"\n\r", ##arg)
#define IPERF_WARN(fmt, arg...) IPERF_LOG(IPERF_WARN_ON, "[iperf WARN] "fmt"\r", ##arg)
#define IPERF_ERR(fmt, arg...)                              \
    do {                                                    \
        IPERF_LOG(IPERF_ERR_ON, "[iperf ERR] %s():%d, "fmt, \
               __func__, __LINE__, ##arg);                  \
        if (IPERF_ABORT_ON)                                 \
            IPERF_ABORT();                                  \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif /* PRJCONF_NET_EN */
#endif /* _IPERF_DEBUG_H_ */
