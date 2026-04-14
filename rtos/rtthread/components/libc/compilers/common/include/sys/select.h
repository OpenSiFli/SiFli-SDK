/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-07-21     Meco Man     The first version
 * 2021-12-25     Meco Man     Handle newlib 2.2.0 or lower version
 */

#ifndef __SYS_SELECT_H__
#define __SYS_SELECT_H__

#ifndef RT_USING_NEWLIB
    #include "libc/libc_fdset.h"
    #include "time.h"

    int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout);
#endif /* RT_USING_NEWLIB */


#endif /* __SYS_SELECT_H__ */

