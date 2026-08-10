/**
  ******************************************************************************
  * @file   netal_sock.c
  * @author AIC software development team
  ******************************************************************************
*/
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

#include "netal_sock.h"

#ifdef RT_USING_SAL
#include "sys/socket.h"
#else
#include "lwip/sockets.h"
#endif

int netal_accept(int s, void *addr, netal_socklen_t *addrlen)
{
    return accept(s, addr, addrlen);
}

int netal_bind(int s, const void *name, netal_socklen_t namelen)
{
    return bind(s, name, namelen);
}

int netal_shutdown(int s, int how)
{
    return shutdown(s, how);
}

int netal_getpeername(int s, void *name, netal_socklen_t *namelen)
{
    return getpeername(s, name, namelen);
}

int netal_getsockname(int s, void *name, netal_socklen_t *namelen)
{
    return getsockname(s, name, namelen);
}

int netal_getsockopt(int s, int level, int optname, void *optval, netal_socklen_t *optlen)
{
    return getsockopt(s, level, optname, optval, optlen);
}

int netal_setsockopt(int s, int level, int optname, const void *optval, netal_socklen_t optlen)
{
    return setsockopt(s, level, optname, optval, optlen);
}

int netal_connect(int s, const void *name, netal_socklen_t namelen)
{
    return connect(s, name, namelen);
}

int netal_listen(int s, int backlog)
{
    return listen(s, backlog);
}

int netal_recv(int s, void *mem, size_t len, int flags)
{
    return recv(s, mem, len, flags);
}

int netal_recvfrom(int s, void *mem, size_t len, int flags,
                   void *from, netal_socklen_t *fromlen)
{
    return recvfrom(s, mem, len, flags, from, fromlen);
}

int netal_recvmsg(int s, void *message, int flags)
{
    return recvmsg(s, message, flags);
}

int netal_sendmsg(int s, const void *message, int flags)
{
    return sendmsg(s, message, flags);
}

int netal_send(int s, const void *dataptr, size_t size, int flags)
{
    return send(s, dataptr, size, flags);
}

int netal_sendto(int s, const void *dataptr, size_t size, int flags,
                 const void *to, netal_socklen_t tolen)
{
    return sendto(s, dataptr, size, flags, to, tolen);
}

int netal_socket(int domain, int type, int protocol)
{
    return socket(domain, type, protocol);
}

int netal_closesocket(int s)
{
    return closesocket(s);
}

int netal_ioctlsocket(int s, long cmd, void *arg)
{
    return ioctlsocket(s, cmd, arg);
}

int netal_select(int nfds, void *readfds, void *writefds, void *exceptfds, void *timeout)
{
    return select(nfds, readfds, writefds, exceptfds, timeout);
}
