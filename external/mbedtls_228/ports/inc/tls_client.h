/*
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */
 
#ifndef MBEDTLS_CLIENT_H
#define MBEDTLS_CLIENT_H

#include "mbedtls/platform.h"
#include "mbedtls/net.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/certs.h"

typedef struct MbedTLSSession
{
    char* host;
    char* port;

    unsigned char *buffer;
    size_t buffer_len;
    
    mbedtls_ssl_context ssl;
    mbedtls_ssl_config conf;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_net_context server_fd;
    mbedtls_x509_crt cacert;
}MbedTLSSession;
 
 extern int mbedtls_client_init(MbedTLSSession *session, void *entropy, size_t entropyLen);
 extern int mbedtls_client_close(MbedTLSSession *session);
 extern int mbedtls_client_context(MbedTLSSession *session);
 extern int mbedtls_client_connect(MbedTLSSession *session);
 extern int mbedtls_client_connect_with_fallback(MbedTLSSession *session);
 extern int mbedtls_client_read(MbedTLSSession *session, unsigned char *buf , size_t len);
 extern int mbedtls_client_write(MbedTLSSession *session, const unsigned char *buf , size_t len);

/*
 * Compatibility option:
 * Redirect calls of mbedtls_client_connect() to the fallback implementation
 * at compile-time, so upper-layer modules (e.g. webclient) don't need to be
 * patched to gain dynamic certificate fetching capability.
 *
 * This alias is enabled globally by Kconfig option
 * PKG_USING_MBEDTLS_TLS_CLIENT_CONNECT_FALLBACK_ALIAS.
 *
 * Implementation files should define
 * MBEDTLS_TLS_CLIENT_CONNECT_FALLBACK_ALIAS_GUARD before
 * including this header to avoid macro expansion on the function definition.
 */
#if defined(MBEDTLS_SSL_KEEP_PEER_CERTIFICATE) && \
    defined(PKG_USING_MBEDTLS_TLS_CLIENT_CONNECT_FALLBACK_ALIAS) && \
    !defined(MBEDTLS_TLS_CLIENT_CONNECT_FALLBACK_ALIAS_GUARD)
#define mbedtls_client_connect(session) mbedtls_client_connect_with_fallback(session)
#endif

#endif
