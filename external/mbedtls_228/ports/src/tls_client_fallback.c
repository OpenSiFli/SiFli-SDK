/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>

#ifndef MBEDTLS_TLS_CLIENT_CONNECT_FALLBACK_ALIAS_GUARD
#define MBEDTLS_TLS_CLIENT_CONNECT_FALLBACK_ALIAS_GUARD 1
#endif
#include "tls_client.h"
#include "mbedtls/x509.h"

#define DBG_ENABLE
#define DBG_COLOR
#define DBG_SECTION_NAME    "mbedtls.clnt"
#ifdef MBEDTLS_DEBUG_C
#define DBG_LEVEL           DBG_LOG
#else
#define DBG_LEVEL           DBG_INFO
#endif /* MBEDTLS_DEBUG_C */
#include <rtdbg.h>

static void mbedtls_client_transport_reset(MbedTLSSession *session)
{
    if (session == RT_NULL)
    {
        return;
    }

    mbedtls_ssl_close_notify(&session->ssl);
    mbedtls_net_free(&session->server_fd);
    mbedtls_ssl_session_reset(&session->ssl);
}

static int mbedtls_client_set_authmode(MbedTLSSession *session, int authmode)
{
    if (session == RT_NULL)
    {
        return -RT_ERROR;
    }

    mbedtls_ssl_conf_authmode(&session->conf, authmode);

    return RT_EOK;
}

static unsigned int mbedtls_client_get_verify_flags(MbedTLSSession *session)
{
    if (session == RT_NULL)
    {
        return 0;
    }

    return mbedtls_ssl_get_verify_result(&session->ssl);
}

static int mbedtls_client_import_peer_root_ca(MbedTLSSession *session)
{
    int ret = 0;
    const mbedtls_x509_crt *chain = RT_NULL;
    const mbedtls_x509_crt *root = RT_NULL;

    if (session == RT_NULL)
    {
        return -RT_ERROR;
    }

    chain = mbedtls_ssl_get_peer_cert(&session->ssl);
    if (chain == RT_NULL)
    {
        LOG_E("peer certificate is null, enable MBEDTLS_SSL_KEEP_PEER_CERTIFICATE");
        return -RT_ERROR;
    }

    root = chain;
    while (root->next != RT_NULL)
    {
        root = root->next;
    }

    ret = mbedtls_x509_crt_parse_der(&session->cacert, root->raw.p, root->raw.len);
    if (ret != 0)
    {
        LOG_E("mbedtls_x509_crt_parse_der error, return -0x%x", -ret);
        return ret;
    }

    mbedtls_ssl_conf_ca_chain(&session->conf, &session->cacert, NULL);
    LOG_W("Import peer root/intermediate certificate success, len=%u", (unsigned int)root->raw.len);

    return RT_EOK;
}

/*
    * One-shot fallback flow :
    * 1) Try normal connect with peer verification.
    * 2) If verification fails with NOT_TRUSTED, reconnect with VERIFY_NONE to
    *    fetch the peer certificate chain.
    * 3) Import the last certificate in the received chain as a temporary trust
    *    anchor, then reconnect with peer verification again.
    *
    * Note:
    * - The last certificate in the received chain is NOT guaranteed to be the
    *   real root CA. Many TLS servers/platforms do not send the root CA during
    *   the handshake; therefore, the chain tail is often an intermediate CA.
    * - We intentionally take the chain tail as a pragmatic fallback anchor.
    */
int mbedtls_client_connect_with_fallback(MbedTLSSession *session)
{
    int ret = 0;
    int old_authmode = MBEDTLS_SSL_VERIFY_REQUIRED;
    unsigned int verify_flags = 0;

    if (session == RT_NULL)
    {
        return -RT_ERROR;
    }

    ret = mbedtls_client_connect(session);
    if (ret == RT_EOK)
    {
        return RT_EOK;
    }

    verify_flags = mbedtls_client_get_verify_flags(session);
    if ((verify_flags & MBEDTLS_X509_BADCERT_NOT_TRUSTED) == 0)
    {
        return ret;
    }

    LOG_W("TLS verify not trusted, start one-shot fallback to fetch peer root certificate");

    old_authmode = session->conf.authmode;
    mbedtls_client_transport_reset(session);
    mbedtls_client_set_authmode(session, MBEDTLS_SSL_VERIFY_NONE);

    ret = mbedtls_client_connect(session);
    if (ret != RT_EOK)
    {
        LOG_E("Fallback connect(VERIFY_NONE) failed, ret=-0x%x", -ret);
        mbedtls_client_set_authmode(session, old_authmode);
        mbedtls_client_transport_reset(session);
        return ret;
    }

    ret = mbedtls_client_import_peer_root_ca(session);
    mbedtls_client_transport_reset(session);
    mbedtls_client_set_authmode(session, old_authmode);
    if (ret != RT_EOK)
    {
        return ret;
    }

    ret = mbedtls_client_connect(session);
    if (ret == RT_EOK)
    {
        LOG_W("TLS fallback reconnect success");
    }

    return ret;
}
