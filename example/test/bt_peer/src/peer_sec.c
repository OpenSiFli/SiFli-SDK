/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pairing / bonding for the bt_peer instrument.
 *
 * sec.bond configures the IO capability and the passkey the instrument will
 * use, then starts bonding on the given link. When the stack asks for a
 * passkey to display, it is reported as `@evt passkey` so the host can type it
 * on the DUT.
 */

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>

#include "peer.h"
#include "bf0_sibles.h"
#include "ble_connection_manager.h"

static uint8_t g_io_cap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
static uint32_t g_passkey;
static uint8_t g_passkey_valid;

/** How long `sec.bond` waits for pairing to finish, in ms. */
#define PEER_SEC_BOND_TIMEOUT_MS     20000
/** Pairing and encryption are two steps; the second one is not instant. */
#define PEER_SEC_ENCRYPT_TIMEOUT_MS  5000

/** Link whose `sec.bond` is waiting, or PEER_NO_CONN when none is. */
static uint8_t g_bond_conn = PEER_NO_CONN;
/** Outcome of that bonding: 0 to pair, else the stack's reason. */
static int g_bond_rc;

static uint8_t peer_sec_io_parse(const char *io, int *ok)
{
    static const struct
    {
        const char *name;
        uint8_t cap;
    } s_names[] =
    {
        {"display_only", GAP_IO_CAP_DISPLAY_ONLY},
        {"display_yes_no", GAP_IO_CAP_DISPLAY_YES_NO},
        {"keyboard_only", GAP_IO_CAP_KB_ONLY},
        {"no_input_no_output", GAP_IO_CAP_NO_INPUT_NO_OUTPUT},
        {"keyboard_display", GAP_IO_CAP_KB_DISPLAY},
    };
    uint8_t i;

    for (i = 0; i < sizeof(s_names) / sizeof(s_names[0]); i++)
    {
        if (strcmp(io, s_names[i].name) == 0)
        {
            *ok = 1;
            return s_names[i].cap;
        }
    }

    /* An unknown name is not a capability; a bare number still is. */
    *ok = 0;
    return 0;
}

/**
 * Parse an authentication requirement: a name or the numeric @see gap_auth.
 *
 * Legacy and Secure Connections differ in what the pairing procedure can
 * negotiate, and a peer that needs encryption does not necessarily need SC, so
 * the suite picks the procedure instead of inheriting the build's default.
 */
static uint8_t peer_sec_auth_parse(const char *auth, int *ok)
{
    static const struct
    {
        const char *name;
        uint8_t auth;
    } s_names[] =
    {
        {"none", GAP_AUTH_REQ_NO_MITM_NO_BOND},
        {"legacy", GAP_AUTH_REQ_NO_MITM_BOND},
        {"legacy_mitm", GAP_AUTH_REQ_MITM_BOND},
        {"secure", GAP_AUTH_REQ_SEC_CON_BOND},
        {"secure_mitm", GAP_AUTH_REQ_SEC_CON_MITM_BOND},
    };
    uint8_t i;

    for (i = 0; i < sizeof(s_names) / sizeof(s_names[0]); i++)
    {
        if (strcmp(auth, s_names[i].name) == 0)
        {
            *ok = 1;
            return s_names[i].auth;
        }
    }

    *ok = 0;
    return 0;
}

void peer_sec_init(void)
{
    g_io_cap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
    g_passkey = 0;
    g_passkey_valid = 0;
}

int peer_sec_bond(peer_link_t *link, const char *io, const char *auth,
                  uint8_t has_passkey, uint32_t passkey, peer_reply_t *reply)
{
    int ok = 0;
    uint8_t cap = g_io_cap;
    int waited;

    if (io != RT_NULL && io[0] != '\0')
    {
        cap = peer_sec_io_parse(io, &ok);
        if (!ok)
        {
            /* A bare integer is also accepted, for a host that sends the enum. */
            char *end = RT_NULL;
            long parsed = strtol(io, &end, 0);

            if (end == io || *end != '\0' || parsed < 0 || parsed >= GAP_IO_CAP_LAST)
            {
                peer_reply_err(reply, PEER_ERR_INVALID_ARG, "io");
                return -1;
            }
            cap = (uint8_t)parsed;
        }
    }

    if (link == RT_NULL)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "link");
        return -1;
    }

    if (auth != RT_NULL && auth[0] != '\0')
    {
        uint8_t wanted = peer_sec_auth_parse(auth, &ok);

        if (!ok)
        {
            char *end = RT_NULL;
            long parsed = strtol(auth, &end, 0);

            if (end == auth || *end != '\0')
            {
                peer_reply_err(reply, PEER_ERR_INVALID_ARG, "auth");
                return -1;
            }
            wanted = (uint8_t)parsed;
        }

        /* The stack owns the value table: it rejects what it cannot negotiate. */
        if (connection_manager_set_bond_cnf_auth(wanted) != CM_STATUS_OK)
        {
            peer_reply_err(reply, PEER_ERR_INVALID_ARG, "auth");
            return -1;
        }
    }

    g_io_cap = cap;
    /*
     * PENDING is what makes the connection manager publish
     * CONNECTION_MANAGER_BOND_AUTH_INFOR. Without it the default BOND_ACCEPT
     * path answers the stack itself, so no `@evt passkey` can ever be emitted.
     */
    connection_manager_set_bond_ack(BOND_PENDING);
    connection_manager_set_bond_cnf_iocap(cap);

    if (has_passkey)
    {
        g_passkey = passkey;
        g_passkey_valid = 1;
        connection_manager_bond_pin_code_set(passkey);
    }

    g_bond_rc = -1;
    g_bond_conn = link->conn_idx;
    peer_ble_clear_bits(PEER_BIT_BOND_DONE);
    connection_manager_create_bond(link->conn_idx);

    /*
     * Pairing is the first half: the terminal @ok must not go out until the
     * link is actually encrypted, because that is when a protected attribute
     * (a HID CCCD, for one) starts accepting writes. The stack does not hand
     * us an encryption indication on every path, so poll the state it keeps.
     */
    if (peer_ble_wait(PEER_BIT_BOND_DONE, PEER_SEC_BOND_TIMEOUT_MS) != RT_EOK)
    {
        g_bond_conn = PEER_NO_CONN;
        peer_reply_err(reply, PEER_ERR_TIMEOUT, "sec.bond");
        return -1;
    }

    if (g_bond_rc != 0)
    {
        g_bond_conn = PEER_NO_CONN;
        peer_reply_err(reply, PEER_ERR_BOND_FAILED, "sec.bond rc=%d", g_bond_rc);
        return -1;
    }

    for (waited = 0; waited < PEER_SEC_ENCRYPT_TIMEOUT_MS; waited += 10)
    {
        if (connection_manager_get_enc_state(link->conn_idx) == ENC_STATE_ON)
            break;
        rt_thread_mdelay(10);
    }

    g_bond_conn = PEER_NO_CONN;

    if (connection_manager_get_enc_state(link->conn_idx) != ENC_STATE_ON)
    {
        peer_reply_err(reply, PEER_ERR_BOND_FAILED, "sec.bond unencrypted");
        return -1;
    }

    peer_reply_ok(reply, "link=%u io=%u", link->link_id, cap);
    return 0;
}

int peer_sec_clear(peer_reply_t *reply)
{
    uint8_t status = connection_manager_delete_all_bond();

    g_passkey = 0;
    g_passkey_valid = 0;

    if (status != CM_STATUS_OK && status != CM_FLASH_PEDING)
    {
        peer_reply_err(reply, PEER_ERR_STATE, "sec.clear");
        return -1;
    }

    peer_reply_ok(reply, "");
    return 0;
}

static int peer_sec_event_handler(uint16_t event_id, uint8_t *data, uint16_t len,
                                  uint32_t context)
{
    (void)len;
    (void)context;

    /* Pairing finishes asynchronously; wake the `sec.bond` that asked for it. */
    if (event_id == CONNECTION_MANAGER_PAIRING_SUCCEED ||
            event_id == CONNECTION_MANAGER_PAIRING_FAILED)
    {
        ble_gap_bond_ind_t *ind = (ble_gap_bond_ind_t *)data;

        if (ind->conn_idx == g_bond_conn)
        {
            g_bond_rc = event_id == CONNECTION_MANAGER_PAIRING_SUCCEED
                        ? 0 : (int)ind->data.reason;
            peer_ble_signal(PEER_BIT_BOND_DONE);
        }
        return 0;
    }

    if (event_id != CONNECTION_MANAGER_BOND_AUTH_INFOR)
        return 0;

    {
        connection_manager_bond_ack_infor_t *info =
            (connection_manager_bond_ack_infor_t *)data;
        peer_link_t *link = peer_link_find_conn(info->conn_idx);
        uint8_t link_id = link != RT_NULL ? link->link_id : 0;

        if (info->request == GAPC_PAIRING_REQ)
        {
            connection_manager_bond_ack_reply(info->conn_idx, GAPC_PAIRING_REQ, true);
        }
        else if (info->request == GAPC_TK_EXCH)
        {
            if (info->type == GAP_TK_DISPLAY)
            {
                peer_emit_evt("passkey", "link=%u value=%u", link_id,
                              (unsigned)info->confirm_data);
                connection_manager_bond_ack_reply(info->conn_idx, GAPC_TK_EXCH, true);
            }
            else if (info->type == GAP_TK_KEY_ENTRY)
            {
                /* Always answer: leaving the key exchange unanswered hangs
                 * pairing. A host that configured a passkey gets it; otherwise
                 * the defined default 000000 is used. */
                connection_manager_bond_pin_code_set(
                    g_passkey_valid ? g_passkey : 0);
                connection_manager_bond_ack_reply(info->conn_idx, GAPC_TK_EXCH, true);
            }
            else
            {
                connection_manager_bond_ack_reply(info->conn_idx, GAPC_TK_EXCH, true);
            }
        }
        else if (info->request == GAPC_NC_EXCH)
        {
            connection_manager_bond_ack_reply(info->conn_idx, GAPC_NC_EXCH, true);
        }
    }

    return 0;
}

BLE_EVENT_REGISTER(peer_sec_event_handler, NULL);
