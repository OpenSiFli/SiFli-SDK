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

void peer_sec_init(void)
{
    g_io_cap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
    g_passkey = 0;
    g_passkey_valid = 0;
}

int peer_sec_bond(peer_link_t *link, const char *io, uint8_t has_passkey,
                  uint32_t passkey, peer_reply_t *reply)
{
    int ok = 0;
    uint8_t cap = g_io_cap;

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

    connection_manager_create_bond(link->conn_idx);

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
