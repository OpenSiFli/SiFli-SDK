/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "framed_proto_link.h"
#include <rtthread.h>

RT_WEAK int framed_proto_pan_send_data(const void *data, uint32_t data_len)
{
    (void)data;
    (void)data_len;
    return -1;
}

RT_WEAK uint8_t framed_proto_pan_is_active(void)
{
    return 0;
}

RT_WEAK uint8_t framed_proto_pan_is_wifi(void)
{
    return 0;
}

uint8_t framed_proto_link_type_get(void)
{
    if (!framed_proto_pan_is_active())
    {
        return FRAMED_PROTO_LINK_BLE;
    }

    /* The relay is up: either on WiFi or on Bluetooth PAN. */
    return framed_proto_pan_is_wifi() ? FRAMED_PROTO_LINK_WIFI : FRAMED_PROTO_LINK_PAN;
}

void framed_proto_link_assemble_mode_no_free_set(void)
{
    /* TCP delivers whole frames, so there is no streaming assembly state. */
}

void framed_proto_link_assemble_clean(void)
{
}
