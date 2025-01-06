/**
  ******************************************************************************
  * @file   bf0_ble_hci.c
  * @author Sifli software development team
  * @brief SIFLI BLE utility source.
*
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <string.h>
#include <stdlib.h>
#include "bf0_ble_hci.h"
#include "bf0_ble_err.h"
#include "bf0_sibles_util.h"

#if defined(SOC_SF32LB55X) && defined(SOC_BF0_HCPU)
/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */


typedef __PACKED_STRUCT
{
    uint8_t event_id;
    uint8_t len;
    uint8_t data[__ARRAY_EMPTY];
} ble_hci_event_header_t;


typedef __PACKED_STRUCT
{
    uint8_t type;
    uint16_t opcode;
    uint8_t len;
    uint8_t param[__ARRAY_EMPTY];
} ble_hci_command_msg_t;

typedef __PACKED_STRUCT
{
    uint8_t type;
    uint16_t hdl_flag;
    uint16_t len;
    uint8_t data[__ARRAY_EMPTY];
} ble_hci_data_msg_t;


uint8_t ble_hci_send_command(ble_hci_command_t *command)
{
    // Whether could use macro?
    if (command == NULL)
    {
        return LL_ERR_INVALID_HCI_PARAM;
    }
    uint8_t ret = HL_ERR_NO_ERROR;
    ipc_queue_handle_t dev = sifli_get_mbox_write_dev();
    int written;
    uint16_t len = sizeof(ble_hci_command_t) + command->len;
    ble_hci_command_msg_t *msg = (ble_hci_command_msg_t *) bt_mem_alloc(len);
    if (msg == NULL)
        return LL_ERR_MEMORY_CAPA_EXCEED;

    msg->type = HCI_CMD_MSG_TYPE;
    msg->opcode = command->opcode;
    msg->len = command->len;
    memcpy(msg->param, command->data, command->len);


    written = ipc_queue_write(dev, msg, len, 10);
    RT_ASSERT(len == written);
    bt_mem_free(msg);

    return ret;
}

uint8_t ble_hci_send_data(ble_hci_data_t *data)
{
    if (data == NULL)
    {
        return LL_ERR_INVALID_HCI_PARAM;
    }
    uint8_t ret = HL_ERR_NO_ERROR;
    ipc_queue_handle_t dev = sifli_get_mbox_write_dev();
    int written;
    uint16_t len = sizeof(ble_hci_data_msg_t) + data->len;

    ble_hci_data_msg_t *msg = (ble_hci_data_msg_t *) bt_mem_alloc(len);
    if (msg == NULL)
        return LL_ERR_MEMORY_CAPA_EXCEED;

    msg->type = HCI_ACL_MSG_TYPE;
    // Handle + PB_flag + BC_flag, 2 bytes
    memcpy((void *)&msg->hdl_flag, &data->len, 2);
    msg->len = data->len;
    memcpy(msg->data, data->data, data->len);

    written = ipc_queue_write(dev, msg, len, 10);
    RT_ASSERT(len == written);
    bt_mem_free(msg);

    return ret;
}




void sifli_hci_process(void)
{
    ipc_queue_handle_t dev = sifli_get_mbox_read_dev();
    uint8_t *buf = sifli_get_mbox_buffer();
    ipc_queue_read(dev, buf, sizeof(ble_hci_event_header_t));
    ble_hci_event_header_t *header = (ble_hci_event_header_t *)buf;
    uint8_t *buf_ptr = (uint8_t *)(&header->data);
    ipc_queue_read(dev, buf_ptr, header->len);
    ble_hci_evt_t evt;
    evt.event_id = header->event_id;
    evt.len = header->len;
    evt.data = buf_ptr;

    ble_event_publish(BLE_HCI_EVENT_IND, &evt, sizeof(ble_hci_evt_t));
    // Notify application
}


#endif // defined(SOC_SF32LB55X) && defined(SOC_BF0_HCPU)


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
