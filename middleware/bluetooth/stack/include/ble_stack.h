/**
  ******************************************************************************
  * @file   ble_stack.h
  * @author Sifli software development team
  * @brief SIFLI BLE stack external interface.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2020 - 2020,  Sifli Technology
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


#ifndef __BLE_STACK_H
#define __BLE_STACK_H

#if defined(BSP_BLE_SIBLES) && !defined(BSP_USING_PC_SIMULATOR)
    #include "bf0_sibles_util.h"
#endif

#include <rtthread.h>
#include <rtdevice.h>
//#include "ipc_queue.h"

#ifdef SOC_SF32LB55X
typedef struct
{
    uint8_t *env_buf;
    uint8_t *msg_buf;
    uint8_t *att_buf;
    uint8_t *nt_buf;
    uint8_t *log_buf;
    uint16_t env_buf_size;
    uint16_t msg_buf_size;
    uint16_t att_buf_size;
    uint16_t nt_buf_size;
    uint16_t log_buf_size;
    int8_t max_nb_of_hci_completed;
} ble_mem_config_t;
#else
typedef struct
{
    uint8_t *env_buf;
    uint8_t *msg_buf;
    uint8_t *att_buf;
    uint8_t *nt_buf;
    uint8_t *log_buf;
    uint8_t *nvds_buf;
    uint16_t env_buf_size;
    uint16_t msg_buf_size;
    uint16_t att_buf_size;
    uint16_t nt_buf_size;
    uint16_t log_buf_size;
    uint16_t nvds_buf_size;
    int8_t max_nb_of_hci_completed;
} ble_mem_config_t;
#endif

// Order should not be changed
typedef enum
{
    ROM_EM_BLE_START_OFFSET = 0x0,
    ROM_EM_BLE_CS_OFFSET,
    ROM_EM_BLE_WPAL_OFFSET,
    ROM_EM_BLE_RAL_OFFSET,
    ROM_EM_BLE_RX_DESC_OFFSET,
    ROM_EM_BLE_TX_DESC_OFFSET,
    ROM_EM_BLE_LLCPTXBUF_OFFSET,
    ROM_EM_BLE_ADVEXTHDRTXBUF_OFFSET,
    ROM_EM_BLE_ADVDATATXBUF_OFFSET,
    ROM_EM_BLE_AUXCONNECTREQTXBUF_OFFSET,
    ROM_EM_BLE_DATARXBUF_OFFSET,
    ROM_EM_BLE_ACLTXBUF_OFFSET,
    ROM_EM_BLE_ISO_HOP_SEQ_OFFSET,
    ROM_EM_BLE_ISO_DESC_OFFSET,
    ROM_EM_BLE_ISO_BUF_OFFSET,
    ROM_EM_BLE_RX_CTE_DESC_OFFSET,
    ROM_EM_BLE_TX_ANTENNA_ID_OFFSET,
    ROM_EM_BLE_RX_ANTENNA_ID_OFFSET,
    ROM_EM_BLE_END,
    ROM_EM_BT_CS_OFFSET,
    ROM_EM_BT_RXDESC_OFFSET,
    ROM_EM_BT_TXDESC_OFFSET,
    ROM_EM_BT_LMPRXBUF_OFFSET,
    ROM_EM_BT_LMPTXBUF_OFFSET,
    ROM_EM_BT_ISCANFHSTXBUF_OFFSET,
    ROM_EM_BT_PAGEFHSTXBUF_OFFSET,
    ROM_EM_BT_EIRTXBUF_OFFSET,
    ROM_EM_BT_LOCAL_SAM_SUBMAP_OFFSET,
    ROM_EM_BT_PEER_SAM_MAP_OFFSET,
    ROM_EM_BT_STPTXBUF_OFFSET,
    ROM_EM_BT_ACLRXBUF_OFFSET,
    ROM_EM_BT_ACLTXBUF_OFFSET,
    ROM_EM_BT_AUDIOBUF_OFFSET,
    ROM_EM_BT_END,
    ROM_EM_END
} rom_em_offset_t;

typedef struct
{
    uint16_t size;
    uint16_t cnt;
} rom_em_default_attr_t;


typedef int (*sifli_msg_func_t)(uint16_t const msgid, void const *param,
                                uint16_t const dest_id, uint16_t const src_id);

typedef int (*tl_data_callback_t)(uint8_t *bufptr, uint32_t size);


int bluetooth_init(void);

void ble_boot(sifli_msg_func_t callback);

#if defined(BSP_BLE_SIBLES) && !defined(BSP_USING_PC_SIMULATOR)
void *ble_stack_msg_alloc(sifli_msg_id_t const id, sifli_task_id_t const dest_id,
                          sifli_task_id_t const src_id, uint16_t const param_len);

void ble_stack_msg_send(void const *param_ptr);
#endif

void bluetooth_stack_ipc_register_tx_cb(tl_data_callback_t callback);

int bluetooth_stack_ipc_write_data(uint8_t *bufptr, uint32_t size);

void bluetooth_config(void);


#endif // __BLE_STACK_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/


