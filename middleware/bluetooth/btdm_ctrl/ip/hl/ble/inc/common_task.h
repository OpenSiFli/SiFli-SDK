/**
  ******************************************************************************
  * @file   common_task.h
  * @author Sifli software development team
  * @brief Header file - Handle common command.
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2023 - 2023,  Sifli Technology
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



#ifndef COMMON_TASK_H_
#define COMMON_TASK_H_




enum comm_msg_id
{
    /// Enter BT test mode
    COMM_BT_TEST_MODE_CTRL_CMD = TASK_FIRST_MSG(TASK_ID_COMMON),
    /// Indicate response of BT test mode
    COMM_BT_TEST_MODE_CTRL_RSP,
};


enum bt_test_operation_t
{
    BT_TEST_OP_ENTER_TEST,
    BT_TEST_OP_EXIT_TEST,
    BT_TEST_OP_TX_TEST,
    BT_TEST_OP_RX_TEST,
    BT_TEST_OP_STOP_TEST,
};


struct bt_test_mode_tx_para_t
{
    uint8_t channel;
    uint8_t pkt_payload;
    uint8_t pkt_type;
    uint8_t pwr_lvl;
    uint16_t pkt_len;
};

struct bt_test_mode_rx_para_t
{
    uint8_t channel;
    uint8_t pkt_type;
};

struct bt_test_mode_stop_test_para_t
{
    uint16_t cnt;
};


typedef union
{
    struct bt_test_mode_tx_para_t tx_para;
    struct bt_test_mode_rx_para_t rx_para;
} bt_test_mode_cmd_para_t;

typedef union
{
    struct bt_test_mode_stop_test_para_t stop_para;
} bt_test_mode_rsp_para_t;


struct bt_test_mode_ctrl_cmd
{
    enum bt_test_operation_t op;
    bt_test_mode_cmd_para_t para;
};

struct bt_test_mode_ctrl_rsp
{
    enum bt_test_operation_t op;
    uint8_t status;
    bt_test_mode_rsp_para_t para;
};



#endif // COMMON_TASK_H_

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
