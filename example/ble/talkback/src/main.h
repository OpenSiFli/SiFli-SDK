/**
  ******************************************************************************
  * @file   main.h
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2024 - 2024,  Sifli Technology
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

#ifndef MAIN_H_
#define MAIN_H_


#define LOG_TAG "ble_app"
#include "audio_server.h"

#include "log.h"

#define APP_MIC_ALWAYS_ON


#define APP_MAX_PER_ADV_LEN (100)

#define DEFAULT_NETWORK_CODE "0000"

#define DEFAULT_NETWORK_LEN 4

#define MAX_SYNCD_DEVICE 1


typedef struct
{
    struct rt_delayed_work work;
    uint8_t state;
    uint8_t is_stopping;
} app_send_env_t;

typedef struct
{
    struct rt_delayed_work work;
    uint8_t state;
    uint8_t is_scaning;
    uint8_t is_scan_restart;
    // Periodic sync
    uint8_t sync_idx;
    uint8_t synced_dev;
} app_recv_env_t;

typedef struct
{
    uint8_t is_power_on;

    uint8_t click;
    // Mbox thread
    rt_mailbox_t mb_handle;
    app_send_env_t s_env;
    app_recv_env_t r_env;
} app_env_t;

app_env_t *ble_app_get_env(void);

int ble_app_sender_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context);

void ble_app_sender_init(void);

void ble_app_receviver_init(void);


int ble_app_receiver_event_handler(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context);

uint8_t app_send_voice_data(uint16_t len, uint8_t *voice_data);
void ble_talk_downlink(uint8_t *data, uint16_t data_len);

int talk_init(audio_rwflag_t flag);

int talk_deinit(void);

uint8_t ble_app_sender_trigger(void);

uint8_t ble_app_sender_stop(void);

uint8_t ble_app_scan_init(void);

void ble_app_peri_advertising_init(void);

uint8_t ble_app_scan_restart(void);

uint8_t ble_app_sender_is_working(void);


#endif // MAIN_H_
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
