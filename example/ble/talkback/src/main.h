/*
 * SPDX-FileCopyrightText: 2024-2024 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MAIN_H_
#define MAIN_H_


#include "bf0_ble_gap.h"

#include "audio_server.h"
#include "log.h"

#ifdef __cplusplus
extern "C" {
#endif
#undef LOG_TAG
#define LOG_TAG "ble_app"


#define APP_MIC_ALWAYS_ON

#define APP_MAX_PER_ADV_LEN (100)

#define DEFAULT_NETWORK_CODE "0000"

#define DEFAULT_NETWORK_LEN 4

#define MAX_SYNCD_DEVICE 2

typedef struct
{
    struct rt_work work;
    uint8_t state;
    uint8_t is_stopping;
} app_send_env_t;

typedef struct
{
    uint8_t sync_idx;
    uint8_t dev_state;
    ble_gap_addr_t addr;
} sync_info_t;

typedef struct
{
    struct rt_work work;
    uint8_t state;
    uint8_t is_scaning;
    uint8_t is_scan_restart;
    // Periodic sync
    sync_info_t sync_dev[MAX_SYNCD_DEVICE];
    uint8_t sync_created_dev;
    uint8_t synced_dev_num;
    uint8_t syncing_idx;
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
void ble_talk_downlink(uint8_t actv_idx, uint8_t *data, uint16_t data_len);

int talk_init(audio_rwflag_t flag);

int talk_deinit(void);

uint8_t ble_app_sender_trigger(void);

uint8_t ble_app_sender_stop(void);

uint8_t ble_app_scan_init(void);

void ble_app_peri_advertising_init(void);

uint8_t ble_app_scan_restart(void);

uint8_t ble_app_sender_is_working(void);


#ifdef __cplusplus
}
#endif
#endif // MAIN_H_

