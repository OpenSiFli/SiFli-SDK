/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file fhost_config.h
 *
 * @brief Definition of configuration for Fully Hosted firmware.
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */
#ifndef _FHOST_CONFIG_H_
#define _FHOST_CONFIG_H_

/**
 ****************************************************************************************
 * @addtogroup FHOST
 * @{
 ****************************************************************************************
 */

#include "rwnx_config.h"
#include "fhost_api.h"

/**
 * WiFi country code string
 *  Supported country codes: "00", "CA", "CN", "US" default
 */
#ifndef CONFIG_WIFI_COUNTRY_CODE
#define CONFIG_WIFI_COUNTRY_CODE            "00"
#endif

#ifndef DEFAULT_CHANNEL_TXPOWER
#define DEFAULT_CHANNEL_TXPOWER             20
#endif

typedef struct {
    uint8_t first_chan;
    uint8_t chan_cnt;
    int8_t power;
    uint8_t flags;
} wifi_regrule_t;

typedef struct {
    char cc[3]; // country code string
    uint8_t reg_rules_cnt;
    wifi_regrule_t reg_rules_arr[6];
} wifi_regdom_t;

/** RF: 2.4G */
#define UAP_RF_24_G     0x01
/** RF: 5.0G */
#define UAP_RF_50_G     0x02
#define MAX_CHANNELS    14

typedef struct _aicwf_config_scan_chan
{
    uint16_t action; /* 0: get, 1: set */
    uint16_t band;
    uint16_t total_chan; /* Total no. of channels in below array */
    uint8_t  chan_num[MAX_CHANNELS]; /* Array of channel nos. for the given band */
} aicwf_config_scan_chan;

extern struct me_chan_config_req fhost_chan;

/**
 ****************************************************************************************
 * @brief Initialize wifi configuration structure from fhost configuration
 *
 * To be called before initializing the wifi stack.
 * Can also be used to retrieve firmware feature list at run-time. In this case @p init
 * is false.
 *
 * @param[out] me_config     Configuration structure for the UMAC (i.e. ME task)
 * @param[out] start         Configuration structure for the LMAC (i.e. MM task)
 * @param[out] base_mac_addr Base MAC address of the device (from which all VIF MAC
 *                           addresses are computed)
 * @param[in]  init          Whether it is called before firmware initialization or not.
 ****************************************************************************************
 */
void fhost_config_prepare(struct me_config_req *me_config, struct mm_start_req *start,
                          struct mac_addr *base_mac_addr, bool init);

/**
 ****************************************************************************************
 * @brief Return the channel associated to a given frequency
 *
 * @param[in] freq Channel frequency
 *
 * @return Channel definition whose primary frequency is the requested one and NULL if
 * there no such channel.
 ****************************************************************************************
 */
struct mac_chan_def *fhost_chan_get(int freq);
uint16_t fhost_chan_bw_flags_get_by_freq(uint16_t ch_freq);
void fhost_chan_clean(void);
int fhost_chan_update(const char * country_code);
int aicwf_channel_set(aicwf_config_scan_chan *config_scan_chan);

extern void set_mac_address(uint8_t *addr);
extern uint8_t* get_mac_address(void);

const char * fhost_config_name_get(enum fhost_config_id id);
uint32_t fhost_config_value_get(enum fhost_config_id id);
void fhost_config_value_set(enum fhost_config_id id, uint32_t val);

/**
 * @}
 */
#endif /* _FHOST_CONFIG_H_ */