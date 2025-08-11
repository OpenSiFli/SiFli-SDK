/** @file supp_main.h
 *
 *  @brief  This file provides wpa supplicant init APIs.
 *
 *  Copyright 2023 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __SUPP_MAIN_H_
#define __SUPP_MAIN_H_

#include "drivers/driver.h"

typedef enum __wpa_supp_event
{
    DUMMY = 0U,
    EVENT = 1U,
} wpa_supp_event_t;

struct wpa_supplicant_event_msg
{
    int hostapd;
#ifdef __ZEPHYR__
    bool ignore_msg;
#endif
    void *ctx;
    enum wpa_event_type event;
    void *data;
};

int send_wpa_supplicant_dummy_event();
int send_wpa_supplicant_event(struct wpa_supplicant_event_msg *msg);
void process_wpa_supplicant_event();
void wpa_supplicant_event_wrapper_deep_copy_free(struct wpa_supplicant_event_msg *msg);
int wpa_supplicant_event_wrapper_deep_copy(struct wpa_supplicant_event_msg *msg,
                                           enum wpa_event_type event,
                                           union wpa_event_data *data);

int start_wpa_supplicant(char *iface_name);
int stop_wpa_supplicant(void);

struct hostapd_iface *hostapd_get_interface(const char *ifname);
#endif /* __SUPP_MAIN_H_ */
