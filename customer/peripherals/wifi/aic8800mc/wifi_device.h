/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _WIFI_SERVICE_H_
#define _WIFI_SERVICE_H_

#include <rtthread.h>

#define WIFI_THREAD_STACK_SIZE              1024
#define WIFI_THREAD_PREORITY                0X18

#define WIFI_CONTROL_CONNECT                0X10000001
#define WIFI_CONTROL_DISCONNECT             0X10000002
#define WIFI_CONTROL_GET_VER                0X10000003
#define WIFI_CONTROL_GET_STATUS             0X10000004
#define WIFI_CONTROL_GET_MAC                0X10000005
#define WIFI_CONTROL_CLEAR_CFG              0X10000006


enum wifi_status_t
{
    WIFI_DISCONNECT,        // AIC-STA hasn't connected AP
    WIFI_CONNECTED,         // AIC-STA has connected AP
    WIFI_SCANNING,
    WIFI_CONNECTING,
};

typedef struct
{
    char *ssid;
    char *pwd;
} wifi_connect_t;

#endif //SERVICE_H_

