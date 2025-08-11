#ifndef __WIFI_MAIN_H__
#define __WIFI_MAIN_H__
#include "rtthread.h"

enum wifi_start_t
{
    WIFI_CLOSE,
    WIFI_OPEN,
    WIFI_SCANING,
    WIFI_SCANEND,
    WIFI_CONNECTING,
    WIFI_CONNECTED,
    WIFI_DISCONNECT,
};

enum wifi_cmd_flag
{
    WIFI_CMD_SCAN_STATR,
    WIFI_CMD_SCAN_STOP,
    WIFI_CMD_OPEN,
    WIFI_CMD_CLOSE,
    WIFI_CMD_CONNECT,
    WIFI_CMD_DISCONNECT,
    WIFI_CMD_GET_STATE,
    WIFI_CMD_GET_SCAN_RESUIT,
    WIFI_CMD_SET_MAC,
    WIFI_CMD_SET_IPADDR,
};

struct wifi_device_msg
{
    uint16_t cmd_flag;
    uint16_t wifi_start;
    uint32_t scant_num;
    uint8_t *scan_result;
    uint8_t *ssid;
    uint8_t *passwd;
    uint8_t *mac;
    uint8_t *ip;
    uint8_t *key_mode;
};
struct wifi_scan_t
{
    uint32_t scant_num;
    void *scan_buff;
};

#endif