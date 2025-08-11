/** @file wlan_basic_cli.c
 *
 *  @brief  This file provides Connection Manager CLI
 *
 *  Copyright 2008-2020 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <wlan_wifi.h>

#ifdef __ZEPHYR__
    #include "wifi_shell.h"
#else
    #include <cli.h>
#endif

static void test_wfa_wlan_version(int argc, char **argv)
{
    unsigned char i;
    char version_str[MLAN_MAX_VER_STR_LEN];
    char *ext_ver_str = wlan_get_firmware_version_ext();
    (void)strcpy(version_str, ext_ver_str);
    for (i = 0; i < strlen(version_str); i++)
    {
        if (version_str[i] == '.' && version_str[i + 1U] == 'p')
        {
            version_str[i] = '\0';
            break;
        }
    }

    (void)PRINTF("WLAN Version : %s\r\n", version_str);
}

static void test_wlan_version(int argc, char **argv)
{
    char *version_str;

    version_str = wlan_get_firmware_version_ext();

    (void)PRINTF("WLAN Driver Version   : %s\r\n", WLAN_DRV_VERSION);
    (void)PRINTF("WLAN Firmware Version : %s\r\n", version_str);
}

static void test_wlan_get_mac_address(int argc, char **argv)
{
    uint8_t sta_mac[MLAN_MAC_ADDR_LENGTH], uap_mac[MLAN_MAC_ADDR_LENGTH];

    (void)PRINTF("MAC address\r\n");
    if (wlan_get_mac_address_uap(uap_mac) || wlan_get_mac_address(sta_mac))
    {
        (void)PRINTF("Error: unable to retrieve MAC address\r\n");
    }
    else
    {
        (void)PRINTF("STA MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n", sta_mac[0], sta_mac[1], sta_mac[2],
                     sta_mac[3], sta_mac[4], sta_mac[5]);
        (void)PRINTF("uAP MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\r\n", uap_mac[0], uap_mac[1], uap_mac[2],
                     uap_mac[3], uap_mac[4], uap_mac[5]);
    }
}

static struct cli_command wlan_wfa_basic_commands[] =
{
    {"wlan-version", NULL, test_wfa_wlan_version},
    {"wlan-mac", NULL, test_wlan_get_mac_address},
};

static struct cli_command wlan_basic_commands[] =
{
    {"wlan-version", NULL, test_wlan_version},
    {"wlan-mac", NULL, test_wlan_get_mac_address},
};

static bool wlan_wfa_basic_cli_init_done = false;

int wlan_wfa_basic_cli_init(void)
{
    unsigned int i;

    for (i = 0; i < sizeof(wlan_wfa_basic_commands) / sizeof(struct cli_command); i++)
    {
        if (cli_register_command(&wlan_wfa_basic_commands[i]) != 0)
        {
            return WLAN_ERROR_ACTION;
        }
    }

    wlan_wfa_basic_cli_init_done = true;

    return WLAN_ERROR_NONE;
}

int wlan_wfa_basic_cli_deinit(void)
{
    unsigned int i;

    for (i = 0; i < sizeof(wlan_wfa_basic_commands) / sizeof(struct cli_command); i++)
    {
        if (cli_unregister_command(&wlan_wfa_basic_commands[i]) != 0)
        {
            return WLAN_ERROR_ACTION;
        }
    }

    wlan_wfa_basic_cli_init_done = false;

    return WLAN_ERROR_NONE;
}

int wlan_basic_cli_init(void)
{
    unsigned int i;

    if (wlan_wfa_basic_cli_init_done)
    {
        return WLAN_ERROR_NONE;
    }

    for (i = 0; i < sizeof(wlan_basic_commands) / sizeof(struct cli_command); i++)
    {
        if (cli_register_command(&wlan_basic_commands[i]) != 0)
        {
            return WLAN_ERROR_ACTION;
        }
    }

    return WLAN_ERROR_NONE;
}

int wlan_basic_cli_deinit(void)
{
    unsigned int i;

    for (i = 0; i < sizeof(wlan_basic_commands) / sizeof(struct cli_command); i++)
    {
        if (cli_unregister_command(&wlan_basic_commands[i]) != 0)
        {
            return WLAN_ERROR_ACTION;
        }
    }

    return WLAN_ERROR_NONE;
}
