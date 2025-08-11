/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/* @file
 * @brief Wi-Fi shell sample
 */

#include "includes.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <string.h>
#include <cli.h>
#include <cli_utils.h>

//int cli_main(int argc, char **argv);
#if CONFIG_HOSTAPD
//int hostapd_cli_main(int argc, char **argv);
int hlr_main(int argc, char *argv[]);
#endif

#if 0
void test_wpa_cli(int argc, char **argv)
{
    argc--;
    argv = &argv[1];
    (void)cli_main(argc, argv);
}
#endif

#if CONFIG_HOSTAPD
#if 0
void test_hostapd_cli(int argc, char **argv)
{
    argc--;
    argv = &argv[1];
//    (void)hostapd_cli_main(argc, argv);
}
#endif

void test_hlr_cli(int argc, char **argv)
{
    argc--;
    argv = &argv[1];
    (void)hlr_main(argc, argv);
}
#endif

static struct cli_command wpa_cli[] = {
//    {"wpa_cli", "<standard wpa cli options>", test_wpa_cli},
#if CONFIG_HOSTAPD
//    {"hostapd_cli", "<standard hostapd cli options>", test_hostapd_cli},
    {"wlan-hlr-cli", "<standard hlr cli options>", test_hlr_cli},
#endif
};

#if CONFIG_HOSTAPD
extern int hlr_cli_init(void);
extern int hlr_cli_deinit(void);
#endif

int wpa_cli_init(void)
{
    unsigned int i;
    for (i = 0; i < sizeof(wpa_cli) / sizeof(struct cli_command); i++)
    {
        if (cli_register_command(&wpa_cli[i]) != 0)
        {
            return -WM_FAIL;
        }
    }

#if CONFIG_HOSTAPD
    return hlr_cli_init();
#else
    return WM_SUCCESS;
#endif
}

int wpa_cli_deinit(void)
{
    unsigned int i;

    for (i = 0; i < sizeof(wpa_cli) / sizeof(struct cli_command); i++)
    {
        if (cli_unregister_command(&wpa_cli[i]) != 0)
        {
            return -WM_FAIL;
        }
    }

#if CONFIG_HOSTAPD
    return hlr_cli_deinit();
#else
    return WM_SUCCESS;
#endif
}
