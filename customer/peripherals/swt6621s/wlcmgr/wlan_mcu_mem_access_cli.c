/** @file wlan_mcu_access_cli.c
 *
 *  @brief  This file provides functions to read and write memory during cli initialization.
 *
 *  Copyright 2008-2020 NXP
 *
 *  Licensed under the LA_OPT_NXP_Software_License.txt (the "Agreement")
 *
 */
#ifdef CONFIG_MCU_MEM_ACCESS
#include <cli.h>
#include <cli_utils.h>
#include <wm_os.h>

static void test_mcu_mem_read(int argc, char **argv)
{
    int *addr;
    int value = 0;
    int length = 0;
    int i = 0;

    if (argc < 2)
    {
        (void)PRINTF("Input error!\r\n");
        (void)PRINTF("Usage:\r\n");
        (void)PRINTF("  mcu_mem_wirte <memory addr> [length]\r\n");
        (void)PRINTF("  <length> is dword length. Default is 1 and max is 16\r\n");
        return;
    }
    addr = (int *)a2hex_or_atoi(argv[1]);
    if (argc == 3)
        length = a2hex_or_atoi(argv[2]);
    else
        length = 1;
    (void)PRINTF("Read memory 0x%x ", addr);
    for (i = 0; i < length; i++)
    {
        value = *(addr + i);
        (void)PRINTF("0x%x ", value);
    }
    (void)PRINTF("\r\n");
}

static void test_mcu_mem_write(int argc, char **argv)
{
    int *addr;
    int value = 0;

    if (argc != 3)
    {
        (void)PRINTF("Invalid number of arguments!\r\n");
        (void)PRINTF("Usage:\r\n");
        (void)PRINTF("  mcu_mem_write <memory addr> <value>\r\n");
        return;
    }
    addr = (int *)a2hex_or_atoi(argv[1]);
    value = a2hex_or_atoi(argv[2]);
    *addr = value;
    value = *addr;
    (void)PRINTF("Write memory 0x%x to 0x%x\r\n", addr, value);
}

static struct cli_command mcu_mem_access_commands[] =
{
    {"mcu_mem_read", "<addr> [length]", test_mcu_mem_read},
    {"mcu_mem_write", "<addr> <value>", test_mcu_mem_write},
};

int mcu_mem_access_init()
{
    int i;
    for (i = 0; i < sizeof(mcu_mem_access_commands) / sizeof(struct cli_command); i++)
        if (cli_register_command(&mcu_mem_access_commands[i]) != 0)
            return -WM_FAIL;

    return WM_SUCCESS;
}
#endif