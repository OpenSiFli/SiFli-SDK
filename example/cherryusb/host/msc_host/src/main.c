/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "usbh_core.h"
#include <stdio.h>
#include <string.h>

/**
 * @brief  Main program
 * @param  None
 * @retval 0 if success, otherwise failure number
 */
int main(void)
{
    rt_kprintf("cherryusb host demo!\n");

    usbh_initialize(0, (uintptr_t)USBC_BASE, RT_NULL);

    // Scan for USB mass storage devices (/dev/sda ~ /dev/sdz)
    rt_device_t dev = RT_NULL;
    while(dev == RT_NULL)
    {
        rt_kprintf("Scanning for USB mass storage devices...\n");
        for(char c = 'a'; c <= 'z'; c++)
        {
            char dev_name[16];
            rt_snprintf(dev_name, sizeof(dev_name), "/dev/sd%c", c);
            dev = rt_device_find(dev_name);
            if(dev != RT_NULL)
            {
                rt_kprintf("USB mass storage device found: %s\n", dev_name);
                break;
            }
        }
        if(dev == RT_NULL)
        {
            rt_thread_mdelay(1000);
        }
    }

    FILE* f = fopen("README.TXT", "r");
    if( f == NULL )
    {
        rt_kprintf("Failed to open file!\n");
        return -1;
    }
    char buffer[128];
    size_t n = fread(buffer, 1, sizeof(buffer)-1, f);
    buffer[n] = '\0';
    rt_kprintf("File content:\n%s\n", buffer);
    fclose(f);

    f = fopen("WRITE.TXT", "w");
    if( f == NULL )
    {
        rt_kprintf("Failed to open file for writing!\n");
        return -1;
    }
    const char* text = "Hello, CherryUSB!\nThis is a test file written by USB host demo.\n";
    fwrite(text, 1, strlen(text), f);
    fclose(f);

    while (1)
    {
        rt_thread_mdelay(1000);
    }
    return 0;
}
