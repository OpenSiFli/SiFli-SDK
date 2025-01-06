/**
  ******************************************************************************
  * @file   bluetooth_hci_flash.c
  * @author Sifli software development team
  * @brief SIFLI bluetooth hci log written to flash implementation.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2023 - 2023,  Sifli Technology
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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "rtthread.h"
#include "os_adaptor.h"
#include "board.h"
#include "ble_stack.h"
#include "ipc_queue.h"
#include "rthw.h"
#include "log.h"
#include "bluetooth_hci_flash.h"

#ifdef HCI_ON_FLASH

#ifdef RT_USING_DFS
    #include <dfs_posix.h>
#endif

#ifdef BSP_USING_PC_SIMULATOR
    #include "time.h"
    #define time _time64
    #define localtime _localtime64
#endif

#ifdef USING_FILE_LOGGER
    #include "file_logger.h"
#endif

#define BT_HCI_MAX_PATH_LEN 40

#define BT_HCI_TIME_LEN (20)
#define BT_HCI_TIME_OFFSET (0)
#define BT_HCI_WP_OFFSET (BT_HCI_TIME_LEN)
#define BT_HCI_DATA_OFFSET (BT_HCI_TIME_OFFSET + BT_HCI_WP_OFFSET)

static uint8_t g_file_ready = 1;
static uint8_t g_filePath[50];

#if defined(USING_FILE_LOGGER)||defined(RT_USING_DFS)
    static int g_fptr;
    static uint32_t g_file_pos;
    static uint32_t g_hci_wr_p;
    static uint32_t g_file_len;
    static uint32_t g_file_flush_size;
    static uint32_t g_file_wr_size;
    static uint8_t g_file_writing;
    static uint8_t g_file_force_flush;
#endif

#if defined(USING_FILE_LOGGER)

static uint32_t bt_hci_init_int(uint32_t flush_size)
{
    uint32_t ret;
    char logName[BT_HCI_MAX_PATH_LEN] = {0};

    snprintf(logName, BT_HCI_MAX_PATH_LEN, "%shci.bin", g_filePath);

    g_fptr = (int)file_logger_init(logName, flush_size);
    if (!g_fptr)
        ret = 1;
    else
        ret = 0;

    return ret;

}

uint32_t bt_hci_init(uint32_t flush_size, uint32_t is_start)
{
    uint32_t ret = 0xFF;

    do
    {
        memcpy(g_filePath, BT_HCI_PATH, strlen(BT_HCI_PATH));

        if (g_filePath[strlen((const char *)g_filePath) - 1] != '/' && g_filePath[strlen((const char *)g_filePath) - 1] != '\\')
        {
            g_filePath[strlen((const char *)g_filePath)] = '/';
        }

        LOG_I("path %s", g_filePath);
        if (0 != access((const char *)g_filePath, 0) && 0 != mkdir((const char *)g_filePath, 0))
        {
            ret = 2;
            break;
        }

        if (bt_hci_init_int(flush_size) != 0)
        {
            ret = 3;
            break;
        }


        g_file_flush_size = flush_size;
        g_file_ready = 1;
        ret = 0;
    }
    while (0);

    return ret;
}


uint32_t bt_hci_write(uint8_t *buffer, uint32_t len)
{
    uint32_t ret = 1;

    if (g_file_ready && g_fptr != 0 && buffer != NULL)
    {
        fl_err_t log_ret = file_logger_write((void *)g_fptr, buffer, len);
        if (log_ret != FL_OK)
        {
            LOG_E("HCI write failed %d", log_ret);
            ret = 2;
        }
        else
            ret = 0;
    }

    return ret;
}

uint32_t bt_hci_flush(void)
{
    uint32_t ret = 1;

    if (g_file_ready && g_fptr != 0)
    {
        fl_err_t log_ret = file_logger_flush((void *)g_fptr);
        if (log_ret != FL_OK)
        {
            LOG_E("HCI flush failed %d", log_ret);
        }
        else
            ret = 0;
    }

    return ret;
}

uint32_t bt_hci_close(void)
{
    uint32_t ret = 1;

    if (g_file_ready && g_fptr != 0)
    {
        fl_err_t log_ret = file_logger_close((void *)g_fptr);
        if (log_ret != FL_OK)
        {
            LOG_E("HCI close failed %d", log_ret);
        }
        else
        {
            g_file_ready = 0;
            g_fptr = 0;
            ret = 0;
        }
    }

    return ret;
}

uint32_t bt_hci_open(void)
{
    uint32_t ret = 1;
    if (g_file_ready == 0 && g_file_flush_size != 0)
    {
        if (bt_hci_init_int(g_file_flush_size) != 0)
        {
            LOG_E("HCI open failed");
            ret = 2;
        }
        else
        {
            g_file_ready = 1;
            ret = 0;
        }
    }

    return ret;
}


static void bt_hci(uint8_t argc, char **argv)
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "flush") == 0)
        {
            bt_hci_flush();
        }
        else if (strcmp(argv[1], "open") == 0)
        {
            bt_hci_open();
        }
        else if (strcmp(argv[1], "close") == 0)
        {
            bt_hci_close();
        }
    }
}

MSH_CMD_EXPORT(bt_hci, BT HCI command);


#elif defined(RT_USING_DFS)

static uint32_t bt_hci_flush_int(void)
{
    char logName[BT_HCI_MAX_PATH_LEN] = {0};

    uint32_t ret = 0;
    int res;
    if (g_file_ready && g_fptr != -1)
    {

        LOG_I("flush int");
        //TODO: Protect
        snprintf(logName, BT_HCI_MAX_PATH_LEN, "%shci.bin", g_filePath);
        LOG_I("close name %s", logName);

        res =  lseek(g_fptr, BT_HCI_WP_OFFSET, SEEK_SET);
        LOG_I("seek1 res %d", res);
        write(g_fptr, &g_file_pos, sizeof(g_file_pos));
        res = close(g_fptr);
        LOG_I("close %d", res);
        g_fptr = open(logName, O_RDWR | O_BINARY, 0);
        if (g_fptr >= 0)
        {
            lseek(g_fptr, g_file_pos, SEEK_SET);
        }
        else
        {
            ret = 2;
            LOG_E("Reopen file failed!!!");
        }
    }
    else
        ret = 1;

    return ret;

}

uint32_t bt_hci_init(uint32_t flush_size, uint32_t is_start)
{
    uint32_t ret = 0xFF;
    g_fptr = -1;
    g_file_wr_size = 0;
    uint32_t fal_size;
    int res;

    do
    {
#if 0
//#ifndef BSP_USING_PC_SIMULATOR
        const struct fal_partition *fal;
        fal = fal_partition_find(BT_HCI_PARTION);
        if (fal)
        {

            fal_size = fal->len;
            LOG_I("fal size %d", fal->len);
            if (fal_size == 0)
            {
                ret = 4;
                break;
            }

            const struct fal_flash_dev *flash_dev = fal_flash_device_find(fal->flash_name);
            RT_ASSERT(flash_dev);
        }
        else
#endif
            fal_size = BT_HCI_DFT_SIZE;


        memcpy(g_filePath, BT_HCI_PATH, strlen(BT_HCI_PATH));

        if (g_filePath[strlen((const char *)g_filePath) - 1] != '/' && g_filePath[strlen((const char *)g_filePath) - 1] != '\\')
        {
            g_filePath[strlen((const char *)g_filePath)] = '/';
        }

        LOG_I("path %s", g_filePath);
        if (0 != access((const char *)g_filePath, 0) && 0 != mkdir((const char *)g_filePath, 0))
        {
            ret = 2;
            break;
        }

        char logName[50] = {0};

        sprintf(logName, "%shci.bin", g_filePath);

        g_fptr = open(logName, O_RDWR | O_BINARY, 0);
        if (g_fptr < 0)
            g_fptr = open(logName, O_RDWR | O_BINARY | O_CREAT | O_TRUNC, 0);

        LOG_I("g_fptr %x", g_fptr);

        if (g_fptr < 0)
        {
            ret = 3;
            break;
        }

        if (fal_size  >= 32 * 1024)
            g_file_len = fal_size  - 10 * 1024;
        else
            g_file_len = fal_size / 2;

        g_file_ready = is_start;
        g_file_flush_size = flush_size;

        int fileLen = 0;
        fileLen = lseek(g_fptr, 0, SEEK_END);
        LOG_I("len %d", fileLen);
        if (fileLen < 20)
        {
            g_file_pos = 0;
            char timeStr[BT_HCI_TIME_LEN] = {0};
            time_t now;
            struct tm *p_tm;

            now = time(RT_NULL);
            //rt_enter_critical();
            p_tm = localtime(&now);
            //rt_exit_critical();
            sprintf(timeStr, "%04d_%02d_%02d %02d:%02d:%02d",
                    p_tm->tm_year + 1900, p_tm->tm_mon + 1, p_tm->tm_mday, p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);

            res = write(g_fptr, timeStr, sizeof(timeStr));
            LOG_I
            ("res w %d", res);
            g_file_pos += sizeof(timeStr);

            g_file_pos += sizeof(g_file_pos);
            res = write(g_fptr, &g_file_pos, sizeof(g_file_pos));
            LOG_I
            ("res w2 %d", res);
            close(g_fptr);
            g_fptr = open(logName, O_RDWR | O_BINARY, 0);
            fileLen = lseek(g_fptr, 0, SEEK_END);
            LOG_I("len1 %d", fileLen);
        }
        else
        {
            lseek(g_fptr, BT_HCI_WP_OFFSET, SEEK_SET);
            read(g_fptr, &g_file_pos, sizeof(g_file_pos));
            LOG_I("wr_p %d", g_file_pos);
            lseek(g_fptr, g_file_pos, SEEK_SET);
        }

        ret = 0;

    }
    while (0);

    LOG_D("init ret %d", ret);
    return ret;
}


uint32_t bt_hci_write(uint8_t *buffer, uint32_t len)
{
    rt_base_t level;
    uint8_t is_flushed = 0;
    uint32_t ret = 1;
    int res;
    if (g_file_ready && g_fptr >= 0 && buffer != NULL)
    {
        LOG_I("write len %d", len);
        level = rt_hw_interrupt_disable();
        g_file_writing = 1;
        rt_hw_interrupt_enable(level);

        if (g_file_pos + len == g_file_len)
        {
            write(g_fptr, (const uint8_t *)buffer, len);
            g_file_pos = BT_HCI_DATA_OFFSET;
            lseek(g_fptr, g_file_pos, SEEK_SET);
        }
        else if (g_file_pos + len > g_file_len)
        {
            uint32_t len1 = g_file_len - g_file_pos;
            write(g_fptr, (const uint8_t *)buffer, len1);
            lseek(g_fptr, BT_HCI_DATA_OFFSET, SEEK_SET);
            write(g_fptr, (const uint8_t *)(buffer + len1), len - len1);
            g_file_pos = BT_HCI_DATA_OFFSET + len - len1;
        }
        else
        {

            res  = write(g_fptr, (const uint8_t *)buffer, len);
            LOG_I
            ("res w3 %d", res);
            g_file_pos += len;
        }
        g_file_wr_size += len;
        if ((g_file_flush_size != BT_HCI_INVALID_FLUSH_SIZE) && (g_file_wr_size >= g_file_flush_size))
        {
            bt_hci_flush_int();
            is_flushed = 1;
            g_file_wr_size = 0;
        }

        level = rt_hw_interrupt_disable();
        g_file_writing = 0;
        rt_hw_interrupt_enable(level);

        if (g_file_force_flush && !is_flushed)
        {
            bt_hci_flush_int();
            level = rt_hw_interrupt_disable();
            g_file_force_flush = 0;
            rt_hw_interrupt_enable(level);
        }

        ret = 0;
    }

    return ret;
}



uint32_t bt_hci_flush(void)
{
    char logName[BT_HCI_MAX_PATH_LEN] = {0};
    rt_base_t level;

    uint32_t ret = 0;
    if (g_file_writing)
    {

        LOG_I("flush len");
        level = rt_hw_interrupt_disable();
        g_file_force_flush = 1;
        rt_hw_interrupt_enable(level);
    }
    else
        ret = bt_hci_flush_int();

    return ret;

}

uint32_t bt_hci_close(void)
{
    uint32_t ret = 0;

    if (g_file_ready)
    {
        g_file_ready = 0;
        if (g_fptr >= 0)
            bt_hci_flush_int();
    }

    return ret;
}

uint32_t bt_hci_open(void)
{
    uint32_t ret = 0;

    if (!g_file_ready)
    {
        g_file_ready = 1;
    }

    return ret;
}



static void bt_hci(uint8_t argc, char **argv)
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "flush") == 0)
        {
            bt_hci_flush();
        }
        else if (strcmp(argv[1], "open") == 0)
        {
            bt_hci_open();
        }
        else if (strcmp(argv[1], "close") == 0)
        {
            bt_hci_close();
        }
    }
}



MSH_CMD_EXPORT(bt_hci, BT HCI command);

#else

uint32_t bt_hci_close(void)
{
    return 0xFF;
}

uint32_t bt_hci_open(void)
{
    return 0xFF;
}

uint32_t bt_hci_flush(void)
{
    return 0xFF;
}

#endif // RT_USING_DFS


int bt_hci_log_path_get(char *path)
{
    char logName[BT_HCI_MAX_PATH_LEN] = {0};

    snprintf(logName, BT_HCI_MAX_PATH_LEN, "%shci.bin", g_filePath);

    strncpy(path, logName, BT_HCI_MAX_PATH_LEN);
    return 1;
}


int bt_hci_log_onoff(int flag)
{
    //0 off 1 on 2 query
    int ret = 0;
    switch (flag)
    {
    case 0:
    {
        bt_hci_flush();
        bt_hci_close();
        ret = 0;
    }
    break;
    case 1:
    {
        bt_hci_open();
        ret = 1;
    }
    case 2:
    {
        ret = g_file_ready ? 1 : 0;
    }
    break;
    default:
        break;
    }

    return ret;
}

int bt_hci_log_type_get(void)
{
    //Use FS
    return 1;
}

int bt_hci_log_clear(void)
{
#if defined(RT_USING_DFS)
    char logName[BT_HCI_MAX_PATH_LEN] = {0};

    snprintf(logName, BT_HCI_MAX_PATH_LEN, "%shci.bin", g_filePath);

    unlink(logName);
#endif
    return 0;
}

#endif // HCI_ON_FLASH

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
