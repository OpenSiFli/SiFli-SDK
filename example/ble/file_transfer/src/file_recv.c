/**
  ******************************************************************************
  * @file   file_recv.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2025,  Sifli Technology
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


#include <rtthread.h>
#include "bf0_sibles_watchface.h"
#include "dfs_posix.h"

#define LOG_TAG "FILE_RECV"
#include "log.h"

#define FILE_RECV_DIR           "/file_recv/"
#define FILE_MAX_LEN            256
#define BFREE_RESERVED          10
#define CRC_INIT_VAL            0xffffffff


typedef struct
{
    uint8_t  space_check;
    uint16_t type;
    uint16_t name_len;
    uint32_t total_file_blocks;
    uint32_t accumulated_len;
    uint32_t all_files_len;
    uint32_t crc_result;
    uint32_t f_bsize;
    uint32_t f_bfree;
    int      fd;
    uint32_t current_size;
    uint32_t total_size;
    char     path[FILE_MAX_LEN];
} file_recv_env_t;

static file_recv_env_t g_file_recv_env;

//Name         Polynomial Representations
//Normal      Reversed    Reciprocal  Reversedreciprocal
//CRC - 3 - GSM    0x3         0x6         0x5         0x5
//CRC - 8        0xD5        0xAB        0x57        0xEA
//CRC - 16 - CCITT 0x1021      0x8408      0x811       0x8810
//CRC - 32       0x04C11DB7  0xEDB88320  0xDB710641  0x82608EDB
uint32_t polynomial_crc_calculate(uint8_t *pSrc, uint32_t len, uint32_t pre_crc)
{
    uint32_t crc = pre_crc;
    for (uint32_t m = 0; m < len; m++)
    {
        crc ^= ((uint32_t)pSrc[m]) << 24;

        for (uint32_t n = 0; n < 8; n++)
        {
            if (crc & 0x80000000)
            {
                crc = (crc << 1) ^ 0x04c11db7;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void file_recv_init_env(void)
{
    rt_memset(&g_file_recv_env, 0, sizeof(g_file_recv_env));
    g_file_recv_env.fd = -1;
}

static file_recv_env_t *file_recv_get_env(void)
{
    return &g_file_recv_env;
}


//create directory level by level
static int file_recv_mkdir(const char *dir)
{
    int ret = 0;
    char *temp_path = rt_calloc(1, strlen(dir) + 1);
    if (!temp_path)
    {
        return -1;
    }
    strcpy(temp_path, dir);
    char *p, *temp = temp_path;
    temp++;
    while (temp[0] && (p = strchr(temp, '/')))
    {
        char c = p[0];
        char *pos = p;
        p[0] = 0;
        if (0 != access(temp_path, 0) && 0 != mkdir(temp_path, 0))
        {
            LOG_E("mkdir %s failed ", dir, temp_path);
            rt_free(temp_path);
            return -2;
        }
        temp = p + 1;
        pos[0] = c;
    }
    rt_free(temp_path);
    return 0;
}


static int file_recv_open(char *path)
{
    file_recv_env_t *env = file_recv_get_env();
    int fd = open(path, O_CREAT | O_RDWR | O_BINARY | O_TRUNC, 0);
    if (fd < 0)
    {
        LOG_E("open failed: %s", env->path);
        return BLE_WATCHFACE_STATUS_FILE_OPEN_ERROR;
    }
    env->fd = fd;
    return BLE_WATCHFACE_STATUS_OK;
}

//write file and remvove the last 4 crc bytes
static int file_recv_write(uint8_t *data, uint32_t data_len)
{
    file_recv_env_t *env = file_recv_get_env();
    RT_ASSERT(data_len >= 4);
    uint32_t len = data_len;
    if ((env->current_size + data_len) > env->total_size)
    {
        LOG_E("file size err:cur_size %d + len %d > total len %d!", env->current_size, data_len, env->total_size);
        return BLE_WATCHFACE_STATUS_FILE_SIZE_ERROR;
    }
    else if (env->current_size + data_len == env->total_size)
    {
        len = data_len - 4;
        if (0 == len) return BLE_WATCHFACE_STATUS_OK;
    }
    int size = write(env->fd, data, len);
    if (size != len)
    {
        LOG_E("write failed: write %d, ret %d", data_len, size);
        return BLE_WATCHFACE_STATUS_FILE_WRITE_ERROR;
    }
    env->current_size += data_len;
    env->accumulated_len += data_len;
    return BLE_WATCHFACE_STATUS_OK;
}

static int file_recv_close(void)
{
    file_recv_env_t *env = file_recv_get_env();

    if (0 <= env->fd)
    {
        int result = close(env->fd);
        if (0 != result)
        {
            LOG_E("close fd %d failed\n", env->fd, result);
            return BLE_WATCHFACE_STATUS_FILE_CLOSE_ERROR;
        }
        env->fd = -1;
    }
    return BLE_WATCHFACE_STATUS_OK;
}

//Compare the original crc with the calculated crc
static int file_recv_check_crc32(uint8_t *data, uint32_t len, uint32_t cal_crc)
{
    uint32_t orig_crc = 0;
    //Get original crc value,it is the last 4 byets for one file
    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t temp = *(data + len - i - 1);
        orig_crc += (temp << (24 - i * 8));
    }
    if (orig_crc != cal_crc)
    {
        LOG_E("crc failed!, orgin = %#x, cal_crc = %#x", orig_crc, cal_crc);
        return BLE_WATCHFACE_STATUS_CRC_CALCULATE_ERROR;
    }
    return BLE_WATCHFACE_STATUS_OK;
}

//Calculated crc for data received
static uint8_t file_recv_accumulate_crc(ble_watchface_file_download_ind_t *files)
{
    uint8_t ret = BLE_WATCHFACE_STATUS_OK;
    file_recv_env_t *env = file_recv_get_env();

    LOG_I("cur %d next %d total %d\n", files->data_len, env->current_size + files->data_len, env->total_size);
    if (env->current_size + files->data_len == env->total_size)
    {
        //APP will align the file to 4 bytes, and then segment it. The length of the segment is a multiple of 4.
        //Therefor, CRC will not cross the segments.
        if (files->data_len > 4)
        {
            env->crc_result = polynomial_crc_calculate(files->data, (files->data_len - 4), env->crc_result);
        }
        ret = file_recv_check_crc32(files->data, files->data_len, env->crc_result);
    }
    else if (env->current_size + files->data_len < env->total_size)
    {
        env->crc_result = polynomial_crc_calculate(files->data, files->data_len, env->crc_result);
    }
    else
    {
        ret = BLE_WATCHFACE_STATUS_FILE_SIZE_ERROR;
        LOG_E("file size err!");
    }
    return ret;
}


watchface_event_ack_t file_recv_event_handler(uint16_t event, uint16_t length, void *param)
{
    file_recv_env_t *env = file_recv_get_env();
    int rsp = WATCHFACE_EVENT_SUCCESSED;
    switch (event)
    {
    case WATCHFACE_APP_START:
    {
        struct statfs buffer;
        rsp = file_recv_mkdir(FILE_RECV_DIR);
        if (rsp != 0)
        {
            rsp = BLE_WATCHFACE_STATUS_MKDIR_ERROR;
            ble_watchface_send_start_rsp_file_info(rsp, buffer.f_bsize, buffer.f_bfree);
            goto __FAILED;
        }
        //Retrieve directory partion information
        rsp = dfs_statfs(FILE_RECV_DIR, &buffer);
        if (rsp != 0)
        {
            LOG_E("dfs_statfs failed!");
            rsp = BLE_WATCHFACE_STATUS_SPACE_ERROR;
            ble_watchface_send_start_rsp_file_info(rsp, buffer.f_bsize, buffer.f_bfree);
            goto __FAILED;
        }
        file_recv_init_env();
        ble_watchface_start_ind_t *info = (ble_watchface_start_ind_t *)param;
        env->all_files_len = info->all_files_len;
        env->type = info->type;
        env->f_bfree = buffer.f_bfree;
        env->f_bsize = buffer.f_bsize;
        //Send partion information to remote phone
        ble_watchface_send_start_rsp_file_info(WATCHFACE_EVENT_SUCCESSED, buffer.f_bsize, buffer.f_bfree);
        break;
    }
    case WATCHFACE_APP_FILE_START:
    {
        ble_watchface_file_start_ind_t *files = (ble_watchface_file_start_ind_t *)param;
        //file length must be 4 bytes aligned
        if (files->file_len % 4)
        {
            LOG_E("file size %d not 4 bytes align!", files->file_len);
            ble_watchface_file_start_rsp(BLE_WATCHFACE_STATUS_FILE_SIZE_ALIGNED_MISSING);
            goto __FAILED;
        }
        if (files->file_name_len + 1 + strlen(FILE_RECV_DIR) > FILE_MAX_LEN)
        {
            LOG_E("file name len  %d is too long!!", files->file_name_len);
            ble_watchface_file_start_rsp(BLE_WATCHFACE_STATUS_APP_ERROR);
            goto __FAILED;
        }
        env->current_size = 0;
        env->total_size = files->file_len;
        env->name_len = files->file_name_len;
        env->crc_result = CRC_INIT_VAL;

        char *temp = malloc(env->name_len + 1);
        RT_ASSERT(temp);
        rt_memcpy(temp, files->file_name, env->name_len);
        temp[env->name_len] = 0;

        rt_memset(env->path, 0, sizeof(env->path));
        strcpy(env->path, FILE_RECV_DIR);

        //Place file in the designated directory
        char *p = strrchr(temp, '/');
        if (p)
        {
            p += 1;
            strcat(env->path, p);
        }
        else
        {
            rt_memcpy(&env->path[strlen(FILE_RECV_DIR)], files->file_name, env->name_len);
        }
        free(temp);
        LOG_I("file: %s", env->path);
        if (!env->space_check)
        {
            env->space_check = 1;
            //Check fs free space
            if (BFREE_RESERVED + env->total_file_blocks > env->f_bfree)
            {
                LOG_I("free size not enough, reserved %d + need %d > free %d!", BFREE_RESERVED, env->total_file_blocks, env->f_bfree);
                ble_watchface_file_start_rsp(BLE_WATCHFACE_STATUS_SPACE_ERROR);
                goto __FAILED;
            }
        }
        if (0 != file_recv_open(env->path))
        {
            ble_watchface_file_start_rsp(BLE_WATCHFACE_STATUS_FILE_OPEN_ERROR);
            goto __FAILED;
        }
        ble_watchface_file_start_rsp(WATCHFACE_EVENT_SUCCESSED);
        break;
    }
    case WATCHFACE_APP_FILE_DOWNLOAD:
    {
        ble_watchface_file_download_ind_t *files = (ble_watchface_file_download_ind_t *)param;
        //Calculate crc and try to compare with orininal crc if last packet
        if (BLE_WATCHFACE_STATUS_OK != file_recv_accumulate_crc(files))
        {
            ble_watchface_file_download_rsp(BLE_WATCHFACE_STATUS_CRC_CALCULATE_ERROR);
            goto __FAILED;
        }
        rsp = file_recv_write(files->data, files->data_len);
        if (rsp != 0)
        {
            ble_watchface_file_download_rsp(rsp);
            goto __FAILED;
        }
        ble_watchface_file_download_rsp(WATCHFACE_EVENT_SUCCESSED);
        break;
    }
    case WATCHFACE_APP_FILE_END:
    {
        rsp = file_recv_close();
        if (BLE_WATCHFACE_STATUS_OK != rsp)
        {
            ble_watchface_file_end_rsp(rsp);
            goto __FAILED;
        }
        ble_watchface_file_end_ind_t *info = (ble_watchface_file_end_ind_t *)param;
        if (info->end_status != BLE_WATCHFACE_STATUS_OK)
        {
            rsp = info->end_status;
            LOG_E("file end error %d", __func__, info->end_status);
            ble_watchface_file_end_rsp(info->end_status);
            goto __FAILED;
        }
        ble_watchface_file_end_rsp(WATCHFACE_EVENT_SUCCESSED);
        break;
    }
    case WATCHFACE_APP_END:
    {
        //Compare all files length with received length
        if (env->accumulated_len != env->all_files_len)
        {
            ble_watchface_end_rsp(BLE_WATCHFACE_STATUS_FILE_SIZE_ERROR);
            goto __FAILED;
        }
        LOG_I("file recv success!");
        ble_watchface_end_rsp(BLE_WATCHFACE_STATUS_OK);
        break;
    }
    case WATCHFACE_APP_FILE_INFO:
    {
        ble_watchface_file_info_ind_t *info = (ble_watchface_file_info_ind_t *)param;
        //Record total fils blocks need to transfer
        env->total_file_blocks = info->file_blocks;
        LOG_I("need blocks %d", info->file_blocks);
        ble_watchface_file_info_rsp(WATCHFACE_EVENT_SUCCESSED);
        break;
    }
    case WATCHFACE_APP_ERROR:
    {
        ble_watchface_error_ind_t *info = (ble_watchface_error_ind_t *)param;
        LOG_E("ble err %d", info->error_type);
        goto __FAILED;
    }
    default:
        break;
    }
    return WATCHFACE_EVENT_SUCCESSED;

__FAILED:
    file_recv_close();
    ble_watchface_abort();
    file_recv_init_env();
    return WATCHFACE_EVENT_FAILED;
}


int file_recv_init(void)
{
    //Register transfer msg callback
    watchface_register(file_recv_event_handler);
    return 0;
}

INIT_APP_EXPORT(file_recv_init);

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

