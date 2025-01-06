/**
  ******************************************************************************
  * @file   tc_drv_rtc.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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
#include <rtdevice.h>
#include <rtdbg.h>

//#include "utest.h"
#include "string.h"
#include "time.h"
#include "data_service_subscriber.h"
#ifdef WIN32
    #include "bf0_hal_hlp.h"
#else
    #include "bf0_hal.h"
#endif

#define MAX_LEN     80
struct data_clnt_arg
{
    char svc_name[20];
    uint32_t context;
    uint8_t handle;
    uint8_t config_len;
    uint8_t data_len;
    uint8_t config[MAX_LEN >> 1];
    uint8_t data[MAX_LEN];
};

#define DATASVC_CLNT_CONTEXT 0x12345678
#define DATASVC_MAX_CLNT        16
static struct data_clnt_arg g_data_clnt[DATASVC_MAX_CLNT];
static int g_datac_index;

static char temp[80];
static void htoa(unsigned char *p, unsigned char d)
{
    (*p) = d & 0xf0;
    (*p) >>= 4;
    if ((*p) < 10)
        *p += '0';
    else
        *p += ('A' - 10);
    p++;
    (*p) = d & 0xf;
    if ((*p) < 10)
        *p += '0';
    else
        *p += ('A' - 10);
}

static void log_data(uint8_t *data, rt_size_t size)
{
    unsigned int i;
    char *p = temp;

    memset(temp, ' ', sizeof(temp));
    for (i = 0; i < size; i++)
    {
        if (i && (i & 0xf) == 0)
        {
            LOG_I("%s\n", temp);
            memset(temp, ' ', sizeof(temp) - 1);
            p = temp;
        }
        htoa((unsigned char *)p, data[i]);
        p += 2;
        *p = ' ';
        p++;
    }
    LOG_I("%s\n", temp);
}


/* Only available during testcase */
static void test_datasvc_clnt_set_parameter(int argc, char **argv)
{
    char client_tag[] = "client";
    /* Set/Year/Month/Day/Hour/Minute/Seconds*/
    if (argc != 4 && strncmp(client_tag, argv[0], sizeof(client_tag)))
    {
        /* parameter error, no need to handle */
        LOG_E("parameter error %d, %s", argc, argv[0]);
        return;
    };
    uint8_t index = atoi(argv[1]);
    struct data_clnt_arg *p_arg = &g_data_clnt[index];
    strcpy(p_arg->svc_name, argv[2]);
    p_arg->config_len = strlen(argv[3]) >> 1;
    hex2data(argv[4], p_arg->config, MAX_LEN >> 1);
    p_arg->data_len = strlen(argv[3]) >> 1;
    hex2data(argv[5], p_arg->data, MAX_LEN);
}

#define TEST_RTC_DELAY_TIME 5

const char *data_msg_name[] =
{
    "NONE",
    "SUBSCRIBE",
    "CONFIG",
    "TX",
    "RX",
    "PROXY_IND",
    "START",
    "STOP",
    "DATA_READY",
    "DATA_NOTIF",
};

char msg_name[20];
char *data_msg_str(uint16_t msgid)
{
    uint16_t id;

    id = msgid & ~RSP_MSG_TYPE;
    if (id > MSG_SERVICE_SYS_ID_END)
        strcpy(msg_name, "CUSTOM_ID");
    else
    {
        strcpy(msg_name, data_msg_name[id]);
        if (msgid & RSP_MSG_TYPE)
            strcat(msg_name, "_RSP");
    }
    return msg_name;
}

int datasvc_clnt_callback(data_callback_arg_t *arg)
{
    LOG_I("Callback msg_id=0x%x, context=%x\n", arg->msg_id, arg->user_data);
    if (arg->msg_id & RSP_MSG_TYPE)
    {
        data_rsp_t *rsp;
        rsp = (data_rsp_t *)arg->data;
        if (rsp->result < 0)
        {
            LOG_E("Command failed %d\n", rsp->result);
        }
        else
        {
            LOG_I("Command success %d\n", rsp->result);
        }

        if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
        {
            data_subscribe_rsp_t *sub_rsp;
            sub_rsp = (data_subscribe_rsp_t *)rsp;
            RT_ASSERT(arg->user_data < DATASVC_MAX_CLNT);
            if (sub_rsp->result < 0)
            {
                rt_kprintf("Subscribe %s failed\n", g_data_clnt[arg->user_data].svc_name);
            }
            else
            {
                rt_kprintf("Subscribe %s succ:%d\n", g_data_clnt[arg->user_data].svc_name, sub_rsp->handle);
                RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != sub_rsp->handle);
            }
        }
    }
    else if (arg->msg_id == MSG_SERVICE_DATA_NTF_IND)
    {
        LOG_I("Data indication %d bytes:\n", arg->data_len);
        log_data(arg->data, arg->data_len);
    }
    else
    {
        LOG_I("Got message %s, %d bytes:\n", data_msg_str(arg->msg_id), arg->data_len);
        log_data(arg->data, arg->data_len);
    }
    return RT_EOK;
}
#if 0
static void test_datasvc_clnt(void)
{
    uint8_t cid;

    cid = datac_subscribe(g_data_clnt[g_datac_index].svc_name, datasvc_clnt_callback, DATASVC_CLNT_CONTEXT);
    uassert_false(cid == DATA_CLIENT_INVALID_HANDLE);
    if (cid != DATA_CLIENT_INVALID_HANDLE)
    {
        rt_err_t ret = RT_EOK;
        g_data_clnt[g_datac_index].handle = cid;
        uassert_int_equal(ret, RT_EOK);
        rt_thread_delay(100);
        ret = datac_config(g_data_clnt[g_datac_index].handle, g_data_clnt[g_datac_index].config_len, g_data_clnt[g_datac_index].config);
        uassert_int_equal(ret, RT_EOK);
        rt_thread_delay(100);
        ret = datac_tx(g_data_clnt[g_datac_index].handle, g_data_clnt[g_datac_index].data_len, g_data_clnt[g_datac_index].data);
        uassert_int_equal(ret, RT_EOK);
        rt_thread_delay(100);
        ret = datac_rx(g_data_clnt[g_datac_index].handle, 32, NULL);
        uassert_int_equal(ret, RT_EOK);
        rt_thread_delay(100);
        ret = datac_unsubscribe(g_data_clnt[g_datac_index].handle);
        uassert_int_equal(ret, RT_EOK);
    }
}

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    /* timer handle should be clear when delete timer. */
    memset(g_data_clnt, 0, DATASVC_MAX_CLNT * sizeof(struct data_clnt_arg));
    return RT_EOK;
}


static void testcase(int argc, char **argv)
{
    for (int i = 0; i < argc; i++)
    {
        LOG_D("param[%d]: %s", i, argv[i]);
    }
    test_datasvc_clnt_set_parameter(argc, argv);

    UTEST_UNIT_RUN(test_datasvc_clnt);
}
UTEST_TC_EXPORT(testcase, "tc_drv_rtc", utest_tc_init, utest_tc_cleanup, 10);
#endif
static void datac(int argc, char *argv[])
{
    if (argc >= 2)
    {
        if (strcmp(argv[1], "list") == 0)
        {
            int i;
            rt_kprintf("idx\thandle\tname\tcfg len\tdata len\n");
            for (i = 0; i < DATASVC_MAX_CLNT; i++)
                if (g_data_clnt[i].svc_name[0])
                    rt_kprintf("%d\t%d\t%s\t%d\t%d\n", i, g_data_clnt[i].handle, g_data_clnt[i].svc_name, g_data_clnt[i].config_len, g_data_clnt[i].data_len);
        }
        else
        {
            uint8_t index = atoi(argv[2]);
            if (strcmp(argv[1], "connect") == 0)
            {
                uint32_t context = atoh(argv[4]);
                g_data_clnt[index].handle = datac_open();
                strcpy(g_data_clnt[index].svc_name, argv[3]);
                g_data_clnt[index].context = context;
                datac_subscribe(g_data_clnt[index].handle, argv[3], datasvc_clnt_callback, index);
            }
            else if (strcmp(argv[1], "stress") == 0)
            {
                for (int i = 0; i < DATASVC_MAX_CLNT; i++)
                {
                    g_data_clnt[i].handle = datac_open();
                    strcpy(g_data_clnt[i].svc_name, argv[2]);
                    datac_subscribe(g_data_clnt[i].handle, argv[2], datasvc_clnt_callback, i);
                }
            }
            else if (strcmp(argv[1], "config") == 0)
            {
                g_data_clnt[index].config_len = strlen(argv[3]) >> 1;
                hex2data(argv[3], g_data_clnt[index].config, MAX_LEN >> 1);
                datac_config(g_data_clnt[index].handle, g_data_clnt[index].config_len, g_data_clnt[index].config);
            }
            else if (strcmp(argv[1], "tx") == 0)
            {
                g_data_clnt[index].data_len = strlen(argv[3]) >> 1;
                hex2data(argv[2], g_data_clnt[index].data, MAX_LEN);
                datac_tx(g_data_clnt[index].handle, g_data_clnt[index].data_len, g_data_clnt[index].data);
            }
            else if (strcmp(argv[1], "rx") == 0)
            {
                datac_rx(g_data_clnt[index].handle, atoi(argv[3]), NULL);
            }
            else if (strcmp(argv[1], "ping") == 0)
            {
                rt_err_t res = datac_ping(g_data_clnt[index].handle, (uint8_t)atoi(argv[3]));
                rt_kprintf("Ping %d res %d\n", index, res);
            }
            else if (strcmp(argv[1], "disc") == 0)
            {
                datac_unsubscribe(g_data_clnt[index].handle);
                memset(&(g_data_clnt[index]), 0, sizeof(g_data_clnt[index]));
            }
        }
    }
}
MSH_CMD_EXPORT(datac, Test data service subscriber);




/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
