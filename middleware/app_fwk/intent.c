/**
  ******************************************************************************
  * @file   gui_app_intent.c
  * @author Sifli software development team
  * @brief Unify general used intent description.
  ******************************************************************************
*/
/*
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
#include <board.h>
#include <string.h>
#include <stdlib.h>
#include "intent_int.h"

#define DBG_TAG           "APP.FWK.INT"
#define DBG_LVL           DBG_LOG
#include "rtdbg.h"

typedef p_intent intent_t;


#if 0

    #define DEFAULT_APP           "app_fwk"
    #define DEFAULT_SMS_APP       "message"
    #define DEFAULT_CALL_APP      "phone"
    #define DEFAULT_ALARM_APP     "alarm"
    #define DEFAULT_POWER_APP     "power"


    #define INTENT_NEW_SMS          DEFAULT_SMS_APP" "DEFAULT_ACTION_NEW
    #define INTENT_INCOMING_CALL    DEFAULT_CALL_APP" "DEFAULT_ACTION_INCOMINGCALL
    #define INTENT_OUTGOING_CALL    DEFAULT_CALL_APP" "DEFAULT_ACTION_OUTGOINGCALL
    #define INTENT_ALARM_TIMESUP    DEFAULT_ALARM_APP" "DEFAULT_ACTION_ALERT
#endif /* 0 */

extern int gui_app_run_by_intent(intent_t intent);


/*Add action or parameter to intent*/
static uint32_t add_to_intent(p_intent i, const char *str)
{
    uint32_t write_len, str_len;

    RT_ASSERT(NULL != i);
    RT_ASSERT(NULL != str);

    str_len = strlen(str);
    write_len = str_len + 1; /*prefix*/

    if (i->content_len + write_len >= INTENT_MAX_LEN)
    {
        return 0;
    }
    else
    {
        uint32_t pos = i->content_len;

        //if (pos != 0)
        //i->content[pos++] = INTENT_SEPARATER;


        memcpy(&i->content[pos], str, str_len);
        pos += str_len;
        i->content[pos] = INTENT_SEPARATER;

        i->content_len += write_len;
        return write_len;
    }
}

static int app_cmd_split(char *cmd, rt_size_t length, char *argv[], int max_argc)
{
    int argc = 0;
    char *ptr = cmd;

    while ((ptr - cmd) < (int)length)
    {
        /* strip bank and tab */
        while ((*ptr == ' ' || *ptr == '\t') && (ptr - cmd) < (int)length)
            *ptr++ = '\0';
        /* check whether it's the end of line */
        if ((ptr - cmd) >= (int)length) break;

        /* handle string with quote */
        if (*ptr == '"')
        {
            argv[argc++] = ++ptr;

            /* skip this string */
            while (*ptr != '"' && (ptr - cmd) < (int)length)
                if (*ptr ++ == '\\')  ptr ++;
            if ((ptr - cmd) >= (int)length) break;

            /* skip '"' */
            *ptr ++ = '\0';
        }
        else
        {
            argv[argc++] = ptr;
            while ((*ptr != ' ' && *ptr != '\t') && (ptr - cmd) < (int)length)
                ptr ++;
        }

        if (argc >= max_argc) break;
    }

    return argc;
}



intent_t intent_init(const char *action)
{
    char *action_copy;
    rt_size_t len;
    p_intent i;

    if (!action) return NULL;

    //Init new intent data
    i = (intent_t) rt_malloc(sizeof(_intent));
    if (!i) return NULL;
    memset(i->content, INTENT_SEPARATER, sizeof(i->content));
    i->content_len = 0;


    //Copy 'action' to 'action_copy'
    len = strlen(action);
    len = (len >= (INTENT_MAX_LEN - 1)) ? (INTENT_MAX_LEN - 1) : len;
    action_copy = (char *) rt_malloc(len + 1);
    if (NULL == action_copy)
    {
        rt_free(i);
        return NULL;
    }
    memcpy(action_copy, action, len);
    action_copy[len] = '\0';


    /* split arguments, and add to intent */
    {
        int c, argc;
        char *argv[8];
        memset(argv, 0x00, sizeof(argv));
        argc = app_cmd_split(action_copy, len, argv, 8);
        p_intent tmp = i;

        for (c = 0; c < argc; c++) add_to_intent(i, argv[c]);
    }
    rt_free(action_copy);


    return i;
}

void intent_deinit(intent_t i)
{
    if (i)
        rt_free(i);
}


int intent_set_string(intent_t i, const char *name, const char *value)
{
    uint32_t ret_v;
    uint32_t name_len = strlen(name);
    uint32_t value_len = strlen(value);
    uint32_t buffer_len = name_len + 1 /* '=' */ + value_len + 1;

    char *buffer = (char *) rt_malloc(buffer_len);
    if (!buffer) return RT_ENOMEM;

    memset(buffer, 0, buffer_len);

    rt_snprintf(buffer, buffer_len, "%s=%s", name, value);
    ret_v = add_to_intent(i, buffer);
    rt_free(buffer);

    if (ret_v > 0)
        return RT_EOK;
    else
        return RT_ERROR;
}

const char *intent_get_string(intent_t i, const char *name)
{
    uint32_t idx = 0;
    uint32_t name_len;

    RT_ASSERT(NULL != i);
    RT_ASSERT(NULL != name);

    name_len = strlen(name);
    while (idx < i->content_len)
    {
        if (i->content[idx] != INTENT_SEPARATER)
        {
            if (0 == strncmp(&(i->content[idx]), name, name_len))
            {
                if ('=' == i->content[idx + name_len])
                    return &(i->content[idx + name_len + 1]);
            }

            //goto next token
            while (i->content[idx] != INTENT_SEPARATER) idx++;
        }
        else
        {
            //goto next token
            while (INTENT_SEPARATER == i->content[idx]) idx++;
        }
    }

    return NULL;
}


int intent_set_uint32(intent_t i, const char *name, uint32_t value)
{
    uint32_t ret_v;
    uint32_t name_len;
    uint32_t buffer_len;
    char *buffer;

    RT_ASSERT(NULL != i);
    RT_ASSERT(NULL != name);

    name_len = strlen(name);
    buffer_len = name_len + 1 /* '=' */ + 8 + 1;

    buffer = (char *) rt_malloc(buffer_len);
    if (!buffer) return RT_ENOMEM;

    memset(buffer, 0, buffer_len);

    rt_snprintf(buffer, buffer_len, "%s=%x", name, value);
    ret_v = add_to_intent(i, buffer);
    rt_free(buffer);

    if (ret_v > 0)
        return RT_EOK;
    else
        return RT_ERROR;
}


uint32_t intent_get_uint32(intent_t i, const char *name, uint32_t err_value)
{
    const char *str = intent_get_string(i, name);

    if (NULL == str) return err_value;

    return strtoul(str, 0, 16);
}

const char *intent_get_action(intent_t i)
{
    if (!i) return NULL;
    p_intent p = (p_intent) i;
    return (const char *)&p->content[0];
}


void printf_intent(intent_t i)
{
    if (NULL == i)
    {
        LOG_I("NULL");
    }
    else
    {
        p_intent tmp = (intent_t) rt_malloc(sizeof(_intent));
        if (NULL == tmp)
        {
            LOG_E("Fail to alloc print buf");
        }
        else
        {
            rt_memcpy(tmp, i, sizeof(_intent));

            //Overwrite INTENT_SEPARATER with visble character except last INTENT_SEPARATER.
            for (uint32_t idx = 0; idx < INTENT_MAX_LEN - 1; idx++)
            {
                if ((INTENT_SEPARATER == tmp->content[idx]) && (INTENT_SEPARATER != tmp->content[idx + 1]))
                {
                    tmp->content[idx] = ' ';
                }
                else if (('\0' == tmp->content[idx]) && ('\0' == tmp->content[idx + 1])) //Reach the end
                {
                    break;
                }
            }

            LOG_I("[%s]", &tmp->content[0]);

            rt_free(tmp);
        }
    }
}

int intent_runapp(intent_t i)
{
    rt_err_t err;
    RT_ASSERT(NULL != i);

    err =  gui_app_run_by_intent(i);

    return err;
}





