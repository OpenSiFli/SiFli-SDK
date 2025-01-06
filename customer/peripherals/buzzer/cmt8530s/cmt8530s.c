/**
  ******************************************************************************
  * @file   cmt8530s.c
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

#include "cmt8530s.h"

#define DRV_DEBUG
#define LOG_TAG              "drv.cmt8530s"
#include <drv_log.h>


static struct rt_device_pwm *cmt8530s_dev = NULL;

int cmt8530s_init()
{
    cmt8530s_dev = (struct rt_device_pwm *)rt_device_find(BUZZER_INTERFACE_NAME);
    if (cmt8530s_dev == NULL)
    {
        LOG_E("CM58530S can not find device %s\n", BUZZER_INTERFACE_NAME);
        return -1;
    }
    // switch pinmux to gptimer
    HAL_PIN_Set(PAD_PB_00, GPTIM1_CH_0, PIN_PULLUP, 0);
    //HAL_PIN_Set(PAD_PB_01, GPTIM1_CH_1, PIN_PULLUP, 0);

    return 0;
}

int cmt8530s_open()
{
    //uint32_t period;  // move freq setting to other interface
    if (cmt8530s_dev)
    {
        //period = 1000000000 / BUZZER_FREQ; // period is ns based
        //rt_pwm_set(cmt8530s_dev, BUZZER_CHANEL_NUM, period, period / 2);
        rt_pwm_enable(cmt8530s_dev, BUZZER_CHANEL_NUM);
        return 0;
    }

    return -1;
}

int cmt8530s_set_freq(uint32_t freq)
{
    uint32_t period;
    if (freq < 20 || freq > 20000)
        return -2;

    period = 1000000000 / freq;
    if (cmt8530s_dev)
    {
        //period = 1000000000 / BUZZER_FREQ; // period is ns based
        rt_pwm_set(cmt8530s_dev, BUZZER_CHANEL_NUM, period, period / 2);
        return 0;
    }

    return -1;
}

int cmt8530s_close()
{
    if (cmt8530s_dev)
    {
        rt_pwm_disable(cmt8530s_dev, BUZZER_CHANEL_NUM);
        return 0;
    }

    return -1;
}


#define DRV_CMT8530S_TEST
#ifdef DRV_CMT8530S_TEST

#include <string.h>
#include "buzzer_test.h"


int cmt8530s_test(int argc, char *argv[])
{
    int ret;
    if (argc < 2)
    {
        LOG_I("Invalid parameter\n");
        return 0;
    }
    if (strcmp(argv[1], "-init") == 0)
    {
        ret = cmt8530s_init();
        LOG_I("cmt8530s_init %d\n", ret);
    }
    else if (strcmp(argv[1], "-config") == 0)
    {
        int freq = atoi(argv[2]);
        ret = cmt8530s_set_freq(freq);
        LOG_I("cmt8530s_set_freq to %d: %d\n", freq, ret);
    }
    else if (strcmp(argv[1], "-open") == 0)
    {
        ret = cmt8530s_open();
        LOG_I("cmt8530s_open %d\n", ret);
    }
    else if (strcmp(argv[1], "-close") == 0)
    {
        ret = cmt8530s_close();
        LOG_I("cmt8530s_close %d\n", ret);
    }
    else if (strcmp(argv[1], "-loop") == 0)
    {
        int i;
        for (i = 1; i < 100; i++)
        {
            cmt8530s_set_freq(i * 100);
            rt_thread_delay(2000);
        }
    }
    else if (strcmp(argv[1], "-music") == 0)
    {
        int mid = atoi(argv[2]);
        if (mid == 1)
        {
            musicPlay(MyScore1);
        }
        else if (mid == 2)
        {
            musicPlay(MyScore2);
        }
        else if (mid == 3)
        {
            musicPlay(MyScore3);
        }
    }
    else
    {
        LOG_I("Invalid parameter\n");
    }

    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmt8530s_test, __cmd_cmt8530s, Test hw cmt8530s);
#endif //DRV_CMT8530S_TEST


