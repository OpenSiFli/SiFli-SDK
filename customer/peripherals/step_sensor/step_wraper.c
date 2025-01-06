/**
  ******************************************************************************
  * @file   step_wraper.c
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

#include <string.h>
#include "board.h"
#include "step_wraper.h"
#include "lsm6dsl.h"
#include "counter.h"

#ifdef STEP_USING_VIRTUAL

rt_thread_t vsteo_thread = NULL;

static void virt_step_sensor_task(void *params)  //20ms
{
    int32_t ret;
    int debug_cnt = 0;
    int accx = 0, accy = 0, accz = 0;
    SportDataType data = {0};

    while (1)
    {
        ret = lsm6dsl_accel_read(&accx, &accy, &accz);
        if (ret != 0)
            rt_kprintf("get accel data fail\n");
        Sport_Calculator(accx, accy, accz); // suppose parameter mg based

        debug_cnt++;    // remove it if close debug
        if ((debug_cnt & 0x7f) == 0)    // output each 128 * 20 ms
        {
            Read_SportData(&data);
            rt_kprintf("Step count %d\n", data.steps);
            //rt_kprintf("Input 0x%x, 0x%x, 0x%x\n", accx, accy, accz);
        }

        rt_thread_delay(20);    // continue check with 50hz
    }
}

int virt_step_init(void)
{
    int res;

    res = lsm6dsl_init();
    if (res != 0)
        return res;

    Sport_Init();
    Set_Parameter(175, 80);
    vsteo_thread = rt_thread_create("virt_step", virt_step_sensor_task, NULL, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
    if (vsteo_thread != NULL)
    {
        rt_thread_startup(vsteo_thread);
        rt_kprintf("step thread started\n");
    }
    else
    {
        rt_kprintf("Create step thread fail\n");
        return 1;
    }

    return 0;
}

int vstep_get_step()
{
    SportDataType data = {0};

    Read_SportData(&data);

    return (int)data.steps;
}

int vstep_get_distance()
{
    SportDataType data = {0};

    Read_SportData(&data);

    return (int)data.dist;
}


#define DRV_VSTEO_TEST
#ifdef DRV_VSTEO_TEST

int vstep_test(int argc, char *argv[])
{
    if (argc < 2)
    {
        rt_kprintf("Invalid parameter\n");
        return 0;
    }

    if (strcmp(argv[1], "-open") == 0) // pedometer lib-c
    {
        int res = virt_step_init();
        if (res == 0)
            rt_kprintf("Step sensor start\n");
        else
            rt_kprintf("Step sensor start fail\n");
    }
    else if (strcmp(argv[1], "-s") == 0) // pedometer lib-c
    {
        rt_kprintf("Step %d\n", vstep_get_step());
    }
    else if (strcmp(argv[1], "-d") == 0) // pedometer lib-c
    {
        rt_kprintf("Distance %d\n", vstep_get_distance());
    }
    else
    {
        rt_kprintf("Invalid parameter\n");
    }

    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(vstep_test, __cmd_step, Test hw step);
#endif //DRV_VSTEO_TEST

#endif /*STEP_USING_VIRTUAL*/
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
