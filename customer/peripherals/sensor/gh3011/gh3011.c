/**
  ******************************************************************************
  * @file   gh3011.c
  * @author Sifli software development team
  * @brief   This file includes the gh3011 driver functions
  *
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
#include <math.h>
#include <string.h>
#include "stdlib.h"
#include "board.h"
#include "gh3011_example.h"

#define DRV_DEBUG
#define LOG_TAG              "drv.hbd"
#include <drv_log.h>


#ifdef HR_USING_GH3011

extern void hal_gh30x_pin_set(uint8_t en);

int init_gh3011_sensor(void)
{
    int res =  gh30x_module_init();
    if (res == 1)
        return 0;

    return 1;
}

int open_gh3011(void)
{
    gh30x_module_start(2);
    return 0;
}

int close_gh3011(void)
{
    gh30x_module_stop();
    return 0;
}

#define DRV_GH3011_TEST

#ifdef DRV_GH3011_TEST
#include <string.h>

int cmd_hbd(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "-open") == 0)
        {
            int res = gh30x_module_init();
            LOG_I("Initial GH3011 %d\n", res);
        }
        else if (strcmp(argv[1], "-hb") == 0)
        {
            gh30x_module_start(2);
            LOG_I("start HB\n");
        }
        else if (strcmp(argv[1], "-spo") == 0)
        {
            gh30x_module_start(7);
            LOG_I("start spo2\n");
        }
        else if (strcmp(argv[1], "-hb2") == 0)
        {
            gh30x_module_start_without_adt(2);
            LOG_I("start HB\n");
        }
        else if (strcmp(argv[1], "-spo2") == 0)
        {
            gh30x_module_start_without_adt(7);
            LOG_I("start spo2\n");
        }
        else if (strcmp(argv[1], "-stop") == 0)
        {
            gh30x_module_stop();
            LOG_I("stop gh3011\n");
        }
        else
        {
            LOG_I("Invalid parameter\n");
        }
    }
    else
    {
        LOG_I("Invalid parameter\n");
    }
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_hbd, __cmd_hbd, Test driver gh3011);

#endif //DRV_GH3011_TEST


#endif  // HR_USING_GH3011
/********END OF FILE********* (C) COPYRIGHT 2018 .********/
