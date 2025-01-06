/**
  ******************************************************************************
  * @file   drv_aw8155.c
  * @author Sifli software development team
  * @brief   Audio Process driver adaption layer
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
#include <stdlib.h>
#include <string.h>
#include "board.h"
#if defined(PMIC_CTRL_ENABLE)
    #include "pmic_controller.h"
#endif
#define DBG_TAG           "aw8155"
#include "rtdbg.h"

#define  AW8155_WORK_MODE   1   // only 1 2 3 4
#if (defined(BSP_USING_BOARD_EH_SS6700XXX) || defined(BSP_USING_BOARD_EH_LB561XXX) || defined(BSP_USING_BOARD_EH_LB563XXX) \
    || defined(BSP_USING_BOARD_EH_LB561XXX_V2) || defined(BSP_USING_BOARD_EH_LB563XXX_V2))

#define  AW8155_GPIO_PIN    28
#define  AW8155_GPIO_HWP    hwp_gpio1
#elif defined(BSP_USING_BOARD_EM_LB567XXX) || defined(BSP_USING_BOARD_EM_LB566XXX)
#define  AW8155_GPIO_PIN    18
#define  AW8155_GPIO_HWP    hwp_gpio2
#elif defined(SOC_SF32LB52X)
#define  AW8155_GPIO_PIN    10
#define  AW8155_GPIO_HWP    hwp_gpio1
#else
#define  AW8155_GPIO_PIN    21
#define  AW8155_GPIO_HWP    hwp_gpio2
#endif

void aw8155_gpio_write(bool State)
{
    GPIO_PinState PinState = (GPIO_PinState)State;
    HAL_GPIO_WritePin(AW8155_GPIO_HWP, AW8155_GPIO_PIN, PinState);
    HAL_Delay_us(5);
}

static int aw8155_mode = AW8155_WORK_MODE;
void sifli_aw8155_start()
{
    int i = aw8155_mode;
    GPIO_InitTypeDef GPIO_InitStruct;
#if defined(PMIC_CTRL_ENABLE)
    pmic_device_control(PMIC_OUT_VBAT_HVSW150_1, 1, 1);//wait ???
#else
    /* @todo power handle */
    rt_kprintf("sifli_aw8155 to do power handle \n");
#endif
    // set sensor pin to output mode
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin  = AW8155_GPIO_PIN;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(AW8155_GPIO_HWP, &GPIO_InitStruct);
    HAL_Delay_us(550);

    while (1)
    {
        aw8155_gpio_write(1);
        i--;
        if (i > 0)
        {
            aw8155_gpio_write(0);
        }
        else
        {
            break;
        }
    }
    rt_kprintf("sifli_aw8155_start,mode:%d %d\n", aw8155_mode, AW8155_GPIO_PIN);

}

void sifli_aw8155_stop()
{
    aw8155_gpio_write(0);
    HAL_Delay_us(550);
#if defined(PMIC_CTRL_ENABLE)
    pmic_device_control(PMIC_OUT_VBAT_HVSW150_1, 0, 1);
#else
    /* @todo power handle */
#endif
    rt_kprintf("sifli_aw8155_stop \n");
}

int apa_set_mode(int argc, char *argv[])
{
    rt_thread_t thread;
    int mode;

    if (argc != 2)
    {
        rt_kprintf("arg para num error\n");
        return -1;
    }
    mode = strtol(argv[1], NULL, 10);
    rt_kprintf("analog PA 8155 work mode=%d\n", mode);

    if ((mode >= 1) && (mode <= 4))
    {
        aw8155_mode = mode;
    }
    else
    {
        rt_kprintf("analog PA mode error \n");
    }


    return 0;
}

MSH_CMD_EXPORT(apa_set_mode, set analog pa mode);



