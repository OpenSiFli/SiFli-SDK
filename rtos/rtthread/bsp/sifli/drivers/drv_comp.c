/**
  ******************************************************************************
  * @file   drv_comp.c
  * @author Sifli software development team
  * @brief COMP BSP driver
  * @{
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

#include <board.h>

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_comp COMP
  * @brief COMP BSP driver
  * @{
  */

#ifdef BSP_USING_COMP

//#define DRV_DEBUG
#define LOG_TAG             "drv.comp"
#include <drv_log.h>


struct sifli_comp_dev
{
    struct rt_device parent;
    COMP_HandleTypeDef hcomp;
};

static struct sifli_comp_dev sifli_comp_obj;


static rt_size_t comp_dev_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    COMP_HandleTypeDef *hcmp = (COMP_HandleTypeDef *)dev->user_data;
    int res;
    if (hcmp == NULL || buffer == NULL)
        return 0;

    res = HAL_COMP_PollForComp(hcmp, pos == 0 ? 0 : 1, 10000);
    if (res >= 0)
    {
        *((uint32_t *)buffer) = res;
        return 1;
    }
    LOG_I("res = %d\n", res);
    return 0;
}
static rt_size_t comp_dev_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    COMP_HandleTypeDef *hcmp = (COMP_HandleTypeDef *)dev->user_data;

    return 0;
}

static rt_err_t comp_dev_control(rt_device_t dev, int         cmd, void *args)
{
    COMP_HandleTypeDef *hcmp = (COMP_HandleTypeDef *)dev->user_data;
    HAL_StatusTypeDef res = HAL_OK;

    // change channel, speed, triger mode, callback ......
    switch (cmd)
    {
    case 0:
    {
        //uint32_t chnl = (uint32_t)args;
        res = HAL_COMP_Start(hcmp);
        if (res != HAL_OK)
        {
            LOG_I("comp start fail %d\n", res);
        }
        else
        {
            HAL_NVIC_SetPriority(LPCOMP_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(LPCOMP_IRQn);
        }
        break;
    }
    case 1:
    {
        //uint32_t chnl = (uint32_t)args;
        HAL_NVIC_DisableIRQ(LPCOMP_IRQn);
        res = HAL_COMP_Stop(hcmp);
        if (res != HAL_OK)
        {
            LOG_I("comp stop fail %d\n", res);
        }
        break;
    }
    case 2:
    {
        COMP_ConfigTypeDef *cfg = (COMP_ConfigTypeDef *)args;
        res = HAL_COMP_Config(hcmp, cfg);
        if (res != HAL_OK)
        {
            LOG_I("comp config fail %d\n", res);
            //return res;
        }
        break;
    }
    default:
        break;
    }
    return (rt_err_t)res;
}
#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops lpcom_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    comp_dev_read,
    comp_dev_write,
    comp_dev_control
};
#endif

void LPCOMP_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    //LOG_I("LPCOMP1_IRQHandler\n");

    HAL_COMP_IRQHandler(&(sifli_comp_obj.hcomp));

    /* leave interrupt */
    rt_interrupt_leave();
}

static int sifli_comp_init(void)
{
    int result = RT_EOK;

    sifli_comp_obj.hcomp.Instance = hwp_lpcomp;
    sifli_comp_obj.hcomp.ErrorCode = 0;
    sifli_comp_obj.hcomp.Lock = 0;
    sifli_comp_obj.hcomp.State = 0;
#if (USE_HAL_COMP_REGISTER_CALLBACKS == 1)
    sifli_comp_obj.hcomp.MspDeInitCallback = NULL;
    sifli_comp_obj.hcomp.MspInitCallback = NULL;
    sifli_comp_obj.hcomp.TriggerCallback = NULL;
#endif
    sifli_comp_obj.hcomp.Init.Mode = COMP_POWERMODE_HIGHSPEED;
    sifli_comp_obj.hcomp.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
    sifli_comp_obj.hcomp.Init.InvertingInput = COMP_INPUT_MINUS_VREF;
    sifli_comp_obj.hcomp.Init.Hysteresis = COMP_HYSTERESIS_NONE;
    sifli_comp_obj.hcomp.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
    sifli_comp_obj.hcomp.Init.InternalVRef = COMP_VREFINT_0D6V;
    sifli_comp_obj.hcomp.Init.WorkingPin = 0;

    HAL_StatusTypeDef res = HAL_COMP_Init(&sifli_comp_obj.hcomp);
    if (res == 0)   // register a rt-device to use device interface?
    {
        struct rt_device *device = &(sifli_comp_obj.parent);
        /* set device type */
        device->type    = RT_Device_Class_Miscellaneous;
        /* initialize device interface */
#ifdef RT_USING_DEVICE_OPS
        device->ops     = &lpcom_ops;
#else
        device->init    = RT_NULL;
        device->open    = RT_NULL;
        device->close   = RT_NULL;
        device->read    = comp_dev_read;
        device->write   = comp_dev_write;
        device->control = comp_dev_control;
#endif
        device->user_data = &(sifli_comp_obj.hcomp);
        rt_device_register(device, "comp", RT_DEVICE_FLAG_RDWR);

    }

    return result;
}
INIT_BOARD_EXPORT(sifli_comp_init);

#define DRV_COMP_TEST
#ifdef DRV_COMP_TEST

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
    /* Prevent unused argument(s) compilation warning */
    //UNUSED(hcomp);

    LOG_I("Get comp result %d\n", HAL_COMP_GetOutputLevel(hcomp));
}

#include <string.h>

int cmd_comp(int argc, char *argv[])
{
    rt_device_t dev;
    if (argc < 3)
    {
        LOG_I("Invalid parameter\n");
        LOG_I("comp -enable/-read/-config/-disable  channel\n");
    }
    else if (strcmp(argv[1], "-enable") == 0)
    {
        uint32_t chnl = atoi(argv[2]);
        dev = rt_device_find("comp");
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            LOG_I("COMP channel %d enabled\n",  chnl);
        }
        else
        {
            LOG_I("Find comp device fail\n");
        }
    }
    else if (strcmp(argv[1], "-disable") == 0)
    {
        uint32_t chnl = atoi(argv[2]);
        dev = rt_device_find("comp");
        if (dev)
        {
            rt_device_close(dev);
            LOG_I("COMP channel %d disabled\n", chnl);
        }
        else
        {
            LOG_I("Find COMP device fail\n");
        }
    }
    else if (strcmp(argv[1], "-read") == 0)
    {
        uint32_t chnl = atoi(argv[2]);
        uint32_t value, res;
        dev = rt_device_find("comp");
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            res = rt_device_read(dev, chnl, &value, 1);
            LOG_I("Read COMP channel %d : %d, res = %d \n", chnl, value, res);
        }
        else
        {
            LOG_I("Find COMP device fail\n");
        }
    }
    else if (strcmp(argv[1], "-conf") == 0)
    {
        COMP_ConfigTypeDef cfg;
        int res;
        if (argc <= 6)
            LOG_I("Invalid parameter\n");

        cfg.WorkingPin = atoi(argv[2]);
        cfg.Mode = atoi(argv[3]);
        cfg.InvertingInput = atoi(argv[4]);
        cfg.TriggerMode = atoi(argv[5]);
        cfg.InternalVRef = atoi(argv[6]);
        dev = rt_device_find("comp");
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            res = rt_device_control(dev, 2, &cfg);
            LOG_I("config comp res = %d \n", res);
        }
        else
        {
            LOG_I("Find COMP device fail\n");
        }
    }
    else if (strcmp(argv[1], "-start") == 0)
    {
        uint32_t chnl = atoi(argv[2]);
        uint32_t res;
        dev = rt_device_find("comp");
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            res = rt_device_control(dev, 0, 0);
            LOG_I("start COMP channel %d, res = %d \n", chnl, res);
        }
        else
        {
            LOG_I("find COMP device fail\n");
        }
    }
    else if (strcmp(argv[1], "-stop") == 0)
    {
        uint32_t chnl = atoi(argv[2]);
        uint32_t res;
        dev = rt_device_find("comp");
        if (dev)
        {
            rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);
            res = rt_device_control(dev, 1, 0);
            LOG_I("start COMP channel %d, res = %d \n", chnl, res);
        }
        else
        {
            LOG_I("find COMP device fail\n");
        }
    }
    else
    {
        LOG_I("Invalid parameter\n");
        LOG_I("comp -enable/-read/-config/-disable  channel\n");
    }

    return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_comp, __cmd_comp, Test comp driver);

#endif /* DRV_COMP_TEST */

#endif /* BSP_USING_COMP */

/// @} drv_lcpu
/// @} bsp_driver

/// @} file
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
