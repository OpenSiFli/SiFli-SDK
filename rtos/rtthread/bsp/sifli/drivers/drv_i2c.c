/**
  ******************************************************************************
  * @file   drv_i2c.c
  * @author Sifli software development team
  * @brief I2C BSP driver
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

#include <string.h>
#include "board.h"
#include "drv_config.h"
#include "drv_i2c.h"

#define DBG_LEVEL            DBG_ERROR //DBG_LOG //
#define LOG_TAG              "drv.i2c"

#include <drv_log.h>


/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_i2c I2C
  * @brief I2C BSP driver
  * @{
  */


/**********************************
    SFLB55X LCPU ROM patch
*************************************/
#if defined(SOC_SF32LB55X)&&defined(SOC_BF0_LCPU)
static void register_i2c_device(uint8_t i);

extern const struct rt_i2c_bus_device_ops i2c_ops_rom;
static struct rt_i2c_bus_device_ops lcpu_patch_ops;


#if BSP_LB55X_CHIP_ID > 2 //A3


struct rt_i2c_bus_device_a3
{
    struct rt_device parent;
    const struct rt_i2c_bus_device_ops *ops;
    rt_uint16_t  flags;
    rt_uint16_t  addr;
    struct rt_mutex lock;
    rt_uint32_t  timeout;
    rt_uint32_t  retries;
    struct rt_i2c_configuration config; //<---- The different one in A3 ROM
    void *priv;
};

typedef struct bf0_i2c_a3
{
    struct rt_i2c_bus_device_a3 bus;
    I2C_HandleTypeDef handle;
    bf0_i2c_config_t *bf0_i2c_cfg;
    struct
    {
        DMA_HandleTypeDef dma_rx;
        DMA_HandleTypeDef dma_tx;
    } dma;
    rt_uint8_t i2c_dma_flag;
} bf0_i2c_a3_t;

#define I2C_NUM  (sizeof(bf0_i2c_cfg) / sizeof(bf0_i2c_cfg[0]))

static bf0_i2c_config_t bf0_i2c_cfg[] =
{
#ifdef BSP_USING_I2C4
    BF0_I2C4_CFG,
#endif

#ifdef BSP_USING_I2C5
    BF0_I2C5_CFG,
#endif

#ifdef BSP_USING_I2C6
    BF0_I2C6_CFG,
#endif
};


static bf0_i2c_a3_t i2c_a3_obj[I2C_NUM] = {0};
static struct rt_i2c_configuration rt_i2c_cfg_default[I2C_NUM] =
{
#if defined(BSP_USING_I2C1)||defined(BSP_USING_I2C2)||defined(BSP_USING_I2C3)
#error "Can't use I2C1~3 on LPSYS"
#endif

#ifdef BSP_USING_I2C4
    {
        0,       //rt_uint16_t mode;  RT_I2C_ADDR_10BIT / RT_I2C_NO_START / RT_I2C_IGNORE_NACK / RT_I2C_NO_READ_ACK
        0,       //rt_uint16_t addr;
        5000,    //rt_uint32_t timeout;
        400000   //rt_uint32_t max_hz;
    },
#endif
#ifdef BSP_USING_I2C5
    {
        0,       //rt_uint16_t mode;  RT_I2C_ADDR_10BIT / RT_I2C_NO_START / RT_I2C_IGNORE_NACK / RT_I2C_NO_READ_ACK
        0,       //rt_uint16_t addr;
        5000,    //rt_uint32_t timeout;
        400000   //rt_uint32_t max_hz;
    },
#endif
#ifdef BSP_USING_I2C6
    {
        0,       //rt_uint16_t mode;  RT_I2C_ADDR_10BIT / RT_I2C_NO_START / RT_I2C_IGNORE_NACK / RT_I2C_NO_READ_ACK
        0,       //rt_uint16_t addr;
        5000,    //rt_uint32_t timeout;
        400000   //rt_uint32_t max_hz;
    },
#endif
};

static uint32_t get_index_by_bus_handle(struct rt_i2c_bus_device *bus)
{
    for (int i = 0; i < I2C_NUM; i++)
    {
        if ((uint32_t)bus == (uint32_t)&i2c_a3_obj[i].bus) return i;
    }

    return UINT32_MAX;
}

#ifdef BSP_USING_I2C
static void I2Cx_IRQHandler(uint32_t index)
{
    I2C_HandleTypeDef *handle;
    /* enter interrupt */
    rt_interrupt_enter();

    handle = &i2c_a3_obj[index].handle;

    if (handle->XferISR != NULL)
    {
        handle->XferISR(handle, 0, 0);
    }


    /* leave interrupt */
    rt_interrupt_leave();
}

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
static void I2Cx_DMA_IRQHandler(uint32_t index)
{
    I2C_HandleTypeDef *handle;

    /* enter interrupt */
    rt_interrupt_enter();

    handle = &i2c_a3_obj[index].handle;

    if (handle->State == HAL_I2C_STATE_BUSY_TX)
    {
        HAL_DMA_IRQHandler(handle->hdmatx);
    }
    else if (handle->State == HAL_I2C_STATE_BUSY_RX)
    {
        HAL_DMA_IRQHandler(handle->hdmarx);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif

#if defined(BSP_USING_I2C4)
void I2C4_IRQHandler(void)
{
    I2Cx_IRQHandler(I2C4_INDEX);
}

#if defined(BSP_I2C4_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void I2C4_DMA_IRQHandler(void)
{
    I2Cx_DMA_IRQHandler(I2C4_INDEX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif

#if defined(BSP_USING_I2C5)
void I2C5_IRQHandler(void)
{
    I2Cx_IRQHandler(I2C5_INDEX);
}

#if defined(BSP_I2C5_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void I2C5_DMA_IRQHandler(void)
{
    I2Cx_DMA_IRQHandler(I2C5_INDEX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif

#if defined(BSP_USING_I2C6)
void I2C6_IRQHandler(void)
{
    I2Cx_IRQHandler(I2C6_INDEX);
}

#if defined(BSP_I2C6_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void I2C6_DMA_IRQHandler(void)
{
    I2Cx_DMA_IRQHandler(I2C6_INDEX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif

static rt_size_t master_xfer_lcpu_patch_a3(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    rt_size_t ret_v;
    struct bf0_i2c_a3 *bf0_i2c = (struct bf0_i2c_a3 *)bus;
    I2C_HandleTypeDef *hi2c = &bf0_i2c->handle;
    RT_ASSERT(bf0_i2c != RT_NULL);
    LOG_I("master_xfer_lcpu_patch_a3 %d, %x", get_index_by_bus_handle(bus), &bf0_i2c->handle);

    __HAL_I2C_ENABLE(&bf0_i2c->handle);

    for (rt_uint32_t i = 0; i < num; i++)
    {
        if ((msgs[i].flags & RT_I2C_RD) && (bus->parent.open_flag & RT_DEVICE_FLAG_DMA_RX))
        {
            mpu_dcache_invalidate(msgs[i].buf, msgs[i].len);
        }
    }

    ret_v = i2c_ops_rom.master_xfer(bus, msgs, num);



    if (__HAL_I2C_GET_FLAG(hi2c, I2C_SR_UB) == SET) //Fix I2C STOP not finished in ROM
    {
        HAL_Delay_us(100); //STOP bit takes 100us at I2C 10KHz
    }

    if (ret_v != num) //Reset bus if error occurred
    {
        LOG_E("STATE %d, i2c bus reset and send 9 clks", HAL_I2C_GetState(&bf0_i2c->handle));
        HAL_I2C_Reset(&bf0_i2c->handle);
    }
    __HAL_I2C_DISABLE(&bf0_i2c->handle);
    return ret_v;
}

#if 0
static rt_err_t i2c_bus_control_lcpu_patch_a3(struct rt_i2c_bus_device *bus, rt_uint32_t cmd, void *arg)
{
    rt_err_t ret_v;
    struct bf0_i2c_a3 *bf0_i2c = (struct bf0_i2c_a3 *)bus;

    LOG_I("i2c_bus_control_lcpu_patch_a3 %d, %x", get_index_by_bus_handle(bus), &bf0_i2c->handle);

    ret_v = i2c_ops_rom.i2c_bus_control(bus, cmd, arg);


    return ret_v;
}
#endif /* 0 */

static rt_err_t i2c_bus_configure_lcpu_patch_a3(struct rt_i2c_bus_device *bus, struct rt_i2c_configuration *configuration)
{
    rt_err_t ret_v;
    struct bf0_i2c_a3 *bf0_i2c = (struct bf0_i2c_a3 *)bus;
    I2C_HandleTypeDef *hi2c = &bf0_i2c->handle;
    uint32_t i2c_idx = get_index_by_bus_handle(bus);
    LOG_I("i2c_bus_configure_lcpu_patch_a3 %d,%x", i2c_idx, &bf0_i2c->handle);

    if (!configuration) //Use default cfg
    {
        configuration = &rt_i2c_cfg_default[i2c_idx];
    }

    {
        //Reset I2C before call HAL_I2C_Init of ROM
        if (hi2c->Instance == hwp_i2c4)
            HAL_RCC_ResetModule(RCC_MOD_I2C4);
        else if (hi2c->Instance == hwp_i2c5)
            HAL_RCC_ResetModule(RCC_MOD_I2C5);
        else if (hi2c->Instance == hwp_i2c6)
            HAL_RCC_ResetModule(RCC_MOD_I2C6);
        else
            HAL_ASSERT(0);

        //Clear LCR.SLV to 0, to disbale send I2C 'bus reset' in ROM's HAL_I2C_Init
        hi2c->Instance->LCR = 0;
    }

    ret_v = i2c_ops_rom.i2c_bus_configure(bus, configuration);

    //Fix bugs of HAL_I2C_Init in ROM
    {
        //bug1: I2C controller need reset it self after send bus reset
        hi2c->Instance->CR |= I2C_CR_UR;
        HAL_Delay_us(1);        // Delay at least 9 cycle.
        hi2c->Instance->CR &= ~I2C_CR_UR;

        //bug2: I2C enter slave mode while enable, enable it only in master_xfer
        __HAL_I2C_DISABLE(hi2c);


        //bug3: The clock divider of Fast mode & Standard mode are not correct
        uint32_t i2c_clk = HAL_RCC_GetPCLKFreq(hi2c->core, 1);

        uint32_t flv;
        /* Standard mode only use to send bus reset, use lowest frequence */
        uint32_t slv = I2C_LCR_SLV_Msk >> I2C_LCR_SLV_Pos;


        int div2 = 7 + hi2c->Init.ClockSpeed / 200000;  // verify on 100k, 200k and 400k
        flv = ((i2c_clk / hi2c->Init.ClockSpeed - hi2c->dnf - div2) / 2);

        HAL_ASSERT(flv <= (I2C_LCR_FLV_Msk >> I2C_LCR_FLV_Pos));
        MODIFY_REG(hi2c->Instance->LCR, I2C_LCR_FLV_Msk | I2C_LCR_SLV_Msk, (flv << I2C_LCR_FLV_Pos) | (slv << I2C_LCR_SLV_Pos));


        hi2c->Instance->IER = 0;

        //bug4: Fixed slave address conflict with customer's deivce
        hi2c->Instance->SAR = hi2c->Init.OwnAddress1;
    }

    return ret_v;
}

static void register_i2c_device(uint8_t i)
{
    HAL_StatusTypeDef ret;
    rt_uint32_t flag = RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX;

    i2c_a3_obj[i].bf0_i2c_cfg = &bf0_i2c_cfg[i];
    i2c_a3_obj[i].bus.parent.user_data = &bf0_i2c_cfg[i];
    i2c_a3_obj[i].bus.ops = &lcpu_patch_ops; //Replace with patch 'ops'
    i2c_a3_obj[i].handle.Instance = bf0_i2c_cfg[i].Instance;


    if (i2c_a3_obj[i].i2c_dma_flag)
    {
        flag |= (RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
        __HAL_LINKDMA(&i2c_a3_obj[i].handle, hdmarx, i2c_a3_obj[i].dma.dma_rx);
        __HAL_LINKDMA(&i2c_a3_obj[i].handle, hdmatx, i2c_a3_obj[i].dma.dma_tx);
        HAL_I2C_DMA_Init(&i2c_a3_obj[i].handle, bf0_i2c_cfg[i].dma_rx, bf0_i2c_cfg[i].dma_tx);

        //HAL_NVIC_SetPriority(bf0_i2c_cfg[i].dma_rx->dma_irq, 0, 0);
        //HAL_NVIC_EnableIRQ(bf0_i2c_cfg[i].dma_rx->dma_irq);
    }

    rt_i2c_bus_device_register((struct rt_i2c_bus_device *) &i2c_a3_obj[i].bus, //Treat a3 bus as cur bus
                               i2c_a3_obj[i].bf0_i2c_cfg->device_name, flag);

    //ret = HAL_I2C_Init(&i2c_a3_obj[i].handle);
    ret = lcpu_patch_ops.i2c_bus_configure((struct rt_i2c_bus_device *) &i2c_a3_obj[i].bus,  //Treat a3 bus as cur bus
                                           &rt_i2c_cfg_default[i]);
    RT_ASSERT(HAL_OK == ret);

    LOG_I("%s bus init done, %x", i2c_a3_obj[i].bf0_i2c_cfg->device_name, i2c_a3_obj[i].handle.Instance);
}

int rt_hw_i2c_init_lcpu_a3(void)
{
    int r = RT_EOK;

    memcpy(&lcpu_patch_ops, &i2c_ops_rom, sizeof(struct rt_i2c_bus_device_ops));
    lcpu_patch_ops.master_xfer = master_xfer_lcpu_patch_a3;
    //lcpu_patch_ops.slave_xfer =  ignore use ROM
    //lcpu_patch_ops.i2c_bus_control = i2c_bus_control_lcpu_patch_a3;
    lcpu_patch_ops.i2c_bus_configure = i2c_bus_configure_lcpu_patch_a3;


#ifdef BSP_I2C4_USING_DMA
    static struct dma_config i2c4_trx_dma = I2C4_TRX_DMA_CONFIG;
    i2c_a3_obj[I2C4_INDEX].i2c_dma_flag = 1;
    bf0_i2c_cfg[I2C4_INDEX].dma_rx = &i2c4_trx_dma;
    bf0_i2c_cfg[I2C4_INDEX].dma_tx = &i2c4_trx_dma;
#endif
#ifdef BSP_I2C5_USING_DMA
    static struct dma_config i2c5_trx_dma = I2C5_TRX_DMA_CONFIG;
    i2c_a3_obj[I2C5_INDEX].i2c_dma_flag = 1;
    bf0_i2c_cfg[I2C5_INDEX].dma_rx = &i2c5_trx_dma;
    bf0_i2c_cfg[I2C5_INDEX].dma_tx = &i2c5_trx_dma;
#endif
#ifdef BSP_I2C6_USING_DMA
    static struct dma_config i2c6_trx_dma = I2C6_TRX_DMA_CONFIG;
    i2c_a3_obj[I2C6_INDEX].i2c_dma_flag = 1;
    bf0_i2c_cfg[I2C6_INDEX].dma_rx = &i2c6_trx_dma;
    bf0_i2c_cfg[I2C6_INDEX].dma_tx = &i2c6_trx_dma;
#endif

#ifdef BSP_USING_I2C4
    register_i2c_device(I2C4_INDEX);
#endif

#ifdef BSP_USING_I2C5
    register_i2c_device(I2C5_INDEX);
#endif

#ifdef BSP_USING_I2C6
    register_i2c_device(I2C6_INDEX);
#endif

    return r;
}

INIT_BOARD_EXPORT(rt_hw_i2c_init_lcpu_a3);

#else //A0

#define I2C4_ROM_INDEX 0
#define I2C5_ROM_INDEX 1
#define I2C6_ROM_INDEX 2

#define I2C_NUM 3 //Fixed index in ROM:  I2C4-0,  I2C5-1, I2C6-2

/*Variables in ROM*/
extern bf0_i2c_t i2c_obj[]; //size 0x414
extern struct rt_i2c_configuration rt_i2c_cfg_default[];
extern bf0_i2c_config_t bf0_i2c_cfg[];


static uint32_t get_index_by_bus_handle(struct rt_i2c_bus_device *bus)
{
    for (int i = 0; i < I2C_NUM; i++)
    {
        if (bus == &i2c_obj[i].bus) return i;
    }

    return UINT32_MAX;
}

static void I2Cx_IRQHandler(uint32_t index)
{
    I2C_HandleTypeDef *handle;
    /* enter interrupt */
    rt_interrupt_enter();

    handle = &i2c_obj[index].handle;

    if (handle->XferISR != NULL)
    {
        handle->XferISR(handle, 0, 0);
    }


    /* leave interrupt */
    rt_interrupt_leave();
}

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
static void I2Cx_DMA_IRQHandler(uint32_t index)
{
    I2C_HandleTypeDef *handle;

    /* enter interrupt */
    rt_interrupt_enter();

    handle = &i2c_obj[index].handle;

    if (handle->State == HAL_I2C_STATE_BUSY_TX)
    {
        HAL_DMA_IRQHandler(handle->hdmatx);
    }
    else if (handle->State == HAL_I2C_STATE_BUSY_RX)
    {
        HAL_DMA_IRQHandler(handle->hdmarx);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

#if defined(BSP_USING_I2C4)
void I2C4_IRQHandler(void)
{
    I2Cx_IRQHandler(I2C4_ROM_INDEX);
}

#if defined(BSP_I2C4_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void I2C4_DMA_IRQHandler(void)
{
    I2Cx_DMA_IRQHandler(I2C4_ROM_INDEX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif

#if defined(BSP_USING_I2C5)
void I2C5_IRQHandler(void)
{
    I2Cx_IRQHandler(I2C5_ROM_INDEX);
}

#if defined(BSP_I2C5_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void I2C5_DMA_IRQHandler(void)
{
    I2Cx_DMA_IRQHandler(I2C5_ROM_INDEX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif

#if defined(BSP_USING_I2C6)
void I2C6_IRQHandler(void)
{
    I2Cx_IRQHandler(I2C6_ROM_INDEX);
}

#if defined(BSP_I2C6_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void I2C6_DMA_IRQHandler(void)
{
    I2Cx_DMA_IRQHandler(I2C6_ROM_INDEX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif


static rt_size_t master_xfer_mem_access_lcpu_patch_a0(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    rt_size_t ret = (0);

    rt_uint32_t index = 0;
    struct bf0_i2c *bf0_i2c = RT_NULL;
    struct rt_i2c_msg *msg = RT_NULL;
    HAL_StatusTypeDef status;
    uint16_t mem_addr_type;

    bf0_i2c = (struct bf0_i2c *)bus;
    LOG_I("master_xfer_mem_access_lcpu_patch_a0 %d, %x", get_index_by_bus_handle(bus), &bf0_i2c->handle);

    for (index = 0; index < num; index++)
    {
        msg = &msgs[index];
        if (msg->flags & RT_I2C_MEM_ACCESS)
        {
            if (8 >= msg->mem_addr_size)
            {
                mem_addr_type = I2C_MEMADD_SIZE_8BIT;
            }
            else
            {
                mem_addr_type = I2C_MEMADD_SIZE_16BIT;
            }
            if (msg->flags & RT_I2C_RD)
            {
                status = HAL_I2C_Mem_Read(&bf0_i2c->handle, msg->addr, msg->mem_addr, mem_addr_type, msg->buf, msg->len, bus->timeout);
            }
            else
            {
                status = HAL_I2C_Mem_Write(&bf0_i2c->handle, msg->addr, msg->mem_addr, mem_addr_type, msg->buf, msg->len, bus->timeout);
            }
        }


        while (status == HAL_OK)
        {
            if (HAL_I2C_GetState(&bf0_i2c->handle) == HAL_I2C_STATE_READY)
            {
                status = HAL_OK;
                break;
            }
            if (HAL_I2C_GetState(&bf0_i2c->handle) == HAL_I2C_STATE_ERROR)
                status = HAL_ERROR;
            if (HAL_I2C_GetState(&bf0_i2c->handle) == HAL_I2C_STATE_TIMEOUT)
                status = HAL_TIMEOUT;
        }
    }

    ret = index;

exit:
    return ret;
}

static rt_size_t master_xfer_lcpu_patch_a0(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    rt_size_t ret_v;
    struct bf0_i2c *bf0_i2c = (struct bf0_i2c *)bus;
    I2C_HandleTypeDef *hi2c = &bf0_i2c->handle;
    RT_ASSERT(bf0_i2c != RT_NULL);
    LOG_I("master_xfer_lcpu_patch_a0 %d, %x", get_index_by_bus_handle(bus), &bf0_i2c->handle);

    __HAL_I2C_ENABLE(&bf0_i2c->handle);

    for (rt_uint32_t i = 0; i < num; i++)
    {
        if ((msgs[i].flags & RT_I2C_RD) && (bus->parent.open_flag & RT_DEVICE_FLAG_DMA_RX))
        {
            mpu_dcache_invalidate(msgs[i].buf, msgs[i].len);
        }
    }

    if ((msgs[0].flags & RT_I2C_MEM_ACCESS) && (0 == (bus->parent.open_flag & (RT_DEVICE_FLAG_DMA_TX | RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_INT_RX))))
    {
        //Mem r/w CPU polling mode patch
        ret_v = master_xfer_mem_access_lcpu_patch_a0(bus, msgs, num);
    }
    else
    {
        ret_v = i2c_ops_rom.master_xfer(bus, msgs, num);
    }


    if (__HAL_I2C_GET_FLAG(hi2c, I2C_SR_UB) == SET) //Fix I2C STOP not finished in ROM
    {
        HAL_Delay_us(100); //STOP bit takes 100us at I2C 10KHz
    }

    if (ret_v != num) //Reset bus if error occurred
    {
        LOG_E("STATE %d, i2c bus reset and send 9 clks", HAL_I2C_GetState(&bf0_i2c->handle));
        HAL_I2C_Reset(&bf0_i2c->handle);
    }
    __HAL_I2C_DISABLE(&bf0_i2c->handle);
    return ret_v;
}

static rt_err_t i2c_bus_control_lcpu_patch_a0(struct rt_i2c_bus_device *bus, rt_uint32_t cmd, void *arg)
{
    rt_err_t ret_v;
    struct bf0_i2c *bf0_i2c = (struct bf0_i2c *)bus;

    LOG_I("i2c_bus_control_lcpu_patch_a0 %d, %x", get_index_by_bus_handle(bus), &bf0_i2c->handle);

    ret_v = i2c_ops_rom.i2c_bus_control(bus, cmd, arg);
    if (RT_DEVICE_CTRL_SUSPEND == cmd)
    {
        HAL_I2C_DeInit(&(bf0_i2c->handle));
    }

    return ret_v;
}

static rt_err_t i2c_bus_configure_lcpu_patch_a0(struct rt_i2c_bus_device *bus, struct rt_i2c_configuration *configuration)
{
    rt_err_t ret_v;
    struct bf0_i2c *bf0_i2c = (struct bf0_i2c *)bus;
    I2C_HandleTypeDef *hi2c = &bf0_i2c->handle;
    uint32_t i2c_idx = get_index_by_bus_handle(bus);
    LOG_I("i2c_bus_configure_lcpu_patch_a0 %d,%x", i2c_idx, &bf0_i2c->handle);

    if (!configuration) //Use default cfg
    {
        configuration = &rt_i2c_cfg_default[i2c_idx];
    }

    {
        //Reset I2C before call HAL_I2C_Init of ROM
        if (hi2c->Instance == hwp_i2c4)
            HAL_RCC_ResetModule(RCC_MOD_I2C4);
        else if (hi2c->Instance == hwp_i2c5)
            HAL_RCC_ResetModule(RCC_MOD_I2C5);
        else if (hi2c->Instance == hwp_i2c6)
            HAL_RCC_ResetModule(RCC_MOD_I2C6);
        else
            HAL_ASSERT(0);

        //Clear LCR.SLV to 0, to disbale send I2C 'bus reset' in ROM's HAL_I2C_Init
        hi2c->Instance->LCR = 0;
    }


    ret_v = i2c_ops_rom.i2c_bus_configure(bus, configuration);
    //ret_v = i2c_bus_configure(bus, configuration);

    //Fix bugs of HAL_I2C_Init in ROM
    {
        //bug1: I2C controller need reset it self after send bus reset
        hi2c->Instance->CR |= I2C_CR_UR;
        HAL_Delay_us(1);        // Delay at least 9 cycle.
        hi2c->Instance->CR &= ~I2C_CR_UR;

        //bug2: I2C enter slave mode while enable, enable it only in master_xfer
        __HAL_I2C_DISABLE(hi2c);


        //bug3: The clock divider of Fast mode & Standard mode are not correct
        uint32_t i2c_clk = HAL_RCC_GetPCLKFreq(hi2c->core, 1);

        uint32_t flv;
        /* Standard mode only use to send bus reset, use lowest frequence */
        uint32_t slv = I2C_LCR_SLV_Msk >> I2C_LCR_SLV_Pos;


        int div2 = 7 + hi2c->Init.ClockSpeed / 200000;  // verify on 100k, 200k and 400k
        flv = ((i2c_clk / hi2c->Init.ClockSpeed - hi2c->dnf - div2) / 2);

        HAL_ASSERT(flv <= (I2C_LCR_FLV_Msk >> I2C_LCR_FLV_Pos));
        MODIFY_REG(hi2c->Instance->LCR, I2C_LCR_FLV_Msk | I2C_LCR_SLV_Msk, (flv << I2C_LCR_FLV_Pos) | (slv << I2C_LCR_SLV_Pos));


        hi2c->Instance->IER = 0;

        //bug4: Fixed slave address conflict with customer's deivce
        hi2c->Instance->SAR = hi2c->Init.OwnAddress1;
    }

    memcpy(&rt_i2c_cfg_default[get_index_by_bus_handle(bus)],
           configuration, sizeof(struct rt_i2c_configuration));

    return ret_v;
}

static void register_i2c_device(uint8_t i)
{
    HAL_StatusTypeDef ret;
    rt_uint32_t flag = RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX;
    i2c_obj[i].bf0_i2c_cfg = &bf0_i2c_cfg[i];
    i2c_obj[i].bus.parent.user_data = &bf0_i2c_cfg[i];
    i2c_obj[i].bus.ops = &lcpu_patch_ops; //Replace with patch 'ops'
    i2c_obj[i].handle.Instance = bf0_i2c_cfg[i].Instance;
    //i2c_obj[i].handle.Init = i2c_init_default[i];

    if (i2c_obj[i].i2c_dma_flag)
    {
        flag |= (RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
        __HAL_LINKDMA(&i2c_obj[i].handle, hdmarx, i2c_obj[i].dma.dma_rx);
        __HAL_LINKDMA(&i2c_obj[i].handle, hdmatx, i2c_obj[i].dma.dma_tx);
        HAL_I2C_DMA_Init(&i2c_obj[i].handle, bf0_i2c_cfg[i].dma_rx, bf0_i2c_cfg[i].dma_tx);

        //HAL_NVIC_SetPriority(bf0_i2c_cfg[i].dma_rx->dma_irq, 0, 0);
        //HAL_NVIC_EnableIRQ(bf0_i2c_cfg[i].dma_rx->dma_irq);
    }

    rt_i2c_bus_device_register(&i2c_obj[i].bus, i2c_obj[i].bf0_i2c_cfg->device_name, flag);

    //ret = HAL_I2C_Init(&i2c_obj[i].handle);
    ret = lcpu_patch_ops.i2c_bus_configure(&i2c_obj[i].bus, &rt_i2c_cfg_default[i]);
    RT_ASSERT(HAL_OK == ret);

    LOG_I("%s bus init done, %x", i2c_obj[i].bf0_i2c_cfg->device_name, i2c_obj[i].handle.Instance);
}


int rt_hw_i2c_init_lcpu_a0(void)
{
    int r = RT_EOK;

    memcpy(&lcpu_patch_ops, &i2c_ops_rom, sizeof(struct rt_i2c_bus_device_ops));
    lcpu_patch_ops.master_xfer = master_xfer_lcpu_patch_a0;
    //lcpu_patch_ops.slave_xfer =  ignore use ROM
    lcpu_patch_ops.i2c_bus_control = i2c_bus_control_lcpu_patch_a0;
    lcpu_patch_ops.i2c_bus_configure = i2c_bus_configure_lcpu_patch_a0;


#ifdef BSP_I2C4_USING_DMA
    static struct dma_config i2c4_trx_dma = I2C4_TRX_DMA_CONFIG;
    i2c_obj[I2C4_ROM_INDEX].i2c_dma_flag = 1;
    bf0_i2c_cfg[I2C4_ROM_INDEX].dma_rx = &i2c4_trx_dma;
    bf0_i2c_cfg[I2C4_ROM_INDEX].dma_tx = &i2c4_trx_dma;
#endif
#ifdef BSP_I2C5_USING_DMA
    static struct dma_config i2c5_trx_dma = I2C5_TRX_DMA_CONFIG;
    i2c_obj[I2C5_ROM_INDEX].i2c_dma_flag = 1;
    bf0_i2c_cfg[I2C5_ROM_INDEX].dma_rx = &i2c5_trx_dma;
    bf0_i2c_cfg[I2C5_ROM_INDEX].dma_tx = &i2c5_trx_dma;
#endif
#ifdef BSP_I2C6_USING_DMA
    static struct dma_config i2c6_trx_dma = I2C6_TRX_DMA_CONFIG;
    i2c_obj[I2C6_ROM_INDEX].i2c_dma_flag = 1;
    bf0_i2c_cfg[I2C6_ROM_INDEX].dma_rx = &i2c6_trx_dma;
    bf0_i2c_cfg[I2C6_ROM_INDEX].dma_tx = &i2c6_trx_dma;
#endif

#ifdef BSP_USING_I2C4
    register_i2c_device(I2C4_ROM_INDEX);
#endif

#ifdef BSP_USING_I2C5
    register_i2c_device(I2C5_ROM_INDEX);
#endif

#ifdef BSP_USING_I2C6
    register_i2c_device(I2C6_ROM_INDEX);
#endif

    return r;
}

INIT_BOARD_EXPORT(rt_hw_i2c_init_lcpu_a0);

#endif


#else //HCPU || !SF32LB55X

#define I2C_NUM  (sizeof(bf0_i2c_cfg) / sizeof(bf0_i2c_cfg[0]))

static bf0_i2c_config_t bf0_i2c_cfg[] =
{
#ifdef BSP_USING_I2C1
    BF0_I2C1_CFG,
#endif

#ifdef BSP_USING_I2C2
    BF0_I2C2_CFG,
#endif

#ifdef BSP_USING_I2C3
    BF0_I2C3_CFG,
#endif

#ifdef BSP_USING_I2C4
    BF0_I2C4_CFG,
#endif

#ifdef BSP_USING_I2C5
    BF0_I2C5_CFG,
#endif

#ifdef BSP_USING_I2C6
    BF0_I2C6_CFG,
#endif

#ifdef BSP_USING_I2C7
    BF0_I2C7_CFG,
#endif

};

static struct rt_i2c_configuration rt_i2c_cfg_default[I2C_NUM] =
{
#ifdef BSP_USING_I2C1
    {
        0,       //rt_uint16_t mode;  RT_I2C_ADDR_10BIT / RT_I2C_NO_START / RT_I2C_IGNORE_NACK / RT_I2C_NO_READ_ACK
        0,       //rt_uint16_t addr;
        5000,    //rt_uint32_t timeout;
        400000   //rt_uint32_t max_hz;
    },
#endif
#ifdef BSP_USING_I2C2
    {
        0,       //rt_uint16_t mode;  RT_I2C_ADDR_10BIT / RT_I2C_NO_START / RT_I2C_IGNORE_NACK / RT_I2C_NO_READ_ACK
        0,       //rt_uint16_t addr;
        5000,    //rt_uint32_t timeout;
        400000   //rt_uint32_t max_hz;
    },
#endif
#ifdef BSP_USING_I2C3
    {
        0,       //rt_uint16_t mode;  RT_I2C_ADDR_10BIT / RT_I2C_NO_START / RT_I2C_IGNORE_NACK / RT_I2C_NO_READ_ACK
        0,       //rt_uint16_t addr;
        5000,    //rt_uint32_t timeout;
        400000   //rt_uint32_t max_hz;
    },
#endif
#ifdef BSP_USING_I2C4
    {
        0,       //rt_uint16_t mode;  RT_I2C_ADDR_10BIT / RT_I2C_NO_START / RT_I2C_IGNORE_NACK / RT_I2C_NO_READ_ACK
        0,       //rt_uint16_t addr;
        5000,    //rt_uint32_t timeout;
        400000   //rt_uint32_t max_hz;
    },
#endif
#ifdef BSP_USING_I2C5
    {
        0,       //rt_uint16_t mode;  RT_I2C_ADDR_10BIT / RT_I2C_NO_START / RT_I2C_IGNORE_NACK / RT_I2C_NO_READ_ACK
        0,       //rt_uint16_t addr;
        5000,    //rt_uint32_t timeout;
        400000   //rt_uint32_t max_hz;
    },
#endif
#ifdef BSP_USING_I2C6
    {
        0,       //rt_uint16_t mode;  RT_I2C_ADDR_10BIT / RT_I2C_NO_START / RT_I2C_IGNORE_NACK / RT_I2C_NO_READ_ACK
        0,       //rt_uint16_t addr;
        5000,    //rt_uint32_t timeout;
        400000   //rt_uint32_t max_hz;
    },
#endif
#ifdef BSP_USING_I2C7
    {
        0,       //rt_uint16_t mode;  RT_I2C_ADDR_10BIT / RT_I2C_NO_START / RT_I2C_IGNORE_NACK / RT_I2C_NO_READ_ACK
        0,       //rt_uint16_t addr;
        5000,    //rt_uint32_t timeout;
        400000   //rt_uint32_t max_hz;
    },
#endif

};


static bf0_i2c_t i2c_obj[I2C_NUM];
static rt_sem_t i2c_sema[I2C_NUM];

static uint32_t get_index_by_bus_handle(struct rt_i2c_bus_device *bus)
{
    for (int i = 0; i < I2C_NUM; i++)
    {
        if (bus == &i2c_obj[i].bus) return i;
    }

    return UINT32_MAX;
}

static rt_size_t master_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    rt_size_t ret = (0);

    rt_uint32_t index = 0;
    rt_uint32_t i2c_index;
    struct bf0_i2c *bf0_i2c = RT_NULL;
    struct rt_i2c_msg *msg = RT_NULL;
    HAL_StatusTypeDef status;
    uint16_t mem_addr_type;
    rt_err_t rt_err_v;

    RT_ASSERT(bus != RT_NULL);

    bf0_i2c = (struct bf0_i2c *)bus;

    i2c_index = get_index_by_bus_handle(bus);
    LOG_I("master_xfer start");
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_start();
#endif  /* RT_USING_PM */
    __HAL_I2C_ENABLE(&bf0_i2c->handle);
    for (index = 0; index < num; index++)
    {
        rt_err_v = rt_sem_control(i2c_sema[i2c_index], RT_IPC_CMD_RESET, 0);
        RT_ASSERT(RT_EOK == rt_err_v);

        msg = &msgs[index];
        if (msg->flags & RT_I2C_MEM_ACCESS)
        {
            if (8 >= msg->mem_addr_size)
            {
                mem_addr_type = I2C_MEMADD_SIZE_8BIT;
            }
            else
            {
                mem_addr_type = I2C_MEMADD_SIZE_16BIT;
            }
            if (msg->flags & RT_I2C_RD)
            {
                if (bus->parent.open_flag & RT_DEVICE_FLAG_DMA_RX)
                {
                    HAL_DMA_Init(&bf0_i2c->dma.dma_rx);
                    mpu_dcache_invalidate(msg->buf, msg->len);
                    status = HAL_I2C_Mem_Read_DMA(&bf0_i2c->handle, msg->addr, msg->mem_addr, mem_addr_type, msg->buf, msg->len);
                }
                else if (bus->parent.open_flag & RT_DEVICE_FLAG_INT_RX)
                {
                    status = HAL_I2C_Mem_Read_IT(&bf0_i2c->handle, msg->addr, msg->mem_addr, mem_addr_type, msg->buf, msg->len);
                }
                //else if (msg->flags & RT_I2C_NORMAL_MODE)
                else
                {
                    status = HAL_I2C_Mem_Read(&bf0_i2c->handle, msg->addr, msg->mem_addr, mem_addr_type, msg->buf, msg->len, bus->timeout);
                }
            }
            else
            {
                if (bus->parent.open_flag & RT_DEVICE_FLAG_DMA_TX)
                {
                    HAL_DMA_Init(&bf0_i2c->dma.dma_tx);
                    status = HAL_I2C_Mem_Write_DMA(&bf0_i2c->handle, msg->addr, msg->mem_addr, mem_addr_type, msg->buf, msg->len);
                }
                else if (bus->parent.open_flag & RT_DEVICE_FLAG_INT_TX)
                {
                    status = HAL_I2C_Mem_Write_IT(&bf0_i2c->handle, msg->addr, msg->mem_addr, mem_addr_type, msg->buf, msg->len);
                }
                //else if (msg->flags & RT_I2C_NORMAL_MODE)
                else
                {
                    status = HAL_I2C_Mem_Write(&bf0_i2c->handle, msg->addr, msg->mem_addr, mem_addr_type, msg->buf, msg->len, bus->timeout);
                }

            }
        }
        else
        {
            if (msg->flags & RT_I2C_RD)
            {
                if (bus->parent.open_flag & RT_DEVICE_FLAG_DMA_RX)
                {
                    HAL_DMA_Init(&bf0_i2c->dma.dma_rx);
                    mpu_dcache_invalidate(msg->buf, msg->len);
                    status = HAL_I2C_Master_Receive_DMA(&bf0_i2c->handle, msg->addr, msg->buf, msg->len);
                }
                else if (bus->parent.open_flag & RT_DEVICE_FLAG_INT_RX)
                {
                    status = HAL_I2C_Master_Receive_IT(&bf0_i2c->handle, msg->addr, msg->buf, msg->len);
                }
                //else if (msg->flags & RT_I2C_NORMAL_MODE)
                else
                {
                    status = HAL_I2C_Master_Receive(&bf0_i2c->handle, msg->addr, msg->buf, msg->len, bus->timeout);
                }
            }
            else
            {
                if (bus->parent.open_flag & RT_DEVICE_FLAG_DMA_TX)
                {
                    HAL_DMA_Init(&bf0_i2c->dma.dma_tx);
                    status = HAL_I2C_Master_Transmit_DMA(&bf0_i2c->handle, msg->addr, msg->buf, msg->len);
                }
                else if (bus->parent.open_flag & RT_DEVICE_FLAG_INT_TX)
                {
                    status = HAL_I2C_Master_Transmit_IT(&bf0_i2c->handle, msg->addr, msg->buf, msg->len);
                }
                //else if (msg->flags & RT_I2C_NORMAL_MODE)
                else
                {
                    status = HAL_I2C_Master_Transmit(&bf0_i2c->handle, msg->addr, msg->buf, msg->len, bus->timeout);
                }
            }
        }

        if (HAL_OK != status) goto exit;

        while (1)
        {
            HAL_I2C_StateTypeDef i2c_state = HAL_I2C_GetState(&bf0_i2c->handle);

            if (HAL_I2C_STATE_READY == i2c_state)
            {
                status = HAL_OK;
            }
            else if (HAL_I2C_STATE_TIMEOUT == i2c_state)
            {
                status = HAL_TIMEOUT;
            }
            else if ((HAL_I2C_STATE_BUSY_TX == i2c_state) || (HAL_I2C_STATE_BUSY_RX == i2c_state)) //Interrupt or DMA mode, wait semaphore
            {
                rt_err_v = rt_sem_take(i2c_sema[i2c_index], bus->timeout);
                if (-RT_ETIMEOUT == rt_err_v)
                {
                    LOG_E("i2c sem timeout!");
                    status = HAL_TIMEOUT;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                status = HAL_ERROR;
            }

            break;
        }

        if (bf0_i2c->handle.ErrorCode) goto exit;

        if (HAL_OK != status) goto exit;

#if 0
        //Wordaround: I2C Wait TE fail in 'I2C_MasterRequestWrite' after ISR mode read(After 58x)  - Fixed
        bf0_i2c->handle.Instance->CR |= I2C_CR_UR;
        HAL_Delay_us(1);        // Delay at least 9 cycle.
        bf0_i2c->handle.Instance->CR &= ~I2C_CR_UR;
#endif

    }

    ret = index;

exit:
    if ((ret != num) || (HAL_OK != status))
    {
        LOG_E("bus err:%d, xfer:%d/%d, i2c_stat:%x, i2c_errcode=%x", status, index, num, HAL_I2C_GetState(&bf0_i2c->handle), bf0_i2c->handle.ErrorCode);

        HAL_I2C_Reset(&bf0_i2c->handle);
        LOG_E("reset and send 9 clks");
    }
    __HAL_I2C_DISABLE(&bf0_i2c->handle);
#ifdef RT_USING_PM
    rt_pm_hw_device_stop();
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif  /* RT_USING_PM */

    LOG_I("master_xfer end");

    return ret;
}

static rt_size_t slave_xfer(struct rt_i2c_bus_device *bus, struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    return 0;
}

static rt_err_t i2c_bus_control(struct rt_i2c_bus_device *bus, rt_uint32_t cmd, void *arg)
{
    RT_ASSERT(bus != RT_NULL);
    //RT_ASSERT(arg != RT_NULL);

    struct bf0_i2c *bf0_i2c = (struct bf0_i2c *)bus;
    LOG_I("i2c_bus_control start  cmd =%x arg=%x", cmd, (rt_uint32_t)arg);

    switch (cmd)
    {
    /* disable interrupt */
    case RT_DEVICE_CTRL_CLR_INT:
        if ((((rt_uint32_t)arg) & RT_DEVICE_FLAG_INT_RX) || (((rt_uint32_t)arg) & RT_DEVICE_FLAG_INT_TX))
        {
            NVIC_DisableIRQ(bf0_i2c->bf0_i2c_cfg->irq_type);
            //I2C_Disable_IRQ(bf0_i2c->handle, I2C_CTR_IEN);
        }

        if ((((rt_uint32_t)arg) & RT_DEVICE_FLAG_DMA_RX) || (((rt_uint32_t)arg) & RT_DEVICE_FLAG_DMA_TX))
        {
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            NVIC_DisableIRQ(bf0_i2c->bf0_i2c_cfg->dma_rx->dma_irq);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            NVIC_DisableIRQ(bf0_i2c->bf0_i2c_cfg->irq_type);
            //I2C_Disable_IRQ(bf0_i2c->handle, I2C_CTR_DIEN);
        }

        break;

    /* enable interrupt */
    case RT_DEVICE_CTRL_SET_INT:
        if ((((rt_uint32_t)arg) & RT_DEVICE_FLAG_INT_RX) || (((rt_uint32_t)arg) & RT_DEVICE_FLAG_INT_TX))
        {
            HAL_NVIC_SetPriority(bf0_i2c->bf0_i2c_cfg->irq_type, 1, 0);
            NVIC_EnableIRQ(bf0_i2c->bf0_i2c_cfg->irq_type);
            //I2C_Enable_IRQ(bf0_i2c->handle, I2C_CTR_IEN);
        }

        if ((((rt_uint32_t)arg) & RT_DEVICE_FLAG_DMA_RX) || (((rt_uint32_t)arg) & RT_DEVICE_FLAG_DMA_TX))
        {
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_SetPriority(bf0_i2c->bf0_i2c_cfg->dma_rx->dma_irq, 1, 0);
            NVIC_EnableIRQ(bf0_i2c->bf0_i2c_cfg->dma_rx->dma_irq);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            HAL_NVIC_SetPriority(bf0_i2c->bf0_i2c_cfg->irq_type, 1, 0);
            NVIC_EnableIRQ(bf0_i2c->bf0_i2c_cfg->irq_type);
        }

        break;

    case RT_DEVICE_CTRL_CONFIG:
        //todo:

        break;
    case RT_DEVICE_CTRL_SUSPEND:
        HAL_I2C_DeInit(&(bf0_i2c->handle));
        break;
    }

    LOG_I("i2c_bus_control end");

    return RT_EOK;
}

static rt_err_t i2c_bus_configure(struct rt_i2c_bus_device *bus, struct rt_i2c_configuration *configuration)
{
    rt_err_t ret;

    LOG_I("i2c_bus_configure start");

    RT_ASSERT(bus != RT_NULL);

    struct bf0_i2c *bf0_i2c = (struct bf0_i2c *)bus;
    uint32_t i2c_idx = get_index_by_bus_handle(bus);
    if (!configuration) //Use default cfg
    {
        configuration = &rt_i2c_cfg_default[i2c_idx];
    }

    bus->timeout = configuration->timeout;
    //bus->flags = configuration->mode;

    if (configuration->mode & RT_I2C_ADDR_10BIT)
    {
        bf0_i2c->handle.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
        bf0_i2c->handle.Init.OwnAddress1 = (configuration->addr & 0x7fff);
    }
    else
    {
        bf0_i2c->handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        bf0_i2c->handle.Init.OwnAddress1 = (configuration->addr & 0x7fff) << 1;
    }

    bf0_i2c->handle.Init.ClockSpeed = configuration->max_hz;
    bf0_i2c->handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    bf0_i2c->handle.core = bf0_i2c->bf0_i2c_cfg->core;
    bf0_i2c->handle.Mode = HAL_I2C_MODE_MASTER;

    memcpy(&rt_i2c_cfg_default[get_index_by_bus_handle(bus)],
           configuration, sizeof(struct rt_i2c_configuration));
    if (bf0_i2c->handle.Instance == hwp_i2c1)
        HAL_RCC_EnableModule(RCC_MOD_I2C1);
    else if (bf0_i2c->handle.Instance == hwp_i2c2)
        HAL_RCC_EnableModule(RCC_MOD_I2C2);
    else if (bf0_i2c->handle.Instance == hwp_i2c3)
        HAL_RCC_EnableModule(RCC_MOD_I2C3);
    else if (bf0_i2c->handle.Instance == hwp_i2c4)
        HAL_RCC_EnableModule(RCC_MOD_I2C4);
#ifdef BSP_USING_I2C5
    else if (bf0_i2c->handle.Instance == hwp_i2c5)
        HAL_RCC_EnableModule(RCC_MOD_I2C5);
#endif /* BSP_USING_I2C5 */
#ifdef BSP_USING_I2C6
    else if (bf0_i2c->handle.Instance == hwp_i2c6)
        HAL_RCC_EnableModule(RCC_MOD_I2C6);
#endif /* BSP_USING_I2C6  */
#ifdef BSP_USING_I2C7
    else if (bf0_i2c->handle.Instance == hwp_i2c7)
        HAL_RCC_EnableModule(RCC_MOD_I2C7);
#endif /* BSP_USING_I2C7 */
    else
        RT_ASSERT(0);

    ret = HAL_I2C_Init(&(bf0_i2c->handle));

    LOG_I("i2c_bus_configure end");

    return ret;
}


static const struct rt_i2c_bus_device_ops ops =
{
    .master_xfer = master_xfer,
    .slave_xfer = slave_xfer,
    .i2c_bus_control = i2c_bus_control,
    .i2c_bus_configure = i2c_bus_configure,
};


static void I2Cx_IRQHandler(uint32_t index)
{
    I2C_HandleTypeDef *handle;
    /* enter interrupt */
    rt_interrupt_enter();

    handle = &i2c_obj[index].handle;

    if (handle->XferISR != NULL)
    {
        handle->XferISR(handle, 0, 0);
    }

    if ((HAL_I2C_STATE_BUSY_TX != handle->State) && (HAL_I2C_STATE_BUSY_RX != handle->State))
    {
        rt_sem_release(i2c_sema[index]);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}

static void I2Cx_DMA_IRQHandler(uint32_t index)
{
    I2C_HandleTypeDef *handle;

    /* enter interrupt */
    rt_interrupt_enter();

    handle = &i2c_obj[index].handle;

    if (handle->State == HAL_I2C_STATE_BUSY_TX)
    {
        HAL_DMA_IRQHandler(handle->hdmatx);
    }
    else if (handle->State == HAL_I2C_STATE_BUSY_RX)
    {
        HAL_DMA_IRQHandler(handle->hdmarx);
    }
    else
    {
        /*The I2C_handle->State is READY before this DMA IRQ sometimes.*/
        if (handle->hdmatx != NULL)
            if (HAL_DMA_STATE_BUSY == handle->hdmatx->State)
                HAL_DMA_IRQHandler(handle->hdmatx);

        if (handle->hdmarx != NULL)
            if (HAL_DMA_STATE_BUSY == handle->hdmarx->State)
                HAL_DMA_IRQHandler(handle->hdmarx);
    }


    /* leave interrupt */
    rt_interrupt_leave();
}


#if defined(BSP_USING_I2C1)
void I2C1_IRQHandler(void)
{
    I2Cx_IRQHandler(I2C1_INDEX);
}

#if defined(BSP_I2C1_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void I2C1_DMA_IRQHandler(void)
{
    I2Cx_DMA_IRQHandler(I2C1_INDEX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif

#if defined(BSP_USING_I2C2)
void I2C2_IRQHandler(void)
{
    I2Cx_IRQHandler(I2C2_INDEX);
}

#if defined(BSP_I2C2_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void I2C2_DMA_IRQHandler(void)
{
    I2Cx_DMA_IRQHandler(I2C2_INDEX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif

#if defined(BSP_USING_I2C3)
void I2C3_IRQHandler(void)
{
    I2Cx_IRQHandler(I2C3_INDEX);
}

#if defined(BSP_I2C3_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void I2C3_DMA_IRQHandler(void)
{
    I2Cx_DMA_IRQHandler(I2C3_INDEX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif




#if defined(BSP_USING_I2C4)
void I2C4_IRQHandler(void)
{
    I2Cx_IRQHandler(I2C4_INDEX);
}

#if defined(BSP_I2C4_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void I2C4_DMA_IRQHandler(void)
{
    I2Cx_DMA_IRQHandler(I2C4_INDEX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif

#if defined(BSP_USING_I2C5)
void I2C5_IRQHandler(void)
{
    I2Cx_IRQHandler(I2C5_INDEX);
}

#if defined(BSP_I2C5_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void I2C5_DMA_IRQHandler(void)
{
    I2Cx_DMA_IRQHandler(I2C5_INDEX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif

#if defined(BSP_USING_I2C6)
void I2C6_IRQHandler(void)
{
    I2Cx_IRQHandler(I2C6_INDEX);
}

#if defined(BSP_I2C6_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void I2C6_DMA_IRQHandler(void)
{
    I2Cx_DMA_IRQHandler(I2C6_INDEX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif

#if defined(BSP_USING_I2C7)
void I2C7_IRQHandler(void)
{
    I2Cx_IRQHandler(I2C7_INDEX);
}

#if defined(BSP_I2C7_USING_DMA) && !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void I2C7_DMA_IRQHandler(void)
{
    I2Cx_DMA_IRQHandler(I2C7_INDEX);
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif

static void i2c_get_dma_info(void)
{
#ifdef BSP_I2C1_USING_DMA
    static struct dma_config i2c1_trx_dma = I2C1_TRX_DMA_CONFIG;
    i2c_obj[I2C1_INDEX].i2c_dma_flag = 1;
    bf0_i2c_cfg[I2C1_INDEX].dma_rx = &i2c1_trx_dma;
    bf0_i2c_cfg[I2C1_INDEX].dma_tx = &i2c1_trx_dma;
#endif
#ifdef BSP_I2C2_USING_DMA
    static struct dma_config i2c2_trx_dma = I2C2_TRX_DMA_CONFIG;
    i2c_obj[I2C2_INDEX].i2c_dma_flag = 1;
    bf0_i2c_cfg[I2C2_INDEX].dma_rx = &i2c2_trx_dma;
    bf0_i2c_cfg[I2C2_INDEX].dma_tx = &i2c2_trx_dma;
#endif
#ifdef BSP_I2C3_USING_DMA
    static struct dma_config i2c3_trx_dma = I2C3_TRX_DMA_CONFIG;
    i2c_obj[I2C3_INDEX].i2c_dma_flag = 1;
    bf0_i2c_cfg[I2C3_INDEX].dma_rx = &i2c3_trx_dma;
    bf0_i2c_cfg[I2C3_INDEX].dma_tx = &i2c3_trx_dma;
#endif
#ifdef BSP_I2C4_USING_DMA
    static struct dma_config i2c4_trx_dma = I2C4_TRX_DMA_CONFIG;
    i2c_obj[I2C4_INDEX].i2c_dma_flag = 1;
    bf0_i2c_cfg[I2C4_INDEX].dma_rx = &i2c4_trx_dma;
    bf0_i2c_cfg[I2C4_INDEX].dma_tx = &i2c4_trx_dma;
#endif
#ifdef BSP_I2C5_USING_DMA
    static struct dma_config i2c5_trx_dma = I2C5_TRX_DMA_CONFIG;
    i2c_obj[I2C5_INDEX].i2c_dma_flag = 1;
    bf0_i2c_cfg[I2C5_INDEX].dma_rx = &i2c5_trx_dma;
    bf0_i2c_cfg[I2C5_INDEX].dma_tx = &i2c5_trx_dma;
#endif
#ifdef BSP_I2C6_USING_DMA
    static struct dma_config i2c6_trx_dma = I2C6_TRX_DMA_CONFIG;
    i2c_obj[I2C6_INDEX].i2c_dma_flag = 1;
    bf0_i2c_cfg[I2C6_INDEX].dma_rx = &i2c6_trx_dma;
    bf0_i2c_cfg[I2C6_INDEX].dma_tx = &i2c6_trx_dma;
#endif
#ifdef BSP_I2C7_USING_DMA
    static struct dma_config i2c7_trx_dma = I2C7_TRX_DMA_CONFIG;
    i2c_obj[I2C7_INDEX].i2c_dma_flag = 1;
    bf0_i2c_cfg[I2C7_INDEX].dma_rx = &i2c7_trx_dma;
    bf0_i2c_cfg[I2C7_INDEX].dma_tx = &i2c7_trx_dma;
#endif
}

__ROM_USED int rt_hw_i2c_init2(bf0_i2c_t *objs, bf0_i2c_config_t *cfg, struct rt_i2c_configuration *cfg_default, rt_size_t obj_num)
{
    HAL_StatusTypeDef ret;
    rt_uint32_t flag = RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX;

    //rt_err_t result;
    for (int i = 0; i < obj_num; i++)
    {
        objs[i].bf0_i2c_cfg = &cfg[i];
        objs[i].bus.parent.user_data = &cfg[i];
        objs[i].bus.ops = &ops;
        objs[i].handle.Instance = cfg[i].Instance;
        //i2c_obj[i].handle.Init = i2c_init_default[i];

        if (objs[i].i2c_dma_flag)
        {
            flag |= (RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
            __HAL_LINKDMA(&objs[i].handle, hdmarx, objs[i].dma.dma_rx);
            __HAL_LINKDMA(&objs[i].handle, hdmatx, objs[i].dma.dma_tx);
            HAL_I2C_DMA_Init(&objs[i].handle, cfg[i].dma_rx, cfg[i].dma_tx);

            //HAL_NVIC_SetPriority(cfg[i].dma_rx->dma_irq, 0, 0); //Enable irq in i2c_bus_control
            //HAL_NVIC_EnableIRQ(cfg[i].dma_rx->dma_irq);
            //HAL_NVIC_SetPriority(bf0_i2c_cfg[i].dma_tx->dma_irq, 0, 0);
            //HAL_NVIC_EnableIRQ(bf0_i2c_cfg[i].dma_tx->dma_irq);
        }

        //HAL_NVIC_SetPriority(i2c_obj[i].bf0_i2c_cfg->irq_type, 0, 0);
        //HAL_NVIC_EnableIRQ(i2c_obj[i].bf0_i2c_cfg->irq_type);

        rt_i2c_bus_device_register(&objs[i].bus, objs[i].bf0_i2c_cfg->device_name, flag);

        //ret = HAL_I2C_Init(&i2c_obj[i].handle);
        ret = i2c_bus_configure(&objs[i].bus, &cfg_default[i]);
        RT_ASSERT(HAL_OK == ret);

        i2c_sema[i] = rt_sem_create(objs[i].bf0_i2c_cfg->device_name, 0, RT_IPC_FLAG_FIFO);
        RT_ASSERT(i2c_sema[i]);

        //LOG_D("%s bus init done", i2c_obj[i].bf0_i2c_cfg->device_name);
    }
    return RT_EOK;
}

int rt_hw_i2c_init(void)
{
    rt_size_t obj_num = I2C_NUM;
    int r = RT_EOK;

    if (obj_num > 0)
    {
        i2c_get_dma_info();
        r = rt_hw_i2c_init2(i2c_obj, bf0_i2c_cfg, rt_i2c_cfg_default, obj_num);
    }
    return r;
}
INIT_BOARD_EXPORT(rt_hw_i2c_init);

#endif /*defined(SOC_SF32LB55X)&&defined(SOC_BF0_LCPU)*/









//#define DRV_TEST

#ifdef DRV_TEST

#define  SMPLRT_DIV    0x19  //gyro sample rate, e.g. 0x07 (125Hz)
#define  CONFIG        0x1A  //lowpass filter freq, e.g. 0x06(5Hz)
#define  GYRO_CONFIG   0x1B  //lowpass filter self-test and measurement range, e.g. 0x18(no self-test, 2000deg/s)
#define  ACCEL_CONFIG  0x1C  //accel self-test, measurement range and highpass filter freq, e.g. 0x01(no self-test, 2g, 5Hz)
#define  ACCEL_XOUT_H  0x3B
#define  ACCEL_XOUT_L  0x3C
#define  ACCEL_YOUT_H  0x3D
#define  ACCEL_YOUT_L  0x3E
#define  ACCEL_ZOUT_H  0x3F
#define  ACCEL_ZOUT_L  0x40
#define  TEMP_OUT_H    0x41
#define  TEMP_OUT_L    0x42
#define  GYRO_XOUT_H   0x43
#define  GYRO_XOUT_L   0x44
#define  GYRO_YOUT_H   0x45
#define  GYRO_YOUT_L   0x46
#define  GYRO_ZOUT_H   0x47
#define  GYRO_ZOUT_L   0x48
#define  PWR_MGMT_1    0x6B  //power management, e.g. 0x00(normal)
#define  WHO_AM_I      0x75  //I2C address register, default: 0x68, readonly
#define  MPU6050_DEV_ADDR (0x68)

bf0_i2c_t i2c1;

int cmd_i2c(int argc, char *argv[])
{
    i2c1 = i2c_obj[0];

    if (argc > 1)
    {
        if (strcmp(argv[1], "init") == 0)
        {
            uint8_t data;
            rt_size_t len;
            data = 0x00;
            len = rt_i2c_mem_write(&i2c1.bus, MPU6050_DEV_ADDR, PWR_MGMT_1, 8, &data, 1);
            RT_ASSERT(1 == len);
            data = 0x07;
            len = rt_i2c_mem_write(&i2c1.bus, MPU6050_DEV_ADDR, SMPLRT_DIV, 8, &data, 1);
            RT_ASSERT(1 == len);
            data = 0x00;
            len = rt_i2c_mem_write(&i2c1.bus, MPU6050_DEV_ADDR, CONFIG, 8, &data, 1);
            RT_ASSERT(1 == len);
            data = 0x00;
            len = rt_i2c_mem_write(&i2c1.bus, MPU6050_DEV_ADDR, GYRO_CONFIG, 8, &data, 1);
            RT_ASSERT(1 == len);
            data = 0x01;
            len = rt_i2c_mem_write(&i2c1.bus, MPU6050_DEV_ADDR, ACCEL_CONFIG, 8, &data, 1);
            RT_ASSERT(1 == len);
        }
        else if (strcmp(argv[1], "read_acc") == 0)
        {
            uint16_t data;
            rt_size_t len;

            len = rt_i2c_mem_read(&i2c1.bus, MPU6050_DEV_ADDR, ACCEL_XOUT_H, 8, &data, sizeof(data));
            data = (data >> 8) | ((data & 0xFF) << 8);
            RT_ASSERT(sizeof(data) == len);
            LOG_I("ACC_X:%d", (int16_t)data);
            len = rt_i2c_mem_read(&i2c1.bus, MPU6050_DEV_ADDR, ACCEL_YOUT_H, 8, &data, sizeof(data));
            RT_ASSERT(sizeof(data) == len);
            data = (data >> 8) | ((data & 0xFF) << 8);
            LOG_I("ACC_Y:%d", (int16_t)data);
            len = rt_i2c_mem_read(&i2c1.bus, MPU6050_DEV_ADDR, ACCEL_ZOUT_H, 8, &data, sizeof(data));
            RT_ASSERT(sizeof(data) == len);
            data = (data >> 8) | ((data & 0xFF) << 8);
            LOG_I("ACC_Z:%d", (int16_t)data);
        }
        else if (strcmp(argv[1], "write") == 0)
        {
            uint8_t data;
            uint8_t addr;
            rt_size_t len;

            if (argc > 3)
            {
                addr = atoi(argv[2]);
                data = atoi(argv[3]);
                len = rt_i2c_mem_write(&i2c1.bus, MPU6050_DEV_ADDR, addr, 8, &data, sizeof(data));
                RT_ASSERT(sizeof(data) == len);
            }
            else
            {
                LOG_E("invalid data");
            }
        }
        else if (strcmp(argv[1], "read") == 0)
        {
            uint8_t data;
            uint8_t addr;
            rt_size_t len;

            if (argc > 2)
            {
                addr = atoi(argv[2]);
                len = rt_i2c_mem_read(&i2c1.bus, MPU6050_DEV_ADDR, addr, 8, &data, sizeof(data));
                RT_ASSERT(sizeof(data) == len);
                LOG_I("[0x%x]: %d", addr, data);
            }
            else
            {
                LOG_E("invalid data");
            }
        }
        else if (strcmp(argv[1], "read1") == 0)
        {
            uint8_t data;
            uint8_t addr;
            rt_size_t len;
            struct rt_i2c_msg msgs[2];

            if (argc > 2)
            {
                addr = atoi(argv[2]);

                msgs[0].addr     = MPU6050_DEV_ADDR;
                msgs[0].flags    = RT_I2C_WR;
                msgs[0].buf      = &addr;
                msgs[0].len      =  1;

                msgs[1].addr     = MPU6050_DEV_ADDR;
                msgs[1].flags    = RT_I2C_RD;
                msgs[1].buf      = &data;
                msgs[1].len      = 1;

                len = rt_i2c_transfer(&i2c1.bus, msgs, 2);
                RT_ASSERT(2 == len);
                LOG_I("[0x%x]: %d", addr, data);
            }
            else
            {
                LOG_I("invalid data");
            }
        }
        else
        {
            LOG_I("unknown arg1:%s", argv[1]);
        }
    }
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_i2c, __cmd_i2c, Test i2c sensor);

#endif

/// @} drv_i2c
/// @} bsp_driver

/// @} file

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
