/**
  ******************************************************************************
  * @file   pmic_controller.c
  * @author Sifli software development team
  * @brief   This file includes the BMP280 driver functions
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

#include "pmic_controller.h"
#include <rtconfig.h>
#include <math.h>
#include "stdlib.h"
#include "board.h"
#include "bf0_hal_hlp.h"
#if defined(PMIC_CONTROL_SERVICE)
    #include "pmic_service.h"
#endif

//#define DRV_PMIC_TEST

#ifdef PMIC_CTRL_ENABLE
//read only
#define REG_05_BUCK_UVLO_OUT_POS        (1) //buck power ok
#define REG_05_BUCK_PG_OUT_POS          (2) //buck power low
#define REG_05_BUCK_UVLO_OUT_MASK       (1<<REG_05_BUCK_UVLO_OUT_POS)
#define REG_05_BUCK_PG_OUT_MASK         (1<<REG_05_BUCK_PG_OUT_POS)

//rw LDO4
#define REG_06_LDO28_SET_VOUT_POS        (3)
#define REG_06_LDO28_SET_VOUT_MASK       (0x0F<<REG_06_LDO28_SET_VOUT_POS)

//rw LDO3
#define REG_07_LDO30_SET_VOUT_POS        (3)
#define REG_07_LDO30_SEL_POS             (7)
#define REG_07_LDO30_SET_VOUT_MASK       (0x0F<<REG_07_LDO30_SET_VOUT_POS)
#define REG_07_LDO30_SEL_MASK            (1<<REG_07_LDO30_SEL_POS)

//rw LDO2
#define REG_08_LDO33_SET_VOUT_POS        (3)
#define REG_08_LDO33_SET_VOUT_MASK       (0x0F<<REG_07_LDO30_SET_VOUT_POS)

//rw LDO1
#define REG_09_LDO33_SET_VOUT_POS        (3)
#define REG_09_LDO33_SET_VOUT_MASK       (0x0F<<REG_07_LDO30_SET_VOUT_POS)

//read only
#define REG_0A_LDO2_POWER_OK_MASK       (1)
#define REG_0A_LDO3_POWER_OK_MASK       (2)
#define REG_0A_LDO4_POWER_OK_MASK       (4)
#define REG_0A_LDO1_POWER_OK_MASK       (8)

//wr
#define REG_0C_LVSW_SEL_MASK    (1<<0)
#define REG_0C_LVSW1_EN_MASK    (1<<1)
#define REG_0C_LVSW2_EN_MASK    (1<<2)
#define REG_0C_LVSW3_EN_MASK    (1<<3)
#define REG_0C_LVSW4_EN_MASK    (1<<4)
#define REG_0C_LVSW5_EN_MASK    (1<<5)
#define REG_0C_HVSW1_EN_MASK    (1<<6)
#define REG_0C_HVSW2_EN_MASK    (1<<7)
//wr
#define REG_0D_SDAT_PE_MASK     (1<<0)
#define REG_0D_SDAT_DS_MASK     (1<<2)
#define REG_0D_SCLK_PE_MASK     (1<<3)
#define REG_0D_SCLK_DS_MASK     (1<<5)


typedef struct
{
    int pmic_half_period_us;
#if 0//def SF32LB55X
    __IO uint32_t *scl_dor;
    uint32_t  scl_mask;
    __IO uint32_t *sda_dor;
    __IO uint32_t *sda_dir;
    uint32_t  sda_mask;
    uint32_t  sda_shift;
#else
    uint16_t scl;
    uint16_t sda;
#endif /* SF32LB55X */
} PMIC_PIN_PARAM_T;

typedef struct
{
    int sda;
    int scl;
} PMIC_TWI_PIN_T;

static PMIC_PIN_PARAM_T g_pmic_handle;
#if defined(PMIC_CONTROL_SERVICE)
#if defined(SOC_SF32LB58X)
static const pmic_control_map_t g_pmic_control_map[] =
{
    {PMIC_CONTROL_HR, PMIC_OUT_1V8_LVSW100_3 | PMIC_OUT_LDO28_VOUT},
    {PMIC_CONTROL_BT, PMIC_OUT_VBAT_HVSW150_1},
    {PMIC_CONTROL_MOTOR, PMIC_OUT_LDO30_VOUT},
    {PMIC_CONTROL_GSENSOR, PMIC_OUT_LDO28_VOUT},
    {PMIC_CONTROL_GPS, PMIC_OUT_VBAT_HVSW150_2 | PMIC_OUT_1V8_LVSW100_5},
    {PMIC_CONTROL_AU, PMIC_OUT_1V8_LVSW100_1 | PMIC_OUT_VBAT_HVSW150_1}

};
#else
static const pmic_control_map_t g_pmic_control_map[] =
{
    {PMIC_CONTROL_HR, PMIC_OUT_1V8_LVSW100_3 | PMIC_OUT_LDO28_VOUT},
    {PMIC_CONTROL_BT, PMIC_OUT_VBAT_HVSW150_1},
    {PMIC_CONTROL_MOTOR, PMIC_OUT_LDO30_VOUT},
    {PMIC_CONTROL_GSENSOR, PMIC_OUT_1V8_LVSW100_2},
    {PMIC_CONTROL_GPS, PMIC_OUT_VBAT_HVSW150_2 | PMIC_OUT_1V8_LVSW100_5}
};
#endif
#endif


#ifdef DRV_PMIC_TEST
    static int g_debug = 0;
#endif
HAL_RETM_BSS_SECT(g_twi_busy, static int g_twi_busy);
#if 0//def SF32LB55X
//#define SCL(op) (*scl_dor)=(op==1)?((*scl_dor)|scl_mask):((*scl_dor)&~scl_mask)
//#define SDA(op) (*sda_dor)=(op==1)?((*sda_dor)|sda_mask):((*sda_dor)&~sda_mask)
//#define SDO() (((*sda_dir)&sda_mask)>>sda_shift)
static void SCL(PMIC_PIN_PARAM_T *pm, uint8_t op)
{
    if (op)
        *(pm->scl_dor) |= pm->scl_mask;
    else
        *(pm->scl_dor) &= ~(pm->scl_mask);
}

static void SDA(PMIC_PIN_PARAM_T *pm, uint8_t op)
{
    if (op)
        *(pm->sda_dor) |= pm->sda_mask;
    else
        *(pm->sda_dor) &= ~(pm->sda_mask);

}

static uint32_t SDO(PMIC_PIN_PARAM_T *pm)
{
    return ((*(pm->sda_dir)) & pm->sda_mask) >> pm->sda_shift;
}

#define PA_MAX_PIN_CNT      GPIO1_PIN_NUM

#define LBIT_FIRST          (1)

static void PMIC_init_pins(PMIC_PIN_PARAM_T *pm, uint16_t scl, uint16_t sda)
{
    __IO uint32_t *p ;

    if (scl >= PA_MAX_PIN_CNT)
    {
        p = &(hwp_gpio2->DOR0);
        scl = scl - PA_MAX_PIN_CNT;
        HAL_PIN_Set(PAD_PB00 + scl, GPIO_B0 + scl, PIN_PULLDOWN, 0);
    }
    else
    {
        p = &(hwp_gpio1->DOR0);
        //scl = scl - GPIO_A0;
        HAL_PIN_Set(PAD_PA00 + scl, GPIO_A0 + scl, PIN_PULLDOWN, 1);
    }
    if (scl >= 64)
    {
        p += (sizeof(GPIOx_TypeDef) >> 1);
        scl -= 64;
    }
    else if (scl >= 32)
    {
        p += (sizeof(GPIOx_TypeDef) >> 2);
        scl -= 32;
    }

    pm->scl_mask = (1UL << scl);
    pm->scl_dor = p;
    *(p + 2) |= pm->scl_mask;

    if (sda >= PA_MAX_PIN_CNT)
    {
        p = &(hwp_gpio2->DOR0);
        sda = sda - PA_MAX_PIN_CNT;
        HAL_PIN_Set(PAD_PB00 + sda, GPIO_B0 + sda, PIN_PULLDOWN, 0);
    }
    else
    {
        p = &(hwp_gpio1->DOR0);
        //sda = sda - GPIO_A0;
        HAL_PIN_Set(PAD_PA00 + sda, GPIO_A0 + sda, PIN_PULLDOWN, 1);
    }
    if (sda >= 64)
    {
        p += (sizeof(GPIOx_TypeDef) >> 1);
        sda -= 64;
    }
    else if (sda >= 32)
    {
        p += (sizeof(GPIOx_TypeDef) >> 2);
        sda -= 32;
    }

    pm->sda_shift = sda;
    pm->sda_mask = (1UL << sda);
    pm->sda_dor = p;
    pm->sda_dir = p - 1;
    *(p + 2) |= pm->sda_mask;   // default output
}

static void DBG_SDA_OUTPUT(PMIC_PIN_PARAM_T *pm)
{
    __IO uint32_t *p ;
    p = pm->sda_dor;
    *(p + 2) |= pm->sda_mask;   // oesr
}

static void DBG_SDA_INPUT(PMIC_PIN_PARAM_T *pm)
{
    __IO uint32_t *p ;
    p = pm->sda_dir;
    *(p + 4) |= pm->sda_mask;   // oecr
    *(p + 7) |= pm->sda_mask;   // iecr
}

#else


#define PA_MAX_PIN_CNT      GPIO1_PIN_NUM

#define LBIT_FIRST          (1)

static void GPIO_Init(uint16_t gpio, uint32_t mode)
{
    GPIO_TypeDef *hwp_gpio = (gpio < PA_MAX_PIN_CNT) ? hwp_gpio1 : hwp_gpio2;
    GPIO_InitTypeDef GPIO_InitStruct;

    //Set pin mux to gpio
    if (gpio < PA_MAX_PIN_CNT)
        HAL_PIN_Set(PAD_PA00 + gpio, GPIO_A0 + gpio, PIN_PULLDOWN, 1);
    else
        HAL_PIN_Set(PAD_PB00 + gpio - PA_MAX_PIN_CNT, GPIO_B0 + gpio - PA_MAX_PIN_CNT, PIN_PULLDOWN, 0);

    // set pin to output mode
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pin = (gpio < PA_MAX_PIN_CNT) ? gpio : (gpio - PA_MAX_PIN_CNT);
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(hwp_gpio, &GPIO_InitStruct);
}


static void GPIO_Write(uint16_t gpio, uint16_t val)
{
    if (gpio < PA_MAX_PIN_CNT)
        HAL_GPIO_WritePin(hwp_gpio1, gpio, (GPIO_PinState)val);
    else
        HAL_GPIO_WritePin(hwp_gpio2, gpio - PA_MAX_PIN_CNT, (GPIO_PinState)val);
}

static uint32_t GPIO_Read(uint16_t gpio)
{
    if (gpio < PA_MAX_PIN_CNT)
        return (uint32_t) HAL_GPIO_ReadPin(hwp_gpio1, gpio);
    else
        return (uint32_t) HAL_GPIO_ReadPin(hwp_gpio2, gpio - PA_MAX_PIN_CNT);
}

static void SCL(PMIC_PIN_PARAM_T *pm, uint8_t op)
{
    GPIO_Write(pm->scl, op);
}

static void SDA(PMIC_PIN_PARAM_T *pm, uint8_t op)
{
    GPIO_Write(pm->sda, op);
}

static uint32_t SDO(PMIC_PIN_PARAM_T *pm)
{
    return GPIO_Read(pm->sda);
}

static void PMIC_init_pins(PMIC_PIN_PARAM_T *pm, uint16_t scl, uint16_t sda)
{
    //Save pin
    pm->scl = scl;
    pm->sda = sda;

    GPIO_Init(pm->scl, GPIO_MODE_OUTPUT);
    GPIO_Init(pm->sda, GPIO_MODE_OUTPUT);

    GPIO_Write(pm->scl, 1);
    GPIO_Write(pm->sda, 1);
}

static void DBG_SDA_OUTPUT(PMIC_PIN_PARAM_T *pm)
{
    GPIO_Init(pm->sda, GPIO_MODE_OUTPUT);
}

static void DBG_SDA_INPUT(PMIC_PIN_PARAM_T *pm)
{
    GPIO_Init(pm->sda, GPIO_MODE_INPUT);
}

#endif /* SF32LB55X */

static void PMIC_WAIT(PMIC_PIN_PARAM_T *pm)
{
    HAL_Delay_us(pm->pmic_half_period_us);
}

static void PMIC_set_bits(PMIC_PIN_PARAM_T *pm, uint8_t len, uint16_t data)
{
    int i;
    uint8_t bit;

    //if (pm->sda_dor == NULL || len > 16)
    if (len > 16)
        return;
    //SDA(1);
#if LBIT_FIRST
    for (i = 0; i < len; i++)
#else   // highest bit first
    for (i = len - 1; i >= 0; i--)
#endif
    {
        SCL(pm, 1);
        bit = (data >> i) & 1;
        SDA(pm, bit);
        PMIC_WAIT(pm);
        SCL(pm, 0);
        PMIC_WAIT(pm);
    }
}

static uint32_t PMIC_get_bits(PMIC_PIN_PARAM_T *pm, uint8_t len)
{
    int i;
    uint32_t data = 0;

    //if (pm->sda_dir == NULL || len > 32)
    if (len > 32)
        return 0;

    for (i = 0; i < len; i++)
    {
        SCL(pm, 0);
        PMIC_WAIT(pm);
        data <<= 1;
        data |= SDO(pm) & 1;  // for master use rising to get data
        SCL(pm, 1);
        PMIC_WAIT(pm);
    }
#if LBIT_FIRST
    uint32_t res = 0;

    for (i = 0; i < len; i++)   // switch big/little endian
    {
        res |= ((data >> i) & 0x1) << (len - i - 1);
    }
    data = res;
#endif
    return data;
}


static void PMIC_Interface_Init(PMIC_PIN_PARAM_T *pm, uint32_t freq, uint16_t scl, uint16_t sda)
{
    PMIC_init_pins(pm, scl, sda);

    // default low
    SDA(pm, 0);
    SCL(pm, 0);

    if (freq < 500000 && freq > 0)  // max support 500kh to make sure delay us can work
    {
        pm->pmic_half_period_us = (1000000 / freq) >> 1;
    }
    else
    {
        pm->pmic_half_period_us = 1;
    }
}

// Read PMIC register, slave addr 14 bits, reg addr 8 bits
// Timing: 1 cyc start H + 1 cyc cmd L(read) + 14 cyc addr + 1 cyc dummy + 0.5cyc() + 8 cyc rdata
// 0.5cyc between dummy and rdata : slave send at failing, master read at rising to make sure get data
static uint8_t PMIC_ReadReg(PMIC_PIN_PARAM_T *pm, uint16_t RegAddr)
{
    uint8_t res;
    // 1cyc start H
    SCL(pm, 1);
    SDA(pm, 1);
    PMIC_WAIT(pm);
    SCL(pm, 0);
    PMIC_WAIT(pm);

    // 1 cyc cmd L for read
    SCL(pm, 1);
    SDA(pm, 0);
    PMIC_WAIT(pm);
    SCL(pm, 0);
    PMIC_WAIT(pm);

    // 14 cycle addr
    PMIC_set_bits(pm, 14, RegAddr);

    // 1 dummy cyc
    SCL(pm, 1);
    SDA(pm, 0); // default to low when dummy???
    PMIC_WAIT(pm);
    SCL(pm, 0);
    PMIC_WAIT(pm);

    // switch sda to input
    DBG_SDA_INPUT(pm);
    //0.5 cycle switch
    SCL(pm, 1);
    PMIC_WAIT(pm);

    //SCL(0);
    //PMIC_WAIT();

    res = (uint8_t)PMIC_get_bits(pm, 8);

    // set idle
    //SCL(1);
    //PMIC_WAIT();
    SCL(pm, 0);
    SDA(pm, 0);
    PMIC_WAIT(pm);
    // set default output
    DBG_SDA_OUTPUT(pm);

    return res;
}

// Write PMIC register, slave addr 14 bits, reg addr 8 bits
// Timing: 1 cyc start H + 1 cyc cmd H(write) + 14 cyc addr + 8 cyc wdata + 1 cycle dummy
static void PMIC_WriteReg(PMIC_PIN_PARAM_T *pm, uint16_t RegAddr, uint8_t Val)
{
    // 1cyc start H
    SCL(pm, 1);
    SDA(pm, 1);
    PMIC_WAIT(pm);
    SCL(pm, 0);
    PMIC_WAIT(pm);

    // 1 cyc cmd H for write
    SCL(pm, 1);
    SDA(pm, 1);
    PMIC_WAIT(pm);
    SCL(pm, 0);
    PMIC_WAIT(pm);

    // 14 cycle addr
    PMIC_set_bits(pm, 14, RegAddr);

    // 8 cycle data
    PMIC_set_bits(pm, 8, Val);

    // 1 dummy cyc
    SCL(pm, 1);
    PMIC_WAIT(pm);
    SCL(pm, 0);
    PMIC_WAIT(pm);

    // set idle
    SCL(pm, 1);
    PMIC_WAIT(pm);
    SCL(pm, 0);
    SDA(pm, 0);
    PMIC_WAIT(pm);
    return;
}


void BSP_PMIC_Init()
{
    PMIC_TWI_PIN_T pins;
    pins.scl = PMICC_SCL_PIN;
    pins.sda = PMICC_SDA_PIN;
    PMIC_Interface_Init(&g_pmic_handle, 10000, pins.scl, pins.sda);
}

#ifdef BSP_USING_RTTHREAD
#include "rtthread.h"

#define DRV_DEBUG
#define LOG_TAG              "drv.pmic"
#include <drv_log.h>
static struct rt_semaphore g_pmic_cmd_sem;

static int pmic_os_init(void)
{
    rt_sem_init(&g_pmic_cmd_sem, "pmicos", 1, RT_IPC_FLAG_FIFO);
    LOG_I("PMIC Os init\n");
    return 0;
}
INIT_BOARD_EXPORT(pmic_os_init);
#else
#define rt_sem_take(sem,time) 0
#define rt_sem_release(sem)   0
#define RT_ASSERT(EX)
#endif

static void pmic_delay(uint32_t timeout_ms)
{
    while (timeout_ms > 0)
    {
        HAL_Delay_us(1000);
        timeout_ms--;
    }
}
static uint8_t BSP_PMIC_Read(uint8_t reg)
{
    int     try;
    uint8_t res0, res1;
    res0 = PMIC_ReadReg(&g_pmic_handle, reg);
    pmic_delay(10);
    //skip invalid twi signal by power up gpio state
    for (try = 0 ; try < 3; try++)
                {
                    res1 = PMIC_ReadReg(&g_pmic_handle, reg);
                    pmic_delay(10);
                    if (res0 == res1)
                    {
                        break;
                    }
                    res0 = res1;
                }
    return res0;
}
static void BSP_PMIC_Write(uint8_t reg, uint8_t value)
{
    if (reg == 6 || reg == 7 || reg == 8 || reg == 9)
    {
        if (value & 1)
            value |= (3 << 1);
    }
    while (1)
    {
        PMIC_WriteReg(&g_pmic_handle, (uint16_t)reg, value);
        //if (PMIC_ReadReg(&g_pmic_handle, reg) == value)
        break;
    }
}
int BSP_PMIC_Control(uint16_t out_map, int is_enable, bool lvsw1_twi_en)
{
    uint8_t reg, res, data;
    RT_ASSERT(g_twi_busy == 0);
    g_twi_busy = 1;
    reg  = 0x0c;
    data = 0;
    if (out_map & PMIC_OUT_1V8_LVSW100_1)
        data |= (uint8_t)REG_0C_LVSW1_EN_MASK;
    if (out_map & PMIC_OUT_1V8_LVSW100_2)
        data |= (uint8_t)REG_0C_LVSW2_EN_MASK;
    if (out_map & PMIC_OUT_1V8_LVSW100_3)
        data |= (uint8_t)REG_0C_LVSW3_EN_MASK;
    if (out_map & PMIC_OUT_1V8_LVSW100_4)
        data |= (uint8_t)REG_0C_LVSW4_EN_MASK;
    if (out_map & PMIC_OUT_1V8_LVSW100_5)
        data |= (uint8_t)REG_0C_LVSW5_EN_MASK;
    if (out_map & PMIC_OUT_VBAT_HVSW150_1)
        data |= (uint8_t)REG_0C_HVSW1_EN_MASK;
    if (out_map & PMIC_OUT_VBAT_HVSW150_2)
        data |= (uint8_t)REG_0C_HVSW2_EN_MASK
                ;

    if (data != 0)
    {
        res = BSP_PMIC_Read(reg);
        if (lvsw1_twi_en && (out_map & PMIC_OUT_1V8_LVSW100_1))
            res &= ~REG_0C_LVSW_SEL_MASK; //select by twi
        else if (!lvsw1_twi_en && (out_map & PMIC_OUT_1V8_LVSW100_1))
            res |= REG_0C_LVSW_SEL_MASK;

        if (is_enable)
            res |= data;
        else
            res &= ~data;

        BSP_PMIC_Write(reg, res);
    }
    if (out_map & PMIC_OUT_LDO28_VOUT) //LDO4
    {
        reg  = 0x06;
        data = 3 << REG_06_LDO28_SET_VOUT_POS; // 3 : 2.8V,  0x0f : 3.4V, 0 : 2.65V
        BSP_PMIC_Read(reg);
        if (is_enable)
            data |= 1;
        else
            data &= ~1;
        BSP_PMIC_Write(reg, data);
    }
    if (out_map & PMIC_OUT_LDO30_VOUT) //LDO3
    {
        reg  = 0x07;
        data = 7 << REG_07_LDO30_SET_VOUT_POS; //7:3.0V, 0x0f;3.4V,  0 : 2.65V
        BSP_PMIC_Read(reg);
        if (is_enable)
            data |= 1;
        else
            data &= ~1;
        BSP_PMIC_Write(reg, data);
    }
    if (out_map & PMIC_OUT_LDO33_VOUT) //LDO2
    {
        //select by twi
        reg  = 0x08;
        data = 0x0D << REG_08_LDO33_SET_VOUT_POS; //0x0D:3.0V, 0x0f;3.4V, 0x0-2.65V
        BSP_PMIC_Read(reg);
        if (is_enable)
            data |= 1;
        else
            data &= ~1;
        BSP_PMIC_Write(reg, data);
    }
    g_twi_busy = 0;
    return 0;
}

int pmic_device_control(uint16_t out_map, int is_enable, bool lvsw1_twi_en)
{
    rt_sem_take(&g_pmic_cmd_sem, RT_WAITING_FOREVER);
    BSP_PMIC_Control(out_map, is_enable, lvsw1_twi_en);
    rt_sem_release(&g_pmic_cmd_sem);
    return 0;
}

bool pmic_get_status(pmic_status_t which_to_check)
{
    uint8_t buck = BSP_PMIC_Read(0x05);
    uint8_t ldo  = BSP_PMIC_Read(0x0A);
    uint8_t result = 0;

    switch (which_to_check)
    {
    case PMIC_STATUS_BUCK_POWERON:
        result = buck & (1 << 2);
        break;
    case PMIC_STATUS_BUCK_LOWPOWER:
        result = buck & (1 << 1);
        break;
    case PMIC_STATUS_LDO1_POWERON:
        result = ldo & (1 << 3);
        break;
    case PMIC_STATUS_LDO2_POWERON:
        result = ldo & (1 << 0);
        break;
    case PMIC_STATUS_LDO3_POWERON:
        result = ldo & (1 << 1);
        break;
    case PMIC_STATUS_LDO4_POWERON:
        result = ldo & (1 << 2);
        break;
    }
    return (result != 0);
}

void pmic_twi_ldo_set(pmic_ldo_set_t ldo, bool is_open, uint8_t vol_steps)
{
    uint8_t reg = 0, data = 0;
    if (ldo == PMIC_LDO_4)
    {
        reg  = 0x06;
        data = (vol_steps & 0x0F) << REG_06_LDO28_SET_VOUT_POS;
        BSP_PMIC_Read(reg);
        if (is_open)
            data |= 1;
        else
            data &= ~1;
        BSP_PMIC_Write(reg, data);
    }
    else if (ldo == PMIC_LDO_3) //LDO3
    {
        reg  = 0x07;
        data = (vol_steps & 0x0F) << REG_07_LDO30_SET_VOUT_POS;
        BSP_PMIC_Read(reg);
        if (is_open)
            data |= 1;
        else
            data &= ~1;
        BSP_PMIC_Write(reg, data);
    }
    else if (ldo == PMIC_LDO_2)
    {
        //select by twi
        reg  = 0x08;
        data = (vol_steps & 0x0F) << REG_08_LDO33_SET_VOUT_POS;
        BSP_PMIC_Read(reg);
        if (is_open)
            data |= 1;
        else
            data &= ~1;
        BSP_PMIC_Write(reg, data);
    }
    else if (ldo == PMIC_LDO_1)
    {
        //select by twi
        reg  = 0x09;
        data = (vol_steps & 0x0F) << 3;
        BSP_PMIC_Read(reg);
        BSP_PMIC_Write(reg, data);
    }
}

void pmic_twi_loadsw_set(pmic_loadsw_t sw, bool is_open, bool lvsw1_twi_en)
{
    switch (sw)
    {
    case PMIC_LVSW_1:
        BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_1, is_open, lvsw1_twi_en);
        break;
    case PMIC_LVSW_2:
        BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_2, is_open, lvsw1_twi_en);
        break;
    case PMIC_LVSW_3:
        BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_3, is_open, lvsw1_twi_en);
        break;
    case PMIC_LVSW_4:
        BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_4, is_open, lvsw1_twi_en);
        break;
    case PMIC_LVSW_5:
        BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_5, is_open, lvsw1_twi_en);
        break;
    case PMIC_HVSW_1:
        BSP_PMIC_Control(PMIC_OUT_VBAT_HVSW150_1, is_open, lvsw1_twi_en);
        break;
    case PMIC_HVSW_2:
        BSP_PMIC_Control(PMIC_OUT_VBAT_HVSW150_2, is_open, lvsw1_twi_en);
        break;
    }
}

void pmic_buck_voltage_fine_tuning(uint8_t steps)
{
    uint8_t val = BSP_PMIC_Read(0x00) & 0xC3;
    val |= ((steps & 0x0F) << 2);
    BSP_PMIC_Write(0x00, val);
}

void pmic_twi_pin_set(bool scl_big_current, bool scl_pull_enable, bool sda_big_current, bool sda_pull_enable)
{
    uint8_t val = BSP_PMIC_Read(0x0D);
    val &= 0xD2;
    if (scl_big_current)
        val |= (1 << 5);
    if (scl_pull_enable)
        val |= (1 << 3);
    if (sda_big_current)
        val |= (1 << 2);
    if (sda_pull_enable)
        val |= (1 << 0);

    BSP_PMIC_Write(0x0D, val);
}

#if defined(PMIC_CONTROL_SERVICE)&&defined(BSP_USING_RTTHREAD)
int pmic_control_get_out_map(pmic_control_device_type_t type, uint16_t *out_map)
{
    uint16_t size =  sizeof(g_pmic_control_map) / sizeof(pmic_control_map_t);
    for (uint16_t index = 0; index < size; index++)
    {
        if (type == g_pmic_control_map[index].type)
        {
            *out_map = g_pmic_control_map[index].out_map;
            return 0;
        }
    }
    rt_kprintf("pmic_control_get_out_map not find type:%d", type);
    return -1;
}
#endif

#if defined(DRV_PMIC_TEST)&&defined(BSP_USING_RTTHREAD)
#include <string.h>

int cmd_pmic(int argc, char *argv[])
{
    int addr, res = -1;
    uint8_t value;
    if (argc < 3)
    {
        LOG_I("invalid argc=%d\n", argc);
        return -1;
    }

    if (strcmp(argv[1], "-r") == 0)
    {
        addr = atoi(argv[2]);
        value = BSP_PMIC_Read((uint8_t)addr);
        LOG_I("PMIC reg[0x%02x] = 0x%02x\n", addr, value);
    }
    else if (strcmp(argv[1], "-w") == 0)
    {
        addr = atoi(argv[2]);
        value = (uint8_t)atoi(argv[3]);
        PMIC_WriteReg(&g_pmic_handle, (uint16_t)addr, (uint8_t)value);
        pmic_delay(10);
        res = BSP_PMIC_Read((uint8_t)addr);
        LOG_I("PMIC write 0x%02x = 0x%02x, res %d\n", addr, value, res);
    }
    else if (strcmp(argv[1], "-e") == 0 && argc > 2)
    {
        pmic_device_control(atoi(argv[2]), 1, 1);
    }
    else if (strcmp(argv[1], "-d") == 0 && argc > 2)
    {
        pmic_device_control(atoi(argv[2]), 0, 1);
    }
    else
    {
        LOG_I("unknow param\n");
    }
    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_pmic, __cmd_pmic, Test driver pmic);

#endif //DRV_PMIC_TEST

#endif  // PMIC_CTRL_ENABLE

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
