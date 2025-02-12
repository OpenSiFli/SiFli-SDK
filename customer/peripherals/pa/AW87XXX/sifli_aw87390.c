/**
  ******************************************************************************
  * @file   drv_aw87390.c
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
#include "board.h"
#include "pmic_controller.h"
#include <stdlib.h>

#define DBG_TAG           "aw87390"
#define DBG_LVL           AUDIO_DBG_LVL
#include "log.h"


static struct rt_i2c_bus_device *aw_i2c_bus = NULL;
#define AW87390_I2C_NAME  "i2c3"
#define AW87390_I2C_ADDR  0x58

#if defined(BSP_USING_BOARD_EC_LB567XXX)
    #define  AW87390_GPIO_PIN    78
    #define  AW87390_GPIO_HWP    hwp_gpio1
#else
    #define  AW87390_GPIO_PIN    88
    #define  AW87390_GPIO_HWP    hwp_gpio1
#endif

void aw87390_gpio_write(bool State)
{
    GPIO_PinState PinState = (GPIO_PinState)State;
    HAL_GPIO_WritePin(AW87390_GPIO_HWP, AW87390_GPIO_PIN, PinState);
    HAL_Delay_us(5);
}
void aw87390_gpio_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // set sensor pin to output mode
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin = AW87390_GPIO_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(AW87390_GPIO_HWP, &GPIO_InitStruct);
}


typedef struct
{
    uint8_t reg_addr;
    uint8_t reg_data;
} aw87xxx_i2c_type;

#if 0
//喇叭模式寄存器
static aw87xxx_i2c_type aw87390_kspk_reg[] =
{
    {0x67, 0x23},/*排列顺序：reg_addr, reg_data*/
    {0x02, 0x07},
    {0x02, 0x00},
    {0x02, 0x1D},
    {0x03, 0x08},
    {0x04, 0x05},
    {0x05, 0x12},//0x0C
    {0x06, 0x07},
    {0x07, 0x4E},
    {0x08, 0x06},
    {0x09, 0x08},
    {0x0A, 0x4B},
    {0x61, 0xB3},
    {0x62, 0x24},
    {0x63, 0x09},
    {0x64, 0x24},
    {0x65, 0x15},
    {0x79, 0x7A},
    {0x7A, 0x6C},
    {0x78, 0x80},
    {0x66, 0x38},
    {0x76, 0x00},
    {0x78, 0x00},
    {0x68, 0x1B},
    {0x69, 0x5B},
    {0x70, 0x1C},
    {0x71, 0x00},
    {0x72, 0xFE},
    {0x73, 0x4F},
    {0x74, 0x24},
    {0x75, 0x02},
    {0x01, 0x07},
    {0xFF, 0x50},
};
#else
static aw87xxx_i2c_type aw87390_kspk_reg[] =
{
    {0x67, 0x03},/*排列顺序：reg_addr, reg_data*/
    {0x02, 0x07},
    {0x02, 0x00},
    {0x02, 0x0D},
    {0x03, 0x08},
    {0x04, 0x05},
    {0x05, 0x0C},
    {0x06, 0x0E},//0x07
    {0x07, 0x4E},
    {0x08, 0x06},
    {0x09, 0x08},
    {0x0A, 0x4B},//0x3A
    {0x61, 0xB3},
    {0x62, 0x24},
    {0x63, 0x05},
    {0x64, 0x48},
    {0x65, 0x17},
    {0x79, 0x7A},
    {0x7A, 0x6C},
    {0x78, 0x80},
    {0x66, 0x38},
    {0x76, 0x00},
    {0x78, 0x00},
    {0x68, 0x1B},
    {0x69, 0x5B},
    {0x70, 0x1D},
    {0x71, 0x10},
    {0x72, 0xB4},
    {0x73, 0x4F},
    {0x74, 0x24},
    {0x75, 0x02},
    {0x01, 0x07},
    {0xFF, 0x00},
};
#endif
//听筒模式寄存器
static const aw87xxx_i2c_type aw87390_rcv_reg[] =
{
    {0x67, 0x03},/*排列顺序：reg_addr, reg_data*/
    {0x02, 0x07},
    {0x02, 0x00},
    {0x02, 0x00},
    {0x03, 0x08},
    {0x04, 0x05},
    {0x05, 0x13},
    {0x06, 0x0F},
    {0x07, 0x4E},
    {0x08, 0x09},
    {0x09, 0x08},
    {0x0A, 0x4B},
    {0x61, 0xB3},
    {0x62, 0x24},
    {0x63, 0x05},
    {0x64, 0x28},
    {0x65, 0x15},
    {0x79, 0x7A},
    {0x7A, 0x6C},
    {0x78, 0x80},
    {0x66, 0xB8},
    {0x76, 0x00},
    {0x78, 0x00},
    {0x68, 0x1B},
    {0x69, 0x5B},
    {0x70, 0x1C},
    {0x71, 0x10},
    {0x72, 0xB4},
    {0x73, 0x4F},
    {0x74, 0x24},
    {0x75, 0x02},
    {0x01, 0x07},
    {0xFF, 0x10},
};

static const aw87xxx_i2c_type aw87390_off_reg[] =
{
    {0x01, 0x00}, /*排列顺序：reg_addr, reg_data*/
};


static void I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
    struct rt_i2c_msg msgs[2];
    uint8_t value[2];
    uint32_t res;

    if (aw_i2c_bus)
    {
        value[0] = RegAddr;
        value[1] = Data;

        msgs[0].addr  = DevAddr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = value;             /* Slave register address */
        msgs[0].len   = 2;                /* Number of bytes sent */



        res = rt_i2c_transfer(aw_i2c_bus, msgs, 1);
        if (res == 1)
        {
            LOG_D("aw8390 write %d : %d OK\n", RegAddr, Data);
        }
        else
        {
            LOG_E("I2C_WriteOneByte FAIL %d\n", res);
        }

    }
}


void aw87390_WriteReg(uint8_t RegAddr, uint8_t Val)
{
    I2C_WriteOneByte(AW87390_I2C_ADDR, RegAddr, Val);
}


void AW87390_I2C3_Init()
{
    /* get i2c bus device */
    aw_i2c_bus = rt_i2c_bus_device_find(AW87390_I2C_NAME);
    if (aw_i2c_bus)
    {
        LOG_D("Find i2c bus device %s\n", AW87390_I2C_NAME);
        //if (0 != rt_device_open(&(aw_i2c_bus->parent), RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX))
        if (0 != rt_device_open(&(aw_i2c_bus->parent), RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX))
        {
            LOG_E("Can not open i2c bus %s!!\n", AW87390_I2C_NAME);
        }
        {
            struct rt_i2c_configuration configuration =
            {
                .mode = 0,
                .addr = 0,
                .timeout = 5000,
                .max_hz  = 400000,
            };

            rt_i2c_configure(aw_i2c_bus, &configuration);
        }

    }
    else
    {
        LOG_E("Can not found i2c bus %s, init fail\n", AW87390_I2C_NAME);
        //return -1;
    }
}



void sifli_aw87390_start()
{
    aw87390_gpio_write(1);
    HAL_Delay(5);
    aw87390_WriteReg(0, 0xAA);//PA soft reset
    //HAL_Delay(5);

    /* 喇叭模式 */
    for (int i = 0; i < sizeof(aw87390_kspk_reg) / sizeof(aw87390_kspk_reg[0]); i++)
    {
        aw87390_WriteReg(aw87390_kspk_reg[i].reg_addr, aw87390_kspk_reg[i].reg_data);
    }
#if 0
    /* 听筒模式 */
    for (int i = 0; i < sizeof(aw87390_rcv_reg) / sizeof(aw87390_rcv_reg[0]); i++)
    {
        aw87390_WriteReg(aw87390_rcv_reg[i].reg_addr, aw87390_rcv_reg[i].reg_data);
    }
#endif
}

void sifli_aw87390_stop()
{
    for (int i = 0; i < sizeof(aw87390_off_reg) / sizeof(aw87390_off_reg[0]); i++)
    {
        aw87390_WriteReg(aw87390_off_reg[i].reg_addr, aw87390_off_reg[i].reg_data);
    }
    aw87390_gpio_write(0);
}

void sifli_aw87390_init()
{
    pmic_device_control(PMIC_OUT_1V8_LVSW100_1, 1, 1);
    //hwp_pinmux1->PAD_PA88 = 0x230;   //pull up VDD18_AU
    //HAL_Delay(10);
    aw87390_gpio_init();
    AW87390_I2C3_Init();


    LOG_I("sifli_aw87390_init \n");
}

int apa_set_gain(int argc, char *argv[])
{
    rt_thread_t thread;
    int gain;

    if (argc != 2)
    {
        rt_kprintf("arg para num error\n");
        return -1;
    }
    gain = strtol(argv[1], NULL, 10);
    rt_kprintf("analog PA gain=%d\n", gain);

    if ((gain >= 8) && (gain <= 18))
    {
        aw87390_kspk_reg[6].reg_data = gain;
    }
    else
    {
        rt_kprintf("analog PA gain error \n");
    }


    return 0;
}

MSH_CMD_EXPORT(apa_set_gain, set analog pa gain);

int apa_set_reg(int argc, char *argv[])
{
    int index, value;

    if (argc != 3)
    {
        rt_kprintf("arg para num error\n");
        return -1;
    }
    index = strtol(argv[1], NULL, 10);
    value = strtol(argv[2], NULL, 16);

    if (index < 33)
    {
        aw87390_kspk_reg[index].reg_data = value;
    }
    else
    {
        rt_kprintf("reg index is error\n");
        return -1;
    }

    rt_kprintf("analog PA reg set ok\n");
    rt_kprintf("now aw87390 reg is:\n");

    for (int i = 0; i < 33; i++)
    {
        rt_kprintf("regaddr:0x%x,regdata:0x%x\n", aw87390_kspk_reg[i].reg_addr, aw87390_kspk_reg[i].reg_data);
    }


    return 0;
}

MSH_CMD_EXPORT(apa_set_reg, set analog pa register);



