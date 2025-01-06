/**
  ******************************************************************************
  * @file   dialog7212.c
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
#include "board.h"
#include "dialog7212.h"
#include "string.h"

//#define DRV_DEBUG
#define LOG_TAG              "drv.DA7212"
#include <drv_log.h>


/*******************************************************************************
 * Definitations
 ******************************************************************************/
/*! @brief da7212 reigster structure */
typedef struct _da7212_register_value
{
    uint8_t addr;
    uint8_t value;
} da7212_register_value_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

// make it singletone
static da7212_handle_t da7212_handle;

static const da7212_register_value_t kInputRegisterSequence[kDA7212_Input_MAX][17] =
{
    /* DA7212_Input_AUX */
    {
        {DIALOG7212_MIXIN_L_SELECT, 0x01},
        {DIALOG7212_MIXIN_R_SELECT, 0x01},
        {DIALOG7212_CP_CTRL, 0xFD},
        {DIALOG7212_AUX_L_CTRL, 0xB4},
        {DIALOG7212_AUX_R_CTRL, 0xB0},
        {DIALOG7212_MIC_1_CTRL, 0x04},
        {DIALOG7212_MIC_2_CTRL, 0x04},
        {DIALOG7212_MIXIN_L_CTRL, 0x88},
        {DIALOG7212_MIXIN_R_CTRL, 0x88},
        {DIALOG7212_ADC_L_CTRL, 0xA0},
        {DIALOG7212_GAIN_RAMP_CTRL, 0x02},
        {DIALOG7212_PC_COUNT, 0x02},
        {DIALOG7212_CP_DELAY, 0x95},
    },
    /* DA7212_Input_MIC1_Dig */
    {
        {DIALOG7212_MICBIAS_CTRL, 0xA9},
        {DIALOG7212_CP_CTRL, 0xF1},
        {DIALOG7212_MIXIN_L_SELECT, 0x80},
        {DIALOG7212_MIXIN_R_SELECT, 0x80},
        {DIALOG7212_SYSTEM_MODES_INPUT, 0xFE},
        {DIALOG7212_SYSTEM_MODES_OUTPUT, 0xF7},
        {DIALOG7212_MIC_CONFIG, 0x07},
        {DIALOG7212_MIC_2_GAIN, 0x04},
        {DIALOG7212_MIC_2_CTRL, 0x84},
        {DIALOG7212_MIC_1_GAIN, 0x01},
        {DIALOG7212_MIC_1_CTRL, 0x80},
        {DIALOG7212_ADC_FILTERS1, 0x08},
    },
    /* DA7212_Input_MIC1_An */
    {
        {DIALOG7212_MIXIN_L_SELECT, 0x02},      // MIC1 route to L MINXIN
        {DIALOG7212_MIXIN_R_SELECT, 0x04},      // MIC1 route to R MINXIN
        {DIALOG7212_MIXIN_L_GAIN, 0x03},        // L MIXIN Gain = 0dB
        {DIALOG7212_MIXIN_R_GAIN, 0x03},        // R MIXIN Gain = 0dB
        {DIALOG7212_ADC_L_GAIN, 0x6F},          // L ADC Gain = 0dB
        {DIALOG7212_ADC_R_GAIN, 0x6F},          // R ADC Gain = 0dB
        {DIALOG7212_ADC_FILTERS1, 0x08},        // Voice HFP = 100Hz @ 16k SR
        {DIALOG7212_MIC_1_GAIN, 0x04},          // MIC1 Gain = 18dB
        {DIALOG7212_MIC_2_GAIN, 0x01},          // MIC2 Gain = 0dB
        {DIALOG7212_MICBIAS_CTRL, 0x19},        // MIC2 Bias OFF, MIC1 Bias ON @ 2.2V
        {DIALOG7212_MIC_1_CTRL, 0x84},          // MIC1 AMP Enable, Source = MIC1_P single-ended
        {DIALOG7212_MIC_2_CTRL, 0x04},          // MIC2 AMP Disable
        {DIALOG7212_MIXIN_L_CTRL, 0x88},        // L MIXIN Enable
        {DIALOG7212_MIXIN_R_CTRL, 0x40},        // R MIXIN Disable
        {DIALOG7212_ADC_L_CTRL, 0xA0},          // L ADC Enable, Unmute,
        {DIALOG7212_ADC_R_CTRL, 0xA0},          // R ADC Enable, Unmute
        {DIALOG7212_GAIN_RAMP_CTRL, 0x02},      // 1 second fade-in from zero to max
    },
    /* DA7212_Input_MIC2 */
    {
        {DIALOG7212_MIXIN_L_SELECT, 0x04},
        {DIALOG7212_MIXIN_R_SELECT, 0x02},
        {DIALOG7212_MIC_2_GAIN, 0x04},
        {DIALOG7212_CP_CTRL, 0xFD},
        {DIALOG7212_AUX_R_CTRL, 0x40},
        {DIALOG7212_MICBIAS_CTRL, 0x91},
        {DIALOG7212_MIC_1_CTRL, 0x08},
        {DIALOG7212_MIC_2_CTRL, 0x84},
        {DIALOG7212_MIXIN_L_CTRL, 0x88},
        {DIALOG7212_MIXIN_R_CTRL, 0x88},
        {DIALOG7212_ADC_L_CTRL, 0xA0},
        {DIALOG7212_GAIN_RAMP_CTRL, 0x02},
        {DIALOG7212_PC_COUNT, 0x02},
        {DIALOG7212_CP_DELAY, 0x95},
    }
};

static const da7212_register_value_t kOutputRegisterSequence[kDA7212_Output_MAX][4] =
{
    /* DA7212_Output_HP */
    {
        {DIALOG7212_CP_CTRL, 0xF9},
        {DIALOG7212_LINE_CTRL, 0},
        {
            DIALOG7212_HP_L_CTRL, (DIALOG7212_HP_L_CTRL_AMP_EN_MASK | DIALOG7212_HP_L_CTRL_AMP_RAMP_EN_MASK |
                                   DIALOG7212_HP_L_CTRL_AMP_ZC_EN_MASK | DIALOG7212_HP_L_CTRL_AMP_OE_MASK)
        },
        {
            DIALOG7212_HP_R_CTRL, (DIALOG7212_HP_R_CTRL_AMP_EN_MASK | DIALOG7212_HP_R_CTRL_AMP_RAMP_EN_MASK |
                                   DIALOG7212_HP_R_CTRL_AMP_ZC_EN_MASK | DIALOG7212_HP_R_CTRL_AMP_OE_MASK)
        },
    },
    /* DA7212_Output_SP */
    {
        {DIALOG7212_CP_CTRL, 0x3D},
        {DIALOG7212_HP_L_CTRL, 0x40},
        {DIALOG7212_HP_R_CTRL, 0x40},
        {DIALOG7212_LINE_CTRL, 0xA8},
    }
};

static const da7212_register_value_t kInitRegisterSequence[DA7212_INIT_SIZE] =
{
    {
        DIALOG7212_DIG_ROUTING_DAI,
        DIALOG7212_DIG_ROUTING_DAI_R_SRC_ADC_RIGHT | DIALOG7212_DIG_ROUTING_DAI_L_SRC_ADC_LEFT,
    },
    {
        DIALOG7212_SR,
        DIALOG7212_SR_16KHZ,
    },
    {
        DIALOG7212_REFERENCES,
        DIALOG7212_REFERENCES_BIAS_EN_MASK,
    },
    {
        DIALOG7212_PLL_FRAC_TOP,
        CLEAR_REGISTER,
    },
    {
        DIALOG7212_PLL_FRAC_BOT,
        CLEAR_REGISTER,
    },
    {
        DIALOG7212_PLL_INTEGER,
        0x20,
    },
    {
        DIALOG7212_PLL_CTRL,
        (DIALOG7212_PLL_INDIV_10_20MHZ | DIALOG7212_PLL_EN_MASK | DIALOG7212_PLL_SRM_EN_MASK),
    },
    {
        DIALOG7212_DAI_CLK_MODE,
        DIALOG7212_DAI_BCLKS_PER_WCLK_BCLK32 | DIALOG7212_DAI_CLK_EN_MASK,
    },
    {
        DIALOG7212_DAI_CTRL,
        (DIALOG7212_DAI_EN_MASK | DIALOG7212_DAI_OE_MASK | DIALOG7212_DAI_WORD_LENGTH_16B |
         DIALOG7212_DAI_FORMAT_I2S_MODE),
    },
    {
        DIALOG7212_DIG_ROUTING_DAC,
        (DIALOG7212_DIG_ROUTING_DAC_R_RSC_DAC_R | DIALOG7212_DIG_ROUTING_DAC_L_RSC_DAC_L),
    },
    {
        DIALOG7212_CP_CTRL,
        (DIALOG7212_CP_CTRL_EN_MASK | DIALOG7212_CP_CTRL_SMALL_SWIT_CH_FREQ_EN_MASK |
         DIALOG7212_CP_CTRL_MCHANGE_OUTPUT | DIALOG7212_CP_CTRL_MOD_CPVDD_1 |
         DIALOG7212_CP_CTRL_ANALOG_VLL_LV_BOOSTS_CP),
    },
    {
        DIALOG7212_MIXOUT_L_SELECT,
        (DIALOG7212_MIXOUT_L_SELECT_DAC_L_MASK),
    },
    {
        DIALOG7212_MIXOUT_R_SELECT,
        (DIALOG7212_MIXOUT_R_SELECT_DAC_R_MASK | DIALOG7212_MIXOUT_R_SELECT_MIXIN_L_MASK),
    },
    {
        DIALOG7212_DAC_L_CTRL,
        (DIALOG7212_DAC_L_CTRL_ADC_EN_MASK | DIALOG7212_DAC_L_CTRL_ADC_RAMP_EN_MASK),
    },
    {
        DIALOG7212_DAC_R_CTRL,
        (DIALOG7212_DAC_R_CTRL_ADC_EN_MASK | DIALOG7212_DAC_R_CTRL_ADC_RAMP_EN_MASK),
    },
    {
        DIALOG7212_HP_L_CTRL,
        (DIALOG7212_HP_L_CTRL_AMP_EN_MASK | DIALOG7212_HP_L_CTRL_AMP_RAMP_EN_MASK |
         DIALOG7212_HP_L_CTRL_AMP_ZC_EN_MASK | DIALOG7212_HP_L_CTRL_AMP_OE_MASK),
    },
    {
        DIALOG7212_HP_R_CTRL,
        (DIALOG7212_HP_R_CTRL_AMP_EN_MASK | DIALOG7212_HP_R_CTRL_AMP_RAMP_EN_MASK |
         DIALOG7212_HP_R_CTRL_AMP_ZC_EN_MASK | DIALOG7212_HP_R_CTRL_AMP_OE_MASK),
    },
    {
        DIALOG7212_MIXOUT_L_CTRL,
        (DIALOG7212_MIXOUT_L_CTRL_AMP_EN_MASK | DIALOG7212_MIXOUT_L_CTRL_AMP_SOFT_MIX_EN_MASK |
         DIALOG7212_MIXOUT_L_CTRL_AMP_MIX_EN_MASK),
    },
    {
        DIALOG7212_MIXOUT_R_CTRL,
        (DIALOG7212_MIXOUT_R_CTRL_AMP_EN_MASK | DIALOG7212_MIXOUT_R_CTRL_AMP_SOFT_MIX_EN_MASK |
         DIALOG7212_MIXOUT_R_CTRL_AMP_MIX_EN_MASK),
    },
    {
        DIALOG7212_CP_VOL_THRESHOLD1,
        (DIALOG7212_CP_VOL_THRESHOLD1_VDD2(0x32)),
    },
    {
        DIALOG7212_SYSTEM_STATUS,
        CLEAR_REGISTER,
    },
    {
        DIALOG7212_DAC_L_GAIN,
        kDA7212_DACGain0DB,
    },
    {
        DIALOG7212_DAC_R_GAIN,
        kDA7212_DACGain0DB,
    }
};

/*******************************************************************************
 * Code
 ******************************************************************************/
// init DA7212 pin / power
static void DA7212_POWER_ON(void)
{
    if (DA7212_POWER_PIN >= 255) // invalid pin
        return;

    rt_pin_mode(DA7212_POWER_PIN + 1, PIN_MODE_OUTPUT);
    rt_pin_write(DA7212_POWER_PIN + 1, 1);
}

/// i2c for da7212 init
static uint32_t DA7212_I2C_Init(void)
{
    struct rt_i2c_bus_device *da7212_i2cbus = NULL;
    /* get i2c bus device */
    da7212_i2cbus = rt_i2c_bus_device_find(DA7212_INTERFACE_NAME);
    if (da7212_i2cbus)
    {
        LOG_D("Find i2c bus device %s\n", DA7212_INTERFACE_NAME);
    }
    else
    {
        LOG_E("Can not found i2c bus %s\n", DA7212_INTERFACE_NAME);
    }
    return (uint32_t)da7212_i2cbus;
}

int DA7212_WriteRegister(da7212_handle_t *handle, uint8_t u8Register, uint8_t u8RegisterData)
{
    //assert(handle.config);
    assert(handle->config.slaveAddress != 0U);

    struct rt_i2c_msg msgs;
    uint8_t value[2];
    uint32_t res;

    if (handle->i2cHandle)
    {
        value[0] = u8Register;
        value[1] = u8RegisterData;

        msgs.addr  = handle->config.slaveAddress;    /* Slave address */
        msgs.flags = RT_I2C_WR;        /* Write flag */
        msgs.buf   = value;             /* Slave register address */
        msgs.len   = 2;                /* Number of bytes sent */

        res = rt_i2c_transfer((struct rt_i2c_bus_device *)handle->i2cHandle, &msgs, 1);
        if (res != 1)
        {
            LOG_I("da7212_i2c_write_reg FAIL %d\n", res);
            return 1;
        }
    }

    return 0;
}

int DA7212_ReadRegister(da7212_handle_t *handle, uint8_t u8Register, uint8_t *pu8RegisterData)
{
    //assert(handle.config);
    assert(handle->config.slaveAddress != 0U);

    struct rt_i2c_msg msgs[2];
    uint32_t res;
    uint8_t reg_addr = u8Register;

    if (handle->i2cHandle)
    {
        msgs[0].addr  = handle->config.slaveAddress;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &reg_addr;             /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = handle->config.slaveAddress;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = pu8RegisterData;              /* Read data pointer */
        msgs[1].len   = 1;              /* Number of bytes read */

        res = rt_i2c_transfer((struct rt_i2c_bus_device *)handle->i2cHandle, msgs, 2);
        if (res != 2)
        {
            LOG_I("da7212_i2c_read_reg fail %d\n", res);
            return 1;
        }
    }

    return 0;
}

int DA7212_ModifyRegister(da7212_handle_t *handle, uint8_t reg, uint8_t mask, uint8_t value)
{
    int result;
    uint8_t regValue;

    result = DA7212_ReadRegister(handle, reg, &regValue);
    if (result != 0)
    {
        return result;
    }

    regValue &= (uint8_t)~mask;
    regValue |= value;

    return DA7212_WriteRegister(handle, reg, regValue);
}

int DA7212_Init(da7212_handle_t *handle, da7212_config_t *codecConfig)
{
    assert(codecConfig != NULL);
    assert(handle != NULL);

    uint32_t i              = 0;
    da7212_config_t *config = codecConfig;
    //handle->config          = config;
    memcpy(&(handle->config), config, sizeof(da7212_config_t));

    DA7212_POWER_ON();

    /* i2c bus initialization */
    handle->i2cHandle = DA7212_I2C_Init();
    if (handle->i2cHandle == 0)
        return 1;

    /* reset codec registers */
    DA7212_WriteRegister(handle, DIALOG7212_CIF_CTRL, DIALOG7212_CIF_CTRL_CIF_REG_SOFT_RESET_MASK);

    for (volatile uint32_t i = 0; i < 100000; i++)
    {
        ;
    }
    // self check, read/write register
    uint8_t value, value2;
    DA7212_ReadRegister(handle, DIALOG7212_TONE_GEN_CFG1, &value);
    //LOG_D("Read DIALOG7212_TONE_GEN_CFG1 = 0x%x\n", value);
    DA7212_WriteRegister(handle, DIALOG7212_TONE_GEN_CFG1, value + 1);
    for (volatile uint32_t i = 0; i < 100000; i++)
    {
        ;
    }
    DA7212_ReadRegister(handle, DIALOG7212_TONE_GEN_CFG1, &value2);
    //LOG_D("Read DIALOG7212_TONE_GEN_CFG1 = 0x%x\n", value2);
    if (value2 != value + 1)
    {
        LOG_I("DIALOG7212_TONE_GEN_CFG1 write %x, return %x\n", value + 1, value2);
        return 2;
    }
    DA7212_WriteRegister(handle, DIALOG7212_TONE_GEN_CFG1, value);

    /* If no config structure, use default settings */
    for (i = 0; i < DA7212_INIT_SIZE; i++)
    {
        DA7212_WriteRegister(handle, kInitRegisterSequence[i].addr, kInitRegisterSequence[i].value);
    }

    /* Set to be master or slave */
    DA7212_WriteRegister(handle, DIALOG7212_DAI_CLK_MODE, DIALOG7212_DAI_BCLKS_PER_WCLK_BCLK32 | (config->isMaster << 7U));
    //DA7212_WriteRegister(handle, DIALOG7212_DAI_CLK_MODE, DIALOG7212_DAI_BCLKS_PER_WCLK_BCLK32 | (0 << 7U));

    /* Set the audio protocol */
    DA7212_WriteRegister(handle, DIALOG7212_DAI_CTRL, (DIALOG7212_DAI_EN_MASK | config->protocol));

    /* Set DA7212 functionality */
    if (config->dacSource == kDA7212_DACSourceADC)
    {
        /* Left ADC to R and L DAC */
        DA7212_WriteRegister(handle, DIALOG7212_DIG_ROUTING_DAC, 0x00);
    }
    else
    {
        DA7212_WriteRegister(handle, DIALOG7212_DIG_ROUTING_DAC, 0x32);
    }

    DA7212_ConfigAudioFormat(handle, config->format.mclk_HZ, config->format.sampleRate, config->format.bitWidth);
    DA7212_ChangeInput(handle, kDA7212_Input_MIC1_An);  // set default input as analog mic
    DA7212_ChangeOutput(handle, kDA7212_Output_SP);     // set default speaker output
    DA7212_Mute(handle, 0);     // unmute
    DA7212_SetChannelVolume(handle, 4, 55); // set default volume to 60
    DA7212_SetChannelVolume(handle, 3, 20); // set default volume to 60

    return 0;
}

int DA7212_SetProtocol(da7212_handle_t *handle, da7212_protocol_t protocol)
{
    uint8_t regVal = 0;
    DA7212_ReadRegister(handle, DIALOG7212_DAI_CTRL, &regVal);
    regVal &= ~DIALOG7212_DAI_FORMAT_MASK;
    regVal |= protocol;
    regVal |= DIALOG7212_DAI_EN_MASK;
    //return DA7212_WriteRegister(handle, DIALOG7212_DAI_CTRL, (DIALOG7212_DAI_EN_MASK | protocol));
    return DA7212_WriteRegister(handle, DIALOG7212_DAI_CTRL, regVal);
}

// mono = 1 DAI mono mode, mono=0 DAI stereo mode
int DA7212_SetDAIMono(da7212_handle_t *handle, uint8_t mono)
{
    uint8_t regVal;

    DA7212_ReadRegister(handle, DIALOG7212_DAI_CTRL, &regVal);
    if (mono)
        regVal |= DIALOG7212_DAI_MONO_MODE_MASK;
    else
        regVal &= ~DIALOG7212_DAI_MONO_MODE_MASK;

    return DA7212_WriteRegister(handle, DIALOG7212_DAI_CTRL, regVal);
}

int DA7212_ConfigAudioFormat(da7212_handle_t *handle,
                             uint32_t masterClock_Hz,
                             uint32_t sampleRate_Hz,
                             uint32_t dataBits)
{
    uint32_t sysClock_Hz = 0;
    uint8_t indiv = 0, inputDiv = 0, regVal = 0;
    uint64_t PllValue = 0;
    uint32_t PllFractional;
    uint8_t PllInteger;
    uint8_t PllFracTop;
    uint8_t PllFracBottom;

    switch (sampleRate_Hz)
    {
    case 8000:
        DA7212_WriteRegister(handle, DIALOG7212_SR, DIALOG7212_SR_8KHZ);
        break;
    case 11025:
        DA7212_WriteRegister(handle, DIALOG7212_SR, DIALOG7212_SR_11_025KHZ);
        break;
    case 12000:
        DA7212_WriteRegister(handle, DIALOG7212_SR, DIALOG7212_SR_12KHZ);
        break;
    case 16000:
        DA7212_WriteRegister(handle, DIALOG7212_SR, DIALOG7212_SR_16KHZ);
        break;
    case 22050:
        DA7212_WriteRegister(handle, DIALOG7212_SR, DIALOG7212_SR_22KHZ);
        break;
    case 24000:
        DA7212_WriteRegister(handle, DIALOG7212_SR, DIALOG7212_SR_24KHZ);
        break;
    case 32000:
        DA7212_WriteRegister(handle, DIALOG7212_SR, DIALOG7212_SR_32KHZ);
        break;
    case 44100:
        DA7212_WriteRegister(handle, DIALOG7212_SR, DIALOG7212_SR_44_1KHZ);
        break;
    case 48000:
        DA7212_WriteRegister(handle, DIALOG7212_SR, DIALOG7212_SR_48KHZ);
        break;
    case 88200:
        DA7212_WriteRegister(handle, DIALOG7212_SR, DIALOG7212_SR_88_2KHZ);
        break;
    case 96000:
        DA7212_WriteRegister(handle, DIALOG7212_SR, DIALOG7212_SR_96KHZ);
        break;
    default:
        break;
    }

    /* Set data bits of word */
    DA7212_ReadRegister(handle, DIALOG7212_DAI_CTRL, &regVal);
    regVal &= ~DIALOG7212_DAI_WORD_LENGTH_MASK;
    switch (dataBits)
    {
    case 16:
        regVal |= DIALOG7212_DAI_WORD_LENGTH_16B;
        break;
    case 20:
        regVal |= DIALOG7212_DAI_WORD_LENGTH_20B;
        break;
    case 24:
        regVal |= DIALOG7212_DAI_WORD_LENGTH_24B;
        break;
    case 32:
        regVal |= DIALOG7212_DAI_WORD_LENGTH_32B;
        break;
    default:
        break;
    }
    DA7212_WriteRegister(handle, DIALOG7212_DAI_CTRL, regVal);

    /* Set PLL clock settings */
    if ((sampleRate_Hz == 8000) || (sampleRate_Hz == 16000) || (sampleRate_Hz == 24000) || (sampleRate_Hz == 32000) ||
            (sampleRate_Hz == 48000) || (sampleRate_Hz == 96000))
    {
        sysClock_Hz = 12288000;
    }
    else
    {
        sysClock_Hz = 11289600;
    }

    /* Compute the PLL_INDIV and DIV value for sysClock */
    if ((masterClock_Hz > 2000000) && (masterClock_Hz <= 10000000))
    {
        indiv    = DIALOG7212_PLL_INDIV_2_10MHZ;
        inputDiv = 2;
    }
    else if ((masterClock_Hz > 10000000) && (masterClock_Hz <= 20000000))
    {
        indiv    = DIALOG7212_PLL_INDIV_10_20MHZ;
        inputDiv = 4;
    }
    else if ((masterClock_Hz > 20000000) && (masterClock_Hz <= 40000000))
    {
        indiv    = DIALOG7212_PLL_INDIV_20_40MHZ;
        inputDiv = 8;
    }
    else
    {
        indiv    = DIALOG7212_PLL_INDIV_40_80MHZ;
        inputDiv = 16;
    }

    /* PLL feedback divider is a Q13 value */
    PllValue = (uint64_t)(((uint64_t)((((uint64_t)sysClock_Hz * 8) * inputDiv) << 13)) / (masterClock_Hz));

    /* extract integer and fractional */
    PllInteger    = PllValue >> 13;
    PllFractional = (PllValue - (PllInteger << 13));
    PllFracTop    = (PllFractional >> 8);
    PllFracBottom = (PllFractional & 0xFF);

    DA7212_WriteRegister(handle, DIALOG7212_PLL_FRAC_TOP, PllFracTop);

    DA7212_WriteRegister(handle, DIALOG7212_PLL_FRAC_BOT, PllFracBottom);

    DA7212_WriteRegister(handle, DIALOG7212_PLL_INTEGER, PllInteger);

    regVal = DIALOG7212_PLL_EN_MASK | indiv | DIALOG7212_PLL_SRM_EN_MASK;

    DA7212_WriteRegister(handle, DIALOG7212_PLL_CTRL, regVal);

    return 0;
}

void DA7212_ChangeInput(da7212_handle_t *handle, da7212_Input_t DA7212_Input)
{
    uint32_t i       = 0;
    uint32_t seqSize = sizeof(kInputRegisterSequence[DA7212_Input]) / sizeof(da7212_register_value_t);

    for (i = 0; i < seqSize; i++)
    {
        DA7212_WriteRegister(handle, kInputRegisterSequence[DA7212_Input][i].addr,
                             kInputRegisterSequence[DA7212_Input][i].value);
    }
}

void DA7212_ChangeOutput(da7212_handle_t *handle, da7212_Output_t DA7212_Output)
{
    uint32_t i       = 0;
    uint32_t seqSize = sizeof(kOutputRegisterSequence[DA7212_Output]) / sizeof(da7212_register_value_t);

    for (i = 0; i < seqSize; i++)
    {
        DA7212_WriteRegister(handle, kOutputRegisterSequence[DA7212_Output][i].addr,
                             kOutputRegisterSequence[DA7212_Output][i].value);
    }
}

void DA7212_ChangeHPVolume(da7212_handle_t *handle, da7212_volume_t volume)
{
    DA7212_WriteRegister(handle, DIALOG7212_DAC_L_GAIN, volume);
    DA7212_WriteRegister(handle, DIALOG7212_DAC_R_GAIN, volume);
}

void DA7212_Mute(da7212_handle_t *handle, bool isMuted)
{
    uint8_t val = 0;

    if (isMuted)
    {
        val = DA7212_DAC_MUTE_ENABLED;
    }
    else
    {
        val = DA7212_DAC_MUTE_DISABLED;
    }

    DA7212_WriteRegister(handle, DIALOG7212_DAC_L_CTRL, val);
    DA7212_WriteRegister(handle, DIALOG7212_DAC_R_CTRL, val);
}

int DA7212_SetChannelVolume(da7212_handle_t *handle, uint32_t channel, uint32_t volume)
{
    int retVal   = 0;
    uint16_t muteCtrl = volume == 0U ? 0x40 : 0x80U;
    uint32_t vol      = volume >= 64U ? 64U : volume;

    if (channel & kDA7212_HeadphoneLeft)
    {
        retVal = DA7212_WriteRegister(handle, DIALOG7212_HP_L_GAIN, vol - 1U);
        retVal = DA7212_ModifyRegister(handle, DIALOG7212_HP_L_CTRL, 0xC0U, muteCtrl);
    }

    if (channel & kDA7212_HeadphoneRight)
    {
        retVal = DA7212_WriteRegister(handle, DIALOG7212_HP_R_GAIN, vol - 1U);
        retVal = DA7212_ModifyRegister(handle, DIALOG7212_HP_R_CTRL, 0xC0U, muteCtrl);
    }

    if (channel & kDA7212_Speaker)
    {
        retVal = DA7212_WriteRegister(handle, DIALOG7212_LINE_GAIN, vol - 1U);
        retVal = DA7212_ModifyRegister(handle, DIALOG7212_LINE_CTRL, 0xC0U, muteCtrl);
    }

    return retVal;
}

int DA7212_SetChannelMute(da7212_handle_t *handle, uint32_t channel, bool isMute)
{
    uint8_t regValue = isMute == true ? 0x40U : 0x80U;
    int retVal  = 0;

    if (channel & kDA7212_HeadphoneLeft)
    {
        retVal = DA7212_ModifyRegister(handle, DIALOG7212_HP_L_CTRL, 0xC0U, regValue);
    }

    if (channel & kDA7212_HeadphoneRight)
    {
        retVal = DA7212_ModifyRegister(handle, DIALOG7212_HP_R_CTRL, 0xC0U, regValue);
    }

    if (channel & kDA7212_Speaker)
    {
        retVal = DA7212_ModifyRegister(handle, DIALOG7212_LINE_CTRL, 0xC0U, regValue);
    }

    return retVal;
}

int DA7212_ToneGenerate(da7212_handle_t *handle, int mode, int on)
{
    int retVal  = 0;
    uint8_t value;
    if (on == 0)
    {
        retVal = DA7212_ModifyRegister(handle, DIALOG7212_TONE_GEN_CFG1, DIALOG7212_TONE_GEN_CFG1_START_STOPN_MASK, 0);
        return retVal;
    }
    //value = 0x55;
    //retVal = DA7212_WriteRegister(handle, DIALOG7212_TONE_GEN_FREQ1_L,value);
    //value = 0x15;
    //retVal = DA7212_WriteRegister(handle, DIALOG7212_TONE_GEN_FREQ1_U,value);

    // mode 0 for sine-wave and mode 1 for beep
    if (mode == 0)
    {
        value = (3 << 4) | 0;
        retVal = DA7212_WriteRegister(handle, DIALOG7212_TONE_GEN_CFG2, value);
        value = (1 << 7) | (1 << 4) | 7;
        retVal = DA7212_WriteRegister(handle, DIALOG7212_TONE_GEN_CFG1, value);
    }
    else
    {
        value = 7;  // INFINITE
        retVal = DA7212_WriteRegister(handle, DIALOG7212_TONE_GEN_CYCLES, value);
        value = 0X14;  // 200MS ON
        retVal = DA7212_WriteRegister(handle, DIALOG7212_TONE_GEN_ON_PER, value);
        value = 0X0A;  // 100MS OFF
        retVal = DA7212_WriteRegister(handle, DIALOG7212_TONE_GEN_OFF_PER, value);
        value = (1 << 7); // start
        retVal = DA7212_WriteRegister(handle, DIALOG7212_TONE_GEN_CFG1, value);
    }

    return retVal;
}

int DA7212_Deinit(da7212_handle_t *handle)
{
    return 0;
}

da7212_handle_t *DA7212_GetHandle(void)
{
    return &da7212_handle;
}

int rt_da7212_default_init(void)
{
    int ret = 0;
    da7212_config_t codecConfig;

    codecConfig.protocol = kDA7212_BusI2S; //kDA7212_BusDSPMode; //kDA7212_BusI2S;
    codecConfig.dacSource = kDA7212_DACSourceInputStream; //kDA7212_DACSourceADC; //kDA7212_DACSourceInputStream;
    codecConfig.format.mclk_HZ = 12288000;
    codecConfig.format.sampleRate = 16000;
    codecConfig.format.bitWidth = 16;
    codecConfig.isMaster = 0;
    codecConfig.slaveAddress = DA7212_ADDRESS;
    ret = DA7212_Init(&da7212_handle, &codecConfig);

    return ret;
}

INIT_COMPONENT_EXPORT(rt_da7212_default_init);


#define DRV_DA7212_TEST
#ifdef DRV_DA7212_TEST

#include <string.h>

da7212_config_t codecConfig;

int da7212_test(int argc, char *argv[])
{
    int ret;
    if (argc < 2)
    {
        LOG_I("Invalid parameter\n");
        return 0;
    }

    if (strcmp(argv[1], "-open") == 0)
    {
        codecConfig.protocol = kDA7212_BusI2S; //kDA7212_BusDSPMode; //kDA7212_BusI2S;
        codecConfig.dacSource = kDA7212_DACSourceInputStream; //kDA7212_DACSourceADC; //kDA7212_DACSourceInputStream;
        codecConfig.format.mclk_HZ = 12288000;
        codecConfig.format.sampleRate = 16000;
        codecConfig.format.bitWidth = 16;
        codecConfig.isMaster = 0;
        codecConfig.slaveAddress = DA7212_ADDRESS;
        ret = DA7212_Init(&da7212_handle, &codecConfig);
        if (ret == 0)
        {
            LOG_I("DA7212 open success\n");
        }
        else
        {
            LOG_I("DA7212 open fail ret\n", ret);
        }
        //DA7212_SetDAIMono(&handle, 1);
    }
    else if (strcmp(argv[1], "-mute") == 0)
    {
        int mute = atoi(argv[2]);
        DA7212_Mute(&da7212_handle, mute);
    }
    else if (strcmp(argv[1], "-vol") == 0)
    {
        int chnl = atoi(argv[2]);    // 1 hp left, 2 hp right, 4 sp
        int vol = atoi(argv[3]);
        DA7212_SetChannelVolume(&da7212_handle, chnl, vol);
    }
    else if (strcmp(argv[1], "-input") == 0)
    {
        int in = atoi(argv[2]); // 0 aux, 1 mic1 dig, 2 mic1 an, 3 mic2
        DA7212_ChangeInput(&da7212_handle, in);
    }
    else if (strcmp(argv[1], "-output") == 0)
    {
        int out = atoi(argv[2]);  // 0 hp, 1 sp
        DA7212_ChangeOutput(&da7212_handle, out);
    }
    else if (strcmp(argv[1], "-config") == 0)
    {
        int clk = atoi(argv[2]);
        int samplerate = atoi(argv[3]);
        int bitwidth = atoi(argv[4]);
        DA7212_ConfigAudioFormat(&da7212_handle, clk, samplerate, bitwidth);
    }
    else if (strcmp(argv[1], "-test") == 0)
    {
        int mode = atoi(argv[2]);
        int on = atoi(argv[3]);
        ret = DA7212_ToneGenerate(&da7212_handle, mode, on);
        LOG_I("DA7212 start(1)/stop(0) test: %d, %d\n", on, ret);
    }
    else if (strcmp(argv[1], "-read") == 0)
    {
        uint8_t reg = atoi(argv[2]) & 0xff;
        uint8_t value;
        ret = DA7212_ReadRegister(&da7212_handle, reg, &value);
        LOG_I("Read Reg x%x, value 0x%x\n", reg, value);
    }
    else if (strcmp(argv[1], "-write") == 0)
    {
        uint8_t reg = atoi(argv[2]) & 0xff;
        uint8_t value = atoi(argv[3]) & 0xff;
        ret = DA7212_WriteRegister(&da7212_handle, reg, value);
        LOG_I("write Reg x%x, value 0x%x\n", reg, value);
    }
    else
    {
        LOG_I("invalid parameter\n");
    }

    return 0;

}

FINSH_FUNCTION_EXPORT_ALIAS(da7212_test, __cmd_da7212, Test hw da7212);
#endif //DRV_DA7212_TEST


