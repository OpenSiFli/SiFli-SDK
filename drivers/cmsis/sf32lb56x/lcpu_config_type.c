/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>

#include "bf0_hal.h"
#include "lcpu_config_type.h"


uint8_t LCPU_CONFIG_set(uint8_t *base_addr, uint8_t conifg_type, uint8_t *value, uint16_t length)
{
    uint8_t ret = 1;
    switch (conifg_type)
    {
    case HAL_LCPU_CONFIG_ADC_CALIBRATION_ROM:
    {
        if (length == LCPU_CONFIG_ADC_CALIBRATION_ROM_LENGTH) // 32 bit
        {
            memcpy(base_addr + LCPU_CONFIG_ADC_CALIBRATION_ROM_OFFSET, value, LCPU_CONFIG_ADC_CALIBRATION_ROM_LENGTH);
            ret = 0;
        }
        break;
    }
    case HAL_LCPU_CONFIG_SDADC_CALIBRATION_ROM:
    {
        if (length == LCPU_CONFIG_SDADC_CALIBRATION_ROM_LENGTH) // 64 bit
        {
            memcpy(base_addr + LCPU_CONFIG_SDADC_CALIBRATION_ROM_OFFSET, value, LCPU_CONFIG_SDADC_CALIBRATION_ROM_LENGTH);
            ret = 0;
        }
        break;
    }
    case HAL_LCPU_CONFIG_SN_ROM:
    {
        if (length <= LCPU_CONFIG_SN_ROM_LENGTH)
        {
            memcpy(base_addr + LCPU_CONFIG_SN_ROM_OFFSET, value, length);
            memcpy(base_addr + LCPU_CONFIG_SN_LEN_ROM_OFFSET, &length, LCPU_CONFIG_SN_LEN_ROM_LENGTH);
            ret = 0;
        }
        break;
    }
    case HAL_LCPU_CONFIG_CHIP_REV_ROM:
    {
        if (length == LCPU_CONFIG_CHIP_REV_ROM_LENGTH)
        {
            memcpy(base_addr + LCPU_CONFIG_CHIP_REV_ROM_OFFSET, value, length);
            ret = 0;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BATTERY_CALIBRATION_ROM:
    {
        if (length == 8)
        {
            memcpy(base_addr + LCPU_CONFIG_BATTERY_A_ROM_OFFSET, value, length);
            ret = 0;
        }
        else if (length == 12) //maybe 12 bytes
        {
            memcpy(base_addr + LCPU_CONFIG_BATTERY_A_ROM_OFFSET, value + 4, length - 4);
            ret = 0;
        }
        break;
    }
    case HAL_LCPU_CONFIG_IS_SOFT_CVSD_ROM:
    {
        if (length == LCPU_CONFIG_IS_SOFT_CVSD_ROM_LENGTH)
        {
            memcpy(base_addr + LCPU_CONFIG_IS_SOFT_CVSD_ROM_OFFSET, value, length);
            ret = 0;
        }
        break;
    }
    default:
        break;
    }

    return ret;
}



uint8_t LCPU_CONFIG_get(uint8_t *base_addr, uint8_t config_type, uint8_t *value, uint16_t *length)
{
    uint8_t ret = 1;
    switch (config_type)
    {
    case HAL_LCPU_CONFIG_ADC_CALIBRATION_ROM:
    {
        if (*length == LCPU_CONFIG_ADC_CALIBRATION_ROM_LENGTH) // 32 bit
        {
            memcpy((void *)value, base_addr + LCPU_CONFIG_ADC_CALIBRATION_ROM_OFFSET, LCPU_CONFIG_ADC_CALIBRATION_ROM_LENGTH);
            ret = 0;
        }
        break;
    }
    case HAL_LCPU_CONFIG_SDADC_CALIBRATION_ROM:
    {
        if (*length == LCPU_CONFIG_SDADC_CALIBRATION_ROM_LENGTH) // 64 bit
        {
            memcpy((void *)value, base_addr + LCPU_CONFIG_SDADC_CALIBRATION_ROM_OFFSET, LCPU_CONFIG_SDADC_CALIBRATION_ROM_LENGTH);
            ret = 0;
        }
        break;
    }
    case HAL_LCPU_CONFIG_SN_ROM:
    {
        uint16_t sn_len;
        memcpy(&sn_len, base_addr + LCPU_CONFIG_SN_LEN_ROM_OFFSET, LCPU_CONFIG_SN_LEN_ROM_LENGTH);
        if (*length >= sn_len)
        {
            memcpy(value, base_addr + LCPU_CONFIG_SN_ROM_OFFSET, sn_len);
            *length = sn_len;
            ret = 0;
        }
        break;
    }
    case HAL_LCPU_CONFIG_CHIP_REV_ROM:
    {
        if (*length == LCPU_CONFIG_CHIP_REV_ROM_LENGTH)
        {
            memcpy(value, base_addr + LCPU_CONFIG_CHIP_REV_ROM_OFFSET, LCPU_CONFIG_CHIP_REV_ROM_LENGTH);
            ret = 0;
        }
        break;
    }
    case HAL_LCPU_CONFIG_BATTERY_CALIBRATION_ROM:
    {
        if (*length == 8)
        {
            memcpy(value, base_addr + LCPU_CONFIG_BATTERY_A_ROM_OFFSET, 8);
            ret = 0;
        }
        else if (*length == 12)
        {
            *((uint32_t *)value) = 0xe8091ad7;
            memcpy(value + 4, base_addr + LCPU_CONFIG_BATTERY_A_ROM_OFFSET, 8);
            ret = 0;
        }
        break;
    }
    case HAL_LCPU_CONFIG_IS_SOFT_CVSD_ROM:
    {
        if (*length == LCPU_CONFIG_IS_SOFT_CVSD_ROM_LENGTH)
        {
            memcpy(value, base_addr + LCPU_CONFIG_IS_SOFT_CVSD_ROM_OFFSET, LCPU_CONFIG_IS_SOFT_CVSD_ROM_LENGTH);
            ret = 0;
        }
        break;
    }
    default:
        break;
    }
    return ret;

}


uint16_t LCPU_CONFIG_get_total_size(void)
{
    uint16_t size = LCPU_CONFIG_ROM_SIZE;
    return size;
}
