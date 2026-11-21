/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef USE_HAL_DRIVER
    #include "bf0_hal.h"
    #include "bf0_hal_pinmux.h"
    #include "string.h"
#else
    #include "bf0_hal_hlp.h"
#endif
/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @defgroup HAL_DBG Hal Debug functions
 * @brief HAL debug driver
 * @{
 */
#ifdef  USE_HAL_DRIVER
__WEAK void HAL_DBG_printf(const char *fmt, ...)
{
}
#endif

/**
 * @brief  Print a byte value in hex string.
 * @param[in,out] p: hex string.
 * @param[in] d: a byte value.
 * @retval None.
 */
static void print_hex(unsigned char *p, unsigned char d)
{
    (*p) = d & 0xf0;
    (*p) >>= 4;
    if ((*p) < 10)
        *p += '0';
    else
        *p += ('A' - 10);
    p++;
    (*p) = d & 0xf;
    if ((*p) < 10)
        *p += '0';
    else
        *p += ('A' - 10);

}

/**
 * @brief  Set ascii output for a char, if not visible, display '.' .
 * @param[in] buffer: buffer for current line
 * @param[in] ch: char to display
 * @param[in] offset: offset for current line.
 * @retval None.
 */
static void setchar(char *buffer, unsigned char ch, int offset)
{
    if (ch > ' ' && ch < '~')
        buffer[60 + offset] = ch;
    else
        buffer[60 + offset] = '.';
}


/**
 * @brief  Dump a data in hex, also display ASCII on the right.
 * @param[in] data: data to dump.
 * @param[in] offset: start offset from data.
 * @param[in] len: length of data to dump.
 * @retval None.
 */
__HAL_ROM_USED void HAL_DBG_print_data(char *data, unsigned int offset, int len)
{
    int i;
    static char temp[80];
    char *p = temp;

    memset(temp, ' ', sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    for (i = 0; i < len; i++)
    {
        if ((i & 0xf) == 0)
        {
            HAL_DBG_printf("%s\n", temp);
            memset(temp, ' ', sizeof(temp) - 1);
            p = temp;
            print_hex((unsigned char *)p, (offset & 0xff00) >> 8);
            p += 2;
            print_hex((unsigned char *)p, offset & 0xff);
            p += 2;
            *p++ = ':';
            *p++ = ' ';
        }
        setchar(temp, data[offset], i & 0xf);
        print_hex((unsigned char *)p, data[offset++]);
        p += 2;
        *p = ' ';
        p++;
    }
    HAL_DBG_printf("%s\n", temp);
    memset(temp, ' ', sizeof(temp) - 1);
}

#ifdef USE_HAL_DRIVER
static __IO uint32_t *scl_dor;
static uint32_t  scl_mask;
static __IO uint32_t *sda_dor;
static uint32_t  sda_mask;
#define SCL(op) (*scl_dor)=(op==1)?((*scl_dor)|scl_mask):((*scl_dor)&~scl_mask)
#define SDA(op) (*sda_dor)=(op==1)?((*sda_dor)|sda_mask):((*sda_dor)&~sda_mask)


__HAL_ROM_USED void HAL_DBG_i2c_pins(uint16_t scl, uint16_t sda)
{
    __IO GPIO_TypeDef *p ;
#if GPIO2_BASE
    if (scl >= GPIO_B0)
    {
        p = hwp_gpio2;
        scl = scl - GPIO_B0;
    }
    else
#endif /* GPIO2_BASE */
    {
        p = hwp_gpio1;
        scl = scl - GPIO_A0;
    }
    if (scl >= 64)
    {
        p += 2;
        scl -= 64;
    }
    else if (scl >= 32)
    {
        p += 1;
        scl -= 32;
    }

    scl_mask = (1UL << scl);
    scl_dor = &(p->DOR);
    p->DOER |= scl_mask;

#if GPIO2_BASE
    if (sda >= GPIO_B0)
    {
        p = hwp_gpio2;
        sda = sda - GPIO_B0;
    }
    else
#endif /* GPIO2_BASE */
    {
        p = hwp_gpio1;
        sda = sda - GPIO_A0;
    }
    if (sda >= 64)
    {
        p += 2;
        sda -= 64;
    }
    else if (sda >= 32)
    {
        p += 1;
        sda -= 32;
    }

    sda_mask = (1UL << sda);
    sda_dor = &(p->DOR);
    p->DOER |= sda_mask;
}


__HAL_ROM_USED void HAL_DBG_i2c_print_byte(uint8_t byte)
{
    int i;
    uint8_t bit;

    if (sda_dor == NULL)
        return;
    SDA(1);
    SCL(1);
    SDA(0);
    for (i = 7; i >= 0; i--)
    {
        SCL(0);
        bit = (byte >> i) & 1;
        SDA(bit);
        SCL(1);
    }
    SCL(0);
    SDA(0);
    SCL(1);
    SDA(1);
}

__HAL_ROM_USED void HAL_DBG_i2c_print(uint8_t *data, int len)
{
    int i;
    for (i = 0; i < len; i++, data++)
        HAL_DBG_i2c_print_byte(*data);
}

#endif

static int8_t atohex(char ch)
{

    int8_t r = 0;
    if (ch >= '0' && ch <= '9')
        r = ch - '0';
    else if (ch >= 'a' && ch <= 'f')
        r = ch - 'a' + 10;
    else if (ch >= 'A' && ch <= 'F')
        r = ch - 'A' + 10;
    else
        r = -1;
    return r;
}

/**
 * @brief  Convert a hex string to integer.
 * @param[in] astr hex string
 * @retval integer represented by astr.
 */
__HAL_ROM_USED uint32_t atoh(char *astr)
{
    uint32_t r = 0;
    int hex = 0;

    while (astr && *astr)
    {
        hex = atohex(*astr++);
        if (hex >= 0)
            r = (r << 4) + hex;
        else
            break;
    }
    return r;
}

/**
 * @brief  Convert a hex string to byte array
 * @param[in] hexstr hex string
 * @param[in, out] byte array
 * @param[in] max_len max length of byte array
 * @retval length of hex data array.
 */
__HAL_ROM_USED int hex2data(char *hexstr, uint8_t *data, uint8_t max_len)
{
    int i = 0;

    while (*hexstr && i < max_len)
    {
        char temp[3];
        temp[0] = *hexstr++;
        temp[1] = *hexstr;
        if (*hexstr)
        {
            temp[2] = 0;
            hexstr++;
        }
        data[i++] = atoh(temp);
    }
    return i;
}

/**
 * @brief  Convert a hex string to byte array in revert sequence
 * @param[in] hexstr hex string
 * @param[in, out] byte array
 * @param[in] max_len max length of byte array
 * @retval integer represented by astr.
 */
__HAL_ROM_USED int hex2data_revert(char *hexstr, uint8_t *data, uint8_t max_len)
{
    int i = max_len - 1;

    while (*hexstr && i >= 0)
    {
        char temp[3];
        temp[0] = *hexstr++;
        temp[1] = *hexstr;
        if (*hexstr)
        {
            temp[2] = 0;
            hexstr++;
        }
        data[i--] = atoh(temp);
    }
    return i;
}

#ifdef USE_HAL_DRIVER

/**
 * @brief  Set the RTC backup register, used to share value between H/L CPU
 * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
 *                the configuration information for RTC.
 * @param  idx index of backup register
 * @param  value value to set into backup register
 * @retval None
 */

HAL_RAM_RET_CODE_SECT(HAL_Set_backup, __HAL_ROM_USED void HAL_Set_backup(uint8_t idx, uint32_t value))
{
    volatile uint32_t *p = &hwp_rtc->BKP0R;
    *(p + idx) = value;
}

/**
 * @brief  Get the RTC backup register, used to share value between H/L CPU
 * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
 *                the configuration information for RTC.
 * @param  idx index of backup register
 * @retval value saved in the backup register
 */
HAL_RAM_RET_CODE_SECT(HAL_Get_backup, __HAL_ROM_USED uint32_t HAL_Get_backup(uint8_t idx))
{
    volatile uint32_t *p = &hwp_rtc->BKP0R;
    return *(p + idx);
}

#endif

/**
 * @} HAL_Exported_Functions
 */


/**
 * @} BF0_HAL_Driver
 */