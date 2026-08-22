/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "utf8_convert_to_unicode.h"
#include <rtthread.h>
#ifndef NULL
    #define NULL  ((void*)0)
#endif

static int enc_get_utf8_size(unsigned char pInput);
static int enc_utf8_to_unicode_one(unsigned char *pInput, unsigned short *Unic);
static int enc_unicode_to_utf8_one(unsigned long unic, unsigned char *pOutput, int outSize);


static int enc_get_utf8_size(unsigned char pInput)
{
    unsigned char c = pInput;

    if (c < 0x80) return 0;
    if (c >= 0x80 && c < 0xC0) return -1;
    if (c >= 0xC0 && c < 0xE0) return 2;
    if (c >= 0xE0 && c < 0xF0) return 3;
    if (c >= 0xF0 && c < 0xF8) return 4;
    if (c >= 0xF8 && c < 0xFC) return 5;
    if (c >= 0xFC) return 6;
    return 0 ;
}


static int enc_utf8_to_unicode_one(unsigned char *pInput, unsigned short *Unic)
{
    char b1, b2, b3, b4, b5, b6;

    *Unic = 0x0;
    int utfbytes = enc_get_utf8_size(*pInput);
    unsigned char *pOutput = (unsigned char *) Unic;

    switch (utfbytes)
    {
    case 0:
        *pOutput     = *pInput;
        utfbytes    += 1;
        break;
    case 2:
        b1 = *pInput;
        b2 = *(pInput + 1);
        if ((b2 & 0xc0) != 0x80)     //0xe0
            return 0;
        *pOutput     = (b1 << 6) + (b2 & 0x3F);
        *(pOutput + 1) = (b1 >> 2) & 0x07;
        break;
    case 3:
        b1 = *pInput;
        b2 = *(pInput + 1);
        b3 = *(pInput + 2);
        if (((b2 & 0xC0) != 0x80) || ((b3 & 0xC0) != 0x80))
            return 0;
        *pOutput     = (b2 << 6) + (b3 & 0x3F);
        *(pOutput + 1) = (b1 << 4) + ((b2 >> 2) & 0x0F);
        break;
    case 4:
        b1 = *pInput;
        b2 = *(pInput + 1);
        b3 = *(pInput + 2);
        b4 = *(pInput + 3);
        if (((b2 & 0xC0) != 0x80) || ((b3 & 0xC0) != 0x80)
                || ((b4 & 0xC0) != 0x80))
            return 0;
        *pOutput     = (b3 << 6) + (b4 & 0x3F);
        *(pOutput + 1) = (b2 << 4) + ((b3 >> 2) & 0x0F);
        *(pOutput + 2) = ((b1 << 2) & 0x1C)  + ((b2 >> 4) & 0x03);
        break;
    case 5:
        b1 = *pInput;
        b2 = *(pInput + 1);
        b3 = *(pInput + 2);
        b4 = *(pInput + 3);
        b5 = *(pInput + 4);
        if (((b2 & 0xC0) != 0x80) || ((b3 & 0xC0) != 0x80)
                || ((b4 & 0xC0) != 0x80) || ((b5 & 0xC0) != 0x80))
            return 0;
        *pOutput     = (b4 << 6) + (b5 & 0x3F);
        *(pOutput + 1) = (b3 << 4) + ((b4 >> 2) & 0x0F);
        *(pOutput + 2) = (b2 << 2) + ((b3 >> 4) & 0x03);
        *(pOutput + 3) = (b1 << 6);
        break;
    case 6:
        b1 = *pInput;
        b2 = *(pInput + 1);
        b3 = *(pInput + 2);
        b4 = *(pInput + 3);
        b5 = *(pInput + 4);
        b6 = *(pInput + 5);
        if (((b2 & 0xC0) != 0x80) || ((b3 & 0xC0) != 0x80)
                || ((b4 & 0xC0) != 0x80) || ((b5 & 0xC0) != 0x80)
                || ((b6 & 0xC0) != 0x80))
            return 0;
        *pOutput     = (b5 << 6) + (b6 & 0x3F);
        *(pOutput + 1) = (b5 << 4) + ((b6 >> 2) & 0x0F);
        *(pOutput + 2) = (b3 << 2) + ((b4 >> 4) & 0x03);
        *(pOutput + 3) = ((b1 << 6) & 0x40) + (b2 & 0x3F);
        break;
    default:
        return 0;
//            break;
    }

    return utfbytes;
}

uint32_t utf8decode2unicode(uint8_t *p_dst, uint32_t max_size, uint8_t *p_src, uint32_t length)
{
    uint32_t  message_index  = 0;
    uint16_t  unicode = 0;

    for (uint32_t index = 0; index < length; index ++)
    {

        //if(message_index < max_size )
        {
            uint8_t size = enc_utf8_to_unicode_one(&p_src[index], &unicode);
            if (size == 0)
            {
                break;
            }
            p_dst[message_index]   = (unicode & 0x00ff);
            p_dst[message_index + 1] = (unicode >> 8 & 0x00ff);
            index += (size - 1);
            message_index += 2;
        }
        if (message_index >= max_size)
        {
            break;
        }
    }

    return message_index;
}


static int enc_unicode_to_utf8_one(unsigned long unic, unsigned char *pOutput, int outSize)
{
    if (pOutput == NULL || outSize < 6) return 0;

    if (unic <= 0x0000007F)
    {
        // * U-00000000 - U-0000007F:  0xxxxxxx
        *pOutput     = (unic & 0x7F);
        return 1;
    }
    else if (unic >= 0x00000080 && unic <= 0x000007FF)
    {
        // * U-00000080 - U-000007FF:  110xxxxx 10xxxxxx
        *(pOutput + 1) = (unic & 0x3F) | 0x80;
        *pOutput     = ((unic >> 6) & 0x1F) | 0xC0;
        return 2;
    }
    else if (unic >= 0x00000800 && unic <= 0x0000FFFF)
    {
        // * U-00000800 - U-0000FFFF:  1110xxxx 10xxxxxx 10xxxxxx
        *(pOutput + 2) = (unic & 0x3F) | 0x80;
        *(pOutput + 1) = ((unic >>  6) & 0x3F) | 0x80;
        *pOutput     = ((unic >> 12) & 0x0F) | 0xE0;
        return 3;
    }
    else if (unic >= 0x00010000 && unic <= 0x001FFFFF)
    {
        // * U-00010000 - U-001FFFFF:  11110xxx 10xxxxxx 10xxxxxx 10xxxxxx
        *(pOutput + 3) = (unic & 0x3F) | 0x80;
        *(pOutput + 2) = ((unic >>  6) & 0x3F) | 0x80;
        *(pOutput + 1) = ((unic >> 12) & 0x3F) | 0x80;
        *pOutput     = ((unic >> 18) & 0x07) | 0xF0;
        return 4;
    }
    else if (unic >= 0x00200000 && unic <= 0x03FFFFFF)
    {
        // * U-00200000 - U-03FFFFFF:  111110xx 10xxxxxx 10xxxxxx 10xxxxxx 10xxxxxx
        *(pOutput + 4) = (unic & 0x3F) | 0x80;
        *(pOutput + 3) = ((unic >>  6) & 0x3F) | 0x80;
        *(pOutput + 2) = ((unic >> 12) & 0x3F) | 0x80;
        *(pOutput + 1) = ((unic >> 18) & 0x3F) | 0x80;
        *pOutput     = ((unic >> 24) & 0x03) | 0xF8;
        return 5;
    }
    else if (unic >= 0x04000000 && unic <= 0x7FFFFFFF)
    {
        // * U-04000000 - U-7FFFFFFF:  1111110x 10xxxxxx 10xxxxxx 10xxxxxx 10xxxxxx 10xxxxxx
        *(pOutput + 5) = (unic & 0x3F) | 0x80;
        *(pOutput + 4) = ((unic >>  6) & 0x3F) | 0x80;
        *(pOutput + 3) = ((unic >> 12) & 0x3F) | 0x80;
        *(pOutput + 2) = ((unic >> 18) & 0x3F) | 0x80;
        *(pOutput + 1) = ((unic >> 24) & 0x3F) | 0x80;
        *pOutput     = ((unic >> 30) & 0x01) | 0xFC;
        return 6;
    }

    return 0;
}

//max_size is p_dst's len, p_dst lastest element must be 0. return value is utf8 bytes.
uint32_t unicodedecode2utf8(uint8_t *p_dst, uint32_t max_size, uint8_t *p_src, uint32_t length)
{
    uint8_t get_data[6];
    uint8_t size = 0;
    unsigned long unicode = 0;
    uint32_t  message_index  = 0;

    for (uint32_t index = 0; index < length; index += 2)
    {
        unicode = p_src[index] | (p_src[index + 1] << 8);
        size  = enc_unicode_to_utf8_one(unicode, &get_data[0], 6);
        if (message_index + size >= max_size)
        {
            break;
        }

        rt_memcpy(p_dst + message_index, &get_data[0], size);

        message_index += size;
    }

    p_dst[message_index] = 0;

    return message_index;

}

