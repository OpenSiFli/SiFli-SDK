/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <rtthread.h>
#include <string.h>
#include "str_encoding_detector.h"

// BOM���ֽ�˳���ǣ����
static str_encode_t check_bom(const unsigned char *data, size_t size)
{
    if (size >= 3 && data[0] == 0xEF && data[1] == 0xBB && data[2] == 0xBF)
        return STR_ENCODE_UTF8;

    if (size >= 2)
    {
        if (data[0] == 0xFE && data[1] == 0xFF)
            return STR_ENCODE_UTF16_BE;
        if (data[0] == 0xFF && data[1] == 0xFE)
            return STR_ENCODE_UTF16_LE;
    }

    return STR_ENCODE_UNKNOWN;
}

// UTF-8ģʽ���
static int is_utf8(const unsigned char *data, size_t size)
{
    size_t i = 0;
    while (i < size)
    {
        unsigned char byte = data[i];

        // ASCII�ַ�
        if ((byte & 0x80) == 0)
        {
            i++;
            continue;
        }

        // ���ֽ��ַ�
        int bytes = 0;
        if ((byte & 0xE0) == 0xC0) bytes = 2;
        else if ((byte & 0xF0) == 0xE0) bytes = 3;
        else if ((byte & 0xF8) == 0xF0) bytes = 4;
        else return 0; // ��Ч����ʼ�ֽ�

        // �������ֽ�
        if (i + bytes > size) return 0; // �������Ķ��ֽ�����
        for (int j = 1; j < bytes; j++)
        {
            if ((data[i + j] & 0xC0) != 0x80) return 0; // �����ֽڸ�ʽ����ȷ
        }

        i += bytes;
    }
    return 1;
}

// GBKģʽ��� (�򻯰�)
static int is_gbk(const unsigned char *data, size_t size)
{
    size_t i = 0;
    while (i < size)
    {
        unsigned char byte = data[i];

        // ASCII�ַ�
        if ((byte & 0x80) == 0)
        {
            i++;
            continue;
        }

        // GBK˫�ֽ��ַ� (0x81-0xFE + 0x40-0xFE, ����0x7F)
        if (byte >= 0x81 && byte <= 0xFE)
        {
            if (i + 1 >= size) return 0; // ��������˫�ֽ�����
            unsigned char byte2 = data[i + 1];
            if ((byte2 >= 0x40 && byte2 <= 0x7E) || (byte2 >= 0x80 && byte2 <= 0xFE))
            {
                i += 2;
                continue;
            }
        }

        // ������GBKģʽ
        return 0;
    }
    return 1;
}

// UTF-16ģʽ��� (��BOM)
static str_encode_t detect_utf16_without_bom(const unsigned char *data, size_t size)
{
    if (size < 2) return STR_ENCODE_UNKNOWN;

    // ͳ��LE��BEģʽ�µ���ЧUnicode��Χ�ַ���
    int le_valid = 0, be_valid = 0;
    int le_suspicious = 0, be_suspicious = 0;

    for (size_t i = 0; i < size - 1; i += 2)
    {
        // UTF-16 LE
        unsigned short code_le = data[i] | (data[i + 1] << 8);
        // UTF-16 BE
        unsigned short code_be = data[i + 1] | (data[i] << 8);

        // ����Ƿ�����ЧUnicode��Χ�� (�ų�0xD800-0xDFFF, ���Ǵ���������)
        if ((code_le >= 0x0001 && code_le <= 0xD7FF) || (code_le >= 0xE000 && code_le <= 0xFFFF))
            le_valid++;
        else if (code_le != 0)
            le_suspicious++;

        if ((code_be >= 0x0001 && code_be <= 0xD7FF) || (code_be >= 0xE000 && code_be <= 0xFFFF))
            be_valid++;
        else if (code_be != 0)
            be_suspicious++;
    }

    // ��ͶƱ����ȷ����LE����BE
    if (le_valid > be_valid && le_suspicious < le_valid)
        return STR_ENCODE_UTF16_LE;
    else if (be_valid > le_valid && be_suspicious < be_valid)
        return STR_ENCODE_UTF16_BE;

    return STR_ENCODE_UNKNOWN;
}

// ASCII���
static int is_ascii(const unsigned char *data, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        if (data[i] & 0x80)
            return 0; // ��ASCII�ַ�
    }
    return 1;
}

// ����ַ�������
str_encode_t str_detect_encoding(const unsigned char *data, size_t size)
{
    // ���ȼ��BOM
    str_encode_t encoding = check_bom(data, size);
    if (encoding != STR_ENCODE_UNKNOWN) return encoding;

    // ����Ƿ�ΪASCII
    if (is_ascii(data, size)) return STR_ENCODE_ASCII;

    // ����Ƿ�ΪUTF-8����BOM��
    if (is_utf8(data, size)) return STR_ENCODE_UTF8;

    // ����Ƿ�ΪGBK
    if (is_gbk(data, size)) return STR_ENCODE_GBK;

    // ����Ƿ�ΪUTF-16����BOM��
    encoding = detect_utf16_without_bom(data, size);
    if (encoding != STR_ENCODE_UNKNOWN) return encoding;

    // �޷�ȷ������
    return STR_ENCODE_UNKNOWN;
}
