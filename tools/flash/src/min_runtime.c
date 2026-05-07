/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>

static void *flash_memcpy_impl(void *dst, const void *src, size_t len)
{
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;

    while (len-- > 0)
    {
        *d++ = *s++;
    }

    return dst;
}

static void *flash_memmove_impl(void *dst, const void *src, size_t len)
{
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;

    if ((d == s) || (len == 0))
    {
        return dst;
    }

    if (d < s)
    {
        while (len-- > 0)
        {
            *d++ = *s++;
        }
    }
    else
    {
        d += len;
        s += len;
        while (len-- > 0)
        {
            *--d = *--s;
        }
    }

    return dst;
}

static void *flash_memset_impl(void *dst, int ch, size_t len)
{
    unsigned char *d = (unsigned char *)dst;
    unsigned char value = (unsigned char)ch;

    while (len-- > 0)
    {
        *d++ = value;
    }

    return dst;
}

static int flash_memcmp_impl(const void *lhs, const void *rhs, size_t len)
{
    const unsigned char *a = (const unsigned char *)lhs;
    const unsigned char *b = (const unsigned char *)rhs;

    while (len-- > 0)
    {
        if (*a != *b)
        {
            return (int)*a - (int)*b;
        }
        ++a;
        ++b;
    }

    return 0;
}

static size_t flash_strlen_impl(const char *str)
{
    size_t len = 0;

    while (str[len] != '\0')
    {
        ++len;
    }

    return len;
}

static int flash_strcmp_impl(const char *lhs, const char *rhs)
{
    while ((*lhs != '\0') && (*lhs == *rhs))
    {
        ++lhs;
        ++rhs;
    }

    return (int)(unsigned char)(*lhs) - (int)(unsigned char)(*rhs);
}

static int flash_strncmp_impl(const char *lhs, const char *rhs, size_t len)
{
    while ((len > 0) && (*lhs != '\0') && (*lhs == *rhs))
    {
        ++lhs;
        ++rhs;
        --len;
    }

    if (len == 0)
    {
        return 0;
    }

    return (int)(unsigned char)(*lhs) - (int)(unsigned char)(*rhs);
}

static char *flash_strcpy_impl(char *dst, const char *src)
{
    char *ret = dst;

    while ((*dst++ = *src++) != '\0')
    {
    }

    return ret;
}

static int flash_atoi_impl(const char *str)
{
    int sign = 1;
    int value = 0;

    while ((*str == ' ') || (*str == '\t') || (*str == '\n') || (*str == '\r'))
    {
        ++str;
    }

    if (*str == '-')
    {
        sign = -1;
        ++str;
    }
    else if (*str == '+')
    {
        ++str;
    }

    while ((*str >= '0') && (*str <= '9'))
    {
        value = (value * 10) + (*str - '0');
        ++str;
    }

    return sign * value;
}

void *__wrap_memcpy(void *dst, const void *src, size_t len)
{
    return flash_memcpy_impl(dst, src, len);
}

void *__wrap_memmove(void *dst, const void *src, size_t len)
{
    return flash_memmove_impl(dst, src, len);
}

void *__wrap_memset(void *dst, int ch, size_t len)
{
    return flash_memset_impl(dst, ch, len);
}

int __wrap_memcmp(const void *lhs, const void *rhs, size_t len)
{
    return flash_memcmp_impl(lhs, rhs, len);
}

size_t __wrap_strlen(const char *str)
{
    return flash_strlen_impl(str);
}

int __wrap_strcmp(const char *lhs, const char *rhs)
{
    return flash_strcmp_impl(lhs, rhs);
}

int __wrap_strncmp(const char *lhs, const char *rhs, size_t len)
{
    return flash_strncmp_impl(lhs, rhs, len);
}

char *__wrap_strcpy(char *dst, const char *src)
{
    return flash_strcpy_impl(dst, src);
}

int __wrap_atoi(const char *str)
{
    return flash_atoi_impl(str);
}

extern __typeof(__wrap_memcpy) memcpy __attribute__((alias("__wrap_memcpy")));
extern __typeof(__wrap_memmove) memmove __attribute__((alias("__wrap_memmove")));
extern __typeof(__wrap_memset) memset __attribute__((alias("__wrap_memset")));
extern __typeof(__wrap_memcmp) memcmp __attribute__((alias("__wrap_memcmp")));
extern __typeof(__wrap_strlen) strlen __attribute__((alias("__wrap_strlen")));
extern __typeof(__wrap_strcmp) strcmp __attribute__((alias("__wrap_strcmp")));
extern __typeof(__wrap_strncmp) strncmp __attribute__((alias("__wrap_strncmp")));
extern __typeof(__wrap_strcpy) strcpy __attribute__((alias("__wrap_strcpy")));
extern __typeof(__wrap_atoi) atoi __attribute__((alias("__wrap_atoi")));

void __aeabi_memcpy(void *dst, const void *src, size_t len)
{
    flash_memcpy_impl(dst, src, len);
}

void __aeabi_memcpy4(void *dst, const void *src, size_t len)
{
    flash_memcpy_impl(dst, src, len);
}

void __aeabi_memcpy8(void *dst, const void *src, size_t len)
{
    flash_memcpy_impl(dst, src, len);
}

void __aeabi_memmove(void *dst, const void *src, size_t len)
{
    flash_memmove_impl(dst, src, len);
}

void __aeabi_memmove4(void *dst, const void *src, size_t len)
{
    flash_memmove_impl(dst, src, len);
}

void __aeabi_memmove8(void *dst, const void *src, size_t len)
{
    flash_memmove_impl(dst, src, len);
}

void __aeabi_memset(void *dst, size_t len, int ch)
{
    flash_memset_impl(dst, ch, len);
}

void __aeabi_memset4(void *dst, size_t len, int ch)
{
    flash_memset_impl(dst, ch, len);
}

void __aeabi_memset8(void *dst, size_t len, int ch)
{
    flash_memset_impl(dst, ch, len);
}

void __aeabi_memclr(void *dst, size_t len)
{
    flash_memset_impl(dst, 0, len);
}

void __aeabi_memclr4(void *dst, size_t len)
{
    flash_memset_impl(dst, 0, len);
}

void __aeabi_memclr8(void *dst, size_t len)
{
    flash_memset_impl(dst, 0, len);
}
