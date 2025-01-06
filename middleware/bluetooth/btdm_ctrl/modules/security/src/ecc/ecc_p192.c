/**
 ****************************************************************************************
 *
 * @file ecc_p192.c
 *
 * @brief ECC function definitions for P192
 *
 * Copyright (C) RivieraWaves 2009-2022
 *
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <board.h>
#include "ecc_p192.h"

#if (BT_EMB_PRESENT)
#include "co_error.h"
#include "dbg_swdiag.h"
#include "co_list.h"
#include "ke_event.h"
#include "ke_mem.h"
#include <string.h>
#include "co_utils.h"
#include "co_math.h"

#if 0
    #include "led.h"
#else
    #define led_set(a)
    #define led_reset(a)
#endif


/*
 * MACROS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */
//
#define ECC_P192_OPER_SUCCESS      0x00
#define ECC_P192_OPER_PENDING      0x01
#define ECC_P192_OPER_FAILED       0x02

#define ECC_P192_WORD_BITS4        16

#define ECC_P192_WORD_MASK2        0xffffffff
#define ECC_P192_WORD_MASK2l       0x0000ffff
#define ECC_P192_WORD_MASK2h1      0xffff8000
#define ECC_P192_WORD_MASK2h       0xffff0000
#define ECC_P192_WORD_TBIT         0x80000000

#define ECC_P192_SPLBITS(a)        ((a) & ECC_P192_WORD_MASK2l)
#define ECC_P192_SPHBITS(a)        (((a) >> ECC_P192_WORD_BITS4)& ECC_P192_WORD_MASK2l)
#define ECC_P192_SPL2HBITS(a)      (((a) << ECC_P192_WORD_BITS4)& ECC_P192_WORD_MASK2)

#define ECC_P192_NUM_192_SIGN_BIT  0x80000000
#define ECC_P192_NUM_192_BYTES     0x06
#define ECC_P192_NUM_192_BITS      32

#define ECC_P192_TYPE_NUM_192      1
#define ECC_P192_TYPE_NUM_384      2

#define ECC_P192_WNAF_WINDOW           0x03
#define ECC_P192_WNAF_MSB_BIT          (1<<ECC_P192_WNAF_WINDOW)
#define ECC_P192_WNAF_NEXT_DIGIT_BIT   (ECC_P192_WNAF_MSB_BIT<<1)
#define ECC_P192_WNAF_DIGIT_MASK       (ECC_P192_WNAF_NEXT_DIGIT_BIT-1)

#define ECC_P192_P192_PRE_COMP_POINTS  4

#define ECC_P192_DHKEY_CALC_NUM_BITS_PER_CALL  0x08



/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// 192-bit number
typedef struct ecc_num_192
{
    uint32_t Neg;
    uint32_t Number[6];
} ecc_num_192_t;

/// 384-bit number
typedef struct ecc_num_384
{
    uint32_t Neg;
    uint32_t Number[12];
} ecc_num_384_t;

/// P192 point
typedef struct ecc_p192_point
{
    ecc_num_192_t X;
    ecc_num_192_t Y;
    ecc_num_192_t Z;
} ecc_p192_point_t;

/// P-192 curve data
typedef struct ecc_p192_group_data
{
    ecc_p192_point_t    G;
    ecc_num_192_t       A;
} ecc_p192_group_data_t;

/// ECC computation data element
typedef struct ecc_p192_data
{
    /// List element for chaining in the ECC environment
    struct co_list_hdr   hdr;
    /// Callback function to execute once algorithm completes
    ecc_p192_result_cb   cb_result;
    /// Metadata information to return with result in order to retrieve execution context
    uint32_t             metainfo;

    /// P192 Computation Global Data
    ecc_p192_point_t     Pre_comp[ECC_P192_P192_PRE_COMP_POINTS];
    ecc_p192_point_t     RemotePublicKey;
    ecc_p192_point_t     computed_key;
    signed char          wnaf[196];
    uint32_t             wnaf_len;
    uint8_t              Loop_ctr;
    uint8_t              Result_at_infinity;
    uint8_t              Result_inverted;
} ecc_p192_data_t;


/// ECC environment structure
typedef struct ecc_p192_env_
{
    /// Compute queue
    struct co_list compute_queue;
} ecc_p192_env_t;


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Debug Private key - from BT standard HCI (E.7.6.4 Write Simple Pairing Debug Mode Command)
__STATIC const uint8_t ecc_p192_debug_priv_key[PRIV_KEY_192_LEN] =
{
    0xFF, 0x18, 0xA5, 0xF4, 0xEF, 0xD2, 0x5E, 0x62, 0x2B, 0x14, 0x0C, 0xCF, 0xD6, 0xF1, 0x5D, 0x00, 0x27, 0xDC, 0x8D, 0x91, 0x86, 0x5F, 0x91, 0x07
};

/// Debug Public Key - from BT standard HCI (E.7.6.4 Write Simple Pairing Debug Mode Command)
__STATIC const uint8_t ecc_p192_debug_pub_key_x[PUB_KEY_192_LEN / 2] =
{
    0xED, 0xF8, 0x25, 0x11, 0xA5, 0x9E, 0x80, 0xD2, 0x29, 0x43, 0x7E, 0xFE, 0xC3, 0x9F, 0x6F, 0x58, 0xA6, 0x21, 0x44, 0x98, 0x09, 0x70, 0x20, 0x15
};

__STATIC const uint8_t ecc_p192_debug_pub_key_y[PUB_KEY_192_LEN / 2] =
{
    0x25, 0xEA, 0xF7, 0xB9, 0x6F, 0x85, 0xCA, 0x7F, 0x85, 0xAA, 0xBB, 0x9D, 0xB5, 0xE4, 0x79, 0x9F, 0x00, 0xBD, 0xC5, 0x1B, 0xB8, 0x42, 0x9D, 0xB0
};


/// P-192 elliptic curve base point
__STATIC const uint8_t ecc_p192_base_point_x[24] =
{
    0x12, 0x10, 0xFF, 0x82, 0xFD, 0x0A, 0xFF, 0xF4, 0x00, 0x88, 0xA1, 0x43, 0xEB, 0x20, 0xBF, 0x7C, 0xF6, 0x90, 0x30, 0xB0, 0x0E, 0xA8, 0x8D, 0x18
};

__STATIC const uint8_t ecc_p192_base_point_y[24] =
{
    0x11, 0x48, 0x79, 0x1E, 0xA1, 0x77, 0xF9, 0x73, 0xD5, 0xCD, 0x24, 0x6B, 0xED, 0x11, 0x10, 0x63, 0x78, 0xDA, 0xC8, 0xFF, 0x95, 0x2B, 0x19, 0x07
};

/// SP P192 Field
__STATIC const ecc_num_192_t ecc_p192_field =
{
    0x00000000,
    { 0xffffffff, 0xffffffff, 0xfffffffe, 0xffffffff, 0xffffffff, 0xffffffff }
};

/// SP P192 Data
__STATIC const ecc_p192_group_data_t ecc_p192_data =
{
    {
        {
            0x00000000,
            {0x82FF1012, 0xF4FF0AFD, 0x43A18800, 0x7CBF20EB, 0xB03090F6, 0x188DA80E}
        },  // G(X)
        {
            0x00000000,
            {0x1e794811, 0x73f977a1, 0x6b24cdd5, 0x631011ed, 0xffc8da78, 0x07192b95}
        },  // G(Y)
        {
            0x00000000,
            {0x00000001, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000}
        }   //
    },
    {
        0x00000000,
        {0xFFFFFFFC, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
    }
};

/// SP P192 Coef B
__STATIC const ecc_num_192_t ecc_p192_coef_b =
{
    0x00000000,
    { 0xc146b9b1, 0xfeb8deec, 0x72243049, 0x0fa7e9ab, 0xe59c80e7, 0x64210519 }
};

/// SP P192 Maximum Secret Key Value - LSB first
__STATIC const uint8_t ecc_p192_max_secret_key[24] =
{
    0x18, 0x14, 0x69, 0xDA, 0xD8, 0xE4, 0x35, 0x0A, 0x1B, 0x7C, 0xEF, 0xCC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F
};

/// ECC Environment
__STATIC ecc_p192_env_t ecc_p192_env;

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This function checks if the input is zero
 *
 * @param[in] a      The input
 * @param[in] Type   Type of the input
 *
 * @return non zero if number is zero else returns 0
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n_is_zero(void *a, uint32_t type)
{
    uint16_t i;
    type *= ECC_P192_NUM_192_BYTES;
    for (i = 0; i < type; i++)
    {
        if (((ecc_num_384_t *)a)->Number[i])
            return 0;
    }
    return 1;
}


/**
 ****************************************************************************************
 * @brief This function stores the one on a given number.
 *
 * @param[in] a     The input
 * @param[in] Type  Type of the input
 ****************************************************************************************
 */
__STATIC void ecc_p192_n_one(void *a, uint32_t type)
{
    if (type == 2)
    {
        memset(a, 0, sizeof(ecc_num_384_t));
    }
    else
    {
        memset(a, 0, sizeof(ecc_num_192_t));
    }
    ((ecc_num_192_t *)a)->Number[0] = 1;
}

/*
 ****************************************************************************************
 * @brief This function returns the position of the highest bit set in a uint32_t
 *
 * @param[in] x the input word
 *
 * @return uint32_t
 *
 ****************************************************************************************
 */
__STATIC uint32_t ecc_p192_word_highest_bit_idx(uint32_t x)
{
    uint32_t r = 0;
    if (0 == x)
        return 0;
    if (x & 0xffff0000)
    {
        x >>= 16;
        r += 16;
    }
    if (x & 0x0000ff00)
    {
        x >>= 8;
        r += 8;
    }
    if (x & 0x000000f0)
    {
        x >>= 4;
        r += 4;
    }
    if (x & 0x0000000c)
    {
        x >>= 2;
        r += 2;
    }
    if (x & 0x00000002)
    {
        r += 1;
    }
    return (r + 1);
}

/*
 ****************************************************************************************
 * @brief This function adds the number of words to be added
 *
 * @param[in] r  the result of the word addition
 * @param[in] a  One of the operand for addition
 * @param[in] b  Other operand for addition
 * @param[in] n  number of uint32_t words to be added
 *
 * @return the carry after addition.
 *
 ****************************************************************************************
 */

__STATIC uint32_t ecc_p192_word_add(uint32_t *r, uint32_t *a, uint32_t *b, int n)
{
    uint32_t c, l, t;
    int i;
    if (n <= 0)
    {
        return ((uint32_t)0);
    }

    c = 0;
    for (i = 0; i < n; i++)
    {
        t = a[i];
        t = (t + c);
        c = (t < c);
        l = (t + b[i]);
        c += (l < t);
        r[i] = l;
    }
    return ((uint32_t)c);
}

/*
 ****************************************************************************************
 * @brief This function subtracts the array pf two words
 *
 * @param[in] r  the result of the word addition
 * @param[in] a  One of the operand for addition
 * @param[in] b  Other operand for addition
 * @param[in] n  number of uint32_t words to be added
 *
 * @return uint32_t the borrow
 *
 ****************************************************************************************
 */
__STATIC uint32_t ecc_p192_word_sub(uint32_t *r, uint32_t *a, uint32_t *b, int n)
{
    uint32_t t1, t2;
    int c = 0;
    int i;

    if (n == 0)
    {
        return ((uint32_t)0);
    }

    for (i = 0; i < n; i++)
    {
        t1 = a[i];
        t2 = b[i];
        r[i] = (t1 - t2 - c);
        if (t1 != t2)
        {
            c = (t1 < t2);
        }
    }
    return (c);
}

/*
 ****************************************************************************************
 * @brief This function multiplies the array of uint32_t with a int
 *
 * @param[in]  rp      The result of the multiplication
 * @param[in]  ap      The array of number to be multiplied
 * @param[in]  num     The number of words to be multiplied
 * @param[in]  w       The number to be multiplied
 *
 * @return uint32_t 0 on success , non zero on failure
 *
 ****************************************************************************************
 */

__STATIC uint32_t ecc_p192_word_mul(uint32_t *rp, uint32_t *ap, uint16_t num, uint32_t w)
{
    uint32_t carry = 0;
    uint32_t bl, bh;
    uint16_t i;

    if (num <= 0)
    {
        return ((uint32_t)0);
    }

    bl = ECC_P192_SPLBITS(w);
    bh = ECC_P192_SPHBITS(w);

    for (i = 0; i < num; i++)
    {
        uint32_t m, m1, l, h;
        h = ap[i];
        l = ECC_P192_SPLBITS(h);
        h = ECC_P192_SPHBITS(h);
        m = bh * l;
        l *= bl;
        m1 = bl * h;
        h *= bh;
        m = (m + m1);
        if (m < m1)
        {
            h += ECC_P192_SPL2HBITS((uint32_t)1);
        }
        h += ECC_P192_SPHBITS(m);
        m1 = ECC_P192_SPL2HBITS(m);
        l += m1;
        if (l < m1)
        {
            h++;
        }
        l += carry;
        if (l < carry)
        {
            h++;
        }
        carry = h;
        rp[i] = l;

    }
    return (carry);
}

/*
 ****************************************************************************************
 * @brief This function does the multiplication and the addition of the words
 *
 * @param[in] rp     The result of the multiplication
 * @param[in] ap     The array of number to be multiplied
 * @param[in] num    The number of words to be multiplied
 * @param[in] w      The number to be multiplied
 *
 * @return uint32_t 0 on success , non zero on failure
 *
 ****************************************************************************************
 */
__STATIC uint32_t ecc_p192_word_mul_add(uint32_t *rp, const uint32_t *ap, uint16_t num, uint32_t w)
{
    uint32_t i, c = 0;
    uint32_t bl, bh;

    if (num == 0)
    {
        return ((uint32_t)0);
    }

    bl = ECC_P192_SPLBITS(w);
    bh = ECC_P192_SPHBITS(w);

    for (i = 0; i < num; i++)
    {
        uint32_t l, h;
        uint32_t m, m1;
        h = ap[i];
        l = ECC_P192_SPLBITS(h);
        h = ECC_P192_SPHBITS(h);
        m = bh * l;
        l *= bl;
        m1 = bl * h;
        h *= bh;
        m = (m + m1);
        if (m < m1)
        {
            h += ECC_P192_SPL2HBITS((uint32_t)1);
        }
        h += ECC_P192_SPHBITS(m);
        m1 = ECC_P192_SPL2HBITS(m);
        l += m1;
        if (l < m1)
        {
            h++;
        }
        l = (l + c);
        if (l < c)
        {
            h++;
        }
        c = rp[i];
        l = (l + c);
        if (l < c)
        {
            h++;
        }
        c = h;
        rp[i] = l;
    }
    return (c);
}

/*
 ****************************************************************************************
 * @brief This function is used to square the number
 *
 * @param[in] r    The square of the array
 * @param[in] a    The input number to be squared
 * @param[in] n    The number of elements in the array a
 *
 * @return uint32_t 0 on success , non zero on failure
 *
 ****************************************************************************************
 */
__STATIC void ecc_p192_word_sqr(uint32_t *r, const uint32_t *a, int n)
{
    int i;
    if (n == 0)
    {
        return;
    }
    for (i = 0; i < n; i++)
    {
        uint32_t l, h, m;
        h = a[i];
        l = ECC_P192_SPLBITS(h);
        h = ECC_P192_SPHBITS(h);
        m = (l) * (h);
        l *= l;
        h *= h;
        h += (m & ECC_P192_WORD_MASK2h1) >> (ECC_P192_WORD_BITS4 - 1);
        m = (m & ECC_P192_WORD_MASK2l) << (ECC_P192_WORD_BITS4 + 1);
        l = (l + m);
        if (l < m)
        {
            h++;
        }
        r[2 * i] = l;
        r[2 * i + 1] = h;
    }
}

/*
 ****************************************************************************************
 * @brief This function is used to get the number of words in the N-192 number
 *
 * @param[in] a     The input
 * @param[in] Type  Type of the input
 *
 * @return the number of words in a number
 *
 ****************************************************************************************
 */
__STATIC uint16_t ecc_p192_n_get_words(void *a, uint32_t type)
{
    int16_t i = 0;

    type *= ECC_P192_NUM_192_BYTES;

    for (i = (type - 1); i >= 0; i--)
    {
        if ((((ecc_num_384_t *)a)->Number[i]))
        {
            break;
        }
    }
    return (i + 1);
}

/*
 ****************************************************************************************
 * @brief This function os used to make a given number zero
 *
 * @param[in] a     The input
 * @param[in] Type  Type of the input
 ****************************************************************************************
 */

__STATIC void ecc_p192_n_zero(void *a, uint32_t type)
{
    if (type == ECC_P192_TYPE_NUM_384)
    {
        memset(a, 0, sizeof(ecc_num_384_t));
    }
    else // ECC_P192_TYPE_NUM_192
    {
        memset(a, 0, sizeof(ecc_num_192_t));
    }
}

/*
 ****************************************************************************************
 * @brief This function returns the number of bits in a given number
 *
 * @param[in]  a     The input
 * @param[in]  Type  Type of the input
 *
 * @return number of bits in a given number
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n_num_bits(void *a, uint32_t type)
{
    int i = 0;
    type *= ECC_P192_NUM_192_BYTES;
    uint32_t result = 0;

    if (ecc_p192_n_is_zero(a, type))
    {
        return 0;
    }
    for (i = (type - 1); i >= 0; i--)
    {
        result = ((ecc_num_384_t *)a)->Number[i];
        if (result)
        {
            break;
        }
    }
    return ((i * ECC_P192_NUM_192_BITS) + ecc_p192_word_highest_bit_idx(result));
}


/*
 ****************************************************************************************
 * @brief This function is used to check if the input number is one
 *
 * @param[in]  a     The input
 * @param[in]  Type  Type of the input
 *
 * @return nonzero if number is one else zero.
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n_is_one(void *a, uint32_t type)
{
    uint16_t i;
    type *= ECC_P192_NUM_192_BYTES;
    for (i = 1; i < type; i++)
    {
        if (((ecc_num_384_t *)a)->Number[i])
            return 0;
    }
    if (((ecc_num_384_t *)a)->Number[0] != 1)
    {
        return 0;
    }
    return 1;
}

/*
 ****************************************************************************************
 * @brief This function copies the number to other number.
 *
 * @param[in] dest  The destination of copy
 * @param[in] src   The source of copy
 * @param[in] Type  Type of the input
 *
 * @return non zero on success and vice-versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n_copy(void *dest, void *src, uint32_t type)
{
    if (dest == src)
    {
        return 1;
    }
    else
    {
        if (type == 2)
        {
            memset(dest, 0, sizeof(ecc_num_384_t));
            memcpy(dest, src, sizeof(ecc_num_384_t));
        }
        else
        {
            memset(dest, 0, sizeof(ecc_num_192_t));
            memcpy(dest, src, sizeof(ecc_num_192_t));
        }
    }
    return 1;
}

/*
 ****************************************************************************************
 * @brief This function checks if the given bit in the number is set or clear
 *
 * @param[in] a       The input
 * @param[in] bit     The bit number to be checked for value
 * @param[in] Type    The type of the number
 *
 * @return non-zero if bit is set and vice-versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n_is_set(void *a, uint16_t bit, uint32_t type)
{
    uint16_t word;
    ecc_num_384_t *tmp;
    tmp = (ecc_num_384_t *)a;
    word = bit / ECC_P192_NUM_192_BITS;
    bit = bit - (word * ECC_P192_NUM_192_BITS);

    if (word >= (ECC_P192_NUM_192_BYTES * type))
    {
        return 0;
    }
    else
    {
        return (tmp->Number[word] & ((uint32_t)1) << bit) ? 1 : 0;
    }
}

/*
 ****************************************************************************************
 * @brief This functions does the unsigned comparison of two NUM192 numbers
 *
 * @param[in] ECC_P192_NUM_192* a: One of the operand for comparison
 * @param[in] ECC_P192_NUM_192* b: other operand for comparison
 *
 * @return non zero if not equal else zero
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_ucmp(ecc_num_192_t *a, ecc_num_192_t  *b)
{
    int i;
    uint32_t t1, t2;

    for (i = (ECC_P192_NUM_192_BYTES - 1); i >= 0; i--)
    {
        t1 = a->Number[i];
        t2 = b->Number[i];
        if (t1 != t2)
            return ((t1 > t2) ? 1 : -1);
    }
    return (0);
}

/*
 ****************************************************************************************
 * @brief This functions does the unsigned comparison of NUM192 and NUM384
 *        numbers
 *
 * @param[in] ecc_num_192_t* a: One of the operand for comparison
 * @param[in] ecc_num_384_t* b: other operand for comparison
 *
 * @return non zero if not equal else zero
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_384_ucmp(ecc_num_384_t *a, ecc_num_192_t *b)
{
    int la = ecc_p192_n_num_bits((void *)a, ECC_P192_TYPE_NUM_384);
    if (la > 192)
    {
        return 1;
    }
    return ecc_p192_n192_ucmp((ecc_num_192_t *)a, b);
}

/*
 ****************************************************************************************
 * @brief This function does the comparison of the signed numbers
 *
 * @param[in] a One of the operand for comparison
 * @param[in] b Other operand for comparison
 *
 * @return non zero if not equal else zero
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_384_cmp(ecc_num_384_t *a, ecc_num_192_t *b)
{
    int i;
    if (a->Neg != b->Neg)
    {
        if (a->Neg)
        {
            return (-1);
        }
        else
        {
            return (1);
        }
    }

    i = ecc_p192_n192_384_ucmp(a, b);
    if (a->Neg)
    {
        return -i;
    }
    else
    {
        return i;
    }
}

/*
 ****************************************************************************************
 * @brief This function subtracts two unsigned numbers
 *
 * @param[in] r      The result of subtraction
 * @param[in] a      The operand for the subtraction
 * @param[in] b      The operand for the subtraction
 *
 * @return Borrow
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_usub(ecc_num_192_t *r, ecc_num_192_t *a, ecc_num_192_t *b)
{
    int c = 0;
    ecc_p192_n_zero(r, ECC_P192_TYPE_NUM_192);
    c = ecc_p192_word_sub(r->Number, a->Number, b->Number, ECC_P192_NUM_192_BYTES);
    if (c)
    {
        return 0;
    }
    return 1;
}

/*
 ****************************************************************************************
 * @brief This function performs the addition of two 192 bit numbers
 *
 * @param[in] r    The result of addition
 * @param[in] a    The operand for the addition
 * @param[in] b    The operand for the addition
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_uadd(ecc_num_384_t *r, ecc_num_192_t *a, ecc_num_192_t *b)
{
    uint32_t c = 0;

    ecc_p192_n_zero(r, ECC_P192_TYPE_NUM_384);
    c = ecc_p192_word_add(r->Number, a->Number, b->Number, ECC_P192_NUM_192_BYTES);
    r->Number[ECC_P192_NUM_192_BYTES] = c;
    return 1;
}

/*
 ****************************************************************************************
 * @brief This function does the subtraction of two signed 192 bit numbers
 *
 * @param[in] r       The result of subtraction
 * @param[in] a       The operand for the subtraction
 * @param[in] b       The operand for the subtraction
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_sub(ecc_num_384_t *r, ecc_num_192_t *a, ecc_num_192_t *b)
{
    int add = 0, neg = 0;
    ecc_num_192_t *tmp;

    if (a->Neg)
    {
        if (b->Neg)
        {
            tmp = a;
            a = b;
            b = tmp;
        }
        else
        {
            add = 1;
            neg = 1;
        }
    }
    else
    {
        if (b->Neg)
        {
            add = 1;
            neg = 0;
        }
    }

    if (add)
    {
        if (!ecc_p192_n192_uadd(r, a, b))
        {
            return (0);
        }
        r->Neg = neg;
        return (1);
    }

    if (ecc_p192_n192_ucmp(a, b) < 0)
    {
        if (!ecc_p192_n192_usub((ecc_num_192_t *)r, b, a))
        {
            return (0);
        }
        r->Neg = 1;
    }
    else
    {
        if (!ecc_p192_n192_usub((ecc_num_192_t *)r, a, b))
        {
            return (0);
        }
        r->Neg = 0;
    }
    return (1);
}

/*
 ****************************************************************************************
 * @brief This function does the addition of two signed 192 bit numbers
 *
 * @param[in] r        The result of addition
 * @param[in] a        The operand for the addition
 * @param[in] b        The operand for the addition
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_add(ecc_num_384_t *r, ecc_num_192_t *a, ecc_num_192_t *b)
{
    ecc_num_192_t *tmp;
    int a_Neg = a->Neg, ret;

    if (a_Neg ^ b->Neg)
    {
        if (a_Neg)
        {
            tmp = a;
            a = b;
            b = tmp;
        }

        if (ecc_p192_n192_ucmp(a, b) < 0)
        {
            if (!ecc_p192_n192_usub((ecc_num_192_t *)r, b, a))
            {
                return (0);
            }
            r->Neg = 1;
        }
        else
        {
            if (!ecc_p192_n192_usub((ecc_num_192_t *)r, a, b))
            {
                return (0);
            }
            r->Neg = 0;
        }
        return (1);
    }

    ret = ecc_p192_n192_uadd(r, a, b);
    r->Neg = a_Neg;
    return ret;
}


/*
 ****************************************************************************************
 * @brief This function performs the left shift by one for 192 bit number mul by 2
 *
 * @param[in] r      The result of shifting
 * @param[in] a      The operand for the shifting
 *
 * @return non-zero on success and vice-versa.
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_lshift1(ecc_num_384_t *r, ecc_num_192_t *a)
{
    uint32_t *ap, *rp, t, c;
    int i;

    if (r != (ecc_num_384_t *)a)
    {
        ecc_p192_n_zero(r, ECC_P192_TYPE_NUM_384);
        ecc_p192_n_copy(r, a, ECC_P192_TYPE_NUM_192);
    }
    ap = a->Number;
    rp = r->Number;
    c = 0;

    for (i = 0; i < ECC_P192_NUM_192_BYTES; i++)
    {
        t = *(ap++);
        *(rp++) = ((t << 1) | c);
        c = (t & ECC_P192_NUM_192_SIGN_BIT) ? 1 : 0;
    }
    if (c)
    {
        *rp = 1;
    }
    return (1);
}

/*
 ****************************************************************************************
 * @brief This function performs the right shift by one
 *
 * @param[in] r        The result of shifting
 * @param[in] a        The operand for the shifting
 * @param[in] Type     Type of the operand
 *
 * @return non-zero on success and vice-versa.
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_rshift1(void *r, void *a, uint32_t type)
{
    uint32_t *ap, *rp, t, c;
    int i;

    if (ecc_p192_n_is_zero(a, type))
    {
        ecc_p192_n_zero(r, type);
        return (1);
    }
    if (a != r)
    {
        ((ecc_num_384_t *)r)->Neg = ((ecc_num_384_t *)a)->Neg;
    }

    ap = ((ecc_num_384_t *)a)->Number;
    rp = ((ecc_num_384_t *)r)->Number;
    c = 0;
    type *= ECC_P192_NUM_192_BYTES;
    type -= 1;
    for (i = type; i >= 0; i--)
    {
        t = ap[i];
        rp[i] = (t >> 1) | c;
        c = (t & 1) ? ECC_P192_NUM_192_SIGN_BIT : 0;
    }
    return (1);
}


/*
 ****************************************************************************************
 * @brief This function performs the left shift operation.
 *
 * @param[in] r    The result
 * @param[in] a    The operand
 * @param[in] n    The number of bits to be shifted
 * @param[in] type Type of the number
 *
 * @return non-zero on success and vice-versa.
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_lshift(ecc_num_384_t *r, void *a, uint16_t n, uint32_t type)
{
    int i, nw, lb, rb;
    uint32_t *t, *f;
    uint32_t l;

    type *= ECC_P192_NUM_192_BYTES;
    if (r != a)
    {
        ecc_p192_n_zero(r, ECC_P192_TYPE_NUM_384);
    }


    if (n > (192 * type))
    {
        ecc_p192_n_zero(r, ECC_P192_TYPE_NUM_384);
        return 1;
    }

    r->Neg = ((ecc_num_384_t *)a)->Neg;

    nw = n / ECC_P192_NUM_192_BITS;
    lb = n - (nw * ECC_P192_NUM_192_BITS);
    rb = ECC_P192_NUM_192_BITS - lb;
    f = ((ecc_num_384_t *)a)->Number;
    t = r->Number;

    if (lb == 0)
    {
        for (i = (type - 1); i >= 0; i--)
        {
            t[nw + i] = f[i];
        }
    }
    else
    {
        for (i = (type - 1); i >= 0; i--)
        {
            l = f[i];
            t[nw + i + 1] |= (l >> rb);
            t[nw + i] = (l << lb);
        }
    }
    memset(t, 0, nw * sizeof(t[0]));
    return (1);
}


/*
 ****************************************************************************************
 * @brief This function performs the right shift operation
 *
 * @param[in] r       The result
 * @param[in] a       The operand
 * @param[in] n       The number of bits to be shifted
 *
 * @return non-zero on success and vice-versa.
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_rshift(ecc_num_192_t *r, ecc_num_192_t *a, int n)
{
    int i, j, nw, lb, rb;
    uint32_t *t, *f;
    uint32_t l, tmp;

    nw = n / ECC_P192_NUM_192_BITS;
    rb = n - (nw * ECC_P192_NUM_192_BITS);
    lb = ECC_P192_NUM_192_BITS - rb;

    if (n  >  192)
    {
        ecc_p192_n_zero(r, ECC_P192_TYPE_NUM_192);
        return (1);
    }

    if (r != a)
    {
        r->Neg = a->Neg;
    }
    else
    {
        if (n == 0)
            return 1;
    }

    f = &(a->Number[nw]);
    t = r->Number;
    j = ECC_P192_NUM_192_BYTES - nw;

    if (rb == 0)
    {
        for (i = j; i != 0; i--)
            *(t++) = *(f++);
    }
    else
    {
        l = *(f++);
        for (i = j - 1; i != 0; i--)
        {
            tmp = (l >> rb);
            l = *(f++);
            *(t++) = (tmp | (l << lb));
        }
        *(t++) = (l >> rb);
    }
    return (1);
}

/*
 ****************************************************************************************
 * @brief This function multiplies two ecc_num_192_tnumbers.
 *
 * @param[in] r      The result
 * @param[in] a      The operand for the Mul
 * @param[in] b      The operand for the Mul
 *
 * @return non-zero on success and vice-versa.
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_mul(ecc_num_384_t *r, ecc_num_192_t *a, ecc_num_192_t *b)
{
    uint16_t i;
    uint32_t *rr;
    uint16_t  na, nb;

    na = ecc_p192_n_get_words(a, ECC_P192_TYPE_NUM_192);
    nb = ecc_p192_n_get_words(b, ECC_P192_TYPE_NUM_192);
    ecc_p192_n_zero(r, ECC_P192_TYPE_NUM_384);
    if ((!na) || (!nb))
    {
        return 1;
    }

    if (na < nb)
    {
        i = na;
        na = nb;
        nb = i;

        rr = (uint32_t *)a;
        a = b;
        b = (ecc_num_192_t *) rr;

    }
    rr = &(r->Number[na]);

    rr[0] = ecc_p192_word_mul(r->Number, a->Number, na, b->Number[0]);

    for (i = 0; i < (nb - 1); i++)
    {
        rr[i + 1] = ecc_p192_word_mul_add(&(r->Number[i + 1]), (a->Number), na, b->Number[i + 1]);
    }
    return 1;
}

/*
 ****************************************************************************************
 * @brief This function squares two ecc_num_192_t numbers.
 *
 * @param[in] ecc_num_384_t* r,The result
 * @param[in] ecc_num_192_t* a, The operand for the Sqr
 *
 * @return non-zero on success and vice-versa.
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_sqr(ecc_num_384_t *r, ecc_num_192_t *a)
{
    ecc_num_384_t tmp;
    int i, j, max, n;
    uint32_t *ap;
    uint32_t *rp;

    r->Neg = 0;
    ecc_p192_n_zero(&tmp, ECC_P192_TYPE_NUM_384);
    n = ecc_p192_n_get_words(a, ECC_P192_TYPE_NUM_192);
    max = n << 1;
    ap = a->Number;
    rp = r->Number;
    rp[0] = rp[max - 1] = 0;
    rp++;
    j = n;

    if (--j > 0)
    {
        ap++;
        rp[j] = ecc_p192_word_mul(rp, ap, (uint16_t) j, ap[-1]);
        rp += 2;
    }

    for (i = n - 2; i > 0; i--)
    {
        j--;
        ap++;
        rp[j] = ecc_p192_word_mul_add(rp, ap, (uint16_t) j, ap[-1]);
        rp += 2;
    }

    ecc_p192_word_add(r->Number, r->Number, r->Number, max);
    ecc_p192_word_sqr(tmp.Number, a->Number, n);
    ecc_p192_word_add(r->Number, r->Number, tmp.Number, max);
    return 1;
}

/*
 ****************************************************************************************
 * @brief This function does the subtraction of two signed 192 bit numbers from
 *        384 bit number
 *
 * @param[in] r    The result of subtraction
 * @param[in] a    The operand for the subtraction
 * @param[in] b    The operand for the subtraction
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n384_192_sub(ecc_num_192_t *r, ecc_num_384_t *a, ecc_num_192_t *b)
{
    int c = 0;
    c = ecc_p192_word_sub(r->Number, a->Number, b->Number, ECC_P192_NUM_192_BYTES);
    if (c)
    {
        if (a->Number[ECC_P192_NUM_192_BYTES])
        {
            a->Number[ECC_P192_NUM_192_BYTES] -= c;
            if (a->Number[ECC_P192_NUM_192_BYTES])
            {
                return 0;
            }
        }
    }
    return 1;

}

/*
 ****************************************************************************************
 * @brief This function does the modular addition of two 192 both number.
 *
 * @param[in] r     The result of addition
 * @param[in] a     The operand for the addition
 * @param[in] b     The operand for the addition
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_mod_add(ecc_num_192_t *r, ecc_num_192_t *a, ecc_num_192_t *b)
{
    ecc_num_384_t tmp;

    memset((uint8_t *)&tmp, 0, sizeof(ecc_num_384_t));

    if (!ecc_p192_n192_uadd(&tmp, a, b))
    {
        return 0;
    }

    if (ecc_p192_n192_384_ucmp(&tmp, (ecc_num_192_t *)&ecc_p192_field) >= 0)
    {
        return ecc_p192_n384_192_sub(r, &tmp, (ecc_num_192_t *)&ecc_p192_field);
    }
    else
    {
        return ecc_p192_n_copy(r, &tmp, ECC_P192_TYPE_NUM_192);
    }
}

/*
 ****************************************************************************************
 * @brief This function performs the modular sub.
 *
 * @param[in] r  The result of sub
 * @param[in] a  The operand for the sub
 * @param[in] b  The operand for the sub
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_mod_sub(ecc_num_192_t *r, ecc_num_192_t *a, ecc_num_192_t *b)
{
    ecc_num_384_t tmp;

    memset((uint8_t *)&tmp, 0, sizeof(ecc_num_384_t));

    if (!ecc_p192_n192_sub(&tmp, a, b)) return 0;
    if (tmp.Neg)
    {
        return (ecc_p192_n192_add((ecc_num_384_t *)r, (ecc_num_192_t *)&tmp,
                                  (ecc_num_192_t *)&ecc_p192_field));
    }
    else
    {
        return ecc_p192_n_copy(r, &tmp, ECC_P192_TYPE_NUM_192);
    }
}

/**
 ****************************************************************************************
 * @brief This function is used to find the Modulus with the P-192 field.
 *
 * @param[in] r,The result
 * @param[in] a, The operand for the Modulus
 *
 * @return non-zero on success and vice-versa.
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_n_mod(ecc_num_192_t *r, ecc_num_384_t *a)
{
    int i;
    uint32_t *res;
    ecc_num_384_t tmp1;
    ecc_num_192_t tmp2;

    memset((uint8_t *)&tmp1, 0, sizeof(tmp1));
    memset((uint8_t *)&tmp2, 0, sizeof(tmp2));

    r->Neg = 0;
    if (ecc_p192_n_is_zero(a, ECC_P192_TYPE_NUM_384))
    {
        return ecc_p192_n_copy(r, a, ECC_P192_TYPE_NUM_384);
    }

    i = ecc_p192_n192_384_cmp(a, (ecc_num_192_t *)&ecc_p192_field);
    if (i == 0)
    {
        ecc_p192_n_zero(r, ECC_P192_TYPE_NUM_192);
        return 1;
    }
    else if (i < 0)
    {
        return ecc_p192_n_copy(r, a, ECC_P192_TYPE_NUM_192);
    }
    i = 0;
    if (!ecc_p192_n_copy(&tmp1, a, ECC_P192_TYPE_NUM_192))
    {
        return 0;
    }
    res = r->Number;

    ecc_p192_n_zero(&tmp2, ECC_P192_TYPE_NUM_192);
    tmp2.Number[0] = a->Number[6];
    tmp2.Number[1] = a->Number[7];
    tmp2.Number[2] = a->Number[6];
    tmp2.Number[3] = a->Number[7];

    if (ecc_p192_word_add(res, tmp1.Number, tmp2.Number, ECC_P192_NUM_192_BYTES))
    {
        i++;
    }

    ecc_p192_n_zero(&tmp2, ECC_P192_TYPE_NUM_192);
    tmp2.Number[2] = a->Number[8];
    tmp2.Number[3] = a->Number[9];
    tmp2.Number[4] = a->Number[8];
    tmp2.Number[5] = a->Number[9];

    if (ecc_p192_word_add(res, res, tmp2.Number, ECC_P192_NUM_192_BYTES))
    {
        i++;
    }

    ecc_p192_n_zero(&tmp2, ECC_P192_TYPE_NUM_192);
    tmp2.Number[0] = a->Number[10];
    tmp2.Number[1] = a->Number[11];
    tmp2.Number[2] = a->Number[10];
    tmp2.Number[3] = a->Number[11];
    tmp2.Number[4] = a->Number[10];
    tmp2.Number[5] = a->Number[11];

    if (ecc_p192_word_add(res, res, tmp2.Number, ECC_P192_NUM_192_BYTES))
    {
        i++;
    }

    while (i)
    {
        ecc_p192_word_sub(res, res, (uint32_t *)ecc_p192_field.Number, ECC_P192_NUM_192_BYTES);
        --i;
    }
    if (ecc_p192_n192_ucmp(r, (ecc_num_192_t *)&ecc_p192_field) >= 0)
    {
        ecc_p192_word_sub(res, res, (uint32_t *)ecc_p192_field.Number, ECC_P192_NUM_192_BYTES);
    }

    return 1;
}

/*
 ****************************************************************************************
 * @brief This function performs the modular mul
 *
 * @param[in] r    The result of MUL
 * @param[in] a    The operand for the MUL
 * @param[in] b    The operand for the MUL
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_mod_mul(ecc_num_192_t *r, ecc_num_192_t *a, ecc_num_192_t *b)
{
    ecc_num_384_t t;

    memset((uint8_t *)&t, 0, sizeof(t));

    if (a == b)
    {
        if (!ecc_p192_n192_sqr(&t, a))
            return 0;
    }
    else
    {
        if (!ecc_p192_n192_mul(&t, a, b))
        {
            return 0;
        }
    }
    if (!ecc_p192_n192_n_mod(r, &t))
    {
        return 0;
    }
    return 1;
}


/*
 ****************************************************************************************
 * @brief This function performs the modular sqr
 *
 * @param[in] r     The result of sqr
 * @param[in] a     The operand for the sqr
 * @param[in] b     The operand for the sqr
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_mod_sqr(ecc_num_192_t *r, ecc_num_192_t *a)
{
    ecc_num_384_t tmp;
    ecc_p192_n_zero(&tmp, ECC_P192_TYPE_NUM_384);
    if (!ecc_p192_n192_sqr(&tmp, a))
    {
        return 0;
    }
    return ecc_p192_n192_n_mod(r, &tmp);
}

/*
 ****************************************************************************************
 * @brief This function performs the modular left shift or mul by 2
 *
 * @param[in] r      The result of sqr
 * @param[in] a      The operand for the sqr
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_mod_lshift1_quick(ecc_num_192_t *r, ecc_num_192_t *a)
{
    ecc_num_384_t tmp;
    ecc_p192_n_zero(&tmp, ECC_P192_TYPE_NUM_384);
    if (!ecc_p192_n192_lshift1(&tmp, a))
    {
        return 0;
    }
    if (ecc_p192_n192_384_cmp(&tmp, (ecc_num_192_t *)&ecc_p192_field) >= 0)
    {
        return ecc_p192_n384_192_sub(r, &tmp, (ecc_num_192_t *)&ecc_p192_field);
    }
    else
    {
        return ecc_p192_n_copy(r, &tmp, ECC_P192_TYPE_NUM_192);
    }
}

/*
 ****************************************************************************************
 * @brief This function performs the modular left shift.
 *
 * @param[in] r       The result of sqr
 * @param[in] a       The operand for the sqr
 * @param[in] n       The number of bits to be shifted
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_mod_lshift_quick(ecc_num_192_t *r, ecc_num_192_t *a, int n)
{
    ecc_num_384_t tmp;

    ecc_p192_n_zero(&tmp, ECC_P192_TYPE_NUM_384);
    ecc_p192_n_copy(&tmp, a, ECC_P192_TYPE_NUM_192);

    while (n > 0)
    {
        int max_shift;

        max_shift = ECC_P192_NUM_192_BITS * ECC_P192_NUM_192_BYTES - ecc_p192_n_num_bits(&tmp, ECC_P192_TYPE_NUM_192);

        if (max_shift < 0)
        {
            return 0;
        }

        if (max_shift > n)
        {
            max_shift = n;
        }

        if (max_shift)
        {
            if (!ecc_p192_n192_lshift(&tmp, &tmp, max_shift, ECC_P192_TYPE_NUM_192))
            {
                return 0;
            }
            n -= max_shift;
        }
        else
        {
            if (!ecc_p192_n192_lshift1(&tmp, (ecc_num_192_t *)&tmp))
            {
                return 0;
            }
            --n;
        }
        if (ecc_p192_n192_384_cmp(&tmp, (ecc_num_192_t *)&ecc_p192_field) >= 0)
        {
            if (!ecc_p192_n384_192_sub((ecc_num_192_t *)&tmp,
                                       &tmp,
                                       (ecc_num_192_t *)&ecc_p192_field))
            {
                return 0;
            }
        }
    }
    ecc_p192_n_copy(r, &tmp, ECC_P192_TYPE_NUM_192);
    return 1;
}


/*
 ****************************************************************************************
 * @brief This function returns the inverse of a number i.e. solves ax=1.
 *
 * @param[in] r         The result of sqr
 * @param[in] a         The operand for the sqr
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_mod_inverse(ecc_num_192_t *in, ecc_num_192_t *a)
{
    ecc_num_192_t X, Y;
    ecc_num_192_t A;
    ecc_num_192_t B;
    uint16_t shift;

    memset((uint8_t *)&A, 0, sizeof(A));
    memset((uint8_t *)&B, 0, sizeof(B));

    ecc_p192_n_one(&X, ECC_P192_TYPE_NUM_192);
    ecc_p192_n_zero(&Y, ECC_P192_TYPE_NUM_192);

    if (!ecc_p192_n_copy(&B, a, ECC_P192_TYPE_NUM_192))
    {
        return 0;
    }
    if (!ecc_p192_n_copy(&A, (ecc_num_192_t *)&ecc_p192_field, ECC_P192_TYPE_NUM_192))
    {
        return 0;
    }

    A.Neg = 0;

    if (B.Neg || (ecc_p192_n192_ucmp(&B, &A) >= 0))
    {
        ecc_num_384_t tmp;
        ecc_p192_n_zero(&tmp, ECC_P192_TYPE_NUM_384);
        ecc_p192_n_copy(&tmp, &B, ECC_P192_TYPE_NUM_192);
        if (!ecc_p192_n192_n_mod(&B, (ecc_num_384_t *)&tmp))
        {
            return 0;
        }
    }
    while (!ecc_p192_n_is_zero(&B, ECC_P192_TYPE_NUM_192))
    {
        shift = 0;
        while (!ecc_p192_n_is_set(&B, shift, ECC_P192_TYPE_NUM_192))
        {
            shift++;
            if (X.Number[0] & 0x01)
            {
                ecc_num_384_t tmp;
                if (!ecc_p192_n192_add(&tmp, &X, (ecc_num_192_t *)&ecc_p192_field))
                {
                    return 0;
                }
                if (!ecc_p192_n192_rshift1(&tmp, &tmp, ECC_P192_TYPE_NUM_384))
                {
                    return 0;
                }
                if (!ecc_p192_n192_n_mod(&X, &tmp))
                {
                    return 0;
                }
            }
            else
            {
                if (!ecc_p192_n192_rshift1(&X, &X, ECC_P192_TYPE_NUM_192))
                {
                    return 0;
                }
            }
        }
        if (shift > 0)
        {
            if (!ecc_p192_n192_rshift(&B, &B, shift))
            {
                return 0;
            }
        }

        shift = 0;
        while (!ecc_p192_n_is_set(&A, shift, ECC_P192_TYPE_NUM_192))
        {
            shift++;

            if (Y.Number[0] & 0x01)
            {
                ecc_num_384_t tmp;
                if (!ecc_p192_n192_add(&tmp, &Y, (ecc_num_192_t *)&ecc_p192_field))
                {
                    return 0;
                }
                if (!ecc_p192_n192_rshift1(&tmp, &tmp, ECC_P192_TYPE_NUM_384))
                {
                    return 0;
                }
                if (!ecc_p192_n192_n_mod(&Y, &tmp))
                {
                    return 0;
                }
            }
            else
            {
                if (!ecc_p192_n192_rshift1(&Y, &Y, ECC_P192_TYPE_NUM_192))
                {
                    return 0;
                }
            }
        }
        if (shift > 0)
        {
            if (!ecc_p192_n192_rshift(&A, &A, shift))
            {
                return 0;
            }
        }

        if (ecc_p192_n192_ucmp(&B, &A) >= 0)
        {
            if (!ecc_p192_n192_mod_sub(&X, &X, &Y))
            {
                return 0;
            }
            if (!ecc_p192_n192_mod_sub(&B, &B, &A))
            {
                return 0;
            }
        }
        else
        {
            if (!ecc_p192_n192_mod_sub(&Y, &Y, &X))
            {
                return 0;
            }
            if (!ecc_p192_n192_mod_sub(&A, &A, &B))
            {
                return 0;
            }
        }
    }
    if (ecc_p192_n_is_one(&A, ECC_P192_TYPE_NUM_192))
    {
        if (!ecc_p192_n_copy(in, &Y, ECC_P192_TYPE_NUM_192))
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
    return 1;
}

/*
 ****************************************************************************************
 * @brief This function checks of the point is at infinity
 *
 * @param[in] p        Input
 *
 * @return non zero if point is at infinity and vice-versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_is_point_at_inf(ecc_p192_point_t *p)
{
    return ecc_p192_n_is_zero(&p->Z, ECC_P192_TYPE_NUM_192);
}

// Check if public key is valid
__STATIC_NO_INLINE bool ecc_p192_is_point_valid(ecc_p192_point_t *p)
{
    // Check to see if X and Y point exist on the ECDH curve.
    // This is only required for Public Keys recieved from the peer.
    // Y^2 = X^3 - 3X  + b

    ecc_num_192_t Y_square;
    ecc_num_192_t X_square;
    ecc_num_192_t X_cube;
    ecc_num_192_t double_X;
    ecc_num_192_t tripple_X;
    ecc_num_192_t interim_result1;
    ecc_num_192_t interim_result2;

    memset(&Y_square.Number[0], 0, sizeof(Y_square.Number));
    Y_square.Neg = 0;
    memset(&X_square.Number[0], 0, sizeof(X_square.Number));
    X_square.Neg = 0;
    memset(&X_cube.Number[0], 0, sizeof(X_cube.Number));
    X_cube.Neg = 0;
    memset(&double_X.Number[0], 0, sizeof(double_X.Number));
    double_X.Neg = 0;
    memset(&tripple_X.Number[0], 0, sizeof(tripple_X.Number));
    tripple_X.Neg = 0;
    memset(&interim_result1.Number[0], 0, sizeof(interim_result1.Number));
    interim_result1.Neg = 0;
    memset(&interim_result2.Number[0], 0, sizeof(interim_result2.Number));
    interim_result2.Neg = 0;

    // First Determine Y^2
    ecc_p192_n192_mod_mul(&Y_square, &p->Y, &p->Y);

    // Now Determine X^3
    ecc_p192_n192_mod_mul(&X_square, &p->X, &p->X);
    ecc_p192_n192_mod_mul(&X_cube, &p->X, &X_square);

    // Determine 3x
    ecc_p192_n192_mod_add(&double_X, &p->X, &p->X);
    ecc_p192_n192_mod_add(&tripple_X, &p->X, &double_X);

    // X^3 -3x
    ecc_p192_n192_mod_sub(&interim_result1, &X_cube, &tripple_X);

    // interim_result + coeficient_b
    ecc_p192_n192_mod_add(&interim_result2, &interim_result1, (ecc_num_192_t *) &ecc_p192_coef_b);

    // Now check if Y^2 = interim_result2 (x^3 -3x + b)
    return (memcmp(&Y_square.Number[0], &interim_result2.Number[0], 24) == 0);
}


/**
 ****************************************************************************************
 * @brief This function converts the uint8_t Array to uint32_t Array
 *
 * @param[out] Output              Output array
 * @param[in]  Input               Input array
 * @param[in]  NumWords            Number of words to convert
 ****************************************************************************************
 */
__STATIC void ecc_p192_convert_u8_to_u32(uint32_t *OutPut, const uint8_t *Input, uint32_t NumWords)
{
    uint8_t ctr = 0, InputBytePos;
    uint32_t Temp;

    while (ctr < NumWords)
    {
        Temp = 0;
        InputBytePos = (ctr << 2);
        Temp = Input[InputBytePos + 3] & 0xff;
        Temp <<= 8;
        Temp |= Input[InputBytePos + 2] & 0xff;
        Temp <<= 8;
        Temp |= Input[InputBytePos + 1] & 0xff;
        Temp <<= 8;
        Temp |= Input[InputBytePos + 0] & 0xff;

        OutPut[ctr] = Temp;
        ctr++;
    }
}

/*
 ****************************************************************************************
 * @brief This function converts the uint8_t Array to uint32_t Array
 *
 * @param[out] Output                Output uint8_t size value
 * @param[in]  Input                 Input uint32_t size value
 * @param[in]  NumByte               Number of bytes
 *
 ****************************************************************************************
 */
__STATIC void ecc_p192_convert_u32_to_u8(uint8_t *OutPut, const uint32_t *Input, uint32_t NumByte)
{
    uint8_t ctr = 0, InputWordPos;
    uint32_t Temp;

    while (ctr < NumByte)
    {
        InputWordPos = (ctr >> 2);
        memcpy(&Temp, &(Input[InputWordPos]), sizeof(uint32_t));
        OutPut[ctr] = Temp & 0xff;
        Temp >>= 8;
        OutPut[ctr + 1] = Temp & 0xff;
        Temp >>= 8;
        OutPut[ctr + 2] = Temp & 0xff;
        Temp >>= 8;
        OutPut[ctr + 3] = Temp & 0xff;
        ctr += 4;
    }
}



/**
 ****************************************************************************************
 * @brief This function returns converts the number into naf form with window
 *        size of 4
 *
 * @param[in] wnaf     returns wnaf form
 * @param[in] input    the input num to be converted
 * @param[in] ret_len  the length of the wnaf form number
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_n192_convert_wnaf(signed char *r, ecc_num_192_t *input, uint32_t *ret_len)
{

    int window_val;
    uint32_t len, j;

    len = ecc_p192_n_num_bits(input, ECC_P192_TYPE_NUM_192);
    window_val = input->Number[0] & ECC_P192_WNAF_DIGIT_MASK;
    j = 0;
    while ((window_val != 0) || (j + ECC_P192_WNAF_WINDOW + 1 < len))
    {

        int digit = 0;
        if (window_val & 1)
        {
            if (window_val & ECC_P192_WNAF_MSB_BIT)
            {
                digit = window_val - ECC_P192_WNAF_NEXT_DIGIT_BIT;

                if (j + ECC_P192_WNAF_WINDOW + 1 >= len)
                {
                    digit = window_val & (ECC_P192_WNAF_DIGIT_MASK >> 1);
                }
            }
            else
            {
                digit = window_val;
            }

            if (digit <= -ECC_P192_WNAF_MSB_BIT ||
                    digit >= ECC_P192_WNAF_MSB_BIT || !(digit & 1)
               )
            {
                return 0;
            }
            window_val -= digit;
            if (window_val != 0 &&
                    window_val != ECC_P192_WNAF_NEXT_DIGIT_BIT &&
                    window_val != ECC_P192_WNAF_MSB_BIT
               )
            {
                return 0;
            }
        }

        r[j++] = digit;

        window_val >>= 1;
        window_val += ECC_P192_WNAF_MSB_BIT * ecc_p192_n_is_set(input, j + ECC_P192_WNAF_WINDOW, ECC_P192_TYPE_NUM_192);

        if (window_val > ECC_P192_WNAF_NEXT_DIGIT_BIT)
        {
            return 0;
        }
    }

    if (j > len + 1)
    {
        return 0;
    }
    len = j;
    *ret_len = len;
    return 1;
}

/**
 ****************************************************************************************
 * @brief This function sets the input point to infinity
 *
 * @param[in] p     Input
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_point_to_inf(ecc_p192_point_t *p)
{
    ecc_p192_n_zero(&p->Z, ECC_P192_TYPE_NUM_192);
    return 1;
}

/**
 ****************************************************************************************
 * @brief This function converts the coordinate system from jacobian to affine
 *
 * @param[in] p      Input
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC int ecc_p192_point_jacobian_to_affine(ecc_p192_point_t *p)
{
    ecc_num_192_t Z_1;
    ecc_num_192_t Z_2;

    memset((uint8_t *)&Z_1, 0, sizeof(Z_1));
    memset((uint8_t *)&Z_2, 0, sizeof(Z_2));

    if (ecc_p192_n_is_one(&p->Z, ECC_P192_TYPE_NUM_192) || ecc_p192_is_point_at_inf(p))
    {
        return 1;
    }

    if (!ecc_p192_n192_mod_inverse(&Z_1, &p->Z))
    {
        return 0;
    }
    if (!ecc_p192_n192_mod_sqr(&Z_2, &Z_1))
    {
        return 0;
    }
    if (!ecc_p192_n192_mod_mul(&p->X, &Z_2, &p->X))
    {
        return 0;
    }
    if (!ecc_p192_n192_mod_mul(&Z_2, &Z_2, &Z_1))
    {
        return 0;
    }

    if (!ecc_p192_n192_mod_mul(&p->Y, &Z_2, &p->Y))
    {
        return 0;
    }
    ecc_p192_n_one(&p->Z, ECC_P192_TYPE_NUM_192);
    return 1;
}

/**
 ****************************************************************************************
 * @brief This function is used to convert the points from jacobian to affine
 *
 * @param[in] point  The array of points to be converted from Jacobian to affine
 *
 * @return non zero on success and vice versa
 *
 ****************************************************************************************
 */
__STATIC_NO_INLINE int ecc_p192_points_jacobian_to_affine(ecc_p192_point_t *points)
{
    uint32_t i;
    // tmp0 + tmp1 +heap[8]
    ecc_num_192_t *addition_memory = ke_malloc_system(sizeof(ecc_num_192_t) * 10, KE_MEM_NON_RETENTION);
    uint32_t offset = 0;
    ecc_num_192_t *tmp0 = addition_memory + offset++;
    ecc_num_192_t *tmp1 = addition_memory + offset++;
    // heap number is 8
    ecc_num_192_t *heap  = addition_memory + offset;

    memset((uint8_t *)tmp0, 0, sizeof(ecc_num_192_t));
    memset((uint8_t *)tmp1, 0, sizeof(ecc_num_192_t));
    memset(heap, 0, sizeof(ecc_num_192_t) * 8);

    for (i = 0; i < 4; i++)
    {
        ecc_p192_n_copy((heap + 4 + i), &((points + i)->Z), ECC_P192_TYPE_NUM_192);
    }

    for (i = 3; i > 0; i--)
    {
        if (!ecc_p192_n192_mod_mul(&heap[i], &heap[2 * i], &heap[2 * i + 1]))
        {
            ke_free(addition_memory);
            return 0;
        }
    }
    if (!ecc_p192_n192_mod_inverse(&heap[1], &heap[1]))
    {
        ke_free(addition_memory);
        return 0;
    }

    for (i = 2; i < 8; i += 2)
    {
        if (!ecc_p192_n192_mod_mul(tmp0, &heap[i / 2], &heap[i + 1]))
        {
            ke_free(addition_memory);
            return 0;
        }
        if (!ecc_p192_n192_mod_mul(tmp1, &heap[i / 2], &heap[i]))
        {
            ke_free(addition_memory);
            return 0;
        }
        if (!ecc_p192_n_copy(&heap[i], tmp0, ECC_P192_TYPE_NUM_192))
        {
            ke_free(addition_memory);
            return 0;
        }
        if (!ecc_p192_n_copy(&heap[i + 1], tmp1, ECC_P192_TYPE_NUM_192))
        {
            ke_free(addition_memory);
            return 0;
        }
    }
    for (i = 4; i < 8; i++)
    {
        ecc_p192_point_t *p = &points[i - 4];

        if (!ecc_p192_n_is_zero(&p->Z, ECC_P192_TYPE_NUM_192))
        {
            if (!ecc_p192_n192_mod_sqr(tmp1, &heap[i]))
            {
                ke_free(addition_memory);
                return 0;
            }
            if (!ecc_p192_n192_mod_mul(&p->X, &p->X, tmp1))
            {
                ke_free(addition_memory);
                return 0;
            }

            if (!ecc_p192_n192_mod_mul(tmp1, tmp1, &heap[i]))
            {
                ke_free(addition_memory);
                return 0;
            }
            if (!ecc_p192_n192_mod_mul(&p->Y, &p->Y, tmp1))
            {
                ke_free(addition_memory);
                return 0;
            }
            ecc_p192_n_one(&p->Z, ECC_P192_TYPE_NUM_192);
        }
    }

    ke_free(addition_memory);
    return 1;
}



/**
 ****************************************************************************************
 * @brief This function doubles the given p192 point
 *
 * @param[in] r     The result after point doubling
 * @param[in] a     The point to be doubled
 *
 * @return non zero on success and vice versa
 ****************************************************************************************
 */
__STATIC int ecc_p192_dbl(ecc_p192_point_t *r, ecc_p192_point_t *a)
{
    ecc_num_192_t n0;
    ecc_num_192_t n1;
    ecc_num_192_t n2;
    ecc_num_192_t n3;

    memset((uint8_t *)&n0, 0, sizeof(n0));
    memset((uint8_t *)&n1, 0, sizeof(n1));
    memset((uint8_t *)&n2, 0, sizeof(n2));
    memset((uint8_t *)&n3, 0, sizeof(n3));

    if (ecc_p192_is_point_at_inf(a))
    {
        ecc_p192_n_zero(&r->Z, ECC_P192_TYPE_NUM_192);
        return 1;
    }

    if (ecc_p192_n_is_one(&a->Z, ECC_P192_TYPE_NUM_192))
    {
        if (!ecc_p192_n192_mod_sqr(&n0, &a->X))
        {
            return 0;
        }
        if (!ecc_p192_n192_mod_lshift1_quick(&n1, &n0))
        {
            return 0;
        }
        if (!ecc_p192_n192_mod_add(&n0, &n0, &n1))
        {
            return 0;
        }
        if (!ecc_p192_n192_mod_add(&n1, &n0, (ecc_num_192_t *)&ecc_p192_data.A))
        {
            return 0;
        }
    }
    else
    {
        if (!ecc_p192_n192_mod_sqr(&n1, &a->Z))
        {
            return 0;
        }
        if (!ecc_p192_n192_mod_add(&n0, &a->X, &n1))
        {
            return 0;
        }
        if (!ecc_p192_n192_mod_sub(&n2, &a->X, &n1))
        {
            return 0;
        }
        if (!ecc_p192_n192_mod_mul(&n1, &n0, &n2))
        {
            return 0;
        }
        if (!ecc_p192_n192_mod_lshift1_quick(&n0, &n1))
        {
            return 0;
        }
        if (!ecc_p192_n192_mod_add(&n1, &n0, &n1))
        {
            return 0;
        }
    }

    if (ecc_p192_n_is_one(&a->Z, ECC_P192_TYPE_NUM_192))
    {
        if (!ecc_p192_n_copy(&n0, &a->Y, ECC_P192_TYPE_NUM_192))
        {
            return 0;
        }
    }
    else
    {
        if (!ecc_p192_n192_mod_mul(&n0, &a->Y, &a->Z))
        {
            return 0;
        }
    }
    if (!ecc_p192_n192_mod_lshift1_quick(&r->Z, &n0))
    {
        return 0;
    }

    if (!ecc_p192_n192_mod_sqr(&n3, &a->Y))
    {
        return 0;
    }
    if (!ecc_p192_n192_mod_mul(&n2, &a->X, &n3))
    {
        return 0;
    }
    if (!ecc_p192_n192_mod_lshift_quick(&n2, &n2, 2))
    {
        return 0;
    }

    if (!ecc_p192_n192_mod_lshift1_quick(&n0, &n2))
    {
        return 0;
    }
    if (!ecc_p192_n192_mod_sqr(&r->X, &n1))
    {
        return 0;
    }
    if (!ecc_p192_n192_mod_sub(&r->X, &r->X, &n0))
    {
        return 0;
    }

    if (!ecc_p192_n192_mod_sqr(&n0, &n3))
    {
        return 0;
    }
    if (!ecc_p192_n192_mod_lshift_quick(&n3, &n0, 3))
    {
        return 0;
    }

    if (!ecc_p192_n192_mod_sub(&n0, &n2, &r->X))
    {
        return 0;
    }
    if (!ecc_p192_n192_mod_mul(&n0, &n1, &n0))
    {
        return 0;
    }
    if (!ecc_p192_n192_mod_sub(&r->Y, &n0, &n3))
    {
        return 0;
    }
    return 1;

}


/**
 ****************************************************************************************
 * @brief This function adds two  p192 point
 *
 * @param[in] r     The result after point addition
 * @param[in] a     The point to be added
 * @param[in] b     The point to be added
 *
 * @return non zero on success and vice versa
 ****************************************************************************************
 */
__STATIC int ecc_p192_add(ecc_p192_point_t *r, ecc_p192_point_t *a, ecc_p192_point_t *b)
{

    ecc_num_192_t *addition_memory = ke_malloc_system(sizeof(ecc_num_192_t) * 7, KE_MEM_NON_RETENTION);
    uint32_t offset = 0;
    memset(addition_memory, 0, sizeof(ecc_num_192_t) * 7);

    ecc_num_192_t *n0 = addition_memory + offset++;
    ecc_num_192_t *n1 = addition_memory + offset++;
    ecc_num_192_t *n2 = addition_memory + offset++;
    ecc_num_192_t *n3 = addition_memory + offset++;
    ecc_num_192_t *n4 = addition_memory + offset++;
    ecc_num_192_t *n5 = addition_memory + offset++;
    ecc_num_192_t *n6 = addition_memory + offset;

    if (a == b)
    {
        ke_free(addition_memory);
        return ecc_p192_dbl(r, a);
    }
    if (ecc_p192_is_point_at_inf(a))
    {
        memcpy(r, a, sizeof(ecc_p192_point_t));
        ke_free(addition_memory);
        return 1;
    }
    if (ecc_p192_is_point_at_inf(b))
    {
        memcpy(r, b, sizeof(ecc_p192_point_t));
        ke_free(addition_memory);
        return 1;
    }

    if (ecc_p192_n_is_one(&b->Z, ECC_P192_TYPE_NUM_192))
    {
        if (!ecc_p192_n_copy(n1, &a->X, ECC_P192_TYPE_NUM_192))
        {
            goto end;
        }
        if (!ecc_p192_n_copy(n2, &a->Y, ECC_P192_TYPE_NUM_192))
        {
            goto end;
        }
    }
    else
    {
        if (!ecc_p192_n192_mod_sqr(n0, &b->Z))
        {
            goto end;
        }
        if (!ecc_p192_n192_mod_mul(n1, &a->X, n0))
        {
            goto end;
        }

        if (!ecc_p192_n192_mod_mul(n0, n0, &b->Z))
        {
            goto end;
        }
        if (!ecc_p192_n192_mod_mul(n2, &a->Y, n0))
        {
            goto end;
        }
    }

    if (ecc_p192_n_is_one(&a->Z, ECC_P192_TYPE_NUM_192))
    {
        if (!ecc_p192_n_copy(n3, &b->X, ECC_P192_TYPE_NUM_192))
        {
            goto end;
        }
        if (!ecc_p192_n_copy(n4, &b->Y, ECC_P192_TYPE_NUM_192))
        {
            goto end;
        }
    }
    else
    {
        if (!ecc_p192_n192_mod_sqr(n0, &a->Z))
        {
            goto end;
        }
        if (!ecc_p192_n192_mod_mul(n3, &b->X, n0))
        {
            goto end;
        }

        if (!ecc_p192_n192_mod_mul(n0, n0, &a->Z))
        {
            goto end;
        }
        if (!ecc_p192_n192_mod_mul(n4, &b->Y, n0))
        {
            goto end;
        }
    }

    if (!ecc_p192_n192_mod_sub(n5, n1, n3))
    {
        goto end;
    }
    if (!ecc_p192_n192_mod_sub(n6, n2, n4))
    {
        goto end;
    }

    if (ecc_p192_n_is_zero(n5, ECC_P192_TYPE_NUM_192))
    {
        if (ecc_p192_n_is_zero(n6, ECC_P192_TYPE_NUM_192))
        {
            int ret = ecc_p192_dbl(r, a);
            ke_free(addition_memory);
            return ret;
        }
        else
        {
            ecc_p192_n_zero(&r->Z, ECC_P192_TYPE_NUM_192);
            ke_free(addition_memory);
            return 1;
        }
    }

    if (!ecc_p192_n192_mod_add(n1, n1, n3))
    {
        goto end;
    }
    if (!ecc_p192_n192_mod_add(n2, n2, n4))
    {
        goto end;
    }

    if (ecc_p192_n_is_one(&a->Z, ECC_P192_TYPE_NUM_192) && ecc_p192_n_is_one(&b->Z, ECC_P192_TYPE_NUM_192))
    {
        if (!ecc_p192_n_copy(&r->Z, n5, ECC_P192_TYPE_NUM_192))
        {
            goto end;
        }
    }
    else
    {
        if (ecc_p192_n_is_one(&a->Z, ECC_P192_TYPE_NUM_192))
        {
            if (!ecc_p192_n_copy(n0, &b->Z, ECC_P192_TYPE_NUM_192))
            {
                goto end;
            }
        }
        else if (ecc_p192_n_is_one(&b->Z, ECC_P192_TYPE_NUM_192))
        {
            if (!ecc_p192_n_copy(n0, &a->Z, ECC_P192_TYPE_NUM_192))
            {
                goto end;
            }
        }
        else
        {
            if (!ecc_p192_n192_mod_mul(n0, &a->Z, &b->Z))
            {
                goto end;
            }
        }
        if (!ecc_p192_n192_mod_mul(&r->Z, n0, n5))
        {
            goto end;
        }
    }

    if (!ecc_p192_n192_mod_sqr(n0, n6))
    {
        goto end;
    }
    if (!ecc_p192_n192_mod_sqr(n4, n5))
    {
        goto end;
    }
    if (!ecc_p192_n192_mod_mul(n3, n1, n4))
    {
        goto end;
    }
    if (!ecc_p192_n192_mod_sub(&r->X, n0, n3))
    {
        goto end;
    }

    if (!ecc_p192_n192_mod_lshift1_quick(n0, &r->X))
    {
        goto end;
    }
    if (!ecc_p192_n192_mod_sub(n0, n3, n0))
    {
        goto end;
    }

    if (!ecc_p192_n192_mod_mul(n0, n0, n6))
    {
        goto end;
    }
    if (!ecc_p192_n192_mod_mul(n5, n4, n5))
    {
        goto end;
    }
    if (!ecc_p192_n192_mod_mul(n1, n2, n5))
    {
        goto end;
    }
    if (!ecc_p192_n192_mod_sub(n0, n0, n1))
    {
        goto end;
    }
    if (n0->Number[0] & 0x01)
    {
        ecc_num_384_t tmp;
        if (!ecc_p192_n192_add(&tmp, n0, (ecc_num_192_t *)&ecc_p192_field))
        {
            goto end;
        }
        if (!ecc_p192_n192_rshift1(&tmp, &tmp, ECC_P192_TYPE_NUM_384))
        {
            goto end;
        }
        if (!ecc_p192_n_copy(&r->Y, &tmp, ECC_P192_TYPE_NUM_192))
        {
            goto end;
        }
        ke_free(addition_memory);
        return 1;
    }

    if (!ecc_p192_n192_rshift1(&r->Y, n0, ECC_P192_TYPE_NUM_192))
    {
        goto end;
    }

    ke_free(addition_memory);
    return 1;

end:
    ke_free(addition_memory);
    return 0;
}


/**
 ****************************************************************************************
 * @brief This function is used to precompute the points needed for scalar mul
 *
 * @param[in] r     The array for P192 pre computation
 *
 * @return non zero on success and vice versa
 ****************************************************************************************
 */
__STATIC int ecc_p192_pre_compute_points(ecc_p192_point_t *r)
{
    ecc_p192_point_t tmp;

    memset((uint8_t *)&tmp, 0, sizeof(tmp));

    int i;

    if (!ecc_p192_dbl(&tmp, &r[0]))
    {
        return 0;
    }
    for (i = 1; i < 4; i++)
    {
        if (!ecc_p192_add(&r[i], &r[i - 1], &tmp))
        {
            return 0;
        }
    }
    return 1;
}

/**
 ****************************************************************************************
 * @brief This function inverts the input p192 point
 *
 * @param[in] p    Input
 *
 * @return non zero on success and vice versa
 ****************************************************************************************
 */
__STATIC int ecc_p192_invert(ecc_p192_point_t *p)
{
    if (ecc_p192_is_point_at_inf(p) || ecc_p192_n_is_zero(&p->Y, ECC_P192_TYPE_NUM_192))
    {
        return 1;
    }
    {
        ecc_num_192_t tmp;
        ecc_p192_n_copy(&tmp, &p->Y, ECC_P192_TYPE_NUM_192);
        return (ecc_p192_n192_usub(&p->Y, (ecc_num_192_t *)&ecc_p192_field, &tmp));
    }
}


/// Check if private key is valid
__STATIC bool ecc_p192_is_valid_priv_key(uint8_t *secret_key)
{
    int i;

    // Check private key is not all zeros
    for (i = 23; i >= 0; i--)
    {
        if (secret_key[i] != 0)
            break;
    }

    if ((i == -1) && (secret_key[0] == 0))
        return false;

    // Check private key is not greater than r/2
    for (i = 23; i >= 0; i--)
    {
        if (secret_key[i] > ecc_p192_max_secret_key[i])
            return false;
        else if (secret_key[i] < ecc_p192_max_secret_key[i])
            return true;
    }

    return true;
}


/**
 ****************************************************************************************
 * @brief Start Diffie Hellman key calculation
 *
 * @param[in] secret_key        Private key                  - MSB First
 * @param[in] public_key_x      Peer public key x coordinate - LSB First
 * @param[in] public_key_y      Peer public key y coordinate - LSB First
 * @param[in] public_key_valid  True: public key is valid, False: public key uncertain
 * @param[in] metainfo          Metadata information that will be returned in procedure callback functions (see \glos{METAINFO})
 * @param[in] cb_result         Callback function to execute once algorithm completes
 *
 * @return status   0 if key generation is started, > 0 otherwise
 ****************************************************************************************
 */
__STATIC uint8_t ecc_gen_key192(const uint8_t *secret_key, const uint8_t *public_key_x, const uint8_t *public_key_y,
                                bool public_key_valid, uint32_t metainfo, ecc_p192_result_cb cb_result)
{
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;
    ecc_p192_data_t *p_data = NULL;

    do
    {
        ecc_p192_point_t *G;
        ecc_num_192_t PrivateKey;

        if (cb_result == NULL) break;

        p_data = ke_malloc_user(sizeof(ecc_p192_data_t), KE_MEM_KE_MSG);
        if (p_data == NULL)
        {
            status = CO_ERROR_MEMORY_CAPA_EXCEED;
            break;
        }
        memset(p_data, 0, sizeof(ecc_p192_data_t));

        p_data->metainfo = metainfo;
        p_data->cb_result = cb_result;
        PrivateKey.Neg = 0;

        ecc_p192_convert_u8_to_u32(PrivateKey.Number, secret_key, PRIV_KEY_192_LEN / 4);
        ecc_p192_convert_u8_to_u32(p_data->RemotePublicKey.X.Number, public_key_x, PUB_KEY_192_LEN / 8);
        ecc_p192_convert_u8_to_u32(p_data->RemotePublicKey.Y.Number, public_key_y, PUB_KEY_192_LEN / 8);
        ecc_p192_n_one(&(p_data->RemotePublicKey.Z), ECC_P192_TYPE_NUM_192);


        if (ecc_p192_n_is_zero(&PrivateKey, ECC_P192_TYPE_NUM_192)) break;

        if (ecc_p192_n_is_zero(&(p_data->RemotePublicKey.X), ECC_P192_TYPE_NUM_192)) break;

        if (ecc_p192_n_is_zero(&(p_data->RemotePublicKey.Y), ECC_P192_TYPE_NUM_192)) break;

        if (!public_key_valid && !ecc_p192_is_point_valid(&(p_data->RemotePublicKey))) break;

        if (!ecc_p192_n192_convert_wnaf(p_data->wnaf, &PrivateKey, &(p_data->wnaf_len))) break;

        G = p_data->Pre_comp;
        memcpy(G, &(p_data->RemotePublicKey), sizeof(ecc_p192_point_t));

        if (!ecc_p192_pre_compute_points(G)) break;

        if (!ecc_p192_points_jacobian_to_affine(G)) break;

        // Initialize the loop counter
        p_data->Loop_ctr = p_data->wnaf_len - 1;
        p_data->Result_at_infinity  = 1;
        p_data->Result_inverted  = 0;

        // Insert the multiplication at the end of the list
        co_list_push_back(&ecc_p192_env.compute_queue, &p_data->hdr);
        // Start the event
        ke_event_set(KE_EVENT_ECC_P192_MULTIPLICATION);

        status = CO_ERROR_NO_ERROR;
    }
    while (0);


    // handle error case
    if ((status != CO_ERROR_NO_ERROR) && (p_data != NULL))
    {
        ke_free(p_data);
    }

    return status;
}

/// @brief Continue execution of P192 key computation
__STATIC uint8_t ecc_gen_key192_continue(ecc_p192_data_t *p_data)
{
    ecc_p192_point_t *r;
    int k;
    uint8_t r_is_at_infinity;
    uint8_t r_is_inverted;
    uint8_t LoopCtr;

    r = &(p_data->computed_key);
    k = p_data->Loop_ctr;
    r_is_at_infinity = p_data->Result_at_infinity;
    r_is_inverted =  p_data->Result_inverted;
    LoopCtr = ECC_P192_DHKEY_CALC_NUM_BITS_PER_CALL;

    for (; LoopCtr > 0; LoopCtr--)
    {
        int digit = (int)p_data->wnaf[k];
        int is_neg;

        if (!r_is_at_infinity)
        {
            if (!ecc_p192_dbl(r, r))
            {
                return ECC_P192_OPER_FAILED;
            }
        }

        if (digit)
        {
            is_neg = digit < 0;

            if (is_neg)
            {
                digit = -digit;
            }

            if (is_neg != r_is_inverted)
            {
                if (!r_is_at_infinity)
                {
                    if (!ecc_p192_invert(r))
                    {
                        return ECC_P192_OPER_FAILED;
                    }
                }
                r_is_inverted = !r_is_inverted;
            }

            if (r_is_at_infinity)
            {
                memcpy(r, &(p_data->Pre_comp[digit >> 1]), sizeof(ecc_p192_point_t));
                r_is_at_infinity = 0;
            }
            else
            {
                if (!ecc_p192_add(r, r, &(p_data->Pre_comp[digit >> 1])))
                {
                    return ECC_P192_OPER_FAILED;
                }
            }
        }
        k--;
        if (k < 0)
        {
            break;
        }
    }
    if (k == -1)
    {
        if (r_is_at_infinity)
        {
            if (!ecc_p192_point_to_inf(r))
            {
                return ECC_P192_OPER_FAILED;
            }
        }
        else
        {
            if (r_is_inverted)
            {
                if (!ecc_p192_invert(r))
                {
                    return ECC_P192_OPER_FAILED;
                }
            }
        }
        if (!ecc_p192_point_jacobian_to_affine(r))
        {
            return ECC_P192_OPER_FAILED;
        }

        return ECC_P192_OPER_SUCCESS;
    }
    p_data->Result_at_infinity = r_is_at_infinity;
    p_data->Result_inverted = r_is_inverted;
    p_data->Loop_ctr = k;
    return ECC_P192_OPER_PENDING;
}


/**
 ****************************************************************************************
 * @brief ECC ecc_multiplication event handler
 ****************************************************************************************
 */
__STATIC void ecc_p192_multiplication_event_handler(void)
{
    DBG_SWDIAG(ECDH, COMPUTE, 1);
    // Take the next multiplication
    ecc_p192_data_t *p_data = (ecc_p192_data_t *) co_list_pick(&ecc_p192_env.compute_queue);

    led_set(4);

    // Check if element is present
    ASSERT_ERR(p_data != NULL);
    if (p_data != NULL)
    {
        uint8_t operation_status;
        DBG_SWDIAG(ECDH, MULT, 1);
        operation_status = ecc_gen_key192_continue(p_data);
        DBG_SWDIAG(ECDH, MULT, 0);

        switch (operation_status)
        {
        case ECC_P192_OPER_FAILED:
        case ECC_P192_OPER_SUCCESS:
        {
            ecc_p192_result_t result;
            DBG_SWDIAG(ECDH, END, 1);
            co_list_pop_front(&ecc_p192_env.compute_queue);
            ecc_p192_convert_u32_to_u8(&(result.key_res_x[0]), p_data->computed_key.X.Number, PUB_KEY_192_LEN / 2);
            ecc_p192_convert_u32_to_u8(&(result.key_res_y[0]), p_data->computed_key.Y.Number, PUB_KEY_192_LEN / 2);

            p_data->cb_result(p_data->metainfo,
                              (operation_status == ECC_P192_OPER_SUCCESS) ? CO_ERROR_NO_ERROR : CO_ERROR_UNSPECIFIED_ERROR,
                              &result);

            ke_free(p_data);
            DBG_SWDIAG(ECDH, END, 0);
        }
        break;
        default: { /* Do nothing */ } break;
        }
    }

    // Stop background computation if all multiplication performed
    if (co_list_is_empty(&(ecc_p192_env.compute_queue)))
    {
        ke_event_clear(KE_EVENT_ECC_P192_MULTIPLICATION);
        DBG_SWDIAG(ECDH, BUSY, 0);
        led_reset(4);
    }
    DBG_SWDIAG(ECDH, COMPUTE, 0);
}


/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Initialize Elliptic Curve algorithm
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void ecc_p192_init(bool is_reset)
{
    if (!is_reset)
    {
        // Register event to handle multiplication steps
        ke_event_callback_set(KE_EVENT_ECC_P192_MULTIPLICATION, &ecc_p192_multiplication_event_handler);
    }
    else
    {
        // Clear the kernel event if pending
        ke_event_clear(KE_EVENT_ECC_P192_MULTIPLICATION);

        // Empty multiplications list
        while (!co_list_is_empty(&ecc_p192_env.compute_queue))
        {
            // Free allocated memory
            ecc_p192_data_t *p_data = (ecc_p192_data_t *) co_list_pop_front(&ecc_p192_env.compute_queue);
            ke_free(p_data);
        }
        led_reset(4);
    }

    // Initialize multiplications list
    co_list_init(&ecc_p192_env.compute_queue);
}

void ecc_p192_gen_new_secret_key(uint8_t *secret_key)
{
    /*
     * Get a new private key from 32 Byte random number generator.
     * The private keys shall be between 1 and r/2, where r is the order of the Abelian group
     * on the elliptic curve (i.e., between 1 and 2^192/2).
     */
    do
    {
        co_write32p(&secret_key[0],  co_rand_word());
        co_write32p(&secret_key[4],  co_rand_word());
        co_write32p(&secret_key[8],  co_rand_word());
        co_write32p(&secret_key[12], co_rand_word());
        co_write32p(&secret_key[16], co_rand_word());
        co_write32p(&secret_key[20], co_rand_word());
    }
    while (!ecc_p192_is_valid_priv_key(secret_key));
}


uint8_t ecc_p192_gen_new_public_key(const uint8_t *secret_key, uint32_t metainfo, ecc_p192_result_cb cb_result)
{
    return ecc_gen_key192(secret_key, ecc_p192_base_point_x, ecc_p192_base_point_y, true, metainfo, cb_result);
}

uint8_t ecc_p192_gen_dh_key(const uint8_t *secret_key, const uint8_t *public_key_x, const uint8_t *public_key_y,
                            uint32_t metainfo, ecc_p192_result_cb cb_result)
{
    return ecc_gen_key192(secret_key, public_key_x, public_key_y, false, metainfo, cb_result);
}


uint8_t ecc_p192_gen_key_pair(bool debug_key, uint8_t *secret_key, uint32_t metainfo, ecc_p192_result_cb cb_result)
{
    if (debug_key)
    {
        ecc_p192_result_t result;

        memcpy(secret_key, ecc_p192_debug_priv_key, PRIV_KEY_192_LEN);
        memcpy(result.key_res_x, ecc_p192_debug_pub_key_x, PUB_KEY_192_LEN / 2);
        memcpy(result.key_res_y, ecc_p192_debug_pub_key_y, PUB_KEY_192_LEN / 2);

        // Call immediately result function
        cb_result(metainfo, CO_ERROR_NO_ERROR, &result);
    }
    else
    {
        ecc_p192_gen_new_secret_key(secret_key);
        ecc_p192_gen_new_public_key(secret_key, metainfo, cb_result);
    }

    return CO_ERROR_NO_ERROR;
}

void ecc_p192_abort_key_gen(uint16_t metainfo)
{
    ecc_p192_data_t *p_prev = NULL;
    ecc_p192_data_t *p_current = (ecc_p192_data_t *) co_list_pick(&ecc_p192_env.compute_queue);

    // Empty multiplications list
    while (p_current != NULL)
    {
        // Check if the element correspond to the abort request
        if (p_current->metainfo == metainfo)
        {
            // Extract element from the list
            co_list_extract_after(&ecc_p192_env.compute_queue, &p_prev->hdr, &p_current->hdr);

            // Free allocated memory
            ke_free(p_current);
            break;
        }

        // Jump to next
        p_prev = p_current;
        p_current = (ecc_p192_data_t *) co_list_next(&p_current->hdr);
    }


    // Stop event if nothing more to compute
    if (co_list_is_empty(&(ecc_p192_env.compute_queue)))
    {
        ke_event_clear(KE_EVENT_ECC_P192_MULTIPLICATION);
    }
}

const uint8_t *ecc_p192_get_debug_pub_key_x(void)
{
    return &(ecc_p192_debug_pub_key_x[0]);
}

const uint8_t *ecc_p192_get_debug_pub_key_y(void)
{
    return &(ecc_p192_debug_pub_key_y[0]);
}

#endif // (BT_EMB_PRESENT)
