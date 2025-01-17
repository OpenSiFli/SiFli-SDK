/**
  ******************************************************************************
  * @file   bts2_util.h
  * @author Sifli software development team
  ******************************************************************************
*/
/*
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

#ifndef _BTS2_UTIL_H_
#define _BTS2_UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

#define ABS(a)              ((a) > 0 ? (a) : -(a))
#ifndef MAX
#define MAX(a,b)            (((U32)a > (U32)b) ? a : b)
#endif /* MAX */
#ifndef MIN
#define MIN(a,b)            (((U32)a > (U32)b) ? b : a)
#endif /* MIN */
#define ADD(a, b)           ((a) + (b))
#define SUB(a, b)           ((S32)(a) - (S32)(b))
#define BT_IN(x,min,max)    ((U32)(x) >= (U32)min && (U32)(x) <= (U32)max)
#define BT_OUT(x,min,max)   ((U32)(x) <  (U32)min || (U32)(x) >  (U32)max)
#define UPPER(c)            (((c) >= 'a') && ((c) <= 'z') ? ((c) - 0x20) : (c))
#define LOWER(c)            (((c) >= 'A') && ((c) <= 'Z') ? ((c) + 0x20) : (c))

#if defined(CFG_LITTLE_ENDIAN)

#define U8R(p, o) *(p + o)
#define U16R(p, o) (*(U16 *)(p + o))
#define U32R(p, o) (*(U32 *)(p + o))

#elif defined(CFG_BIG_ENDIAN)

#define U8R(p, o) *(p + o)
#define U16R(p, o) (U16)((U16)p[0 + o] | (U16)p[1 + o] << 8)
#define U32R(p, o) (U32)((U32)p[0 + o] | (U32)p[1 + o] << 8  | (U32)p[2 + o] << 16 | (U32)p[3 + o] << 24)
#endif

/* These macros extract the HCI opcodes from a buffer
*/
#define HCI_GET_CMD_HDR_OPCODE(p)                    \
  (uint16_t)((*((uint8_t*)((p) + 1) + (p)->offset) + \
              (*((uint8_t*)((p) + 1) + (p)->offset + 1) << 8)))
#define HCI_GET_CMD_HDR_PARAM_LEN(p) \
  (uint8_t)(*((uint8_t*)((p) + 1) + (p)->offset + 2))

#define HCI_GET_EVT_HDR_OPCODE(p) \
  (uint8_t)(*((uint8_t*)((p) + 1) + (p)->offset))
#define HCI_GET_EVT_HDR_PARAM_LEN(p) \
  (uint8_t)(*((uint8_t*)((p) + 1) + (p)->offset + 1))

/*******************************************************************************
 * Macros to get and put bytes to and from a stream (Little Endian format).
*/
#define UINT64_TO_BE_STREAM(p, u64)  \
  {                                  \
    *(p)++ = (uint8_t)((u64) >> 56); \
    *(p)++ = (uint8_t)((u64) >> 48); \
    *(p)++ = (uint8_t)((u64) >> 40); \
    *(p)++ = (uint8_t)((u64) >> 32); \
    *(p)++ = (uint8_t)((u64) >> 24); \
    *(p)++ = (uint8_t)((u64) >> 16); \
    *(p)++ = (uint8_t)((u64) >> 8);  \
    *(p)++ = (uint8_t)(u64);         \
  }
#define UINT32_TO_STREAM(p, u32)     \
  {                                  \
    *(p)++ = (uint8_t)(u32);         \
    *(p)++ = (uint8_t)((u32) >> 8);  \
    *(p)++ = (uint8_t)((u32) >> 16); \
    *(p)++ = (uint8_t)((u32) >> 24); \
  }
#define UINT24_TO_STREAM(p, u24)     \
  {                                  \
    *(p)++ = (uint8_t)(u24);         \
    *(p)++ = (uint8_t)((u24) >> 8);  \
    *(p)++ = (uint8_t)((u24) >> 16); \
  }
#define UINT16_TO_STREAM(p, u16)    \
  {                                 \
    *(p)++ = (uint8_t)(u16);        \
    *(p)++ = (uint8_t)((u16) >> 8); \
  }
#define UINT8_TO_STREAM(p, u8) \
  { *(p)++ = (uint8_t)(u8); }
#define INT8_TO_STREAM(p, u8) \
  { *(p)++ = (int8_t)(u8); }
#define ARRAY32_TO_STREAM(p, a)                                     \
  {                                                                 \
    int ijk;                                                        \
    for (ijk = 0; ijk < 32; ijk++) *(p)++ = (uint8_t)(a)[31 - ijk]; \
  }
#define ARRAY16_TO_STREAM(p, a)                                     \
  {                                                                 \
    int ijk;                                                        \
    for (ijk = 0; ijk < 16; ijk++) *(p)++ = (uint8_t)(a)[15 - ijk]; \
  }
#define ARRAY8_TO_STREAM(p, a)                                    \
  {                                                               \
    int ijk;                                                      \
    for (ijk = 0; ijk < 8; ijk++) *(p)++ = (uint8_t)(a)[7 - ijk]; \
  }
#define BDADDR_TO_STREAM(p, a)                      \
  {                                                 \
    int ijk;                                        \
    for (ijk = 0; ijk < BD_ADDR_LEN; ijk++)         \
      *(p)++ = (uint8_t)(a)[BD_ADDR_LEN - 1 - ijk]; \
  }
#define LAP_TO_STREAM(p, a)                     \
  {                                             \
    int ijk;                                    \
    for (ijk = 0; ijk < LAP_LEN; ijk++)         \
      *(p)++ = (uint8_t)(a)[LAP_LEN - 1 - ijk]; \
  }
#define DEVCLASS_TO_STREAM(p, a)                      \
  {                                                   \
    int ijk;                                          \
    for (ijk = 0; ijk < DEV_CLASS_LEN; ijk++)         \
      *(p)++ = (uint8_t)(a)[DEV_CLASS_LEN - 1 - ijk]; \
  }
#define ARRAY_TO_STREAM(p, a, len)                                \
  {                                                               \
    int ijk;                                                      \
    for (ijk = 0; ijk < (len); ijk++) *(p)++ = (uint8_t)(a)[ijk]; \
  }
#define REVERSE_ARRAY_TO_STREAM(p, a, len)                                  \
  {                                                                         \
    int ijk;                                                                \
    for (ijk = 0; ijk < (len); ijk++) *(p)++ = (uint8_t)(a)[(len)-1 - ijk]; \
  }

#define STREAM_TO_INT8(u8, p) \
  {                           \
    (u8) = (*((int8_t*)p));   \
    (p) += 1;                 \
  }
#define STREAM_TO_UINT8(u8, p) \
  {                            \
    (u8) = (uint8_t)(*(p));    \
    (p) += 1;                  \
  }
#define STREAM_TO_UINT16(u16, p)                                  \
  {                                                               \
    (u16) = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); \
    (p) += 2;                                                     \
  }
#define STREAM_TO_UINT24(u32, p)                                      \
  {                                                                   \
    (u32) = (((uint32_t)(*(p))) + ((((uint32_t)(*((p) + 1)))) << 8) + \
             ((((uint32_t)(*((p) + 2)))) << 16));                     \
    (p) += 3;                                                         \
  }
#define STREAM_TO_UINT32(u32, p)                                      \
  {                                                                   \
    (u32) = (((uint32_t)(*(p))) + ((((uint32_t)(*((p) + 1)))) << 8) + \
             ((((uint32_t)(*((p) + 2)))) << 16) +                     \
             ((((uint32_t)(*((p) + 3)))) << 24));                     \
    (p) += 4;                                                         \
  }
#define STREAM_TO_BDADDR(a, p)                                \
  {                                                           \
    int ijk;                                                  \
    uint8_t* pbda = (uint8_t*)(a) + BD_ADDR_LEN - 1;          \
    for (ijk = 0; ijk < BD_ADDR_LEN; ijk++) *pbda-- = *(p)++; \
  }
#define STREAM_TO_ARRAY32(a, p)                     \
  {                                                 \
    int ijk;                                        \
    uint8_t* _pa = (uint8_t*)(a) + 31;              \
    for (ijk = 0; ijk < 32; ijk++) *_pa-- = *(p)++; \
  }
#define STREAM_TO_ARRAY16(a, p)                     \
  {                                                 \
    int ijk;                                        \
    uint8_t* _pa = (uint8_t*)(a) + 15;              \
    for (ijk = 0; ijk < 16; ijk++) *_pa-- = *(p)++; \
  }
#define STREAM_TO_ARRAY8(a, p)                     \
  {                                                \
    int ijk;                                       \
    uint8_t* _pa = (uint8_t*)(a) + 7;              \
    for (ijk = 0; ijk < 8; ijk++) *_pa-- = *(p)++; \
  }
#define STREAM_TO_DEVCLASS(a, p)                               \
  {                                                            \
    int ijk;                                                   \
    uint8_t* _pa = (uint8_t*)(a) + DEV_CLASS_LEN - 1;          \
    for (ijk = 0; ijk < DEV_CLASS_LEN; ijk++) *_pa-- = *(p)++; \
  }
#define STREAM_TO_LAP(a, p)                               \
  {                                                       \
    int ijk;                                              \
    uint8_t* plap = (uint8_t*)(a) + LAP_LEN - 1;          \
    for (ijk = 0; ijk < LAP_LEN; ijk++) *plap-- = *(p)++; \
  }
#define STREAM_TO_ARRAY(a, p, len)                                   \
  {                                                                  \
    int ijk;                                                         \
    for (ijk = 0; ijk < (len); ijk++) ((uint8_t*)(a))[ijk] = *(p)++; \
  }
#define REVERSE_STREAM_TO_ARRAY(a, p, len)             \
  {                                                    \
    int ijk;                                           \
    uint8_t* _pa = (uint8_t*)(a) + (len)-1;            \
    for (ijk = 0; ijk < (len); ijk++) *_pa-- = *(p)++; \
  }

#define STREAM_SKIP_UINT8(p) \
  do {                       \
    (p) += 1;                \
  } while (0)
#define STREAM_SKIP_UINT16(p) \
  do {                        \
    (p) += 2;                 \
  } while (0)

/*******************************************************************************
 * Macros to get and put bytes to and from a field (Little Endian format).
 * These are the same as to stream, except the pointer is not incremented.
*/
#define UINT32_TO_FIELD(p, u32)                    \
  {                                                \
    *(uint8_t*)(p) = (uint8_t)(u32);               \
    *((uint8_t*)(p) + 1) = (uint8_t)((u32) >> 8);  \
    *((uint8_t*)(p) + 2) = (uint8_t)((u32) >> 16); \
    *((uint8_t*)(p) + 3) = (uint8_t)((u32) >> 24); \
  }
#define UINT24_TO_FIELD(p, u24)                    \
  {                                                \
    *(uint8_t*)(p) = (uint8_t)(u24);               \
    *((uint8_t*)(p) + 1) = (uint8_t)((u24) >> 8);  \
    *((uint8_t*)(p) + 2) = (uint8_t)((u24) >> 16); \
  }
#define UINT16_TO_FIELD(p, u16)                   \
  {                                               \
    *(uint8_t*)(p) = (uint8_t)(u16);              \
    *((uint8_t*)(p) + 1) = (uint8_t)((u16) >> 8); \
  }
#define UINT8_TO_FIELD(p, u8) \
  { *(uint8_t*)(p) = (uint8_t)(u8); }

/*******************************************************************************
 * Macros to get and put bytes to and from a stream (Big Endian format)
*/
#define UINT32_TO_BE_STREAM(p, u32)  \
  {                                  \
    *(p)++ = (uint8_t)((u32) >> 24); \
    *(p)++ = (uint8_t)((u32) >> 16); \
    *(p)++ = (uint8_t)((u32) >> 8);  \
    *(p)++ = (uint8_t)(u32);         \
  }
#define UINT24_TO_BE_STREAM(p, u24)  \
  {                                  \
    *(p)++ = (uint8_t)((u24) >> 16); \
    *(p)++ = (uint8_t)((u24) >> 8);  \
    *(p)++ = (uint8_t)(u24);         \
  }
#define UINT16_TO_BE_STREAM(p, u16) \
  {                                 \
    *(p)++ = (uint8_t)((u16) >> 8); \
    *(p)++ = (uint8_t)(u16);        \
  }
#define UINT8_TO_BE_STREAM(p, u8) \
  { *(p)++ = (uint8_t)(u8); }
#define ARRAY_TO_BE_STREAM(p, a, len)                             \
  {                                                               \
    int ijk;                                                      \
    for (ijk = 0; ijk < (len); ijk++) *(p)++ = (uint8_t)(a)[ijk]; \
  }
#define ARRAY_TO_BE_STREAM_REVERSE(p, a, len)                               \
  {                                                                         \
    int ijk;                                                                \
    for (ijk = 0; ijk < (len); ijk++) *(p)++ = (uint8_t)(a)[(len)-ijk - 1]; \
  }

#define BE_STREAM_TO_UINT8(u8, p) \
  {                               \
    (u8) = (uint8_t)(*(p));       \
    (p) += 1;                     \
  }
#define BE_STREAM_TO_UINT16(u16, p)                                       \
  {                                                                       \
    (u16) = (uint16_t)(((uint16_t)(*(p)) << 8) + (uint16_t)(*((p) + 1))); \
    (p) += 2;                                                             \
  }
#define BE_STREAM_TO_UINT24(u32, p)                                     \
  {                                                                     \
    (u32) = (((uint32_t)(*((p) + 2))) + ((uint32_t)(*((p) + 1)) << 8) + \
             ((uint32_t)(*(p)) << 16));                                 \
    (p) += 3;                                                           \
  }
#define BE_STREAM_TO_UINT32(u32, p)                                      \
  {                                                                      \
    (u32) = ((uint32_t)(*((p) + 3)) + ((uint32_t)(*((p) + 2)) << 8) +    \
             ((uint32_t)(*((p) + 1)) << 16) + ((uint32_t)(*(p)) << 24)); \
    (p) += 4;                                                            \
  }
#define BE_STREAM_TO_UINT64(u64, p)                                            \
  {                                                                            \
    (u64) = ((uint64_t)(*((p) + 7)) + ((uint64_t)(*((p) + 6)) << 8) +          \
             ((uint64_t)(*((p) + 5)) << 16) + ((uint64_t)(*((p) + 4)) << 24) + \
             ((uint64_t)(*((p) + 3)) << 32) + ((uint64_t)(*((p) + 2)) << 40) + \
             ((uint64_t)(*((p) + 1)) << 48) + ((uint64_t)(*(p)) << 56));       \
    (p) += 8;                                                                  \
  }
#define BE_STREAM_TO_ARRAY(p, a, len)                                \
  {                                                                  \
    int ijk;                                                         \
    for (ijk = 0; ijk < (len); ijk++) ((uint8_t*)(a))[ijk] = *(p)++; \
  }

/*******************************************************************************
 * Macros to get and put bytes to and from a field (Big Endian format).
 * These are the same as to stream, except the pointer is not incremented.
*/
#define UINT32_TO_BE_FIELD(p, u32)                 \
  {                                                \
    *(uint8_t*)(p) = (uint8_t)((u32) >> 24);       \
    *((uint8_t*)(p) + 1) = (uint8_t)((u32) >> 16); \
    *((uint8_t*)(p) + 2) = (uint8_t)((u32) >> 8);  \
    *((uint8_t*)(p) + 3) = (uint8_t)(u32);         \
  }
#define UINT24_TO_BE_FIELD(p, u24)                \
  {                                               \
    *(uint8_t*)(p) = (uint8_t)((u24) >> 16);      \
    *((uint8_t*)(p) + 1) = (uint8_t)((u24) >> 8); \
    *((uint8_t*)(p) + 2) = (uint8_t)(u24);        \
  }
#define UINT16_TO_BE_FIELD(p, u16)          \
  {                                         \
    *(uint8_t*)(p) = (uint8_t)((u16) >> 8); \
    *((uint8_t*)(p) + 1) = (uint8_t)(u16);  \
  }
#define UINT8_TO_BE_FIELD(p, u8) \
  { *(uint8_t*)(p) = (uint8_t)(u8); }



#if defined(CFG_LITTLE_ENDIAN)

#define DEFMT_8(p)     ((U8 *)p)[0]
#define DEFMT_16(p)    ((U16)((U8 *)p)[1]) << 8 | ((U16)((U8 *)p)[0])
#define DEFMT_24(p)    ((U32)((U8 *)p)[2]) << 16 | ((U32)((U8 *)p)[1]) << 8 | ((U32)((U8 *)p)[0])
#define DEFMT_32(p)    ((U32)((U8 *)p)[3]) << 24 | ((U32)((U8 *)p)[2]) << 16 | ((U32)((U8 *)p)[1]) << 8 | ((U32) ((U8 *)p)[0])

#endif


/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Copy the source BlueTooth Address into the destination BlueTooth Address.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *dest: destination BlueTooth Address.
 *      const BTS2S_BD_ADDR *src: source BlueTooth Address.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bd_copy(BTS2S_BD_ADDR *dest, const BTS2S_BD_ADDR *src);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Compare two BlueTooth Address for equality.
 *
 * INPUT:
 *      const BTS2S_BD_ADDR *bd1: BlueTooth Address1.
 *      const BTS2S_BD_ADDR *bd2: BlueTooth Address2.
 *
 * OUTPUT:
 *      if equality,return TRUE; otherwise return FALSE.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
BOOL bd_eq(const BTS2S_BD_ADDR *bd1, const BTS2S_BD_ADDR *bd2);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Set the BlueTooth address to be empty.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd: BlueTooth address.
 *
 * OUTPUT:
 *      void
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bd_set_empty(BTS2S_BD_ADDR *bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Check if the input BlueTooth Addresses is empty.
 *
 * INPUT:
 *      BTS2S_BD_ADDR *bd: BlueTooth Addresses.
 *
 * OUTPUT:
 *      if the BlueTooth Addresses is empty, return TRUE,otherwise return FALSE.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
BOOL bd_is_empty(const BTS2S_BD_ADDR *bd);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bmemset function sets the first count bytes of dest to the character c.
 *
 * INPUT:
 *      void *dest: Pointer to destination.
 *      unsigned char c: Character to set.
 *      int count: Number of characters.
 *
 * OUTPUT:
 *      bmemset returns the value of dest.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void *bmemset(void *dest, unsigned char c, int count);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bmemcpy function copies count bytes of src to dest. If the source
 *      and destination overlap, this function does not ensure that the original
 *      source bytes in the overlapping region are copied before being overwritten.
 *      Use memmove to handle overlapping regions.
 *
 * INPUT:
 *      void *dest: New buffer.
 *      const void *src: Buffer to copy from.
 *      int count: Number of characters to copy.
 *
 * OUTPUT:
 *      bmemcpy returns the value of dest.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void *bmemcpy(void *dest, const void *src, U32 count);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bmemcmp function compares the first count bytes of buf1 and buf2 and
 *      returns a value indicating their relationship
 *
 * INPUT:
 *      const unsigned char *buf1: First buffer.
 *      const unsigned char *buf2: Second buffer.
 *      int count: Number of characters.
 *
 * OUTPUT:
 *      The return value indicates the relationship between the buffers:
 *      < 0     buf1 less than buf2
 *      0       buf1 identical to buf2
 *      >0      buf1 greater than buf2
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
int bmemcmp(const unsigned char *buf1, const unsigned char *buf2, int count);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bstrcpy function copies strSource, including the terminating null
 *      character, to the location specified by strDestination. No overflow checking
 *      is performed when strings are copied or appended. The behavior of strcpy is
 *      undefined if the source and destination strings overlap.
 *
 * INPUT:
 *      char *dest: Destination string.
 *      char *src: Null-terminated source string.
 *
 * OUTPUT:
 *      Each of these functions returns the destination string. No return value
 *      is reserved to indicate an error
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
char *bstrcpy(char *dest, char *src);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bstrncpy function copies the initial count characters of strSource
 *      to strDest and returns strDest. If count is less than or equal to the
 *      length of strSource, a null character is not appended automatically to
 *      the copied string. If count is greater than the length of strSource,
 *      the destination string is padded with null characters up to length count.
 *      The behavior of strncpy is undefined if the source and destination strings
 *      overlap.
 *
 * INPUT:
 *      char *dest: Destination string.
 *      char *src: Source string.
 *      int count: Number of characters to be copied.
 *
 * OUTPUT:
 *      Each of these functions returns strDest. No return value is reserved
 *      to indicate an error.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
char *bstrncpy(char *dest, char *src, int count);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bstrcat function appends strSource to strDestination and terminates
 *      the resulting string with a null character. The initial character of
 *      strSource overwrites the terminating null character of strDestination.
 *      No overflow checking is performed when strings are copied or appended.
 *      The behavior of strcat is undefined if the source and destination
 *      strings overlap.
 *
 * INPUT:
 *      char *dest: Null-terminated destination string.
 *      const char *src: Null-terminated source string.
 *
 * OUTPUT:
 *      Each of these functions returns the destination string (strDestination).
 *      No return value is reserved to indicate an error.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
char *bstrcat(char *dest, const char *src);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bstrncat function appends, at most, the first count characters of
 *      strSource to strDest. The initial character of strSource overwrites the
 *      terminating null character of strDest. If a null character appears in
 *      strSource before count characters are appended, strncat appends all
 *      characters from strSource, up to the null character. If count is greater
 *      than the length of strSource, the length of strSource is used in place of
 *      count. The resulting string is terminated with a null character. If
 *      copying takes place between strings that overlap, the behavior is undefined.
 *
 * INPUT:
 *      char *dest: Null-terminated destination string.
 *      char *src: Null-terminated source string.
 *      int count: Number of characters to append.
 *
 * OUTPUT:
 *      Each of these functions returns a pointer to the destination string.
 *      No return value is reserved to indicate an error.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
char *bstrncat(char *dest, const char *src, int count);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bstrchr function finds the first occurrence of c in string, or it
 *      returns NULL if c is not found. The null-terminating character is included
 *      in the search.
 *
 * INPUT:
 *     const char *str: Null-terminated source string.
 *     int c: Character to be located.
 *
 * OUTPUT:
 *      Each of these functions returns a pointer to the first occurrence of c
 *      in string, or NULL if c is not found.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
char *bstrchr(const char *str, int c);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bstrchr function finds the first occurrence of sub_str in string, or it
 *      returns NULL if sub_str is not found. The null-terminating character is included
 *      in the search.
 *
 * INPUT:
 *      const char *buf: Null-terminated source string.
 *      const char *sub_str: Character to be located.
 *
 * OUTPUT:
 *      Each of these functions returns a pointer to the first occurrence of sub_str
 *      in string, or NULL if sub_str is not found.
 *
 * NOTE:
 *
 *
 *----------------------------------------------------------------------------*/
char *bstrstr(const char *buf, const char *sub_str);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Each of these functions returns the number of characters in string,
 *      not including the terminating null character.
 *
 * INPUT:
 *      const char *str: Null-terminated string.
 *
 * OUTPUT:
 *      Each of these functions returns the number of characters in string,
 *      excluding the terminal NULL. No return value is reserved to indicate
 *      an error.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U32 bstrlen(const char *str);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Compare string in uppercase.
 *
 * INPUT:
 *      char *str1: string one.
 *      char *str2: string two.
 *
 * OUTPUT:
 *      The return value indicates the relation of string1 to string2.
 *      <0   str1 less than str2.
 *      0    str1 identical to str2.
 *      >0   str1 greater than str2.
 *
 * NOTE:
 *      The bstricmp function lexicographically compares uppercase versions of
 *      string1 and string2 and returns a value indicating their relationship.
 *
 *----------------------------------------------------------------------------*/
int bstricmp(char *str1, char *str2);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Compare string in uppercase with the specified account.
 *
 * INPUT:
 *      const char *str: string one.
 *      const char *str2: string two.
 *      int count: The number of comparing.
 *
 * OUTPUT:
 *      The return value indicates the relation of string1 to string2.
 *      1   str1 greater than str2.
 *      -1  str1 less than str2.
 *      0   str1 identical to str2.
 * NOTE:
 *      The bstrnicmp function lexicographically compares uppercase versions of
 *      string1 and string2 and returns a value indicating their relationship.
 *
 *----------------------------------------------------------------------------*/
int bstrnicmp(const char *str1, const char *str2, int count);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Copy one UNICODE string to another.
 *
 * INPUT:
 *      U8 *dest: destination string.
 *      U8 *src: source string.
 *
 * OUTPUT:
 *      returns the destination string.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 *bustrcpy(U8 *dest, U8 *src);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Copy an UNICODE string, parameter including the length.
 *
 * INPUT:
 *      U8 *target: destination string.
 *      U8 *src: source string.
 *      U32 count: Number of characters to be copied.
 *
 * OUTPUT:
 *      returns the destination string.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 *bustrncpy(U8 *dest, U8 *src, U32 count);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Calculate the UNICODE string length.
 *
 * INPUT:
 *      U8 *unicode_str: UNICODE string.
 *
 * OUTPUT:
 *      Each of these functions returns the number of characters in string,
 *      excluding the terminal NULL.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U16 bustrlen(U8 *ustr);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Convert UTF8 to UNICODE string.
 *
 * INPUT:
 *      U8 *utf8: UTF8 string.
 *
 * OUTPUT:
 *      return UNICODE string.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U16 *bu8str2u(U8 *utf8);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Convert UTF8 string to UNICODE string, include the count number.
 *
 * INPUT:
 *      U8 *utf8str: UTF8 string.
 *      int count: Number of characters to be converted.
 *
 * OUTPUT:
 *      return UNICODE string.
 *
 * NOTE:
 *      After call this routine,it needs free.
 *
 *----------------------------------------------------------------------------*/
U8 *bu8strn2u(U8 *utf8, int count);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Convert char string to UNICODE string, include the length.
 *
 * INPUT:
 *      char *dest: destination string.
 *      const char *str: source string.
 *      int len: Number of characters to be converted.
 *
 * OUTPUT:
 *      return UNICODE string.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U32 bstrn2u(char *dest, const char *str, int len);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Convert UNICODE string to char string.
 *
 * INPUT:
 *      char *dest: destination string.
 *      const char *str: source string.
 *
 * OUTPUT:
 *      return char string.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U32 bu2str(char *dest, const char *str);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Convert Unicode string to UTF8 string.
 *
 * INPUT:
 *      U8 *src: source string.
 *
 * OUTPUT:
 *      return destination string.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 *bu2u8str(U8 *src);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Convert char string to UNICODE string.
 *
 * INPUT:
 *      char *dest: destination string.
 *      const char *str: source string.
 *
 * OUTPUT:
 *      destination UNICODE string.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U32 bstr2u(char *dest, const char *str);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Convert UTF8 to UNICODE string, not include the count number.
 *
 * INPUT:
 *      U8 *utf8str: UTF8 string.
 *
 * OUTPUT:
 *      return UNICODE string.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 *bu8_2_u(U8 *utf8str);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Convert UNICODE to UTF8.
 *
 * INPUT:
 *      U8 *src: UNICODE string.
 *
 * OUTPUT:
 *      return UTF8 string.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U8 *bu2u8(U8 *src);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Convert U32 to hexadecimal string.
 *
 * INPUT:
 *      U32 num: U32 need to be convert.
 *      char *str: pointer to the string.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void u32_2_hex_str(U32 num, char *str);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Convert hexadecimal string to unsigned int16.
 *
 * INPUT:
 *      const char *str: source string.
 *      U16 *r:
 *
 * OUTPUT:
 *      if source string is hexadecimal, return TRUE, otherwise return FALSE.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
BOOL hex_str_2_u16(const char *str, U16 *r);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Convert hexadecimal string to unsigned int32.
 *
 * INPUT:
 *      const char *str: source string.
 *      U32 *r:
 *
 * OUTPUT:
 *      if source string is hexadecimal, return TRUE, otherwise return FALSE.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
BOOL hex_str_2_u32(const char *str, U32 *r);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      estimate c whether is '\t' or '\n' or '\f' or '\r' or ' '
 *
 * INPUT:
 *      U8 c
 *
 * OUTPUT:
 *      true or false
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
BOOL bspace(U8 c);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Get y powers of x
 *
 * INPUT:
 *      U32 x
 *      U32 y
 *
 * OUTPUT:
 *      int
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
U32 bpow(U32 x, U32 y);

void bint_2_str(S32 num, char *str);
void bu16_to_hex(U16 num, char *str);
U32 bstr_to_int(const char *str);

#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
