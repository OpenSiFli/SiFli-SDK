/**
  ******************************************************************************
  * @file   pbap_public.h
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

#ifndef _PBAP_PUBLIC_H_
#define _PBAP_PUBLIC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Bit mask definition for remote device supported repositories */
#define PBAP_REP_LOCAL (1<<0)   /* Mask for Local Repository */
#define PBAP_REP_SIM  (1<<1)    /* Mask for SIM1 Repository  */

/* PhoneBook repositories */
typedef enum
{
    PBAP_CURRENT,
    PBAP_LOCAL,
    PBAP_SIM1,
    PBAP_UNKNOWN_REPO
} BTS2E_PBAP_PHONE_REPOSITORY;

/* Phone Book Objects */
typedef enum
{
    PBAP_TELECOM,
    PBAP_PB,
    PBAP_ICH,
    PBAP_OCH,
    PBAP_MCH,
    PBAP_CCH,
    PBAP_UNKNOWN_PHONEBOOK
} BTS2E_PBAP_PHONE_BOOK;

/* Order values for use with the PullvCardListing function. */
typedef enum
{
    PBAP_ORDER_IDX = 0x00,      /* Indexed. */
    PBAP_ORDER_ALPHA = 0x01,    /* Alphanumeric */
    PBAP_ORDER_PHONE = 0x02     /* Phonetic */
} BTS2E_PBAP_ORDER_VALUES;

/* Search Attributes to use with the PullvCardListing function. */
typedef enum
{
    PBAP_SEARCH_NAME = 0x00,    /* Name. */
    PBAP_SEARCH_NUMBER = 0x01,  /* Number */
    PBAP_SEARCH_SOUND = 0x02    /* Sound */
} BTS2E_PBAP_SEARCH_VALUES;

/* vCard formats to use with the PullvCardEntry and PullPhonebook functions. */
typedef enum
{
    PBAP_FORMAT_21 = 0x00,  /* vCard 2.1. */
    PBAP_FORMAT_30 = 0x01,  /* vCard 3.0 */
} BTS2E_PBAP_VCARD_FORMAT;


/* PBAP application default parameters values */
#define PBAP_ORDER_DEFAULT              PBAP_ORDER_IDX     /* Order Index */
#define PBAP_SEARCH_ATTR_DEFAULT        PBAP_SEARCH_NAME   /* Search Attribute [Name] */
/* Maximum List Count, The value 65535 means that the number of entries is not restricted */
#define PBAP_MAX_LIST_DEFAULT           65535
#define PBAP_LIST_START_OFFSET_DEFAULT  0   /* List Start Offset, the offset of the first entry */
#define PBAP_FORMAT_DEFAULT             PBAP_FORMAT_21  /* The format vCard 2.1 is the default format */


/* Filter Attribute Mask Values */
#define PBAP_FILTER_VERSION     (1<<0)  /* vCard Version */
#define PBAP_FILTER_FN          (1<<1)  /* Formatted Name (FN) */
#define PBAP_FILTER_N           (1<<2)  /* Structured Presentation of Name (N) */
#define PBAP_FILTER_PHOTO       (1<<3)  /* Photo Associated with the name (PHOTO) */
#define PBAP_FILTER_BDAY        (1<<4)  /* Birthday (BDAY) */
#define PBAP_FILTER_ADR         (1<<5)  /* Delivery Address (ADR) */
#define PBAP_FILTER_LABEL       (1<<6)  /* Delivery Label (LABEL) */
#define PBAP_FILTER_TEL         (1<<7)  /* Telephone Number (TEL) */
#define PBAP_FILTER_EMAIL       (1<<8)  /* Electronic Mail Address (EMAIL) */
#define PBAP_FILTER_MAILER      (1<<9)  /* Electronic Mail (MAILER) */
#define PBAP_FILTER_TZ          (1<<10) /* Time Zone (TZ) */
#define PBAP_FILTER_GEO         (1<<11) /* Geographic Position (GEO) */
#define PBAP_FILTER_TITLE       (1<<12) /* Job Title (TITLE) */
#define PBAP_FILTER_ROLE        (1<<13) /* Role within the Organization (ROLE) */
#define PBAP_FILTER_LOGO        (1<<14) /* Organization Logo (LOGO) */
#define PBAP_FILTER_AGENT       (1<<15) /* vCard of person representing (AGENT) */
#define PBAP_FILTER_ORG         (1<<16) /* Name of Organization (ORG) */
#define PBAP_FILTER_NOTE        (1<<17) /* Comments (NOTE) */
#define PBAP_FILTER_REV         (1<<18) /* vCard Revision (REV) */
#define PBAP_FILTER_SOUND       (1<<19) /* Pronunciation of Name (SOUND) */
#define PBAP_FILTER_URL         (1<<20) /* Uniform Resource Locator (URL) */
#define PBAP_FILTER_UID         (1<<21) /* Unique ID (UID) */
#define PBAP_FILTER_KEY         (1<<22) /* Public Encryption Key (KEY) */
#define PBAP_FILTER_NICKNAME    (1<<23) /* Nickname (NICKNAME) */
#define PBAP_FILTER_CATAGORY    (1<<24) /* Categories (CATAGORIES) */
#define PBAP_FILTER_PROID       (1<<25) /* Product ID (PROID) */
#define PBAP_FILTER_CLASS       (1<<26) /* Class Information (CLASS) */
#define PBAP_FILTER_SORT        (1<<27) /* String used for sorting operations (SORT-STRING) */
#define PBAP_FILTER_CALL_DATETIME (1<<28) /* Timestamp (X-IRMC-CALL-DATETIME) */

/* vCard version 2.1 Mandatory attributes */
#define PBAP_FILTER_VCARD21 (PBAP_FILTER_N | PBAP_FILTER_TEL)
/* vCard version 3.0 Mandatory attributes */
#define PBAP_FILTER_VCARD30 (PBAP_FILTER_FN | PBAP_FILTER_TEL)

#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
