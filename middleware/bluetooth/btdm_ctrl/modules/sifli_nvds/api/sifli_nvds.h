/**
 ****************************************************************************************
 *
 * @file sifli_nvds.h
 *
 * @brief SiFli NVDS definition.
 *
 * Copyright (C) SiFli 2019-2022
 *
 *
 ****************************************************************************************
 */


#ifndef _SIFLI_NVDS_H
#define _SIFLI_NVDS_H
#include "rtconfig.h"
#include <stdint.h>
#include "compiler.h"


#define NVDS_PATTERN 0x4E564453

#if (NVDS_SUPPORT)
#define SIFLI_NVDS_UNINIT 0
#define SIFLI_NVDS_READY 1
#define SIFLI_NVDS_UPDATING 2
typedef uint8_t sifli_nvds_state_t;

// To
/// Possible Returned Status
enum NVDS_STATUS
{
    /// NVDS status OK
    NVDS_OK,
    /// generic NVDS status KO
    NVDS_FAIL,
    /// NVDS TAG unrecognized
    NVDS_TAG_NOT_DEFINED,
    /// No space for NVDS
    NVDS_NO_SPACE_AVAILABLE,
    /// Length violation
    NVDS_LENGTH_OUT_OF_RANGE,
    /// NVDS parameter locked
    NVDS_PARAM_LOCKED,
    /// NVDS corrupted
    NVDS_CORRUPT
};


typedef uint8_t nvds_tag_len_t;

#define SIFLI_NVDS_TYPE_STACK 1
#define SIFLI_NVDS_TYPE_APP   2
#define SIFLI_NVDS_KEY_APP "SIF_APP"
#define SIFLI_NVDS_KEY_LEN_APP 512
typedef uint8_t sifli_nvds_type_t;

typedef struct
{
    sifli_nvds_type_t type;
} sifli_nvds_get_value_t;

typedef struct
{
    int32_t  status;
    sifli_nvds_type_t type;
    uint16_t len;
    uint8_t value[__ARRAY_EMPTY];
} sifli_nvds_get_value_cnf_t;



typedef struct
{
    sifli_nvds_type_t type;
    uint16_t len;
    uint8_t value[__ARRAY_EMPTY];
} sifli_nvds_set_value_t;

typedef struct
{
    uint8_t status;
    sifli_nvds_type_t type;
} sifli_nvds_set_value_cnf_t;

typedef struct
{
    uint8_t tag;
    uint8_t len;
    uint8_t value[__ARRAY_EMPTY];
} sifli_nvds_tag_value_t;

typedef struct
{
    uint32_t pattern;
    uint16_t used_mem;
    uint16_t writting;
} sifli_nvds_mem_init_t;


/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

typedef int32_t (*sifli_nvds_init_handler)(uint8_t *buffer, uint16_t buf_len);
typedef int32_t (*sifli_nvds_update_handler)(uint8_t *buffer, uint16_t buf_len);
typedef uint8_t (*sifli_nvds_get_handler)(uint8_t tag, nvds_tag_len_t *lengthPtr, uint8_t *buf);
typedef uint8_t (*sifli_nvds_put_handler)(uint8_t tag, nvds_tag_len_t length, uint8_t *buf);
typedef uint8_t (*sifli_nvds_lock_handler)(uint8_t tag);
typedef uint8_t (*sifli_nvds_del_handler)(uint8_t tag);

extern sifli_nvds_init_handler _sifli_nvds_init_handler;
extern sifli_nvds_update_handler _sifli_nvds_update_handler;


#define sifli_nvds_init(buffer, buf_len) _sifli_nvds_init_handler(buffer, buf_len)
#define sifli_nvds_update(buffer, buf_len) _sifli_nvds_update_handler(buffer, buf_len)






int32_t _sifli_nvds_init(uint8_t *buffer, uint16_t buf_len);

int32_t _sifli_nvds_update(uint8_t *buffer, uint16_t buf_len);

uint8_t sifli_nvds_get(uint8_t tag, nvds_tag_len_t *lengthPtr, uint8_t *buf);

uint8_t sifli_nvds_put(uint8_t tag, nvds_tag_len_t length, uint8_t *buf);

uint8_t sifli_nvds_lock(uint8_t tag);

uint8_t sifli_nvds_del(uint8_t tag);


#endif

#endif // _SIFLI_NVDS_H
