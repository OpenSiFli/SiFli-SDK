/**
  ******************************************************************************
  * @file   bf0_sibles_nvds.h
  * @author Sifli software development team
  * @brief SiFli NVDS definition.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2020 - 2021,  Sifli Technology
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

#ifndef _BF0_SIBLES_NVDS_H
#define _BF0_SIBLES_NVDS_H

#include "os_adaptor.h"
#include "bf0_ble_common.h"
#include "bf0_ble_gap.h"
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
    NVDS_CORRUPT,
    /// NVDS operation is pending.
    NVDS_PENDING,
    /// Not support
    NVDS_NOT_SUPPORT,
};

/* Event will notify users use BLE_EVENT_REGISTER to register. */
enum ble_nvds_event
{
    BLE_NVDS_ASYNC_READ_CNF = BLE_NVDS_TYPE,  /* @see sifli_nvds_get_value_cnf_t */
    BLE_NVDS_ASYNC_WRITE_CNF,                 /* @see sifli_nvds_set_value_cnf_t */
    BLE_NVDS_ASYNC_READ_TAG_CNF,              /* @see sifli_nvds_read_tag_cnf_t */
    BLE_NVDS_ASYNC_WRITE_TAG_CNF,             /* @see sifli_nvds_write_tag_cnf_t */
    BLE_NVDS_AYSNC_FLUSH_TAG_CNF,             /* NULL parameter*/
    BLE_NVDS_AYSNC_UPDATE_ADDR_CNF            /* @see sifli_nvds_update_addr_rsp_t */
};



/// List of parameters identifiers
enum stack_nvds_tag
{
    /// Definition of the tag associated to each parameters
    /// Local Bd Address
    NVDS_STACK_TAG_BD_ADDRESS                 = 0x01,
    NVDS_STACK_LEN_BD_ADDRESS                 = 6,
    /// Radio Drift
    NVDS_STACK_TAG_LPCLK_DRIFT                = 0x07,
    NVDS_STACK_LEN_LPCLK_DRIFT                = 2,
    /// External wake-up time
    NVDS_STACK_TAG_EXT_WAKEUP_TIME            = 0x0D,
    NVDS_STACK_LEN_EXT_WAKEUP_TIME            = 2,
    /// Oscillator wake-up time
    NVDS_STACK_TAG_OSC_WAKEUP_TIME            = 0x0E,
    NVDS_STACK_LEN_OSC_WAKEUP_TIME            = 2,
    /// Enable sleep mode
    NVDS_STACK_TAG_SLEEP_ENABLE               = 0x11,
    NVDS_STACK_LEN_SLEEP_ENABLE               = 1,
    /// Enable External Wakeup
    NVDS_STACK_TAG_EXT_WAKEUP_ENABLE          = 0x12,
    NVDS_STACK_LEN_EXT_WAKEUP_ENABLE          = 1,

    /// Enable/disable scanning for extended advertising PDUs
    NVDS_STACK_TAG_SCAN_EXT_ADV               = 0x16,
    NVDS_STACK_LEN_SCAN_EXT_ADV               = 1,

    /// Sleep algorithm duration
    NVDS_STACK_TAG_SLEEP_ALGO_DUR             = 0x2E,
    NVDS_STACK_LEN_SLEEP_ALGO_DUR            = 2,
    /// Tracer configuration
    NVDS_STACK_TAG_TRACER_CONFIG              = 0x2F,
    NVDS_STACK_LEN_TRACER_CONFIG              = 4,

    /// SC Private Key (Low Energy)
    NVDS_STACK_TAG_LE_PRIVATE_KEY_P256        = 0x80,
    NVDS_STACK_LEN_PRIVATE_KEY_P256           = 32,
    /// SC Public Key (Low Energy)
    NVDS_STACK_TAG_LE_PUBLIC_KEY_P256         = 0x81,
    NVDS_STACK_LEN_PUBLIC_KEY_P256            = 64,
    /// SC Debug: Used Fixed Private Key from NVDS (Low Energy)
    NVDS_STACK_TAG_LE_DBG_FIXED_P256_KEY      = 0x82,
    NVDS_STACK_LEN_DBG_FIXED_P256_KEY         = 1,

    /// LE Coded PHY 500 Kbps selection
    NVDS_STACK_TAG_LE_CODED_PHY_500           = 0x85,
    NVDS_STACK_LEN_LE_CODED_PHY_500           = 1,

    /// BTC used assert or assert msg
    NVDS_STACK_TAG_ASSERT_MSG_ENABLE                = 0x86,
    NVDS_STACK_LEN_ASSERT_MSG_ENABLE               = 1,

    /// Application specific
    NVDS_STACK_TAG_APP_SPECIFIC_FIRST         = 0x90,
    NVDS_STACK_TAG_APP_SPECIFIC_LAST          = 0xAF,
};


enum app_nvds_tag
{
    /// BD Address
    NVDS_APP_TAG_BD_ADDRESS                 = NVDS_STACK_TAG_APP_SPECIFIC_FIRST + 0x01,
    NVDS_APP_LEN_BD_ADDRESS                 = 6,

    /// Device Name
    NVDS_APP_TAG_DEVICE_NAME                = NVDS_STACK_TAG_APP_SPECIFIC_FIRST + 0x02,
    NVDS_APP_LEN_DEVICE_NAME                = 62,

    /// Local device Identity resolving key
    NVDS_APP_TAG_LOC_IRK                    = NVDS_STACK_TAG_APP_SPECIFIC_FIRST + 0x1E,
    NVDS_APP_LEN_LOC_IRK                    = GAP_KEY_LEN,


    NVDS_APP_TAG_APP_EDN                    = NVDS_STACK_TAG_APP_SPECIFIC_LAST,
};


#define SIFLI_NVDS_TYPE_STACK 1
#define SIFLI_NVDS_TYPE_APP   2
#define SIFLI_NVDS_TYPE_CM   3
#define SIFLI_NVDS_TYPE_BT_HOST 4
#define SIFLI_NVDS_TYPE_BT_CM 5
typedef uint8_t sifli_nvds_type_t;


#define SIFLI_NVDS_KEY_LEN_APP 512
#define SIFLI_NVDS_KEY_LEN_STACK 512
#define SIFLI_NVDS_KEY_LEN_CM 3072
#define SIFLI_NVDS_KEY_LEN_BT_CM 512


typedef uint8_t nvds_tag_len_t;


typedef struct
{
    uint8_t tag;
    uint8_t len;
    uint8_t value[__ARRAY_EMPTY];
} sifli_nvds_tag_value_t;

typedef struct
{
    sifli_nvds_type_t type;
} sifli_nvds_get_value_t;

typedef struct
{
    os_status_t status;
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
    uint8_t is_flush;
    ble_common_update_type_t type;
    sifli_nvds_tag_value_t value;
} sifli_nvds_write_tag_t;

typedef struct
{
    uint8_t tag;
    uint8_t status;
} sifli_nvds_write_tag_cnf_t;

typedef struct
{
    uint8_t tag;
    uint16_t length;
} sifli_nvds_read_tag_t;


typedef struct
{
    uint8_t tag;
    uint8_t status;
    uint16_t length;
    uint8_t buffer[0];
} sifli_nvds_read_tag_cnf_t;

typedef struct
{
    bd_addr_t addr;
    ble_common_update_type_t u_type;
    uint8_t is_flush;
} sifli_nvds_update_addr_req_t;

typedef struct
{
    uint8_t status;
} sifli_nvds_update_addr_rsp_t;




void sifli_nvds_init(void);

void sifli_nvds_handler(void *header, uint8_t *data_ptr, uint16_t param_len);

/* Return NVDS_PENDING indicates the operation is async. Need wait event callback. @see enum ble_nvds_event*/

uint8_t sifli_nvds_read(sifli_nvds_type_t type, uint16_t *len, uint8_t *buffer);

size_t sifli_nvds_flash_read(const char *key, void *value_buf, size_t buf_len);

uint8_t sifli_nvds_write(sifli_nvds_type_t type, uint16_t len, uint8_t *ptr);

uint8_t sifli_nvds_flash_write(const char *key, const void *value_buf, size_t buf_len);

uint8_t sifli_nvds_write_tag_value(sifli_nvds_write_tag_t *tag);

uint8_t sifli_nvds_read_tag_value(sifli_nvds_read_tag_t *tag, uint8_t *tag_buffer);

uint8_t sifli_nvds_flush(void);

uint8_t ble_nvds_update_address(bd_addr_t *addr, ble_common_update_type_t u_type, uint8_t is_flush);

uint8_t sifli_nvds_get_default_vaule(uint8_t *ptr, uint16_t *len);


#if defined(SOC_SF32LB56X)
    void ble_nvds_enable_btc_assert_msg(uint8_t is_enable);
#endif // SOC_SF32LB56X

void sifli_hci_log_enable(bool is_on);


// Porting APIs
uint8_t sifli_nvds_flash_adaptor_delete(const char *key);
uint8_t sifli_nvds_flash_adaptor_init(void);
uint8_t sifli_nvds_flash_adaptor_write(const char *key, const void *value_buf, size_t buf_len);
size_t sifli_nvds_flash_adaptor_read(const char *key, void *value_buf, size_t buf_len);




#endif // _BF0_SIBLES_NVDS_H
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
