/**
  ******************************************************************************
  * @file   bf0_sibles_serial_trans_service.h
  * @author Sifli software development team
  * @brief Header file - Sibles serial transfer service.
 *
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

#ifdef BSP_BLE_SERIAL_TRANSMISSION

#ifndef _BF0_SIBLES_SERIAL_TRANS_SERVICE_H
#define _BF0_SIBLES_SERIAL_TRANS_SERVICE_H


/**
 * @defgroup Serial_tran Serial transmission service
 * @ingroup profile
 * @{
 */

#define BLE_DFU_CATEID          0x01
#define BLE_CONSOLE_CATEID      0x02
#define BLE_CUSTOMIZE_CATEID    0x10
#define BLE_LOG_CATEID          0x03
#define BLE_WATCHFACE_CATEID    0x04
#define BLE_EQ_CONFIG_CATEID    0x05

/* Remote device should follow serial tranmission data format, detail please refer document.
 *  ---------------------------------------------------
 *  | 1Byte  | 1Byte | 2Byte  | N byte |     | N byte |
 *  |--------|-------|--------|--------|     |--------|
 *  | CateID | Flag  | Length | Data 1 |     | Data N |
 *  |--------|-------|--------|--------|     |--------|
 *
 *  CateID, is categoryID for differernt users.
 *  Flag, is for fragmentation.

 *       0x00: Completed packet
 *       0x01: Fisrt packet.
 *       0x02: Continue packet
 *       0x03: Last packet.
 *  Length Packet length, is only available when flag equal 0x00;
 *  Data, is serial data.
 *
*/


#define serial_tran_svc_uuid { \
    0x73, 0x69, 0x66, 0x6c, \
    0x69, 0x5f, 0x73, 0x64, \
    0x00, 0x00, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00 \
};

#define serial_tran_configure_uuid { \
    0x73, 0x69, 0x66, 0x6c, \
    0x69, 0x5f, 0x73, 0x64, \
    0x00, 0x01, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00 \
}

#define serial_tran_data_uuid { \
    0x73, 0x69, 0x66, 0x6c, \
    0x69, 0x5f, 0x73, 0x64, \
    0x00, 0x02, 0x00, 0x00, \
    0x00, 0x00, 0x00, 0x00 \
}


/**
 * @brief The structure of transmission event.
 */
typedef enum
{
    BLE_SERIAL_TRAN_OPEN,           /**< BLE serial transmission channel opened. */
    BLE_SERIAL_TRAN_DATA,           /**< BLE serial transmission data received. */
    BLE_SERIAL_TRAN_SEND_AVAILABLE, /**< BLE serial transmission can try send data if last send failed. */
    BLE_SERIAL_TRAN_CLOSE,          /**< BLE serial transmission channel closed. */
    BLE_SERIAL_TRAN_ERROR,          /**< BLE serial transmission error due to packet lose or other reason */
} ble_serial_tran_event_t;

typedef enum
{
    BLE_SERIAL_TRAN_ERROR_CATE_ID,
    BLE_SERIAL_TRAN_ERROR_LACK_OF_MEMORY,
    BLE_SERIAL_TRAN_ERROR_ASSEMBLE_OVER_LEN,
    BLE_SERIAL_TRAN_ERROR_ASSEMBLE_ERROR,
} ble_serial_tran_error_event_t;


/**
 * @brief The structure of transmission channel open.
 */
typedef struct
{
    uint8_t handle;        /**< Handle for the transmission channel. */
} ble_serial_open_t;


/**
 * @brief The structure of transmission data.
 */
typedef struct
{
    uint8_t handle;         /**< Handle for the transmission channel. */
    uint8_t cate_id;        /**< CategoryID for different user. */
    uint16_t len;           /**< Data length. */
    uint8_t *data;          /**< Transmission data. */
} ble_serial_tran_data_t;

/**
 * @brief The structure of transmission data.
 */
typedef struct
{
    uint8_t handle;         /**< Handle for the transmission channel. */
    uint8_t cate_id;        /**< CategoryID for different user. */
    uint8_t error;          /**< error code. */
} ble_serial_tran_error_t;

/**
 * @brief The structure of transmission channel close.
 */
typedef struct
{
    uint8_t handle;        /**< Handle for the transmission channel. */
} ble_serial_close_t;


/**
 * @brief The structure of transmission export.
 */
typedef struct
{
    void (*callback)(ble_serial_tran_event_t event, uint8_t *);   /**< User callback. */
    uint8_t cate_id;                                       /**< CategoryID. */
} ble_serial_tran_export_t;



#if defined(_MSC_VER)
#pragma section("SerialTranExport$f",read)
/**
 * @brief The macro that register user categoryID and callback.
 * @param[in] cate_id categoryID.
 * @param[in] callback user callback.
 */
#define BLE_SERIAL_TRAN_EXPORT(cate_id, callback)                                             \
__declspec(allocate("SerialTranExport$f"))const ble_serial_tran_export_t _serial_tran_export= \
    {                                                                          \
        callback,                                                              \
        cate_id,                                                                   \
    }
#pragma comment(linker, "/merge:RSerialTranExport=mytext")
#else
/**
 * @brief The macro that register user categoryID and callback.
 * @param[in] cate_id categoryID.
 * @param[in] callback user callback.
 */
#define BLE_SERIAL_TRAN_EXPORT(cate_id, callback)                \
RT_USED static const ble_serial_tran_export_t _serial_tran_export   \
SECTION("SerialTranExport") =                                              \
{                                                                          \
    callback,                                                              \
    cate_id,                                                                   \
}
#endif


/**
 * @brief Send serial transmission data.
 * @param[in] data serial transmission data.
 * @retval result 0 is successful, others are failed.
 */
int ble_serial_tran_send_data(ble_serial_tran_data_t *data);

/**
 * @brief Enable serial transmission service.
 */
void ble_serial_tran_init(void);


/**
 * @brief Register serial device for BLE serial service.
 * @param[in] name for the device.
 * @retval result 0 is successful, others are failed.
 */
int rt_ble_serial_dev_register(const char *name);

int ble_serial_tran_send_data_advance(uint8_t handle, uint8_t *data, uint16_t data_len);

/**
* @}
*/



#endif // _BF0_SIBLES_SERIAL_TRANS_SERVICE_H
#endif //BSP_BLE_SERIAL_TRANSMISSION

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
