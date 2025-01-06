
/**
  ******************************************************************************
  * @file   hid_api.h
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




#ifndef _HID_API_H_
#define _HID_API_H_

#ifdef __cplusplus
extern "C" {
#endif


/* HID device roles */
#define HID_Device   ((U16)0x00)
#define HID_Host     ((U16)0x01)
#define HID_INVLD_ROLE         ((U8)0x03)


#define HID_ABS  0x00 // bit2 - 0 absolute, 1 relative
#define HID_REL  0x04

#define HID_NWRP 0x00 // bit3 - 0 no wrap, 1 wrap
#define HID_WRAP 0x08

#define HID_LINE 0x00 // bit4 - 0 linear, 1 non linear
#define HID_NLIN 0x10

#define HID_PREF 0x00 // bit5 - 0 preferred state, 1 no preferred
#define HID_NPRF 0x20

#define HID_NNUL 0x00 // bit6 - 0 no null position, 1 null state
#define HID_NULL 0x40

#define HID_NVLA 0x00 // bit7 - 0 non volatile, 1 volatile
#define HID_VOLA 0x80

#define HID_BOOT_PROTOCOL_MODE      0x0
#define HID_REPORT_PROTOCOL_MODE    0x1

#define HID_BOOT_PROTOCOL_KEYBOARD_REPORT_ID  0x01 // 9-octets, 1-octet report id + standard 8-octet keyboard boot report
#define HID_BOOT_PROTOCOL_MOUSE_REPORT_ID     0x02 // 4-octets, 1-octet report id + standard 3-octet mouse boot report

#define HID_MOUSE_REPORT_ID 0x02
#define HID_CONSUMER_REPORT_ID 0x03


typedef struct
{
    void *next_struct;
    U8   *payload;
    U16  data_len;
} BTS2S_HID_DESCRIPTOR_LIST;

typedef struct
{
    U16 type;
} BTS2S_HID_RESET;

typedef struct
{
    U16 type;
    U16 tid;
    U16 local_role;
} BTS2S_HID_ENB_REQ;

typedef struct
{
    U16 type;
    U16 tid;
    U16 data_len;
    U8  *data_ptr;
    BOOL send_on_interrupt;
} BTS2S_HID_SEND_REPORT_REQ;


typedef struct
{
    U16 type;
    U8  res;
} BTS2S_HID_ENB_CFM;

typedef struct
{
    U16 type;
    U8  res;
} BTS2S_HID_DISB_CFM;


typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd;
    U16          local_psm;
    U8  res;
} BTS2S_HID_DISC_IND;

typedef struct
{
    U16 type;
    BTS2S_BD_ADDR bd;
    U16          local_psm;
    U8  res;
} BTS2S_HID_DISC_CFM;

typedef struct
{
    U16 type;
    U8  mode;
    U16  cid;
} BTS2S_HID_MODE_CHANGE_IND;

typedef struct
{
    U16 type;
} BTS2S_HID_DISB_REQ;


typedef struct
{
    U16          type;
    U16          tid;
    BTS2S_BD_ADDR bd;
    U16          rmt_role;
    U16          local_role;
} BTS2S_HID_CONN_REQ;


typedef struct
{
    U16 type;
} BTS2S_HID_DISC_REQ;


typedef struct
{
    U16          type;
    U8           res;
    BTS2S_BD_ADDR bd;
    U16          mfs;
    U16          local_psm;
} BTS2S_HID_CONN_CFM;


typedef struct
{
    U16          type;
    BTS2S_BD_ADDR bd;
    U16          local_psm;
} BTS2S_HID_CONN_IND;


typedef enum BTS2E_HID_MSG_TAG
{
    BTS2MD_HID_ENB_REQ = BTS2MD_START,
    BTS2MD_HID_DISB_REQ,
    BTS2MD_HID_CONN_REQ,
    BTS2MD_HID_DISC_REQ,
    BTS2MD_HID_HANDSHAKE,
    BTS2MD_HID_CONTROL_REQ,
    BTS2MD_HID_DATA_REQ,
    BTS2MD_HID_DATA_RSP,
    BTS2MD_HID_RESET_REQ,
    BTS2MD_HID_HANDSHAKE_RSP,

    BTS2MU_HID_CONN_IND = BTS2MU_START,
    BTS2MU_HID_DISC_IND,
    BTS2MU_HID_DISC_CFM,
    BTS2MU_HID_MODE_CHANGE_IND,
    BTS2MU_HID_ENB_CFM,
    BTS2MU_HID_DISB_CFM,
    BTS2MU_HID_CONN_CFM,
    BTS2MU_HID_CONTROL_IND,
    BTS2MU_HID_GET_REPORT_IND,
    BTS2MU_HID_SET_REPORT_IND,
    BTS2MU_HID_GET_PROTOCOL_IND,
    BTS2MU_HID_SET_PROTOCOL_IND,
    BTS2MD_HID_DATA_IND,
    BTS2MD_HID_UNKNOWN,
} HIDMSG;


#define BTS2MD_HIGHEST_HID_RECV_MSG_NUM (BTS2MD_HID_HANDSHAKE_RSP + 1)


typedef enum
{
    HID_MSG_TYPE_HANDSHAKE = 0x00,
    HID_MSG_TYPE_HID_CONTROL = 0x01,
    HID_MSG_TYPE_GET_REPORT = 0x04,
    HID_MSG_TYPE_SET_REPORT = 0x05,
    HID_MSG_TYPE_GET_PROTOCOL = 0x06,
    HID_MSG_TYPE_SET_PROTOCOL = 0x07,
    HID_MSG_TYPE_DATA = 0x0a
} hid_message_type_enum_t;


typedef enum
{
    HID_HANDSHAKE_PARAM_TYPE_SUCCESSFUL = 0x00,        // This code is used to acknowledge requests. A device that has correctly received SET_REPORT, SET_IDLE or SET_PROTOCOL payload transmits an acknowledgment to the host.
    HID_HANDSHAKE_PARAM_TYPE_NOT_READY,                // This code indicates that a device is too busy to accept data. The Bluetooth HID Host should retransmit the data the next time it communicates with the device.
    HID_HANDSHAKE_PARAM_TYPE_ERR_INVALID_REPORT_ID,    // Invalid report ID transmitted.
    HID_HANDSHAKE_PARAM_TYPE_ERR_UNSUPPORTED_REQUEST,  // The device does not support the request. This result code shall be used if the HIDP message type is unsupported.
    HID_HANDSHAKE_PARAM_TYPE_ERR_INVALID_PARAMETER,    // A parameter value is out of range or inappropriate for the request.
    HID_HANDSHAKE_PARAM_TYPE_ERR_UNKNOWN = 0x0E,       // Device could not identify the error condition.
    HID_HANDSHAKE_PARAM_TYPE_ERR_FATAL = 0x0F          // Restart is essential to resume functionality
} hid_handshake_param_type_enum_t;


typedef enum
{
    HID_CONTROL_PARAM_NOP_DEPRECATED = 0,              // Deprecated: No Operation.
    HID_CONTROL_PARAM_HARD_RESET_DEPRECATED,           // Deprecated: Device performs Power On System Test (POST) then initializes all internal variables and initiates normal operations.
    HID_CONTROL_PARAM_SOFT_RESET_DEPRECATED,           // Deprecated: Device initializes all internal variables and initiates normal operations.
    HID_CONTROL_PARAM_SUSPEND = 0x03,                  // Go to reduced power mode.
    HID_CONTROL_PARAM_EXIT_SUSPEND,                    // Exit reduced power mode.
    HID_CONTROL_PARAM_VIRTUAL_CABLE_UNPLUG
} hid_control_param_enum_t;


typedef enum
{
    HID_REPORT_TYPE_RESERVED = 0,
    HID_REPORT_TYPE_INPUT,
    HID_REPORT_TYPE_OUTPUT,
    HID_REPORT_TYPE_FEATURE
} hid_report_type_enum_t;


typedef struct
{
    U16 type;
    U8 param;
    U8 data_len;
    U8 *data;
} BTS2S_HID_CMD_IND;

typedef struct
{
    U16 type;
} BTS2S_HID_RESET_REQ;


typedef BTS2S_HID_CMD_IND BTS2S_HID_CONTROL_IND;
typedef BTS2S_HID_CMD_IND BTS2S_HID_GET_REPORT_IND;
typedef BTS2S_HID_CMD_IND BTS2S_HID_SET_REPORT_IND;
typedef BTS2S_HID_CMD_IND BTS2S_GET_PROTOCOL_IND;
typedef BTS2S_HID_CMD_IND BTS2S_SET_PROTOCOL_IND;
typedef BTS2S_HID_CMD_IND BTS2S_HID_DATA_IND;



/*
Description:

Input:

Time:2023/05/04 09:31:47

Author:zhengyu

Modify:
*/
BTS2S_HID_DESCRIPTOR_LIST *hid_get_descriptor_list(void);


/*
Description:
    This signal is used to enable a service and make it accessible from
    a remote device.
Input:
    U16 tid: task id.
    U16 local_role: the role of this device.
Time:2023/04/20 16:36:27

Author:zhengyu

Modify:
*/
void hid_enb_req(U16 tid, U16 local_role);


/*
Description:
    upper layer send hid report
Input:
    U16 tid: task id.
    data_len:data length
    data:report data
    send_on_interrupt_channel:send at interrupt channel
Time:2023/04/26 16:17:19

Author:zhengyu

Modify:
*/
void hid_send_report_req(U16 tid, U16 data_len, U8 *data, BOOL send_on_interrupt_channel);


/*
Description:
    This signal is used to disable a service and make it accessible from
    a remote device.
Input:
    U16 tid: task id.
    U16 local_role: the role of this device.
Time:2023/04/20 16:36:27

Author:zhengyu

Modify:
*/
void hid_disb_req(void);


/*
Description:
    upper layer connect hid
Input:
    U16 tid: task id.
    BTS2S_BD_ADDR bd: address of device to connect to.
    U16 rmt_role: wanted role of the device to connect to.
    U16 local_role: the device's own role.
Time:2023/04/26 16:20:13

Author:zhengyu

Modify:
*/
void hid_conn_req(U16 tid,
                  BTS2S_BD_ADDR bd,
                  U16 rmt_role,
                  U16 local_role);


/*
Description:
    upper layer disconnect hid
Input:

Time:2023/04/26 16:19:21

Author:zhengyu

Modify:
*/
void hid_disc_req(void);

void hid_reset_req(void);




#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/