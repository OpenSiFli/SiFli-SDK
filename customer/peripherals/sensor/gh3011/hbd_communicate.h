/*
*File Name: hbd_communicate.h
************************************************************************
*
* Copyright 2018 - 2020 All rights reserved. Goodix Inc.
*
* Description: GH30x sensor control library header.
* Note:
* Identifier abbreviation:  Wnd - Window
*                           Msg - Message
* MODIFICATION HISTORY：    See hbd_ctrl.h
**********************************************************************/

#ifndef __HBD_COMMUNICATE_H__
#define __HBD_COMMUNICATE_H__

#include "hbd_ctrl.h"

/* Hbd commhandler return value type. */
typedef enum
{
    COMM_CMD_ALGO_IN_MCU_HB_START = 0,
    COMM_CMD_ALGO_IN_MCU_HB_STOP,
    COMM_CMD_ALGO_IN_APP_HB_START,
    COMM_CMD_ALGO_IN_APP_HB_STOP,
    COMM_CMD_ALGO_IN_MCU_HRV_START,
    COMM_CMD_ALGO_IN_MCU_HRV_STOP,
    COMM_CMD_ALGO_IN_APP_HRV_START,
    COMM_CMD_ALGO_IN_APP_HRV_STOP,
    COMM_CMD_ADT_SINGLE_MODE_START,
    COMM_CMD_ADT_SINGLE_MODE_STOP,
    COMM_CMD_ADT_CONTINUOUS_MODE_START,
    COMM_CMD_ADT_CONTINUOUS_MODE_STOP,
    COMM_CMD_ALGO_IN_MCU_SPO2_START,
    COMM_CMD_ALGO_IN_MCU_SPO2_STOP,
    COMM_CMD_ALGO_IN_APP_SPO2_START,
    COMM_CMD_ALGO_IN_APP_SPO2_STOP,
    COMM_CMD_INVALID,
} EM_COMM_CMD_TYPE;

/****************************************************************
* Description: parse communicate receive data
* Input:    uchCommInterFaceId: this id from HBD_SetSendDataFunc return,
            uchDataBuffArr: data buff ptr of receive data,
            uchLen: data len of receive data,
* Return: see EM_COMM_CMD_TYPE typedef
******************************************************************/
EM_COMM_CMD_TYPE HBD_CommParseHandler(GU8 uchCommInterFaceId, GU8 uchDataBuffArr[], GU8 uchLen);

/****************************************************************
* Description: Set send data function
* Input:    pSendDataFunc: send data func ptr,
* Return: val > 0:communication interface id,
          HBD_RET_RESOURCE_ERROR: have no resource,
******************************************************************/
GS8 HBD_SetSendDataFunc(void (*pSendDataFunc)(GU8 uchDataBuffArr[], GU8 uchLen));

/****************************************************************
* Description: send rawdata Package by newdata INT
* Input:    stGsAxisValue: gsensor axis value
            emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Return: auto led state flag: 0: success, 1: error
******************************************************************/
GU8 HBD_SendRawdataPackageByNewdataInt(ST_GS_DATA_TYPE *stGsAxisValue, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity);

#endif /* __HBD_COMMUNICATE_H__ */

/********END OF FILE********* (C) COPYRIGHT 2018 .********/
