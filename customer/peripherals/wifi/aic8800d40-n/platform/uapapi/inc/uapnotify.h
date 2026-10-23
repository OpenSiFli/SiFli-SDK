/** @file wps_notify.h
 *
 *  @brief This file declares all APIs that will be called from MOAL module.
 *  It also defines the data structures used for APIs between MLAN and MOAL.
 *
 *  Copyright (C) 2008-2014, Marvell International Ltd.
 *  All Rights Reserved
 *

 *  MARVELL CONFIDENTIAL

*  The source code contained or described herein and all documents related to

*  the source code ("Material") are owned by Marvell International Ltd or its

*  suppliers or licensors. Title to the Material remains with Marvell International Ltd

*  or its suppliers and licensors. The Material contains trade secrets and

*  proprietary and confidential information of Marvell or its suppliers and

*  licensors. The Material is protected by worldwide copyright and trade secret

*  laws and treaty provisions. No part of the Material may be used, copied,

*  reproduced, modified, published, uploaded, posted, transmitted, distributed,

*  or disclosed in any way without Marvell's prior express written permission.

*

 *  No license under any patent, copyright, trade secret or other intellectual

*  property right is granted to or conferred upon you by disclosure or delivery

*  of the Materials, either expressly, by implication, inducement, estoppel or

*  otherwise. Any license under such intellectual property rights must be

*  express and approved by Marvell in writing.

*
*/

#ifndef UAP_NOTIFY_H
#define UAP_NOTIFY_H

/*
*@brief:UAPNOTIFYFUNC is a callback function.
*@param:state indicate the state of WIFI.
  state = 0xff: wifi isn't configuared
  state = 1: wifi is on
  state = 0: wifi is off
*@note:Don't process too much in this function,don't suspend in this function
*/
typedef void (*UAPNOTIFYFUNC)(unsigned char state);

/*
*@brief:Add callback functions to WIFI application.
*@param:NotifyFunc is the callback function.
*@return:-1 indicate add callback function error;
         0 indicate add callback function sucess.
*/
int UAP_NotifyReg(UAPNOTIFYFUNC NotifyFunc);

#endif /*UAP_NOTIFY_H*/