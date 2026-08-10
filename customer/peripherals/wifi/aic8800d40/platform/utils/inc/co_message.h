/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __COMESSAGE_H__
#define __COMESSAGE_H__

#define KEY_ID_BUILD(name, func)    ((uint32_t)((name) | (func)))
#define KEY_NAME_GET(key_id)        (key_id & 0xFFFF0000)
#define KEY_FUNC_GET(key_id)        ((key_id) & 0xFFFF)

//key name
#define APP_KEY_NUM0             (0<<16)
#define APP_KEY_NUM1             (1<<16)
#define APP_KEY_NUM2             (2<<16)
#define APP_KEY_NUM3             (3<<16)
#define APP_KEY_NUM4             (4<<16)
#define APP_KEY_NUM5             (5<<16)
#define APP_KEY_NUM6             (6<<16)
#define APP_KEY_NUM7             (7<<16)
#define APP_KEY_NUM8             (8<<16)
#define APP_KEY_NUM9             (9<<16)

#define APP_KEY_POWER            (10<<16)
#define APP_KEY_PLAY             (11<<16)
#define APP_KEY_NEXT             (12<<16)
#define APP_KEY_PREV             (13<<16)
#define APP_KEY_VOLADD           (14<<16)
#define APP_KEY_VOLSUB           (15<<16)
#define APP_KEY_EQ               (16<<16)
#define APP_KEY_MUTE             (17<<16)
#define APP_KEY_GSENSOR          (18<<16)
#define APP_KEY_NULL             (19<<16)
//msg
#define APP_MSG_RTC              (32<<16)
#define APP_MSG_WAIT_TIMEOUT     (33<<16)
#define APP_MSG_SLEEP            (34<<16)
#define APP_MSG_WAKE_UP          (35<<16)
#define APP_MSG_CHARGING         (36<<16)
#define APP_MSG_BATT             (37<<16)
#define APP_MSG_LOW_POWER        (38<<16)
#define APP_MSG_POWER_DOWN       (39<<16)

//key function
#define APP_KEY_REPEAT           (1)
#define APP_KEY_HOLD             (2)
#define APP_KEY_DOWN             (3)
#define APP_KEY_UP               (4)
#define APP_KEY_PRESS            (5)
#define APP_KEY_HOLD_2S          (6)
#define APP_KEY_HOLD_3S          (7)
#define APP_KEY_HOLD_5S          (8)
#define APP_KEY_DOUBLE_CLICK     (9)
#define APP_KEY_TRIPLE_CLICK     (10)

#endif
