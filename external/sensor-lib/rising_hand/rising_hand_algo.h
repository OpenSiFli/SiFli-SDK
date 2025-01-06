#ifndef GESTURES_ALOG_H__
#define GESTURES_ALOG_H__

#include "stdint.h"
#include "stdio.h"
//#include "app.h"
#include "math.h"

typedef struct
{
    float  mpss[3];
} rising_hand_data_t;


typedef enum
{
    EVENT_RAISE_HAND,      // 抬手亮屏
    EVENT_FAILING_HAND,    // 放手关屏
} rising_hand_type;

//can be used in 25HZ and 28HZ

//方向要求
/************************************
**正面水平0度，正面朝上平放 Z=9.8F
**正面垂直90度，正立放置 X=-9.8F
**正面左翻90度，正面向左 Y=9.8F
*************************************
**/
extern void SmartWear_SportHealth_Gesture_State_Update_Callback(rising_hand_type);
extern void Rising_hand_algorithm_data_handle(rising_hand_data_t sensor_data);
extern uint32_t get_algo_rise_hand_lib_version_major(void);
extern uint32_t get_algo_rise_hand_lib_version_minor(void);
extern uint32_t get_algo_rise_hand_lib_version_patch(void);

#endif











