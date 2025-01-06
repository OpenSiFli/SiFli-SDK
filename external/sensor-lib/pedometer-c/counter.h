/*******************************************************************************
  * @file    counter.h
  * @author  CODOON
  * @version
  * @date    20-2-2019
  * @brief
  *******************************************************************************/

#ifndef _counter_h_
#define _counter_h_

#include "stdint.h"


typedef struct
{
    uint32_t  steps;
    uint32_t  cals;
    uint32_t  dist;
    uint8_t   state;
} SportDataType;

void Sport_Init(void);                         //初始化函数

void Set_Parameter(uint8_t height, uint8_t weight);        //设置身高、体重参数

void Sport_Calculator(int32_t x, int32_t y, int32_t z); //解算函数

void Read_SportData(SportDataType *data);               //读取步数、卡路里、里程、状态数据,赋值给DATA

uint8_t Set_SportData(uint32_t step, uint32_t cal, uint32_t dis); //设置运动数据，成功返回1，失败返回0

uint32_t Get_Version(void);                             //获取算法库的版本信息


#endif

