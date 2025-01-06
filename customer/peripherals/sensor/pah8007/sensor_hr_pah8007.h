/**
  ******************************************************************************
  * @file   sensor_hr_pah8007.h
  * @author wk software development team
  ******************************************************************************
*/


#ifndef SENSOR_HR_PAH8007_H__
#define SENSOR_HR_PAH8007_H__

#include "board.h"
#include "sensor.h"


/* pah8007 device structure */
struct pah8007_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
};


int rt_hw_pah8007_init(const char *name, struct rt_sensor_config *cfg);

#endif  // SENSOR_HR_PAH8007_H__
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
