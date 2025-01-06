#ifndef SENSOR_SC_SC7A20_H__
#define SENSOR_SC_SC7A20_H__

#include "board.h"
#include "sensor.h"


/* sc7r30 device structure */
struct sc7r30_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
};


int rt_hw_sc7r30_init(const char *name, struct rt_sensor_config *cfg);

#endif  // SENSOR_SC_SC7A20_H__

