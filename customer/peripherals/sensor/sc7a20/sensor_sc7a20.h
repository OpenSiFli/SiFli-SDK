#ifndef SENSOR_SC_SC7A20_H__
#define SENSOR_SC_SC7A20_H__

#include "board.h"
#include "sensor.h"


/* sc7a20 device structure */
struct sc7a20_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
};

int rt_hw_sc7a20_register(const char *name, struct rt_sensor_config *cfg);
int rt_hw_sc7a20_init(void);
int rt_hw_sc7a20_deinit(void);


#endif  // SENSOR_SC_SC7A20_H__

