#ifndef _VC32S_H_
#define _VC32S_H_

#include <rtthread.h>
#include <stdbool.h>

rt_err_t hr_hw_init(void);
void hr_hw_deinit(void);
void hr_hw_power_onoff(bool  onoff);
rt_err_t hr_hw_self_check(void);
uint32_t vc32s_get_i2c_handle(void);
uint8_t vc32s_get_dev_addr(void);
uint8_t vc32s_get_dev_id(void);

#endif // _VC32S_H_