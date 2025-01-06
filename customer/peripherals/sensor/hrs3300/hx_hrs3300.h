#ifndef _HX_HRS3300_H_
#define _HX_HRS3300_H_

#include <rtthread.h>
#include <stdbool.h>

#if defined(HR_USING_HRS3300)
    bool Hrs3300_write_reg(uint8_t addr, uint8_t data);
    uint8_t Hrs3300_read_reg(uint8_t addr);
    void Hrs3300_chip_enable(void);
    void Hrs3300_chip_disable(void);
    uint16_t Hrs3300_read_hrs(void);
    uint16_t Hrs3300_read_als(void);
    uint32_t Hrs3300_get_device_handler(void);
    uint8_t Hrs3300_get_dev_addr(void);
    uint8_t Hrs3300_get_dev_id(void);
    rt_err_t hr_hw_init(void);
    void hr_hw_deinit(void);
    void hr_hw_power_onoff(bool onoff);
    rt_err_t hr_hw_self_check(void);
#endif

#endif // _HRS3300_H_
