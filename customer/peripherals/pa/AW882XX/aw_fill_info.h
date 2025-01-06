#ifndef _AW_FILL_INFO_H_
#define _AW_FILL_INFO_H_

#include "aw_base.h"

struct aw_fill_info
{
    aw_dev_index_t dev_index;
    unsigned char i2c_addr;
    aw_phase_sync_t phase_sync; /*Enable phase synchronization flag, default is 0, not enabled*/

    unsigned char mix_chip_count;   /*Number of mixed chips, usually is 1*/
    struct aw_prof_info *prof_info; /*Chip configuration parameters*/

    int (*i2c_read_func)(uint16_t dev_addr, uint8_t reg_addr,
                         uint8_t *pdata, uint16_t len);
    int (*i2c_write_func)(uint16_t dev_addr, uint8_t reg_addr,
                          uint8_t *pdata, uint16_t len);
    void (*reset_gpio_ctl)(bool PinState);
};

#endif
