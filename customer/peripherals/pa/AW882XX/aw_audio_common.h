/************************************************************************
 *Name: aw_audio_common.h
 *Created on: 2020.4.19
 *Author: www.awinic.com.cn
 ************************************************************************/
#ifndef _AW_AUDIO_COMMON_H_
#define _AW_AUDIO_COMMON_H_

#include <stdint.h>
#include "aw_fill_info.h"

typedef enum
{
    START = 0,
    STOP = 1,
} aw_ctrl_t;

typedef enum
{
    AW_HW_IRQ_HANDLE_OFF,
    AW_HW_IRQ_HANDLE_ON,
} aw_hw_irq_handle_t;

typedef enum
{
    AW_MONITOR_HANDLE_OFF,
    AW_MONITOR_HANDLE_ON,
} aw_monitor_handle_t;

typedef enum
{
    AW_FADE_OUT,
    AW_FADE_IN,
} aw_fade_dir_t;

struct aw_iface_cfg
{
    int (*init)(void *aw_info);
    void (*deinit)(aw_dev_index_t dev_index);
    int (*set_profile_byname)(aw_dev_index_t dev_index, char *prof_name);
    int (*ctrl_state)(aw_dev_index_t dev_index, aw_ctrl_t aw_ctrl);
    int (*set_volume)(aw_dev_index_t dev_index, uint32_t volume);
    int (*get_volume)(aw_dev_index_t dev_index, uint32_t *volume);
    int (*set_fade_time)(aw_fade_dir_t fade_dir, uint32_t ms_time);
    int (*get_fade_time)(aw_fade_dir_t fade_dir, uint32_t *ms_time);
    int (*set_fade_step)(aw_dev_index_t dev_index, uint32_t fade_step);
    int (*get_fade_step)(aw_dev_index_t dev_index, uint32_t *fade_step);

    int (*cali)(aw_dev_index_t dev_index);
    int (*cali_re)(aw_dev_index_t dev_index);
    int (*cali_f0)(aw_dev_index_t dev_index);

    aw_hw_irq_handle_t (*get_hw_irq_status)(aw_dev_index_t dev_index);
    void (*irq_handler)(aw_dev_index_t dev_index);
    void (*irq_trigger)(aw_dev_index_t dev_index);

    void (*monitor_work)(aw_dev_index_t dev_index);
    void (*monitor_set_status)(aw_dev_index_t dev_index);
    int (*set_fs)(aw_dev_index_t dev_index, uint32_t value, uint8_t chn_sel);
};

struct aw_debug_attr
{
    int (*reg_show)(aw_dev_index_t dev_index);
    int (*reg_store)(aw_dev_index_t dev_index, uint8_t reg_addr, uint16_t reg_data);
    int (*dsp_show)(aw_dev_index_t dev_index);
    int (*dsp_read)(aw_dev_index_t dev_index, uint16_t dsp_addr, uint16_t *dsp_data);
    int (*dsp_write)(aw_dev_index_t dev_index, uint16_t dsp_addr, uint16_t dsp_data);
    int (*hw_reset)(aw_dev_index_t dev_index);
    int (*soft_reset)(aw_dev_index_t dev_index);
};


extern struct aw_iface_cfg *aw88xx_hal_iface_fops;
extern struct aw_debug_attr *aw88xx_hal_debug_attr;

#endif
