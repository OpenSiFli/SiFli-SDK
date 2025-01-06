/*
* aw_monitor.c
*
* Copyright (c) 2021 AWINIC Technology CO., LTD
*
* Author: <zhaolei@awinic.com>
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aw882xx.h"
#include "aw_monitor.h"
#include "aw_base.h"
#include "aw_device.h"
#include "aw_profile_process.h"

#ifdef AW_MONITOR

/*****************************************************
* device monitor
*****************************************************/
static int aw_get_hmute(struct aw_device *aw_dev)
{
    int ret = -1;
    unsigned int reg_val = 0;
    struct aw_mute_desc *desc = &aw_dev->mute_desc;

    aw_dev_dbg(aw_dev->dev_index, "enter");

    ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->reg, &reg_val);
    if (ret < 0)
    {
        return ret;
    }

    if (reg_val & (~desc->mask))
    {
        ret = 1;
    }
    else
    {
        ret = 0;
    }
    return ret;
}

static int aw_monitor_get_voltage(struct aw_device *aw_dev, unsigned int *vol)
{
    int ret = -1;
    uint16_t local_vol = 0;
    struct aw_voltage_desc *desc = &aw_dev->voltage_desc;

    if (desc->reg == AW_REG_NONE)
    {
        aw_dev_info(aw_dev->dev_index, "read voltage from reg unsupported");
        return -EINVAL;
    }
    else
    {
        ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->reg, vol);
        if (ret < 0)
        {
            aw_dev_err(aw_dev->dev_index, "read voltage failed!");
            return ret;
        }
        local_vol = ((*vol) * desc->vbat_range) / desc->int_bit;
        *vol = local_vol;
    }

    aw_dev_info(aw_dev->dev_index, "chip voltage is %d", *vol);

    return 0;
}

static int aw_monitor_get_temperature(struct aw_device *aw_dev, int *temp)
{
    int ret = -1;
    unsigned int reg_val = 0;
    uint16_t local_temp;
    struct aw_temperature_desc *desc = &aw_dev->temp_desc;

    if (desc->reg == AW_REG_NONE)
    {
        aw_dev_info(aw_dev->dev_index, "read temperature from reg unsupported");
        return -EINVAL;
    }
    else
    {
        ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->reg, &reg_val);
        if (ret < 0)
        {
            aw_dev_err(aw_dev->dev_index, "get temperature failed!");
            return ret;
        }
        aw_dev_info(aw_dev->dev_index, "reg val is 0x%04x", reg_val);
        local_temp = reg_val;
        if (local_temp & (~desc->sign_mask))
        {
            local_temp = local_temp | desc->neg_mask;
        }
        *temp = (int16_t)local_temp;
    }

    aw_dev_info(aw_dev->dev_index, "chip temperature = %d", *temp);
    return 0;
}

static int aw_monitor_get_temp_and_vol(struct aw_device *aw_dev)
{
    int ret = -1;
    struct aw_monitor_desc *monitor = &aw_dev->monitor_desc;
    unsigned int voltage = 0;
    int current_temp = 0;

    ret = aw_monitor_get_voltage(aw_dev, &voltage);
    if (ret < 0)
    {
        return ret;
    }
    ret = aw_monitor_get_temperature(aw_dev, &current_temp);
    if (ret < 0)
    {
        return ret;
    }
    monitor->vol_trace.sum_val += voltage;
    monitor->temp_trace.sum_val += current_temp;
    monitor->samp_count++;

    return 0;
}

static int aw_monitor_first_get_data_form_table(struct aw_device *aw_dev,
        struct aw_table_info table_info,
        struct aw_monitor_trace *data_trace)
{
    int i;

    if (table_info.aw_table == NULL)
    {
        aw_dev_err(aw_dev->dev_index, "table_info.aw_table is null");
        return -EINVAL;
    }

    for (i = 0; i < table_info.table_num; i++)
    {
        if (data_trace->sum_val >= table_info.aw_table[i].min_val)
        {
            memcpy(&data_trace->aw_table, &table_info.aw_table[i],
                   sizeof(struct aw_table));
            break;
        }
    }

    return 0;
}

static int aw_monitor_trace_data_from_table(struct aw_device *aw_dev,
        struct aw_table_info table_info,
        struct aw_monitor_trace *data_trace)
{
    int i;

    if (table_info.aw_table == NULL)
    {
        aw_dev_err(aw_dev->dev_index, "table_info.aw_table is null");
        return -EINVAL;
    }

    for (i = 0; i < table_info.table_num; i++)
    {
        if (data_trace->sum_val >= table_info.aw_table[i].min_val &&
                data_trace->sum_val <= table_info.aw_table[i].max_val)
        {
            memcpy(&data_trace->aw_table, &table_info.aw_table[i],
                   sizeof(struct aw_table));
            break;
        }
    }

    return 0;
}

static int aw_monitor_get_data_from_table(struct aw_device *aw_dev,
        struct aw_table_info table_info,
        struct aw_monitor_trace *data_trace,
        uint32_t aplha)
{
    struct aw_monitor_desc *monitor = &aw_dev->monitor_desc;

    if (monitor->first_entry == AW_FIRST_ENTRY)
    {
        return aw_monitor_first_get_data_form_table(aw_dev,
                table_info, data_trace);
    }
    else
    {
        data_trace->sum_val = data_trace->sum_val / monitor->samp_count;
        data_trace->sum_val = ((int32_t)aplha * data_trace->sum_val +
                               (1000 - (int32_t)aplha) * data_trace->pre_val) / 1000;
        return aw_monitor_trace_data_from_table(aw_dev,
                                                table_info, data_trace);
    }
}

static int aw_monitor_get_data(struct aw_device *aw_dev)
{
    int ret = -1;
    struct aw_monitor_desc *monitor = &aw_dev->monitor_desc;
    struct aw_monitor_cfg *monitor_cfg = &monitor->monitor_cfg;
    struct aw_monitor_trace *vol_trace = &monitor->vol_trace;
    struct aw_monitor_trace *temp_trace = &monitor->temp_trace;

    if (monitor_cfg->vol_switch)
    {
        ret = aw_monitor_get_data_from_table(aw_dev,
                                             monitor_cfg->vol_info, vol_trace,
                                             monitor_cfg->vol_aplha);
        if (ret < 0)
        {
            return ret;
        }
    }
    else
    {
        vol_trace->aw_table.ipeak = IPEAK_NONE;
        vol_trace->aw_table.gain = GAIN_NONE;
        vol_trace->aw_table.vmax = VMAX_NONE;
    }

    if (monitor_cfg->temp_switch)
    {
        ret = aw_monitor_get_data_from_table(aw_dev,
                                             monitor_cfg->temp_info, temp_trace,
                                             monitor_cfg->temp_aplha);
        if (ret < 0)
        {
            return ret;
        }
    }
    else
    {
        temp_trace->aw_table.ipeak = IPEAK_NONE;
        temp_trace->aw_table.gain = GAIN_NONE;
        temp_trace->aw_table.vmax = VMAX_NONE;
    }

    aw_dev_dbg(aw_dev->dev_index, "filter_vol:%d, vol: ipeak = 0x%x, gain = 0x%x",
               monitor->vol_trace.sum_val, vol_trace->aw_table.ipeak,
               vol_trace->aw_table.gain);

    aw_dev_dbg(aw_dev->dev_index, "filter_temp:%d, temp: ipeak = 0x%x, gain = 0x%x",
               monitor->temp_trace.sum_val, temp_trace->aw_table.ipeak,
               temp_trace->aw_table.gain);

    return 0;
}

static void aw_monitor_get_cfg(struct aw_device *aw_dev,
                               struct aw_table *set_table)
{
    struct aw_monitor_desc *monitor = &aw_dev->monitor_desc;
    struct aw_table *temp_data = &monitor->temp_trace.aw_table;
    struct aw_table *vol_data = &monitor->vol_trace.aw_table;

    if (temp_data->ipeak == IPEAK_NONE && vol_data->ipeak == IPEAK_NONE)
    {
        memcpy(set_table, temp_data, sizeof(struct aw_table));
    }
    else if (temp_data->ipeak == IPEAK_NONE)
    {
        memcpy(set_table, vol_data, sizeof(struct aw_table));
    }
    else if (vol_data->ipeak == IPEAK_NONE)
    {
        memcpy(set_table, temp_data, sizeof(struct aw_table));
    }
    else
    {
        if (monitor->monitor_cfg.logic_switch == AW_MON_LOGIC_OR)
        {
            set_table->ipeak = (temp_data->ipeak < vol_data->ipeak ?
                                temp_data->ipeak : vol_data->ipeak);
            set_table->gain = (temp_data->gain < vol_data->gain ?
                               vol_data->gain : temp_data->gain);
            set_table->vmax = (temp_data->vmax < vol_data->vmax ?
                               vol_data->vmax : temp_data->vmax);
        }
        else
        {
            set_table->ipeak = (temp_data->ipeak < vol_data->ipeak ?
                                vol_data->ipeak : temp_data->ipeak);
            set_table->gain = (temp_data->gain < vol_data->gain ?
                               temp_data->gain : vol_data->gain);
            set_table->vmax = (temp_data->vmax < vol_data->vmax ?
                               temp_data->vmax : vol_data->vmax);
        }
    }
}

static void aw_monitor_set_ipeak(struct aw_device *aw_dev,
                                 uint16_t ipeak)
{
    int ret = -1;
    struct aw_monitor_cfg *monitor_cfg = &aw_dev->monitor_desc.monitor_cfg;
    unsigned int reg_val = 0;
    unsigned int read_reg_val;
    struct aw_ipeak_desc *desc = &aw_dev->ipeak_desc;

    if (ipeak == IPEAK_NONE || (!monitor_cfg->ipeak_switch))
    {
        return;
    }
    ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->reg, &reg_val);
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "read ipeak failed");
        return;
    }

    read_reg_val = reg_val;

    read_reg_val &= (~desc->mask);

    if (read_reg_val == ipeak)
    {
        aw_dev_info(aw_dev->dev_index, "ipeak = 0x%x, no change",
                    read_reg_val);
        return;
    }

    reg_val &= desc->mask;
    read_reg_val = ipeak;
    reg_val |= read_reg_val;

    ret = aw_dev->ops.aw_i2c_write(aw_dev, desc->reg, reg_val);
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "write ipeak failed");
        return;
    }
    aw_dev_info(aw_dev->dev_index, "set reg val = 0x%x, ipeak = 0x%x",
                reg_val, ipeak);
}

static void aw_monitor_set_gain(struct aw_device *aw_dev, uint16_t gain)
{
    int ret = -1;
    struct aw_monitor_cfg *monitor_cfg = &aw_dev->monitor_desc.monitor_cfg;
    unsigned int read_volume;
    unsigned int set_volume;

    if (gain == GAIN_NONE || (!monitor_cfg->gain_switch))
    {
        return;
    }
    ret = aw_dev->ops.aw_get_volume(aw_dev, &read_volume);
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "read volume failed");
        return;
    }

    gain = aw_dev->ops.aw_reg_val_to_db(gain);

    /*add offset*/
    set_volume = gain + aw_dev->volume_desc.init_volume;

    if (read_volume == set_volume)
    {
        aw_dev_info(aw_dev->dev_index, "gain = 0x%x, no change", read_volume);
        return;
    }

    ret = aw_dev->ops.aw_set_volume(aw_dev, set_volume);
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "set gain failed");
        return;
    }
    aw_dev_info(aw_dev->dev_index, "set reg val = 0x%x, gain = 0x%x",
                set_volume, gain);

}


static int aw_monitor_process(struct aw_device *aw_dev)
{
    int ret = -1;
    struct aw_monitor_desc *monitor = &aw_dev->monitor_desc;
    struct aw_monitor_cfg *monitor_cfg = &monitor->monitor_cfg;
    struct aw_table set_table;

    ret = aw_monitor_get_temp_and_vol(aw_dev);
    if (ret < 0)
    {
        return ret;
    }

    if (monitor->samp_count < monitor_cfg->monitor_count &&
            (monitor->first_entry == AW_NOT_FIRST_ENTRY))
    {
        return 0;
    }
    ret = aw_monitor_get_data(aw_dev);
    if (ret < 0)
    {
        return ret;
    }

    aw_monitor_get_cfg(aw_dev, &set_table);

    aw_dev_dbg(aw_dev->dev_index, "set_ipeak = 0x%x, set_gain = 0x%x",
               set_table.ipeak, set_table.gain);

    aw_monitor_set_ipeak(aw_dev, set_table.ipeak);

    aw_monitor_set_gain(aw_dev, set_table.gain);

    monitor->samp_count = 0;
    monitor->temp_trace.pre_val = monitor->temp_trace.sum_val;
    monitor->temp_trace.sum_val = 0;

    monitor->vol_trace.pre_val = monitor->vol_trace.sum_val;
    monitor->vol_trace.sum_val = 0;

    if (monitor->first_entry == AW_FIRST_ENTRY)
    {
        monitor->first_entry = AW_NOT_FIRST_ENTRY;
    }
    return 0;
}

int aw_monitor_work_func(void *dev)
{
    int ret = -1;
    struct aw_device *aw_dev = dev;
    struct aw_monitor_cfg *monitor_cfg = &aw_dev->monitor_desc.monitor_cfg;
    struct aw_monitor_desc *monitor = &aw_dev->monitor_desc;

    if ((monitor->spk_mode == AW_SPK_MODE) &&
            (monitor_cfg->monitor_status == AW_MON_CFG_OK) &&
            monitor_cfg->monitor_switch &&
            (monitor->monitor_start == AW_MON_START) &&
            (monitor->monitor_handle == AW_MONITOR_HANDLE_ON) &&
            (aw_dev->bop_en == AW_BOP_DISABLE))
    {
        if (!aw_get_hmute(aw_dev))
        {
            ret = aw_monitor_process(aw_dev);
            monitor->monitor_handle = AW_MONITOR_HANDLE_OFF;
        }
    }

    return ret;
}

void aw_check_bop_status(struct aw_device *aw_dev)
{
    struct aw_bop_desc *bop_desc = &aw_dev->bop_desc;
    unsigned int reg_val = 0;

    aw_dev_dbg(aw_dev->dev_index, "enter");

    if (aw_dev->bop_desc.reg == AW_REG_NONE)
        return;

    aw_dev->ops.aw_i2c_read(aw_dev, bop_desc->reg, &reg_val);
    reg_val = (uint16_t)reg_val & (~bop_desc->mask);
    if (reg_val == bop_desc->enable)
        aw_dev->bop_en = AW_BOP_ENABLE;
    else
        aw_dev->bop_en = AW_BOP_DISABLE;

    aw_dev_dbg(aw_dev->dev_index, "check done! bop status is %d", aw_dev->bop_en);
}

void aw_monitor_start(void *dev)
{
    int ret = -1;
    struct aw_device *aw_dev = dev;
    struct aw_monitor_desc *monitor_desc = &aw_dev->monitor_desc;
    aw_dev_info(aw_dev->dev_index, "enter");

    monitor_desc->first_entry = AW_FIRST_ENTRY;
    monitor_desc->samp_count = 0;
    monitor_desc->vol_trace.sum_val = 0;
    monitor_desc->temp_trace.sum_val = 0;
    monitor_desc->monitor_handle = AW_MONITOR_HANDLE_ON;
    monitor_desc->monitor_start = AW_MON_START;

    aw_check_bop_status(aw_dev);

    ret = aw_monitor_work_func((void *)aw_dev);
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "monitor work failed");
    }
}

void aw_monitor_stop(void *dev)
{
    struct aw_device *aw_dev = dev;
    aw_dev_info(aw_dev->dev_index, "enter");

    aw_dev->monitor_desc.monitor_start = AW_MON_STOP;

    return;
}

/*****************************************************
* load monitor config
*****************************************************/

static int aw_monitor_param_check_sum(struct aw_device *aw_dev,
                                      uint8_t *data, uint32_t data_len)
{
    int i, check_sum = 0;
    struct aw_monitor_hdr *monitor_hdr =
        (struct aw_monitor_hdr *)data;

    if (data_len < sizeof(struct aw_monitor_hdr))
    {
        aw_dev_err(aw_dev->dev_index, "data size smaller than hdr , please check monitor bin");
        return -ENOMEM;
    }

    for (i = 4; i < data_len; i++)
    {
        check_sum += (uint8_t)data[i];
    }
    if (monitor_hdr->check_sum != check_sum)
    {
        aw_dev_err(aw_dev->dev_index, "check_sum[%d] is not equal to actual check_sum[%d]",
                   monitor_hdr->check_sum, check_sum);
        return -ENOMEM;
    }

    return 0;
}

static int aw_monitor_check_fw(struct aw_device *aw_dev,
                               uint8_t *data, uint32_t data_len)
{
    struct aw_monitor_hdr *monitor_hdr = (struct aw_monitor_hdr *)data;
    int temp_size, vol_size;

    if (data_len < sizeof(struct aw_monitor_hdr))
    {
        aw_dev_err(aw_dev->dev_index, "params size[%d] < struct aw_monitor_hdr size[%d]!",
                   data_len, (int)sizeof(struct aw_monitor_hdr));
        return -ENOMEM;
    }

    if (monitor_hdr->temp_offset > data_len)
    {
        aw_dev_err(aw_dev->dev_index, "temp_offset[%d] overflow file size[%d]!",
                   monitor_hdr->temp_offset, data_len);
        return -ENOMEM;
    }

    if (monitor_hdr->vol_offset > data_len)
    {
        aw_dev_err(aw_dev->dev_index, "vol_offset[%d] overflow file size[%d]!",
                   monitor_hdr->vol_offset, data_len);
        return -ENOMEM;
    }

    temp_size = monitor_hdr->temp_num * monitor_hdr->single_temp_size;
    if (temp_size > data_len)
    {
        aw_dev_err(aw_dev->dev_index, "temp_size:[%d] overflow file size[%d]!",
                   temp_size, data_len);
        return -ENOMEM;
    }

    vol_size = monitor_hdr->vol_num * monitor_hdr->single_vol_size;
    if (vol_size > data_len)
    {
        aw_dev_err(aw_dev->dev_index, "vol_size:[%d] overflow file size[%d]!",
                   vol_size, data_len);
        return -ENOMEM;
    }

    return 0;
}

static int aw_monitor_check_fw_v_0_1_1(struct aw_device *aw_dev,
                                       uint8_t *data, uint32_t data_len)
{
    struct aw_monitor_hdr_v_0_1_1 *monitor_hdr =
        (struct aw_monitor_hdr_v_0_1_1 *)data;
    int temp_size, vol_size;

    if (data_len < sizeof(struct aw_monitor_hdr_v_0_1_1))
    {
        aw_dev_err(aw_dev->dev_index, "params size[%d] < struct aw_monitor_hdr size[%d]!",
                   data_len, (int)sizeof(struct aw_monitor_hdr));
        return -ENOMEM;
    }

    if (monitor_hdr->temp_offset > data_len)
    {
        aw_dev_err(aw_dev->dev_index, "temp_offset[%d] overflow file size[%d]!",
                   monitor_hdr->temp_offset, data_len);
        return -ENOMEM;
    }

    if (monitor_hdr->vol_offset > data_len)
    {
        aw_dev_err(aw_dev->dev_index, "vol_offset[%d] overflow file size[%d]!",
                   monitor_hdr->vol_offset, data_len);
        return -ENOMEM;
    }

    temp_size = monitor_hdr->temp_num * monitor_hdr->single_temp_size;
    if (temp_size > data_len)
    {
        aw_dev_err(aw_dev->dev_index, "temp_size:[%d] overflow file size[%d]!",
                   temp_size, data_len);
        return -ENOMEM;
    }

    vol_size = monitor_hdr->vol_num * monitor_hdr->single_vol_size;
    if (vol_size > data_len)
    {
        aw_dev_err(aw_dev->dev_index, "vol_size:[%d] overflow file size[%d]!",
                   vol_size, data_len);
        return -ENOMEM;
    }

    return 0;
}


static void aw_monitor_parse_hdr(struct aw_device *aw_dev, uint8_t *data)
{
    struct aw_monitor_hdr *monitor_hdr =
        (struct aw_monitor_hdr *)data;
    struct aw_monitor_cfg *monitor_cfg = &aw_dev->monitor_desc.monitor_cfg;

    monitor_cfg->monitor_switch = monitor_hdr->monitor_switch;
    monitor_cfg->monitor_time = monitor_hdr->monitor_time;
    monitor_cfg->monitor_count = monitor_hdr->monitor_count;
    monitor_cfg->ipeak_switch = monitor_hdr->ipeak_switch;
    monitor_cfg->gain_switch = monitor_hdr->gain_switch;
    monitor_cfg->vmax_switch = monitor_hdr->vmax_switch;
    monitor_cfg->temp_switch = monitor_hdr->temp_switch;
    monitor_cfg->temp_aplha = monitor_hdr->temp_aplha;
    monitor_cfg->vol_switch = monitor_hdr->vol_switch;
    monitor_cfg->vol_aplha = monitor_hdr->vol_aplha;

    aw_dev_info(aw_dev->dev_index, "monitor_switch:%d, monitor_time:%d (ms), monitor_count:%d",
                monitor_cfg->monitor_switch, monitor_cfg->monitor_time,
                monitor_cfg->monitor_count);

    aw_dev_info(aw_dev->dev_index, "ipeak_switch:%d, gain_switch:%d, vmax_switch:%d",
                monitor_cfg->ipeak_switch, monitor_cfg->gain_switch,
                monitor_cfg->vmax_switch);

    aw_dev_info(aw_dev->dev_index, "temp_switch:%d, temp_aplha:%d, vol_switch:%d, vol_aplha:%d",
                monitor_cfg->temp_switch, monitor_cfg->temp_aplha,
                monitor_cfg->vol_switch, monitor_cfg->vol_aplha);
}

static void aw_monitor_parse_hdr_v_0_1_1(struct aw_device *aw_dev, uint8_t *data)
{
    struct aw_monitor_hdr_v_0_1_1 *monitor_hdr =
        (struct aw_monitor_hdr_v_0_1_1 *)data;
    struct aw_monitor_cfg *monitor_cfg = &aw_dev->monitor_desc.monitor_cfg;

    monitor_cfg->monitor_switch = (monitor_hdr->enable_flag >> MONITOR_EN_BIT) & MONITOR_EN_MASK;
    monitor_cfg->monitor_time = monitor_hdr->monitor_time;
    monitor_cfg->monitor_count = monitor_hdr->monitor_count;
    monitor_cfg->ipeak_switch = (monitor_hdr->enable_flag >> MONITOR_IPEAK_EN_BIT) & MONITOR_EN_MASK;
    monitor_cfg->logic_switch = (monitor_hdr->enable_flag >> MONITOR_LOGIC_BIT) & MONITOR_EN_MASK;
    monitor_cfg->gain_switch = (monitor_hdr->enable_flag >> MONITOR_GAIN_EN_BIT) & MONITOR_EN_MASK;
    monitor_cfg->vmax_switch = (monitor_hdr->enable_flag >> MONITOR_VMAX_EN_BIT) & MONITOR_EN_MASK;
    monitor_cfg->temp_switch = (monitor_hdr->enable_flag >> MONITOR_TEMP_EN_BIT) & MONITOR_EN_MASK;
    monitor_cfg->temp_aplha = monitor_hdr->temp_aplha;
    monitor_cfg->vol_switch = (monitor_hdr->enable_flag >> MONITOR_VOL_EN_BIT) & MONITOR_EN_MASK;
    monitor_cfg->vol_aplha = monitor_hdr->vol_aplha;

    aw_dev_info(aw_dev->dev_index, "monitor_switch:%d, monitor_time:%d (ms), monitor_count:%d",
                monitor_cfg->monitor_switch, monitor_cfg->monitor_time,
                monitor_cfg->monitor_count);

    aw_dev_info(aw_dev->dev_index, "logic_switch:%d, ipeak_switch:%d, gain_switch:%d, vmax_switch:%d",
                monitor_cfg->logic_switch, monitor_cfg->ipeak_switch,
                monitor_cfg->gain_switch, monitor_cfg->vmax_switch);

    aw_dev_info(aw_dev->dev_index, "temp_switch:%d, temp_aplha:%d, vol_switch:%d, vol_aplha:%d",
                monitor_cfg->temp_switch, monitor_cfg->temp_aplha,
                monitor_cfg->vol_switch, monitor_cfg->vol_aplha);
}

static void aw_monitor_write_data_to_table(struct aw_device *aw_dev,
        struct aw_table_info *table_info, const uint8_t *offset_ptr)
{
    int i;

    for (i = 0; i < table_info->table_num * AW_TABLE_SIZE; i += AW_TABLE_SIZE)
    {
        table_info->aw_table[i / AW_TABLE_SIZE].min_val =
            AW_GET_16_DATA(offset_ptr[1 + i], offset_ptr[i]);
        table_info->aw_table[i / AW_TABLE_SIZE].max_val =
            AW_GET_16_DATA(offset_ptr[3 + i], offset_ptr[2 + i]);
        table_info->aw_table[i / AW_TABLE_SIZE].ipeak =
            AW_GET_16_DATA(offset_ptr[5 + i], offset_ptr[4 + i]);
        table_info->aw_table[i / AW_TABLE_SIZE].gain =
            AW_GET_16_DATA(offset_ptr[7 + i], offset_ptr[6 + i]);
        table_info->aw_table[i / AW_TABLE_SIZE].vmax =
            AW_GET_32_DATA(offset_ptr[11 + i], offset_ptr[10 + i],
                           offset_ptr[9 + i], offset_ptr[8 + i]);
    }

    for (i = 0; i < table_info->table_num; i++)
        aw_dev_info(aw_dev->dev_index,
                    "min_val:%d, max_val:%d, ipeak:0x%x, gain:0x%x, vmax:0x%x",
                    table_info->aw_table[i].min_val,
                    table_info->aw_table[i].max_val,
                    table_info->aw_table[i].ipeak,
                    table_info->aw_table[i].gain,
                    table_info->aw_table[i].vmax);
}

static int aw_monitor_parse_temp_data(struct aw_device *aw_dev, uint8_t *data)
{
    struct aw_monitor_hdr *monitor_hdr =
        (struct aw_monitor_hdr *)data;
    struct aw_table_info *temp_info =
            &aw_dev->monitor_desc.monitor_cfg.temp_info;

    aw_dev_info(aw_dev->dev_index, "===parse temp start ===");

    if (temp_info->aw_table != NULL)
    {
        free(temp_info->aw_table);
        temp_info->aw_table = NULL;
    }

    temp_info->aw_table = calloc(monitor_hdr->temp_num, AW_TABLE_SIZE);
    if (temp_info->aw_table == NULL)
    {
        return -ENOMEM;
    }

    temp_info->table_num = monitor_hdr->temp_num;
    aw_monitor_write_data_to_table(aw_dev, temp_info,
                                   &data[monitor_hdr->temp_offset]);
    aw_dev_info(aw_dev->dev_index, "===parse temp end ===");
    return 0;
}

static int aw_monitor_parse_temp_data_v_0_1_1(struct aw_device *aw_dev, uint8_t *data)
{
    struct aw_monitor_hdr_v_0_1_1 *monitor_hdr =
        (struct aw_monitor_hdr_v_0_1_1 *)data;
    struct aw_table_info *temp_info =
            &aw_dev->monitor_desc.monitor_cfg.temp_info;

    aw_dev_info(aw_dev->dev_index, "===parse temp start ===");

    if (temp_info->aw_table != NULL)
    {
        free(temp_info->aw_table);
        temp_info->aw_table = NULL;
    }

    temp_info->aw_table = calloc(monitor_hdr->temp_num, AW_TABLE_SIZE);
    if (temp_info->aw_table == NULL)
    {
        return -ENOMEM;
    }

    temp_info->table_num = monitor_hdr->temp_num;
    aw_monitor_write_data_to_table(aw_dev, temp_info,
                                   &data[monitor_hdr->temp_offset]);
    aw_dev_info(aw_dev->dev_index, "===parse temp end ===");
    return 0;
}

static int aw_monitor_parse_vol_data(struct aw_device *aw_dev, uint8_t *data)
{
    struct aw_monitor_hdr *monitor_hdr =
        (struct aw_monitor_hdr *)data;
    struct aw_table_info *vol_info =
            &aw_dev->monitor_desc.monitor_cfg.vol_info;

    aw_dev_info(aw_dev->dev_index, "===parse vol start ===");

    if (vol_info->aw_table != NULL)
    {
        free(vol_info->aw_table);
        vol_info->aw_table = NULL;
    }

    vol_info->aw_table = calloc(monitor_hdr->vol_num, AW_TABLE_SIZE);
    if (vol_info->aw_table == NULL)
    {
        return -ENOMEM;
    }

    vol_info->table_num = monitor_hdr->vol_num;
    aw_monitor_write_data_to_table(aw_dev, vol_info,
                                   &data[monitor_hdr->vol_offset]);
    aw_dev_info(aw_dev->dev_index, "===parse vol end ===");
    return 0;
}

static int aw_monitor_parse_vol_data_v_0_1_1(struct aw_device *aw_dev, uint8_t *data)
{
    struct aw_monitor_hdr_v_0_1_1 *monitor_hdr =
        (struct aw_monitor_hdr_v_0_1_1 *)data;
    struct aw_table_info *vol_info =
            &aw_dev->monitor_desc.monitor_cfg.vol_info;

    aw_dev_info(aw_dev->dev_index, "===parse vol start ===");

    if (vol_info->aw_table != NULL)
    {
        free(vol_info->aw_table);
        vol_info->aw_table = NULL;
    }

    vol_info->aw_table = calloc(monitor_hdr->vol_num, AW_TABLE_SIZE);
    if (vol_info->aw_table == NULL)
    {
        return -ENOMEM;
    }

    vol_info->table_num = monitor_hdr->vol_num;
    aw_monitor_write_data_to_table(aw_dev, vol_info,
                                   &data[monitor_hdr->vol_offset]);
    aw_dev_info(aw_dev->dev_index, "===parse vol end ===");
    return 0;
}

static int aw_monitor_parse_data(struct aw_device *aw_dev,
                                 uint8_t *data, uint32_t data_len)
{
    int ret = -1;
    struct aw_monitor_cfg *monitor_cfg = &aw_dev->monitor_desc.monitor_cfg;

    ret = aw_monitor_check_fw(aw_dev, data, data_len);
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "check monitor failed");
        return ret;
    }

    aw_monitor_parse_hdr(aw_dev, data);

    ret = aw_monitor_parse_temp_data(aw_dev, data);
    if (ret < 0)
    {
        return ret;
    }
    ret = aw_monitor_parse_vol_data(aw_dev, data);
    if (ret < 0)
    {
        if (monitor_cfg->temp_info.aw_table != NULL)
        {
            free(monitor_cfg->temp_info.aw_table);
            monitor_cfg->temp_info.aw_table = NULL;
            monitor_cfg->temp_info.table_num = 0;
        }
        return ret;
    }

    monitor_cfg->monitor_status = AW_MON_CFG_OK;
    return 0;
}

static int aw_monitor_parse_data_v_0_1_1(struct aw_device *aw_dev,
        uint8_t *data, uint32_t data_len)
{
    int ret = -1;
    struct aw_monitor_cfg *monitor_cfg = &aw_dev->monitor_desc.monitor_cfg;

    ret = aw_monitor_check_fw_v_0_1_1(aw_dev, data, data_len);
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "check monitor failed");
        return ret;
    }

    aw_monitor_parse_hdr_v_0_1_1(aw_dev, data);

    ret = aw_monitor_parse_temp_data_v_0_1_1(aw_dev, data);
    if (ret < 0)
    {
        return ret;
    }
    ret = aw_monitor_parse_vol_data_v_0_1_1(aw_dev, data);
    if (ret < 0)
    {
        if (monitor_cfg->temp_info.aw_table != NULL)
        {
            free(monitor_cfg->temp_info.aw_table);
            monitor_cfg->temp_info.aw_table = NULL;
            monitor_cfg->temp_info.table_num = 0;
        }
        return ret;
    }

    monitor_cfg->monitor_status = AW_MON_CFG_OK;
    return 0;
}

int aw_monitor_parse_fw(struct aw_device *aw_dev)
{
    int ret = -1;
    uint8_t *data = NULL;
    data = (uint8_t *)aw_dev->prof_info->monitor_data->data;
    uint32_t data_len = aw_dev->prof_info->monitor_data->len;

    struct aw_monitor_hdr *monitor_hdr = NULL;

    aw_dev_info(aw_dev->dev_index, "enter");

    if (data == NULL)
    {
        aw_dev_err(aw_dev->dev_index, "monitor data is NULL");
        return -EINVAL;
    }

    monitor_hdr = (struct aw_monitor_hdr *)data;
    ret = aw_monitor_param_check_sum(aw_dev, data, data_len);
    if (ret < 0)
    {
        return ret;
    }

    aw_dev_info(aw_dev->dev_index, "monitor_ver=0x%x", monitor_hdr->monitor_ver);

    switch (monitor_hdr->monitor_ver)
    {
    case AW_MONITOR_HDR_VER_0_1_0:
        return aw_monitor_parse_data(aw_dev, data, data_len);
    case AW_MONITOR_HDR_VER_0_1_1:
        return aw_monitor_parse_data_v_0_1_1(aw_dev, data, data_len);
    default:
        aw_dev_err(aw_dev->dev_index, "cfg version:0x%x unsupported",
                   monitor_hdr->monitor_ver);
        return -EINVAL;
    }
}

int aw_monitor_init(void *dev)
{
    int ret = -1;
    struct aw_device *aw_dev = dev;

    aw_dev_info(aw_dev->dev_index, "enter");

    if (aw_dev->prof_info->monitor_data == NULL)
    {
        aw_dev_info(aw_dev->dev_index, "monitor data is NULL");
        return 0;
    }

    ret = aw_monitor_parse_fw(aw_dev);
    if (ret < 0)
    {
        aw_dev_info(aw_dev->dev_index, "monitor parse failed");
        return ret;
    }

    return 0;
}

void aw_monitor_set_handle(void *dev)
{
    struct aw_device *aw_dev = dev;

    aw_dev->monitor_desc.monitor_handle = AW_MONITOR_HANDLE_ON;

    aw_dev_info(aw_dev->dev_index, "set_monitor_handle=%d", aw_dev->monitor_desc.monitor_handle);
}

void aw_monitor_deinit(void *dev)
{
    struct aw_device *aw_dev = dev;
    struct aw_monitor_cfg *monitor_cfg =
            &aw_dev->monitor_desc.monitor_cfg;

    aw_dev->monitor_desc.monitor_start = AW_MON_STOP;
    monitor_cfg->monitor_status = AW_MON_CFG_ST;

    if (monitor_cfg->temp_info.aw_table != NULL)
    {
        free(monitor_cfg->temp_info.aw_table);
        monitor_cfg->temp_info.aw_table = NULL;
    }

    if (monitor_cfg->vol_info.aw_table != NULL)
    {
        free(monitor_cfg->vol_info.aw_table);
        monitor_cfg->vol_info.aw_table = NULL;
    }

    memset(monitor_cfg, 0, sizeof(struct aw_monitor_cfg));
}

#endif
