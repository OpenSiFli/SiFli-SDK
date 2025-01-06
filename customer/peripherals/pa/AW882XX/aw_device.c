/*
* aw_device.c
*
* Copyright (c) 2021 AWINIC Technology CO., LTD
*
* Author: <zhaolei@awinic.com>
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aw_base.h"
#include "aw_device.h"
#include "aw_profile_process.h"

#define AW_DEV_SYSST_CHECK_MAX      (10)

static unsigned int g_fade_in_time = AW_1_MS;
static unsigned int g_fade_out_time = AW_1_MS;


/*****************************awinic device*************************************/
/*pwd enable update reg*/
static int aw_dev_reg_fw_update(struct aw_device *aw_dev)
{
    int ret = -1;
    int i = 0;
    unsigned int reg_addr = 0;
    unsigned int reg_val = 0;
    unsigned int read_val;
    unsigned int init_volume = 0;
    struct aw_int_desc *int_desc = &aw_dev->int_desc;
    struct aw_profctrl_desc *profctrl_desc = &aw_dev->profctrl_desc;
    struct aw_bstctrl_desc *bstctrl_desc = &aw_dev->bstctrl_desc;
    struct aw_sec_data_desc *reg_data = NULL;
    uint16_t *data = NULL;
    int data_len;

    reg_data = aw_dev_get_prof_data_byname(aw_dev, aw_dev->set_prof_name, AW_PROFILE_DATA_TYPE_REG);
    if (reg_data == NULL)
    {
        return -EINVAL;
    }

    data = (uint16_t *)reg_data->data;
    data_len = reg_data->len >> 1;

    for (i = 0; i < data_len; i += 2)
    {
        reg_addr = data[i];
        reg_val = data[i + 1];

        if (reg_addr == int_desc->mask_reg)
        {
            int_desc->int_mask = reg_val;
            reg_val = int_desc->mask_default;
        }

        if (aw_dev->bstcfg_enable)
        {
            if (reg_addr == profctrl_desc->reg)
            {
                profctrl_desc->cfg_prof_mode =
                    reg_val & (~profctrl_desc->mask);
            }

            if (reg_addr == bstctrl_desc->reg)
            {
                bstctrl_desc->cfg_bst_type =
                    reg_val & (~bstctrl_desc->mask);
            }
        }

        /*keep amppd status*/
        if (reg_addr == aw_dev->amppd_desc.reg)
        {
            aw_dev->amppd_st = reg_val & (~aw_dev->amppd_desc.mask);
            aw_dev_info(aw_dev->dev_index, "amppd_st=0x%04x", aw_dev->amppd_st);
            aw_dev->ops.aw_i2c_read(aw_dev,
                                    (unsigned char)reg_addr,
                                    (unsigned int *)&read_val);
            read_val &= (~aw_dev->amppd_desc.mask);
            reg_val &= aw_dev->amppd_desc.mask;
            reg_val |= read_val;
        }

        /*keep pwd status*/
        if (reg_addr == aw_dev->pwd_desc.reg)
        {
            aw_dev->ops.aw_i2c_read(aw_dev,
                                    (unsigned char)reg_addr,
                                    (unsigned int *)&read_val);
            read_val &= (~aw_dev->pwd_desc.mask);
            reg_val &= aw_dev->pwd_desc.mask;
            reg_val |= read_val;
        }
        /*keep mute status*/
        if (reg_addr == aw_dev->mute_desc.reg)
        {
            /*get bin value*/
            aw_dev->mute_st = reg_val & (~aw_dev->mute_desc.mask);
            aw_dev_info(aw_dev->dev_index, "mute_st=0x%04x", aw_dev->mute_st);
            aw_dev->ops.aw_i2c_read(aw_dev,
                                    (unsigned char)reg_addr,
                                    (unsigned int *)&read_val);
            read_val &= (~aw_dev->mute_desc.mask);
            reg_val &= aw_dev->mute_desc.mask;
            reg_val |= read_val;
        }

        if (reg_addr == aw_dev->vcalb_desc.vcalb_reg)
        {
            continue;
        }

        aw_dev_info(aw_dev->dev_index, "reg=0x%04x, val = 0x%04x",
                    (uint16_t)reg_addr, (uint16_t)reg_val);
        ret = aw_dev->ops.aw_i2c_write(aw_dev,
                                       (unsigned char)reg_addr,
                                       (unsigned int)reg_val);
        if (ret < 0)
        {
            break;
        }
    }

    aw_dev->ops.aw_i2c_read(aw_dev, profctrl_desc->reg, &reg_val);
    reg_val = (reg_val & (~profctrl_desc->mask));
    if (reg_val == profctrl_desc->spk_mode)
    {
        aw_dev->monitor_desc.spk_mode = AW_SPK_MODE;
    }
    else
    {
        aw_dev->monitor_desc.spk_mode = AW_NOT_SPK_MODE;
    }
    aw_dev_info(aw_dev->dev_index, "reg0x%02x=0x%04x, spk_mode=%d", profctrl_desc->reg,
                reg_val, aw_dev->monitor_desc.spk_mode);

    aw_dev->ops.aw_get_volume(aw_dev, &init_volume);
    aw_dev->volume_desc.init_volume = init_volume;

    /*keep min volume*/
    aw_dev->ops.aw_set_volume(aw_dev, aw_dev->volume_desc.mute_volume);

    aw_dev_info(aw_dev->dev_index, "load prof [%s] done", aw_dev->set_prof_name);

    return ret;
}

static void aw_dev_fade_in(struct aw_device *aw_dev)
{
    int i = 0;
    int fade_step = aw_dev->vol_step;
    struct aw_volume_desc *desc = &aw_dev->volume_desc;

    if (fade_step == 0 || g_fade_in_time == 0)
    {
        aw_dev->ops.aw_set_volume(aw_dev, desc->init_volume);
        return;
    }
    /*volume up*/
    for (i = desc->mute_volume; i >= desc->init_volume; i -= fade_step)
    {
        if (i < desc->init_volume)
        {
            i = desc->init_volume;
        }
        aw_dev->ops.aw_set_volume(aw_dev, i);
        AW_MS_DELAY(g_fade_in_time);
    }
    if (i != desc->init_volume)
    {
        aw_dev->ops.aw_set_volume(aw_dev, desc->init_volume);
    }
}

static void aw_dev_fade_out(struct aw_device *aw_dev)
{
    int i = 0;
    unsigned start_volume = 0;
    int fade_step = aw_dev->vol_step;
    struct aw_volume_desc *desc = &aw_dev->volume_desc;

    if (fade_step == 0 || g_fade_out_time == 0)
    {
        aw_dev->ops.aw_set_volume(aw_dev, desc->mute_volume);
        return;
    }

    aw_dev->ops.aw_get_volume(aw_dev, &start_volume);
    i = start_volume;
    for (i = start_volume; i <= desc->mute_volume; i += fade_step)
    {
        if (i > desc->mute_volume)
        {
            i = desc->mute_volume;
        }
        aw_dev->ops.aw_set_volume(aw_dev, i);
        AW_MS_DELAY(g_fade_out_time);
    }
    if (i != desc->mute_volume)
    {
        aw_dev->ops.aw_set_volume(aw_dev, desc->mute_volume);
        AW_MS_DELAY(g_fade_out_time);
    }
}

#ifdef AW_VOLUME
void aw_dev_get_init_volume(struct aw_device *aw_dev, uint32_t *volume)
{
    struct aw_volume_desc *desc = &aw_dev->volume_desc;

    *volume =  desc->init_volume;
}

int aw_dev_set_init_volume(struct aw_device *aw_dev, uint32_t volume)
{
    struct aw_volume_desc *desc = &aw_dev->volume_desc;

    if (volume > desc->mute_volume)
    {
        aw_dev_err(aw_dev->dev_index, "unsupported volume:%d", volume);
        return -EINVAL;
    }

    desc->init_volume = volume;
    aw_dev->ops.aw_set_volume(aw_dev, desc->init_volume);
    return 0;
}
#endif

int aw_dev_set_init_fs(struct aw_device *aw_dev, uint32_t fs, uint8_t chn_sel)
{
    int ret;
    aw_dev->samplerate = fs;
    aw_dev->chn_sel = chn_sel;
    ret = aw_dev->ops.aw_set_sf(aw_dev, fs, chn_sel);
    return ret;
}


static void aw_dev_pwd(struct aw_device *aw_dev, bool pwd)
{
    struct aw_pwd_desc *pwd_desc = &aw_dev->pwd_desc;

    aw_dev_dbg(aw_dev->dev_index, "enter, pwd: %d", pwd);

    if (pwd)
    {
        aw_dev->ops.aw_i2c_write_bits(aw_dev, pwd_desc->reg,
                                      pwd_desc->mask,
                                      pwd_desc->enable);
    }
    else
    {
        aw_dev->ops.aw_i2c_write_bits(aw_dev, pwd_desc->reg,
                                      pwd_desc->mask,
                                      pwd_desc->disable);
    }
    aw_dev_info(aw_dev->dev_index, "done");
}

static void aw_dev_amppd(struct aw_device *aw_dev, bool amppd)
{
    struct aw_amppd_desc *amppd_desc = &aw_dev->amppd_desc;

    aw_dev_dbg(aw_dev->dev_index, "enter, amppd: %d", amppd);

    if (amppd)
    {
        aw_dev->ops.aw_i2c_write_bits(aw_dev, amppd_desc->reg,
                                      amppd_desc->mask,
                                      amppd_desc->enable);
    }
    else
    {
        aw_dev->ops.aw_i2c_write_bits(aw_dev, amppd_desc->reg,
                                      amppd_desc->mask,
                                      amppd_desc->disable);
    }
    aw_dev_info(aw_dev->dev_index, "done");
}

static void aw_dev_mute(struct aw_device *aw_dev, bool mute)
{
    struct aw_mute_desc *mute_desc = &aw_dev->mute_desc;

    aw_dev_dbg(aw_dev->dev_index, "enter, mute: %d", mute);

    if (mute)
    {
        aw_dev_fade_out(aw_dev);
        aw_dev->ops.aw_i2c_write_bits(aw_dev, mute_desc->reg,
                                      mute_desc->mask,
                                      mute_desc->enable);
    }
    else
    {
        aw_dev->ops.aw_i2c_write_bits(aw_dev, mute_desc->reg,
                                      mute_desc->mask,
                                      mute_desc->disable);
        aw_dev_fade_in(aw_dev);
    }
    aw_dev_info(aw_dev->dev_index, "done");
}

static void aw_dev_uls_hmute(struct aw_device *aw_dev, bool uls_hmute)
{
    struct aw_uls_hmute_desc *uls_hmute_desc = &aw_dev->uls_hmute_desc;

    aw_dev_dbg(aw_dev->dev_index, "enter, uls_hmute: %d", uls_hmute);

    if (uls_hmute_desc->reg == AW_REG_NONE)
        return;

    if (uls_hmute)
    {
        aw_dev->ops.aw_i2c_write_bits(aw_dev, uls_hmute_desc->reg,
                                      uls_hmute_desc->mask,
                                      uls_hmute_desc->enable);
    }
    else
    {
        aw_dev->ops.aw_i2c_write_bits(aw_dev, uls_hmute_desc->reg,
                                      uls_hmute_desc->mask,
                                      uls_hmute_desc->disable);
    }
    aw_dev_info(aw_dev->dev_index, "done");
}

static int aw_dev_get_icalk(struct aw_device *aw_dev, int16_t *icalk)
{
    int ret = -1;
    unsigned int reg_val = 0;
    uint16_t reg_icalk = 0;
    uint16_t reg_icalkl = 0;
    struct aw_vcalb_desc *desc = &aw_dev->vcalb_desc;

    if (desc->icalkl_reg == AW_REG_NONE)
    {
        ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->icalk_reg, &reg_val);
        reg_icalk = (uint16_t)reg_val & (~desc->icalk_reg_mask);
    }
    else
    {
        ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->icalk_reg, &reg_val);
        reg_icalk = (uint16_t)reg_val & (~desc->icalk_reg_mask);
        ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->icalkl_reg, &reg_val);
        reg_icalkl = (uint16_t)reg_val & (~desc->icalkl_reg_mask);
        reg_icalk = (reg_icalk >> desc->icalk_shift) | (reg_icalkl >> desc->icalkl_shift);
    }

    if (reg_icalk & (~desc->icalk_sign_mask))
    {
        reg_icalk = reg_icalk | (~desc->icalk_neg_mask);
    }

    *icalk = (int16_t)reg_icalk;

    return ret;
}

static int aw_dev_get_vcalk(struct aw_device *aw_dev, int16_t *vcalk)
{
    int ret = -1;
    unsigned int reg_val = 0;
    uint16_t reg_vcalk = 0;
    uint16_t reg_vcalkl = 0;
    struct aw_vcalb_desc *desc = &aw_dev->vcalb_desc;

    if (desc->vcalkl_reg == AW_REG_NONE)
    {
        ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->vcalk_reg, &reg_val);
        reg_vcalk = (uint16_t)reg_val & (~desc->vcalk_reg_mask);
    }
    else
    {
        ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->vcalk_reg, &reg_val);
        reg_vcalk = (uint16_t)reg_val & (~desc->vcalk_reg_mask);
        ret = aw_dev->ops.aw_i2c_read(aw_dev, desc->vcalkl_reg, &reg_val);
        reg_vcalkl = (uint16_t)reg_val & (~desc->vcalkl_reg_mask);
        reg_vcalk = (reg_vcalk >> desc->vcalk_shift) | (reg_vcalkl >> desc->vcalkl_shift);
    }

    if (reg_vcalk & (~desc->vcalk_sign_mask))
    {
        reg_vcalk = reg_vcalk | (~desc->vcalk_neg_mask);
    }
    *vcalk = (int16_t)reg_vcalk;

    return ret;
}

static int aw_dev_set_vcalb(struct aw_device *aw_dev)
{
    int ret = -1;
    unsigned int reg_val;
    int vcalb;
    int icalk;
    int vcalk;
    int16_t icalk_val = 0;
    int16_t vcalk_val = 0;

    struct aw_vcalb_desc *desc = &aw_dev->vcalb_desc;

    if (desc->icalk_reg == AW_REG_NONE || desc->vcalb_reg == AW_REG_NONE)
    {
        aw_dev_info(aw_dev->dev_index, "REG None !");
        return 0;
    }

    ret = aw_dev_get_icalk(aw_dev, &icalk_val);
    if (ret < 0)
    {
        return ret;
    }

    ret = aw_dev_get_vcalk(aw_dev, &vcalk_val);
    if (ret < 0)
    {
        return ret;
    }

    icalk = desc->cabl_base_value + desc->icalk_value_factor * icalk_val;
    vcalk = desc->cabl_base_value + desc->vcalk_value_factor * vcalk_val;
    if (!vcalk)
    {
        aw_dev_err(aw_dev->dev_index, "vcalk is 0");
        return -EINVAL;
    }

    vcalb = desc->vcal_factor * icalk / vcalk;

    reg_val = (unsigned int)vcalb;
    aw_dev_dbg(aw_dev->dev_index, "icalk=%d, vcalk=%d, vcalb=%d, reg_val=%d",
               icalk, vcalk, vcalb, reg_val);

    ret =  aw_dev->ops.aw_i2c_write(aw_dev, desc->vcalb_reg, reg_val);

    aw_dev_info(aw_dev->dev_index, "done");

    return ret;
}

int aw_dev_get_int_status(struct aw_device *aw_dev, uint16_t *int_status)
{
    int ret = -1;
    unsigned int reg_val = 0;

    ret = aw_dev->ops.aw_i2c_read(aw_dev, aw_dev->int_desc.st_reg, &reg_val);
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "read interrupt reg fail, ret=%d", ret);
    }
    else
    {
        *int_status = reg_val;
    }

    aw_dev_dbg(aw_dev->dev_index, "read interrupt reg = 0x%04x", *int_status);
    return ret;
}

void aw_dev_clear_int_status(struct aw_device *aw_dev)
{
    uint16_t int_status = 0;

    /*read int status and clear*/
    aw_dev_get_int_status(aw_dev, &int_status);
    /*make suer int status is clear*/
    aw_dev_get_int_status(aw_dev, &int_status);
    aw_dev_info(aw_dev->dev_index, "done");
}

int aw_dev_set_intmask(struct aw_device *aw_dev, bool flag)
{
    int ret = -1;
    struct aw_int_desc *desc = &aw_dev->int_desc;

    if (flag)
    {
        ret = aw_dev->ops.aw_i2c_write(aw_dev, desc->mask_reg,
                                       desc->int_mask);
    }
    else
    {
        ret = aw_dev->ops.aw_i2c_write(aw_dev, desc->mask_reg,
                                       desc->mask_default);
    }
    aw_dev_info(aw_dev->dev_index, "done");
    return ret;
}

static int aw_dev_mode1_pll_check(struct aw_device *aw_dev)
{
    int ret = -1;
    unsigned char i;
    unsigned int reg_val = 0;
    struct aw_sysst_desc *desc = &aw_dev->sysst_desc;

    for (i = 0; i < AW_DEV_SYSST_CHECK_MAX; i++)
    {
        aw_dev->ops.aw_i2c_read(aw_dev, desc->reg, &reg_val);
        if (reg_val & desc->pll_check)
        {
            ret = 0;
            break;
        }
        else
        {
            aw_dev_dbg(aw_dev->dev_index, "check pll lock fail, cnt=%d, reg_val=0x%04x",
                       i, reg_val);
            AW_MS_DELAY(AW_2_MS);
        }
    }
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "check fail");
    }
    else
    {
        aw_dev_info(aw_dev->dev_index, "done");
    }

    return ret;
}

static int aw_dev_mode2_pll_check(struct aw_device *aw_dev)
{
    int ret = -1;
    unsigned int reg_val = 0;
    struct aw_cco_mux_desc *cco_mux_desc = &aw_dev->cco_mux_desc;

    aw_dev->ops.aw_i2c_read(aw_dev, cco_mux_desc->reg, &reg_val);
    reg_val &= (~cco_mux_desc->mask);
    aw_dev_dbg(aw_dev->dev_index, "REG_PLLCTRL1_bit14 = 0x%04x", reg_val);
    if (reg_val == cco_mux_desc->divided_val)
    {
        aw_dev_dbg(aw_dev->dev_index, "CCO_MUX is already divided");
        return ret;
    }

    /* change mode2 */
    aw_dev->ops.aw_i2c_write_bits(aw_dev, cco_mux_desc->reg,
                                  cco_mux_desc->mask, cco_mux_desc->bypass_val);
    ret = aw_dev_mode1_pll_check(aw_dev);

    /* change mode1 */
    aw_dev->ops.aw_i2c_write_bits(aw_dev, cco_mux_desc->reg,
                                  cco_mux_desc->mask, cco_mux_desc->divided_val);
    if (ret == 0)
    {
        AW_MS_DELAY(AW_2_MS);
        ret = aw_dev_mode1_pll_check(aw_dev);
    }

    return ret;
}

static int aw_dev_syspll_check(struct aw_device *aw_dev)
{
    int ret = -1;

    ret = aw_dev_mode1_pll_check(aw_dev);
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index,
                   "mode1 check iis failed try switch to mode2 check");

        ret = aw_dev_mode2_pll_check(aw_dev);
        if (ret < 0)
        {
            aw_dev_err(aw_dev->dev_index, "mode2 check iis failed");
        }
    }

    return ret;
}

static int aw_dev_sysst_check(struct aw_device *aw_dev)
{
    int ret = -1;
    unsigned char i;
    unsigned int reg_val = 0;
    struct aw_sysst_desc *desc = &aw_dev->sysst_desc;

    for (i = 0; i < AW_DEV_SYSST_CHECK_MAX; i++)
    {
        aw_dev->ops.aw_i2c_read(aw_dev, desc->reg, &reg_val);
        if (((reg_val & (~desc->mask)) & desc->st_check) == desc->st_check)
        {
            ret = 0;
            break;
        }
        else
        {
            aw_dev_info(aw_dev->dev_index, "check fail, cnt=%d, reg_val=0x%04x",
                        i, reg_val);
            AW_MS_DELAY(AW_2_MS);
        }
    }
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "check fail");
    }
    else
    {
        aw_dev_info(aw_dev->dev_index, "done");
    }

    return ret;
}

#ifdef AW_FADE
int aw_dev_get_fade_vol_step(struct aw_device *aw_dev)
{
    aw_dev_dbg(aw_dev->dev_index, "enter");
    return aw_dev->vol_step;
}

void aw_dev_set_fade_vol_step(struct aw_device *aw_dev, unsigned int step)
{
    aw_dev_dbg(aw_dev->dev_index, "enter");
    aw_dev->vol_step = step;
}

void aw_dev_get_fade_time(unsigned int *time, bool fade_in)
{
    if (fade_in)
    {
        *time = g_fade_in_time;
    }
    else
    {
        *time = g_fade_out_time;
    }
}

void aw_dev_set_fade_time(unsigned int time, bool fade_in)
{
    if (fade_in)
    {
        g_fade_in_time = time;
    }
    else
    {
        g_fade_out_time = time;
    }
}
#endif

int aw_dev_status(struct aw_device *aw_dev)
{
    return aw_dev->status;
}

void aw_dev_soft_reset(struct aw_device *aw_dev)
{
    struct aw_soft_rst *reset = &aw_dev->soft_rst;

    aw_dev->ops.aw_i2c_write(aw_dev, reset->reg, reset->reg_value);
    aw_dev_info(aw_dev->dev_index, "soft reset done");
}

int aw_device_irq_reinit(struct aw_device *aw_dev)
{
    int ret = -1;

    /*reg re load*/
    ret = aw_dev_reg_fw_update(aw_dev);
    if (ret < 0)
    {
        return ret;
    }

    /*update vcalb*/
    aw_dev_set_vcalb(aw_dev);

    return 0;
}

int aw_device_init(struct aw_device *aw_dev)
{
    int ret = - 1;

    if (aw_dev == NULL)
    {
        aw_dev_err(aw_dev->dev_index, "pointer is NULL");
        return -ENOMEM;
    }

    aw_dev_soft_reset(aw_dev);

    strncpy(aw_dev->cur_prof_name, aw_dev->first_prof_name, AW_PROF_NAME_MAX - 1);
    strncpy(aw_dev->set_prof_name, aw_dev->first_prof_name, AW_PROF_NAME_MAX - 1);

    ret = aw_dev_reg_fw_update(aw_dev);
    if (ret < 0)
    {
        return ret;
    }

    ret = aw_dev_set_vcalb(aw_dev);
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "can't set vcalb");
        return ret;
    }

    aw_dev->status = AW_DEV_PW_ON;

    aw_device_stop(aw_dev);

    aw_dev_info(aw_dev->dev_index, "init done");
    return 0;
}

int aw_dev_reg_update(struct aw_device *aw_dev, bool force)
{
    int ret = -1;

    if (force)
    {
        aw_dev_soft_reset(aw_dev);
        ret = aw_dev_reg_fw_update(aw_dev);
        if (ret < 0)
        {
            return ret;
        }
    }
    else
    {
        if (strncmp(aw_dev->cur_prof_name, aw_dev->set_prof_name,  AW_PROF_NAME_MAX) != 0)
        {
            ret = aw_dev_reg_fw_update(aw_dev);
            if (ret < 0)
            {
                return ret;
            }
        }
    }

    aw_dev_info(aw_dev->dev_index, "cur_prof=%s", aw_dev->cur_prof_name);

    strncpy(aw_dev->cur_prof_name, aw_dev->set_prof_name, AW_PROF_NAME_MAX - 1);

    aw_dev_info(aw_dev->dev_index, "done");
    return 0;
}

int aw_dev_prof_update(struct aw_device *aw_dev, bool force)
{
    int ret = -1;

    /*if power on need off -- load -- on*/
    if (aw_dev->status == AW_DEV_PW_ON)
    {
        aw_device_stop(aw_dev);

        ret = aw_dev_reg_update(aw_dev, force);
        if (ret)
        {
            aw_dev_err(aw_dev->dev_index, "fw update failed ");
            return ret;
        }

        ret = aw_device_start(aw_dev);
        if (ret)
        {
            aw_dev_err(aw_dev->dev_index, "start failed ");
            return ret;
        }
    }
    else
    {
        /*if pa off , only update set_prof value*/
        aw_dev_info(aw_dev->dev_index, "set prof[%s] done !", aw_dev->set_prof_name);
    }

    aw_dev_info(aw_dev->dev_index, "update done !");
    return 0;
}

static void aw_dev_boost_type_set(struct aw_device *aw_dev)
{
    struct aw_profctrl_desc *profctrl_desc = &aw_dev->profctrl_desc;
    struct aw_bstctrl_desc *bstctrl_desc = &aw_dev->bstctrl_desc;

    aw_dev_dbg(aw_dev->dev_index, "enter");

    if (aw_dev->bstcfg_enable)
    {
        /*set spk mode*/
        aw_dev->ops.aw_i2c_write_bits(aw_dev, profctrl_desc->reg,
                                      profctrl_desc->mask, profctrl_desc->spk_mode);

        /*force boost*/
        aw_dev->ops.aw_i2c_write_bits(aw_dev, bstctrl_desc->reg,
                                      bstctrl_desc->mask, bstctrl_desc->frc_bst);

        aw_dev_dbg(aw_dev->dev_index, "boost type set done");
    }
}

static void aw_dev_boost_type_recover(struct aw_device *aw_dev)
{
    struct aw_profctrl_desc *profctrl_desc = &aw_dev->profctrl_desc;
    struct aw_bstctrl_desc *bstctrl_desc = &aw_dev->bstctrl_desc;

    aw_dev_dbg(aw_dev->dev_index, "enter");

    if (aw_dev->bstcfg_enable)
    {
        /*set transprant*/
        aw_dev->ops.aw_i2c_write_bits(aw_dev, bstctrl_desc->reg,
                                      bstctrl_desc->mask, bstctrl_desc->tsp_type);

        AW_MS_DELAY(AW_5_MS);
        /*set cfg boost type*/
        aw_dev->ops.aw_i2c_write_bits(aw_dev, bstctrl_desc->reg,
                                      bstctrl_desc->mask, bstctrl_desc->cfg_bst_type);

        /*set cfg prof mode*/
        aw_dev->ops.aw_i2c_write_bits(aw_dev, profctrl_desc->reg,
                                      profctrl_desc->mask, profctrl_desc->cfg_prof_mode);

        aw_dev_dbg(aw_dev->dev_index, "boost type recover done");
    }
}

int aw_dev_reg_dump(struct aw_device *aw_dev)
{
    int reg_num = aw_dev->ops.aw_get_reg_num();
    uint8_t i = 0;
    uint32_t reg_val = 0;

    for (i = 0; i < reg_num; i++)
    {
        if (aw_dev->ops.aw_check_rd_access(i))
        {
            aw_dev->ops.aw_i2c_read(aw_dev, i, &reg_val);
            aw_dev_info(aw_dev->dev_index, "read: reg = 0x%02x, val = 0x%04x",
                        i, reg_val);
        }
    }

    return 0;
}

int aw_device_start(struct aw_device *aw_dev)
{
    int ret = -1;

    aw_dev_dbg(aw_dev->dev_index, "enter");

    if (aw_dev->status == AW_DEV_PW_ON)
    {
        aw_dev_info(aw_dev->dev_index, "already power on");
        return 0;
    }

    /*set froce boost*/
    aw_dev_boost_type_set(aw_dev);

    /*power on*/
    aw_dev_pwd(aw_dev, false);
    AW_MS_DELAY(AW_2_MS);

    ret = aw_dev_syspll_check(aw_dev);
    if (ret < 0)
    {
        aw_dev_reg_dump(aw_dev);
        aw_dev_pwd(aw_dev, true);
        aw_dev_dbg(aw_dev->dev_index, "pll check failed cannot start");
        return ret;
    }

    /*amppd on*/
    aw_dev_amppd(aw_dev, false);
    AW_MS_DELAY(AW_1_MS);

    /*check i2s status*/
    ret = aw_dev_sysst_check(aw_dev);
    if (ret < 0)
    {
        aw_dev_reg_dump(aw_dev);
        /*close tx feedback*/
        if (aw_dev->ops.aw_i2s_enable)
        {
            aw_dev->ops.aw_i2s_enable(aw_dev, false);
        }
        /*clear interrupt*/
        aw_dev_clear_int_status(aw_dev);
        /*close amppd*/
        aw_dev_amppd(aw_dev, true);
        /*power down*/
        aw_dev_pwd(aw_dev, true);
        return -EINVAL;
    }

    /*boost type recover*/
    aw_dev_boost_type_recover(aw_dev);

    /*enable tx feedback*/
    if (aw_dev->ops.aw_i2s_enable)
    {
        aw_dev->ops.aw_i2s_enable(aw_dev, true);
    }

    if (aw_dev->amppd_st)
    {
        aw_dev_amppd(aw_dev, true);
    }

    if (aw_dev->ops.aw_reg_force_set)
    {
        aw_dev->ops.aw_reg_force_set(aw_dev);
    }

    /*close uls hmute*/
    aw_dev_uls_hmute(aw_dev, false);

    if (!aw_dev->mute_st)
    {
        /*close mute*/
        aw_dev_mute(aw_dev, false);
    }

    /*clear inturrupt*/
    aw_dev_clear_int_status(aw_dev);
    /*set inturrupt mask*/
    aw_dev_set_intmask(aw_dev, true);
#ifdef AW_MONITOR
    aw_monitor_start((void *)aw_dev);
#endif
    aw_dev->status = AW_DEV_PW_ON;
    aw_dev_info(aw_dev->dev_index, "done");
    return 0;
}

int aw_device_stop(struct aw_device *aw_dev)
{
    aw_dev_dbg(aw_dev->dev_index, "enter");

    if (aw_dev->status == AW_DEV_PW_OFF)
    {
        aw_dev_dbg(aw_dev->dev_index, "already power off");
        return 0;
    }

    aw_dev->status = AW_DEV_PW_OFF;
#ifdef AW_MONITOR
    aw_monitor_stop((void *)aw_dev);
#endif
    /*clear interrupt*/
    aw_dev_clear_int_status(aw_dev);

    /*set defaut int mask*/
    aw_dev_set_intmask(aw_dev, false);

    /*set uls hmute*/
    aw_dev_uls_hmute(aw_dev, true);

    /*set mute*/
    aw_dev_mute(aw_dev, true);

    /*close tx feedback*/
    if (aw_dev->ops.aw_i2s_enable)
    {
        aw_dev->ops.aw_i2s_enable(aw_dev, false);
    }

    AW_MS_DELAY(AW_1_MS);

    /*enable amppd*/
    aw_dev_amppd(aw_dev, true);

    /*set power down*/
    aw_dev_pwd(aw_dev, true);

    aw_dev_info(aw_dev->dev_index, "done");
    return 0;
}

int aw_device_probe(struct aw_device *aw_dev, struct aw_fill_info *fill_info)
{
    int ret = -1;

    ret = aw_dev_prof_init(aw_dev, fill_info);
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "prof init failed");
        return ret;
    }
#ifdef AW_MONITOR
    ret = aw_monitor_init((void *)aw_dev);
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "monitor init failed");
        return ret;
    }
#endif
    ret = aw_device_init(aw_dev);
    if (ret < 0)
    {
        aw_dev_err(aw_dev->dev_index, "dev init failed");
        return ret;
    }

    return 0;
}

