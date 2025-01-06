/*
* aw_profile_process.c
*
* Copyright (c) 2021 AWINIC Technology CO., LTD
*
* Author: <zhaolei@awinic.com>
*
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "aw_profile_process.h"
#include "aw_device.h"
#include "aw_base.h"

int aw_dev_check_prof(aw_dev_index_t dev_index, struct aw_prof_info *prof_info)
{
    struct aw_prof_desc *prof_desc = NULL;
    int i = 0;
    int j = 0;

    if (prof_info == NULL)
    {
        aw_dev_err(dev_index, "prof_info is NULL");
        return -EINVAL;
    }

    if (prof_info->count <= 0)
    {
        aw_dev_err(dev_index, "prof count :%d unsupported", prof_info->count);
        return -EINVAL;
    }

    prof_desc = prof_info->prof_desc;
    for (i = 0; i < prof_info->count; i++)
    {
        if (prof_desc == NULL)
        {
            aw_dev_err(dev_index, "invalid prof_desc");
            return -EINVAL;
        }

        if (prof_desc->sec_desc->len <= 0)
        {
            aw_dev_err(dev_index, "prof len:%d unsupported", prof_desc->sec_desc->len);
            return -EINVAL;
        }

        if (prof_desc->sec_desc->data == NULL)
        {
            aw_dev_err(dev_index, "prof data is NULL");
            return -EINVAL;
        }
    }

    for (i = 0; i < prof_info->count; i++)
    {
        for (j = i + 1; j < prof_info->count; j++)
        {
            if (strncmp(prof_info->prof_desc[i].name, prof_info->prof_desc[j].name, AW_PROF_NAME_MAX) == 0)
            {
                aw_dev_err(dev_index, "prof_desc pos[%d] and prof_desc pos[%d] conflict with prof_name[%s]",
                           i, j, prof_info->prof_desc[j].name);
                return -EINVAL;
            }
        }
    }

    return 0;
}

struct aw_sec_data_desc *aw_dev_get_prof_data_byname(struct aw_device *aw_dev, char *prof_name, int data_type)
{
    struct aw_sec_data_desc *sec_data = NULL;
    struct aw_prof_desc *prof_desc = NULL;
    struct aw_prof_info *prof_info = aw_dev->prof_info;
    int i = 0;

    if (data_type >= AW_PROFILE_DATA_TYPE_MAX)
    {
        aw_dev_err(aw_dev->dev_index, "unsupport data type id [%d]", data_type);
        return NULL;
    }

    for (i = 0; i < prof_info->count; i++)
    {
        if (strncmp(prof_name, prof_info->prof_desc[i].name, AW_PROF_NAME_MAX) == 0)
        {
            prof_desc = &aw_dev->prof_info->prof_desc[i];
            sec_data = &prof_desc->sec_desc[data_type];
            aw_dev_dbg(aw_dev->dev_index, "get prof[%s] data len[%d]",
                       prof_desc->name, sec_data->len);
            return sec_data;
        }
    }

    aw_dev_err(aw_dev->dev_index, "not found prof_name[%s]", prof_name);
    return NULL;
}

static int aw_dev_check_profile_name(struct aw_device *aw_dev, const char *prof_name)
{
    int i = 0;
    struct aw_prof_info *prof_info = aw_dev->prof_info;

    for (i = 0; i < prof_info->count; i++)
    {
        if (strncmp(prof_name, prof_info->prof_desc[i].name, AW_PROF_NAME_MAX) == 0)
        {
            return 0;
        }
    }
    aw_dev_err(aw_dev->dev_index, "not found prof_name[%s]", prof_name);
    return -EINVAL;
}

int aw_dev_set_profile_name(struct aw_device *aw_dev, const char *prof_name)
{
    if (aw_dev_check_profile_name(aw_dev, prof_name))
    {
        return -EINVAL;
    }
    else
    {
        strncpy(aw_dev->set_prof_name, prof_name, AW_PROF_NAME_MAX - 1);
        aw_dev_info(aw_dev->dev_index, "set prof_name[%s]", aw_dev->set_prof_name);
    }

    return 0;
}

char *aw_dev_get_profile_name(struct aw_device *aw_dev)
{
    return aw_dev->set_prof_name;
}

int aw_dev_prof_init(struct aw_device *aw_dev, struct aw_fill_info *fill_info)
{
    int i = 0;
    const char *first_prof_name = NULL;
    /*find profile*/
    for (i = 0; i < fill_info->mix_chip_count; i++)
    {
        if (fill_info->prof_info[i].chip_id == aw_dev->chip_id)
        {
            aw_dev->prof_info = &fill_info->prof_info[i];
            first_prof_name = fill_info->prof_info[i].prof_desc[0].name;
            strncpy(aw_dev->first_prof_name, first_prof_name, AW_PROF_NAME_MAX - 1);
            aw_dev_info(aw_dev->dev_index, "first prof_name[%s]", aw_dev->first_prof_name);
            return 0;
        }
    }

    aw_dev_err(aw_dev->dev_index, "no supported profile");
    return -EINVAL;
}

