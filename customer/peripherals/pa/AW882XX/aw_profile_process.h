#ifndef __AW_PROFILE_PROCESS_H__
#define __AW_PROFILE_PROCESS_H__

#include "aw_device.h"


/*******************awinic audio parse acf***********************/
enum profile_data_type
{
    AW_PROFILE_DATA_TYPE_REG = 0,
    AW_PROFILE_DATA_TYPE_MAX,
};

struct aw_monitor_data_desc
{
    uint32_t len;
    const unsigned char *data;
};

struct aw_sec_data_desc
{
    uint32_t len;
    const unsigned char *data;

};

struct aw_prof_desc
{
    char name [AW_PROF_NAME_MAX];
    struct aw_sec_data_desc sec_desc[AW_PROFILE_DATA_TYPE_MAX];
};

struct aw_prof_info
{
    uint32_t chip_id;       /*The ID corresponding to the chip*/
    uint32_t count;         /*profile count*/
    struct aw_prof_desc *prof_desc;/*Describes the configuration of each scenario*/
    struct aw_monitor_data_desc *monitor_data;
};

int aw_dev_check_prof(aw_dev_index_t dev_index, struct aw_prof_info *prof_info);
struct aw_sec_data_desc *aw_dev_get_prof_data_byname(struct aw_device *aw_dev, char *prof_name, int data_type);
int aw_dev_set_profile_name(struct aw_device *aw_dev, const char *prof_name);
char *aw_dev_get_profile_name(struct aw_device *aw_dev);
int aw_dev_prof_init(struct aw_device *aw_dev, struct aw_fill_info *fill_info);

#endif
