#ifndef _AW882XX_H_
#define _AW882XX_H_

#include "aw_device.h"
#include "aw_fill_info.h"
#include "aw_audio_common.h"
#include "aw_base.h"


#define AW882XX_CHIP_ID_REG     (0x00)
#define MAX_I2C_BUFFER_SIZE     (65536)
#define AW882XX_I2C_READ_MSG_NUM    (2)
#define AW_START_RETRIES        (5)

#define AW882XX_SOFT_RESET_REG      (0x00)
#define AW882XX_SOFT_RESET_VALUE    (0x55aa)


#define AW_PID_2055_VERSION_DIFF_REG    (0x23)
#define AW_I2C_RETRIES          (5) /* 5 times */
#define AW_I2C_RETRY_DELAY      (5) /* 5 ms */

enum aw882xx_chipid
{
    PID_1852_ID = 0x1852,
    PID_2013_ID = 0x2013,
    PID_2032_ID = 0x2032,
    PID_2055_ID = 0x2055,
    PID_2055A_ID = 0x2055A,
    PID_2071_ID = 0x2071,
    PID_2113_ID = 0x2113,
};

enum aw882xx_int_type
{
    INT_TYPE_NONE = 0,
    INT_TYPE_UVLO = 0x1,
    INT_TYPE_BSTOC = 0x2,
    INT_TYPE_OCDI = 0x4,
    INT_TYPE_OTHI = 0x8,
};

enum
{
    AW_BSTCFG_DISABLE = 0,
    AW_BSTCFG_ENABLE,
};

enum
{
    AW_FRCSET_DISABLE = 0,
    AW_FRCSET_ENABLE,
};

enum
{
    AW_BOP_DISABLE = 0,
    AW_BOP_ENABLE,
};

typedef enum
{
    AW_PIN_RESET = 0,
    AW_PIN_SET,
} aw_pinstate;


/********************************************
 * struct aw882xx
 *******************************************/
struct aw882xx
{
    unsigned int chip_id;

    aw_dev_index_t dev_index;
    unsigned char phase_sync;
    unsigned char i2c_addr;

    struct aw_device *aw_pa;
    aw_hw_irq_handle_t irq_handle;

    int (*i2c_read_func)(uint16_t dev_addr, uint8_t reg_addr,
                         uint8_t *pdata, uint16_t len);
    int (*i2c_write_func)(uint16_t dev_addr, uint8_t reg_addr,
                          uint8_t *pdata, uint16_t len);
    void (*reset_gpio_ctl)(bool PinState);
};

int aw882xx_get_version(char *buf, int size);
int aw882xx_get_dev_num(void);
int aw882xx_init(struct aw882xx *aw882xx, struct aw_fill_info *fill_info);
int aw882xx_i2c_write(struct aw882xx *aw882xx,
                      unsigned char reg_addr, unsigned int reg_data);
int aw882xx_i2c_read(struct aw882xx *aw882xx,
                     unsigned char reg_addr, unsigned int *reg_data);
int aw882xx_i2c_write_bits(struct aw882xx *aw882xx,
                           unsigned char reg_addr, unsigned int mask, unsigned int reg_data);

int aw882xx_hw_reset_by_index(aw_dev_index_t dev_index);
int aw882xx_soft_reset(aw_dev_index_t dev_index);
int aw882xx_reg_show(aw_dev_index_t dev_index);
int aw882xx_reg_store(aw_dev_index_t dev_index, uint8_t reg_addr, uint16_t reg_data);

int aw882xx_set_fade_time(aw_fade_dir_t fade_dir, uint32_t time);
int aw882xx_get_fade_time(aw_fade_dir_t fade_dir, uint32_t *time);
int aw882xx_get_fade_step(aw_dev_index_t dev_index, uint32_t *step);
int aw882xx_set_fade_step(aw_dev_index_t dev_index, uint32_t step);
int aw882xx_get_volume(aw_dev_index_t dev_index, uint32_t *volume);
int aw882xx_set_volume(aw_dev_index_t dev_index,  uint32_t volume);

int aw882xx_set_profile_byname(aw_dev_index_t dev_index, char *prof_name);
int aw882xx_ctrl_state(aw_dev_index_t dev_index, aw_ctrl_t aw_ctrl);
int aw882xx_smartpa_init(void *aw_info);
void aw882xx_smartpa_deinit(aw_dev_index_t dev_index);

aw_hw_irq_handle_t aw882xx_get_hw_irq_status(aw_dev_index_t dev_index);
void aw882xx_irq_handler(aw_dev_index_t dev_index);
void aw882xx_irq_trigger(aw_dev_index_t dev_index);

void aw882xx_monitor_work(aw_dev_index_t dev_index);
void aw882xx_monitor_set_status(aw_dev_index_t dev_index);
int aw882xx_set_fs(aw_dev_index_t dev_index,  uint32_t fs, uint8_t chn_sel);

#endif

