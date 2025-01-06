/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-08-08     Yang        the first version
 */

#include <rtthread.h>
#include "board.h"
#include "ztw622.h"
#include "drv_touch.h"
#include "string.h"

//#include "zinitix_firmware.h"


#ifdef TSC_USING_ZTW622

#define DRV_DEBUG
#define  DBG_LEVEL        DBG_ERROR  //DBG_LOG //
#define LOG_TAG              "drv.ztw622"
#include <drv_log.h>


#define CHIP_ID_REG                 0xCC00
#define CHIP_ID_VALUE               0xE628
#define TOUCH_SLAVE_ADDR            0x20
#define X_NODE_NUM                  6
#define Y_NODE_NUM                  6
#define I2C_BUFFER_SIZE             2/*2bytes*/
#define TC_SECTOR_SZ                8
#define REG_TYPE                    uint16_t


#define GPIO_TP_RESET 16
#define GPIO_TP_1V8_EN 0

#define GPIO_TP_INT  TOUCH_IRQ_PIN

extern void HAL_Delay_us(__IO uint32_t us);

#define ztw_hw_us_delay(us) HAL_Delay_us(us)
#define gpio_set_level(pin,level) rt_pin_write(pin,level)
#define gpio_get_level(pin) rt_pin_read(pin)

static struct touch_drivers ztw_driver = {0};
static struct rt_i2c_bus_device *ft_bus = NULL;
struct _ts_zinitix_point_info touch_info;

static void ts_isr_enable(rt_bool_t enable)
{
    rt_touch_irq_pin_enable(enable ? 1 : 0);
}
void ts_touch_isr(void *args)
{
    LOG_D("ts_touch_isr");

    rt_sem_release(ztw_driver.isr_sem);
}
static int ts_read(REG_TYPE addr, rt_uint8_t *buffer, rt_size_t length)
{
    int ret = -1;
    int retries = 0;

    struct rt_i2c_msg msgs[] =
    {
        {
            .addr   = TOUCH_SLAVE_ADDR,
            .flags  = RT_I2C_WR,
            .len    = sizeof(addr),
            .buf    = (uint8_t *) &addr,
        },
        {
            .addr   = TOUCH_SLAVE_ADDR,
            .flags  = RT_I2C_RD,
            .len    = length,
            .buf    = buffer,
        },
    };

    while (retries < IIC_RETRY_NUM)
    {
        ret = rt_i2c_transfer(ft_bus, msgs, 2);
        if (ret == 2)break;
        retries++;
    }

    if (retries >= IIC_RETRY_NUM)
    {
        LOG_E("%s i2c read error: %d", __func__, ret);
        return -1;
    }

    return ret;
}

static int ts_write(REG_TYPE addr, rt_uint8_t *buffer, rt_size_t length)
{
    int ret;
    REG_TYPE *send_buffer = malloc(length + sizeof(addr));
    RT_ASSERT(send_buffer);
    send_buffer[0] = addr;
    memcpy(send_buffer + 1, buffer, length);

    struct rt_i2c_msg msgs[] =
    {
        {
            .addr   = TOUCH_SLAVE_ADDR,
            .flags  = RT_I2C_WR,
            .len    = length + sizeof(addr),
            .buf    = (uint8_t *)send_buffer,
        }
    };

    ret = rt_i2c_transfer(ft_bus, msgs, 1);
    free(send_buffer);
    send_buffer = RT_NULL;
    return ret;
}


static rt_err_t ts_write_reg(REG_TYPE addr, REG_TYPE data)
{
    if (ts_write(addr, (uint8_t *)&data, sizeof(addr)))
        return RT_EOK;
    else
        return RT_ERROR;
}

static rt_err_t ts_read_reg(REG_TYPE addr, REG_TYPE *data)
{
    if (ts_read(addr, (uint8_t *)data, 2))
        return RT_EOK;
    else
        return RT_ERROR;
}
static rt_err_t ts_write_cmd(REG_TYPE cmd)
{
    uint16_t data = 1;
    return ts_write_reg(cmd, data);
}


static void ts_power_control(rt_bool_t enable)
{
    if (enable)
    {
        rt_thread_mdelay(1);
        gpio_set_level(GPIO_TP_RESET, 1);
        rt_thread_mdelay(10);
        gpio_set_level(GPIO_TP_RESET, 0);
        rt_thread_mdelay(100);
        gpio_set_level(GPIO_TP_RESET, 1);
        rt_thread_mdelay(CHIP_ON_DELAY);
    }
    else
    {
        gpio_set_level(GPIO_TP_RESET, 0);
        rt_thread_mdelay(CHIP_OFF_DELAY);
    }
}


static rt_err_t ts_power_sequence(void)
{
    uint16_t chip_code;

    if (ts_write_reg(ZINITIX_VENDOR_REG, 0x0001) != RT_EOK) /**/
    {
        LOG_E("power sequence error (vendor cmd enable)\n");
        return RT_ERROR;
    }
    ztw_hw_us_delay(10);/*10us*/

    if (ts_read(CHIP_ID_REG, (uint8_t *)&chip_code, 2) == 0)
    {
        LOG_E("fail to read chip code\n");
        return RT_ERROR;
    }
    ztw_hw_us_delay(10);
    LOG_D("chip code = 0x%x\n", chip_code);

    if (ts_write_cmd(0xc004) != RT_EOK) /*cmd*/
    {
        LOG_E("power sequence error (intn clear)\n");
        return RT_ERROR;
    }
    ztw_hw_us_delay(10);/*10us*/

    if (ts_write_reg(ZINITIX_NVM_REG, 0x0001) != RT_EOK) /*cmd*/
    {
        LOG_E("power sequence error (nvm init)\n");
        return RT_ERROR;
    }
    rt_thread_mdelay(2);
    if (ts_write_reg(0xc001, 0x0001) != RT_EOK) /*cmd*/
    {
        LOG_E("power sequence error (program start)\n");
        return RT_ERROR;
    }
    rt_thread_mdelay(FIRMWARE_ON_DELAY);/*delay*/
    LOG_D("ts_power_sequence PASS.\n");

    return RT_EOK;
}


#ifdef TOUCH_TEST_CMD
static uint8_t get_raw_data(uint16_t *buff, uint16_t skip_cnt)
{
    uint32_t num_reg;
    for (uint16_t i = 0; i < skip_cnt; i++)
    {
        ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD);
        rt_thread_mdelay(1);
    }


    for (uint16_t i = 0; i < 10; i++)
    {
        while (gpio_get_level(GPIO_TP_INT))
        {
            rt_thread_mdelay(1);
        }
        ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD);
        rt_thread_mdelay(10);
    }

    num_reg = X_NODE_NUM * Y_NODE_NUM * 2;

    uint8_t i;

    while (gpio_get_level(GPIO_TP_INT))
    {
        rt_thread_mdelay(1);
    }
    ts_read(ZINITIX_RAWDATA_REG, (uint8_t *)(buff), num_reg);

    for (i = 0; i < 36; i++)
    {
        LOG_D("%d\n", *(buff + i));
    }

    ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD);

    rt_thread_mdelay(10);
//  ts_isr_enable(ENABLE);
    return 1;
}

static uint8_t get_short_data(uint16_t *buff, uint16_t skip_cnt)
{
    uint32_t num_reg;

    for (uint16_t i = 0; i < skip_cnt; i++)
    {
        ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD);
        rt_thread_mdelay(1);
    }


    for (uint16_t i = 0; i < 10; i++)
    {
        while (gpio_get_level(GPIO_TP_INT))
        {
            rt_thread_mdelay(1);
        }
        ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD);
        rt_thread_mdelay(10);
    }

    num_reg = (X_NODE_NUM + Y_NODE_NUM) * 2;

    uint8_t i;

    while (gpio_get_level(GPIO_TP_INT))
    {
        rt_thread_mdelay(1);
    }
    ts_read(ZINITIX_RAWDATA_REG, (uint8_t *)(buff), num_reg);

    for (i = 0; i < 24; i++)
    {
        LOG_D("%d\n", *(buff + i));
    }

    ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD);

    rt_thread_mdelay(10);
//  ts_isr_enable(ENABLE);
    return 1;
}
#endif /* TOUCH_TEST_CMD */

uint16_t get_firmware_version(void)
{
    uint16_t version;
    ts_read_reg(ZINITIX_FIRMWARE_VERSION, &version); /*固件主版本号u16 firmware_version;*/
    return version;
}
uint16_t get_minor_firmware_version(void)
{
    uint16_t version;
    ts_read_reg(ZINITIX_MINOR_FW_VERSION, &version);/*固件次版本号u16 minor_firmware_version;*/
    return version;
}
uint16_t get_reg_data_version(void)
{
    uint16_t version;
    ts_read_reg(ZINITIX_DATA_VERSION_REG, &version) ;/*固件寄存器版本号u16 reg_data_version;*/
    return version;
}


#if TOUCH_ONESHOT_UPGRADE
static rt_err_t ts_check_need_upgrade(uint16_t curRegVersion, uint16_t curr_firmware_ver, uint16_t minor_version)
{
    uint16_t    new_version, new_minor_version, new_reg_version;

    new_version = (uint16_t)(m_firmware_data[52] | (m_firmware_data[53] << 8));
    new_minor_version = (uint16_t)(m_firmware_data[56] | (m_firmware_data[57] << 8));
    new_reg_version = (uint16_t)(m_firmware_data[60] | (m_firmware_data[61] << 8));

    LOG_D("cur firmware version = 0x%x, new firmware version = 0x%x\n", curr_firmware_ver, new_version);
    LOG_D("cur minor firmware version = 0x%x, new minor firmware version = 0x%x\n", minor_version, new_minor_version);
    LOG_D("cur reg data version = 0x%x, new reg data version = 0x%x\n", curRegVersion, new_reg_version);
    if (curr_firmware_ver < new_version)
        return RT_EOK;
    else
        return RT_ERROR;

    if (minor_version < new_minor_version)
        return RT_EOK;
    else
        return RT_ERROR;

    if (curRegVersion < new_reg_version)
        return RT_EOK;
    else
        return RT_ERROR;

    return RT_ERROR;
}

static rt_err_t ts_upgrade_firmware(struct touch_drivers *driver)
{
    uint16_t flash_addr, chip_code;
    uint8_t verify_data[TC_SECTOR_SZ];
    int i, retry_cnt = 0, page_sz = 64;

retry_upgrade:
    ts_power_control(DISABLE);
    ts_power_control(ENABLE);

    if (ts_write_reg(0xc000, 0x0001) != RT_EOK)
    {
        LOG_E("power sequence error (vendor cmd enable)\n");
        goto fail_upgrade;
    }
    ztw_hw_us_delay(10);

    if (ts_read(0xcc00, (uint8_t *)&chip_code, 2) == 0)
    {
        LOG_E("fail to read chip code\n");
        goto fail_upgrade;
    }
    LOG_E("chip code = 0x%x\n", chip_code);

    /*Link modify the flash size*/
//  if(chip_code == ZT7554_CHIP_CODE)
//      page_sz = 64*1024;
//  else if ((chip_code == ZT7548_CHIP_CODE)||(chip_code == ZTW523_CHIP_CODE))
//      page_sz = 48*1024;
//  else if ((chip_code == ZT7538_CHIP_CODE)||(chip_code == ZTW522_CHIP_CODE))
//      page_sz = 44*1024;

    ztw_hw_us_delay(10);
    if (ts_write_cmd(0xc004) != RT_EOK)
    {
        LOG_E("power sequence error (intn clear)\n");
        goto fail_upgrade;
    }
    ztw_hw_us_delay(10);

    if (ts_write_reg(0xc002, 0x0001) != RT_EOK)
    {
        LOG_E("power sequence error (nvm init)\n");
        goto fail_upgrade;
    }
    rt_thread_mdelay(10);

    LOG_D("init flash\n");

    if (ts_write_reg(0xc003, 0x0001) != RT_EOK)
    {
        LOG_E("fail to write nvm vpp on\n");
        goto fail_upgrade;
    }
    rt_thread_mdelay(1);

    if (ts_write_reg(0xc104, 0x0001) != RT_EOK)
    {
        LOG_E("fail to write nvm wp disable\n");
        goto fail_upgrade;
    }

    if (ts_write_cmd(ZINITIX_INIT_FLASH) != RT_EOK)
    {
        LOG_E("fail to init flash\n");
        goto fail_upgrade;
    }
    ztw_hw_us_delay(10);

    LOG_D("writing firmware data\n");
    for (flash_addr = 0; flash_addr < sizeof(m_firmware_data);)
    {
        for (i = 0; i < page_sz / TC_SECTOR_SZ; i++)
        {
            if (ts_write(ZINITIX_WRITE_FLASH, (uint8_t *)&m_firmware_data[flash_addr], TC_SECTOR_SZ) == 0)
            {
                LOG_E("error : write zinitix tc firmare\n");
                goto fail_upgrade;
            }
            flash_addr += TC_SECTOR_SZ;
            ztw_hw_us_delay(150);
        }
        rt_thread_mdelay(30);
    }
    rt_thread_mdelay(100);

    if (ts_write_reg(0xc003, 0x0000) != RT_EOK)
    {
        LOG_E("fail to write nvm vpp on\n");
        goto fail_upgrade;
    }

    if (ts_write_reg(0xc104, 0x0000) != RT_EOK)
    {
        LOG_E("fail to write nvm wp disable\n");
        goto fail_upgrade;
    }

    if (ts_write_cmd(ZINITIX_INIT_FLASH) != RT_EOK)
    {
        LOG_E("fail to init flash\n");
        goto fail_upgrade;
    }

    LOG_D("read firmware data\n");
    for (flash_addr = 0; flash_addr < sizeof(m_firmware_data);)
    {
        for (i = 0; i < page_sz / TC_SECTOR_SZ; i++)
        {
            LOG_D("read :addr=%04x, len=%d\n", flash_addr, TC_SECTOR_SZ);

            if (ts_read(ZINITIX_READ_FLASH, verify_data, TC_SECTOR_SZ) == 0)
            {
                LOG_E("error : read zinitix tc firmare\n");
                goto fail_upgrade;
            }
            /*verify*/
            if (memcmp(verify_data, &m_firmware_data[flash_addr], TC_SECTOR_SZ) != 0)
            {
                LOG_E("verify firmware data err\n");
                goto fail_upgrade;
            }

            flash_addr += TC_SECTOR_SZ;
        }
    }
    LOG_D("upgrade finished\n");

    ts_power_control(DISABLE);
    ts_power_control(ENABLE);
    ts_power_sequence();
    return RT_EOK;
fail_upgrade:
    ts_power_control(DISABLE);
    if (retry_cnt++ < ZINITIX_INIT_RETRY_CNT)
    {
        LOG_E("upgrade fail : so retry... (%d)\n", retry_cnt);
        goto retry_upgrade;
    }
    LOG_E("upgrade fail..\n");
    return RT_ERROR;
}
#endif



static rt_err_t init(void)
{
    uint8_t i;
    uint16_t eeprom_info, reg_ver, firmware_ver, minor_ver;

    gpio_set_level(GPIO_TP_1V8_EN, 1);
    rt_thread_mdelay(TP_POWERON_DELAY);
    /*RESET*/
    ts_power_control(ENABLE);

    rt_touch_irq_pin_attach(PIN_IRQ_MODE_FALLING, ts_touch_isr, NULL);
    rt_touch_irq_pin_enable(1); //Must enable before read I2C


    if (ts_power_sequence() != RT_EOK)
        return RT_ERROR;

    if (ts_read_reg(ZINITIX_EEPROM_INFO, &eeprom_info) != RT_EOK)
        LOG_E("fail to read eeprom info");

    for (i = 0; i < 10; i++)
    {
        if (ts_write_cmd(ZINITIX_SWRESET_CMD) == 0)
            break;/*return 0 mean write success then break*/
        rt_thread_mdelay(10);
    }
    ts_write_reg(ZINITIX_INT_ENABLE_FLAG, 0);



    for (i = 0; i < 10; i++)
    {
        if (ts_write_cmd(ZINITIX_SWRESET_CMD) == 0)
            break;/*return 0 mean write success then break*/
        rt_thread_mdelay(10);
    }
    uint16_t reg_val = 0;
    zinitix_bit_set(reg_val, BIT_PT_CNT_CHANGE);
    zinitix_bit_set(reg_val, BIT_DOWN);
    zinitix_bit_set(reg_val, BIT_MOVE);
    zinitix_bit_set(reg_val, BIT_UP);
    zinitix_bit_set(reg_val, BIT_UP);

#if SUPPORTED_PALM_TOUCH
    zinitix_bit_set(reg_val, BIT_PALM);
#endif

//  zinitix_bit_set(reg_val, BIT_PT_EXIST);


//  if(ts_check_need_upgrade(get_reg_data_version(i2c_bus)) == RT_EOK) {
//      if(ts_upgrade_firmware(i2c_bus) != RT_EOK)
//          return RT_ERROR;
//  }

    if (ts_read_reg(ZINITIX_EEPROM_INFO, &eeprom_info) != RT_EOK) /*?*/
        LOG_E("fail to read eeprom info");

    firmware_ver = get_firmware_version();
    minor_ver = get_minor_firmware_version();
    reg_ver = get_reg_data_version();

    ts_write_reg(ZINITIX_SUPPORTED_FINGER_NUM, MAX_SUPPORTED_FINGER_NUM);
    ts_write_reg(ZINITIX_X_RESOLUTION, (TPD_RES_MAX_X));
    ts_write_reg(ZINITIX_Y_RESOLUTION, (TPD_RES_MAX_Y));

    ts_write_reg(ZINITIX_INITIAL_TOUCH_MODE, TOUCH_POINT_MODE);
    ts_write_reg(ZINITIX_TOUCH_MODE, TOUCH_POINT_MODE);

    ts_write_reg(ZINITIX_INT_ENABLE_FLAG, reg_val);

    for (i = 0; i < 10; i++)
    {
        ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD);
        ztw_hw_us_delay(10);
    }
    return RT_EOK;
}

static rt_bool_t probe(void)
{
    ft_bus = (struct rt_i2c_bus_device *)rt_device_find(TOUCH_DEVICE_NAME);
    if (RT_Device_Class_I2CBUS != ft_bus->parent.type)
    {
        ft_bus = NULL;
    }
    if (ft_bus)
    {
        rt_device_open((rt_device_t)ft_bus, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    }
    else
    {
        LOG_I("bus not find\n");
        return RT_FALSE;
    }

    return RT_TRUE;
}



static rt_err_t deinit(void)
{
    LOG_D("ztw622 deinit");

    rt_touch_irq_pin_enable(0);
    return RT_EOK;

}
static rt_err_t read_point(touch_msg_t p_msg)
{
    LOG_D("get_point");

    /*lock?*/
    uint8_t i;
    if (ts_read(ZINITIX_POINT_STATUS_REG, (uint8_t *) & (touch_info), sizeof(touch_info)))
    {
        (ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD));
//      return RT_ERROR;
    }
    if (touch_info.status == 0)
    {
        (ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD));
        return RT_ERROR;
    }

#if SUPPORTED_PALM_TOUCH
    if (zinitix_bit_test(touch_info.status, BIT_PALM))
    {

        LOG_D("large touch palm enter\n");
        /*
            control lcd off here

        */
    }

#endif


    for (i = 0; i < MAX_SUPPORTED_FINGER_NUM; i++)
    {

        if (zinitix_bit_test(touch_info.coord[i].sub_status, SUB_BIT_EXIST))
        {
            //p_msg->event = TOUCH_EVENT_EXIST;
            p_msg->x = touch_info.coord[i].x;
            p_msg->y = touch_info.coord[i].y;
            if (zinitix_bit_test(touch_info.coord[i].sub_status, SUB_BIT_DOWN))
            {
                p_msg->event = TOUCH_EVENT_DOWN;
                LOG_D("EVENT:TP DOWM \nX:%d Y:%d", p_msg->x, p_msg->y);
            }
            else
            {
                LOG_D("X:%d Y:%d", p_msg->x, p_msg->y);
            }
        }
        else if (zinitix_bit_test(touch_info.coord[i].sub_status, SUB_BIT_UP))
        {
            p_msg->event = TOUCH_EVENT_UP;
            p_msg->x = touch_info.coord[i].x;
            p_msg->y = touch_info.coord[i].y;
            LOG_D("EVENT:TP UP \nX:%d Y:%d", p_msg->x, p_msg->y);
        }
        else
        {
            p_msg->event = TOUCH_EVENT_NONE;
            memset(&touch_info.coord[i], 0x0, sizeof(struct _ts_zinitix_coord));
            return RT_ERROR;
        }
    }
    if (ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD))
        return RT_ERROR;

    return RT_EEMPTY;
}

static struct touch_ops ops =
{
    read_point,
    init,
    deinit
};


static int ztw_driver_init(void)
{
    ztw_driver.probe = probe;
    ztw_driver.ops = &ops;
    ztw_driver.user_data = RT_NULL;
    ztw_driver.isr_sem = rt_sem_create("ztw622", 0, RT_IPC_FLAG_FIFO);

    rt_touch_drivers_register(&ztw_driver);

    return 0;
}
INIT_COMPONENT_EXPORT(ztw_driver_init);


/*API*/
rt_err_t zinitix_sleep(uint16_t skip_cnt)
{
    ts_isr_enable(DISABLE);/*触发无效中断*/
    rt_thread_mdelay(10);
    while (ts_write_cmd(ZINITIX_SLEEP_CMD) != RT_EOK)
    {
        if (skip_cnt-- == 0)
        {
            return RT_ERROR;
        }
        rt_thread_mdelay(1);
    }
    return RT_EOK;
}
rt_err_t zinitix_weekup(uint16_t skip_cnt)
{
    gpio_set_level(GPIO_TP_RESET, 1);
    rt_thread_mdelay(1);
    gpio_set_level(GPIO_TP_RESET, 0);
    rt_thread_mdelay(10);
    gpio_set_level(GPIO_TP_RESET, 1);
    rt_thread_mdelay(1);
    rt_thread_mdelay(50);

    ts_power_sequence();

    //zinitix_init(&ztw_driver);
    ts_isr_enable(ENABLE);
    return RT_EOK;
}
#if TOUCH_TEST_CMD

rt_err_t zinitix_open_test(struct touch_drivers *driver, uint16_t *recbuff)
{

    //disable irq
    ts_write_cmd(ZINITIX_SWRESET_CMD);
    rt_thread_mdelay(20);

    ts_write_reg(ZINITIX_DND_U_COUNT, SEC_PDND_U_COUNT);
    ts_write_reg(ZINITIX_AFE_FREQUENCY, SEC_PDND_FREQUENCY);
    ts_write_reg(ZINITIX_DND_N_COUNT, SEC_PDND_N_COUNT);

    ts_write_reg(ZINITIX_DELAY_RAW_FOR_HOST, RAWDATA_DELAY_FOR_HOST);

    if (ts_write_reg(ZINITIX_TOUCH_MODE, 11) == RT_EOK)
    {

    }
    rt_thread_mdelay(1);

    ts_write_cmd(ZINITIX_SWRESET_CMD);
    rt_thread_mdelay(20);

    ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD);
    //写模式11结束

    get_raw_data(driver, recbuff, 10);


    ts_write_reg(ZINITIX_TOUCH_MODE, TOUCH_POINT_MODE);

    for (uint8_t i = 0; i < 10; i++)
    {
        rt_thread_mdelay(20);
        ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD);
    }
    //enable irq

    //写模式TOUCH_POINT_MODE结束
    int16_t raw_min[] =
    {
        -200, 2211, 4053, 3839, 1990, -200,
            2114, 4201, 4199, 4280, 4563, 2627,
            4033, 4211, 4206, 4319, 4775, 4630,
            4018, 4215, 4194, 4329, 4782, 4679,
            2062, 4194, 4193, 4315, 4624, 3319,
            -200, 2049, 3871, 4176, 2388, -200,
        };
    int16_t raw_max[] =
    {
        30000,  4963,   7205,   6825,   4570,   30000,
        4792,   7469,   7465,   7610,   8113,   5703,
        7171,   7487,   7476,   7679,   8489,   8232,
        7144,   7493,   7456,   7697,   8502,   8319,
        4700,   7456,   7453,   7671,   8220,   6933,
        30000,  4675,   6883,   7424,   5278,   30000,
    };

    int16_t H_DIFF_REFERENCE[X_NODE_NUM][Y_NODE_NUM - 1] =
    {
        200, 55, 19, 39, 200,
        60,  10, 13, 16, 31,
        13,  10, 13, 19, 10,
        13,  10, 12, 19, 11,
        50,  10, 10, 16, 50,
        200, 51, 18, 55, 200,
    };

    int16_t V_DIFF_REFERENCE[X_NODE_NUM - 1][Y_NODE_NUM] =
    {
        200, 60, 18, 12, 46, 200,
        58,  10, 10, 10, 11, 33,
        11,  10, 10, 10, 10, 11,
        48,  10, 10, 11, 15, 52,
        200, 51, 10, 18, 63, 200,
    };

    float *H_DIFF = (float *)&recbuff;
    float *V_DIFF = (float *)&recbuff;
    uint32_t H_DIFF_VALUE[X_NODE_NUM][Y_NODE_NUM - 1];
    uint32_t V_DIFF_VALUE[X_NODE_NUM - 1][Y_NODE_NUM];

    /*****************Rawdata test*******************/
    for (uint8_t i = 0; i < X_NODE_NUM * Y_NODE_NUM; i++)
    {

        if (recbuff[i] < raw_min[i] || recbuff[i] >  raw_max[i])
        {
            return RT_ERROR;
        }
    }
    LOG_D("Rawdata test pass!\n");
    /*****************H_DIFF test*******************/
    for (uint8_t i = 0; i < (X_NODE_NUM * Y_NODE_NUM - 1); i++)
    {

        H_DIFF_VALUE[i] = (int16_t)(zinitix_abs(H_DIFF[i], H_DIFF[i + 1]) / (float)zinitix_max(H_DIFF[i], H_DIFF[i + 1]) * 100);
    }


#if 0
    for (uint8_t i = 0; i < X_NODE_NUM; i++)
    {

        for (uint8_t j = 0; j < (Y_NODE_NUM - 1); j++)
        {

            H_DIFF_VALUE[i][j] = (zinitix_abs(H_DIFF[i][j], H_DIFF[i][j + 1]) / zinitix_max(H_DIFF[i][j], H_DIFF[i][j + 1]) * 100);

            /******print all H_DIFF  Value  here ******/

            /*  if (H_DIFF[i][j] > H_DIFF_REFERENCE[i][j]) {
                    return RT_ERROR;
                }*/
        }
    }
#endif
    LOG_D("H_DIFF test pass!\n");
    /*****************V_DIFF test*******************/
    for (uint8_t i = 0; i < (X_NODE_NUM - 1); i++)
    {

        for (uint8_t j = 0; j < Y_NODE_NUM; j++)
        {

            V_DIFF_VALUE[i][j] = (uint32_t)(zinitix_abs(V_DIFF[i][j], V_DIFF[i + 1][j]) / zinitix_max(V_DIFF[i][j], V_DIFF[i + 1][j]) * 100);


            /******print all V_DIFF  Value  here ******/

            /*if (V_DIFF[i][j] > V_DIFF_REFERENCE[i][j]) {
                return RT_ERROR;
            }*/
        }
    }
    LOG_D("V_DIFF test pass!\n");

    return RT_EOK;
}


rt_err_t zinitix_short_test(struct touch_drivers *driver, uint16_t *recbuff)
{
    ts_write_cmd(ZINITIX_SWRESET_CMD);
    rt_thread_mdelay(20);

    ts_write_reg(ZINITIX_DND_U_COUNT, SEC_PDND_U_COUNT);
    ts_write_reg(ZINITIX_AFE_FREQUENCY, SEC_PDND_FREQUENCY);
    ts_write_reg(ZINITIX_DND_N_COUNT, SEC_PDND_N_COUNT);

    ts_write_reg(ZINITIX_DELAY_RAW_FOR_HOST, RAWDATA_DELAY_FOR_HOST);

    if (ts_write_reg(ZINITIX_TOUCH_MODE, TOUCH_CHECK_SHORT_MODE) == RT_EOK)
    {

    }
    rt_thread_mdelay(1);

    ts_write_cmd(ZINITIX_SWRESET_CMD);
    rt_thread_mdelay(20);

    ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD);
    //写模式11结束

    get_short_data(driver, recbuff, 10);

    ts_write_reg(ZINITIX_TOUCH_MODE, TOUCH_POINT_MODE);

    for (uint8_t i = 0; i < 10; i++)
    {
        rt_thread_mdelay(20);
        ts_write_cmd(ZINITIX_CLEAR_INT_STATUS_CMD);
    }
    //写模式TOUCH_POINT_MODE结束

    for (uint8_t i = 0; i < X_NODE_NUM + Y_NODE_NUM; i++)
    {

        if (recbuff[i] != NORMAL_SHORT_VALUE)
        {
            return RT_ERROR;
        }
    }
    return RT_EOK;
}

void print_rawdata()
{
    uint16_t buffer[36];
    zinitix_open_test(&ztw_driver, buffer);
    zinitix_short_test(&ztw_driver, buffer);
}
MSH_CMD_EXPORT(print_rawdata, print rawdata);
#endif /* TOUCH_TEST_CMD */

void ts_goto_sleep()
{
    zinitix_sleep(1);
}
MSH_CMD_EXPORT(ts_goto_sleep,);
void ts_weekup()
{
    zinitix_weekup(1);
}
MSH_CMD_EXPORT(ts_weekup,);

#if TOUCH_ONESHOT_UPGRADE
void ts_upgrade()
{
    ts_upgrade_firmware(&ztw_driver);
}
MSH_CMD_EXPORT(ts_upgrade,);
#endif /* TOUCH_ONESHOT_UPGRADE */
#endif
