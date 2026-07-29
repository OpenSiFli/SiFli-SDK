/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "board.h"
#include "tma525b.h"
#include "drv_touch.h"
//#include "EventRecorder.h"

/* Define -------------------------------------------------------------------*/

//#define DRV_DEBUG
#define  DBG_LEVEL            DBG_INFO  //DBG_LOG // 
#define LOG_TAG              "drv.tma525b"
#include <drv_log.h>

#define I2C_ADDR                    (0x24)

#define PT_MAX_PIP_MSG_SIZE 264

#define PIP_MSG_HEAD_SIZE    5   /*2B report len + 1B report id + 2B timestamp*/

#define TMA_MAX_WIDTH                   (454)
#define TMA_MAX_HEIGHT                  (454)

typedef enum
{
    PIP_REG_HID_DESC        = 0x01,
    PIP_REG_REPORT_DESC     = 0x02,
    PIP_REG_INPUT_REPORT    = 0x03,
    PIP_REG_OUTPUT_REPORT   = 0x04,
    PIP_COMMAND             = 0x05,
    PIP_DATA                = 0x06,
} PIP_REG_T;

typedef enum
{
    PIP_REPORT_INVALID = 0x00,
    PIP_REPORT_TOUCH   = 0x01,
    PIP_REPORT_BUTTON  = 0x03,
    PIP_NON_HID_RESPONSE = 0x1F,
    PIP_NON_HID_COMMAND = 0x2F,

} PIP_REPORT_ID_T;

enum PIP1_CMD_ID
{
    PIP1_CMD_ID_NULL = 0x00,
    PIP1_CMD_ID_START_BOOTLOADER = 0x01,
    PIP1_CMD_ID_GET_SYSINFO = 0x02,
    PIP1_CMD_ID_SUSPEND_SCANNING = 0x03,
    PIP1_CMD_ID_RESUME_SCANNING = 0x04,
    PIP1_CMD_ID_GET_PARAM = 0x05,
    PIP1_CMD_ID_SET_PARAM = 0x06,
    PIP1_CMD_ID_GET_NOISE_METRICS = 0x07,
    PIP1_CMD_ID_RESERVED = 0x08,
    PIP1_CMD_ID_ENTER_EASYWAKE_STATE = 0x09,
    PIP1_CMD_ID_EXIT_EASYWAKE_STATE = 0x10,
    PIP1_CMD_ID_VERIFY_CONFIG_BLOCK_CRC = 0x20,
    PIP1_CMD_ID_GET_CONFIG_ROW_SIZE = 0x21,
    PIP1_CMD_ID_READ_DATA_BLOCK = 0x22,
    PIP1_CMD_ID_WRITE_DATA_BLOCK = 0x23,
    PIP1_CMD_ID_GET_DATA_STRUCTURE = 0x24,
    PIP1_CMD_ID_LOAD_SELF_TEST_PARAM = 0x25,
    PIP1_CMD_ID_RUN_SELF_TEST = 0x26,
    PIP1_CMD_ID_GET_SELF_TEST_RESULT = 0x27,
    PIP1_CMD_ID_CALIBRATE_IDACS = 0x28,
    PIP1_CMD_ID_INITIALIZE_BASELINES = 0x29,
    PIP1_CMD_ID_EXEC_PANEL_SCAN = 0x2A,
    PIP1_CMD_ID_RETRIEVE_PANEL_SCAN = 0x2B,
    PIP1_CMD_ID_START_SENSOR_DATA_MODE = 0x2C,
    PIP1_CMD_ID_STOP_SENSOR_DATA_MODE = 0x2D,
    PIP1_CMD_ID_START_TRACKING_HEATMAP_MODE = 0x2E,
    PIP1_CMD_ID_START_SELF_CAP_RPT_MODE = 0x2F,
    PIP1_CMD_ID_CALIBRATE_DEVICE_EXTENDED = 0x30,
    PIP1_CMD_ID_INT_PIN_OVERRIDE = 0x40,
    PIP1_CMD_ID_STORE_PANEL_SCAN = 0x60,
    PIP1_CMD_ID_PROCESS_PANEL_SCAN = 0x61,
    PIP1_CMD_ID_DISCARD_INPUT_REPORT,
    PIP1_CMD_ID_LAST,
    PIP1_CMD_ID_USER_CMD,
};

#pragma pack(push, 1) //make sure no padding bytes between report_id and desc

typedef struct _touch_record_t
{
    uint8_t touch_type : 3;
    uint8_t reservd1   : 5;

    uint8_t touch_id   : 5;
    uint8_t event_id   : 2;
    uint8_t tip        : 1;

    uint16_t x;
    uint16_t y;

    uint8_t pressure;
    uint8_t major_ax_len;
    uint8_t minor_ax_len;
    uint8_t orientation;
} touch_record_t;

typedef struct _touch_desc_t
{
    uint16_t timestamp;
    uint8_t nb_records : 5;
    uint8_t large_obj  : 1;
    uint8_t reservd1   : 2;

    uint8_t noise_effect : 3;
    uint8_t reservd2     : 3;
    uint8_t report_cnt   : 2;

    touch_record_t record[10];

} touch_desc_t;

typedef struct _rsp_desc_t
{
    uint8_t reservd1;
    uint8_t cmd : 7;
    uint8_t TGL  : 1;
    uint8_t ret_data[10];
} rsp_desc_t;

typedef struct _pip_in_report_t
{
    uint16_t report_len;
    uint8_t report_id;

    union
    {
        touch_desc_t touch_rpt;
        rsp_desc_t   rsp_rpt;
    } desc;
} pip_in_report_t;

typedef struct _pip_out_report_t
{
    PIP_REG_T reg_addr;
    uint16_t report_len; //Size of pip_out_report_t, Not include reg_addr
    uint8_t report_id;
    uint8_t rsvd1;
    uint8_t cmd;
    uint8_t param[4];
} pip_out_report_t;

#pragma pack(pop)

static pip_in_report_t *p_pip_in_report = NULL;

// rotate to left with 90, 180, 270
// rotate to left with 360 for mirror
//#define IT_ROTATE_LEFT                 (90)

/* function and value-----------------------------------------------------------*/
static rt_err_t i2c_read(rt_uint8_t *buf, rt_uint16_t len);
static rt_err_t i2c_write(rt_uint8_t *buf, rt_uint16_t len);
static rt_err_t tma525b_parse_touch_report(touch_desc_t *pt_desc, touch_msg_t p_msg);

static struct rt_i2c_bus_device *ft_bus = NULL;
static uint32_t start_tick;

static void tma525b_parse_rsp_report(rsp_desc_t *rsp_desc, uint16_t ret_len)
{
    uint8_t i;

    LOG_D("tma525b rsp cmd %xh,", rsp_desc->cmd);
    LOG_HEX("ret_data", 8, rsp_desc->ret_data, ret_len);
}

static struct touch_drivers tma525b_driver;

static rt_err_t i2c_write(rt_uint8_t *buf, rt_uint16_t len)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs;

    msgs.addr  = I2C_ADDR;    /* slave address */
    msgs.flags = RT_I2C_WR;        /* write flag */
    msgs.buf   = buf;              /* Send data pointer */
    msgs.len   = len;

    if (rt_i2c_transfer(ft_bus, &msgs, 1) == 1)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

static rt_err_t i2c_read(rt_uint8_t *buf, rt_uint16_t len)
{
    rt_int8_t res = 0;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = I2C_ADDR;    /* Slave address */
    msgs[0].flags = RT_I2C_RD;        /* Read flag */
    msgs[0].buf   = buf;              /* Read data pointer */
    msgs[0].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(ft_bus, msgs, 1) == 1)
    {
        res = RT_EOK;
    }
    else
    {
        res = -RT_ERROR;
    }
    return res;
}

#if 1

typedef enum
{
    RC_PASS = 0,
    RC_FAIL = -1
} PT_RC_CHECK;

#define u8 uint8_t
#define u16 uint16_t
#define HIGH_BYTE(x) (u8)(((x) >> 8) & 0xFF)
#define LOW_BYTE(x) (u8)((x)&0xFF)

typedef struct
{
    u8 reg[2];
    u16 len;
    u8 report_id;
    u8 rsvd;
    u8 cmd_id;
} __packed PIP1_CMD_HEADER;

#define PT_CMD_OUTPUT_REGISTER  0x0004
static uint8_t touch_write_buf[256];
static PT_RC_CHECK pt_send_pip1_cmd(int cmd_id, u8 *data, u8 data_len)
{
    PT_RC_CHECK rc;
    uint16_t index = 0;

    PIP1_CMD_HEADER pip1_header = {0};
    pip1_header.reg[0] = LOW_BYTE(PT_CMD_OUTPUT_REGISTER);
    pip1_header.reg[1] = HIGH_BYTE(PT_CMD_OUTPUT_REGISTER);
    /* head.len(2)report id(1) rsvd(1) cmd_id(1) */
    pip1_header.len = data_len + 0x05;
    pip1_header.report_id = 0x2f;
    pip1_header.rsvd = 0x00;
    pip1_header.cmd_id = cmd_id;

    LOG_D("pt_send_pip1_cmd %x, %x", cmd_id, *((uint32_t *)data));

    index = 0;
    rt_memcpy(&touch_write_buf[index], (const void *)&pip1_header, sizeof(pip1_header));
    index += sizeof(pip1_header);

    if (data && data_len)
    {
        rt_memcpy(&touch_write_buf[index], data, data_len);
        index += data_len;
    }

    rc = i2c_write(touch_write_buf, index);
    return rc;
}
/*
ie.
   tma_cmd 05 0a       read 0x0a register
   tma_cmd 05 0a010f   write 0x0a register, length 0x01, value 0x0f
*/
static rt_err_t tma_cmd(int argc, char **argv)
{
    uint8_t data[16];
    uint8_t cmd;
    uint8_t len;

    cmd = atoh(argv[1]);
    if (argc > 2)
    {
        len = hex2data(argv[2], data, 16);
        pt_send_pip1_cmd(cmd, data, len);
    }
    else
        pt_send_pip1_cmd(cmd, NULL, 0);

    LOG_D("tma525b send cmd %d", cmd);
    HAL_Delay_us(5);
    rt_sem_release(tma525b_driver.isr_sem);
    return 0;
}
MSH_CMD_EXPORT(tma_cmd, tma_cmd);

#endif

static rt_err_t tma525b_parse_touch_report(touch_desc_t *pt_desc, touch_msg_t p_msg)
{
    rt_err_t ret = RT_ERROR;

    uint8_t num_cur_tch;
    uint8_t i, touch_id, event_id;
    uint16_t x, y ;

    if (pt_desc)
    {
        if (pt_desc->large_obj)
        {
            LOG_D("Large Object detected\n");
        }

        num_cur_tch = pt_desc->nb_records;
        if (num_cur_tch)
        {
            for (i = 0; i < num_cur_tch; i ++)
            {
                touch_id = pt_desc->record[i].touch_id;
                event_id = pt_desc->record[i].event_id;

#ifdef LCD_USING_RM69090
                y = pt_desc->record[i].x;
                x = pt_desc->record[i].y;
#else
                x = TMA_MAX_WIDTH - pt_desc->record[i].x;
                y = TMA_MAX_HEIGHT - pt_desc->record[i].y;
#endif /* LCD_USING_RM69330 */

                //Improvement: return all data
                if (i + 1 != num_cur_tch)
                    LOG_E("tma525b lost event(%d): ID=%d, X=%d, Y=%d", event_id, touch_id, x, y);

                if (event_id == 3)
                {
                    LOG_D("(%d): Lift, ID=%d, X=%d, Y=%d\n", i, touch_id, x, y);
                    p_msg->event = TOUCH_EVENT_UP;
                    p_msg->x = 0;
                    p_msg->y = 0;
                    ret = RT_EOK;

                }
                else
                {
                    LOG_D("(%d): Down, ID=%d, X=%d, Y=%d\n", i, touch_id, x, y);
                    p_msg->event = TOUCH_EVENT_DOWN;
                    p_msg->x = x;
                    p_msg->y = y;
                    ret = RT_EOK;

                }
            }
        }
        else
        {
            LOG_D("All Lift-off\n");
            p_msg->event = TOUCH_EVENT_UP;
            p_msg->x = 0;
            p_msg->y = 0;

            ret = RT_EOK;
        }
        //rt_kprintf("read more: %d %d %d\n", pos_rec_beg, pos_rec_end, num_cur_tch);

    }

    return ret;
}

void tma525b_irq_handler(void *arg)
{
    rt_err_t ret = RT_ERROR;

    LOG_D("tma525b touch_irq_handler\n");

    rt_touch_irq_pin_enable(0);

    ret = rt_sem_release(tma525b_driver.isr_sem);
    RT_ASSERT(RT_EOK == ret);
}

static rt_err_t read_point(touch_msg_t p_msg)
{
    rt_err_t ret = RT_ERROR;

    LOG_D("tma525b read_point");
    rt_touch_irq_pin_enable(1);

    if (start_tick != UINT32_MAX) //Send cmd below only once after tma525b initialed.
    {
        if ((rt_tick_get() - start_tick) > (RT_TICK_PER_SECOND * 3 / 2))
        {
            LOG_I("Config tma525b start.");
            //Wait to tma525b system ready (Calibration Routine Execution Time)
            {
                uint32_t act_dist0 = 0x0001010A;
                pt_send_pip1_cmd(PIP1_CMD_ID_SET_PARAM, (uint8_t *)&act_dist0, 3);

                uint32_t act_dist2 = 0x0001010B;
                pt_send_pip1_cmd(PIP1_CMD_ID_SET_PARAM, (uint8_t *)&act_dist2, 3);
            }
            start_tick = UINT32_MAX;
            LOG_I("Config tma525b end.");
        }
    }
    //read report length
    while (RT_EOK == i2c_read((rt_uint8_t *)p_pip_in_report, 2))
    {
        if ((p_pip_in_report->report_len < PIP_MSG_HEAD_SIZE)
                || (p_pip_in_report->report_len > PT_MAX_PIP_MSG_SIZE)
           )
        {
            //Invalid report
            return RT_EINVAL;
        }

        //read whole report
        if (RT_EOK == i2c_read((rt_uint8_t *)p_pip_in_report, p_pip_in_report->report_len))
        {
            switch (p_pip_in_report->report_id)
            {
            case PIP_REPORT_TOUCH:
                ret = tma525b_parse_touch_report(&p_pip_in_report->desc.touch_rpt, p_msg);
                return ret;
                break;

            case PIP_NON_HID_RESPONSE:
                tma525b_parse_rsp_report(&p_pip_in_report->desc.rsp_rpt, p_pip_in_report->report_len - 5);
                break;

            default:
                //LOG_D("tma525b report id %d", p_pip_in_report->report_id);
                break;
            }
        }
    }

    return RT_ERROR;
}

static rt_err_t init(void)
{
    struct touch_message msg;

    LOG_I("tma525b init");
    BSP_TP_Reset(0);
    rt_thread_mdelay(100);
    BSP_TP_Reset(1);

    rt_touch_irq_pin_attach(PIN_IRQ_MODE_FALLING, tma525b_irq_handler, NULL);
    rt_touch_irq_pin_enable(1); //Must enable before read I2C

    start_tick = rt_tick_get();

    while (RT_EINVAL != read_point(&msg))
    {
        //Read all previous commands report.
        if ((rt_tick_get() - start_tick) > RT_TICK_PER_SECOND)
        {
            LOG_E("tma525b init read point TIMEOUT!");
            break;
        }

        rt_thread_mdelay(100);
    }

    LOG_I("tma525b init OK");
    return RT_EOK;

}

static rt_err_t deinit(void)
{
    LOG_I("tma525b deinit");

    rt_touch_irq_pin_enable(0);

    return RT_EOK;
}

static rt_bool_t probe(void)
{
    rt_err_t err;

    if (NULL == p_pip_in_report)
    {
        p_pip_in_report = (pip_in_report_t *) rt_malloc(sizeof(pip_in_report_t));
    }

    if (NULL == p_pip_in_report)
    {
        LOG_E("tma525b malloc memory fail\n");
        return RT_FALSE;
    }

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

    {
        struct rt_i2c_configuration configuration =
        {
            .mode = 0,
            .addr = 0,
            .timeout = 500,
            .max_hz  = 400000,
        };

        rt_i2c_configure(ft_bus, &configuration);
    }

    LOG_I("tma525b probe OK");

    return RT_TRUE;
}

static struct touch_ops ops =
{
    read_point,
    init,
    deinit
};

static int rt_tma525b_init(void)
{
    tma525b_driver.probe = probe;
    tma525b_driver.ops = &ops;
    tma525b_driver.user_data = RT_NULL;
    tma525b_driver.isr_sem = rt_sem_create("tma525b", 0, RT_IPC_FLAG_FIFO);

    rt_touch_drivers_register(&tma525b_driver);

    return 0;
}
INIT_COMPONENT_EXPORT(rt_tma525b_init);

