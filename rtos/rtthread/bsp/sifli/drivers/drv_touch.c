/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-02-08     Zhangyihong  the first version
 */

#include "drv_touch.h"
#include "bf0_hal.h"
#include <string.h>
#include "drv_io.h"
#include "mem_section.h"
#ifdef BSP_USING_TOUCHD
#ifndef BSP_TOUCH_SAMPLE_HZ
    #if 0 //def LV_DISP_DEF_REFR_PERIOD
        #define BSP_TOUCH_SAMPLE_HZ    (LV_DISP_DEF_REFR_PERIOD * 2)
    #else
        #define BSP_TOUCH_SAMPLE_HZ    (120)
    #endif
#endif

#define DBG_ENABLE
#define DBG_SECTION_NAME  "TOUCH"
#define DBG_LEVEL          DBG_INFO  //DBG_LOG //
#define DBG_COLOR

/*
    Define REMOTE_TOUCH_OUTPUT_DEVICE - Output tp data to specified device
    Define REMOTE_TOUCH_INPUT_DEVICE  - Read tp data from specified  device
*/
//#define REMOTE_TOUCH_OUTPUT_DEVICE "uart3"
//#define REMOTE_TOUCH_INPUT_DEVICE "uart1"


#include <rtdbg.h>


static struct rt_device g_touch_device;
static touch_drv_t current_driver = NULL;
static struct rt_thread touch_thread;
static rt_thread_t p_touch_thread;
static rt_sem_t api_lock;
ALIGN(RT_ALIGN_SIZE)
L1_NON_RET_BSS_SECT_BEGIN(touch_thread_stack)
L1_NON_RET_BSS_SECT(touch_thread_stack, static char touch_thread_stack[1024]);
L1_NON_RET_BSS_SECT_END


#define MAX_TOUCH_REC 16


static rt_mutex_t more_data_lock;

/*
    if pos_rec_beg == pos_rec_end, EMPTY

    if pos_rec_end + 1 == pos_rec_beg,  FULL
*/
#define REC_BUF_FULL()  (((pos_rec_end + 1) & (MAX_TOUCH_REC - 1)) == pos_rec_beg)
#define REC_BUF_EMPTY()  (pos_rec_end == pos_rec_beg)

static struct touch_message pos_rec[MAX_TOUCH_REC];
static int8_t pos_rec_beg = 0;//The record that NOT be readed.
static int8_t pos_rec_end = 0;//The record that can be write into.

static struct touch_message last_rec = {0, 0, TOUCH_EVENT_UP};
#if (DBG_LEVEL == DBG_LOG)
    static bool enable_tp_buf_log = true;
#else
    static bool enable_tp_buf_log = false;
#endif

static bool rotate_180 = false;
static struct rt_device_rect_info rotate_rect;

static void touch_write_more(rt_uint8_t  event, rt_uint16_t x, rt_uint16_t  y)
{
    rt_bool_t send_indicate = RT_FALSE;

    if (rotate_180)
    {
        //Rotate piont in rotate_rect
        if ((x >= rotate_rect.x) && (x < rotate_rect.x + rotate_rect.width)
                && (y >= rotate_rect.y) && (y < rotate_rect.y + rotate_rect.height))
        {
            x = FLIP_V_BY_AREA(x, rotate_rect.x, rotate_rect.x + rotate_rect.width - 1);
            y = FLIP_V_BY_AREA(y, rotate_rect.y, rotate_rect.y + rotate_rect.height - 1);
        }
    }

    if (last_rec.event == event && last_rec.x == x && last_rec.y == y)
    {
        return;
    }

    rt_mutex_take(more_data_lock, RT_WAITING_FOREVER);

    if (REC_BUF_FULL())
    {
        static uint8_t g_touch_full  = 0;
        g_touch_full++;
        if (g_touch_full == 1)
        {
            LOG_W("touch buffer overwrite[%d, %d %d] with [%d, %d %d]\n",
                  pos_rec[pos_rec_end].event,
                  pos_rec[pos_rec_end].x,
                  pos_rec[pos_rec_end].y,
                  event, x, y);
        }
    }
    else if (enable_tp_buf_log)
    {
        LOG_I("touch buffer in[%d][%d, %d %d]\n", pos_rec_end, event, x, y);
    }

    pos_rec[pos_rec_end].event = event;
    pos_rec[pos_rec_end].x = x;
    pos_rec[pos_rec_end].y = y;
    last_rec = pos_rec[pos_rec_end];

    if (!REC_BUF_FULL())
    {
        pos_rec_end = (pos_rec_end + 1) & (MAX_TOUCH_REC - 1);
        send_indicate = RT_TRUE;
    }
    else
    {
        send_indicate = RT_FALSE; //Overwrite touch data, not to send indicate
    }
    rt_mutex_release(more_data_lock);

    if (send_indicate && g_touch_device.rx_indicate)
    {
        g_touch_device.rx_indicate(&g_touch_device, 1);
    }

}


static rt_list_t driver_list;

void rt_touch_drivers_register(touch_drv_t drv)
{
    LOG_I("Regist touch screen driver, probe=%p ", drv->probe);

    rt_list_insert_before(&driver_list, &drv->list);
}

/*
    Touch api lock: prevent nested call current_driver->ops->xxx
*/
static void touch_api_lock(void)
{
    rt_err_t err;
    err = rt_sem_take(api_lock, RT_WAITING_FOREVER);
    RT_ASSERT(RT_EOK == err);
}
static void touch_api_unlock(void)
{
    rt_err_t err;
    err = rt_sem_release(api_lock);
    RT_ASSERT(RT_EOK == err);
}


#ifdef BSP_TOUCH_IRQ_FROM_DATASVC
#include "data_service.h"
#include "pin_service.h"

static datac_handle_t pin_irq_service;
static uint8_t pin_irq_service_connect_state = 0; // 0 - not connect, 1 - connecting, 2 - connected, 3 - connect error
static void (*irq_pin_hdr)(void *args);
static void  *irq_pin_hdr_args;

static int irq_pin_service_callback(data_callback_arg_t *arg)
{
    if (MSG_SERVICE_DATA_NTF_IND == arg->msg_id)
    {
        pin_common_msg_t pin_msg;
        RT_ASSERT(arg != NULL);
        RT_ASSERT(sizeof(pin_common_msg_t) == arg->data_len);
        memcpy(&pin_msg, arg->data, arg->data_len);
        RT_ASSERT(pin_msg.id == TOUCH_IRQ_PIN);
        if (irq_pin_hdr)irq_pin_hdr(irq_pin_hdr_args);
    }
    else if (MSG_SERVICE_SUBSCRIBE_RSP == arg->msg_id)
    {
        data_subscribe_rsp_t *rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        LOG_I("Subscribe irq_pin_service ret %d", rsp->result);
        if (rsp->result == 0)
        {
            LOG_D("rt_touch irq_pin_service connected");
            pin_irq_service_connect_state = 2;
        }
        else
        {
            LOG_E("rt_touch irq_pin_service fail");
            pin_irq_service_connect_state = 3;
        }

    }
    else if (MSG_SERVICE_UNSUBSCRIBE_RSP == arg->msg_id)
    {
        LOG_I("Unsubscribe irq_pin_service RSP.");
        pin_irq_service_connect_state = 0;
        pin_irq_service = DATA_CLIENT_INVALID_HANDLE;
    }
    else
    {
        LOG_D("irq_pin_service_callback %x", arg->msg_id);
    }

    return 0;
}


static rt_err_t subscribe_irq_pin_service(void)
{
    uint32_t retry_count = 0;
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE == pin_irq_service);

    pin_irq_service = datac_open();
    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != pin_irq_service);

    do
    {

        if (1 != pin_irq_service_connect_state)
        {
            datac_subscribe(pin_irq_service,
#ifdef SOC_BF0_HCPU
                            "pin_l",
#else
                            "pin_h",
#endif /* SOC_BF0_HCPU */
                            irq_pin_service_callback, 0);
        }

        rt_thread_delay(RT_TICK_PER_SECOND / BSP_TOUCH_SAMPLE_HZ); //Let system breath

        if (3 == pin_irq_service_connect_state)
        {
            retry_count++;
            LOG_E("Connect irq_pin_service error, retry... %d", retry_count);
            rt_thread_mdelay((rt_int32_t)100 * retry_count); //Delay some milliseconds to retry
        }

        if (retry_count > 10)
        {
            LOG_E("Connect irq_pin_service timeout.");
            return RT_ERROR;
        }
    }
    while (2 != pin_irq_service_connect_state);

    return RT_EOK;
}

static rt_err_t unsubscribe_irq_pin_service(void)
{
    if (DATA_CLIENT_INVALID_HANDLE != pin_irq_service)
    {
        /*
            Unsubscirbe RSP msg will fast than other request msg, and cause assert in data_service.c
            Delay 10ms for waitting previous service request rsp.
            Remove it if bug fixed.
        */
        rt_thread_mdelay(10);
        //datac_unsubscribe(pin_irq_service);
        datac_close(pin_irq_service);
        pin_irq_service_connect_state = 0;
        pin_irq_service = DATA_CLIENT_INVALID_HANDLE;
    }

    return RT_EOK;
}


rt_err_t rt_touch_irq_pin_attach(rt_uint32_t mode,
                                 void (*hdr)(void *args), void  *args)
{
    rt_err_t ret;
    pin_config_msg_t config;

    config.id = TOUCH_IRQ_PIN;
    config.mode = PIN_MODE_INPUT;
    config.irq_mode = mode;
    config.flag = PIN_SERVICE_FLAG_SET_IRQ_MODE | PIN_SERVICE_FLAG_AUTO_DISABLE_IRQ;

    RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != pin_irq_service);
    ret = datac_config(pin_irq_service, sizeof(pin_config_msg_t), (uint8_t *)&config);
    LOG_I("ret %d", ret);

    irq_pin_hdr = hdr;
    irq_pin_hdr_args = args;
    return RT_EOK;
}

rt_err_t rt_touch_irq_pin_detach(void)
{

    data_msg_t msg;
    pin_common_msg_t *p_pin_msg;
    p_pin_msg = (pin_common_msg_t *) data_service_init_msg(&msg, PIN_MSG_DETACH_IRQ_REQ, sizeof(pin_common_msg_t));

    p_pin_msg->id = TOUCH_IRQ_PIN;
    return datac_send_msg(pin_irq_service, &msg);
}

rt_err_t rt_touch_irq_pin_enable(rt_uint32_t enabled)
{

    data_msg_t msg;
    pin_common_msg_t *p_pin_msg;
    p_pin_msg = (pin_common_msg_t *) data_service_init_msg(&msg,
                enabled ? PIN_MSG_ENABLE_IRQ_REQ : PIN_MSG_DISABLE_IRQ_REQ,
                sizeof(pin_common_msg_t));

    p_pin_msg->id = TOUCH_IRQ_PIN;
    return datac_send_msg(pin_irq_service, &msg);
}

#else
rt_err_t rt_touch_irq_pin_attach(rt_uint32_t mode,
                                 void (*hdr)(void *args), void  *args)
{
    LOG_D("rt_touch_pin_attach_irq init irq pin=%d", TOUCH_IRQ_PIN);

    rt_pin_mode(TOUCH_IRQ_PIN, PIN_MODE_INPUT);
    rt_pin_attach_irq(TOUCH_IRQ_PIN, mode, hdr, args);

    return RT_EOK;
}

rt_err_t rt_touch_irq_pin_detach(void)
{
    return rt_pin_detach_irq(TOUCH_IRQ_PIN);
}

rt_err_t rt_touch_irq_pin_enable(rt_uint32_t enabled)
{
    return rt_pin_irq_enable(TOUCH_IRQ_PIN, enabled);
}
#endif /* BSP_TOUCH_IRQ_FROM_DATASVC */

#ifdef REMOTE_TOUCH_INPUT_DEVICE
struct rt_semaphore rx_sem;
static rt_err_t rmt_in_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);

    return RT_EOK;
}
static void touch_remote_in_entry(void *parameter)
{
    struct touch_message msg;
    rt_err_t err;

    rt_device_t p_remote_touch = NULL;

    rt_thread_mdelay(2000);//Wait system init done

    p_remote_touch = rt_device_find(REMOTE_TOUCH_INPUT_DEVICE);
    if (p_remote_touch != RT_NULL)
    {
        err = rt_device_open(p_remote_touch, RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_STREAM);

        if (RT_EOK != err)
        {
            LOG_E("Err =%d, Can't open device "REMOTE_TOUCH_INPUT_DEVICE, err);
            return;
        }
    }
    else
    {
        LOG_E("Can't open device "REMOTE_TOUCH_INPUT_DEVICE);
        return;
    }



    rt_sem_init(&rx_sem, "rmttp", 0, 0);
    rt_device_set_rx_indicate(p_remote_touch, rmt_in_rx_ind);

    LOG_I("Touch input from rmt device "REMOTE_TOUCH_INPUT_DEVICE);
    current_driver = (touch_drv_t) 1; //To cheat drv_touch.read() function
    do
    {

        //rt_device_read(p_remote_touch, 0, &msg, sizeof(msg));
        for (rt_size_t i = 0; i < sizeof(msg) - 1;)
        {
            uint8_t *p = (uint8_t *) &msg;
            rt_size_t len = rt_device_read(p_remote_touch, 0, p + i, sizeof(msg) - i);


            for (rt_size_t j = 0; j < len; j++)
            {
                LOG_D("rmt touch in %x", *(p + i + j));
            }


            if (0 == len)
                rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
            else
                i += len;
        }

        switch (msg.event)
        {
        case TOUCH_EVENT_DOWN:
        case TOUCH_EVENT_UP:
            touch_write_more(msg.event, msg.x, msg.y);
            break;
        default:
        {
            //Data was lost, drop 1 byte every time util fix it.
            uint8_t dummy;
            rt_device_read(p_remote_touch, 0, &dummy, 1);
            (void) dummy;
        }
        break;
        }
        LOG_D("rmt touch in[%d, %d %d]", msg.event, msg.x, msg.y);

        rt_thread_delay(RT_TICK_PER_SECOND / BSP_TOUCH_SAMPLE_HZ); //Let system breath
    }
    while (err == RT_EOK);
}

#endif /* REMOTE_TOUCH_INPUT_DEVICE */

static void tp_init_thread_entry(void *parameter)
{
    rt_sem_t tp_init_lock = (rt_sem_t) parameter;
    rt_err_t err;

    touch_api_lock();
    do
    {
#ifdef BSP_TOUCH_IRQ_FROM_DATASVC
        if (RT_EOK != subscribe_irq_pin_service())
        {
            break;
        }
#endif /* BSP_TOUCH_IRQ_FROM_DATASVC */


        if (NULL == current_driver)
        {
            LOG_I("Find touch screen driver...");

            for (rt_list_t *l = driver_list.next; l != &driver_list; l = l->next)
            {
                rt_bool_t (*probe_func)(void);

                probe_func = rt_list_entry(l, struct touch_drivers, list)->probe;

                LOG_I("Probe %x", probe_func);
                if (probe_func())
                {
                    current_driver = rt_list_entry(l, struct touch_drivers, list);
                    break;
                }
            }
            if (current_driver == RT_NULL)
            {
                LOG_E("no touch screen or do not have driver");
                break;
            }

            RT_ASSERT(current_driver->isr_sem);
            LOG_I("touch screen found driver  %p, %s", current_driver, &current_driver->isr_sem->parent.parent.name[0]);
        }



        //touch->ops->isr_enable(RT_TRUE);

        RT_ASSERT(current_driver->ops->init);
        current_driver->ops->init();
    }
    while (0);
    touch_api_unlock();

    rt_sem_release(tp_init_lock);
}

static rt_err_t tp_init(void)
{
    rt_sem_t tp_init_lock = rt_sem_create("tp_init", 0, RT_IPC_FLAG_FIFO);

    RT_ASSERT(tp_init_lock != NULL);

    /*Temporary thread with large stack for tp init only*/
    rt_thread_t init_task = rt_thread_create("tp_init",
                            tp_init_thread_entry,
                            (void *)tp_init_lock,
                            4096,
                            RT_THREAD_PRIORITY_HIGH,
                            RT_THREAD_TICK_DEFAULT);
    RT_ASSERT(init_task != NULL);
    rt_thread_startup(init_task);
    rt_sem_take(tp_init_lock, RT_WAITING_FOREVER);

    rt_sem_delete(tp_init_lock);

    return (NULL == current_driver) ? RT_ERROR : RT_EOK;
}

#ifdef REMOTE_TOUCH_INPUT_DEVICE
/*Tp main thread*/
static void tp_read_thread_entry(void *parameter)
{
    touch_remote_in_entry(parameter);
}

#else /*!REMOTE_TOUCH_INPUT_DEVICE*/

/*Tp main thread*/
static void tp_read_thread_entry(void *parameter)
{
    struct touch_message msg;
    rt_tick_t emouse_id = 0;
    rt_err_t err;

#ifdef REMOTE_TOUCH_OUTPUT_DEVICE
    rt_device_t p_remote_touch = NULL;

    p_remote_touch = rt_device_find(REMOTE_TOUCH_OUTPUT_DEVICE);
    if (p_remote_touch != RT_NULL)
    {
        err = rt_device_open(p_remote_touch, RT_DEVICE_OFLAG_WRONLY);

        if (RT_EOK != err)
        {
            LOG_E("Err =%d, Can't open device "REMOTE_TOUCH_OUTPUT_DEVICE, err);
            return;
        }
    }
    else
    {
        LOG_E("Can't find device "REMOTE_TOUCH_OUTPUT_DEVICE);
        return;
    }
    LOG_I("Touch output to rmt device "REMOTE_TOUCH_OUTPUT_DEVICE);
#endif /* REMOTE_TOUCH_INPUT_DEVICE */

    if (RT_EOK != tp_init())
    {
        LOG_E("tp_init error");
        return;
    }

    while (1)
    {
        if (rt_sem_take(current_driver->isr_sem, RT_WAITING_FOREVER) != RT_EOK)
        {
            continue;
        }

        //touch->ops->isr_enable(RT_TRUE);
        do
        {
            touch_api_lock();
            err = current_driver->ops->read_point(&msg);
            touch_api_unlock();
            switch (msg.event)
            {
            case TOUCH_EVENT_DOWN:
            case TOUCH_EVENT_UP:
                emouse_id = rt_tick_get();
#ifdef REMOTE_TOUCH_OUTPUT_DEVICE
                rt_device_write(p_remote_touch, 0, &msg, sizeof(msg));
                LOG_D("rmt touch out[%d, %d %d]", msg.event, msg.x, msg.y);
#else
                touch_write_more(msg.event, msg.x, msg.y);
#endif /* REMOTE_TOUCH_OUTPUT_DEVICE */
                //post_down_event(msg.x, msg.y, emouse_id);
                break;
            case TOUCH_EVENT_MOVE:
                //post_motion_event(msg.x, msg.y, emouse_id);
                break;
            default:
                break;
            }
            rt_thread_delay(RT_TICK_PER_SECOND / BSP_TOUCH_SAMPLE_HZ); //Let system breath
        }
        while (err == RT_EOK);

    }
}
#endif /* REMOTE_TOUCH_INPUT_DEVICE */

static void tp_poweron_thread_entry(void *parameter)
{
    touch_api_lock();
    LOG_I("Power up");
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif  /* RT_USING_PM */

    BSP_TP_PowerUp();
    current_driver->ops->init();

#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif  /* RT_USING_PM */

    LOG_I("Power up done.");
    touch_api_unlock();
}

static void tp_poweroff_thread_entry(void *parameter)
{
    touch_api_lock();
    LOG_I("Power off");
#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif  /* RT_USING_PM */


    //In case of cutomer driver forgot to disable irq.
    rt_touch_irq_pin_enable(0);
    current_driver->ops->deinit();

    BSP_TP_PowerDown();




#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif  /* RT_USING_PM */

    LOG_I("Power off done.");
    touch_api_unlock();

}

static rt_err_t init(struct rt_device *dev)
{
    more_data_lock = rt_mutex_create("tplck", RT_IPC_FLAG_FIFO);
    RT_ASSERT(more_data_lock != NULL);
    api_lock = rt_sem_create("tp_ctrl", 1, RT_IPC_FLAG_FIFO);
    RT_ASSERT(api_lock != NULL);

    current_driver = RT_NULL;

#ifndef LCD_MISSING
    rt_err_t err;

    p_touch_thread = &touch_thread;
    err = rt_thread_init(p_touch_thread, "tpread", tp_read_thread_entry, NULL,
                         touch_thread_stack, sizeof(touch_thread_stack),
                         RT_THREAD_PRIORITY_HIGH + RT_THREAD_PRIORITY_HIGHER * 2, RT_THREAD_TICK_DEFAULT);

    if (RT_EOK != err)
    {
        p_touch_thread = RT_NULL;
        return RT_ENOMEM;
    }
#endif

    return RT_EOK;
}




/**
 * @brief open touch device
 * @param[in] dev touch device to open
 * @param[in] oflag Flags for opening
 * @retval RT_EOK if successful.
*/
static rt_err_t touch_open(struct rt_device *dev, rt_uint16_t oflag)
{
    LOG_I("Open");
    BSP_TP_PowerUp();

    if (p_touch_thread)
    {
        if (RT_THREAD_INIT == (p_touch_thread->stat & RT_THREAD_STAT_MASK))
        {
            rt_thread_startup(p_touch_thread);
        }
        else
        {
#ifdef BSP_TOUCH_IRQ_FROM_DATASVC
            rt_err_t err = subscribe_irq_pin_service();
            if (RT_EOK != err)
            {
                LOG_I("Subscribe pin service err %d", err);
                return err;
            }
#endif /* BSP_TOUCH_IRQ_FROM_DATASVC */
            touch_api_lock();
            if (current_driver) current_driver->ops->init();
            touch_api_unlock();
            rt_thread_resume(p_touch_thread);
        }
    }

    LOG_I("Opened.");
    return RT_EOK;

}

/**
 * @brief close touch device
 * @param[in] dev touch device to close
 * @retval RT_EOK if successful.
*/
static rt_err_t touch_close(struct rt_device *dev)
{
    LOG_I("Close");

    if (p_touch_thread) rt_thread_suspend(p_touch_thread);
    touch_api_lock();
    if (current_driver) current_driver->ops->deinit();

#ifdef BSP_TOUCH_IRQ_FROM_DATASVC
    unsubscribe_irq_pin_service();
#endif /* BSP_TOUCH_IRQ_FROM_DATASVC */

    BSP_TP_PowerDown();
    touch_api_unlock();
    LOG_I("Closed.");

    return RT_EOK;
}

/**
 * @brief Read data from touch device
 * @param[in] dev touch device to read
 * @param[in] pos position to read
 * @param[in,out] buffer read buffer
 * @param[in] size size to read.
 * @retval size read
*/
static rt_size_t touch_read(struct rt_device *dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_size_t    read_size = 0;
    touch_msg_t  p_touch_data = (touch_msg_t) buffer;

    rt_mutex_take(more_data_lock, RT_WAITING_FOREVER);

    if (REC_BUF_EMPTY())
    {
        p_touch_data->event   = last_rec.event;
        p_touch_data->x = last_rec.x;
        p_touch_data->y = last_rec.y;
    }
    else
    {
        int cur_pos;
        cur_pos = pos_rec_beg;

        p_touch_data->event   = pos_rec[cur_pos].event;
        p_touch_data->x = pos_rec[cur_pos].x;
        p_touch_data->y = pos_rec[cur_pos].y;

        if (enable_tp_buf_log)
        {
            LOG_I("touch buffer out[%d][%d, %d %d]\n", cur_pos, pos_rec[cur_pos].event, pos_rec[cur_pos].x, pos_rec[cur_pos].y);
        }

        pos_rec_beg = (pos_rec_beg + 1) & (MAX_TOUCH_REC - 1);
        read_size = 1;
    }

    rt_mutex_release(more_data_lock);

    return read_size;
}


static rt_err_t control(struct rt_device *dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    if (NULL == current_driver)  return RT_EEMPTY;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        //current_driver->ops->deinit();
        break;

    case RT_DEVICE_CTRL_RESUME:
        /* resume device */
        break;

//    case RTGRAPHIC_CTRL_POWERON:
//#ifdef TSC_USING_TMA525B
//        current_driver->ops->init();
//#endif /* TSC_USING_TMA525B */
//        break;

    case RTGRAPHIC_CTRL_POWERON:
    {
        /*Temporary thread with large stack for tp init only*/
        rt_thread_t init_task = rt_thread_create("tp_on",
                                tp_poweron_thread_entry,
                                NULL,
                                4096,
                                RT_THREAD_PRIORITY_HIGH,
                                RT_THREAD_TICK_DEFAULT);
        RT_ASSERT(init_task != NULL);
        rt_thread_startup(init_task);
    }
    break;

    case RTGRAPHIC_CTRL_POWEROFF:
    {
        /*Temporary thread with large stack for tp init only*/
        rt_thread_t init_task = rt_thread_create("tp_off",
                                tp_poweroff_thread_entry,
                                NULL,
                                4096,
                                RT_THREAD_PRIORITY_HIGH,
                                RT_THREAD_TICK_DEFAULT);
        RT_ASSERT(init_task != NULL);
        rt_thread_startup(init_task);

    }
    break;

    case RTGRAPHIC_CTRL_ROTATE_180:
    {
        if (!args) return RT_ERROR;

        rotate_rect = *(struct rt_device_rect_info *)args;

        rotate_180 = !rotate_180;

        LOG_I("Rotate_en=%d, [x0y0:%d,%d, x1y1:%d,%d]", rotate_180, rotate_rect.x, rotate_rect.y,
              rotate_rect.x + rotate_rect.width - 1,
              rotate_rect.y + rotate_rect.height - 1);
    }
    break;

    default:
        result = RT_ERROR;
        break;
    }

    return result;
}


#ifdef RT_USING_DEVICE_OPS
const rt_device_ops dev_ops =
{
    init,
    touch_open,
    touch_close,
    touch_read,
    NULL,
    control,
};
#endif

static int rt_touch_driver_init(void)
{
    rt_err_t err = RT_EOK;
    rt_device_t device;

    rt_list_init(&driver_list);



    device = &g_touch_device;

    device->type        = RT_Device_Class_Graphic;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    device->ops         = &dev_ops;
#else
    device->init        = init;
    device->open        = touch_open;
    device->close       = touch_close;
    device->read        = touch_read;
    device->write       = NULL;
    device->control     = control;
#endif
    device->user_data   = NULL;

    err = rt_device_register(device, "touch", RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_STANDALONE);

    if (err != RT_EOK)
    {
        LOG_E("touch_init failed, err=%d\n", err);
    }


    return 0;
}
INIT_BOARD_EXPORT(rt_touch_driver_init);

#ifdef FINSH_USING_MSH
#include <finsh.h>
static rt_err_t en_drvtp_log(int argc, char **argv)
{
    if (current_driver)
    {
        LOG_I("tp driver=%p,usr_data=%p", current_driver->ops, current_driver->user_data);
    }

    LOG_I("last record event=%d,x:%d,y:%d]", last_rec.event, last_rec.x, last_rec.y);

    enable_tp_buf_log = (strcmp(argv[1], "1") == 0) ? true : false;

    return RT_EOK;
}
MSH_CMD_EXPORT(en_drvtp_log, enable touch panel log);

static rt_err_t tp_ctrl(int argc, char **argv)
{

    if (argc < 2)
    {
        LOG_I("tp_ctrl [OPTION] [VALUE]\n\n");
        LOG_I("Example:\n");
        LOG_I("    tp_ctrl on      - Power on tp\n");
        LOG_I("    tp_ctrl off     - Power off tp\n");
        LOG_I("    tp_ctrl log     - Toggle log\n");
        LOG_I("    tp_ctrl click <x> <y>  - Click at specified coordinates <x> <y>\n");
        LOG_I("    tp_ctrl slide <x1> <y1> <x2> <y2>  - Slide from <x1> <y1> to <x2> <y2>\n");
        return RT_EOK;
    }

    if (!current_driver)
    {
        LOG_I("No touch panel found! \n");
    }


    if (strcmp(argv[1], "on") == 0)
    {
        control(&g_touch_device, RTGRAPHIC_CTRL_POWERON, NULL);
    }
    else if (strcmp(argv[1], "off") == 0)
    {
        control(&g_touch_device, RTGRAPHIC_CTRL_POWEROFF, NULL);
    }
    else if (strcmp(argv[1], "log") == 0)
    {
        if (current_driver)
        {
            LOG_I("tp driver=%p,usr_data=%p", current_driver->ops, current_driver->user_data);
        }

        LOG_I("last record event=%d,x:%d,y:%d]", last_rec.event, last_rec.x, last_rec.y);

        enable_tp_buf_log = !enable_tp_buf_log;
        LOG_I("enable_tp_buf_log=%d", enable_tp_buf_log);
    }
    else if (strcmp(argv[1], "click") == 0)
    {
        if (argc < 4)
        {
            LOG_I("Too less param, e.g: tp_ctrl click <x> <y>");
            return RT_EOK;
        }

        rt_uint16_t x, y;

        x = strtol(argv[2], 0, 10);
        y = strtol(argv[3], 0, 10);

        LOG_I("Clicked at %d,%d", x, y);
        touch_write_more(TOUCH_EVENT_DOWN, x, y);
        rt_thread_delay(rt_tick_from_millisecond(100));
        touch_write_more(TOUCH_EVENT_UP, x, y);
    }
    else if (strcmp(argv[1], "slide") == 0)
    {
        if (argc < 6)
        {
            LOG_I("Too less param, e.g: tp_ctrl slide <x1> <y1> <x2> <y2>");
            return RT_EOK;
        }

        rt_uint16_t x1, y1, x2, y2;

        x1 = strtol(argv[2], 0, 10);
        y1 = strtol(argv[3], 0, 10);
        x2 = strtol(argv[4], 0, 10);
        y2 = strtol(argv[5], 0, 10);

        LOG_I("Slide frome %d,%d -->  %d,%d", x1, y1, x2, y2);
        touch_write_more(TOUCH_EVENT_DOWN, x1, y1);
        rt_thread_delay(rt_tick_from_millisecond(100));
        touch_write_more(TOUCH_EVENT_DOWN, x2, y2);
        rt_thread_delay(rt_tick_from_millisecond(100));
        touch_write_more(TOUCH_EVENT_UP, x2, y2);
    }


    return RT_EOK;
}
MSH_CMD_EXPORT(tp_ctrl, Touch panel driver cmd);
#endif /*FINSH_USING_MSH*/

#endif /*BSP_USING_TOUCHD*/
