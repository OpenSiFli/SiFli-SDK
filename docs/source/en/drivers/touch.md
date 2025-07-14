# Touch Screen
## Introduction

For touch controller drivers, we have implemented a unified interface called "touch" rt_device that provides a simple framework for registering different touch controller drivers and automatically selecting the appropriate driver.
This chapter mainly introduces the internal framework functionality of Touch device and how to register a new touch controller to this framework.

Touch device implementation consists of 2 parts:
- rt_device device (drv_touch.c)
    - Registers a device named "touch" that provides interrupt callback registration and touch data reading interfaces
    - Creates a task for slow communication with touch devices (such as initialization, reading touch point data, etc.)
    - Performs caching, filtering (filtering duplicate points), and cache overflow packet dropping for touch data points read
- Specific touch device driver implementation
    - Registers a new driver with touch_device and provides the following implementations:
		- probe   - Identify supported devices
		- init    - Initialization after device identification
		- deinit  - Deinitialization after device identification
		- read_point   - Read a valid data point (**Note: Return RT_EOK if there are still unread data points, otherwise return other values**)
		- A semaphore   - Used to block rt_device layer threads
	- Also needs to implement internally:
	   - Touch interrupt detection
	   - Communication interface initialization

![Figure 1: Touch driver software architecture](../../assets/touch_device_arch.png)
<br>
<br>
<br>
<br>
<br>
<br>

## Process for Adding New Touch Controller Code
## 1. Select the corresponding board project under example\\rt_driver
- This project contains a thread _touch_read_task_ that reads touch data and prints it

## 2. Add new driver to compilation project
- Add new touch controller code to directory _customer\\peripherals_
    - You can copy code from other existing drivers, then change the name, Slave_Address, read process, etc. to your own
    ```{note} 
    Note to modify the depend macro in the Kconfig file under the copied code directory
    ```
- In _customer\\peripherals\\Kconfig_, add a hidden option for the newly added driver, for example:
    ```c
    config TSC_USING_TMA525B
        bool
        default n
    ```
- Add the previously added hidden touch controller switch to the board-level configuration screen module switch:
    ```c
    config LCD_USING_ED_LB55DSI13902_DSI_LB555
        bool "1.39 round 454RGB*454 DSI LCD(ED-LB55DSI13902)"
        select TSC_USING_TMA525B         <--------  Added touch controller switch
        select LCD_USING_RM69330
        select BSP_LCDC_USING_DSI
        if LCD_USING_ED_LB55DSI13902_DSI_LB555
            config LCD_RM69330_VSYNC_ENABLE
                bool "Enable LCD VSYNC (TE signal)"
                def_bool n
        endif
    ```
- If using scons compilation, need to enter the project menuconfig selection menu, then select the newly added screen module, finally generating _.config_ and _rtconfig.h_
- If using Keil compilation, you can also directly add source code (but still recommend the same method as scons compilation, so it will be automatically included when regenerating Keil project next time)

## 3. Check pinmux for pins used by new touch controller and reset pin
- In the SDK's `drv_io.c`, functions _BSP_TP_PowerUp&BSP_TP_PowerDown_ perform power up/down and reset operations for touch controllers
<br>
<br>


## Touch Controller Debugging Suggestions
- First check if power supply and reset pin status are normal
- Then check if communication interface waveforms are normal, such as whether I2C interface has ack
<br>


## TMA525B Touch Device Driver Implementation Example Code

TMA525B uses the falling edge of TOUCH_IRQ_PIN as trigger condition, I2C as communication interface at 400KHz speed, I2C read/write timeout of 5ms
This implementation releases semaphore in interrupt to make touch_device layer call its own read_point to get touch data
Also disables interrupt enable in interrupt, then re-enables interrupt enable after entering read_point, preventing too many interrupts from repeatedly releasing too many semaphores

```c

static struct rt_i2c_bus_device *ft_bus = NULL;
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


void tma525b_irq_handler(void *arg)
{
    rt_err_t ret = RT_ERROR;

    rt_pin_irq_enable(TOUCH_IRQ_PIN, 0);

    ret = rt_sem_release(tma525b_driver.isr_sem);
    RT_ASSERT(RT_EOK == ret);
}


static rt_err_t read_point(touch_msg_t p_msg)
{
    rt_err_t ret = RT_ERROR;
	rt_uint8_t  touch_report[16];  

    rt_pin_irq_enable(TOUCH_IRQ_PIN, 1);


    //read touch report
    if(RT_EOK == i2c_read((rt_uint8_t *)touch_report, 16))
    {
		if(touch_report[1] == 1)
		{
			p_msg->event = TOUCH_EVENT_DOWN;
		}
		else
		{
			p_msg->event = TOUCH_EVENT_UP;
		}

		p_msg->x     = touch_report[2];
		p_msg->y     = touch_report[3];

		if(touch_report[4] > 1)
			return RT_EOK;       //More pending touch data
		else	
			return RT_EEMPTY;    //No more touch data to be read
    }

	p_msg->event = TOUCH_EVENT_NONE; //Read point fail
    return RT_ERROR;
}

static rt_err_t init(void)
{
    rt_pin_mode(TOUCH_IRQ_PIN, PIN_MODE_INPUT);
    rt_pin_attach_irq(TOUCH_IRQ_PIN, PIN_IRQ_MODE_FALLING, tma525b_irq_handler, (void *)(rt_uint32_t)TOUCH_IRQ_PIN);
    rt_pin_irq_enable(TOUCH_IRQ_PIN, 1); //Must enable before read I2C

    return RT_EOK;

}

static rt_err_t deinit(void)
{
    rt_pin_detach_irq(TOUCH_IRQ_PIN);
    return RT_EOK;
}

static rt_bool_t probe(void)
{
    rt_err_t err;

    ft_bus = (struct rt_i2c_bus_device *)rt_device_find(TOUCH_DEVICE_NAME);
    if (RT_Device_Class_I2CBUS != ft_bus->parent.type)
    {
        ft_bus = NULL;
    }
    if (ft_bus)
    {
        rt_device_open((rt_device_t)ft_bus, RT_DEVICE_FLAG_RDWR);
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
            .timeout = 5,
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
```
<br>
<br>

## Touch Device Upper Layer Usage Example Code
Example registers interrupt callback to release semaphore, then semaphore drives reading touch data, then prints touch points

```c
static struct rt_semaphore tp_sema;

static rt_err_t tp_rx_indicate(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&tp_sema);

    return RT_EOK;
}

static void touch_read_task(void *parameter)
{
    rt_sem_init(&tp_sema, "tpsem", 0, RT_IPC_FLAG_FIFO);

    /*Open touch device*/
    rt_device_t touch_device = NULL;
    touch_device = rt_device_find("touch");
    if (touch_device)
    {
        if (RT_EOK == rt_device_open(touch_device, RT_DEVICE_FLAG_RDONLY))
        {
            touch_device->rx_indicate = tp_rx_indicate;

            while (1)
            {
                rt_err_t err;
                struct touch_message touch_data;

                err = rt_sem_take(&tp_sema, rt_tick_from_millisecond(500));
                if (RT_EOK == err)
                {
                    rt_device_read(touch_device, 0, &touch_data, 1);
                    rt_kprintf("read data %d, [%d,%d]\r\n", touch_data.event, touch_data.x, touch_data.y);
                }
            }
        }
        else
        {
            rt_kprintf("Touch open error!\n");
            touch_device = NULL;
        }
    }
    else
    {
        rt_kprintf("No touch device found!\n");
    }

}

```