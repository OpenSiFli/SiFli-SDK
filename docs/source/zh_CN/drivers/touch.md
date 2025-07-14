# Touch Screen
## 简介

触控驱动我们实现了一个名为"touch"的rt_device统一对上的接口，其内部有一个简单的框架用于注册不同的触控驱动，并自动选择适配的驱动。
本章节主要介绍Touch device的内部框架功能以及如何注册一个新触控到该框架。

Touch device的实现分2部分
- rt_device设备(drv_touch.c )
    - 注册了一个名为"touch"的device,对外提供中断回调注册和读触控数据接口
    - 创建了一个task用于和触控设备的慢速通信（如初始化、读取触控点数据等）
    - 对读到的触控数据点,做缓存、过滤(过滤重复点)、缓存溢出丢包处理
- 具体触控设备的驱动实现
    - 向touch_device注册一个新驱动并提供以下实现:
		- probe   识别支持设备
		- init    识别设备后的初始化
		- deinit  识别设备后的去初始化
		- read_point   读一个有效数据点(**注意：还有未读走的数据点返回RT_EOK, 否则返回其他值**)
		- 一个信号量   用于阻塞rt_device层的线程
	- 同时内部需要实现：
	   - 触控中断的检测
	   - 通信接口的初始化

![图 1: 触控驱动软件结构](../../assets/touch_device_arch.png)
<br>
<br>
<br>
<br>
<br>
<br>

## 增加一个新触控代码的流程
## 1. 选择example\\rt_driver下对应板子的工程
- 这个工程里面有一个读取触控数据并打印的线程 _touch_read_task_

## 2. 将新驱动添加到编译工程里面
- 添加新触控代码到目录_customer\\peripherals_内
    - 可以从其他已有的驱动复制一份代码，然后将名字、Slave_Address、读取流程等改成自己的
    ```{note} 
    注意修改复制代码目录下的Kconfig文件的depend宏
    ```
- 在_customer\\peripherals\\Kconfig_内，为新加的驱动添加一个隐藏开选项，比如：
    ```c
    config TSC_USING_TMA525B
        bool
        default n
    ```
- 在板级的配置的屏幕模组的开关中添加前面添加的隐藏触控开关：
    ```c
    config LCD_USING_ED_LB55DSI13902_DSI_LB555
        bool "1.39 round 454RGB*454 DSI LCD(ED-LB55DSI13902)"
        select TSC_USING_TMA525B         <--------  添加的触控开关
        select LCD_USING_RM69330
        select BSP_LCDC_USING_DSI
        if LCD_USING_ED_LB55DSI13902_DSI_LB555
            config LCD_RM69330_VSYNC_ENABLE
                bool "Enable LCD VSYNC (TE signal)"
                def_bool n
        endif
    ```
- 若用scons 编译，则需要进入工程的menuconfig选择菜单，然后选择前面新增的屏幕模组，最终生成_.config_和_rtconfig.h_
- 若用Keil编译，也可以直接添加源代码进入（但还是建议和scons编译添加方法一样，这样下次重新生成Keil工程后会自动加入）

## 3. 检查新增的触控用到的pin，以及reset pin 的pinmux是否正确
- SDK的`drv_io.c`内 函数_BSP_TP_PowerUp&BSP_TP_PowerDown_，对触控做了上下电和reset的操作
<br>
<br>


## 触控调试的建议
- 先检查供电、reset脚的状态是否正常
- 然后检查通信接口波形是否正常，比如I2C接口是否有ack
<br>


## tma525b触控设备的驱动实现示例代码

tma525b通过TOUCH_IRQ_PIN的下降沿为触发条件，I2C为通信接口速度400KHz， I2C读写超时时间5ms
该实现中在中断中释放信号量使touch_device层调用自己的read_point去获取触控数据
同时在中断中关闭中断使能，在进read_point后再打开中断使能，防止中断太多反复release太多信号量

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

## Touch device上层使用方法的示例代码
示例通过注册中断回调释放信号量，然后信号量再去驱动读触控数据，然后打印触控点

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

