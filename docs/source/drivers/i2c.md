# I2C设备

I2C驱动包括两层：硬件访问层（HAL）和RT-Thread的适配层。<br>
硬件访问层提供用于访问 I2C 外设寄存器的基本 API。 有关详细信息，请参阅 I2C HAL 的 API 文档。<br>
适配层提供对 RT-Thread 驱动框架的支持。 用户可以使用 RT-Thread POSIX 驱动程序接口进行 I2C 编程。 请参阅 RT-Thread 驱动程序的 API 文档。<br>
主要功能包括： <br>
- 多实例支持
- I2C 工作在主模式
- 支持 I2C 工作在速度模式 100Kbps/400Kbps/1Mbps
- 支持地址模式10bit/7bit
- 支持基于中断的 RX 和 TX
- RX 和 TX 均支持 DMA

## 驱动配置

在{menuselection}`On-Chip Peripheral RTOS Drivers --> Enable I2C BUS`菜单中选择要使用的I2C总线设备，配置是否要支持DMA。

下面的宏开关使能了I2C1/I2C2/I2C3三个设备，并且都支持DMA
```c
#define RT_USING_I2C
#define BSP_USING_I2C1
#define BSP_USING_I2C2
#define BSP_USING_I2C3
#define BSP_I2C1_USING_DMA
#define BSP_I2C2_USING_DMA
#define BSP_I2C3_USING_DMA

```

```{note}
menuconfig选中了DMA只是表示将驱动配置为支持DMA，但驱动是否使用DMA还是要看`rt_device_open`是传入的flag，如果flag要求使用DMA，但驱动没有配置DMA，rt_device_open会返回失败。反之，如果驱动支持DMA，但open的时候没有指定使用DMA，驱动仍然以非DMA的模式工作
```

## 设备名称
- `i2c<x>`，
其中x为设备编号，如`i2c1`、`i2c2`，与操作的外设编号对应


## 示例代码


```c
// Find and open device
struct rt_i2c_bus_device *rt_i2c_bus_device_find(const char *bus_name);
  bus_name: i2c1 / i2c2 / i2c3
rt_err_t rt_i2c_open(struct rt_i2c_bus_device *dev, rt_uint16_t oflag);
  oflag: dma mode: RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_DMA_RX|RT_DEVICE_FLAG_DMA_TX
         int mode: RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX|RT_DEVICE_FLAG_INT_TX
		 normal mode: RT_DEVICE_FLAG_RDWR
// Configure I2C
rt_err_t i2c_bus_configure(struct rt_i2c_bus_device *bus, struct rt_i2c_configuration *configuration);
  configuration: use struct rt_i2c_configuration as following:
	struct rt_i2c_configuration
	{
		rt_uint16_t mode;
		rt_uint16_t addr;
		rt_uint32_t timeout;
		rt_uint32_t max_hz;
	};

// RX/TX
rt_size_t rt_i2c_transfer(struct rt_i2c_bus_device *bus,
                          struct rt_i2c_msg         msgs[],
                          rt_uint32_t               num)
  msgs: use struct rt_i2c_msg  as following:
	struct rt_i2c_msg
	{
		rt_uint16_t addr;
		rt_uint16_t mem_addr;
		rt_uint16_t mem_addr_size;
		rt_uint16_t flags;
		rt_uint16_t len;
		rt_uint8_t  *buf;
	};

// Interrupt callback, try not issue read in interrupt context.
__weak void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
__weak void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);


// example
void i2c_trans_test()
{
    struct rt_i2c_bus_device *i2c_bus = RT_NULL;
	struct rt_i2c_msg msgs;
	struct rt_i2c_configuration cfg = {0};
    rt_uint8_t *rd_buff;
	rt_err_t ret;
	
	
    i2c_bus = rt_i2c_bus_device_find("i2c1");
	if (RT_NULL == i2c_bus)
    {
        return;
    }
	
    ret = rt_device_open(&(i2c_bus->parent), RT_DEVICE_FLAG_RDWR);
    if (RT_EOK != ret)
    {
        return;
    }
	
    cfg.timeout = 5000;
    cfg.max_hz = 400000;
	//cfg.mode |= RT_I2C_ADDR_10BIT;
	//cfg.addr = 0xe;

    if (rt_i2c_configure(i2c_bus, &cfg) != HAL_OK)
    {
		rt_device_close(&(i2c_bus->parent));
		return;
    }
	
	rd_buff = rt_malloc(100);
    if (RT_NULL == rd_buff)
    {
        rt_device_close(&(i2c_bus->parent));
        return;
    }

    for (int m = 0; m < rw_len; m++)
    {
        rd_buff[m] = m;
    }
		
    msgs.addr = 0xe;
    msgs.flags = RT_I2C_WR;
    msgs.buf = rd_buff;
    msgs.len = 100;

    rt_i2c_transfer(i2c_bus, &msgs, 1);

    //msgs.addr = 0xe;
    msgs.flags = RT_I2C_RD;
    //msgs.buf = rd_buff;
    //msgs.len = 100;

    rt_i2c_transfer(i2c_bus, &msgs, 1);

    rt_device_close(&(i2c_bus->parent));
}


```

```{note}
DMA 支持是通过 menuconfig 工具配置的，要使用 DMA，开发者需要在打开设备时设置相应的标志。
```


[i2c]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/i2c/i2c
## RT-Thread参考文档

- [I2C设备][i2c]
