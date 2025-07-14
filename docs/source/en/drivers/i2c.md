
# I2C Device

The I2C driver consists of two layers: the Hardware Access Layer (HAL) and the RT-Thread adapter layer.  
The hardware access layer provides basic APIs for accessing I2C peripheral registers. For more details, please refer to the I2C HAL API documentation.  
The adapter layer provides support for the RT-Thread driver framework. Users can use the RT-Thread POSIX driver interface for I2C programming. Please refer to the RT-Thread driver API documentation.  
Main features include:
- Multi-instance support
- I2C operates in master mode
- Supports I2C operation at speeds of 100Kbps/400Kbps/1Mbps
- Supports both 10-bit/7-bit addressing modes
- Supports interrupt-based RX and TX
- Both RX and TX support DMA

## Driver Configuration

In the `{menuselection}` `On-Chip Peripheral RTOS Drivers --> Enable I2C BUS` menu, select the I2C bus device to be used and configure whether DMA support is required.

The following macro switches enable I2C1/I2C2/I2C3 devices, and all support DMA:
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
Selecting DMA in menuconfig only configures the driver to support DMA, but whether DMA is used depends on the flag passed to `rt_device_open`. If the flag requires DMA but the driver is not configured for DMA, `rt_device_open` will return a failure. Conversely, if the driver supports DMA but it is not specified at open time, the driver will still operate in non-DMA mode.
```

## Device Names
- `i2c<x>`,
Where `x` is the device number, such as `i2c1`, `i2c2`, corresponding to the peripheral number being operated on.


## Example Code

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
  configuration: use struct rt_i2c_configuration as follows:
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
  msgs: use struct rt_i2c_msg as follows:
	struct rt_i2c_msg
	{
		rt_uint16_t addr;
		rt_uint16_t mem_addr;
		rt_uint16_t mem_addr_size;
		rt_uint16_t flags;
		rt_uint16_t len;
		rt_uint8_t  *buf;
	};

// Interrupt callback, try not to issue read in interrupt context.
__weak void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
__weak void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)


```
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
DMA support is configured via the menuconfig tool. To use DMA, developers need to set the corresponding flag when opening the device.
```


[i2c]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/i2c/i2c
## RT-Thread Reference Documentation

- [I2C Device][i2c]
