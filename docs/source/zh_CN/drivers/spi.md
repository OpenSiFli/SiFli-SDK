# SPI设备

SPI驱动包括两层：硬件访问层（HAL）和RT-Thread的适配层。<br>
硬件访问层提供用于访问 SPI 外设寄存器的基本 API。 有关详细信息，请参阅 SPI HAL 的 API 文档。<br>
适配层提供对 RT-Thread 驱动框架的支持。 用户可以使用 RT-Thread POSIX 驱动程序接口进行 SPI 编程。 请参阅 RT-Thread 驱动程序的 API 文档。<br>
主要功能包括： <br>
- 多实例支持
- 支持SPI工作在主模式或从模式
- 支持SPI速度高达24Mbps
- 支持 3 线或 4 线 SPI 工作
- 支持基于中断的 RX 和 TX
- RX 和 TX 均支持 DMA

## 驱动配置

在{menuselection}`On-Chip Peripheral RTOS Drivers -> Enable SPI BUS`菜单中选择要使用的SPI总线设备，配置是否要支持DMA。

下面的宏开关表示使能了SPI1和SPI2两个设备，并且收发都支持DMA：
```c
#define RT_USING_SPI
#define BSP_USING_SPI
#define BSP_USING_SPI1
#define BSP_USING_SPI2
#define BSP_SPI1_TX_USING_DMA
#define BSP_SPI1_RX_USING_DMA
#define BSP_SPI2_TX_USING_DMA
#define BSP_SPI2_RX_USING_DMA

```

```{note}
menuconfig选中了DMA只是表示将驱动配置为支持DMA，但驱动是否使用DMA还是要看`rt_device_open`是传入的flag，如果flag要求使用DMA，但驱动没有配置DMA，rt_device_open会返回失败。反之，如果驱动支持DMA，但open的时候没有指定使用DMA，驱动仍然以非DMA的模式工作
```

## 设备名称
- `spi<x>`，
其中x为设备编号，如`spi1`、`spi2`，与操作的外设编号对应

## 示例代码

```c
// Find and open device
rt_device_t rt_device_find(const char *name);
  name: spi1 / spi2
rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name);
  bus_name: spi1 / spi2
rt_err_t rt_device_open(rt_device_t dev, rt_uint16_t oflag);
  oflag: dma mode: RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_DMA_RX|RT_DEVICE_FLAG_DMA_TX
         int mode: RT_DEVICE_FLAG_RDWR|RT_DEVICE_FLAG_INT_RX|RT_DEVICE_FLAG_INT_TX
		 normal mode: RT_DEVICE_FLAG_RDWR
		 
// Configure SPI
rt_err_t rt_spi_configure(struct rt_spi_device *device, struct rt_spi_configuration *cfg)
  cfg: use struct rt_spi_configuration as following:
	struct rt_spi_configuration
	{
		rt_uint8_t mode;
		rt_uint8_t data_width;
		rt_uint16_t frameMode;
		rt_uint32_t max_hz;
	};

// RX/TX
rt_size_t rt_spi_transfer(struct rt_spi_device *device,
                          const void           *send_buf,
                          void                 *recv_buf,
                          rt_size_t             length);

// Interrupt callback, try not issue read in interrupt context.
__weak void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
__weak void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
__weak void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
__weak void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)

// example
void spi_trans_test()
{
    rt_device_t spi_bus = RT_NULL;
    struct rt_spi_device *spi_dev = RT_NULL;
	struct rt_spi_configuration cfg;
    rt_err_t err_code;
	rt_uint8_t *rx_buff = RT_NULL;
    rt_uint8_t *tx_buff = RT_NULL;
	
	spi_bus = rt_device_find("spi1");
    if (RT_NULL == spi_bus)
    {
        return;
    }

    spi_dev = (struct rt_spi_device *)rt_device_find("spi1_dev");
    if (RT_NULL == spi_dev)
    {
        err_code = rt_hw_spi_device_attach("spi1", "spi1_dev");
        if (RT_EOK != err_code)
        {
            return;
        }
		spi_dev = (struct rt_spi_device *) rt_device_find("spi1_dev");
    }
    
    if (RT_NULL == spi_dev)
    {
        return;
    }
	
    err_code = rt_device_open(&(spi_dev->parent), RT_DEVICE_FLAG_RDWR);
    if (RT_EOK != err_code)
    {
        return;
    }
	
	cfg.data_width = 8;
    cfg.max_hz = 4000000;
    //frame_mode,  //b0:SPO b1:SPH b2:moto(spi) b3:ti(ssi) b4:microwire
    //cfg.mode = RT_SPI_MODE_1 | RT_SPI_MSB;
    cfg.mode = RT_SPI_MSB | RT_SPI_MASTER | RT_SPI_MODE_1;
    cfg.frameMode = RT_SPI_MOTO;

    err_code = rt_spi_configure(spi_dev, &cfg);
    uassert_int_equal(RT_EOK, err_code);

    err_code = rt_spi_take_bus(spi_dev);
    uassert_int_equal(RT_EOK, err_code);

    err_code = rt_spi_take(spi_dev);
    uassert_int_equal(RT_EOK, err_code);

    rx_buff = rt_malloc(rw_len + 2);
    tx_buff = rt_malloc(rw_len + 2);
    uassert_true((RT_NULL != tx_buff) && (RT_NULL != rx_buff));

	if ((RT_NULL != tx_buff) && (RT_NULL != rx_buff))
	{
		for (int m = 0; m < rw_len; m++)
		{
			tx_buff[m] = m;
		}

		memset(rx_buff, 0x5a, rw_len + 2);
		rt_spi_transfer(spi_dev, tx_buff, rx_buff, rw_len / (data_size / 8));
	}

    if (RT_NULL != tx_buff)
    {
        rt_free(tx_buff);
    }

    if (RT_NULL != rx_buff)
    {
        rt_free(rx_buff);
    }

    err_code = rt_spi_release(spi_dev);
    uassert_int_equal(RT_EOK, err_code);

    err_code = rt_spi_release_bus(spi_dev);
    uassert_int_equal(RT_EOK, err_code);

    err_code = rt_device_close(&(spi_dev->parent));
    uassert_true_ret(RT_EOK == err_code);
	
}


```

```{note}
DMA 支持是通过 menuconfig 工具配置的，要使用 DMA，开发者需要在打开设备时设置相应的标志。
```


[spi]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/spi/spi

## RT-Thread参考文档

- [SPI设备][spi]

