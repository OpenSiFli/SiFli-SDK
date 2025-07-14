# SPI Device

SPI driver includes two layers: hardware access layer (HAL) and RT-Thread adaptation layer.<br>
The hardware access layer provides basic APIs for accessing SPI peripheral registers. For details, please refer to the API documentation of SPI HAL.<br>
The adaptation layer provides support for RT-Thread driver framework. Users can use RT-Thread POSIX driver interface for SPI programming. Please refer to the API documentation of RT-Thread driver.<br>
Main features include:<br>
- Multi-instance support
- Support SPI working in master or slave mode
- Support SPI speed up to 24Mbps
- Support 3-wire or 4-wire SPI operation
- Support interrupt-based RX and TX
- DMA support for both RX and TX

## Driver Configuration

Select the SPI bus devices to use in the {menuselection}`On-Chip Peripheral RTOS Drivers -> Enable SPI BUS` menu, and configure whether to support DMA.

The following macro switches indicate that SPI1 and SPI2 devices are enabled, and both support DMA for transmit and receive:
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
Selecting DMA in menuconfig only means configuring the driver to support DMA, but whether the driver uses DMA depends on the flag passed to `rt_device_open`. If the flag requires DMA but the driver is not configured for DMA, rt_device_open will return failure. Conversely, if the driver supports DMA but DMA is not specified when opening, the driver will still work in non-DMA mode.
```

## Device Names
- `spi<x>`,
where x is the device number, such as `spi1`, `spi2`, corresponding to the peripheral numbers being operated

## Example Code

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
DMA support is configured through the menuconfig tool. To use DMA, developers need to set the corresponding flags when opening the device.
```

[spi]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/spi/spi

## RT-Thread Reference Documentation

- [SPI Device][spi]
