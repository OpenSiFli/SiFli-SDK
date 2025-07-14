# UART Device
## Driver Configuration
Select the UART devices to use in the {menuselection}`On-Chip Peripheral RTOS Drivers --> Enable UART` menu, and configure whether to support DMA.

The following macro switches indicate that UART1 and UART2 devices are enabled, and both support RX DMA:

```c
#define BSP_USING_UART
#define BSP_USING_UART1
#define BSP_UART1_RX_USING_DMA
#define BSP_USING_UART2
#define BSP_UART2_RX_USING_DMA
```

```{note}
Selecting DMA in menuconfig only means configuring the driver to support DMA, but whether the driver uses DMA depends on the flag passed to `rt_device_open`. If the flag requires DMA but the driver is not configured for DMA, rt_device_open will return failure. Conversely, if the driver supports DMA but DMA is not specified when opening, the driver will still work in non-DMA mode.
```

## Device Names
- `uart<x>`,
where x is the device number, such as `uart1`, `uart2`, corresponding to the peripheral numbers being operated

## Example Code

```c
// Find and open device
rt_device_t uart_dev = rt_device_find("uart1");
// RX use DMA
rt_err_t err = rt_device_open(uart_dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_DMA_RX);

// Configure UART
struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
config.baudrate=115200;
rt_device_control(uart_dev, RT_DEVICE_CTRL_CONFIG, &config);

// TX
uint8_t data=[1,2,3,4,5,6,7,8];
rt_device_write(uart_dev, 
	-1, 			// Start offset, for UART, this is ignored.
	data, 			
	sizeof(data));
        
// RX
#define BLOCK_SIZE 256
uint8_t g_rx_data=[BLOCK_SIZE];
static rt_sem_t rx_sem;

// Interrupt callback, try not issue read in interrupt context.
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(rx_sem);
    return RT_EOK;
}

...

// Create semphore to communicate with IRQ context
 rt_sem_create("uart_sem", 1, RT_IPC_FLAG_FIFO);
// Set RX indication functions
rt_device_set_rx_indicate(uart_dev, uart_input);
// Wait fo RX interrupt.
rt_sem_take(rx_sem,  1000);
// Read up to BLOCK_SIZE, will return len actually read.
int len=rt_device_read(uart_dev, 
	-1, 
	g_rx_data, 
	BLOCK_SIZE);

```

[uart]: https://www.rt-thread.org/document/site/#/rt-thread-version/rt-thread-standard/programming-manual/device/uart/uart_v1/uart

## RT-Thread Reference Documentation

- [UART Device][uart]