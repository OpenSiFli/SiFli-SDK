# UART设备
## 驱动配置
在{menuselection}`On-Chip Peripheral RTOS Drivers --> Enable UART`菜单中选择要使用的UART设备，配置是否要支持DMA。

下面的宏开关表示使能了UART1和UART2两个设备，并且都支持RX DMA

```c
#define BSP_USING_UART
#define BSP_USING_UART1
#define BSP_UART1_RX_USING_DMA
#define BSP_USING_UART2
#define BSP_UART2_RX_USING_DMA
```

```{note}
menuconfig选中了DMA只是表示将驱动配置为支持DMA，但驱动是否使用DMA还是要看`rt_device_open`是传入的flag，如果flag要求使用DMA，但驱动没有配置DMA，rt_device_open会返回失败。反之，如果驱动支持DMA，但open的时候没有指定使用DMA，驱动仍然以非DMA的模式工作
```

## 设备名称
- `uart<x>`，
其中x为设备编号，如`uart1`、`uart2`，与操作的外设编号对应


## 示例代码


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
## RT-Thread参考文档

- [UART设备][uart]