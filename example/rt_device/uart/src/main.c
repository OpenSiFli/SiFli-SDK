#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "board.h"

#define DBG_TAG "uart2"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* uart example for RT-Thread based platform -----------------------------------------------*/


#define UART_DEMO_NAME "uart2"
#define ONE_DATA_MAXLEN 256

static rt_device_t g_uart_device = RT_NULL;
static struct rt_semaphore rx_sem;
static uint8_t data[ONE_DATA_MAXLEN] = { 0 };
/* receiving thread */
static void serial_rx_thread_entry(void *parameter)
{
    uint16_t count = 0;
    uint8_t cnt = 0;

    while (1)
    {
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        while (1)
        {
            cnt = rt_device_read(g_uart_device, -1, &data[count], ONE_DATA_MAXLEN);
            count += cnt;
            rt_kprintf("uart_rec: cnt = %d,count = %d\n", cnt, count);
            if (0 == cnt) break;
        }
        rt_kprintf("rev:");
        for (uint16_t i = 0; i < count; i++)
        {
            rt_kprintf("%c", data[i]);
        }
        count = 0;
        rt_kprintf("\n");
    }
}

/* uart callback function */
static rt_err_t uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    /* release the semaphore when recieve the data */
    if (size > 0)
    {
        rt_sem_release(&rx_sem);
    }
    return RT_EOK;
}
static void uart_send_data(uint8_t *p_data, uint16_t length)
{
    uint16_t write_len = 0;

    if (g_uart_device == RT_NULL)
    {
        return;
    }
    write_len = rt_device_write(g_uart_device, 0, p_data, length);
    rt_kprintf("send:");
    for (uint16_t i = 0; i <= write_len; i++)
    {
        rt_kprintf("%c", *(p_data + i));
    }
}

int uart2_init(void)
{
    rt_err_t ret = RT_EOK;
    /* 1, pinmux set to uart mode */
#if defined(BSP_USING_BOARD_SF32LB52_LCD_N16R8)
    HAL_PIN_Set(PAD_PA20, USART2_RXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA27, USART2_TXD, PIN_PULLUP, 1);
#elif defined (BSP_USING_BOARD_SF32LB58_LCD_N16R64N4)
    HAL_PIN_Set(PAD_PA29, USART2_RXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA28, USART2_TXD, PIN_PULLUP, 1);
#endif

    /* 2, find  and config uart2 device */
    g_uart_device = rt_device_find(UART_DEMO_NAME);
    if (!g_uart_device)
    {
        LOG_E("find %s failed!\n", UART_DEMO_NAME);
        return RT_ERROR;
    }

    /* config uart2 baud_rate */
    {
        rt_err_t err;
        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
        config.baud_rate = 1000000;
        err = rt_device_control(g_uart_device, RT_DEVICE_CTRL_CONFIG, &config);
        LOG_D("uart device config %d", err);
    }

    /* Initialize the semaphore */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);

    //using DMA mode first
    rt_err_t open_result = rt_device_open(g_uart_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX);
    //using interring mode when DMA mode not supported
    if (open_result == -RT_EIO)
    {
        rt_device_open(g_uart_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }
    /* set the callback function of recieving */
    rt_device_set_rx_indicate(g_uart_device, uart_rx_ind);

    /* creat the thread of g_uart_device */
    rt_thread_t thread = rt_thread_create("g_uart_device", serial_rx_thread_entry, RT_NULL,  3 * 1024, 12, 10);

    /* start the thread of g_uart_device */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        ret = RT_ERROR;
    }
    return ret;
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    uint8_t tx_data[ONE_DATA_MAXLEN] = {'u', 'a', 'r', 't', '2', ' ', 'd', 'e', 'm', 'o', '\n'};

    rt_kprintf("Start uart demo!\n");
    uart2_init();
    uart_send_data(tx_data, 12);
    rt_kprintf("uart demo end!\n");
    while (1)
    {
        rt_thread_mdelay(5000);
        //rt_kprintf("__main loop__\r\n");
    }
    return RT_EOK;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

