#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "dfs_file.h"
#include "drv_flash.h"
#include "drivers/usb_device.h"
#include "bf0_hal_pcd.h"


#define vcom_RX_LEN 1024
rt_device_t usb_vcom = NULL;
static struct rt_semaphore rx_sem;
char cdc_rx_buf[vcom_RX_LEN] = {0};
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);
    return RT_EOK;
}
static void serial_thread_entry(void *parameter)
{
    char ch;
    int cnt = 0;
    while (1)
    {
        while (rt_device_read(usb_vcom, -1, &ch, 1) != 1)
        {
            rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        }
        cdc_rx_buf[cnt++] = ch;
        if (ch == '\n' || ch == '\0' || cnt == (vcom_RX_LEN - 1))
        {
            rt_kprintf("%s cdc_rx_buf=%s\n", __func__, cdc_rx_buf);
            cnt = 0;
            memset(cdc_rx_buf, 0, cnt);
        }
    }
}

int usb_cdc_init(void)
{
    usb_vcom = rt_device_find("vcom");
    RT_ASSERT(usb_vcom);
    rt_device_init(usb_vcom);
    rt_device_open(usb_vcom, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX);

    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    rt_device_set_rx_indicate(usb_vcom, uart_input);
    rt_thread_t thread = rt_thread_create("usb_vcom", serial_thread_entry, RT_NULL, 1024, 25, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    return 0;
}
INIT_APP_EXPORT(usb_cdc_init);

void vcom_write(int avgc, char **argv)
{
    rt_size_t len =  rt_device_write(usb_vcom, 0, argv[1], strlen(argv[1]));
    if (len != strlen(argv[1])) rt_kprintf("write fail!\n");
    else rt_kprintf("write succ!\n");
}
MSH_CMD_EXPORT(vcom_write, vcom write);

int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("Use help to check USB cdc vcom command!\n");
    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);    // Let system breath.
    }
    return 0;
}