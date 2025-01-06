#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "rtdevice.h"
#include "drv_flash.h"
#include "drivers/usb_device.h"
#include "bf0_hal_pcd.h"

#define USB_FINSH_TEST               0

#if defined(RT_USB_DEVICE_CDC)
enum
{
    USB_OUTPUT,
    USB_INPUT,
};
static      uint8_t                              usb_state = USB_OUTPUT;                    /*USB status*/
extern void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd);
extern void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd);
static void usb_set_state(uint8_t state)
{
    usb_state = state;
}

static uint8_t usb_get_state(void)
{
    return usb_state;
}

static void pin_irq_callback(void *args)
{
#if -1 < INSERT_DETE_USB_PIN
    uint8_t read_pin = 0;
    read_pin = rt_pin_read(INSERT_DETE_USB_PIN);
    rt_kprintf("read_pin=%d\n", read_pin);
    for (int i = 0; i < 1000; i++) {;} //Shake off
    if (read_pin == rt_pin_read(INSERT_DETE_USB_PIN))
    {
        rt_kprintf("pin_irq_callback read_pin=%d\n", read_pin);
        if (read_pin == 1)
        {
            //High level USB interface insertion
            if (USB_OUTPUT == usb_get_state()) //Determine the current status of USB
            {
                rt_kprintf("%s  %d\n", __func__, __LINE__);
                usb_set_state(USB_INPUT);//Set USB status
                HAL_PCD_MspInit(NULL);
            }
        }
        else
        {
            //Low level USB unplugged
            if (USB_INPUT == usb_get_state())
            {
                HAL_PCD_DisconnectCallback(NULL);//Send a unplugged interrupt to USB
                usb_set_state(USB_OUTPUT);//Set USB status
                HAL_PCD_MspDeInit(NULL);
                rt_kprintf("%s  %d\n", __func__, __LINE__);
            }
        }
    }
#endif
}

static void usb_vbus_pin_irq(void)
{
#if -1 < INSERT_DETE_USB_PIN
    //Set pin input mode
    rt_pin_mode(INSERT_DETE_USB_PIN, PIN_MODE_INPUT);
    //Enable rasing edge interrupt mode
    rt_pin_attach_irq(INSERT_DETE_USB_PIN, PIN_IRQ_MODE_RISING_FALLING, pin_irq_callback, RT_NULL);
    //Enable interrupt
    rt_pin_irq_enable(INSERT_DETE_USB_PIN, 1);
#endif
    HAL_PCD_MspDeInit(NULL);
}


#define vcom_RX_LEN 64
rt_device_t usb_vcom = NULL;
static struct rt_semaphore rx_sem, tx_sem;
char cdc_rx_buf[vcom_RX_LEN] = {0};
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);
    return RT_EOK;
}

static rt_err_t uart_output(rt_device_t dev, void *buffer)
{
    int tx_data_flow = (int)buffer;
    if (tx_data_flow == DATA_FLOW_END)
        rt_sem_release(&tx_sem);
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
        if (ch == '\n' || ch == '\0' || cnt == vcom_RX_LEN)
        {
            rt_kprintf("cnt=%d cdc_rx_buf=%s\n", cnt, cdc_rx_buf);
            memset(cdc_rx_buf, 0, cnt);
            cnt = 0;
        }
    }
}

/*******************************************************************/
int usb_cdc_init(void)
{
    usb_vcom = rt_device_find("vcom");
    RT_ASSERT(usb_vcom);
    rt_device_init(usb_vcom);
    rt_device_open(usb_vcom, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX);

    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&tx_sem, "tx_sem", 0, RT_IPC_FLAG_FIFO);
    rt_device_set_rx_indicate(usb_vcom, uart_input);
    rt_device_set_tx_complete(usb_vcom, uart_output);
    rt_thread_t thread = rt_thread_create("usb_vcom", serial_thread_entry, RT_NULL, 1024, 25, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    //usb_vbus_pin_irq();//Configure USB plug and unplug detection pins

    return 0;
}
INIT_APP_EXPORT(usb_cdc_init);
#endif

#ifdef RT_USB_DEVICE_PRINTER
static struct               rt_semaphore printer_tx_sema;
rt_device_t usb_printer = NULL;
#define RX_LEN 1024
char printer_rx_buf[RX_LEN] = {0};
/*
    Data receiving callback function
*/
static rt_err_t printer_input(rt_device_t dev, rt_size_t size)
{
    if (size == rt_device_read(usb_printer, -1, &printer_rx_buf, size))
    {
        rt_kprintf("len=%d cdc_rx_buf=%s\n", size, printer_rx_buf);
        rt_memset(printer_rx_buf, 0x00, size);
    }
    return RT_EOK;
}
/*
Data sending completion callback function
*/
static rt_err_t printer_out(rt_device_t dev, void *buffer)
{
    int tx_data_flow = (int)buffer;
    if (tx_data_flow == DATA_FLOW_END)
        rt_sem_release(&printer_tx_sema);
    return RT_EOK;
}

static void usb_printer_thread_entry(void *parameter)
{

}

int usb_printer_init(void)
{
    usb_printer = rt_device_find("printer");
    RT_ASSERT(usb_printer);
    rt_device_init(usb_printer);
    rt_device_open(usb_printer, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX);

    rt_device_set_rx_indicate(usb_printer, printer_input);
    rt_device_set_tx_complete(usb_printer, printer_out);
    RT_ASSERT(RT_EOK == rt_sem_init(&printer_tx_sema, "printer_tx_sema", 0, RT_IPC_FLAG_FIFO));
    rt_thread_t thread = rt_thread_create("usb_printer", usb_printer_thread_entry, RT_NULL, 1024, 25, 10);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    //usb_vbus_pin_irq();//Configure USB plug and unplug detection pins

    return 0;
}
INIT_APP_EXPORT(usb_printer_init);

#endif

#if defined(RT_USING_FINSH) && defined(USB_FINSH_TEST)
#include <finsh.h>

#ifdef RT_USB_DEVICE_PRINTER
void printer_write(int avgc, char **argv)
{
    rt_size_t len =  rt_device_write(usb_printer, 0, argv[1], strlen(argv[1]));
    if (len != strlen(argv[1])) rt_kprintf("write fail!\n");
    else rt_kprintf("write succ!\n");
}
MSH_CMD_EXPORT(printer_write, printer write);

/*
Continuously sending test functions
*/
void printer_write_buff(int avgc, char **argv)
{
    char buff_printer[1024] = {0};
    rt_memset(buff_printer, 0x55, 1024);
    uint16_t num = atoi(argv[1]);
    uint32_t write_byt = num * 1024;//Byte
    uint32_t open_time = HAL_GTIMER_READ();
    while (num --)
    {
        rt_device_write(usb_printer, 0, buff_printer, 1024);
        rt_sem_take(&printer_tx_sema, RT_WAITING_FOREVER);//Waiting for sending to complete
    }
    uint32_t end_time = HAL_GTIMER_READ();
    float test_time = ((end_time - open_time) / HAL_LPTIM_GetFreq()) * 1000;//ms
    float speed_test = (num) / (test_time / 1000); //KB/S
    rt_kprintf("%s open_time=%d endtime=%d testtime=%.6lfms,speed=%.6lfKB/s\n", __func__, open_time, end_time, test_time, speed_test);
}
MSH_CMD_EXPORT(printer_write_buff, printer write buff);
#endif


#ifdef RT_USB_DEVICE_CDC
void vcom_write(int avgc, char **argv)
{
    rt_size_t len =  rt_device_write(usb_vcom, 0, argv[1], strlen(argv[1]));
    if (len != strlen(argv[1])) rt_kprintf("write fail!\n");
    else rt_kprintf("write succ!\n");
}
MSH_CMD_EXPORT(vcom_write, vcom write);

/*
Continuously sending test functions
*/

void vcom_write_buff(int avgc, char **argv)
{
    char buff_vcom[1024] = {0};
    rt_memset(buff_vcom, 0x55, 1024);
    uint16_t num = atoi(argv[1]);
    uint32_t write_byt = num * 1024;//Byte
    uint32_t open_time = HAL_GTIMER_READ();
    while (num--)
    {
        rt_device_write(usb_vcom, 0, buff_vcom, 1024);
        rt_sem_take(&tx_sem, RT_WAITING_FOREVER);//Waiting for sending to complete
    }
    uint32_t end_time = HAL_GTIMER_READ();
    float test_time = ((end_time - open_time) / HAL_LPTIM_GetFreq()) * 1000;//ms
    float speed_test = (num) / (test_time / 1000); //KB/S
    rt_kprintf("%s open_time=%d endtime=%d testtime=%.6lfms,speed=%.6lfKB/s\n", __func__, open_time, end_time, test_time, speed_test);


}
MSH_CMD_EXPORT(vcom_write_buff, vcom write buff);

//Default enabled
//vcom_flow_data 0 1 //open
//vcom_flow_data 0 0 //stop

void vcom_flow_data(int avgc, char **argv)
{
    struct flow_data flow_ep;
    flow_ep.dir = atoi(argv[1]);//Select USB direction
    flow_ep.flag = atoi(argv[2]);//Can it be enabled 1 open 0 stop
    rt_device_control(usb_vcom, DATA_FLOW_CONTROL, &flow_ep);
}
MSH_CMD_EXPORT(vcom_flow_data, vcom_flow data);
#endif

#endif


