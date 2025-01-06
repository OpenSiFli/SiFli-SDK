/*
 * File      : printers.c
 * COPYRIGHT (C) 2008 - 2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-03-13     Urey         the first version
 * 2017-11-16     ZYH          Update to common printers
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtservice.h>
#include <rtdevice.h>

#include "drivers/usb_common.h"
#include "drivers/usb_device.h"

#include "printers.h"
#include "mem_section.h"
#include "bf0_hal_pcd.h"


#ifdef RT_USB_DEVICE_PRINTER

struct printer_s
{
    struct rt_device parent;
    struct ufunction *func;
    uep_t ep_in;
    uep_t ep_out;
    int status;
    rt_uint16_t protocol;
    rt_uint8_t report_buf[MAX_REPORT_SIZE];
    struct rt_messagequeue printers_mq;
};
static struct printer_s   *printer_data = RT_NULL;
struct printers_rx_buff
{
    rt_uint16_t size;
    rt_uint8_t buf[MAX_REPORT_SIZE];
};
static struct printers_rx_buff rx_data ;

struct printers_report
{
    uint8_t report_id;
    uint8_t report[63];
    uint8_t size;
};
typedef struct printers_report *printers_report_t;
#define PRINT_SER_NO  "32021919830108"


/* Customprinters_ConfigDescriptor */
ALIGN(4)
const rt_uint8_t _report_desc[] =
{
    // Media Control

}; /* Customprinters_ReportDescriptor */

ALIGN(4)
static struct udevice_descriptor _dev_desc =
{
    USB_DESC_LENGTH_DEVICE,     //bLength;
    USB_DESC_TYPE_DEVICE,       //type;
    USB_BCD_VERSION,            //bcdUSB;
    USB_CLASS_PRINTER,              //bDeviceClass;
    0x00,                       //bDeviceSubClass;
    0x00,                       //bDeviceProtocol;
    64,                         //bMaxPacketSize0;
    _VENDOR_ID,                 //idVendor;
    _PRODUCT_ID,                //idProduct;
    USB_BCD_DEVICE,             //bcdDevice;
    USB_STRING_MANU_INDEX,      //iManufacturer;
    USB_STRING_PRODUCT_INDEX,   //iProduct;
    USB_STRING_SERIAL_INDEX,    //iSerialNumber;
    USB_DYNAMIC,                //bNumConfigurations;
};

//FS and HS needed
ALIGN(4)
static struct usb_qualifier_descriptor dev_qualifier =
{
    sizeof(dev_qualifier),          //bLength
    USB_DESC_TYPE_DEVICEQUALIFIER,  //bDescriptorType
    0x0200,                         //bcdUSB
    USB_CLASS_PRINTER,         //bDeviceClass
    0x06,                           //bDeviceSubClass
    0x50,                           //bDeviceProtocol
    64,                             //bMaxPacketSize0
    0x01,                           //bNumConfigurations
    0,
};


/* printers interface descriptor */
ALIGN(4)
const static struct printf_comm_descriptor _printer_comm_desc =
{
#ifdef RT_USB_DEVICE_COMPOSITE
    {
        /* Interface Association Descriptor */
        USB_DESC_LENGTH_IAD,
        USB_DESC_TYPE_IAD,
        0x03,
        0x01,
        USB_CLASS_PRINTER,                       /* bInterfaceClass: printers */
        0x01,
        0x02,
        0x00,
    },
#endif
    {
        /* Interface Descriptor */
        USB_DESC_LENGTH_INTERFACE,
        USB_DESC_TYPE_INTERFACE,
        0x03,                /* bInterfaceNumber: Number of Interface */
        0x00,                       /* bAlternateSetting: Alternate setting */
        0x02,                       /* bNumEndpoints */
        USB_CLASS_PRINTER,                       /* bInterfaceClass: printers */
        0x01,                         /* bInterfaceSubClass : USB_SUBCLASS_PRINTER */
        0x02,
        0,                          /* iInterface: Index of string descriptor */
    },
    {
        /* Endpoint Descriptor OUT */
        USB_DESC_LENGTH_ENDPOINT,
        USB_DESC_TYPE_ENDPOINT,
        0x03,
        USB_EP_ATTR_BULK,
        0x40,
        0x00,
    },
    {
        /* Endpoint Descriptor IN */
        USB_DESC_LENGTH_ENDPOINT,
        USB_DESC_TYPE_ENDPOINT,
        0x83,
        USB_EP_ATTR_BULK,
        0x40,
        0x00,
    }
};

ALIGN(4)
const static char *prt_ustring[] =
{
    "Language",
    "RT-Thread Team.",
    "RTT printers-Device",
    "32021919830108",
    "Configuration",
    "Interface",
};

const static char prt_desc[32] =
{
    0X00, 0X20, 'M', 'F', 'G', ':',
    'N', 'I', 'I', 'M', 'B', 'O', 'T', ';',
    'C', 'M', 'D', ':', 'E', 'S', 'C', ';'
    , 'M', 'D', 'L', ':', ' ', 'M', '2', '_', 'H', ';'
};

static void dump_data(uint8_t *data, rt_size_t size)
{
    rt_size_t i;
    for (i = 0; i < size; i++)
    {
        rt_kprintf("%02x ", *data++);
        if ((i + 1) % 8 == 0)
        {
            rt_kprintf("\n");
        }
        else if ((i + 1) % 4 == 0)
        {
            rt_kprintf(" ");
        }
    }
}
static void dump_report(struct printers_report *report)
{
    rt_kprintf("\nprinters Recived:");
    rt_kprintf("\nReport ID %02x \n", report->report_id);
    dump_data(report->report, report->size);
}

static rt_err_t _ep_out_handler(ufunction_t func, rt_size_t size)
{
    struct printer_s *data;
    struct printers_report report;
    RT_ASSERT(func != RT_NULL);
    RT_ASSERT(func->device != RT_NULL);
    data = (struct printer_s *) func->user_data;
    RT_DEBUG_LOG(RT_DEBUG_USB, ("%s %d size=%d\n", __func__, __LINE__, size));
    if (data->parent.tx_complete != RT_NULL)
    {
        data->parent.tx_complete(&data->parent, (void *)DATA_FLOW_END);
    }

    return RT_EOK;
}

static rt_err_t _ep_in_handler(ufunction_t func, rt_size_t size)
{
    struct printer_s *data;
    RT_ASSERT(func != RT_NULL);
    RT_ASSERT(func->device != RT_NULL);

    data = (struct printer_s *) func->user_data;
    if (size != 0)
    {
        rt_memcpy(&rx_data.buf, (void *)data->ep_in->buffer, size);
        rx_data.size = size;
        rt_memset((void *)data->ep_in->buffer, 0x00, size);
        if (data->parent.rx_indicate != RT_NULL)
        {
            data->parent.rx_indicate(&data->parent, size);
        }
        RT_DEBUG_LOG(RT_DEBUG_USB, ("%s %d size=%d buff=%p,%s\n", __func__, __LINE__, size, data->ep_in->request.buffer, rx_data.buf));
    }
    return RT_EOK;
}

static rt_err_t _printer_set_report_callback(udevice_t device, rt_size_t size)
{
    RT_DEBUG_LOG(RT_DEBUG_USB, ("_printer_set_report_callback\n"));

    if (size != 0)
    {
    }

    dcd_ep0_send_status(device->dcd);

    return RT_EOK;
}

/**
 * This function will handle printers interface bRequest.
 *
 * @param device the usb device object.
 * @param setup the setup bRequest.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _interface_handler(ufunction_t func, ureq_t setup)
{
    RT_ASSERT(func != RT_NULL);
    RT_ASSERT(func->device != RT_NULL);
    RT_ASSERT(setup != RT_NULL);

    struct printer_s *data = (struct printer_s *) func->user_data;
    RT_DEBUG_LOG(RT_DEBUG_USB, ("printer _interface_handler :%d wValue=%d\n", setup->bRequest, setup->wValue));
    rt_usbd_ep0_write(func->device, (rt_uint8_t *)&prt_desc, 32);
    return RT_EOK;
}


/**
 * This function will run cdc function, it will be called on handle set configuration bRequest.
 *
 * @param func the usb function object.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _function_enable(ufunction_t func)
{
    struct printer_s *data;

    RT_ASSERT(func != RT_NULL);
    RT_ASSERT(func->device != RT_NULL);
    data = (struct printer_s *) func->user_data;

    RT_DEBUG_LOG(RT_DEBUG_USB, ("printers function enable\n"));

    if (data->ep_out->buffer == RT_NULL)
    {
        data->ep_out->buffer        = rt_malloc(RX_BUFSIZE);
    }
    data->ep_out->request.buffer    = data->ep_out->buffer;
    data->ep_out->request.size      = EP_MAXPACKET(data->ep_out);
    data->ep_out->request.req_type  = UIO_REQUEST_READ_BEST;

    rt_usbd_io_request(func->device, data->ep_out, &data->ep_out->request);

    return RT_EOK;
}

/**
 * This function will stop cdc function, it will be called on handle set configuration bRequest.
 *
 * @param func the usb function object.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _function_disable(ufunction_t func)
{
    struct printer_s *data;

    RT_ASSERT(func != RT_NULL);
    RT_ASSERT(func->device != RT_NULL);
    data = (struct printer_s *) func->user_data;

    RT_DEBUG_LOG(RT_DEBUG_USB, ("printers function disable\n"));

    if (data->ep_out->buffer != RT_NULL)
    {
        rt_free(data->ep_out->buffer);
        data->ep_out->buffer = RT_NULL;
    }
    if (data->ep_in->buffer != RT_NULL)
    {
        rt_free(data->ep_in->buffer);
        data->ep_in->buffer = RT_NULL;
    }
    return RT_EOK;
}

static struct ufunction_ops ops =
{
    _function_enable,
    _function_disable,
    RT_NULL,
};


/**
 * This function will configure printers descriptor.
 *
 * @param comm the communication interface number.
 * @param data the data interface number.
 *
 * @return RT_EOK on successful.
 */
static rt_err_t _printer_descriptor_config(printf_comm_desc_t printers, rt_uint8_t cintf_nr)
{
#ifdef RT_USB_DEVICE_COMPOSITE
    printers->iad_desc.bFirstInterface = cintf_nr;
#endif

    return RT_EOK;
}
static rt_size_t _printer_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    struct printer_s *printersdev = (struct printer_s *)dev;
    struct printers_report report;
    rt_size_t send_size = size;
    if (printersdev->func->device->state == USB_STATE_CONFIGURED)
    {
        printersdev->ep_out->request.size = send_size;
        printersdev->ep_out->request.req_type = UIO_REQUEST_WRITE;
        printersdev->ep_out->request.buffer = (rt_uint8_t *)buffer;
        RT_DEBUG_LOG(RT_DEBUG_USB, ("%s %d,send_size=%d %d\n", __func__, __LINE__, send_size, printersdev->ep_out->request.size));
        rt_usbd_io_request(printersdev->func->device, printersdev->ep_out, &printersdev->ep_out->request);
    }
    return 0;
}

static rt_size_t _printer_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    struct printer_s *printersdev = (struct printer_s *)dev;
    if (printersdev->func->device->state == USB_STATE_CONFIGURED)
    {
        rt_memcpy(buffer, rx_data.buf, rx_data.size);
        size = rx_data.size;
        rt_memset(rx_data.buf, 0x00, rx_data.size);
        RT_DEBUG_LOG(RT_DEBUG_USB, ("%s %d\n", __func__, __LINE__));
        return size;
    }
    return 0;
}


static rt_err_t _printer_init(rt_device_t dev)
{
    struct printer_s *printersdev = (struct printer_s *)dev;
    struct printers_report report;
    uint8_t size = printersdev->ep_in->ep_desc->wMaxPacketSize;
    rt_uint8_t *buff_read_test = (rt_uint8_t *)rt_malloc(size);

    printersdev->ep_in->request.buffer = (void *)buff_read_test;
    printersdev->ep_in->request.size = MAX_REPORT_SIZE;
    printersdev->ep_in->request.remain_size = MAX_REPORT_SIZE;
    printersdev->ep_in->request.req_type = UIO_REQUEST_READ_BEST;
    printersdev->ep_in->buffer = (void *)buff_read_test;
    RT_DEBUG_LOG(RT_DEBUG_USB, ("%s %d buff_read_test=%p\n", __func__, __LINE__, buff_read_test));
    return RT_EOK;
}


uint8_t rt_usbd_printer_getepnum(uint8_t ep)
{
    RT_ASSERT(printer_data != RT_NULL);
    if (ep) //in
    {
        rt_kprintf("%s %d in_id_addr=%d\n", __func__, __LINE__, printer_data->ep_in->id->addr);
        return printer_data->ep_in->id->addr;
    }
    else //out
    {
        rt_kprintf("%s %d in_id_addr=%d\n", __func__, __LINE__, printer_data->ep_out->id->addr);
        return printer_data->ep_out->id->addr;
    }
}

//data_dir-> Data direction  1 or 0 ;1->in 0->out
//flag-> 1:continue receive; 0:stop
void _printer_flow_control(uint8_t data_dir, uint8_t flag)
{
    HAL_PCD_Set_RxbuffControl(rt_usbd_printer_getepnum(data_dir), flag);
    if (flag) HAL_PCD_Set_RxscrACK(rt_usbd_printer_getepnum(data_dir));
}

static rt_err_t _printer_control(struct rt_device *dev,
                                 int              cmd,
                                 void             *args)
{
    uint8_t res = 0;
    struct flow_data *data_ep = (struct flow_data *)args;
    RT_DEBUG_LOG(RT_DEBUG_USB, ("%s %d ,cmd=0x%x \n", __func__, __LINE__, cmd));
    switch (cmd)
    {
    case DATA_FLOW_CONTROL:
        _printer_flow_control(data_ep->dir, data_ep->flag);
        RT_DEBUG_LOG(RT_DEBUG_USB, ("%s %d res=%d\n", __func__, __LINE__));
        break;
    }
    return res;
}

RT_WEAK void PRINTERS_Report_Received(printers_report_t report)
{
    dump_report(report);
}
L1_NON_RET_BSS_SECT_BEGIN(printer_thread_stack)
ALIGN(RT_ALIGN_SIZE)
L1_NON_RET_BSS_SECT(printer_thread_stack, static rt_uint8_t printer_thread_stack[512]);
L1_NON_RET_BSS_SECT_END
static struct rt_thread printer_thread;

static void printer_thread_entry(void *parameter)
{
    struct printers_report report;
    struct printer_s *dev;
    dev = (struct printer_s *)parameter;

    while (1)
    {
        if (rt_mq_recv(&dev->printers_mq, &report, sizeof(report), RT_WAITING_FOREVER) != RT_EOK)
            continue;
        PRINTERS_Report_Received(&report);
    }
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops printer_device_ops =
{
    _printer_init,
    RT_NULL,
    RT_NULL,
    _printer_read,
    _printer_write,
    _printer_control,
};
#endif

static rt_uint8_t mq_pool[(sizeof(struct printers_report) + sizeof(void *)) * 8];
static void rt_usb_printer_init(struct ufunction *func)
{
    struct printer_s *printerdev;
    printerdev = (struct printer_s *)func->user_data;
    rt_memset(&printerdev->parent, 0, sizeof(printerdev->parent));

#ifdef RT_USING_DEVICE_OPS
    printerdev->parent.ops   = &printer_device_ops;
#else
    printerdev->parent.init = _printer_init;
    printerdev->parent.read = _printer_read;
    printerdev->parent.write = _printer_write;
    printerdev->parent.control = _printer_control;

#endif
    printerdev->func = func;

    rt_device_register(&printerdev->parent, "printer", RT_DEVICE_FLAG_RDWR);
    rt_mq_init(&printerdev->printers_mq, "printermq", mq_pool, sizeof(struct printers_report),
               sizeof(mq_pool), RT_IPC_FLAG_FIFO);

    rt_thread_init(&printer_thread, "printer", printer_thread_entry, printerdev,
                   printer_thread_stack, sizeof(printer_thread_stack), RT_USBD_THREAD_PRIO, 20);
    rt_thread_startup(&printer_thread);

}
/**
 * This function will create a printers function instance.
 *
 * @param device the usb device object.
 *
 * @return RT_EOK on successful.
 */
ufunction_t rt_usbd_function_printer_create(udevice_t device)
{
    ufunction_t     func;

    uintf_t         printer_intf;
    ualtsetting_t   printer_setting;
    printf_comm_desc_t printf_desc;

    /* parameter check */
    RT_ASSERT(device != RT_NULL);

    /* set usb device string description */
    rt_usbd_device_set_string(device, prt_ustring);

    /* create a cdc function */
    func = rt_usbd_function_new(device, &_dev_desc, &ops);
    //not support hs
    //rt_usbd_device_set_qualifier(device, &_dev_qualifier);

    /* allocate memory for cdc vcom data */
    printer_data = (struct printer_s *)rt_malloc(sizeof(struct printer_s));
    rt_memset(printer_data, 0, sizeof(struct printer_s));
    func->user_data = (void *)printer_data;

    /* create an interface object */
    printer_intf = rt_usbd_interface_new(device, _interface_handler);

    /* create an alternate setting object */
    printer_setting = rt_usbd_altsetting_new(sizeof(struct printf_comm_descriptor));

    /* config desc in alternate setting */
    rt_usbd_altsetting_config_descriptor(printer_setting, &_printer_comm_desc, (rt_off_t) & ((printf_comm_desc_t)0)->intf_desc);

    /* configure the printers interface descriptor */
    _printer_descriptor_config(printer_setting->desc, printer_intf->intf_num);

    /* create endpoint */
    printf_desc = (printf_comm_desc_t)printer_setting->desc;
    printer_data->ep_out = rt_usbd_endpoint_new(&printf_desc->ep_out_desc, _ep_out_handler);
    printer_data->ep_in  = rt_usbd_endpoint_new(&printf_desc->ep_in_desc, _ep_in_handler);

    /* add the int out and int in endpoint to the alternate setting */
    rt_usbd_altsetting_add_endpoint(printer_setting, printer_data->ep_out);
    rt_usbd_altsetting_add_endpoint(printer_setting, printer_data->ep_in);

    /* add the alternate setting to the interface, then set default setting */
    rt_usbd_interface_add_altsetting(printer_intf, printer_setting);
    rt_usbd_set_altsetting(printer_intf, 0);

    /* add the interface to the mass storage function */
    rt_usbd_function_add_interface(func, printer_intf);

    /* initilize printers */
    rt_usb_printer_init(func);
    return func;
}
struct udclass printer_class =
{
    .rt_usbd_function_create = rt_usbd_function_printer_create
};

int rt_usbd_printer_class_register(void)
{
    rt_usbd_class_register(&printer_class);
    return 0;
}
INIT_PREV_EXPORT(rt_usbd_printer_class_register);
#endif /* RT_USB_DEVICE_PRINTER */
