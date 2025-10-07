/*
 * USB-UART Bridge using CDC ACM with RT-Thread RingBuffer
 */

#include "usbd_core.h"
#include "usbd_cdc_acm.h"
#include "rtthread.h"
#include "rtdevice.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "board.h"
#include <stdbool.h>

#define DBG_TAG "usb_uart"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* USB Configuration */
#define CDC_IN_EP  0x85
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x86

#define USBD_VID           0xFFFF
#define USBD_PID           0xFFFF
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

/* UART Configuration */
#define UART_DEVICE_NAME "uart2"
#define RING_BUFFER_SIZE 4096

/* Global Variables */
static rt_device_t uart_device = RT_NULL;
static struct rt_semaphore uart_rx_sem;
static struct rt_semaphore usb_rx_sem;
static struct rt_mutex buffer_mutex;

/* Ring Buffers for bidirectional data transfer */
static struct rt_ringbuffer uart_to_usb_rb;
static struct rt_ringbuffer usb_to_uart_rb;
static rt_uint8_t uart_to_usb_pool[RING_BUFFER_SIZE];
static rt_uint8_t usb_to_uart_pool[RING_BUFFER_SIZE];

/* USB transfer buffers - must be in non-cacheable memory */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t usb_read_buffer[2048];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t usb_write_buffer[2048];

/* State flags */
static volatile bool ep_tx_busy = false;
static volatile bool usb_configured = false;



/* Default UART configuration */
static struct serial_configure uart_config = {
    .baud_rate = 1000000,
    .data_bits = DATA_BITS_8,
    .stop_bits = STOP_BITS_1,
    .parity = PARITY_NONE,
    .bit_order = BIT_ORDER_LSB,
    .invert = NRZ_NORMAL,
    .bufsz = 2048
};

/* USB Descriptors */
#ifdef CONFIG_USBDEV_ADVANCE_DESC
static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x02, 0x00, 0x00, USBD_VID, USBD_PID, 0x0200, 0x01)
};

static const uint8_t config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MAX_MPS, 0x02)
};

static const uint8_t device_quality_descriptor[] = {
    0x0a, USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
};

static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 },
    "CherryUSB",
    "CDC_ACM_UART",
    "202412345678",
};

static const uint8_t *device_descriptor_callback(uint8_t speed) { return device_descriptor; }
static const uint8_t *config_descriptor_callback(uint8_t speed) { return config_descriptor; }
static const uint8_t *device_quality_descriptor_callback(uint8_t speed) { return device_quality_descriptor; }

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    if (index > 3) return NULL;
    return string_descriptors[index];
}

const struct usb_descriptor cdc_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback
};
#endif

/* UART RX callback - called when UART receives data */
static rt_err_t uart_rx_callback(rt_device_t dev, rt_size_t size)
{
    if (size > 0) {
        rt_sem_release(&uart_rx_sem);
    }
    return RT_EOK;
}

/* Thread: UART to USB data transfer */
static void uart_to_usb_thread(void *parameter)
{
    uint8_t temp_buffer[512];
    rt_size_t count;
    
    while (1) {
        /* Wait for UART data or check periodically */
        if (rt_sem_take(&uart_rx_sem, RT_TICK_PER_SECOND/100) == RT_EOK || 
            rt_ringbuffer_data_len(&uart_to_usb_rb) > 0) {
            
            /* Read from UART device */
            if (uart_device) {
                count = rt_device_read(uart_device, -1, temp_buffer, sizeof(temp_buffer));
                if (count > 0) {
                    rt_mutex_take(&buffer_mutex, RT_WAITING_FOREVER);
                    rt_size_t written = rt_ringbuffer_put(&uart_to_usb_rb, temp_buffer, count);
                    rt_mutex_release(&buffer_mutex);
                    
                    if (written < count) {
                        LOG_W("UART RX buffer overflow: %u bytes lost", count - written);
                    }
                }
            }
            
            /* Send to USB if data available and USB is ready */
            if (usb_configured && !ep_tx_busy) {
                rt_mutex_take(&buffer_mutex, RT_WAITING_FOREVER);
                count = rt_ringbuffer_get(&uart_to_usb_rb, usb_write_buffer, CDC_MAX_MPS);
                rt_mutex_release(&buffer_mutex);
                
                if (count > 0) {
                    ep_tx_busy = true;
                    int result = usbd_ep_start_write(0, CDC_IN_EP, usb_write_buffer, count);
                    if (result < 0) {
                        ep_tx_busy = false;
                        LOG_E("USB TX failed: %d", result);
                    }
                }
            }
        }
    }
}

/* Thread: USB to UART data transfer */
static void usb_to_uart_thread(void *parameter)
{
    uint8_t temp_buffer[512];
    rt_size_t count;
    
    while (1) {
        /* Wait for USB data */
        rt_sem_take(&usb_rx_sem, RT_WAITING_FOREVER);
        
        /* Send all buffered data to UART */
        if (uart_device) {
            while (1) {
                rt_mutex_take(&buffer_mutex, RT_WAITING_FOREVER);
                count = rt_ringbuffer_get(&usb_to_uart_rb, temp_buffer, sizeof(temp_buffer));
                rt_mutex_release(&buffer_mutex);
                
                if (count == 0) {
                    break;
                }
                
                rt_size_t sent = rt_device_write(uart_device, 0, temp_buffer, count);
                
                /* If UART is busy, put remaining data back */
                if (sent < count) {
                    LOG_W("UART busy, %u bytes pending", count - sent);
                    rt_mutex_take(&buffer_mutex, RT_WAITING_FOREVER);
                    rt_ringbuffer_put_force(&usb_to_uart_rb, temp_buffer + sent, count - sent);
                    rt_mutex_release(&buffer_mutex);
                    break;
                }
            }
        }
    }
}

/* USB Event Handler */
static void usb_event_handler(uint8_t busid, uint8_t event)
{
    switch (event) {
        case USBD_EVENT_CONFIGURED:
            usb_configured = true;
            ep_tx_busy = false;
            /* Start reading from USB */
            usbd_ep_start_read(busid, CDC_OUT_EP, usb_read_buffer, sizeof(usb_read_buffer));
            LOG_I("USB Configured");
            break;
        default:
            break;
    }
}

/* USB CDC ACM Bulk OUT callback - called when USB receives data */
void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    if (nbytes > 0) {
        /* Buffer received data */
        rt_mutex_take(&buffer_mutex, RT_WAITING_FOREVER);
        rt_size_t written = rt_ringbuffer_put(&usb_to_uart_rb, usb_read_buffer, nbytes);
        rt_mutex_release(&buffer_mutex);
        
        if (written < nbytes) {
            LOG_W("USB RX buffer overflow: %u bytes lost", nbytes - written);
        }
        
        /* Signal thread to process data */
        rt_sem_release(&usb_rx_sem);
    }
    
    /* Start next read */
    usbd_ep_start_read(busid, CDC_OUT_EP, usb_read_buffer, sizeof(usb_read_buffer));
}

/* USB CDC ACM Bulk IN callback - called when USB transmission completes */
void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    /* Send ZLP if needed */
    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes) {
        usbd_ep_start_write(busid, CDC_IN_EP, NULL, 0);
    } else {
        /* Clear busy flag to allow next transmission */
        ep_tx_busy = false;
    }
}

/* USB Endpoints */
struct usbd_endpoint cdc_out_ep = { .ep_addr = CDC_OUT_EP, .ep_cb = usbd_cdc_acm_bulk_out };
struct usbd_endpoint cdc_in_ep = { .ep_addr = CDC_IN_EP, .ep_cb = usbd_cdc_acm_bulk_in };

static struct usbd_interface intf0;
static struct usbd_interface intf1;

/* Initialize UART */
static rt_err_t uart_init(void)
{
    rt_err_t result;

    /* Configure UART pins based on board */
#if defined(BSP_USING_BOARD_SF32LB52_LCD_N16R8)
    HAL_PIN_Set(PAD_PA03, USART2_RXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA04, USART2_TXD, PIN_PULLUP, 1);
#elif defined(BSP_USING_BOARD_SF32LB58_LCD_N16R64N4)
    HAL_PIN_Set(PAD_PA29, USART2_RXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA28, USART2_TXD, PIN_PULLUP, 1);
#endif

    /* Find UART device */
    uart_device = rt_device_find(UART_DEVICE_NAME);
    if (!uart_device) {
        LOG_E("UART device %s not found", UART_DEVICE_NAME);
        return RT_ERROR;
    }

    /* Configure UART */
    result = rt_device_control(uart_device, RT_DEVICE_CTRL_CONFIG, &uart_config);
    if (result != RT_EOK) {
        LOG_E("UART config failed: %d", result);
        return result;
    }

    /* Initialize RT-Thread ring buffers */
    rt_ringbuffer_init(&uart_to_usb_rb, uart_to_usb_pool, RING_BUFFER_SIZE);
    rt_ringbuffer_init(&usb_to_uart_rb, usb_to_uart_pool, RING_BUFFER_SIZE);

    /* Initialize semaphores and mutex */
    rt_sem_init(&uart_rx_sem, "uart_rx", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&usb_rx_sem, "usb_rx", 0, RT_IPC_FLAG_FIFO);
    rt_mutex_init(&buffer_mutex, "buf_mtx", RT_IPC_FLAG_PRIO);

    /* Open UART device - try DMA mode first */
    result = rt_device_open(uart_device, 
                           RT_DEVICE_OFLAG_RDWR | 
                           RT_DEVICE_FLAG_DMA_RX | 
                           RT_DEVICE_FLAG_DMA_TX);
    if (result != RT_EOK) {
        /* Fallback to interrupt mode */
        result = rt_device_open(uart_device, 
                               RT_DEVICE_OFLAG_RDWR | 
                               RT_DEVICE_FLAG_INT_RX);
    }
    
    if (result != RT_EOK) {
        LOG_E("UART open failed: %d", result);
        return result;
    }

    /* Set UART RX callback */
    rt_device_set_rx_indicate(uart_device, uart_rx_callback);

    /* Create worker threads */
    rt_thread_t thread1 = rt_thread_create("uart2usb", 
                                          uart_to_usb_thread, 
                                          RT_NULL, 
                                          2048, 10, 10);
    if (thread1) {
        rt_thread_startup(thread1);
    } else {
        LOG_E("Failed to create uart2usb thread");
        return RT_ERROR;
    }

    rt_thread_t thread2 = rt_thread_create("usb2uart", 
                                          usb_to_uart_thread, 
                                          RT_NULL, 
                                          2048, 10, 10);
    if (thread2) {
        rt_thread_startup(thread2);
    } else {
        LOG_E("Failed to create usb2uart thread");
        return RT_ERROR;
    }

    LOG_I("UART initialized successfully");
    return RT_EOK;
}



/* Main initialization function */
void cdc_acm_init(uint8_t busid, uintptr_t reg_base)
{
    /* Register USB descriptors */
#ifdef CONFIG_USBDEV_ADVANCE_DESC
    usbd_desc_register(busid, &cdc_descriptor);
#else
    usbd_desc_register(busid, cdc_descriptor);
#endif

    /* Add USB interfaces and endpoints */
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0));
    usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
    usbd_add_endpoint(busid, &cdc_out_ep);
    usbd_add_endpoint(busid, &cdc_in_ep);

    /* Initialize USB device */
    usbd_initialize(busid, reg_base, usb_event_handler);

    /* Initialize UART */
    if (uart_init() != RT_EOK) {
        LOG_E("Failed to initialize UART");
        return;
    }

    LOG_I("USB-UART bridge initialized");
}

/* USB CDC ACM control callbacks */
void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    /* DTR signal handling - can be used for flow control */
    LOG_D("DTR: %s", dtr ? "Set" : "Clear");
}

void usbd_cdc_acm_set_rts(uint8_t busid, uint8_t intf, bool rts)
{
    /* RTS signal handling - can be used for flow control */
    LOG_D("RTS: %s", rts ? "Set" : "Clear");
}

void usbd_cdc_acm_set_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{
    if (uart_device && line_coding) {
        struct serial_configure config = uart_config;

        /* Update baud rate */
        if (line_coding->dwDTERate >= 300 && line_coding->dwDTERate <= 3000000) {
            config.baud_rate = line_coding->dwDTERate;
            LOG_I("Set baud rate: %u", line_coding->dwDTERate);
        }
        
        /* Update data bits */
        if (line_coding->bDataBits >= 5 && line_coding->bDataBits <= 8) {
            config.data_bits = line_coding->bDataBits;
        }

        /* Update parity */
        config.parity = line_coding->bParityType;
        
        /* Update stop bits */
        config.stop_bits = (line_coding->bCharFormat == 2) ? STOP_BITS_2 : STOP_BITS_1;

        /* Apply configuration */
        if (rt_device_control(uart_device, RT_DEVICE_CTRL_CONFIG, &config) == RT_EOK) {
            uart_config = config;
        }
    }
}

void usbd_cdc_acm_get_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{
    if (line_coding) {
        line_coding->dwDTERate = uart_config.baud_rate;
        line_coding->bCharFormat = (uart_config.stop_bits == STOP_BITS_2) ? 2 : 0;
        line_coding->bParityType = uart_config.parity;
        line_coding->bDataBits = uart_config.data_bits;
    }
}