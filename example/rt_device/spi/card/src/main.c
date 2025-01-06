#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "board.h"
#include "spi_msd.h"
#define DBG_TAG "spi1"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* spi example for RT-Thread based platform -----------------------------------------------*/
#include "drv_spi.h"
#ifdef MSD_TRACE
    #define MSD_DEBUG(...)         rt_kprintf("[MSD] %d ", rt_tick_get()); rt_kprintf(__VA_ARGS__);
#else
    #define MSD_DEBUG(...)
#endif /* #ifdef MSD_TRACE */

#define DUMMY                 0xFF

#define CARD_NCR_MAX          9

#define CARD_NRC              1
#define CARD_NCR              1
static struct rt_spi_device *spi_dev_handle = {0};

static struct msd_device  _msd_device;
ALIGN(4)
static const uint8_t ones_data[512] =
{
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

#define NOR_FLASH_DEVICE_NAME     "nor_flash"
#define SPI_BUS_NAME "spi1"
/* function define */
static rt_bool_t rt_tick_timeout(rt_tick_t tick_start, rt_tick_t tick_long);

static rt_err_t MSD_take_owner(struct rt_spi_device *spi_device);

static rt_err_t _wait_token(struct rt_spi_device *device, uint8_t token);
static rt_err_t _wait_ready(struct rt_spi_device *device);
static rt_err_t  rt_msd_init(rt_device_t dev);
static rt_err_t  rt_msd_open(rt_device_t dev, rt_uint16_t oflag);
static rt_err_t  rt_msd_close(rt_device_t dev);
static rt_size_t rt_msd_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size);
static rt_size_t rt_msd_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size);
static rt_size_t rt_msd_sdhc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size);
static rt_size_t rt_msd_sdhc_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size);
static rt_err_t rt_msd_control(rt_device_t dev, int cmd, void *args);

static rt_err_t MSD_take_owner(struct rt_spi_device *spi_device)
{
    rt_err_t result;

    result = rt_mutex_take(&(spi_device->bus->lock), RT_WAITING_FOREVER);
    if (result == RT_EOK)
    {
        if (spi_device->bus->owner != spi_device)
        {
            /* not the same owner as current, re-configure SPI bus */
            result = spi_device->bus->ops->configure(spi_device, &spi_device->config);
            if (result == RT_EOK)
            {
                /* set SPI bus owner */
                spi_device->bus->owner = spi_device;
            }
        }
    }

    return result;
}

static rt_err_t MSD_take_detection(struct rt_spi_device *spi_device)
{
    rt_err_t result;

    result = rt_mutex_take(&(spi_device->bus->lock), 1000);
    if (result == RT_EOK)
    {
        if (spi_device->bus->owner != spi_device)
        {
            /* not the same owner as current, re-configure SPI bus */
            result = spi_device->bus->ops->configure(spi_device, &spi_device->config);
            if (result == RT_EOK)
            {
                /* set SPI bus owner */
                spi_device->bus->owner = spi_device;
            }
        }
    }

    return result;
}


static rt_bool_t rt_tick_timeout(rt_tick_t tick_start, rt_tick_t tick_long)
{
    rt_tick_t tick_end = tick_start + tick_long;
    rt_tick_t tick_now = rt_tick_get();
    rt_bool_t result = RT_FALSE;

    if (tick_end >= tick_start)
    {
        if (tick_now >= tick_end)
        {
            result = RT_TRUE;
        }
        else
        {
            result = RT_FALSE;
        }
    }
    else
    {
        if ((tick_now < tick_start) && (tick_now >= tick_end))
        {
            result = RT_TRUE;
        }
        else
        {
            result = RT_FALSE;
        }
    }

    return result;
}

static uint8_t crc7(const uint8_t *buf, int len)
{
    unsigned char   i, j, crc, ch, ch2, ch3;

    crc = 0;

    for (i = 0; i < len; i ++)
    {
        ch = buf[i];

        for (j = 0; j < 8; j ++, ch <<= 1)
        {
            ch2 = (crc & 0x40) ? 1 : 0;
            ch3 = (ch & 0x80) ? 1 : 0;

            if (ch2 ^ ch3)
            {
                crc ^= 0x04;
                crc <<= 1;
                crc |= 0x01;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static rt_err_t _send_cmd(
    struct rt_spi_device *device,
    uint8_t cmd,
    uint32_t arg,
    uint8_t crc,
    response_type type,
    uint8_t *response
)
{
    struct rt_spi_message message;
    uint8_t cmd_buffer[8];
    uint8_t recv_buffer[sizeof(cmd_buffer)];
    uint32_t i;

    cmd_buffer[0] = DUMMY;
    cmd_buffer[1] = (cmd | 0x40);
    cmd_buffer[2] = (uint8_t)(arg >> 24);
    cmd_buffer[3] = (uint8_t)(arg >> 16);
    cmd_buffer[4] = (uint8_t)(arg >> 8);
    cmd_buffer[5] = (uint8_t)(arg);

    if (crc == 0x00)
    {
        crc = crc7(&cmd_buffer[1], 5);
        crc = (crc << 1) | 0x01;
    }
    cmd_buffer[6] = (crc);

    cmd_buffer[7] = DUMMY;

    /* initial message */
    message.send_buf = cmd_buffer;
    message.recv_buf = recv_buffer;
    message.length = sizeof(cmd_buffer);
    message.cs_take = message.cs_release = 0;

    _wait_ready(device);

    /* transfer message */
    device->bus->ops->xfer(device, &message);

    for (i = CARD_NCR; i < (CARD_NCR_MAX + 1); i++)
    {
        uint8_t send = DUMMY;

        /* initial message */
        message.send_buf = &send;
        message.recv_buf = response;
        message.length = 1;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);

        if (0 == (response[0] & 0x80))
        {
            break;
        }
    } /* wait response */

    if ((CARD_NCR_MAX + 1) == i)
    {
        return RT_ERROR;//fail
    }

    //recieve other byte
    if (type == response_r1)
    {
        return RT_EOK;
    }
    else if (type == response_r1b)
    {
        rt_tick_t tick_start = rt_tick_get();
        uint8_t recv;

        while (1)
        {
            /* initial message */
            message.send_buf = RT_NULL;
            message.recv_buf = &recv;
            message.length = 1;
            message.cs_take = message.cs_release = 0;

            /* transfer message */
            device->bus->ops->xfer(device, &message);

            if (recv == DUMMY)
            {
                return RT_EOK;
            }

            if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(2000)))
            {
                return RT_ETIMEOUT;
            }
        }
    }
    else if (type == response_r2)
    {
        /* initial message */
        message.send_buf = RT_NULL;
        message.recv_buf = response + 1;
        message.length = 1;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
    }
    else if ((type == response_r3) || (type == response_r7))
    {
        /* initial message */
        message.send_buf = RT_NULL;
        message.recv_buf = response + 1;
        message.length = 4;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
    }
    else
    {
        return RT_ERROR; // unknow type?
    }

    return RT_EOK;
}

static rt_err_t _wait_token(struct rt_spi_device *device, uint8_t token)
{
    struct rt_spi_message message;
    rt_tick_t tick_start;
    uint8_t send, recv;

    tick_start = rt_tick_get();

    /* wati token */
    /* initial message */
    send = DUMMY;
    message.send_buf = &send;
    message.recv_buf = &recv;
    message.length = 1;
    message.cs_take = message.cs_release = 0;

    while (1)
    {
        /* transfer message */
        device->bus->ops->xfer(device, &message);

        if (recv == token)
        {
            return RT_EOK;
        }

        if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(CARD_WAIT_TOKEN_TIMES)))
        {
            MSD_DEBUG("[err] wait data start token timeout!\r\n");
            return RT_ETIMEOUT;
        }
    } /* wati token */
}

static rt_err_t _wait_ready(struct rt_spi_device *device)
{
    struct rt_spi_message message;
    rt_tick_t tick_start;
    uint8_t send, recv;

    tick_start = rt_tick_get();

    send = DUMMY;
    /* initial message */
    message.send_buf = &send;
    message.recv_buf = &recv;
    message.length = 1;
    message.cs_take = message.cs_release = 0;

    while (1)
    {
        /* transfer message */
        device->bus->ops->xfer(device, &message);

        if (recv == DUMMY)
        {
            return RT_EOK;
        }

        if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(1000)))
        {
            MSD_DEBUG("[err] wait ready timeout!\r\n");
            return RT_ETIMEOUT;
        }
    }
}

static rt_err_t _read_block(struct rt_spi_device *device, void *buffer, uint32_t block_size)
{
    struct rt_spi_message message;
    rt_err_t result;

    /* wati token */
    result = _wait_token(device, MSD_TOKEN_READ_START);
    if (result != RT_EOK)
    {
        return result;
    }

    /* read data */
    {
        RT_ASSERT(sizeof(ones_data) >= block_size);
        /* initial message */
        /* The SD protocol requires sending ones while reading
         */
        message.send_buf = ones_data;
        message.recv_buf = buffer;
        message.length = block_size;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
    } /* read data */

    /* get crc */
    {
        uint8_t recv_buffer[2];

        /* initial message */
        message.send_buf = ones_data;
        message.recv_buf = recv_buffer;
        message.length = 2;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
    } /* get crc */

    return RT_EOK;
}

static rt_err_t _write_block(struct rt_spi_device *device, const void *buffer, uint32_t block_size, uint8_t token)
{
    struct rt_spi_message message;
    uint8_t send_buffer[16], recv_buffer[16];

    rt_memset(send_buffer, DUMMY, sizeof(send_buffer));
    send_buffer[sizeof(send_buffer) - 1] = token;

    /* send start block token */
    {
        /* initial message */
        message.send_buf = send_buffer;
        message.recv_buf = recv_buffer;
        message.length = sizeof(send_buffer);
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
    }

    /* send data */
    {
        /* initial message */
        message.send_buf = buffer;
        message.recv_buf = (void *)buffer;
        message.length = block_size;
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);
    }

    /* put crc and get data response */
    {
        uint8_t recv_buffer[3];
        uint8_t response;

        /* initial message */
        message.send_buf = send_buffer;
        message.recv_buf = recv_buffer;
        message.length = sizeof(recv_buffer);
        message.cs_take = message.cs_release = 0;

        /* transfer message */
        device->bus->ops->xfer(device, &message);

//        response = 0x0E & recv_buffer[2];
        response = MSD_GET_DATA_RESPONSE(recv_buffer[2]);
        if (response != MSD_DATA_OK)
        {
            MSD_DEBUG("[err] write block fail! data response : 0x%02X\r\n", response);
            return RT_ERROR;
        }
    }

    /* wati ready */
    return _wait_ready(device);
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops msd_ops =
{
    RT_NULL,
    rt_msd_open,
    rt_msd_close,
    rt_msd_read,
    rt_msd_write,
    rt_msd_control
};

const static struct rt_device_ops msd_sdhc_ops =
{
    RT_NULL,
    rt_msd_open,
    rt_msd_close,
    rt_msd_sdhc_read,
    rt_msd_sdhc_write,
    rt_msd_control
};
#endif

/* RT-Thread Device Driver Interface */
static rt_err_t rt_msd_init(rt_device_t dev)
{
    struct msd_device *msd = (struct msd_device *)dev;
    uint8_t response[MSD_RESPONSE_MAX_LEN];
    rt_err_t result = RT_EOK;
    rt_tick_t tick_start;
    uint32_t OCR;
    rt_kprintf("%s %d msd=0x%p device=0x%p\n", __func__, __LINE__, msd, msd->spi_device);
    if (msd->spi_device == RT_NULL)
    {
        rt_kprintf("[err] the SPI SD device has no SPI!\r\n");
        return RT_EIO;
    }
#if 1 //Temporarily adopting a fixed frequency method
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
        cfg.max_hz = 400 * 1000; /* 24MHzbit/s */
        cfg.frameMode = RT_SPI_MOTO;
        rt_spi_configure(msd->spi_device, &cfg);
    }
#endif

    /* init SD card */
    {
        struct rt_spi_message message;

        result = MSD_take_owner(msd->spi_device);

        if (result != RT_EOK)
        {
            goto _exit;
        }

        rt_spi_release(msd->spi_device);

        /* The host shall supply power to the card so that the voltage is reached to Vdd_min within 250ms and
           start to supply at least 74 SD clocks to the SD card with keeping CMD line to high.
           In case of SPI mode, CS shall be held to high during 74 clock cycles. */

        {
            uint8_t send_buffer[100]; /* 100byte > 74 clock */

            /* initial message */
            memset(send_buffer, DUMMY, sizeof(send_buffer));
            message.send_buf = send_buffer;
            message.recv_buf = RT_NULL;
            message.length = sizeof(send_buffer);
            message.cs_take = message.cs_release = 0;

            /* transfer message */
            msd->spi_device->bus->ops->xfer(msd->spi_device, &message);
        } /* send 74 clock */

        /* Send CMD0 (GO_IDLE_STATE) to put MSD in SPI mode */
        {
            tick_start = rt_tick_get();
            while (1)
            {
                rt_spi_take(msd->spi_device);
                result = _send_cmd(msd->spi_device, GO_IDLE_STATE, 0x00, 0x95, response_r1, response);
                rt_spi_release(msd->spi_device);

                if ((result == RT_EOK) && (response[0] == MSD_IN_IDLE_STATE))
                {
                    break;
                }

                if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(CARD_TRY_TIMES)))
                {
                    rt_kprintf("[err] SD card goto IDLE mode timeout!\r\n");
                    result = RT_ETIMEOUT;
                    goto _exit;
                }
            }

            rt_kprintf("[info] SD card goto IDLE mode OK!\r\n");
        } /* Send CMD0 (GO_IDLE_STATE) to put MSD in SPI mode */

        /* CMD8 */
        {
            tick_start = rt_tick_get();

            do
            {
                rt_spi_take(msd->spi_device);
                result = _send_cmd(msd->spi_device, SEND_IF_COND, 0x01AA, 0x87, response_r7, response);
                rt_spi_release(msd->spi_device);

                if (result == RT_EOK)
                {
                    rt_kprintf("[info] CMD8 response : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n",
                               response[0], response[1], response[2], response[3], response[4]);

                    if (response[0] & (1 << 2))
                    {
                        /* illegal command, SD V1.x or MMC card */
                        rt_kprintf("[info] CMD8 is illegal command.\r\n");
                        rt_kprintf("[info] maybe Ver1.X SD Memory Card or MMC card!\r\n");
                        msd->card_type = MSD_CARD_TYPE_SD_V1_X;
                        break;
                    }
                    else
                    {
                        /* SD V2.0 or later or SDHC or SDXC memory card! */
                        rt_kprintf("[info] Ver2.00 or later or SDHC or SDXC memory card!\r\n");
                        msd->card_type = MSD_CARD_TYPE_SD_V2_X;
                    }

                    if ((0xAA == response[4]) && (0x00 == response[3]))
                    {
                        /* SD2.0 not support current voltage */
                        rt_kprintf("[err] VCA = 0, SD2.0 not surpport current operation voltage range\r\n");
                        result = RT_ERROR;
                        goto _exit;
                    }
                }
                else
                {
                    if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(200)))
                    {
                        rt_kprintf("[err] CMD8 SEND_IF_COND timeout!\r\n");
                        result = RT_ETIMEOUT;
                        goto _exit;
                    }
                }
            }
            while (0xAA != response[4]);
        } /* CMD8 */
        //CMD55
        {
            /* try CMD55 + ACMD41 */
            do
            {
                rt_spi_take(msd->spi_device);
                if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(CARD_TRY_TIMES_ACMD41)))
                {
                    rt_spi_release(msd->spi_device);
                    rt_kprintf("[err] SD Ver2.x or later try CMD55 + ACMD41 timeout!\r\n");
                    result = RT_ERROR;
                    goto _exit;
                }

                /* CMD55 APP_CMD */
                result = _send_cmd(msd->spi_device, APP_CMD, 0x00, 0x65, response_r1, response);
//                if((result != RT_EOK) || (response[0] == 0x01))
                //  rt_kprintf("result:%d\n",result);
                // if(result==RT_EOK)
                // {
                //         rt_kprintf("[info] CMD55 response : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n",
                //               response[0], response[1], response[2], response[3], response[4]);
                // }
                if (result != RT_EOK)
                {
                    rt_kprintf("zmjzmjzmjz\n");
                    rt_spi_release(msd->spi_device);
                    continue;
                }

                if ((response[0] & 0xFE) != 0)
                {
                    rt_spi_release(msd->spi_device);
                    rt_kprintf("[err] Not SD ready!\r\n");
                    result = RT_ERROR;
                    goto _exit;
                }

                /* ACMD41 SD_SEND_OP_COND */
                result = _send_cmd(msd->spi_device, SD_SEND_OP_COND, 0x40000000, 0x77, response_r1, response);
                if (result != RT_EOK)
                {
                    rt_spi_release(msd->spi_device);
                    rt_kprintf("[err] ACMD41 fail!\r\n");
                    result = RT_ERROR;
                    goto _exit;
                }

                if ((response[0] & 0xFE) != 0)
                {
                    rt_spi_release(msd->spi_device);
                    rt_kprintf("[info] Not SD card4 , response : 0x%02X\r\n", response[0]);
                    //break;
                }
            }
            while (response[0] != MSD_RESPONSE_NO_ERROR);
            rt_spi_release(msd->spi_device);
            \
            /* try CMD55 + ACMD41 */
        }//CMD55+ACMD41

    }//init tfcard


    //send  CMD10 CID
    {

        tick_start = rt_tick_get();
        do
        {
            rt_spi_take(msd->spi_device);
            result = _send_cmd(msd->spi_device, SEND_CID, 0x0000, 0xc0, response_r1, response);
            if (result != RT_EOK)
            {
                rt_spi_release(msd->spi_device);
                rt_kprintf("[ERR] SEND CMD10 fail\r\n");
                result = RT_ERROR;
                goto _exit;
            }
            if ((response[0] & 0xFE) != 0)
            {
                rt_spi_release(msd->spi_device);
                rt_kprintf("[info] Not response fail : 0x%02X\r\n", response[0]);
                //break;
            }
        }
        while (response[0] != MSD_RESPONSE_NO_ERROR);
        rt_spi_release(msd->spi_device);
    }


_exit:
    rt_spi_release(msd->spi_device);
    rt_mutex_release(&(msd->spi_device->bus->lock));
    return result;
}
static rt_uint8_t spi_recv_byte(rt_device_t device)
{

    spi_dev_handle = (struct rt_spi_device *)device;
    rt_size_t size;
    struct rt_spi_message msg;
    rt_uint8_t recv_buf[1];
    msg.send_buf = RT_NULL;
    msg.recv_buf = recv_buf;
    msg.length = 1;
    msg.cs_take = 0;
    msg.cs_release = 0;
    msg.next = 0;
    rt_spi_transfer_message(spi_dev_handle, &msg);

    return recv_buf[0];
}
static rt_err_t rt_msd_open(rt_device_t dev, rt_uint16_t oflag)
{
//    struct msd_device * msd = (struct msd_device *)dev;
    return RT_EOK;
}

static rt_err_t rt_msd_close(rt_device_t dev)
{
//    struct msd_device * msd = (struct msd_device *)dev;
    return RT_EOK;
}

static rt_size_t rt_msd_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    struct msd_device *msd = (struct msd_device *)dev;
    uint8_t response[MSD_RESPONSE_MAX_LEN];
    rt_err_t result = RT_EOK;
    pos += msd->offset;
    result = MSD_take_owner(msd->spi_device);

    if (result != RT_EOK)
    {
        goto _exit;
    }

    /* SINGLE_BLOCK? */
    if (size == 1)
    {
        rt_spi_take(msd->spi_device);

        result = _send_cmd(msd->spi_device, READ_SINGLE_BLOCK, pos * msd->geometry.bytes_per_sector, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] read SINGLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
            goto _exit;
        }

        result = _read_block(msd->spi_device, buffer, msd->geometry.bytes_per_sector);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[err] read SINGLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
        }
    }
    else if (size > 1)
    {
        uint32_t i;

        rt_spi_take(msd->spi_device);

        result = _send_cmd(msd->spi_device, READ_MULTIPLE_BLOCK, pos * msd->geometry.bytes_per_sector, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] read READ_MULTIPLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
            goto _exit;
        }

        for (i = 0; i < size; i++)
        {
            result = _read_block(msd->spi_device,
                                 (uint8_t *)buffer + msd->geometry.bytes_per_sector * i,
                                 msd->geometry.bytes_per_sector);
            if (result != RT_EOK)
            {
                MSD_DEBUG("[err] read READ_MULTIPLE_BLOCK #%d fail!\r\n", pos);
                size = i;
                break;
            }
        }

        /* send CMD12 stop transfer */
        result = _send_cmd(msd->spi_device, STOP_TRANSMISSION, 0x00, 0x00, response_r1b, response);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[err] read READ_MULTIPLE_BLOCK, send stop token fail!\r\n");
        }
    } /* READ_MULTIPLE_BLOCK */

_exit:
    /* release and exit */
    rt_spi_release(msd->spi_device);
    rt_mutex_release(&(msd->spi_device->bus->lock));

    return size;
}
//extern uint8_t test_flag;

static rt_size_t rt_msd_sdhc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)//4096//8
{
    struct msd_device *msd = (struct msd_device *)dev;
    uint8_t response[MSD_RESPONSE_MAX_LEN];
    rt_err_t result = RT_EOK;
    pos +=  msd->offset;
#if 0 //Due to issues with multi block reading, it has been temporarily changed to single block reading
    uint16_t blk_offset = 0;
    //rt_kprintf("%s %d dev-name=%s pos=0x%x,offset=0x%x,size=%d buffer=%p\n", __func__, __LINE__, dev->parent.name, pos, msd->offset, size, buffer);
    result = MSD_take_owner(msd->spi_device);
    while (size)
    {

        if (result != RT_EOK)
        {
            goto _exit;
        }
        rt_spi_take(msd->spi_device);
        result = _send_cmd(msd->spi_device, READ_SINGLE_BLOCK, pos + blk_offset, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] read SINGLE_BLOCK #%d fail! result=%d %d\r\n", pos, result, response[0]);
            size = 0;
            goto _exit;
        }
        result = _read_block(msd->spi_device,
                             (uint8_t *)buffer + msd->geometry.bytes_per_sector * blk_offset,
                             msd->geometry.bytes_per_sector);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[err] read SINGLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
            goto _exit;
        }
        blk_offset ++;
        size --;
    }
#else

    result = MSD_take_owner(msd->spi_device);

    if (result != RT_EOK)
    {
        goto _exit;
    }
    /* SINGLE_BLOCK? */
    if (size == 1)
    {
        rt_spi_take(msd->spi_device);

        result = _send_cmd(msd->spi_device, READ_SINGLE_BLOCK, pos, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] read SINGLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
            goto _exit;
        }

        result = _read_block(msd->spi_device, buffer, msd->geometry.bytes_per_sector);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[err] read SINGLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
        }
    }
    else if (size > 1)
    {
        uint32_t i;

        rt_spi_take(msd->spi_device);

        result = _send_cmd(msd->spi_device, READ_MULTIPLE_BLOCK, pos, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] read READ_MULTIPLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
            goto _exit;
        }

        for (i = 0; i < size; i++)
        {
            result = _read_block(msd->spi_device,
                                 (uint8_t *)buffer + msd->geometry.bytes_per_sector * i,
                                 msd->geometry.bytes_per_sector);
            if (result != RT_EOK)
            {
                MSD_DEBUG("[err] read READ_MULTIPLE_BLOCK #%d fail!\r\n", pos);
                size = i;
                break;
            }
        }

        /* send CMD12 stop transfer */
        result = _send_cmd(msd->spi_device, STOP_TRANSMISSION, 0x00, 0x00, response_r1b, response);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[err] read READ_MULTIPLE_BLOCK, send stop token fail!\r\n");
        }
    } /* READ_MULTIPLE_BLOCK */
#endif
_exit:
    /* release and exit */
    rt_spi_release(msd->spi_device);
    rt_mutex_release(&(msd->spi_device->bus->lock));
    return size;//blk_offset;//size;
}

static rt_size_t rt_msd_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    struct msd_device *msd = (struct msd_device *)dev;
    uint8_t response[MSD_RESPONSE_MAX_LEN];
    rt_err_t result;
    pos += msd->offset;

    result = MSD_take_owner(msd->spi_device);

    if (result != RT_EOK)
    {
        MSD_DEBUG("[err] get SPI owner fail!\r\n");
        goto _exit;
    }


    /* SINGLE_BLOCK? */
    if (size == 1)
    {
        rt_spi_take(msd->spi_device);
        result = _send_cmd(msd->spi_device, WRITE_BLOCK, pos * msd->geometry.bytes_per_sector, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] CMD WRITE_BLOCK fail!\r\n");
            size = 0;
            goto _exit;
        }

        result = _write_block(msd->spi_device, buffer, msd->geometry.bytes_per_sector, MSD_TOKEN_WRITE_SINGLE_START);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[err] write SINGLE_BLOCK #%d fail!\r\n", pos);
            size = 0;
        }
    }
    else if (size > 1)
    {
        struct rt_spi_message message;
        uint32_t i;

        rt_spi_take(msd->spi_device);

#ifdef MSD_USE_PRE_ERASED
        if (msd->card_type != MSD_CARD_TYPE_MMC)
        {
            /* CMD55 APP_CMD */
            result = _send_cmd(msd->spi_device, APP_CMD, 0x00, 0x00, response_r1, response);
            if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
            {
                MSD_DEBUG("[err] CMD55 APP_CMD fail!\r\n");
                size = 0;
                goto _exit;
            }

            /* ACMD23 Pre-erased */
            result = _send_cmd(msd->spi_device, SET_WR_BLK_ERASE_COUNT, size, 0x00, response_r1, response);
            if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
            {
                MSD_DEBUG("[err] ACMD23 SET_BLOCK_COUNT fail!\r\n");
                size = 0;
                goto _exit;
            }
        }
#endif

        result = _send_cmd(msd->spi_device, WRITE_MULTIPLE_BLOCK, pos * msd->geometry.bytes_per_sector, 0x00, response_r1, response);
        if ((result != RT_EOK) || (response[0] != MSD_RESPONSE_NO_ERROR))
        {
            MSD_DEBUG("[err] CMD WRITE_MULTIPLE_BLOCK fail!\r\n");
            size = 0;
            goto _exit;
        }

        /* write all block */
        for (i = 0; i < size; i++)
        {
            result = _write_block(msd->spi_device,
                                  (const uint8_t *)buffer + msd->geometry.bytes_per_sector * i,
                                  msd->geometry.bytes_per_sector,
                                  MSD_TOKEN_WRITE_MULTIPLE_START);
            if (result != RT_EOK)
            {
                MSD_DEBUG("[err] write SINGLE_BLOCK #%d fail!\r\n", pos);
                size = i;
                break;
            }
        } /* write all block */

        /* send stop token */
        {
            uint8_t send_buffer[18];

            rt_memset(send_buffer, DUMMY, sizeof(send_buffer));
            send_buffer[sizeof(send_buffer) - 1] = MSD_TOKEN_WRITE_MULTIPLE_STOP;

            /* initial message */
            message.send_buf = send_buffer;
            message.recv_buf = RT_NULL;
            message.length = sizeof(send_buffer);
            message.cs_take = message.cs_release = 0;

            /* transfer message */
            msd->spi_device->bus->ops->xfer(msd->spi_device, &message);
        }

        /* wait ready */
        result = _wait_ready(msd->spi_device);
        if (result != RT_EOK)
        {
            MSD_DEBUG("[warning] wait WRITE_MULTIPLE_BLOCK stop token ready timeout!\r\n");
        }
    } /* size > 1 */

_exit:
    /* release and exit */
    rt_spi_release(msd->spi_device);
    rt_mutex_release(&(msd->spi_device->bus->lock));

    return size;
}

static rt_err_t rt_msd_detection(rt_device_t dev)
{
    struct msd_device *msd = (struct msd_device *)dev;
    uint8_t response[MSD_RESPONSE_MAX_LEN];
    rt_err_t result = RT_EOK;
    rt_tick_t tick_start;
    result = MSD_take_detection(msd->spi_device);
    if (result != RT_EOK)
    {
        rt_kprintf("[err] MSD take owner fail result=%d \r\n", result);
        if (-RT_ETIMEOUT == result) result = RT_EOK;
        goto _exit;
    }
    /* CMD8 */
    {
        tick_start = rt_tick_get();
        do
        {
            rt_spi_take(msd->spi_device);
            result = _send_cmd(msd->spi_device, SEND_IF_COND, 0x01AA, 0x87, response_r7, response);
            rt_spi_release(msd->spi_device);

            if (result == RT_EOK)
            {
                if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(200)))
                {
                    MSD_DEBUG("[err] CMD8 SEND_IF_COND timeout!\r\n");
                    result = RT_ETIMEOUT;
                    goto _exit;
                }
                if ((0xAA == response[4]) && (0x00 == response[3]))
                {
                    /* SD2.0 not support current voltage */
                    MSD_DEBUG("[err] VCA = 0, SD2.0 not surpport current operation voltage range\r\n");
                    result = RT_ERROR;
                    goto _exit;
                }
            }
            else
            {
                if (rt_tick_timeout(tick_start, rt_tick_from_millisecond(200)))
                {
                    MSD_DEBUG("[err] CMD8 SEND_IF_COND timeout!\r\n");
                    result = RT_ETIMEOUT;
                    goto _exit;
                }
            }
        }
        while (0xAA != response[4]);
    } /* CMD8 */
_exit:

    rt_spi_release(msd->spi_device);
    rt_mutex_release(&(msd->spi_device->bus->lock));

    return result;

}

static rt_err_t rt_msd_control(rt_device_t dev, int cmd, void *args)
{
    struct msd_device *msd = (struct msd_device *)dev;
    struct rt_device_blk_geometry *geometry;
    rt_err_t result = RT_EOK;
    RT_ASSERT(dev != RT_NULL);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_BLK_GETGEOME:
        geometry = (struct rt_device_blk_geometry *)args;
        if (geometry == RT_NULL) return -RT_ERROR;

        geometry->bytes_per_sector = msd->geometry.bytes_per_sector;
        geometry->block_size = msd->geometry.block_size;
        geometry->sector_count = msd->geometry.sector_count;
        break;
    case RT_DEVICE_CTRL_GET_INT:
        result = rt_msd_detection(dev);
        break;
    }

    return result;
}


rt_err_t msd_init(const char *sd_device_name, const char *spi_device_name)
{
    rt_err_t result = RT_EOK;
    struct rt_spi_device *spi_device;

    spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if (spi_device == RT_NULL)
    {
        MSD_DEBUG("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }

    rt_memset(&_msd_device, 0, sizeof(_msd_device));
    _msd_device.spi_device = spi_device;
    rt_kprintf("%s %d _msd_device.spi_device=0x%p\n", __func__, __LINE__, _msd_device.spi_device);

    /* register sdcard device */
    _msd_device.parent.type    = RT_Device_Class_Block;

    _msd_device.geometry.bytes_per_sector = 0;
    _msd_device.geometry.sector_count = 0;
    _msd_device.geometry.block_size = 0;

#ifdef RT_USING_DEVICE_OPS
    _msd_device.parent.ops     = &msd_ops;
#else
    _msd_device.parent.init    = RT_NULL;
    _msd_device.parent.open    = rt_msd_open;
    _msd_device.parent.close   = rt_msd_close;
    _msd_device.parent.read    = rt_msd_read;
    _msd_device.parent.write   = rt_msd_write;
    _msd_device.parent.control = rt_msd_control;
#endif

    /* no private, no callback */
    //_msd_device.parent.user_data = RT_NULL;
    _msd_device.parent.rx_indicate = RT_NULL;
    _msd_device.parent.tx_complete = RT_NULL;
    _msd_device.offset = 0;
    _msd_device.parent.user_data = (void *)&_msd_device;
    rt_kprintf("%s %d _msd_device=0x%p\n", __func__, __LINE__, _msd_device);
    if (RT_EOK != rt_msd_init((rt_device_t)&_msd_device))
    {
        MSD_DEBUG("[SD] init failed\n");
        return RT_ERROR;
    }
    result = rt_device_register(&_msd_device.parent, sd_device_name,
                                RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE);
    return RT_EOK;
}

int rt_spi_msd_init(void)
{
#ifdef  SF32LB52X
    HAL_PIN_Set(PAD_PA24, SPI1_DIO, PIN_PULLDOWN, 1);       // SPI1 (Nor flash)
    HAL_PIN_Set(PAD_PA25, SPI1_DI,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA28, SPI1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, SPI1_CS,  PIN_NOPULL, 1);

#elif defined(SF32LB58X)
    HAL_PIN_Set(PAD_PA21, SPI1_DO, PIN_PULLDOWN, 1);       // SPI1 (Nor flash)
    HAL_PIN_Set(PAD_PA20, SPI1_DI,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA28, SPI1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, SPI1_CS,  PIN_NOPULL, 1);
#endif

    rt_device_t spi_bus = rt_device_find("sdcard");
    if (spi_bus == RT_NULL)
    {
        if (rt_hw_spi_device_attach("spi1", "sdcard") == RT_EOK)
        {
            rt_kprintf("[BUS]SPI1 probe sdcard...\n");
        }
        else
        {
            rt_kprintf("[BUS]SPI1 probe RT_ERROR\n");
            return RT_ERROR;
        }
    }
    rt_kprintf("[BUS]SPI1 sdcard succ ...\n");
    rt_device_t spi_dev = rt_device_find("sdcard");
    spi_dev_handle = (struct rt_spi_device *)spi_dev;
    if (rt_device_open(spi_dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
        rt_kprintf("[SD] OPEN SPI1 FAIL !\n");
    rt_kprintf("%s %d spi_dev=%p 0x%x %s\n", __func__, __LINE__, spi_dev, spi_dev->open_flag, spi_dev->parent.name);

    if (msd_init("sd0", "sdcard") != RT_EOK)
    {
        rt_kprintf("[SD]msd init failed,spi_dev=%p\n", spi_dev);
        if (spi_dev)
        {
            rt_device_close(spi_dev);
            rt_device_unregister(spi_dev);
        }
        return RT_ERROR;
    }
    else
    {
        rt_kprintf("[SD]msd init ok\n");
    }
    rt_device_t msd = rt_device_find("sd0");
    if (msd == NULL) rt_kprintf("find sd0 fail !\n");
    else rt_kprintf("find sd0 ok ! %p\n", msd);

    rt_uint32_t tf_id[16];
    rt_uint8_t i = 0;

    for (i = 0; i < 16; i++)
    {
        tf_id[i] = spi_recv_byte(spi_dev);

    }
    rt_kprintf("tf_id:%x\n", tf_id);


    return RT_EOK;
}
int main(void)
{
    rt_kprintf("tfcard start\n");
    rt_spi_msd_init();
    while (1)
    {
        //rt_thread_mdelay(5000);
        //rt_kprintf("__main loop__\r\n");
    }
    return RT_EOK;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

