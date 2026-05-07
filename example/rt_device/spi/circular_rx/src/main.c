/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "rtdevice.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "drv_spi.h"
#include <string.h>

#ifdef SF32LB56X
    #define SPI_BUS_NAME            "spi2"
#else
    #define SPI_BUS_NAME            "spi1"
#endif
#define SPI_DEVICE_NAME         "spi_circular"
#define SPI_BAUDRATE_HZ         20000000U
#define SPI_DMA_BUFFER_SIZE     256U
#define SPI_DMA_SAMPLE_BYTES    8U

/*
 * Demo mode selection:
 * - 0: Master TRX mode (loopback test, requires MOSI-MISO shorted)
 * - 1: Slave RX only mode (requires external Master to provide clock)
 * - 2: Slave TX only mode (requires external Master to provide clock)
 */
#ifndef SPI_CIRCULAR_DEMO_MODE
    #define SPI_CIRCULAR_DEMO_MODE  0
#endif

static struct rt_spi_device *g_spi_dev = RT_NULL;
static uint8_t g_spi_dma_tx_buf[SPI_DMA_BUFFER_SIZE];
static uint8_t g_spi_dma_rx_buf[SPI_DMA_BUFFER_SIZE];

static uint8_t *g_tx_start = g_spi_dma_tx_buf;
static uint8_t *g_tx_half  = g_spi_dma_tx_buf + (SPI_DMA_BUFFER_SIZE / 2U);

static volatile uint32_t g_rx_half_cnt = 0;
static volatile uint32_t g_rx_full_cnt = 0;
static volatile uint32_t g_tx_half_cnt = 0;
static volatile uint32_t g_tx_full_cnt = 0;

static rt_err_t spi_dma_circular_rx_ind(rt_device_t dev, rt_size_t offset)
{
    RT_ASSERT(dev != RT_NULL);

    if (offset == 0U)
    {
        g_rx_half_cnt++;
    }
    else if (offset == (SPI_DMA_BUFFER_SIZE / 2U))
    {
        g_rx_full_cnt++;
    }

    return RT_EOK;
}

static rt_err_t spi_dma_circular_tx_ind(rt_device_t dev, void *buffer)
{
    RT_ASSERT(dev != RT_NULL);

    if (buffer == g_tx_start)
    {
        g_tx_half_cnt++;
    }
    else if (buffer == g_tx_half)
    {
        g_tx_full_cnt++;
    }

    return RT_EOK;
}

static rt_err_t spi_prepare_circular_mode(void)
{
    struct rt_spi_dma_circular_config config;

    rt_memset(&config, 0, sizeof(config));
    config.enable = 1U;

#if (SPI_CIRCULAR_DEMO_MODE == 0)
    config.direction = RT_SPI_DMA_CIRCULAR_DIR_TXRX;
#elif (SPI_CIRCULAR_DEMO_MODE == 1)
    config.direction = RT_SPI_DMA_CIRCULAR_DIR_RX;
#else
    config.direction = RT_SPI_DMA_CIRCULAR_DIR_TX;
#endif

    rt_device_set_rx_indicate((rt_device_t)g_spi_dev, spi_dma_circular_rx_ind);
    rt_device_set_tx_complete((rt_device_t)g_spi_dev, spi_dma_circular_tx_ind);

    return rt_device_control((rt_device_t)g_spi_dev,
                             RT_SPI_CTRL_CONFIG_DMA_CIRCULAR,
                             &config);
}

static void spi_pinmux_init(void)
{
#ifdef SF32LB52X
    HAL_PIN_Set(PAD_PA24, SPI1_DIO, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA25, SPI1_DI,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA28, SPI1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, SPI1_CS,  PIN_NOPULL, 1);
#elif defined(SF32LB58X)
    HAL_PIN_Set(PAD_PA21, SPI1_DO,  PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA20, SPI1_DI,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA28, SPI1_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA29, SPI1_CS,  PIN_NOPULL, 1);
#elif defined(SF32LB56X)
    HAL_PIN_Set(PAD_PA64, SPI2_DO,  PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA69, SPI2_DI,  PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA73, SPI2_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA71, SPI2_CS,  PIN_NOPULL, 1);
#else
#error "Unsupported chip for circular_rx example"
#endif
}

static rt_err_t spi_prepare_rt_device(void)
{
    rt_err_t ret;
    struct rt_spi_configuration cfg;

    if (rt_device_find(SPI_DEVICE_NAME) == RT_NULL)
    {
        ret = rt_hw_spi_device_attach(SPI_BUS_NAME, SPI_DEVICE_NAME);
        if (ret != RT_EOK)
        {
            return ret;
        }
    }

    g_spi_dev = (struct rt_spi_device *)rt_device_find(SPI_DEVICE_NAME);
    if (g_spi_dev == RT_NULL)
    {
        return -RT_ERROR;
    }

    ret = rt_device_open((rt_device_t)g_spi_dev,
                         RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
    if (ret != RT_EOK)
    {
        return ret;
    }

    cfg.data_width = 8;
    cfg.max_hz = SPI_BAUDRATE_HZ;
    cfg.frameMode = RT_SPI_MOTO;

#if (SPI_CIRCULAR_DEMO_MODE == 0)
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_MASTER;
    rt_kprintf("Mode: Master TRX (loopback)\n");
#elif (SPI_CIRCULAR_DEMO_MODE == 1)
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_SLAVE;
    rt_kprintf("Mode: Slave RX only\n");
#elif (SPI_CIRCULAR_DEMO_MODE == 2)
    cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_SLAVE;
    rt_kprintf("Mode: Slave TX only\n");
#endif

    ret = rt_spi_configure(g_spi_dev, &cfg);
    if (ret != RT_EOK)
    {
        return ret;
    }

    ret = rt_spi_take_bus(g_spi_dev);
    if (ret != RT_EOK)
    {
        return ret;
    }

    ret = rt_spi_release_bus(g_spi_dev);
    if (ret != RT_EOK)
    {
        return ret;
    }

    ret = spi_prepare_circular_mode();
    if (ret != RT_EOK)
    {
        return ret;
    }

    return RT_EOK;
}

static void spi_prepare_tx_pattern(void)
{
    uint32_t i;

    for (i = 0; i < SPI_DMA_BUFFER_SIZE; i++)
    {
        g_spi_dma_tx_buf[i] = (uint8_t)i;
    }
}

static void spi_dump_rx_samples(void)
{
    uint32_t i;
    uint32_t mismatch = 0;

    mpu_dcache_invalidate(g_spi_dma_rx_buf, SPI_DMA_BUFFER_SIZE);

    rt_kprintf("rx[0..%d]:", SPI_DMA_SAMPLE_BYTES - 1U);
    for (i = 0; i < SPI_DMA_SAMPLE_BYTES; i++)
    {
        rt_kprintf(" %02x", g_spi_dma_rx_buf[i]);
        if (g_spi_dma_rx_buf[i] != g_spi_dma_tx_buf[i])
        {
            mismatch++;
        }
    }

    rt_kprintf(", rx[mid..mid+%d]:", SPI_DMA_SAMPLE_BYTES - 1U);
    for (i = 0; i < SPI_DMA_SAMPLE_BYTES; i++)
    {
        rt_kprintf(" %02x", g_spi_dma_rx_buf[(SPI_DMA_BUFFER_SIZE / 2U) + i]);
        if (g_spi_dma_rx_buf[(SPI_DMA_BUFFER_SIZE / 2U) + i] !=
                g_spi_dma_tx_buf[(SPI_DMA_BUFFER_SIZE / 2U) + i])
        {
            mismatch++;
        }
    }

    rt_kprintf(", mismatch=%lu\n", (unsigned long)mismatch);
}

static void spi_dump_tx_samples(void)
{
    uint32_t i;

    rt_kprintf("tx[0..%d]:", SPI_DMA_SAMPLE_BYTES - 1U);
    for (i = 0; i < SPI_DMA_SAMPLE_BYTES; i++)
    {
        rt_kprintf(" %02x", g_spi_dma_tx_buf[i]);
    }

    rt_kprintf(", tx[mid..mid+%d]:", SPI_DMA_SAMPLE_BYTES - 1U);
    for (i = 0; i < SPI_DMA_SAMPLE_BYTES; i++)
    {
        rt_kprintf(" %02x", g_spi_dma_tx_buf[(SPI_DMA_BUFFER_SIZE / 2U) + i]);
    }
    rt_kprintf("\n");
}

int main(void)
{
    int start_ret;
    uint32_t last_rx_half = 0, last_rx_full = 0;
    uint32_t last_tx_half = 0, last_tx_full = 0;

    rt_kprintf("Start SPI circular DMA demo!\n");

    spi_pinmux_init();

    if (spi_prepare_rt_device() != RT_EOK)
    {
        rt_kprintf("spi rt_device prepare failed!\n");
        while (1)
        {
            rt_thread_mdelay(1000);
        }
    }

    spi_prepare_tx_pattern();
    memset(g_spi_dma_rx_buf, 0, sizeof(g_spi_dma_rx_buf));
    g_rx_half_cnt = 0;
    g_rx_full_cnt = 0;
    g_tx_half_cnt = 0;
    g_tx_full_cnt = 0;

#if (SPI_CIRCULAR_DEMO_MODE == 0)
    start_ret = (rt_spi_transfer(g_spi_dev,
                                 g_spi_dma_tx_buf,
                                 g_spi_dma_rx_buf,
                                 SPI_DMA_BUFFER_SIZE) == SPI_DMA_BUFFER_SIZE) ? RT_EOK : -RT_ERROR;
    if (start_ret != RT_EOK)
    {
        rt_kprintf("rt_spi_transfer circular start failed, ret=%d\n", start_ret);
        while (1)
        {
            rt_thread_mdelay(1000);
        }
    }
    rt_kprintf("Tip: Short MOSI(DO) to MISO(DI) for loopback verification.\n");

#elif (SPI_CIRCULAR_DEMO_MODE == 1)
    start_ret = (rt_device_read((rt_device_t)g_spi_dev,
                                0,
                                g_spi_dma_rx_buf,
                                SPI_DMA_BUFFER_SIZE) == SPI_DMA_BUFFER_SIZE) ? RT_EOK : -RT_ERROR;
    if (start_ret != RT_EOK)
    {
        rt_kprintf("rt_device_read circular start failed, ret=%d\n", start_ret);
        while (1)
        {
            rt_thread_mdelay(1000);
        }
    }
    rt_kprintf("Tip: Connect external SPI Master to provide clock.\n");

#elif (SPI_CIRCULAR_DEMO_MODE == 2)
    start_ret = (rt_device_write((rt_device_t)g_spi_dev,
                                 0,
                                 g_spi_dma_tx_buf,
                                 SPI_DMA_BUFFER_SIZE) == SPI_DMA_BUFFER_SIZE) ? RT_EOK : -RT_ERROR;
    if (start_ret != RT_EOK)
    {
        rt_kprintf("rt_device_write circular start failed, ret=%d\n", start_ret);
        while (1)
        {
            rt_thread_mdelay(1000);
        }
    }
    rt_kprintf("Tip: Connect external SPI Master to provide clock.\n");
#endif

    rt_kprintf("Circular DMA started on %s\n", SPI_BUS_NAME);

    while (1)
    {
#if (SPI_CIRCULAR_DEMO_MODE == 0)
        if ((last_rx_half != g_rx_half_cnt) || (last_rx_full != g_rx_full_cnt))
        {
            last_rx_half = g_rx_half_cnt;
            last_rx_full = g_rx_full_cnt;
            rt_kprintf("RX: half=%lu full=%lu ",
                       (unsigned long)last_rx_half,
                       (unsigned long)last_rx_full);
            spi_dump_rx_samples();
        }
#elif (SPI_CIRCULAR_DEMO_MODE == 1)
        if ((last_rx_half != g_rx_half_cnt) || (last_rx_full != g_rx_full_cnt))
        {
            last_rx_half = g_rx_half_cnt;
            last_rx_full = g_rx_full_cnt;
            rt_kprintf("RX: half=%lu full=%lu\n",
                       (unsigned long)last_rx_half,
                       (unsigned long)last_rx_full);
        }
#elif (SPI_CIRCULAR_DEMO_MODE == 2)
        if ((last_tx_half != g_tx_half_cnt) || (last_tx_full != g_tx_full_cnt))
        {
            last_tx_half = g_tx_half_cnt;
            last_tx_full = g_tx_full_cnt;
            rt_kprintf("TX: half=%lu full=%lu ",
                       (unsigned long)last_tx_half,
                       (unsigned long)last_tx_full);
            spi_dump_tx_samples();
        }
#endif

        rt_thread_mdelay(200);
    }
}
