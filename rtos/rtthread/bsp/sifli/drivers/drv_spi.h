/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __DRV_SPI_H_
#define __DRV_SPI_H_

#include <rtthread.h>
#include "rtdevice.h"
#include <rthw.h>
#include <drv_common.h>

/*
 * Weak hooks that let a driver reusing the SPI peripheral (e.g. the I2S slave
 * receiver in drv_spi_i2s_slave_rx.c) take over the HAL SPI Rx complete /
 * half-complete / error callbacks for its own SPI handle.
 *
 * drv_spi.c owns the HAL callbacks and dispatches to these hooks before
 * touching any SPI bus state, so the driver must not touch the handle of a
 * different owner. The default implementations in drv_spi.c do nothing, which
 * keeps behavior unchanged when the hook provider is not built.
 */
rt_bool_t bsp_i2s_slave_rx_owns(SPI_HandleTypeDef *hspi);
void bsp_i2s_slave_rx_cplt(SPI_HandleTypeDef *hspi);
void bsp_i2s_slave_rx_half_cplt(SPI_HandleTypeDef *hspi);
void bsp_i2s_slave_rx_err(SPI_HandleTypeDef *hspi);

/*
 * drv_spi.c compiles the HAL SPI callback implementations (and their dispatch
 * to the hooks above) only when the SPI bus driver is built and a SPI bus is
 * enabled with DMA-allocation / camera / circular support. Keep in sync with
 * the callback guards in drv_spi.c. drv_spi_i2s_slave_rx.c uses this to decide
 * whether it can rely on the dispatch or must provide the HAL callbacks itself.
 */
#if defined(RT_USING_SPI) && defined(BSP_USING_SPI) && \
    (defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC) || defined(BSP_USING_SPI_CAMERA) || defined(BSP_USING_SPI_DMA_CIRCULAR))
    #define DRV_SPI_PROVIDES_HAL_CALLBACKS 1
#endif

//rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name, GPIO_TypeDef* cs_gpiox, uint16_t cs_gpio_pin);
rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name);

struct sifli_hw_spi_cs
{
    //GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
};

struct sifli_spi_config
{
    SPI_TypeDef *Instance;
    char *bus_name;
    IRQn_Type irq_type;
    uint8_t core;
    struct dma_config *dma_rx, *dma_tx;
};

struct sifli_spi_device
{
    rt_uint32_t pin;
    char *bus_name;
    char *device_name;
};

#define SPI_USING_RX_DMA_FLAG   (1<<0)
#define SPI_USING_TX_DMA_FLAG   (1<<1)

/*
 * SPI private control commands.
 * RT_SPI_CTRL_CONFIG_DMA_CIRCULAR / RT_SPI_CTRL_STOP_DMA_CIRCULAR are
 * intended to be used with rt_device_control() + read/write/transfer.
 */
#if defined(BSP_USING_SPI_DMA_CIRCULAR)

#define RT_SPI_CTRL_CONFIG_DMA_CIRCULAR  (0x82)  // Control command to configure DMA circular mode
#define RT_SPI_CTRL_STOP_DMA_CIRCULAR    (0x83)  // Control command to stop DMA circular mode

#define RT_SPI_DMA_CIRCULAR_DIR_NONE     (0U)    // DMA circular mode direction: none
#define RT_SPI_DMA_CIRCULAR_DIR_RX       (1U << 0)  // DMA circular mode direction: receive only
#define RT_SPI_DMA_CIRCULAR_DIR_TX       (1U << 1)  // DMA circular mode direction: transmit only
#define RT_SPI_DMA_CIRCULAR_DIR_TXRX     (RT_SPI_DMA_CIRCULAR_DIR_RX | RT_SPI_DMA_CIRCULAR_DIR_TX)  // DMA circular mode direction: both TX and RX

struct rt_spi_dma_circular_config
{
    rt_uint8_t enable;      // Enable flag for DMA circular mode
    rt_uint8_t direction;   // DMA circular mode direction (RX/TX/both)
    rt_uint16_t reserved;   // Reserved field for alignment
};

#endif

/* sifli spi dirver class */
struct sifli_spi
{
    SPI_HandleTypeDef handle;
    struct sifli_spi_config *config;
    struct rt_spi_configuration *cfg;

    struct
    {
        DMA_HandleTypeDef handle_rx;
        DMA_HandleTypeDef handle_tx;
    } dma;

    rt_uint8_t spi_dma_flag;        // DMA flags indicating DMA usage
#if defined(BSP_USING_SPI_DMA_CIRCULAR)
    struct
    {
        struct rt_spi_device *device;   // Associated SPI device pointer
        rt_uint8_t enabled;             // Circular mode enabled flag
        rt_uint8_t direction;           // Circular mode direction (RX/TX/both)
        rt_uint8_t active;              // Circular mode active state flag
        rt_uint8_t cs_held;             // CS signal held flag
#ifdef RT_USING_PM
        rt_uint8_t pm_active;           // Power management active state flag
#endif
        rt_size_t size;                 // Circular buffer size
        rt_uint8_t *rx_start;           // Receive buffer start address
        rt_uint8_t *rx_half;            // Receive buffer half address (for half-complete callback)
        rt_uint8_t *tx_start;           // Transmit buffer start address
        rt_uint8_t *tx_half;            // Transmit buffer half address (for half-complete callback)
    } circular;
#endif
#if defined(BSP_USING_SPI_CAMERA)
    struct
    {
        rt_uint8_t active;
        rt_size_t size;
        rt_uint8_t *rx_start;
        rt_uint8_t *rx_half;
    } camera;
#endif
    struct rt_spi_bus spi_bus;
    rt_sem_t spi_sema;
};

#endif /*__DRV_SPI_H_ */
