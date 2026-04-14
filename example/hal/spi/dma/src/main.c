/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"
#include "bf0_hal.h"
#include "bf0_hal_dma.h"
#include "drv_io.h"
#include "dma_config.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"

#if defined(SF32LB52X)
    #define SPI1_RX_DMA_IRQ_HANDLER    DMAC1_CH6_IRQHandler
    #define SPI1_TX_DMA_IRQ_HANDLER    DMAC1_CH3_IRQHandler
#elif defined(SF32LB58X)
    #define SPI1_RX_DMA_IRQ_HANDLER    DMAC1_CH3_IRQHandler
    #define SPI1_TX_DMA_IRQ_HANDLER    DMAC1_CH4_IRQHandler
#endif

#define SPI_MODE                    0
#define SPI_BAUDRATE_HZ             20000000U
#define SPI_DMA_BUFFER_SIZE         256U
#define SPI_DMA_SAMPLE_BYTES        8U

static SPI_HandleTypeDef spi_Handle = {0};
static DMA_HandleTypeDef spi_dma_rx_handle = {0};
static DMA_HandleTypeDef spi_dma_tx_handle = {0};

static uint8_t spi_dma_tx_buf[SPI_DMA_BUFFER_SIZE];
static uint8_t spi_dma_rx_buf[SPI_DMA_BUFFER_SIZE];

static volatile uint32_t spi_dma_half_count = 0;
static volatile uint32_t spi_dma_full_count = 0;
static volatile uint32_t spi_dma_err_count = 0;
static volatile uint32_t spi_dma_last_err = HAL_SPI_ERROR_NONE;

void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&spi_Handle);
}

#if defined(BSP_USING_NO_OS) || !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void SPI1_RX_DMA_IRQ_HANDLER(void)
{
    HAL_DMA_IRQHandler(&spi_dma_rx_handle);
}

void SPI1_TX_DMA_IRQ_HANDLER(void)
{
    HAL_DMA_IRQHandler(&spi_dma_tx_handle);
}
#endif

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &spi_Handle)
    {
        spi_dma_half_count++;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &spi_Handle)
    {
        spi_dma_full_count++;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &spi_Handle)
    {
        spi_dma_err_count++;
        spi_dma_last_err = HAL_SPI_GetError(hspi);
    }
}

static uint32_t spi_get_apb_clock(SPI_HandleTypeDef *hspi)
{
#ifdef SF32LB55X
    return HAL_RCC_GetPCLKFreq(hspi->core, 1);
#else
    uint32_t spi_apb_clock = 48000000U; /* SPI1/2 fixed 48MHz */
#ifdef BSP_USING_SPI3
    if (SPI3 == hspi->Instance)
    {
        spi_apb_clock = 24000000U;
    }
#endif
#ifdef BSP_USING_SPI4
    if (SPI4 == hspi->Instance)
    {
        spi_apb_clock = 24000000U;
    }
#endif
    return spi_apb_clock;
#endif
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
#endif
}

static HAL_StatusTypeDef spi_dma_hw_init(void)
{
    uint32_t spi_apb_clock;

    spi_pinmux_init();
    HAL_RCC_EnableModule(RCC_MOD_SPI1);

    spi_Handle.Instance = SPI1;
    spi_Handle.Init.Direction = SPI_DIRECTION_2LINES;
    spi_Handle.Init.Mode = SPI_MODE_MASTER;
    spi_Handle.Init.DataSize = SPI_DATASIZE_8BIT;

#if   (SPI_MODE == 0)
    spi_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    spi_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
#elif (SPI_MODE == 1)
    spi_Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    spi_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
#elif (SPI_MODE == 2)
    spi_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    spi_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
#else
    spi_Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    spi_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
#endif

    spi_Handle.core = CORE_ID_HCPU;
    spi_apb_clock = spi_get_apb_clock(&spi_Handle);
    spi_Handle.Init.BaudRatePrescaler = (spi_apb_clock + SPI_BAUDRATE_HZ / 2U) / SPI_BAUDRATE_HZ;
    spi_Handle.Init.FrameFormat = SPI_FRAME_FORMAT_SPI;
    spi_Handle.Init.SFRMPol = SPI_SFRMPOL_HIGH;
    spi_Handle.State = HAL_SPI_STATE_RESET;

    if (HAL_SPI_Init(&spi_Handle) != HAL_OK)
    {
        return HAL_ERROR;
    }

    spi_dma_rx_handle.Instance = SPI1_RX_DMA_INSTANCE;
    spi_dma_rx_handle.Init.Request = SPI1_RX_DMA_REQUEST;
    spi_dma_rx_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    spi_dma_rx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    spi_dma_rx_handle.Init.MemInc = DMA_MINC_ENABLE;
    spi_dma_rx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    spi_dma_rx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    spi_dma_rx_handle.Init.Mode = DMA_CIRCULAR;
    spi_dma_rx_handle.Init.Priority = DMA_PRIORITY_HIGH;
#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    spi_dma_rx_handle.Init.IrqPrio = SPI1_RX_DMA_IRQ_PRIO;
#endif

    if (HAL_DMA_Init(&spi_dma_rx_handle) != HAL_OK)
    {
        return HAL_ERROR;
    }
    __HAL_LINKDMA(&spi_Handle, hdmarx, spi_dma_rx_handle);

    spi_dma_tx_handle.Instance = SPI1_TX_DMA_INSTANCE;
    spi_dma_tx_handle.Init.Request = SPI1_TX_DMA_REQUEST;
    spi_dma_tx_handle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    spi_dma_tx_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    spi_dma_tx_handle.Init.MemInc = DMA_MINC_ENABLE;
    spi_dma_tx_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    spi_dma_tx_handle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    spi_dma_tx_handle.Init.Mode = DMA_CIRCULAR;
    spi_dma_tx_handle.Init.Priority = DMA_PRIORITY_LOW;
#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    spi_dma_tx_handle.Init.IrqPrio = SPI1_TX_DMA_IRQ_PRIO;
#endif

    if (HAL_DMA_Init(&spi_dma_tx_handle) != HAL_OK)
    {
        return HAL_ERROR;
    }
    __HAL_LINKDMA(&spi_Handle, hdmatx, spi_dma_tx_handle);

    HAL_NVIC_SetPriority(SPI1_RX_DMA_IRQ, 0, 0);
    HAL_NVIC_SetPriority(SPI1_TX_DMA_IRQ, 0, 1);

    return HAL_OK;
}

static HAL_StatusTypeDef spi_dma_start_circular(void)
{
    uint32_t i;

    for (i = 0; i < SPI_DMA_BUFFER_SIZE; i++)
    {
        spi_dma_tx_buf[i] = (uint8_t)i;
    }
    memset(spi_dma_rx_buf, 0, sizeof(spi_dma_rx_buf));
    mpu_dcache_clean(spi_dma_tx_buf, sizeof(spi_dma_tx_buf));
    mpu_dcache_invalidate(spi_dma_rx_buf, sizeof(spi_dma_rx_buf));

    spi_dma_half_count = 0;
    spi_dma_full_count = 0;
    spi_dma_err_count = 0;
    spi_dma_last_err = HAL_SPI_ERROR_NONE;

    HAL_NVIC_DisableIRQ(SPI1_RX_DMA_IRQ);
    HAL_NVIC_DisableIRQ(SPI1_TX_DMA_IRQ);

    if (HAL_SPI_TransmitReceive_DMA(&spi_Handle,
            spi_dma_tx_buf,
            spi_dma_rx_buf,
            SPI_DMA_BUFFER_SIZE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_NVIC_EnableIRQ(SPI1_RX_DMA_IRQ);
    HAL_NVIC_EnableIRQ(SPI1_TX_DMA_IRQ);

    return HAL_OK;
}

static void spi_dump_samples(void)
{
    uint32_t i;

    mpu_dcache_invalidate(spi_dma_rx_buf, SPI_DMA_BUFFER_SIZE);

    rt_kprintf("rx[0..%d]:", SPI_DMA_SAMPLE_BYTES - 1U);
    for (i = 0; i < SPI_DMA_SAMPLE_BYTES; i++)
    {
        rt_kprintf(" %02x", spi_dma_rx_buf[i]);
    }
    rt_kprintf(", rx[mid..mid+%d]:", SPI_DMA_SAMPLE_BYTES - 1U);
    for (i = 0; i < SPI_DMA_SAMPLE_BYTES; i++)
    {
        rt_kprintf(" %02x", spi_dma_rx_buf[(SPI_DMA_BUFFER_SIZE / 2U) + i]);
    }
    rt_kprintf("\n");
}

int main(void)
{
    uint32_t last_half = 0;
    uint32_t last_full = 0;
    uint32_t last_err = 0;

    rt_kprintf("Start spi dma circular demo!\n");

    if (spi_dma_hw_init() != HAL_OK)
    {
        rt_kprintf("spi dma init failed!\n");
        while (1)
        {
            rt_thread_mdelay(1000);
        }
    }

    if (spi_dma_start_circular() != HAL_OK)
    {
        rt_kprintf("spi dma start failed, err=0x%x\n", HAL_SPI_GetError(&spi_Handle));
        while (1)
        {
            rt_thread_mdelay(1000);
        }
    }

    rt_kprintf("spi dma running, tx/rx circular started.\n");
    rt_kprintf("tip: short SPI1 MOSI(DIO/DO) to MISO(DI) for loopback verification.\n");

    while (1)
    {
        if ((last_half != spi_dma_half_count) || (last_full != spi_dma_full_count))
        {
            last_half = spi_dma_half_count;
            last_full = spi_dma_full_count;
            rt_kprintf("half=%lu, full=%lu, err=%lu\n",
                       (unsigned long)last_half,
                       (unsigned long)last_full,
                       (unsigned long)spi_dma_err_count);
            spi_dump_samples();
        }

        if (last_err != spi_dma_err_count)
        {
            last_err = spi_dma_err_count;
            rt_kprintf("spi dma error! cnt=%lu, spi_err=0x%lx\n",
                       (unsigned long)last_err,
                       (unsigned long)spi_dma_last_err);
        }

        rt_thread_mdelay(200);
    }
}
