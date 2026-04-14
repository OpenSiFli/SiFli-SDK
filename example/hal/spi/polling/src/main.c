#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "string.h"
#include "rtthread.h"

static SPI_HandleTypeDef spi_Handle = {0};
#define SPI_MODE 0
#define SPI_LOOPBACK_LEN 16

static void spi_prepare_tx_pattern(uint8_t *tx_buf, uint32_t len, uint8_t seed)
{
    uint32_t i;

    for (i = 0; i < len; i++)
    {
        tx_buf[i] = (uint8_t)(seed + i);
    }
}

static uint32_t spi_count_mismatch(const uint8_t *tx_buf, const uint8_t *rx_buf, uint32_t len)
{
    uint32_t i;
    uint32_t mismatch = 0;

    for (i = 0; i < len; i++)
    {
        if (tx_buf[i] != rx_buf[i])
        {
            mismatch++;
        }
    }

    return mismatch;
}

static void spi_test(void)
{
    uint32_t baundRate = 20000000; //hz
    uint8_t tx_data[SPI_LOOPBACK_LEN] = {0};
    uint8_t rx_data[SPI_LOOPBACK_LEN] = {0};
    uint32_t round = 0;
    HAL_StatusTypeDef ret;

    //----------------------------------------------
    /* 1, pinmux set to spi1 mode */
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
    /* 2, open spi1 clock source  */
    HAL_RCC_EnableModule(RCC_MOD_SPI1);

    //----------------------------------------------
    // 2. spi init
    spi_Handle.Instance = SPI1;
    spi_Handle.Init.Direction = SPI_DIRECTION_2LINES;
    spi_Handle.Init.Mode = SPI_MODE_MASTER;
    spi_Handle.Init.DataSize = SPI_DATASIZE_8BIT;

#if   (SPI_MODE == 0)
    spi_Handle.Init.CLKPhase  = SPI_PHASE_1EDGE;
    spi_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
#elif (SPI_MODE == 1)
    spi_Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    spi_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
#elif (SPI_MODE == 2)
    spi_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    spi_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
#else //(SPI_MODE == 3)
    spi_Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    spi_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
#endif

    spi_Handle.core = CORE_ID_HCPU;

#ifdef SF32LB55X
    rt_uint32_t SPI_APB_CLOCK = HAL_RCC_GetPCLKFreq(spi_Handle.core, 1);
#else
    rt_uint32_t SPI_APB_CLOCK = 48000000; /* always 48MHz to SPI1&2 */
#ifdef BSP_USING_SPI3
    if (SPI3 == spi_Handle.Instance)
    {
        SPI_APB_CLOCK = 24000000;  /* always 24MHz to SPI3*/
    }
#endif /* BSP_USING_SPI3 */
#ifdef BSP_USING_SPI4
    if (SPI4 == spi_Handle.Instance)
    {
        SPI_APB_CLOCK = 24000000;  /* always 24MHz to SPI4 */
    }
#endif /* BSP_USING_SPI4 */
#endif /* SF32LB55X */
    spi_Handle.Init.BaudRatePrescaler = (SPI_APB_CLOCK + baundRate / 2) / baundRate;

    spi_Handle.Init.FrameFormat = SPI_FRAME_FORMAT_SPI;
    spi_Handle.Init.SFRMPol = SPI_SFRMPOL_HIGH;
    spi_Handle.State = HAL_SPI_STATE_RESET;
    if (HAL_SPI_Init(&spi_Handle) != HAL_OK)
    {
        rt_kprintf("spi init err!");
        return;
    }

    //----------------------------------------------
    // 3. polling loopback test
    rt_kprintf("tip: short SPI1 MOSI(DIO/DO) to MISO(DI) for loopback verification.\n");

    while (1)
    {
        uint32_t mismatch;
        uint8_t i;

        spi_prepare_tx_pattern(tx_data, SPI_LOOPBACK_LEN, (uint8_t)round);
        memset(rx_data, 0, sizeof(rx_data));

        ret = HAL_SPI_TransmitReceive(&spi_Handle, tx_data, rx_data, SPI_LOOPBACK_LEN, 1000);
        mismatch = spi_count_mismatch(tx_data, rx_data, SPI_LOOPBACK_LEN);

        rt_kprintf("round=%lu ret=%d mismatch=%lu tx:", (unsigned long)round, ret, (unsigned long)mismatch);
        for (i = 0; i < SPI_LOOPBACK_LEN; i++)
        {
            rt_kprintf(" %02x", tx_data[i]);
        }

        rt_kprintf(" rx:");
        for (i = 0; i < SPI_LOOPBACK_LEN; i++)
        {
            rt_kprintf(" %02x", rx_data[i]);
        }
        rt_kprintf("\n");

        round++;
        rt_thread_mdelay(500);
    }
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* Output a message on console using printf function */
    rt_kprintf("Start spi polling loopback demo!\n");
    spi_test();

    return 0;
}
