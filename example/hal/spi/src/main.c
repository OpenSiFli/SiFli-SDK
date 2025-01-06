#include "rtconfig.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "rtthread.h"


static SPI_HandleTypeDef spi_Handle = {0};
#define     SPI_MODE    0
static void gpio_set(uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_PIN_Set(PAD_PB00 + pin, GPIO_B0 + pin, PIN_PULLDOWN, 0);
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(hwp_gpio2, &GPIO_InitStruct);

    HAL_GPIO_WritePin(hwp_gpio2, pin, 1);
}

void spi_config(void)
{

}
static void spi_test(void)
{
    uint32_t baundRate = 20000000; //hz
    uint8_t cmd[16] = {0};   //SPI_DATASIZE_8BIT
    uint8_t read_data[16] = {0};
    uint8_t pid = 0;
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

    //gpio_set(30);

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
    // 3.1. spi sync rtx
    cmd[0] = 0xff;
    ret = HAL_SPI_Transmit(&spi_Handle, (uint8_t *)cmd, 1, 1000);
    cmd[0] = 0x9f;
    __HAL_SPI_TAKE_CS(&spi_Handle);
    ret = HAL_SPI_TransmitReceive(&spi_Handle, (uint8_t *)&cmd, (uint8_t *)&read_data, 16, 1000);
    __HAL_SPI_RELEASE_CS(&spi_Handle);
    HAL_Delay_us(5);
    __HAL_SPI_TAKE_CS(&spi_Handle);
    ret = HAL_SPI_Transmit(&spi_Handle, (uint8_t *)cmd, 1, 1000);
    ret = HAL_SPI_Receive(&spi_Handle, (uint8_t *)read_data, 16, 1000);
    __HAL_SPI_RELEASE_CS(&spi_Handle);

    rt_kprintf("ret:%d,spi read:", ret);
    for (uint8_t i = 0; i < 16; i++)
    {
        rt_kprintf("0x%x,", read_data[i]);
    }
    rt_kprintf("\n");

}


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    HAL_StatusTypeDef  ret = HAL_OK;

    /* Output a message on console using printf function */
    rt_kprintf("Start spi demo!\n");
    spi_test();
    rt_kprintf("spi demo end!\n");
    while (1);
    return 0;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

